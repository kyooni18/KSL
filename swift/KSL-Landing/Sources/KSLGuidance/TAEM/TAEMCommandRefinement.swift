import KSLCore
import KSLPhysics

/// Explicit numerical search spacing, not vehicle capability or a flight gain.
/// There are no experimental default steps or hidden minimum maneuver times.
public struct TAEMCommandRefinementSteps: Sendable {
  public let alphaRadians: Double, bankRadians: Double, durationSeconds: Double
  public init(alphaRadians: Double, bankRadians: Double, durationSeconds: Double) {
    self.alphaRadians = alphaRadians; self.bankRadians = bankRadians; self.durationSeconds = durationSeconds
  }
}

public enum TAEMCommandRefinementEnd: String, Codable, Sendable {
  case resolutionReached, candidateBudget, cancelled, modelUnavailable, unassessed
}
public struct TAEMCommandRefinementResult: Sendable {
  public let end: TAEMCommandRefinementEnd
  public let trials: [TAEMCommandTrial]
  public let incumbentTrial: Int?
  public let finalSteps: TAEMCommandRefinementSteps
  public let polls: Int
}

extension TAEMCommandSearch {
  /// One deterministic coordinate poll around a supplied command schedule.
  /// All segment +/- alpha, bank and duration neighbors use the same original
  /// seed, including full limiter/FCS/plant history and terminal accounting.
  /// Further polls/step adaptation belong to the caller's explicit budget.
  public func refineOnce(_ center: TAEMCommandCandidate, from seed: NativeCommandSeed,
                         steps: TAEMCommandRefinementSteps, maximumEvaluations: Int,
                         cancelled: () -> Bool = { false },
                         continuation: (TAEMCommandCandidate, TAEMSchedulePrefix) -> TerminalPlan?) throws -> TAEMCommandSearchResult {
    guard maximumEvaluations >= 0 else { throw FlightError.invalidConfiguration("Negative refinement budget") }
    return try search(refinementCandidates(center, steps: steps), from: seed,
      maximumEvaluations: maximumEvaluations, cancelled: cancelled, continuation: continuation)
  }

  /// Adaptive coordinate polling with a single total propagation budget.
  /// Resolution describes parameter-space coverage, never flight readiness.
  public func refine(_ initial: TAEMCommandCandidate, from seed: NativeCommandSeed,
                     initialSteps: TAEMCommandRefinementSteps,
                     resolution: TAEMCommandRefinementSteps, maximumEvaluations: Int,
                     cancelled: () -> Bool = { false },
                     continuation: (TAEMCommandCandidate, TAEMSchedulePrefix) -> TerminalPlan?) throws -> TAEMCommandRefinementResult {
    let start = [initialSteps.alphaRadians, initialSteps.bankRadians, initialSteps.durationSeconds]
    let minimum = [resolution.alphaRadians, resolution.bankRadians, resolution.durationSeconds]
    guard maximumEvaluations >= 0, zip(start, minimum).allSatisfy({
      $0.isFinite && $1.isFinite && $0 >= $1 && $1 > 0
    }) else { throw FlightError.invalidConfiguration("Invalid refinement resolution or budget") }
    var center = initial, spacing = initialSteps, trials: [TAEMCommandTrial] = []
    var incumbent: Int?, visited = Set<[UInt64]>(), poll = 0, allComparable = true
    func result(_ end: TAEMCommandRefinementEnd) -> TAEMCommandRefinementResult {
      .init(end: end, trials: trials, incumbentTrial: incumbent, finalSteps: spacing, polls: poll)
    }
    while true {
      if cancelled() || Task.isCancelled { return result(.cancelled) }
      let candidates = try refinementCandidates(center, steps: spacing)
      let previousIncumbent = incumbent
      poll += 1
      for candidate in candidates {
        // In this fixed topology/segment family only these three coordinates
        // change; phase and device requests remain inherited from the center.
        let key = candidate.segments.flatMap {
          [$0.duration.bitPattern, $0.command.target.angleOfAttack.bitPattern, $0.command.target.bank.bitPattern]
        }
        if visited.contains(key) { continue }
        if cancelled() || Task.isCancelled { return result(.cancelled) }
        if trials.count >= maximumEvaluations { return result(.candidateBudget) }
        visited.insert(key)
        let named = TAEMCommandCandidate(id: "\(initial.id)/refine/\(trials.count)",
          topology: candidate.topology, segments: candidate.segments)
        let trial = try evaluate(named, from: seed, incumbent: incumbent.flatMap { trials[$0].outcome },
          incumbentTrial: incumbent, cancelled: cancelled, continuation: continuation)
        trials.append(trial)
        if trial.outcome.map({ TerminalRecoveryOrder.compare($0, to: $0) != .equivalent }) ?? true
          || trial.relation.map({ [.unassessed, .contextMismatch, .incomparableOutcomes].contains($0) }) == true {
          allComparable = false
        }
        if trial.outcome != nil && (incumbent == nil || trial.relation == .challengerPreferred) {
          incumbent = trials.count-1
        }
        if trial.prefix.end == .cancelled || trial.outcome?.evaluation?.end == .cancelled
          || cancelled() || Task.isCancelled { return result(.cancelled) }
        // No candidate can advance from this shared seed without the missing
        // originating model evidence. Stop, retaining the diagnostic trial.
        if trials.count == 1 && trial.prefix.end == .unsupportedDynamics && trial.prefix.steps.isEmpty {
          return result(.modelUnavailable)
        }
      }
      if let incumbent { center = trials[incumbent].candidate }
      if incumbent != previousIncumbent { continue }
      let current = [spacing.alphaRadians, spacing.bankRadians, spacing.durationSeconds]
      if current == minimum { return result(allComparable ? .resolutionReached : .unassessed) }
      // Binary numerical mesh refinement, not a vehicle capability factor.
      spacing = .init(alphaRadians: max(minimum[0], current[0]/2),
        bankRadians: max(minimum[1], current[1]/2), durationSeconds: max(minimum[2], current[2]/2))
    }
  }

  private func refinementCandidates(_ center: TAEMCommandCandidate,
                                    steps: TAEMCommandRefinementSteps) throws -> [TAEMCommandCandidate] {
    let values = [steps.alphaRadians, steps.bankRadians, steps.durationSeconds]
    guard values.allSatisfy({ $0.isFinite && $0 > 0 }), !center.id.isEmpty else {
      throw FlightError.invalidConfiguration("Command refinement requires explicit finite positive search steps")
    }
    let c = configuration
    // Only search inside real configured attitude bounds and declared time
    // coverage. Clipping a request is not allowed to produce duplicate polls.
    guard center.segments.allSatisfy({ segment in
      let t = segment.command.target
      return segment.duration.isFinite && segment.duration > 0
        && t.angleOfAttack >= c.vehicle.minimumAngleOfAttack && t.angleOfAttack <= c.vehicle.maximumAngleOfAttack
        && abs(t.bank) <= c.vehicle.maximumBank
    }), center.segments.reduce(0, { $0+$1.duration }) <= c.guidance.maximumPredictionDuration else {
      throw FlightError.invalidConfiguration("Refinement center lies outside configured command/search coverage")
    }
    // Each candidate differs in precisely one search coordinate. Other
    // commands/device requests/phase metadata are inherited without reset.
    var candidates = [center]
    for index in center.segments.indices {
      for coordinate in 0..<3 {
        for sign in [-1.0, 1.0] {
          var segments = center.segments
          let old = segments[index]
          var command = replacing(old.command)
          var duration = old.duration
          switch coordinate {
          case 0:
            command = replacing(command, alpha: clamp(old.command.target.angleOfAttack+sign*steps.alphaRadians,
              c.vehicle.minimumAngleOfAttack, c.vehicle.maximumAngleOfAttack))
          case 1:
            command = replacing(command, bank: clamp(old.command.target.bank+sign*steps.bankRadians,
              -c.vehicle.maximumBank, c.vehicle.maximumBank))
          default: duration += sign*steps.durationSeconds
          }
          guard duration.isFinite, duration > 0,
            command.target != old.command.target || duration != old.duration else { continue }
          segments[index] = .init(duration: duration, command: command, untilCourseChange: old.untilCourseChange)
          guard segments.reduce(0, { $0+$1.duration }) <= c.guidance.maximumPredictionDuration else { continue }
          let id = "\(center.id)/poll/\(index)/\(coordinate)/\(sign < 0 ? "minus" : "plus")"
          candidates.append(.init(id: id, topology: center.topology, segments: segments))
        }
      }
    }
    return candidates
  }

  private func replacing(_ solution: GuidanceSolution, alpha: Double? = nil, bank: Double? = nil) -> GuidanceSolution {
    var target = solution.target
    if let alpha { target.angleOfAttack = alpha }
    if let bank { target.bank = bank }
    // Old tracking/admission/entry diagnostics and targetHistory describe
    // another evaluated request, never this new command trial.
    return GuidanceSolution(phase: solution.phase, target: target,
      diagnostic: "command refinement trial")
  }
}
