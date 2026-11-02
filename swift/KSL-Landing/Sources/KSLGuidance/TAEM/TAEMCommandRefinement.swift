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

extension TAEMCommandSearch {
  /// One deterministic coordinate poll around a supplied command schedule.
  /// All segment +/- alpha, bank and duration neighbors use the same original
  /// seed, including full limiter/FCS/plant history and terminal accounting.
  /// Further polls/step adaptation belong to the caller's explicit budget.
  public func refineOnce(_ center: TAEMCommandCandidate, from seed: NativeCommandSeed,
                         steps: TAEMCommandRefinementSteps, maximumEvaluations: Int,
                         cancelled: () -> Bool = { false },
                         continuation: (TAEMCommandCandidate, TAEMSchedulePrefix) -> TerminalPlan?) throws -> TAEMCommandSearchResult {
    let values = [steps.alphaRadians, steps.bankRadians, steps.durationSeconds]
    guard values.allSatisfy({ $0.isFinite && $0 > 0 }), maximumEvaluations >= 0,
      !center.id.isEmpty else {
      throw FlightError.invalidConfiguration("Command refinement requires explicit finite positive search steps and nonnegative evaluation budget")
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
          segments[index] = .init(duration: duration, command: command)
          guard segments.reduce(0, { $0+$1.duration }) <= c.guidance.maximumPredictionDuration else { continue }
          let id = "\(center.id)/poll/\(index)/\(coordinate)/\(sign < 0 ? "minus" : "plus")"
          candidates.append(.init(id: id, topology: center.topology, segments: segments))
        }
      }
    }
    return try search(candidates, from: seed, maximumEvaluations: maximumEvaluations,
      cancelled: cancelled, continuation: continuation)
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
