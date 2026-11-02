import KSLCore
import KSLPhysics

/// Search intent only. Propagated heading/winding must establish realization;
/// attaching a topology name does not establish an achieved spiral or join.
public enum TAEMCommandTopology: String, Codable, Sendable {
  case direct, left, right, prolongedLeft, prolongedRight
  case windingLeft, windingRight, returnLeft, returnRight
  case leftRight, rightLeft, incumbent, protection
}
public struct TAEMCommandCandidate: Sendable {
  public let id: String
  public let topology: TAEMCommandTopology
  public let segments: [TAEMCommandSegment]
  public init(id: String, topology: TAEMCommandTopology, segments: [TAEMCommandSegment]) {
    self.id = id; self.topology = topology; self.segments = segments
  }
}
public enum TAEMCommandSearchEnd: String, Codable, Sendable {
  case familyExhausted, candidateBudget, cancelled
}
public struct TAEMCommandTrial: Sendable {
  public let candidate: TAEMCommandCandidate
  public let prefix: TAEMSchedulePrefix
  public let continuation: TerminalPlan?
  public let outcome: TerminalClosedLoopCase?
  public let incumbentTrial: Int?
  public let relation: TerminalRecoveryRelation?
}
public struct TAEMCommandSearchResult: Sendable {
  public let end: TAEMCommandSearchEnd
  public let trials: [TAEMCommandTrial]
  /// Diagnostic incumbent only. Admission and control ownership are separate.
  public let incumbentTrial: Int?
  public let suppliedCandidates: Int
}

/// Bounded deterministic shooting driver. Candidate schedules are supplied by
/// the topology/refinement layer; each trial branches the same actual history.
/// The continuation builder sees the reached state/history, never a geometric
/// tangent substituted for motion. It must not call this driver recursively.
public final class TAEMCommandSearch {
  let configuration: MissionConfiguration
  private let schedule: TAEMCommandScheduleEvaluator
  private let terminal: TerminalTrajectoryEvaluator
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    schedule = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    terminal = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func search(_ candidates: [TAEMCommandCandidate], from seed: NativeCommandSeed,
                     maximumEvaluations: Int,
                     cancelled: () -> Bool = { false },
                     continuation: (TAEMCommandCandidate, TAEMSchedulePrefix) -> TerminalPlan?) throws -> TAEMCommandSearchResult {
    // Duplicate labels would make a recorded challenger ambiguous. This is a
    // search-contract check, not a flight constraint or a capability adjustment.
    guard maximumEvaluations >= 0, Set(candidates.map(\.id)).count == candidates.count,
      candidates.allSatisfy({ !$0.id.isEmpty }) else {
      throw FlightError.invalidConfiguration("Command search requires unique candidate IDs and a nonnegative evaluation budget")
    }
    var trials: [TAEMCommandTrial] = [], incumbent: Int?
    func result(_ end: TAEMCommandSearchEnd) -> TAEMCommandSearchResult {
      .init(end: end, trials: trials, incumbentTrial: incumbent, suppliedCandidates: candidates.count)
    }
    for candidate in candidates {
      if cancelled() || Task.isCancelled { return result(.cancelled) }
      if trials.count >= maximumEvaluations { return result(.candidateBudget) }
      let trial = try evaluate(candidate, from: seed, incumbent: incumbent.map { trials[$0].outcome! },
        incumbentTrial: incumbent, cancelled: cancelled, continuation: continuation)
      trials.append(trial)
      if trial.outcome != nil && (incumbent == nil || trial.relation == .challengerPreferred) {
        incumbent = trials.count-1
      }
      if trial.prefix.end == .cancelled || trial.outcome?.evaluation?.end == .cancelled || cancelled() || Task.isCancelled { return result(.cancelled) }
    }
    return result(.familyExhausted)
  }
  func evaluate(_ candidate: TAEMCommandCandidate, from seed: NativeCommandSeed,
                incumbent: TerminalClosedLoopCase?, incumbentTrial: Int?,
                cancelled: () -> Bool,
                continuation: (TAEMCommandCandidate, TAEMSchedulePrefix) -> TerminalPlan?) throws -> TAEMCommandTrial {
    let prefix = try schedule.evaluate(candidate.segments, from: seed,
      cancelled: { cancelled() || Task.isCancelled })
    // Missing model support cannot be repaired by drawing a terminal route.
    let plan = prefix.end == .scheduleExhausted && prefix.momentSupport.available
      && !cancelled() && !Task.isCancelled ? continuation(candidate, prefix) : nil
    let outcome = plan.map {
      terminal.evaluate(plan: $0, from: prefix.seed.state, controlSnapshot: nil,
        previousTarget: nil, lift: 1, drag: 1, commandPrefix: prefix,
        cancelled: { cancelled() || Task.isCancelled })
    }
    let relation = outcome.flatMap { challenger in incumbent.map { TerminalRecoveryOrder.compare(challenger, to: $0) } }
    return .init(candidate: candidate, prefix: prefix, continuation: plan, outcome: outcome,
      incumbentTrial: incumbentTrial, relation: relation)
  }

}
