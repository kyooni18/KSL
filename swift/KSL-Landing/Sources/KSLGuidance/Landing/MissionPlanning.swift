import Foundation
import KSLCore
import KSLControl
import KSLPhysics

public struct MissionPlanningRequest: Sendable {
  public let time: Double
  public let entry: EntryPlanningRequest?
  public let terminalState: VehicleState?
  public let controls: FlightControlSnapshot?
  public let previousTarget: GuidanceTarget?
  /// Full reference history, including bank/alpha rates; preferred over the
  /// legacy previousTarget when present. Value copied into both planning lanes.
  public var targetLimiter: GuidanceTargetLimiter? = nil
  public var approach: ApproachPlanningRequest?
  public var environment = EnvironmentCalibration()
  public var hac: HACFitContext? = nil
  /// Dynamics revision observed before taking the planning snapshot.
  public var dynamicsRevision: UInt64? = nil

  /// Preserve one immutable flight snapshot, but hand each independent worker
  /// only the work it owns. This also preserves the historical plan API.
  public func entryOnly() -> Self {
    .init(time: time, entry: entry, terminalState: nil, controls: controls,
      previousTarget: previousTarget, targetLimiter: targetLimiter, approach: nil, environment: environment,
      hac: hac, dynamicsRevision: dynamicsRevision)
  }
  public func terminalOnly() -> Self {
    .init(time: time, entry: nil, terminalState: terminalState, controls: controls,
      previousTarget: previousTarget, targetLimiter: targetLimiter, approach: approach, environment: environment,
      hac: hac, dynamicsRevision: dynamicsRevision)
  }
}

public struct MissionPlanningProposal: Codable, Sendable {
  public let time: Double
  public let entry: EntryPlanningProposal?
  public let terminal: PlanningResult?
  public var approach: ApproachPlanningProposal?
  /// Nil in historical records, which must retain their conservative time rule.
  public var dynamicsRevision: UInt64? = nil
}

public struct RecordedPlanningProposal: Sendable {
  public let proposal: MissionPlanningProposal
  public let encoded: Data
}

/// Owns separate planner instances. Only immutable value snapshots cross from
/// urgent flight control into this actor; at most one request is outstanding.
public actor MissionPlanningWorker {
  private let entry: EntryGuidanceEngine
  private let terminal: HACPlanningEngine
  private let approach: TerminalEnergyGuide
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    approach = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    entry = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    terminal = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func plan(_ request: MissionPlanningRequest) -> MissionPlanningProposal {
    if let final = request.approach {
      return .init(time: request.time, entry: nil, terminal: nil, approach: approach.plan(final),
        dynamicsRevision: request.dynamicsRevision)
    }
    let terminalResult = request.terminalState.map {
      let model = request.hac.map { aerodynamics.scaled(lift: $0.liftScale, drag: $0.dragScale) } ?? aerodynamics
      let planner = HACPlanningEngine(configuration: configuration,
        atmosphere: atmosphere.corrected(request.environment), aerodynamics: model)
      let preview = planner.plan(from: $0, allowDirect: request.hac?.onSpiral != true, preview: true,
        controlSnapshot: request.controls, previousTarget: request.previousTarget, targetLimiter: request.targetLimiter, context: request.hac)
      // The preexisting worker only ever returned previews, while MM305's
      // actual ownership contract requires a successful closed-loop stop.
      // Qualify only an inlet that has passed the cheap physical/geometry
      // gate; do not spend three complete landings on rejected advisories.
      if let candidate = preview.plan, !Task.isCancelled,
         MM305Admission.shouldQualifyPreview($0, plan: candidate, configuration: configuration) {
        return planner.plan(from: $0, allowDirect: request.hac?.onSpiral != true, preview: false,
          controlSnapshot: request.controls, previousTarget: request.previousTarget, targetLimiter: request.targetLimiter, context: request.hac)
      }
      return preview
    }
    let entryResult = !Task.isCancelled && request.terminalState == nil
      ? request.entry.map { entry.plan($0) } : nil
    return .init(time: request.time, entry: entryResult, terminal: terminalResult,
      dynamicsRevision: request.dynamicsRevision)
  }
  /// Serialization stays on the planning actor, outside the urgent loop.
  public func recordedPlan(_ request: MissionPlanningRequest) throws -> RecordedPlanningProposal {
    try Task.checkCancellation()
    let proposal = plan(request)
    try Task.checkCancellation()
    return .init(proposal: proposal, encoded: try JSONEncoder().encode(proposal))
  }
  public func planTerminal(from state: VehicleState) -> PlanningResult {
    terminal.plan(from: state, allowDirect: true, preview: true)
  }
}
