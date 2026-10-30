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
  public var approach: ApproachPlanningRequest?
  public var environment = EnvironmentCalibration()
}

public struct MissionPlanningProposal: Codable, Sendable {
  public let time: Double
  public let entry: EntryPlanningProposal?
  public let terminal: PlanningResult?
  public var approach: ApproachPlanningProposal?
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
      return .init(time: request.time, entry: nil, terminal: nil, approach: approach.plan(final))
    }
    let terminalResult = request.terminalState.map {
      let planner = request.environment.observedAt == nil ? terminal : HACPlanningEngine(
        configuration: configuration, atmosphere: atmosphere.corrected(request.environment), aerodynamics: aerodynamics)
      return planner.plan(from: $0, allowDirect: false,
        controlSnapshot: request.controls, previousTarget: request.previousTarget)
    }
    let entryResult = !Task.isCancelled && request.terminalState == nil
      ? request.entry.map { entry.plan($0) } : nil
    return .init(time: request.time, entry: entryResult, terminal: terminalResult)
  }
  /// Serialization stays on the planning actor, outside the urgent loop.
  public func recordedPlan(_ request: MissionPlanningRequest) throws -> RecordedPlanningProposal {
    let proposal = plan(request)
    return .init(proposal: proposal, encoded: try JSONEncoder().encode(proposal))
  }
  public func planTerminal(from state: VehicleState) -> PlanningResult {
    terminal.plan(from: state, allowDirect: false)
  }
}
