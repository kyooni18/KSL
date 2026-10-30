import Foundation
import KSLCore
import KSLControl
import KSLGuidance
import KSLPhysics

/// Replays the exact externally accepted proposals at their recorded control
/// boundaries. Planner completion timing is evidence, never recomputed as if
/// an expensive live plan had completed instantly.
public final class FlightRecordReplayer {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  private var computer: FlightComputer?
  private var orbitalPlan: DeorbitPlan?
  private var burn: DeorbitBurnController?
  private var coast = OrbitalAttitudeController()
  private var previousOrbitalTime: Double?
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
  }
  public func step(_ recorded: FlightRecord) throws -> FlightRecord {
    if recorded.orbitalCommand != nil { return try replayOrbital(recorded) }
    if computer == nil || (recorded.sequence == 1 && recorded.initialPhase != nil) {
      let seed = try recorded.initialTerminalPlan.map { try JSONDecoder().decode(TerminalPlan.self, from: $0) }
      guard recorded.planningMode == nil || ["external-proposals-v1", "inline-deterministic-v1"].contains(recorded.planningMode!) else {
        throw FlightError.protocolViolation("Unsupported replay planning mode")
      }
      computer = try .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
        externalPlanning: recorded.planningMode == "external-proposals-v1",
        initialPhase: recorded.initialPhase ?? .entry, initialPlan: seed)
    }
    if let bytes = recorded.planningProposal {
      guard recorded.planningMode == "external-proposals-v1" else {
        throw FlightError.protocolViolation("External proposal in an inline-planning record")
      }
      computer!.accept(try JSONDecoder().decode(MissionPlanningProposal.self, from: bytes), state: recorded.state)
    }
    let result = try computer!.step(recorded.state)
    guard result.sequence == recorded.sequence else {
      throw FlightError.protocolViolation("Full-rate replay has a missing, duplicate or reordered control record")
    }
    return result
  }
  private func replayOrbital(_ recorded: FlightRecord) throws -> FlightRecord {
    if let bytes = recorded.initialDeorbitPlan {
      let plan = try JSONDecoder().decode(DeorbitPlan.self, from: bytes)
      guard plan.configuration == configuration else { throw FlightError.invalidConfiguration("Replay orbital plan/configuration mismatch") }
      orbitalPlan = plan; burn = .init(plan: plan); coast = .init(); previousOrbitalTime = nil
      computer = nil
    }
    guard let plan = orbitalPlan, let burn else { throw FlightError.protocolViolation("Orbital replay requires its recorded plan seed") }
    let state = recorded.state, c = configuration
    let resumed = recorded.orbital?.telemetryGapResumed == true
    if resumed {
      if !burn.complete { try burn.resumeAfterTelemetryGap(at: state) }
      coast = OrbitalAttitudeController()
    }
    let dt = resumed ? c.guidance.controlInterval : (previousOrbitalTime.map { state.time-$0 } ?? c.guidance.controlInterval)
    previousOrbitalTime = state.time
    let phase: MissionPhase, command: OrbitalControlCommand
    if burn.complete {
      let target = Quaternion.windAttitude(position: state.position, airVelocity: state.velocity,
        alpha: c.entrySettings.nominalAlpha, bank: 0)
      phase = .entryCoast
      command = .init(planID: plan.identifier,
        axes: try coast.command(state: state, direction: target.rotate(.init(1, 0, 0)), dt: dt), throttle: 0)
    } else {
      let elements = try OrbitalMechanics(planet: c.planet).elements(.init(fixed: state, planet: c.planet))
      let output = try burn.update(state, periapsisAltitude: elements.periapsisAltitude)
      phase = output.phase; command = output.command
    }
    var result = FlightRecord(sequence: recorded.sequence, state: state,
      guidance: .init(phase: phase, target: .init(angleOfAttack: burn.complete ? c.entrySettings.nominalAlpha : 0)),
      command: command.axes)
    result.orbitalCommand = command
    return result
  }
}
