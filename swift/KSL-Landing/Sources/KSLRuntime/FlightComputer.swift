import Foundation
import KSLControl
import KSLCore
import KSLGuidance
import KSLPhysics

/// Synchronous deterministic computation shared verbatim by live and offline runners.
public final class FlightComputer {
  public let configuration: MissionConfiguration
  public let mission: LandingMissionController
  public let controller: FlightController
  public let calibration: FlightCalibrationEstimator
  private var sequence: UInt64 = 0
  private var previousTime: Double?
  private var dynamics = VehicleDynamicsMonitor()
  public var dynamicsRevision: UInt64 { dynamics.revision }
  public var lastDynamicsChange: VehicleDynamicsChange? { dynamics.lastChange }
  @discardableResult public func observeConfiguration(_ state: VehicleState) -> Bool {
    guard let change = dynamics.observe(state) else { return false }
    controller.adaptToDynamicsChange(change, at: state)
    mission.invalidateDynamics(at: state)
    calibration.invalidateDynamics()
    return true
  }
  private let initialPhase: MissionPhase
  private let encodedInitialPlan: Data?
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine, externalPlanning: Bool = false, initialPhase: MissionPhase = .entry,
    initialPlan: TerminalPlan? = nil, entryReplanning: Bool = true
  ) throws {
    try configuration.validate()
    self.configuration = configuration
    self.initialPhase = initialPhase
    encodedInitialPlan = try initialPlan.map { try JSONEncoder().encode($0) }
    mission = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
      externalPlanning: externalPlanning, initialPhase: initialPhase, initialPlan: initialPlan,
      entryReplanning: entryReplanning)
    controller = .init(configuration: configuration)
    calibration = try .init(configuration: configuration, atmosphere: atmosphere)
  }
  public func planningRequest(from state: VehicleState, allowEntry: Bool = true,
                              allowTerminal: Bool = true) -> MissionPlanningRequest? {
    prepare(state)
    guard var request = mission.planningRequest(from: state, controls: controller.snapshot,
      allowEntry: allowEntry, allowTerminal: allowTerminal) else { return nil }
    request.dynamicsRevision = dynamics.revision
    return request
  }
  private func prepare(_ state: VehicleState) {
    observeConfiguration(state)
    calibration.observe(state)
    mission.environment = calibration.diagnostics.environment
  }
  @discardableResult public func accept(_ proposal: MissionPlanningProposal, state: VehicleState) -> Bool {
    observeConfiguration(state)
    guard proposal.time <= state.time else { return false }
    if let revision = proposal.dynamicsRevision {
      guard revision == dynamics.revision else { return false }
    } else if let change = dynamics.lastChange, proposal.time <= change.time {
      return false
    }
    mission.accept(proposal, state: state)
    if let entry = proposal.entry, entry.createdAt <= state.time,
      state.time-entry.createdAt <= 2*configuration.guidance.entryReplanInterval {
      calibration.setForecast(entry.prediction)
    }
    return true
  }
  public func resumeAfterTelemetryGap(at state: VehicleState) {
    previousTime = nil
    mission.resumeAfterTelemetryGap()
    controller.resumeAfterTelemetryGap(at: state)
  }
  public func step(_ state: VehicleState) throws -> FlightRecord {
    guard state.systems.unpowered else {
      throw FlightError.invalidTelemetry("Propulsion, SAS or airbrakes active")
    }
    try state.validate(configuration: configuration)
    if let previousTime, state.time <= previousTime || state.time-previousTime > 1 {
      throw FlightError.rearmRequired("Flight computer clock discontinuity")
    }
    prepare(state)
    mission.controlFeedback = controller.diagnostics
    mission.controlSnapshot = controller.snapshot
    let solution = try mission.update(state)
    calibration.setForecast(mission.latestEntryPrediction)
    let dt = previousTime.map { state.time - $0 } ?? configuration.guidance.controlInterval
    let command = try controller.command(state: state, solution: solution, dt: dt)
    previousTime = state.time
    sequence += 1
    var record = FlightRecord(sequence: sequence, state: state, guidance: solution, command: command,
      control: controller.diagnostics)
    record.calibration = calibration.diagnostics
    if lastDynamicsChange?.time == state.time { record.dynamicsChange = lastDynamicsChange }
    record.planningMode = mission.externalPlanning ? "external-proposals-v1" : "inline-deterministic-v1"
    if sequence == 1 { record.initialPhase = initialPhase; record.initialTerminalPlan = encodedInitialPlan }
    return record
  }
  /// Attitude that realises the guidance angle of attack and bank with zero sideslip, using the
  /// same wind-axis definitions as the measured bank in the live adapter.
  public func attitudeTarget(state: VehicleState, solution: GuidanceSolution) -> AttitudeTarget? {
    Self.attitudeTarget(
      state: state, angleOfAttack: solution.target.angleOfAttack, bank: solution.target.bank)
  }
  public static func attitudeTarget(state: VehicleState, angleOfAttack alpha: Double, bank: Double)
    -> AttitudeTarget?
  {
    guard let q = AirRelativeAttitude.target(position: state.position,
      airVelocity: state.velocity - state.atmosphere.wind, alpha: alpha, bank: bank) else { return nil }
    return .init(nose: q.rotate(.init(1, 0, 0)), roof: q.rotate(.init(0, 0, -1)))
  }
}
