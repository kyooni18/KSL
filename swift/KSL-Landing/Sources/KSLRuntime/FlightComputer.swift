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
  private var sequence: UInt64 = 0
  private var previousTime: Double?
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) throws {
    try configuration.validate()
    self.configuration = configuration
    mission = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    controller = .init(configuration: configuration)
  }
  public func step(_ state: VehicleState) throws -> FlightRecord {
    guard state.systems.unpowered else {
      throw FlightError.invalidTelemetry("Propulsion, SAS, RCS or airbrakes active")
    }
    let solution = try mission.update(state)
    let dt = previousTime.map { state.time - $0 } ?? configuration.guidance.controlInterval
    let command = try controller.command(state: state, solution: solution, dt: dt)
    previousTime = state.time
    sequence += 1
    return .init(sequence: sequence, state: state, guidance: solution, command: command)
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
    let v = state.velocity
    guard v.length > 20 else { return nil }
    let va = v.normalized
    let upHere = state.position.normalized
    let right = va.cross(upHere)
    guard right.length > 1e-6 else { return nil }
    let windRight = right.normalized
    let windUp = windRight.cross(va)
    let roofWind = windUp * cos(bank) + windRight * sin(bank)
    let nose = (va * cos(alpha) + roofWind * sin(alpha)).normalized
    let roof = (roofWind * cos(alpha) - va * sin(alpha)).normalized
    return AttitudeTarget(nose: nose, roof: roof)
  }
}
