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
}
