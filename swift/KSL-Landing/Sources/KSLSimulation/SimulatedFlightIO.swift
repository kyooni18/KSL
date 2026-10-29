import Foundation
import KSLCore
import KSLPhysics

public actor SimulatedFlightIO: FlightIO {
  private let physics: ShuttlePhysicsEngine
  private let interval: Double
  private var state: VehicleState
  private var sequence: UInt64 = 0
  public private(set) var closed = false
  public init(scenario: SimulationScenario) throws {
    let c = scenario.configuration
    physics = .init(
      configuration: c, atmosphere: try .init(planet: c.planet),
      aerodynamics: try .init(referenceArea: c.vehicle.referenceArea))
    state = physics.observe(scenario.initialState)
    interval = c.guidance.controlInterval
  }
  public func readState() throws -> VehicleState {
    guard !closed else { throw FlightError.disconnected("Simulation closed") }
    return state
  }
  public func apply(_ envelope: CommandEnvelope) throws {
    guard !closed, envelope.sequence > sequence, abs(envelope.stateTime - state.time) < 1e-6 else {
      throw FlightError.invalidCommand("Out-of-order simulation command")
    }
    state = try physics.advance(state, command: envelope.command, duration: interval)
    sequence = envelope.sequence
  }
  public func shutdown() { closed = true }
}
