import Foundation
import KSLCore

/// Identify the retrograde component from successive vacuum state vectors.
/// This is a finite-difference estimate, not measured engine impulse. Subtract
/// the known gravitational contribution before projecting into the velocity
/// direction; total engine integral and retrograde delta-v are distinct.
public struct RetrogradeImpulseObserver: Sendable {
  private let planet: PlanetConfiguration
  private var previous: OrbitalState?
  private var previousThrust = false
  public private(set) var identifiedDeltaV = 0.0
  public init(planet: PlanetConfiguration) { self.planet = planet }
  public mutating func observe(_ state: OrbitalState, thrustReported: Bool) throws {
    defer { previous = state; previousThrust = thrustReported }
    guard let old = previous else { return }
    let dt = state.time-old.time
    guard dt > 0, dt <= 1 else { throw FlightError.rearmRequired("Impulse observer clock discontinuity") }
    guard thrustReported || previousThrust else { return }
    guard min(old.position.length,state.position.length) > planet.radius+planet.atmosphereTop else {
      throw FlightError.infeasible("Vacuum impulse inference is contaminated by atmospheric forces")
    }
    let gravityImpulse = (planet.gravity(at: old.position)+planet.gravity(at: state.position))*(dt/2)
    let thrustImpulse = state.velocity-old.velocity-gravityImpulse
    let retrograde = -(state.velocity+old.velocity).normalized
    identifiedDeltaV += thrustImpulse.dot(retrograde)
  }
}
