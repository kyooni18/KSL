import Foundation
import KSLCore

public struct FiniteBurnResult: Codable, Sendable {
  public let state: OrbitalState
  public let elapsed: Double
  public let deliveredDeltaV: Double
  public let complete: Bool
}

/// C finite-burn integration, with measured Isp/dry mass rather than a constant
/// wet mass. The aligned ideal burn screens geometry; execution qualification
/// must also check native attitude acquisition and the downstream flight.
public final class FiniteBurnModel: Sendable {
  private let orbit: OrbitalMechanics
  public init(planet: PlanetConfiguration) { orbit = .init(planet: planet) }
  public func propagate(from initial: OrbitalState, deltaV: Double,
                        propulsion: PropulsionTelemetry, settings: DeorbitConfiguration,
                        radialAngle: Double = 0, normalAngle: Double = 0) throws -> FiniteBurnResult {
    try propulsion.validate(mass: initial.mass)
    guard deltaV.isFinite, deltaV >= 0, abs(radialAngle) < .pi/3, abs(normalAngle) < .pi/3 else {
      throw FlightError.invalidConfiguration("Invalid finite burn demand")
    }
    var state = initial, delivered = 0.0
    while delivered < deltaV-BurnThrottleLaw.terminalDeltaVTolerance && state.time-initial.time < 1800 {
      try Task.checkCancellation()
      if state.mass <= propulsion.dryMass { break }
      let maxAcceleration = propulsion.availableThrust*settings.maximumThrottle/state.mass
      let fraction = BurnThrottleLaw.fraction(elapsed: state.time-initial.time,
        remaining: deltaV-delivered, maximumAcceleration: maxAcceleration, ramp: settings.throttleRampDuration)
      let acceleration = maxAcceleration*fraction
      var dt = min(0.25, max(0.02, (deltaV-delivered)/max(acceleration, 0.01)))
      let direction = OrbitalMechanics.burnDirection(position: state.position, velocity: state.velocity,
        radialAngle: radialAngle, normalAngle: normalAngle)
      let projection = max(0, direction.dot(-state.velocity.normalized))
      let thrust = propulsion.availableThrust*settings.maximumThrottle*fraction
      if thrust > 0 { dt = min(dt, (state.mass-propulsion.dryMass)*propulsion.specificImpulse*9.80665/thrust) }
      if dt <= 1e-9 { break }
      let consumed = min(state.mass-propulsion.dryMass, thrust*dt/(propulsion.specificImpulse*9.80665))
      let actualAcceleration = thrust/(state.mass-consumed/2)
      state = orbit.step(state, dt: dt) { position, velocity in
        OrbitalMechanics.burnDirection(position: position, velocity: velocity,
          radialAngle: radialAngle, normalAngle: normalAngle)*actualAcceleration
      }
      state.mass -= consumed
      delivered += actualAcceleration*projection*dt
      if state.position.length < orbit.planet.radius+orbit.planet.atmosphereTop { break }
    }
    return .init(state: state, elapsed: state.time-initial.time, deliveredDeltaV: delivered,
      complete: delivered >= deltaV-BurnThrottleLaw.terminalDeltaVTolerance)
  }
}
