import Foundation
import KSLCore

public struct FiniteBurnResult: Codable, Sendable {
  public let state: OrbitalState
  public let elapsed: Double
  public let deliveredDeltaV: Double
  public let complete: Bool
  public var deliveredNormalDeltaV: Double? = nil
}

/// C finite-burn integration, with measured Isp/dry mass rather than a constant
/// wet mass. The aligned ideal burn screens geometry; execution qualification
/// must also check native attitude acquisition and the downstream flight.
public final class FiniteBurnModel: Sendable {
  private let orbit: OrbitalMechanics
  public init(planet: PlanetConfiguration) { orbit = .init(planet: planet) }
  public func propagate(from initial: OrbitalState, deltaV: Double,
                        propulsion: PropulsionTelemetry, settings: DeorbitConfiguration,
                        radialAngle: Double = 0, normalAngle: Double = 0,
                        normalDeltaV: Double? = nil) throws -> FiniteBurnResult {
    try propulsion.validate(mass: initial.mass)
    guard deltaV.isFinite, deltaV >= 0, abs(radialAngle) < .pi/3,
      normalDeltaV.map({ $0.isFinite && radialAngle == 0 }) ?? (abs(normalAngle) < .pi/3) else {
      throw FlightError.invalidConfiguration("Invalid finite burn demand")
    }
    var state = initial, delivered = 0.0, normalDelivered = 0.0
    let tolerance = BurnThrottleLaw.terminalDeltaVTolerance
    func remaining() -> Double {
      normalDeltaV.map { abs($0-normalDelivered) > tolerance
        ? abs($0-normalDelivered) : max(0,deltaV-delivered) }
        ?? max(0, deltaV-delivered)
    }
    func direction(_ position: Vector3, _ velocity: Vector3) -> Vector3 {
      guard let normalDeltaV else {
        return OrbitalMechanics.burnDirection(position: position, velocity: velocity,
          radialAngle: radialAngle, normalAngle: normalAngle)
      }
      // Correct the plane before lowering periapsis. Spreading retrograde
      // work across a long normal-dominated burn can reach atmosphere before
      // the plane correction finishes, despite adequate propellant.
      let normalRemaining = normalDeltaV-normalDelivered
      if abs(normalRemaining) > tolerance {
        return position.cross(velocity).normalized * (normalRemaining < 0 ? -1 : 1)
      }
      return -velocity.normalized
    }
    while remaining() > tolerance && state.time-initial.time < 1800 {
      try Task.checkCancellation()
      if state.mass <= propulsion.dryMass { break }
      let maxAcceleration = propulsion.availableThrust*settings.maximumThrottle/state.mass
      let fraction = BurnThrottleLaw.fraction(elapsed: state.time-initial.time,
        remaining: remaining(), maximumAcceleration: maxAcceleration, ramp: settings.throttleRampDuration)
      let acceleration = maxAcceleration*fraction
      var dt = min(0.25, max(0.02, remaining()/max(acceleration, 0.01)))
      if normalDeltaV != nil && acceleration > 0 { dt = min(dt, remaining()/acceleration) }
      let thrustDirection = direction(state.position, state.velocity)
      let projection = max(0, thrustDirection.dot(-state.velocity.normalized))
      let normalProjection = thrustDirection.dot(state.position.cross(state.velocity).normalized)
      let thrust = propulsion.availableThrust*settings.maximumThrottle*fraction
      if thrust > 0 { dt = min(dt, (state.mass-propulsion.dryMass)*propulsion.specificImpulse*9.80665/thrust) }
      if dt <= 1e-9 { break }
      let consumed = min(state.mass-propulsion.dryMass, thrust*dt/(propulsion.specificImpulse*9.80665))
      let actualAcceleration = thrust/(state.mass-consumed/2)
      state = orbit.step(state, dt: dt) { position, velocity in
        direction(position, velocity)*actualAcceleration
      }
      state.mass -= consumed
      delivered += actualAcceleration*projection*dt
      normalDelivered += actualAcceleration*normalProjection*dt
      if state.position.length < orbit.planet.radius+orbit.planet.atmosphereTop { break }
    }
    return .init(state: state, elapsed: state.time-initial.time, deliveredDeltaV: delivered,
      complete: remaining() <= tolerance,
      deliveredNormalDeltaV: normalDeltaV == nil ? nil : normalDelivered)
  }
}
