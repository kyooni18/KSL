import Foundation
import KSLCore

public struct KinematicState: Sendable {
  public var position: Vector3
  public var velocity: Vector3
  public init(position: Vector3, velocity: Vector3) {
    self.position = position
    self.velocity = velocity
  }
}

public final class TranslationalDynamicsEngine: Sendable {
  public let planet: PlanetConfiguration
  public let atmosphere: AtmosphericModelEngine
  public let aerodynamics: AerodynamicsEngine
  public init(
    planet: PlanetConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.planet = planet
    self.atmosphere = atmosphere
    self.aerodynamics = aerodynamics
  }
  public func acceleration(state: KinematicState, time: Double, mass: Double, attitude: Quaternion,
                           systems: SystemState = .init())
    -> Vector3
  {
    let atm = atmosphere.sample(position: state.position, time: time)
    let aero = aerodynamics.evaluate(
      position: state.position, velocity: state.velocity, attitude: attitude, atmosphere: atm,
      gear: systems.gearDown, brakes: systems.brakes, airbrakes: systems.airbrakes)
    return planet.fixedAcceleration(
      position: state.position, velocity: state.velocity, specificForce: aero.force / mass)
  }
  public func step(
    _ s: KinematicState, time: Double, mass: Double, attitude: Quaternion, dt: Double, systems: SystemState = .init()
  ) -> KinematicState {
    func derivative(_ p: Vector3, _ v: Vector3, _ t: Double) -> Vector3 {
      acceleration(state: .init(position: p, velocity: v), time: t, mass: mass, attitude: attitude, systems: systems)
    }
    let k1p = s.velocity
    let k1v = derivative(s.position, s.velocity, time)
    let k2p = s.velocity + k1v * (dt / 2)
    let k2v = derivative(s.position + k1p * (dt / 2), k2p, time + dt / 2)
    let k3p = s.velocity + k2v * (dt / 2)
    let k3v = derivative(s.position + k2p * (dt / 2), k3p, time + dt / 2)
    let k4p = s.velocity + k3v * dt
    let k4v = derivative(s.position + k3p * dt, k4p, time + dt)
    return .init(
      position: s.position + (k1p + k2p * 2 + k3p * 2 + k4p) * (dt / 6),
      velocity: s.velocity + (k1v + k2v * 2 + k3v * 2 + k4v) * (dt / 6))
  }
}
