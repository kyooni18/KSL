import Foundation
import KSLCore

public struct OrbitalElements: Codable, Sendable {
  public let specificEnergy: Double
  public let angularMomentum: Vector3
  public let eccentricity: Double
  public let periapsisAltitude: Double
  public let apoapsisAltitude: Double?
  public let period: Double?
}

/// Native C predictor_propagate_vacuum / predictor_postburn_periapsis equations.
public final class OrbitalMechanics: Sendable {
  public let planet: PlanetConfiguration
  public init(planet: PlanetConfiguration) { self.planet = planet }
  public func elements(_ state: OrbitalState) throws -> OrbitalElements {
    guard state.position.isFinite, state.velocity.isFinite, state.mass > 0,
      state.time.isFinite, state.position.length > planet.radius*0.5 else {
      throw FlightError.invalidTelemetry("Invalid inertial orbital state")
    }
    let mu = planet.gravitationalParameter, radius = state.position.length
    let h = state.position.cross(state.velocity)
    let energy = state.velocity.squaredLength/2-mu/radius
    let eccentricity = sqrt(max(0, 1+2*energy*h.squaredLength/(mu*mu)))
    let periapsis = h.squaredLength/(mu*(1+eccentricity))-planet.radius
    let a = energy < 0 ? -mu/(2*energy) : nil
    return .init(specificEnergy: energy, angularMomentum: h, eccentricity: eccentricity,
      periapsisAltitude: periapsis, apoapsisAltitude: a.map { $0*(1+eccentricity)-planet.radius },
      period: a.map { 2 * .pi * sqrt($0*$0*$0/mu) })
  }
  public func step(_ state: OrbitalState, dt: Double,
                   thrust: (Vector3, Vector3) -> Vector3 = { _, _ in .zero }) -> OrbitalState {
    func acceleration(_ p: Vector3, _ v: Vector3) -> Vector3 { planet.gravity(at: p)+thrust(p, v) }
    let p1 = state.velocity, v1 = acceleration(state.position, p1)
    let p2 = state.velocity+v1*(dt/2), v2 = acceleration(state.position+p1*(dt/2), p2)
    let p3 = state.velocity+v2*(dt/2), v3 = acceleration(state.position+p2*(dt/2), p3)
    let p4 = state.velocity+v3*dt, v4 = acceleration(state.position+p3*dt, p4)
    return .init(time: state.time+dt,
      position: state.position+(p1+2*p2+2*p3+p4)*(dt/6),
      velocity: state.velocity+(v1+2*v2+2*v3+v4)*(dt/6), mass: state.mass)
  }
  public func propagate(_ initial: OrbitalState, to time: Double, step maximumStep: Double = 3) throws -> OrbitalState {
    guard time.isFinite, time >= initial.time, maximumStep.isFinite, (0.01...120).contains(maximumStep) else {
      throw FlightError.invalidConfiguration("Invalid orbital propagation interval")
    }
    var state = initial
    while state.time < time {
      try Task.checkCancellation()
      state = step(state, dt: min(maximumStep, time-state.time))
      if state.position.length <= planet.radius { throw FlightError.infeasible("Vacuum coast intersects terrain") }
    }
    return state
  }
  /// Locate the descending interface crossing without skipping its epoch.
  public func entryInterface(from initial: OrbitalState, altitude: Double,
                              maximumDuration: Double) throws -> OrbitalState? {
    var state = initial
    let radius = planet.radius+altitude
    while state.time-initial.time < maximumDuration {
      try Task.checkCancellation()
      if state.position.length <= radius && state.position.dot(state.velocity) < 0 { return state }
      let previous = state
      state = step(state, dt: min(3, maximumDuration-(state.time-initial.time)))
      if state.position.length <= radius && previous.position.length > radius {
        var low = 0.0, high = state.time-previous.time
        for _ in 0..<30 {
          let mid = (low+high)/2
          if step(previous, dt: mid).position.length > radius { low = mid } else { high = mid }
        }
        return step(previous, dt: high)
      }
    }
    return nil
  }
  public static func burnDirection(position: Vector3, velocity: Vector3,
                                    radialAngle: Double = 0, normalAngle: Double = 0) -> Vector3 {
    let retro = -velocity.normalized
    let normal = position.cross(velocity).normalized
    let radial = position.normalized.projected(onPlaneWithNormal: retro).normalized
    return (retro+radial*tan(radialAngle)+normal*tan(normalAngle)).normalized
  }
}
