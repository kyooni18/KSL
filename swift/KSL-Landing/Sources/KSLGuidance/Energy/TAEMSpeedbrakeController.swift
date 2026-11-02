import Foundation
import KSLCore
import KSLPhysics

/// Energy-to-drag allocation shared by TAEM and Final. Predictive allocation
/// requires an identified drag area. Actual flight can instead close the loop
/// on measured drag work, without inventing deployment authority for forecasts.
struct TAEMSpeedbrakeController: Sendable {
  let configuration: MissionConfiguration
  let atmosphere: AtmosphericModelEngine
  let frame: RunwayFrame

  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }

  private func effectiveEnergy(speed: Double, position: Vector3) -> Double {
    let rotation = configuration.planet.angularVelocity
    return speed*speed/2 - configuration.planet.gravitationalParameter/position.length
      - rotation.cross(position).squaredLength/2
  }

  func feedback(_ state: VehicleState, target: NominalEnergyPoint,
                remainingDistance: Double, dt: Double, responseSeconds: Double,
                availableLiftAcceleration: Double) -> Double {
    guard dt > 0, dt <= 1, remainingDistance > 0,
      TerminalBrakeEnvelope.permits(state, configuration: configuration,
        availableLiftAcceleration: availableLiftAcceleration) else { return 0 }
    let targetPosition = frame.position(along: target.along, cross: 0, height: target.height)
    let targetAir = atmosphere.sample(position: targetPosition, time: state.time)
    let tangent = frame.direction(at: .init(target.along, 0), tangent: .init(1, 0))
    let targetVelocity = target.airspeed * (tangent*cos(target.flightPathAngle)
      + targetPosition.normalized*sin(target.flightPathAngle)) + targetAir.wind
    let excess = effectiveEnergy(speed: state.velocity.length, position: state.position)
      - effectiveEnergy(speed: targetVelocity.length, position: targetPosition)
    guard excess > 0 else { return 0 }
    let airVelocity = state.velocity-state.atmosphere.wind
    let workPerGroundMetre = airVelocity.normalized.dot(state.velocity)/max(1, state.horizontalSpeed)
    guard workPerGroundMetre > 0 else { return 0 }
    let required = excess/(remainingDistance*workPerGroundMetre)
    let observed = state.aerodynamics.drag/state.mass
    let error = required-observed
    let current = state.systems.speedbrakeFraction ?? 0
    // Integral drag-work closure. The dimensionless relative residual is
    // integrated over the existing guidance/actuator response horizon. No
    // full-open CdA or artificial vehicle capability is inferred from it.
    let horizon = max(TerminalBrakeEnvelope.travelSeconds, responseSeconds)
    let increment = error/max(required, observed)*dt/horizon
    return clamp(current+increment, 0, 1)
  }

  func fraction(speed: Double, position: Vector3, time: Double, mass: Double,
                remainingDistance: Double, target: NominalEnergyPoint?,
                cleanDragAcceleration: Double) -> Double {
    guard let area = configuration.vehicle.speedbrakeFullDeploymentDragArea,
      let target, remainingDistance > 0, speed > 0, mass > 0,
      cleanDragAcceleration.isFinite else { return 0 }
    let targetPosition = frame.position(along: target.along, cross: 0, height: target.height)
    let air = atmosphere.sample(position: position, time: time)
    let fullAuthority = 0.5*air.density*speed*speed*area/mass
    guard fullAuthority > 0 else { return 0 }
    let required = (effectiveEnergy(speed: speed, position: position)
      - effectiveEnergy(speed: target.airspeed, position: targetPosition))/remainingDistance
    return clamp((required-cleanDragAcceleration)/fullAuthority, 0, 1)
  }

}
