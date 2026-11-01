import Foundation
import KSLCore
import KSLPhysics

/// Energy-to-drag allocation for the split rudder. It never creates drag to
/// correct an energy deficit and never operates on Final, whose energy guide
/// owns the remaining trajectory. A missing identified drag area disables it.
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
