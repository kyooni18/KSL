import KSLCore

/// Rotating-frame mechanical energy relative to the selected runway.
/// Heights are an energy unit (J/kg divided by fixed runway gravity), not
/// geometric altitude. Velocity is planet-fixed; airspeed belongs in q only.
struct RunwayEnergy {
  let planet: PlanetConfiguration
  let origin: Vector3
  let gravity: Double
  init(configuration: MissionConfiguration) {
    planet = configuration.planet
    origin = RunwayFrame(runway: configuration.runway, planet: planet).origin
    gravity = planet.gravitationalParameter / origin.squaredLength
  }
  func potentialHeight(at position: Vector3) -> Double {
    (planet.gravitationalParameter * (1 / origin.length - 1 / position.length)
      - (planet.angularVelocity.cross(position).squaredLength
        - planet.angularVelocity.cross(origin).squaredLength) / 2) / gravity
  }
  func height(position: Vector3, speedSquared: Double) -> Double {
    potentialHeight(at: position) + speedSquared / (2 * gravity)
  }
  func height(_ state: VehicleState) -> Double {
    height(position: state.position, speedSquared: state.velocity.squaredLength)
  }
  func speedSquared(height: Double, position: Vector3) -> Double {
    max(0, 2 * gravity * (height - potentialHeight(at: position)))
  }
}
