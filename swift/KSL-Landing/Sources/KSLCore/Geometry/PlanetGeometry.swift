import Foundation

public struct GeodeticPosition: Codable, Sendable, Equatable {
  public let latitude: Double
  public let longitude: Double
  public let altitude: Double
  public init(latitude: Double, longitude: Double, altitude: Double) {
    self.latitude = latitude
    self.longitude = longitude
    self.altitude = altitude
  }
  public func cartesian(radius: Double) -> Vector3 {
    let r = radius + altitude
    let c = cos(latitude)
    return .init(r * c * cos(longitude), r * c * sin(longitude), r * sin(latitude))
  }
  public init(position: Vector3, radius: Double) {
    latitude = atan2(position.z, hypot(position.x, position.y))
    longitude = atan2(position.y, position.x)
    altitude = position.length - radius
  }
}

public struct LocalFrame: Sendable {
  public let north: Vector3
  public let east: Vector3
  public let up: Vector3
  public init(position: Vector3) {
    up = position.normalized
    let longitude = atan2(position.y, position.x)
    east = .init(-sin(longitude), cos(longitude), 0)
    north = up.cross(east).normalized
  }
  public func vector(north n: Double, east e: Double, up u: Double) -> Vector3 {
    north * n + east * e + up * u
  }
  public func heading(of velocity: Vector3) -> Double {
    atan2(velocity.dot(east), velocity.dot(north))
  }
}

public struct PlanetConfiguration: Codable, Sendable, Equatable {
  public var radius: Double = 600_000
  public var gravitationalParameter: Double = 3.5316e12
  public var rotationRate: Double = 2 * .pi / 21_549.425
  public var atmosphereTop: Double = 70_000
  public init() {}
  public var angularVelocity: Vector3 { .init(0, 0, rotationRate) }
  public func gravity(at position: Vector3) -> Vector3 {
    let r = position.length
    return position * (-gravitationalParameter / (r * r * r))
  }
  public func fixedAcceleration(position: Vector3, velocity: Vector3, specificForce: Vector3)
    -> Vector3
  {
    let w = angularVelocity
    return gravity(at: position) + specificForce - w.cross(velocity) * 2
      - w.cross(w.cross(position))
  }
  public func inertialVelocity(position: Vector3, fixedVelocity: Vector3) -> Vector3 {
    fixedVelocity + angularVelocity.cross(position)
  }
}
