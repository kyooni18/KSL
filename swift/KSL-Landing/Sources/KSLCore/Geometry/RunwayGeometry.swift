import Foundation

public struct RunwayGeometry: Codable, Sendable, Equatable {
  public var name: String = "KSC Runway 09"
  public var latitude: Double = radians(-0.0486111111)
  public var longitude: Double = radians(-74.7283333333)
  public var elevation: Double = 68.44
  public var heading: Double = .pi / 2
  public var length: Double = 2500
  public var width: Double = 70
  public init() {}
  public func contains(along: Double, cross: Double, margin: Double = 0) -> Bool {
    along >= margin && along <= length - margin && abs(cross) <= width / 2 - margin
  }
}

public struct RunwayCoordinates: Codable, Sendable, Equatable {
  public let along: Double
  public let cross: Double
  public let height: Double
  public var horizontal: Vector2 { .init(along, cross) }
}

/// Spherical azimuthal-equidistant chart; cross is positive RIGHT of the runway.
/// Height is radial altitude above the runway, never a flat tangent-plane z.
public struct RunwayFrame: Sendable {
  public let runway: RunwayGeometry
  public let planet: PlanetConfiguration
  public let origin: Vector3
  public let up: Vector3
  public let alongAxis: Vector3
  public let rightAxis: Vector3
  public init(runway: RunwayGeometry, planet: PlanetConfiguration = .init()) {
    self.runway = runway
    self.planet = planet
    origin = GeodeticPosition(
      latitude: runway.latitude, longitude: runway.longitude, altitude: runway.elevation
    ).cartesian(radius: planet.radius)
    let local = LocalFrame(position: origin)
    up = local.up
    alongAxis = local.north * cos(runway.heading) + local.east * sin(runway.heading)
    rightAxis = alongAxis.cross(up)
  }
  public func coordinates(of position: Vector3) -> RunwayCoordinates {
    let n = position.normalized
    let dot = clamp(up.dot(n), -1, 1)
    let tangent = n - up * dot
    let angle = atan2(tangent.length, dot)
    let scaled = tangent.length > 1e-12 ? tangent * (planet.radius * angle / tangent.length) : .zero
    return .init(
      along: scaled.dot(alongAxis), cross: scaled.dot(rightAxis),
      height: position.length - planet.radius - runway.elevation)
  }
  public func position(along: Double, cross: Double, height: Double) -> Vector3 {
    let r = hypot(along, cross)
    let tangent = r > 1e-12 ? (alongAxis * along + rightAxis * cross) / r : alongAxis
    let n = up * cos(r / planet.radius) + tangent * sin(r / planet.radius)
    return n * (planet.radius + runway.elevation + height)
  }
  public func direction(at point: Vector2, tangent: Vector2) -> Vector3 {
    // Differentiate the chart on the sphere, including convergence of meridians.
    let p0 = position(along: point.x, cross: point.y, height: 0)
    let p1 = position(along: point.x + tangent.x, cross: point.y + tangent.y, height: 0)
    return (p1 - p0).projected(onPlaneWithNormal: p0.normalized).normalized
  }
  public func bearing(from position: Vector3, to target: Vector3) -> Double {
    let local = LocalFrame(position: position)
    let direction = target.normalized.projected(onPlaneWithNormal: local.up)
    return local.heading(of: direction)
  }
  public func distance(from position: Vector3, to target: Vector3) -> Double {
    let a = position.normalized
    let b = target.normalized
    return planet.radius * atan2(a.cross(b).length, clamp(a.dot(b), -1, 1))
  }
}
