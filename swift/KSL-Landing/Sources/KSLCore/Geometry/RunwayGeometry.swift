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
  public func reciprocal(planet: PlanetConfiguration) -> Self {
    let frame = RunwayFrame(runway: self, planet: planet)
    let end = frame.position(along: length, cross: 0, height: 0)
    let geo = GeodeticPosition(position: end, radius: planet.radius)
    let reverse = -frame.direction(at: .init(length, 0), tangent: .init(1, 0))
    var result = self
    result.latitude = geo.latitude; result.longitude = geo.longitude
    result.heading = LocalFrame(position: end).heading(of: reverse)
    // The physical same runway can be described from either end. Avoid
    // appending repeated "(reciprocal)" labels on successive replans.
    if name.hasSuffix("09") { result.name = String(name.dropLast(2))+"27" }
    else if name.hasSuffix("27") { result.name = String(name.dropLast(2))+"09" }
    else if name.hasSuffix("36") { result.name = String(name.dropLast(2))+"18" }
    else if name.hasSuffix("18") { result.name = String(name.dropLast(2))+"36" }
    else { result.name = name+" (reciprocal)" }
    return result
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
    // Analytic differential of the azimuthal-equidistant chart. A chart
    // tangent is not a projection onto the fixed axes at the threshold.
    let radius = point.length
    guard radius > planet.radius * sqrt(Double.ulpOfOne) else {
      return (alongAxis * tangent.x + rightAxis * tangent.y).normalized
    }
    let radial = point / radius
    let theta = radius / planet.radius
    let axis = alongAxis * radial.x + rightAxis * radial.y
    let localRadial = axis * cos(theta) - up * sin(theta)
    let transverse = tangent - radial * tangent.dot(radial)
    return (localRadial * tangent.dot(radial)
      + (alongAxis * transverse.x + rightAxis * transverse.y) * (sin(theta) / theta)).normalized
  }
  /// Exact differential of coordinates(of:), in chart metres/second.
  /// Includes the altitude scale; orthonormal runway-right speed is not
  /// d(cross)/dt away from the threshold. The antipode is a chart singularity.
  public func coordinateVelocity(at position: Vector3, velocity: Vector3) -> Vector2 {
    let point = coordinates(of: position).horizontal
    let radius = point.length
    let scale = planet.radius / position.length
    guard radius > planet.radius * sqrt(Double.ulpOfOne) else {
      return .init(velocity.dot(alongAxis) * scale, velocity.dot(rightAxis) * scale)
    }
    let radial = point / radius
    let transverse = Vector2(-radial.y, radial.x)
    let theta = radius / planet.radius
    guard abs(sin(theta)) > Double.ulpOfOne else { return .init(.nan, .nan) }
    let axis = alongAxis * radial.x + rightAxis * radial.y
    let localRadial = axis * cos(theta) - up * sin(theta)
    let localTransverse = alongAxis * transverse.x + rightAxis * transverse.y
    return (radial * velocity.dot(localRadial)
      + transverse * (velocity.dot(localTransverse) * theta / sin(theta))) * scale
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
