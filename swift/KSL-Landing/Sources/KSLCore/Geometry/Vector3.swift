import Foundation

/// Right-handed planet-fixed coordinates: X prime meridian, Y east, Z north.
public struct Vector3: Codable, Sendable, Equatable {
  public var x: Double
  public var y: Double
  public var z: Double
  public init(_ x: Double = 0, _ y: Double = 0, _ z: Double = 0) {
    self.x = x
    self.y = y
    self.z = z
  }
  public static let zero = Self()
  public var squaredLength: Double { dot(self) }
  public var length: Double { sqrt(squaredLength) }
  public var isFinite: Bool { x.isFinite && y.isFinite && z.isFinite }
  public var normalized: Self { length > 1e-12 ? self / length : .zero }
  public func dot(_ other: Self) -> Double { x * other.x + y * other.y + z * other.z }
  public func cross(_ b: Self) -> Self {
    .init(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x)
  }
  public func projected(onPlaneWithNormal n: Self) -> Self { self - n * dot(n) }
  public func rotated(about axis: Self, angle: Double) -> Self {
    let n = axis.normalized
    return self * cos(angle) + n.cross(self) * sin(angle) + n * (n.dot(self) * (1 - cos(angle)))
  }
  public static func + (a: Self, b: Self) -> Self { .init(a.x + b.x, a.y + b.y, a.z + b.z) }
  public static func - (a: Self, b: Self) -> Self { .init(a.x - b.x, a.y - b.y, a.z - b.z) }
  public static prefix func - (a: Self) -> Self { .init(-a.x, -a.y, -a.z) }
  public static func * (a: Self, s: Double) -> Self { .init(a.x * s, a.y * s, a.z * s) }
  public static func * (s: Double, a: Self) -> Self { a * s }
  public static func / (a: Self, s: Double) -> Self { .init(a.x / s, a.y / s, a.z / s) }
  public func multiplied(componentwise b: Self) -> Self { .init(x * b.x, y * b.y, z * b.z) }
  public func divided(componentwise b: Self) -> Self { .init(x / b.x, y / b.y, z / b.z) }
}
