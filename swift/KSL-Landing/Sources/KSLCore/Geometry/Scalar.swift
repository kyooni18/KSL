import Foundation

public func radians(_ degrees: Double) -> Double { degrees * .pi / 180 }
public func degrees(_ radians: Double) -> Double { radians * 180 / .pi }
public func clamp(_ value: Double, _ lower: Double, _ upper: Double) -> Double {
  min(upper, max(lower, value))
}
public func wrapAngle(_ angle: Double) -> Double {
  guard angle.isFinite else { return .nan }
  var result = angle.truncatingRemainder(dividingBy: 2 * .pi)
  if result > .pi { result -= 2 * .pi }
  if result < -.pi { result += 2 * .pi }
  return result
}
public func moveToward(_ current: Double, _ target: Double, maximumDelta: Double) -> Double {
  current + clamp(target - current, -maximumDelta, maximumDelta)
}

public struct Vector2: Codable, Sendable, Equatable {
  public var x: Double
  public var y: Double
  public init(_ x: Double = 0, _ y: Double = 0) {
    self.x = x
    self.y = y
  }
  public var length: Double { hypot(x, y) }
  public var normalized: Self { length > 1e-12 ? self / length : .init() }
  public var rightNormal: Self { .init(-y, x) }
  public func dot(_ other: Self) -> Double { x * other.x + y * other.y }
  public func cross(_ other: Self) -> Double { x * other.y - y * other.x }
  public static func + (a: Self, b: Self) -> Self { .init(a.x + b.x, a.y + b.y) }
  public static func - (a: Self, b: Self) -> Self { .init(a.x - b.x, a.y - b.y) }
  public static prefix func - (a: Self) -> Self { .init(-a.x, -a.y) }
  public static func * (a: Self, s: Double) -> Self { .init(a.x * s, a.y * s) }
  public static func / (a: Self, s: Double) -> Self { .init(a.x / s, a.y / s) }
}
