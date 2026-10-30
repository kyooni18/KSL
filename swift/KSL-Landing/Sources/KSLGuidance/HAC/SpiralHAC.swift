import Foundation
import KSLCore

public struct HorizontalPathSample: Sendable {
  public let position: Vector2
  public let tangent: Vector2
  /// Positive is a turn to the right in the runway's along/right chart.
  public let curvature: Double
  public init(position: Vector2, tangent: Vector2, curvature: Double) {
    self.position = position
    self.tangent = tangent
    self.curvature = curvature
  }
}

/// A genuine variable-radius spiral; theta is REMAINING angle in radians.
/// The trajectory travels toward theta=0, at which its tangent is runway-forward.
public struct SpiralHAC: Codable, Sendable {
  public static let maximumTurnAngle = 3 * Double.pi
  public let exitAlong: Double
  public let finalRadius: Double
  public let growth: Double
  public let turnAngle: Double
  public let side: Int
  public init(exitAlong: Double, finalRadius: Double, growth: Double, turnAngle: Double, side: Int)
    throws
  {
    guard exitAlong.isFinite, finalRadius.isFinite, finalRadius > 0, growth.isFinite, growth > 0,
      growth < finalRadius / 2, turnAngle.isFinite, turnAngle > 0, turnAngle <= Self.maximumTurnAngle,
      side == -1 || side == 1
    else { throw FlightError.invalidConfiguration("Invalid spiral geometry") }
    self.exitAlong = exitAlong
    self.finalRadius = finalRadius
    self.growth = growth
    self.turnAngle = turnAngle
    self.side = side
  }
  public func radius(at theta: Double) -> Double { finalRadius + growth * theta * theta }
  public func sample(remainingAngle theta: Double) -> HorizontalPathSample {
    let r = radius(at: theta)
    let rp = 2 * growth * theta
    let rpp = 2 * growth
    let s = Double(side)
    let p = Vector2(exitAlong - r * sin(theta), s * (finalRadius - r * cos(theta)))
    let derivative = Vector2(
      -rp * sin(theta) - r * cos(theta), -s * (rp * cos(theta) - r * sin(theta)))
    let second = Vector2(
      -rpp * sin(theta) - 2 * rp * cos(theta) + r * sin(theta),
      -s * (rpp * cos(theta) - 2 * rp * sin(theta) - r * cos(theta)))
    let curvature = -derivative.cross(second) / pow(derivative.length, 3)
    return .init(position: p, tangent: -derivative.normalized, curvature: curvature)
  }
  /// Invert the polar chart at the measured position, unwrapping against
  /// this spiral's winding. No accumulated fixed-path station is needed.
  public func fittedRemainingAngle(at point: Vector2) -> Double {
    let raw = atan2(exitAlong-point.x, finalRadius-Double(side)*point.y)
    var candidates = [0.0, turnAngle]
    for winding in 0...Int(ceil(turnAngle/(2 * .pi))) {
      let angle = raw + Double(winding)*2 * .pi
      if angle >= 0 && angle <= turnAngle { candidates.append(angle) }
    }
    return candidates.min { (sample(remainingAngle: $0).position-point).length
      < (sample(remainingAngle: $1).position-point).length } ?? 0
  }
  public func length(samples: Int = 256) -> Double {
    let count = max(2, samples)
    let d = turnAngle / Double(count)
    var sum = 0.0
    for i in 0...count {
      let t = Double(i) * d
      let speed = hypot(radius(at: t), 2 * growth * t)
      sum += speed * (i == 0 || i == count ? 0.5 : 1)
    }
    return sum * d
  }
}
