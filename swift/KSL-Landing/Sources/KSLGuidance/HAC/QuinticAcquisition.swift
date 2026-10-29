import Foundation
import KSLCore

/// C2 Hermite acquisition curve. Endpoint tangents AND curvatures match.
public struct QuinticAcquisition: Sendable {
  private let a: [Vector2]
  public init(start: HorizontalPathSample, end: HorizontalPathSample, tangentLength: Double) {
    let l = max(1, tangentLength)
    let a0 = start.position
    let a1 = start.tangent * l
    let a2 = start.tangent.rightNormal * (start.curvature * l * l / 2)
    let b0 = end.position - a0 - a1 - a2
    let b1 = end.tangent * l - a1 - a2 * 2
    let b2 = end.tangent.rightNormal * (end.curvature * l * l) - a2 * 2
    a = [
      a0, a1, a2, b0 * 10 - b1 * 4 + b2 * 0.5, b0 * (-15) + b1 * 7 - b2, b0 * 6 - b1 * 3 + b2 * 0.5,
    ]
  }
  public func sample(_ u: Double) -> HorizontalPathSample {
    let t = clamp(u, 0, 1)
    var p = a[5]
    var d = a[5] * 5
    var dd = a[5] * 20
    for i in stride(from: 4, through: 0, by: -1) { p = p * t + a[i] }
    for i in stride(from: 4, through: 1, by: -1) { d = d * t + a[i] * Double(i) }
    for i in stride(from: 4, through: 2, by: -1) { dd = dd * t + a[i] * Double(i * (i - 1)) }
    return .init(
      position: p, tangent: d.normalized,
      curvature: d.length > 1e-8 ? d.cross(dd) / pow(d.length, 3) : .infinity)
  }
}
