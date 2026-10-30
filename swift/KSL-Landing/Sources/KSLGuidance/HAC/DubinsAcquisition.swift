import Foundation
import KSLCore

/// Turn-straight-turn acquisition with a speed-limited first turn. From a
/// supersonic TAEM state a coordinated turn has a radius of tens of km, so
/// the acquisition must turn at that radius, dissipate energy on a straight
/// leg, then roll into an arc that continues smoothly into the HAC spiral.
/// Curvature sign follows the chart convention: positive turns toward
/// `tangent.rightNormal`.
public struct DubinsAcquisition: Sendable {
  public let samples: [HorizontalPathSample]
  public let length: Double

  /// - Parameters:
  ///   - startRadius/startTurn: first-arc radius and direction (+1/-1).
  ///   - endRadius/endTurn: arc arriving at `end`, matching the HAC inlet.
  public init?(start: HorizontalPathSample, startRadius r1: Double, startTurn s1: Double,
               end: HorizontalPathSample, endRadius r2: Double, endTurn s2: Double,
               spacing: Double = 250, transition: Double = 0) {
    guard r1 > 0, r2 > 0, r1.isFinite, r2.isFinite, spacing > 0 else { return nil }
    let c1 = start.position + start.tangent.rightNormal * (s1 * r1)
    let c2 = end.position + end.tangent.rightNormal * (s2 * r2)
    let d = c2 - c1, k = s2 * r2 - s1 * r1
    let dd = d.dot(d)
    guard dd > k * k + 1 else { return nil }
    let straight = sqrt(dd - k * k)
    let angle = atan2(d.y, d.x) - atan2(k, straight)
    let u = Vector2(cos(angle), sin(angle))
    func sweep(from t: Vector2, to v: Vector2, turn s: Double) -> Double {
      var a = s * atan2(t.cross(v), t.dot(v))
      if a < 0 { a += 2 * .pi }
      return a
    }
    let a1 = sweep(from: start.tangent, to: u, turn: s1)
    let a2 = sweep(from: u, to: end.tangent, turn: s2)
    func arc(center c: Vector2, radius r: Double, turn s: Double, from t: Vector2, angle total: Double)
      -> [HorizontalPathSample] {
      let n = max(1, Int(ceil(r * total / spacing)))
      return (0...n).map { i in
        let a = total * Double(i) / Double(n)
        let tangent = t * cos(a) + t.rightNormal * (s * sin(a))
        return .init(position: c - tangent.rightNormal * (s * r), tangent: tangent, curvature: s / r)
      }
    }
    var path = arc(center: c1, radius: r1, turn: s1, from: start.tangent, angle: a1)
    let t1 = c1 - u.rightNormal * (s1 * r1)
    let m = max(1, Int(ceil(straight / spacing)))
    for i in 1...m {
      path.append(.init(position: t1 + u * (straight * Double(i) / Double(m)), tangent: u, curvature: 0))
    }
    path += arc(center: c2, radius: r2, turn: s2, from: u, angle: a2).dropFirst()
    length = r1 * a1 + straight + r2 * a2
    samples = Self.rollLimited(path, initialCurvature: start.curvature, end: end, transition: transition)
  }

  /// Replace curvature steps with linear roll-in/roll-out over `transition`
  /// metres (a moving average preserves the heading change), re-integrate the
  /// track, and distribute the small closure error so the end still meets the
  /// HAC inlet exactly.
  static func rollLimited(_ path: [HorizontalPathSample], initialCurvature: Double,
                          end: HorizontalPathSample, transition: Double) -> [HorizontalPathSample] {
    guard path.count > 2, transition > 0 else { return path }
    var distance = [0.0]
    for i in 1..<path.count { distance.append(distance[i - 1] + (path[i].position - path[i - 1].position).length) }
    let total = distance.last!
    func raw(_ s: Double) -> Double {
      if s <= 0 { return initialCurvature }
      if s >= total { return path.last!.curvature }
      var lo = 0, hi = path.count - 1
      while hi - lo > 1 { let mid = (lo + hi) / 2; if distance[mid] <= s { lo = mid } else { hi = mid } }
      return path[lo].curvature
    }
    let half = transition / 2, n = 8
    let smooth = distance.map { s in
      (0...n).reduce(0.0) { $0 + raw(s - half + transition * Double($1) / Double(n)) } / Double(n + 1)
    }
    var out = [HorizontalPathSample(position: path[0].position, tangent: path[0].tangent, curvature: smooth[0])]
    var heading = atan2(path[0].tangent.y, path[0].tangent.x), position = path[0].position
    for i in 1..<path.count {
      let ds = distance[i] - distance[i - 1]
      let mid = heading + 0.5 * ds * (smooth[i - 1] + smooth[i]) / 2
      position = position + Vector2(cos(mid), sin(mid)) * ds
      heading += ds * (smooth[i - 1] + smooth[i]) / 2
      out.append(.init(position: position, tangent: Vector2(cos(heading), sin(heading)), curvature: smooth[i]))
    }
    let error = end.position - out.last!.position
    return out.enumerated().map { i, p in
      .init(position: p.position + error * (distance[i] / max(1, total)), tangent: p.tangent, curvature: p.curvature)
    }
  }
}
