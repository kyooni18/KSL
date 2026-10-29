import Foundation
import KSLCore

/// A finite C2 capture from the measured height and flight-path angle onto the
/// final reference. The same curve is priced for energy and flown by guidance.
struct ApproachCapture {
  let profile: ApproachProfile
  let start: Double
  let span: Double
  private let coefficients: [Double]

  init(profile: ApproachProfile, along: Double, height: Double, slope: Double) {
    self.profile = profile
    start = along
    span = max(200, min(2500, (profile.preflareAlong - along) * 0.7))
    let reference = profile.reference(along: along)
    let a0 = height - reference.height
    let a1 = span * (slope - reference.slope)
    let a2 = -reference.secondDerivative * span * span / 2
    let b0 = -a0 - a1 - a2
    let b1 = -a1 - 2 * a2
    let b2 = -2 * a2
    coefficients = [
      a0, a1, a2, 10 * b0 - 4 * b1 + b2 / 2, -15 * b0 + 7 * b1 - b2, 6 * b0 - 3 * b1 + b2 / 2,
    ]
  }

  func reference(along: Double) -> VerticalReference {
    let base = profile.reference(along: along)
    guard along < start + span else { return base }
    let u = clamp((along - start) / span, 0, 1)
    let a = coefficients
    var h = a[5]
    var slope = 5 * a[5]
    var second = 20 * a[5]
    for i in stride(from: 4, through: 0, by: -1) { h = h * u + a[i] }
    for i in stride(from: 4, through: 1, by: -1) { slope = slope * u + Double(i) * a[i] }
    for i in stride(from: 4, through: 2, by: -1) {
      second = second * u + Double(i * (i - 1)) * a[i]
    }
    return .init(
      height: base.height + h, slope: base.slope + slope / span,
      secondDerivative: base.secondDerivative + second / (span * span))
  }

  func stations(touchdownAlong: Double) -> [PathStation] {
    let length = touchdownAlong - start
    guard length > 1 else { return [] }
    let count = max(2, Int(ceil(length / 40)))
    return (0...count).map { i in
      let distance = length * Double(i) / Double(count)
      let along = start + distance
      let r = reference(along: along)
      return PathStation(
        distance: distance, position: .init(along, 0), tangent: .init(1, 0), curvature: 0,
        height: r.height, slope: r.slope,
        verticalCurvature: r.secondDerivative / pow(1 + r.slope * r.slope, 1.5), segment: .final)
    }
  }
}
