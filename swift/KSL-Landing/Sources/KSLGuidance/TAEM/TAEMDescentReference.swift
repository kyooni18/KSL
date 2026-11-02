import Foundation
import KSLCore

/// A runway-relative altitude plan fixed at acquisition/downmode, not a new
/// altitude step every guidance tick. d is remaining ground-track distance.
struct TAEMDescentReference {
  let distance: Double
  let startHeight: Double
  let endHeight: Double
  let startSlope: Double
  let endSlope: Double
  private var a: Double { 2*endHeight-2*startHeight+distance*(endSlope+startSlope) }
  private var b: Double { -3*endHeight+3*startHeight-distance*(2*endSlope+startSlope) }
  func sample(_ remaining: Double) -> (height: Double, slope: Double, curvature: Double) {
    let x=clamp(remaining/distance,0,1)
    return (((a*x+b)*x+distance*endSlope)*x+endHeight,
      (3*a*x*x+2*b*x+distance*endSlope)/distance,
      (6*a*x+2*b)/(distance*distance))
  }
  /// Required upward normal acceleration along this profile. A negative
  /// value is an impossible push-over for the positive-lift TAEM bank law;
  /// it must not be clipped to zero when assessing a candidate.
  func normalAcceleration(at remaining: Double, speedSquared: Double,
                          gravity: Double, radius: Double) -> Double {
    let r = sample(remaining), metric = hypot(1, r.slope)
    let horizontalSquared = speedSquared / (metric * metric)
    return (gravity - horizontalSquared / radius) / metric
      + speedSquared * r.curvature / (metric * metric * metric)
  }
  func fits(maximumGlide: Double) -> Bool {
    // The derivative is quadratic. Endpoints and its one stationary point
    // exactly bound the planned slope; no sampled optimistic path length.
    var points=[0.0,1.0]
    if a != 0 {let x = -b/(3*a);if x>0 && x<1 {points.append(x)}}
    return points.allSatisfy {
      let slope=sample($0*distance).slope
      return slope>=0 && slope<=tan(maximumGlide)+Double.ulpOfOne*16
    }
  }
}
