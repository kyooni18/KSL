import Foundation

/// f1eebdc vehicle/math.c burn_fraction, including the 0.08 m/s cutoff.
public enum BurnThrottleLaw {
  public static let terminalDeltaVTolerance = 0.08
  public static func fraction(elapsed: Double, remaining: Double, maximumAcceleration: Double,
                              ramp: Double) -> Double {
    guard remaining > terminalDeltaVTolerance else { return 0 }
    let x = clamp(elapsed/max(0.5, ramp), 0, 1), transition = max(1, maximumAcceleration*1.25)
    let smooth = x*x*(3-2*x)
    let taper = remaining >= transition ? 1 : max(0.18, sqrt(max(0, remaining)/transition))
    return clamp(min(smooth, taper), 0, 1)
  }
}
