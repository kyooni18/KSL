import Foundation
import KSLCore

/// Constants and equations are from f1eebdc/control/flight_control.c. Angle
/// arithmetic is radians; dimensionless actuator trim gains are converted
/// explicitly at their call sites. These are control design values, not plant data.
enum ControlLaw {
  static let pitchLag = 0.12, rollLag = 0.12, yawLag = 0.20
  static let pitchWN = 1.05, rollWN = 0.65, yawWN = 1.0
  static let damping = 1.15
  static func lowpass(_ previous: Double, _ value: Double, dt: Double, tau: Double) -> Double {
    previous + (1 - exp(-dt / max(tau, Double.ulpOfOne))) * (value - previous)
  }
  static func demand(error: Double, rate: Double, targetRate: Double = 0,
                     authority: Double, wn: Double, damping: Double,
                     lag: Double, previous: Double, balancingInput: Double = 0) -> Double {
    guard authority > 1e-12 else { return 0 }
    // A steady trim input balances aerodynamic moment; it is not net angular
    // acceleration. C's uncompensated lag predictor otherwise fights trim.
    let applied = (clamp(previous, -1, 1) - balancingInput) * authority
    let relativeRate = rate - targetRate
    let rateAhead = rate + applied * lag
    let errorAhead = error - relativeRate * lag - 0.5 * applied * lag * lag
    return (wn * wn * errorAhead - 2 * damping * wn * (rateAhead - targetRate)) / authority
  }
  static func bandwidth(q: Double, mach: Double, terminal: Bool) -> (pitch: Double, roll: Double) {
    var factor = 0.70 + 0.30 * clamp(q / 8000, 0, 1)
    if q > 35_000 { factor *= max(0.85, 1 - 0.15 * (q - 35_000) / 15_000) }
    let pitchMach = mach > 1.2 && mach < 5 ? 1.20 : mach >= 5 ? 1.05 : 1
    return (pitchWN * factor * pitchMach * (terminal ? 1.10 : 1),
            rollWN * factor * (mach > 2.5 ? 0.85 : 1))
  }
  static func responseScale(_ current: Double, error: Double, rate: Double,
                            wn: Double, dt: Double) -> Double {
    let reference = max(0.10, wn) * max(abs(error), radians(0.35))
    let ratio = abs(rate) / max(reference, radians(0.05))
    let desired = clamp(ratio > 1 ? 1 / sqrt(ratio) : 1, 0.40, 1)
    return clamp(lowpass(current, desired, dt: dt, tau: desired < current ? 0.40 : 3), 0.40, 1)
  }
}
