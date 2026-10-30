import Foundation
import KSLCore

/// Passive, time-weighted local u = a + b*alpha identification from the
/// historical C adaptive_trim.h. Internally degrees preserve C conditioning.
struct AdaptiveTrim: Sendable {
  private var weight = 0.0, x = 0.0, y = 0.0, xx = 0.0, xy = 0.0, yy = 0.0
  private var low = 0.0, high = 0.0, mach = 0.0, pressure = 0.0
  private var lastTime: Double?
  private var regimeValid = false
  var fit: (intercept: Double, slope: Double, noise: Double)? {
    guard weight >= 2, high - low >= 3 else { return nil }
    let variance = xx / weight - pow(x / weight, 2)
    guard variance >= 1 else { return nil }
    let b = (xy / weight - x * y / (weight * weight)) / variance
    let a = y / weight - b * x / weight
    let residual = yy / weight - 2 * a * y / weight - 2 * b * xy / weight
      + a * a + 2 * a * b * x / weight + b * b * xx / weight
    let noise = sqrt(max(0, residual))
    guard [a, b, noise].allSatisfy(\.isFinite), noise < 0.1 else { return nil }
    return (a, b, noise)
  }
  mutating func observe(time: Double, mach: Double, pressure: Double,
                        alpha: Double, input: Double, settled: Bool) {
    guard time.isFinite else { return }
    let dt = lastTime.map { time - $0 } ?? 0
    lastTime = time
    guard dt > 0, dt <= 2 else { weight = 0; regimeValid = false; return }
    let decay = exp(-dt / 30)
    weight *= decay; x *= decay; y *= decay; xx *= decay; xy *= decay; yy *= decay
    let incidence = degrees(alpha)
    guard settled, [mach, pressure, incidence, input].allSatisfy(\.isFinite),
          pressure > 0, abs(incidence) <= 180, abs(input) < 0.95 else { return }
    if regimeValid && (abs(mach - self.mach) > 0.35 || pressure / self.pressure > 2 || pressure / self.pressure < 0.5) {
      weight = 0; regimeValid = false
    }
    if !regimeValid || weight < 0.05 {
      weight = 0; x = 0; y = 0; xx = 0; xy = 0; yy = 0
      low = incidence; high = incidence; self.mach = mach; self.pressure = pressure; regimeValid = true
    }
    weight += dt; x += dt * incidence; y += dt * input
    xx += dt * incidence * incidence; xy += dt * incidence * input; yy += dt * input * input
    low = min(low, incidence); high = max(high, incidence)
  }
  func input(alpha: Double) -> Double? {
    guard weight >= 0.05, alpha.isFinite else { return nil }
    let incidence = degrees(alpha)
    if let fit { return clamp(fit.intercept + fit.slope * incidence, -0.75, 0.75) }
    guard abs(incidence - x / weight) <= 1 else { return nil }
    return clamp(y / weight, -0.75, 0.75)
  }
  func ceiling(initial: Double, maximum: Double) -> Double {
    guard let fit else { return min(maximum, max(0, initial)) }
    let upper = min(degrees(maximum), high + high - low)
    let variance = xx / weight - pow(x / weight, 2), mean = x / weight
    var ceiling = 0.0
    for incidence in stride(from: 0.0, through: upper, by: 0.25) {
      let uncertainty = (fit.noise + 0.03) * sqrt(1 + pow(incidence - mean, 2) / variance)
      if abs(fit.intercept + fit.slope * incidence) + 2 * uncertainty <= 0.75 { ceiling = incidence }
    }
    return min(maximum, radians(max(0.25, ceiling)))
  }
}
