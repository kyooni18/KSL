import Foundation
import KSLCore

/// Passive, time-weighted local u = a + b*alpha identification from the
/// historical C adaptive_trim.h. Internally degrees preserve C conditioning.
struct AdaptiveTrim: Sendable {
  private var weight = 0.0, x = 0.0, y = 0.0, xx = 0.0, xy = 0.0, yy = 0.0
  private var low = 0.0, high = 0.0, mach = 0.0, pressure = 0.0
  private var lastTime: Double?
  private var lastIdentifiedTime: Double?
  private var regimeValid = false
  private struct LocalBalance: Sendable {
    let time: Double, mach: Double, pressure: Double, alpha: Double, input: Double
  }
  private var localBalance: LocalBalance?
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
                        alpha: Double, input: Double, settled: Bool, predictionSettled: Bool? = nil) {
    guard time.isFinite else { return }
    let dt = lastTime.map { time - $0 } ?? 0
    lastTime = time
    guard dt > 0, dt <= 2 else {
      weight = 0; regimeValid = false; localBalance = nil; lastIdentifiedTime = nil
      return
    }
    let decay = exp(-dt / 30)
    weight *= decay; x *= decay; y *= decay; xx *= decay; xy *= decay; yy *= decay
    let incidence = degrees(alpha)
    guard [mach, pressure, incidence, input].allSatisfy(\.isFinite),
          pressure > 0, abs(incidence) <= 180, abs(input) <= 1 else { return }
    // A measured equilibrium at the actuator stop still identifies the local
    // balancing input. It does NOT identify a trim slope or the maximum AoA.
    // Keep it separate from the unsaturated regression and control feedforward.
    if predictionSettled ?? settled {
      localBalance = .init(time: time, mach: mach, pressure: pressure, alpha: alpha, input: input)
    }
    // Prediction-only equilibrium does not update the controller's regression,
    // its freshness clock, feedforward, or claimed attainable-incidence ceiling.
    guard settled else { return }
    guard abs(input) < 0.95 else { return }
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
    lastIdentifiedTime = time
  }
  func input(alpha: Double) -> Double? {
    guard weight >= 0.05, alpha.isFinite else { return nil }
    let incidence = degrees(alpha)
    if let fit {
      // A saturated actuator does not justify extrapolating the unsaturated
      // moment-balance regression beyond the measured incidence interval.
      guard incidence >= low && incidence <= high else { return nil }
      return clamp(fit.intercept + fit.slope * incidence, -0.75, 0.75)
    }
    guard abs(incidence - x / weight) <= 1 else { return nil }
    return clamp(y / weight, -0.75, 0.75)
  }
  /// A fit may only provide feedforward in its measured local regime.
  /// An unrelated/saturated observation must not refresh its freshness clock.
  func input(time: Double, mach: Double, pressure: Double, alpha: Double) -> Double? {
    guard regimeValid, let lastIdentifiedTime,
      [time, mach, pressure, alpha].allSatisfy(\.isFinite),
      time >= lastIdentifiedTime, time - lastIdentifiedTime <= 2,
      abs(mach - self.mach) <= 0.35, self.pressure > 0,
      pressure / self.pressure >= 0.5, pressure / self.pressure <= 2 else { return nil }
    return input(alpha: alpha)
  }
  /// Seed a prediction with the observed local moment balance, including
  /// saturation. These are the observer's existing time/regime/local-incidence
  /// support limits. The result is not a future q/Mach moment law or authority.
  func predictionInput(time: Double, mach: Double, pressure: Double, alpha: Double) -> Double? {
    guard let balance = localBalance,
      [time, mach, pressure, alpha].allSatisfy(\.isFinite),
      time >= balance.time, time - balance.time <= 2,
      abs(mach - balance.mach) <= 0.35,
      pressure / balance.pressure >= 0.5, pressure / balance.pressure <= 2 else { return nil }
    if abs(degrees(alpha - balance.alpha)) <= 1 { return balance.input }
    // Saturated samples do not update the unsaturated fit. Its old regime
    // must independently support any fallback away from the local equilibrium.
    return input(time: time, mach: mach, pressure: pressure, alpha: alpha)
  }
  /// An unsaturated local fit alone does not identify the maximum trimmable
  /// AoA. Only consider an upper bound if the *requested* ceiling lies inside
  /// the sampled incidence range. Otherwise report unknown, not a hard limit.
  func ceiling(maximum: Double) -> Double? {
    guard let fit, maximum.isFinite, degrees(maximum) <= high else { return nil }
    let lower = max(0, low), upper = degrees(maximum)
    guard upper >= lower else { return nil }
    let variance = xx / weight - pow(x / weight, 2), mean = x / weight
    var supported: Double?
    for incidence in stride(from: lower, through: upper, by: 0.25) {
      let uncertainty = (fit.noise + 0.03) * sqrt(1 + pow(incidence - mean, 2) / variance)
      if abs(fit.intercept + fit.slope * incidence) + 2 * uncertainty <= 0.75 {
        supported = incidence
      }
    }
    return supported.map { radians($0) }
  }
  func ceiling(time: Double, mach: Double, pressure: Double, maximum: Double) -> Double? {
    guard regimeValid, let lastIdentifiedTime,
      [time, mach, pressure, maximum].allSatisfy(\.isFinite),
      time >= lastIdentifiedTime, time - lastIdentifiedTime <= 2,
      abs(mach - self.mach) <= 0.35, self.pressure > 0,
      pressure / self.pressure >= 0.5, pressure / self.pressure <= 2 else { return nil }
    return ceiling(maximum: maximum)
  }
}
