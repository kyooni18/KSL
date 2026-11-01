import Foundation

/// Select a stock rails-warp slot from the remaining time to the 1x return.
/// The index is a REQUEST, not a guarantee: the live KSP altitude policy may
/// clamp it. This controller never changes the physics-warp factor.
public enum OrbitalRailsWarpSchedule {
  private static let stockRates: [Double] = [1, 5, 10, 50, 100, 1_000, 10_000, 100_000]

  public static func nominalRate(_ factor: UInt64) -> Double {
    stockRates[Int(min(7, factor))]
  }

  /// KSP may reject an unavailable high rate with factor zero rather than
  /// returning a lower accepted rate. Bound the fallback to stock slots.
  public static func nextLowerAfterRejection(_ factor: UInt64) -> UInt64? {
    guard factor > 1 && factor <= 7 else { return nil }
    return factor - 1
  }

  /// A rate is permitted only if a slow RPC / Unity tick cannot consume the
  /// remaining window in one jump. The safety time increases with the warp
  /// rate because very fast warp creates greater scheduling uncertainty.
  public static func factor(remainingToStop remaining: Double) -> UInt64 {
    guard remaining.isFinite, remaining > 2 else { return 0 }
    for index in stride(from: 7, through: 1, by: -1) {
      let nominal = stockRates[index]
      let guardWallSeconds: Double =
        nominal >= 1000 ? 8 : nominal >= 100 ? 6 : nominal >= 50 ? 4 : 2
      if remaining > nominal * guardWallSeconds { return UInt64(index) }
    }
    return 0
  }

  /// Bound the polling interval in wall-clock time. A newly available
  /// un-clamped rate must not cause one poll to overshoot the target UT.
  public static func pollIntervalMillis(remainingToStop remaining: Double,
                                         requestedFactor: UInt64) -> Int {
    let nominal = nominalRate(requestedFactor)
    guard remaining.isFinite, remaining > 0 else { return 40 }
    return Int(max(40, min(350, 1000 * remaining / (nominal * 20))))
  }
}
