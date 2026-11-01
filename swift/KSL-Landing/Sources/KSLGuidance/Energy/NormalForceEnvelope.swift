import Foundation
import KSLCore

/// Feasible positive-lift vectors in wind-normal coordinates. This is force
/// allocation, not a new aerodynamic model: callers supply the existing
/// polar's lift bound and the configured bank/load limits.
struct NormalForceEnvelope {
  let maximumLift: Double
  let maximumBank: Double

  struct Allocation {
    let bank: Double
    let lift: Double
    let vertical: Double
    let lateral: Double
  }

  /// Preserve the requested positive normal support first, then project the
  /// requested turn into the remaining force cone. Extra vertical lift is
  /// permitted when the bank limit prevents realizing a shallow force vector.
  func allocate(vertical: Double, lateral: Double) -> Allocation {
    let limit = max(0, maximumLift)
    guard limit > 0 else { return .init(bank: 0, lift: 0, vertical: 0, lateral: 0) }
    let reserve = min(limit, max(0, vertical))
    let bankLimit = min(maximumBank, acos(clamp(reserve / limit, 0, 1)))
    let bank = clamp(atan2(lateral, max(0, vertical)), -bankLimit, bankLimit)
    let cosine = cos(bank)
    let minimum = reserve / max(Double.leastNormalMagnitude, cosine)
    let projected = max(0, vertical) * cosine + lateral * sin(bank)
    let lift = clamp(projected, minimum, limit)
    return .init(bank: bank, lift: lift, vertical: lift * cosine, lateral: lift * sin(bank))
  }

  func lateralAuthority(reserving vertical: Double) -> Double {
    let limit = max(0, maximumLift)
    guard limit > max(0, vertical) else { return 0 }
    let bank = min(maximumBank, acos(clamp(max(0, vertical) / limit, 0, 1)))
    return limit * sin(bank)
  }
}
