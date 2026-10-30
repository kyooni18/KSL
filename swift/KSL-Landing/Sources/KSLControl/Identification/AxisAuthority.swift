import Foundation
import KSLCore

/// C's FlightControlAxisAuthority, in radians. Body torque/inertia and the
/// observed acceleration of the controlled coordinate remain separate.
struct AxisAuthority: Sendable {
  var reported = 0.0
  var controlled = 0.0
  var confidence = 0.0
  var liveConfidence = 0.0
  var drift = 0.0
  private var lastRate: Double?

  mutating func observe(rate: Double, input: Double, dt: Double,
                        interference: Double, responseTime: Double) {
    defer { lastRate = rate }
    guard let lastRate, dt > 0, dt <= 1, rate.isFinite else { return }
    let acceleration = (rate - lastRate) / dt
    guard acceleration.isFinite else { return }
    let magnitude = abs(input)
    if magnitude <= Double.ulpOfOne { drift = acceleration; return }
    var weight = magnitude / (magnitude + abs(interference) + Double.ulpOfOne)
    if responseTime > 0 { weight *= min(1, magnitude) * (1 - exp(-dt / responseTime)) }
    let observed = (acceleration - drift) / input
    guard observed.isFinite, observed > 0 else { return }
    controlled = confidence > 0 ? controlled + weight * (observed - controlled) : observed
    liveConfidence = 1 - (1 - liveConfidence) * (1 - weight)
    confidence = 1 - (1 - confidence) * (1 - weight)
  }

  func available(fallback: Double, blendLive: Bool, reportedAvailable: Bool = false) -> Double {
    // Zero is a measured physical constraint, not a missing measurement.
    // Never resurrect a previous high-q fit after the actuator loses authority.
    if reportedAvailable && reported <= 0 { return 0 }
    let observed = confidence > 0 && controlled > 0 ? controlled : fallback
    guard reported > 0 else { return max(0, observed) }
    guard blendLive, observed > 0 else { return reported }
    return 1 / ((1 - liveConfidence) / reported + liveConfidence / observed)
  }
}

/// Regression of beta-dot against body yaw/roll rates. The initial coefficients
/// follow the wind/body kinematic identity, not a fitted force coefficient.
/// This is diagnostic identification; it never invents missing actuator torque.
struct SideslipResponseModel: Sendable {
  private var yy = 0.0, rr = 0.0, yr = 0.0, by = 0.0, br = 0.0
  private var meanYaw = 0.0, meanRoll = 0.0, meanBetaRate = 0.0
  private var count = 0
  private(set) var yawGain = 0.0, rollGain = 0.0, confidence = 0.0
  mutating func observe(yaw: Double, roll: Double, betaRate: Double) {
    guard [yaw, roll, betaRate].allSatisfy(\.isFinite) else { return }
    count += 1
    let n = Double(count)
    let dy = yaw - meanYaw, dr = roll - meanRoll, db = betaRate - meanBetaRate
    meanYaw += dy / n; meanRoll += dr / n; meanBetaRate += db / n
    yy += dy * (yaw - meanYaw); rr += dr * (roll - meanRoll)
    yr += dy * (roll - meanRoll); by += db * (yaw - meanYaw)
    br += db * (roll - meanRoll)
    let determinant = yy * rr - yr * yr
    guard count >= 3, determinant > Double.ulpOfOne * max(1, yy * rr) else { return }
    yawGain = (by * rr - br * yr) / determinant
    rollGain = (br * yy - by * yr) / determinant
    confidence = clamp(determinant / max(yy * rr, Double.ulpOfOne), 0, 1)
  }
}
