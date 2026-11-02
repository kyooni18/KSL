import Foundation

/// Command slew is part of the flight computer, so every forecast starts from
/// the same last commanded target and uses this exact limiter. Targets are a
/// second-order reference: bounded rate AND bounded acceleration (the plant's
/// available roll/pitch acceleration at the current dynamic pressure), with
/// deceleration ahead of the goal, so the vehicle is never asked to change
/// attitude rate instantaneously.
public struct GuidanceTargetLimiter: Codable, Sendable, Equatable {
  /// Fraction of rated attitude response guidance may demand.
  public static let responseReserve = 0.85
  public private(set) var previous: GuidanceTarget?
  private var bankRate = 0.0, alphaRate = 0.0
  public init(previous: GuidanceTarget? = nil) { self.previous = previous }
  public mutating func update(_ requested: GuidanceTarget, phase: MissionPhase,
                              configuration c: MissionConfiguration, dt: Double,
                              dynamicPressure: Double? = nil, rollResponseSeconds: Double? = nil,
                              pitchResponseSeconds: Double? = nil,
                              controlAuthority: ControlAuthorityTelemetry? = nil) -> GuidanceTarget {
    var target = requested
    // Limit the goal before pricing/slewing it. A command schedule cannot
    // purchase force by aiming beyond the configured incidence/bank envelope.
    target.bank = clamp(target.bank, -c.vehicle.maximumBank, c.vehicle.maximumBank)
    target.angleOfAttack = clamp(target.angleOfAttack,
      c.vehicle.minimumAngleOfAttack, c.vehicle.maximumAngleOfAttack)
    if let old = previous {
      let plant = c.vehicle.controlPlant
      var rollRateLimit = [.approach, .preflare, .flare].contains(phase)
        ? (c.guidance.approachRollRate ?? c.guidance.bankSlewRate)
        : [.acquisition, .headingAlignment].contains(phase)
        ? (c.guidance.terminalRollRate ?? c.entrySettings.rollRate) : c.entrySettings.rollRate
      if [.acquisition, .headingAlignment, .approach, .preflare, .flare].contains(phase) {
        // Transfer attitude over the outer-loop/identified response horizon,
        // not a minimum-time slew to a moving force-allocation corner.
        let response = max(c.guidance.pathLookaheadTime, rollResponseSeconds ?? 0)
        rollRateLimit = min(rollRateLimit, abs(wrapAngle(target.bank-old.bank))/response)
      }
      let pitchRateLimit = phase == .preflare || phase == .flare
        ? plant.maximumPitchRate * 0.8 : c.guidance.alphaSlewRate
      // Use the same active actuator envelope as the FCS. Reaction wheels
      // and enabled RCS do not lose authority when q first becomes positive.
      // A surface-only q prior otherwise freezes the reference at entry while
      // the measured attitude actuators remain available.
      var rollAcceleration = Double.infinity, pitchAcceleration = Double.infinity
      if let authority = controlAuthority, authority.isValid {
        let available = authority.minimumAcceleration
        rollAcceleration = available.x * Self.responseReserve
        pitchAcceleration = available.y * Self.responseReserve
      } else if let q = dynamicPressure, q.isFinite, q >= 0 {
        let kPa = q / 1000, reserve = Self.responseReserve
        rollAcceleration = min(plant.rollAccelerationMaximum, plant.rollAccelerationPerKPa * kPa) * reserve
        pitchAcceleration = min(plant.pitchAccelerationMaximum, plant.pitchAccelerationPerKPa * kPa) * reserve
      }
      if phase == .entry { rollAcceleration = min(rollAcceleration, c.entrySettings.rollAcceleration) }
      // The target feeds a closed attitude loop: changing its rate faster
      // than that loop's response time only injects feedback noise.
      if let t = rollResponseSeconds, t > 0 { rollAcceleration = min(rollAcceleration, rollRateLimit / t) }
      if let t = pitchResponseSeconds, t > 0 { pitchAcceleration = min(pitchAcceleration, pitchRateLimit / t) }
      (target.bank, bankRate) = Self.step(old.bank, toward: old.bank + wrapAngle(target.bank - old.bank),
        rate: bankRate, rateLimit: rollRateLimit, acceleration: rollAcceleration, dt: dt)
      (target.angleOfAttack, alphaRate) = Self.step(old.angleOfAttack, toward: target.angleOfAttack,
        rate: alphaRate, rateLimit: pitchRateLimit, acceleration: pitchAcceleration, dt: dt)
    }
    // Never hand the FCS an attitude outside the vehicle envelope.
    target.bank = clamp(target.bank, -c.vehicle.maximumBank, c.vehicle.maximumBank)
    target.angleOfAttack = clamp(target.angleOfAttack,
      c.vehicle.minimumAngleOfAttack, c.vehicle.maximumAngleOfAttack)
    previous = target
    return target
  }

  /// One step of a rate/acceleration-limited reference that arrives at the
  /// goal with zero rate (minimum-time bang-bang profile).
  static func step(_ value: Double, toward goal: Double, rate: Double, rateLimit: Double,
                   acceleration: Double, dt: Double) -> (Double, Double) {
    guard dt.isFinite, dt > 0, rateLimit.isFinite, rateLimit > 0,
      acceleration > 0 else { return (value, 0) }
    if acceleration == .infinity {
      let next = moveToward(value, goal, maximumDelta: rateLimit * dt)
      return (next, (next - value) / dt)
    }
    guard acceleration.isFinite else { return (value, 0) }
    let error = goal - value, direction: Double = error < 0 ? -1 : 1
    let desired = direction * min(rateLimit, sqrt(2 * acceleration * abs(error)))
    // A reference has no inertia to honour: it may never move away from,
    // or past, its goal, however small the available deceleration is.
    var newRate = moveToward(rate, desired, maximumDelta: acceleration * dt)
    if newRate * direction < 0 { newRate = 0 }
    newRate = direction * min(abs(newRate), abs(error) / max(dt, 1e-9))
    let next = value + newRate * dt
    if (goal - value) * (goal - next) <= 0 { return (goal, 0) }
    return (next, newRate)
  }
}
