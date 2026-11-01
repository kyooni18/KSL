import Foundation
import KSLCore

struct EntryLateralMemory: Sendable {
  var initialized = false, captured = false
  var sign = 1.0, bank = 0.0, rate = 0.0
  var lastReversal = -Double.infinity
  var previousBeta: Double?, previousCrossRate: Double?
  var reversals = 0
  /// Crossrange TAEM can still turn out at capture (m); nil before any
  /// capture forecast exists, when the MM305 handoff radius applies.
  var corridor: Double?
}

/// C entry_lateral.c azimuth/crossrange feedback and bounded double-integrator
/// reference, with measured roll/yaw torque limiting its reversal envelope.
enum EntryLateralController {
  static func exchangeTime(span: Double, rate: Double, acceleration: Double) -> Double {
    guard rate > 0, acceleration > 0 else { return .infinity }
    let span = abs(span), triangular = rate * rate / acceleration
    return span <= triangular ? 2 * sqrt(span / acceleration) : span / rate + rate / acceleration
  }
  static func demand(state s: VehicleState, configuration c: MissionConfiguration,
                     azimuthError: Double, crossError: Double, crossRate: Double,
                     alignment: Double, range: Double) -> Double {
    let timeToGo = max(c.guidance.pathLookaheadTime,
      range / max(1, s.horizontalSpeed * abs(alignment)))
    // Bearing pursuit turns with total horizontal speed. Reusing the
    // along-track closure time made its command vanish at perpendicular
    // heading, exactly where the cubic projection is intentionally blended out.
    let pursuitTime = max(c.guidance.pathLookaheadTime,
      hypot(range, crossError) / max(1, s.horizontalSpeed))
    // Cubic interception assumes the aim is ahead. Once it is abeam/behind,
    // that endpoint law can command zero on a reciprocal centerline departure.
    // A rear-hemisphere bearing must retain a turn command, not sin(pi) == 0.
    if cos(azimuthError) <= 0 {
      var error = wrapAngle(azimuthError)
      if abs(abs(error) - .pi) <= sqrt(Double.ulpOfOne) {
        error = s.aerodynamics.bank < 0 ? -.pi : .pi
      }
      return 2 * s.horizontalSpeed * sin(clamp(error, -.pi / 2, .pi / 2)) / pursuitTime
    }
    // Same signed cross-axis intercept law as the orbital window screen.
    // crossRate is minus the physical runway-chart cross velocity.
    let crossDemand = EntryLateralCaptureKinematics.endpointAccelerations(
      crossError: crossError, crossVelocity: -crossRate,
      timeToCapture: timeToGo).initial
    let pursuit = 2 * s.horizontalSpeed * sin(azimuthError) / pursuitTime
    return alignment * crossDemand + (1 - alignment * alignment) * pursuit
  }
  static func update(memory m: inout EntryLateralMemory, state s: VehicleState,
                     configuration c: MissionConfiguration, verticalLift: Double,
                     azimuthError: Double, crossError: Double, crossRate: Double,
                     alignment: Double, range: Double, dt: Double,
                     lateralDemand: Double? = nil, allocatedBank: Double? = nil,
                     diagnostics d: inout EntryDiagnostics) -> Double {
    let a = s.aerodynamics, e = c.entrySettings
    let lift = max(0, a.lift / s.mass)
    if !m.initialized {
      m.initialized = true; m.bank = a.bank
      m.sign = abs(azimuthError) > 1e-12 ? (azimuthError < 0 ? -1 : 1) : (a.bank < 0 ? -1 : 1)
    }
    let kPa = a.dynamicPressure / 1000, p = c.vehicle.controlPlant
    let reported = s.controlAuthority?.minimumAcceleration
    let bodyRoll = reported?.x ?? min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa)
    let bodyYaw = reported?.z ?? min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa)
    // A wind-axis bank exchange needs p*cos(alpha)+r*sin(alpha), while
    // coordinated yaw must also keep p*sin(alpha)-r*cos(alpha) near zero.
    let rollAuthority = bodyRoll / max(abs(cos(a.angleOfAttack)), 1e-12)
    let yawAuthority = bodyYaw / max(abs(sin(a.angleOfAttack)), 1e-12)
    let acceleration = min(e.rollAcceleration, rollAuthority, yawAuthority)
    var rateLimit = min(e.rollRate, p.maximumRollRate)
    if let previous = m.previousBeta, dt > 0 {
      let betaRate = wrapAngle(a.sideslip - previous) / dt
      let outward = (a.sideslip < 0 ? -1.0 : 1.0) * betaRate
      if outward > 0 {
        rateLimit = min(rateLimit, acceleration * max(0, e.maximumSideslip - abs(a.sideslip)) / outward)
      }
    }
    m.previousBeta = a.sideslip
    rateLimit = max(rateLimit, max(0, abs(m.rate) - acceleration * dt))
    let energyBank = lift > 0 ? acos(clamp(verticalLift / lift, cos(c.vehicle.maximumBank), 1)) : 0
    let timeToGo = max(c.guidance.pathLookaheadTime,
      range / max(1, s.horizontalSpeed*abs(alignment)))
    // Direction and magnitude share the same interception/recovery law.
    // crossRate is eDot=-velocityRight; the signed runway projection is
    // retained while the aim is ahead, with explicit recovery when behind.
    let signedLateral = lateralDemand ?? demand(state: s, configuration: c,
      azimuthError: azimuthError, crossError: crossError, crossRate: crossRate,
      alignment: alignment, range: range)
    let lateral = abs(signedLateral)
    // Turn only to correct an actual miss, and only with lift left after
    // vertical recovery. Infeasibility cannot authorize banking away support.
    let force = NormalForceEnvelope(maximumLift: min(lift, c.vehicle.maximumLoadFactor * 9.80665),
      maximumBank: c.vehicle.maximumBank)
    // MM304 has already chosen AoA; its present lift is measured, not another
    // free allocation variable. Invert L*sin(bank), reserving vertical support.
    let commandedLateral = min(lateral, force.lateralAuthority(reserving: max(0, verticalLift)))
    // AoA/bank originate from one modeled force vector. Present measured
    // lift limits the bank until the pitched attitude actually catches up.
    let measuredBank = lift > 0 ? asin(clamp(commandedLateral / lift, 0, 1)) : 0
    var magnitude = allocatedBank.map { min(abs($0), measuredBank) } ?? measuredBank
    d.lateralDemand = signedLateral
    d.lateralAuthority = force.lateralAuthority(reserving: max(0, verticalLift))
    let protect = a.dynamicPressure >= 0.9 * c.vehicle.maximumDynamicPressure
      || lift >= 0.9 * c.vehicle.maximumLoadFactor * 9.80665
    if protect { magnitude = 0 }
    let response = exchangeTime(span: abs(a.bank) + magnitude, rate: rateLimit, acceleration: acceleration)
    let altitude = s.position.length - c.planet.radius - c.runway.elevation
    let availableTime = min(timeToGo, max(0, altitude - 1200) / max(1, -s.verticalSpeed))
    let feasible = lift > 0 && acceleration > 0 && response.isFinite && response < availableTime
      && abs(a.sideslip) < e.maximumSideslip && !protect
    let crossAcceleration = m.previousCrossRate.map { (crossRate - $0) / max(dt, 1e-12) } ?? 0
    m.previousCrossRate = crossRate
    var projected = crossError
    if response.isFinite {
      projected += crossRate * response + 0.5 * crossAcceleration * response * response
      let lateralAuthority = lift * abs(sin(magnitude))
      if lateralAuthority > 0 && alignment > 0 {
        let futureRate = crossRate + crossAcceleration * response
        projected += (futureRate < 0 ? -1 : 1) * futureRate * futureRate / (lateralAuthority * (1 + alignment))
      }
    }
    // Arrival precision is the configured handoff tolerance, not the
    // distance a later HAC might turn out. Necessary steering corrections
    // are rate-shaped below instead of waiting for an arbitrary bank leg.
    let demandedSign = signedLateral < 0 ? -1.0 : 1.0
    // USA005512 section 2.14: entry rolls are reversed to contain DELAZ,
    // not to create an S-turn for drag. First reversal is at 10.5 degrees;
    // subsequent reversals use 17.5 degrees, tapered to 10 below Mach 4.
    // A large measured crossrange miss takes precedence over that nominal
    // corridor; a reversal to correct it must not wait for the angle gate.
    let reversalCorridor = m.reversals == 0 ? radians(10.5)
      : radians(10 + 7.5*clamp(a.mach-3, 0, 1))
    let outsideCaptureLateral = abs(crossError) > e.handoffRadius
    let requested = demandedSign != m.sign && lateral > 0 &&
      (outsideCaptureLateral || abs(azimuthError) >= reversalCorridor)
    if requested {
      m.sign = demandedSign; m.lastReversal = s.time
      m.captured = false; m.reversals += 1
    }
    let target = m.sign * magnitude
    if acceleration > 0 && rateLimit > 0 {
      let error = target - m.bank, direction = error < 0 ? -1.0 : 1.0
      let lead = direction * (m.bank - a.bank)
      let reserve = rateLimit * rateLimit / (2 * acceleration)
      let desiredRate = direction * min(rateLimit, sqrt(2 * acceleration * abs(error)),
        sqrt(2 * acceleration * max(0, reserve - max(0, lead))))
      m.rate = moveToward(m.rate, desiredRate, maximumDelta: acceleration * dt)
      let next = m.bank + m.rate * dt
      if (target - m.bank) * (target - next) <= 0 { m.bank = target; m.rate = 0 }
      else { m.bank = next }
    }
    m.bank = clamp(m.bank, -c.vehicle.maximumBank, c.vehicle.maximumBank)
    if abs(m.bank) >= c.vehicle.maximumBank && m.bank*m.rate > 0 { m.rate = 0 }
    if magnitude > 0 && m.sign * a.bank >= 0.9 * magnitude { m.captured = true }
    d.energyBank = energyBank; d.commandedBank = m.bank
    d.reversalTime = response.isFinite ? response : nil; d.reversalAvailableTime = availableTime
    d.reversalArmed = requested; d.reversalFeasible = feasible; d.reversalCount = m.reversals
    d.projectedCrossrange = projected; d.rollRateLimit = rateLimit
    d.limitingReason = protect ? "q/load protection" : acceleration <= 0 ? "no atmospheric moment authority"
      : verticalLift > lift ? "vertical force deficit"
      : lateral > (d.lateralAuthority ?? 0) ? "lateral force deficit"
      : requested && !feasible ? "insufficient coordinated reversal envelope"
      : "tracking"
    return m.bank
  }
}
