import Foundation
import KSLCore

struct EntryLateralMemory: Sendable {
  var initialized = false, captured = false
  var sign = 1.0, bank = 0.0, rate = 0.0
  var lastReversal = -Double.infinity
  var previousBeta: Double?, previousCrossRate: Double?
  var reversals = 0
}

/// C entry_lateral.c azimuth/crossrange feedback and bounded double-integrator
/// reference, with measured roll/yaw torque limiting its reversal envelope.
enum EntryLateralController {
  static func exchangeTime(span: Double, rate: Double, acceleration: Double) -> Double {
    guard rate > 0, acceleration > 0 else { return .infinity }
    let span = abs(span), triangular = rate * rate / acceleration
    return span <= triangular ? 2 * sqrt(span / acceleration) : span / rate + rate / acceleration
  }
  static func update(memory m: inout EntryLateralMemory, state s: VehicleState,
                     configuration c: MissionConfiguration, verticalLift: Double,
                     azimuthError: Double, crossError: Double, crossRate: Double,
                     alignment: Double, range: Double, dt: Double,
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
    let timeToGo = range / max(1, s.horizontalSpeed)
    let zeroEffortMiss = crossError + crossRate * timeToGo
    let lateral = timeToGo > 0 && alignment > 0 ? 2 * abs(zeroEffortMiss) / (timeToGo * timeToGo * alignment) : 0
    var magnitude = energyBank
    if hypot(max(0, verticalLift), lateral) > lift {
      magnitude = clamp(atan2(lateral, max(0, verticalLift)), 0, c.vehicle.maximumBank)
    }
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
    let deadband = radians(clamp(3 + a.airspeed / 180, 4, 16))
    var demandedSign = azimuthError < 0 ? -1.0 : 1.0
    let pointBehind = cos(azimuthError) <= 0
    if pointBehind { demandedSign = m.sign }
    else if abs(projected) > e.handoffRadius, alignment > 0 { demandedSign = projected < 0 ? -1 : 1 }
    let requested = demandedSign != m.sign
      && (abs(azimuthError) > deadband || abs(projected) > e.handoffRadius)
    let ready = s.time - m.lastReversal >= max(e.minimumLegDuration, response)
    if requested && feasible && ready && (m.captured || magnitude < radians(5)) {
      m.sign = demandedSign; m.lastReversal = s.time; m.captured = false; m.reversals += 1
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
    if magnitude > 0 && m.sign * a.bank >= 0.9 * magnitude { m.captured = true }
    d.energyBank = energyBank; d.commandedBank = m.bank
    d.reversalTime = response.isFinite ? response : nil; d.reversalAvailableTime = availableTime
    d.reversalArmed = requested; d.reversalFeasible = feasible; d.reversalCount = m.reversals
    d.projectedCrossrange = projected; d.rollRateLimit = rateLimit
    d.limitingReason = protect ? "q/load protection" : acceleration <= 0 ? "no atmospheric moment authority"
      : requested && !feasible ? "insufficient coordinated reversal envelope"
      : requested && !ready ? "reversal dwell" : requested && !m.captured ? "bank leg not captured" : "tracking"
    return m.bank
  }
}
