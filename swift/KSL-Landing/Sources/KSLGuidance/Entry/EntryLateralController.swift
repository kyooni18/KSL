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
    let timeToGo = max(c.guidance.pathLookaheadTime,
      range / max(1, s.horizontalSpeed*abs(alignment)))
    // Cubic endpoint interception: e(T)=0 and eDot(T)=0. crossRate is
    // eDot=-velocityRight, so positive demand is a right-bank correction.
    // Direction and magnitude must come from the same signed acceleration.
    let crossDemand = 6*crossError/(timeToGo*timeToGo)+4*crossRate/timeToGo
    // Bank-right projects onto runway-right with a signed cosine. A
    // reciprocal approach reverses that projection, not the desired miss.
    // Blend to bearing pursuit near perpendicular flight, where inversion
    // of the runway-axis projection becomes singular.
    let pursuit = 2*s.horizontalSpeed*sin(azimuthError)/timeToGo
    let signedLateral = alignment*crossDemand+(1-alignment*alignment)*pursuit
    let lateral = abs(signedLateral)
    // Turn only to correct an actual miss, and only with lift left after
    // vertical recovery. Infeasibility cannot authorize banking away support.
    var magnitude = min(energyBank, atan2(lateral, max(Double.leastNormalMagnitude, verticalLift)))
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
    let requested = demandedSign != m.sign && lateral > 0
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
      : requested && !feasible ? "insufficient coordinated reversal envelope"
      : "tracking"
    return m.bank
  }
}
