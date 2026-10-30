import Foundation
import KSLCore
import KSLPhysics

public final class EnergyManagementEngine {
  /// Fraction of rated roll response a path may demand (pricing reserve).
  public static let bankRateReserve = GuidanceTargetLimiter.responseReserve
  /// Lowest priced touchdown speed, as a fraction of landing speed.
  public static let minimumTouchdownFraction = 0.82
  /// Bank whose load factor remains available after the configured
  /// aerodynamic uncertainty: n = min(n_max, 1/cos(bank_max))*(1-u).
  public static func acquisitionBank(_ c: MissionConfiguration) -> Double {
    let v = c.vehicle
    let n = min(v.maximumLoadFactor, 1 / cos(v.maximumBank)) * (1 - v.aerodynamicUncertainty)
    return n > 1 ? acos(1 / n) : 0
  }
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aero: AerodynamicsEngine
  private let rotation: RotationalDynamicsEngine
  private let frame: RunwayFrame
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere
    aero = aerodynamics
    rotation = .init(configuration: configuration.vehicle)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func energyHeight(_ state: VehicleState) -> Double {
    let r = state.position.length
    let rr = configuration.planet.radius + configuration.runway.elevation
    let g = configuration.planet.gravitationalParameter / (rr * rr)
    return
      (configuration.planet.gravitationalParameter * (1 / rr - 1 / r) + state.velocity.squaredLength
      / 2) / g
  }
  /// Energy-consistent TAEM vertical profile along a horizontal path. Heights
  /// follow a dynamic-pressure schedule (descend as speed bleeds so lift can
  /// still support the banked turn), slope changes are limited by vertical
  /// push-over/pull-up authority (dm/ds <= a/V^2), the glide envelope is respected,
  /// and the last segment blends into the final-approach exit. The result is
  /// only a candidate: `evaluate` re-prices it independently.
  public func scheduledProfile(
    distances: [Double], horizontal: [HorizontalPathSample], from state: VehicleState,
    exitHeight h1: Double, exitSlope m1: Double
  ) -> [(height: Double, slope: Double, verticalCurvature: Double)]? {
    guard distances.count == horizontal.count, distances.count > 2, let length = distances.last, length > 0
    else { return nil }
    let v = configuration.vehicle, gc = configuration.guidance, g0 = 9.80665
    let p0 = frame.coordinates(of: state.position)
    // Log-density table at the path start (the atmosphere is horizontally
    // smooth over a terminal-area footprint).
    let top = max(p0.height + 2000, 1000.0), step = 250.0
    let heights = Array(stride(from: 0.0, through: top, by: step))
    let logDensity = heights.map { log(max(1e-9, atmosphere.sample(
      position: frame.position(along: p0.along, cross: p0.cross, height: $0), time: state.time).density)) }
    func height(forDensity rho: Double) -> Double {
      let target = log(max(1e-9, rho))
      if target >= logDensity[0] { return 0 }
      for i in 1..<heights.count where logDensity[i] <= target {
        let f = (logDensity[i - 1] - target) / max(1e-12, logDensity[i - 1] - logDensity[i])
        return heights[i - 1] + f * step
      }
      return top
    }
    let maxGlide = gc.taemGlideAngle ?? gc.outerGlideAngle
    // Same geometric envelope the pricer enforces; authority limits the rest.
    let maxDescent = tan(maxGlide), maxClimb = tan(maxGlide)
    let u = v.aerodynamicUncertainty
    let turnLoad = 1 / cos(Self.acquisitionBank(configuration))
    let q0 = max(1, state.aerodynamics.dynamicPressure)
    var h = p0.height, m = clamp(tan(state.flightPathAngle), -maxDescent, maxClimb)
    var speed = max(1, state.aerodynamics.airspeed)
    var out: [(Double, Double, Double)] = []
    var blendStart: Int?
    for i in distances.indices {
      out.append((h, m, 0))
      if i + 1 == distances.count { blendStart = i; break }
      let ds = distances[i + 1] - distances[i]
      let s = horizontal[i]
      let pos = frame.position(along: s.position.x, cross: s.position.y, height: h)
      let atm = atmosphere.sample(position: pos, time: state.time)
      let q = 0.5 * atm.density * speed * speed, mach = speed / atm.speedOfSound
      let gamma = atan(m), vh = speed * cos(gamma)
      let lateral = vh * vh * s.curvature
      let g = configuration.planet.gravitationalParameter / pos.squaredLength
      let vertical = (g - vh * vh / pos.length) * cos(gamma)
      let required = hypot(vertical, lateral)
      let unit = aero.liftSolution(acceleration: 0, mass: state.mass, dynamicPressure: q, mach: mach,
        minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: v.maximumAngleOfAttack)
      let minimumLift = max(0, aero.coefficients(mach: mach, alpha: v.minimumAngleOfAttack,
        dynamicPressure: q, gear: false).lift * q * v.referenceArea / state.mass)
      // Vertical authority from the aerodynamics: unload toward (never past)
      // minimum positive lift; pull up within uncertain lift and load limits.
      let push = max(0, required - minimumLift)
      let pull = max(0, min(unit.maximumAcceleration * (1 - u), v.maximumLoadFactor * g0) - required)
      // Lowest q at which uncertain maximum lift still flies the acquisition bank.
      let qFloor = unit.maximumAcceleration > 0
        ? q * turnLoad * g0 / (unit.maximumAcceleration * (1 - u)) : v.maximumDynamicPressure
      let qReference = min(v.maximumDynamicPressure, max(q0, qFloor))
      let remaining = length - distances[i]
      // Start the exit blend once rotating onto the exit slope needs the
      // remaining distance at the available pull-up authority.
      if remaining <= max(gc.finalDistance, speed * speed * abs(m - m1) / max(0.1, pull)) {
        blendStart = i; break
      }
      let target = clamp(height(forDensity: 2 * qReference / (speed * speed)),
        h1 + remaining * tan(gc.shallowGlideAngle), h1 + remaining * maxDescent)
      // Minimum-time approach to the schedule under the vertical authority.
      let error = target - h
      let desired = clamp((error < 0 ? -1 : 1) * sqrt(2 * (error < 0 ? push : pull) * abs(error)) / speed,
        -maxDescent, maxClimb)
      let inverseSpeed = ds / (speed * speed)
      m += clamp(desired - m, -push * inverseSpeed, pull * inverseSpeed)
      h += m * ds
      // Speed along the path with the same point-mass drag model as pricing.
      let lift = aero.liftSolution(acceleration: required, mass: state.mass, dynamicPressure: q, mach: mach,
        minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: v.maximumAngleOfAttack)
      let cd = aero.coefficients(mach: mach, alpha: lift.alpha, dynamicPressure: q, gear: false).drag
      let work = q * v.referenceArea * cd / state.mass * ds / max(0.2, cos(gamma))
      let potential = g * m * ds
      let next = speed * speed - 2 * (work + potential)
      if next < 30 * 30 { return nil }
      speed = sqrt(next)
    }
    guard let b = blendStart else { return nil }
    // Hermite from the scheduled state to the final-approach exit.
    let hb = out[b].0, mb = out[b].1, l = length - distances[b]
    guard l > 1 else { return out.map { (height: $0.0, slope: $0.1, verticalCurvature: $0.2) } }
    let ca = 2 * hb - 2 * h1 + l * (mb + m1), cb = -3 * hb + 3 * h1 - l * (2 * mb + m1), cc = l * mb
    var result = out.prefix(b).map { (height: $0.0, slope: $0.1, verticalCurvature: 0.0) }
    for i in b..<distances.count {
      let x = (distances[i] - distances[b]) / l
      let hh = ((ca * x + cb) * x + cc) * x + hb
      let slope = (3 * ca * x * x + 2 * cb * x + cc) / l
      let second = (6 * ca * x + 2 * cb) / (l * l)
      result.append((height: hh, slope: slope, verticalCurvature: second / pow(1 + slope * slope, 1.5)))
    }
    // Scheduled-segment curvature from the slope sequence.
    for i in 1..<max(1, b) {
      let ds = max(1, distances[i + 1] - distances[i - 1])
      let k = (result[i + 1].slope - result[i - 1].slope) / ds
      result[i].verticalCurvature = k / pow(1 + result[i].slope * result[i].slope, 1.5)
    }
    return result
  }
  public func evaluate(
    stations: [PathStation], from initial: VehicleState, dragMultiplier: Double = 1,
    liftMultiplier: Double = 1
  ) -> ([PathStation], TerminalAssessment) {
    var result: [PathStation] = []
    var violations: [ConstraintViolation] = []
    var speed = max(1, initial.aerodynamics.airspeed)
    var time = initial.time
    var maxBank = 0.0
    var maxQ = 0.0
    var maxLoad = 0.0
    var minLift = Double.infinity
    var previousBank = initial.aerodynamics.bank
    let v = configuration.vehicle
    func finish(_ terminalSpeed: Double) -> TerminalAssessment {
      let penalty = Double(violations.count) * 1e6
      let score =
        penalty + pow(terminalSpeed - v.landingSpeed, 2) + (stations.last?.distance ?? 0) * 0.0001
      return .init(
        feasible: violations.isEmpty, violations: violations, touchdownSpeed: terminalSpeed,
        minimumLiftMargin: minLift.isFinite ? minLift : 0, maximumBank: maxBank, maximumQ: maxQ,
        maximumLoad: maxLoad,
        score: score, uncertaintyFraction: v.aerodynamicUncertainty)
    }
    for i in stations.indices {
      let s = stations[i]
      let pos = frame.position(along: s.position.x, cross: s.position.y, height: s.height)
      let atm = atmosphere.sample(position: pos, time: time)
      let gamma = atan(s.slope)
      let q = 0.5 * atm.density * speed * speed
      let g = configuration.planet.gravitationalParameter / pow(pos.length, 2)
      let vh = speed * cos(gamma)
      let vertical =
        (g - vh * vh / pos.length) * cos(gamma) + speed * speed * s.verticalCurvature
      let lateral = vh * vh * s.curvature
      let bank = atan2(lateral, max(0.1, vertical))
      let required = hypot(vertical, lateral)
      let supportedPrior = initial.controlAuthority?.evidence == .provisional
        && speed/atm.speedOfSound <= 0.9 && (500...12000).contains(q)
      let maxAlpha = min(
        supportedPrior ? rotation.maximumTrimAlpha(dynamicPressure: q) : v.maximumAngleOfAttack,
        s.height < 500 ? v.maximumLandingAngleOfAttack : v.maximumAngleOfAttack)
      let lift = aero.liftSolution(
        acceleration: required / max(0.01, liftMultiplier), mass: initial.mass, dynamicPressure: q,
        mach: speed / atm.speedOfSound, minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: maxAlpha
      )
      let load = required / 9.80665
      maxBank = max(maxBank, abs(bank))
      maxQ = max(maxQ, q)
      maxLoad = max(maxLoad, load)
      minLift = min(minLift, lift.maximumAcceleration * liftMultiplier - required)
      if !lift.achievable {
        violations.append(
          .init(
            .insufficientLift, "Path demands unavailable lift/trim authority", value: required,
            limit: lift.maximumAcceleration * liftMultiplier))
      }
      if abs(bank) > v.maximumBank {
        violations.append(
          .init(
            .excessiveBank, "Path curvature exceeds bank envelope", value: abs(bank),
            limit: v.maximumBank))
      }
      if q > v.maximumDynamicPressure {
        violations.append(
          .init(
            .excessiveDynamicPressure, "Path q exceeds envelope", value: q,
            limit: v.maximumDynamicPressure))
      }
      if load > v.maximumLoadFactor {
        violations.append(
          .init(
            .excessiveLoad, "Path normal load exceeds envelope", value: load,
            limit: v.maximumLoadFactor))
      }
      if s.height < v.gearHeight - 0.1 {
        violations.append(.init(.terrainCollision, "Vertical path penetrates runway plane"))
      }
      if s.segment != .final, let maximumGlide = configuration.guidance.taemGlideAngle,
        -gamma > max(maximumGlide, configuration.guidance.outerGlideAngle) {
        violations.append(.init(.invalidGeometry, "Terminal path exceeds configured glide-slope envelope",
          value: -gamma, limit: max(maximumGlide, configuration.guidance.outerGlideAngle)))
      }
      result.append(
        .init(
          distance: s.distance, position: s.position, tangent: s.tangent, curvature: s.curvature,
          height: s.height, slope: s.slope, verticalCurvature: s.verticalCurvature, speed: speed,
          angleOfAttack: lift.alpha, bank: bank, segment: s.segment))
      if !violations.isEmpty { return (result, finish(speed)) }
      if i + 1 < stations.count {
        let next = stations[i + 1]
        let ds = next.distance - s.distance
        let dt = ds / max(5, vh)
        if i > 1
          && abs(wrapAngle(bank - previousBank)) / max(dt, 0.01)
            > min(v.controlPlant.maximumRollRate,
              (s.segment == .final ? configuration.guidance.approachRollRate : configuration.guidance.terminalRollRate)
                ?? configuration.guidance.bankSlewRate) * Self.bankRateReserve
        {
          violations.append(
            .init(.actuatorAuthority, "Path bank-rate demand exceeds identified response"))
          return (result, finish(speed))
        }
        let coefficient = aero.coefficients(mach: speed / atm.speedOfSound, alpha: lift.alpha,
          dynamicPressure: q, gear: s.segment == .final)
        let drag = q * v.referenceArea * coefficient.drag * dragMultiplier / initial.mass
        let work = drag * ds / max(0.2, cos(gamma))
        let potential =
          configuration.planet.gravitationalParameter
          * (1 / pos.length - 1
            / (configuration.planet.radius + configuration.runway.elevation + next.height))
        let nextSquared = speed * speed - 2 * (work + potential)
        if nextSquared < 30 * 30 {
          violations.append(
            .init(
              .insufficientEnergy, "Path exhausts kinetic energy", value: nextSquared, limit: 900))
          return (result, finish(0))
        }
        speed = sqrt(nextSquared)
        time += dt
        previousBank = bank
      }
    }
    if speed < v.landingSpeed * Self.minimumTouchdownFraction {
      violations.append(
        .init(
          .insufficientEnergy, "Touchdown speed below validated design envelope", value: speed,
          limit: v.landingSpeed * Self.minimumTouchdownFraction))
    }
    if speed > v.maximumTouchdownSpeed {
      violations.append(
        .init(
          .excessiveEnergy, "Touchdown speed/rollout distance too high", value: speed,
          limit: v.maximumTouchdownSpeed))
    }
    let stopDistance = speed * speed / (2 * max(0.1, v.brakeFriction * 9.80665))
    if configuration.guidance.touchdownAlong + stopDistance > configuration.runway.length - 100 {
      violations.append(
        .init(
          .excessiveEnergy, "Insufficient runway braking distance", value: stopDistance,
          limit: configuration.runway.length - configuration.guidance.touchdownAlong - 100))
    }
    return (result, finish(speed))
  }
}
