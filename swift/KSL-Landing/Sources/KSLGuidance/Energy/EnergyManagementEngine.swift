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
  private let speedbrake: TAEMSpeedbrakeController
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere
    aero = aerodynamics
    rotation = .init(configuration: configuration.vehicle)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    speedbrake = .init(configuration: configuration, atmosphere: atmosphere)
  }
  /// Planet-fixed mechanical energy relative to the runway, normalized by
  /// runway gravitational acceleration to give an equivalent energy height.
  /// The centrifugal term belongs to the rotating effective potential;
  /// including it keeps terminal pricing consistent with MM304's true work
  /// equation, eDot = -dragAcceleration*airspeed in co-rotating still air.
  public func energyHeight(_ state: VehicleState) -> Double {
    RunwayEnergy(configuration: configuration).height(state)
  }
  /// Energy-consistent TAEM vertical profile along a horizontal path. Heights
  /// follow a dynamic-pressure schedule (descend as speed bleeds so lift can
  /// still support the banked turn), slope changes are limited by vertical
  /// push-over/pull-up authority (dm/ds <= a/V^2), the glide envelope is respected,
  /// and the last segment blends into the final-approach exit. The result is
  /// only a candidate: `evaluate` re-prices it independently.
  public func scheduledProfile(
    distances: [Double], horizontal: [HorizontalPathSample], from state: VehicleState,
    exitHeight h1: Double, exitSlope m1: Double, exitDynamicPressure: Double? = nil
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
    let q0 = max(0, state.aerodynamics.dynamicPressure)
    var h = p0.height, m = clamp(tan(state.flightPathAngle), -maxDescent, maxClimb)
    guard state.aerodynamics.airspeed.isFinite, state.aerodynamics.airspeed > 0 else { return nil }
    var speed = state.aerodynamics.airspeed
    var gearDown = state.systems.gearDown
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
      gearDown = gearDown || h <= (gc.gearDeploymentAltitude ?? 1500)
      let gamma = atan(m), vh = speed * cos(gamma)
      let lateral = vh * vh * s.curvature
      let g = configuration.planet.gravitationalParameter / pos.squaredLength
      let vertical = (g - speed * speed / pos.length) * cos(gamma)
      let required = hypot(vertical, lateral)
      let unit = aero.liftSolution(acceleration: 0, mass: state.mass, dynamicPressure: q, mach: mach,
        minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: v.maximumAngleOfAttack, gear: gearDown)
      let minimumLift = max(0, aero.coefficients(mach: mach, alpha: v.minimumAngleOfAttack,
        dynamicPressure: q, gear: gearDown).lift * q * aero.referenceArea / state.mass)
      // Vertical and lateral acceleration spend the same lift vector.
      // A turn cannot retain lateral force while unloading total lift to zero.
      let maximumLift = min(unit.maximumAcceleration * (1 - u), v.maximumLoadFactor * g0)
      let minimumVertical = max(minimumLift * cos(v.maximumBank),
        abs(lateral) / tan(v.maximumBank))
      let maximumVertical = sqrt(max(0, maximumLift * maximumLift - lateral * lateral))
      guard minimumVertical <= maximumVertical else { return nil }
      let push = max(0, vertical - minimumVertical)
      let pull = max(0, maximumVertical - vertical)
      // Lowest q at which uncertain maximum lift still flies the acquisition bank.
      let qFloor = unit.maximumAcceleration > 0
        ? q * turnLoad * g0 / (unit.maximumAcceleration * (1 - u)) : v.maximumDynamicPressure
      let remaining = length - distances[i]
      // Plan speed to the solved NEP: ramp q from its measured value to the
      // NEP's along the path. Holding the inlet q all the way down made the
      // schedule bleed airspeed as density rose (2244: ~3 kPa at 11.5 km
      // implies ~75 m/s near the NEP, against a solved 124 m/s).
      let qPlan = exitDynamicPressure.map { q0 + ($0 - q0) * distances[i] / length } ?? q0
      let qReference = min(v.maximumDynamicPressure, max(qPlan, qFloor))
      // Start the exit blend once rotating onto the exit slope needs the
      // remaining distance at the available pull-up authority.
      if pull > 0 && remaining <= max(gc.finalDistance, speed * speed * abs(m - m1) / pull) {
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
        minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: v.maximumAngleOfAttack, gear: gearDown)
      let cd = aero.coefficients(mach: mach, alpha: lift.alpha, dynamicPressure: q, gear: gearDown).drag
      guard ds > 0, cos(gamma) > 0 else { return nil }
      let work = q * aero.referenceArea * cd / state.mass * ds / cos(gamma)
      let potential = g * m * ds
      let next = speed * speed - 2 * (work + potential)
      if !next.isFinite || next <= 0 { return nil }
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
    liftMultiplier: Double = 1, advisoryOnly: Bool = false, trimAlphaCeiling: Double? = nil,
    nominalEnergyPoint: NominalEnergyPoint? = nil
  ) -> ([PathStation], TerminalAssessment) {
    var result: [PathStation] = []
    var violations: [ConstraintViolation] = []
    var speed = initial.aerodynamics.airspeed
    var time = initial.time
    var maxBank = 0.0, maxQ = 0.0, maxLoad = 0.0
    var minLift = Double.infinity
    var previousBank = initial.aerodynamics.bank
    var previousAlpha = initial.aerodynamics.angleOfAttack
    var previousStationTime = initial.time
    var gearDown = initial.systems.gearDown
    var exhaustion: Double?
    let v = configuration.vehicle
    func finish(_ terminalSpeed: Double, complete: Bool = false) -> TerminalAssessment {
      let total = stations.last?.distance ?? 0
      let priced = result.last?.distance ?? 0
      var assessment = TerminalAssessment(
        feasible: complete && violations.isEmpty, violations: violations, touchdownSpeed: terminalSpeed,
        minimumLiftMargin: minLift.isFinite ? minLift : 0, maximumBank: maxBank, maximumQ: maxQ,
        maximumLoad: maxLoad, score: Double(Set(violations.map { $0.kind.rawValue }).count) * 1e6
          + pow(terminalSpeed - v.landingSpeed, 2) + total * 0.0001,
        uncertaintyFraction: v.aerodynamicUncertainty)
      assessment.predictionComplete = complete
      assessment.predictedThroughDistance = priced
      assessment.energyExhaustionDistance = exhaustion
      assessment.unpricedDistance = max(0, total - priced)
      return assessment
    }
    guard stations.count >= 2, speed.isFinite, speed > 0 else {
      violations.append(.init(.insufficientEnergy, "No moving state or complete path to price"))
      return (result, finish(0))
    }
    if (initial.systems.speedbrakeFraction ?? 0) > 0,
      configuration.vehicle.speedbrakeFullDeploymentDragArea == nil {
      violations.append(.init(.modelUncertainty,
        "Measured speedbrake deployment has no identified drag model"))
      return (result, finish(0))
    }
    let nepDistance = stations.last(where: { $0.segment != .final })?.distance
    for i in stations.indices {
      if Task.isCancelled {
        violations.append(.init(.timeout, "Terminal pricing cancelled; remaining geometry is unpriced"))
        return (result, finish(0))
      }
      let s = stations[i]
      guard s.position.x.isFinite, s.position.y.isFinite, s.tangent.x.isFinite, s.tangent.y.isFinite,
        s.tangent.length > 0, s.height.isFinite, s.slope.isFinite, s.curvature.isFinite,
        s.verticalCurvature.isFinite, s.distance.isFinite else {
        violations.append(.init(.invalidGeometry, "Non-finite or degenerate terminal station"))
        return (result, finish(0))
      }
      gearDown = gearDown || s.height <= (configuration.guidance.gearDeploymentAltitude ?? 1500)
      let pos = frame.position(along: s.position.x, cross: s.position.y, height: s.height)
      let atm = atmosphere.sample(position: pos, time: time)
      let gamma = atan(s.slope)
      let q = 0.5 * atm.density * speed * speed
      let g = configuration.planet.gravitationalParameter / pos.squaredLength
      let vh = speed * cos(gamma)
      // Point-mass normal equation, with gamma curvature per actual path length.
      let vertical = (g - speed * speed / pos.length) * cos(gamma)
        + speed * speed * s.verticalCurvature
      let lateral = vh * vh * s.curvature
      let bank = atan2(lateral, vertical)
      let required = hypot(vertical, lateral)
      // MM305 and its closed-loop landing suffix are unpowered; KSP RCS is
      // disabled by the flight controller regardless of its orbital setting.
      // Only a MEASURED/identified trim envelope can override this conservative
      // transonic prior. A config flag permitting orbital RCS does not grant
      // extra terminal pitch authority.
      let supportedPrior = initial.controlAuthority?.evidence != .identified
        && speed / atm.speedOfSound <= 0.9 && (500...12000).contains(q)
      let maxAlpha = min(
        supportedPrior ? rotation.maximumTrimAlpha(dynamicPressure: q) : v.maximumAngleOfAttack,
        trimAlphaCeiling ?? v.maximumAngleOfAttack,
        s.height < 500 ? v.maximumLandingAngleOfAttack : v.maximumAngleOfAttack)
      let lift = aero.liftSolution(
        acceleration: required / max(Double.leastNormalMagnitude, liftMultiplier),
        mass: initial.mass, dynamicPressure: q, mach: speed / atm.speedOfSound,
        minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: maxAlpha, gear: gearDown)
      let load = required / 9.80665
      maxBank = max(maxBank, abs(bank)); maxQ = max(maxQ, q); maxLoad = max(maxLoad, load)
      minLift = min(minLift, lift.maximumAcceleration * liftMultiplier - required)
      if vertical < 0 {
        violations.append(.init(.insufficientLift,
          "Path requires negative upward normal force outside the positive-lift bank envelope",
          value: vertical, limit: 0))
      }
      if !lift.achievable {
        violations.append(.init(.insufficientLift, "Path demands unavailable lift/trim authority",
          value: required, limit: lift.maximumAcceleration * liftMultiplier))
      }
      if abs(bank) > v.maximumBank {
        violations.append(.init(.excessiveBank, "Path curvature exceeds bank envelope",
          value: abs(bank), limit: v.maximumBank))
      }
      if q > v.maximumDynamicPressure {
        violations.append(.init(.excessiveDynamicPressure, "Path q exceeds envelope",
          value: q, limit: v.maximumDynamicPressure))
      }
      if load > v.maximumLoadFactor {
        violations.append(.init(.excessiveLoad, "Path normal load exceeds envelope",
          value: load, limit: v.maximumLoadFactor))
      }
      if s.height < v.gearHeight - 0.1 {
        violations.append(.init(.terrainCollision, "Vertical path penetrates runway plane"))
      }
      let coefficient = aero.coefficients(mach: speed / atm.speedOfSound, alpha: lift.alpha,
        dynamicPressure: q, gear: gearDown)
      let cleanDrag = q * aero.referenceArea * coefficient.drag * dragMultiplier / initial.mass
      let brakeFraction: Double
      if s.segment != .final, let distance = nepDistance {
        brakeFraction = speedbrake.fraction(speed: speed, position: pos, time: time,
          mass: initial.mass, remainingDistance: distance-s.distance,
          target: nominalEnergyPoint, cleanDragAcceleration: cleanDrag)
      } else { brakeFraction = 0 }
      let brakeDrag = q * (configuration.vehicle.speedbrakeFullDeploymentDragArea ?? 0)
        * brakeFraction / initial.mass
      let maximumGlide = configuration.guidance.taemGlideAngle ?? configuration.guidance.outerGlideAngle
      if s.segment != .final, -gamma > maximumGlide {
        violations.append(.init(.invalidGeometry, "Terminal path exceeds configured glide-slope envelope",
          value: -gamma, limit: maximumGlide))
      }
      let elapsed = time - previousStationTime
      if i > 0, elapsed > 0,
        abs(wrapAngle(bank - previousBank)) / elapsed
          > min(v.controlPlant.maximumRollRate,
            (s.segment == .final ? configuration.guidance.approachRollRate : configuration.guidance.terminalRollRate)
              ?? configuration.guidance.bankSlewRate) * Self.bankRateReserve {
        violations.append(.init(.actuatorAuthority, "Path bank-rate demand exceeds available response"))
      }
      if i > 0, elapsed > 0,
        abs(lift.alpha - previousAlpha) / elapsed > configuration.guidance.alphaSlewRate {
        violations.append(.init(.actuatorAuthority, "Path angle-of-attack rate exceeds the guidance slew envelope"))
      }
      result.append(.init(
        distance: s.distance, position: s.position, tangent: s.tangent, curvature: s.curvature,
        height: s.height, slope: s.slope, verticalCurvature: s.verticalCurvature, speed: speed,
        angleOfAttack: lift.alpha, bank: bank, segment: s.segment,
        speedbrakeFraction: brakeFraction > 0 ? brakeFraction : nil))
      if !advisoryOnly && !violations.isEmpty { return (result, finish(0)) }
      if i + 1 < stations.count {
        let next = stations[i + 1]
        let ds = next.distance - s.distance
        guard ds > 0, vh > 0, (ds / vh).isFinite else {
          violations.append(.init(.invalidGeometry, "Non-increasing or non-traversable path station"))
          return (result, finish(0))
        }
        let drag = cleanDrag + brakeDrag
        let work = drag * ds / cos(gamma)
        let potential = configuration.planet.gravitationalParameter
          * (1 / pos.length - 1 / (configuration.planet.radius + configuration.runway.elevation + next.height))
        let nextSquared = speed * speed - 2 * (work + potential)
        if !nextSquared.isFinite || nextSquared <= 0 {
          // Stop the prediction. The caller may retain the unpriced geometry
          // for bounded recovery guidance, but must not invent speed or arrival time.
          exhaustion = s.distance
          violations.append(.init(.insufficientEnergy,
            "Path exhausts kinetic energy before the next station; continuation is unpriced",
            value: nextSquared.isFinite ? nextSquared : 0, limit: 0))
          return (result, finish(0))
        }
        // The first traversed interval starts at the measured attitude, not
        // at a freely substituted first reference bank/AoA.
        if i > 0 { previousBank = bank; previousAlpha = lift.alpha }
        previousStationTime = time
        speed = sqrt(nextSquared); time += ds / vh
      }
    }
    if speed < v.landingSpeed * Self.minimumTouchdownFraction {
      violations.append(.init(.insufficientEnergy, "Touchdown speed below design envelope",
        value: speed, limit: v.landingSpeed * Self.minimumTouchdownFraction))
    }
    if speed > v.maximumTouchdownSpeed {
      violations.append(.init(.excessiveEnergy, "Touchdown speed/rollout distance too high",
        value: speed, limit: v.maximumTouchdownSpeed))
    }
    let stopDistance = speed * speed / (2 * v.brakeFriction * 9.80665)
    if configuration.guidance.touchdownAlong + stopDistance > configuration.runway.length - 100 {
      violations.append(.init(.excessiveEnergy, "Insufficient runway braking distance",
        value: stopDistance, limit: configuration.runway.length - configuration.guidance.touchdownAlong - 100))
    }
    return (result, finish(speed, complete: true))
  }
}
