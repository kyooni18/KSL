import Foundation
import KSLCore
import KSLPhysics

/// Flight-measured lift and drag calibrate the reference polar at the
/// *observed* Mach, AoA, dynamic pressure, mass and part/gear state. A
/// ballistic coefficient changes with incidence: measured drag at 30 deg
/// must not be extrapolated as if its CD applied at the best-glide angle.
struct EntryAerodynamicCalibration {
  let ballisticCoefficient: Double
  let liftToDrag: Double
  let evidence: EvidenceKind

  static func evaluate(_ s: VehicleState, configuration c: MissionConfiguration,
                       model: AerodynamicsEngine, maximumAlpha: Double) -> Self {
    let fallback = Self(ballisticCoefficient: c.entrySettings.estimatedBallisticCoefficient,
      liftToDrag: c.entrySettings.estimatedLiftToDrag, evidence: .provisional)
    let a = s.aerodynamics, q = a.dynamicPressure
    guard a.measuredForces, s.mass > 0, q.isFinite, q > 1,
      a.drag.isFinite, a.drag > 1, a.lift.isFinite, a.lift > 1,
      a.mach.isFinite, a.angleOfAttack.isFinite else { return fallback }
    let now = model.coefficients(mach: a.mach, alpha: a.angleOfAttack,
      dynamicPressure: q, beta: a.sideslip, gear: s.systems.gearDown)
    guard now.drag > 1e-8, now.lift > 1e-8 else { return fallback }
    let modelArea = c.vehicle.referenceArea
    let dragGain = a.drag / (q * modelArea * now.drag)
    let liftGain = a.lift / (q * modelArea * now.lift)
    guard dragGain.isFinite, dragGain > 0, liftGain.isFinite, liftGain > 0 else {
      return fallback
    }
    let bestAlpha = model.bestGlideAlpha(mach: a.mach,
      maximumAlpha: maximumAlpha, dynamicPressure: q)
    let best = model.coefficients(mach: a.mach, alpha: bestAlpha,
      dynamicPressure: q, gear: s.systems.gearDown)
    guard best.drag > 1e-8, best.lift > 0 else { return fallback }
    let coefficient = s.mass / (modelArea * best.drag * dragGain)
    let ratio = best.lift * liftGain / (best.drag * dragGain)
    guard coefficient.isFinite, coefficient > 0, ratio.isFinite, ratio > 0 else {
      return fallback
    }
    return .init(ballisticCoefficient: coefficient, liftToDrag: ratio, evidence: .measured)
  }
}

/// Active f1eebdc DRAGREF law. The measured-table/history polar replaces only
/// the retired analytic incidence multiplier; the range/vertical law is kept.
final class EntryDragReference {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aero: AerodynamicsEngine
  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
  }
  private func smooth(_ value: Double) -> Double {
    let x = clamp(value, 0, 1); return x * x * (3 - 2 * x)
  }
  func compute(_ s: VehicleState, phase: EntryPhase, range: Double,
               referenceRangeScale: Double = 1,
               objective: EntryCaptureObjective, maximumAlpha: Double) -> EntryDiagnostics {
    let c = configuration, e = c.entrySettings, a = s.aerodynamics
    let targetAltitude = objective.altitude, targetSpeed = objective.speed
    let speed = a.airspeed, height = s.position.length - c.planet.radius
    let polar = EntryAerodynamicCalibration.evaluate(s, configuration: c,
      model: aero, maximumAlpha: maximumAlpha)
    let minimum = 0.04
    let targetRadius = c.planet.radius + targetAltitude
    let targetGravity = c.planet.gravitationalParameter / (targetRadius * targetRadius)
    let maximum = max(minimum, min(0.72 * c.vehicle.maximumLoadFactor * targetGravity,
      c.vehicle.maximumDynamicPressure / polar.ballisticCoefficient))
    func equilibrium(_ velocity: Double, _ h: Double) -> Double {
      let r = c.planet.radius + h, g = c.planet.gravitationalParameter / (r * r)
      return clamp(g - velocity * velocity / r, 0.12 * g, 0.995 * g)
    }
    func equilibriumDrag(_ velocity: Double) -> Double {
      clamp(equilibrium(velocity, targetAltitude) / max(0.05, polar.liftToDrag), minimum, maximum)
    }
    let anchor = max(targetSpeed + 1, targetSpeed * 3.40)
    func progress(_ velocity: Double) -> Double { clamp((anchor - velocity) / max(1, anchor - targetSpeed), 0, 1) }
    func velocity(_ progress: Double) -> Double { anchor - progress * (anchor - targetSpeed) }
    let geo = GeodeticPosition(position: s.position, radius: c.planet.radius)
    func rotatingEnergy(radius: Double, latitude: Double, speed: Double) -> Double {
      speed * speed / 2 - c.planet.gravitationalParameter / radius
        - pow(c.planet.rotationRate * radius * cos(latitude), 2) / 2
    }
    let remaining = rotatingEnergy(radius: s.position.length, latitude: geo.latitude, speed: speed)
      - objective.specificEnergy(configuration: c)
    // The physical geodesic distance is never a drag-tuning parameter.
    // A planner may tune the reference profile, but cannot shorten the actual
    // runway/capture distance or claim a tighter achievable range by scaling it.
    let requiredRange = max(0, range)
    let referenceTargetRange = requiredRange * clamp(referenceRangeScale, 0.1, 2)
    let average = clamp(max(0, remaining) / max(1000, referenceTargetRange), minimum, maximum)
    let constant = clamp(exp(0.70 * log(max(average, 1e-9))
      + 0.30 * log(max(equilibriumDrag(velocity(0.58)), 1e-9))), minimum, maximum)
    func sample(_ h: Double) -> AtmosphericState {
      atmosphere.sample(position: s.position.normalized * (c.planet.radius + h), time: s.time)
    }
    let targetDensity = sample(targetAltitude).density
    let targetDrag = clamp(0.5 * targetDensity * targetSpeed * targetSpeed / polar.ballisticCoefficient, minimum, maximum)
    let nowDensity = sample(height).density
    let qNow = 0.5 * nowDensity * speed * speed
    let modeled = aero.coefficients(mach: a.mach, alpha: a.angleOfAttack, dynamicPressure: a.dynamicPressure,
      beta: a.sideslip, gear: s.systems.gearDown).drag * a.dynamicPressure * c.vehicle.referenceArea / s.mass
    let observed = a.measuredForces || a.modelEvidence != nil ? a.drag / s.mass : modeled
    let confidence = a.measuredForces ? 1.0 : a.modelConfidence ?? 0
    let useLocal = observed.isFinite && observed > 0 && qNow > 0
    let currentCD = aero.coefficients(mach: a.mach, alpha: a.angleOfAttack,
      dynamicPressure: qNow, beta: a.sideslip, gear: s.systems.gearDown).drag
    func projectedBounds(_ v: Double) -> (Double, Double) {
      guard useLocal else { return (minimum, maximum) }
      let fraction = clamp((v - targetSpeed) / max(1, speed - targetSpeed), 0, 1)
      let h = targetAltitude + (height - targetAltitude) * smooth(fraction)
      let atm = sample(h), q = 0.5 * atm.density * v * v
      // Drag feasibility must use the same attainable incidence envelope as
      // the guidance command, not the pre-entry nominal attitude as a cap.
      let ceiling = clamp(maximumAlpha, c.vehicle.minimumAngleOfAttack, c.vehicle.maximumAngleOfAttack)
      let floorAlpha = aero.bestGlideAlpha(mach: v/atm.speedOfSound, maximumAlpha: ceiling, dynamicPressure: q)
      let ceilingCD = aero.coefficients(mach: v/atm.speedOfSound, alpha: ceiling,
        dynamicPressure: q, gear: s.systems.gearDown).drag
      let cap = clamp(observed * q/qNow * ceilingCD/max(Double.leastNormalMagnitude, currentCD), minimum, maximum)
      let floorCD = aero.coefficients(mach: v / atm.speedOfSound, alpha: floorAlpha,
        dynamicPressure: q, gear: s.systems.gearDown).drag
      let floor = clamp(observed * q / qNow * max(0.02, floorCD) / max(0.05, currentCD), minimum, maximum)
      return (min(floor, cap), cap)
    }
    func baseDrag(_ v: Double) -> Double {
      let p = progress(v)
      if p <= 0.30 { return constant * (0.55 + 0.45 * smooth(p / 0.30)) }
      if p <= 0.58 {
        let window = pow(sin(.pi * (p - 0.30) / 0.28), 2)
        return constant + (equilibriumDrag(v) - constant) * 0.34 * window
      }
      if p <= 0.82 { return constant }
      return constant + (targetDrag - constant) * smooth((p - 0.82) / 0.18)
    }
    // Cache the common integration support for the unshaped and shaped profile.
    let count = 96, dv = max(0, speed - targetSpeed) / Double(count)
    let points = (0..<count).map { n -> (Double, Double, Double, Double) in
      let v = targetSpeed + (Double(n) + 0.5) * dv
      let (low, high) = projectedBounds(v)
      return (v, baseDrag(v), low, high)
    }
    func feasible(_ v: Double, _ scale: Double) -> Double {
      let (low, high) = projectedBounds(v)
      return clamp(baseDrag(v) * scale, low, high)
    }
    func predictedRange(_ scale: Double) -> Double {
      guard remaining > 0 else { return 0 }
      let kineticRange = points.reduce(0.0) { total, point in
        total + point.0 / max(minimum, clamp(point.1 * scale, point.2, point.3)) * dv
      }
      let other = remaining - max(0, (speed * speed - targetSpeed * targetSpeed) / 2)
      let mean = sqrt(max(minimum, feasible(max(speed, targetSpeed), scale)) * max(minimum, feasible(targetSpeed, scale)))
      return max(0, kineticRange + other / mean)
    }
    let rangeScale = max(10_000, max(1000, referenceTargetRange) * 0.28)
    // Solve the same capture-energy/range integral, within the plant's drag
    // envelope. A fixed scale floor could keep shedding energy far too early.
    var low = points.map { $0.2/max(Double.leastNormalMagnitude, $0.1) }.min() ?? 1
    var high = points.map { $0.3/max(Double.leastNormalMagnitude, $0.1) }.max() ?? 1
    let tolerance = max(configuration.entrySettings.handoffRadius, s.horizontalSpeed*c.guidance.controlInterval)
    var scale = (low+high)/2
    for _ in 0...Double.significandBitCount {
      scale = (low+high)/2
      let error = predictedRange(scale)-referenceTargetRange
      if abs(error) <= tolerance || scale == low || scale == high { break }
      if error > 0 { low = scale } else { high = scale }
    }
    let reference = feasible(speed, scale), predicted = predictedRange(scale)
    let errorFraction = (reference - observed) / max(0.25, abs(reference))
    let inertial = c.planet.inertialVelocity(position: s.position, fixedVelocity: s.velocity)
    let support = inertial.projected(onPlaneWithNormal: s.position.normalized).length
    let correction = clamp(0.28 * clamp(errorFraction, -1.5, 1.5)
      + 0.22 * clamp((predicted - referenceTargetRange) / rangeScale, -1.5, 1.5), -0.72, 0.72)
    var out = EntryDiagnostics()
    out.phase = phase; out.referenceDrag = reference; out.observedDrag = observed
    out.referenceBallisticCoefficient = polar.ballisticCoefficient
    out.referenceLiftToDrag = polar.liftToDrag
    out.referencePolarEvidence = polar.evidence
    out.verticalLift = equilibrium(support, height) * clamp(1 - correction, 0.10, 1.50)
    // predictedRange is the INVERSE-SOLVED drag-reference integral, not an
    // independent forward trajectory / MM305 capture feasibility forecast.
    out.predictedRange = predicted
    out.requiredRange = requiredRange
    out.referenceTargetRange = referenceTargetRange
    out.rangeError = predicted - referenceTargetRange
    out.dragScale = scale; out.confidence = confidence
    out.dragFloorActive = baseDrag(speed) * scale < projectedBounds(speed).0
    out.degraded = !a.measuredForces || confidence < 0.20 || remaining <= 0 || speed <= targetSpeed
    return out
  }
}
