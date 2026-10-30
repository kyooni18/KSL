import Foundation
import KSLCore
import KSLPhysics

struct EntryLawMemory: Sendable {
  var phase: EntryPhase = .preentry
  var previousTime: Double?, previousAlpha: Double?
  var modulation = 0.0, dragBoost = 0.0, deficit = 0.0
  var ceiling: Double?
  var identifiedTrimCeiling: Double?
  var lateral = EntryLateralMemory()
  /// Planned lateral offset of the MM304 aim point from the configured
  /// handoff point (m); chosen backward from TAEM capture, not fixed.
  var crossAim = 0.0
}

/// One owner per live executive or forecast branch. C DRAGREF, ALPHA and
/// lateral feedback remain separate, and all branches fly the same commands.
final class EntryCommandLaw {
  let configuration: MissionConfiguration
  let atmosphere: AtmosphericModelEngine
  let aero: AerodynamicsEngine
  let drag: EntryDragReference
  let frame: RunwayFrame
  let rotation: RotationalDynamicsEngine
  var memory: EntryLawMemory
  private(set) var diagnostics = EntryDiagnostics()
  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
       aerodynamics: AerodynamicsEngine, memory: EntryLawMemory = .init()) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
    drag = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet)
    self.memory = memory
  }
  func update(_ s: VehicleState, rangeMultiplier: Double = 1) -> GuidanceTarget {
    let c = configuration, e = c.entrySettings, a = s.aerodynamics
    let dt = min(1, max(0, s.time - (memory.previousTime ?? (s.time - c.guidance.controlInterval))))
    memory.previousTime = s.time
    let aimCross = e.handoffCross + memory.crossAim
    let target = frame.position(along: e.handoffAlong, cross: aimCross, height: 0)
    let range = frame.distance(from: s.position, to: target)
    let targetAtmosphere = atmosphere.sample(position: target.normalized * (c.planet.radius + e.terminalTargetAltitude), time: s.time)
    let terminalSpeed = e.terminalTargetMach * targetAtmosphere.speedOfSound
    let anchor = 3.4 * terminalSpeed
    let progress = clamp((anchor - a.airspeed) / max(1, anchor - terminalSpeed), 0, 1)
    let inferred: EntryPhase = a.dynamicPressure <= 0 ? .preentry
      : progress <= 0.30 ? .temperatureControl : progress <= 0.58 ? .equilibriumGlide
      : progress <= 0.82 ? .constantDrag : .transition
    // Velocity gates only advance the entry subphase. MM305 owns admission.
    if inferred.rawValue > memory.phase.rawValue { memory.phase = inferred }
    var d = drag.compute(s, phase: memory.phase, range: range * rangeMultiplier,
      targetAltitude: e.terminalTargetAltitude, targetSpeed: terminalSpeed, targetRange: e.terminalTargetRange)
    let confidence = a.measuredForces || a.modelEvidence != nil ? 1.0 : d.confidence
    let scales = [0.0, 0.35, 0.65, 1.0, 0.45], negative = [1.0, 0.85, 0.50, 0.20, 0.35]
    // Drag modulation answers energy/density errors, not the vehicle's own
    // transient AoA error: measured drag is referred to the commanded AoA,
    // otherwise the law closes a loop around the pitch response and cycles.
    let commandedAlpha = memory.previousAlpha ?? a.angleOfAttack
    let actualDragCoefficient = aero.coefficients(mach: a.mach, alpha: a.angleOfAttack).drag
    let attitudeDrag = actualDragCoefficient > 0
      ? d.observedDrag * aero.coefficients(mach: a.mach, alpha: commandedAlpha).drag / actualDragCoefficient
      : d.observedDrag
    let error = clamp((d.referenceDrag - attitudeDrag) / max(0.25, abs(d.referenceDrag)), -1.5, 1.5)
    var modulation = radians(3) * error * scales[memory.phase.rawValue] * confidence
    if modulation < 0 { modulation *= negative[memory.phase.rawValue] }
    modulation = clamp(modulation, -radians(4), radians(4))
    memory.modulation = moveToward(memory.modulation, modulation, maximumDelta: radians(0.8) * dt)
    // With RCS attitude control the hypersonic phase flies the maximum AoA
    // (heating/drag), blending to the nominal glide AoA across the same
    // subphase gates the law already uses (temperature control -> transition).
    func smoothStep(_ value: Double) -> Double { let x = clamp(value, 0, 1); return x * x * (3 - 2 * x) }
    let baseAlpha = c.vehicle.rcsAttitudeControl
      ? c.vehicle.maximumAngleOfAttack - (c.vehicle.maximumAngleOfAttack - e.nominalAlpha) * smoothStep((progress - 0.30) / (0.82 - 0.30))
      : e.nominalAlpha
    var alpha = max(baseAlpha, baseAlpha + memory.modulation)
    func smooth(_ value: Double) -> Double { let x = clamp(value, 0, 1); return x * x * (3 - 2 * x) }
    let qRatio = a.dynamicPressure / c.vehicle.maximumDynamicPressure
    let loadRatio = abs(a.lift) / (s.mass * 9.80665 * c.vehicle.maximumLoadFactor)
    if qRatio > 0.92 { alpha = max(alpha, baseAlpha + radians(3) * smooth((qRatio - 0.92) / 0.08)) }
    if loadRatio > 0.92 { alpha = min(alpha, baseAlpha - radians(4) * smooth((loadRatio - 0.92) / 0.08)) }
    let yawReserve = 1 - abs(s.appliedControls?.yaw ?? 0)
    let yawCeiling = baseAlpha + max(0, c.vehicle.maximumAngleOfAttack - baseAlpha) * yawReserve
    let deficit = d.referenceDrag > 0 ? clamp(1 - attitudeDrag / d.referenceDrag, 0, 1) : 0
    memory.deficit += clamp(dt / 8, 0, 1) * (deficit - memory.deficit)
    let previousCeiling = memory.ceiling ?? yawCeiling
    memory.ceiling = previousCeiling + clamp(dt / (yawCeiling < previousCeiling ? 2 : 8), 0, 1) * (yawCeiling - previousCeiling)
    let boostGoal = qRatio < 1 && loadRatio < 1 ? max(0, memory.ceiling! - alpha) * memory.deficit : 0
    memory.dragBoost = moveToward(memory.dragBoost, boostGoal, maximumDelta: radians(0.5) * dt)
    alpha += memory.dragBoost
    if d.dragScale <= 0.55 + 1e-9 && d.rangeError < 0 {
      alpha = min(alpha, max(baseAlpha, aero.bestGlideAlpha(mach: a.mach,
        maximumAlpha: c.vehicle.maximumAngleOfAttack, dynamicPressure: a.dynamicPressure)))
    }
    let trimCeiling = memory.identifiedTrimCeiling ?? c.vehicle.maximumAngleOfAttack
    alpha = min(alpha, c.vehicle.maximumAngleOfAttack, trimCeiling)
    alpha = moveToward(memory.previousAlpha ?? a.angleOfAttack, alpha,
      maximumDelta: min(radians(2), c.guidance.alphaSlewRate) * dt)
    memory.previousAlpha = alpha
    let p = frame.coordinates(of: s.position)
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let right = forward.cross(s.position.normalized).normalized
    let horizontal = s.velocity.projected(onPlaneWithNormal: s.position.normalized)
    let alignment = horizontal.normalized.dot(forward)
    let azimuth = wrapAngle(frame.bearing(from: s.position, to: target) - s.course)
    let bank = EntryLateralController.update(memory: &memory.lateral, state: s, configuration: c,
      verticalLift: d.verticalLift, azimuthError: azimuth, crossError: aimCross - p.cross,
      crossRate: -s.velocity.dot(right), alignment: alignment, range: range, dt: dt, diagnostics: &d)
    if alpha >= trimCeiling - 1e-9 && trimCeiling < e.nominalAlpha {
      d.limitingReason += "; identified local pitch-trim envelope limits alpha"
    }
    diagnostics = d
    return .init(angleOfAttack: alpha, bank: bank, desiredSpeed: terminalSpeed,
      desiredHeight: e.terminalTargetAltitude, desiredFlightPath: s.flightPathAngle)
  }
}
