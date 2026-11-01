import Foundation
import KSLCore
import KSLPhysics

/// A capture state in the runway chart, shared by drag, lateral steering and prediction.
public struct EntryCaptureObjective: Codable, Sendable {
  public var along: Double
  public var cross: Double
  public var altitude: Double
  public var speed: Double
  public var energyHeight: Double
  public var flightPath: Double?
  /// Rotating-frame mechanical energy, J/kg. Altitude is ASL, never runway AGL.
  public func specificEnergy(configuration c: MissionConfiguration) -> Double {
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let p = frame.position(along: along, cross: cross, height: altitude-c.runway.elevation)
    return speed*speed/2-c.planet.gravitationalParameter/p.length
      - c.planet.angularVelocity.cross(p).squaredLength/2
  }
  static func seed(configuration c: MissionConfiguration, atmosphere: AtmosphericModelEngine,
                   frame: RunwayFrame, crossAim: Double, time: Double) -> Self {
    let e = c.entrySettings, cross = e.handoffCross + crossAim
    let glide = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
    let range = c.nepOverheadCaptureRange
    let along = -sqrt(max(0, range * range - cross * cross))
    let radius = c.planet.radius + e.terminalTargetAltitude
    let position = frame.position(along: along, cross: cross, height: 0).normalized * radius
    let speed = e.terminalTargetMach
      * atmosphere.sample(position: position, time: time).speedOfSound
    let g = c.planet.gravitationalParameter / (radius * radius)
    return .init(along: along, cross: cross, altitude: e.terminalTargetAltitude, speed: speed,
      energyHeight: e.terminalTargetAltitude - c.runway.elevation + speed * speed / (2 * g), flightPath: -glide)
  }
}

struct EntryLawMemory: Sendable {
  var objective: EntryCaptureObjective?
  var pitchResponseSeconds: Double?
  var rollResponseSeconds: Double?
  /// Prospective terminal geometry, never an MM305 ownership grant.
  var terminalAdvice: TerminalPlan?
  var terminalProjectionIndex = 0
  var terminalDeficit: TAEMCaptureDeficit?

  var phase: EntryPhase = .preentry
  var previousTime: Double?, previousAlpha: Double?
  var identifiedTrimCeiling: Double?
  var lateral = EntryLateralMemory()
  /// Planned lateral offset of the MM304 aim point from the configured
  /// handoff point (m); chosen backward from TAEM capture, not fixed.
  var crossAim = 0.0
  mutating func retargetCrossAim(_ value: Double) {
    crossAim = value
    objective = nil
  }
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
  private let terminalTracker: PathTrackingEngine
  private var trackedTerminalEpoch: Double?
  var memory: EntryLawMemory
  private(set) var diagnostics = EntryDiagnostics()
  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
       aerodynamics: AerodynamicsEngine, memory: EntryLawMemory = .init()) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
    drag = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet)
    self.memory = memory
    terminalTracker = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
      initialIndex: memory.terminalProjectionIndex)
    trackedTerminalEpoch = memory.terminalAdvice?.createdAt
  }
  static func descendingCaptureGamma(proposed: Double, actual: Double,
                                      currentAltitude: Double, targetAltitude: Double,
                                      verticalSpeed: Double) -> Double {
    guard currentAltitude < targetAltitude, verticalSpeed < 0, actual < 0 else { return proposed }
    return min(proposed, actual)
  }

  func update(_ s: VehicleState, rangeMultiplier: Double = 1) -> GuidanceTarget {
    let c = configuration, e = c.entrySettings, a = s.aerodynamics
    let dt = min(1, max(0, s.time - (memory.previousTime ?? (s.time - c.guidance.controlInterval))))
    memory.previousTime = s.time
    let objective = memory.objective ?? EntryCaptureObjective.seed(configuration: c,
      atmosphere: atmosphere, frame: frame, crossAim: memory.crossAim, time: s.time)
    let aimCross = objective.cross, targetAltitude = objective.altitude
    let target = frame.position(along: objective.along, cross: aimCross, height: 0)
    let range = frame.distance(from: s.position, to: target)
    let terminalSpeed = objective.speed
    let anchor = 3.4 * terminalSpeed
    let progress = clamp((anchor - a.airspeed) / max(1, anchor - terminalSpeed), 0, 1)
    let inferred: EntryPhase = a.dynamicPressure <= 0 ? .preentry
      : progress <= 0.30 ? .temperatureControl : progress <= 0.58 ? .equilibriumGlide
      : progress <= 0.82 ? .constantDrag : .transition
    // Velocity gates only advance the entry subphase. MM305 owns admission.
    if inferred.rawValue > memory.phase.rawValue { memory.phase = inferred }
    let ceilingAlpha = rotation.supportedTrimAlpha(s, identifiedCeiling: memory.identifiedTrimCeiling)
    var d = drag.compute(s, phase: memory.phase, range: range,
      referenceRangeScale: rangeMultiplier,
      objective: objective, maximumAlpha: ceilingAlpha)
    // Capture altitude and flight-path angle are part of the terminal state.
    // Aim at the same capture state as prediction. The endpoint slope only
    // enters over the physical lookahead, rather than demanding a loft for
    // hundreds of kilometres because the terminal glide is steep.
    let horizon = max(c.guidance.pathLookaheadTime, range / max(1, s.horizontalSpeed))
    let height = s.position.length-c.planet.radius
    let meanSlope = (targetAltitude-height)/max(range, s.horizontalSpeed*c.guidance.pathLookaheadTime)
    let terminalSlope = tan(objective.flightPath ?? -(c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle))
    let geometricGamma = atan(meanSlope+(terminalSlope-meanSlope)*c.guidance.pathLookaheadTime/horizon)
    // A point above a descending, unpowered vehicle is not a reachable
    // terminal capture state merely because the inverse geometry says so.
    // After passing below the preferred capture height, do not command a
    // climb/shallower flight-path toward that abandoned point. Retain the
    // measured descending slope while terminal planning searches for a
    // supported lower inlet. The entry law still evaluates the true range.
    let desiredGamma = Self.descendingCaptureGamma(proposed: geometricGamma,
      actual: s.flightPathAngle, currentAltitude: height,
      targetAltitude: targetAltitude, verticalSpeed: s.verticalSpeed)
    let gravity = c.planet.gravitationalParameter/s.position.squaredLength
    let inertial = c.planet.inertialVelocity(position: s.position, fixedVelocity: s.velocity)
    let support = max(0, (gravity-inertial.projected(onPlaneWithNormal: s.position.normalized).squaredLength/s.position.length)*cos(s.flightPathAngle))
    let captureLift = max(0, support+a.airspeed*(desiredGamma-s.flightPathAngle)/horizon)
    d.verticalLift = captureLift
    let p = frame.coordinates(of: s.position)
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let horizontal = s.velocity.projected(onPlaneWithNormal: s.position.normalized)
    let alignment = horizontal.normalized.dot(forward)
    let coordinateRate = frame.coordinateVelocity(at: s.position, velocity: s.velocity)
    let crossRate = -coordinateRate.y
    let azimuth = wrapAngle(frame.bearing(from: s.position, to: target) - s.course)
    var lateralDemand = EntryLateralController.demand(state: s, configuration: c,
      azimuthError: azimuth, crossError: aimCross - p.cross, crossRate: crossRate,
      alignment: alignment, range: abs(objective.along - p.along))
    var terminalRemaining: Double?
    if let advice = memory.terminalAdvice,
      advice.assessment.feasible && advice.assessment.predictionComplete != false,
      advice.stations.count >= 2,
      s.time >= advice.createdAt, s.time - advice.createdAt <= 2 * c.guidance.terminalReplanInterval {
      if trackedTerminalEpoch != advice.createdAt {
        terminalTracker.reset(); trackedTerminalEpoch = advice.createdAt
      }
      let steering = terminalTracker.lateralDemand(s, plan: advice,
        rollResponseSeconds: memory.rollResponseSeconds)
      lateralDemand = steering.acceleration
      terminalRemaining = max(0, advice.length - steering.reference.distance)
      memory.terminalProjectionIndex = terminalTracker.index
      d.terminalAdviceAge = s.time - advice.createdAt
      d.terminalSteeringActive = true
      d.terminalLookaheadDistance = terminalTracker.lookaheadDistance
      d.terminalLookaheadHeadingError = terminalTracker.lookaheadHeadingError
    } else {
      d.terminalSteeringActive = false
    }
    let response = memory.pitchResponseSeconds ?? (c.vehicle.controlPlant.maximumPitchRate
      / max(Double.leastNormalMagnitude, c.vehicle.controlPlant.pitchAccelerationMaximum))
    // The target limiter already reserves plant acceleration. A second
    // division by (1-reserve) hid a ~6.7-response-time lag in the old law.
    let tau = response
    let factor = a.dynamicPressure*aero.referenceArea/s.mass
    let current = aero.coefficients(mach: a.mach, alpha: a.angleOfAttack,
      dynamicPressure: a.dynamicPressure, beta: a.sideslip, gear: s.systems.gearDown)
    let dragGain = factor*current.drag > 0 && d.observedDrag > 0 ? d.observedDrag/(factor*current.drag) : 1
    let liftGain = factor*current.lift > 0 && a.lift > 0 ? a.lift/s.mass/(factor*current.lift) : 1
    let uncertainty = c.vehicle.aerodynamicUncertainty
    let floor = min(ceilingAlpha, max(0, c.vehicle.minimumAngleOfAttack))
    let choices = (0...64).map { i -> EntryForcePoint in
      let alpha = floor+(ceilingAlpha-floor)*Double(i)/64
      let polar = aero.coefficients(mach: a.mach, alpha: alpha, dynamicPressure: a.dynamicPressure,
        beta: a.sideslip, gear: s.systems.gearDown)
      return .init(alpha: alpha, lift: max(0, polar.lift*factor*liftGain*(1-uncertainty)),
        drag: max(0, polar.drag*factor*dragGain))
    }
    let loadLimit = c.vehicle.maximumLoadFactor * 9.80665
    let supporting = choices.filter { $0.lift >= captureLift &&
      $0.lift <= loadLimit * (1-uncertainty) }
    // Energy reserve is based on the attainable lowest-drag supporting polar.
    let recovery = supporting.min { $0.drag < $1.drag }
      ?? choices.max { $0.lift < $1.lift }!
    let energy = s.velocity.squaredLength/2-c.planet.gravitationalParameter/s.position.length
      - c.planet.angularVelocity.cross(s.position).squaredLength/2
    var endpointEnergy = objective.specificEnergy(configuration: c)
    if terminalRemaining != nil {
      // Prospective HAC geometry includes the return turn and final, so price
      // it against the configured touchdown state, not a 25-km point already abeam.
      let touchdown = frame.position(along: c.guidance.touchdownAlong, cross: 0, height: c.vehicle.gearHeight)
      endpointEnergy = c.vehicle.landingSpeed * c.vehicle.landingSpeed / 2
        - c.planet.gravitationalParameter / touchdown.length
        - c.planet.angularVelocity.cross(touchdown).squaredLength / 2
    }
    let available = energy - endpointEnergy
    let pathDistance = max(terminalRemaining ?? range, s.horizontalSpeed * c.guidance.pathLookaheadTime)
      * a.airspeed / max(1, s.horizontalSpeed)
    // Reserve uncertain best-range drag work and the unavoidable work while
    // the measured pitch loop sheds its old demand. Both are J/kg, not metres.
    let reserve = uncertainty*recovery.drag*pathDistance
      + max(0, d.observedDrag-recovery.drag)*a.airspeed*tau
    let budget = max(0, available-reserve)/max(Double.leastNormalMagnitude, pathDistance)
    let joint = EntryForceAllocator.assessJointForces(points: choices,
      verticalDemand: captureLift, lateralDemand: lateralDemand,
      maximumBank: c.vehicle.maximumBank, loadLimit: loadLimit,
      uncertainty: uncertainty, dragBudget: budget)
    d.jointRequiredLift = joint.requiredLift
    d.jointMaximumLift = joint.maximumAvailableLift
    d.jointMaximumLateral = joint.maximumLateralWithVerticalSupport
    d.jointMinimumDrag = joint.minimumSimultaneousDrag
    d.jointDragBudgetDeficit = joint.dragBudgetDeficit
    d.jointForceAttainable = joint.simultaneouslySupported
    d.referenceDrag = terminalRemaining == nil ? min(d.referenceDrag, budget) : budget
    // The same force allocation is flown live and evaluated by the deorbit
    // predictor (EntryCaptureForecaster -> EntryCommandLaw).
    let decision = EntryForceAllocator.decide(points: choices,
      verticalDemand: captureLift, lateralDemand: lateralDemand,
      referenceDrag: d.referenceDrag, dragBudget: budget,
      maximumBank: c.vehicle.maximumBank, loadLimit: loadLimit,
      uncertainty: uncertainty, previousAlpha: memory.previousAlpha ?? a.angleOfAttack,
      heightAboveCapture: height-targetAltitude,
      timeToCapture: range / max(1, s.horizontalSpeed))
    d.lateralAllocationTarget = decision.lateral
    d.selectedDragAcceleration = decision.drag
    d.dragTrackingResidual = decision.drag - d.referenceDrag
    d.captureSpecificEnergy = endpointEnergy
    d.terminalDeficit = memory.terminalDeficit?.rawValue
    d.energyBudgetEndpoint = terminalRemaining == nil ? "entry-objective" : "terminal-touchdown"
    let maximumTurn = choices.reduce(0.0) { result, point in
      max(result, NormalForceEnvelope(maximumLift: min(point.lift, loadLimit),
        maximumBank: c.vehicle.maximumBank).lateralAuthority(reserving: captureLift))
    }
    let runwayHeadingError = abs(wrapAngle(s.localFrame.heading(of: forward) - s.course))
    if maximumTurn > 0 { d.minimumTurnTime = s.horizontalSpeed * runwayHeadingError / maximumTurn }
    d.captureEnergyMargin = available-reserve
    d.energyDragBudget = budget
    d.energyReserve = reserve
    d.recoveryAlpha = recovery.alpha
    d.bestRangeRecovery = !decision.affordable
    d.energyAllocationMode = !decision.verticallySupported
      ? (abs(decision.bank) > 1e-9 ? "altitude-budgeted-turn-recovery" : "vertical-support-recovery")
      : !decision.affordable ? (abs(lateralDemand) > 0 ? "turn-energy-recovery" : "minimum-drag-recovery")
      : "energy-budgeted-steering"
    // There is one AoA decision. No modulation, stored drag boost, q bump or
    // post-filter trim adjustment can spend the capture reserve behind it.
    let previous = memory.previousAlpha ?? a.angleOfAttack
    // In vacuum every aerodynamic choice has exactly the same zero force.
    // Prepare the configured entry attitude instead of holding an arbitrary
    // initial AoA until aerodynamic authority has already built up.
    let requestedAlpha = a.dynamicPressure == 0
      ? clamp(e.nominalAlpha, c.vehicle.minimumAngleOfAttack, ceilingAlpha) : decision.alpha
    if a.dynamicPressure == 0 { d.energyAllocationMode = "preentry-attitude" }
    var alpha = previous+(requestedAlpha-previous)*(-expm1(-dt/tau))
    alpha = moveToward(previous, alpha, maximumDelta: c.guidance.alphaSlewRate*dt)
    alpha = clamp(alpha, c.vehicle.minimumAngleOfAttack, ceilingAlpha)
    memory.previousAlpha = alpha
    let bank = EntryLateralController.update(memory: &memory.lateral, state: s, configuration: c,
      // In the altitude-budgeted branch, the plan explicitly permits a
      // bounded reduction in vertical support to buy steering. Do not have
      // the measured-lift bank limiter silently undo that allocation using
      // the original, infeasible vertical request.
      verticalLift: decision.verticallySupported ? d.verticalLift
        : min(d.verticalLift, max(0, a.lift / s.mass) * cos(abs(decision.bank))),
      azimuthError: azimuth, crossError: aimCross - p.cross,
      crossRate: crossRate, alignment: alignment, range: abs(objective.along-p.along), dt: dt,
      lateralDemand: lateralDemand, allocatedBank: decision.bank, diagnostics: &d)
    if alpha >= ceilingAlpha - 1e-9 && ceilingAlpha < e.nominalAlpha {
      d.limitingReason += "; identified local pitch-trim envelope limits alpha"
    }
    diagnostics = d
    return .init(angleOfAttack: alpha, bank: bank, desiredSpeed: terminalSpeed,
      desiredHeight: targetAltitude, desiredFlightPath: desiredGamma)
  }
}
