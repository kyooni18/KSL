import Foundation
import KSLControl
import KSLCore
import KSLPhysics

/// One closed-loop MM304 branch flown to the TAEM interface, then judged by
/// the TAEM capture evaluator. The predicted ground track is retained so the
/// actual trajectory can later be compared with the prediction.
public struct EntryCaptureForecast: Codable, Sendable {
  public var advisoryInlet: TAEMAdvisoryInlet? = nil
  public var suggestedObjective: EntryCaptureObjective?
  public var objective: EntryCaptureObjective?
  public let createdAt: Double
  public let rangeMultiplier: Double
  public var crossAim = 0.0
  public let reachedTerminalRegion: Bool
  public let violations: [ConstraintViolation]
  public let terminalTime: Double
  public let terminalAltitude: Double
  public let terminalMach: Double
  public let terminalAlong: Double
  public let terminalCross: Double
  public let maximumDynamicPressure: Double
  public let maximumLoad: Double
  public var capture: TAEMCaptureAssessment
  /// Sparse predicted path (time, along, cross, height, speed, alpha, bank).
  public var track: [[Double]]
  public var feasible: Bool { reachedTerminalRegion && violations.isEmpty && capture.feasible }
}

/// Forecasts whether the current MM304 law, flown in the configured plant,
/// ends inside the TAEM reachable envelope. Shared by MM304's receding-horizon
/// objective and by landing-aware deorbit window selection.
public final class EntryCaptureForecaster {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  private let objectivePlanner: EntryCaptureObjectivePlanner
  private let frame: RunwayFrame
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    objectivePlanner = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }

  /// MM304's terminal target speed; capture is then searched beyond it.
  public func terminalSpeed(at time: Double) -> Double {
    let e = configuration.entrySettings
    let point = frame.position(along: configuration.taemInterfaceAlong, cross: e.handoffCross, height: 0)
    return configuration.mm304DeliveryMach * atmosphere.sample(
      position: point.normalized * (configuration.planet.radius + configuration.mm304DeliveryAltitude),
      time: time).speedOfSound
  }

  /// Legacy coordinated-level-turn length scale, retained for API compatibility.
  /// This assumes sufficient lift; it is not a reachability or force bound.
  /// Capture assessment and the reversal corridor use the priced force envelope.
  public func turningReach(at time: Double) -> Double {
    let v = terminalSpeed(at: time), c = configuration
    let g = c.planet.gravitationalParameter / pow(c.planet.radius + c.mm304DeliveryAltitude, 2)
    return v * v / (g * tan(max(1e-3, EnergyManagementEngine.acquisitionBank(c))))
  }

  /// Propose a crossrange inlet from the *predicted* TAEM-height miss, not
  /// from today's surface cross-track as if it were the future miss. The HAC
  /// radius supplies a bounded search extent, NOT a reachability claim.
  /// The separate closed-loop forecast and HAC fit must judge this candidate.
  public func lateralAims(from state: VehicleState, track: [[Double]]) -> [Double] {
    let c = configuration
    let targetHeight = c.mm304DeliveryAltitude-c.runway.elevation
    let sampled = track.filter { $0.count >= 4 }
      .min { abs($0[3]-targetHeight) < abs($1[3]-targetHeight) }
    guard let sampled, sampled[2].isFinite else { return [0] }
    let limit = min(c.guidance.maximumHACRadius,
      c.entrySettings.terminalTargetRange-c.guidance.finalDistance)
    guard limit > c.entrySettings.handoffRadius else { return [0] }
    let aim = clamp(sampled[2]-c.entrySettings.handoffCross, -limit, limit)
    guard abs(aim) > c.entrySettings.handoffRadius else { return [0] }
    return [0, aim]
  }

  /// Forecast from a fresh MM304 executive (deorbit planning).
  public func forecast(from state: VehicleState, rangeMultiplier: Double = 1, crossAim: Double = 0,
                       certify: Bool = false) -> EntryCaptureForecast {
    var memory = EntryLawMemory(); memory.crossAim = crossAim
    // A fresh deorbit rollout has no accepted MM304 proposal yet. Match its
    // native fallback target, rather than silently assuming an optimized
    // objective was installed in the mission controller before entry.
    memory.objective = EntryCaptureObjective.seed(configuration: configuration,
      atmosphere: atmosphere, frame: frame, crossAim: crossAim, time: state.time)
    return forecast(from: state, memory: memory, rangeMultiplier: rangeMultiplier,
      certify: certify, controls: nil, previousTarget: nil)
  }

  func forecast(from state: VehicleState, memory: EntryLawMemory, rangeMultiplier: Double,
                certify: Bool = false,
                controls: FlightControlSnapshot?, previousTarget: GuidanceTarget?,
                targetLimiter: GuidanceTargetLimiter? = nil) -> EntryCaptureForecast {
    let c = configuration
    var memory = memory
    if memory.objective == nil { memory.objective = objectivePlanner.objective(from: state, crossAim: memory.crossAim) }
    let predictor = TrajectoryPropagationEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    // Guidance keeps the nominal model: dispersions live in the plant only.
    let law = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics, memory: memory)
    let evaluator = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    func captureContext() -> HACFitContext {
      var context = HACFitContext()
      context.rollResponseSeconds = law.memory.rollResponseSeconds
      context.pitchResponseSeconds = law.memory.pitchResponseSeconds
      context.trimAlphaCeiling = law.memory.identifiedTrimCeiling
      context.previousSpiral = law.memory.terminalAdvice?.spiral
      context.previousPlanID = law.memory.terminalAdvice?.identifier
      return context
    }
    func evaluateCapture(_ state: VehicleState) -> TAEMCaptureAssessment {
      // Explore at the same early range as the live owner. A useful advisory
      // can shape MM304 without satisfying the MM305 ownership contract yet.
      var assessment = evaluator.evaluate(state, context: captureContext())
      // Match the live MM304 owner: a diagnostic best-effort HAC that has
      // failed its own physical assessment must not steer entry. Keeping it
      // here makes the forecast fly a different energy/lateral policy from
      // the native mission controller, even with identical initial state.
      law.memory.terminalAdvice = assessment.plan.flatMap {
        $0.assessment.feasible && $0.assessment.predictionComplete != false ? $0 : nil
      }
      law.memory.terminalProjectionIndex = 0
      law.memory.terminalDeficit = assessment.deficit
      if let plan = assessment.plan {
        let admission = MM305Admission.evaluate(state, plan: plan, configuration: c, requireQualification: false)
        if !admission.accepted {
          assessment.feasible = false
          if assessment.deficit == .none { assessment.deficit = .terminalRegionNotReached }
          assessment.violations += admission.violations
        }
      }
      return assessment
    }
    var terminalControls = controls, terminalTarget = previousTarget
    var terminalLimiter = targetLimiter
    var terminalSeed: NativeCommandSeed?
    var capture = TAEMCaptureAssessment()
    var searched = false, handedOff = false, nextCapture = state.time
    let first = predictor.predict(from: state, duration: c.guidance.maximumPredictionDuration,
      step: c.guidance.predictionStep, recordEvery: 10, terminalSpeed: 0,
      controlSnapshot: controls, previousTarget: previousTarget, targetLimiter: targetLimiter,
      feedback: {
        law.memory.pitchResponseSeconds = $0.pitchResponseSeconds
        law.memory.rollResponseSeconds = $0.rollResponseSeconds
        law.memory.identifiedTrimCeiling = $0.trimEvidence == .identified ? $0.trimAlphaCeiling : nil
      },
      handoff: { terminalControls = $0; terminalTarget = $1 },
      limiterHandoff: { terminalLimiter = $0 },
      nativeHandoff: { terminalSeed = $0 },
      terminalCondition: { s in
        let altitude = s.position.length-c.planet.radius
        let objective = law.memory.objective ?? EntryCaptureObjective.seed(configuration: c,
          atmosphere: self.atmosphere, frame: self.frame,
          crossAim: law.memory.crossAim, time: s.time)
        // Search advisories can steer entry, but ownership ends at precisely
        // the live executive's boundary, independently of plan feasibility.
        if LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
          state:s,configuration:c,objective:objective,
          responseSeconds:max(c.guidance.pathLookaheadTime,
            law.memory.pitchResponseSeconds ?? 0,law.memory.rollResponseSeconds ?? 0),
          forecast:law.memory.rangeReference?.handoffForecast,
          maximumLiftAcceleration:EntryHandoffTiming.maximumLift(state:s,configuration:c,
            aerodynamics:self.aerodynamics,
            ceiling:RotationalDynamicsEngine(configuration:c.vehicle,planet:c.planet)
              .supportedTrimAlpha(s,identifiedCeiling:law.memory.identifiedTrimCeiling))) {
          searched = true; handedOff = true
          capture = evaluateCapture(s)
          return true
        }
        if !searched {
          let runwayRange = self.frame.distance(from: s.position, to: self.frame.origin)
          guard runwayRange <= TerminalExploration.searchRange(state: s, configuration: c,
            minimumTurnTime: law.diagnostics.minimumTurnTime, rollResponseSeconds: law.memory.rollResponseSeconds)
          else { return false }
        }
        guard s.time >= nextCapture else { return false }
        searched = true
        capture = evaluateCapture(s)
        // A forecast needs physical capture surveys at distinct approach
        // states, not a full HAC family every five simulated seconds while
        // still well above the MM305 altitude envelope. Survey the current
        // state, then aim the next forecast at the upper inlet altitude or
        // halfway to its lower tolerance. The real MM305 planner retains
        // its original 5-second cadence. Surveys remain advisory until
        // the shared ownership boundary. Never skip the near-inlet survey.
        let cadence = c.guidance.terminalReplanInterval
        let upperInlet = c.entrySettings.terminalMaximumAltitude
        let lowerInlet = c.mm304DeliveryFloor
        let nextHeight = altitude > upperInlet ? upperInlet : (altitude + lowerInlet) / 2
        let descent = max(1.0, -s.verticalSpeed)
        let travel = max(0, altitude - nextHeight) / descent
        nextCapture = s.time + clamp(travel, cadence, altitude > upperInlet ? 12*cadence : 3*cadence)
        return false
      }) { s in
        let target = law.update(s, rangeMultiplier: rangeMultiplier)
        return .init(alpha: target.angleOfAttack, bank: target.bank)
      }
    let samples = first.samples, violations = first.violations, end = first.terminalState
    if handedOff && capture.feasible && certify {
      capture = evaluator.evaluate(end, certify: true, controlSnapshot: terminalControls,
        previousTarget: terminalTarget, targetLimiter: terminalLimiter, nativeSeed: terminalSeed, context: captureContext())
    }
    let p = frame.coordinates(of: end.position)
    if !searched {
      capture = .init(); capture.time = end.time; capture.altitude = p.height
      capture.mach = end.aerodynamics.mach
      capture.rangeToRunway = frame.distance(from: end.position, to: frame.origin)
      capture.violations = violations
      capture.deficit = violations.isEmpty ? .terminalRegionNotReached : .envelopeViolation
    }
    if !handedOff {
      capture.feasible = false
      capture.deficit = .terminalRegionNotReached
    }
    if searched, !violations.isEmpty {
      capture.feasible = false; capture.violations += violations; capture.deficit = .envelopeViolation
    }
    if violations.contains(where: { $0.kind == .modelUncertainty }) {
      capture.feasible = false; capture.certified = false; capture.deficit = .evaluationUnassessed
      if !capture.violations.contains(where: { $0.kind == .modelUncertainty }) { capture.violations += violations }
    }
    var maxQ = 0.0, maxLoad = 0.0
    let track: [[Double]] = samples.map { s in
      maxQ = max(maxQ, s.dynamicPressure); maxLoad = max(maxLoad, s.loadFactor)
      let q = frame.coordinates(of: s.position)
      return [s.time, q.along, q.cross, q.height, s.velocity.length, s.angleOfAttack, s.bank]
    }
    var result = EntryCaptureForecast(createdAt: state.time, rangeMultiplier: rangeMultiplier,
      reachedTerminalRegion: handedOff,
      violations: violations, terminalTime: end.time, terminalAltitude: p.height,
      terminalMach: end.aerodynamics.mach, terminalAlong: p.along, terminalCross: p.cross,
      maximumDynamicPressure: maxQ, maximumLoad: maxLoad, capture: capture, track: track)
    if handedOff { result.advisoryInlet = .init(sourceTime:state.time,state:end,model:"native-MM304-handoff") }
    result.crossAim = memory.crossAim
    result.objective = memory.objective ?? EntryCaptureObjective.seed(configuration: c,
      atmosphere: atmosphere, frame: frame, crossAim: memory.crossAim, time: state.time)
    let envelope = capture.envelope
    // A completed HAC family can bracket a nonempty energy interval even
    // when the flown state is just outside it. Center that interval, then
    // re-fly the objective; waiting for an already feasible trajectory would
    // leave an infeasible seed objective permanently in command.
    if end.position.length - c.planet.radius >= c.mm304DeliveryMinimumAltitude,
      frame.distance(from: end.position, to: frame.origin) <= c.entrySettings.terminalTargetRange
        + end.horizontalSpeed*c.guidance.terminalReplanInterval,
      envelope.excessEnergyMargin > -Double.greatestFiniteMagnitude,
      envelope.deficitEnergyMargin > -Double.greatestFiniteMagnitude,
      envelope.excessEnergyMargin + envelope.deficitEnergyMargin > 0 {
      let energyModel = RunwayEnergy(configuration: c)
      let energy = capture.envelope.energyHeight
        + (capture.envelope.excessEnergyMargin - capture.envelope.deficitEnergyMargin) / 2
      // Preserve high capture altitude and center energy through speed,
      // inside the configured moderate-speed band.
      let sound = end.atmosphere.speedOfSound
      let target = result.objective!
      let targetPosition = frame.position(along: target.along, cross: target.cross,
        height: target.altitude-c.runway.elevation)
      let centeredSpeed = clamp(sqrt(energyModel.speedSquared(height: energy, position: targetPosition)),
        max(0, c.mm304DeliveryMach-c.entrySettings.terminalMachHalfWidth)*sound,
        (c.mm304DeliveryMach+c.entrySettings.terminalMachHalfWidth)*sound)
      // The optimizer may refine the capture energy, not move the five-km
      // centerline objective to wherever an unsuccessful rollout happened to end.
      result.suggestedObjective = .init(along: target.along, cross: target.cross,
        altitude: target.altitude, speed: centeredSpeed,
        energyHeight: energyModel.height(position: targetPosition, speedSquared: centeredSpeed*centeredSpeed),
        flightPath: target.flightPath)
    }
    return result
  }
}
