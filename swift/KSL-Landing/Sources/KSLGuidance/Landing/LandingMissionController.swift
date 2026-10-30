import Foundation
import KSLCore
import KSLControl
import KSLPhysics

/// Owns phase and committed plans. Numerical models and flight control live elsewhere.
public final class LandingMissionController {
  public let configuration: MissionConfiguration
  private let entry: EntryGuidanceEngine
  private let planner: HACPlanningEngine
  private let tracker: PathTrackingEngine
  private let approach: ApproachGuidanceEngine
  private let frame: RunwayFrame
  private let touchdown: TouchdownMonitor
  private let authority: AerodynamicAuthorityEstimator
  private let maximumGlideRatio: Double
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  public var environment = EnvironmentCalibration()
  public var latestEntryPrediction: TrajectoryPrediction? { entry.bestPrediction }
  public private(set) var phase: MissionPhase = .entry
  public private(set) var plan: TerminalPlan?
  public private(set) var lastPlanningResult: PlanningResult?
  public var controlFeedback: ControlDiagnostics?
  public var controlSnapshot: FlightControlSnapshot?
  private var lastPlanAttempt = -Double.infinity
  private var stoppedSince: Double?
  private var poorTrackingSince: Double?
  private var hacEstablished = false, hacCompleted = false
  private var limiter = GuidanceTargetLimiter()
  public let externalPlanning: Bool
  public private(set) var lastAdmission: MM305Admission?
  /// MM305's own physical verdict on the last searched state.
  public private(set) var lastCapture: TAEMCaptureAssessment?
  private var publishedForecastEpoch: Double?
  private var previousTime: Double?
  private var wasAirborne = false
  private var invalidationFailure: ConstraintViolation?
  /// Seed MM304's lateral aim from the landing-aware deorbit plan.
  public func planEntryAim(_ crossAim: Double, objective: EntryCaptureObjective? = nil) {
    entry.setCrossAim(crossAim); entry.setObjective(objective)
  }
  public func invalidateDynamics(at state: VehicleState) {
    authority.reset(); entry.invalidatePrediction(); approach.invalidatePrediction()
    controlSnapshot = nil; controlFeedback = nil
    lastPlanningResult = nil; lastAdmission = nil; lastPlanAttempt = -.infinity
    if plan != nil {
      tracker.reset(); poorTrackingSince = nil
      invalidationFailure = .init(.modelUncertainty,
        "Payload/configuration changed; retained HAC geometry while control re-identifies the plant")
    }
  }
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine, externalPlanning: Bool = false, initialPhase: MissionPhase = .entry,
    initialPlan: TerminalPlan? = nil
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    self.externalPlanning = externalPlanning
    phase = initialPhase
    plan = initialPlan
    entry = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    planner = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    tracker = .init(configuration: configuration, aerodynamics: aerodynamics)
    approach = .init(configuration: configuration, atmosphere: atmosphere,
      aerodynamics: aerodynamics, externalPlanning: externalPlanning)
    authority = .init(model: aerodynamics)
    maximumGlideRatio = aerodynamics.maximumLiftToDragRatio
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    touchdown = .init(configuration: configuration)
  }
  private var terminalSearchArmed = false
  private func inTerminalSearchEnvelope(_ state: VehicleState) -> Bool {
    let e = configuration.entrySettings
    let cadence = configuration.guidance.terminalReplanInterval
    if let forecast = entry.captureForecast, forecast.feasible {
      let capturePosition = frame.position(along: forecast.terminalAlong,
        cross: forecast.terminalCross, height: forecast.terminalAltitude)
      let reach = state.aerodynamics.airspeed * cadence
      if state.time + cadence >= forecast.capture.time
        || frame.distance(from: state.position, to: capturePosition) <= reach {
        terminalSearchArmed = true
      }
    }
    // Once armed, continue at MM305 cadence until a plan is admitted, even
    // after flying through the forecast region or losing the entry aim point.
    if terminalSearchArmed { return true }
    // Arm on approach to the operator's runway capture region. Once armed,
    // no aim-point distance or model verdict can stop MM305 from planning.
    let runwayRange = frame.distance(from: state.position, to: frame.origin)
    let inside = runwayRange <= e.captureRange + state.horizontalSpeed*cadence
    terminalSearchArmed = inside
    return inside
  }
  private func hacContext() -> HACFitContext {
    var context = HACFitContext()
    context.previousSpiral = plan?.spiral; context.previousPlanID = plan?.identifier
    context.onSpiral = (hacEstablished || phase == .headingAlignment) && !hacCompleted
    context.liftScale = authority.liftEffectiveness; context.dragScale = authority.dragEffectiveness
    context.rollResponseSeconds = controlFeedback?.rollResponseSeconds
    context.pitchResponseSeconds = controlFeedback?.pitchResponseSeconds
    if controlFeedback?.trimEvidence == .identified { context.trimAlphaCeiling = controlFeedback?.trimAlphaCeiling }
    return context
  }
  public func shouldPrioritizeTerminalSearch(_ state: VehicleState) -> Bool {
    [.entry, .acquisition, .headingAlignment].contains(phase) && !hacCompleted
      && (plan != nil || inTerminalSearchEnvelope(state))
  }
  public func planningRequest(from state: VehicleState, controls: FlightControlSnapshot?) -> MissionPlanningRequest? {
    guard externalPlanning else { return nil }
    entry.environment = environment
    if [.approach, .preflare, .flare].contains(phase), let request = approach.planningRequest() {
      return .init(time: state.time, entry: nil, terminalState: nil, controls: nil,
        previousTarget: nil, approach: request, environment: environment)
    }
    guard [.entry, .acquisition, .headingAlignment].contains(phase), !hacCompleted else { return nil }
    let entryRequest = phase == .entry && plan == nil
      ? entry.request(from: state, controls: controls, previousTarget: limiter.previous) : nil
    var terminalState: VehicleState?
    if (plan != nil || inTerminalSearchEnvelope(state)), state.time - lastPlanAttempt >= configuration.guidance.terminalReplanInterval {
      terminalState = state; lastPlanAttempt = state.time
    }
    guard entryRequest != nil || terminalState != nil else { return nil }
    return .init(time: state.time, entry: entryRequest, terminalState: terminalState,
      controls: controls, previousTarget: limiter.previous, environment: environment, hac: hacContext())
  }
  public func accept(_ proposal: MissionPlanningProposal, state: VehicleState) {
    guard proposal.time <= state.time else { return }
    if let result = proposal.approach { approach.accept(result) }
    guard [.entry, .acquisition, .headingAlignment].contains(phase), !hacCompleted else { return }
    if phase == .entry, plan == nil, let result = proposal.entry { entry.accept(result, at: state.time) }
    if let result = proposal.terminal { acceptTerminal(result, state: state) }
  }
  private func acceptTerminal(_ result: PlanningResult, state: VehicleState) {
    lastPlanningResult = result
    var verdict = TAEMCaptureEvaluator.assess(result, state: state, configuration: configuration)
    verdict.plan = nil; lastCapture = verdict
    guard let candidate = result.plan else { return }
    let admission = MM305Admission.evaluate(state, plan: candidate, configuration: configuration)
    lastAdmission = admission
    if plan == nil { hacEstablished = phase == .headingAlignment; hacCompleted = false }
    plan = candidate; tracker.reset()
  }
  public func update(_ state: VehicleState) throws -> GuidanceSolution {
    try state.validate(configuration: configuration)
    // Guidance never abandons the vehicle on its own judgement: every
    // envelope, energy or contact verdict is reported as an advisory while
    // the best available landing law keeps flying. Only a destroyed vehicle
    // ends the mission.
    var advisories: [ConstraintViolation] = invalidationFailure.map { [$0] } ?? []
    entry.environment = environment
    if previousTime == nil && [.approach, .preflare, .flare].contains(phase), !approach.canCapture(state) {
      advisories.append(.init(.modelUncertainty, "Initial Final capture/energy model advisory"))
    }
    if previousTime == nil && [.acquisition, .headingAlignment].contains(phase) {
      if plan == nil {
        acceptTerminal(planner.plan(from: state, allowDirect: false,
          controlSnapshot: controlSnapshot, previousTarget: limiter.previous), state: state)
      }
      if let plan { lastAdmission = MM305Admission.evaluate(state, plan: plan, configuration: configuration) }
    }
    authority.observe(state)
    if let t = previousTime, state.time <= t { throw FlightError.staleTelemetry }
    let dt = previousTime.map { state.time - $0 } ?? configuration.guidance.controlInterval
    if dt > 1 { throw FlightError.rearmRequired("Guidance clock skipped \(dt) seconds") }
    previousTime = state.time
    if !state.contact.isGrounded {
      var envelope: [ConstraintViolation] = []
      let v = configuration.vehicle
      if state.aerodynamics.dynamicPressure > v.maximumDynamicPressure {
        envelope.append(
          .init(
            .excessiveDynamicPressure, "Observed q exceeds the configured envelope",
            value: state.aerodynamics.dynamicPressure, limit: v.maximumDynamicPressure))
      }
      let load = abs(state.aerodynamics.lift) / (state.mass * 9.80665)
      if load > v.maximumLoadFactor {
        envelope.append(
          .init(
            .excessiveLoad, "Observed normal load exceeds the configured envelope", value: load,
            limit: v.maximumLoadFactor))
      }
      advisories += envelope
    }
    let contactFailures = touchdown.observe(state)
    advisories += contactFailures
    if state.contact == .crashed {
      phase = .abort
      return .init(
        phase: .abort, target: .init(),
        violations: [.init(.contactQuality, "Crash or runway excursion observed")])
    }
    if state.contact.isGrounded {
      let p = frame.coordinates(of: state.position)
      if !configuration.runway.contains(along: p.along, cross: p.cross) {
        // Off the pavement the best remaining action is still to stop.
        advisories.append(.init(.runwayExcursion, "Ground contact outside configured runway"))
      }
      if !wasAirborne {
        throw FlightError.invalidTelemetry(
          "Landing mission must begin airborne; existing parked vessel is not a successful landing")
      }
      phase = state.wheelContact?.noseGear == true ? .rollout : .touchdown
      if state.horizontalSpeed < 0.5 && state.wheelContact?.noseGear == true
        && state.wheelContact?.mainGear == true {
        if stoppedSince == nil { stoppedSince = state.time }
        if state.time - stoppedSince! >= configuration.guidance.stopConfirmationDuration {
          phase = .complete
        }
      } else {
        stoppedSince = nil
      }
      return .init(phase: phase, target: .init(gearDown: true), violations: advisories, planID: plan?.identifier)
    }
    wasAirborne = true
    if phase == .complete { return .init(phase: phase, target: .init()) }
    if phase == .abort { phase = .entry }
    let p = frame.coordinates(of: state.position)
    var raw: GuidanceSolution
    if plan == nil && state.aerodynamics.airspeed < 500 && p.height < 20_000
      && state.atmosphere.wind.length < 1
    {
      let u = configuration.vehicle.aerodynamicUncertainty
      let ld = maximumGlideRatio * (1 + u) / (1 - u)
      let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
      // Optimistic still-air screening: best polar L/D at every Mach,
      // favourable uncertainty, and an additional ballistic speed reserve.
      // Failure is an advisory about this modeled envelope.
      let optimisticRange = ld * max(0, p.height) + (ld / 2 + 1) * state.velocity.squaredLength / g
      // Reachability is to usable pavement ahead. A nominal aim point
      // already behind the vehicle is not a required return waypoint.
      let stopping = state.horizontalSpeed*state.horizontalSpeed
        / (2*configuration.vehicle.brakeFriction*g)
      let latestTouchdown = max(0, configuration.runway.length-stopping)
      let reachableAlong = clamp(p.along, 0, latestTouchdown)
      let distance = frame.distance(from: state.position,
        to: frame.position(along: reachableAlong, cross: 0, height: 0))
      if distance > optimisticRange {
        advisories += [
            .init(
              .insufficientEnergy, "Runway exceeds even the optimistic still-air modeled range",
              value: distance, limit: optimisticRange)
          ] + (lastCapture.map { [.init(Self.classification($0.deficit),
            "Last MM305 capture verdict: \($0.deficit.rawValue)")] } ?? [])
      }
    }
    if [.approach, .preflare, .flare].contains(phase) || (plan == nil && approach.canCapture(state))
    {
      raw = approach.update(
        state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
        pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
        identifiedTrimCeiling: controlFeedback?.trimEvidence == .identified ? controlFeedback?.trimAlphaCeiling : nil)
    } else {
      if !externalPlanning && !hacCompleted && (plan != nil || inTerminalSearchEnvelope(state))
        && state.time - lastPlanAttempt >= configuration.guidance.terminalReplanInterval
      {
        let context = hacContext()
        let currentPlanner = HACPlanningEngine(configuration: configuration,
          atmosphere: atmosphere.corrected(environment),
          aerodynamics: aerodynamics.scaled(lift: context.liftScale, drag: context.dragScale))
        let result = currentPlanner.plan(
          from: state, allowDirect: false, preview: true,
          controlSnapshot: controlSnapshot, previousTarget: limiter.previous, context: context)
        lastPlanningResult = result
        lastPlanAttempt = state.time
        acceptTerminal(result, state: state)
      }
      if let committed = plan {
        let (target, reference) = tracker.update(
          state, plan: committed, liftEffectiveness: authority.liftEffectiveness,
          identifiedTrimCeiling: controlFeedback?.trimEvidence == .identified ? controlFeedback?.trimAlphaCeiling : nil,
          rollResponseSeconds: controlFeedback?.rollResponseSeconds, pitchResponseSeconds: controlFeedback?.pitchResponseSeconds)
        if reference.segment == .spiral { hacEstablished = true }
        if hacEstablished, let lastSpiral = committed.stations.last(where: { $0.segment == .spiral }),
          reference.segment != .spiral, reference.distance >= lastSpiral.distance { hacCompleted = true }
        if reference.segment == .final {
          _ = approach.canCapture(state)
          raw = approach.update(
            state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
            pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
            identifiedTrimCeiling: controlFeedback?.trimEvidence == .identified ? controlFeedback?.trimAlphaCeiling : nil)
        } else {
          raw = .init(
            phase: reference.segment == .spiral ? .headingAlignment : .acquisition, target: target,
            violations: committed.stressCases.contains(where: { !$0.feasible })
              ? [
                .init(
                  .modelUncertainty,
                  "Nominal terminal route fails at least one configured aerodynamic stress case; this is not a robust-feasibility certificate"
                )
              ] : [],
            planID: committed.identifier, pathRemaining: committed.length - reference.distance)
        }
        if tracker.trackingError > max(1500, state.horizontalSpeed * 8) {
          if poorTrackingSince == nil { poorTrackingSince = state.time }
          if state.time - poorTrackingSince! > 8 && p.height > 5000 {
            tracker.reset()
            raw.violations.append(.init(.modelUncertainty,
              "HAC tracking error persists; retained route and reset path tracking"))
            poorTrackingSince = nil
          }
        } else {
          poorTrackingSince = nil
        }
      } else {
        let target = entry.update(state, synchronousPlanning: !externalPlanning,
          controls: controlSnapshot, previousTarget: limiter.previous, feedback: controlFeedback)
        var failures: [ConstraintViolation] = []
        if p.height < 1200 && !approach.canCapture(state) {
          failures = [
            .init(.insufficientEnergy, "No complete terminal path or final capture remains")
          ]
        }
        if !failures.isEmpty, let capture = lastCapture, !capture.feasible {
          failures.append(.init(Self.classification(capture.deficit),
            "MM305 capture infeasible: \(capture.deficit.rawValue)",
            value: capture.envelope.energyMargin.isFinite ? max(-1e9, capture.envelope.energyMargin) : 0))
        }
        // Out of terminal options near the ground: fly the final-approach
        // law toward the runway as the best remaining attempt.
        if !failures.isEmpty {
          raw = approach.update(
            state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
            pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
            identifiedTrimCeiling: controlFeedback?.trimEvidence == .identified ? controlFeedback?.trimAlphaCeiling : nil)
          raw.violations += failures
        } else {
          raw = .init(phase: .entry, target: target)
        }
        raw.diagnostic =
          "Entry receding-horizon candidates=\(entry.candidateCount), reversals=\(entry.reversalCount)"
        raw.entry = entry.diagnostics
      }
    }
    phase = raw.phase
    let target = limiter.update(raw.target, phase: raw.phase, configuration: configuration, dt: dt,
      dynamicPressure: state.aerodynamics.dynamicPressure,
      rollResponseSeconds: controlFeedback?.rollResponseSeconds,
      pitchResponseSeconds: controlFeedback?.pitchResponseSeconds)
    var solution = GuidanceSolution(
      phase: raw.phase, target: target, violations: raw.violations, planID: raw.planID,
      pathRemaining: raw.pathRemaining, diagnostic: raw.diagnostic)
    solution.violations += advisories
    solution.entry = raw.entry
    solution.approach = raw.approach
    if phase == .entry || phase == .acquisition || phase == .headingAlignment || phase == .abort {
      solution.capture = handoffDiagnostics()
    }
    if let admission = lastAdmission {
      var value = TerminalAdmissionDiagnostics()
      value.accepted = admission.accepted; value.altitudeMargin = admission.altitudeMargin
      value.machMargin = admission.machMargin; value.headingMargin = admission.headingMargin
      value.planAge = admission.planAge; value.violations = admission.violations
      solution.admission = value
    }
    return solution
  }

  static func classification(_ deficit: TAEMCaptureDeficit) -> FailureClassification {
    switch deficit {
    case .excessiveEnergy: return .excessiveEnergy
    case .insufficientEnergy, .insufficientAltitude, .terminalRegionNotReached: return .insufficientEnergy
    case .crossrangeBeyondTurningAuthority, .adverseHeading: return .invalidGeometry
    case .inadequateControlAuthority: return .actuatorAuthority
    case .envelopeViolation: return .excessiveDynamicPressure
    case .closedLoopFailure, .none: return .modelUncertainty
    }
  }

  private func handoffDiagnostics() -> CaptureHandoffDiagnostics {
    var d = CaptureHandoffDiagnostics()
    d.plannedCrossAim = entry.crossAim
    if let f = entry.captureForecast {
      let e = f.capture.envelope
      d.predictedFeasible = f.feasible; d.predictedDeficit = f.capture.deficit.rawValue
      d.predictedAt = f.createdAt; d.predictedCaptureTime = f.capture.time
      d.predictedAltitude = f.capture.altitude; d.predictedMach = f.capture.mach
      d.predictedAlong = f.terminalAlong; d.predictedCross = f.terminalCross
      d.predictedHACRadius = f.capture.hacRadius; d.predictedHACTurnAngle = f.capture.hacTurnAngle
      d.predictedHACSide = f.capture.hacSide; d.predictedPathLength = f.capture.planLength
      d.excessEnergyMargin = e.excessEnergyMargin > -1e12 ? e.excessEnergyMargin : nil
      d.deficitEnergyMargin = e.deficitEnergyMargin > -1e12 ? e.deficitEnergyMargin : nil
      d.requiredHeadingChange = e.requiredHeadingChange; d.availableTurnRate = e.availableTurnRate
      d.rangeMultiplier = f.rangeMultiplier
      // The sparse track is published once per new forecast.
      if publishedForecastEpoch != f.createdAt { d.predictedTrack = f.track; publishedForecastEpoch = f.createdAt }
    }
    if let capture = lastCapture {
      d.mm305Searched = true; d.mm305Deficit = capture.deficit.rawValue
      d.mm305Violations = Array(capture.violations.prefix(6))
    }
    d.mm305PlanAccepted = plan != nil
    if let plan {
      d.activePlanID = plan.identifier; d.activePlanCreatedAt = plan.createdAt
      d.activeHACRadius = plan.spiral?.finalRadius; d.activeHACGrowth = plan.spiral?.growth
      d.activeHACTurnAngle = plan.spiral?.turnAngle; d.activeHACSide = plan.spiral?.side
      d.activeHACExitAlong = plan.spiral?.exitAlong
      d.trackingError = tracker.trackingError; d.referenceHeadingError = tracker.headingError
      if let ref = tracker.reference {
        d.referenceHeight = ref.height; d.referenceSpeed = ref.speed; d.referenceSegment = ref.segment.rawValue
      }
      let stride = max(1, Int(ceil(Double(plan.stations.count)*configuration.guidance.controlInterval
        / configuration.guidance.pathLookaheadTime)))
      var reference: [[Double]] = [], elapsed = 0.0, previousDistance = 0.0
      for (index, station) in plan.stations.enumerated() {
        elapsed += (station.distance-previousDistance)/max(configuration.vehicle.landingSpeed
          * EnergyManagementEngine.minimumTouchdownFraction, station.speed)
        previousDistance = station.distance
        if index % stride == 0 || index == plan.stations.count-1 {
          let geo = GeodeticPosition(position: frame.position(along: station.position.x,
            cross: station.position.y, height: station.height), radius: configuration.planet.radius)
          reference.append([plan.createdAt+elapsed, degrees(geo.latitude), degrees(geo.longitude), geo.altitude, station.speed])
        }
      }
      d.activeReference = reference
    }
    if let admission = lastAdmission, !admission.violations.isEmpty {
      d.mm305Advisory = admission.violations.map(\.message).joined(separator: "; ")
    } else if let capture = lastCapture, !capture.feasible {
      d.mm305Advisory = "HAC model advisory: \(capture.deficit.rawValue)"
    }
    return d
  }
}
