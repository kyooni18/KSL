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
  private let taem: TAEMGuidanceEngine
  private let nepSolver: NominalEnergyPointSolver
  private let speedbrake: TAEMSpeedbrakeController
  private var nominalEnergyPoint: (time: Double, point: NominalEnergyPoint?)?
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
  public private(set) var advisoryPlan: TerminalPlan?
  public var controlFeedback: ControlDiagnostics?
  public var controlSnapshot: FlightControlSnapshot?
  private var lastPlanAttempt = -Double.infinity
  private var stoppedSince: Double?
  private var poorTrackingSince: Double?
  private var hacEstablished = false, hacCompleted = false
  private var activePlanDynamicsCurrent = true
  private var activeReferenceCache: [[Double]]?
  private var limiter = GuidanceTargetLimiter()
  private var incidence = AttainableIncidenceMonitor()
  /// Tightest evidenced AoA ceiling: identified trim, or sustained pitch
  /// saturation. Planner, admission, tracker and recovery all price forces
  /// with it so they agree with what the closed loop can actually hold.
  private var attainableAlphaCeiling: Double? {
    incidence.combined(with: controlFeedback?.trimEvidence == .identified ? controlFeedback?.trimAlphaCeiling : nil)
  }
  public var measuredAttainableAlphaCeiling: Double? { incidence.ceiling }
  public var taemStatus: TAEMStatus { taem.status }
  public let externalPlanning: Bool
  public private(set) var lastAdmission: MM305Admission?
  /// MM305's own physical verdict on the last searched state.
  public private(set) var lastCapture: TAEMCaptureAssessment?
  private var publishedForecastEpoch: Double?
  private var publishedTerminalEpoch: Double?
  private var previousTime: Double?
  private var wasAirborne = false
  private var crashObserved = false
  private var invalidationFailure: ConstraintViolation?
  /// Seed MM304's lateral aim from the landing-aware deorbit plan.
  public func planEntryAim(_ crossAim: Double, objective: EntryCaptureObjective? = nil) {
    entry.setCrossAim(crossAim); entry.setObjective(objective)
  }
  public func invalidateDynamics(at state: VehicleState) {
    authority.reset(); entry.invalidatePrediction(); approach.invalidatePrediction()
    incidence.reset()
    nominalEnergyPoint = nil
    controlSnapshot = nil; controlFeedback = nil
    lastPlanningResult = nil; lastAdmission = nil; lastPlanAttempt = -.infinity
    lastCapture = nil; advisoryPlan = nil; publishedTerminalEpoch = nil
    if plan != nil {
      activePlanDynamicsCurrent = false; activeReferenceCache = nil
      poorTrackingSince = nil
      invalidationFailure = .init(.modelUncertainty,
        "Payload/configuration changed; retained HAC geometry while control re-identifies the plant")
    }
  }
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine, externalPlanning: Bool = false, initialPhase: MissionPhase = .entry,
    initialPlan: TerminalPlan? = nil, entryReplanning: Bool = true
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    self.externalPlanning = externalPlanning
    phase = initialPhase
    plan = initialPlan
    entry = .init(configuration: configuration, atmosphere: atmosphere,
      aerodynamics: aerodynamics, permitsReplanning: entryReplanning)
    planner = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    tracker = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    approach = .init(configuration: configuration, atmosphere: atmosphere,
      aerodynamics: aerodynamics, externalPlanning: externalPlanning)
    taem = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    nepSolver = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    speedbrake = .init(configuration: configuration, atmosphere: atmosphere)
    authority = .init(model: aerodynamics)
    maximumGlideRatio = aerodynamics.maximumLiftToDragRatio
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    touchdown = .init(configuration: configuration)
  }
  private var terminalSearchArmed = false
  private var forcedTerminalOwnership = false
  /// Shuttle TAEM is initiated on velocity (JSC-16522): on its drag profile
  /// the orbiter reaches the interface velocity at the interface energy.
  /// MM304 delivers that ENERGY (interface altitude at interface Mach); this
  /// vehicle decelerates through Mach 2.5 above it, 60+ km out, so a bare
  /// Mach gate hands over early. Hand over at the interface energy, at the
  /// bottom of the configured Mach window, or at the altitude deadline.
  /// Crossing an altitude far up-range is not a handoff.
  public static func terminalHandoffDue(phase: MissionPhase, plan: TerminalPlan?,
    state: VehicleState, configuration c: MissionConfiguration,
    objective: EntryCaptureObjective? = nil) -> Bool {
    guard phase == .entry && plan == nil && state.verticalSpeed < 0 else { return false }
    let altitude = state.position.length - c.planet.radius
    if altitude <= c.mm304HandoffDeadlineAltitude { return true }
    let a = state.aerodynamics
    guard a.dynamicPressure > 0, a.mach > 0 else { return false }
    if a.mach <= c.mm304DeliveryMach - c.entrySettings.terminalMachHalfWidth { return true }
    // Use the same fixed interface state as MM304's reference, not a Mach
    // speed recomputed using the atmosphere at the vehicle's current altitude.
    guard let target = objective ?? (try? AtmosphericModelEngine(planet: c.planet)).map({
      EntryCaptureObjective.seed(configuration: c, atmosphere: $0,
        frame: RunwayFrame(runway: c.runway, planet: c.planet), crossAim: 0, time: state.time)
    }) else { return false }
    let energy = state.velocity.squaredLength / 2 - c.planet.gravitationalParameter / state.position.length
      - c.planet.angularVelocity.cross(state.position).squaredLength / 2
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let targetPosition = frame.position(along: target.along, cross: target.cross, height: 0)
    // Reaching interface energy up-range is a delivery deficit, not arrival.
    // Keep the existing altitude/Mach deadlines above as recovery downmodes.
    let reachedInterface = frame.distance(from: state.position, to: frame.origin)
      <= frame.distance(from: targetPosition, to: frame.origin) + c.entrySettings.handoffRadius
    return reachedInterface && energy <= target.specificEnergy(configuration: c)
  }
  private func inTerminalSearchEnvelope(_ state: VehicleState) -> Bool {
    let cadence = configuration.guidance.terminalReplanInterval
    if let forecast = entry.captureForecast, forecast.capture.time > 0,
      forecast.capture.deficit != .terminalRegionNotReached {
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
    // Delivery accuracy is an MM304 objective, never permission to search.
    // Survey while useful terminal altitude and speed remain, even when the
    // entry path misses the centerline or passes abeam the target.
    // Survey permission is independent of the MM305 operating envelope.
    // This publishes advice at high Mach without transferring control.
    let inside = runwayRange <= TerminalExploration.searchRange(state: state, configuration: configuration,
      minimumTurnTime: entry.diagnostics.minimumTurnTime, rollResponseSeconds: controlFeedback?.rollResponseSeconds)
    terminalSearchArmed = inside
    return inside
  }
  private var tracksCurrentRoute: Bool {
    tracker.reference != nil && tracker.trackingError <= configuration.entrySettings.handoffRadius
      && abs(tracker.headingError) <= configuration.entrySettings.headingHalfWidth
  }
  private func hacContext() -> HACFitContext {
    var context = HACFitContext()
    context.previousSpiral = (plan ?? advisoryPlan)?.spiral; context.previousPlanID = (plan ?? advisoryPlan)?.identifier
    // Preserve winding through an on-route exit/unload, but allow a genuine
    // reacquisition after leaving the route instead of latching continuation forever.
    context.onSpiral = (hacEstablished || phase == .headingAlignment) && !hacCompleted && tracksCurrentRoute
    context.liftScale = authority.liftEffectiveness; context.dragScale = authority.dragEffectiveness
    context.rollResponseSeconds = controlFeedback?.rollResponseSeconds
    context.pitchResponseSeconds = controlFeedback?.pitchResponseSeconds
    context.trimAlphaCeiling = attainableAlphaCeiling
    return context
  }
  private func terminalSearchDue(_ state: VehicleState) -> Bool {
    // HAC previews advise MM304 only; TAEM owns MM305 without a certificate.
    phase == .entry && !hacCompleted
      && (plan != nil || inTerminalSearchEnvelope(state))
      && state.time - lastPlanAttempt >= configuration.guidance.terminalReplanInterval
  }
  public func shouldPrioritizeTerminalSearch(_ state: VehicleState) -> Bool {
    // Distant advisory exploration runs when the worker is available. It must
    // not repeatedly cancel MM304's entry forecast while a complete terminal
    // route cannot even fit within the configured geometric horizon.
    terminalSearchDue(state) && (forcedTerminalOwnership || plan != nil
      || frame.distance(from: state.position, to: frame.origin) <= configuration.guidance.maximumPathLength)
  }
  public func planningRequest(from state: VehicleState, controls: FlightControlSnapshot?,
                              allowEntry: Bool = true, allowTerminal: Bool = true) -> MissionPlanningRequest? {
    guard externalPlanning else { return nil }
    entry.environment = environment
    if allowTerminal && [.approach, .preflare, .flare].contains(phase),
       let request = approach.planningRequest() {
      return .init(time: state.time, entry: nil, terminalState: nil, controls: nil,
        previousTarget: nil, approach: request, environment: environment)
    }
    guard [.entry, .acquisition, .headingAlignment].contains(phase), !hacCompleted else { return nil }
    var terminalState: VehicleState?
    if allowTerminal && terminalSearchDue(state) {
      terminalState = state; lastPlanAttempt = state.time
    }
    // Separate entry and terminal workers can update independently. Neither
    // consumes the other's request epoch or cancels its trajectory rollout.
    let entryRequest = allowEntry && phase == .entry && plan == nil
      ? entry.request(from: state, controls: controls, previousTarget: limiter.previous) : nil
    guard entryRequest != nil || terminalState != nil else { return nil }
    return .init(time: state.time, entry: entryRequest, terminalState: terminalState,
      controls: controls, previousTarget: limiter.previous, environment: environment, hac: hacContext())
  }
  public func accept(_ proposal: MissionPlanningProposal, state: VehicleState) {
    guard proposal.time <= state.time else { return }
    if let result = proposal.approach { approach.accept(result) }
    guard [.entry, .acquisition, .headingAlignment].contains(phase), !hacCompleted else { return }
    if phase == .entry, plan == nil, let result = proposal.entry { entry.accept(result, at: state.time) }
    // A stale terminal worker may complete after a newer state (or even a
    // newer terminal proposal) has already been accepted. Admission rejects
    // an old plan, but acceptTerminal also updates MM304's advisory and the
    // last physical verdict before admission; do not let an expired result
    // overwrite either of them.
    if let result = proposal.terminal,
      state.time - proposal.time <= 2 * configuration.guidance.terminalReplanInterval,
      publishedTerminalEpoch.map({ proposal.time > $0 }) ?? true {
      publishedTerminalEpoch = proposal.time
      acceptTerminal(result, state: state)
    }
  }
  private func acceptTerminal(_ result: PlanningResult, state: VehicleState) {
    lastPlanningResult = result
    var verdict = TAEMCaptureEvaluator.assess(result, state: state, configuration: configuration)
    verdict.plan = nil; lastCapture = verdict
    advisoryPlan = result.plan
    // An infeasible preview stays a diagnostic. It must never overwrite
    // MM304's steering or energy path with an unachievable terminal reference.
    if plan == nil {
      let steeringCandidate = result.plan.flatMap {
        $0.assessment.feasible && $0.assessment.predictionComplete != false ? $0 : nil
      }
      entry.considerTerminal(steeringCandidate, deficit: verdict.deficit)
    }
    guard let candidate = result.plan else { return }
    let trim = RotationalDynamicsEngine(configuration: configuration.vehicle, planet: configuration.planet)
      .supportedTrimAlpha(state, identifiedCeiling: attainableAlphaCeiling)
    let available = aerodynamics.liftSolution(acceleration: 0, mass: state.mass,
      dynamicPressure: state.aerodynamics.dynamicPressure, mach: state.aerodynamics.mach,
      minimumAlpha: configuration.vehicle.minimumAngleOfAttack, maximumAlpha: trim,
      beta: state.aerodynamics.sideslip, gear: state.systems.gearDown).maximumAcceleration * authority.liftEffectiveness
    let admission = MM305Admission.evaluate(state, plan: candidate,
      configuration: configuration, availableLift: available)
    lastAdmission = admission
    // NASA TAEM: a HAC preview never takes control. MM304 hands over on
    // Mach and TAEM regenerates its ground track every cycle.
  }
  /// Keep the active route and target limiter, but restart the sample clock.
  public func resumeAfterTelemetryGap() { previousTime = nil }
  private func prepareUpdate(_ state: VehicleState, advisories: inout [ConstraintViolation],
                             dt: inout Double) throws -> GuidanceSolution? {
    try state.validate(configuration: configuration)
    // Guidance never abandons the vehicle on its own judgement: every
    // envelope, energy or contact verdict is reported as an advisory while
    // the best available landing law keeps flying, including after reported
    // crash contact where remaining controls may still be useful.
    advisories = invalidationFailure.map { [$0] } ?? []
    entry.environment = environment
    if previousTime == nil && [.approach, .preflare, .flare].contains(phase), !approach.canCapture(state) {
      advisories.append(.init(.modelUncertainty, "Initial Final capture/energy model advisory"))
    }
    if previousTime == nil && [.acquisition, .headingAlignment].contains(phase) {
      if plan == nil {
        acceptTerminal(planner.plan(from: state, allowDirect: !hacContext().onSpiral,
          controlSnapshot: controlSnapshot, previousTarget: limiter.previous), state: state)
      }
      if let plan { lastAdmission = MM305Admission.evaluate(state, plan: plan, configuration: configuration) }
    }
    authority.observe(state)
    if let t = previousTime, state.time <= t { throw FlightError.staleTelemetry }
    dt = previousTime.map { state.time - $0 } ?? configuration.guidance.controlInterval
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
      crashObserved = true
      phase = .touchdown
      stoppedSince = nil
      advisories.append(.init(.contactQuality, "Crash contact observed; continuing recovery"))
      return .init(phase: phase, target: .init(gearDown: true),
        violations: advisories, planID: plan?.identifier)
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
      if !crashObserved && touchdown.accepted
        && configuration.runway.contains(along: p.along, cross: p.cross)
        && state.horizontalSpeed < 0.5 && state.wheelContact?.noseGear == true
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
    // A bounce breaks the required continuous ground-stop interval.
    stoppedSince = nil
    wasAirborne = true
    if phase == .complete { return .init(phase: phase, target: .init()) }
    if phase == .abort { phase = .entry }
    return nil
  }

  public func update(_ state: VehicleState) throws -> GuidanceSolution {
    var advisories: [ConstraintViolation] = []
    var dt = configuration.guidance.controlInterval
    if let completed = try prepareUpdate(state, advisories: &advisories, dt: &dt) { return completed }
    incidence.observe(time: state.time, measuredAlpha: state.aerodynamics.angleOfAttack,
      commandedAlpha: limiter.previous?.angleOfAttack, feedback: controlFeedback,
      fallbackResponseSeconds: configuration.guidance.pathLookaheadTime)
    let p = frame.coordinates(of: state.position)
    if Self.terminalHandoffDue(phase: phase, plan: plan, state: state, configuration: configuration,
      objective: entry.captureObjective(at: state.time)) {
      phase = .acquisition
      taem.reset()
      forcedTerminalOwnership = true
      terminalSearchArmed = true
      lastPlanAttempt = -.infinity
    }
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
    // Grounded states returned above. If contact is lost after touchdown
    // or rollout, Final retains ownership; do not reopen MM304/MM305 search.
    if [.approach, .preflare, .flare, .touchdown, .rollout].contains(phase)
      || (plan == nil && approach.canCapture(state))
    {
      raw = approach.update(
        state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
        pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
        identifiedTrimCeiling: attainableAlphaCeiling)
    } else {
      if !externalPlanning && phase == .entry && !hacCompleted && inTerminalSearchEnvelope(state)
        && state.time - lastPlanAttempt >= configuration.guidance.terminalReplanInterval
      {
        let context = hacContext()
        let currentPlanner = HACPlanningEngine(configuration: configuration,
          atmosphere: atmosphere.corrected(environment),
          aerodynamics: aerodynamics.scaled(lift: context.liftScale, drag: context.dragScale))
        let preview = currentPlanner.plan(
          from: state, allowDirect: !hacContext().onSpiral, preview: true,
          controlSnapshot: controlSnapshot, previousTarget: limiter.previous, context: context)
        let result: PlanningResult
        if let candidate = preview.plan,
           MM305Admission.shouldQualifyPreview(state, plan: candidate, configuration: configuration) {
          result = currentPlanner.plan(from: state, allowDirect: !hacContext().onSpiral, preview: false,
            controlSnapshot: controlSnapshot, previousTarget: limiter.previous, context: context)
        } else {
          result = preview
        }
        lastPlanningResult = result
        lastPlanAttempt = state.time
        acceptTerminal(result, state: state)
      }
      if phase == .acquisition || phase == .headingAlignment {
        if state.time - (nominalEnergyPoint?.time ?? -.infinity) >= configuration.guidance.terminalReplanInterval {
          // The NEP is a runway-fixed station; a refresh that fails from a
          // transient state (an S-turn bank) keeps the last solved one.
          nominalEnergyPoint = (state.time, nepSolver.solve(from: state, trimAlphaCeiling: attainableAlphaCeiling)
            ?? nominalEnergyPoint?.point)
        }
        if let nep = nominalEnergyPoint?.point, !approach.recoveryDue(state) {
          let target = taem.update(state, nep: nep, liftGain: authority.liftEffectiveness,
            dragGain: authority.dragEffectiveness, attainableAlphaCeiling: attainableAlphaCeiling,
            pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
            rollResponseSeconds: controlFeedback?.rollResponseSeconds)
          let s = taem.status
          var violations: [ConstraintViolation] = []
          if s.insufficientEnergy {
            violations.append(.init(.insufficientEnergy,
              "TAEM energy below best-L/D envelope; planned descent remains unqualified",
              value: s.energyHeight, limit: s.minimumEnergy))
          }
          if !s.descentPlanFits {
            violations.append(.init(.invalidGeometry,
              "No TAEM descent plan fits the configured glide and geometry limits; continuing recovery"))
          }
          if let failure = s.descentCheckFailure {
            violations.append(.init(.invalidGeometry,
              "TAEM committed-profile force check failed: \(failure); recovery remains unqualified"))
          }
          raw = .init(phase: s.phase == .headingAlignment ? .headingAlignment : .acquisition,
            target: target, violations: violations, pathRemaining: s.range,
            diagnostic: "TAEM phase=\(s.phase.rawValue) side=\(s.side) hacTurn=\(degrees(s.remainingTurn)) deg rpred=\(s.range) m E=\(s.energyHeight) m EN=\(s.nominalEnergy) m EMIN=\(s.minimumEnergy) m href=\(s.referenceHeight) m qref=\(s.referenceDynamicPressure) Pa qmin=\(s.minimumDynamicPressure) Pa rf=\(s.finalRadius) m growth=\(s.growth) m/rad2 downmoded=\(s.downmoded) descentFits=\(s.descentPlanFits) descentCheck=\(s.descentCheckFailure ?? "necessaryChecksPassed") referenceTracking=\(s.referenceTracking ?? false)")
        } else if approach.recoveryDue(state) {
          // Below the configured Final inlet height, keep a best-effort
          // landing owner even if normal NEP capture was not achieved.
          hacCompleted = true
          raw = approach.update(
            state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
            pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
            identifiedTrimCeiling: attainableAlphaCeiling)
          raw.violations.append(.init(.invalidGeometry,
            nominalEnergyPoint?.point == nil ? "No solved NEP; Final law flies without TAEM references"
              : "Below Final inlet height without measured capture; Final recovery unqualified"))
        } else {
          // A missing NEP at altitude is a planning failure, not permission
          // to apply the runway flare/final law to a high-energy vehicle.
          raw = .init(phase: .acquisition,
            target: .init(angleOfAttack: aerodynamics.bestGlideAlpha(mach: state.aerodynamics.mach,
              maximumAlpha: attainableAlphaCeiling ?? configuration.vehicle.maximumAngleOfAttack,
              dynamicPressure: state.aerodynamics.dynamicPressure), bank: 0,
              desiredHeight: p.height + configuration.runway.elevation),
            violations: [.init(.invalidGeometry, "No solved NEP; retaining TAEM best-glide recovery")])
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
            identifiedTrimCeiling: attainableAlphaCeiling)
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
    var requested = raw.target
    let brakeLift = TerminalBrakeAuthority.availableLift(state, configuration: configuration,
      aerodynamics: aerodynamics, identifiedTrimCeiling: attainableAlphaCeiling,
      liftEffectiveness: authority.liftEffectiveness)
    if [.acquisition, .headingAlignment].contains(phase) {
      if taem.status.referenceTracking == true, let nep = nominalEnergyPoint?.point {
        requested.speedbrakeFraction = speedbrake.feedback(state, target: nep,
          remainingDistance: taem.status.range, dt: dt,
          responseSeconds: max(configuration.guidance.pathLookaheadTime, controlFeedback?.pitchResponseSeconds ?? 0),
          availableLiftAcceleration: brakeLift)
      } else { requested.speedbrakeFraction = 0 }
    } else if [.approach, .preflare, .flare].contains(phase) {
      let endpoint = NominalEnergyPoint(along: configuration.guidance.touchdownAlong,
        height: configuration.vehicle.gearHeight, flightPathAngle: 0,
        airspeed: configuration.vehicle.landingSpeed,
        predictedTouchdownSpeed: configuration.vehicle.landingSpeed)
      requested.speedbrakeFraction = speedbrake.feedback(state, target: endpoint,
        remainingDistance: endpoint.along-p.along, dt: dt,
        responseSeconds: max(configuration.guidance.pathLookaheadTime, controlFeedback?.pitchResponseSeconds ?? 0),
        availableLiftAcceleration: brakeLift)
    }
    let target = limiter.update(requested, phase: raw.phase, configuration: configuration, dt: dt,
      dynamicPressure: state.aerodynamics.dynamicPressure,
      rollResponseSeconds: controlFeedback?.rollResponseSeconds,
      pitchResponseSeconds: controlFeedback?.pitchResponseSeconds,
      controlAuthority: state.controlAuthority)
    var solution = GuidanceSolution(
      phase: raw.phase, target: target, violations: raw.violations, planID: raw.planID,
      pathRemaining: raw.pathRemaining, diagnostic: raw.diagnostic)
    solution.violations += advisories
    // The airborne NEP continues to steer TAEM even when the selected
    // pavement cannot support its modeled rollout. Keep that deficit visible
    // through Final without conflating recovery ownership with qualification.
    solution.violations += nominalEnergyPoint?.point?.finalAssessment?.rolloutViolations ?? []
    solution.entry = raw.entry
    solution.approach = raw.approach
    if phase == .entry || phase == .acquisition || phase == .headingAlignment || phase == .abort {
      solution.capture = handoffDiagnostics(state: state)
    }
    if let admission = lastAdmission {
      var value = TerminalAdmissionDiagnostics()
      value.accepted = admission.accepted; value.altitudeMargin = admission.altitudeMargin
      value.machMargin = admission.machMargin; value.headingMargin = admission.headingMargin
      value.planAge = admission.planAge; value.violations = admission.violations
      value.physicallyFeasible = admission.physicallyFeasible
      value.positionMargin = admission.positionMargin
      value.controlOwnershipTransferred = plan != nil
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

  private func handoffDiagnostics(state: VehicleState) -> CaptureHandoffDiagnostics {
    var d = CaptureHandoffDiagnostics()
    let measured = frame.coordinates(of: state.position)
    d.actualAlong = measured.along
    d.actualCross = measured.cross
    d.actualRunwayRange = frame.distance(from: state.position, to: frame.origin)
    d.plannedCrossAim = entry.crossAim
    if let nep = nominalEnergyPoint?.point ?? (plan ?? advisoryPlan)?.nominalEnergyPoint
      ?? lastPlanningResult?.nominalEnergyPoint {
      d.nominalEnergyAlong = nep.along; d.nominalEnergyHeight = nep.height
      d.nominalEnergyAirspeed = nep.airspeed; d.nominalEnergyFlightPath = nep.flightPathAngle
      d.nominalEnergyPredictedTouchdownSpeed = nep.predictedTouchdownSpeed
    }
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
      d.mm305SearchHorizonExceeded = capture.envelope.searchHorizonExceeded
      d.mm305MinimumRequiredLength = capture.envelope.minimumRequiredPathLength
      d.mm305MaximumSearchedLength = capture.envelope.maximumSearchedPathLength
      d.mm305TurnAuthorityAssessed = capture.envelope.turnAuthorityAssessed
      d.mm305Violations = Array(capture.violations.prefix(6))
    }
    d.mm305PlanAccepted = plan != nil
    d.advisoryPlanID = advisoryPlan?.identifier
    d.advisoryPlanFeasible = advisoryPlan?.assessment.feasible
    // Publish only the priced prefix of an advisory. Unpriced geometry is not
    // a flight prediction: in particular it has zero placeholder speed and
    // must not acquire invented arrival times after energy exhaustion.
    if plan == nil, let advisory = advisoryPlan, advisory.stations.count > 1 {
      let stations = advisory.stations
      let stride = max(1, Int(ceil(Double(stations.count) / 48.0)))
      var track: [[Double]] = [], elapsed = 0.0
      var previous: PathStation?
      for (index, station) in stations.enumerated() {
        guard station.speed > 0,
          station.distance <= (advisory.assessment.predictedThroughDistance ?? advisory.length) else { break }
        if let p = previous {
          let speed = p.speed * cos(atan(p.slope))
          guard speed > 0 else { break }
          elapsed += max(0, station.distance - p.distance) / speed
        }
        previous = station
        if index % stride != 0 && index != stations.count - 1 { continue }
        let geo = GeodeticPosition(position: frame.position(
          along: station.position.x, cross: station.position.y, height: station.height),
          radius: configuration.planet.radius)
        guard geo.latitude.isFinite, geo.longitude.isFinite, geo.altitude.isFinite else { continue }
        track.append([advisory.createdAt + elapsed,
          degrees(geo.latitude), degrees(geo.longitude), geo.altitude, station.speed])
      }
      if track.count > 1 { d.advisoryReference = track }
    }
    if let plan {
      d.activePlanID = plan.identifier; d.activePlanCreatedAt = plan.createdAt
      d.activeHACRadius = plan.spiral?.finalRadius; d.activeHACGrowth = plan.spiral?.growth
      d.activeHACTurnAngle = plan.spiral?.turnAngle; d.activeHACSide = plan.spiral?.side
      d.activeHACExitAlong = plan.spiral?.exitAlong
      d.trackingError = tracker.trackingError; d.referenceHeadingError = tracker.headingError
      d.lookaheadDistance = tracker.lookaheadDistance; d.lookaheadHeadingError = tracker.lookaheadHeadingError
      if let ref = tracker.reference {
        d.referenceHeight = ref.height; d.referenceSegment = ref.segment.rawValue
        d.referenceSpeed = activePlanDynamicsCurrent && ref.speed > 0 ? ref.speed : nil
        d.referenceSpeedbrakeFraction = ref.speedbrakeFraction
      }
      d.activeReferencePredictionComplete = plan.assessment.predictionComplete
      d.activeReferenceDynamicsCurrent = activePlanDynamicsCurrent
      d.activeReferencePredictedThroughDistance = plan.assessment.predictedThroughDistance
      d.referenceLateralDemand = tracker.lateralAccelerationDemand
      d.allocatedLateralAcceleration = tracker.allocatedLateralAcceleration
      if activePlanDynamicsCurrent {
        // Immutable per-plan output: do not rebuild the same route every control tick.
        if activeReferenceCache == nil {
          let stride = max(1, Int(ceil(Double(plan.stations.count)*configuration.guidance.controlInterval
            / configuration.guidance.pathLookaheadTime)))
          var reference: [[Double]] = [], elapsed = 0.0
          var previousStation: PathStation?
          for (index, station) in plan.stations.enumerated() {
            guard station.speed > 0,
              station.distance <= (plan.assessment.predictedThroughDistance ?? plan.length) else { break }
            if let previous = previousStation {
              elapsed += (station.distance-previous.distance) / (previous.speed * cos(atan(previous.slope)))
            }
            previousStation = station
            if index % stride == 0 || index == plan.stations.count-1 {
              let geo = GeodeticPosition(position: frame.position(along: station.position.x,
                cross: station.position.y, height: station.height), radius: configuration.planet.radius)
              reference.append([plan.createdAt+elapsed, degrees(geo.latitude), degrees(geo.longitude), geo.altitude, station.speed])
            }
          }
          activeReferenceCache = reference
        }
        d.activeReference = activeReferenceCache
      }
    }
    if let admission = lastAdmission, !admission.violations.isEmpty {
      d.mm305Advisory = admission.violations.map(\.message).joined(separator: "; ")
    } else if let capture = lastCapture, !capture.feasible {
      d.mm305Advisory = "HAC model advisory: \(capture.deficit.rawValue)"
    }
    return d
  }
}
