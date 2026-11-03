import Foundation
import KSLCore

/// Native reconstruction of f1eebdc quaternion FCS; no transport/planner ownership.
public final class FlightController {
  private let configuration: MissionConfiguration
  private var rollout: RolloutController
  private var pitchAuthority = AxisAuthority(), rollAuthority = AxisAuthority()
  private var yawAuthority = AxisAuthority(), betaModel = SideslipResponseModel()
  private var pitchTrim = 0.0, rollTrim = 0.0, pitchScale = 0.70
  private var previous = ControlCommand()
  private var previousError: Vector3?
  private var errorRate = Vector3.zero
  // The entry RCS pitch damper uses measured body rate and a filtered wind-frame
  // target rate. Differentiating a sub-degree quaternion error at 10 Hz turned
  // attitude/target noise into large alternating RCS commands in live entry.
  private var previousDesiredAttitude: Quaternion?
  private var pitchReferenceRate = 0.0
  private var previousState: VehicleState?
  private var previousPhase: MissionPhase?
  private var mainContactLatched = false
  private var trimModel = AdaptiveTrim()
  private var previousFeedforward = 0.0
  private var feedforwardWasIdentified = false
  private var feedforwardWasPrior = false
  private var coupledRollAcceleration = 0.0
  /// TAEM has handed attitude to measured non-RCS authority; never re-armed.
  private var reactionControlReleased = false
  public private(set) var diagnostics = ControlDiagnostics()

  public init(configuration: MissionConfiguration, snapshot: FlightControlSnapshot? = nil) {
    self.configuration = configuration
    rollout = .init(configuration: configuration)
    if let s = snapshot {
      pitchAuthority = s.pitchAuthority; rollAuthority = s.rollAuthority; yawAuthority = s.yawAuthority
      betaModel = s.betaModel; pitchTrim = s.pitchTrim; rollTrim = s.rollTrim; pitchScale = s.pitchScale
      previous = s.previous; previousError = s.previousError; errorRate = s.errorRate
      previousDesiredAttitude = s.previousDesiredAttitude
      pitchReferenceRate = s.pitchReferenceRate
      previousState = s.previousState; previousPhase = s.previousPhase; trimModel = s.trimModel
      previousFeedforward = s.previousFeedforward; feedforwardWasIdentified = s.feedforwardWasIdentified
      feedforwardWasPrior = s.feedforwardWasPrior
      coupledRollAcceleration = s.coupledRollAcceleration
      reactionControlReleased = s.reactionControlReleased
      diagnostics = s.diagnostics; mainContactLatched = s.mainContactLatched
      rollout = .init(configuration: configuration, snapshot: s.rollout)
    }
  }
  public var snapshot: FlightControlSnapshot {
    .init(diagnostics: diagnostics, mainContactLatched: mainContactLatched, rollout: rollout.snapshot,
      pitchAuthority: pitchAuthority, rollAuthority: rollAuthority, yawAuthority: yawAuthority,
      betaModel: betaModel, pitchTrim: pitchTrim, rollTrim: rollTrim, pitchScale: pitchScale,
      previous: previous, previousError: previousError, errorRate: errorRate,
      previousDesiredAttitude: previousDesiredAttitude, pitchReferenceRate: pitchReferenceRate,
      previousState: previousState, previousPhase: previousPhase, trimModel: trimModel,
      previousFeedforward: previousFeedforward, feedforwardWasIdentified: feedforwardWasIdentified,
      feedforwardWasPrior: feedforwardWasPrior, coupledRollAcceleration: coupledRollAcceleration,
      reactionControlReleased: reactionControlReleased)
  }
  /// Configured RCS stays on through MM304 and into TAEM until measured
  /// non-RCS (surface and wheel) authority covers this attitude law's own
  /// demand on every axis: the bandwidth times the configured maximum pitch
  /// rate and TAEM roll rate, with the body yaw a stability-axis roll needs
  /// at the measured AoA. Release then latches; approach and flare never use RCS.
  /// Missing non-RCS evidence keeps RCS rather than guessing authority.
  public func reactionControl(state: VehicleState, phase: MissionPhase) -> Bool {
    guard configuration.vehicle.rcsAttitudeControl else { return false }
    if phase == .entry { return true }
    guard [.acquisition, .headingAlignment].contains(phase), !reactionControlReleased else { return false }
    let report = state.systems.rcs ? state.controlAuthority?.excludingRCS : state.controlAuthority
    guard let report, report.isValid, report.evidence == .measured else { return true }
    let a = state.aerodynamics, p = configuration.vehicle.controlPlant
    let wn = ControlLaw.bandwidth(q: a.dynamicPressure, mach: a.mach, terminal: false)
    let rollRate = min(p.maximumRollRate,
      configuration.guidance.terminalRollRate ?? configuration.guidance.bankSlewRate)
    let roll = wn.roll * rollRate
    let required = Vector3(roll, wn.pitch * p.maximumPitchRate, roll * abs(sin(a.angleOfAttack)))
    let available = report.minimumAcceleration
    if available.x >= required.x, available.y >= required.y, available.z >= required.z {
      reactionControlReleased = true
    }
    return !reactionControlReleased
  }
  public func reset() {
    pitchAuthority = .init(); rollAuthority = .init(); yawAuthority = .init()
    betaModel = .init(); pitchTrim = 0; rollTrim = 0; pitchScale = 0.70
    previous = .init(); previousError = nil; errorRate = .zero
    previousDesiredAttitude = nil; pitchReferenceRate = 0
    previousState = nil; previousPhase = nil; mainContactLatched = false
    trimModel = .init(); previousFeedforward = 0; feedforwardWasIdentified = false; feedforwardWasPrior = false
    coupledRollAcceleration = 0
    diagnostics = .init(); rollout = .init(configuration: configuration)
    reactionControlReleased = false
  }
  /// Drop the old mass-distribution fit without stepping the actuators to zero.
  public func invalidateDynamics(at state: VehicleState) {
    let applied = state.appliedControls ?? previous
    let contacted = mainContactLatched
    reset()
    previous = applied; mainContactLatched = contacted
  }
  /// Preserve the identified plant and trims; discard derivatives across missing samples.
  public func resumeAfterTelemetryGap(at state: VehicleState) {
    previousState = nil; previousError = nil; errorRate = .zero
    previousDesiredAttitude = nil; pitchReferenceRate = 0
    previous = state.appliedControls ?? previous
  }
  /// Continuous propellant depletion may move the CoM by 1 cm over several
  /// minutes, reaching the cumulative revision threshold without changing the
  /// control plant abruptly. Keep its live trim and authority fit. Real payload,
  /// topology, inertia or stepwise CoM changes still invalidate that evidence.
  public func adaptToDynamicsChange(_ change: VehicleDynamicsChange, at state: VehicleState) {
    let gradualCoMDrift: Bool
    if change.reasons == ["center-of-mass-shifted"], let old = previousState,
       let a = old.vesselConfiguration?.centerOfMassFromAnchor,
       let b = state.vesselConfiguration?.centerOfMassFromAnchor {
      gradualCoMDrift = state.time > old.time && state.time-old.time <= 1
        && state.mass <= old.mass && (b-a).length < 0.001
    } else { gradualCoMDrift = false }
    if gradualCoMDrift { resumeAfterTelemetryGap(at: state) }
    else { invalidateDynamics(at: state) }
  }
  public func command(state: VehicleState, solution: GuidanceSolution, dt: Double) throws
    -> ControlCommand {
    guard dt.isFinite, dt > 0, dt <= 1 else { throw FlightError.staleTelemetry }
    if let old = previousState, state.time <= old.time {
      throw FlightError.invalidTelemetry("Time moved backwards or repeated in FCS")
    }
    if state.wheelContact?.mainGear == true { mainContactLatched = true }
    if state.contact.isGrounded || state.contact == .crashed {
      pitchTrim = 0; rollTrim = 0
      previous = rollout.command(state: state, dt: dt)
      diagnostics = .init()
      diagnostics.rollout = rollout.diagnostics
      diagnostics.sampleInterval = dt
      diagnostics.appliedDemand = .init(previous.roll, previous.pitch, previous.yaw)
      previousState = state
      return previous
    }
    if solution.phase == .complete {
      previous = .init(gearDown: state.systems.gearDown)
      return previous
    }
    let a = state.aerodynamics
    guard let desired = AirRelativeAttitude.target(
      position: state.position, airVelocity: state.velocity - state.atmosphere.wind,
      alpha: solution.target.angleOfAttack, bank: solution.target.bank) else {
      throw FlightError.invalidTelemetry("Air-relative attitude is undefined")
    }
    let error = AirRelativeAttitude.error(current: state.attitude, target: desired)
    // The reference includes motion of the wind frame as well as commanded
    // incidence. The target quaternion is planet-fixed and the measured gyro
    // is planet-relative; their rates are therefore in the same frame. Rotate
    // the finite target increment into the current body frame before damping.
    if previousPhase == solution.phase, previousState != nil,
       let prior = previousDesiredAttitude {
      let increment = AirRelativeAttitude.error(current: prior, target: desired)
      let inBody = (state.attitude.conjugate * prior).rotate(increment)
      let limit = configuration.vehicle.controlPlant.maximumPitchRate
      let sampleRate = clamp(inBody.y / dt, -limit, limit)
      pitchReferenceRate = ControlLaw.lowpass(pitchReferenceRate, sampleRate,
        dt: dt, tau: 3 * ControlLaw.pitchLag)
    } else { pitchReferenceRate = 0 }
    previousDesiredAttitude = desired
    let aerodynamicError = wrapAngle(solution.target.angleOfAttack - a.angleOfAttack)
    if previousPhase == solution.phase, let previousError {
      errorRate = .init(
        ControlLaw.lowpass(errorRate.x, wrapAngle(error.x - previousError.x) / dt,
                           dt: dt, tau: ControlLaw.rollLag),
        ControlLaw.lowpass(errorRate.y, wrapAngle(error.y - previousError.y) / dt,
                           dt: dt, tau: ControlLaw.pitchLag),
        ControlLaw.lowpass(errorRate.z, wrapAngle(error.z - previousError.z) / dt,
                           dt: dt, tau: ControlLaw.yawLag))
    } else { errorRate = .zero }
    let aeroRate = previousState.map {
      Vector3(wrapAngle(a.bank - $0.aerodynamics.bank) / dt,
              wrapAngle(a.angleOfAttack - $0.aerodynamics.angleOfAttack) / dt,
              wrapAngle(a.sideslip - $0.aerodynamics.sideslip) / dt)
    } ?? .zero
    let observed = state.appliedControls ?? previous
    // A steady but off-target aircraft still identifies its *local* balancing
    // input. Requiring small target error before learning trim creates a
    // circular dependency: the missing trim causes the very error that blocks
    // identification. Retain C's strict terminal gate; during MM304 also admit
    // a directly observed moment equilibrium with <3% equivalent input
    // residual/change (the existing AdaptiveTrim observation noise reserve).
    let pitchAuthorityReport = state.controlAuthority?.minimumAcceleration.y ?? 0
    let measuredPitchAcceleration = previousState.map {
      (state.angularVelocity.y-$0.angularVelocity.y)/dt
    }
    let trimCoefficient = configuration.vehicle.bodyFlapTrimInputAtFullDeflection ?? 0
    let observedTrimInput = observed.pitch + (observed.bodyFlapTrimFraction ?? 0)*trimCoefficient
    let previousTrimInput = previousState.map {
      ($0.appliedControls?.pitch ?? 0) + ($0.appliedControls?.bodyFlapTrimFraction ?? 0)*trimCoefficient
    }
    let measuredMomentSettled = pitchAuthorityReport > 0
      && measuredPitchAcceleration.map { abs($0)/pitchAuthorityReport <= 0.03 } == true
      && previousTrimInput.map { abs(observedTrimInput-$0) <= 0.03 } == true
    let momentSettled = solution.phase == .entry && measuredMomentSettled
    let predictionSettled = [.entry, .acquisition, .headingAlignment].contains(solution.phase)
      && !state.systems.rcs && previousState?.systems.rcs == false
      && state.controlAuthority?.evidence == .measured && a.measuredForces
      && state.appliedControls != nil && previousState?.appliedControls != nil
      && abs(aeroRate.y) < radians(0.3) && measuredMomentSettled
    trimModel.observe(time: state.time, mach: a.mach, pressure: a.dynamicPressure,
      alpha: a.angleOfAttack, input: observedTrimInput,
      settled: !state.systems.rcs && previousState != nil
        && abs(aeroRate.y) < radians(0.3)
        && (abs(aerodynamicError) < radians(0.5) || momentSettled) && state.appliedControls != nil,
      predictionSettled: predictionSettled)
    pitchAuthority.observe(rate: state.angularVelocity.y, input: observed.pitch,
                           dt: dt, interference: 0, responseTime: 0)
    rollAuthority.observe(rate: state.angularVelocity.x, input: observed.roll,
                          dt: dt, interference: observed.yaw, responseTime: ControlLaw.rollLag)
    // Keep the yaw observer in body coordinates: beta acceleration is different.
    yawAuthority.observe(rate: state.angularVelocity.z, input: observed.yaw,
                         dt: dt, interference: observed.roll, responseTime: ControlLaw.yawLag)
    if previousState != nil {
      betaModel.observe(yaw: state.angularVelocity.z, roll: state.angularVelocity.x,
                        betaRate: aeroRate.z)
    }
    let reported = state.controlAuthority?.minimumAcceleration ?? .zero
    pitchAuthority.reported = reported.y; rollAuthority.reported = reported.x
    yawAuthority.reported = reported.z
    let reportAvailable = state.controlAuthority?.isValid == true
    let p = configuration.vehicle.controlPlant
    let kPa = max(0, a.dynamicPressure / 1000)
    let fallbackAllowed = state.controlAuthority == nil || state.controlAuthority?.evidence == .provisional
    let fallbackScale = fallbackAllowed ? 1.0 : 0.0
    let pitch = pitchAuthority.available(
      fallback: fallbackScale * min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa),
      blendLive: false, reportedAvailable: reportAvailable)
    let roll = rollAuthority.available(
      fallback: fallbackScale * min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa),
      blendLive: true, reportedAvailable: reportAvailable)
    let yaw = yawAuthority.available(
      fallback: fallbackScale * min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa),
      blendLive: true, reportedAvailable: reportAvailable)
    var usedPitch = min(pitch, radians(40))
    let usedRoll = min(roll, radians(100)), usedYaw = min(yaw, radians(30))
    let terminal = [.approach, .preflare, .flare].contains(solution.phase)
    let adaptive = !terminal
    let frequencies = ControlLaw.bandwidth(q: a.dynamicPressure, mach: a.mach, terminal: terminal)
    // MM304 has active RCS and a direct body gyro. Use it for damping rather
    // than the noisy derivative of a small quaternion residual. Keep the C
    // relative-error controller for RCS-off TAEM/landing until separately proven.
    let entryGyro = solution.phase == .entry && state.systems.rcs
    let measuredPitchRate = entryGyro ? state.angularVelocity.y : -errorRate.y
    let referencePitchRate = entryGyro ? pitchReferenceRate : 0
    let relativePitchRate = measuredPitchRate - referencePitchRate
    if adaptive {
      pitchScale = ControlLaw.responseScale(pitchScale, error: error.y, rate: relativePitchRate,
                                           wn: frequencies.pitch, dt: dt)
    } else { pitchScale = 1 }
    let pitchWN = frequencies.pitch * pitchScale, rollWN = frequencies.roll
    let pitchDamping = adaptive ? clamp(ControlLaw.damping / pitchScale, 1.15, 2.4) : 1.15
    // Identified local trim takes precedence. The archive-derived restoring
    // moment remains an explicit provisional prior outside its support.
    let identifiedTrim = state.systems.rcs ? nil : trimModel.input(
      time: state.time, mach: a.mach, pressure: a.dynamicPressure,
      alpha: a.angleOfAttack)
    let stiffness = p.pitchStiffnessPerKPa * kPa
    let momentPrior = stiffness * (a.angleOfAttack - p.trimAngleOfAttack)
      + 2 * p.pitchDampingRatio * sqrt(stiffness) * state.angularVelocity.y
    // The inherited direct-moment prior is STS-N subsonic data. It may only
    // balance that explicit provisional simulation plant, never real STS
    // telemetry or a high-Mach entry whose moment law is not identified.
    let supportedPrior = state.controlAuthority?.evidence == .provisional
      && a.mach <= 0.9 && a.dynamicPressure >= 500 && a.dynamicPressure <= 12000
    let feedforward = identifiedTrim ?? (supportedPrior && pitch > 1e-12 ? momentPrior/pitch : 0)
    // Bumpless transfer whenever the feedforward source changes (identified
    // trim <-> provisional prior <-> none), not only on identification.
    if (identifiedTrim != nil) != feedforwardWasIdentified
      || (identifiedTrim == nil && supportedPrior != feedforwardWasPrior) {
      pitchTrim -= feedforward - previousFeedforward
    }
    feedforwardWasPrior = identifiedTrim == nil && supportedPrior
    previousFeedforward = feedforward; feedforwardWasIdentified = identifiedTrim != nil
    var targetPitchRate = 0.0
    if solution.phase == .flare {
      let blend = clamp((120 - max(0, state.radarAltitude)) / 80, 0, 1)
      usedPitch = min(usedPitch, radians(40 + (16 - 40) * blend))
      if error.y > 0, state.verticalSpeed < -0.5 {
        let height = max(0.35, state.radarAltitude - configuration.vehicle.gearHeight)
        let timeToContact = max(0.40, height / max(0.5, -state.verticalSpeed))
        targetPitchRate = clamp(error.y / timeToContact, 0, radians(6))
      }
    }
    var pitchRaw = ControlLaw.demand(error: error.y, rate: measuredPitchRate,
      targetRate: targetPitchRate + referencePitchRate,
      authority: usedPitch, wn: pitchWN, damping: pitchDamping,
      lag: ControlLaw.pitchLag, previous: previous.pitch, balancingInput: feedforward + pitchTrim)
    let pdPitch = clamp(pitchRaw, -1, 1)
    // Offload measured steady pitch bias to the rear flap without disabling
    // its ordinary pitch input. With no identified deployment coefficient the
    // only safe automatic trim command is stowed, not a guessed sign/gain.
    let flapCoefficient = !state.systems.rcs && !mainContactLatched
      ? configuration.vehicle.bodyFlapTrimInputAtFullDeflection : nil
    let flapTrim = flapCoefficient.map {
      clamp((pitchTrim + feedforward) / $0, -1, 1)
    } ?? 0
    let flapPitchInput = flapTrim * (flapCoefficient ?? 0)
    if !mainContactLatched, usedPitch > 1e-12, a.dynamicPressure > 100 {
      let nearGround = terminal && state.radarAltitude < 700
      let taem = [.acquisition, .headingAlignment].contains(solution.phase)
      // TAEM: integral time constant equals the PD loop's response time
      // (2*zeta/wn + lag), which keeps the third-order loop ~4*zeta^2 inside
      // its Routh limit at every q/Mach. Converted from angular acceleration
      // to input units by the authority in use.
      let responseTime = 2 * pitchDamping / max(1e-3, pitchWN) + ControlLaw.pitchLag
      let taemGain = pitchWN * pitchWN / (responseTime * usedPitch)
      let gainPerRadian = taem && !nearGround ? taemGain : degrees(nearGround ? 0.025 : 0.008)
      let total = pdPitch + pitchTrim + feedforward - flapPitchInput
      let windup = (total >= 1 && aerodynamicError > 0) || (total <= -1 && aerodynamicError < 0)
      if !windup, abs(aerodynamicError) > radians(0.20) {
        pitchTrim += gainPerRadian * aerodynamicError * dt
      }
      pitchTrim = clamp(pitchTrim, nearGround ? -1 : taem ? -0.60 : -0.35,
                         nearGround ? 1 : taem ? 0.90 : 0.50)
    } else { pitchTrim *= max(0, 1 - dt) }
    pitchRaw = pdPitch + pitchTrim + feedforward - flapPitchInput
    var pitchCommand = clamp(pitchRaw, -1, 1)
    if adaptive {
      pitchCommand = ControlLaw.lowpass(previous.pitch, pitchCommand, dt: dt,
                                       tau: ControlLaw.pitchLag / pitchScale)
    }
    var rollRaw = ControlLaw.demand(error: error.x, rate: -errorRate.x,
      authority: usedRoll, wn: rollWN, damping: ControlLaw.damping,
      lag: ControlLaw.rollLag, previous: previous.roll)
    if terminal, usedRoll > 1e-12, rollWN > 0 {
      let responseTime = 2 * ControlLaw.damping / rollWN + ControlLaw.rollLag
      let increment = dt * rollWN * rollWN * error.x / (usedRoll * responseTime)
      let candidate = clamp(rollTrim + increment, -1, 1)
      if abs(rollRaw + candidate) <= 1 || (rollRaw + candidate) * increment < 0 { rollTrim = candidate }
      rollRaw += rollTrim
    } else { rollTrim = 0 }
    if let previousState {
      coupledRollAcceleration = ControlLaw.lowpass(coupledRollAcceleration,
        (state.angularVelocity.x - previousState.angularVelocity.x) / dt,
        dt: dt, tau: ControlLaw.rollLag)
    }
    // beta-dot = p*sin(alpha) - r*cos(alpha) plus motion of the wind frame.
    // Preserve body-yaw identification, then project its authority into beta.
    // This supplies roll/yaw coordination without a fabricated moment fit.
    let yawProjection = max(0, cos(a.angleOfAttack))
    let betaAuthority = usedYaw * yawProjection
    let coordination = betaAuthority > 1e-12
      ? coupledRollAcceleration * sin(a.angleOfAttack) / betaAuthority : 0
    let yawWN = min(ControlLaw.yawWN, sqrt(betaAuthority / max(abs(a.sideslip), 1e-12)),
                    1 / (2 * ControlLaw.yawLag))
    var yawRaw = ControlLaw.demand(error: a.sideslip, rate: -aeroRate.z,
      authority: betaAuthority, wn: yawWN, damping: 1, lag: ControlLaw.yawLag,
      previous: previous.yaw, balancingInput: coordination) + coordination
    // C's recovery rate-arrest law; trigger on an actual departure, not a label.
    let rateFirst = abs(error.x) > .pi / 2 || abs(a.sideslip) > .pi / 4
    let dampingExhausted = usedPitch > 0 && usedRoll > 0 && usedYaw > 0 && (
      2 * pitchDamping * pitchWN * abs(state.angularVelocity.y) >= usedPitch ||
      2 * ControlLaw.damping * rollWN * abs(state.angularVelocity.x) >= usedRoll ||
      2 * yawWN * abs(state.angularVelocity.z) >= usedYaw)
    if rateFirst && dampingExhausted {
      pitchCommand = clamp(ControlLaw.demand(error: 0, rate: state.angularVelocity.y,
        authority: usedPitch, wn: pitchWN, damping: pitchDamping, lag: 0, previous: 0), -1, 1)
      rollRaw = ControlLaw.demand(error: 0, rate: state.angularVelocity.x,
        authority: usedRoll, wn: rollWN, damping: ControlLaw.damping, lag: 0, previous: 0)
      yawRaw = ControlLaw.demand(error: 0, rate: state.angularVelocity.z,
        authority: usedYaw, wn: yawWN, damping: 1, lag: 0, previous: 0)
      pitchTrim = 0; rollTrim = 0
    }
    var pitchGuard = false
    if solution.phase == .flare && !mainContactLatched {
      let rate = max(0, state.angularVelocity.y)
      let stoppingAuthority = min(usedPitch, state.controlAuthority?.minimumAcceleration.y ?? usedPitch)
      if stoppingAuthority > 0 {
        let lag = ControlLaw.pitchLag + dt
        let stopDistance = rate * lag + rate * rate / (2 * stoppingAuthority)
        let margin = configuration.vehicle.maximumTouchdownPitch - state.pitch
        if margin <= stopDistance {
          pitchCommand = min(pitchCommand, clamp(feedforward - 1, -1, 1))
          pitchGuard = true
        }
      }
    }
    if mainContactLatched || usedPitch <= 1e-12 { pitchCommand = 0; pitchTrim = 0 }
    let delta = p.surfaceRate * dt
    previous = .init(
      pitch: moveToward(previous.pitch, pitchCommand, maximumDelta: delta),
      roll: moveToward(previous.roll, clamp(rollRaw, -1, 1), maximumDelta: delta),
      yaw: moveToward(previous.yaw, clamp(yawRaw, -1, 1), maximumDelta: delta),
      gearDown: solution.target.gearDown,
      rcs: reactionControl(state: state, phase: solution.phase),
      speedbrakeFraction: solution.target.speedbrakeFraction ?? 0,
      bodyFlapTrimFraction: flapTrim)
    var d = ControlDiagnostics()
    d.pitchSaturated = abs(pitchRaw) >= 1; d.rollSaturated = abs(rollRaw) >= 1
    d.yawSaturated = abs(yawRaw) >= 1; d.pitchError = aerodynamicError
    d.bankError = wrapAngle(solution.target.bank - a.bank); d.quaternionError = error
    d.relativeRate = .init(-errorRate.x, relativePitchRate, -errorRate.z)
    d.bodyRate = state.angularVelocity; d.aerodynamicRate = aeroRate
    d.authority = .init(usedRoll, usedPitch, usedYaw); d.reportedAuthority = reported
    d.authorityConfidence = .init(rollAuthority.liveConfidence, pitchAuthority.liveConfidence,
                                  yawAuthority.liveConfidence)
    d.authorityEvidence = state.controlAuthority?.evidence ?? .provisional
    d.pitchTrim = pitchTrim; d.rollTrim = rollTrim; d.pitchBandwidthScale = pitchScale
    d.pitchResponseSeconds = 2 * pitchDamping / max(0.1, pitchWN) + ControlLaw.pitchLag
    d.rollResponseSeconds = 2 * ControlLaw.damping / max(0.1, rollWN) + ControlLaw.rollLag
    d.pitchFeedforward = feedforward
    // A pointwise trim feedforward is not an identified pitch-trim envelope.
    // Never promote nominal entry AoA into a hard guidance ceiling merely
    // because the controller held that AoA steadily.
    let identifiedCeiling = state.systems.rcs ? nil : trimModel.ceiling(
      time: state.time, mach: a.mach, pressure: a.dynamicPressure,
      maximum: configuration.vehicle.maximumAngleOfAttack)
    d.trimEvidence = identifiedCeiling != nil ? .identified : supportedPrior ? .provisional : .unavailable
    d.trimAlphaCeiling = identifiedCeiling
    d.rawDemand = .init(rollRaw, pitchRaw, yawRaw)
    d.appliedDemand = .init(previous.roll, previous.pitch, previous.yaw)
    d.recoveryRateFirst = rateFirst && dampingExhausted; d.sampleInterval = dt
    d.touchdownPitchGuardActive = pitchGuard
    d.betaRollGain = betaModel.rollGain; d.betaYawGain = betaModel.yawGain
    d.betaConfidence = betaModel.confidence; diagnostics = d
    previousError = error; previousState = state; previousPhase = solution.phase
    try previous.validate(grounded: false)
    return previous
  }
}
