import Foundation
import KSLCore
import KSLControl
import KSLPhysics

/// Executes an already selected route using the real guidance, native FCS,
/// actuators and six-DOF plant through contact and stopping. No planner calls
/// itself recursively and no guidance target overwrites physical attitude.
public final class TerminalTrajectoryEvaluator {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
  }
  public func evaluate(plan: TerminalPlan, from initial: VehicleState,
                       controlSnapshot: FlightControlSnapshot?, previousTarget: GuidanceTarget?,
                       targetLimiter: GuidanceTargetLimiter? = nil,
                       nativeSeed: NativeCommandSeed? = nil,
                       lift: Double, drag: Double,
                       commandPrefix: TAEMSchedulePrefix? = nil,
                       trace: ((VehicleState, PathStation, GuidanceTarget) -> Void)? = nil,
                       cancelled: () -> Bool = { false }) -> TerminalClosedLoopCase {
    let c = configuration, dt = c.guidance.controlInterval
    let seed = commandPrefix?.seed ?? nativeSeed ?? NativeCommandSeed(state: initial, controls: controlSnapshot,
      limiter: targetLimiter ?? GuidanceTargetLimiter(previous: previousTarget))
    let model = lift == 1 && drag == 1 ? aerodynamics : aerodynamics.scaled(lift: lift, drag: drag)
    let tailContext = TerminalComparisonContext(seed: seed, configuration: c,
      atmosphere: atmosphere, aerodynamics: model)
    let comparisonContext = commandPrefix?.comparisonContext ?? tailContext
    let origin = comparisonContext.startState
    let prefixValid = commandPrefix.map { prefix in
      prefix.end == .scheduleExhausted && prefix.comparisonContext.originID == seed.originID
        && prefix.comparisonContext.configuration == c
        && prefix.comparisonContext.atmosphereID == atmosphere.instanceID
        && prefix.comparisonContext.aerodynamicsID == model.instanceID
        && prefix.comparisonContext.continuationPolicy == tailContext.continuationPolicy
        && prefix.seed.state == initial
    } ?? true
    let rollout = NativeCommandRollout(seed: seed, configuration: c, atmosphere: atmosphere, aerodynamics: model)
    let tracker = PathTrackingEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    let approach = ApproachGuidanceEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    let authority = AerodynamicAuthorityEstimator(model: aerodynamics)
    let touchdown = TouchdownMonitor(configuration: c)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    var state = rollout.state, phase = MissionPhase.acquisition
    var failures: [ConstraintViolation] = [], maximumTrackingError = 0.0
    var finalCaptured = false, stoppedSince: Double?, success = false
    var lastReference: PathStation?
    var evaluationStatus = TerminalEvaluationStatus.incomplete
    var end = TerminalEvaluationEnd.horizon
    var accounting = commandPrefix?.accounting ?? NativeTrajectoryAccounting()
    func add(_ violation: ConstraintViolation) {
      if !failures.contains(where: { $0.kind == violation.kind }) { failures.append(violation) }
    }
    func energy(_ s: VehicleState) -> Double {
      s.velocity.squaredLength/2-c.planet.gravitationalParameter/s.position.length
        - c.planet.angularVelocity.cross(s.position).squaredLength/2
    }
    if !prefixValid {
      add(.init(.modelUncertainty, "Command prefix is incomplete or its context/endpoint differs"))
      evaluationStatus = .invalid; end = .unsupportedDynamics
    }
    // As in entry propagation, discard only sub-resolution integration residue.
    while prefixValid && c.guidance.maximumPredictionDuration-(state.time-origin.time) > dt*1e-6 {
      if Task.isCancelled || cancelled() { end = .cancelled; break }
      guard rollout.modelConsistent, nativeSeed.map({
        $0.state.time == initial.time && $0.state.position == initial.position
          && $0.state.velocity == initial.velocity && $0.state.attitude == initial.attitude
          && $0.state.mass == initial.mass
      }) ?? true else {
        add(.init(.modelUncertainty, "Continuation seed/model does not match evaluation initial state"))
        evaluationStatus = .invalid; end = .unsupportedDynamics; break
      }
      if state.contact == .crashed {
        add(.init(.contactQuality, "Closed-loop route failed runway contact or containment"))
        evaluationStatus = .complete; end = .modeledCrash; break
      }
      if !state.contact.isGrounded && !rollout.pitchMomentSupported {
        add(.init(.modelUncertainty, "Terminal prediction lacks supported original pitch-moment evidence: \(rollout.pitchMomentSupport.rawValue)"))
        evaluationStatus = .invalid; end = .unsupportedDynamics; break
      }
      for violation in touchdown.observe(state) { add(violation) }
      if state.aerodynamics.dynamicPressure > c.vehicle.maximumDynamicPressure {
        add(.init(.excessiveDynamicPressure, "Closed-loop terminal q envelope"))
      }
      if abs(state.aerodynamics.lift)/(state.mass*9.80665) > c.vehicle.maximumLoadFactor {
        add(.init(.excessiveLoad, "Closed-loop terminal load envelope"))
      }
      authority.observe(state)
      let trim = rollout.diagnostics.trimEvidence == .identified ? rollout.diagnostics.trimAlphaCeiling : nil
      let raw: GuidanceSolution
      if state.contact.isGrounded {
        phase = state.wheelContact?.noseGear == true ? .rollout : .touchdown
        let p = frame.coordinates(of: state.position)
        if !c.runway.contains(along: p.along, cross: p.cross) {
          add(.init(.runwayExcursion, "Closed-loop rollout left runway"))
          evaluationStatus = .invalid; end = .unsupportedDynamics; break
        }
        if state.horizontalSpeed < 0.5 && state.wheelContact?.noseGear == true {
          if stoppedSince == nil { stoppedSince = state.time }
          if state.time-stoppedSince! >= c.guidance.stopConfirmationDuration {
            success = true; evaluationStatus = .complete; end = .sustainedStop; break
          }
        } else { stoppedSince = nil }
        raw = .init(phase: phase, target: .init(gearDown: true))
      } else if finalCaptured {
        raw = approach.update(state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
          pitchResponseSeconds: rollout.diagnostics.pitchResponseSeconds, identifiedTrimCeiling: trim)
      } else {
        let (target, reference) = tracker.update(state, plan: plan, liftEffectiveness: authority.liftEffectiveness,
          identifiedTrimCeiling: trim,
          rollResponseSeconds: rollout.diagnostics.rollResponseSeconds, pitchResponseSeconds: rollout.diagnostics.pitchResponseSeconds)
        lastReference = reference
        maximumTrackingError = max(maximumTrackingError, tracker.trackingError)
        if reference.segment == .final && approach.canCapture(state) {
          finalCaptured = true
          raw = approach.update(state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
            pitchResponseSeconds: rollout.diagnostics.pitchResponseSeconds, identifiedTrimCeiling: trim)
        } else {
          raw = .init(phase: reference.segment == .spiral ? .headingAlignment : .acquisition, target: target)
        }
      }
      phase = raw.phase
      do {
        let duration = min(dt, c.guidance.maximumPredictionDuration-(state.time-origin.time))
        let step = try rollout.advance(raw, duration: duration)
        accounting.include(step, configuration: c)
        if let reference = lastReference { trace?(step.before, reference, step.limited) }
        state = step.after
      } catch {
        add(.init(.actuatorAuthority, "Closed-loop terminal computation: \(error)"))
        evaluationStatus = .invalid; end = .computationFailure; break
      }
    }
    if evaluationStatus == .incomplete {
      failures.append(.init(.timeout, end == .cancelled ? "Terminal qualification cancelled" : "Terminal qualification did not complete rollout"))
    }
    if accounting.maximumPressureExcess > 0 { add(.init(.excessiveDynamicPressure, "Command/terminal trajectory q envelope")) }
    if accounting.maximumLoadExcess > 0 { add(.init(.excessiveLoad, "Command/terminal trajectory load envelope")) }
    let event = rollout.touchdown
    return .init(liftMultiplier: lift, dragMultiplier: drag, feasible: success && event?.acceptable == true && failures.isEmpty,
      elapsed: state.time-origin.time, touchdownSpeed: event?.horizontalSpeed,
      touchdownSink: event?.verticalSpeed, touchdownPitch: event?.pitch,
      stoppedAlong: success ? frame.coordinates(of: state.position).along : nil,
      maximumTrackingError: maximumTrackingError, violations: failures, evidence: "offline-native-control-model-only",
      evaluation: .init(status: evaluationStatus, end: end, state: state,
        aerodynamicWork: accounting.aerodynamicWork, energyChange: energy(state)-energy(origin),
        heightChange: state.position.length-origin.position.length,
        maximumPressureExcess: accounting.maximumPressureExcess, pressureExcessIntegral: accounting.pressureExcessIntegral,
        maximumLoadExcess: accounting.maximumLoadExcess, loadExcessIntegral: accounting.loadExcessIntegral,
        comparisonContext: comparisonContext, touchdown: event))
  }
}
