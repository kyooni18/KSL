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
                       lift: Double, drag: Double,
                       trace: ((VehicleState, PathStation, GuidanceTarget) -> Void)? = nil) -> TerminalClosedLoopCase {
    let c = configuration, dt = c.guidance.controlInterval
    let controller = FlightController(configuration: c, snapshot: controlSnapshot)
    let plant = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aerodynamics.scaled(lift: lift, drag: drag),
      initialControls: initial.appliedControls ?? .init())
    let tracker = PathTrackingEngine(configuration: c, aerodynamics: aerodynamics)
    let approach = ApproachGuidanceEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    let authority = AerodynamicAuthorityEstimator(model: aerodynamics)
    let touchdown = TouchdownMonitor(configuration: c)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    var limiter = GuidanceTargetLimiter(previous: previousTarget)
    var state = plant.observe(initial), phase = MissionPhase.acquisition
    var failures: [ConstraintViolation] = [], maximumTrackingError = 0.0
    var finalCaptured = false, stoppedSince: Double?, success = false
    var lastReference: PathStation?
    while state.time-initial.time < c.guidance.maximumPredictionDuration {
      if Task.isCancelled { break }
      if state.contact == .crashed {
        failures.append(.init(.contactQuality, "Closed-loop route failed runway contact or containment")); break
      }
      failures += touchdown.observe(state)
      if !failures.isEmpty { break }
      if state.aerodynamics.dynamicPressure > c.vehicle.maximumDynamicPressure {
        failures.append(.init(.excessiveDynamicPressure, "Closed-loop terminal q envelope")); break
      }
      if abs(state.aerodynamics.lift)/(state.mass*9.80665) > c.vehicle.maximumLoadFactor {
        failures.append(.init(.excessiveLoad, "Closed-loop terminal load envelope")); break
      }
      authority.observe(state)
      let trim = controller.diagnostics.trimEvidence == .identified ? controller.diagnostics.trimAlphaCeiling : nil
      let raw: GuidanceSolution
      if state.contact.isGrounded {
        phase = state.wheelContact?.noseGear == true ? .rollout : .touchdown
        let p = frame.coordinates(of: state.position)
        if !c.runway.contains(along: p.along, cross: p.cross) {
          failures.append(.init(.runwayExcursion, "Closed-loop rollout left runway")); break
        }
        if state.horizontalSpeed < 0.5 && state.wheelContact?.noseGear == true {
          if stoppedSince == nil { stoppedSince = state.time }
          if state.time-stoppedSince! >= c.guidance.stopConfirmationDuration { success = true; break }
        } else { stoppedSince = nil }
        raw = .init(phase: phase, target: .init(gearDown: true))
      } else if finalCaptured {
        raw = approach.update(state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
          pitchResponseSeconds: controller.diagnostics.pitchResponseSeconds, identifiedTrimCeiling: trim)
      } else {
        let (target, reference) = tracker.update(state, plan: plan, liftEffectiveness: authority.liftEffectiveness,
          identifiedTrimCeiling: trim)
        lastReference = reference
        maximumTrackingError = max(maximumTrackingError, tracker.trackingError)
        if reference.segment == .final && approach.canCapture(state) {
          finalCaptured = true
          raw = approach.update(state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness, dragEffectiveness: authority.dragEffectiveness,
            pitchResponseSeconds: controller.diagnostics.pitchResponseSeconds, identifiedTrimCeiling: trim)
        } else {
          raw = .init(phase: reference.segment == .spiral ? .headingAlignment : .acquisition, target: target)
        }
      }
      phase = raw.phase
      let target = limiter.update(raw.target, phase: phase, configuration: c, dt: dt,
        dynamicPressure: state.aerodynamics.dynamicPressure)
      if let reference = lastReference { trace?(state, reference, target) }
      do {
        let command = try controller.command(state: state, solution: .init(phase: phase, target: target), dt: dt)
        state = try plant.advance(state, command: command, duration: dt, maximumStep: 0.02)
      } catch {
        failures.append(.init(.actuatorAuthority, "Closed-loop terminal computation: \(error)")); break
      }
    }
    if !success && failures.isEmpty {
      failures.append(.init(.timeout, Task.isCancelled ? "Terminal qualification cancelled" : "Terminal qualification did not complete rollout"))
    }
    let event = plant.touchdown
    return .init(liftMultiplier: lift, dragMultiplier: drag, feasible: success && event?.acceptable == true,
      elapsed: state.time-initial.time, touchdownSpeed: event?.horizontalSpeed,
      touchdownSink: event?.verticalSpeed, touchdownPitch: event?.pitch,
      stoppedAlong: success ? frame.coordinates(of: state.position).along : nil,
      maximumTrackingError: maximumTrackingError, violations: failures, evidence: "offline-native-control-model-only")
  }
}
