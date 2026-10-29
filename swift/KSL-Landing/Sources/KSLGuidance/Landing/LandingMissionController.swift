import Foundation
import KSLCore
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
  public private(set) var phase: MissionPhase = .entry
  public private(set) var plan: TerminalPlan?
  public private(set) var lastPlanningResult: PlanningResult?
  private var lastPlanAttempt = -Double.infinity
  private var stoppedSince: Double?
  private var poorTrackingSince: Double?
  private var previousTarget: GuidanceTarget?
  private var previousTime: Double?
  private var wasAirborne = false
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    entry = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    planner = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    tracker = .init(configuration: configuration, aerodynamics: aerodynamics)
    approach = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    authority = .init(model: aerodynamics)
    maximumGlideRatio = aerodynamics.maximumLiftToDragRatio
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    touchdown = .init(configuration: configuration)
  }
  public func update(_ state: VehicleState) throws -> GuidanceSolution {
    try state.validate(configuration: configuration)
    authority.observe(state)
    if let t = previousTime, state.time <= t { throw FlightError.staleTelemetry }
    let dt = previousTime.map { state.time - $0 } ?? configuration.guidance.controlInterval
    if dt > 1 { throw FlightError.staleTelemetry }
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
      if !envelope.isEmpty {
        phase = .abort
        return .init(phase: .abort, target: .init(), violations: envelope)
      }
    }
    let contactFailures = touchdown.observe(state)
    if !contactFailures.isEmpty {
      phase = .abort
      return .init(phase: .abort, target: .init(gearDown: true), violations: contactFailures)
    }
    if state.contact == .crashed {
      phase = .abort
      return .init(
        phase: .abort, target: .init(),
        violations: [.init(.contactQuality, "Crash or runway excursion observed")])
    }
    if state.contact.isGrounded {
      let p = frame.coordinates(of: state.position)
      guard configuration.runway.contains(along: p.along, cross: p.cross) else {
        phase = .abort
        return .init(
          phase: .abort, target: .init(gearDown: true),
          violations: [.init(.runwayExcursion, "Ground contact outside configured runway")])
      }
      if !wasAirborne {
        throw FlightError.invalidTelemetry(
          "Landing mission must begin airborne; existing parked vessel is not a successful landing")
      }
      phase = .rollout
      if state.horizontalSpeed < 0.5 {
        if stoppedSince == nil { stoppedSince = state.time }
        if state.time - stoppedSince! >= configuration.guidance.stopConfirmationDuration {
          phase = .complete
        }
      } else {
        stoppedSince = nil
      }
      return .init(phase: phase, target: .init(gearDown: true), planID: plan?.identifier)
    }
    wasAirborne = true
    if phase == .abort || phase == .complete { return .init(phase: phase, target: .init()) }
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
      // Failure rejects this modeled envelope, not all possible real craft.
      let optimisticRange = ld * max(0, p.height) + (ld / 2 + 1) * state.velocity.squaredLength / g
      let distance = frame.distance(
        from: state.position,
        to: frame.position(along: configuration.guidance.touchdownAlong, cross: 0, height: 0))
      if distance > optimisticRange {
        phase = .abort
        return .init(
          phase: .abort, target: .init(),
          violations: [
            .init(
              .insufficientEnergy, "Runway exceeds even the optimistic still-air modeled range",
              value: distance, limit: optimisticRange)
          ])
      }
    }
    if [.approach, .preflare, .flare].contains(phase) || (plan == nil && approach.canCapture(state))
    {
      raw = approach.update(
        state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness)
    } else {
      if plan == nil && p.height < 35_000 && state.aerodynamics.airspeed < 800
        && state.time - lastPlanAttempt >= configuration.guidance.terminalReplanInterval
      {
        let result = planner.plan(
          from: state, allowDirect: configuration.guidance.allowDirectTerminal)
        lastPlanningResult = result
        lastPlanAttempt = state.time
        if let candidate = result.plan {
          plan = candidate
          tracker.reset()
        }
      }
      if let committed = plan {
        let (target, reference) = tracker.update(
          state, plan: committed, liftEffectiveness: authority.liftEffectiveness)
        if reference.segment == .final {
          raw = approach.update(
            state, previousPhase: phase, liftEffectiveness: authority.liftEffectiveness)
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
            plan = nil
            poorTrackingSince = nil
          }
        } else {
          poorTrackingSince = nil
        }
      } else {
        let target = entry.update(state)
        var failures: [ConstraintViolation] = []
        if p.height < 1200 && !approach.canCapture(state) {
          failures = [
            .init(.insufficientEnergy, "No complete terminal path or final capture remains")
          ]
        }
        raw = .init(
          phase: failures.isEmpty ? .entry : .abort, target: target, violations: failures,
          diagnostic:
            "Entry receding-horizon candidates=\(entry.candidateCount), reversals=\(entry.reversalCount)"
        )
      }
    }
    phase = raw.phase
    var target = raw.target
    if let previousTarget {
      target.bank =
        previousTarget.bank
        + clamp(
          wrapAngle(target.bank - previousTarget.bank), -configuration.guidance.bankSlewRate * dt,
          configuration.guidance.bankSlewRate * dt)
      target.angleOfAttack = moveToward(
        previousTarget.angleOfAttack, target.angleOfAttack,
        maximumDelta: (raw.phase == .preflare || raw.phase == .flare
          ? configuration.vehicle.controlPlant.maximumPitchRate * 0.8
          : configuration.guidance.alphaSlewRate) * dt)
    }
    previousTarget = target
    return .init(
      phase: raw.phase, target: target, violations: raw.violations, planID: raw.planID,
      pathRemaining: raw.pathRemaining, diagnostic: raw.diagnostic)
  }
}
