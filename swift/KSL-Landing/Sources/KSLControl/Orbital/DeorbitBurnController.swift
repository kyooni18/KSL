import Foundation
import KSLCore

/// C burn_fraction, measured thrust integration, periapsis taper and loss-of-
/// thrust watchdog. The caller supplies the osculating measured periapsis.
public final class DeorbitBurnController {
  private let plan: DeorbitPlan
  private var pointing = OrbitalAttitudeController()
  private var previousTime: Double?
  private var started: Double?
  private var previousAcceleration = 0.0
  private var previousNormalAcceleration = 0.0
  public private(set) var deliveredNormalDeltaV = 0.0
  private var normalCorrectionComplete = false
  private var lossSeconds = 0.0
  public private(set) var deliveredDeltaV = 0.0
  public private(set) var unobservedDeltaVUpperBound = 0.0
  public private(set) var complete = false
  public private(set) var diagnostics = OrbitalExecutionDiagnostics()
  public var controlDiagnostics: ControlDiagnostics { pointing.diagnostics }
  public init(plan: DeorbitPlan) { self.plan = plan }
  /// The thrust history during an outage is not observable. If the burn was
  /// started, conservatively bound unobserved delivered delta-v by the
  /// configured maximum throttle and measured current mass/thrust. Keep that
  /// upper bound separate from measured impulse. The
  /// osculating periapsis remains the primary closure measurement.
  public func resumeAfterTelemetryGap(at state: VehicleState) throws {
    if let previousTime {
      let gap = state.time - previousTime
      guard gap >= 0, gap.isFinite else { throw FlightError.rearmRequired("Burn telemetry clock moved backwards") }
      if started != nil && !complete && gap > 0 {
        guard let propulsion = state.propulsion, state.mass > 0 else {
          throw FlightError.invalidTelemetry("Missing measured propulsion after orbital outage")
        }
        let thrust = max(propulsion.availableThrust, plan.propulsion.availableThrust)
        unobservedDeltaVUpperBound += thrust * plan.configuration.deorbitSettings.maximumThrottle / state.mass * gap
      }
    }
    previousTime = nil; previousAcceleration = 0; previousNormalAcceleration = 0; lossSeconds = 0
    pointing = OrbitalAttitudeController()
  }
  public static func periapsisFraction(remainingAltitude: Double) -> Double {
    // C regression: a 2 km residual must not start the old 8 km taper.
    clamp((remainingAltitude-250)/1500, 0, 1)
  }
  public func update(_ state: VehicleState, periapsisAltitude: Double) throws
    -> (phase: MissionPhase, command: OrbitalControlCommand) {
    let c = plan.configuration, settings = c.deorbitSettings
    let dt = previousTime.map { state.time-$0 } ?? c.guidance.controlInterval
    guard dt > 0, dt <= 1, periapsisAltitude.isFinite else {
      throw FlightError.rearmRequired("Burn telemetry clock or orbital state discontinuity")
    }
    previousTime = state.time
    let orbital = OrbitalState(fixed: state, planet: c.planet)
    if let normalTarget = plan.window.normalDeltaV {
      return try updateComponents(state, orbital: orbital, periapsisAltitude: periapsisAltitude,
        normalTarget: normalTarget, dt: dt)
    }
    let inertialDirection = (-orbital.velocity.normalized
      + orbital.position.cross(orbital.velocity).normalized*tan(plan.window.normalAngle)).normalized
    let direction = inertialDirection.rotated(about: .init(0, 0, 1), angle: -c.planet.rotationRate*state.time)
    let engineAxis = plan.propulsion.thrustDirectionBody ?? .init(1, 0, 0)
    let noseTarget = OrbitalAttitudeController.compensatedNoseDirection(
      desiredThrust: direction, position: state.position, bodyThrustDirection: engineAxis)
    let axes = try pointing.command(state: state, direction: noseTarget, dt: dt)
    // Throttle safety and delivered impulse are projected along the actual
    // canted engine line, NOT the body nose that the attitude loop controls.
    let physicalThrust = state.attitude.rotate(engineAxis).normalized
    let alignment = acos(clamp(physicalThrust.dot(direction), -1, 1))
    let projection = max(0, physicalThrust.dot(
      (-orbital.velocity.normalized).rotated(about: .init(0, 0, 1), angle: -c.planet.rotationRate*state.time)))
    let measuredAcceleration = max(0, state.systems.thrust/state.mass)*projection
    if started != nil { deliveredDeltaV += (previousAcceleration+measuredAcceleration)*dt/2 }
    previousAcceleration = measuredAcceleration
    let periapsisRemaining = periapsisAltitude-plan.window.periapsisAltitude
    let dvLimit = plan.window.deltaV+8 // C finite-periapsis closure reserve.
    if started != nil && (periapsisRemaining <= 300 || deliveredDeltaV+unobservedDeltaVUpperBound >= dvLimit) { complete = true }
    var phase = MissionPhase.orbitalCoast, throttle = 0.0
    if !complete && state.time >= plan.window.burnStart {
      if started == nil {
        // Missing the planned ignition instant is not itself a propulsion
        // failure. Continue aligning at zero throttle and acquire the thrust
        // line when within tolerance; the actual delivered impulse/periapsis
        // determines cutoff. The plan's entry altitude limit is enforced by
        // the live I/O and orbital-to-entry transition.
        if alignment <= radians(10) { started = state.time }
      }
      phase = .deorbitBurn
      if let started, alignment <= radians(10) {
        guard let propulsion = state.propulsion else { throw FlightError.invalidTelemetry("Missing measured propulsion") }
        let maximumAcceleration = propulsion.availableThrust*settings.maximumThrottle/state.mass
        let remaining = max(0, dvLimit-deliveredDeltaV-unobservedDeltaVUpperBound)
        let fraction = BurnThrottleLaw.fraction(elapsed: state.time-started, remaining: remaining,
          maximumAcceleration: maximumAcceleration, ramp: settings.throttleRampDuration)
        let authority = clamp((10-degrees(alignment))/6, 0, 1)
        throttle = remaining < BurnThrottleLaw.terminalDeltaVTolerance ? 0
          : settings.maximumThrottle*min(fraction, Self.periapsisFraction(remainingAltitude: periapsisRemaining))*authority
        let expected = propulsion.availableThrust*throttle/state.mass
        if remaining > 2 && expected > 0.02 && measuredAcceleration < max(0.01, expected*0.10) {
          lossSeconds += dt
        } else { lossSeconds = 0 }
        if lossSeconds > 6 { throw FlightError.infeasible("Deorbit burn lost measured thrust for more than six seconds") }
      }
    } else if !complete && state.time >= plan.window.burnStart-12 { phase = .burnSetup }
    if complete { phase = .entryCoast; throttle = 0 }
    // Burn duration is a prediction, not an abort criterion. Attitude loss
    // pauses propulsion; continue correcting at zero throttle and use measured
    // periapsis/delivered impulse for shutdown. DeorbitRuntime separately
    // ends OMS if the craft actually descends into the atmosphere.
    diagnostics.planID = plan.identifier; diagnostics.deliveredDeltaV = deliveredDeltaV
    diagnostics.unobservedDeltaVUpperBound = unobservedDeltaVUpperBound
    diagnostics.targetDeltaV = plan.window.deltaV; diagnostics.periapsisAltitude = periapsisAltitude
    diagnostics.alignmentError = alignment; diagnostics.burnElapsed = started.map { state.time-$0 } ?? 0
    diagnostics.thrustLossSeconds = lossSeconds
    return (phase, .init(planID: plan.identifier, axes: axes, throttle: throttle))
  }

  /// Independent component closure. Normal thrust remains observable even
  /// when its retrograde projection is zero; periapsis alone cannot stop it.
  private func updateComponents(_ state: VehicleState, orbital: OrbitalState,
    periapsisAltitude: Double, normalTarget: Double, dt: Double) throws
    -> (phase: MissionPhase, command: OrbitalControlCommand) {
    let c = plan.configuration, settings = c.deorbitSettings
    guard normalTarget.isFinite else { throw FlightError.invalidCommand("Non-finite normal impulse target") }
    let rotation = -c.planet.rotationRate*state.time
    let retrograde = (-orbital.velocity.normalized).rotated(about: .init(0,0,1), angle: rotation)
    let normal = orbital.position.cross(orbital.velocity).normalized
      .rotated(about: .init(0,0,1), angle: rotation)
    let engineAxis = plan.propulsion.thrustDirectionBody ?? .init(1,0,0)
    let physicalThrust = state.attitude.rotate(engineAxis).normalized
    let acceleration = max(0, state.systems.thrust/state.mass)
    let retroAcceleration = acceleration*max(0,physicalThrust.dot(retrograde))
    let normalAcceleration = acceleration*physicalThrust.dot(normal)
    if started != nil {
      deliveredDeltaV += (previousAcceleration+retroAcceleration)*dt/2
      deliveredNormalDeltaV += (previousNormalAcceleration+normalAcceleration)*dt/2
    }
    previousAcceleration = retroAcceleration; previousNormalAcceleration = normalAcceleration
    let sign = normalTarget < 0 ? -1.0 : 1.0
    // One conservative total impulse bound also bounds either projection.
    // Never silently treat missing thrust history as measured normal impulse.
    let normalRemaining = max(0, abs(normalTarget)-sign*deliveredNormalDeltaV-unobservedDeltaVUpperBound)
    let periapsisRemaining = periapsisAltitude-plan.window.periapsisAltitude
    let limit = plan.window.deltaV+8 // Existing finite-periapsis closure reserve.
    let retrogradeDone = periapsisRemaining <= 300 || deliveredDeltaV+unobservedDeltaVUpperBound >= limit
    let retrogradeRemaining = retrogradeDone ? 0 : max(0,
      (deliveredDeltaV < plan.window.deltaV ? plan.window.deltaV : limit)
        - deliveredDeltaV-unobservedDeltaVUpperBound)
    let tolerance = BurnThrottleLaw.terminalDeltaVTolerance
    if started != nil && normalRemaining <= tolerance { normalCorrectionComplete = true }
    if started != nil && retrogradeDone && normalCorrectionComplete && normalRemaining <= tolerance { complete = true }
    let taper = Self.periapsisFraction(remainingAltitude: periapsisRemaining)
    // Retain normal closure while aligning and firing the retrograde leg:
    // a small off-axis thrust component must not undo the completed plane
    // correction and then disappear behind a periapsis-only cutoff.
    let demand = normalCorrectionComplete ? retrograde*(retrogradeRemaining*taper)+normal*(sign*normalRemaining)
      : normal*(sign*normalRemaining)
    // A completed demand has no pointing direction; retain the measured
    // engine line until the normal entry-pointing owner takes over.
    let direction = demand.length > 0 ? demand.normalized : physicalThrust
    let nose = OrbitalAttitudeController.compensatedNoseDirection(desiredThrust: direction,
      position: state.position, bodyThrustDirection: engineAxis)
    let axes = try pointing.command(state: state, direction: nose, dt: dt)
    let alignment = acos(clamp(physicalThrust.dot(direction),-1,1))
    var phase = MissionPhase.orbitalCoast, throttle = 0.0
    if !complete && state.time >= plan.window.burnStart {
      if started == nil && alignment <= radians(10) { started = state.time }
      phase = .deorbitBurn
      if let started, alignment <= radians(10) {
        guard let propulsion = state.propulsion else { throw FlightError.invalidTelemetry("Missing measured propulsion") }
        let maximum = propulsion.availableThrust*settings.maximumThrottle/state.mass
        let remaining = normalCorrectionComplete ? hypot(retrogradeRemaining,normalRemaining) : normalRemaining
        let fraction = BurnThrottleLaw.fraction(elapsed: state.time-started, remaining: remaining,
          maximumAcceleration: maximum, ramp: settings.throttleRampDuration)
        let authority = clamp((10-degrees(alignment))/6,0,1)
        let closure = remaining > 0 ? demand.length/remaining : 0
        throttle = remaining <= tolerance ? 0 : settings.maximumThrottle*min(fraction,closure)*authority
        let expected = propulsion.availableThrust*throttle/state.mass
        let observed = acceleration*max(0,physicalThrust.dot(direction))
        if remaining > 2 && expected > 0.02 && observed < max(0.01,expected*0.10) { lossSeconds += dt }
        else { lossSeconds = 0 }
        if lossSeconds > 6 { throw FlightError.infeasible("Deorbit burn lost measured thrust for more than six seconds") }
      }
    } else if !complete && state.time >= plan.window.burnStart-12 { phase = .burnSetup }
    if complete { phase = .entryCoast; throttle = 0 }
    diagnostics.planID = plan.identifier; diagnostics.deliveredDeltaV = deliveredDeltaV
    diagnostics.deliveredNormalDeltaV = deliveredNormalDeltaV; diagnostics.targetNormalDeltaV = normalTarget
    diagnostics.unobservedDeltaVUpperBound = unobservedDeltaVUpperBound
    diagnostics.targetDeltaV = plan.window.deltaV; diagnostics.periapsisAltitude = periapsisAltitude
    diagnostics.alignmentError = alignment; diagnostics.burnElapsed = started.map { state.time-$0 } ?? 0
    diagnostics.thrustLossSeconds = lossSeconds
    return (phase,.init(planID:plan.identifier,axes:axes,throttle:throttle))
  }
}
