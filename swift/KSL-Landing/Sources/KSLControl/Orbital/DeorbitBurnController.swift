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
    previousTime = nil; previousAcceleration = 0; lossSeconds = 0
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
}
