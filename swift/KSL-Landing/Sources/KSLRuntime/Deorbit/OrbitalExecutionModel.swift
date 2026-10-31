import Foundation
import KSLCore
import KSLControl
import KSLPhysics

struct OrbitalExecutionResult {
  let entry: VehicleState
  let burnCompleted: Bool
  let actualBurnEnd: Double
}

/// Executes the native burn and pointing laws with torque/inertia, rate-limited
/// controls, fuel consumption and inertial propagation. No attitude is snapped
/// to a target at ignition, shutdown or entry interface.
final class OrbitalExecutionModel {
  let configuration: MissionConfiguration
  init(configuration: MissionConfiguration) { self.configuration = configuration }
  func predict(source: VehicleState, window: DeorbitWindow, propulsion: PropulsionTelemetry,
               dispersion: DeorbitDispersion) throws -> OrbitalExecutionResult {
    let c = configuration, orbit = OrbitalMechanics(planet: c.planet), dt = c.guidance.controlInterval
    guard let torque = source.controlAuthority, torque.isValid,
      torque.minimumAcceleration.x > 0, torque.minimumAcceleration.y > 0, torque.minimumAcceleration.z > 0 else {
      throw FlightError.infeasible("No measured three-axis vacuum attitude authority for deorbit execution")
    }
    // Vacuum pointing may use enabled RCS, but entry below is explicitly
    // RCS-off. Never carry its torque into the atmospheric plant. Old logs
    // without the same-snapshot breakdown cannot qualify this handoff.
    let entryTorque: ControlAuthorityTelemetry
    if source.systems.rcs {
      guard let withoutRCS = torque.excludingRCS else {
        throw FlightError.infeasible("RCS-on orbital state lacks measured RCS-off entry authority")
      }
      entryTorque = withoutRCS
    } else {
      entryTorque = torque
    }
    var initial = OrbitalState(fixed: source, planet: c.planet)
    let radial = initial.position.normalized
    initial.position = initial.position+radial*dispersion.position
    initial.velocity = initial.velocity+initial.velocity.projected(onPlaneWithNormal: radial).normalized*dispersion.velocity
    initial.mass *= dispersion.mass
    var prop = propulsion; prop.availableThrust *= dispersion.thrust
    try prop.validate(mass: initial.mass)
    let start = window.burnStart+dispersion.timing
    var cutoffPeriapsis = window.periapsisAltitude
    if dispersion.deltaV != 0 {
      let coast = try orbit.propagate(initial, to: start, step: 12)
      let cutoff = try FiniteBurnModel(planet: c.planet).propagate(from: coast,
        deltaV: max(0, window.deltaV+dispersion.deltaV), propulsion: prop,
        settings: c.deorbitSettings, normalAngle: window.normalAngle)
      guard cutoff.complete else { throw FlightError.infeasible("Dispersed cutoff target cannot be completed") }
      cutoffPeriapsis = try orbit.elements(cutoff.state).periapsisAltitude
    }
    // Cutoff-delta-v uncertainty perturbs the equivalent periapsis closure
    // target; it does not inject an unphysical velocity impulse after shutdown.
    let stressedWindow = DeorbitWindow(burnStart: start, deltaV: max(0, window.deltaV+dispersion.deltaV),
      duration: window.duration, normalAngle: window.normalAngle, periapsisAltitude: cutoffPeriapsis,
      afterBurn: window.afterBurn, entryInterface: window.entryInterface, entryRange: window.entryRange,
      entrySpeed: window.entrySpeed, entryFlightPath: window.entryFlightPath,
      entryCourseError: window.entryCourseError, crossrange: window.crossrange, miss: window.miss)
    let simulationPlan = DeorbitPlan(source: source, configuration: c, propulsion: prop, window: stressedWindow,
      cases: [], modelLimitations: ["simulation input; no actuation authorization"], evaluatedWindows: 0)
    let burn = DeorbitBurnController(plan: simulationPlan), entryPointing = OrbitalAttitudeController()
    var orbital = initial, attitude = source.attitude, rate = source.angularVelocity
    var state = orbital.planetFixed(planet: c.planet, attitude: attitude, angularVelocity: rate)
    state.controlAuthority = torque; state.propulsion = prop
    var surfaces = SurfaceActuatorState(command: source.appliedControls ?? .init())
    var previousThrust = 0.0, burnEnd: Double?
    let deadline = start+max(30, 2*window.duration)+4500
    while orbital.time < deadline {
      try Task.checkCancellation()
      state = orbital.planetFixed(planet: c.planet, attitude: attitude, angularVelocity: rate)
      state.controlAuthority = torque; state.propulsion = prop; state.systems.thrust = previousThrust
      state.appliedControls = .init(pitch: surfaces.pitch, roll: surfaces.roll, yaw: surfaces.yaw)
      let height = state.position.length-c.planet.radius
      if height <= c.planet.atmosphereTop-c.deorbitSettings.entryAltitudeMargin && state.verticalSpeed < 0 {
        guard burn.complete else { throw FlightError.infeasible("Entered atmosphere before native OMS shutdown") }
        state.systems = .init()
        state.controlAuthority = entryTorque
        return .init(entry: state, burnCompleted: true, actualBurnEnd: burnEnd ?? state.time)
      }
      let axes: ControlCommand, throttle: Double
      if burn.complete {
        if burnEnd == nil { burnEnd = state.time }
        let target = Quaternion.windAttitude(position: state.position, airVelocity: state.velocity,
          alpha: c.entrySettings.nominalAlpha, bank: 0)
        axes = try entryPointing.command(state: state, direction: target.rotate(.init(1, 0, 0)), dt: dt)
        throttle = 0
      } else {
        let output = try burn.update(state, periapsisAltitude: orbit.elements(orbital).periapsisAltitude)
        axes = output.command.axes; throttle = output.command.throttle
      }
      var remaining = dt
      while remaining > 1e-9 {
        var h = min(0.02, remaining)
        let thrust = prop.availableThrust*throttle
        if thrust > 0 { h = min(h, max(0, orbital.mass-prop.dryMass)*prop.specificImpulse*9.80665/thrust) }
        guard h > 1e-9 else { throw FlightError.infeasible("OMS propellant exhausted during controlled burn") }
        surfaces.advance(command: axes, dt: h/2, rate: c.vehicle.controlPlant.surfaceRate)
        func moment(_ positive: Double, _ negative: Double, _ input: Double) -> Double {
          input >= 0 ? abs(positive)*input : abs(negative)*input
        }
        let moment = Vector3(moment(torque.positiveTorque.x, torque.negativeTorque.x, surfaces.roll),
          moment(torque.positiveTorque.y, torque.negativeTorque.y, surfaces.pitch),
          moment(torque.positiveTorque.z, torque.negativeTorque.z, surfaces.yaw))
        let frameRate = attitude.conjugate.rotate(c.planet.angularVelocity), inertialRate = rate+frameRate
        let acceleration = (moment-inertialRate.cross(torque.inertia.multiplied(componentwise: inertialRate)))
          .divided(componentwise: torque.inertia)+rate.cross(frameRate)
        let midRate = rate+acceleration*(h/2), midAttitude = attitude.advanced(bodyRate: midRate, dt: h/2)
        let nose = midAttitude.rotate(.init(1, 0, 0)).rotated(about: .init(0, 0, 1), angle: c.planet.rotationRate*(orbital.time+h/2))
        let orbitNormal = orbital.position.cross(orbital.velocity).normalized
        let thrustDirection = (nose+orbital.position.normalized*tan(dispersion.radial)
          + orbitNormal*tan(dispersion.normal)).normalized
        let consumed = thrust*h/(prop.specificImpulse*9.80665)
        let thrustAcceleration = thrustDirection*(thrust/(orbital.mass-consumed/2))
        orbital = orbit.step(orbital, dt: h) { _, _ in thrustAcceleration }
        orbital.mass -= consumed
        attitude = attitude.advanced(bodyRate: midRate, dt: h); rate = rate+acceleration*h
        surfaces.advance(command: axes, dt: h/2, rate: c.vehicle.controlPlant.surfaceRate)
        remaining -= h; previousThrust = thrust
      }
    }
    throw FlightError.infeasible("Controlled orbital execution did not reach entry interface before timeout")
  }
}
