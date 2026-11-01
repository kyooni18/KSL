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
  func predict(source: VehicleState, window: DeorbitWindow, propulsion: PropulsionTelemetry)
    throws -> OrbitalExecutionResult {
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
    let initial = OrbitalState(fixed: source, planet: c.planet)
    // Use the measured OMS and actual orbital state, without generated
    // propulsion, mass, position, timing or pointing perturbations.
    let prop = propulsion
    try prop.validate(mass: initial.mass)
    let start = window.burnStart
    let simulationPlan = DeorbitPlan(source: source, configuration: c, propulsion: prop, window: window,
      cases: [], modelLimitations: ["simulation input; no actuation authorization"], evaluatedWindows: 0)
    let burn = DeorbitBurnController(plan: simulationPlan), entryPointing = OrbitalAttitudeController()
    var orbital = initial, attitude = source.attitude, rate = source.angularVelocity
    var state = orbital.planetFixed(planet: c.planet, attitude: attitude, angularVelocity: rate)
    state.controlAuthority = torque; state.propulsion = prop
    var surfaces = SurfaceActuatorState(command: source.appliedControls ?? .init())
    var previousThrust = 0.0, burnEnd: Double?
    var rcsEnabled = source.systems.rcs
    let deadline = start+max(30, 2*window.duration)+4500
    while orbital.time < deadline {
      try Task.checkCancellation()
      state = orbital.planetFixed(planet: c.planet, attitude: attitude, angularVelocity: rate)
      state.controlAuthority = rcsEnabled && source.systems.rcs ? torque : entryTorque
      state.propulsion = prop; state.systems.thrust = previousThrust; state.systems.rcs = rcsEnabled
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
      // Match the actuation command, not merely the orbital snapshot: the
      // pointing controller normally switches RCS off when wheels suffice.
      // When RCS was initially off, no RCS moment was measured in this
      // snapshot, so do not invent additional torque if it is requested.
      rcsEnabled = axes.rcs && c.vehicle.rcsAttitudeControl
      let appliedTorque = rcsEnabled && source.systems.rcs ? torque : entryTorque
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
        let moment = Vector3(moment(appliedTorque.positiveTorque.x, appliedTorque.negativeTorque.x, surfaces.roll),
          moment(appliedTorque.positiveTorque.y, appliedTorque.negativeTorque.y, surfaces.pitch),
          moment(appliedTorque.positiveTorque.z, appliedTorque.negativeTorque.z, surfaces.yaw))
        let frameRate = attitude.conjugate.rotate(c.planet.angularVelocity), inertialRate = rate+frameRate
        let acceleration = (moment-inertialRate.cross(torque.inertia.multiplied(componentwise: inertialRate)))
          .divided(componentwise: torque.inertia)+rate.cross(frameRate)
        let midRate = rate+acceleration*(h/2), midAttitude = attitude.advanced(bodyRate: midRate, dt: h/2)
        // A real OMS nozzle can be canted ~20 deg off the vessel's nose.
        // The flight and qualification must accelerate along this SAME
        // measured axis (the attitude controller compensates its nose target).
        let bodyThrust = prop.thrustDirectionBody ?? .init(1, 0, 0)
        let actualThrust = midAttitude.rotate(bodyThrust).rotated(
          about: .init(0, 0, 1), angle: c.planet.rotationRate*(orbital.time+h/2))
        let thrustDirection = actualThrust.normalized
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
