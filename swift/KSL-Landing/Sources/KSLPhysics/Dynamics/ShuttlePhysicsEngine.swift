import Foundation
import KSLCore

/// One simulation owns one engine and actuator history. No callbacks or parent references.
public final class ShuttlePhysicsEngine {
  public let configuration: MissionConfiguration
  public let atmosphere: AtmosphericModelEngine
  public let aerodynamics: AerodynamicsEngine
  public let translation: TranslationalDynamicsEngine
  public let rotation: RotationalDynamicsEngine
  private let ground: GroundDynamicsEngine
  private var surfaces = SurfaceActuatorState()
  public private(set) var touchdown: TouchdownEvent?
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine, initialControls: ControlCommand = .init()
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere
    self.aerodynamics = aerodynamics
    surfaces = .init(command: initialControls)
    translation = .init(
      planet: configuration.planet, atmosphere: atmosphere, aerodynamics: aerodynamics)
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet)
    ground = .init(configuration: configuration)
  }
  public func observe(_ state: VehicleState) -> VehicleState {
    var s = state
    s.atmosphere = atmosphere.sample(position: s.position, time: s.time)
    s.aerodynamics = aerodynamics.evaluate(
      position: s.position, velocity: s.velocity, attitude: s.attitude, atmosphere: s.atmosphere,
      gear: s.systems.gearDown, brakes: s.systems.brakes, airbrakes: s.systems.airbrakes)
    let p = configuration.vehicle.controlPlant
    let kPa = max(0, s.aerodynamics.dynamicPressure / 1000)
    let acceleration = Vector3(min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa),
      min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa),
      min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa))
    let torque = acceleration.multiplied(componentwise: configuration.vehicle.inertia)
    s.controlAuthority = .init(positiveTorque: torque, negativeTorque: -torque,
      inertia: configuration.vehicle.inertia, evidence: .provisional)
    s.appliedControls = .init(pitch: surfaces.pitch, roll: surfaces.roll, yaw: surfaces.yaw,
                              gearDown: s.systems.gearDown)
    let r = RunwayFrame(runway: configuration.runway, planet: configuration.planet).coordinates(
      of: s.position)
    let terrain =
      configuration.runway.contains(along: r.along, cross: r.cross)
      ? configuration.runway.elevation : 0
    s.radarAltitude = max(0, s.position.length - configuration.planet.radius - terrain)
    return s
  }
  public func advance(
    _ state: VehicleState, command: ControlCommand, duration: Double, maximumStep: Double = 0.02
  ) throws -> VehicleState {
    guard duration > 0, duration.isFinite, maximumStep > 0, maximumStep <= 0.1 else {
      throw FlightError.invalidConfiguration("Invalid integration step")
    }
    try command.validate(grounded: state.contact.isGrounded)
    var s = observe(state)
    var left = duration
    while left > 1e-9 && s.contact != .crashed {
      let dt = min(maximumStep, left)
      left -= dt
      s.systems.gearDown = command.gearDown
      s.systems.brakes = command.wheelBrake > 0.5
      if s.contact.isGrounded {
        ground.step(state: &s, command: command, dt: dt)
        s = observe(s)
        continue
      }
      let previous = s
      surfaces.advance(
        command: command, dt: dt / 2, rate: configuration.vehicle.controlPlant.surfaceRate)
      let acceleration = rotation.angularAcceleration(state: s, surfaces: surfaces)
      let midRate = s.angularVelocity + acceleration * (dt / 2)
      let midAttitude = s.attitude.advanced(bodyRate: midRate, dt: dt / 2)
      let next = translation.step(
        .init(position: s.position, velocity: s.velocity), time: s.time, mass: s.mass,
        attitude: midAttitude, dt: dt, systems: s.systems)
      var midpoint = s
      midpoint.position = (s.position + next.position) / 2
      midpoint.velocity = (s.velocity + next.velocity) / 2
      midpoint.time = s.time + dt / 2
      midpoint.attitude = midAttitude
      midpoint.angularVelocity = midRate
      midpoint = observe(midpoint)
      let midpointAcceleration = rotation.angularAcceleration(state: midpoint, surfaces: surfaces)
      s.position = next.position
      s.velocity = next.velocity
      s.attitude = s.attitude.advanced(bodyRate: midRate, dt: dt)
      s.angularVelocity = s.angularVelocity + midpointAcceleration * dt
      s.time += dt
      surfaces.advance(command: command, dt: dt / 2, rate: configuration.vehicle.controlPlant.surfaceRate)
      s = observe(s)
      if let event = ground.detectContact(previous: previous, current: &s), touchdown == nil {
        touchdown = event
      }
      guard s.position.isFinite, s.velocity.isFinite, s.angularVelocity.isFinite else {
        throw FlightError.invalidTelemetry("Numerical divergence")
      }
    }
    return s
  }
}
