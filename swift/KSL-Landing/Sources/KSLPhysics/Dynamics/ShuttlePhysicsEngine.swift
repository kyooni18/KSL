import Foundation
import KSLCore

/// Exact in-memory continuation of this native plant. This preserves actuator
/// integration and authority provenance; it does not add empirical validity.
public struct ShuttlePhysicsSnapshot: Sendable {
  let configuration: MissionConfiguration
  let atmosphere: AtmosphericModelEngine
  let aerodynamics: AerodynamicsEngine
  public func matches(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
                      aerodynamics: AerodynamicsEngine) -> Bool {
    self.configuration == configuration && self.atmosphere === atmosphere && self.aerodynamics === aerodynamics
  }
  let surfaces: SurfaceActuatorState
  let authorityAnchor: ControlAuthorityTelemetry?
  let authorityAnchorRCS: Bool?
  let touchdown: TouchdownEvent?
  let pitchBalancingInput: Double?
  let localAngularAccelerationBias: Vector3
}

/// One simulation owns one engine and actuator history. No callbacks or parent references.
public final class ShuttlePhysicsEngine {
  public let configuration: MissionConfiguration
  public let atmosphere: AtmosphericModelEngine
  public let aerodynamics: AerodynamicsEngine
  public let translation: TranslationalDynamicsEngine
  public let rotation: RotationalDynamicsEngine
  private let ground: GroundDynamicsEngine
  private var surfaces = SurfaceActuatorState()
  private var authorityAnchorRCS: Bool?
  private let pitchBalancingInput: Double?
  private let localAngularAccelerationBias: Vector3
  private var authorityAnchor: ControlAuthorityTelemetry?
  public private(set) var touchdown: TouchdownEvent?
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine, initialControls: ControlCommand = .init(),
    initialState: VehicleState? = nil, pitchBalancingInput: Double? = nil,
    snapshot: ShuttlePhysicsSnapshot? = nil, localAngularAccelerationBias: Vector3 = .zero
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere
    self.aerodynamics = aerodynamics
    self.pitchBalancingInput = snapshot.map { $0.pitchBalancingInput } ?? pitchBalancingInput
    self.localAngularAccelerationBias = snapshot?.localAngularAccelerationBias ?? localAngularAccelerationBias
    surfaces = snapshot?.surfaces ?? .init(command: initialControls)
    authorityAnchorRCS = snapshot?.authorityAnchorRCS ?? initialState?.systems.rcs
    touchdown = snapshot?.touchdown
    authorityAnchor = snapshot?.authorityAnchor ?? initialState?.controlAuthority.flatMap {
      $0.isValid && [.measured, .identified].contains($0.evidence) ? $0 : nil
    }
    translation = .init(
      planet: configuration.planet, atmosphere: atmosphere, aerodynamics: aerodynamics)
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet,
      pitchBalancingInput: self.pitchBalancingInput, localAngularAccelerationBias: self.localAngularAccelerationBias)
    ground = .init(configuration: configuration)
  }
  public var snapshot: ShuttlePhysicsSnapshot {
    .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
      surfaces: surfaces, authorityAnchor: authorityAnchor,
      authorityAnchorRCS: authorityAnchorRCS, touchdown: touchdown, pitchBalancingInput: pitchBalancingInput,
      localAngularAccelerationBias: localAngularAccelerationBias)
  }
  /// RCS-on total torque cannot be reused as RCS-off authority. The archived
  /// same-snapshot exclusion is preferred; absent that decomposition, report
  /// missing support instead of silently borrowing enabled-RCS torque.
  public func authoritySupports(rcs: Bool) -> Bool {
    guard let anchor = authorityAnchor, let anchorRCS = authorityAnchorRCS else { return true }
    return rcs == anchorRCS || (anchorRCS && !rcs && anchor.excludingRCS != nil)
  }
  public func observe(_ state: VehicleState) -> VehicleState {
    // Refresh only from an external measurement, never from this engine's
    // extrapolated .identified output.
    if let report = state.controlAuthority,
      report.isValid, report.evidence == .measured {
      authorityAnchor = report; authorityAnchorRCS = state.systems.rcs
    }
    var s = state
    s.atmosphere = atmosphere.sample(position: s.position, time: s.time)
    s.aerodynamics = aerodynamics.evaluate(
      position: s.position, velocity: s.velocity, attitude: s.attitude, atmosphere: s.atmosphere,
      gear: s.systems.gearDown, brakes: s.systems.brakes, airbrakes: s.systems.airbrakes,
      speedbrakeFraction: s.systems.speedbrakeFraction ?? 0)
    let p = configuration.vehicle.controlPlant
    let kPa = max(0, s.aerodynamics.dynamicPressure / 1000)
    let acceleration = Vector3(min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa),
      min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa),
      min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa))
    // A measured total includes surfaces, wheels and the enabled RCS. Taking
    // max(total, q-only prior) double-counted/inflated authority and then
    // mislabeled that prior as identified. Hold the measured local torque and
    // inertia instead; its extrapolation away from the seed remains unvalidated.
    if var anchor = authorityAnchor {
      if authorityAnchorRCS == true, !s.systems.rcs, let excluded = anchor.excludingRCS {
        anchor = excluded
      }
      anchor.evidence = .identified
      anchor.actuatorSources = "measured/identified seed total torque and inertia held locally; q evolution unvalidated"
      s.controlAuthority = anchor
    } else {
      let torque = acceleration.multiplied(componentwise: configuration.vehicle.inertia)
      s.controlAuthority = .init(positiveTorque: torque, negativeTorque: -torque,
        inertia: configuration.vehicle.inertia, evidence: .provisional)
      s.controlAuthority?.actuatorSources = "provisional q-only moment model; no measured anchor"
    }
    s.appliedControls = .init(pitch: surfaces.pitch, roll: surfaces.roll, yaw: surfaces.yaw,
                              gearDown: s.systems.gearDown, speedbrakeFraction: surfaces.speedbrake,
                              bodyFlapTrimFraction: surfaces.bodyFlapTrim)
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
    // This plant has aggregate actuator coordinates, not a per-part deflection
    // model. Never carry actual surface samples into a modeled future epoch.
    s.controlSurfaces = nil
    var left = duration
    while left > 1e-9 && s.contact != .crashed {
      let dt = min(maximumStep, left)
      left -= dt
      s.systems.gearDown = command.gearDown
      s.systems.rcs = command.rcsEnabled ?? false
      // Refresh the active authority before the first rotational substep,
      // including the RCS-off command at entry/TAEM ownership transfer.
      s = observe(s)
      s.systems.brakes = command.wheelBrake > 0.5
      if s.contact.isGrounded {
        ground.step(state: &s, command: command, dt: dt)
        s = observe(s)
        continue
      }
      let previous = s
      surfaces.advance(
        command: command, dt: dt / 2, rate: configuration.vehicle.controlPlant.surfaceRate)
      s.systems.speedbrakeFraction = surfaces.speedbrake
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
      s.systems.speedbrakeFraction = surfaces.speedbrake
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
