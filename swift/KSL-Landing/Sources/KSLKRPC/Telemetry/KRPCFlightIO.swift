import Foundation
import KSLCore

public struct KRPCVesselIdentity: Codable, Sendable {
  public let name: String
  public let body: String
  public let partCount: Int
  public let serverVersion: String
}

/// Owns discovery references, coherent snapshots and command sequence. Read-only until arm().
public actor KRPCFlightIO: OrbitalFlightIO {
  private let session: KRPCSession
  private let configuration: MissionConfiguration
  private let airbrakeGroup: UInt32
  private var vessel: UInt64 = 0, control: UInt64 = 0, surfaceFrame: UInt64 = 0, flight: UInt64 = 0,
    parts: UInt64 = 0, autopilot: UInt64 = 0, bodyFrame: UInt64 = 0
  private var wheels = KRPCWheelInventory()
  private var armed = false, busy = false
  private var landingRecoveryPending = false
  private var orbitalRecoveryPending = false
  private var orbitalRecoveryParts = Set<UInt64>()
  private var generation: UInt64 = 0
  private var latest: VehicleState?
  private var sampledAt: ContinuousClock.Instant?
  private var lastSequence: UInt64 = 0
  private var initialPartCount = 0
  private var partIdentifiers = Set<UInt64>()
  private var topologyRevision: UInt64 = 0
  private var anchorPart: UInt64 = 0, anchorFrame: UInt64 = 0
  private var supportsInertiaTensor = false
  private var supportsActuatorBreakdown = false
  private var supportsRCSTorque = false
  private var expectedVesselName: String? = "STS"
  private var deorbitPlan: DeorbitPlan?
  private var burnEngines: [UInt64] = []
  private var burnEngineParts: Set<UInt64> = []
  private var burnFinished = false
  public private(set) var identity: KRPCVesselIdentity?
  public init(
    connection: KRPCConnectionConfiguration = .init(),
    configuration: MissionConfiguration = .init(), airbrakeActionGroup: UInt32 = 1
  ) {
    session = .init(configuration: connection)
    self.configuration = configuration
    airbrakeGroup = airbrakeActionGroup
  }
  private func objectCall(_ procedure: String, _ object: UInt64) -> KRPCCall {
    .init(procedure: procedure, arguments: [KRPCValue.uint(object)])
  }
  public func connect(expectedVesselName: String? = "STS") async throws -> KRPCVesselIdentity {
    try configuration.validate()
    guard airbrakeGroup <= 9 else {
      throw FlightError.invalidConfiguration("kRPC action groups are 0 through 9")
    }
    guard !busy else { throw FlightError.invalidCommand("Concurrent kRPC operation") }
    busy = true
    self.expectedVesselName = expectedVesselName
    let operationGeneration = generation
    defer { busy = false }
    do {
      try await session.connect()
      let services = try await session.call([.init(service: "KRPC", procedure: "GetServices")])[0]
      supportsInertiaTensor = services.range(of: Data("Vessel_get_InertiaTensor".utf8)) != nil
      supportsActuatorBreakdown = services.range(of: Data("Vessel_get_AvailableControlSurfaceTorque".utf8)) != nil
        && services.range(of: Data("Vessel_get_AvailableReactionWheelTorque".utf8)) != nil
      supportsRCSTorque = services.range(of: Data("Vessel_get_AvailableRCSTorque".utf8)) != nil
      let first = try await session.call([
        .init(procedure: "get_ActiveVessel"), .init(service: "KRPC", procedure: "GetStatus"),
      ])
      let discoveredVessel = try KRPCValue.decodeUInt(first[0])
      if vessel != 0, discoveredVessel != vessel {
        throw FlightError.rearmRequired("Active vessel changed during reconnect")
      }
      vessel = discoveredVessel
      let status = try Protobuf.decode(first[1])
      let version =
        Protobuf.bytes(status, number: 1).last.flatMap { String(data: $0, encoding: .utf8) }
        ?? "unknown"
      let objects = try await session.call([
        objectCall("Vessel_get_Control", vessel),
        objectCall("Vessel_get_SurfaceReferenceFrame", vessel),
        objectCall("Vessel_get_Orbit", vessel), objectCall("Vessel_get_Name", vessel),
        objectCall("Vessel_get_Parts", vessel), objectCall("Vessel_get_AutoPilot", vessel),
      ])
      control = try KRPCValue.decodeUInt(objects[0])
      surfaceFrame = try KRPCValue.decodeUInt(objects[1])
      let orbit = try KRPCValue.decodeUInt(objects[2])
      let name = try KRPCValue.decodeString(objects[3])
      if let expectedVesselName, name != expectedVesselName {
        throw FlightError.invalidTelemetry(
          "Active vessel \(name) differs from expected \(expectedVesselName)")
      }
      parts = try KRPCValue.decodeUInt(objects[4])
      autopilot = try KRPCValue.decodeUInt(objects[5])
      let refs = try await session.call([
        objectCall("Orbit_get_Body", orbit), objectCall("Parts_get_All", parts),
      ])
      let body = try KRPCValue.decodeUInt(refs[0])
      // A Flight created in the vessel surface frame reports zero velocity because that frame
      // moves with the vessel. This hybrid takes its position and velocity (with the planet's
      // rotation) from the body-fixed frame and its axes from the local up/north/east frame.
      bodyFrame = try KRPCValue.decodeUInt(
        try await session.call([objectCall("CelestialBody_get_ReferenceFrame", body)])[0])
      let hybridFrame = try KRPCValue.decodeUInt(
        try await session.call([
          .init(
            procedure: "ReferenceFrame_static_CreateHybrid",
            arguments: [
              KRPCValue.uint(bodyFrame), KRPCValue.uint(surfaceFrame), KRPCValue.uint(0),
              KRPCValue.uint(0),
            ])
        ])[0])
      flight = try KRPCValue.decodeUInt(
        try await session.call([
          .init(
            procedure: "Vessel_Flight",
            arguments: [KRPCValue.uint(vessel), KRPCValue.uint(hybridFrame)])
        ])[0])
      let allParts = try KRPCValue.decodeTuple(refs[1]).map(KRPCValue.decodeUInt)
      initialPartCount = allParts.count
      partIdentifiers = Set(allParts); topologyRevision &+= 1
      anchorPart = try KRPCValue.decodeUInt(try await session.call([objectCall("Parts_get_Root", parts)])[0])
      anchorFrame = try KRPCValue.decodeUInt(try await session.call([objectCall("Part_get_ReferenceFrame", anchorPart)])[0])
      wheels = try await KRPCWheelInventory.discover(session: session, parts: allParts)
      guard initialPartCount > 0 else {
        throw FlightError.invalidTelemetry("Active vessel has no parts")
      }
      let bodyInfo = try await session.call([
        objectCall("CelestialBody_get_Name", body),
        objectCall("CelestialBody_get_EquatorialRadius", body),
        objectCall("CelestialBody_get_GravitationalParameter", body),
      ])
      let bodyName = try KRPCValue.decodeString(bodyInfo[0])
      let radius = try KRPCValue.decodeDouble(bodyInfo[1])
      let mu = try KRPCValue.decodeDouble(bodyInfo[2])
      guard bodyName == "Kerbin", abs(radius - configuration.planet.radius) < 1,
        abs(mu / configuration.planet.gravitationalParameter - 1) < 1e-4
      else { throw FlightError.invalidTelemetry("Planet does not match configured Kerbin model") }
      let value = KRPCVesselIdentity(
        name: name, body: bodyName, partCount: initialPartCount, serverVersion: version)
      guard generation == operationGeneration else {
        throw FlightError.disconnected("Connection shut down during discovery")
      }
      identity = value
      return value
    } catch {
      await session.close()
      identity = nil
      throw error
    }
  }
  public func arm() async throws {
    guard identity != nil, !busy else {
      throw FlightError.invalidCommand("Connect and inspect before arming")
    }
    busy = true
    let operationGeneration = generation
    defer { busy = false }
    // This kRPC build has no AutoPilot.Disengage procedure; Engaged is a property.
    _ = try await session.call(
      [
        .init(
          procedure: "AutoPilot_set_Engaged",
          arguments: [KRPCValue.uint(autopilot), KRPCValue.bool(false)])
      ] + safetyCalls())
    guard generation == operationGeneration else {
      throw FlightError.disconnected("Connection shut down while arming")
    }
    armed = true
    deorbitPlan = nil; burnFinished = true
    lastSequence = 0
    latest = nil
    sampledAt = nil
  }
  private func safetyCalls(throttle: Double = 0, rcs: Bool = false) -> [KRPCCall] {
    [
      .init(
        procedure: "Control_set_Throttle", arguments: [KRPCValue.uint(control), KRPCValue.float(Float(throttle))]),
      .init(
        procedure: "Control_set_SAS", arguments: [KRPCValue.uint(control), KRPCValue.bool(false)]),
      .init(
        procedure: "Control_set_RCS", arguments: [KRPCValue.uint(control), KRPCValue.bool(rcs)]),
      .init(
        procedure: "Control_SetActionGroup",
        arguments: [
          KRPCValue.uint(control), KRPCValue.uint(UInt64(airbrakeGroup)), KRPCValue.bool(false),
        ]),
    ]
  }
  public func readState() async throws -> VehicleState {
    guard identity != nil, !busy else {
      throw FlightError.disconnected("Not connected or concurrent snapshot")
    }
    busy = true
    let operationGeneration = generation
    defer { busy = false }
    // Refresh topology BEFORE issuing cached wheel-module calls. A released
    // payload may remove one of those modules; do not dereference stale IDs.
    let inventory = try await session.call([objectCall("Parts_get_All", parts)])
    let currentParts = try KRPCValue.decodeTuple(inventory[0]).map(KRPCValue.decodeUInt)
    let currentIdentifiers = Set(currentParts)
    guard !currentIdentifiers.isEmpty else { throw FlightError.invalidTelemetry("Vessel has no parts") }
    if currentIdentifiers != partIdentifiers {
      if deorbitPlan != nil && !burnFinished {
        // The active burn certificate binds mass/engine geometry. Do not fly
        // an obsolete burn plan across staging or a payload release.
        _ = try await session.call(safetyCalls())
        throw FlightError.rearmRequired("Vessel topology changed during bound deorbit execution")
      }
      wheels = try await KRPCWheelInventory.discover(session: session, parts: currentParts)
      if !currentIdentifiers.contains(anchorPart) {
        anchorPart = try KRPCValue.decodeUInt(try await session.call([objectCall("Parts_get_Root", parts)])[0])
        anchorFrame = try KRPCValue.decodeUInt(try await session.call([objectCall("Part_get_ReferenceFrame", anchorPart)])[0])
      }
      partIdentifiers = currentIdentifiers; initialPartCount = currentParts.count
      topologyRevision &+= 1
      latest = nil; sampledAt = nil
      if let old = identity {
        identity = .init(name: old.name, body: old.body, partCount: currentParts.count, serverVersion: old.serverVersion)
      }
    }
    let properties = [
      "Latitude", "Longitude", "MeanAltitude", "Velocity", "SurfaceAltitude", "DynamicPressure",
      "TrueAirSpeed", "SpeedOfSound", "StaticPressure", "StaticAirTemperature", "Lift", "Drag",
    ]
    var calls = [KRPCCall(procedure: "get_UT")]
    calls += properties.map { objectCall("Flight_get_" + $0, flight) }
    calls += [
      .init(
        procedure: "Vessel_Rotation",
        arguments: [KRPCValue.uint(vessel), KRPCValue.uint(surfaceFrame)]),
      objectCall("Vessel_get_Mass", vessel), objectCall("Vessel_get_Situation", vessel),
      objectCall("Control_get_Gear", control),
      objectCall("Control_get_Throttle", control), objectCall("Vessel_get_Thrust", vessel),
      objectCall("Control_get_SAS", control), objectCall("Control_get_RCS", control),
      .init(
        procedure: "Control_GetActionGroup",
        arguments: [KRPCValue.uint(control), KRPCValue.uint(UInt64(airbrakeGroup))]),
      objectCall("Parts_get_All", parts), .init(procedure: "get_ActiveVessel"),
      objectCall("Control_get_Brakes", control),
      objectCall("Vessel_get_AvailableTorque", vessel),
      objectCall("Vessel_get_MomentOfInertia", vessel),
      .init(procedure: "Vessel_AngularVelocity",
            arguments: [KRPCValue.uint(vessel), KRPCValue.uint(bodyFrame)]),
      .init(procedure: "Vessel_Rotation",
            arguments: [KRPCValue.uint(vessel), KRPCValue.uint(bodyFrame)]),
      objectCall("Control_get_Pitch", control), objectCall("Control_get_Roll", control),
      objectCall("Control_get_Yaw", control),
      objectCall("Vessel_get_AvailableThrust", vessel), objectCall("Vessel_get_DryMass", vessel),
      objectCall("Vessel_get_SpecificImpulse", vessel),
    ]
    let wheelStart = calls.count
    calls += wheels.calls
    let wheelEnd = calls.count
    let centerOfMassIndex = calls.count
    calls.append(.init(procedure: "Vessel_Position",
      arguments: [KRPCValue.uint(vessel), KRPCValue.uint(anchorFrame)]))
    let tensorIndex: Int? = supportsInertiaTensor ? calls.count : nil
    if supportsInertiaTensor { calls.append(objectCall("Vessel_get_InertiaTensor", vessel)) }
    let actuatorIndex: Int? = supportsActuatorBreakdown ? calls.count : nil
    if supportsActuatorBreakdown {
      calls += [objectCall("Vessel_get_AvailableControlSurfaceTorque", vessel),
        objectCall("Vessel_get_AvailableReactionWheelTorque", vessel)]
      if supportsRCSTorque { calls.append(objectCall("Vessel_get_AvailableRCSTorque", vessel)) }
    }
    calls.append(.init(procedure: "get_UT"))
    let started = ContinuousClock.now
    let values = try await session.call(calls)
    guard generation == operationGeneration else {
      throw FlightError.disconnected("Connection shut down during snapshot")
    }
    let ended = ContinuousClock.now
    guard started.duration(to: ended) <= .seconds(configuration.guidance.maximumTelemetryAge) else {
      throw FlightError.staleTelemetry
    }
    let t0 = try KRPCValue.decodeDouble(values[0])
    let t1 = try KRPCValue.decodeDouble(values[values.count - 1])
    let span = t1 - t0
    guard span >= 0, span <= configuration.guidance.maximumSnapshotSpan else {
      throw FlightError.staleTelemetry
    }
    guard try KRPCValue.decodeUInt(values[23]) == vessel else {
      throw FlightError.invalidTelemetry("Active vessel changed")
    }
    let finalParts = Set(try KRPCValue.decodeTuple(values[22]).map(KRPCValue.decodeUInt))
    guard finalParts == partIdentifiers else { throw FlightError.staleTelemetry }
    let latitude = radians(try KRPCValue.decodeDouble(values[1]))
    let longitude = radians(try KRPCValue.decodeDouble(values[2]))
    let altitude = try KRPCValue.decodeDouble(values[3])
    let position = GeodeticPosition(latitude: latitude, longitude: longitude, altitude: altitude)
      .cartesian(radius: configuration.planet.radius)
    let velocity = KRPCFrameConverter.surfaceVector(
      try KRPCValue.decodeVector(values[4]), at: position)
    let rotation = try KRPCValue.decodeTuple(values[13], count: 4).map {
      try KRPCValue.decodeDouble($0)
    }
    let raw = Quaternion(w: rotation[3], x: rotation[0], y: rotation[1], z: rotation[2])
    guard abs(raw.norm - 1) < 0.01 else {
      throw FlightError.invalidTelemetry("Invalid kRPC rotation")
    }
    let attitude = KRPCFrameConverter.attitude(raw: raw, at: position)
    let bodyQuaternion = try KRPCValue.decodeTuple(values[28], count: 4).map(KRPCValue.decodeDouble)
    let rates = KRPCFrameConverter.bodyRate(
      rawAngularVelocity: try KRPCValue.decodeVector(values[27]),
      rawBodyRotation: .init(w: bodyQuaternion[3], x: bodyQuaternion[0],
                             y: bodyQuaternion[1], z: bodyQuaternion[2]))
    let torque = try KRPCValue.decodeTuple(values[25], count: 2).map(KRPCValue.decodeVector)
    let inertia = try KRPCValue.decodeVector(values[26])
    func frd(_ v: Vector3) -> Vector3 { .init(v.y, v.x, v.z) }
    var authority = ControlAuthorityTelemetry(positiveTorque: frd(torque[0]),
      negativeTorque: frd(torque[1]), inertia: frd(inertia))
    authority.actuatorSources = "total-vessel-orbital"
    // RCS stays disabled even during OMS execution. Total vessel torque can
    // include disabled RCS or unlit-engine gimbals; never feed that optimistic
    // envelope to the native pointing controller. Surfaces and active powered
    // reaction wheels are the demonstrated commandable set in every phase.
    do {
      if let actuatorIndex {
        let surfaces = try KRPCValue.decodeTuple(values[actuatorIndex], count: 2).map(KRPCValue.decodeVector)
        let reaction = try KRPCValue.decodeTuple(values[actuatorIndex+1], count: 2).map(KRPCValue.decodeVector)
        func magnitude(_ v: Vector3) -> Vector3 { .init(abs(v.x), abs(v.y), abs(v.z)) }
        var positive = magnitude(surfaces[0])+magnitude(reaction[0])
        var negative = magnitude(surfaces[1])+magnitude(reaction[1])
        authority.rcsExcludedPositiveTorque = frd(positive)
        authority.rcsExcludedNegativeTorque = -frd(negative)
        authority.actuatorSources = "aerodynamic-surfaces+active-powered-reaction-wheels"
        // RCS counts only while it is actually enabled (commanded by the FCS).
        if supportsRCSTorque, try KRPCValue.decodeBool(values[20]) {
          let rcs = try KRPCValue.decodeTuple(values[actuatorIndex+2], count: 2).map(KRPCValue.decodeVector)
          positive = positive + magnitude(rcs[0]); negative = negative + magnitude(rcs[1])
          authority.actuatorSources = (authority.actuatorSources ?? "") + "+rcs"
        }
        authority.positiveTorque = frd(positive)
        authority.negativeTorque = -frd(negative)
      } else {
        authority.positiveTorque = .zero; authority.negativeTorque = .zero
        authority.evidence = .unavailable
        authority.actuatorSources = "unavailable-with-RCS-and-engine-gimbals-excluded"
      }
    }
    guard authority.isValid else { throw FlightError.invalidTelemetry("Invalid torque/inertia report") }
    let wheelContact = try wheels.decode(values[wheelStart..<wheelEnd])
    let speed = try KRPCValue.decodeFloat(values[7])
    let q = try KRPCValue.decodeFloat(values[6])
    let sound = try KRPCValue.decodeFloat(values[8])
    let atmosphere = AtmosphericState(
      density: speed > 1 ? 2 * q / (speed * speed) : 0,
      pressure: try KRPCValue.decodeFloat(values[9]),
      temperature: try KRPCValue.decodeFloat(values[10]), speedOfSound: max(1, sound), evidence: .measured)
    let va = velocity.normalized
    let body = attitude.conjugate.rotate(va)
    let windRight = va.cross(position.normalized).normalized
    let windUp = windRight.cross(va)
    let up = attitude.rotate(.init(0, 0, -1))
    var aero = AerodynamicState()
    aero.airspeed = speed
    aero.mach = sound > 1 ? speed / sound : 0
    aero.dynamicPressure = q
    aero.angleOfAttack = atan2(body.z, body.x)
    aero.sideslip = asin(clamp(body.y, -1, 1))
    aero.bank = atan2(up.dot(windRight), up.dot(windUp))
    let lift = KRPCFrameConverter.surfaceVector(
      try KRPCValue.decodeVector(values[11]), at: position)
    let drag = KRPCFrameConverter.surfaceVector(
      try KRPCValue.decodeVector(values[12]), at: position)
    let liftDirection = attitude.rotate(.init(0, 1, 0)).cross(va).normalized
    aero.lift = lift.dot(liftDirection)
    aero.drag = max(0, -drag.dot(va))
    aero.force = lift + drag
    aero.measuredForces = true
    var systems = SystemState()
    systems.gearDown = try KRPCValue.decodeBool(values[16])
    systems.throttle = try KRPCValue.decodeFloat(values[17])
    systems.thrust = try KRPCValue.decodeFloat(values[18])
    systems.sas = try KRPCValue.decodeBool(values[19])
    systems.rcs = try KRPCValue.decodeBool(values[20])
    systems.airbrakes = try KRPCValue.decodeBool(values[21])
    systems.brakes = try KRPCValue.decodeBool(values[24])
    let situation = try KRPCValue.decodeSInt(values[15])
    let contact: ContactState
    // kRPC situation 6 is SPLASHED, not vessel destruction. Continue
    // control and report the off-runway contact instead of decision-aborting.
    if situation == 6 { contact = velocity.length < 0.5 ? .stopped : .rolling }
    else if wheelContact.mainGear == true {
      contact = wheelContact.noseGear == true ? (velocity.length < 0.5 ? .stopped : .rolling) : .mainGear
    } else if wheelContact.mainGear == nil && situation == 5 {
      contact = velocity.length < 0.5 ? .stopped : .rolling
    } else { contact = .airborne }
    let applied = ControlCommand(pitch: try KRPCValue.decodeFloat(values[29]),
      roll: try KRPCValue.decodeFloat(values[30]), yaw: try KRPCValue.decodeFloat(values[31]),
      gearDown: systems.gearDown, wheelBrake: systems.brakes ? 1 : 0)
    var state = VehicleState(
      time: t1, position: position, velocity: velocity, attitude: attitude, angularVelocity: rates,
      mass: try KRPCValue.decodeFloat(values[14]), atmosphere: atmosphere, aerodynamics: aero,
      contact: contact, systems: systems,
      radarAltitude: try KRPCValue.decodeDouble(values[5]), snapshotSpan: span,
      controlAuthority: authority, wheelContact: wheelContact, appliedControls: applied)
    state.propulsion = .init(availableThrust: try KRPCValue.decodeFloat(values[32]),
      dryMass: try KRPCValue.decodeFloat(values[33]), specificImpulse: try KRPCValue.decodeFloat(values[34]))
    var physical = VesselConfigurationTelemetry(revision: topologyRevision, partCount: partIdentifiers.count,
      anchorPart: anchorPart, centerOfMassFromAnchor: try KRPCValue.decodeVector(values[centerOfMassIndex]),
      centerOfMassEvidence: .measured, measuredAt: t1)
    if let tensorIndex {
      let raw = try KRPCValue.decodeTuple(values[tensorIndex], count: 9).map(KRPCValue.decodeDouble)
      let permutation = [1, 0, 2]
      physical.inertiaTensorFRD = permutation.flatMap { i in permutation.map { j in raw[3*i+j] } }
    }
    state.vesselConfiguration = physical
    try state.validate(configuration: configuration)
    latest = state
    sampledAt = ended
    return state
  }
  public func apply(_ envelope: CommandEnvelope) async throws {
    guard deorbitPlan == nil else { throw FlightError.invalidCommand("Complete the orbital-to-entry ownership transfer first") }
    guard armed, !busy, let latest, let sampledAt else {
      throw FlightError.invalidCommand("Not armed or no coherent snapshot")
    }
    guard envelope.sequence > lastSequence, abs(envelope.stateTime - latest.time) < 1e-6,
      sampledAt.duration(to: .now) <= .seconds(configuration.guidance.maximumTelemetryAge)
    else { throw FlightError.staleTelemetry }
    try envelope.command.validate(grounded: latest.contact.isGrounded)
    busy = true
    let operationGeneration = generation
    defer { busy = false }
    let c = envelope.command
    var calls = safetyCalls(rcs: c.rcs)
    for (name, value) in [
      ("Pitch", c.pitch), ("Roll", c.roll), ("Yaw", c.yaw), ("WheelSteering", c.wheelSteering),
    ] {
      calls.append(.init(procedure: "Control_set_" + name,
        arguments: [KRPCValue.uint(control), KRPCValue.float(Float(value))]))
    }
    calls.append(
      .init(
        procedure: "Control_set_Gear",
        arguments: [KRPCValue.uint(control), KRPCValue.bool(c.gearDown)]))
    calls.append(
      .init(
        procedure: "Control_set_Brakes",
        arguments: [KRPCValue.uint(control), KRPCValue.bool(c.wheelBrake > 0.5)]))
    if c.deployParachutes == true {
      guard latest.wheelContact?.mainGear == true, (5...100).contains(latest.horizontalSpeed) else {
        throw FlightError.invalidCommand("Rollout parachutes require measured main-wheel contact and 5...100 m/s")
      }
      calls.append(.init(procedure: "Control_set_Parachutes",
        arguments: [KRPCValue.uint(control), KRPCValue.bool(true)]))
    }
    do {
      _ = try await session.call(calls)
      guard generation == operationGeneration else {
        throw FlightError.disconnected("Connection shut down during command")
      }
      lastSequence = envelope.sequence
    } catch {
      // A failed batch leaves the previous control state in KSP; recover() re-asserts safety.
      throw error
    }
  }
  /// Writes a KSP quicksave with the given name (SpaceCenter.Save); used for phase checkpoints.
  public func save(name: String) async throws {
    guard !busy else { throw FlightError.invalidCommand("Concurrent kRPC operation") }
    busy = true
    defer { busy = false }
    _ = try await session.call([
      .init(procedure: "Save", arguments: [KRPCValue.string(name)])
    ])
  }
  public func assertSafety() async throws {
    guard armed, !busy else { return }
    busy = true
    defer { busy = false }
    _ = try await session.call(safetyCalls())
  }
  public func armDeorbit(plan: DeorbitPlan, reducedAssurance: Bool) async throws {
    try plan.authorize(identifier: plan.identifier, reducedAssurance: reducedAssurance)
    guard plan.configuration == configuration else { throw FlightError.invalidConfiguration("Deorbit plan/configuration mismatch") }
    let state = try await readState()
    guard state.time < plan.window.burnStart, state.position.length-configuration.planet.radius > configuration.planet.atmosphereTop,
      let propulsion = state.propulsion else { throw FlightError.invalidCommand("Deorbit plan is stale or vessel is already in the atmosphere") }
    try propulsion.validate(mass: state.mass)
    let selected = try await discoverBurnEngines()
    try await arm()
    burnEngines = selected.engines; burnEngineParts = selected.parts
    deorbitPlan = plan; burnFinished = false
  }
  private func discoverBurnEngines() async throws -> (engines: [UInt64], parts: Set<UInt64>) {
    guard identity != nil, !busy else { throw FlightError.invalidCommand("Concurrent OMS discovery") }
    busy = true
    let operationGeneration = generation
    defer { busy = false }
    // Read existing active rocket engines. Never stage or activate an unrelated engine.
    let engineRefs = try KRPCValue.decodeTuple(try await session.call([objectCall("Parts_get_Engines", parts)])[0])
      .map(KRPCValue.decodeUInt)
    var selectedParts = Set<UInt64>()
    var inventory: [(engine: UInt64, part: UInt64)] = []
    for engine in engineRefs {
      let values = try await session.call([objectCall("Engine_get_Active", engine),
        objectCall("Engine_get_AvailableThrust", engine), objectCall("Engine_get_HasFuel", engine),
        objectCall("Engine_get_Part", engine)])
      let part = try KRPCValue.decodeUInt(values[3])
      inventory.append((engine, part))
      if try KRPCValue.decodeBool(values[0]) && KRPCValue.decodeFloat(values[1]) > 0 && KRPCValue.decodeBool(values[2]) {
        selectedParts.insert(part)
      }
    }
    guard generation == operationGeneration else {
      throw FlightError.rearmRequired("Connection changed during OMS discovery")
    }
    guard !selectedParts.isEmpty else { throw FlightError.infeasible("No active, fuelled OMS engine has measured available thrust") }
    guard selectedParts.isSubset(of: partIdentifiers) else {
      throw FlightError.rearmRequired("OMS part not present in the selected vessel")
    }
    return (try OrbitalEngineBinding.resolve(parts: selectedParts, inventory: inventory), selectedParts)
  }
  /// Engine wrappers can receive new object IDs on every inventory read. Rebind
  /// by the original vessel part, rejecting ambiguous multiple engine modules.
  /// A shutdown may already have succeeded: never reactivate on reconnect.
  private func rebindBurnEngines() async throws {
    guard !busy, !burnEngineParts.isEmpty else { throw FlightError.rearmRequired("Missing original OMS identity") }
    busy = true; defer { busy = false }
    let refs = try KRPCValue.decodeTuple(try await session.call([objectCall("Parts_get_Engines", parts)])[0])
      .map(KRPCValue.decodeUInt)
    var inventory: [(engine: UInt64, part: UInt64)] = []
    for engine in refs {
      let part = try KRPCValue.decodeUInt(try await session.call([objectCall("Engine_get_Part", engine)])[0])
      inventory.append((engine, part))
    }
    burnEngines = try OrbitalEngineBinding.resolve(parts: burnEngineParts, inventory: inventory)
  }
  public func applyOrbital(sequence: UInt64, stateTime: Double, command: OrbitalControlCommand) async throws {
    guard armed, !busy, let plan = deorbitPlan, command.planID == plan.identifier,
      let latest, let sampledAt else { throw FlightError.invalidCommand("No bound orbital execution capability") }
    guard sequence > lastSequence, abs(stateTime-latest.time) < 1e-6,
      sampledAt.duration(to: .now) <= .seconds(configuration.guidance.maximumTelemetryAge) else { throw FlightError.staleTelemetry }
    try command.axes.validate(grounded: false)
    guard command.throttle.isFinite, (0...plan.configuration.deorbitSettings.maximumThrottle).contains(command.throttle) else {
      throw FlightError.invalidCommand("Throttle exceeds the reviewed plan")
    }
    if command.throttle > 0 {
      guard !burnFinished, stateTime >= plan.window.burnStart,
        stateTime <= plan.window.burnStart+max(30, 2*plan.window.duration),
        latest.position.length-configuration.planet.radius > configuration.planet.atmosphereTop else {
        throw FlightError.invalidCommand("Propulsion outside the bound deorbit window")
      }
    }
    busy = true; let epoch = generation; defer { busy = false }
    var calls = safetyCalls(throttle: command.throttle, rcs: configuration.vehicle.rcsAttitudeControl)
    for (name, value) in [("Pitch", command.axes.pitch), ("Roll", command.axes.roll), ("Yaw", command.axes.yaw)] {
      calls.append(.init(procedure: "Control_set_"+name,
        arguments: [KRPCValue.uint(control), KRPCValue.float(Float(value))]))
    }
    _ = try await session.call(calls)
    guard generation == epoch else { throw FlightError.rearmRequired("Orbital connection generation changed") }
    lastSequence = sequence
  }
  public func finishDeorbit() async throws {
    guard armed, !busy, deorbitPlan != nil else { throw FlightError.invalidCommand("No active deorbit ownership") }
    busy = true; defer { busy = false }
    let stop = burnEngines.map { KRPCCall(procedure: "Engine_set_Active",
      arguments: [KRPCValue.uint($0), KRPCValue.bool(false)]) }
    _ = try await session.call(safetyCalls(rcs: configuration.vehicle.rcsAttitudeControl)+stop)
    burnFinished = true
  }
  public func beginEntry() async throws {
    guard armed, burnFinished, !busy else { throw FlightError.invalidCommand("OMS shutdown must precede entry transfer") }
    deorbitPlan = nil; lastSequence = 0; latest = nil; sampledAt = nil
  }
  /// Rebind a dropped link to the same vessel. Neutralize throttle and axes
  /// before restoring the existing owner; never replay an interrupted command.
  public func recover() async throws {
    guard !busy else { return }
    latest = nil
    sampledAt = nil
    if await session.isConnected == false || landingRecoveryPending || orbitalRecoveryPending {
      let resumeLanding = landingRecoveryPending || (armed && deorbitPlan == nil)
      let resumeOrbital = orbitalRecoveryPending || (armed && deorbitPlan != nil)
      if resumeOrbital && !orbitalRecoveryPending { orbitalRecoveryParts = partIdentifiers }
      landingRecoveryPending = resumeLanding
      orbitalRecoveryPending = resumeOrbital
      armed = false
      generation &+= 1
      await session.close()
      identity = nil
      _ = try await connect(expectedVesselName: expectedVesselName)
      let neutral = ["Pitch", "Roll", "Yaw", "WheelSteering"].map {
        KRPCCall(procedure: "Control_set_" + $0,
          arguments: [KRPCValue.uint(control), KRPCValue.float(0)])
      }
      _ = try await session.call(safetyCalls(rcs: configuration.vehicle.rcsAttitudeControl) + neutral)
      if resumeOrbital && partIdentifiers != orbitalRecoveryParts {
        throw FlightError.rearmRequired("Orbital vessel topology changed during reconnect")
      }
      lastSequence = 0
      if resumeOrbital {
        if !burnFinished { try await rebindBurnEngines() }
        armed = true
        orbitalRecoveryPending = false; orbitalRecoveryParts.removeAll()
      } else if resumeLanding {
        armed = true
        landingRecoveryPending = false
      } else {
        throw FlightError.rearmRequired("Transport reconnected without an existing control owner")
      }
    }
  }
  public func shutdown() async {
    landingRecoveryPending = false
    orbitalRecoveryPending = false; orbitalRecoveryParts.removeAll()
    generation &+= 1
    if armed && !busy {
      busy = true
      let neutral = ["Pitch", "Roll", "Yaw", "WheelSteering"].map {
        KRPCCall(
          procedure: "Control_set_" + $0, arguments: [KRPCValue.uint(control), KRPCValue.float(0)])
      }
      let release = KRPCCall(
        procedure: "AutoPilot_set_Engaged",
        arguments: [KRPCValue.uint(autopilot), KRPCValue.bool(false)])
      let engineStop = burnEngines.map { KRPCCall(procedure: "Engine_set_Active",
        arguments: [KRPCValue.uint($0), KRPCValue.bool(false)]) }
      _ = try? await session.call([release] + safetyCalls() + neutral + engineStop)
      busy = false
    }
    armed = false
    latest = nil
    sampledAt = nil
    await session.close()
    identity = nil
    deorbitPlan = nil; burnEngines = []; burnEngineParts.removeAll(); burnFinished = true
  }
  /// Reconnection is read-only. A caller must reacquire state and create a fresh runtime before arming.
  public func reconnect(expectedVesselName: String? = "STS") async throws -> KRPCVesselIdentity {
    await shutdown()
    return try await connect(expectedVesselName: expectedVesselName)
  }
}
