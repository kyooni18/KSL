import Foundation
import KSLCore

public struct KRPCVesselIdentity: Codable, Sendable {
  public let name: String
  public let body: String
  public let partCount: Int
  public let serverVersion: String
}

/// Owns discovery references, coherent snapshots and command sequence. Read-only until arm().
public actor KRPCFlightIO: FlightIO {
  private let session: KRPCSession
  private let configuration: MissionConfiguration
  private let airbrakeGroup: UInt32
  private var vessel: UInt64 = 0, control: UInt64 = 0, surfaceFrame: UInt64 = 0, flight: UInt64 = 0,
    parts: UInt64 = 0, autopilot: UInt64 = 0
  private var armed = false, busy = false
  private var generation: UInt64 = 0
  private var latest: VehicleState?
  private var sampledAt: ContinuousClock.Instant?
  private var lastSequence: UInt64 = 0
  private var initialPartCount = 0
  private var expectedVesselName: String? = "STS"
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
      let first = try await session.call([
        .init(procedure: "get_ActiveVessel"), .init(service: "KRPC", procedure: "GetStatus"),
      ])
      vessel = try KRPCValue.decodeUInt(first[0])
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
      let bodyFrame = try KRPCValue.decodeUInt(
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
      initialPartCount = try KRPCValue.decodeTuple(refs[1]).count
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
    lastSequence = 0
    latest = nil
    sampledAt = nil
  }
  private func safetyCalls() -> [KRPCCall] {
    [
      .init(
        procedure: "Control_set_Throttle", arguments: [KRPCValue.uint(control), KRPCValue.float(0)]),
      .init(
        procedure: "Control_set_SAS", arguments: [KRPCValue.uint(control), KRPCValue.bool(false)]),
      .init(
        procedure: "Control_set_RCS", arguments: [KRPCValue.uint(control), KRPCValue.bool(false)]),
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
      objectCall("Control_get_Brakes", control), .init(procedure: "get_UT"),
    ]
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
    let t1 = try KRPCValue.decodeDouble(values[25])
    let span = t1 - t0
    guard span >= 0, span <= configuration.guidance.maximumSnapshotSpan else {
      throw FlightError.staleTelemetry
    }
    guard try KRPCValue.decodeUInt(values[23]) == vessel else {
      throw FlightError.invalidTelemetry("Active vessel changed")
    }
    let partCount = try KRPCValue.decodeTuple(values[22]).count
    guard partCount == initialPartCount else {
      throw FlightError.invalidTelemetry(
        "Vessel part count changed from \(initialPartCount) to \(partCount)")
    }
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
    let rates =
      latest.map {
        KRPCFrameConverter.measuredBodyRate(
          previous: $0.attitude, current: attitude, dt: t1 - $0.time)
      } ?? .zero
    let speed = try KRPCValue.decodeFloat(values[7])
    let q = try KRPCValue.decodeFloat(values[6])
    let sound = try KRPCValue.decodeFloat(values[8])
    let atmosphere = AtmosphericState(
      density: speed > 1 ? 2 * q / (speed * speed) : 0,
      pressure: try KRPCValue.decodeFloat(values[9]),
      temperature: try KRPCValue.decodeFloat(values[10]), speedOfSound: max(1, sound))
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
    let contact: ContactState =
      situation == 5
      ? (velocity.length < 0.5 ? .stopped : .rolling) : situation == 6 ? .crashed : .airborne
    let state = VehicleState(
      time: t1, position: position, velocity: velocity, attitude: attitude, angularVelocity: rates,
      mass: try KRPCValue.decodeFloat(values[14]), atmosphere: atmosphere, aerodynamics: aero,
      contact: contact, systems: systems,
      radarAltitude: try KRPCValue.decodeDouble(values[5]), snapshotSpan: span)
    try state.validate(configuration: configuration)
    latest = state
    sampledAt = ended
    return state
  }
  public func apply(_ envelope: CommandEnvelope) async throws {
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
    var calls = safetyCalls()
    for (name, value) in [
      ("Pitch", c.pitch), ("Roll", c.roll), ("Yaw", c.yaw), ("WheelSteering", c.wheelSteering),
    ] {
      calls.append(
        .init(
          procedure: "Control_set_" + name,
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
  public func assertSafety() async throws {
    guard armed, !busy else { return }
    busy = true
    defer { busy = false }
    _ = try await session.call(
      [
        .init(
          procedure: "AutoPilot_set_Engaged",
          arguments: [KRPCValue.uint(autopilot), KRPCValue.bool(false)])
      ] + safetyCalls())
  }
  /// Reconnects (fresh handshake and discovery) after a transport fault and re-asserts the
  /// restrictions; a still-open link only discards the cached snapshot.
  public func recover() async throws {
    guard !busy else { return }
    latest = nil
    sampledAt = nil
    if await session.isConnected == false {
      await session.close()
      identity = nil
      _ = try await connect(expectedVesselName: expectedVesselName)
      try await assertSafety()
    }
  }
  public func shutdown() async {
    generation &+= 1
    if armed && !busy {
      busy = true
      let neutral = ["Pitch", "Roll", "Yaw", "WheelSteering"].map {
        KRPCCall(
          procedure: "Control_set_" + $0, arguments: [KRPCValue.uint(control), KRPCValue.float(0)])
      }
      _ = try? await session.call(safetyCalls() + neutral)
      busy = false
    }
    armed = false
    latest = nil
    sampledAt = nil
    await session.close()
    identity = nil
  }
  /// Reconnection is read-only. A caller must reacquire state and create a fresh runtime before arming.
  public func reconnect(expectedVesselName: String? = "STS") async throws -> KRPCVesselIdentity {
    await shutdown()
    return try await connect(expectedVesselName: expectedVesselName)
  }
}
