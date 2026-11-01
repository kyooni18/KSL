import Foundation
import KSLCore

/// Axis allocation for the installed OV-100 parts; nil leaves an unrelated
/// surface's configuration unchanged. Deployment remains an independent trim.
enum STSSurfaceAllocation {
  static func axes(partName: String) -> [Bool]? {
    switch partName.lowercased().replacingOccurrences(of: "_", with: ".") {
    case "benjee10.shuttle.elevon2", "benjee10.shuttle.bodyflap": return [true, false, false]
    case "benjee10.shuttle.elevon1": return [false, true, false]
    case "benjee10.shuttle.rudder": return [false, false, true]
    default: return nil
    }
  }
}

public struct KRPCVesselIdentity: Codable, Sendable {
  public let name: String
  public let body: String
  public let partCount: Int
  public let serverVersion: String
}

/// Owns discovery references, coherent snapshots and command sequence. Read-only until arm().
public actor KRPCFlightIO: OrbitalWarpFlightIO {
  private let session: KRPCSession
  // Dedicated SpaceCenter-only connection. A kRPC timeout during rails warp
  // must not poison the armed flight-control connection or its object IDs.
  private let warpSession: KRPCSession
  private let configuration: MissionConfiguration
  private let airbrakeGroup: UInt32
  private var vessel: UInt64 = 0, control: UInt64 = 0, surfaceFrame: UInt64 = 0, flight: UInt64 = 0,
    parts: UInt64 = 0, autopilot: UInt64 = 0, bodyFrame: UInt64 = 0
  private var wheels = KRPCWheelInventory()
  private var surfaceActuators = STSSurfaceActuators()
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
  private var supportsControlSurfaceAxes = false
  private var lastTelemetryIssueKinds: [String] = []
  private var expectedVesselName: String? = "STS"
  private var deorbitPlan: DeorbitPlan?
  private var burnEngines: [UInt64] = []
  private var burnEngineParts: Set<UInt64> = []
  private var measuredOMSThrustAxis: Vector3?
  private var measuredOMSAvailableThrust = 0.0
  private var measuredOMSSpecificImpulse = 0.0
  private var burnFinished = false
  private var ownsRailsWarp = false
  public private(set) var identity: KRPCVesselIdentity?
  public init(
    connection: KRPCConnectionConfiguration = .init(),
    configuration: MissionConfiguration = .init(), airbrakeActionGroup: UInt32 = 1
  ) {
    session = .init(configuration: connection)
    var warpConnection = connection
    warpConnection.clientName = "KSL Orbital Rails Warp"
    warpConnection.timeout = 15
    warpSession = .init(configuration: warpConnection)
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
      supportsControlSurfaceAxes = services.range(of: Data("Parts_get_ControlSurfaces".utf8)) != nil
        && services.range(of: Data("ControlSurface_set_PitchEnabled".utf8)) != nil
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
      // Obtain the actual OMS thrust line before the deorbit planner reads
      // its source telemetry. Engines are not assumed coaxial with the nose.
      measuredOMSThrustAxis = try? await readActiveOMSThrustAxis()
      initialPartCount = allParts.count
      partIdentifiers = Set(allParts); topologyRevision &+= 1
      anchorPart = try KRPCValue.decodeUInt(try await session.call([objectCall("Parts_get_Root", parts)])[0])
      anchorFrame = try KRPCValue.decodeUInt(try await session.call([objectCall("Part_get_ReferenceFrame", anchorPart)])[0])
      wheels = try await KRPCWheelInventory.discover(session: session, parts: allParts)
      surfaceActuators = try await STSSurfaceActuators.discover(session: session, parts: allParts)
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
    guard surfaceActuators.rudder.count == 2 else {
      throw FlightError.invalidTelemetry("STS rudder has no verified two-module yaw/speedbrake allocation")
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
      ] + safetyCalls() + surfaceActuators.armCalls())
    // Preflight: verify the requested STS surface roles before taking ownership.
    if try await enableControlSurfaceAxes() > 0 {
      throw FlightError.invalidTelemetry("Control-surface axis allocation failed read-back")
    }
    // The first rudder module may have been rewritten by ControlSurface.set_*;
    // verify both module-level axes AFTER the part-level allocation.
    try await surfaceActuators.verifySplitGeometry(session: session)
    guard generation == operationGeneration else {
      throw FlightError.disconnected("Connection shut down while arming")
    }
    armed = true
    deorbitPlan = nil; burnFinished = true
    lastSequence = 0
    latest = nil
    sampledAt = nil
  }
  /// Configure inboard elevons and body flap for pitch, outboard elevons for
  /// roll, and rudder for yaw. Preserve deployment, trim and authority limits.
  @discardableResult
  private func enableControlSurfaceAxes() async throws -> Int {
    guard supportsControlSurfaceAxes else {
      throw FlightError.invalidTelemetry("Server lacks control-surface axis procedures; requested STS allocation cannot be verified")
    }
    let surfaces = try KRPCValue.decodeTuple(
      try await session.call([objectCall("Parts_get_ControlSurfaces", parts)])[0])
      .map(KRPCValue.decodeUInt)
    guard !surfaces.isEmpty else { throw FlightError.invalidTelemetry("No control surfaces available") }
    let surfaceParts = try await session.call(surfaces.map { objectCall("ControlSurface_get_Part", $0) })
      .map(KRPCValue.decodeUInt)
    let names = try await session.call(surfaceParts.map { objectCall("Part_get_Name", $0) })
      .map(KRPCValue.decodeString)
    let targets = names.map { STSSurfaceAllocation.axes(partName: $0) }
    guard targets.contains([true, false, false]), targets.contains([false, true, false]),
          targets.contains([false, false, true]),
          names.contains(where: { $0.lowercased().replacingOccurrences(of: "_", with: ".") == "benjee10.shuttle.bodyflap" }),
          names.contains(where: { $0.lowercased().replacingOccurrences(of: "_", with: ".") == "benjee10.shuttle.elevon2" }) else {
      throw FlightError.invalidTelemetry("Cannot identify STS inner/outer elevons, body flap and rudder")
    }
    let axes = ["PitchEnabled", "RollEnabled", "YawEnabled"]
    func read() async throws -> [[Bool]] {
      guard !surfaces.isEmpty else { return [] }
      let values = try await session.call(surfaces.flatMap { surface in
        axes.map { objectCall("ControlSurface_get_" + $0, surface) } })
      return try stride(from: 0, to: values.count, by: axes.count).map {
        try values[$0..<$0+axes.count].map(KRPCValue.decodeBool) }
    }
    let before = try await read()
    var changes: [KRPCCall] = []
    for i in surfaces.indices {
      guard let desired = targets[i] else { continue }
      for j in axes.indices where before[i][j] != desired[j] {
        changes.append(.init(procedure: "ControlSurface_set_" + axes[j],
          arguments: [KRPCValue.uint(surfaces[i]), KRPCValue.bool(desired[j])]))
      }
    }
    if !changes.isEmpty { _ = try await session.call(changes) }
    let after = try await read()
    let mismatches = surfaces.indices.reduce(0) { count, i in
      guard let target = targets[i] else { return count }
      return count + zip(target, after[i]).filter { $0.0 != $0.1 }.count
    }
    try? FileHandle.standardError.write(contentsOf: Data(
      "ksl-control-surfaces: STS separated allocation, \(changes.count) settings changed, \(mismatches) read-back mismatches\n".utf8))
    return mismatches
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
    let surfaceStart = calls.count
    calls += surfaceActuators.telemetryCalls
    let surfaceEnd = calls.count
    let centerOfMassIndex = calls.count
    calls.append(.init(procedure: "Vessel_Position",
      arguments: [KRPCValue.uint(vessel), KRPCValue.uint(anchorFrame)]))
    let anchorRotationIndex = calls.count
    calls.append(.init(procedure: "Part_Rotation",
      arguments: [KRPCValue.uint(anchorPart), KRPCValue.uint(bodyFrame)]))
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
    // Total vessel torque can include disabled RCS or unlit-engine gimbals.
    // Count active surfaces/wheels and only RCS reported enabled in this same
    // snapshot; the configured permission alone is not measured authority.
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
    // Torque and inertia are optional actuator observations, not position or
    // attitude. Preserve a coherent kinematic sample when this report is bad;
    // nil selects the controller's explicitly provisional/identified fallback.
    // Never pass NaN through the estimator, call it measured zero, or reuse a
    // stale inertia after a payload/topology change.
    var telemetryIssues: [String] = []
    if !authority.isValid {
      telemetryIssues.append("Invalid torque/inertia report: sources=\(authority.actuatorSources ?? "unknown"), positive=\(authority.positiveTorque), negative=\(authority.negativeTorque), inertia=\(authority.inertia)")
    }
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
    var flapDeployment: Double?
    if surfaceEnd > surfaceStart {
      let fractions = try surfaceActuators.decodeFractions(values[surfaceStart..<surfaceEnd])
      systems.speedbrakeFraction = fractions.brake
      flapDeployment = fractions.flap
    }
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
      gearDown: systems.gearDown, wheelBrake: systems.brakes ? 1 : 0,
      speedbrakeFraction: systems.speedbrakeFraction ?? 0, bodyFlapTrimFraction: flapDeployment ?? 0)
    var state = VehicleState(
      time: t1, position: position, velocity: velocity, attitude: attitude, angularVelocity: rates,
      mass: try KRPCValue.decodeFloat(values[14]), atmosphere: atmosphere, aerodynamics: aero,
      contact: contact, systems: systems,
      radarAltitude: try KRPCValue.decodeDouble(values[5]), snapshotSpan: span,
      controlAuthority: authority.isValid ? authority : nil, wheelContact: wheelContact, appliedControls: applied)
    // Vessel_get_AvailableThrust and Vessel_get_SpecificImpulse include all
    // active stages (including SSMEs). Never feed them to OMS planning.
    if let measuredOMSThrustAxis, measuredOMSAvailableThrust > 0,
       measuredOMSSpecificImpulse > 0 {
      state.propulsion = .init(availableThrust: measuredOMSAvailableThrust,
        dryMass: try KRPCValue.decodeFloat(values[33]),
        specificImpulse: measuredOMSSpecificImpulse,
        thrustDirectionBody: measuredOMSThrustAxis)
    } else { state.propulsion = nil }
    let centerOfMass = try KRPCValue.decodeVector(values[centerOfMassIndex])
    var physical = VesselConfigurationTelemetry(revision: topologyRevision, partCount: partIdentifiers.count,
      anchorPart: anchorPart, centerOfMassFromAnchor: centerOfMass.isFinite ? centerOfMass : nil,
      centerOfMassEvidence: centerOfMass.isFinite ? .measured : .unavailable, measuredAt: t1)
    if centerOfMass.isFinite {
      let q = try KRPCValue.decodeTuple(values[anchorRotationIndex], count: 4).map(KRPCValue.decodeDouble)
      let rotation = Quaternion(w: q[3], x: q[0], y: q[1], z: q[2]).normalized
      let rawBody = rotation.rotate(centerOfMass)
      physical.centerOfMassBodyFRD = Vector3(rawBody.y, rawBody.x, rawBody.z)
    }
    if !centerOfMass.isFinite { telemetryIssues.append("Invalid center-of-mass report: \(centerOfMass)") }
    if let tensorIndex {
      let raw = try KRPCValue.decodeTuple(values[tensorIndex], count: 9).map(KRPCValue.decodeDouble)
      if raw.allSatisfy(\.isFinite), raw[0] > 0, raw[4] > 0, raw[8] > 0 {
        let permutation = [1, 0, 2]
        physical.inertiaTensorFRD = permutation.flatMap { i in permutation.map { j in raw[3*i+j] } }
      } else {
        telemetryIssues.append("Invalid inertia tensor: \(raw)")
      }
    }
    state.vesselConfiguration = physical
    state.telemetryIssues = telemetryIssues.isEmpty ? nil : telemetryIssues
    try state.validate(configuration: configuration)
    // Rate-limit stderr by sensor failure category; the full changing raw
    // values remain in each structured record without repeating the warning.
    let issueKinds = telemetryIssues.map { String($0.prefix { $0 != ":" }) }
    if issueKinds != lastTelemetryIssueKinds {
      let message = telemetryIssues.isEmpty ? "optional telemetry recovered"
        : "degraded optional telemetry: " + telemetryIssues.joined(separator: "; ")
      try? FileHandle.standardError.write(contentsOf: Data(("ksl-telemetry: " + message + "\n").utf8))
      lastTelemetryIssueKinds = issueKinds
    }
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
    // The flight controller explicitly requests attitude RCS during MM304 entry
    // and releases it on MM305/terminal ownership. Respect that per-tick decision;
    // the old unconditional post-OMS prohibition contradicted the FCS and killed
    // a live flight at the atmospheric interface.
    guard !envelope.command.rcs || configuration.vehicle.rcsAttitudeControl else {
      throw FlightError.invalidCommand("Attitude RCS disabled by vehicle configuration")
    }
    guard (envelope.command.speedbrakeFraction ?? 0) == 0
      || configuration.vehicle.speedbrakeFullDeploymentDragArea != nil else {
      throw FlightError.invalidCommand("Speedbrake requires identified full-deployment drag area")
    }
    guard (envelope.command.bodyFlapTrimFraction ?? 0) == 0
      || configuration.vehicle.bodyFlapTrimInputAtFullDeflection != nil else {
      throw FlightError.invalidCommand("Body-flap trim requires identified pitch-input equivalence")
    }
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
    // Do not advance the actuator cache until all other command validation
    // has succeeded; a rejected parachute request must not poison it.
    calls += try surfaceActuators.commandCalls(brake: c.speedbrakeFraction ?? 0,
      flap: c.bodyFlapTrimFraction ?? 0)
    do {
      _ = try await session.call(calls)
      guard generation == operationGeneration else {
        throw FlightError.disconnected("Connection shut down during command")
      }
      lastSequence = envelope.sequence
    } catch {
      // A batch can fail after a subset of kRPC writes. Reassert the full
      // deployment state next time rather than trusting a speculative cache.
      surfaceActuators.invalidateCachedCommands()
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
  /// During intentional on-rails warp do not sample the physics vessel, run
  /// the attitude controller, or issue a long blocking kRPC WarpTo request.
  /// SpaceCenter UT polling uses an independent transport and its own clock
  /// domain: warp-induced game-time jumps cannot trigger snapshot timeouts.
  public func coastWarp(until stopUT: Double) async throws -> Double {
    guard armed, !busy, let plan = deorbitPlan, !burnFinished,
      stopUT.isFinite, stopUT >= plan.createdAt + 60,
      stopUT < plan.window.burnStart - 60 else {
      throw FlightError.invalidCommand("Rails warp requires armed orbital coast and pre-burn margin")
    }
    busy = true
    defer { busy = false }
    func telemetry() async throws -> (ut: Double, factor: UInt64) {
      try await warpSession.connect()
      let replies = try await warpSession.call([
        .init(procedure: "get_UT"), .init(procedure: "get_RailsWarpFactor")])
      let ut = try KRPCValue.decodeDouble(replies[0])
      let factor = try KRPCValue.decodeRailsWarpFactor(replies[1])
      guard ut.isFinite else { throw FlightError.invalidTelemetry("Invalid time warp UT") }
      return (ut, factor)
    }
    let begin = try await telemetry()
    guard begin.ut < stopUT-30 else { return begin.ut }
    guard begin.factor == 0, begin.ut < plan.window.burnStart else {
      throw FlightError.rearmRequired("Another time warp is active or OMS window has started")
    }
    let neutral = ["Pitch", "Roll", "Yaw", "WheelSteering"].map {
      KRPCCall(procedure: "Control_set_" + $0,
        arguments: [KRPCValue.uint(control), KRPCValue.float(0)])
    }
    _ = try await session.call(safetyCalls() + neutral)
    var current = begin
    var lastChangeUT = begin.ut
    var lastAdvance = ContinuousClock.now
    var lastReport = begin.ut
    var transientFailures = 0
    func requestFactor(_ factor: UInt64) async throws {
      try await warpSession.connect()
      let encodedFactor = try KRPCValue.encodeRailsWarpFactor(factor)
      _ = try await warpSession.call([.init(procedure: "set_RailsWarpFactor",
        arguments: [encodedFactor])])
    }
    func report(_ message: String) {
      try? FileHandle.standardError.write(contentsOf: Data(("deorbit rails warp: "+message+"\n").utf8))
    }
    report("start UT=\(Int(begin.ut)) stopUT=\(Int(stopUT)) 1x return precedes OMS by 90s")
    ownsRailsWarp = true // mark before RPC; server may apply it before replying
    do {
      // No artificial 100x cap. Use the strongest rate that still leaves a
      // several-second command/telemetry latency margin before the 90s hold.
      // KSP can independently clamp the *accepted* rate by altitude.
      // At low Kerbin orbit, attempting an unsupported higher stock slot can
      // return factor ZERO rather than clamping to 10x. Probe downwards until
      // KSP confirms a nonzero accepted rate, instead of treating rejection
      // as operator-requested cancellation and coast-waiting for five hours.
      func acceptedFactor(atMost target: UInt64) async throws -> UInt64 {
        var candidate = target
        while candidate > 0 {
          try await requestFactor(candidate)
          // KSP applies warp changes on subsequent Unity frames. The RPC
          // setter ack does NOT mean the getter has transitioned yet; a
          // same-frame read can still return 0 and falsely reject every slot.
          // Await actual game-time/warp state, not a blocking WarpTo RPC.
          var observed = try await telemetry()
          for _ in 0..<8 {
            guard observed.ut < plan.window.burnStart-60 else {
              throw FlightError.rearmRequired("Rails warp probe crossed OMS safety margin")
            }
            if observed.factor > 0 { break }
            try Task.checkCancellation()
            try await Task.sleep(for: .milliseconds(175))
            observed = try await telemetry()
          }
          if observed.factor > 0 {
            report("requestedIndex=\(candidate) acceptedIndex=\(observed.factor)")
            return min(observed.factor, candidate)
          }
          report("rail index=\(candidate) remained unavailable after clock settling; probing lower slot")
          guard let lower = OrbitalRailsWarpSchedule.nextLowerAfterRejection(candidate) else { break }
          candidate = lower
        }
        return 0
      }
      var requested = OrbitalRailsWarpSchedule.factor(remainingToStop: stopUT-current.ut)
      var usableCeiling = try await acceptedFactor(atMost: requested)
      requested = usableCeiling
      if requested == 0 { report("KSP forbids all orbital rails warp; returning to 1x") }
      while requested > 0 && current.ut < stopUT-2 {
        try Task.checkCancellation()
        let remaining = stopUT-current.ut
        let nextFactor = min(usableCeiling, OrbitalRailsWarpSchedule.factor(remainingToStop: remaining))
        if nextFactor != requested {
          if nextFactor == 0 {
            report("1x terminal alignment reached: remaining=\(Int(remaining))s")
            break
          }
          let accepted = try await acceptedFactor(atMost: nextFactor)
          usableCeiling = min(usableCeiling, accepted)
          requested = accepted
          report("stepped down: remaining=\(Int(remaining))s accepted index=\(requested)")
          if requested == 0 { break }
        }
        do {
          current = try await telemetry()
          transientFailures = 0
        } catch is CancellationError {
          throw CancellationError()
        } catch {
          transientFailures += 1
          if transientFailures >= 6 {
            throw FlightError.rearmRequired("Orbital rails warp lost SpaceCenter clock after six retries: \(error)")
          }
          report("transient kRPC interruption, reconnecting clock (attempt \(transientFailures)/6)")
          try await Task.sleep(for: .milliseconds(350))
          continue
        }
        guard current.ut >= lastChangeUT-0.01,
          current.ut < plan.window.burnStart-60 else {
          throw FlightError.rearmRequired("Rails warp crossed the OMS safety margin or clock moved backwards")
        }
        if current.ut > lastChangeUT+0.01 {
          lastChangeUT = current.ut
          lastAdvance = .now
        }
        if current.factor == 0 {
          report("operator/KSP stopped time warp at UT=\(Int(current.ut))")
          break
        }
        if lastAdvance.duration(to: .now) > .seconds(20) {
          report("KSP clock not advancing; leaving warp and falling back to 1x")
          break
        }
        if current.ut-lastReport >= 1200 || (current.factor < requested && current.ut-lastReport >= 20) {
          report("UT=\(Int(current.ut)) remaining=\(Int(remaining))s requestedIndex=\(requested) acceptedIndex=\(current.factor)")
          lastReport = current.ut
        }
        // Poll on wall time, accounting for the *requested* rate in case a
        // previously clamped altitude band later permits a faster warp.
        let milliseconds = OrbitalRailsWarpSchedule.pollIntervalMillis(
          remainingToStop: remaining, requestedFactor: requested)
        try await Task.sleep(for: .milliseconds(milliseconds))
      }
      try await requestFactor(0)
      ownsRailsWarp = false
      try await Task.sleep(for: .milliseconds(250))
      let achieved = try await telemetry()
      report("complete at UT=\(Int(achieved.ut)), resuming 1x orbital control")
      return achieved.ut
    } catch {
      if ownsRailsWarp {
        // A failed set-factor response can leave warp active. Reconnect the
        // independent SpaceCenter session and explicitly demand 1x.
        for _ in 0..<3 {
          do { try await requestFactor(0); break }
          catch { try? await Task.sleep(for: .milliseconds(250)) }
        }
        ownsRailsWarp = false
      }
      throw error
    }
  }

  public func armDeorbit(plan: DeorbitPlan, reducedAssurance: Bool) async throws {
    try plan.authorize(identifier: plan.identifier, reducedAssurance: reducedAssurance)
    guard plan.configuration == configuration else { throw FlightError.invalidConfiguration("Deorbit plan/configuration mismatch") }
    let state = try await readState()
    guard state.time < plan.window.burnStart, state.position.length-configuration.planet.radius > configuration.planet.atmosphereTop,
      let propulsion = state.propulsion else { throw FlightError.invalidCommand("Deorbit plan is stale or vessel is already in the atmosphere") }
    try propulsion.validate(mass: state.mass)
    // Refuse to execute a plan that assumed nose-aligned thrust. The exact
    // same measured axis must survive the pre-burn state refresh and engine
    // discovery, or modeled orbit-to-entry timing is not trustworthy.
    guard let plannedAxis = plan.propulsion.thrustDirectionBody,
      let actualAxis = propulsion.thrustDirectionBody,
      plannedAxis.dot(actualAxis) > 0.999 else {
      throw FlightError.invalidTelemetry(
        "OMS engine thrust axis is unmeasured or changed since deorbit planning")
    }
    let selected = try await discoverBurnEngines()
    // The validated plan must bind exactly the available Terrier pair, never
    // an active SSME or a different OMS configuration.
    guard abs(selected.availableThrust / propulsion.availableThrust - 1) < 0.001 else {
      throw FlightError.rearmRequired("Measured OMS-only thrust changed since planning")
    }
    try await arm()
    burnEngines = selected.engines; burnEngineParts = selected.parts
    // SSME may be active in older KSP saves. Explicitly deactivate every
    // non-OMS engine before the first positive throttle command.
    let excluded = selected.otherActiveEngines.map { KRPCCall(
      procedure: "Engine_set_Active", arguments: [KRPCValue.uint($0), KRPCValue.bool(false)]) }
    if !excluded.isEmpty { _ = try await session.call(excluded) }
    deorbitPlan = plan; burnFinished = false
  }
  /// Return the real, Terrier-only net thrust axis and measured propulsion.
  /// The SSMEs on this STS remain excluded even if the KSP save has them active.
  private func readActiveOMSThrustAxis() async throws -> Vector3? {
    measuredOMSAvailableThrust = 0
    measuredOMSSpecificImpulse = 0
    let frame = try KRPCValue.decodeUInt(
      try await session.call([objectCall("Vessel_get_ReferenceFrame", vessel)])[0])
    let engineIds = try KRPCValue.decodeTuple(
      try await session.call([objectCall("Parts_get_Engines", parts)])[0])
      .map(KRPCValue.decodeUInt)
    var weighted = Vector3.zero
    var activeOMS: [(thrust: Double, isp: Double)] = []
    for engine in engineIds {
      let values = try await session.call([
        objectCall("Engine_get_Active", engine),
        objectCall("Engine_get_AvailableThrust", engine),
        objectCall("Engine_get_HasFuel", engine),
        objectCall("Engine_get_Part", engine),
        objectCall("Engine_get_SpecificImpulse", engine),
      ])
      let part = try KRPCValue.decodeUInt(values[3])
      let name = try KRPCValue.decodeString(try await session.call([
        objectCall("Part_get_Name", part)])[0])
      guard OrbitalEngineBinding.isOMSPartName(name) else { continue }
      let active = try KRPCValue.decodeBool(values[0])
      let fuel = try KRPCValue.decodeBool(values[2])
      let thrust = Double(try KRPCValue.decodeFloat(values[1]))
      let isp = Double(try KRPCValue.decodeFloat(values[4]))
      guard active && fuel && thrust > 0 && isp > 0 else { continue }
      let raw = try KRPCValue.decodeVector(
        try await session.call([.init(procedure: "Part_Direction",
          arguments: [KRPCValue.uint(part), KRPCValue.uint(frame)])])[0])
      let bodyFRD = Vector3(raw.y, raw.x, raw.z)
      guard bodyFRD.isFinite && bodyFRD.length > 0.9 else {
        throw FlightError.invalidTelemetry("Invalid measured OMS part axis")
      }
      weighted = weighted + bodyFRD.normalized * thrust
      activeOMS.append((thrust, isp))
    }
    // Asymmetric thrust from one missing Terrier is not safe to assume.
    guard activeOMS.count == OrbitalEngineBinding.requiredTerrierCount,
      let effectiveIsp = OrbitalEngineBinding.specificImpulse(for: activeOMS) else { return nil }
    let axis = weighted.normalized
    guard axis.x > 0.5, axis.isFinite else {
      throw FlightError.invalidTelemetry("Net OMS thrust is not predominantly forward")
    }
    measuredOMSAvailableThrust = activeOMS.reduce(0) { $0 + $1.thrust }
    measuredOMSSpecificImpulse = effectiveIsp
    return axis
  }

  private func discoverBurnEngines() async throws
    -> (engines: [UInt64], parts: Set<UInt64>, otherActiveEngines: [UInt64], availableThrust: Double) {
    guard identity != nil, !busy else { throw FlightError.invalidCommand("Concurrent OMS discovery") }
    busy = true
    let operationGeneration = generation
    defer { busy = false }
    let engineRefs = try KRPCValue.decodeTuple(try await session.call([
      objectCall("Parts_get_Engines", parts)])[0]).map(KRPCValue.decodeUInt)
    var selectedParts = Set<UInt64>()
    var inventory: [(engine: UInt64, part: UInt64)] = []
    var otherActive: [UInt64] = []
    var selectedThrust = 0.0
    for engine in engineRefs {
      let values = try await session.call([
        objectCall("Engine_get_Active", engine),
        objectCall("Engine_get_AvailableThrust", engine),
        objectCall("Engine_get_HasFuel", engine),
        objectCall("Engine_get_Part", engine),
      ])
      let part = try KRPCValue.decodeUInt(values[3])
      let name = try KRPCValue.decodeString(try await session.call([
        objectCall("Part_get_Name", part)])[0])
      let active = try KRPCValue.decodeBool(values[0])
      let hasFuel = try KRPCValue.decodeBool(values[2])
      let thrust = Double(try KRPCValue.decodeFloat(values[1]))
      inventory.append((engine, part))
      if !OrbitalEngineBinding.isOMSPartName(name) {
        if active { otherActive.append(engine) }
        continue
      }
      guard active && thrust > 0 && hasFuel else {
        throw FlightError.infeasible("STS Terrier OMS engine unavailable: " + name)
      }
      selectedParts.insert(part)
      selectedThrust += thrust
    }
    guard generation == operationGeneration else {
      throw FlightError.rearmRequired("Connection changed during OMS discovery")
    }
    guard selectedParts.count == OrbitalEngineBinding.requiredTerrierCount else {
      throw FlightError.infeasible("STS OMS requires exactly two active Terrier engines; SSME cannot substitute")
    }
    guard selectedParts.isSubset(of: partIdentifiers) else {
      throw FlightError.rearmRequired("OMS part not present in the selected vessel")
    }
    return (try OrbitalEngineBinding.resolve(parts: selectedParts, inventory: inventory),
      selectedParts, otherActive, selectedThrust)
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
      // The predicted burn duration is not an authorization deadline.
      // Attitude corrections or low throttle can lengthen the physical burn;
      // the controller retains measured delta-v and periapsis closure.
      // Atmospheric entry and plan identity remain hard propulsion boundaries.
      guard !burnFinished, stateTime >= plan.window.burnStart,
        latest.position.length-configuration.planet.radius > configuration.planet.atmosphereTop else {
        throw FlightError.invalidCommand("Propulsion outside the orbital deorbit envelope")
      }
    }
    busy = true; let epoch = generation; defer { busy = false }
    // Honor the orbital controller's actuator selection. Never globally
    // enable RCS for every orbit tick when powered reaction wheels suffice.
    var calls = safetyCalls(throttle: command.throttle,
      rcs: configuration.vehicle.rcsAttitudeControl && command.axes.rcs)
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
    _ = try await session.call(safetyCalls(rcs: false)+stop)
    burnFinished = true
  }
  public func beginEntry() async throws {
    guard armed, burnFinished, !busy else { throw FlightError.invalidCommand("OMS shutdown must precede entry transfer") }
    busy = true; defer { busy = false }
    // Entry is already committed: report a failure, never abandon the handover.
    do { try await enableControlSurfaceAxes() } catch {
      try? FileHandle.standardError.write(contentsOf: Data(
        "ksl-control-surfaces: entry axis check failed: \(error)\n".utf8))
    }
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
      // Neutral on reconnection; the next fresh measured controller tick
      // explicitly requests RCS again only when reaction wheels are absent.
      _ = try await session.call(safetyCalls(rcs: false) + neutral)
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
    if ownsRailsWarp {
      try? await warpSession.connect()
      _ = try? await warpSession.call([.init(procedure: "set_RailsWarpFactor",
        arguments: [KRPCValue.sint(0)])])
      ownsRailsWarp = false
    }
    await warpSession.close()
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
      let surfaces = (try? surfaceActuators.commandCalls(brake: 0, flap: 0)) ?? []
      _ = try? await session.call([release] + safetyCalls() + neutral + engineStop + surfaces)
      busy = false
    }
    armed = false
    latest = nil
    sampledAt = nil
    await session.close()
    identity = nil
    deorbitPlan = nil; burnEngines = []; burnEngineParts.removeAll()
    measuredOMSThrustAxis = nil; measuredOMSAvailableThrust = 0
    measuredOMSSpecificImpulse = 0; burnFinished = true
  }
  /// Reconnection is read-only. A caller must reacquire state and create a fresh runtime before arming.
  public func reconnect(expectedVesselName: String? = "STS") async throws -> KRPCVesselIdentity {
    await shutdown()
    return try await connect(expectedVesselName: expectedVesselName)
  }
}
