import Foundation
import KSLCore
import KSLDiagnostics
import KSLGuidance
import KSLKRPC
import KSLPhysics
import KSLRuntime

/// NDJSON protocol v1. Owns operator state; heavy planning and flight control
/// have separate actors and exchange immutable, generation-bound values.
actor BackendServer {
  var configuration: MissionConfiguration
  var settings: OperatorConfigurationResult
  let vessel: String
  var io: KRPCFlightIO?
  var flight: Task<Void, Never>?
  var planning: Task<Void, Never>?
  var preparing = false
  var generation: UInt64 = 0
  var connection = "disconnected", phase = "Idle", status = "Idle"
  var engaged = false
  var lastError: String?
  var telemetry: [String: Any] = [:], command: [String: Any] = [:]
  var plan: DeorbitPlan?
  var planningProgress: DeorbitPlanningProgress?
  var logDirectory: String?
  var lastPublished: ContinuousClock.Instant?

  init(configuration: MissionConfiguration, vessel: String, airbrakeGroup: UInt32) throws {
    self.vessel = vessel
    let path = ProcessInfo.processInfo.environment["KSL_SWIFT_OPERATOR_CONFIGURATION"] ?? "Configuration/sts.json"
    let data = FileManager.default.fileExists(atPath: path) ? try Data(contentsOf: URL(fileURLWithPath: path)) : Data("{}".utf8)
    settings = try OperatorConfigurationMapper.map(data, base: configuration, baseAirbrakeGroup: airbrakeGroup)
    self.configuration = settings.mission
  }
  func object<T: Encodable>(_ value: T) throws -> Any {
    try JSONSerialization.jsonObject(with: JSONEncoder().encode(value))
  }
  func emit(_ object: [String: Any]) {
    do {
      var data = try JSONSerialization.data(withJSONObject: object, options: .sortedKeys)
      data.append(10); try FileHandle.standardOutput.write(contentsOf: data)
    } catch {
      try? FileHandle.standardError.write(contentsOf: Data("Operator output failed: \(error)\n".utf8))
    }
  }
  func publish() { emit(["type": "snapshot", "snapshot": snapshot()]) }
  func ready() {
    emit(["type": "ready", "protocolVersion": 1,
      "configuration": (try? JSONSerialization.jsonObject(with: settings.effectiveOperatorJSON)) ?? [:], "snapshot": snapshot()])
  }
  func applyConfiguration(_ input: Any?) async throws {
    guard let input else { return }
    guard !engaged, flight == nil, planning == nil, !preparing else {
      throw FlightError.invalidCommand("Configuration is immutable while planning, preparing or engaged")
    }
    let data = try JSONSerialization.data(withJSONObject: input)
    let mapped = try OperatorConfigurationMapper.map(data, base: configuration,
      baseConnection: settings.connection, baseAirbrakeGroup: settings.airbrakeGroup,
      baseAllowReciprocal: settings.allowReciprocal)
    let changed = mapped.mission != configuration || mapped.connection != settings.connection
      || mapped.airbrakeGroup != settings.airbrakeGroup || mapped.allowReciprocal != settings.allowReciprocal
    configuration = mapped.mission; settings = mapped
    if changed {
      generation &+= 1; plan = nil
      if let io { await io.shutdown(); self.io = nil; connection = "disconnected" }
    }
    status = "Validated configuration applied"
  }
  func connectIfNeeded() async throws -> KRPCFlightIO {
    if let io { return io }
    var cc = KRPCConnectionConfiguration()
    cc.host = settings.connection.host; cc.port = settings.connection.port
    cc.timeout = settings.connection.timeout; cc.clientName = settings.connection.clientName
    let value = KRPCFlightIO(connection: cc, configuration: configuration, airbrakeActionGroup: settings.airbrakeGroup)
    _ = try await value.connect(expectedVesselName: vessel)
    io = value; connection = "connected"
    return value
  }
  func resolveHistoryAndRunway(sessionID: String? = nil) async throws -> (KRPCFlightIO, VehicleState, AerodynamicsEngine) {
    let previous = configuration
    let variant = ProcessInfo.processInfo.environment["KSP_LANDER_PHYSICS_VARIANT"]
    configuration = try await PhysicsModelLoader.resolvingDefaultArchive(configuration,
      path: ProcessInfo.processInfo.environment["KSL_SWIFT_PHYSICS_ARCHIVE"] ?? "Runtime/Physics/observations.sqlite3",
      variant: variant)
    if let sessionID {
      configuration.physicsArchive?.excludedSession = sessionID
      configuration.physicsArchive?.excludedFlight = sessionID
    }
    // The kRPC adapter never reads the archive identity; resolving it must not
    // cost a full reconnect (tens of seconds against a loaded KSP scene).
    var ioRelevant = configuration; ioRelevant.physicsArchive = previous.physicsArchive
    if ioRelevant != previous, let io { await io.shutdown(); self.io = nil }
    var value = try await connectIfNeeded(), state = try await value.readState()
    // The deorbit planner searches both runway orientations using complete
    // burn-to-stop evidence. Select by measured approach state only when
    // already inside the atmosphere (otherwise orbital track is misleading).
    if settings.allowReciprocal,
       state.position.length-configuration.planet.radius <= configuration.planet.atmosphereTop,
       state.contact == .airborne {
      let selected = RunwayApproachSelector.choose(state, configuration: configuration)
      if selected.runway != configuration.runway {
        configuration = selected
        await value.shutdown(); io = nil
        value = try await connectIfNeeded(); state = try await value.readState()
      }
    }
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: configuration)
    return (value, state, aero)
  }
  func recorder(phase: MissionPhase, aero: AerodynamicsEngine, sessionID: String) throws -> ServerRecorder {
    let directory = URL(fileURLWithPath: "runs/native-swift-"+sessionID, isDirectory: true)
    let file = try SessionTelemetryRecorder(directory: directory, configuration: configuration,
      sessionID: sessionID, initialPhase: phase, context: aero.history?.history.contexts.first)
    logDirectory = directory.standardizedFileURL.path
    return .init(server: self, file: file)
  }
  func handle(line: String) async -> Bool {
    guard let data = line.data(using: .utf8) else { return true }
    let request: [String: Any]
    do {
      guard let value = try JSONSerialization.jsonObject(with: data) as? [String: Any] else {
        throw FlightError.protocolViolation("Request must be an object")
      }
      request = value
    } catch { emit(["type": "response", "id": "", "ok": false, "error": String(describing: error)]); return true }
    let id = request["id"] as? String ?? String(describing: request["id"] ?? "")
    func respond(_ ok: Bool, _ detail: String? = nil) {
      var value: [String: Any] = ["type": "response", "id": id, "ok": ok]
      if ok { value["result"] = ["snapshot": snapshot()] }
      else { value["error"] = detail ?? "Request failed" }
      emit(value)
    }
    do {
      guard let method = request["method"] as? String else { throw FlightError.protocolViolation("Missing method") }
      switch method {
      case "updateConfiguration":
        guard request["configuration"] != nil else { throw FlightError.invalidConfiguration("Missing configuration") }
        try await applyConfiguration(request["configuration"])
      case "connect":
        guard !engaged, !preparing else { throw FlightError.invalidCommand("Already engaged or preparing") }
        try await applyConfiguration(request["configuration"])
        _ = try await connectIfNeeded(); status = "Connected to native Swift flight computer"
      case "createPlan":
        try await applyConfiguration(request["configuration"])
        try await startPlanning()
      case "engage":
        try await applyConfiguration(request["configuration"])
        let authorization = request["authorizeReducedAssurancePlan"] as? String
        try await engageDeorbit(reviewedPlanID: request["planID"] as? String, authorization: authorization)
      case "engageReentry", "engageFinalTest", "engageMM305", "engageTAEM":
        try await applyConfiguration(request["configuration"])
        let requested: MissionPhase
        if method == "engageFinalTest" { requested = .approach }
        else if method == "engageMM305" || method == "engageTAEM" { requested = .acquisition }
        else if let name = request["phase"] as? String {
          switch name.lowercased() {
          case "mm304", "entry": requested = .entry
          case "mm305", "taem": requested = .acquisition
          case "final", "approach": requested = .approach
          default: throw FlightError.invalidCommand("Unknown requested landing phase: \(name)")
          }
        } else { requested = .entry }
        try engageLanding(phase: requested)
      case "abort":
        await stopAll(); phase = "Abort"; status = "Control released; a fresh engage is required"
      case "saveCheckpoint":
        guard !engaged, !preparing, let io, let name = request["name"] as? String, !name.isEmpty else {
          throw FlightError.invalidCommand("Checkpoint saving requires an idle connection and a name")
        }
        try await io.save(name: name)
      case "disconnect":
        await stopAll(); plan = nil; phase = "Idle"; status = "Disconnected"
      case "getSnapshot", "snapshot":
        break
      case "shutdown":
        await stopAll(); respond(true); return false
      default: throw FlightError.invalidCommand("Unsupported method: \(method)")
      }
      lastError = nil; respond(true); publish()
    } catch {
      lastError = String(describing: error); status = lastError!
      respond(false, lastError); publish()
    }
    return true
  }
  func stopAll() async {
    generation &+= 1
    let pendingPlan = planning, pendingFlight = flight
    pendingPlan?.cancel(); pendingFlight?.cancel()
    await pendingPlan?.value; await pendingFlight?.value
    planning = nil; flight = nil; preparing = false; engaged = false
    if let io { await io.shutdown(); self.io = nil }
    connection = "disconnected"
  }
  func flightEnded(_ outcome: RuntimeOutcome?, error: String?, generation epoch: UInt64) {
    guard epoch == generation else { return }
    engaged = false; flight = nil; io = nil; connection = "disconnected"
    switch outcome {
    case .complete: phase = "Complete"; status = "Measured touchdown and full rollout complete"
    case .aborted: phase = "Abort"; status = "Flight executive aborted; inspect the full-rate record"
    case .cancelled: phase = "Abort"; status = "Cancelled; native controls released"
    case .timedOut: phase = "Abort"; status = "Mission time limit reached"
    case nil: phase = "Fault"; status = error ?? "Runtime failed without an outcome"
    }
    if let error { lastError = error }
    publish()
  }
}

struct ServerRecorder: TelemetryRecording {
  let server: BackendServer
  let file: SessionTelemetryRecorder
  func record(_ record: FlightRecord) async throws {
    // Keep live status flowing even when the independent disk sink fails.
    await server.observe(record)
    try await file.record(record)
  }
  func close() async throws { try await file.close() }
}
