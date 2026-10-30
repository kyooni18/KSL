import Foundation
import KSLCore
import KSLDiagnostics
import KSLKRPC
import KSLPhysics
import KSLRuntime

/// NDJSON operator protocol v1 (the one `ksl` speaks to `landing_backend`) over stdin/stdout.
/// Supported: updateConfiguration, connect, engageReentry, engageFinalTest, abort, shutdown.
/// There is no deorbit planner, so createPlan/engage are rejected with an explicit error.
actor BackendServer {
  private let planetRadius: Double
  private var host = "127.0.0.1"
  private var port: UInt16 = 50000
  private let vessel: String
  private let airbrakeGroup: UInt32
  private var io: KRPCFlightIO?
  private var flight: Task<Void, Never>?
  private var connection = "disconnected"
  private var phase = "Idle"
  private var engaged = false
  private var status = "Idle"
  private var lastError: String?
  private var telemetry: [String: Any] = [:]
  private var command: [String: Any] = [:]
  private var finished = false

  init(configuration: MissionConfiguration, vessel: String, airbrakeGroup: UInt32) {
    planetRadius = configuration.planet.radius
    self.vessel = vessel
    self.airbrakeGroup = airbrakeGroup
  }

  private func emit(_ object: [String: Any]) {
    guard let data = try? JSONSerialization.data(withJSONObject: object) else { return }
    var line = data
    line.append(10)
    FileHandle.standardOutput.write(line)
  }
  private func snapshot() -> [String: Any] {
    var s: [String: Any] = [
      "phase": phase, "automationEngaged": engaged, "connectionStatus": connection,
      "statusMessage": status, "telemetry": telemetry, "command": command,
    ]
    if let lastError { s["lastError"] = lastError }
    return s
  }
  private func publish() { emit(["type": "snapshot", "snapshot": snapshot()]) }
  func ready() { emit(["type": "ready", "protocolVersion": 1, "configuration": [:], "snapshot": snapshot()]) }

  static func backendPhase(_ p: MissionPhase) -> String {
    switch p {
    case .entry: "MM304 Entry"
    case .acquisition, .headingAlignment: "TAEM"
    case .approach, .preflare: "Final Approach"
    case .flare: "Flare"
    case .touchdown: "Touchdown"
    case .rollout: "Rollout"
    case .complete: "Complete"
    case .abort: "Abort"
    }
  }

  func observe(_ record: FlightRecord) {
    let s = record.state
    let up = s.position.normalized
    let a = s.aerodynamics
    phase = Self.backendPhase(record.guidance.phase)
    telemetry = [
      "ut": s.time, "meanAltitude": s.position.length - planetRadius,
      "surfaceAltitude": s.radarAltitude, "trueAirSpeed": a.airspeed,
      "verticalSpeed": s.velocity.dot(up), "mach": a.mach, "dynamicPressure": a.dynamicPressure,
      "angleOfAttack": a.angleOfAttack * 180 / .pi, "roll": a.bank * 180 / .pi,
      "gForce": a.force.length / max(1, s.mass * 9.80665),
      "targetAoA": record.guidance.target.angleOfAttack * 180 / .pi,
      "targetRoll": record.guidance.target.bank * 180 / .pi,
    ]
    command = [
      "targetAoA": record.guidance.target.angleOfAttack * 180 / .pi,
      "targetRoll": record.guidance.target.bank * 180 / .pi,
    ]
    if !record.guidance.violations.isEmpty {
      status = "Swift guidance: " + record.guidance.violations.map { $0.message }.joined(separator: "; ")
    }
    publish()
  }

  func handle(line: String) async -> Bool {
    guard let data = line.data(using: .utf8),
      let request = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
      let method = request["method"] as? String
    else { return true }
    let id = request["id"] as? String ?? String(describing: request["id"] ?? "")
    func respond(_ ok: Bool, _ detail: String? = nil) {
      var m: [String: Any] = ["type": "response", "id": id, "ok": ok]
      if ok { m["result"] = [String: Any]() } else { m["error"] = detail ?? "failed" }
      emit(m)
    }
    switch method {
    case "updateConfiguration":
      if let c = request["configuration"] as? [String: Any],
        let conn = c["connection"] as? [String: Any]
      {
        if let a = conn["address"] as? String { host = a }
        if let p = conn["rpcPort"] as? Int, let n = UInt16(exactly: p) { port = n }
      }
      respond(true)
    case "connect":
      do {
        var cc = KRPCConnectionConfiguration()
        cc.host = host
        cc.port = port
        let flightIO = KRPCFlightIO(connection: cc, airbrakeActionGroup: airbrakeGroup)
        if ProcessInfo.processInfo.environment["KSL_SWIFT_RAW_AXES"] == "1" {
          await flightIO.setUseAutopilotAttitude(false)
        }
        _ = try await flightIO.connect(expectedVesselName: vessel)
        io = flightIO
        connection = "connected"
        status = "Connected (Swift engine)"
        respond(true)
        publish()
      } catch {
        connection = "failed"
        lastError = "\(error)"
        respond(false, lastError)
        publish()
      }
    case "createPlan", "engage":
      respond(false, "The Swift engine has no deorbit planner; start from mm304, mm305 or final-approach")
    case "engageReentry", "engageFinalTest":
      guard let flightIO = io, flight == nil else {
        respond(false, io == nil ? "Not connected" : "Already engaged")
        return true
      }
      do {
        try await flightIO.arm()
        let recorder = ServerRecorder(server: self)
        let runtime = try FlightRuntime(io: flightIO, recorder: recorder, configuration: .init())
        engaged = true
        status = "Swift guidance engaged"
        phase = "MM304 Entry"
        respond(true)
        publish()
        flight = Task { [weak self] in
          let outcome = try? await runtime.run(maximumDuration: 7200)
          await self?.flightEnded(outcome)
        }
      } catch {
        lastError = "\(error)"
        respond(false, lastError)
      }
    case "abort":
      flight?.cancel()
      respond(true)
    case "saveCheckpoint":
      // Between control steps only: the flight runtime holds the IO busy while commanding.
      guard let flightIO = io, let name = request["name"] as? String, !name.isEmpty else {
        respond(false, "saveCheckpoint needs a connection and a name")
        return true
      }
      do {
        try await flightIO.save(name: name)
        respond(true)
      } catch {
        respond(false, "\(error)")
      }
    case "shutdown":
      flight?.cancel()
      await flight?.value
      if let io { await io.shutdown() }
      respond(true)
      return false
    default:
      respond(false, "Unsupported method: \(method)")
    }
    return true
  }

  private func flightEnded(_ outcome: RuntimeOutcome?) {
    engaged = false
    flight = nil
    switch outcome {
    case .complete: phase = "Complete"; status = "Rollout complete"
    case .aborted, .cancelled: phase = "Abort"; status = "Aborted"
    case .timedOut: phase = "Abort"; status = "Timed out"
    case nil: phase = "Fault"; lastError = lastError ?? "Swift flight runtime failed"; status = "Fault"
    }
    publish()
  }
}

struct ServerRecorder: TelemetryRecording {
  let server: BackendServer
  func record(_ record: FlightRecord) async throws { await server.observe(record) }
  func close() async throws {}
}
