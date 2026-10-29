import Dispatch
import Foundation
import KSLCore
import KSLDiagnostics
import KSLGuidance
import KSLKRPC
import KSLPhysics
import KSLRuntime
import KSLSimulation

@main
struct LandingCLI {
  static func main() async {
    do { try await run(Array(CommandLine.arguments.dropFirst())) } catch {
      FileHandle.standardError.write(Data("ksl-landing: \(error)\n".utf8))
      exit(1)
    }
  }
  static func run(_ args: [String]) async throws {
    func option(_ key: String) -> String? {
      guard let i = args.firstIndex(of: key), i + 1 < args.count else { return nil }
      return args[i + 1]
    }
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
    func output<T: Encodable>(_ value: T) throws {
      let data = try encoder.encode(value)
      if let path = option("--output") {
        try data.write(to: URL(fileURLWithPath: path))
      } else {
        FileHandle.standardOutput.write(data)
        FileHandle.standardOutput.write(Data([10]))
      }
    }
    func scenarioFromOptions() throws -> SimulationScenario {
      if let path = option("--scenario") {
        let url = URL(fileURLWithPath: path)
        return path.hasSuffix(".ini")
          ? try .loadINI(url: url)
          : try JSONDecoder().decode(SimulationScenario.self, from: Data(contentsOf: url))
      }
      func number(_ key: String, _ fallback: Double) throws -> Double {
        guard let raw = option(key) else { return fallback }
        guard let value = Double(raw), value.isFinite else {
          throw FlightError.invalidConfiguration("Invalid numeric option \(key)")
        }
        return value
      }
      var result: SimulationScenario
      if args.contains("--entry") {
        result = try .entry(
          along: number("--along", -450_000), cross: number("--cross", 0),
          headingError: number("--heading-error", 0),
          speed: number("--speed", 1900), height: number("--height", 72_000),
          flightPath: number("--flight-path", -3))
      } else {
        result = try .approach(
          along: number("--along", -10_000), cross: number("--cross", 0),
          height: number("--height", 4200),
          speed: number("--speed", 150), flightPath: number("--flight-path", -24),
          headingError: number("--heading-error", 0))
      }
      result.initialState.mass = try number("--mass", result.configuration.vehicle.nominalMass)
      return result
    }
    switch args.first ?? "help" {
    case "simulate":
      var scenario = try scenarioFromOptions()
      if args.contains("--spiral-only") {
        scenario.configuration.guidance.allowDirectTerminal = false
      }
      if let s = option("--duration"), let duration = Double(s) {
        scenario.maximumDuration = duration
      }
      let engine = try SimulationEngine(
        scenario: scenario, liftScale: Double(option("--lift-scale") ?? "1") ?? 1,
        dragScale: Double(option("--drag-scale") ?? "1") ?? 1,
        densityScale: Double(option("--density-scale") ?? "1") ?? 1,
        windENU: .init(
          Double(option("--wind-east") ?? "0") ?? 0, Double(option("--wind-north") ?? "0") ?? 0,
          Double(option("--wind-up") ?? "0") ?? 0),
        controlAuthorityScale: Double(option("--actuator-scale") ?? "1") ?? 1)
      let result = try engine.run(
        logURL: option("--log").map { URL(fileURLWithPath: $0) },
        logEvery: Int(option("--log-every") ?? "1") ?? 1)
      try output(result)
      if args.contains("--require-success") && result.outcome != "complete" { exit(2) }
    case "configuration": try output(MissionConfiguration())
    case "scenario": try output(scenarioFromOptions())
    case "plan":
      let scenario = try scenarioFromOptions()
      let c = scenario.configuration
      let atmosphere = try AtmosphericModelEngine(planet: c.planet)
      let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
      let physics = ShuttlePhysicsEngine(
        configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      let planner = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      try output(
        planner.plan(
          from: physics.observe(scenario.initialState), allowDirect: !args.contains("--spiral-only")
        ))
    case "replay":
      guard let path = option("--input") else {
        throw FlightError.invalidConfiguration("replay requires --input")
      }
      let c: MissionConfiguration
      if let path = option("--configuration") {
        c = try JSONDecoder().decode(
          MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      } else {
        c = .init()
      }
      let reader = try FlightReplayReader(url: URL(fileURLWithPath: path))
      defer { try? reader.close() }
      let computer = try FlightComputer(
        configuration: c, atmosphere: try .init(planet: c.planet),
        aerodynamics: try .init(referenceArea: c.vehicle.referenceArea))
      var count = 0
      var mismatches = 0
      var phases: [String] = []
      while let record = try reader.next() {
        let r = try computer.step(record.state)
        if phases.last != r.guidance.phase.rawValue { phases.append(r.guidance.phase.rawValue) }
        if r.command != record.command || r.guidance.target != record.guidance.target
          || r.guidance.phase != record.guidance.phase
        {
          mismatches += 1
        }
        count += 1
      }
      struct ReplayReport: Encodable {
        let records: Int
        let commandOrGuidanceMismatches: Int
        let phases: [String]
        let evidence: String
      }
      try output(
        ReplayReport(
          records: count, commandOrGuidanceMismatches: mismatches, phases: phases,
          evidence: "snapshot-replay-only"))
      if args.contains("--verify-commands") && mismatches > 0 { exit(2) }
    case "legacy-audit", "import-legacy":
      guard let path = option("--input") else {
        throw FlightError.invalidConfiguration("Legacy import requires --input")
      }
      let reader = try LegacyTelemetryReader(
        url: URL(fileURLWithPath: path), skipMalformed: args.contains("--skip-malformed"),
        session: option("--session"))
      defer { try? reader.close() }
      let writer = try option("--states").map { try JSONLFileWriter(url: URL(fileURLWithPath: $0)) }
      defer { try? writer?.close() }
      var selected: LegacySnapshot?
      let time = Double(option("--at-time") ?? "0") ?? 0
      while let snapshot = try reader.next() {
        try writer?.write(snapshot)
        if selected == nil && snapshot.state.time >= time && snapshot.state.contact == .airborne {
          selected = snapshot
        }
      }
      if args.first == "import-legacy" {
        guard let selected else {
          throw FlightError.invalidTelemetry(
            "No complete airborne snapshot matches the requested session/time")
        }
        try output(
          SimulationScenario(
            name: "archival-\(selected.session)-line-\(selected.line)", configuration: .init(),
            initialState: selected.state))
      } else {
        try output(reader.statistics)
      }
    case "inspect", "fly":
      var connection = KRPCConnectionConfiguration()
      if let host = option("--host") { connection.host = host }
      if let port = option("--port"), let number = UInt16(port) { connection.port = number }
      let config: MissionConfiguration
      if let path = option("--configuration") {
        config = try JSONDecoder().decode(
          MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      } else {
        config = .init()
      }
      try config.validate()
      let io = KRPCFlightIO(
        connection: connection, configuration: config,
        airbrakeActionGroup: UInt32(option("--airbrake-group") ?? "1") ?? 1)
      do {
        let identity = try await io.connect(expectedVesselName: option("--vessel") ?? "STS")
        if args.first == "inspect" {
          let state = try await io.readState()
          struct Inspection: Encodable {
            let identity: KRPCVesselIdentity
            let state: VehicleState
          }
          try output(Inspection(identity: identity, state: state))
          await io.shutdown()
        } else {
          let logPath = option("--log") ?? "ksl-live-\(Int(Date().timeIntervalSince1970)).jsonl"
          try encoder.encode(config).write(
            to: URL(fileURLWithPath: logPath + ".configuration.json"))
          let recorder = try JSONLTelemetryRecorder(url: URL(fileURLWithPath: logPath))
          try await io.arm()
          let runtime = try FlightRuntime(
            io: io, recorder: recorder, configuration: config,
            maximumOutage: Double(option("--max-outage") ?? "20") ?? 20)
          let task = Task {
            try await runtime.run(maximumDuration: Double(option("--duration") ?? "2400") ?? 2400)
          }
          signal(SIGINT, SIG_IGN)
          let interrupt = DispatchSource.makeSignalSource(signal: SIGINT, queue: .global())
          interrupt.setEventHandler { task.cancel() }
          interrupt.resume()
          defer {
            interrupt.cancel()
            signal(SIGINT, SIG_DFL)
          }
          let result = try await task.value
          try output([
            "outcome": result.rawValue, "evidence": "live-kRPC-telemetry", "log": logPath,
          ])
          if result != .complete { exit(2) }
        }
      } catch {
        await io.shutdown()
        throw error
      }
    default:
      print(
        """
        KSL Swift landing runtime (Swift 6)
          simulate [--entry | --scenario file.ini|file.json] [--duration seconds]
                   [--log file.jsonl] [--log-every 1] [--output report.json] [--require-success]
                   [--lift-scale 1] [--drag-scale 1] [--density-scale 1] [--actuator-scale 1]
                   [--wind-east 0] [--wind-north 0] [--wind-up 0] [--spiral-only]
          scenario [--entry] [--output scenario.json]
          plan [--entry | --scenario file] [--spiral-only] [--output plan.json]
          Initial-state options: --along meters --cross meters --height meters
             --speed m/s --flight-path degrees --heading-error degrees --mass kg
          configuration [--output configuration.json]
          replay --input flight.jsonl [--configuration config.json] [--verify-commands]
          legacy-audit --input archive.jsonl [--skip-malformed] [--session id] [--states snapshots.jsonl]
          import-legacy --input archive.jsonl [--skip-malformed] [--session id]
                        [--at-time UT] --output historical-scenario.json
          inspect --host 127.0.0.1 --port 50000 --vessel STS
          fly --host 127.0.0.1 --port 50000 --vessel STS [--log live.jsonl]
              [--configuration configuration.json] [--airbrake-group 1] [--max-outage 20]
        inspect is read-only. fly takes control of the named vessel after deorbit.
        All simulation output is labeled offline-model-only. No ascent, OMS or deorbit commands exist.
        A full-rate log and its exact configuration are required for command-identical replay.
        """)
    }
  }
}
