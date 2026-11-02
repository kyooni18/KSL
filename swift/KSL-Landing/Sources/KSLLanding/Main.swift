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
    // Help must never fall through to an operational subcommand: in
    // particular, `fly --help` must not connect, arm, or command a vessel.
    let command = args.contains("--help") || args.contains("-h") ? "help" : (args.first ?? "help")
    switch command {
    case "deorbit-plan":
      // Offline replanning from a recorded plan's source state and configuration.
      guard let input = option("--input") else {
        throw FlightError.invalidConfiguration("deorbit-plan requires --input plan.json")
      }
      let recorded = try JSONDecoder().decode(DeorbitPlan.self, from: Data(contentsOf: URL(fileURLWithPath: input)))
      let configuration = try await PhysicsModelLoader.resolvingDefaultArchive(recorded.configuration,
        path: ProcessInfo.processInfo.environment["KSL_SWIFT_PHYSICS_ARCHIVE"] ?? "Runtime/Physics/observations.sqlite3")
      let clock = ContinuousClock(), start = clock.now
      let aero = try await PhysicsModelLoader.aerodynamics(configuration: configuration)
      let loaded = clock.now
      let plan = try await DeorbitPlanner.planSelectingRunway(from: recorded.source,
        configuration: configuration, aerodynamics: aero, allowReciprocal: true) { p in
        FileHandle.standardError.write(Data("\(clock.now - start) \(p.runwayName ?? "") \(p.stage) \(p.candidate)/\(p.totalCases)\n".utf8))
      }
      print("archive \(loaded - start) planning \(clock.now - loaded) runway \(plan.configuration.runway.name) qualified \(plan.executionQualified)")
      if let output = option("--output") { try JSONEncoder().encode(plan).write(to: URL(fileURLWithPath: output)) }
    case "deorbit-trace":
      guard let input = option("--input"), let path = option("--log") else {
        throw FlightError.invalidConfiguration("deorbit-trace requires --input plan.json and --log output.jsonl")
      }
      let plan = try JSONDecoder().decode(DeorbitPlan.self, from: Data(contentsOf: URL(fileURLWithPath: input)))
      try await DeorbitDiagnostic.traceNominal(plan: plan, output: URL(fileURLWithPath: path),
        entryReplanning: args.contains("--entry-replanning"))
    case "deorbit-screen":
      guard let input = option("--input"), let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("deorbit-screen requires --input state.json and --configuration")
      }
      let configuration = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      let state = try JSONDecoder().decode(VehicleState.self, from: Data(contentsOf: URL(fileURLWithPath: input)))
      guard let propulsion = state.propulsion, propulsion.evidence == .measured else {
        throw FlightError.invalidTelemetry("Deorbit screening requires measured propulsion")
      }
      let search = try DeorbitWindowSearch(configuration: configuration).search(from: state, propulsion: propulsion)
      struct CandidateForecastSummary: Encodable {
        var burnStart: Double
        var entryRange: Double
        var entryCrossrange: Double
        var terminalAlong: Double
        var terminalCross: Double
        var terminalAltitude: Double
        var terminalMach: Double
        var reachedTerminalRegion: Bool
        var feasible: Bool
        var captureFeasible: Bool
        var captureDeficit: String
        var energyMargin: Double
        var violationCount: Int
        /// State closest to the 25 km MM304-to-MM305 target (runway chart).
        var nearTAEMAltitude: Double?
        var nearTAEMAlong: Double?
        var nearTAEMCross: Double?
        var nearTAEMSpeed: Double?
      }
      struct ScreenReport: Encodable {
        // This inexpensive command starts entry at the ideal window state;
        // unlike DeorbitPlanner it does not execute the controlled OMS burn.
        let evidence = "offline-ideal-orbital-entry-screen-only-not-an-execution-certificate"
        let evaluated: Int
        let horizon: Double
        let rejections: [String: Int]
        let candidates: [DeorbitWindow]
        let preview: [CandidateForecastSummary]?
      }
      var preview: [CandidateForecastSummary]? = nil
      if args.contains("--forecast") {
        let atmosphere = try AtmosphericModelEngine(planet: configuration.planet)
        let aero = try await PhysicsModelLoader.aerodynamics(configuration: configuration)
        let entry = LandingAwareWindowSelector(configuration: configuration,
          atmosphere: atmosphere, aerodynamics: aero)
        let forecast = EntryCaptureForecaster(configuration: configuration,
          atmosphere: atmosphere, aerodynamics: aero)
        preview = search.candidates.map { window in
          let result = forecast.forecast(from: entry.entryState(window))
          let targetHeight = configuration.entrySettings.terminalTargetAltitude
            - configuration.runway.elevation
          let nearTAEM = result.track.filter { $0.count >= 5 }
            .min { abs($0[3]-targetHeight) < abs($1[3]-targetHeight) }
          return CandidateForecastSummary(burnStart: window.burnStart,
            entryRange: window.entryRange, entryCrossrange: window.crossrange,
            terminalAlong: result.terminalAlong, terminalCross: result.terminalCross,
            terminalAltitude: result.terminalAltitude, terminalMach: result.terminalMach,
            reachedTerminalRegion: result.reachedTerminalRegion, feasible: result.feasible,
            captureFeasible: result.capture.feasible,
            captureDeficit: result.capture.deficit.rawValue,
            energyMargin: result.capture.envelope.energyMargin,
            violationCount: result.violations.count,
            nearTAEMAltitude: nearTAEM?[3],
            nearTAEMAlong: nearTAEM?[1],
            nearTAEMCross: nearTAEM?[2],
            nearTAEMSpeed: nearTAEM?[4])
        }
      }
      try output(ScreenReport(evaluated: search.evaluated, horizon: search.horizon,
        rejections: search.rejections, candidates: search.candidates, preview: preview))
    case "taem-response-audit":
      guard let input = option("--input"), let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("taem-response-audit requires --input and --configuration")
      }
      let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      try output(await TAEMResponseAudit.run(input: input, configuration: c))
    case "pitch-balance-probe":
      guard let input = option("--input"), let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("pitch-balance-probe requires --input and --configuration")
      }
      let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      try await PitchBalanceProbe.run(input: input, configuration: c)
    case "entry-energy-probe":
      guard let input = option("--input"), let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("entry-energy-probe requires --input and --configuration")
      }
      let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      if let elapsed = option("--compare-from").flatMap(Double.init) {
        try await EntryEnergyProbe.compare(input: input, configuration: c, elapsed: elapsed,
          duration: Double(option("--duration") ?? "120") ?? 120)
      } else { try await EntryEnergyProbe.run(input: input, configuration: c) }
    case "shuttlesim-controller":
      try await ShuttleSimStream.run(arguments: args)
    case "entry-forecast":
      // Inspect a single physical MM304/TAEM continuation from recorded
      // telemetry. Never connects to KSP; captures cannot qualify a landing.
      guard let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("entry-forecast requires --configuration")
      }
      let c = try JSONDecoder().decode(MissionConfiguration.self,
        from: Data(contentsOf: URL(fileURLWithPath: path)))
      let state = try JSONDecoder().decode(VehicleState.self,
        from: FileHandle.standardInput.readDataToEndOfFile())
      let atm = try AtmosphericModelEngine(planet: c.planet)
      let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
      let forecast = EntryCaptureForecaster(configuration: c, atmosphere: atm, aerodynamics: aero)
        .forecast(from: state, rangeMultiplier: option("--range-multiplier").flatMap(Double.init) ?? 1.0,
          crossAim: option("--cross-aim").flatMap(Double.init) ?? 0.0)
      try output(forecast)
    case "taem-interface":
      // Offline: place the TAEM interface on this vehicle's nominal TAEM
      // energy line at configured Mach. Derives altitude, speed, q and
      // range together; reports a straight-in downmode if overhead fails.
      guard let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("taem-interface requires --configuration")
      }
      let c = try JSONDecoder().decode(MissionConfiguration.self,
        from: Data(contentsOf: URL(fileURLWithPath: path)))
      let atm = try AtmosphericModelEngine(planet: c.planet)
      let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
      print(try TAEMInterfacePlacement.solve(configuration: c, atmosphere: atm, aerodynamics: aero))
    case "entry-plan":
      // Offline diagnostic of the exact MM304 proposal built from a recorded
      // VehicleState. Does not connect to or control KSP.
      guard let path = option("--configuration") else {
        throw FlightError.invalidConfiguration("entry-plan requires --configuration")
      }
      let c = try JSONDecoder().decode(MissionConfiguration.self,
        from: Data(contentsOf: URL(fileURLWithPath: path)))
      let state = try JSONDecoder().decode(VehicleState.self,
        from: FileHandle.standardInput.readDataToEndOfFile())
      let atmosphere = try AtmosphericModelEngine(planet: c.planet)
      let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
      let entry = EntryGuidanceEngine(configuration: c, atmosphere: atmosphere,
        aerodynamics: aero)
      if let request = entry.request(from: state) { try output(entry.plan(request)) }
    case "capture-probe":
      // Diagnostic: run MM305's own closed-loop capture evaluation on a
      // recorded VehicleState (JSON on stdin) with a run's configuration.
      guard let path = option("--configuration") else { throw FlightError.invalidConfiguration("--configuration required") }
      let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      let state = try JSONDecoder().decode(VehicleState.self, from: FileHandle.standardInput.readDataToEndOfFile())
      let atmosphere = try AtmosphericModelEngine(planet: c.planet)
      let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
      if args.contains("--scan-objectives") {
        let frame = RunwayFrame(runway: c.runway, planet: c.planet)
        let profile = ApproachProfile(configuration: c)
        let exit = profile.reference(along: -c.guidance.finalDistance)
        let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
        let evaluator = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
        let e = c.entrySettings
        for height in [e.terminalTargetAltitude, e.terminalMinimumAltitude, e.terminalMaximumAltitude] {
          let glide = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
          let distance = c.guidance.finalDistance + (height - c.runway.elevation - exit.height) / tan(glide)
          for mach in [e.terminalTargetMach, e.terminalTargetMach-e.terminalMachHalfWidth] {
            for range in [distance, e.terminalTargetRange, c.guidance.finalDistance + (distance-c.guidance.finalDistance)/2] {
              let position = frame.position(along: -range, cross: 0, height: height-c.runway.elevation)
              let speed = mach * atmosphere.sample(position: position, time: state.time).speedOfSound
              let forward = frame.direction(at: .init(-range, 0), tangent: .init(1, 0))
              for gamma in [-glide, atan2(exit.height-(height-c.runway.elevation), range-c.guidance.finalDistance)] {
                let velocity = (forward*cos(gamma)+position.normalized*sin(gamma))*speed
                let sample = physics.observe(.init(time: state.time, position: position, velocity: velocity,
                  attitude: .windAttitude(position: position, airVelocity: velocity, alpha: e.nominalAlpha, bank: 0), mass: state.mass))
                let capture = evaluator.evaluate(sample)
                if capture.feasible, args.contains("--first-feasible") {
                  try output(sample)
                  return
                }
                print(String(format: "OBJECTIVE h=%.0f M=%.2f range=%.0f gamma=%.1f feasible=%@ margin=%.0f deficit=%@ completed=%d violations=%@",
                  height, mach, range, degrees(gamma), String(capture.feasible), capture.envelope.energyMargin,
                  capture.deficit.rawValue, capture.envelope.candidatesCompleted, capture.envelope.dominantViolations.description))
              }
            }
          }
        }
        return
      }
      var capture = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
        .evaluate(state, certify: true)
      capture.plan = nil
      if args.contains("--trace") {
        // Priced reference vs flown state along the first priced candidate.
        guard let plan = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
          .plan(from: state, allowDirect: c.guidance.allowDirectTerminal, preview: true).plan else { return }
        var next = 0.0
        let frame = RunwayFrame(runway: c.runway, planet: c.planet)
        _ = TerminalTrajectoryEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
          .evaluate(plan: plan, from: state, controlSnapshot: nil, previousTarget: nil, lift: 1, drag: 1) { s, ref, target in
            guard ref.distance >= next else { return }
            next = ref.distance + Double(ProcessInfo.processInfo.environment["KSL_TRACE_STEP"] ?? "2000")!
            let a = s.aerodynamics
            print(String(format: "d=%6.0f seg=%@ V ref=%5.1f act=%5.1f | h ref=%6.0f act=%6.0f | alpha ref=%5.1f tgt=%5.1f act=%5.1f | bank ref=%5.1f act=%5.1f | beta=%4.1f CD*q/m=%.2f",
              ref.distance, ref.segment.rawValue, ref.speed, a.airspeed, ref.height, frame.coordinates(of: s.position).height,
              ref.angleOfAttack * 57.3, target.angleOfAttack * 57.3, a.angleOfAttack * 57.3, ref.bank * 57.3, a.bank * 57.3, a.sideslip * 57.3,
              a.drag / s.mass))
          }
        return
      }
      if args.contains("--closed-loop-cases") {
        let result = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
          .plan(from: state, allowDirect: c.guidance.allowDirectTerminal)
        try output(result.closedLoopAdvisories ?? [])
      } else { try output(capture) }
    case "simulate":
      var scenario = try scenarioFromOptions()
      if args.contains("--spiral-only") {
        scenario.configuration.guidance.allowDirectTerminal = false
      }
      if let s = option("--duration"), let duration = Double(s) {
        scenario.maximumDuration = duration
      }
      let engine = try SimulationEngine(scenario: scenario)
      let result = try engine.run(
        logURL: option("--log").map { URL(fileURLWithPath: $0) },
        logEvery: Int(option("--log-every") ?? "1") ?? 1)
      try output(result)
      if args.contains("--require-success") && result.outcome != "complete" { exit(2) }
    case "physics-audit":
      guard let path = option("--database"), let structure = option("--structure-id") else {
        throw FlightError.invalidConfiguration("physics-audit requires --database and --structure-id")
      }
      let store = try PhysicsObservationStore(path: path)
      var contexts = try await store.contexts().filter { $0.structureID == structure }
      if let environment = option("--environment-id") { contexts = contexts.filter { $0.environmentID == environment } }
      guard Set(contexts.map(\.environmentID)).count == 1 else {
        throw FlightError.invalidConfiguration("Select exactly one archived environment with --environment-id")
      }
      let history = try await store.history(contexts: contexts, excludingSession: option("--exclude-session") ?? "",
        excludingFlight: option("--exclude-flight") ?? "", planet: .init())
      try output(history)
    case "configuration":
      var c = MissionConfiguration(); c.entry = c.entrySettings
      try output(c)
    case "scenario": try output(scenarioFromOptions())
    case "runway-select":
      let scenario = try scenarioFromOptions()
      let input = scenario.configuration
      let state = scenario.initialState
      let options = RunwayApproachSelector.candidates(configuration: input)
      let selected = RunwayApproachSelector.choose(state, configuration: input)
      struct RunwayDecision: Encodable {
        let selected: String
        let evidence: String
        let options: [RunwayMetric]
      }
      struct RunwayMetric: Encodable {
        let name: String
        let turnToInletDegrees: Double
        let turnIntoFinalDegrees: Double
        let minimumHeadingTravelDegrees: Double
        let pathDistanceMetres: Double
      }
      try output(RunwayDecision(selected: selected.runway.name,
        evidence: "Measured entry state; necessary heading geometry only, not landing qualification",
        options: options.map {
          let x = RunwayApproachSelector.assess(state, configuration: $0)
          return RunwayMetric(name: x.runway.name,
            turnToInletDegrees: degrees(x.turnToInlet),
            turnIntoFinalDegrees: degrees(x.turnIntoFinal),
            minimumHeadingTravelDegrees: degrees(x.minimumHeadingTravel),
            pathDistanceMetres: x.pathDistance)
        }))
    case "plan":
      let scenario = try scenarioFromOptions()
      let c = scenario.configuration
      let atmosphere = try AtmosphericModelEngine(planet: c.planet)
 let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea,
   speedbrakeFullDeploymentDragArea: c.vehicle.speedbrakeFullDeploymentDragArea)
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
      let computer = try await FlightRecordReplayer(
        configuration: c, atmosphere: try .init(planet: c.planet),
        aerodynamics: PhysicsModelLoader.aerodynamics(configuration: c))
      var count = 0
      var mismatches = 0
      var phases: [String] = []
      while let record = try reader.next() {
        let r = try computer.step(record)
        if phases.last != r.guidance.phase.rawValue { phases.append(r.guidance.phase.rawValue) }
        if r.command != record.command || r.guidance.target != record.guidance.target
          || r.guidance.phase != record.guidance.phase || r.orbitalCommand != record.orbitalCommand
          || (record.guidance.targetHistory != nil && r.guidance.targetHistory != record.guidance.targetHistory)
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
          evidence: "full-rate-snapshot-and-accepted-proposal-replay"))
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
    case "backend":
      // NDJSON operator protocol v1 server (drop-in for `landing_backend` in `ksl`).
      let server = try BackendServer(
        configuration: .init(),
        vessel: ProcessInfo.processInfo.environment["KSL_SWIFT_VESSEL"] ?? "STS",
        airbrakeGroup: UInt32(ProcessInfo.processInfo.environment["KSL_SWIFT_AIRBRAKE_GROUP"] ?? "1")
          ?? 1)
      await server.ready()
      while let line = readLine() {
        if await !server.handle(line: line) { break }
      }
    case "inspect", "fly":
      var connection = KRPCConnectionConfiguration()
      if let host = option("--host") { connection.host = host }
      if let port = option("--port"), let number = UInt16(port) { connection.port = number }
      var config: MissionConfiguration
      if let path = option("--configuration") {
        config = try JSONDecoder().decode(
          MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
      } else {
        config = .init()
      }
      config.entry = config.entrySettings
      try config.validate()
      var io = KRPCFlightIO(
        connection: connection, configuration: config,
        airbrakeActionGroup: UInt32(option("--airbrake-group") ?? "1") ?? 1)
      do {
        let identity = try await io.connect(expectedVesselName: option("--vessel") ?? "STS")
        if args.first == "inspect" {
          let state = try await io.readState()
          struct Inspection: Encodable {
            let identity: KRPCVesselIdentity
            let state: VehicleState
            let attitudeCheckDegrees: [String: Double]?
          }
          // Self-consistency of the native FCS target: the attitude rebuilt from the measured
          // angle of attack and bank must equal the measured nose/roof directions.
          var check: [String: Double]?
          if let t = FlightComputer.attitudeTarget(
            state: state, angleOfAttack: state.aerodynamics.angleOfAttack,
            bank: state.aerodynamics.bank)
          {
            let nose = state.attitude.rotate(.init(1, 0, 0)), roof = state.attitude.rotate(.init(0, 0, -1))
            func angle(_ a: Vector3, _ b: Vector3) -> Double {
              acos(max(-1, min(1, a.dot(b)))) * 180 / .pi
            }
            check = ["nose": angle(t.nose, nose), "roof": angle(t.roof, roof)]
          }
          try output(Inspection(identity: identity, state: state, attitudeCheckDegrees: check))
          await io.shutdown()
        } else {
          // Native flights always have a discoverable KSP REPLAY journal.
          // An explicit --log remains authoritative, but the unconfigured
          // default now lives in runs/<unique-id>/live-sts-flight.jsonl rather
          // than an unindexed JSONL in the shell's working directory.
          let logPath: String
          if let explicit = option("--log") {
            logPath = explicit
          } else {
            let runs = ProcessInfo.processInfo.environment["KSL_RUNS_DIR"] ?? "runs"
            let session = "\(Int(Date().timeIntervalSince1970))-swift-native-\(UUID().uuidString.prefix(8))"
            logPath = URL(fileURLWithPath: runs).appendingPathComponent(session)
              .appendingPathComponent("live-sts-flight.jsonl").path
          }
          try FileManager.default.createDirectory(
            at: URL(fileURLWithPath: logPath).deletingLastPathComponent(),
            withIntermediateDirectories: true)
          let source = try await io.readState()
          let originalRunway = config.runway
          let aero = try await PhysicsModelLoader.aerodynamics(configuration: config)
          var selectedDeorbitPlan: DeorbitPlan?
          if args.contains("--deorbit") {
            if let planPath = option("--deorbit-plan") {
              selectedDeorbitPlan = try JSONDecoder().decode(DeorbitPlan.self,
                from: Data(contentsOf: URL(fileURLWithPath: planPath)))
            } else {
              selectedDeorbitPlan = try await DeorbitPlanner.planSelectingRunway(
                from: source, configuration: config, aerodynamics: aero,
                allowReciprocal: !args.contains("--fixed-runway")) { progress in
                  let sim = progress.simulationElapsed.map { " sim=\(String(format: "%.1f", $0))" } ?? ""
                  let phase = progress.phase.map { " phase=\($0)" } ?? ""
                  let message = "deorbit \(progress.stage) candidate=\(progress.candidate) case=\(progress.stressCase)/\(progress.totalCases)\(sim)\(phase)\n"
                  FileHandle.standardError.write(Data(message.utf8))
                }
            }
            // Before publishing a runway choice, connecting an armed backend,
            // or recording a burn plan, enforce the actual route gate. A
            // failed offline prediction must not masquerade as a selection.
            if let planned = selectedDeorbitPlan {
              try planned.authorize(identifier: planned.identifier,
                reducedAssurance: args.contains("--reduced-assurance"))
              config = planned.configuration
            }
          } else if !args.contains("--fixed-runway") {
            // Standalone fly uses the same automatic approach-end decision as
            // the operator/backend, never a hardcoded Runway 09 or 27.
            config = RunwayApproachSelector.choose(source, configuration: config)
          }
          if config.runway != originalRunway {
            await io.shutdown()
            io = KRPCFlightIO(connection: connection, configuration: config,
              airbrakeActionGroup: UInt32(option("--airbrake-group") ?? "1") ?? 1)
            _ = try await io.connect(expectedVesselName: option("--vessel") ?? "STS")
          }
          try config.validate()
          try encoder.encode(config).write(
            to: URL(fileURLWithPath: logPath + ".configuration.json"))
          let recorder = try JSONLTelemetryRecorder(url: URL(fileURLWithPath: logPath))
          let planKind = selectedDeorbitPlan?.executionQualified == true
            ? "qualified nominal orbit-to-rollout continuation"
            : "physically screened orbital entry; terminal/rollout unqualified, continue live guidance"
          FileHandle.standardError.write(Data(
            "ksl-landing: automatic approach \(config.runway.name), heading \(degrees(config.runway.heading)) deg; \(planKind)\n".utf8))
          // Orbital execution owns the same IO and recorder through entry and
          // rollout. Keep its existing qualification and fresh-orbit checks;
          // ordinary `fly` remains an entry replay for backwards compatibility.
          let task: Task<RuntimeOutcome, Error>
          if args.contains("--deorbit") {
            guard let plan = selectedDeorbitPlan else {
              throw FlightError.invalidConfiguration("Deorbit selection returned no selected plan")
            }
            let recordedPlanURL = URL(fileURLWithPath: logPath + ".deorbit-plan.json").standardizedFileURL
            if option("--deorbit-plan").map({ URL(fileURLWithPath: $0).standardizedFileURL }) != recordedPlanURL {
              try encoder.encode(plan).write(to: recordedPlanURL)
            }
            let reduced = args.contains("--reduced-assurance")
            try plan.authorize(identifier: plan.identifier, reducedAssurance: reduced)
            let runtime = DeorbitRuntime(io: io, recorder: recorder, plan: plan,
              aerodynamics: aero, reducedAssurance: reduced,
              maximumOutage: Double(option("--max-outage") ?? "20") ?? 20)
            task = Task { try await runtime.run(maximumDuration: Double(option("--duration") ?? "36000") ?? 36000) }
          } else {
            guard option("--deorbit-plan") == nil else {
              throw FlightError.invalidConfiguration("--deorbit-plan requires --deorbit")
            }
            try await io.arm()
            let runtime = try FlightRuntime(
              io: io, recorder: recorder, configuration: config,
              maximumOutage: Double(option("--max-outage") ?? "20") ?? 20, aerodynamics: aero)
            task = Task { try await runtime.run(maximumDuration: Double(option("--duration") ?? "2400") ?? 2400) }
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
                   [--spiral-only]
          scenario [--entry] [--output scenario.json]
          plan [--entry | --scenario file] [--spiral-only] [--output plan.json]
          Initial-state options: --along meters --cross meters --height meters
             --speed m/s --flight-path degrees --heading-error degrees --mass kg
          configuration [--output configuration.json]
          deorbit-screen --input state.json --configuration configuration.json [--output screen.json]
          deorbit-trace --input plan.json --log offline-model.jsonl
          taem-response-audit --input flight.jsonl --configuration config.json
          replay --input flight.jsonl [--configuration config.json] [--verify-commands]
          legacy-audit --input archive.jsonl [--skip-malformed] [--session id] [--states snapshots.jsonl]
          import-legacy --input archive.jsonl [--skip-malformed] [--session id]
                        [--at-time UT] --output historical-scenario.json
          inspect --host 127.0.0.1 --port 50000 --vessel STS
          fly --host 127.0.0.1 --port 50000 --vessel STS [--log live.jsonl]
              [--configuration configuration.json] [--airbrake-group 1] [--max-outage 20] [--raw-axes]
              [--deorbit [--reduced-assurance] [--deorbit-plan plan.json]]
        inspect is read-only. Without --deorbit, fly starts in entry, not orbit.
        With --deorbit, fly computes or loads and records a plan; freshness and configuration are checked before arming.
        All simulation output is labeled offline-model-only. No ascent commands exist.
        A full-rate log and its exact configuration are required for command-identical replay.
        """)
    }
  }
}
