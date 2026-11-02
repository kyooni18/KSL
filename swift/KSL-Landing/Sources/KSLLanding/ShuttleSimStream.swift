import Foundation
import KSLCore
import KSLControl
import KSLPhysics
import KSLGuidance
import KSLRuntime
import KSLSimulation

/// Opt-in NDJSON adapter. No changes to the operator backend protocol or to
/// ShuttleSim. Every atmospheric command comes from the actual FlightComputer.
enum ShuttleSimStream {
  static func run(arguments: [String]) async throws {
    func option(_ name: String) -> String? {
      guard let i = arguments.firstIndex(of: name), i+1 < arguments.count else { return nil }
      return arguments[i+1]
    }
    guard let modelPath = option("--attitude-model") else {
      throw FlightError.invalidConfiguration("shuttlesim-controller requires --attitude-model matching the independent plant")
    }
    var c = MissionConfiguration(); c.runway.elevation = 70
    if let path = option("--configuration") {
      c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: path)))
    }
    try c.validate()
    let decoder = try ShuttleSimTelemetryDecoder(configuration: c,
      attitudeINI: String(contentsOfFile: modelPath, encoding: .utf8))
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let flight = try FlightComputer(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let orbit = OrbitalMechanics(planet: c.planet)
    let orbitExperiment = arguments.contains("--unqualified-orbit-experiment")
    let windowIndex = Int(option("--window-index") ?? "0") ?? 0
    let isp = Double(option("--engine-isp") ?? "285") ?? 285
    var window: DeorbitWindow?, started: Double?, burned = false, inEntry = false
    var metadata: [String: Any]?, previousOrbit: VehicleState?
    var lowQSteps = 0
    var impulse = RetrogradeImpulseObserver(planet: c.planet)
    func object<T: Encodable>(_ value: T) throws -> Any { try JSONSerialization.jsonObject(with: JSONEncoder().encode(value)) }
    while let line = readLine() {
      if line.isEmpty { continue }
      let sample = try decoder.decode(Data(line.utf8))
      let state = sample.state, height = state.position.length-c.planet.radius
      var command: [String: Any] = ["step": true, "throttle": 0.0, "pitch_input": 0.0,
        "roll_input": 0.0, "yaw_input": 0.0, "gear_down": state.systems.gearDown,
        "brakes": false, "wheel_steering": 0.0, "airbrakes": false, "ut": state.time]
      if sample.lowPressureControlUnsupported { lowQSteps += 1 }
      command["bodyRateEvidence"] = sample.bodyRateEvidence.rawValue
      command["geometryErrorMeters"] = sample.geometryError
      let chart = RunwayFrame(runway: c.runway, planet: c.planet).coordinates(of: state.position)
      command["nativeAzimuthalChart"] = ["along": chart.along, "cross": chart.cross]
      command["lowPressureControlUnsupported"] = sample.lowPressureControlUnsupported
      command["lowPressureSteps"] = lowQSteps
      command["evidence"] = "independent-ShuttleSim-physics; offline-unqualified"
      var phase = "orbitalCoast"
      if height > c.planet.atmosphereTop {
        try impulse.observe(.init(fixed: state, planet: c.planet), thrustReported: state.systems.thrust > 0)
      }
      if height > c.planet.atmosphereTop-500 && !inEntry {
        guard orbitExperiment else {
          throw FlightError.infeasible("ShuttleSim lacks native vacuum torque/CoM telemetry; orbital experiments require explicit --unqualified-orbit-experiment")
        }
        if window == nil {
          guard let input = try JSONSerialization.jsonObject(with: Data(line.utf8)) as? [String: Any],
            let vehicle = input["vehicle"] as? [String: Any], let thrust = vehicle["available_thrust_n"] as? Double else {
            throw FlightError.invalidTelemetry("No independent engine thrust telemetry")
          }
          // The simulator engine has no dry-mass field or depletion boundary.
          // Dry mass is an explicit experiment input, never a measurement.
          guard let dry = option("--dry-mass"), let dryMass = Double(dry) else {
            throw FlightError.invalidConfiguration("Orbital experiment requires explicit --dry-mass")
          }
          let prop = PropulsionTelemetry(availableThrust: thrust, dryMass: dryMass, specificImpulse: isp, evidence: .provisional)
          // This experimental adapter executes a legacy fixed-angle servo,
          // not the native two-component burn controller. Do not misexecute
          // a normal-first candidate as one constant-angle deorbit burn.
          let result = try DeorbitWindowSearch(configuration: c).search(from: state,
            propulsion: prop, includeNormalCorrections: false)
          metadata = ["candidates": try object(result.candidates), "executionQualified": false,
            "limitation": "orbital thrust-axis servo is ShuttleSim-owned; no native vacuum actuator qualification"]
          if let legacy = option("--legacy-window").flatMap(Int.init) {
            // Deliberate envelope test: fly a raw orbital-geometry candidate
            // without landing-aware screening, but still report its forecast.
            guard result.candidates.indices.contains(legacy) else { throw FlightError.infeasible("Requested legacy candidate index outside search result") }
            window = result.candidates[legacy]
            let selector = LandingAwareWindowSelector(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
            let forecast = EntryCaptureForecaster(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
              .forecast(from: selector.entryState(window!))
            metadata!["selection"] = ["mode": "legacy-orbital-geometry", "legacyIndex": legacy,
              "landingForecast": try object(forecast)]
            FileHandle.standardError.write(Data("legacy window \(legacy): \(forecast.capture.summary)\n".utf8))
          } else {
            let clock = Date()
            let selection = try LandingAwareWindowSelector(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
              .select(result, from: state, propulsion: prop)
            for r in selection.survey {
              let e = r.nominal.capture.envelope
              FileHandle.standardError.write(Data(String(format: "survey pass %d offset %+.0f EI %.0fkm -> TAEM h=%.0f M=%.2f along=%.1fkm cross=%.1fkm %@ excess=%.0f deficit=%.0f %@\n",
                r.pass, r.burnOffset, r.window.entryRange/1000, r.nominal.terminalAltitude, r.nominal.terminalMach,
                r.nominal.terminalAlong/1000, r.nominal.terminalCross/1000, r.nominal.capture.summary,
                max(-1e9, e.excessEnergyMargin), max(-1e9, e.deficitEnergyMargin),
                e.dominantViolations.description + " " + (r.nominal.capture.violations.first?.message ?? "")).utf8))
            }
            for (i, r) in selection.ranked.enumerated() {
              FileHandle.standardError.write(Data(("rank \(i) pass \(r.pass) offset \(r.burnOffset) burn \(Int(r.window.burnStart)) "
                + "EI range \(Int(r.window.entryRange/1000))km aim \(Int(r.nominal.crossAim/1000))km \(r.nominal.capture.summary) "
                + "corridor \(r.corridor ?? [])\n").utf8))
            }
            FileHandle.standardError.write(Data("landing-aware selection: \(selection.evaluatedForecasts) forecasts, \(Int(Date().timeIntervalSince(clock)))s\n".utf8))
            guard selection.ranked.indices.contains(windowIndex) else { throw FlightError.infeasible("Requested landing-aware rank outside selection") }
            let chosen = selection.ranked[windowIndex]
            if !chosen.nominal.feasible {
              FileHandle.standardError.write(Data("Advisory: flying best-ranked unqualified window: \(chosen.rejection ?? "")\n".utf8))
            }
            window = chosen.window
            flight.mission.planEntryAim(chosen.nominal.crossAim, objective: chosen.nominal.objective)
            metadata!["selection"] = ["mode": "landing-aware", "rank": windowIndex,
              "ranked": try object(selection.ranked),
              "survey": try object(selection.survey), "evaluatedForecasts": selection.evaluatedForecasts]
          }
          metadata!["window"] = try object(window!)
        }
        let w = window!
        if !burned && state.time >= w.burnStart {
          let periapsis = try orbit.elements(.init(fixed: state, planet: c.planet)).periapsisAltitude
          if periapsis <= w.periapsisAltitude+300 || impulse.identifiedDeltaV >= w.deltaV-BurnThrottleLaw.terminalDeltaVTolerance {
            burned = true
            command["burnResult"] = ["ut": state.time, "plannedDeltaV": w.deltaV,
              "actualDeltaV": sample.deliveredDeltaV, "periapsis": periapsis,
              "identifiedRetrogradeDeltaV": impulse.identifiedDeltaV,
              "targetConvention": "retrograde-component", "actualDeltaVConvention": "total-thrust-integral"]
          } else {
            if started == nil { started = state.time }
            let raw = try JSONSerialization.jsonObject(with: Data(line.utf8)) as! [String: Any]
            let vehicle = raw["vehicle"] as! [String: Any]
            let thrust = (vehicle["available_thrust_n"] as! NSNumber).doubleValue
            let fraction = BurnThrottleLaw.fraction(elapsed: state.time-started!,
              remaining: max(0,w.deltaV-impulse.identifiedDeltaV),
              maximumAcceleration: c.deorbitSettings.maximumThrottle*thrust/state.mass,
              ramp: c.deorbitSettings.throttleRampDuration)
            let os = OrbitalState(fixed: state, planet: c.planet)
            let direction = OrbitalMechanics.burnDirection(position: os.position, velocity: os.velocity,
              normalAngle: w.normalAngle).rotated(about: .init(0,0,1), angle: sample.initialInertialPhase)
            command["throttle"] = c.deorbitSettings.maximumThrottle*fraction
            command["thrust_dir"] = [direction.x,direction.y,direction.z]; phase = "deorbitBurn"
          }
        }
        previousOrbit = state
      } else {
        if !inEntry {
          if let w = window {
            guard burned else { throw FlightError.infeasible("Atmosphere reached before experimental burn shutdown") }
            let predicted = w.entryInterface.planetFixed(planet: c.planet, attitude: .init())
            command["entryError"] = ["time": state.time-predicted.time,
              "positionMeters": (state.position-predicted.position).length,
              "speed": state.velocity.length-predicted.velocity.length,
              "previousAltitude": (previousOrbit?.position.length ?? state.position.length)-c.planet.radius]
          }
          inEntry = true
        }
        let record = try flight.step(state)
        command["pitch_input"] = record.command.pitch; command["roll_input"] = record.command.roll
        command["yaw_input"] = record.command.yaw; command["gear_down"] = record.command.gearDown
        command["brakes"] = record.command.wheelBrake > 0.5; command["wheel_steering"] = record.command.wheelSteering
        command["record"] = try object(record)
        phase = record.guidance.phase.rawValue
        command["terminal"] = phase == "abort" || phase == "complete"
      }
      command["phase"] = phase
      if let value = metadata { command["plan"] = value; metadata = nil }
      var encoded = try JSONSerialization.data(withJSONObject: command, options: [.sortedKeys])
      encoded.append(10); FileHandle.standardOutput.write(encoded)
      if command["terminal"] as? Bool == true { break }
    }
  }
}
