import Foundation
import KSLControl
import KSLCore
import KSLDiagnostics
import KSLPhysics

/// Held-out measured-command replay over a short interval. The moment bias is
/// formed only from samples at or before the seed; no future observation is fit.
enum PitchBalanceProbe {
  struct Window: Codable {
    let time: Double, duration: Double, mach: Double, pressure: Double
    let alpha: Double, input: Double, localBalance: Double, legacyBalance: Double?
    let measuredAlpha: Double, legacyAlpha: Double, correctedAlpha: Double
    let legacyPitchRateError: Double, correctedPitchRateError: Double
  }
  struct Report: Codable {
    let evidence: String
    let intervalSeconds: Double
    let limitations: [String]
    let windows: [Window]
  }
  static func run(input: String, configuration c: MissionConfiguration) async throws {
    let reader = try JSONLReader(url: URL(fileURLWithPath: input))
    defer { try? reader.close() }
    var records: [FlightRecord] = []
    let decoder = JSONDecoder()
    while let data = try reader.nextLine() { records.append(try decoder.decode(FlightRecord.self, from: data)) }
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let controller = FlightController(configuration: c)
    var windows: [Window] = [], nextWindow = -Double.infinity
    guard records.count > 2 else { throw FlightError.invalidConfiguration("Insufficient measured records") }
    for i in 1..<(records.count - 1) {
      let record = records[i], state = record.state, a = state.aerodynamics
      let dt = state.time - records[i - 1].state.time
      guard dt > 0, dt <= 1 else { controller.resumeAfterTelemetryGap(at: state); continue }
      _ = try controller.command(state: state, solution: record.guidance, dt: dt)
      guard (3...5).contains(a.mach), a.dynamicPressure > 200, !state.systems.rcs,
        state.time >= nextWindow,
        let bias = controller.snapshot.predictionPitchTrim(state: state) else { continue }
      var end = i + 1
      while end < records.count && records[end].state.time - state.time < 1 { end += 1 }
      guard end < records.count,
        (i..<end).allSatisfy({ records[$0 + 1].state.time > records[$0].state.time
          && records[$0 + 1].state.time - records[$0].state.time <= 1 }) else { continue }
      let oldBias = controller.snapshot.predictionPitchTrim(alpha: a.angleOfAttack)
      func replay(balance: Double?) throws -> VehicleState {
        let plant = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero,
          initialControls: state.appliedControls ?? record.command, initialState: state,
          pitchBalancingInput: balance)
        var predicted = plant.observe(state)
        for k in i..<end {
          predicted = try plant.advance(predicted, command: records[k].command,
            duration: records[k + 1].state.time - records[k].state.time, maximumStep: 0.02)
        }
        return predicted
      }
      let legacy = try replay(balance: oldBias), corrected = try replay(balance: bias)
      let observed = records[end].state
      windows.append(.init(time: state.time, duration: observed.time - state.time,
        mach: a.mach, pressure: a.dynamicPressure, alpha: degrees(a.angleOfAttack),
        input: state.appliedControls?.pitch ?? record.command.pitch,
        localBalance: bias, legacyBalance: oldBias,
        measuredAlpha: degrees(observed.aerodynamics.angleOfAttack),
        legacyAlpha: degrees(legacy.aerodynamics.angleOfAttack),
        correctedAlpha: degrees(corrected.aerodynamics.angleOfAttack),
        legacyPitchRateError: legacy.angularVelocity.y - observed.angularVelocity.y,
        correctedPitchRateError: corrected.angularVelocity.y - observed.angularVelocity.y))
      nextWindow = observed.time
    }
    let report = Report(evidence: "historical-live-state-held-out-recorded-command-replay",
      intervalSeconds: 1, limitations: [
        "One flight and the observed Mach 3-5 regime only; no independent aerodynamic moment slope is identified.",
        "Local balancing input and seed torque are held over each short replay; their evolution across q/Mach is unvalidated.",
        "Applied control telemetry is not a measurement of individual surface deflection.",
        "This is not closed-loop route qualification, asynchronous timing validation, or a live landing."
      ], windows: windows)
    let encoder = JSONEncoder(); encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
    FileHandle.standardOutput.write(try encoder.encode(report)); FileHandle.standardOutput.write(Data([10]))
  }
}
