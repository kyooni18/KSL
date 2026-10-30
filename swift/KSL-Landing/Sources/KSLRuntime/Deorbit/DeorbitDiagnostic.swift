import Foundation
import KSLCore
import KSLControl
import KSLPhysics
import KSLGuidance

/// Offline diagnostic of the planner's nominal model, not flight authorization.
public enum DeorbitDiagnostic {
  public static func traceNominal(plan: DeorbitPlan, output: URL) async throws {
    let c = plan.configuration, dispersion = DeorbitDispersion.cases(c)[0]
    let orbital = try OrbitalExecutionModel(configuration: c).predict(source: plan.source,
      window: plan.window, propulsion: plan.propulsion, dispersion: dispersion)
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let computer = try FlightComputer(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let plant = ShuttlePhysicsEngine(configuration: c,
      atmosphere: try .init(planet: c.planet, densityScale: dispersion.atmosphere),
      aerodynamics: aero.scaled(lift: dispersion.lift, drag: dispersion.drag),
      initialControls: orbital.entry.appliedControls ?? .init())
    guard !FileManager.default.fileExists(atPath: output.path),
      FileManager.default.createFile(atPath: output.path, contents: nil) else {
      throw FlightError.invalidConfiguration("Cannot create diagnostic log")
    }
    let file = try FileHandle(forWritingTo: output)
    defer { try? file.close() }
    let encoder = JSONEncoder()
    var state = plant.observe(orbital.entry), nextRecord = state.time
    while state.time-orbital.entry.time < c.guidance.maximumPredictionDuration {
      try Task.checkCancellation()
      let record = try computer.step(state)
      if state.time >= nextRecord {
        try file.write(contentsOf: encoder.encode(record))
        try file.write(contentsOf: Data([10]))
        nextRecord += 1
      }
      if state.contact == .crashed || record.guidance.phase == .complete || record.guidance.phase == .abort { break }
      state = try plant.advance(state, command: record.command,
        duration: c.guidance.controlInterval, maximumStep: 0.02)
      if state.contact == .crashed { break }
    }
    let p = RunwayFrame(runway: c.runway, planet: c.planet).coordinates(of: state.position)
    let summary = "offline-model-only: UT=\(state.time) contact=\(state.contact.rawValue) ASL=\(state.position.length-c.planet.radius) along=\(p.along) cross=\(p.cross) airspeed=\(state.aerodynamics.airspeed) touchdown=\(String(describing: plant.touchdown))\n"
    FileHandle.standardError.write(Data(summary.utf8))
  }
}
