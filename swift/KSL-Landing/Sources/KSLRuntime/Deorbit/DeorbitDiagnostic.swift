import Foundation
import KSLCore
import KSLControl
import KSLPhysics
import KSLGuidance

/// Offline diagnostic of the planner's nominal model, not flight authorization.
public enum DeorbitDiagnostic {
  /// Evaluate the nominal full native route with actual configured physics.
  /// This is offline model
  /// evidence, not an authorization to fly or an independent validation.
  public static func evaluateWindow(plan: DeorbitPlan, caseIndex: Int,
                                    entryReplanning: Bool = true) async throws -> DeorbitStressCase {
    let c = plan.configuration
    guard caseIndex == 0 else {
      throw FlightError.invalidConfiguration("Invalid deorbit diagnostic case index")
    }
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    return try await DeorbitPlanner.evaluate(configuration: c, aero: aero, source: plan.source,
      window: plan.window, propulsion: plan.propulsion,
      index: caseIndex, candidateIndex: 0, progress: { _ in }, entryReplanning: entryReplanning)
  }
  public static func traceNominal(plan: DeorbitPlan, output: URL, entryReplanning: Bool = false) async throws {
    try await trace(plan: plan, caseIndex: 0, output: output, entryReplanning: entryReplanning)
  }
  /// Record the nominal orbital state, atmosphere and aerodynamic plant.
  /// This is diagnostic evidence only, not a route admission or live flight.
  public static func trace(plan: DeorbitPlan, caseIndex: Int, output: URL,
                           entryReplanning: Bool = false) async throws {
    let c = plan.configuration
    guard caseIndex == 0 else {
      throw FlightError.invalidConfiguration("Invalid deorbit diagnostic case index")
    }
    let orbital = try OrbitalExecutionModel(configuration: c).predict(source: plan.source,
      window: plan.window, propulsion: plan.propulsion)
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let computer = try FlightComputer(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, entryReplanning: entryReplanning)
    let plant = ShuttlePhysicsEngine(configuration: c,
      atmosphere: try .init(planet: c.planet), aerodynamics: aero,
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
