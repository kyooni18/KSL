import Foundation
import KSLCore
import KSLControl
import KSLGuidance
import KSLPhysics

public struct DeorbitPlanningProgress: Codable, Sendable {
  public let stage: String
  public let candidate: Int
  public let stressCase: Int
  public let totalCases: Int
}

/// Owns all expensive orbit-to-stop qualification outside the operator and
/// urgent flight-control actors. Plan values carry every failed stress case.
public actor DeorbitPlanner {
  private let configuration: MissionConfiguration
  private let aero: AerodynamicsEngine
  public init(configuration: MissionConfiguration, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; aero = aerodynamics
  }
  public func plan(from source: VehicleState,
    progress: @Sendable (DeorbitPlanningProgress) async -> Void = { _ in }) async throws -> DeorbitPlan {
    try source.validate(configuration: configuration)
    guard let propulsion = source.propulsion, propulsion.evidence == .measured else {
      throw FlightError.invalidTelemetry("Deorbit requires measured OMS propulsion telemetry")
    }
    try propulsion.validate(mass: source.mass)
    await progress(.init(stage: "orbital-window-search", candidate: 0, stressCase: 0, totalCases: 15))
    let search = try DeorbitWindowSearch(configuration: configuration).search(from: source, propulsion: propulsion)
    let dispersions = DeorbitDispersion.cases(configuration)
    var selected: DeorbitWindow?, selectedCases: [DeorbitStressCase] = [], bestScore = Double.infinity
    for (index, candidate) in search.candidates.enumerated() {
      try Task.checkCancellation()
      await progress(.init(stage: "native-orbit-to-rollout", candidate: index+1, stressCase: 0, totalCases: 15))
      let nominal = try evaluate(source: source, window: candidate, propulsion: propulsion, dispersion: dispersions[0], index: 0)
      var cases = [nominal]
      if nominal.passed {
        for i in 1..<dispersions.count {
          try Task.checkCancellation()
          await progress(.init(stage: "full-route-stress", candidate: index+1, stressCase: i, totalCases: 15))
          cases.append(try evaluate(source: source, window: candidate, propulsion: propulsion, dispersion: dispersions[i], index: i))
        }
      }
      let passes = cases.filter(\.passed).count, hardUnsafe = cases.filter(\.hardUnsafe).count
      let score = nominal.passed ? -Double(passes)*1e9+Double(hardUnsafe)*1e8+candidate.miss
        : (nominal.hardUnsafe ? 1e12 : 1e11)+Double(nominal.violations.count)*1e6+candidate.miss
      if score < bestScore { selected = candidate; selectedCases = cases; bestScore = score }
      if nominal.passed && hardUnsafe == 0 && Double(passes)/15 >= configuration.deorbitSettings.minimumRobustPassFraction { break }
    }
    guard let window = selected else { throw FlightError.infeasible("No candidate completed native trajectory evaluation") }
    // Even a failed nominal result keeps an honest 15-case regression report;
    // failed cases are executed, never filled with fabricated passes or misses.
    if selectedCases.count == 1 {
      for i in 1..<dispersions.count {
        try Task.checkCancellation()
        await progress(.init(stage: "failed-plan-stress-report", candidate: 0, stressCase: i, totalCases: 15))
        selectedCases.append(try evaluate(source: source, window: window, propulsion: propulsion, dispersion: dispersions[i], index: i))
      }
    }
    return .init(source: source, configuration: configuration, propulsion: propulsion, window: window,
      cases: selectedCases, modelLimitations: [
        "The inherited STS-N subsonic pitch fit does not validate STS entry moments.",
        "Roll/yaw restoring and damping coefficients lack an accepted STS high-q identification.",
        "Ground gear/tyre dynamics are provisional; no native KSP rollout qualification has been recorded.",
        "Offline planner scheduling assumes completion at the requested simulation epoch."
      ], evaluatedWindows: search.evaluated)
  }
  private func evaluate(source: VehicleState, window: DeorbitWindow, propulsion: PropulsionTelemetry,
                        dispersion: DeorbitDispersion, index: Int) throws -> DeorbitStressCase {
    let c = configuration, frame = RunwayFrame(runway: c.runway, planet: c.planet)
    var burned = false, entryAngle: Double?, maxQ = 0.0, maxLoad = 0.0
    var event: TouchdownEvent?, stoppedAlong: Double?, failures: [ConstraintViolation] = []
    var passed = false
    do {
      let orbital = try OrbitalExecutionModel(configuration: c).predict(source: source,
        window: window, propulsion: propulsion, dispersion: dispersion)
      burned = orbital.burnCompleted; entryAngle = orbital.entry.flightPathAngle
      if orbital.entry.flightPathAngle < c.deorbitSettings.minimumEntryFlightPath {
        failures.append(.init(.invalidGeometry, "Descending entry is steeper than the configured hard limit",
          value: orbital.entry.flightPathAngle, limit: c.deorbitSettings.minimumEntryFlightPath))
      } else {
        let atmosphere = try AtmosphericModelEngine(planet: c.planet)
        let computer = try FlightComputer(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
        let plant = ShuttlePhysicsEngine(configuration: c,
          atmosphere: try .init(planet: c.planet, densityScale: dispersion.atmosphere),
          aerodynamics: aero.scaled(lift: dispersion.lift, drag: dispersion.drag),
          initialControls: orbital.entry.appliedControls ?? .init())
        var state = plant.observe(orbital.entry)
        while state.time-orbital.entry.time < c.guidance.maximumPredictionDuration {
          try Task.checkCancellation()
          let record = try computer.step(state)
          maxQ = max(maxQ, state.aerodynamics.dynamicPressure)
          maxLoad = max(maxLoad, abs(state.aerodynamics.lift)/(state.mass*9.80665))
          if record.guidance.phase == .complete {
            passed = plant.touchdown?.acceptable == true
            stoppedAlong = frame.coordinates(of: state.position).along; break
          }
          if record.guidance.phase == .abort { failures += record.guidance.violations; break }
          state = try plant.advance(state, command: record.command, duration: c.guidance.controlInterval, maximumStep: 0.02)
          if state.contact == .crashed { failures.append(.init(.contactQuality, "Full native route failed ground contact or runway containment")); break }
        }
        event = plant.touchdown
        if !passed && failures.isEmpty { failures.append(.init(.timeout, "Native trajectory did not complete touchdown and rollout")) }
      }
    } catch is CancellationError { throw CancellationError() }
    catch { failures.append(.init(burned ? .actuatorAuthority : .invalidGeometry, String(describing: error))) }
    let hardUnsafe = !burned || entryAngle == nil || (entryAngle ?? 0) < c.deorbitSettings.minimumEntryFlightPath
      || maxQ > c.vehicle.maximumDynamicPressure || maxLoad > c.vehicle.maximumLoadFactor
    return .init(index: index, passed: passed, hardUnsafe: hardUnsafe, burnCompleted: burned,
      entryFlightPath: entryAngle, maximumDynamicPressure: maxQ, maximumLoad: maxLoad,
      touchdownAlong: event?.along, touchdownCross: event?.cross, touchdownSink: event?.verticalSpeed,
      touchdownPitch: event?.pitch, stoppedAlong: stoppedAlong, violations: failures)
  }
}
