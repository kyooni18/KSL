import Foundation
import KSLCore
import KSLDiagnostics
import KSLGuidance
import KSLPhysics
import KSLRuntime

public struct SimulationResult: Codable, Sendable {
  public let scenario: String
  public let evidence: String
  public let outcome: String
  public let elapsed: Double
  public let steps: Int
  public let phases: [String]
  public let touchdown: TouchdownEvent?
  public let finalState: VehicleState
  public let maximumDynamicPressure: Double
  public let maximumLoadFactor: Double
  public let saturatedControlFraction: Double
  public let violations: [ConstraintViolation]
  public let constraintOccurrences: [String: Int]
  public let terminalPlan: TerminalPlan?
  public let lastPlanningAdvisories: [String: Int]
  public let lastTerminalAdvisory: TerminalAssessment?
  public let lastTerminalCandidateCount: Int
  public let maximumComputationMilliseconds: Double
  public let meanComputationMilliseconds: Double
}

public final class SimulationEngine {
  public let scenario: SimulationScenario
  private let physics: ShuttlePhysicsEngine
  private let computer: FlightComputer
  public init(scenario: SimulationScenario) throws {
    guard scenario.maximumDuration > 0, scenario.maximumDuration.isFinite
    else { throw FlightError.invalidConfiguration("Invalid simulation duration") }
    self.scenario = scenario
    let c = scenario.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
  let aerodynamics = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea,
    speedbrakeFullDeploymentDragArea: c.vehicle.speedbrakeFullDeploymentDragArea)
    // The plant and FCS use the same actual configured vehicle model.
    computer = try .init(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    physics = .init(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func run(logURL: URL? = nil, logEvery: Int = 1, physicsStep: Double = 0.02) throws
    -> SimulationResult
  {
    let writer = try logURL.map { try JSONLFileWriter(url: $0) }
    defer { try? writer?.close() }
    let c = scenario.configuration
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    var state = physics.observe(scenario.initialState)
    var steps = 0
    var phases: [String] = []
    var maxQ = 0.0
    var maxLoad = 0.0
    var saturated = 0
    var violations: [ConstraintViolation] = []
    var outcome = "timedOut"
    var occurrences: [String: Int] = [:]
    var totalCompute = 0.0
    var maxCompute = 0.0
    var computations = 0
    while state.time - scenario.initialState.time < scenario.maximumDuration {
      let started = ContinuousClock.now
      let record = try computer.step(state)
      let timing = started.duration(to: .now).components
      let milliseconds = Double(timing.seconds) * 1000 + Double(timing.attoseconds) / 1e15
      totalCompute += milliseconds
      maxCompute = max(maxCompute, milliseconds)
      computations += 1
      for violation in record.guidance.violations {
        occurrences[violation.kind.rawValue, default: 0] += 1
      }
      if phases.last != record.guidance.phase.rawValue {
        phases.append(record.guidance.phase.rawValue)
      }
      if steps % max(1, logEvery) == 0 || record.guidance.phase == .complete
        || record.guidance.phase == .abort
      {
        try writer?.write(record)
      }
      let diagnostics = computer.controller.diagnostics
      if diagnostics.pitchSaturated || diagnostics.rollSaturated || diagnostics.yawSaturated {
        saturated += 1
      }
      maxQ = max(maxQ, state.aerodynamics.dynamicPressure)
      maxLoad = max(maxLoad, abs(state.aerodynamics.lift) / (state.mass * 9.80665))
      if record.guidance.phase == .complete {
        outcome = "complete"
        break
      }
      if record.guidance.phase == .abort {
        outcome = "aborted"
        violations = record.guidance.violations
        break
      }
      state = try physics.advance(
        state, command: record.command, duration: c.guidance.controlInterval,
        maximumStep: physicsStep)
      steps += 1
      if state.contact == .crashed {
        outcome = "crashed"
        violations = [.init(.contactQuality, "Invalid ground contact")]
        break
      }
      if state.aerodynamics.dynamicPressure > c.vehicle.maximumDynamicPressure * 1.1 {
        outcome = "envelopeViolation"
        violations = [
          .init(
            .excessiveDynamicPressure, "Simulation exceeded q limit",
            value: state.aerodynamics.dynamicPressure, limit: c.vehicle.maximumDynamicPressure)
        ]
        break
      }
      if state.position.length < c.planet.radius {
        outcome = "crashed"
        break
      }
      if frame.coordinates(of: state.position).height > 140_000 {
        outcome = "escapedEntry"
        break
      }
    }
    try writer?.close()
    return .init(
      scenario: scenario.name, evidence: "offline-model-only", outcome: outcome,
      elapsed: state.time - scenario.initialState.time, steps: steps,
      phases: phases, touchdown: physics.touchdown, finalState: state, maximumDynamicPressure: maxQ,
      maximumLoadFactor: maxLoad,
      saturatedControlFraction: Double(saturated) / Double(max(1, steps)), violations: violations,
      constraintOccurrences: occurrences,
      terminalPlan: computer.mission.plan,
      lastPlanningAdvisories: computer.mission.lastPlanningResult?.advisoryCounts ?? [:],
      lastTerminalAdvisory: computer.mission.lastPlanningResult?.bestAdvisory,
      lastTerminalCandidateCount: computer.mission.lastPlanningResult?.evaluatedCandidates ?? 0,
      maximumComputationMilliseconds: maxCompute,
      meanComputationMilliseconds: totalCompute / Double(max(1, computations)))
  }
}
