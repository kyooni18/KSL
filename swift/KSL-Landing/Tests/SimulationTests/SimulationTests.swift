import KSLCore
import KSLDiagnostics
import KSLRuntime
import Testing
import XCTest

@testable import KSLSimulation

final class SimulationTests: XCTestCase {
  func testDeterministicPlantAndGuidance() throws {
    var scenario = SimulationScenario.approach()
    scenario.maximumDuration = 5
    let a = try SimulationEngine(scenario: scenario).run()
    let b = try SimulationEngine(scenario: scenario).run()
    XCTAssertEqual(a.finalState, b.finalState)
    XCTAssertEqual(a.phases, b.phases)
    XCTAssertEqual(a.evidence, "offline-model-only")
  }
}

struct RuntimeLifecycleTests {
  @Test func testCancellationClosesFlightIO() async throws {
    let scenario = SimulationScenario.approach()
    let io = try SimulatedFlightIO(scenario: scenario)
    let runtime = try FlightRuntime(
      io: io, recorder: NullTelemetryRecorder(), configuration: scenario.configuration)
    let task = Task { try await runtime.run(realTime: true) }
    try await Task.sleep(for: .milliseconds(30))
    task.cancel()
    let result = try await task.value
    let closed = await io.closed
    #expect(result == .cancelled)
    #expect(closed)
  }
  @Test func testCommandSequenceRejected() async throws {
    let scenario = SimulationScenario.approach()
    let io = try SimulatedFlightIO(scenario: scenario)
    let state = try await io.readState()
    try await io.apply(.init(sequence: 1, stateTime: state.time, command: .init()))
    do {
      try await io.apply(.init(sequence: 1, stateTime: state.time, command: .init()))
      Issue.record("Duplicate accepted")
    } catch {}
    await io.shutdown()
  }
}
