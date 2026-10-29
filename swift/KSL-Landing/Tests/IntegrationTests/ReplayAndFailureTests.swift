import Foundation
import KSLCore
import KSLDiagnostics
import KSLPhysics
import KSLRuntime
import KSLSimulation
import Testing

private actor FailureFixture: FlightIO {
  enum Failure { case read, command, frozen, invalid }
  let failure: Failure
  let state: VehicleState
  private(set) var closed = false
  private(set) var commands = 0
  init(_ failure: Failure) throws {
    self.failure = failure
    let scenario = SimulationScenario.approach()
    let physics = ShuttlePhysicsEngine(
      configuration: scenario.configuration, atmosphere: try .init(), aerodynamics: try .init())
    var state = physics.observe(scenario.initialState)
    if failure == .invalid { state.aerodynamics.lift = .nan }
    self.state = state
  }
  func readState() throws -> VehicleState {
    if failure == .read { throw FlightError.disconnected("Injected read failure") }
    return state
  }
  func apply(_ command: CommandEnvelope) throws {
    commands += 1
    if failure == .command { throw FlightError.disconnected("Injected write failure") }
  }
  func shutdown() { closed = true }
}

private actor RecorderFixture: TelemetryRecording {
  private(set) var closed = false
  private(set) var count = 0
  func record(_ record: FlightRecord) { count += 1 }
  func close() { closed = true }
}

struct ReplayAndFailureTests {
  @Test func runtimeClosesBothOwnersOnReadAndCommandFailure() async throws {
    for failure in [FailureFixture.Failure.read, .command, .invalid] {
      let io = try FailureFixture(failure)
      let recorder = RecorderFixture()
      let runtime = try FlightRuntime(io: io, recorder: recorder, configuration: .init())
      do {
        _ = try await runtime.run(realTime: false)
        Issue.record("Failure was swallowed")
      } catch {}
      #expect(await io.closed)
      #expect(await recorder.closed)
      if failure == .invalid { #expect(await io.commands == 0) }
      if failure == .command { #expect(await io.commands == 1) }
    }
  }

  @Test func frozenTelemetryStopsWithoutRepeatedCommands() async throws {
    let io = try FailureFixture(.frozen)
    let recorder = RecorderFixture()
    var c = MissionConfiguration()
    c.guidance.maximumTelemetryAge = 0.08
    let runtime = try FlightRuntime(io: io, recorder: recorder, configuration: c)
    do {
      _ = try await runtime.run(realTime: false)
      Issue.record("Frozen telemetry accepted")
    } catch {}
    #expect(await io.commands == 1)
    #expect(await io.closed)
    #expect(await recorder.closed)
  }

  @Test func nativeJSONLReplayReproducesCommands() throws {
    let path = FileManager.default.temporaryDirectory.appendingPathComponent(
      UUID().uuidString + ".jsonl")
    defer { try? FileManager.default.removeItem(at: path) }
    var scenario = SimulationScenario.approach()
    scenario.maximumDuration = 8
    _ = try SimulationEngine(scenario: scenario).run(logURL: path)
    let reader = try FlightReplayReader(url: path)
    defer { try? reader.close() }
    let c = scenario.configuration
    let computer = try FlightComputer(
      configuration: c, atmosphere: .init(planet: c.planet),
      aerodynamics: .init(referenceArea: c.vehicle.referenceArea))
    var count = 0
    while let recorded = try reader.next() {
      let replayed = try computer.step(recorded.state)
      #expect(replayed.command == recorded.command)
      #expect(replayed.guidance.target == recorded.guidance.target)
      #expect(replayed.guidance.phase == recorded.guidance.phase)
      count += 1
    }
    #expect(count >= 80)
  }

  @Test func oversizedAndCorruptLogsAreRejected() throws {
    let path = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
    defer { try? FileManager.default.removeItem(at: path) }
    try Data(repeating: 65, count: 5000).write(to: path)
    let reader = try JSONLReader(url: path, maximumLineBytes: 100)
    #expect(throws: (any Error).self) { try reader.nextLine() }
    try reader.close()
    try Data("not json\n".utf8).write(to: path)
    let legacy = try LegacyTelemetryReader(url: path)
    #expect(throws: (any Error).self) { try legacy.next() }
    try legacy.close()
    let skipping = try LegacyTelemetryReader(url: path, skipMalformed: true)
    #expect(try skipping.next() == nil)
    #expect(skipping.statistics.malformedLines == 1)
    try skipping.close()
  }

  @Test func recorderRejectsWritesAfterExplicitClose() throws {
    let path = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
    defer { try? FileManager.default.removeItem(at: path) }
    let writer = try JSONLFileWriter(url: path)
    try writer.write(["value": 1])
    try writer.close()
    try writer.close()
    #expect(throws: (any Error).self) { try writer.write(["value": 2]) }
  }
}
