import Foundation
import KSLCore
import KSLDiagnostics
import KSLPhysics
import KSLRuntime
import KSLSimulation
import Testing
@testable import KSLGuidance

private actor PlanningGate {
  private var open = false
  private var waiters: [CheckedContinuation<Void, Never>] = []
  private(set) var events: [String] = []
  func note(_ event: String) { events.append(event) }
  func wait() async {
    if open { return }
    await withCheckedContinuation { waiters.append($0) }
  }
  func release() {
    guard !open else { return }
    open = true; events.append("release")
    let pending = waiters; waiters = []
    for waiter in pending { waiter.resume() }
  }
}

private actor InjectedPlanner: MissionPlanningService {
  enum Behavior: Sendable { case immediate, held, cancellationAware }
  let behavior: Behavior
  let gate: PlanningGate
  private(set) var calls = 0
  private(set) var cancellations = 0
  private var firstCall: [CheckedContinuation<Void, Never>] = []
  init(_ behavior: Behavior, gate: PlanningGate) { self.behavior = behavior; self.gate = gate }
  func waitUntilStarted() async {
    if calls > 0 { return }
    await withCheckedContinuation { firstCall.append($0) }
  }
  func recordedPlan(_ request: MissionPlanningRequest) async throws -> RecordedPlanningProposal {
    calls += 1
    let waiting = firstCall; firstCall = []
    for waiter in waiting { waiter.resume() }
    if behavior == .held { await gate.wait() }
    if behavior == .cancellationAware {
      do { try await Task.sleep(for: .seconds(30)) }
      catch { cancellations += 1; throw error }
    }
    await gate.note("worker-return")
    // Return a late result even when cancellation was ignored. The runtime
    // must discard it, not publish it or grant terminal control ownership.
    let proposal = MissionPlanningProposal(time: request.time, entry: nil, terminal: nil,
      dynamicsRevision: request.dynamicsRevision)
    return .init(proposal: proposal, encoded: try JSONEncoder().encode(proposal))
  }
}

private actor PlanningIO: FlightIO {
  private var state: VehicleState
  private let gate: PlanningGate
  private let jumpOnRead: Int?
  private var reads = 0
  private(set) var commands = 0
  private(set) var unsafeCommands = 0
  init(gate: PlanningGate, jumpOnRead: Int? = nil) throws {
    self.gate = gate; self.jumpOnRead = jumpOnRead
    let scenario = SimulationScenario.entry(along: -60_000, speed: 700, height: 25_000)
    state = ShuttlePhysicsEngine(configuration: scenario.configuration,
      atmosphere: try .init(), aerodynamics: try .init()).observe(scenario.initialState)
  }
  func readState() async throws -> VehicleState {
    await Task.yield()
    reads += 1
    state.time += reads == jumpOnRead ? 2 : 0.1
    return state
  }
  func apply(_ envelope: CommandEnvelope) {
    commands += 1
    if envelope.command.rcs || abs(envelope.command.pitch) > 1 || abs(envelope.command.roll) > 1 {
      unsafeCommands += 1
    }
  }
  func shutdown() async { await gate.note("shutdown") }
}

private actor PlanningRecorder: TelemetryRecording {
  private(set) var records = 0
  private(set) var closed = false
  func record(_ record: FlightRecord) { records += 1 }
  func close() { closed = true }
}

struct RuntimePlanningCancellationTests {
  @Test func shutdownPrecedesJoinOfUncooperativePlanner() async throws {
    let gate = PlanningGate(), planner = InjectedPlanner(.held, gate: gate)
    let io = try PlanningIO(gate: gate), recorder = PlanningRecorder()
    let runtime = try FlightRuntime(io: io, recorder: recorder, configuration: .init(),
      entryPlanningService: InjectedPlanner(.immediate, gate: gate), terminalPlanningService: planner)
    let flight = Task { try await runtime.run(maximumDuration: 100, realTime: true) }
    await planner.waitUntilStarted()
    flight.cancel()
    // This release also guarantees the pre-fix ordering fails without leaving
    // an unjoined test task. Cancellation alone intentionally does not open it.
    try await Task.sleep(for: .milliseconds(200))
    let beforeRelease = await gate.events
    await gate.release()
    let outcome = try await flight.value
    #expect(outcome == .cancelled)
    #expect(beforeRelease.contains("shutdown"))
    #expect(await recorder.closed)
  }

  @Test func slowTerminalExpiresWhileEntryAndCommandsContinue() async throws {
    let gate = PlanningGate(), terminal = InjectedPlanner(.cancellationAware, gate: gate)
    let entry = InjectedPlanner(.immediate, gate: gate)
    let io = try PlanningIO(gate: gate), recorder = PlanningRecorder()
    var c = MissionConfiguration()
    c.guidance.entryReplanInterval = 0.2; c.guidance.terminalReplanInterval = 0.2
    let runtime = try FlightRuntime(io: io, recorder: recorder, configuration: c,
      entryPlanningService: entry, terminalPlanningService: terminal)
    let outcome = try await runtime.run(maximumDuration: 3, realTime: false)
    #expect(outcome == .timedOut)
    #expect(await terminal.cancellations > 0)
    #expect(await terminal.calls > 1)
    #expect(await entry.calls > 1)
    #expect(await io.commands >= 20)
    #expect(await io.unsafeCommands == 0)
  }

  @Test func telemetryGapDoesNotWaitForOldWorker() async throws {
    let gate = PlanningGate(), terminal = InjectedPlanner(.held, gate: gate)
    let io = try PlanningIO(gate: gate, jumpOnRead: 3), recorder = PlanningRecorder()
    let runtime = try FlightRuntime(io: io, recorder: recorder, configuration: .init(),
      entryPlanningService: InjectedPlanner(.immediate, gate: gate), terminalPlanningService: terminal)
    let flight = Task { try await runtime.run(maximumDuration: 5, realTime: true) }
    await terminal.waitUntilStarted()
    try await Task.sleep(for: .milliseconds(400))
    let commandsWhileHeld = await io.commands
    flight.cancel()
    await gate.release()
    _ = try await flight.value
    #expect(commandsWhileHeld >= 3)
    #expect(await io.unsafeCommands == 0)
  }
}
