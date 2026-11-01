import Foundation
import KSLCore
import KSLDiagnostics
import KSLGuidance
import KSLPhysics

public enum RuntimeOutcome: String, Codable, Sendable {
  case complete, aborted, timedOut, cancelled
}

/// Owns flight-computer lifetime; actor isolation prevents concurrent control updates.
/// No detached tasks, callbacks, timers retaining self, or unchecked Sendable classes.
public actor FlightRuntime {
  private let io: any FlightIO
  private let recorder: ResilientTelemetryRecorder
  private let computer: FlightComputer
  private let entryPlanner: any MissionPlanningService
  private let terminalPlanner: any MissionPlanningService
  private var planningTask: Task<Void, Never>?
  private var terminalTask: Task<Void, Never>?
  private var completedPlan: RecordedPlanningProposal?
  private var completedTerminalPlan: RecordedPlanningProposal?
  private var planningError: String?
  private var terminalError: String?
  private var generation: UInt64 = 0
  private var planningPhase: MissionPhase?
  private var planningTaskGeneration: UInt64?
  private var terminalTaskGeneration: UInt64?
  private var planningRequestedAt: Double?
  private var terminalRequestedAt: Double?
  private var maximumTerminalPlanningAge: Double = 0
  private let maximumEntryPlanningAge: Double
  private let terminalReplanInterval: Double
  private let approachPredictionInterval: Double
  private var dynamicsRevision: UInt64 = 0
  private let interval: Double
  private var running = false
  /// Longest tolerated link/telemetry outage, paused-game wait, and forbidden-system persistence.
  private let maximumOutage: Double, maximumPause: Double, maximumUnsafe: Double
  public init(
    io: any FlightIO, recorder: any TelemetryRecording, configuration: MissionConfiguration,
    maximumOutage: Double = 20, maximumPause: Double = 600, maximumUnsafe: Double = 3,
    aerodynamics: AerodynamicsEngine? = nil, initialPhase: MissionPhase = .entry,
    initialPlan: TerminalPlan? = nil,
    entryPlanningService: (any MissionPlanningService)? = nil,
    terminalPlanningService: (any MissionPlanningService)? = nil
  ) throws {
    self.maximumOutage = maximumOutage
    self.maximumPause = maximumPause
    self.maximumUnsafe = maximumUnsafe
    self.io = io
    self.recorder = (recorder as? ResilientTelemetryRecorder) ?? .init(sink: recorder)
    interval = configuration.guidance.controlInterval
    maximumEntryPlanningAge = 2 * configuration.guidance.entryReplanInterval
    terminalReplanInterval = configuration.guidance.terminalReplanInterval
    approachPredictionInterval = configuration.guidance.approachPredictionInterval ?? 1.5
    let atmosphere = try AtmosphericModelEngine(planet: configuration.planet)
  let aerodynamics = try aerodynamics ?? AerodynamicsEngine(referenceArea: configuration.vehicle.referenceArea,
    speedbrakeFullDeploymentDragArea: configuration.vehicle.speedbrakeFullDeploymentDragArea)
    computer = try .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
      externalPlanning: true, initialPhase: initialPhase, initialPlan: initialPlan)
    entryPlanner = entryPlanningService ?? MissionPlanningWorker(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    terminalPlanner = terminalPlanningService ?? MissionPlanningWorker(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func run(maximumDuration: Double = 2400, realTime: Bool = true) async throws
    -> RuntimeOutcome
  {
    guard !running else { throw FlightError.invalidCommand("Runtime already running") }
    running = true
    generation += 1
    do {
      let result = try await execute(maximumDuration: maximumDuration, realTime: realTime)
      await finishFlight()
      running = false
      return result
    } catch {
      // Cleanup must not inherit the failed operation's cancellation state.
      // Neutralise I/O before joining workers: cancellation is cooperative and
      // a planner may still be returning from an expensive synchronous call.
      let cleanup = Task { await finishFlight() }
      await cleanup.value
      running = false
      if error is CancellationError { return .cancelled }
      throw error
    }
  }
  private func invalidatePlanning() {
    generation &+= 1
    planningTask?.cancel(); terminalTask?.cancel()
    completedPlan = nil; completedTerminalPlan = nil
    planningError = nil; terminalError = nil
  }
  private func finishFlight() async {
    invalidatePlanning()
    let entry = planningTask, terminal = terminalTask
    await io.shutdown()
    await entry?.value
    await terminal?.value
    planningTask = nil; terminalTask = nil
    planningTaskGeneration = nil; terminalTaskGeneration = nil
    planningRequestedAt = nil; terminalRequestedAt = nil
    completedPlan = nil; completedTerminalPlan = nil
    planningError = nil; terminalError = nil
    await recorder.close()
  }
  private func receive(_ proposal: RecordedPlanningProposal?, error: String? = nil,
                       generation: UInt64, terminal: Bool) {
    if terminal {
      guard terminalTaskGeneration == generation else { return }
      terminalTask = nil; terminalTaskGeneration = nil; terminalRequestedAt = nil
      guard running, generation == self.generation else { return }
      completedTerminalPlan = proposal; terminalError = error
    } else {
      guard planningTaskGeneration == generation else { return }
      planningTask = nil; planningTaskGeneration = nil; planningRequestedAt = nil
      guard running, generation == self.generation else { return }
      completedPlan = proposal; planningError = error
    }
  }
  private func schedulePlanning(from state: VehicleState) -> Data? {
    computer.observeConfiguration(state)
    if dynamicsRevision != computer.dynamicsRevision {
      dynamicsRevision = computer.dynamicsRevision
      generation &+= 1
      planningTask?.cancel(); terminalTask?.cancel()
      completedPlan = nil; completedTerminalPlan = nil
      planningError = nil; terminalError = nil
    }
    if let planningError {
      note("entry planning proposal discarded: \(planningError)")
      self.planningError = nil
    }
    if let terminalError {
      note("terminal planning proposal discarded: \(terminalError)")
      self.terminalError = nil
    }
    // Publish at most one planning proposal per control record. The other
    // remains buffered for the next 100-ms tick, preserving deterministic replay.
    var accepted: Data?
    if let proposal = completedTerminalPlan {
      if computer.accept(proposal.proposal, state: state) { accepted = proposal.encoded }
      completedTerminalPlan = nil
    } else if let proposal = completedPlan {
      if computer.accept(proposal.proposal, state: state) { accepted = proposal.encoded }
      completedPlan = nil
    }
    if planningTask != nil, computer.mission.phase != .entry { planningTask?.cancel() }
    if planningTask != nil, let requestedAt = planningRequestedAt,
       state.time - requestedAt > maximumEntryPlanningAge {
      // A 10-s-old entry rollout can no longer be flown as the current state.
      // Its own worker receives cancellation; terminal surveys run separately.
      if planningTask?.isCancelled == false {
        note("discarding expired entry forecast; terminal planning unaffected")
        planningTask?.cancel()
      }
    }
    if terminalTask != nil, let requestedAt = terminalRequestedAt,
       state.time - requestedAt > maximumTerminalPlanningAge,
       terminalTask?.isCancelled == false {
      // A result older than the relevant admission/approach freshness limit
      // cannot be used. Cancel the terminal worker independently so a long
      // search does not starve fresh surveys while entry remains in control.
      note("discarding expired terminal forecast; entry planning unaffected")
      terminalTask?.cancel()
    }
    let entryFree = planningTask == nil && completedPlan == nil
    let terminalFree = terminalTask == nil && completedTerminalPlan == nil
    guard entryFree || terminalFree,
          let request = computer.planningRequest(from: state,
            allowEntry: entryFree, allowTerminal: terminalFree) else { return accepted }
    let epoch = generation
    if request.terminalState != nil || request.approach != nil {
      let worker = terminalPlanner
      terminalTaskGeneration = epoch
      terminalRequestedAt = request.time
      maximumTerminalPlanningAge = request.approach != nil
        ? 3 * approachPredictionInterval : 2 * terminalReplanInterval
      terminalTask = Task { [weak self] in
        do {
          let proposal = try await worker.recordedPlan(request.terminalOnly())
          await self?.receive(Task.isCancelled ? nil : proposal, generation: epoch, terminal: true)
        } catch {
          await self?.receive(nil, error: Task.isCancelled ? nil : String(describing: error),
            generation: epoch, terminal: true)
        }
      }
    }
    if request.entry != nil {
      let worker = entryPlanner
      planningPhase = computer.mission.phase
      planningTaskGeneration = epoch
      planningRequestedAt = request.time
      planningTask = Task { [weak self] in
        do {
          let proposal = try await worker.recordedPlan(request.entryOnly())
          await self?.receive(Task.isCancelled ? nil : proposal, generation: epoch, terminal: false)
        } catch {
          await self?.receive(nil, error: Task.isCancelled ? nil : String(describing: error),
            generation: epoch, terminal: false)
        }
      }
    }
    return accepted
  }
  private func note(_ message: String) {
    try? FileHandle.standardError.write(contentsOf: Data(("ksl-landing: " + message + "\n").utf8))
  }
  private func execute(maximumDuration: Double, realTime: Bool) async throws -> RuntimeOutcome {
    let clock = ContinuousClock()
    var start: Double?
    var lastTime: Double?
    var lastFresh = clock.now
    var outageSince: ContinuousClock.Instant?
    var unsafeSince: ContinuousClock.Instant?
    var pausedNoted = false
    var faults = 0
    var previousWallTick: ContinuousClock.Instant?
    var missedDeadlines: UInt64 = 0
    func seconds(_ duration: Duration) -> Double {
      let p = duration.components
      return Double(p.seconds) + Double(p.attoseconds) / 1e18
    }
    while true {
      try Task.checkCancellation()
      let tick = clock.now
      do {
        let state = try await io.readState()
        let acquired = clock.now
        if let lastTime, state.time < lastTime - 1e-6 {
          throw FlightError.rearmRequired("Time moved backwards (revert or quickload)")
        }
        if start == nil { start = state.time }
        if state.time - start! > maximumDuration { return .timedOut }
        if let lastTime, state.time == lastTime {
          // The game is paused (or between physics frames): hold the last controls and wait.
          let paused = lastFresh.duration(to: clock.now)
          if paused > .seconds(computer.configuration.guidance.maximumTelemetryAge), !pausedNoted {
            pausedNoted = true
            note("game time is not advancing (paused?); holding last controls")
          }
          if paused > .seconds(maximumPause) { throw FlightError.staleTelemetry }
          try await clock.sleep(for: .milliseconds(20))
          continue
        }
        if pausedNoted { note("game time resumed"); pausedNoted = false }
        if !state.systems.unpowered {
          // SAS, airbrakes or an engine came on (staging, a keypress, a game reload):
          // push the restrictions again and give them a moment before giving up.
          let since = unsafeSince ?? clock.now
          unsafeSince = since
          if since.duration(to: clock.now) > .seconds(maximumUnsafe) {
            throw FlightError.invalidTelemetry(
              "Propulsion, SAS or airbrakes stayed active after re-assertion")
          }
          try await io.assertSafety()
          try await clock.sleep(for: .milliseconds(100))
          continue
        }
        unsafeSince = nil
        let simulationInterval = lastTime.map { state.time - $0 } ?? interval
        if outageSince != nil || simulationInterval > 1 {
          // Results and sample derivatives from before the outage are stale.
          // Invalidate old snapshots without blocking the resumed command
          // loop on a cancelled worker. Its lane stays occupied until receive
          // acknowledges completion, so no replacement can race that worker.
          invalidatePlanning()
          computer.resumeAfterTelemetryGap(at: state)
        }
        lastTime = state.time
        lastFresh = clock.now
        let acceptedPlanning = schedulePlanning(from: state)
        var record = try computer.step(state)
        record.planningProposal = acceptedPlanning
        let computed = clock.now
        try Task.checkCancellation()
        let attitude = computer.attitudeTarget(state: state, solution: record.guidance)
        try await io.apply(
          .init(
            sequence: record.sequence, stateTime: state.time, command: record.command,
            attitude: attitude))
        let applied = clock.now
        let elapsed = seconds(tick.duration(to: applied))
        if elapsed > interval { missedDeadlines += UInt64(floor(elapsed / interval)) }
        record.timing = .init(simulationInterval: simulationInterval,
          acquisitionSeconds: seconds(tick.duration(to: acquired)),
          computationSeconds: seconds(acquired.duration(to: computed)),
          commandSeconds: seconds(computed.duration(to: applied)),
          wallInterval: previousWallTick.map { seconds($0.duration(to: tick)) } ?? 0,
          missedDeadlines: missedDeadlines)
        previousWallTick = tick
        try await recorder.record(record)
        if outageSince != nil { note("link recovered after \(faults) fault(s)") }
        outageSince = nil
        faults = 0
        if record.guidance.phase == .complete { return .complete }
      } catch let error as FlightError where error.isRecoverable {
        let since = outageSince ?? clock.now
        outageSince = since
        faults += 1
        if since.duration(to: clock.now) > .seconds(maximumOutage) { throw error }
        if faults == 1 || faults % 10 == 0 { note("recovering from: \(error) (fault \(faults))") }
        let stale: Bool
        if case .staleTelemetry = error { stale = true } else { stale = false }
        try await clock.sleep(for: .milliseconds(stale ? 20 : 150))
        do { try await io.recover() } catch let error as FlightError where error.isRecoverable {}
        continue
      }
      if realTime { try await clock.sleep(until: tick.advanced(by: .seconds(interval))) }
    }
  }
}
