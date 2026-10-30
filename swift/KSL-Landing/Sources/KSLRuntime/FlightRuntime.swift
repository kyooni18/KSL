import Foundation
import KSLCore
import KSLDiagnostics
import KSLPhysics

public enum RuntimeOutcome: String, Codable, Sendable {
  case complete, aborted, timedOut, cancelled
}

/// Owns flight-computer lifetime; actor isolation prevents concurrent control updates.
/// No detached tasks, callbacks, timers retaining self, or unchecked Sendable classes.
public actor FlightRuntime {
  private let io: any FlightIO
  private let recorder: any TelemetryRecording
  private let computer: FlightComputer
  private let interval: Double
  private var running = false
  /// Longest tolerated link/telemetry outage, paused-game wait, and forbidden-system persistence.
  private let maximumOutage: Double, maximumPause: Double, maximumUnsafe: Double
  public init(
    io: any FlightIO, recorder: any TelemetryRecording, configuration: MissionConfiguration,
    maximumOutage: Double = 20, maximumPause: Double = 600, maximumUnsafe: Double = 3
  ) throws {
    self.maximumOutage = maximumOutage
    self.maximumPause = maximumPause
    self.maximumUnsafe = maximumUnsafe
    self.io = io
    self.recorder = recorder
    interval = configuration.guidance.controlInterval
    let atmosphere = try AtmosphericModelEngine(planet: configuration.planet)
    let aerodynamics = try AerodynamicsEngine(referenceArea: configuration.vehicle.referenceArea)
    computer = try .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func run(maximumDuration: Double = 2400, realTime: Bool = true) async throws
    -> RuntimeOutcome
  {
    guard !running else { throw FlightError.invalidCommand("Runtime already running") }
    running = true
    do {
      let result = try await execute(maximumDuration: maximumDuration, realTime: realTime)
      await io.shutdown()
      try await recorder.close()
      running = false
      return result
    } catch {
      let io = self.io
      let recorder = self.recorder
      // A bounded, explicitly awaited cleanup task is intentionally not a
      // child of the cancelled operation, so cancellation cannot skip I/O
      // neutralisation on a still usable connection.
      let cleanup = Task {
        await io.shutdown()
        try? await recorder.close()
      }
      await cleanup.value
      running = false
      if error is CancellationError { return .cancelled }
      throw error
    }
  }
  private func note(_ message: String) {
    FileHandle.standardError.write(Data(("ksl-landing: " + message + "\n").utf8))
  }
  private func execute(maximumDuration: Double, realTime: Bool) async throws -> RuntimeOutcome {
    let clock = ContinuousClock()
    var start: Double?
    var lastTime: Double?
    var lastFresh = clock.now
    var outageSince: ContinuousClock.Instant?
    var unsafeSince: ContinuousClock.Instant?
    var heldAttitude: AttitudeTarget?
    var pausedNoted = false
    var faults = 0
    while true {
      try Task.checkCancellation()
      let tick = clock.now
      do {
        let state = try await io.readState()
        if let lastTime, state.time < lastTime - 1e-6 {
          throw FlightError.invalidTelemetry("Time moved backwards (revert or quickload)")
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
          // SAS, RCS, airbrakes or an engine came on (staging, a keypress, a game reload):
          // push the restrictions again and give them a moment before giving up.
          let since = unsafeSince ?? clock.now
          unsafeSince = since
          if since.duration(to: clock.now) > .seconds(maximumUnsafe) {
            throw FlightError.invalidTelemetry(
              "Propulsion, SAS, RCS or airbrakes stayed active after re-assertion")
          }
          try await io.assertSafety()
          try await clock.sleep(for: .milliseconds(100))
          continue
        }
        unsafeSince = nil
        lastTime = state.time
        lastFresh = clock.now
        let record = try computer.step(state)
        try Task.checkCancellation()
        // On abort keep flying the last good attitude rather than snapping to 0/0.
        let attitude =
          record.guidance.phase == .abort && heldAttitude != nil
          ? heldAttitude : computer.attitudeTarget(state: state, solution: record.guidance)
        if record.guidance.phase != .abort { heldAttitude = attitude }
        try await io.apply(
          .init(
            sequence: record.sequence, stateTime: state.time, command: record.command,
            attitude: attitude))
        try await recorder.record(record)
        if outageSince != nil { note("link recovered after \(faults) fault(s)") }
        outageSince = nil
        faults = 0
        if record.guidance.phase == .complete { return .complete }
        if record.guidance.phase == .abort { return .aborted }
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
