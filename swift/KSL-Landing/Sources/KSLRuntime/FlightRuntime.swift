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
  public init(
    io: any FlightIO, recorder: any TelemetryRecording, configuration: MissionConfiguration
  ) throws {
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
  private func execute(maximumDuration: Double, realTime: Bool) async throws -> RuntimeOutcome {
    let clock = ContinuousClock()
    var start: Double?
    var lastTime: Double?
    var lastFresh = clock.now
    while true {
      try Task.checkCancellation()
      let tick = clock.now
      let state = try await io.readState()
      if start == nil { start = state.time }
      if state.time - start! > maximumDuration { return .timedOut }
      if let lastTime, state.time == lastTime {
        if lastFresh.duration(to: clock.now)
          > .seconds(computer.configuration.guidance.maximumTelemetryAge)
        {
          throw FlightError.staleTelemetry
        }
        try await clock.sleep(for: .milliseconds(20))
        continue
      }
      lastTime = state.time
      lastFresh = clock.now
      let record = try computer.step(state)
      try Task.checkCancellation()
      try await io.apply(
        .init(sequence: record.sequence, stateTime: state.time, command: record.command))
      try await recorder.record(record)
      if record.guidance.phase == .complete { return .complete }
      if record.guidance.phase == .abort { return .aborted }
      if realTime { try await clock.sleep(until: tick.advanced(by: .seconds(interval))) }
    }
  }
}
