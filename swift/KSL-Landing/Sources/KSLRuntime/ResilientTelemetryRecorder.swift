import Foundation
import KSLCore

/// Recording is an observer, never a flight-control or link-recovery dependency.
/// Retains only a gap summary; an unavailable disk cannot grow an in-memory log.
public actor ResilientTelemetryRecorder: TelemetryRecording {
  private let sink: any TelemetryRecording
  private var gap: RecordingGap?
  private var closed = false
  public init(sink: any TelemetryRecording) { self.sink = sink }

  public func record(_ record: FlightRecord) async throws {
    var annotated = record
    annotated.recordingGap = gap ?? record.recordingGap
    do {
      try await sink.record(annotated)
      if let gap {
        note("recording resumed after \(gap.lostRecords) missing records (UT \(gap.firstTime)...\(gap.lastTime)); continuity is not established")
      }
      gap = nil
    } catch is CancellationError {
      throw CancellationError()
    } catch {
      let reason = String(describing: error)
      if var existing = gap {
        existing.lastSequence = record.sequence; existing.lastTime = record.state.time
        existing.lostRecords += 1; existing.lastError = reason
        gap = existing
      } else {
        gap = .init(sequence: record.sequence, time: record.state.time, error: reason)
        note("recording failed at sequence \(record.sequence); flight control continues: \(reason)")
      }
    }
  }

  public func close() async {
    guard !closed else { return }
    closed = true
    if let gap {
      note("recording ended with \(gap.lostRecords) missing records (UT \(gap.firstTime)...\(gap.lastTime))")
    }
    do { try await sink.close() }
    catch { note("recording close failed: \(error)") }
  }

  private func note(_ message: String) {
    // stderr can itself be redirected to the same full filesystem.
    try? FileHandle.standardError.write(contentsOf: Data(("ksl-recording: " + message + "\n").utf8))
  }
}
