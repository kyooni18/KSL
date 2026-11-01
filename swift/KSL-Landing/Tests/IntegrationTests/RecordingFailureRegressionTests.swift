import Foundation
import KSLCore
import KSLRuntime
import Testing

private actor FailingRecordingSink: TelemetryRecording {
  private var remainingFailures = 2
  private(set) var records: [FlightRecord] = []
  private(set) var closed = false
  func record(_ record: FlightRecord) throws {
    if remainingFailures > 0 {
      remainingFailures -= 1
      throw NSError(domain: NSCocoaErrorDomain, code: 640)
    }
    records.append(record)
  }
  func close() throws {
    closed = true
    throw NSError(domain: NSCocoaErrorDomain, code: 640)
  }
}

struct RecordingFailureRegressionTests {
  private func record(_ sequence: UInt64) -> FlightRecord {
    .init(sequence: sequence,
      state: .init(time: Double(sequence), position: .init(670_000, 0, 0),
        velocity: .init(0, 2000, 0), attitude: .init()),
      guidance: .init(phase: .entry, target: .init(angleOfAttack: 0)), command: .init())
  }

  @Test func diskFailureDoesNotEscapeAndRecoveryRecordsTheMissingInterval() async throws {
    let sink = FailingRecordingSink()
    let recorder = ResilientTelemetryRecorder(sink: sink)
    for sequence in UInt64(1)...4 { try await recorder.record(record(sequence)) }
    let records = await sink.records
    #expect(records.count == 2)
    let recovered = try #require(records.first)
    let following = try #require(records.last)
    #expect(recovered.sequence == 3)
    #expect(recovered.recordingGap?.lostRecords == 2)
    #expect(recovered.recordingGap?.firstSequence == 1)
    #expect(recovered.recordingGap?.lastSequence == 2)
    #expect(recovered.recordingGap?.firstTime == 1)
    #expect(recovered.recordingGap?.lastTime == 2)
    #expect(following.recordingGap == nil)
    await recorder.close()
    #expect(await sink.closed)
  }
}
