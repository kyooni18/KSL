import Foundation
import KSLCore
import KSLDiagnostics
import KSLPhysics

/// Full control-rate records and new observations have separate files. The
/// original SQLite archive is opened only by PhysicsObservationStore.
public actor SessionTelemetryRecorder: TelemetryRecording {
  private let writer: JSONLFileWriter
  private let journal: PhysicsObservationJournal?
  private var journalContextValid = true
  public init(directory: URL, configuration: MissionConfiguration, sessionID: String,
              initialPhase: MissionPhase, context: PhysicsContext?) throws {
    try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
    let encoder = JSONEncoder(); encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
    try encoder.encode(configuration).write(to: directory.appendingPathComponent("configuration.json"), options: .withoutOverwriting)
    struct Manifest: Encodable {
      let schema: String, sessionID: String, initialPhase: MissionPhase
      let evidence: String, nativeAxes: Bool, sourceReference: String
    }
    try encoder.encode(Manifest(schema: "native-swift-flight-v2", sessionID: sessionID,
      initialPhase: initialPhase, evidence: "live-kRPC-observations", nativeAxes: true,
      sourceReference: "f1eebdc0e6f8bb7e8b8a354b42eb2d829cdc3cda"))
      .write(to: directory.appendingPathComponent("manifest.json"), options: .withoutOverwriting)
    writer = try .init(url: directory.appendingPathComponent("flight.jsonl"))
    if let archive = configuration.physicsArchive, let context {
      journal = try .init(url: directory.appendingPathComponent("new-observations.jsonl"),
        protectedArchive: URL(fileURLWithPath: archive.path), context: context,
        sessionID: sessionID, flightKey: sessionID)
    } else { journal = nil }
  }
  public func record(_ record: FlightRecord) async throws {
    try writer.write(record)
    // A journal is bound to the original archive structure/session. Preserve
    // all raw telemetry, but quarantine derived observations after a dynamics
    // change until a new compatible context is explicitly established.
    if record.dynamicsChange != nil { journalContextValid = false }
    if journalContextValid { try await journal?.observe(record.state, quality: record.calibration?.observation) }
  }
  public func close() async throws {
    var failure: (any Error)?
    do { try writer.close() } catch { failure = error }
    do { try await journal?.close() } catch { if failure == nil { failure = error } }
    if let failure { throw failure }
  }
}
