import Foundation
import KSLCore

/// Single owner, bounded buffering, explicit flush/close. Used synchronously offline.
public final class JSONLFileWriter {
  private var handle: FileHandle?
  private let encoder: JSONEncoder
  private var committedOffset: UInt64 = 0
  private var needsRollback = false
  public init(url: URL) throws {
    try Data().write(to: url, options: .withoutOverwriting)
    handle = try FileHandle(forWritingTo: url)
    encoder = JSONEncoder()
    encoder.outputFormatting = [.sortedKeys]
  }
  public func write<T: Encodable>(_ value: T) throws {
    guard let handle else { throw FlightError.io("Recorder already closed") }
    var data = try encoder.encode(value)
    data.append(10)
    try repairPartialWrite(handle)
    do {
      try handle.write(contentsOf: data)
      committedOffset += UInt64(data.count)
    } catch {
      // ENOSPC can leave half a JSON object. Retry only from the last complete
      // line so resuming the recorder never joins two different records.
      needsRollback = true
      throw error
    }
  }
  private func repairPartialWrite(_ handle: FileHandle) throws {
    guard needsRollback else { return }
    try handle.truncate(atOffset: committedOffset)
    try handle.seek(toOffset: committedOffset)
    needsRollback = false
  }
  public func close() throws {
    guard let h = handle else { return }
    handle = nil
    do { try repairPartialWrite(h); try h.synchronize() } catch {
      try? h.close()
      throw error
    }
    try h.close()
  }
  deinit { try? handle?.close() }
}

public actor JSONLTelemetryRecorder: TelemetryRecording {
  private let writer: JSONLFileWriter
  public init(url: URL) throws { writer = try .init(url: url) }
  public func record(_ record: FlightRecord) throws { try writer.write(record) }
  public func close() throws { try writer.close() }
}

public actor NullTelemetryRecorder: TelemetryRecording {
  public private(set) var records = 0
  public init() {}
  public func record(_ record: FlightRecord) { records += 1 }
  public func close() {}
}
