import Foundation
import KSLCore

/// Single owner, bounded buffering, explicit flush/close. Used synchronously offline.
public final class JSONLFileWriter {
  private var handle: FileHandle?
  private let encoder: JSONEncoder
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
    try handle.write(contentsOf: data)
  }
  public func close() throws {
    guard let h = handle else { return }
    handle = nil
    do { try h.synchronize() } catch {
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
