import Foundation
import KSLCore

public final class JSONLReader {
  private var handle: FileHandle?
  private var buffer = Data()
  private var eof = false
  public private(set) var lineNumber = 0
  public let maximumLineBytes: Int
  public init(url: URL, maximumLineBytes: Int = 2_000_000) throws {
    handle = try FileHandle(forReadingFrom: url)
    self.maximumLineBytes = maximumLineBytes
  }
  public func nextLine() throws -> Data? {
    while true {
      if let newline = buffer.firstIndex(of: 10) {
        guard buffer.distance(from: buffer.startIndex, to: newline) <= maximumLineBytes else {
          throw FlightError.io("Oversized telemetry line")
        }
        let line = Data(buffer[..<newline])
        buffer.removeSubrange(...newline)
        lineNumber += 1
        return line
      }
      if eof {
        if buffer.isEmpty { return nil }
        let line = buffer
        buffer.removeAll(keepingCapacity: false)
        lineNumber += 1
        return line
      }
      guard let h = handle else { throw FlightError.io("Reader closed") }
      let data = try h.read(upToCount: 64 * 1024) ?? Data()
      if data.isEmpty { eof = true } else { buffer.append(data) }
      if buffer.count > maximumLineBytes && !buffer.contains(10) {
        throw FlightError.io("Oversized telemetry line")
      }
    }
  }
  public func close() throws {
    try handle?.close()
    handle = nil
  }
  deinit { try? handle?.close() }
}

public final class FlightReplayReader {
  private let reader: JSONLReader
  private let decoder = JSONDecoder()
  private var lastTime: Double?
  public init(url: URL) throws { reader = try .init(url: url) }
  public func next() throws -> FlightRecord? {
    guard let data = try reader.nextLine() else { return nil }
    let record = try decoder.decode(FlightRecord.self, from: data)
    if let t = lastTime, record.state.time <= t {
      throw FlightError.invalidTelemetry("Nonmonotonic replay at line \(reader.lineNumber)")
    }
    lastTime = record.state.time
    return record
  }
  public func close() throws { try reader.close() }
}
