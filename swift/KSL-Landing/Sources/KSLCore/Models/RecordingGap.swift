import Foundation

/// Missing control-rate records are explicit when a failed recorder resumes.
/// This does not certify continuity or reconstruct observations that were lost.
public struct RecordingGap: Codable, Sendable {
  public let firstSequence: UInt64
  public var lastSequence: UInt64
  public let firstTime: Double
  public var lastTime: Double
  public var lostRecords: UInt64
  public var lastError: String
  public init(sequence: UInt64, time: Double, error: String) {
    firstSequence = sequence; lastSequence = sequence
    firstTime = time; lastTime = time; lostRecords = 1; lastError = error
  }
}
