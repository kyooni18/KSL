import Foundation

public protocol FlightStateProviding: Sendable {
  func readState() async throws -> VehicleState
}
public protocol FlightCommanding: Sendable {
  func apply(_ envelope: CommandEnvelope) async throws
  func shutdown() async
}
public protocol FlightIO: FlightStateProviding, FlightCommanding {}

public struct FlightRecord: Codable, Sendable {
  public let sequence: UInt64
  public let state: VehicleState
  public let guidance: GuidanceSolution
  public let command: ControlCommand
  public init(
    sequence: UInt64, state: VehicleState, guidance: GuidanceSolution, command: ControlCommand
  ) {
    self.sequence = sequence
    self.state = state
    self.guidance = guidance
    self.command = command
  }
}
public protocol TelemetryRecording: Sendable {
  func record(_ record: FlightRecord) async throws
  func close() async throws
}
