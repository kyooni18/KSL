import Foundation

public protocol FlightStateProviding: Sendable {
  func readState() async throws -> VehicleState
}
public protocol FlightCommanding: Sendable {
  func apply(_ envelope: CommandEnvelope) async throws
  func shutdown() async
  /// Re-establish a usable link after a transient fault; never replays commands.
  func recover() async throws
  /// Re-assert the forbidden-system restrictions (throttle, SAS, RCS, airbrakes) now.
  func assertSafety() async throws
}
extension FlightCommanding {
  public func recover() async throws {}
  public func assertSafety() async throws {}
}
public protocol FlightIO: FlightStateProviding, FlightCommanding {}

public struct FlightRecord: Codable, Sendable {
  public let sequence: UInt64
  public let state: VehicleState
  public let guidance: GuidanceSolution
  public let command: ControlCommand
  public var control: ControlDiagnostics?
  public var timing: ControlLoopTiming?
  public var orbital: OrbitalExecutionDiagnostics?
  public var orbitalCommand: OrbitalControlCommand?
  public var planningMode: String?
  public var planningProposal: Data?
  public var calibration: CalibrationDiagnostics?
  public var dynamicsChange: VehicleDynamicsChange?
  public var initialPhase: MissionPhase?
  public var initialTerminalPlan: Data?
  public var initialDeorbitPlan: Data?
  public var recordingGap: RecordingGap?
  public init(
    sequence: UInt64, state: VehicleState, guidance: GuidanceSolution, command: ControlCommand,
    control: ControlDiagnostics? = nil, timing: ControlLoopTiming? = nil
  ) {
    self.sequence = sequence
    self.state = state
    self.guidance = guidance
    self.command = command
    self.control = control
    self.timing = timing
  }
}
public protocol TelemetryRecording: Sendable {
  func record(_ record: FlightRecord) async throws
  func close() async throws
}
