import Foundation

/// The command vocabulary cannot request propulsion, SAS, RCS, or airbrakes.
public struct ControlCommand: Codable, Sendable, Equatable {
  public let pitch: Double
  public let roll: Double
  public let yaw: Double
  public let gearDown: Bool
  public let wheelBrake: Double
  public let wheelSteering: Double
  public init(
    pitch: Double = 0, roll: Double = 0, yaw: Double = 0, gearDown: Bool = false,
    wheelBrake: Double = 0, wheelSteering: Double = 0
  ) {
    self.pitch = pitch
    self.roll = roll
    self.yaw = yaw
    self.gearDown = gearDown
    self.wheelBrake = wheelBrake
    self.wheelSteering = wheelSteering
  }
  public var throttle: Double { 0 }
  public var sas: Bool { false }
  public var rcs: Bool { false }
  public var airbrakes: Bool { false }
  public func validate(grounded: Bool) throws {
    guard [pitch, roll, yaw, wheelSteering].allSatisfy({ $0.isFinite && abs($0) <= 1 }),
      wheelBrake.isFinite, (0...1).contains(wheelBrake), grounded || wheelBrake == 0
    else {
      throw FlightError.invalidCommand("Nonfinite/out-of-range controls or airborne wheel braking")
    }
  }
}

public struct CommandEnvelope: Sendable {
  public let sequence: UInt64
  public let stateTime: Double
  public let command: ControlCommand
  public init(sequence: UInt64, stateTime: Double, command: ControlCommand) {
    self.sequence = sequence
    self.stateTime = stateTime
    self.command = command
  }
}

public enum FlightError: Error, Sendable, CustomStringConvertible {
  case invalidConfiguration(String)
  case invalidTelemetry(String)
  case invalidCommand(String)
  case disconnected(String)
  case protocolViolation(String)
  case deadlineExceeded, staleTelemetry
  case infeasible(String)
  case io(String)
  public var description: String {
    switch self {
    case .invalidConfiguration(let s): "Configuration: \(s)"
    case .invalidTelemetry(let s): "Telemetry: \(s)"
    case .invalidCommand(let s): "Command: \(s)"
    case .disconnected(let s): "Disconnected: \(s)"
    case .protocolViolation(let s): "Protocol: \(s)"
    case .deadlineExceeded: "Transport deadline exceeded"
    case .staleTelemetry: "Telemetry stale or time discontinuity"
    case .infeasible(let s): "Infeasible: \(s)"
    case .io(let s): "I/O: \(s)"
    }
  }
}
