import Foundation

/// The command vocabulary cannot request propulsion, SAS, or the legacy
/// airbrake action group. Split-rudder drag and body-flap trim are separate,
/// bounded aerodynamic commands. RCS is an attitude actuator.
public struct ControlCommand: Codable, Sendable, Equatable {
  public let pitch: Double
  public let roll: Double
  public let yaw: Double
  public let gearDown: Bool
  public let wheelBrake: Double
  public let wheelSteering: Double
  public let deployParachutes: Bool?
  public let rcsEnabled: Bool?
  public let speedbrakeFraction: Double?
  public let bodyFlapTrimFraction: Double?
  public init(
    pitch: Double = 0, roll: Double = 0, yaw: Double = 0, gearDown: Bool = false,
    wheelBrake: Double = 0, wheelSteering: Double = 0, deployParachutes: Bool? = nil,
    rcs: Bool = false, speedbrakeFraction: Double = 0, bodyFlapTrimFraction: Double = 0
  ) {
    self.rcsEnabled = rcs ? true : nil
    self.pitch = pitch
    self.roll = roll
    self.yaw = yaw
    self.gearDown = gearDown
    self.wheelBrake = wheelBrake
    self.wheelSteering = wheelSteering
    self.deployParachutes = deployParachutes
    self.speedbrakeFraction = speedbrakeFraction == 0 ? nil : speedbrakeFraction
    self.bodyFlapTrimFraction = bodyFlapTrimFraction == 0 ? nil : bodyFlapTrimFraction
  }
  public var throttle: Double { 0 }
  public var sas: Bool { false }
  public var rcs: Bool { rcsEnabled ?? false }
  public var airbrakes: Bool { false }
  public func validate(grounded: Bool) throws {
    guard [pitch, roll, yaw, wheelSteering].allSatisfy({ $0.isFinite && abs($0) <= 1 }),
      (speedbrakeFraction ?? 0).isFinite, (0...1).contains(speedbrakeFraction ?? 0),
      (bodyFlapTrimFraction ?? 0).isFinite, abs(bodyFlapTrimFraction ?? 0) <= 1,
      wheelBrake.isFinite, (0...1).contains(wheelBrake), grounded || wheelBrake == 0,
      grounded || deployParachutes != true
    else {
      throw FlightError.invalidCommand("Nonfinite/out-of-range controls or airborne wheel braking")
    }
  }
}

/// Desired vehicle nose and roof directions (unit vectors in the planet-fixed local frame).
public struct AttitudeTarget: Sendable, Equatable {
  public let nose: Vector3
  public let roof: Vector3
  public init(nose: Vector3, roof: Vector3) {
    self.nose = nose
    self.roof = roof
  }
}

public struct CommandEnvelope: Sendable {
  public let sequence: UInt64
  public let stateTime: Double
  public let command: ControlCommand
  /// Diagnostic attitude form of the guidance target. Actuation always uses native axis commands.
  public let attitude: AttitudeTarget?
  public init(
    sequence: UInt64, stateTime: Double, command: ControlCommand, attitude: AttitudeTarget? = nil
  ) {
    self.sequence = sequence
    self.stateTime = stateTime
    self.command = command
    self.attitude = attitude
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
  case rearmRequired(String)
  /// Faults that a link or game hiccup can cause and that a retry can clear. Identity changes
  /// (another vessel, different parts or planet), time reversal and bad commands stay fatal.
  public var isRecoverable: Bool {
    switch self {
    case .deadlineExceeded, .staleTelemetry, .disconnected, .io: true
    case .invalidTelemetry(let s):
      !(s.hasPrefix("Active vessel") || s.hasPrefix("Vessel part count")
        || s.hasPrefix("Planet") || s.hasPrefix("Time moved backwards"))
    case .invalidConfiguration, .invalidCommand, .protocolViolation, .infeasible, .rearmRequired: false
    }
  }
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
    case .rearmRequired(let s): "Control disarmed; fresh engage required: \(s)"
    }
  }
}
