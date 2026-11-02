import Foundation

public struct OrbitalControlCommand: Codable, Sendable, Equatable {
  public let planID: String
  public let axes: ControlCommand
  public let throttle: Double
  public init(planID: String, axes: ControlCommand, throttle: Double) {
    self.planID = planID; self.axes = axes; self.throttle = throttle
  }
}
public struct OrbitalExecutionDiagnostics: Codable, Sendable {
  public var planID = ""
  public var deliveredDeltaV = 0.0
  public var deliveredNormalDeltaV: Double?
  public var targetNormalDeltaV: Double?
  /// Upper bound on impulse not observed across telemetry gaps, not a measurement.
  public var unobservedDeltaVUpperBound: Double?
  public var telemetryGapResumed: Bool?
  public var targetDeltaV = 0.0
  public var periapsisAltitude = 0.0
  public var alignmentError = 0.0
  public var burnElapsed = 0.0
  public var thrustLossSeconds = 0.0
  public var reducedAssuranceAuthorized = false
  public init() {}
}

/// Propulsion is outside the unpowered command vocabulary. An implementation
/// must bind this capability to one reviewed plan and end it before entry.
public protocol OrbitalFlightIO: FlightIO {
  func armDeorbit(plan: DeorbitPlan, reducedAssurance: Bool) async throws
  func applyOrbital(sequence: UInt64, stateTime: Double, command: OrbitalControlCommand) async throws
  func finishDeorbit() async throws
  func beginEntry() async throws
}

/// Optional rail-warp ownership. Real kRPC flight I/O implements this, while
/// simulated/external FlightIO implementations keep their existing contract.
public protocol OrbitalWarpFlightIO: OrbitalFlightIO {
  /// Warp only during a propulsion-free orbital coast. Return the achieved UT.
  /// Must always restore 1x before returning, throwing or cancellation.
  func coastWarp(until stopUT: Double) async throws -> Double
}
