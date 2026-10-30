import Foundation

public struct ApproachDiagnostics: Codable, Sendable {
  public var wheelHeight = 0.0
  public var along = 0.0
  public var preflareLatched = false
  public var arcActive = false
  public var triggerHeight = 0.0
  public var desiredSink = 0.0
  public var pullAcceleration = 0.0
  public var availablePull = 0.0
  public var oneGravitySpeed: Double?
  public var predictedTouchdownAlong: Double?
  public var predictedTouchdownSpeed: Double?
  public var requiredNormalAcceleration = 0.0
  public var liftEffectiveness = 1.0
  public var dragEffectiveness = 1.0
  public init() {}
}

public struct TerminalAdmissionDiagnostics: Codable, Sendable {
  public var accepted = false
  public var altitudeMargin = 0.0
  public var machMargin = 0.0
  public var headingMargin = 0.0
  public var planAge = 0.0
  public var violations: [ConstraintViolation] = []
  public init() {}
}

/// MM304 -> MM305 handoff evidence: the predicted capture MM304 is steering
/// toward, and MM305's independent verdict on the measured state.
public struct CaptureHandoffDiagnostics: Codable, Sendable {
  public var predictedFeasible = false
  public var predictedDeficit = "terminalRegionNotReached"
  public var predictedAt: Double?
  public var predictedCaptureTime: Double?
  public var predictedAltitude: Double?
  public var predictedMach: Double?
  public var predictedAlong: Double?
  public var predictedCross: Double?
  public var predictedHACRadius: Double?
  public var predictedHACTurnAngle: Double?
  public var predictedHACSide: Int?
  public var predictedPathLength: Double?
  public var excessEnergyMargin: Double?
  public var deficitEnergyMargin: Double?
  public var requiredHeadingChange: Double?
  public var availableTurnRate: Double?
  public var rangeMultiplier: Double?
  /// Lateral MM304 aim offset planned backward from TAEM capture (m).
  public var plannedCrossAim: Double?
  /// Sparse predicted path [time, along, cross, height, speed, alpha, bank].
  public var predictedTrack: [[Double]] = []
  /// MM305's own envelope verdict on the current state (when searched).
  public var mm305Searched = false
  public var mm305Deficit: String?
  public var mm305PlanAccepted = false
  public var mm305Advisory: String?
  /// MM305's own physical/closed-loop violations for the last search.
  public var mm305Violations: [ConstraintViolation] = []
  public init() {}
}
