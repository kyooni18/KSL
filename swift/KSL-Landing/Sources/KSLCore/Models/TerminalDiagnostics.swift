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
  /// Nominal complete-path pricing, distinct from all ownership-contract checks.
  public var physicallyFeasible: Bool?
  public var positionMargin: Double?
  public var controlOwnershipTransferred: Bool?
  public init() {}
}

/// MM304 -> MM305 handoff evidence: the predicted capture MM304 is steering
/// toward, and MM305's independent verdict on the measured state.
public struct CaptureHandoffDiagnostics: Codable, Sendable {
  public var advisoryInletTime: Double?
  public var advisorySourceTime: Double?
  public var advisoryInletModel: String?
  /// Measured runway-chart coordinates, distinct from the forecasted capture
  /// endpoint and from the distance to the movable MM304 inlet objective.
  public var actualAlong: Double?
  public var actualCross: Double?
  public var actualRunwayRange: Double?
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
  public var mm305SearchHorizonExceeded: Bool?
  public var mm305MinimumRequiredLength: Double?, mm305MaximumSearchedLength: Double?
  public var mm305TurnAuthorityAssessed: Bool?
  public var mm305PlanAccepted = false
  public var activePlanID: Int?
  public var activePlanCreatedAt: Double?
  public var activeHACRadius: Double?
  public var activeHACGrowth: Double?
  public var activeHACTurnAngle: Double?
  public var activeHACSide: Int?
  public var activeHACExitAlong: Double?
  /// Current regenerated reference [UT, latitude deg, longitude deg, ASL altitude, airspeed].
  public var activeReference: [[Double]]?
  public var activeReferencePredictionComplete: Bool?
  public var activeReferenceDynamicsCurrent: Bool?
  public var referenceLateralDemand: Double?
  public var allocatedLateralAcceleration: Double?
  public var activeReferencePredictedThroughDistance: Double?
  public var advisoryPlanID: Int?
  public var advisoryPlanFeasible: Bool?
  /// Unaccepted MM305 candidate geometry for a display-only provisional overlay.
  /// [estimated UT, latitude deg, longitude deg, ASL altitude, nominal speed].
  /// This is NOT a qualified landing path or an actuator reference.
  public var advisoryReference: [[Double]]?
  public var trackingError: Double?
  public var referenceHeadingError: Double?
  /// Forward geometric preview; separate from nearest-reference error.
  public var lookaheadDistance: Double?, lookaheadHeadingError: Double?
  public var referenceHeight: Double?
  public var referenceSpeed: Double?
  public var referenceSpeedbrakeFraction: Double?
  /// Current model-derived Final alignment target, not a capture certificate.
  public var nominalEnergyAlong: Double?
  public var nominalEnergyHeight: Double?
  public var nominalEnergyAirspeed: Double?
  public var nominalEnergyFlightPath: Double?
  public var nominalEnergyPredictedTouchdownSpeed: Double?
  public var referenceSegment: String?
  public var mm305Advisory: String?
  /// MM305's own physical/closed-loop violations for the last search.
  public var mm305Violations: [ConstraintViolation] = []
  public init() {}
}
