import Foundation

public enum EntryPhase: Int, Codable, Sendable {
  case preentry, temperatureControl, equilibriumGlide, constantDrag, transition
}
public struct EntryDiagnostics: Codable, Sendable {
  public var phase: EntryPhase = .preentry
  public var referenceDrag = 0.0, observedDrag = 0.0, verticalLift = 0.0
  /// predictedRange is a fitted drag-reference integral (not a forecast).
  /// requiredRange is the unscaled physical geodesic to the capture objective.
  /// rangeError is relative to referenceTargetRange, NOT MM305 reachability.
  public var predictedRange = 0.0, requiredRange = 0.0, rangeError = 0.0
  public var referenceTargetRange: Double?
  public var dragScale = 1.0, confidence = 0.0
  /// Effective best-glide polar parameters used by DRAGREF; provenance makes
  /// provisional configuration values distinct from flight-calibrated data.
  public var referenceBallisticCoefficient: Double?
  public var referenceLiftToDrag: Double?
  public var referencePolarEvidence: EvidenceKind?
  public var degraded = true, dragFloorActive = false
  public var energyBank = 0.0, commandedBank = 0.0
  public var reversalTime: Double?, reversalAvailableTime: Double?
  public var reversalCount = 0, reversalArmed = false, reversalFeasible = false
  public var projectedCrossrange = 0.0, rollRateLimit = 0.0
  public var limitingReason = "initializing"
  /// Optional for backward-compatible replay of pre-budget telemetry. J/kg,
  /// except energyDragBudget (m/s²) and recoveryAlpha (radians).
  public var captureSpecificEnergy: Double?, captureEnergyMargin: Double?
  public var energyDragBudget: Double?, energyReserve: Double?, recoveryAlpha: Double?
  public var bestRangeRecovery: Bool?
  /// Requested and available ground-track normal acceleration, m/s².
  public var lateralDemand: Double?, lateralAuthority: Double?
  /// Modeled allocation targets/residuals, not measured forces (m/s²).
  public var lateralAllocationTarget: Double?, selectedDragAcceleration: Double?, dragTrackingResidual: Double?
  /// Optimistic *static* joint polar feasibility, NOT full MM305 reachability.
  /// Force and drag quantities are accelerations (m/s2).
  public var jointRequiredLift: Double?, jointMaximumLift: Double?
  public var jointMaximumLateral: Double?
  public var jointMinimumDrag: Double?
  public var jointDragBudgetDeficit: Double?
  public var jointForceAttainable: Bool?
  public var terminalLookaheadDistance: Double?, terminalLookaheadHeadingError: Double?
  public var minimumTurnTime: Double?, terminalAdviceAge: Double?
  public var terminalSteeringActive: Bool?
  public var energyBudgetEndpoint: String?
  public var energyAllocationMode: String?
  public var terminalDeficit: String?
  /// Point-mass range reference; never a certified closed-loop trajectory.
  public var rangeReferenceBlend: Double?, rangeReferenceAlong: Double?
  public var rangeReferenceAltitude: Double?, rangeReferenceSpeed: Double?
  public var rangeReferenceBracketed: Bool?, rangeReferenceReachedEnergy: Bool?
  public init() {}
}
