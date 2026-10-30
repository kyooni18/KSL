import Foundation

public enum EntryPhase: Int, Codable, Sendable {
  case preentry, temperatureControl, equilibriumGlide, constantDrag, transition
}
public struct EntryDiagnostics: Codable, Sendable {
  public var phase: EntryPhase = .preentry
  public var referenceDrag = 0.0, observedDrag = 0.0, verticalLift = 0.0
  public var predictedRange = 0.0, requiredRange = 0.0, rangeError = 0.0
  public var dragScale = 1.0, confidence = 0.0
  public var degraded = true, dragFloorActive = false
  public var energyBank = 0.0, commandedBank = 0.0
  public var reversalTime: Double?, reversalAvailableTime: Double?
  public var reversalCount = 0, reversalArmed = false, reversalFeasible = false
  public var projectedCrossrange = 0.0, rollRateLimit = 0.0
  public var limitingReason = "initializing"
  public init() {}
}
