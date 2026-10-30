import Foundation
import KSLCore

public struct HACFitContext: Sendable {
  public var previousSpiral: SpiralHAC?
  public var previousPlanID: Int?
  public var onSpiral = false
  public var liftScale = 1.0
  public var dragScale = 1.0
  public var rollResponseSeconds: Double?
  public var pitchResponseSeconds: Double?
  public var trimAlphaCeiling: Double?
  public init() {}
}

public struct HACFitDiagnostics: Codable, Sendable {
  public var stateTime: Double
  public var previousPlanID: Int?
  public var continuingSpiral: Bool
  public var fittedRemainingAngle: Double?
  public var availableBank: Double
  public var availableTurnRate: Double
  public var turningRadius: Double
  public var rollResponseSeconds: Double
  public var joinDistance: Double
  public var energyHeight: Double
  public var energyPricedDistance: Double
  public var liftScale: Double
  public var dragScale: Double
}
