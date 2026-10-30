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
