import Foundation

/// Operator settings use radians and SI units after validation/mapping.
public struct EntryConfiguration: Codable, Sendable, Equatable {
  public var nominalAlpha = radians(18)
  public var estimatedBallisticCoefficient = 700.0
  public var estimatedLiftToDrag = 0.4
  public var minimumLegDuration = 24.0
  public var rollRate = radians(4.5)
  public var rollAcceleration = radians(1.8)
  public var maximumSideslip = radians(8)
  public var terminalMinimumAltitude = 22_000.0
  public var terminalMaximumAltitude = 30_000.0
  public var terminalTargetMach = 2.5
  public var terminalMachHalfWidth = 0.35
  public var terminalTargetAltitude = 25_000.0
  public var terminalTargetRange = 96_500.0
  public var handoffAlong = -8000.0
  public var handoffCross = 0.0
  public var handoffRadius = 1000.0
  public var headingHalfWidth = radians(30)
  public init() {}
  public func validate() throws {
    guard nominalAlpha >= 0, nominalAlpha < .pi / 2, estimatedBallisticCoefficient > 0,
      estimatedLiftToDrag > 0, minimumLegDuration >= 0, rollRate > 0, rollAcceleration > 0,
      maximumSideslip > 0, maximumSideslip < .pi / 2,
      terminalMinimumAltitude > 0, terminalMaximumAltitude > terminalMinimumAltitude,
      terminalTargetAltitude >= terminalMinimumAltitude, terminalTargetAltitude <= terminalMaximumAltitude,
      terminalTargetMach > terminalMachHalfWidth, terminalMachHalfWidth > 0,
      terminalTargetRange > 0, handoffRadius > 0, headingHalfWidth > 0, headingHalfWidth <= .pi / 2
    else { throw FlightError.invalidConfiguration("Invalid MM304/MM305 settings") }
    _ = try JSONEncoder().encode(self)
  }
}
