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
  /// Operator-selected MM304 handoff radius. Separate from the search envelope.
  public static let defaultCaptureTargetRange = 5_000.0
  public var captureTargetRange: Double? = Self.defaultCaptureTargetRange
  public var captureRange: Double { captureTargetRange ?? Self.defaultCaptureTargetRange }
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
      terminalTargetRange > 0, captureRange > 0, captureRange <= terminalTargetRange, handoffRadius > 0, headingHalfWidth > 0, headingHalfWidth <= .pi / 2
    else { throw FlightError.invalidConfiguration("Invalid MM304/MM305 settings") }
    _ = try JSONEncoder().encode(self)
  }
}

// Deliver beside the compact HAC, rather than at the outer search radius.
extension MissionConfiguration {
  public var terminalCaptureRange: Double {
    min(entrySettings.captureRange, guidance.finalDistance
      + 2*guidance.minimumHACRadius + entrySettings.handoffRadius)
  }
  /// Runway distance of MM304's delivery: the configured TAEM interface
  /// (guidance.taemInterfaceRange). Shuttle TAEM starts there on Mach and
  /// manages the remaining energy with its own ground track (JSC-16522).
  public var taemInterfaceRange: Double { entrySettings.terminalTargetRange }
  /// MM304's energy objective: the configured TAEM interface window.
  public var mm304DeliveryAltitude: Double { entrySettings.terminalTargetAltitude }
  public var mm304DeliveryMach: Double { entrySettings.terminalTargetMach }
  public var mm304DeliveryMinimumAltitude: Double { entrySettings.terminalMinimumAltitude }
  /// Lower edge of MM304's forecast/survey window.
  public var mm304DeliveryFloor: Double { mm304DeliveryMinimumAltitude - entrySettings.handoffRadius }
  /// MM304's irrevocable handoff deadline: the floor of its energy window.
  /// The nominal handoff is the TAEM interface Mach; this only bounds it.
  public var mm304HandoffDeadlineAltitude: Double { mm304DeliveryMinimumAltitude }
  /// Runway-frame along coordinate of the TAEM interface on the configured
  /// handoff crossrange. Single target for MM304, the deorbit window ranking
  /// and runway selection.
  public var taemInterfaceAlong: Double {
    let r = taemInterfaceRange, cross = entrySettings.handoffCross
    return -sqrt(max(0, r*r - cross*cross))
  }
}
