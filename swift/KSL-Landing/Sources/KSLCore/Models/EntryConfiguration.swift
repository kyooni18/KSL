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
  /// Runway distance of MM304's NEP-overhead delivery: the NEP station, the
  /// diameter of a minimum-radius HAC beside it, and the delivery tolerance.
  /// Derived from HAC geometry; the operator precision aim is not a cap here.
  public var nepOverheadCaptureRange: Double {
    guidance.finalDistance + 2*guidance.minimumHACRadius + entrySettings.handoffRadius
  }
  /// MM304's energy objective over the NEP: the configured TAEM window.
  /// (A subsonic HAC-acquisition objective was tried offline: MM304's
  /// range-linear drag budget then spends the energy the subsonic glide needs
  /// and the 72 km forecast ended 65 km short at Mach 0.36.)
  public var mm304DeliveryAltitude: Double { entrySettings.terminalTargetAltitude }
  public var mm304DeliveryMach: Double { entrySettings.terminalTargetMach }
  public var mm304DeliveryMinimumAltitude: Double { entrySettings.terminalMinimumAltitude }
  /// Lower edge of MM304's forecast/survey window.
  public var mm304DeliveryFloor: Double { mm304DeliveryMinimumAltitude - entrySettings.handoffRadius }
  /// MM304's irrevocable handoff deadline: the floor of its energy window.
  /// Flying MM304's law below it was checked offline from the 2244 72 km
  /// state: AoA 20-30 deg collapsed airspeed to ~100 m/s by 12 km while still
  /// 18 km out, so a later handoff needs a subsonic MM304 law, not a lower gate.
  public var mm304HandoffDeadlineAltitude: Double { mm304DeliveryMinimumAltitude }
  /// Runway-frame along coordinate of the NEP-overhead delivery on the
  /// configured handoff crossrange. Single target for MM304, the deorbit
  /// window ranking and runway selection (replaces mm304HandoffAlongTrack).
  public var nepOverheadCaptureAlong: Double {
    let r = nepOverheadCaptureRange, cross = entrySettings.handoffCross
    return -sqrt(max(0, r*r - cross*cross))
  }
}
