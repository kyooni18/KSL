import Foundation

/// Current-flight environment observations. These never alter the prior-flight
/// aerodynamic data book or identify a control-surface moment coefficient.
public struct EnvironmentCalibration: Codable, Sendable, Equatable {
  public var densityScale = 1.0
  public var soundSpeedScale = 1.0
  public var observedAt: Double?
  public var evidence: EvidenceKind = .unavailable
  public init() {}
}

public struct CalibrationObservationQuality: Codable, Sendable {
  public var passiveEligible = false
  public var trajectoryEligible = false
  public var forceEvidence: EvidenceKind = .unavailable
  public var stallFraction = 0.0
  public var stallEvidence: EvidenceKind = .provisional
  public var informationIncrement = 0.0
  public var informationBlend = 0.0
  public var rejectionReasons: [String] = []
  public init() {}
}

public struct AerodynamicRegimeCalibration: Codable, Sendable {
  public let machAnchor: Double
  public var ballisticCoefficient: Double
  public var liftToDrag: Double
  public var confidence = 0.0
  public init(machAnchor: Double, ballisticCoefficient: Double, liftToDrag: Double) {
    self.machAnchor = machAnchor; self.ballisticCoefficient = ballisticCoefficient
    self.liftToDrag = liftToDrag
  }
}

public struct CalibrationDiagnostics: Codable, Sendable {
  public var observation = CalibrationObservationQuality()
  public var environment = EnvironmentCalibration()
  public var regimes: [AerodynamicRegimeCalibration] = []
  public var currentRegime = 0
  public var acceptedPassiveSamples = 0, rejectedPassiveSamples = 0
  public var acceptedTrajectorySamples = 0
  public var trajectoryConfidence = 0.0, bankEffectiveness = 1.0
  public var altitudeResidual = 0.0, speedResidual = 0.0, rangeResidual = 0.0
  public var forecastEpoch: Double?
  public var autoApplyRequested = false
  public var certifiedProfileChanged = false
  public init() {}
}
