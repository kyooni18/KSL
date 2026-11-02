import Foundation

/// One standard kRPC ControlSurface handle. The API represents the first
/// ModuleControlSurface on a part; a split rudder's second module is not
/// implicitly measured. Angle is actual motion, not a deflection override or
/// requested deploy angle. All angles/rates are radians and radians/second.
public struct ControlSurfaceObservation: Codable, Sendable, Equatable {
  public let surfaceID: UInt64, partID: UInt64
  public let partName: String
  public let measuredAt: Double
  public let deflectionAngle: Double
  public let controlRange: Double?, actuatorSpeed: Double?, authorityFraction: Double?
  public let deployed: Bool?, deployAngle: Double?
  public let pitchEnabled: Bool?, rollEnabled: Bool?, yawEnabled: Bool?
  public let evidence: EvidenceKind
  public init(surfaceID: UInt64, partID: UInt64, partName: String, measuredAt: Double,
              deflectionAngle: Double, controlRange: Double? = nil, actuatorSpeed: Double? = nil,
              authorityFraction: Double? = nil, deployed: Bool? = nil, deployAngle: Double? = nil,
              pitchEnabled: Bool? = nil, rollEnabled: Bool? = nil, yawEnabled: Bool? = nil,
              evidence: EvidenceKind = .measured) {
    self.surfaceID = surfaceID; self.partID = partID; self.partName = partName; self.measuredAt = measuredAt
    self.deflectionAngle = deflectionAngle; self.controlRange = controlRange; self.actuatorSpeed = actuatorSpeed
    self.authorityFraction = authorityFraction; self.deployed = deployed; self.deployAngle = deployAngle
    self.pitchEnabled = pitchEnabled; self.rollEnabled = rollEnabled; self.yawEnabled = yawEnabled
    self.evidence = evidence
  }
  public var isCoherent: Bool {
    measuredAt.isFinite && deflectionAngle.isFinite && !partName.isEmpty
      && (controlRange.map { $0.isFinite && $0 >= 0 } ?? true)
      && (actuatorSpeed.map { $0.isFinite && $0 >= 0 } ?? true)
      && (authorityFraction.map { $0.isFinite && (0...1).contains($0) } ?? true)
      && (deployAngle.map { $0.isFinite } ?? true)
  }
}
