import Foundation

/// A topology epoch and a live mass-distribution observation. The anchor uses
/// kRPC Part.reference_frame axes, not the aircraft's FRD axes. A relative CoM
/// vector must never be interpreted as a body-axis torque lever without first
/// transforming the part frame.
public struct VesselConfigurationTelemetry: Codable, Sendable, Equatable {
  public let revision: UInt64
  public let partCount: Int
  public let anchorPart: UInt64?
  public let centerOfMassFromAnchor: Vector3?
  public let centerOfMassEvidence: EvidenceKind
  /// Symmetric 3x3, row-major, about current CoM, in body FRD axes.
  /// nil is unavailable; diagonal moments are not a substitute for the tensor.
  public var inertiaTensorFRD: [Double]?
  public let measuredAt: Double
  public init(revision: UInt64, partCount: Int, anchorPart: UInt64? = nil,
    centerOfMassFromAnchor: Vector3? = nil,
    centerOfMassEvidence: EvidenceKind = .unavailable, measuredAt: Double) {
    self.revision = revision; self.partCount = partCount; self.anchorPart = anchorPart
    self.centerOfMassFromAnchor = centerOfMassFromAnchor
    self.centerOfMassEvidence = centerOfMassEvidence; self.measuredAt = measuredAt
    inertiaTensorFRD = nil
  }
  public var isCoherent: Bool {
    partCount > 0 && measuredAt.isFinite &&
    (centerOfMassFromAnchor == nil || centerOfMassFromAnchor!.isFinite) &&
    (centerOfMassEvidence != .measured || centerOfMassFromAnchor != nil) &&
    (inertiaTensorFRD == nil || (inertiaTensorFRD!.count == 9 && inertiaTensorFRD!.allSatisfy(\.isFinite)))
  }
}
