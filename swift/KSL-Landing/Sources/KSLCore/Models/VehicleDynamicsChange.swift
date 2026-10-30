import Foundation

/// Detection thresholds describe telemetry sensitivity, not a validated flight
/// envelope. Compare against the last accepted baseline so slow redistribution
/// also invalidates fitted trim and predictions once it becomes material.
public struct VehicleDynamicsMonitor: Sendable {
  private var baseline: VehicleState?
  public private(set) var revision: UInt64 = 0
  public private(set) var lastChange: VehicleDynamicsChange?
  public init() {}
  @discardableResult public mutating func observe(_ state: VehicleState) -> VehicleDynamicsChange? {
    guard let prior = baseline else { baseline = state; return nil }
    var reasons: [String] = []
    if let old = prior.vesselConfiguration, let new = state.vesselConfiguration,
      old.revision != new.revision || old.partCount != new.partCount || old.anchorPart != new.anchorPart {
      reasons.append("part-topology-changed")
    }
    if abs(state.mass-prior.mass) > 0.01*prior.mass { reasons.append("mass-changed") }
    if let a = prior.controlAuthority?.inertia, let b = state.controlAuthority?.inertia,
      abs(a.x-b.x) > 0.01*a.x || abs(a.y-b.y) > 0.01*a.y || abs(a.z-b.z) > 0.01*a.z {
      reasons.append("inertia-changed")
    }
    if let a = prior.vesselConfiguration, let b = state.vesselConfiguration,
      a.anchorPart == b.anchorPart, a.anchorPart != nil,
      let p = a.centerOfMassFromAnchor, let q = b.centerOfMassFromAnchor,
      (q-p).length > 0.01 { reasons.append("center-of-mass-shifted") }
    if let a = prior.vesselConfiguration?.inertiaTensorFRD,
      let b = state.vesselConfiguration?.inertiaTensorFRD, a.count == 9, b.count == 9 {
      let difference = sqrt(zip(a,b).reduce(0) { $0+pow($1.0-$1.1,2) })
      let size = sqrt(a.reduce(0) { $0+$1*$1 })
      if difference > 0.01*size { reasons.append("inertia-tensor-changed") }
    }
    guard !reasons.isEmpty else {
      // Adopt newly available channels without forgetting the cumulative mass
      // baseline. Otherwise a first packet lacking CoM disables shift detection
      // for the rest of the flight.
      if baseline?.vesselConfiguration == nil { baseline?.vesselConfiguration = state.vesselConfiguration }
      if baseline?.controlAuthority == nil { baseline?.controlAuthority = state.controlAuthority }
      return nil
    }
    revision &+= 1
    let change = VehicleDynamicsChange(revision: revision, time: state.time, reasons: reasons,
      previousMass: prior.mass, currentMass: state.mass,
      inertiaEvidence: state.controlAuthority?.evidence ?? .unavailable,
      centerOfMassEvidence: state.vesselConfiguration?.centerOfMassEvidence ?? .unavailable)
    lastChange = change; baseline = state
    return change
  }
}
public struct VehicleDynamicsChange: Codable, Sendable, Equatable {
  public let revision: UInt64
  public let time: Double
  public let reasons: [String]
  public let previousMass: Double
  public let currentMass: Double
  public let inertiaEvidence: EvidenceKind
  public let centerOfMassEvidence: EvidenceKind
}
