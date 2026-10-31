import Foundation

public enum EvidenceKind: String, Codable, Sendable {
  case measured, identified, provisional, unavailable
}

/// All vectors use body FRD ordering: roll, pitch, yaw. Torques are N m and
/// inertia is kg m²; these are actuator moments, never lift/drag coefficients.
public struct ControlAuthorityTelemetry: Codable, Sendable, Equatable {
  public var positiveTorque: Vector3
  public var negativeTorque: Vector3
  public var inertia: Vector3
  public var evidence: EvidenceKind
  /// Optional actuator provenance; legacy archives omit it.
  public var actuatorSources: String?
  /// Same-snapshot active surface and wheel moments without RCS. These are
  /// needed to predict an RCS-off atmospheric handoff from an RCS-on orbit.
  public var rcsExcludedPositiveTorque: Vector3?
  public var rcsExcludedNegativeTorque: Vector3?
  public init(positiveTorque: Vector3, negativeTorque: Vector3, inertia: Vector3,
              evidence: EvidenceKind = .measured) {
    self.positiveTorque = positiveTorque
    self.negativeTorque = negativeTorque
    self.inertia = inertia
    self.evidence = evidence
    actuatorSources = nil
    rcsExcludedPositiveTorque = nil
    rcsExcludedNegativeTorque = nil
  }
  public var excludingRCS: Self? {
    guard let positive = rcsExcludedPositiveTorque, let negative = rcsExcludedNegativeTorque,
      positive.isFinite, negative.isFinite else { return nil }
    var result = self
    result.positiveTorque = positive; result.negativeTorque = negative
    result.actuatorSources = "aerodynamic-surfaces+active-powered-reaction-wheels"
    return result.isValid ? result : nil
  }
  public var acceleration: Vector3 {
    guard isValid else { return .zero }
    return Vector3(max(abs(positiveTorque.x), abs(negativeTorque.x)),
                   max(abs(positiveTorque.y), abs(negativeTorque.y)),
                   max(abs(positiveTorque.z), abs(negativeTorque.z)))
      .divided(componentwise: inertia)
  }
  public var minimumAcceleration: Vector3 {
    guard isValid else { return .zero }
    return Vector3(min(abs(positiveTorque.x), abs(negativeTorque.x)),
                   min(abs(positiveTorque.y), abs(negativeTorque.y)),
                   min(abs(positiveTorque.z), abs(negativeTorque.z))).divided(componentwise: inertia)
  }
  public var isValid: Bool {
    positiveTorque.isFinite && negativeTorque.isFinite && inertia.isFinite
      && inertia.x > 0 && inertia.y > 0 && inertia.z > 0
  }
}

public struct WheelContactTelemetry: Codable, Sendable, Equatable {
  /// nil means unavailable, not false. A vessel-situation flag is not wheel evidence.
  public var mainGear: Bool?
  public var noseGear: Bool?
  public var wheelCount: Int
  public var evidence: EvidenceKind?
  public init(mainGear: Bool? = nil, noseGear: Bool? = nil, wheelCount: Int = 0,
              evidence: EvidenceKind? = nil) {
    self.mainGear = mainGear; self.noseGear = noseGear; self.wheelCount = wheelCount
    self.evidence = evidence
  }
}

public struct RolloutDiagnostics: Codable, Sendable {
  public let mainContact: Bool
  public let noseContact: Bool
  public let brakeInhibited: Bool
  public let derotationRate: Double
  public let stoppingDistance: Double?
  public let remainingRunway: Double
  public let contactEvidence: EvidenceKind
  public init(mainContact: Bool, noseContact: Bool, brakeInhibited: Bool,
              derotationRate: Double, stoppingDistance: Double?, remainingRunway: Double,
              contactEvidence: EvidenceKind) {
    self.mainContact = mainContact; self.noseContact = noseContact
    self.brakeInhibited = brakeInhibited; self.derotationRate = derotationRate
    self.stoppingDistance = stoppingDistance; self.remainingRunway = remainingRunway
    self.contactEvidence = contactEvidence
  }
}

public struct ControlDiagnostics: Codable, Sendable {
  public var pitchSaturated = false, rollSaturated = false, yawSaturated = false
  public var pitchError = 0.0, bankError = 0.0
  public var quaternionError = Vector3.zero
  public var relativeRate = Vector3.zero
  public var aerodynamicRate = Vector3.zero
  public var bodyRate = Vector3.zero
  public var authority = Vector3.zero
  public var authorityConfidence = Vector3.zero
  public var reportedAuthority = Vector3.zero
  public var authorityEvidence: EvidenceKind = .unavailable
  public var pitchTrim = 0.0, rollTrim = 0.0
  public var pitchFeedforward: Double?
  public var trimAlphaCeiling: Double?
  public var trimEvidence: EvidenceKind?
  public var pitchBandwidthScale = 0.7
  public var pitchResponseSeconds: Double?
  public var rollResponseSeconds: Double?
  public var recoveryRateFirst = false
  public var touchdownPitchGuardActive: Bool?
  public var rollout: RolloutDiagnostics?
  public var rawDemand = Vector3.zero
  public var appliedDemand = Vector3.zero
  public var sampleInterval = 0.0
  public var betaRollGain = 0.0, betaYawGain = 0.0, betaConfidence = 0.0
  public init() {}
}

public struct ControlLoopTiming: Codable, Sendable {
  public var simulationInterval: Double
  public var acquisitionSeconds: Double
  public var computationSeconds: Double
  public var commandSeconds: Double
  public var wallInterval: Double
  public var missedDeadlines: UInt64
  public init(simulationInterval: Double, acquisitionSeconds: Double, computationSeconds: Double,
              commandSeconds: Double, wallInterval: Double, missedDeadlines: UInt64) {
    self.simulationInterval = simulationInterval; self.acquisitionSeconds = acquisitionSeconds
    self.computationSeconds = computationSeconds; self.commandSeconds = commandSeconds
    self.wallInterval = wallInterval; self.missedDeadlines = missedDeadlines
  }
}
