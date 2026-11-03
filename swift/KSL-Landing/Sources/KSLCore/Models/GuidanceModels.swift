import Foundation

public enum MissionPhase: String, Codable, Sendable {
  case orbitalCoast, burnSetup, deorbitBurn, entryCoast
  case entry, acquisition, headingAlignment, approach, preflare, flare, touchdown, rollout,
    complete, abort
}

public enum FailureClassification: String, Codable, Sendable {
  case insufficientEnergy, excessiveEnergy, insufficientLift, excessiveBank, excessiveLoad
  case excessiveDynamicPressure, terrainCollision, invalidGeometry, actuatorAuthority
  case telemetry, contactQuality, runwayExcursion, modelUncertainty, timeout
}

public struct ConstraintViolation: Codable, Sendable, Equatable {
  public let kind: FailureClassification
  public let message: String
  public let value: Double
  public let limit: Double
  public init(
    _ kind: FailureClassification, _ message: String, value: Double = 0, limit: Double = 0
  ) {
    self.kind = kind
    self.message = message
    self.value = value
    self.limit = limit
  }
}

public struct GuidanceTarget: Codable, Sendable, Equatable {
  public var angleOfAttack: Double
  public var bank: Double
  public var desiredSpeed: Double
  public var desiredHeight: Double
  public var desiredFlightPath: Double
  public var gearDown: Bool
  /// Requested split-rudder deployment. nil is stowed in new commands and
  /// absent in historical records; this is not the legacy action-group flag.
  public var speedbrakeFraction: Double?
  public init(
    angleOfAttack: Double = 0, bank: Double = 0, desiredSpeed: Double = 0,
    desiredHeight: Double = 0, desiredFlightPath: Double = 0, gearDown: Bool = false,
    speedbrakeFraction: Double? = nil
  ) {
    self.angleOfAttack = angleOfAttack
    self.bank = bank
    self.desiredSpeed = desiredSpeed
    self.desiredHeight = desiredHeight
    self.desiredFlightPath = desiredFlightPath
    self.gearDown = gearDown
    self.speedbrakeFraction = speedbrakeFraction
  }
}

/// Actual reference-computer history at one command boundary. The request is
/// after device allocation and before slew limiting; neither is achieved attitude.
/// Together with the record's phase/state/configuration and preceding FCS
/// diagnostics this permits exact limiter replay, including internal rates.
public struct GuidanceCommandTrace: Codable, Sendable, Equatable {
  public let requested: GuidanceTarget
  public let before: GuidanceTargetLimiter
  public let after: GuidanceTargetLimiter
  public init(requested: GuidanceTarget, before: GuidanceTargetLimiter, after: GuidanceTargetLimiter) {
    self.requested = requested; self.before = before; self.after = after
  }
}

public struct GuidanceSolution: Codable, Sendable {
  public let phase: MissionPhase
  public let target: GuidanceTarget
  public var violations: [ConstraintViolation]
  public let planID: Int?
  public let pathRemaining: Double
  public var diagnostic: String
  /// Absent in legacy records and commands that bypass the airborne limiter.
  public var commandRecovery: CommandRecoveryDiagnostics?
  public var targetHistory: GuidanceCommandTrace?
  public var entry: EntryDiagnostics?
  public var approach: ApproachDiagnostics?
  public var admission: TerminalAdmissionDiagnostics?
  public var capture: CaptureHandoffDiagnostics?
  public init(
    phase: MissionPhase, target: GuidanceTarget, violations: [ConstraintViolation] = [],
    planID: Int? = nil, pathRemaining: Double = 0, diagnostic: String = ""
  ) {
    self.phase = phase
    self.target = target
    self.violations = violations
    self.planID = planID
    self.pathRemaining = pathRemaining
    self.diagnostic = diagnostic
  }
}

public struct TrajectorySample: Codable, Sendable {
  public let time: Double
  public let position: Vector3
  public let velocity: Vector3
  public let angleOfAttack: Double
  public let bank: Double
  public let dynamicPressure: Double
  public let loadFactor: Double
  public init(
    time: Double, position: Vector3, velocity: Vector3, angleOfAttack: Double, bank: Double,
    dynamicPressure: Double, loadFactor: Double
  ) {
    self.time = time
    self.position = position
    self.velocity = velocity
    self.angleOfAttack = angleOfAttack
    self.bank = bank
    self.dynamicPressure = dynamicPressure
    self.loadFactor = loadFactor
  }
}

public struct TrajectoryPrediction: Codable, Sendable {
  public let samples: [TrajectorySample]
  public let terminalState: VehicleState
  public let violations: [ConstraintViolation]
  public let duration: Double
  public let reachedTerminalRegion: Bool
  public init(
    samples: [TrajectorySample], terminalState: VehicleState, violations: [ConstraintViolation],
    duration: Double, reachedTerminalRegion: Bool
  ) {
    self.samples = samples
    self.terminalState = terminalState
    self.violations = violations
    self.duration = duration
    self.reachedTerminalRegion = reachedTerminalRegion
  }
}

/// Default TAEM command owner; absent in older recordings.
public struct CommandRecoveryDiagnostics: Codable, Sendable {
  public var mode: String
  public var proposalID: String?
  public var candidateID: String?
  public var evaluatedCandidates: Int
  public var forecastComplete: Bool
  public var runwayQualified: Bool
  public var sourceTime: Double?
  public var forecastFailure: String? = nil
  public var referencePlanID: Int? = nil
  public var referenceStationDistance: Double? = nil
  public var referencePathError: Double? = nil
  public var requestedForce: Vector2? = nil
  public var modeledForce: Vector2? = nil
  public var forceResidual: Vector2? = nil
  public init(mode: String, proposalID: String? = nil, candidateID: String? = nil,
              evaluatedCandidates: Int = 0, forecastComplete: Bool = false,
              runwayQualified: Bool = false, sourceTime: Double? = nil) {
    self.mode = mode; self.proposalID = proposalID; self.candidateID = candidateID
    self.evaluatedCandidates = evaluatedCandidates; self.forecastComplete = forecastComplete
    self.runwayQualified = runwayQualified; self.sourceTime = sourceTime
  }
}
