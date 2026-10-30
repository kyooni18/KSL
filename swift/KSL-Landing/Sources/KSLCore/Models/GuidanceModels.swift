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
  public init(
    angleOfAttack: Double = 0, bank: Double = 0, desiredSpeed: Double = 0,
    desiredHeight: Double = 0, desiredFlightPath: Double = 0, gearDown: Bool = false
  ) {
    self.angleOfAttack = angleOfAttack
    self.bank = bank
    self.desiredSpeed = desiredSpeed
    self.desiredHeight = desiredHeight
    self.desiredFlightPath = desiredFlightPath
    self.gearDown = gearDown
  }
}

public struct GuidanceSolution: Codable, Sendable {
  public let phase: MissionPhase
  public let target: GuidanceTarget
  public var violations: [ConstraintViolation]
  public let planID: Int?
  public let pathRemaining: Double
  public var diagnostic: String
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
