import Foundation

/// An orbital screening result. It carries no landing/execution certificate.
public struct DeorbitWindow: Codable, Sendable {
  public let burnStart: Double
  public let deltaV: Double
  public let duration: Double
  public let normalAngle: Double
  public let periapsisAltitude: Double
  public let afterBurn: OrbitalState
  public let entryInterface: OrbitalState
  public let entryRange: Double
  public let entrySpeed: Double
  public let entryFlightPath: Double
  public let entryCourseError: Double
  public let crossrange: Double
  public let miss: Double
  public var closing: Bool { abs(entryCourseError) < .pi/2 }
  public init(burnStart: Double, deltaV: Double, duration: Double, normalAngle: Double,
    periapsisAltitude: Double, afterBurn: OrbitalState, entryInterface: OrbitalState,
    entryRange: Double, entrySpeed: Double, entryFlightPath: Double, entryCourseError: Double,
    crossrange: Double, miss: Double) {
    self.burnStart = burnStart; self.deltaV = deltaV; self.duration = duration
    self.normalAngle = normalAngle; self.periapsisAltitude = periapsisAltitude
    self.afterBurn = afterBurn; self.entryInterface = entryInterface
    self.entryRange = entryRange; self.entrySpeed = entrySpeed; self.entryFlightPath = entryFlightPath
    self.entryCourseError = entryCourseError; self.crossrange = crossrange; self.miss = miss
  }
}

public struct DeorbitStressCase: Codable, Sendable {
  public let index: Int
  public let passed: Bool
  public let hardUnsafe: Bool
  public let burnCompleted: Bool
  public let entryFlightPath: Double?
  public let maximumDynamicPressure: Double
  public let maximumLoad: Double
  public let touchdownAlong: Double?
  public let touchdownCross: Double?
  public let touchdownSink: Double?
  public let touchdownPitch: Double?
  public let stoppedAlong: Double?
  public let violations: [ConstraintViolation]
  public let evidence: String
  public init(index: Int, passed: Bool, hardUnsafe: Bool, burnCompleted: Bool,
    entryFlightPath: Double?, maximumDynamicPressure: Double, maximumLoad: Double,
    touchdownAlong: Double?, touchdownCross: Double?, touchdownSink: Double?, touchdownPitch: Double?,
    stoppedAlong: Double?, violations: [ConstraintViolation], evidence: String = "offline-native-controller-and-provisional-plant") {
    self.index = index; self.passed = passed; self.hardUnsafe = hardUnsafe; self.burnCompleted = burnCompleted
    self.entryFlightPath = entryFlightPath; self.maximumDynamicPressure = maximumDynamicPressure
    self.maximumLoad = maximumLoad; self.touchdownAlong = touchdownAlong; self.touchdownCross = touchdownCross
    self.touchdownSink = touchdownSink; self.touchdownPitch = touchdownPitch; self.stoppedAlong = stoppedAlong
    self.violations = violations; self.evidence = evidence
  }
}

public struct DeorbitPlan: Codable, Sendable {
  public let identifier: String
  public let createdAt: Double
  public let source: VehicleState
  public let configuration: MissionConfiguration
  public let propulsion: PropulsionTelemetry
  public let window: DeorbitWindow
  public let cases: [DeorbitStressCase]
  public let modelLimitations: [String]
  public let evaluatedWindows: Int
  public var passedCases: Int { cases.filter(\.passed).count }
  public var unsafeCases: Int { cases.filter(\.hardUnsafe).count }
  public var nominalPassed: Bool { cases.first(where: { $0.index == 0 })?.passed == true }
  /// One complete nominal orbit-to-stop evaluation is the only tested case.
  /// It is still an offline model result, not independent flight evidence.
  public var hasNominalEvaluation: Bool {
    cases.count == 1 && cases[0].index == 0
  }
  public var fullRouteQualified: Bool {
    hasNominalEvaluation && nominalPassed && unsafeCases == 0
  }
  public var executionQualified: Bool { fullRouteQualified && modelLimitations.isEmpty }
  /// The orbital OMS-to-entry safety envelope is distinct from predicting a
  /// qualified TAEM capture or rollout. A failed terminal simulation must not
  /// prohibit an otherwise physically admissible, fully evaluated entry test.
  /// This is one standard flight-planning policy, not an experimental mode.
  public var hasExecutableEntryWindow: Bool {
    hasNominalEvaluation && unsafeCases == 0 && cases[0].burnCompleted
  }
  public var permitsExplicitReducedAssurance: Bool { hasExecutableEntryWindow }
  public init(identifier: String = UUID().uuidString, source: VehicleState,
    configuration: MissionConfiguration, propulsion: PropulsionTelemetry, window: DeorbitWindow,
    cases: [DeorbitStressCase], modelLimitations: [String], evaluatedWindows: Int) {
    self.identifier = identifier; createdAt = source.time; self.source = source
    self.configuration = configuration; self.propulsion = propulsion; self.window = window
    self.cases = cases; self.modelLimitations = modelLimitations; self.evaluatedWindows = evaluatedWindows
  }
  public func authorize(identifier: String, reducedAssurance: Bool) throws {
    guard identifier == self.identifier else { throw FlightError.invalidCommand("Plan identity differs from the reviewed plan") }
    // Authorization requires a fully screened, physically admissible entry
    // window, not a provably successful prediction of a later controller.
    // Keep the uncertainty visible; do not fabricate a passed rollout case.
    guard hasExecutableEntryWindow else {
      throw FlightError.infeasible("OMS-to-entry window failed physical checks: nominal evaluated=\(hasNominalEvaluation), unsafe=\(unsafeCases), burn complete=\(cases.first?.burnCompleted == true)")
    }
  }
}
