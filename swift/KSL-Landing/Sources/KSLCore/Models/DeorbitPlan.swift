import Foundation

/// An orbital screening result. It carries no landing/execution certificate.
public struct DeorbitWindow: Codable, Sendable {
  public let burnStart: Double
  /// Retrograde-projected thrust impulse (m/s), including in legacy records.
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
  /// Signed orbital-normal impulse, m/s; nil preserves legacy fixed-angle burns.
  public var normalDeltaV: Double? = nil
  /// Total commanded propulsive impulse, rather than retrograde projection.
  public var propulsiveDeltaV:Double {
    if let normalDeltaV {return abs(normalDeltaV)+abs(deltaV)}
    return abs(deltaV)/cos(normalAngle)
  }
  public var closing: Bool { abs(entryCourseError) < .pi/2 }
  public init(burnStart: Double, deltaV: Double, duration: Double, normalAngle: Double,
    periapsisAltitude: Double, afterBurn: OrbitalState, entryInterface: OrbitalState,
    entryRange: Double, entrySpeed: Double, entryFlightPath: Double, entryCourseError: Double,
    crossrange: Double, miss: Double, normalDeltaV: Double? = nil) {
    self.burnStart = burnStart; self.deltaV = deltaV; self.duration = duration
    self.normalAngle = normalAngle; self.periapsisAltitude = periapsisAltitude
    self.afterBurn = afterBurn; self.entryInterface = entryInterface
    self.entryRange = entryRange; self.entrySpeed = entrySpeed; self.entryFlightPath = entryFlightPath
    self.entryCourseError = entryCourseError; self.crossrange = crossrange; self.miss = miss
    self.normalDeltaV = normalDeltaV
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

/// Downstream MM304 evidence retained across runway comparisons. No landing certificate.
public struct DeorbitEntryScreen: Codable, Sendable {
  /// Nil in historical records. A stopped atmospheric prefix is not TAEM delivery.
  public let deliveryAssessed: Bool?
  public let feasible: Bool
  public let altitudeRejected: Bool
  public let machRejected: Bool
  public let captureMiss: Double
  public let terminalAlong: Double
  public let terminalCross: Double
  public let terminalAltitude: Double
  public let terminalMach: Double
  public init(feasible: Bool, altitudeRejected: Bool, machRejected: Bool, captureMiss: Double,
    terminalAlong: Double, terminalCross: Double, terminalAltitude: Double, terminalMach: Double,
    deliveryAssessed: Bool? = nil) {
    self.deliveryAssessed=deliveryAssessed
    self.feasible=feasible;self.altitudeRejected=altitudeRejected;self.machRejected=machRejected
    self.captureMiss=captureMiss;self.terminalAlong=terminalAlong;self.terminalCross=terminalCross
    self.terminalAltitude=terminalAltitude;self.terminalMach=terminalMach
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
  public var entryScreen: DeorbitEntryScreen? = nil
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
    guard hasNominalEvaluation,unsafeCases==0,cases[0].burnCompleted,
      propulsion.dryMass>0,propulsion.dryMass<source.mass else {return false}
    let available=propulsion.specificImpulse*9.80665*log(source.mass/propulsion.dryMass)
    return window.propulsiveDeltaV<=configuration.deorbitSettings.burnDeltaVLimit
      && window.propulsiveDeltaV<available
  }
  public var permitsExplicitReducedAssurance: Bool { hasExecutableEntryWindow }
  public init(identifier: String = UUID().uuidString, source: VehicleState,
    configuration: MissionConfiguration, propulsion: PropulsionTelemetry, window: DeorbitWindow,
    cases: [DeorbitStressCase], modelLimitations: [String], evaluatedWindows: Int,
    entryScreen: DeorbitEntryScreen? = nil) {
    self.identifier = identifier; createdAt = source.time; self.source = source
    self.configuration = configuration; self.propulsion = propulsion; self.window = window
    self.cases = cases; self.modelLimitations = modelLimitations; self.evaluatedWindows = evaluatedWindows
    self.entryScreen = entryScreen
  }
  public func authorize(identifier: String, reducedAssurance: Bool) throws {
    guard identifier == self.identifier else { throw FlightError.invalidCommand("Plan identity differs from the reviewed plan") }
    try propulsion.validate(mass:source.mass)
    let available=propulsion.specificImpulse*9.80665*log(source.mass/propulsion.dryMass)
    guard window.propulsiveDeltaV.isFinite,window.propulsiveDeltaV<available,
      window.propulsiveDeltaV<=configuration.deorbitSettings.burnDeltaVLimit else {
      throw FlightError.infeasible("Deorbit impulse exceeds the recorded OMS propellant budget")
    }
    // Authorization requires a fully screened, physically admissible entry
    // window, not a provably successful prediction of a later controller.
    // Keep the uncertainty visible; do not fabricate a passed rollout case.
    guard hasExecutableEntryWindow else {
      throw FlightError.infeasible("OMS-to-entry window failed physical checks: nominal evaluated=\(hasNominalEvaluation), unsafe=\(unsafeCases), burn complete=\(cases.first?.burnCompleted == true)")
    }
  }
}
