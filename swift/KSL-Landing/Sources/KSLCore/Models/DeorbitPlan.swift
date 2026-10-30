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
  public var passFraction: Double { Double(passedCases)/15 }
  public var hasCompleteStressMatrix: Bool {
    cases.count == 15 && Set(cases.map(\.index)) == Set(0..<15)
  }
  public var fullRouteQualified: Bool {
    hasCompleteStressMatrix && nominalPassed && unsafeCases == 0
      && passFraction >= configuration.deorbitSettings.minimumRobustPassFraction
  }
  public var executionQualified: Bool { fullRouteQualified && modelLimitations.isEmpty }
  /// Explicit reduced assurance accepts model assessments as advisories, not
  /// as evidence that the real vehicle will succeed or remain within limits.
  /// A complete report is still required for traceability. Actual command,
  /// propulsion, orbital freshness and configured flight limits are separate.
  public var permitsExplicitReducedAssurance: Bool {
    hasCompleteStressMatrix
  }
  public init(identifier: String = UUID().uuidString, source: VehicleState,
    configuration: MissionConfiguration, propulsion: PropulsionTelemetry, window: DeorbitWindow,
    cases: [DeorbitStressCase], modelLimitations: [String], evaluatedWindows: Int) {
    self.identifier = identifier; createdAt = source.time; self.source = source
    self.configuration = configuration; self.propulsion = propulsion; self.window = window
    self.cases = cases; self.modelLimitations = modelLimitations; self.evaluatedWindows = evaluatedWindows
  }
  public func authorize(identifier: String, reducedAssurance: Bool) throws {
    guard identifier == self.identifier else { throw FlightError.invalidCommand("Plan identity differs from the reviewed plan") }
    guard executionQualified || (reducedAssurance && permitsExplicitReducedAssurance) else {
      throw FlightError.infeasible("Deorbit plan is not executable: nominal=\(nominalPassed), full-route cases=\(passedCases)/15, unsafe=\(unsafeCases), model limitations=\(modelLimitations.joined(separator: "; "))")
    }
  }
}
