import Foundation
import KSLControl
import KSLCore
import KSLPhysics

/// Physical reason a state cannot be captured by TAEM/HAC guidance.
public enum TAEMCaptureDeficit: String, Codable, Sendable {
  case none
  /// Even the longest admissible HAC path arrives too fast.
  case excessiveEnergy
  /// Even the shortest admissible HAC path exhausts energy or lands too slow.
  case insufficientEnergy
  /// Required ground-track turn exceeds available lift/bank authority.
  case crossrangeBeyondTurningAuthority
  /// Paths intersect terrain or demand glide slopes outside the envelope.
  case insufficientAltitude
  /// Vehicle is heading away and no path can turn it back within authority.
  case adverseHeading
  /// Lift/trim or bank-rate authority insufficient for every candidate.
  case inadequateControlAuthority
  /// Dynamic pressure or load envelope violated by every candidate.
  case envelopeViolation
  /// No assessed complete terminal reference within the planning/entry region.
  /// Search coverage alone does not establish physical runway unreachability.
  case terminalRegionNotReached
  /// Energy/geometry admissible, but the closed-loop vehicle failed to stop.
  case closedLoopFailure
}

/// Reachable envelope of the HAC family from one state, in physical units.
/// Energy margins are energy height (m): positive excess margin means at
/// least one admissible path is long enough to dissipate the state's energy;
/// positive deficit margin means at least one is short enough to retain it.
public struct TAEMReachableEnvelope: Codable, Sendable {
  public var energyHeight = 0.0
  public var excessEnergyMargin = -Double.greatestFiniteMagnitude
  public var deficitEnergyMargin = -Double.greatestFiniteMagnitude
  public var shortestFeasiblePath: Double?
  public var longestFeasiblePath: Double?
  public var shortestExaminedPath: Double?
  public var longestExaminedPath: Double?
  /// Heading change to align with the selected runway, and the local turn
  /// rate from the current force/trim-constrained HAC survey (rad, rad/s).
  public var requiredHeadingChange = 0.0
  public var availableTurnRate = 0.0
  /// A legacy zero rate can mean unassessed; preserve that distinction.
  public var turnAuthorityAssessed: Bool?
  public var minimumRequiredPathLength: Double?, maximumSearchedPathLength: Double?
  public var searchHorizonExceeded: Bool?
  public var candidatesExamined = 0
  public var candidatesCompleted = 0
  public var candidatesFeasible = 0
  public var dominantViolations: [String: Int] = [:]
  public init() {}
  /// Smallest of the two energy margins; the capture "robustness" in metres.
  public var energyMargin: Double { min(excessEnergyMargin, deficitEnergyMargin) }
}

public struct TAEMCaptureAssessment: Codable, Sendable {
  public var feasible = false
  /// True only when a closed-loop route was flown through stop.
  public var certified = false
  public var deficit = TAEMCaptureDeficit.terminalRegionNotReached
  public var envelope = TAEMReachableEnvelope()
  public var time = 0.0
  public var altitude = 0.0
  public var mach = 0.0
  public var rangeToRunway = 0.0
  public var headingError = 0.0
  public var runwayHeadingError: Double? = nil
  public var planIdentifier: Int?
  public var planLength: Double?
  public var hacRadius: Double?
  public var hacTurnAngle: Double?
  public var hacSide: Int?
  public var plan: TerminalPlan?
  public var violations: [ConstraintViolation] = []
  public init() {}
  public var summary: String {
    feasible ? "capture \(certified ? "certified" : "feasible") margin=\(String(format: "%.0f", envelope.energyMargin))m"
      : "capture infeasible: \(deficit.rawValue)"
  }
}

/// Decides whether TAEM/MM305 can physically capture a state, using the same
/// dynamic-radius HAC family, energy pricer and closed-loop plant as MM305.
/// No individual threshold makes a state feasible: an admissible priced path
/// must exist, and certification additionally flies it to a stop.
public final class TAEMCaptureEvaluator {
  private let configuration: MissionConfiguration
  private let planner: HACPlanningEngine
  private let frame: RunwayFrame
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    planner = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }

  public func evaluate(_ state: VehicleState, certify: Bool = false,
                       controlSnapshot: FlightControlSnapshot? = nil,
                       previousTarget: GuidanceTarget? = nil,
                       context: HACFitContext? = nil) -> TAEMCaptureAssessment {
    let c = configuration
    let preview = planner.plan(from: state, allowDirect: context?.onSpiral != true, preview: true,
      controlSnapshot: controlSnapshot, previousTarget: previousTarget, context: context)
    var a = Self.assess(preview, state: state, configuration: c)
    guard a.feasible, certify else { return a }
    let closed = planner.plan(from: state, allowDirect: context?.onSpiral != true,
      controlSnapshot: controlSnapshot, previousTarget: previousTarget, context: context)
    if let plan = closed.plan {
      a.certified = plan.nominalQualified; Self.describe(plan, into: &a)
      if !a.certified {
        a.feasible = false; a.deficit = .closedLoopFailure
        a.violations = plan.closedLoopCases?.flatMap(\.violations) ?? plan.assessment.violations
      }
    } else {
      a.feasible = false; a.deficit = .closedLoopFailure
      a.violations = closed.closedLoopAdvisories?.flatMap(\.violations) ?? []
    }
    return a
  }

  /// Physical verdict on one HAC planning result (preview or closed-loop).
  public static func assess(_ result: PlanningResult, state: VehicleState,
                            configuration c: MissionConfiguration) -> TAEMCaptureAssessment {
    let v = c.vehicle, frame = RunwayFrame(runway: c.runway, planet: c.planet)
    var a = TAEMCaptureAssessment()
    let p = frame.coordinates(of: state.position)
    let g = c.planet.gravitationalParameter / state.position.length.squared
    let airspeed = max(1, state.aerodynamics.airspeed)
    a.time = state.time; a.altitude = p.height; a.mach = state.aerodynamics.mach
    a.rangeToRunway = frame.distance(from: state.position, to: frame.origin)
    a.headingError = wrapAngle(frame.bearing(from: state.position, to: frame.origin) - state.course)
    a.envelope.energyHeight = p.height + airspeed * airspeed / (2 * g)
    let runwayDirection = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    a.runwayHeadingError = wrapAngle(state.localFrame.heading(of: runwayDirection) - state.course)
    a.envelope.requiredHeadingChange = abs(a.runwayHeadingError ?? 0)
    // g*tan(bank) assumes a supported coordinated turn; it was not an
    // available-acceleration measurement for this descending, lift-limited shuttle.
    a.envelope.availableTurnRate = result.fit?.availableTurnRate ?? 0
    a.envelope.turnAuthorityAssessed = result.fit != nil
    let summary = result.envelope ?? .init()
    a.envelope.minimumRequiredPathLength = summary.minimumRequiredLength
    a.envelope.maximumSearchedPathLength = summary.pathLengthLimit
    if let required = summary.minimumRequiredLength, let limit = summary.pathLengthLimit {
      a.envelope.searchHorizonExceeded = required > limit
    }
    a.envelope.candidatesExamined = summary.examined
    a.envelope.candidatesCompleted = summary.completed
    a.envelope.candidatesFeasible = summary.feasible
    a.envelope.dominantViolations = summary.violationCounts
    a.envelope.shortestFeasiblePath = summary.shortestFeasibleLength
    a.envelope.longestFeasiblePath = summary.longestFeasibleLength
    a.envelope.shortestExaminedPath = summary.shortestExaminedLength
    a.envelope.longestExaminedPath = summary.longestExaminedLength
    // Energy-height margins share the same runway-gravity normalization as
    // EnergyManagementEngine.energyHeight, not Earth's standard gravity.
    let referenceGravity = c.planet.gravitationalParameter / frame.origin.squaredLength
    let gs = 2 * referenceGravity
    if let slow = summary.slowestTouchdown {
      a.envelope.excessEnergyMargin = (v.maximumTouchdownSpeed.squared - slow.squared) / gs
    }
    if let fast = summary.fastestTouchdown {
      a.envelope.deficitEnergyMargin = (fast.squared - (v.landingSpeed * EnergyManagementEngine.minimumTouchdownFraction).squared) / gs
    }
    guard let candidate = result.plan else {
      a.deficit = classify(summary, headingError: a.runwayHeadingError ?? a.headingError, envelope: a.envelope)
      a.violations = result.bestAdvisory?.violations ?? []
      if a.envelope.searchHorizonExceeded == true,
        let required = summary.minimumRequiredLength, let limit = summary.pathLengthLimit {
        a.violations.append(.init(.invalidGeometry,
          "Complete runway path exceeds the configured search horizon; physical reachability is unassessed",
          value: required, limit: limit))
      }
      if a.deficit == .none || summary.feasible > 0 {
        // Priced paths existed but none survived closed-loop flight.
        a.deficit = .closedLoopFailure
        a.violations = result.closedLoopAdvisories?.flatMap(\.violations) ?? a.violations
      }
      return a
    }
    a.feasible = candidate.assessment.feasible
    a.deficit = a.feasible ? .none : classify(summary, headingError: a.runwayHeadingError ?? a.headingError, envelope: a.envelope)
    a.violations = candidate.assessment.violations
    a.certified = candidate.nominalQualified
    describe(candidate, into: &a)
    return a
  }

  private static func describe(_ plan: TerminalPlan, into a: inout TAEMCaptureAssessment) {
    a.plan = plan; a.planIdentifier = plan.identifier
    a.planLength = plan.stations.last?.distance
    a.hacRadius = plan.spiral?.finalRadius
    a.hacTurnAngle = plan.spiral?.turnAngle
    a.hacSide = plan.spiral?.side
  }

  /// Attribute the failure to the mechanism that eliminated the family.
  private static func classify(_ s: TerminalEnvelopeSummary, headingError: Double,
                        envelope: TAEMReachableEnvelope) -> TAEMCaptureDeficit {
    // An empty searched family supplies no force/energy evidence. In
    // particular, a configured path-length ceiling is not physical unreachability.
    if s.examined == 0 { return .terminalRegionNotReached }
    if s.completed > 0 {
      // Energy alone decided the completed paths: compare the two extremes.
      if envelope.excessEnergyMargin > -Double.greatestFiniteMagnitude,
        envelope.excessEnergyMargin < 0 { return .excessiveEnergy }
      if envelope.deficitEnergyMargin > -Double.greatestFiniteMagnitude,
        envelope.deficitEnergyMargin < 0 { return .insufficientEnergy }
    }
    let exhausted = s.violationCounts[FailureClassification.insufficientEnergy.rawValue] ?? 0
    let excessive = s.violationCounts[FailureClassification.excessiveEnergy.rawValue] ?? 0
    // All examined paths ran out of energy. Low-speed lift and slew failures
    // on those same paths must not suppress MM304's lower-drag recovery branch.
    let liftFailures = s.violationCounts[FailureClassification.insufficientLift.rawValue] ?? 0
    let actuatorFailures = s.violationCounts[FailureClassification.actuatorAuthority.rawValue] ?? 0
    if s.completed == 0, max(liftFailures, actuatorFailures) >= s.examined {
      // A family with no completed path and universal force/trim rejection
      // cannot prove that changing drag alone would restore MM305 capture.
      return .inadequateControlAuthority
    }
    if s.completed == 0, exhausted >= s.examined, excessive == 0 { return .insufficientEnergy }
    func count(_ kinds: [FailureClassification]) -> Int {
      // Counts by kind can overlap. Their maximum is an evidenced lower bound
      // on affected candidates; their sum double-counts multi-cause failures.
      kinds.reduce(0) { max($0, s.violationCounts[$1.rawValue] ?? 0) }
    }
    let turning = count([.excessiveBank]), authority = count([.insufficientLift, .actuatorAuthority])
    let altitude = count([.terrainCollision, .invalidGeometry])
    let envelopeCount = count([.excessiveDynamicPressure, .excessiveLoad])
    let energy = count([.excessiveEnergy, .insufficientEnergy])
    let top = max(turning, authority, altitude, envelopeCount, energy)
    guard top > 0 else { return .terminalRegionNotReached }
    if top == energy {
      return (s.violationCounts[FailureClassification.excessiveEnergy.rawValue] ?? 0)
        >= (s.violationCounts[FailureClassification.insufficientEnergy.rawValue] ?? 0)
        ? .excessiveEnergy : .insufficientEnergy
    }
    if top == turning { return abs(headingError) > .pi / 2 ? .adverseHeading : .crossrangeBeyondTurningAuthority }
    if top == altitude {
      // A glide steeper than the envelope means too high for the distance.
      return (s.violationCounts[FailureClassification.invalidGeometry.rawValue] ?? 0)
        > (s.violationCounts[FailureClassification.terrainCollision.rawValue] ?? 0)
        ? .excessiveEnergy : .insufficientAltitude
    }
    if top == envelopeCount { return .envelopeViolation }
    return .inadequateControlAuthority
  }
}

extension Double {
  fileprivate var squared: Double { self * self }
}
