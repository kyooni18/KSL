import Foundation
import KSLCore
import KSLPhysics

/// Runtime comparison identity. The originating history UUID is shared only
/// by branches of the same immutable seed; configuration/start-state equality
/// also guards against accidentally comparing rebased tails. Model UUIDs are
/// instance identities, not empirical provenance or archived resource hashes.
public struct TerminalComparisonContext: Codable, Sendable, Equatable {
  public let originID: UUID
  public let startState: VehicleState
  public let configuration: MissionConfiguration
  public let atmosphereID: UUID, aerodynamicsID: UUID
  public let continuationPolicy: String
  public init(seed: NativeCommandSeed, configuration: MissionConfiguration,
              atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine,
              continuationPolicy: String = "native-tracker-approach-rollout-v1") {
    originID = seed.originID; startState = seed.state; self.configuration = configuration
    atmosphereID = atmosphere.instanceID; aerodynamicsID = aerodynamics.instanceID
    self.continuationPolicy = continuationPolicy
  }
}

public enum TerminalRecoveryRelation: String, Codable, Sendable {
  case challengerPreferred, incumbentPreferred, equivalent
  case contextMismatch, unassessed, incomparableOutcomes
}

/// Partial order on complete, consistently evaluated model outcomes. Unknown
/// suffixes have no total price. A tradeoff among irreversible failures has
/// no invented weighted score: preserve the incumbent and expose incomparability.
public enum TerminalRecoveryOrder {
  public static func compare(_ challenger: TerminalClosedLoopCase,
                             to incumbent: TerminalClosedLoopCase) -> TerminalRecoveryRelation {
    guard let a = challenger.evaluation, let b = incumbent.evaluation,
      let ac = a.comparisonContext, let bc = b.comparisonContext else { return .unassessed }
    guard ac == bc else { return .contextMismatch }
    let completeA = hasCompleteAccounting(challenger)
    let completeB = hasCompleteAccounting(incumbent)
    if completeA != completeB { return completeA ? .challengerPreferred : .incumbentPreferred }
    guard completeA else { return .unassessed }
    if challenger.qualified != incumbent.qualified {
      return challenger.qualified ? .challengerPreferred : .incumbentPreferred
    }
    if challenger.qualified {
      // All evaluated model criteria already passed. Precision within their
      // admissible set must not purchase arbitrary additional flight time.
      if challenger.elapsed == incumbent.elapsed { return .equivalent }
      return challenger.elapsed < incumbent.elapsed ? .challengerPreferred : .incumbentPreferred
    }
    // There is a narrow policy-free comparison among failed recoveries that
    // both reach an otherwise accepted contact and sustained runway stop:
    // lower peak AND accumulated q/load excursion, with no other failure.
    // This permits a better failed recovery without requiring qualification.
    // Crash/miss/attitude/terrain tradeoffs await full terminal-deficit pricing.
    let envelopeKinds: Set<FailureClassification> = [.excessiveDynamicPressure, .excessiveLoad]
    guard a.end == .sustainedStop, b.end == .sustainedStop,
      a.touchdown?.acceptable == true, b.touchdown?.acceptable == true,
      challenger.violations.allSatisfy({ envelopeKinds.contains($0.kind) }),
      incumbent.violations.allSatisfy({ envelopeKinds.contains($0.kind) }) else { return .incomparableOutcomes }
    let av = [a.maximumPressureExcess, a.pressureExcessIntegral, a.maximumLoadExcess, a.loadExcessIntegral]
    let bv = [b.maximumPressureExcess, b.pressureExcessIntegral, b.maximumLoadExcess, b.loadExcessIntegral]
    if av == bv { return .equivalent }
    if zip(av, bv).allSatisfy({ $0 <= $1 }) { return .challengerPreferred }
    if zip(bv, av).allSatisfy({ $0 <= $1 }) { return .incumbentPreferred }
    return .incomparableOutcomes
  }

  public static func hasCompleteAccounting(_ result: TerminalClosedLoopCase) -> Bool {
    result.evaluation?.status == .complete && validAccounting(result)
  }

  private static func validAccounting(_ result: TerminalClosedLoopCase) -> Bool {
    guard let e = result.evaluation, let context = e.comparisonContext,
      result.elapsed.isFinite, result.elapsed >= 0,
      e.state.time == context.startState.time+result.elapsed,
      e.state.position.isFinite, e.state.velocity.isFinite,
      [e.maximumPressureExcess, e.pressureExcessIntegral, e.maximumLoadExcess, e.loadExcessIntegral]
        .allSatisfy({ $0.isFinite && $0 >= 0 }),
      e.aerodynamicWork.isFinite, e.energyChange.isFinite, e.heightChange.isFinite else { return false }
    guard (try? e.state.validate(configuration: context.configuration)) != nil else { return false }
    if result.violations.contains(where: { $0.kind == .excessiveDynamicPressure }), e.maximumPressureExcess <= 0 { return false }
    if result.violations.contains(where: { $0.kind == .excessiveLoad }), e.maximumLoadExcess <= 0 { return false }
    switch e.end {
    case .modeledCrash: return e.state.contact == .crashed && !result.feasible
    case .sustainedStop:
      let frame = RunwayFrame(runway: context.configuration.runway, planet: context.configuration.planet)
      let p = frame.coordinates(of: e.state.position)
      return e.state.contact.isGrounded && e.state.wheelContact?.noseGear == true
        && e.state.horizontalSpeed < 0.5 // same existing sustained-stop criterion as evaluator
        && context.configuration.runway.contains(along: p.along, cross: p.cross)
        && e.touchdown.map { event in
          [event.time, event.along, event.cross, event.horizontalSpeed, event.verticalSpeed, event.bank, event.pitch].allSatisfy(\.isFinite)
            && event.time >= context.startState.time
            && e.state.time-event.time >= context.configuration.guidance.stopConfirmationDuration
        } == true
    default: return false
    }
  }
}

/// One already-executed comparison. Trial indices are local to a PlanningResult,
/// not globally unique candidate IDs. The first trial has no incumbent/relation.
/// Retaining the outcome preserves unknown suffixes and Pareto alternatives;
/// this record grants no plan admission or control ownership.
public struct TerminalRecoveryComparison: Codable, Sendable {
  public let trial: Int
  public let incumbentTrial: Int?
  public let relation: TerminalRecoveryRelation?
  public let outcome: TerminalClosedLoopCase
}
