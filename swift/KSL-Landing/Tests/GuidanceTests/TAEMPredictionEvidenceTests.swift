import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMPredictionEvidenceTests: XCTestCase {
  private struct RecordedSeed: Decodable {
    var configuration: MissionConfiguration
    var state: VehicleState
    let target: GuidanceTarget
  }
  private func recorded() throws -> RecordedSeed {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    return try JSONDecoder().decode(RecordedSeed.self, from: Data(contentsOf: url))
  }
  func testRecordedRCSHandoffUsesExcludedTorqueAndRejectsMissingDecomposition() throws {
    let r = try recorded(), c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    func branch(_ state: VehicleState) -> NativeCommandRollout {
      .init(seed: .init(state: state, controls: nil, limiter: .init(previous: r.target)),
        configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    }
    let rollout = branch(r.state)
    XCTAssertTrue(r.state.systems.rcs)
    XCTAssertFalse(rollout.pitchMomentSupported, "Measured torque alone cannot identify a missing moment balance")
    let step = try rollout.advance(.init(phase: .acquisition, target: r.target), duration: c.guidance.controlInterval)
    XCTAssertFalse(step.issued.rcs)
    XCTAssertFalse(step.after.systems.rcs)
    let expected = try XCTUnwrap(r.state.controlAuthority?.excludingRCS)
    XCTAssertEqual(step.after.controlAuthority?.minimumAcceleration, expected.minimumAcceleration)
    // Remove measurement metadata, not physical vehicle capability. Prediction
    // must report unknown instead of borrowing the enabled-RCS total.
    var missing = r.state
    missing.controlAuthority?.rcsExcludedPositiveTorque = nil
    missing.controlAuthority?.rcsExcludedNegativeTorque = nil
    let unsupported = branch(missing)
    XCTAssertThrowsError(try unsupported.advance(.init(phase: .acquisition, target: r.target),
      duration: c.guidance.controlInterval))
    XCTAssertEqual(unsupported.state.time, r.state.time)
    XCTAssertEqual(unsupported.limiter.previous, r.target)
  }

  func testUnidentifiedPlantAndMissingSuffixNeverQualifyAsTerminalOutcomes() throws {
    var r = try recorded()
    let plan = TerminalPlan(identifier: 1, createdAt: r.state.time, spiral: nil, stations: [],
      assessment: .init(feasible: true, violations: [], touchdownSpeed: 0, minimumLiftMargin: 0,
        maximumBank: 0, maximumQ: 0, maximumLoad: 0, score: 0, uncertaintyFraction: 0))
    let atmosphere = try AtmosphericModelEngine(planet: r.configuration.planet)
    let aero = try AerodynamicsEngine(referenceArea: r.configuration.vehicle.referenceArea)
    func evaluate() -> TerminalClosedLoopCase {
      TerminalTrajectoryEvaluator(configuration: r.configuration, atmosphere: atmosphere, aerodynamics: aero)
        .evaluate(plan: plan, from: r.state, controlSnapshot: nil, previousTarget: r.target, lift: 1, drag: 1)
    }
    let unknown = evaluate()
    XCTAssertFalse(unknown.qualified)
    XCTAssertEqual(unknown.evaluation?.status, .invalid)
    XCTAssertEqual(unknown.evaluation?.end, .unsupportedDynamics)
    XCTAssertEqual(unknown.evaluation?.state.time, r.state.time)
    let assessedPlan = TerminalPlan(identifier: plan.identifier, createdAt: plan.createdAt,
      spiral: nil, stations: [], assessment: plan.assessment, closedLoopCases: [unknown])
    let assessment = TAEMCaptureEvaluator.assess(.init(plan: assessedPlan, bestAdvisory: nil,
      evaluatedCandidates: 1, advisoryCounts: [:]), state: r.state, configuration: r.configuration)
    XCTAssertFalse(assessment.feasible)
    XCTAssertEqual(assessment.deficit, .evaluationUnassessed)
    XCTAssertTrue(EntryGuidanceEngine.holdNeutralReference(for: [assessment.deficit]))
    XCTAssertNil(EntryGuidanceEngine.unanimousEnergyDeficit([assessment.deficit]))
    // A zero search horizon omits the entire suffix. This changes search
    // coverage only and does not alter any physical plant parameter.
    r.configuration.guidance.maximumPredictionDuration = 0
    let incomplete = evaluate()
    XCTAssertFalse(incomplete.qualified)
    XCTAssertEqual(incomplete.evaluation?.status, .incomplete)
    XCTAssertEqual(incomplete.evaluation?.end, .horizon)
    XCTAssertTrue(incomplete.violations.contains { $0.kind == .timeout })
    let decoded = try JSONDecoder().decode(TerminalClosedLoopCase.self,
      from: JSONEncoder().encode(incomplete))
    XCTAssertEqual(decoded.evaluation?.end, .horizon)
    // An optimistic legacy flag must not override explicit missing-tail data.
    let optimistic = TerminalClosedLoopCase(liftMultiplier: 1, dragMultiplier: 1, feasible: true,
      elapsed: 0, touchdownSpeed: nil, touchdownSink: nil, touchdownPitch: nil, stoppedAlong: nil,
      maximumTrackingError: 0, violations: [], evidence: "incomplete-proposal",
      evaluation: incomplete.evaluation)
    XCTAssertFalse(optimistic.qualified)
  }
}
