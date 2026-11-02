import XCTest
import KSLCore
import KSLPhysics
import KSLControl
@testable import KSLGuidance

final class TAEMPredictionEvidenceTests: XCTestCase {
  private struct RecordedSeed: Decodable {
    var configuration: MissionConfiguration
    var state: VehicleState
    let target: GuidanceTarget
    let balanceHistory: [BalanceRecord]
  }
  private struct BalanceRecord: Decodable {
    let state: VehicleState, phase: MissionPhase, target: GuidanceTarget
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
  func testMeasuredMomentEvidenceExpiresWithoutRefreshFromModeledObservations() throws {
    let r = try recorded(), c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let controller = FlightController(configuration: c)
    let history = r.balanceHistory
    var previousTime: Double?
    for row in history.dropLast() {
      let dt = previousTime.map { row.state.time-$0 } ?? c.guidance.controlInterval
      _ = try controller.command(state: row.state, solution: .init(phase: row.phase, target: row.target), dt: dt)
      previousTime = row.state.time
    }
    let next = try XCTUnwrap(history.last)
    XCTAssertNotNil(controller.snapshot.predictionPitchTrim(state: next.state))
    let branch = NativeCommandRollout(seed: .init(state: next.state, controls: controller.snapshot,
      limiter: .init(previous: next.target)), configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    XCTAssertTrue(branch.pitchMomentSupported)
    XCTAssertEqual(branch.pitchMomentSupport, .localBalance)
    // This is native history/coverage bookkeeping on the genuine STS plant,
    // not a historical-target replay or a counterfactual performance claim.
    for _ in 0..<21 {
      _ = try branch.advance(.init(phase: next.phase, target: next.target), duration: c.guidance.controlInterval)
    }
    XCTAssertFalse(branch.pitchMomentSupported)
    let continued = NativeCommandRollout(seed: branch.seed, configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    XCTAssertEqual(continued.pitchMomentSupport, .outsideEvidence)
    XCTAssertFalse(continued.pitchMomentSupported, "A prediction checkpoint must not refresh measured moment evidence")
    let prediction = TrajectoryPropagationEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    var requestedUnsupportedCommand = false
    let unsupported = prediction.predict(from: r.state, duration: c.guidance.pathLookaheadTime,
      terminalSpeed: 0) { _ in
        requestedUnsupportedCommand = true
        return .init(alpha: r.target.angleOfAttack, bank: r.target.bank)
      }
    XCTAssertFalse(requestedUnsupportedCommand)
    XCTAssertEqual(unsupported.duration, 0)
    XCTAssertTrue(unsupported.violations.contains { $0.kind == .modelUncertainty })
  }

  func testCommandSchedulePreservesHistoryAcrossSegmentsAndReturnsOnlyAPrefix() throws {
    let r = try recorded(), c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let controller = FlightController(configuration: c)
    var previous: Double?
    for row in r.balanceHistory.dropLast() {
      _ = try controller.command(state: row.state, solution: .init(phase: row.phase, target: row.target),
        dt: previous.map { row.state.time-$0 } ?? c.guidance.controlInterval)
      previous = row.state.time
    }
    let row = try XCTUnwrap(r.balanceHistory.last)
    let seed = NativeCommandSeed(state: row.state, controls: controller.snapshot,
      limiter: .init(previous: row.target))
    let command = GuidanceSolution(phase: row.phase, target: row.target)
    let dt = c.guidance.controlInterval
    let evaluator = TAEMCommandScheduleEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let together = try evaluator.evaluate([.init(duration: dt, command: command),
      .init(duration: dt/2, command: command)], from: seed)
    let first = try evaluator.evaluate([.init(duration: dt, command: command)], from: seed)
    let tail = try evaluator.evaluate([.init(duration: dt/2, command: command)], from: first.seed)
    XCTAssertEqual(together.end, .scheduleExhausted)
    XCTAssertEqual(together.steps.count, 2)
    XCTAssertEqual(together.seed.originID, seed.originID)
    XCTAssertEqual(together.seed.state, tail.seed.state)
    XCTAssertEqual(together.seed.limiter, tail.seed.limiter)
    XCTAssertEqual(together.steps.last?.issued, tail.steps.last?.issued)
    XCTAssertEqual(together.distance, first.distance+tail.distance, accuracy: 1e-9)
    let unsupported = try evaluator.evaluate([.init(duration: dt, command: command)],
      from: .init(state: r.state, controls: nil, limiter: .init(previous: r.target)))
    XCTAssertEqual(unsupported.end, .unsupportedDynamics)
    XCTAssertTrue(unsupported.steps.isEmpty)
    let cancelled = try evaluator.evaluate([.init(duration: dt, command: command)], from: seed, cancelled: { true })
    XCTAssertEqual(cancelled.end, .cancelled)
    XCTAssertTrue(cancelled.steps.isEmpty)
    XCTAssertThrowsError(try evaluator.evaluate([.init(duration: .nan, command: command)], from: seed))
    // Search coverage only; actual vehicle and plant are unchanged.
    var short = c
    short.guidance.maximumPredictionDuration = dt/2
    let bounded = try TAEMCommandScheduleEvaluator(configuration: short, atmosphere: atmosphere,
      aerodynamics: aero).evaluate([.init(duration: dt, command: command)], from: seed)
    XCTAssertEqual(bounded.end, .durationCoverage)
    XCTAssertEqual(bounded.steps.count, 1)
    XCTAssertEqual(bounded.seed.state.time, seed.state.time+dt/2)
  }

}
