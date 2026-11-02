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
    XCTAssertEqual(together.courseProgress.status, .available)
    XCTAssertEqual(together.courseProgress.signedChange,
      first.courseProgress.signedChange+tail.courseProgress.signedChange, accuracy: 1e-12)
    XCTAssertEqual(together.courseProgress.absoluteChange,
      first.courseProgress.absoluteChange+tail.courseProgress.absoluteChange, accuracy: 1e-12)
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
    let exact = try TAEMCommandScheduleEvaluator(configuration: short, atmosphere: atmosphere,
      aerodynamics: aero).evaluate([.init(duration: dt/2, command: command)], from: seed)
    XCTAssertEqual(exact.end, .scheduleExhausted)
    let plan = TerminalPlan(identifier: 1, createdAt: row.state.time, spiral: nil, stations: [],
      assessment: .init(feasible: true, violations: [], touchdownSpeed: 0, minimumLiftMargin: 0,
        maximumBank: 0, maximumQ: 0, maximumLoad: 0, score: 0, uncertaintyFraction: 0))
    let terminal = TerminalTrajectoryEvaluator(configuration: short, atmosphere: atmosphere, aerodynamics: aero)
    let accounted = terminal.evaluate(plan: plan, from: exact.seed.state, controlSnapshot: nil,
      previousTarget: nil, lift: 1, drag: 1, commandPrefix: exact)
    XCTAssertEqual(accounted.evaluation?.end, .horizon)
    XCTAssertFalse(accounted.qualified)
    XCTAssertEqual(accounted.elapsed, exact.seed.state.time-seed.state.time)
    XCTAssertEqual(accounted.evaluation?.comparisonContext, exact.comparisonContext)
    XCTAssertEqual(accounted.evaluation?.aerodynamicWork, exact.accounting.aerodynamicWork)
    XCTAssertEqual(accounted.evaluation?.heightChange, exact.seed.state.position.length-seed.state.position.length)
    let invalid = terminal.evaluate(plan: plan, from: bounded.seed.state, controlSnapshot: nil,
      previousTarget: nil, lift: 1, drag: 1, commandPrefix: bounded)
    XCTAssertEqual(invalid.evaluation?.end, .unsupportedDynamics)
    XCTAssertEqual(invalid.evaluation?.status, .invalid)
    let wrongModel = TerminalTrajectoryEvaluator(configuration: short,
      atmosphere: try AtmosphericModelEngine(planet: c.planet), aerodynamics: aero)
      .evaluate(plan: plan, from: exact.seed.state, controlSnapshot: nil, previousTarget: nil,
        lift: 1, drag: 1, commandPrefix: exact)
    XCTAssertEqual(wrongModel.evaluation?.end, .unsupportedDynamics)
    // Empty command prefixes exercise the continuation wiring only. A zero
    // search horizon intentionally supplies no landing forecast or flight claim.
    var zeroCoverage = c
    zeroCoverage.guidance.maximumPredictionDuration = 0
    var joins: [VehicleState] = []
    let searched = try TAEMCommandSearch(configuration: zeroCoverage, atmosphere: atmosphere,
      aerodynamics: aero).search([
        .init(id: "first", topology: .incumbent, segments: []),
        .init(id: "second", topology: .protection, segments: [])], from: seed,
        maximumEvaluations: 2) { _, prefix in
          joins.append(prefix.seed.state); return plan
        }
    XCTAssertEqual(joins.count, 2)
    XCTAssertEqual(searched.trials[0].outcome?.evaluation?.comparisonContext,
      searched.trials[1].outcome?.evaluation?.comparisonContext)
    XCTAssertEqual(searched.trials[1].relation, .unassessed)
    XCTAssertEqual(searched.incumbentTrial, 0)
    var continuationCalls = 0
    let adaptive = try TAEMCommandSearch(configuration: zeroCoverage, atmosphere: atmosphere,
      aerodynamics: aero).refine(.init(id: "empty-wiring-only", topology: .incumbent, segments: []),
        from: seed, initialSteps: .init(alphaRadians: c.guidance.alphaSlewRate*c.guidance.controlInterval,
          bankRadians: c.guidance.bankSlewRate*c.guidance.controlInterval, durationSeconds: c.guidance.controlInterval),
        resolution: .init(alphaRadians: c.guidance.alphaSlewRate*c.guidance.controlInterval/2,
          bankRadians: c.guidance.bankSlewRate*c.guidance.controlInterval/2, durationSeconds: c.guidance.controlInterval/2),
        maximumEvaluations: 2) { _, _ in continuationCalls += 1; return plan }
    XCTAssertEqual(adaptive.end, .unassessed, "Missing terminal suffix cannot establish convergence")
    XCTAssertEqual(adaptive.trials.count, 1)
    XCTAssertEqual(continuationCalls, 1, "Repeated unchanged schedules must reuse evidence")
    XCTAssertEqual(adaptive.incumbentTrial, 0)
    XCTAssertTrue(searched.trials.allSatisfy { $0.outcome?.evaluation?.end == .horizon && $0.outcome?.qualified == false })

  }

  func testCommandSearchRetainsUnsupportedCandidatesWithoutInventingContinuations() throws {
    let r = try recorded(), c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let search = TAEMCommandSearch(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let seed = NativeCommandSeed(state: r.state, controls: nil, limiter: .init(previous: r.target))
    let candidates = [TAEMCommandCandidate(id: "incumbent", topology: .incumbent, segments: [
      .init(duration: c.guidance.controlInterval, command: .init(phase: .acquisition, target: r.target))]),
      .init(id: "protection", topology: .protection, segments: [
        .init(duration: c.guidance.controlInterval, command: .init(phase: .acquisition, target: r.target))])]
    var continuationCalled = false
    let result = try search.search(candidates, from: seed, maximumEvaluations: 2) { _, _ in
      continuationCalled = true; return nil
    }
    XCTAssertFalse(continuationCalled)
    XCTAssertEqual(result.end, .familyExhausted)
    XCTAssertEqual(result.trials.count, 2)
    XCTAssertNil(result.incumbentTrial)
    XCTAssertTrue(result.trials.allSatisfy { $0.prefix.end == .unsupportedDynamics && $0.outcome == nil })
    XCTAssertEqual(result.trials[0].prefix.comparisonContext, result.trials[1].prefix.comparisonContext)
    let truncated = try search.search(candidates, from: seed, maximumEvaluations: 1) { _, _ in nil }
    XCTAssertEqual(truncated.end, .candidateBudget)
    XCTAssertEqual(truncated.trials.count, 1)
    XCTAssertEqual(truncated.suppliedCandidates, 2)
    let cancelled = try search.search(candidates, from: seed, maximumEvaluations: 2,
      cancelled: { true }) { _, _ in nil }
    XCTAssertEqual(cancelled.end, .cancelled)
    XCTAssertTrue(cancelled.trials.isEmpty)
    XCTAssertThrowsError(try search.search([candidates[0], candidates[0]], from: seed,
      maximumEvaluations: 2) { _, _ in nil })
  }

  func testCommandRefinementPollChangesRequestsWithinRealBoundsWithoutResettingOrigin() throws {
    let r = try recorded(), c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let search = TAEMCommandSearch(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let seed = NativeCommandSeed(state: r.state, controls: nil, limiter: .init(previous: r.target))
    var target = r.target
    target.angleOfAttack = c.vehicle.maximumAngleOfAttack
    target.bank = c.vehicle.maximumBank
    target.speedbrakeFraction = 0.4
    let center = TAEMCommandCandidate(id: "bounded-center", topology: .right, segments: [
      .init(duration: c.guidance.controlInterval, command: .init(phase: .acquisition, target: target))])
    let steps = TAEMCommandRefinementSteps(alphaRadians: c.guidance.alphaSlewRate*c.guidance.controlInterval,
      bankRadians: c.guidance.bankSlewRate*c.guidance.controlInterval,
      durationSeconds: c.guidance.controlInterval/2)
    let poll = try search.refineOnce(center, from: seed, steps: steps, maximumEvaluations: 10) { _, _ in
      XCTFail("Missing original moment support must not be repaired by a continuation"); return nil
    }
    // Positive alpha/bank neighbors clip back to the center and are omitted.
    XCTAssertEqual(poll.end, .familyExhausted)
    XCTAssertEqual(poll.trials.count, 5)
    XCTAssertEqual(Set(poll.trials.map { $0.candidate.id }).count, 5)
    XCTAssertNil(poll.incumbentTrial)
    for trial in poll.trials {
      let segment = try XCTUnwrap(trial.candidate.segments.first)
      XCTAssertLessThanOrEqual(segment.command.target.angleOfAttack, c.vehicle.maximumAngleOfAttack)
      XCTAssertLessThanOrEqual(abs(segment.command.target.bank), c.vehicle.maximumBank)
      XCTAssertGreaterThan(segment.duration, 0)
      XCTAssertEqual(segment.command.target.speedbrakeFraction, target.speedbrakeFraction)
      XCTAssertEqual(trial.prefix.comparisonContext.originID, seed.originID)
      XCTAssertEqual(trial.prefix.end, .unsupportedDynamics)
      XCTAssertTrue(trial.prefix.steps.isEmpty)
    }
    let limited = try search.refineOnce(center, from: seed, steps: steps, maximumEvaluations: 1) { _, _ in nil }
    XCTAssertEqual(limited.end, .candidateBudget)
    XCTAssertEqual(limited.trials.count, 1)
    let resolution = TAEMCommandRefinementSteps(alphaRadians: steps.alphaRadians/2,
      bankRadians: steps.bankRadians/2, durationSeconds: steps.durationSeconds/2)
    let adaptive = try search.refine(center, from: seed, initialSteps: steps,
      resolution: resolution, maximumEvaluations: 10) { _, _ in nil }
    XCTAssertEqual(adaptive.end, .modelUnavailable)
    XCTAssertEqual(adaptive.trials.count, 1, "All branches lack the same original pitch evidence")
    let noBudget = try search.refine(center, from: seed, initialSteps: steps,
      resolution: resolution, maximumEvaluations: 0) { _, _ in
        XCTFail("A zero budget cannot start a continuation"); return nil
      }
    XCTAssertEqual(noBudget.end, .candidateBudget)
    XCTAssertTrue(noBudget.trials.isEmpty)
  }

}
