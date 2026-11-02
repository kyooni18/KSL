import XCTest
import KSLCore
import KSLPhysics
import KSLControl
@testable import KSLGuidance

/// Unmodified states/commands from the Nov 2 orbital return. These checks
/// exercise demonstrated defects; they are not new flight continuations.
final class TerminalFlightRegressionTests: XCTestCase {
  private struct Fixture: Decodable {
    let configuration: MissionConfiguration
    let samples: [Sample]
  }
  private struct Sample: Decodable {
    let label: String
    let state: VehicleState
    let target: GuidanceTarget
    let control: ControlDiagnostics
    let phase: MissionPhase
  }
  private func fixture() throws -> Fixture {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "terminal-flight-20261102",
      withExtension: "json", subdirectory: "Fixtures"))
    return try JSONDecoder().decode(Fixture.self, from: Data(contentsOf: url))
  }
  private func sample(_ label: String, in f: Fixture) throws -> Sample {
    try XCTUnwrap(f.samples.first { $0.label == label })
  }

  func testRecordedTAEMRecoveryDoesNotAuthorizeVerticalGlide() throws {
    let f = try fixture(), c = f.configuration
    let s = try sample("acquisition", in: f).state
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let nep = try XCTUnwrap(NominalEnergyPointSolver(configuration: c,
      atmosphere: atmosphere, aerodynamics: aero).solve(from: s))
    let engine = TAEMGuidanceEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let target = engine.update(s, nep: nep, liftGain: 1, dragGain: 1,
      attainableAlphaCeiling: nil, pitchResponseSeconds: nil, rollResponseSeconds: nil)
    XCTAssertGreaterThanOrEqual(target.desiredFlightPath, -c.guidance.outerGlideAngle)
    XCTAssertLessThanOrEqual(abs(target.bank), c.vehicle.maximumBank)
    XCTAssertTrue(engine.status.descentPlanFits)
    // The ~33 km old route could not lose ~25 km inside this glide envelope.
    let height = RunwayFrame(runway: c.runway, planet: c.planet).coordinates(of: s.position).height
    XCTAssertGreaterThanOrEqual(engine.status.range, (height-nep.height)/tan(c.guidance.outerGlideAngle))
    print("TAEM recorded handoff: planned range \(engine.status.range) m; glide \(degrees(target.desiredFlightPath)) deg; force check \(engine.status.descentCheckFailure ?? "necessaryChecksPassed")")
  }

  func testRecordedRapidBankRequestsUseTerminalResponseHorizon() throws {
    let f = try fixture(), c = f.configuration
    let rows = f.samples.filter { $0.label == "bank-history" }
    let first = try XCTUnwrap(rows.first)
    var limiter = GuidanceTargetLimiter(previous: first.target)
    var previous = first
    var limited = first.target
    let rateLimit = c.guidance.terminalRollRate ?? c.entrySettings.rollRate
    for row in rows.dropFirst() {
      let dt = row.state.time-previous.state.time
      let oldBank = limited.bank
      limited = limiter.update(row.target, phase: row.phase, configuration: c,
        dt: dt, dynamicPressure: row.state.aerodynamics.dynamicPressure,
        rollResponseSeconds: row.control.rollResponseSeconds,
        controlAuthority: row.state.controlAuthority)
      XCTAssertLessThanOrEqual(abs(limited.bank-oldBank), rateLimit*dt+1e-12)
      XCTAssertLessThanOrEqual(abs(limited.bank), c.vehicle.maximumBank)
      previous = row
    }
    let recordedTravel = abs(try XCTUnwrap(rows.last).target.bank-first.target.bank)
    let limitedTravel = abs(limited.bank-first.target.bank)
    XCTAssertLessThan(limitedTravel, recordedTravel)
    print("Recorded bank interval: \(degrees(recordedTravel)) deg requested, \(degrees(limitedTravel)) deg revised over \(previous.state.time-first.state.time) s")
  }

  func testNormalFinalOwnsSpeedbrakeWithoutAnExperimentalSwitchOrInventedCdA() throws {
    let f = try fixture(), c = f.configuration
    let s = try sample("approach", in: f).state
    XCTAssertNil(c.vehicle.speedbrakeFullDeploymentDragArea)
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true, initialPhase: .approach)
    let solution = try mission.update(s)
    XCTAssertGreaterThan(solution.target.speedbrakeFraction ?? 0, 0)
    XCTAssertLessThanOrEqual(solution.target.speedbrakeFraction ?? 0, 1)
    let reserve = TerminalBrakeAuthority.availableLift(s, configuration: c, aerodynamics: aero)
    XCTAssertTrue(TerminalBrakeEnvelope.permits(s, configuration: c, availableLiftAcceleration: reserve))
    let late = try sample("late-float", in: f).state
    let lateReserve = TerminalBrakeAuthority.availableLift(late, configuration: c, aerodynamics: aero)
    XCTAssertFalse(TerminalBrakeEnvelope.permits(late, configuration: c, availableLiftAcceleration: lateReserve))
  }

  func testFloatClosesTheActualTouchdownAimWithinSinkLimit() throws {
    let f = try fixture(), c = f.configuration
    let controller = TerminalFlareController(configuration: c, aerodynamics: try .init())
    let start = try sample("preflare", in: f)
    _ = controller.update(start.state, liftEffectiveness: 1,
      responseSeconds: start.control.pitchResponseSeconds ?? 3,
      maximumAlpha: c.vehicle.maximumLandingAngleOfAttack)
    let late = try sample("late-float", in: f)
    let reference = controller.update(late.state, liftEffectiveness: 1,
      responseSeconds: late.control.pitchResponseSeconds ?? 3,
      maximumAlpha: c.vehicle.maximumLandingAngleOfAttack)
    XCTAssertTrue(reference.arcActive)
    XCTAssertEqual(reference.desiredSink, c.vehicle.maximumTouchdownSink, accuracy: 1e-10)
  }

  func testRolloutUsesFreshSupportAndObservedStoppingWork() throws {
    let f = try fixture()
    var c = f.configuration
    c.guidance.rolloutParachutes = nil // missing setting uses the normal policy
    let controller = RolloutController(configuration: c)
    let main = try sample("touchdown", in: f).state
    let contact = controller.command(state: main, dt: 0.1)
    XCTAssertEqual(contact.wheelBrake, 0)
    XCTAssertEqual(contact.deployParachutes, true)
    XCTAssertNil(controller.diagnostics?.stoppingDistance)
    let nose = try sample("rollout", in: f).state
    XCTAssertEqual(controller.command(state: nose, dt: 0.1).wheelBrake, 1)
    XCTAssertNil(controller.diagnostics?.stoppingDistance)
    var previous = nose.time
    for row in f.samples.filter({ $0.label == "ground-history" }) {
      _ = controller.command(state: row.state, dt: row.state.time-previous)
      previous = row.state.time
    }
    let originalStop = try XCTUnwrap(sample("rollout", in: f).control.rollout?.stoppingDistance)
    XCTAssertGreaterThan(try XCTUnwrap(controller.diagnostics?.stoppingDistance), originalStop)
    print("Recorded rollout observed-work stop: \(controller.diagnostics?.stoppingDistance ?? -1) m; runway remaining \(controller.diagnostics?.remainingRunway ?? -1) m")
    // A previously contacted nose cannot authorize brakes on a later bounce.
    let bounce = controller.command(state: main, dt: 0.1)
    XCTAssertEqual(bounce.wheelBrake, 0)
    XCTAssertNil(controller.diagnostics?.stoppingDistance)
  }
}
