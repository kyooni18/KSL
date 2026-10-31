import KSLCore
import KSLSimulation
import XCTest

final class EndToEndTests: XCTestCase {
  func testEntrySpiralTouchdownAndRollout() throws {
    let result = try SimulationEngine(scenario: .entry(headingError: 8)).run()
    XCTAssertEqual(result.outcome, "complete")
    XCTAssertTrue(result.phases.contains("headingAlignment"))
    XCTAssertTrue(result.phases.contains("flare"))
    XCTAssertEqual(result.finalState.contact, .stopped)
    XCTAssertLessThan(result.finalState.horizontalSpeed, 0.5)
    XCTAssertEqual(result.touchdown?.acceptable, true)
    let touchdown = try XCTUnwrap(result.touchdown)
    XCTAssertLessThan(abs(touchdown.cross), 5)
    XCTAssertLessThan(abs(touchdown.verticalSpeed), 3)
    XCTAssertEqual(result.evidence, "offline-model-only")
  }

  func testModerateTenPercentLiftLossStillLands() throws {
    let result = try SimulationEngine(scenario: .approach(), liftScale: 0.90).run()
    XCTAssertEqual(result.outcome, "complete")
    XCTAssertEqual(result.touchdown?.acceptable, true)
    XCTAssertLessThan(try XCTUnwrap(result.touchdown).pitch, radians(14))
  }

  func testExtremeTwentyPercentLiftLossReportsContactFailure() throws {
    // -20% is a stress/failure-boundary probe, not a required nominal landing.
    // Preserve the actual independent safety limits and failure evidence.
    let result = try SimulationEngine(scenario: .approach(), liftScale: 0.80).run()
    XCTAssertEqual(result.evidence, "offline-model-only")
    XCTAssertEqual(result.outcome, "crashed")
    XCTAssertEqual(result.touchdown?.acceptable, false)
    XCTAssertTrue(result.violations.contains(where: { $0.kind == .contactQuality }))
  }

  func testApproachStepRefinement() throws {
    let a = try SimulationEngine(scenario: .approach()).run(physicsStep: 0.02)
    let b = try SimulationEngine(scenario: .approach()).run(physicsStep: 0.01)
    XCTAssertEqual(a.outcome, "complete")
    XCTAssertEqual(b.outcome, "complete")
    let first = try XCTUnwrap(a.touchdown)
    let second = try XCTUnwrap(b.touchdown)
    XCTAssertEqual(first.along, second.along, accuracy: 15)
    XCTAssertEqual(first.horizontalSpeed, second.horizontalSpeed, accuracy: 1)
    XCTAssertEqual(first.verticalSpeed, second.verticalSpeed, accuracy: 0.2)
  }

  func testLowEnergyDoesNotInventSuccessfulLanding() throws {
    var scenario = SimulationScenario.approach(
      along: -120000, height: 1000, speed: 70, flightPath: -20)
    scenario.maximumDuration = 10
    let result = try SimulationEngine(scenario: scenario).run()
    // An unreachable runway is an advisory, not permission to abandon the
    // airborne vehicle. Continue recovery without inventing a landing.
    XCTAssertEqual(result.outcome, "timedOut")
    XCTAssertNil(result.touchdown)
    XCTAssertGreaterThan(result.constraintOccurrences[FailureClassification.insufficientEnergy.rawValue] ?? 0, 0)
    XCTAssertFalse(result.phases.contains("complete"))
  }
}
