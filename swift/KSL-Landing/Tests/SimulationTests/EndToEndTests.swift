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

  func testReducedLiftStillLandsUsingObservedAuthority() throws {
    let result = try SimulationEngine(scenario: .approach(), liftScale: 0.8).run()
    XCTAssertEqual(result.outcome, "complete")
    XCTAssertEqual(result.touchdown?.acceptable, true)
    XCTAssertLessThan(try XCTUnwrap(result.touchdown).pitch, radians(14))
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
    XCTAssertEqual(result.outcome, "aborted")
    XCTAssertNil(result.touchdown)
    XCTAssertTrue(result.violations.contains(where: { $0.kind == .insufficientEnergy }))
  }
}
