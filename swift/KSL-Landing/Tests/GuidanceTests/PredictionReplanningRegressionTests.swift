import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class PredictionReplanningRegressionTests: XCTestCase {
  private func entryState(_ c: MissionConfiguration) -> VehicleState {
    var a = AerodynamicState()
    a.airspeed = 2100; a.angleOfAttack = radians(2); a.measuredForces = true
    let position = Vector3(c.planet.radius + 72_000, 0, 0)
    let velocity = Vector3(-60, 2100, 0)
    return VehicleState(time: 100, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity,
        alpha: a.angleOfAttack, bank: 0), aerodynamics: a)
  }

  func testForecastRolloutKeepsFeedbackWithoutLaunchingAnotherForecast() throws {
    let c = MissionConfiguration()
    let engine = EntryGuidanceEngine(configuration: c, atmosphere: try .init(),
      aerodynamics: try .init(), permitsReplanning: false)
    var state = entryState(c)
    XCTAssertNil(engine.request(from: state))
    // This normally invokes synchronous planning. A rollout must still move
    // the entry attitude while suppressing recursive full-trajectory searches.
    let first = engine.update(state)
    XCTAssertGreaterThan(first.angleOfAttack, state.aerodynamics.angleOfAttack)
    state.time += c.guidance.entryReplanInterval * 3
    XCTAssertNil(engine.request(from: state))
    let later = engine.update(state)
    XCTAssertGreaterThan(later.angleOfAttack, first.angleOfAttack)
    XCTAssertEqual(engine.candidateCount, 0)
    XCTAssertEqual(engine.diagnostics.energyAllocationMode, "preentry-attitude")
  }

  func testLiveDefaultStillRequestsEntryReplanning() throws {
    let c = MissionConfiguration()
    let engine = EntryGuidanceEngine(configuration: c, atmosphere: try .init(),
      aerodynamics: try .init())
    XCTAssertNotNil(engine.request(from: entryState(c)))
  }

  func testSuppressingEntryRecursionDoesNotSuppressTerminalSearch() throws {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: -30_000, cross: 0, height: 25_000)
    let velocity = frame.direction(at: .init(-30_000, 0), tangent: .init(1, 0))*700
    var state = VehicleState(time: 100, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity,
        alpha: radians(20), bank: 0))
    state.aerodynamics.airspeed = 700
    state.aerodynamics.mach = 2.3
    state.aerodynamics.dynamicPressure = 3500
    let mission = LandingMissionController(configuration: c, atmosphere: try .init(),
      aerodynamics: try .init(), externalPlanning: true, entryReplanning: false)
    let request = try XCTUnwrap(mission.planningRequest(from: state, controls: nil))
    XCTAssertNotNil(request.terminalState)
    XCTAssertNil(request.entry)
  }
}
