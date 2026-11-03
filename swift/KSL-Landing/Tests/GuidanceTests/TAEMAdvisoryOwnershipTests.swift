import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMAdvisoryOwnershipTests: XCTestCase {
  func testActualUnqualifiedAdvisoryDrivesLocalCurveWithoutClaimingCompleteForecast() throws {
    struct Recorded: Decodable {
      let configuration: MissionConfiguration, state: VehicleState, advisoryPlan: TerminalPlan
    }
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-advisory-reference-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    let r = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    let c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    XCTAssertFalse(r.advisoryPlan.assessment.feasible)
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true, initialPhase: .acquisition)
    let result = PlanningResult(plan: r.advisoryPlan, bestAdvisory: r.advisoryPlan.assessment,
      evaluatedCandidates: 1, advisoryCounts: [:])
    mission.accept(.init(time: r.advisoryPlan.createdAt, entry: nil, terminal: result), state: r.state)
    let referenceTracker = PathTrackingEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let expected = referenceTracker.lateralDemand(r.state, plan: r.advisoryPlan)
    let command = try mission.update(r.state)
    let diagnostics = try XCTUnwrap(command.commandRecovery)
    XCTAssertEqual(diagnostics.mode, "local-advisory-path-no-complete-forecast")
    XCTAssertEqual(diagnostics.referencePlanID, r.advisoryPlan.identifier)
    XCTAssertEqual(diagnostics.referenceStationDistance, expected.reference.distance)
    XCTAssertEqual(try XCTUnwrap(diagnostics.requestedForce).y, expected.acceleration, accuracy: 1e-10)
    XCTAssertFalse(diagnostics.forecastComplete)
    XCTAssertFalse(diagnostics.runwayQualified)
    XCTAssertNil(mission.plan, "Local curve steering must not turn an advisory into a certified terminal plan")
    XCTAssertLessThanOrEqual(abs(command.target.bank), c.vehicle.maximumBank)
    XCTAssertLessThanOrEqual(command.target.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    let decoded = try JSONDecoder().decode(GuidanceSolution.self, from: JSONEncoder().encode(command))
    XCTAssertEqual(decoded.commandRecovery?.referencePlanID, r.advisoryPlan.identifier)
  }
}
