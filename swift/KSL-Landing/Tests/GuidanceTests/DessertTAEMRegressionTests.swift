import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class DessertTAEMRegressionTests: XCTestCase {
  private struct RecordedHandoff: Decodable {
    let configuration: MissionConfiguration
    let state: VehicleState
  }

  func testRecordedBrakingDeficitDoesNotEraseAirborneTAEMReference() throws {
    // Exact configuration and first TAEM state from native session
    // B090715F-7F7C-4F5E-AE19-05E4FC473E85. No modified vehicle capability.
    let url = try XCTUnwrap(Bundle.module.url(forResource: "dessert-taem-20261102",
      withExtension: "json", subdirectory: "Fixtures"))
    let recorded = try JSONDecoder().decode(RecordedHandoff.self, from: Data(contentsOf: url))
    let c = recorded.configuration, state = recorded.state
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let nep = try XCTUnwrap(NominalEnergyPointSolver(configuration: c,
      atmosphere: atmosphere, aerodynamics: aero).solve(from: state))
    XCTAssertEqual(nep.airspeed, 119.169877, accuracy: 0.001)
    XCTAssertEqual(nep.predictedTouchdownSpeed, c.vehicle.landingSpeed, accuracy: 0.001)
    let assessment = try XCTUnwrap(nep.finalAssessment)
    XCTAssertEqual(assessment.airborneFeasible, true)
    XCTAssertFalse(assessment.feasible, "The runway stopping deficit must remain unqualified")
    let braking = try XCTUnwrap(assessment.rolloutViolations?.first)
    XCTAssertEqual(assessment.violations.count, 1)
    XCTAssertEqual(braking.kind, .excessiveEnergy)
    XCTAssertEqual(braking.value, 1315.616312, accuracy: 0.001)
    XCTAssertEqual(braking.limit, 1250, accuracy: 0.001)
    // Keep the downstream verdict in serialized planning/flight evidence.
    let decoded = try JSONDecoder().decode(NominalEnergyPoint.self,
      from: JSONEncoder().encode(nep))
    XCTAssertEqual(decoded.finalAssessment?.rolloutViolations?.first?.value, braking.value)

    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true, initialPhase: .acquisition)
    let solution = try mission.update(state)
    let trace = try XCTUnwrap(solution.targetHistory)
    XCTAssertEqual(trace.after.previous, solution.target)
    XCTAssertEqual(try JSONDecoder().decode(GuidanceCommandTrace.self,
      from: JSONEncoder().encode(trace)), trace)
    XCTAssertTrue(solution.diagnostic.hasPrefix("TAEM phase="),
      "The recorded missing-NEP fallback must now reach the normal TAEM law")
    XCTAssertFalse(solution.violations.contains { $0.message.hasPrefix("No solved NEP") })
    XCTAssertTrue(solution.violations.contains {
      $0.message == braking.message && $0.value == braking.value && $0.limit == braking.limit
    })
    XCTAssertGreaterThan(abs(solution.target.bank), 0)
    XCTAssertLessThanOrEqual(abs(solution.target.bank), c.vehicle.maximumBank)
    XCTAssertLessThanOrEqual(solution.target.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    print("Dessert recorded TAEM: \(solution.diagnostic); bank=\(degrees(solution.target.bank)) deg; braking deficit=\(braking.value-braking.limit) m")
  }
}
