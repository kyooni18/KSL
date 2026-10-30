import Foundation
import KSLCore
import KSLPhysics
import KSLGuidance
import KSLRuntime
import KSLSimulation
import XCTest

final class ShuttleSimAdapterTests: XCTestCase {
  let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
  func fixture() throws -> [Data] {
    try String(contentsOf: root.appendingPathComponent("Fixtures/shuttlesim-polar100-first-two.jsonl"), encoding: .utf8)
      .split(whereSeparator: \.isNewline).map { Data($0.utf8) }
  }
  func decoder() throws -> ShuttleSimTelemetryDecoder {
    var c = MissionConfiguration(); c.runway.elevation = 70
    return try .init(configuration: c, attitudeINI: "direct_pitch_accel_per_kpa_deg=12.6211")
  }
  func testInertialUTAndDistinctRunwayCoordinateDefinitions() throws {
    let records = try fixture(), d = try decoder()
    let first = try d.decode(records[0]), second = try d.decode(records[1])
    XCTAssertLessThan(first.geometryError, 0.001)
    XCTAssertEqual(first.state.mass, 26146.85, accuracy: 1e-6)
    XCTAssertEqual(first.state.position.length-600000,100000,accuracy:0.001)
    XCTAssertEqual(first.bodyRateEvidence, .unavailable)
    XCTAssertEqual(second.bodyRateEvidence, .identified)
    XCTAssertGreaterThan(second.state.angularVelocity.length, 0)
    XCTAssertTrue(first.lowPressureControlUnsupported)
    XCTAssertEqual(first.state.controlAuthority?.minimumAcceleration, .zero)
    XCTAssertNil(first.state.vesselConfiguration)
    XCTAssertFalse(first.state.aerodynamics.measuredForces)
  }
  func testWrongRotationPhaseCannotSilentlyEnterController() throws {
    var raw = try XCTUnwrap(JSONSerialization.jsonObject(with: fixture()[0]) as? [String: Any])
    var world = raw["world"] as! [String: Any]
    world["rotation_phase_rad_at_ut0"] = 0.0; raw["world"] = world
    XCTAssertThrowsError(try decoder().decode(JSONSerialization.data(withJSONObject: raw)))
  }
  func testOldConfigurationProposalRejectedAfterPayloadChange() async throws {
    let scenario = SimulationScenario.approach(), c = scenario.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet), aero = try AerodynamicsEngine()
    let plant = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let computer = try FlightComputer(configuration: c, atmosphere: atmosphere, aerodynamics: aero, externalPlanning: true)
    var state = plant.observe(scenario.initialState)
    _ = try computer.step(state)
    let request = try XCTUnwrap(computer.planningRequest(from: state))
    let worker = MissionPlanningWorker(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let proposal = await worker.plan(request)
    state.time += 0.1; state.mass += 3000
    computer.accept(proposal, state: state)
    let record = try computer.step(state)
    XCTAssertEqual(computer.dynamicsRevision, 1)
    XCTAssertNotNil(record.dynamicsChange)
    XCTAssertNil(record.guidance.approach?.predictedTouchdownAlong)
  }
}
