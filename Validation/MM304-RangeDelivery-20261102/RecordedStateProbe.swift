import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance
final class ZZScratchEntryRangeProbeTests: XCTestCase {
 func testRecordedDragRecovery() throws {
  let root = "/Users/kyooni18/Code/KSL/"
  let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: root+"runs/20261102T031333-taem-live-72km/configuration.json")))
  let s = try JSONDecoder().decode(VehicleState.self, from: Data(contentsOf: URL(fileURLWithPath: root+"Validation/MM304-RangeDelivery-20261102/state.json")))
  let law = EntryCommandLaw(configuration: c, atmosphere: try AtmosphericModelEngine(planet:c.planet), aerodynamics: try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea))
  law.memory.pitchResponseSeconds = 2.3214691806107606
  let t = law.update(s)
  print("RECORDED RANGE RECOVERY", t, law.diagnostics)
  XCTAssertGreaterThan(t.desiredFlightPath, s.flightPathAngle)
  XCTAssertFalse(LandingMissionController.terminalHandoffDue(phase:.entry, plan:nil,state:s,configuration:c))
 }
}
