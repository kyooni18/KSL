import XCTest
import KSLCore
import KSLControl
import KSLGuidance
import KSLPhysics
@testable import KSLRuntime

final class NormalDeorbitCorrectionTests: XCTestCase {
  func testRecordedDessertBurnCanPriceAndExecuteIndependentNormalImpulse() throws {
    // Original measured orbital state, propulsion and configuration. This is
    // one alternative maneuver, not an altered vehicle or landing campaign.
    let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent()
      .deletingLastPathComponent().deletingLastPathComponent()
    let original = try JSONDecoder().decode(DeorbitPlan.self,
      from: Data(contentsOf: root.appendingPathComponent("Fixtures/dessert-deorbit-20261102.json")))
    let c = original.configuration, orbit = OrbitalMechanics(planet: c.planet)
    let initial = OrbitalState(fixed: original.source, planet: c.planet)
    let candidate = try DeorbitWindowSearch(configuration: c).normalCorrection(initial: initial,
      seed: original.window, propulsion: original.propulsion)
    let normal = try XCTUnwrap(candidate.normalDeltaV)
    XCTAssertLessThanOrEqual(candidate.propulsiveDeltaV,c.deorbitSettings.burnDeltaVLimit)
    XCTAssertTrue(candidate.closing)
    XCTAssertTrue(candidate.miss.isFinite,
      "The mission budget may prevent orbital convergence; do not imply capture")
    XCTAssertGreaterThanOrEqual(candidate.burnStart, initial.time+c.deorbitSettings.minimumPlanningLeadTime)
    let coast = try orbit.propagate(initial, to: candidate.burnStart, step: 12)
    let burn = try FiniteBurnModel(planet: c.planet).propagate(from: coast,
      deltaV: candidate.deltaV, propulsion: original.propulsion,
      settings: c.deorbitSettings, normalDeltaV: normal)
    XCTAssertTrue(burn.complete)
    XCTAssertEqual(try XCTUnwrap(burn.deliveredNormalDeltaV), normal,
      accuracy: BurnThrottleLaw.terminalDeltaVTolerance)
    XCTAssertEqual(burn.deliveredDeltaV, candidate.deltaV,
      accuracy: BurnThrottleLaw.terminalDeltaVTolerance)
    XCTAssertGreaterThan(burn.state.mass, original.propulsion.dryMass)
    let plan = DeorbitPlan(source: original.source, configuration: c,
      propulsion: original.propulsion, window: candidate, cases: [],
      modelLimitations: ["test maneuver; no live actuation authorization"], evaluatedWindows: 1)
    let encoded = try JSONEncoder().encode(plan)
    let decoded = try JSONDecoder().decode(DeorbitPlan.self, from: encoded)
    XCTAssertEqual(decoded.window.normalDeltaV, normal)
    let controlled = try OrbitalExecutionModel(configuration: c).predict(source: original.source,
      window: decoded.window, propulsion: original.propulsion)
    XCTAssertTrue(controlled.burnCompleted)
    let deliveredNormal = try XCTUnwrap(controlled.deliveredNormalDeltaV)
    XCTAssertGreaterThanOrEqual((normal < 0 ? -1 : 1)*deliveredNormal,
      abs(normal)-BurnThrottleLaw.terminalDeltaVTolerance)
    XCTAssertGreaterThan(controlled.entry.mass, original.propulsion.dryMass)
    XCTAssertLessThan(controlled.entry.position.length-c.planet.radius, c.planet.atmosphereTop)
    let baseline = try OrbitalExecutionModel(configuration: c).predict(source: original.source,
      window: original.window, propulsion: original.propulsion)
    let frame = RunwayFrame(runway:c.runway,planet:c.planet)
    func crossMiss(_ state: VehicleState) -> Double {
      let range = frame.distance(from:state.position,to:frame.origin)
      let error = frame.bearing(from:state.position,to:frame.origin)-state.course
      return abs(asin(clamp(sin(range/c.planet.radius)*sin(error),-1,1))*c.planet.radius)
    }
    XCTAssertLessThan(crossMiss(controlled.entry),crossMiss(baseline.entry))
    print("NORMAL_OMS candidate", candidate, "finiteDeliveredNormal", burn.deliveredNormalDeltaV as Any,
      "controlledEntry", controlled.entry.position, controlled.entry.velocity,
      "controlledEntryFPA", degrees(controlled.entry.flightPathAngle),
      "controlledNormal",deliveredNormal,"controlledCrossMiss",crossMiss(controlled.entry),
      "baselineControlledCrossMiss",crossMiss(baseline.entry))
  }
}
