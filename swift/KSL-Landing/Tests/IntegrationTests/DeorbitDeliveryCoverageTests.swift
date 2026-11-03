import XCTest
import KSLCore
import KSLGuidance
import KSLPhysics
@testable import KSLRuntime

final class DeorbitDeliveryCoverageTests: XCTestCase {
  func testRecordedAtmosphericPrefixCannotPurchaseAnUnpricedTAEMArrival() throws {
    let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent()
      .deletingLastPathComponent().deletingLastPathComponent()
    let plan = try JSONDecoder().decode(DeorbitPlan.self,
      from: Data(contentsOf: root.appendingPathComponent("Fixtures/deorbit-truncated-screen-20261103.json")))
    let c = plan.configuration
    let screen = try XCTUnwrap(plan.entryScreen)
    XCTAssertGreaterThan(screen.terminalAltitude+c.runway.elevation, c.entrySettings.terminalMaximumAltitude)
    // This is the recorded diagnostic endpoint, not a new trajectory/plant.
    let prefix = [[plan.window.entryInterface.time, screen.terminalAlong, screen.terminalCross,
      screen.terminalAltitude, plan.window.entrySpeed, 0, 0]]
    var selected = DeorbitPlanner.infeasibleApproachRank(track: prefix, hardUnsafe: false,
      screenFeasible: false, violationCount: 1, entryMiss: plan.window.miss,
      configuration: c, terminalSpeed: c.guidance.finalSpeed)
    XCTAssertFalse(selected.deliveryAssessed)
    XCTAssertEqual(selected.captureMiss, .greatestFiniteMagnitude)
    // The smallest decisive additional check is only the existing orbital
    // proposal search, from this exact measured source/propulsion. No entry
    // trajectory, altered vehicle, perturbation or flight campaign is run.
    let windows = try DeorbitWindowSearch(configuration: c).search(from: plan.source, propulsion: plan.propulsion)
    let orbital = try XCTUnwrap(windows.candidates.min { $0.miss < $1.miss })
    XCTAssertLessThan(orbital.miss, plan.window.miss)
    var fallback = DeorbitPlanner.infeasibleApproachRank(track: prefix, hardUnsafe: false,
      screenFeasible: false, violationCount: 1, entryMiss: orbital.miss,
      configuration: c, terminalSpeed: c.guidance.finalSpeed)
    selected.impulse = plan.window.propulsiveDeltaV; selected.burnStart = plan.window.burnStart
    fallback.impulse = orbital.propulsiveDeltaV; fallback.burnStart = orbital.burnStart
    XCTAssertLessThan(fallback, selected, "An unpriced near-runway prefix cannot outrank the configured orbital-entry aim")
    print("RECORDED_ORBITAL_COVERAGE", "selectedRange", plan.window.entryRange,
      "selectedMiss", plan.window.miss, "fallbackRange", orbital.entryRange,
      "fallbackMiss", orbital.miss, "fallbackDeltaV", orbital.propulsiveDeltaV,
      "fallbackBurnStart", orbital.burnStart, "candidates", windows.candidates.count)
    var metadata = DeorbitEntryScreen(feasible: false, altitudeRejected: true, machRejected: true,
      captureMiss: .greatestFiniteMagnitude, terminalAlong: screen.terminalAlong,
      terminalCross: screen.terminalCross, terminalAltitude: screen.terminalAltitude,
      terminalMach: screen.terminalMach, deliveryAssessed: false)
    let encoded = try JSONEncoder().encode(metadata)
    metadata = try JSONDecoder().decode(DeorbitEntryScreen.self, from: encoded)
    XCTAssertEqual(metadata.deliveryAssessed, false)
    // Metadata-only final-plan ordering: neither DTO claims a native route
    // evaluation. Unknown endpoint data must not override the orbital aim.
    let oldPlan = DeorbitPlan(source: plan.source, configuration: c, propulsion: plan.propulsion,
      window: plan.window, cases: [], modelLimitations: ["ordering-only; no route evaluation"],
      evaluatedWindows: 1, entryScreen: metadata)
    let fallbackPlan = DeorbitPlan(source: plan.source, configuration: c, propulsion: plan.propulsion,
      window: orbital, cases: [], modelLimitations: ["ordering-only; no route evaluation"],
      evaluatedWindows: 1, entryScreen: metadata)
    XCTAssertTrue(DeorbitPlanner.ranksPlansAhead(fallbackPlan, oldPlan))
    XCTAssertFalse(DeorbitPlanner.ranksPlansAhead(oldPlan, fallbackPlan))
  }
}
