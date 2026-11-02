import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class EntryHandoffTimingTests: XCTestCase {
  struct Fixture:Decodable {
    struct Sample:Decodable {let state:VehicleState;let objective:EntryCaptureObjective;let responseSeconds:Double;let forecast:EntryHandoffForecast}
    let configuration:MissionConfiguration
    let samples:[Sample]
  }
  func testRecordedOpportunityTransfersBeforeTheLateMachDeadline() throws {
    let root=URL(fileURLWithPath:#filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
    let f=try JSONDecoder().decode(Fixture.self,from:Data(contentsOf:root.appendingPathComponent("Fixtures/mm304-handoff-opportunity-20261102.json")))
    let decisions=f.samples.map {s in EntryHandoffTiming.assess(state:s.state,configuration:f.configuration,
      objective:s.objective,responseSeconds:s.responseSeconds,forecast:s.forecast)}
    for (s,d) in zip(f.samples,decisions) {print("HANDOFF_OPPORTUNITY",s.state.time,d)}
    let aero=try AerodynamicsEngine(referenceArea:f.configuration.vehicle.referenceArea)
    let physical=f.samples.map {s in EntryHandoffTiming.assess(state:s.state,configuration:f.configuration,
      objective:s.objective,responseSeconds:s.responseSeconds,forecast:s.forecast,
      maximumLiftAcceleration:EntryHandoffTiming.maximumLift(state:s.state,configuration:f.configuration,
        aerodynamics:aero,ceiling:RotationalDynamicsEngine(configuration:f.configuration.vehicle,
          planet:f.configuration.planet).supportedTrimAlpha(s.state,identifiedCeiling:nil)))}
    for (s,d) in zip(f.samples,physical) {print("HANDOFF_WITH_TURN",s.state.time,d)}
    XCTAssertFalse(physical[0].transferNow,"The heading warning cannot bypass the upper terminal window")
    XCTAssertTrue(physical[1].transferNow)
    XCTAssertEqual(physical[1].reason,"heading-turn-response-lead")
    XCTAssertGreaterThan(try XCTUnwrap(physical[1].localHeadingTurnSeconds),
      try XCTUnwrap(physical[1].secondsToClosestApproach))
    XCTAssertFalse(decisions[0].transferNow,"Excess-energy state above the terminal Mach window must keep entry")
    XCTAssertFalse(decisions[1].transferNow,"Waiting is useful while the fresh entry forecast still improves the inlet")
    XCTAssertEqual(decisions[1].reason,"entry-forecast-still-improves-inlet")
    XCTAssertFalse(decisions[2].transferNow,"Before the measured response reserve is spent, the forecast still improves")
    XCTAssertTrue(decisions[3].transferNow,"Do not spend the handover response reserve waiting for interface energy")
    XCTAssertGreaterThan(try XCTUnwrap(decisions[3].energyMargin),0,
      "The response/opportunity decision must act before the old energy threshold")
    XCTAssertTrue(decisions[4].transferNow,"Actual interface energy must transfer even outside the 5 km aim")
    XCTAssertLessThan(f.samples[3].state.time,216440.55235288214,
      "Recorded late handoff lost 13.38 seconds after interface energy")
  }
}
