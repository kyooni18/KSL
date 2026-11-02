import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class RecordedRunway36RecoveryTests: XCTestCase {
  struct Fixture: Decodable {
    struct Range: Decodable {let state:VehicleState;let control:ControlDiagnostics;let objective:EntryCaptureObjective}
    let configuration:MissionConfiguration
    let range:Range
    let handoff:VehicleState
    let liftGain:Double
    let dragGain:Double
  }
  private func fixture() throws -> Fixture {
    let root=URL(fileURLWithPath:#filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
    return try JSONDecoder().decode(Fixture.self,from:Data(contentsOf:root.appendingPathComponent("Fixtures/runway36-recovery-20261102.json")))
  }
  func testUnbracketedReferencePricesTheCompleteArrival() throws {
    let f=try fixture(),c=f.configuration,s=f.range.state
    let atm=try AtmosphericModelEngine(planet:c.planet),aero=try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea)
    let seed=EntryCaptureObjective.seed(configuration:c,atmosphere:atm,
      frame:RunwayFrame(runway:c.runway,planet:c.planet),crossAim:0,time:s.time)
    XCTAssertEqual(seed.along,c.taemInterfaceAlong,accuracy:1e-9)
    XCTAssertEqual(hypot(seed.along,seed.cross),c.entrySettings.captureRange,accuracy:1e-9)
    let mission=LandingMissionController(configuration:c,atmosphere:atm,aerodynamics:aero,externalPlanning:true)
    XCTAssertNotNil(mission.planningRequest(from:f.handoff,controls:nil)?.terminalState,
      "Missing the 5 km delivery aim cannot suppress terminal search")
    let control=f.range.control
    let ceiling=RotationalDynamicsEngine(configuration:c.vehicle,planet:c.planet).supportedTrimAlpha(s,
      identifiedCeiling:control.trimEvidence == .identified ? control.trimAlphaCeiling:nil)
    let result=EntryRangeReference.solve(from:s,objective:f.range.objective,configuration:c,atmosphere:atm,aero:aero,
      ceiling:ceiling,pitchResponse:control.pitchResponseSeconds ?? c.guidance.pathLookaheadTime,
      rollResponse:control.rollResponseSeconds ?? c.guidance.pathLookaheadTime)
    XCTAssertTrue(result.reachedEnergy)
    XCTAssertFalse(result.bracketed,"Both recorded endpoint along residuals are positive")
    XCTAssertGreaterThan(result.blend,0,"Along-only fallback previously selected best glide")
    XCTAssertLessThan(abs(result.cross),37902.85201511519,
      "A 1.46 km along improvement must not discard 37.8 km of crossrange improvement")
    print("RECORDED_RECOVERY_RANGE",result)
  }
  func testNoTrimSupportedTurnCannotCommitLongestOverheadAsFeasible() throws {
    let f=try fixture(),c=f.configuration
    let atm=try AtmosphericModelEngine(planet:c.planet),aero=try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea)
    let nep=try XCTUnwrap(NominalEnergyPointSolver(configuration:c,atmosphere:atm,aerodynamics:aero).solve(from:f.handoff))
    let engine=TAEMGuidanceEngine(configuration:c,atmosphere:atm,aerodynamics:aero)
    let target=engine.update(f.handoff,nep:nep,liftGain:f.liftGain,dragGain:f.dragGain,
      attainableAlphaCeiling:nil,pitchResponseSeconds:2.370130650626248,rollResponseSeconds:4.481791722752409)
    XCTAssertNil(engine.status.attainableAcquisitionRadius)
    XCTAssertEqual(engine.status.acquisitionTurnAvailable,false)
    XCTAssertEqual(engine.status.descentCheckFailure,"acquisitionTurnUnavailable")
    XCTAssertEqual(engine.status.referenceTracking,false)
    XCTAssertLessThan(engine.status.remainingTurn,2 * .pi)
    XCTAssertLessThan(engine.status.range,145202.23544634835)
    XCTAssertLessThanOrEqual(abs(target.bank),c.vehicle.maximumBank)
    print("RECORDED_RECOVERY_TAEM",engine.status,"target",target)
  }
}
