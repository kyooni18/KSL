import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMTransitionTests: XCTestCase {
  func testHorizontalNEPCrossingCannotForceHighAltitudeFinal() throws {
    let c=MissionConfiguration(), atm=try AtmosphericModelEngine(), aero=try AerodynamicsEngine()
    let frame=RunwayFrame(runway:c.runway,planet:c.planet)
    let approach=ApproachGuidanceEngine(configuration:c,atmosphere:atm,aerodynamics:aero)
    func state(height:Double)->VehicleState {
      let position=frame.position(along: -c.guidance.finalDistance+100,cross:0,height:height)
      let velocity=frame.direction(at:.init(-c.guidance.finalDistance+100,0),tangent:.init(1,0))*400-position.normalized*80
      return .init(time:100,position:position,velocity:velocity,
        attitude:.windAttitude(position:position,airVelocity:velocity,alpha:radians(15),bank:0))
    }
    XCTAssertFalse(approach.recoveryDue(state(height:25_000)))
    XCTAssertFalse(approach.recoveryDue(state(height:15_543)))
    let profile=ApproachProfile(configuration:c)
    XCTAssertTrue(approach.recoveryDue(state(height:profile.reference(along:-c.guidance.finalDistance).height)))
  }

  func testOverheadProgressCannotLoseOneWindingAtTheAngleBoundary() {
    let remaining=2 * Double.pi+0.03
    let next=TAEMGuidanceEngine.advanceTurn(remaining,from:0.03,to:-0.01)
    XCTAssertEqual(next,2 * Double.pi-0.01,accuracy:1e-12)
    XCTAssertGreaterThan(next,Double.pi)
    XCTAssertEqual(TAEMGuidanceEngine.advanceTurn(0.01,from:0.01,to:-0.01),0)
  }

  func testDescentPlanRequiresEnoughDistanceAndPreservesEndpointSlopes() {
    let short=TAEMDescentReference(distance:33_360,startHeight:26_388,endHeight:962,
      startSlope:0.2,endSlope:tan(radians(24)))
    XCTAssertFalse(short.fits(maximumGlide:radians(24)))
    let planned=TAEMDescentReference(distance:80_000,startHeight:26_388,endHeight:962,
      startSlope:0.2,endSlope:tan(radians(24)))
    XCTAssertTrue(planned.fits(maximumGlide:radians(24)))
    XCTAssertEqual(planned.sample(planned.distance).height,planned.startHeight,accuracy:1e-8)
    XCTAssertEqual(planned.sample(planned.distance).slope,planned.startSlope,accuracy:1e-10)
    XCTAssertEqual(planned.sample(0).height,planned.endHeight,accuracy:1e-8)
    XCTAssertEqual(planned.sample(0).slope,planned.endSlope,accuracy:1e-10)
  }
}
