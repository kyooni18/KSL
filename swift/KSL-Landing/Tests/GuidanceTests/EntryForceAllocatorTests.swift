import XCTest
import KSLCore
@testable import KSLGuidance

final class EntryForceAllocatorTests: XCTestCase {
  func testAoAAndBankShareOnePhysicalLiftVector() {
    let points = [
      EntryForcePoint(alpha: radians(20), lift: 9, drag: 2),
      EntryForcePoint(alpha: radians(25), lift: 12, drag: 2.5),
      EntryForcePoint(alpha: radians(30), lift: 15, drag: 3)
    ]
    let f = EntryForceAllocator.decide(points: points, verticalDemand: 9,
      lateralDemand: 6, referenceDrag: 2.5, dragBudget: 3,
      maximumBank: radians(70), loadLimit: 34.3, uncertainty: 0,
      previousAlpha: radians(25))
    XCTAssertTrue(f.affordable)
    XCTAssertGreaterThan(f.bank, 0)
    XCTAssertGreaterThanOrEqual(f.vertical, 9-1e-8)
    XCTAssertLessThanOrEqual(f.drag, 3+1e-8)
    XCTAssertEqual(hypot(f.vertical, f.lateral),
      points.first(where: { $0.alpha == f.alpha })!.lift, accuracy: 1e-8)
  }

  func testNoLiftDoesNotInventLateralControl() {
    let points = [
      EntryForcePoint(alpha: radians(18), lift: 2, drag: 1),
      EntryForcePoint(alpha: radians(35), lift: 5, drag: 2)
    ]
    let f = EntryForceAllocator.decide(points: points, verticalDemand: 9,
      lateralDemand: 20, referenceDrag: 1, dragBudget: 1,
      maximumBank: radians(70), loadLimit: 34.3, uncertainty: 0,
      previousAlpha: radians(35))
    XCTAssertFalse(f.verticallySupported)
    XCTAssertEqual(f.bank, 0, accuracy: 1e-12)
    XCTAssertEqual(f.lateral, 0, accuracy: 1e-12)
    XCTAssertEqual(f.vertical, 5, accuracy: 1e-12)
  }

  func testImpossibleVerticalSupportTradesSurplusAltitudeForBestLiftToDrag() {
    // Captured STS polar near 37 km: 30 deg has slightly more lift but more
    // than twice the drag of the best-glide candidate; neither can hold level.
    let points = [
      EntryForcePoint(alpha: radians(20), lift: 2.7, drag: 3.2),
      EntryForcePoint(alpha: radians(25), lift: 3.0, drag: 5.2),
      EntryForcePoint(alpha: radians(30), lift: 3.3, drag: 7.0)
    ]
    let result = EntryForceAllocator.decide(points: points, verticalDemand: 4.0,
      lateralDemand: 2, referenceDrag: 1.8, dragBudget: 2.0,
      maximumBank: radians(70), loadLimit: 34, uncertainty: 0,
      previousAlpha: radians(30), heightAboveCapture: 12_000)
    XCTAssertEqual(result.alpha, radians(20), accuracy: 1e-12)
    XCTAssertEqual(result.drag, 3.2, accuracy: 1e-12)
    XCTAssertEqual(result.bank, 0, accuracy: 1e-12)
    XCTAssertFalse(result.verticallySupported)
    XCTAssertFalse(result.affordable)
  }

  func testNoAltitudeReserveKeepsMaximumLift() {
    let points = [
      EntryForcePoint(alpha: radians(20), lift: 2.7, drag: 3.2),
      EntryForcePoint(alpha: radians(30), lift: 3.3, drag: 7.0)
    ]
    let result = EntryForceAllocator.decide(points: points, verticalDemand: 4.0,
      lateralDemand: 1, referenceDrag: 1.8, dragBudget: 2,
      maximumBank: radians(70), loadLimit: 34, uncertainty: 0,
      previousAlpha: radians(30), heightAboveCapture: -100)
    XCTAssertEqual(result.alpha, radians(30), accuracy: 1e-12)
  }

  func testAffordableMaximumLiftDoesNotTradeAltitudeUnnecessarily() {
    let points = [
      EntryForcePoint(alpha: radians(20), lift: 2.7, drag: 3.2),
      EntryForcePoint(alpha: radians(30), lift: 3.3, drag: 7.0)
    ]
    let result = EntryForceAllocator.decide(points: points, verticalDemand: 4.0,
      lateralDemand: 0, referenceDrag: 7, dragBudget: 8,
      maximumBank: radians(70), loadLimit: 34, uncertainty: 0,
      previousAlpha: radians(30), heightAboveCapture: 12_000)
    XCTAssertEqual(result.alpha, radians(30), accuracy: 1e-12)
  }

  func testJointForceDiagnosticRejectsImpossibleCrossrangeAndDoesNotInventDragMargin() {
    let points = [
      EntryForcePoint(alpha: radians(20), lift: 2.7, drag: 3.2),
      EntryForcePoint(alpha: radians(30), lift: 4.74, drag: 8.86),
    ]
    let infeasible = EntryForceAllocator.assessJointForces(points: points,
      verticalDemand: 3.66, lateralDemand: 30.4, maximumBank: radians(70),
      loadLimit: 34, uncertainty: 0, dragBudget: 3.56)
    XCTAssertEqual(infeasible.requiredLift, hypot(3.66, 30.4), accuracy: 1e-10)
    XCTAssertEqual(infeasible.maximumAvailableLift, 4.74, accuracy: 1e-10)
    XCTAssertFalse(infeasible.simultaneouslySupported)
    XCTAssertNil(infeasible.minimumSimultaneousDrag)
    XCTAssertNil(infeasible.dragBudgetDeficit)
    // It is an immediate joint-force impossibility, NOT a proof that a
    // different flown trajectory or earlier crossrange correction cannot land.
    let feasible = EntryForceAllocator.assessJointForces(points: points,
      verticalDemand: 3.66, lateralDemand: 3, maximumBank: radians(70),
      loadLimit: 34, uncertainty: 0, dragBudget: 3.56)
    XCTAssertTrue(feasible.simultaneouslySupported)
    XCTAssertEqual(try! XCTUnwrap(feasible.minimumSimultaneousDrag), 8.86, accuracy: 1e-10)
    XCTAssertEqual(try! XCTUnwrap(feasible.dragBudgetDeficit), 5.3, accuracy: 1e-10)
  }

  func testJointDragCannotBeBorrowedFromAnUnrelatedLowLiftPolarPoint() {
    let points = [
      EntryForcePoint(alpha: radians(15), lift: 2.5, drag: 1.0),
      EntryForcePoint(alpha: radians(27), lift: 9.0, drag: 5.0),
    ]
    let joint = EntryForceAllocator.assessJointForces(points: points,
      verticalDemand: 8, lateralDemand: 3, maximumBank: radians(60),
      loadLimit: 34, uncertainty: 0, dragBudget: 4)
    XCTAssertTrue(joint.simultaneouslySupported)
    XCTAssertEqual(try! XCTUnwrap(joint.minimumSimultaneousDrag), 5, accuracy: 1e-12)
    XCTAssertEqual(try! XCTUnwrap(joint.dragBudgetDeficit), 1, accuracy: 1e-12)
    let tooLittleBank = EntryForceAllocator.assessJointForces(points: points,
      verticalDemand: 5, lateralDemand: 7.0, maximumBank: radians(40),
      loadLimit: 34, uncertainty: 0, dragBudget: 10)
    XCTAssertFalse(tooLittleBank.simultaneouslySupported)
    XCTAssertNil(tooLittleBank.minimumSimultaneousDrag)
  }

  func testNoVerticalSolutionMayTradeBoundedAltForLateralTurn() {
    let points = [
      EntryForcePoint(alpha: radians(16), lift: 1.0, drag: 0.8),
      EntryForcePoint(alpha: radians(25), lift: 2.0, drag: 2.4)
    ]
    // At h=40 km, about 15 km of headroom and ~180 s to the 25 km
    // preferred inlet. Total permitted extra normal deficit ~0.74 m/s2
    // with 20% held as an uncertainty reserve.
    let time = 180.0, margin = 15_000.0, uncertainty = 0.20
    let a = EntryForceAllocator.decide(points: points, verticalDemand: 2.4,
      lateralDemand: -25, referenceDrag: 2.0, dragBudget: 1.5,
      maximumBank: radians(70), loadLimit: 34, uncertainty: uncertainty,
      previousAlpha: radians(20), heightAboveCapture: margin,
      timeToCapture: time)
    XCTAssertFalse(a.verticallySupported)
    XCTAssertLessThan(a.bank, 0)
    XCTAssertLessThan(a.lateral, 0)
    XCTAssertGreaterThan(abs(a.lateral), 0.5)
    XCTAssertGreaterThanOrEqual(a.vertical + 2*margin*(1-uncertainty)/(time*time),
      2.4-1e-10)
    XCTAssertEqual(a.vertical*a.vertical + a.lateral*a.lateral, 4, accuracy: 1e-10)

    let none = EntryForceAllocator.decide(points: points, verticalDemand: 2.4,
      lateralDemand: -25, referenceDrag: 2, dragBudget: 1.5,
      maximumBank: radians(70), loadLimit: 34, uncertainty: uncertainty,
      previousAlpha: radians(20), heightAboveCapture: 0, timeToCapture: time)
    XCTAssertEqual(none.bank, 0, accuracy: 1e-12)
    XCTAssertEqual(none.lateral, 0, accuracy: 1e-12)
  }

  func testEnergyShortfallRequiresMinimumDragAndTurn() {
    let points = [
      EntryForcePoint(alpha: radians(18), lift: 10, drag: 1),
      EntryForcePoint(alpha: radians(30), lift: 14, drag: 5)
    ]
    let f = EntryForceAllocator.decide(points: points, verticalDemand: 8,
      lateralDemand: -5, referenceDrag: 1, dragBudget: 1.2,
      maximumBank: radians(70), loadLimit: 34.3, uncertainty: 0,
      previousAlpha: radians(30))
    XCTAssertLessThan(f.bank, 0)
    XCTAssertLessThan(f.lateral, 0)
    XCTAssertLessThanOrEqual(f.drag, 1.2)
    XCTAssertGreaterThanOrEqual(f.vertical, 8-1e-8)
  }
}
