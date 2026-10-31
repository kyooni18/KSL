import XCTest
@testable import KSLGuidance

final class HACRankingTests: XCTestCase {
  func testFeasibleReferenceRanksFirstButInfeasibleFamilyStillHasOrdering() {
    XCTAssertTrue(HACPlanningEngine.ranksAhead(feasible: true, cost: 100,
      ofFeasible: false, cost: 0))
    XCTAssertFalse(HACPlanningEngine.ranksAhead(feasible: false, cost: 0,
      ofFeasible: true, cost: 100))
    XCTAssertTrue(HACPlanningEngine.ranksAhead(feasible: false, cost: 1,
      ofFeasible: false, cost: 2))
    XCTAssertFalse(HACPlanningEngine.ranksAhead(feasible: false, cost: 2,
      ofFeasible: false, cost: 1))
  }
}
