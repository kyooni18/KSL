import XCTest
import KSLCore
@testable import KSLRuntime

final class OMSWaitingPolicyTests:XCTestCase {
  func testRecordedOMSInventoryExcludesNonOMSResourcesAndSharedTankDuplication() throws {
    struct Inventory:Decodable {
      struct Resource:Decodable {let ratio:Double;let available:Double;let density:Double}
      let mass:Double;let dryMass:Double;let engines:[[Resource]]
    }
    let root=URL(fileURLWithPath:#filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
    let f=try JSONDecoder().decode(Inventory.self,from:Data(contentsOf:root.appendingPathComponent("Fixtures/oms-propellant-inventory-20261102.json")))
    let pool=zip(f.engines[0],f.engines[1]).map {(amount:min($0.available,$1.available),ratio:$0.ratio,density:$0.density)}
    let usable=try OMSPropellantBudget.usableMass(pool)
    let burnout=f.mass-usable
    let real=345*9.80665*log(f.mass/burnout)
    let wrong=345*9.80665*log(f.mass/f.dryMass)
    XCTAssertEqual(MissionConfiguration().deorbitSettings.burnDeltaVLimit,150)
    XCTAssertGreaterThan(burnout,f.dryMass)
    XCTAssertLessThan(real,wrong)
    XCTAssertLessThan(real,863.5620704351578+34.82828276581665)
    XCTAssertEqual(usable,pool.reduce(0){$0+$1.amount*$1.density},accuracy:0.001,
      "The recorded matched mixture is consumed once, not once per engine")
    print("OMS_BUDGET",usable,"burnoutMass",burnout,"availableDV",real,"wrongDryMassDV",wrong)
  }
  func testRecorded898MetrePerSecondPlanCannotAuthorizeUnder150Limit() throws {
    let root=URL(fileURLWithPath:#filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
    let plan=try JSONDecoder().decode(DeorbitPlan.self,from:Data(contentsOf:root.appendingPathComponent("Fixtures/overbudget-deorbit-20261102.json")))
    XCTAssertGreaterThan(plan.window.propulsiveDeltaV,150)
    XCTAssertEqual(plan.configuration.deorbitSettings.burnDeltaVLimit,150,
      "Old configuration records must inherit the mission limit")
    XCTAssertFalse(plan.hasExecutableEntryWindow)
    XCTAssertThrowsError(try plan.authorize(identifier:plan.identifier,reducedAssurance:true))
  }
  func testWaitingWinsAtComparableDeliveryAndQualifiedRoutesPreferFuel() {
    var near=DeorbitPlanner.InfeasibleApproachRank(hardUnsafe:false,screenFeasible:false,
      altitudeRejected:false,machRejected:false,captureMiss:23500,violationCount:1,entryMiss:1000)
    near.impulse=898;near.burnStart=1000;near.resolution=1000
    var later=DeorbitPlanner.InfeasibleApproachRank(hardUnsafe:false,screenFeasible:false,altitudeRejected:false,machRejected:false,captureMiss:23900,violationCount:1,entryMiss:1000);later.impulse=35;later.burnStart=9000;later.resolution=1000
    XCTAssertLessThan(later,near,"A sub-tolerance delivery improvement cannot justify a large plane change")
    var betterDelivery=DeorbitPlanner.InfeasibleApproachRank(hardUnsafe:false,screenFeasible:false,altitudeRejected:false,machRejected:false,captureMiss:2000,violationCount:1,entryMiss:1000)
    betterDelivery.resolution=1000
    XCTAssertLessThan(betterDelivery,later,"Fuel does not hide a substantial delivery deficit")
    let first=DeorbitPlanner.NominalRouteRank(qualified:true,unsafe:0,passed:1,miss:100,
      impulse:898,burnStart:1000)
    let waiting=DeorbitPlanner.NominalRouteRank(qualified:true,unsafe:0,passed:1,miss:1000,
      impulse:35,burnStart:9000)
    XCTAssertLessThan(waiting,first)
  }
}
