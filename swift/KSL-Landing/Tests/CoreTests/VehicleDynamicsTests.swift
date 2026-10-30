import Foundation
import KSLCore
import XCTest

final class VehicleDynamicsTests: XCTestCase {
  func sample(time: Double = 100, revision: UInt64 = 1, center: Double = 0, mass: Double = 25000) -> VehicleState {
    var s = VehicleState(time: time, position: .init(610000, 0, 0), velocity: .init(0, 1000, 0),
      attitude: .init(), mass: mass)
    s.vesselConfiguration = .init(revision: revision, partCount: 22, anchorPart: 101,
      centerOfMassFromAnchor: .init(center, 0, 0), centerOfMassEvidence: .measured, measuredAt: time)
    return s
  }
  func testSamePartCountTopologyChangeInvalidates() {
    var monitor = VehicleDynamicsMonitor()
    XCTAssertNil(monitor.observe(sample()))
    XCTAssertEqual(monitor.observe(sample(time: 100.1, revision: 2))?.reasons, ["part-topology-changed"])
    XCTAssertNil(monitor.observe(sample(time: 100.2, revision: 2)))
    XCTAssertEqual(monitor.revision, 1)
  }
  func testCumulativeMassAndInternalPayloadMovement() {
    var monitor = VehicleDynamicsMonitor()
    XCTAssertNil(monitor.observe(sample()))
    XCTAssertNil(monitor.observe(sample(time: 100.1, center: 0.006, mass: 25150)))
    let change = monitor.observe(sample(time: 100.2, center: 0.012, mass: 25300))
    XCTAssertEqual(Set(change?.reasons ?? []), Set(["mass-changed", "center-of-mass-shifted"]))
  }
  func testUnknownCenterOfMassIsNotInventedAndOldRecordsDecode() throws {
    let s = sample()
    var object = try XCTUnwrap(JSONSerialization.jsonObject(with: JSONEncoder().encode(s)) as? [String: Any])
    object.removeValue(forKey: "vesselConfiguration")
    let old = try JSONDecoder().decode(VehicleState.self, from: JSONSerialization.data(withJSONObject: object))
    XCTAssertNil(old.vesselConfiguration)
    var monitor = VehicleDynamicsMonitor()
    XCTAssertNil(monitor.observe(old))
    XCTAssertNil(monitor.observe(s))
  }
}
