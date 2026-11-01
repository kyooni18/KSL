import XCTest
@testable import KSLKRPC

final class OrbitalEngineBindingTests: XCTestCase {
  func testOnlyTerrierPairIsOMSSafeAndIspUsesMassFlow() {
    XCTAssertTrue(OrbitalEngineBinding.isOMSPartName("liquidEngine3.v2"))
    XCTAssertFalse(OrbitalEngineBinding.isOMSPartName("SSME"))
    XCTAssertFalse(OrbitalEngineBinding.isOMSPartName("liquidEngine2"))
    XCTAssertFalse(OrbitalEngineBinding.isOMSPartName("UnknownEngine"))
    XCTAssertEqual(OrbitalEngineBinding.requiredTerrierCount, 2)
    let terriers = [(thrust: 60_000.0, isp: 345.0), (thrust: 60_000.0, isp: 345.0)]
    XCTAssertEqual(OrbitalEngineBinding.specificImpulse(for: terriers) ?? -1, 345, accuracy: 0.00001)
    XCTAssertNil(OrbitalEngineBinding.specificImpulse(for: [(thrust: 0, isp: 345)]))
  }
  func testRebindsNewWrappersByOriginalPartAndRejectsAmbiguity() throws {
    let selected: Set<UInt64> = [98, 99]
    let inventory: [(engine: UInt64, part: UInt64)] = [(400, 95), (401, 99), (402, 98)]
    XCTAssertEqual(try OrbitalEngineBinding.resolve(parts: selected, inventory: inventory), [402, 401])
    XCTAssertThrowsError(try OrbitalEngineBinding.resolve(parts: selected, inventory: [(402, 98)]))
    XCTAssertThrowsError(try OrbitalEngineBinding.resolve(parts: selected, inventory: inventory + [(403, 98)]))
    XCTAssertThrowsError(try OrbitalEngineBinding.resolve(parts: [], inventory: inventory))
  }
}
