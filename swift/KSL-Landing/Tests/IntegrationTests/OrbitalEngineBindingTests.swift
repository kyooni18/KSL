import XCTest
@testable import KSLKRPC

final class OrbitalEngineBindingTests: XCTestCase {
  func testRebindsNewWrappersByOriginalPartAndRejectsAmbiguity() throws {
    let selected: Set<UInt64> = [98, 99]
    let inventory: [(engine: UInt64, part: UInt64)] = [(400, 95), (401, 99), (402, 98)]
    XCTAssertEqual(try OrbitalEngineBinding.resolve(parts: selected, inventory: inventory), [402, 401])
    XCTAssertThrowsError(try OrbitalEngineBinding.resolve(parts: selected, inventory: [(402, 98)]))
    XCTAssertThrowsError(try OrbitalEngineBinding.resolve(parts: selected, inventory: inventory + [(403, 98)]))
    XCTAssertThrowsError(try OrbitalEngineBinding.resolve(parts: [], inventory: inventory))
  }
}
