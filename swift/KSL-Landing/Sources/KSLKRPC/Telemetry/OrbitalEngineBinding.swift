import KSLCore

/// Engine wrapper references are not stable across kRPC inventory reads; the
/// original vessel-part references are already checked as an unchanged topology
/// on reconnect. Multiple engine modules on one selected part are ambiguous.
enum OrbitalEngineBinding {
  static func resolve(parts: Set<UInt64>, inventory: [(engine: UInt64, part: UInt64)]) throws -> [UInt64] {
    guard !parts.isEmpty else { throw FlightError.rearmRequired("Missing original OMS identity") }
    var result: [UInt64] = []
    for part in parts.sorted() {
      let matches = inventory.filter { $0.part == part }
      guard matches.count == 1, matches[0].engine != 0 else {
        throw FlightError.rearmRequired("Original OMS part is missing or has ambiguous engine modules")
      }
      result.append(matches[0].engine)
    }
    return result
  }
}
