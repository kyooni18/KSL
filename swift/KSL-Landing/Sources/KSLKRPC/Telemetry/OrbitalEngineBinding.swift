import KSLCore

/// Engine wrapper references are not stable across kRPC inventory reads; the
/// original vessel-part references are already checked as an unchanged topology
/// on reconnect. Multiple engine modules on one selected part are ambiguous.
enum OrbitalEngineBinding {
  // STS OMS propulsion consists of its two LV-909/Terrier parts. Main engines
  // (SSME) must never enter an orbital deorbit maneuver, even when active.
  // Unknown engine names fail closed rather than silently becoming OMS.
  static func isOMSPartName(_ name: String) -> Bool { name == "liquidEngine3.v2" }
  static let requiredTerrierCount = 2

  /// Combine Isp by propellant mass flow, not by averaging engine Isp.
  static func specificImpulse(for engines: [(thrust: Double, isp: Double)]) -> Double? {
    guard !engines.isEmpty, engines.allSatisfy({
      $0.thrust.isFinite && $0.thrust > 0 && $0.isp.isFinite && $0.isp > 0
    }) else { return nil }
    let thrust = engines.reduce(0) { $0 + $1.thrust }
    let massFlowFactor = engines.reduce(0) { $0 + $1.thrust / $1.isp }
    guard thrust.isFinite && massFlowFactor.isFinite && massFlowFactor > 0 else { return nil }
    return thrust / massFlowFactor
  }

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
