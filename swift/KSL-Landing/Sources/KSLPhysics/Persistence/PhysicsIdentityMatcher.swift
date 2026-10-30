import Foundation

/// f1eebdc persistence/store/{identity,schema}.inc semantic compatibility.
/// Manual model variants remain separate even when their part inventories match.
public enum PhysicsIdentityMatcher {
  private static func object(_ text: String) -> [String: Any]? {
    guard let data = text.data(using: .utf8) else { return nil }
    return (try? JSONSerialization.jsonObject(with: data)) as? [String: Any]
  }
  public static func environmentsMatch(_ a: String, _ b: String) -> Bool {
    guard let a = object(a), let b = object(b),
          var pa = a["planet"] as? [String: Any], var pb = b["planet"] as? [String: Any] else { return false }
    for key in ["northAxisKRPC", "primeMeridianAtEpochKRPC", "epochUT"] {
      pa.removeValue(forKey: key); pb.removeValue(forKey: key)
    }
    guard NSDictionary(dictionary: pa).isEqual(to: pb) else { return false }
    let da = a["atmosphereCurveDigest"], db = b["atmosphereCurveDigest"]
    if let da, !(da is NSNull), let db, !(db is NSNull) {
      return NSDictionary(dictionary: ["digest": da]).isEqual(to: ["digest": db])
    }
    return true
  }
  private struct Geometry: Hashable {
    let name: String
    let coordinates: [Int64]
  }
  private struct Structure: Equatable {
    let names: [String]
    let geometry: [Geometry: Int]?
    init?(_ json: String) {
      guard let root = PhysicsIdentityMatcher.object(json),
            let parts = root["parts"] as? [[String: Any]] else { return nil }
      names = parts.map { $0["name"] as? String ?? "<unavailable>" }.sorted()
      var positions: [Geometry: Int] = [:]
      if root["geometryComplete"] as? Bool == true,
         let items = root["geometry"] as? [[String: Any]], !parts.isEmpty, items.count == parts.count {
        for item in items {
          guard let name = item["name"] as? String, let p = item["position"] as? [Double],
                p.count == 3, p.allSatisfy({ $0.isFinite && abs($0) < 1e9 }) else { return nil }
          let g = Geometry(name: name, coordinates: p.map { Int64(($0 / 0.05).rounded(.toNearestOrEven)) })
          positions[g, default: 0] += 1
        }
        geometry = positions
      } else { geometry = nil }
    }
  }
  public static func compatible(_ candidate: PhysicsContext, with current: PhysicsContext) -> Bool {
    guard environmentsMatch(candidate.environmentJSON, current.environmentJSON) else { return false }
    if candidate.structureID == current.structureID { return true }
    guard candidate.structureID.hasPrefix("sha256:"),
          let a = Structure(candidate.structureJSON), let b = Structure(current.structureJSON),
          !a.names.isEmpty, !b.names.isEmpty else { return false }
    return a == b
  }
}
