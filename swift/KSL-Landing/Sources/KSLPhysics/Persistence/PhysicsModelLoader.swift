import Foundation
import KSLCore

public enum PhysicsModelLoader {
  /// C manual model identity is authoritative. Configuration variants are
  /// explicit; a closed-bay data book never silently replaces base STS.
  public static func resolvingDefaultArchive(_ configuration: MissionConfiguration,
    path: String = "Runtime/Physics/observations.sqlite3", variant: String? = nil) async throws -> MissionConfiguration {
    if configuration.physicsArchive != nil { return configuration }
    let structure = "model:"+configuration.vehicle.identifier+(variant.map { "#"+$0 } ?? "")
    let store = try PhysicsObservationStore(path: path)
    let contexts = try await store.contexts().filter { $0.structureID == structure }
    var compatible: [PhysicsContext] = []
    for context in contexts {
      do { try validateEnvironment(context, planet: configuration.planet); compatible.append(context) }
      catch { continue }
    }
    guard let anchor = compatible.first,
      compatible.allSatisfy({ PhysicsIdentityMatcher.environmentsMatch($0.environmentJSON, anchor.environmentJSON) }) else {
      throw FlightError.invalidConfiguration("No unique compatible archive environment for \(structure); provide physicsArchive explicitly")
    }
    var result = configuration
    result.physicsArchive = .init(path: URL(fileURLWithPath: path).standardizedFileURL.path,
      structureID: structure, environmentID: anchor.environmentID)
    return result
  }
  public static func aerodynamics(configuration: MissionConfiguration) async throws -> AerodynamicsEngine {
    guard let archive = configuration.physicsArchive else {
      return try .init(referenceArea: configuration.vehicle.referenceArea)
    }
    let store = try PhysicsObservationStore(path: archive.path)
    let stored = try await store.contexts()
    let anchors = stored.filter {
      $0.structureID == archive.structureID && $0.environmentID == archive.environmentID
    }
    guard let anchor = anchors.first else {
      throw FlightError.invalidConfiguration("No matching physics archive structure/environment")
    }
    let contexts = [anchor] + stored.filter { $0 != anchor && PhysicsIdentityMatcher.compatible($0, with: anchor) }
    for context in contexts {
      try validateEnvironment(context, planet: configuration.planet)
    }
    let history = try await store.history(contexts: contexts, excludingSession: archive.excludedSession,
      excludingFlight: archive.excludedFlight, planet: configuration.planet)
    guard !history.samples.isEmpty else {
      throw FlightError.invalidConfiguration("Physics archive has no provenance-valid compatible samples")
    }
    return try .init(referenceArea: configuration.vehicle.referenceArea, history: .init(history: history))
  }
  public static func validateEnvironment(_ context: PhysicsContext, planet: PlanetConfiguration) throws {
    guard let data = context.environmentJSON.data(using: .utf8),
          let root = try JSONSerialization.jsonObject(with: data) as? [String: Any],
          let p = root["planet"] as? [String: Any], p["name"] as? String == "Kerbin" else {
      throw FlightError.invalidConfiguration("Invalid physics archive environment manifest")
    }
    for (name, expected) in [("radius", planet.radius), ("gravitationalParameter", planet.gravitationalParameter),
                             ("rotationalSpeed", planet.rotationRate), ("atmosphereDepth", planet.atmosphereTop)] {
      guard let actual = p[name] as? Double, actual.isFinite,
            abs(actual - expected) <= max(1, abs(expected)) * 1e-12 else {
        throw FlightError.invalidConfiguration("Physics archive planet mismatch: " + name)
      }
    }
  }
}
