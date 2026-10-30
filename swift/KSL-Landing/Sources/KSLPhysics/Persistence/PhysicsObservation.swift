import Foundation
import KSLCore

public struct PhysicsContext: Codable, Sendable, Hashable {
  public let structureID: String
  public let environmentID: String
  public let structureJSON: String
  public let environmentJSON: String
  public init(structureID: String, environmentID: String, structureJSON: String, environmentJSON: String) {
    self.structureID = structureID; self.environmentID = environmentID
    self.structureJSON = structureJSON; self.environmentJSON = environmentJSON
  }
}

public struct PhysicsObservation: Codable, Sendable {
  public let id: Int64
  public let structureID: String, environmentID: String, sessionID: String
  public let flightKey: String?, timelineEpoch: Int
  public let time: Double, dynamicPressure: Double, mach: Double
  public let alpha: Double, beta: Double, bank: Double, mass: Double
  public let gear: Bool, brakes: Bool, airbrakes: Bool, rcs: Int
  /// Original archive coordinates: kRPC body-non-rotating, left handed.
  public let position: Vector3, velocity: Vector3, lift: Vector3, drag: Vector3
  public let profile: String, bodyRates: Vector3?
  public let legacyQuality: Double, legacyEligible: Bool
  public var source: String { flightKey.flatMap { $0.isEmpty ? nil : $0 } ?? sessionID }
}

public struct HistoricalAeroSample: Codable, Sendable {
  public let observation: PhysicsObservation
  public let independentFlights: Int
  /// One medoid witness per independent flight, before cross-flight reduction.
  public let support: [ObservationWitness]
  /// Drag, lift, side-force areas (m²), in the banked wind frame. No moments.
  public let forcePerDynamicPressure: Vector3
}

public struct ObservationWitness: Codable, Sendable, Hashable {
  public let observationID: Int64
  public let sessionID: String
  public let source: String
  public let timelineEpoch: Int
  init(_ row: PhysicsObservation) {
    observationID = row.id; sessionID = row.sessionID
    source = row.source; timelineEpoch = row.timelineEpoch
  }
}

public struct PhysicsHistory: Codable, Sendable {
  public let contexts: [PhysicsContext]
  public let queriedRows: Int, rejectedRows: Int, independentSources: Int
  public let samples: [HistoricalAeroSample]
  public let excludedSession: String, excludedFlight: String
  public let evidence: String
}

struct ArchiveCell: Hashable {
  let q: Int, mach: Int, alpha: Int, beta: Int
  let gear: Bool, brakes: Bool, airbrakes: Bool
  init(_ r: PhysicsObservation) {
    // C lrint uses ties-to-even under the default rounding mode.
    q = Int(log2(r.dynamicPressure).rounded(.toNearestOrEven))
    mach = Int((r.mach * 2).rounded(.toNearestOrEven))
    alpha = Int((degrees(r.alpha) / 4).rounded(.toNearestOrEven))
    beta = Int((degrees(r.beta) / 4).rounded(.toNearestOrEven))
    gear = r.gear; brakes = r.brakes; airbrakes = r.airbrakes
  }
  func distance(to b: Self) -> Double {
    guard gear == b.gear, brakes == b.brakes, airbrakes == b.airbrakes else { return .infinity }
    return sqrt(pow(Double(q - b.q), 2) + pow(Double(mach - b.mach), 2)
      + pow(Double(alpha - b.alpha) / 2.5, 2) + pow(Double(beta - b.beta) / 2.5, 2))
  }
}
