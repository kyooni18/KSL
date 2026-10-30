import Foundation
import KSLCore

public struct HistoricalForceEstimate: Sendable {
  public let forcePerDynamicPressure: Vector3
  public let localityConfidence: Double
  public let observationIDs: [Int64]
  public let independentFlights: Int
}

/// Immutable snapshot of f1eebdc vessel_physics_aero_config's compact kernel.
/// A query outside support returns nil; a prior is never relabeled measured.
public final class HistoricalAerodynamics: Sendable {
  public let history: PhysicsHistory
  public init(history: PhysicsHistory) { self.history = history }
  public func estimate(q: Double, mach: Double, alpha: Double, beta: Double,
                       gear: Bool, brakes: Bool, airbrakes: Bool) -> HistoricalForceEstimate? {
    guard q.isFinite, q > 0, [mach, alpha, beta].allSatisfy(\.isFinite) else { return nil }
    var force = Vector3.zero, weight = 0.0, confidence = 0.0
    var ids: [Int64] = [], sources = Set<String>()
    for sample in history.samples {
      let o = sample.observation
      guard o.gear == gear, o.brakes == brakes, o.airbrakes == airbrakes else { continue }
      let radius2 = pow(log2(q / o.dynamicPressure), 2) + pow((mach - o.mach) / 0.5, 2)
        + pow(degrees(alpha - o.alpha) / 10, 2) + pow(degrees(beta - o.beta) / 10, 2)
      guard radius2 < 1 else { continue }
      let compact = 1 - sqrt(max(0, radius2))
      let w = compact * compact * Double(sample.independentFlights)
      force = force + sample.forcePerDynamicPressure * w
      weight += w; confidence += w * compact
      ids.append(contentsOf: sample.support.map(\.observationID))
      sources.formUnion(sample.support.map(\.source))
    }
    guard weight > 0 else { return nil }
    return .init(forcePerDynamicPressure: force / weight, localityConfidence: confidence / weight,
                 observationIDs: Array(Set(ids)).sorted(), independentFlights: sources.count)
  }
}
