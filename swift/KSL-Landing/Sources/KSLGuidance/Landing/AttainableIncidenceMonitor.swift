import Foundation
import KSLCore

/// Measured upper bound on the AoA the closed loop can actually hold.
///
/// The polar says how much lift an incidence WOULD produce; it does not say
/// the pitch loop can reach it. When the pitch actuator stays saturated for at
/// least the identified pitch response time while the measured AoA stays below
/// the command, the highest AoA reached during that window is evidence of the
/// sustainable nose-up equilibrium at this flight condition. Guidance then
/// prices forces on [floor, observed] instead of re-demanding the same
/// unreachable attitude. No fixed angle is assumed: the bound is released as
/// soon as the loop holds the ceiling without saturating, so a later probe can
/// re-identify it under new Mach/q.
struct AttainableIncidenceMonitor {
  private(set) var ceiling: Double?
  private var saturatedSince: Double?
  private var highestDuringSaturation = -Double.infinity

  mutating func reset() { ceiling = nil; saturatedSince = nil; highestDuringSaturation = -.infinity }

  /// - Parameters:
  ///   - commandedAlpha: the AoA guidance asked for on the previous step.
  ///   - responseSeconds: identified pitch response time; without it the
  ///     configured guidance lookahead is the only justified horizon.
  mutating func observe(time: Double, measuredAlpha: Double, commandedAlpha: Double?,
                        feedback: ControlDiagnostics?, fallbackResponseSeconds: Double) {
    guard let feedback, let commandedAlpha, measuredAlpha.isFinite else { return }
    let noseUpShortfall = commandedAlpha > measuredAlpha
    if feedback.pitchSaturated && noseUpShortfall {
      if saturatedSince == nil { saturatedSince = time; highestDuringSaturation = measuredAlpha }
      highestDuringSaturation = max(highestDuringSaturation, measuredAlpha)
      let horizon = feedback.pitchResponseSeconds ?? fallbackResponseSeconds
      if time - saturatedSince! >= horizon {
        ceiling = min(ceiling ?? .infinity, highestDuringSaturation)
      }
      return
    }
    saturatedSince = nil; highestDuringSaturation = -.infinity
    // The loop holds (or exceeds) the bound without saturating: it is no
    // longer the binding limit, so stop constraining guidance with it.
    if let bound = ceiling, !feedback.pitchSaturated, measuredAlpha >= bound, commandedAlpha >= bound {
      ceiling = nil
    }
  }

  /// Combine with an identified trim ceiling; the tighter evidence wins.
  func combined(with identified: Double?) -> Double? {
    switch (ceiling, identified) {
    case let (a?, b?): return min(a, b)
    case let (a?, nil): return a
    case let (nil, b): return b
    }
  }
}
