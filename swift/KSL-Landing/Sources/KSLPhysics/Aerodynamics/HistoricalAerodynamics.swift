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
  /// Kernel axes scaled so its support is the unit ball: log2 q, Mach/0.5,
  /// alpha/10 deg, beta/10 deg. With unit cells every sample inside the kernel
  /// lies in the query cell or one of its 3^4 neighbours. Cells are stored
  /// densely over the archive's own bounds (CSR offsets), so a lookup is
  /// array indexing rather than hashing on the guidance hot path.
  private let low: [Int], size: [Int], offsets: [Int32], members: [Int32]
  /// Per-member kernel coordinates and weights in CSR order (contiguous scan).
  private let mq: [Double], mm: [Double], ma: [Double], mb: [Double], mw: [Double]
  private let fx: [Double], fy: [Double], fz: [Double]
  private static func coordinates(_ gear: Bool, _ brakes: Bool, _ airbrakes: Bool,
                                  q: Double, mach: Double, alpha: Double, beta: Double) -> [Int] {
    [(gear ? 1 : 0) | (brakes ? 2 : 0) | (airbrakes ? 4 : 0), Int(floor(log2(q))),
     Int(floor(mach / 0.5)), Int(floor(degrees(alpha) / 10)), Int(floor(degrees(beta) / 10))]
  }
  public init(history: PhysicsHistory) {
    self.history = history
    var entries: [(cell: [Int], index: Int)] = []
    for (i, sample) in history.samples.enumerated() {
      let o = sample.observation
      guard o.dynamicPressure > 0, [o.mach, o.alpha, o.beta].allSatisfy(\.isFinite) else { continue }
      entries.append((Self.coordinates(o.gear, o.brakes, o.airbrakes, q: o.dynamicPressure,
        mach: o.mach, alpha: o.alpha, beta: o.beta), i))
    }
    var low = Array(repeating: 0, count: 5), high = Array(repeating: -1, count: 5)
    if let first = entries.first { low = first.cell; high = first.cell }
    for e in entries { for d in 0..<5 { low[d] = min(low[d], e.cell[d]); high[d] = max(high[d], e.cell[d]) } }
    let size = (0..<5).map { high[$0] - low[$0] + 1 }
    let count = size.reduce(1, *)
    func flat(_ c: [Int]) -> Int { (0..<5).reduce(0) { $0 * size[$1] + (c[$1] - low[$1]) } }
    var counts = [Int32](repeating: 0, count: count + 1)
    for e in entries { counts[flat(e.cell) + 1] += 1 }
    for i in 0..<count { counts[i + 1] += counts[i] }
    var fill = counts, members = [Int32](repeating: 0, count: entries.count)
    for e in entries { let f = flat(e.cell); members[Int(fill[f])] = Int32(e.index); fill[f] += 1 }
    self.low = low; self.size = size; offsets = counts; self.members = members
    let ordered = members.map { history.samples[Int($0)] }
    mq = ordered.map { log2($0.observation.dynamicPressure) }
    mm = ordered.map { $0.observation.mach / 0.5 }
    ma = ordered.map { degrees($0.observation.alpha) / 10 }
    mb = ordered.map { degrees($0.observation.beta) / 10 }
    mw = ordered.map { Double($0.independentFlights) }
    fx = ordered.map(\.forcePerDynamicPressure.x)
    fy = ordered.map(\.forcePerDynamicPressure.y)
    fz = ordered.map(\.forcePerDynamicPressure.z)
  }
  private func kernel(q: Double, mach: Double, alpha: Double, beta: Double,
                      gear: Bool, brakes: Bool, airbrakes: Bool,
                      visit: (HistoricalAeroSample, Double) -> Void) {
    guard !members.isEmpty else { return }
    let c = Self.coordinates(gear, brakes, airbrakes, q: q, mach: mach, alpha: alpha, beta: beta)
    let flags = c[0] - low[0]
    guard flags >= 0, flags < size[0] else { return }
    func range(_ d: Int) -> ClosedRange<Int>? {
      let a = max(0, c[d] - 1 - low[d]), b = min(size[d] - 1, c[d] + 1 - low[d])
      return a <= b ? a...b : nil
    }
    guard let rq = range(1), let rm = range(2), let ra = range(3), let rb = range(4) else { return }
    let samples = history.samples
    for iq in rq { for im in rm { for ia in ra {
      let base = (((flags * size[1] + iq) * size[2] + im) * size[3] + ia) * size[4]
      for ib in rb {
        let f = base + ib
        for k in Int(offsets[f])..<Int(offsets[f + 1]) {
          let sample = samples[Int(members[k])], o = sample.observation
          let lq = log2(q / o.dynamicPressure), lm = (mach - o.mach) / 0.5
          let la = degrees(alpha - o.alpha) / 10, lb = degrees(beta - o.beta) / 10
          let radius2 = lq * lq + lm * lm + la * la + lb * lb
          guard radius2 < 1 else { continue }
          visit(sample, 1 - sqrt(max(0, radius2)))
        }
      }
    }}}
  }
  /// Force-only kernel for the guidance/simulation hot path.
  public func force(q: Double, mach: Double, alpha: Double, beta: Double,
                    gear: Bool, brakes: Bool, airbrakes: Bool) -> (forcePerDynamicPressure: Vector3, localityConfidence: Double)? {
    guard q.isFinite, q > 0, mach.isFinite, alpha.isFinite, beta.isFinite, !members.isEmpty else { return nil }
    let flags = ((gear ? 1 : 0) | (brakes ? 2 : 0) | (airbrakes ? 4 : 0)) - low[0]
    guard flags >= 0, flags < size[0] else { return nil }
    let lq = log2(q), lm = mach / 0.5, la = degrees(alpha) / 10, lb = degrees(beta) / 10
    func span(_ x: Double, _ d: Int) -> (Int, Int) {
      let c = Int(floor(x)); return (max(0, c - 1 - low[d]), min(size[d] - 1, c + 1 - low[d]))
    }
    let (q0, q1) = span(lq, 1), (m0, m1) = span(lm, 2), (a0, a1) = span(la, 3), (b0, b1) = span(lb, 4)
    guard q0 <= q1, m0 <= m1, a0 <= a1, b0 <= b1 else { return nil }
    var sx = 0.0, sy = 0.0, sz = 0.0, weight = 0.0, confidence = 0.0
    mq.withUnsafeBufferPointer { mq in mm.withUnsafeBufferPointer { mm in
    ma.withUnsafeBufferPointer { ma in mb.withUnsafeBufferPointer { mb in
    offsets.withUnsafeBufferPointer { offsets in
    for iq in q0...q1 { for im in m0...m1 { for ia in a0...a1 {
      let base = (((flags * size[1] + iq) * size[2] + im) * size[3] + ia) * size[4]
      for k in Int(offsets[base + b0])..<Int(offsets[base + b1 + 1]) {
        let dq = lq - mq[k]; var r2 = dq * dq; if r2 >= 1 { continue }
        let dm = lm - mm[k]; r2 += dm * dm; if r2 >= 1 { continue }
        let da = la - ma[k]; r2 += da * da; if r2 >= 1 { continue }
        let db = lb - mb[k]; r2 += db * db; if r2 >= 1 { continue }
        let compact = 1 - sqrt(r2), w = compact * compact * mw[k]
        sx += fx[k] * w; sy += fy[k] * w; sz += fz[k] * w
        weight += w; confidence += w * compact
      }
    }}}
    }}}}}
    guard weight > 0 else { return nil }
    return (Vector3(sx, sy, sz) / weight, confidence / weight)
  }
  public func estimate(q: Double, mach: Double, alpha: Double, beta: Double,
                       gear: Bool, brakes: Bool, airbrakes: Bool) -> HistoricalForceEstimate? {
    guard let value = force(q: q, mach: mach, alpha: alpha, beta: beta,
      gear: gear, brakes: brakes, airbrakes: airbrakes) else { return nil }
    var ids: [Int64] = [], sources = Set<String>()
    kernel(q: q, mach: mach, alpha: alpha, beta: beta, gear: gear, brakes: brakes, airbrakes: airbrakes) { sample, _ in
      ids.append(contentsOf: sample.support.map(\.observationID))
      sources.formUnion(sample.support.map(\.source))
    }
    return .init(forcePerDynamicPressure: value.forcePerDynamicPressure,
                 localityConfidence: value.localityConfidence,
                 observationIDs: Array(Set(ids)).sorted(), independentFlights: sources.count)
  }
}
