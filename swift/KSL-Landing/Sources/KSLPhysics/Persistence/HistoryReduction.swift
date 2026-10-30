import Foundation
import KSLCore

/// Literal C history/reduction contracts: per-flight medoid, cross-flight
/// medoid, then topology-preserving farthest-point sampling. No frame-count trust.
enum HistoryReduction {
  private struct SourceCell: Hashable { let source: String; let cell: ArchiveCell }
  private struct Selected {
    let row: PhysicsObservation
    let support: [ObservationWitness]
    var independent: Int { support.count }
  }
  private static func tied(_ a: Double, _ b: Double) -> Bool {
    abs(a - b) <= Double.ulpOfOne * max(1, abs(a), abs(b))
  }
  private static func medoid(_ rows: [PhysicsObservation], newer: Bool) -> PhysicsObservation {
    func median(_ values: [Double]) -> Double {
      let s = values.sorted(), m = s.count / 2
      return s.count % 2 == 0 ? (s[m - 1] + s[m]) / 2 : s[m]
    }
    let lift = median(rows.map { $0.lift.length / $0.dynamicPressure })
    let drag = median(rows.map { $0.drag.length / $0.dynamicPressure })
    var best = rows[0], score = Double.infinity
    for r in rows {
      let candidate = hypot(r.lift.length / r.dynamicPressure - lift, r.drag.length / r.dynamicPressure - drag)
      if candidate < score || (tied(candidate, score) && (newer ? r.id > best.id : r.id < best.id)) {
        best = r; score = candidate
      }
    }
    return best
  }
  private static func regime(_ mach: Double) -> Int {
    let anchors = [0.35, 1.05, 2.6, 6.0]
    for i in 0..<3 where mach < (anchors[i] + anchors[i + 1]) / 2 { return i }
    return 3
  }
  private static func sameTopology(_ a: Selected, _ b: Selected) -> Bool {
    a.row.gear == b.row.gear && a.row.brakes == b.row.brakes && a.row.airbrakes == b.row.airbrakes
      && regime(a.row.mach) == regime(b.row.mach)
  }
  private static func better(_ a: Selected, _ b: Selected) -> Bool {
    a.independent != b.independent ? a.independent > b.independent : a.row.id > b.row.id
  }
  static func reduce(_ rows: [PhysicsObservation], maximum: Int, planet: PlanetConfiguration)
    -> [HistoricalAeroSample] {
    guard !rows.isEmpty else { return [] }
    let bySource = Dictionary(grouping: rows) { SourceCell(source: $0.source, cell: ArchiveCell($0)) }
    var seenSources = Set<SourceCell>(), candidates: [PhysicsObservation] = []
    for row in rows {
      let key = SourceCell(source: row.source, cell: ArchiveCell(row))
      if seenSources.insert(key).inserted { candidates.append(medoid(bySource[key]!, newer: false)) }
    }
    let byCell = Dictionary(grouping: candidates, by: ArchiveCell.init)
    var seenCells = Set<ArchiveCell>(), selected: [Selected] = []
    for row in candidates {
      let key = ArchiveCell(row)
      if seenCells.insert(key).inserted {
        let values = byCell[key]!
        selected.append(.init(row: medoid(values, newer: true), support: values.map(ObservationWitness.init)))
      }
    }
    var retained: [Int] = []
    if selected.count <= maximum { retained = Array(selected.indices) }
    else {
      for i in selected.indices where retained.count < maximum {
        if retained.contains(where: { sameTopology(selected[$0], selected[i]) }) { continue }
        var best = i
        for j in selected.indices where sameTopology(selected[j], selected[i]) {
          if better(selected[j], selected[best]) { best = j }
        }
        retained.append(best)
      }
      var included = Set(retained)
      while retained.count < maximum {
        var best: Int?, distance = -Double.infinity
        for i in selected.indices where !included.contains(i) {
          let cell = ArchiveCell(selected[i].row)
          let nearest = retained.map { cell.distance(to: ArchiveCell(selected[$0].row)) }.min() ?? .infinity
          if best == nil || nearest > distance || (tied(nearest, distance) && better(selected[i], selected[best!])) {
            best = i; distance = nearest
          }
        }
        guard let best else { break }
        included.insert(best); retained.append(best)
      }
    }
    return retained.compactMap { index in
      let cell = selected[index], r = cell.row
      func canonical(_ v: Vector3) -> Vector3 { .init(v.x, v.z, v.y) }
      let position = canonical(r.position)
      let air = canonical(r.velocity) - planet.angularVelocity.cross(position)
      guard air.length > 1 else { return nil }
      let direction = air.normalized
      let up = position.projected(onPlaneWithNormal: direction).normalized
      let side = direction.cross(up).normalized
      let liftAxis = up * cos(r.bank) + side * sin(r.bank)
      let lateral = side * cos(r.bank) - up * sin(r.bank)
      let force = canonical(r.lift + r.drag)
      let area = Vector3(-force.dot(direction), force.dot(liftAxis), force.dot(lateral)) / r.dynamicPressure
      guard area.isFinite, area.x >= 0 else { return nil }
      return .init(observation: r, independentFlights: cell.independent,
                   support: cell.support, forcePerDynamicPressure: area)
    }
  }
}
