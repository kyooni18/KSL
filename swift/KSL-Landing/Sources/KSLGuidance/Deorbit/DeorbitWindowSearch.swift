import Foundation
import KSLCore
import KSLPhysics

public struct DeorbitWindowSearchResult: Sendable {
  public let candidates: [DeorbitWindow]
  public let evaluated: Int
  public let horizon: Double
  public let rejections: [String: Int]
}

/// f1eebdc deorbit_planning.inc: finite-burn periapsis solve, rotating
/// ground-track window search, then normal-component refinement. Only the
/// downstream native flight-computer evaluator can qualify these windows.
public final class DeorbitWindowSearch {
  private let configuration: MissionConfiguration
  private let orbit: OrbitalMechanics
  private let burn: FiniteBurnModel
  private let frame: RunwayFrame
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration; orbit = .init(planet: configuration.planet)
    burn = .init(planet: configuration.planet)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func solveDeltaV(coast: OrbitalState, propulsion: PropulsionTelemetry, normal: Double) throws -> Double {
    let c = configuration.deorbitSettings
    let target = c.targetPeriapsisAltitude, speed = coast.velocity.length
    if try orbit.elements(coast).periapsisAltitude <= target { return 0 }
    let direction = OrbitalMechanics.burnDirection(position: coast.position, velocity: coast.velocity, normalAngle: normal)
    func impulsePeriapsis(_ dv: Double) throws -> Double {
      var s = coast; s.velocity = s.velocity+direction*dv; return try orbit.elements(s).periapsisAltitude
    }
    var low = 0.0, high = speed
    guard try impulsePeriapsis(high) <= target else { throw FlightError.infeasible("Target periapsis has no retrograde impulse bracket") }
    for _ in 0..<48 {
      let mid = (low+high)/2
      if try impulsePeriapsis(mid) > target { low = mid } else { high = mid }
    }
    func finitePeriapsis(_ dv: Double) throws -> Double {
      let result = try burn.propagate(from: coast, deltaV: dv, propulsion: propulsion, settings: c, normalAngle: normal)
      guard result.complete else { throw FlightError.infeasible("Available propulsion cannot complete the periapsis bracket burn") }
      return try orbit.elements(result.state).periapsisAltitude
    }
    low = 0
    while try finitePeriapsis(high) > target {
      low = high; high = min(speed, high*2)
      if high <= low { throw FlightError.infeasible("No finite-burn periapsis bracket") }
    }
    for _ in 0..<48 {
      try Task.checkCancellation()
      if high-low <= min(0.01, 0.125*BurnThrottleLaw.terminalDeltaVTolerance) { break }
      let mid = (low+high)/2
      if try finitePeriapsis(mid) > target { low = mid } else { high = mid }
    }
    return (low+high)/2
  }
  public func probe(initial: OrbitalState, start: Double, normal: Double,
                    propulsion: PropulsionTelemetry) throws -> DeorbitWindow {
    let c = configuration.deorbitSettings, planet = configuration.planet
    let coast = try orbit.propagate(initial, to: start, step: 12)
    let dv = try solveDeltaV(coast: coast, propulsion: propulsion, normal: normal)
    let result = try burn.propagate(from: coast, deltaV: dv, propulsion: propulsion, settings: c, normalAngle: normal)
    guard result.complete else { throw FlightError.infeasible("Finite burn incomplete") }
    guard let ei = try orbit.entryInterface(from: result.state,
      altitude: planet.atmosphereTop-c.entryAltitudeMargin, maximumDuration: 4500) else {
      throw FlightError.infeasible("No descending entry-interface crossing")
    }
    let state = ei.planetFixed(planet: planet, attitude: .init())
    let range = frame.distance(from: state.position, to: frame.origin)
    let courseError = wrapAngle(frame.bearing(from: state.position, to: frame.origin)-state.course)
    let cross = abs(asin(clamp(sin(range/planet.radius)*sin(courseError), -1, 1))*planet.radius)
    var targetRange = c.targetEntryRange
    if let reference = c.entryRangeReferenceSpeed, reference > 0 {
      let terminalSpeed = max(1, configuration.entrySettings.terminalTargetMach)*320
      let radius = planet.radius+60000, g = planet.gravitationalParameter/(radius*radius)
      let potential = 2*g*max(0, planet.atmosphereTop-c.entryAltitudeMargin-configuration.entrySettings.terminalTargetAltitude)
      let entryEnergy = state.velocity.dot(state.velocity)-terminalSpeed*terminalSpeed+potential
      let referenceEnergy = max(1, reference*reference-terminalSpeed*terminalSpeed+potential)
      targetRange *= clamp(entryEnergy/referenceEnergy, 0.7, 2)
    }
    return .init(burnStart: start, deltaV: dv, duration: result.elapsed, normalAngle: normal,
      periapsisAltitude: try orbit.elements(result.state).periapsisAltitude,
      afterBurn: result.state, entryInterface: ei, entryRange: range, entrySpeed: state.velocity.length,
      entryFlightPath: state.flightPathAngle, entryCourseError: courseError, crossrange: cross,
      miss: hypot(cross, range-targetRange))
  }
  public func search(from state: VehicleState, propulsion: PropulsionTelemetry) throws -> DeorbitWindowSearchResult {
    try configuration.validate(); try propulsion.validate(mass: state.mass)
    let initial = OrbitalState(fixed: state, planet: configuration.planet), elements = try orbit.elements(initial)
    guard let period = elements.period, initial.position.length > configuration.planet.radius+configuration.planet.atmosphereTop else {
      throw FlightError.infeasible("Deorbit planning requires a bound orbit above the atmosphere")
    }
    let c = configuration.deorbitSettings, p = configuration.planet
    let normal = elements.angularMomentum.normalized, rate = 2 * .pi / period-p.rotationRate*normal.z
    var horizon = period*c.minimumSearchOrbits
    if abs(rate) > 1e-12 { horizon = max(horizon, 2 * .pi / abs(rate)) }
    let apsisSpan = (elements.apoapsisAltitude ?? elements.periapsisAltitude)-elements.periapsisAltitude
    if p.rotationRate != 0 && (p.radius*normal.cross(.init(0, 0, 1)).length > c.captureRadius || apsisSpan > c.captureRadius) {
      horizon = max(horizon, 2 * .pi / abs(p.rotationRate))
    }
    var count = 0, rejected: [String: Int] = [:]
    func evaluate(_ start: Double, _ normal: Double) throws -> DeorbitWindow? {
      try Task.checkCancellation(); count += 1
      do { return try probe(initial: initial, start: start, normal: normal, propulsion: propulsion) }
      catch is CancellationError { throw CancellationError() }
      catch { rejected[String(describing: error), default: 0] += 1; return nil }
    }
    func better(_ a: DeorbitWindow, _ b: DeorbitWindow?) -> Bool {
      guard let b else { return true }
      if a.closing != b.closing { return a.closing }
      let ac = a.miss <= c.captureRadius, bc = b.miss <= c.captureRadius
      if ac != bc { return ac }
      if ac && abs(a.burnStart-b.burnStart) > 1200 { return a.burnStart < b.burnStart }
      if a.miss != b.miss { return a.miss < b.miss }
      if a.crossrange != b.crossrange { return a.crossrange < b.crossrange }
      return abs(a.entryFlightPath-c.targetEntryFlightPath) < abs(b.entryFlightPath-c.targetEntryFlightPath)
    }
    let start = initial.time+c.minimumPlanningLeadTime
    let samples = max(361, Int(ceil(horizon/c.coarseWindowStep))+1)
    let step = horizon/Double(samples-1)
    var best: DeorbitWindow?
    var coarse: [DeorbitWindow] = []
    for i in 0..<samples {
      if let candidate = try evaluate(start+Double(i)*step, 0) {
        coarse.append(candidate)
        if better(candidate, best) { best = candidate }
      }
    }
    guard let best else { throw FlightError.infeasible("No burn window reaches entry interface: \(rejected)") }
    // Search across distinct orbital ground-track passages. The best single
    // great-circle range/crossrange approximation is not a landing certificate:
    // the downstream controlled 3-D entry evaluator must see alternatives.
    // Preserve both minimum total miss and low-crossrange entries so a timing
    // trade against lateral control authority is available.
    var seeds: [DeorbitWindow] = [best]
    let separation = max(120, min(period * 0.20, c.captureRadius/max(1, best.entrySpeed) * 3))
    let byMiss = coarse.sorted { $0.miss < $1.miss }
    let byCrossrange = coarse.sorted {
      if $0.crossrange != $1.crossrange { return $0.crossrange < $1.crossrange }
      return $0.miss < $1.miss
    }
    // Alternate rankings: filling the budget from byMiss first leaves no
    // low-crossrange candidates for the expensive full-route evaluator.
    // Separation is shared, so a pass found by both rankings is sampled once.
    let rankings = [byMiss, byCrossrange]
    var indices = [0, 0]
    let seedBudget = min(c.candidateBudget, 6)
    while seeds.count < seedBudget {
      var added = false
      for ranking in rankings.indices where seeds.count < seedBudget {
        while indices[ranking] < rankings[ranking].count {
          let candidate = rankings[ranking][indices[ranking]]
          indices[ranking] += 1
          guard candidate.closing else { continue }
          if seeds.allSatisfy({ abs($0.burnStart-candidate.burnStart) > separation }) {
            seeds.append(candidate)
            added = true
            break
          }
        }
      }
      if !added { break }
    }
    func refine(_ original: DeorbitWindow) throws -> DeorbitWindow {
      var selected = original
      var increment = step
      for _ in 0..<9 {
        let center = selected.burnStart
        for sign in [-1.0, 1.0] {
          let time = center+sign*increment
          if time < start || time > start+horizon { continue }
          if let candidate = try evaluate(time, selected.normalAngle), better(candidate, selected) {
            selected = candidate
          }
        }
        increment /= 2
      }
      var angleStep = radians(45)
      for _ in 0..<12 {
        let center = selected.normalAngle
        for sign in [-1.0, 1.0] {
          let angle = clamp(center+sign*angleStep, -radians(45), radians(45))
          if let candidate = try evaluate(selected.burnStart, angle),
            candidate.closing && candidate.miss < selected.miss { selected = candidate }
        }
        angleStep /= 2
      }
      return selected
    }
    let refined = try seeds.map { try refine($0) }.sorted { $0.miss < $1.miss }
    var windows: [DeorbitWindow] = []
    // A local time offset can change the entry state materially; include it,
    // but not at the expense of a different viable orbital passage.
    for candidate in refined where windows.count < c.candidateBudget {
      windows.append(candidate)
    }
    // Sample both sides of each local minimum before expanding the offset
    // magnitude or moving to another pair. A budget which expires after only
    // negative offsets cannot discover a later burn that corrects overshoot.
    // Distribute pairs across passages before trying a larger magnitude.
    for magnitude in [1.0, 2.0] where windows.count < c.candidateBudget {
      for candidate in refined where windows.count < c.candidateBudget {
        let spacing = max(step, c.captureRadius/max(1, candidate.entrySpeed))
        for sign in [-1.0, 1.0] where windows.count < c.candidateBudget {
          let time = candidate.burnStart+sign*magnitude*spacing
          guard time >= start, time <= start+horizon else { continue }
          if let option = try evaluate(time, candidate.normalAngle), option.closing,
            windows.allSatisfy({ abs($0.burnStart-option.burnStart) > max(1, step * 0.25) }) {
            windows.append(option)
          }
        }
      }
    }
    return .init(candidates: windows, evaluated: count, horizon: horizon, rejections: rejected)
  }
}
