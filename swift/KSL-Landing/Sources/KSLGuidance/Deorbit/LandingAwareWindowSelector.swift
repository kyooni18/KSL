import Foundation
import KSLCore
import KSLPhysics

/// One orbital deorbit option judged by its downstream landing feasibility.
public struct LandingAwareCandidate: Codable, Sendable {
  public let window: DeorbitWindow
  /// Orbital pass this option belongs to (index into the distinct seeds).
  public let pass: Int
  public let burnOffset: Double
  public let nominal: EntryCaptureForecast
  /// Plant lift/drag dispersions flown by the same nominal guidance.
  public var dispersed: [EntryCaptureForecast] = []
  public var robustPasses: Int { dispersed.filter(\.feasible).count }
  /// Capture-feasible span of burn offsets on this pass (s), when bracketed.
  public var corridor: [Double]?
  public var rejection: String? {
    nominal.feasible ? nil : "MM304 forecast ends outside TAEM capture: \(nominal.capture.deficit.rawValue)"
  }
}

public struct LandingAwareSelection: Codable, Sendable {
  /// Best option per orbital pass, best first. Infeasible passes follow all
  /// feasible ones and are never silently promoted.
  public let ranked: [LandingAwareCandidate]
  /// Every scanned option (compact: no HAC plans, no tracks).
  public let survey: [LandingAwareCandidate]
  public let evaluatedForecasts: Int
}

/// Orders deorbit windows by what happens after the burn: MM304 flown in the
/// plant to the TAEM interface, then TAEM/HAC capture. Entry-interface miss
/// distance only breaks ties among physically equivalent options.
public final class LandingAwareWindowSelector {
  private let configuration: MissionConfiguration
  private let search: DeorbitWindowSearch
  private let forecaster: EntryCaptureForecaster
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    search = .init(configuration: configuration)
    forecaster = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }

  /// Entry-interface vehicle state at the nominal entry attitude.
  public func entryState(_ window: DeorbitWindow) -> VehicleState {
    let c = configuration
    var state = window.entryInterface.planetFixed(planet: c.planet, attitude: .init())
    if let attitude = AirRelativeAttitude.target(position: state.position, airVelocity: state.velocity,
      alpha: c.entrySettings.nominalAlpha, bank: 0) { state.attitude = attitude }
    return ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics,
      initialControls: .init()).observe(state)
  }

  public func select(_ result: DeorbitWindowSearchResult, from state: VehicleState,
                     propulsion: PropulsionTelemetry) throws -> LandingAwareSelection {
    let initial = OrbitalState(fixed: state, planet: configuration.planet)
    let period = try OrbitalMechanics(planet: configuration.planet).elements(initial).period ?? 5400
    let earliest = initial.time + configuration.deorbitSettings.minimumPlanningLeadTime
    // Offsets beyond half a period belong to a different orbital passage.
    let scanStep = configuration.deorbitSettings.coarseWindowStep, maximumScan = period / 2
    // Distinct orbital passages; local time offsets are generated here.
    var seeds: [DeorbitWindow] = []
    for w in result.candidates.sorted(by: { $0.burnStart < $1.burnStart }) where w.closing {
      if seeds.allSatisfy({ abs($0.burnStart - w.burnStart) > period / 2 }) { seeds.append(w) }
    }
    var survey: [LandingAwareCandidate] = [], ranked: [LandingAwareCandidate] = [], count = 0
    func option(_ seed: DeorbitWindow, _ pass: Int, _ offset: Double, certify: Bool = false) throws -> LandingAwareCandidate? {
      try Task.checkCancellation()
      let start = seed.burnStart + offset
      guard start >= earliest,
        let w = try? search.probe(initial: initial, start: start, normal: seed.normalAngle, propulsion: propulsion),
        w.closing else { return nil }
      // The lateral aim is planned with the window: each burn option is
      // flown to each candidate aim and keeps the best capture.
      let entry = entryState(w)
      let aims = forecaster.lateralAims(at: entry.time)
      count += aims.count
      let forecasts = aims.map { forecaster.forecast(from: entry, crossAim: $0) }
      var best = forecasts.max { rank($0) < rank($1) }!
      if certify && best.feasible { count += 1; best = forecaster.forecast(from: entry, crossAim: best.crossAim, certify: true) }
      return .init(window: w, pass: pass, burnOffset: offset, nominal: best)
    }
    for (pass, seed) in seeds.enumerated() {
      // Scan burn timing, extending only in the direction the physics asks:
      // excess energy at TAEM wants a longer entry (earlier burn), deficit a
      // shorter one. Stops once the feasible corridor is bracketed.
      var scanned: [Double: LandingAwareCandidate] = [:]
      func visit(_ offset: Double) throws -> LandingAwareCandidate? {
        if let known = scanned[offset] { return known }
        let value = try option(seed, pass, offset); if let value { scanned[offset] = value }
        return value
      }
      var offset = 0.0
      _ = try visit(0)
      for direction in [-1.0, 1.0] {
        var o = 0.0, seenFeasible = scanned[0]?.nominal.feasible == true
        while abs(o) < maximumScan, let current = scanned[o] {
          if !current.nominal.feasible {
            let deficit = current.nominal.capture.deficit
            let helps = direction < 0 ? deficit != .insufficientEnergy : deficit == .insufficientEnergy
            if seenFeasible || !helps { break }
          }
          o += direction * scanStep
          guard let next = try visit(o) else { break }
          if next.nominal.feasible { seenFeasible = true }
        }
      }
      // Resolve both corridor edges down to the burn-timing uncertainty;
      // finer resolution is not executable.
      let resolution = max(configuration.deorbitSettings.timingUncertainty, 1e-3)
      for edge in scanned.filter({ $0.value.nominal.feasible }).keys.sorted() {
        for direction in [-1.0, 1.0] {
          guard let outside = scanned[edge + direction * scanStep], !outside.nominal.feasible else { continue }
          var inside = edge, out = edge + direction * scanStep
          while abs(out - inside) > resolution {
            let mid = (inside + out) / 2
            guard let probe = try visit(mid) else { break }
            if probe.nominal.feasible { inside = mid } else { out = mid }
          }
        }
      }
      let feasibleOffsets = scanned.filter { $0.value.nominal.feasible }.keys.sorted()
      var chosen: LandingAwareCandidate?
      if !feasibleOffsets.isEmpty {
        // Center of the longest contiguous feasible run maximizes the timing
        // (and therefore energy) margin to both corridor edges.
        var runs: [[Double]] = [[feasibleOffsets[0]]]
        for o in feasibleOffsets.dropFirst() {
          let previous = runs[runs.count - 1].last!
          let broken = scanned.contains { $0.key > previous && $0.key < o && !$0.value.nominal.feasible }
          if broken { runs.append([o]) } else { runs[runs.count - 1].append(o) }
        }
        let run = runs.max { ($0.last! - $0.first!) < ($1.last! - $1.first!) }!
        offset = (run.first! + run.last!) / 2
        let centered = try option(seed, pass, offset, certify: true)
        if var c = centered, c.nominal.capture.feasible {
          c.corridor = [run.first!, run.last!]; chosen = c
        } else {
          // Certification failed at the corridor center; try the scanned
          // option with the largest energy margin before giving up.
          let best = run.max { scanned[$0]!.nominal.capture.envelope.energyMargin
            < scanned[$1]!.nominal.capture.envelope.energyMargin }!
          if var c = try option(seed, pass, best, certify: true) { c.corridor = [run.first!, run.last!]; chosen = c }
          else { chosen = centered }
        }
      } else {
        chosen = scanned.values.max { $0.nominal.capture.envelope.energyMargin < $1.nominal.capture.envelope.energyMargin }
      }
      survey += scanned.values.sorted { $0.burnOffset < $1.burnOffset }.map(compact)
      guard var best = chosen else { continue }
      if best.nominal.feasible {
        let u = configuration.vehicle.aerodynamicUncertainty
        let entry = entryState(best.window)
        best.dispersed = [(1 - u, 1 + u), (1 + u, 1 - u), (1 - u, 1), (1, 1 + u)].map {
          count += 1
          return forecaster.forecast(from: entry, crossAim: best.nominal.crossAim, liftScale: $0.0, dragScale: $0.1)
        }.map(compact)
      }
      ranked.append(best)
    }
    ranked.sort(by: better)
    return .init(ranked: ranked, survey: survey, evaluatedForecasts: count)
  }

  /// Feasible captures first, then the larger energy margin.
  private func rank(_ f: EntryCaptureForecast) -> Double {
    (f.feasible ? 1e12 : 0) + max(-1e11, f.capture.envelope.energyMargin)
  }

  /// Lexicographic landing-first ordering (see deorbit window policy).
  func better(_ a: LandingAwareCandidate, _ b: LandingAwareCandidate) -> Bool {
    let x = a.nominal, y = b.nominal, v = configuration.vehicle
    if x.feasible != y.feasible { return x.feasible }
    if x.capture.certified != y.capture.certified { return x.capture.certified }
    // Differences smaller than the configured model uncertainty are not
    // physically meaningful; only larger ones decide safety and reserves.
    let u = v.aerodynamicUncertainty
    func envelopeRatio(_ f: EntryCaptureForecast) -> Double {
      max(f.maximumLoad / v.maximumLoadFactor, f.maximumDynamicPressure / v.maximumDynamicPressure)
    }
    if abs(envelopeRatio(x) - envelopeRatio(y)) > u { return envelopeRatio(x) < envelopeRatio(y) }
    let mx = x.capture.envelope.energyMargin, my = y.capture.envelope.energyMargin
    if abs(mx - my) > u * max(abs(mx), abs(my)) { return mx > my }
    if a.robustPasses != b.robustPasses { return a.robustPasses > b.robustPasses }
    if abs(a.window.deltaV - b.window.deltaV) > 0.5 { return a.window.deltaV < b.window.deltaV }
    return a.window.miss < b.window.miss
  }

  private func compact(_ f: EntryCaptureForecast) -> EntryCaptureForecast {
    var v = f; v.capture.plan = nil; v.track = []; return v
  }
  private func compact(_ c: LandingAwareCandidate) -> LandingAwareCandidate {
    var v = LandingAwareCandidate(window: c.window, pass: c.pass, burnOffset: c.burnOffset, nominal: compact(c.nominal))
    v.corridor = c.corridor; return v
  }
}
