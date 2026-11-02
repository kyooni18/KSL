import Foundation
import KSLCore
import KSLControl
import KSLGuidance
import KSLPhysics

public struct DeorbitPlanningProgress: Codable, Sendable {
  public let stage: String
  public let candidate: Int
  public let stressCase: Int
  public let totalCases: Int
  /// Progress inside one modeled flight; not elapsed live-flight telemetry.
  public var simulationElapsed: Double? = nil
  public var phase: MissionPhase? = nil
  public var casePassed: Bool? = nil
  public var runwayName: String? = nil
}

/// Owns all expensive orbit-to-stop qualification outside the operator and
/// urgent flight-control actors. Plan values carry every failed stress case.
public actor DeorbitPlanner {
  private let configuration: MissionConfiguration
  private let aero: AerodynamicsEngine
  public init(configuration: MissionConfiguration, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; aero = aerodynamics
  }
  /// Evaluate BOTH allowable runway orientations before committing to an
  /// orbital deorbit. Each branch independently plans the finite OMS burn,
  /// MM304 entry, MM305/HAC, final, touchdown and stopping. Heading-only
  /// selection at the current orbital epoch is not a landing prediction.
  ///
  /// Every runway and OMS candidate is evaluated by the same full-route
  /// planner. An authorization acknowledgement must never select a different
  /// flight-planning policy or bypass the physical path assessment.
  public static func planSelectingRunway(from source: VehicleState,
    configuration base: MissionConfiguration, aerodynamics: AerodynamicsEngine,
    allowReciprocal: Bool,
    checkpoint: (any DeorbitCaseCheckpoint)? = nil,
    progress: @escaping @Sendable (DeorbitPlanningProgress) async -> Void = { _ in })
    async throws -> DeorbitPlan {
    let configurations = allowReciprocal
      ? RunwayApproachSelector.candidates(configuration: base) : [base]
    // Share the existing global compute budget across runway branches, so a
    // slow first direction cannot prevent any examination of the reciprocal.
    let totalWidth = max(1, ProcessInfo.processInfo.activeProcessorCount - 1)
    let branchWidth = max(1, totalWidth / configurations.count)
    struct Outcome: Sendable { let index: Int; let plan: DeorbitPlan?; let error: String? }
    let evaluateRunway: @Sendable (Int, MissionConfiguration) async throws -> Outcome = { index, c in
      try Task.checkCancellation()
      let report: @Sendable (DeorbitPlanningProgress) async -> Void = { value in
        var tagged = value; tagged.runwayName = c.runway.name
        await progress(tagged)
      }
      await report(.init(stage: "runway-candidate-" + c.runway.name,
        candidate: 0, stressCase: 0, totalCases: 1))
      do {
        let plan = try await DeorbitPlanner(configuration: c, aerodynamics: aerodynamics)
          .plan(from: source, checkpoint: checkpoint, maximumConcurrentCases: branchWidth, progress: report)
        return Outcome(index: index, plan: plan, error: nil)
      } catch is CancellationError { throw CancellationError() }
      catch { return Outcome(index: index, plan: nil, error: c.runway.name + ": " + String(describing: error)) }
    }
    let outcomes: [Outcome]
    if totalWidth < configurations.count {
      var serial: [Outcome] = []
      for (index, c) in configurations.enumerated() { serial.append(try await evaluateRunway(index, c)) }
      outcomes = serial
    } else {
      outcomes = try await withThrowingTaskGroup(of: Outcome.self) { group in
        for (index, c) in configurations.enumerated() {
          group.addTask { try await evaluateRunway(index, c) }
        }
        var results: [Outcome] = []
        for try await result in group { results.append(result) }
        return results.sorted { $0.index < $1.index }
      }
    }
    let evaluated = outcomes.compactMap(\.plan)
    let errors = outcomes.compactMap(\.error)
    guard !evaluated.isEmpty else {
      let failures = errors.joined(separator: "; ")
      throw FlightError.infeasible("No candidate runway produced a complete deorbit plan: "+failures)
    }
    // Compare nominal full-route qualification first, then robust pass count
    // and safety failures. Only when all physical evidence ties use necessary
    // entry-interface heading change, then window miss, as advisory tie-breakers.
    func ranksAhead(_ x: DeorbitPlan, _ y: DeorbitPlan) -> Bool {
      if x.executionQualified != y.executionQualified { return x.executionQualified }
      if x.fullRouteQualified != y.fullRouteQualified { return x.fullRouteQualified }
      // Hard physical entry limits outrank a nominal endpoint success in the
      // provisional plant; never prefer an unsafe burn just for a rollout.
      if x.unsafeCases != y.unsafeCases { return x.unsafeCases < y.unsafeCases }
      if x.nominalPassed != y.nominalPassed { return x.nominalPassed }
      if x.passedCases != y.passedCases { return x.passedCases > y.passedCases }
      if let a=x.entryScreen, let b=y.entryScreen {
        if a.feasible != b.feasible {return a.feasible}
        if a.altitudeRejected != b.altitudeRejected {return !a.altitudeRejected}
        if a.machRejected != b.machRejected {return !a.machRejected}
        if a.captureMiss != b.captureMiss {return a.captureMiss < b.captureMiss}
      }
      let a = x.window.entryInterface.planetFixed(planet: x.configuration.planet,
        attitude: .init())
      let b = y.window.entryInterface.planetFixed(planet: y.configuration.planet,
        attitude: .init())
      let first = RunwayApproachSelector.assess(a, configuration: x.configuration)
      let second = RunwayApproachSelector.assess(b, configuration: y.configuration)
      if abs(first.minimumHeadingTravel-second.minimumHeadingTravel) > radians(0.1) {
        return first.minimumHeadingTravel < second.minimumHeadingTravel
      }
      return x.window.miss < y.window.miss
    }
    // A missed/unqualified MM305 or rollout forecast is not proof that an
    // orbital entry window is unusable. Choose the best physically screened
    // runway/OMS continuation and carry its unqualified terminal evidence
    // forward. Do NOT mark any failed continuation as a successful landing.
    // Actual burn authority, window freshness and propulsion limits remain
    // checked by DeorbitPlan, DeorbitRuntime and KRPCFlightIO.
    let safeEntry = evaluated.filter { $0.hasExecutableEntryWindow }
    guard let choice = safeEntry.sorted(by: ranksAhead).first else {
      throw FlightError.infeasible(
        "No physically admissible OMS-to-entry window for either runway. "
        + errors.joined(separator: "; "))
    }
    return choice
  }

  /// A failed full-route flight cannot be qualified, but its approximate
  /// position and speed at the MM304/MM305 interface are still meaningful.
  /// Prefer the candidate closest to the actual high-altitude TAEM handoff,
  /// not the one matching a fixed distance at the 70-km atmosphere boundary.
  struct InfeasibleApproachRank: Comparable {
    let hardUnsafe: Bool
    let screenFeasible: Bool
    let altitudeRejected: Bool
    let machRejected: Bool
    let captureMiss: Double
    let violationCount: Int
    let entryMiss: Double
    static func < (lhs: Self, rhs: Self) -> Bool {
      if lhs.hardUnsafe != rhs.hardUnsafe { return !lhs.hardUnsafe }
      if lhs.screenFeasible != rhs.screenFeasible { return lhs.screenFeasible }
      if lhs.altitudeRejected != rhs.altitudeRejected { return !lhs.altitudeRejected }
      if lhs.machRejected != rhs.machRejected { return !lhs.machRejected }
      if lhs.captureMiss != rhs.captureMiss { return lhs.captureMiss < rhs.captureMiss }
      if lhs.violationCount != rhs.violationCount { return lhs.violationCount < rhs.violationCount }
      return lhs.entryMiss < rhs.entryMiss
    }
  }

  /// A qualifying complete matrix always outranks an unsafe or insufficient
  /// matrix, regardless of its raw pass count or geometric entry miss.
  struct NominalRouteRank: Comparable {
    let qualified: Bool
    let unsafe: Int
    let passed: Int
    let miss: Double
    static func < (lhs: Self, rhs: Self) -> Bool {
      if lhs.qualified != rhs.qualified { return lhs.qualified }
      if lhs.unsafe != rhs.unsafe { return lhs.unsafe < rhs.unsafe }
      if lhs.passed != rhs.passed { return lhs.passed > rhs.passed }
      return lhs.miss < rhs.miss
    }
  }

  /// Preview the same controlled OMS and entry-pointing sequence used by
  /// full-route qualification. The ideal finite-burn window's snapped
  /// attitude and absent authority/inertia are not an executable entry state.
  public nonisolated static func nominalScreenEntry(configuration: MissionConfiguration,
    source: VehicleState, window: DeorbitWindow,
    propulsion: PropulsionTelemetry) throws -> VehicleState {
    try OrbitalExecutionModel(configuration: configuration).predict(source: source,
      window: window, propulsion: propulsion).entry
  }
  static func infeasibleApproachRank(track: [[Double]], hardUnsafe: Bool,
                                    screenFeasible: Bool, violationCount: Int,
                                    entryMiss: Double, configuration c: MissionConfiguration,
                                    terminalSpeed: Double) -> InfeasibleApproachRank {
    let e = c.entrySettings
    let targetHeight = c.mm304DeliveryAltitude-c.runway.elevation
    // The track is runway-frame [UT, along, cross, height, speed, alpha, bank].
    let point = track.filter { $0.count >= 5 && $0[1...4].allSatisfy(\.isFinite) }
      .min { abs($0[3]-targetHeight) < abs($1[3]-targetHeight) }
    guard let point else {
      return .init(hardUnsafe: hardUnsafe, screenFeasible: screenFeasible,
        altitudeRejected: true, machRejected: true, captureMiss: .greatestFiniteMagnitude,
        violationCount: violationCount, entryMiss: entryMiss)
    }
    let mach = point[4]*c.mm304DeliveryMach/max(1, terminalSpeed)
    return .init(hardUnsafe: hardUnsafe, screenFeasible: screenFeasible,
      altitudeRejected: abs(point[3]-targetHeight) > e.handoffRadius,
      machRejected: abs(mach-c.mm304DeliveryMach) > e.terminalMachHalfWidth,
      captureMiss: hypot(point[1]-c.taemInterfaceAlong, point[2]-e.handoffCross),
      violationCount: violationCount, entryMiss: entryMiss)
  }
  public func plan(from source: VehicleState,
    checkpoint: (any DeorbitCaseCheckpoint)? = nil,
    maximumConcurrentCases: Int? = nil,
    progress: @escaping @Sendable (DeorbitPlanningProgress) async -> Void = { _ in }) async throws -> DeorbitPlan {
    try source.validate(configuration: configuration)
    guard let propulsion = source.propulsion, propulsion.evidence == .measured else {
      throw FlightError.invalidTelemetry("Deorbit requires measured OMS propulsion telemetry")
    }
    try propulsion.validate(mass: source.mass)
    await progress(.init(stage: "orbital-window-search", candidate: 0, stressCase: 0, totalCases: 1))
    let search = try DeorbitWindowSearch(configuration: configuration).search(from: source, propulsion: propulsion)
    // Do not rank orbital burns by entry-interface range or heading alone.
    // This is the same EntryCaptureForecaster -> EntryCommandLaw -> polar /
    // AoA+bank force allocator that the live MM304 and its forecasts fly.
    // It supplies an MM305 feasibility screen. The following one-case
    // nominal continuation is a separate modeled control/path check, not a
    // live asynchronous-guidance or actual KSP runway-stop certificate.
    let atmosphere = try AtmosphericModelEngine(planet: configuration.planet)
    let screen = EntryCaptureForecaster(configuration: configuration, atmosphere: atmosphere,
      aerodynamics: aero)
    // Each screen is an independent whole-entry forecast. Run them in
    // parallel (one forecaster per task) instead of serially per candidate.
    struct Screened: Sendable { let index: Int; let window: DeorbitWindow
      let forecast: EntryCaptureForecast?; let error: String? }
    let c = configuration, a = aero, candidates = search.candidates
    let screenWidth = max(1, (maximumConcurrentCases ?? ProcessInfo.processInfo.activeProcessorCount))
    await progress(.init(stage: "shared-MM304-TAEM-forecast",
      candidate: 0, stressCase: 0, totalCases: candidates.count))
    let screened = try await withThrowingTaskGroup(of: Screened.self) { group in
      var results: [Screened] = [], next = 0
      func add() {
        let index = next, window = candidates[index]; next += 1
        group.addTask {
          try Task.checkCancellation()
          do {
            let entry = try Self.nominalScreenEntry(configuration: c,
              source: source, window: window, propulsion: propulsion)
            let forecaster = EntryCaptureForecaster(configuration: c, atmosphere: atmosphere, aerodynamics: a)
            return .init(index: index, window: window, forecast: forecaster.forecast(from: entry), error: nil)
          } catch is CancellationError { throw CancellationError() }
          catch { return .init(index: index, window: window, forecast: nil, error: "candidate \(index+1): \(error)") }
        }
      }
      while next < min(screenWidth, candidates.count) { add() }
      for try await result in group {
        results.append(result)
        await progress(.init(stage: "shared-MM304-TAEM-forecast",
          candidate: results.count, stressCase: 0, totalCases: candidates.count))
        if next < candidates.count { add() }
      }
      return results.sorted { $0.index < $1.index }
    }
    var entryForecasts = screened.compactMap { s in s.forecast.map { (window: s.window, forecast: $0) } }
    let executionFailures = screened.compactMap(\.error)
    guard !entryForecasts.isEmpty else {
      throw FlightError.infeasible("No candidate reached entry under controlled OMS execution: "
        + executionFailures.joined(separator: "; "))
    }
    entryForecasts.sort { a, b in
      if a.forecast.feasible != b.forecast.feasible { return a.forecast.feasible }
      if a.forecast.reachedTerminalRegion != b.forecast.reachedTerminalRegion {
        return a.forecast.reachedTerminalRegion
      }
      // An unreachable HAC is not a valid burn target. When no candidate
      // is feasible, try the smallest modeled inlet miss first, not the
      // closest nominal 70-km entry-interface ground track. The miss is
      // measured from the same TAEM interface objective MM304 flies to.
      let target = configuration.entrySettings
      let along = configuration.taemInterfaceAlong
      let missA = hypot(a.forecast.terminalAlong - along,
        a.forecast.terminalCross - target.handoffCross)
      let missB = hypot(b.forecast.terminalAlong - along,
        b.forecast.terminalCross - target.handoffCross)
      if missA != missB { return missA < missB }
      return a.window.miss < b.window.miss
    }
    // Evaluate only the recorded, unmodified vehicle and OMS configuration.
    // Do not generate alternate lift, drag, thrust, mass or timing plants.
    // Candidate windows differ in the actual maneuver plan, not vessel physics.
    let defaultWidth = max(1, ProcessInfo.processInfo.activeProcessorCount - 1)
    let width = max(1, min(defaultWidth, maximumConcurrentCases ?? defaultWidth))
    var selected: DeorbitWindow?, selectedCases: [DeorbitStressCase] = []
    var bestPassedRank: NominalRouteRank?
    var bestFailureRank: InfeasibleApproachRank?
    var hasQualifiedNominal = false
    candidateSearch: for batchStart in stride(from: 0, to: entryForecasts.count, by: width) {
      let end = min(batchStart + width, entryForecasts.count)
      let batch = (batchStart..<end).map { i in
        // The fast preview already ran the real MM304 law to the inlet.
        // The full native continuation must not synchronously solve several
        // whole-entry predictions inside each 5-second simulated epoch: the
        // live controller uses independently scheduled, expiring workers,
        // not this recursively blocking path. Evaluate the measured plant
        // and fallback entry law through actual terminal touchdown/stop;
        // separately acknowledge missing async plan acceptance evidence.
        CaseRequest(window: entryForecasts[i].window,
          index: 0, candidateIndex: i+1, entryReplanning: false)
      }
      for i in batchStart..<end {
        await progress(.init(stage: "native-orbit-to-rollout",
          candidate: i+1, stressCase: 0, totalCases: 1))
      }
      let nominalResults = try await Self.evaluateBatch(configuration: configuration,
        aero: aero, source: source, propulsion: propulsion,
        requests: batch, concurrency: width, checkpoint: checkpoint, progress: progress)
      for (offset, nominal) in nominalResults.enumerated() {
        let i = batchStart + offset
        let candidate = entryForecasts[i].window
        let cases = [nominal]
        if nominal.passed {
          let rank = NominalRouteRank(qualified: !nominal.hardUnsafe,
            unsafe: nominal.hardUnsafe ? 1 : 0, passed: 1, miss: candidate.miss)
          if bestPassedRank == nil || rank < bestPassedRank! {
            selected = candidate; selectedCases = cases; bestPassedRank = rank
          }
          hasQualifiedNominal = true
        } else if !hasQualifiedNominal {
          let rank = Self.infeasibleApproachRank(track: entryForecasts[i].forecast.track,
            hardUnsafe: nominal.hardUnsafe, screenFeasible: entryForecasts[i].forecast.feasible,
            violationCount: nominal.violations.count, entryMiss: candidate.miss,
            configuration: configuration,
            terminalSpeed: screen.terminalSpeed(at: candidate.entryInterface.time))
          if bestFailureRank == nil || rank < bestFailureRank! {
            selected = candidate; selectedCases = cases; bestFailureRank = rank
          }
        }
        if nominal.passed && !nominal.hardUnsafe { break candidateSearch }
      }
    }
    guard let window = selected else {
      throw FlightError.infeasible("No candidate completed native trajectory evaluation")
    }
    let forecast=entryForecasts.first { $0.window.burnStart==window.burnStart
      && $0.window.normalAngle==window.normalAngle && $0.window.normalDeltaV==window.normalDeltaV }!.forecast
    let rank=Self.infeasibleApproachRank(track:forecast.track,hardUnsafe:false,
      screenFeasible:forecast.feasible,violationCount:forecast.violations.count,entryMiss:window.miss,
      configuration:configuration,terminalSpeed:screen.terminalSpeed(at:window.entryInterface.time))
    let retained=DeorbitEntryScreen(feasible:forecast.feasible,altitudeRejected:rank.altitudeRejected,
      machRejected:rank.machRejected,captureMiss:rank.captureMiss,terminalAlong:forecast.terminalAlong,
      terminalCross:forecast.terminalCross,terminalAltitude:forecast.terminalAltitude,terminalMach:forecast.terminalMach)
    return .init(source: source, configuration: configuration, propulsion: propulsion, window: window,
      cases: selectedCases, modelLimitations: [
        "The inherited STS-N subsonic pitch fit does not validate STS entry moments.",
        "Roll/yaw restoring and damping coefficients lack an accepted STS high-q identification.",
        "Ground gear/tyre dynamics are provisional; no native KSP rollout qualification has been recorded.",
        "Nominal continuation uses default MM304 feedback after its initial screened window; live asynchronous MM304 plan acceptance/freshness remains unvalidated.",
        "Offline terminal planning assumes completion at the requested simulation epoch."
      ], evaluatedWindows: search.evaluated, entryScreen:retained)
  }
  struct CaseRequest: Sendable {
    let window: DeorbitWindow
    let index: Int
    let candidateIndex: Int
    var entryReplanning = false
  }

  /// Bounded structured concurrency: each flight owns its own vehicle plant,
  /// guidance memory and controller. Return results in the input order.
  nonisolated static func evaluateBatch(
    configuration c: MissionConfiguration, aero: AerodynamicsEngine,
    source: VehicleState, propulsion: PropulsionTelemetry,
    requests: [CaseRequest], concurrency: Int,
    checkpoint: (any DeorbitCaseCheckpoint)? = nil,
    progress: @escaping @Sendable (DeorbitPlanningProgress) async -> Void
  ) async throws -> [DeorbitStressCase] {
    guard !requests.isEmpty else { return [] }
    return try await withThrowingTaskGroup(
      of: (Int, DeorbitStressCase).self, returning: [DeorbitStressCase].self
    ) { group in
      let initial = min(max(1, concurrency), requests.count)
      for i in 0..<initial {
        let work = requests[i]
        group.addTask {
          let result = try await Self.evaluateCheckpointed(configuration: c, aero: aero,
            source: source, propulsion: propulsion, work: work, checkpoint: checkpoint, progress: progress)
          return (i, result)
        }
      }
      var ordered = [DeorbitStressCase?](repeating: nil, count: requests.count)
      var next = initial
      while let (position, result) = try await group.next() {
        ordered[position] = result
        if next < requests.count {
          let i = next
          let work = requests[i]
          next += 1
          group.addTask {
            let result = try await Self.evaluateCheckpointed(configuration: c, aero: aero,
              source: source, propulsion: propulsion, work: work, checkpoint: checkpoint, progress: progress)
            return (i, result)
          }
        }
      }
      return ordered.compactMap { $0 }
    }
  }

  private nonisolated static func evaluateCheckpointed(
    configuration: MissionConfiguration, aero: AerodynamicsEngine,
    source: VehicleState, propulsion: PropulsionTelemetry, work: CaseRequest,
    checkpoint: (any DeorbitCaseCheckpoint)?,
    progress: @escaping @Sendable (DeorbitPlanningProgress) async -> Void
  ) async throws -> DeorbitStressCase {
    try Task.checkCancellation()
    let identity = DeorbitCaseIdentity(configuration: configuration, source: source,
      window: work.window, index: work.index, entryReplanning: work.entryReplanning)
    if let result = try await checkpoint?.load(identity) {
      guard result.index == work.index else {
        throw FlightError.invalidConfiguration("Checkpoint case index does not match its identity")
      }
      var status = DeorbitPlanningProgress(stage: "native-case-resumed",
        candidate: work.candidateIndex, stressCase: work.index, totalCases: 1)
      status.casePassed = result.passed
      await progress(status)
      return result
    }
    await progress(.init(stage: "native-case-started", candidate: work.candidateIndex,
      stressCase: work.index, totalCases: 1))
    let result = try await evaluate(configuration: configuration, aero: aero, source: source,
      window: work.window, propulsion: propulsion,
      index: work.index, candidateIndex: work.candidateIndex, progress: progress,
      entryReplanning: work.entryReplanning)
    // Completed results survive a sibling's cancellation or a later failure.
    try await checkpoint?.save(identity, result: result)
    return result
  }

  nonisolated static func evaluate(
    configuration c: MissionConfiguration, aero: AerodynamicsEngine,
    source: VehicleState, window: DeorbitWindow, propulsion: PropulsionTelemetry,
    index: Int, candidateIndex: Int,
    progress: @escaping @Sendable (DeorbitPlanningProgress) async -> Void,
    entryReplanning: Bool = false
  ) async throws -> DeorbitStressCase {
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    var burned = false, entryAngle: Double?, maxQ = 0.0, maxLoad = 0.0
    var event: TouchdownEvent?, stoppedAlong: Double?, failures: [ConstraintViolation] = []
    var passed = false, elapsed = 0.0
    var reportedPhase: MissionPhase?
    let progressClock = ContinuousClock()
    var lastProgress = progressClock.now
    do {
      let orbital = try OrbitalExecutionModel(configuration: c).predict(source: source,
        window: window, propulsion: propulsion)
      burned = orbital.burnCompleted; entryAngle = orbital.entry.flightPathAngle
      if orbital.entry.flightPathAngle < c.deorbitSettings.minimumEntryFlightPath {
        failures.append(.init(.invalidGeometry, "Descending entry is steeper than the configured hard limit",
          value: orbital.entry.flightPathAngle, limit: c.deorbitSettings.minimumEntryFlightPath))
      } else {
        let atmosphere = try AtmosphericModelEngine(planet: c.planet)
        // The cheap screen suppresses entry replanning. A separate
        // confirmation explicitly enables it; individual forecasts still
        // suppress recursion within their own rollout.
        let computer = try FlightComputer(configuration: c, atmosphere: atmosphere,
          aerodynamics: aero, entryReplanning: entryReplanning)
        let plant = ShuttlePhysicsEngine(configuration: c,
          atmosphere: try .init(planet: c.planet), aerodynamics: aero,
          initialControls: orbital.entry.appliedControls ?? .init())
        var state = plant.observe(orbital.entry)
        while state.time-orbital.entry.time < c.guidance.maximumPredictionDuration {
          try Task.checkCancellation()
          let record = try computer.step(state)
          elapsed = state.time-orbital.entry.time
          let now = progressClock.now
          if reportedPhase != record.guidance.phase || lastProgress.duration(to: now) >= .seconds(1) {
            var status = DeorbitPlanningProgress(stage: "native-rollout-progress",
              candidate: candidateIndex, stressCase: index, totalCases: 1)
            status.simulationElapsed = elapsed; status.phase = record.guidance.phase
            await progress(status)
            reportedPhase = record.guidance.phase; lastProgress = now
          }
          maxQ = max(maxQ, state.aerodynamics.dynamicPressure)
          maxLoad = max(maxLoad, abs(state.aerodynamics.lift)/(state.mass*9.80665))
          if record.guidance.phase == .complete {
            passed = plant.touchdown?.acceptable == true
            stoppedAlong = frame.coordinates(of: state.position).along; break
          }
          if record.guidance.phase == .abort { failures += record.guidance.violations; break }
          state = try plant.advance(state, command: record.command, duration: c.guidance.controlInterval, maximumStep: 0.02)
          if state.contact == .crashed { failures.append(.init(.contactQuality, "Full native route failed ground contact or runway containment")); break }
        }
        event = plant.touchdown
        if !passed && failures.isEmpty { failures.append(.init(.timeout, "Native trajectory did not complete touchdown and rollout")) }
      }
    } catch is CancellationError { throw CancellationError() }
    catch { failures.append(.init(burned ? .actuatorAuthority : .invalidGeometry, String(describing: error))) }
    let hardUnsafe = !burned || entryAngle == nil || (entryAngle ?? 0) < c.deorbitSettings.minimumEntryFlightPath
      || maxQ > c.vehicle.maximumDynamicPressure || maxLoad > c.vehicle.maximumLoadFactor
    var status = DeorbitPlanningProgress(stage: "native-rollout-result",
      candidate: candidateIndex, stressCase: index, totalCases: 1)
    status.simulationElapsed = elapsed; status.phase = reportedPhase; status.casePassed = passed
    await progress(status)
    return .init(index: index, passed: passed, hardUnsafe: hardUnsafe, burnCompleted: burned,
      entryFlightPath: entryAngle, maximumDynamicPressure: maxQ, maximumLoad: maxLoad,
      touchdownAlong: event?.along, touchdownCross: event?.cross, touchdownSink: event?.verticalSpeed,
      touchdownPitch: event?.pitch, stoppedAlong: stoppedAlong, violations: failures,
      evidence: entryReplanning ? "offline-native-synchronous-entry-replanning-rollout"
        : "offline-native-feedback-rollout-without-recursive-entry-search")
  }
}
