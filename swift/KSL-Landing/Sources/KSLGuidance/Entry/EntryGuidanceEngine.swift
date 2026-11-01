import Foundation
import KSLControl
import KSLCore
import KSLPhysics

public struct EntryPlanningRequest: Sendable {
  public let state: VehicleState
  public let controls: FlightControlSnapshot?
  let memory: EntryLawMemory
  let rangeMultiplier: Double
  let previousTarget: GuidanceTarget?
  let environment: EnvironmentCalibration
}
public struct EntryPlanningProposal: Codable, Sendable {
  public let createdAt: Double
  public let rangeMultiplier: Double
  public let prediction: TrajectoryPrediction?
  public let candidates: Int
  public let terminalRouteFeasible: Bool
  /// Capture forecast of the selected branch (HAC plan stripped).
  public var capture: EntryCaptureForecast?
  public var crossAim: Double?
}

/// Fast C-derived MM304 feedback owns commands. Expensive closed-loop forecasts
/// propose a bounded range correction; they never directly overwrite actuators.
public final class EntryGuidanceEngine {
  private let configuration: MissionConfiguration
  /// A trajectory rollout flies feedback; it must not recursively launch a
  /// new whole-entry forecast from each simulated replanning epoch.
  private let permitsReplanning: Bool
  private let predictor: TrajectoryPropagationEngine
  private let law: EntryCommandLaw
  private let terminalPlanner: HACPlanningEngine
  private let frame: RunwayFrame
  private var rangeMultiplier = 1.0
  private var lastSolve = -Double.infinity
  private var lastAcceptedProposal = -Double.infinity
  public private(set) var bestPrediction: TrajectoryPrediction?
  public private(set) var candidateCount = 0
  public private(set) var captureForecast: EntryCaptureForecast?
  public var reversalCount: Int { law.memory.lateral.reversals }
  public var diagnostics: EntryDiagnostics { law.diagnostics }
  public var environment = EnvironmentCalibration()
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine, permitsReplanning: Bool = true) {
    self.configuration = configuration
    self.permitsReplanning = permitsReplanning
    law = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    predictor = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    terminalPlanner = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func request(from state: VehicleState, controls: FlightControlSnapshot? = nil,
                      previousTarget: GuidanceTarget? = nil) -> EntryPlanningRequest? {
    guard permitsReplanning,
      state.time - lastSolve >= configuration.guidance.entryReplanInterval else { return nil }
    lastSolve = state.time
    return .init(state: state, controls: controls, memory: law.memory, rangeMultiplier: rangeMultiplier,
      previousTarget: previousTarget, environment: environment)
  }
  /// Lateral aim planned backward from TAEM capture (deorbit selection).
  public func setCrossAim(_ value: Double) { law.memory.retargetCrossAim(value) }
  public func setObjective(_ objective: EntryCaptureObjective?) { law.memory.objective = objective }
  public var crossAim: Double { law.memory.crossAim }
  public func considerTerminal(_ plan: TerminalPlan?, deficit: TAEMCaptureDeficit) {
    // Only a complete, physically feasible preview may shape MM304 while
    // it keeps flight authority. An unqualified or impossible HAC is advice
    // to the planner, never an actuator reference.
    law.memory.terminalAdvice = plan.flatMap {
      $0.assessment.feasible && $0.assessment.predictionComplete != false ? $0 : nil
    }
    law.memory.terminalProjectionIndex = 0
    law.memory.terminalDeficit = deficit
  }
  public func invalidatePrediction() {
    bestPrediction = nil; candidateCount = 0; captureForecast = nil
    law.memory.objective = nil
    rangeMultiplier = 1
    lastSolve = -.infinity
    law.memory.identifiedTrimCeiling = nil
    law.memory.terminalAdvice = nil; law.memory.terminalProjectionIndex = 0
    law.memory.terminalDeficit = nil
  }
  public func accept(_ proposal: EntryPlanningProposal, at time: Double) {
    guard proposal.createdAt <= time,
      time - proposal.createdAt <= 2 * configuration.guidance.entryReplanInterval,
      proposal.createdAt > lastAcceptedProposal else { return }
    lastAcceptedProposal = proposal.createdAt
    rangeMultiplier = proposal.rangeMultiplier
    bestPrediction = proposal.prediction; candidateCount = proposal.candidates
    captureForecast = proposal.capture
    if let objective = proposal.capture?.objective { law.memory.objective = objective }
    if let aim = proposal.crossAim { law.memory.crossAim = aim }
    if let f = proposal.capture, f.feasible, let speed = f.track.last?[4],
      f.capture.envelope.availableTurnRate > 0 {
      law.memory.lateral.corridor = speed / f.capture.envelope.availableTurnRate
    } else { law.memory.lateral.corridor = nil }
  }
  public func update(_ state: VehicleState, synchronousPlanning: Bool = true,
                     controls: FlightControlSnapshot? = nil,
                     previousTarget: GuidanceTarget? = nil,
                     feedback: ControlDiagnostics? = nil) -> GuidanceTarget {
    if synchronousPlanning, let request = request(from: state, controls: controls, previousTarget: previousTarget) {
      accept(plan(request), at: state.time)
    }
    law.memory.pitchResponseSeconds = feedback?.pitchResponseSeconds
    law.memory.rollResponseSeconds = feedback?.rollResponseSeconds
    if feedback?.trimEvidence == .identified { law.memory.identifiedTrimCeiling = feedback?.trimAlphaCeiling }
    else { law.memory.identifiedTrimCeiling = nil }
    return law.update(state, rangeMultiplier: rangeMultiplier)
  }
  /// A missing terminal-region assessment is NOT evidence of excess energy.
  /// Preserve neutral range reference until an explicit physical deficit exists.
  static func holdNeutralReference(for deficits: [TAEMCaptureDeficit]) -> Bool {
    !deficits.isEmpty && deficits.allSatisfy { $0 == .terminalRegionNotReached }
  }

  /// A common, dimensionless runway residual for ranking unpriced failures.
  static func fallbackDeliveryCost(along: Double, cross: Double,
                                   configuration c: MissionConfiguration) -> Double {
    hypot(along-c.taemInterfaceAlong, cross-c.entrySettings.handoffCross)
      / max(1, c.entrySettings.terminalTargetRange)
  }

  /// Recovery candidates obey the sign of a PHYSICALLY classified energy
  /// shortage/excess. Modelled crossrange/heading failures cannot silently
  /// authorize a drag-reference energy correction.
  static func recoveryReferenceScale(for deficit: TAEMCaptureDeficit,
                                     candidates: [Double]) -> Double? {
    let valid = candidates.filter { $0.isFinite && $0 > 0 }
    guard !valid.isEmpty else { return nil }
    switch deficit {
    case .insufficientEnergy:
      // A longer reference distance asks for less drag, preserving energy.
      return valid.max()
    case .excessiveEnergy:
      // A shorter reference distance allows more energy dissipation.
      return valid.min()
    case .terminalRegionNotReached:
      // No physical energy verdict exists; do not invent one.
      return valid.min { abs($0 - 1) < abs($1 - 1) }
    default:
      return nil
    }
  }

  /// A scalar energy-reference direction is justified only when every
  /// modeled continuation agrees on the SAME explicit energy mechanism.
  /// Mixed geometry, heading, control-authority, and energy failures do not
  /// imply any signed drag correction, even if one ranking happens to win.
  static func unanimousEnergyDeficit(_ deficits: [TAEMCaptureDeficit]) -> TAEMCaptureDeficit? {
    guard let first = deficits.first,
          first == .insufficientEnergy || first == .excessiveEnergy,
          deficits.allSatisfy({ $0 == first }) else { return nil }
    return first
  }

  /// Even unanimous energy labels are not a signed physical verdict if no
  /// terminal candidate ever completed its priced HAC/final route. In that
  /// case the "insufficientEnergy" label may reflect bad heading, excessive
  /// path length or search limits. Only completed-path bracketing can
  /// authorize a signed *reference* adjustment (never a feasibility claim).
  static func supportedEnergyCorrection(_ captures: [TAEMCaptureAssessment])
    -> TAEMCaptureDeficit? {
    guard !captures.isEmpty, captures.allSatisfy({
      $0.envelope.candidatesCompleted > 0
    }) else { return nil }
    return unanimousEnergyDeficit(captures.map(\.deficit))
  }

  public func plan(_ request: EntryPlanningRequest) -> EntryPlanningProposal {
    let state = request.state, c = configuration
    let atmosphere = law.atmosphere.corrected(request.environment)
    let forecaster = EntryCaptureForecaster(configuration: c, atmosphere: atmosphere, aerodynamics: law.aero)
    // Branches span the configured aerodynamic uncertainty. The objective is
    // the TAEM capture region, judged by the same evaluator as MM305.
    let u = c.vehicle.aerodynamicUncertainty
    var factors = Array(Set([max(0.1, 1-u), 1.0, 1+u, request.rangeMultiplier])).sorted()
    var forecasts: [EntryCaptureForecast] = []
    func score(_ f: EntryCaptureForecast) -> Double {
      // Compare heading work with the turn available over the local approach
      // time. Dividing by a tiny substitute for zero turn rate overflowed to
      // infinity and made unrelated infeasible branches tie in enumeration order.
      // This bounded local screening term is not a full reachability certificate.
      let speed = max(c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed,
        f.track.last?[4] ?? forecaster.terminalSpeed(at: state.time))
      let approachTime = max(0, f.capture.rangeToRunway) / speed
      let availableHeading = max(0, f.capture.envelope.availableTurnRate) * approachTime
      let headingCost = max(0, f.capture.envelope.requiredHeadingChange - availableHeading)
        / c.entrySettings.headingHalfWidth
      let target = f.objective ?? EntryCaptureObjective.seed(configuration: c,
        atmosphere: atmosphere, frame: frame, crossAim: f.crossAim, time: state.time)
      // Price the same physically searched inlet that was actually flown by
      // the forecast. Re-seeding a 5-km target here made off-center candidates
      // appear artificially good or bad depending on their offset.
      let rangeCost = hypot(f.terminalAlong-target.along, f.terminalCross-target.cross)
        / max(1, c.entrySettings.terminalTargetRange)
      let altitudeCost = max(0, c.mm304DeliveryMinimumAltitude
        - f.capture.altitude - c.runway.elevation)
        / (c.entrySettings.terminalMaximumAltitude - c.mm304DeliveryMinimumAltitude)
      if f.feasible { return rangeCost + altitudeCost + headingCost - f.capture.envelope.energyMargin / max(1, f.capture.envelope.energyHeight) }
      let envelope = f.capture.envelope
      let energyCost = envelope.energyMargin > -Double.greatestFiniteMagnitude
        ? max(0, -envelope.energyMargin) / max(1, envelope.energyHeight)
        : abs(f.terminalAltitude - c.mm304DeliveryAltitude) / max(1, c.mm304DeliveryAltitude)
      // Infeasible candidates have no executable terminal route to price.
      // Compare their residual to the SAME runway delivery objective, not
      // each candidate's own displaced aim: otherwise a forecast can win
      // merely by moving its target to its own off-center predicted miss.
      // This is a fallback ranking, never a gate on feasible return HACs.
      let deliveryCost = Self.fallbackDeliveryCost(along: f.terminalAlong,
        cross: f.terminalCross, configuration: c)
      return (f.violations.isEmpty ? 100 : 1000 + Double(f.violations.count) * 100)
        + deliveryCost + altitudeCost + headingCost + energyCost
    }
    func ranksAhead(_ lhs: EntryCaptureForecast, _ rhs: EntryCaptureForecast) -> Bool {
      if lhs.feasible != rhs.feasible { return lhs.feasible }
      return score(lhs) < score(rhs)
    }
    func fly(_ list: [Double]) {
      for factor in list where !Task.isCancelled {
        forecasts.append(forecaster.forecast(from: state, memory: request.memory, rangeMultiplier: factor,
          controls: request.controls, previousTarget: request.previousTarget))
      }
    }
    fly(factors)
    if !forecasts.contains(where: \.feasible) {
      // Limit the extra 3-D search to one alternative on a modeled TAEM
      // crossrange miss. Each candidate uses the same unpowered force model
      // and is flown all the way to the MM305 admission survey. An advisory
      // is never interpreted as a certified landing.
      let leading = forecasts.min(by: ranksAhead)
      for aim in forecaster.lateralAims(from: state, track: leading?.track ?? [])
        where abs(aim-request.memory.crossAim) > c.entrySettings.handoffRadius && !Task.isCancelled {
        var memory = request.memory; memory.retargetCrossAim(aim)
        forecasts.append(forecaster.forecast(from: state, memory: memory,
          rangeMultiplier: request.rangeMultiplier,
          controls: request.controls, previousTarget: request.previousTarget))
      }
    }
    if !forecasts.contains(where: \.feasible),
       let energyDeficit = Self.supportedEnergyCorrection(forecasts.map(\.capture)) {
      // A completed terminal path is necessary evidence for an energy-sign
      // correction. No completed HAC is a geometry/search/energy ambiguity.
      switch energyDeficit {
      case .excessiveEnergy: factors = [max(0.1, factors.first! * (1 - u))]
      case .insufficientEnergy: factors = [factors.last! * (1 + u)]
      default: factors = []
      }
      fly(factors)
    }
    var chosen = forecasts.min(by: ranksAhead)
    // When no branch reaches MM305, the trajectory-energy *classification*
    // takes precedence over an incidental position/heading score. An
    // insufficient-energy branch may not win by commanding MORE reference
    // drag, which was repeatedly observed in the STS 72 km reproduction.
    if !forecasts.contains(where: \.feasible) {
      if let deficit = Self.supportedEnergyCorrection(forecasts.map(\.capture)),
         let reference = Self.recoveryReferenceScale(for: deficit,
           candidates: forecasts.map(\.rangeMultiplier)) {
        // Forward-simulated branches all independently found this same
        // energy condition; choose the conservative drag-reference direction.
        chosen = forecasts.min { abs($0.rangeMultiplier - reference) < abs($1.rangeMultiplier - reference) }
      } else {
        // No established energy sign: keep neutral drag, but use the real
        // forecasted position/heading/capture assessment to compare the
        // bounded lateral options. Old code picked the first neutral profile
        // and discarded every off-center alternative without comparing it.
        let neutral = forecasts.filter { abs($0.rangeMultiplier-1) < 1e-12 }
        chosen = neutral.min(by: ranksAhead)
          ?? forecasts.min { abs($0.rangeMultiplier-1) < abs($1.rangeMultiplier-1) }
      }
    }
    // Hysteresis: keep the flown branch unless it lost capture or another is
    // better by more than the model uncertainty can resolve.
    if let current = forecasts.first(where: { abs($0.rangeMultiplier - request.rangeMultiplier) < 1e-12 }),
      current.feasible, let best = chosen, best.rangeMultiplier != current.rangeMultiplier {
      let gain = best.capture.envelope.energyMargin - current.capture.envelope.energyMargin
      if gain <= u * max(abs(current.capture.envelope.energyMargin), abs(best.capture.envelope.energyMargin)) {
        chosen = current
      }
    }
    // Re-fly a selected admissible capture objective before publishing it:
    // forecast and live commands must use the very same energy/geometry goal.
    if let best = chosen, let objective = best.suggestedObjective {
      var memory = request.memory; memory.objective = objective; memory.crossAim = best.crossAim
      let consistent = forecaster.forecast(from: state, memory: memory,
        rangeMultiplier: best.rangeMultiplier, controls: request.controls, previousTarget: request.previousTarget)
      // Publish exactly the objective that this branch flew, rather than its
      // next optimizer iterate.
      var verified = consistent; verified.objective = objective
      if !ranksAhead(best, verified) { chosen = verified }
    }
    var compact = chosen
    compact?.capture.plan = nil
    var proposal = EntryPlanningProposal(createdAt: state.time,
      rangeMultiplier: chosen?.rangeMultiplier ?? request.rangeMultiplier, prediction: nil,
      // A forward entry/HAC *preview* only establishes candidate feasibility.
      // Report a landing-capable terminal route only after the SAME plan has
      // a nominally successful closed-loop continuation through rollout.
      candidates: forecasts.count,
      terminalRouteFeasible: chosen?.feasible == true && chosen?.capture.certified == true)
    proposal.capture = compact
    proposal.crossAim = chosen?.crossAim
    return proposal
  }
}
