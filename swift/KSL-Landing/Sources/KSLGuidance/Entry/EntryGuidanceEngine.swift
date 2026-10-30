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
  private let predictor: TrajectoryPropagationEngine
  private let law: EntryCommandLaw
  private let terminalPlanner: HACPlanningEngine
  private let frame: RunwayFrame
  private var rangeMultiplier = 1.0
  private var lastSolve = -Double.infinity
  public private(set) var bestPrediction: TrajectoryPrediction?
  public private(set) var candidateCount = 0
  public private(set) var captureForecast: EntryCaptureForecast?
  public var reversalCount: Int { law.memory.lateral.reversals }
  public var diagnostics: EntryDiagnostics { law.diagnostics }
  public var environment = EnvironmentCalibration()
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    law = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    predictor = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    terminalPlanner = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func request(from state: VehicleState, controls: FlightControlSnapshot? = nil,
                      previousTarget: GuidanceTarget? = nil) -> EntryPlanningRequest? {
    guard state.time - lastSolve >= configuration.guidance.entryReplanInterval else { return nil }
    lastSolve = state.time
    return .init(state: state, controls: controls, memory: law.memory, rangeMultiplier: rangeMultiplier,
      previousTarget: previousTarget, environment: environment)
  }
  /// Lateral aim planned backward from TAEM capture (deorbit selection).
  public func setCrossAim(_ value: Double) { law.memory.retargetCrossAim(value) }
  public func setObjective(_ objective: EntryCaptureObjective?) { law.memory.objective = objective }
  public var crossAim: Double { law.memory.crossAim }
  public func invalidatePrediction() {
    bestPrediction = nil; candidateCount = 0; captureForecast = nil
    law.memory.objective = nil
    rangeMultiplier = 1
    lastSolve = -.infinity
    law.memory.identifiedTrimCeiling = nil; law.memory.ceiling = nil
    law.memory.deficit = 0; law.memory.dragBoost = 0
  }
  public func accept(_ proposal: EntryPlanningProposal, at time: Double) {
    guard proposal.createdAt <= time,
      time - proposal.createdAt <= 2 * configuration.guidance.entryReplanInterval else { return }
    rangeMultiplier = proposal.rangeMultiplier
    bestPrediction = proposal.prediction; candidateCount = proposal.candidates
    captureForecast = proposal.capture
    if let objective = proposal.capture?.objective { law.memory.objective = objective }
    if let aim = proposal.crossAim { law.memory.crossAim = aim }
    if let f = proposal.capture, f.feasible, let speed = f.track.last?[4] {
      let g = configuration.planet.gravitationalParameter
        / pow(configuration.planet.radius + f.capture.altitude, 2)
      law.memory.lateral.corridor = speed * speed
        / (g * tan(max(1e-3, EnergyManagementEngine.acquisitionBank(configuration))))
    }
  }
  public func update(_ state: VehicleState, synchronousPlanning: Bool = true,
                     controls: FlightControlSnapshot? = nil,
                     previousTarget: GuidanceTarget? = nil,
                     feedback: ControlDiagnostics? = nil) -> GuidanceTarget {
    if synchronousPlanning, let request = request(from: state, controls: controls, previousTarget: previousTarget) {
      accept(plan(request), at: state.time)
    }
    law.memory.pitchResponseSeconds = feedback?.pitchResponseSeconds
    if feedback?.trimEvidence == .identified { law.memory.identifiedTrimCeiling = feedback?.trimAlphaCeiling }
    else { law.memory.identifiedTrimCeiling = nil }
    return law.update(state, rangeMultiplier: rangeMultiplier)
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
      // Turning consumes distance before HAC acquisition. Compare its arc
      // length to the actual runway range, using the forecast's bank authority.
      let turningDistance = (f.track.last?[4] ?? forecaster.terminalSpeed(at: state.time))
        * f.capture.envelope.requiredHeadingChange
        / max(Double.leastNormalMagnitude, f.capture.envelope.availableTurnRate)
      let headingCost = turningDistance / max(1, f.capture.rangeToRunway)
      let target = EntryCaptureObjective.seed(configuration: c, atmosphere: atmosphere,
        frame: frame, crossAim: f.crossAim, time: state.time)
      let rangeCost = f.capture.rangeToRunway / max(1, hypot(target.along, target.cross))
      let altitudeCost = max(0, c.entrySettings.terminalMinimumAltitude
        - f.capture.altitude - c.runway.elevation)
        / (c.entrySettings.terminalMaximumAltitude - c.entrySettings.terminalMinimumAltitude)
      if f.feasible { return rangeCost + altitudeCost + headingCost - f.capture.envelope.energyMargin / max(1, f.capture.envelope.energyHeight) }
      let envelope = f.capture.envelope
      let energyCost = envelope.energyMargin > -Double.greatestFiniteMagnitude
        ? max(0, -envelope.energyMargin) / max(1, envelope.energyHeight)
        : abs(f.terminalAltitude - c.entrySettings.terminalTargetAltitude) / max(1, c.entrySettings.terminalTargetAltitude)
      return (f.violations.isEmpty ? 100 : 1000 + Double(f.violations.count) * 100)
        + rangeCost + altitudeCost + headingCost + energyCost
    }
    func fly(_ list: [Double]) {
      for factor in list where !Task.isCancelled {
        forecasts.append(forecaster.forecast(from: state, memory: request.memory, rangeMultiplier: factor,
          controls: request.controls, previousTarget: request.previousTarget))
      }
    }
    fly(factors)
    if !forecasts.contains(where: \.feasible) {
      // The flown aim lost capture: re-plan the lateral aim (same physical
      // candidates as deorbit planning) before changing the drag profile.
      for aim in forecaster.lateralAims(at: state.time) where abs(aim - request.memory.crossAim) > 1 && !Task.isCancelled {
        var memory = request.memory; memory.retargetCrossAim(aim)
        forecasts.append(forecaster.forecast(from: state, memory: memory, rangeMultiplier: request.rangeMultiplier,
          controls: request.controls, previousTarget: request.previousTarget))
      }
    }
    if !forecasts.contains(where: \.feasible), let best = forecasts.min(by: { score($0) < score($1) }) {
      // Correct early, in the direction the physics asks: a shorter
      // perceived range commands more drag (excess energy), a longer one less.
      switch best.capture.deficit {
      case .excessiveEnergy, .terminalRegionNotReached: factors = [max(0.1, factors.first! * (1 - u))]
      case .insufficientEnergy, .insufficientAltitude: factors = [factors.last! * (1 + u)]
      default: factors = []
      }
      fly(factors)
    }
    var chosen = forecasts.min { score($0) < score($1) }
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
      if score(verified) <= score(best) { chosen = verified }
    }
    var compact = chosen
    compact?.capture.plan = nil
    var proposal = EntryPlanningProposal(createdAt: state.time,
      rangeMultiplier: chosen?.rangeMultiplier ?? request.rangeMultiplier, prediction: nil,
      candidates: forecasts.count, terminalRouteFeasible: chosen?.feasible == true)
    proposal.capture = compact
    proposal.crossAim = chosen?.crossAim
    return proposal
  }
}
