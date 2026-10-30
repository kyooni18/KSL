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
  public func invalidatePrediction() {
    bestPrediction = nil; candidateCount = 0; captureForecast = nil
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
  }
  public func update(_ state: VehicleState, synchronousPlanning: Bool = true,
                     controls: FlightControlSnapshot? = nil,
                     previousTarget: GuidanceTarget? = nil,
                     feedback: ControlDiagnostics? = nil) -> GuidanceTarget {
    if synchronousPlanning, let request = request(from: state, controls: controls, previousTarget: previousTarget) {
      accept(plan(request), at: state.time)
    }
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
      if f.feasible { return -f.capture.envelope.energyMargin / max(1, f.capture.envelope.energyHeight) }
      return (f.violations.isEmpty ? 100 : 1000 + Double(f.violations.count) * 100)
        + abs(f.terminalAltitude - c.entrySettings.terminalTargetAltitude) / max(1, c.entrySettings.terminalTargetAltitude)
    }
    func fly(_ list: [Double]) {
      for factor in list where !Task.isCancelled {
        forecasts.append(forecaster.forecast(from: state, memory: request.memory, rangeMultiplier: factor,
          controls: request.controls, previousTarget: request.previousTarget))
      }
    }
    fly(factors)
    if !forecasts.contains(where: \.feasible), let best = forecasts.min(by: { score($0) < score($1) }) {
      // Correct early, in the direction the physics asks: a shorter
      // perceived range commands more drag (excess energy), a longer one less.
      switch best.capture.deficit {
      case .excessiveEnergy, .terminalRegionNotReached: factors = [max(0.1, factors.first! * (1 - u))]
      case .insufficientEnergy: factors = [factors.last! * (1 + u)]
      default: factors = []
      }
      fly(factors)
    }
    let chosen = forecasts.min { score($0) < score($1) }
    var compact = chosen
    if let f = chosen {
      var capture = f.capture; capture.plan = nil
      compact = .init(createdAt: f.createdAt, rangeMultiplier: f.rangeMultiplier, liftScale: f.liftScale,
        dragScale: f.dragScale, reachedTerminalRegion: f.reachedTerminalRegion, violations: f.violations,
        terminalTime: f.terminalTime, terminalAltitude: f.terminalAltitude, terminalMach: f.terminalMach,
        terminalAlong: f.terminalAlong, terminalCross: f.terminalCross,
        maximumDynamicPressure: f.maximumDynamicPressure, maximumLoad: f.maximumLoad, capture: capture,
        track: f.track)
    }
    var proposal = EntryPlanningProposal(createdAt: state.time,
      rangeMultiplier: chosen?.rangeMultiplier ?? request.rangeMultiplier, prediction: nil,
      candidates: forecasts.count, terminalRouteFeasible: chosen?.feasible == true)
    proposal.capture = compact
    return proposal
  }
}
