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
    bestPrediction = nil; candidateCount = 0
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
    let state = request.state, c = configuration, e = c.entrySettings
    let atmosphere = law.atmosphere.corrected(request.environment)
    let predictor = request.environment.observedAt == nil ? self.predictor : TrajectoryPropagationEngine(
      configuration: c, atmosphere: atmosphere, aerodynamics: law.aero)
    let terminalPlanner = request.environment.observedAt == nil ? self.terminalPlanner : HACPlanningEngine(
      configuration: c, atmosphere: atmosphere, aerodynamics: law.aero)
    let terminalPoint = frame.position(along: e.handoffAlong, cross: e.handoffCross, height: 0)
    let terminalSpeed = e.terminalTargetMach * atmosphere.sample(
      position: terminalPoint.normalized * (c.planet.radius + e.terminalTargetAltitude), time: state.time).speedOfSound
    // Range correction is bounded by the configured aerodynamic uncertainty.
    // A forecast may tune the reference, never certify a failed terminal path.
    let u = c.vehicle.aerodynamicUncertainty
    let candidates = Array(Set([max(0.1, 1-u), 1.0, 1+u, request.rangeMultiplier])).sorted()
    var best = request.rangeMultiplier, bestScore = Double.infinity
    var bestPrediction: TrajectoryPrediction?, terminalFeasible = false, count = 0
    for factor in candidates {
      if Task.isCancelled { break }
      let branch = EntryCommandLaw(configuration: c, atmosphere: atmosphere,
        aerodynamics: law.aero, memory: request.memory)
      let prediction = predictor.predict(from: state, duration: c.guidance.maximumPredictionDuration,
        step: c.guidance.predictionStep, recordEvery: 10, terminalSpeed: terminalSpeed,
        minimumDuration: state.aerodynamics.airspeed < terminalSpeed ? 4*c.guidance.entryReplanInterval : 0,
        controlSnapshot: request.controls, previousTarget: request.previousTarget) { s in
          let target = branch.update(s, rangeMultiplier: factor)
          return .init(alpha: target.angleOfAttack, bank: target.bank)
        }
      count += 1
      let end = prediction.terminalState
      let p = frame.coordinates(of: end.position)
      let range = frame.distance(from: end.position, to: terminalPoint)
      let rangeScale = max(1, e.terminalTargetRange)
      let heightScale = max(1, e.terminalMaximumAltitude-e.terminalMinimumAltitude)
      var score = pow((range-e.terminalTargetRange)/rangeScale,2)
        + pow((p.height-e.terminalTargetAltitude)/heightScale,2)
        + pow((end.aerodynamics.mach-e.terminalTargetMach)/e.terminalMachHalfWidth,2)
      let heading = wrapAngle(frame.bearing(from: end.position, to: terminalPoint)-end.course)
      score += pow(heading / .pi, 2)
      let route = prediction.reachedTerminalRegion && prediction.violations.isEmpty
        ? terminalPlanner.plan(from: end, allowDirect: c.guidance.allowDirectTerminal, preview: true).plan : nil
      if route == nil { score += 100 }
      if !prediction.violations.isEmpty { score += 1000 + Double(prediction.violations.count)*100 }
      if score < bestScore {
        bestScore = score; best = factor; bestPrediction = prediction; terminalFeasible = route?.isRobust == true
      }
    }
    return .init(createdAt: state.time, rangeMultiplier: best, prediction: bestPrediction,
      candidates: count, terminalRouteFeasible: terminalFeasible)
  }
}
