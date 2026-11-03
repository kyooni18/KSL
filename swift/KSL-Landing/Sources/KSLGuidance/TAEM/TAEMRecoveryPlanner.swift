import Foundation
import KSLCore
import KSLControl
import KSLPhysics

public struct TAEMRecoveryTrialRecord: Codable, Sendable {
  public let candidateID: String
  public let topology: TAEMCommandTopology
  public let prefixEnd: TAEMScheduleEnd
  public let evaluationStatus: TerminalEvaluationStatus?
  public let evaluationEnd: TerminalEvaluationEnd?
  public let relation: TerminalRecoveryRelation?
}
public final class TAEMRecoveryProposal: Codable, Sendable {
  public let id: String
  public let time: Double
  public let candidate: TAEMCommandCandidate?
  public let outcome: TerminalClosedLoopCase?
  public let continuation: TerminalPlan?
  public let reference: TAEMPropagatedReference?
  public let trials: [TAEMRecoveryTrialRecord]
  public let searchEnd: TAEMCommandSearchEnd
  public let failure: String?
  public init(id: String, time: Double, candidate: TAEMCommandCandidate?, outcome: TerminalClosedLoopCase?,
              continuation: TerminalPlan?, reference: TAEMPropagatedReference?, trials: [TAEMRecoveryTrialRecord],
              searchEnd: TAEMCommandSearchEnd, failure: String? = nil) {
    self.id = id; self.time = time; self.candidate = candidate; self.outcome = outcome
    self.continuation = continuation; self.reference = reference; self.trials = trials
    self.searchEnd = searchEnd; self.failure = failure
  }
}

/// Default MM305 planning lane. The urgent loop retains local coupled recovery
/// while this worker prices command schedules and their actual reached suffixes.
public enum TAEMRecoveryPlanner {
  public static func plan(state: VehicleState, configuration c: MissionConfiguration,
                          atmosphere: AtmosphericModelEngine, aerodynamics aero: AerodynamicsEngine,
                          controls: FlightControlSnapshot?, limiter: GuidanceTargetLimiter,
                          context: HACFitContext?, incumbent: [TAEMCommandSegment]?) -> TAEMRecoveryProposal {
    let id = UUID().uuidString, a = state.aerodynamics
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let p = frame.coordinates(of: state.position)
    let velocity = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
    let speed = velocity.length
    let response = max(c.guidance.pathLookaheadTime, context?.rollResponseSeconds ?? 0,
      context?.pitchResponseSeconds ?? 0)
    let objective = Vector2(-c.guidance.finalDistance, 0)-p.horizontal
    let error = atan2(velocity.cross(objective), velocity.dot(objective))
    let lateral = speed > 0 ? 2*speed*sin(error)/response : 0
    let reference = ApproachProfile(configuration: c).reference(along: -c.guidance.finalDistance)
    let gamma = atan2(reference.height-p.height, objective.length)
    let gravity = c.planet.gravitationalParameter/state.position.squaredLength
    let support = max(0, (gravity-a.airspeed*a.airspeed/state.position.length)*cos(state.flightPathAngle))
    let vertical = support+a.airspeed*(gamma-state.flightPathAngle)/response
    let ceiling = RotationalDynamicsEngine(configuration: c.vehicle, planet: c.planet)
      .supportedTrimAlpha(state, identifiedCeiling: context?.trimAlphaCeiling)
    let previous = limiter.previous ?? .init(angleOfAttack: a.angleOfAttack, bank: a.bank)
    let fit = try? TAEMNormalForceFitter.fit(vertical: vertical,
      lateral: a.dynamicPressure > c.vehicle.maximumDynamicPressure ? 0 : lateral,
      state: state, configuration: c, aerodynamics: aero, maximumAlpha: ceiling, baseTarget: previous)
    let protection = GuidanceSolution(phase: .acquisition, target: fit?.target ?? previous)
    let duration = min(c.guidance.maximumPredictionDuration, response)
    var parameters: [TAEMTopologySeedParameters] = []
    if let fit, abs(fit.target.bank) > 0, speed > 0 {
      let work = max(abs(error), abs(fit.modeled.y)*response/speed)
      if work > 0 {
        parameters = [.init(alpha: fit.target.angleOfAttack, bankMagnitude: abs(fit.target.bank),
          courseWork: work, maximumSegmentDuration: min(c.guidance.maximumPredictionDuration,
            response+(work+2 * .pi)*speed/abs(fit.modeled.y)))]
      }
    }
    do {
      let family = try TAEMTopologySeeds.generate(configuration: c, baseTarget: protection.target,
        parameters: parameters,
        incumbent: incumbent ?? [.init(duration: duration, command: .init(phase: .acquisition, target: previous))],
        protection: [.init(duration: duration, command: protection)], id: id)
      let seed = NativeCommandSeed(state: state, controls: controls, limiter: limiter)
      let search = TAEMCommandSearch(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      let continuation = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      let result = try search.search(family.candidates, from: seed,
        maximumEvaluations: family.candidates.count, cancelled: { Task.isCancelled }) { _, prefix in
          continuation.plan(from: prefix.seed.state, allowDirect: true, preview: true,
            controlSnapshot: prefix.seed.controls, previousTarget: prefix.seed.limiter.previous,
            targetLimiter: prefix.seed.limiter, nativeSeed: prefix.seed).plan
        }
      var allTrials = result.trials
      var chosen = result.incumbentTrial.map { result.trials[$0] }
      // Refine only a complete comparable outcome. Unknown tails cannot supply
      // a numeric optimizer objective. Budget derives from one full coordinate
      // stencil and binary mesh levels down to configured command resolution.
      if let trial = chosen, let outcome = trial.outcome,
        TerminalRecoveryOrder.compare(outcome, to: outcome) == .equivalent, !trial.candidate.segments.isEmpty {
        let alphaResolution = c.guidance.alphaSlewRate*c.guidance.controlInterval
        let bankResolution = c.guidance.bankSlewRate*c.guidance.controlInterval
        let initial = TAEMCommandRefinementSteps(alphaRadians: max(alphaResolution, ceiling-c.vehicle.minimumAngleOfAttack),
          bankRadians: max(bankResolution, c.vehicle.maximumBank), durationSeconds: max(c.guidance.controlInterval, response))
        let levels = max(1, Int(ceil(log2(max(initial.alphaRadians/alphaResolution,
          initial.bankRadians/bankResolution, initial.durationSeconds/c.guidance.controlInterval)))))
        let refined = try search.refine(trial.candidate, from: seed, initialSteps: initial,
          resolution: .init(alphaRadians: alphaResolution, bankRadians: bankResolution,
            durationSeconds: c.guidance.controlInterval),
          maximumEvaluations: 1+6*trial.candidate.segments.count*levels,
          cancelled: { Task.isCancelled }) { _, prefix in
            continuation.plan(from: prefix.seed.state, allowDirect: true, preview: true,
              controlSnapshot: prefix.seed.controls, previousTarget: prefix.seed.limiter.previous,
              targetLimiter: prefix.seed.limiter, nativeSeed: prefix.seed).plan
          }
        allTrials += refined.trials
        if let index = refined.incumbentTrial, let challenger = refined.trials[index].outcome,
          TerminalRecoveryOrder.compare(challenger, to: outcome) == .challengerPreferred { chosen = refined.trials[index] }
      }
      let complete = chosen?.outcome.map { TerminalRecoveryOrder.hasCompleteAccounting($0) } ?? false
      return .init(id: id, time: state.time, candidate: complete ? chosen?.candidate : nil,
        outcome: chosen?.outcome, continuation: complete ? chosen?.continuation : nil, reference: complete ? chosen?.propagatedReference() : nil,
        trials: allTrials.map { .init(candidateID: $0.candidate.id, topology: $0.candidate.topology,
          prefixEnd: $0.prefix.end, evaluationStatus: $0.outcome?.evaluation?.status,
          evaluationEnd: $0.outcome?.evaluation?.end, relation: $0.relation) }, searchEnd: result.end)
    } catch {
      return TAEMRecoveryProposal(id: id, time: state.time, candidate: nil, outcome: nil, continuation: nil, reference: nil,
        trials: [], searchEnd: Task.isCancelled ? .cancelled : .familyExhausted, failure: String(describing: error))
    }
  }
}
