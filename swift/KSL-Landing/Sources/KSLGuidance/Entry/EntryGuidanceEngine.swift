import Foundation
import KSLCore
import KSLPhysics

/// Receding-horizon search over a continuous lift-allocation parameter.
/// Lateral feedback is evaluated inside every candidate rollout, not frozen at t0.
public final class EntryGuidanceEngine {
  private let configuration: MissionConfiguration
  private let predictor: TrajectoryPropagationEngine
  private let aerodynamics: AerodynamicsEngine
  private let rotation: RotationalDynamicsEngine
  private let frame: RunwayFrame
  private let terminalPlanner: HACPlanningEngine
  private var lastSolve = -Double.infinity
  private var allocation = 0.25
  private var bankSign = 1.0
  private var lastReversal = -Double.infinity
  public private(set) var reversalCount = 0
  public private(set) var bestPrediction: TrajectoryPrediction?
  public private(set) var candidateCount = 0
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    self.aerodynamics = aerodynamics
    terminalPlanner = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    predictor = .init(
      configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    rotation = .init(configuration: configuration.vehicle)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  private func steering(_ s: VehicleState, allocation: Double, preferredSign: Double)
    -> PredictionAttitude
  {
    let v = configuration.vehicle
    let speed = s.aerodynamics.airspeed
    let target = frame.position(
      along: -configuration.guidance.finalDistance - 12_000, cross: 0, height: 0)
    let bearing = frame.bearing(from: s.position, to: target)
    let error = wrapAngle(bearing - s.course)
    let alphaGlide = aerodynamics.bestGlideAlpha(
      mach: s.aerodynamics.mach, maximumAlpha: v.maximumAngleOfAttack)
    let nominal = radians(clamp(15 + (speed - 650) / 90, 15, 35))
    var alpha = allocation < 0 ? nominal + (-allocation) * (alphaGlide - nominal) : nominal
    alpha = min(alpha, rotation.maximumTrimAlpha(dynamicPressure: s.aerodynamics.dynamicPressure))
    let deadband = radians(clamp(3 + speed / 180, 4, 16))
    let sign = abs(error) > deadband ? (error < 0 ? -1.0 : 1.0) : preferredSign
    let headingBank = clamp(abs(error) * 0.65, 0, v.maximumBank)
    let energyBank = max(0, allocation) * v.maximumBank
    let bank = sign * max(headingBank, energyBank)
    return .init(alpha: alpha, bank: bank)
  }
  /// Bank reversals only make sense once the airframe can actually realize the commanded
  /// attitude: below that dynamic pressure the plant cannot trim the nominal alpha, a bank
  /// swing produces no lateral lift, and reversing only flips the attitude for nothing.
  private func hasControlAuthority(_ s: VehicleState) -> Bool {
    let nominal = radians(clamp(15 + (s.aerodynamics.airspeed - 650) / 90, 15, 35))
    return rotation.maximumTrimAlpha(dynamicPressure: s.aerodynamics.dynamicPressure) >= nominal
  }
  public func update(_ state: VehicleState) -> GuidanceTarget {
    if state.time - lastSolve >= configuration.guidance.entryReplanInterval {
      solve(state)
      lastSolve = state.time
    }
    let proposed = steering(state, allocation: allocation, preferredSign: bankSign)
    let newSign = proposed.bank < 0 ? -1.0 : 1.0
    if newSign != bankSign && state.time - lastReversal > 12 && abs(proposed.bank) > radians(5)
      && hasControlAuthority(state)
    {
      bankSign = newSign
      lastReversal = state.time
      reversalCount += 1
    }
    let bank = abs(proposed.bank) * bankSign
    return .init(
      angleOfAttack: proposed.alpha, bank: bank, desiredSpeed: 650,
      desiredHeight: 25_000, desiredFlightPath: state.flightPathAngle)
  }
  private func solve(_ state: VehicleState) {
    let target = frame.position(
      along: -configuration.guidance.finalDistance - 12_000, cross: 0, height: 0)
    var bestScore = Double.infinity
    var best = allocation
    var bestSign = bankSign
    var prediction: TrajectoryPrediction?
    let candidates = Set([
      -1.0, -0.5, 0, 0.2, 0.4, 0.6, 0.8, 1.0, clamp(allocation - 0.1, -1, 1),
      clamp(allocation + 0.1, -1, 1),
    ]).sorted()
    let signs =
      state.time - lastReversal > 12 && hasControlAuthority(state) ? [bankSign, -bankSign] : [bankSign]
    candidateCount = candidates.count * signs.count
    var ranked:
      [(score: Double, allocation: Double, sign: Double, prediction: TrajectoryPrediction)] = []
    for initialSign in signs {
      for sigma in candidates {
        var forecastSign = initialSign
        var forecastReversal = initialSign == bankSign ? lastReversal : state.time
        let p = predictor.predict(
          from: state, duration: configuration.guidance.maximumPredictionDuration,
          step: configuration.guidance.predictionStep, recordEvery: 10,
          // A rejected terminal handoff must retain control-dependent forecasts.
          // Otherwise every candidate returns the current state below 650 m/s.
          minimumDuration: state.aerodynamics.airspeed < 650
            ? 4 * configuration.guidance.entryReplanInterval : 0
        ) { [self] s in
          let proposed = steering(s, allocation: sigma, preferredSign: forecastSign)
          let sign = proposed.bank < 0 ? -1.0 : 1.0
          if sign != forecastSign && s.time - forecastReversal > 12
            && abs(proposed.bank) > radians(5) && hasControlAuthority(s)
          {
            forecastSign = sign
            forecastReversal = s.time
          }
          return .init(alpha: proposed.alpha, bank: abs(proposed.bank) * forecastSign)
        }
        let end = p.terminalState
        let pos = frame.coordinates(of: end.position)
        let range = frame.distance(from: end.position, to: target)
        let bearing = frame.bearing(from: end.position, to: target)
        let courseError = abs(wrapAngle(bearing - end.course))
        // Entry retains a set of possible HAC inlets. No fixed handoff point/heading gate.
        let alpha = aerodynamics.bestGlideAlpha(mach: end.aerodynamics.mach)
        let c = aerodynamics.coefficients(mach: end.aerodynamics.mach, alpha: alpha)
        let ld = max(0.3, c.lift / max(c.drag, 1e-8))
        let energyHeight = max(0, pos.height) + end.velocity.squaredLength / (2 * 9.80665)
        let turnDistance =
          end.horizontalSpeed * end.horizontalSpeed / (9.80665 * tan(radians(45)))
          * max(0, courseError - .pi / 6)
        let required = (range + turnDistance + 12_000) / ld + 2200
        let deficit = max(0, required - energyHeight)
        let excess = max(0, energyHeight - required - 25_000)
        var score = pow(range / 50_000, 2) + pow(deficit / 8000, 2) * 8 + pow(excess / 20_000, 2)
        score +=
          pow(courseError / .pi, 2) + pow(pos.cross / 80_000, 2) + 0.08 * pow(sigma - allocation, 2)
        if pos.along > 0 { score += pow(pos.along / 15_000, 2) + 5 }
        if !p.reachedTerminalRegion { score += 20 }
        if !p.violations.isEmpty { score += 1000 + Double(p.violations.count) * 100 }
        score += initialSign == bankSign ? 0 : 0.05
        ranked.append((score, sigma, initialSign, p))
        if score < bestScore {
          bestScore = score
          best = sigma
          bestSign = initialSign
          prediction = p
        }
      }
    }
    // The complete terminal trajectory, including flare and rollout margin,
    // decides among the leading entry forecasts. The geometric entry score
    // is a fallback when no complete terminal route exists yet.
    var feasibleScore = Double.infinity
    for candidate in ranked.sorted(by: { $0.score < $1.score }).prefix(3)
    where candidate.prediction.reachedTerminalRegion && candidate.prediction.violations.isEmpty {
      let terminal = terminalPlanner.plan(
        from: candidate.prediction.terminalState,
        allowDirect: configuration.guidance.allowDirectTerminal, preview: true)
      if let plan = terminal.plan {
        let score = candidate.score + plan.assessment.score / 1000
        if score < feasibleScore {
          feasibleScore = score
          best = candidate.allocation
          bestSign = candidate.sign
          prediction = candidate.prediction
        }
      }
    }
    if bestSign != bankSign && state.time - lastReversal > 12 && hasControlAuthority(state) {
      bankSign = bestSign
      lastReversal = state.time
      reversalCount += 1
    }
    allocation = best
    bestPrediction = prediction
  }
}
