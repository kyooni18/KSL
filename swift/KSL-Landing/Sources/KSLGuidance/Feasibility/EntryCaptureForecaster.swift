import Foundation
import KSLControl
import KSLCore
import KSLPhysics

/// One closed-loop MM304 branch flown to the TAEM interface, then judged by
/// the TAEM capture evaluator. The predicted ground track is retained so the
/// actual trajectory can later be compared with the prediction.
public struct EntryCaptureForecast: Codable, Sendable {
  public var suggestedObjective: EntryCaptureObjective?
  public var objective: EntryCaptureObjective?
  public let createdAt: Double
  public let rangeMultiplier: Double
  public var crossAim = 0.0
  public let liftScale: Double
  public let dragScale: Double
  public let reachedTerminalRegion: Bool
  public let violations: [ConstraintViolation]
  public let terminalTime: Double
  public let terminalAltitude: Double
  public let terminalMach: Double
  public let terminalAlong: Double
  public let terminalCross: Double
  public let maximumDynamicPressure: Double
  public let maximumLoad: Double
  public var capture: TAEMCaptureAssessment
  /// Sparse predicted path (time, along, cross, height, speed, alpha, bank).
  public var track: [[Double]]
  public var feasible: Bool { reachedTerminalRegion && violations.isEmpty && capture.feasible }
}

/// Forecasts whether the current MM304 law, flown in the configured plant,
/// ends inside the TAEM reachable envelope. Shared by MM304's receding-horizon
/// objective and by landing-aware deorbit window selection.
public final class EntryCaptureForecaster {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  private let frame: RunwayFrame
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }

  /// MM304's terminal target speed; capture is then searched beyond it.
  public func terminalSpeed(at time: Double) -> Double {
    let e = configuration.entrySettings
    let point = frame.position(along: e.handoffAlong, cross: e.handoffCross, height: 0)
    return e.terminalTargetMach * atmosphere.sample(
      position: point.normalized * (configuration.planet.radius + e.terminalTargetAltitude), time: time).speedOfSound
  }

  /// TAEM turning reach at the MM304 terminal speed: the coordinated turn
  /// radius at the acquisition bank. Lateral aims and the reversal corridor
  /// are expressed in this physical unit.
  public func turningReach(at time: Double) -> Double {
    let v = terminalSpeed(at: time), c = configuration
    let g = c.planet.gravitationalParameter / pow(c.planet.radius + c.entrySettings.terminalTargetAltitude, 2)
    return v * v / (g * tan(max(1e-3, EnergyManagementEngine.acquisitionBank(c))))
  }

  /// Candidate lateral aims: on the handoff point and one turning reach to
  /// either side of the runway axis.
  public func lateralAims(at time: Double) -> [Double] {
    let w = turningReach(at: time); return [0, -w, w]
  }

  /// Forecast from a fresh MM304 executive (deorbit planning).
  public func forecast(from state: VehicleState, rangeMultiplier: Double = 1, crossAim: Double = 0,
                       liftScale: Double = 1, dragScale: Double = 1,
                       certify: Bool = false) -> EntryCaptureForecast {
    var memory = EntryLawMemory(); memory.crossAim = crossAim
    return forecast(from: state, memory: memory, rangeMultiplier: rangeMultiplier, liftScale: liftScale,
      dragScale: dragScale, certify: certify, controls: nil, previousTarget: nil)
  }

  func forecast(from state: VehicleState, memory: EntryLawMemory, rangeMultiplier: Double,
                liftScale: Double = 1, dragScale: Double = 1, certify: Bool = false,
                controls: FlightControlSnapshot?, previousTarget: GuidanceTarget?) -> EntryCaptureForecast {
    let c = configuration
    let plantAero = liftScale == 1 && dragScale == 1 ? aerodynamics
      : aerodynamics.scaled(lift: liftScale, drag: dragScale)
    let predictor = TrajectoryPropagationEngine(configuration: c, atmosphere: atmosphere, aerodynamics: plantAero)
    // Guidance keeps the nominal model: dispersions live in the plant only.
    let law = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics, memory: memory)
    let speed = terminalSpeed(at: state.time)
      * (c.entrySettings.terminalTargetMach + c.entrySettings.terminalMachHalfWidth) / c.entrySettings.terminalTargetMach
    let first = predictor.predict(from: state, duration: c.guidance.maximumPredictionDuration,
      step: c.guidance.predictionStep, recordEvery: 10, terminalSpeed: speed,
      minimumDuration: state.aerodynamics.airspeed < speed ? 4 * c.guidance.entryReplanInterval : 0,
      controlSnapshot: controls, previousTarget: previousTarget,
      feedback: { law.memory.pitchResponseSeconds = $0.pitchResponseSeconds }) { s in
        let target = law.update(s, rangeMultiplier: rangeMultiplier)
        return .init(alpha: target.angleOfAttack, bank: target.bank)
      }
    var samples = first.samples, violations = first.violations, end = first.terminalState
    let evaluator = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    func evaluateCapture(_ state: VehicleState) -> TAEMCaptureAssessment {
      var assessment = evaluator.evaluate(state)
      if let plan = assessment.plan {
        let admission = MM305Admission.evaluate(state, plan: plan, configuration: c, requireQualification: false)
        if !admission.accepted {
          assessment.feasible = false; assessment.deficit = .terminalRegionNotReached
          assessment.violations += admission.violations
        }
      }
      return assessment
    }
    var capture = TAEMCaptureAssessment()
    if first.reachedTerminalRegion && violations.isEmpty {
      // The capture region is wherever an admissible HAC first exists, not a
      // fixed Mach: keep flying the MM304 law and re-ask at MM305's cadence.
      capture = evaluateCapture(end)
      while !Task.isCancelled, !capture.feasible, capture.deficit != .insufficientEnergy, violations.isEmpty,
        end.time - state.time < c.guidance.maximumPredictionDuration,
        frame.coordinates(of: end.position).height > c.vehicle.gearHeight {
        let next = predictor.predict(from: end, duration: c.guidance.terminalReplanInterval,
          step: c.guidance.predictionStep, recordEvery: 10, terminalSpeed: 0,
          feedback: { law.memory.pitchResponseSeconds = $0.pitchResponseSeconds }) { s in
            let target = law.update(s, rangeMultiplier: rangeMultiplier)
            return .init(alpha: target.angleOfAttack, bank: target.bank)
          }
        if next.terminalState.time <= end.time { break }
        samples += next.samples.dropFirst(); violations += next.violations; end = next.terminalState
        if !violations.isEmpty { break }
        capture = evaluateCapture(end)
      }
      if capture.feasible && certify {
        capture = evaluator.evaluate(end, certify: true, controlSnapshot: controls, previousTarget: previousTarget)
      }
    }
    let p = frame.coordinates(of: end.position)
    if !(first.reachedTerminalRegion && violations.isEmpty) {
      capture = .init(); capture.time = end.time; capture.altitude = p.height
      capture.mach = end.aerodynamics.mach
      capture.rangeToRunway = frame.distance(from: end.position, to: frame.origin)
      capture.violations = violations
      capture.deficit = violations.isEmpty ? .terminalRegionNotReached : .envelopeViolation
    }
    var maxQ = 0.0, maxLoad = 0.0
    let track: [[Double]] = samples.map { s in
      maxQ = max(maxQ, s.dynamicPressure); maxLoad = max(maxLoad, s.loadFactor)
      let q = frame.coordinates(of: s.position)
      return [s.time, q.along, q.cross, q.height, s.velocity.length, s.angleOfAttack, s.bank]
    }
    var result = EntryCaptureForecast(createdAt: state.time, rangeMultiplier: rangeMultiplier, liftScale: liftScale,
      dragScale: dragScale, reachedTerminalRegion: first.reachedTerminalRegion,
      violations: violations, terminalTime: end.time, terminalAltitude: p.height,
      terminalMach: end.aerodynamics.mach, terminalAlong: p.along, terminalCross: p.cross,
      maximumDynamicPressure: maxQ, maximumLoad: maxLoad, capture: capture, track: track)
    result.crossAim = memory.crossAim
    result.objective = memory.objective ?? EntryCaptureObjective.seed(configuration: c,
      atmosphere: atmosphere, frame: frame, crossAim: memory.crossAim, time: state.time)
    if capture.feasible, capture.envelope.energyMargin > 0 {
      let gravity = c.planet.gravitationalParameter / pow(end.position.length, 2)
      let energy = capture.envelope.energyHeight
        + (capture.envelope.excessEnergyMargin - capture.envelope.deficitEnergyMargin) / 2
      // Keep capture speed; center the altitude/energy target between the two
      // priced HAC energy boundaries, retaining positive room on both sides.
      result.suggestedObjective = .init(along: p.along, cross: p.cross,
        altitude: c.runway.elevation + energy - pow(end.aerodynamics.airspeed, 2) / (2 * gravity),
        speed: end.aerodynamics.airspeed, energyHeight: energy)
    }
    return result
  }
}
