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
  private let objectivePlanner: EntryCaptureObjectivePlanner
  private let frame: RunwayFrame
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    objectivePlanner = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
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
    var memory = memory
    if memory.objective == nil { memory.objective = objectivePlanner.objective(from: state, crossAim: memory.crossAim) }
    let plantAero = liftScale == 1 && dragScale == 1 ? aerodynamics
      : aerodynamics.scaled(lift: liftScale, drag: dragScale)
    let predictor = TrajectoryPropagationEngine(configuration: c, atmosphere: atmosphere, aerodynamics: plantAero)
    // Guidance keeps the nominal model: dispersions live in the plant only.
    let law = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics, memory: memory)
    let evaluator = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    func evaluateCapture(_ state: VehicleState) -> TAEMCaptureAssessment {
      let admission = MM305Admission.physicalAdmission(state, configuration: c)
      let belowCaptureBand = state.position.length-c.planet.radius < c.entrySettings.terminalMinimumAltitude
      if !admission.violations.isEmpty || belowCaptureBand {
        var assessment = TAEMCaptureAssessment()
        let p = frame.coordinates(of: state.position)
        let g = c.planet.gravitationalParameter / pow(state.position.length, 2)
        let speed = state.aerodynamics.airspeed
        assessment.time = state.time; assessment.altitude = p.height
        assessment.mach = state.aerodynamics.mach
        assessment.rangeToRunway = frame.distance(from: state.position, to: frame.origin)
        assessment.headingError = wrapAngle(frame.bearing(from: state.position, to: frame.origin) - state.course)
        assessment.envelope.requiredHeadingChange = abs(assessment.headingError)
        assessment.envelope.availableTurnRate = g * tan(c.vehicle.maximumBank) / max(1, speed)
        assessment.envelope.energyHeight = p.height + speed * speed / (2 * g)
        assessment.violations = admission.violations
        if belowCaptureBand { assessment.deficit = .insufficientAltitude }
        return assessment
      }
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
    var searched = false, nextCapture = state.time
    let first = predictor.predict(from: state, duration: c.guidance.maximumPredictionDuration,
      step: c.guidance.predictionStep, recordEvery: 10, terminalSpeed: 0,
      controlSnapshot: controls, previousTarget: previousTarget,
      feedback: { law.memory.pitchResponseSeconds = $0.pitchResponseSeconds },
      terminalCondition: { s in
        let altitude = s.position.length-c.planet.radius
        if !searched {
          guard altitude <= c.entrySettings.terminalMaximumAltitude,
            s.aerodynamics.mach <= c.entrySettings.terminalTargetMach+c.entrySettings.terminalMachHalfWidth,
            s.verticalSpeed < 0 else { return false }
        }
        guard s.time >= nextCapture else { return false }
        searched = true; nextCapture = s.time+c.guidance.terminalReplanInterval
        capture = evaluateCapture(s)
        // Verdicts do not end the survey. Keep the same FCS, limiter and
        // physical integrator until a HAC exists or the high capture band ends.
        return capture.feasible || altitude < c.entrySettings.terminalMinimumAltitude
      }) { s in
        let target = law.update(s, rangeMultiplier: rangeMultiplier)
        return .init(alpha: target.angleOfAttack, bank: target.bank)
      }
    let samples = first.samples, violations = first.violations, end = first.terminalState
    if capture.feasible && certify {
      capture = evaluator.evaluate(end, certify: true, controlSnapshot: controls, previousTarget: previousTarget)
    }
    let p = frame.coordinates(of: end.position)
    if !searched {
      capture = .init(); capture.time = end.time; capture.altitude = p.height
      capture.mach = end.aerodynamics.mach
      capture.rangeToRunway = frame.distance(from: end.position, to: frame.origin)
      capture.violations = violations
      capture.deficit = violations.isEmpty ? .terminalRegionNotReached : .envelopeViolation
    }
    if searched, !violations.isEmpty {
      capture.feasible = false; capture.violations += violations; capture.deficit = .envelopeViolation
    }
    var maxQ = 0.0, maxLoad = 0.0
    let track: [[Double]] = samples.map { s in
      maxQ = max(maxQ, s.dynamicPressure); maxLoad = max(maxLoad, s.loadFactor)
      let q = frame.coordinates(of: s.position)
      return [s.time, q.along, q.cross, q.height, s.velocity.length, s.angleOfAttack, s.bank]
    }
    var result = EntryCaptureForecast(createdAt: state.time, rangeMultiplier: rangeMultiplier, liftScale: liftScale,
      dragScale: dragScale, reachedTerminalRegion: searched,
      violations: violations, terminalTime: end.time, terminalAltitude: p.height,
      terminalMach: end.aerodynamics.mach, terminalAlong: p.along, terminalCross: p.cross,
      maximumDynamicPressure: maxQ, maximumLoad: maxLoad, capture: capture, track: track)
    result.crossAim = memory.crossAim
    result.objective = memory.objective ?? EntryCaptureObjective.seed(configuration: c,
      atmosphere: atmosphere, frame: frame, crossAim: memory.crossAim, time: state.time)
    let envelope = capture.envelope
    // A completed HAC family can bracket a nonempty energy interval even
    // when the flown state is just outside it. Center that interval, then
    // re-fly the objective; waiting for an already feasible trajectory would
    // leave an infeasible seed objective permanently in command.
    if end.position.length - c.planet.radius >= c.entrySettings.terminalMinimumAltitude,
      envelope.excessEnergyMargin > -Double.greatestFiniteMagnitude,
      envelope.deficitEnergyMargin > -Double.greatestFiniteMagnitude,
      envelope.excessEnergyMargin + envelope.deficitEnergyMargin > 0 {
      let gravity = c.planet.gravitationalParameter / pow(end.position.length, 2)
      let energy = capture.envelope.energyHeight
        + (capture.envelope.excessEnergyMargin - capture.envelope.deficitEnergyMargin) / 2
      // Keep capture speed; center the altitude/energy target between the two
      // priced HAC energy boundaries, retaining positive room on both sides.
      result.suggestedObjective = .init(along: p.along, cross: p.cross,
        altitude: c.runway.elevation + energy - pow(end.aerodynamics.airspeed, 2) / (2 * gravity),
        speed: end.aerodynamics.airspeed, energyHeight: energy, flightPath: end.flightPathAngle)
    }
    return result
  }
}
