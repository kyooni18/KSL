import Foundation
import KSLCore

/// f1eebdc calibration_observation.c, calibration.c and trajectory_calibration.c.
/// Same-flight force reconstruction is advisory; direct environment readings
/// can correct forecasts. Neither path rewrites certified aerodynamics/speeds.
public final class FlightCalibrationEstimator {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let prior: StockLiftingSurfacePrior
  private let frame: RunwayFrame
  private var previousTime: Double?, previousCourse: Double?
  private var forecast: [TrajectorySample] = []
  public private(set) var diagnostics = CalibrationDiagnostics()
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine) throws {
    self.configuration = configuration; self.atmosphere = atmosphere
    prior = try .init(); frame = .init(runway: configuration.runway, planet: configuration.planet)
    diagnostics.regimes = [0.35, 1.05, 2.6, 6].map {
      .init(machAnchor: $0, ballisticCoefficient: configuration.entrySettings.estimatedBallisticCoefficient,
        liftToDrag: configuration.entrySettings.estimatedLiftToDrag)
    }
    diagnostics.autoApplyRequested = (configuration.calibration ?? .init()).autoApply
  }
  public func invalidateDynamics() {
    previousTime = nil; previousCourse = nil; forecast = []
    diagnostics = .init()
    diagnostics.regimes = [0.35, 1.05, 2.6, 6].map {
      .init(machAnchor: $0, ballisticCoefficient: configuration.entrySettings.estimatedBallisticCoefficient,
        liftToDrag: configuration.entrySettings.estimatedLiftToDrag)
    }
    diagnostics.autoApplyRequested = (configuration.calibration ?? .init()).autoApply
  }
  public static func conservativeStallFraction(_ state: VehicleState, configuration c: MissionConfiguration) -> Double {
    let a = state.aerodynamics, safeSpeed = c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed
    let maximumAlpha = c.vehicle.maximumAngleOfAttack
    guard safeSpeed > 1, maximumAlpha > 0 else { return 1 }
    let speedRisk = clamp((1.15*safeSpeed-a.airspeed)/max(10, 0.35*safeSpeed), 0, 1)
    let protective = clamp(c.entrySettings.nominalAlpha, 0, maximumAlpha)
    let thinAir = clamp((700-max(0, a.dynamicPressure))/350, 0, 1)
    let onset = max(0.75*maximumAlpha, protective+thinAir*0.35*max(0, maximumAlpha-protective))
    let incidenceRisk = clamp((abs(a.angleOfAttack)-onset)/max(1e-6, maximumAlpha-onset), 0, 1)
    return max(speedRisk, incidenceRisk)
  }
  public func setForecast(_ prediction: TrajectoryPrediction?) {
    guard let samples = prediction?.samples, samples.count >= 2,
      samples.first?.time != diagnostics.forecastEpoch else { return }
    forecast = samples; diagnostics.forecastEpoch = samples.first?.time
  }
  public func observe(_ state: VehicleState) {
    if let previousTime, state.time == previousTime { return }
    if let previousTime, state.time < previousTime {
      self.previousTime = nil; previousCourse = nil; forecast = []; diagnostics.forecastEpoch = nil
    }
    let dt = previousTime.map { max(0, state.time-$0) } ?? 0
    let c = configuration, settings = c.calibration ?? .init(), a = state.aerodynamics
    var quality = CalibrationObservationQuality()
    let airborne = state.contact == .airborne
    let simulated = state.controlAuthority?.evidence == .provisional
    let forceAvailable = a.measuredForces || simulated
    quality.forceEvidence = a.measuredForces ? .measured : (simulated ? .provisional : .unavailable)
    quality.stallFraction = Self.conservativeStallFraction(state, configuration: c)
    let forcesValid = forceAvailable && a.force.isFinite && a.lift.isFinite && a.drag.isFinite
      && a.dynamicPressure.isFinite && a.dynamicPressure >= 0 && state.mass.isFinite && state.mass > 0
    let forceObservable = forcesValid && a.airspeed > 0 && a.drag > 0 && hypot(a.lift, a.drag) > 0
    if !airborne { quality.rejectionReasons.append("not-airborne") }
    if !forcesValid { quality.rejectionReasons.append("force-unavailable-or-invalid") }
    if !forceObservable { quality.rejectionReasons.append("force-response-unobservable") }
    if state.systems.thrust > 0 { quality.rejectionReasons.append("thrust-contamination") }
    if state.systems.gearDown || state.systems.brakes || state.systems.airbrakes {
      quality.rejectionReasons.append("not-clean-configuration")
    }
    if let input = state.appliedControls, max(abs(input.pitch), abs(input.roll), abs(input.yaw)) >= 1 {
      quality.rejectionReasons.append("actuator-at-domain-boundary")
    }
    if a.dynamicPressure < settings.minimumDynamicPressure { quality.rejectionReasons.append("q-below-sample-domain") }
    if a.dynamicPressure > min(settings.maximumSampleDynamicPressure, c.vehicle.maximumDynamicPressure) {
      quality.rejectionReasons.append("q-above-sample-domain")
    }
    if hypot(a.angleOfAttack, a.sideslip) > c.vehicle.maximumAngleOfAttack {
      quality.rejectionReasons.append("incidence-outside-envelope")
    }
    if abs(a.sideslip) > settings.maximumSideslip { quality.rejectionReasons.append("sideslip-outside-sample-domain") }
    if a.lift/(state.mass*9.80665) > c.vehicle.maximumLoadFactor { quality.rejectionReasons.append("normal-load-outside-envelope") }
    if quality.stallFraction > settings.abortStallFraction { quality.rejectionReasons.append("conservative-stall-risk") }
    if dt > 1 { quality.rejectionReasons.append("clock-gap") }
    if forceObservable && dt <= 1 {
      quality.informationIncrement = dt*hypot(a.lift, a.drag)/(state.mass*a.airspeed)
      quality.informationBlend = -expm1(-quality.informationIncrement)
    }
    quality.passiveEligible = settings.enablePassive && quality.rejectionReasons.isEmpty
    quality.trajectoryEligible = settings.enableTrajectory && quality.rejectionReasons.isEmpty
    diagnostics.observation = quality
    let anchors = diagnostics.regimes.map(\.machAnchor)
    diagnostics.currentRegime = (0..<3).first { max(0, a.mach) < (anchors[$0]+anchors[$0+1])/2 } ?? 3
    if quality.passiveEligible {
      let factor = prior.factors(mach: a.mach, incidence: hypot(a.angleOfAttack, a.sideslip),
        referenceAlpha: c.entrySettings.nominalAlpha)
      var regime = diagnostics.regimes[diagnostics.currentRegime], updated = false
      let beta = a.dynamicPressure*state.mass/a.drag*factor.drag
      if beta.isFinite && beta > 0 {
        regime.ballisticCoefficient = positiveBlend(regime.ballisticCoefficient, beta, quality.informationBlend)
        updated = true
      }
      if factor.lift != 0 {
        let ld = a.lift/a.drag*factor.drag/abs(factor.lift)
        if ld.isFinite && ld > 0 {
          regime.liftToDrag = positiveBlend(regime.liftToDrag, ld, quality.informationBlend); updated = true
        }
      }
      if updated {
        diagnostics.acceptedPassiveSamples += 1
        regime.confidence = 1-(1-regime.confidence)*exp(-quality.informationIncrement)
        diagnostics.regimes[diagnostics.currentRegime] = regime
      } else { diagnostics.rejectedPassiveSamples += 1 }
    } else if settings.enablePassive && airborne { diagnostics.rejectedPassiveSamples += 1 }
    if settings.enableTrajectory && forcesValid {
      let altitude = state.position.length-c.planet.radius
      let base = atmosphere.sample(position: state.position, time: state.time)
      if altitude >= 0 && altitude < c.planet.atmosphereTop {
        let density = state.atmosphere.density/base.density
        let sound = state.atmosphere.speedOfSound/base.speedOfSound
        if density > 0 && density.isFinite { diagnostics.environment.densityScale = density }
        if sound > 0 && sound.isFinite { diagnostics.environment.soundSpeedScale = sound }
        diagnostics.environment.observedAt = state.time
        diagnostics.environment.evidence = state.atmosphere.evidence ?? quality.forceEvidence
      }
    }
    if quality.trajectoryEligible {
      diagnostics.acceptedTrajectorySamples += 1
      diagnostics.trajectoryConfidence = 1-(1-diagnostics.trajectoryConfidence)*exp(-quality.informationIncrement)
      updateResiduals(state, blend: quality.informationBlend)
      if let course = previousCourse, dt > 0 && dt <= 1 {
        let expected = a.lift/state.mass*sin(a.bank)/max(Double.leastNormalMagnitude, state.horizontalSpeed)
        if expected != 0 {
          let sample = wrapAngle(state.course-course)/dt/expected
          if sample.isFinite && sample > 0 {
            let blend = -expm1(-quality.informationIncrement*pow(sin(a.bank), 2))
            diagnostics.bankEffectiveness = positiveBlend(diagnostics.bankEffectiveness, sample, blend)
          }
        }
      }
    }
    previousTime = state.time; previousCourse = state.course
  }
  private func positiveBlend(_ current: Double, _ sample: Double, _ blend: Double) -> Double {
    guard current > 0, sample > 0, sample.isFinite else { return current }
    return exp(log(current)+clamp(blend, 0, 1)*(log(sample)-log(current)))
  }
  private func updateResiduals(_ state: VehicleState, blend: Double) {
    guard forecast.count >= 2, state.time >= forecast[0].time, state.time <= forecast.last!.time else { return }
    var low = 0, high = forecast.count-1
    while high-low > 1 {
      let mid = (low+high)/2
      if forecast[mid].time >= state.time { high = mid } else { low = mid }
    }
    let a = forecast[low], b = forecast[high], duration = b.time-a.time
    guard duration > 0 else { return }
    let f = clamp((state.time-a.time)/duration, 0, 1)
    let ga = GeodeticPosition(position: a.position, radius: configuration.planet.radius)
    let gb = GeodeticPosition(position: b.position, radius: configuration.planet.radius)
    let predicted = GeodeticPosition(latitude: ga.latitude+f*(gb.latitude-ga.latitude),
      longitude: wrapAngle(ga.longitude+f*wrapAngle(gb.longitude-ga.longitude)),
      altitude: ga.altitude+f*(gb.altitude-ga.altitude)).cartesian(radius: configuration.planet.radius)
    let velocity = a.velocity+(b.velocity-a.velocity)*f
    let speed = (velocity-atmosphere.sample(position: predicted, time: state.time).wind).length
    let altitudeError = state.position.length-predicted.length
    let rangeError = frame.distance(from: state.position, to: frame.origin)-frame.distance(from: predicted, to: frame.origin)
    diagnostics.altitudeResidual += blend*(altitudeError-diagnostics.altitudeResidual)
    diagnostics.speedResidual += blend*(state.aerodynamics.airspeed-speed-diagnostics.speedResidual)
    diagnostics.rangeResidual += blend*(rangeError-diagnostics.rangeResidual)
  }
}
