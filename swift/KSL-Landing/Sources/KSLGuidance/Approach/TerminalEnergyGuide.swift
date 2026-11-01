import Foundation
import KSLCore
import KSLPhysics

public struct ApproachPlanningRequest: Sendable {
  let state: VehicleState
  let flare: FlareReference
  let maximumAlpha: Double
  let lift: Double
  let drag: Double
}
public struct ApproachPlanningProposal: Codable, Sendable {
  public let time: Double
  public let touchdownSpeed: Double?
  public let touchdownAlong: Double?
}

struct TerminalEnergyReference {
  let flightPath: Double
  let desiredSpeed: Double
  let predictedTouchdownSpeed: Double?
  let predictedTouchdownAlong: Double?
}

/// C final/sequence.inc outer-glide energy/aim closure. Its point-mass sizing
/// forecast is advisory; TerminalTrajectoryEvaluator supplies the independent
/// actual-controller contact/rollout qualification.
final class TerminalEnergyGuide {
  private let configuration: MissionConfiguration
  private let aero: AerodynamicsEngine
  private let atmosphere: AtmosphericModelEngine
  private let frame: RunwayFrame
  private var aim: Double?, previousTime: Double?, shift = 0.0
  private var latestRequest: ApproachPlanningRequest?
  private var lastRequestTime = -Double.infinity
  private var latestProposal: ApproachPlanningProposal?
  private var lowEnergySince: Double?
  private var recoveredRangeSince: Double?
  private var lowEnergyRecovery = false
  private var nominalEnergySpeed: Double?
  func captureNominalEnergySpeed(_ speed: Double) {
    if speed.isFinite, speed > 0 { nominalEnergySpeed = speed }
  }
  func invalidatePrediction() {
    latestRequest = nil; latestProposal = nil; lastRequestTime = -.infinity
    lowEnergySince = nil; recoveredRangeSince = nil; lowEnergyRecovery = false
    shift = 0
  }
  func request() -> ApproachPlanningRequest? {
    guard let request = latestRequest,
      request.state.time-lastRequestTime >= (configuration.guidance.approachPredictionInterval ?? 1.5) else { return nil }
    lastRequestTime = request.state.time
    return request
  }
  func accept(_ proposal: ApproachPlanningProposal) { latestProposal = proposal }
  func plan(_ request: ApproachPlanningRequest) -> ApproachPlanningProposal {
    let result = predict(request.state, flare: request.flare, maximumAlpha: request.maximumAlpha,
      lift: request.lift, drag: request.drag)
    let along = frame.coordinates(of: request.state.position).along
    return .init(time: request.state.time, touchdownSpeed: result?.speed,
      touchdownAlong: result.map { along+$0.range })
  }
  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
       aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  func update(_ state: VehicleState, flare: FlareReference, maximumAlpha: Double,
              lift: Double, drag: Double, synchronousPlanning: Bool) -> TerminalEnergyReference {
    let c = configuration, p = frame.coordinates(of: state.position), a = state.aerodynamics
    let finalEntrySpeed = nominalEnergySpeed ?? c.guidance.finalSpeed
    let dt = clamp(state.time-(previousTime ?? state.time), 0, 1)
    previousTime = state.time
    let g = c.planet.gravitationalParameter/state.position.squaredLength
    let bestAlpha = aero.bestGlideAlpha(mach: a.mach, maximumAlpha: maximumAlpha,
      dynamicPressure: a.dynamicPressure, gear: true)
    let best = aero.coefficients(mach: a.mach, alpha: bestAlpha,
      dynamicPressure: a.dynamicPressure, gear: true)
    let ld = max(0.1, best.lift*lift/max(1e-6, best.drag*drag))
    let sink = max(0, -state.verticalSpeed), arrest = max(0.01, flare.designAcceleration)
    let pullHeight = max(0, sink*sink-3.5*3.5)/(2*arrest)
    let energyHeight = max(0, a.airspeed*a.airspeed-c.vehicle.landingSpeed*c.vehicle.landingSpeed)/(2*g)
    let arcDistance = state.horizontalSpeed*sink/arrest*0.6
    let floatDistance = 0.8*ld*(energyHeight+12)
    let slope = max(tan(radians(4)), abs(tan(state.flightPathAngle)))
    let desiredAim = clamp(c.guidance.touchdownAlong+150-arcDistance-floatDistance+(pullHeight+12)/slope+shift,
      c.guidance.touchdownAlong-6000, c.guidance.touchdownAlong+150+max(0, shift))
    latestRequest = .init(state: state, flare: flare, maximumAlpha: maximumAlpha, lift: lift, drag: drag)
    if synchronousPlanning, let request = request() { accept(plan(request)) }
    var forecast: (speed: Double, range: Double)?
    if let proposal = latestProposal, state.time >= proposal.time,
      state.time-proposal.time <= 3*(c.guidance.approachPredictionInterval ?? 1.5),
      let speed = proposal.touchdownSpeed, let along = proposal.touchdownAlong {
      forecast = (speed, along-p.along)
    }
    // C's low-energy delivery has a three-second dwell. Apply its recovery
    // speed floor only when the measured-force forecast persistently misses
    // the pavement and the touchdown speed, not merely the nominal aim point.
    if let forecast, p.along+forecast.range < 0, forecast.speed < c.vehicle.landingSpeed {
      if lowEnergySince == nil { lowEnergySince = state.time }
      if state.time-lowEnergySince! >= 3 { lowEnergyRecovery = true }
    } else { lowEnergySince = nil }
    if lowEnergyRecovery, let forecast, p.along+forecast.range >= c.guidance.touchdownAlong {
      if recoveredRangeSince == nil { recoveredRangeSince = state.time }
      if state.time-recoveredRangeSince! >= 3 { lowEnergyRecovery = false; lowEnergySince = nil }
    } else { recoveredRangeSince = nil }
    if aim == nil { aim = clamp(p.along+flare.wheelHeight/slope,
      c.guidance.touchdownAlong-6000, c.guidance.touchdownAlong) }
    aim! += dt/(4+dt)*(desiredAim-aim!)
    if let forecast {
      let error = clamp(c.guidance.touchdownAlong-p.along-forecast.range, -300, 300)
      shift = clamp(shift+dt*error/6, -1500, 1500)
    }
    let floor = 1.5*max(c.vehicle.landingSpeed, flare.oneGravitySpeed)
    let energySpeed = forecast.map {
      sqrt(max(0, a.airspeed*a.airspeed+c.vehicle.landingSpeed*c.vehicle.landingSpeed-$0.speed*$0.speed))
    } ?? floor
    let referenceSpeed: Double
    if lowEnergyRecovery {
      // A persistently short, slow forecast cannot recover by diving toward
      // the nominal flare-energy speed. Acquire the measured polar's best-L/D
      // support speed instead: V_best^2 = V^2 g cos(gamma) / (L_best/m).
      // This uses the existing C glide-speed closure and preserves its lift
      // and incidence limits; it does not certify the degraded approach.
      let support = a.dynamicPressure*aero.referenceArea*best.lift*lift/state.mass
      let bestSpeed = support > 0 ? a.airspeed*sqrt(g*cos(state.flightPathAngle)/support) : energySpeed
      referenceSpeed = max(c.vehicle.landingSpeed, bestSpeed,
        flare.oneGravitySpeed.isFinite ? 1.05*flare.oneGravitySpeed : c.vehicle.landingSpeed)
    } else {
      referenceSpeed = max(finalEntrySpeed, floor.isFinite ? floor : finalEntrySpeed, energySpeed)
    }
    let geometric = clamp(-atan2(flare.wheelHeight, max(120, aim!-p.along)), radians(-25), radians(-4))
    var flightPath = clamp(geometric+radians(clamp(0.8*(a.airspeed-referenceSpeed), -12, 6)), radians(-35), radians(-3))
    if lowEnergyRecovery || (forecast.map({ $0.range < c.guidance.touchdownAlong-p.along }) ?? false) {
      // C range_short is a reversible advisory, unlike the explicitly
      // admitted degraded delivery's terminal_final_low_energy latch.
      let floor = lowEnergyRecovery ? c.vehicle.landingSpeed : max(c.vehicle.landingSpeed, finalEntrySpeed)
      let stretch = clamp((a.airspeed-floor)/(0.15*c.vehicle.landingSpeed), 0, 1)
      flightPath += stretch*(max(flightPath, -atan(1/ld))-flightPath)
    }
    return .init(flightPath: flightPath, desiredSpeed: referenceSpeed,
      predictedTouchdownSpeed: forecast?.speed, predictedTouchdownAlong: forecast.map { p.along+$0.range })
  }
  private func predict(_ initial: VehicleState, flare: FlareReference, maximumAlpha: Double,
                       lift: Double, drag: Double) -> (speed: Double, range: Double)? {
    let c = configuration
    var height = flare.wheelHeight, speed = initial.aerodynamics.airspeed
    var sink = max(0, -initial.verticalSpeed), latched = false, range = 0.0
    let sinGlide = clamp(sink/max(1, speed), -0.95, 0.95), dt = 0.25
    let g = c.planet.gravitationalParameter/initial.position.squaredLength
    let modelInitial = atmosphere.sample(position: initial.position, time: initial.time)
    let densityScale = initial.atmosphere.density/max(modelInitial.density, 1e-12)
    let soundScale = initial.atmosphere.speedOfSound/modelInitial.speedOfSound
    for step in 0..<1200 {
      if Task.isCancelled { return nil }
      if height <= 0 { return speed > 0 ? (speed, range) : nil }
      if speed <= 5 || !speed.isFinite { return nil }
      let arrest = max(0.01, flare.designAcceleration)
      let pull = max(0, sink*sink-3.5*3.5)/(2*arrest)
      if height <= pull+12+sink*1.5 { latched = true }
      if !latched { sink = speed*sinGlide }
      let reference = TerminalFlareController.sinkProfile(height: height,
        arrestAcceleration: arrest, gravity: g)
      let demand = latched ? clamp((sink-reference)/1.5+(sink > 3.5 && height > 12 ? arrest : 0), -0.3*g, 1.5*g) : 0
      let gamma = -asin(clamp(sink/speed, -0.95, 0.95))
      let position = initial.position.normalized*(c.planet.radius+c.runway.elevation+height+c.vehicle.gearHeight)
      let atm = atmosphere.sample(position: position, time: initial.time+Double(step)*dt)
      let q = 0.5*atm.density*densityScale*speed*speed, mach = speed/(atm.speedOfSound*soundScale)
      let solution = aero.liftSolution(acceleration: max(0, demand+g*cos(gamma))/max(0.01, lift),
        mass: initial.mass, dynamicPressure: q, mach: mach, maximumAlpha: maximumAlpha, gear: true)
      let coefficient = aero.coefficients(mach: mach, alpha: solution.alpha, dynamicPressure: q, gear: true)
      let actualPull = q*aero.referenceArea*coefficient.lift*lift/initial.mass-g*cos(gamma)
      let deceleration = q*aero.referenceArea*coefficient.drag*drag/initial.mass
      range += speed*cos(gamma)*dt
      speed += (-deceleration-g*sin(gamma))*dt
      if latched { sink -= min(demand, actualPull)*dt }
      height -= sink*dt
    }
    return nil
  }
}
