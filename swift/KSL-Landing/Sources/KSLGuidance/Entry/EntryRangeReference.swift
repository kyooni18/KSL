import Foundation
import KSLCore
import KSLPhysics

/// Point-mass shooting reference, not a closed-loop reachability certificate.
/// Solve where interface energy is reached instead of fitting instantaneous
/// drag to an assumed altitude-versus-speed path. Each trial uses the actual
/// rotating planet, atmosphere and polar. Alpha spans best L/D through the
/// configured attainable ceiling; the scalar is solved, never scenario-tuned.
struct EntryRangeReference: Sendable {
  var createdAt: Double
  var blend: Double
  var cross: Double
  var headingError: Double
  /// Full arrival error in configured capture-tolerance units; advisory only.
  var captureResidual: Double
  var along: Double
  var altitude: Double
  var speed: Double
  var bracketed: Bool
  var reachedEnergy: Bool
  var advisoryInlet: TAEMAdvisoryInlet? = nil

  var handoffForecast: EntryHandoffForecast {
    .init(createdAt:createdAt,arrivalResidual:captureResidual)
  }

  static func alpha(blend: Double, mach: Double, q: Double, ceiling: Double,
                    aero: AerodynamicsEngine) -> Double {
    let best = aero.bestGlideAlpha(mach: mach, maximumAlpha: ceiling, dynamicPressure: q)
    return best + blend * (ceiling-best)
  }

  /// Only measured/modelled surplus lift is spent on crossrange correction.
  static func bank(state s: VehicleState, objective: EntryCaptureObjective,
                   configuration c: MissionConfiguration, frame: RunwayFrame,
                   lift: Double) -> Double {
    let p = frame.coordinates(of: s.position)
    let target = frame.position(along: objective.along, cross: objective.cross, height: 0)
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let horizontal = s.velocity.projected(onPlaneWithNormal: s.position.normalized)
    let alignment = horizontal.normalized.dot(forward)
    let lateral = EntryLateralController.demand(state: s, configuration: c,
      azimuthError: wrapAngle(frame.bearing(from: s.position, to: target)-s.course),
      crossError: objective.cross-p.cross,
      crossRate: -frame.coordinateVelocity(at:s.position,velocity:s.velocity).y,
      alignment: alignment, range: abs(objective.along-p.along))
    let inertial = c.planet.inertialVelocity(position:s.position,fixedVelocity:s.velocity)
    let support = max(0, (c.planet.gravitationalParameter/s.position.squaredLength
      - inertial.projected(onPlaneWithNormal:s.position.normalized).squaredLength/s.position.length)
      * cos(s.flightPathAngle))
    let authority = NormalForceEnvelope(maximumLift:lift,maximumBank:c.vehicle.maximumBank)
      .lateralAuthority(reserving:support)
    return lift > 0 ? asin(clamp(lateral,-authority,authority)/lift) : 0
  }

  static func solve(from initial: VehicleState, objective: EntryCaptureObjective,
                    configuration c: MissionConfiguration, atmosphere: AtmosphericModelEngine,
                    aero: AerodynamicsEngine, ceiling: Double,
                    pitchResponse: Double, rollResponse: Double) -> Self {
    let frame = RunwayFrame(runway:c.runway,planet:c.planet)
    let translation = TranslationalDynamicsEngine(planet:c.planet,atmosphere:atmosphere,aerodynamics:aero)
    let endpoint = objective.specificEnergy(configuration:c)
    func energy(_ s: VehicleState) -> Double {
      s.velocity.squaredLength/2-c.planet.gravitationalParameter/s.position.length
        - c.planet.angularVelocity.cross(s.position).squaredLength/2
    }
    func trial(_ blend: Double) -> Self {
      var s=initial, alpha=s.aerodynamics.angleOfAttack, bank=s.aerodynamics.bank
      let end=initial.time+c.guidance.maximumPredictionDuration
      var reached=false
      var inlet: TAEMAdvisoryInlet?
      func captureInlet(_ state: VehicleState) {
        guard inlet == nil else { return }
        let lift = state.position.length-c.planet.radius <= c.entrySettings.terminalMaximumAltitude
          ? EntryHandoffTiming.maximumLift(state:state,configuration:c,aerodynamics:aero,ceiling:ceiling) : nil
        if LandingMissionController.terminalHandoffDue(phase:.entry,plan:nil,state:state,
          configuration:c,objective:objective,responseSeconds:max(pitchResponse,rollResponse,c.guidance.pathLookaheadTime),
          maximumLiftAcceleration:lift) {
          inlet = .init(sourceTime:initial.time,state:state,model:"response-limited-point-mass")
        }
      }
      while s.time < end, s.position.length > c.planet.radius+c.runway.elevation {
        s.atmosphere=atmosphere.sample(position:s.position,time:s.time)
        s.attitude = .windAttitude(position:s.position,airVelocity:s.velocity-s.atmosphere.wind,alpha:alpha,bank:bank)
        s.aerodynamics=aero.evaluate(position:s.position,velocity:s.velocity,attitude:s.attitude,
          atmosphere:s.atmosphere,gear:s.systems.gearDown)
        captureInlet(s)
        if energy(s)<=endpoint {reached=true;break}
        if s.aerodynamics.dynamicPressure>c.vehicle.maximumDynamicPressure
          || s.aerodynamics.lift/s.mass>c.vehicle.maximumLoadFactor*9.80665 {break}
        // Same configured propagation resolution as other entry references.
        let dt=min(c.guidance.predictionStep,end-s.time)
        let targetAlpha=Self.alpha(blend:blend,mach:s.aerodynamics.mach,
          q:s.aerodynamics.dynamicPressure,ceiling:ceiling,aero:aero)
        alpha=moveToward(alpha,alpha+(targetAlpha-alpha)*(-expm1(-dt/pitchResponse)),
          maximumDelta:c.guidance.alphaSlewRate*dt)
        let targetBank=Self.bank(state:s,objective:objective,configuration:c,frame:frame,
          lift:max(0,s.aerodynamics.lift/s.mass)*(1-c.vehicle.aerodynamicUncertainty))
        bank=moveToward(bank,bank+(targetBank-bank)*(-expm1(-dt/rollResponse)),
          maximumDelta:min(c.entrySettings.rollRate,c.vehicle.controlPlant.maximumRollRate)*dt)
        let attitude=Quaternion.windAttitude(position:s.position,airVelocity:s.velocity-s.atmosphere.wind,alpha:alpha,bank:bank)
        let before=s, beforeEnergy=energy(s)
        let next=translation.step(.init(position:s.position,velocity:s.velocity),time:s.time,
          mass:s.mass,attitude:attitude,dt:dt,systems:s.systems)
        s.position=next.position;s.velocity=next.velocity;s.time+=dt
        let afterEnergy=energy(s)
        if afterEnergy<=endpoint, beforeEnergy>endpoint {
          let fraction=(beforeEnergy-endpoint)/(beforeEnergy-afterEnergy)
          s.position=before.position+(s.position-before.position)*fraction
          s.velocity=before.velocity+(s.velocity-before.velocity)*fraction
          s.time=before.time+dt*fraction;reached=true;break
        }
      }
      // Re-evaluate the interpolated endpoint before exposing its full state.
      // This does not change the range solve's commands or stopping energy.
      s.atmosphere=atmosphere.sample(position:s.position,time:s.time)
      s.attitude = .windAttitude(position:s.position,airVelocity:s.velocity-s.atmosphere.wind,alpha:alpha,bank:bank)
      s.aerodynamics=aero.evaluate(position:s.position,velocity:s.velocity,attitude:s.attitude,
        atmosphere:s.atmosphere,gear:s.systems.gearDown)
      captureInlet(s)
      let p = frame.coordinates(of:s.position)
      let v = frame.coordinateVelocity(at:s.position,velocity:s.velocity)
      let heading = abs(atan2(v.y,v.x))
      let air = atmosphere.sample(position:s.position,time:s.time)
      // Equal error units are the operator's capture tolerances, not tuned
      // weights or a fixed-speed turn falsely held throughout recovery.
      // The propagated commands already price their coupled lift and drag.
      let residual=objective.arrivalResidual(along:p.along,cross:p.cross,
        altitude:s.position.length-c.planet.radius,speed:s.velocity.length,
        heading:heading,soundSpeed:air.speedOfSound,configuration:c)
      return .init(createdAt:initial.time,blend:blend,cross:p.cross,headingError:heading,
        captureResidual:residual,along:p.along,
        altitude:s.position.length-c.planet.radius,speed:s.velocity.length,
        bracketed:false,reachedEnergy:reached,advisoryInlet:inlet)
    }
    let low=trial(0), high=trial(1)
    func better(_ a: Self, _ b: Self) -> Bool {
      if a.reachedEnergy != b.reachedEnergy { return a.reachedEnergy }
      return a.captureResidual < b.captureResidual
    }
    var best = better(low,high) ? low:high
    // A scalar along root does not solve the spatial/heading arrival. Search
    // the same propagated incidence family on the complete arrival residual.
    // Golden-section refinement retains the endpoints and stops when trial
    // endpoints differ by less than the configured capture resolution.
    let ratio = (sqrt(5.0)-1)/2
    var a=0.0, b=1.0
    var left=trial(b-ratio*(b-a)), right=trial(a+ratio*(b-a))
    for _ in 0...Double.significandBitCount {
      if better(left,best) {best=left}
      if better(right,best) {best=right}
      let separation = hypot(hypot(left.along-right.along,left.cross-right.cross),left.altitude-right.altitude)
      if max(separation,abs(left.captureResidual-right.captureResidual)*c.entrySettings.handoffRadius)
        <= c.entrySettings.handoffRadius || a.nextUp >= b {break}
      if better(left,right) {
        b=right.blend; right=left; left=trial(b-ratio*(b-a))
      } else {
        a=left.blend; left=right; right=trial(a+ratio*(b-a))
      }
    }
    // Preserve the diagnostic meaning: only a returned endpoint inside the
    // actual along tolerance with an along bracket may call itself bracketed.
    best.bracketed = low.reachedEnergy && high.reachedEnergy
      && (low.along-objective.along)*(high.along-objective.along)<=0
      && abs(best.along-objective.along)<=c.entrySettings.handoffRadius
    return best
  }
}
