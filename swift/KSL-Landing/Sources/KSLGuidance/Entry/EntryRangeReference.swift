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
  var along: Double
  var altitude: Double
  var speed: Double
  var bracketed: Bool
  var reachedEnergy: Bool

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
      while s.time < end, s.position.length > c.planet.radius+c.runway.elevation {
        s.atmosphere=atmosphere.sample(position:s.position,time:s.time)
        s.attitude = .windAttitude(position:s.position,airVelocity:s.velocity-s.atmosphere.wind,alpha:alpha,bank:bank)
        s.aerodynamics=aero.evaluate(position:s.position,velocity:s.velocity,attitude:s.attitude,
          atmosphere:s.atmosphere,gear:s.systems.gearDown)
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
      return .init(createdAt:initial.time,blend:blend,along:frame.coordinates(of:s.position).along,
        altitude:s.position.length-c.planet.radius,speed:s.velocity.length,
        bracketed:false,reachedEnergy:reached)
    }
    var low=trial(0),high=trial(1)
    func residual(_ v: Self) -> Double {v.along-objective.along}
    var best=abs(residual(low))<abs(residual(high)) ? low:high
    if low.reachedEnergy != high.reachedEnergy {best=low.reachedEnergy ? low:high}
    guard low.reachedEnergy, high.reachedEnergy, residual(low)*residual(high)<=0 else {return best}
    // Bisection stops on configured arrival tolerance or floating resolution.
    // Bracket signs, not an assumed monotonic direction, determine each step.
    for _ in 0...Double.significandBitCount {
      let blend=(low.blend+high.blend)/2
      if blend==low.blend || blend==high.blend {break}
      var mid=trial(blend)
      guard mid.reachedEnergy else {break}
      mid.bracketed=true
      if abs(residual(mid))<abs(residual(best)) {best=mid}
      if abs(residual(mid))<=c.entrySettings.handoffRadius {return mid}
      if residual(low)*residual(mid)<=0 {high=mid} else {low=mid}
    }
    return best
  }
}
