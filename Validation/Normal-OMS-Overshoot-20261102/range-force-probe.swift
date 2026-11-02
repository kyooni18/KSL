import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance
final class LiveOvershootProbe: XCTestCase {
 struct Fixture: Decodable { let configuration: MissionConfiguration; let samples: [Sample] }
 struct Sample: Decodable { let state: VehicleState; let control: ControlDiagnostics; let objective: EntryCaptureObjective; let entry: EntryDiagnostics }
 func testRecordedForceAllocation() throws {
  let root=URL(fileURLWithPath:"/Users/kyooni18/Code/KSL")
  let f=try JSONDecoder().decode(Fixture.self,from:Data(contentsOf:root.appendingPathComponent("Validation/Normal-OMS-Overshoot-20261102/force-samples.json")))
  let c=f.configuration, atm=try AtmosphericModelEngine(planet:c.planet), aero=try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea)
  let frame=RunwayFrame(runway:c.runway,planet:c.planet)
  for sample in f.samples {
   let s=sample.state, a=s.aerodynamics, objective=sample.objective
   let rotation=RotationalDynamicsEngine(configuration:c.vehicle,planet:c.planet)
   let ceiling=rotation.supportedTrimAlpha(s,identifiedCeiling: sample.control.trimEvidence == .identified ? sample.control.trimAlphaCeiling : nil)
   let factor=a.dynamicPressure*aero.referenceArea/s.mass
   let current=aero.coefficients(mach:a.mach,alpha:a.angleOfAttack,dynamicPressure:a.dynamicPressure,beta:a.sideslip,gear:s.systems.gearDown)
   let liftGain=factor*current.lift>0 && a.lift>0 ? a.lift/s.mass/(factor*current.lift):1
   let p=frame.coordinates(of:s.position)
   let forward=frame.direction(at:p.horizontal,tangent:.init(1,0))
   let horizontal=s.velocity.projected(onPlaneWithNormal:s.position.normalized)
   let target=frame.position(along:objective.along,cross:objective.cross,height:0)
   let lateral=EntryLateralController.demand(state:s,configuration:c,azimuthError:wrapAngle(frame.bearing(from:s.position,to:target)-s.course),crossError:objective.cross-p.cross,crossRate:-frame.coordinateVelocity(at:s.position,velocity:s.velocity).y,alignment:horizontal.normalized.dot(forward),range:abs(objective.along-p.along))
   let inertial=c.planet.inertialVelocity(position:s.position,fixedVelocity:s.velocity)
   let support=max(0,(c.planet.gravitationalParameter/s.position.squaredLength-inertial.projected(onPlaneWithNormal:s.position.normalized).squaredLength/s.position.length)*cos(s.flightPathAngle))
   let alpha=EntryRangeReference.alpha(blend:sample.entry.rangeReferenceBlend ?? 0,mach:a.mach,q:a.dynamicPressure,ceiling:ceiling,aero:aero)
   let floor=min(ceiling,max(0,c.vehicle.minimumAngleOfAttack))
   let points=(0...64).map { i -> (alpha:Double,lift:Double,drag:Double) in
    let x=floor+(ceiling-floor)*Double(i)/64
    let polar=aero.coefficients(mach:a.mach,alpha:x,dynamicPressure:a.dynamicPressure,beta:a.sideslip,gear:s.systems.gearDown)
    return (x,max(0,polar.lift*factor*liftGain*(1-c.vehicle.aerodynamicUncertainty)),max(0,polar.drag*factor))
   }
   let safe=points.filter{$0.lift<=c.vehicle.maximumLoadFactor*9.80665*(1-c.vehicle.aerodynamicUncertainty)}
   let point=(safe.isEmpty ? points:safe).min{abs($0.alpha-alpha)<abs($1.alpha-alpha)}!
   let possible=points.map { ($0.alpha, NormalForceEnvelope(maximumLift:$0.lift,maximumBank:c.vehicle.maximumBank).lateralAuthority(reserving:support)) }.max{$0.1<$1.1}!
   let bank=EntryRangeReference.bank(state:s,objective:objective,configuration:c,frame:frame,lift:point.lift)
   print("FORCE",s.time,"rawLateral",lateral,"support",support,"ceilingDeg",degrees(ceiling),"selectedAlphaDeg",degrees(point.alpha),"selectedLift",point.lift,"selectedBankDeg",degrees(bank),"maximumPolarLateral",possible.1,"atAlphaDeg",degrees(possible.0),"loggedJointLift",sample.entry.jointRequiredLift as Any,"loggedMaxLift",sample.entry.jointMaximumLift as Any)
   if s.time<219000 { _=DiagnosticRangeReference.solve(from:s,objective:objective,configuration:c,atmosphere:atm,aero:aero,ceiling:ceiling,pitchResponse:sample.control.pitchResponseSeconds ?? c.guidance.pathLookaheadTime,rollResponse:sample.control.rollResponseSeconds ?? c.guidance.pathLookaheadTime) }
  }
 }
}
struct DiagnosticRangeReference: Sendable {
  var createdAt: Double
  var blend: Double
  var cross: Double
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
      return .init(createdAt:initial.time,blend:blend,cross:frame.coordinates(of:s.position).cross,along:frame.coordinates(of:s.position).along,
        altitude:s.position.length-c.planet.radius,speed:s.velocity.length,
        bracketed:false,reachedEnergy:reached)
    }
    var low=trial(0),high=trial(1)
    print("SHOOTING_ENDPOINTS", "objective", objective, "low", low, "high", high)
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
