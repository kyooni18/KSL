import Foundation
import KSLCore
import KSLPhysics

/// The same propagated incidence reference used by MM304's command law.
public struct EntryHandoffForecast: Codable, Sendable {
  public let createdAt: Double
  public let arrivalResidual: Double
  public init(createdAt:Double,arrivalResidual:Double) {
    self.createdAt=createdAt;self.arrivalResidual=arrivalResidual
  }
}

/// A local comparison of handing off now against the accepted MM304 forecast.
/// It is a timing advisory, never a landing or force-feasibility certificate.
public struct EntryHandoffTiming: Codable, Sendable {
  public var transferNow = false
  public var reason = "continue-entry"
  public var energyMargin: Double?
  public var responseEnergy: Double?
  public var secondsToClosestApproach: Double?
  public var currentArrivalResidual: Double?
  public var forecastArrivalResidual: Double?
  public var localHeadingTurnSeconds: Double?

  var diagnostic:String {
    var parts=["handoffTiming=\(reason)"]
    for (name,value) in [("energyMargin",energyMargin),("responseEnergy",responseEnergy),
      ("closestApproach",secondsToClosestApproach),("headingTurnTime",localHeadingTurnSeconds),
      ("currentResidual",currentArrivalResidual),("forecastResidual",forecastArrivalResidual)] {
      if let value {parts.append("\(name)=\(value)")}
    }
    return parts.joined(separator:" ")
  }

  static func maximumLift(state:VehicleState,configuration c:MissionConfiguration,
    aerodynamics:AerodynamicsEngine,ceiling:Double,liftGain:Double=1)->Double {
    let a=state.aerodynamics
    return min(c.vehicle.maximumLoadFactor*9.80665,
      aerodynamics.liftSolution(acceleration:0,mass:state.mass,dynamicPressure:a.dynamicPressure,
        mach:a.mach,minimumAlpha:min(ceiling,max(0,c.vehicle.minimumAngleOfAttack)),
        maximumAlpha:ceiling,gear:state.systems.gearDown).maximumAcceleration
        * liftGain*(1-c.vehicle.aerodynamicUncertainty))
  }

  public static func assess(state s: VehicleState, configuration c: MissionConfiguration,
    objective: EntryCaptureObjective, responseSeconds: Double,
    forecast: EntryHandoffForecast? = nil, maximumLiftAcceleration:Double? = nil) -> Self {
    var result=Self()
    func transfer(_ reason:String) -> Self {
      var copy=result;copy.transferNow=true;copy.reason=reason;return copy
    }
    guard s.verticalSpeed < 0 else {result.reason="not-descending";return result}
    let altitude=s.position.length-c.planet.radius
    if altitude<=c.mm304HandoffDeadlineAltitude {return transfer("altitude-deadline")}
    let a=s.aerodynamics,e=c.entrySettings
    guard a.dynamicPressure>0,a.mach>0 else {result.reason="outside-atmosphere";return result}
    if a.mach<=c.mm304DeliveryMach-e.terminalMachHalfWidth {return transfer("mach-deadline")}
    let energy=s.velocity.squaredLength/2-c.planet.gravitationalParameter/s.position.length
      - c.planet.angularVelocity.cross(s.position).squaredLength/2
    let margin=energy-objective.specificEnergy(configuration:c)
    result.energyMargin=margin
    // A delivery miss must not defer an already-due energy handoff.
    if margin<=0 {return transfer("interface-energy")}
    // The existing upper operating window prevents thin-air/high-Mach
    // closest approaches from forcing TAEM prematurely. Failed feasibility
    // assessments do not veto recovery inside this timing window.
    guard altitude<=e.terminalMaximumAltitude,
      a.mach<=c.mm304DeliveryMach+e.terminalMachHalfWidth else {
      result.reason="before-terminal-window";return result
    }
    let response=max(c.guidance.controlInterval,responseSeconds)
    // The actual measured drag work spent while attitude/control changes
    // owner is unavailable for a later turn. Lead by that physical reserve.
    let loss=max(0,a.drag/s.mass)*a.airspeed*response
    result.responseEnergy=loss
    if margin<=loss {return transfer("handover-response-reserve")}
    let frame=RunwayFrame(runway:c.runway,planet:c.planet)
    let p=frame.coordinates(of:s.position)
    let v=frame.coordinateVelocity(at:s.position,velocity:s.velocity)
    let offset=Vector2(p.along-objective.along,p.cross-objective.cross)
    if v.dot(v)>0 {
      let closest = -offset.dot(v)/v.dot(v)
      result.secondsToClosestApproach=closest
      // Closest approach is a local geometric opportunity, not proof of a
      // capture. Once its response lead is spent, waiting cannot improve
      // this straight-line intercept. TAEM retains best-effort ownership.
      if let lift=maximumLiftAcceleration, lift>0 {
        // Optimistic local lateral bound: even spending the maximum banked
        // lift on steering needs this much heading time. It omits the normal
        // support reservation, so it must not be called a feasible turn.
        // Future q can change; this is a latest-start warning, not a global
        // reachability theorem or a new limit on either guidance law.
        let lateral=lift*sin(c.vehicle.maximumBank)
        let headingTime=abs(atan2(v.y,v.x))*v.length/lateral
        result.localHeadingTurnSeconds=headingTime
        if closest<=response+headingTime {return transfer("heading-turn-response-lead")}
      }
      if closest<=response {return transfer("closest-approach-response-lead")}
    }
    let current=objective.arrivalResidual(along:p.along,cross:p.cross,
      altitude:altitude,speed:a.airspeed,heading:abs(atan2(v.y,v.x)),
      soundSpeed:s.atmosphere.speedOfSound,configuration:c)
    result.currentArrivalResidual=current
    if let f=forecast,f.createdAt<=s.time,s.time-f.createdAt<=2*c.guidance.entryReplanInterval {
      let later=f.arrivalResidual
      result.forecastArrivalResidual=later
      // Compare the same objective and tolerances. This is explicitly a
      // current-vs-forecast choice, not an assertion of a global optimum.
      if current<=later {return transfer("current-inlet-better-than-entry-forecast")}
      result.reason="entry-forecast-still-improves-inlet"
    } else {result.reason="continue-entry-no-fresh-comparison"}
    return result
  }
}
