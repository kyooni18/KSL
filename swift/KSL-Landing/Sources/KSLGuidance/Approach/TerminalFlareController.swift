import Foundation
import KSLCore
import KSLPhysics

public struct FlareReference: Sendable {
  public let latched: Bool
  public let arcActive: Bool
  public let wheelHeight: Double
  public let desiredSink: Double
  public let upwardAcceleration: Double
  public let triggerHeight: Double
  public let oneGravitySpeed: Double
  public let availablePull: Double
  public let designAcceleration: Double
}

/// C f1eebdc guidance/final/sequence.inc: response-led constant-deceleration
/// arc, short shallow glide, final flare and float. These are sizing laws;
/// only a full native-controller forecast may qualify a terminal route.
public final class TerminalFlareController {
  private let configuration: MissionConfiguration
  private let aero: AerodynamicsEngine
  private let frame: RunwayFrame
  private var latched = false
  private var arcStarted = false
  public init(configuration: MissionConfiguration, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; aero = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public static func sinkProfile(height: Double, arrestAcceleration: Double, gravity: Double,
                                 floatHeight: Double = 1.5, floatSink: Double = 1) -> Double {
    let shallowSink = 3.5, shallowHeight = 12.0, finalAcceleration = 0.1 * gravity
    if height > shallowHeight {
      return sqrt(shallowSink * shallowSink + 2 * arrestAcceleration * (height-shallowHeight))
    }
    let finalHeight = floatHeight + max(0, shallowSink*shallowSink-floatSink*floatSink)/(2*finalAcceleration)
    if height > finalHeight { return shallowSink }
    if height > floatHeight { return sqrt(floatSink*floatSink+2*finalAcceleration*(height-floatHeight)) }
    return floatSink
  }
  public func update(_ state: VehicleState, liftEffectiveness: Double, responseSeconds: Double,
                     maximumAlpha: Double) -> FlareReference {
    let c = configuration, a = state.aerodynamics
    let p = frame.coordinates(of: state.position)
    let pavement = c.runway.contains(along: p.along, cross: p.cross)
    let height = max(0, (pavement ? state.radarAltitude : p.height)-c.vehicle.gearHeight)
    let gravity = c.planet.gravitationalParameter/state.position.squaredLength
    let maxLift = aero.liftSolution(acceleration: .greatestFiniteMagnitude, mass: state.mass,
      dynamicPressure: a.dynamicPressure, mach: a.mach,
      minimumAlpha: c.vehicle.minimumAngleOfAttack, maximumAlpha: maximumAlpha,
      beta: a.sideslip, gear: true).maximumAcceleration * liftEffectiveness
    let available = max(0, maxLift*cos(a.bank)-gravity*cos(state.flightPathAngle))
    // The C design reserve is bounded by actual available lift; its .12 g
    // floor must never manufacture authority when the measured model lacks it.
    let arrest = min(available, clamp(0.4*available, 0.12*gravity, 0.35*gravity))
    let sink = max(0, -state.verticalSpeed)
    let shallowSink = 3.5, shallowHeight = 12.0
    let pull = aero.liftSolution(acceleration: (gravity*cos(state.flightPathAngle)+arrest)
        / max(0.01, liftEffectiveness), mass: state.mass, dynamicPressure: a.dynamicPressure,
      mach: a.mach, minimumAlpha: 0, maximumAlpha: maximumAlpha, gear: true)
    let slew = min(radians(5), c.vehicle.controlPlant.maximumPitchRate*0.8)
    let response = max(responseSeconds, 1.5) + max(0, pull.alpha-a.angleOfAttack)/slew
    let pullHeight = max(0, sink*sink-shallowSink*shallowSink)/(2*max(0.01, arrest))
    let trigger = max(c.guidance.flareHeight, shallowHeight+pullHeight+sink*response)
    if height <= trigger { latched = true }
    let onsetRoom = max(0, height-shallowHeight-sink*(response+1))
    let onsetSink = sqrt(shallowSink*shallowSink+2*arrest*onsetRoom)
    if latched && (onsetSink < sink || sink <= shallowSink || height <= shallowHeight) {
      arcStarted = true
    }
    let oneGravitySpeed = maxLift > 0 ? a.airspeed*sqrt(gravity/maxLift) : .infinity
    guard latched && arcStarted else {
      return .init(latched: latched, arcActive: false, wheelHeight: height, desiredSink: sink,
        upwardAcceleration: 0, triggerHeight: trigger,
        oneGravitySpeed: oneGravitySpeed, availablePull: available, designAcceleration: arrest)
    }
    let longLanding = p.along > c.guidance.touchdownAlong + 0.35*c.runway.length
    let floatHeight = longLanding ? 0.0 : 1.5
    let floatSink = longLanding ? 1.8 : abs(c.guidance.touchdownSinkRate ?? -1.0)
    var acceleration: Double, desired: Double
    if height > shallowHeight {
      let residualLag = min(response, max(1, abs(pull.alpha-a.angleOfAttack)/slew))
      let room = max(0, height-shallowHeight-sink*residualLag)
      let designSink = sqrt(shallowSink*shallowSink+2*arrest*room)
      let observedPull = max(0, a.lift*cos(a.bank)/state.mass-gravity*cos(state.flightPathAngle))
      let predictedSink = max(0, sink-observedPull)
      acceleration = sink > shallowSink ? max(0, arrest+(predictedSink-designSink)/1.5)
        : (sink-shallowSink)/1.5
      desired = max(shallowSink, sink-acceleration*1.5)
    } else {
      let lead = max(0, height-sink*min(response, 1.0))
      let profile = Self.sinkProfile(height: lead, arrestAcceleration: arrest, gravity: gravity,
        floatHeight: floatHeight, floatSink: floatSink)
      desired = height > 4.5 ? profile : min(profile, max(floatSink, sink))
      // C float energy guard: use the measured drag and remaining speed
      // above max-lift 1-g speed to settle while lift can still support it.
      let deceleration = a.drag/state.mass
      var settling = false
      if deceleration > 0.1 && oneGravitySpeed.isFinite && height > 0 {
        let floatTime = (a.airspeed-1.05*oneGravitySpeed)/deceleration
        let settle = floatTime > 0 ? height/floatTime : Double.infinity
        if min(settle, 2) > desired { desired = min(settle, 2); settling = true }
      }
      let finalHeight = floatHeight+(shallowSink*shallowSink-floatSink*floatSink)/(0.2*gravity)
      let feedforward = lead > floatHeight && lead <= finalHeight && sink > floatSink ? 0.1*gravity : 0
      let leadSink = sink+0.5*(gravity*cos(state.flightPathAngle)-a.lift*cos(a.bank)/state.mass)
      acceleration = leadSink-desired+feedforward
      if settling { acceleration = max(acceleration, -0.1*gravity) }
    }
    return .init(latched: true, arcActive: true, wheelHeight: height, desiredSink: desired,
      upwardAcceleration: clamp(acceleration, -0.3*gravity, min(1.5*gravity, available)),
      triggerHeight: trigger, oneGravitySpeed: oneGravitySpeed, availablePull: available, designAcceleration: arrest)
  }
}
