import Foundation
import KSLCore

public final class RolloutController {
  private let frame: RunwayFrame
  private let configuration: MissionConfiguration
  private var brake = 0.0
  private var mainContact = false, noseContact = false
  private var groundHistory: [(time: Double, speed: Double, position: Vector3)] = []
  public private(set) var diagnostics: RolloutDiagnostics?
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func command(state: VehicleState, dt: Double) -> ControlCommand {
    let p = frame.coordinates(of: state.position)
    let desired = state.localFrame.heading(
      of: frame.direction(at: p.horizontal, tangent: .init(1, 0)))
    mainContact = mainContact || state.wheelContact?.mainGear == true
    noseContact = noseContact || (mainContact && state.wheelContact?.noseGear == true)
    let supported = state.contact.isGrounded && state.wheelContact?.mainGear == true
      && state.wheelContact?.noseGear == true
    let heading = state.localFrame.heading(of: state.attitude.rotate(.init(1, 0, 0)))
    let error = wrapAngle(desired - heading)
    let drift = state.horizontalSpeed*sin(wrapAngle(state.course-desired))
    // f1eebdc final/phases.inc, preserving degrees in the C steering gain.
    let rateWeight = clamp((state.horizontalSpeed-8)/35, 0, 1)
    let authority = clamp(55/max(state.horizontalSpeed, 15), 0.25, 1)
    let steering = clamp((degrees(error)*0.09-p.cross*0.0033-drift*0.036*rateWeight)*authority, -0.4, 0.4)
    // Main-contact pitch release is latched by FCS. The nose lowers under its
    // measured moments; neither a pitch estimate nor vessel situation enables brakes.
    // Native kRPC wheel brakes are boolean, not proportional. Request them
    // on verified support immediately; the OV-100 ModuleWheelBrakes already
    // supplies brakeResponse=1. An extra two-second software ramp was only
    // a one-second dead time before the >0.5 adapter threshold.
    let brakeRiseSeconds = 1.0 // installed ModuleWheelBrakes.brakeResponse
    brake = supported ? 1 : 0
    // A friction coefficient is not measured wheel braking. The old estimate
    // assumed full braking even at the start of the ramp and interpreted the
    // derotation force projection as sustained downforce. Price only observed
    // kinetic-energy loss per ground distance, including actual brakes/chutes.
    // Contact loss or a telemetry gap invalidates that observation.
    if !supported || dt <= 0 || dt > 1
      || groundHistory.last.map({ state.time <= $0.time || state.time-$0.time > 1 }) == true {
      groundHistory.removeAll()
    }
    var stoppingDistance: Double?
    if supported {
      groundHistory.append((state.time, state.horizontalSpeed, state.position))
      while groundHistory.count > 2, state.time-groundHistory[1].time >= brakeRiseSeconds {
        groundHistory.removeFirst()
      }
      if let first = groundHistory.first, state.time-first.time >= brakeRiseSeconds {
        let distance = (state.position-first.position).length
        let work = (first.speed*first.speed-state.horizontalSpeed*state.horizontalSpeed)/2
        if distance > 0, work > 0 {
          let observedDeceleration = work/distance
          stoppingDistance = state.horizontalSpeed*state.horizontalSpeed/(2*observedDeceleration)
        }
      }
    }
    diagnostics = .init(mainContact: mainContact, noseContact: noseContact,
      brakeInhibited: !supported, derotationRate: state.angularVelocity.y,
      stoppingDistance: stoppingDistance,
      remainingRunway: configuration.runway.length-p.along,
      contactEvidence: state.wheelContact?.evidence ?? .unavailable)
    return .init(
      pitch: 0, roll: 0, yaw: 0, gearDown: true, wheelBrake: brake,
      wheelSteering: steering,
      deployParachutes: (configuration.guidance.rolloutParachutes ?? true)
        && state.contact.isGrounded && state.wheelContact?.mainGear == true
        && (5...100).contains(state.horizontalSpeed) ? true : nil)
  }
}
