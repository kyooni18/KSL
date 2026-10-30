import Foundation
import KSLCore

public final class RolloutController {
  private let frame: RunwayFrame
  private let configuration: MissionConfiguration
  private var brake = 0.0
  private var mainContact = false, noseContact = false
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
    let heading = state.localFrame.heading(of: state.attitude.rotate(.init(1, 0, 0)))
    let error = wrapAngle(desired - heading)
    let drift = state.horizontalSpeed*sin(wrapAngle(state.course-desired))
    // f1eebdc final/phases.inc, preserving degrees in the C steering gain.
    let rateWeight = clamp((state.horizontalSpeed-8)/35, 0, 1)
    let authority = clamp(55/max(state.horizontalSpeed, 15), 0.25, 1)
    let steering = clamp((degrees(error)*0.09-p.cross*0.0033-drift*0.036*rateWeight)*authority, -0.4, 0.4)
    // Main-contact pitch release is latched by FCS. The nose lowers under its
    // measured moments; neither a pitch estimate nor vessel situation enables brakes.
    brake = noseContact ? moveToward(brake, 1, maximumDelta: dt/2) : 0
    let g = configuration.planet.gravitationalParameter/state.position.squaredLength
    let normal = max(0, g-state.aerodynamics.lift/state.mass)
    let deceleration = normal*configuration.vehicle.brakeFriction+state.aerodynamics.drag/state.mass
    diagnostics = .init(mainContact: mainContact, noseContact: noseContact,
      brakeInhibited: !noseContact, derotationRate: state.angularVelocity.y,
      stoppingDistance: deceleration > 0 ? state.horizontalSpeed*state.horizontalSpeed/(2*deceleration) : nil,
      remainingRunway: configuration.runway.length-p.along,
      contactEvidence: state.wheelContact?.evidence ?? .unavailable)
    return .init(
      pitch: 0, roll: 0, yaw: 0, gearDown: true, wheelBrake: brake,
      wheelSteering: steering,
      deployParachutes: configuration.guidance.rolloutParachutes == true && mainContact
        && (5...100).contains(state.horizontalSpeed) ? true : nil)
  }
}
