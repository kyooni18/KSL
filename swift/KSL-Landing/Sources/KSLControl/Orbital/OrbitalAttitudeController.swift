import Foundation
import KSLCore

/// Native vacuum pointing. Stops with measured bidirectional torque/inertia;
/// orbital attitude acquisition never engages kRPC AutoPilot or SAS.
public final class OrbitalAttitudeController {
  private var lastTarget: Quaternion?
  private var lastTime: Double?
  public private(set) var alignmentError = Double.pi
  public private(set) var diagnostics = ControlDiagnostics()
  public init() {}
  public static func boundedDemand(error: Double, relativeRate: Double, authority: Double,
                                    hold: Double) -> Double {
    guard authority > 0, authority.isFinite,
      error.isFinite, relativeRate.isFinite, hold.isFinite else { return 0 }
    // Rate-shaped pointing with a measured braking limit. The former
    // bang-bang switch demanded +/-100% torque when a 0.05-degree error
    // remained, continually overshooting and firing RCS.
    //
    // The settling horizon spans three observed control/actuator delays:
    // first converge at a bounded angular speed, then damp actual angular
    // velocity instead of commanding another full acceleration reversal.
    let horizon = max(3*max(hold, 0.01), 0.03)
    let stopSpeed = sqrt(max(0, 2*authority*abs(error)))
    let desiredRate = (error < 0 ? -1.0 : 1.0)
      * min(stopSpeed, abs(error)/horizon)
    let acceleration = (desiredRate-relativeRate)/horizon
    return clamp(acceleration/authority, -1, 1)
  }
  /// Compute the nose vector that gives the desired physical thrust vector
  /// after applying the measured net body-FRD engine cant. The companion
  /// attitude is constrained to the orbital radial-up reference, not a free
  /// roll. A symmetric canted OMS pair has almost zero body lateral component.
  public static func compensatedNoseDirection(desiredThrust: Vector3,
                                                position: Vector3,
                                                bodyThrustDirection: Vector3) -> Vector3 {
    let effective = bodyThrustDirection.normalized
    let ideal = Quaternion.windAttitude(position: position,
      airVelocity: desiredThrust.normalized, alpha: 0, bank: 0)
    let front = Vector3(1, 0, 0)
    let axis = effective.cross(front)
    let correction = Quaternion.axisAngle(axis.normalized,
      angle: acos(clamp(effective.dot(front), -1, 1)))
    return (ideal * correction).rotate(front).normalized
  }
  public func command(state: VehicleState, direction: Vector3, dt: Double) throws -> ControlCommand {
    guard dt > 0, dt <= 1, direction.isFinite, direction.length > 0.99,
      let authority = state.controlAuthority, authority.isValid,
      authority.minimumAcceleration.x > 0, authority.minimumAcceleration.y > 0,
      authority.minimumAcceleration.z > 0 else {
      throw FlightError.invalidTelemetry("Orbital attitude requires coherent measured torque/inertia")
    }
    if let time = lastTime, state.time <= time { throw FlightError.rearmRequired("Orbital attitude clock discontinuity") }
    let forward = direction.normalized
    var roof = state.position.normalized.projected(onPlaneWithNormal: forward).normalized
    if roof.length < 0.5 { roof = state.localFrame.north.projected(onPlaneWithNormal: forward).normalized }
    let target = Quaternion(forward: forward, right: forward.cross(roof).normalized, down: -roof)
    let error = AirRelativeAttitude.error(current: state.attitude, target: target)
    alignmentError = acos(clamp(state.attitude.rotate(.init(1, 0, 0)).dot(forward), -1, 1))
    var targetRate = Vector3.zero
    if let previous = lastTarget {
      let delta = AirRelativeAttitude.error(current: previous, target: target)/dt
      targetRate = state.attitude.conjugate.rotate(previous.rotate(delta))
    }
    let rate = state.angularVelocity-targetRate, a = authority.minimumAcceleration
    let demand = Vector3(
      Self.boundedDemand(error: error.x, relativeRate: rate.x, authority: a.x, hold: dt+ControlLaw.rollLag),
      Self.boundedDemand(error: error.y, relativeRate: rate.y, authority: a.y, hold: dt+ControlLaw.pitchLag),
      Self.boundedDemand(error: error.z, relativeRate: rate.z, authority: a.z, hold: dt+ControlLaw.yawLag))
    lastTarget = target; lastTime = state.time
    diagnostics = .init(); diagnostics.quaternionError = error; diagnostics.bodyRate = state.angularVelocity
    diagnostics.relativeRate = rate; diagnostics.authority = a; diagnostics.authorityEvidence = authority.evidence
    diagnostics.rawDemand = demand; diagnostics.appliedDemand = demand; diagnostics.sampleInterval = dt
    // Proven powered reaction wheels supply measured three-axis torque without
    // consuming monopropellant. Do not keep the RCS jets enabled during hours
    // of orbital coast just because the vehicle config permits them.
    // Unknown/wheel-less actuator provenance retains the original RCS option.
    let wheelAvailable = authority.evidence == .measured
      && (authority.actuatorSources?.contains("active-powered-reaction-wheels") == true)
    return .init(pitch: demand.y, roll: demand.x, yaw: demand.z,
      gearDown: state.systems.gearDown, rcs: !wheelAvailable)
  }
}
