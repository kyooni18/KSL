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
    guard authority > 0, authority.isFinite else { return 0 }
    let direction = error >= 0 ? 1.0 : -1.0, distance = abs(error), rate = direction*relativeRate
    if distance < 1e-12 { return clamp(-relativeRate/(authority*max(hold, 0.01)), -1, 1) }
    let stopDistance = rate > 0 ? rate*rate/(2*authority) : 0
    if stopDistance >= distance { return -direction }
    var acceleration = authority
    if hold > 0 {
      let endRate = rate+authority*hold
      let remaining = distance-rate*hold-0.5*authority*hold*hold
      if endRate > 0 && remaining <= endRate*endRate/(2*authority) {
        let qa = hold*hold, qb = authority*hold*hold+2*rate*hold
        let qc = rate*rate+2*authority*rate*hold-2*authority*distance
        let discriminant = qb*qb-4*qa*qc
        if discriminant >= 0 { acceleration = clamp((-qb+sqrt(discriminant))/(2*qa), -authority, authority) }
      }
    }
    return direction*acceleration/authority
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
    return .init(pitch: demand.y, roll: demand.x, yaw: demand.z, gearDown: state.systems.gearDown)
  }
}
