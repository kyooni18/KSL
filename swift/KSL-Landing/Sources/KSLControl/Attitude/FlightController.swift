import Foundation
import KSLCore
import KSLPhysics

public struct ControlDiagnostics: Codable, Sendable {
  public let pitchSaturated: Bool
  public let rollSaturated: Bool
  public let yawSaturated: Bool
  public let pitchError: Double
  public let bankError: Double
}

/// q-scheduled dynamic inversion with rate feedback and conditional integration.
public final class FlightController {
  private let configuration: MissionConfiguration
  private let rollout: RolloutController
  private var pitchIntegral = 0.0
  private var previous = ControlCommand()
  private var lastTarget: GuidanceTarget?
  private var alphaTargetRate = 0.0
  private var bankTargetRate = 0.0
  public private(set) var diagnostics = ControlDiagnostics(
    pitchSaturated: false, rollSaturated: false, yawSaturated: false, pitchError: 0, bankError: 0)
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration
    rollout = .init(configuration: configuration)
  }
  public func command(state: VehicleState, solution: GuidanceSolution, dt: Double) throws
    -> ControlCommand
  {
    guard dt > 0, dt <= 1 else { throw FlightError.staleTelemetry }
    if state.contact.isGrounded { return rollout.command(state: state, dt: dt) }
    if solution.phase == .abort { return .init(gearDown: state.systems.gearDown) }
    let p = configuration.vehicle.controlPlant
    let a = state.aerodynamics
    let target = solution.target
    let kPa = max(0, a.dynamicPressure / 1000)
    let pitchAuthority = min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa)
    let rollAuthority = min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa)
    let yawAuthority = min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa)
    let pitchError = wrapAngle(target.angleOfAttack - a.angleOfAttack)
    let bankError = wrapAngle(target.bank - a.bank)
    guard pitchAuthority > 1e-8, rollAuthority > 1e-8 else {
      pitchIntegral = 0
      previous = .init(gearDown: target.gearDown)
      return previous
    }
    let speed = max(20, a.airspeed)
    let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
    let gammaRate =
      (a.lift / state.mass * cos(a.bank) - (g - speed * speed / state.position.length)
        * cos(state.flightPathAngle))
      / speed
    let courseRate = a.lift / state.mass * sin(a.bank) / max(20, state.horizontalSpeed)
    let feedPitch = gammaRate * cos(a.bank) + courseRate * cos(state.flightPathAngle) * sin(a.bank)
    let feedYaw = courseRate * cos(state.flightPathAngle) * cos(a.bank) - gammaRate * sin(a.bank)
    let authorityScale = sqrt(clamp(a.dynamicPressure / 250, 0.001, 1))
    let wn = 0.9 * authorityScale
    let rollWN = 1.1 * authorityScale
    if let lastTarget {
      let filter = 1 - exp(-dt / 0.25)
      let alphaRate = clamp(
        (target.angleOfAttack - lastTarget.angleOfAttack) / dt, -p.maximumPitchRate,
        p.maximumPitchRate)
      let bankRate = clamp(
        wrapAngle(target.bank - lastTarget.bank) / dt, -p.maximumRollRate, p.maximumRollRate)
      alphaTargetRate += filter * (alphaRate - alphaTargetRate)
      bankTargetRate += filter * (bankRate - bankTargetRate)
    }
    lastTarget = target
    let desiredPitchRate = clamp(
      wn * pitchError + feedPitch + alphaTargetRate, -p.maximumPitchRate, p.maximumPitchRate)
    let pitchAcceleration = 2 * wn * (desiredPitchRate - state.angularVelocity.y)
    let stiffness = p.pitchStiffnessPerKPa * kPa
    let trim = stiffness * (a.angleOfAttack - p.trimAngleOfAttack)
    let pitchRaw =
      (pitchAcceleration + trim + 2 * p.pitchDampingRatio * sqrt(stiffness)
        * state.angularVelocity.y + pitchIntegral) / pitchAuthority
    if abs(pitchRaw) < 0.95 && a.dynamicPressure > 100 {
      pitchIntegral = clamp(
        pitchIntegral + pitchError * 0.015 * dt, -pitchAuthority * 0.2, pitchAuthority * 0.2)
    } else {
      pitchIntegral *= exp(-dt / 10)
    }
    let desiredRollRate = clamp(
      rollWN * bankError + bankTargetRate, -p.maximumRollRate, p.maximumRollRate)
    let rollRaw =
      (2 * rollWN * (desiredRollRate - state.angularVelocity.x) + p.rollDamping * sqrt(kPa / 10)
        * state.angularVelocity.x) / rollAuthority
    let weathercock = p.yawStiffnessPerKPa * kPa
    let yawAcceleration = 1.2 * a.sideslip - 1.5 * (state.angularVelocity.z - feedYaw)
    let yawRaw =
      yawAuthority > 1e-8
      ? (yawAcceleration - weathercock * a.sideslip + 0.6 * sqrt(weathercock)
        * state.angularVelocity.z) / yawAuthority : 0
    diagnostics = .init(
      pitchSaturated: abs(pitchRaw) > 1, rollSaturated: abs(rollRaw) > 1,
      yawSaturated: abs(yawRaw) > 1, pitchError: pitchError, bankError: bankError)
    let delta = p.surfaceRate * dt
    previous = .init(
      pitch: moveToward(previous.pitch, clamp(pitchRaw, -1, 1), maximumDelta: delta),
      roll: moveToward(previous.roll, clamp(rollRaw, -1, 1), maximumDelta: delta),
      yaw: moveToward(previous.yaw, clamp(yawRaw, -1, 1), maximumDelta: delta),
      gearDown: target.gearDown)
    try previous.validate(grounded: false)
    return previous
  }
}
