import Foundation
import KSLCore

public struct SurfaceActuatorState: Sendable {
  public var pitch = 0.0, roll = 0.0, yaw = 0.0
  public init() {}
  public init(command: ControlCommand) {
    pitch = command.pitch; roll = command.roll; yaw = command.yaw
  }
  public mutating func advance(command: ControlCommand, dt: Double, rate: Double) {
    pitch = moveToward(pitch, command.pitch, maximumDelta: rate * dt)
    roll = moveToward(roll, command.roll, maximumDelta: rate * dt)
    yaw = moveToward(yaw, command.yaw, maximumDelta: rate * dt)
  }
}

/// Provisional q-dependent moment plant plus rigid-body gyroscopic coupling.
/// Unidentified roll/yaw coefficients remain explicit configuration, not hidden assistance.
public final class RotationalDynamicsEngine: Sendable {
  public let configuration: VehicleConfiguration
  public let planet: PlanetConfiguration
  public init(configuration: VehicleConfiguration, planet: PlanetConfiguration = .init()) {
    self.configuration = configuration
    self.planet = planet
  }
  public func angularAcceleration(state: VehicleState, surfaces: SurfaceActuatorState) -> Vector3 {
    let p = configuration.controlPlant
    let kPa = max(0, state.aerodynamics.dynamicPressure / 1000)
    let stiffness = p.pitchStiffnessPerKPa * kPa
    let pitchControl = min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa)
    let rollControl = min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa)
    let yawControl = min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa)
    let w = state.angularVelocity
    let weathercock = p.yawStiffnessPerKPa * kPa
    let acceleration = Vector3(
      rollControl * surfaces.roll - p.rollDamping * sqrt(kPa / 10) * w.x,
      pitchControl * surfaces.pitch - stiffness
        * (state.aerodynamics.angleOfAttack - p.trimAngleOfAttack) - 2 * p.pitchDampingRatio
        * sqrt(stiffness) * w.y,
      yawControl * surfaces.yaw + weathercock * state.aerodynamics.sideslip - 0.6
        * sqrt(weathercock) * w.z)
    let inertia = configuration.inertia
    let moment = inertia.multiplied(componentwise: acceleration)
    let frameRate = state.attitude.conjugate.rotate(planet.angularVelocity)
    let inertialRate = w + frameRate
    // Euler's equation uses inertial angular velocity. The last term converts
    // its derivative back to body rates relative to the rotating planet.
    return (moment - inertialRate.cross(inertia.multiplied(componentwise: inertialRate))).divided(
      componentwise: inertia) + w.cross(frameRate)
  }
  public func maximumTrimAlpha(dynamicPressure: Double) -> Double {
    let p = configuration.controlPlant
    let kPa = max(0, dynamicPressure / 1000)
    guard kPa > 1e-8 else { return configuration.maximumAngleOfAttack }
    let authority = min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa)
    return min(
      configuration.maximumAngleOfAttack,
      p.trimAngleOfAttack + 0.92 * authority / (p.pitchStiffnessPerKPa * kPa))
  }
  /// Never interpret an inherited STS-N subsonic fit as an identified live
  /// STS trim limit. Local identified trim may constrain actual guidance.
  public func supportedTrimAlpha(_ state: VehicleState, identifiedCeiling: Double? = nil) -> Double {
    // The orbital/entry RCS setting is not evidence of aerodynamic trim
    // authority, and terminal guidance flies with RCS off.
    if let identifiedCeiling, identifiedCeiling.isFinite { return min(configuration.maximumAngleOfAttack, identifiedCeiling) }
    let a = state.aerodynamics
    if state.controlAuthority?.evidence == .provisional && a.mach <= 0.9
      && (500...12000).contains(a.dynamicPressure) {
      return maximumTrimAlpha(dynamicPressure: a.dynamicPressure)
    }
    return configuration.maximumAngleOfAttack
  }
}
