import Foundation
import KSLCore

public struct SurfaceActuatorState: Sendable {
  public var pitch = 0.0, roll = 0.0, yaw = 0.0
  public var bodyFlapTrim = 0.0
  public var speedbrake = 0.0
  public init() {}
  public init(command: ControlCommand) {
    pitch = command.pitch; roll = command.roll; yaw = command.yaw
    bodyFlapTrim = command.bodyFlapTrimFraction ?? 0
    speedbrake = command.speedbrakeFraction ?? 0
  }
  public mutating func advance(command: ControlCommand, dt: Double, rate: Double) {
    pitch = moveToward(pitch, command.pitch, maximumDelta: rate * dt)
    roll = moveToward(roll, command.roll, maximumDelta: rate * dt)
    yaw = moveToward(yaw, command.yaw, maximumDelta: rate * dt)
    bodyFlapTrim = moveToward(bodyFlapTrim, command.bodyFlapTrimFraction ?? 0, maximumDelta: rate * dt)
    speedbrake = moveToward(speedbrake, command.speedbrakeFraction ?? 0, maximumDelta: rate * dt)
  }
}

/// Provisional q-dependent moment plant plus rigid-body gyroscopic coupling.
/// Unidentified roll/yaw coefficients remain explicit configuration, not hidden assistance.
public final class RotationalDynamicsEngine: Sendable {
  public let configuration: VehicleConfiguration
  public let planet: PlanetConfiguration
  private let pitchBalancingInput: Double?
  /// Explicit local measured residual, not additional actuator authority or a
  /// fitted global moment law. Callers must retain its evidence/support limits.
  private let localAngularAccelerationBias: Vector3
  public init(configuration: VehicleConfiguration, planet: PlanetConfiguration = .init(),
              pitchBalancingInput: Double? = nil, localAngularAccelerationBias: Vector3 = .zero) {
    self.configuration = configuration
    self.planet = planet
    self.pitchBalancingInput = pitchBalancingInput
    self.localAngularAccelerationBias = localAngularAccelerationBias
  }
  public func angularAcceleration(state: VehicleState, surfaces: SurfaceActuatorState) -> Vector3 {
    let p = configuration.controlPlant
    let kPa = max(0, state.aerodynamics.dynamicPressure / 1000)
    let authority = state.controlAuthority
    let anchored = authority?.isValid == true && [.measured, .identified].contains(authority!.evidence)
    // A measured live seed is not the direct-moment ShuttleSim surrogate.
    // Do not reintroduce the high-Mach moment the FCS explicitly rejects.
    // Explicit provisional simulations retain their configured plant.
    let stiffness = anchored ? 0 : p.pitchStiffnessPerKPa * kPa
    let pitchControl = anchored ? authority!.minimumAcceleration.y : min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa)
    let rollControl = anchored ? authority!.minimumAcceleration.x : min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa)
    let yawControl = anchored ? authority!.minimumAcceleration.z : min(p.yawAccelerationMaximum, p.yawAccelerationPerKPa * kPa)
    let w = state.angularVelocity
    let weathercock = p.yawStiffnessPerKPa * kPa
    let acceleration = Vector3(
      rollControl * surfaces.roll - p.rollDamping * sqrt(kPa / 10) * w.x,
      pitchControl * (surfaces.pitch + surfaces.bodyFlapTrim
        * (configuration.bodyFlapTrimInputAtFullDeflection ?? 0)
        - (anchored ? pitchBalancingInput ?? 0 : 0)) - stiffness
        * (state.aerodynamics.angleOfAttack - p.trimAngleOfAttack) - 2 * p.pitchDampingRatio
        * sqrt(stiffness) * w.y,
      yawControl * surfaces.yaw + weathercock * state.aerodynamics.sideslip - 0.6
        * sqrt(weathercock) * w.z)
    let inertia = authority?.isValid == true ? authority!.inertia : configuration.inertia
    let moment = inertia.multiplied(componentwise: acceleration)
    let frameRate = state.attitude.conjugate.rotate(planet.angularVelocity)
    let inertialRate = w + frameRate
    // Euler's equation uses inertial angular velocity. The last term converts
    // its derivative back to body rates relative to the rotating planet.
    return (moment - inertialRate.cross(inertia.multiplied(componentwise: inertialRate))).divided(
      componentwise: inertia) + w.cross(frameRate) + localAngularAccelerationBias
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
    // Configured RCS availability does not invalidate a measured local trim limit.
    if let identifiedCeiling, identifiedCeiling.isFinite {
      return clamp(identifiedCeiling, configuration.minimumAngleOfAttack, configuration.maximumAngleOfAttack)
    }
    // The orbital/entry RCS setting is not evidence of aerodynamic trim
    // authority, and terminal guidance flies with RCS off.
    let a = state.aerodynamics
    if state.controlAuthority?.evidence != .identified && a.mach <= 0.9
      && (500...12000).contains(a.dynamicPressure) {
      return maximumTrimAlpha(dynamicPressure: a.dynamicPressure)
    }
    return configuration.maximumAngleOfAttack
  }
}
