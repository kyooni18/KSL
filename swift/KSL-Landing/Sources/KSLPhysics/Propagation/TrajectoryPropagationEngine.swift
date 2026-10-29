import Foundation
import KSLCore

public struct PredictionAttitude: Sendable {
  public let alpha: Double
  public let bank: Double
  public init(alpha: Double, bank: Double) {
    self.alpha = alpha
    self.bank = bank
  }
}

/// Reduced translation forecast with bounded attitude-rate/acceleration response.
/// The force/atmosphere model is shared; this is not a 6-DOF control qualification.
public final class TrajectoryPropagationEngine {
  public let configuration: MissionConfiguration
  public let dynamics: TranslationalDynamicsEngine
  private let rotation: RotationalDynamicsEngine
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    dynamics = .init(
      planet: configuration.planet, atmosphere: atmosphere, aerodynamics: aerodynamics)
    rotation = .init(configuration: configuration.vehicle)
  }
  public func predict(
    from initial: VehicleState, duration: Double, step: Double = 2, recordEvery: Int = 5,
    terminalSpeed: Double = 650, minimumDuration: Double = 0,
    law: (VehicleState) -> PredictionAttitude
  ) -> TrajectoryPrediction {
    var s = initial
    var samples: [TrajectorySample] = []
    var violations: [ConstraintViolation] = []
    var alpha = s.aerodynamics.angleOfAttack
    var bank = s.aerodynamics.bank
    var alphaRate = 0.0
    var bankRate = 0.0
    var reached = false
    var iteration = 0
    let end = s.time + duration
    let p = configuration.vehicle.controlPlant
    while s.time < end {
      let dt = min(step, end - s.time)
      s.atmosphere = dynamics.atmosphere.sample(position: s.position, time: s.time)
      s.attitude = .windAttitude(
        position: s.position, airVelocity: s.velocity - s.atmosphere.wind, alpha: alpha, bank: bank)
      s.aerodynamics = dynamics.aerodynamics.evaluate(
        position: s.position, velocity: s.velocity, attitude: s.attitude, atmosphere: s.atmosphere)
      let h = s.position.length - configuration.planet.radius
      if h < configuration.runway.elevation + 150 {
        violations.append(
          .init(.terrainCollision, "Prediction reached ground before a terminal solution"))
        break
      }
      if s.aerodynamics.airspeed < terminalSpeed && h < 35_000 && s.verticalSpeed < 0
        && s.time - initial.time >= minimumDuration
      {
        reached = true
        break
      }
      if s.aerodynamics.dynamicPressure > configuration.vehicle.maximumDynamicPressure * 1.01 {
        violations.append(
          .init(
            .excessiveDynamicPressure, "Entry forecast q limit",
            value: s.aerodynamics.dynamicPressure,
            limit: configuration.vehicle.maximumDynamicPressure))
        break
      }
      let load = abs(s.aerodynamics.lift) / (s.mass * 9.80665)
      if load > configuration.vehicle.maximumLoadFactor * 1.01 {
        violations.append(
          .init(
            .excessiveLoad, "Entry forecast load limit", value: load,
            limit: configuration.vehicle.maximumLoadFactor))
        break
      }
      let command = law(s)
      let kPa = s.aerodynamics.dynamicPressure / 1000
      let alphaTarget = min(
        command.alpha, rotation.maximumTrimAlpha(dynamicPressure: s.aerodynamics.dynamicPressure))
      let alphaAcceleration = min(p.pitchAccelerationMaximum, p.pitchAccelerationPerKPa * kPa)
      let bankAcceleration = min(p.rollAccelerationMaximum, p.rollAccelerationPerKPa * kPa)
      alphaRate = moveToward(
        alphaRate,
        clamp(
          (alphaTarget - alpha) / 3, -min(p.maximumPitchRate, configuration.guidance.alphaSlewRate),
          min(p.maximumPitchRate, configuration.guidance.alphaSlewRate)),
        maximumDelta: alphaAcceleration * dt)
      bankRate = moveToward(
        bankRate,
        clamp(
          wrapAngle(command.bank - bank) / 4, -configuration.guidance.bankSlewRate,
          configuration.guidance.bankSlewRate), maximumDelta: bankAcceleration * dt)
      alpha = clamp(
        alpha + alphaRate * dt, configuration.vehicle.minimumAngleOfAttack,
        configuration.vehicle.maximumAngleOfAttack)
      bank = wrapAngle(bank + bankRate * dt)
      s.attitude = .windAttitude(
        position: s.position, airVelocity: s.velocity - s.atmosphere.wind, alpha: alpha, bank: bank)
      let k = dynamics.step(
        .init(position: s.position, velocity: s.velocity), time: s.time, mass: s.mass,
        attitude: s.attitude, dt: dt)
      if recordEvery > 0 && iteration % recordEvery == 0 {
        samples.append(
          .init(
            time: s.time, position: s.position, velocity: s.velocity, angleOfAttack: alpha,
            bank: bank, dynamicPressure: s.aerodynamics.dynamicPressure, loadFactor: load))
      }
      s.position = k.position
      s.velocity = k.velocity
      s.time += dt
      iteration += 1
    }
    s.atmosphere = dynamics.atmosphere.sample(position: s.position, time: s.time)
    s.attitude = .windAttitude(
      position: s.position, airVelocity: s.velocity - s.atmosphere.wind, alpha: alpha, bank: bank)
    s.aerodynamics = dynamics.aerodynamics.evaluate(
      position: s.position, velocity: s.velocity, attitude: s.attitude, atmosphere: s.atmosphere)
    return .init(
      samples: samples, terminalState: s, violations: violations, duration: s.time - initial.time,
      reachedTerminalRegion: reached)
  }
}
