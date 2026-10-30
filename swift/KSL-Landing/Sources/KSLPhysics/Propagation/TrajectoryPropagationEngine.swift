import Foundation
import KSLControl
import KSLCore

public struct PredictionAttitude: Sendable {
  public let alpha: Double, bank: Double
  public init(alpha: Double, bank: Double) { self.alpha = alpha; self.bank = bank }
}

/// Closed-loop six-DOF forecast. Every branch owns the same native FCS and
/// actuator/rigid-body integrator used by offline flight. Guidance targets never
/// overwrite the physical attitude. Forecasts retain unqualified moment priors.
public final class TrajectoryPropagationEngine {
  public let configuration: MissionConfiguration
  public let dynamics: TranslationalDynamicsEngine
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    dynamics = .init(planet: configuration.planet, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func predict(
    from initial: VehicleState, duration: Double, step: Double = 2, recordEvery: Int = 5,
    terminalSpeed: Double = 650, minimumDuration: Double = 0,
    controlSnapshot: FlightControlSnapshot? = nil, previousTarget: GuidanceTarget? = nil,
    law: (VehicleState) -> PredictionAttitude
  ) -> TrajectoryPrediction {
    var samples: [TrajectorySample] = [], violations: [ConstraintViolation] = []
    var s = initial, reached = false
    guard duration.isFinite, duration >= 0, step.isFinite, step > 0 else {
      return .init(samples: [], terminalState: s,
        violations: [.init(.invalidGeometry, "Invalid prediction interval")], duration: 0, reachedTerminalRegion: false)
    }
    let controller = FlightController(configuration: configuration, snapshot: controlSnapshot)
    let physics = ShuttlePhysicsEngine(configuration: configuration, atmosphere: dynamics.atmosphere,
      aerodynamics: dynamics.aerodynamics, initialControls: initial.appliedControls ?? .init())
    let controlStep = min(configuration.guidance.controlInterval, step)
    let end = initial.time + duration
    var nextRecord = initial.time
    var limiter = GuidanceTargetLimiter(previous: previousTarget)
    s = physics.observe(s)
    while s.time < end {
      if Task.isCancelled { break }
      let height = s.position.length - configuration.planet.radius
      let load = abs(s.aerodynamics.lift) / (s.mass * 9.80665)
      if height < configuration.runway.elevation + 150 || s.contact == .crashed {
        violations.append(.init(.terrainCollision, "Closed-loop prediction reached terrain before terminal admission")); break
      }
      if s.aerodynamics.airspeed < terminalSpeed && height < 35_000 && s.verticalSpeed < 0
        && s.time - initial.time >= minimumDuration { reached = true; break }
      if s.aerodynamics.dynamicPressure > configuration.vehicle.maximumDynamicPressure {
        violations.append(.init(.excessiveDynamicPressure, "Closed-loop forecast q limit",
          value: s.aerodynamics.dynamicPressure, limit: configuration.vehicle.maximumDynamicPressure)); break
      }
      if load > configuration.vehicle.maximumLoadFactor {
        violations.append(.init(.excessiveLoad, "Closed-loop forecast load limit",
          value: load, limit: configuration.vehicle.maximumLoadFactor)); break
      }
      if recordEvery > 0, s.time >= nextRecord {
        samples.append(.init(time: s.time, position: s.position, velocity: s.velocity,
          angleOfAttack: s.aerodynamics.angleOfAttack, bank: s.aerodynamics.bank,
          dynamicPressure: s.aerodynamics.dynamicPressure, loadFactor: load))
        nextRecord += step * Double(recordEvery)
      }
      let dt = min(controlStep, end - s.time)
      let requested = law(s)
      let target = limiter.update(.init(angleOfAttack: requested.alpha, bank: requested.bank),
        phase: .entry, configuration: configuration, dt: dt, dynamicPressure: s.aerodynamics.dynamicPressure,
        rollResponseSeconds: controller.diagnostics.rollResponseSeconds,
        pitchResponseSeconds: controller.diagnostics.pitchResponseSeconds)
      do {
        let command = try controller.command(state: s,
          solution: .init(phase: .entry, target: target), dt: dt)
        s = try physics.advance(s, command: command, duration: dt, maximumStep: 0.02)
      } catch {
        violations.append(.init(.actuatorAuthority, "Closed-loop forecast failed: \(error)")); break
      }
    }
    return .init(samples: samples, terminalState: s, violations: violations,
      duration: s.time - initial.time, reachedTerminalRegion: reached)
  }
}
