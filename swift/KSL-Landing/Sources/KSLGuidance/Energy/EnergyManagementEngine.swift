import Foundation
import KSLCore
import KSLPhysics

public final class EnergyManagementEngine {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aero: AerodynamicsEngine
  private let rotation: RotationalDynamicsEngine
  private let frame: RunwayFrame
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    self.atmosphere = atmosphere
    aero = aerodynamics
    rotation = .init(configuration: configuration.vehicle)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func energyHeight(_ state: VehicleState) -> Double {
    let r = state.position.length
    let rr = configuration.planet.radius + configuration.runway.elevation
    let g = configuration.planet.gravitationalParameter / (rr * rr)
    return
      (configuration.planet.gravitationalParameter * (1 / rr - 1 / r) + state.velocity.squaredLength
      / 2) / g
  }
  public func evaluate(
    stations: [PathStation], from initial: VehicleState, dragMultiplier: Double = 1,
    liftMultiplier: Double = 1
  ) -> ([PathStation], TerminalAssessment) {
    var result: [PathStation] = []
    var violations: [ConstraintViolation] = []
    var speed = max(1, initial.aerodynamics.airspeed)
    var time = initial.time
    var maxBank = 0.0
    var maxQ = 0.0
    var maxLoad = 0.0
    var minLift = Double.infinity
    var previousBank = initial.aerodynamics.bank
    let v = configuration.vehicle
    func finish(_ terminalSpeed: Double) -> TerminalAssessment {
      let penalty = Double(violations.count) * 1e6
      let score =
        penalty + pow(terminalSpeed - v.landingSpeed, 2) + (stations.last?.distance ?? 0) * 0.0001
      return .init(
        feasible: violations.isEmpty, violations: violations, touchdownSpeed: terminalSpeed,
        minimumLiftMargin: minLift.isFinite ? minLift : 0, maximumBank: maxBank, maximumQ: maxQ,
        maximumLoad: maxLoad,
        score: score, uncertaintyFraction: v.aerodynamicUncertainty)
    }
    for i in stations.indices {
      let s = stations[i]
      let pos = frame.position(along: s.position.x, cross: s.position.y, height: s.height)
      let atm = atmosphere.sample(position: pos, time: time)
      let gamma = atan(s.slope)
      let q = 0.5 * atm.density * speed * speed
      let g = configuration.planet.gravitationalParameter / pow(pos.length, 2)
      let vh = speed * cos(gamma)
      let vertical =
        (g - vh * vh / pos.length) * cos(gamma) + speed * speed * s.verticalCurvature
      let lateral = vh * vh * s.curvature
      let bank = atan2(lateral, max(0.1, vertical))
      let required = hypot(vertical, lateral)
      let maxAlpha = min(
        rotation.maximumTrimAlpha(dynamicPressure: q),
        s.height < 500 ? v.maximumLandingAngleOfAttack : v.maximumAngleOfAttack)
      let lift = aero.liftSolution(
        acceleration: required / max(0.01, liftMultiplier), mass: initial.mass, dynamicPressure: q,
        mach: speed / atm.speedOfSound, minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: maxAlpha
      )
      let load = required / 9.80665
      maxBank = max(maxBank, abs(bank))
      maxQ = max(maxQ, q)
      maxLoad = max(maxLoad, load)
      minLift = min(minLift, lift.maximumAcceleration * liftMultiplier - required)
      if !lift.achievable {
        violations.append(
          .init(
            .insufficientLift, "Path demands unavailable lift/trim authority", value: required,
            limit: lift.maximumAcceleration * liftMultiplier))
      }
      if abs(bank) > v.maximumBank {
        violations.append(
          .init(
            .excessiveBank, "Path curvature exceeds bank envelope", value: abs(bank),
            limit: v.maximumBank))
      }
      if q > v.maximumDynamicPressure {
        violations.append(
          .init(
            .excessiveDynamicPressure, "Path q exceeds envelope", value: q,
            limit: v.maximumDynamicPressure))
      }
      if load > v.maximumLoadFactor {
        violations.append(
          .init(
            .excessiveLoad, "Path normal load exceeds envelope", value: load,
            limit: v.maximumLoadFactor))
      }
      if s.height < v.gearHeight - 0.1 {
        violations.append(.init(.terrainCollision, "Vertical path penetrates runway plane"))
      }
      result.append(
        .init(
          distance: s.distance, position: s.position, tangent: s.tangent, curvature: s.curvature,
          height: s.height, slope: s.slope, verticalCurvature: s.verticalCurvature, speed: speed,
          angleOfAttack: lift.alpha, bank: bank, segment: s.segment))
      if !violations.isEmpty { return (result, finish(speed)) }
      if i + 1 < stations.count {
        let next = stations[i + 1]
        let ds = next.distance - s.distance
        let dt = ds / max(5, vh)
        if i > 1
          && abs(wrapAngle(bank - previousBank)) / max(dt, 0.01)
            > min(v.controlPlant.maximumRollRate, configuration.guidance.bankSlewRate * 0.85)
        {
          violations.append(
            .init(.actuatorAuthority, "Path bank-rate demand exceeds identified response"))
          return (result, finish(speed))
        }
        let coefficient = aero.coefficients(mach: speed / atm.speedOfSound, alpha: lift.alpha)
        let drag = q * v.referenceArea * coefficient.drag * dragMultiplier / initial.mass
        let work = drag * ds / max(0.2, cos(gamma))
        let potential =
          configuration.planet.gravitationalParameter
          * (1 / pos.length - 1
            / (configuration.planet.radius + configuration.runway.elevation + next.height))
        let nextSquared = speed * speed - 2 * (work + potential)
        if nextSquared < 30 * 30 {
          violations.append(
            .init(
              .insufficientEnergy, "Path exhausts kinetic energy", value: nextSquared, limit: 900))
          return (result, finish(0))
        }
        speed = sqrt(nextSquared)
        time += dt
        previousBank = bank
      }
    }
    if speed < v.landingSpeed * 0.82 {
      violations.append(
        .init(
          .insufficientEnergy, "Touchdown speed below validated design envelope", value: speed,
          limit: v.landingSpeed * 0.82))
    }
    if speed > v.maximumTouchdownSpeed {
      violations.append(
        .init(
          .excessiveEnergy, "Touchdown speed/rollout distance too high", value: speed,
          limit: v.maximumTouchdownSpeed))
    }
    let stopDistance = speed * speed / (2 * max(0.1, v.brakeFriction * 9.80665))
    if configuration.guidance.touchdownAlong + stopDistance > configuration.runway.length - 100 {
      violations.append(
        .init(
          .excessiveEnergy, "Insufficient runway braking distance", value: stopDistance,
          limit: configuration.runway.length - configuration.guidance.touchdownAlong - 100))
    }
    return (result, finish(speed))
  }
}
