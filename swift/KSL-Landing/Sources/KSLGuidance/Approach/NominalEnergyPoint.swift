import Foundation
import KSLCore
import KSLPhysics

/// A model-derived Final entry state, not a closed-loop or live certificate.
public struct NominalEnergyPoint: Codable, Sendable {
  public let along: Double
  public let height: Double // metres above the selected runway
  public let flightPathAngle: Double
  public let airspeed: Double
  public let predictedTouchdownSpeed: Double
}

/// Prices the existing outer-glide/flare geometry from the NEP to touchdown.
/// A missing root or any model constraint violation is reported as nil rather
/// than silently replacing the answer with a configured nominal speed.
public struct NominalEnergyPointSolver {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aero: AerodynamicsEngine

  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aero = aerodynamics
  }

  public func solve(from measured: VehicleState, trimAlphaCeiling: Double? = nil) -> NominalEnergyPoint? {
    let c = configuration, profile = ApproachProfile(configuration: c)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let along = -c.guidance.finalDistance
    let reference = profile.reference(along: along)
    let length = c.guidance.touchdownAlong - along
    guard length > 0, reference.height > c.vehicle.gearHeight else { return nil }
    let count = max(2, Int(ceil(length / 40)))
    let stations: [PathStation] = (0...count).map { i in
      let d = length * Double(i) / Double(count), x = along + d
      let r = profile.reference(along: x)
      return .init(distance: d, position: .init(x, 0), tangent: .init(1, 0),
        curvature: 0, height: r.height, slope: r.slope,
        verticalCurvature: r.secondDerivative / pow(1 + r.slope*r.slope, 1.5), segment: .final)
    }
    let position = frame.position(along: along, cross: 0, height: reference.height)
    let air = atmosphere.sample(position: position, time: measured.time)
    guard air.density > 0, air.speedOfSound > 0 else { return nil }
    let gamma = atan(reference.slope)
    let maximumQSpeed = sqrt(2 * c.vehicle.maximumDynamicPressure / air.density)
    let upper = min(maximumQSpeed, c.guidance.finalAlignmentSpeed ?? maximumQSpeed)
    let lower = c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed
    guard upper > lower else { return nil }
    let evaluator = EnergyManagementEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    func price(_ speed: Double) -> TerminalAssessment {
      var seed = measured
      seed.position = position
      let tangent = frame.direction(at: .init(along, 0), tangent: .init(1, 0))
      seed.velocity = tangent * (speed * cos(gamma)) + position.normalized * (speed * sin(gamma)) + air.wind
      seed.atmosphere = air
      seed.aerodynamics.airspeed = speed
      seed.aerodynamics.mach = speed / air.speedOfSound
      seed.aerodynamics.dynamicPressure = 0.5 * air.density * speed * speed
      seed.aerodynamics.bank = 0
      // Landing gear already extended in measured telemetry remains extended
      // in the NEP projection; silently retracting it underprices Final drag.
      seed.systems.gearDown = measured.systems.gearDown
        || reference.height <= (c.guidance.gearDeploymentAltitude ?? 1500)
      let g = c.planet.gravitationalParameter / position.squaredLength
      let required = max(0, (g - speed*speed/position.length)*cos(gamma)
        + speed*speed*stations[0].verticalCurvature)
      seed.aerodynamics.angleOfAttack = aero.liftSolution(acceleration: required,
        mass: seed.mass, dynamicPressure: seed.aerodynamics.dynamicPressure,
        mach: seed.aerodynamics.mach, minimumAlpha: c.vehicle.minimumAngleOfAttack,
        maximumAlpha: min(c.vehicle.maximumAngleOfAttack, trimAlphaCeiling ?? c.vehicle.maximumAngleOfAttack),
        gear: seed.systems.gearDown).alpha
      return evaluator.evaluate(stations: stations, from: seed, advisoryOnly: true,
        trimAlphaCeiling: trimAlphaCeiling).1
    }
    let target = c.vehicle.landingSpeed
    let lowResult = price(lower), highResult = price(upper)
    guard (lowResult.predictionComplete != true || lowResult.touchdownSpeed <= target),
          highResult.predictionComplete == true, highResult.touchdownSpeed >= target else { return nil }
    var lo = lower, hi = upper
    // Fixed iterations give approximately 10^-3 m/s numerical resolution over
    // the physically bounded interval; they are not a flight tolerance.
    for _ in 0..<20 {
      if Task.isCancelled { return nil }
      let mid = (lo + hi) / 2, result = price(mid)
      if result.predictionComplete != true || result.touchdownSpeed < target { lo = mid }
      else { hi = mid }
    }
    let speed = (lo + hi) / 2, result = price(speed)
    guard result.feasible else { return nil }
    return .init(along: along, height: reference.height, flightPathAngle: gamma,
      airspeed: speed, predictedTouchdownSpeed: result.touchdownSpeed)
  }
}
