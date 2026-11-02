import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance
final class DessertInvestigationProbe: XCTestCase {
  func testRecordedNEPDiagnostic() throws {
    let root = URL(fileURLWithPath: "/Users/kyooni18/Code/KSL")
    let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: root.appendingPathComponent("runs/native-swift-B090715F-7F7C-4F5E-AE19-05E4FC473E85/configuration.json")))
    let s = try JSONDecoder().decode(VehicleState.self, from: Data(contentsOf: root.appendingPathComponent("Validation/Dessert-TAEM-20261102/handoff-state.json")))
    let atm = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    print("SOLVED", DessertDiagnosticSolver(configuration:c, atmosphere:atm, aerodynamics:aero).solve(from:s) as Any)
  }
}
public struct DessertDiagnosticSolver {
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
    print("ENDPOINTS", "height", reference.height, "lower", lower, lowResult, "upper", upper, highResult)
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
    print("ROOT", speed, result)
    guard result.feasible else { return nil }
    return .init(along: along, height: reference.height, flightPathAngle: gamma,
      airspeed: speed, predictedTouchdownSpeed: result.touchdownSpeed)
  }
}
