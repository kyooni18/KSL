import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class EntryAerodynamicCalibrationTests: XCTestCase {
  func testMeasuredForcesCalibrateBestGlidePolarInsteadOfStaticEntryEstimates() throws {
    let c = MissionConfiguration(), model = try AerodynamicsEngine()
    var state = VehicleState(time: 100, position: .init(650_000, 0, 0),
      velocity: .init(-200, 2100, 0), attitude: .init(), mass: 24_300)
    state.aerodynamics.measuredForces = true
    state.aerodynamics.dynamicPressure = 1700
    state.aerodynamics.airspeed = 2100
    state.aerodynamics.mach = 5.8
    state.aerodynamics.angleOfAttack = radians(24)
    let now = model.coefficients(mach: state.aerodynamics.mach,
      alpha: state.aerodynamics.angleOfAttack, dynamicPressure: 1700)
    state.aerodynamics.drag = 1700 * c.vehicle.referenceArea * now.drag * 0.75
    state.aerodynamics.lift = 1700 * c.vehicle.referenceArea * now.lift * 1.15
    let observed = EntryAerodynamicCalibration.evaluate(state,
      configuration: c, model: model, maximumAlpha: radians(30))
    let bestAlpha = model.bestGlideAlpha(mach: 5.8,
      maximumAlpha: radians(30), dynamicPressure: 1700)
    let best = model.coefficients(mach: 5.8, alpha: bestAlpha,
      dynamicPressure: 1700)
    XCTAssertEqual(observed.evidence, .measured)
    XCTAssertEqual(observed.ballisticCoefficient,
      state.mass / (c.vehicle.referenceArea * best.drag * 0.75), accuracy: 1e-6)
    XCTAssertEqual(observed.liftToDrag, best.lift * 1.15 / (best.drag * 0.75),
      accuracy: 1e-6)
  }

  func testVacuumOrNoMeasuredForceUsesExplicitProvisionalFallback() throws {
    let c = MissionConfiguration(), model = try AerodynamicsEngine()
    var state = VehicleState(time: 100, position: .init(670_000, 0, 0),
      velocity: .init(-100, 2100, 0), attitude: .init(), mass: 24_300)
    state.aerodynamics.measuredForces = true
    state.aerodynamics.dynamicPressure = 0
    let vacuum = EntryAerodynamicCalibration.evaluate(state,
      configuration: c, model: model, maximumAlpha: radians(30))
    XCTAssertEqual(vacuum.evidence, .provisional)
    XCTAssertEqual(vacuum.ballisticCoefficient, c.entrySettings.estimatedBallisticCoefficient)
    XCTAssertEqual(vacuum.liftToDrag, c.entrySettings.estimatedLiftToDrag)
    state.aerodynamics.dynamicPressure = 1700
    state.aerodynamics.drag = 200
    state.aerodynamics.lift = 1000
    state.aerodynamics.measuredForces = false
    XCTAssertEqual(EntryAerodynamicCalibration.evaluate(state,
      configuration: c, model: model, maximumAlpha: radians(30)).evidence, .provisional)
  }

  func testSnapshotMassOverridesAnyOldVehicleMassAssumption() throws {
    let c = MissionConfiguration(), model = try AerodynamicsEngine()
    var state = VehicleState(time: 100, position: .init(650_000, 0, 0),
      velocity: .init(-200, 2100, 0), attitude: .init(), mass: 25_000)
    state.aerodynamics.measuredForces = true
    state.aerodynamics.dynamicPressure = 1500
    state.aerodynamics.airspeed = 2100
    state.aerodynamics.mach = 6
    state.aerodynamics.angleOfAttack = radians(25)
    let now = model.coefficients(mach: 6, alpha: radians(25), dynamicPressure: 1500)
    state.aerodynamics.lift = 1500 * c.vehicle.referenceArea * now.lift
    state.aerodynamics.drag = 1500 * c.vehicle.referenceArea * now.drag
    let first = EntryAerodynamicCalibration.evaluate(state,
      configuration: c, model: model, maximumAlpha: radians(30))
    state.mass = 20_000
    let second = EntryAerodynamicCalibration.evaluate(state,
      configuration: c, model: model, maximumAlpha: radians(30))
    XCTAssertEqual(first.ballisticCoefficient / second.ballisticCoefficient, 1.25, accuracy: 1e-10)
    XCTAssertEqual(first.liftToDrag, second.liftToDrag, accuracy: 1e-10)
  }
}
