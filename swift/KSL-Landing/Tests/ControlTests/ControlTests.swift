import KSLCore
import KSLPhysics
import XCTest

@testable import KSLControl

final class ControlTests: XCTestCase {
  func testPositivePitchAndRightBankCommands() throws {
    var c = MissionConfiguration(); c.vehicle.rcsAttitudeControl = true
    let plant = ShuttlePhysicsEngine(
      configuration: c, atmosphere: try .init(), aerodynamics: try .init())
    let p = Vector3(610_000, 0, 0)
    let v = Vector3(0, 200, 0)
    let state = plant.observe(
      .init(
        time: 0, position: p, velocity: v,
        attitude: .windAttitude(position: p, airVelocity: v, alpha: radians(3), bank: 0)))
    let controller = FlightController(configuration: c)
    let command = try controller.command(
      state: state,
      solution: .init(phase: .entry, target: .init(angleOfAttack: radians(10), bank: radians(20))),
      dt: 0.1)
    XCTAssertGreaterThan(command.pitch, 0)
    XCTAssertGreaterThan(command.roll, 0)
    XCTAssertEqual(command.throttle, 0)
    XCTAssertTrue(command.rcs)  // configured attitude RCS is authorized during entry
    var acquired = state; acquired.time += c.guidance.controlInterval
    let terminal = try controller.command(state: acquired,
      solution: .init(phase: .acquisition, target: .init(angleOfAttack: radians(10), bank: radians(20))),
      dt: c.guidance.controlInterval)
    XCTAssertTrue(terminal.rcs)  // TAEM keeps RCS until measured surface authority suffices
    XCTAssertFalse(command.sas)
    XCTAssertFalse(command.airbrakes)
  }
  func testLocalTrimIsNotAnIdentifiedAngleOfAttackCeiling() {
    var trim = AdaptiveTrim()
    for i in 0...40 {
      trim.observe(time: Double(i) * 0.2, mach: 7, pressure: 100,
        alpha: radians(18), input: 0.03, settled: true)
    }
    // Enough local observations for a usable feedforward, but no measured
    // AoA variation or slope to justify an absolute 18-degree limit.
    XCTAssertNotNil(trim.input(alpha: radians(18)))
    XCTAssertNil(trim.fit)
    XCTAssertNil(trim.ceiling(maximum: radians(40)))
  }

  func testIdentifiedTrimRejectsExtrapolatedAndStaleFeedforward() {
    var trim = AdaptiveTrim()
    for i in 0..<35 {
      let index = Double(i % 5)
      trim.observe(time: Double(i) * 0.2, mach: 4, pressure: 4000,
        alpha: radians(18 + index), input: 0.20 + 0.02 * index, settled: true)
    }
    XCTAssertNotNil(trim.fit)
    // Exactly the observed sample time; 34 * 0.2 need not bit-equal 6.8.
    let freshTime = Double(34) * 0.2
    XCTAssertNotNil(trim.input(time: freshTime, mach: 4, pressure: 4000, alpha: radians(20)))
    XCTAssertNil(trim.input(time: freshTime, mach: 4, pressure: 4000, alpha: radians(30)))
    XCTAssertNil(trim.input(time: freshTime, mach: 5, pressure: 4000, alpha: radians(20)))
    XCTAssertNil(trim.input(time: freshTime, mach: 4, pressure: 1000, alpha: radians(20)))
    XCTAssertNil(trim.input(time: 9, mach: 4, pressure: 4000, alpha: radians(20)))
    // Saturated observations keep local predictor evidence, not the old
    // unsaturated feedforward alive past its observation time.
    for i in 0..<12 {
      trim.observe(time: 7 + Double(i) * 0.2, mach: 4, pressure: 4000,
        alpha: radians(24), input: 1, settled: true)
    }
    XCTAssertNil(trim.input(time: 9.2, mach: 4, pressure: 4000, alpha: radians(20)))
    XCTAssertNotNil(trim.predictionInput(time: 9.2, mach: 4, pressure: 4000, alpha: radians(24)))
  }
  func testControllerOwnershipReleases() {
    weak var weakController: FlightController?
    do {
      let owner = FlightController(configuration: .init())
      weakController = owner
      XCTAssertNotNil(weakController)
    }
    XCTAssertNil(weakController)
  }
}
