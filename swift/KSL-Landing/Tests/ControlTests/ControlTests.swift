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
    XCTAssertTrue(command.rcs)
    var acquired = state; acquired.time += c.guidance.controlInterval
    let terminal = try controller.command(state: acquired,
      solution: .init(phase: .acquisition, target: .init(angleOfAttack: radians(10), bank: radians(20))),
      dt: c.guidance.controlInterval)
    XCTAssertTrue(terminal.rcs)
    XCTAssertFalse(command.sas)
    XCTAssertFalse(command.airbrakes)
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
