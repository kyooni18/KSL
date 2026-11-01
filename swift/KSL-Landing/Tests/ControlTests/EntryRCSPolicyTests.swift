import KSLCore
import KSLPhysics
import XCTest

@testable import KSLControl

final class EntryRCSPolicyTests: XCTestCase {
  func testConfiguredEntryRCSIsReleasedForTerminalGuidance() throws {
    var configuration = MissionConfiguration()
    let position = Vector3(610_000, 0, 0), velocity = Vector3(0, 200, 0)
    let plant = ShuttlePhysicsEngine(
      configuration: configuration, atmosphere: try .init(), aerodynamics: try .init())
    let seed = plant.observe(.init(time: 0, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: radians(3), bank: 0)))
    for enabled in [true, false] {
      configuration.vehicle.rcsAttitudeControl = enabled
      let controller = FlightController(configuration: configuration)
      var state = seed
      for phase in [MissionPhase.entry, .acquisition, .headingAlignment, .approach, .preflare, .flare] {
        state.time += configuration.guidance.controlInterval
        let command = try controller.command(state: state,
          solution: .init(phase: phase, target: .init(angleOfAttack: radians(10), bank: 0)),
          dt: configuration.guidance.controlInterval)
        XCTAssertEqual(command.rcs, enabled && phase == .entry, "\(enabled), \(phase)")
        state.systems.rcs = command.rcs
      }
    }
  }

  func testRCSSettingDoesNotOverrideAerodynamicTrimEvidence() throws {
    var configuration = MissionConfiguration()
    let plant = ShuttlePhysicsEngine(
      configuration: configuration, atmosphere: try .init(), aerodynamics: try .init())
    let position = Vector3(610_000, 0, 0), velocity = Vector3(0, 200, 0)
    let state = plant.observe(.init(time: 0, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: radians(3), bank: 0)))
    let rotation = RotationalDynamicsEngine(configuration: configuration.vehicle)
    let expected = rotation.maximumTrimAlpha(dynamicPressure: state.aerodynamics.dynamicPressure)
    XCTAssertLessThan(expected, configuration.vehicle.maximumAngleOfAttack)
    XCTAssertEqual(rotation.supportedTrimAlpha(state), expected)
    configuration.vehicle.rcsAttitudeControl = false
    XCTAssertEqual(RotationalDynamicsEngine(configuration: configuration.vehicle).supportedTrimAlpha(state), expected)
    XCTAssertEqual(rotation.supportedTrimAlpha(state, identifiedCeiling: radians(18)), radians(18))
  }
}
