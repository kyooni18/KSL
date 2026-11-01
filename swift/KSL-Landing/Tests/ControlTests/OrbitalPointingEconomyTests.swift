import XCTest
import KSLCore
import KSLControl

final class OrbitalPointingEconomyTests: XCTestCase {
  func testTinyAlignmentErrorIsRateDampedInsteadOfFullTorquePulse() {
    let u = OrbitalAttitudeController.boundedDemand(
      error: 0.001, relativeRate: 0, authority: 0.02, hold: 0.25)
    XCTAssertGreaterThan(u, 0)
    XCTAssertLessThan(u, 0.3)
    let braking = OrbitalAttitudeController.boundedDemand(
      error: 0.001, relativeRate: 0.01, authority: 0.02, hold: 0.25)
    XCTAssertLessThan(braking, 0)
    XCTAssertGreaterThan(braking, -1)
    XCTAssertEqual(OrbitalAttitudeController.boundedDemand(
      error: 0, relativeRate: 0, authority: 0.02, hold: 0.25), 0)
  }

  func testCantedOMSAxisIsCompensatedBeforeBurnAlignment() {
    let position = Vector3(680_000, 0, 0)
    let desired = Vector3(0, -1, 0)
    // Actual KSP measurement from the symmetric STS OMS Terrier mounts.
    let axis = Vector3(0.925415, 0, 0.342025).normalized
    let nose = OrbitalAttitudeController.compensatedNoseDirection(
      desiredThrust: desired, position: position, bodyThrustDirection: axis)
    var roof = position.normalized.projected(onPlaneWithNormal: nose).normalized
    if roof.length < 0.5 { roof = Vector3(0, 0, 1) }
    let attitude = Quaternion(forward: nose, right: nose.cross(roof).normalized, down: -roof)
    let achieved = attitude.rotate(axis)
    XCTAssertGreaterThan(achieved.dot(desired), 0.9999)
    XCTAssertGreaterThan(degrees(acos(clamp(nose.dot(desired), -1, 1))), 18)
  }

  func testUncantedThrustRetainsNoseAlignment() {
    let position = Vector3(680_000, 0, 0)
    let desired = Vector3(0, -1, 0)
    let nose = OrbitalAttitudeController.compensatedNoseDirection(
      desiredThrust: desired, position: position, bodyThrustDirection: .init(1, 0, 0))
    XCTAssertGreaterThan(nose.dot(desired), 0.999999)
  }

  private func sample(sources: String?) -> VehicleState {
    let c = MissionConfiguration()
    let position = Vector3(c.planet.radius+90_000, 0, 0)
    var state = VehicleState(time: 10, position: position,
      velocity: .init(0, 2200, 0), attitude: .init(), mass: 25_000)
    var authority = ControlAuthorityTelemetry(
      positiveTorque: .init(10_000, 10_000, 10_000),
      negativeTorque: .init(-10_000, -10_000, -10_000),
      inertia: .init(83_000, 900_000, 940_000), evidence: .measured)
    authority.actuatorSources = sources
    state.controlAuthority = authority
    return state
  }

  func testPoweredReactionWheelsSuppressRCSInOrbitalCoast() throws {
    let controller = OrbitalAttitudeController()
    let state = sample(sources: "aerodynamic-surfaces+active-powered-reaction-wheels+rcs")
    let command = try controller.command(state: state, direction: .init(0, -1, 0), dt: 0.1)
    XCTAssertFalse(command.rcs)
    XCTAssertTrue([command.pitch, command.roll, command.yaw].allSatisfy { abs($0) <= 1 })
  }

  func testWithoutPoweredReactionWheelsRetainsRCSAvailability() throws {
    let controller = OrbitalAttitudeController()
    let state = sample(sources: "aerodynamic-surfaces+rcs")
    let command = try controller.command(state: state, direction: .init(0, -1, 0), dt: 0.1)
    XCTAssertTrue(command.rcs)
  }
}
