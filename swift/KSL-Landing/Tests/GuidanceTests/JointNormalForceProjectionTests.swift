import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class JointNormalForceProjectionTests: XCTestCase {
  func testJointProjectionCanTradeDesiredSupportForTurnWithoutIncreasingVehicleLimits() throws {
    struct Recorded: Decodable { let configuration: MissionConfiguration, state: VehicleState }
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    let r = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    let c = r.configuration, s = r.state, a = s.aerodynamics
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let maximum = min(c.vehicle.maximumLoadFactor*9.80665,
      aero.liftSolution(acceleration: 0, mass: s.mass, dynamicPressure: a.dynamicPressure, mach: a.mach,
        minimumAlpha: c.vehicle.minimumAngleOfAttack, maximumAlpha: c.vehicle.maximumAngleOfAttack).maximumAcceleration)
    XCTAssertGreaterThan(maximum, 0)
    let envelope = NormalForceEnvelope(maximumLift: maximum, maximumBank: c.vehicle.maximumBank)
    // Conflicting requested vector at the modeled force bound. This changes
    // the request, never the real vehicle's aero/authority/physical capability.
    let lateral = maximum*sin(c.vehicle.maximumBank)
    let verticalFirst = envelope.allocate(vertical: maximum, lateral: lateral)
    let joint = try XCTUnwrap(envelope.projectJointly(vertical: maximum, lateral: lateral))
    XCTAssertEqual(verticalFirst.lateral, 0, accuracy: 1e-12)
    XCTAssertGreaterThan(joint.lateral, 0)
    XCTAssertLessThan(joint.vertical, maximum)
    XCTAssertLessThanOrEqual(abs(joint.bank), c.vehicle.maximumBank)
    XCTAssertEqual(hypot(joint.vertical, joint.lateral), maximum, accuracy: 1e-12)
    let oldError = pow(verticalFirst.vertical-maximum, 2)+pow(verticalFirst.lateral-lateral, 2)
    let newError = pow(joint.vertical-maximum, 2)+pow(joint.lateral-lateral, 2)
    XCTAssertLessThan(newError, oldError)
    let reverse = try XCTUnwrap(envelope.projectJointly(vertical: maximum, lateral: -lateral))
    XCTAssertEqual(reverse.vertical, joint.vertical, accuracy: 1e-12)
    XCTAssertEqual(reverse.lateral, -joint.lateral, accuracy: 1e-12)
    XCTAssertNil(envelope.projectJointly(vertical: .nan, lateral: 0))
  }
}
