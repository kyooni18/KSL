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
    let base = GuidanceTarget(desiredSpeed: a.airspeed, speedbrakeFraction: 0.4)
    let fitted = try XCTUnwrap(TAEMNormalForceFitter.fit(vertical: maximum, lateral: lateral,
      state: s, configuration: c, aerodynamics: aero, maximumAlpha: c.vehicle.maximumAngleOfAttack,
      baseTarget: base))
    XCTAssertEqual(fitted.modelID, aero.instanceID)
    XCTAssertEqual(fitted.stateTime, s.time)
    XCTAssertEqual(fitted.maximumAlpha, c.vehicle.maximumAngleOfAttack)
    XCTAssertGreaterThanOrEqual(fitted.target.angleOfAttack, c.vehicle.minimumAngleOfAttack)
    XCTAssertLessThanOrEqual(fitted.target.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    XCTAssertLessThanOrEqual(abs(fitted.target.bank), c.vehicle.maximumBank)
    XCTAssertEqual(fitted.target.speedbrakeFraction, base.speedbrakeFraction)
    XCTAssertGreaterThan(fitted.evaluatedIncidences, 0)
    XCTAssertLessThanOrEqual(fitted.modeled.length, c.vehicle.maximumLoadFactor*9.80665)
    // Independently forward-query the selected polar; the fit may never claim
    // that its relaxed projection is produced just because inversion returned.
    let selected = aero.coefficients(mach: a.mach, alpha: fitted.target.angleOfAttack,
      dynamicPressure: a.dynamicPressure, beta: a.sideslip, gear: s.systems.gearDown,
      brakes: s.systems.brakes, airbrakes: s.systems.airbrakes,
      speedbrakeFraction: s.systems.speedbrakeFraction ?? 0)
    let factor = a.dynamicPressure*aero.referenceArea/s.mass
    XCTAssertEqual(fitted.modeled.length, selected.lift*factor, accuracy: 1e-12)
    XCTAssertEqual(fitted.modeledDragAcceleration, selected.drag*factor, accuracy: 1e-12)
    XCTAssertEqual(fitted.residual.x, fitted.modeled.x-maximum, accuracy: 1e-12)
    XCTAssertEqual(fitted.residual.y, fitted.modeled.y-lateral, accuracy: 1e-12)
    let decoded = try JSONDecoder().decode(TAEMNormalForceFit.self, from: JSONEncoder().encode(fitted))
    XCTAssertEqual(decoded.target, fitted.target)
    let family = try TAEMTopologySeeds.generate(configuration: c, baseTarget: fitted.target,
      parameters: [.init(alpha: fitted.target.angleOfAttack, bankMagnitude: abs(fitted.target.bank),
        courseWork: .pi/2, maximumSegmentDuration: c.guidance.pathLookaheadTime)],
      incumbent: [], protection: [], id: "force-fit-constructor-check")
    let right = try XCTUnwrap(family.candidates.first { $0.topology == .right })
    XCTAssertEqual(right.segments[0].command.target, fitted.target)
    XCTAssertEqual(right.segments[0].command.phase, .acquisition)
  }
}
