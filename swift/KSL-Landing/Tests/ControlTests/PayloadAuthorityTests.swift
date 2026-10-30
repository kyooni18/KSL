import KSLCore
import XCTest
@testable import KSLControl

final class PayloadAuthorityTests: XCTestCase {
  func testMeasuredZeroCannotReviveHighPressureIdentification() {
    var axis = AxisAuthority()
    axis.reported = 0.5; axis.controlled = 0.8; axis.confidence = 1; axis.liveConfidence = 1
    XCTAssertGreaterThan(axis.available(fallback: 0.4, blendLive: true, reportedAvailable: true), 0)
    axis.reported = 0
    XCTAssertEqual(axis.available(fallback: 0.4, blendLive: true, reportedAvailable: true), 0)
  }
  func testSteadyOffTargetEntryResponseCanIdentifyItsLocalTrim() throws {
    let c = MissionConfiguration(), position = Vector3(635000,0,0), velocity = Vector3(0,1300,0)
    var a = AerodynamicState(); a.airspeed = 1300; a.mach = 4; a.dynamicPressure = 3000
    a.angleOfAttack = radians(12)
    var s = VehicleState(time: 100, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: a.angleOfAttack, bank: 0),
      aerodynamics: a, controlAuthority: .init(positiveTorque: .init(200000,200000,100000),
        negativeTorque: .init(-200000,-200000,-100000), inertia: .init(400000,400000,300000)),
      appliedControls: .init(pitch: 0.4))
    let controller = FlightController(configuration: c)
    let goal = GuidanceSolution(phase: .entry, target: .init(angleOfAttack: radians(25)))
    _ = try controller.command(state: s, solution: goal, dt: 0.1)
    s.time += 0.1
    _ = try controller.command(state: s, solution: goal, dt: 0.1)
    XCTAssertEqual(controller.diagnostics.trimEvidence, .identified)
    XCTAssertEqual(try XCTUnwrap(controller.diagnostics.pitchFeedforward),0.4,accuracy:1e-8)
    XCTAssertGreaterThan(abs(controller.diagnostics.pitchError),radians(10))
  }
  func testChangedPayloadUsesCurrentReportedInertia() throws {
    let c = MissionConfiguration(), position = Vector3(610000, 0, 0), velocity = Vector3(0, 1000, 0)
    var a = AerodynamicState(); a.airspeed = 1000; a.mach = 3; a.dynamicPressure = 2000
    var s = VehicleState(time: 100, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: 0, bank: 0), aerodynamics: a)
    s.controlAuthority = .init(positiveTorque: .init(200000,200000,100000),
      negativeTorque: .init(-200000,-200000,-100000), inertia: .init(400000,400000,300000))
    let controller = FlightController(configuration: c)
    let target = GuidanceSolution(phase: .entry, target: .init(angleOfAttack: radians(3)))
    _ = try controller.command(state: s, solution: target, dt: 0.1)
    XCTAssertEqual(controller.diagnostics.authority.y, 0.5, accuracy: 1e-12)
    s.time += 0.1; s.controlAuthority!.inertia.y *= 2
    controller.invalidateDynamics(at: s)
    _ = try controller.command(state: s, solution: target, dt: 0.1)
    XCTAssertEqual(controller.diagnostics.authority.y, 0.25, accuracy: 1e-12)
    XCTAssertLessThanOrEqual(controller.diagnostics.authorityConfidence.y, 0.01)
  }
}
