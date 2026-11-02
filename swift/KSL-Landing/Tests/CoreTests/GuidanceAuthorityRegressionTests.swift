import XCTest
@testable import KSLCore

final class GuidanceAuthorityRegressionTests: XCTestCase {
  private var authority: ControlAuthorityTelemetry {
    .init(positiveTorque: .init(10_000, 20_000, 20_000),
      negativeTorque: .init(-10_000, -20_000, -20_000),
      inertia: .init(80_000, 900_000, 950_000))
  }

  func testActiveAttitudeAuthorityRemainsContinuousAtAtmosphereBoundary() {
    let c = MissionConfiguration()
    let request = GuidanceTarget(angleOfAttack: radians(18), bank: radians(20))
    var vacuum = GuidanceTargetLimiter(previous: .init(angleOfAttack: 0, bank: 0))
    var atmosphere = vacuum
    let a = vacuum.update(request, phase: .entry, configuration: c, dt: 0.1,
      dynamicPressure: 0, controlAuthority: authority)
    let b = atmosphere.update(request, phase: .entry, configuration: c, dt: 0.1,
      dynamicPressure: 0.02, controlAuthority: authority)
    XCTAssertGreaterThan(b.angleOfAttack, 0)
    XCTAssertEqual(a.angleOfAttack, b.angleOfAttack, accuracy: 1e-12)
    XCTAssertEqual(a.bank, b.bank, accuracy: 1e-12)
    XCTAssertLessThanOrEqual(b.angleOfAttack, c.guidance.alphaSlewRate * 0.1)
    XCTAssertLessThanOrEqual(abs(b.bank), c.vehicle.maximumBank)
  }

  func testMeasuredZeroAuthorityDoesNotBecomeAnUnlimitedSlew() {
    var zero = authority
    zero.positiveTorque = .zero; zero.negativeTorque = .zero
    var limiter = GuidanceTargetLimiter(previous: .init(angleOfAttack: 0, bank: 0))
    let target = limiter.update(.init(angleOfAttack: radians(18), bank: radians(20)),
      phase: .entry, configuration: .init(), dt: 0.1, dynamicPressure: 10_000,
      controlAuthority: zero)
    XCTAssertEqual(target.angleOfAttack, 0)
    XCTAssertEqual(target.bank, 0)
  }

  func testBrakingAReferenceCannotOvershootOrMoveAwayFromItsGoal() {
    let forward = GuidanceTargetLimiter.step(0, toward: 0.001, rate: 1,
      rateLimit: 1, acceleration: 0.001, dt: 0.1)
    XCTAssertGreaterThanOrEqual(forward.0, 0)
    XCTAssertLessThanOrEqual(forward.0, 0.001)
    let reversed = GuidanceTargetLimiter.step(0, toward: -0.001, rate: 1,
      rateLimit: 1, acceleration: 0.001, dt: 0.1)
    XCTAssertLessThanOrEqual(reversed.0, 0)
    XCTAssertGreaterThanOrEqual(reversed.0, -0.001)
  }
  func testPredictionCopyPreservesSlewMomentumWithoutOvershoot() throws {
    let c = MissionConfiguration()
    let request = GuidanceTarget(angleOfAttack: radians(18), bank: radians(20))
    var live = GuidanceTargetLimiter(previous: .init())
    for _ in 0..<4 {
      _ = live.update(request, phase: .acquisition, configuration: c, dt: 0.1,
        dynamicPressure: 3000, controlAuthority: authority)
    }
    var branch = try JSONDecoder().decode(GuidanceTargetLimiter.self, from: JSONEncoder().encode(live))
    var cold = GuidanceTargetLimiter(previous: live.previous)
    let expected = live.update(request, phase: .acquisition, configuration: c, dt: 0.1,
      dynamicPressure: 3000, controlAuthority: authority)
    let copied = branch.update(request, phase: .acquisition, configuration: c, dt: 0.1,
      dynamicPressure: 3000, controlAuthority: authority)
    let reset = cold.update(request, phase: .acquisition, configuration: c, dt: 0.1,
      dynamicPressure: 3000, controlAuthority: authority)
    XCTAssertEqual(copied, expected)
    XCTAssertNotEqual(copied.bank, reset.bank, "Last target alone loses the reference rates")
    XCTAssertLessThanOrEqual(copied.bank, request.bank)
    XCTAssertLessThanOrEqual(copied.angleOfAttack, request.angleOfAttack)
  }

  func testRequestedTargetsStayWithinTheConfiguredAttitudeEnvelope() {
    let c = MissionConfiguration()
    let request = GuidanceTarget(angleOfAttack: c.vehicle.maximumAngleOfAttack+radians(1),
      bank: c.vehicle.maximumBank+radians(1))
    var fresh = GuidanceTargetLimiter()
    let bounded = fresh.update(request, phase: .acquisition, configuration: c, dt: c.guidance.controlInterval)
    XCTAssertEqual(bounded.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    XCTAssertEqual(bounded.bank, c.vehicle.maximumBank)
    var moving = GuidanceTargetLimiter(previous: .init())
    let slewed = moving.update(request, phase: .acquisition, configuration: c, dt: c.guidance.controlInterval,
      dynamicPressure: 3000, controlAuthority: authority)
    XCTAssertGreaterThan(slewed.angleOfAttack, 0)
    XCTAssertLessThanOrEqual(slewed.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    XCTAssertLessThanOrEqual(abs(slewed.bank), c.vehicle.maximumBank)
    var freshLower = GuidanceTargetLimiter()
    let lower = freshLower.update(.init(angleOfAttack: c.vehicle.minimumAngleOfAttack-radians(1)),
      phase: .acquisition, configuration: c, dt: c.guidance.controlInterval)
    XCTAssertEqual(lower.angleOfAttack, c.vehicle.minimumAngleOfAttack)
  }

}
