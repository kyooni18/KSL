import Foundation
import KSLCore
import KSLControl
import KSLGuidance
import KSLPhysics
@testable import KSLRuntime
import XCTest

final class DeorbitExecutionTests: XCTestCase {
  private func source() -> VehicleState {
    let p = PlanetConfiguration(), radius = p.radius+100000
    var state = VehicleState(time: 0, position: .init(radius, 0, 0),
      velocity: .init(0, sqrt(p.gravitationalParameter/radius)-p.rotationRate*radius, 0),
      attitude: .init(forward: .init(0, -1, 0), right: .init(0, 0, 1), down: .init(-1, 0, 0)))
    state.controlAuthority = .init(positiveTorque: .init(10000, 10000, 10000),
      negativeTorque: .init(-10000, -10000, -10000), inertia: .init(100000, 100000, 100000))
    state.propulsion = .init(availableThrust: 60000, dryMass: 22000, specificImpulse: 316)
    return state
  }
  private func cases(passes: Set<Int>, hardUnsafe: Set<Int> = []) -> [DeorbitStressCase] {
    (0..<15).map { index in
      .init(index: index, passed: passes.contains(index), hardUnsafe: hardUnsafe.contains(index),
        burnCompleted: true, entryFlightPath: radians(-2), maximumDynamicPressure: 20000,
        maximumLoad: 2, touchdownAlong: 400, touchdownCross: 0, touchdownSink: -1.5,
        touchdownPitch: radians(8), stoppedAlong: passes.contains(index) ? 1400 : nil,
        violations: [], evidence: "authorization-contract-fixture")
    }
  }
  private func plan(cases: [DeorbitStressCase], limitations: [String] = []) -> DeorbitPlan {
    let state = source(), orbital = OrbitalState(fixed: state, planet: .init())
    let window = DeorbitWindow(burnStart: 0, deltaV: 60, duration: 200, normalAngle: 0,
      periapsisAltitude: 50000, afterBurn: orbital, entryInterface: orbital,
      entryRange: 1030000, entrySpeed: 2200, entryFlightPath: radians(-2),
      entryCourseError: 0, crossrange: 0, miss: 0)
    return .init(identifier: "reviewed-plan", source: state, configuration: .init(),
      propulsion: state.propulsion!, window: window, cases: cases,
      modelLimitations: limitations, evaluatedWindows: 1)
  }
  func testFailedModelVerdictsRemainAdvisoryWithExplicitReducedAssurance() throws {
    let zero = plan(cases: cases(passes: []))
    XCTAssertFalse(zero.executionQualified)
    XCTAssertTrue(zero.permitsExplicitReducedAssurance)
    XCTAssertThrowsError(try zero.authorize(identifier: zero.identifier, reducedAssurance: false))
    try zero.authorize(identifier: zero.identifier, reducedAssurance: true)
    let failedNominal = plan(cases: cases(passes: Set(1..<15)))
    XCTAssertFalse(failedNominal.executionQualified)
    try failedNominal.authorize(identifier: failedNominal.identifier, reducedAssurance: true)
  }
  func testExplicitAuthorizationIsBoundToPlanAndCompleteMatrix() throws {
    let provisional = plan(cases: cases(passes: Set(0..<15)), limitations: ["unvalidated moment model"])
    XCTAssertTrue(provisional.fullRouteQualified)
    XCTAssertFalse(provisional.executionQualified)
    XCTAssertThrowsError(try provisional.authorize(identifier: provisional.identifier, reducedAssurance: false))
    XCTAssertThrowsError(try provisional.authorize(identifier: "different-plan", reducedAssurance: true))
    try provisional.authorize(identifier: provisional.identifier, reducedAssurance: true)
    let duplicate = plan(cases: Array(repeating: cases(passes: [0])[0], count: 15))
    XCTAssertFalse(duplicate.hasCompleteStressMatrix)
    XCTAssertThrowsError(try duplicate.authorize(identifier: duplicate.identifier, reducedAssurance: true))
    let danger = plan(cases: cases(passes: Set(0..<15), hardUnsafe: [4]))
    XCTAssertFalse(danger.executionQualified)
    XCTAssertEqual(danger.unsafeCases, 1)
    try danger.authorize(identifier: danger.identifier, reducedAssurance: true)
  }
  func testCPeriapsisTaperAndNoPrematureCutoff() throws {
    XCTAssertEqual(DeorbitBurnController.periapsisFraction(remainingAltitude: 2000), 1)
    XCTAssertEqual(DeorbitBurnController.periapsisFraction(remainingAltitude: 700), 0.3, accuracy: 1e-12)
    let burn = DeorbitBurnController(plan: plan(cases: []))
    var state = source()
    XCTAssertEqual(try burn.update(state, periapsisAltitude: 52000).command.throttle, 0)
    state.time += 0.1; state.systems.thrust = 1000
    XCTAssertGreaterThan(try burn.update(state, periapsisAltitude: 51475).command.throttle, 0)
    XCTAssertFalse(burn.complete)
    state.time += 0.1
    let cutoff = try burn.update(state, periapsisAltitude: 50299)
    XCTAssertTrue(burn.complete)
    XCTAssertEqual(cutoff.phase, .entryCoast)
    XCTAssertEqual(cutoff.command.throttle, 0)
    XCTAssertGreaterThan(burn.deliveredDeltaV, 0)
  }
  func testPointingAndThrustLossCannotSilentlyMoveBurnWindow() throws {
    var late = source(); late.time = 1
    XCTAssertThrowsError(try DeorbitBurnController(plan: plan(cases: [])).update(late, periapsisAltitude: 52000))
    var state = source(); state.attitude = .init()
    let unaligned = DeorbitBurnController(plan: plan(cases: []))
    XCTAssertEqual(try unaligned.update(state, periapsisAltitude: 52000).command.throttle, 0)
    state.time = 0.6
    XCTAssertThrowsError(try unaligned.update(state, periapsisAltitude: 52000))
    let dry = DeorbitBurnController(plan: plan(cases: []))
    state = source()
    var rejected = false
    for step in 0...120 {
      state.time = Double(step)*0.1
      do { _ = try dry.update(state, periapsisAltitude: 52000) }
      catch { rejected = true; break }
    }
    XCTAssertTrue(rejected, "A commanded burn with no measured thrust must time out")
    XCTAssertFalse(dry.complete)
  }
  func testNativeOrbitalPointingStopsWithoutAttitudeTeleportation() throws {
    let controller = OrbitalAttitudeController()
    var state = source()
    let target = Vector3(0, -1, 0)
    state.attitude = state.attitude * .axisAngle(.init(0, 1, 0), angle: radians(25))
    var largestStep = 0.0
    let dt = 0.02, authority = state.controlAuthority!.minimumAcceleration
    for i in 0..<1000 {
      state.time = Double(i)*dt
      let command = try controller.command(state: state, direction: target, dt: dt)
      let acceleration = Vector3(command.roll*authority.x, command.pitch*authority.y, command.yaw*authority.z)
      let midpoint = state.angularVelocity+acceleration*(dt/2)
      largestStep = max(largestStep, midpoint.length*dt)
      state.attitude = state.attitude.advanced(bodyRate: midpoint, dt: dt)
      state.angularVelocity = state.angularVelocity+acceleration*dt
    }
    XCTAssertLessThan(controller.alignmentError, radians(0.5))
    XCTAssertLessThan(state.angularVelocity.length, radians(0.5))
    XCTAssertLessThan(largestStep, radians(1))
  }
  func testFifteenControlledOrbitalDispersionsReachDescendingInterface() throws {
    let c = MissionConfiguration(), state = source(), propulsion = state.propulsion!
    let window = try DeorbitWindowSearch(configuration: c).probe(
      initial: .init(fixed: state, planet: c.planet), start: 60, normal: 0, propulsion: propulsion)
    let dispersions = DeorbitDispersion.cases(c)
    XCTAssertEqual(dispersions.count, 15)
    for (index, dispersion) in dispersions.enumerated() {
      let result = try OrbitalExecutionModel(configuration: c).predict(source: state,
        window: window, propulsion: propulsion, dispersion: dispersion)
      XCTAssertTrue(result.burnCompleted, "orbital dispersion \(index)")
      XCTAssertLessThan(result.entry.verticalSpeed, 0)
      XCTAssertLessThan(result.entry.position.length-c.planet.radius, c.planet.atmosphereTop)
      XCTAssertLessThan(result.entry.mass, state.mass*dispersion.mass)
      XCTAssertGreaterThan(result.entry.mass, propulsion.dryMass)
      XCTAssertTrue(result.entry.systems.unpowered)
    }
  }
}
