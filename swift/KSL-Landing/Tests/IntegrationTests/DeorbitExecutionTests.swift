import Foundation
import KSLCore
import KSLControl
import KSLGuidance
import KSLPhysics
@testable import KSLRuntime
import XCTest

private actor FreshnessOnlyIO: OrbitalFlightIO {
  func readState() throws -> VehicleState { throw FlightError.io("not used by freshness test") }
  func apply(_ envelope: CommandEnvelope) throws { throw FlightError.io("not used") }
  func armDeorbit(plan: DeorbitPlan, reducedAssurance: Bool) throws { throw FlightError.io("not used") }
  func applyOrbital(sequence: UInt64, stateTime: Double, command: OrbitalControlCommand) throws { throw FlightError.io("not used") }
  func finishDeorbit() throws { throw FlightError.io("not used") }
  func beginEntry() throws { throw FlightError.io("not used") }
  func shutdown() {}
}
private actor FreshnessOnlyRecorder: TelemetryRecording {
  var records: [FlightRecord] = []
  func record(_ record: FlightRecord) { records.append(record) }
  func close() {}
}
private actor OrbitalGapFixture: OrbitalFlightIO {
  let initial: VehicleState, planet: PlanetConfiguration
  var reads = 0, time = 0.0, recoveries = 0
  var sequences: [UInt64] = []
  init(_ state: VehicleState, planet: PlanetConfiguration) { initial = state; self.planet = planet }
  func readState() throws -> VehicleState {
    reads += 1
    if reads == 3 { throw FlightError.deadlineExceeded }
    if reads > 1 { time += 0.1 }
    let inertial = try OrbitalMechanics(planet: planet).propagate(.init(fixed: initial, planet: planet), to: time)
    let fixed = inertial.planetFixed(planet: planet, attitude: initial.attitude)
    var result = initial; result.time = time; result.position = fixed.position; result.velocity = fixed.velocity
    return result
  }
  func recover() { recoveries += 1; time += 3 }
  func armDeorbit(plan: DeorbitPlan, reducedAssurance: Bool) {}
  func applyOrbital(sequence: UInt64, stateTime: Double, command: OrbitalControlCommand) { sequences.append(sequence) }
  func apply(_ envelope: CommandEnvelope) {}
  func finishDeorbit() {}
  func beginEntry() {}
  func shutdown() {}
}

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
  func testLoadedPlanRequiresFreshOrbitAndCurrentBurnWindow() async throws {
    let original = plan(cases: cases(passes: Set(0..<15)))
    let w = original.window
    let future = DeorbitWindow(burnStart: 100, deltaV: w.deltaV, duration: w.duration,
      normalAngle: w.normalAngle, periapsisAltitude: w.periapsisAltitude, afterBurn: w.afterBurn,
      entryInterface: w.entryInterface, entryRange: w.entryRange, entrySpeed: w.entrySpeed,
      entryFlightPath: w.entryFlightPath, entryCourseError: w.entryCourseError, crossrange: w.crossrange, miss: w.miss)
    let encoded = try JSONEncoder().encode(DeorbitPlan(source: original.source, configuration: original.configuration,
      propulsion: original.propulsion, window: future, cases: original.cases, modelLimitations: [], evaluatedWindows: 1))
    let loaded = try JSONDecoder().decode(DeorbitPlan.self, from: encoded)
    let runtime = DeorbitRuntime(io: FreshnessOnlyIO(), recorder: FreshnessOnlyRecorder(), plan: loaded,
      aerodynamics: try AerodynamicsEngine())
    try await runtime.validateFreshOrbit(loaded.source)
    var wrongMass = loaded.source; wrongMass.mass *= 2
    do { try await runtime.validateFreshOrbit(wrongMass); XCTFail("changed mass accepted") } catch {}
    var late = loaded.source; late.time = future.burnStart
    do { try await runtime.validateFreshOrbit(late); XCTFail("expired burn accepted") } catch {}
    var reversed = loaded.source; reversed.time -= 1
    do { try await runtime.validateFreshOrbit(reversed); XCTFail("pre-plan epoch accepted") } catch {}
  }
  func testOrbitalCoastRecoversFromTimeoutAndForwardGap() async throws {
    let original = plan(cases: cases(passes: Set(0..<15))), w = original.window
    let future = DeorbitWindow(burnStart: 100, deltaV: w.deltaV, duration: w.duration,
      normalAngle: w.normalAngle, periapsisAltitude: w.periapsisAltitude, afterBurn: w.afterBurn,
      entryInterface: w.entryInterface, entryRange: w.entryRange, entrySpeed: w.entrySpeed,
      entryFlightPath: w.entryFlightPath, entryCourseError: w.entryCourseError, crossrange: w.crossrange, miss: w.miss)
    let p = DeorbitPlan(source: original.source, configuration: original.configuration,
      propulsion: original.propulsion, window: future, cases: original.cases, modelLimitations: [], evaluatedWindows: 1)
    let io = OrbitalGapFixture(p.source, planet: p.configuration.planet)
    let recorder = FreshnessOnlyRecorder(), aero = try AerodynamicsEngine()
    let runtime = DeorbitRuntime(io: io, recorder: recorder, plan: p, aerodynamics: aero, maximumOutage: 1)
    let outcome = try await runtime.run(maximumDuration: 3.3, realTime: false)
    XCTAssertEqual(outcome, .timedOut)
    let recoveries = await io.recoveries, sequences = await io.sequences
    XCTAssertEqual(recoveries, 1)
    XCTAssertEqual(sequences, Array(1...sequences.count).map(UInt64.init))
    XCTAssertGreaterThanOrEqual(sequences.count, 2)
    let records = await recorder.records
    XCTAssertTrue(records.contains { $0.orbital?.telemetryGapResumed == true })
    let replay = FlightRecordReplayer(configuration: p.configuration,
      atmosphere: try .init(planet: p.configuration.planet), aerodynamics: aero)
    for record in records {
      XCTAssertEqual(try replay.step(record).orbitalCommand, record.orbitalCommand)
    }
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
  func testBurnGapKeepsMeasuredImpulseSeparateFromUncertaintyBound() throws {
    let p = plan(cases: []), burn = DeorbitBurnController(plan: p)
    var state = source()
    _ = try burn.update(state, periapsisAltitude: 52000)
    state.time = 0.1; state.systems.thrust = 1000
    _ = try burn.update(state, periapsisAltitude: 52000)
    let measured = burn.deliveredDeltaV
    state.time = 2.1
    try burn.resumeAfterTelemetryGap(at: state)
    XCTAssertEqual(burn.deliveredDeltaV, measured, accuracy: 1e-12)
    let bound = state.propulsion!.availableThrust*p.configuration.deorbitSettings.maximumThrottle/state.mass*2
    XCTAssertEqual(burn.unobservedDeltaVUpperBound, bound, accuracy: 1e-12)
    let output = try burn.update(state, periapsisAltitude: 52000)
    XCTAssertGreaterThan(output.command.throttle, 0)
    XCTAssertEqual(burn.diagnostics.unobservedDeltaVUpperBound!, bound, accuracy: 1e-12)
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
