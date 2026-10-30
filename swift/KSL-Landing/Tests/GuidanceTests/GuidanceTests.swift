import KSLCore
import KSLPhysics
import XCTest

@testable import KSLGuidance

final class GuidanceTests: XCTestCase {
  func testEntryCaptureObjectiveAndPitchResponseShapeTheDecidedAlpha() throws {
    var c = MissionConfiguration()
    c.vehicle.rcsAttitudeControl = true
    let atmosphere = try AtmosphericModelEngine(), aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let p = frame.position(along: -200_000, cross: 0, height: 40_000)
    let velocity = frame.direction(at: .init(-200_000, 0), tangent: .init(1, 0)) * 1500
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let state = physics.observe(.init(time: 100, position: p, velocity: velocity,
      attitude: .windAttitude(position: p, airVelocity: velocity, alpha: c.entrySettings.nominalAlpha, bank: 0)))
    var memory = EntryLawMemory()
    memory.previousAlpha = c.entrySettings.nominalAlpha
    memory.previousTime = state.time - c.guidance.controlInterval
    memory.pitchResponseSeconds = 2
    let fast = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero, memory: memory)
    let fastTarget = fast.update(state)
    memory.pitchResponseSeconds = 8
    let slow = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero, memory: memory)
    let slowTarget = slow.update(state)
    XCTAssertLessThan(abs(slowTarget.angleOfAttack - memory.previousAlpha!),
      abs(fastTarget.angleOfAttack - memory.previousAlpha!))
    memory.objective = .seed(configuration: c, atmosphere: atmosphere, frame: frame, crossAim: 0, time: state.time)
    XCTAssertEqual(hypot(memory.objective!.along, memory.objective!.cross), c.entrySettings.captureRange, accuracy: 1e-6)
    XCTAssertEqual(memory.objective!.altitude, c.entrySettings.terminalTargetAltitude)
    var retargeted = memory
    retargeted.retargetCrossAim(c.entrySettings.handoffRadius)
    XCTAssertNil(retargeted.objective)
    XCTAssertEqual(retargeted.crossAim, c.entrySettings.handoffRadius)
    memory.objective!.along = -150_000
    let near = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero, memory: memory)
    _ = near.update(state)
    XCTAssertLessThan(near.diagnostics.requiredRange, slow.diagnostics.requiredRange)
    memory.previousAlpha = c.vehicle.maximumAngleOfAttack
    memory.identifiedTrimCeiling = c.entrySettings.nominalAlpha
    let rcs = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero, memory: memory)
    XCTAssertGreaterThan(rcs.update(state).angleOfAttack, c.entrySettings.nominalAlpha)
    c.vehicle.rcsAttitudeControl = false
    let aerodynamic = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero, memory: memory)
    XCTAssertLessThanOrEqual(aerodynamic.update(state).angleOfAttack, c.entrySettings.nominalAlpha)
  }

  func testHighCloseCaptureObjectiveProducesAnAdmissibleHAC() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine(), aero = try AerodynamicsEngine()
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let initial = VehicleState(time: 0, position: frame.position(along: -200_000, cross: 0, height: 70_000),
      velocity: .zero, attitude: .init(), mass: c.vehicle.nominalMass)
    let objective = EntryCaptureObjectivePlanner(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      .objective(from: initial, crossAim: 0)
    XCTAssertGreaterThanOrEqual(objective.altitude, c.entrySettings.terminalMinimumAltitude)
    XCTAssertEqual(hypot(objective.along, objective.cross), c.entrySettings.captureRange, accuracy: 1e-6)
    let position = frame.position(along: objective.along, cross: objective.cross,
      height: objective.altitude-c.runway.elevation)
    let forward = frame.origin.normalized.projected(onPlaneWithNormal: position.normalized).normalized
    let gamma = try XCTUnwrap(objective.flightPath)
    let velocity = (forward*cos(gamma)+position.normalized*sin(gamma))*objective.speed
    let sample = physics.observe(.init(time: 0, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: c.entrySettings.nominalAlpha, bank: 0),
      mass: c.vehicle.nominalMass))
    let result = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      .plan(from: sample, allowDirect: false)
    let plan = try XCTUnwrap(result.plan)
    XCTAssertNotNil(plan.spiral)
    XCTAssertTrue(MM305Admission.evaluate(sample, plan: plan, configuration: c).accepted)
  }

  func testSpiralRadiusCurvatureAndMirroring() throws {
    let right = try SpiralHAC(
      exitAlong: -8000, finalRadius: 4000, growth: 250, turnAngle: radians(450), side: 1)
    let left = try SpiralHAC(
      exitAlong: -8000, finalRadius: 4000, growth: 250, turnAngle: radians(450), side: -1)
    XCTAssertGreaterThan(right.radius(at: 2), right.radius(at: 1))
    for t in [0.0, 0.2, 1, 3, 6, 7.5] {
      let a = right.sample(remainingAngle: t)
      let b = left.sample(remainingAngle: t)
      XCTAssertEqual(a.position.x, b.position.x, accuracy: 1e-9)
      XCTAssertEqual(a.position.y, -b.position.y, accuracy: 1e-9)
      XCTAssertEqual(a.curvature, -b.curvature, accuracy: 1e-12)
      let epsilon = 1e-5
      let p0 = right.sample(remainingAngle: t + epsilon)
      let p1 = right.sample(remainingAngle: t - epsilon)
      let angle = atan2(p0.tangent.cross(p1.tangent), p0.tangent.dot(p1.tangent))
      XCTAssertEqual(angle / (p1.position - p0.position).length, a.curvature, accuracy: 1e-8)
    }
    XCTAssertEqual(right.sample(remainingAngle: 0).tangent.x, 1, accuracy: 1e-12)
    XCTAssertNotEqual(
      1 / right.sample(remainingAngle: 1).curvature, right.radius(at: 1), accuracy: 1)
  }
  func testAcquisitionC2Joins() throws {
    let spiral = try SpiralHAC(
      exitAlong: -8000, finalRadius: 5000, growth: 200, turnAngle: 2, side: 1)
    let start = HorizontalPathSample(
      position: .init(-30_000, 10_000), tangent: .init(1, 0), curvature: 0)
    let end = spiral.sample(remainingAngle: 2)
    let curve = QuinticAcquisition(start: start, end: end, tangentLength: 20_000)
    XCTAssertEqual((curve.sample(0).position - start.position).length, 0, accuracy: 1e-8)
    XCTAssertEqual((curve.sample(1).position - end.position).length, 0, accuracy: 1e-7)
    XCTAssertEqual(curve.sample(1).tangent.dot(end.tangent), 1, accuracy: 1e-10)
    XCTAssertEqual(curve.sample(1).curvature, end.curvature, accuracy: 1e-10)
  }
  func testApproachProfileContinuous() {
    let profile = ApproachProfile(configuration: .init())
    for x in [profile.preflareAlong, profile.shallowAlong] {
      let a = profile.reference(along: x - 1e-5)
      let b = profile.reference(along: x + 1e-5)
      XCTAssertEqual(a.height, b.height, accuracy: 1e-4)
      XCTAssertEqual(a.slope, b.slope, accuracy: 1e-7)
      XCTAssertEqual(a.secondDerivative, b.secondDerivative, accuracy: 1e-7)
    }
  }
  func testLowEnergyHACIsRetainedWithAdvisories() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let f = RunwayFrame(runway: c.runway)
    let p = f.position(along: -120_000, cross: 0, height: 4000)
    let vel = f.direction(at: .init(-120_000, 0), tangent: .init(1, 0)) * 100
    let state = physics.observe(
      .init(
        time: 0, position: p, velocity: vel,
        attitude: .windAttitude(position: p, airVelocity: vel, alpha: radians(10), bank: 0)))
    let planner = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let result = planner.plan(from: state)
    let plan = try XCTUnwrap(result.plan)
    XCTAssertFalse(plan.assessment.violations.isEmpty)
    XCTAssertEqual(plan.stations.last?.segment, .final)
    XCTAssertTrue(MM305Admission.evaluate(state, plan: plan, configuration: c).accepted)
    var offBand = state
    offBand.time = plan.createdAt + 4*c.guidance.terminalReplanInterval
    offBand.aerodynamics.dynamicPressure = 0
    let advisory = MM305Admission.evaluate(offBand, plan: plan, configuration: c)
    XCTAssertTrue(advisory.accepted)
    XCTAssertFalse(advisory.violations.isEmpty)
    XCTAssertGreaterThan(result.evaluatedCandidates, 0)
  }
  func testMeasuredApproachCaptureIsC2() {
    let profile = ApproachProfile(configuration: .init())
    let capture = ApproachCapture(
      profile: profile, along: -10_000, height: 4200, slope: tan(radians(-24)))
    let first = capture.reference(along: -10_000)
    XCTAssertEqual(first.height, 4200, accuracy: 1e-9)
    XCTAssertEqual(first.slope, tan(radians(-24)), accuracy: 1e-10)
    XCTAssertEqual(first.secondDerivative, 0, accuracy: 1e-10)
    let x = capture.start + capture.span
    let a = capture.reference(along: x - 1e-5)
    let b = profile.reference(along: x + 1e-5)
    XCTAssertEqual(a.height, b.height, accuracy: 1e-4)
    XCTAssertEqual(a.slope, b.slope, accuracy: 1e-7)
    XCTAssertEqual(a.secondDerivative, b.secondDerivative, accuracy: 1e-7)
  }
  func testAuthorityEstimateConvergesWithoutChangingPolar() throws {
    let model = try AerodynamicsEngine()
    let estimator = AerodynamicAuthorityEstimator(model: model)
    let configuration = MissionConfiguration()
    let physics = ShuttlePhysicsEngine(
      configuration: configuration, atmosphere: try .init(),
      aerodynamics: try .init(liftScale: 0.8, dragScale: 1.2))
    let p = RunwayFrame(runway: configuration.runway).position(
      along: -10000, cross: 0, height: 4000)
    let v = Vector3(100, 30, 0).projected(onPlaneWithNormal: p.normalized).normalized * 150
    var state = physics.observe(
      .init(
        time: 0, position: p, velocity: v,
        attitude: .windAttitude(position: p, airVelocity: v, alpha: radians(8), bank: 0)))
    for i in 0..<150 {
      state.time = Double(i) * 0.1
      estimator.observe(state)
    }
    XCTAssertEqual(estimator.liftEffectiveness, 0.8, accuracy: 0.001)
    XCTAssertEqual(estimator.dragEffectiveness, 1.2, accuracy: 0.001)
    XCTAssertEqual(model.liftScale, 1)
  }

  func testVerticalPulloutUsesNormalRatherThanVerticalAcceleration() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let frame = RunwayFrame(runway: c.runway)
    let position = frame.position(along: -15000, cross: 0, height: 5000)
    let forward = frame.direction(at: .init(-15000, 0), tangent: .init(1, 0))
    let gamma = radians(-30)
    let speed = 200.0
    let pulloutRadius = 4000.0
    let velocity = (forward * cos(gamma) + position.normalized * sin(gamma)) * speed
    let state = physics.observe(
      .init(
        time: 0, position: position, velocity: velocity,
        attitude: .windAttitude(
          position: position, airVelocity: velocity, alpha: radians(8), bank: 0)))
    let station = PathStation(
      distance: 0, position: .init(-15000, 0), tangent: .init(1, 0),
      curvature: 0, height: 5000, slope: tan(gamma), verticalCurvature: 1 / pulloutRadius,
      segment: .acquisition)
    let engine = EnergyManagementEngine(
      configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let (_, assessment) = engine.evaluate(stations: [station], from: state)
    let g = c.planet.gravitationalParameter / position.squaredLength
    let gravityNormal = (g - pow(speed * cos(gamma), 2) / position.length) * cos(gamma)
    XCTAssertEqual(
      assessment.maximumLoad * 9.80665, gravityNormal + speed * speed / pulloutRadius,
      accuracy: 1e-8)
  }

  func testDescendingAcquisitionCurvatureMatchesPlantAcceleration() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let frame = RunwayFrame(runway: c.runway)
    let position = frame.position(along: -24000, cross: 4000, height: 12000)
    let forward = frame.direction(
      at: .init(-24000, 4000),
      tangent: .init(cos(radians(-20)), sin(radians(-20))))
    let velocity = (forward * cos(radians(-24)) + position.normalized * sin(radians(-24))) * 220
    let attitude =
      Quaternion.windAttitude(
        position: position, airVelocity: velocity,
        alpha: radians(8), bank: 0) * .axisAngle(.init(1, 0, 0), angle: radians(40))
    let initial = physics.observe(
      .init(
        time: 1000, position: position, velocity: velocity,
        attitude: attitude))
    let planner = HACPlanningEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let dbg = planner.plan(from: initial, allowDirect: false)
    print("DBG", dbg.envelope.map { "\($0.examined) \($0.completed) \($0.feasible) \($0.violationCounts)" } ?? "nil",
      dbg.bestAdvisory?.violations.map(\.message) ?? [], dbg.closedLoopAdvisories?.flatMap { $0.violations.map(\.message) } ?? [])
    if let pv = planner.plan(from: initial, allowDirect: false, preview: true).plan {
      var last = -1.0
      let r = TerminalTrajectoryEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero).evaluate(
        plan: pv, from: initial, controlSnapshot: nil, previousTarget: nil, lift: 1, drag: 1) { st, ref, tg in
          if st.time - last >= 2 { last = st.time; let q = frame.coordinates(of: st.position)
            print(String(format: "TR t=%.0f h=%.0f href=%.0f V=%.0f Vref=%.0f a=%.1f at=%.1f b=%.1f bt=%.1f seg=%@ ce=%.0f", st.time, q.height, ref.height, st.aerodynamics.airspeed, ref.speed,
              degrees(st.aerodynamics.angleOfAttack), degrees(tg.angleOfAttack), degrees(st.aerodynamics.bank), degrees(tg.bank), "\(ref.segment)", q.cross)) } }
      print("TRR", r.feasible, r.violations.map(\.message), r.touchdownSpeed ?? -1, r.touchdownSink ?? -1)
    }
    let plan = try XCTUnwrap(dbg.plan)
    let dt = 0.002
    let next = try physics.advance(initial, command: .init(), duration: dt, maximumStep: 0.001)
    let right = velocity.projected(onPlaneWithNormal: position.normalized).normalized
      .cross(position.normalized).normalized
    let measuredCurvature =
      (next.velocity - velocity).dot(right) / dt
      / (initial.horizontalSpeed * initial.horizontalSpeed)
    XCTAssertEqual(plan.stations[0].curvature, measuredCurvature, accuracy: 1e-7)
  }
}
