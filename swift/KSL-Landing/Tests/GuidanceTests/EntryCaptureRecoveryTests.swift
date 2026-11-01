import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class EntryCaptureRecoveryTests: XCTestCase {
  func testHandoffTelemetryDistinguishesMeasuredRunwayMissFromObjectiveAndForecast() throws {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let p = frame.position(along: -85_000, cross: 57_000, height: 21_000)
    let velocity = frame.direction(at: .init(-85_000, 57_000),
      tangent: .init(1, 0)) * 470-p.normalized * 70
    let state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero).observe(.init(time: 100, position: p, velocity: velocity,
        attitude: .windAttitude(position: p, airVelocity: velocity,
          alpha: radians(22), bank: 0)))
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true)
    let record = try mission.update(state)
    let capture = try XCTUnwrap(record.capture)
    XCTAssertEqual(try XCTUnwrap(capture.actualAlong), -85_000, accuracy: 0.01)
    XCTAssertEqual(try XCTUnwrap(capture.actualCross), 57_000, accuracy: 0.01)
    XCTAssertEqual(try XCTUnwrap(capture.actualRunwayRange),
      frame.distance(from: p, to: frame.origin), accuracy: 0.01)
    XCTAssertGreaterThan(try XCTUnwrap(capture.actualRunwayRange), 100_000)
    XCTAssertNil(capture.predictedCross)
  }
  func testTerminalCrossrangeAimComesFromPredictedInletAndIsBounded() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: -160_000, cross: 90_000, height: 36_000)
    let state = VehicleState(time: 10, position: position, velocity: .zero, attitude: .init())
    let forecaster = EntryCaptureForecaster(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero)
    let height = c.entrySettings.terminalTargetAltitude-c.runway.elevation
    let good = forecaster.lateralAims(from: state, track: [
      [11, -100_000, 75_000, 40_000],
      [12, -60_000, 35_000, height],
      [13, -50_000, 30_000, 14_000],
    ])
    XCTAssertEqual(good, [0, 35_000])
    let clipped = forecaster.lateralAims(from: state, track: [
      [12, -60_000, -90_000, height],
    ])
    XCTAssertEqual(clipped, [0, -c.guidance.maximumHACRadius])
    XCTAssertEqual(forecaster.lateralAims(from: state, track: []), [0])
  }
  func testMissedHighInletDoesNotRequestClimbFromBelow() {
    let corrected = EntryCommandLaw.descendingCaptureGamma(
      proposed: radians(2.0), actual: radians(-10.4),
      currentAltitude: 21_000, targetAltitude: 25_000, verticalSpeed: -80)
    XCTAssertEqual(corrected, radians(-10.4), accuracy: 1e-12)
    let above = EntryCommandLaw.descendingCaptureGamma(
      proposed: radians(-2), actual: radians(-4),
      currentAltitude: 28_000, targetAltitude: 25_000, verticalSpeed: -80)
    XCTAssertEqual(above, radians(-2), accuracy: 1e-12)
    let alreadySteep = EntryCommandLaw.descendingCaptureGamma(
      proposed: radians(-12), actual: radians(-10),
      currentAltitude: 21_000, targetAltitude: 25_000, verticalSpeed: -80)
    XCTAssertEqual(alreadySteep, radians(-12), accuracy: 1e-12)
  }

  func testHACAdvisoryCannotTakeOverMM304RangeReference() throws {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let atmosphere = try AtmosphericModelEngine()
    let aerodynamics = try AerodynamicsEngine()
    let p = frame.position(along: -75_000, cross: 8_000, height: 24_000)
    let velocity = frame.direction(at: .init(-75_000, 8_000), tangent: .init(1, 0))*710
      - p.normalized*60
    let state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aerodynamics).observe(.init(time: 100, position: p, velocity: velocity,
      attitude: .windAttitude(position: p, airVelocity: velocity, alpha: radians(15), bank: 0)))
    let chart = frame.coordinates(of: p)
    let stations = [
      PathStation(distance: 0, position: chart.horizontal, tangent: .init(1, 0),
        curvature: 0, height: chart.height, slope: -0.05, speed: 710,
        angleOfAttack: radians(15), bank: 0, segment: .acquisition),
      PathStation(distance: 10_000, position: chart.horizontal + .init(10_000, 0),
        tangent: .init(1, 0), curvature: 0, height: chart.height-500,
        slope: -0.05, speed: 650, angleOfAttack: radians(15),
        bank: 0, segment: .spiral),
    ]
    let assessment = TerminalAssessment(feasible: false,
      violations: [.init(.insufficientLift, "No feasible closed-loop HAC")],
      touchdownSpeed: 150, minimumLiftMargin: -10, maximumBank: 1.0,
      maximumQ: 1000, maximumLoad: 1, score: 0, uncertaintyFraction: 0)
    let bad = TerminalPlan(identifier: 10, createdAt: 100,
      spiral: nil, stations: stations, assessment: assessment)
    let engine = EntryGuidanceEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aerodynamics, permitsReplanning: false)
    engine.considerTerminal(bad, deficit: .insufficientEnergy)
    _ = engine.update(state, synchronousPlanning: false)
    XCTAssertEqual(engine.diagnostics.terminalSteeringActive, false)
    XCTAssertNil(engine.diagnostics.terminalAdviceAge)

    var acceptable = TerminalAssessment(feasible: true, violations: [],
      touchdownSpeed: 100, minimumLiftMargin: 1, maximumBank: 0,
      maximumQ: 1000, maximumLoad: 1, score: 0, uncertaintyFraction: 0)
    acceptable.predictionComplete = true
    let candidate = TerminalPlan(identifier: 11, createdAt: 100,
      spiral: nil, stations: stations, assessment: acceptable)
    let baseline = EntryGuidanceEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aerodynamics, permitsReplanning: false)
    let advised = EntryGuidanceEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aerodynamics, permitsReplanning: false)
    advised.considerTerminal(candidate, deficit: .none)
    let plainTarget = baseline.update(state, synchronousPlanning: false)
    let advisoryTarget = advised.update(state, synchronousPlanning: false)
    XCTAssertEqual(advisoryTarget.angleOfAttack, plainTarget.angleOfAttack, accuracy: 1e-12)
    XCTAssertEqual(advisoryTarget.bank, plainTarget.bank, accuracy: 1e-12)
    engine.considerTerminal(candidate, deficit: .none)
    _ = engine.update(state, synchronousPlanning: false)
    XCTAssertEqual(engine.diagnostics.terminalSteeringActive, false)
    XCTAssertEqual(engine.diagnostics.energyBudgetEndpoint, "entry-objective",
      "Terminal advice must not spend the interface energy reserved for TAEM")
    let objective = EntryCaptureObjective.seed(configuration: c, atmosphere: atmosphere,
      frame: frame, crossAim: engine.crossAim, time: state.time)
    XCTAssertEqual(try XCTUnwrap(engine.diagnostics.captureSpecificEnergy),
      objective.specificEnergy(configuration: c), accuracy: 1e-6)
  }

  func testReciprocalRunwayHeadingActuallyAlignsWithEastboundEntry() {
    var c = MissionConfiguration()
    let westbound = RunwayFrame(runway: c.runway, planet: c.planet)
    let entryPoint = westbound.position(along: 250_000, cross: -60_000, height: 35_000)
    let eastward = -westbound.direction(at: .init(250_000, -60_000), tangent: .init(1, 0))
    let reciprocal = c.runway.reciprocal(planet: c.planet)
    let eastbound = RunwayFrame(runway: reciprocal, planet: c.planet)
    let inChart = eastbound.coordinates(of: entryPoint)
    let eastboundAxis = eastbound.direction(at: inChart.horizontal, tangent: .init(1, 0))
    XCTAssertGreaterThan(eastward.dot(eastboundAxis), 0.90)
    XCTAssertLessThan(eastward.dot(westbound.direction(
      at: .init(250_000, -60_000), tangent: .init(1, 0))), -0.99)
    c.runway = reciprocal
    XCTAssertNoThrow(try c.validate())
  }
}
