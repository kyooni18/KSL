import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class MM305AdmissionQualificationTests: XCTestCase {
  func testPreviewPhysicalPassCannotTransferUnqualifiedNominalPlan() throws {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let atmosphere = try AtmosphericModelEngine()
    let position = frame.position(along: -8_000, cross: 0,
      height: c.entrySettings.terminalTargetAltitude - c.runway.elevation)
    let speed = c.entrySettings.terminalTargetMach
      * atmosphere.sample(position: position, time: 100).speedOfSound
    let forward = frame.direction(at: .init(-8_000, 0), tangent: .init(1, 0))
    let velocity = forward * (speed * cos(radians(3)))
      - position.normalized * (speed * sin(radians(3)))
    var aero = AerodynamicState()
    aero.airspeed = speed
    aero.mach = c.entrySettings.terminalTargetMach
    aero.dynamicPressure = 0.5 * atmosphere.sample(position: position, time: 100).density * speed * speed
    aero.bank = 0
    aero.sideslip = 0
    var state = VehicleState(time: 100, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: radians(18), bank: 0),
      aerodynamics: aero)
    state.controlAuthority = .init(
      positiveTorque: .init(100_000, 100_000, 100_000),
      negativeTorque: .init(-100_000, -100_000, -100_000),
      inertia: c.vehicle.inertia, evidence: .measured)
    let coords = frame.coordinates(of: position)
    let stations = [
      PathStation(distance: 0, position: coords.horizontal, tangent: .init(1, 0),
        curvature: 0, height: coords.height, slope: -0.05, speed: speed,
        angleOfAttack: radians(18), bank: 0, segment: .acquisition),
      PathStation(distance: 5_000, position: coords.horizontal + .init(5_000, 0),
        tangent: .init(1, 0), curvature: 0, height: coords.height - 250,
        slope: -0.05, speed: speed * 0.97, angleOfAttack: radians(18),
        bank: 0, segment: .final),
    ]
    var assessment = TerminalAssessment(feasible: true, violations: [],
      touchdownSpeed: 100, minimumLiftMargin: 1, maximumBank: 0, maximumQ: 1000,
      maximumLoad: 1, score: 0, uncertaintyFraction: 0)
    assessment.predictionComplete = true
    let previewOnly = TerminalPlan(identifier: 77, createdAt: 100, spiral: nil,
      stations: stations, assessment: assessment, closedLoopCases: nil)
    let advisory = MM305Admission.evaluate(state, plan: previewOnly, configuration: c,
      requireQualification: false)
    XCTAssertTrue(MM305Admission.shouldQualifyPreview(state, plan: previewOnly, configuration: c),
      "Admissible preview must trigger real nominal closed-loop qualification")
    XCTAssertTrue(advisory.accepted,
      "A geometrically admissible advisory should be available for exploration: \(advisory.violations)")
    let strict = MM305Admission.evaluate(state, plan: previewOnly, configuration: c)
    XCTAssertFalse(strict.accepted, "MM305 must not claim an unverified landing continuation.")
    XCTAssertTrue(strict.violations.contains {
      $0.message.contains("rollout unconfirmed")
    })

    let certifiedNominal = TerminalClosedLoopCase(liftMultiplier: 1, dragMultiplier: 1,
      feasible: true, elapsed: 250, touchdownSpeed: 100, touchdownSink: -2,
      touchdownPitch: radians(10), stoppedAlong: 0, maximumTrackingError: 10,
      violations: [], evidence: "synthetic-unit-test-nominal")
    let closed = TerminalPlan(identifier: 78, createdAt: 100, spiral: nil,
      stations: stations, assessment: assessment, closedLoopCases: [certifiedNominal])
    XCTAssertTrue(closed.nominalQualified)
    let verified = MM305Admission.evaluate(state, plan: closed, configuration: c)
    XCTAssertTrue(verified.accepted, "\(verified.violations)")
  }
}
