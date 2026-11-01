import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TerminalRecoveryOwnershipTests: XCTestCase {
  func testCrashContactRemainsAdvisoryAndNeverReportsCompletion() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let p = frame.position(along: -10_000, cross: 0, height: 5_000)
    let velocity = frame.direction(at: .init(-10_000, 0), tangent: .init(1, 0)) * 200
    var state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero).observe(.init(time: 100, position: p, velocity: velocity,
        attitude: .windAttitude(position: p, airVelocity: velocity, alpha: radians(10), bank: 0)))
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true)
    _ = try mission.update(state)
    state.time += 0.1
    state.contact = .crashed
    let result = try mission.update(state)
    XCTAssertEqual(result.phase, .touchdown)
    XCTAssertTrue(result.violations.contains { $0.kind == .contactQuality })
  }

  func testLateralCorrectionPenalizesOpeningCrossVelocity() {
    let closing = EntryLateralCaptureKinematics.peakDemand(
      crossError: -40_000, crossVelocity: -200, timeToCapture: 100)
    let opening = EntryLateralCaptureKinematics.peakDemand(
      crossError: -40_000, crossVelocity: 200, timeToCapture: 100)
    XCTAssertEqual(closing, 20, accuracy: 1e-8)
    XCTAssertEqual(opening, 32, accuracy: 1e-8)
    XCTAssertLessThan(closing, opening)
  }

  func testMM305HandoffWaitsForNEPOverheadRegionNotThirtyKilometres() {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    func state(along: Double, cross: Double, altitude: Double) -> VehicleState {
      let position = frame.position(along: along, cross: cross, height: altitude-c.runway.elevation)
      let velocity = frame.direction(at: .init(along, cross), tangent: .init(1, 0))*1100
        - position.normalized*90
      return VehicleState(time: 100, position: position, velocity: velocity, attitude: .init())
    }
    let upRange = state(along: -70_000, cross: 45_000, altitude: c.entrySettings.terminalMaximumAltitude-1)
    XCTAssertFalse(LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
      state: upRange, configuration: c), "Crossing 30 km far up-range is not an MM305 handoff")
    let overhead = state(along: -c.guidance.finalDistance, cross: 0, altitude: 26_000)
    XCTAssertTrue(LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
      state: overhead, configuration: c), "MM304 hands over above the NEP")
    let deadline = state(along: -70_000, cross: 45_000, altitude: c.entrySettings.terminalMinimumAltitude-1)
    XCTAssertTrue(LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
      state: deadline, configuration: c), "MM304 cannot keep authority below the MM305 window")
    XCTAssertFalse(LandingMissionController.terminalHandoffDue(phase: .acquisition, plan: nil,
      state: deadline, configuration: c), "MM305 ownership cannot repeatedly reset itself")
  }

  func testMissionActuallyEntersMM305WithNoQualifiedRoute() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: -70_000, cross: 45_000,
      height: c.entrySettings.terminalMinimumAltitude-c.runway.elevation-100)
    let velocity = frame.direction(at: .init(-70_000, 45_000), tangent: .init(1, 0))*900
      - position.normalized*100
    let state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero).observe(.init(time: 200, position: position, velocity: velocity,
        attitude: .windAttitude(position: position, airVelocity: velocity,
          alpha: radians(18), bank: 0)))
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true, initialPhase: .entry)
    let solution = try mission.update(state)
    XCTAssertEqual(solution.phase, .acquisition)
    XCTAssertNil(solution.planID, "A fallback cannot invent a certified HAC")
    XCTAssertTrue(solution.diagnostic.contains("MM305 no-plan recovery"))
    XCTAssertNotNil(mission.planningRequest(from: state, controls: nil)?.terminalState,
      "Forced MM305 must still search for the next full-route candidate")
  }

  func testUnqualifiedMM305RecoveryCommandsOnlyAttainableForceDirections() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: -70_000, cross: 45_000, height: 25_000-c.runway.elevation)
    let velocity = frame.direction(at: .init(-70_000, 45_000), tangent: .init(1, 0))*750
      - position.normalized*90
    let state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero).observe(.init(time: 200, position: position, velocity: velocity,
        attitude: .windAttitude(position: position, airVelocity: velocity,
          alpha: radians(18), bank: 0)))
    let recovery = TerminalRecoveryGuidanceEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
      .update(state, attainableAlphaCeiling: nil, pitchResponseSeconds: nil)
    XCTAssertTrue(recovery.target.angleOfAttack.isFinite)
    XCTAssertTrue(recovery.target.bank.isFinite)
    XCTAssertTrue(recovery.lateralDemand.isFinite)
    XCTAssertTrue(recovery.lateralAvailable.isFinite)
    XCTAssertGreaterThanOrEqual(recovery.lateralAvailable, 0)
    XCTAssertLessThanOrEqual(abs(recovery.target.bank), c.vehicle.maximumBank+1e-12)
    XCTAssertLessThanOrEqual(recovery.target.angleOfAttack, c.vehicle.maximumAngleOfAttack+1e-12)
    XCTAssertLessThanOrEqual(recovery.target.desiredFlightPath, 1e-12, "No-plan recovery never commands a climb")
  }

  private func glideState(_ c: MissionConfiguration, along: Double, cross: Double, height: Double,
                          speed: Double, headingOffset: Double) throws -> VehicleState {
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: along, cross: cross, height: height)
    let tangent = Vector2(cos(headingOffset), sin(headingOffset))
    let velocity = frame.direction(at: .init(along, cross), tangent: tangent)*speed
      - position.normalized*speed*0.15
    return ShuttlePhysicsEngine(configuration: c, atmosphere: try AtmosphericModelEngine(),
      aerodynamics: try AerodynamicsEngine()).observe(.init(time: 300, position: position, velocity: velocity,
        attitude: .windAttitude(position: position, airVelocity: velocity, alpha: radians(10), bank: 0)))
  }

  func testLowEnergyRecoveryFliesBestGlideToPavementInsteadOfTheNEP() throws {
    let c = MissionConfiguration()
    let recovery = TerminalRecoveryGuidanceEngine(configuration: c,
      atmosphere: try AtmosphericModelEngine(), aerodynamics: try AerodynamicsEngine())
    // Abeam and low: the NEP alignment detour is unaffordable.
    let state = try glideState(c, along: -25_000, cross: 6_000, height: 4_500, speed: 170, headingOffset: -0.2)
    let decision = recovery.update(state, attainableAlphaCeiling: nil, pitchResponseSeconds: nil)
    XCTAssertNotEqual(decision.mode, .alignment)
    XCTAssertGreaterThanOrEqual(decision.aimAlong, -c.guidance.finalDistance)
    XCTAssertEqual(decision.target.speedbrakeFraction, 0, "Low energy never opens the speedbrake")
    XCTAssertLessThan(decision.target.desiredFlightPath, 0)
    if decision.mode == .runwayUnreachable {
      XCTAssertGreaterThan(decision.requiredRange, decision.availableRange)
    } else {
      XCTAssertLessThanOrEqual(decision.requiredRange, decision.availableRange * 1.0001 + 1)
    }
  }

  func testRecoveryNeverCommandsAboveEvidencedAttainableAoA() throws {
    let c = MissionConfiguration()
    let recovery = TerminalRecoveryGuidanceEngine(configuration: c,
      atmosphere: try AtmosphericModelEngine(), aerodynamics: try AerodynamicsEngine())
    let state = try glideState(c, along: -80_000, cross: 1_000, height: 26_000, speed: 900, headingOffset: 0)
    let ceiling = radians(26)
    let decision = recovery.update(state, attainableAlphaCeiling: ceiling, pitchResponseSeconds: 3)
    XCTAssertLessThanOrEqual(decision.target.angleOfAttack, ceiling + 1e-12)
  }

  func testSustainedPitchSaturationBoundsThenReleasesAttainableAoA() {
    var monitor = AttainableIncidenceMonitor()
    var feedback = ControlDiagnostics()
    feedback.pitchSaturated = true; feedback.pitchResponseSeconds = 3
    // Commanded 40 deg, measured 25..27 deg while saturated for > response time.
    for k in 0...40 {
      monitor.observe(time: Double(k) * 0.1, measuredAlpha: radians(25 + 2 * Double(k % 2)),
        commandedAlpha: radians(40), feedback: feedback, fallbackResponseSeconds: 1)
    }
    XCTAssertEqual(monitor.ceiling ?? 0, radians(27), accuracy: 1e-12)
    // Holding the bound without saturation releases it for re-identification.
    feedback.pitchSaturated = false
    monitor.observe(time: 5, measuredAlpha: radians(27), commandedAlpha: radians(27),
      feedback: feedback, fallbackResponseSeconds: 1)
    XCTAssertNil(monitor.ceiling)
  }

  func testExcessEnergyWithoutPlanSTurnsInsteadOfDiving() throws {
    let c = MissionConfiguration()
    let recovery = TerminalRecoveryGuidanceEngine(configuration: c,
      atmosphere: try AtmosphericModelEngine(), aerodynamics: try AerodynamicsEngine())
    // 2211 flight shape (runway-aligned, high, close in), with clear excess.
    let state = try glideState(c, along: -19_700, cross: 800, height: 15_000, speed: 167, headingOffset: 0)
    let decision = recovery.update(state, attainableAlphaCeiling: nil, pitchResponseSeconds: 3)
    XCTAssertEqual(decision.mode, .alignment)
    XCTAssertGreaterThan(decision.availableRange, decision.requiredRange * (1 + c.vehicle.aerodynamicUncertainty))
    XCTAssertLessThan(decision.target.bank, 0, "Right of centerline: the S-turn leg turns back across it")
    XCTAssertGreaterThan(decision.target.desiredFlightPath, atan(-15_000 / decision.requiredRange),
      "Descent is spread over the lengthened path, not dived straight at the NEP")
  }
}
