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

  func testTAEMHandoffIsOnInterfaceMachNotAltitudeUpRange() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: try AerodynamicsEngine())
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    func state(along: Double, altitude: Double, speed: Double) -> VehicleState {
      let position = frame.position(along: along, cross: 0, height: altitude-c.runway.elevation)
      let velocity = frame.direction(at: .init(along, 0), tangent: .init(1, 0))*speed - position.normalized*90
      return physics.observe(.init(time: 100, position: position, velocity: velocity,
        attitude: .windAttitude(position: position, airVelocity: velocity, alpha: radians(15), bank: 0)))
    }
    let fast = state(along: -150_000, altitude: c.entrySettings.terminalMaximumAltitude-1, speed: 1200)
    XCTAssertGreaterThan(fast.aerodynamics.mach, c.mm304DeliveryMach)
    XCTAssertFalse(LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
      state: fast, configuration: c), "Crossing 30 km far up-range is not the TAEM interface")
    let interface = state(along: -96_000, altitude: 25_000, speed: 700)
    XCTAssertLessThanOrEqual(interface.aerodynamics.mach, c.mm304DeliveryMach)
    XCTAssertTrue(LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
      state: interface, configuration: c), "TAEM starts on interface Mach")
    let deadline = state(along: -150_000, altitude: c.entrySettings.terminalMinimumAltitude-1, speed: 1200)
    XCTAssertTrue(LandingMissionController.terminalHandoffDue(phase: .entry, plan: nil,
      state: deadline, configuration: c), "MM304 cannot keep authority below its window")
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
    XCTAssertTrue(solution.diagnostic.contains("TAEM phase="), "TAEM owns MM305 without a certificate")
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
}
