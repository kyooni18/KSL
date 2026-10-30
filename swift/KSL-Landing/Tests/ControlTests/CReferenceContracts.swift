import Foundation
import KSLCore
import KSLPhysics
import XCTest
@testable import KSLControl

final class CReferenceContracts: XCTestCase {
  private func state(alpha: Double = 0, bank: Double = 0, time: Double = 100) -> VehicleState {
    let position = Vector3(610_000, 0, 0), velocity = Vector3(0, 1000, 0)
    var aero = AerodynamicState()
    aero.airspeed = 1000; aero.mach = 3; aero.dynamicPressure = 1000
    aero.angleOfAttack = alpha; aero.bank = bank
    return .init(time: time, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: alpha, bank: bank),
      aerodynamics: aero, radarAltitude: 10_000,
      controlAuthority: .init(positiveTorque: .init(200_000, 200_000, 150_000),
        negativeTorque: .init(-200_000, -200_000, -150_000), inertia: .init(400_000, 400_000, 300_000)))
  }
  func testCQuaternionErrorSignsAndShortRotation() throws {
    let s = state()
    let target = try XCTUnwrap(AirRelativeAttitude.target(position: s.position,
      airVelocity: s.velocity, alpha: radians(10), bank: 0))
    let e = AirRelativeAttitude.error(current: s.attitude, target: target)
    XCTAssertEqual(degrees(e.y), 10, accuracy: 1e-9)
    XCTAssertEqual(e.x, 0, accuracy: 1e-9)
    let current = state(bank: radians(179))
    let reversed = try XCTUnwrap(AirRelativeAttitude.target(position: s.position,
      airVelocity: s.velocity, alpha: 0, bank: radians(-179)))
    XCTAssertEqual(degrees(AirRelativeAttitude.error(current: current.attitude, target: reversed).x),
                   2, accuracy: 1e-9)
  }
  func testCMovingReferenceUsesRelativeErrorForDamping() throws {
    var a = state(), b = state()
    b.angularVelocity = .init(radians(-4), radians(7), radians(3))
    let first = FlightController(configuration: .init()), second = FlightController(configuration: .init())
    let solution = GuidanceSolution(phase: .entry, target: .init(angleOfAttack: radians(5), bank: radians(-6)))
    _ = try first.command(state: a, solution: solution, dt: 0.1)
    _ = try second.command(state: b, solution: solution, dt: 0.1)
    a.time += 0.1; b.time += 0.1
    _ = try first.command(state: a, solution: solution, dt: 0.1)
    _ = try second.command(state: b, solution: solution, dt: 0.1)
    XCTAssertEqual(first.diagnostics.relativeRate, second.diagnostics.relativeRate)
    XCTAssertEqual(second.diagnostics.relativeRate, .zero)
    XCTAssertEqual(second.diagnostics.bodyRate, b.angularVelocity)
  }
  func testBankEulerAliasDoesNotCommandRoll() throws {
    var s = state(); s.aerodynamics.bank = .pi
    let controller = FlightController(configuration: .init())
    let command = try controller.command(state: s,
      solution: .init(phase: .entry, target: .init()), dt: 0.1)
    XCTAssertEqual(command.roll, 0, accuracy: 1e-12)
    XCTAssertEqual(command.yaw, 0, accuracy: 1e-12)
  }
  func testLivePitchAuthorityOverridesProvisionalCoefficients() throws {
    var c = MissionConfiguration()
    c.vehicle.controlPlant.pitchAccelerationPerKPa = radians(0.25)
    let controller = FlightController(configuration: c)
    _ = try controller.command(state: state(),
      solution: .init(phase: .entry, target: .init(angleOfAttack: radians(5))), dt: 0.1)
    XCTAssertEqual(controller.diagnostics.reportedAuthority.y, 0.5, accuracy: 1e-12)
    XCTAssertEqual(controller.diagnostics.authority.y, 0.5, accuracy: 1e-12)
    XCTAssertEqual(controller.diagnostics.authorityEvidence, .measured)
  }
  func testWindTargetIsIndependentOfGroundVelocity() throws {
    var s = state(alpha: radians(35), bank: radians(60))
    s.atmosphere.wind = .init(0, 0, 100)
    let air = s.velocity - s.atmosphere.wind
    let target = try XCTUnwrap(AirRelativeAttitude.target(position: s.position,
      airVelocity: air, alpha: radians(35), bank: radians(60)))
    let flow = target.conjugate.rotate(air.normalized)
    XCTAssertEqual(atan2(flow.z, flow.x), radians(35), accuracy: 1e-12)
    XCTAssertEqual(flow.y, 0, accuracy: 1e-12)
  }
  func testCappedLagEquationMatchesCUnits() {
    // Evaluated directly from fc_capped_pd_command, f1eebdc, degree inputs.
    let error = 5.0, rate = 2.0, target = 0.5, authority = 30.0
    let lag = 0.12, previous = 0.2, wn = 1.05, damping = 1.15
    let applied = previous * authority
    let expected = (wn * wn * (error - (rate - target) * lag - 0.5 * applied * lag * lag)
      - 2 * damping * wn * (rate + applied * lag - target)) / authority
    XCTAssertEqual(ControlLaw.demand(error: radians(error), rate: radians(rate),
      targetRate: radians(target), authority: radians(authority), wn: wn, damping: damping,
      lag: lag, previous: previous), expected, accuracy: 1e-12)
  }
  func testInvalidTimeCannotIntegrateOrReuseCommands() throws {
    let controller = FlightController(configuration: .init())
    let solution = GuidanceSolution(phase: .entry, target: .init())
    _ = try controller.command(state: state(), solution: solution, dt: 0.1)
    XCTAssertThrowsError(try controller.command(state: state(time: 99), solution: solution, dt: 0.1))
    controller.reset()
    XCTAssertNoThrow(try controller.command(state: state(time: 99), solution: solution, dt: 0.1))
  }
}
