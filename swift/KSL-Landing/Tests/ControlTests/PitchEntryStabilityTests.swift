import KSLCore
import XCTest

@testable import KSLControl

/// Regressions from the Nov 1 STS MM304 flight: RCS pitch chatter at Mach 6+
/// and a cumulative 1-cm fuel/CoM drift resetting a learned pitch bias.
final class PitchEntryStabilityTests: XCTestCase {
  private let position = Vector3(610_000, 0, 0)
  private let velocity = Vector3(0, 2_000, 0)

  private func sample(time: Double, alpha: Double = radians(5),
                      center: Double = 0, mass: Double = 25_000,
                      pitchRate: Double = 0) -> VehicleState {
    var aero = AerodynamicState()
    aero.airspeed = 2_000; aero.mach = 6; aero.dynamicPressure = 1_000
    aero.angleOfAttack = alpha
    var systems = SystemState(); systems.rcs = true
    var s = VehicleState(time: time, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: alpha, bank: 0),
      angularVelocity: .init(0, pitchRate, 0), mass: mass,
      aerodynamics: aero, systems: systems)
    s.controlAuthority = .init(positiveTorque: .init(20_000, 50_000, 20_000),
      negativeTorque: .init(-20_000, -50_000, -20_000),
      inertia: .init(400_000, 1_000_000, 400_000))
    s.vesselConfiguration = .init(revision: 1, partCount: 26, anchorPart: 109,
      centerOfMassFromAnchor: .init(center, -9.8, 0.3),
      centerOfMassEvidence: .measured, measuredAt: time)
    return s
  }

  func testEntryPitchDamperUsesGyroInsteadOfDifferentiatingAttitudeJitter() throws {
    let controller = FlightController(configuration: .init())
    let target = GuidanceSolution(phase: .entry,
      target: .init(angleOfAttack: radians(5), bank: 0))
    var s = sample(time: 1)
    for i in 0..<20 {
      s.time = 1+Double(i)*0.1
      _ = try controller.command(state: s, solution: target, dt: 0.1)
    }
    var peak = 0.0
    for i in 0..<40 {
      let angle = radians(5 + (i.isMultiple(of: 2) ? 0.18 : -0.18))
      s = sample(time: 3+Double(i)*0.1, alpha: angle)
      let c = try controller.command(state: s, solution: target, dt: 0.1)
      peak = max(peak, abs(c.pitch))
    }
    // A 0.18-degree snapshot wobble with a zero body gyro cannot command
    // half/full RCS. The old 0.1-s error derivative swung near saturation.
    XCTAssertLessThan(peak, 0.15)
    XCTAssertEqual(controller.diagnostics.relativeRate.y, 0, accuracy: 1e-9)
  }

  func testWindFrameReferenceRatePreventsDampingARealPitchSlew() throws {
    let controller = FlightController(configuration: .init())
    for i in 0..<60 {
      let alpha = radians(5 + 0.2 * Double(i))
      let s = sample(time: 1+0.1*Double(i), alpha: alpha,
        pitchRate: radians(2))
      let target = GuidanceSolution(phase: .entry, target: .init(angleOfAttack: alpha))
      _ = try controller.command(state: s, solution: target, dt: 0.1)
    }
    // Moving target +2 deg/s, measured body +2 deg/s: little relative rate.
    XCTAssertEqual(controller.diagnostics.bodyRate.y, radians(2), accuracy: 1e-9)
    XCTAssertEqual(controller.diagnostics.relativeRate.y, 0, accuracy: radians(0.08))
  }

  private func warmTrim() throws -> (FlightController, VehicleState) {
    let controller = FlightController(configuration: .init())
    let target = GuidanceSolution(phase: .entry,
      target: .init(angleOfAttack: radians(5.5)))
    var last = sample(time: 1)
    for i in 0..<90 {
      last = sample(time: 1 + Double(i)*0.1, mass: 25_000-Double(i)*0.1)
      _ = try controller.command(state: last, solution: target, dt: 0.1)
    }
    XCTAssertGreaterThan(controller.snapshot.pitchTrim, 0.01)
    return (controller, last)
  }

  func testContinuousPropellantCoMDriftRetainsTrim() throws {
    let (controller, old) = try warmTrim()
    let previous = controller.snapshot.pitchTrim
    let next = sample(time: old.time+0.1, center: 0.0001, mass: old.mass-0.1)
    var monitor = VehicleDynamicsMonitor()
    _ = monitor.observe(sample(time: old.time, center: -0.012, mass: 25_000))
    let event = try XCTUnwrap(monitor.observe(next))
    controller.adaptToDynamicsChange(event, at: next)
    XCTAssertEqual(controller.snapshot.pitchTrim, previous, accuracy: 1e-12)
    XCTAssertNil(controller.snapshot.previousState) // rebase temporal derivatives
  }

  func testAbruptCoMOrTopologyChangeStillInvalidatesTrim() throws {
    let (controller, old) = try warmTrim()
    let next = sample(time: old.time+0.1, center: 0.03, mass: old.mass)
    var monitor = VehicleDynamicsMonitor()
    _ = monitor.observe(sample(time: old.time, center: 0, mass: 25_000))
    let event = try XCTUnwrap(monitor.observe(next))
    controller.adaptToDynamicsChange(event, at: next)
    XCTAssertEqual(controller.snapshot.pitchTrim, 0, accuracy: 1e-12)
  }
}
