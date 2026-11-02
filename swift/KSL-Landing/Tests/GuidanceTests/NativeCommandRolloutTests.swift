import XCTest
import KSLCore
import KSLControl
import KSLPhysics

final class NativeCommandRolloutTests: XCTestCase {
  private struct RecordedHandoff: Decodable {
    let configuration: MissionConfiguration
    let state: VehicleState
  }

  func testSharedRolloutMatchesNativePipelineWithWarmControllerAndLimiter() throws {
    // Genuine recorded STS configuration/state, unchanged capability. This is
    // a three-tick software continuity check, not historical model closure.
    let url = try XCTUnwrap(Bundle.module.url(forResource: "dessert-taem-20261102",
      withExtension: "json", subdirectory: "Fixtures"))
    let recorded = try JSONDecoder().decode(RecordedHandoff.self, from: Data(contentsOf: url))
    let c = recorded.configuration, initial = recorded.state, dt = c.guidance.controlInterval
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let controller = FlightController(configuration: c)
    let plant = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero,
      initialControls: initial.appliedControls ?? .init(), initialState: initial)
    var state = plant.observe(initial)
    var limiter = GuidanceTargetLimiter(previous: .init(
      angleOfAttack: initial.aerodynamics.angleOfAttack, bank: initial.aerodynamics.bank))
    let request = GuidanceSolution(phase: .acquisition, target: .init(
      angleOfAttack: initial.aerodynamics.angleOfAttack + radians(1), bank: initial.aerodynamics.bank + radians(1), speedbrakeFraction: 0.4))
    func limited(_ controller: FlightController) -> GuidanceTarget {
      limiter.update(request.target, phase: request.phase, configuration: c, dt: dt,
        dynamicPressure: state.aerodynamics.dynamicPressure,
        rollResponseSeconds: controller.diagnostics.rollResponseSeconds,
        pitchResponseSeconds: controller.diagnostics.pitchResponseSeconds, controlAuthority: state.controlAuthority)
    }
    let first = limited(controller)
    state = try plant.advance(state, command: controller.command(state: state,
      solution: .init(phase: request.phase, target: first), dt: dt), duration: dt)
    let seed = NativeCommandSeed(state: state, controls: controller.snapshot, limiter: limiter)
    let branch = NativeCommandRollout(seed: seed, configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let independent = NativeCommandRollout(seed: seed, configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    XCTAssertEqual(branch.diagnostics.rollResponseSeconds, controller.diagnostics.rollResponseSeconds)
    XCTAssertEqual(branch.diagnostics.pitchResponseSeconds, controller.diagnostics.pitchResponseSeconds)
    for _ in 0..<2 {
      let target = limited(controller)
      let command = try controller.command(state: state, solution: .init(phase: request.phase, target: target), dt: dt)
      state = try plant.advance(state, command: command, duration: dt)
      let step = try branch.advance(request, duration: dt)
      let other = try independent.advance(request, duration: dt)
      XCTAssertEqual(step.limited, target)
      XCTAssertEqual(step.issued.pitch, command.pitch, accuracy: 1e-12)
      XCTAssertEqual(step.issued.roll, command.roll, accuracy: 1e-12)
      XCTAssertEqual(step.after.position.x, state.position.x, accuracy: 1e-8)
      XCTAssertEqual(step.after.velocity.y, state.velocity.y, accuracy: 1e-8)
      XCTAssertEqual(step.after.aerodynamics.lift, state.aerodynamics.lift, accuracy: 1e-6)
      XCTAssertEqual(step.issued.roll, other.issued.roll, accuracy: 1e-12)
      XCTAssertEqual(step.after.position, other.after.position)
      XCTAssertLessThanOrEqual(abs(step.limited.bank), c.vehicle.maximumBank)
    }
  }
}
