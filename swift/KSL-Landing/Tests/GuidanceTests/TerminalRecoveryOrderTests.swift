import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

/// Accounting-contract tests only. End-event DTOs are constructed below; no
/// vehicle capability changes, trajectory simulation, or landing claim is made.
final class TerminalRecoveryOrderTests: XCTestCase {
  private struct Recorded: Decodable {
    let configuration: MissionConfiguration, state: VehicleState, target: GuidanceTarget
  }
  private func setup() throws -> (Recorded, TerminalComparisonContext) {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    let r = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    let c = r.configuration
    let context = TerminalComparisonContext(seed: .init(state: r.state, controls: nil,
      limiter: .init(previous: r.target)), configuration: c,
      atmosphere: try .init(planet: c.planet), aerodynamics: try .init(referenceArea: c.vehicle.referenceArea))
    return (r, context)
  }
  private func outcome(_ context: TerminalComparisonContext, pressure: Double, integral: Double,
                       complete: Bool = true, feasible: Bool = false, elapsed: Double = 100, stoppedAlong: Double? = nil) throws -> TerminalClosedLoopCase {
    let c = context.configuration, frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let along = stoppedAlong ?? c.guidance.touchdownAlong
    let position = frame.position(along: along, cross: 0, height: c.vehicle.gearHeight)
    var state = VehicleState(time: context.startState.time+elapsed, position: position,
      velocity: .zero, attitude: .windAttitude(position: position, airVelocity: frame.alongAxis, alpha: 0, bank: 0),
      mass: context.startState.mass, contact: .stopped)
    state.systems.gearDown = true
    state.wheelContact = .init(mainGear: true, noseGear: true, wheelCount: 3, evidence: .provisional)
    let event = try JSONDecoder().decode(TouchdownEvent.self, from: JSONSerialization.data(withJSONObject: [
      "time": state.time-c.guidance.stopConfirmationDuration-1,
      "along": c.guidance.touchdownAlong, "cross": 0, "horizontalSpeed": c.vehicle.landingSpeed,
      "verticalSpeed": -1, "bank": 0, "pitch": 0, "onRunway": true, "acceptable": true]))
    return .init(liftMultiplier: 1, dragMultiplier: 1, feasible: feasible,
      elapsed: state.time-context.startState.time, touchdownSpeed: event.horizontalSpeed,
      touchdownSink: event.verticalSpeed, touchdownPitch: event.pitch,
      stoppedAlong: along, maximumTrackingError: 0,
      violations: pressure > 0 ? [.init(.excessiveDynamicPressure, "accounting-contract fixture")] : [],
      evidence: "unit-accounting-only-not-a-flight-outcome",
      evaluation: .init(status: complete ? .complete : .incomplete, end: complete ? .sustainedStop : .horizon,
        state: state, aerodynamicWork: -1, energyChange: -1, heightChange: -1,
        maximumPressureExcess: pressure, pressureExcessIntegral: integral,
        maximumLoadExcess: 0, loadExcessIntegral: 0, comparisonContext: context, touchdown: complete ? event : nil))
  }
  func testMissingSuffixCannotBeatCompleteFailedRecoveryAndBetterFailureNeedsNoQualification() throws {
    let (_, context) = try setup()
    let failed = try outcome(context, pressure: 200, integral: 300)
    let truncated = try outcome(context, pressure: 0, integral: 0, complete: false)
    XCTAssertFalse(failed.qualified)
    XCTAssertEqual(TerminalRecoveryOrder.compare(truncated, to: failed), .incumbentPreferred)
    XCTAssertEqual(TerminalRecoveryOrder.compare(failed, to: truncated), .challengerPreferred)
    var falselyComplete = truncated
    let extent = try XCTUnwrap(truncated.evaluation)
    falselyComplete.evaluation = .init(status: .complete, end: .horizon, state: extent.state,
      aerodynamicWork: 0, energyChange: 0, heightChange: 0,
      maximumPressureExcess: 0, pressureExcessIntegral: 0, maximumLoadExcess: 0, loadExcessIntegral: 0,
      comparisonContext: context)
    XCTAssertEqual(TerminalRecoveryOrder.compare(falselyComplete, to: failed), .incumbentPreferred)
    let betterFailed = try outcome(context, pressure: 100, integral: 200)
    XCTAssertFalse(betterFailed.qualified)
    XCTAssertEqual(TerminalRecoveryOrder.compare(betterFailed, to: failed), .challengerPreferred)
    let tradeoff = try outcome(context, pressure: 100, integral: 400)
    XCTAssertEqual(TerminalRecoveryOrder.compare(tradeoff, to: failed), .incomparableOutcomes)
    // A contradictory optimistic flag cannot waive a recorded envelope excess.
    XCTAssertFalse(try outcome(context, pressure: 100, integral: 200, feasible: true).qualified)
  }
  func testComparisonRejectsMixedOriginsModelsAndPoliciesAndDoesNotRewardStopPrecision() throws {
    let (r, context) = try setup()
    let incumbent = try outcome(context, pressure: 0, integral: 0, feasible: true, elapsed: 100)
    let faster = try outcome(context, pressure: 0, integral: 0, feasible: true, elapsed: 90,
      stoppedAlong: context.configuration.guidance.touchdownAlong+1)
    XCTAssertTrue(incumbent.qualified)
    XCTAssertEqual(TerminalRecoveryOrder.compare(faster, to: incumbent), .challengerPreferred)
    XCTAssertEqual(r.configuration, context.configuration)
    let encoded = try XCTUnwrap(JSONSerialization.jsonObject(with: JSONEncoder().encode(context)) as? [String: Any])
    for field in ["originID", "atmosphereID", "aerodynamicsID", "continuationPolicy"] {
      // Identity/policy metadata only; no physical model parameter is changed.
      var changed = encoded
      changed[field] = field == "continuationPolicy" ? "another-ending-policy" : UUID().uuidString
      let other = try JSONDecoder().decode(TerminalComparisonContext.self,
        from: JSONSerialization.data(withJSONObject: changed))
      XCTAssertEqual(TerminalRecoveryOrder.compare(try outcome(other, pressure: 0, integral: 0, feasible: true),
        to: incumbent), .contextMismatch, field)
    }
    let decoded = try JSONDecoder().decode(TerminalClosedLoopCase.self,
      from: JSONEncoder().encode(incumbent))
    XCTAssertEqual(decoded.evaluation?.comparisonContext, context)
  }
}
