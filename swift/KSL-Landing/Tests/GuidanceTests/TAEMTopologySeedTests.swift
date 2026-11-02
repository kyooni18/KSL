import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMTopologySeedTests: XCTestCase {
  func testCommandFamiliesRetainActualRequestsAndDoNotPruneSummedEventCeilings() throws {
    struct Recorded: Decodable { var configuration: MissionConfiguration; let state: VehicleState, target: GuidanceTarget }
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    let r = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    var c = r.configuration
    // Search coverage only; genuine STS physical capability is unchanged.
    c.guidance.maximumPredictionDuration = c.guidance.controlInterval
    let held = TAEMCommandSegment(duration: c.guidance.controlInterval,
      command: .init(phase: .acquisition, target: r.target))
    let family = try TAEMTopologySeeds.generate(configuration: c, baseTarget: r.target,
      parameters: [.init(alpha: r.target.angleOfAttack, bankMagnitude: c.vehicle.maximumBank,
        courseWork: .pi/2, maximumSegmentDuration: c.guidance.controlInterval)],
      incumbent: [held], protection: [held], id: "recorded-seed")
    XCTAssertEqual(family.requestedParameterCount, 1)
    XCTAssertEqual(family.candidates.count, 9)
    XCTAssertEqual(Set(family.candidates.map(\.id)).count, 9)
    XCTAssertTrue(try XCTUnwrap(family.candidates.first { $0.topology == .direct }).segments.isEmpty)
    XCTAssertEqual(family.candidates[0].segments[0].command.target, r.target)
    XCTAssertEqual(family.candidates[1].segments[0].command.target, r.target)
    let winding = try XCTUnwrap(family.candidates.first { $0.topology == .windingLeft })
    XCTAssertEqual(winding.segments[0].untilCourseChange, -5 * .pi/2)
    let reversal = try XCTUnwrap(family.candidates.first { $0.topology == .leftRight })
    XCTAssertEqual(reversal.segments.map { $0.command.target.bank }, [-c.vehicle.maximumBank, c.vehicle.maximumBank])
    XCTAssertEqual(reversal.segments.map(\.untilCourseChange), [-.pi/2, .pi/2])
    XCTAssertGreaterThan(reversal.segments.reduce(0) { $0+$1.duration }, c.guidance.maximumPredictionDuration)
    XCTAssertTrue(reversal.segments.allSatisfy {
      $0.command.phase == .acquisition && $0.command.target.angleOfAttack == r.target.angleOfAttack
        && $0.command.target.speedbrakeFraction == r.target.speedbrakeFraction
    })
    // Missing model evidence stops before any propagation. This checks that
    // summed event ceilings no longer veto the candidate/refinement center.
    let poll = try TAEMCommandSearch(configuration: c,
      atmosphere: AtmosphericModelEngine(planet: c.planet),
      aerodynamics: AerodynamicsEngine(referenceArea: c.vehicle.referenceArea))
      .refineOnce(reversal, from: .init(state: r.state, controls: nil, limiter: .init(previous: r.target)),
        steps: .init(alphaRadians: c.guidance.alphaSlewRate*c.guidance.controlInterval,
          bankRadians: c.guidance.bankSlewRate*c.guidance.controlInterval,
          durationSeconds: c.guidance.controlInterval/2), maximumEvaluations: 1) { _, _ in
            XCTFail("No identified pitch balance"); return nil
          }
    XCTAssertEqual(poll.trials.count, 1)
    XCTAssertEqual(poll.trials[0].prefix.end, .unsupportedDynamics)
    XCTAssertTrue(poll.trials[0].prefix.steps.isEmpty)
  }
}
