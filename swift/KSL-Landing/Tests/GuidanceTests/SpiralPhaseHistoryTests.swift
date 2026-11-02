import XCTest
import KSLCore
@testable import KSLGuidance

/// Geometric coordinate/representation checks only, not a synthetic plant or
/// an assertion of feasible tracking on either spiral.
final class SpiralPhaseHistoryTests: XCTestCase {
  func testReprojectionPreservesMeasuredWorkAndExplicitWindingWithoutFreeSenseSwitch() throws {
    let c = MissionConfiguration(), radius = c.guidance.minimumHACRadius
    for side in [-1, 1] {
      let spiral = try SpiralHAC(exitAlong: -c.guidance.finalDistance, finalRadius: radius,
        growth: radius/10, turnAngle: 2 * .pi+0.4, side: side)
      let initialPoint = spiral.sample(remainingAngle: spiral.turnAngle).position
      var history = try SpiralPhaseHistory(spiral: spiral, unwrappedHint: spiral.turnAngle,
        point: initialPoint, time: 0, maximumSampleInterval: c.guidance.maximumSnapshotSpan)
      let point = spiral.sample(remainingAngle: 2 * .pi-0.2).position
      history.observe(point: point, time: c.guidance.controlInterval)
      XCTAssertEqual(history.phase, 2 * .pi-0.2, accuracy: 1e-12)
      XCTAssertEqual(history.measuredTurn, 0.6, accuracy: 1e-12)
      let before = history.phase, turn = history.measuredTurn
      let replacement = try SpiralHAC(exitAlong: spiral.exitAlong-c.runway.width,
        finalRadius: radius+c.runway.width, growth: spiral.growth,
        turnAngle: spiral.turnAngle, side: side)
      try history.rebase(to: replacement, at: point, time: history.time)
      XCTAssertEqual(history.measuredTurn, turn)
      XCTAssertLessThan(abs(history.phase-before), .pi)
      XCTAssertEqual(history.phase, history.initialPhase-history.measuredTurn+history.representationShift, accuracy: 1e-12)
      let changedPhase = history.phase
      let next = replacement.sample(remainingAngle: changedPhase-0.1).position
      history.observe(point: next, time: 2*c.guidance.controlInterval)
      XCTAssertEqual(history.measuredTurn, turn+0.1, accuracy: 1e-12)
      let encoded = try JSONEncoder().encode(history)
      let decoded = try JSONDecoder().decode(SpiralPhaseHistory.self, from: encoded)
      XCTAssertEqual(decoded.phase, history.phase)
      XCTAssertEqual(decoded.measuredTurn, history.measuredTurn)
      let reversed = try SpiralHAC(exitAlong: replacement.exitAlong, finalRadius: replacement.finalRadius,
        growth: replacement.growth, turnAngle: replacement.turnAngle, side: -side)
      let frozen = history.phase
      XCTAssertThrowsError(try history.rebase(to: reversed, at: next, time: history.time))
      XCTAssertEqual(history.phase, frozen)
      XCTAssertThrowsError(try history.rebase(to: replacement, at: next, time: history.time+c.guidance.controlInterval))
      XCTAssertEqual(history.status, .available)
      history.observe(point: next, time: history.time+2*c.guidance.maximumSnapshotSpan)
      XCTAssertEqual(history.status, .sampleGap)
      XCTAssertThrowsError(try history.rebase(to: replacement, at: next, time: history.time))
    }
  }
}
