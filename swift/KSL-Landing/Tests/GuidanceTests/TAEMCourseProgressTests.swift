import XCTest
import KSLCore
@testable import KSLGuidance

/// Chart-geometry bookkeeping only, not a dynamically possible trajectory or
/// a vehicle performance experiment. No physical plant is instantiated.
final class TAEMCourseProgressTests: XCTestCase {
  func testUnwrappedCourseRetainsWindingAndReversalAndRejectsLostCoverage() throws {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    struct Recorded: Decodable { let configuration: MissionConfiguration, state: VehicleState }
    let recorded = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    let c = recorded.configuration
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    func sample(_ angle: Double, tick: Int) -> VehicleState {
      var s = recorded.state
      s.position = frame.position(along: 0, cross: 0, height: 0)
      s.velocity = frame.alongAxis*cos(angle)+frame.rightAxis*sin(angle)
      s.time += Double(tick)*c.guidance.controlInterval
      return s
    }
    var progress = TAEMCourseProgress(frame: frame, state: sample(0, tick: 0), maximumSampleInterval: c.guidance.maximumSnapshotSpan)
    for tick in 1...8 { progress.observe(sample(Double(tick) * .pi/2, tick: tick)) }
    XCTAssertEqual(progress.status, .available)
    XCTAssertEqual(progress.signedChange, 4 * .pi, accuracy: 1e-12)
    XCTAssertEqual(progress.absoluteChange, 4 * .pi, accuracy: 1e-12)
    let last = sample(4 * .pi, tick: 8)
    progress.observe(last)
    XCTAssertEqual(progress.signedChange, 4 * .pi, accuracy: 1e-12)
    progress.observe(sample(7 * .pi/2, tick: 9))
    progress.observe(sample(3 * .pi, tick: 10))
    XCTAssertEqual(progress.signedChange, 3 * .pi, accuracy: 1e-12)
    XCTAssertEqual(progress.absoluteChange, 5 * .pi, accuracy: 1e-12)
    var stopped = sample(3 * .pi, tick: 11)
    stopped.velocity = .zero
    progress.observe(stopped)
    XCTAssertEqual(progress.status, .undefinedCourse)
    let through = progress.throughTime
    progress.observe(sample(2 * .pi, tick: 12))
    XCTAssertEqual(progress.throughTime, through, "A later sample cannot recover missing winding")
    var stale = TAEMCourseProgress(frame: frame, state: last, maximumSampleInterval: c.guidance.maximumSnapshotSpan)
    stale.observe(sample(0, tick: 7))
    XCTAssertEqual(stale.status, .nonmonotonicTime)
    var gap = TAEMCourseProgress(frame: frame, state: sample(0, tick: 0),
      maximumSampleInterval: c.guidance.maximumSnapshotSpan)
    gap.observe(sample(.pi/2, tick: 3))
    XCTAssertEqual(gap.status, .sampleGap)
    XCTAssertEqual(gap.signedChange, 0)
  }
}
