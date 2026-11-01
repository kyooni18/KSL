import XCTest
@testable import KSLCore

final class RunwayApproachSelectorTests: XCTestCase {
  func testAutomaticSelectionPrefersInboundDirectionAndIsSymmetric() {
    var c = MissionConfiguration()
    let f = RunwayFrame(runway: c.runway, planet: c.planet)
    // A vessel west of KSC heading east toward 09, but opposite to 27.
    let p = f.position(along: -160_000, cross: 0, height: 40_000)
    let east = f.direction(at: .init(-160_000, 0), tangent: .init(1, 0))
    let state = VehicleState(time: 10, position: p,
      velocity: east*1800-p.normalized*60, attitude: .init())
    XCTAssertTrue(RunwayApproachSelector.choose(state, configuration: c).runway.name.hasSuffix("09"))
    // Runway 27 requires an approach from the east, not a westbound
    // vehicle that has ALREADY passed west of both runway thresholds.
    let pEast = f.position(along: 160_000, cross: 0, height: 40_000)
    let west = -f.direction(at: .init(160_000, 0), tangent: .init(1, 0))
    let reverse = VehicleState(time: 10, position: pEast,
      velocity: west*1800-pEast.normalized*60, attitude: .init())
    XCTAssertTrue(RunwayApproachSelector.choose(reverse, configuration: c).runway.name.hasSuffix("27"))

    // Runway choice must be independent of which end was supplied first.
    let reversedConfig = RunwayApproachSelector.candidates(configuration: c)[1]
    XCTAssertEqual(RunwayApproachSelector.choose(state, configuration: reversedConfig).runway.name,
      RunwayApproachSelector.choose(state, configuration: c).runway.name)
    XCTAssertEqual(RunwayApproachSelector.choose(reverse, configuration: reversedConfig).runway.name,
      RunwayApproachSelector.choose(reverse, configuration: c).runway.name)

    // Both orientations refer to the SAME 2500m physical runway.
    let twice = reversedConfig.runway.reciprocal(planet: c.planet)
    XCTAssertEqual(twice.name, c.runway.name)
    XCTAssertEqual(twice.length, c.runway.length)
    let roundTripFrame = RunwayFrame(runway: twice, planet: c.planet)
    XCTAssertLessThan((roundTripFrame.origin-f.origin).length, 0.01)
  }

  func testSelectedRunwayIsConsistentWithMeasuredVelocityFrames() {
    var c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let p = frame.position(along: -160_000, cross: 65_000, height: 35_000)
    let velocity = frame.direction(at: .init(-160_000, 65_000), tangent: .init(1, 0))*1600
      - p.normalized*80
    let state = VehicleState(time: 11, position: p, velocity: velocity, attitude: .init())
    let assessments = RunwayApproachSelector.candidates(configuration: c)
      .map { RunwayApproachSelector.assess(state, configuration: $0) }
    XCTAssertTrue(assessments.allSatisfy {
      $0.pathDistance.isFinite && $0.turnToInlet.isFinite
        && $0.turnIntoFinal.isFinite
        && $0.minimumHeadingTravel >= 0
    })
    XCTAssertEqual(RunwayApproachSelector.choose(state, configuration: c).runway.name,
      assessments.min { $0.minimumHeadingTravel < $1.minimumHeadingTravel }!.runway.name)
  }
}
