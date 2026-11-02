import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class MissionPlanningSplitTests: XCTestCase {
  func testApproachRequestSurvivesTerminalWorkerSplit() {
    let state = VehicleState(time: 100, position: .init(600_100, 0, 0),
      velocity: .init(0, 150, -2), attitude: .init())
    let flare = FlareReference(latched: false, arcActive: false, wheelHeight: 100,
      desiredSink: 2, upwardAcceleration: 0, triggerHeight: 20,
      oneGravitySpeed: 120, availablePull: 2, designAcceleration: 1)
    let approach = ApproachPlanningRequest(state: state, flare: flare,
      maximumAlpha: .pi/6, lift: 1, drag: 1)
    let request = MissionPlanningRequest(time: state.time, entry: nil,
      terminalState: nil, controls: nil, previousTarget: nil, approach: approach)
    let work = request.terminalOnly()
    XCTAssertNotNil(work.approach)
    XCTAssertNil(work.entry)
    XCTAssertNil(work.terminalState)
    XCTAssertNil(request.entryOnly().approach)
  }

  func testUnrelatedStateAndDynamicsRevisionAreRetainedBySplit() {
    let state = VehicleState(time: 100, position: .init(600_100, 0, 0),
      velocity: .init(0, 150, -2), attitude: .init())
    var request = MissionPlanningRequest(time: state.time, entry: nil,
      terminalState: state, controls: nil, previousTarget: nil)
    request.dynamicsRevision = 6
    let c = MissionConfiguration(), target = GuidanceTarget(angleOfAttack: 0.1, bank: 0.2)
    var limiter = GuidanceTargetLimiter(previous: .init())
    _ = limiter.update(target, phase: .entry, configuration: c, dt: 0.1, dynamicPressure: 3000)
    request.targetLimiter = limiter
    let expected = limiter.update(target, phase: .entry, configuration: c, dt: 0.1, dynamicPressure: 3000)
    for split in [request.entryOnly(), request.terminalOnly()] {
      var copy = split.targetLimiter
      XCTAssertEqual(copy?.update(target, phase: .entry, configuration: c, dt: 0.1,
        dynamicPressure: 3000), expected)
    }
    XCTAssertNotNil(request.terminalOnly().terminalState)
    XCTAssertEqual(request.terminalOnly().dynamicsRevision, 6)
    XCTAssertNil(request.entryOnly().terminalState)
    XCTAssertEqual(request.entryOnly().dynamicsRevision, 6)
  }
}
