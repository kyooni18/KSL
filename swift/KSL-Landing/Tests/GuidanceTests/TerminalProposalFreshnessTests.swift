import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TerminalProposalFreshnessTests: XCTestCase {
  func testExpiredAndOlderEntryProposalsCannotReplaceCurrentAim() throws {
    let c = MissionConfiguration()
    let entry = EntryGuidanceEngine(configuration: c, atmosphere: try AtmosphericModelEngine(),
      aerodynamics: try AerodynamicsEngine())
    func proposal(_ time: Double, _ aim: Double) -> EntryPlanningProposal {
      .init(createdAt: time, rangeMultiplier: 1, prediction: nil, candidates: 1,
        terminalRouteFeasible: false, crossAim: aim)
    }
    entry.accept(proposal(100 - 2*c.guidance.entryReplanInterval - 1, 1), at: 100)
    XCTAssertEqual(entry.crossAim, 0)
    entry.accept(proposal(99, 2), at: 100)
    entry.accept(proposal(98, 3), at: 100)
    XCTAssertEqual(entry.crossAim, 2)
    entry.accept(proposal(100, 4), at: 100)
    XCTAssertEqual(entry.crossAim, 4)
  }

  func testExpiredAndOlderTerminalProposalsCannotReplaceCurrentVerdict() throws {
    let c = MissionConfiguration(), atmosphere = try AtmosphericModelEngine()
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: try AerodynamicsEngine(), externalPlanning: true)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let state = VehicleState(time: 100, position: frame.position(along: -60_000, cross: 0, height: 25_000),
      velocity: frame.direction(at: .init(-60_000, 0), tangent: .init(1, 0))*700,
      attitude: .init())
    func proposal(_ time: Double, _ count: Int) -> MissionPlanningProposal {
      let result = PlanningResult(plan: nil, bestAdvisory: nil, evaluatedCandidates: count,
        advisoryCounts: [:])
      return .init(time: time, entry: nil, terminal: result)
    }
    mission.accept(proposal(100 - 2*c.guidance.terminalReplanInterval - 1, 1), state: state)
    XCTAssertNil(mission.lastPlanningResult)
    mission.accept(proposal(99, 2), state: state)
    XCTAssertEqual(mission.lastPlanningResult?.evaluatedCandidates, 2)
    mission.accept(proposal(98, 3), state: state)
    XCTAssertEqual(mission.lastPlanningResult?.evaluatedCandidates, 2)
    mission.accept(proposal(100, 4), state: state)
    XCTAssertEqual(mission.lastPlanningResult?.evaluatedCandidates, 4)
  }
}
