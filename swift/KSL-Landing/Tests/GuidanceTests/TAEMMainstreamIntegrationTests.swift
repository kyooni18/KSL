import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMMainstreamIntegrationTests: XCTestCase {
  private struct Recorded: Decodable { let configuration: MissionConfiguration, state: VehicleState }
  func testDefaultMissionIssuesCoupledRecoveryAndDefaultWorkerRunsCommandSearch() async throws {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    let r = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    let c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true, initialPhase: .acquisition)
    let command = try mission.update(r.state)
    XCTAssertEqual(command.phase, .acquisition)
    XCTAssertEqual(command.commandRecovery?.mode, "local-coupled-no-complete-forecast")
    XCTAssertNotNil(mission.taemStatus.forceFit, "The actual mission call must reach the coupled polar fitter")
    XCTAssertLessThanOrEqual(abs(command.target.bank), c.vehicle.maximumBank)
    XCTAssertLessThanOrEqual(command.target.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    let request = try XCTUnwrap(mission.planningRequest(from: r.state, controls: nil,
      allowEntry: false, allowTerminal: true))
    XCTAssertTrue(request.taemRecovery)
    XCTAssertTrue(request.terminalOnly().taemRecovery)
    let worker = MissionPlanningWorker(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let result = await worker.plan(request.terminalOnly())
    let recovery = try XCTUnwrap(result.recovery)
    XCTAssertNotNil(result.terminal, "Command searches also refresh advisory geometry when complete forecasts are unavailable")
    XCTAssertFalse(recovery.trials.isEmpty)
    XCTAssertNil(recovery.candidate, "Missing original moment evidence must not manufacture a complete command forecast")
    mission.accept(result, state: r.state)
    XCTAssertEqual(mission.lastRecoveryProposal?.id, recovery.id)
    var next = r.state
    next.time += c.guidance.controlInterval
    let issued = try mission.update(next)
    XCTAssertEqual(issued.commandRecovery?.proposalID, recovery.id)
    XCTAssertEqual(issued.commandRecovery?.evaluatedCandidates, recovery.trials.count)
    XCTAssertFalse(issued.commandRecovery?.forecastComplete ?? true)
    let decoded = try JSONDecoder().decode(MissionPlanningProposal.self,
      from: JSONEncoder().encode(result))
    XCTAssertEqual(decoded.recovery?.id, recovery.id)
  }
  func testScheduleCursorUsesMeasuredEpochAndDoesNotLoseTerminalHandoff() throws {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-response-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    let r = try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
    let c = r.configuration, dt = c.guidance.controlInterval
    let left = GuidanceTarget(angleOfAttack: r.state.aerodynamics.angleOfAttack, bank: -c.vehicle.maximumBank)
    let right = GuidanceTarget(angleOfAttack: r.state.aerodynamics.angleOfAttack, bank: c.vehicle.maximumBank)
    let candidate = TAEMCommandCandidate(id: "cursor-contract", topology: .leftRight, segments: [
      .init(duration: dt, command: .init(phase: .acquisition, target: left)),
      .init(duration: dt, command: .init(phase: .acquisition, target: right))])
    let proposal = TAEMRecoveryProposal(id: "cursor-contract", time: r.state.time, candidate: candidate,
      outcome: nil, continuation: nil, reference: nil, trials: [], searchEnd: .familyExhausted)
    var cursor = TAEMCommandCursor(proposal: proposal, state: r.state, configuration: c)
    XCTAssertEqual(cursor.update(r.state), left)
    var next = r.state
    next.time += dt
    XCTAssertEqual(cursor.update(next), right)
    XCTAssertFalse(cursor.finished)
    XCTAssertEqual(cursor.tail?.count, 1)
    next.time += dt
    XCTAssertNil(cursor.update(next))
    XCTAssertTrue(cursor.finished)
    XCTAssertNil(cursor.tail)
  }

}
