import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMAdvisoryInletTests: XCTestCase {
  struct Sample: Decodable { let state: VehicleState; let guidance: GuidanceSolution }
  struct Recorded: Decodable {
    let configuration: MissionConfiguration, sample: Sample, oldAdvisory: TerminalPlan
    let observedHandoff: VehicleState
  }
  private func recorded() throws -> Recorded {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-mm304-inlet-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    return try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
  }
  func testEntryAdvisoryStartsAtPredictedOwnershipEndWithoutEarlyControlTransfer() async throws {
    let r = try recorded(), c = r.configuration, s = r.sample.state
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero, externalPlanning: true)
    // Transport regression: a genuine later observed MM304 boundary is
    // supplied as historical endpoint evidence. This tests wiring, not a
    // claim that the unsupported native plant predicted this flight.
    let observed = r.observedHandoff
    let q = RunwayFrame(runway: c.runway, planet: c.planet).coordinates(of: observed.position)
    var capture = EntryCaptureForecast(createdAt: s.time, rangeMultiplier: 1,
      reachedTerminalRegion: true, violations: [], terminalTime: observed.time,
      terminalAltitude: q.height, terminalMach: observed.aerodynamics.mach,
      terminalAlong: q.along, terminalCross: q.cross,
      maximumDynamicPressure: observed.aerodynamics.dynamicPressure,
      maximumLoad: observed.aerodynamics.lift/observed.mass/9.80665,
      capture: .init(), track: [])
    capture.advisoryInlet = .init(sourceTime: s.time, state: observed, model: "recorded-MM304-boundary")
    let entry = EntryPlanningProposal(createdAt: s.time, rangeMultiplier: 1, prediction: nil,
      candidates: 0, terminalRouteFeasible: false, capture: capture)
    mission.accept(.init(time: s.time, entry: entry, terminal: nil), state: s)
    _ = try mission.update(s)
    XCTAssertEqual(mission.phase, .entry)
    let request = try XCTUnwrap(mission.planningRequest(from: s, controls: nil,
      allowEntry: false, allowTerminal: true))
    let inlet = try XCTUnwrap(request.terminalInlet)
    let end = try XCTUnwrap(request.terminalState)
    XCTAssertGreaterThan(end.time, s.time)
    XCTAssertEqual(inlet.state.position, end.position)
    XCTAssertEqual(inlet.state.velocity, end.velocity)
    XCTAssertEqual(inlet.sourceTime, s.time)
    XCTAssertEqual(inlet.model, "recorded-MM304-boundary")
    XCTAssertEqual(request.terminalOnly().terminalInlet?.state.time, end.time)
    XCTAssertNil(request.entryOnly().terminalInlet)
    let worker = MissionPlanningWorker(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let proposal = await worker.plan(request)
    let plan = try XCTUnwrap(proposal.terminal?.plan)
    let p = RunwayFrame(runway: c.runway, planet: c.planet).coordinates(of: end.position)
    let first = try XCTUnwrap(plan.stations.first)
    XCTAssertEqual(plan.createdAt, end.time)
    XCTAssertEqual(first.position.x, p.along, accuracy: 1e-6)
    XCTAssertEqual(first.position.y, p.cross, accuracy: 1e-6)
    XCTAssertEqual(first.height, p.height, accuracy: 1e-6)
    XCTAssertEqual(first.speed, end.aerodynamics.airspeed, accuracy: 1e-6)
    let tangent = RunwayFrame(runway: c.runway, planet: c.planet)
      .coordinateVelocity(at: end.position, velocity: end.velocity).normalized
    XCTAssertEqual(first.tangent.x, tangent.x, accuracy: 1e-10)
    XCTAssertEqual(first.tangent.y, tangent.y, accuracy: 1e-10)
    XCTAssertNotEqual(first.position, RunwayFrame(runway: c.runway, planet: c.planet)
      .coordinates(of: s.position).horizontal)
    mission.accept(proposal, state: s)
    XCTAssertEqual(mission.advisoryPlan?.identifier, plan.identifier)
    XCTAssertNil(mission.plan)
    XCTAssertNil(mission.lastAdmission)
    XCTAssertEqual(mission.phase, .entry)
    mission.invalidateDynamics(at: s)
    XCTAssertNil(mission.advisoryPlan, "Plant revision invalidates the future inlet and its path")
    XCTAssertNil(mission.planningRequest(from: s, controls: nil,
      allowEntry: false, allowTerminal: true)?.terminalInlet)
    let decoded = try JSONDecoder().decode(MissionPlanningProposal.self,
      from: JSONEncoder().encode(proposal))
    XCTAssertEqual(decoded.terminalInlet?.state.time, end.time)
    print("MM304_ADVISORY_INLET", "sourceUT", s.time, "handoffUT", end.time,
      "height", p.height, "speed", end.aerodynamics.airspeed,
      "glideDeparture", HACAdvisoryPreference.glideDeparture(plan,
        maximumGlide: c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle))
  }

  func testNormalGlidePreferenceAndNoFreeReversalWithinUnqualifiedEvidence() throws {
    let r = try recorded(), c = r.configuration
    let limit = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
    let old = r.oldAdvisory
    let rollRate = c.guidance.terminalRollRate ?? c.guidance.bankSlewRate
    XCTAssertGreaterThan(HACAdvisoryPreference.glideDeparture(old, maximumGlide: limit), 0)
    // Analytic normal-glide geometry checks ordering only, not flight
    // performance. No vehicle/aero/actuator capability is changed.
    let height = try XCTUnwrap(old.stations.first).height
    let endHeight = try XCTUnwrap(old.nominalEnergyPoint).height
    let distance = (height-endHeight)/tan(limit)
    let stations = [
      PathStation(distance: 0, position: .init(-distance, 0), tangent: .init(1, 0),
        curvature: 0, height: height, slope: -tan(limit), segment: .acquisition),
      PathStation(distance: distance, position: .init(0, 0), tangent: .init(1, 0),
        curvature: 0, height: endHeight, slope: -tan(limit), segment: .spiral)]
    let normal = TerminalPlan(identifier: 1, createdAt: old.createdAt, spiral: old.spiral,
      stations: stations, assessment: old.assessment)
    XCTAssertLessThan(HACAdvisoryPreference.glideDeparture(normal, maximumGlide: limit),
      HACAdvisoryPreference.glideDeparture(old, maximumGlide: limit))
    XCTAssertEqual(HACAdvisoryPreference.prefers(normal, over: old,
      incumbentSide: old.spiral?.side, maximumGlide: limit, rollRate: rollRate), true)
    let spiral = try XCTUnwrap(old.spiral)
    let reversed = try SpiralHAC(exitAlong: spiral.exitAlong, finalRadius: spiral.finalRadius,
      growth: spiral.growth, turnAngle: spiral.turnAngle, side: -spiral.side)
    let other = TerminalPlan(identifier: 2, createdAt: normal.createdAt, spiral: reversed,
      stations: normal.stations, assessment: normal.assessment)
    XCTAssertEqual(HACAdvisoryPreference.prefers(normal, over: other,
      incumbentSide: spiral.side, maximumGlide: limit, rollRate: rollRate), true)
    XCTAssertEqual(HACAdvisoryPreference.prefers(other, over: normal,
      incumbentSide: spiral.side, maximumGlide: limit, rollRate: rollRate), false)
  }

  func testRollReversalAndLargeBankRankBehindGentleTurnOnSameGlide() throws {
    let r = try recorded(), c = r.configuration
    let limit = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
    let rollRate = c.guidance.terminalRollRate ?? c.guidance.bankSlewRate
    let old = r.oldAdvisory
    func plan(_ id: Int, _ banks: [Double]) -> TerminalPlan {
      let stations = banks.enumerated().map { i, bank in
        PathStation(distance: Double(i)*1000, position: .init(Double(i)*1000, 0), tangent: .init(1, 0),
          curvature: 0, height: 8000-Double(i)*1000*tan(limit), slope: -tan(limit),
          speed: 150, bank: bank, segment: .spiral)
      }
      return TerminalPlan(identifier: id, createdAt: old.createdAt, spiral: old.spiral,
        stations: stations, assessment: old.assessment)
    }
    let gentle = plan(1, [0.3, 0.3, 0.3, 0.3])
    let reversing = plan(2, [0.3, -0.3, 0.3, -0.3])
    let steep = plan(3, [1.1, 1.1, 1.1, 1.1])
    for radical in [reversing, steep] {
      XCTAssertEqual(HACAdvisoryPreference.prefers(gentle, over: radical,
        incumbentSide: old.spiral?.side, maximumGlide: limit, rollRate: rollRate), true)
    }
  }
}
