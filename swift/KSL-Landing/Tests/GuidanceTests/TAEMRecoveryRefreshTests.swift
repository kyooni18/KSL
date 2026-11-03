import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class TAEMRecoveryRefreshTests: XCTestCase {
  struct Sample: Decodable { let state: VehicleState; let guidance: GuidanceSolution }
  struct Recorded: Decodable { let configuration: MissionConfiguration; let samples: [Sample] }
  private func recorded() throws -> Recorded {
    let url = try XCTUnwrap(Bundle.module.url(forResource: "taem-frozen-recovery-20261103",
      withExtension: "json", subdirectory: "Fixtures"))
    return try JSONDecoder().decode(Recorded.self, from: Data(contentsOf: url))
  }

  func testRecordedUnattainableTurnCannotSpendReservedRecoverySupport() throws {
    let r = try recorded(), c = r.configuration, s = r.samples[1].state
    let demand = try XCTUnwrap(r.samples[1].guidance.commandRecovery?.requestedForce)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let unconstrained = try XCTUnwrap(TAEMNormalForceFitter.fit(vertical: demand.x, lateral: demand.y,
      state: s, configuration: c, aerodynamics: aero, maximumAlpha: c.vehicle.maximumAngleOfAttack,
      baseTarget: r.samples[1].guidance.target))
    let recovery = try XCTUnwrap(TAEMNormalForceFitter.fit(vertical: demand.x, lateral: demand.y,
      state: s, configuration: c, aerodynamics: aero, maximumAlpha: c.vehicle.maximumAngleOfAttack,
      minimumVertical: max(0, demand.x), baseTarget: r.samples[1].guidance.target))
    let peak = aero.liftSolution(acceleration: .greatestFiniteMagnitude, mass: s.mass,
      dynamicPressure: s.aerodynamics.dynamicPressure, mach: s.aerodynamics.mach,
      minimumAlpha: c.vehicle.minimumAngleOfAttack, maximumAlpha: c.vehicle.maximumAngleOfAttack,
      beta: s.aerodynamics.sideslip, gear: s.systems.gearDown, brakes: s.systems.brakes).maximumAcceleration
    let reserve = min(demand.x, peak, c.vehicle.maximumLoadFactor*9.80665)
    XCTAssertGreaterThan(recovery.modeled.x, unconstrained.modeled.x)
    XCTAssertGreaterThanOrEqual(recovery.modeled.x, reserve-1e-10)
    XCTAssertEqual(recovery.requested, demand, "The original unattainable request remains visible")
    XCTAssertLessThanOrEqual(abs(recovery.target.bank), c.vehicle.maximumBank)
    XCTAssertLessThanOrEqual(recovery.target.angleOfAttack, c.vehicle.maximumAngleOfAttack)
    let polar = aero.coefficients(mach: s.aerodynamics.mach, alpha: recovery.target.angleOfAttack,
      dynamicPressure: s.aerodynamics.dynamicPressure, beta: s.aerodynamics.sideslip,
      gear: s.systems.gearDown, brakes: s.systems.brakes,
      airbrakes: s.systems.airbrakes, speedbrakeFraction: s.systems.speedbrakeFraction ?? 0)
    let lift = polar.lift*s.aerodynamics.dynamicPressure*aero.referenceArea/s.mass
    XCTAssertEqual(recovery.modeled.x, lift*cos(recovery.target.bank), accuracy: 1e-10)
    XCTAssertEqual(recovery.modeled.y, lift*sin(recovery.target.bank), accuracy: 1e-10)
    print("RECORDED_RECOVERY_SUPPORT", "requested", demand, "joint", unconstrained.modeled,
      "reserved", recovery.modeled, "bank", degrees(recovery.target.bank))
  }

  func testMM305WorkerRefreshesGeometryFromEachRecordedStateWithoutCertifyingIt() async throws {
    let r = try recorded(), c = r.configuration
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let worker = MissionPlanningWorker(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    var previous: TerminalPlan?
    for sample in r.samples {
      let s = sample.state
      var context = HACFitContext(); context.previousSpiral = previous?.spiral
      context.previousPlanID = previous?.identifier
      let request = MissionPlanningRequest(time: s.time, entry: nil, terminalState: s,
        controls: nil, previousTarget: sample.guidance.target, hac: context, taemRecovery: true)
      let proposal = await worker.plan(request)
      XCTAssertNotNil(proposal.recovery)
      let fresh = try XCTUnwrap(proposal.terminal?.plan)
      XCTAssertEqual(fresh.createdAt, s.time)
      XCTAssertNotEqual(fresh.identifier, previous?.identifier)
      if let previous, fresh.assessment.airborneFeasible != true,
        previous.assessment.airborneFeasible != true {
        XCTAssertEqual(fresh.spiral?.side, previous.spiral?.side,
          "Equally unqualified regeneration preserves the chosen winding")
      }
      let position = RunwayFrame(runway: c.runway, planet: c.planet).coordinates(of: s.position)
      XCTAssertEqual(try XCTUnwrap(fresh.stations.first).position.x, position.along, accuracy: 1e-6)
      XCTAssertEqual(try XCTUnwrap(fresh.stations.first).position.y, position.cross, accuracy: 1e-6)
      // These are separate matched-state command checks, not a trajectory
      // replay with an invented twenty-second gap in controller history.
      let mission = LandingMissionController(configuration: c, atmosphere: atmosphere,
        aerodynamics: aero, externalPlanning: true, initialPhase: .acquisition)
      mission.accept(proposal, state: s)
      let solution = try mission.update(s)
      XCTAssertEqual(mission.advisoryPlan?.identifier, fresh.identifier)
      XCTAssertEqual(solution.commandRecovery?.referencePlanID, fresh.identifier)
      XCTAssertEqual(solution.commandRecovery?.forecastComplete, false)
      XCTAssertNil(mission.plan, "A fresh advisory cannot manufacture complete controller coverage")
      previous = fresh
    }
  }
}
