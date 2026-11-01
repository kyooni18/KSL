import KSLCore
import KSLPhysics
import XCTest
@testable import KSLGuidance

final class CaptureFailureRegressionTests: XCTestCase {
  func testEnvelopeCountsCandidatesInsteadOfRepeatedStationViolations() {
    let lift = ConstraintViolation(.insufficientLift, "unavailable lift")
    let exhausted = ConstraintViolation(.insufficientEnergy, "energy exhausted")
    let assessment = TerminalAssessment(feasible: false,
      violations: [lift, lift, lift, exhausted], touchdownSpeed: 0,
      minimumLiftMargin: -1, maximumBank: 0, maximumQ: 0, maximumLoad: 0,
      score: 0, uncertaintyFraction: 0)
    var summary = TerminalEnvelopeSummary()
    summary.record(length: 1000, assessment: assessment, completed: false)
    XCTAssertEqual(summary.examined, 1)
    XCTAssertEqual(summary.violationCounts[FailureClassification.insufficientLift.rawValue], 1)
    XCTAssertEqual(summary.violationCounts[FailureClassification.insufficientEnergy.rawValue], 1)
  }

  func testUniversalActuatorAndEnergyFailuresDoNotProveAnEnergyOnlyRemedy() {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let state = VehicleState(time: 1, position: frame.position(along: -50_000, cross: 0, height: 3400),
      velocity: .init(0, 45, 0), attitude: .init())
    // A candidate summary counts each physical violation once per path.
    // When every path also fails lift/trim, the energy label by itself cannot
    // authorize a signed drag correction. The other cause must be reported.
    var summary = TerminalEnvelopeSummary()
    summary.examined = 35
    summary.violationCounts = [FailureClassification.insufficientEnergy.rawValue: 35,
      FailureClassification.insufficientLift.rawValue: 35,
      FailureClassification.terrainCollision.rawValue: 35,
      FailureClassification.actuatorAuthority.rawValue: 35]
    let result = PlanningResult(plan: nil, bestAdvisory: nil, evaluatedCandidates: 35,
      advisoryCounts: [:], envelope: summary)
    let assessment = TAEMCaptureEvaluator.assess(result, state: state, configuration: c)
    XCTAssertEqual(assessment.deficit, .inadequateControlAuthority)
    XCTAssertFalse(assessment.feasible)
    summary.violationCounts = [FailureClassification.insufficientEnergy.rawValue: 35]
    let purelyEnergy = PlanningResult(plan: nil, bestAdvisory: nil, evaluatedCandidates: 35,
      advisoryCounts: [:], envelope: summary)
    XCTAssertEqual(TAEMCaptureEvaluator.assess(purelyEnergy, state: state,
      configuration: c).deficit, .insufficientEnergy)
  }

  func testVacuumEntryPreparesConfiguredAttitudeWithoutInventingLift() throws {
    let c = MissionConfiguration()
    var aero = AerodynamicState()
    aero.airspeed = 2100; aero.angleOfAttack = radians(2); aero.measuredForces = true
    let position = Vector3(c.planet.radius + c.planet.atmosphereTop + 1000, 0, 0)
    let velocity = Vector3(-60, 2100, 0)
    let state = VehicleState(time: 1, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: aero.angleOfAttack, bank: 0),
      aerodynamics: aero)
    let law = EntryCommandLaw(configuration: c, atmosphere: try .init(), aerodynamics: try .init())
    let target = law.update(state)
    XCTAssertGreaterThan(target.angleOfAttack, aero.angleOfAttack)
    XCTAssertLessThanOrEqual(target.angleOfAttack, c.entrySettings.nominalAlpha)
    XCTAssertEqual(law.diagnostics.selectedDragAcceleration, 0)
    XCTAssertEqual(law.diagnostics.energyAllocationMode, "preentry-attitude")
  }
}
