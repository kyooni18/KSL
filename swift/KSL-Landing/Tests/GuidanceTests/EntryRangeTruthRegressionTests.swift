import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

/// Range scales a reference profile only; it never changes runway geometry.
final class EntryRangeTruthRegressionTests: XCTestCase {
  func testDragReferenceScaleCannotShortenPhysicalRange() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine(), aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: -330_000, cross: 80_000, height: 45_000)
    let velocity = frame.direction(at: .init(-330_000, 80_000), tangent: .init(1, 0))*1800
    let state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero).observe(.init(time: 200, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity,
        alpha: radians(24), bank: 0)))
    let objective = EntryCaptureObjective.seed(configuration: c, atmosphere: atmosphere,
      frame: frame, crossAim: 0, time: state.time)
    let target = frame.position(along: objective.along, cross: objective.cross, height: 0)
    let distance = frame.distance(from: position, to: target)
    let drag = EntryDragReference(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let a = drag.compute(state, phase: .equilibriumGlide, range: distance,
      referenceRangeScale: 1, objective: objective, maximumAlpha: c.vehicle.maximumAngleOfAttack)
    let b = drag.compute(state, phase: .equilibriumGlide, range: distance,
      referenceRangeScale: 0.8, objective: objective, maximumAlpha: c.vehicle.maximumAngleOfAttack)
    XCTAssertEqual(a.requiredRange, distance, accuracy: 1e-6)
    XCTAssertEqual(b.requiredRange, distance, accuracy: 1e-6)
    XCTAssertEqual(try XCTUnwrap(a.referenceTargetRange), distance, accuracy: 1e-6)
    XCTAssertEqual(try XCTUnwrap(b.referenceTargetRange), distance*0.8, accuracy: 1e-6)
    XCTAssertTrue(b.predictedRange.isFinite)
    XCTAssertEqual(b.rangeError, b.predictedRange-distance*0.8, accuracy: 1e-6)
    let lawFull = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let lawShort = EntryCommandLaw(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    _ = lawFull.update(state, rangeMultiplier: 1)
    _ = lawShort.update(state, rangeMultiplier: 0.8)
    XCTAssertEqual(lawFull.diagnostics.requiredRange, lawShort.diagnostics.requiredRange,
      accuracy: 1e-6)
  }

  func testOnlyUnanimousExplicitEnergyDeficitsJustifyAReferenceCorrection() {
    XCTAssertEqual(EntryGuidanceEngine.unanimousEnergyDeficit(
      [.insufficientEnergy, .insufficientEnergy]), .insufficientEnergy)
    XCTAssertEqual(EntryGuidanceEngine.unanimousEnergyDeficit(
      [.excessiveEnergy, .excessiveEnergy]), .excessiveEnergy)
    XCTAssertNil(EntryGuidanceEngine.unanimousEnergyDeficit(
      [.terminalRegionNotReached, .insufficientEnergy]))
    XCTAssertNil(EntryGuidanceEngine.unanimousEnergyDeficit(
      [.crossrangeBeyondTurningAuthority, .insufficientEnergy]))
    XCTAssertNil(EntryGuidanceEngine.unanimousEnergyDeficit(
      [.insufficientEnergy, .excessiveEnergy]))
    XCTAssertNil(EntryGuidanceEngine.unanimousEnergyDeficit(
      [.insufficientAltitude, .insufficientEnergy]))
    XCTAssertNil(EntryGuidanceEngine.unanimousEnergyDeficit([]))
  }

  func testTerminalEnergyHeightMatchesRotatingFrameEffectivePotential() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let engine = EnergyManagementEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero)
    let rest = VehicleState(time: 0, position: frame.origin, velocity: .zero,
      attitude: .init())
    XCTAssertEqual(engine.energyHeight(rest), 0, accuracy: 1e-9)
    let p = frame.position(along: -60_000, cross: 22_000, height: 25_000)
    let v = Vector3(450, 400, 150)
    let state = VehicleState(time: 100, position: p, velocity: v, attitude: .init())
    let mu = c.planet.gravitationalParameter
    let rotation = c.planet.angularVelocity
    let expected = (v.squaredLength/2 + mu*(1/frame.origin.length - 1/p.length)
      - (rotation.cross(p).squaredLength - rotation.cross(frame.origin).squaredLength)/2)
      / (mu/frame.origin.squaredLength)
    XCTAssertEqual(engine.energyHeight(state), expected, accuracy: 1e-9)
    var zeroRotation = c
    zeroRotation.planet.rotationRate = 0
    let inertialEngine = EnergyManagementEngine(configuration: zeroRotation,
      atmosphere: atmosphere, aerodynamics: aero)
    let inertial = inertialEngine.energyHeight(state)
    XCTAssertGreaterThan(abs(engine.energyHeight(state)-inertial), 1e-3)
  }

  func testNoCompletedTerminalRouteCannotTriggerSignedEnergyCorrection() {
    var low = TAEMCaptureAssessment()
    low.deficit = .insufficientEnergy
    var excessive = TAEMCaptureAssessment()
    excessive.deficit = .excessiveEnergy
    // The 72-km reproduction had terminal failures at low altitude, but
    // no completed HAC. The label alone may not consume further energy.
    XCTAssertNil(EntryGuidanceEngine.supportedEnergyCorrection([low, low]))
    low.envelope.candidatesCompleted = 2
    XCTAssertNil(EntryGuidanceEngine.supportedEnergyCorrection(
      [low, TAEMCaptureAssessment()]))
    XCTAssertEqual(EntryGuidanceEngine.supportedEnergyCorrection([low, low]),
      .insufficientEnergy)
    excessive.envelope.candidatesCompleted = 2
    XCTAssertEqual(EntryGuidanceEngine.supportedEnergyCorrection([excessive, excessive]),
      .excessiveEnergy)
    XCTAssertNil(EntryGuidanceEngine.supportedEnergyCorrection([low, excessive]))
    XCTAssertNil(EntryGuidanceEngine.supportedEnergyCorrection([]))
  }

  func testEnergyDeficitChoosesTheCorrectDragDirection() {
    let factors = [0.8, 1.0, 1.2, 1.44]
    XCTAssertEqual(EntryGuidanceEngine.recoveryReferenceScale(
      for: .insufficientEnergy, candidates: factors), 1.44)
    XCTAssertEqual(EntryGuidanceEngine.recoveryReferenceScale(
      for: .excessiveEnergy, candidates: factors), 0.8)
    XCTAssertEqual(EntryGuidanceEngine.recoveryReferenceScale(
      for: .terminalRegionNotReached, candidates: factors), 1.0)
    XCTAssertNil(EntryGuidanceEngine.recoveryReferenceScale(
      for: .crossrangeBeyondTurningAuthority, candidates: factors))
    XCTAssertNil(EntryGuidanceEngine.recoveryReferenceScale(
      for: .insufficientEnergy, candidates: []))
    XCTAssertNil(EntryGuidanceEngine.recoveryReferenceScale(
      for: .insufficientEnergy, candidates: [.nan, -1]))
  }

  func testUnclassifiedTerminalRegionDoesNotProveExcessEnergy() {
    XCTAssertTrue(EntryGuidanceEngine.holdNeutralReference(for: [.terminalRegionNotReached]))
    XCTAssertTrue(EntryGuidanceEngine.holdNeutralReference(for:
      [.terminalRegionNotReached, .terminalRegionNotReached]))
    XCTAssertFalse(EntryGuidanceEngine.holdNeutralReference(for: [.excessiveEnergy]))
    XCTAssertFalse(EntryGuidanceEngine.holdNeutralReference(for: [.insufficientEnergy]))
    XCTAssertFalse(EntryGuidanceEngine.holdNeutralReference(for:
      [.terminalRegionNotReached, .crossrangeBeyondTurningAuthority]))
    XCTAssertFalse(EntryGuidanceEngine.holdNeutralReference(for: []))
  }
}
