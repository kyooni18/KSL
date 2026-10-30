import KSLCore
import KSLGuidance
import XCTest

final class DeorbitWindowDiversityTests: XCTestCase {
  func testPolar100kmCandidatesIncludeDistinctOrbitalPasses() throws {
    var c = MissionConfiguration()
    c.runway.elevation = 70
    var deorbit = c.deorbitSettings
    deorbit.candidateBudget = 8
    c.deorbit = deorbit
    // ShuttleSim orbit_scenarios.py 100x100 km polar fixture. Its inertial
    // coordinates have a +pi/2 origin relative to the native Swift frame.
    let time = 10_000.0
    let phase = Double.pi / 2 + c.planet.rotationRate*time
    let z = Vector3(0, 0, 1)
    let position = Vector3(490_974.0265938786, 202_128.6145457036, 456_167.21538491553)
      .rotated(about: z, angle: -phase)
    let velocity = Vector3(1353.5207153359622, 557.2296132399498, -1703.7076299728599)
      .rotated(about: z, angle: -phase) - c.planet.angularVelocity.cross(position)
    let state = VehicleState(time: time, position: position, velocity: velocity,
      attitude: .init(), mass: 26146.85)
    let propulsion = PropulsionTelemetry(availableThrust: 60_000, dryMass: 21_000,
      specificImpulse: 285, evidence: .provisional)
    let result = try DeorbitWindowSearch(configuration: c).search(from: state, propulsion: propulsion)
    XCTAssertGreaterThanOrEqual(result.candidates.count, 3)
    let passages = Set(result.candidates.map { Int($0.burnStart / 120) })
    XCTAssertGreaterThanOrEqual(passages.count, 3)
    XCTAssertTrue(result.candidates.allSatisfy { $0.entryInterface.time > $0.afterBurn.time })
    XCTAssertTrue(result.candidates.allSatisfy { $0.deltaV > 0 && $0.crossrange >= 0 })
  }
}
