import KSLCore
import KSLGuidance
import XCTest

final class DeorbitWindowDiversityTests: XCTestCase {
  func testPolar100kmCandidatesIncludeDistinctOrbitalPasses() throws {
    var c = MissionConfiguration()
    c.runway.elevation = 70
    var deorbit = c.deorbitSettings
    deorbit.candidateBudget = 12
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
    // The first six are seeds. Sample both timing directions on each of
    // multiple passages before spending budget on larger offsets.
    XCTAssertGreaterThanOrEqual(result.candidates.count, 8)
    if result.candidates.count >= 8 {
      let seeds = Array(result.candidates.prefix(6))
      let offsets = Array(result.candidates.dropFirst(6))
      let pairedSeeds = seeds.filter { seed in
        offsets.contains { earlier in
          earlier.burnStart < seed.burnStart && offsets.contains { later in
            later.burnStart > seed.burnStart
              && abs((seed.burnStart-earlier.burnStart) - (later.burnStart-seed.burnStart)) < 0.01
          }
        }
      }
      XCTAssertFalse(pairedSeeds.isEmpty, "A sampled passage must include earlier and later burns")
      let samples = max(361, Int(ceil(result.horizon / deorbit.coarseWindowStep)) + 1)
      let halfGrid = result.horizon / Double(samples - 1) / 2
      XCTAssertTrue(seeds.contains { seed in
        offsets.contains { abs(abs($0.burnStart - seed.burnStart) - halfGrid) < 0.01 }
      }, "The first local timing probes must resolve half the coarse grid")
    }
  }
}
