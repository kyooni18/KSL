import KSLCore
import KSLGuidance
import KSLPhysics
@testable import KSLRuntime
import XCTest

final class DeorbitScreenExecutionTests: XCTestCase {
  func testScreenUsesControlledEntryAndRequiresMeasuredAuthority() throws {
    let c = MissionConfiguration(), p = c.planet, radius = p.radius + 100_000
    var source = VehicleState(time: 0, position: .init(radius, 0, 0),
      velocity: .init(0, sqrt(p.gravitationalParameter / radius) - p.rotationRate * radius, 0),
      attitude: .init(forward: .init(0, -1, 0), right: .init(0, 0, 1), down: .init(-1, 0, 0)))
    source.controlAuthority = .init(positiveTorque: .init(10000, 10000, 10000),
      negativeTorque: .init(-10000, -10000, -10000), inertia: .init(100000, 100000, 100000))
    let propulsion = PropulsionTelemetry(availableThrust: 60000, dryMass: 22000, specificImpulse: 316)
    source.propulsion = propulsion
    let window = try DeorbitWindowSearch(configuration: c).probe(
      initial: .init(fixed: source, planet: p), start: 60, normal: 0, propulsion: propulsion)
    let screened = try DeorbitPlanner.nominalScreenEntry(configuration: c, source: source,
      window: window, propulsion: propulsion)
    let full = try OrbitalExecutionModel(configuration: c).predict(source: source,
      window: window, propulsion: propulsion).entry
    XCTAssertEqual(screened.time, full.time, accuracy: 1e-9)
    XCTAssertEqual((screened.position-full.position).length, 0, accuracy: 1e-8)
    XCTAssertEqual(screened.mass, full.mass, accuracy: 1e-8)
    XCTAssertNotNil(screened.controlAuthority)
    XCTAssertTrue(screened.systems.unpowered)
    source.controlAuthority = nil
    XCTAssertThrowsError(try DeorbitPlanner.nominalScreenEntry(configuration: c, source: source,
      window: window, propulsion: propulsion))
  }
}
