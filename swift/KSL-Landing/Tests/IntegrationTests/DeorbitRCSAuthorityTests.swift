import KSLCore
import KSLGuidance
import KSLPhysics
@testable import KSLRuntime
import XCTest

final class DeorbitRCSAuthorityTests: XCTestCase {
  func testOrbitalRCSTorqueDoesNotCarryIntoUnpoweredEntry() throws {
    let c = MissionConfiguration(), p = c.planet, radius = p.radius + 100_000
    var source = VehicleState(time: 0, position: .init(radius, 0, 0),
      velocity: .init(0, sqrt(p.gravitationalParameter / radius) - p.rotationRate * radius, 0),
      attitude: .init(forward: .init(0, -1, 0), right: .init(0, 0, 1), down: .init(-1, 0, 0)))
    source.controlAuthority = .init(positiveTorque: .init(10000, 10000, 10000),
      negativeTorque: .init(-10000, -10000, -10000), inertia: .init(100000, 100000, 100000))
    let propulsion = PropulsionTelemetry(availableThrust: 60000, dryMass: 22000, specificImpulse: 316)
    source.propulsion = propulsion; source.systems.rcs = true
    let window = try DeorbitWindowSearch(configuration: c).probe(
      initial: .init(fixed: source, planet: p), start: 60, normal: 0, propulsion: propulsion)
    let model = OrbitalExecutionModel(configuration: c)
    XCTAssertThrowsError(try model.predict(source: source, window: window,
      propulsion: propulsion))
    source.controlAuthority?.rcsExcludedPositiveTorque = .init(8000, 8000, 8000)
    source.controlAuthority?.rcsExcludedNegativeTorque = .init(-8000, -8000, -8000)
    let entry = try model.predict(source: source, window: window,
      propulsion: propulsion).entry
    XCTAssertFalse(entry.systems.rcs)
    XCTAssertEqual(entry.controlAuthority?.positiveTorque.x, 8000)
    XCTAssertEqual(entry.controlAuthority?.negativeTorque.y, -8000)
  }
}
