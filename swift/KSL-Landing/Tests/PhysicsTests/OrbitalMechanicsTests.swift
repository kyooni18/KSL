import Foundation
import KSLCore
import KSLPhysics
import XCTest

final class OrbitalMechanicsTests: XCTestCase {
  func testInertialConversionAndOneOrbitConservation() throws {
    let p = PlanetConfiguration(), orbit = OrbitalMechanics(planet: .init())
    let radius = p.radius+100000, speed = sqrt(p.gravitationalParameter/radius)
    let fixed = VehicleState(time: 196000, position: .init(radius, 0, 0),
      velocity: .init(0, speed-p.rotationRate*radius, 0), attitude: .init())
    let start = OrbitalState(fixed: fixed, planet: p)
    let roundtrip = start.planetFixed(planet: p, attitude: .init())
    XCTAssertLessThan((roundtrip.position-fixed.position).length, 1e-8)
    XCTAssertLessThan((roundtrip.velocity-fixed.velocity).length, 1e-9)
    let elements = try orbit.elements(start)
    XCTAssertEqual(elements.periapsisAltitude, 100000, accuracy: 0.1)
    let final = try orbit.propagate(start, to: start.time+XCTUnwrap(elements.period))
    XCTAssertLessThan((final.position-start.position).length, 0.1)
    XCTAssertEqual(try orbit.elements(final).specificEnergy, elements.specificEnergy, accuracy: 0.01)
  }
  func testFiniteBurnConsumesMeasuredPropellantAndFindsDescendingInterface() throws {
    let p = PlanetConfiguration(), orbit = OrbitalMechanics(planet: .init())
    let radius = p.radius+100000
    let state = OrbitalState(time: 0, position: .init(radius, 0, 0),
      velocity: .init(0, sqrt(p.gravitationalParameter/radius), 0), mass: 25000)
    let propulsion = PropulsionTelemetry(availableThrust: 60000, dryMass: 22000, specificImpulse: 316)
    let result = try FiniteBurnModel(planet: p).propagate(from: state, deltaV: 60,
      propulsion: propulsion, settings: .init())
    XCTAssertTrue(result.complete)
    XCTAssertLessThan(result.state.mass, state.mass)
    XCTAssertGreaterThan(result.state.mass, propulsion.dryMass)
    XCTAssertEqual(result.deliveredDeltaV, 60, accuracy: BurnThrottleLaw.terminalDeltaVTolerance)
    let entry = try XCTUnwrap(orbit.entryInterface(from: result.state, altitude: 69500, maximumDuration: 3000))
    XCTAssertEqual(entry.position.length-p.radius, 69500, accuracy: 0.0001)
    XCTAssertLessThan(entry.position.dot(entry.velocity), 0)
  }
  func testCThrottleRampAndCutoffContracts() {
    XCTAssertEqual(BurnThrottleLaw.fraction(elapsed: 0, remaining: 40, maximumAcceleration: 0.5, ramp: 12), 0)
    XCTAssertEqual(BurnThrottleLaw.fraction(elapsed: 6, remaining: 40, maximumAcceleration: 0.5, ramp: 12), 0.5)
    XCTAssertEqual(BurnThrottleLaw.fraction(elapsed: 20, remaining: 0.25, maximumAcceleration: 0.5, ramp: 12), 0.5)
    XCTAssertEqual(BurnThrottleLaw.fraction(elapsed: 20, remaining: 0.08, maximumAcceleration: 0.5, ramp: 12), 0)
  }
}
