import KSLCore
import XCTest

@testable import KSLPhysics

final class PhysicsTests: XCTestCase {
  func testStockPressureAndVacuum() throws {
    let a = try AtmosphericModelEngine()
    XCTAssertEqual(a.sample(altitude: 0).pressure, 101325, accuracy: 1e-8)
    XCTAssertEqual(a.sample(altitude: 26977.92).pressure, 690.5929, accuracy: 1e-5)
    XCTAssertEqual(a.sample(altitude: 70_000).density, 0)
    XCTAssertGreaterThan(a.sample(altitude: 69_000).density, 0)
    XCTAssertNotEqual(
      a.sample(altitude: 1000, latitude: 0).temperature,
      a.sample(altitude: 1000, latitude: radians(70)).temperature)
  }
  func testPolarUsesCorrectSTSData() throws {
    let a = try AerodynamicsEngine()
    let c = a.coefficients(mach: 0, alpha: radians(8))
    XCTAssertEqual(c.lift, 0.35142547, accuracy: 1e-8)
    XCTAssertEqual(c.drag, 0.11302716, accuracy: 1e-8)
  }
  func testAeroDragRemovesEnergy() throws {
    let a = try AerodynamicsEngine()
    let atm = AtmosphericState(density: 1, pressure: 101000, temperature: 288, speedOfSound: 340)
    let pos = Vector3(600_000, 0, 0)
    let velocity = Vector3(0, 200, 0)
    let q = Quaternion.windAttitude(
      position: pos, airVelocity: velocity, alpha: radians(10), bank: radians(35))
    let result = a.evaluate(position: pos, velocity: velocity, attitude: q, atmosphere: atm)
    XCTAssertEqual(result.force.dot(velocity), -result.drag * 200, accuracy: 1e-6)
    XCTAssertEqual(degrees(result.bank), 35, accuracy: 1e-8)
  }
  func testVacuumOrbitRK4() throws {
    var planet = PlanetConfiguration()
    planet.rotationRate = 0
    let a = try AtmosphericModelEngine(planet: planet)
    let aero = try AerodynamicsEngine()
    let dynamics = TranslationalDynamicsEngine(planet: planet, atmosphere: a, aerodynamics: aero)
    let radius = 800_000.0
    let speed = sqrt(planet.gravitationalParameter / radius)
    var s = KinematicState(position: .init(radius, 0, 0), velocity: .init(0, speed, 0))
    let energy0 = s.velocity.squaredLength / 2 - planet.gravitationalParameter / s.position.length
    for i in 0..<1000 {
      s = dynamics.step(s, time: Double(i), mass: 25_000, attitude: .init(), dt: 1)
    }
    let energy = s.velocity.squaredLength / 2 - planet.gravitationalParameter / s.position.length
    XCTAssertEqual((energy - energy0) / abs(energy0), 0, accuracy: 1e-9)
    XCTAssertEqual(s.position.length, radius, accuracy: 0.01)
  }
  func testNoArtificialVacuumControl() throws {
    let c = MissionConfiguration()
    let physics = ShuttlePhysicsEngine(
      configuration: c, atmosphere: try .init(), aerodynamics: try .init())
    let pos = Vector3(680_000, 0, 0)
    let vel = Vector3(0, 2200, 0)
    let q = Quaternion.windAttitude(position: pos, airVelocity: vel, alpha: 0, bank: 0)
    let s = VehicleState(time: 0, position: pos, velocity: vel, attitude: q)
    let result = try physics.advance(s, command: .init(pitch: 1, roll: 1), duration: 1)
    XCTAssertEqual(result.angularVelocity.length, 0, accuracy: 1e-14)
    XCTAssertEqual(result.attitude.w, q.w, accuracy: 1e-14)
  }
  func testMalformedAeroTableRejected() {
    XCTAssertThrowsError(try AerodynamicsEngine(csv: "mach,alpha,cl,cd\n0,0,1,0.1\n0,0,2,0.2\n"))
  }
  func testVacuumAngularMomentumInRotatingCoordinates() throws {
    let c = MissionConfiguration()
    let engine = ShuttlePhysicsEngine(
      configuration: c, atmosphere: try .init(), aerodynamics: try .init())
    let position = Vector3(800000, 0, 0)
    let velocity = Vector3(0, 2000, 0)
    let initial = VehicleState(
      time: 0, position: position, velocity: velocity,
      attitude: .windAttitude(
        position: position, airVelocity: velocity, alpha: radians(12), bank: radians(35)),
      angularVelocity: .init(0.01, 0.02, 0.03))
    func momentum(_ state: VehicleState) -> Vector3 {
      let inertialRate =
        state.angularVelocity + state.attitude.conjugate.rotate(c.planet.angularVelocity)
      let fixed = state.attitude.rotate(c.vehicle.inertia.multiplied(componentwise: inertialRate))
      return fixed.rotated(about: .init(0, 0, 1), angle: c.planet.rotationRate * state.time)
    }
    let initialH = momentum(initial)
    let result = try engine.advance(initial, command: .init(), duration: 100, maximumStep: 0.02)
    XCTAssertLessThan((momentum(result) - initialH).length / initialH.length, 1e-5)
  }

  func testRejectedHandoffRetainsControlDependentForecast() throws {
    let c = MissionConfiguration()
    let atmosphere = try AtmosphericModelEngine()
    let aero = try AerodynamicsEngine()
    let engine = TrajectoryPropagationEngine(
      configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let position = Vector3(615000, 0, 0)
    let velocity = Vector3(-80, 480, 0)
    let initial = VehicleState(
      time: 0, position: position, velocity: velocity,
      attitude: .windAttitude(position: position, airVelocity: velocity, alpha: radians(8), bank: 0)
    )
    let left = engine.predict(from: initial, duration: 100, minimumDuration: 20) { _ in
      .init(alpha: radians(8), bank: radians(-45))
    }
    let right = engine.predict(from: initial, duration: 100, minimumDuration: 20) { _ in
      .init(alpha: radians(8), bank: radians(45))
    }
    XCTAssertTrue(left.reachedTerminalRegion)
    XCTAssertTrue(right.reachedTerminalRegion)
    XCTAssertGreaterThanOrEqual(left.duration, 20)
    XCTAssertGreaterThan((left.terminalState.position - right.terminalState.position).length, 100)
  }
}
