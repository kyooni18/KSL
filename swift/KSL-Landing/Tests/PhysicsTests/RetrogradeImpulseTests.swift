import Foundation
import KSLCore
import KSLPhysics
import XCTest
final class RetrogradeImpulseTests: XCTestCase {
  func testNormalThrustImpulseIsNotRetrogradeImpulse() throws {
    let p = PlanetConfiguration(), orbit = OrbitalMechanics(planet: .init())
    var s = OrbitalState(time: 0,position: .init(700000,0,0),velocity: .init(0,sqrt(p.gravitationalParameter/700000),0),mass:25000)
    var observer = RetrogradeImpulseObserver(planet:p)
    try observer.observe(s,thrustReported:false)
    var total = 0.0
    for _ in 0..<1000 {
      s = orbit.step(s,dt:0.1) { position,velocity in
        OrbitalMechanics.burnDirection(position:position,velocity:velocity,normalAngle:radians(30))*0.5
      }
      total += 0.05
      try observer.observe(s,thrustReported:true)
    }
    XCTAssertEqual(total,50,accuracy:1e-8)
    XCTAssertEqual(observer.identifiedDeltaV,50*cos(radians(30)),accuracy:0.01)
    XCTAssertGreaterThan(total-observer.identifiedDeltaV,6)
  }
}
