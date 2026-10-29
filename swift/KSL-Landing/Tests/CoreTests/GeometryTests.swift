import XCTest

@testable import KSLCore

final class GeometryTests: XCTestCase {
  func testSphericalRunwayRoundTrip() {
    let f = RunwayFrame(runway: .init())
    for p in [
      Vector2(-8000, 0), .init(-80_000, 40_000), .init(-1_050_000, 300_000), .init(2400, -25),
    ] {
      let r = f.coordinates(of: f.position(along: p.x, cross: p.y, height: 25_000))
      XCTAssertEqual(r.along, p.x, accuracy: 1e-6)
      XCTAssertEqual(r.cross, p.y, accuracy: 1e-6)
      XCTAssertEqual(r.height, 25_000, accuracy: 1e-6)
    }
  }
  func testWindAttitudeRoundTrip() {
    let position = Vector3(600_000, 0, 0)
    let velocity = Vector3(-10, 200, 30)
    for alpha in [-5.0, 0, 15, 35] {
      for bank in [-60.0, 0, 45] {
        let q = Quaternion.windAttitude(
          position: position, airVelocity: velocity, alpha: radians(alpha), bank: radians(bank))
        let body = q.conjugate.rotate(velocity.normalized)
        XCTAssertEqual(degrees(atan2(body.z, body.x)), alpha, accuracy: 1e-8)
        XCTAssertEqual(body.y, 0, accuracy: 1e-10)
        XCTAssertEqual(q.norm, 1, accuracy: 1e-12)
      }
    }
  }
  func testNoAirborneBrakes() {
    XCTAssertThrowsError(try ControlCommand(wheelBrake: 1).validate(grounded: false))
  }
}
