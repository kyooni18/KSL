import Foundation
import KSLCore
import XCTest
final class AerodynamicOrientationTests: XCTestCase {
  func testAlphaBetaAndWindBankPreservedTogether() throws {
    let position = Vector3(600000,0,0), velocity = Vector3(-25,1000,40)
    let flow = velocity.normalized, right = flow.cross(position.normalized).normalized
    let up = right.cross(flow).normalized
    for alpha in [-5.0,0,20,35] { for beta in [-8.0,0,8] { for bank in [-70.0,0,70] {
      let q = try XCTUnwrap(AerodynamicOrientation.reconstruct(position: position,velocity: velocity,
        alpha: radians(alpha),beta: radians(beta),bank: radians(bank)))
      let body = q.conjugate.rotate(flow), roof = q.rotate(.init(0,0,-1))
      XCTAssertEqual(degrees(atan2(body.z,body.x)),alpha,accuracy:1e-8)
      XCTAssertEqual(degrees(asin(body.y)),beta,accuracy:1e-8)
      XCTAssertEqual(degrees(atan2(roof.dot(right),roof.dot(up))),bank,accuracy:1e-8)
    } } }
  }
}
