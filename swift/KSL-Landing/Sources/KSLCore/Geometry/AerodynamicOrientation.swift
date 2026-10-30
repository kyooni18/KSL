import Foundation

public enum AerodynamicOrientation {
  /// Reconstruct attitude from aerodynamic coordinates when the independent
  /// simulator explicitly marks its body quaternion unavailable. Each of alpha,
  /// beta and wind-axis bank is preserved; no body Euler bank substitution.
  public static func reconstruct(position: Vector3, velocity: Vector3,
    alpha: Double, beta: Double, bank: Double) -> Quaternion? {
    let flow = velocity.normalized
    let right = flow.cross(position.normalized).normalized
    guard flow.length > 0.5, right.length > 0.5,
      [alpha,beta,bank].allSatisfy(\.isFinite) else { return nil }
    let up = right.cross(flow).normalized
    let alongDown = cos(beta)*sin(alpha)
    let bodyUp = -flow*alongDown + (up*cos(bank)+right*sin(bank))*sqrt(max(0,1-alongDown*alongDown))
    let down = -bodyUp
    let projected = flow-down*alongDown
    let length = projected.length
    guard length > 1e-9 else { return nil }
    let e = projected/length, t = down.cross(e)
    let forward = e*(cos(beta)*cos(alpha)/length)-t*(sin(beta)/length)
    let bodyRight = down.cross(forward)
    return .init(forward: forward, right: bodyRight, down: down)
  }
}
