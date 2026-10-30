import Foundation

/// Native counterpart of f1eebdc flight_control_airframe_error. Build the
/// banked wind plane BEFORE applying incidence; roll about an already pitched
/// nose is a different attitude and introduces sideslip.
public enum AirRelativeAttitude {
  public static func target(position: Vector3, airVelocity: Vector3,
                            alpha: Double, bank: Double) -> Quaternion? {
    guard airVelocity.length > 1e-9, position.length > 1,
          alpha.isFinite, bank.isFinite else { return nil }
    let forwardWind = airVelocity.normalized
    var up = position.normalized.projected(onPlaneWithNormal: forwardWind)
    if up.length < 1e-8 {
      let axes = [Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)]
      let seed = axes.min { abs($0.dot(forwardWind)) < abs($1.dot(forwardWind)) }!
      up = seed.projected(onPlaneWithNormal: forwardWind)
    }
    let bankedUp = up.normalized.rotated(about: forwardWind, angle: bank)
    let forward = (forwardWind * cos(alpha) + bankedUp * sin(alpha)).normalized
    let roof = (bankedUp * cos(alpha) - forwardWind * sin(alpha)).normalized
    return .init(forward: forward, right: forward.cross(roof).normalized, down: -roof)
  }

  /// Shortest rotation vector in body FRD, radians (roll, pitch, yaw).
  public static func error(current: Quaternion, target: Quaternion) -> Vector3 {
    var q = (current.conjugate * target).normalized
    if q.w < 0 { q = .init(w: -q.w, x: -q.x, y: -q.y, z: -q.z) }
    let v = Vector3(q.x, q.y, q.z)
    guard v.length > 1e-12 else { return .zero }
    return v * (2 * atan2(v.length, max(0, q.w)) / v.length)
  }
}
