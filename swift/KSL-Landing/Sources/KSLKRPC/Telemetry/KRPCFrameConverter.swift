import Foundation
import KSLCore

public enum KRPCFrameConverter {
  /// Angular velocity is an axial vector. Both the reference-frame reflection
  /// and the right/forward body permutation must be applied with their signs.
  public static func bodyRate(rawAngularVelocity: Vector3, rawBodyRotation: Quaternion) -> Vector3 {
    let rawBody = rawBodyRotation.conjugate.rotate(rawAngularVelocity)
    return .init(-rawBody.y, -rawBody.x, -rawBody.z)
  }
  /// kRPC local surface coordinates are (up, north, east), a left-handed basis.
  public static func surfaceVector(_ raw: Vector3, at position: Vector3) -> Vector3 {
    let local = LocalFrame(position: position)
    return local.vector(north: raw.y, east: raw.z, up: raw.x)
  }
  /// Input quaternion maps kRPC vessel (right, forward, down) to (up,north,east).
  /// Both basis permutations are applied to basis vectors, never to quaternion components.
  public static func attitude(raw: Quaternion, at position: Vector3) -> Quaternion {
    let forward = surfaceVector(raw.rotate(.init(0, 1, 0)), at: position)
    let right = surfaceVector(raw.rotate(.init(1, 0, 0)), at: position)
    let down = surfaceVector(raw.rotate(.init(0, 0, 1)), at: position)
    return .init(forward: forward, right: right, down: down)
  }
  public static func measuredBodyRate(previous: Quaternion, current: Quaternion, dt: Double)
    -> Vector3
  {
    guard dt > 0, dt <= 1 else { return .zero }
    var delta = (previous.conjugate * current).normalized
    if delta.w < 0 { delta = .init(w: -delta.w, x: -delta.x, y: -delta.y, z: -delta.z) }
    let vector = Vector3(delta.x, delta.y, delta.z)
    let n = vector.length
    guard n > 1e-10 else { return .zero }
    let angle = 2 * atan2(n, delta.w)
    return current.conjugate.rotate(previous.rotate(vector * (angle / (n * dt))))
  }
}
