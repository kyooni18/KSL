import Foundation

/// Body FRD (forward, right, down) to right-handed planet-fixed rotation.
public struct Quaternion: Codable, Sendable, Equatable {
  public var w: Double
  public var x: Double
  public var y: Double
  public var z: Double
  public init(w: Double = 1, x: Double = 0, y: Double = 0, z: Double = 0) {
    self.w = w
    self.x = x
    self.y = y
    self.z = z
  }
  public var norm: Double { sqrt(w * w + x * x + y * y + z * z) }
  public var normalized: Self {
    let n = norm
    return n > 1e-12 ? .init(w: w / n, x: x / n, y: y / n, z: z / n) : .init()
  }
  public var conjugate: Self { .init(w: w, x: -x, y: -y, z: -z) }
  public static func * (a: Self, b: Self) -> Self {
    .init(
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w)
  }
  public func rotate(_ v: Vector3) -> Vector3 {
    let q = Vector3(x, y, z)
    let t = q.cross(v) * 2
    return v + t * w + q.cross(t)
  }
  public static func axisAngle(_ axis: Vector3, angle: Double) -> Self {
    let a = axis.normalized * sin(angle / 2)
    return .init(w: cos(angle / 2), x: a.x, y: a.y, z: a.z)
  }
  public func advanced(bodyRate: Vector3, dt: Double) -> Self {
    let speed = bodyRate.length
    return speed > 1e-12
      ? (self * .axisAngle(bodyRate / speed, angle: speed * dt)).normalized : self
  }
  public init(forward f: Vector3, right r: Vector3, down d: Vector3) {
    let trace = f.x + r.y + d.z
    if trace > 0 {
      let s = sqrt(trace + 1) * 2
      self.init(w: s / 4, x: (r.z - d.y) / s, y: (d.x - f.z) / s, z: (f.y - r.x) / s)
    } else if f.x > r.y && f.x > d.z {
      let s = sqrt(1 + f.x - r.y - d.z) * 2
      self.init(w: (r.z - d.y) / s, x: s / 4, y: (r.x + f.y) / s, z: (d.x + f.z) / s)
    } else if r.y > d.z {
      let s = sqrt(1 + r.y - f.x - d.z) * 2
      self.init(w: (d.x - f.z) / s, x: (r.x + f.y) / s, y: s / 4, z: (d.y + r.z) / s)
    } else {
      let s = sqrt(1 + d.z - f.x - r.y) * 2
      self.init(w: (f.y - r.x) / s, x: (d.x + f.z) / s, y: (d.y + r.z) / s, z: s / 4)
    }
    self = normalized
  }
  public static func windAttitude(
    position: Vector3, airVelocity: Vector3, alpha: Double, bank: Double
  ) -> Self {
    let f = airVelocity.normalized
    var r = f.cross(position.normalized).normalized
    if r.length < 0.5 { r = LocalFrame(position: position).east }
    let liftUp = r.cross(f).normalized.rotated(about: f, angle: bank)
    let forward = f * cos(alpha) + liftUp * sin(alpha)
    let up = liftUp * cos(alpha) - f * sin(alpha)
    return .init(forward: forward, right: forward.cross(up).normalized, down: -up)
  }
}
