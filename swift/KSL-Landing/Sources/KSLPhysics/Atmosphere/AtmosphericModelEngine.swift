import Foundation
import KSLCore

public struct CurveKey: Sendable {
  public let x: Double, y: Double, incoming: Double, outgoing: Double
}

/// Cubic Hermite interpolation, preserving stock KSP FloatCurve tangents.
public struct HermiteCurve: Sendable {
  public let keys: [CurveKey]
  public init(rows: [[Double]]) throws {
    guard rows.count >= 2, rows.allSatisfy({ $0.count == 4 && $0.allSatisfy(\.isFinite) }),
      zip(rows, rows.dropFirst()).allSatisfy({ $0[0] < $1[0] })
    else { throw FlightError.invalidConfiguration("Invalid atmosphere FloatCurve") }
    keys = rows.map { .init(x: $0[0], y: $0[1], incoming: $0[2], outgoing: $0[3]) }
  }
  /// Exact derivative of the same Hermite interpolant used by `value`.
  public func derivative(at x: Double) -> Double {
    guard x > keys[0].x, x < keys[keys.count - 1].x else { return 0 }
    var low = 0, high = keys.count - 1
    while high - low > 1 {
      let mid = (low + high) / 2
      if keys[mid].x <= x { low = mid } else { high = mid }
    }
    let a = keys[low], b = keys[high], dx = b.x - a.x
    let t = (x - a.x) / dx
    return (6*t*t - 6*t) * a.y / dx + (3*t*t - 4*t + 1) * a.outgoing
      + (-6*t*t + 6*t) * b.y / dx + (3*t*t - 2*t) * b.incoming
  }
  public func value(at x: Double) -> Double {
    if x <= keys[0].x { return keys[0].y }
    if x >= keys[keys.count - 1].x { return keys[keys.count - 1].y }
    var low = 0
    var high = keys.count - 1
    while high - low > 1 {
      let mid = (low + high) / 2
      if keys[mid].x <= x { low = mid } else { high = mid }
    }
    let a = keys[low]
    let b = keys[high]
    let dx = b.x - a.x
    let t = (x - a.x) / dx
    let t2 = t * t
    let t3 = t2 * t
    return (2 * t3 - 3 * t2 + 1) * a.y + (t3 - 2 * t2 + t) * dx * a.outgoing
      + (-2 * t3 + 3 * t2) * b.y + (t3 - t2) * dx * b.incoming
  }
}

public final class AtmosphericModelEngine: Sendable {
  /// Runtime identity of this immutable calibrated model, not a resource hash.
  public let instanceID = UUID()
  public let planet: PlanetConfiguration
  public let densityScale: Double
  public let soundSpeedScale: Double
  public let windENU: Vector3
  private let pressure: HermiteCurve
  private let temperature: HermiteCurve
  private let sunMultiplier: HermiteCurve
  private let latitudeBias: HermiteCurve
  private let latitudeSun: HermiteCurve
  public init(
    planet: PlanetConfiguration = .init(), densityScale: Double = 1, soundSpeedScale: Double = 1,
    windENU: Vector3 = .zero
  ) throws {
    self.planet = planet
    self.densityScale = densityScale
    self.soundSpeedScale = soundSpeedScale
    self.windENU = windENU
    guard densityScale.isFinite, densityScale > 0, soundSpeedScale.isFinite, soundSpeedScale > 0, windENU.isFinite,
      let url = Bundle.module.url(forResource: "KerbinCurves", withExtension: "json")
    else { throw FlightError.invalidConfiguration("Missing atmosphere data or invalid dispersion") }
    let curves = try JSONDecoder().decode([String: [[Double]]].self, from: Data(contentsOf: url))
    func curve(_ name: String) throws -> HermiteCurve {
      guard let rows = curves[name] else {
        throw FlightError.invalidConfiguration("Missing \(name)")
      }
      return try .init(rows: rows)
    }
    pressure = try curve("KERBIN_PRESSURE_CURVE")
    temperature = try curve("KERBIN_TEMPERATURE_CURVE")
    sunMultiplier = try curve("KERBIN_TEMPERATURE_SUN_MULT_CURVE")
    latitudeBias = try curve("KERBIN_LATITUDE_BIAS_CURVE")
    latitudeSun = try curve("KERBIN_LATITUDE_SUN_MULT_CURVE")
  }
  private init(copy: AtmosphericModelEngine, correction: EnvironmentCalibration) {
    planet = copy.planet; windENU = copy.windENU
    densityScale = copy.densityScale*clamp(correction.densityScale, 0.35, 2.5)
    soundSpeedScale = copy.soundSpeedScale*clamp(correction.soundSpeedScale, 0.7, 1.3)
    pressure = copy.pressure; temperature = copy.temperature; sunMultiplier = copy.sunMultiplier
    latitudeBias = copy.latitudeBias; latitudeSun = copy.latitudeSun
  }
  /// C vessel_physics_environment bounds extrapolation, retaining the raw
  /// measured correction in diagnostics. Curves are shared immutable values.
  public func corrected(_ correction: EnvironmentCalibration) -> AtmosphericModelEngine {
    guard correction.observedAt != nil, correction.densityScale.isFinite, correction.densityScale > 0,
      correction.soundSpeedScale.isFinite, correction.soundSpeedScale > 0 else { return self }
    return .init(copy: self, correction: correction)
  }
  /// d ln(rho) / dh (1/m), at fixed latitude, longitude and time.
  /// Includes both pressure and temperature gradients of the actual Kerbin
  /// curves; a constant exponential scale height is not assumed.
  public func logDensityAltitudeGradient(position: Vector3, time: Double) -> Double {
    let geo = GeodeticPosition(position: position, radius: planet.radius), h = geo.altitude
    guard h > 0, h < planet.atmosphereTop else { return 0 }
    let p = pressure.value(at: h)
    guard p > 0 else { return 0 }
    let solarHour = geo.longitude + (planet.rotationRate - 2 * .pi / 9_203_544.61750141) * time - 3.14000010490417
    let illumination = 0.5 * cos(solarHour - .pi / 4) + 0.5
    let bias = latitudeBias.value(at: abs(degrees(geo.latitude)))
      + latitudeSun.value(at: abs(degrees(geo.latitude))) * illumination
    let t = temperature.value(at: h) + sunMultiplier.value(at: h) * bias
    let dt = t > 1 ? temperature.derivative(at: h) + sunMultiplier.derivative(at: h) * bias : 0
    return pressure.derivative(at: h) / p - dt / max(1, t)
  }
  public func sample(position: Vector3, time: Double) -> AtmosphericState {
    let geo = GeodeticPosition(position: position, radius: planet.radius)
    return sample(
      altitude: geo.altitude, latitude: geo.latitude, longitude: geo.longitude, time: time)
  }
  public func sample(
    altitude: Double, latitude: Double = 0, longitude: Double = 0, time: Double = 0
  ) -> AtmosphericState {
    let h = clamp(altitude, 0, planet.atmosphereTop)
    // Same phase as original world.c: +pi/2 fixed->inertial cancels the solar +pi/2.
    let solarHour =
      longitude + (planet.rotationRate - 2 * .pi / 9_203_544.61750141) * time - 3.14000010490417
    let illumination = 0.5 * cos(solarHour - .pi / 4) + 0.5
    let bias =
      latitudeBias.value(at: abs(degrees(latitude))) + latitudeSun.value(at: abs(degrees(latitude)))
      * illumination
    let t = max(1, temperature.value(at: h) + sunMultiplier.value(at: h) * bias)
    let p = altitude < planet.atmosphereTop ? max(0, pressure.value(at: h) * 1000) : 0
    let gas = 8.31446261815324 / 0.0289644002914429
    let local = LocalFrame(
      position: GeodeticPosition(latitude: latitude, longitude: longitude, altitude: h).cartesian(
        radius: planet.radius))
    return .init(
      density: p / (gas * t) * densityScale, pressure: p * densityScale, temperature: t,
      speedOfSound: sqrt(1.39999997615814 * gas * t)*soundSpeedScale,
      wind: local.vector(north: windENU.y, east: windENU.x, up: windENU.z), evidence: .provisional)
  }
}
