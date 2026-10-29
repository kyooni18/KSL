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
  public let planet: PlanetConfiguration
  public let densityScale: Double
  public let windENU: Vector3
  private let pressure: HermiteCurve
  private let temperature: HermiteCurve
  private let sunMultiplier: HermiteCurve
  private let latitudeBias: HermiteCurve
  private let latitudeSun: HermiteCurve
  public init(
    planet: PlanetConfiguration = .init(), densityScale: Double = 1, windENU: Vector3 = .zero
  ) throws {
    self.planet = planet
    self.densityScale = densityScale
    self.windENU = windENU
    guard densityScale.isFinite, densityScale > 0, windENU.isFinite,
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
      speedOfSound: sqrt(1.39999997615814 * gas * t),
      wind: local.vector(north: windENU.y, east: windENU.x, up: windENU.z))
  }
}
