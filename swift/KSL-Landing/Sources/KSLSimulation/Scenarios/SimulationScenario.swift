import Foundation
import KSLCore
import KSLPhysics

public struct SimulationScenario: Codable, Sendable {
  public var name: String
  public var configuration: MissionConfiguration
  public var initialState: VehicleState
  public var maximumDuration: Double
  public init(
    name: String, configuration: MissionConfiguration, initialState: VehicleState,
    maximumDuration: Double = 1800
  ) {
    self.name = name
    self.configuration = configuration
    self.initialState = initialState
    self.maximumDuration = maximumDuration
  }
  public static func approach(
    name: String = "nominal-final", along: Double = -10_000, cross: Double = 0,
    height: Double = 4200,
    speed: Double = 150, flightPath: Double = -24, headingError: Double = 0
  ) -> Self {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: along, cross: cross, height: height)
    let forward = frame.direction(
      at: .init(along, cross),
      tangent: .init(cos(radians(headingError)), sin(radians(headingError))))
    let velocity =
      (forward * cos(radians(flightPath)) + position.normalized * sin(radians(flightPath))) * speed
    let attitude = Quaternion.windAttitude(
      position: position, airVelocity: velocity, alpha: radians(8), bank: 0)
    return .init(
      name: name, configuration: c,
      initialState: .init(time: 1000, position: position, velocity: velocity, attitude: attitude),
      maximumDuration: 400)
  }
  public static func entry(
    name: String = "nominal-entry", along: Double = -450_000, cross: Double = 0,
    headingError: Double = 0, speed: Double = 1900, height: Double = 72_000, flightPath: Double = -3
  ) -> Self {
    let c = MissionConfiguration()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: along, cross: cross, height: height)
    let target = frame.position(along: -20_000, cross: 0, height: 0)
    let forward = target.normalized.projected(onPlaneWithNormal: position.normalized).normalized
      .rotated(about: position.normalized, angle: -radians(headingError))
    let velocity =
      (forward * cos(radians(flightPath)) + position.normalized * sin(radians(flightPath))) * speed
    let attitude = Quaternion.windAttitude(
      position: position, airVelocity: velocity, alpha: radians(1.4), bank: 0)
    return .init(
      name: name, configuration: c,
      initialState: .init(
        time: 188_839, position: position, velocity: velocity, attitude: attitude),
      maximumDuration: 2000)
  }
  public static func loadINI(url: URL) throws -> Self {
    let text = try String(contentsOf: url, encoding: .utf8)
    var values: [String: String] = [:]
    for raw in text.split(whereSeparator: \.isNewline) {
      let line = raw.trimmingCharacters(in: .whitespaces)
      if line.hasPrefix("#") || !line.contains("=") { continue }
      let pair = line.split(separator: "=", maxSplits: 1).map {
        String($0).trimmingCharacters(in: .whitespaces)
      }
      if pair.count == 2 { values[pair[0]] = pair[1] }
    }
    func number(_ key: String, _ fallback: Double? = nil) throws -> Double {
      if let s = values[key], let x = Double(s), x.isFinite { return x }
      if let fallback { return fallback }
      throw FlightError.invalidConfiguration("Missing/invalid INI key \(key)")
    }
    guard try number("deorbit_delta_v_mps", 0) == 0 else {
      throw FlightError.invalidConfiguration(
        "Scenario includes a deorbit burn; landing runtime begins after deorbit")
    }
    var c = MissionConfiguration()
    c.runway.latitude = radians(try number("runway_latitude_deg", degrees(c.runway.latitude)))
    c.runway.longitude = radians(try number("runway_longitude_deg", degrees(c.runway.longitude)))
    c.runway.elevation = try number("runway_elevation_m", c.runway.elevation)
    c.runway.heading = radians(try number("runway_heading_deg", 90))
    c.runway.length = try number("runway_length_m", 2500)
    c.runway.width = try number("runway_width_m", 70)
    let time = try number("ut0", 0)
    let mass = try number("mass_kg", 25_000)
    var position: Vector3
    var velocity: Vector3
    if values["position_x_m"] != nil {
      let inertialPosition = try Vector3(
        number("position_x_m"), number("position_y_m"), number("position_z_m"))
      let inertialVelocity = try Vector3(
        number("velocity_x_mps"), number("velocity_y_mps"), number("velocity_z_mps"))
      let phase = .pi / 2 + c.planet.rotationRate * time
      position = inertialPosition.rotated(about: .init(0, 0, 1), angle: -phase)
      velocity =
        inertialVelocity.rotated(about: .init(0, 0, 1), angle: -phase)
        - c.planet.angularVelocity.cross(position)
    } else {
      position = try GeodeticPosition(
        latitude: radians(number("latitude_deg")), longitude: radians(number("longitude_deg")),
        altitude: number("altitude_m")
      ).cartesian(radius: c.planet.radius)
      let local = LocalFrame(position: position)
      let heading = radians(try number("heading_deg"))
      let gamma = radians(try number("flight_path_angle_deg"))
      let speed = try number("surface_speed_mps")
      velocity = local.vector(
        north: cos(heading) * cos(gamma) * speed, east: sin(heading) * cos(gamma) * speed,
        up: sin(gamma) * speed)
    }
    let attitude = Quaternion.windAttitude(
      position: position, airVelocity: velocity, alpha: radians(try number("initial_aoa_deg", 0)),
      bank: radians(try number("initial_bank_deg", 0)))
    return .init(
      name: values["name"] ?? url.deletingPathExtension().lastPathComponent, configuration: c,
      initialState: .init(
        time: time, position: position, velocity: velocity, attitude: attitude, mass: mass),
      maximumDuration: 2200)
  }
}
