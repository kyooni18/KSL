import Foundation
import KSLCore

public struct LegacyImportStatistics: Encodable, Sendable {
  public var lines = 0
  public var malformedLines = 0
  public var sequenceGaps = 0
  public var orphanDeltas = 0
  public var duplicateTimes = 0
  public var invalidSnapshots = 0
  public var snapshots = 0
  public var sessions: [String] = []
  public var phases: [String: Int] = [:]
  public let evidence = "archived telemetry; flight provenance must be checked separately"
  public init() {}
}

public struct LegacySnapshot: Codable, Sendable {
  public let session: String
  public let line: Int
  public let originalPhase: String
  public let state: VehicleState
  public let limitations: [String]
}

/// Bounded streaming decoder for archival schema 5. A corrupt line invalidates
/// every delta chain because its session cannot be trusted. Only a new complete
/// keyframe can resume that session; fields are never merged across sessions.
public final class LegacyTelemetryReader {
  private struct Chain {
    var fields: [String: Any]
    var sequence: Int
    var lastTime: Double?
  }
  private let reader: JSONLReader
  private let configuration: MissionConfiguration
  private let skipMalformed: Bool
  private let selectedSession: String?
  private var chains: [String: Chain] = [:]
  public private(set) var statistics = LegacyImportStatistics()

  public init(
    url: URL, configuration: MissionConfiguration = .init(),
    skipMalformed: Bool = false, session: String? = nil
  ) throws {
    reader = try JSONLReader(url: url)
    self.configuration = configuration
    self.skipMalformed = skipMalformed
    selectedSession = session
  }

  public func next() throws -> LegacySnapshot? {
    while let data = try reader.nextLine() {
      statistics.lines += 1
      let record: [String: Any]
      do {
        guard let object = try JSONSerialization.jsonObject(with: data) as? [String: Any],
          object["schemaVersion"] as? Int == 5,
          let session = object["sessionId"] as? String, !session.isEmpty
        else {
          throw FlightError.invalidTelemetry("Unsupported archival record")
        }
        record = object
      } catch {
        chains.removeAll(keepingCapacity: true)
        statistics.malformedLines += 1
        if !skipMalformed {
          throw FlightError.invalidTelemetry(
            "Malformed archival line \(reader.lineNumber); delta chains invalidated")
        }
        continue
      }
      let session = record["sessionId"] as! String
      if !statistics.sessions.contains(session) {
        guard statistics.sessions.count < 32 else {
          throw FlightError.invalidTelemetry("Too many sessions in one archival file")
        }
        statistics.sessions.append(session)
      }
      if let selectedSession, session != selectedSession { continue }
      guard let type = record["recordType"] as? String,
        type == "vehicleKeyframe" || type == "vehicleDelta",
        let sequence = record["recordSequence"] as? Int,
        let fields = record["fields"] as? [String: Any]
      else { continue }
      if type == "vehicleKeyframe" {
        chains[session] = Chain(
          fields: fields, sequence: sequence, lastTime: chains[session]?.lastTime)
      } else {
        guard var chain = chains[session] else {
          statistics.orphanDeltas += 1
          continue
        }
        guard sequence == chain.sequence + 1 else {
          chains[session] = nil
          statistics.sequenceGaps += 1
          continue
        }
        chain.fields = Self.merge(chain.fields, fields)
        chain.sequence = sequence
        chains[session] = chain
      }
      guard var chain = chains[session], let time = record["ut"] as? Double, time > 0 else {
        continue
      }
      if let previous = chain.lastTime, time <= previous {
        statistics.duplicateTimes += 1
        continue
      }
      do {
        let state = try decode(chain.fields, time: time)
        try state.validate(configuration: configuration)
        let phase = (chain.fields["state"] as? [String: Any])?["phase"] as? String ?? "unknown"
        statistics.snapshots += 1
        statistics.phases[phase, default: 0] += 1
        chain.lastTime = time
        chains[session] = chain
        return LegacySnapshot(
          session: session, line: reader.lineNumber, originalPhase: phase, state: state,
          limitations: [
            "No live-flight claim from filenames", "Wind assumed zero",
            "Attitude reconstructed from logged aerodynamic angles",
            "Missing body rates default to zero; use as replay initial conditions, not identified actuator truth",
          ])
      } catch {
        statistics.invalidSnapshots += 1
        // Keep a structurally valid chain so a later delta can supply a
        // missing measurement, but do not emit a fabricated snapshot.
      }
    }
    return nil
  }

  private static func merge(_ previous: [String: Any], _ delta: [String: Any]) -> [String: Any] {
    var result = previous
    for (key, value) in delta {
      if let nested = value as? [String: Any], let old = result[key] as? [String: Any] {
        result[key] = merge(old, nested)
      } else {
        result[key] = value
      }
    }
    return result
  }

  private func decode(_ fields: [String: Any], time: Double) throws -> VehicleState {
    func group(_ name: String) throws -> [String: Any] {
      guard let value = fields[name] as? [String: Any] else {
        throw FlightError.invalidTelemetry("Missing \(name)")
      }
      return value
    }
    func number(_ group: [String: Any], _ key: String) throws -> Double {
      guard let value = group[key] as? Double, value.isFinite else {
        throw FlightError.invalidTelemetry("Missing/nonfinite \(key)")
      }
      return value
    }
    func vector(_ group: [String: Any], _ key: String) throws -> Vector3 {
      guard let a = group[key] as? [Double], a.count == 3, a.allSatisfy(\.isFinite) else {
        throw FlightError.invalidTelemetry("Missing \(key) vector")
      }
      return .init(a[0], a[1], a[2])
    }
    let p = try group("position")
    let m = try group("motion")
    let a = try group("attitude")
    let aero = try group("aero")
    let vehicle = try group("vehicle")
    let flags = try group("state")
    let latitude = radians(try number(p, "latitude"))
    let longitude = radians(try number(p, "longitude"))
    let position = GeodeticPosition(
      latitude: latitude, longitude: longitude, altitude: try number(p, "altitude")
    ).cartesian(radius: configuration.planet.radius)
    let inertialPosition = try vector(p, "inertial")
    let inertialVelocity = try vector(m, "velocity")
    // Recover the archival inertial-to-fixed phase from the two measured
    // positions. This avoids assuming that all sessions share one epoch.
    let phase = atan2(inertialPosition.y, inertialPosition.x) - longitude
    guard abs(inertialPosition.length - position.length) < 50 else {
      throw FlightError.invalidTelemetry("Inconsistent inertial/geographic positions")
    }
    let velocity =
      inertialVelocity.rotated(about: .init(0, 0, 1), angle: -phase)
      - configuration.planet.angularVelocity.cross(position)
    var aerodynamic = AerodynamicState()
    aerodynamic.airspeed = try number(m, "trueAirSpeed")
    guard abs(velocity.length - aerodynamic.airspeed) < max(5, 0.03 * aerodynamic.airspeed) else {
      throw FlightError.invalidTelemetry("Inconsistent fixed velocity and airspeed")
    }
    aerodynamic.mach = try number(aero, "mach")
    aerodynamic.dynamicPressure = try number(aero, "dynamicPressure")
    aerodynamic.angleOfAttack = radians(try number(a, "angleOfAttack"))
    aerodynamic.bank = radians(try number(a, "roll"))
    aerodynamic.sideslip = radians(try number(a, "sideslip"))
    aerodynamic.lift = try number(aero, "liftForce")
    aerodynamic.drag = try number(aero, "dragForce")
    // Logged force values may be modeled. Never label them measured merely
    // because they were imported from a historical file.
    aerodynamic.measuredForces = false
    let density = try number(aero, "density")
    let pressure = try number(aero, "staticPressure")
    let sound = aerodynamic.mach > 1e-8 ? aerodynamic.airspeed / aerodynamic.mach : 340
    let temperature =
      density > 1e-12 ? pressure / (287.05 * density) : sound * sound / (1.4 * 287.05)
    let atmosphere = AtmosphericState(
      density: density, pressure: pressure, temperature: temperature, speedOfSound: sound)
    var systems = SystemState()
    systems.gearDown = flags["gear"] as? Bool ?? false
    systems.brakes = flags["brakes"] as? Bool ?? false
    systems.sas = flags["sasEnabled"] as? Bool ?? false
    systems.rcs = flags["rcsEnabled"] as? Bool ?? false
    systems.airbrakes = flags["airbrakes"] as? Bool ?? false
    systems.thrust = try number(vehicle, "currentThrust")
    systems.throttle =
      (fields["appliedControl"] as? [String: Any])?["reportedThrottle"] as? Double ?? 0
    let situation = flags["vesselSituation"] as? String ?? "unknown"
    let contact: ContactState =
      situation == "landed" ? .rolling : situation == "splashed" ? .crashed : .airborne
    let rates = Vector3(
      radians(a["bodyRollRate"] as? Double ?? 0), radians(a["bodyPitchRate"] as? Double ?? 0),
      radians(a["bodyYawRate"] as? Double ?? 0))
    return VehicleState(
      time: time, position: position, velocity: velocity,
      attitude: .windAttitude(
        position: position, airVelocity: velocity, alpha: aerodynamic.angleOfAttack,
        bank: aerodynamic.bank),
      angularVelocity: rates, mass: try number(vehicle, "mass"), atmosphere: atmosphere,
      aerodynamics: aerodynamic, contact: contact, systems: systems,
      radarAltitude: try number(p, "radarAltitude"))
  }

  public func close() throws {
    try reader.close()
    chains.removeAll()
  }
}
