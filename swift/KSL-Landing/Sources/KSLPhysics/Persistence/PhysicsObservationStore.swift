import Foundation
import CSQLite
import KSLCore

/// Schema-3 historical C archive reader. Original data/schema is never modified.
public actor PhysicsObservationStore {
  private let database: SQLiteReadConnection
  public init(path: String) throws {
    database = try .init(path: path)
    var schema: String?
    try database.query("SELECT value FROM physics_meta WHERE key='schema'") {
      schema = SQLiteReadConnection.text($0, 0)
    }
    guard schema == "3" else { throw FlightError.invalidConfiguration("Unsupported physics archive schema") }
  }
  public func contexts() throws -> [PhysicsContext] {
    var contexts: [PhysicsContext] = []
    try database.query("SELECT structure_id,environment_id,structure_json,environment_json FROM physics_contexts ORDER BY last_seen DESC") { s in
      guard let structure = SQLiteReadConnection.text(s, 0), let environment = SQLiteReadConnection.text(s, 1),
            let structureJSON = SQLiteReadConnection.text(s, 2), let environmentJSON = SQLiteReadConnection.text(s, 3) else { return }
      contexts.append(.init(structureID: structure, environmentID: environment,
                             structureJSON: structureJSON, environmentJSON: environmentJSON))
    }
    return contexts
  }
  public func counts() throws -> [String: Int] {
    var result: [String: Int] = [:]
    for table in ["aero_observations", "aero_observation_quality", "physics_sessions"] {
      try database.query("SELECT COUNT(*) FROM " + table) { result[table] = Int(sqlite3_column_int64($0, 0)) }
    }
    return result
  }
  public func history(contexts requested: [PhysicsContext], excludingSession: String,
                      excludingFlight: String, planet: PlanetConfiguration, maximum: Int = 192) throws -> PhysicsHistory {
    guard !requested.isEmpty, maximum > 0, maximum <= 192 else {
      throw FlightError.invalidConfiguration("Physics history requires explicit compatible contexts and a 1...192 sample budget")
    }
    let stored = try contexts()
    guard requested.allSatisfy({ stored.contains($0) }) else {
      throw FlightError.invalidConfiguration("Requested physics context is absent or its identity manifest differs")
    }
    guard requested.allSatisfy({ PhysicsIdentityMatcher.compatible($0, with: requested[0]) }) else {
      throw FlightError.invalidConfiguration("Physics history cannot combine incompatible contexts")
    }
    let predicates = requested.map { _ in "(a.structure_id=? AND a.environment_id=?)" }.joined(separator: " OR ")
    let sql = """
      SELECT a.id,a.structure_id,a.environment_id,a.session_id,s.flight_key,a.timeline_epoch,
        a.ut,a.q,a.mach,a.aoa,a.beta,a.roll,a.mass,a.gear,a.brakes,a.airbrakes,a.rcs,
        a.pos_x,a.pos_y,a.pos_z,a.vel_x,a.vel_y,a.vel_z,
        a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z,
        q.control_profile,q.body_pitch_rate,q.body_roll_rate,q.body_yaw_rate,q.quality,q.eligible
      FROM aero_observations a JOIN aero_observation_quality q ON q.observation_id=a.id
      LEFT JOIN physics_sessions s ON s.session_id=a.session_id
      WHERE (\(predicates)) AND a.session_id<>? AND (s.flight_key IS NULL OR s.flight_key<>?)
      ORDER BY a.id DESC LIMIT 4096
      """
    let bindings = requested.flatMap { [$0.structureID, $0.environmentID] } + [excludingSession, excludingFlight]
    var rows: [PhysicsObservation] = [], queried = 0, rejected = 0
    try database.query(sql, strings: bindings) { s in
      queried += 1
      let d = SQLiteReadConnection.number
      func v(_ i: Int32) -> Vector3 { .init(d(s, i), d(s, i + 1), d(s, i + 2)) }
      guard let structure = SQLiteReadConnection.text(s, 1), let environment = SQLiteReadConnection.text(s, 2),
            let session = SQLiteReadConnection.text(s, 3), !session.isEmpty,
            (6...12).allSatisfy({ d(s, Int32($0)).isFinite }), d(s, 7) > 0, d(s, 8) >= 0, d(s, 12) > 0,
            [17, 20, 23, 26].allSatisfy({ v(Int32($0)).isFinite }),
            [13, 14, 15].allSatisfy({ [0.0, 1.0].contains(d(s, Int32($0))) }) else { rejected += 1; return }
      let rates = Vector3(d(s, 31), d(s, 30), d(s, 32))
      rows.append(.init(id: sqlite3_column_int64(s, 0), structureID: structure, environmentID: environment,
        sessionID: session, flightKey: SQLiteReadConnection.text(s, 4), timelineEpoch: Int(sqlite3_column_int(s, 5)),
        time: d(s, 6), dynamicPressure: d(s, 7), mach: d(s, 8), alpha: radians(d(s, 9)), beta: radians(d(s, 10)),
        bank: radians(d(s, 11)), mass: d(s, 12), gear: d(s, 13) == 1, brakes: d(s, 14) == 1,
        airbrakes: d(s, 15) == 1, rcs: Int(sqlite3_column_int(s, 16)), position: v(17), velocity: v(20),
        lift: v(23), drag: v(26), profile: SQLiteReadConnection.text(s, 29) ?? "",
        bodyRates: rates.isFinite ? rates * (.pi / 180) : nil,
        legacyQuality: d(s, 33), legacyEligible: d(s, 34) == 1))
    }
    let samples = HistoryReduction.reduce(rows, maximum: maximum, planet: planet)
    return .init(contexts: requested, queriedRows: queried, rejectedRows: rejected,
      independentSources: Set(rows.map(\.source)).count, samples: samples,
      excludedSession: excludingSession, excludedFlight: excludingFlight,
      evidence: "f1eebdc-schema3-quality-provenance-independent-flight-medoid")
  }
}
