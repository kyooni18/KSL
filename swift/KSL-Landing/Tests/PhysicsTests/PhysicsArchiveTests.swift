import CSQLite
import Foundation
import KSLCore
@testable import KSLPhysics
import Testing

struct PhysicsArchiveTests {
  private let environment = "{\"schema\":3,\"planet\":{\"name\":\"Kerbin\",\"radius\":600000},\"atmosphereCurveDigest\":null}"
  private func context(_ id: String = "model:STS", environment e: String? = nil) -> PhysicsContext {
    .init(structureID: id, environmentID: "env", structureJSON:
      "{\"parts\":[{\"name\":\"wing\"}],\"geometryComplete\":true,\"geometry\":[{\"name\":\"wing\",\"position\":[1,2,3]}]}",
      environmentJSON: e ?? environment)
  }
  private func fixture() throws -> URL {
    let url = FileManager.default.temporaryDirectory.appendingPathComponent("physics-contract-\(UUID()).sqlite3")
    var db: OpaquePointer?
    #expect(sqlite3_open(url.path, &db) == SQLITE_OK)
    guard let db else { throw FlightError.io("Fixture could not open") }
    defer { sqlite3_close(db) }
    func sql(_ statement: String) throws {
      guard sqlite3_exec(db, statement, nil, nil, nil) == SQLITE_OK else {
        throw FlightError.io(String(cString: sqlite3_errmsg(db)))
      }
    }
    try sql("""
      CREATE TABLE physics_meta(key TEXT PRIMARY KEY,value TEXT);
      INSERT INTO physics_meta VALUES('schema','3');
      CREATE TABLE physics_contexts(structure_id TEXT,environment_id TEXT,structure_json TEXT,environment_json TEXT,last_seen REAL);
      CREATE TABLE physics_sessions(session_id TEXT PRIMARY KEY,flight_key TEXT);
      CREATE TABLE aero_observations(id INTEGER PRIMARY KEY,structure_id TEXT,environment_id TEXT,session_id TEXT,timeline_epoch INTEGER,
        ut REAL,q REAL,mach REAL,aoa REAL,beta REAL,roll REAL,mass REAL,gear INTEGER,brakes INTEGER,airbrakes INTEGER,rcs INTEGER,
        pos_x REAL,pos_y REAL,pos_z REAL,vel_x REAL,vel_y REAL,vel_z REAL,lift_x REAL,lift_y REAL,lift_z REAL,drag_x REAL,drag_y REAL,drag_z REAL);
      CREATE TABLE aero_observation_quality(observation_id INTEGER PRIMARY KEY,control_profile TEXT,body_pitch_rate REAL,
        body_roll_rate REAL,body_yaw_rate REAL,quality REAL,eligible INTEGER);
      INSERT INTO physics_sessions VALUES('a','flight-a'),('a2','flight-a'),('b','flight-b'),('current','flight-current'),('reload','flight-current');
      """)
    for c in [context(), context("model:STS#payload-bay-open")] {
      try sql("INSERT INTO physics_contexts VALUES('\(c.structureID)','env','\(c.structureJSON)','\(c.environmentJSON)',0)")
    }
    let rows: [(Int, String, Double, Double, Bool, String)] = [
      (1,"a",2,1024,true,"model:STS"), (2,"a",2,1024,true,"model:STS"),
      (3,"a2",20,1024,true,"model:STS"), (4,"b",4,1024,true,"model:STS"),
      (5,"b",40,1024,false,"model:STS"), (6,"current",40,1024,true,"model:STS"),
      (7,"reload",40,1024,true,"model:STS"), (8,"b",40,1024,true,"model:STS#payload-bay-open"),
      (9,"b",40,-1,true,"model:STS")]
    for (id, session, lift, q, modern, structure) in rows {
      try sql("INSERT INTO aero_observations VALUES(\(id),'\(structure)','env','\(session)',0,100,\(q),1,8,0,0,25000,0,0,0,0,600100,0,0,0,0,400,\(lift*1024),0,0,0,0,\(-lift*512))")
      if modern {
        // Retired eligibility scores must not discard valid measured forces.
        try sql("INSERT INTO aero_observation_quality VALUES(\(id),'Recovery',50,60,70,0,0)")
      }
    }
    return url
  }
  @Test func historyPreservesIndependentFlightProvenanceAndNeverWritesArchive() async throws {
    let url = try fixture()
    defer { try? FileManager.default.removeItem(at: url) }
    let before = try Data(contentsOf: url)
    let store = try PhysicsObservationStore(path: url.path)
    let history = try await store.history(contexts: [context()], excludingSession: "current",
      excludingFlight: "flight-current", planet: .init())
    #expect(history.queriedRows == 5)
    #expect(history.rejectedRows == 1)
    #expect(history.independentSources == 2)
    let sample = try #require(history.samples.first)
    #expect(history.samples.count == 1)
    #expect(sample.observation.id == 4)
    #expect(sample.independentFlights == 2)
    #expect(Set(sample.support.map(\.observationID)) == [1,4])
    #expect(sample.forcePerDynamicPressure == Vector3(2,4,0))
    #expect(!sample.observation.legacyEligible)
    #expect(try Data(contentsOf: url) == before)
    let model = HistoricalAerodynamics(history: history)
    let force = try #require(model.estimate(q: 1024, mach: 1, alpha: radians(8), beta: 0,
      gear: false, brakes: false, airbrakes: false))
    #expect(force.localityConfidence == 1)
    #expect(force.independentFlights == 2)
    #expect(force.observationIDs == [1,4])
    #expect(model.estimate(q: 2048, mach: 1, alpha: radians(8), beta: 0,
      gear: false, brakes: false, airbrakes: false) == nil)
    #expect(model.estimate(q: 1024, mach: 1, alpha: radians(8), beta: 0,
      gear: true, brakes: false, airbrakes: false) == nil)
    do {
      _ = try await store.history(contexts: [context(), context("model:STS#payload-bay-open")],
        excludingSession: "", excludingFlight: "", planet: .init())
      Issue.record("Incompatible vehicle variant accepted")
    } catch { #expect(error is FlightError) }
  }
  @Test func identityIgnoresCoordinateEpochButRetainsPlanetAndVariant() {
    let a = context()
    let b = context(environment: "{\"planet\":{\"epochUT\":1234,\"radius\":600000.0,\"name\":\"Kerbin\"},\"atmosphereCurveDigest\":\"legacy\"}")
    #expect(PhysicsIdentityMatcher.compatible(b, with: a))
    #expect(!PhysicsIdentityMatcher.compatible(context("model:STS#payload-bay-open"), with: a))
    #expect(PhysicsIdentityMatcher.compatible(context("sha256:legacy"), with: a))
    #expect(!PhysicsIdentityMatcher.environmentsMatch(b.environmentJSON,
      b.environmentJSON.replacingOccurrences(of: "legacy", with: "different")))
    #expect(!PhysicsIdentityMatcher.environmentsMatch(a.environmentJSON,
      a.environmentJSON.replacingOccurrences(of: "600000", with: "610000")))
  }
}
