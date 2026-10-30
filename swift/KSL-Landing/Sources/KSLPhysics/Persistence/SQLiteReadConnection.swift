import Foundation
import CSQLite
import KSLCore

/// Owned exclusively by PhysicsObservationStore. SQLite itself is also opened
/// FULLMUTEX. There is deliberately no statement execution/write API.
final class SQLiteReadConnection {
  let handle: OpaquePointer
  init(path: String) throws {
    var pointer: OpaquePointer?
    let result = sqlite3_open_v2(path, &pointer, SQLITE_OPEN_READONLY | SQLITE_OPEN_FULLMUTEX, nil)
    guard result == SQLITE_OK, let pointer else {
      let message = pointer.map { String(cString: sqlite3_errmsg($0)) } ?? "open failed"
      if let pointer { sqlite3_close(pointer) }
      throw FlightError.io("Physics archive: " + message)
    }
    handle = pointer
    sqlite3_busy_timeout(handle, 250)
  }
  deinit { sqlite3_close(handle) }
  func query(_ sql: String, strings: [String] = [], row: (OpaquePointer) throws -> Void) throws {
    guard sql.trimmingCharacters(in: .whitespacesAndNewlines).uppercased().hasPrefix("SELECT ") else {
      throw FlightError.invalidCommand("Physics archive only accepts SELECT")
    }
    var statement: OpaquePointer?
    guard sqlite3_prepare_v2(handle, sql, -1, &statement, nil) == SQLITE_OK, let statement else {
      throw FlightError.io("Physics archive query: " + String(cString: sqlite3_errmsg(handle)))
    }
    defer { sqlite3_finalize(statement) }
    let transient = unsafeBitCast(-1, to: sqlite3_destructor_type.self)
    for (i, text) in strings.enumerated() {
      guard sqlite3_bind_text(statement, Int32(i + 1), text, -1, transient) == SQLITE_OK else {
        throw FlightError.io("Physics archive binding failed")
      }
    }
    var status = sqlite3_step(statement)
    while status == SQLITE_ROW { try row(statement); status = sqlite3_step(statement) }
    guard status == SQLITE_DONE else {
      throw FlightError.io("Physics archive read: " + String(cString: sqlite3_errmsg(handle)))
    }
  }
  static func text(_ statement: OpaquePointer, _ index: Int32) -> String? {
    sqlite3_column_text(statement, index).map { String(cString: $0) }
  }
  static func number(_ statement: OpaquePointer, _ index: Int32) -> Double {
    sqlite3_column_type(statement, index) == SQLITE_NULL ? .nan : sqlite3_column_double(statement, index)
  }
}
