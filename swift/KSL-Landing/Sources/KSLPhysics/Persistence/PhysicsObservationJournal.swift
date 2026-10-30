import Foundation
import KSLCore

/// Explicitly enabled, separate persistence path. Import into the original
/// archive is a separate operation; this class has no archive write access.
public actor PhysicsObservationJournal {
  private let file: FileHandle
  private let encoder = JSONEncoder()
  private let context: PhysicsContext
  private let sessionID: String, flightKey: String
  private var closed = false
  private var epoch = 0
  private var previousTime: Double?
  public init(url: URL, protectedArchive: URL, context: PhysicsContext,
              sessionID: String, flightKey: String) throws {
    guard url.resolvingSymlinksInPath().standardizedFileURL != protectedArchive.resolvingSymlinksInPath().standardizedFileURL,
          !sessionID.isEmpty, !flightKey.isEmpty, url.pathExtension == "jsonl",
          !FileManager.default.fileExists(atPath: url.path) else {
      throw FlightError.invalidConfiguration("Observation journal requires a new separate JSONL file and explicit identity")
    }
    guard FileManager.default.createFile(atPath: url.path, contents: nil) else {
      throw FlightError.io("Cannot create observation journal")
    }
    file = try FileHandle(forWritingTo: url)
    self.context = context; self.sessionID = sessionID; self.flightKey = flightKey
  }
  deinit { try? file.close() }
  public func observe(_ state: VehicleState, quality: CalibrationObservationQuality? = nil) throws {
    guard !closed else { throw FlightError.io("Observation journal is closed") }
    if let previousTime, state.time < previousTime { epoch += 1 }
    previousTime = state.time
    guard state.aerodynamics.measuredForces, state.systems.unpowered, !state.contact.isGrounded,
          state.aerodynamics.dynamicPressure > 0 else { return }
    struct Entry: Encodable {
      let schema: String, context: PhysicsContext, sessionID: String, flightKey: String
      let timelineEpoch: Int, state: VehicleState
      let quality: CalibrationObservationQuality?
    }
    var bytes = try encoder.encode(Entry(schema: "swift-physics-observation-v2", context: context,
      sessionID: sessionID, flightKey: flightKey, timelineEpoch: epoch, state: state, quality: quality))
    bytes.append(10); try file.write(contentsOf: bytes)
  }
  public func close() throws {
    if !closed { try file.synchronize(); try file.close(); closed = true }
  }
}
