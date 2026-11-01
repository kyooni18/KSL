// Offline-only campaign runner. Link against the built KSL library objects.
// search CONFIG STATE OUTPUT_DIRECTORY
// case PLAN OUTPUT_DIRECTORY CASE_INDEX
// Every completed case is atomically persisted; progress is never suppressed.
import Foundation
import CryptoKit
import Darwin
import KSLCore
import KSLPhysics
import KSLRuntime

private func encoded<T: Encodable>(_ value: T) throws -> Data {
  let encoder = JSONEncoder(); encoder.outputFormatting = [.sortedKeys]
  return try encoder.encode(value)
}
private func digest(_ data: Data) -> String {
  SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
}
private struct CheckpointRecord: Codable {
  let identity: DeorbitCaseIdentity
  let result: DeorbitStressCase
}
private actor Journal: DeorbitCaseCheckpoint {
  let directory: URL
  let progressFile: FileHandle
  private(set) var resumed = 0
  init(directory: URL, fingerprint: [String: String]) throws {
    self.directory = directory
    try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
    let identity = directory.appendingPathComponent("model-identity.json")
    let bytes = try encoded(fingerprint)
    if FileManager.default.fileExists(atPath: identity.path) {
      guard try Data(contentsOf: identity) == bytes else {
        throw FlightError.invalidConfiguration("Checkpoint executable or resource identity changed; use a new directory")
      }
    } else { try bytes.write(to: identity, options: .withoutOverwriting) }
    let log = directory.appendingPathComponent("progress.jsonl")
    if !FileManager.default.fileExists(atPath: log.path) {
      guard FileManager.default.createFile(atPath: log.path, contents: nil) else {
        throw FlightError.io("Cannot create progress journal")
      }
    }
    progressFile = try FileHandle(forWritingTo: log); try progressFile.seekToEnd()
  }
  private func url(_ identity: DeorbitCaseIdentity) throws -> URL {
    directory.appendingPathComponent("case-" + digest(try encoded(identity)) + ".json")
  }
  func load(_ identity: DeorbitCaseIdentity) throws -> DeorbitStressCase? {
    let path = try url(identity)
    guard FileManager.default.fileExists(atPath: path.path) else { return nil }
    let record = try JSONDecoder().decode(CheckpointRecord.self, from: Data(contentsOf: path))
    guard try encoded(record.identity) == encoded(identity), record.result.index == identity.index else {
      throw FlightError.invalidConfiguration("Checkpoint identity mismatch")
    }
    resumed += 1
    return record.result
  }
  func save(_ identity: DeorbitCaseIdentity, result: DeorbitStressCase) throws {
    let path = try url(identity), bytes = try encoded(CheckpointRecord(identity: identity, result: result))
    if FileManager.default.fileExists(atPath: path.path) {
      guard try Data(contentsOf: path) == bytes else {
        throw FlightError.invalidConfiguration("Identical case inputs produced conflicting completed results")
      }
    } else { try bytes.write(to: path, options: .atomic) }
  }
  func progress(_ value: DeorbitPlanningProgress) {
    struct Event: Encodable { let wallTime: Double; let progress: DeorbitPlanningProgress }
    do {
      try progressFile.write(contentsOf: encoded(Event(wallTime: Date().timeIntervalSince1970, progress: value)))
      try progressFile.write(contentsOf: Data([10]))
    } catch {
      FileHandle.standardError.write(Data(("Progress journal failed: \(error)\n").utf8))
    }
  }
  func close() throws { try progressFile.synchronize(); try progressFile.close() }
}

private func modelFingerprint(executable: URL, configuration c: MissionConfiguration) throws -> [String: String] {
  var fingerprint = ["schema": "native-case-checkpoint-v1", "executableSHA256": digest(try Data(contentsOf: executable))]
  let resources = executable.deletingLastPathComponent().appendingPathComponent("KSL-Landing_KSLPhysics.bundle")
  guard let files = FileManager.default.enumerator(at: resources, includingPropertiesForKeys: [.isRegularFileKey]) else {
    throw FlightError.invalidConfiguration("Captured physics resource bundle is missing")
  }
  var resourceCount = 0
  for case let file as URL in files where try file.resourceValues(forKeys: [.isRegularFileKey]).isRegularFile == true {
    fingerprint["resource:" + String(file.path.dropFirst(resources.path.count))] = digest(try Data(contentsOf: file))
    resourceCount += 1
  }
  guard resourceCount > 0 else { throw FlightError.invalidConfiguration("Empty physics resource bundle") }
  // This runner requires an immutable bundle. Archive-backed experiments
  // need a separately captured SQLite snapshot and identity implementation.
  guard c.physicsArchive == nil else {
    throw FlightError.invalidConfiguration("Capture an immutable physics-archive snapshot before using this checkpoint runner")
  }
  return fingerprint
}

@main struct ResumeDeorbitPlan {
  static func main() async {
    do { try await run() }
    catch {
      FileHandle.standardError.write(Data(("offline-checkpoint-error: \(error)\n").utf8))
      exit(1)
    }
  }
  static func run() async throws {
    let args = CommandLine.arguments
    guard args.count == 5, ["search", "case"].contains(args[1]) else {
      throw FlightError.invalidConfiguration("search CONFIG STATE OUTPUT_DIR | case PLAN OUTPUT_DIR CASE_INDEX")
    }
    let decoder = JSONDecoder()
    let plan: DeorbitPlan? = args[1] == "case"
      ? try decoder.decode(DeorbitPlan.self, from: Data(contentsOf: URL(fileURLWithPath: args[2]))) : nil
    let c = try plan?.configuration ?? decoder.decode(MissionConfiguration.self,
      from: Data(contentsOf: URL(fileURLWithPath: args[2])))
    let source = try plan?.source ?? decoder.decode(VehicleState.self,
      from: Data(contentsOf: URL(fileURLWithPath: args[3])))
    // Production planning derives propulsion from the source snapshot. The
    // diagnostic must preserve that invariant, or the case identity would omit
    // a separately edited plan.propulsion value used by OrbitalExecutionModel.
    if let plan {
      guard let measured = source.propulsion,
        try encoded(measured) == encoded(plan.propulsion) else {
        throw FlightError.invalidConfiguration("Case propulsion differs from the keyed source snapshot; update source and plan consistently")
      }
    }
    let directory = URL(fileURLWithPath: args[1] == "case" ? args[3] : args[4], isDirectory: true)
    let executable = URL(fileURLWithPath: args[0]).standardizedFileURL
    let fingerprint = try modelFingerprint(executable: executable, configuration: c)
    let journal = try Journal(directory: directory, fingerprint: fingerprint)
    if let plan {
      guard let index = Int(args[4]), index == 0 else {
        throw FlightError.invalidConfiguration("Invalid case index")
      }
      let identity = DeorbitCaseIdentity(configuration: c, source: source,
        window: plan.window, index: index, entryReplanning: true)
      let result: DeorbitStressCase
      if let cached = try await journal.load(identity) { result = cached }
      else {
        result = try await DeorbitDiagnostic.evaluateWindow(plan: plan, caseIndex: index, entryReplanning: true)
        try await journal.save(identity, result: result)
      }
      print(String(decoding: try encoded(result), as: UTF8.self))
    } else {
      let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
      let result = try await DeorbitPlanner.planSelectingRunway(from: source, configuration: c,
        aerodynamics: aero, allowReciprocal: true, checkpoint: journal) { await journal.progress($0) }
      let output = directory.appendingPathComponent("plan-" + UUID().uuidString + ".json")
      try encoded(result).write(to: output, options: .withoutOverwriting)
      print("plan=\(output.path) runway=\(result.configuration.runway.name) nominal=\(result.nominalPassed) completed=\(result.cases.count)/1 passed=\(result.passedCases) fullRoute=\(result.fullRouteQualified) execution=\(result.executionQualified)")
    }
    print("resumedCases=\(await journal.resumed)")
    try await journal.close()
  }
}
