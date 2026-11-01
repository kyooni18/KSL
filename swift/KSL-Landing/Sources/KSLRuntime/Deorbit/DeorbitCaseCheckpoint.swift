import Foundation
import KSLCore

/// The complete inputs to one native orbit-to-stop case. The checkpoint owner
/// must additionally bind its store to the executable and loaded model resources.
public struct DeorbitCaseIdentity: Codable, Sendable {
  public let configuration: MissionConfiguration
  public let source: VehicleState
  public let window: DeorbitWindow
  public let index: Int
  public let entryReplanning: Bool
  public init(configuration: MissionConfiguration, source: VehicleState,
              window: DeorbitWindow, index: Int, entryReplanning: Bool) {
    self.configuration = configuration; self.source = source; self.window = window
    self.index = index; self.entryReplanning = entryReplanning
  }
}

/// Optional offline persistence; the flight planner uses no cache by default.
/// A failed save is an experiment failure, never a silently lost case result.
public protocol DeorbitCaseCheckpoint: Sendable {
  func load(_ identity: DeorbitCaseIdentity) async throws -> DeorbitStressCase?
  func save(_ identity: DeorbitCaseIdentity, result: DeorbitStressCase) async throws
}
