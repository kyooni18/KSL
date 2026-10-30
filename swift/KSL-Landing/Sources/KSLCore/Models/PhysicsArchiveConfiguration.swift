import Foundation

public struct PhysicsArchiveConfiguration: Codable, Sendable, Equatable {
  public var path: String
  public var structureID: String
  public var environmentID: String
  public var excludedSession: String
  public var excludedFlight: String
  public init(path: String, structureID: String, environmentID: String,
              excludedSession: String = "", excludedFlight: String = "") {
    self.path = path; self.structureID = structureID; self.environmentID = environmentID
    self.excludedSession = excludedSession; self.excludedFlight = excludedFlight
  }
}
