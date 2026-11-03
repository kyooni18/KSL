import Foundation
import KSLCore

/// Future MM304 endpoint for advisory geometry, never a control-ownership
/// certificate. Heap ownership avoids enlarging every planning value snapshot.
public final class TAEMAdvisoryInlet: Codable, Sendable {
  public let sourceTime: Double
  public let state: VehicleState
  public let model: String
  public init(sourceTime: Double, state: VehicleState, model: String) {
    self.sourceTime = sourceTime; self.state = state; self.model = model
  }
}
