import Foundation

/// A planning lane owns only immutable snapshots. The live runtime can keep
/// commanding the vehicle while a lane is slow or cooperatively cancelling.
public protocol MissionPlanningService: Sendable {
  func recordedPlan(_ request: MissionPlanningRequest) async throws -> RecordedPlanningProposal
}

extension MissionPlanningWorker: MissionPlanningService {}
