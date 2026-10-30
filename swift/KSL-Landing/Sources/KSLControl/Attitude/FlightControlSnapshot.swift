import KSLCore

/// Value snapshot for prediction branches. No live controller or reference
/// semantics cross the planning boundary. Rollout is never forecast as flight.
public struct FlightControlSnapshot: Sendable {
  let pitchAuthority: AxisAuthority, rollAuthority: AxisAuthority, yawAuthority: AxisAuthority
  let betaModel: SideslipResponseModel
  let pitchTrim: Double, rollTrim: Double, pitchScale: Double
  let previous: ControlCommand
  let previousError: Vector3?, errorRate: Vector3
  let previousState: VehicleState?, previousPhase: MissionPhase?
  let trimModel: AdaptiveTrim
  let previousFeedforward: Double, feedforwardWasIdentified: Bool
  let feedforwardWasPrior: Bool
  let coupledRollAcceleration: Double
}
