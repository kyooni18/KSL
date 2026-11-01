import KSLCore

/// Value snapshot for prediction branches. No live controller or reference
/// semantics cross the planning boundary. Rollout is never forecast as flight.
public struct FlightControlSnapshot: Sendable {
  /// Only identified trim is plant evidence; a PD integral is not a moment fit.
  public func predictionPitchTrim(alpha: Double) -> Double? { trimModel.input(alpha: alpha) }
  /// Preserve an observed local equilibrium even at full actuator input.
  /// This changes the predicted restoring bias, never the available torque.
  public func predictionPitchTrim(state: VehicleState) -> Double? {
    guard !state.systems.rcs else { return nil }
    return trimModel.predictionInput(time: state.time, mach: state.aerodynamics.mach,
      pressure: state.aerodynamics.dynamicPressure, alpha: state.aerodynamics.angleOfAttack)
  }
  let pitchAuthority: AxisAuthority, rollAuthority: AxisAuthority, yawAuthority: AxisAuthority
  let betaModel: SideslipResponseModel
  let pitchTrim: Double, rollTrim: Double, pitchScale: Double
  let previous: ControlCommand
  let previousError: Vector3?, errorRate: Vector3
  let previousDesiredAttitude: Quaternion?, pitchReferenceRate: Double
  let previousState: VehicleState?, previousPhase: MissionPhase?
  let trimModel: AdaptiveTrim
  let previousFeedforward: Double, feedforwardWasIdentified: Bool
  let feedforwardWasPrior: Bool
  let coupledRollAcceleration: Double
}
