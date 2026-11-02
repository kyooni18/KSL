import KSLCore

/// Completion of physical accounting is independent of runway qualification.
public enum TerminalEvaluationStatus: String, Codable, Sendable {
  case complete, incomplete, invalid
}
public enum TerminalEvaluationEnd: String, Codable, Sendable {
  case sustainedStop, modeledCrash, horizon, cancelled, unsupportedDynamics, computationFailure
}

/// An early violation never supplies a free terminal suffix. Positive excess
/// integrals cannot be diluted by extending an otherwise benign flight.
public struct TerminalEvaluation: Codable, Sendable {
  public let status: TerminalEvaluationStatus
  public let end: TerminalEvaluationEnd
  public let state: VehicleState
  /// Integral of aerodynamic force dot ground-relative velocity / mass (J/kg).
  /// This retains wind work in the same rotating energy convention as guidance.
  public let aerodynamicWork: Double
  public let energyChange: Double
  public let heightChange: Double
  public let maximumPressureExcess: Double, pressureExcessIntegral: Double
  public let maximumLoadExcess: Double, loadExcessIntegral: Double
}
