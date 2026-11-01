import Foundation
import KSLCore

/// Survey scheduling only. A local turning estimate may advance exploration,
/// but can neither certify reachability nor grant MM305 control ownership.
enum TerminalExploration {
  static func searchRange(state: VehicleState, configuration c: MissionConfiguration,
                          minimumTurnTime: Double?, rollResponseSeconds: Double?) -> Double {
    let cadence = c.guidance.terminalReplanInterval
    let baseline = c.entrySettings.terminalTargetRange + state.horizontalSpeed * cadence
    let turnTime = minimumTurnTime.flatMap { $0.isFinite && $0 >= 0 ? $0 : nil } ?? 0
    let response = max(0, rollResponseSeconds ?? 0)
    // A maximum length for one complete HAC candidate is not a permission
    // gate for early exploration. A large turn deficit must advance advice,
    // not be clipped away at the same distance on every entry.
    // Bound this local extrapolation by the configured prediction TIME horizon;
    // the planner independently retains its complete-path length/force limits.
    let leadTime = min(c.guidance.maximumPredictionDuration, turnTime + response + cadence)
    let maneuverLead = state.horizontalSpeed * leadTime
    return max(baseline, maneuverLead)
  }
}
