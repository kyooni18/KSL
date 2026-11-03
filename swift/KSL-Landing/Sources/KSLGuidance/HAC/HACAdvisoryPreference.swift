import Foundation
import KSLCore

/// Ordering inside one physical evidence class. This never certifies a route.
enum HACAdvisoryPreference {
  /// Distance-weighted departure from the configured normal descent envelope.
  /// Final has its own flare profile. Both excessive descent and climbing
  /// geometry are priced; a short, steep shortcut is not a better normal plan.
  static func glideDeparture(_ plan: TerminalPlan, maximumGlide: Double) -> Double {
    var work = 0.0, length = 0.0
    for i in plan.stations.indices.dropFirst() {
      let a = plan.stations[i-1], b = plan.stations[i]
      guard a.segment != .final else { continue }
      let ds = b.distance-a.distance
      guard ds > 0 else { continue }
      func excess(_ slope: Double) -> Double {
        let gamma = atan(slope)
        return max(0, -gamma-maximumGlide, gamma)
      }
      work += ds*(excess(a.slope)+excess(b.slope))/2; length += ds
    }
    return length > 0 ? work/length : .infinity
  }
  static func prefers(_ a: TerminalPlan, over b: TerminalPlan,
                      incumbentSide: Int?, maximumGlide: Double) -> Bool? {
    if a.assessment.feasible != b.assessment.feasible { return a.assessment.feasible }
    let completeA = a.assessment.predictionComplete == true
    let completeB = b.assessment.predictionComplete == true
    let airborneA = completeA && a.assessment.airborneFeasible == true
    let airborneB = completeB && b.assessment.airborneFeasible == true
    if airborneA != airborneB { return airborneA }
    // Equally unqualified airborne forecasts cannot buy a free direction reversal.
    // A physically feasible alternative still wins above; actual command
    // transfer retains native admission and finite-response limiting.
    if !a.assessment.feasible, let incumbentSide {
      let sameA = a.spiral?.side == incumbentSide, sameB = b.spiral?.side == incumbentSide
      if sameA != sameB { return sameA }
    }
    if completeA != completeB { return completeA }
    let ga = glideDeparture(a, maximumGlide: maximumGlide)
    let gb = glideDeparture(b, maximumGlide: maximumGlide)
    // Ignore only floating-point angle roundoff, not a physical glide margin.
    let roundoff = 16*Double.ulpOfOne*max(maximumGlide, abs(ga), abs(gb))
    if abs(ga-gb) > roundoff { return ga < gb }
    return nil
  }
}
