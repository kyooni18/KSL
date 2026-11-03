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
  /// Dimensionless turn severity outside final. Banked flight needs
  /// n^2-1 = tan^2(bank) of extra induced drag, priced as its distance-weighted
  /// mean. Curvature changes and roll reversals are priced as the fraction of
  /// flight time the configured roll rate spends on total bank variation, so
  /// a reversal costs twice its bank magnitude.
  static func turnSeverity(_ plan: TerminalPlan, rollRate: Double) -> Double {
    var induced = 0.0, length = 0.0, variation = 0.0, duration = 0.0
    for i in plan.stations.indices.dropFirst() {
      let a = plan.stations[i-1], b = plan.stations[i]
      guard a.segment != .final, b.segment != .final else { continue }
      let ds = b.distance-a.distance
      guard ds > 0 else { continue }
      func drag(_ bank: Double) -> Double { let t = tan(min(abs(bank), .pi/2*0.999)); return t*t }
      induced += ds*(drag(a.bank)+drag(b.bank))/2; length += ds
      variation += abs(wrapAngle(b.bank-a.bank))
      let speed = (a.speed+b.speed)/2
      if speed > 0 { duration += ds/speed }
    }
    guard length > 0 else { return 0 }
    let rolling = duration > 0 && rollRate > 0 ? variation/rollRate/duration : 0
    return induced/length + rolling
  }
  static func prefers(_ a: TerminalPlan, over b: TerminalPlan,
                      incumbentSide: Int?, maximumGlide: Double, rollRate: Double) -> Bool? {
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
    // Gentler is better within one evidence class: glide-envelope departure
    // and turn severity are both dimensionless and summed on equal footing.
    let ga = glideDeparture(a, maximumGlide: maximumGlide)+turnSeverity(a, rollRate: rollRate)
    let gb = glideDeparture(b, maximumGlide: maximumGlide)+turnSeverity(b, rollRate: rollRate)
    // Ignore only floating-point angle roundoff, not a physical glide margin.
    let roundoff = 16*Double.ulpOfOne*max(maximumGlide, abs(ga), abs(gb))
    if abs(ga-gb) > roundoff { return ga < gb }
    return nil
  }
}
