import Foundation
import KSLCore

public struct MM305Admission: Codable, Sendable {
  public let accepted: Bool
  public let altitudeMargin: Double
  public let machMargin: Double
  public let headingMargin: Double
  public let planAge: Double
  public let violations: [ConstraintViolation]

  public static func evaluate(_ state: VehicleState, plan: TerminalPlan,
                              configuration c: MissionConfiguration) -> Self {
    let e = c.entrySettings
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let altitude = state.position.length - c.planet.radius
    // f1eebdc mm305_acquisition_ready includes the later HAC acquisition
    // band. These settings enlarge only the search/admission envelope;
    // the complete closed-loop route certificate is still mandatory.
    let minimumAltitude = min(e.terminalMinimumAltitude, c.guidance.hacAcquisitionAltitude ?? e.terminalMinimumAltitude)
    let minimumMach = min(e.terminalTargetMach-e.terminalMachHalfWidth,
      c.guidance.hacAcquisitionMach ?? e.terminalTargetMach)
    let altitudeMargin = min(altitude-minimumAltitude, e.terminalMaximumAltitude-altitude)
    let machMargin = min(state.aerodynamics.mach-minimumMach,
      e.terminalTargetMach+e.terminalMachHalfWidth-state.aerodynamics.mach)
    let inlet = plan.stations.first
    let direction = inlet.map { frame.direction(at: $0.position, tangent: $0.tangent) } ?? .zero
    let headingError = abs(wrapAngle(state.localFrame.heading(of: direction)-state.course))
    let headingMargin = e.headingHalfWidth-headingError
    let planAge = state.time-plan.createdAt
    var failures: [ConstraintViolation] = []
    let range = frame.distance(from: state.position, to: frame.origin)
    let minimumRange = max(c.guidance.finalDistance*1.5,
      c.guidance.finalDistance+(c.guidance.hacRadius ?? 12000))
    let closing = state.velocity.dot(frame.origin.normalized.projected(onPlaneWithNormal: state.position.normalized)) > 0
    if range < minimumRange || range > e.terminalTargetRange || !closing || state.verticalSpeed >= 0 {
      failures.append(.init(.invalidGeometry, "MM305 requires descending, closing acquisition inside the terminal range band", value: range))
    }
    if state.aerodynamics.airspeed < (c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed) {
      failures.append(.init(.insufficientEnergy, "MM305 airspeed below configured minimum safe speed"))
    }
    if altitudeMargin < 0 { failures.append(.init(.invalidGeometry, "MM305 altitude admission margin", value: altitudeMargin)) }
    if machMargin < 0 { failures.append(.init(.excessiveEnergy, "MM305 Mach admission margin", value: machMargin)) }
    if headingMargin < 0 { failures.append(.init(.invalidGeometry, "MM305 heading admission margin", value: headingMargin)) }
    if planAge < 0 || planAge > 2*c.guidance.terminalReplanInterval {
      failures.append(.init(.telemetry, "MM305 plan no longer matches the measured epoch", value: planAge,
        limit: 2*c.guidance.terminalReplanInterval))
    }
    if !plan.nominalQualified {
      failures.append(.init(.modelUncertainty, "MM305 requires a successful nominal closed-loop touchdown and rollout"))
    }
    // isRobust remains false when configured stress cases fail. Admission is
    // not a robust or live-flight certificate; all nominal physical gates stay.
    let q = state.aerodynamics.dynamicPressure
    if q > c.vehicle.maximumDynamicPressure || q <= 0 {
      failures.append(.init(.excessiveDynamicPressure, "MM305 dynamic pressure outside flight envelope", value: q,
        limit: c.vehicle.maximumDynamicPressure))
    }
    if abs(state.aerodynamics.sideslip) > e.maximumSideslip {
      failures.append(.init(.actuatorAuthority, "MM305 sideslip exceeds coordinated flight envelope",
        value: abs(state.aerodynamics.sideslip), limit: e.maximumSideslip))
    }
    return .init(accepted: failures.isEmpty, altitudeMargin: altitudeMargin,
      machMargin: machMargin, headingMargin: headingMargin, planAge: planAge, violations: failures)
  }
}
