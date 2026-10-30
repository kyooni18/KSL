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
                              configuration c: MissionConfiguration, requireQualification: Bool = true) -> Self {
    let e = c.entrySettings
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let physical = physicalAdmission(state, configuration: c)
    let altitudeMargin = physical.altitudeMargin, machMargin = physical.machMargin
    let inlet = plan.stations.first
    let direction = inlet.map { frame.direction(at: $0.position, tangent: $0.tangent) } ?? .zero
    let headingError = abs(wrapAngle(state.localFrame.heading(of: direction)-state.course))
    let headingMargin = e.headingHalfWidth-headingError
    let planAge = state.time-plan.createdAt
    var failures = physical.violations
    if headingMargin < 0 { failures.append(.init(.invalidGeometry, "MM305 heading admission margin", value: headingMargin)) }
    if planAge < 0 || planAge > 2*c.guidance.terminalReplanInterval {
      failures.append(.init(.telemetry, "MM305 plan no longer matches the measured epoch", value: planAge,
        limit: 2*c.guidance.terminalReplanInterval))
    }
    if requireQualification && !plan.nominalQualified {
      failures.append(.init(.modelUncertainty, "MM305 requires a successful nominal closed-loop touchdown and rollout"))
    }
    // isRobust remains false when configured stress cases fail. Admission is
    // not a robust or live-flight certificate; all nominal physical gates stay.
    return .init(accepted: failures.isEmpty, altitudeMargin: altitudeMargin,
      machMargin: machMargin, headingMargin: headingMargin, planAge: planAge, violations: failures)
  }
  /// Plan-independent admission gates, also used before a forecast performs
  /// an expensive HAC survey. A state failing these cannot be a capture state.
  static func physicalAdmission(_ state: VehicleState, configuration c: MissionConfiguration)
    -> (altitudeMargin: Double, machMargin: Double, violations: [ConstraintViolation]) {
    let e = c.entrySettings
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let altitude = state.position.length - c.planet.radius
    // f1eebdc mm305_acquisition_ready includes the later HAC acquisition
    // band. Below the MM304 band the floor is physical: the HAC exit on the
    // final-approach reference (lower, final capture owns the vehicle). The
    // speed floor is the minimum safe speed below; the closed-loop route
    // certificate remains mandatory either way.
    let exitHeight = ApproachProfile(configuration: c).reference(along: -c.guidance.finalDistance).height
    let minimumAltitude = min(e.terminalMinimumAltitude, c.guidance.hacAcquisitionAltitude ?? exitHeight, exitHeight)
    let minimumMach = c.guidance.hacAcquisitionMach ?? 0
    let altitudeMargin = min(altitude-minimumAltitude, e.terminalMaximumAltitude-altitude)
    let machMargin = min(state.aerodynamics.mach-minimumMach,
      e.terminalTargetMach+e.terminalMachHalfWidth-state.aerodynamics.mach)
    var failures: [ConstraintViolation] = []
    let range = frame.distance(from: state.position, to: frame.origin)
    let minimumRange = max(c.guidance.finalDistance*1.5,
      c.guidance.finalDistance+(c.guidance.hacRadius ?? 12000))
    let closing = state.velocity.dot(frame.origin.normalized.projected(onPlaneWithNormal: state.position.normalized)) > 0
    if range < minimumRange || range > c.guidance.maximumPathLength || !closing || state.verticalSpeed >= 0 {
      failures.append(.init(.invalidGeometry, "MM305 requires descending, closing acquisition inside the terminal range band", value: range))
    }
    if state.aerodynamics.airspeed < (c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed) {
      failures.append(.init(.insufficientEnergy, "MM305 airspeed below configured minimum safe speed"))
    }
    if altitudeMargin < 0 { failures.append(.init(.invalidGeometry, "MM305 altitude admission margin", value: altitudeMargin)) }
    if machMargin < 0 { failures.append(.init(.excessiveEnergy, "MM305 Mach admission margin", value: machMargin)) }
    let q = state.aerodynamics.dynamicPressure
    if q > c.vehicle.maximumDynamicPressure || q <= 0 {
      failures.append(.init(.excessiveDynamicPressure, "MM305 dynamic pressure outside flight envelope", value: q,
        limit: c.vehicle.maximumDynamicPressure))
    }
    if abs(state.aerodynamics.sideslip) > e.maximumSideslip {
      failures.append(.init(.actuatorAuthority, "MM305 sideslip exceeds coordinated flight envelope",
        value: abs(state.aerodynamics.sideslip), limit: e.maximumSideslip))
    }
    return (altitudeMargin, machMargin, failures)
  }

}
