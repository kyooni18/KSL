import Foundation
import KSLCore

/// Admission controls ownership, not whether an advisory search is allowed.
/// A rejected transfer leaves the existing guidance owner flying.
public struct MM305Admission: Codable, Sendable {
  public let accepted: Bool
  public let altitudeMargin: Double
  public let machMargin: Double
  public let headingMargin: Double
  public let planAge: Double
  public let violations: [ConstraintViolation]
  public var physicallyFeasible: Bool? = nil
  public var positionMargin: Double? = nil

  public static func evaluate(_ state: VehicleState, plan: TerminalPlan,
                              configuration c: MissionConfiguration, requireQualification: Bool = true,
                              availableLift: Double? = nil) -> Self {
    let e = c.entrySettings
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let physical = physicalAdmission(state, configuration: c)
    let planAge = state.time - plan.createdAt
    var failures = physical.violations
    let elapsedDistance = max(0, planAge) * state.horizontalSpeed
    var headingMargin = -Double.pi
    var positionMargin = -Double.greatestFiniteMagnitude
    if plan.stations.count >= 2 {
      let inlet = plan.station(at: min(plan.length, elapsedDistance))
      let p = frame.coordinates(of: state.position)
      let direction = frame.direction(at: p.horizontal, tangent: inlet.tangent)
      let headingError = abs(wrapAngle(state.localFrame.heading(of: direction) - state.course))
      headingMargin = e.headingHalfWidth - headingError
      positionMargin = e.handoffRadius - (p.horizontal - inlet.position).length
      if headingMargin < 0 {
        failures.append(.init(.invalidGeometry, "MM305 inlet heading does not match the measured course",
          value: headingMargin))
      }
      if positionMargin < 0 {
        failures.append(.init(.invalidGeometry, "MM305 inlet position no longer matches the measured state",
          value: positionMargin))
      }
      if let availableLift {
        let speed = state.aerodynamics.airspeed
        let g = c.planet.gravitationalParameter / state.position.squaredLength
        let gamma = atan(inlet.slope)
        let vertical = (g - speed * speed / state.position.length) * cos(gamma)
          + speed * speed * inlet.verticalCurvature
        let lateral = state.horizontalSpeed * state.horizontalSpeed * inlet.curvature
        let required = hypot(vertical, lateral)
        if vertical < 0 || abs(atan2(lateral, vertical)) > c.vehicle.maximumBank
          || required > min(availableLift, c.vehicle.maximumLoadFactor * 9.80665) {
          failures.append(.init(.insufficientLift, "MM305 inlet force is unavailable at the current measured state",
            value: required, limit: min(availableLift, c.vehicle.maximumLoadFactor * 9.80665)))
        }
      }
    } else {
      failures.append(.init(.invalidGeometry, "MM305 has no complete terminal geometry"))
    }
    if planAge < 0 || planAge > 2 * c.guidance.terminalReplanInterval {
      failures.append(.init(.telemetry, "MM305 plan no longer matches the measured epoch",
        value: planAge, limit: 2 * c.guidance.terminalReplanInterval))
    }
    let feasible = plan.assessment.feasible && plan.assessment.predictionComplete != false
    if !feasible {
      failures += plan.assessment.violations
      failures.append(.init(.modelUncertainty, "MM305 candidate remains advisory; the current guidance owner retains control"))
    }
    if let nominal = plan.closedLoopCases?.first(where: {
      abs($0.liftMultiplier - 1) < 1e-12 && abs($0.dragMultiplier - 1) < 1e-12
    }), !nominal.feasible {
      failures.append(.init(.modelUncertainty,
        "MM305 nominal closed-loop assessment failed; candidate remains advisory"))
    }
    // A preview or point-mass-priced HAC is an advisory, never an MM305
    // ownership grant. Evaluate *all* requirements, including nominal
    // closed-loop touchdown and rollout, BEFORE computing accepted.
    if requireQualification && !plan.nominalQualified {
      failures.append(.init(.modelUncertainty, "MM305 nominal closed-loop touchdown and rollout unconfirmed (advisory)"))
    }
    let accepted = feasible && failures.isEmpty
    var result = Self(accepted: accepted, altitudeMargin: physical.altitudeMargin,
      machMargin: physical.machMargin, headingMargin: headingMargin, planAge: planAge, violations: failures)
    result.physicallyFeasible = feasible
    result.positionMargin = positionMargin
    return result
  }

  /// Cheap previews may seed the next CLOSED-LOOP qualification, but may not
  /// transfer flight authority. Check the SAME full-state inlet geometry
  /// before paying for a nominal and stress-case terminal rollout.
  static func shouldQualifyPreview(_ state: VehicleState, plan: TerminalPlan,
                                   configuration c: MissionConfiguration) -> Bool {
    !plan.nominalQualified && evaluate(state, plan: plan, configuration: c,
      requireQualification: false).accepted
  }

  /// Operating limits are necessary conditions; they never establish capture.
  /// A return HAC may initially be opening from the runway or inside the entry aim.
  static func physicalAdmission(_ state: VehicleState, configuration c: MissionConfiguration)
    -> (altitudeMargin: Double, machMargin: Double, violations: [ConstraintViolation]) {
    let e = c.entrySettings
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let altitude = state.position.length - c.planet.radius
    let exitASL = ApproachProfile(configuration: c).reference(along: -c.guidance.finalDistance).height
      + c.runway.elevation
    let minimumAltitude = min(e.terminalMinimumAltitude, c.guidance.hacAcquisitionAltitude ?? exitASL, exitASL)
    let altitudeMargin = min(altitude - minimumAltitude, e.terminalMaximumAltitude - altitude)
    // hacAcquisitionMach is MM304's delivery target, not an MM305 floor: the
    // HAC is flown subsonic. The physical lower bound is the configured
    // minimum safe airspeed, checked below.
    let machMargin = e.terminalTargetMach + e.terminalMachHalfWidth - state.aerodynamics.mach
    var failures: [ConstraintViolation] = []
    let range = frame.distance(from: state.position, to: frame.origin)
    if range > c.guidance.maximumPathLength || state.verticalSpeed >= 0 {
      failures.append(.init(.invalidGeometry, "MM305 requires descending acquisition within the configured path horizon",
        value: range, limit: c.guidance.maximumPathLength))
    }
    if state.aerodynamics.airspeed < (c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed) {
      failures.append(.init(.insufficientEnergy, "MM305 airspeed below configured minimum safe speed"))
    }
    if altitudeMargin < 0 { failures.append(.init(.invalidGeometry, "MM305 altitude admission margin", value: altitudeMargin)) }
    if machMargin < 0 { failures.append(.init(.excessiveEnergy, "MM305 Mach admission margin", value: machMargin)) }
    let q = state.aerodynamics.dynamicPressure
    if q > c.vehicle.maximumDynamicPressure || q <= 0 {
      failures.append(.init(.excessiveDynamicPressure, "MM305 dynamic pressure outside flight envelope",
        value: q, limit: c.vehicle.maximumDynamicPressure))
    }
    if abs(state.aerodynamics.bank) > c.vehicle.maximumBank {
      failures.append(.init(.excessiveBank, "MM305 measured bank exceeds the configured envelope",
        value: abs(state.aerodynamics.bank), limit: c.vehicle.maximumBank))
    }
    if abs(state.aerodynamics.sideslip) > e.maximumSideslip {
      failures.append(.init(.actuatorAuthority, "MM305 sideslip exceeds coordinated flight envelope",
        value: abs(state.aerodynamics.sideslip), limit: e.maximumSideslip))
    }
    if let authority = state.controlAuthority {
      if !authority.isValid || authority.evidence == .unavailable
        || authority.minimumAcceleration.x <= 0 || authority.minimumAcceleration.y <= 0 {
        failures.append(.init(.actuatorAuthority, "MM305 has no usable roll/pitch authority estimate"))
      }
    } else {
      failures.append(.init(.actuatorAuthority, "MM305 roll/pitch authority is unavailable"))
    }
    return (altitudeMargin, machMargin, failures)
  }
}
