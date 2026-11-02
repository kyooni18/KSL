import Foundation

/// Independent measured-force protection for terminal split-rudder feedback.
/// This permits closed-loop drag control, not an assumed drag benefit in a
/// forecast. Unknown deployment authority must remain unpriced by planners.
public enum TerminalBrakeEnvelope {
  /// OV-100 rudder: ctrlSurfaceRange=35 deg, actuatorSpeed=40 deg/s in
  /// Benjee10_shuttleOrbiter/Parts/OV_rudder.cfg (both deployment channels).
  public static let travelSeconds = 35.0 / 40.0

  public static func permits(_ state: VehicleState, configuration c: MissionConfiguration,
                            availableLiftAcceleration: Double) -> Bool {
    let a = state.aerodynamics
    guard state.contact == .airborne, state.systems.unpowered,
      a.measuredForces, a.lift > 0, a.drag > 0, state.mass > 0,
      a.dynamicPressure > 0, a.dynamicPressure <= c.vehicle.maximumDynamicPressure,
      a.airspeed > 0, availableLiftAcceleration.isFinite, availableLiftAcceleration > 0,
      let fraction = state.systems.speedbrakeFraction,
      fraction.isFinite, (0...1).contains(fraction) else { return false }
    let g = c.planet.gravitationalParameter/state.position.squaredLength
    let normal = availableLiftAcceleration * cos(a.bank) * (1-c.vehicle.aerodynamicUncertainty)
    let support = max(0, g*cos(state.flightPathAngle))
    guard normal > 0 else { return false }
    let supportSpeedSquared = a.airspeed*a.airspeed*support/normal
    let floor = c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed
    let reserve = max(floor*floor, supportSpeedSquared)
    // Retain the drag work consumed while a fully deployed fin retracts.
    // Using the current drag for the whole travel is conservative as it stows.
    let retractionWork = a.drag/state.mass*a.airspeed*travelSeconds
    return (a.airspeed*a.airspeed-reserve)/2 > retractionWork
  }
}
