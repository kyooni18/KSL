import Foundation
import KSLCore

/// The lift reserve available while split-fin drag removes kinetic energy.
/// Use the attainable trim/polar envelope, not present lift: steady descent
/// normally uses exactly its support lift and can still have incidence reserve.
public enum TerminalBrakeAuthority {
  public static func availableLift(_ state: VehicleState, configuration c: MissionConfiguration,
                                   aerodynamics aero: AerodynamicsEngine,
                                   identifiedTrimCeiling: Double? = nil,
                                   liftEffectiveness: Double? = nil) -> Double {
    let a = state.aerodynamics
    guard a.measuredForces, a.dynamicPressure > 0, state.mass > 0 else { return 0 }
    let trim = RotationalDynamicsEngine(configuration: c.vehicle, planet: c.planet)
      .supportedTrimAlpha(state, identifiedCeiling: identifiedTrimCeiling)
    let coefficient = aero.coefficients(mach: a.mach, alpha: a.angleOfAttack,
      dynamicPressure: a.dynamicPressure, beta: a.sideslip, gear: state.systems.gearDown)
    guard coefficient.lift > 0 else { return 0 }
    // Never increase nominal capability from one force observation. Degraded
    // measured lift and the guidance estimator may only reduce this reserve.
    let observed = a.lift/(a.dynamicPressure*aero.referenceArea*coefficient.lift)
    let gain = max(0, min(1, observed, liftEffectiveness ?? 1))
    let maximum = aero.liftSolution(acceleration: 0, mass: state.mass,
      dynamicPressure: a.dynamicPressure, mach: a.mach,
      minimumAlpha: c.vehicle.minimumAngleOfAttack,
      maximumAlpha: min(trim, c.vehicle.maximumLandingAngleOfAttack),
      beta: a.sideslip, gear: state.systems.gearDown).maximumAcceleration
    return min(c.vehicle.maximumLoadFactor*9.80665, maximum*gain)
  }
}
