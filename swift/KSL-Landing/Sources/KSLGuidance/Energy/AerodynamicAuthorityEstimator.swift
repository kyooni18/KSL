import Foundation
import KSLCore
import KSLPhysics

/// Bounded force-ratio observation used for lift inversion, with a two-second
/// time constant. It does not rewrite the polar or silently certify predictions.
/// Simulation supplies plant observations; live flight supplies kRPC force data.
public final class AerodynamicAuthorityEstimator {
  private let model: AerodynamicsEngine
  private var previousTime: Double?
  public private(set) var liftEffectiveness = 1.0
  public private(set) var dragEffectiveness = 1.0
  public init(model: AerodynamicsEngine) { self.model = model }
  public func reset() { previousTime = nil; liftEffectiveness = 1; dragEffectiveness = 1 }
  public func observe(_ state: VehicleState) {
    let dt = min(1, max(0, state.time - (previousTime ?? state.time)))
    previousTime = state.time
    let a = state.aerodynamics
    guard a.dynamicPressure > 200, a.lift > 1000, !state.contact.isGrounded else { return }
    let c = model.coefficients(mach: a.mach, alpha: a.angleOfAttack,
      dynamicPressure: a.dynamicPressure, beta: a.sideslip,
      gear: state.systems.gearDown, brakes: state.systems.brakes, airbrakes: state.systems.airbrakes,
      speedbrakeFraction: state.systems.speedbrakeFraction ?? 0)
    let scale = a.dynamicPressure * model.referenceArea
    let filter = 1 - exp(-dt / 2)
    if c.lift > 0.05 {
      let observed = clamp(a.lift / (scale * c.lift), 0.5, 1.5)
      liftEffectiveness += filter * (observed - liftEffectiveness)
    }
    if c.drag > 0.02 {
      let observed = clamp(a.drag / (scale * c.drag), 0.5, 1.5)
      dragEffectiveness += filter * (observed - dragEffectiveness)
    }
  }
}
