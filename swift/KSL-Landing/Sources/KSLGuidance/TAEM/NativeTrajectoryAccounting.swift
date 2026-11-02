import KSLCore
import KSLPhysics

/// Additive accounting shared by command prefixes and terminal continuation.
/// Excess integrals are never averaged over flight time or reset at a join.
public struct NativeTrajectoryAccounting: Sendable {
  public private(set) var aerodynamicWork = 0.0
  public private(set) var pressureExcessIntegral = 0.0, loadExcessIntegral = 0.0
  public private(set) var maximumPressureExcess = 0.0, maximumLoadExcess = 0.0
  public init() {}
  public mutating func include(_ step: NativeCommandStep, configuration c: MissionConfiguration) {
    let dt = step.after.time-step.before.time
    func q(_ s: VehicleState) -> Double { max(0, s.aerodynamics.dynamicPressure-c.vehicle.maximumDynamicPressure) }
    func load(_ s: VehicleState) -> Double { max(0, abs(s.aerodynamics.lift)/(s.mass*9.80665)-c.vehicle.maximumLoadFactor) }
    aerodynamicWork += dt/2 * (step.before.aerodynamics.force.dot(step.before.velocity)/step.before.mass
      + step.after.aerodynamics.force.dot(step.after.velocity)/step.after.mass)
    pressureExcessIntegral += dt*(q(step.before)+q(step.after))/2
    loadExcessIntegral += dt*(load(step.before)+load(step.after))/2
    maximumPressureExcess = max(maximumPressureExcess, q(step.before), q(step.after))
    maximumLoadExcess = max(maximumLoadExcess, load(step.before), load(step.after))
  }
}
