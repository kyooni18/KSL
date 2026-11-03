import Foundation
import KSLCore
import KSLPhysics

/// Instantaneous sampled command seed, not achieved force or admission.
public struct TAEMNormalForceFit: Codable, Sendable {
  public let target: GuidanceTarget
  /// Wind-normal lift components: x is vertical, y is rightward (m/s²).
  public let requested: Vector2, relaxedProjection: Vector2, modeled: Vector2, residual: Vector2
  public let modeledDragAcceleration: Double
  public let evaluatedIncidences: Int
  public let modelID: UUID
  public let stateTime: Double
  public let minimumAlpha: Double, maximumAlpha: Double
  /// Attainable part of the explicitly requested support floor; nil for joint pricing.
  public var supportReserve: Double? = nil
}

public enum TAEMNormalForceFitter {
  /// Samples real allowed incidence choices and forward-evaluates each polar.
  /// Required trim ceiling and optional extra samples are explicit caller
  /// inputs, not inferred empirical support. Nil means no admissible sampled
  /// command; it is not proof of physical infeasibility of every command.
  public static func fit(vertical: Double, lateral: Double, state: VehicleState,
                         configuration c: MissionConfiguration, aerodynamics aero: AerodynamicsEngine,
                         maximumAlpha: Double, additionalAlphas: [Double] = [],
                         minimumVertical: Double? = nil,
                         baseTarget: GuidanceTarget) throws -> TAEMNormalForceFit? {
    let a = state.aerodynamics, floor = c.vehicle.minimumAngleOfAttack
    guard vertical.isFinite, lateral.isFinite, state.time.isFinite, state.mass.isFinite, state.mass > 0,
      a.dynamicPressure.isFinite, a.dynamicPressure >= 0, a.mach.isFinite, a.mach >= 0, a.sideslip.isFinite,
      maximumAlpha.isFinite, maximumAlpha >= floor, maximumAlpha <= c.vehicle.maximumAngleOfAttack,
      minimumVertical.map({ $0.isFinite && $0 >= 0 }) ?? true,
      additionalAlphas.allSatisfy({ $0.isFinite && $0 >= floor && $0 <= maximumAlpha }),
      [baseTarget.desiredSpeed, baseTarget.desiredHeight, baseTarget.desiredFlightPath,
        baseTarget.speedbrakeFraction ?? 0].allSatisfy(\.isFinite) else {
      throw FlightError.invalidTelemetry("Invalid force-fit state or explicit incidence coverage")
    }
    let peakSolution = aero.liftSolution(acceleration: .greatestFiniteMagnitude, mass: state.mass, dynamicPressure: a.dynamicPressure,
      mach: a.mach, minimumAlpha: floor, maximumAlpha: maximumAlpha,
      beta: a.sideslip, gear: state.systems.gearDown, brakes: state.systems.brakes)
    let peak = peakSolution.maximumAcceleration
    let loadLimit = c.vehicle.maximumLoadFactor*9.80665
    let envelope = NormalForceEnvelope(maximumLift: min(peak, loadLimit), maximumBank: c.vehicle.maximumBank)
    guard let joint = envelope.projectJointly(vertical: vertical, lateral: lateral) else { return nil }
    // Only an unpriced recovery opts into this physical support constraint.
    // If support itself is unattainable, use all attainable lift upright;
    // do not manufacture authority or spend it chasing a geometric turn.
    let reserve = minimumVertical.map { min(envelope.maximumLift, $0) }
    let projected = reserve.map { envelope.allocate(vertical: $0, lateral: lateral) } ?? joint
    let inverse = aero.liftSolution(acceleration: projected.lift, mass: state.mass,
      dynamicPressure: a.dynamicPressure, mach: a.mach, minimumAlpha: floor, maximumAlpha: maximumAlpha,
      beta: a.sideslip, gear: state.systems.gearDown, brakes: state.systems.brakes)
    var alphas = [floor, maximumAlpha, inverse.alpha]+additionalAlphas
    if reserve != nil { alphas.append(peakSolution.alpha) }
    if a.angleOfAttack.isFinite && a.angleOfAttack >= floor && a.angleOfAttack <= maximumAlpha { alphas.append(a.angleOfAttack) }
    alphas = Array(Set(alphas)).sorted()
    // For a nonzero fixed polar lift, angle projection remains appropriate
    // even when the relaxed sector projects onto its zero-lift apex.
    let desiredBank = clamp(atan2(lateral, vertical), -c.vehicle.maximumBank, c.vehicle.maximumBank)
    var best: (alpha: Double, bank: Double, force: Vector2, drag: Double, error: Double)?
    var evaluated = 0
    for alpha in alphas {
      let polar = aero.coefficients(mach: a.mach, alpha: alpha, dynamicPressure: a.dynamicPressure,
        beta: a.sideslip, gear: state.systems.gearDown, brakes: state.systems.brakes,
        airbrakes: state.systems.airbrakes, speedbrakeFraction: state.systems.speedbrakeFraction ?? 0)
      let lift = polar.lift*a.dynamicPressure*aero.referenceArea/state.mass
      let drag = polar.drag*a.dynamicPressure*aero.referenceArea/state.mass
      evaluated += 1
      // This positive-lift seed family does not claim negative-lift recovery.
      // Never clamp modeled lift down to a load limit without changing alpha.
      guard lift.isFinite, lift >= 0, lift <= loadLimit, drag.isFinite, drag >= 0 else { continue }
      var bank = desiredBank
      if let reserve, reserve > 0 {
        // Relative roundoff only: evaluating the same peak polar may differ
        // by a few ulps from the inverse solver's cached maximum.
        guard lift >= reserve * (1 - 8 * Double.ulpOfOne) else { continue }
        let supportBank = acos(clamp(reserve / max(lift, Double.leastNormalMagnitude), 0, 1))
        bank = clamp(bank, -supportBank, supportBank)
      }
      let force = Vector2(lift*cos(bank), lift*sin(bank))
      let error = hypot(force.x-vertical, force.y-lateral)
      guard error.isFinite else { continue }
      if best == nil || error < best!.error || (error == best!.error && drag < best!.drag) {
        best = (alpha, lift > 0 ? bank : 0, force, drag, error)
      }
    }
    guard let best else { return nil }
    var target = baseTarget
    target.angleOfAttack = best.alpha; target.bank = best.bank
    return .init(target: target, requested: .init(vertical, lateral),
      relaxedProjection: .init(projected.vertical, projected.lateral), modeled: best.force,
      residual: .init(best.force.x-vertical, best.force.y-lateral),
      modeledDragAcceleration: best.drag, evaluatedIncidences: evaluated, modelID: aero.instanceID,
      stateTime: state.time, minimumAlpha: floor, maximumAlpha: maximumAlpha, supportReserve: reserve)
  }
}
