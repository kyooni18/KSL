import KSLCore

/// Fifteen C deorbit_planning.inc dispersions, including data-book lift/drag
/// mismatch. The same actual flight computer observes every dispersed plant.
struct DeorbitDispersion: Sendable {
  var timing = 0.0, thrust = 1.0, deltaV = 0.0, mass = 1.0
  var position = 0.0, velocity = 0.0, radial = 0.0, normal = 0.0
  var atmosphere = 1.0, drag = 1.0, lift = 1.0
  static func cases(_ configuration: MissionConfiguration) -> [Self] {
    let c = configuration.deorbitSettings
    // The final two cases are explicitly full-envelope uncertainty stress tests.
    // Never manufacture a 30% minimum when archival evidence does not support it.
    let u = configuration.vehicle.aerodynamicUncertainty
    let t = c.timingUncertainty, f = c.thrustUncertainty, dv = c.deltaVUncertainty
    let m = c.massUncertainty, p = c.positionUncertainty, v = c.velocityUncertainty
    let a = c.pointingUncertainty, rho = c.atmosphereUncertainty
    return [
      .init(),
      .init(timing: t, thrust: 1-f, deltaV: -dv, mass: 1+m, position: p, velocity: -v, radial: a),
      .init(timing: -t, thrust: 1+f, deltaV: dv, mass: 1-m, position: -p, velocity: v, radial: -a),
      .init(timing: t, thrust: 1-f, mass: 1+m, velocity: -v, normal: a),
      .init(timing: -t, thrust: 1+f, mass: 1-m, velocity: v, normal: -a),
      .init(deltaV: -dv, mass: 1+m, position: p, radial: a, atmosphere: 1+rho),
      .init(deltaV: dv, mass: 1-m, position: -p, radial: -a, atmosphere: 1-rho),
      .init(position: p, velocity: v, normal: a, atmosphere: 1+rho),
      .init(position: -p, velocity: -v, normal: -a, atmosphere: 1-rho),
      .init(timing: t, mass: 1+m, position: p, velocity: v, radial: -a, normal: a, atmosphere: 1+rho),
      .init(timing: -t, mass: 1-m, position: -p, velocity: -v, radial: a, normal: -a, atmosphere: 1-rho),
      .init(thrust: 1-f, deltaV: -dv, mass: 1+m, atmosphere: 1+rho),
      .init(thrust: 1+f, deltaV: dv, mass: 1-m, atmosphere: 1-rho),
      .init(drag: 1+u, lift: max(0.5, 1-u)),
      .init(drag: max(0.5, 1-u), lift: 1+u),
    ]
  }
}
