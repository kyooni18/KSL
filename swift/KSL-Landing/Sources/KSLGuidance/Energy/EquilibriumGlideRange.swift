import Foundation
import KSLCore
import KSLPhysics

/// Still-air glide distance between two energy heights on the best attainable
/// L/D. Best L/D and its CL change strongly with Mach, so each energy level is
/// placed on the EQUILIBRIUM GLIDE: the altitude where q*S*CL(best L/D)
/// carries the weight with h + V^2/2g = e. Pricing the whole descent with the
/// current (transonic) L/D underestimated the range by a factor of 2-3 in the
/// 1 Nov 2211 and 2244 flights; this model matched the flown distance from
/// the recorded handoff states. Force gains scale the measured polar; the
/// AoA interval is the evidenced attainable one. `derate` applies the model
/// uncertainty: (1-u)/(1+u) for a conservative claim, 1 for nominal.
struct EquilibriumGlideRange {
  let aero: AerodynamicsEngine
  let atmosphere: AtmosphericModelEngine
  let frame: RunwayFrame
  let along: Double, cross: Double
  let time: Double, mass: Double, gravity: Double
  let alphaFloor: Double, alphaCeiling: Double
  var liftGain = 1.0, dragGain = 1.0
  var gear = false

  func range(from top: Double, to bottom: Double, derate: Double, samples: Int = 12) -> Double {
    guard top > bottom, alphaCeiling >= alphaFloor else { return 0 }
    let step = max(1e-3, (alphaCeiling - alphaFloor) / 32)
    func best(height: Double, speed v: Double) -> (ratio: Double, lift: Double) {
      let position = frame.position(along: along, cross: cross, height: height)
      let air = atmosphere.sample(position: position, time: time)
      let q = air.density * v * v / 2, mach = v / max(1, air.speedOfSound)
      var result = (ratio: 0.0, lift: 0.0)
      for alpha in stride(from: alphaFloor, through: alphaCeiling, by: step) {
        let polar = aero.coefficients(mach: mach, alpha: alpha, dynamicPressure: q, beta: 0, gear: gear)
        let ratio = polar.lift * liftGain / max(1e-9, polar.drag * dragGain)
        if ratio > result.ratio {
          result = (ratio, q * aero.referenceArea * polar.lift * liftGain / max(1, mass))
        }
      }
      return result
    }
    var total = 0.0
    for k in 0..<samples {
      let e = top - (top - bottom) * (Double(k) + 0.5) / Double(samples)
      // Lift surplus falls monotonically with altitude at fixed energy.
      var low = 0.0, high = max(0, e)
      for _ in 0..<24 {
        let mid = (low + high) / 2
        if best(height: mid, speed: sqrt(max(0, 2 * gravity * (e - mid)))).lift >= gravity { low = mid } else { high = mid }
      }
      let point = best(height: low, speed: sqrt(max(0, 2 * gravity * (e - low))))
      total += point.ratio * derate * (top - bottom) / Double(samples)
    }
    return total
  }
}
