import Foundation
import KSLCore

/// Exact stock KSP curve keys used by f1eebdc vehicle/math.c. Only a
/// lifting-surface prior: body lift/drag cubes are not fabricated here.
struct StockLiftingSurfacePrior {
  let lift: HermiteCurve, liftMach: HermiteCurve, drag: HermiteCurve, dragMach: HermiteCurve
  init() throws {
    lift = try .init(rows: [[0,0,0,1.965926], [0.258819,0.5114774,1.990092,1.905806],
      [0.5,0.9026583,0.7074468,-0.7074468], [0.7071068,0.5926583,-2.087948,-1.990095],
      [1,0,-2.014386,-2.014386]])
    liftMach = try .init(rows: [[0,1,0,0], [0.3,0.5,-1.671345,-0.8273422],
      [1,0.125,-0.0005291355,-0.02625772], [5,0.0625,0,0], [25,0.05,0,0]])
    drag = try .init(rows: [[0,0.01,0,0], [0.3420201,0.06,0.1750731,0.1750731],
      [0.5,0.24,2.60928,2.60928], [0.7071068,1.7,3.349777,3.349777], [1,2.4,1.387938,0]])
    dragMach = try .init(rows: [[0,0.35,0,-0.8463008], [0.15,0.125,0,0],
      [0.9,0.275,0.541598,0.541598], [1.1,0.75,0,0], [1.4,0.4,-0.3626955,-0.3626955],
      [1.6,0.35,-0.1545923,-0.1545923], [2,0.3,-0.09013031,-0.09013031],
      [5,0.22,0,0], [25,0.3,0.0006807274,0]])
  }
  func coefficients(mach: Double, alpha: Double) -> (lift: Double, drag: Double) {
    let flow = sin(clamp(abs(alpha), 0, .pi/2)), m = max(0, mach)
    return (copysign(max(0, lift.value(at: flow)*liftMach.value(at: m)), alpha),
      max(0, drag.value(at: flow)*dragMach.value(at: m)))
  }
  func factors(mach: Double, incidence: Double, referenceAlpha: Double) -> (lift: Double, drag: Double) {
    let raw = coefficients(mach: mach, alpha: incidence)
    let reference = coefficients(mach: 5, alpha: referenceAlpha)
    return (reference.lift != 0 ? raw.lift/abs(reference.lift) : raw.lift,
      max(Double.leastNormalMagnitude, reference.drag > 0 ? raw.drag/reference.drag : raw.drag))
  }
}
