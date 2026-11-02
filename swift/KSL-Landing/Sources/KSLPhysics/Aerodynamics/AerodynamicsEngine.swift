import Foundation
import KSLCore

public struct AerodynamicCoefficients: Sendable {
  public let lift: Double
  public let drag: Double
}
public struct LiftSolution: Sendable {
  public let alpha: Double
  public let achievable: Bool
  public let maximumAcceleration: Double
}

/// Immutable measured-table model. No online mutation of an already committed plan's model.
public final class AerodynamicsEngine: Sendable {
  /// Runtime identity of this immutable calibrated model, not a resource hash.
  public let instanceID = UUID()
  public let referenceArea: Double
  public let liftScale: Double
  public let dragScale: Double
  public let maximumLiftToDragRatio: Double
  public let history: HistoricalAerodynamics?
  public let speedbrakeFullDeploymentDragArea: Double?
  private let machs: [Double]
  private let alphas: [Double]
  /// Exact lift-search breakpoints are immutable for one physical model.
  /// Constructing and sorting this set for every HAC path station dominated
  /// repeated entry/terminal previews; keep the identical knots once.
  private let liftKnots: [Double]
  private let liftTable: [[Double]]
  private let dragTable: [[Double]]
  private init(copy: AerodynamicsEngine, liftMultiplier: Double, dragMultiplier: Double) {
    referenceArea = copy.referenceArea; history = copy.history
    speedbrakeFullDeploymentDragArea = copy.speedbrakeFullDeploymentDragArea
    machs = copy.machs; alphas = copy.alphas; liftKnots = copy.liftKnots
    liftTable = copy.liftTable; dragTable = copy.dragTable
    liftScale = copy.liftScale * liftMultiplier; dragScale = copy.dragScale * dragMultiplier
    maximumLiftToDragRatio = copy.maximumLiftToDragRatio * liftMultiplier / dragMultiplier
  }
  public func scaled(lift: Double, drag: Double) -> AerodynamicsEngine {
    precondition(lift.isFinite && drag.isFinite && lift > 0 && drag > 0)
    return .init(copy: self, liftMultiplier: lift, dragMultiplier: drag)
  }
  public init(
    referenceArea: Double = 250, liftScale: Double = 1, dragScale: Double = 1, csv: String? = nil,
    history: HistoricalAerodynamics? = nil, speedbrakeFullDeploymentDragArea: Double? = nil
  ) throws {
    guard referenceArea > 0, liftScale > 0, dragScale > 0 else {
      throw FlightError.invalidConfiguration("Invalid aerodynamic scaling")
    }
    self.referenceArea = referenceArea
    self.liftScale = liftScale
    self.dragScale = dragScale
    self.history = history
    self.speedbrakeFullDeploymentDragArea = speedbrakeFullDeploymentDragArea
    let contents: String
    if let csv {
      contents = csv
    } else {
      guard let url = Bundle.module.url(forResource: "STSAerodynamics", withExtension: "csv") else {
        throw FlightError.io("Missing STS polar")
      }
      contents = try String(contentsOf: url, encoding: .utf8)
    }
    var rows: [[Double]] = []
    for line in contents.split(whereSeparator: \.isNewline) {
      if line.hasPrefix("#") || line.hasPrefix("mach") { continue }
      let values = line.split(separator: ",", omittingEmptySubsequences: false).compactMap {
        Double($0.trimmingCharacters(in: .whitespaces))
      }
      guard values.count == 4, values.allSatisfy(\.isFinite), values[3] >= 0 else {
        throw FlightError.invalidConfiguration("Malformed polar row")
      }
      rows.append(values)
    }
    machs = Array(Set(rows.map { $0[0] })).sorted()
    alphas = Array(Set(rows.map { radians($0[1]) })).sorted()
    // Sample alphas bracket the history kernel's lift maximum. Quantize them to
    // 0.25 deg (well under the kernel's 10-deg support) so liftSolution scans
    // hundreds of knots rather than one per archived sample.
    let knotStep = radians(0.25)
    let sampleKnots = Set((history?.history.samples ?? []).map { (($0.observation.alpha) / knotStep).rounded() * knotStep })
    liftKnots = Array(Set(alphas).union(sampleKnots)).sorted()
    guard machs.count >= 2, alphas.count >= 2, rows.count == machs.count * alphas.count else {
      throw FlightError.invalidConfiguration("Incomplete aerodynamic grid")
    }
    var cls = Array(
      repeating: Array(repeating: Double.nan, count: alphas.count), count: machs.count)
    var cds = cls
    for row in rows {
      let i = machs.firstIndex(of: row[0])!
      let j = alphas.firstIndex(of: radians(row[1]))!
      guard cls[i][j].isNaN else {
        throw FlightError.invalidConfiguration("Duplicate aerodynamic cell")
      }
      cls[i][j] = row[2]
      cds[i][j] = row[3]
    }
    liftTable = cls
    dragTable = cds
    maximumLiftToDragRatio =
      rows.map { max(0, $0[2]) * liftScale / max(1e-8, $0[3] * dragScale) }.max() ?? 0
  }
  private func bracket(_ xs: [Double], _ x: Double) -> (Int, Double) {
    var low = 0
    var high = xs.count - 1
    while high - low > 1 {
      let mid = (low + high) / 2
      if xs[mid] <= x { low = mid } else { high = mid }
    }
    return (low, clamp((x - xs[low]) / (xs[low + 1] - xs[low]), 0, 1))
  }
  public func coefficients(mach: Double, alpha: Double, dynamicPressure q: Double? = nil,
                           beta: Double = 0, gear: Bool = false, brakes: Bool = false,
                           airbrakes: Bool = false, speedbrakeFraction: Double = 0) -> AerodynamicCoefficients {
    let historicalBrake = airbrakes || (speedbrakeFullDeploymentDragArea == nil && speedbrakeFraction >= 1)
    if let q, let sample = history?.force(q: q, mach: mach, alpha: alpha, beta: beta,
      gear: gear, brakes: brakes, airbrakes: historicalBrake) {
      let brakeDrag = (speedbrakeFullDeploymentDragArea ?? 0) * speedbrakeFraction / referenceArea
      return .init(lift: sample.forcePerDynamicPressure.y * liftScale / referenceArea,
        drag: sample.forcePerDynamicPressure.x * dragScale / referenceArea + brakeDrag)
    }
    let (i, t) = bracket(machs, mach)
    let (j, u) = bracket(alphas, alpha)
    func interpolate(_ table: [[Double]]) -> Double {
      (table[i][j] * (1 - u) + table[i][j + 1] * u) * (1 - t)
        + (table[i + 1][j] * (1 - u) + table[i + 1][j + 1] * u) * t
    }
    let brakeDrag = (speedbrakeFullDeploymentDragArea ?? 0) * speedbrakeFraction / referenceArea
    return .init(lift: interpolate(liftTable) * liftScale,
      drag: max(0, interpolate(dragTable) * dragScale) + brakeDrag)
  }
  public func evaluate(
    position: Vector3, velocity: Vector3, attitude: Quaternion, atmosphere: AtmosphericState,
    gear: Bool = false, brakes: Bool = false, airbrakes: Bool = false, speedbrakeFraction: Double = 0
  ) -> AerodynamicState {
    let va = velocity - atmosphere.wind
    let speed = va.length
    var result = AerodynamicState()
    result.airspeed = speed
    result.mach = speed / max(1, atmosphere.speedOfSound)
    result.dynamicPressure = 0.5 * atmosphere.density * speed * speed
    guard speed > 1e-9 else { return result }
    let f = va / speed
    let body = attitude.conjugate.rotate(f)
    result.angleOfAttack = atan2(body.z, body.x)
    result.sideslip = asin(clamp(body.y, -1, 1))
    let windRight = f.cross(position.normalized).normalized
    let windUp = windRight.cross(f).normalized
    let bodyUp = attitude.rotate(.init(0, 0, -1))
    result.bank = atan2(bodyUp.dot(windRight), bodyUp.dot(windUp))
    let historicalBrake = airbrakes || (speedbrakeFullDeploymentDragArea == nil && speedbrakeFraction >= 1)
    let c = coefficients(mach: result.mach, alpha: result.angleOfAttack,
      speedbrakeFraction: speedbrakeFraction)
    let observed = history?.force(q: result.dynamicPressure, mach: result.mach,
      alpha: result.angleOfAttack, beta: result.sideslip, gear: gear, brakes: brakes,
      airbrakes: historicalBrake)
    result.lift = result.dynamicPressure * (observed.map { $0.forcePerDynamicPressure.y * liftScale } ?? (referenceArea * c.lift))
    result.drag = result.dynamicPressure * (observed.map {
      $0.forcePerDynamicPressure.x * dragScale + (speedbrakeFullDeploymentDragArea ?? 0)*speedbrakeFraction
    } ?? (referenceArea * c.drag))
    result.modelEvidence = observed == nil ? .provisional : .measured
    result.modelConfidence = observed?.localityConfidence
    let liftDirection = windUp * cos(result.bank) + windRight * sin(result.bank)
    let sideDirection = windRight * cos(result.bank) - windUp * sin(result.bank)
    let sideMagnitude = observed.map { $0.forcePerDynamicPressure.z * result.dynamicPressure * liftScale }
      ?? (-result.dynamicPressure * referenceArea * 0.25 * result.sideslip)
    let sideForce = sideDirection * sideMagnitude
    result.force = liftDirection * result.lift - f * result.drag + sideForce
    return result
  }
  public func liftSolution(
    acceleration: Double, mass: Double, dynamicPressure q: Double, mach: Double,
    minimumAlpha: Double = 0, maximumAlpha: Double = radians(40),
    beta: Double = 0, gear: Bool = false, brakes: Bool = false
  ) -> LiftSolution {
    let factor = q * referenceArea / max(1, mass)
    // Boundary knots and all interior polar/history breakpoints are unchanged.
    // Unlike sorting a new Set on every evaluation, these are already ordered.
    let knots = minimumAlpha < maximumAlpha
      ? [minimumAlpha] + liftKnots.filter { $0 > minimumAlpha && $0 < maximumAlpha } + [maximumAlpha]
      : [minimumAlpha]
    func lift(_ alpha: Double) -> Double {
      coefficients(mach: mach, alpha: alpha, dynamicPressure: q, beta: beta,
        gear: gear, brakes: brakes).lift * factor
    }
    var maximum = 0.0
    var best = minimumAlpha
    for alpha in knots {
      let value = lift(alpha)
      if value > maximum {
        maximum = value
        best = alpha
      }
    }
    for (a, b) in zip(knots, knots.dropFirst()) {
      let la = lift(a)
      let lb = lift(b)
      if acceleration <= la {
        return .init(alpha: a, achievable: true, maximumAcceleration: maximum)
      }
      if lb >= acceleration && lb > la {
        if history == nil {
          return .init(alpha: a + (b - a) * (acceleration - la) / (lb - la),
                       achievable: true, maximumAcceleration: maximum)
        }
        var low = a, high = b
        for _ in 0..<20 {
          let mid = (low + high) / 2
          if lift(mid) < acceleration { low = mid } else { high = mid }
        }
        return .init(alpha: high, achievable: true, maximumAcceleration: maximum)
      }
    }
    return .init(alpha: best, achievable: acceleration <= maximum, maximumAcceleration: maximum)
  }
  public func bestGlideAlpha(mach: Double, maximumAlpha: Double = radians(30),
                             dynamicPressure: Double? = nil, gear: Bool = false) -> Double {
    var best = radians(10)
    var ratio = -Double.infinity
    if history == nil || dynamicPressure == nil {
      // At an alpha knot the bilinear polar is just a Mach interpolation.
      // Reuse that bracket instead of searching the same grids at every knot.
      let (i, t) = bracket(machs, mach)
      for (j, a) in alphas.enumerated() where a >= 0 && a <= maximumAlpha {
        let lift = (liftTable[i][j]*(1-t)+liftTable[i+1][j]*t)*liftScale
        let drag = max(0, (dragTable[i][j]*(1-t)+dragTable[i+1][j]*t)*dragScale)
        let ld = lift/max(drag, 1e-8)
        if ld > ratio { ratio = ld; best = a }
      }
      return best
    }
    for a in alphas where a >= 0 && a <= maximumAlpha {
      let c = coefficients(mach: mach, alpha: a, dynamicPressure: dynamicPressure, gear: gear)
      let ld = c.lift / max(c.drag, 1e-8)
      if ld > ratio {
        ratio = ld
        best = a
      }
    }
    return best
  }
}
