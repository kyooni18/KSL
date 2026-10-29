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
  public let referenceArea: Double
  public let liftScale: Double
  public let dragScale: Double
  public let maximumLiftToDragRatio: Double
  private let machs: [Double]
  private let alphas: [Double]
  private let liftTable: [[Double]]
  private let dragTable: [[Double]]
  public init(
    referenceArea: Double = 250, liftScale: Double = 1, dragScale: Double = 1, csv: String? = nil
  ) throws {
    guard referenceArea > 0, liftScale > 0, dragScale > 0 else {
      throw FlightError.invalidConfiguration("Invalid aerodynamic scaling")
    }
    self.referenceArea = referenceArea
    self.liftScale = liftScale
    self.dragScale = dragScale
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
  public func coefficients(mach: Double, alpha: Double) -> AerodynamicCoefficients {
    let (i, t) = bracket(machs, mach)
    let (j, u) = bracket(alphas, alpha)
    func interpolate(_ table: [[Double]]) -> Double {
      (table[i][j] * (1 - u) + table[i][j + 1] * u) * (1 - t)
        + (table[i + 1][j] * (1 - u) + table[i + 1][j + 1] * u) * t
    }
    return .init(
      lift: interpolate(liftTable) * liftScale, drag: max(0, interpolate(dragTable) * dragScale))
  }
  public func evaluate(
    position: Vector3, velocity: Vector3, attitude: Quaternion, atmosphere: AtmosphericState
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
    let c = coefficients(mach: result.mach, alpha: result.angleOfAttack)
    result.lift = result.dynamicPressure * referenceArea * c.lift
    result.drag = result.dynamicPressure * referenceArea * c.drag
    let right = attitude.rotate(.init(0, 1, 0))
    let liftDirection = right.cross(f).normalized
    let sideForce = right * (-result.dynamicPressure * referenceArea * 0.25 * result.sideslip)
    result.force = liftDirection * result.lift - f * result.drag + sideForce
    return result
  }
  public func liftSolution(
    acceleration: Double, mass: Double, dynamicPressure q: Double, mach: Double,
    minimumAlpha: Double = 0, maximumAlpha: Double = radians(40)
  ) -> LiftSolution {
    let factor = q * referenceArea / max(1, mass)
    let knots =
      ([minimumAlpha] + alphas.filter { $0 > minimumAlpha && $0 < maximumAlpha } + [maximumAlpha])
      .sorted()
    var maximum = 0.0
    var best = minimumAlpha
    for alpha in knots {
      let lift = coefficients(mach: mach, alpha: alpha).lift * factor
      if lift > maximum {
        maximum = lift
        best = alpha
      }
    }
    for (a, b) in zip(knots, knots.dropFirst()) {
      let la = coefficients(mach: mach, alpha: a).lift * factor
      let lb = coefficients(mach: mach, alpha: b).lift * factor
      if acceleration <= la {
        return .init(alpha: a, achievable: true, maximumAcceleration: maximum)
      }
      if lb >= acceleration && lb > la {
        return .init(
          alpha: a + (b - a) * (acceleration - la) / (lb - la), achievable: true,
          maximumAcceleration: maximum)
      }
    }
    return .init(alpha: best, achievable: acceleration <= maximum, maximumAcceleration: maximum)
  }
  public func bestGlideAlpha(mach: Double, maximumAlpha: Double = radians(30)) -> Double {
    var best = radians(10)
    var ratio = -Double.infinity
    for a in alphas where a >= 0 && a <= maximumAlpha {
      let c = coefficients(mach: mach, alpha: a)
      let ld = c.lift / max(c.drag, 1e-8)
      if ld > ratio {
        ratio = ld
        best = a
      }
    }
    return best
  }
}
