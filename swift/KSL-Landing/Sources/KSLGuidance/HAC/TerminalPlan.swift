import Foundation
import KSLCore

public enum PathSegment: String, Codable, Sendable { case acquisition, spiral, final }
public struct PathStation: Codable, Sendable {
  public let distance: Double
  public let position: Vector2
  public let tangent: Vector2
  public let curvature: Double
  public let height: Double
  public let slope: Double
  public let verticalCurvature: Double
  public let speed: Double
  public let angleOfAttack: Double
  public let bank: Double
  public let segment: PathSegment
  public init(
    distance: Double, position: Vector2, tangent: Vector2, curvature: Double, height: Double,
    slope: Double,
    verticalCurvature: Double = 0, speed: Double = 0, angleOfAttack: Double = 0, bank: Double = 0,
    segment: PathSegment
  ) {
    self.distance = distance
    self.position = position
    self.tangent = tangent
    self.curvature = curvature
    self.height = height
    self.slope = slope
    self.verticalCurvature = verticalCurvature
    self.speed = speed
    self.angleOfAttack = angleOfAttack
    self.bank = bank
    self.segment = segment
  }
}

public struct TerminalAssessment: Codable, Sendable {
  public let feasible: Bool
  public let violations: [ConstraintViolation]
  public let touchdownSpeed: Double
  public let minimumLiftMargin: Double
  public let maximumBank: Double
  public let maximumQ: Double
  public let maximumLoad: Double
  public let score: Double
  public let uncertaintyFraction: Double
}

public struct TerminalStressCase: Codable, Sendable {
  public let liftMultiplier: Double
  public let dragMultiplier: Double
  public let feasible: Bool
  public let touchdownSpeed: Double
  public let violations: [ConstraintViolation]
}

public struct TerminalPlan: Codable, Sendable {
  public let identifier: Int
  public let createdAt: Double
  public let spiral: SpiralHAC?
  public let stations: [PathStation]
  public let assessment: TerminalAssessment
  public let stressCases: [TerminalStressCase]
  public var length: Double { stations.last?.distance ?? 0 }
  public init(
    identifier: Int, createdAt: Double, spiral: SpiralHAC?, stations: [PathStation],
    assessment: TerminalAssessment, stressCases: [TerminalStressCase] = []
  ) {
    self.identifier = identifier
    self.createdAt = createdAt
    self.spiral = spiral
    self.stations = stations
    self.assessment = assessment
    self.stressCases = stressCases
  }
  public func nearest(to p: Vector2, startingAt: Int = 0) -> Int {
    guard !stations.isEmpty else { return 0 }
    var best = min(max(0, startingAt), stations.count - 1)
    var distance = Double.infinity
    // Monotonic index window avoids jumping to an overlapping wraparound revolution.
    let end = min(stations.count, best + 16)
    for i in max(0, best - 3)..<end {
      let d = (stations[i].position - p).length
      if d < distance {
        distance = d
        best = i
      }
    }
    return best
  }
  public func station(at distance: Double) -> PathStation {
    if distance <= 0 { return stations[0] }
    if distance >= length { return stations[stations.count - 1] }
    var lo = 0
    var hi = stations.count - 1
    while hi - lo > 1 {
      let mid = (lo + hi) / 2
      if stations[mid].distance < distance { lo = mid } else { hi = mid }
    }
    let a = stations[lo]
    let b = stations[hi]
    let u = (distance - a.distance) / max(1e-8, b.distance - a.distance)
    func blend(_ x: Double, _ y: Double) -> Double { x + (y - x) * u }
    return .init(
      distance: distance, position: a.position + (b.position - a.position) * u,
      tangent: (a.tangent + (b.tangent - a.tangent) * u).normalized,
      curvature: blend(a.curvature, b.curvature),
      height: blend(a.height, b.height), slope: blend(a.slope, b.slope),
      verticalCurvature: blend(a.verticalCurvature, b.verticalCurvature),
      speed: blend(a.speed, b.speed), angleOfAttack: blend(a.angleOfAttack, b.angleOfAttack),
      bank: blend(a.bank, b.bank), segment: b.segment)
  }
}
