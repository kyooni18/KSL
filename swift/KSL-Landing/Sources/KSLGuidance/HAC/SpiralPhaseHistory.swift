import Foundation
import KSLCore

public enum SpiralPhaseStatus: String, Codable, Sendable {
  case available, undefinedPolar, nonmonotonicTime, sampleGap
}

/// Unwrapped polar representation with an explicit initial winding hint.
/// Measured polar work and representation changes are separate. This does not
/// establish path tracking, force attainability or physical spiral completion.
public struct SpiralPhaseHistory: Codable, Sendable {
  public private(set) var spiral: SpiralHAC
  public let initialPhase: Double
  public private(set) var phase: Double
  /// Positive when observed motion decreases remaining polar angle.
  public private(set) var measuredTurn = 0.0
  public private(set) var representationShift = 0.0
  public private(set) var point: Vector2
  public private(set) var time: Double
  public private(set) var status: SpiralPhaseStatus = .available
  private let maximumSampleInterval: Double
  public init(spiral: SpiralHAC, unwrappedHint: Double, point: Vector2,
              time: Double, maximumSampleInterval: Double) throws {
    guard unwrappedHint.isFinite, time.isFinite, maximumSampleInterval.isFinite, maximumSampleInterval > 0,
      let raw = Self.polar(spiral, point) else {
      throw FlightError.invalidTelemetry("Cannot initialize spiral phase without finite epoch/polar coordinate and explicit winding")
    }
    self.spiral = spiral; self.point = point; self.time = time
    self.maximumSampleInterval = maximumSampleInterval
    phase = unwrappedHint+wrapAngle(raw-unwrappedHint)
    initialPhase = phase
  }
  public mutating func observe(point next: Vector2, time nextTime: Double) {
    guard status == .available else { return }
    guard nextTime.isFinite, nextTime >= time else { status = .nonmonotonicTime; return }
    if nextTime == time {
      if next != point { status = .nonmonotonicTime }
      return
    }
    guard nextTime-time <= maximumSampleInterval+nextTime.ulp+time.ulp else { status = .sampleGap; return }
    guard let previous = Self.polar(spiral, point), let raw = Self.polar(spiral, next) else {
      status = .undefinedPolar; return
    }
    let delta = wrapAngle(raw-previous)
    // A half-turn or larger interval is not uniquely unwrapped from endpoints.
    // As with sampled course, adequate cadence beyond this exact ambiguity
    // still requires independent motion/response evidence.
    guard abs(delta) != .pi else { status = .undefinedPolar; return }
    phase += delta; measuredTurn -= delta
    point = next; time = nextTime
  }
  /// Reproject at the identical observed epoch/point. Never hide motion inside
  /// geometry regeneration, switch turn sense for free, or reset the winding.
  public mutating func rebase(to replacement: SpiralHAC, at observedPoint: Vector2,
                              time observedTime: Double) throws {
    guard status == .available, observedPoint == point, observedTime == time,
      replacement.side == spiral.side,
      let previous = Self.polar(spiral, point), let raw = Self.polar(replacement, point) else {
      throw FlightError.invalidTelemetry("Spiral rebase requires the same observed epoch/point and turn sense with valid phase coverage")
    }
    let shift = wrapAngle(raw-previous)
    guard abs(shift) != .pi else { throw FlightError.invalidTelemetry("Ambiguous half-turn spiral rebase") }
    phase += shift; representationShift += shift
    spiral = replacement
  }
  private static func polar(_ spiral: SpiralHAC, _ point: Vector2) -> Double? {
    guard point.x.isFinite, point.y.isFinite else { return nil }
    let x = spiral.exitAlong-point.x, y = spiral.finalRadius-Double(spiral.side)*point.y
    // Near the center, subtracting large chart coordinates cannot resolve a
    // polar direction. This is a relative roundoff guard, not an inner HAC gate.
    let scale = max(abs(spiral.exitAlong), abs(point.x), spiral.finalRadius, abs(point.y))
    guard hypot(x, y) > sqrt(Double.ulpOfOne)*scale else { return nil }
    return atan2(x, y)
  }
}
