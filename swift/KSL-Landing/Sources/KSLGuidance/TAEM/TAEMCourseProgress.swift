import Foundation
import KSLCore

public enum TAEMCourseProgressStatus: String, Codable, Sendable {
  case available, undefinedCourse, nonmonotonicTime, sampleGap, ambiguousHalfTurn, invalidCadence
}

/// Sampled ground-course work in the fixed selected-runway chart. Positive is
/// RIGHT. Independent of every proposed spiral center/radius/winding label.
/// This is not spiral polar phase and does not establish an achieved spiral.
public struct TAEMCourseProgress: Sendable {
  private let frame: RunwayFrame
  private let maximumSampleInterval: Double
  private var previousPosition: Vector3
  private var previousVelocity: Vector3
  public private(set) var status: TAEMCourseProgressStatus
  public private(set) var throughTime: Double
  public private(set) var wrappedCourse: Double
  public private(set) var signedChange = 0.0
  public private(set) var absoluteChange = 0.0
  public init(frame: RunwayFrame, state: VehicleState, maximumSampleInterval: Double) {
    self.frame = frame; self.maximumSampleInterval = maximumSampleInterval
    previousPosition = state.position; previousVelocity = state.velocity
    throughTime = state.time
    let v = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
    let valid = state.time.isFinite && Self.resolvesCourse(v, velocity: state.velocity)
    wrappedCourse = valid ? atan2(v.y, v.x) : 0
    status = maximumSampleInterval.isFinite && maximumSampleInterval > 0
      ? (valid ? .available : .undefinedCourse) : .invalidCadence
  }
  public mutating func observe(_ state: VehicleState) {
    // Once coverage is lost, a later sample cannot restore missed winding.
    guard status == .available else { return }
    guard state.time.isFinite, state.time >= throughTime else { status = .nonmonotonicTime; return }
    if state.time == throughTime {
      if state.position != previousPosition || state.velocity != previousVelocity { status = .nonmonotonicTime }
      return
    }
    // Declared sampling coverage is not a physical heading-rate bound.
    // The ulp allowance only covers timestamp subtraction roundoff.
    guard state.time-throughTime <= maximumSampleInterval+state.time.ulp+throughTime.ulp else { status = .sampleGap; return }
    let v = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
    guard Self.resolvesCourse(v, velocity: state.velocity) else { status = .undefinedCourse; return }
    let course = atan2(v.y, v.x)
    let delta = wrapAngle(course-wrappedCourse)
    // Exactly opposite samples do not determine which half-turn was flown.
    // No arbitrary angle/speed threshold substitutes for that missing evidence.
    guard abs(delta) != .pi else { status = .ambiguousHalfTurn; return }
    signedChange += delta; absoluteChange += abs(delta)
    wrappedCourse = course; throughTime = state.time
    previousPosition = state.position; previousVelocity = state.velocity
  }
  private static func resolvesCourse(_ chart: Vector2, velocity: Vector3) -> Bool {
    // Near radial/zero velocity, course divides by a numerically unresolved
    // horizontal projection. sqrt(epsilon) is a relative precision guard,
    // not a minimum flight speed or turn-authority threshold.
    chart.x.isFinite && chart.y.isFinite && chart.length.isFinite && velocity.length.isFinite
      && chart.length > sqrt(Double.ulpOfOne)*velocity.length
  }

}
