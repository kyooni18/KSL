import Foundation
import KSLCore

/// Contact is observed, not inferred from altitude alone. The immediately preceding
/// airborne sample preserves impact velocity, which KSP may zero after collision.
public final class TouchdownMonitor {
  private let configuration: MissionConfiguration
  private let frame: RunwayFrame
  private var precedingAirborne: VehicleState?
  private var accepted = false
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func observe(_ state: VehicleState) -> [ConstraintViolation] {
    if !state.contact.isGrounded {
      precedingAirborne = state
      return []
    }
    if accepted { return [] }
    guard let prior = precedingAirborne,
      state.time - prior.time <= configuration.guidance.maximumTelemetryAge
    else {
      return [.init(.contactQuality, "No recent airborne sample establishes touchdown")]
    }
    let p = frame.coordinates(of: state.position)
    let v = configuration.vehicle
    var failures: [ConstraintViolation] = []
    if !configuration.runway.contains(along: p.along, cross: p.cross) {
      failures.append(.init(.runwayExcursion, "Touchdown outside runway"))
    }
    if !state.systems.gearDown {
      failures.append(.init(.contactQuality, "Touchdown without deployed gear"))
    }
    if prior.verticalSpeed < -v.maximumTouchdownSink {
      failures.append(
        .init(
          .contactQuality, "Excessive pre-contact sink", value: -prior.verticalSpeed,
          limit: v.maximumTouchdownSink))
    }
    if prior.horizontalSpeed > v.maximumTouchdownSpeed {
      failures.append(
        .init(
          .contactQuality, "Excessive touchdown speed", value: prior.horizontalSpeed,
          limit: v.maximumTouchdownSpeed))
    }
    if abs(prior.aerodynamics.bank) > v.maximumTouchdownBank
      || prior.pitch > v.maximumTouchdownPitch || prior.pitch < radians(-3)
    {
      failures.append(.init(.contactQuality, "Touchdown attitude exceeds gear clearance envelope"))
    }
    accepted = failures.isEmpty
    return failures
  }
}
