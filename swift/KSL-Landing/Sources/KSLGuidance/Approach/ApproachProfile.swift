import Foundation
import KSLCore

public struct VerticalReference: Sendable {
  public let height: Double
  public let slope: Double
  public let secondDerivative: Double
  public init(height: Double, slope: Double, secondDerivative: Double = 0) {
    self.height = height
    self.slope = slope
    self.secondDerivative = secondDerivative
  }
}

/// Continuous steep/preflare/shallow geometry, in meters above the surveyed runway.
public final class ApproachProfile: Sendable {
  public let configuration: MissionConfiguration
  public let preflareAlong: Double
  public let shallowAlong: Double
  public let preflareHeight: Double
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration
    shallowAlong =
      configuration.guidance.touchdownAlong - configuration.guidance.shallowSegmentLength
    // Radius derives from the configured entry speed and available normal-acceleration reserve.
    // A short shallow segment avoids spending the unpowered flare energy before touchdown.
    let radius =
      configuration.guidance.finalSpeed * configuration.guidance.finalSpeed
      / configuration.guidance.preflareAccelerationReserve
    let span = max(
      250,
      radius * (configuration.guidance.outerGlideAngle - configuration.guidance.shallowGlideAngle))
    preflareAlong = shallowAlong - span
    let shallowHeight =
      configuration.vehicle.gearHeight + (configuration.guidance.touchdownAlong - shallowAlong)
      * tan(configuration.guidance.shallowGlideAngle)
    preflareHeight =
      shallowHeight + span
      * (tan(configuration.guidance.outerGlideAngle) + tan(configuration.guidance.shallowGlideAngle))
      / 2
  }
  public func reference(along x: Double) -> VerticalReference {
    let c = configuration.guidance
    let gear = configuration.vehicle.gearHeight
    let steep = -tan(c.outerGlideAngle)
    let shallow = -tan(c.shallowGlideAngle)
    if x < preflareAlong {
      return .init(height: preflareHeight + steep * (x - preflareAlong), slope: steep)
    }
    if x >= shallowAlong {
      return .init(
        height: gear + max(0, c.touchdownAlong - x) * (-shallow),
        slope: x < c.touchdownAlong ? shallow : 0)
    }
    let span = shallowAlong - preflareAlong
    let u = (x - preflareAlong) / span
    // Integrate smoothstep slope. C2 at both joins.
    let slope = steep + (shallow - steep) * (3 * u * u - 2 * u * u * u)
    let h =
      preflareHeight + span * (steep * u + (shallow - steep) * (u * u * u - 0.5 * u * u * u * u))
    return .init(
      height: h, slope: slope, secondDerivative: (shallow - steep) * (6 * u - 6 * u * u) / span)
  }
  public func speed(along: Double) -> Double {
    let c = configuration.guidance
    let td = configuration.vehicle.landingSpeed
    if along < preflareAlong { return c.finalSpeed }
    let u = clamp((along - preflareAlong) / (c.touchdownAlong - preflareAlong), 0, 1)
    return c.finalSpeed + (td - c.finalSpeed) * u
  }
}
