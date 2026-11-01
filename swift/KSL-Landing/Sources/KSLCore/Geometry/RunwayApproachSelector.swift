import Foundation

/// An observed-velocity/geometric decision for a vehicle ALREADY entering.
/// This is a necessary-course comparison, not an MM305/landing certificate.
/// For a deorbit still in orbit, evaluate the TWO complete full-route plans
/// instead; current orbital velocity is not a forecast of terminal geometry.
public struct RunwayApproachAssessment: Sendable {
  public let runway: RunwayGeometry
  public let turnToInlet: Double
  public let turnIntoFinal: Double
  public let pathDistance: Double
  public var minimumHeadingTravel: Double { turnToInlet + turnIntoFinal }
}

public enum RunwayApproachSelector {
  public static func candidates(configuration c: MissionConfiguration) -> [MissionConfiguration] {
    var inverse = c
    inverse.runway = c.runway.reciprocal(planet: c.planet)
    return [c, inverse]
  }

  public static func assess(_ state: VehicleState, configuration c: MissionConfiguration)
    -> RunwayApproachAssessment {
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let inletAlong = c.nepOverheadCaptureAlong
    let inletCross = c.entrySettings.handoffCross
    let pInlet = frame.position(along: inletAlong, cross: inletCross,
      height: c.entrySettings.terminalTargetAltitude - c.runway.elevation)
    let airHorizontal = (state.velocity - state.atmosphere.wind)
      .projected(onPlaneWithNormal: state.position.normalized)
    let track = airHorizontal.length > 1e-3
      ? state.localFrame.heading(of: airHorizontal)
      : state.localFrame.heading(of: frame.direction(at: frame.coordinates(of: state.position).horizontal,
        tangent: .init(1, 0)))
    // First leg is a great-circle direction to the actual MM305 inlet; the
    // second leg must connect to the local runway-aligned final tangent.
    // Computing both headings at their respective local tangent planes avoids
    // comparing chart vectors as if the small Kerbin surface were flat.
    let toInlet = frame.bearing(from: state.position, to: pInlet)
    let incoming = frame.bearing(from: pInlet, to: state.position)
    let incomingCourse = wrapAngle(incoming + .pi)
    let runwayTangent = frame.direction(at: .init(inletAlong, inletCross), tangent: .init(1, 0))
    let finalCourse = LocalFrame(position: pInlet).heading(of: runwayTangent)
    return .init(runway: c.runway,
      turnToInlet: abs(wrapAngle(toInlet - track)),
      turnIntoFinal: abs(wrapAngle(finalCourse - incomingCourse)),
      pathDistance: frame.distance(from: state.position, to: pInlet))
  }

  /// Minimum *necessary* heading change across the inlet and runway axis.
  /// This does not assert that lift, trim, energy or HAC geometry can realize
  /// either continuation. The choice is made ONCE before a flight computer
  /// is created and remains fixed throughout MM304/MM305.
  public static func choose(_ state: VehicleState, configuration c: MissionConfiguration)
    -> MissionConfiguration {
    let pair = candidates(configuration: c)
    let a = assess(state, configuration: pair[0])
    let b = assess(state, configuration: pair[1])
    if abs(a.minimumHeadingTravel - b.minimumHeadingTravel) > radians(0.1) {
      return a.minimumHeadingTravel < b.minimumHeadingTravel ? pair[0] : pair[1]
    }
    return a.pathDistance <= b.pathDistance ? pair[0] : pair[1]
  }
}
