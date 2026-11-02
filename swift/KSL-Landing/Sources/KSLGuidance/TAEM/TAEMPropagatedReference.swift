import Foundation
import KSLCore
import KSLPhysics

/// A spatial view of an achieved native state, not a newly requested attitude.
public struct TAEMPropagatedStation: Codable, Sendable {
  public let time: Double
  /// Sum of runway-chart displacement chords; not continuous spatial arc length.
  public let chartDistance: Double
  public let coordinates: RunwayCoordinates?
  public let tangent: Vector2?
  /// Mean over the incoming interval: sampled course change / chart chord.
  /// Nil is unknown/undefined, never an invented straight segment.
  public let incomingCurvature: Double?
  public let incomingSlope: Double?
  public let courseStatus: TAEMCourseProgressStatus
}

/// Dynamics-generated advisory reference. States, requested/limited attitudes
/// and issued actuator commands remain separate. In particular, replaying the
/// achieved state attitudes as fresh targets would not reproduce this motion.
public struct TAEMPropagatedReference: Codable, Sendable {
  public let context: TerminalComparisonContext
  public let end: TAEMScheduleEnd
  public let completedSegments: Int
  public let momentSupport: NativeMomentSupport
  public let diagnostic: String?
  public let chartCoverageComplete: Bool
  public let states: [VehicleState]
  public let stations: [TAEMPropagatedStation]
  /// Commands[i] acts across states[i] -> states[i+1].
  public let commands: [GuidanceSolution]
  public let limitedTargets: [GuidanceTarget]
  public let issuedControls: [ControlCommand]
  public let joinState: VehicleState

  public init(prefix: TAEMSchedulePrefix) {
    context = prefix.comparisonContext; end = prefix.end; completedSegments = prefix.completedSegments
    momentSupport = prefix.momentSupport; diagnostic = prefix.diagnostic
    joinState = prefix.seed.state
    let c = context.configuration
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let first = prefix.steps.first?.before ?? prefix.seed.state
    let states = [first]+prefix.steps.map(\.after)
    self.states = states
    commands = prefix.steps.map(\.requested)
    limitedTargets = prefix.steps.map(\.limited)
    issuedControls = prefix.steps.map(\.issued)
    var progress = TAEMCourseProgress(frame: frame, state: first,
      maximumSampleInterval: c.guidance.maximumSnapshotSpan)
    var stations: [TAEMPropagatedStation] = [], distance = 0.0
    var coverageComplete = true
    var previous: RunwayCoordinates?
    for state in states {
      let beforeStatus = progress.status, beforeChange = progress.signedChange
      progress.observe(state)
      let velocity = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
      let p = frame.coordinates(of: state.position)
      let chartValid = velocity.x.isFinite && velocity.y.isFinite
        && p.along.isFinite && p.cross.isFinite && p.height.isFinite
      let coordinates: RunwayCoordinates? = chartValid ? p : nil
      let chord = coordinates.flatMap { next in previous.map { (next.horizontal-$0.horizontal).length } }
      if coordinates == nil { coverageComplete = false }
      if let chord {
        if chord.isFinite && (distance+chord).isFinite { distance += chord }
        else { coverageComplete = false }
      }
      let curvature: Double?
      if let chord, chord > 0, chord.isFinite, beforeStatus == .available, progress.status == .available {
        let value = (progress.signedChange-beforeChange)/chord
        curvature = value.isFinite ? value : nil
      } else { curvature = nil }
      let slope: Double?
      if let chord, chord > 0, chord.isFinite, let previous, let coordinates {
        let value = (coordinates.height-previous.height)/chord
        slope = value.isFinite ? value : nil
      } else { slope = nil }
      stations.append(.init(time: state.time, chartDistance: distance, coordinates: coordinates,
        tangent: progress.status == .available ? velocity.normalized : nil,
        incomingCurvature: curvature, incomingSlope: slope, courseStatus: progress.status))
      previous = coordinates
    }
    self.stations = stations
    chartCoverageComplete = coverageComplete
  }
}

extension TAEMCommandTrial {
  /// Does not extend the priced extent or promote model support/qualification.
  public func propagatedReference() -> TAEMPropagatedReference { .init(prefix: prefix) }
}
