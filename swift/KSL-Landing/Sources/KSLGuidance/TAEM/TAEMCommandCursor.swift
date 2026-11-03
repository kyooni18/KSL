import KSLCore

/// Advances issued schedule intent from measured motion/time. Does not replace
/// the mission's limiter, FCS, trim or actuator history at acceptance/replans.
struct TAEMCommandCursor {
  let proposal: TAEMRecoveryProposal
  private(set) var index = 0
  private var elapsed = 0.0
  private var previousTime: Double
  private var progress: TAEMCourseProgress
  private var segmentStart = 0.0
  init(proposal: TAEMRecoveryProposal, state: VehicleState, configuration c: MissionConfiguration) {
    self.proposal = proposal; previousTime = state.time
    progress = .init(frame: .init(runway: c.runway, planet: c.planet), state: state,
      maximumSampleInterval: c.guidance.maximumSnapshotSpan)
  }
  mutating func update(_ state: VehicleState) -> GuidanceTarget? {
    guard let candidate = proposal.candidate, index < candidate.segments.count, state.time >= previousTime else { return nil }
    let roundoff = state.time.ulp+previousTime.ulp
    elapsed += state.time-previousTime; previousTime = state.time
    progress.observe(state)
    let segment = candidate.segments[index]
    if let goal = segment.untilCourseChange {
      guard progress.status == .available else { return nil }
      let achieved = progress.signedChange-segmentStart
      if (goal > 0 && achieved >= goal) || (goal < 0 && achieved <= goal) {
        index += 1; elapsed = 0; segmentStart = progress.signedChange
      } else if elapsed+roundoff >= segment.duration { return nil }
    } else if elapsed+roundoff >= segment.duration { index += 1; elapsed = 0; segmentStart = progress.signedChange }
    guard index < candidate.segments.count else { return nil }
    return candidate.segments[index].command.target
  }
  var finished: Bool { proposal.candidate.map { index >= $0.segments.count } ?? true }
  var tail: [TAEMCommandSegment]? {
    guard let candidate = proposal.candidate, index < candidate.segments.count else { return nil }
    return candidate.segments[index...].enumerated().map { offset, segment in
      let remaining = offset == 0 ? segment.duration-elapsed : segment.duration
      let work = offset == 0 ? segment.untilCourseChange.map { $0-(progress.signedChange-segmentStart) } : segment.untilCourseChange
      return .init(duration: remaining, command: segment.command, untilCourseChange: work)
    }
  }
}
