import KSLCore
import KSLPhysics

/// Commands, not attitude/position assignments. Segment duration is an explicit
/// search variable in seconds, or time coverage for an achieved-course event.
/// Topology interpretation belongs to the search.
public struct TAEMCommandSegment: Sendable {
  public let duration: Double
  public let command: GuidanceSolution
  /// Optional signed course work (radians, positive right). For this event
  /// segment duration is maximum search coverage, not a claimed turn time.
  public let untilCourseChange: Double?
  public init(duration: Double, command: GuidanceSolution, untilCourseChange: Double? = nil) {
    self.duration = duration; self.command = command; self.untilCourseChange = untilCourseChange
  }
}

public enum TAEMScheduleEnd: String, Codable, Sendable {
  case scheduleExhausted, durationCoverage, pathCoverage, cancelled
  case unsupportedDynamics, computationFailure, contact, courseNotReached, courseUnavailable
}

/// A prefix, never a qualified or event-complete recovery. A terminal evaluator
/// must continue `seed` with the same model/configuration and original history.
public struct TAEMSchedulePrefix: Sendable {
  public let end: TAEMScheduleEnd
  public let steps: [NativeCommandStep]
  public let seed: NativeCommandSeed
  public let comparisonContext: TerminalComparisonContext
  public let courseProgress: TAEMCourseProgress
  public let accounting: NativeTrajectoryAccounting
  public let completedSegments: Int
  public let distance: Double
  public let momentSupport: NativeMomentSupport
  public let diagnostic: String?
}

/// The command-shooting evaluation primitive. It uses the actual limiter/FCS/
/// finite-response plant, retaining achieved motion and every issued command.
/// This does not yet generate topologies, optimize parameters, or own control.
public final class TAEMCommandScheduleEvaluator {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aerodynamics: AerodynamicsEngine
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; self.aerodynamics = aerodynamics
  }

  public func evaluate(_ segments: [TAEMCommandSegment], from seed: NativeCommandSeed,
                       cancelled: () -> Bool = { false }) throws -> TAEMSchedulePrefix {
    let c = configuration
    // Validate the entire schedule before touching its branch. Invalid search
    // parameters must not become a partially evaluated alternative.
    guard c.guidance.controlInterval.isFinite, c.guidance.controlInterval > 0,
      c.guidance.controlInterval <= 1,
      c.guidance.maximumPredictionDuration.isFinite, c.guidance.maximumPredictionDuration >= 0,
      c.guidance.maximumPathLength.isFinite, c.guidance.maximumPathLength >= 0,
      segments.allSatisfy({ segment in
        let t = segment.command.target
        return segment.duration.isFinite && segment.duration > 0
          && (segment.untilCourseChange.map { $0.isFinite && $0 != 0 } ?? true)
          && [.acquisition, .headingAlignment].contains(segment.command.phase)
          && [t.angleOfAttack, t.bank, t.desiredSpeed, t.desiredHeight, t.desiredFlightPath,
              t.speedbrakeFraction ?? 0].allSatisfy(\.isFinite)
      }) else {
      throw FlightError.invalidConfiguration("Invalid TAEM command schedule or search coverage")
    }
    let rollout = NativeCommandRollout(seed: seed, configuration: c,
      atmosphere: atmosphere, aerodynamics: aerodynamics)
    let context = TerminalComparisonContext(seed: seed, configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    var courseProgress = TAEMCourseProgress(frame: .init(runway: c.runway, planet: c.planet), state: rollout.state, maximumSampleInterval: c.guidance.maximumSnapshotSpan)
    var accounting = NativeTrajectoryAccounting()
    var steps: [NativeCommandStep] = [], distance = 0.0
    var completedSegments = 0
    func result(_ end: TAEMScheduleEnd, diagnostic: String? = nil) -> TAEMSchedulePrefix {
      .init(end: end, steps: steps, seed: rollout.seed, comparisonContext: context, courseProgress: courseProgress, accounting: accounting, completedSegments: completedSegments, distance: distance,
        momentSupport: rollout.pitchMomentSupport, diagnostic: diagnostic)
    }
    guard rollout.modelConsistent else { return result(.unsupportedDynamics, diagnostic: "Continuation model/configuration changed") }
    var elapsed = 0.0
    for segment in segments {
      var segmentElapsed = 0.0
      let startCourse = courseProgress.signedChange
      func reachedCourse() -> Bool {
        guard let goal = segment.untilCourseChange else { return false }
        let achieved = courseProgress.signedChange-startCourse
        return goal > 0 ? achieved >= goal : achieved <= goal
      }
      while segmentElapsed < segment.duration && !reachedCourse() {
        if cancelled() { return result(.cancelled) }
        if rollout.state.contact.isGrounded || rollout.state.contact == .crashed { return result(.contact) }
        if elapsed >= c.guidance.maximumPredictionDuration { return result(.durationCoverage) }
        if distance >= c.guidance.maximumPathLength { return result(.pathCoverage) }
        if segment.untilCourseChange != nil && courseProgress.status != .available {
          return result(.courseUnavailable, diagnostic: courseProgress.status.rawValue)
        }
        if !rollout.pitchMomentSupported { return result(.unsupportedDynamics, diagnostic: rollout.pitchMomentSupport.rawValue) }
        let dt = min(c.guidance.controlInterval, segment.duration-segmentElapsed,
          c.guidance.maximumPredictionDuration-elapsed)
        do {
          let step = try rollout.advance(segment.command, duration: dt)
          steps.append(step)
          accounting.include(step, configuration: c)
          courseProgress.observe(step.after)
          distance += (step.after.position-step.before.position).length
          elapsed += dt; segmentElapsed += dt
        } catch {
          return result(.computationFailure, diagnostic: String(describing: error))
        }
        // Support can expire within the last integration step. Preserve that
        // priced extent but do not call it a supported exhausted schedule.
        if rollout.state.contact.isGrounded || rollout.state.contact == .crashed { return result(.contact) }
        if !rollout.pitchMomentSupported { return result(.unsupportedDynamics, diagnostic: rollout.pitchMomentSupport.rawValue) }
        if segment.untilCourseChange != nil && courseProgress.status != .available {
          return result(.courseUnavailable, diagnostic: courseProgress.status.rawValue)
        }
      }
      if segment.untilCourseChange != nil && !reachedCourse() { return result(.courseNotReached) }
      completedSegments += 1
    }
    if rollout.state.contact.isGrounded || rollout.state.contact == .crashed { return result(.contact) }
    return result(.scheduleExhausted)
  }
}
