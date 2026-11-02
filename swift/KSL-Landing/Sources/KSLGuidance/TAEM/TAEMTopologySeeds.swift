import Foundation
import KSLCore

/// Explicit experimental search knots, not validated flight defaults. AoA and
/// bank are commanded together; attained force/descent comes from propagation.
public struct TAEMTopologySeedParameters: Sendable {
  public let alpha: Double, bankMagnitude: Double, courseWork: Double, maximumSegmentDuration: Double
  public init(alpha: Double, bankMagnitude: Double, courseWork: Double, maximumSegmentDuration: Double) {
    self.alpha = alpha; self.bankMagnitude = bankMagnitude
    self.courseWork = courseWork; self.maximumSegmentDuration = maximumSegmentDuration
  }
}
public struct TAEMTopologySeedFamily: Sendable {
  public let candidates: [TAEMCommandCandidate]
  public let requestedParameterCount: Int
}

/// Interpretable command seeds. No position, tangent, attitude, radius or turn
/// duration is asserted attainable; the shared native rollout establishes it.
public enum TAEMTopologySeeds {
  public static func generate(configuration c: MissionConfiguration, baseTarget: GuidanceTarget,
                              parameters: [TAEMTopologySeedParameters],
                              incumbent: [TAEMCommandSegment], protection: [TAEMCommandSegment],
                              id: String) throws -> TAEMTopologySeedFamily {
    guard !id.isEmpty, parameters.allSatisfy({ p in
      [p.alpha, p.bankMagnitude, p.courseWork, p.maximumSegmentDuration].allSatisfy(\.isFinite)
        && p.alpha >= c.vehicle.minimumAngleOfAttack && p.alpha <= c.vehicle.maximumAngleOfAttack
        && p.bankMagnitude > 0 && p.bankMagnitude <= c.vehicle.maximumBank
        && p.courseWork > 0 && (p.courseWork+2 * .pi).isFinite && p.maximumSegmentDuration > 0
    }) else { throw FlightError.invalidConfiguration("Invalid explicit topology seed parameters") }
    // Always preserve the actually supplied recovery/protection commands. An
    // empty direct prefix offers immediate terminal continuation from the
    // original measured/controller seed, not instantaneous runway alignment.
    var candidates: [TAEMCommandCandidate] = [
      .init(id: "\(id)/incumbent", topology: .incumbent, segments: incumbent),
      .init(id: "\(id)/protection", topology: .protection, segments: protection),
      .init(id: "\(id)/direct", topology: .direct, segments: [])]
    for (index, p) in parameters.enumerated() {
      func segment(_ sign: Double, work: Double) -> TAEMCommandSegment {
        var target = baseTarget
        target.angleOfAttack = p.alpha; target.bank = sign*p.bankMagnitude
        // This is a turning/recovery command, not a claimed acquired spiral.
        return .init(duration: p.maximumSegmentDuration,
          command: .init(phase: .acquisition, target: target), untilCourseChange: sign*work)
      }
      let left = segment(-1, work: p.courseWork), right = segment(1, work: p.courseWork)
      let family: [(TAEMCommandTopology, [TAEMCommandSegment])] = [
        (.left, [left]), (.right, [right]),
        (.windingLeft, [segment(-1, work: p.courseWork+2 * .pi)]),
        (.windingRight, [segment(1, work: p.courseWork+2 * .pi)]),
        (.leftRight, [left, right]), (.rightLeft, [right, left])]
      for (topology, segments) in family {
        // Event segment times are ceilings, not minimum required durations.
        // Global coverage is enforced on actual propagation; summing ceilings
        // must not prune a reversal that can reach both events earlier.
        candidates.append(.init(id: "\(id)/\(index)/\(topology.rawValue)", topology: topology, segments: segments))
      }
    }
    return .init(candidates: candidates, requestedParameterCount: parameters.count)
  }
}
