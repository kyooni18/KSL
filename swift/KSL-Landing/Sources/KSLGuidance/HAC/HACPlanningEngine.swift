import Foundation
import KSLCore
import KSLControl
import KSLPhysics

public struct PlanningResult: Codable, Sendable {
  public let plan: TerminalPlan?
  public let bestRejected: TerminalAssessment?
  public let evaluatedCandidates: Int
  public let rejectedConstraints: [String: Int]
  public var closedLoopRejections: [TerminalClosedLoopCase]?
}

public final class HACPlanningEngine {
  private let configuration: MissionConfiguration
  private let energy: EnergyManagementEngine
  private let approach: ApproachProfile
  private let frame: RunwayFrame
  private let trajectory: TerminalTrajectoryEvaluator
  private var nextIdentifier = 1
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    trajectory = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    energy = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    approach = .init(configuration: configuration)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func plan(from state: VehicleState, allowDirect: Bool = true, preview: Bool = false,
                   controlSnapshot: FlightControlSnapshot? = nil, previousTarget: GuidanceTarget? = nil)
    -> PlanningResult
  {
    let p = frame.coordinates(of: state.position)
    let c = configuration.guidance
    let horizontal = state.velocity.projected(onPlaneWithNormal: state.position.normalized)
      .normalized
    let startTangent = Vector2(horizontal.dot(frame.alongAxis), horizontal.dot(frame.rightAxis))
      .normalized
    // A descending entry is not a coordinated level turn: g*tan(bank)
    // can demand several times the available lateral acceleration. Match
    // the observed ground-track curvature, including rotating-frame terms.
    let omega = configuration.planet.angularVelocity
    let acceleration =
      state.aerodynamics.force / state.mass
      - 2 * omega.cross(state.velocity) - omega.cross(omega.cross(state.position))
    let groundRight = horizontal.cross(state.position.normalized).normalized
    let startCurvature =
      acceleration.dot(groundRight)
      / max(100, state.horizontalSpeed * state.horizontalSpeed)
    let start = HorizontalPathSample(
      position: p.horizontal, tangent: startTangent, curvature: startCurvature)
    var chosen: TerminalPlan?
    var candidates: [TerminalPlan] = []
    var closedLoopRejections: [TerminalClosedLoopCase] = []
    var rejected: TerminalAssessment?
    var count = 0
    var rejectedConstraints: [String: Int] = [:]
    func examine(_ horizontal: [(HorizontalPathSample, PathSegment)], _ spiral: SpiralHAC?) {
      guard horizontal.count > 2 else { return }
      var distances = [0.0]
      for i in 1..<horizontal.count {
        distances.append(
          distances[i - 1] + (horizontal[i].0.position - horizontal[i - 1].0.position).length)
      }
      let terminalLength = distances.last ?? 0
      guard terminalLength > 100, terminalLength < c.maximumPathLength else { return }
      let exit = approach.reference(along: -c.finalDistance)
      // Cubic altitude spline matching measured entry FPA and final slope.
      let h0 = p.height
      let h1 = exit.height
      let m0 = clamp(tan(state.flightPathAngle), -0.8, 0.05)
      let m1 = exit.slope
      let l = terminalLength
      let a = 2 * h0 - 2 * h1 + l * (m0 + m1)
      let b = -3 * h0 + 3 * h1 - l * (2 * m0 + m1)
      let cc = l * m0
      var stations: [PathStation] = []
      for i in horizontal.indices {
        let (s, segment) = horizontal[i]
        let d = distances[i]
        let u = d / l
        let h = ((a * u + b) * u + cc) * u + h0
        let slope = (3 * a * u * u + 2 * b * u + cc) / l
        let second = (6 * a * u + 2 * b) / (l * l)
        let verticalCurvature = second / pow(1 + slope * slope, 1.5)
        stations.append(
          .init(
            distance: d, position: s.position, tangent: s.tangent, curvature: s.curvature,
            height: h, slope: slope, verticalCurvature: verticalCurvature, segment: segment))
      }
      let finalSpan = c.finalDistance + c.touchdownAlong
      let steps = Int(ceil(finalSpan / 40))
      for i in 1...steps {
        let x = -c.finalDistance + finalSpan * Double(i) / Double(steps)
        let v = approach.reference(along: x)
        stations.append(
          .init(
            distance: terminalLength + x + c.finalDistance, position: .init(x, 0),
            tangent: .init(1, 0), curvature: 0,
            height: v.height, slope: v.slope,
            verticalCurvature: v.secondDerivative / pow(1 + v.slope * v.slope, 1.5), segment: .final
          ))
      }
      count += 1
      let (priced, assessment) = energy.evaluate(stations: stations, from: state)
      if assessment.feasible {
        let u = configuration.vehicle.aerodynamicUncertainty
        let stresses = [(1-u, 1+u), (1+u, 1-u)].map { lift, drag in
          let (_, stressed) = energy.evaluate(stations: priced, from: state,
            dragMultiplier: drag, liftMultiplier: lift)
          return TerminalStressCase(liftMultiplier: lift, dragMultiplier: drag,
            feasible: stressed.feasible, touchdownSpeed: stressed.touchdownSpeed, violations: stressed.violations)
        }
        // Retain extreme stress failures as evidence; they must not erase
        // a physically feasible nominal candidate before closed-loop testing.
        candidates.append(.init(identifier: nextIdentifier, createdAt: state.time,
          spiral: spiral, stations: priced, assessment: assessment, stressCases: stresses))
      } else {
        for violation in assessment.violations {
          rejectedConstraints[violation.kind.rawValue, default: 0] += 1
        }
        if rejected == nil || assessment.score < rejected!.score { rejected = assessment }
      }
    }
    if allowDirect && p.along < -c.finalDistance - 1000 {
      let end = HorizontalPathSample(
        position: .init(-c.finalDistance, 0), tangent: .init(1, 0), curvature: 0)
      let distance = (end.position - start.position).length
      if distance > 1000 {
        let curve = QuinticAcquisition(start: start, end: end, tangentLength: distance)
        let n = max(20, Int(distance / 180))
        examine((0...n).map { (curve.sample(Double($0) / Double(n)), .acquisition) }, nil)
      }
    }
    let radii =
      [c.minimumHACRadius, 4500, 6500, 9000, c.hacRadius ?? 12_000, 17_000, 24_000, 32_000, c.maximumHACRadius]
      .filter { $0 >= c.minimumHACRadius && $0 <= c.maximumHACRadius }
    for side in [-1, 1] {
      for radius in Set(radii).sorted() {
        for angle in [60.0, 100, 150, 210, 270, 330, 390, 450] {
          if Task.isCancelled { break }
          let theta = radians(angle)
          let dynamicRadius =
            state.horizontalSpeed * state.horizontalSpeed / (9.80665 * tan(radians(50)))
          let growth = clamp(
            (dynamicRadius - radius) / (theta * theta), radius * 0.03, radius * 0.30)
          let unloadLength = max(1000, c.finalSpeed * 6)
          guard
            let spiral = try? SpiralHAC(
              exitAlong: -c.finalDistance - unloadLength, finalRadius: radius, growth: growth,
              turnAngle: theta, side: side)
          else { continue }
          let inlet = spiral.sample(remainingAngle: theta)
          let distance = (inlet.position - start.position).length
          let spiralLength = spiral.length(samples: 64)
          if distance + spiralLength > c.maximumPathLength || distance < 100 { continue }
          let curve = QuinticAcquisition(
            start: start, end: inlet, tangentLength: max(500, distance * 1.15))
          let n = max(20, Int(distance / 220))
          let ns = max(25, Int(spiralLength / 220))
          var horizontal = (0...n).map {
            (curve.sample(Double($0) / Double(n)), PathSegment.acquisition)
          }
          for i in 1...ns {
            horizontal.append(
              (spiral.sample(remainingAngle: theta * (1 - Double(i) / Double(ns))), .spiral))
          }
          let unload = QuinticAcquisition(
            start: spiral.sample(remainingAngle: 0),
            end: .init(position: .init(-c.finalDistance, 0), tangent: .init(1, 0), curvature: 0),
            tangentLength: unloadLength)
          for i in 1...25 { horizontal.append((unload.sample(Double(i) / 25), .acquisition)) }
          examine(horizontal, spiral)
        }
      }
    }
    candidates.sort { $0.assessment.score < $1.assessment.score }
    if preview {
      chosen = candidates.first // Geometry/energy preview carries no closed-loop certificate.
    } else {
      let budget = configuration.guidance.terminalCandidateBudget ?? 8
      let uncertainty = configuration.vehicle.aerodynamicUncertainty
      for candidate in candidates.prefix(budget) {
        if Task.isCancelled { break }
        var cases: [TerminalClosedLoopCase] = []
        for (lift, drag) in [(1.0, 1.0), (1-uncertainty, 1+uncertainty), (1+uncertainty, 1-uncertainty)] {
          let result = trajectory.evaluate(plan: candidate, from: state,
            controlSnapshot: controlSnapshot, previousTarget: previousTarget, lift: lift, drag: drag)
          cases.append(result)
          if cases.count == 1 && !result.feasible { break }
        }
        if cases.first?.feasible == true {
          chosen = .init(identifier: candidate.identifier, createdAt: candidate.createdAt,
            spiral: candidate.spiral, stations: candidate.stations, assessment: candidate.assessment,
            stressCases: candidate.stressCases, closedLoopCases: cases)
          break
        }
        closedLoopRejections += cases.filter { !$0.feasible }
        for result in cases { for violation in result.violations {
          rejectedConstraints[violation.kind.rawValue, default: 0] += 1
        } }
      }
      if chosen == nil && candidates.count > budget {
        rejectedConstraints[FailureClassification.timeout.rawValue, default: 0] += 1
      }
    }
    if chosen != nil { nextIdentifier += 1 }
    return .init(plan: chosen, bestRejected: rejected, evaluatedCandidates: count,
      rejectedConstraints: rejectedConstraints, closedLoopRejections: closedLoopRejections)
  }
}
