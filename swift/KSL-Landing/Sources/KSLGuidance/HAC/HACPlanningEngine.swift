import Foundation
import KSLCore
import KSLControl
import KSLPhysics

public struct PlanningResult: Codable, Sendable {
  public let plan: TerminalPlan?
  public let bestAdvisory: TerminalAssessment?
  public let evaluatedCandidates: Int
  public let advisoryCounts: [String: Int]
  public var closedLoopAdvisories: [TerminalClosedLoopCase]?
  /// Physical energy/geometry envelope of the whole examined HAC family.
  public var envelope: TerminalEnvelopeSummary?
  public var fit: HACFitDiagnostics? = nil
}

// Earlier records lack the new advisory fields. Geometry remains replayable;
// missing historical diagnostics are simply empty.
extension PlanningResult {
  private enum CodingKeys: String, CodingKey {
    case plan, bestAdvisory, evaluatedCandidates, advisoryCounts, closedLoopAdvisories, envelope, fit
  }
  public init(from decoder: Decoder) throws {
    let c = try decoder.container(keyedBy: CodingKeys.self)
    plan = try c.decodeIfPresent(TerminalPlan.self, forKey: .plan)
    bestAdvisory = try c.decodeIfPresent(TerminalAssessment.self, forKey: .bestAdvisory)
    evaluatedCandidates = try c.decode(Int.self, forKey: .evaluatedCandidates)
    advisoryCounts = try c.decodeIfPresent([String: Int].self, forKey: .advisoryCounts) ?? [:]
    closedLoopAdvisories = try c.decodeIfPresent([TerminalClosedLoopCase].self, forKey: .closedLoopAdvisories)
    envelope = try c.decodeIfPresent(TerminalEnvelopeSummary.self, forKey: .envelope)
    fit = try c.decodeIfPresent(HACFitDiagnostics.self, forKey: .fit)
  }
}

/// What the priced HAC family can physically absorb from one state. A
/// "completed" candidate was priced to touchdown without a geometry, lift,
/// bank-rate or envelope violation, so its terminal speed is an energy result.
public struct TerminalEnvelopeSummary: Codable, Sendable {
  public var examined = 0
  public var completed = 0
  public var feasible = 0
  /// Slowest/fastest priced touchdown among completed candidates (m/s); a
  /// path that exhausts kinetic energy counts as zero touchdown speed.
  public var slowestTouchdown: Double?
  public var fastestTouchdown: Double?
  public var shortestFeasibleLength: Double?
  public var longestFeasibleLength: Double?
  public var shortestExaminedLength: Double?
  public var longestExaminedLength: Double?
  public var violationCounts: [String: Int] = [:]
  public init() {}
  mutating func record(length: Double, assessment: TerminalAssessment, completed done: Bool) {
    examined += 1
    shortestExaminedLength = min(shortestExaminedLength ?? length, length)
    longestExaminedLength = max(longestExaminedLength ?? length, length)
    for violation in assessment.violations { violationCounts[violation.kind.rawValue, default: 0] += 1 }
    guard done else { return }
    completed += 1
    slowestTouchdown = min(slowestTouchdown ?? assessment.touchdownSpeed, assessment.touchdownSpeed)
    fastestTouchdown = max(fastestTouchdown ?? assessment.touchdownSpeed, assessment.touchdownSpeed)
    if assessment.feasible {
      feasible += 1
      shortestFeasibleLength = min(shortestFeasibleLength ?? length, length)
      longestFeasibleLength = max(longestFeasibleLength ?? length, length)
    }
  }
}

public final class HACPlanningEngine {
  private let configuration: MissionConfiguration
  private let energy: EnergyManagementEngine
  private let approach: ApproachProfile
  private let frame: RunwayFrame
  private let trajectory: TerminalTrajectoryEvaluator
  private let aero: AerodynamicsEngine
  private var nextIdentifier = 1
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    aero = aerodynamics
    trajectory = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    energy = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    approach = .init(configuration: configuration)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func plan(from state: VehicleState, allowDirect: Bool = true, preview: Bool = false,
                   controlSnapshot: FlightControlSnapshot? = nil, previousTarget: GuidanceTarget? = nil,
                   context: HACFitContext? = nil)
    -> PlanningResult
  {
    let p = frame.coordinates(of: state.position)
    let c = configuration.guidance
    nextIdentifier = max(nextIdentifier, Int((state.time/c.controlInterval).rounded()))
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
    let speedSquared = max(100, state.horizontalSpeed * state.horizontalSpeed)
    // Physically bounded: no admissible turn exceeds the load envelope.
    let curvatureLimit = configuration.vehicle.maximumLoadFactor * 9.80665 / speedSquared
    let measuredCurvature = acceleration.dot(groundRight) / speedSquared
    let startCurvature = measuredCurvature.isFinite
      ? clamp(measuredCurvature, -curvatureLimit, curvatureLimit) : 0
    let start = HorizontalPathSample(
      position: p.horizontal, tangent: startTangent, curvature: startCurvature)
    var chosen: TerminalPlan?
    var candidates: [TerminalPlan] = []
    var closedLoopAdvisories: [TerminalClosedLoopCase] = []
    var advisory: TerminalAssessment?
    var count = 0
    var advisoryCounts: [String: Int] = [:]
    var envelope = TerminalEnvelopeSummary()
    func examine(_ horizontal: [(HorizontalPathSample, PathSegment)], _ spiral: SpiralHAC?) {
      guard horizontal.count > 2 else { return }
      var distances = [0.0]
      for i in 1..<horizontal.count {
        distances.append(
          distances[i - 1] + (horizontal[i].0.position - horizontal[i - 1].0.position).length)
      }
      let terminalLength = distances.last ?? 0
      guard terminalLength > 0 else { return }
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
      let cubic = distances.map { d -> (height: Double, slope: Double, verticalCurvature: Double) in
        let u = d / l
        let slope = (3 * a * u * u + 2 * b * u + cc) / l
        let second = (6 * a * u + 2 * b) / (l * l)
        return (((a * u + b) * u + cc) * u + h0, slope, second / pow(1 + slope * slope, 1.5))
      }
      // The cubic ignores energy: from a supersonic TAEM state it keeps the
      // vehicle high while drag bleeds speed until lift cannot support it.
      // The q-scheduled profile is the energy-consistent alternative.
      var profiles = [cubic]
      if let scheduled = energy.scheduledProfile(distances: distances, horizontal: horizontal.map(\.0),
        from: state, exitHeight: h1, exitSlope: m1) { profiles.append(scheduled) }
      for vertical in profiles {
      var stations: [PathStation] = []
      for i in horizontal.indices {
        let (s, segment) = horizontal[i]
        stations.append(
          .init(
            distance: distances[i], position: s.position, tangent: s.tangent, curvature: s.curvature,
            height: vertical[i].height, slope: vertical[i].slope,
            verticalCurvature: vertical[i].verticalCurvature, segment: segment))
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
      let energyOnly: Set<FailureClassification> = [.insufficientEnergy, .excessiveEnergy]
      envelope.record(length: stations.last?.distance ?? 0, assessment: assessment,
        completed: assessment.violations.allSatisfy { energyOnly.contains($0.kind) })
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
          advisoryCounts[violation.kind.rawValue, default: 0] += 1
        }
        if advisory == nil || assessment.score < advisory!.score { advisory = assessment }
        // A model verdict is an advisory, never a reason to remove a HAC.
        candidates.append(.init(identifier: nextIdentifier, createdAt: state.time,
          spiral: spiral, stations: priced + stations.dropFirst(priced.count), assessment: assessment, stressCases: []))
      }
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
    // Fit the spiral from the measured state on every planning epoch.
    // Radius comes from available normal force/bank, angle from heading,
    // remaining energy and the current winding; none is a fixed angle grid.
    let v = configuration.vehicle, aState = state.aerodynamics
    let gravity = configuration.planet.gravitationalParameter/state.position.squaredLength
    let support = max(0, (gravity-state.horizontalSpeed*state.horizontalSpeed/state.position.length)*cos(state.flightPathAngle))
    let trimAlpha = RotationalDynamicsEngine(configuration: v, planet: configuration.planet)
      .supportedTrimAlpha(state, identifiedCeiling: context?.trimAlphaCeiling)
    let maxLift = aero.liftSolution(acceleration: 0, mass: state.mass,
      dynamicPressure: aState.dynamicPressure, mach: aState.mach,
      minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: trimAlpha).maximumAcceleration
    let normalLimit = min(maxLift, v.maximumLoadFactor*gravity)
    let bankFromLift = normalLimit > support ? acos(clamp(support/normalLimit, 0, 1)) : 0
    let bankAuthority = min(v.maximumBank, EnergyManagementEngine.acquisitionBank(configuration), bankFromLift)
    let lateralAuthority = min(gravity*tan(bankAuthority), normalLimit*sin(bankAuthority))
    let turnRate = state.horizontalSpeed > 0 ? lateralAuthority/state.horizontalSpeed : 0
    let dynamicRadius = turnRate > 0 ? clamp(state.horizontalSpeed/turnRate, c.minimumHACRadius, c.maximumHACRadius) : c.maximumHACRadius
    let rollRate = min(v.controlPlant.maximumRollRate, c.terminalRollRate ?? c.bankSlewRate)
      * EnergyManagementEngine.bankRateReserve
    let bodyRoll = state.controlAuthority?.minimumAcceleration.x
      ?? min(v.controlPlant.rollAccelerationMaximum, v.controlPlant.rollAccelerationPerKPa*aState.dynamicPressure/1000)
    let bodyYaw = state.controlAuthority?.minimumAcceleration.z
      ?? min(v.controlPlant.yawAccelerationMaximum, v.controlPlant.yawAccelerationPerKPa*aState.dynamicPressure/1000)
    let rollAcceleration = min(bodyRoll/max(Double.leastNormalMagnitude, abs(cos(aState.angleOfAttack))),
      bodyYaw/max(Double.leastNormalMagnitude, abs(sin(aState.angleOfAttack))))
    let exchange = EntryLateralController.exchangeTime(span: abs(aState.bank)+bankAuthority,
      rate: rollRate, acceleration: rollAcceleration)
    let response = min(c.maximumPredictionDuration, max(context?.rollResponseSeconds ?? 0,
      exchange.isFinite ? exchange : c.maximumPredictionDuration))
    let joinTime = min(c.maximumPredictionDuration, max(c.pathLookaheadTime, response))
    let rollTransition = min(c.maximumPathLength, state.horizontalSpeed*joinTime)
    let spacing = max(c.finalSpeed, state.horizontalSpeed)*c.predictionStep
    let unloadLength = c.finalSpeed*c.pathLookaheadTime
    let glideAlpha = aero.bestGlideAlpha(mach: aState.mach, maximumAlpha: trimAlpha,
      dynamicPressure: aState.dynamicPressure)
    let polar = aero.coefficients(mach: aState.mach, alpha: glideAlpha, dynamicPressure: aState.dynamicPressure)
    let liftDrag = polar.drag > 0 ? max(0, polar.lift/polar.drag) : configuration.entrySettings.estimatedLiftToDrag
    let energyHeight = energy.energyHeight(state)
    let touchdownEnergy = v.landingSpeed*v.landingSpeed/(2*gravity)
    let desiredLength = max(0, energyHeight-touchdownEnergy)*liftDrag
    let directToExit = (p.horizontal-Vector2(-c.finalDistance-unloadLength, 0)).length
    let nominalExitRadius = c.finalSpeed*c.finalSpeed/max(Double.leastNormalMagnitude, gravity*tan(EnergyManagementEngine.acquisitionBank(configuration)))
    let old = context?.previousSpiral
    let continuing = context?.onSpiral == true && old != nil
    let remaining = old?.fittedRemainingAngle(at: p.horizontal)
    let u = v.aerodynamicUncertainty
    let minimumRadius = min(c.maximumHACRadius, max(c.minimumHACRadius, nominalExitRadius))
    let exitOffset = p.horizontal-Vector2(-c.finalDistance-unloadLength, 0)
    let tangentRadius = abs(exitOffset.y) > Double.leastNormalMagnitude
      ? exitOffset.dot(exitOffset)/(2*abs(exitOffset.y)) : dynamicRadius
    let radii = Array(Set([minimumRadius, tangentRadius, c.maximumHACRadius,
      nominalExitRadius, c.hacRadius ?? nominalExitRadius,
      dynamicRadius*(1-u), dynamicRadius, dynamicRadius*(1+u), old?.finalRadius ?? dynamicRadius]
      .map { clamp($0, minimumRadius, c.maximumHACRadius) })).sorted()
    let runwayDirection = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let runwayHeading = state.localFrame.heading(of: runwayDirection)
    let angleResolution = max(Double.ulpOfOne, turnRate*c.controlInterval)
    let sides = continuing ? [old!.side] : [-1, 1]
    for side in sides {
      for radius in radii {
        var headingAngle = Double(side)*wrapAngle(runwayHeading-state.course)
        if headingAngle < 0 { headingAngle += 2 * .pi }
        let energyAngle = max(0, desiredLength-directToExit-c.finalDistance)/max(radius, dynamicRadius)
        // Price an inlet beside the vehicle, rather than only an angle inferred
        // from runway heading; heading alone can put the inlet far ahead.
        let geometricAngle = atan2(-exitOffset.x, radius-Double(side)*exitOffset.y)
        let inletAngle = geometricAngle >= 0 ? geometricAngle : geometricAngle+2 * .pi
        var angles = [headingAngle, energyAngle, inletAngle]
        if let remaining {
          angles += [remaining, remaining*(1-u), remaining*(1+u)]
          if continuing { angles += [max(0, remaining-turnRate*joinTime)] }
        }
        for theta in Set(angles.map { clamp($0, angleResolution, SpiralHAC.maximumTurnAngle) }).sorted() {
          if Task.isCancelled { break }
          let growth = clamp((dynamicRadius-radius)/(theta*theta), radius*Double.ulpOfOne,
            radius/2*GuidanceTargetLimiter.responseReserve)
          guard let spiral = try? SpiralHAC(exitAlong: -c.finalDistance-unloadLength,
            finalRadius: radius, growth: growth, turnAngle: theta, side: side) else { continue }
          let inlet = spiral.sample(remainingAngle: theta)
          let distance = (inlet.position-start.position).length
          if distance <= 0 { continue }
          let curve = QuinticAcquisition(start: start, end: inlet,
            tangentLength: max(rollTransition, distance/GuidanceTargetLimiter.responseReserve))
          let n = max(2, Int(ceil(distance/spacing)))
          let spiralLength = spiral.length(samples: max(2, Int(ceil(radius*theta/spacing))))
          let ns = max(2, Int(ceil(spiralLength/spacing)))
          let joinSegment: PathSegment = continuing ? .spiral : .acquisition
          var tail: [(HorizontalPathSample, PathSegment)] = []
          for i in 1...ns {
            tail.append((spiral.sample(remainingAngle: theta*(1-Double(i)/Double(ns))), .spiral))
          }
          let unload = QuinticAcquisition(start: spiral.sample(remainingAngle: 0),
            end: .init(position: .init(-c.finalDistance, 0), tangent: .init(1, 0), curvature: 0),
            tangentLength: unloadLength)
          let unloadSamples = max(2, Int(ceil(unloadLength/spacing)))
          for i in 1...unloadSamples { tail.append((unload.sample(Double(i)/Double(unloadSamples)), .acquisition)) }
          examine((0...n).map { (curve.sample(Double($0)/Double(n)), joinSegment) } + tail, spiral)
          if dynamicRadius > radius, abs(inlet.curvature) > Double.ulpOfOne {
            for turn in [-1.0, 1.0] {
              if let dubins = DubinsAcquisition(start: start, startRadius: dynamicRadius, startTurn: turn,
                end: inlet, endRadius: 1/abs(inlet.curvature), endTurn: inlet.curvature > 0 ? 1 : -1,
                spacing: spacing, transition: rollTransition) {
                examine(dubins.samples.map { ($0, joinSegment) } + tail, spiral)
              }
            }
          }
        }
      }
    }
    // Energy-equivalent plans (touchdown speed within the aerodynamic
    // uncertainty of landing speed) are ordered by total roll travel from the
    // current bank, so MM305 prefers the predictable, monotonic turn.
    func rollTravel(_ plan: TerminalPlan) -> Double {
      var bank = state.aerodynamics.bank, total = 0.0
      for station in plan.stations { total += abs(wrapAngle(station.bank - bank)); bank = station.bank }
      return total
    }
    func fitCost(_ plan: TerminalPlan) -> Double {
      let a = plan.assessment
      let distanceScale = max(state.horizontalSpeed*c.pathLookaheadTime, desiredLength, c.finalDistance)
      let pathEnergy = abs(plan.length-desiredLength)/distanceScale
      let liftDeficit = max(0, -a.minimumLiftMargin)/max(gravity, Double.leastNormalMagnitude)
      let bankDeficit = max(0, a.maximumBank/v.maximumBank-1)
      let pressure = max(0, a.maximumQ/v.maximumDynamicPressure-1)
      let load = max(0, a.maximumLoad/v.maximumLoadFactor-1)
      let rollDistance = rollTravel(plan)*state.horizontalSpeed*response
      var acquisitionLength = 0.0, spiralLength = 0.0
      for i in plan.stations.indices.dropFirst() {
        let station = plan.stations[i]
        let distance = station.distance-plan.stations[i-1].distance
        if station.segment == .acquisition { acquisitionLength += distance }
        if station.segment == .spiral { spiralLength += distance }
      }
      // Joining is a response transient. The spiral should do the turning and
      // energy management, rather than a long acquisition consuming MM305.
      let acquisitionBudget = max(spacing, state.horizontalSpeed*response)
      let lateJoin = max(0, acquisitionLength-acquisitionBudget)/distanceScale
      let acquisitionDominance = max(0, acquisitionLength-spiralLength)/distanceScale
      let radiusExcess = plan.spiral.map {
        max(0, $0.finalRadius/max(minimumRadius, dynamicRadius)-1)
          * spiralLength/distanceScale
      } ?? 0
      return pathEnergy+liftDeficit+bankDeficit+pressure+load
        + rollDistance/distanceScale+lateJoin+acquisitionDominance+radiusExcess
    }
    candidates.sort {
      if $0.assessment.feasible != $1.assessment.feasible { return $0.assessment.feasible }
      return fitCost($0) < fitCost($1)
    }
    // Select before diagnostics. Neither pricing nor a closed-loop prediction
    // may veto this plan; the flight computer always flies the best available HAC.
    chosen = candidates.first
    if let candidate = chosen, !candidate.assessment.feasible {
      // Keep survey work bounded: complete the selected advisory reference
      // once, rather than simulating every poor route to the runway.
      let (priced, assessment) = energy.evaluate(stations: candidate.stations, from: state, advisoryOnly: true)
      chosen = .init(identifier: candidate.identifier, createdAt: candidate.createdAt,
        spiral: candidate.spiral, stations: priced, assessment: assessment, stressCases: candidate.stressCases)
    }
    if !preview, let candidate = chosen, candidate.assessment.feasible, !Task.isCancelled {
      let uncertainty = configuration.vehicle.aerodynamicUncertainty
      let cases = [(1.0, 1.0), (1-uncertainty, 1+uncertainty), (1+uncertainty, 1-uncertainty)].map { lift, drag in
        trajectory.evaluate(plan: candidate, from: state,
          controlSnapshot: controlSnapshot, previousTarget: previousTarget, lift: lift, drag: drag)
      }
      closedLoopAdvisories = cases.filter { !$0.feasible }
      chosen = .init(identifier: candidate.identifier, createdAt: candidate.createdAt,
        spiral: candidate.spiral, stations: candidate.stations, assessment: candidate.assessment,
        stressCases: candidate.stressCases, closedLoopCases: cases)
    }
    if chosen != nil { nextIdentifier += 1 }
    var result = PlanningResult(plan: chosen, bestAdvisory: advisory, evaluatedCandidates: count,
      advisoryCounts: advisoryCounts, closedLoopAdvisories: closedLoopAdvisories, envelope: envelope)
    result.fit = .init(stateTime: state.time, previousPlanID: context?.previousPlanID,
      continuingSpiral: continuing, fittedRemainingAngle: remaining, availableBank: bankAuthority,
      availableTurnRate: turnRate, turningRadius: dynamicRadius, rollResponseSeconds: response,
      joinDistance: rollTransition, energyHeight: energyHeight, energyPricedDistance: desiredLength,
      liftScale: context?.liftScale ?? 1, dragScale: context?.dragScale ?? 1)
    return result
  }
}
