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
  /// Model-derived target; absent means the Final suffix could not be solved.
  public var nominalEnergyPoint: NominalEnergyPoint? = nil
}

// Earlier records lack the new advisory fields. Geometry remains replayable;
// missing historical diagnostics are simply empty.
extension PlanningResult {
  private enum CodingKeys: String, CodingKey {
    case plan, bestAdvisory, evaluatedCandidates, advisoryCounts, closedLoopAdvisories, envelope, fit, nominalEnergyPoint
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
    nominalEnergyPoint = try c.decodeIfPresent(NominalEnergyPoint.self, forKey: .nominalEnergyPoint)
  }
}

/// What the priced HAC family can physically absorb from one state. A
/// "completed" candidate was priced to touchdown without a geometry, lift,
/// bank-rate or envelope violation, so its terminal speed is an energy result.
public struct TerminalEnvelopeSummary: Codable, Sendable {
  public var examined = 0
  public var completed = 0
  public var feasible = 0
  /// Slowest/fastest priced touchdown among completed candidates (m/s).
  /// Energy-exhausted prefixes have no touchdown prediction and are excluded.
  public var slowestTouchdown: Double?
  public var fastestTouchdown: Double?
  public var shortestFeasibleLength: Double?
  public var longestFeasibleLength: Double?
  public var shortestExaminedLength: Double?
  public var longestExaminedLength: Double?
  /// Geometric search coverage, not an aerodynamic reachability verdict.
  public var minimumRequiredLength: Double?
  public var pathLengthLimit: Double?
  public var violationCounts: [String: Int] = [:]
  public init() {}
  mutating func record(length: Double, assessment: TerminalAssessment, completed done: Bool) {
    examined += 1
    shortestExaminedLength = min(shortestExaminedLength ?? length, length)
    longestExaminedLength = max(longestExaminedLength ?? length, length)
    // Count affected candidates, not every station at which a constraint is
    // violated. Otherwise path discretization can outweigh the physical cause
    // that prevented every candidate from reaching the runway.
    for kind in Set(assessment.violations.map { $0.kind.rawValue }) {
      violationCounts[kind, default: 0] += 1
    }
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
  static func ranksAhead(feasible: Bool, cost: Double, ofFeasible: Bool, cost otherCost: Double) -> Bool {
    feasible == ofFeasible ? cost < otherCost : feasible
  }
  private let configuration: MissionConfiguration
  private let energy: EnergyManagementEngine
  private let approach: ApproachProfile
  private let frame: RunwayFrame
  private let trajectory: TerminalTrajectoryEvaluator
  private let aero: AerodynamicsEngine
  private let atmosphere: AtmosphericModelEngine
  private let nepSolver: NominalEnergyPointSolver
  private var nextIdentifier = 1
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    aero = aerodynamics
    self.atmosphere = atmosphere
    nepSolver = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    trajectory = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    energy = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    approach = .init(configuration: configuration)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func plan(from state: VehicleState, allowDirect: Bool = true, preview: Bool = false,
                   controlSnapshot: FlightControlSnapshot? = nil, previousTarget: GuidanceTarget? = nil,
                       targetLimiter: GuidanceTargetLimiter? = nil,
                   context: HACFitContext? = nil)
    -> PlanningResult
  {
    let p = frame.coordinates(of: state.position)
    let c = configuration.guidance
    var envelope = TerminalEnvelopeSummary()
    // Every sampled chart path to touchdown is at least this straight-line
    // distance. Early exploration outside that horizon needs a truthful,
    // constant-work response, not allocation of an impossible full HAC family.
    let minimumLength = (p.horizontal - Vector2(c.touchdownAlong, 0)).length
    envelope.minimumRequiredLength = minimumLength
    envelope.pathLengthLimit = c.maximumPathLength
    if minimumLength > c.maximumPathLength {
      return .init(plan: nil, bestAdvisory: nil, evaluatedCandidates: 0,
        advisoryCounts: [:], envelope: envelope)
    }
    let nep = nepSolver.solve(from: state, trimAlphaCeiling: context?.trimAlphaCeiling)
    nextIdentifier = max(nextIdentifier, Int((state.time/c.controlInterval).rounded()))
    let horizontal = state.velocity.projected(onPlaneWithNormal: state.position.normalized)
      .normalized
    let chartVelocity = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
    let startTangent = chartVelocity.normalized
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
    func examine(_ horizontal: [(HorizontalPathSample, PathSegment)], _ spiral: SpiralHAC?) {
      guard horizontal.count > 2 else { return }
      var distances = [0.0]
      for i in 1..<horizontal.count {
        distances.append(
          distances[i - 1] + (horizontal[i].0.position - horizontal[i - 1].0.position).length)
      }
      let terminalLength = distances.last ?? 0
      guard terminalLength.isFinite, terminalLength > 0,
        horizontal.allSatisfy({ $0.0.curvature.isFinite && $0.0.tangent.length > 0 }),
        terminalLength + c.finalDistance + c.touchdownAlong <= c.maximumPathLength else {
        advisoryCounts[FailureClassification.invalidGeometry.rawValue, default: 0] += 1
        return
      }
      let exit = approach.reference(along: -c.finalDistance)
      // C2 vertical connection: a position/slope-only cubic could begin with
      // a large unachievable push-over. Match the observed inlet normal force
      // and the final profile's curvature as well. Interior feasibility is
      // still priced; endpoint closure never certifies the trajectory.
      let h0 = p.height, h1 = exit.height
      let m0 = tan(state.flightPathAngle), m1 = exit.slope, l = terminalLength
      guard m0.isFinite else { return }
      let air = (state.velocity - state.atmosphere.wind).normalized
      let windRight = air.cross(state.position.normalized).normalized
      let windUp = windRight.cross(air).normalized
      let speed = state.aerodynamics.airspeed
      let gravity = configuration.planet.gravitationalParameter / state.position.squaredLength
      let gravityVector = state.position.normalized * (-gravity)
      let normalAcceleration = (acceleration + gravityVector).dot(windUp)
      let inletCurvature = speed > 0
        ? normalAcceleration / (speed * speed) + cos(state.flightPathAngle) / state.position.length : 0
      let a0 = h0, a1 = l * m0
      let a2 = inletCurvature * pow(1 + m0 * m0, 1.5) * l * l / 2
      let b0 = h1 - a0 - a1 - a2
      let b1 = l * m1 - a1 - 2 * a2
      let b2 = l * l * exit.secondDerivative - 2 * a2
      let a3 = 10 * b0 - 4 * b1 + b2 / 2
      let a4 = -15 * b0 + 7 * b1 - b2
      let a5 = 6 * b0 - 3 * b1 + b2 / 2
      let joined = distances.map { d -> (height: Double, slope: Double, verticalCurvature: Double) in
        let u = d / l
        let height = ((((a5 * u + a4) * u + a3) * u + a2) * u + a1) * u + a0
        let slope = (((5 * a5 * u + 4 * a4) * u + 3 * a3) * u + 2 * a2) * u + a1
        let second = ((20 * a5 * u + 12 * a4) * u + 6 * a3) * u + 2 * a2
        let derivative = slope / l
        return (height, derivative, second / (l * l * pow(1 + derivative * derivative, 1.5)))
      }
      // The q-scheduled profile remains a separate energy-aware candidate.
      var profiles = [joined]
      let exitQ = nep.map { n -> Double in
        let at = frame.position(along: n.along, cross: 0, height: n.height)
        return 0.5 * atmosphere.sample(position: at, time: state.time).density * n.airspeed * n.airspeed
      }
      if let scheduled = energy.scheduledProfile(distances: distances, horizontal: horizontal.map(\.0),
        from: state, exitHeight: h1, exitSlope: m1, exitDynamicPressure: exitQ) { profiles.append(scheduled) }
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
      // A preview is an existence/energy screen, never flight qualification.
      // Use fewer analytic-final samples here; any admitted candidate is
      // independently re-priced at 40 m spacing and flown closed-loop below.
      let steps = Int(ceil(finalSpan / (preview ? 120.0 : 40.0)))
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
      let (priced, assessment) = energy.evaluate(stations: stations, from: state,
        advisoryOnly: true, trimAlphaCeiling: context?.trimAlphaCeiling, nominalEnergyPoint: nep)
      let assessed: TerminalAssessment
      if nep == nil {
        var missing = TerminalAssessment(feasible: false,
          violations: assessment.violations + [.init(.modelUncertainty,
            "No physically priced NEP altitude/speed solution; candidate is recovery geometry only")],
          touchdownSpeed: assessment.touchdownSpeed,
          minimumLiftMargin: assessment.minimumLiftMargin, maximumBank: assessment.maximumBank,
          maximumQ: assessment.maximumQ, maximumLoad: assessment.maximumLoad,
          score: assessment.score + 1e6, uncertaintyFraction: assessment.uncertaintyFraction)
        missing.predictionComplete = assessment.predictionComplete
        missing.predictedThroughDistance = assessment.predictedThroughDistance
        missing.energyExhaustionDistance = assessment.energyExhaustionDistance
        missing.unpricedDistance = assessment.unpricedDistance
        assessed = missing
      } else { assessed = assessment }
      let energyOnly: Set<FailureClassification> = [.insufficientEnergy, .excessiveEnergy]
      envelope.record(length: stations.last?.distance ?? 0, assessment: assessed,
        completed: assessed.predictionComplete == true
          && assessed.violations.allSatisfy { energyOnly.contains($0.kind) })
      if assessed.feasible {
        candidates.append(.init(identifier: nextIdentifier, createdAt: state.time,
          spiral: spiral, stations: priced, assessment: assessed, nominalEnergyPoint: nep))
      } else {
        for kind in Set(assessed.violations.map { $0.kind.rawValue }) {
          advisoryCounts[kind, default: 0] += 1
        }
        if advisory == nil || assessed.score < advisory!.score { advisory = assessed }
        // Retain geometric recovery information after the physical prediction
        // stops. Unpriced stations keep speed=0; they cannot supply an energy
        // target, arrival time or a feasible control-reference certificate.
        let recoveryGeometry = priced + Array(stations.dropFirst(priced.count))
        candidates.append(.init(identifier: nextIdentifier, createdAt: state.time,
          spiral: spiral, stations: recoveryGeometry, assessment: assessed,
          nominalEnergyPoint: nep))
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
    let support = max(0, (gravity-aState.airspeed*aState.airspeed/state.position.length)*cos(state.flightPathAngle))
    let trimAlpha = RotationalDynamicsEngine(configuration: v, planet: configuration.planet)
      .supportedTrimAlpha(state, identifiedCeiling: context?.trimAlphaCeiling)
    let maxLift = aero.liftSolution(acceleration: 0, mass: state.mass,
      dynamicPressure: aState.dynamicPressure, mach: aState.mach,
      minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: trimAlpha).maximumAcceleration
    let normalLimit = min(maxLift, v.maximumLoadFactor * 9.80665)
    let bankFromLift = normalLimit > support ? acos(clamp(support/normalLimit, 0, 1)) : 0
    let bankAuthority = min(v.maximumBank, EnergyManagementEngine.acquisitionBank(configuration), bankFromLift)
    let force = NormalForceEnvelope(maximumLift: normalLimit, maximumBank: bankAuthority)
    let lateralAuthority = force.lateralAuthority(reserving: support)
    let turnRate = state.horizontalSpeed > 0 ? lateralAuthority / state.horizontalSpeed : 0
    let rawRadius = turnRate > 0 ? state.horizontalSpeed / turnRate : Double.infinity
    let requiredRadius: Double? = rawRadius.isFinite ? rawRadius : nil
    // maximumHACRadius bounds the searched final radius, not instantaneous
    // aircraft capability. Never shrink a measured-state turn to that ceiling.
    let dynamicRadius = requiredRadius.map { max(c.minimumHACRadius, $0) } ?? c.maximumHACRadius
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
    // Price the energy-consistent path length over the whole descent: L/D
    // at the current (often transonic) state badly undervalues the subsonic
    // remainder, which made the family prefer straight-ins that arrived high.
    let glide = EquilibriumGlideRange(aero: aero, atmosphere: atmosphere, frame: frame,
      along: p.along, cross: p.cross, time: state.time, mass: state.mass, gravity: gravity,
      alphaFloor: max(0, v.minimumAngleOfAttack), alphaCeiling: trimAlpha, gear: state.systems.gearDown)
    let desiredLength = max(max(0, energyHeight-touchdownEnergy)*liftDrag,
      glide.range(from: energyHeight, to: touchdownEnergy, derate: 1))
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
      let radiusStep = old.map { $0.finalRadius*u
        * min(1, c.terminalReplanInterval/max(c.terminalReplanInterval, response)) } ?? 0
      let fittedRadii = continuing ? Array(Set(radii.map {
        clamp($0, max(minimumRadius, old!.finalRadius-radiusStep),
          max(minimumRadius, min(c.maximumHACRadius, old!.finalRadius+radiusStep)))
      })) : radii
      for radius in fittedRadii.sorted() {
        var headingAngle = Double(side)*wrapAngle(runwayHeading-state.course)
        if headingAngle < 0 { headingAngle += 2 * .pi }
        let energyAngle = max(0, desiredLength-directToExit-c.finalDistance)/max(radius, dynamicRadius)
        // Price an inlet beside the vehicle, rather than only an angle inferred
        // from runway heading; heading alone can put the inlet far ahead.
        let geometricAngle = atan2(-exitOffset.x, radius-Double(side)*exitOffset.y)
        let inletAngle = geometricAngle >= 0 ? geometricAngle : geometricAngle+2 * .pi
        var angles = continuing ? [] : [headingAngle, energyAngle, inletAngle]
        if let remaining {
          if continuing {
            // Refit the remaining winding; never add another loop or replace
            // a nearly completed turn with a new heading/energy angle.
            angles += [remaining, max(0, remaining-turnRate*c.terminalReplanInterval)]
          } else { angles += [remaining, remaining*(1-u), remaining*(1+u)] }
        }
        let minimumAngle = continuing
          ? max(Double.ulpOfOne, min(angleResolution, remaining ?? angleResolution)) : angleResolution
        for theta in Set(angles.map { clamp($0, minimumAngle, SpiralHAC.maximumTurnAngle) }).sorted() {
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
          let n = max(2, Int(ceil(max(distance, rollTransition) / spacing)))
          let spiralLength = spiral.length(samples: max(2, Int(ceil(radius*theta/spacing))))
          let ns = max(2, Int(ceil(spiralLength/spacing)))
          let joinSegment: PathSegment = .acquisition
          var tail: [(HorizontalPathSample, PathSegment)] = []
          for i in 1...ns {
            tail.append((spiral.sample(remainingAngle: theta*(1-Double(i)/Double(ns))), .spiral))
          }
          let unload = QuinticAcquisition(start: spiral.sample(remainingAngle: 0),
            end: .init(position: .init(-c.finalDistance, 0), tangent: .init(1, 0), curvature: 0),
            tangentLength: unloadLength)
          let unloadSamples = max(2, Int(ceil(unloadLength/spacing)))
          for i in 1...unloadSamples { tail.append((unload.sample(Double(i)/Double(unloadSamples)), .acquisition)) }
          // Label the actual analytic inlet, not the whole rejoin, as spiral.
          let join = (0...n).map { (curve.sample(Double($0) / Double(n)), $0 == n ? PathSegment.spiral : joinSegment) }
          examine(join + tail, spiral)
          if dynamicRadius > radius, abs(inlet.curvature) > Double.ulpOfOne {
            for turn in [-1.0, 1.0] {
              if let dubins = DubinsAcquisition(start: start, startRadius: dynamicRadius, startTurn: turn,
                end: inlet, endRadius: 1/abs(inlet.curvature), endTurn: inlet.curvature > 0 ? 1 : -1,
                spacing: spacing, transition: rollTransition,
                maximumLength: c.maximumPathLength - spiralLength - unloadLength - c.finalDistance - c.touchdownAlong) {
                let join = dubins.samples.enumerated().map {
                  ($0.element, $0.offset == dubins.samples.count - 1 ? PathSegment.spiral : joinSegment)
                }
                examine(join + tail, spiral)
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
      for station in plan.stations where station.speed > 0 {
        total += abs(wrapAngle(station.bank - bank)); bank = station.bank
      }
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
        if plan.stations[i - 1].segment == .acquisition { acquisitionLength += distance }
        if plan.stations[i - 1].segment == .spiral { spiralLength += distance }
      }
      // Joining is a response transient. The spiral should do the turning and
      // energy management, rather than a long acquisition consuming MM305.
      let acquisitionBudget = max(spacing, state.horizontalSpeed*response)
      let lateJoin = max(0, acquisitionLength-acquisitionBudget)/acquisitionBudget
      let acquisitionDominance = max(0, acquisitionLength-spiralLength)/max(spacing, spiralLength)
      let radiusExcess = plan.spiral.map {
        max(0, $0.finalRadius/max(minimumRadius, dynamicRadius)-1)
          * spiralLength/distanceScale
      } ?? 0
      let unpriced = (a.unpricedDistance ?? 0) / max(1, plan.length)
      let energyError = abs(a.touchdownSpeed - v.landingSpeed) / v.landingSpeed
      // Compare the priced NEP kinetic energy to the same modeled Final
      // requirement for every candidate. An unpriced inlet is not assigned a
      // fictitious NEP speed and remains subject to the unpriced penalty.
      let nepEnergyError: Double
      if let nep, let inlet = plan.stations.last(where: { $0.segment != .final }),
         inlet.speed > 0, inlet.distance <= (a.predictedThroughDistance ?? plan.length) {
        nepEnergyError = abs(inlet.speed * inlet.speed - nep.airspeed * nep.airspeed)
          / max(nep.airspeed * nep.airspeed, Double.leastNormalMagnitude)
      } else { nepEnergyError = 0 }
      let firstBank = plan.stations.first?.bank ?? aState.bank
      let continuity = abs(wrapAngle(firstBank - (previousTarget?.bank ?? aState.bank)))
        / max(Double.leastNormalMagnitude, rollRate * c.terminalReplanInterval)
      return pathEnergy+liftDeficit+bankDeficit+pressure+load+unpriced+energyError+nepEnergyError+continuity
        + rollDistance/distanceScale+lateJoin+acquisitionDominance+radiusExcess
    }
    candidates.sort {
      // Prefer an assessed feasible route, but retain all advisory routes as
      // a fallback when the entire family is infeasible.
      if $0.assessment.feasible != $1.assessment.feasible { return $0.assessment.feasible }
      let leftComplete = $0.assessment.predictionComplete == true
      let rightComplete = $1.assessment.predictionComplete == true
      if leftComplete != rightComplete { return leftComplete }
      return Self.ranksAhead(feasible: $0.assessment.feasible, cost: fitCost($0),
        ofFeasible: $1.assessment.feasible, cost: fitCost($1))
    }
    // Always return useful advisory geometry when one exists. Selection is
    // separate from the mission owner's physical admission decision.
    chosen = candidates.first
    if !preview, let first = chosen, first.assessment.feasible, !Task.isCancelled {
      // A point-mass ranking is not a closed-loop ranking. If its first
      // geometrically feasible route fails tracking or touchdown, try the
      // remaining priced feasible routes rather than repeatedly returning the
      // same failed preview. Every selected candidate must independently
      // pass the same nominal rollout; cancellation keeps stale work bounded
      // by the planning owner, not by relaxing any physical admission gate.
      var selected = first
      var nominal: TerminalClosedLoopCase?
      for candidate in candidates where candidate.assessment.feasible && !Task.isCancelled {
        let result = trajectory.evaluate(plan: candidate, from: state,
          controlSnapshot: controlSnapshot, previousTarget: previousTarget, targetLimiter: targetLimiter, lift: 1, drag: 1)
        if nominal == nil || result.feasible { selected = candidate; nominal = result }
        if result.feasible { break }
      }
      if let nominal, !Task.isCancelled {
        let cases = [nominal]
        closedLoopAdvisories = cases.filter { !$0.feasible }
        chosen = .init(identifier: selected.identifier, createdAt: selected.createdAt,
          spiral: selected.spiral, stations: selected.stations, assessment: selected.assessment,
          stressCases: selected.stressCases, closedLoopCases: cases,
          nominalEnergyPoint: selected.nominalEnergyPoint)
      }
    }
    if chosen != nil { nextIdentifier += 1 }
    var result = PlanningResult(plan: chosen, bestAdvisory: advisory, evaluatedCandidates: count,
      advisoryCounts: advisoryCounts, closedLoopAdvisories: closedLoopAdvisories, envelope: envelope)
    result.nominalEnergyPoint = nep
    result.fit = .init(stateTime: state.time, previousPlanID: context?.previousPlanID,
      continuingSpiral: continuing, fittedRemainingAngle: remaining, availableBank: bankAuthority,
      availableTurnRate: turnRate, turningRadius: dynamicRadius, rollResponseSeconds: response,
      joinDistance: chosen?.stations.first(where: { $0.segment == .spiral })?.distance ?? chosen?.length ?? 0,
      energyHeight: energyHeight, energyPricedDistance: desiredLength,
      liftScale: context?.liftScale ?? 1, dragScale: context?.dragScale ?? 1)
    result.fit?.responseDistance = rollTransition
    result.fit?.requiredTurningRadius = requiredRadius
    result.fit?.turningRadiusExceedsSearch = requiredRadius.map { $0 > c.maximumHACRadius } ?? true
    return result
  }
}
