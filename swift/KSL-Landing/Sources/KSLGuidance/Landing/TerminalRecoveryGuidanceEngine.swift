import Foundation
import KSLCore
import KSLPhysics

/// What MM305's no-plan recovery is doing. None of these is a HAC plan or a
/// landing certificate; they rank physically priced continuations.
public enum TerminalRecoveryMode: String, Codable, Sendable {
  /// Energy covers a turn-priced path to the solved NEP; fly toward Final
  /// alignment while the planner keeps searching for a complete route.
  case alignment
  /// The NEP is no longer affordable but usable pavement is: fly the
  /// minimum-loss turn-then-glide continuation to the runway.
  case directRunway
  /// Even the minimum-loss continuation exceeds the modeled glide range.
  /// Report it and keep flying best glide toward the runway.
  case runwayUnreachable
}

public struct TerminalRecoveryDecision {
  public var target: GuidanceTarget
  public var mode: TerminalRecoveryMode
  public var lateralDemand: Double
  public var lateralAvailable: Double
  public var selectedDrag: Double
  /// Energy height above the runway (m), from measured altitude and airspeed.
  public var energyHeight: Double
  /// Conservative (uncertainty-derated) still-air glide distance (m).
  public var availableRange: Double
  /// Turn-priced path length of the chosen continuation (m).
  public var requiredRange: Double
  /// Runway-along coordinate of the chosen aim point (m).
  public var aimAlong: Double
  public var turnRadius: Double
  /// Nominal-model glide distance (m); the runway is reported unreachable
  /// only when even this is short.
  public var nominalRange: Double = 0
}

/// MM305's unqualified no-plan fallback. It is NOT a HAC plan or a landing
/// certificate. Every tick it reprices, from the measured state, the
/// continuation to the solved NEP and the minimum-loss continuation to usable
/// pavement, both with finite turn radius and the attainable polar, and flies
/// the best affordable one. A fixed alignment waypoint is never pursued once
/// the measured energy shows it is counterproductive.
final class TerminalRecoveryGuidanceEngine {
  private let configuration: MissionConfiguration
  private let frame: RunwayFrame
  private let aero: AerodynamicsEngine
  private let atmosphere: AtmosphericModelEngine
  private let rotation: RotationalDynamicsEngine
  private let profile: ApproachProfile
  private let nepSolver: NominalEnergyPointSolver
  private let speedbrake: TAEMSpeedbrakeController
  private var lastAlpha: Double?
  private var lastNEPTime = -Double.infinity
  private var lastNEP: NominalEnergyPoint?
  private var lastMode: TerminalRecoveryMode?
  var currentNominalEnergyPoint: NominalEnergyPoint? { lastNEP }

  func invalidatePrediction() {
    lastAlpha = nil; lastNEP = nil; lastNEPTime = -.infinity; lastMode = nil
  }

  init(configuration c: MissionConfiguration, atmosphere: AtmosphericModelEngine,
       aerodynamics: AerodynamicsEngine) {
    configuration = c
    frame = RunwayFrame(runway: c.runway, planet: c.planet)
    aero = aerodynamics
    self.atmosphere = atmosphere
    rotation = RotationalDynamicsEngine(configuration: c.vehicle, planet: c.planet)
    profile = ApproachProfile(configuration: c)
    nepSolver = .init(configuration: c, atmosphere: atmosphere, aerodynamics: aerodynamics)
    speedbrake = .init(configuration: c, atmosphere: atmosphere)
  }

  /// Length of a minimum-radius turn followed by a straight leg to a point at
  /// `distance` and relative bearing `bearing` (Dubins CS), and the heading
  /// change flown in the turn (signed like `bearing`). A target inside the
  /// turn circle needs an opening manoeuvre, priced as a half circle extra.
  static func turnThenStraight(distance d: Double, bearing: Double, radius r: Double)
    -> (length: Double, turn: Double) {
    let beta = abs(wrapAngle(bearing)), sign: Double = bearing < 0 ? -1 : 1
    guard r > 0, r.isFinite else { return (d, 0) }
    // Left-turn frame: start at origin heading +x, centre at (0, r).
    let tx = d * cos(beta), ty = d * sin(beta)
    let q = hypot(tx, ty - r)
    guard q > r else { return (d + .pi * r, sign * .pi) }
    let theta = atan2(ty - r, tx)
    let tangentAngle = theta - acos(r / q)
    var arc = (tangentAngle + .pi / 2).truncatingRemainder(dividingBy: 2 * .pi)
    if arc < 0 { arc += 2 * .pi }
    return (r * arc + sqrt(q * q - r * r), sign * arc)
  }

  func update(_ state: VehicleState, attainableAlphaCeiling: Double?,
              pitchResponseSeconds: Double?) -> TerminalRecoveryDecision {
    let c = configuration, a = state.aerodynamics, u = c.vehicle.aerodynamicUncertainty
    let p = frame.coordinates(of: state.position)
    let g = c.planet.gravitationalParameter / state.position.squaredLength
    let vh = max(1, state.horizontalSpeed), speed = max(1, a.airspeed)

    // ---- Attainable polar (measured force gains, evidenced AoA ceiling) ----
    let ceiling = rotation.supportedTrimAlpha(state, identifiedCeiling: attainableAlphaCeiling)
    let floor = min(ceiling, max(0, c.vehicle.minimumAngleOfAttack))
    let factor = max(0, a.dynamicPressure) * aero.referenceArea / max(1, state.mass)
    let measured = aero.coefficients(mach: a.mach, alpha: a.angleOfAttack,
      dynamicPressure: a.dynamicPressure, beta: a.sideslip, gear: state.systems.gearDown)
    let liftGain = a.measuredForces && factor * measured.lift > 0 && a.lift > 0
      ? a.lift / state.mass / (factor * measured.lift) : 1
    let dragGain = a.measuredForces && factor * measured.drag > 0 && a.drag > 0
      ? a.drag / state.mass / (factor * measured.drag) : 1
    let points = (0...64).map { i -> EntryForcePoint in
      let alpha = floor + (ceiling - floor) * Double(i) / 64
      let polar = aero.coefficients(mach: a.mach, alpha: alpha,
        dynamicPressure: a.dynamicPressure, beta: a.sideslip, gear: state.systems.gearDown)
      return EntryForcePoint(alpha: alpha, lift: max(0, factor * polar.lift * liftGain * (1 - u)),
        drag: max(0, factor * polar.drag * dragGain))
    }
    func ratio(_ e: EntryForcePoint) -> Double { e.lift / max(1e-9, e.drag * (1 + u)) }
    let glidePoint = points.max { ratio($0) < ratio($1) }!
    let glideRatio = ratio(glidePoint)
    let inertial = c.planet.inertialVelocity(position: state.position, fixedVelocity: state.velocity)
    let tangentSpeed2 = inertial.projected(onPlaneWithNormal: state.position.normalized).squaredLength
    let support = max(0, (g - tangentSpeed2 / state.position.length) * cos(state.flightPathAngle))
    let loadLimit = c.vehicle.maximumLoadFactor * 9.80665
    let maxLift = min(points.map(\.lift).max() ?? 0, loadLimit)
    // Finite turn radius from the strongest attainable lateral force that
    // still carries the current normal support; if no AoA can carry it, the
    // turn is flown at the bank limit and paid for in sink (priced below).
    var lateralForTurn = NormalForceEnvelope(maximumLift: maxLift, maximumBank: c.vehicle.maximumBank)
      .lateralAuthority(reserving: support)
    if lateralForTurn <= 0 { lateralForTurn = maxLift * sin(c.vehicle.maximumBank) }
    let radius = vh * vh / max(1e-6, lateralForTurn)
    let turnCos = maxLift > 0 ? clamp(support / maxLift, cos(c.vehicle.maximumBank), 1) : 1

    // ---- Energy and glide range ----
    let energyHeight = p.height + speed * speed / (2 * g)
    let touchdownEnergyHeight = c.vehicle.landingSpeed * c.vehicle.landingSpeed / (2 * g)
    /// Still-air range from energy height `top` down to `bottom` on the
    /// best attainable L/D. Best L/D and its CL change strongly with Mach, so
    /// each energy level is placed on the EQUILIBRIUM GLIDE: the altitude
    /// where q*S*CL(best L/D) carries the weight with h + V^2/2g = e. The
    /// 1 Nov 2211 flight showed that keeping the current Mach-1.5 split for
    /// the whole descent underestimated the range by a factor of 2-3 (22.7
    /// km predicted, >47 km flown with energy to spare). Measured force gains
    /// and the evidenced AoA ceiling are carried along. `derate` applies the
    /// configured uncertainty: (1-u)/(1+u) for a conservative claim, 1 for
    /// the nominal model.
    func glideRange(from top: Double, to bottom: Double, derate: Double) -> Double {
      guard top > bottom else { return 0 }
      let samples = 12, alphaStep = max(1e-3, (ceiling - floor) / 32)
      func best(height: Double, speed v: Double) -> (ratio: Double, lift: Double) {
        let position = frame.position(along: p.along, cross: p.cross, height: height)
        let air = atmosphere.sample(position: position, time: state.time)
        let q = air.density * v * v / 2, mach = v / max(1, air.speedOfSound)
        var result = (ratio: 0.0, lift: 0.0)
        for alpha in stride(from: floor, through: ceiling, by: alphaStep) {
          let polar = aero.coefficients(mach: mach, alpha: alpha, dynamicPressure: q,
            beta: 0, gear: state.systems.gearDown)
          let ratio = polar.lift * liftGain / max(1e-9, polar.drag * dragGain)
          if ratio > result.ratio { result = (ratio, q * aero.referenceArea * polar.lift * liftGain / max(1, state.mass)) }
        }
        return result
      }
      var range = 0.0
      for k in 0..<samples {
        let e = top - (top - bottom) * (Double(k) + 0.5) / Double(samples)
        // Lift surplus falls monotonically with altitude at fixed energy.
        var low = 0.0, high = max(0, e)
        for _ in 0..<24 {
          let mid = (low + high) / 2
          if best(height: mid, speed: sqrt(max(0, 2 * g * (e - mid)))).lift >= g { low = mid } else { high = mid }
        }
        let point = best(height: low, speed: sqrt(max(0, 2 * g * (e - low))))
        range += point.ratio * derate * (top - bottom) / Double(samples)
      }
      return range
    }
    let conservative = (1 - u) / (1 + u)
    let availableRange = glideRange(from: energyHeight, to: touchdownEnergyHeight, derate: conservative)
    let nominalRange = glideRange(from: energyHeight, to: touchdownEnergyHeight, derate: 1)
    let course = state.course
    let runwayHeading = state.localFrame.heading(of: frame.direction(at: p.horizontal, tangent: .init(1, 0)))
    /// Turn-priced still-air path to a centerline point, including the turn
    /// back onto the runway heading there. Turning arcs are charged at the
    /// reduced effective L/D of banked flight.
    func continuation(toAlong along: Double, height: Double) -> (path: Double, cost: Double) {
      let aim = frame.position(along: along, cross: 0, height: height)
      let d = frame.distance(from: state.position, to: aim)
      let leg = Self.turnThenStraight(distance: d, bearing: frame.bearing(from: state.position, to: aim) - course,
        radius: radius)
      let align = abs(wrapAngle(runwayHeading - (course + leg.turn))) * radius
      let arcs = abs(leg.turn) * radius + align
      let path = leg.length + align
      return (path, (path - arcs) + arcs / turnCos)
    }

    // NEP continuation (requires a solved NEP: no configured substitute).
    if state.time - lastNEPTime >= c.guidance.terminalReplanInterval {
      lastNEP = nepSolver.solve(from: state, trimAlphaCeiling: attainableAlphaCeiling)
      lastNEPTime = state.time
    }
    let nepAlong = -c.guidance.finalDistance
    var alignmentAffordable = false
    var alignmentCost = Double.infinity, alignmentRange = 0.0
    if let nep = lastNEP, p.along < nepAlong {
      let nepEnergyHeight = nep.height + nep.airspeed * nep.airspeed / (2 * g)
      alignmentCost = continuation(toAlong: nepAlong, height: nep.height).cost
      alignmentRange = glideRange(from: energyHeight, to: nepEnergyHeight, derate: conservative)
      // Hysteresis is the model's own uncertainty, not a tuned gate.
      let margin = lastMode == .alignment ? 1.0 : 1 + u
      alignmentAffordable = alignmentRange >= alignmentCost * margin
    }
    // Minimum-loss direct continuation: any centerline aim from the NEP
    // station to the last touchdown point that still leaves stopping distance.
    let stopping = c.vehicle.landingSpeed * c.vehicle.landingSpeed / (2 * c.vehicle.brakeFriction * g)
    let latestTouchdown = max(0, c.runway.length - stopping)
    var direct = (along: max(0, min(latestTouchdown, p.along)), path: Double.infinity, cost: Double.infinity)
    for i in 0...32 {
      let along = nepAlong + (latestTouchdown - nepAlong) * Double(i) / 32
      let leg = continuation(toAlong: along, height: 0)
      // Up-range aims still have to glide the centerline to the threshold.
      let cost = leg.cost + max(0, -along)
      if cost < direct.cost { direct = (along, leg.path + max(0, -along), cost) }
    }

    // Stretching the glide is justified only when even the NOMINAL model is
    // short; an uncertain shortfall must not trade away energy management.
    let mode: TerminalRecoveryMode = alignmentAffordable ? .alignment
      : nominalRange >= direct.cost ? .directRunway : .runwayUnreachable
    lastMode = mode
    let aimAlong = mode == .alignment ? nepAlong : direct.along
    let aimHeight = mode == .alignment ? (lastNEP?.height ?? 0) : 0
    let aim = frame.position(along: aimAlong, cross: 0, height: aimHeight)
    let distance = frame.distance(from: state.position, to: aim)
    // Excess energy with no plan and no modeled speedbrake: lengthen the
    // ground track with S-turns about the centerline until the turn-priced
    // path matches the conservatively affordable range. Path per distance
    // made good on a dogleg at offset psi is 1/cos(psi). Only while there is
    // room for a reversal and the final alignment turn (a turn diameter each).
    var dissipationPath = alignmentCost
    var headingOffset = 0.0
    if mode == .alignment, alignmentRange > alignmentCost * (1 + u), alignmentCost > 4 * radius {
      dissipationPath = alignmentRange
      headingOffset = (p.cross > 0 ? -1.0 : 1.0) * acos(clamp(alignmentCost / alignmentRange, 0, 1))
    }
    let bearingError = wrapAngle(frame.bearing(from: state.position, to: aim) + headingOffset - course)
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let horizontal = state.velocity.projected(onPlaneWithNormal: state.position.normalized)
    let alignment = horizontal.normalized.dot(forward)
    let crossRate = -frame.coordinateVelocity(at: state.position, velocity: state.velocity).y
    // An S-turn leg is a commanded heading, not a centerline intercept: the
    // intercept law would cancel it while the track is runway-aligned.
    let demand = headingOffset != 0
      ? 2 * vh * sin(clamp(bearingError, -.pi / 2, .pi / 2)) / c.guidance.pathLookaheadTime
      : EntryLateralController.demand(state: state, configuration: c,
        azimuthError: bearingError, crossError: -p.cross, crossRate: crossRate,
        alignment: alignment, range: abs(aimAlong - p.along))
    let authority = NormalForceEnvelope(maximumLift: maxLift, maximumBank: c.vehicle.maximumBank)
      .lateralAuthority(reserving: support)
    let gear = state.systems.gearDown || p.height <= (c.guidance.gearDeploymentAltitude ?? 1500)

    guard mode == .alignment, let nep = lastNEP else {
      // Low energy: fly the attainable glide point; never trade range for a
      // vertical-force or alignment objective the energy cannot pay for.
      // Below the configured minimum safe speed, restore airspeed with the
      // lowest AoA whose L/D is within the model's own uncertainty of best.
      var chosen = glidePoint
      if speed < (c.vehicle.minimumSafeSpeed ?? c.vehicle.landingSpeed) {
        chosen = points.filter { ratio($0) >= glideRatio * (1 - u) }.min { $0.alpha < $1.alpha } ?? glidePoint
      }
      let lateral = min(abs(demand), chosen.lift * sin(c.vehicle.maximumBank))
      let bank = (demand < 0 ? -1.0 : 1.0) * (chosen.lift > 0 ? asin(clamp(lateral / chosen.lift, 0, 1)) : 0)
      lastAlpha = chosen.alpha
      // Geometric descent to the chosen pavement aim over the priced path.
      let glideGamma = -atan(max(0, p.height) / max(1, direct.path))
      return .init(target: GuidanceTarget(angleOfAttack: chosen.alpha, bank: bank,
        desiredSpeed: speed, desiredHeight: p.height + c.runway.elevation,
        desiredFlightPath: glideGamma, gearDown: gear, speedbrakeFraction: 0),
        mode: mode, lateralDemand: demand, lateralAvailable: authority, selectedDrag: chosen.drag,
        energyHeight: energyHeight, availableRange: availableRange, requiredRange: direct.path,
        aimAlong: aimAlong, turnRadius: radius, nominalRange: nominalRange)
    }

    // Sufficient energy: acquire Final alignment at the solved NEP with a
    // drag budget priced over the turn-aware path, not the straight distance.
    let h = state.position.length - c.planet.radius
    let horizon = max(c.guidance.pathLookaheadTime, distance / vh)
    let preferredHeight = profile.reference(along: aimAlong).height + c.runway.elevation
    // Descend over the path actually to be flown (including S-turns). Never
    // command a climb, but DO allow shallowing a steep dive: holding the
    // current path angle as a ceiling made the 2211 flight dive at -35 deg
    // and convert excess height into excess speed.
    let geometricGamma = atan((preferredHeight - h) / max(dissipationPath, vh * c.guidance.pathLookaheadTime))
    let gamma = state.verticalSpeed < 0 ? min(geometricGamma, 0) : geometricGamma
    let pitchTime = max(c.guidance.pathLookaheadTime, pitchResponseSeconds ?? 0)
    let vertical = support + speed * (gamma - state.flightPathAngle) / pitchTime
    let finalPosition = frame.position(along: aimAlong, cross: 0, height: preferredHeight - c.runway.elevation)
    let energy = state.velocity.squaredLength / 2 - c.planet.gravitationalParameter / state.position.length
      - c.planet.angularVelocity.cross(state.position).squaredLength / 2
    let finalEnergy = nep.airspeed * nep.airspeed / 2 - c.planet.gravitationalParameter / finalPosition.length
      - c.planet.angularVelocity.cross(finalPosition).squaredLength / 2
    let path = max(dissipationPath, vh * c.guidance.pathLookaheadTime) * speed / vh
    let dragBudget = max(0, energy - finalEnergy) / max(1, path)
    let allocation = EntryForceAllocator.decide(points: points, verticalDemand: vertical,
      lateralDemand: demand, referenceDrag: dragBudget, dragBudget: dragBudget,
      maximumBank: c.vehicle.maximumBank, loadLimit: loadLimit,
      uncertainty: u, previousAlpha: lastAlpha ?? a.angleOfAttack,
      heightAboveCapture: h - c.entrySettings.terminalMinimumAltitude,
      timeToCapture: horizon)
    lastAlpha = allocation.alpha
    let existingBrake = a.dynamicPressure * (c.vehicle.speedbrakeFullDeploymentDragArea ?? 0)
      * (state.systems.speedbrakeFraction ?? 0)
    let cleanDrag = a.measuredForces ? max(0, a.drag - existingBrake) / state.mass : allocation.drag
    let brake = speedbrake.fraction(speed: a.airspeed, position: state.position, time: state.time,
      mass: state.mass, remainingDistance: alignmentCost, target: nep, cleanDragAcceleration: cleanDrag)
    return .init(target: GuidanceTarget(angleOfAttack: allocation.alpha, bank: allocation.bank,
      desiredSpeed: nep.airspeed, desiredHeight: preferredHeight, desiredFlightPath: gamma,
      gearDown: gear, speedbrakeFraction: brake),
      mode: mode, lateralDemand: demand, lateralAvailable: authority, selectedDrag: allocation.drag,
      energyHeight: energyHeight, availableRange: alignmentRange, requiredRange: alignmentCost,
      aimAlong: aimAlong, turnRadius: radius, nominalRange: nominalRange)
  }
}
