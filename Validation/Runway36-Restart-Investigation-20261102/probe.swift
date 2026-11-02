import XCTest
import KSLCore
import KSLPhysics
import KSLRuntime
@testable import KSLGuidance

final class ZZRunway36InvestigationTests: XCTestCase {
  func testRecordedHandoff() throws {
    let base=URL(fileURLWithPath:"/Users/kyooni18/Code/KSL/Validation/Runway36-Restart-Investigation-20261102")
    let decoder=JSONDecoder()
    let c=try decoder.decode(MissionConfiguration.self,from:Data(contentsOf:base.appendingPathComponent("configuration.json")))
    let atm=try AtmosphericModelEngine(planet:c.planet)
    let aero=try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea)
    let records=try String(contentsOf:base.appendingPathComponent("entry-through-handoff.jsonl"),encoding:.utf8).split(separator:"\n").map {try decoder.decode(FlightRecord.self,from:Data($0.utf8))}
    let computer=try FlightComputer(configuration:c,atmosphere:atm,aerodynamics:aero,externalPlanning:true)
    let authority=AerodynamicAuthorityEstimator(model:aero)
    for record in records.dropLast() {
      if let bytes=record.planningProposal {computer.accept(try decoder.decode(MissionPlanningProposal.self,from:bytes),state:record.state)}
      _=try computer.step(record.state)
      if record.dynamicsChange != nil {authority.reset()}
      authority.observe(record.state)
    }
    let last=records.last!, state=last.state
    if last.dynamicsChange != nil {authority.reset()}
    authority.observe(state)
    let ceiling=computer.mission.measuredAttainableAlphaCeiling
    let nep=try XCTUnwrap(NominalEnergyPointSolver(configuration:c,atmosphere:atm,aerodynamics:aero).solve(from:state,trimAlphaCeiling:ceiling))
    let engine=InvestigationTAEMEngine(configuration:c,atmosphere:atm,aerodynamics:aero)
    let t=engine.update(state,nep:nep,liftGain:authority.liftEffectiveness,dragGain:authority.dragEffectiveness,attainableAlphaCeiling:ceiling,pitchResponseSeconds:computer.controller.diagnostics.pitchResponseSeconds,rollResponseSeconds:computer.controller.diagnostics.rollResponseSeconds)
    print("PROBE_GAINS",authority.liftEffectiveness,authority.dragEffectiveness,"CEILING",ceiling as Any,"NEP",nep)
    print("PROBE_TARGET",t,"STATUS",engine.status,"DEBUG",engine.debug)
    print("RECORDED_TARGET",last.guidance.target)
    XCTAssertEqual(engine.status.remainingTurn, radians(536.6033364977496),accuracy:1e-8)
    XCTAssertEqual(engine.status.descentCheckFailure,"trimLift")
  }
}
final class InvestigationTAEMEngine {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aero: AerodynamicsEngine
  private let frame: RunwayFrame
  private let rotation: RotationalDynamicsEngine
  private let energyModel: RunwayEnergy
  private(set) var status = TAEMStatus()
  private(set) var interfaceFailure: String?
  var debug = "" // SCRATCH
  var debugLines: String { guard let l = lines else { return "" }; return stride(from: 0.0, through: 60000, by: 4000).map { d in let n = l.nominal.sample(d), m = l.minimum.sample(d); return String(format: "%.0f: EN %.0f h %.0f q %.0f | EMIN %.0f h %.0f q %.0f", d, n.energy, n.height, n.dynamicPressure, m.energy, m.height, m.dynamicPressure) }.joined(separator: "\n") } // SCRATCH
  private var geometry: (side: Int, finalRadius: Double, growth: Double, turn: Double)?
  private var geometryAdapted = false
  private var lines: (time: Double, side: Int, turn: Double, nominal: TAEMEnergyLine, minimum: TAEMEnergyLine)?
  private var sTurnSign = 0.0
  private var hacPolarAngle: Double?
  private var hacPosition: Vector2?
  private var captureTurn: Double?
  private var joinBlend: (time: Double, lateral: Double)?
  private var descent: TAEMDescentReference?
  private struct DescentCheck {
    var energy: Double?
    var failure: String?
  }
  private var descentCheck: (time: Double, result: DescentCheck)?

  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    energyModel = .init(configuration: configuration)
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet)
  }

  func reset() {
    status = .init(); geometry = nil; geometryAdapted = false; lines = nil; sTurnSign = 0
    captureTurn = nil; descent = nil; hacPolarAngle = nil; hacPosition = nil; descentCheck = nil; joinBlend = nil
  }

  /// Ground position repeats every winding. Only measured angular progress
  /// can distinguish the overhead pass from the final pass at that position.
  static func advanceTurn(_ remaining: Double, from previousPolar: Double, to polar: Double,
                          maximumProgress: Double = .infinity) -> Double {
    // Inside the HAC, polar angle can close much faster than distance flown.
    // Reference station must not spend more arc than actual ground travel.
    max(0, remaining + max(-maximumProgress, wrapAngle(polar-previousPolar)))
  }

  // MARK: Geometry (TGXHAC / GTP)

  /// Bank of the nominal geometry: the load factor that remains after the
  /// configured aerodynamic uncertainty (the same reserve MM305 prices).
  private var geometryBank: Double { EnergyManagementEngine.acquisitionBank(configuration) }
  private func turnRadius(speed: Double, gravity: Double) -> Double {
    speed * speed / (gravity * tan(max(1e-3, geometryBank)))
  }
  private var nepAlong: Double { -configuration.guidance.finalDistance }
  private func referencePosition(_ height: Double) -> Vector3 {
    frame.position(along: nepAlong, cross: 0, height: height)
  }
  private func referenceSpeedSquared(energy: Double, height: Double) -> Double {
    energyModel.speedSquared(height: energy, position: referencePosition(height))
  }
  private func gravity(atHeight height: Double) -> Double {
    configuration.planet.gravitationalParameter / referencePosition(height).squaredLength
  }
  /// The reference is an atmospheric, still-air trajectory. At its upper
  /// root bound density is zero; do not assume geometric h <= energy height
  /// when gravity and centrifugal potential vary with altitude.
  private var maximumReferenceHeight: Double {
    configuration.planet.atmosphereTop - configuration.runway.elevation
  }

  /// Spiral length from remaining angle `theta` to the NEP.
  static func spiralLength(_ s: SpiralHAC, to theta: Double) -> Double {
    let n = max(8, Int(ceil(theta / 0.05)))
    var total = 0.0
    for i in 0..<n {
      let t = theta * (Double(i) + 0.5) / Double(n)
      total += hypot(s.radius(at: t), 2 * s.growth * t) * theta / Double(n)
    }
    return total
  }

  /// Remaining HAC angles whose tangent line passes through `point` with the
  /// spiral ahead of it, from straight-in to one full loop.
  static func tangencies(of s: SpiralHAC, from point: Vector2, maximumTurn: Double) -> [Double] {
    func f(_ t: Double) -> Double { let x = s.sample(remainingAngle: t); return x.tangent.cross(point - x.position) }
    var roots: [Double] = []
    let step = radians(2)
    var a = 0.0, fa = f(0)
    while a < maximumTurn {
      let b = min(maximumTurn, a + step), fb = f(b)
      if fa == 0 || fa * fb < 0 {
        var lo = a, hi = b, flo = fa
        for _ in 0..<40 {
          let mid = (lo + hi) / 2, fm = f(mid)
          if flo * fm <= 0 { hi = mid } else { lo = mid; flo = fm }
        }
        let t = (lo + hi) / 2, x = s.sample(remainingAngle: t)
        if x.tangent.dot(x.position - point) > 0 { roots.append(t) }
      }
      a = b; fa = fb
    }
    return roots
  }

  private func spiral(side: Int, finalRadius: Double, growth: Double, turn: Double) -> SpiralHAC? {
    try? SpiralHAC(exitAlong: nepAlong, finalRadius: finalRadius,
      growth: min(growth, finalRadius / 2 * (1 - 1e-6)), turnAngle: max(1e-6, turn), side: side)
  }

  /// HAC size: the final radius flies the NEP airspeed, the spiral grows so
  /// a full loop's entry radius flies the front-side speed there, both at
  /// the uncertainty-reserved bank and never below the configured minimum.
  /// Sized once from the first reference lines, then fixed like the
  /// Shuttle's R1/R2: a HAC that grows with the lines moves under the
  /// vehicle approaching it.
  private func prepareGeometry(nep: NominalEnergyPoint, gravity g: Double, adapt: Bool) {
    if geometry == nil {
      let rf = max(configuration.guidance.minimumHACRadius, turnRadius(speed: nep.airspeed, gravity: g))
      // Provisional minimal growth until the reference line gives the HAC
      // entry speed (SpiralHAC needs 0 < growth < rf/2). Growing from the
      // NEP side converges on the compact spiral, not the radius cap.
      geometry = (1, rf, rf * 1e-3 / (4 * .pi * .pi), 2 * .pi)
    }
    if adapt, !geometryAdapted, let line = lines?.minimum, let hac = spiral(side: geometry!.side,
      finalRadius: geometry!.finalRadius, growth: geometry!.growth, turn: 2 * .pi) {
      let entry = line.sample(Self.spiralLength(hac, to: 2 * .pi))
      let entrySpeed = sqrt(referenceSpeedSquared(energy: entry.energy, height: entry.height))
      geometry!.growth = max(geometry!.finalRadius * 1e-3,
        turnRadius(speed: entrySpeed, gravity: g) - geometry!.finalRadius) / (4 * .pi * .pi)
      geometryAdapted = true
    }
  }

  /// TAEM interface placement (offline): the point on HREF at configured
  /// interface Mach, flying an overhead HAC of `turn`.
  /// Returns that range and the HAC part of it. The Shuttle's 52 nm comes
  /// from the orbiter's polar; this vehicle's supersonic L/D is below one.
  func nominalInterface(state: VehicleState, nep: NominalEnergyPoint, turn: Double = 2 * .pi)
    -> (range: Double, hacLength: Double, height: Double, speed: Double, q: Double, slope: Double)? {
    let c = configuration
    let g = c.planet.gravitationalParameter / frame.origin.squaredLength
    let ceiling = rotation.supportedTrimAlpha(state, identifiedCeiling: nil)
    reset()
    interfaceFailure = nil
    var range = c.entrySettings.terminalTargetRange, hacLength = 0.0
    var result: (range: Double, hacLength: Double, height: Double, speed: Double, q: Double, slope: Double)?
    for _ in 0..<6 {
      geometryAdapted = false
      prepareGeometry(nep: nep, gravity: g, adapt: true)
      guard let hac = spiral(side: 1, finalRadius: geometry!.finalRadius, growth: geometry!.growth, turn: turn)
      else { return nil }
      hacLength = Self.spiralLength(hac, to: turn)
      let line = makeLine(state: state, nep: nep, maximumRange: c.guidance.maximumPathLength, hac: hac, hacTurn: turn,
        liftGain: 1, dragGain: 1, alphaCeiling: ceiling, nominal: true)
      let minimum = makeLine(state: state, nep: nep, maximumRange: c.guidance.maximumPathLength, hac: hac, hacTurn: turn,
        liftGain: 1, dragGain: 1, alphaCeiling: ceiling, nominal: false)
      lines = (state.time, 1, turn, line, minimum)
      // Mach remains an explicit configured regime boundary; altitude and
      // range must be on the same attainable HREF, not a copied Earth pair.
      func mach(_ d: Double) -> Double {
        let x = line.sample(d)
        let speed = sqrt(referenceSpeedSquared(energy: x.energy, height: x.height))
        return speed / atmosphere.sample(position: referencePosition(x.height), time: state.time).speedOfSound
      }
      guard let i = line.range.indices.dropFirst().first(where: {
        line.range[$0] >= hacLength && mach(line.range[$0]) >= c.mm304DeliveryMach
      }) else { interfaceFailure = "Configured Mach not reached within maximumPathLength"; reset(); return nil }
      var lo = max(hacLength, line.range[i - 1]), hi = line.range[i]
      guard mach(lo) <= c.mm304DeliveryMach else {
        interfaceFailure = "full HAC entry Mach \(mach(hacLength)), configured interface Mach \(c.mm304DeliveryMach), HAC length \(hacLength)"
        reset(); return nil
      }
      for _ in 0..<32 {
        let mid = (lo + hi) / 2
        if mach(mid) < c.mm304DeliveryMach { lo = mid } else { hi = mid }
      }
      range = (lo + hi) / 2
      let x = line.sample(range)
      let speed = sqrt(referenceSpeedSquared(energy: x.energy, height: x.height))
      let rho = atmosphere.sample(position: referencePosition(x.height), time: state.time).density
      result = (range, hacLength, x.height, speed, 0.5 * rho * speed * speed, line.slope(range))
    }
    reset()
    return result
  }

  // MARK: Reference lines (TGCOMP)

  private func makeLine(state: VehicleState, nep: NominalEnergyPoint, maximumRange: Double,
                        hac: SpiralHAC?, hacTurn: Double,
                        acquisition: (span: ClosedRange<Double>, load: Double)? = nil,
                        liftGain: Double, dragGain: Double,
                        alphaCeiling: Double, nominal: Bool) -> TAEMEnergyLine {
    let c = configuration, v = c.vehicle
    let g = c.planet.gravitationalParameter / frame.origin.squaredLength
    let hacLength = hac.map { Self.spiralLength($0, to: hacTurn) } ?? 0
    // Spiral curvature by distance before the NEP, for the turn load.
    func curvature(_ d: Double) -> Double {
      guard let hac, d < hacLength, hacLength > 0 else { return 0 }
      return abs(hac.sample(remainingAngle: hacTurn * d / hacLength).curvature)
    }
    func air(_ h: Double) -> AtmosphericState {
      atmosphere.sample(position: frame.position(along: nepAlong, cross: 0, height: max(0, h)), time: state.time)
    }
    let floor = min(alphaCeiling, max(0, v.minimumAngleOfAttack))
    // Front-side q: best-L/D AoA carries weight times the turn load.
    func best(mach: Double, q: Double) -> AerodynamicCoefficients {
      let alpha = aero.bestGlideAlpha(mach: mach, maximumAlpha: alphaCeiling, dynamicPressure: q,
        gear: state.systems.gearDown)
      return aero.coefficients(mach: mach, alpha: alpha, dynamicPressure: q, gear: state.systems.gearDown)
    }
    func frontQ(mach: Double, q: Double, load: Double, gravity: Double) -> Double {
      gravity * load * state.mass / max(1e-9, aero.referenceArea * best(mach: mach, q: q).lift * liftGain)
    }
    // Schedule (cf. QBREF): the minimum line flies the front-side (best-L/D)
    // q; the nominal flies the q at which the modelled drag is the drag the
    // configured aerodynamic uncertainty allows (best L/D / reserve), so a
    // vehicle tracking its altitude (HREF) on EN spends that reserve instead
    // of carrying it to the NEP. Both rise to the NEP q over the distance the
    // NEP glide slope needs to gain that speed: kinetic height gained per
    // metre is tan|gamma_NEP| - 1/(L/D). Each line is priced where it is
    // flown, so near the NEP, where the ramp already exceeds the reserved
    // q, no reserve is priced that the vehicle could not spend.
    let reserve = 1 + v.aerodynamicUncertainty
    func reservedQ(mach: Double, q: Double, gravity: Double, load: Double) -> Double {
      let bestAlpha = aero.bestGlideAlpha(mach: mach, maximumAlpha: alphaCeiling, dynamicPressure: q,
        gear: state.systems.gearDown)
      func coefficients(_ alpha: Double) -> AerodynamicCoefficients {
        aero.coefficients(mach: mach, alpha: alpha, dynamicPressure: q, gear: state.systems.gearDown)
      }
      func ratio(_ alpha: Double) -> Double { coefficients(alpha).lift / max(1e-9, coefficients(alpha).drag) }
      let target = ratio(bestAlpha) / reserve
      var lo = min(floor, bestAlpha), hi = bestAlpha
      if ratio(lo) >= target { hi = lo }
      for _ in 0..<24 where hi > lo {
        let mid = (lo + hi) / 2
        if ratio(mid) < target { lo = mid } else { hi = mid }
      }
      return min(v.maximumDynamicPressure,
        gravity * load * state.mass / max(1e-9, aero.referenceArea * max(1e-6, coefficients(hi).lift) * liftGain))
    }
    let nepAir = air(nep.height)
    let nepQ = 0.5 * nepAir.density * nep.airspeed * nep.airspeed
    let nepMach = nep.airspeed / max(1, nepAir.speedOfSound)
    let nepFront = frontQ(mach: nepMach, q: nepQ, load: 1, gravity: gravity(atHeight: nep.height))
    let nepPolar = best(mach: nepMach, q: nepQ)
    let gain = tan(abs(nep.flightPathAngle)) - nepPolar.drag * dragGain / max(1e-9, nepPolar.lift * liftGain)
    let ramp = max(0, (nepQ - nepFront) / nepAir.density / gravity(atHeight: nep.height)) / max(1e-3, gain)
    func schedule(_ d: Double) -> Double { ramp > 0 ? nepQ * max(0, 1 - d / ramp) : 0 }
    let steps = 240, dd = maximumRange / Double(steps)
    var out = TAEMEnergyLine()
    var energy = energyModel.height(position: referencePosition(nep.height), speedSquared: nep.airspeed * nep.airspeed)
    var previousHeight = nep.height
    for i in 0...steps {
      let d = Double(i) * dd
      // Solve the altitude on this energy level whose q meets the schedule.
      var lo = 0.0, hi = maximumReferenceHeight
      var load = 1.0
      for _ in 0..<28 {
        let h = (lo + hi) / 2, a = air(h), speed = sqrt(referenceSpeedSquared(energy: energy, height: h))
        let q = 0.5 * a.density * speed * speed
        // The HAC radius follows the line's own speed; the acquisition arc's
        // load is fixed by how it is flown (see acquisitionLeg).
        if let acquisition, acquisition.span.contains(d) { load = acquisition.load }
        else { load = hypot(1, speed * speed * curvature(d) / gravity(atHeight: h)) }
        // No more load than the configured bank limit can fly.
        load = min(load, 1 / cos(v.maximumBank))
        // A turn raises the front-side q by its load (as the flight law's
        // q floor does), else best L/D could not carry it.
        let mach = speed / max(1, a.speedOfSound), localGravity = gravity(atHeight: h)
        let front = frontQ(mach: mach, q: q, load: load, gravity: localGravity)
        let target = max(schedule(d), front,
          nominal ? reservedQ(mach: mach, q: q, gravity: localGravity, load: load) : front)
        if q > target { lo = h } else { hi = h }
      }
      let h = lo, a = air(h), speed = sqrt(referenceSpeedSquared(energy: energy, height: h))
      let q = 0.5 * a.density * speed * speed, mach = speed / max(1, a.speedOfSound)
      let reference = h
      out.append(range: d, energy: energy, height: reference, dynamicPressure: q,
        frontDynamicPressure: frontQ(mach: mach, q: q, load: load, gravity: gravity(atHeight: h)))
      // Drag where the line is flown (HREF for the nominal), of the AoA that
      // carries the load, with measured force gains.
      let flown = air(reference), flownSpeed = sqrt(referenceSpeedSquared(energy: energy, height: reference))
      let flownQ = 0.5 * flown.density * flownSpeed * flownSpeed
      let flownMach = flownSpeed / max(1, flown.speedOfSound)
      let lift = aero.liftSolution(acceleration: gravity(atHeight: reference) * load / max(1e-9, liftGain), mass: state.mass,
        dynamicPressure: flownQ, mach: flownMach, minimumAlpha: floor, maximumAlpha: alphaCeiling,
        gear: state.systems.gearDown)
      let cd = aero.coefficients(mach: flownMach, alpha: lift.alpha, dynamicPressure: flownQ,
        gear: state.systems.gearDown).drag * dragGain
      let slope = i == 0 ? tan(abs(nep.flightPathAngle)) : (reference - previousHeight) / dd
      previousHeight = reference
      energy += flownQ * aero.referenceArea * cd / (state.mass * g) * dd * hypot(1, slope)
    }
    return out
  }

  /// Necessary force/energy check on the commanded altitude profile, not the
  /// unrelated q-scheduled EN/EMIN curve. This still-air point-mass check can
  /// reject a path; passing is not an actuator or closed-loop certificate.
  private func checkDescent(state: VehicleState, profile: TAEMDescentReference,
    range: Double, hac: SpiralHAC, turn: Double, acquisitionArc: Double,
    acquisitionCurvature: Double, liftGain: Double, dragGain: Double,
    alphaCeiling: Double) -> DescentCheck {
    let c = configuration, v = c.vehicle
    let hacLength = Self.spiralLength(hac, to: turn)
    var remaining = range, energy = energyModel.height(state), elapsed = 0.0
    var gearDown = state.systems.gearDown
    func station(_ d: Double, _ e: Double) -> (speed: Double, loss: Double, failure: String?) {
      let ref = profile.sample(d)
      let position = referencePosition(ref.height)
      let speedSquared = energyModel.speedSquared(height: e, position: position)
      guard speedSquared > 0 else { return (0, 0, "kineticEnergy") }
      let speed = sqrt(speedSquared), metric = hypot(1, ref.slope)
      let air = atmosphere.sample(position: position, time: state.time + elapsed)
      guard air.wind.length == 0 else { return (speed, 0, "windUnpriced") }
      let gear = gearDown || ref.height <= (c.guidance.gearDeploymentAltitude ?? 1500)
      let q = 0.5 * air.density * speedSquared
      guard q <= v.maximumDynamicPressure else { return (speed, 0, "dynamicPressure") }
      // Invert spiral arclength instead of assuming angle proportional to d.
      var curvature = 0.0
      if d < hacLength {
        var lo = 0.0, hi = turn
        for _ in 0..<24 {
          let mid = (lo + hi) / 2
          if Self.spiralLength(hac, to: mid) < d { lo = mid } else { hi = mid }
        }
        curvature = abs(hac.sample(remainingAngle: (lo + hi) / 2).curvature)
      } else if d > range - acquisitionArc { curvature = acquisitionCurvature }
      let horizontalSquared = speedSquared / (metric * metric)
      let normal = profile.normalAcceleration(at: d, speedSquared: speedSquared,
        gravity: gravity(atHeight: ref.height), radius: position.length)
      guard normal >= 0 else { return (speed, 0, "negativeNormalLift") }
      let lateral = horizontalSquared * curvature
      let bank = atan2(lateral, normal), demand = hypot(normal, lateral)
      guard bank <= v.maximumBank else { return (speed, 0, "bank") }
      guard demand <= v.maximumLoadFactor * 9.80665 else { return (speed, 0, "load") }
      let solution = aero.liftSolution(acceleration: demand / max(1e-9, liftGain),
        mass: state.mass, dynamicPressure: q, mach: speed / air.speedOfSound,
        minimumAlpha: max(0, v.minimumAngleOfAttack), maximumAlpha: alphaCeiling,
        gear: gear)
      guard solution.achievable,
        demand <= solution.maximumAcceleration * liftGain * (1-v.aerodynamicUncertainty) else {
        print("TRIM_FAIL", "range",range,"station",d,"height",ref.height,"speed",speed,"q",q,"normal",normal,"lateral",lateral,"bank",degrees(bank),"demand",demand,"max",solution.maximumAcceleration*liftGain*(1-v.aerodynamicUncertainty),"ceiling",degrees(alphaCeiling),"gain",liftGain,"arc",acquisitionArc,"curvature",acquisitionCurvature)
        return (speed, 0, "trimLift") }
      let coefficients = aero.coefficients(mach: speed / air.speedOfSound,
        alpha: solution.alpha, dynamicPressure: q, gear: gear)
      let produced = q * aero.referenceArea * coefficients.lift * liftGain / state.mass
      // A nonzero minimum incidence cannot supply less lift than its polar.
      // 1e-6 relative acceleration is only interpolation roundoff tolerance.
      guard produced <= demand + 1e-6 * max(1, demand) else { return (speed, 0, "minimumLift") }
      let loss = q * aero.referenceArea * coefficients.drag * dragGain / state.mass
        * metric / energyModel.gravity
      return (speed, loss, nil)
    }
    while remaining > 0, elapsed < c.guidance.maximumPredictionDuration {
      let start = station(remaining, energy)
      if let failure = start.failure { return .init(energy: nil, failure: failure) }
      let horizontal = start.speed / hypot(1, profile.sample(remaining).slope)
      let distance = min(remaining, horizontal * c.guidance.predictionStep)
      guard distance > 0 else { return .init(energy: nil, failure: "noProgress") }
      let mid = station(remaining - distance / 2, energy - start.loss * distance / 2)
      if let failure = mid.failure { return .init(energy: nil, failure: failure) }
      energy -= mid.loss * distance
      elapsed += distance / horizontal
      remaining = max(0, remaining - distance)
      gearDown = gearDown || profile.sample(remaining).height <= (c.guidance.gearDeploymentAltitude ?? 1500)
    }
    guard remaining == 0 else { return .init(energy: nil, failure: "predictionHorizon") }
    let end = station(0, energy)
    return .init(energy: end.failure == nil ? energy : nil, failure: end.failure)
  }

  // MARK: Update

  func update(_ state: VehicleState, nep: NominalEnergyPoint, liftGain: Double, dragGain: Double,
              attainableAlphaCeiling: Double?, pitchResponseSeconds: Double?,
              rollResponseSeconds: Double?) -> GuidanceTarget {
    let c = configuration, v = c.vehicle, a = state.aerodynamics
    let g = c.planet.gravitationalParameter / state.position.squaredLength
    let p = frame.coordinates(of: state.position)
    let point = p.horizontal
    let velocity = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
    let vh = max(1, velocity.length), speed = max(1, a.airspeed)
    let tau = max(c.guidance.pathLookaheadTime, pitchResponseSeconds ?? 0)
    let rollTime = max(c.guidance.pathLookaheadTime, rollResponseSeconds ?? 0)
    let ceiling = rotation.supportedTrimAlpha(state, identifiedCeiling: attainableAlphaCeiling)
    let energy = energyModel.height(state)
    status.energyHeight = energy

    // A completed winding high above the NEP needs another recovery plan;
    // crossing the runway station cannot turn altitude into a Final capture.
    // Allow the measured descent to reach the NEP within one roll response;
    // a few metres above it must not trigger another overhead winding while
    // the vehicle is already rolling out for Final.
    if status.phase == .headingAlignment, p.along >= nepAlong,
      p.height + min(0, state.verticalSpeed) * rollTime > nep.height,
      status.remainingTurn*max(status.finalRadius,c.guidance.minimumHACRadius)
        <= vh*c.guidance.controlInterval {
      status.phase = .acquisition; lines = nil; captureTurn = nil; descent = nil; hacPolarAngle = nil; hacPosition = nil; descentCheck = nil; joinBlend = nil
    }

    // Flight commits geometry together with its tangent and descent. The
    // legacy q-line may size an offline interface, but must not resize a
    // latched flight path on the next tick (and invalidate its tangent).
    prepareGeometry(nep: nep, gravity: g, adapt: false)
    var rf = geometry!.finalRadius
    let growth = geometry!.growth

    // Ground track for each HAC side and winding (GTP).
    struct Track {
      var side: Int; var turn: Double; var range: Double; var tangent: Vector2; var hac: SpiralHAC
      /// Acquisition turn flown first, at the far end of the track, and its load.
      var arc = 0.0
      var arcLoad = 1.0
    }
    let acquisitionRadius = turnRadius(speed: vh, gravity: g)
    func track(side: Int, turn: Double) -> Track? {
      guard let hac = spiral(side: side, finalRadius: rf, growth: growth, turn: turn) else { return nil }
      let entry = hac.sample(remainingAngle: turn).position
      let offset = entry - point
      let bearing = atan2(velocity.cross(offset), velocity.dot(offset))
      let leg = Self.acquisitionLeg(distance: offset.length, bearing: bearing, radius: acquisitionRadius,
        speed: vh, gravity: g)
      return .init(side: side, turn: turn, range: leg.length + Self.spiralLength(hac, to: turn),
        tangent: entry, hac: hac, arc: leg.arc, arcLoad: leg.load)
    }
    // Absent a separate mission descent envelope, continue the configured
    // Final glide envelope upstream. nil must not authorize a vertical HAC.
    // Force checks still decide attainability; this is not a lift guarantee.
    let glideLimit = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
    func descentPlan(for t: Track) -> TAEMDescentReference {
      .init(distance: max(vh*c.guidance.controlInterval,t.range),startHeight:p.height,
        endHeight:nep.height,startSlope:max(0,-tan(state.flightPathAngle)),
        endSlope:max(0,-tan(nep.flightPathAngle)))
    }
    // Include a full overhead winding beyond the tangent, and enough
    // winding to price the altitude loss at the configured glide limit.
    // Search remains bounded by the configured maximum path length.
    let descentDistance=max(0,p.height-nep.height)/tan(glideLimit)
    let maximumTurn=min(SpiralHAC.maximumTurnAngle,c.guidance.maximumPathLength/rf,
      2 * .pi + max(2 * .pi,descentDistance/rf))
    func tracks() -> [Track] {
      var result: [Track] = []
      for side in [1,-1] {
        guard let probe=spiral(side:side,finalRadius:rf,growth:growth,turn:maximumTurn) else {continue}
        for turn in Self.tangencies(of:probe,from:point,maximumTurn:maximumTurn) {
          if let t=track(side:side,turn:turn),t.range<=c.guidance.maximumPathLength {result.append(t)}
        }
      }
      return result
    }
    var candidates=tracks()
    if lines == nil, !candidates.contains(where:{descentPlan(for:$0).fits(maximumGlide:glideLimit)}) {
      // A cubic Hermite slope is a quadratic Bezier. Keeping its middle
      // derivative control point <= tan(glide) gives a sufficient distance,
      // derived from height loss and the two endpoint slopes.
      let required=max(descentDistance,3*max(0,p.height-nep.height) /
        (tan(glideLimit)+max(0,-tan(state.flightPathAngle))+max(0,-tan(nep.flightPathAngle))))
      for _ in 0...Double.significandBitCount {
        guard let overhead=candidates.filter({$0.turn>=2 * .pi})
          .min(by:{abs($0.range-required)<abs($1.range-required)}) else {break}
        if descentPlan(for:overhead).fits(maximumGlide:glideLimit) {break}
        let next=max(c.guidance.minimumHACRadius,rf+(required-overhead.range)/overhead.turn)
        guard next<=c.guidance.maximumHACRadius,abs(next-rf)>Double.ulpOfOne*rf else {break}
        rf=next;candidates=tracks()
      }
      geometry!.finalRadius=rf
      geometryAdapted=true
    }

    // On the HAC: follow the measured remaining angle on the latched spiral.
    if status.phase == .headingAlignment {
      let polar=atan2(nepAlong-point.x,rf-Double(status.side)*point.y)
      if let previous=hacPolarAngle, let previousPosition=hacPosition {
        let angle = status.remainingTurn
        let arcPerRadian = hypot(rf + growth*angle*angle, 2*growth*angle)
        let maximumProgress = (point-previousPosition).length / arcPerRadian
        status.remainingTurn=Self.advanceTurn(status.remainingTurn,from:previous,to:polar,
          maximumProgress:maximumProgress)
      }
      hacPolarAngle=polar
      hacPosition=point
      if let latched = spiral(side: status.side, finalRadius: rf, growth: growth, turn: max(1e-6, status.remainingTurn)) {
        candidates = [.init(side: status.side, turn: status.remainingTurn,
          range: Self.spiralLength(latched, to: status.remainingTurn), tangent: point, hac: latched)]
      }
    }
    if candidates.isEmpty, let t = track(side: status.side, turn: max(status.remainingTurn, 1e-3)) {
      candidates = [t]
    }
    guard !candidates.isEmpty else {
      return .init(angleOfAttack: a.angleOfAttack, bank: 0, desiredSpeed: speed,
        desiredHeight: p.height + c.runway.elevation)
    }

    // Reference lines for the latched (or provisional) HAC.
    func refreshLines(for t: Track) {
      let range = max(t.range, candidates.map(\.range).max() ?? 0) * 1.25 + c.guidance.finalDistance
      let turn = (span: (t.range - t.arc)...t.range, load: t.arcLoad)
      let nominal = makeLine(state: state, nep: nep, maximumRange: range, hac: t.hac, hacTurn: t.turn,
        acquisition: turn,
        liftGain: liftGain, dragGain: dragGain, alphaCeiling: ceiling, nominal: true)
      let minimum = makeLine(state: state, nep: nep, maximumRange: range, hac: t.hac, hacTurn: t.turn,
        acquisition: turn,
        liftGain: liftGain, dragGain: dragGain, alphaCeiling: ceiling, nominal: false)
      lines = (state.time, t.side, t.turn, nominal, minimum)
    }

    func check(_ candidate: Track, profile: TAEMDescentReference) -> DescentCheck {
      let curvature = sqrt(max(0, candidate.arcLoad*candidate.arcLoad-1))*g/(vh*vh)
      return checkDescent(state: state, profile: profile, range: candidate.range,
        hac: candidate.hac, turn: candidate.turn, acquisitionArc: candidate.arc,
        acquisitionCurvature: curvature, liftGain: liftGain, dragGain: dragGain,
        alphaCeiling: ceiling)
    }
    let requiredNEPEnergy = energyModel.height(position: referencePosition(nep.height),
      speedSquared: nep.airspeed*nep.airspeed)

    // Selection (overhead default, below). Latched; one low-energy downmode.
    // The side is latched once chosen (only the downmode changes it); the
    // winding follows the measured geometry on that side.
    var selected: Track?
    if lines != nil, status.phase != .headingAlignment, let captureTurn {
      selected=track(side:status.side,turn:captureTurn)
    } else if lines != nil {
      selected=candidates.first
    }
    // Never jump to another winding: past the tangency, keep this one.
    if lines != nil, selected == nil, let t = track(side: status.side, turn: max(status.remainingTurn, 1e-3)) {
      selected = t
    }
    if lines == nil || selected == nil {
      // Use the same physical profile check for every candidate. Legacy
      // EN/EMIN remains telemetry/advisory, never a route admission price.
      let fitting = candidates.filter {descentPlan(for: $0).fits(maximumGlide: glideLimit)}
      let options = (fitting.isEmpty ? candidates : fitting).sorted {$0.range > $1.range}
      for candidate in options {
        let result = check(candidate, profile: descentPlan(for: candidate))
        if let delivered = result.energy, delivered >= requiredNEPEnergy {
          selected = candidate
          descentCheck = (state.time, result)
          break
        }
      }
      // No admissible candidate is not permission to abandon control or
      // claim a cheaper trajectory. Keep the overhead recovery unqualified.
      if selected == nil { selected = options.first; descentCheck = nil }
      captureTurn=selected!.turn
      descent=descentPlan(for:selected!)
      refreshLines(for: selected!)
    } else if state.time - lines!.time >= c.guidance.terminalReplanInterval
      || lines!.side != selected!.side || abs(lines!.turn - selected!.turn) > radians(10) {
      refreshLines(for: selected!)
    }
    debug = candidates.map { "side \($0.side) turn \(degrees($0.turn)) range \($0.range) arc \($0.arc) EN \(lines!.nominal.sample($0.range).energy)" }.joined(separator: "; ") + " acqR \(acquisitionRadius)" // SCRATCH
    var t = selected!
    if descentCheck == nil || state.time - descentCheck!.time >= c.guidance.terminalReplanInterval {
      // An unfollowed profile is stale even if a later force check passes.
      // Re-enter tracking from measured height and slope, never by catching
      // up to altitude lost only by the rejected prediction.
      if status.referenceTracking != true { descent = descentPlan(for: t) }
      var current = check(t, profile: descent ?? descentPlan(for: t))
      // A rejected profile is not a control reference. Rebuild from the
      // measured height/slope while preserving the selected ground track.
      // Commit a replacement only after the same force/energy check passes.
      if current.energy == nil || current.energy! < requiredNEPEnergy {
        let regenerated = descentPlan(for: t)
        let result = check(t, profile: regenerated)
        if let delivered = result.energy, delivered >= requiredNEPEnergy {
          descent = regenerated
          current = result
        }
      }
      descentCheck = (state.time, current)
      // Both sides of a downmode must refer to the profile actually flown.
      // A failed force check means unknown delivery, not a fabricated energy
      // price. Retain recovery unless another route passes the same check.
      if status.phase != .headingAlignment, !status.downmoded,
        current.energy == nil || current.energy! < requiredNEPEnergy {
        for shorter in candidates.filter({$0.range < t.range}).sorted(by: {$0.range > $1.range}) {
          let proposed = descentPlan(for: shorter)
          guard proposed.fits(maximumGlide: glideLimit) else { continue }
          let alternative = check(shorter, profile: proposed)
          guard let delivered = alternative.energy, delivered >= requiredNEPEnergy else { continue }
          status.downmoded = true; t = shorter; refreshLines(for: t)
          captureTurn = t.turn; descent = proposed
          descentCheck = (state.time, alternative)
          break
        }
      }
    }
    status.descentCheckFailure = descentCheck?.result.failure
    status.predictedNEPEnergy = descentCheck?.result.energy
    status.side = t.side; status.remainingTurn = t.turn; status.range = t.range
    status.finalRadius = rf; status.growth = growth
    if descent == nil {descent=descentPlan(for:t)}
    status.descentPlanFits=descent!.fits(maximumGlide:glideLimit)
    let planned=descent!.sample(t.range)
    let referenceUsable = status.descentPlanFits && descentCheck?.result.failure == nil
      && (descentCheck?.result.energy ?? -Double.infinity) >= requiredNEPEnergy
    status.referenceTracking = referenceUsable
    let nominal = lines!.nominal.sample(t.range), minimum = lines!.minimum.sample(t.range)
    status.nominalEnergy = nominal.energy; status.minimumEnergy = minimum.energy
    status.insufficientEnergy = energy < minimum.energy

    // Phase transitions (TGTRAN).
    let toTangent = t.tangent - point
    let headingError = atan2(velocity.cross(toTangent), velocity.dot(toTangent))
    // S-turn initiation line (TGTRAN's ES above EN): the nominal's own
    // margin above the minimum line, mirrored above EN. A smaller excess is
    // absorbed by q; turning this vehicle is costly, so do not S-turn on it.
    let sTurnEnergy = nominal.energy + max(0, nominal.energy - minimum.energy)
    switch status.phase {
    case .sTurn:
      if !referenceUsable || energy <= nominal.energy { status.phase = .acquisition }
    case .acquisition:
      // Roll onto the spiral one roll response before its tangent point. An
      // S-turn turns away, back across the track and realigns: a full
      // circle of acquisition-radius turning must fit before the tangent.
      if toTangent.length <= vh * rollTime {
        joinBlend = (state.time, 2 * vh * sin(clamp(headingError, -.pi / 2, .pi / 2))
          / max(rollTime, toTangent.length / vh))
        status.phase = .headingAlignment
        let polar=atan2(nepAlong-point.x,rf-Double(t.side)*point.y)
        status.remainingTurn=max(0,t.turn+wrapAngle(polar-t.turn))
        hacPolarAngle=polar
        hacPosition=point
      } else if referenceUsable, energy > sTurnEnergy, toTangent.length > 2 * .pi * acquisitionRadius + vh * rollTime {
        status.phase = .sTurn
        sTurnSign = -Double(t.side)
        if sTurnSign * headingError < 0 && t.turn < .pi / 2 { sTurnSign = -sTurnSign }
      }
    case .headingAlignment:
      break
    }

    // Lateral (TGPHIC), as lateral acceleration.
    let support = max(0, (g - state.horizontalSpeed * state.horizontalSpeed / state.position.length)
      * cos(state.flightPathAngle))
    var lateral: Double
    switch status.phase {
    case .sTurn:
      lateral = sTurnSign * support * tan(geometryBank)
    case .acquisition:
      // Pure pursuit to the committed join, with at least the measured
      // roll response distance. A distant join is not a six-second turn.
      let pursuitTime=max(rollTime,toTangent.length/vh)
      lateral = 2 * vh * sin(clamp(headingError, -.pi / 2, .pi / 2)) / pursuitTime
    case .headingAlignment:
      let center = Vector2(nepAlong, Double(t.side) * rf)
      let radial = point - center
      let r = radial.length, rdot = velocity.dot(radial) / max(1, r)
      let reference = t.hac.radius(at: t.turn)
      let rdotReference = -vh * 2 * growth * t.turn / max(1, reference)
      let omega = 1 / max(rollTime,reference/vh)
      let centripetal = max(0, vh * vh - rdot * rdot) / max(1, reference)
      lateral = Double(t.side) * (centripetal + omega * omega * (r - reference) + 2 * omega * (rdot - rdotReference))
      // Join begins one measured roll response before the tangent. Transfer
      // acceleration continuously over that response instead of reversing
      // bank in one tick at the acquisition/HAC label change.
      if let joinBlend {
        let u = Self.smoothTransition((state.time - joinBlend.time) / rollTime)
        lateral = joinBlend.lateral + u * (lateral - joinBlend.lateral)
      }
      // The spiral ends tangent to the runway but with nonzero curvature.
      // Anticipate its straight continuation by one roll-response distance;
      // otherwise bank persists through the NEP and Final inherits the turn.
      let exitBlend = Self.smoothTransition(1 - t.range / (vh * rollTime))
      let runwayLateral = -2 * velocity.y / rollTime - point.y / (rollTime * rollTime)
      lateral += exitBlend * (runwayLateral - lateral)
    }

    // Vertical (TGNZC). Track the nominal line's altitude (HREF): energy
    // above EN then shows as speed, whose drag dissipates it. The q limits
    // (QBMNNZ/QBMXNZ) become altitude bounds for the vehicle's OWN energy:
    // never higher than its energy can hold at the front-side (best-L/D) q,
    // so altitude it cannot pay for is not chased (the 2244 collapse), and
    // never lower than it holds at the maximum q.
    let density = max(1e-9, state.atmosphere.density)
    let bestAlpha = aero.bestGlideAlpha(mach: a.mach, maximumAlpha: ceiling, dynamicPressure: a.dynamicPressure,
      gear: state.systems.gearDown)
    let bestLift = aero.coefficients(mach: a.mach, alpha: bestAlpha, dynamicPressure: a.dynamicPressure,
      gear: state.systems.gearDown).lift * liftGain
    // Price minimum q for the planned normal acceleration, not an assumed
    // level turn. A descending capture need not dive to a 1-g best-L/D q.
    let normalReference=max(0,support+speed*vh*planned.curvature/(1+planned.slope*planned.slope))
    let attainableNormal = aero.liftSolution(acceleration: 0,mass:state.mass,
      dynamicPressure:a.dynamicPressure,mach:a.mach,minimumAlpha:max(0,v.minimumAngleOfAttack),
      maximumAlpha:ceiling,gear:state.systems.gearDown).maximumAcceleration * liftGain
    let supportCoefficient = a.dynamicPressure > 0
      ? max(bestLift,attainableNormal*state.mass/(a.dynamicPressure*aero.referenceArea)) : bestLift
    // Price the lift vector we can command, not the bank still being flown
    // during roll response. Using sec(measuredBank) demanded more q (a dive)
    // even while the allocator was unloading that same turn to regain lift.
    let reservedLift = min(v.maximumLoadFactor * 9.80665,
      attainableNormal * (1-v.aerodynamicUncertainty))
    let plannedLateral = min(abs(lateral), normalReference * tan(v.maximumBank),
      sqrt(max(0, reservedLift*reservedLift-normalReference*normalReference)))
    let frontQ = hypot(normalReference, plannedLateral) * state.mass
      / max(1e-9,aero.referenceArea*supportCoefficient*(1-v.aerodynamicUncertainty))
    func height(atDynamicPressure q: Double) -> Double {
      var lo = 0.0, hi = maximumReferenceHeight
      for _ in 0..<28 {
        let h = (lo + hi) / 2
        let rho = atmosphere.sample(position: frame.position(along: p.along, cross: p.cross, height: h),
          time: state.time).density
        let position = frame.position(along: p.along, cross: p.cross, height: h)
        let groundSpeed = sqrt(energyModel.speedSquared(height: energy, position: position))
        let wind = atmosphere.sample(position: position, time: state.time).wind
        let airSpeedSquared = (state.velocity.normalized * groundSpeed - wind).squaredLength
        if 0.5 * rho * airSpeedSquared > q { lo = h } else { hi = h }
      }
      return lo
    }
    // A committed altitude plan is not permission to consume its last
    // kinetic energy chasing height. Retain enough q for attainable lift.
    let supportHeight=height(atDynamicPressure:frontQ)
    let targetHeight = referenceUsable
      ? min(supportHeight,max(planned.height,height(atDynamicPressure:v.maximumDynamicPressure)))
      : p.height
    let targetDensity = atmosphere.sample(position: frame.position(along: p.along, cross: p.cross,
      height: targetHeight), time: state.time).density
    let targetPosition = frame.position(along: p.along, cross: p.cross, height: targetHeight)
    let targetGroundSpeed = sqrt(energyModel.speedSquared(height: energy, position: targetPosition))
    let targetWind = atmosphere.sample(position: targetPosition, time: state.time).wind
    let commandedQ = 0.5 * targetDensity * (state.velocity.normalized * targetGroundSpeed - targetWind).squaredLength
    status.minimumDynamicPressure = frontQ
    status.referenceDynamicPressure = commandedQ
    status.referenceHeight = targetHeight
    let glide = glideLimit
    let dragAcceleration = a.measuredForces ? a.drag / max(1, state.mass)
      : aero.coefficients(mach: a.mach, alpha: a.angleOfAttack, dynamicPressure: a.dynamicPressure,
          gear: state.systems.gearDown).drag * a.dynamicPressure * aero.referenceArea / max(1, state.mass)
    // Feedforward: HREF's own descent while tracking it (range closes at
    // the ground speed), else the steady glide at the present drag.
    let tracksReference = referenceUsable && targetHeight == planned.height
    let feedforward = tracksReference ? -vh * planned.slope
      : -speed * clamp(dragAcceleration / g, 0, 1)
    let desiredRate = feedforward + (targetHeight - p.height) / (2 * tau)
    // HREF is height against ground distance. Its rate and the correction
    // are defined against horizontal ground speed, not a fixed total speed.
    // asin(rate/airspeed) exaggerates steep slopes and can saturate at -90°.
    let referenceGamma = tracksReference
      ? descent!.flightPath(at: t.range, height: p.height, horizontalSpeed: vh, responseSeconds: tau)
      : atan2(desiredRate, vh)
    // A rejected path must not latch the aircraft's existing steep descent
    // as the new goal. Recover toward the mission glide envelope; lift/q
    // protection below may report/saturate an unattainable recovery.
    let gamma = clamp(referenceUsable ? referenceGamma : state.flightPathAngle, -glide, glide)
    var vertical = (tracksReference ? normalReference : support) + speed * (gamma - state.flightPathAngle) / tau
    // Hard q bounds (QBMNNZ/QBMXNZ) for a state far off the target.
    // In still air qdot/q = V*gammaSin*dln(rho)/dh
    //                         - 2*(D/m + geff*gammaSin)/V.
    // The density term can dominate on Kerbin. Solve this local equation
    // over the identified pitch response instead of treating q as speed
    // alone in a constant-density atmosphere. Wind cases retain the
    // kinematic speed bound until a wind-gradient model is available.
    let densityGradient = atmosphere.logDensityAltitudeGradient(position: state.position, time: state.time)
    let effectiveGravity = g - c.planet.angularVelocity.cross(state.position).squaredLength / state.position.length
    func pressureBound(_ q: Double) -> Double {
      let floorSpeed = sqrt(2 * q / density)
      let speedOnly = -asin(clamp((floorSpeed - speed) / (g * tau), -1, 1))
      let coefficient = speed * densityGradient - 2 * effectiveGravity / speed
      guard state.atmosphere.wind.squaredLength == 0, a.dynamicPressure > 0,
        coefficient < 0 else { return speedOnly }
      let fractionalRate = log(q / a.dynamicPressure) / tau
      return asin(clamp((fractionalRate + 2 * dragAcceleration / speed) / coefficient, -1, 1))
    }
    // Preserve actual support/pressure protection, with the planned normal
    // demand and attainable CL above rather than a blanket level-flight q.
    // This can steepen an energy-starved descent beyond the nominal profile;
    // holding its glide angle while stalled would command maximum incidence.
    if a.dynamicPressure < frontQ {
      vertical = min(vertical,support+speed*(pressureBound(frontQ)-state.flightPathAngle)/tau)
    } else if a.dynamicPressure > v.maximumDynamicPressure {
      vertical = max(vertical, support + speed * (pressureBound(v.maximumDynamicPressure) - state.flightPathAngle) / tau)
    }

    // One lift vector: vertical first, lateral from what remains.
    let floorAlpha = min(ceiling, max(0, v.minimumAngleOfAttack))
    let maximumLift = min(v.maximumLoadFactor * 9.80665, aero.liftSolution(acceleration: 0, mass: state.mass,
      dynamicPressure: a.dynamicPressure, mach: a.mach, minimumAlpha: floorAlpha, maximumAlpha: ceiling,
      gear: state.systems.gearDown).maximumAcceleration * liftGain * (1 - v.aerodynamicUncertainty))
    let cannotHoldReference = referenceUsable && vertical > attainableNormal && p.height < targetHeight
      && a.dynamicPressure < frontQ
    // Recovery uses the same jointly attainable force cone as tracking.
    // Previously the best-L/D fallback tilted almost all efficient lift into
    // a 70-degree turn, silently sacrificing descent support. Preserve the
    // normal acceleration needed for the glide/pressure recovery and steer
    // with what remains. Maximum lift/load/trim limits remain authoritative.
    vertical = clamp(vertical, 0, maximumLift)
    // A nearly ballistic descent has little lift to tilt. Do not turn a
    // small lateral correction into a near-limit roll solely because the
    // planned vertical component is small; defer that correction instead.
    if referenceUsable, support > 0, vertical < support {lateral *= vertical/support}
    let lateralLimit = min(sqrt(max(0, maximumLift * maximumLift - vertical * vertical)),
      vertical * tan(v.maximumBank))
    lateral = clamp(lateral, -lateralLimit, lateralLimit)
    let total = hypot(vertical, lateral)
    let solution = aero.liftSolution(acceleration: total / max(1e-9, liftGain), mass: state.mass,
      dynamicPressure: a.dynamicPressure, mach: a.mach, minimumAlpha: floorAlpha, maximumAlpha: ceiling,
      gear: state.systems.gearDown)
    // If even maximum lift cannot hold a reference above the aircraft, more
    // incidence only spends range. Best L/D is the attainable recovery;
    // retain q/load protections and defer the turn until support returns.
    let alpha = cannotHoldReference ? bestAlpha : solution.alpha
    let bank = cannotHoldReference ? 0 : (total > 0 ? atan2(lateral, vertical) : 0)
    let gear = state.systems.gearDown || p.height <= (c.guidance.gearDeploymentAltitude ?? 1500)
    return .init(angleOfAttack: alpha, bank: bank,
      desiredSpeed: sqrt(2 * commandedQ / density),
      desiredHeight: targetHeight + c.runway.elevation, desiredFlightPath: gamma,
      gearDown: gear, speedbrakeFraction: nil)
  }

  /// C1 interpolation of a phase transfer, over the physical response time
  /// or distance supplied by the caller. No additional flight tuning gain.
  static func smoothTransition(_ fraction: Double) -> Double {
    let u = clamp(fraction, 0, 1)
    return u * u * (3 - 2 * u)
  }

  /// Acquisition to a point at `distance` and relative `bearing`: the
  /// geometry-bank turn then straight (Dubins CS), or, for a point inside
  /// that turn circle (nearly ahead at high speed), the single gentler arc
  /// through it, which is what the bank law actually flies. Returns the
  /// path length, its turning arc and that arc's load factor.
  static func acquisitionLeg(distance d: Double, bearing: Double, radius r: Double, speed: Double,
                             gravity g: Double) -> (length: Double, arc: Double, load: Double) {
    let beta = abs(wrapAngle(bearing))
    let tx = d * cos(beta), ty = d * sin(beta)
    if r > 0, hypot(tx, ty - r) <= r, beta > 1e-9, beta < .pi / 2 {
      let radius = d / (2 * sin(beta)), arc = 2 * beta * radius
      return (arc, arc, hypot(1, speed * speed / (g * radius)))
    }
    let leg = turnThenStraight(distance: d, bearing: bearing, radius: r)
    return (leg.length, abs(leg.turn) * r, hypot(1, speed * speed / (g * max(1, r))))
  }

  /// Turn at `radius` then straight to a point at `distance` and relative
  /// bearing (Dubins CS); a point inside the turn circle costs a half turn.
  static func turnThenStraight(distance d: Double, bearing: Double, radius r: Double)
    -> (length: Double, turn: Double) {
    let beta = abs(wrapAngle(bearing)), sign: Double = bearing < 0 ? -1 : 1
    guard r > 0, r.isFinite else { return (d, 0) }
    let tx = d * cos(beta), ty = d * sin(beta)
    let q = hypot(tx, ty - r)
    guard q > r else { return (d + .pi * r, sign * .pi) }
    let theta = atan2(ty - r, tx)
    let tangentAngle = theta - acos(r / q)
    var arc = (tangentAngle + .pi / 2).truncatingRemainder(dividingBy: 2 * .pi)
    if arc < 0 { arc += 2 * .pi }
    return (r * arc + sqrt(q * q - r * r), sign * arc)
  }
}


final class ZZRunway36RangeInvestigationTests: XCTestCase {
 struct Sample: Decodable {var state:VehicleState;var control:ControlDiagnostics;var objective:EntryCaptureObjective}
 func testRecordedRangeChoice() throws {
  let base=URL(fileURLWithPath:"/Users/kyooni18/Code/KSL/Validation/Runway36-Restart-Investigation-20261102")
  let decoder=JSONDecoder()
  let c=try decoder.decode(MissionConfiguration.self,from:Data(contentsOf:base.appendingPathComponent("configuration.json")))
  let s=try decoder.decode(Sample.self,from:Data(contentsOf:base.appendingPathComponent("range-sample.json")))
  let atm=try AtmosphericModelEngine(planet:c.planet), aero=try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea)
  let ceiling=RotationalDynamicsEngine(configuration:c.vehicle,planet:c.planet).supportedTrimAlpha(s.state,identifiedCeiling:s.control.trimEvidence == .identified ? s.control.trimAlphaCeiling:nil)
  print("RANGE_SELECTED",InvestigationRangeReference.solve(from:s.state,objective:s.objective,configuration:c,atmosphere:atm,aero:aero,ceiling:ceiling,pitchResponse:s.control.pitchResponseSeconds ?? c.guidance.pathLookaheadTime,rollResponse:s.control.rollResponseSeconds ?? c.guidance.pathLookaheadTime))
 }
}
import Foundation
import KSLCore
import KSLPhysics

/// Point-mass shooting reference, not a closed-loop reachability certificate.
/// Solve where interface energy is reached instead of fitting instantaneous
/// drag to an assumed altitude-versus-speed path. Each trial uses the actual
/// rotating planet, atmosphere and polar. Alpha spans best L/D through the
/// configured attainable ceiling; the scalar is solved, never scenario-tuned.
struct InvestigationRangeReference: Sendable {
  var createdAt: Double
  var blend: Double
  var cross: Double
  var along: Double
  var altitude: Double
  var speed: Double
  var bracketed: Bool
  var reachedEnergy: Bool

  static func alpha(blend: Double, mach: Double, q: Double, ceiling: Double,
                    aero: AerodynamicsEngine) -> Double {
    let best = aero.bestGlideAlpha(mach: mach, maximumAlpha: ceiling, dynamicPressure: q)
    return best + blend * (ceiling-best)
  }

  /// Only measured/modelled surplus lift is spent on crossrange correction.
  static func bank(state s: VehicleState, objective: EntryCaptureObjective,
                   configuration c: MissionConfiguration, frame: RunwayFrame,
                   lift: Double) -> Double {
    let p = frame.coordinates(of: s.position)
    let target = frame.position(along: objective.along, cross: objective.cross, height: 0)
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let horizontal = s.velocity.projected(onPlaneWithNormal: s.position.normalized)
    let alignment = horizontal.normalized.dot(forward)
    let lateral = EntryLateralController.demand(state: s, configuration: c,
      azimuthError: wrapAngle(frame.bearing(from: s.position, to: target)-s.course),
      crossError: objective.cross-p.cross,
      crossRate: -frame.coordinateVelocity(at:s.position,velocity:s.velocity).y,
      alignment: alignment, range: abs(objective.along-p.along))
    let inertial = c.planet.inertialVelocity(position:s.position,fixedVelocity:s.velocity)
    let support = max(0, (c.planet.gravitationalParameter/s.position.squaredLength
      - inertial.projected(onPlaneWithNormal:s.position.normalized).squaredLength/s.position.length)
      * cos(s.flightPathAngle))
    let authority = NormalForceEnvelope(maximumLift:lift,maximumBank:c.vehicle.maximumBank)
      .lateralAuthority(reserving:support)
    return lift > 0 ? asin(clamp(lateral,-authority,authority)/lift) : 0
  }

  static func solve(from initial: VehicleState, objective: EntryCaptureObjective,
                    configuration c: MissionConfiguration, atmosphere: AtmosphericModelEngine,
                    aero: AerodynamicsEngine, ceiling: Double,
                    pitchResponse: Double, rollResponse: Double) -> Self {
    let frame = RunwayFrame(runway:c.runway,planet:c.planet)
    let translation = TranslationalDynamicsEngine(planet:c.planet,atmosphere:atmosphere,aerodynamics:aero)
    let endpoint = objective.specificEnergy(configuration:c)
    func energy(_ s: VehicleState) -> Double {
      s.velocity.squaredLength/2-c.planet.gravitationalParameter/s.position.length
        - c.planet.angularVelocity.cross(s.position).squaredLength/2
    }
    func trial(_ blend: Double) -> Self {
      var s=initial, alpha=s.aerodynamics.angleOfAttack, bank=s.aerodynamics.bank
      let end=initial.time+c.guidance.maximumPredictionDuration
      var reached=false
      while s.time < end, s.position.length > c.planet.radius+c.runway.elevation {
        s.atmosphere=atmosphere.sample(position:s.position,time:s.time)
        s.attitude = .windAttitude(position:s.position,airVelocity:s.velocity-s.atmosphere.wind,alpha:alpha,bank:bank)
        s.aerodynamics=aero.evaluate(position:s.position,velocity:s.velocity,attitude:s.attitude,
          atmosphere:s.atmosphere,gear:s.systems.gearDown)
        if energy(s)<=endpoint {reached=true;break}
        if s.aerodynamics.dynamicPressure>c.vehicle.maximumDynamicPressure
          || s.aerodynamics.lift/s.mass>c.vehicle.maximumLoadFactor*9.80665 {break}
        // Same configured propagation resolution as other entry references.
        let dt=min(c.guidance.predictionStep,end-s.time)
        let targetAlpha=Self.alpha(blend:blend,mach:s.aerodynamics.mach,
          q:s.aerodynamics.dynamicPressure,ceiling:ceiling,aero:aero)
        alpha=moveToward(alpha,alpha+(targetAlpha-alpha)*(-expm1(-dt/pitchResponse)),
          maximumDelta:c.guidance.alphaSlewRate*dt)
        let targetBank=Self.bank(state:s,objective:objective,configuration:c,frame:frame,
          lift:max(0,s.aerodynamics.lift/s.mass)*(1-c.vehicle.aerodynamicUncertainty))
        bank=moveToward(bank,bank+(targetBank-bank)*(-expm1(-dt/rollResponse)),
          maximumDelta:min(c.entrySettings.rollRate,c.vehicle.controlPlant.maximumRollRate)*dt)
        let attitude=Quaternion.windAttitude(position:s.position,airVelocity:s.velocity-s.atmosphere.wind,alpha:alpha,bank:bank)
        let before=s, beforeEnergy=energy(s)
        let next=translation.step(.init(position:s.position,velocity:s.velocity),time:s.time,
          mass:s.mass,attitude:attitude,dt:dt,systems:s.systems)
        s.position=next.position;s.velocity=next.velocity;s.time+=dt
        let afterEnergy=energy(s)
        if afterEnergy<=endpoint, beforeEnergy>endpoint {
          let fraction=(beforeEnergy-endpoint)/(beforeEnergy-afterEnergy)
          s.position=before.position+(s.position-before.position)*fraction
          s.velocity=before.velocity+(s.velocity-before.velocity)*fraction
          s.time=before.time+dt*fraction;reached=true;break
        }
      }
      return .init(createdAt:initial.time,blend:blend,cross:frame.coordinates(of:s.position).cross,along:frame.coordinates(of:s.position).along,
        altitude:s.position.length-c.planet.radius,speed:s.velocity.length,
        bracketed:false,reachedEnergy:reached)
    }
    var low=trial(0),high=trial(1)
    print("RANGE_LOW",low,"RANGE_HIGH",high,"OBJECTIVE",objective)
    func residual(_ v: Self) -> Double {v.along-objective.along}
    var best=abs(residual(low))<abs(residual(high)) ? low:high
    if low.reachedEnergy != high.reachedEnergy {best=low.reachedEnergy ? low:high}
    guard low.reachedEnergy, high.reachedEnergy, residual(low)*residual(high)<=0 else {return best}
    // Bisection stops on configured arrival tolerance or floating resolution.
    // Bracket signs, not an assumed monotonic direction, determine each step.
    for _ in 0...Double.significandBitCount {
      let blend=(low.blend+high.blend)/2
      if blend==low.blend || blend==high.blend {break}
      var mid=trial(blend)
      guard mid.reachedEnergy else {break}
      mid.bracketed=true
      if abs(residual(mid))<abs(residual(best)) {best=mid}
      if abs(residual(mid))<=c.entrySettings.handoffRadius {return mid}
      if residual(low)*residual(mid)<=0 {high=mid} else {low=mid}
    }
    return best
  }
}
