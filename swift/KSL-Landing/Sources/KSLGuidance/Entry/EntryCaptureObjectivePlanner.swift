import Foundation
import KSLCore
import KSLPhysics

/// Prices high-altitude capture states with MM305's HAC family before MM304
/// spends energy flying toward one. The outer search radius is never the aim.
final class EntryCaptureObjectivePlanner {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let aero: AerodynamicsEngine
  private let frame: RunwayFrame
  // HAC pricing depends on the seed's mass and atmosphere epoch.
  // Cross-aim alone cannot identify an objective across distinct windows or
  // payload changes. Keep only the current seed's candidates to bound memory.
  private var cachedTime: Double?
  private var cachedMass: Double?
  private var cached: [Double: EntryCaptureObjective] = [:]
  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  func objective(from initial: VehicleState, crossAim: Double) -> EntryCaptureObjective {
    if cachedTime != initial.time || cachedMass != initial.mass {
      cached.removeAll(keepingCapacity: true)
      cachedTime = initial.time; cachedMass = initial.mass
    }
    if let value = cached[crossAim] { return value }
    let c = configuration, e = c.entrySettings
    let glide = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
    let evaluator = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    // MM304 delivers to the NEP OVERHEAD capture region: the NEP station on
    // the selected centerline plus a compact HAC beside it (terminal capture
    // range). The previous objective added a straight-in descent of the whole
    // capture altitude on the TAEM glide slope, (h - h_NEP)/tan(gamma_TAEM),
    // which at 25 km ASL moved the inlet ~56 km up-range and made MM304 hand
    // a distant straight-in waypoint to MM305. Altitude above the NEP is
    // dissipated on the HAC overhead; the HAC search still prices it.
    func preferredRange(_ altitude: Double) -> Double { c.nepOverheadCaptureRange }
    var best: (EntryCaptureObjective, Double)?
    for altitude in [e.terminalTargetAltitude, e.terminalMinimumAltitude, e.terminalMaximumAltitude] {
      let height = altitude-c.runway.elevation
      let ideal = preferredRange(altitude)
      let distances = [ideal]
      let cross = e.handoffCross+crossAim
      for range in distances where !Task.isCancelled {
        guard abs(cross) < range else { continue }
        let along = -sqrt(max(0, range*range-cross*cross))
        let position = frame.position(along: along, cross: cross, height: height)
        let toward = frame.origin.normalized.projected(onPlaneWithNormal: position.normalized).normalized
        let sound = atmosphere.sample(position: position, time: initial.time).speedOfSound
        for mach in [e.terminalTargetMach, e.terminalTargetMach-e.terminalMachHalfWidth] where !Task.isCancelled {
          let speed = mach*sound
          let velocity = (toward*cos(glide)-position.normalized*sin(glide))*speed
          let sample = physics.observe(.init(time: initial.time, position: position, velocity: velocity,
            attitude: .windAttitude(position: position, airVelocity: velocity,
              alpha: e.nominalAlpha, bank: 0), mass: initial.mass))
          let capture = evaluator.evaluate(sample)
          let envelope = capture.envelope
          // Price a COMPLETE HAC/Final, never a distance-only waypoint.
          guard capture.feasible, capture.hacRadius != nil,
                envelope.energyMargin.isFinite, envelope.energyMargin > 0 else { continue }
          let g = c.planet.gravitationalParameter/position.squaredLength
          let energy = envelope.energyHeight
            + (envelope.excessEnergyMargin-envelope.deficitEnergyMargin)/2
          let centeredSpeed = clamp(sqrt(max(0, 2*g*(energy-height))),
            (e.terminalTargetMach-e.terminalMachHalfWidth)*sound,
            (e.terminalTargetMach+e.terminalMachHalfWidth)*sound)
          let objective = EntryCaptureObjective(along: along, cross: cross,
            altitude: altitude, speed: centeredSpeed,
            energyHeight: height+centeredSpeed*centeredSpeed/(2*g), flightPath: -glide)
          // Prefer the altitude-consistent inlet, not the old shortest
          // centerline intercept. An actual HAC can overrule the ideal range.
          let cost = abs(range-ideal)/e.terminalTargetRange
            + abs(altitude-e.terminalTargetAltitude)
              / (e.terminalMaximumAltitude-e.terminalMinimumAltitude)
            + (capture.planLength ?? range)/c.guidance.maximumPathLength
            - envelope.energyMargin/max(1, envelope.energyHeight)
          if best == nil || cost < best!.1 { best = (objective, cost) }
        }
      }
    }
    // An unsuccessful preview is not a landing certificate. Steer toward the
    // same NEP overhead region while the live planner searches for a route.
    let fallbackRange = preferredRange(e.terminalTargetAltitude)
    let fallbackCross = clamp(e.handoffCross+crossAim,
      -fallbackRange+1, fallbackRange-1)
    let fallbackAlong = -sqrt(max(0, fallbackRange*fallbackRange-fallbackCross*fallbackCross))
    let seeded = EntryCaptureObjective.seed(configuration: c, atmosphere: atmosphere,
      frame: frame, crossAim: crossAim, time: initial.time)
    let fallback = EntryCaptureObjective(along: fallbackAlong, cross: fallbackCross,
      altitude: seeded.altitude, speed: seeded.speed,
      energyHeight: seeded.energyHeight, flightPath: seeded.flightPath)
    let result = best?.0 ?? fallback
    cached[crossAim] = result
    return result
  }
}
