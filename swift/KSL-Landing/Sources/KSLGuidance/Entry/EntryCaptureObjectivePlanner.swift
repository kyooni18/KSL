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
    // MM304 delivers to the configured TAEM interface (Shuttle: Mach 2.5 at
    // its range-to-go). TAEM's own ground track dissipates what remains.
    func preferredRange(_ altitude: Double) -> Double { c.taemInterfaceRange }
    var best: (EntryCaptureObjective, Double)?
    for altitude in [c.mm304DeliveryAltitude] {
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
        for mach in [c.mm304DeliveryMach, max(0, c.mm304DeliveryMach-e.terminalMachHalfWidth)] where !Task.isCancelled {
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
            max(0, c.mm304DeliveryMach-e.terminalMachHalfWidth)*sound,
            (c.mm304DeliveryMach+e.terminalMachHalfWidth)*sound)
          let objective = EntryCaptureObjective(along: along, cross: cross,
            altitude: altitude, speed: centeredSpeed,
            energyHeight: height+centeredSpeed*centeredSpeed/(2*g), flightPath: -glide)
          // Prefer the altitude-consistent inlet, not the old shortest
          // centerline intercept. An actual HAC can overrule the ideal range.
          let cost = abs(range-ideal)/e.terminalTargetRange
            + abs(altitude-c.mm304DeliveryAltitude)
              / (e.terminalMaximumAltitude-c.mm304DeliveryMinimumAltitude)
            + (capture.planLength ?? range)/c.guidance.maximumPathLength
            - envelope.energyMargin/max(1, envelope.energyHeight)
          if best == nil || cost < best!.1 { best = (objective, cost) }
        }
      }
    }
    // An unsuccessful preview is not a landing certificate. Steer toward the
    // same TAEM interface; TAEM does not need a preview to take over.
    let fallbackRange = preferredRange(c.mm304DeliveryAltitude)
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
