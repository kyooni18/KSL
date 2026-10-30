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
  private var cached: [Double: EntryCaptureObjective] = [:]
  init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration; self.atmosphere = atmosphere; aero = aerodynamics
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  func objective(from initial: VehicleState, crossAim: Double) -> EntryCaptureObjective {
    if let value = cached[crossAim] { return value }
    let c = configuration, e = c.entrySettings
    let glide = c.guidance.taemGlideAngle ?? c.guidance.outerGlideAngle
    let evaluator = TAEMCaptureEvaluator(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    var best: (EntryCaptureObjective, Double)?
    for altitude in [e.terminalTargetAltitude, e.terminalMinimumAltitude, e.terminalMaximumAltitude] {
      let height = altitude - c.runway.elevation
      let range = e.captureRange
      let cross = e.handoffCross + crossAim
      guard abs(cross) < range else { continue }
      let along = -sqrt(range*range-cross*cross)
      let position = frame.position(along: along, cross: cross, height: height)
      let toward = frame.origin.normalized.projected(onPlaneWithNormal: position.normalized).normalized
      let sound = atmosphere.sample(position: position, time: initial.time).speedOfSound
      for mach in [e.terminalTargetMach, e.terminalTargetMach-e.terminalMachHalfWidth] where !Task.isCancelled {
        let speed = mach*sound
        let velocity = (toward*cos(glide)-position.normalized*sin(glide))*speed
        let sample = physics.observe(.init(time: initial.time, position: position, velocity: velocity,
          attitude: .windAttitude(position: position, airVelocity: velocity, alpha: e.nominalAlpha, bank: 0), mass: initial.mass))
        let capture = evaluator.evaluate(sample)
        let envelope = capture.envelope
        guard capture.feasible, capture.hacRadius != nil, envelope.energyMargin > 0 else { continue }
        let g = c.planet.gravitationalParameter / position.squaredLength
        let energy = envelope.energyHeight + (envelope.excessEnergyMargin-envelope.deficitEnergyMargin)/2
        let centeredSpeed = sqrt(max(0, 2*g*(energy-height)))
        let objective = EntryCaptureObjective(along: along, cross: cross, altitude: altitude,
          speed: centeredSpeed, energyHeight: energy, flightPath: -glide)
        let cost = range/e.terminalTargetRange
          + abs(altitude-e.terminalTargetAltitude)/(e.terminalMaximumAltitude-e.terminalMinimumAltitude)
          - envelope.energyMargin/max(1, envelope.energyHeight)
        if best == nil || cost < best!.1 { best = (objective, cost) }
      }
    }
    let result = best?.0 ?? .seed(configuration: c, atmosphere: atmosphere, frame: frame, crossAim: crossAim, time: initial.time)
    cached[crossAim] = result
    return result
  }
}
