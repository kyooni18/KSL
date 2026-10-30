import Foundation

/// Command slew is part of the flight computer, so every forecast starts from
/// the same last commanded target and uses this exact limiter.
public struct GuidanceTargetLimiter: Sendable {
  public private(set) var previous: GuidanceTarget?
  public init(previous: GuidanceTarget? = nil) { self.previous = previous }
  public mutating func update(_ requested: GuidanceTarget, phase: MissionPhase,
                              configuration c: MissionConfiguration, dt: Double) -> GuidanceTarget {
    var target = requested
    if let old = previous {
      let bankRate = [.approach, .preflare, .flare].contains(phase)
        ? (c.guidance.approachRollRate ?? c.guidance.bankSlewRate)
        : [.acquisition, .headingAlignment].contains(phase)
        ? (c.guidance.terminalRollRate ?? c.guidance.bankSlewRate) : c.entrySettings.rollRate
      target.bank = old.bank + clamp(wrapAngle(target.bank - old.bank),
        -bankRate * dt, bankRate * dt)
      let alphaRate = phase == .preflare || phase == .flare
        ? c.vehicle.controlPlant.maximumPitchRate * 0.8 : c.guidance.alphaSlewRate
      target.angleOfAttack = moveToward(old.angleOfAttack, target.angleOfAttack,
        maximumDelta: alphaRate * dt)
    }
    previous = target
    return target
  }
}
