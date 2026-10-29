import Foundation
import KSLCore
import KSLPhysics

public final class ApproachGuidanceEngine {
  private let configuration: MissionConfiguration
  private let profile: ApproachProfile
  private let frame: RunwayFrame
  private let aerodynamics: AerodynamicsEngine
  private let rotation: RotationalDynamicsEngine
  private let energy: EnergyManagementEngine
  private var capture: ApproachCapture?
  public private(set) var lastCaptureAssessment: TerminalAssessment?
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine
  ) {
    self.configuration = configuration
    profile = .init(configuration: configuration)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    self.aerodynamics = aerodynamics
    rotation = .init(configuration: configuration.vehicle)
    energy = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  public func canCapture(_ state: VehicleState) -> Bool {
    let p = frame.coordinates(of: state.position)
    let reference = profile.reference(along: p.along)
    let direction = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let error = abs(wrapAngle(state.localFrame.heading(of: direction) - state.course))
    guard p.along < configuration.guidance.touchdownAlong && p.along > -18_000,
      abs(p.cross) < max(50, (-p.along) * 0.03), error < radians(12),
      abs(state.aerodynamics.bank) < radians(25),
      p.height > configuration.vehicle.gearHeight,
      abs(p.height - reference.height) < max(220, reference.height * 0.2),
      state.aerodynamics.airspeed < 260
    else { return false }
    let candidate = ApproachCapture(
      profile: profile, along: p.along, height: p.height, slope: tan(state.flightPathAngle))
    let (_, assessment) = energy.evaluate(
      stations: candidate.stations(touchdownAlong: configuration.guidance.touchdownAlong),
      from: state)
    lastCaptureAssessment = assessment
    if assessment.feasible {
      capture = candidate
      return true
    }
    return false
  }

  public func update(
    _ state: VehicleState, previousPhase: MissionPhase, liftEffectiveness: Double = 1
  ) -> GuidanceSolution {
    let p = frame.coordinates(of: state.position)
    let c = configuration.guidance
    let v = configuration.vehicle
    if capture == nil {
      capture = .init(
        profile: profile, along: p.along, height: p.height, slope: tan(state.flightPathAngle))
    }
    let ref = capture!.reference(along: p.along)
    let speed = max(1, state.aerodynamics.airspeed)
    let vh = max(20, state.horizontalSpeed)
    var phase: MissionPhase = p.along >= profile.preflareAlong - 100 ? .preflare : .approach
    if p.height <= c.flareHeight && p.along > profile.preflareAlong || previousPhase == .flare {
      phase = .flare
    }
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let headingError = wrapAngle(state.localFrame.heading(of: forward) - state.course)
    let lateral = 2 * 0.9 / 5 * vh * sin(headingError) - p.cross / 25
    let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
    var desiredVertical: Double
    var verticalFeedforward = 0.0
    if phase == .flare {
      desiredVertical = -clamp(0.11 * max(0, p.height - v.gearHeight) + 1, 1, 3)
    } else {
      let speedCorrection = clamp((speed - profile.speed(along: p.along)) * 0.0008, -0.035, 0.035)
      desiredVertical =
        speed * sin(atan(ref.slope) + speedCorrection) + clamp((ref.height - p.height) / 7, -30, 30)
      let preview = capture!.reference(along: p.along + vh * 1.0)
      verticalFeedforward =
        speed * speed * preview.secondDerivative / pow(1 + preview.slope * preview.slope, 1.5)
    }
    let tau = phase == .flare ? 1.4 : 3.0
    let vertical =
      (g - vh * vh / state.position.length) * cos(state.flightPathAngle)
      + (desiredVertical - state.verticalSpeed) / tau + verticalFeedforward
    let bankLimit = p.height < 150 ? radians(8) : radians(25)
    let bank = clamp(atan2(lateral, max(2, vertical)), -bankLimit, bankLimit)
    let required = clamp(vertical / max(0.5, cos(bank)), 0, 9.80665 * v.maximumLoadFactor)
    let maxAlpha = min(
      v.maximumLandingAngleOfAttack,
      rotation.maximumTrimAlpha(dynamicPressure: state.aerodynamics.dynamicPressure))
    let lift = aerodynamics.liftSolution(
      acceleration: required / max(0.5, liftEffectiveness), mass: state.mass,
      dynamicPressure: state.aerodynamics.dynamicPressure,
      mach: state.aerodynamics.mach, minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: maxAlpha)
    var violations: [ConstraintViolation] = []
    if !lift.achievable {
      violations.append(
        .init(
          .insufficientLift, "Approach normal-acceleration demand saturated", value: required,
          limit: lift.maximumAcceleration * liftEffectiveness))
    }
    if p.along > configuration.runway.length - 200 && !state.contact.isGrounded {
      violations.append(.init(.runwayExcursion, "No usable runway remains ahead"))
    }
    return .init(
      phase: phase,
      target: .init(
        angleOfAttack: lift.alpha, bank: bank, desiredSpeed: profile.speed(along: p.along),
        desiredHeight: ref.height,
        desiredFlightPath: atan2(desiredVertical, vh), gearDown: true), violations: violations,
      pathRemaining: max(0, c.touchdownAlong - p.along))
  }
}
