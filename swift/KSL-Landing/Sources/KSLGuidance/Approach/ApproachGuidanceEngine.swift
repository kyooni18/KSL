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
  private let flare: TerminalFlareController
  private let outer: TerminalEnergyGuide
  private let externalPlanning: Bool
  private var capture: ApproachCapture?
  public private(set) var lastCaptureAssessment: TerminalAssessment?
  public init(
    configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
    aerodynamics: AerodynamicsEngine, externalPlanning: Bool = false
  ) {
    self.configuration = configuration
    self.externalPlanning = externalPlanning
    profile = .init(configuration: configuration)
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    self.aerodynamics = aerodynamics
    flare = .init(configuration: configuration, aerodynamics: aerodynamics)
    outer = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
    rotation = .init(configuration: configuration.vehicle)
    energy = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics)
  }
  /// Preserve captured geometry and flare latches; only the old prediction
  /// and energy-recovery evidence belong to the previous payload state.
  public func invalidatePrediction() { outer.invalidatePrediction() }
  public func planningRequest() -> ApproachPlanningRequest? { outer.request() }
  public func accept(_ proposal: ApproachPlanningProposal) { outer.accept(proposal) }
  /// Unqualified last-chance takeover is altitude-based. Merely crossing
  /// the NEP's horizontal station at high altitude is not Final capture.
  public func recoveryDue(_ state: VehicleState) -> Bool {
    let nepHeight = profile.reference(along: -configuration.guidance.finalDistance).height
    return frame.coordinates(of: state.position).height <= nepHeight && state.verticalSpeed < 0
  }
  public func canCapture(_ state: VehicleState) -> Bool {
    let p = frame.coordinates(of: state.position)
    let reference = profile.reference(along: p.along)
    let direction = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let error = abs(wrapAngle(state.localFrame.heading(of: direction) - state.course))
    // NEP is a physical handover station, not an open-ended region after it.
    // Permit at most one measured guidance step on either side of the station;
    // the independent Final energy price still decides capture feasibility.
    let nepAlong = -configuration.guidance.finalDistance
    let nepSampleDistance = max(1, state.horizontalSpeed * configuration.guidance.controlInterval)
    guard p.along >= nepAlong - nepSampleDistance
      && p.along <= nepAlong + nepSampleDistance
      && p.along < configuration.guidance.touchdownAlong,
      abs(p.cross) < max(50, (-p.along) * 0.03), error < radians(12),
      abs(state.aerodynamics.bank) < radians(25),
      p.height > configuration.vehicle.gearHeight,
      abs(p.height - reference.height) < max(220, reference.height * 0.2),
      state.aerodynamics.airspeed >= (configuration.vehicle.minimumSafeSpeed ?? configuration.vehicle.landingSpeed),
      state.aerodynamics.airspeed <= (configuration.guidance.finalAlignmentSpeed ?? 260)
    else { return false }
    let candidate = ApproachCapture(
      profile: profile, along: p.along, height: p.height, slope: tan(state.flightPathAngle))
    let (_, assessment) = energy.evaluate(
      stations: candidate.stations(touchdownAlong: configuration.guidance.touchdownAlong),
      from: state)
    lastCaptureAssessment = assessment
    if assessment.feasible {
      capture = candidate
      outer.captureNominalEnergySpeed(state.aerodynamics.airspeed)
      return true
    }
    return false
  }

  public func update(
    _ state: VehicleState, previousPhase: MissionPhase, liftEffectiveness: Double = 1, dragEffectiveness: Double = 1,
    pitchResponseSeconds: Double? = nil, identifiedTrimCeiling: Double? = nil
  ) -> GuidanceSolution {
    let p = frame.coordinates(of: state.position)
    let c = configuration.guidance
    let v = configuration.vehicle
    if capture == nil {
      capture = .init(
        profile: profile, along: p.along, height: p.height, slope: tan(state.flightPathAngle))
      outer.captureNominalEnergySpeed(state.aerodynamics.airspeed)
    }
    let ref = capture!.reference(along: p.along)
    let speed = max(1, state.aerodynamics.airspeed)
    let vh = max(20, state.horizontalSpeed)
    let response = pitchResponseSeconds ?? (2 * 1.15 / (1.05 * 1.10) + 0.12)
    var maxAlpha = min(v.maximumLandingAngleOfAttack,
      rotation.supportedTrimAlpha(state, identifiedCeiling: identifiedTrimCeiling))
    let flareReference = flare.update(state, liftEffectiveness: liftEffectiveness,
      responseSeconds: response, maximumAlpha: maxAlpha)
    let phase: MissionPhase = flareReference.latched
      ? (flareReference.wheelHeight <= 12 ? .flare : .preflare) : .approach
    let forward = frame.direction(at: p.horizontal, tangent: .init(1, 0))
    let headingError = wrapAngle(state.localFrame.heading(of: forward) - state.course)
    let lateral = 2 * 0.9 / 5 * vh * sin(headingError) - p.cross / 25
    let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
    let desiredVertical: Double
    let vertical: Double
    let energyReference = flareReference.arcActive ? nil : outer.update(state, flare: flareReference,
      maximumAlpha: maxAlpha, lift: liftEffectiveness, drag: dragEffectiveness, synchronousPlanning: !externalPlanning)
    let desiredSpeed = energyReference?.desiredSpeed ?? v.landingSpeed
    if flareReference.arcActive {
      desiredVertical = -flareReference.desiredSink
      // Convert vertical acceleration to wind-normal lift; at a steep glide
      // vertical and normal acceleration cannot be used interchangeably.
      vertical = (g - vh*vh/state.position.length + flareReference.upwardAcceleration
        + state.aerodynamics.drag/state.mass*sin(state.flightPathAngle))
        / max(0.3, cos(state.flightPathAngle))
    } else {
      let gamma = energyReference?.flightPath ?? atan(ref.slope)
      desiredVertical = speed*sin(gamma)
      // Lead the vertical-speed feedback through the measured pitch response
      // delay. With slow attitude response, reacting to the present sink alone
      // commands another pull after existing lift has already begun to arrest
      // it, causing a dive/pull oscillation and excess drag work.
      let verticalAcceleration = state.aerodynamics.lift/state.mass*cos(state.aerodynamics.bank)
        * cos(state.flightPathAngle) - state.aerodynamics.drag/state.mass*sin(state.flightPathAngle)
        - g + vh*vh/state.position.length
      let predictedVertical = state.verticalSpeed + response*verticalAcceleration
      vertical = (g-vh*vh/state.position.length + (desiredVertical-predictedVertical)/(3+response)
        + state.aerodynamics.drag/state.mass*sin(state.flightPathAngle))
        / max(0.3, cos(state.flightPathAngle))
    }
    let bankLimit = p.height < 150 ? radians(8) : radians(25)
    let bank = clamp(atan2(lateral, max(2, vertical)), -bankLimit, bankLimit)
    let required = clamp(vertical / max(0.5, cos(bank)), 0, 9.80665 * v.maximumLoadFactor)
    if phase == .flare {
      // AoA is not touchdown pitch. Reserve the observed pitch-rate travel
      // during actuator lag before the structural touchdown attitude limit.
      let pitchReserve = max(0, state.angularVelocity.y) * 0.12
      maxAlpha = min(maxAlpha, v.maximumTouchdownPitch - state.flightPathAngle - pitchReserve)
    }
    let lift = aerodynamics.liftSolution(
      acceleration: required / max(0.5, liftEffectiveness), mass: state.mass,
      dynamicPressure: state.aerodynamics.dynamicPressure,
      mach: state.aerodynamics.mach, minimumAlpha: v.minimumAngleOfAttack, maximumAlpha: maxAlpha,
      beta: state.aerodynamics.sideslip, gear: state.systems.gearDown)
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
    var diagnostics = ApproachDiagnostics()
    diagnostics.wheelHeight = flareReference.wheelHeight; diagnostics.along = p.along
    diagnostics.preflareLatched = flareReference.latched; diagnostics.arcActive = flareReference.arcActive
    diagnostics.triggerHeight = flareReference.triggerHeight; diagnostics.desiredSink = flareReference.desiredSink
    diagnostics.pullAcceleration = flareReference.upwardAcceleration; diagnostics.availablePull = flareReference.availablePull
    diagnostics.oneGravitySpeed = flareReference.oneGravitySpeed.isFinite ? flareReference.oneGravitySpeed : nil
    diagnostics.predictedTouchdownAlong = energyReference?.predictedTouchdownAlong
    diagnostics.predictedTouchdownSpeed = energyReference?.predictedTouchdownSpeed
    diagnostics.requiredNormalAcceleration = required
    diagnostics.liftEffectiveness = liftEffectiveness; diagnostics.dragEffectiveness = dragEffectiveness
    var result = GuidanceSolution(
      phase: phase,
      target: .init(
        angleOfAttack: lift.alpha, bank: bank, desiredSpeed: flareReference.arcActive ? v.landingSpeed : desiredSpeed,
        desiredHeight: ref.height,
        desiredFlightPath: atan2(desiredVertical, vh),
        gearDown: state.systems.gearDown || flareReference.latched
          || p.height <= (c.gearDeploymentAltitude ?? 1500)), violations: violations,
      pathRemaining: max(0, c.touchdownAlong - p.along))
    result.approach = diagnostics
    return result
  }
}
