import Foundation
import KSLCore
import KSLPhysics

public final class PathTrackingEngine {
  private let configuration: MissionConfiguration
  private let frame: RunwayFrame
  private let aero: AerodynamicsEngine
  private let rotation: RotationalDynamicsEngine
  public private(set) var index = 0
  public private(set) var trackingError = 0.0
  public private(set) var reference: PathStation?
  public private(set) var headingError = 0.0
  public init(configuration: MissionConfiguration, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    aero = aerodynamics
    rotation = .init(configuration: configuration.vehicle)
  }
  public func reset() {
    index = 0
    trackingError = 0; reference = nil; headingError = 0
  }
  public func update(_ state: VehicleState, plan: TerminalPlan, liftEffectiveness: Double = 1,
                     identifiedTrimCeiling: Double? = nil, rollResponseSeconds: Double? = nil,
                     pitchResponseSeconds: Double? = nil) -> (
    GuidanceTarget, PathStation
  ) {
    let p = frame.coordinates(of: state.position)
    index = max(index, plan.nearest(to: p.horizontal, startingAt: index))
    let nearest = plan.stations[index]
    let alongOffset = (p.horizontal - nearest.position).dot(nearest.tangent)
    let ref = plan.station(at: clamp(nearest.distance + alongOffset, 0, plan.length))
    reference = ref
    let offset = p.horizontal - ref.position
    trackingError = offset.length
    let tangent = frame.direction(at: ref.position, tangent: ref.tangent)
    let course = state.localFrame.heading(of: tangent)
    let headingError = wrapAngle(course - state.course)
    self.headingError = headingError
    let crossError = -offset.dot(ref.tangent.rightNormal)
    let vh = max(25, state.horizontalSpeed)
    // Outer loops must be slower than the attitude loop they drive (a
    // cascade needs bandwidth separation), so every time constant is
    // floored at a multiple of the measured closed-loop response.
    let separation = 1 / (1 - GuidanceTargetLimiter.responseReserve)
    let rollFloor = (rollResponseSeconds ?? 0) * separation
    let pitchFloor = (pitchResponseSeconds ?? 0) * separation
    let timeConstant = max(rollFloor, ref.segment == .spiral && abs(ref.curvature) > 1e-8
      ? max(configuration.guidance.pathLookaheadTime,
          0.65*(configuration.guidance.hacLookaheadAngle ?? radians(16))/(abs(ref.curvature)*vh))
      : configuration.guidance.pathLookaheadTime)
    let lateral =
      vh * vh * ref.curvature + 2 * 0.9 / timeConstant * vh * sin(headingError) + crossError
      / (timeConstant * timeConstant)
    let gamma = atan(ref.slope)
    let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
    // TAEM energy exchange: off the final segment, a speed error is repaid
    // with its exact energy-height equivalent instead of holding geometric
    // height with angle of attack (which turns a deficit into more drag).
    let speed = state.aerodynamics.airspeed
    let exchange = ref.segment != .final && ref.speed > 0
      ? (ref.speed * ref.speed - speed * speed) / (2 * g) : 0
    let heightTarget = ref.height - exchange
    let rateTime = max(5, pitchFloor), heightTime = rateTime * 12 / 5
    let desiredVertical =
      speed * sin(gamma) + clamp((heightTarget - p.height) / heightTime, -35, 35)
    let vertical =
      ((g - vh * vh / state.position.length) * cos(state.flightPathAngle)
        + (desiredVertical - state.verticalSpeed) / rateTime + state.aerodynamics.airspeed
        * state.aerodynamics.airspeed * ref.verticalCurvature)
    let maxAlpha = rotation.supportedTrimAlpha(state, identifiedCeiling: identifiedTrimCeiling)
    let availableLift = aero.liftSolution(acceleration: 0, mass: state.mass,
      dynamicPressure: state.aerodynamics.dynamicPressure, mach: state.aerodynamics.mach,
      minimumAlpha: configuration.vehicle.minimumAngleOfAttack, maximumAlpha: maxAlpha)
      .maximumAcceleration * max(0.5, liftEffectiveness)
    // A low-energy vehicle cannot repay tracking error by banking away the
    // lift supporting it. Unload the turn until normal force is available.
    let support = max(0, (g-vh*vh/state.position.length)*cos(state.flightPathAngle))
    let reservedVertical = max(support, vertical)
    let bankAuthority = availableLift > reservedVertical
      ? min(configuration.vehicle.maximumBank, acos(clamp(reservedVertical/availableLift, 0, 1))) : 0
    let bank = clamp(atan2(lateral, max(1, vertical)), -bankAuthority, bankAuthority)
    let required = clamp(
      vertical / max(0.3, cos(bank)), 0, configuration.vehicle.maximumLoadFactor * 9.80665)
    let lift = aero.liftSolution(
      acceleration: required / max(0.5, liftEffectiveness), mass: state.mass,
      dynamicPressure: state.aerodynamics.dynamicPressure,
      mach: state.aerodynamics.mach, minimumAlpha: configuration.vehicle.minimumAngleOfAttack,
      maximumAlpha: maxAlpha)
    return (
      .init(
        angleOfAttack: lift.alpha, bank: bank, desiredSpeed: ref.speed, desiredHeight: ref.height,
        desiredFlightPath: gamma,
        gearDown: p.height < (configuration.guidance.gearDeploymentAltitude ?? 1500)), ref
    )
  }
}
