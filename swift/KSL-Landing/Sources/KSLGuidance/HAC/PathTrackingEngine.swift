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
  public init(configuration: MissionConfiguration, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    aero = aerodynamics
    rotation = .init(configuration: configuration.vehicle)
  }
  public func reset() {
    index = 0
    trackingError = 0
  }
  public func update(_ state: VehicleState, plan: TerminalPlan, liftEffectiveness: Double = 1) -> (
    GuidanceTarget, PathStation
  ) {
    let p = frame.coordinates(of: state.position)
    index = max(index, plan.nearest(to: p.horizontal, startingAt: index))
    let nearest = plan.stations[index]
    let alongOffset = (p.horizontal - nearest.position).dot(nearest.tangent)
    let ref = plan.station(at: clamp(nearest.distance + alongOffset, 0, plan.length))
    let offset = p.horizontal - ref.position
    trackingError = offset.length
    let tangent = frame.direction(at: ref.position, tangent: ref.tangent)
    let course = state.localFrame.heading(of: tangent)
    let headingError = wrapAngle(course - state.course)
    let crossError = -offset.dot(ref.tangent.rightNormal)
    let timeConstant = configuration.guidance.pathLookaheadTime
    let vh = max(25, state.horizontalSpeed)
    let lateral =
      vh * vh * ref.curvature + 2 * 0.9 / timeConstant * vh * sin(headingError) + crossError
      / (timeConstant * timeConstant)
    let gamma = atan(ref.slope)
    let desiredVertical =
      state.aerodynamics.airspeed * sin(gamma) + clamp((ref.height - p.height) / 12, -35, 35)
    let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
    let vertical =
      ((g - vh * vh / state.position.length) * cos(state.flightPathAngle)
        + (desiredVertical - state.verticalSpeed) / 5 + state.aerodynamics.airspeed
        * state.aerodynamics.airspeed * ref.verticalCurvature)
    let bank = clamp(
      atan2(lateral, max(1, vertical)), -configuration.vehicle.maximumBank,
      configuration.vehicle.maximumBank)
    let required = clamp(
      vertical / max(0.3, cos(bank)), 0, configuration.vehicle.maximumLoadFactor * 9.80665)
    let maxAlpha = rotation.maximumTrimAlpha(dynamicPressure: state.aerodynamics.dynamicPressure)
    let lift = aero.liftSolution(
      acceleration: required / max(0.5, liftEffectiveness), mass: state.mass,
      dynamicPressure: state.aerodynamics.dynamicPressure,
      mach: state.aerodynamics.mach, minimumAlpha: configuration.vehicle.minimumAngleOfAttack,
      maximumAlpha: maxAlpha)
    return (
      .init(
        angleOfAttack: lift.alpha, bank: bank, desiredSpeed: ref.speed, desiredHeight: ref.height,
        desiredFlightPath: gamma, gearDown: ref.height < 1500), ref
    )
  }
}
