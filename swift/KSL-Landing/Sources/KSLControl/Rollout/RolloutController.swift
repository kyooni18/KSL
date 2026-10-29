import Foundation
import KSLCore

public final class RolloutController {
  private let frame: RunwayFrame
  private var brake = 0.0
  public init(configuration: MissionConfiguration) {
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func command(state: VehicleState, dt: Double) -> ControlCommand {
    let p = frame.coordinates(of: state.position)
    let desired = state.localFrame.heading(
      of: frame.direction(at: p.horizontal, tangent: .init(1, 0)))
    let error = wrapAngle(desired - state.course)
    let steering = clamp(error * 2.5 - p.cross * 0.01, -1, 1)
    brake = moveToward(brake, 1, maximumDelta: dt / 2)
    return .init(
      pitch: 0, roll: 0, yaw: clamp(error * 0.5, -0.2, 0.2), gearDown: true, wheelBrake: brake,
      wheelSteering: steering)
  }
}
