import Foundation
import KSLCore

public struct TouchdownEvent: Codable, Sendable {
  public let time: Double
  public let along: Double
  public let cross: Double
  public let horizontalSpeed: Double
  public let verticalSpeed: Double
  public let bank: Double
  public let pitch: Double
  public let onRunway: Bool
  public let acceptable: Bool
}

public final class GroundDynamicsEngine: Sendable {
  private let configuration: MissionConfiguration
  private let frame: RunwayFrame
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration
    frame = .init(runway: configuration.runway, planet: configuration.planet)
  }
  public func detectContact(previous: VehicleState, current: inout VehicleState) -> TouchdownEvent?
  {
    let v = configuration.vehicle
    let r = frame.coordinates(of: current.position)
    let runway = configuration.runway.contains(along: r.along, cross: r.cross)
    let surface = runway ? configuration.runway.elevation : 0.0
    let altitude = current.position.length - configuration.planet.radius
    guard altitude <= surface + v.gearHeight else { return nil }
    let ok =
      runway && current.systems.gearDown && abs(current.verticalSpeed) <= v.maximumTouchdownSink
      && current.horizontalSpeed <= v.maximumTouchdownSpeed
      && abs(current.aerodynamics.bank) <= v.maximumTouchdownBank
      && current.pitch <= v.maximumTouchdownPitch && current.pitch >= radians(-3)
    let event = TouchdownEvent(
      time: current.time, along: r.along, cross: r.cross, horizontalSpeed: current.horizontalSpeed,
      verticalSpeed: current.verticalSpeed, bank: current.aerodynamics.bank, pitch: current.pitch,
      onRunway: runway, acceptable: ok)
    current.contact = ok ? .mainGear : .crashed
    if ok {
      current.position =
        current.position.normalized * (configuration.planet.radius + surface + v.gearHeight)
      current.velocity = current.velocity.projected(onPlaneWithNormal: current.position.normalized)
    }
    return event
  }
  public func step(state: inout VehicleState, command: ControlCommand, dt: Double) {
    let v = configuration.vehicle
    let speed = state.horizontalSpeed
    let up = state.position.normalized
    let g = configuration.planet.gravitationalParameter / pow(state.position.length, 2)
    let normal = max(0, g - state.aerodynamics.lift / state.mass)
    let deceleration =
      normal * (v.rollingFriction + v.brakeFriction * (command.wheelBrake > 0.5 ? 1 : 0)) + state
      .aerodynamics.drag / state.mass
    let nextSpeed = max(0, speed - deceleration * dt)
    var direction = state.velocity.projected(onPlaneWithNormal: up).normalized
    if direction.length < 0.5 { direction = frame.alongAxis }
    let turnRate = clamp(command.wheelSteering * 0.07, -0.18, 0.18) * min(1, speed / 12)
    direction = direction.rotated(about: up, angle: -turnRate * dt)
    state.position = state.position + direction * ((speed + nextSpeed) * 0.5 * dt)
    state.position =
      state.position.normalized
      * (configuration.planet.radius + configuration.runway.elevation + v.gearHeight)
    state.velocity =
      direction.projected(onPlaneWithNormal: state.position.normalized).normalized * nextSpeed
    state.attitude = .windAttitude(
      position: state.position, airVelocity: direction, alpha: 0, bank: 0)
    state.angularVelocity = .zero
    state.time += dt
    let p = frame.coordinates(of: state.position)
    if !configuration.runway.contains(along: p.along, cross: p.cross) {
      state.contact = .crashed
    } else {
      state.contact = nextSpeed < 0.3 ? .stopped : .rolling
    }
  }
}
