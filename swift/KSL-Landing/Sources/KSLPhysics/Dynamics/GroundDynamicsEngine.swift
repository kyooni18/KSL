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
  private let rotation: RotationalDynamicsEngine
  public init(configuration: MissionConfiguration) {
    self.configuration = configuration
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet)
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
    // Locate the crossing inside the integration step. Sampling the state at
    // its end aliases impact velocity and makes touchdown depend on step size.
    let oldHeight = previous.position.length - configuration.planet.radius - surface - v.gearHeight
    let newHeight = altitude - surface - v.gearHeight
    let fraction = oldHeight > 0 ? clamp(oldHeight / max(1e-12, oldHeight - newHeight), 0, 1) : 1
    var impact = current
    impact.position = previous.position + (current.position - previous.position) * fraction
    impact.velocity = previous.velocity + (current.velocity - previous.velocity) * fraction
    impact.time = previous.time + (current.time - previous.time) * fraction
    impact.attitude = previous.attitude.advanced(bodyRate: (previous.angularVelocity + current.angularVelocity) / 2,
      dt: impact.time - previous.time)
    let location = frame.coordinates(of: impact.position)
    let ok =
      runway && current.systems.gearDown && abs(impact.verticalSpeed) <= v.maximumTouchdownSink
      && impact.horizontalSpeed <= v.maximumTouchdownSpeed
      && abs(current.aerodynamics.bank) <= v.maximumTouchdownBank
      && impact.pitch <= v.maximumTouchdownPitch && impact.pitch >= radians(-3)
    let event = TouchdownEvent(
      time: impact.time, along: location.along, cross: location.cross, horizontalSpeed: impact.horizontalSpeed,
      verticalSpeed: impact.verticalSpeed, bank: current.aerodynamics.bank, pitch: impact.pitch,
      onRunway: runway, acceptable: ok)
    current.contact = ok ? .mainGear : .crashed
    if ok {
      current.wheelContact = .init(mainGear: true, noseGear: impact.pitch <= radians(0.5),
        wheelCount: 3, evidence: .provisional)
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
    let normal = max(0, g - state.aerodynamics.force.dot(up) / state.mass)
    let deceleration =
      normal * (v.rollingFriction + v.brakeFriction * (command.wheelBrake > 0.5 ? 1 : 0)) + state
      .aerodynamics.drag / state.mass
    let nextSpeed = max(0, speed - deceleration * dt)
    var direction = state.velocity.projected(onPlaneWithNormal: up).normalized
    if direction.length < 0.5 { direction = frame.alongAxis }
    let noseContact = state.wheelContact?.noseGear == true
    let turnRate = noseContact ? clamp(command.wheelSteering * 0.07, -0.18, 0.18) * min(1, speed / 12) : 0
    direction = direction.rotated(about: up, angle: -turnRate * dt)
    state.position = state.position + direction * ((speed + nextSpeed) * 0.5 * dt)
    state.position =
      state.position.normalized
      * (configuration.planet.radius + configuration.runway.elevation + v.gearHeight)
    state.velocity =
      direction.projected(onPlaneWithNormal: state.position.normalized).normalized * nextSpeed
    if !noseContact {
      // Main-wheel pivot from C terminal_wheel_height: the measured STS main
      // axle is 0.9058 m aft of COM. This ground model remains provisional
      // until its pivot, strut and tyre response are checked against live data.
      let aft = 0.9058, down = v.gearHeight, angle = state.pitch
      let heightDerivative = aft*cos(angle)-down*sin(angle)
      let heightSecond = -aft*sin(angle)-down*cos(angle)
      let inertia = v.inertia.y
      let moment = rotation.angularAcceleration(state: state, surfaces: .init(command: command)).y*inertia
      let reactionWithoutAngularAcceleration = state.mass*(normal+heightSecond*state.angularVelocity.y*state.angularVelocity.y)
      let acceleration = (moment-reactionWithoutAngularAcceleration*heightDerivative)
        / (inertia+state.mass*heightDerivative*heightDerivative)
      let pitch = max(0, angle+state.angularVelocity.y*dt+0.5*acceleration*dt*dt)
      let rate = state.angularVelocity.y+acceleration*dt
      let landedNose = pitch <= radians(0.5)
      state.attitude = .windAttitude(position: state.position, airVelocity: direction,
        alpha: landedNose ? 0 : pitch, bank: 0)
      state.angularVelocity = .init(0, landedNose ? 0 : rate, 0)
      state.wheelContact = .init(mainGear: true, noseGear: landedNose, wheelCount: 3, evidence: .provisional)
      let comHeight = landedNose ? down : down*cos(pitch)+aft*sin(pitch)
      state.position = state.position.normalized*(configuration.planet.radius+configuration.runway.elevation+comHeight)
    } else {
      state.attitude = .windAttitude(position: state.position, airVelocity: direction, alpha: 0, bank: 0)
      state.angularVelocity = .zero
    }
    state.time += dt
    let p = frame.coordinates(of: state.position)
    if !configuration.runway.contains(along: p.along, cross: p.cross) {
      state.contact = .crashed
    } else {
      state.contact = state.wheelContact?.noseGear == true ? (nextSpeed < 0.3 ? .stopped : .rolling) : .mainGear
    }
  }
}
