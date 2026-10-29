import Foundation

public struct AtmosphericState: Codable, Sendable, Equatable {
  public var density: Double
  public var pressure: Double
  public var temperature: Double
  public var speedOfSound: Double
  public var wind: Vector3
  public init(
    density: Double = 0, pressure: Double = 0, temperature: Double = 186.946,
    speedOfSound: Double = 274, wind: Vector3 = .zero
  ) {
    self.density = density
    self.pressure = pressure
    self.temperature = temperature
    self.speedOfSound = speedOfSound
    self.wind = wind
  }
}

public struct AerodynamicState: Codable, Sendable, Equatable {
  public var airspeed: Double = 0
  public var mach: Double = 0
  public var dynamicPressure: Double = 0
  public var angleOfAttack: Double = 0
  public var sideslip: Double = 0
  public var bank: Double = 0
  public var lift: Double = 0
  public var drag: Double = 0
  public var force: Vector3 = .zero
  public var measuredForces: Bool = false
  public init() {}
}

public enum ContactState: String, Codable, Sendable {
  case airborne, mainGear, rolling, stopped, crashed
  public var isGrounded: Bool { self == .mainGear || self == .rolling || self == .stopped }
}

public struct SystemState: Codable, Sendable, Equatable {
  public var gearDown = false
  public var throttle = 0.0
  public var thrust = 0.0
  public var sas = false
  public var rcs = false
  public var airbrakes = false
  public var brakes = false
  public init() {}
  public var unpowered: Bool { throttle <= 0.0001 && thrust <= 1 && !sas && !rcs && !airbrakes }
}

public struct VehicleState: Codable, Sendable, Equatable {
  public var time: Double
  public var position: Vector3
  /// Derivative of planet-fixed position. This is NOT inertial velocity.
  public var velocity: Vector3
  public var attitude: Quaternion
  /// Body FRD angular velocity relative to the rotating planet.
  public var angularVelocity: Vector3
  public var mass: Double
  public var atmosphere: AtmosphericState
  public var aerodynamics: AerodynamicState
  public var contact: ContactState
  public var systems: SystemState
  public var radarAltitude: Double
  public var snapshotSpan: Double
  public init(
    time: Double, position: Vector3, velocity: Vector3, attitude: Quaternion,
    angularVelocity: Vector3 = .zero, mass: Double = 25_000,
    atmosphere: AtmosphericState = .init(), aerodynamics: AerodynamicState = .init(),
    contact: ContactState = .airborne, systems: SystemState = .init(),
    radarAltitude: Double = 0, snapshotSpan: Double = 0
  ) {
    self.time = time
    self.position = position
    self.velocity = velocity
    self.attitude = attitude
    self.angularVelocity = angularVelocity
    self.mass = mass
    self.atmosphere = atmosphere
    self.aerodynamics = aerodynamics
    self.contact = contact
    self.systems = systems
    self.radarAltitude = radarAltitude
    self.snapshotSpan = snapshotSpan
  }
  public var localFrame: LocalFrame { .init(position: position) }
  public var verticalSpeed: Double { velocity.dot(position.normalized) }
  public var horizontalSpeed: Double {
    velocity.projected(onPlaneWithNormal: position.normalized).length
  }
  public var flightPathAngle: Double { atan2(verticalSpeed, horizontalSpeed) }
  public var course: Double { localFrame.heading(of: velocity) }
  public var pitch: Double {
    asin(clamp(attitude.rotate(.init(1, 0, 0)).dot(position.normalized), -1, 1))
  }
  public func validate(configuration c: MissionConfiguration) throws {
    guard time.isFinite, position.isFinite, velocity.isFinite, angularVelocity.isFinite,
      mass.isFinite, mass > 0, abs(attitude.norm - 1) < 0.01,
      position.length > c.planet.radius * 0.9,
      snapshotSpan.isFinite, snapshotSpan >= 0, snapshotSpan <= c.guidance.maximumSnapshotSpan,
      aerodynamics.dynamicPressure.isFinite, aerodynamics.dynamicPressure >= 0,
      aerodynamics.angleOfAttack.isFinite, aerodynamics.bank.isFinite,
      atmosphere.density.isFinite, atmosphere.density >= 0,
      radarAltitude.isFinite, radarAltitude >= -1,
      [
        aerodynamics.airspeed, aerodynamics.mach, aerodynamics.sideslip, aerodynamics.lift,
        aerodynamics.drag,
        atmosphere.pressure, atmosphere.temperature, atmosphere.speedOfSound, systems.throttle,
        systems.thrust,
      ].allSatisfy(\.isFinite),
      aerodynamics.airspeed >= 0, aerodynamics.mach >= 0, aerodynamics.drag >= 0,
      aerodynamics.force.isFinite,
      atmosphere.pressure >= 0, atmosphere.temperature > 0, atmosphere.speedOfSound > 0,
      atmosphere.wind.isFinite,
      systems.throttle >= 0, systems.throttle <= 1, systems.thrust >= 0
    else {
      throw FlightError.invalidTelemetry("Nonfinite, incoherent, or nonphysical flight snapshot")
    }
  }
}
