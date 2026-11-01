import Foundation

/// Right-handed inertial axes coincide with the configured planet-fixed axes
/// at UT zero. This is explicitly different from VehicleState.velocity.
public struct OrbitalState: Codable, Sendable, Equatable {
  public var time: Double
  public var position: Vector3
  public var velocity: Vector3
  public var mass: Double
  public init(time: Double, position: Vector3, velocity: Vector3, mass: Double) {
    self.time = time; self.position = position; self.velocity = velocity; self.mass = mass
  }
  public init(fixed: VehicleState, planet: PlanetConfiguration) {
    let angle = planet.rotationRate*fixed.time
    time = fixed.time; mass = fixed.mass
    position = fixed.position.rotated(about: .init(0, 0, 1), angle: angle)
    velocity = planet.inertialVelocity(position: fixed.position, fixedVelocity: fixed.velocity)
      .rotated(about: .init(0, 0, 1), angle: angle)
  }
  public func planetFixed(planet: PlanetConfiguration, attitude: Quaternion,
                          angularVelocity: Vector3 = .zero) -> VehicleState {
    let angle = -planet.rotationRate*time
    let p = position.rotated(about: .init(0, 0, 1), angle: angle)
    let v = velocity.rotated(about: .init(0, 0, 1), angle: angle)-planet.angularVelocity.cross(p)
    return .init(time: time, position: p, velocity: v, attitude: attitude,
      angularVelocity: angularVelocity, mass: mass)
  }
}

public struct PropulsionTelemetry: Codable, Sendable, Equatable {
  public var availableThrust: Double
  public var dryMass: Double
  public var specificImpulse: Double
  public var evidence: EvidenceKind
  /// Measured net OMS thrust direction in body FRD, aggregated over the
  /// active fuelled engines and weighted by their available thrust.
  /// A missing vector is NOT evidence that thrust follows the vessel nose.
  public var thrustDirectionBody: Vector3?
  public init(availableThrust: Double, dryMass: Double, specificImpulse: Double,
              evidence: EvidenceKind = .measured, thrustDirectionBody: Vector3? = nil) {
    self.availableThrust = availableThrust; self.dryMass = dryMass
    self.specificImpulse = specificImpulse; self.evidence = evidence
    self.thrustDirectionBody = thrustDirectionBody
  }
  public func validate(mass: Double) throws {
    guard availableThrust.isFinite, availableThrust > 0, dryMass.isFinite,
      dryMass > 0, dryMass < mass, specificImpulse.isFinite, specificImpulse > 0 else {
      throw FlightError.invalidTelemetry("OMS thrust, dry mass and specific impulse must be measured before planning")
    }
    if let axis = thrustDirectionBody, (!axis.isFinite || abs(axis.length-1) > 0.01 || axis.x < 0.5) {
      throw FlightError.invalidTelemetry("Measured OMS engine thrust direction is invalid")
    }
  }
}

public struct DeorbitConfiguration: Codable, Sendable, Equatable {
  public var maximumThrottle = 0.18
  public var throttleRampDuration = 12.0
  public var minimumPlanningLeadTime = 60.0
  public var targetPeriapsisAltitude = 50_000.0
  public var entryAltitudeMargin = 500.0
  public var targetEntryRange = 1_030_000.0
  public var entryRangeReferenceSpeed: Double?
  public var targetEntryFlightPath = radians(-2)
  public var minimumEntryFlightPath = radians(-3)
  public var captureRadius = 100_000.0
  public var timingUncertainty = 0.5
  public var thrustUncertainty = 0.08
  public var deltaVUncertainty = 0.5
  public var massUncertainty = 0.03
  public var positionUncertainty = 50.0
  public var velocityUncertainty = 0.5
  public var pointingUncertainty = radians(2)
  public var atmosphereUncertainty = 0.12
  public var minimumSearchOrbits = 1.0
  public var coarseWindowStep = 30.0
  public var candidateBudget = 12
  public init() {}
  public func validate(planet: PlanetConfiguration) throws {
    // OMS must never run at full thrust; ensure planning and live execution
    // agree on the same bounded low-throttle envelope.
    guard maximumThrottle > 0, maximumThrottle <= 0.25, throttleRampDuration >= 0,
      throttleRampDuration <= 60, minimumPlanningLeadTime >= 5,
      targetPeriapsisAltitude > 0, targetPeriapsisAltitude < planet.atmosphereTop,
      entryAltitudeMargin >= 0, entryAltitudeMargin < planet.atmosphereTop/2,
      targetEntryRange > 0, (entryRangeReferenceSpeed ?? 0) >= 0, captureRadius > 0,
      minimumEntryFlightPath < 0, targetEntryFlightPath >= minimumEntryFlightPath,
      targetEntryFlightPath < 0, timingUncertainty >= 0, deltaVUncertainty >= 0,
      (0..<0.8).contains(thrustUncertainty), (0..<0.5).contains(massUncertainty),
      positionUncertainty >= 0, velocityUncertainty >= 0,
      pointingUncertainty >= 0, pointingUncertainty <= radians(10),
      (0..<0.8).contains(atmosphereUncertainty),
      minimumSearchOrbits > 0, minimumSearchOrbits <= 5,
      coarseWindowStep >= 1, coarseWindowStep <= 120,
      (1...100).contains(candidateBudget) else {
      throw FlightError.invalidConfiguration("Invalid deorbit envelope or search configuration")
    }
    _ = try JSONEncoder().encode(self)
  }
}
