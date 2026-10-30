import Foundation

public struct ControlPlantConfiguration: Codable, Sendable, Equatable {
  // STS-N subsonic identification, inherited by the STS 2026-10-28
  // simulator. Its use for STS entry, and roll/yaw, remain provisional.
  public var pitchAccelerationPerKPa = radians(12.6211)
  public var pitchAccelerationMaximum = radians(56.7948)
  public var pitchStiffnessPerKPa = 0.61037
  public var trimAngleOfAttack = radians(2.445)
  public var pitchDampingRatio = 0.0
  public var rollAccelerationPerKPa = radians(6)
  public var rollAccelerationMaximum = radians(60)
  public var rollDamping = 0.8
  public var yawAccelerationPerKPa = radians(3)
  public var yawAccelerationMaximum = radians(30)
  public var yawStiffnessPerKPa = 0.10
  public var maximumPitchRate = radians(7.93)
  public var maximumRollRate = radians(20.18)
  public var surfaceRate = 2.5
  public init() {}
}

public struct VehicleConfiguration: Codable, Sendable, Equatable {
  public var identifier = "STS"
  public var nominalMass = 25_000.0
  public var referenceArea = 250.0
  public var inertia = Vector3(86_820, 948_498, 980_321)  // FRD; archival L4 estimate
  public var maximumDynamicPressure = 45_000.0
  public var maximumLoadFactor = 3.5
  public var maximumBank = radians(70)
  public var minimumAngleOfAttack = radians(-5)
  public var maximumAngleOfAttack = radians(40)
  public var maximumLandingAngleOfAttack = radians(18)
  public var gearHeight = 3.2
  public var landingSpeed = 85.0
  public var minimumSafeSpeed: Double?
  public var maximumTouchdownSpeed = 115.0
  public var maximumTouchdownSink = 3.0
  public var maximumTouchdownBank = radians(7)
  public var maximumTouchdownPitch = radians(14)
  public var brakeFriction = 0.28
  public var rollingFriction = 0.015
  public var aerodynamicUncertainty = 0.20
  public var controlPlant = ControlPlantConfiguration()
  public init() {}
}

public struct GuidanceConfiguration: Codable, Sendable, Equatable {
  public var controlInterval = 0.1
  public var entryReplanInterval = 5.0
  public var terminalReplanInterval = 5.0
  public var terminalCandidateBudget: Int?
  public var approachPredictionInterval: Double?
  public var terminalRollRate: Double?
  public var approachRollRate: Double?
  public var gearDeploymentAltitude: Double?
  public var finalAlignmentSpeed: Double?
  public var hacRadius: Double?
  public var hacLookaheadAngle: Double?
  public var hacAcquisitionAltitude: Double?
  public var hacAcquisitionMach: Double?
  public var taemGlideAngle: Double?
  public var terminalPlanningMaximumSpeed: Double?
  public var touchdownSinkRate: Double?
  public var rolloutParachutes: Bool?
  public var maximumPredictionDuration = 1600.0
  public var predictionStep = 2.0
  public var finalDistance = 8000.0
  public var outerGlideAngle = radians(24)
  public var finalSpeed = 120.0
  public var touchdownAlong = 400.0
  public var shallowGlideAngle = radians(1.8)
  public var flareHeight = 18.0
  public var preflareAccelerationReserve = 0.60 * 9.80665
  public var shallowSegmentLength = 100.0
  public var minimumHACRadius = 3000.0
  public var maximumHACRadius = 40_000.0
  public var maximumPathLength = 220_000.0
  public var bankSlewRate = radians(18)
  public var alphaSlewRate = radians(2)
  public var pathLookaheadTime = 6.0
  public var allowDirectTerminal = true
  public var maximumSnapshotSpan = 0.12
  public var maximumTelemetryAge = 0.5
  public var stopConfirmationDuration = 2.0
  public init() {}
}

public struct MissionConfiguration: Codable, Sendable, Equatable {
  public var planet = PlanetConfiguration()
  public var runway = RunwayGeometry()
  public var vehicle = VehicleConfiguration()
  public var guidance = GuidanceConfiguration()
  public var physicsArchive: PhysicsArchiveConfiguration?
  public var entry: EntryConfiguration?
  public var deorbit: DeorbitConfiguration?
  public var calibration: CalibrationConfiguration?
  public var deorbitSettings: DeorbitConfiguration { deorbit ?? .init() }
  public var entrySettings: EntryConfiguration { entry ?? .init() }
  public init() {}
  public func validate() throws {
    try entrySettings.validate()
    try deorbitSettings.validate(planet: planet)
    try calibration?.validate()
    guard planet.radius > 0, planet.gravitationalParameter > 0,
      vehicle.nominalMass > 0, vehicle.referenceArea > 0,
      vehicle.inertia.x > 0, vehicle.inertia.y > 0, vehicle.inertia.z > 0,
      runway.length > 0, runway.width > 0,
      guidance.controlInterval > 0, guidance.controlInterval <= 0.2,
      vehicle.maximumBank > 0, vehicle.maximumBank < .pi / 2,
      guidance.minimumHACRadius > 0,
      guidance.maximumHACRadius >= guidance.minimumHACRadius,
      vehicle.maximumAngleOfAttack > vehicle.minimumAngleOfAttack,
      vehicle.landingSpeed > 0,
      guidance.outerGlideAngle > 0, guidance.outerGlideAngle < .pi / 3,
      abs(runway.latitude) < .pi / 2, abs(runway.longitude) <= .pi,
      guidance.finalDistance >= 500, guidance.finalDistance <= 30_000,
      guidance.touchdownAlong >= 0, guidance.touchdownAlong < runway.length - 200,
      guidance.finalSpeed > vehicle.landingSpeed,
      guidance.shallowGlideAngle > 0, guidance.shallowGlideAngle < guidance.outerGlideAngle,
      guidance.preflareAccelerationReserve > 0,
      guidance.preflareAccelerationReserve < 9.80665 * (vehicle.maximumLoadFactor - 1),
      guidance.shallowSegmentLength > 0, guidance.shallowSegmentLength < 1000,
      guidance.maximumPredictionDuration > 0, guidance.maximumPredictionDuration <= 7200,
      guidance.predictionStep > 0, guidance.predictionStep <= 5,
      guidance.entryReplanInterval >= guidance.controlInterval,
      guidance.terminalReplanInterval >= guidance.controlInterval,
      (1...128).contains(guidance.terminalCandidateBudget ?? 8),
      (guidance.approachPredictionInterval ?? 1.5) >= guidance.controlInterval,
      (guidance.terminalRollRate ?? guidance.bankSlewRate) > 0,
      (guidance.terminalRollRate ?? guidance.bankSlewRate) <= vehicle.controlPlant.maximumRollRate,
      (guidance.approachRollRate ?? guidance.bankSlewRate) > 0,
      (guidance.approachRollRate ?? guidance.bankSlewRate) <= vehicle.controlPlant.maximumRollRate,
      (guidance.gearDeploymentAltitude ?? 1500) > vehicle.gearHeight,
      (guidance.finalAlignmentSpeed ?? 260) >= guidance.finalSpeed,
      (guidance.hacRadius ?? 12000) >= guidance.minimumHACRadius,
      (guidance.hacRadius ?? 12000) <= guidance.maximumHACRadius,
      (guidance.hacLookaheadAngle ?? radians(16)) > 0,
      (guidance.hacLookaheadAngle ?? radians(16)) <= .pi,
      (guidance.hacAcquisitionAltitude ?? entrySettings.terminalMinimumAltitude) > 0,
      (guidance.hacAcquisitionAltitude ?? entrySettings.terminalMinimumAltitude) <= entrySettings.terminalMaximumAltitude,
      (guidance.hacAcquisitionMach ?? entrySettings.terminalTargetMach) > 0,
      (guidance.hacAcquisitionMach ?? entrySettings.terminalTargetMach) <= entrySettings.terminalTargetMach+entrySettings.terminalMachHalfWidth,
      (guidance.taemGlideAngle ?? guidance.outerGlideAngle) > 0,
      (guidance.taemGlideAngle ?? guidance.outerGlideAngle) < .pi/3,
      (guidance.terminalPlanningMaximumSpeed ?? 1300) > guidance.finalSpeed,
      (guidance.touchdownSinkRate ?? -1) < 0,
      abs(guidance.touchdownSinkRate ?? -1) < vehicle.maximumTouchdownSink,
      (vehicle.minimumSafeSpeed ?? vehicle.landingSpeed) >= vehicle.landingSpeed,
      (vehicle.minimumSafeSpeed ?? vehicle.landingSpeed) < guidance.finalSpeed,
      guidance.flareHeight > vehicle.gearHeight,
      guidance.bankSlewRate > 0, guidance.bankSlewRate <= vehicle.controlPlant.maximumRollRate,
      guidance.alphaSlewRate > 0, guidance.pathLookaheadTime > 0,
      guidance.maximumTelemetryAge > 0, guidance.maximumSnapshotSpan > 0,
      guidance.stopConfirmationDuration >= 0,
      vehicle.brakeFriction > 0, vehicle.rollingFriction >= 0,
      vehicle.aerodynamicUncertainty >= 0, vehicle.aerodynamicUncertainty < 0.8,
      vehicle.maximumLoadFactor > 1, vehicle.maximumDynamicPressure > 0,
      vehicle.maximumTouchdownSpeed > 0, vehicle.maximumTouchdownSink > 0,
      vehicle.controlPlant.maximumPitchRate > 0, vehicle.controlPlant.maximumRollRate > 0,
      vehicle.controlPlant.pitchAccelerationPerKPa > 0,
      vehicle.controlPlant.pitchAccelerationMaximum > 0,
      vehicle.controlPlant.rollAccelerationPerKPa > 0,
      vehicle.controlPlant.rollAccelerationMaximum > 0,
      vehicle.controlPlant.pitchStiffnessPerKPa >= 0, vehicle.controlPlant.surfaceRate > 0
    else {
      throw FlightError.invalidConfiguration("Nonphysical or unsupported configuration value")
    }
    // Encoding rejects NaN/infinity anywhere in nested configuration.
    _ = try JSONEncoder().encode(self)
  }
}
