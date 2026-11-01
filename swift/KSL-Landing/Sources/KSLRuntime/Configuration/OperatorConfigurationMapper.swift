import Foundation
import CoreFoundation
import KSLCore

public struct OperatorConnectionSettings: Codable, Sendable, Equatable {
  public var host = "127.0.0.1"
  public var port: UInt16 = 50000
  public var timeout = 4.0
  public var clientName = "KSL Native Swift"
  public init() {}
}
public struct ConfigurationDisposition: Codable, Sendable {
  public let field: String
  public let disposition: String
  public let detail: String
}
public struct OperatorConfigurationResult: Sendable {
  public let mission: MissionConfiguration
  public let connection: OperatorConnectionSettings
  public let airbrakeGroup: UInt32
  public let allowReciprocal: Bool
  public let effectiveOperatorJSON: Data
  public let dispositions: [ConfigurationDisposition]
}

/// Strict legacy-operator -> SI native configuration boundary. Every supplied
/// leaf is applied, explicitly inactive for this transport/mode, or rejected.
public enum OperatorConfigurationMapper {
  public static func map(_ data: Data, base: MissionConfiguration = .init(),
    baseConnection: OperatorConnectionSettings = .init(), baseAirbrakeGroup: UInt32 = 1,
    baseAllowReciprocal: Bool = false) throws -> OperatorConfigurationResult {
    guard let input = try JSONSerialization.jsonObject(with: data) as? [String: Any] else {
      throw FlightError.invalidConfiguration("Operator configuration must be an object")
    }
    let sections: Set<String> = ["connection", "site", "vehicle", "guidance", "calibration", "physicsArchive"]
    guard Set(input.keys).isSubset(of: sections) else {
      throw FlightError.invalidConfiguration("Unknown configuration sections: \(Set(input.keys).subtracting(sections).sorted())")
    }
    var leaves: [String: Any] = [:], effective = input
    for (section, value) in input where section != "physicsArchive" {
      guard let values = value as? [String: Any] else { throw FlightError.invalidConfiguration("\(section) must be an object") }
      for (key, value) in values { leaves[section+"."+key] = value }
    }
    var used = Set<String>(), dispositions: [ConfigurationDisposition] = []
    func record(_ key: String, _ kind: String = "applied", _ detail: String = "Validated native SI setting") {
      used.insert(key); dispositions.append(.init(field: key, disposition: kind, detail: detail))
    }
    func number(_ key: String) throws -> Double? {
      guard let value = leaves[key] else { return nil }
      guard let n = value as? NSNumber, CFGetTypeID(n) != CFBooleanGetTypeID(), n.doubleValue.isFinite else {
        throw FlightError.invalidConfiguration("\(key) must be a finite number")
      }
      record(key); return n.doubleValue
    }
    func boolean(_ key: String) throws -> Bool? {
      guard let value = leaves[key] else { return nil }
      guard let n = value as? NSNumber, CFGetTypeID(n) == CFBooleanGetTypeID() else {
        throw FlightError.invalidConfiguration("\(key) must be a boolean")
      }
      record(key); return n.boolValue
    }
    func string(_ key: String) throws -> String? {
      guard let value = leaves[key] else { return nil }
      guard let s = value as? String, !s.isEmpty else { throw FlightError.invalidConfiguration("\(key) must be a nonempty string") }
      record(key); return s
    }
    func setEffective(_ section: String, _ key: String, _ value: Any) {
      var values = effective[section] as? [String: Any] ?? [:]
      values[key] = value; effective[section] = values
    }
    var c = base, connection = baseConnection, airbrake = baseAirbrakeGroup, reciprocal = baseAllowReciprocal
    if let host = try string("connection.rpcHost") { connection.host = host }
    if let host = try string("connection.address") {
      guard leaves["connection.rpcHost"] == nil || host == connection.host else {
        throw FlightError.invalidConfiguration("Conflicting connection.address and rpcHost")
      }
      connection.host = host
    }
    if let port = try number("connection.rpcPort") {
      guard let value = UInt16(exactly: port), value > 0 else { throw FlightError.invalidConfiguration("Invalid TCP port") }
      connection.port = value
    }
    if let timeout = try number("connection.timeoutMs") { connection.timeout = timeout/1000 }
    if let name = try string("connection.clientName") { connection.clientName = name }
    if try string("connection.serialPort") != nil { record("connection.serialPort", "inactive", "TCP selected; serial transport is not opened") }
    if let baud = try number("connection.baudRate") {
      guard baud > 0 else { throw FlightError.invalidConfiguration("Invalid serial baud rate") }
      record("connection.baudRate", "inactive", "TCP selected; serial transport is not opened")
    }
    guard (0.05...30).contains(connection.timeout), connection.clientName.utf8.count <= 1024 else {
      throw FlightError.invalidConfiguration("Invalid connection deadline/client name")
    }
    let sites: [(String, WritableKeyPath<RunwayGeometry, Double>, Bool)] = [
      ("altitude", \.elevation, false), ("latitude", \.latitude, true), ("longitude", \.longitude, true),
      ("runwayHeading", \.heading, true), ("runwayLength", \.length, false), ("runwayWidth", \.width, false)]
    for (name, key, angle) in sites { if let x = try number("site."+name) { c.runway[keyPath: key] = angle ? radians(x) : x } }
    if let name = try string("site.name") { c.runway.name = name }
    reciprocal = try boolean("site.allowReciprocalRunway") ?? baseAllowReciprocal
    if let model = try string("vehicle.modelId") {
      if model != c.vehicle.identifier { c.physicsArchive = nil }
      c.vehicle.identifier = model
    }
    if let group = try number("vehicle.airbrakeActionGroup") {
      guard let value = UInt32(exactly: group), value <= 9 else { throw FlightError.invalidConfiguration("Invalid airbrake action group") }
      airbrake = value
    }
    let vehicles: [(String, WritableKeyPath<VehicleConfiguration, Double>, Bool)] = [
      ("maximumAngleOfAttack", \.maximumAngleOfAttack, true), ("maximumBankAngle", \.maximumBank, true),
      ("maximumDynamicPressure", \.maximumDynamicPressure, false), ("maximumGLoad", \.maximumLoadFactor, false),
      ("touchdownSpeed", \.landingSpeed, false)]
    for (name, key, angle) in vehicles { if let x = try number("vehicle."+name) { c.vehicle[keyPath: key] = angle ? radians(x) : x } }
    if let x = try number("vehicle.minimumSafeSpeed") { c.vehicle.minimumSafeSpeed = x }
    if let x = try number("vehicle.finalApproachSpeed") { c.guidance.finalSpeed = x }
    if let x = try number("vehicle.speedbrakeFullDeploymentDragArea") {
      c.vehicle.speedbrakeFullDeploymentDragArea = x
    }
    if let x = try number("vehicle.bodyFlapTrimInputAtFullDeflection") {
      c.vehicle.bodyFlapTrimInputAtFullDeflection = x
    }
    let powered = try boolean("vehicle.allowPoweredApproach") ?? false
    let maxApproachThrottle = try number("vehicle.maximumApproachThrottle") ?? 0
    guard !powered, maxApproachThrottle == 0 else {
      throw FlightError.invalidConfiguration("Powered approach conflicts with the unpowered landing contract")
    }
    var entry = c.entrySettings
    let entries: [(String, WritableKeyPath<EntryConfiguration, Double>, Bool)] = [
      ("vehicle.entryAngleOfAttack", \.nominalAlpha, true),
      ("vehicle.estimatedBallisticCoefficient", \.estimatedBallisticCoefficient, false),
      ("vehicle.estimatedLiftToDrag", \.estimatedLiftToDrag, false),
      ("guidance.entryRollRate", \.rollRate, true), ("guidance.entryRollAcceleration", \.rollAcceleration, true),
      ("guidance.sTurnMinimumLegDuration", \.minimumLegDuration, false),
      ("guidance.mm305MinimumAltitude", \.terminalMinimumAltitude, false), ("guidance.mm305MaximumAltitude", \.terminalMaximumAltitude, false),
      ("guidance.mm305TargetMach", \.terminalTargetMach, false), ("guidance.mm305MachHalfWidth", \.terminalMachHalfWidth, false),
      ("guidance.taemInterfaceAltitude", \.terminalTargetAltitude, false), ("guidance.taemInterfaceRange", \.terminalTargetRange, false),
      ("guidance.mm304HandoffAlongTrack", \.handoffAlong, false), ("guidance.mm304HandoffCrossTrack", \.handoffCross, false),
      ("guidance.mm304HandoffRadius", \.handoffRadius, false), ("guidance.mm304PerpendicularHeadingHalfWidth", \.headingHalfWidth, true)]
    for (name, key, angle) in entries { if let x = try number(name) { entry[keyPath: key] = angle ? radians(x) : x } }
    c.entry = entry
    let guidance: [(String, WritableKeyPath<GuidanceConfiguration, Double>, Bool)] = [
      ("finalApproachDistance", \.finalDistance, false), ("finalGlideSlope", \.outerGlideAngle, true),
      ("flareAltitude", \.flareHeight, false), ("touchdownAimAlong", \.touchdownAlong, false)]
    for (name, key, angle) in guidance { if let x = try number("guidance."+name) { c.guidance[keyPath: key] = angle ? radians(x) : x } }
    let optionalGuidance: [(String, WritableKeyPath<GuidanceConfiguration, Double?>, Bool)] = [
      ("approachRollRate", \.approachRollRate, true), ("taemRollRate", \.terminalRollRate, true),
      ("finalAlignmentSpeed", \.finalAlignmentSpeed, false), ("gearDeploymentAltitude", \.gearDeploymentAltitude, false),
      ("hacRadius", \.hacRadius, false), ("hacLookAheadAngle", \.hacLookaheadAngle, true),
      ("hacAcquisitionAltitude", \.hacAcquisitionAltitude, false), ("hacAcquisitionMach", \.hacAcquisitionMach, false),
      ("taemGlideSlope", \.taemGlideAngle, true), ("taemForceHandoffSpeed", \.terminalPlanningMaximumSpeed, false),
      ("touchdownSinkRate", \.touchdownSinkRate, false)]
    for (name, key, angle) in optionalGuidance { if let x = try number("guidance."+name) { c.guidance[keyPath: key] = angle ? radians(x) : x } }
    if let rate = try number("guidance.guidanceRate") {
      guard (5...100).contains(rate) else { throw FlightError.invalidConfiguration("guidanceRate must be 5...100 Hz") }
      c.guidance.controlInterval = 1/rate
    }
    if let interval = try number("guidance.predictionInterval") {
      c.guidance.entryReplanInterval = interval; c.guidance.terminalReplanInterval = interval
      c.guidance.approachPredictionInterval = interval
    }
    if let enabled = try boolean("guidance.taemEnergyLaw"), !enabled {
      throw FlightError.invalidConfiguration("Native MM305 requires energy-aware route feasibility")
    }
    if let enabled = try boolean("guidance.taemHeadingAlignmentCone"), !enabled {
      throw FlightError.invalidConfiguration("Native MM305 requires the spiral HAC admission contract")
    }
    if let enabled = try boolean("guidance.adaptiveFlare"), enabled {
      throw FlightError.invalidConfiguration("The selected native Final law is the C response-led arc; adaptiveFlare is not qualified")
    }
    if let enabled = try boolean("guidance.stsTrajectoryDelivery"), enabled {
      throw FlightError.invalidConfiguration("Experimental C trajectory delivery is not a substitute for native route qualification")
    }
    c.guidance.rolloutParachutes = try boolean("guidance.rolloutParachutes")
    if let enabled = try boolean("guidance.terminalSpeedbrake") {
      if enabled && c.vehicle.speedbrakeFullDeploymentDragArea == nil {
        throw FlightError.invalidConfiguration(
          "terminalSpeedbrake requires vehicle.speedbrakeFullDeploymentDragArea")
      }
      if !enabled { c.vehicle.speedbrakeFullDeploymentDragArea = nil }
      setEffective("guidance", "terminalSpeedbrake", enabled)
      record("guidance.terminalSpeedbrake", enabled ? "applied" : "policyOverride",
        enabled ? "Energy-priced split-rudder control with identified drag area"
          : "Split-rudder energy actuation disabled")
    }
    if try boolean("guidance.useTimeWarp") != nil {
      setEffective("guidance", "useTimeWarp", false)
      record("guidance.useTimeWarp", "inactive", "Native continuous-control execution uses 1x; it never requests timewarp")
    }
    if try number("guidance.deorbitRobustMinimumPassFraction") != nil {
      record("guidance.deorbitRobustMinimumPassFraction", "inactive", "Native deorbit qualification is the nominal full-route case; no stress pass fraction applies")
    }
    var deorbit = c.deorbitSettings
    if let speed = try number("guidance.entryRangeReferenceSpeed") { deorbit.entryRangeReferenceSpeed = speed }
    let burns: [(String, WritableKeyPath<DeorbitConfiguration, Double>, Bool)] = [
      ("deorbitMaximumThrottle", \.maximumThrottle, false), ("deorbitThrottleRampDuration", \.throttleRampDuration, false),
      ("minimumPlanningLeadTime", \.minimumPlanningLeadTime, false), ("targetPostBurnPeriapsisAltitude", \.targetPeriapsisAltitude, false),
      ("entryInterfaceAltitudeMargin", \.entryAltitudeMargin, false), ("targetEntryRange", \.targetEntryRange, false),
      ("targetEntryFlightPathAngle", \.targetEntryFlightPath, true), ("maximumEntryFlightPathAngle", \.minimumEntryFlightPath, true),
      ("targetDeorbitCaptureRadius", \.captureRadius, false), ("deorbitTimingUncertainty", \.timingUncertainty, false),
      ("deorbitThrustUncertaintyFraction", \.thrustUncertainty, false), ("deorbitDeltaVUncertainty", \.deltaVUncertainty, false),
      ("deorbitMassUncertaintyFraction", \.massUncertainty, false), ("deorbitPositionUncertainty", \.positionUncertainty, false),
      ("deorbitVelocityUncertainty", \.velocityUncertainty, false), ("deorbitPointingUncertainty", \.pointingUncertainty, true),
      ("deorbitAtmosphereUncertaintyFraction", \.atmosphereUncertainty, false)]
    for (name, key, angle) in burns { if let x = try number("guidance."+name) { deorbit[keyPath: key] = angle ? radians(x) : x } }
    c.deorbit = deorbit
    var calibration = c.calibration ?? .init()
    let calibrations: [(String, WritableKeyPath<CalibrationConfiguration, Double>, Bool)] = [
      ("abortStallFraction", \.abortStallFraction, false), ("angleOfAttackStep", \.alphaStep, true),
      ("minimumAngleOfAttack", \.minimumAlpha, true), ("maximumAngleOfAttack", \.maximumAlpha, true),
      ("maximumCalibrationDynamicPressure", \.maximumDynamicPressure, false),
      ("maximumSampleDynamicPressure", \.maximumSampleDynamicPressure, false),
      ("maximumCalibrationGLoad", \.maximumLoad, false), ("maximumCalibrationSinkRate", \.maximumSink, false),
      ("maximumSideslip", \.maximumSideslip, true), ("minimumCalibrationRadarAltitude", \.minimumRadarAltitude, false),
      ("minimumDynamicPressure", \.minimumDynamicPressure, false), ("samplingDuration", \.samplingDuration, false),
      ("settlingDuration", \.settlingDuration, false), ("trajectoryLearningRate", \.trajectoryLearningRate, false)]
    for (name, key, angle) in calibrations { if let x = try number("calibration."+name) { calibration[keyPath: key] = angle ? radians(x) : x } }
    if let flag = try boolean("calibration.enablePassiveCalibration") { calibration.enablePassive = flag }
    if let flag = try boolean("calibration.enableTrajectoryCalibration") { calibration.enableTrajectory = flag }
    if let flag = try boolean("calibration.autoApplyInFlight") { calibration.autoApply = flag }
    for name in ["angleOfAttackStep", "minimumAngleOfAttack", "maximumAngleOfAttack", "samplingDuration", "settlingDuration",
      "maximumCalibrationDynamicPressure", "maximumCalibrationGLoad", "maximumCalibrationSinkRate", "minimumCalibrationRadarAltitude"] where leaves["calibration."+name] != nil {
      record("calibration."+name, "inactive", "Validated active-sweep setting; landing execution performs passive identification only")
    }
    if leaves["calibration.trajectoryLearningRate"] != nil {
      record("calibration.trajectoryLearningRate", "inactive", "C uses measured environment directly and dt/(V/aero-acceleration) information blending")
    }
    if leaves["calibration.autoApplyInFlight"] != nil {
      record("calibration.autoApplyInFlight", "advisory", "Requested policy recorded; same-flight data cannot rewrite certified speeds without a qualified prior-flight product")
    }
    c.calibration = calibration
    if let archive = input["physicsArchive"] {
      c.physicsArchive = try JSONDecoder().decode(PhysicsArchiveConfiguration.self,
        from: JSONSerialization.data(withJSONObject: archive))
      record("physicsArchive", "applied", "Read-only archive identity and separately controlled observation journal")
    }
    let unknown = Set(leaves.keys).subtracting(used)
    guard unknown.isEmpty else { throw FlightError.invalidConfiguration("Unmapped configuration fields: \(unknown.sorted())") }
    try c.validate()
    // A path has exactly one final disposition, including any explicit policy override.
    var last: [String: ConfigurationDisposition] = [:]
    for disposition in dispositions { last[disposition.field] = disposition }
    return .init(mission: c, connection: connection, airbrakeGroup: airbrake,
      allowReciprocal: reciprocal, effectiveOperatorJSON: try JSONSerialization.data(withJSONObject: effective, options: .sortedKeys),
      dispositions: last.values.sorted { $0.field < $1.field })
  }
}
