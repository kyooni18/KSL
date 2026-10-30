import Foundation
import KSLCore
import KSLRuntime

extension BackendServer {
  static func backendPhase(_ phase: MissionPhase) -> String {
    switch phase {
    case .orbitalCoast: "Coast"
    case .burnSetup: "Burn Setup"
    case .deorbitBurn: "Deorbit Burn"
    case .entryCoast: "Entry Coast"
    case .entry: "MM304 Entry"
    case .acquisition, .headingAlignment: "TAEM"
    case .approach, .preflare: "Final Approach"
    case .flare: "Flare"
    case .touchdown: "Touchdown"
    case .rollout: "Rollout"
    case .complete: "Complete"
    case .abort: "Abort"
    }
  }
  func planSummary(_ value: DeorbitPlan) -> [String: Any] {
    ["identifier": value.identifier, "burnUT": value.window.burnStart+value.window.duration/2,
      "burnStartUT": value.window.burnStart, "deltaV": value.window.deltaV,
      "estimatedBurnDuration": value.window.duration, "burnNormalAngle": degrees(value.window.normalAngle),
      "predictedPostBurnPeriapsisAltitude": value.window.periapsisAltitude,
      "predictedEntryRange": value.window.entryRange, "predictedEntryFlightPathAngle": degrees(value.window.entryFlightPath),
      "executionQualified": value.executionQualified, "fullRouteQualified": value.fullRouteQualified,
      "targetCaptureAchieved": value.nominalPassed, "robustPassed": value.passedCases, "robustScenarios": value.cases.count,
      "robustPassFraction": value.passFraction, "unsafeCases": value.unsafeCases,
      "canAuthorizeReducedAssurance": value.permitsExplicitReducedAssurance,
      "modelLimitations": value.modelLimitations]
  }
  func snapshot() -> [String: Any] {
    var value: [String: Any] = ["phase": phase, "automationEngaged": engaged,
      "connectionStatus": connection, "statusMessage": status, "telemetry": telemetry, "command": command,
      "preparing": preparing, "planning": planning != nil]
    if let lastError { value["lastError"] = lastError }
    if let logDirectory { value["fullRateLogDirectory"] = logDirectory }
    do {
      value["nativeConfiguration"] = try object(configuration)
      value["configurationDisposition"] = try object(settings.dispositions)
      var effective = try JSONSerialization.jsonObject(with: settings.effectiveOperatorJSON) as? [String: Any] ?? [:]
      effective["site"] = ["name": configuration.runway.name, "latitude": degrees(configuration.runway.latitude),
        "longitude": degrees(configuration.runway.longitude), "altitude": configuration.runway.elevation,
        "runwayHeading": degrees(configuration.runway.heading), "runwayLength": configuration.runway.length,
        "runwayWidth": configuration.runway.width, "allowReciprocalRunway": settings.allowReciprocal]
      if let archive = configuration.physicsArchive { effective["physicsArchive"] = try object(archive) }
      value["configuration"] = effective
      if let planningProgress { value["planningProgress"] = try object(planningProgress) }
      if let plan { value["plan"] = planSummary(plan) }
    } catch { value["lastError"] = "Snapshot encoding: \(error)" }
    return value
  }
  func observe(_ record: FlightRecord) {
    let now = ContinuousClock.now, oldPhase = phase
    phase = Self.backendPhase(record.guidance.phase)
    // The full-rate file was written first. Operator publication has its own
    // independent wall-clock cadence and never determines the control rate.
    if oldPhase == phase, let lastPublished, lastPublished.duration(to: now) < .milliseconds(500) { return }
    lastPublished = now
    let s = record.state, a = s.aerodynamics
    telemetry = ["ut": s.time, "meanAltitude": s.position.length-configuration.planet.radius,
      "surfaceAltitude": s.radarAltitude, "trueAirSpeed": a.airspeed, "verticalSpeed": s.verticalSpeed,
      "mach": a.mach, "dynamicPressure": a.dynamicPressure, "angleOfAttack": degrees(a.angleOfAttack),
      "roll": degrees(a.bank), "sideslip": degrees(a.sideslip), "gForce": a.force.length/(s.mass*9.80665),
      "targetAoA": degrees(record.guidance.target.angleOfAttack), "targetRoll": degrees(record.guidance.target.bank),
      "pitchInput": record.command.pitch, "rollInput": record.command.roll, "yawInput": record.command.yaw,
      "throttle": s.systems.throttle, "thrust": s.systems.thrust]
    let geo = GeodeticPosition(position: s.position, radius: configuration.planet.radius)
    let runway = RunwayFrame(runway: configuration.runway, planet: configuration.planet).coordinates(of: s.position)
    telemetry["latitude"] = degrees(geo.latitude); telemetry["longitude"] = degrees(geo.longitude)
    telemetry["surfaceSpeed"] = s.velocity.length; telemetry["horizontalSpeed"] = s.horizontalSpeed
    telemetry["groundTrackHeading"] = degrees(((s.course.truncatingRemainder(dividingBy: 2 * .pi) + 2 * .pi).truncatingRemainder(dividingBy: 2 * .pi)))
    telemetry["heading"] = degrees(((s.localFrame.heading(of: s.attitude.rotate(.init(1,0,0))) + 2 * .pi).truncatingRemainder(dividingBy: 2 * .pi)))
    telemetry["pitch"] = degrees(s.pitch); telemetry["mass"] = s.mass
    let frame = RunwayFrame(runway: configuration.runway, planet: configuration.planet)
    let radial = s.position.normalized
    telemetry["runwayAlongTrack"] = configuration.planet.radius*atan2(radial.dot(frame.alongAxis),radial.dot(frame.up))
    telemetry["runwayCrossTrack"] = configuration.planet.radius*asin(clamp(radial.dot(frame.rightAxis),-1,1))
    telemetry["runwayCoordinateDefinition"] = "great-circle-track"
    telemetry["nativeRunwayChart"] = ["along":runway.along,"cross":runway.cross]
    telemetry["aerodynamicBank"] = degrees(a.bank)
    command = ["targetAoA": degrees(record.guidance.target.angleOfAttack),
      "targetRoll": degrees(record.guidance.target.bank), "pitch": record.command.pitch,
      "roll": record.command.roll, "yaw": record.command.yaw, "wheelBrake": record.command.wheelBrake,
      "wheelSteering": record.command.wheelSteering, "gear": record.command.gearDown,
      "throttle": record.orbitalCommand?.throttle ?? 0]
    do {
      telemetry["state"] = try object(s)
      telemetry["guidance"] = try object(record.guidance)
      if let control = record.control { telemetry["nativeControl"] = try object(control) }
      if let timing = record.timing { telemetry["controlTiming"] = try object(timing) }
      if let orbital = record.orbital { telemetry["orbitalExecution"] = try object(orbital) }
      if let calibration = record.calibration { telemetry["calibration"] = try object(calibration) }
    } catch { lastError = "Operator telemetry encoding: \(error)" }
    status = record.guidance.violations.isEmpty ? phase
      : record.guidance.violations.map(\.message).joined(separator: "; ")
    publish()
  }
}
