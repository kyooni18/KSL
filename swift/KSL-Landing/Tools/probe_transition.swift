// Offline diagnostics only. Build after `swift build -c debug` using the module
// objects, e.g. for the Xcode SwiftPM backend:
// swiftc -parse-as-library -I .build/out/Products/Debug -I Sources/CSQLite Tools/probe_transition.swift \
//   .build/out/Products/Debug/{KSLCore,KSLControl,KSLPhysics,KSLGuidance,KSLDiagnostics}.o \
//   -lsqlite3 -o .build/out/Products/Debug/ksl-transition-probe
// Keep the probe beside the physics resource bundle. From the repository root:
// swift/KSL-Landing/.build/debug/ksl-transition-probe RUN OUTPUT_DIRECTORY [--dense-entry]
import Foundation
import KSLCore
import KSLControl
import KSLPhysics
import KSLGuidance
import KSLDiagnostics

@main struct TransitionProbe {
  static func main() async throws {
    let args = CommandLine.arguments
    guard args.count >= 3 else { fatalError("usage: probe_transition RUN OUTPUT_DIRECTORY [--dense-entry]") }
    let run = URL(fileURLWithPath: args[1]), out = URL(fileURLWithPath: args[2])
    try FileManager.default.createDirectory(at: out, withIntermediateDirectories: true)
    let c = try JSONDecoder().decode(MissionConfiguration.self,
      from: Data(contentsOf: run.appendingPathComponent("live.jsonl.configuration.json")))
    let reader = try JSONLReader(url: run.appendingPathComponent("live.jsonl"))
    defer { try? reader.close() }
    var records: [FlightRecord] = []
    while let line = try reader.nextLine() { records.append(try JSONDecoder().decode(FlightRecord.self, from: line)) }
    guard let first = records.first else { return }
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let warm = FlightController(configuration: c)
    let authority = AerodynamicAuthorityEstimator(model: aero)
  let tracker = PathTrackingEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero)
    var plan: TerminalPlan?, survey: [[String: Any]] = [], nextSurvey = first.state.time
    var plantCSV = "elapsed,dt,pitchMeasured,pitchPredicted,pitchPredictedWithoutTrim,rollMeasured,rollPredicted,alphaNext,alphaPredicted,bankNext,bankPredicted,turnMeasured,turnPredicted,pitchTrimAnchor,commandError,rcsMeasured,rcsPredicted\n"
    var trackerCSV = "elapsed,planCreated,segment,distance,trackingError,refHeight,refSpeed,refSlope,refVerticalCurvature,rawAlpha,rawBank,limitedAlpha,limitedBank,actualAlpha,actualBank,verticalDemand,availableLift,liftScale,dragScale\n"
    var maximumCommandError = 0.0
    var lastChange = -Double.infinity, discardedProposals = 0
    for i in records.indices {
      let r = records[i], s = r.state, a = s.aerodynamics
      if r.dynamicsChange != nil {
        warm.invalidateDynamics(at: s); authority.reset(); tracker.reset()
        lastChange = s.time
      }
      let feedback = warm.diagnostics
      if let bytes = r.planningProposal {
        let proposal = try JSONDecoder().decode(MissionPlanningProposal.self, from: bytes)
        if proposal.time <= lastChange { discardedProposals += 1 }
        else if let candidate = proposal.terminal?.plan { plan = candidate; tracker.reset() }
      }
      authority.observe(s)
      let elapsed = s.time-first.state.time
      let trim = feedback.trimEvidence == .identified ? feedback.trimAlphaCeiling : nil
      let dense = args.contains("--dense-entry") && r.guidance.phase == .entry
        && frame.distance(from: s.position, to: frame.origin)
          <= c.entrySettings.terminalTargetRange+s.horizontalSpeed*c.guidance.terminalReplanInterval
      if s.time >= nextSurvey || dense {
        nextSurvey = s.time+c.guidance.terminalReplanInterval
        var context = HACFitContext()
        context.previousPlanID = plan?.identifier; context.previousSpiral = plan?.spiral
        context.onSpiral = r.guidance.phase == .headingAlignment
        context.liftScale = authority.liftEffectiveness; context.dragScale = authority.dragEffectiveness
        context.trimAlphaCeiling = trim
        context.rollResponseSeconds = feedback.rollResponseSeconds
        context.pitchResponseSeconds = feedback.pitchResponseSeconds
        let planner = HACPlanningEngine(configuration: c,
          atmosphere: atmosphere.corrected(r.calibration?.environment ?? .init()),
          aerodynamics: aero.scaled(lift: context.liftScale, drag: context.dragScale))
        let result = planner.plan(from: s, allowDirect: false, preview: true,
          controlSnapshot: warm.snapshot, previousTarget: i > 0 ? records[i-1].guidance.target : nil,
          context: context)
        var item = try JSONSerialization.jsonObject(with: JSONEncoder().encode(result)) as! [String: Any]
        // Keep the full family summary/fit, but not hundreds of path stations.
        item["plan"] = nil
        item["elapsed"] = elapsed
        item["selectedFeasible"] = result.plan?.assessment.feasible ?? false
        item["selectedAssessment"] = try result.plan.map {
          var assessment = try JSONSerialization.jsonObject(with: JSONEncoder().encode($0.assessment)) as! [String: Any]
          assessment["violations"] = nil
          return assessment
        }
        survey.append(item)
      }
      if let plan, r.guidance.phase != .entry {
        let (target, ref) = tracker.update(s, plan: plan, liftEffectiveness: authority.liftEffectiveness,
          identifiedTrimCeiling: trim, rollResponseSeconds: feedback.rollResponseSeconds,
          pitchResponseSeconds: feedback.pitchResponseSeconds)
        // Independent expansion of PathTrackingEngine's vertical command.
        let speed = a.airspeed, vh = max(25, s.horizontalSpeed)
        let g = c.planet.gravitationalParameter/s.position.squaredLength
        let floor = (feedback.pitchResponseSeconds ?? 0)/(1-GuidanceTargetLimiter.responseReserve)
        let rateTime = max(5, floor), heightTime = rateTime*12/5
        let exchange = ref.segment != .final && ref.speed > 0 ? (ref.speed*ref.speed-speed*speed)/(2*g) : 0
        let desiredVertical = speed*sin(atan(ref.slope))
          + clamp((ref.height-exchange-frame.coordinates(of: s.position).height)/heightTime, -35, 35)
        let vertical = (g-vh*vh/s.position.length)*cos(s.flightPathAngle)
          + (desiredVertical-s.verticalSpeed)/rateTime+speed*speed*ref.verticalCurvature
        let maxAlpha = RotationalDynamicsEngine(configuration: c.vehicle, planet: c.planet)
          .supportedTrimAlpha(s, identifiedCeiling: trim)
        let maxLift = aero.liftSolution(acceleration: 0, mass: s.mass, dynamicPressure: a.dynamicPressure,
          mach: a.mach, minimumAlpha: c.vehicle.minimumAngleOfAttack, maximumAlpha: maxAlpha).maximumAcceleration
        let values: [Double] = [elapsed, plan.createdAt-first.state.time, ref.distance, tracker.trackingError,
          ref.height, ref.speed, ref.slope, ref.verticalCurvature, degrees(target.angleOfAttack), degrees(target.bank),
          degrees(r.guidance.target.angleOfAttack), degrees(r.guidance.target.bank), degrees(a.angleOfAttack), degrees(a.bank),
          vertical, maxLift, authority.liftEffectiveness, authority.dragEffectiveness]
        let fields = [String(values[0]), String(values[1]), ref.segment.rawValue]+values.dropFirst(2).map { String($0) }
        trackerCSV += fields.joined(separator: ",")+"\n"
      }
      let dtControl = i > 0 ? s.time-records[i-1].state.time : c.guidance.controlInterval
      let reproduced = try warm.command(state: s, solution: r.guidance, dt: dtControl)
      let error = max(abs(reproduced.pitch-r.command.pitch), abs(reproduced.roll-r.command.roll), abs(reproduced.yaw-r.command.yaw))
      maximumCommandError = max(maximumCommandError, error)
      guard i+1 < records.count else { continue }
      let next = records[i+1].state, dt = next.time-s.time
      guard dt > 0 && dt <= 1 else { continue }
      let trimInput = warm.snapshot.predictionPitchTrim(alpha: a.angleOfAttack)
      func advance(trim: Double?) throws -> VehicleState {
        let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero,
          initialControls: s.appliedControls ?? r.command, initialState: s, pitchBalancingInput: trim)
        return try physics.advance(s, command: r.command, duration: dt)
      }
      let predicted = try advance(trim: trimInput), noTrim = try advance(trim: nil)
      let right = s.velocity.projected(onPlaneWithNormal: s.position.normalized).normalized.cross(s.position.normalized).normalized
      let measuredTurn = ((next.velocity-s.velocity)/dt).dot(right)
      let predictedTurn = ((predicted.velocity-s.velocity)/dt).dot(right)
      let values: [Double] = [elapsed, dt, (next.angularVelocity.y-s.angularVelocity.y)/dt,
        (predicted.angularVelocity.y-s.angularVelocity.y)/dt, (noTrim.angularVelocity.y-s.angularVelocity.y)/dt,
        (next.angularVelocity.x-s.angularVelocity.x)/dt, (predicted.angularVelocity.x-s.angularVelocity.x)/dt,
        degrees(next.aerodynamics.angleOfAttack), degrees(predicted.aerodynamics.angleOfAttack),
        degrees(next.aerodynamics.bank), degrees(predicted.aerodynamics.bank), measuredTurn, predictedTurn,
        trimInput ?? .nan, error, s.systems.rcs ? 1.0 : 0.0, predicted.systems.rcs ? 1.0 : 0.0]
      plantCSV += values.map { String($0) }.joined(separator: ",")+"\n"
    }
    try plantCSV.write(to: out.appendingPathComponent("matched-warm-plant.csv"), atomically: true, encoding: .utf8)
    try trackerCSV.write(to: out.appendingPathComponent("tracking.csv"), atomically: true, encoding: .utf8)
    try JSONSerialization.data(withJSONObject: survey, options: [.prettyPrinted, .sortedKeys])
      .write(to: out.appendingPathComponent("survey.json"))
    print("maximum reproduced control error: \(maximumCommandError)")
    print("delivered proposals discarded by dynamics epoch guard: \(discardedProposals)")
    print("maximum tabulated L/D: \(aero.maximumLiftToDragRatio)")
    let reciprocal = RunwayGeometry().reciprocal(planet: c.planet)
    print("reciprocal latitude/longitude/heading: \(reciprocal.latitude), \(reciprocal.longitude), \(reciprocal.heading)")
    print("survey states: \(survey.count), selected feasible: \(survey.filter { $0["selectedFeasible"] as? Bool == true }.count)")
  }
}
