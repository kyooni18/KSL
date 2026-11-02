import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance
final class ZZLocalReplayTests: XCTestCase {
  func testMatchedFlightHistory() throws {
    let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
    let run = root.appendingPathComponent("runs/20261102T031637Z-bounce-ownership-72km")
    let decoder = JSONDecoder()
    let c = try decoder.decode(MissionConfiguration.self, from: Data(contentsOf: run.appendingPathComponent("configuration.json")))
    let atm = try AtmosphericModelEngine(planet: c.planet), aero = try AerodynamicsEngine(referenceArea: c.vehicle.referenceArea)
    let engine = TAEMGuidanceEngine(configuration: c, atmosphere: atm, aerodynamics: aero)
    let baseline = RecordedBaselineTAEM(configuration: c, atmosphere: atm, aerodynamics: aero)
    let authority = AerodynamicAuthorityEstimator(model: aero)
    var incidence = AttainableIncidenceMonitor()
    var solvedNEP: NominalEnergyPoint?
    var lastNEP = -Double.infinity
    let solver = NominalEnergyPointSolver(configuration:c,atmosphere:atm,aerodynamics:aero)
    var previous: ControlDiagnostics?, previousTarget: GuidanceTarget?
    struct Row: Decodable { let state: VehicleState; let control: ControlDiagnostics; let guidance: Guidance }
    struct Guidance: Decodable { let phase: String; let target: GuidanceTarget; let capture: Capture? }
    struct Capture: Decodable {
      let nominalEnergyAlong: Double?; let nominalEnergyHeight: Double?; let nominalEnergyFlightPath: Double?
      let nominalEnergyAirspeed: Double?; let nominalEnergyPredictedTouchdownSpeed: Double?
    }
    var output = "time,actualGamma,oldGamma,newGamma,oldBank,newBank,oldTracking,newTracking,range,ceiling,liftGain,dragGain,oldFailure,newFailure\n"
    let content = try String(contentsOf: run.appendingPathComponent("live-sts-flight.jsonl"), encoding: .utf8)
    for line in content.split(separator: "\n") {
      guard let r = try? decoder.decode(Row.self, from: Data(line.utf8)) else { continue }
      let s=r.state
      authority.observe(s)
      incidence.observe(time:s.time,measuredAlpha:s.aerodynamics.angleOfAttack,commandedAlpha:previousTarget?.angleOfAttack,
        feedback:previous,fallbackResponseSeconds:c.guidance.pathLookaheadTime)
      defer { previous=r.control;previousTarget=r.guidance.target }
      guard ["acquisition","headingAlignment"].contains(r.guidance.phase) else {continue}
      let ceiling=incidence.combined(with:previous?.trimEvidence == .identified ? previous?.trimAlphaCeiling : nil)
      if s.time-lastNEP >= c.guidance.terminalReplanInterval {
        solvedNEP = solver.solve(from:s,trimAlphaCeiling:ceiling) ?? solvedNEP
        lastNEP = s.time
      }
      let nep = try XCTUnwrap(solvedNEP)
      let old=baseline.update(s,nep:nep,liftGain:authority.liftEffectiveness,dragGain:authority.dragEffectiveness,
        attainableAlphaCeiling:ceiling,pitchResponseSeconds:previous?.pitchResponseSeconds,rollResponseSeconds:previous?.rollResponseSeconds)
      let new=engine.update(s,nep:nep,liftGain:authority.liftEffectiveness,dragGain:authority.dragEffectiveness,
        attainableAlphaCeiling:ceiling,pitchResponseSeconds:previous?.pitchResponseSeconds,rollResponseSeconds:previous?.rollResponseSeconds)
      XCTAssertTrue(new.angleOfAttack.isFinite && new.bank.isFinite)
      XCTAssertLessThanOrEqual(abs(new.bank),c.vehicle.maximumBank+1e-12)
      output += "\(s.time),\(degrees(s.flightPathAngle)),\(degrees(old.desiredFlightPath)),\(degrees(new.desiredFlightPath)),\(degrees(old.bank)),\(degrees(new.bank)),\(baseline.status.referenceTracking ?? false),\(engine.status.referenceTracking ?? false),\(engine.status.range),\(ceiling ?? -1),\(authority.liftEffectiveness),\(authority.dragEffectiveness),\(baseline.status.descentCheckFailure ?? "ok"),\(engine.status.descentCheckFailure ?? "ok")\n"
    }
    try output.write(to:root.appendingPathComponent("Validation/TAEM-SmoothCapture-20261102/matched-replay.csv"),atomically:true,encoding:.utf8)
  }
}
