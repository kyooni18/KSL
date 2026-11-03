import XCTest
import KSLCore
import KSLPhysics
import KSLControl
@testable import KSLGuidance
final class ZZScratchEntryDeliveryTests: XCTestCase {
 func testBestGlideRangeDiagnostic() throws {
  let root = "/Users/kyooni18/Code/KSL/"
  let c = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath:root+"runs/20261102T033630-mm304-range-live-72km/configuration.json")))
  let initial = try JSONDecoder().decode(VehicleState.self, from:Data(contentsOf:URL(fileURLWithPath:root+"Validation/MM304-725-Delivery-20261102/measured-initial-state.json")))
  let atm = try AtmosphericModelEngine(planet:c.planet), aero = try AerodynamicsEngine(referenceArea:c.vehicle.referenceArea)
  let frame = RunwayFrame(runway:c.runway,planet:c.planet)
  let law=EntryGuidanceEngine(configuration:c,atmosphere:atm,aerodynamics:aero,permitsReplanning:false)
  var feedback: ControlDiagnostics? = nil
  let propagation=TrajectoryPropagationEngine(configuration:c,atmosphere:atm,aerodynamics:aero)
  let warm=FlightController(configuration:c)
  let text=try String(contentsOfFile:root+"runs/20261102T033630-mm304-range-live-72km/live-sts-flight.jsonl",encoding:.utf8)
  var previousTime: Double?, previousTarget: GuidanceTarget?
  for line in text.split(separator:"\n") {
    let record=try JSONDecoder().decode(FlightRecord.self,from:Data(line.utf8))
    if record.state.time>=initial.time {break}
    if let previousTime { _ = try warm.command(state:record.state,solution:record.guidance,dt:record.state.time-previousTime) }
    previousTime=record.state.time;previousTarget=record.guidance.target
  }
  var reportAt=initial.time
  let result=propagation.predict(from:initial,duration:c.guidance.maximumPredictionDuration,
    step:c.guidance.predictionStep,recordEvery:10,terminalSpeed:725,
    controlSnapshot:warm.snapshot,previousTarget:previousTarget,
    feedback:{feedback=$0},law:{state in
      let target=law.update(state,synchronousPlanning:false,feedback:feedback)
      if state.time>=reportAt {
        reportAt+=20
        let d=law.diagnostics
        print("RANGEDEBUG",state.time-initial.time,frame.coordinates(of:state.position).along,
          state.aerodynamics.airspeed,degrees(state.aerodynamics.angleOfAttack),degrees(target.angleOfAttack),
          d.rangeReferenceBlend ?? -1,d.rangeReferenceAlong ?? 0,d.rangeReferenceBracketed ?? false,
          d.rangeReferenceReachedEnergy ?? false,feedback?.pitchResponseSeconds ?? -1)
      }
      return .init(alpha:target.angleOfAttack,bank:target.bank)
    })
  let end=result.terminalState
  print("CLOSEDLOOP",result.duration,end.position.length-c.planet.radius,end.aerodynamics.airspeed,frame.coordinates(of:end.position),result.violations)
  let encoder=JSONEncoder();encoder.outputFormatting=[.prettyPrinted,.sortedKeys]
  try encoder.encode(result).write(to:URL(fileURLWithPath:root+"Validation/MM304-725-Delivery-20261102/closedloop.json"))

 }
}
