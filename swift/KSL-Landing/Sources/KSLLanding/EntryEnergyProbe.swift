import Foundation
import KSLCore
import KSLDiagnostics
import KSLGuidance
import KSLPhysics

/// Matched-state evidence, not a landing certificate. Replays recorded actuator
/// commands for each interval, resetting to measured state at every boundary.
enum EntryEnergyProbe {
  static func run(input: String, configuration c: MissionConfiguration) async throws {
    let reader = try JSONLReader(url: URL(fileURLWithPath: input))
    defer { try? reader.close() }
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let rotation = RotationalDynamicsEngine(configuration: c.vehicle, planet: c.planet)
    func energy(_ s: VehicleState) -> Double {
      s.velocity.squaredLength/2 - c.planet.gravitationalParameter/s.position.length
        - c.planet.angularVelocity.cross(s.position).squaredLength/2
    }
    print("time,elapsed,altitude,mach,speed,specificEnergy,energyHeight,runwayRange,verticalSpeed,bank,bankTarget,alpha,alphaTarget,lift,drag,referenceDrag,objectiveAltitude,objectiveSpeed,forecastAltitude,forecastMach,measuredEnergyLoss,dragWork,plantEnergyLoss,plantAlpha,pitchAcceleration,measuredPitchAcceleration,modelLift,modelDrag,bestGlideAlpha")
    var previous: FlightRecord?, t0: Double?
    var objectiveAltitude = c.entrySettings.terminalTargetAltitude
    var objectiveSpeed = 0.0
    while let bytes = try reader.nextLine() {
      let r = try JSONDecoder().decode(FlightRecord.self, from: bytes)
      let s = r.state, a = s.aerodynamics
      if t0 == nil { t0 = s.time }
      if let proposal = r.planningProposal,
        let o = try JSONDecoder().decode(MissionPlanningProposal.self, from: proposal).entry?.capture?.objective {
        objectiveAltitude = o.altitude; objectiveSpeed = o.speed
      }
      guard let p = previous else { previous = r; continue }
      previous = r
      let dt = s.time-p.state.time
      guard dt > 0, dt <= 1 else { continue }
      let physics = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere, aerodynamics: aero,
        initialControls: p.state.appliedControls ?? p.command)
      let predicted = try physics.advance(p.state, command: p.command, duration: dt)
      let accel = rotation.angularAcceleration(state: p.state, surfaces: .init(command: p.command)).y
      let measuredAccel = (s.angularVelocity.y-p.state.angularVelocity.y)/dt
      let model = aero.evaluate(position: s.position, velocity: s.velocity, attitude: s.attitude,
        atmosphere: s.atmosphere, gear: s.systems.gearDown)
      let referenceGravity = c.planet.gravitationalParameter/pow(c.planet.radius+c.runway.elevation, 2)
      let runwayEnergy = -c.planet.gravitationalParameter/(c.planet.radius+c.runway.elevation)
        - c.planet.angularVelocity.cross(frame.origin).squaredLength/2
      let cap = r.guidance.capture
      let values: [Double] = [s.time,s.time-t0!,s.position.length-c.planet.radius,a.mach,a.airspeed,
        energy(s),(energy(s)-runwayEnergy)/referenceGravity,
        frame.distance(from: s.position, to: frame.origin),s.verticalSpeed,
        degrees(a.bank),degrees(r.guidance.target.bank),degrees(a.angleOfAttack),degrees(r.guidance.target.angleOfAttack),
        a.lift/s.mass,a.drag/s.mass,r.guidance.entry?.referenceDrag ?? 0,objectiveAltitude,objectiveSpeed,
        cap?.predictedAltitude ?? .nan,cap?.predictedMach ?? .nan,
        energy(p.state)-energy(s),dt*(p.state.aerodynamics.drag/p.state.mass*p.state.aerodynamics.airspeed+a.drag/s.mass*a.airspeed)/2,
        energy(p.state)-energy(predicted),degrees(predicted.aerodynamics.angleOfAttack),accel,measuredAccel,
        model.lift/s.mass,model.drag/s.mass,
        degrees(aero.bestGlideAlpha(mach: a.mach, maximumAlpha: c.vehicle.maximumAngleOfAttack, dynamicPressure: a.dynamicPressure))]
      print(values.map { String($0) }.joined(separator: ","))
    }
  }
}
