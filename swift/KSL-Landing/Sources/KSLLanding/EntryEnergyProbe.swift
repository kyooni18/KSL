import Foundation
import KSLControl
import KSLCore
import KSLDiagnostics
import KSLGuidance
import KSLPhysics

/// Matched-state evidence, not a landing certificate. Replays recorded actuator
/// commands for each interval, resetting to measured state at every boundary.
enum EntryEnergyProbe {
  static func compare(input: String, configuration c: MissionConfiguration, elapsed: Double, duration: Double) async throws {
    let reader = try JSONLReader(url: URL(fileURLWithPath: input))
    defer { try? reader.close() }
    var records: [FlightRecord] = []
    while let data = try reader.nextLine() { records.append(try JSONDecoder().decode(FlightRecord.self, from: data)) }
    guard let first = records.first,
      let start = records.firstIndex(where: { $0.state.time >= first.state.time+elapsed }) else {
      throw FlightError.invalidConfiguration("Probe start outside recording")
    }
    let atm = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let warm = FlightController(configuration: c)
    for i in 1..<start {
      let dt = records[i].state.time-records[i-1].state.time
      if dt > 0 && dt <= 1 { _ = try warm.command(state: records[i].state, solution: records[i].guidance, dt: dt) }
      else { warm.resumeAfterTelemetryGap(at: records[i].state) }
    }
    let initial = records[start].state, dt = c.guidance.controlInterval
    print("branch,elapsed,altitude,mach,speed,specificEnergy,range,verticalSpeed,alpha,bank,alphaTarget,bankTarget,drag,lift,referenceDrag,captureEnergyMargin")
    for corrected in [false, true] {
      let controller = FlightController(configuration: c, snapshot: warm.snapshot)
      let plant = ShuttlePhysicsEngine(configuration: c, atmosphere: atm, aerodynamics: aero,
        initialControls: initial.appliedControls ?? .init(), initialState: initial,
        pitchBalancingInput: warm.snapshot.predictionPitchTrim(state: initial))
      let law = EntryGuidanceEngine(configuration: c, atmosphere: atm, aerodynamics: aero)
      var limiter = GuidanceTargetLimiter(previous: records[start].guidance.target)
      var state = plant.observe(initial), index = start, nextOutput = initial.time
      while state.time-initial.time < duration && !state.contact.isGrounded && state.contact != .crashed {
        while index+1 < records.count && records[index+1].state.time <= state.time { index += 1 }
        let request = corrected ? law.update(state, synchronousPlanning: false, feedback: controller.diagnostics)
          : records[index].guidance.target
        let target = limiter.update(request, phase: .entry, configuration: c, dt: dt,
          dynamicPressure: state.aerodynamics.dynamicPressure,
          rollResponseSeconds: controller.diagnostics.rollResponseSeconds, pitchResponseSeconds: controller.diagnostics.pitchResponseSeconds,
          controlAuthority: state.controlAuthority)
        if state.time >= nextOutput {
          nextOutput += 1
          let a = state.aerodynamics
          let e = state.velocity.squaredLength/2-c.planet.gravitationalParameter/state.position.length
            - c.planet.angularVelocity.cross(state.position).squaredLength/2
          let numbers = [state.time-first.state.time,state.position.length-c.planet.radius,a.mach,a.airspeed,e,
            frame.distance(from: state.position,to:frame.origin),state.verticalSpeed,degrees(a.angleOfAttack),degrees(a.bank),
            degrees(target.angleOfAttack),degrees(target.bank),a.drag/state.mass,a.lift/state.mass,
            law.diagnostics.referenceDrag,law.diagnostics.captureEnergyMargin ?? .nan]
          print(([corrected ? "corrected" : "recorded-targets"]+numbers.map { String($0) }).joined(separator: ","))
        }
        let command = try controller.command(state: state, solution: .init(phase: .entry,target:target),dt:dt)
        state = try plant.advance(state,command:command,duration:dt)
      }
    }
  }

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
        initialControls: p.state.appliedControls ?? p.command, initialState: p.state)
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
