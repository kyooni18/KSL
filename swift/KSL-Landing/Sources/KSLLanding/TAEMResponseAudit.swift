import Foundation
import KSLCore
import KSLControl
import KSLPhysics
import KSLDiagnostics

/// One historical response interval. The FCS is warmed exclusively with prior
/// measured states and recorded limited targets. Plant replay uses recorded
/// issued commands, never future force/trim corrections or measured resets.
enum TAEMResponseAudit {
  struct LocalResidualPrediction: Encodable {
    let anchorInterval: Double, angularAccelerationBias: Vector3
    let altitudeError: Double, speedError: Double, alphaError: Double, bankError: Double
    let angularRateError: Vector3, energyChange: Double
  }
  struct Report: Encodable {
    let evidence: String
    let warmRecords: Int, dynamicsResets: Int
    let maximumWarmCommandError: Double
    let seedTime: Double, duration: Double, intervals: Int
    let maximumReplayCommandError: Double
    let seedLiftResidual: Double, seedDragResidual: Double
    let altitudeError: Double, speedError: Double, alphaError: Double, bankError: Double
    let angularRateError: Vector3
    let measuredEnergyChange: Double, predictedEnergyChange: Double
    let seedRCS: Bool, firstCommandRCS: Bool
    let localResidualPrediction: LocalResidualPrediction
    let limitations: [String]
  }
  static func run(input: String, configuration c: MissionConfiguration) async throws -> Report {
    let reader = try FlightReplayReader(url: URL(fileURLWithPath: input))
    defer { try? reader.close() }
    let atmosphere = try AtmosphericModelEngine(planet: c.planet)
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: c)
    var controller = FlightController(configuration: c)
    var previousTime: Double?, sequence: UInt64?, warm = 0, resets = 0
    var maximumWarmError = 0.0
    var seed: VehicleState?, plant: ShuttlePhysicsEngine?, predicted: VehicleState?
    var localPlant: ShuttlePhysicsEngine?, localPredicted: VehicleState?, previousState: VehicleState?
    var localBias = Vector3.zero, anchorInterval = 0.0
    var precedingCommand: ControlCommand?, end: VehicleState?
    var horizon = 0.0, intervals = 0, replayError = 0.0
    var liftResidual = 0.0, dragResidual = 0.0, firstRCS = false
    func error(_ a: ControlCommand, _ b: ControlCommand) -> Double {
      max(abs(a.pitch-b.pitch), abs(a.roll-b.roll), abs(a.yaw-b.yaw),
        abs((a.speedbrakeFraction ?? 0)-(b.speedbrakeFraction ?? 0)))
    }
    func energy(_ s: VehicleState) -> Double {
      s.velocity.squaredLength/2-c.planet.gravitationalParameter/s.position.length
        - c.planet.angularVelocity.cross(s.position).squaredLength/2
    }
    while let r = try reader.next() {
      if r.orbitalCommand != nil { continue }
      if r.sequence == 1 {
        guard seed == nil else { throw FlightError.protocolViolation("New session inside response interval") }
        controller = .init(configuration: c); previousTime = nil; previousState = nil; sequence = nil
        warm = 0; resets = 0; maximumWarmError = 0
      }
      if let sequence, r.sequence != sequence+1 {
        throw FlightError.protocolViolation("Missing/reordered atmospheric control history")
      }
      sequence = r.sequence
      let dt = previousTime.map { r.state.time-$0 } ?? c.guidance.controlInterval
      guard dt > 0, dt <= 1 else {
        throw FlightError.protocolViolation("Unreconciled historical telemetry gap")
      }
      if let change = r.dynamicsChange {
        controller.adaptToDynamicsChange(change, at: r.state); resets += 1
      }
      if seed == nil && [.acquisition, .headingAlignment].contains(r.guidance.phase) {
        seed = r.state
        horizon = max(c.guidance.pathLookaheadTime,
          controller.diagnostics.pitchResponseSeconds ?? 0, controller.diagnostics.rollResponseSeconds ?? 0)
        plant = .init(configuration: c, atmosphere: atmosphere, aerodynamics: aero,
          initialControls: r.state.appliedControls ?? .init(), initialState: r.state,
          pitchBalancingInput: controller.snapshot.predictionPitchTrim(state: r.state))
        predicted = plant!.observe(r.state)
        liftResidual = (predicted!.aerodynamics.lift-r.state.aerodynamics.lift)/r.state.mass
        dragResidual = (predicted!.aerodynamics.drag-r.state.aerodynamics.drag)/r.state.mass
        firstRCS = r.command.rcs
        guard let prior = previousState, prior.systems.rcs == r.state.systems.rcs,
          r.state.controlAuthority?.evidence == .measured, let applied = r.state.appliedControls else {
          throw FlightError.invalidTelemetry("No measured same-actuator history for local residual identification")
        }
        // Use only measurements through the seed. Subtract the known native
        // control/gyro terms from measured body acceleration; do not treat a
        // PD integral as aerodynamic moment evidence.
        anchorInterval = r.state.time-prior.time
        let model = RotationalDynamicsEngine(configuration: c.vehicle, planet: c.planet)
        localBias = (r.state.angularVelocity-prior.angularVelocity)/anchorInterval
          - model.angularAcceleration(state: r.state, surfaces: .init(command: applied))
        localPlant = .init(configuration: c, atmosphere: atmosphere, aerodynamics: aero,
          initialControls: applied, initialState: r.state, localAngularAccelerationBias: localBias)
        localPredicted = localPlant!.observe(r.state)
      }
      if seed != nil, let precedingCommand, let p = predicted {
        predicted = try plant!.advance(p, command: precedingCommand, duration: dt)
        localPredicted = try localPlant!.advance(localPredicted!, command: precedingCommand, duration: dt)
        intervals += 1
      }
      let command = try controller.command(state: r.state, solution: r.guidance, dt: dt)
      if seed == nil { warm += 1; maximumWarmError = max(maximumWarmError, error(command,r.command)) }
      else { replayError = max(replayError,error(command,r.command)) }
      previousTime = r.state.time; previousState = r.state
      if let seed {
        end = r.state
        if r.state.time-seed.time >= horizon { break }
        precedingCommand = r.command
      }
    }
    guard let seed, let end, let predicted, let localPredicted, intervals > 0, end.time-seed.time >= horizon else {
      throw FlightError.invalidConfiguration("No complete first-TAEM response interval in recording")
    }
    return .init(evidence: "recorded-limited-target-FCS-reconstruction-and-issued-command-plant-replay",
      warmRecords: warm, dynamicsResets: resets, maximumWarmCommandError: maximumWarmError,
      seedTime: seed.time, duration: end.time-seed.time, intervals: intervals,
      maximumReplayCommandError: replayError, seedLiftResidual: liftResidual, seedDragResidual: dragResidual,
      altitudeError: predicted.position.length-end.position.length,
      speedError: predicted.aerodynamics.airspeed-end.aerodynamics.airspeed,
      alphaError: wrapAngle(predicted.aerodynamics.angleOfAttack-end.aerodynamics.angleOfAttack),
      bankError: wrapAngle(predicted.aerodynamics.bank-end.aerodynamics.bank),
      angularRateError: predicted.angularVelocity-end.angularVelocity,
      measuredEnergyChange: energy(end)-energy(seed), predictedEnergyChange: energy(predicted)-energy(seed),
      seedRCS: seed.systems.rcs, firstCommandRCS: firstRCS,
      localResidualPrediction: .init(anchorInterval: anchorInterval, angularAccelerationBias: localBias,
        altitudeError: localPredicted.position.length-end.position.length,
        speedError: localPredicted.aerodynamics.airspeed-end.aerodynamics.airspeed,
        alphaError: wrapAngle(localPredicted.aerodynamics.angleOfAttack-end.aerodynamics.angleOfAttack),
        bankError: wrapAngle(localPredicted.aerodynamics.bank-end.aerodynamics.bank),
        angularRateError: localPredicted.angularVelocity-end.angularVelocity,
        energyChange: energy(localPredicted)-energy(seed)), limitations: [
        "Recorded targets are already limited; raw guidance and limiter rates are not reconstructed.",
        "Applied axis telemetry is not individual surface-deflection measurement.",
        "Frozen nominal resources are used; seed force residuals expose unmodeled calibration/history.",
        "The optional local residual is identified at the seed only; its time/q/Mach/alpha dependence is unknown.",
        "Same-snapshot RCS-excluded torque is applied at the command boundary; other authority evolution remains unvalidated.",
        "No uncertainty-based acceptance tolerance is identified; residuals are evidence, not certification.",
        "This is one recorded command interval, not alternative-turn validation or landing qualification."
      ])
  }
}
