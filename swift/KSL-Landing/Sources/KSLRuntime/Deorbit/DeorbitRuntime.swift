import Foundation
import KSLCore
import KSLControl
import KSLPhysics

/// One orbital execution owner. A successful OMS shutdown transfers the same
/// I/O and recorder to the ordinary native MM304-through-rollout runtime.
public actor DeorbitRuntime {
  private let io: any OrbitalFlightIO
  private let recorder: any TelemetryRecording
  private let plan: DeorbitPlan
  private let aero: AerodynamicsEngine
  private let reducedAssurance: Bool
  private let maximumOutage: Double
  private var running = false
  public init(io: any OrbitalFlightIO, recorder: any TelemetryRecording,
              plan: DeorbitPlan, aerodynamics: AerodynamicsEngine, reducedAssurance: Bool = false,
              maximumOutage: Double = 20) {
    self.io = io; self.recorder = recorder; self.plan = plan; aero = aerodynamics
    self.reducedAssurance = reducedAssurance
    self.maximumOutage = maximumOutage
  }
  public func run(maximumDuration: Double = 36000, realTime: Bool = true) async throws -> RuntimeOutcome {
    guard !running else { throw FlightError.invalidCommand("Deorbit runtime is already running") }
    running = true
    do {
      try plan.authorize(identifier: plan.identifier, reducedAssurance: reducedAssurance)
      if reducedAssurance && !plan.executionQualified {
        let warning = "deorbit advisory: reduced assurance; modeled nominal=\(plan.nominalPassed), passing=\(plan.passedCases)/15, unsafe=\(plan.unsafeCases); this is not a flight safety or landing certificate\n"
        FileHandle.standardError.write(Data(warning.utf8))
      }
      let fresh = try await io.readState()
      try validateFreshOrbit(fresh)
      try await io.armDeorbit(plan: plan, reducedAssurance: reducedAssurance)
      let encodedPlan = try JSONEncoder().encode(plan)
      let result = try await execute(maximumDuration: maximumDuration, realTime: realTime, encodedPlan: encodedPlan)
      if result != .complete { await io.shutdown(); try await recorder.close() }
      running = false
      return result
    } catch {
      let io = self.io, recorder = self.recorder
      let cleanup = Task { await io.shutdown(); try? await recorder.close() }
      await cleanup.value; running = false
      if error is CancellationError { return .cancelled }
      throw error
    }
  }
  public func validateFreshOrbit(_ state: VehicleState) throws {
    let c = plan.configuration, s = c.deorbitSettings
    guard state.time >= plan.createdAt, state.time < plan.window.burnStart else {
      throw FlightError.rearmRequired("Plan clock/window is no longer current")
    }
    let orbit = OrbitalMechanics(planet: c.planet)
    let expected = try orbit.propagate(.init(fixed: plan.source, planet: c.planet), to: state.time)
    let actual = OrbitalState(fixed: state, planet: c.planet)
    guard (expected.position-actual.position).length <= s.positionUncertainty+1,
      (expected.velocity-actual.velocity).length <= s.velocityUncertainty+0.01,
      abs(state.mass-plan.source.mass) <= s.massUncertainty*plan.source.mass+0.01,
      let propulsion = state.propulsion,
      abs(propulsion.availableThrust/plan.propulsion.availableThrust-1) <= s.thrustUncertainty+1e-6,
      state.systems.throttle <= 0.0001, state.systems.thrust <= 1 else {
      throw FlightError.rearmRequired("Current orbit, propulsion or mass lies outside the reviewed dispersions")
    }
  }
  private func execute(maximumDuration: Double, realTime: Bool, encodedPlan: Data) async throws -> RuntimeOutcome {
    let c = plan.configuration, dt = c.guidance.controlInterval
    let orbit = OrbitalMechanics(planet: c.planet), burn = DeorbitBurnController(plan: plan)
    let coastPointing = OrbitalAttitudeController(), clock = ContinuousClock()
    var previousTime: Double?, sequence: UInt64 = 0, previousTick: ContinuousClock.Instant?
    var lastFresh = clock.now, shutdownComplete = false, missed: UInt64 = 0
    func seconds(_ duration: Duration) -> Double {
      let p = duration.components; return Double(p.seconds)+Double(p.attoseconds)/1e18
    }
    while true {
      try Task.checkCancellation()
      let tick = clock.now, state = try await io.readState(), acquired = clock.now
      if let t = previousTime {
        if state.time < t || state.time-t > 1 { throw FlightError.rearmRequired("Orbital clock discontinuity") }
        if state.time == t {
          if lastFresh.duration(to: tick) > .seconds(600) { throw FlightError.staleTelemetry }
          try await clock.sleep(for: .milliseconds(20)); continue
        }
      }
      let interval = previousTime.map { state.time-$0 } ?? dt
      previousTime = state.time; lastFresh = tick
      if state.time-plan.createdAt > maximumDuration { return .timedOut }
      guard !state.systems.sas, !state.systems.airbrakes else {
        throw FlightError.rearmRequired("An external attitude or airbrake system changed during orbital execution")
      }
      let height = state.position.length-c.planet.radius
      if shutdownComplete && height <= c.planet.atmosphereTop-c.deorbitSettings.entryAltitudeMargin && state.verticalSpeed < 0 {
        guard state.systems.unpowered else { throw FlightError.infeasible("OMS thrust persisted after engine shutdown") }
        try await io.beginEntry()
        let entry = try FlightRuntime(io: io, recorder: recorder, configuration: c,
          maximumOutage: maximumOutage, aerodynamics: aero)
        return try await entry.run(maximumDuration: max(0, maximumDuration-(state.time-plan.createdAt)), realTime: realTime)
      }
      let phase: MissionPhase, command: OrbitalControlCommand, control: ControlDiagnostics
      if shutdownComplete {
        let attitude = Quaternion.windAttitude(position: state.position, airVelocity: state.velocity,
          alpha: c.entrySettings.nominalAlpha, bank: 0)
        let axes = try coastPointing.command(state: state, direction: attitude.rotate(.init(1, 0, 0)), dt: interval)
        phase = .entryCoast; command = .init(planID: plan.identifier, axes: axes, throttle: 0)
        control = coastPointing.diagnostics
      } else {
        let output = try burn.update(state, periapsisAltitude: orbit.elements(.init(fixed: state, planet: c.planet)).periapsisAltitude)
        phase = output.phase; command = output.command; control = burn.controlDiagnostics
        if burn.complete { try await io.finishDeorbit(); shutdownComplete = true }
      }
      let computed = clock.now
      sequence += 1
      try Task.checkCancellation()
      try await io.applyOrbital(sequence: sequence, stateTime: state.time, command: command)
      let applied = clock.now
      if seconds(tick.duration(to: applied)) > dt { missed += UInt64(floor(seconds(tick.duration(to: applied))/dt)) }
      var record = FlightRecord(sequence: sequence, state: state,
        guidance: .init(phase: phase, target: .init(angleOfAttack: shutdownComplete ? c.entrySettings.nominalAlpha : 0)),
        command: command.axes, control: control)
      var diagnostics = burn.diagnostics; diagnostics.reducedAssuranceAuthorized = reducedAssurance
      record.orbital = diagnostics; record.orbitalCommand = command
      record.planningMode = "native-orbital-v1"
      if sequence == 1 { record.initialPhase = .orbitalCoast; record.initialDeorbitPlan = encodedPlan }
      record.timing = .init(simulationInterval: interval, acquisitionSeconds: seconds(tick.duration(to: acquired)),
        computationSeconds: seconds(acquired.duration(to: computed)), commandSeconds: seconds(computed.duration(to: applied)),
        wallInterval: previousTick.map { seconds($0.duration(to: tick)) } ?? 0, missedDeadlines: missed)
      previousTick = tick
      try await recorder.record(record)
      if realTime { try await clock.sleep(until: tick.advanced(by: .seconds(dt))) }
    }
  }
}
