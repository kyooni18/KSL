import Foundation
import KSLCore
import KSLControl
import KSLPhysics

/// One orbital execution owner. A successful OMS shutdown transfers the same
/// I/O and recorder to the ordinary native MM304-through-rollout runtime.
public actor DeorbitRuntime {
  private let io: any OrbitalFlightIO
  private let recorder: ResilientTelemetryRecorder
  private let plan: DeorbitPlan
  private let aero: AerodynamicsEngine
  private let reducedAssurance: Bool
  private let maximumOutage: Double
  private var running = false
  // Return to 1x 90 seconds before OMS. The rails warp implementation also
  // requires 30 seconds between its initial sample and this return point.
  static let preBurnWarpHoldSeconds = 90.0
  static let minimumWarpLeadSeconds = 30.0
  static func shouldStartCoastWarp(burnStart: Double, currentTime: Double) -> Bool {
    let remaining = burnStart - currentTime
    return remaining.isFinite && remaining > preBurnWarpHoldSeconds + minimumWarpLeadSeconds
  }
  public init(io: any OrbitalFlightIO, recorder: any TelemetryRecording,
              plan: DeorbitPlan, aerodynamics: AerodynamicsEngine, reducedAssurance: Bool = false,
              maximumOutage: Double = 20) {
    self.io = io; self.recorder = (recorder as? ResilientTelemetryRecorder) ?? .init(sink: recorder); self.plan = plan; aero = aerodynamics
    self.reducedAssurance = reducedAssurance
    self.maximumOutage = maximumOutage
  }
  public func run(maximumDuration: Double = 36000, realTime: Bool = true) async throws -> RuntimeOutcome {
    guard !running else { throw FlightError.invalidCommand("Deorbit runtime is already running") }
    running = true
    do {
      try plan.authorize(identifier: plan.identifier, reducedAssurance: reducedAssurance)
      if !plan.executionQualified {
        let warning = "deorbit: executing physically screened OMS-to-entry; terminal/rollout continuation NOT QUALIFIED (nominal passed=\(plan.nominalPassed), unsafe orbital entry=\(plan.unsafeCases)); retain live MM304/MM305 monitoring\n"
        try? FileHandle.standardError.write(contentsOf: Data(warning.utf8))
      }
      let fresh = try await io.readState()
      try validateFreshOrbit(fresh)
      try await io.armDeorbit(plan: plan, reducedAssurance: reducedAssurance)
      // No physics snapshots or actuator commands while KSP is on rails.
      // Begin the normal 1x closed-loop controller only after the warp has
      // been terminated and a fresh coherent orbital snapshot is available.
      if realTime, let warp = io as? any OrbitalWarpFlightIO,
        Self.shouldStartCoastWarp(burnStart: plan.window.burnStart, currentTime: fresh.time) {
        let resumeAt = plan.window.burnStart - Self.preBurnWarpHoldSeconds
        let achievedUT = try await warp.coastWarp(until: resumeAt)
        guard achievedUT < plan.window.burnStart else {
          throw FlightError.rearmRequired("Rails warp overran the selected OMS window")
        }
        let afterWarp = try await io.readState()
        try validateFreshOrbit(afterWarp)
      }
      let encodedPlan = try JSONEncoder().encode(plan)
      let result = try await execute(maximumDuration: maximumDuration, realTime: realTime, encodedPlan: encodedPlan)
      if result != .complete { await io.shutdown(); await recorder.close() }
      running = false
      return result
    } catch {
      let io = self.io, recorder = self.recorder
      let cleanup = Task { await io.shutdown(); await recorder.close() }
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
    // During many orbits of rails coast, sub-second ephemeris phase drift
    // becomes hundreds of metres of along-track displacement. Treat the
    // separately configured timing uncertainty as ALONG-track only. Maintain
    // the original tight cross-track/radial and inertial velocity limits.
    // Using 3D distance alone wrongly rejects an otherwise valid coast.
    let displacement = actual.position-expected.position
    let tangent = expected.velocity.normalized
    let alongSigned = displacement.dot(tangent)
    let alongTrackError = abs(alongSigned)
    let lateralError = displacement.projected(onPlaneWithNormal: tangent).length
    let alongTrackLimit = s.positionUncertainty
      + s.timingUncertainty*expected.velocity.length + 1
    let lateralLimit = s.positionUncertainty + 1
    // A rail-coast phase shift is not an engine-induced velocity error.
    // On a circular 86 km orbit, ~170 m of permitted along-track phase shift
    // rotates the inertial velocity vector by ~0.56 m/s. Comparing the same
    // UT alone double-counts that phase error and falsely disarms at 1x
    // return. Project the reference to the OBSERVED orbital phase, but keep
    // the original along/cross-track and velocity residual limits separate.
    let phaseSeconds = alongSigned / expected.velocity.length
    let phaseAligned = try orbit.propagate(.init(fixed: plan.source, planet: c.planet),
      to: state.time + phaseSeconds)
    let velocityError = (phaseAligned.velocity-actual.velocity).length
    let velocityLimit = s.velocityUncertainty + 0.01
    let massError = abs(state.mass-plan.source.mass)
    let massLimit = s.massUncertainty*plan.source.mass + 0.01
    let actualThrust = state.propulsion?.availableThrust
    let thrustFractionError = actualThrust.map {
      abs($0/plan.propulsion.availableThrust-1)
    }
    guard alongTrackError.isFinite, lateralError.isFinite,
      velocityError.isFinite, massError.isFinite,
      alongTrackError <= alongTrackLimit, lateralError <= lateralLimit,
      velocityError <= velocityLimit, massError <= massLimit,
      let thrustFractionError, thrustFractionError <= s.thrustUncertainty+1e-6,
      state.systems.throttle <= 0.0001, state.systems.thrust <= 1 else {
      throw FlightError.rearmRequired(
        "Reviewed coast dispersion exceeded: along=\(alongTrackError)m/\(alongTrackLimit)m, "
        + "lateral=\(lateralError)m/\(lateralLimit)m, "
        + "velocity=\(velocityError)m/s/\(velocityLimit), "
        + "mass=\(massError)kg/\(massLimit), "
        + "thrustFraction=\(thrustFractionError.map { String($0) } ?? "missing")")
    }
  }
  private func execute(maximumDuration: Double, realTime: Bool, encodedPlan: Data) async throws -> RuntimeOutcome {
    let c = plan.configuration, dt = c.guidance.controlInterval
    let orbit = OrbitalMechanics(planet: c.planet), burn = DeorbitBurnController(plan: plan)
    var coastPointing = OrbitalAttitudeController()
    let clock = ContinuousClock()
    var previousTime: Double?, sequence: UInt64 = 0, previousTick: ContinuousClock.Instant?
    var lastFresh = clock.now, shutdownComplete = false, missed: UInt64 = 0
    var outageSince: ContinuousClock.Instant?, faults = 0
    func seconds(_ duration: Duration) -> Double {
      let p = duration.components; return Double(p.seconds)+Double(p.attoseconds)/1e18
    }
    while true {
      try Task.checkCancellation()
      let tick = clock.now
      do {
      let state = try await io.readState(), acquired = clock.now
      if let t = previousTime {
        if state.time < t { throw FlightError.rearmRequired("Orbital clock moved backwards") }
        if state.time == t {
          if lastFresh.duration(to: tick) > .seconds(600) { throw FlightError.staleTelemetry }
          try await clock.sleep(for: .milliseconds(20)); continue
        }
      }
      let gap = previousTime.map { state.time-$0 } ?? dt
      let resumed = outageSince != nil || gap > 1
      if resumed {
        if !shutdownComplete { try burn.resumeAfterTelemetryGap(at: state) }
        coastPointing = OrbitalAttitudeController()
        if state.time < plan.window.burnStart { try validateFreshOrbit(state) }
      }
      let interval = resumed ? dt : gap
      previousTime = state.time; lastFresh = tick
      if state.time-plan.createdAt > maximumDuration { return .timedOut }
      guard !state.systems.sas, !state.systems.airbrakes else {
        throw FlightError.rearmRequired("An external attitude or airbrake system changed during orbital execution")
      }
      let height = state.position.length-c.planet.radius
      if height <= c.planet.atmosphereTop-c.deorbitSettings.entryAltitudeMargin && state.verticalSpeed < 0 {
        // The vehicle may reach atmospheric entry before a long, interrupted
        // OMS burn has achieved the planned periapsis. Never continue applying
        // orbital thrust below the entry interface, and never abandon entry
        // guidance merely because the original burn duration was exceeded.
        if !shutdownComplete {
          try await io.finishDeorbit()
          shutdownComplete = true
          try? FileHandle.standardError.write(contentsOf: Data(
            "deorbit: atmospheric handoff before planned OMS completion; underburn recorded, unpowered entry continues\n".utf8))
        }
        // Hand off only after measured throttle/thrust decays to zero; a
        // command to shut down is not itself evidence that thrust stopped.
        if !state.systems.unpowered {
          try await io.applyOrbital(sequence: sequence+1, stateTime: state.time,
            command: .init(planID: plan.identifier, axes: .init(), throttle: 0))
          sequence += 1
          continue
        }
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
      try Task.checkCancellation()
      try await io.applyOrbital(sequence: sequence+1, stateTime: state.time, command: command)
      sequence += 1
      let applied = clock.now
      if seconds(tick.duration(to: applied)) > dt { missed += UInt64(floor(seconds(tick.duration(to: applied))/dt)) }
      var record = FlightRecord(sequence: sequence, state: state,
        guidance: .init(phase: phase, target: .init(angleOfAttack: shutdownComplete ? c.entrySettings.nominalAlpha : 0)),
        command: command.axes, control: control)
      var diagnostics = burn.diagnostics; diagnostics.reducedAssuranceAuthorized = reducedAssurance
      diagnostics.telemetryGapResumed = resumed
      record.orbital = diagnostics; record.orbitalCommand = command
      record.planningMode = "native-orbital-v1"
      if sequence == 1 { record.initialPhase = .orbitalCoast; record.initialDeorbitPlan = encodedPlan }
      record.timing = .init(simulationInterval: interval, acquisitionSeconds: seconds(tick.duration(to: acquired)),
        computationSeconds: seconds(acquired.duration(to: computed)), commandSeconds: seconds(computed.duration(to: applied)),
        wallInterval: previousTick.map { seconds($0.duration(to: tick)) } ?? 0, missedDeadlines: missed)
      previousTick = tick
      try await recorder.record(record)
      if outageSince != nil {
        try? FileHandle.standardError.write(contentsOf: Data("deorbit: link recovered after \(faults) fault(s)\n".utf8))
      }
      outageSince = nil; faults = 0
      } catch let error as FlightError where error.isRecoverable {
        let since = outageSince ?? clock.now
        outageSince = since; faults += 1
        if since.duration(to: clock.now) > .seconds(maximumOutage) { throw error }
        if faults == 1 || faults % 10 == 0 {
          try? FileHandle.standardError.write(contentsOf: Data("deorbit: recovering from \(error) (fault \(faults))\n".utf8))
        }
        try await clock.sleep(for: .milliseconds(150))
        do { try await io.recover() } catch let retry as FlightError where retry.isRecoverable {}
        continue
      }
      if realTime { try await clock.sleep(until: tick.advanced(by: .seconds(dt))) }
    }
  }
}
