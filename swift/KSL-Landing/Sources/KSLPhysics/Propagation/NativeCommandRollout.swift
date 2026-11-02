import KSLCore
import KSLControl

/// Immutable branch input. Configuration and calibrated model instances are
/// supplied by the caller. This is a native-model seed, not an exact live-plant
/// reconstruction: measured surface/authority anchors retain their uncertainty.
public struct NativeCommandSeed: Sendable {
  public let state: VehicleState
  public let controls: FlightControlSnapshot?
  public let limiter: GuidanceTargetLimiter
  public let plant: ShuttlePhysicsSnapshot?
  public init(state: VehicleState, controls: FlightControlSnapshot?, limiter: GuidanceTargetLimiter,
              plant: ShuttlePhysicsSnapshot? = nil) {
    self.state = state; self.controls = controls; self.limiter = limiter; self.plant = plant
  }
}

/// Keep requested guidance, limited guidance and issued actuator command
/// separate. Forces in before/after belong to their respective physical states;
/// neither the requested target nor the limited target overwrites attitude.
public struct NativeCommandStep: Sendable {
  public let before: VehicleState, after: VehicleState
  public let requested: GuidanceSolution
  public let limited: GuidanceTarget
  public let issued: ControlCommand
}

/// Shared deterministic command path for entry and terminal prediction. Each
/// branch owns its FCS, limiter, surface history and finite-response plant.
/// Stopping conditions and runway qualification belong to the caller: a short
/// rollout is never implicitly a fully priced terminal continuation.
public final class NativeCommandRollout {
  private let configuration: MissionConfiguration
  public let modelConsistent: Bool
  private let controller: FlightController
  private let plant: ShuttlePhysicsEngine
  public private(set) var state: VehicleState
  public private(set) var limiter: GuidanceTargetLimiter
  /// Measured actuator authority does not identify aerodynamic pitch moment.
  /// A missing observed balance must not silently become zero restoring bias.
  /// This necessary seed check does not certify extrapolated moment dynamics.
  public var pitchMomentSupported: Bool {
    let anchored = state.controlAuthority.map {
      $0.isValid && [.measured, .identified].contains($0.evidence)
    } ?? false
    return !anchored || plant.snapshot.pitchBalancingInput != nil
  }
  public var diagnostics: ControlDiagnostics { controller.diagnostics }
  public var controlSnapshot: FlightControlSnapshot { controller.snapshot }
  public var seed: NativeCommandSeed {
    .init(state: state, controls: controller.snapshot, limiter: limiter, plant: plant.snapshot)
  }
  public var touchdown: TouchdownEvent? { plant.touchdown }

  public init(seed: NativeCommandSeed, configuration: MissionConfiguration,
              atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    modelConsistent = seed.plant?.matches(configuration: configuration,
      atmosphere: atmosphere, aerodynamics: aerodynamics) ?? true
    controller = .init(configuration: configuration, snapshot: seed.controls)
    plant = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
      initialControls: seed.state.appliedControls ?? .init(), initialState: seed.state,
      pitchBalancingInput: seed.controls?.predictionPitchTrim(state: seed.state), snapshot: seed.plant)
    limiter = seed.limiter
    state = plant.observe(seed.state)
  }

  @discardableResult
  public func advance(_ requested: GuidanceSolution, duration: Double) throws -> NativeCommandStep {
    // Check before mutating limiter/controller history.
    guard duration.isFinite, duration > 0, duration <= 1 else { throw FlightError.staleTelemetry }
    guard modelConsistent else {
      throw FlightError.invalidConfiguration("Continuation model/configuration changed; rebase from measured history")
    }
    let rcs = requested.phase == .entry && configuration.vehicle.rcsAttitudeControl
    guard plant.authoritySupports(rcs: rcs) else {
      throw FlightError.invalidConfiguration("Prediction lacks authority for requested RCS transition")
    }
    let before = state
    let target = limiter.update(requested.target, phase: requested.phase,
      configuration: configuration, dt: duration, dynamicPressure: state.aerodynamics.dynamicPressure,
      rollResponseSeconds: diagnostics.rollResponseSeconds,
      pitchResponseSeconds: diagnostics.pitchResponseSeconds, controlAuthority: state.controlAuthority)
    let command = try controller.command(state: state,
      solution: .init(phase: requested.phase, target: target), dt: duration)
    state = try plant.advance(state, command: command, duration: duration, maximumStep: 0.02)
    return .init(before: before, after: state, requested: requested, limited: target, issued: command)
  }
}
