import KSLCore
import KSLControl

/// Immutable branch input. Configuration and calibrated model instances are
/// supplied by the caller. This is a native-model seed, not an exact live-plant
/// reconstruction: measured surface/authority anchors retain their uncertainty.
public struct NativeCommandSeed: Sendable {
  public let state: VehicleState
  public let controls: FlightControlSnapshot?
  public let limiter: GuidanceTargetLimiter
  public init(state: VehicleState, controls: FlightControlSnapshot?, limiter: GuidanceTargetLimiter) {
    self.state = state; self.controls = controls; self.limiter = limiter
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
  private let controller: FlightController
  private let plant: ShuttlePhysicsEngine
  public private(set) var state: VehicleState
  public private(set) var limiter: GuidanceTargetLimiter
  public var diagnostics: ControlDiagnostics { controller.diagnostics }
  public var controlSnapshot: FlightControlSnapshot { controller.snapshot }
  public var touchdown: TouchdownEvent? { plant.touchdown }

  public init(seed: NativeCommandSeed, configuration: MissionConfiguration,
              atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    controller = .init(configuration: configuration, snapshot: seed.controls)
    plant = .init(configuration: configuration, atmosphere: atmosphere, aerodynamics: aerodynamics,
      initialControls: seed.state.appliedControls ?? .init(), initialState: seed.state,
      pitchBalancingInput: seed.controls?.predictionPitchTrim(state: seed.state))
    limiter = seed.limiter
    state = plant.observe(seed.state)
  }

  @discardableResult
  public func advance(_ requested: GuidanceSolution, duration: Double) throws -> NativeCommandStep {
    // Check before mutating limiter/controller history.
    guard duration.isFinite, duration > 0, duration <= 1 else { throw FlightError.staleTelemetry }
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
