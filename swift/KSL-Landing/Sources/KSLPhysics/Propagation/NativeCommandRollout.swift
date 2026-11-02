import KSLCore
import KSLControl

/// Local model coverage, independent of runway feasibility. Configured-plant
/// support remains provisional; none of these states establishes live readiness.
public enum NativeMomentSupport: String, Codable, Sendable {
  case configuredPlant, vacuum, localBalance
  case missingAuthority, missingBalance, outsideEvidence, nonconstantBalance, unvalidatedResidual
  public var available: Bool {
    self == .configuredPlant || self == .vacuum || self == .localBalance
  }
}

/// Immutable branch input. Configuration and calibrated model instances are
/// supplied by the caller. This is a native-model seed, not an exact live-plant
/// reconstruction: measured surface/authority anchors retain their uncertainty.
public struct NativeCommandSeed: Sendable {
  public let state: VehicleState
  public let controls: FlightControlSnapshot?
  public let limiter: GuidanceTargetLimiter
  public let plant: ShuttlePhysicsSnapshot?
  fileprivate let momentEvidence: FlightControlSnapshot?
  fileprivate let measuredVehicle: Bool
  public init(state: VehicleState, controls: FlightControlSnapshot?, limiter: GuidanceTargetLimiter,
              plant: ShuttlePhysicsSnapshot? = nil) {
    self.state = state; self.controls = controls; self.limiter = limiter; self.plant = plant
    momentEvidence = controls
    measuredVehicle = state.aerodynamics.measuredForces || (state.controlAuthority.map {
      [.measured, .identified].contains($0.evidence)
    } ?? false)
  }
  fileprivate init(state: VehicleState, controls: FlightControlSnapshot?, limiter: GuidanceTargetLimiter,
                   plant: ShuttlePhysicsSnapshot, inherited: Self) {
    self.state = state; self.controls = controls; self.limiter = limiter; self.plant = plant
    momentEvidence = inherited.momentEvidence; measuredVehicle = inherited.measuredVehicle
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
  private let originalSeed: NativeCommandSeed
  private let controller: FlightController
  private let plant: ShuttlePhysicsEngine
  public private(set) var state: VehicleState
  public private(set) var limiter: GuidanceTargetLimiter
  /// Measured actuator authority does not identify aerodynamic pitch moment.
  /// A missing observed balance must not silently become zero restoring bias.
  /// Check the original evidence at the current predicted state; a simulated
  /// observation cannot refresh its age or enlarge its measured regime.
  public var pitchMomentSupported: Bool { pitchMomentSupport.available }
  public var pitchMomentSupport: NativeMomentSupport {
    let anchored = state.controlAuthority.map {
      $0.isValid && [.measured, .identified].contains($0.evidence)
    } ?? false
    guard originalSeed.measuredVehicle else { return .configuredPlant }
    guard anchored else { return .missingAuthority }
    // No aerodynamic restoring moment exists at exactly zero q. This is a
    // physical zero, not a tunable low-q threshold.
    guard plant.snapshot.localAngularAccelerationBias == .zero else { return .unvalidatedResidual }
    if state.aerodynamics.dynamicPressure == 0 { return .vacuum }
    guard let pinned = plant.snapshot.pitchBalancingInput else { return .missingBalance }
    guard let supported = originalSeed.momentEvidence?.predictionPitchTrim(state: state) else { return .outsideEvidence }
    // A constant-bias plant cannot claim a state where the evidenced moment
    // function requires another value. Do not extrapolate a local anchor.
    return pinned == supported ? .localBalance : .nonconstantBalance
  }
  public var diagnostics: ControlDiagnostics { controller.diagnostics }
  public var controlSnapshot: FlightControlSnapshot { controller.snapshot }
  public var seed: NativeCommandSeed {
    .init(state: state, controls: controller.snapshot, limiter: limiter, plant: plant.snapshot, inherited: originalSeed)
  }
  public var touchdown: TouchdownEvent? { plant.touchdown }

  public init(seed: NativeCommandSeed, configuration: MissionConfiguration,
              atmosphere: AtmosphericModelEngine, aerodynamics: AerodynamicsEngine) {
    self.configuration = configuration
    originalSeed = seed
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
