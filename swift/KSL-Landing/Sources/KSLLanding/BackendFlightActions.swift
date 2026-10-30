import Foundation
import KSLCore
import KSLGuidance
import KSLPhysics
import KSLRuntime

extension BackendServer {
  func requireIdle() throws {
    guard flight == nil, planning == nil, !preparing, !engaged else {
      throw FlightError.invalidCommand("Another preparation, plan or flight owns this session")
    }
  }
  func startPlanning() async throws {
    try requireIdle()
    generation &+= 1; let epoch = generation
    preparing = true; phase = "Planning"; status = "Preparing read-only physics history and orbital snapshot"
    planning = Task { [weak self] in
      do {
        guard let input = try await self?.preparePlanning(generation: epoch) else { return }
        let worker = DeorbitPlanner(configuration: input.configuration, aerodynamics: input.aerodynamics)
        let value = try await worker.plan(from: input.state) { [weak self] progress in
          await self?.reportPlanning(progress, generation: epoch)
        }
        try Task.checkCancellation()
        await self?.planningEnded(value, error: nil, generation: epoch)
      } catch { await self?.planningEnded(nil, error: String(describing: error), generation: epoch) }
    }
  }
  struct PlanningInput: Sendable {
    let configuration: MissionConfiguration
    let state: VehicleState
    let aerodynamics: AerodynamicsEngine
  }
  func preparePlanning(generation epoch: UInt64) async throws -> PlanningInput {
    let (_, state, aero) = try await resolveHistoryAndRunway()
    try Task.checkCancellation()
    guard epoch == generation else { throw FlightError.rearmRequired("Planning generation changed") }
    preparing = false
    return .init(configuration: configuration, state: state, aerodynamics: aero)
  }
  func reportPlanning(_ progress: DeorbitPlanningProgress, generation epoch: UInt64) {
    guard epoch == generation else { return }
    planningProgress = progress; status = progress.stage; publish()
  }
  func planningEnded(_ result: DeorbitPlan?, error: String?, generation epoch: UInt64) {
    guard epoch == generation else { return }
    planning = nil; preparing = false; planningProgress = nil
    if let result {
      plan = result; phase = "Idle"
      status = result.executionQualified ? "Full route and model evidence qualified for execution"
        : "Plan report ready; execution is not qualified (\(result.passedCases)/15 full-route passes)"
      do {
        let directory = URL(fileURLWithPath: "runs", isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        let encoder = JSONEncoder(); encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        try encoder.encode(result).write(to: directory.appendingPathComponent("native-deorbit-\(result.identifier).json"), options: .withoutOverwriting)
      } catch { lastError = "Plan report persistence failed: \(error)"; status = lastError! }
    } else { phase = "Fault"; lastError = error ?? "No deorbit result"; status = lastError! }
    publish()
  }
  func engageLanding(phase initialPhase: MissionPhase) throws {
    try requireIdle()
    generation &+= 1; let epoch = generation
    preparing = true; phase = "Preparing"; status = "Checking measured \(initialPhase.rawValue) admission"
    flight = Task { [weak self] in
      do {
        guard let runtime = try await self?.prepareLanding(phase: initialPhase, generation: epoch) else { return }
        let outcome = try await runtime.run(maximumDuration: 7200)
        await self?.flightEnded(outcome, error: nil, generation: epoch)
      } catch {
        await self?.preparationFailed(String(describing: error), generation: epoch)
      }
    }
  }
  func prepareLanding(phase initialPhase: MissionPhase, generation epoch: UInt64) async throws -> FlightRuntime {
    let sessionID = UUID().uuidString
    let (value, state, aero) = try await resolveHistoryAndRunway(sessionID: sessionID)
    let atmosphere = try AtmosphericModelEngine(planet: configuration.planet)
    var terminalPlan: TerminalPlan?
    if initialPhase == .approach {
      let approach = ApproachGuidanceEngine(configuration: configuration, atmosphere: atmosphere, aerodynamics: aero)
      guard approach.canCapture(state) else { throw FlightError.infeasible("Final engage fails measured alignment, energy or capture geometry") }
    } else if initialPhase == .acquisition {
      let worker = MissionPlanningWorker(configuration: configuration, atmosphere: atmosphere, aerodynamics: aero)
      let result = await worker.planTerminal(from: state)
      guard let candidate = result.plan else { throw FlightError.infeasible("No complete robust MM305 route: \(result.rejectedConstraints)") }
      let fresh = try await value.readState()
      let admission = MM305Admission.evaluate(fresh, plan: candidate, configuration: configuration)
      guard admission.accepted else { throw FlightError.infeasible("MM305 admission rejected: \(admission.violations.map(\.message))") }
      terminalPlan = candidate
    }
    try Task.checkCancellation()
    guard epoch == generation else { throw FlightError.rearmRequired("Preparation was superseded") }
    let recorder = try recorder(phase: initialPhase, aero: aero, sessionID: sessionID)
    do {
      let runtime = try FlightRuntime(io: value, recorder: recorder, configuration: configuration,
        aerodynamics: aero, initialPhase: initialPhase, initialPlan: terminalPlan)
      try await value.arm()
      try Task.checkCancellation()
      preparing = false; engaged = true; phase = Self.backendPhase(initialPhase)
      status = "Native \(phase) engaged"; publish()
      return runtime
    } catch { await value.shutdown(); try? await recorder.close(); throw error }
  }
  func engageDeorbit(reviewedPlanID: String?, authorization: String?) async throws {
    try requireIdle()
    guard let plan, plan.configuration == configuration else { throw FlightError.invalidCommand("Create a plan for the current configuration first") }
    let id = reviewedPlanID ?? plan.identifier
    let reduced = authorization == plan.identifier
    try plan.authorize(identifier: id, reducedAssurance: reduced)
    generation &+= 1; let epoch = generation
    preparing = true; phase = "Preparing"; status = "Validating the reviewed orbit before native burn control"
    flight = Task { [weak self] in
      do {
        guard let runtime = try await self?.prepareDeorbit(plan: plan, reduced: reduced, generation: epoch) else { return }
        let outcome = try await runtime.run()
        await self?.flightEnded(outcome, error: nil, generation: epoch)
      } catch { await self?.preparationFailed(String(describing: error), generation: epoch) }
    }
  }
  func prepareDeorbit(plan: DeorbitPlan, reduced: Bool, generation epoch: UInt64) async throws -> DeorbitRuntime {
    let value = try await connectIfNeeded()
    let aero = try await PhysicsModelLoader.aerodynamics(configuration: configuration)
    try Task.checkCancellation()
    guard epoch == generation else { throw FlightError.rearmRequired("Deorbit preparation was superseded") }
    let recorder = try recorder(phase: .orbitalCoast, aero: aero, sessionID: UUID().uuidString)
    preparing = false; engaged = true
    return .init(io: value, recorder: recorder, plan: plan, aerodynamics: aero, reducedAssurance: reduced)
  }
  func preparationFailed(_ error: String, generation epoch: UInt64) async {
    guard epoch == generation else { return }
    preparing = false
    if let io { await io.shutdown() }
    flightEnded(nil, error: error, generation: epoch)
  }
}
