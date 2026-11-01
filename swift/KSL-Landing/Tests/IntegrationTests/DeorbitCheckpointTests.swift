import Foundation
import KSLCore
import KSLPhysics
import Testing
@testable import KSLRuntime

private actor CompletedFailureCheckpoint: DeorbitCaseCheckpoint {
  private(set) var loaded: [Int] = []
  private(set) var saved = 0
  let wrongIndex: Bool
  init(wrongIndex: Bool = false) { self.wrongIndex = wrongIndex }
  func load(_ identity: DeorbitCaseIdentity) -> DeorbitStressCase? {
    loaded.append(identity.index)
    return .init(index: wrongIndex ? 99 : identity.index, passed: false, hardUnsafe: false,
      burnCompleted: true, entryFlightPath: -0.03, maximumDynamicPressure: 10_000,
      maximumLoad: 2, touchdownAlong: nil, touchdownCross: nil, touchdownSink: nil,
      touchdownPitch: nil, stoppedAlong: nil, violations: [], evidence: "checkpoint-test-fixture")
  }
  func save(_ identity: DeorbitCaseIdentity, result: DeorbitStressCase) { saved += 1 }
}

struct DeorbitCheckpointTests {
  private func fixture() -> (VehicleState, DeorbitWindow, PropulsionTelemetry) {
    let state = VehicleState(time: 0, position: .init(686_000, 0, 0),
      velocity: .init(0, 2200, 0), attitude: .init())
    let orbit = OrbitalState(fixed: state, planet: .init())
    let window = DeorbitWindow(burnStart: 100, deltaV: 30, duration: 30, normalAngle: 0,
      periapsisAltitude: 55_000, afterBurn: orbit, entryInterface: orbit,
      entryRange: 1_000_000, entrySpeed: 2200, entryFlightPath: -0.03,
      entryCourseError: 0, crossrange: 0, miss: 0)
    return (state, window, .init(availableThrust: 60_000, dryMass: 20_000, specificImpulse: 316))
  }


  @Test func mismatchedCheckpointResultIsRejected() async throws {
    let c = MissionConfiguration(), (source, window, propulsion) = fixture()
    let request = DeorbitPlanner.CaseRequest(window: window,
      index: 0, candidateIndex: 1, entryReplanning: true)
    do {
      _ = try await DeorbitPlanner.evaluateBatch(configuration: c, aero: try .init(),
        source: source, propulsion: propulsion, requests: [request], concurrency: 1,
        checkpoint: CompletedFailureCheckpoint(wrongIndex: true), progress: { _ in })
      Issue.record("A result for another case was accepted")
    } catch let error as FlightError {
      if case .invalidConfiguration = error {} else { Issue.record("Wrong failure: \(error)") }
    }
  }
}
