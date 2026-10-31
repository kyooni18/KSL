import XCTest
import KSLCore
import KSLPhysics
@testable import KSLGuidance

final class DeorbitForecastPolicyTests: XCTestCase {
  func testUnpricedFailuresUseOneRunwayResidual() {
    let c = MissionConfiguration(), along = c.entrySettings.handoffAlong
    let close = EntryGuidanceEngine.fallbackDeliveryCost(along: along, cross: 1000, configuration: c)
    let distant = EntryGuidanceEngine.fallbackDeliveryCost(along: along, cross: 40_000, configuration: c)
    XCTAssertLessThan(close, distant)
    XCTAssertEqual(EntryGuidanceEngine.fallbackDeliveryCost(along: along,
      cross: c.entrySettings.handoffCross, configuration: c), 0, accuracy: 1e-12)
  }
  func testFreshForecastUsesUnplannedMissionFallbackObjective() throws {
    var c = MissionConfiguration()
    c.guidance.maximumPredictionDuration = 0.1
    let atmosphere = try AtmosphericModelEngine(), aero = try AerodynamicsEngine()
    let frame = RunwayFrame(runway: c.runway, planet: c.planet)
    let position = frame.position(along: -500_000, cross: 100_000, height: 60_000)
    let velocity = frame.direction(at: .init(-500_000, 100_000), tangent: .init(1, 0))*2000
    let state = ShuttlePhysicsEngine(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero).observe(.init(time: 200, position: position, velocity: velocity,
        attitude: .windAttitude(position: position, airVelocity: velocity,
          alpha: radians(18), bank: 0)))
    let expected = EntryCaptureObjective.seed(configuration: c, atmosphere: atmosphere,
      frame: frame, crossAim: 0, time: state.time)
    let forecaster = EntryCaptureForecaster(configuration: c, atmosphere: atmosphere,
      aerodynamics: aero)
    let forecast = forecaster.forecast(from: state)
    let actual = try XCTUnwrap(forecast.objective)
    XCTAssertEqual(actual.along, expected.along, accuracy: 1e-6)
    XCTAssertEqual(actual.speed, expected.speed, accuracy: 1e-6)
    XCTAssertEqual(actual.energyHeight, expected.energyHeight, accuracy: 1e-6)
    // The online planning request starts with the same empty law memory.
    // It must forecast the policy actually flown before a proposal arrives.
    let online = forecaster.forecast(from: state, memory: EntryLawMemory(),
      rangeMultiplier: 1, controls: nil, previousTarget: nil)
    let onlineObjective = try XCTUnwrap(online.objective)
    XCTAssertEqual(onlineObjective.along, expected.along, accuracy: 1e-6)
    XCTAssertEqual(onlineObjective.speed, expected.speed, accuracy: 1e-6)
    XCTAssertEqual(onlineObjective.energyHeight, expected.energyHeight, accuracy: 1e-6)
  }
}
