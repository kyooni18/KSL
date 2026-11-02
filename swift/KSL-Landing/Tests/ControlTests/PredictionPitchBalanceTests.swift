import XCTest
import KSLCore
@testable import KSLControl

final class PredictionPitchBalanceTests: XCTestCase {
  func testSaturatedEquilibriumIsPredictionBiasNotAClaimedTrimEnvelope() {
    var trim = AdaptiveTrim()
    for i in 0...30 {
      trim.observe(time: Double(i) * 0.1, mach: 4, pressure: 4000,
        alpha: radians(24), input: 1, settled: true)
    }
    XCTAssertEqual(trim.predictionInput(time: 3.1, mach: 4, pressure: 4000, alpha: radians(24)), 1)
    XCTAssertNil(trim.input(alpha: radians(24)), "Do not inject saturated observations into control feedforward")
    XCTAssertNil(trim.ceiling(maximum: radians(40)), "A local equilibrium does not identify an AoA envelope")
  }

  func testBalanceCannotCrossUnsupportedRegimeOrTelemetryGap() {
    var trim = AdaptiveTrim()
    for time in [0.0, 0.1, 0.2] {
      trim.observe(time: time, mach: 4, pressure: 4000, alpha: radians(24), input: 1, settled: true)
    }
    XCTAssertNil(trim.predictionInput(time: 3, mach: 4, pressure: 4000, alpha: radians(24)))
    XCTAssertNil(trim.predictionInput(time: 0.3, mach: 3, pressure: 4000, alpha: radians(24)))
    XCTAssertNil(trim.predictionInput(time: 0.3, mach: 4, pressure: 9000, alpha: radians(24)))
    XCTAssertNil(trim.predictionInput(time: 0.3, mach: 4, pressure: 4000, alpha: radians(30)))
    trim.observe(time: 10, mach: 4, pressure: 4000, alpha: radians(24), input: 1, settled: true)
    XCTAssertNil(trim.predictionInput(time: 10, mach: 4, pressure: 4000, alpha: radians(24)))
  }

  func testUnsettledSaturationIsNotMomentEvidence() {
    var trim = AdaptiveTrim()
    for i in 0...30 {
      trim.observe(time: Double(i) * 0.1, mach: 4, pressure: 4000,
        alpha: radians(24), input: 1, settled: false)
    }
    XCTAssertNil(trim.predictionInput(time: 3.1, mach: 4, pressure: 4000, alpha: radians(24)))
  }

  func testSaturatedNewRegimeCannotReuseAnOldUnsaturatedFit() {
    var trim = AdaptiveTrim()
    for i in 0...40 {
      let alpha = 15 + Double(i) * 0.2
      trim.observe(time: Double(i) * 0.1, mach: 2, pressure: 1000,
        alpha: radians(alpha), input: 0.1 + 0.01 * alpha, settled: true)
    }
    XCTAssertNotNil(trim.input(alpha: radians(20)))
    trim.observe(time: 4.1, mach: 4, pressure: 4000,
      alpha: radians(30), input: 1, settled: true)
    XCTAssertEqual(trim.predictionInput(time: 4.2, mach: 4, pressure: 4000, alpha: radians(30)), 1)
    XCTAssertNil(trim.predictionInput(time: 4.2, mach: 4, pressure: 4000, alpha: radians(20)))
  }

  func testPredictionOnlyEquilibriumDoesNotBecomeFeedforwardOrAnIncidenceEnvelope() {
    var trim = AdaptiveTrim()
    for i in 0...3 {
      trim.observe(time: Double(i)*0.1, mach: 2.8, pressure: 3480,
        alpha: radians(26.5), input: 0.8, settled: false, predictionSettled: true)
    }
    XCTAssertEqual(trim.predictionInput(time: 0.4, mach: 2.8, pressure: 3480, alpha: radians(26.5)), 0.8)
    XCTAssertNil(trim.input(alpha: radians(26.5)))
    XCTAssertNil(trim.ceiling(maximum: radians(40)))
  }

}
