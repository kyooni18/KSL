import Foundation

public struct CalibrationConfiguration: Codable, Sendable, Equatable {
  public var enablePassive = true
  public var enableTrajectory = true
  public var autoApply = true
  public var minimumDynamicPressure = 500.0
  public var maximumDynamicPressure = 35000.0
  public var maximumSampleDynamicPressure = 55000.0
  public var maximumLoad = 2.5
  public var maximumSink = 80.0
  public var minimumRadarAltitude = 3000.0
  public var maximumSideslip = radians(8)
  public var minimumAlpha = radians(3)
  public var maximumAlpha = radians(18)
  public var alphaStep = radians(3)
  public var samplingDuration = 8.0
  public var settlingDuration = 4.0
  public var abortStallFraction = 0.32
  public var trajectoryLearningRate = 0.18
  public init() {}
  public func validate() throws {
    guard minimumDynamicPressure >= 0, maximumDynamicPressure > minimumDynamicPressure,
      maximumSampleDynamicPressure >= maximumDynamicPressure,
      maximumLoad > 0, maximumSink > 0, minimumRadarAltitude >= 0,
      maximumSideslip > 0, maximumSideslip < .pi/2,
      minimumAlpha >= 0, maximumAlpha > minimumAlpha, maximumAlpha < .pi/2,
      alphaStep > 0, samplingDuration > 0, settlingDuration >= 0,
      (0...1).contains(abortStallFraction), (0...1).contains(trajectoryLearningRate) else {
      throw FlightError.invalidConfiguration("Invalid calibration limits")
    }
    _ = try JSONEncoder().encode(self)
  }
}
