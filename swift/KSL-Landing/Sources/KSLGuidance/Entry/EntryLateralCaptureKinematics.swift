import Foundation

/// Ground-track cross-axis interception with zero cross-axis velocity at the
/// capture inlet. The cubic boundary-value solution's endpoint accelerations
/// are an early *steering demand*, not a claim that an orbiter can deliver it.
/// Chart positions are metres, chart velocities metres/second, time seconds.
public enum EntryLateralCaptureKinematics {
  public static func endpointAccelerations(crossError: Double, crossVelocity: Double,
    timeToCapture: Double) -> (initial: Double, terminal: Double) {
    guard crossError.isFinite, crossVelocity.isFinite, timeToCapture.isFinite,
      timeToCapture > 0 else { return (.infinity, .infinity) }
    let t = timeToCapture
    // x(T)=xTarget and x'(T)=0 for cubic x(t).
    return (6*crossError/(t*t)-4*crossVelocity/t,
      -6*crossError/(t*t)+2*crossVelocity/t)
  }

  public static func peakDemand(crossError: Double, crossVelocity: Double,
    timeToCapture: Double) -> Double {
    let endpoints = endpointAccelerations(crossError: crossError,
      crossVelocity: crossVelocity, timeToCapture: timeToCapture)
    return max(abs(endpoints.initial), abs(endpoints.terminal))
  }
}
