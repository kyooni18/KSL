import Foundation
import KSLCore
import KSLPhysics

public final class PathTrackingEngine {
  private let configuration: MissionConfiguration
  private let frame: RunwayFrame
  private let aero: AerodynamicsEngine
  private let rotation: RotationalDynamicsEngine
  private let speedbrake: TAEMSpeedbrakeController
  public private(set) var index = 0
  public private(set) var trackingError = 0.0
  public private(set) var reference: PathStation?
  public private(set) var headingError = 0.0
  public private(set) var lateralAccelerationDemand = 0.0
  public private(set) var allocatedLateralAcceleration = 0.0
  public private(set) var lookaheadDistance = 0.0
  public private(set) var lookaheadHeadingError = 0.0
  public init(configuration: MissionConfiguration, atmosphere: AtmosphericModelEngine,
              aerodynamics: AerodynamicsEngine, initialIndex: Int = 0) {
    self.configuration = configuration
    frame = .init(runway: configuration.runway, planet: configuration.planet)
    aero = aerodynamics
    rotation = .init(configuration: configuration.vehicle, planet: configuration.planet)
    speedbrake = .init(configuration: configuration, atmosphere: atmosphere)
    index = max(0, initialIndex)
  }
  public func reset() {
    index = 0
    trackingError = 0; reference = nil; headingError = 0
    lateralAccelerationDemand = 0; allocatedLateralAcceleration = 0
    lookaheadDistance = 0; lookaheadHeadingError = 0
  }

  /// Advance along this winding only. A quarter-turn of accumulated tangent
  /// change keeps the preview on the local arc even when a large response
  /// distance would otherwise look through a complete loop of advisory geometry.
  private func lookahead(from reference: PathStation, plan: TerminalPlan, distance: Double) -> PathStation {
    let end = min(plan.length, reference.distance + max(0, distance))
    var previous = reference, turn = 0.0
    for i in (index + 1)..<plan.stations.count {
      guard plan.stations[i].distance > previous.distance else { continue }
      let next = plan.station(at: min(end, plan.stations[i].distance))
      let change = abs(atan2(previous.tangent.cross(next.tangent), previous.tangent.dot(next.tangent)))
      if turn + change > .pi / 2 {
        let fraction = (.pi / 2 - turn) / change
        return plan.station(at: previous.distance + (next.distance - previous.distance) * fraction)
      }
      turn += change
      if next.distance >= end { return next }
      previous = next
    }
    return previous
  }

  /// Shared lateral geometry for MM304's advisory steering and MM305 control.
  /// Walk contiguous segments; a nearby later winding is not an acquired path.
  public func lateralDemand(_ state: VehicleState, plan: TerminalPlan,
                            rollResponseSeconds: Double? = nil) -> (acceleration: Double, reference: PathStation) {
    let p = frame.coordinates(of: state.position)
    guard !plan.stations.isEmpty else {
      // Invalid external/legacy geometry must not crash the active owner.
      // Return an unpriced local reference and retain bounded support recovery.
      let velocity = frame.coordinateVelocity(at: state.position, velocity: state.velocity)
      let tangent = velocity.length > 0 && velocity.length.isFinite ? velocity.normalized : Vector2(1, 0)
      let ref = PathStation(distance: 0, position: p.horizontal, tangent: tangent,
        curvature: 0, height: p.height, slope: tan(state.flightPathAngle), segment: .acquisition)
      index = 0; reference = ref; trackingError = 0; headingError = 0; lateralAccelerationDemand = 0
      lookaheadDistance = 0; lookaheadHeadingError = 0
      return (0, ref)
    }
    index = min(max(0, index), max(0, plan.stations.count - 2))
    while index + 2 < plan.stations.count {
      let a = plan.stations[index], b = plan.stations[index + 1]
      let segment = b.position - a.position
      guard (p.horizontal - b.position).dot(segment) >= 0 else { break }
      let next = plan.stations[index + 2].position - b.position
      let denominator = next.dot(next)
      let u = denominator > 0 ? clamp((p.horizontal - b.position).dot(next) / denominator, 0, 1) : 0
      let nextDistance = (p.horizontal - (b.position + next * u)).length
      if nextDistance > (p.horizontal - b.position).length { break }
      index += 1
    }
    let a = plan.stations[index]
    let b = plan.stations[min(index + 1, plan.stations.count - 1)]
    let segment = b.position - a.position
    let denominator = segment.dot(segment)
    let u = denominator > 0 ? clamp((p.horizontal - a.position).dot(segment) / denominator, 0, 1) : 0
    let ref = plan.station(at: a.distance + (b.distance - a.distance) * u)
    reference = ref
    let offset = p.horizontal - ref.position
    trackingError = offset.length
    // Compare directions at the vehicle, not two different tangent planes.
    let tangent = frame.direction(at: p.horizontal, tangent: ref.tangent)
    headingError = wrapAngle(state.localFrame.heading(of: tangent) - state.course)
    let crossError = -offset.dot(ref.tangent.rightNormal)
    let vh = max(1, state.horizontalSpeed)
    let separation = 1 / (1 - GuidanceTargetLimiter.responseReserve)
    let rollFloor = (rollResponseSeconds ?? 0) * separation
    let timeConstant = max(rollFloor, ref.segment == .spiral && abs(ref.curvature) > 1e-8
      ? max(configuration.guidance.pathLookaheadTime,
          0.65 * (configuration.guidance.hacLookaheadAngle ?? radians(16)) / (abs(ref.curvature) * vh))
      : configuration.guidance.pathLookaheadTime)
    // A regenerated C2 inlet matches measured position, course and curvature.
    // Tracking only that station initially reproduces the current turn on every
    // replan and does not anticipate the bend being requested downstream.
    // Park/Deyst/How nonlinear lookahead: a_lat = 2 Vh² sin(eta) / L1.
    // Do not add local curvature again; the preview already supplies it.
    let chartSpeed = frame.coordinateVelocity(at: state.position, velocity: state.velocity).length
    let preview = lookahead(from: ref, plan: plan,
      distance: (chartSpeed.isFinite ? chartSpeed : vh) * timeConstant)
    let aim = frame.position(along: preview.position.x, cross: preview.position.y, height: p.height)
    let direction = (aim - state.position).projected(onPlaneWithNormal: state.position.normalized)
    lookaheadDistance = direction.length
    var error = lookaheadDistance > 0
      ? wrapAngle(state.localFrame.heading(of: direction) - state.course) : headingError
    // At exactly reciprocal heading either turn is geometrically equivalent;
    // retain the current turn direction rather than toggling on roundoff.
    if abs(abs(error) - .pi) <= sqrt(Double.ulpOfOne) {
      error = state.aerodynamics.bank < 0 ? -.pi : .pi
    }
    lookaheadHeadingError = error
    let captureSine = sin(clamp(error, -.pi / 2, .pi / 2))
    if lookaheadDistance > 0 {
      // One travelled control interval is a numerical endpoint guard, not an
      // aerodynamic radius or a claim that the demanded force is available.
      lateralAccelerationDemand = 2 * vh * vh * captureSine
        / max(vh * configuration.guidance.controlInterval, lookaheadDistance)
    } else {
      // No forward path remains: keep its final tangent and cross-track
      // recovery defined. Actual final capture still has its separate gate.
      lateralAccelerationDemand = vh * vh * ref.curvature
        + 2 * 0.9 / timeConstant * vh * captureSine
        + crossError / (timeConstant * timeConstant)
    }
    return (lateralAccelerationDemand, ref)
  }

  public func update(_ state: VehicleState, plan: TerminalPlan, liftEffectiveness: Double = 1,
                     identifiedTrimCeiling: Double? = nil, rollResponseSeconds: Double? = nil,
                     pitchResponseSeconds: Double? = nil, referenceDynamicsCurrent: Bool = true) -> (GuidanceTarget, PathStation) {
    let (lateral, ref) = lateralDemand(state, plan: plan, rollResponseSeconds: rollResponseSeconds)
    let p = frame.coordinates(of: state.position)
    let a = state.aerodynamics
    let speed = state.aerodynamics.airspeed
    let g = configuration.planet.gravitationalParameter / state.position.squaredLength
    let gamma = atan(ref.slope)
    let pitchFloor = (pitchResponseSeconds ?? 0) / (1 - GuidanceTargetLimiter.responseReserve)
    // An unpriced geometric tail has no predicted speed to exchange for height.
    let priced = referenceDynamicsCurrent && ref.speed > 0
      && ref.distance <= (plan.assessment.predictedThroughDistance ?? plan.length)
    // Convert excess kinetic energy to height when fast. When slow, the only
    // available correction is to lower the reference and recover airspeed;
    // raising it would spend kinetic energy that the vehicle does not have.
    let exchange = ref.segment != .final && priced
      ? min(0, (ref.speed * ref.speed - speed * speed) / (2 * g)) : 0
    // Below the configured minimum safe airspeed, trade height for the
    // missing kinetic energy rather than holding a reference height.
    let safeSpeed = configuration.vehicle.minimumSafeSpeed ?? configuration.vehicle.landingSpeed
    let deficit = ref.segment != .final ? max(0, safeSpeed * safeSpeed - speed * speed) / (2 * g) : 0
    let heightTarget = ref.height - exchange - deficit
    let rateTime = max(5, pitchFloor), heightTime = rateTime * 12 / 5
    let desiredVertical = speed * sin(gamma) + clamp((heightTarget - p.height) / heightTime, -35, 35)
    let desiredGamma = asin(clamp(desiredVertical / max(1, speed), -1, 1))
    let support = max(0, (g - speed * speed / state.position.length) * cos(state.flightPathAngle))
    let vertical = support + speed * wrapAngle(desiredGamma - state.flightPathAngle) / rateTime
      + speed * speed * ref.verticalCurvature
    let maxAlpha = rotation.supportedTrimAlpha(state, identifiedCeiling: identifiedTrimCeiling)
    let scale = max(Double.leastNormalMagnitude, liftEffectiveness)
    let ceiling = speed < 1.15*safeSpeed
      ? min(maxAlpha, aero.bestGlideAlpha(mach: state.aerodynamics.mach, maximumAlpha: maxAlpha,
          dynamicPressure: state.aerodynamics.dynamicPressure, gear: state.systems.gearDown)) : maxAlpha
    let fitted = try? TAEMNormalForceFitter.fit(vertical: vertical,
      lateral: state.aerodynamics.dynamicPressure > configuration.vehicle.maximumDynamicPressure ? 0 : lateral,
      state: state, configuration: configuration, aerodynamics: aero.scaled(lift: scale, drag: 1),
      maximumAlpha: ceiling, baseTarget: .init())
    allocatedLateralAcceleration = fitted?.modeled.y ?? 0
    let alpha = fitted?.target.angleOfAttack ?? ceiling
    let bank = fitted?.target.bank ?? 0
    let brake: Double
    if ref.segment != .final, priced, let nepDistance = plan.nominalEnergyStation?.distance {
      let existing = a.dynamicPressure * (configuration.vehicle.speedbrakeFullDeploymentDragArea ?? 0)
        * (state.systems.speedbrakeFraction ?? 0)
      let cleanDrag = a.measuredForces ? max(0, a.drag-existing)/state.mass
        : a.dynamicPressure*aero.referenceArea*aero.coefficients(mach: a.mach,
          alpha: a.angleOfAttack, dynamicPressure: a.dynamicPressure, beta: a.sideslip,
          gear: state.systems.gearDown).drag/state.mass
      brake = speedbrake.fraction(speed: speed, position: state.position, time: state.time,
        mass: state.mass, remainingDistance: nepDistance-ref.distance,
        target: plan.nominalEnergyPoint, cleanDragAcceleration: cleanDrag)
    } else { brake = 0 }
    return (.init(angleOfAttack: alpha, bank: bank,
      desiredSpeed: priced ? max(ref.speed, ref.segment != .final ? safeSpeed : 0) : safeSpeed,
      desiredHeight: heightTarget, desiredFlightPath: desiredGamma,
      gearDown: state.systems.gearDown || p.height <= (configuration.guidance.gearDeploymentAltitude ?? 1500),
      speedbrakeFraction: brake), ref)
  }
}
