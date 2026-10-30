import Foundation
import KSLCore
import KSLPhysics

public struct ShuttleSimDecodedState {
  public let state: VehicleState
  public let initialInertialPhase: Double
  public let deliveredDeltaV: Double
  public let bodyRateEvidence: EvidenceKind
  public let lowPressureControlUnsupported: Bool
  public let geometryError: Double
}

/// Independent simulator transport decoder. It contains no propagation,
/// guidance, actuator integrator, or altered simulator physics.
public final class ShuttleSimTelemetryDecoder {
  private let configuration: MissionConfiguration
  private let atmosphere: AtmosphericModelEngine
  private let coefficients: [String: Double]
  private var previous: (time: Double, attitude: Quaternion)?
  public init(configuration: MissionConfiguration, attitudeINI: String) throws {
    self.configuration = configuration
    atmosphere = try .init(planet: configuration.planet)
    var values: [String: Double] = [:]
    for line in attitudeINI.split(whereSeparator: \.isNewline) where !line.hasPrefix("#") {
      let fields = line.split(separator: "=", maxSplits: 1).map { $0.trimmingCharacters(in: .whitespaces) }
      if fields.count == 2, let number = Double(fields[1]), number.isFinite { values[fields[0]] = number }
    }
    coefficients = values
  }
  public func decode(_ data: Data) throws -> ShuttleSimDecodedState {
    guard let raw = try JSONSerialization.jsonObject(with: data) as? [String: Any],
      raw["type"] as? String == "telemetry", raw["source"] as? String == "sim" else {
      throw FlightError.protocolViolation("Expected ShuttleSim telemetry schema 1")
    }
    func group(_ key: String) throws -> [String: Any] {
      guard let d = raw[key] as? [String: Any] else { throw FlightError.protocolViolation("Missing ShuttleSim group: \(key)") }
      return d
    }
    func number(_ d: [String: Any], _ key: String) throws -> Double {
      guard let value = d[key] as? NSNumber, value.doubleValue.isFinite else {
        throw FlightError.protocolViolation("Missing/nonfinite ShuttleSim number: \(key)")
      }
      return value.doubleValue
    }
    func flag(_ d: [String: Any], _ key: String) -> Bool { d[key] as? Bool ?? false }
    func vector(_ d: [String: Any], _ keys: [String]) throws -> Vector3 {
      try .init(number(d, keys[0]), number(d, keys[1]), number(d, keys[2]))
    }
    let world = try group("world"), p = try group("position"), v = try group("velocity")
    let a = try group("attitude"), env = try group("atmosphere"), f = try group("aero")
    let ground = try group("ground"), vehicle = try group("vehicle"), runway = try group("runway")
    let c = configuration, planet = c.planet, time = try number(raw, "ut")
    guard abs(try number(world, "radius_m")-planet.radius) < 0.1,
      abs(try number(world, "mu_m3_s2")/planet.gravitationalParameter-1) < 1e-8,
      abs(try number(world, "rotation_rate_rad_s")-planet.rotationRate) < 1e-12 else {
      throw FlightError.invalidConfiguration("ShuttleSim world does not match the native mission")
    }
    let phase = try number(world, "rotation_phase_rad_at_ut0")
    let z = Vector3(0, 0, 1), theta = phase+planet.rotationRate*time
    let position = try vector(p, ["x", "y", "z"]).rotated(about: z, angle: -theta)
    let velocity = try vector(v, ["x_mps", "y_mps", "z_mps"]).rotated(about: z, angle: -theta)
      - planet.angularVelocity.cross(position)
    let qi = try Quaternion(w: number(a,"q_w"), x: number(a,"q_x"), y: number(a,"q_y"), z: number(a,"q_z"))
    guard abs(qi.norm-1) < 1e-5 else { throw FlightError.invalidTelemetry("ShuttleSim nonunit attitude") }
    let attitude: Quaternion
    if flag(a,"body_attitude_available") {
      attitude = (Quaternion.axisAngle(z, angle: -theta)*qi).normalized
    } else {
      guard let reconstructed = try AerodynamicOrientation.reconstruct(position: position, velocity: velocity,
        alpha: radians(number(a,"aoa_deg")), beta: radians(number(a,"sideslip_deg")), bank: radians(number(a,"bank_deg"))) else {
        throw FlightError.invalidTelemetry("Independent aerodynamic attitude reconstruction is singular")
      }
      attitude = reconstructed
    }
    let geodetic = try GeodeticPosition(latitude: radians(number(p,"lat_deg")),
      longitude: radians(number(p,"lon_deg")), altitude: number(p,"altitude_m")).cartesian(radius: planet.radius)
    let geometryError = (position-geodetic).length
    guard geometryError < 0.02,
      abs(velocity.length - (try number(v,"surface_mps"))) < 0.02 else {
      throw FlightError.invalidTelemetry("ShuttleSim inertial/fixed/UT conversion failed its independent geodetic or speed check")
    }
    let frame = RunwayFrame(runway: c.runway, planet: planet)
    // ShuttleSim reports great-circle track coordinates. Native planning uses
    // an azimuthal-equidistant chart; the two are NOT interchangeable far from
    // the runway. Verify the simulator definition independently in Cartesian
    // geometry, while retaining the native chart for the native controller.
    let radial = position.normalized
    let simAlong = planet.radius*atan2(radial.dot(frame.alongAxis), radial.dot(frame.up))
    let simCross = planet.radius*asin(clamp(radial.dot(frame.rightAxis), -1, 1))
    guard abs(simAlong - (try number(runway,"along_m"))) < 0.1,
      abs(simCross - (try number(runway,"cross_m"))) < 0.1 else {
      throw FlightError.invalidConfiguration("Native and ShuttleSim runway geometry disagree")
    }
    var rates = Vector3.zero
    let rateEvidence: EvidenceKind
    if flag(a,"body_rates_available") {
      rates = try vector(a,["body_roll_rate_deg_s", "body_pitch_rate_deg_s", "body_yaw_rate_deg_s"]) * (.pi/180)
      // Body quaternion is inertial in ShuttleSim; native VehicleState rates
      // are relative to the rotating body-fixed frame.
      rates = rates-attitude.conjugate.rotate(planet.angularVelocity)
      rateEvidence = .provisional
    } else if let previous, time > previous.time, time-previous.time <= 1 {
      var delta = (previous.attitude.conjugate*attitude).normalized
      if delta.w < 0 { delta = .init(w: -delta.w, x: -delta.x, y: -delta.y, z: -delta.z) }
      let axis = Vector3(delta.x, delta.y, delta.z), sine = axis.length
      if sine > 1e-12 { rates = axis*(2*atan2(sine, delta.w)/(sine*(time-previous.time))) }
      rateEvidence = .identified
    } else { rateEvidence = .unavailable }
    previous = (time, attitude)
    var air = atmosphere.sample(position: position, time: time)
    let pressure = try number(env,"pressure_pa"), density = try number(env,"density_kg_m3")
    let sound = try number(env,"speed_of_sound_mps"), temperature = try number(env,"temperature_k")
    air.pressure = pressure; air.density = density; air.evidence = .provisional
    // Vacuum has no sound-speed or temperature observation in ShuttleSim.
    // Keep only the explicitly provisional atmospheric model's conventions.
    if sound > 0 { air.speedOfSound = sound }
    if temperature > 0 { air.temperature = temperature }
    var aero = AerodynamicState()
    aero.airspeed = try number(v,"air_mps"); aero.mach = try number(f,"mach")
    aero.dynamicPressure = try number(f,"q_pa")
    aero.angleOfAttack = try radians(number(a,"aoa_deg")); aero.bank = try radians(number(a,"bank_deg"))
    aero.sideslip = try radians(number(a,"sideslip_deg"))
    aero.lift = try number(f,"lift_n"); aero.drag = try number(f,"drag_n")
    let forward = velocity.normalized, right = forward.cross(position.normalized).normalized
    let up = right.cross(forward).normalized
    aero.force = (up*cos(aero.bank)+right*sin(aero.bank))*aero.lift-forward*aero.drag
    aero.measuredForces = false; aero.modelEvidence = .provisional
    var systems = SystemState()
    systems.gearDown = flag(ground,"gear_down"); systems.brakes = flag(ground,"brakes")
    systems.airbrakes = flag(ground,"airbrakes")
    systems.throttle = try number(vehicle,"throttle"); systems.thrust = try number(vehicle,"current_thrust_n")
    let main = flag(ground,"main_contact"), nose = flag(ground,"nose_contact")
    let contact: ContactState = flag(ground,"crashed") ? .crashed
      : main ? (nose ? (flag(ground,"stopped") ? .stopped : .rolling) : .mainGear) : .airborne
    var state = VehicleState(time: time, position: position, velocity: velocity, attitude: attitude,
      angularVelocity: rates, mass: try number(vehicle,"mass_kg"), atmosphere: air, aerodynamics: aero,
      contact: contact, systems: systems, radarAltitude: max(0, try number(runway,"vertical_m")),
      wheelContact: .init(mainGear: main, noseGear: nose, wheelCount: 3, evidence: .provisional),
      appliedControls: try .init(pitch: number(a,"input_pitch"), roll: number(a,"input_roll"),
        yaw: number(a,"input_yaw"), gearDown: systems.gearDown, wheelBrake: systems.brakes ? 1 : 0))
    // These are the independent attitude.c defaults, overridden by the actual
    // INI used by the runner. They are a provisional simulator plant, NOT
    // measured vessel torque or a payload inertia identification.
    let kPa = aero.dynamicPressure/1000
    let lowQ = aero.dynamicPressure < (coefficients["direct_min_q_pa"] ?? 500)
    func acceleration(_ axis: String, _ per: Double, _ limit: Double) -> Double {
      lowQ ? 0 : min(radians(coefficients["direct_"+axis+"_accel_per_kpa_deg"] ?? per)*kPa,
                     radians(coefficients["direct_"+axis+"_accel_max_deg"] ?? limit))
    }
    let response = Vector3(acceleration("roll",6,60),acceleration("pitch",4,40),acceleration("yaw",3,30))
    let inertia = c.vehicle.inertia
    let torque = response.multiplied(componentwise: inertia)
    state.controlAuthority = .init(positiveTorque: torque, negativeTorque: -torque,
      inertia: inertia, evidence: .provisional)
    state.controlAuthority?.actuatorSources = "ShuttleSim INI direct angular-response surrogate; not measured inertia/torque"
    // ShuttleSim does not supply a part inventory, structural CoM or tensor.
    // Leave vesselConfiguration nil rather than fabricating payload telemetry.
    try state.validate(configuration: c)
    return .init(state: state, initialInertialPhase: phase,
      deliveredDeltaV: try number(vehicle,"engine_dv_mps"), bodyRateEvidence: rateEvidence,
      lowPressureControlUnsupported: lowQ, geometryError: geometryError)
  }
}
