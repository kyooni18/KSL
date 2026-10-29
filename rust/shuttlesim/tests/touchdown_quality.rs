//! Closed-loop Final approach to touchdown: production Final guidance and the
//! direct flight-control law fly the simulator onto the runway inside the
//! mission's touchdown quality limits.

use core::ffi::{c_char, CStr};
use landing::app::controller::{
    AerodynamicEnvelope, AerodynamicModel, DeorbitPlan, GuidanceCommand, GuidanceMachine, GuidanceResult, LandingConfiguration,
    PlanetModel, ShuttleSimModelFile, Telemetry, Vector3, VehicleProfile, VehicleState, VesselAeroSample, VesselPhysicsModel,
    SHUTTLE_SIM_MODEL_AERO, SHUTTLE_SIM_MODEL_ATMOSPHERE, SHUTTLE_SIM_MODEL_ATTITUDE, SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR,
    SHUTTLE_SIM_MODEL_FORCE_BOOK,
};
use landing::control::flight_control::{FlightControlOutput, FlightControlState};
use shuttlesim::ffi::*;
use shuttlesim::protocol::SimCommand;
use shuttlesim::scenario::Scenario;
use shuttlesim::sim::Simulation;
use std::ffi::CString;

#[allow(clashing_extern_declarations)]
extern "C" {
    fn landing_configuration_default() -> LandingConfiguration;
    fn landing_configuration_normalize(c: *mut LandingConfiguration);
    fn shuttle_sim_model_path(root: *const c_char, kind: ShuttleSimModelFile, buffer: *mut c_char, size: usize) -> *const c_char;
    fn shuttle_sim_load_planet(path: *const c_char, planet: *mut PlanetModel, error: *mut c_char, size: usize) -> bool;
    fn shuttle_sim_decode_telemetry(
        packet: *const c_char,
        planet: *const PlanetModel,
        previous: *const Telemetry,
        t: *mut Telemetry,
        state: *mut VehicleState,
        error: *mut c_char,
        size: usize,
    ) -> bool;
    fn shuttle_sim_prepare_guidance_telemetry(t: *mut Telemetry, cfg: *const LandingConfiguration, planet: *const PlanetModel);
    fn vessel_physics_init(m: *mut VesselPhysicsModel);
    fn vessel_physics_import_sample(m: *mut VesselPhysicsModel, s: *const VesselAeroSample);
    fn vessel_physics_derive_envelope(
        m: *mut VesselPhysicsModel,
        baseline: *const VehicleProfile,
        envelope: *mut AerodynamicEnvelope,
        planning: *mut AerodynamicModel,
    ) -> bool;
    fn guidance_machine_init(g: *mut GuidanceMachine);
    fn guidance_set_engaged(g: *mut GuidanceMachine, engaged: bool);
    fn surface_course(pos: Vector3, vel: Vector3, omega: Vector3, north: Vector3, fallback: f64) -> f64;
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
    fn guidance_begin_final_test(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        course: f64,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        message: *mut c_char,
        size: usize,
    ) -> bool;
    fn guidance_update(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        plan: *const DeorbitPlan,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
    ) -> GuidanceResult;
    fn guidance_result_clear(r: *mut GuidanceResult);
    fn flight_control_init(s: *mut FlightControlState, nominal_dt: f64);
    fn flight_control_step(
        s: *mut FlightControlState,
        t: *const Telemetry,
        c: *const GuidanceCommand,
        dt: f64,
        out: *mut FlightControlOutput,
    ) -> bool;
}

fn model_path(kind: ShuttleSimModelFile) -> CString {
    let mut buf = [0 as c_char; 512];
    unsafe { shuttle_sim_model_path(core::ptr::null(), kind, buf.as_mut_ptr(), buf.len()) };
    unsafe { CStr::from_ptr(buf.as_ptr()) }.to_owned()
}

/// The certified prior force book, as a vessel-physics model sees it.
fn load_force_book(path: &str, model: &mut VesselPhysicsModel) {
    let Ok(text) = std::fs::read_to_string(path) else { return };
    for line in text.lines().filter(|l| !l.contains("q_pa")) {
        let f: Vec<f64> = line.split(',').map_while(|v| v.trim().parse().ok()).collect();
        if f.len() < 5 || !(f[0] > 1.0) || !f[..5].iter().all(|v| v.is_finite()) {
            continue;
        }
        let observations = (if f.len() >= 6 { f[5] } else { 1.0 }).round().max(1.0) as u32;
        let sample = VesselAeroSample {
            q: f[0],
            mach: f[1],
            aoa: f[2],
            beta: 0.0,
            mass: 40252.91796875,
            force_per_q: Vector3 { x: f[4].max(0.0), y: f[3].max(0.0), z: 0.0 },
            gear: false,
            brakes: false,
            airbrakes: 0,
            observations,
            trust: (0.72 + 0.035 * observations as f64).clamp(0.72, 1.0),
            last_observation_ut: 0.0,
        };
        unsafe { vessel_physics_import_sample(model, &sample) };
    }
}

fn envelope_at_mach(e: &AerodynamicEnvelope, mach: f64) -> AerodynamicModel {
    const ANCHOR: [f64; 4] = [0.35, 1.05, 2.6, 6.0];
    let x = mach.max(0.0);
    if x <= ANCHOR[0] {
        return e.regimes[0];
    }
    for i in 0..3 {
        if x <= ANCHOR[i + 1] {
            let f = (x - ANCHOR[i]) / (ANCHOR[i + 1] - ANCHOR[i]).max(1e-9);
            let (a, b) = (e.regimes[i], e.regimes[i + 1]);
            return AerodynamicModel {
                lift_to_drag: a.lift_to_drag + (b.lift_to_drag - a.lift_to_drag) * f,
                ballistic_coefficient: a.ballistic_coefficient + (b.ballistic_coefficient - a.ballistic_coefficient) * f,
                confidence: a.confidence + (b.confidence - a.confidence) * f,
            };
        }
    }
    e.regimes[3]
}

fn with_aero(t: &mut Telemetry, a: AerodynamicModel) {
    t.estimated_lift_to_drag = a.lift_to_drag;
    t.estimated_ballistic_coefficient = a.ballistic_coefficient;
    t.aerodynamic_confidence = a.confidence;
}

#[test]
fn final_approach_touches_down_inside_quality_limits() {
    std::env::set_current_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).unwrap();
    std::env::set_var("KSP_LANDER_FINAL_APPROACH_TEST", "1");
    let sc = Scenario::load("ShuttleSim/scenarios/final-mm305-ideal-3p5km-185mps-aoa3.ini").expect("scenario");
    let mut sim = Simulation::new(&sc);
    let atmosphere = model_path(SHUTTLE_SIM_MODEL_ATMOSPHERE);
    unsafe {
        assert!(world_load_atmosphere_csv(&mut sim.world, atmosphere.as_ptr()));
        assert!(aero_load_csv(&mut sim.aero, model_path(SHUTTLE_SIM_MODEL_AERO).as_ptr()));
        aero_load_book_csv(&mut sim.aero, model_path(SHUTTLE_SIM_MODEL_FORCE_BOOK).as_ptr());
        assert!(attitude_load_ini(&mut sim.state.attitude, model_path(SHUTTLE_SIM_MODEL_ATTITUDE).as_ptr()));
        let s = &mut sim.state;
        s.aero = aero_compute(&sim.world, &sim.aero, s.position_i_m, s.velocity_i_mps, s.ut, s.mass_kg, s.attitude.aoa_rad, s.attitude.bank_rad);
    }

    // SAFETY (all zeroed values below): plain C data whose all-zero state is
    // valid and is fully initialised by the init calls that follow.
    let mut cfg = unsafe { landing_configuration_default() };
    unsafe { landing_configuration_normalize(&mut cfg) };
    cfg.site.latitude = sc.runway_latitude_deg;
    cfg.site.longitude = sc.runway_longitude_deg;
    cfg.site.altitude = sc.runway_elevation_m;
    cfg.site.runway_heading = sc.runway_heading_deg;
    cfg.site.runway_length = sc.runway_length_m;
    cfg.site.runway_width = sc.runway_width_m;
    let mut planet: Box<PlanetModel> = unsafe { Box::new_zeroed().assume_init() };
    let mut error = [0 as c_char; 256];
    assert!(unsafe { shuttle_sim_load_planet(atmosphere.as_ptr(), &mut *planet, error.as_mut_ptr(), error.len()) });
    let mut physics: Box<VesselPhysicsModel> = unsafe { Box::new_zeroed().assume_init() };
    unsafe { vessel_physics_init(&mut *physics) };
    load_force_book(model_path(SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR).to_str().unwrap(), &mut physics);
    let mut env: AerodynamicEnvelope = unsafe { core::mem::zeroed() };
    let mut plan_aero: AerodynamicModel = unsafe { core::mem::zeroed() };
    assert!(unsafe { vessel_physics_derive_envelope(&mut *physics, &cfg.vehicle, &mut env, &mut plan_aero) });

    let decode = |sim: &Simulation, previous: Option<&Telemetry>| -> (Telemetry, VehicleState) {
        let json = CString::new(sim.telemetry_json(50.0)).unwrap();
        let mut t: Telemetry = unsafe { core::mem::zeroed() };
        let mut v: VehicleState = unsafe { core::mem::zeroed() };
        let mut error = [0 as c_char; 256];
        let prev = previous.map_or(core::ptr::null(), |p| p as *const Telemetry);
        assert!(unsafe { shuttle_sim_decode_telemetry(json.as_ptr(), &*planet, prev, &mut t, &mut v, error.as_mut_ptr(), error.len()) });
        unsafe { shuttle_sim_prepare_guidance_telemetry(&mut t, &cfg, &*planet) };
        let aero = envelope_at_mach(&env, t.mach);
        with_aero(&mut t, aero);
        (t, v)
    };

    let (t0, v0) = decode(&sim, None);
    let mut g: Box<GuidanceMachine> = unsafe { Box::new_zeroed().assume_init() };
    unsafe {
        guidance_machine_init(&mut *g);
        guidance_set_engaged(&mut *g, true);
        let course0 = surface_course(v0.position, v0.velocity, planet_rotation_vector(&*planet), planet.north_axis, t0.heading);
        let mut message = [0 as c_char; 256];
        let begun = guidance_begin_final_test(
            &mut *g,
            &t0,
            course0,
            &*planet,
            envelope_at_mach(&env, t0.mach),
            &cfg,
            message.as_mut_ptr(),
            message.len(),
        );
        assert!(begun, "guidance_begin_final_test: {}", CStr::from_ptr(message.as_ptr()).to_string_lossy());
    }
    let mut fcs: FlightControlState = unsafe { core::mem::zeroed() };
    unsafe { flight_control_init(&mut fcs, 0.02) };

    let dt = 0.02;
    let mut previous: Option<Telemetry> = None;
    for _ in 0..2500 {
        if sim.state.on_ground {
            break;
        }
        let (t, v) = decode(&sim, previous.as_ref());
        let aero = envelope_at_mach(&env, t.mach);
        let mut r = unsafe { guidance_update(&mut *g, &t, &v, core::ptr::null(), &*planet, aero, &cfg) };
        let mut out: FlightControlOutput = unsafe { core::mem::zeroed() };
        assert!(unsafe { flight_control_step(&mut fcs, &t, &r.command, dt, &mut out) }, "flight control step failed");
        unsafe { guidance_result_clear(&mut r) };
        sim.apply_command(&SimCommand {
            has_gear: true,
            gear_down: out.gear,
            has_brakes: true,
            brakes: out.brakes,
            has_airbrakes: true,
            airbrakes: out.airbrakes,
            has_wheel_steering: true,
            wheel_steering: out.wheel_steering,
            has_inputs: true,
            pitch_input: out.pitch,
            roll_input: out.roll,
            yaw_input: out.yaw,
            ..Default::default()
        });
        sim.step(dt);
        previous = Some(t);
    }

    let s = sim.state;
    assert!(s.touchdown_seen && s.touchdown_gear, "no gear touchdown");
    let report = format!(
        "sink {:.2} m/s, speed {:.2} m/s, along {:.1} m, cross {:.1} m, on runway {}",
        s.touchdown_sink_mps, s.touchdown_speed_mps, s.touchdown_along_m, s.touchdown_cross_m, s.on_runway_at_touchdown
    );
    // Mission limits: sink <= 3 m/s, 60-75 m/s, on the 2.5 km runway, within 35 m.
    assert!(s.touchdown_sink_mps <= 3.0, "{report}");
    assert!((60.0..=75.0).contains(&s.touchdown_speed_mps), "{report}");
    assert!((0.0..=2500.0).contains(&s.touchdown_along_m), "{report}");
    assert!(s.touchdown_cross_m.abs() <= 35.0 && s.on_runway_at_touchdown, "{report}");
}
