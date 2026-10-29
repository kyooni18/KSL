//! Simulator-only deterministic teacher for `ShuttleSim/rl` (ctypes). The
//! production guidance flies the offline simulator's telemetry; the result is
//! the commanded attitude plus the hard physical gates that bound a learned
//! residual. It never drives actuators or opens sockets.

use core::ffi::{c_char, c_int, c_uint, c_void, CStr};
use landing::app::controller::{
    AerodynamicEnvelope, AerodynamicModel, DeorbitPlan, GuidanceMachine, GuidancePhase, GuidanceResult, GuidanceSettings, JsonWriter,
    LandingConfiguration, LandingSite, PlanetModel, TaemInterfaceTarget, Telemetry, TerminalModel, TerminalModelSourceFiles,
    TrajectoryCalibrationModel, VehicleProfile, VehicleState, PHASE_ABORT, PHASE_ATTITUDE_RECOVERY, PHASE_ENTRY_ENERGY, PHASE_FAULT,
    PHASE_TAEM, TERMINAL_PATH_HAC, TERMINAL_PATH_NONE,
};
use landing::prediction::decision_envelope::{ControlAuthorityEnvelope, TaemInterfaceCapture};
use landing::vehicle::math::EntryTerminalDemand;
use landing::vehicle::models::{JsonDoc, JsonToken};
use std::ffi::CString;
use std::fmt::Write;

#[allow(clashing_extern_declarations)]
extern "C" {
    fn landing_configuration_default() -> LandingConfiguration;
    fn landing_configuration_normalize(c: *mut LandingConfiguration);
    fn landing_configuration_from_json(c: *mut LandingConfiguration, d: *const JsonDoc, object: c_int) -> bool;
    fn json_parse(text: *const c_char, tokens: *mut JsonToken, capacity: c_int, doc: *mut JsonDoc) -> c_int;
    fn shuttle_sim_load_planet(path: *const c_char, planet: *mut PlanetModel, error: *mut c_char, error_size: usize) -> bool;
    fn shuttle_sim_decode_telemetry(
        packet: *const c_char,
        planet: *const PlanetModel,
        previous: *const Telemetry,
        t: *mut Telemetry,
        state: *mut VehicleState,
        error: *mut c_char,
        error_size: usize,
    ) -> bool;
    fn shuttle_sim_prepare_guidance_telemetry(t: *mut Telemetry, cfg: *const LandingConfiguration, planet: *const PlanetModel);
    fn terminal_model_capture(
        m: *mut TerminalModel,
        files: *const TerminalModelSourceFiles,
        cfg: *const LandingConfiguration,
        snapshot_id: u64,
        reason: *mut c_char,
        reason_size: usize,
    ) -> bool;
    fn guidance_machine_init(g: *mut GuidanceMachine);
    fn guidance_initialize_reentry_continuation(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        p: *const PlanetModel,
        cfg: *const LandingConfiguration,
        initial_s_turn_sign: f64,
        late_terminal_test: bool,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
    );
    fn guidance_update_with_terminal_model(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        plan: *const DeorbitPlan,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        terminal_model: *const TerminalModel,
    ) -> GuidanceResult;
    fn guidance_result_clear(r: *mut GuidanceResult);
    fn entry_terminal_demand(
        latitude: f64,
        altitude: f64,
        range: f64,
        horizontal_speed: f64,
        course_error: f64,
        vertical_speed: f64,
        true_air_speed: f64,
        entry_reference_speed: f64,
        p: *const PlanetModel,
        v: *const VehicleProfile,
        site: *const LandingSite,
        s: *const GuidanceSettings,
    ) -> EntryTerminalDemand;
    fn vessel_physics_conservative_stall_fraction(tas: f64, aoa: f64, q: f64, v: *const VehicleProfile) -> f64;
    fn entry_bank_authority_limit(tas: f64, q: f64, g_force: f64, v: *const VehicleProfile, maximum_bank: f64) -> f64;
    fn entry_taem_interface_capture(
        q: *const TaemInterfaceTarget,
        t: *const Telemetry,
        course: f64,
        p: *const PlanetModel,
        cfg: *const LandingConfiguration,
    ) -> TaemInterfaceCapture;
    fn entry_low_q_protective_aoa_floor(q: f64, v: *const VehicleProfile) -> f64;
    fn entry_terminal_turn_aoa_floor(tas: f64, q: f64, v: *const VehicleProfile) -> f64;
    fn decision_control_authority_envelope(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        p: *const PlanetModel,
        cfg: *const LandingConfiguration,
    ) -> ControlAuthorityEnvelope;
    fn jw_init(w: *mut JsonWriter);
    fn jw_free(w: *mut JsonWriter);
    fn jw_raw(w: *mut JsonWriter, text: *const c_char);
    fn jw_string(w: *mut JsonWriter, text: *const c_char);
    fn jw_number(w: *mut JsonWriter, value: f64);
    fn jw_bool(w: *mut JsonWriter, value: bool);
}

const PHASE_NAMES: [&str; 19] = [
    "IDLE",
    "PLANNING",
    "CALIBRATION",
    "COAST",
    "BURN_SETUP",
    "DEORBIT_BURN",
    "ENTRY_INTERFACE",
    "ENTRY_ENERGY",
    "TAEM",
    "HEADING_ALIGNMENT",
    "FINAL",
    "FLARE",
    "TOUCHDOWN",
    "ROLLOUT",
    "COMPLETE",
    "PAUSED",
    "ABORT",
    "FAULT",
    "ATTITUDE_RECOVERY",
];
const RESULT_CAPACITY: usize = 8192;

pub struct OfflineExpert {
    cfg: LandingConfiguration,
    planet: PlanetModel,
    aero: AerodynamicModel,
    envelope: AerodynamicEnvelope,
    calibration: TrajectoryCalibrationModel,
    plan: DeorbitPlan,
    g: GuidanceMachine,
    terminal_model: TerminalModel,
    initialized: bool,
    have_prev: bool,
    previous: Telemetry,
    /// `None` when zeroed; holds the last command record.
    result: Option<CString>,
}

unsafe fn load_config(path: &str, c: &mut LandingConfiguration) -> bool {
    *c = landing_configuration_default();
    let Ok(text) = std::fs::read(path) else {
        landing_configuration_normalize(c);
        return false;
    };
    let text = CString::new(text).unwrap_or_default();
    let mut tokens: Vec<JsonToken> = vec![core::mem::zeroed(); 16384];
    let mut d: JsonDoc = core::mem::zeroed();
    let ok = json_parse(text.as_ptr(), tokens.as_mut_ptr(), tokens.len() as c_int, &mut d) > 0
        && (*d.tokens).type_0 == landing::vehicle::models::JSON_OBJECT
        && landing_configuration_from_json(c, &d, 0);
    landing_configuration_normalize(c);
    ok
}

/// A terminal-model source: its environment override, else `filename` beside
/// the atmosphere file.
fn terminal_source_path(environment: &str, atmosphere: &str, filename: &str) -> CString {
    if let Ok(v) = std::env::var(environment) {
        if !v.is_empty() {
            return CString::new(v).unwrap_or_default();
        }
    }
    let dir = atmosphere.rfind('/').map(|i| &atmosphere[..i]).unwrap_or("");
    CString::new(if atmosphere.is_empty() { filename.to_string() } else { format!("{dir}/{filename}") }).unwrap_or_default()
}

fn error_text(buf: &[c_char]) -> String {
    unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy().into_owned()
}

#[no_mangle]
pub unsafe extern "C" fn expert_create(config: *const c_char, atmosphere: *const c_char) -> *mut c_void {
    if config.is_null() || atmosphere.is_null() {
        return core::ptr::null_mut();
    }
    let (Ok(config), Ok(atmosphere_path)) = (CStr::from_ptr(config).to_str(), CStr::from_ptr(atmosphere).to_str()) else {
        return core::ptr::null_mut();
    };
    // SAFETY: the guidance state is plain C data and `result` zeroes to `None`.
    let mut e: Box<OfflineExpert> = Box::new_zeroed().assume_init();
    if !load_config(config, &mut e.cfg) {
        return core::ptr::null_mut();
    }
    let mut error = [0 as c_char; 256];
    if !shuttle_sim_load_planet(atmosphere, &mut e.planet, error.as_mut_ptr(), error.len()) {
        eprintln!("expert: {}", error_text(&error));
        return core::ptr::null_mut();
    }
    let aero = terminal_source_path("KSP_LANDER_TERMINAL_AERO", atmosphere_path, "stsn_aero_ksp_robust.csv");
    let book = terminal_source_path("KSP_LANDER_TERMINAL_AERO_BOOK", atmosphere_path, "stsn_force_book.csv");
    let attitude = terminal_source_path("KSP_LANDER_TERMINAL_ATTITUDE", atmosphere_path, "stsn_attitude_ksp.ini");
    let files = TerminalModelSourceFiles { atmosphere_csv: atmosphere, aero_csv: aero.as_ptr(), aero_book_csv: book.as_ptr(), attitude_ini: attitude.as_ptr() };
    let mut reason = [0 as c_char; 256];
    if !terminal_model_capture(&mut e.terminal_model, &files, &e.cfg, 1, reason.as_mut_ptr(), reason.len()) {
        eprintln!("expert: native MM305 model unavailable: {}", error_text(&reason));
        return core::ptr::null_mut();
    }
    let v = &e.cfg.vehicle;
    e.aero = AerodynamicModel { lift_to_drag: v.estimated_lift_to_drag, ballistic_coefficient: v.estimated_ballistic_coefficient, confidence: 0.9 };
    e.envelope.regimes = [e.aero; 4];
    let c = &mut e.calibration;
    c.density_scale = 1.0;
    c.drag_scale = 1.0;
    c.lift_scale = 1.0;
    c.bank_effectiveness = 1.0;
    c.speed_of_sound_scale = 1.0;
    c.speed_of_sound = 300.0;
    c.stress_drag_scale = 1.0;
    c.stress_lift_scale = 1.0;
    c.confidence = 0.9;
    e.plan.execution_qualified = true;
    e.plan.target_capture_achieved = true;
    e.plan.predicted_post_burn_periapsis_altitude = 50000.0;
    guidance_machine_init(&mut e.g);
    Box::into_raw(e) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn expert_destroy(handle: *mut c_void) {
    if !handle.is_null() {
        drop(Box::from_raw(handle as *mut OfflineExpert));
    }
}

fn g9(out: &mut String, v: f64) {
    let mut buf = [0u8; 64];
    // SAFETY: a single-double conversion into a bounded buffer.
    let n = unsafe { libc::snprintf(buf.as_mut_ptr().cast(), buf.len(), c"%.9g".as_ptr(), v) };
    out.push_str(core::str::from_utf8(&buf[..(n.max(0) as usize).min(buf.len() - 1)]).unwrap_or("nan"));
}

fn b(v: bool) -> &'static str {
    if v { "true" } else { "false" }
}

/// One control interval: telemetry packet in, teacher command and gates out.
/// Phase, reversal, HAC and gear ownership deliberately remain with the teacher.
#[no_mangle]
pub unsafe extern "C" fn expert_command(handle: *mut c_void, packet: *const c_char) -> *const c_char {
    let Some(e) = (handle as *mut OfflineExpert).as_mut() else { return core::ptr::null() };
    if packet.is_null() {
        return core::ptr::null();
    }
    let mut t: Telemetry = core::mem::zeroed();
    let mut s: VehicleState = core::mem::zeroed();
    let mut error = [0 as c_char; 256];
    let previous: *const Telemetry = if e.have_prev { &e.previous } else { core::ptr::null() };
    if !shuttle_sim_decode_telemetry(packet, &e.planet, previous, &mut t, &mut s, error.as_mut_ptr(), error.len()) {
        return core::ptr::null();
    }
    shuttle_sim_prepare_guidance_telemetry(&mut t, &e.cfg, &e.planet);
    e.previous = t;
    e.have_prev = true;
    if !e.initialized {
        guidance_initialize_reentry_continuation(&mut e.g, &t, &e.planet, &e.cfg, 1.0, false, &e.envelope, &e.calibration);
        e.initialized = true;
    }
    let cfg = &e.cfg;
    let v = &cfg.vehicle;
    let reference = if e.g.entry_reference_speed > 0.0 { e.g.entry_reference_speed } else { t.true_air_speed };
    let demand = entry_terminal_demand(
        t.latitude,
        t.mean_altitude,
        t.range_to_site,
        t.horizontal_speed,
        t.course_to_site_error,
        t.vertical_speed,
        t.true_air_speed,
        reference,
        &e.planet,
        v,
        &cfg.site,
        &cfg.guidance,
    );
    t.energy_excess_range = -demand.projected_taem_range_error;
    let mut r = guidance_update_with_terminal_model(&mut e.g, &t, &s, &e.plan, &e.planet, e.aero, cfg, &e.terminal_model);
    let aoa = if r.command.has_target_aoa { r.command.target_aoa } else { r.command.target_pitch - t.flight_path_angle };
    let bank = r.command.target_roll;
    let stall = vessel_physics_conservative_stall_fraction(t.true_air_speed, t.angle_of_attack, t.dynamic_pressure, v);
    let bank_limit = entry_bank_authority_limit(t.true_air_speed, t.dynamic_pressure, t.g_force, v, v.maximum_bank_angle);
    let mut capture = entry_taem_interface_capture(&e.g.taem_interface_target, &t, t.ground_track_heading, &e.planet, cfg);
    capture.ready = capture.valid && capture.veto == 0;
    let floor = entry_low_q_protective_aoa_floor(t.dynamic_pressure, v).max(entry_terminal_turn_aoa_floor(t.true_air_speed, t.dynamic_pressure, v));
    let valid = aoa.is_finite() && bank.is_finite() && stall.is_finite();
    // Restrict learning to a small entry residual. A GuidanceResult warning is
    // not itself a physical abort: the simulator starts from an unqualified
    // post-burn checkpoint, so MM304 commonly publishes an advisory while the
    // vehicle remains inside the aerodynamic envelope. Hard physical protection
    // remains here and in Environment.step(): finite commands, the active
    // phase, speed/altitude, stall margin, q/G limits and the AoA/bank
    // projection bound every learned command. The learner cannot commit a
    // reversal, change phase, select HAC geometry, or bypass an abort/fault.
    let mut authority_state = t;
    authority_state.stall_fraction = stall;
    authority_state.stall_fraction_is_measured = stall.is_finite();
    let authority = decision_control_authority_envelope(&e.g, &authority_state, &e.planet, cfg);
    let phase: GuidancePhase = r.phase;
    let aborting = !valid || phase == PHASE_ABORT || phase == PHASE_FAULT;
    let hard_physical_gate = aborting || !authority.valid || !authority.survivable;
    let residual_phase = phase == PHASE_ENTRY_ENERGY || phase == PHASE_TAEM;
    let resolution = f64::EPSILON.sqrt() * bank.abs().max(bank_limit.abs()).max(1.0);
    let enabled = !hard_physical_gate
        && residual_phase
        && authority.controllable
        && bank.abs() <= bank_limit + resolution
        && aoa.abs() <= v.maximum_angle_of_attack;

    let mut o = String::with_capacity(RESULT_CAPACITY);
    let name = PHASE_NAMES.get(phase as usize).copied().unwrap_or("?");
    let _ = write!(o, "{{\"phase\":\"{name}\",\"phase_id\":{},\"phase_domain_max\":{}", phase as c_int, PHASE_ATTITUDE_RECOVERY as c_int);
    macro_rules! num {
        ($k:literal, $v:expr) => {{
            o.push_str(concat!(",\"", $k, "\":"));
            g9(&mut o, $v);
        }};
    }
    macro_rules! flag {
        ($k:literal, $v:expr) => {{
            let _ = write!(o, concat!(",\"", $k, "\":{}"), b($v));
        }};
    }
    num!("aoa", if valid { aoa } else { t.angle_of_attack });
    num!("bank", if valid { bank } else { t.roll });
    flag!("gear", r.command.gear);
    flag!("brakes", r.command.brakes);
    flag!("abort", aborting);
    flag!("enabled", enabled);
    num!("aoa_min", floor);
    num!("aoa_max", v.maximum_angle_of_attack);
    num!("bank_max", bank_limit);
    num!("q_max", v.maximum_dynamic_pressure);
    num!("g_max", v.maximum_g_load);
    num!("stall", stall);
    num!("touchdown_speed_mps", v.touchdown_speed);
    num!("touchdown_sink_rate_mps", cfg.guidance.touchdown_sink_rate);
    flag!("authority_valid", authority.valid);
    flag!("authority_survivable", authority.survivable);
    flag!("authority_controllable", authority.controllable);
    num!("authority_response_time_s", authority.control_response_time_s);
    num!("authority_speed_margin", authority.speed.margin);
    num!("authority_q_margin", authority.dynamic_pressure.margin);
    num!("authority_load_margin", authority.load.margin);
    num!("authority_stall_margin", authority.stall.margin);
    num!("authority_speed_normalized_margin", authority.speed.normalized_margin);
    num!("authority_q_normalized_margin", authority.dynamic_pressure.normalized_margin);
    num!("authority_load_normalized_margin", authority.load.normalized_margin);
    num!("authority_stall_normalized_margin", authority.stall.normalized_margin);
    num!("energy_error", demand.projected_taem_range_error);
    num!("course_error", t.course_to_site_error);
    num!("range", t.range_to_site);
    num!("bearing", t.bearing_to_site);
    num!("heading_error", t.heading_error);
    flag!("warning", r.has_warning);
    num!("course_rate", t.course_rate);
    num!("hac_radius", e.g.hac_radius);
    num!("hac_remaining", e.g.hac_remaining);
    flag!("hac_captured", e.g.hac_captured);
    flag!("reversal_committed", e.g.entry_reversal_scheduled);
    num!("hac_violation", 0.0);
    flag!("taem_capture_valid", capture.valid);
    flag!("taem_capture_ready", capture.ready);
    let _ = write!(o, ",\"taem_capture_veto\":{}", capture.veto as c_uint);
    num!("taem_capture_along_m", capture.along);
    num!("taem_capture_cross_m", capture.cross);
    num!("taem_capture_course_error_deg", capture.course_error);
    num!("taem_capture_altitude_error_m", capture.altitude_error);
    num!("taem_capture_energy_margin", capture.energy_margin);
    num!("taem_capture_turn_margin", capture.turn_margin);
    num!("taem_capture_path_length_m", capture.path_length_m);
    num!("taem_capture_required_time_s", capture.required_time_s);
    num!("taem_capture_available_time_s", capture.available_time_s);
    num!("taem_capture_minimum_turn_radius_m", capture.minimum_turn_radius_m);
    num!("taem_energy_available", capture.energy_available);
    num!("taem_energy_drag_work", capture.energy_drag_work);
    num!("taem_energy_uncertainty", capture.energy_uncertainty);

    // Diagnostics through the backend's JSON writer, as the flight log formats them.
    let mut w: JsonWriter = core::mem::zeroed();
    jw_init(&mut w);
    jw_raw(&mut w, c",\"diagnostics\":{\"status\":".as_ptr());
    jw_string(&mut w, r.status.as_ptr());
    jw_raw(&mut w, c",\"warning\":".as_ptr());
    jw_string(&mut w, r.warning.as_ptr());
    let key = |w: &mut JsonWriter, k: &str| {
        let k = CString::new(format!(",\"{k}\":")).unwrap_or_default();
        jw_raw(w, k.as_ptr());
    };
    let dn = |w: &mut JsonWriter, k: &str, v: f64| {
        key(w, k);
        jw_number(w, v);
    };
    let db = |w: &mut JsonWriter, k: &str, v: bool| {
        key(w, k);
        jw_bool(w, v);
    };
    let g = &e.g;
    let native = g.mm305_route_committed && g.mm305_route.valid;
    let on = |v: f64| if native { v } else { 0.0 };
    let target = &g.taem_interface_target;
    db(&mut w, "taem_owned", g.taem_exec.ownership_latched);
    dn(&mut w, "taem_phase", g.taem_exec.phase as f64);
    dn(&mut w, "transition_reason", g.taem_exec.last_transition_reason as f64);
    db(&mut w, "terminal_region_entered", g.terminal_region_entered);
    db(&mut w, "terminal_glide_mode", g.terminal_glide_mode);
    db(&mut w, "terminal_test_capture_active", g.terminal_test_capture_active);
    db(&mut w, "taem_interface_captured", g.taem_interface_captured);
    db(&mut w, "final_approach_captured", g.final_approach_captured);
    db(&mut w, "entry_exec_initialized", g.entry_exec.initialized);
    db(&mut w, "entry_exec_complete", g.entry_exec.entry_complete);
    dn(&mut w, "entry_exec_phase", g.entry_exec.phase as f64);
    db(&mut w, "entry_target_side_latched", g.entry_target_side_latched);
    dn(&mut w, "entry_target_side", g.entry_target_side as f64);
    db(&mut w, "target_valid", target.valid);
    db(&mut w, "path_committed", g.terminal_path_committed);
    db(&mut w, "candidate_valid", native);
    dn(&mut w, "candidate_kind", if native { TERMINAL_PATH_HAC } else { TERMINAL_PATH_NONE } as f64);
    db(&mut w, "candidate_degraded", false);
    db(&mut w, "candidate_geometry_degraded", false);
    db(&mut w, "candidate_energy_degraded", false);
    dn(&mut w, "candidate_radius_m", on(g.mm305_route.hac.radius_m));
    dn(&mut w, "candidate_slope_deg", on(g.mm305_route.profile_final_slope_deg));
    dn(&mut w, "candidate_altitude_m", on(g.mm305_route.profile_final_altitude_m));
    dn(&mut w, "candidate_speed_mps", on(t.true_air_speed));
    dn(&mut w, "candidate_course_deg", on(g.mm305_route.runway_heading_deg));
    dn(&mut w, "candidate_response_s", 0.0);
    dn(&mut w, "candidate_arrival_ut", f64::NAN);
    dn(&mut w, "candidate_lead_length_m", on(g.mm305_route.lead_length_m));
    dn(&mut w, "candidate_join_length_m", on(g.mm305_route.length_m));
    dn(&mut w, "candidate_arc_remaining_m", on(g.hac_remaining));
    dn(&mut w, "candidate_final_distance_m", on(g.mm305_route.hac.final_length_m));
    dn(&mut w, "candidate_violation", 0.0);
    dn(&mut w, "candidate_peak_lateral_mps2", 0.0);
    dn(&mut w, "candidate_peak_rate_ratio", 0.0);
    dn(&mut w, "path_kind", g.terminal_path_kind as f64);
    dn(&mut w, "reference_heading_deg", on(g.mm305_route.runway_heading_deg));
    dn(&mut w, "reference_bank_deg", 0.0);
    dn(&mut w, "reference_aoa_deg", on(t.angle_of_attack));
    dn(&mut w, "reference_path_lateral_accel_mps2", 0.0);
    dn(&mut w, "reference_path_bank_deg", 0.0);
    dn(&mut w, "reference_path_course_error_deg", 0.0);
    dn(&mut w, "reference_path_arc_remaining_m", on(g.hac_remaining));
    db(&mut w, "reference_path_transition_active", false);
    dn(&mut w, "candidate_live_energy_margin_j_kg", 0.0);
    db(&mut w, "candidate_live_energy_valid", false);
    db(&mut w, "hac_transition_active", false);
    db(&mut w, "hac_captured", g.hac_captured);
    dn(&mut w, "hac_side", g.hac_side);
    dn(&mut w, "terminal_mix", on(1.0));
    dn(&mut w, "target_along_m", target.along_track);
    dn(&mut w, "target_cross_m", target.cross_track);
    dn(&mut w, "target_course_deg", target.course);
    dn(&mut w, "target_altitude_m", target.altitude);
    dn(&mut w, "target_speed_mps", target.speed);
    dn(&mut w, "target_fpa_deg", target.flight_path_angle);
    dn(&mut w, "target_selected_ut", target.selected_ut);
    dn(&mut w, "target_arrival_ut", target.arrival_ut);
    dn(&mut w, "target_acquisition_lead_m", target.acquisition_lead);
    dn(&mut w, "target_remaining_path_m", target.remaining_path);
    dn(&mut w, "target_response_s", target.response_time);
    dn(&mut w, "capture_course_error_deg", capture.course_error);
    db(&mut w, "delivery_valid", g.taem_exec.terminal_evaluation.valid);
    dn(&mut w, "delivery_block", g.taem_exec.terminal_evaluation.block_reason as f64);
    dn(&mut w, "delivery_altitude_margin_m", g.taem_exec.terminal_contract.altitude_margin);
    dn(&mut w, "delivery_energy_margin_j_kg", g.taem_exec.terminal_contract.specific_energy_margin);
    db(&mut w, "preflare_valid", g.terminal_preflare_plan_valid);
    dn(&mut w, "preflare_height_loss_m", g.preflare_predicted_height_loss);
    dn(&mut w, "vertical_stage", g.terminal_vertical_stage as f64);
    dn(&mut w, "atmosphere_samples", e.planet.atmosphere_sample_count as f64);
    // Entry departure/reversal state is diagnostic evidence for the teacher;
    // it grants no control authority and does not change the decision.
    db(&mut w, "attitude_recovery", g.attitude_recovery);
    dn(&mut w, "control_bad_duration_s", g.control_bad_duration);
    dn(&mut w, "control_good_duration_s", g.control_good_duration);
    dn(&mut w, "recovery_duration_s", g.recovery_duration);
    dn(&mut w, "roll_oscillation_score", g.roll_oscillation_score);
    dn(&mut w, "roll_rate_excess_duration_s", g.roll_rate_excess_duration);
    dn(&mut w, "previous_relative_roll_rate_deg_s", g.previous_relative_roll_rate);
    db(&mut w, "entry_reversal_scheduled", g.entry_reversal_scheduled);
    db(&mut w, "entry_reversal_is_final", g.entry_reversal_is_final);
    dn(&mut w, "entry_reversal_ut", g.entry_reversal_ut);
    dn(&mut w, "entry_reversal_sign", g.entry_reversal_sign);
    dn(&mut w, "entry_control_reversals", g.entry_control_reversals as f64);
    dn(&mut w, "entry_command_bank_deg", g.entry_control_bank);
    dn(&mut w, "entry_plan_bank_deg", g.entry_s_turn_plan.target_bank);
    dn(&mut w, "entry_plan_aoa_deg", g.entry_s_turn_plan.target_aoa);
    dn(&mut w, "entry_lateral_bank_magnitude_deg", g.entry_lateral_bank_magnitude);
    dn(&mut w, "entry_geometry_bank_deg", g.entry_geometry_bank);
    dn(&mut w, "entry_vertical_bank_magnitude_deg", g.entry_vertical_bank_magnitude);
    dn(&mut w, "entry_demand_bank_deg", g.entry_demand_bank);
    dn(&mut w, "entry_lateral_required_bank_deg", g.entry_lateral_required_bank);
    jw_raw(&mut w, c"}}".as_ptr());
    let diagnostics =
        if w.failed || w.data.is_null() { None } else { Some(String::from_utf8_lossy(core::slice::from_raw_parts(w.data as *const u8, w.length)).into_owned()) };
    jw_free(&mut w);
    guidance_result_clear(&mut r);
    // The record replaces the summary's closing brace and must fit the
    // teacher's fixed result buffer.
    let Some(diagnostics) = diagnostics else { return core::ptr::null() };
    o.push_str(&diagnostics);
    if o.len() >= RESULT_CAPACITY {
        return core::ptr::null();
    }
    e.result.insert(CString::new(o).unwrap_or_default()).as_ptr()
}
