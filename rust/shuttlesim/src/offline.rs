//! Offline-only C ABI (loaded by `ShuttleSim/rl` through ctypes). No sockets,
//! actuator control, or changes to sim physics.

use crate::ffi::*;
use crate::protocol::SimCommand;
use crate::scenario::Scenario;
use crate::sim::Simulation;
use core::ffi::{c_char, c_int, c_void, CStr};
use std::ffi::CString;

pub struct OfflineSim {
    sim: Box<Simulation>,
    telemetry: CString,
    summary: CString,
    initial_conditions_set: bool,
}

unsafe fn text<'a>(p: *const c_char) -> Option<&'a str> {
    if p.is_null() { None } else { CStr::from_ptr(p).to_str().ok() }
}

unsafe fn env<'a>(handle: *mut c_void) -> Option<&'a mut OfflineSim> {
    (handle as *mut OfflineSim).as_mut()
}

#[no_mangle]
pub unsafe extern "C" fn offline_create(
    scenario: *const c_char,
    atmosphere: *const c_char,
    aero: *const c_char,
    attitude: *const c_char,
) -> *mut c_void {
    let Some(initial) = text(scenario).and_then(Scenario::load) else { return core::ptr::null_mut() };
    if atmosphere.is_null() || aero.is_null() || attitude.is_null() {
        return core::ptr::null_mut();
    }
    let mut sim = Simulation::new(&initial);
    if !world_load_atmosphere_csv(&mut sim.world, atmosphere)
        || !aero_load_csv(&mut sim.aero, aero)
        || !attitude_load_ini(&mut sim.state.attitude, attitude)
    {
        return core::ptr::null_mut();
    }
    let e = OfflineSim { sim, telemetry: CString::default(), summary: CString::default(), initial_conditions_set: false };
    Box::into_raw(Box::new(e)) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn offline_destroy(handle: *mut c_void) {
    if !handle.is_null() {
        drop(Box::from_raw(handle as *mut OfflineSim));
    }
}

/// Advance one guidance interval. The caller supplies elapsed simulator time;
/// physics substeps come from the configured timestep, including a partial
/// final step. Replay speed is independent of a wall clock.
#[no_mangle]
pub unsafe extern "C" fn offline_step(handle: *mut c_void, aoa: f64, bank: f64, gear: c_int, brakes: c_int, duration_s: f64) -> c_int {
    let Some(e) = env(handle) else { return 0 };
    if !aoa.is_finite() || !bank.is_finite() || !duration_s.is_finite() || !(duration_s > 0.0) {
        return 0;
    }
    let dt = e.sim.physics_dt_s;
    if !dt.is_finite() || !(dt > 0.0) {
        return 0;
    }
    let command = SimCommand {
        has_attitude: true,
        aoa_deg: aoa,
        bank_deg: bank,
        has_gear: true,
        gear_down: gear != 0,
        has_brakes: true,
        brakes: brakes != 0,
        ..Default::default()
    };
    e.sim.apply_command(&command);
    let mut elapsed = 0.0;
    while elapsed < duration_s {
        let step = dt.min(duration_s - elapsed);
        if !step.is_finite() || !(step > 0.0) {
            return 0;
        }
        e.sim.step(step);
        let next = elapsed + step;
        if !(next > elapsed) {
            return 0;
        }
        elapsed = next;
    }
    1
}

#[no_mangle]
pub unsafe extern "C" fn offline_telemetry(handle: *mut c_void) -> *const c_char {
    let Some(e) = env(handle) else { return core::ptr::null() };
    e.telemetry = CString::new(e.sim.telemetry_json(0.0)).unwrap_or_default();
    e.telemetry.as_ptr()
}

#[no_mangle]
pub unsafe extern "C" fn offline_summary(handle: *mut c_void) -> *const c_char {
    let Some(e) = env(handle) else { return core::ptr::null() };
    e.summary = CString::new(e.sim.summary_json()).unwrap_or_default();
    e.summary.as_ptr()
}

#[no_mangle]
pub unsafe extern "C" fn offline_runway_contains(handle: *mut c_void, along_m: f64, cross_m: f64) -> c_int {
    let Some(e) = env(handle) else { return 0 };
    if !along_m.is_finite() || !cross_m.is_finite() {
        return 0;
    }
    runway_contains(&e.sim.runway, along_m, cross_m) as c_int
}

fn seed_state(e: &mut OfflineSim, position: Vec3, velocity: Vec3, air_velocity: Vec3) {
    let s = &mut e.sim;
    let ut = s.state.ut;
    s.state.position_i_m = position;
    s.state.velocity_i_mps = velocity;
    let a = s.state.attitude;
    unsafe {
        s.state.body_q_i = attitude_body_quat(position, air_velocity, a.aoa_rad, a.bank_rad);
        s.state.aero = aero_compute(&s.world, &s.aero, position, velocity, ut, s.state.mass_kg, a.aoa_rad, a.bank_rad);
    }
    e.initial_conditions_set = true;
}

/// Re-seed the atmospheric entry state for one simulator-only episode, in
/// local flight variables (surface-relative speed, north-clockwise heading).
#[no_mangle]
pub unsafe extern "C" fn offline_set_initial_conditions(
    handle: *mut c_void,
    altitude_m: f64,
    speed_mps: f64,
    flight_path_angle_deg: f64,
    heading_deg: f64,
    latitude_deg: f64,
    longitude_deg: f64,
) -> c_int {
    let Some(e) = env(handle) else { return 0 };
    if ![altitude_m, speed_mps, flight_path_angle_deg, heading_deg, latitude_deg, longitude_deg].iter().all(|v| v.is_finite()) {
        return 0;
    }
    let radius = e.sim.world.radius_m;
    // Spherical-coordinate and positive-speed domains.
    if e.initial_conditions_set
        || e.sim.state.sim_elapsed_s != 0.0
        || !(radius > 0.0)
        || altitude_m <= -radius
        || !(speed_mps > 0.0)
        || flight_path_angle_deg <= -90.0
        || flight_path_angle_deg >= 90.0
        || latitude_deg.abs() > 90.0
    {
        return 0;
    }
    let w = &e.sim.world;
    let ut = e.sim.state.ut;
    let position = world_lla_to_inertial(w, deg2rad(latitude_deg), deg2rad(longitude_deg), altitude_m, ut);
    let frame = world_local_frame_i(w, position, ut);
    let (heading, fpa) = (deg2rad(heading_deg), deg2rad(flight_path_angle_deg));
    let horizontal = normalized(add(scale(frame.north, heading.cos()), scale(frame.east, heading.sin())));
    let surface = add(scale(horizontal, speed_mps * fpa.cos()), scale(frame.up, speed_mps * fpa.sin()));
    let velocity = add(surface, world_atmosphere_velocity_i(w, position));
    seed_state(e, position, velocity, surface);
    1
}

/// Build a pre-deorbit Keplerian orbit, burn retrograde at apoapsis to the
/// target periapsis, and leave the simulator at the post-burn state.
#[no_mangle]
pub unsafe extern "C" fn offline_set_deorbit_orbit(
    handle: *mut c_void,
    apoapsis_altitude_m: f64,
    periapsis_altitude_m: f64,
    inclination_deg: f64,
    raan_deg: f64,
    argument_of_periapsis_deg: f64,
    target_periapsis_altitude_m: f64,
) -> c_int {
    let Some(e) = env(handle) else { return 0 };
    let inputs = [apoapsis_altitude_m, periapsis_altitude_m, inclination_deg, raan_deg, argument_of_periapsis_deg, target_periapsis_altitude_m];
    if !inputs.iter().all(|v| v.is_finite()) {
        return 0;
    }
    let w = &e.sim.world;
    let (radius, mu, top) = (w.radius_m, w.mu_m3_s2, w.atmosphere_top_m);
    // Orbital validity: the pre-deorbit orbit stays outside the atmosphere and
    // the post-burn periapsis intersects it.
    if e.initial_conditions_set
        || e.sim.state.sim_elapsed_s != 0.0
        || !(radius > 0.0)
        || !(mu > 0.0)
        || !(top >= 0.0)
        || apoapsis_altitude_m < top
        || periapsis_altitude_m < top
        || periapsis_altitude_m > apoapsis_altitude_m
        || !(0.0..=180.0).contains(&inclination_deg)
        || target_periapsis_altitude_m <= -radius
        || target_periapsis_altitude_m >= top
        || target_periapsis_altitude_m >= periapsis_altitude_m
    {
        return 0;
    }
    let ra = radius + apoapsis_altitude_m;
    let rp = radius + periapsis_altitude_m;
    let target_rp = radius + target_periapsis_altitude_m;
    let a = 0.5 * (ra + rp);
    let target_a = 0.5 * (ra + target_rp);
    let pre_speed = (mu * (2.0 / ra - 1.0 / a)).sqrt();
    let post_speed = (mu * (2.0 / ra - 1.0 / target_a)).sqrt();
    let delta_v = pre_speed - post_speed;
    if !pre_speed.is_finite() || !post_speed.is_finite() || !delta_v.is_finite() || !(delta_v > 0.0) {
        return 0;
    }
    let phase = w.rotation_phase_rad_at_ut0 + w.rotation_rate_rad_s * e.sim.state.ut;
    let raan = deg2rad(raan_deg) + phase;
    let argument = deg2rad(argument_of_periapsis_deg);
    let (ci, si) = (deg2rad(inclination_deg).cos(), deg2rad(inclination_deg).sin());
    let (co, so, cw, sw) = (raan.cos(), raan.sin(), argument.cos(), argument.sin());
    let periapsis_axis = v3(co * cw - so * sw * ci, so * cw + co * sw * ci, sw * si);
    let transverse_axis = v3(-co * sw - so * cw * ci, -so * sw + co * cw * ci, cw * si);
    // At apoapsis (true anomaly pi) position and velocity oppose the perifocal axes.
    let position = scale(periapsis_axis, -ra);
    let pre_velocity = scale(transverse_axis, -pre_speed);
    let post_velocity = add(pre_velocity, scale(normalized(pre_velocity), -delta_v));
    seed_state(e, position, post_velocity, post_velocity);
    1
}

/// Episode-local uncertainty; never writes calibration files. Reset only.
#[no_mangle]
pub unsafe extern "C" fn offline_randomize(handle: *mut c_void, mass: f64, density: f64, lift: f64, drag: f64, lag: f64, velocity: f64) -> c_int {
    let Some(e) = env(handle) else { return 0 };
    let within = |v: f64, lo: f64, hi: f64| v.is_finite() && (lo..=hi).contains(&v);
    if !within(mass, 0.95, 1.05)
        || !within(density, 0.9, 1.1)
        || !within(lift, 0.9, 1.1)
        || !within(drag, 0.9, 1.1)
        || !within(lag, 0.9, 1.1)
        || !within(velocity, -2.0, 2.0)
        || e.sim.state.sim_elapsed_s != 0.0
    {
        return 0;
    }
    let s = &mut e.sim;
    s.state.mass_kg *= mass;
    s.state.velocity_i_mps.x += velocity;
    // No low-level servo lag model: vary the target-following rate instead.
    s.state.attitude.max_pitch_rate_rad_s *= lag;
    s.state.attitude.max_roll_rate_rad_s *= lag;
    let atm = &mut s.world.atmosphere;
    for p in &mut atm.p[..atm.count] {
        p.sample.density_kg_m3 *= density;
    }
    let t = &mut s.aero;
    for i in 0..t.mach_count {
        for j in 0..t.alpha_count {
            t.cl[i][j] *= lift;
            t.cd[i][j] *= drag;
        }
    }
    for b in &mut t.book[..t.book_count] {
        b.lift_per_q_m2 *= lift;
        b.drag_per_q_m2 *= drag;
    }
    1
}

#[no_mangle]
pub unsafe extern "C" fn offline_load_book(handle: *mut c_void, path: *const c_char) -> c_int {
    let Some(e) = env(handle) else { return 0 };
    if path.is_null() {
        return 0;
    }
    aero_load_book_csv(&mut e.sim.aero, path) as c_int
}
