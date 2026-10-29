//! The simulator: the shared airborne plant plus speedbrake, orbital engine,
//! landing-gear contact and touchdown/rollout classification.

use crate::ffi::*;
use crate::protocol::SimCommand;
use crate::scenario::Scenario;
use core::f64::consts::PI;
use std::fmt::Write;

#[derive(Copy, Clone)]
pub struct SimState {
    pub ut: f64,
    pub sim_elapsed_s: f64,
    pub position_i_m: Vec3,
    pub velocity_i_mps: Vec3,
    pub mass_kg: f64,
    pub attitude: AttitudeModel,
    pub body_q_i: Quat,
    pub gear_down: bool,
    pub brakes: bool,
    pub airbrakes: bool,
    pub speedbrake_fraction: f64,
    pub wheel_steering: f64,
    pub ground_lateral_accel_mps2: f64,
    pub ground_airbrake_decel_mps2: f64,
    pub on_ground: bool,
    pub touchdown_seen: bool,
    pub on_runway_at_touchdown: bool,
    pub touchdown_sink_mps: f64,
    pub touchdown_speed_mps: f64,
    pub touchdown_along_m: f64,
    pub touchdown_cross_m: f64,
    pub touchdown_gear: bool,
    pub tail_strike: bool,
    pub touchdown_pitch_deg: f64,
    pub aero: AeroForces,
    /// Landing-gear contact model. Point order: main, nose, tail.
    pub contact_prev_height_m: [f64; 3],
    pub contact_prev_valid: bool,
    pub contact_load_n: [f64; 3],
    pub main_contact: bool,
    pub nose_contact: bool,
    /// Post-mains contract: after the first main-wheel contact pitch is free
    /// (stick zero unless the direct-control FCS says otherwise) and evolves as
    /// a rigid body under identified aerodynamic and gear moments.
    pub pitch_released: bool,
    pub released_pitch_rad: f64,
    pub released_pitch_rate_rad_s: f64,
    pub released_prev_fpa_rad: f64,
    /// Horizontal nose direction, kept when slow.
    pub released_heading_i: Vec3,
    pub main_touchdown_seen: bool,
    pub main_touchdown_ut: f64,
    pub nose_touchdown_seen: bool,
    pub nose_touchdown_sink_mps: f64,
    pub nose_touchdown_delay_s: f64,
    pub bounce_count: i32,
    pub airborne_after_contact_s: f64,
    pub max_gear_load_g: f64,
    pub runway_departure: bool,
    pub departure_along_m: f64,
    pub departure_cross_m: f64,
    pub departure_speed_mps: f64,
    pub stopped: bool,
    pub stop_along_m: f64,
    pub stop_cross_m: f64,
    /// Non-survivable impact: the vehicle is destroyed and the state freezes.
    pub crashed: bool,
}

#[derive(Clone)]
pub struct Simulation {
    pub world: KerbinWorld,
    pub runway: Runway,
    pub aero: AeroTable,
    pub scenario: Scenario,
    pub state: SimState,
    pub physics_dt_s: f64,
    pub rolling_mu: f64,
    pub brake_mu: f64,
    pub ground_airbrake_cda_m2: f64,
    pub ground_max_lateral_accel_g: f64,
    pub ground_max_yaw_rate_rad_s: f64,
    /// Extended-gear drag area. Live KSP identification finds no measurable
    /// gear increment (dCD_gear = -0.003, reference-model/stsn_aero_identified
    /// .report.json), so the default is zero.
    pub gear_cda_m2: f64,
    /// Pitch inertia for gear moments: SURROGATE (radius of gyration 5.5 m at
    /// 40 t), not identified. Aerodynamic pitch moments come from the identified
    /// direct-control coefficients, which are already per inertia.
    pub pitch_inertia_kg_m2: f64,
    pub paused: bool,
    pub deorbit_burn_delay_remaining_s: f64,
    pub deorbit_burn_remaining_s: f64,
    pub deorbit_burn_pending_duration_s: f64,
    pub deorbit_impulse_pending_mps: f64,
    pub deorbit_burn_accel_mps2: f64,
    pub commanded_throttle: f64,
    /// Orbital-engine thrust axis (unit, inertial). Follows the commanded
    /// inertial direction (rate-limited if the scenario sets a slew rate);
    /// until one is commanded the engine thrusts retrograde.
    pub has_thrust_axis: bool,
    pub thrust_axis: Vec3,
    pub has_thrust_dir_command: bool,
    pub thrust_dir_command: Vec3,
    pub propellant_used_kg: f64,
    pub delivered_dv_mps: f64,
}

struct RunwayPlane {
    origin_i: Vec3,
    forward_i: Vec3,
    right_i: Vec3,
    normal_i: Vec3,
}

/* Landing-gear contact geometry (aerospace body frame: +X fwd, +Y right, +Z
   down, origin at the CoM).
   - Main wheels: MEASURED in KSP (quickload): contact patch 3.1535 m below and
     0.9058 m aft of the CoM (both main wheels lumped on the centreline).
   - Nose wheel: its height is set so the vehicle rests at the MEASURED stance
     (pitch -4.3 deg, CoM ~3.06 m above the deck; 22 live stops 2026-09-24/25).
     Its x-station (10 m forward) is a SURROGATE; it only sets the nose/main load
     split and the nose-slam lever arm.
   - Tail point: SURROGATE placed for a 15 deg mains-on-deck tail-strike pitch.
   - Gear retracted: belly points 2 m shallower (SURROGATE).
   Struts are spring-dampers sized for ~0.15 m static compression and ~0.6 of
   critical heave damping; they are a SURROGATE for KSP wheel colliders. */
const CP_MAIN: usize = 0;
const CP_NOSE: usize = 1;
const CP_TAIL: usize = 2;
const GEAR_MAIN_X: f64 = -0.9058;
const GEAR_MAIN_Z: f64 = 3.1535;
const GEAR_NOSE_X: f64 = 10.0;
const GEAR_STANCE_PITCH_RAD: f64 = -4.3 * PI / 180.0;
const GEAR_STATIC_COMPRESSION_M: f64 = 0.15;
const GEAR_HEAVE_DAMPING_RATIO: f64 = 0.6;
const GROUND_PLATEAU_HALF_SIZE_M: f64 = 5000.0;

fn gear_nose_z() -> f64 {
    let (c, sn) = (GEAR_STANCE_PITCH_RAD.cos(), GEAR_STANCE_PITCH_RAD.sin());
    let rest_height = GEAR_MAIN_Z * c - GEAR_MAIN_X * sn;
    (rest_height + GEAR_NOSE_X * sn) / c
}

fn contact_point_b(s: &SimState, which: usize) -> Vec3 {
    let belly = if s.gear_down { 0.0 } else { 2.0 };
    match which {
        CP_MAIN => v3(GEAR_MAIN_X, 0.0, GEAR_MAIN_Z - belly),
        CP_NOSE => v3(GEAR_NOSE_X, 0.0, gear_nose_z() - belly),
        _ => {
            let tail_x = -18.0;
            v3(tail_x, 0.0, GEAR_MAIN_Z - (GEAR_MAIN_X - tail_x) * (15.0 * PI / 180.0).tan())
        }
    }
}

struct Ground {
    height: f64,
    normal: Vec3,
    on_runway: bool,
    along: f64,
    cross: f64,
}

/// Air-relative velocity.
fn air(w: &KerbinWorld, p: Vec3, v: Vec3) -> Vec3 {
    sub(v, unsafe { world_atmosphere_velocity_i(w, p) })
}

fn bool_str(b: bool) -> &'static str {
    if b { "true" } else { "false" }
}

/// Append one number formatted by C `printf`, byte for byte.
fn cf(out: &mut String, spec: &core::ffi::CStr, v: f64) {
    let mut buf = [0u8; 64];
    // SAFETY: `spec` is a single-double conversion; the buffer bound is passed.
    let n = unsafe { libc::snprintf(buf.as_mut_ptr().cast(), buf.len(), spec.as_ptr(), v) };
    out.push_str(core::str::from_utf8(&buf[..(n.max(0) as usize).min(buf.len() - 1)]).unwrap_or("nan"));
}

impl Simulation {
    pub fn new(scenario: &Scenario) -> Box<Simulation> {
        // SAFETY: every field is plain numeric data for which all-zero is valid.
        let mut sim: Box<Simulation> = unsafe { Box::new_zeroed().assume_init() };
        unsafe {
            world_seed_kerbin(&mut sim.world);
            runway_seed_ksp09(&mut sim.runway);
            aero_seed_stsn(&mut sim.aero);
        }
        sim.scenario = *scenario;
        if scenario.runway_override {
            sim.runway = Runway {
                lat_rad: deg2rad(scenario.runway_latitude_deg),
                lon_rad: deg2rad(scenario.runway_longitude_deg),
                elevation_m: scenario.runway_elevation_m,
                heading_rad: deg2rad(scenario.runway_heading_deg),
                length_m: scenario.runway_length_m,
                width_m: scenario.runway_width_m,
            };
        }
        sim.physics_dt_s = 0.02;
        sim.rolling_mu = 0.025;
        sim.brake_mu = 0.18;
        // Conservative terminal-ground model. The split vertical fin is
        // incremental drag area; steering is bounded by lateral acceleration and
        // yaw rate so a rollover-prone rollout cannot hide behind an
        // unrealistically fast heading correction.
        sim.ground_airbrake_cda_m2 = 8.0;
        sim.ground_max_lateral_accel_g = 0.12;
        sim.ground_max_yaw_rate_rad_s = deg2rad(10.0);
        sim.gear_cda_m2 = 0.0;
        sim.pitch_inertia_kg_m2 = 40000.0 * 5.5 * 5.5;
        let s = &mut sim.state;
        s.ut = scenario.ut0;
        s.mass_kg = scenario.mass_kg;
        unsafe { attitude_seed(&mut s.attitude) };
        let a = &mut s.attitude;
        a.aoa_rad = deg2rad(scenario.initial_aoa_deg);
        a.cmd_aoa_rad = a.aoa_rad;
        a.requested_aoa_rad = a.aoa_rad;
        a.bank_rad = deg2rad(scenario.initial_bank_deg);
        a.cmd_bank_rad = a.bank_rad;
        a.requested_bank_rad = a.bank_rad;
        a.sideslip_rad = deg2rad(scenario.initial_sideslip_deg);
        sim.derive_initial_orbit();
        // The scenario is authoritative: Cartesian checkpoints keep their
        // recorded velocity instead of being redirected to KSC.
        sim.apply_deorbit_initial();
        let s = &mut sim.state;
        let vair = air(&sim.world, s.position_i_m, s.velocity_i_mps);
        s.body_q_i = unsafe { attitude_body_quat(s.position_i_m, vair, s.attitude.aoa_rad, s.attitude.bank_rad) };
        if scenario.has_body_attitude {
            let frame = unsafe { world_local_frame_i(&sim.world, s.position_i_m, s.ut) };
            s.body_q_i = unsafe {
                attitude_body_quat_euler(
                    frame,
                    deg2rad(scenario.initial_body_pitch_deg),
                    deg2rad(scenario.initial_body_heading_deg),
                    deg2rad(scenario.initial_body_roll_deg),
                )
            };
            // Preserve the recorded orientation on the first direct body tick.
            s.attitude.body_rates_initialized = true;
        }
        if scenario.has_body_rates {
            s.attitude.body_pitch_rate_rad_s = deg2rad(scenario.initial_body_pitch_rate_deg_s);
            s.attitude.body_roll_rate_rad_s = deg2rad(scenario.initial_body_roll_rate_deg_s);
            s.attitude.body_yaw_rate_rad_s = deg2rad(scenario.initial_body_yaw_rate_deg_s);
            s.attitude.body_rates_initialized = true;
        }
        sim
    }

    fn derive_initial_orbit(&mut self) {
        let s = self.scenario;
        let w = &self.world;
        if s.has_cartesian_state {
            self.state.position_i_m = s.position_i_m;
            self.state.velocity_i_mps = s.velocity_i_mps;
            return;
        }
        let p = unsafe { world_lla_to_inertial(w, deg2rad(s.latitude_deg), deg2rad(s.longitude_deg), s.altitude_m, s.ut0) };
        let lf = unsafe { world_local_frame_i(w, p, s.ut0) };
        let hdg = deg2rad(s.heading_deg);
        let horizontal = normalized(add(scale(lf.north, hdg.cos()), scale(lf.east, hdg.sin())));
        self.state.position_i_m = p;
        self.state.velocity_i_mps = if s.has_surface_flight_state {
            let fpa = deg2rad(s.flight_path_angle_deg);
            let surface = add(scale(horizontal, s.surface_speed_mps * fpa.cos()), scale(lf.up, s.surface_speed_mps * fpa.sin()));
            add(surface, unsafe { world_atmosphere_velocity_i(w, p) })
        } else {
            scale(horizontal, (w.mu_m3_s2 / norm(p)).sqrt())
        };
    }

    /// Recorded delta-v metadata is a legacy open-loop fallback. When a scenario
    /// exposes an orbital engine, guidance owns the burn through the throttle
    /// command; pre-applying the recorded impulse would double-count deorbit.
    fn apply_deorbit_initial(&mut self) {
        if self.scenario.orbital_engine_available_thrust_n > 0.0 {
            return;
        }
        let dv = self.scenario.deorbit_delta_v_mps;
        if dv <= 0.0 {
            return;
        }
        self.deorbit_burn_delay_remaining_s = self.scenario.deorbit_delay_s.max(0.0);
        if self.scenario.deorbit_duration_s <= 0.0 {
            if self.deorbit_burn_delay_remaining_s <= 0.0 {
                let v = self.state.velocity_i_mps;
                self.state.velocity_i_mps = add(v, scale(normalized(v), -dv));
            } else {
                self.deorbit_impulse_pending_mps = dv;
            }
        } else {
            self.deorbit_burn_accel_mps2 = dv / self.scenario.deorbit_duration_s;
            if self.deorbit_burn_delay_remaining_s <= 0.0 {
                self.deorbit_burn_remaining_s = self.scenario.deorbit_duration_s;
            } else {
                self.deorbit_burn_pending_duration_s = self.scenario.deorbit_duration_s;
            }
        }
    }

    pub fn set_attitude(&mut self, aoa_deg: f64, bank_deg: f64) {
        unsafe { attitude_set_command(&mut self.state.attitude, deg2rad(aoa_deg), deg2rad(bank_deg)) };
    }

    pub fn apply_command(&mut self, c: &SimCommand) {
        if std::env::var_os("SHUTTLESIM_COMMAND_DIAG").is_some() {
            let b = |has: bool, v: bool| if has { v as i32 } else { -1 };
            eprintln!(
                "SHUTTLESIM_CMD t={:.3} attitude={} aoa={:.3} bank={:.3} gear={} brakes={} airbrakes={} speedbrake={:.3} wheel={:.3} throttle={:.4} step={} pause={} resume={}",
                self.state.sim_elapsed_s,
                c.has_attitude as i32,
                c.aoa_deg,
                c.bank_deg,
                b(c.has_gear, c.gear_down),
                b(c.has_brakes, c.brakes),
                b(c.has_airbrakes, c.airbrakes),
                if c.has_speedbrake { c.speedbrake } else { -1.0 },
                if c.has_wheel_steering { c.wheel_steering } else { self.state.wheel_steering },
                if c.has_throttle { c.throttle } else { self.commanded_throttle },
                c.step as i32,
                c.pause as i32,
                c.resume as i32
            );
        }
        if c.has_attitude {
            self.set_attitude(c.aoa_deg, c.bank_deg);
        }
        if c.has_inputs {
            unsafe { attitude_set_inputs(&mut self.state.attitude, c.pitch_input, c.roll_input, c.yaw_input) };
        }
        let s = &mut self.state;
        if c.has_gear {
            s.gear_down = c.gear_down;
        }
        if c.has_brakes {
            s.brakes = c.brakes;
        }
        if c.has_airbrakes {
            s.airbrakes = c.airbrakes;
        }
        if c.has_speedbrake {
            s.speedbrake_fraction = c.speedbrake;
        }
        if c.has_wheel_steering {
            s.wheel_steering = c.wheel_steering.max(-1.0).min(1.0);
        }
        if c.has_throttle {
            self.commanded_throttle = c.throttle.max(0.0).min(1.0);
        }
        if c.has_thrust_dir {
            self.has_thrust_dir_command = true;
            self.thrust_dir_command = normalized(v3(c.thrust_dir[0], c.thrust_dir[1], c.thrust_dir[2]));
        }
        if c.pause {
            self.paused = true;
        }
        if c.resume {
            self.paused = false;
        }
    }

    fn runway_plane_at(&self, ut: f64) -> RunwayPlane {
        let r = &self.runway;
        let w = &self.world;
        let origin_i = unsafe { world_lla_to_inertial(w, r.lat_rad, r.lon_rad, r.elevation_m, ut) };
        let lf = unsafe { world_local_frame_i(w, origin_i, ut) };
        let mut forward_i = normalized(add(scale(lf.north, r.heading_rad.cos()), scale(lf.east, r.heading_rad.sin())));
        let mut normal_i = lf.up;
        if self.scenario.runway_slope_deg != 0.0 {
            let slope = deg2rad(self.scenario.runway_slope_deg);
            let forward = forward_i;
            forward_i = add(scale(forward, slope.cos()), scale(lf.up, slope.sin()));
            normal_i = sub(scale(lf.up, slope.cos()), scale(forward, slope.sin()));
        }
        let right_i = normalized(cross(normal_i, forward_i));
        RunwayPlane { origin_i, forward_i, right_i, normal_i }
    }

    /// Runway-plane (along, cross, height) of a point.
    fn plane_coords(p: &RunwayPlane, point: Vec3) -> (f64, f64, f64) {
        let d = sub(point, p.origin_i);
        (dot(d, p.forward_i), dot(d, p.right_i), dot(d, p.normal_i))
    }

    /// Ground surface: the KSC runway is a finite flat deck on a plateau at
    /// runway elevation (SURROGATE: the runway plane extended to
    /// +/-GROUND_PLATEAU_HALF_SIZE_M); beyond it the terrain is sea level.
    /// Leaving the paved rectangle is a runway departure, not a fall.
    fn ground(&self, plane: &RunwayPlane, point: Vec3) -> Ground {
        let (a, c, h) = Self::plane_coords(plane, point);
        let centre = 0.5 * self.runway.length_m;
        // |h| bound: points elsewhere on the planet can project into the box.
        if (a - centre).abs() <= GROUND_PLATEAU_HALF_SIZE_M && c.abs() <= GROUND_PLATEAU_HALF_SIZE_M && h.abs() < 2000.0 {
            let on_runway = unsafe { runway_contains(&self.runway, a, c) };
            return Ground { height: h, normal: plane.normal_i, on_runway, along: a, cross: c };
        }
        Ground { height: norm(point) - self.world.radius_m, normal: normalized(point), on_runway: false, along: a, cross: c }
    }

    fn surface_fpa(&self, up: Vec3) -> f64 {
        let s = &self.state;
        let surf = air(&self.world, s.position_i_m, s.velocity_i_mps);
        let sp = norm(surf);
        if sp > 1e-6 { clamp(dot(surf, up) / sp, -1.0, 1.0).asin() } else { 0.0 }
    }

    /// After release the body attitude is the rigid-body pitch about the
    /// retained ground heading, not a velocity-relative construction: the
    /// velocity direction is meaningless as the vehicle stops. The heading
    /// follows the ground track while it is well defined.
    fn released_update_body(&mut self, up: Vec3) {
        let st = air(&self.world, self.state.position_i_m, self.state.velocity_i_mps);
        let s = &mut self.state;
        let horizontal = sub(st, scale(up, dot(st, up)));
        if norm(horizontal) > 2.0 {
            s.released_heading_i = normalized(horizontal);
        }
        let hdg = s.released_heading_i;
        let body_fwd = normalized(add(scale(hdg, s.released_pitch_rad.cos()), scale(up, s.released_pitch_rad.sin())));
        let right = normalized(cross(hdg, up));
        let body_up = rotate_axis(normalized(cross(right, body_fwd)), body_fwd, s.attitude.bank_rad);
        let body_right = normalized(cross(body_fwd, body_up));
        s.body_q_i = unsafe { quat_from_basis(body_fwd, body_right, scale(body_up, -1.0)) };
    }

    /// Gear/structure contact forces, friction, steering and touchdown/rollout
    /// classification, once per tick after the airborne integration so
    /// aerodynamic lift unloads the gear naturally.
    fn contact_step(&mut self, dt: f64) {
        let plane = self.runway_plane_at(self.state.ut);
        if self.state.pitch_released {
            self.released_update_body(plane.normal_i);
        }
        let s = self.state;
        let radius = norm(s.position_i_m);
        let g = self.world.mu_m3_s2 / (radius * radius);
        let weight = s.mass_kg * g;
        let k_main = weight / GEAR_STATIC_COMPRESSION_M;
        let k_nose = 0.25 * k_main;
        let c_main = 2.0 * GEAR_HEAVE_DAMPING_RATIO * (k_main * s.mass_kg).sqrt();
        let c_nose = 2.0 * GEAR_HEAVE_DAMPING_RATIO * (k_nose * s.mass_kg).sqrt();
        let (k_tail, c_tail) = (2.0 * k_main, c_main);

        let atmosphere = unsafe { world_atmosphere_velocity_i(&self.world, s.position_i_m) };
        let mut surf = sub(s.velocity_i_mps, atmosphere);
        let mut total = v3(0.0, 0.0, 0.0);
        let mut pitch_moment = 0.0;
        let body_right = rotate(s.body_q_i, v3(0.0, 1.0, 0.0));
        let mut loaded = [false; 3];
        let mut wheel_off_runway = false;
        let mut contact_normal = plane.normal_i;
        let mut deepest: f64 = 0.0;
        for i in 0..3 {
            let r = rotate(self.state.body_q_i, contact_point_b(&self.state, i));
            let point = add(self.state.position_i_m, r);
            let gnd = self.ground(&plane, point);
            let st = &mut self.state;
            let h = gnd.height;
            let hdot = if st.contact_prev_valid { (h - st.contact_prev_height_m[i]) / dt } else { 0.0 };
            st.contact_prev_height_m[i] = h;
            st.contact_load_n[i] = 0.0;
            if h >= 0.0 {
                continue;
            }
            let pen = -h;
            deepest = deepest.max(pen);
            let k = [k_main, k_nose, k_tail][i];
            let c = [c_main, c_nose, c_tail][i];
            let normal_force = (k * pen - c * hdot).max(0.0);
            if !(normal_force > 0.0) {
                continue;
            }
            let n = gnd.normal;
            loaded[i] = true;
            contact_normal = n;
            st.contact_load_n[i] = normal_force;
            if !gnd.on_runway && (i == CP_MAIN || i == CP_NOSE) {
                wheel_off_runway = true;
            }
            // Tangential force: rolling/brake friction on wheels (brakes act on
            // the mains), sliding friction for belly/tail/retracted gear.
            let tangential = sub(surf, scale(n, dot(surf, n)));
            let ts = norm(tangential);
            let wheel = st.gear_down && i != CP_TAIL;
            let mu = if !wheel {
                0.5
            } else if i == CP_MAIN && st.brakes {
                self.brake_mu
            } else {
                self.rolling_mu
            };
            let friction = if ts > 1e-6 { scale(tangential, -mu * normal_force / ts) } else { v3(0.0, 0.0, 0.0) };
            let f = add(scale(n, normal_force), friction);
            total = add(total, f);
            pitch_moment += dot(cross(r, f), body_right);
            if !st.touchdown_seen {
                let vv = dot(surf, n);
                let horiz = sub(surf, scale(n, vv));
                st.touchdown_seen = true;
                st.on_runway_at_touchdown = gnd.on_runway;
                st.touchdown_sink_mps = (-vv).max(0.0);
                st.touchdown_speed_mps = norm(horiz);
                st.touchdown_along_m = gnd.along;
                st.touchdown_cross_m = gnd.cross;
                st.touchdown_gear = st.gear_down && i == CP_MAIN;
                st.tail_strike = i == CP_TAIL;
                let fwd = rotate(st.body_q_i, v3(1.0, 0.0, 0.0));
                st.touchdown_pitch_deg = rad2deg(clamp(dot(fwd, n), -1.0, 1.0).asin());
            }
            if i == CP_MAIN && !st.main_touchdown_seen {
                st.main_touchdown_seen = true;
                st.main_touchdown_ut = st.ut;
            }
            if i == CP_NOSE && !st.nose_touchdown_seen {
                st.nose_touchdown_seen = true;
                st.nose_touchdown_sink_mps = (-hdot).max(0.0);
                st.nose_touchdown_delay_s = if st.main_touchdown_seen { st.ut - st.main_touchdown_ut } else { -1.0 };
            }
            if i == CP_TAIL {
                st.tail_strike = true;
            }
            // Crash: a contact normal speed or ground speed far outside anything
            // a gear or airframe survives. 25 m/s is ~8x the qualification sink
            // limit and 200 m/s is ~2.7x touchdown speed.
            let vn = -dot(surf, n);
            if !st.crashed && (vn > 25.0 || norm(sub(surf, scale(n, dot(surf, n)))) > 200.0) {
                st.crashed = true;
            }
        }
        let w = &self.world;
        let st = &mut self.state;
        if st.crashed {
            st.velocity_i_mps = unsafe { world_atmosphere_velocity_i(w, st.position_i_m) };
            st.on_ground = true;
            st.stopped = true;
            st.contact_prev_valid = false;
            return;
        }
        st.contact_prev_valid = true;
        let load = total.x * contact_normal.x + total.y * contact_normal.y + total.z * contact_normal.z;
        if weight > 0.0 {
            st.max_gear_load_g = st.max_gear_load_g.max(load / weight);
        }
        let any = loaded.iter().any(|&l| l);
        st.main_contact = loaded[CP_MAIN];
        st.nose_contact = loaded[CP_NOSE];
        if st.touchdown_seen && !any {
            st.airborne_after_contact_s += dt;
        } else {
            if any && st.airborne_after_contact_s > 0.1 {
                st.bounce_count += 1;
            }
            st.airborne_after_contact_s = 0.0;
        }
        st.on_ground = any;
        if any {
            // A collider cannot be penetrated far: remove numerical blow-through
            // (only reachable in crash-grade impacts).
            if deepest > 0.5 {
                st.position_i_m = add(st.position_i_m, scale(contact_normal, deepest - 0.5));
            }
            st.velocity_i_mps = add(st.velocity_i_mps, scale(total, dt / st.mass_kg));
            if wheel_off_runway && !st.runway_departure {
                let (a, c, _) = Self::plane_coords(&plane, st.position_i_m);
                st.runway_departure = true;
                st.departure_along_m = a;
                st.departure_cross_m = c;
                let sv = sub(st.velocity_i_mps, atmosphere);
                st.departure_speed_mps = norm(sub(sv, scale(contact_normal, dot(sv, contact_normal))));
            }
            // Nose-wheel steering rotates the ground track, bounded by lateral
            // acceleration, yaw rate and the load actually on the wheels.
            surf = sub(st.velocity_i_mps, atmosphere);
            let tangential = sub(surf, scale(contact_normal, dot(surf, contact_normal)));
            let mut speed = norm(tangential);
            st.ground_lateral_accel_mps2 = 0.0;
            if st.gear_down && loaded[CP_NOSE] && speed > 0.5 && st.wheel_steering.abs() > 1e-6 {
                let load_fraction = clamp(load / weight.max(1.0), 0.0, 1.0);
                let max_lat = self.ground_max_lateral_accel_g * g * load_fraction;
                let max_yaw = self.ground_max_yaw_rate_rad_s.min(max_lat / speed.max(1.0));
                let yaw = st.wheel_steering * max_yaw * dt;
                let t = rotate_axis(tangential, contact_normal, -yaw);
                st.velocity_i_mps = add(add(t, scale(contact_normal, dot(surf, contact_normal))), atmosphere);
                st.ground_lateral_accel_mps2 = (speed * st.wheel_steering * max_yaw).abs();
            }
            if st.brakes && speed < 0.3 && st.gear_down {
                st.velocity_i_mps = add(scale(contact_normal, dot(surf, contact_normal)), atmosphere);
                speed = 0.0;
            }
            if speed < 0.3 && !st.stopped {
                let (a, c, _) = Self::plane_coords(&plane, st.position_i_m);
                st.stopped = true;
                st.stop_along_m = a;
                st.stop_cross_m = c;
            }
        }
        // Post-mains contract: first main-wheel contact releases pitch for good.
        let up = plane.normal_i;
        if loaded[CP_MAIN] && self.state.gear_down && !self.state.pitch_released {
            let fpa = self.surface_fpa(up);
            let st = &mut self.state;
            let fwd = rotate(st.body_q_i, v3(1.0, 0.0, 0.0));
            st.pitch_released = true;
            st.released_pitch_rad = clamp(dot(fwd, up), -1.0, 1.0).asin();
            st.released_prev_fpa_rad = fpa;
            st.released_pitch_rate_rad_s = if st.attitude.direct_body_coordinates && st.attitude.body_rates_initialized {
                st.attitude.body_pitch_rate_rad_s
            } else {
                st.attitude.aoa_rate_rad_s
            };
            st.released_heading_i = normalized(sub(fwd, scale(up, dot(fwd, up))));
        }
        if self.state.pitch_released {
            let fpa = self.surface_fpa(up);
            let inertia = self.pitch_inertia_kg_m2;
            let st = &mut self.state;
            let fpa_rate = (fpa - st.released_prev_fpa_rad) / dt;
            st.released_prev_fpa_rad = fpa;
            let aoa = st.released_pitch_rad - fpa;
            let acc = released_pitch_aero_accel(&st.attitude, st.aero.dynamic_pressure_pa, aoa, st.released_pitch_rate_rad_s)
                + pitch_moment / inertia;
            st.released_pitch_rate_rad_s += acc * dt;
            st.released_pitch_rad += st.released_pitch_rate_rad_s * dt;
            let a = &mut st.attitude;
            a.aoa_rad = st.released_pitch_rad - fpa;
            a.aoa_rate_rad_s = st.released_pitch_rate_rad_s - fpa_rate;
            a.cmd_aoa_rad = a.aoa_rad;
            a.requested_aoa_rad = a.aoa_rad;
            if st.on_ground {
                // Wheels on the deck level the wings.
                a.bank_rad *= (1.0 - 2.0 * dt).max(0.0);
                a.bank_rate_rad_s = 0.0;
                a.cmd_bank_rad = a.bank_rad;
            }
            self.released_update_body(up);
            let st = &mut self.state;
            if norm(air(&self.world, st.position_i_m, st.velocity_i_mps)) < 2.0 {
                // Incidence is undefined when stopped; report pitch.
                let a = &mut st.attitude;
                a.aoa_rad = st.released_pitch_rad;
                a.aoa_rate_rad_s = st.released_pitch_rate_rad_s;
                a.cmd_aoa_rad = a.aoa_rad;
                a.requested_aoa_rad = a.aoa_rad;
            }
        }
        let st = &mut self.state;
        st.aero = unsafe {
            aero_compute(&self.world, &self.aero, st.position_i_m, st.velocity_i_mps, st.ut, st.mass_kg, st.attitude.aoa_rad, st.attitude.bank_rad)
        };
    }

    pub fn step(&mut self, dt: f64) {
        if self.paused {
            return;
        }
        if self.deorbit_burn_delay_remaining_s > 0.0 {
            self.deorbit_burn_delay_remaining_s = (self.deorbit_burn_delay_remaining_s - dt).max(0.0);
            if self.deorbit_burn_delay_remaining_s <= 0.0 {
                if self.deorbit_impulse_pending_mps > 0.0 {
                    let v = self.state.velocity_i_mps;
                    self.state.velocity_i_mps = add(v, scale(normalized(v), -self.deorbit_impulse_pending_mps));
                    self.deorbit_impulse_pending_mps = 0.0;
                }
                if self.deorbit_burn_pending_duration_s > 0.0 {
                    self.deorbit_burn_remaining_s = self.deorbit_burn_pending_duration_s;
                    self.deorbit_burn_pending_duration_s = 0.0;
                }
            }
        }
        if self.state.stopped && self.state.on_ground {
            self.state.ut += dt;
            self.state.sim_elapsed_s += dt;
            return;
        }
        // The shared plant preserves KSP's force-at-tick-start, velocity,
        // position, then attitude ordering. Do not RK4 this force field.
        let engine_thrust = self.scenario.orbital_engine_available_thrust_n * self.commanded_throttle;
        let s = &self.state;
        let mut airborne =
            FlightAirborneState { position_i_m: s.position_i_m, velocity_i_mps: s.velocity_i_mps, attitude: s.attitude, body_q_i: s.body_q_i, aero: s.aero };
        let mut input = FlightAirborneStepInput {
            mass_kg: s.mass_kg,
            ut: s.ut,
            dt_s: dt,
            burn_accel_mps2: if self.deorbit_burn_remaining_s > 0.0 { self.deorbit_burn_accel_mps2 } else { 0.0 },
            engine_thrust_n: engine_thrust,
            engine_thrust_dir_i: v3(0.0, 0.0, 0.0),
        };
        if engine_thrust > 0.0 {
            // Slew the thrust axis toward the commanded inertial direction.
            if !self.has_thrust_dir_command || !self.has_thrust_axis {
                self.thrust_axis = scale(normalized(s.velocity_i_mps), -1.0);
                self.has_thrust_axis = true;
            }
            if self.has_thrust_dir_command {
                let slew = self.scenario.orbital_attitude_slew_dps;
                let cosang = dot(self.thrust_axis, self.thrust_dir_command).min(1.0).max(-1.0);
                let ang = cosang.acos();
                let step_rad = if slew > 0.0 { slew * PI / 180.0 * dt } else { ang };
                if ang <= step_rad || ang < 1e-12 {
                    self.thrust_axis = self.thrust_dir_command;
                } else {
                    let perp = sub(self.thrust_dir_command, scale(self.thrust_axis, cosang));
                    let pn = norm(perp);
                    self.thrust_axis = if pn > 1e-12 {
                        let perp = scale(perp, 1.0 / pn);
                        normalized(add(scale(self.thrust_axis, step_rad.cos()), scale(perp, step_rad.sin())))
                    } else {
                        self.thrust_dir_command
                    };
                }
            }
            input.engine_thrust_dir_i = self.thrust_axis;
        }
        unsafe { flight_physics_airborne_step(&self.world, &self.aero, &input, &mut airborne) };
        let s = &mut self.state;
        if engine_thrust > 0.0 && s.mass_kg > 0.0 {
            self.delivered_dv_mps += engine_thrust / s.mass_kg * dt;
            if self.scenario.orbital_engine_isp_s > 0.0 {
                let dm = (engine_thrust / (self.scenario.orbital_engine_isp_s * 9.80665) * dt).min(0.5 * s.mass_kg);
                s.mass_kg -= dm;
                self.propellant_used_kg += dm;
            }
        }
        s.position_i_m = airborne.position_i_m;
        s.velocity_i_mps = airborne.velocity_i_mps;
        s.attitude = airborne.attitude;
        s.body_q_i = airborne.body_q_i;
        s.aero = airborne.aero;
        if s.gear_down && self.gear_cda_m2 > 0.0 && s.aero.airspeed_mps > 1.0 {
            let vair = air(&self.world, s.position_i_m, s.velocity_i_mps);
            let dv = s.aero.dynamic_pressure_pa * self.gear_cda_m2 / s.mass_kg * dt;
            let va = norm(vair);
            if va > 1e-6 {
                s.velocity_i_mps = sub(s.velocity_i_mps, scale(vair, dv.min(va) / va));
            }
        }
        s.ground_airbrake_decel_mps2 = 0.0;
        // The split fin opens proportionally (deploy limiter) or fully (action
        // group); its drag area scales with the opening.
        let fin_open = if s.airbrakes { 1.0 } else { s.speedbrake_fraction };
        if fin_open > 0.0 && s.aero.airspeed_mps > 1.0 {
            let vair = air(&self.world, s.position_i_m, s.velocity_i_mps);
            let cda = self.ground_airbrake_cda_m2 * fin_open;
            let dv = s.aero.dynamic_pressure_pa * cda / s.mass_kg * dt;
            let va = norm(vair);
            if va > 1e-6 {
                s.velocity_i_mps = sub(s.velocity_i_mps, scale(vair, dv.min(va) / va));
            }
            s.aero.drag_n += s.aero.dynamic_pressure_pa * cda;
            s.ground_airbrake_decel_mps2 = dv / dt;
        }
        if self.deorbit_burn_remaining_s > 0.0 {
            self.deorbit_burn_remaining_s = (self.deorbit_burn_remaining_s - dt).max(0.0);
        }
        s.ut += dt;
        s.sim_elapsed_s += dt;
        self.contact_step(dt);
    }

    pub fn telemetry_json(&self, rate: f64) -> String {
        let s = &self.state;
        let w = &self.world;
        let l = unsafe { world_lla(w, s.position_i_m, s.ut) };
        let lf = unsafe { world_local_frame_i(w, s.position_i_m, s.ut) };
        let surf = air(w, s.position_i_m, s.velocity_i_mps);
        let (vn, ve, vu) = (dot(surf, lf.north), dot(surf, lf.east), dot(surf, lf.up));
        let horizontal = vn.hypot(ve);
        let mut heading = ve.atan2(vn);
        if heading < 0.0 {
            heading += 2.0 * PI;
        }
        let fpa = vu.atan2(horizontal.max(1e-9));
        let a = &s.attitude;
        let body_rates_available = a.direct && a.direct_body_coordinates && a.body_rates_initialized && !s.pitch_released;
        let body_forward = rotate(s.body_q_i, v3(1.0, 0.0, 0.0));
        let body_right = rotate(s.body_q_i, v3(0.0, 1.0, 0.0));
        let body_up = rotate(s.body_q_i, v3(0.0, 0.0, -1.0));
        let body_pitch = clamp(dot(body_forward, lf.up), -1.0, 1.0).asin();
        let mut body_heading = dot(body_forward, lf.east).atan2(dot(body_forward, lf.north));
        if body_heading < 0.0 {
            body_heading += 2.0 * PI;
        }
        let body_roll = (-dot(body_right, lf.up)).atan2(dot(body_up, lf.up));
        let atm = unsafe { world_atmosphere_sample_state(w, s.position_i_m, s.ut) };
        let (mut along, mut cross_m, mut vertical) = (0.0, 0.0, 0.0);
        unsafe { ss_runway_coordinates(w, &self.runway, s.position_i_m, s.ut, &mut along, &mut cross_m, &mut vertical) };
        // Contact uses the finite planar deck even at zero slope. Report the
        // same local clearance; spherical altitude overstates it at the far end.
        let vertical = self.ground(&self.runway_plane_at(s.ut), s.position_i_m).height;

        let mut o = String::with_capacity(4096);
        macro_rules! key {
            ($k:literal, $spec:literal, $v:expr) => {{
                o.push_str($k);
                cf(&mut o, $spec, $v);
            }};
        }
        let _ = write!(o, "{{\"schema\":1,\"type\":\"telemetry\",\"source\":\"sim\",\"scenario\":\"{}\"", self.scenario.name());
        key!(",\"ut\":", c"%.6f", s.ut);
        key!(",\"sim_time\":", c"%.6f", s.sim_elapsed_s);
        key!(",\"sim_rate\":", c"%.2f", rate);
        key!(",\"world\":{\"radius_m\":", c"%.9g", w.radius_m);
        key!(",\"mu_m3_s2\":", c"%.12g", w.mu_m3_s2);
        key!(",\"rotation_rate_rad_s\":", c"%.12g", w.rotation_rate_rad_s);
        key!(",\"atmosphere_top_m\":", c"%.9g", w.atmosphere_top_m);
        key!(",\"rotation_phase_rad_at_ut0\":", c"%.12g", w.rotation_phase_rad_at_ut0);
        key!("},\"position\":{\"x\":", c"%.6f", s.position_i_m.x);
        key!(",\"y\":", c"%.6f", s.position_i_m.y);
        key!(",\"z\":", c"%.6f", s.position_i_m.z);
        key!(",\"lat_deg\":", c"%.9f", rad2deg(l.lat_rad));
        key!(",\"lon_deg\":", c"%.9f", rad2deg(l.lon_rad));
        key!(",\"altitude_m\":", c"%.3f", l.altitude_m);
        key!("},\"velocity\":{\"x_mps\":", c"%.6f", s.velocity_i_mps.x);
        key!(",\"y_mps\":", c"%.6f", s.velocity_i_mps.y);
        key!(",\"z_mps\":", c"%.6f", s.velocity_i_mps.z);
        key!(",\"inertial_mps\":", c"%.3f", norm(s.velocity_i_mps));
        key!(",\"surface_mps\":", c"%.3f", norm(surf));
        key!(",\"air_mps\":", c"%.3f", norm(surf));
        key!(",\"horizontal_mps\":", c"%.3f", horizontal);
        key!(",\"vertical_mps\":", c"%.3f", vu);
        key!(",\"flight_path_angle_deg\":", c"%.6f", rad2deg(fpa));
        key!("},\"attitude\":{\"aoa_deg\":", c"%.4f", rad2deg(a.aoa_rad));
        key!(",\"bank_deg\":", c"%.4f", rad2deg(a.bank_rad));
        key!(",\"heading_deg\":", c"%.4f", rad2deg(heading));
        key!(",\"cmd_aoa_deg\":", c"%.4f", rad2deg(a.cmd_aoa_rad));
        key!(",\"cmd_bank_deg\":", c"%.4f", rad2deg(a.cmd_bank_rad));
        key!(",\"requested_aoa_deg\":", c"%.4f", rad2deg(a.requested_aoa_rad));
        key!(",\"requested_bank_deg\":", c"%.4f", rad2deg(a.requested_bank_rad));
        key!(",\"aoa_rate_deg_s\":", c"%.6f", rad2deg(a.aoa_rate_rad_s));
        key!(",\"bank_rate_deg_s\":", c"%.6f", rad2deg(a.bank_rate_rad_s));
        key!(",\"sideslip_deg\":", c"%.6f", rad2deg(a.sideslip_rad));
        let _ = write!(
            o,
            ",\"direct_control\":{},\"body_attitude_available\":{},\"body_rates_available\":{}",
            bool_str(a.direct),
            bool_str(a.direct_body_coordinates),
            bool_str(body_rates_available)
        );
        key!(",\"body_pitch_rate_deg_s\":", c"%.6f", rad2deg(a.body_pitch_rate_rad_s));
        key!(",\"body_roll_rate_deg_s\":", c"%.6f", rad2deg(a.body_roll_rate_rad_s));
        key!(",\"body_yaw_rate_deg_s\":", c"%.6f", rad2deg(a.body_yaw_rate_rad_s));
        key!(",\"body_pitch_deg\":", c"%.6f", rad2deg(body_pitch));
        key!(",\"body_roll_deg\":", c"%.6f", rad2deg(body_roll));
        key!(",\"body_heading_deg\":", c"%.6f", rad2deg(body_heading));
        key!(",\"input_pitch\":", c"%.9f", a.input_pitch);
        key!(",\"input_roll\":", c"%.9f", a.input_roll);
        key!(",\"input_yaw\":", c"%.9f", a.input_yaw);
        key!(",\"pitch_wn_s_inv\":", c"%.9g", a.pitch_wn);
        key!(",\"pitch_zeta\":", c"%.9g", a.pitch_zeta);
        key!(",\"roll_wn_s_inv\":", c"%.9g", a.roll_wn);
        key!(",\"roll_zeta\":", c"%.9g", a.roll_zeta);
        key!(",\"max_pitch_rate_deg_s\":", c"%.9g", rad2deg(a.max_pitch_rate_rad_s));
        key!(",\"max_roll_rate_deg_s\":", c"%.9g", rad2deg(a.max_roll_rate_rad_s));
        key!(",\"max_pitch_accel_deg_s2\":", c"%.9g", rad2deg(a.max_pitch_accel_rad_s2));
        key!(",\"max_roll_accel_deg_s2\":", c"%.9g", rad2deg(a.max_roll_accel_rad_s2));
        key!(",\"q_w\":", c"%.9f", s.body_q_i.w);
        key!(",\"q_x\":", c"%.9f", s.body_q_i.x);
        key!(",\"q_y\":", c"%.9f", s.body_q_i.y);
        key!(",\"q_z\":", c"%.9f", s.body_q_i.z);
        key!("},\"atmosphere\":{\"density_kg_m3\":", c"%.12g", atm.density_kg_m3);
        key!(",\"pressure_pa\":", c"%.9g", atm.pressure_pa);
        key!(",\"temperature_k\":", c"%.6f", atm.temperature_k);
        key!(",\"speed_of_sound_mps\":", c"%.6f", atm.speed_of_sound_mps);
        key!("},\"aero\":{\"mach\":", c"%.6f", s.aero.mach);
        key!(",\"q_pa\":", c"%.6f", s.aero.dynamic_pressure_pa);
        key!(",\"lift_n\":", c"%.6f", s.aero.lift_n);
        key!(",\"drag_n\":", c"%.6f", s.aero.drag_n);
        key!("},\"vehicle\":{\"mass_kg\":", c"%.6f", s.mass_kg);
        key!(",\"available_thrust_n\":", c"%.6f", self.scenario.orbital_engine_available_thrust_n);
        key!(",\"current_thrust_n\":", c"%.6f", self.scenario.orbital_engine_available_thrust_n * self.commanded_throttle);
        key!(",\"throttle\":", c"%.9f", self.commanded_throttle);
        key!(",\"propellant_used_kg\":", c"%.6f", self.propellant_used_kg);
        key!(",\"engine_dv_mps\":", c"%.6f", self.delivered_dv_mps);
        key!("},\"runway\":{\"along_m\":", c"%.3f", along);
        key!(",\"cross_m\":", c"%.3f", cross_m);
        key!(",\"vertical_m\":", c"%.3f", vertical);
        key!(",\"length_m\":", c"%.3f", self.runway.length_m);
        key!(",\"width_m\":", c"%.3f", self.runway.width_m);
        key!(",\"heading_deg\":", c"%.6f", rad2deg(self.runway.heading_rad));
        let _ = write!(
            o,
            "}},\"ground\":{{\"gear_down\":{},\"brakes\":{},\"airbrakes\":{}",
            bool_str(s.gear_down),
            bool_str(s.brakes),
            bool_str(s.airbrakes)
        );
        key!(",\"speedbrake\":", c"%.4f", s.speedbrake_fraction);
        key!(",\"wheel_steering\":", c"%.6f", s.wheel_steering);
        key!(",\"steering_lateral_accel_mps2\":", c"%.6f", s.ground_lateral_accel_mps2);
        key!(",\"airbrake_decel_mps2\":", c"%.6f", s.ground_airbrake_decel_mps2);
        let _ = write!(
            o,
            ",\"on_ground\":{},\"stopped\":{},\"touchdown_seen\":{},\"on_runway_touchdown\":{},\"main_contact\":{},\"nose_contact\":{},\"pitch_released\":{}",
            bool_str(s.on_ground),
            bool_str(s.stopped),
            bool_str(s.touchdown_seen),
            bool_str(s.on_runway_at_touchdown),
            bool_str(s.main_contact),
            bool_str(s.nose_contact),
            bool_str(s.pitch_released)
        );
        key!(",\"main_load_n\":", c"%.1f", s.contact_load_n[0]);
        key!(",\"nose_load_n\":", c"%.1f", s.contact_load_n[1]);
        key!(",\"tail_load_n\":", c"%.1f", s.contact_load_n[2]);
        let _ = write!(o, ",\"bounce_count\":{},\"runway_departure\":{}", s.bounce_count, bool_str(s.runway_departure));
        key!(",\"pitch_deg\":", c"%.4f", rad2deg(body_pitch));
        o.push_str("}}");
        o
    }

    pub fn summary_json(&self) -> String {
        let s = &self.state;
        let l = unsafe { world_lla(&self.world, s.position_i_m, s.ut) };
        let (mut along, mut cross_m, mut vertical) = (0.0, 0.0, 0.0);
        unsafe { ss_runway_coordinates(&self.world, &self.runway, s.position_i_m, s.ut, &mut along, &mut cross_m, &mut vertical) };
        let mut o = String::with_capacity(2048);
        macro_rules! key {
            ($k:literal, $v:expr) => {{
                o.push_str($k);
                cf(&mut o, c"%.3f", $v);
            }};
        }
        let _ = write!(o, "{{\"scenario\":\"{}\"", self.scenario.name());
        key!(",\"sim_time_s\":", s.sim_elapsed_s);
        key!(",\"final_altitude_m\":", l.altitude_m);
        let _ = write!(o, ",\"touchdown\":{},\"on_runway\":{}", bool_str(s.touchdown_seen), bool_str(s.on_runway_at_touchdown));
        key!(",\"touchdown_sink_mps\":", s.touchdown_sink_mps);
        key!(",\"touchdown_speed_mps\":", s.touchdown_speed_mps);
        key!(",\"touchdown_along_m\":", s.touchdown_along_m);
        key!(",\"touchdown_cross_m\":", s.touchdown_cross_m);
        let _ = write!(o, ",\"touchdown_gear\":{},\"tail_strike\":{}", bool_str(s.touchdown_gear), bool_str(s.tail_strike));
        key!(",\"touchdown_pitch_deg\":", s.touchdown_pitch_deg);
        key!(",\"final_along_m\":", along);
        key!(",\"final_cross_m\":", cross_m);
        let _ = write!(o, ",\"main_touchdown\":{},\"nose_touchdown\":{}", bool_str(s.main_touchdown_seen), bool_str(s.nose_touchdown_seen));
        key!(",\"nose_touchdown_sink_mps\":", s.nose_touchdown_sink_mps);
        key!(",\"nose_touchdown_delay_s\":", s.nose_touchdown_delay_s);
        let _ = write!(o, ",\"bounce_count\":{}", s.bounce_count);
        key!(",\"max_gear_load_g\":", s.max_gear_load_g);
        let _ = write!(o, ",\"runway_departure\":{}", bool_str(s.runway_departure));
        key!(",\"departure_along_m\":", s.departure_along_m);
        key!(",\"departure_cross_m\":", s.departure_cross_m);
        key!(",\"departure_speed_mps\":", s.departure_speed_mps);
        let _ = write!(o, ",\"stopped\":{}", bool_str(s.stopped));
        key!(",\"stop_along_m\":", s.stop_along_m);
        key!(",\"stop_cross_m\":", s.stop_cross_m);
        let _ = write!(
            o,
            ",\"crashed\":{},\"ground_model\":\"gear-spring-damper-v2 (main geometry measured; nose x, tail, strut, pitch inertia surrogate)\"}}",
            bool_str(s.crashed)
        );
        o
    }
}

/// Rigid-body pitch after the first main-wheel contact. Aerodynamic pitch
/// acceleration uses the identified direct-control coefficients (already per
/// unit inertia); gear moments use the surrogate pitch inertia.
fn released_pitch_aero_accel(a: &AttitudeModel, q_pa: f64, aoa_rad: f64, rate_rad_s: f64) -> f64 {
    let kpa = q_pa.max(0.0) / 1000.0;
    let ctrl = a.direct_pitch_accel_max.min(a.direct_pitch_accel_per_kpa * kpa);
    let stiffness = a.direct_pitch_stiffness_per_kpa * kpa;
    let damping = 2.0 * a.direct_pitch_damping_ratio * stiffness.max(0.0).sqrt();
    // In ideal-servo mode the stick is released by the post-mains contract;
    // in direct mode the FCS stick input is honoured (it must also be zero).
    let input = if a.direct { a.input_pitch } else { 0.0 };
    ctrl * input - stiffness * (aoa_rad - a.direct_trim_aoa_rad) - damping * rate_rad_s
}
