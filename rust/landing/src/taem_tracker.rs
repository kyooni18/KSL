use crate::sys::*;
extern "C" {
    fn world_gravity_accel(w: *const KerbinWorld, position_i: Vec3) -> Vec3;
    fn world_atmosphere_velocity_i(w: *const KerbinWorld, position_i: Vec3) -> Vec3;
    fn aero_compute(
        w: *const KerbinWorld,
        a: *const AeroTable,
        position_i: Vec3,
        velocity_i: Vec3,
        ut: ::core::ffi::c_double,
        mass_kg: ::core::ffi::c_double,
        aoa_rad: ::core::ffi::c_double,
        bank_rad: ::core::ffi::c_double,
    ) -> AeroForces;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn remainder(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn v3_add(a: Vec3, b: Vec3) -> Vec3;
    fn v3_sub(a: Vec3, b: Vec3) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_dot(a: Vec3, b: Vec3) -> ::core::ffi::c_double;
    fn v3_cross(a: Vec3, b: Vec3) -> Vec3;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
    fn v3_normalized(a: Vec3) -> Vec3;
}
pub type size_t = usize;
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LandingSite {
    pub name: [::core::ffi::c_char; 128],
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub runway_heading: ::core::ffi::c_double,
    pub runway_length: ::core::ffi::c_double,
    pub runway_width: ::core::ffi::c_double,
    pub allow_reciprocal_runway: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleProfile {
    pub model_id: [::core::ffi::c_char; 96],
    pub estimated_lift_to_drag: ::core::ffi::c_double,
    pub estimated_ballistic_coefficient: ::core::ffi::c_double,
    pub entry_angle_of_attack: ::core::ffi::c_double,
    pub maximum_angle_of_attack: ::core::ffi::c_double,
    pub maximum_bank_angle: ::core::ffi::c_double,
    pub maximum_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_g_load: ::core::ffi::c_double,
    pub minimum_safe_speed: ::core::ffi::c_double,
    pub final_approach_speed: ::core::ffi::c_double,
    pub touchdown_speed: ::core::ffi::c_double,
    pub terminal_maximum_lift_angle_of_attack: ::core::ffi::c_double,
    pub allow_powered_approach: bool,
    pub maximum_approach_throttle: ::core::ffi::c_double,
    pub airbrake_action_group: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GuidanceSettings {
    pub target_deorbit_capture_radius: ::core::ffi::c_double,
    pub entry_interface_altitude_margin: ::core::ffi::c_double,
    pub target_entry_range: ::core::ffi::c_double,
    pub entry_range_reference_speed: ::core::ffi::c_double,
    pub target_entry_flight_path_angle: ::core::ffi::c_double,
    pub maximum_entry_flight_path_angle: ::core::ffi::c_double,
    pub target_post_burn_periapsis_altitude: ::core::ffi::c_double,
    pub deorbit_maximum_throttle: ::core::ffi::c_double,
    pub deorbit_throttle_ramp_duration: ::core::ffi::c_double,
    pub taem_interface_altitude: ::core::ffi::c_double,
    pub taem_interface_range: ::core::ffi::c_double,
    pub taem_force_handoff_speed: ::core::ffi::c_double,
    pub mm304_handoff_radius: ::core::ffi::c_double,
    pub mm304_perpendicular_heading_half_width: ::core::ffi::c_double,
    pub mm304_handoff_along_track: ::core::ffi::c_double,
    pub mm304_handoff_cross_track: ::core::ffi::c_double,
    pub mm305_min_altitude: ::core::ffi::c_double,
    pub mm305_max_altitude: ::core::ffi::c_double,
    pub mm305_target_mach: ::core::ffi::c_double,
    pub mm305_mach_half_width: ::core::ffi::c_double,
    pub hac_acquisition_altitude: ::core::ffi::c_double,
    pub hac_acquisition_mach: ::core::ffi::c_double,
    pub deorbit_timing_uncertainty: ::core::ffi::c_double,
    pub deorbit_thrust_uncertainty_fraction: ::core::ffi::c_double,
    pub deorbit_delta_v_uncertainty: ::core::ffi::c_double,
    pub deorbit_mass_uncertainty_fraction: ::core::ffi::c_double,
    pub deorbit_position_uncertainty: ::core::ffi::c_double,
    pub deorbit_velocity_uncertainty: ::core::ffi::c_double,
    pub deorbit_pointing_uncertainty: ::core::ffi::c_double,
    pub deorbit_atmosphere_uncertainty_fraction: ::core::ffi::c_double,
    pub deorbit_robust_minimum_pass_fraction: ::core::ffi::c_double,
    pub hac_radius: ::core::ffi::c_double,
    pub hac_look_ahead_angle: ::core::ffi::c_double,
    pub final_approach_distance: ::core::ffi::c_double,
    pub final_alignment_speed: ::core::ffi::c_double,
    pub final_glide_slope: ::core::ffi::c_double,
    pub taem_glide_slope: ::core::ffi::c_double,
    pub gear_deployment_altitude: ::core::ffi::c_double,
    pub flare_altitude: ::core::ffi::c_double,
    pub touchdown_sink_rate: ::core::ffi::c_double,
    pub guidance_rate: ::core::ffi::c_double,
    pub prediction_interval: ::core::ffi::c_double,
    pub s_turn_minimum_leg_duration: ::core::ffi::c_double,
    pub entry_roll_rate: ::core::ffi::c_double,
    pub entry_roll_acceleration: ::core::ffi::c_double,
    pub taem_roll_rate: ::core::ffi::c_double,
    pub approach_roll_rate: ::core::ffi::c_double,
    pub use_time_warp: bool,
    pub taem_energy_law: bool,
    pub sts_trajectory_delivery: bool,
    pub minimum_planning_lead_time: ::core::ffi::c_double,
    pub taem_hac_law: bool,
    pub terminal_speedbrake: bool,
    pub adaptive_flare: bool,
    pub touchdown_aim_along: ::core::ffi::c_double,
    pub rollout_parachutes: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereSample {
    pub density_kg_m3: ::core::ffi::c_double,
    pub pressure_pa: ::core::ffi::c_double,
    pub temperature_k: ::core::ffi::c_double,
    pub speed_of_sound_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroForces {
    pub lift_n: ::core::ffi::c_double,
    pub drag_n: ::core::ffi::c_double,
    pub side_n: ::core::ffi::c_double,
    pub force_i: Vec3,
    pub mach: ::core::ffi::c_double,
    pub dynamic_pressure_pa: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmospherePoint {
    pub altitude_m: ::core::ffi::c_double,
    pub sample: AtmosphereSample,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereTable {
    pub p: [AtmospherePoint; 2048],
    pub count: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KerbinWorld {
    pub radius_m: ::core::ffi::c_double,
    pub mu_m3_s2: ::core::ffi::c_double,
    pub rotation_rate_rad_s: ::core::ffi::c_double,
    pub atmosphere_top_m: ::core::ffi::c_double,
    pub rotation_phase_rad_at_ut0: ::core::ffi::c_double,
    pub orbital_rate_rad_s: ::core::ffi::c_double,
    pub solar_phase_rad_at_ut0: ::core::ffi::c_double,
    pub stock_spatial_atmosphere: bool,
    pub atmosphere: AtmosphereTable,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroBookPoint {
    pub q_pa: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub alpha_deg: ::core::ffi::c_double,
    pub lift_per_q_m2: ::core::ffi::c_double,
    pub drag_per_q_m2: ::core::ffi::c_double,
    pub support: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroTable {
    pub mach: [::core::ffi::c_double; 40],
    pub alpha_deg: [::core::ffi::c_double; 40],
    pub cl: [[::core::ffi::c_double; 40]; 40],
    pub cd: [[::core::ffi::c_double; 40]; 40],
    pub mach_count: size_t,
    pub alpha_count: size_t,
    pub reference_area_m2: ::core::ffi::c_double,
    pub book: [AeroBookPoint; 2048],
    pub book_count: size_t,
    pub book_enabled: bool,
    pub lift_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AttitudeModel {
    pub aoa_rad: ::core::ffi::c_double,
    pub bank_rad: ::core::ffi::c_double,
    pub aoa_rate_rad_s: ::core::ffi::c_double,
    pub bank_rate_rad_s: ::core::ffi::c_double,
    pub requested_aoa_rad: ::core::ffi::c_double,
    pub requested_bank_rad: ::core::ffi::c_double,
    pub cmd_aoa_rad: ::core::ffi::c_double,
    pub cmd_bank_rad: ::core::ffi::c_double,
    pub pitch_wn: ::core::ffi::c_double,
    pub pitch_zeta: ::core::ffi::c_double,
    pub roll_wn: ::core::ffi::c_double,
    pub roll_zeta: ::core::ffi::c_double,
    pub max_pitch_rate_rad_s: ::core::ffi::c_double,
    pub max_roll_rate_rad_s: ::core::ffi::c_double,
    pub max_pitch_accel_rad_s2: ::core::ffi::c_double,
    pub max_roll_accel_rad_s2: ::core::ffi::c_double,
    pub pitch_full_authority_q_pa: ::core::ffi::c_double,
    pub roll_full_authority_q_pa: ::core::ffi::c_double,
    pub direct: bool,
    pub direct_body_coordinates: bool,
    pub body_rates_initialized: bool,
    pub body_pitch_rate_rad_s: ::core::ffi::c_double,
    pub body_roll_rate_rad_s: ::core::ffi::c_double,
    pub body_yaw_rate_rad_s: ::core::ffi::c_double,
    pub input_pitch: ::core::ffi::c_double,
    pub input_roll: ::core::ffi::c_double,
    pub input_yaw: ::core::ffi::c_double,
    pub sideslip_rad: ::core::ffi::c_double,
    pub sideslip_rate_rad_s: ::core::ffi::c_double,
    pub direct_min_q_pa: ::core::ffi::c_double,
    pub direct_pitch_accel_per_kpa: ::core::ffi::c_double,
    pub direct_pitch_accel_max: ::core::ffi::c_double,
    pub direct_pitch_stiffness_per_kpa: ::core::ffi::c_double,
    pub direct_trim_aoa_rad: ::core::ffi::c_double,
    pub direct_pitch_damping_ratio: ::core::ffi::c_double,
    pub direct_roll_accel_per_kpa: ::core::ffi::c_double,
    pub direct_roll_accel_max: ::core::ffi::c_double,
    pub direct_roll_damping_s_inv: ::core::ffi::c_double,
    pub direct_yaw_accel_per_kpa: ::core::ffi::c_double,
    pub direct_yaw_accel_max: ::core::ffi::c_double,
    pub direct_yaw_stiffness_per_kpa: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalModel {
    pub schema_version: uint32_t,
    pub snapshot_id: uint64_t,
    pub atmosphere_source_digest: uint64_t,
    pub aero_source_digest: uint64_t,
    pub aero_book_source_digest: uint64_t,
    pub attitude_source_digest: uint64_t,
    pub replay_validated: bool,
    pub planet_frame_aligned: bool,
    pub world: KerbinWorld,
    pub aero: AeroTable,
    pub attitude: AttitudeModel,
    pub vehicle: VehicleProfile,
    pub site: LandingSite,
    pub guidance: GuidanceSettings,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalDynamicState {
    pub position_i_m: Vec3,
    pub velocity_i_mps: Vec3,
    pub attitude: AttitudeModel,
    pub mass_kg: ::core::ffi::c_double,
    pub ut_s: ::core::ffi::c_double,
    pub trim_aoa_ceiling_rad: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemGeometryState {
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub altitude_above_runway_m: ::core::ffi::c_double,
    pub ground_speed_mps: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub heading_error_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalControl {
    pub angle_of_attack_rad: ::core::ffi::c_double,
    pub bank_rad: ::core::ffi::c_double,
    pub dt_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemPathReference {
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub vertical_curvature_per_m: ::core::ffi::c_double,
    pub station_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTrackerOutput {
    pub valid: bool,
    pub lateral_authority_ok: bool,
    pub vertical_authority_ok: bool,
    pub control: TerminalControl,
    pub cross_track_error_m: ::core::ffi::c_double,
    pub course_error_deg: ::core::ffi::c_double,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub required_vertical_lift_mps2: ::core::ffi::c_double,
    pub delivered_vertical_lift_mps2: ::core::ffi::c_double,
    pub response_time_s: ::core::ffi::c_double,
}
static mut tracker_pi: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
unsafe extern "C" fn radians(
    mut degrees_0: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return degrees_0 * tracker_pi / 180.0f64;
}
unsafe extern "C" fn degrees(
    mut radians_value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return radians_value * 180.0f64 / tracker_pi;
}
unsafe extern "C" fn clamp_value(
    mut x: ::core::ffi::c_double,
    mut lo: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return fmax(lo, fmin(hi, x));
}
unsafe extern "C" fn attitude_authority(
    mut q_pa: ::core::ffi::c_double,
    mut full_authority_q_pa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(full_authority_q_pa > 0.0f64) {
        return 1.0f64;
    }
    return clamp_value(q_pa / full_authority_q_pa, 0.0f64, 1.0f64);
}
static mut command_envelope_fraction: ::core::ffi::c_double = 0.75f64;
unsafe extern "C" fn shape_attitude_command(
    mut target: ::core::ffi::c_double,
    mut previous: ::core::ffi::c_double,
    mut measured: ::core::ffi::c_double,
    mut rate_max: ::core::ffi::c_double,
    mut accel_max: ::core::ffi::c_double,
    mut wn: ::core::ffi::c_double,
    mut zeta: ::core::ffi::c_double,
    mut authority: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut wrap: bool,
    mut lead_response_cycles: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if previous.is_finite() as i32 == 0 {
        previous = if measured.is_finite() as i32 != 0 { measured } else { target };
    }
    if target.is_finite() as i32 == 0 {
        return target;
    }
    if !(dt > 0.0f64) {
        return previous;
    }
    let mut root: ::core::ffi::c_double = sqrt(fmax(authority, 0.0f64));
    let mut rate: ::core::ffi::c_double = command_envelope_fraction * rate_max * root;
    let mut accel: ::core::ffi::c_double = command_envelope_fraction * accel_max
        * fmax(authority, 0.0f64);
    if !(rate > 0.0f64) || !(accel > 0.0f64) {
        return previous;
    }
    let mut error: ::core::ffi::c_double = target - previous;
    if wrap {
        error = remainder(error, 2.0f64 * tracker_pi);
    }
    let mut allowed: ::core::ffi::c_double = fmin(
        rate,
        sqrt(2.0f64 * accel * fabs(error)),
    );
    let mut next: ::core::ffi::c_double = previous
        + clamp_value(error, -allowed * dt, allowed * dt);
    if measured.is_finite() as i32 != 0 {
        let mut response: ::core::ffi::c_double = if wn > 0.0f64 && zeta > 0.0f64 {
            1.0f64 / (wn * zeta)
        } else {
            1.0f64
        };
        let mut lead_horizon: ::core::ffi::c_double = fmax(
            dt,
            lead_response_cycles * response,
        );
        let mut lead_max: ::core::ffi::c_double = rate * lead_horizon;
        let mut lead_previous: ::core::ffi::c_double = previous - measured;
        let mut lead_next: ::core::ffi::c_double = next - measured;
        if wrap {
            lead_previous = remainder(lead_previous, 2.0f64 * tracker_pi);
            lead_next = remainder(lead_next, 2.0f64 * tracker_pi);
        }
        if fabs(lead_next) > lead_max && fabs(lead_next) > fabs(lead_previous) {
            next = if fabs(lead_previous) >= lead_max {
                previous
            } else {
                measured + (if lead_next > 0.0f64 { lead_max } else { -lead_max })
            };
        }
    }
    return if wrap as ::core::ffi::c_int != 0 {
        remainder(next, 2.0f64 * tracker_pi)
    } else {
        next
    };
}
unsafe extern "C" fn lateral_response_time(
    mut m: *const TerminalModel,
) -> ::core::ffi::c_double {
    let mut roll_wn: ::core::ffi::c_double = (*m).attitude.roll_wn;
    let mut roll_zeta: ::core::ffi::c_double = (*m).attitude.roll_zeta;
    let mut settling: ::core::ffi::c_double = if roll_wn > 0.0f64 && roll_zeta > 0.0f64 {
        4.0f64 / (roll_wn * roll_zeta)
    } else {
        8.0f64
    };
    let mut bank_limit: ::core::ffi::c_double = radians(
        fmin((*m).vehicle.maximum_bank_angle, 80.0f64),
    );
    let mut roll_rate: ::core::ffi::c_double = (*m).attitude.max_roll_rate_rad_s;
    let mut reversal: ::core::ffi::c_double = if roll_rate > 0.0f64 {
        2.0f64 * bank_limit / roll_rate
    } else {
        0.0f64
    };
    return clamp_value(fmax(settling, reversal), 1.0f64, 20.0f64);
}
unsafe extern "C" fn lateral_demand(
    mut g: *const TaemGeometryState,
    mut r: *const TaemPathReference,
    mut cross_track: ::core::ffi::c_double,
    mut course_error: ::core::ffi::c_double,
    mut response: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut speed: ::core::ffi::c_double = (*g).ground_speed_mps;
    let mut heading_error: ::core::ffi::c_double = radians(course_error);
    return speed * speed * (*r).curvature_right_per_m
        - 2.0f64 * speed * heading_error / response
        - cross_track / (response * response);
}
#[no_mangle]
pub unsafe extern "C" fn taem_tracker_lateral_demand(
    mut m: *const TerminalModel,
    mut g: *const TaemGeometryState,
    mut r: *const TaemPathReference,
) -> ::core::ffi::c_double {
    if m.is_null() || g.is_null() || r.is_null()
        || (*r).curvature_right_per_m.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut runway_relative_course: ::core::ffi::c_double = radians(
        (*r).course_deg - (*m).site.runway_heading,
    );
    let mut tx: ::core::ffi::c_double = cos(runway_relative_course);
    let mut ty: ::core::ffi::c_double = sin(runway_relative_course);
    let mut cross_track: ::core::ffi::c_double = ((*g).runway_along_m
        - (*r).runway_along_m) * -ty + ((*g).runway_cross_m - (*r).runway_cross_m) * tx;
    let mut course_error: ::core::ffi::c_double = remainder(
        (*g).course_deg - (*r).course_deg,
        360.0f64,
    );
    return lateral_demand(g, r, cross_track, course_error, lateral_response_time(m));
}
#[no_mangle]
pub unsafe extern "C" fn taem_tracker_vertical_lift_demand(
    mut m: *const TerminalModel,
    mut s: *const TerminalDynamicState,
    mut g: *const TaemGeometryState,
    mut desired_gamma_rate: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if m.is_null() || s.is_null() || g.is_null()
        || desired_gamma_rate.is_finite() as i32 == 0 || !((*g).airspeed_mps > 1.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut radius: ::core::ffi::c_double = v3_norm((*s).position_i_m);
    if !(radius > 1.0f64) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut gamma: ::core::ffi::c_double = radians((*g).flight_path_angle_deg);
    let mut airspeed: ::core::ffi::c_double = (*g).airspeed_mps;
    let mut up: Vec3 = v3_scale((*s).position_i_m, 1.0f64 / radius);
    let mut air: Vec3 = v3_sub(
        (*s).velocity_i_mps,
        world_atmosphere_velocity_i(&raw const (*m).world, (*s).position_i_m),
    );
    let mut air_direction: Vec3 = v3_normalized(air);
    let mut lift_up: Vec3 = v3_normalized(
        v3_sub(up, v3_scale(air_direction, v3_dot(up, air_direction))),
    );
    let mut omega: Vec3 = Vec3 {
        x: 0.0f64,
        y: 0.0f64,
        z: (*m).world.rotation_rate_rad_s,
    };
    let mut rotating_terms: Vec3 = v3_add(
        v3_scale(v3_cross(omega, air), 2.0f64),
        v3_cross(omega, v3_cross(omega, (*s).position_i_m)),
    );
    let mut free_accel: Vec3 = v3_sub(
        world_gravity_accel(&raw const (*m).world, (*s).position_i_m),
        rotating_terms,
    );
    let mut vertical_required: ::core::ffi::c_double = airspeed * desired_gamma_rate
        - airspeed * airspeed * cos(gamma) / radius - v3_dot(free_accel, lift_up);
    return vertical_required;
}
#[no_mangle]
pub unsafe extern "C" fn taem_tracker_update(
    mut m: *const TerminalModel,
    mut s: *const TerminalDynamicState,
    mut g: *const TaemGeometryState,
    mut r: *const TaemPathReference,
    mut dt: ::core::ffi::c_double,
) -> TaemTrackerOutput {
    let mut out: TaemTrackerOutput = TaemTrackerOutput {
        valid: false,
        lateral_authority_ok: false,
        vertical_authority_ok: false,
        control: TerminalControl {
            angle_of_attack_rad: 0.,
            bank_rad: 0.,
            dt_s: 0.,
        },
        cross_track_error_m: 0.,
        course_error_deg: 0.,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        required_vertical_lift_mps2: 0.,
        delivered_vertical_lift_mps2: 0.,
        response_time_s: 0.,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemTrackerOutput>() as size_t,
    );
    if m.is_null() || s.is_null() || g.is_null() || r.is_null() || !(dt >= 0.0f64)
        || !((*g).ground_speed_mps > 0.0f64) || !((*g).airspeed_mps > 0.0f64)
        || (*r).curvature_right_per_m.is_finite() as i32 == 0
        || (*r).flight_path_angle_deg.is_finite() as i32 == 0
        || (*r).vertical_curvature_per_m.is_finite() as i32 == 0
    {
        return out;
    }
    let mut runway_relative_course: ::core::ffi::c_double = radians(
        (*r).course_deg - (*m).site.runway_heading,
    );
    let mut tx: ::core::ffi::c_double = cos(runway_relative_course);
    let mut ty: ::core::ffi::c_double = sin(runway_relative_course);
    let mut nx: ::core::ffi::c_double = -ty;
    let mut ny: ::core::ffi::c_double = tx;
    let mut dx: ::core::ffi::c_double = (*g).runway_along_m - (*r).runway_along_m;
    let mut dy: ::core::ffi::c_double = (*g).runway_cross_m - (*r).runway_cross_m;
    let mut cross_track: ::core::ffi::c_double = dx * nx + dy * ny;
    let mut course_error: ::core::ffi::c_double = remainder(
        (*g).course_deg - (*r).course_deg,
        360.0f64,
    );
    let mut response: ::core::ffi::c_double = lateral_response_time(m);
    let mut lateral: ::core::ffi::c_double = lateral_demand(
        g,
        r,
        cross_track,
        course_error,
        response,
    );
    let mut altitude: ::core::ffi::c_double = v3_norm((*s).position_i_m)
        - (*m).world.radius_m;
    let mut radius: ::core::ffi::c_double = (*m).world.radius_m + altitude;
    if !(radius > 0.0f64) {
        return out;
    }
    let mut gravity: ::core::ffi::c_double = (*m).world.mu_m3_s2 / (radius * radius);
    let mut gamma: ::core::ffi::c_double = radians((*g).flight_path_angle_deg);
    let mut target_gamma: ::core::ffi::c_double = radians((*r).flight_path_angle_deg);
    if (*r).altitude_m.is_finite() as i32 != 0 && (*g).ground_speed_mps > 1.0f64 {
        let mut altitude_error: ::core::ffi::c_double = (*r).altitude_m
            - ((*m).site.altitude + (*g).altitude_above_runway_m);
        target_gamma
            += clamp_value(
                atan(altitude_error / (15.0f64 * (*g).ground_speed_mps)),
                -radians(6.0f64),
                radians(6.0f64),
            );
    }
    let mut pitch_wn: ::core::ffi::c_double = (*m).attitude.pitch_wn;
    let mut pitch_zeta: ::core::ffi::c_double = (*m).attitude.pitch_zeta;
    let mut pitch_response: ::core::ffi::c_double = if pitch_wn > 0.0f64
        && pitch_zeta > 0.0f64
    {
        clamp_value(4.0f64 / (pitch_wn * pitch_zeta), 1.0f64, 20.0f64)
    } else {
        8.0f64
    };
    let mut desired_gamma_rate: ::core::ffi::c_double = (*g).ground_speed_mps
        * (*r).vertical_curvature_per_m + (target_gamma - gamma) / pitch_response;
    desired_gamma_rate = clamp_value(
        desired_gamma_rate,
        -radians(8.0f64),
        radians(8.0f64),
    );
    let mut vertical_required: ::core::ffi::c_double = fmax(
        0.0f64,
        taem_tracker_vertical_lift_demand(m, s, g, desired_gamma_rate),
    );
    let mut aoa_max: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_angle_of_attack,
        (*m).aero.alpha_deg[(*m).aero.alpha_count.wrapping_sub(1 as size_t) as usize],
    );
    let mut flow: AeroForces = aero_compute(
        &raw const (*m).world,
        &raw const (*m).aero,
        (*s).position_i_m,
        (*s).velocity_i_mps,
        (*s).ut_s,
        (*s).mass_kg,
        0.0f64,
        0.0f64,
    );
    if flow.mach < 1.0f64
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        aoa_max = fmin(aoa_max, (*m).vehicle.terminal_maximum_lift_angle_of_attack);
    }
    if (*s).trim_aoa_ceiling_rad > 0.0f64 {
        aoa_max = fmin(aoa_max, degrees((*s).trim_aoa_ceiling_rad));
    }
    let mut bank_max: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_bank_angle,
        80.0f64,
    );
    let mut g_limit: ::core::ffi::c_double = fmax(0.0f64, (*m).vehicle.maximum_g_load)
        * gravity;
    let mut best_error: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut chosen_aoa: ::core::ffi::c_double = 0.0f64;
    let mut chosen_bank: ::core::ffi::c_double = 0.0f64;
    let mut chosen_vertical: ::core::ffi::c_double = 0.0f64;
    let mut available_lateral: ::core::ffi::c_double = 0.0f64;
    let mut chosen_lateral_feasible: bool = false_0 != 0;
    let mut best_glide_ratio: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut glide_aoa: ::core::ffi::c_double = 0.0f64;
    let mut glide_bank: ::core::ffi::c_double = 0.0f64;
    let mut glide_vertical: ::core::ffi::c_double = 0.0f64;
    let mut have_glide_candidate: bool = false_0 != 0;
    let mut refinement_low: ::core::ffi::c_double = 0.0f64;
    let mut refinement_high: ::core::ffi::c_double = 0.0f64;
    let mut pass: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while pass < 2 as ::core::ffi::c_int {
        if pass == 1 as ::core::ffi::c_int {
            refinement_low = fmax(0.0f64, chosen_aoa - 0.5f64);
            refinement_high = fmin(aoa_max, chosen_aoa + 0.5f64);
        }
        let mut first_aoa: ::core::ffi::c_double = if pass == 0 as ::core::ffi::c_int {
            0.0f64
        } else {
            refinement_low
        };
        let mut last_aoa: ::core::ffi::c_double = if pass == 0 as ::core::ffi::c_int {
            aoa_max
        } else {
            refinement_high
        };
        let mut step_aoa: ::core::ffi::c_double = if pass == 0 as ::core::ffi::c_int {
            0.5f64
        } else {
            0.1f64
        };
        let mut aoa_deg: ::core::ffi::c_double = first_aoa;
        while aoa_deg <= last_aoa + 1e-9f64 {
            let mut forces: AeroForces = aero_compute(
                &raw const (*m).world,
                &raw const (*m).aero,
                (*s).position_i_m,
                (*s).velocity_i_mps,
                (*s).ut_s,
                (*s).mass_kg,
                radians(aoa_deg),
                0.0f64,
            );
            if !(forces.lift_n.is_finite() as i32 == 0) {
                let mut lift: ::core::ffi::c_double = fmin(
                    fabs(forces.lift_n) / (*s).mass_kg,
                    g_limit,
                );
                if lift > 1e-8f64 {
                    let mut lateral_capacity: ::core::ffi::c_double = lift
                        * sin(radians(bank_max));
                    available_lateral = fmax(available_lateral, lateral_capacity);
                    let mut lateral_feasible: bool = fabs(lateral)
                        <= lateral_capacity + 1e-9f64;
                    let mut bank: ::core::ffi::c_double = 0.;
                    if lateral_feasible {
                        bank = degrees(
                            asin(clamp_value(lateral / lift, -1.0f64, 1.0f64)),
                        );
                    } else {
                        bank = if lateral >= 0.0f64 { bank_max } else { -bank_max };
                    }
                    let mut delivered_lateral: ::core::ffi::c_double = lift
                        * sin(radians(bank));
                    let mut delivered_vertical: ::core::ffi::c_double = lift
                        * cos(radians(bank));
                    let mut vertical_error: ::core::ffi::c_double = fabs(
                        delivered_vertical - vertical_required,
                    );
                    let mut lateral_error: ::core::ffi::c_double = fabs(
                        delivered_lateral - lateral,
                    );
                    let mut score: ::core::ffi::c_double = hypot(
                        vertical_error,
                        lateral_error,
                    );
                    if pass == 0 as ::core::ffi::c_int
                        && lateral_feasible as ::core::ffi::c_int != 0
                        && forces.drag_n.is_finite() as i32 != 0
                        && forces.drag_n > 1e-6f64 && forces.lift_n > 0.0f64
                    {
                        let mut glide_ratio: ::core::ffi::c_double = forces.lift_n
                            / forces.drag_n;
                        if glide_ratio > best_glide_ratio {
                            best_glide_ratio = glide_ratio;
                            glide_aoa = aoa_deg;
                            glide_bank = bank;
                            glide_vertical = delivered_vertical;
                            have_glide_candidate = true_0 != 0;
                        }
                    }
                    let mut choose: bool = false_0 != 0;
                    if lateral_feasible as ::core::ffi::c_int != 0
                        && !chosen_lateral_feasible
                    {
                        choose = true_0 != 0;
                    } else if lateral_feasible as ::core::ffi::c_int
                        == chosen_lateral_feasible as ::core::ffi::c_int
                        && score < best_error
                    {
                        choose = true_0 != 0;
                    }
                    if choose {
                        best_error = score;
                        chosen_aoa = aoa_deg;
                        chosen_bank = bank;
                        chosen_vertical = delivered_vertical;
                        chosen_lateral_feasible = lateral_feasible;
                    }
                }
            }
            aoa_deg += step_aoa;
        }
        pass += 1;
    }
    if best_error.is_finite() as i32 == 0 {
        return out;
    }
    let mut vertical_tolerance: ::core::ffi::c_double = fmax(
        0.25f64,
        0.1f64 * fmax(1.0f64, vertical_required),
    );
    let mut vertical_recovery_requested: bool = target_gamma > gamma + radians(0.5f64);
    if chosen_lateral_feasible as ::core::ffi::c_int != 0
        && have_glide_candidate as ::core::ffi::c_int != 0
        && chosen_vertical < vertical_required - vertical_tolerance
        && !vertical_recovery_requested
    {
        chosen_aoa = glide_aoa;
        chosen_bank = glide_bank;
        chosen_vertical = glide_vertical;
    }
    let mut q: ::core::ffi::c_double = fmax(0.0f64, flow.dynamic_pressure_pa);
    let mut bank_target: ::core::ffi::c_double = shape_attitude_command(
        radians(chosen_bank),
        (*s).attitude.requested_bank_rad,
        (*s).attitude.bank_rad,
        (*m).attitude.max_roll_rate_rad_s,
        (*m).attitude.max_roll_accel_rad_s2,
        (*m).attitude.roll_wn,
        (*m).attitude.roll_zeta,
        attitude_authority(q, (*m).attitude.roll_full_authority_q_pa),
        dt,
        true_0 != 0,
        1.0f64,
    );
    let mut aoa_target: ::core::ffi::c_double = shape_attitude_command(
        radians(chosen_aoa),
        (*s).attitude.requested_aoa_rad,
        (*s).attitude.aoa_rad,
        (*m).attitude.max_pitch_rate_rad_s,
        (*m).attitude.max_pitch_accel_rad_s2,
        (*m).attitude.pitch_wn,
        (*m).attitude.pitch_zeta,
        attitude_authority(q, (*m).attitude.pitch_full_authority_q_pa),
        dt,
        false_0 != 0,
        4.0f64,
    );
    let mut tracker_diag: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !tracker_diag.is_null()
        && strcmp(tracker_diag, b"2\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int && dt > 0.0f64 && dt <= 0.2f64
    {
        let mut pitch_authority: ::core::ffi::c_double = attitude_authority(
            q,
            (*m).attitude.pitch_full_authority_q_pa,
        );
        fprintf(
            stderr,
            b"TAEM pitch trace: gamma=%.2f target=%.2f gammaRate=%.2f rawAoa=%.2f shapedAoa=%.2f actualAoa=%.2f prevAoa=%.2f aoaMax=%.2f q=%.0f authority=%.2f vertReq=%.2f vertRaw=%.2f glideAoa=%.2f recovery=%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            degrees(gamma),
            degrees(target_gamma),
            degrees(desired_gamma_rate),
            chosen_aoa,
            degrees(aoa_target),
            degrees((*s).attitude.aoa_rad),
            degrees((*s).attitude.requested_aoa_rad),
            aoa_max,
            q,
            pitch_authority,
            vertical_required,
            chosen_vertical,
            glide_aoa,
            if vertical_recovery_requested as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
        );
    }
    out.valid = true_0 != 0;
    out.control = TerminalControl {
        angle_of_attack_rad: aoa_target,
        bank_rad: bank_target,
        dt_s: dt,
    };
    out.cross_track_error_m = cross_track;
    out.course_error_deg = course_error;
    out.required_lateral_accel_mps2 = lateral;
    out.available_lateral_accel_mps2 = available_lateral;
    out.required_vertical_lift_mps2 = vertical_required;
    out.delivered_vertical_lift_mps2 = chosen_vertical;
    out.response_time_s = fmax(response, pitch_response);
    out.lateral_authority_ok = fabs(lateral) <= available_lateral + 1e-6f64;
    out.vertical_authority_ok = fabs(chosen_vertical - vertical_required)
        <= fmax(0.25f64, 0.1f64 * fmax(1.0f64, vertical_required));
    return out;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
