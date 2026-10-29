use crate::sys::*;
extern "C" {
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn ceil(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn local_offsets(
        origin: GeoPoint,
        p: GeoPoint,
        radius: ::core::ffi::c_double,
        east: *mut ::core::ffi::c_double,
        north: *mut ::core::ffi::c_double,
    );
    fn trajectory_clear(t: *mut Trajectory);
    fn decision_guidance_attitude_limits(
        telemetry: *const Telemetry,
        settings: *const GuidanceSettings,
        phase: GuidancePhase,
        pitch_axis: bool,
    ) -> GuidanceAttitudeLimits;
    fn decision_axis_capture_time(
        error_deg: ::core::ffi::c_double,
        rate_deg_s: ::core::ffi::c_double,
        acceleration_limit_deg_s2: ::core::ffi::c_double,
        rate_limit_deg_s: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_lateral_bank_limit(
        t: *const Telemetry,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GeoPoint {
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
}
pub type GuidancePhase = ::core::ffi::c_uint;
pub const PHASE_ATTITUDE_RECOVERY: GuidancePhase = 18;
pub const PHASE_FAULT: GuidancePhase = 17;
pub const PHASE_ABORT: GuidancePhase = 16;
pub const PHASE_PAUSED: GuidancePhase = 15;
pub const PHASE_COMPLETE: GuidancePhase = 14;
pub const PHASE_ROLLOUT: GuidancePhase = 13;
pub const PHASE_TOUCHDOWN: GuidancePhase = 12;
pub const PHASE_FLARE: GuidancePhase = 11;
pub const PHASE_FINAL: GuidancePhase = 10;
pub const PHASE_HEADING_ALIGNMENT: GuidancePhase = 9;
pub const PHASE_TAEM: GuidancePhase = 8;
pub const PHASE_ENTRY_ENERGY: GuidancePhase = 7;
pub const PHASE_ENTRY_INTERFACE: GuidancePhase = 6;
pub const PHASE_DEORBIT_BURN: GuidancePhase = 5;
pub const PHASE_BURN_SETUP: GuidancePhase = 4;
pub const PHASE_COAST: GuidancePhase = 3;
pub const PHASE_CALIBRATION: GuidancePhase = 2;
pub const PHASE_PLANNING: GuidancePhase = 1;
pub const PHASE_IDLE: GuidancePhase = 0;
pub type NavballSpeedMode = ::core::ffi::c_uint;
pub const SPEED_SURFACE: NavballSpeedMode = 2;
pub const SPEED_ORBIT: NavballSpeedMode = 1;
pub const SPEED_UNCHANGED: NavballSpeedMode = 0;
pub type TrajectoryKind = ::core::ffi::c_uint;
pub const TRAJ_REFERENCE: TrajectoryKind = 2;
pub const TRAJ_ACTUAL: TrajectoryKind = 1;
pub const TRAJ_PLANNED: TrajectoryKind = 0;
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
pub struct TrajectoryPoint {
    pub ut: ::core::ffi::c_double,
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub speed: ::core::ffi::c_double,
    pub phase: GuidancePhase,
    pub kind: TrajectoryKind,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Trajectory {
    pub points: *mut TrajectoryPoint,
    pub count: size_t,
    pub capacity: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AttitudeResponseModel {
    pub pitch_valid: bool,
    pub roll_valid: bool,
    pub pitch_natural_frequency_s_inv: ::core::ffi::c_double,
    pub pitch_damping_ratio: ::core::ffi::c_double,
    pub roll_natural_frequency_s_inv: ::core::ffi::c_double,
    pub roll_damping_ratio: ::core::ffi::c_double,
    pub maximum_pitch_rate_deg_s: ::core::ffi::c_double,
    pub maximum_roll_rate_deg_s: ::core::ffi::c_double,
    pub maximum_pitch_accel_deg_s2: ::core::ffi::c_double,
    pub maximum_roll_accel_deg_s2: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelemetryControlSurface {
    pub part_name: [::core::ffi::c_char; 64],
    pub role: [::core::ffi::c_char; 16],
    pub deployed: bool,
    pub deflection_override: bool,
    pub override_deflection: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Telemetry {
    pub ut: ::core::ffi::c_double,
    pub vessel_name: [::core::ffi::c_char; 128],
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub mean_altitude: ::core::ffi::c_double,
    pub radar_altitude: ::core::ffi::c_double,
    pub vertical_speed: ::core::ffi::c_double,
    pub horizontal_speed: ::core::ffi::c_double,
    pub surface_speed: ::core::ffi::c_double,
    pub true_air_speed: ::core::ffi::c_double,
    pub atmospheric_density: ::core::ffi::c_double,
    pub speed_of_sound: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub heading: ::core::ffi::c_double,
    pub ground_track_heading: ::core::ffi::c_double,
    pub course_to_site_error: ::core::ffi::c_double,
    pub pitch: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub pitch_rate: ::core::ffi::c_double,
    pub roll_rate: ::core::ffi::c_double,
    pub heading_rate: ::core::ffi::c_double,
    pub angle_of_attack_rate: ::core::ffi::c_double,
    pub has_angle_of_attack_rate: bool,
    pub has_body_pitch_rate: bool,
    pub has_body_roll_rate: bool,
    pub has_body_yaw_rate: bool,
    pub has_course_rate: bool,
    pub body_pitch_rate: ::core::ffi::c_double,
    pub body_roll_rate: ::core::ffi::c_double,
    pub body_yaw_rate: ::core::ffi::c_double,
    pub course_rate: ::core::ffi::c_double,
    pub autopilot_error: ::core::ffi::c_double,
    pub has_attitude_quaternion: bool,
    pub attitude_quaternion: [::core::ffi::c_double; 4],
    pub attitude_reference_frame: [::core::ffi::c_char; 48],
    pub has_command_pitch_error: bool,
    pub has_command_roll_error: bool,
    pub has_command_heading_error: bool,
    pub command_pitch_error: ::core::ffi::c_double,
    pub command_roll_error: ::core::ffi::c_double,
    pub command_heading_error: ::core::ffi::c_double,
    pub has_pid_gains: bool,
    pub autopilot_pitch_pid_gains: [::core::ffi::c_double; 3],
    pub autopilot_roll_pid_gains: [::core::ffi::c_double; 3],
    pub autopilot_yaw_pid_gains: [::core::ffi::c_double; 3],
    pub navball_speed_mode: NavballSpeedMode,
    pub attitude_response: AttitudeResponseModel,
    pub angle_of_attack: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub static_pressure: ::core::ffi::c_double,
    pub g_force: ::core::ffi::c_double,
    pub stall_fraction: ::core::ffi::c_double,
    pub lift_force: ::core::ffi::c_double,
    pub drag_force: ::core::ffi::c_double,
    pub stall_fraction_is_measured: bool,
    pub has_force_vectors: bool,
    pub has_center_of_mass: bool,
    pub has_center_of_mass_root: bool,
    pub physics_sample_valid: bool,
    pub center_of_mass_root: Vector3,
    pub lift_vector: Vector3,
    pub drag_vector: Vector3,
    pub center_of_mass: Vector3,
    pub dry_mass: ::core::ffi::c_double,
    pub physics_confidence: ::core::ffi::c_double,
    pub physics_authority: [::core::ffi::c_double; 3],
    pub physics_authority_confidence: [::core::ffi::c_double; 3],
    pub physics_mass_flow: ::core::ffi::c_double,
    pub physics_model_residual: ::core::ffi::c_double,
    pub physics_model_residual_confidence: ::core::ffi::c_double,
    pub physics_certified_uncertainty: ::core::ffi::c_double,
    pub physics_force_residual_per_q: Vector3,
    pub physics_force_residual_sigma_per_q: Vector3,
    pub physics_force_residual_confidence: ::core::ffi::c_double,
    pub physics_airbrake_model_available: bool,
    pub physics_airbrake_model_confidence: ::core::ffi::c_double,
    pub physics_airbrake_drag_accel: ::core::ffi::c_double,
    pub physics_samples: ::core::ffi::c_uint,
    pub physics_live_samples: ::core::ffi::c_uint,
    pub predicted_physics_observed_seconds: ::core::ffi::c_double,
    pub predicted_physics_fallback_seconds: ::core::ffi::c_double,
    pub predicted_shadow_guidance: bool,
    pub predicted_terminal_policy_feasible: bool,
    pub predicted_terminal_survivability_stress_score: ::core::ffi::c_double,
    pub predicted_physics_relative_uncertainty: ::core::ffi::c_double,
    pub predicted_uncertainty_scenarios: ::core::ffi::c_uint,
    pub predicted_raw_published_position_delta_30s: ::core::ffi::c_double,
    pub predicted_raw_published_position_delta_60s: ::core::ffi::c_double,
    pub predicted_raw_published_position_delta_120s: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub available_thrust: ::core::ffi::c_double,
    pub current_thrust: ::core::ffi::c_double,
    pub apoapsis_altitude: ::core::ffi::c_double,
    pub periapsis_altitude: ::core::ffi::c_double,
    pub orbit_period: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub has_controls: bool,
    pub control_pitch: ::core::ffi::c_double,
    pub control_roll: ::core::ffi::c_double,
    pub control_yaw: ::core::ffi::c_double,
    pub has_torque: bool,
    pub available_pitch_torque: ::core::ffi::c_double,
    pub available_roll_torque: ::core::ffi::c_double,
    pub available_yaw_torque: ::core::ffi::c_double,
    pub has_inertia: bool,
    pub pitch_moment_of_inertia: ::core::ffi::c_double,
    pub roll_moment_of_inertia: ::core::ffi::c_double,
    pub yaw_moment_of_inertia: ::core::ffi::c_double,
    pub has_loop_wall_delta: bool,
    pub has_telemetry_latency: bool,
    pub has_guidance_compute: bool,
    pub has_apply_latency: bool,
    pub has_control_loop: bool,
    pub has_warp: bool,
    pub warp_rails: bool,
    pub physics_warp_factor: ::core::ffi::c_int,
    pub warp_rate: ::core::ffi::c_double,
    pub loop_wall_delta_ms: ::core::ffi::c_double,
    pub telemetry_latency_ms: ::core::ffi::c_double,
    pub guidance_compute_ms: ::core::ffi::c_double,
    pub apply_latency_ms: ::core::ffi::c_double,
    pub control_loop_ms: ::core::ffi::c_double,
    pub has_rpc_budget: bool,
    pub rpc_read_calls: ::core::ffi::c_uint,
    pub rpc_read_wire_requests: ::core::ffi::c_uint,
    pub rpc_apply_calls: ::core::ffi::c_uint,
    pub rpc_apply_wire_requests: ::core::ffi::c_uint,
    pub rpc_total_calls: ::core::ffi::c_uint,
    pub rpc_total_wire_requests: ::core::ffi::c_uint,
    pub gear: bool,
    pub brakes: bool,
    pub has_airbrakes: bool,
    pub airbrakes: bool,
    pub has_sas_state: bool,
    pub sas_enabled: bool,
    pub has_control_surface_state: bool,
    pub control_surface_state_ut: ::core::ffi::c_double,
    pub control_surface_count: size_t,
    pub control_surfaces: [TelemetryControlSurface; 16],
    pub has_main_gear_grounded: bool,
    pub main_gear_grounded: bool,
    pub has_nose_gear_grounded: bool,
    pub nose_gear_grounded: bool,
    pub vessel_situation: [::core::ffi::c_char; 64],
    pub range_to_site: ::core::ffi::c_double,
    pub bearing_to_site: ::core::ffi::c_double,
    pub heading_error: ::core::ffi::c_double,
    pub runway_along_track: ::core::ffi::c_double,
    pub runway_cross_track: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub estimated_lift_to_drag: ::core::ffi::c_double,
    pub estimated_ballistic_coefficient: ::core::ffi::c_double,
    pub aerodynamic_confidence: ::core::ffi::c_double,
    pub calibrated_best_glide_angle_of_attack: ::core::ffi::c_double,
    pub calibrated_stall_speed: ::core::ffi::c_double,
    pub predicted_miss_distance: ::core::ffi::c_double,
    pub predicted_taem_distance: ::core::ffi::c_double,
    pub predicted_taem_range_error: ::core::ffi::c_double,
    pub predicted_entry_range: ::core::ffi::c_double,
    pub predicted_entry_flight_path_angle: ::core::ffi::c_double,
    pub predicted_taem_speed: ::core::ffi::c_double,
    pub predicted_taem_energy_error: ::core::ffi::c_double,
    pub predicted_peak_dynamic_pressure: ::core::ffi::c_double,
    pub predicted_peak_g_load: ::core::ffi::c_double,
    pub predicted_s_turn_reversals: ::core::ffi::c_double,
    pub trajectory_density_scale: ::core::ffi::c_double,
    pub trajectory_drag_scale: ::core::ffi::c_double,
    pub trajectory_lift_scale: ::core::ffi::c_double,
    pub bank_effectiveness: ::core::ffi::c_double,
    pub trajectory_calibration_confidence: ::core::ffi::c_double,
    pub trajectory_altitude_residual: ::core::ffi::c_double,
    pub trajectory_speed_residual: ::core::ffi::c_double,
    pub trajectory_range_residual: ::core::ffi::c_double,
    pub energy_excess_range: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JerkLimiter {
    pub value: ::core::ffi::c_double,
    pub rate: ::core::ffi::c_double,
    pub has_value: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HACGuidance {
    pub heading: ::core::ffi::c_double,
    pub bank: ::core::ffi::c_double,
    pub desired_altitude: ::core::ffi::c_double,
    pub distance_final: ::core::ffi::c_double,
    pub arc_remaining: ::core::ffi::c_double,
    pub radial_error: ::core::ffi::c_double,
    pub course_error: ::core::ffi::c_double,
    pub angle: ::core::ffi::c_double,
    pub lateral_acceleration: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HACPoint2 {
    pub e: ::core::ffi::c_double,
    pub n: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GuidanceAttitudeLimits {
    pub rate_deg_s: ::core::ffi::c_double,
    pub accel_deg_s2: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
#[no_mangle]
pub unsafe extern "C" fn hac_course_rate_cap_for_speed(
    mut true_air_speed: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return 1.50f64
        + 1.50f64 * clampd((220.0f64 - true_air_speed) / 70.0f64, 0.0f64, 1.0f64);
}
#[no_mangle]
pub unsafe extern "C" fn hac_course_rate_cap(
    mut t: *const Telemetry,
) -> ::core::ffi::c_double {
    return hac_course_rate_cap_for_speed(
        if !t.is_null() { (*t).true_air_speed } else { 0.0f64 },
    );
}
#[no_mangle]
pub unsafe extern "C" fn guidance_lateral_speed(
    mut t: *const Telemetry,
) -> ::core::ffi::c_double {
    if t.is_null() {
        return 0.0f64;
    }
    if (*t).horizontal_speed.is_finite() as i32 != 0 && (*t).horizontal_speed > 1.0f64 {
        return (*t).horizontal_speed;
    }
    if (*t).surface_speed.is_finite() as i32 != 0 && (*t).surface_speed > 1.0f64 {
        return (*t).surface_speed;
    }
    if (*t).true_air_speed.is_finite() as i32 != 0 && (*t).true_air_speed > 1.0f64 {
        let mut c: ::core::ffi::c_double = if (*t).flight_path_angle.is_finite() as i32
            != 0
        {
            fabs(cos((*t).flight_path_angle * DEG2RAD))
        } else {
            1.0f64
        };
        return fmax(1.0f64, (*t).true_air_speed * c);
    }
    return 1.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn controlled_roll_rate(
    mut t: *const Telemetry,
) -> ::core::ffi::c_double {
    return if !t.is_null() && (*t).roll_rate.is_finite() as i32 != 0 {
        (*t).roll_rate
    } else {
        0.0f64
    };
}
#[no_mangle]
pub unsafe extern "C" fn controlled_aoa_rate(
    mut t: *const Telemetry,
) -> ::core::ffi::c_double {
    return if !t.is_null() && (*t).has_angle_of_attack_rate as ::core::ffi::c_int != 0
        && (*t).angle_of_attack_rate.is_finite() as i32 != 0
    {
        (*t).angle_of_attack_rate
    } else {
        0.0f64
    };
}
#[no_mangle]
pub unsafe extern "C" fn stabilized_attitude_limits(
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut phase: GuidancePhase,
    mut pitch_axis: bool,
) -> GuidanceAttitudeLimits {
    return decision_guidance_attitude_limits(t, s, phase, pitch_axis);
}
#[no_mangle]
pub unsafe extern "C" fn seed_stabilized_limiter(
    mut limiter: *mut JerkLimiter,
    mut value: ::core::ffi::c_double,
    mut rate: ::core::ffi::c_double,
    mut angle: bool,
) {
    if limiter.is_null() || (*limiter).has_value as ::core::ffi::c_int != 0
        || value.is_finite() as i32 == 0
    {
        return;
    }
    (*limiter).has_value = true_0 != 0;
    (*limiter).value = if angle as ::core::ffi::c_int != 0 {
        norm_deg(value)
    } else {
        value
    };
    (*limiter).rate = if rate.is_finite() as i32 != 0 { rate } else { 0.0f64 };
}
#[no_mangle]
pub unsafe extern "C" fn roll_capture_time(
    mut t: *const Telemetry,
    mut target_bank: ::core::ffi::c_double,
    mut configured_rate_limit: ::core::ffi::c_double,
    mut configured_accel_limit: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || (*t).roll.is_finite() as i32 == 0
        || target_bank.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut rate_limit: ::core::ffi::c_double = fabs(configured_rate_limit);
    let mut accel_limit: ::core::ffi::c_double = fabs(configured_accel_limit);
    if (*t).attitude_response.roll_valid {
        rate_limit = (*t).attitude_response.maximum_roll_rate_deg_s;
        accel_limit = (*t).attitude_response.maximum_roll_accel_deg_s2;
    }
    if !(rate_limit > DBL_MIN) || !(accel_limit > DBL_MIN)
        || rate_limit.is_finite() as i32 == 0 || accel_limit.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut error: ::core::ffi::c_double = norm_signed_deg(
        target_bank - norm_signed_deg((*t).roll),
    );
    return decision_axis_capture_time(
        error,
        controlled_roll_rate(t),
        accel_limit,
        rate_limit,
    );
}
#[no_mangle]
pub unsafe extern "C" fn hac_response_lead_time(
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut nominal_bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if s.is_null() {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return roll_capture_time(
        t,
        nominal_bank,
        (*s).taem_roll_rate,
        (*s).entry_roll_acceleration,
    );
}
#[no_mangle]
pub unsafe extern "C" fn hac_projected_local_state(
    mut t: *const Telemetry,
    mut site: *const LandingSite,
    mut planet_radius: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
    mut seconds: ::core::ffi::c_double,
    mut target_bank: ::core::ffi::c_double,
    mut roll_rate_limit: ::core::ffi::c_double,
    mut roll_accel_limit: ::core::ffi::c_double,
    mut lift_accel: ::core::ffi::c_double,
    mut bank_effectiveness: ::core::ffi::c_double,
    mut drag_accel: ::core::ffi::c_double,
    mut out_e: *mut ::core::ffi::c_double,
    mut out_n: *mut ::core::ffi::c_double,
    mut out_course: *mut ::core::ffi::c_double,
) {
    let mut origin: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut current: GeoPoint = GeoPoint {
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        altitude: (*t).mean_altitude,
    };
    let mut e: ::core::ffi::c_double = 0.0f64;
    let mut n: ::core::ffi::c_double = 0.0f64;
    local_offsets(origin, current, planet_radius, &raw mut e, &raw mut n);
    seconds = fmax(0.0f64, seconds);
    target_bank = clampd(target_bank, -70.0f64, 70.0f64);
    roll_rate_limit = fmax(1.0f64, fabs(roll_rate_limit));
    roll_accel_limit = fmax(0.5f64, fabs(roll_accel_limit));
    bank_effectiveness = clampd(bank_effectiveness, 0.35f64, 1.8f64);
    lift_accel = fmax(0.0f64, lift_accel);
    drag_accel = clampd(drag_accel, -40.0f64, 40.0f64);
    let mut bank: ::core::ffi::c_double = norm_signed_deg((*t).roll);
    let mut bank_rate: ::core::ffi::c_double = clampd(
        controlled_roll_rate(t),
        -roll_rate_limit * 1.4f64,
        roll_rate_limit * 1.4f64,
    );
    let mut course_rate: ::core::ffi::c_double = if (*t).course_rate.is_finite() as i32
        != 0
    {
        clampd((*t).course_rate, -3.5f64, 3.5f64)
    } else {
        0.0f64
    };
    let mut tas0: ::core::ffi::c_double = fmax(1.0f64, (*t).true_air_speed);
    let mut tas: ::core::ffi::c_double = tas0;
    let mut horizontal0: ::core::ffi::c_double = fmax(1.0f64, (*t).horizontal_speed);
    let mut horizontal: ::core::ffi::c_double = horizontal0;
    let mut steps: ::core::ffi::c_int = ceil(seconds / 0.12f64) as ::core::ffi::c_int;
    if steps < 1 as ::core::ffi::c_int {
        steps = 1 as ::core::ffi::c_int;
    }
    if steps > 240 as ::core::ffi::c_int {
        steps = 240 as ::core::ffi::c_int;
    }
    let mut dt: ::core::ffi::c_double = seconds / steps as ::core::ffi::c_double;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < steps {
        let mut bank_error: ::core::ffi::c_double = norm_signed_deg(target_bank - bank);
        let mut desired_bank_rate: ::core::ffi::c_double = clampd(
            bank_error / 1.05f64,
            -roll_rate_limit,
            roll_rate_limit,
        );
        bank_rate
            += clampd(
                desired_bank_rate - bank_rate,
                -roll_accel_limit * dt,
                roll_accel_limit * dt,
            );
        bank = clampd(bank + bank_rate * dt, -70.0f64, 70.0f64);
        let mut speed_scale: ::core::ffi::c_double = clampd(
            tas / tas0 * (tas / tas0),
            0.45f64,
            1.15f64,
        );
        let mut lateral: ::core::ffi::c_double = lift_accel * speed_scale
            * sin(clampd(bank * bank_effectiveness, -89.0f64, 89.0f64) * DEG2RAD);
        let mut kinematic_rate: ::core::ffi::c_double = lateral
            / fmax(horizontal, 20.0f64) * RAD2DEG;
        let mut rate_cap: ::core::ffi::c_double = hac_course_rate_cap_for_speed(tas)
            * 1.60f64 / 1.50f64;
        kinematic_rate = clampd(kinematic_rate, -rate_cap, rate_cap);
        let mut rate_alpha: ::core::ffi::c_double = dt / fmax(0.55f64 + dt, 1e-6f64);
        course_rate += (kinematic_rate - course_rate) * rate_alpha;
        let mut mid_course: ::core::ffi::c_double = course + course_rate * dt * 0.5f64;
        let mut projected_altitude: ::core::ffi::c_double = (*t).mean_altitude
            + (*t).vertical_speed * (i as ::core::ffi::c_double + 0.5f64) * dt;
        let mut surface_horizontal: ::core::ffi::c_double = horizontal * planet_radius
            / fmax(1.0f64, planet_radius + projected_altitude);
        e += surface_horizontal * sin(mid_course * DEG2RAD) * dt;
        n += surface_horizontal * cos(mid_course * DEG2RAD) * dt;
        course = norm_deg(course + course_rate * dt);
        tas = fmax(1.0f64, tas - drag_accel * dt);
        horizontal = fmax(1.0f64, horizontal0 * (tas / tas0));
        i += 1;
    }
    if !out_e.is_null() {
        *out_e = e;
    }
    if !out_n.is_null() {
        *out_n = n;
    }
    if !out_course.is_null() {
        *out_course = norm_deg(course);
    }
}
#[no_mangle]
pub unsafe extern "C" fn hac_fixed_alignment_geometry(
    mut entry_out: *mut HACPoint2,
    mut center_out: *mut HACPoint2,
    mut exit_out: *mut HACPoint2,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut hac_radius: ::core::ffi::c_double,
    mut capture_course: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
) -> bool {
    if entry_out.is_null() || center_out.is_null() || exit_out.is_null()
        || site.is_null() || s.is_null() || !(hac_radius > 0.0f64)
        || fabs(side) != 1.0f64 || capture_course.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut runway: ::core::ffi::c_double = (*site).runway_heading * DEG2RAD;
    let mut capture: ::core::ffi::c_double = capture_course * DEG2RAD;
    let mut vh_e: ::core::ffi::c_double = sin(runway);
    let mut vh_n: ::core::ffi::c_double = cos(runway);
    let mut rh_e: ::core::ffi::c_double = cos(runway);
    let mut rh_n: ::core::ffi::c_double = -sin(runway);
    let mut rf_e: ::core::ffi::c_double = cos(capture);
    let mut rf_n: ::core::ffi::c_double = -sin(capture);
    let mut exit: HACPoint2 = HACPoint2 {
        e: -vh_e * (*s).final_approach_distance,
        n: -vh_n * (*s).final_approach_distance,
    };
    let mut center: HACPoint2 = HACPoint2 {
        e: exit.e + side * hac_radius * rh_e,
        n: exit.n + side * hac_radius * rh_n,
    };
    let mut entry: HACPoint2 = HACPoint2 {
        e: center.e - side * hac_radius * rf_e,
        n: center.n - side * hac_radius * rf_n,
    };
    *entry_out = entry;
    *center_out = center;
    *exit_out = exit;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_runway_line_path_guidance(
    mut t: *const Telemetry,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut vehicle: *const VehicleProfile,
    mut gravity: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
) -> HACGuidance {
    let mut out: HACGuidance = HACGuidance {
        heading: 0.,
        bank: 0.,
        desired_altitude: 0.,
        distance_final: 0.,
        arc_remaining: 0.,
        radial_error: 0.,
        course_error: 0.,
        angle: 0.,
        lateral_acceleration: 0.,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<HACGuidance>() as size_t,
    );
    if t.is_null() || site.is_null() || s.is_null() || vehicle.is_null() {
        return out;
    }
    let mut speed: ::core::ffi::c_double = fmax(1.0f64, guidance_lateral_speed(t));
    let mut station: ::core::ffi::c_double = -(*s).final_approach_distance;
    let mut station_gap: ::core::ffi::c_double = fmax(
        0.0f64,
        station - (*t).runway_along_track,
    );
    let mut bank_limit: ::core::ffi::c_double = terminal_lateral_bank_limit(t, vehicle);
    let mut rate_cap: ::core::ffi::c_double = fmax(0.05f64, hac_course_rate_cap(t))
        * DEG2RAD;
    let mut lateral_limit: ::core::ffi::c_double = fmin(
        speed * rate_cap,
        fmax(0.0f64, gravity) * tan(clampd(bank_limit, 0.0f64, 80.0f64) * DEG2RAD),
    );
    if !(lateral_limit > 0.0f64) || lateral_limit.is_finite() as i32 == 0 {
        out.heading = norm_deg(course);
        out.course_error = 0.0f64;
        out.bank = 0.0f64;
        out.radial_error = (*t).runway_cross_track;
        out.arc_remaining = station_gap;
        out.distance_final = fmax(0.0f64, -(*t).runway_along_track);
        out.desired_altitude = (*site).altitude
            + out.distance_final * tan((*s).taem_glide_slope * DEG2RAD);
        return out;
    }
    let mut look: ::core::ffi::c_double = fmax(speed * 5.5f64, station_gap * 0.45f64);
    let mut intercept: ::core::ffi::c_double = 0.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        intercept = atan2(-(*t).runway_cross_track, fmax(1.0f64, look)) * RAD2DEG;
        out.heading = norm_deg((*site).runway_heading + intercept);
        out.course_error = norm_signed_deg(out.heading - course);
        let mut required: ::core::ffi::c_double = 2.0f64 * speed * speed
            * fabs(sin(out.course_error * DEG2RAD)) / lateral_limit;
        if required.is_finite() as i32 == 0 {
            return HACGuidance {
                heading: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                bank: 0.,
                desired_altitude: 0.,
                distance_final: 0.,
                arc_remaining: 0.,
                radial_error: 0.,
                course_error: 0.,
                angle: 0.,
                lateral_acceleration: 0.,
            };
        }
        look = fmax(look, required);
        i += 1;
    }
    intercept = atan2(-(*t).runway_cross_track, fmax(1.0f64, look)) * RAD2DEG;
    out.heading = norm_deg((*site).runway_heading + intercept);
    out.course_error = norm_signed_deg(out.heading - course);
    let mut lateral: ::core::ffi::c_double = 2.0f64 * speed * speed / fmax(look, 1.0f64)
        * sin(out.course_error * DEG2RAD);
    lateral = clampd(lateral, -lateral_limit, lateral_limit);
    out.lateral_acceleration = lateral;
    out.bank = clampd(
        atan2(lateral, fmax(gravity, 0.01f64)) * RAD2DEG,
        -bank_limit,
        bank_limit,
    );
    out.radial_error = (*t).runway_cross_track;
    out.arc_remaining = station_gap;
    out.distance_final = fmax(0.0f64, -(*t).runway_along_track);
    out.desired_altitude = (*site).altitude
        + out.distance_final * tan((*s).taem_glide_slope * DEG2RAD);
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn reference_trajectory(
    mut out: *mut Trajectory,
    mut site: *const LandingSite,
    mut settings: *const GuidanceSettings,
    mut radius: ::core::ffi::c_double,
    mut hac_side: ::core::ffi::c_double,
) {
    if !out.is_null() {
        trajectory_clear(out);
    }
}
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
