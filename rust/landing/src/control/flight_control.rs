use crate::sys::*;
extern "C" {
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmod(
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
    fn v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vector3;
    fn vadd(a: Vector3, b: Vector3) -> Vector3;
    fn vscale(a: Vector3, s: ::core::ffi::c_double) -> Vector3;
    fn vdot(a: Vector3, b: Vector3) -> ::core::ffi::c_double;
    fn vcross(a: Vector3, b: Vector3) -> Vector3;
    fn vmag(a: Vector3) -> ::core::ffi::c_double;
    fn vnorm(a: Vector3, fallback: Vector3) -> Vector3;
    fn vproject_plane(a: Vector3, normal: Vector3) -> Vector3;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
pub type NavballSpeedMode = ::core::ffi::c_uint;
pub const SPEED_SURFACE: NavballSpeedMode = 2;
pub const SPEED_ORBIT: NavballSpeedMode = 1;
pub const SPEED_UNCHANGED: NavballSpeedMode = 0;
pub type ControlProfile = ::core::ffi::c_uint;
pub const PROFILE_RECOVERY: ControlProfile = 6;
pub const PROFILE_ROLLOUT: ControlProfile = 5;
pub const PROFILE_FLARE: ControlProfile = 4;
pub const PROFILE_APPROACH: ControlProfile = 3;
pub const PROFILE_TAEM: ControlProfile = 2;
pub const PROFILE_ENTRY: ControlProfile = 1;
pub const PROFILE_ORBITAL: ControlProfile = 0;
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
pub struct GuidanceCommand {
    pub target_pitch: ::core::ffi::c_double,
    pub target_heading: ::core::ffi::c_double,
    pub target_roll: ::core::ffi::c_double,
    pub target_throttle: ::core::ffi::c_double,
    pub wheel_steering: ::core::ffi::c_double,
    pub has_target_aoa: bool,
    pub target_aoa: ::core::ffi::c_double,
    pub has_pitch_trim_feedforward: bool,
    pub pitch_trim_feedforward: ::core::ffi::c_double,
    pub speedbrake_fraction: ::core::ffi::c_double,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: bool,
    pub use_inertial_direction: bool,
    pub thrust_alignment: bool,
    pub inertial_direction: Vector3,
    pub autopilot_engaged: bool,
    pub heading_control_enabled: bool,
    pub hac_control_tuning: bool,
    pub terminal_pitch_tuning: bool,
    pub navball_speed_mode: NavballSpeedMode,
    pub control_profile: ControlProfile,
    pub terminal_roll_trim_enabled: bool,
    pub jet_assist_throttle: ::core::ffi::c_double,
}
pub type FlightControlAxis = ::core::ffi::c_uint;
pub const FLIGHT_CONTROL_AXIS_COUNT: FlightControlAxis = 3;
pub const FLIGHT_CONTROL_AXIS_YAW: FlightControlAxis = 2;
pub const FLIGHT_CONTROL_AXIS_ROLL: FlightControlAxis = 1;
pub const FLIGHT_CONTROL_AXIS_PITCH: FlightControlAxis = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlAxisAuthority {
    pub torque_accel: ::core::ffi::c_double,
    pub reported_torque_accel: ::core::ffi::c_double,
    pub non_aero_confidence: ::core::ffi::c_double,
    pub controlled_accel: ::core::ffi::c_double,
    pub controlled_confidence: ::core::ffi::c_double,
    pub live_response_confidence: ::core::ffi::c_double,
    pub aero_per_q: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub drift_accel: ::core::ffi::c_double,
    pub last_rate: ::core::ffi::c_double,
    pub reported_initialized: bool,
    pub has_last_rate: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlBetaModel {
    pub mean_yaw_rate: ::core::ffi::c_double,
    pub mean_roll_rate: ::core::ffi::c_double,
    pub mean_beta_rate: ::core::ffi::c_double,
    pub yy: ::core::ffi::c_double,
    pub rr: ::core::ffi::c_double,
    pub yr: ::core::ffi::c_double,
    pub by: ::core::ffi::c_double,
    pub br: ::core::ffi::c_double,
    pub yaw_gain: ::core::ffi::c_double,
    pub roll_gain: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub initialized: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlAttitudeError {
    pub valid: bool,
    pub use_yaw_error: bool,
    pub pitch_error_deg: ::core::ffi::c_double,
    pub roll_error_deg: ::core::ffi::c_double,
    pub yaw_error_deg: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlDiagnostics {
    pub profile: ControlProfile,
    pub target_pitch_state: ::core::ffi::c_double,
    pub measured_pitch_state: ::core::ffi::c_double,
    pub pitch_error: ::core::ffi::c_double,
    pub effective_pitch_rate: ::core::ffi::c_double,
    pub body_pitch_rate: ::core::ffi::c_double,
    pub body_pitch_rate_available: bool,
    pub roll_error: ::core::ffi::c_double,
    pub effective_roll_rate: ::core::ffi::c_double,
    pub body_roll_rate: ::core::ffi::c_double,
    pub body_roll_rate_available: bool,
    pub target_roll_rate: ::core::ffi::c_double,
    pub commanded_roll_rate: ::core::ffi::c_double,
    pub relative_roll_rate: ::core::ffi::c_double,
    pub recovery_rate_first: bool,
    pub heading_error: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub sideslip_rate: ::core::ffi::c_double,
    pub body_yaw_rate: ::core::ffi::c_double,
    pub body_yaw_rate_available: bool,
    pub flight_path_angle: ::core::ffi::c_double,
    pub pitch_trim: ::core::ffi::c_double,
    pub steady_pitch_trim: ::core::ffi::c_double,
    pub terminal_pitch_integral: ::core::ffi::c_double,
    pub roll_trim: ::core::ffi::c_double,
    pub pitch_authority: ::core::ffi::c_double,
    pub pitch_raw_authority: ::core::ffi::c_double,
    pub pitch_guard_authority: ::core::ffi::c_double,
    pub pitch_aero_fraction: ::core::ffi::c_double,
    pub pitch_fast_command_limit: ::core::ffi::c_double,
    pub pitch_hold_seconds: ::core::ffi::c_double,
    pub pitch_rate_impulse_budget: ::core::ffi::c_double,
    pub roll_authority: ::core::ffi::c_double,
    pub roll_raw_authority: ::core::ffi::c_double,
    pub roll_guard_authority: ::core::ffi::c_double,
    pub roll_aero_fraction: ::core::ffi::c_double,
    pub roll_rate_limit: ::core::ffi::c_double,
    pub roll_command_limit: ::core::ffi::c_double,
    pub roll_hold_seconds: ::core::ffi::c_double,
    pub roll_rate_impulse_budget: ::core::ffi::c_double,
    pub roll_bandwidth_scale: ::core::ffi::c_double,
    pub pitch_bandwidth_scale: ::core::ffi::c_double,
    pub yaw_authority: ::core::ffi::c_double,
    pub yaw_aero_fraction: ::core::ffi::c_double,
    pub beta_yaw_gain: ::core::ffi::c_double,
    pub beta_roll_coupling: ::core::ffi::c_double,
    pub beta_confidence: ::core::ffi::c_double,
    pub sample_dt: ::core::ffi::c_double,
    pub attitude_error: ::core::ffi::c_double,
    pub quaternion_attitude_control: bool,
    pub rcs_transonic_cutoff: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlOutput {
    pub valid: bool,
    pub pitch: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub yaw: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub wheel_steering: ::core::ffi::c_double,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: bool,
    pub rcs_assist: ::core::ffi::c_double,
    pub rcs_requested: bool,
    pub diagnostics: FlightControlDiagnostics,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlState {
    pub nominal_dt: ::core::ffi::c_double,
    pub has_sample: bool,
    pub last_ut: ::core::ffi::c_double,
    pub last_pitch: ::core::ffi::c_double,
    pub last_aoa: ::core::ffi::c_double,
    pub last_roll: ::core::ffi::c_double,
    pub last_heading: ::core::ffi::c_double,
    pub last_sideslip: ::core::ffi::c_double,
    pub pitch_rate: ::core::ffi::c_double,
    pub aoa_rate: ::core::ffi::c_double,
    pub roll_rate: ::core::ffi::c_double,
    pub heading_rate: ::core::ffi::c_double,
    pub sideslip_rate: ::core::ffi::c_double,
    pub last_control: [::core::ffi::c_double; 3],
    pub authority: [FlightControlAxisAuthority; 3],
    pub has_yaw_observation_frame: bool,
    pub yaw_observation_frame: ::core::ffi::c_int,
    pub beta: FlightControlBetaModel,
    pub pitch_trim: ::core::ffi::c_double,
    pub applied_pitch_trim_feedforward: ::core::ffi::c_double,
    pub terminal_pitch_integral: ::core::ffi::c_double,
    pub roll_trim: ::core::ffi::c_double,
    pub terminal_pitch_authority: ::core::ffi::c_double,
    pub terminal_roll_authority: ::core::ffi::c_double,
    pub has_terminal_pitch_authority: bool,
    pub has_terminal_roll_authority: bool,
    pub last_target_pitch_state: ::core::ffi::c_double,
    pub target_pitch_rate: ::core::ffi::c_double,
    pub has_last_target_pitch: bool,
    pub last_target_roll: ::core::ffi::c_double,
    pub target_roll_rate: ::core::ffi::c_double,
    pub has_last_target_roll: bool,
    pub has_quaternion_error_sample: bool,
    pub last_quaternion_pitch_error: ::core::ffi::c_double,
    pub last_quaternion_roll_error: ::core::ffi::c_double,
    pub last_quaternion_yaw_error: ::core::ffi::c_double,
    pub quaternion_pitch_error_rate: ::core::ffi::c_double,
    pub quaternion_roll_error_rate: ::core::ffi::c_double,
    pub quaternion_yaw_error_rate: ::core::ffi::c_double,
    pub pitch_response_scale: ::core::ffi::c_double,
    pub rcs_transonic_cutoff: bool,
    pub rcs_latched: bool,
    pub rcs_q_latched: bool,
    pub rcs_saturation_latched: bool,
    pub last_profile: ControlProfile,
    pub has_last_profile: bool,
    pub main_gear_contact_latched: bool,
    pub nose_gear_contact_latched: bool,
    pub main_gear_contact_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlTuning {
    pub roll_accel_max: ::core::ffi::c_double,
    pub pitch_accel_max: ::core::ffi::c_double,
    pub pitch_lag_s: ::core::ffi::c_double,
    pub pitch_wn: ::core::ffi::c_double,
    pub pitch_zeta: ::core::ffi::c_double,
    pub pitch_trim_ki: ::core::ffi::c_double,
    pub roll_lag_s: ::core::ffi::c_double,
    pub roll_wn: ::core::ffi::c_double,
    pub roll_zeta: ::core::ffi::c_double,
    pub yaw_accel_max: ::core::ffi::c_double,
    pub yaw_wn: ::core::ffi::c_double,
    pub yaw_lag_s: ::core::ffi::c_double,
    pub rcs_release_beta_deg: ::core::ffi::c_double,
    pub rcs_q_on_pa: ::core::ffi::c_double,
    pub rcs_q_off_pa: ::core::ffi::c_double,
}
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const FLIGHT_CONTROL_REVISION: [::core::ffi::c_char; 25] = unsafe {
    ::core::mem::transmute::<
        [u8; 25],
        [::core::ffi::c_char; 25],
    >(*b"native-c-fcs-20260927-r3\0")
};
static mut fc_tuning_values: FlightControlTuning = FlightControlTuning {
    roll_accel_max: 0.,
    pitch_accel_max: 0.,
    pitch_lag_s: 0.,
    pitch_wn: 0.,
    pitch_zeta: 0.,
    pitch_trim_ki: 0.,
    roll_lag_s: 0.,
    roll_wn: 0.,
    roll_zeta: 0.,
    yaw_accel_max: 0.,
    yaw_wn: 0.,
    yaw_lag_s: 0.,
    rcs_release_beta_deg: 0.,
    rcs_q_on_pa: 0.,
    rcs_q_off_pa: 0.,
};
static mut fc_tuning_once: pthread_once_t = PTHREAD_ONCE_INIT;
unsafe extern "C" fn fc_tuning_override(
    mut enabled: bool,
    mut name: *const ::core::ffi::c_char,
    mut value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !enabled {
        return value;
    }
    let mut text: *const ::core::ffi::c_char = getenv(name);
    if text.is_null() {
        return value;
    }
    let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    let mut parsed: ::core::ffi::c_double = strtod(text, &raw mut end);
    return if end != text as *mut ::core::ffi::c_char && parsed.is_finite() as i32 != 0
        && parsed > 0.0f64
    {
        parsed
    } else {
        value
    };
}
unsafe extern "C" fn fc_tuning_init() {
    let mut flag: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_FCS_TUNING_OVERRIDES\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut on: bool = !flag.is_null()
        && strcmp(flag, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int;
    let mut t: FlightControlTuning = FlightControlTuning {
        roll_accel_max: fc_tuning_override(
            on,
            b"KSP_LANDER_ROLL_ACCEL\0" as *const u8 as *const ::core::ffi::c_char,
            100.0f64,
        ),
        pitch_accel_max: fc_tuning_override(
            on,
            b"KSP_LANDER_PITCH_ACCEL\0" as *const u8 as *const ::core::ffi::c_char,
            40.0f64,
        ),
        pitch_lag_s: fc_tuning_override(
            on,
            b"KSP_LANDER_PITCH_LAG\0" as *const u8 as *const ::core::ffi::c_char,
            0.12f64,
        ),
        pitch_wn: fc_tuning_override(
            on,
            b"KSP_LANDER_PITCH_WN\0" as *const u8 as *const ::core::ffi::c_char,
            1.05f64,
        ),
        pitch_zeta: fc_tuning_override(
            on,
            b"KSP_LANDER_PITCH_ZETA\0" as *const u8 as *const ::core::ffi::c_char,
            1.15f64,
        ),
        pitch_trim_ki: fc_tuning_override(
            on,
            b"KSP_LANDER_PITCH_TRIM_KI\0" as *const u8 as *const ::core::ffi::c_char,
            0.025f64,
        ),
        roll_lag_s: fc_tuning_override(
            on,
            b"KSP_LANDER_ROLL_LAG\0" as *const u8 as *const ::core::ffi::c_char,
            0.12f64,
        ),
        roll_wn: fc_tuning_override(
            on,
            b"KSP_LANDER_ROLL_WN\0" as *const u8 as *const ::core::ffi::c_char,
            0.65f64,
        ),
        roll_zeta: fc_tuning_override(
            on,
            b"KSP_LANDER_ROLL_ZETA\0" as *const u8 as *const ::core::ffi::c_char,
            1.15f64,
        ),
        yaw_accel_max: fc_tuning_override(
            on,
            b"KSP_LANDER_YAW_ACCEL\0" as *const u8 as *const ::core::ffi::c_char,
            30.0f64,
        ),
        yaw_wn: fc_tuning_override(
            on,
            b"KSP_LANDER_YAW_WN\0" as *const u8 as *const ::core::ffi::c_char,
            1.0f64,
        ),
        yaw_lag_s: fc_tuning_override(
            on,
            b"KSP_LANDER_YAW_LAG\0" as *const u8 as *const ::core::ffi::c_char,
            0.20f64,
        ),
        rcs_release_beta_deg: fc_tuning_override(
            on,
            b"KSP_LANDER_RCS_RELEASE_BETA\0" as *const u8 as *const ::core::ffi::c_char,
            5.0f64,
        ),
        rcs_q_on_pa: fc_tuning_override(
            on,
            b"KSP_LANDER_RCS_Q_ON_PA\0" as *const u8 as *const ::core::ffi::c_char,
            700.0f64,
        ),
        rcs_q_off_pa: fc_tuning_override(
            on,
            b"KSP_LANDER_RCS_Q_OFF_PA\0" as *const u8 as *const ::core::ffi::c_char,
            500.0f64,
        ),
    };
    if t.rcs_q_off_pa >= t.rcs_q_on_pa {
        t.rcs_q_off_pa = 0.75f64 * t.rcs_q_on_pa;
    }
    fc_tuning_values = t;
}
unsafe extern "C" fn fc_tuning() -> *const FlightControlTuning {
    pthread_once(
        &raw mut fc_tuning_once,
        Some(fc_tuning_init as unsafe extern "C" fn() -> ()),
    );
    return &raw mut fc_tuning_values;
}
unsafe extern "C" fn fc_finite(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 { value } else { fallback };
}
unsafe extern "C" fn fc_clamp(
    mut value: ::core::ffi::c_double,
    mut lower: ::core::ffi::c_double,
    mut upper: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if value < lower {
        return lower;
    }
    if value > upper {
        return upper;
    }
    return value;
}
unsafe extern "C" fn fc_signed_angle(
    mut value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut wrapped: ::core::ffi::c_double = fmod(value, 360.0f64);
    if wrapped < 0.0f64 {
        wrapped += 360.0f64;
    }
    return if wrapped > 180.0f64 { wrapped - 360.0f64 } else { wrapped };
}
unsafe extern "C" fn fc_atmospheric_profile(mut profile: ControlProfile) -> bool {
    return profile as ::core::ffi::c_uint
        == PROFILE_ENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_APPROACH as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint;
}
unsafe extern "C" fn fc_axis_set_reported(
    mut axis: *mut FlightControlAxisAuthority,
    mut value: ::core::ffi::c_double,
) {
    let mut reported: ::core::ffi::c_double = fmax(0.0f64, fc_finite(value, 0.0f64));
    if reported <= 0.0f64 {
        return;
    }
    (*axis).reported_torque_accel = reported;
    if !(*axis).reported_initialized || (*axis).torque_accel <= 0.0f64 {
        (*axis).torque_accel = reported;
        (*axis).reported_initialized = true_0 != 0;
    }
}
unsafe extern "C" fn fc_axis_available(
    mut axis: *const FlightControlAxisAuthority,
    mut q: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if (*axis).controlled_confidence > 0.0f64
        && (*axis).controlled_accel.is_finite() as i32 != 0
        && (*axis).controlled_accel > 0.0f64
    {
        return (*axis).controlled_accel;
    }
    let mut baseline: ::core::ffi::c_double = fmax(
        0.0f64,
        fc_finite((*axis).torque_accel, 0.0f64),
    );
    let mut aerodynamic: ::core::ffi::c_double = fmax(
        0.0f64,
        fc_finite((*axis).aero_per_q, 0.0f64),
    ) * fmax(0.0f64, q);
    return fmax(DBL_EPSILON, baseline + aerodynamic);
}
unsafe extern "C" fn fc_axis_control_authority_known(
    mut axis: *const FlightControlAxisAuthority,
) -> bool {
    return !axis.is_null() && (*axis).controlled_confidence > 0.0f64
        && (*axis).controlled_accel.is_finite() as i32 != 0
        && (*axis).controlled_accel > 0.0f64;
}
unsafe extern "C" fn fc_axis_observe(
    mut axis: *mut FlightControlAxisAuthority,
    mut rate: ::core::ffi::c_double,
    mut previous_input: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut cross_axis_input: ::core::ffi::c_double,
    mut response_time: ::core::ffi::c_double,
) {
    if axis.is_null() || rate.is_finite() as i32 == 0 || dt.is_finite() as i32 == 0
        || !(dt > 0.0f64)
    {
        return;
    }
    if !(*axis).has_last_rate {
        (*axis).last_rate = rate;
        (*axis).has_last_rate = true_0 != 0;
        return;
    }
    let mut acceleration: ::core::ffi::c_double = (rate - (*axis).last_rate) / dt;
    (*axis).last_rate = rate;
    if acceleration.is_finite() as i32 == 0 {
        return;
    }
    let mut input: ::core::ffi::c_double = fc_finite(previous_input, 0.0f64);
    let mut magnitude: ::core::ffi::c_double = fabs(input);
    let mut interference: ::core::ffi::c_double = fabs(
        fc_finite(cross_axis_input, 0.0f64),
    );
    if !(magnitude > DBL_EPSILON) {
        (*axis).drift_accel = acceleration;
        return;
    }
    let mut excitation_weight: ::core::ffi::c_double = magnitude
        / (magnitude + interference + DBL_EPSILON);
    if response_time > DBL_EPSILON {
        excitation_weight
            *= fmin(1.0f64, magnitude) * (1.0f64 - exp(-dt / response_time));
    }
    let mut controlled: ::core::ffi::c_double = (acceleration - (*axis).drift_accel)
        / input;
    if controlled.is_finite() as i32 == 0 || !(controlled > 0.0f64) {
        return;
    }
    if !((*axis).controlled_accel > 0.0f64) || (*axis).controlled_confidence <= 0.0f64 {
        (*axis).controlled_accel = controlled;
    } else {
        (*axis).controlled_accel
            += excitation_weight * (controlled - (*axis).controlled_accel);
    }
    (*axis).live_response_confidence = 1.0f64
        - (1.0f64 - (*axis).live_response_confidence) * (1.0f64 - excitation_weight);
    (*axis).controlled_confidence = 1.0f64
        - (1.0f64 - (*axis).controlled_confidence) * (1.0f64 - excitation_weight);
}
unsafe extern "C" fn fc_reported_axis_accel(
    mut t: *const Telemetry,
    mut axis: FlightControlAxis,
) -> ::core::ffi::c_double {
    if !(*t).has_torque || !(*t).has_inertia {
        return 0.0f64;
    }
    let mut torque: ::core::ffi::c_double = 0.0f64;
    let mut inertia: ::core::ffi::c_double = 0.0f64;
    match axis as ::core::ffi::c_uint {
        0 => {
            torque = (*t).available_pitch_torque;
            inertia = (*t).pitch_moment_of_inertia;
        }
        1 => {
            torque = (*t).available_roll_torque;
            inertia = (*t).roll_moment_of_inertia;
        }
        2 => {
            torque = (*t).available_yaw_torque;
            inertia = (*t).yaw_moment_of_inertia;
        }
        _ => return 0.0f64,
    }
    if torque.is_finite() as i32 == 0 || inertia.is_finite() as i32 == 0
        || inertia <= 0.0f64
    {
        return 0.0f64;
    }
    return fabs(torque) / inertia * RAD2DEG;
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_init(
    mut state: *mut FlightControlState,
    mut nominal_dt: ::core::ffi::c_double,
) {
    if state.is_null() {
        return;
    }
    memset(
        state as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<FlightControlState>() as size_t,
    );
    (*state).nominal_dt = if nominal_dt.is_finite() as i32 != 0 && nominal_dt > 0.0f64 {
        nominal_dt
    } else {
        0.1f64
    };
    (*state).pitch_response_scale = 0.70f64;
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_reset_transients(
    mut state: *mut FlightControlState,
) {
    if state.is_null() {
        return;
    }
    (*state).has_sample = false_0 != 0;
    (*state).pitch_rate = 0.0f64;
    (*state).aoa_rate = 0.0f64;
    (*state).roll_rate = 0.0f64;
    (*state).heading_rate = 0.0f64;
    (*state).sideslip_rate = 0.0f64;
    (*state).pitch_trim = 0.0f64;
    (*state).applied_pitch_trim_feedforward = 0.0f64;
    (*state).terminal_pitch_integral = 0.0f64;
    (*state).roll_trim = 0.0f64;
    (*state).has_terminal_pitch_authority = false_0 != 0;
    (*state).has_terminal_roll_authority = false_0 != 0;
    (*state).has_last_target_pitch = false_0 != 0;
    (*state).target_pitch_rate = 0.0f64;
    (*state).has_last_target_roll = false_0 != 0;
    (*state).target_roll_rate = 0.0f64;
    (*state).has_quaternion_error_sample = false_0 != 0;
    (*state).last_quaternion_pitch_error = 0.0f64;
    (*state).last_quaternion_roll_error = 0.0f64;
    (*state).last_quaternion_yaw_error = 0.0f64;
    (*state).quaternion_pitch_error_rate = 0.0f64;
    (*state).quaternion_roll_error_rate = 0.0f64;
    (*state).quaternion_yaw_error_rate = 0.0f64;
    (*state).pitch_response_scale = 0.70f64;
    (*state).rcs_transonic_cutoff = false_0 != 0;
    (*state).rcs_latched = false_0 != 0;
    (*state).rcs_q_latched = false_0 != 0;
    (*state).rcs_saturation_latched = false_0 != 0;
    (*state).has_last_profile = false_0 != 0;
    (*state).main_gear_contact_latched = false_0 != 0;
    memset(
        &raw mut (*state).last_control as *mut ::core::ffi::c_double
            as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<[::core::ffi::c_double; 3]>() as size_t,
    );
    memset(
        &raw mut (*state).beta as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<FlightControlBetaModel>() as size_t,
    );
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < FLIGHT_CONTROL_AXIS_COUNT as ::core::ffi::c_int {
        (*state).authority[i as usize].has_last_rate = false_0 != 0;
        i += 1;
    }
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_seed_axis_authority(
    mut state: *mut FlightControlState,
    mut axis: FlightControlAxis,
    mut non_aero_accel: ::core::ffi::c_double,
    mut aero_accel_per_pascal: ::core::ffi::c_double,
    mut confidence: ::core::ffi::c_double,
) {
    if state.is_null() || (axis as ::core::ffi::c_uint) < 0 as ::core::ffi::c_uint
        || axis as ::core::ffi::c_uint
            >= FLIGHT_CONTROL_AXIS_COUNT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return;
    }
    let mut estimate: *mut FlightControlAxisAuthority = (&raw mut (*state).authority
        as *mut FlightControlAxisAuthority)
        .offset(axis as isize) as *mut FlightControlAxisAuthority;
    (*estimate).torque_accel = fmax(0.0f64, fc_finite(non_aero_accel, 0.0f64));
    (*estimate).aero_per_q = fmax(0.0f64, fc_finite(aero_accel_per_pascal, 0.0f64));
    (*estimate).confidence = fc_clamp(fc_finite(confidence, 0.0f64), 0.0f64, 1.0f64);
    (*estimate).non_aero_confidence = (*estimate).confidence;
    if (*estimate).torque_accel > 0.0f64 {
        (*estimate).reported_initialized = true_0 != 0;
    }
    if (*estimate).confidence > 0.0f64 && (*estimate).torque_accel > 0.0f64 {
        (*estimate).controlled_accel = (*estimate).torque_accel;
        (*estimate).controlled_confidence = (*estimate).confidence;
    }
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_revision() -> *const ::core::ffi::c_char {
    return FLIGHT_CONTROL_REVISION.as_ptr();
}
unsafe extern "C" fn fc_capped_pd_command(
    mut angle_error: ::core::ffi::c_double,
    mut measured_rate: ::core::ffi::c_double,
    mut target_rate: ::core::ffi::c_double,
    mut authority: ::core::ffi::c_double,
    mut natural_frequency: ::core::ffi::c_double,
    mut damping: ::core::ffi::c_double,
    mut lag_seconds: ::core::ffi::c_double,
    mut previous_input: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut available: ::core::ffi::c_double = fabs(fc_finite(authority, 0.0f64));
    if !(available > DBL_EPSILON) {
        return 0.0f64;
    }
    let mut lag: ::core::ffi::c_double = fmax(0.0f64, fc_finite(lag_seconds, 0.0f64));
    let mut applied: ::core::ffi::c_double = fc_clamp(
        fc_finite(previous_input, 0.0f64),
        -1.0f64,
        1.0f64,
    ) * available;
    let mut relative_rate: ::core::ffi::c_double = measured_rate - target_rate;
    let mut rate_ahead: ::core::ffi::c_double = measured_rate + applied * lag;
    let mut error_ahead: ::core::ffi::c_double = angle_error - relative_rate * lag
        - 0.5f64 * applied * lag * lag;
    let mut relative_rate_ahead: ::core::ffi::c_double = rate_ahead - target_rate;
    let mut wn: ::core::ffi::c_double = fmax(
        0.0f64,
        fc_finite(natural_frequency, 0.0f64),
    );
    let mut zeta: ::core::ffi::c_double = fmax(0.0f64, fc_finite(damping, 0.0f64));
    let mut accel: ::core::ffi::c_double = wn * wn * error_ahead
        - 2.0f64 * zeta * wn * relative_rate_ahead;
    return fc_clamp(accel / available, -1.0f64, 1.0f64);
}
unsafe extern "C" fn fc_smooth_actuator_command(
    mut target: ::core::ffi::c_double,
    mut previous: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut time_constant: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    target = fc_clamp(fc_finite(target, 0.0f64), -1.0f64, 1.0f64);
    previous = fc_clamp(fc_finite(previous, 0.0f64), -1.0f64, 1.0f64);
    let mut tau: ::core::ffi::c_double = fmax(0.0f64, fc_finite(time_constant, 0.0f64));
    if !(tau > DBL_EPSILON) || !(dt > 0.0f64) {
        return target;
    }
    let mut alpha: ::core::ffi::c_double = 1.0f64 - exp(-dt / tau);
    return fc_clamp(previous + alpha * (target - previous), -1.0f64, 1.0f64);
}
unsafe extern "C" fn fc_lowpass_rate(
    mut previous: ::core::ffi::c_double,
    mut raw: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut tau: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if raw.is_finite() as i32 == 0 {
        return fc_finite(previous, 0.0f64);
    }
    if !(dt > 0.0f64) || dt.is_finite() as i32 == 0 {
        return raw;
    }
    tau = fmax(0.0f64, fc_finite(tau, 0.0f64));
    if !(tau > DBL_EPSILON) {
        return raw;
    }
    let mut alpha: ::core::ffi::c_double = 1.0f64 - exp(-dt / tau);
    return fc_finite(previous, 0.0f64) + alpha * (raw - fc_finite(previous, 0.0f64));
}
unsafe extern "C" fn fc_adapt_response_scale(
    mut current: ::core::ffi::c_double,
    mut angle_error: ::core::ffi::c_double,
    mut relative_rate: ::core::ffi::c_double,
    mut nominal_wn: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(current > 0.0f64 && current <= 1.0f64) {
        current = 0.70f64;
    }
    let mut wn: ::core::ffi::c_double = fmax(0.10f64, fc_finite(nominal_wn, 0.10f64));
    let mut error: ::core::ffi::c_double = fabs(fc_finite(angle_error, 0.0f64));
    let mut rate: ::core::ffi::c_double = fabs(fc_finite(relative_rate, 0.0f64));
    let mut reference_rate: ::core::ffi::c_double = wn * fmax(error, 0.35f64);
    let mut response_ratio: ::core::ffi::c_double = rate / fmax(reference_rate, 0.05f64);
    let mut desired: ::core::ffi::c_double = if response_ratio > 1.0f64 {
        1.0f64 / sqrt(response_ratio)
    } else {
        1.0f64
    };
    desired = fc_clamp(desired, 0.40f64, 1.0f64);
    let mut tau: ::core::ffi::c_double = if desired < current {
        0.40f64
    } else {
        3.0f64
    };
    let mut alpha: ::core::ffi::c_double = 1.0f64 - exp(-fmax(0.0f64, dt) / tau);
    return fc_clamp(current + alpha * (desired - current), 0.40f64, 1.0f64);
}
unsafe extern "C" fn fc_pitch_accel_for_profile(
    mut reported: ::core::ffi::c_double,
    mut cap: ::core::ffi::c_double,
    mut profile: ControlProfile,
    mut radar_altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut used: ::core::ffi::c_double = if reported > DBL_EPSILON && reported < cap {
        reported
    } else {
        cap
    };
    if profile as ::core::ffi::c_uint
        != PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return used;
    }
    let mut height: ::core::ffi::c_double = fmax(
        0.0f64,
        fc_finite(radar_altitude, 120.0f64),
    );
    let mut blend: ::core::ffi::c_double = fc_clamp(
        (120.0f64 - height) / 80.0f64,
        0.0f64,
        1.0f64,
    );
    let mut flare_cap: ::core::ffi::c_double = cap + (16.0f64 - cap) * blend;
    return fmin(used, flare_cap);
}
unsafe extern "C" fn fc_schedule_bandwidth(
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut profile: ControlProfile,
    mut base_pitch_wn: ::core::ffi::c_double,
    mut base_roll_wn: ::core::ffi::c_double,
    mut pitch_wn_out: *mut ::core::ffi::c_double,
    mut roll_wn_out: *mut ::core::ffi::c_double,
    mut roll_scale_out: *mut ::core::ffi::c_double,
) {
    let mut f_q: ::core::ffi::c_double = 0.70f64
        + 0.30f64 * fc_clamp(q / 8000.0f64, 0.0f64, 1.0f64);
    if q > 35000.0f64 {
        f_q *= fmax(0.85f64, 1.0f64 - 0.15f64 * ((q - 35000.0f64) / 15000.0f64));
    }
    let mut pitch_mach: ::core::ffi::c_double = 1.0f64;
    if mach > 1.2f64 && mach < 5.0f64 {
        pitch_mach = 1.20f64;
    } else if mach >= 5.0f64 {
        pitch_mach = 1.05f64;
    }
    let mut roll_mach: ::core::ffi::c_double = 1.0f64;
    if mach > 2.5f64 {
        roll_mach = 0.85f64;
    }
    let mut pitch_profile: ::core::ffi::c_double = 1.0f64;
    if profile as ::core::ffi::c_uint
        == PROFILE_APPROACH as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        pitch_profile = 1.10f64;
    }
    let mut p_wn: ::core::ffi::c_double = base_pitch_wn * f_q * pitch_mach
        * pitch_profile;
    let mut r_wn: ::core::ffi::c_double = base_roll_wn * f_q * roll_mach;
    let mut r_scale: ::core::ffi::c_double = f_q * roll_mach;
    if !pitch_wn_out.is_null() {
        *pitch_wn_out = p_wn;
    }
    if !roll_wn_out.is_null() {
        *roll_wn_out = r_wn;
    }
    if !roll_scale_out.is_null() {
        *roll_scale_out = r_scale;
    }
}
unsafe extern "C" fn fc_quat_rotation_matrix(
    mut qv: *const ::core::ffi::c_double,
    mut r: *mut [::core::ffi::c_double; 3],
) {
    let mut x: ::core::ffi::c_double = *qv.offset(0 as ::core::ffi::c_int as isize);
    let mut y: ::core::ffi::c_double = *qv.offset(1 as ::core::ffi::c_int as isize);
    let mut z: ::core::ffi::c_double = *qv.offset(2 as ::core::ffi::c_int as isize);
    let mut w: ::core::ffi::c_double = *qv.offset(3 as ::core::ffi::c_int as isize);
    let mut n: ::core::ffi::c_double = sqrt(x * x + y * y + z * z + w * w);
    if !(n > 1e-12f64) {
        z = 0.0f64;
        y = z;
        x = y;
        w = 1.0f64;
    } else {
        x /= n;
        y /= n;
        z /= n;
        w /= n;
    }
    (*r.offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize] = 1
        as ::core::ffi::c_int as ::core::ffi::c_double
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * (y * y + z * z);
    (*r.offset(0 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * y - z * w);
    (*r.offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * z + y * w);
    (*r.offset(1 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * y + z * w);
    (*r.offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize] = 1
        as ::core::ffi::c_int as ::core::ffi::c_double
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * (x * x + z * z);
    (*r.offset(1 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (y * z - x * w);
    (*r.offset(2 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * z - y * w);
    (*r.offset(2 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (y * z + x * w);
    (*r.offset(2 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize] = 1
        as ::core::ffi::c_int as ::core::ffi::c_double
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * (x * x + y * y);
}
unsafe extern "C" fn fc_matrix_quaternion(
    mut m: *const [::core::ffi::c_double; 3],
    mut qx: *mut ::core::ffi::c_double,
    mut qy: *mut ::core::ffi::c_double,
    mut qz: *mut ::core::ffi::c_double,
    mut qw: *mut ::core::ffi::c_double,
) {
    let mut x: ::core::ffi::c_double = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut y: ::core::ffi::c_double = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut z: ::core::ffi::c_double = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut w: ::core::ffi::c_double = 1 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut tr: ::core::ffi::c_double = (*m
        .offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
        + (*m.offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
        + (*m
            .offset(2 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize];
    let mut scale: ::core::ffi::c_double = 0.;
    if tr > 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        scale = sqrt(
            fmax(1e-12f64, tr + 1 as ::core::ffi::c_int as ::core::ffi::c_double),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = 0.25f64 * scale;
        x = ((*m
            .offset(2 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[2 as ::core::ffi::c_int as usize]) / scale;
        y = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        z = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    0 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
    } else if (*m
        .offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
        > (*m.offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
        && (*m
            .offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
            > (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[2 as ::core::ffi::c_int as usize]
    {
        scale = sqrt(
            fmax(
                1e-12f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + (*m
                        .offset(
                            0 as ::core::ffi::c_int as isize,
                        ))[0 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            1 as ::core::ffi::c_int as isize,
                        ))[1 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            2 as ::core::ffi::c_int as isize,
                        ))[2 as ::core::ffi::c_int as usize],
            ),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = ((*m
            .offset(2 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[2 as ::core::ffi::c_int as usize]) / scale;
        x = 0.25f64 * scale;
        y = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        z = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
    } else if (*m
        .offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
        > (*m.offset(2 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
    {
        scale = sqrt(
            fmax(
                1e-12f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + (*m
                        .offset(
                            1 as ::core::ffi::c_int as isize,
                        ))[1 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            0 as ::core::ffi::c_int as isize,
                        ))[0 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            2 as ::core::ffi::c_int as isize,
                        ))[2 as ::core::ffi::c_int as usize],
            ),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        x = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        y = 0.25f64 * scale;
        z = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
    } else {
        scale = sqrt(
            fmax(
                1e-12f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + (*m
                        .offset(
                            2 as ::core::ffi::c_int as isize,
                        ))[2 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            0 as ::core::ffi::c_int as isize,
                        ))[0 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            1 as ::core::ffi::c_int as isize,
                        ))[1 as ::core::ffi::c_int as usize],
            ),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    0 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
        x = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        y = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
        z = 0.25f64 * scale;
    }
    let mut n: ::core::ffi::c_double = sqrt(x * x + y * y + z * z + w * w);
    if n > 1e-12f64 {
        x /= n;
        y /= n;
        z /= n;
        w /= n;
    }
    if w < 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
    *qx = x;
    *qy = y;
    *qz = z;
    *qw = w;
}
unsafe extern "C" fn fc_rotate_axis(
    mut value: Vector3,
    mut axis: Vector3,
    mut angle: ::core::ffi::c_double,
) -> Vector3 {
    axis = vnorm(
        axis,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut c: ::core::ffi::c_double = cos(angle);
    let mut s: ::core::ffi::c_double = sin(angle);
    return vadd(
        vadd(vscale(value, c), vscale(vcross(axis, value), s)),
        vscale(axis, vdot(axis, value) * (1.0f64 - c)),
    );
}
unsafe extern "C" fn fc_plane_up(
    mut preferred: Vector3,
    mut forward: Vector3,
) -> Vector3 {
    let mut projected: Vector3 = vproject_plane(preferred, forward);
    if vmag(projected) > 1e-8f64 {
        return vnorm(
            projected,
            v3(
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
    }
    let mut seed: Vector3 = v3(
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut best: ::core::ffi::c_double = fabs(vdot(seed, forward));
    let mut y: Vector3 = v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut z: Vector3 = v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut dy: ::core::ffi::c_double = fabs(vdot(y, forward));
    let mut dz: ::core::ffi::c_double = fabs(vdot(z, forward));
    if dy < best {
        seed = y;
        best = dy;
    }
    if dz < best {
        seed = z;
    }
    return vnorm(
        vproject_plane(seed, forward),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_airframe_error(
    mut attitude_quaternion: *const ::core::ffi::c_double,
    mut air_direction: Vector3,
    mut radial_up: Vector3,
    mut target_aoa_deg: ::core::ffi::c_double,
    mut target_bank_deg: ::core::ffi::c_double,
    mut use_yaw_error: bool,
    mut error: *mut FlightControlAttitudeError,
) -> bool {
    if error.is_null() {
        return false_0 != 0;
    }
    memset(
        error as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<FlightControlAttitudeError>() as size_t,
    );
    if attitude_quaternion.is_null() || target_aoa_deg.is_finite() as i32 == 0
        || target_bank_deg.is_finite() as i32 == 0 || vmag(air_direction) <= 1e-9f64
        || vmag(radial_up) <= 1e-9f64
    {
        return false_0 != 0;
    }
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 4 as ::core::ffi::c_int {
        if (*attitude_quaternion.offset(i as isize)).is_finite() as i32 == 0 {
            return false_0 != 0;
        }
        i += 1;
    }
    let mut air: Vector3 = vnorm(
        air_direction,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut radial: Vector3 = vnorm(
        radial_up,
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut flight_up: Vector3 = fc_plane_up(radial, air);
    let mut bank: ::core::ffi::c_double = target_bank_deg * DEG2RAD;
    let mut banked_up: Vector3 = vnorm(fc_rotate_axis(flight_up, air, -bank), flight_up);
    let mut alpha: ::core::ffi::c_double = target_aoa_deg * DEG2RAD;
    let mut forward: Vector3 = vnorm(
        vadd(vscale(air, cos(alpha)), vscale(banked_up, sin(alpha))),
        air,
    );
    let mut right: Vector3 = vnorm(
        vcross(banked_up, air),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut down: Vector3 = vnorm(
        vcross(right, forward),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut current: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    let mut desired: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    let mut relative: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    fc_quat_rotation_matrix(
        attitude_quaternion,
        &raw mut current as *mut [::core::ffi::c_double; 3],
    );
    desired[0 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize] = right
        .x;
    desired[1 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize] = right
        .y;
    desired[2 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize] = right
        .z;
    desired[0 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize] = forward
        .x;
    desired[1 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize] = forward
        .y;
    desired[2 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize] = forward
        .z;
    desired[0 as ::core::ffi::c_int as usize][2 as ::core::ffi::c_int as usize] = down.x;
    desired[1 as ::core::ffi::c_int as usize][2 as ::core::ffi::c_int as usize] = down.y;
    desired[2 as ::core::ffi::c_int as usize][2 as ::core::ffi::c_int as usize] = down.z;
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 3 as ::core::ffi::c_int {
        let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while j < 3 as ::core::ffi::c_int {
            relative[i_0 as usize][j as usize] = 0.0f64;
            let mut k: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while k < 3 as ::core::ffi::c_int {
                relative[i_0 as usize][j as usize]
                    += current[k as usize][i_0 as usize]
                        * desired[k as usize][j as usize];
                k += 1;
            }
            j += 1;
        }
        i_0 += 1;
    }
    let mut x: ::core::ffi::c_double = 0.;
    let mut y: ::core::ffi::c_double = 0.;
    let mut z: ::core::ffi::c_double = 0.;
    let mut w: ::core::ffi::c_double = 0.;
    fc_matrix_quaternion(
        &raw mut relative as *mut [::core::ffi::c_double; 3]
            as *const [::core::ffi::c_double; 3],
        &raw mut x,
        &raw mut y,
        &raw mut z,
        &raw mut w,
    );
    let mut vn: ::core::ffi::c_double = sqrt(x * x + y * y + z * z);
    if vn > 1e-12f64 {
        let mut angle: ::core::ffi::c_double = 2.0f64 * atan2(vn, fmax(0.0f64, w))
            * RAD2DEG / vn;
        (*error).pitch_error_deg = -x * angle;
        (*error).roll_error_deg = -y * angle;
        (*error).yaw_error_deg = -z * angle;
    }
    (*error).use_yaw_error = use_yaw_error;
    (*error).valid = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_step_attitude(
    mut state: *mut FlightControlState,
    mut telemetry: *const Telemetry,
    mut command: *const GuidanceCommand,
    mut attitude_error: *const FlightControlAttitudeError,
    mut sample_dt: ::core::ffi::c_double,
    mut output: *mut FlightControlOutput,
) -> bool {
    if state.is_null() || telemetry.is_null() || command.is_null() || output.is_null() {
        return false_0 != 0;
    }
    memset(
        output as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<FlightControlOutput>() as size_t,
    );
    let mut profile: ControlProfile = (*command).control_profile;
    (*output).throttle = fc_clamp(
        fc_finite((*command).target_throttle, 0.0f64),
        0.0f64,
        1.0f64,
    );
    (*output).wheel_steering = fc_clamp(
        fc_finite((*command).wheel_steering, 0.0f64),
        -1.0f64,
        1.0f64,
    );
    (*output).gear = (*command).gear;
    (*output).brakes = (*command).brakes;
    (*output).airbrakes = (profile as ::core::ffi::c_uint
        == PROFILE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_APPROACH as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint)
        && (*command).airbrakes as ::core::ffi::c_int != 0;
    (*output).diagnostics.profile = profile;
    let mut rollout_profile: bool = profile as ::core::ffi::c_uint
        == PROFILE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut rollout_pitch_hold: bool = rollout_profile as ::core::ffi::c_int != 0
        && (*command).has_target_aoa as ::core::ffi::c_int != 0
        && (*command).target_aoa.is_finite() as i32 != 0;
    let mut rollout_heading_hold: bool = rollout_profile as ::core::ffi::c_int != 0
        && (*command).heading_control_enabled as ::core::ffi::c_int != 0;
    if rollout_profile as ::core::ffi::c_int != 0 && !rollout_pitch_hold
        && !rollout_heading_hold
    {
        (*output).valid = true_0 != 0;
        (*output).diagnostics.rcs_transonic_cutoff = true_0 != 0;
        return true_0 != 0;
    }
    if !rollout_profile && !fc_atmospheric_profile(profile) {
        return false_0 != 0;
    }
    let mut quaternion_control: bool = !rollout_profile && !attitude_error.is_null()
        && (*attitude_error).valid as ::core::ffi::c_int != 0
        && (*attitude_error).pitch_error_deg.is_finite() as i32 != 0
        && (*attitude_error).roll_error_deg.is_finite() as i32 != 0
        && (*attitude_error).yaw_error_deg.is_finite() as i32 != 0;
    let mut quaternion_yaw_control: bool = quaternion_control as ::core::ffi::c_int != 0
        && (*attitude_error).use_yaw_error as ::core::ffi::c_int != 0
        && !(*command).heading_control_enabled
        && profile as ::core::ffi::c_uint
            == PROFILE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut measured_dt: ::core::ffi::c_double = if sample_dt.is_finite() as i32 != 0
        && sample_dt > 0.0f64
    {
        sample_dt
    } else {
        0.0f64
    };
    if (*state).has_sample as ::core::ffi::c_int != 0
        && (*telemetry).ut.is_finite() as i32 != 0 && (*telemetry).ut < (*state).last_ut
    {
        flight_control_reset_transients(state);
    }
    if (*telemetry).has_main_gear_grounded as ::core::ffi::c_int != 0
        && (*telemetry).main_gear_grounded as ::core::ffi::c_int != 0
    {
        if !(*state).main_gear_contact_latched {
            (*state).main_gear_contact_latched = true_0 != 0;
            (*state).main_gear_contact_ut = (*telemetry).ut;
        }
    }
    if (*state).main_gear_contact_latched {
        let mut nose_contact: bool = (*telemetry).has_nose_gear_grounded
            as ::core::ffi::c_int != 0
            && (*telemetry).nose_gear_grounded as ::core::ffi::c_int != 0
            || (*telemetry).pitch <= 0.5f64;
        if nose_contact {
            (*state).nose_gear_contact_latched = true_0 != 0;
        }
    }
    if !(measured_dt > 0.0f64) && (*state).has_sample as ::core::ffi::c_int != 0
        && (*telemetry).ut.is_finite() as i32 != 0
    {
        measured_dt = (*telemetry).ut - (*state).last_ut;
    }
    let mut control_dt: ::core::ffi::c_double = if measured_dt > 0.0f64 {
        measured_dt
    } else {
        (*state).nominal_dt
    };
    if !(control_dt > 0.0f64) || control_dt.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut sample_valid: bool = (*state).has_sample as ::core::ffi::c_int != 0
        && measured_dt > 0.0f64;
    let mut pitch: ::core::ffi::c_double = fc_finite((*telemetry).pitch, 0.0f64);
    let mut aoa: ::core::ffi::c_double = fc_finite((*telemetry).angle_of_attack, 0.0f64);
    let mut roll: ::core::ffi::c_double = fc_signed_angle(
        fc_finite((*telemetry).roll, 0.0f64),
    );
    let mut heading: ::core::ffi::c_double = fc_signed_angle(
        fc_finite((*telemetry).heading, 0.0f64),
    );
    let mut sideslip: ::core::ffi::c_double = fc_signed_angle(
        fc_finite((*telemetry).sideslip, 0.0f64),
    );
    let mut q: ::core::ffi::c_double = fmax(
        0.0f64,
        fc_finite((*telemetry).dynamic_pressure, 0.0f64),
    );
    let mut vertical_speed: ::core::ffi::c_double = fc_finite(
        (*telemetry).vertical_speed,
        0.0f64,
    );
    let mut horizontal_speed: ::core::ffi::c_double = fabs(
        fc_finite((*telemetry).horizontal_speed, 0.0f64),
    );
    let mut flight_path_angle: ::core::ffi::c_double = if (*telemetry)
        .flight_path_angle
        .is_finite() as i32 != 0
    {
        (*telemetry).flight_path_angle
    } else {
        atan2(vertical_speed, fmax(horizontal_speed, DBL_EPSILON)) * RAD2DEG
    };
    if sample_valid {
        (*state).pitch_rate = (pitch - (*state).last_pitch) / measured_dt;
        (*state).aoa_rate = fc_signed_angle(aoa - (*state).last_aoa) / measured_dt;
        (*state).roll_rate = fc_signed_angle(roll - (*state).last_roll) / measured_dt;
        (*state).heading_rate = fc_signed_angle(heading - (*state).last_heading)
            / measured_dt;
        (*state).sideslip_rate = fc_signed_angle(sideslip - (*state).last_sideslip)
            / measured_dt;
    }
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < FLIGHT_CONTROL_AXIS_COUNT as ::core::ffi::c_int {
        fc_axis_set_reported(
            (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
                .offset(i as isize) as *mut FlightControlAxisAuthority,
            fc_reported_axis_accel(telemetry, i as FlightControlAxis),
        );
        i += 1;
    }
    if (*telemetry).attitude_response.pitch_valid {
        (*state)
            .authority[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize]
            .controlled_accel = (*telemetry)
            .attitude_response
            .maximum_pitch_accel_deg_s2;
        (*state)
            .authority[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize]
            .controlled_confidence = 1.0f64;
    }
    if (*telemetry).attitude_response.roll_valid {
        (*state)
            .authority[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize]
            .controlled_accel = (*telemetry).attitude_response.maximum_roll_accel_deg_s2;
        (*state)
            .authority[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize]
            .controlled_confidence = 1.0f64;
    }
    let mut body_pitch_rate_valid: bool = (*telemetry).has_body_pitch_rate
        as ::core::ffi::c_int != 0
        && (*telemetry).body_pitch_rate.is_finite() as i32 != 0;
    let mut body_roll_rate_valid: bool = (*telemetry).has_body_roll_rate
        as ::core::ffi::c_int != 0
        && (*telemetry).body_roll_rate.is_finite() as i32 != 0;
    let mut body_yaw_rate_valid: bool = (*telemetry).has_body_yaw_rate
        as ::core::ffi::c_int != 0 && (*telemetry).body_yaw_rate.is_finite() as i32 != 0;
    let mut body_pitch_rate_stale: bool = body_pitch_rate_valid as ::core::ffi::c_int
        != 0 && fabs((*telemetry).body_pitch_rate) <= DBL_EPSILON
        && fabs((*state).aoa_rate) > DBL_EPSILON;
    let mut body_roll_rate_stale: bool = body_roll_rate_valid as ::core::ffi::c_int != 0
        && fabs((*telemetry).body_roll_rate) <= DBL_EPSILON
        && fabs((*state).roll_rate) > DBL_EPSILON;
    let mut observed_pitch_rate: ::core::ffi::c_double = if (*telemetry)
        .has_angle_of_attack_rate as ::core::ffi::c_int != 0
        && (*telemetry).angle_of_attack_rate.is_finite() as i32 != 0
    {
        (*telemetry).angle_of_attack_rate
    } else {
        (*state).aoa_rate
    };
    if rollout_pitch_hold {
        observed_pitch_rate = if (*telemetry).pitch_rate.is_finite() as i32 != 0 {
            (*telemetry).pitch_rate
        } else {
            (*state).pitch_rate
        };
    } else if quaternion_control as ::core::ffi::c_int != 0
        && body_pitch_rate_valid as ::core::ffi::c_int != 0
    {
        observed_pitch_rate = (*telemetry).body_pitch_rate;
    }
    let mut live_body_roll: bool = body_roll_rate_valid as ::core::ffi::c_int != 0
        && !(fabs((*telemetry).body_roll_rate) <= DBL_EPSILON
            && (*telemetry).roll_rate.is_finite() as i32 != 0
            && fabs((*telemetry).roll_rate) > DBL_EPSILON);
    let mut observed_roll_rate: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0 && body_roll_rate_valid as ::core::ffi::c_int != 0
    {
        (*telemetry).body_roll_rate
    } else if live_body_roll as ::core::ffi::c_int != 0 {
        (*telemetry).body_roll_rate
    } else if (*telemetry).roll_rate.is_finite() as i32 != 0 {
        (*telemetry).roll_rate
    } else {
        (*state).roll_rate
    };
    let mut observed_body_yaw_rate: ::core::ffi::c_double = if body_yaw_rate_valid
        as ::core::ffi::c_int != 0
    {
        (*telemetry).body_yaw_rate
    } else if (*telemetry).heading_rate.is_finite() as i32 != 0 {
        (*telemetry).heading_rate
    } else {
        (*state).heading_rate
    };
    let mut observed_yaw_coordinate_rate: ::core::ffi::c_double = if quaternion_yaw_control
        as ::core::ffi::c_int != 0
    {
        observed_body_yaw_rate
    } else if (*command).heading_control_enabled as ::core::ffi::c_int != 0 {
        if (*telemetry).heading_rate.is_finite() as i32 != 0 {
            (*telemetry).heading_rate
        } else {
            (*state).heading_rate
        }
    } else {
        -(*state).sideslip_rate
    };
    let mut yaw_frame: ::core::ffi::c_int = if quaternion_yaw_control
        as ::core::ffi::c_int != 0
    {
        2 as ::core::ffi::c_int
    } else if (*command).heading_control_enabled as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
    if (*state).has_yaw_observation_frame as ::core::ffi::c_int != 0
        && (*state).yaw_observation_frame != yaw_frame
    {
        let mut axis: *mut FlightControlAxisAuthority = (&raw mut (*state).authority
            as *mut FlightControlAxisAuthority)
            .offset(FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as isize)
            as *mut FlightControlAxisAuthority;
        (*axis).controlled_accel = 0.0f64;
        (*axis).controlled_confidence = 0.0f64;
        (*axis).live_response_confidence = 0.0f64;
        (*axis).drift_accel = 0.0f64;
        (*axis).has_last_rate = false_0 != 0;
    }
    (*state).yaw_observation_frame = yaw_frame;
    (*state).has_yaw_observation_frame = true_0 != 0;
    let mut tuning: *const FlightControlTuning = fc_tuning();
    if sample_valid {
        fc_axis_observe(
            (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
                .offset(FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as isize)
                as *mut FlightControlAxisAuthority,
            observed_pitch_rate,
            (*state)
                .last_control[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize],
            measured_dt,
            0.0f64,
            0.0f64,
        );
        fc_axis_observe(
            (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
                .offset(FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as isize)
                as *mut FlightControlAxisAuthority,
            observed_roll_rate,
            (*state)
                .last_control[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize],
            measured_dt,
            (*state)
                .last_control[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize],
            (*tuning).roll_lag_s,
        );
        fc_axis_observe(
            (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
                .offset(FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as isize)
                as *mut FlightControlAxisAuthority,
            observed_yaw_coordinate_rate,
            (*state)
                .last_control[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize],
            measured_dt,
            (*state)
                .last_control[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize],
            (*tuning).yaw_lag_s,
        );
    }
    let mut target_pitch_state: ::core::ffi::c_double = if rollout_pitch_hold
        as ::core::ffi::c_int != 0
    {
        (*command).target_pitch
    } else if (*command).has_target_aoa as ::core::ffi::c_int != 0
        && (*command).target_aoa.is_finite() as i32 != 0
    {
        (*command).target_aoa
    } else {
        fc_finite((*command).target_pitch, 0.0f64) - flight_path_angle
    };
    let mut measured_pitch_state: ::core::ffi::c_double = if rollout_pitch_hold
        as ::core::ffi::c_int != 0
    {
        pitch
    } else {
        aoa
    };
    let mut aerodynamic_pitch_error: ::core::ffi::c_double = target_pitch_state
        - measured_pitch_state;
    let mut pitch_error: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0
    {
        (*attitude_error).pitch_error_deg
    } else {
        aerodynamic_pitch_error
    };
    if (*state).has_last_target_pitch {
        let mut raw_target_rate: ::core::ffi::c_double = (target_pitch_state
            - (*state).last_target_pitch_state) / control_dt;
        raw_target_rate = fc_clamp(raw_target_rate, -6.0f64, 6.0f64);
        (*state).target_pitch_rate
            += fc_clamp(control_dt / 0.25f64, 0.0f64, 1.0f64)
                * (raw_target_rate - (*state).target_pitch_rate);
    } else {
        (*state).target_pitch_rate = 0.0f64;
    }
    (*state).last_target_pitch_state = target_pitch_state;
    (*state).has_last_target_pitch = true_0 != 0;
    let mut roll_target: ::core::ffi::c_double = fc_signed_angle(
        fc_finite((*command).target_roll, 0.0f64),
    );
    let mut roll_error: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0
    {
        (*attitude_error).roll_error_deg
    } else {
        fc_signed_angle(roll_target - roll)
    };
    if (*state).has_last_target_roll {
        (*state).target_roll_rate = fc_signed_angle(
            roll_target - (*state).last_target_roll,
        ) / control_dt;
    } else {
        (*state).target_roll_rate = 0.0f64;
    }
    (*state).last_target_roll = roll_target;
    (*state).has_last_target_roll = true_0 != 0;
    if quaternion_control {
        let mut continuous_error: bool = (*state).has_quaternion_error_sample
            as ::core::ffi::c_int != 0 && sample_valid as ::core::ffi::c_int != 0
            && (*state).has_last_profile as ::core::ffi::c_int != 0
            && (*state).last_profile as ::core::ffi::c_uint
                == profile as ::core::ffi::c_uint;
        if continuous_error {
            let mut raw_pitch_rate: ::core::ffi::c_double = fc_signed_angle(
                pitch_error - (*state).last_quaternion_pitch_error,
            ) / measured_dt;
            let mut raw_roll_rate: ::core::ffi::c_double = fc_signed_angle(
                roll_error - (*state).last_quaternion_roll_error,
            ) / measured_dt;
            let mut raw_yaw_rate: ::core::ffi::c_double = fc_signed_angle(
                (*attitude_error).yaw_error_deg - (*state).last_quaternion_yaw_error,
            ) / measured_dt;
            (*state).quaternion_pitch_error_rate = fc_lowpass_rate(
                (*state).quaternion_pitch_error_rate,
                raw_pitch_rate,
                control_dt,
                (*tuning).pitch_lag_s,
            );
            (*state).quaternion_roll_error_rate = fc_lowpass_rate(
                (*state).quaternion_roll_error_rate,
                raw_roll_rate,
                control_dt,
                (*tuning).roll_lag_s,
            );
            (*state).quaternion_yaw_error_rate = fc_lowpass_rate(
                (*state).quaternion_yaw_error_rate,
                raw_yaw_rate,
                control_dt,
                (*tuning).yaw_lag_s,
            );
        } else {
            (*state).quaternion_pitch_error_rate = 0.0f64;
            (*state).quaternion_roll_error_rate = 0.0f64;
            (*state).quaternion_yaw_error_rate = 0.0f64;
        }
        (*state).last_quaternion_pitch_error = pitch_error;
        (*state).last_quaternion_roll_error = roll_error;
        (*state).last_quaternion_yaw_error = (*attitude_error).yaw_error_deg;
        (*state).has_quaternion_error_sample = true_0 != 0;
    } else {
        (*state).has_quaternion_error_sample = false_0 != 0;
        (*state).quaternion_pitch_error_rate = 0.0f64;
        (*state).quaternion_roll_error_rate = 0.0f64;
        (*state).quaternion_yaw_error_rate = 0.0f64;
    }
    let mut observed_pitch_authority: ::core::ffi::c_double = fc_axis_available(
        (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
            .offset(FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as isize)
            as *mut FlightControlAxisAuthority,
        q,
    );
    let mut reported_pitch_authority: ::core::ffi::c_double = fc_reported_axis_accel(
        telemetry,
        FLIGHT_CONTROL_AXIS_PITCH,
    );
    let mut raw_pitch_authority: ::core::ffi::c_double = if reported_pitch_authority
        > DBL_EPSILON
    {
        reported_pitch_authority
    } else {
        observed_pitch_authority
    };
    let mut observed_roll_authority: ::core::ffi::c_double = fc_axis_available(
        (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
            .offset(FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as isize)
            as *mut FlightControlAxisAuthority,
        q,
    );
    let mut reported_roll_authority: ::core::ffi::c_double = fc_reported_axis_accel(
        telemetry,
        FLIGHT_CONTROL_AXIS_ROLL,
    );
    let mut raw_roll_authority: ::core::ffi::c_double = if reported_roll_authority
        > DBL_EPSILON
    {
        reported_roll_authority
    } else {
        observed_roll_authority
    };
    if reported_roll_authority > DBL_EPSILON && observed_roll_authority > DBL_EPSILON {
        let mut confidence: ::core::ffi::c_double = fc_clamp(
            (*state)
                .authority[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize]
                .live_response_confidence,
            0.0f64,
            1.0f64,
        );
        raw_roll_authority = 1.0f64
            / ((1.0f64 - confidence) / reported_roll_authority
                + confidence / observed_roll_authority);
    }
    let mut observed_yaw_authority: ::core::ffi::c_double = fc_axis_available(
        (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
            .offset(FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as isize)
            as *mut FlightControlAxisAuthority,
        q,
    );
    let mut reported_yaw_authority: ::core::ffi::c_double = fc_reported_axis_accel(
        telemetry,
        FLIGHT_CONTROL_AXIS_YAW,
    );
    let mut yaw_authority: ::core::ffi::c_double = if reported_yaw_authority
        > DBL_EPSILON
    {
        reported_yaw_authority
    } else {
        observed_yaw_authority
    };
    if reported_yaw_authority > DBL_EPSILON && observed_yaw_authority > DBL_EPSILON {
        let mut confidence_0: ::core::ffi::c_double = fc_clamp(
            (*state)
                .authority[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize]
                .live_response_confidence,
            0.0f64,
            1.0f64,
        );
        yaw_authority = 1.0f64
            / ((1.0f64 - confidence_0) / reported_yaw_authority
                + confidence_0 / observed_yaw_authority);
    }
    let mut pitch_authority: ::core::ffi::c_double = raw_pitch_authority;
    let mut roll_authority: ::core::ffi::c_double = raw_roll_authority;
    let roll_accel_max: ::core::ffi::c_double = (*tuning).roll_accel_max;
    if !(roll_authority > DBL_EPSILON) || roll_authority > roll_accel_max {
        roll_authority = roll_accel_max;
    }
    let mut pitch_guard_known: bool = fc_axis_control_authority_known(
        (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
            .offset(FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as isize)
            as *mut FlightControlAxisAuthority,
    );
    let mut roll_guard_known: bool = fc_axis_control_authority_known(
        (&raw mut (*state).authority as *mut FlightControlAxisAuthority)
            .offset(FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as isize)
            as *mut FlightControlAxisAuthority,
    );
    let mut pitch_guard_authority: ::core::ffi::c_double = if pitch_guard_known
        as ::core::ffi::c_int != 0
    {
        pitch_authority
    } else {
        0.0f64
    };
    let mut roll_guard_authority: ::core::ffi::c_double = if roll_guard_known
        as ::core::ffi::c_int != 0
    {
        roll_authority
    } else {
        0.0f64
    };
    let mut pitch_aero_fraction: ::core::ffi::c_double = fc_clamp(
        fmax(
            0.0f64,
            (*state)
                .authority[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize]
                .aero_per_q,
        ) * q / raw_pitch_authority,
        0.0f64,
        1.0f64,
    );
    let mut roll_aero_fraction: ::core::ffi::c_double = fc_clamp(
        fmax(
            0.0f64,
            (*state)
                .authority[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize]
                .aero_per_q,
        ) * q / raw_roll_authority,
        0.0f64,
        1.0f64,
    );
    let mut yaw_aero_fraction: ::core::ffi::c_double = fc_clamp(
        fmax(
            0.0f64,
            (*state)
                .authority[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize]
                .aero_per_q,
        ) * q / yaw_authority,
        0.0f64,
        1.0f64,
    );
    let mut effective_pitch_rate: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0
    {
        -(*state).quaternion_pitch_error_rate
    } else {
        observed_pitch_rate
    };
    let mut effective_roll_rate: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0
    {
        -(*state).quaternion_roll_error_rate
    } else {
        observed_roll_rate
    };
    (*state).terminal_pitch_authority = pitch_authority;
    (*state).terminal_roll_authority = roll_authority;
    (*state).has_terminal_pitch_authority = true_0 != 0;
    (*state).has_terminal_roll_authority = true_0 != 0;
    let pitch_accel_max: ::core::ffi::c_double = (*tuning).pitch_accel_max;
    let pitch_lag_s: ::core::ffi::c_double = (*tuning).pitch_lag_s;
    let pitch_wn: ::core::ffi::c_double = (*tuning).pitch_wn;
    let pitch_zeta: ::core::ffi::c_double = (*tuning).pitch_zeta;
    let roll_lag_s: ::core::ffi::c_double = (*tuning).roll_lag_s;
    let roll_wn: ::core::ffi::c_double = (*tuning).roll_wn;
    let roll_zeta: ::core::ffi::c_double = (*tuning).roll_zeta;
    let mut pitch_auth_used: ::core::ffi::c_double = fc_pitch_accel_for_profile(
        pitch_authority,
        pitch_accel_max,
        profile,
        (*telemetry).radar_altitude,
    );
    let mut pitch_wn_used: ::core::ffi::c_double = pitch_wn;
    let mut roll_wn_used: ::core::ffi::c_double = roll_wn;
    let mut roll_bandwidth_scale: ::core::ffi::c_double = 1.0f64;
    let mut adaptive_pitch: bool = profile as ::core::ffi::c_uint
        == PROFILE_ENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint;
    if rollout_pitch_hold {
        pitch_wn_used = 0.55f64;
    } else {
        fc_schedule_bandwidth(
            q,
            (*telemetry).mach,
            profile,
            pitch_wn,
            roll_wn,
            &raw mut pitch_wn_used,
            &raw mut roll_wn_used,
            &raw mut roll_bandwidth_scale,
        );
        if adaptive_pitch {
            let mut pitch_commanded_rate: ::core::ffi::c_double = if quaternion_control
                as ::core::ffi::c_int != 0
            {
                0.0f64
            } else {
                (*state).target_pitch_rate
            };
            let mut pitch_relative_rate: ::core::ffi::c_double = effective_pitch_rate
                - pitch_commanded_rate;
            (*state).pitch_response_scale = fc_adapt_response_scale(
                (*state).pitch_response_scale,
                pitch_error,
                pitch_relative_rate,
                pitch_wn_used,
                control_dt,
            );
            pitch_wn_used *= (*state).pitch_response_scale;
        } else {
            (*state).pitch_response_scale = 1.0f64;
        }
    }
    let mut pitch_zeta_used: ::core::ffi::c_double = if rollout_pitch_hold
        as ::core::ffi::c_int != 0
    {
        1.45f64
    } else if adaptive_pitch as ::core::ffi::c_int != 0 {
        fc_clamp(
            pitch_zeta / fmax((*state).pitch_response_scale, 0.40f64),
            pitch_zeta,
            2.4f64,
        )
    } else {
        pitch_zeta
    };
    let mut pitch_commanded_rate_0: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0
    {
        0.0f64
    } else {
        (*state).target_pitch_rate
    };
    if profile as ::core::ffi::c_uint
        == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
        && pitch_error > 0.0f64 && vertical_speed < -0.5f64
        && (*telemetry).radar_altitude.is_finite() as i32 != 0
        && (*telemetry).radar_altitude > 0.0f64
    {
        let main_gear_down: ::core::ffi::c_double = 3.1535f64;
        let mut wheel_height: ::core::ffi::c_double = fmax(
            0.35f64,
            (*telemetry).radar_altitude - main_gear_down,
        );
        let mut time_to_mains: ::core::ffi::c_double = fmax(
            0.40f64,
            wheel_height / fmax(0.5f64, -vertical_speed),
        );
        let mut closure_rate: ::core::ffi::c_double = fc_clamp(
            pitch_error / time_to_mains,
            0.0f64,
            6.0f64,
        );
        pitch_commanded_rate_0 = fmax(pitch_commanded_rate_0, closure_rate);
    }
    let mut pitch_command: ::core::ffi::c_double = fc_capped_pd_command(
        pitch_error,
        effective_pitch_rate,
        pitch_commanded_rate_0,
        pitch_auth_used,
        pitch_wn_used,
        pitch_zeta_used,
        pitch_lag_s,
        (*state).last_control[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize],
    );
    let mut pitch_feedforward: ::core::ffi::c_double = if profile as ::core::ffi::c_uint
        == PROFILE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*command).has_pitch_trim_feedforward as ::core::ffi::c_int != 0
        && (*command).pitch_trim_feedforward.is_finite() as i32 != 0
    {
        fc_clamp((*command).pitch_trim_feedforward, -0.75f64, 0.75f64)
    } else {
        0.0f64
    };
    (*state).pitch_trim -= pitch_feedforward - (*state).applied_pitch_trim_feedforward;
    (*state).applied_pitch_trim_feedforward = pitch_feedforward;
    let mut trim_active: bool = !(*state).main_gear_contact_latched
        && fc_atmospheric_profile(profile) as ::core::ffi::c_int != 0 && q > 100.0f64;
    if trim_active {
        let mut in_approach: bool = (profile as ::core::ffi::c_uint
            == PROFILE_APPROACH as ::core::ffi::c_int as ::core::ffi::c_uint
            || profile as ::core::ffi::c_uint
                == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint)
            && (*telemetry).radar_altitude.is_finite() as i32 != 0
            && (*telemetry).radar_altitude < 700.0f64;
        let mut ki: ::core::ffi::c_double = if in_approach as ::core::ffi::c_int != 0 {
            (*tuning).pitch_trim_ki
        } else if profile as ::core::ffi::c_uint
            == PROFILE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            0.025f64
        } else {
            0.008f64
        };
        let mut elevator_total: ::core::ffi::c_double = pitch_command
            + (*state).pitch_trim + pitch_feedforward;
        let mut saturated_high: bool = elevator_total >= 1.0f64
            && aerodynamic_pitch_error > 0.0f64;
        let mut saturated_low: bool = elevator_total <= -1.0f64
            && aerodynamic_pitch_error < 0.0f64;
        if !saturated_high && !saturated_low && fabs(aerodynamic_pitch_error) > 0.20f64 {
            (*state).pitch_trim += ki * aerodynamic_pitch_error * control_dt;
        }
        let mut taem_trim: bool = profile as ::core::ffi::c_uint
            == PROFILE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint;
        let mut residual_trim: bool = taem_trim as ::core::ffi::c_int != 0
            && (*command).has_pitch_trim_feedforward as ::core::ffi::c_int != 0;
        let mut trim_min: ::core::ffi::c_double = if in_approach as ::core::ffi::c_int
            != 0
        {
            -0.18f64
        } else if residual_trim as ::core::ffi::c_int != 0 {
            -0.25f64
        } else if taem_trim as ::core::ffi::c_int != 0 {
            -0.60f64
        } else {
            -0.35f64
        };
        let mut trim_max: ::core::ffi::c_double = if in_approach as ::core::ffi::c_int
            != 0
        {
            0.42f64
        } else if residual_trim as ::core::ffi::c_int != 0 {
            0.25f64
        } else if taem_trim as ::core::ffi::c_int != 0 {
            0.90f64
        } else {
            0.50f64
        };
        if in_approach as ::core::ffi::c_int != 0
            && (*command).terminal_roll_trim_enabled as ::core::ffi::c_int != 0
        {
            trim_min = -1.0f64;
            trim_max = 1.0f64;
        }
        (*state).pitch_trim = fc_clamp((*state).pitch_trim, trim_min, trim_max);
    } else {
        (*state).pitch_trim *= fmax(0.0f64, 1.0f64 - control_dt);
    }
    (*state).terminal_pitch_integral = (*state).pitch_trim;
    pitch_command = fc_clamp(
        pitch_command + pitch_feedforward + (*state).pitch_trim,
        -1.0f64,
        1.0f64,
    );
    if !(*state).main_gear_contact_latched && adaptive_pitch as ::core::ffi::c_int != 0 {
        pitch_command = fc_smooth_actuator_command(
            pitch_command,
            (*state)
                .last_control[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize],
            control_dt,
            pitch_lag_s / fmax((*state).pitch_response_scale, 0.40f64),
        );
    }
    if (*state).main_gear_contact_latched {
        (*state).pitch_trim = 0.0f64;
        (*state).terminal_pitch_integral = 0.0f64;
        (*state).has_last_target_pitch = false_0 != 0;
        (*state).target_pitch_rate = 0.0f64;
        (*state)
            .last_control[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize] = 0.0f64;
        pitch_command = 0.0f64;
    } else if rollout_profile {
        if rollout_pitch_hold {
            pitch_command = fc_clamp(pitch_command, -0.30f64, 0.30f64);
        } else {
            pitch_command = 0.0f64;
        }
    }
    let mut commanded_roll_rate: ::core::ffi::c_double = if quaternion_control
        as ::core::ffi::c_int != 0
    {
        0.0f64
    } else {
        (*state).target_roll_rate
    };
    let mut relative_rate: ::core::ffi::c_double = effective_roll_rate
        - commanded_roll_rate;
    let mut roll_command: ::core::ffi::c_double = fc_capped_pd_command(
        roll_error,
        effective_roll_rate,
        commanded_roll_rate,
        roll_authority,
        roll_wn_used,
        roll_zeta,
        roll_lag_s,
        (*state).last_control[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize],
    );
    let mut terminal_roll_trim: bool = (*command).terminal_roll_trim_enabled
        as ::core::ffi::c_int != 0
        && (profile as ::core::ffi::c_uint
            == PROFILE_APPROACH as ::core::ffi::c_int as ::core::ffi::c_uint
            || profile as ::core::ffi::c_uint
                == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint)
        && !(*state).main_gear_contact_latched;
    if terminal_roll_trim as ::core::ffi::c_int != 0 && roll_wn_used > DBL_EPSILON {
        let mut response_time: ::core::ffi::c_double = 2.0f64 * roll_zeta / roll_wn_used
            + roll_lag_s;
        let mut increment: ::core::ffi::c_double = control_dt * roll_wn_used
            * roll_wn_used * roll_error / (roll_authority * response_time);
        let mut candidate: ::core::ffi::c_double = fc_clamp(
            (*state).roll_trim + increment,
            -1.0f64,
            1.0f64,
        );
        if fabs(roll_command + candidate) <= 1.0f64
            || (roll_command + candidate) * increment < 0.0f64
        {
            (*state).roll_trim = candidate;
        }
        roll_command = fc_clamp(roll_command + (*state).roll_trim, -1.0f64, 1.0f64);
    } else {
        (*state).roll_trim = 0.0f64;
    }
    let mut heading_error: ::core::ffi::c_double = 0.0f64;
    let mut yaw_error: ::core::ffi::c_double = sideslip;
    let mut yaw_coordinate_rate: ::core::ffi::c_double = observed_yaw_coordinate_rate;
    let mut yaw_target_rate: ::core::ffi::c_double = 0.0f64;
    if (*command).heading_control_enabled {
        heading_error = fc_signed_angle(
            fc_finite((*command).target_heading, 0.0f64) - heading,
        );
        yaw_error = heading_error;
    } else if quaternion_yaw_control {
        yaw_error = (*attitude_error).yaw_error_deg;
        yaw_coordinate_rate = -(*state).quaternion_yaw_error_rate;
    }
    let yaw_accel_max: ::core::ffi::c_double = (*tuning).yaw_accel_max;
    let mut yaw_wn: ::core::ffi::c_double = (*tuning).yaw_wn;
    if !(yaw_authority > DBL_EPSILON) || yaw_authority > yaw_accel_max {
        yaw_authority = yaw_accel_max;
    }
    if profile as ::core::ffi::c_uint
        != PROFILE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint
        && !(*command).heading_control_enabled
    {
        yaw_wn = fmin(yaw_wn, sqrt(yaw_authority / fmax(fabs(yaw_error), DBL_EPSILON)));
        if (*tuning).yaw_lag_s > DBL_EPSILON {
            yaw_wn = fmin(yaw_wn, 1.0f64 / (2.0f64 * (*tuning).yaw_lag_s));
        }
    }
    let mut yaw_command: ::core::ffi::c_double = fc_capped_pd_command(
        yaw_error,
        yaw_coordinate_rate,
        yaw_target_rate,
        yaw_authority,
        yaw_wn,
        1.0f64,
        (*tuning).yaw_lag_s,
        (*state).last_control[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize],
    );
    let mut rcs_assist: ::core::ffi::c_double = 0.0f64;
    let mut recovery_rate_first: bool = profile as ::core::ffi::c_uint
        == PROFILE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint
        && quaternion_control as ::core::ffi::c_int != 0
        && body_pitch_rate_valid as ::core::ffi::c_int != 0
        && body_roll_rate_valid as ::core::ffi::c_int != 0
        && body_yaw_rate_valid as ::core::ffi::c_int != 0
        && pitch_authority > DBL_EPSILON && roll_authority > DBL_EPSILON
        && yaw_authority > DBL_EPSILON
        && (2.0f64 * pitch_zeta * pitch_wn_used * fabs((*telemetry).body_pitch_rate)
            >= pitch_authority
            || 2.0f64 * roll_zeta * roll_wn_used * fabs((*telemetry).body_roll_rate)
                >= roll_authority
            || 2.0f64 * yaw_wn * fabs((*telemetry).body_yaw_rate) >= yaw_authority);
    if recovery_rate_first {
        pitch_command = fc_capped_pd_command(
            0.0f64,
            (*telemetry).body_pitch_rate,
            0.0f64,
            pitch_authority,
            pitch_wn_used,
            pitch_zeta,
            0.0f64,
            0.0f64,
        );
        roll_command = fc_capped_pd_command(
            0.0f64,
            (*telemetry).body_roll_rate,
            0.0f64,
            roll_authority,
            roll_wn_used,
            roll_zeta,
            0.0f64,
            0.0f64,
        );
        yaw_command = fc_capped_pd_command(
            0.0f64,
            (*telemetry).body_yaw_rate,
            0.0f64,
            yaw_authority,
            yaw_wn,
            1.0f64,
            0.0f64,
            0.0f64,
        );
        (*state).pitch_trim = 0.0f64;
        (*state).terminal_pitch_integral = 0.0f64;
    }
    (*state).last_control[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize] = fc_clamp(
        pitch_command,
        -1.0f64,
        1.0f64,
    );
    (*state).last_control[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize] = fc_clamp(
        roll_command,
        -1.0f64,
        1.0f64,
    );
    (*state).last_control[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize] = fc_clamp(
        yaw_command,
        -1.0f64,
        1.0f64,
    );
    (*output).pitch = (*state)
        .last_control[FLIGHT_CONTROL_AXIS_PITCH as ::core::ffi::c_int as usize];
    (*output).roll = if rollout_profile as ::core::ffi::c_int != 0 {
        0.0f64
    } else {
        (*state).last_control[FLIGHT_CONTROL_AXIS_ROLL as ::core::ffi::c_int as usize]
    };
    (*output).yaw = if rollout_heading_hold as ::core::ffi::c_int != 0 {
        (*state).last_control[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize]
    } else if rollout_profile as ::core::ffi::c_int != 0 {
        0.0f64
    } else {
        (*state).last_control[FLIGHT_CONTROL_AXIS_YAW as ::core::ffi::c_int as usize]
    };
    let mut rcs_profile: bool = profile as ::core::ffi::c_uint
        == PROFILE_ENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
        || profile as ::core::ffi::c_uint
            == PROFILE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut axis_saturated: bool = false_0 != 0;
    let mut axis_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while axis_0 < FLIGHT_CONTROL_AXIS_COUNT as ::core::ffi::c_int {
        if fabs((*state).last_control[axis_0 as usize]) >= 1.0f64 - DBL_EPSILON {
            axis_saturated = true_0 != 0;
        }
        axis_0 += 1;
    }
    if !rcs_profile {
        (*state).rcs_q_latched = false_0 != 0;
        (*state).rcs_saturation_latched = false_0 != 0;
    } else {
        if q >= (*tuning).rcs_q_on_pa {
            (*state).rcs_q_latched = true_0 != 0;
        } else if q <= (*tuning).rcs_q_off_pa {
            (*state).rcs_q_latched = false_0 != 0;
        }
        if axis_saturated {
            (*state).rcs_saturation_latched = true_0 != 0;
        } else if fabs(sideslip) < (*tuning).rcs_release_beta_deg {
            (*state).rcs_saturation_latched = false_0 != 0;
        }
    }
    (*state).rcs_latched = (*state).rcs_q_latched as ::core::ffi::c_int != 0
        || (*state).rcs_saturation_latched as ::core::ffi::c_int != 0;
    (*state).rcs_transonic_cutoff = !(*state).rcs_latched;
    if (*state).rcs_latched {
        rcs_assist = 1.0f64;
    }
    (*output).rcs_assist = rcs_assist;
    (*output).rcs_requested = (*state).rcs_latched;
    (*output).valid = true_0 != 0;
    let mut d: *mut FlightControlDiagnostics = &raw mut (*output).diagnostics;
    (*d).target_pitch_state = target_pitch_state;
    (*d).measured_pitch_state = measured_pitch_state;
    (*d).pitch_error = pitch_error;
    (*d).effective_pitch_rate = effective_pitch_rate;
    (*d).body_pitch_rate = if body_pitch_rate_valid as ::core::ffi::c_int != 0 {
        (*telemetry).body_pitch_rate
    } else {
        0.0f64
    };
    (*d).body_pitch_rate_available = body_pitch_rate_valid as ::core::ffi::c_int != 0
        && (quaternion_control as ::core::ffi::c_int != 0 || !body_pitch_rate_stale);
    (*d).roll_error = roll_error;
    (*d).effective_roll_rate = effective_roll_rate;
    (*d).body_roll_rate = if body_roll_rate_valid as ::core::ffi::c_int != 0 {
        (*telemetry).body_roll_rate
    } else {
        0.0f64
    };
    (*d).body_roll_rate_available = body_roll_rate_valid as ::core::ffi::c_int != 0
        && (quaternion_control as ::core::ffi::c_int != 0 || !body_roll_rate_stale);
    (*d).target_roll_rate = (*state).target_roll_rate;
    (*d).commanded_roll_rate = commanded_roll_rate;
    (*d).relative_roll_rate = relative_rate;
    (*d).recovery_rate_first = recovery_rate_first;
    (*d).heading_error = heading_error;
    (*d).sideslip = sideslip;
    (*d).sideslip_rate = (*state).sideslip_rate;
    (*d).body_yaw_rate = observed_body_yaw_rate;
    (*d).body_yaw_rate_available = body_yaw_rate_valid;
    (*d).flight_path_angle = flight_path_angle;
    (*d).pitch_trim = (*state).pitch_trim;
    (*d).steady_pitch_trim = (*state).pitch_trim;
    (*d).terminal_pitch_integral = (*state).terminal_pitch_integral;
    (*d).roll_trim = (*state).roll_trim;
    (*d).pitch_authority = pitch_authority;
    (*d).pitch_raw_authority = raw_pitch_authority;
    (*d).pitch_guard_authority = pitch_guard_authority;
    (*d).pitch_aero_fraction = pitch_aero_fraction;
    (*d).pitch_bandwidth_scale = (*state).pitch_response_scale;
    (*d).pitch_hold_seconds = control_dt;
    (*d).pitch_rate_impulse_budget = pitch_authority * control_dt;
    (*d).roll_authority = roll_authority;
    (*d).roll_raw_authority = raw_roll_authority;
    (*d).roll_guard_authority = roll_guard_authority;
    (*d).roll_aero_fraction = roll_aero_fraction;
    (*d).roll_rate_limit = sqrt(2.0f64 * roll_authority * 180.0f64);
    (*d).roll_command_limit = 1.0f64;
    (*d).roll_hold_seconds = control_dt;
    (*d).roll_rate_impulse_budget = roll_authority * control_dt;
    (*d).roll_bandwidth_scale = roll_bandwidth_scale;
    (*d).yaw_authority = yaw_authority;
    (*d).yaw_aero_fraction = yaw_aero_fraction;
    (*d).beta_yaw_gain = (*state).beta.yaw_gain;
    (*d).beta_roll_coupling = (*state).beta.roll_gain;
    (*d).beta_confidence = (*state).beta.confidence;
    (*d).sample_dt = control_dt;
    (*d).quaternion_attitude_control = quaternion_control;
    let mut controlled_yaw_error: ::core::ffi::c_double = if (*command)
        .heading_control_enabled as ::core::ffi::c_int != 0
    {
        heading_error
    } else if quaternion_yaw_control as ::core::ffi::c_int != 0 {
        (*attitude_error).yaw_error_deg
    } else {
        sideslip
    };
    (*d).attitude_error = fmax(
        fabs(pitch_error),
        fmax(fabs(roll_error), fabs(controlled_yaw_error)),
    );
    (*d).rcs_transonic_cutoff = !(*state).rcs_latched;
    (*state).last_ut = fc_finite((*telemetry).ut, (*state).last_ut + control_dt);
    (*state).last_pitch = pitch;
    (*state).last_aoa = aoa;
    (*state).last_roll = roll;
    (*state).last_heading = heading;
    (*state).last_sideslip = sideslip;
    (*state).has_sample = true_0 != 0;
    (*state).last_profile = profile;
    (*state).has_last_profile = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_step(
    mut state: *mut FlightControlState,
    mut telemetry: *const Telemetry,
    mut command: *const GuidanceCommand,
    mut sample_dt: ::core::ffi::c_double,
    mut output: *mut FlightControlOutput,
) -> bool {
    return flight_control_step_attitude(
        state,
        telemetry,
        command,
        ::core::ptr::null::<FlightControlAttitudeError>(),
        sample_dt,
        output,
    );
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_rollout_parachutes_ready(
    mut t: *const Telemetry,
    mut c: *const GuidanceCommand,
) -> bool {
    if t.is_null() || c.is_null()
        || (*c).control_profile as ::core::ffi::c_uint
            != PROFILE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint || !(*c).gear
        || !(*t).gear
    {
        return false_0 != 0;
    }
    let mut grounded: bool = (*t).has_main_gear_grounded as ::core::ffi::c_int != 0
        && (*t).main_gear_grounded as ::core::ffi::c_int != 0
        || (*t).has_nose_gear_grounded as ::core::ffi::c_int != 0
            && (*t).nose_gear_grounded as ::core::ffi::c_int != 0
        || strcmp(
            &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
            b"landed\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 as ::core::ffi::c_int;
    return grounded as ::core::ffi::c_int != 0
        && (*t).surface_speed.is_finite() as i32 != 0 && (*t).surface_speed >= 5.0f64
        && (*t).surface_speed <= 100.0f64;
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
