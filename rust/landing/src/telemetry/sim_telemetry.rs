use crate::sys::*;
extern "C" {
    fn json_parse(
        text: *const ::core::ffi::c_char,
        tokens: *mut JsonToken,
        capacity: ::core::ffi::c_int,
        doc: *mut JsonDoc,
    ) -> ::core::ffi::c_int;
    fn json_object_get(
        doc: *const JsonDoc,
        object_index: ::core::ffi::c_int,
        key: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    fn json_number(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        fallback: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn json_boolean(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        fallback: bool,
    ) -> bool;
    fn json_string(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        out: *mut ::core::ffi::c_char,
        out_size: size_t,
    ) -> bool;
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
    fn great_circle_distance(
        a: GeoPoint,
        b: GeoPoint,
        radius: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn initial_bearing(a: GeoPoint, b: GeoPoint) -> ::core::ffi::c_double;
    fn telemetry_init(t: *mut Telemetry);
}
pub type size_t = usize;
pub type JsonType = ::core::ffi::c_uint;
pub const JSON_PRIMITIVE: JsonType = 4;
pub const JSON_STRING: JsonType = 3;
pub const JSON_ARRAY: JsonType = 2;
pub const JSON_OBJECT: JsonType = 1;
pub const JSON_UNDEFINED: JsonType = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonToken {
    pub type_0: JsonType,
    pub start: ::core::ffi::c_int,
    pub end: ::core::ffi::c_int,
    pub size: ::core::ffi::c_int,
    pub parent: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonDoc {
    pub text: *const ::core::ffi::c_char,
    pub tokens: *mut JsonToken,
    pub count: ::core::ffi::c_int,
}
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
pub type NavballSpeedMode = ::core::ffi::c_uint;
pub const SPEED_SURFACE: NavballSpeedMode = 2;
pub const SPEED_ORBIT: NavballSpeedMode = 1;
pub const SPEED_UNCHANGED: NavballSpeedMode = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConnectionConfiguration {
    pub serial_port: [::core::ffi::c_char; 256],
    pub rpc_host: [::core::ffi::c_char; 256],
    pub rpc_port: ::core::ffi::c_int,
    pub baud_rate: ::core::ffi::c_int,
    pub timeout_ms: ::core::ffi::c_int,
    pub client_name: [::core::ffi::c_char; 128],
}
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
pub struct CalibrationSettings {
    pub enable_passive_calibration: bool,
    pub enable_trajectory_calibration: bool,
    pub auto_apply_in_flight: bool,
    pub minimum_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_sample_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_sideslip: ::core::ffi::c_double,
    pub minimum_calibration_radar_altitude: ::core::ffi::c_double,
    pub maximum_calibration_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_calibration_g_load: ::core::ffi::c_double,
    pub maximum_calibration_sink_rate: ::core::ffi::c_double,
    pub abort_stall_fraction: ::core::ffi::c_double,
    pub minimum_angle_of_attack: ::core::ffi::c_double,
    pub maximum_angle_of_attack: ::core::ffi::c_double,
    pub angle_of_attack_step: ::core::ffi::c_double,
    pub settling_duration: ::core::ffi::c_double,
    pub sampling_duration: ::core::ffi::c_double,
    pub trajectory_learning_rate: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LandingConfiguration {
    pub connection: ConnectionConfiguration,
    pub site: LandingSite,
    pub vehicle: VehicleProfile,
    pub guidance: GuidanceSettings,
    pub calibration: CalibrationSettings,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlanetModel {
    pub name: [::core::ffi::c_char; 96],
    pub radius: ::core::ffi::c_double,
    pub gravitational_parameter: ::core::ffi::c_double,
    pub rotational_speed: ::core::ffi::c_double,
    pub atmosphere_depth: ::core::ffi::c_double,
    pub surface_density: ::core::ffi::c_double,
    pub atmosphere_adiabatic_index: ::core::ffi::c_double,
    pub atmosphere_sample_count: size_t,
    pub atmosphere_altitude: [::core::ffi::c_double; 1024],
    pub atmosphere_pressure: [::core::ffi::c_double; 1024],
    pub atmosphere_density: [::core::ffi::c_double; 1024],
    pub north_axis: Vector3,
    pub prime_meridian_at_epoch: Vector3,
    pub epoch_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleState {
    pub ut: ::core::ffi::c_double,
    pub position: Vector3,
    pub velocity: Vector3,
    pub mass: ::core::ffi::c_double,
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
pub type ShuttleSimModelFile = ::core::ffi::c_uint;
pub const SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR: ShuttleSimModelFile = 4;
pub const SHUTTLE_SIM_MODEL_ATTITUDE: ShuttleSimModelFile = 3;
pub const SHUTTLE_SIM_MODEL_FORCE_BOOK: ShuttleSimModelFile = 2;
pub const SHUTTLE_SIM_MODEL_AERO: ShuttleSimModelFile = 1;
pub const SHUTTLE_SIM_MODEL_ATMOSPHERE: ShuttleSimModelFile = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub name: *const ::core::ffi::c_char,
    pub vehicle_model_id: *const ::core::ffi::c_char,
    pub files: [*const ::core::ffi::c_char; 5],
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const LANDER_ATMOSPHERE_SAMPLE_MAX: ::core::ffi::c_int = 1024 as ::core::ffi::c_int;
unsafe extern "C" fn set_error(
    mut out: *mut ::core::ffi::c_char,
    mut size: size_t,
    mut message: *const ::core::ffi::c_char,
) {
    if out.is_null() || size == 0 as size_t {
        return;
    }
    snprintf(
        out,
        size,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if !message.is_null() {
            message
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_load_planet(
    mut path: *const ::core::ffi::c_char,
    mut planet: *mut PlanetModel,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if path.is_null() || planet.is_null() {
        set_error(
            error,
            error_size,
            b"Simulator atmosphere path and destination are required\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut file: *mut FILE = fopen(
        path,
        b"r\0" as *const u8 as *const ::core::ffi::c_char,
    ) as *mut FILE;
    if file.is_null() {
        set_error(
            error,
            error_size,
            b"Could not open simulator atmosphere profile\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut model: PlanetModel = PlanetModel {
        name: [
            0 as ::core::ffi::c_int as ::core::ffi::c_char,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ],
        radius: 0.,
        gravitational_parameter: 0.,
        rotational_speed: 0.,
        atmosphere_depth: 0.,
        surface_density: 0.,
        atmosphere_adiabatic_index: 0.,
        atmosphere_sample_count: 0,
        atmosphere_altitude: [0.; 1024],
        atmosphere_pressure: [0.; 1024],
        atmosphere_density: [0.; 1024],
        north_axis: Vector3 { x: 0., y: 0., z: 0. },
        prime_meridian_at_epoch: Vector3 { x: 0., y: 0., z: 0. },
        epoch_ut: 0.,
    };
    snprintf(
        &raw mut model.name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        b"Kerbin\0" as *const u8 as *const ::core::ffi::c_char,
    );
    model.radius = 600000.0f64;
    model.gravitational_parameter = 3.5316e12f64;
    model.rotational_speed = 2.0f64 * LANDER_PI / 21549.425f64;
    model.atmosphere_adiabatic_index = 1.4f64;
    model.north_axis = Vector3 {
        x: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        z: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    model.prime_meridian_at_epoch = Vector3 {
        x: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        z: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    let mut line: [::core::ffi::c_char; 512] = [0; 512];
    let mut valid: bool = true_0 != 0;
    let mut found_top: bool = false_0 != 0;
    while !fgets(
            &raw mut line as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as ::core::ffi::c_int,
            file,
        )
        .is_null()
    {
        if line[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int == '#' as i32
            || !strstr(
                    &raw mut line as *mut ::core::ffi::c_char,
                    b"altitude_m\0" as *const u8 as *const ::core::ffi::c_char,
                )
                .is_null()
        {
            continue;
        }
        let mut h: ::core::ffi::c_double = 0.;
        let mut density: ::core::ffi::c_double = 0.;
        let mut pressure: ::core::ffi::c_double = 0.;
        let mut temperature: ::core::ffi::c_double = 0.;
        let mut sound: ::core::ffi::c_double = 0.;
        let mut trailing: ::core::ffi::c_char = 0;
        if sscanf(
            &raw mut line as *mut ::core::ffi::c_char,
            b"%lf,%lf,%lf,%lf,%lf %c\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut h,
            &raw mut density,
            &raw mut pressure,
            &raw mut temperature,
            &raw mut sound,
            &raw mut trailing,
        ) != 5 as ::core::ffi::c_int || h.is_finite() as i32 == 0 || h < 0.0f64
            || density.is_finite() as i32 == 0 || density < 0.0f64
            || pressure.is_finite() as i32 == 0 || pressure < 0.0f64
            || temperature.is_finite() as i32 == 0 || temperature < 0.0f64
            || sound.is_finite() as i32 == 0 || sound < 0.0f64
            || density > 0.0f64
                && (pressure <= 0.0f64 || temperature <= 0.0f64 || sound <= 0.0f64)
            || density == 0.0f64 && pressure != 0.0f64
            || model.atmosphere_sample_count == LANDER_ATMOSPHERE_SAMPLE_MAX as size_t
        {
            valid = false_0 != 0;
            break;
        } else {
            let mut i: size_t = model.atmosphere_sample_count;
            if i != 0
                && h <= model.atmosphere_altitude[i.wrapping_sub(1 as size_t) as usize]
                || found_top as ::core::ffi::c_int != 0
                    && (density > 0.0f64 || pressure > 0.0f64)
            {
                valid = false_0 != 0;
                break;
            } else {
                if i != 0
                    && model.atmosphere_density[i.wrapping_sub(1 as size_t) as usize]
                        > 0.0f64 && density == 0.0f64
                {
                    model.atmosphere_depth = h;
                    found_top = true_0 != 0;
                }
                model.atmosphere_altitude[i as usize] = h;
                model.atmosphere_density[i as usize] = density;
                model.atmosphere_pressure[i as usize] = pressure;
                model.atmosphere_sample_count = model
                    .atmosphere_sample_count
                    .wrapping_add(1);
            }
        }
    }
    valid = valid as ::core::ffi::c_int != 0 && ferror(file) == 0;
    fclose(file);
    if !valid || model.atmosphere_sample_count < 2 as size_t || !found_top {
        set_error(
            error,
            error_size,
            b"Invalid, incomplete, or oversized simulator atmosphere profile\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    model.surface_density = model.atmosphere_density[0 as ::core::ffi::c_int as usize];
    *planet = model;
    set_error(error, error_size, b"\0" as *const u8 as *const ::core::ffi::c_char);
    return true_0 != 0;
}
unsafe extern "C" fn object_value(
    mut doc: *const JsonDoc,
    mut parent: ::core::ffi::c_int,
    mut key: *const ::core::ffi::c_char,
) -> ::core::ffi::c_int {
    return if parent >= 0 as ::core::ffi::c_int {
        json_object_get(doc, parent, key)
    } else {
        -(1 as ::core::ffi::c_int)
    };
}
unsafe extern "C" fn number_value(
    mut doc: *const JsonDoc,
    mut parent: ::core::ffi::c_int,
    mut key: *const ::core::ffi::c_char,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut token: ::core::ffi::c_int = object_value(doc, parent, key);
    return if token >= 0 as ::core::ffi::c_int {
        json_number(doc, token, fallback)
    } else {
        fallback
    };
}
unsafe extern "C" fn bool_value(
    mut doc: *const JsonDoc,
    mut parent: ::core::ffi::c_int,
    mut key: *const ::core::ffi::c_char,
    mut fallback: bool,
) -> bool {
    let mut token: ::core::ffi::c_int = object_value(doc, parent, key);
    return if token >= 0 as ::core::ffi::c_int {
        json_boolean(doc, token, fallback) as ::core::ffi::c_int
    } else {
        fallback as ::core::ffi::c_int
    } != 0;
}
unsafe extern "C" fn object_present(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
) -> bool {
    return !doc.is_null() && index >= 0 as ::core::ffi::c_int && index < (*doc).count
        && (*(*doc).tokens.offset(index as isize)).type_0 as ::core::ffi::c_uint
            == JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint;
}
unsafe extern "C" fn required_number(
    mut doc: *const JsonDoc,
    mut parent: ::core::ffi::c_int,
    mut key: *const ::core::ffi::c_char,
) -> bool {
    let mut token: ::core::ffi::c_int = object_value(doc, parent, key);
    if token < 0 as ::core::ffi::c_int || token >= (*doc).count
        || (*(*doc).tokens.offset(token as isize)).type_0 as ::core::ffi::c_uint
            != JSON_PRIMITIVE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return false_0 != 0;
    }
    return json_number(doc, token, ::core::f32::NAN as ::core::ffi::c_double).is_finite()
        as i32 != 0;
}
unsafe extern "C" fn signed_degrees(
    mut angle: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if angle.is_finite() as i32 == 0 {
        return angle;
    }
    let mut wrapped: ::core::ffi::c_double = fmod(angle + 180.0f64, 360.0f64);
    if wrapped < 0.0f64 {
        wrapped += 360.0f64;
    }
    return wrapped - 180.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_prepare_guidance_telemetry(
    mut t: *mut Telemetry,
    mut cfg: *const LandingConfiguration,
    mut planet: *const PlanetModel,
) {
    if t.is_null() || cfg.is_null() || planet.is_null() {
        return;
    }
    let mut aero_finite: bool = (*t).dynamic_pressure.is_finite() as i32 != 0
        && (*t).dynamic_pressure >= 0.0f64 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).drag_force.is_finite() as i32 != 0;
    (*t).physics_confidence = if (*t).physics_sample_valid as ::core::ffi::c_int != 0 {
        1.0f64
    } else {
        0.0f64
    };
    (*t).aerodynamic_confidence = if aero_finite as ::core::ffi::c_int != 0 {
        1.0f64
    } else {
        0.0f64
    };
    (*t).estimated_lift_to_drag = if (*t).drag_force > 0.0f64 {
        (*t).lift_force / (*t).drag_force
    } else {
        (*cfg).vehicle.estimated_lift_to_drag
    };
    (*t).estimated_ballistic_coefficient = (*cfg)
        .vehicle
        .estimated_ballistic_coefficient;
    (*t).physics_authority[0 as ::core::ffi::c_int as usize] = if (*t)
        .attitude_response
        .pitch_valid as ::core::ffi::c_int != 0
    {
        (*t).attitude_response.maximum_pitch_accel_deg_s2
    } else {
        0.0f64
    };
    (*t).physics_authority[1 as ::core::ffi::c_int as usize] = if (*t)
        .attitude_response
        .roll_valid as ::core::ffi::c_int != 0
    {
        (*t).attitude_response.maximum_roll_accel_deg_s2
    } else {
        0.0f64
    };
    (*t).physics_authority[2 as ::core::ffi::c_int as usize] = 0.0f64;
    (*t).physics_authority_confidence[0 as ::core::ffi::c_int as usize] = if (*t)
        .attitude_response
        .pitch_valid as ::core::ffi::c_int != 0
    {
        1.0f64
    } else {
        0.0f64
    };
    (*t).physics_authority_confidence[1 as ::core::ffi::c_int as usize] = if (*t)
        .attitude_response
        .roll_valid as ::core::ffi::c_int != 0
    {
        1.0f64
    } else {
        0.0f64
    };
    (*t).physics_authority_confidence[2 as ::core::ffi::c_int as usize] = 0.0f64;
    let mut here: GeoPoint = GeoPoint {
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        altitude: (*t).mean_altitude,
    };
    let mut site: GeoPoint = GeoPoint {
        latitude: (*cfg).site.latitude,
        longitude: (*cfg).site.longitude,
        altitude: (*cfg).site.altitude,
    };
    (*t).range_to_site = great_circle_distance(here, site, (*planet).radius);
    (*t).bearing_to_site = initial_bearing(here, site);
    (*t).heading_error = signed_degrees((*cfg).site.runway_heading - (*t).heading);
    (*t).course_to_site_error = signed_degrees(
        (*t).bearing_to_site - (*t).ground_track_heading,
    );
    snprintf(
        &raw mut (*t).vessel_name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"STS-N Sim\0" as *const u8 as *const ::core::ffi::c_char,
    );
}
unsafe extern "C" fn attitude_axis_response_valid(
    mut natural_frequency: ::core::ffi::c_double,
    mut damping_ratio: ::core::ffi::c_double,
    mut maximum_rate: ::core::ffi::c_double,
    mut maximum_acceleration: ::core::ffi::c_double,
) -> bool {
    return natural_frequency.is_finite() as i32 != 0 && natural_frequency > 0.0f64
        && damping_ratio.is_finite() as i32 != 0 && damping_ratio >= 0.0f64
        && maximum_rate.is_finite() as i32 != 0 && maximum_rate > 0.0f64
        && maximum_acceleration.is_finite() as i32 != 0 && maximum_acceleration > 0.0f64;
}
unsafe extern "C" fn vector_dot(
    mut a: Vector3,
    mut b: Vector3,
) -> ::core::ffi::c_double {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
unsafe extern "C" fn vector_magnitude(mut v: Vector3) -> ::core::ffi::c_double {
    return sqrt(vector_dot(v, v));
}
unsafe extern "C" fn vector_cross(mut a: Vector3, mut b: Vector3) -> Vector3 {
    return Vector3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    };
}
unsafe extern "C" fn fill_orbit_fields(
    mut planet: *const PlanetModel,
    mut state: *const VehicleState,
    mut telemetry: *mut Telemetry,
) {
    if planet.is_null() || state.is_null() || telemetry.is_null() {
        return;
    }
    let mut r: Vector3 = (*state).position;
    let mut v: Vector3 = (*state).velocity;
    let mut radius: ::core::ffi::c_double = vector_magnitude(r);
    let mut speed_squared: ::core::ffi::c_double = vector_dot(v, v);
    let mut mu: ::core::ffi::c_double = (*planet).gravitational_parameter;
    if radius.is_finite() as i32 == 0 || radius <= 0.0f64
        || speed_squared.is_finite() as i32 == 0 || speed_squared < 0.0f64
        || mu.is_finite() as i32 == 0 || mu <= 0.0f64
    {
        return;
    }
    let mut kinetic_specific_energy: ::core::ffi::c_double = 0.5f64 * speed_squared;
    let mut potential_scale: ::core::ffi::c_double = mu / radius;
    let mut energy: ::core::ffi::c_double = kinetic_specific_energy - potential_scale;
    let mut energy_scale: ::core::ffi::c_double = kinetic_specific_energy
        + potential_scale;
    if energy.is_finite() as i32 == 0
        || fabs(energy) <= DBL_EPSILON * fmax(energy_scale, DBL_MIN)
    {
        return;
    }
    let mut semimajor: ::core::ffi::c_double = -mu / (2.0f64 * energy);
    let mut h: Vector3 = vector_cross(r, v);
    let mut h_squared: ::core::ffi::c_double = vector_dot(h, h);
    let mut eccentricity_squared: ::core::ffi::c_double = fmax(
        0.0f64,
        1.0f64 + 2.0f64 * energy * h_squared / (mu * mu),
    );
    let mut eccentricity: ::core::ffi::c_double = sqrt(eccentricity_squared);
    (*telemetry).periapsis_altitude = semimajor * (1.0f64 - eccentricity)
        - (*planet).radius;
    (*telemetry).apoapsis_altitude = semimajor * (1.0f64 + eccentricity)
        - (*planet).radius;
    if semimajor > 0.0f64 {
        (*telemetry).orbit_period = 2.0f64 * LANDER_PI
            * sqrt(semimajor * semimajor * semimajor / mu);
    }
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_decode_telemetry(
    mut packet: *const ::core::ffi::c_char,
    mut planet: *const PlanetModel,
    mut previous: *const Telemetry,
    mut t: *mut Telemetry,
    mut state: *mut VehicleState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if packet.is_null() || planet.is_null() || t.is_null() || state.is_null() {
        set_error(
            error,
            error_size,
            b"Invalid ShuttleSim telemetry decode arguments\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut tokens: [JsonToken; 512] = [JsonToken {
        type_0: JSON_UNDEFINED,
        start: 0,
        end: 0,
        size: 0,
        parent: 0,
    }; 512];
    let mut doc: JsonDoc = JsonDoc {
        text: ::core::ptr::null::<::core::ffi::c_char>(),
        tokens: ::core::ptr::null_mut::<JsonToken>(),
        count: 0,
    };
    if json_parse(
        packet,
        &raw mut tokens as *mut JsonToken,
        512 as ::core::ffi::c_int,
        &raw mut doc,
    ) < 1 as ::core::ffi::c_int
        || (*doc.tokens.offset(0 as ::core::ffi::c_int as isize)).type_0
            as ::core::ffi::c_uint
            != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        set_error(
            error,
            error_size,
            b"Invalid ShuttleSim telemetry JSON\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut source: [::core::ffi::c_char; 32] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_char,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    let mut source_token: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"source\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if source_token >= 0 as ::core::ffi::c_int {
        json_string(
            &raw mut doc,
            source_token,
            &raw mut source as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 32]>() as size_t,
        );
    }
    if strcmp(
        &raw mut source as *mut ::core::ffi::c_char,
        b"sim\0" as *const u8 as *const ::core::ffi::c_char,
    ) != 0 as ::core::ffi::c_int
    {
        if !error.is_null() && error_size != 0 {
            snprintf(
                error,
                error_size,
                b"Unexpected simulator telemetry source '%s'\0" as *const u8
                    as *const ::core::ffi::c_char,
                &raw mut source as *mut ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    telemetry_init(t);
    memset(
        state as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<VehicleState>() as size_t,
    );
    (*t).calibrated_best_glide_angle_of_attack = ::core::f32::NAN
        as ::core::ffi::c_double;
    (*t).calibrated_stall_speed = ::core::f32::NAN as ::core::ffi::c_double;
    let mut position: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"position\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut velocity: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"velocity\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut attitude: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"attitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut atmosphere: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"atmosphere\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut aero: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"aero\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut vehicle: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"vehicle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut runway: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"runway\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut ground: ::core::ffi::c_int = object_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"ground\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut required_objects: bool = object_present(&raw mut doc, position)
        as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, velocity) as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, attitude) as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, atmosphere) as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, aero) as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, vehicle) as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, runway) as ::core::ffi::c_int != 0
        && object_present(&raw mut doc, ground) as ::core::ffi::c_int != 0;
    let mut required_numeric: bool = required_number(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"ut\0" as *const u8 as *const ::core::ffi::c_char,
    ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            position,
            b"x\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            position,
            b"y\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            position,
            b"z\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            position,
            b"lat_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            position,
            b"lon_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            position,
            b"altitude_m\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"x_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"y_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"z_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"surface_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"air_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"horizontal_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"vertical_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            velocity,
            b"flight_path_angle_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"aoa_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"bank_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"heading_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"q_w\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"q_x\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"q_y\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"q_z\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            atmosphere,
            b"density_kg_m3\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            atmosphere,
            b"pressure_pa\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            atmosphere,
            b"speed_of_sound_mps\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            aero,
            b"mach\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            aero,
            b"q_pa\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            aero,
            b"lift_n\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            aero,
            b"drag_n\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            vehicle,
            b"mass_kg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            runway,
            b"along_m\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            runway,
            b"cross_m\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0;
    (*t).ut = number_value(
        &raw mut doc,
        0 as ::core::ffi::c_int,
        b"ut\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*state).ut = (*t).ut;
    (*t).latitude = number_value(
        &raw mut doc,
        position,
        b"lat_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).longitude = number_value(
        &raw mut doc,
        position,
        b"lon_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).mean_altitude = number_value(
        &raw mut doc,
        position,
        b"altitude_m\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).radar_altitude = number_value(
        &raw mut doc,
        runway,
        b"vertical_m\0" as *const u8 as *const ::core::ffi::c_char,
        (*t).mean_altitude,
    );
    (*state).position = Vector3 {
        x: number_value(
            &raw mut doc,
            position,
            b"x\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        ),
        y: number_value(
            &raw mut doc,
            position,
            b"y\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        ),
        z: number_value(
            &raw mut doc,
            position,
            b"z\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        ),
    };
    (*state).velocity = Vector3 {
        x: number_value(
            &raw mut doc,
            velocity,
            b"x_mps\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        ),
        y: number_value(
            &raw mut doc,
            velocity,
            b"y_mps\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        ),
        z: number_value(
            &raw mut doc,
            velocity,
            b"z_mps\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        ),
    };
    (*t).surface_speed = number_value(
        &raw mut doc,
        velocity,
        b"surface_mps\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).true_air_speed = number_value(
        &raw mut doc,
        velocity,
        b"air_mps\0" as *const u8 as *const ::core::ffi::c_char,
        (*t).surface_speed,
    );
    (*t).horizontal_speed = number_value(
        &raw mut doc,
        velocity,
        b"horizontal_mps\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).vertical_speed = number_value(
        &raw mut doc,
        velocity,
        b"vertical_mps\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).flight_path_angle = number_value(
        &raw mut doc,
        velocity,
        b"flight_path_angle_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).angle_of_attack = number_value(
        &raw mut doc,
        attitude,
        b"aoa_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).roll = number_value(
        &raw mut doc,
        attitude,
        b"bank_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).heading = number_value(
        &raw mut doc,
        attitude,
        b"heading_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).ground_track_heading = (*t).heading;
    (*t).pitch = (*t).flight_path_angle + (*t).angle_of_attack;
    (*t).sideslip = number_value(
        &raw mut doc,
        attitude,
        b"sideslip_deg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    let mut body_attitude_available: bool = bool_value(
        &raw mut doc,
        attitude,
        b"body_attitude_available\0" as *const u8 as *const ::core::ffi::c_char,
        false_0 != 0,
    ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_pitch_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_roll_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_heading_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0;
    if body_attitude_available {
        (*t).pitch = number_value(
            &raw mut doc,
            attitude,
            b"body_pitch_deg\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).pitch,
        );
        (*t).roll = number_value(
            &raw mut doc,
            attitude,
            b"body_roll_deg\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).roll,
        );
        (*t).heading = number_value(
            &raw mut doc,
            attitude,
            b"body_heading_deg\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).heading,
        );
    } else if (*t).sideslip.is_finite() as i32 != 0 {
        (*t).heading = fmod((*t).heading - (*t).sideslip + 360.0f64, 360.0f64);
    }
    let mut published_aoa_rate: ::core::ffi::c_double = number_value(
        &raw mut doc,
        attitude,
        b"aoa_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    let mut published_bank_rate: ::core::ffi::c_double = number_value(
        &raw mut doc,
        attitude,
        b"bank_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    let mut has_published_aoa_rate: bool = published_aoa_rate.is_finite() as i32 != 0;
    let mut has_published_bank_rate: bool = published_bank_rate.is_finite() as i32 != 0;
    if has_published_aoa_rate {
        (*t).angle_of_attack_rate = published_aoa_rate;
        (*t).has_angle_of_attack_rate = true_0 != 0;
    }
    if has_published_bank_rate {
        (*t).roll_rate = published_bank_rate;
    }
    let mut pitch_input: ::core::ffi::c_double = number_value(
        &raw mut doc,
        attitude,
        b"input_pitch\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    let mut roll_input: ::core::ffi::c_double = number_value(
        &raw mut doc,
        attitude,
        b"input_roll\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    let mut yaw_input: ::core::ffi::c_double = number_value(
        &raw mut doc,
        attitude,
        b"input_yaw\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    if bool_value(
        &raw mut doc,
        attitude,
        b"direct_control\0" as *const u8 as *const ::core::ffi::c_char,
        false_0 != 0,
    ) as ::core::ffi::c_int != 0 && pitch_input.is_finite() as i32 != 0
        && fabs(pitch_input) <= 1.0f64 && roll_input.is_finite() as i32 != 0
        && fabs(roll_input) <= 1.0f64 && yaw_input.is_finite() as i32 != 0
        && fabs(yaw_input) <= 1.0f64
    {
        (*t).has_controls = true_0 != 0;
        (*t).control_pitch = pitch_input;
        (*t).control_roll = roll_input;
        (*t).control_yaw = yaw_input;
    }
    let mut response: *mut AttitudeResponseModel = &raw mut (*t).attitude_response;
    (*response).pitch_natural_frequency_s_inv = number_value(
        &raw mut doc,
        attitude,
        b"pitch_wn_s_inv\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).pitch_damping_ratio = number_value(
        &raw mut doc,
        attitude,
        b"pitch_zeta\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).roll_natural_frequency_s_inv = number_value(
        &raw mut doc,
        attitude,
        b"roll_wn_s_inv\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).roll_damping_ratio = number_value(
        &raw mut doc,
        attitude,
        b"roll_zeta\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).maximum_pitch_rate_deg_s = number_value(
        &raw mut doc,
        attitude,
        b"max_pitch_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).maximum_roll_rate_deg_s = number_value(
        &raw mut doc,
        attitude,
        b"max_roll_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).maximum_pitch_accel_deg_s2 = number_value(
        &raw mut doc,
        attitude,
        b"max_pitch_accel_deg_s2\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).maximum_roll_accel_deg_s2 = number_value(
        &raw mut doc,
        attitude,
        b"max_roll_accel_deg_s2\0" as *const u8 as *const ::core::ffi::c_char,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    (*response).pitch_valid = attitude_axis_response_valid(
        (*response).pitch_natural_frequency_s_inv,
        (*response).pitch_damping_ratio,
        (*response).maximum_pitch_rate_deg_s,
        (*response).maximum_pitch_accel_deg_s2,
    );
    (*response).roll_valid = attitude_axis_response_valid(
        (*response).roll_natural_frequency_s_inv,
        (*response).roll_damping_ratio,
        (*response).maximum_roll_rate_deg_s,
        (*response).maximum_roll_accel_deg_s2,
    );
    let mut publish_servo: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_SIM_PUBLISH_SERVO\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if publish_servo.is_null()
        || strcmp(publish_servo, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            != 0 as ::core::ffi::c_int
    {
        (*response).pitch_valid = false_0 != 0;
        (*response).roll_valid = false_0 != 0;
    }
    (*t).has_attitude_quaternion = true_0 != 0;
    (*t).attitude_quaternion[0 as ::core::ffi::c_int as usize] = number_value(
        &raw mut doc,
        attitude,
        b"q_x\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).attitude_quaternion[1 as ::core::ffi::c_int as usize] = number_value(
        &raw mut doc,
        attitude,
        b"q_y\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).attitude_quaternion[2 as ::core::ffi::c_int as usize] = number_value(
        &raw mut doc,
        attitude,
        b"q_z\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).attitude_quaternion[3 as ::core::ffi::c_int as usize] = number_value(
        &raw mut doc,
        attitude,
        b"q_w\0" as *const u8 as *const ::core::ffi::c_char,
        1.0f64,
    );
    snprintf(
        &raw mut (*t).attitude_reference_frame as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
        b"body-non-rotating-canonical\0" as *const u8 as *const ::core::ffi::c_char,
    );
    (*t).atmospheric_density = number_value(
        &raw mut doc,
        atmosphere,
        b"density_kg_m3\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).static_pressure = number_value(
        &raw mut doc,
        atmosphere,
        b"pressure_pa\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).speed_of_sound = number_value(
        &raw mut doc,
        atmosphere,
        b"speed_of_sound_mps\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).mach = number_value(
        &raw mut doc,
        aero,
        b"mach\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).dynamic_pressure = number_value(
        &raw mut doc,
        aero,
        b"q_pa\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).lift_force = number_value(
        &raw mut doc,
        aero,
        b"lift_n\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).drag_force = number_value(
        &raw mut doc,
        aero,
        b"drag_n\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).mass = number_value(
        &raw mut doc,
        vehicle,
        b"mass_kg\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*state).mass = (*t).mass;
    (*t).available_thrust = number_value(
        &raw mut doc,
        vehicle,
        b"available_thrust_n\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).current_thrust = number_value(
        &raw mut doc,
        vehicle,
        b"current_thrust_n\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).throttle = number_value(
        &raw mut doc,
        vehicle,
        b"throttle\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).gear = bool_value(
        &raw mut doc,
        ground,
        b"gear_down\0" as *const u8 as *const ::core::ffi::c_char,
        false_0 != 0,
    );
    (*t).brakes = bool_value(
        &raw mut doc,
        ground,
        b"brakes\0" as *const u8 as *const ::core::ffi::c_char,
        false_0 != 0,
    );
    (*t).has_airbrakes = true_0 != 0;
    (*t).airbrakes = false_0 != 0;
    let mut on_ground: bool = bool_value(
        &raw mut doc,
        ground,
        b"on_ground\0" as *const u8 as *const ::core::ffi::c_char,
        false_0 != 0,
    );
    snprintf(
        &raw mut (*t).vessel_situation as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if on_ground as ::core::ffi::c_int != 0 {
            b"landed\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"flying\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    (*t).has_main_gear_grounded = true_0 != 0;
    (*t).main_gear_grounded = bool_value(
        &raw mut doc,
        ground,
        b"main_contact\0" as *const u8 as *const ::core::ffi::c_char,
        on_ground,
    );
    (*t).has_nose_gear_grounded = true_0 != 0;
    (*t).nose_gear_grounded = bool_value(
        &raw mut doc,
        ground,
        b"nose_contact\0" as *const u8 as *const ::core::ffi::c_char,
        on_ground as ::core::ffi::c_int != 0 && (*t).pitch <= 0.5f64,
    );
    (*t).runway_along_track = number_value(
        &raw mut doc,
        runway,
        b"along_m\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).runway_cross_track = number_value(
        &raw mut doc,
        runway,
        b"cross_m\0" as *const u8 as *const ::core::ffi::c_char,
        0.0f64,
    );
    (*t).has_center_of_mass = true_0 != 0;
    (*t).center_of_mass = (*state).position;
    let mut state_finite: bool = (*state).position.x.is_finite() as i32 != 0
        && (*state).position.y.is_finite() as i32 != 0
        && (*state).position.z.is_finite() as i32 != 0
        && (*state).velocity.x.is_finite() as i32 != 0
        && (*state).velocity.y.is_finite() as i32 != 0
        && (*state).velocity.z.is_finite() as i32 != 0
        && (*state).mass.is_finite() as i32 != 0 && (*state).mass > 0.0f64;
    let mut telemetry_finite: bool = (*t).ut.is_finite() as i32 != 0
        && (*t).latitude.is_finite() as i32 != 0
        && (*t).longitude.is_finite() as i32 != 0
        && (*t).mean_altitude.is_finite() as i32 != 0
        && (*t).surface_speed.is_finite() as i32 != 0
        && (*t).true_air_speed.is_finite() as i32 != 0
        && (*t).horizontal_speed.is_finite() as i32 != 0
        && (*t).vertical_speed.is_finite() as i32 != 0
        && (*t).flight_path_angle.is_finite() as i32 != 0
        && (*t).angle_of_attack.is_finite() as i32 != 0
        && (*t).roll.is_finite() as i32 != 0 && (*t).heading.is_finite() as i32 != 0
        && (*t).atmospheric_density.is_finite() as i32 != 0
        && (*t).atmospheric_density >= 0.0f64
        && (*t).static_pressure.is_finite() as i32 != 0 && (*t).static_pressure >= 0.0f64
        && (*t).speed_of_sound.is_finite() as i32 != 0 && (*t).speed_of_sound >= 0.0f64
        && (*t).mach.is_finite() as i32 != 0 && (*t).mach >= 0.0f64
        && (*t).dynamic_pressure.is_finite() as i32 != 0
        && (*t).dynamic_pressure >= 0.0f64 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).drag_force.is_finite() as i32 != 0 && (*t).mass.is_finite() as i32 != 0
        && (*t).mass > 0.0f64 && (*t).runway_along_track.is_finite() as i32 != 0
        && (*t).runway_cross_track.is_finite() as i32 != 0;
    let mut position_domain: bool = (*t).latitude >= -90.0f64 && (*t).latitude <= 90.0f64
        && (*t).longitude >= -360.0f64 && (*t).longitude <= 360.0f64;
    let mut quaternion_finite: bool = (*t)
        .attitude_quaternion[0 as ::core::ffi::c_int as usize]
        .is_finite() as i32 != 0
        && (*t).attitude_quaternion[1 as ::core::ffi::c_int as usize].is_finite() as i32
            != 0
        && (*t).attitude_quaternion[2 as ::core::ffi::c_int as usize].is_finite() as i32
            != 0
        && (*t).attitude_quaternion[3 as ::core::ffi::c_int as usize].is_finite() as i32
            != 0;
    (*t).physics_sample_valid = required_objects as ::core::ffi::c_int != 0
        && required_numeric as ::core::ffi::c_int != 0
        && state_finite as ::core::ffi::c_int != 0
        && telemetry_finite as ::core::ffi::c_int != 0
        && position_domain as ::core::ffi::c_int != 0
        && quaternion_finite as ::core::ffi::c_int != 0;
    (*t).physics_confidence = if (*t).physics_sample_valid as ::core::ffi::c_int != 0 {
        1.0f64
    } else {
        0.0f64
    };
    (*t).aerodynamic_confidence = if (*t).dynamic_pressure.is_finite() as i32 != 0
        && (*t).dynamic_pressure >= 0.0f64 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).drag_force.is_finite() as i32 != 0
    {
        1.0f64
    } else {
        0.0f64
    };
    (*t).estimated_lift_to_drag = if (*t).drag_force > 0.0f64 {
        (*t).lift_force / (*t).drag_force
    } else {
        0.0f64
    };
    (*t).g_force = if (*t).mass > 0.0f64 {
        fabs((*t).lift_force) / ((*t).mass * 9.80665f64)
    } else {
        0.0f64
    };
    (*t).stall_fraction = 0.0f64;
    (*t).stall_fraction_is_measured = false_0 != 0;
    if !previous.is_null() {
        let mut dt: ::core::ffi::c_double = (*t).ut - (*previous).ut;
        if dt.is_finite() as i32 != 0 && dt > 0.0f64 {
            (*t).pitch_rate = ((*t).pitch - (*previous).pitch) / dt;
            if !has_published_bank_rate
                || body_attitude_available as ::core::ffi::c_int != 0
            {
                (*t).roll_rate = signed_degrees((*t).roll - (*previous).roll) / dt;
            }
            (*t).heading_rate = signed_degrees((*t).heading - (*previous).heading) / dt;
            if !has_published_aoa_rate {
                (*t).angle_of_attack_rate = signed_degrees(
                    (*t).angle_of_attack - (*previous).angle_of_attack,
                ) / dt;
                (*t).has_angle_of_attack_rate = true_0 != 0;
            }
            (*t).course_rate = signed_degrees(
                (*t).ground_track_heading - (*previous).ground_track_heading,
            ) / dt;
            (*t).has_course_rate = true_0 != 0;
        }
    }
    (*t).has_body_pitch_rate = false_0 != 0;
    (*t).has_body_roll_rate = false_0 != 0;
    (*t).has_body_yaw_rate = false_0 != 0;
    if body_attitude_available as ::core::ffi::c_int != 0
        && bool_value(
            &raw mut doc,
            attitude,
            b"body_rates_available\0" as *const u8 as *const ::core::ffi::c_char,
            false_0 != 0,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_pitch_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_roll_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_yaw_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_pitch_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_roll_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        && required_number(
            &raw mut doc,
            attitude,
            b"body_heading_deg\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
    {
        (*t).body_pitch_rate = number_value(
            &raw mut doc,
            attitude,
            b"body_pitch_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        );
        (*t).body_roll_rate = number_value(
            &raw mut doc,
            attitude,
            b"body_roll_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        );
        (*t).body_yaw_rate = number_value(
            &raw mut doc,
            attitude,
            b"body_yaw_rate_deg_s\0" as *const u8 as *const ::core::ffi::c_char,
            0.0f64,
        );
        (*t).has_body_yaw_rate = true_0 != 0;
        (*t).has_body_roll_rate = (*t).has_body_yaw_rate;
        (*t).has_body_pitch_rate = (*t).has_body_roll_rate;
    }
    fill_orbit_fields(planet, state, t);
    if !error.is_null() && error_size != 0 {
        *error.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
            as ::core::ffi::c_char;
    }
    return true_0 != 0;
}
static mut model_profiles: [C2RustUnnamed; 12] = [
    C2RustUnnamed {
        name: b"identified\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS-N\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_aero_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_certified_prior_identified.csv\0"
                as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-provisional\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_provisional.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261009\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_live20261009.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261010\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_live20261010.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261011\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_live20261011.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261015\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_cells20261015.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261015-live-only\0" as *const u8
            as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_cells20261015_live_only.csv\0"
                as *const u8 as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261029\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_cells20261029.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_identified.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261028\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_cells20261028.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_attitude_live20261028.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"sts-live-20261015-att28\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_aero_cells20261015_live_only.csv\0"
                as *const u8 as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/sts_attitude_live20261028.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"fitted-legacy\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS-N\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/data/fitted/stsn_aero_ksp_robust.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/data/fitted/stsn_force_book.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/data/fitted/stsn_attitude_ksp.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/data/fitted/stsn_force_book.csv\0" as *const u8
                as *const ::core::ffi::c_char,
        ],
    },
    C2RustUnnamed {
        name: b"reference-b\0" as *const u8 as *const ::core::ffi::c_char,
        vehicle_model_id: b"STS-N\0" as *const u8 as *const ::core::ffi::c_char,
        files: [
            b"ShuttleSim/reference-model/kerbin_atmosphere_reference.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_aero_reference.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_force_book_reference.csv\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_attitude_reference.ini\0" as *const u8
                as *const ::core::ffi::c_char,
            b"ShuttleSim/reference-model/stsn_force_book_reference.csv\0" as *const u8
                as *const ::core::ffi::c_char,
        ],
    },
];
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_model_profile() -> *const ::core::ffi::c_char {
    let mut configured: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_MODEL_PROFILE\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if configured.is_null() || *configured.offset(0 as ::core::ffi::c_int as isize) == 0
    {
        return model_profiles[0 as ::core::ffi::c_int as usize].name;
    }
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[C2RustUnnamed; 12]>() as usize)
            .wrapping_div(::core::mem::size_of::<C2RustUnnamed>() as usize)
    {
        if strcmp(configured, model_profiles[i as usize].name) == 0 as ::core::ffi::c_int
        {
            return model_profiles[i as usize].name;
        }
        i = i.wrapping_add(1);
    }
    return ::core::ptr::null::<::core::ffi::c_char>();
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_model_vehicle_id() -> *const ::core::ffi::c_char {
    let mut profile: *const ::core::ffi::c_char = shuttle_sim_model_profile();
    if profile.is_null() {
        return ::core::ptr::null::<::core::ffi::c_char>();
    }
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[C2RustUnnamed; 12]>() as usize)
            .wrapping_div(::core::mem::size_of::<C2RustUnnamed>() as usize)
    {
        if strcmp(profile, model_profiles[i as usize].name) == 0 as ::core::ffi::c_int {
            return model_profiles[i as usize].vehicle_model_id;
        }
        i = i.wrapping_add(1);
    }
    return ::core::ptr::null::<::core::ffi::c_char>();
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_model_matches_vehicle(
    mut model_id: *const ::core::ffi::c_char,
    mut reason: *mut ::core::ffi::c_char,
    mut size: size_t,
) -> bool {
    if !reason.is_null() && size != 0 {
        *reason.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
            as ::core::ffi::c_char;
    }
    let mut profile: *const ::core::ffi::c_char = shuttle_sim_model_profile();
    let mut vehicle: *const ::core::ffi::c_char = shuttle_sim_model_vehicle_id();
    if !vehicle.is_null() && !model_id.is_null()
        && strcmp(vehicle, model_id) == 0 as ::core::ffi::c_int
    {
        return true_0 != 0;
    }
    if !reason.is_null() && size != 0 {
        snprintf(
            reason,
            size,
            b"Aerodynamic profile '%s' describes %s, not %s; cross-vehicle model reuse is blocked.\0"
                as *const u8 as *const ::core::ffi::c_char,
            if !profile.is_null() {
                profile
            } else {
                b"unknown\0" as *const u8 as *const ::core::ffi::c_char
            },
            if !vehicle.is_null() {
                vehicle
            } else {
                b"unknown\0" as *const u8 as *const ::core::ffi::c_char
            },
            if !model_id.is_null() && *model_id as ::core::ffi::c_int != 0 {
                model_id
            } else {
                b"an unspecified vehicle\0" as *const u8 as *const ::core::ffi::c_char
            },
        );
    }
    return false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_resolve_vehicle_identity(
    mut live_name: *const ::core::ffi::c_char,
    mut configured_model_id: *const ::core::ffi::c_char,
) -> *const ::core::ffi::c_char {
    if !live_name.is_null()
        && *live_name.offset(0 as ::core::ffi::c_int as isize) as ::core::ffi::c_int != 0
        && strcmp(
            live_name,
            b"Active Vessel\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0 as ::core::ffi::c_int
    {
        return live_name;
    }
    return if !configured_model_id.is_null() {
        configured_model_id
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn shuttle_sim_model_path(
    mut root: *const ::core::ffi::c_char,
    mut kind: ShuttleSimModelFile,
    mut buffer: *mut ::core::ffi::c_char,
    mut buffer_size: size_t,
) -> *const ::core::ffi::c_char {
    if buffer.is_null() || buffer_size == 0 as size_t
        || (kind as ::core::ffi::c_int) < 0 as ::core::ffi::c_int
        || kind as ::core::ffi::c_int
            > SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR as ::core::ffi::c_int
    {
        return ::core::ptr::null::<::core::ffi::c_char>();
    }
    let mut profile: *const ::core::ffi::c_char = shuttle_sim_model_profile();
    if profile.is_null() {
        return ::core::ptr::null::<::core::ffi::c_char>();
    }
    let mut file: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[C2RustUnnamed; 12]>() as usize)
            .wrapping_div(::core::mem::size_of::<C2RustUnnamed>() as usize)
    {
        if strcmp(profile, model_profiles[i as usize].name) == 0 as ::core::ffi::c_int {
            file = model_profiles[i as usize].files[kind as usize];
        }
        i = i.wrapping_add(1);
    }
    if file.is_null() {
        return ::core::ptr::null::<::core::ffi::c_char>();
    }
    let mut written: ::core::ffi::c_int = if strcmp(
        file,
        b"none\0" as *const u8 as *const ::core::ffi::c_char,
    ) != 0 as ::core::ffi::c_int && !root.is_null()
        && *root.offset(0 as ::core::ffi::c_int as isize) as ::core::ffi::c_int != 0
    {
        snprintf(
            buffer,
            buffer_size,
            b"%s/%s\0" as *const u8 as *const ::core::ffi::c_char,
            root,
            file,
        )
    } else {
        snprintf(
            buffer,
            buffer_size,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            file,
        )
    };
    if written < 0 as ::core::ffi::c_int || written as size_t >= buffer_size {
        return ::core::ptr::null::<::core::ffi::c_char>();
    }
    return buffer;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
