use crate::sys::*;
extern "C" {
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn expm1(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn great_circle_distance(
        a: GeoPoint,
        b: GeoPoint,
        radius: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn trajectory_init(t: *mut Trajectory);
    fn trajectory_clear(t: *mut Trajectory);
    fn trajectory_copy(dst: *mut Trajectory, src: *const Trajectory) -> bool;
    fn planet_atmospheric_density(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_speed_of_sound(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn calibration_observation_envelope(
        telemetry: *const Telemetry,
        vehicle: *const VehicleProfile,
        settings: *const CalibrationSettings,
        dt: ::core::ffi::c_double,
    ) -> CalibrationObservationEnvelope;
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
pub type ControlProfile = ::core::ffi::c_uint;
pub const PROFILE_RECOVERY: ControlProfile = 6;
pub const PROFILE_ROLLOUT: ControlProfile = 5;
pub const PROFILE_FLARE: ControlProfile = 4;
pub const PROFILE_APPROACH: ControlProfile = 3;
pub const PROFILE_TAEM: ControlProfile = 2;
pub const PROFILE_ENTRY: ControlProfile = 1;
pub const PROFILE_ORBITAL: ControlProfile = 0;
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
pub struct VesselPhysicsModel {
    pub state: VehicleState,
    pub center_of_mass: Vector3,
    pub lift_vector: Vector3,
    pub drag_vector: Vector3,
    pub center_of_mass_root: Vector3,
    pub has_center_of_mass_root: bool,
    pub has_state: bool,
    pub has_center_of_mass: bool,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub thrust: ::core::ffi::c_double,
    pub available_thrust: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub mass_flow: ::core::ffi::c_double,
    pub propellant_per_impulse: ::core::ffi::c_double,
    pub authority: [::core::ffi::c_double; 3],
    pub authority_confidence: [::core::ffi::c_double; 3],
    pub previous_rate: [::core::ffi::c_double; 3],
    pub previous_control: [::core::ffi::c_double; 3],
    pub torque: [::core::ffi::c_double; 3],
    pub inertia: [::core::ffi::c_double; 3],
    pub previous_rate_valid: [bool; 3],
    pub authority_q: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub model_residual: ::core::ffi::c_double,
    pub model_residual_confidence: ::core::ffi::c_double,
    pub certified_uncertainty: ::core::ffi::c_double,
    pub certified_confidence: ::core::ffi::c_double,
    pub last_residual_ut: ::core::ffi::c_double,
    pub has_last_residual_ut: bool,
    pub force_residual_per_q: Vector3,
    pub force_residual_variance_per_q: Vector3,
    pub force_residual_confidence: ::core::ffi::c_double,
    pub dry_mass: ::core::ffi::c_double,
    pub count: ::core::ffi::c_uint,
    pub next: ::core::ffi::c_uint,
    pub accepted: ::core::ffi::c_uint,
    pub rejected: ::core::ffi::c_uint,
    pub live_observations: ::core::ffi::c_uint,
    pub samples: [VesselAeroSample; 192],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VesselAeroSample {
    pub q: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub beta: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub force_per_q: Vector3,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub observations: ::core::ffi::c_uint,
    pub trust: ::core::ffi::c_double,
    pub last_observation_ut: ::core::ffi::c_double,
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
pub struct AerodynamicModel {
    pub lift_to_drag: ::core::ffi::c_double,
    pub ballistic_coefficient: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AerodynamicEnvelope {
    pub regimes: [AerodynamicModel; 4],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TrajectoryCalibrationModel {
    pub physics: *const VesselPhysicsModel,
    pub density_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
    pub lift_scale: ::core::ffi::c_double,
    pub bank_effectiveness: ::core::ffi::c_double,
    pub speed_of_sound: ::core::ffi::c_double,
    pub speed_of_sound_scale: ::core::ffi::c_double,
    pub stress_drag_scale: ::core::ffi::c_double,
    pub stress_lift_scale: ::core::ffi::c_double,
    pub forecast_drag_scale: ::core::ffi::c_double,
    pub forecast_lift_scale: ::core::ffi::c_double,
    pub forecast_exact_policy_bank: bool,
    pub altitude_residual: ::core::ffi::c_double,
    pub speed_residual: ::core::ffi::c_double,
    pub range_residual: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub accepted_samples: ::core::ffi::c_int,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct InFlightTrajectoryCalibrator {
    pub forecast: Trajectory,
    pub model: TrajectoryCalibrationModel,
    pub previous_ut: ::core::ffi::c_double,
    pub has_previous: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CalibrationObservationEnvelope {
    pub valid: bool,
    pub airborne: bool,
    pub clean_configuration: bool,
    pub actuator_domain_valid: bool,
    pub unpowered: bool,
    pub force_observable: bool,
    pub passive_aero_usable: bool,
    pub trajectory_usable: bool,
    pub dynamic_pressure_minimum: DecisionMargin,
    pub dynamic_pressure_maximum: DecisionMargin,
    pub incidence: DecisionMargin,
    pub sideslip: DecisionMargin,
    pub load: DecisionMargin,
    pub stall: DecisionMargin,
    pub thrust: DecisionMargin,
    pub incidence_deg: ::core::ffi::c_double,
    pub aerodynamic_accel_mps2: ::core::ffi::c_double,
    pub aerodynamic_response_time_s: ::core::ffi::c_double,
    pub information_increment: ::core::ffi::c_double,
    pub information_blend: ::core::ffi::c_double,
    pub worst_normalized_margin: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DecisionMargin {
    pub valid: bool,
    pub available: ::core::ffi::c_double,
    pub required: ::core::ffi::c_double,
    pub margin: ::core::ffi::c_double,
    pub normalized_margin: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
unsafe extern "C" fn linear_blend(
    mut current: ::core::ffi::c_double,
    mut sample: ::core::ffi::c_double,
    mut blend: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if sample.is_finite() as i32 == 0 {
        return current;
    }
    if current.is_finite() as i32 == 0 {
        return sample;
    }
    return current + clampd(blend, 0.0f64, 1.0f64) * (sample - current);
}
unsafe extern "C" fn positive_log_blend(
    mut current: ::core::ffi::c_double,
    mut sample: ::core::ffi::c_double,
    mut blend: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(current > 0.0f64) || !(sample > 0.0f64) || current.is_finite() as i32 == 0
        || sample.is_finite() as i32 == 0
    {
        return current;
    }
    blend = clampd(blend, 0.0f64, 1.0f64);
    return exp(log(current) + blend * (log(sample) - log(current)));
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_calibrator_init(
    mut c: *mut InFlightTrajectoryCalibrator,
) {
    memset(
        c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<InFlightTrajectoryCalibrator>() as size_t,
    );
    trajectory_init(&raw mut (*c).forecast);
    (*c).model.density_scale = 1.0f64;
    (*c).model.drag_scale = 1.0f64;
    (*c).model.lift_scale = 1.0f64;
    (*c).model.bank_effectiveness = 1.0f64;
    (*c).model.speed_of_sound_scale = 1.0f64;
    (*c).model.speed_of_sound = 340.0f64;
    (*c).model.confidence = 0.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_calibrator_clear(
    mut c: *mut InFlightTrajectoryCalibrator,
) {
    trajectory_clear(&raw mut (*c).forecast);
    trajectory_calibrator_init(c);
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_calibrator_set_forecast(
    mut c: *mut InFlightTrajectoryCalibrator,
    mut forecast: *const Trajectory,
) {
    trajectory_copy(&raw mut (*c).forecast, forecast);
}
unsafe extern "C" fn forecast_at(
    mut forecast: *const Trajectory,
    mut ut: ::core::ffi::c_double,
    mut out: *mut TrajectoryPoint,
) -> bool {
    if (*forecast).count < 2 as size_t
        || ut < (*(*forecast).points.offset(0 as ::core::ffi::c_int as isize)).ut
        || ut
            > (*(*forecast)
                .points
                .offset((*forecast).count.wrapping_sub(1 as size_t) as isize))
                .ut
    {
        return false_0 != 0;
    }
    let mut lo: size_t = 0 as size_t;
    let mut hi: size_t = (*forecast).count.wrapping_sub(1 as size_t);
    while hi.wrapping_sub(lo) > 1 as size_t {
        let mut mid: size_t = lo
            .wrapping_add(hi.wrapping_sub(lo).wrapping_div(2 as size_t));
        if (*(*forecast).points.offset(mid as isize)).ut >= ut {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    if ut <= (*(*forecast).points.offset(0 as ::core::ffi::c_int as isize)).ut {
        *out = *(*forecast).points.offset(0 as ::core::ffi::c_int as isize);
        return true_0 != 0;
    }
    if ut
        >= (*(*forecast)
            .points
            .offset((*forecast).count.wrapping_sub(1 as size_t) as isize))
            .ut
    {
        *out = *(*forecast)
            .points
            .offset((*forecast).count.wrapping_sub(1 as size_t) as isize);
        return true_0 != 0;
    }
    let mut a: TrajectoryPoint = *(*forecast).points.offset(lo as isize);
    let mut b: TrajectoryPoint = *(*forecast).points.offset(hi as isize);
    let mut denominator: ::core::ffi::c_double = b.ut - a.ut;
    if !(denominator > DBL_MIN) {
        return false_0 != 0;
    }
    let mut fraction: ::core::ffi::c_double = clampd(
        (ut - a.ut) / denominator,
        0.0f64,
        1.0f64,
    );
    *out = a;
    (*out).ut = ut;
    (*out).latitude = a.latitude + (b.latitude - a.latitude) * fraction;
    (*out).longitude = norm_signed_deg(
        a.longitude + norm_signed_deg(b.longitude - a.longitude) * fraction,
    );
    (*out).altitude = a.altitude + (b.altitude - a.altitude) * fraction;
    (*out).speed = a.speed + (b.speed - a.speed) * fraction;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_calibrator_update(
    mut c: *mut InFlightTrajectoryCalibrator,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut planet: *const PlanetModel,
    mut site: *const LandingSite,
    mut env: *const AerodynamicEnvelope,
    mut vehicle: *const VehicleProfile,
    mut command: *const GuidanceCommand,
    mut settings: *const CalibrationSettings,
    mut allow_adaptation: bool,
    mut dt: ::core::ffi::c_double,
) -> TrajectoryCalibrationModel {
    if (*c).has_previous as ::core::ffi::c_int != 0 && (*state).ut < (*c).previous_ut {
        (*c).has_previous = false_0 != 0;
        trajectory_clear(&raw mut (*c).forecast);
        trajectory_init(&raw mut (*c).forecast);
    }
    let mut learning_enabled: bool = (*settings).enable_trajectory_calibration
        as ::core::ffi::c_int != 0 && allow_adaptation as ::core::ffi::c_int != 0
        && (*t).physics_sample_valid as ::core::ffi::c_int != 0;
    let mut observation: CalibrationObservationEnvelope = calibration_observation_envelope(
        t,
        vehicle,
        settings,
        dt,
    );
    let mut base_density: ::core::ffi::c_double = planet_atmospheric_density(
        planet,
        (*t).mean_altitude,
    );
    if learning_enabled as ::core::ffi::c_int != 0 && (*t).mean_altitude >= 0.0f64
        && (*t).mean_altitude < (*planet).atmosphere_depth
    {
        if (*t).atmospheric_density > 0.0f64
            && (*t).atmospheric_density.is_finite() as i32 != 0 && base_density > DBL_MIN
        {
            let mut density_scale: ::core::ffi::c_double = (*t).atmospheric_density
                / base_density;
            if density_scale.is_finite() as i32 != 0 && density_scale > 0.0f64 {
                (*c).model.density_scale = density_scale;
            }
        }
        let mut observed_sound: ::core::ffi::c_double = 0.0f64;
        if (*t).speed_of_sound > 0.0f64 && (*t).speed_of_sound.is_finite() as i32 != 0 {
            observed_sound = (*t).speed_of_sound;
        } else if (*t).mach > DBL_MIN && (*t).true_air_speed > 0.0f64 {
            observed_sound = (*t).true_air_speed / (*t).mach;
        }
        if observed_sound > 0.0f64 && observed_sound.is_finite() as i32 != 0 {
            (*c).model.speed_of_sound = observed_sound;
            let mut profile_sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
                planet,
                (*t).mean_altitude,
            );
            if profile_sound > DBL_MIN {
                let mut scale: ::core::ffi::c_double = observed_sound / profile_sound;
                if scale.is_finite() as i32 != 0 && scale > 0.0f64 {
                    (*c).model.speed_of_sound_scale = scale;
                }
            }
        }
    }
    (*c).model.drag_scale = 1.0f64;
    (*c).model.lift_scale = 1.0f64;
    if learning_enabled as ::core::ffi::c_int != 0
        && observation.trajectory_usable as ::core::ffi::c_int != 0
    {
        (*c).model.accepted_samples += 1;
        (*c).model.confidence = 1.0f64
            - (1.0f64 - (*c).model.confidence) * exp(-observation.information_increment);
        let mut forecast_point: TrajectoryPoint = TrajectoryPoint {
            ut: 0.,
            latitude: 0.,
            longitude: 0.,
            altitude: 0.,
            speed: 0.,
            phase: PHASE_IDLE,
            kind: TRAJ_PLANNED,
        };
        if forecast_at(&raw mut (*c).forecast, (*t).ut, &raw mut forecast_point) {
            let mut predicted: GeoPoint = GeoPoint {
                latitude: forecast_point.latitude,
                longitude: forecast_point.longitude,
                altitude: forecast_point.altitude,
            };
            let mut target: GeoPoint = GeoPoint {
                latitude: (*site).latitude,
                longitude: (*site).longitude,
                altitude: (*site).altitude,
            };
            let mut predicted_range: ::core::ffi::c_double = great_circle_distance(
                predicted,
                target,
                (*planet).radius,
            );
            let mut altitude_error: ::core::ffi::c_double = (*t).mean_altitude
                - forecast_point.altitude;
            let mut speed_error: ::core::ffi::c_double = (*t).true_air_speed
                - forecast_point.speed;
            let mut range_error: ::core::ffi::c_double = (*t).range_to_site
                - predicted_range;
            (*c).model.altitude_residual = linear_blend(
                (*c).model.altitude_residual,
                altitude_error,
                observation.information_blend,
            );
            (*c).model.speed_residual = linear_blend(
                (*c).model.speed_residual,
                speed_error,
                observation.information_blend,
            );
            (*c).model.range_residual = linear_blend(
                (*c).model.range_residual,
                range_error,
                observation.information_blend,
            );
        }
        let mut horizontal_speed: ::core::ffi::c_double = (*t).horizontal_speed;
        let mut lift_accel: ::core::ffi::c_double = if (*t).mass > 0.0f64 {
            fmax(0.0f64, (*t).lift_force) / (*t).mass
        } else {
            0.0f64
        };
        let mut bank: ::core::ffi::c_double = (*t).roll * DEG2RAD;
        let mut lateral_fraction: ::core::ffi::c_double = sin(bank);
        let mut expected_course_rate: ::core::ffi::c_double = 0.0f64;
        if horizontal_speed > DBL_MIN && lift_accel > 0.0f64 {
            expected_course_rate = lift_accel * lateral_fraction / horizontal_speed
                * RAD2DEG;
        }
        let mut measured_course_rate: ::core::ffi::c_double = if (*t).has_course_rate
            as ::core::ffi::c_int != 0
        {
            (*t).course_rate
        } else {
            (*t).heading_rate
        };
        if fabs(expected_course_rate) > DBL_MIN
            && measured_course_rate.is_finite() as i32 != 0
        {
            let mut sample: ::core::ffi::c_double = measured_course_rate
                / expected_course_rate;
            if sample > 0.0f64 && sample.is_finite() as i32 != 0 {
                let mut excitation: ::core::ffi::c_double = lateral_fraction
                    * lateral_fraction;
                let mut bank_blend: ::core::ffi::c_double = -expm1(
                    -observation.information_increment * excitation,
                );
                (*c).model.bank_effectiveness = positive_log_blend(
                    (*c).model.bank_effectiveness,
                    sample,
                    bank_blend,
                );
            }
        }
    }
    (*c).previous_ut = (*state).ut;
    (*c).has_previous = true_0 != 0;
    if !((*c).model.density_scale > 0.0f64)
        || (*c).model.density_scale.is_finite() as i32 == 0
    {
        (*c).model.density_scale = 1.0f64;
    }
    if !((*c).model.speed_of_sound > 0.0f64)
        || (*c).model.speed_of_sound.is_finite() as i32 == 0
    {
        (*c).model.speed_of_sound = 340.0f64;
    }
    if !((*c).model.speed_of_sound_scale > 0.0f64)
        || (*c).model.speed_of_sound_scale.is_finite() as i32 == 0
    {
        (*c).model.speed_of_sound_scale = 1.0f64;
    }
    if !((*c).model.bank_effectiveness > 0.0f64)
        || (*c).model.bank_effectiveness.is_finite() as i32 == 0
    {
        (*c).model.bank_effectiveness = 1.0f64;
    }
    (*c).model.confidence = clampd((*c).model.confidence, 0.0f64, 1.0f64);
    return (*c).model;
}
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
