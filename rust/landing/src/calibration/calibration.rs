use crate::sys::*;
extern "C" {
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn guidance_command_init(c: *mut GuidanceCommand);
    fn calibration_snapshot_init(c: *mut CalibrationSnapshot);
    fn trajectory_init(t: *mut Trajectory);
    fn aerodynamic_force_factors_mach(
        mach: ::core::ffi::c_double,
        angle_of_attack: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
        lift_factor: *mut ::core::ffi::c_double,
        drag_factor: *mut ::core::ffi::c_double,
    );
    fn calibration_observation_envelope(
        telemetry: *const Telemetry,
        vehicle: *const VehicleProfile,
        settings: *const CalibrationSettings,
        dt: ::core::ffi::c_double,
    ) -> CalibrationObservationEnvelope;
    fn calibration_safety_envelope(
        telemetry: *const Telemetry,
        vehicle: *const VehicleProfile,
        settings: *const CalibrationSettings,
    ) -> CalibrationSafetyEnvelope;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
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
pub type CalibrationRunState = ::core::ffi::c_uint;
pub const CAL_ABORTED: CalibrationRunState = 5;
pub const CAL_STOPPED: CalibrationRunState = 4;
pub const CAL_COMPLETE: CalibrationRunState = 3;
pub const CAL_SAMPLING: CalibrationRunState = 2;
pub const CAL_SETTLING: CalibrationRunState = 1;
pub const CAL_IDLE: CalibrationRunState = 0;
pub type TrajectoryKind = ::core::ffi::c_uint;
pub const TRAJ_REFERENCE: TrajectoryKind = 2;
pub const TRAJ_ACTUAL: TrajectoryKind = 1;
pub const TRAJ_PLANNED: TrajectoryKind = 0;
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
pub struct CalibrationSnapshot {
    pub state: CalibrationRunState,
    pub active: bool,
    pub progress: ::core::ffi::c_double,
    pub target_angle_of_attack: ::core::ffi::c_double,
    pub accepted_samples: ::core::ffi::c_int,
    pub rejected_samples: ::core::ffi::c_int,
    pub confidence: ::core::ffi::c_double,
    pub planning_confidence: ::core::ffi::c_double,
    pub best_glide_angle_of_attack: ::core::ffi::c_double,
    pub estimated_stall_speed: ::core::ffi::c_double,
    pub status: [::core::ffi::c_char; 256],
    pub warning: [::core::ffi::c_char; 256],
    pub has_warning: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GuidanceResult {
    pub phase: GuidancePhase,
    pub command: GuidanceCommand,
    pub status: [::core::ffi::c_char; 384],
    pub warning: [::core::ffi::c_char; 384],
    pub has_warning: bool,
    pub reference: Trajectory,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GlideCalibrationMachine {
    pub active: bool,
    pub state: CalibrationRunState,
    pub start_ut: ::core::ffi::c_double,
    pub state_start_ut: ::core::ffi::c_double,
    pub initial_heading: ::core::ffi::c_double,
    pub angles: [::core::ffi::c_double; 16],
    pub angle_count: ::core::ffi::c_int,
    pub angle_index: ::core::ffi::c_int,
    pub status: [::core::ffi::c_char; 256],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AdaptiveFlightCalibrator {
    pub ld: [::core::ffi::c_double; 4],
    pub beta: [::core::ffi::c_double; 4],
    pub confidence: [::core::ffi::c_double; 4],
    pub accepted: ::core::ffi::c_int,
    pub rejected: ::core::ffi::c_int,
    pub best_glide_aoa: ::core::ffi::c_double,
    pub stall_speed: ::core::ffi::c_double,
    pub recommended: VehicleProfile,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CalibrationSafetyEnvelope {
    pub valid: bool,
    pub safe: bool,
    pub altitude: DecisionMargin,
    pub dynamic_pressure: DecisionMargin,
    pub load: DecisionMargin,
    pub sink_rate: DecisionMargin,
    pub stall: DecisionMargin,
    pub speed: DecisionMargin,
    pub worst_normalized_margin: ::core::ffi::c_double,
}
unsafe extern "C" fn regime_for(mut mach: ::core::ffi::c_double) -> ::core::ffi::c_int {
    static mut anchors: [::core::ffi::c_double; 4] = [0.35f64, 1.05f64, 2.6f64, 6.0f64];
    let mut x: ::core::ffi::c_double = fmax(0.0f64, mach);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if x
            < 0.5f64
                * (anchors[i as usize] + anchors[(i + 1 as ::core::ffi::c_int) as usize])
        {
            return i;
        }
        i += 1;
    }
    return 3 as ::core::ffi::c_int;
}
unsafe extern "C" fn positive_log_blend(
    mut current: ::core::ffi::c_double,
    mut sample: ::core::ffi::c_double,
    mut blend: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(current > 0.0f64) || !(sample > 0.0f64) || current.is_finite() as i32 == 0
        || sample.is_finite() as i32 == 0 || blend.is_finite() as i32 == 0
    {
        return current;
    }
    blend = clampd(blend, 0.0f64, 1.0f64);
    return exp(log(current) + blend * (log(sample) - log(current)));
}
#[no_mangle]
pub unsafe extern "C" fn adaptive_calibrator_init(
    mut c: *mut AdaptiveFlightCalibrator,
    mut profile: *const VehicleProfile,
) {
    memset(
        c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<AdaptiveFlightCalibrator>() as size_t,
    );
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 4 as ::core::ffi::c_int {
        (*c).ld[i as usize] = (*profile).estimated_lift_to_drag;
        (*c).beta[i as usize] = (*profile).estimated_ballistic_coefficient;
        (*c).confidence[i as usize] = 0.0f64;
        i += 1;
    }
    (*c).recommended = *profile;
}
#[no_mangle]
pub unsafe extern "C" fn adaptive_calibrator_reset(
    mut c: *mut AdaptiveFlightCalibrator,
    mut profile: *const VehicleProfile,
) {
    adaptive_calibrator_init(c, profile);
}
#[no_mangle]
pub unsafe extern "C" fn adaptive_calibrator_update(
    mut c: *mut AdaptiveFlightCalibrator,
    mut t: *const Telemetry,
    mut s: *const CalibrationSettings,
    mut baseline: *const VehicleProfile,
    mut command: *const GuidanceCommand,
    mut dt: ::core::ffi::c_double,
    mut allow_adaptation: bool,
    mut active: bool,
    mut sampling: bool,
    mut run_state: CalibrationRunState,
    mut progress: ::core::ffi::c_double,
    mut target_aoa: ::core::ffi::c_double,
    mut run_status: *const ::core::ffi::c_char,
    mut warning: *const ::core::ffi::c_char,
    mut current: *mut AerodynamicModel,
    mut planning: *mut AerodynamicModel,
    mut envelope: *mut AerodynamicEnvelope,
    mut recommended: *mut VehicleProfile,
    mut snap: *mut CalibrationSnapshot,
) {
    let mut regime: ::core::ffi::c_int = regime_for((*t).mach);
    let mut enabled: bool = (*s).enable_passive_calibration as ::core::ffi::c_int != 0
        || active as ::core::ffi::c_int != 0;
    let mut observation: CalibrationObservationEnvelope = calibration_observation_envelope(
        t,
        baseline,
        s,
        dt,
    );
    if allow_adaptation as ::core::ffi::c_int != 0 && enabled as ::core::ffi::c_int != 0
        && observation.passive_aero_usable as ::core::ffi::c_int != 0
    {
        let mut lift_factor: ::core::ffi::c_double = 0.0f64;
        let mut drag_factor: ::core::ffi::c_double = 1.0f64;
        aerodynamic_force_factors_mach(
            (*t).mach,
            observation.incidence_deg,
            baseline,
            &raw mut lift_factor,
            &raw mut drag_factor,
        );
        let mut updated: bool = false_0 != 0;
        if drag_factor > DBL_MIN && (*t).drag_force > DBL_MIN {
            let mut beta_sample: ::core::ffi::c_double = (*t).dynamic_pressure
                * (*t).mass / (*t).drag_force * drag_factor;
            if beta_sample.is_finite() as i32 != 0 && beta_sample > 0.0f64 {
                (*c).beta[regime as usize] = positive_log_blend(
                    (*c).beta[regime as usize],
                    beta_sample,
                    observation.information_blend,
                );
                updated = true_0 != 0;
            }
            if fabs(lift_factor) > DBL_MIN && (*t).lift_force >= 0.0f64 {
                let mut observed_ld: ::core::ffi::c_double = (*t).lift_force
                    / (*t).drag_force;
                let mut ld_sample: ::core::ffi::c_double = observed_ld * drag_factor
                    / fabs(lift_factor);
                if ld_sample.is_finite() as i32 != 0 && ld_sample > 0.0f64 {
                    (*c).ld[regime as usize] = positive_log_blend(
                        (*c).ld[regime as usize],
                        ld_sample,
                        observation.information_blend,
                    );
                    updated = true_0 != 0;
                }
            }
        }
        if updated {
            (*c).accepted += 1;
            (*c).confidence[regime as usize] = 1.0f64
                - (1.0f64 - (*c).confidence[regime as usize])
                    * exp(-observation.information_increment);
        } else {
            (*c).rejected += 1;
        }
    } else if allow_adaptation as ::core::ffi::c_int != 0
        && enabled as ::core::ffi::c_int != 0
        && observation.airborne as ::core::ffi::c_int != 0
    {
        (*c).rejected += 1;
    }
    let mut current_conf: ::core::ffi::c_double = clampd(
        (*c).confidence[regime as usize],
        0.0f64,
        1.0f64,
    );
    let mut planning_ld: ::core::ffi::c_double = (*baseline).estimated_lift_to_drag;
    let mut planning_beta: ::core::ffi::c_double = (*baseline)
        .estimated_ballistic_coefficient;
    let mut high_weight: ::core::ffi::c_double = (*c)
        .confidence[2 as ::core::ffi::c_int as usize]
        + (*c).confidence[3 as ::core::ffi::c_int as usize];
    if high_weight > DBL_MIN {
        planning_ld = ((*c).ld[2 as ::core::ffi::c_int as usize]
            * (*c).confidence[2 as ::core::ffi::c_int as usize]
            + (*c).ld[3 as ::core::ffi::c_int as usize]
                * (*c).confidence[3 as ::core::ffi::c_int as usize]) / high_weight;
        planning_beta = ((*c).beta[2 as ::core::ffi::c_int as usize]
            * (*c).confidence[2 as ::core::ffi::c_int as usize]
            + (*c).beta[3 as ::core::ffi::c_int as usize]
                * (*c).confidence[3 as ::core::ffi::c_int as usize]) / high_weight;
    }
    let mut planning_conf: ::core::ffi::c_double = 0.5f64
        * ((*c).confidence[2 as ::core::ffi::c_int as usize]
            + (*c).confidence[3 as ::core::ffi::c_int as usize]);
    (*c).recommended = *baseline;
    (*current).lift_to_drag = (*c).ld[regime as usize];
    (*current).ballistic_coefficient = (*c).beta[regime as usize];
    (*current).confidence = current_conf;
    (*planning).lift_to_drag = planning_ld;
    (*planning).ballistic_coefficient = planning_beta;
    (*planning).confidence = planning_conf;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 4 as ::core::ffi::c_int {
        (*envelope).regimes[i as usize].lift_to_drag = (*c).ld[i as usize];
        (*envelope).regimes[i as usize].ballistic_coefficient = (*c).beta[i as usize];
        (*envelope).regimes[i as usize].confidence = clampd(
            (*c).confidence[i as usize],
            0.0f64,
            1.0f64,
        );
        i += 1;
    }
    *recommended = (*c).recommended;
    calibration_snapshot_init(snap);
    (*snap).state = (if active as ::core::ffi::c_int != 0
        || run_state as ::core::ffi::c_uint
            != CAL_IDLE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        run_state as ::core::ffi::c_uint
    } else {
        CAL_IDLE as ::core::ffi::c_int as ::core::ffi::c_uint
    }) as CalibrationRunState;
    (*snap).active = active;
    (*snap).progress = clampd(progress, 0.0f64, 1.0f64);
    (*snap).target_angle_of_attack = target_aoa;
    (*snap).accepted_samples = (*c).accepted;
    (*snap).rejected_samples = (*c).rejected;
    (*snap).confidence = current_conf;
    (*snap).planning_confidence = planning_conf;
    (*snap).best_glide_angle_of_attack = (*c).best_glide_aoa;
    (*snap).estimated_stall_speed = (*c).stall_speed;
    if !run_status.is_null() && *run_status as ::core::ffi::c_int != 0 {
        snprintf(
            &raw mut (*snap).status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"%s%s\0" as *const u8 as *const ::core::ffi::c_char,
            run_status,
            if active as ::core::ffi::c_int != 0 && !sampling
                && run_state as ::core::ffi::c_uint
                    == CAL_SETTLING as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                b" Waiting for the configured settling interval.\0" as *const u8
                    as *const ::core::ffi::c_char
            } else {
                b"\0" as *const u8 as *const ::core::ffi::c_char
            },
        );
    } else if (*c).accepted == 0 as ::core::ffi::c_int {
        snprintf(
            &raw mut (*snap).status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"Waiting for an in-domain clean unpowered aerodynamic observation.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    } else {
        snprintf(
            &raw mut (*snap).status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"Shadow aerodynamic reconstruction: %d usable observations; current-regime information confidence %.3f.\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*c).accepted,
            current_conf,
        );
    }
    if !warning.is_null() && *warning as ::core::ffi::c_int != 0 {
        (*snap).has_warning = true_0 != 0;
        snprintf(
            &raw mut (*snap).warning as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            warning,
        );
    }
}
#[no_mangle]
pub unsafe extern "C" fn glide_calibration_init(mut g: *mut GlideCalibrationMachine) {
    memset(
        g as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<GlideCalibrationMachine>() as size_t,
    );
    (*g).state = CAL_IDLE;
    snprintf(
        &raw mut (*g).status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"Calibration is idle.\0" as *const u8 as *const ::core::ffi::c_char,
    );
}
unsafe extern "C" fn make_angles(
    mut g: *mut GlideCalibrationMachine,
    mut s: *const CalibrationSettings,
) {
    (*g).angle_count = 0 as ::core::ffi::c_int;
    let mut capacity: size_t = (::core::mem::size_of::<[::core::ffi::c_double; 16]>()
        as size_t)
        .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as size_t);
    let mut angle: ::core::ffi::c_double = (*s).minimum_angle_of_attack;
    while ((*g).angle_count as size_t).wrapping_add(1 as size_t) < capacity
        && angle < (*s).maximum_angle_of_attack
    {
        let fresh0 = (*g).angle_count;
        (*g).angle_count = (*g).angle_count + 1;
        (*g).angles[fresh0 as usize] = angle;
        angle += (*s).angle_of_attack_step;
    }
    if ((*g).angle_count as size_t) < capacity {
        let mut scale: ::core::ffi::c_double = fmax(
            1.0f64,
            fabs((*s).maximum_angle_of_attack),
        );
        let mut already_at_end: bool = (*g).angle_count > 0 as ::core::ffi::c_int
            && fabs(
                (*g).angles[((*g).angle_count - 1 as ::core::ffi::c_int) as usize]
                    - (*s).maximum_angle_of_attack,
            ) <= DBL_EPSILON * scale;
        if !already_at_end {
            let fresh1 = (*g).angle_count;
            (*g).angle_count = (*g).angle_count + 1;
            (*g).angles[fresh1 as usize] = (*s).maximum_angle_of_attack;
        }
    }
}
#[no_mangle]
pub unsafe extern "C" fn glide_calibration_start(
    mut g: *mut GlideCalibrationMachine,
    mut t: *const Telemetry,
    mut s: *const CalibrationSettings,
) {
    glide_calibration_init(g);
    (*g).active = true_0 != 0;
    (*g).state = CAL_SETTLING;
    (*g).state_start_ut = (*t).ut;
    (*g).start_ut = (*g).state_start_ut;
    (*g).initial_heading = (*t).heading;
    make_angles(g, s);
    snprintf(
        &raw mut (*g).status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"Starting a wings-level glide calibration.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
}
#[no_mangle]
pub unsafe extern "C" fn glide_calibration_stop(
    mut g: *mut GlideCalibrationMachine,
    mut status: *const ::core::ffi::c_char,
) {
    (*g).active = false_0 != 0;
    (*g).state = CAL_STOPPED;
    snprintf(
        &raw mut (*g).status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if !status.is_null() {
            status
        } else {
            b"Calibration stopped. Manual control restored.\0" as *const u8
                as *const ::core::ffi::c_char
        },
    );
}
unsafe extern "C" fn safety_failure(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut s: *const CalibrationSettings,
) -> *const ::core::ffi::c_char {
    let mut safety: CalibrationSafetyEnvelope = calibration_safety_envelope(t, v, s);
    if !safety.valid {
        return b"Calibration requires an airborne craft.\0" as *const u8
            as *const ::core::ffi::c_char;
    }
    if safety.altitude.margin < 0.0f64 {
        return b"Calibration altitude floor reached.\0" as *const u8
            as *const ::core::ffi::c_char;
    }
    if safety.dynamic_pressure.margin < 0.0f64 {
        return b"Calibration stopped because dynamic pressure exceeded its safety limit.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if safety.load.margin < 0.0f64 {
        return b"Calibration stopped because G-load exceeded its safety limit.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if safety.sink_rate.margin < 0.0f64 {
        return b"Calibration stopped because sink rate exceeded its safety limit.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if safety.stall.margin < 0.0f64 {
        return b"Calibration stopped at the configured stall boundary.\0" as *const u8
            as *const ::core::ffi::c_char;
    }
    if safety.speed.margin < 0.0f64 {
        return b"Calibration stopped because airspeed fell below the vehicle minimum safe speed.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    return ::core::ptr::null::<::core::ffi::c_char>();
}
#[no_mangle]
pub unsafe extern "C" fn glide_calibration_update(
    mut g: *mut GlideCalibrationMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut s: *const CalibrationSettings,
    mut sampling: *mut bool,
    mut finished: *mut bool,
    mut progress: *mut ::core::ffi::c_double,
    mut target_aoa: *mut ::core::ffi::c_double,
) -> GuidanceResult {
    *sampling = false_0 != 0;
    *finished = false_0 != 0;
    *progress = 0.0f64;
    *target_aoa = 0.0f64;
    let mut safe: GuidanceCommand = GuidanceCommand {
        target_pitch: 0.,
        target_heading: 0.,
        target_roll: 0.,
        target_throttle: 0.,
        wheel_steering: 0.,
        has_target_aoa: false,
        target_aoa: 0.,
        has_pitch_trim_feedforward: false,
        pitch_trim_feedforward: 0.,
        speedbrake_fraction: 0.,
        gear: false,
        brakes: false,
        airbrakes: false,
        use_inertial_direction: false,
        thrust_alignment: false,
        inertial_direction: Vector3 { x: 0., y: 0., z: 0. },
        autopilot_engaged: false,
        heading_control_enabled: false,
        hac_control_tuning: false,
        terminal_pitch_tuning: false,
        navball_speed_mode: SPEED_UNCHANGED,
        control_profile: PROFILE_ORBITAL,
        terminal_roll_trim_enabled: false,
        jet_assist_throttle: 0.,
    };
    guidance_command_init(&raw mut safe);
    if !(*g).active {
        *finished = (*g).state as ::core::ffi::c_uint
            == CAL_COMPLETE as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).state as ::core::ffi::c_uint
                == CAL_STOPPED as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).state as ::core::ffi::c_uint
                == CAL_ABORTED as ::core::ffi::c_int as ::core::ffi::c_uint;
        return GuidanceResult {
            phase: PHASE_CALIBRATION,
            command: safe,
            status: ::core::mem::transmute::<
                [u8; 384],
                [::core::ffi::c_char; 384],
            >(
                *b"Calibration is idle.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            ),
            warning: [0; 384],
            has_warning: false,
            reference: Trajectory {
                points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                count: 0,
                capacity: 0,
            },
        };
    }
    let mut failure: *const ::core::ffi::c_char = safety_failure(t, v, s);
    if !failure.is_null() {
        (*g).active = false_0 != 0;
        (*g).state = CAL_ABORTED;
        snprintf(
            &raw mut (*g).status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            failure,
        );
        let mut result: GuidanceResult = GuidanceResult {
            phase: PHASE_IDLE,
            command: GuidanceCommand {
                target_pitch: 0.,
                target_heading: 0.,
                target_roll: 0.,
                target_throttle: 0.,
                wheel_steering: 0.,
                has_target_aoa: false,
                target_aoa: 0.,
                has_pitch_trim_feedforward: false,
                pitch_trim_feedforward: 0.,
                speedbrake_fraction: 0.,
                gear: false,
                brakes: false,
                airbrakes: false,
                use_inertial_direction: false,
                thrust_alignment: false,
                inertial_direction: Vector3 { x: 0., y: 0., z: 0. },
                autopilot_engaged: false,
                heading_control_enabled: false,
                hac_control_tuning: false,
                terminal_pitch_tuning: false,
                navball_speed_mode: SPEED_UNCHANGED,
                control_profile: PROFILE_ORBITAL,
                terminal_roll_trim_enabled: false,
                jet_assist_throttle: 0.,
            },
            status: [0; 384],
            warning: [0; 384],
            has_warning: false,
            reference: Trajectory {
                points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                count: 0,
                capacity: 0,
            },
        };
        memset(
            &raw mut result as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<GuidanceResult>() as size_t,
        );
        result.phase = PHASE_CALIBRATION;
        result.command = safe;
        result.has_warning = true_0 != 0;
        snprintf(
            &raw mut result.status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            failure,
        );
        snprintf(
            &raw mut result.warning as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            failure,
        );
        trajectory_init(&raw mut result.reference);
        *finished = true_0 != 0;
        return result;
    }
    let mut total: ::core::ffi::c_double = (*s).settling_duration
        + (*s).sampling_duration;
    if total <= 0.0f64 {
        (*g).angle_index = (*g).angle_count;
    } else {
        while (*g).angle_index < (*g).angle_count
            && (*t).ut - (*g).state_start_ut >= total
        {
            (*g).angle_index += 1;
            (*g).state_start_ut += total;
        }
    }
    if (*g).angle_index >= (*g).angle_count {
        (*g).active = false_0 != 0;
        (*g).state = CAL_COMPLETE;
        snprintf(
            &raw mut (*g).status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"Calibration sweep completed. Review the shadow profile; recorded force data becomes certified on a later session.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        let mut result_0: GuidanceResult = GuidanceResult {
            phase: PHASE_IDLE,
            command: GuidanceCommand {
                target_pitch: 0.,
                target_heading: 0.,
                target_roll: 0.,
                target_throttle: 0.,
                wheel_steering: 0.,
                has_target_aoa: false,
                target_aoa: 0.,
                has_pitch_trim_feedforward: false,
                pitch_trim_feedforward: 0.,
                speedbrake_fraction: 0.,
                gear: false,
                brakes: false,
                airbrakes: false,
                use_inertial_direction: false,
                thrust_alignment: false,
                inertial_direction: Vector3 { x: 0., y: 0., z: 0. },
                autopilot_engaged: false,
                heading_control_enabled: false,
                hac_control_tuning: false,
                terminal_pitch_tuning: false,
                navball_speed_mode: SPEED_UNCHANGED,
                control_profile: PROFILE_ORBITAL,
                terminal_roll_trim_enabled: false,
                jet_assist_throttle: 0.,
            },
            status: [0; 384],
            warning: [0; 384],
            has_warning: false,
            reference: Trajectory {
                points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                count: 0,
                capacity: 0,
            },
        };
        memset(
            &raw mut result_0 as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<GuidanceResult>() as size_t,
        );
        result_0.phase = PHASE_CALIBRATION;
        result_0.command = safe;
        snprintf(
            &raw mut result_0.status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut (*g).status as *mut ::core::ffi::c_char,
        );
        trajectory_init(&raw mut result_0.reference);
        *finished = true_0 != 0;
        *progress = 1.0f64;
        return result_0;
    }
    let mut elapsed: ::core::ffi::c_double = fmax(0.0f64, (*t).ut - (*g).state_start_ut);
    *sampling = elapsed >= (*s).settling_duration;
    (*g).state = (if *sampling as ::core::ffi::c_int != 0 {
        CAL_SAMPLING as ::core::ffi::c_int
    } else {
        CAL_SETTLING as ::core::ffi::c_int
    }) as CalibrationRunState;
    let mut angle: ::core::ffi::c_double = (*g).angles[(*g).angle_index as usize];
    *target_aoa = angle;
    let mut interval_progress: ::core::ffi::c_double = if total > 0.0f64 {
        clampd(elapsed / total, 0.0f64, 1.0f64)
    } else {
        1.0f64
    };
    *progress = ((*g).angle_index as ::core::ffi::c_double + interval_progress)
        / (*g).angle_count as ::core::ffi::c_double;
    let mut command_out: GuidanceCommand = GuidanceCommand {
        target_pitch: 0.,
        target_heading: 0.,
        target_roll: 0.,
        target_throttle: 0.,
        wheel_steering: 0.,
        has_target_aoa: false,
        target_aoa: 0.,
        has_pitch_trim_feedforward: false,
        pitch_trim_feedforward: 0.,
        speedbrake_fraction: 0.,
        gear: false,
        brakes: false,
        airbrakes: false,
        use_inertial_direction: false,
        thrust_alignment: false,
        inertial_direction: Vector3 { x: 0., y: 0., z: 0. },
        autopilot_engaged: false,
        heading_control_enabled: false,
        hac_control_tuning: false,
        terminal_pitch_tuning: false,
        navball_speed_mode: SPEED_UNCHANGED,
        control_profile: PROFILE_ORBITAL,
        terminal_roll_trim_enabled: false,
        jet_assist_throttle: 0.,
    };
    guidance_command_init(&raw mut command_out);
    command_out.autopilot_engaged = true_0 != 0;
    command_out.target_heading = (*g).initial_heading;
    command_out.target_roll = 0.0f64;
    command_out.has_target_aoa = true_0 != 0;
    command_out.target_aoa = angle;
    command_out.target_pitch = (*t).flight_path_angle + angle;
    command_out.gear = (*t).gear;
    command_out.airbrakes = false_0 != 0;
    command_out.navball_speed_mode = SPEED_SURFACE;
    command_out.control_profile = PROFILE_APPROACH;
    let mut result_1: GuidanceResult = GuidanceResult {
        phase: PHASE_IDLE,
        command: GuidanceCommand {
            target_pitch: 0.,
            target_heading: 0.,
            target_roll: 0.,
            target_throttle: 0.,
            wheel_steering: 0.,
            has_target_aoa: false,
            target_aoa: 0.,
            has_pitch_trim_feedforward: false,
            pitch_trim_feedforward: 0.,
            speedbrake_fraction: 0.,
            gear: false,
            brakes: false,
            airbrakes: false,
            use_inertial_direction: false,
            thrust_alignment: false,
            inertial_direction: Vector3 { x: 0., y: 0., z: 0. },
            autopilot_engaged: false,
            heading_control_enabled: false,
            hac_control_tuning: false,
            terminal_pitch_tuning: false,
            navball_speed_mode: SPEED_UNCHANGED,
            control_profile: PROFILE_ORBITAL,
            terminal_roll_trim_enabled: false,
            jet_assist_throttle: 0.,
        },
        status: [0; 384],
        warning: [0; 384],
        has_warning: false,
        reference: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
    };
    memset(
        &raw mut result_1 as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<GuidanceResult>() as size_t,
    );
    result_1.phase = PHASE_CALIBRATION;
    result_1.command = command_out;
    trajectory_init(&raw mut result_1.reference);
    snprintf(
        &raw mut result_1.status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        if *sampling as ::core::ffi::c_int != 0 {
            b"Sampling the clean glide at %.1f deg AoA.\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"Settling at %.1f deg AoA before sampling.\0" as *const u8
                as *const ::core::ffi::c_char
        },
        angle,
    );
    snprintf(
        &raw mut (*g).status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut result_1.status as *mut ::core::ffi::c_char,
    );
    return result_1;
}
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
