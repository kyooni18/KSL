use crate::sys::*;
extern "C" {
    fn jw_init(w: *mut JsonWriter);
    fn jw_free(w: *mut JsonWriter);
    fn jw_raw(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_char(w: *mut JsonWriter, c: ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_number(w: *mut JsonWriter, value: ::core::ffi::c_double);
    fn jw_integer(w: *mut JsonWriter, value: ::core::ffi::c_longlong);
    fn jw_bool(w: *mut JsonWriter, value: bool);
    fn jw_null(w: *mut JsonWriter);
    fn taem_terminal_block_reason_string(
        reason: TaemTerminalBlockReason,
    ) -> *const ::core::ffi::c_char;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn landing_configuration_json(w: *mut JsonWriter, c: *const LandingConfiguration);
    fn trajectory_init(t: *mut Trajectory);
    fn trajectory_clear(t: *mut Trajectory);
    fn trajectory_append(t: *mut Trajectory, p: TrajectoryPoint) -> bool;
    fn trajectory_json(w: *mut JsonWriter, t: *const Trajectory);
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonWriter {
    pub data: *mut ::core::ffi::c_char,
    pub length: size_t,
    pub capacity: size_t,
    pub failed: bool,
}
pub type EntryPhase = ::core::ffi::c_uint;
pub const ENTRY_PHASE_TRANSITION: EntryPhase = 5;
pub const ENTRY_PHASE_CONSTANT_DRAG: EntryPhase = 4;
pub const ENTRY_PHASE_EQUILIBRIUM_GLIDE: EntryPhase = 3;
pub const ENTRY_PHASE_TEMPERATURE_CONTROL: EntryPhase = 2;
pub const ENTRY_PHASE_PREENTRY: EntryPhase = 1;
pub type EntryTransitionReason = ::core::ffi::c_uint;
pub const ENTRY_TRANSITION_TAEM_FORCED_HANDOFF: EntryTransitionReason = 8;
pub const ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF: EntryTransitionReason = 7;
pub const ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY: EntryTransitionReason = 6;
pub const ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT: EntryTransitionReason = 5;
pub const ENTRY_TRANSITION_TEMPERATURE_VELOCITY: EntryTransitionReason = 4;
pub const ENTRY_TRANSITION_ATMOSPHERIC_CONTACT: EntryTransitionReason = 3;
pub const ENTRY_TRANSITION_RESTART_CLASSIFICATION: EntryTransitionReason = 2;
pub const ENTRY_TRANSITION_ENTRY_START: EntryTransitionReason = 1;
pub const ENTRY_TRANSITION_NONE: EntryTransitionReason = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryExecTelemetry {
    pub initialized: bool,
    pub entry_complete: bool,
    pub active_phase: EntryPhase,
    pub last_transition_reason: EntryTransitionReason,
    pub last_transition_from: EntryPhase,
    pub last_transition_to: EntryPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_entry: bool,
    pub transition_count: ::core::ffi::c_uint,
}
pub type TaemPhase = ::core::ffi::c_uint;
pub const TAEM_PHASE_FINAL_INTERCEPT: TaemPhase = 2;
pub const TAEM_PHASE_RUNWAY_ALIGNMENT: TaemPhase = 1;
pub const TAEM_PHASE_PATH_ACQUISITION: TaemPhase = 0;
pub type TaemTransitionReason = ::core::ffi::c_uint;
pub const TAEM_TRANSITION_LOW_ENERGY_DELIVERY: TaemTransitionReason = 7;
pub const TAEM_TRANSITION_TERMINAL_PATH_REPLAN: TaemTransitionReason = 6;
pub const TAEM_TRANSITION_FINAL_APPROACH_DELIVERY: TaemTransitionReason = 5;
pub const TAEM_TRANSITION_FINAL_INTERCEPT_GATE: TaemTransitionReason = 4;
pub const TAEM_TRANSITION_TERMINAL_PATH_CAPTURE: TaemTransitionReason = 3;
pub const TAEM_TRANSITION_RESTART_CLASSIFICATION: TaemTransitionReason = 2;
pub const TAEM_TRANSITION_MM304_HANDOFF: TaemTransitionReason = 1;
pub const TAEM_TRANSITION_NONE: TaemTransitionReason = 0;
pub type TaemRecoveryReason = ::core::ffi::c_uint;
pub const TAEM_RECOVERY_ENERGY_INFEASIBLE: TaemRecoveryReason = 4;
pub const TAEM_RECOVERY_CAPTURE_LOSS: TaemRecoveryReason = 3;
pub const TAEM_RECOVERY_PATH_REPLAN: TaemRecoveryReason = 2;
pub const TAEM_RECOVERY_ATTITUDE: TaemRecoveryReason = 1;
pub const TAEM_RECOVERY_NONE: TaemRecoveryReason = 0;
pub type TaemTerminalBlockReason = ::core::ffi::c_uint;
pub const TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE: TaemTerminalBlockReason = 10;
pub const TAEM_TERMINAL_BLOCK_FINITE_RESPONSE: TaemTerminalBlockReason = 9;
pub const TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY: TaemTerminalBlockReason = 8;
pub const TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE: TaemTerminalBlockReason = 7;
pub const TAEM_TERMINAL_BLOCK_ALTITUDE: TaemTerminalBlockReason = 6;
pub const TAEM_TERMINAL_BLOCK_SPEEDBRAKE: TaemTerminalBlockReason = 5;
pub const TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE: TaemTerminalBlockReason = 4;
pub const TAEM_TERMINAL_BLOCK_RANGE: TaemTerminalBlockReason = 3;
pub const TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED: TaemTerminalBlockReason = 2;
pub const TAEM_TERMINAL_BLOCK_CONTRACT_INVALID: TaemTerminalBlockReason = 1;
pub const TAEM_TERMINAL_BLOCK_NONE: TaemTerminalBlockReason = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTerminalContract {
    pub valid: bool,
    pub path_committed: bool,
    pub range_to_go: ::core::ffi::c_double,
    pub range_margin: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub dynamic_pressure_margin: ::core::ffi::c_double,
    pub speedbrake_required: bool,
    pub speedbrake_available: bool,
    pub speedbrake_dynamic_pressure_margin: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub altitude_margin: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub flight_path_angle_margin: ::core::ffi::c_double,
    pub specific_energy: ::core::ffi::c_double,
    pub specific_energy_margin: ::core::ffi::c_double,
    pub response_time_available: ::core::ffi::c_double,
    pub response_time_required: ::core::ffi::c_double,
    pub attitude_response_qualified: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTerminalEvaluation {
    pub valid: bool,
    pub feasible: bool,
    pub block_reason: TaemTerminalBlockReason,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemExecTelemetry {
    pub initialized: bool,
    pub ownership_latched: bool,
    pub taem_complete: bool,
    pub active_phase: TaemPhase,
    pub ownership_ut: ::core::ffi::c_double,
    pub ownership_velocity: ::core::ffi::c_double,
    pub last_transition_reason: TaemTransitionReason,
    pub last_transition_from: TaemPhase,
    pub last_transition_to: TaemPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_taem: bool,
    pub transition_count: ::core::ffi::c_uint,
    pub recovery_active: bool,
    pub recovery_reason: TaemRecoveryReason,
    pub last_recovery_reason: TaemRecoveryReason,
    pub last_recovery_ut: ::core::ffi::c_double,
    pub recovery_count: ::core::ffi::c_uint,
    pub terminal_contract: TaemTerminalContract,
    pub terminal_evaluation: TaemTerminalEvaluation,
}
pub type __uint64_t = u64;
pub type uint64_t = __uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
pub type ConnectionStatus = ::core::ffi::c_uint;
pub const CONN_FAILED: ConnectionStatus = 3;
pub const CONN_CONNECTED: ConnectionStatus = 2;
pub const CONN_CONNECTING: ConnectionStatus = 1;
pub const CONN_DISCONNECTED: ConnectionStatus = 0;
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
pub struct DeorbitPlan {
    pub created_ut: ::core::ffi::c_double,
    pub burn_ut: ::core::ffi::c_double,
    pub delta_v: ::core::ffi::c_double,
    pub estimated_burn_duration: ::core::ffi::c_double,
    pub predicted_taem_distance: ::core::ffi::c_double,
    pub predicted_taem_range_error: ::core::ffi::c_double,
    pub predicted_closest_distance: ::core::ffi::c_double,
    pub planning_mass: ::core::ffi::c_double,
    pub planning_available_thrust: ::core::ffi::c_double,
    pub burn_normal_angle: ::core::ffi::c_double,
    pub predicted_entry_range: ::core::ffi::c_double,
    pub predicted_entry_flight_path_angle: ::core::ffi::c_double,
    pub predicted_post_burn_periapsis_altitude: ::core::ffi::c_double,
    pub nominal_capture_achieved: bool,
    pub robustness_qualified: bool,
    pub target_capture_achieved: bool,
    pub terminal_model_snapshot_id: uint64_t,
    pub terminal_replay_evaluated: bool,
    pub terminal_inlet_qualified: bool,
    pub terminal_native_path_committed: bool,
    pub terminal_final_approach_captured: bool,
    pub terminal_guidance_aborted: bool,
    pub terminal_inlet_energy_margin: ::core::ffi::c_double,
    pub robustness_scenarios: ::core::ffi::c_uint,
    pub robustness_passed: ::core::ffi::c_uint,
    pub robustness_unsafe: ::core::ffi::c_uint,
    pub robustness_pass_fraction: ::core::ffi::c_double,
    pub worst_case_closest_distance: ::core::ffi::c_double,
    pub worst_case_taem_range_error: ::core::ffi::c_double,
    pub execution_qualified: bool,
    pub execution_degraded: bool,
    pub entry_stage_qualified: bool,
    pub entry_stage_scenarios: ::core::ffi::c_uint,
    pub entry_stage_passed: ::core::ffi::c_uint,
    pub entry_stage_pass_fraction: ::core::ffi::c_double,
    pub recovery_passed: ::core::ffi::c_uint,
    pub recovery_pass_fraction: ::core::ffi::c_double,
    pub worst_case_entry_flight_path_angle: ::core::ffi::c_double,
    pub worst_case_post_burn_periapsis_altitude: ::core::ffi::c_double,
    pub worst_case_peak_dynamic_pressure: ::core::ffi::c_double,
    pub worst_case_peak_g_load: ::core::ffi::c_double,
    pub live_cutoff_capture_qualified: bool,
    pub live_cutoff_periapsis_altitude: ::core::ffi::c_double,
    pub live_cutoff_closest_distance: ::core::ffi::c_double,
    pub live_cutoff_entry_flight_path_angle: ::core::ffi::c_double,
    pub achieved_state_verified: bool,
    pub achieved_state_capture_qualified: bool,
    pub achieved_post_burn_periapsis_altitude: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub trajectory: Trajectory,
    pub note: [::core::ffi::c_char; 384],
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
pub struct EntryGuidanceFeedback {
    pub valid: bool,
    pub lateral_valid: bool,
    pub reversal_requested: bool,
    pub ut: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub measured_drag_accel: ::core::ffi::c_double,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub drag_error_accel: ::core::ffi::c_double,
    pub predicted_achievable_range: ::core::ffi::c_double,
    pub required_range_to_go: ::core::ffi::c_double,
    pub energy_margin: ::core::ffi::c_double,
    pub range_error_scale: ::core::ffi::c_double,
    pub required_vertical_lift_accel: ::core::ffi::c_double,
    pub crossrange_error: ::core::ffi::c_double,
    pub crossrange_rate: ::core::ffi::c_double,
    pub projected_crossrange_error: ::core::ffi::c_double,
    pub crossrange_corridor: ::core::ffi::c_double,
    pub energy_bank_magnitude_deg: ::core::ffi::c_double,
    pub commanded_bank_deg: ::core::ffi::c_double,
    pub bank_sign: ::core::ffi::c_double,
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
pub struct KRPCApplyResult {
    pub applied: bool,
    pub autopilot_engaged: bool,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: bool,
    pub target_pitch: ::core::ffi::c_double,
    pub target_heading: ::core::ffi::c_double,
    pub target_roll: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub wheel_steering: ::core::ffi::c_double,
    pub has_actuator_feedback: bool,
    pub control_pitch: ::core::ffi::c_double,
    pub control_roll: ::core::ffi::c_double,
    pub control_yaw: ::core::ffi::c_double,
    pub has_control_diagnostics: bool,
    pub control_body_pitch_rate_available: bool,
    pub control_body_roll_rate_available: bool,
    pub control_body_yaw_rate_available: bool,
    pub control_target_aoa: ::core::ffi::c_double,
    pub control_measured_aoa: ::core::ffi::c_double,
    pub control_aoa_rate: ::core::ffi::c_double,
    pub control_roll_rate: ::core::ffi::c_double,
    pub control_body_pitch_rate: ::core::ffi::c_double,
    pub control_body_roll_rate: ::core::ffi::c_double,
    pub control_body_yaw_rate: ::core::ffi::c_double,
    pub control_pitch_error: ::core::ffi::c_double,
    pub control_pitch_trim: ::core::ffi::c_double,
    pub control_pitch_authority: ::core::ffi::c_double,
    pub control_pitch_aero_fraction: ::core::ffi::c_double,
    pub control_roll_authority: ::core::ffi::c_double,
    pub control_roll_raw_authority: ::core::ffi::c_double,
    pub control_roll_aero_fraction: ::core::ffi::c_double,
    pub control_target_roll_rate: ::core::ffi::c_double,
    pub control_beta_yaw_gain: ::core::ffi::c_double,
    pub control_beta_roll_coupling: ::core::ffi::c_double,
    pub control_beta_confidence: ::core::ffi::c_double,
    pub control_yaw_command: ::core::ffi::c_double,
    pub control_revision: [::core::ffi::c_char; 48],
    pub reference_frame: [::core::ffi::c_char; 24],
    pub speed_mode: [::core::ffi::c_char; 24],
    pub control_profile: [::core::ffi::c_char; 24],
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
pub struct LandingSnapshot {
    pub connection_status: ConnectionStatus,
    pub phase: GuidancePhase,
    pub telemetry: Telemetry,
    pub command: GuidanceCommand,
    pub plan: *mut DeorbitPlan,
    pub actual_trajectory: Trajectory,
    pub reference_trajectory: Trajectory,
    pub predicted_trajectory: Trajectory,
    pub status_message: [::core::ffi::c_char; 384],
    pub warning_message: [::core::ffi::c_char; 384],
    pub last_error: [::core::ffi::c_char; 512],
    pub has_warning: bool,
    pub has_error: bool,
    pub automation_engaged: bool,
    pub paused: bool,
    pub calibration: CalibrationSnapshot,
    pub adaptive_vehicle_profile: VehicleProfile,
    pub tick_sequence: uint64_t,
    pub log_sequence: uint64_t,
    pub wall_monotonic_seconds: ::core::ffi::c_double,
    pub simulation_dt: ::core::ffi::c_double,
    pub has_vehicle_state: bool,
    pub vehicle_state: VehicleState,
    pub has_attitude_quaternion: bool,
    pub attitude_quaternion: [::core::ffi::c_double; 4],
    pub attitude_reference_frame: [::core::ffi::c_char; 48],
    pub command_apply_attempted: bool,
    pub command_applied: bool,
    pub command_computed_wall_seconds: ::core::ffi::c_double,
    pub command_apply_started_wall_seconds: ::core::ffi::c_double,
    pub command_apply_finished_wall_seconds: ::core::ffi::c_double,
    pub applied_command: KRPCApplyResult,
    pub guidance_has_burn_started: bool,
    pub guidance_burn_completed: bool,
    pub guidance_atmosphere_crossed: bool,
    pub guidance_final_captured: bool,
    pub guidance_airbrakes_deployed: bool,
    pub guidance_hac_side_selected: bool,
    pub has_active_runway: bool,
    pub active_runway: LandingSite,
    pub active_final_distance: ::core::ffi::c_double,
    pub active_final_slope: ::core::ffi::c_double,
    pub guidance_delivered_delta_v: ::core::ffi::c_double,
    pub guidance_burn_active_elapsed: ::core::ffi::c_double,
    pub guidance_s_turn_sign: ::core::ffi::c_double,
    pub guidance_hac_side: ::core::ffi::c_double,
    pub guidance_entry_leg_elapsed: ::core::ffi::c_double,
    pub guidance_entry_plan_valid: bool,
    pub guidance_entry_plan_terminal_ready: bool,
    pub guidance_entry_reversal_scheduled: bool,
    pub guidance_entry_reversal_is_final: bool,
    pub guidance_entry_plan_ut: ::core::ffi::c_double,
    pub guidance_entry_plan_target_bank: ::core::ffi::c_double,
    pub guidance_entry_plan_target_aoa: ::core::ffi::c_double,
    pub guidance_entry_plan_target_heading: ::core::ffi::c_double,
    pub guidance_entry_plan_segment_remaining: ::core::ffi::c_double,
    pub guidance_entry_plan_cost: ::core::ffi::c_double,
    pub guidance_entry_plan_taem_range_error: ::core::ffi::c_double,
    pub guidance_entry_plan_taem_speed: ::core::ffi::c_double,
    pub guidance_entry_plan_taem_energy_error: ::core::ffi::c_double,
    pub guidance_entry_reversal_time_remaining: ::core::ffi::c_double,
    pub guidance_entry_reversal_range: ::core::ffi::c_double,
    pub guidance_entry_reversal_sign: ::core::ffi::c_double,
    pub guidance_entry_plan_predicted_reversals: ::core::ffi::c_uint,
    pub guidance_entry_feedback: EntryGuidanceFeedback,
    pub guidance_entry_exec: EntryExecTelemetry,
    pub guidance_taem_exec: TaemExecTelemetry,
    pub guidance_taem_energy_law_active: bool,
    pub guidance_taem_energy_drag_level: ::core::ffi::c_double,
    pub guidance_taem_energy_path_to_go: ::core::ffi::c_double,
    pub guidance_taem_energy_required_path: ::core::ffi::c_double,
    pub guidance_hac_captured: bool,
    pub guidance_hac_completed: bool,
    pub guidance_hac_progress_valid: bool,
    pub guidance_hac_transition_active: bool,
    pub guidance_terminal_prediction_valid: bool,
    pub guidance_terminal_candidate_valid: bool,
    pub guidance_terminal_committed: bool,
    pub guidance_terminal_candidate_radius: ::core::ffi::c_double,
    pub guidance_terminal_candidate_altitude: ::core::ffi::c_double,
    pub guidance_terminal_candidate_speed: ::core::ffi::c_double,
    pub guidance_terminal_candidate_kind: ::core::ffi::c_int,
    pub guidance_terminal_candidate_geometry_degraded: bool,
    pub guidance_terminal_candidate_energy_degraded: bool,
    pub guidance_terminal_candidate_shell_degraded: bool,
    pub guidance_terminal_candidate_side: ::core::ffi::c_double,
    pub guidance_terminal_candidate_slope: ::core::ffi::c_double,
    pub guidance_terminal_candidate_curve_length: ::core::ffi::c_double,
    pub guidance_terminal_candidate_lead_length: ::core::ffi::c_double,
    pub guidance_terminal_candidate_arc_remaining: ::core::ffi::c_double,
    pub guidance_terminal_candidate_final_distance: ::core::ffi::c_double,
    pub guidance_terminal_candidate_quality: ::core::ffi::c_double,
    pub guidance_terminal_candidate_arrival_ut: ::core::ffi::c_double,
    pub guidance_terminal_candidate_course: ::core::ffi::c_double,
    pub guidance_terminal_candidate_peak_lateral: ::core::ffi::c_double,
    pub guidance_terminal_candidate_peak_rate_ratio: ::core::ffi::c_double,
    pub guidance_terminal_candidate_exit_speed: ::core::ffi::c_double,
    pub guidance_terminal_candidate_tracking_error: ::core::ffi::c_double,
    pub guidance_terminal_candidate_tracking_course_error: ::core::ffi::c_double,
    pub guidance_terminal_candidate_tracking_endpoint_error: ::core::ffi::c_double,
    pub guidance_terminal_candidate_tracking_margin: ::core::ffi::c_double,
    pub guidance_terminal_candidate_tracking_evaluated: bool,
    pub guidance_terminal_candidate_path_degraded: bool,
    pub guidance_terminal_candidate_control_degraded: bool,
    pub guidance_terminal_candidate_rate_degraded: bool,
    pub guidance_terminal_candidate_end_degraded: bool,
    pub guidance_terminal_reference_fpa: ::core::ffi::c_double,
    pub guidance_terminal_mix: ::core::ffi::c_double,
    pub guidance_hac_remaining: ::core::ffi::c_double,
    pub guidance_hac_radius: ::core::ffi::c_double,
    pub guidance_minimum_turn_radius: ::core::ffi::c_double,
    pub guidance_hac_transition_progress: ::core::ffi::c_double,
    pub guidance_commanded_course_rate_estimate: ::core::ffi::c_double,
    pub guidance_hac_transition_points: [::core::ffi::c_double; 8],
    pub guidance_hac_transition_heading_cone: bool,
    pub guidance_hac_transition_lead_curve: bool,
    pub guidance_hac_transition_lead_points: [::core::ffi::c_double; 6],
    pub guidance_hac_transition_cone_center: [::core::ffi::c_double; 2],
    pub guidance_hac_transition_cone_start_angle: ::core::ffi::c_double,
    pub guidance_hac_transition_cone_end_angle: ::core::ffi::c_double,
    pub guidance_hac_transition_cone_arc_length: ::core::ffi::c_double,
    pub guidance_hac_circuit_count: ::core::ffi::c_uint,
    pub guidance_hac_circuit_slope: ::core::ffi::c_double,
    pub guidance_terminal_reentry_after_ut: ::core::ffi::c_double,
    pub decision_reason: [::core::ffi::c_char; 384],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryControlPlan {
    pub valid: bool,
    pub terminal_ready: bool,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub parent_plan_id: uint64_t,
    pub parent_plan_version: uint64_t,
    pub planned_ut: ::core::ffi::c_double,
    pub target_bank: ::core::ffi::c_double,
    pub target_aoa: ::core::ffi::c_double,
    pub target_heading: ::core::ffi::c_double,
    pub bank_cap: ::core::ffi::c_double,
    pub target_turn_radius: ::core::ffi::c_double,
    pub segment_duration: ::core::ffi::c_double,
    pub cost: ::core::ffi::c_double,
    pub taem_range_error: ::core::ffi::c_double,
    pub taem_speed: ::core::ffi::c_double,
    pub taem_energy_error: ::core::ffi::c_double,
    pub closest_distance: ::core::ffi::c_double,
    pub has_planned_reversal: bool,
    pub planned_reversal_is_final: bool,
    pub final_heading_lock: bool,
    pub planned_reversal_ut: ::core::ffi::c_double,
    pub planned_reversal_range: ::core::ffi::c_double,
    pub planned_reversal_sign: ::core::ffi::c_double,
    pub predicted_reversals: ::core::ffi::c_uint,
}
pub type C2RustUnnamed = ::core::ffi::c_uint;
pub const PLANNER_LOG_EVENT_TIME_REWIND: C2RustUnnamed = 32;
pub const PLANNER_LOG_EVENT_EXPLICIT: C2RustUnnamed = 16;
pub const PLANNER_LOG_EVENT_PREDICTOR_DISCONTINUITY: C2RustUnnamed = 8;
pub const PLANNER_LOG_EVENT_CONSTRAINT: C2RustUnnamed = 4;
pub const PLANNER_LOG_EVENT_PLAN: C2RustUnnamed = 2;
pub const PLANNER_LOG_EVENT_PHASE: C2RustUnnamed = 1;
pub const PLANNER_LOG_EVENT_NONE: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlannerLogPolicyState {
    pub initialized: bool,
    pub last_plan_valid: bool,
    pub last_constraint_active: bool,
    pub last_phase: GuidancePhase,
    pub last_plan: EntryControlPlan,
    pub last_scalar_ut: ::core::ffi::c_double,
    pub last_trajectory_ut: ::core::ffi::c_double,
    pub last_ut: ::core::ffi::c_double,
    pub next_plan_id: uint64_t,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub parent_plan_id: uint64_t,
    pub parent_plan_version: uint64_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlannerLogDecision {
    pub emit_scalar: bool,
    pub emit_trajectory: bool,
    pub forced: bool,
    pub durable: bool,
    pub phase_changed: bool,
    pub plan_changed: bool,
    pub constraint_changed: bool,
    pub predictor_discontinuity: bool,
    pub time_rewind: bool,
    pub executable_lineage: bool,
    pub event_flags: ::core::ffi::c_uint,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub parent_plan_id: uint64_t,
    pub parent_plan_version: uint64_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlannerLog {
    pub file: *mut FILE,
    pub directory: [::core::ffi::c_char; 1300],
    pub stamp: [::core::ffi::c_char; 96],
    pub vessel: [::core::ffi::c_char; 160],
    pub session_id: [::core::ffi::c_char; 192],
    pub current_path: [::core::ffi::c_char; 1700],
    pub native_build: [::core::ffi::c_char; 96],
    pub vehicle_peer_path: [::core::ffi::c_char; 1700],
    pub vehicle_peer_available: bool,
    pub configuration: LandingConfiguration,
    pub policy: PlannerLogPolicyState,
    pub max_segment_bytes: size_t,
    pub segment_bytes: size_t,
    pub segment_index: ::core::ffi::c_uint,
    pub record_sequence: uint64_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlannerLogSample {
    pub tick_sequence: uint64_t,
    pub ut: ::core::ffi::c_double,
    pub phase: GuidancePhase,
    pub telemetry: *const Telemetry,
    pub snapshot: *const LandingSnapshot,
    pub plan: *const EntryControlPlan,
    pub raw_prediction: *const Trajectory,
    pub published_prediction: *const Trajectory,
    pub calibration: *const TrajectoryCalibrationModel,
    pub envelope: *const AerodynamicEnvelope,
    pub constraint_active: bool,
    pub predictor_discontinuity: bool,
    pub explicit_force: bool,
    pub event_reason: *const ::core::ffi::c_char,
}
pub const EOF: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
pub const PLANNER_LOG_SCALAR_INTERVAL_SECONDS: ::core::ffi::c_double = 2.0f64;
pub const PLANNER_LOG_TRAJECTORY_INTERVAL_SECONDS: ::core::ffi::c_double = 12.0f64;
pub const PLANNER_LOG_DEFAULT_MAX_SEGMENT_BYTES: ::core::ffi::c_uint = (64
    as ::core::ffi::c_uint)
    .wrapping_mul(1024 as ::core::ffi::c_uint)
    .wrapping_mul(1024 as ::core::ffi::c_uint);
pub const PLANNER_LOG_MAX_TRAJECTORY_POINTS: ::core::ffi::c_uint = 160
    as ::core::ffi::c_uint;
unsafe extern "C" fn materially_different(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut eps: ::core::ffi::c_double,
) -> bool {
    if a.is_finite() as i32 == 0 || b.is_finite() as i32 == 0 {
        return a.is_finite() as i32 != b.is_finite() as i32;
    }
    return fabs(a - b) > eps;
}
unsafe extern "C" fn planner_plan_changed(
    mut a: *const EntryControlPlan,
    mut b: *const EntryControlPlan,
) -> bool {
    if a.is_null() || b.is_null() {
        return a != b;
    }
    if (*a).valid as ::core::ffi::c_int != (*b).valid as ::core::ffi::c_int {
        return true_0 != 0;
    }
    if !(*a).valid {
        return false_0 != 0;
    }
    return (*a).plan_id != (*b).plan_id || (*a).plan_version != (*b).plan_version
        || (*a).parent_plan_id != (*b).parent_plan_id
        || (*a).parent_plan_version != (*b).parent_plan_version
        || (*a).terminal_ready as ::core::ffi::c_int
            != (*b).terminal_ready as ::core::ffi::c_int
        || (*a).has_planned_reversal as ::core::ffi::c_int
            != (*b).has_planned_reversal as ::core::ffi::c_int
        || (*a).planned_reversal_is_final as ::core::ffi::c_int
            != (*b).planned_reversal_is_final as ::core::ffi::c_int
        || (*a).final_heading_lock as ::core::ffi::c_int
            != (*b).final_heading_lock as ::core::ffi::c_int
        || materially_different((*a).target_bank, (*b).target_bank, 0.25f64)
            as ::core::ffi::c_int != 0
        || materially_different((*a).target_aoa, (*b).target_aoa, 0.20f64)
            as ::core::ffi::c_int != 0
        || materially_different(
            norm_signed_deg((*a).target_heading - (*b).target_heading),
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0.50f64,
        ) as ::core::ffi::c_int != 0
        || materially_different((*a).segment_duration, (*b).segment_duration, 0.75f64)
            as ::core::ffi::c_int != 0
        || materially_different(
            (*a).planned_reversal_ut,
            (*b).planned_reversal_ut,
            0.25f64,
        ) as ::core::ffi::c_int != 0
        || materially_different(
            (*a).planned_reversal_sign,
            (*b).planned_reversal_sign,
            0.1f64,
        ) as ::core::ffi::c_int != 0
        || materially_different((*a).taem_range_error, (*b).taem_range_error, 250.0f64)
            as ::core::ffi::c_int != 0
        || materially_different((*a).taem_speed, (*b).taem_speed, 2.0f64)
            as ::core::ffi::c_int != 0
        || materially_different(
            (*a).taem_energy_error,
            (*b).taem_energy_error,
            2500.0f64,
        ) as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_policy_init(mut s: *mut PlannerLogPolicyState) {
    if s.is_null() {
        return;
    }
    memset(
        s as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PlannerLogPolicyState>() as size_t,
    );
    (*s).last_ut = -DBL_MAX;
    (*s).last_trajectory_ut = (*s).last_ut;
    (*s).last_scalar_ut = (*s).last_trajectory_ut;
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_policy_step(
    mut s: *mut PlannerLogPolicyState,
    mut ut: ::core::ffi::c_double,
    mut phase: GuidancePhase,
    mut plan: *const EntryControlPlan,
    mut constraint_active: bool,
    mut predictor_discontinuity: bool,
    mut explicit_force: bool,
) -> PlannerLogDecision {
    let mut d: PlannerLogDecision = PlannerLogDecision {
        emit_scalar: false,
        emit_trajectory: false,
        forced: false,
        durable: false,
        phase_changed: false,
        plan_changed: false,
        constraint_changed: false,
        predictor_discontinuity: false,
        time_rewind: false,
        executable_lineage: false,
        event_flags: 0,
        plan_id: 0,
        plan_version: 0,
        parent_plan_id: 0,
        parent_plan_version: 0,
    };
    memset(
        &raw mut d as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PlannerLogDecision>() as size_t,
    );
    if s.is_null() {
        return d;
    }
    let mut plan_valid: bool = !plan.is_null()
        && (*plan).valid as ::core::ffi::c_int != 0;
    let mut executable_lineage: bool = plan_valid as ::core::ffi::c_int != 0
        && (*plan).plan_id != 0 as uint64_t;
    d.executable_lineage = executable_lineage;
    d.time_rewind = (*s).initialized as ::core::ffi::c_int != 0
        && ut.is_finite() as i32 != 0 && ut + 1e-6f64 < (*s).last_ut;
    d.phase_changed = !(*s).initialized
        || phase as ::core::ffi::c_uint != (*s).last_phase as ::core::ffi::c_uint;
    d.plan_changed = !(*s).initialized
        || plan_valid as ::core::ffi::c_int != (*s).last_plan_valid as ::core::ffi::c_int
        || plan_valid as ::core::ffi::c_int != 0
            && planner_plan_changed(plan, &raw mut (*s).last_plan) as ::core::ffi::c_int
                != 0;
    d.constraint_changed = !(*s).initialized
        || constraint_active as ::core::ffi::c_int
            != (*s).last_constraint_active as ::core::ffi::c_int;
    d.predictor_discontinuity = predictor_discontinuity;
    if d.phase_changed {
        d.event_flags
            |= PLANNER_LOG_EVENT_PHASE as ::core::ffi::c_int as ::core::ffi::c_uint;
    }
    if d.plan_changed {
        d.event_flags
            |= PLANNER_LOG_EVENT_PLAN as ::core::ffi::c_int as ::core::ffi::c_uint;
    }
    if d.constraint_changed {
        d.event_flags
            |= PLANNER_LOG_EVENT_CONSTRAINT as ::core::ffi::c_int as ::core::ffi::c_uint;
    }
    if predictor_discontinuity {
        d.event_flags
            |= PLANNER_LOG_EVENT_PREDICTOR_DISCONTINUITY as ::core::ffi::c_int
                as ::core::ffi::c_uint;
    }
    if explicit_force {
        d.event_flags
            |= PLANNER_LOG_EVENT_EXPLICIT as ::core::ffi::c_int as ::core::ffi::c_uint;
    }
    if d.time_rewind {
        d.event_flags
            |= PLANNER_LOG_EVENT_TIME_REWIND as ::core::ffi::c_int
                as ::core::ffi::c_uint;
    }
    d.forced = d.event_flags != 0 as ::core::ffi::c_uint;
    d.durable = d.phase_changed as ::core::ffi::c_int != 0
        || d.plan_changed as ::core::ffi::c_int != 0
        || explicit_force as ::core::ffi::c_int != 0
        || d.time_rewind as ::core::ffi::c_int != 0;
    if executable_lineage {
        (*s).plan_id = (*plan).plan_id;
        (*s).plan_version = (*plan).plan_version;
        (*s).parent_plan_id = (*plan).parent_plan_id;
        (*s).parent_plan_version = (*plan).parent_plan_version;
        if (*plan).plan_id > (*s).next_plan_id {
            (*s).next_plan_id = (*plan).plan_id;
        }
    } else if plan_valid {
        let mut new_family: bool = !(*s).initialized || !(*s).last_plan_valid
            || d.phase_changed as ::core::ffi::c_int != 0
            || d.time_rewind as ::core::ffi::c_int != 0
            || (*s).last_plan.plan_id != 0 as uint64_t;
        if new_family {
            let mut old_id: uint64_t = if (*s).last_plan_valid as ::core::ffi::c_int != 0
            {
                (*s).plan_id
            } else {
                0 as uint64_t
            };
            let mut old_version: uint64_t = if (*s).last_plan_valid as ::core::ffi::c_int
                != 0
            {
                (*s).plan_version
            } else {
                0 as uint64_t
            };
            (*s).parent_plan_id = old_id;
            (*s).parent_plan_version = old_version;
            (*s).next_plan_id = (*s).next_plan_id.wrapping_add(1);
            (*s).plan_id = (*s).next_plan_id;
            (*s).plan_version = 1 as uint64_t;
        } else if d.plan_changed {
            (*s).parent_plan_id = (*s).plan_id;
            (*s).parent_plan_version = (*s).plan_version;
            (*s).plan_version = (*s).plan_version.wrapping_add(1);
        }
    } else {
        (*s).parent_plan_version = 0 as uint64_t;
        (*s).parent_plan_id = (*s).parent_plan_version;
        (*s).plan_version = (*s).parent_plan_id;
        (*s).plan_id = (*s).plan_version;
    }
    d.plan_id = (*s).plan_id;
    d.plan_version = (*s).plan_version;
    d.parent_plan_id = (*s).parent_plan_id;
    d.parent_plan_version = (*s).parent_plan_version;
    if d.time_rewind {
        (*s).last_trajectory_ut = -DBL_MAX;
        (*s).last_scalar_ut = (*s).last_trajectory_ut;
    }
    d.emit_scalar = d.forced as ::core::ffi::c_int != 0
        || (*s).last_scalar_ut.is_finite() as i32 == 0
        || ut - (*s).last_scalar_ut >= PLANNER_LOG_SCALAR_INTERVAL_SECONDS - 1e-9f64;
    d.emit_trajectory = d.phase_changed as ::core::ffi::c_int != 0
        || d.plan_changed as ::core::ffi::c_int != 0
        || d.constraint_changed as ::core::ffi::c_int != 0
        || d.predictor_discontinuity as ::core::ffi::c_int != 0
        || explicit_force as ::core::ffi::c_int != 0
        || d.time_rewind as ::core::ffi::c_int != 0
        || (*s).last_trajectory_ut.is_finite() as i32 == 0
        || ut - (*s).last_trajectory_ut
            >= PLANNER_LOG_TRAJECTORY_INTERVAL_SECONDS - 1e-9f64;
    if d.emit_scalar {
        (*s).last_scalar_ut = ut;
    }
    if d.emit_trajectory {
        (*s).last_trajectory_ut = ut;
    }
    (*s).initialized = true_0 != 0;
    (*s).last_phase = phase;
    (*s).last_plan_valid = plan_valid;
    (*s).last_constraint_active = constraint_active;
    (*s).last_ut = ut;
    if plan_valid {
        (*s).last_plan = *plan;
    } else {
        memset(
            &raw mut (*s).last_plan as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<EntryControlPlan>() as size_t,
        );
    }
    return d;
}
unsafe extern "C" fn phase_name(mut p: GuidancePhase) -> *const ::core::ffi::c_char {
    static mut n: [*const ::core::ffi::c_char; 19] = [
        b"idle\0" as *const u8 as *const ::core::ffi::c_char,
        b"planning\0" as *const u8 as *const ::core::ffi::c_char,
        b"calibration\0" as *const u8 as *const ::core::ffi::c_char,
        b"coast\0" as *const u8 as *const ::core::ffi::c_char,
        b"burnSetup\0" as *const u8 as *const ::core::ffi::c_char,
        b"deorbitBurn\0" as *const u8 as *const ::core::ffi::c_char,
        b"entryInterface\0" as *const u8 as *const ::core::ffi::c_char,
        b"entryEnergy\0" as *const u8 as *const ::core::ffi::c_char,
        b"taem\0" as *const u8 as *const ::core::ffi::c_char,
        b"headingAlignment\0" as *const u8 as *const ::core::ffi::c_char,
        b"final\0" as *const u8 as *const ::core::ffi::c_char,
        b"flare\0" as *const u8 as *const ::core::ffi::c_char,
        b"touchdown\0" as *const u8 as *const ::core::ffi::c_char,
        b"rollout\0" as *const u8 as *const ::core::ffi::c_char,
        b"complete\0" as *const u8 as *const ::core::ffi::c_char,
        b"paused\0" as *const u8 as *const ::core::ffi::c_char,
        b"abort\0" as *const u8 as *const ::core::ffi::c_char,
        b"fault\0" as *const u8 as *const ::core::ffi::c_char,
        b"attitudeRecovery\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    return if (p as ::core::ffi::c_uint as usize)
        < (::core::mem::size_of::<[*const ::core::ffi::c_char; 19]>() as usize)
            .wrapping_div(::core::mem::size_of::<*const ::core::ffi::c_char>() as usize)
    {
        n[p as usize]
    } else {
        b"unknown\0" as *const u8 as *const ::core::ffi::c_char
    };
}
unsafe extern "C" fn key(mut w: *mut JsonWriter, mut k: *const ::core::ffi::c_char) {
    jw_string(w, k);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn comma(mut w: *mut JsonWriter) {
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn num(
    mut w: *mut JsonWriter,
    mut k: *const ::core::ffi::c_char,
    mut v: ::core::ffi::c_double,
) {
    key(w, k);
    if v.is_finite() as i32 != 0 {
        jw_number(w, v);
    } else {
        jw_null(w);
    }
    comma(w);
}
unsafe extern "C" fn boolean(
    mut w: *mut JsonWriter,
    mut k: *const ::core::ffi::c_char,
    mut v: bool,
) {
    key(w, k);
    jw_bool(w, v);
    comma(w);
}
unsafe extern "C" fn integer(
    mut w: *mut JsonWriter,
    mut k: *const ::core::ffi::c_char,
    mut v: ::core::ffi::c_longlong,
) {
    key(w, k);
    jw_integer(w, v);
    comma(w);
}
unsafe extern "C" fn plan_json(mut w: *mut JsonWriter, mut p: *const EntryControlPlan) {
    if p.is_null() || !(*p).valid {
        jw_null(w);
        return;
    }
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    boolean(w, b"valid\0" as *const u8 as *const ::core::ffi::c_char, (*p).valid);
    boolean(
        w,
        b"terminalReady\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).terminal_ready,
    );
    integer(
        w,
        b"planId\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).plan_id as ::core::ffi::c_longlong,
    );
    integer(
        w,
        b"planVersion\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).plan_version as ::core::ffi::c_longlong,
    );
    integer(
        w,
        b"parentPlanId\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).parent_plan_id as ::core::ffi::c_longlong,
    );
    integer(
        w,
        b"parentPlanVersion\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).parent_plan_version as ::core::ffi::c_longlong,
    );
    num(w, b"plannedUT\0" as *const u8 as *const ::core::ffi::c_char, (*p).planned_ut);
    num(w, b"targetBank\0" as *const u8 as *const ::core::ffi::c_char, (*p).target_bank);
    num(w, b"targetAoA\0" as *const u8 as *const ::core::ffi::c_char, (*p).target_aoa);
    num(
        w,
        b"targetHeading\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).target_heading,
    );
    num(w, b"bankCap\0" as *const u8 as *const ::core::ffi::c_char, (*p).bank_cap);
    num(
        w,
        b"turnRadius\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).target_turn_radius,
    );
    num(
        w,
        b"segmentDuration\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).segment_duration,
    );
    num(w, b"plannerCost\0" as *const u8 as *const ::core::ffi::c_char, (*p).cost);
    num(
        w,
        b"taemRangeError\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).taem_range_error,
    );
    num(w, b"taemSpeed\0" as *const u8 as *const ::core::ffi::c_char, (*p).taem_speed);
    num(
        w,
        b"taemEnergyError\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).taem_energy_error,
    );
    num(
        w,
        b"closestDistance\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).closest_distance,
    );
    boolean(
        w,
        b"taemCaptureReady\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).terminal_ready,
    );
    key(w, b"taemCaptureVeto\0" as *const u8 as *const ::core::ffi::c_char);
    if (*p).terminal_ready {
        jw_integer(w, 0 as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    comma(w);
    key(w, b"taemCaptureVetoReasons\0" as *const u8 as *const ::core::ffi::c_char);
    jw_raw(
        w,
        if (*p).terminal_ready as ::core::ffi::c_int != 0 {
            b"[]\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"[\"not-ready\"]\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    comma(w);
    boolean(
        w,
        b"hasPlannedReversal\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).has_planned_reversal,
    );
    boolean(
        w,
        b"plannedReversalFinal\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).planned_reversal_is_final,
    );
    boolean(
        w,
        b"finalHeadingLock\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).final_heading_lock,
    );
    num(
        w,
        b"plannedReversalUT\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).planned_reversal_ut,
    );
    num(
        w,
        b"plannedReversalRange\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).planned_reversal_range,
    );
    num(
        w,
        b"plannedReversalSign\0" as *const u8 as *const ::core::ffi::c_char,
        (*p).planned_reversal_sign,
    );
    key(w, b"predictedReversals\0" as *const u8 as *const ::core::ffi::c_char);
    jw_integer(w, (*p).predicted_reversals as ::core::ffi::c_longlong);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn trajectory_sample_json(
    mut w: *mut JsonWriter,
    mut t: *const Trajectory,
) {
    if t.is_null() || (*t).count == 0 {
        jw_raw(w, b"[]\0" as *const u8 as *const ::core::ffi::c_char);
        return;
    }
    let mut r: Trajectory = Trajectory {
        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
        count: 0,
        capacity: 0,
    };
    trajectory_init(&raw mut r);
    let mut max: size_t = PLANNER_LOG_MAX_TRAJECTORY_POINTS as size_t;
    let mut step: size_t = if (*t).count <= max {
        1 as size_t
    } else {
        (*t)
            .count
            .wrapping_sub(1 as size_t)
            .wrapping_div(max.wrapping_sub(1 as size_t))
            .wrapping_add(
                (if (*t)
                    .count
                    .wrapping_sub(1 as size_t)
                    .wrapping_rem(max.wrapping_sub(1 as size_t)) != 0
                {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                }) as size_t,
            )
    };
    let mut i: size_t = 0 as size_t;
    while i < (*t).count {
        trajectory_append(&raw mut r, *(*t).points.offset(i as isize));
        i = i.wrapping_add(step);
    }
    if r.count == 0 as size_t
        || (*r.points.offset(r.count.wrapping_sub(1 as size_t) as isize)).ut
            != (*(*t).points.offset((*t).count.wrapping_sub(1 as size_t) as isize)).ut
    {
        trajectory_append(
            &raw mut r,
            *(*t).points.offset((*t).count.wrapping_sub(1 as size_t) as isize),
        );
    }
    trajectory_json(w, &raw mut r);
    trajectory_clear(&raw mut r);
}
unsafe extern "C" fn session_start_json(
    mut w: *mut JsonWriter,
    mut l: *const PlannerLog,
) {
    let mut campaign_identity: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_CAMPAIGN_IDENTITY\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    integer(
        w,
        b"schemaVersion\0" as *const u8 as *const ::core::ffi::c_char,
        5 as ::core::ffi::c_longlong,
    );
    integer(
        w,
        b"sessionSchemaVersion\0" as *const u8 as *const ::core::ffi::c_char,
        1 as ::core::ffi::c_longlong,
    );
    key(w, b"recordType\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, b"sessionStart\0" as *const u8 as *const ::core::ffi::c_char);
    comma(w);
    key(w, b"stream\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, b"planner\0" as *const u8 as *const ::core::ffi::c_char);
    comma(w);
    key(w, b"startedUTC\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, &raw const (*l).stamp as *const ::core::ffi::c_char);
    comma(w);
    key(w, b"sessionId\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, &raw const (*l).session_id as *const ::core::ffi::c_char);
    comma(w);
    key(w, b"campaignIdentity\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        w,
        if !campaign_identity.is_null() && *campaign_identity as ::core::ffi::c_int != 0
        {
            campaign_identity
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    comma(w);
    key(w, b"vessel\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, &raw const (*l).vessel as *const ::core::ffi::c_char);
    comma(w);
    key(w, b"peer\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    key(w, b"stream\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, b"vehicle\0" as *const u8 as *const ::core::ffi::c_char);
    comma(w);
    boolean(
        w,
        b"available\0" as *const u8 as *const ::core::ffi::c_char,
        (*l).vehicle_peer_available,
    );
    key(w, b"path\0" as *const u8 as *const ::core::ffi::c_char);
    if (*l).vehicle_peer_path[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*l).vehicle_peer_path as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    comma(w);
    key(w, b"runtime\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    key(w, b"nativeBuild\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(w, &raw const (*l).native_build as *const ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    comma(w);
    key(w, b"modelIdentity\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    key(w, b"vehicleModelId\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        w,
        &raw const (*l).configuration.vehicle.model_id as *const ::core::ffi::c_char,
    );
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    comma(w);
    integer(
        w,
        b"rotationIndex\0" as *const u8 as *const ::core::ffi::c_char,
        (*l).segment_index as ::core::ffi::c_longlong,
    );
    key(w, b"encoding\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        w,
        b"lower-rate exhaustive planner summaries; event-forced <=160-point selected raw/published paths; event-rate <=24-point candidate paths\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    comma(w);
    key(w, b"configuration\0" as *const u8 as *const ::core::ffi::c_char);
    landing_configuration_json(w, &raw const (*l).configuration);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn planner_log_durable_flush(mut l: *mut PlannerLog) {
    if l.is_null() || (*l).file.is_null() {
        return;
    }
    fflush((*l).file);
    let mut fd: ::core::ffi::c_int = fileno((*l).file);
    if fd >= 0 as ::core::ffi::c_int {
        fsync(fd);
    }
}
unsafe extern "C" fn direct_write(
    mut l: *mut PlannerLog,
    mut w: *mut JsonWriter,
) -> bool {
    if l.is_null() || (*l).file.is_null() || w.is_null()
        || (*w).failed as ::core::ffi::c_int != 0 || (*w).data.is_null()
    {
        return false_0 != 0;
    }
    let mut n: size_t = (*w).length.wrapping_add(1 as size_t);
    if fwrite(
        (*w).data as *const ::core::ffi::c_void,
        1 as size_t,
        (*w).length,
        (*l).file,
    ) as size_t != (*w).length || fputc('\n' as i32, (*l).file) == EOF
    {
        return false_0 != 0;
    }
    (*l).segment_bytes = (*l).segment_bytes.wrapping_add(n);
    fflush((*l).file);
    return true_0 != 0;
}
unsafe extern "C" fn open_segment(mut l: *mut PlannerLog) -> bool {
    if l.is_null() {
        return false_0 != 0;
    }
    if !(*l).file.is_null() {
        fflush((*l).file);
        let mut fd: ::core::ffi::c_int = fileno((*l).file);
        if fd >= 0 as ::core::ffi::c_int {
            fsync(fd);
        }
        fclose((*l).file);
        (*l).file = ::core::ptr::null_mut::<FILE>();
    }
    if (*l).segment_index == 0 as ::core::ffi::c_uint {
        snprintf(
            &raw mut (*l).current_path as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1700]>() as size_t,
            b"%s/%s-%s-planner.jsonl\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut (*l).directory as *mut ::core::ffi::c_char,
            &raw mut (*l).stamp as *mut ::core::ffi::c_char,
            &raw mut (*l).vessel as *mut ::core::ffi::c_char,
        );
    } else {
        snprintf(
            &raw mut (*l).current_path as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1700]>() as size_t,
            b"%s/%s-%s-planner-%03u.jsonl\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut (*l).directory as *mut ::core::ffi::c_char,
            &raw mut (*l).stamp as *mut ::core::ffi::c_char,
            &raw mut (*l).vessel as *mut ::core::ffi::c_char,
            (*l).segment_index,
        );
    }
    (*l).file = fopen(
        &raw mut (*l).current_path as *mut ::core::ffi::c_char,
        b"w\0" as *const u8 as *const ::core::ffi::c_char,
    ) as *mut FILE;
    if (*l).file.is_null() {
        return false_0 != 0;
    }
    (*l).segment_bytes = 0 as size_t;
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    session_start_json(&raw mut w, l);
    let mut ok: bool = direct_write(l, &raw mut w);
    jw_free(&raw mut w);
    return ok;
}
unsafe extern "C" fn write_record(
    mut l: *mut PlannerLog,
    mut w: *mut JsonWriter,
) -> bool {
    if l.is_null() || w.is_null() || (*w).failed as ::core::ffi::c_int != 0
        || (*w).data.is_null()
    {
        return false_0 != 0;
    }
    let mut n: size_t = (*w).length.wrapping_add(1 as size_t);
    if !(*l).file.is_null()
        && (*l).segment_bytes.wrapping_add(n) > (*l).max_segment_bytes
        && (*l).segment_bytes > 0 as size_t
    {
        (*l).segment_index = (*l).segment_index.wrapping_add(1);
        if !open_segment(l) {
            return false_0 != 0;
        }
    }
    return direct_write(l, w);
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_open(
    mut directory: *const ::core::ffi::c_char,
    mut stamp: *const ::core::ffi::c_char,
    mut vessel: *const ::core::ffi::c_char,
    mut session_id: *const ::core::ffi::c_char,
    mut native_build: *const ::core::ffi::c_char,
    mut vehicle_peer_path: *const ::core::ffi::c_char,
    mut vehicle_peer_available: bool,
    mut cfg: *const LandingConfiguration,
    mut max_bytes: size_t,
) -> *mut PlannerLog {
    if directory.is_null() || stamp.is_null() || vessel.is_null() || session_id.is_null()
        || native_build.is_null() || cfg.is_null()
    {
        return ::core::ptr::null_mut::<PlannerLog>();
    }
    let mut l: *mut PlannerLog = calloc(
        1 as size_t,
        ::core::mem::size_of::<PlannerLog>() as size_t,
    ) as *mut PlannerLog;
    if l.is_null() {
        return ::core::ptr::null_mut::<PlannerLog>();
    }
    snprintf(
        &raw mut (*l).directory as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1300]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        directory,
    );
    snprintf(
        &raw mut (*l).stamp as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        stamp,
    );
    snprintf(
        &raw mut (*l).vessel as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        vessel,
    );
    snprintf(
        &raw mut (*l).session_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        session_id,
    );
    snprintf(
        &raw mut (*l).native_build as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        native_build,
    );
    if !vehicle_peer_path.is_null() {
        snprintf(
            &raw mut (*l).vehicle_peer_path as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1700]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            vehicle_peer_path,
        );
    }
    (*l).vehicle_peer_available = vehicle_peer_available;
    (*l).configuration = *cfg;
    (*l).max_segment_bytes = (if max_bytes != 0 {
        fmax(max_bytes as ::core::ffi::c_double, 65536.0f64)
    } else {
        PLANNER_LOG_DEFAULT_MAX_SEGMENT_BYTES as ::core::ffi::c_double
    }) as size_t;
    planner_log_policy_init(&raw mut (*l).policy);
    if !open_segment(l) {
        planner_log_close(l);
        return ::core::ptr::null_mut::<PlannerLog>();
    }
    return l;
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_close(mut l: *mut PlannerLog) {
    if l.is_null() {
        return;
    }
    if !(*l).file.is_null() {
        fflush((*l).file);
        let mut fd: ::core::ffi::c_int = fileno((*l).file);
        if fd >= 0 as ::core::ffi::c_int {
            fsync(fd);
        }
        fclose((*l).file);
    }
    free(l as *mut ::core::ffi::c_void);
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_segment_index(
    mut l: *const PlannerLog,
) -> ::core::ffi::c_uint {
    return if !l.is_null() { (*l).segment_index } else { 0 as ::core::ffi::c_uint };
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_segment_bytes(mut l: *const PlannerLog) -> size_t {
    return if !l.is_null() { (*l).segment_bytes } else { 0 as size_t };
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_current_path(
    mut l: *const PlannerLog,
) -> *const ::core::ffi::c_char {
    return if !l.is_null() {
        &raw const (*l).current_path as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn planner_log_record(
    mut l: *mut PlannerLog,
    mut s: *const PlannerLogSample,
) -> PlannerLogDecision {
    let mut none: PlannerLogDecision = PlannerLogDecision {
        emit_scalar: false,
        emit_trajectory: false,
        forced: false,
        durable: false,
        phase_changed: false,
        plan_changed: false,
        constraint_changed: false,
        predictor_discontinuity: false,
        time_rewind: false,
        executable_lineage: false,
        event_flags: 0,
        plan_id: 0,
        plan_version: 0,
        parent_plan_id: 0,
        parent_plan_version: 0,
    };
    memset(
        &raw mut none as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PlannerLogDecision>() as size_t,
    );
    if l.is_null() || s.is_null() {
        return none;
    }
    let mut d: PlannerLogDecision = planner_log_policy_step(
        &raw mut (*l).policy,
        (*s).ut,
        (*s).phase,
        (*s).plan,
        (*s).constraint_active,
        (*s).predictor_discontinuity,
        (*s).explicit_force,
    );
    if !d.emit_scalar {
        return d;
    }
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
    integer(
        &raw mut w,
        b"schemaVersion\0" as *const u8 as *const ::core::ffi::c_char,
        5 as ::core::ffi::c_longlong,
    );
    key(&raw mut w, b"recordType\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, b"plannerSample\0" as *const u8 as *const ::core::ffi::c_char);
    comma(&raw mut w);
    key(&raw mut w, b"stream\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, b"planner\0" as *const u8 as *const ::core::ffi::c_char);
    comma(&raw mut w);
    key(&raw mut w, b"sessionId\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, &raw mut (*l).session_id as *mut ::core::ffi::c_char);
    comma(&raw mut w);
    (*l).record_sequence = (*l).record_sequence.wrapping_add(1);
    integer(
        &raw mut w,
        b"plannerRecordSequence\0" as *const u8 as *const ::core::ffi::c_char,
        (*l).record_sequence as ::core::ffi::c_longlong,
    );
    integer(
        &raw mut w,
        b"tickSequence\0" as *const u8 as *const ::core::ffi::c_char,
        (*s).tick_sequence as ::core::ffi::c_longlong,
    );
    num(&raw mut w, b"ut\0" as *const u8 as *const ::core::ffi::c_char, (*s).ut);
    key(&raw mut w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, phase_name((*s).phase));
    comma(&raw mut w);
    integer(
        &raw mut w,
        b"eventFlags\0" as *const u8 as *const ::core::ffi::c_char,
        d.event_flags as ::core::ffi::c_longlong,
    );
    boolean(
        &raw mut w,
        b"forced\0" as *const u8 as *const ::core::ffi::c_char,
        d.forced,
    );
    boolean(
        &raw mut w,
        b"durable\0" as *const u8 as *const ::core::ffi::c_char,
        d.durable,
    );
    boolean(
        &raw mut w,
        b"phaseChanged\0" as *const u8 as *const ::core::ffi::c_char,
        d.phase_changed,
    );
    boolean(
        &raw mut w,
        b"planChanged\0" as *const u8 as *const ::core::ffi::c_char,
        d.plan_changed,
    );
    boolean(
        &raw mut w,
        b"constraintChanged\0" as *const u8 as *const ::core::ffi::c_char,
        d.constraint_changed,
    );
    boolean(
        &raw mut w,
        b"predictorDiscontinuity\0" as *const u8 as *const ::core::ffi::c_char,
        d.predictor_discontinuity,
    );
    boolean(
        &raw mut w,
        b"trajectoryIncluded\0" as *const u8 as *const ::core::ffi::c_char,
        d.emit_trajectory,
    );
    key(&raw mut w, b"eventReason\0" as *const u8 as *const ::core::ffi::c_char);
    if !(*s).event_reason.is_null() && *(*s).event_reason as ::core::ffi::c_int != 0 {
        jw_string(&raw mut w, (*s).event_reason);
    } else {
        jw_null(&raw mut w);
    }
    comma(&raw mut w);
    key(&raw mut w, b"lineage\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
    integer(
        &raw mut w,
        b"planId\0" as *const u8 as *const ::core::ffi::c_char,
        d.plan_id as ::core::ffi::c_longlong,
    );
    integer(
        &raw mut w,
        b"version\0" as *const u8 as *const ::core::ffi::c_char,
        d.plan_version as ::core::ffi::c_longlong,
    );
    integer(
        &raw mut w,
        b"parentPlanId\0" as *const u8 as *const ::core::ffi::c_char,
        d.parent_plan_id as ::core::ffi::c_longlong,
    );
    integer(
        &raw mut w,
        b"parentVersion\0" as *const u8 as *const ::core::ffi::c_char,
        d.parent_plan_version as ::core::ffi::c_longlong,
    );
    key(&raw mut w, b"lineageSource\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        &raw mut w,
        if d.executable_lineage as ::core::ffi::c_int != 0 {
            b"executablePlan\0" as *const u8 as *const ::core::ffi::c_char
        } else if !(*s).plan.is_null() && (*(*s).plan).valid as ::core::ffi::c_int != 0 {
            b"loggerFallback\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"none\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    comma(&raw mut w);
    key(&raw mut w, b"currentPlan\0" as *const u8 as *const ::core::ffi::c_char);
    plan_json(&raw mut w, (*s).plan);
    comma(&raw mut w);
    key(&raw mut w, b"plannerTrace\0" as *const u8 as *const ::core::ffi::c_char);
    jw_null(&raw mut w);
    comma(&raw mut w);
    if !(*s).telemetry.is_null() {
        let mut t: *const Telemetry = (*s).telemetry;
        key(&raw mut w, b"constraints\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
        num(
            &raw mut w,
            b"dynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).dynamic_pressure,
        );
        num(
            &raw mut w,
            b"gForce\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).g_force,
        );
        num(
            &raw mut w,
            b"stallFraction\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).stall_fraction,
        );
        boolean(
            &raw mut w,
            b"stallFractionMeasured\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).stall_fraction_is_measured,
        );
        boolean(
            &raw mut w,
            b"active\0" as *const u8 as *const ::core::ffi::c_char,
            (*s).constraint_active,
        );
        key(
            &raw mut w,
            b"physicsUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_number(&raw mut w, (*t).physics_certified_uncertainty);
        jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
        comma(&raw mut w);
        key(
            &raw mut w,
            b"predictionAccuracy\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
        num(
            &raw mut w,
            b"altitudeResidual\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).trajectory_altitude_residual,
        );
        num(
            &raw mut w,
            b"speedResidual\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).trajectory_speed_residual,
        );
        num(
            &raw mut w,
            b"rangeResidual\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).trajectory_range_residual,
        );
        num(
            &raw mut w,
            b"rawPublishedDelta30s\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).predicted_raw_published_position_delta_30s,
        );
        num(
            &raw mut w,
            b"rawPublishedDelta60s\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).predicted_raw_published_position_delta_60s,
        );
        num(
            &raw mut w,
            b"rawPublishedDelta120s\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).predicted_raw_published_position_delta_120s,
        );
        num(
            &raw mut w,
            b"physicsRelativeUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
            (*t).predicted_physics_relative_uncertainty,
        );
        num(
            &raw mut w,
            b"terminalSurvivabilityStressScore\0" as *const u8
                as *const ::core::ffi::c_char,
            (*t).predicted_terminal_survivability_stress_score,
        );
        key(
            &raw mut w,
            b"uncertaintyScenarios\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_integer(
            &raw mut w,
            (*t).predicted_uncertainty_scenarios as ::core::ffi::c_longlong,
        );
        jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
        comma(&raw mut w);
    }
    if !(*s).calibration.is_null() {
        key(&raw mut w, b"calibration\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
        num(
            &raw mut w,
            b"densityScale\0" as *const u8 as *const ::core::ffi::c_char,
            (*(*s).calibration).density_scale,
        );
        num(
            &raw mut w,
            b"dragScale\0" as *const u8 as *const ::core::ffi::c_char,
            (*(*s).calibration).drag_scale,
        );
        num(
            &raw mut w,
            b"liftScale\0" as *const u8 as *const ::core::ffi::c_char,
            (*(*s).calibration).lift_scale,
        );
        num(
            &raw mut w,
            b"bankEffectiveness\0" as *const u8 as *const ::core::ffi::c_char,
            (*(*s).calibration).bank_effectiveness,
        );
        num(
            &raw mut w,
            b"confidence\0" as *const u8 as *const ::core::ffi::c_char,
            (*(*s).calibration).confidence,
        );
        key(&raw mut w, b"acceptedSamples\0" as *const u8 as *const ::core::ffi::c_char);
        jw_integer(
            &raw mut w,
            (*(*s).calibration).accepted_samples as ::core::ffi::c_longlong,
        );
        jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
        comma(&raw mut w);
    }
    if !(*s).envelope.is_null() {
        key(&raw mut w, b"aeroEnvelope\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(&raw mut w, '[' as i32 as ::core::ffi::c_char);
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < 4 as ::core::ffi::c_int {
            if i != 0 {
                comma(&raw mut w);
            }
            jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
            num(
                &raw mut w,
                b"liftToDrag\0" as *const u8 as *const ::core::ffi::c_char,
                (*(*s).envelope).regimes[i as usize].lift_to_drag,
            );
            num(
                &raw mut w,
                b"ballisticCoefficient\0" as *const u8 as *const ::core::ffi::c_char,
                (*(*s).envelope).regimes[i as usize].ballistic_coefficient,
            );
            key(&raw mut w, b"confidence\0" as *const u8 as *const ::core::ffi::c_char);
            jw_number(&raw mut w, (*(*s).envelope).regimes[i as usize].confidence);
            jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
            i += 1;
        }
        jw_char(&raw mut w, ']' as i32 as ::core::ffi::c_char);
        comma(&raw mut w);
    }
    if !(*s).snapshot.is_null() {
        let mut q: *const LandingSnapshot = (*s).snapshot;
        key(
            &raw mut w,
            b"terminalGeometry\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
        boolean(
            &raw mut w,
            b"terminalPredictionValid\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_prediction_valid,
        );
        boolean(
            &raw mut w,
            b"terminalCandidateValid\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_valid,
        );
        boolean(
            &raw mut w,
            b"terminalCommitted\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_committed,
        );
        num(
            &raw mut w,
            b"candidateRadius\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_radius,
        );
        num(
            &raw mut w,
            b"candidateAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_altitude,
        );
        num(
            &raw mut w,
            b"candidateSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_speed,
        );
        num(
            &raw mut w,
            b"candidateKind\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_kind as ::core::ffi::c_double,
        );
        boolean(
            &raw mut w,
            b"candidateGeometryDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_geometry_degraded,
        );
        boolean(
            &raw mut w,
            b"candidateEnergyDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_energy_degraded,
        );
        boolean(
            &raw mut w,
            b"candidateShellDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_shell_degraded,
        );
        num(
            &raw mut w,
            b"candidateSide\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_side,
        );
        num(
            &raw mut w,
            b"candidateSlope\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_slope,
        );
        num(
            &raw mut w,
            b"candidateCurveLength\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_curve_length,
        );
        num(
            &raw mut w,
            b"candidateLeadLength\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_lead_length,
        );
        num(
            &raw mut w,
            b"candidateArcRemaining\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_arc_remaining,
        );
        num(
            &raw mut w,
            b"candidateFinalDistance\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_final_distance,
        );
        num(
            &raw mut w,
            b"candidateQuality\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_quality,
        );
        num(
            &raw mut w,
            b"candidateArrivalUT\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_arrival_ut,
        );
        num(
            &raw mut w,
            b"candidateCourse\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_course,
        );
        num(
            &raw mut w,
            b"candidatePeakLateral\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_peak_lateral,
        );
        num(
            &raw mut w,
            b"candidatePeakRateRatio\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_peak_rate_ratio,
        );
        num(
            &raw mut w,
            b"candidateExitSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_exit_speed,
        );
        boolean(
            &raw mut w,
            b"candidateTrackingEvaluated\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_tracking_evaluated,
        );
        num(
            &raw mut w,
            b"candidateTrackingError\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_tracking_error,
        );
        num(
            &raw mut w,
            b"candidateTrackingCourseError\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_tracking_course_error,
        );
        num(
            &raw mut w,
            b"candidateTrackingEndpointError\0" as *const u8
                as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_tracking_endpoint_error,
        );
        num(
            &raw mut w,
            b"candidateTrackingMargin\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_tracking_margin,
        );
        boolean(
            &raw mut w,
            b"candidatePathDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_path_degraded,
        );
        boolean(
            &raw mut w,
            b"candidateControlDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_control_degraded,
        );
        boolean(
            &raw mut w,
            b"candidateRateDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_rate_degraded,
        );
        boolean(
            &raw mut w,
            b"candidateEndDegraded\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_candidate_end_degraded,
        );
        num(
            &raw mut w,
            b"referenceFPA\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_reference_fpa,
        );
        num(
            &raw mut w,
            b"mix\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_terminal_mix,
        );
        num(
            &raw mut w,
            b"hacRemaining\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_remaining,
        );
        num(
            &raw mut w,
            b"hacRadius\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_radius,
        );
        num(
            &raw mut w,
            b"minimumTurnRadius\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_minimum_turn_radius,
        );
        num(
            &raw mut w,
            b"hacTransitionProgress\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_progress,
        );
        boolean(
            &raw mut w,
            b"hacTransitionHeadingCone\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_heading_cone,
        );
        boolean(
            &raw mut w,
            b"hacTransitionLeadCurve\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_curve,
        );
        num(
            &raw mut w,
            b"hacLeadStartE\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_points[0 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacLeadStartN\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_points[1 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacLeadP1E\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_points[2 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacLeadP1N\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_points[3 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacLeadP2E\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_points[4 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacLeadP2N\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_lead_points[5 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacConeCenterE\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_cone_center[0 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacConeCenterN\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_cone_center[1 as ::core::ffi::c_int as usize],
        );
        num(
            &raw mut w,
            b"hacConeStartAngle\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_cone_start_angle,
        );
        num(
            &raw mut w,
            b"hacConeEndAngle\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_cone_end_angle,
        );
        num(
            &raw mut w,
            b"hacConeArcLength\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_hac_transition_cone_arc_length,
        );
        num(
            &raw mut w,
            b"commandedCourseRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_commanded_course_rate_estimate,
        );
        key(
            &raw mut w,
            b"taemTerminalEvaluationValid\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(&raw mut w, (*q).guidance_taem_exec.terminal_evaluation.valid);
        comma(&raw mut w);
        key(
            &raw mut w,
            b"taemTerminalPolicyFeasible\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(&raw mut w, (*q).guidance_taem_exec.terminal_evaluation.feasible);
        comma(&raw mut w);
        key(
            &raw mut w,
            b"taemTerminalBlockReason\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_string(
            &raw mut w,
            taem_terminal_block_reason_string(
                (*q).guidance_taem_exec.terminal_evaluation.block_reason,
            ),
        );
        comma(&raw mut w);
        key(
            &raw mut w,
            b"taemTerminalPathCommitted\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_bool(&raw mut w, (*q).guidance_taem_exec.terminal_contract.path_committed);
        comma(&raw mut w);
        num(
            &raw mut w,
            b"taemTerminalRangeMargin\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.range_margin,
        );
        num(
            &raw mut w,
            b"taemTerminalQMargin\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.dynamic_pressure_margin,
        );
        num(
            &raw mut w,
            b"taemTerminalSpeedbrakeQMargin\0" as *const u8
                as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.speedbrake_dynamic_pressure_margin,
        );
        num(
            &raw mut w,
            b"taemTerminalAltitudeMargin\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.altitude_margin,
        );
        num(
            &raw mut w,
            b"taemTerminalFPAMargin\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.flight_path_angle_margin,
        );
        num(
            &raw mut w,
            b"taemTerminalSpecificEnergyMargin\0" as *const u8
                as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.specific_energy_margin,
        );
        num(
            &raw mut w,
            b"taemTerminalResponseAvailable\0" as *const u8
                as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.response_time_available,
        );
        num(
            &raw mut w,
            b"taemTerminalResponseRequired\0" as *const u8 as *const ::core::ffi::c_char,
            (*q).guidance_taem_exec.terminal_contract.response_time_required,
        );
        key(
            &raw mut w,
            b"taemTerminalAttitudeResponseQualified\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        jw_bool(
            &raw mut w,
            (*q).guidance_taem_exec.terminal_contract.attitude_response_qualified,
        );
        comma(&raw mut w);
        key(&raw mut w, b"hacCircuitCount\0" as *const u8 as *const ::core::ffi::c_char);
        jw_integer(
            &raw mut w,
            (*q).guidance_hac_circuit_count as ::core::ffi::c_longlong,
        );
        jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
        comma(&raw mut w);
    }
    if d.emit_trajectory {
        key(&raw mut w, b"rawPrediction\0" as *const u8 as *const ::core::ffi::c_char);
        trajectory_sample_json(&raw mut w, (*s).raw_prediction);
        comma(&raw mut w);
        key(
            &raw mut w,
            b"publishedPrediction\0" as *const u8 as *const ::core::ffi::c_char,
        );
        trajectory_sample_json(&raw mut w, (*s).published_prediction);
        comma(&raw mut w);
    }
    key(&raw mut w, b"modelIdentity\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(&raw mut w, '{' as i32 as ::core::ffi::c_char);
    key(&raw mut w, b"vehicleModelId\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        &raw mut w,
        &raw mut (*l).configuration.vehicle.model_id as *mut ::core::ffi::c_char,
    );
    comma(&raw mut w);
    key(&raw mut w, b"plannerSchema\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        &raw mut w,
        b"entry-predictor-v3-shadow-parity\0" as *const u8 as *const ::core::ffi::c_char,
    );
    comma(&raw mut w);
    integer(
        &raw mut w,
        b"calibrationAcceptedSamples\0" as *const u8 as *const ::core::ffi::c_char,
        (if !(*s).calibration.is_null() {
            (*(*s).calibration).accepted_samples
        } else {
            0 as ::core::ffi::c_int
        }) as ::core::ffi::c_longlong,
    );
    num(
        &raw mut w,
        b"calibrationConfidence\0" as *const u8 as *const ::core::ffi::c_char,
        if !(*s).calibration.is_null() {
            (*(*s).calibration).confidence
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        },
    );
    let mut physics: *const VesselPhysicsModel = if !(*s).calibration.is_null() {
        (*(*s).calibration).physics
    } else {
        ::core::ptr::null::<VesselPhysicsModel>()
    };
    integer(
        &raw mut w,
        b"physicsSampleCount\0" as *const u8 as *const ::core::ffi::c_char,
        (if !physics.is_null() { (*physics).count } else { 0 as ::core::ffi::c_uint })
            as ::core::ffi::c_longlong,
    );
    integer(
        &raw mut w,
        b"physicsAcceptedSamples\0" as *const u8 as *const ::core::ffi::c_char,
        (if !physics.is_null() { (*physics).accepted } else { 0 as ::core::ffi::c_uint })
            as ::core::ffi::c_longlong,
    );
    integer(
        &raw mut w,
        b"physicsLiveObservations\0" as *const u8 as *const ::core::ffi::c_char,
        (if !physics.is_null() {
            (*physics).live_observations
        } else {
            0 as ::core::ffi::c_uint
        }) as ::core::ffi::c_longlong,
    );
    num(
        &raw mut w,
        b"physicsCertifiedConfidence\0" as *const u8 as *const ::core::ffi::c_char,
        if !physics.is_null() {
            (*physics).certified_confidence
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        },
    );
    key(
        &raw mut w,
        b"physicsCertifiedUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !physics.is_null() && (*physics).certified_uncertainty.is_finite() as i32 != 0 {
        jw_number(&raw mut w, (*physics).certified_uncertainty);
    } else {
        jw_null(&raw mut w);
    }
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    let mut wrote: bool = write_record(l, &raw mut w);
    if wrote as ::core::ffi::c_int != 0 && d.durable as ::core::ffi::c_int != 0 {
        planner_log_durable_flush(l);
    }
    jw_free(&raw mut w);
    return d;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const DBL_MAX: ::core::ffi::c_double = __DBL_MAX__;
pub const __DBL_MAX__: ::core::ffi::c_double = 1.7976931348623157e+308f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
