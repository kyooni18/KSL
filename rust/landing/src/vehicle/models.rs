use crate::sys::*;
extern "C" {
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
    fn json_integer(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        fallback: ::core::ffi::c_long,
    ) -> ::core::ffi::c_long;
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
    fn jw_char(w: *mut JsonWriter, c: ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_number(w: *mut JsonWriter, value: ::core::ffi::c_double);
    fn jw_integer(w: *mut JsonWriter, value: ::core::ffi::c_longlong);
    fn jw_bool(w: *mut JsonWriter, value: bool);
    fn jw_null(w: *mut JsonWriter);
    fn entry_phase_string(phase: EntryPhase) -> *const ::core::ffi::c_char;
    fn entry_transition_reason_string(
        reason: EntryTransitionReason,
    ) -> *const ::core::ffi::c_char;
    fn taem_phase_string(phase: TaemPhase) -> *const ::core::ffi::c_char;
    fn taem_transition_reason_string(
        reason: TaemTransitionReason,
    ) -> *const ::core::ffi::c_char;
    fn taem_recovery_reason_string(
        reason: TaemRecoveryReason,
    ) -> *const ::core::ffi::c_char;
    fn taem_terminal_block_reason_string(
        reason: TaemTerminalBlockReason,
    ) -> *const ::core::ffi::c_char;
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
pub const LANDER_ATMOSPHERE_SAMPLE_MAX: ::core::ffi::c_int = 1024 as ::core::ffi::c_int;
unsafe extern "C" fn copystr(
    mut dst: *mut ::core::ffi::c_char,
    mut cap: size_t,
    mut src: *const ::core::ffi::c_char,
) {
    if cap == 0 as size_t {
        return;
    }
    if src.is_null() {
        src = b"\0" as *const u8 as *const ::core::ffi::c_char;
    }
    snprintf(dst, cap, b"%s\0" as *const u8 as *const ::core::ffi::c_char, src);
}
#[no_mangle]
pub unsafe extern "C" fn connection_status_string(
    mut s: ConnectionStatus,
) -> *const ::core::ffi::c_char {
    static mut v: [*const ::core::ffi::c_char; 4] = [
        b"disconnected\0" as *const u8 as *const ::core::ffi::c_char,
        b"connecting\0" as *const u8 as *const ::core::ffi::c_char,
        b"connected\0" as *const u8 as *const ::core::ffi::c_char,
        b"failed\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    return if (s as ::core::ffi::c_uint) < 4 as ::core::ffi::c_uint {
        v[s as usize]
    } else {
        v[0 as ::core::ffi::c_int as usize]
    };
}
#[no_mangle]
pub unsafe extern "C" fn phase_string(
    mut p: GuidancePhase,
) -> *const ::core::ffi::c_char {
    static mut v: [*const ::core::ffi::c_char; 19] = [
        b"Idle\0" as *const u8 as *const ::core::ffi::c_char,
        b"Planning\0" as *const u8 as *const ::core::ffi::c_char,
        b"Calibration Glide\0" as *const u8 as *const ::core::ffi::c_char,
        b"Coast to Burn\0" as *const u8 as *const ::core::ffi::c_char,
        b"Burn Setup\0" as *const u8 as *const ::core::ffi::c_char,
        b"Deorbit Burn\0" as *const u8 as *const ::core::ffi::c_char,
        b"Entry Interface\0" as *const u8 as *const ::core::ffi::c_char,
        b"MM304 Entry\0" as *const u8 as *const ::core::ffi::c_char,
        b"TAEM\0" as *const u8 as *const ::core::ffi::c_char,
        b"Heading Alignment\0" as *const u8 as *const ::core::ffi::c_char,
        b"Final Approach\0" as *const u8 as *const ::core::ffi::c_char,
        b"Flare\0" as *const u8 as *const ::core::ffi::c_char,
        b"Touchdown\0" as *const u8 as *const ::core::ffi::c_char,
        b"Rollout\0" as *const u8 as *const ::core::ffi::c_char,
        b"Complete\0" as *const u8 as *const ::core::ffi::c_char,
        b"Paused\0" as *const u8 as *const ::core::ffi::c_char,
        b"Abort\0" as *const u8 as *const ::core::ffi::c_char,
        b"Fault\0" as *const u8 as *const ::core::ffi::c_char,
        b"Attitude Recovery\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    return if (p as ::core::ffi::c_uint) < 19 as ::core::ffi::c_uint {
        v[p as usize]
    } else {
        v[0 as ::core::ffi::c_int as usize]
    };
}
#[no_mangle]
pub unsafe extern "C" fn speed_mode_string(
    mut s: NavballSpeedMode,
) -> *const ::core::ffi::c_char {
    static mut v: [*const ::core::ffi::c_char; 3] = [
        b"unchanged\0" as *const u8 as *const ::core::ffi::c_char,
        b"orbit\0" as *const u8 as *const ::core::ffi::c_char,
        b"surface\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    return if (s as ::core::ffi::c_uint) < 3 as ::core::ffi::c_uint {
        v[s as usize]
    } else {
        v[0 as ::core::ffi::c_int as usize]
    };
}
#[no_mangle]
pub unsafe extern "C" fn profile_string(
    mut p: ControlProfile,
) -> *const ::core::ffi::c_char {
    static mut v: [*const ::core::ffi::c_char; 7] = [
        b"orbital\0" as *const u8 as *const ::core::ffi::c_char,
        b"entry\0" as *const u8 as *const ::core::ffi::c_char,
        b"taem\0" as *const u8 as *const ::core::ffi::c_char,
        b"approach\0" as *const u8 as *const ::core::ffi::c_char,
        b"flare\0" as *const u8 as *const ::core::ffi::c_char,
        b"rollout\0" as *const u8 as *const ::core::ffi::c_char,
        b"recovery\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    return if (p as ::core::ffi::c_uint) < 7 as ::core::ffi::c_uint {
        v[p as usize]
    } else {
        v[0 as ::core::ffi::c_int as usize]
    };
}
#[no_mangle]
pub unsafe extern "C" fn cal_state_string(
    mut s: CalibrationRunState,
) -> *const ::core::ffi::c_char {
    static mut v: [*const ::core::ffi::c_char; 6] = [
        b"Idle\0" as *const u8 as *const ::core::ffi::c_char,
        b"Settling\0" as *const u8 as *const ::core::ffi::c_char,
        b"Sampling\0" as *const u8 as *const ::core::ffi::c_char,
        b"Complete\0" as *const u8 as *const ::core::ffi::c_char,
        b"Stopped\0" as *const u8 as *const ::core::ffi::c_char,
        b"Aborted\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    return if (s as ::core::ffi::c_uint) < 6 as ::core::ffi::c_uint {
        v[s as usize]
    } else {
        v[0 as ::core::ffi::c_int as usize]
    };
}
#[no_mangle]
pub unsafe extern "C" fn landing_configuration_default() -> LandingConfiguration {
    let mut c: LandingConfiguration = LandingConfiguration {
        connection: ConnectionConfiguration {
            serial_port: [0; 256],
            rpc_host: [0; 256],
            rpc_port: 0,
            baud_rate: 0,
            timeout_ms: 0,
            client_name: [0; 128],
        },
        site: LandingSite {
            name: [0; 128],
            latitude: 0.,
            longitude: 0.,
            altitude: 0.,
            runway_heading: 0.,
            runway_length: 0.,
            runway_width: 0.,
            allow_reciprocal_runway: false,
        },
        vehicle: VehicleProfile {
            model_id: [0; 96],
            estimated_lift_to_drag: 0.,
            estimated_ballistic_coefficient: 0.,
            entry_angle_of_attack: 0.,
            maximum_angle_of_attack: 0.,
            maximum_bank_angle: 0.,
            maximum_dynamic_pressure: 0.,
            maximum_g_load: 0.,
            minimum_safe_speed: 0.,
            final_approach_speed: 0.,
            touchdown_speed: 0.,
            terminal_maximum_lift_angle_of_attack: 0.,
            allow_powered_approach: false,
            maximum_approach_throttle: 0.,
            airbrake_action_group: 0,
        },
        guidance: GuidanceSettings {
            target_deorbit_capture_radius: 0.,
            entry_interface_altitude_margin: 0.,
            target_entry_range: 0.,
            entry_range_reference_speed: 0.,
            target_entry_flight_path_angle: 0.,
            maximum_entry_flight_path_angle: 0.,
            target_post_burn_periapsis_altitude: 0.,
            deorbit_maximum_throttle: 0.,
            deorbit_throttle_ramp_duration: 0.,
            taem_interface_altitude: 0.,
            taem_interface_range: 0.,
            taem_force_handoff_speed: 0.,
            mm304_handoff_radius: 0.,
            mm304_perpendicular_heading_half_width: 0.,
            mm304_handoff_along_track: 0.,
            mm304_handoff_cross_track: 0.,
            mm305_min_altitude: 0.,
            mm305_max_altitude: 0.,
            mm305_target_mach: 0.,
            mm305_mach_half_width: 0.,
            hac_acquisition_altitude: 0.,
            hac_acquisition_mach: 0.,
            deorbit_timing_uncertainty: 0.,
            deorbit_thrust_uncertainty_fraction: 0.,
            deorbit_delta_v_uncertainty: 0.,
            deorbit_mass_uncertainty_fraction: 0.,
            deorbit_position_uncertainty: 0.,
            deorbit_velocity_uncertainty: 0.,
            deorbit_pointing_uncertainty: 0.,
            deorbit_atmosphere_uncertainty_fraction: 0.,
            deorbit_robust_minimum_pass_fraction: 0.,
            hac_radius: 0.,
            hac_look_ahead_angle: 0.,
            final_approach_distance: 0.,
            final_alignment_speed: 0.,
            final_glide_slope: 0.,
            taem_glide_slope: 0.,
            gear_deployment_altitude: 0.,
            flare_altitude: 0.,
            touchdown_sink_rate: 0.,
            guidance_rate: 0.,
            prediction_interval: 0.,
            s_turn_minimum_leg_duration: 0.,
            entry_roll_rate: 0.,
            entry_roll_acceleration: 0.,
            taem_roll_rate: 0.,
            approach_roll_rate: 0.,
            use_time_warp: false,
            taem_energy_law: false,
            sts_trajectory_delivery: false,
            minimum_planning_lead_time: 0.,
            taem_hac_law: false,
            terminal_speedbrake: false,
            adaptive_flare: false,
            touchdown_aim_along: 0.,
            rollout_parachutes: false,
        },
        calibration: CalibrationSettings {
            enable_passive_calibration: false,
            enable_trajectory_calibration: false,
            auto_apply_in_flight: false,
            minimum_dynamic_pressure: 0.,
            maximum_sample_dynamic_pressure: 0.,
            maximum_sideslip: 0.,
            minimum_calibration_radar_altitude: 0.,
            maximum_calibration_dynamic_pressure: 0.,
            maximum_calibration_g_load: 0.,
            maximum_calibration_sink_rate: 0.,
            abort_stall_fraction: 0.,
            minimum_angle_of_attack: 0.,
            maximum_angle_of_attack: 0.,
            angle_of_attack_step: 0.,
            settling_duration: 0.,
            sampling_duration: 0.,
            trajectory_learning_rate: 0.,
        },
    };
    memset(
        &raw mut c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<LandingConfiguration>() as size_t,
    );
    copystr(
        &raw mut c.connection.rpc_host as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"127.0.0.1\0" as *const u8 as *const ::core::ffi::c_char,
    );
    c.connection.rpc_port = 50000 as ::core::ffi::c_int;
    copystr(
        &raw mut c.connection.serial_port as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"/dev/tty.krpc\0" as *const u8 as *const ::core::ffi::c_char,
    );
    c.connection.baud_rate = 921600 as ::core::ffi::c_int;
    c.connection.timeout_ms = 2000 as ::core::ffi::c_int;
    copystr(
        &raw mut c.connection.client_name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"Spaceplane Landing Guidance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    copystr(
        &raw mut c.site.name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"KSC Runway 09\0" as *const u8 as *const ::core::ffi::c_char,
    );
    c.site.latitude = -0.0486111111f64;
    c.site.longitude = -74.7283333333f64;
    c.site.altitude = 70 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.site.runway_heading = 90 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.site.runway_length = 2500 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.site.runway_width = 70 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.site.allow_reciprocal_runway = true_0 != 0;
    copystr(
        &raw mut c.vehicle.model_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        b"STS-N\0" as *const u8 as *const ::core::ffi::c_char,
    );
    c.vehicle.estimated_lift_to_drag = 0.4f64;
    c.vehicle.estimated_ballistic_coefficient = 700 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    c.vehicle.entry_angle_of_attack = 18 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.vehicle.maximum_angle_of_attack = 28 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    c.vehicle.terminal_maximum_lift_angle_of_attack = 15 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    c.vehicle.maximum_bank_angle = 70 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.vehicle.maximum_dynamic_pressure = 45000 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    c.vehicle.maximum_g_load = 3.5f64;
    c.vehicle.minimum_safe_speed = 85 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.vehicle.final_approach_speed = 115 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.vehicle.touchdown_speed = 75 as ::core::ffi::c_int as ::core::ffi::c_double;
    c.vehicle.allow_powered_approach = false_0 != 0;
    c.vehicle.maximum_approach_throttle = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    c.vehicle.airbrake_action_group = 1 as ::core::ffi::c_uint;
    let mut g: *mut GuidanceSettings = &raw mut c.guidance;
    (*g).target_deorbit_capture_radius = 100000 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).entry_interface_altitude_margin = 500 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).target_entry_range = 1030000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).target_entry_flight_path_angle = -2.0f64;
    (*g).maximum_entry_flight_path_angle = -3.0f64;
    (*g).target_post_burn_periapsis_altitude = 50000 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).deorbit_maximum_throttle = 0.18f64;
    (*g).deorbit_throttle_ramp_duration = 12 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).deorbit_timing_uncertainty = 0.5f64;
    (*g).deorbit_thrust_uncertainty_fraction = 0.08f64;
    (*g).deorbit_delta_v_uncertainty = 0.5f64;
    (*g).deorbit_mass_uncertainty_fraction = 0.03f64;
    (*g).deorbit_position_uncertainty = 50 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).deorbit_velocity_uncertainty = 0.5f64;
    (*g).deorbit_pointing_uncertainty = 2 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).deorbit_atmosphere_uncertainty_fraction = 0.12f64;
    (*g).deorbit_robust_minimum_pass_fraction = 0.75f64;
    (*g).taem_interface_altitude = 25000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).taem_interface_range = 96500 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).taem_force_handoff_speed = 1300 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).mm304_handoff_radius = 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).mm304_perpendicular_heading_half_width = 30 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).mm304_handoff_along_track = -(8000 as ::core::ffi::c_int)
        as ::core::ffi::c_double;
    (*g).mm304_handoff_cross_track = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).mm305_min_altitude = 22000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).mm305_max_altitude = 30000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).mm305_target_mach = 2.5f64;
    (*g).mm305_mach_half_width = 0.35f64;
    (*g).hac_acquisition_altitude = 10700 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).hac_acquisition_mach = 0.95f64;
    (*g).hac_radius = 12000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).hac_look_ahead_angle = 16 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).final_approach_distance = 8000 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).final_alignment_speed = 160 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).final_glide_slope = 20 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).taem_glide_slope = 12 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).gear_deployment_altitude = 1500 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).flare_altitude = 55 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).touchdown_sink_rate = -1.5f64;
    (*g).guidance_rate = 10 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).prediction_interval = 1.5f64;
    (*g).s_turn_minimum_leg_duration = 24 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).entry_roll_rate = 4.5f64;
    (*g).entry_roll_acceleration = 1.8f64;
    (*g).taem_roll_rate = 6 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).approach_roll_rate = 3.5f64;
    (*g).use_time_warp = true_0 != 0;
    (*g).minimum_planning_lead_time = 12 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut a: *mut CalibrationSettings = &raw mut c.calibration;
    (*a).enable_passive_calibration = true_0 != 0;
    (*a).enable_trajectory_calibration = true_0 != 0;
    (*a).auto_apply_in_flight = true_0 != 0;
    (*a).minimum_dynamic_pressure = 500 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).maximum_sample_dynamic_pressure = 55000 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*a).maximum_sideslip = 8 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).minimum_calibration_radar_altitude = 3000 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*a).maximum_calibration_dynamic_pressure = 35000 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*a).maximum_calibration_g_load = 2.5f64;
    (*a).maximum_calibration_sink_rate = 80 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*a).abort_stall_fraction = 0.32f64;
    (*a).minimum_angle_of_attack = 3 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).maximum_angle_of_attack = 18 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).angle_of_attack_step = 3 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).settling_duration = 4 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).sampling_duration = 8 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*a).trajectory_learning_rate = 0.18f64;
    return c;
}
unsafe extern "C" fn finite_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 { value } else { fallback };
}
unsafe extern "C" fn positive_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 && value > 0.0f64 {
        value
    } else {
        fallback
    };
}
unsafe extern "C" fn nonnegative_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 && value >= 0.0f64 {
        value
    } else {
        fallback
    };
}
unsafe extern "C" fn fraction_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if value.is_finite() as i32 == 0 {
        return fallback;
    }
    return clampd(value, 0.0f64, 1.0f64);
}
unsafe extern "C" fn descent_angle_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if value.is_finite() as i32 == 0 || !(value > 0.0f64) || !(value < 90.0f64) {
        return fallback;
    }
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn landing_configuration_normalize(
    mut c: *mut LandingConfiguration,
) {
    if c.is_null() {
        return;
    }
    let mut d: LandingConfiguration = landing_configuration_default();
    if (*c).connection.serial_port[0 as ::core::ffi::c_int as usize] == 0 {
        copystr(
            &raw mut (*c).connection.serial_port as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            &raw mut d.connection.serial_port as *mut ::core::ffi::c_char,
        );
    }
    if (*c).connection.client_name[0 as ::core::ffi::c_int as usize] == 0 {
        copystr(
            &raw mut (*c).connection.client_name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            &raw mut d.connection.client_name as *mut ::core::ffi::c_char,
        );
    }
    if (*c).connection.rpc_host[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
        != 0
        && ((*c).connection.rpc_port < 1 as ::core::ffi::c_int
            || (*c).connection.rpc_port > 65535 as ::core::ffi::c_int)
    {
        (*c).connection.rpc_port = d.connection.rpc_port;
    }
    if (*c).connection.baud_rate <= 0 as ::core::ffi::c_int {
        (*c).connection.baud_rate = d.connection.baud_rate;
    }
    if (*c).connection.timeout_ms <= 0 as ::core::ffi::c_int {
        (*c).connection.timeout_ms = d.connection.timeout_ms;
    }
    (*c).site.latitude = clampd(
        finite_or((*c).site.latitude, d.site.latitude),
        -90.0f64,
        90.0f64,
    );
    (*c).site.longitude = norm_signed_deg(
        finite_or((*c).site.longitude, d.site.longitude),
    );
    (*c).site.altitude = finite_or((*c).site.altitude, d.site.altitude);
    (*c).site.runway_heading = norm_deg(
        finite_or((*c).site.runway_heading, d.site.runway_heading),
    );
    (*c).site.runway_length = positive_or((*c).site.runway_length, d.site.runway_length);
    (*c).site.runway_width = positive_or((*c).site.runway_width, d.site.runway_width);
    let mut v: *mut VehicleProfile = &raw mut (*c).vehicle;
    let mut vd: *const VehicleProfile = &raw mut d.vehicle;
    if (*v).model_id[0 as ::core::ffi::c_int as usize] == 0 {
        copystr(
            &raw mut (*v).model_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
            &raw const (*vd).model_id as *const ::core::ffi::c_char,
        );
    }
    (*v).estimated_lift_to_drag = positive_or(
        (*v).estimated_lift_to_drag,
        (*vd).estimated_lift_to_drag,
    );
    (*v).estimated_ballistic_coefficient = positive_or(
        (*v).estimated_ballistic_coefficient,
        (*vd).estimated_ballistic_coefficient,
    );
    if (*v).maximum_angle_of_attack.is_finite() as i32 == 0
        || !((*v).maximum_angle_of_attack > 0.0f64)
        || !((*v).maximum_angle_of_attack < 90.0f64)
    {
        (*v).maximum_angle_of_attack = (*vd).maximum_angle_of_attack;
    }
    (*v).entry_angle_of_attack = clampd(
        finite_or((*v).entry_angle_of_attack, (*vd).entry_angle_of_attack),
        0.0f64,
        (*v).maximum_angle_of_attack,
    );
    (*v).terminal_maximum_lift_angle_of_attack = clampd(
        positive_or(
            (*v).terminal_maximum_lift_angle_of_attack,
            (*vd).terminal_maximum_lift_angle_of_attack,
        ),
        1.0f64,
        (*v).maximum_angle_of_attack,
    );
    if (*v).maximum_bank_angle.is_finite() as i32 == 0
        || !((*v).maximum_bank_angle > 0.0f64) || !((*v).maximum_bank_angle < 90.0f64)
    {
        (*v).maximum_bank_angle = (*vd).maximum_bank_angle;
    }
    (*v).maximum_dynamic_pressure = positive_or(
        (*v).maximum_dynamic_pressure,
        (*vd).maximum_dynamic_pressure,
    );
    (*v).maximum_g_load = positive_or((*v).maximum_g_load, (*vd).maximum_g_load);
    (*v).minimum_safe_speed = positive_or(
        (*v).minimum_safe_speed,
        (*vd).minimum_safe_speed,
    );
    (*v).final_approach_speed = positive_or(
        (*v).final_approach_speed,
        (*vd).final_approach_speed,
    );
    (*v).final_approach_speed = fmax((*v).final_approach_speed, (*v).minimum_safe_speed);
    (*v).touchdown_speed = positive_or((*v).touchdown_speed, (*vd).touchdown_speed);
    (*v).touchdown_speed = fmin((*v).touchdown_speed, (*v).final_approach_speed);
    (*v).maximum_approach_throttle = fraction_or(
        (*v).maximum_approach_throttle,
        (*vd).maximum_approach_throttle,
    );
    if (*v).airbrake_action_group < 1 as ::core::ffi::c_uint
        || (*v).airbrake_action_group > 10 as ::core::ffi::c_uint
    {
        (*v).airbrake_action_group = (*vd).airbrake_action_group;
    }
    let mut g: *mut GuidanceSettings = &raw mut (*c).guidance;
    let mut gd: *const GuidanceSettings = &raw mut d.guidance;
    (*g).target_deorbit_capture_radius = positive_or(
        (*g).target_deorbit_capture_radius,
        (*gd).target_deorbit_capture_radius,
    );
    (*g).entry_interface_altitude_margin = nonnegative_or(
        (*g).entry_interface_altitude_margin,
        (*gd).entry_interface_altitude_margin,
    );
    (*g).target_entry_range = positive_or(
        (*g).target_entry_range,
        (*gd).target_entry_range,
    );
    if (*g).target_entry_flight_path_angle.is_finite() as i32 == 0
        || (*g).target_entry_flight_path_angle >= 0.0f64
        || (*g).target_entry_flight_path_angle <= -90.0f64
    {
        (*g).target_entry_flight_path_angle = (*gd).target_entry_flight_path_angle;
    }
    if (*g).maximum_entry_flight_path_angle.is_finite() as i32 == 0
        || (*g).maximum_entry_flight_path_angle >= 0.0f64
        || (*g).maximum_entry_flight_path_angle <= -90.0f64
    {
        (*g).maximum_entry_flight_path_angle = (*gd).maximum_entry_flight_path_angle;
    }
    if (*g).maximum_entry_flight_path_angle > (*g).target_entry_flight_path_angle {
        (*g).maximum_entry_flight_path_angle = (*g).target_entry_flight_path_angle;
    }
    (*g).target_post_burn_periapsis_altitude = finite_or(
        (*g).target_post_burn_periapsis_altitude,
        (*gd).target_post_burn_periapsis_altitude,
    );
    (*g).deorbit_maximum_throttle = fraction_or(
        (*g).deorbit_maximum_throttle,
        (*gd).deorbit_maximum_throttle,
    );
    (*g).deorbit_throttle_ramp_duration = nonnegative_or(
        (*g).deorbit_throttle_ramp_duration,
        (*gd).deorbit_throttle_ramp_duration,
    );
    (*g).deorbit_timing_uncertainty = nonnegative_or(
        (*g).deorbit_timing_uncertainty,
        (*gd).deorbit_timing_uncertainty,
    );
    (*g).deorbit_thrust_uncertainty_fraction = fraction_or(
        (*g).deorbit_thrust_uncertainty_fraction,
        (*gd).deorbit_thrust_uncertainty_fraction,
    );
    (*g).deorbit_delta_v_uncertainty = nonnegative_or(
        (*g).deorbit_delta_v_uncertainty,
        (*gd).deorbit_delta_v_uncertainty,
    );
    (*g).deorbit_mass_uncertainty_fraction = fraction_or(
        (*g).deorbit_mass_uncertainty_fraction,
        (*gd).deorbit_mass_uncertainty_fraction,
    );
    (*g).deorbit_position_uncertainty = nonnegative_or(
        (*g).deorbit_position_uncertainty,
        (*gd).deorbit_position_uncertainty,
    );
    (*g).deorbit_velocity_uncertainty = nonnegative_or(
        (*g).deorbit_velocity_uncertainty,
        (*gd).deorbit_velocity_uncertainty,
    );
    (*g).deorbit_pointing_uncertainty = nonnegative_or(
        (*g).deorbit_pointing_uncertainty,
        (*gd).deorbit_pointing_uncertainty,
    );
    (*g).deorbit_atmosphere_uncertainty_fraction = fraction_or(
        (*g).deorbit_atmosphere_uncertainty_fraction,
        (*gd).deorbit_atmosphere_uncertainty_fraction,
    );
    (*g).deorbit_robust_minimum_pass_fraction = fraction_or(
        (*g).deorbit_robust_minimum_pass_fraction,
        (*gd).deorbit_robust_minimum_pass_fraction,
    );
    (*g).taem_interface_altitude = positive_or(
        (*g).taem_interface_altitude,
        (*gd).taem_interface_altitude,
    );
    (*g).taem_interface_range = positive_or(
        (*g).taem_interface_range,
        (*gd).taem_interface_range,
    );
    (*g).taem_force_handoff_speed = positive_or(
        (*g).taem_force_handoff_speed,
        (*gd).taem_force_handoff_speed,
    );
    (*g).taem_force_handoff_speed = fmax(
        (*g).taem_force_handoff_speed,
        (*v).minimum_safe_speed,
    );
    (*g).mm304_handoff_radius = positive_or(
        (*g).mm304_handoff_radius,
        (*gd).mm304_handoff_radius,
    );
    (*g).mm304_handoff_along_track = finite_or(
        (*g).mm304_handoff_along_track,
        (*gd).mm304_handoff_along_track,
    );
    (*g).mm304_handoff_cross_track = finite_or(
        (*g).mm304_handoff_cross_track,
        (*gd).mm304_handoff_cross_track,
    );
    (*g).mm304_perpendicular_heading_half_width = finite_or(
        (*g).mm304_perpendicular_heading_half_width,
        (*gd).mm304_perpendicular_heading_half_width,
    );
    (*g).mm304_perpendicular_heading_half_width = clampd(
        (*g).mm304_perpendicular_heading_half_width,
        0.0f64,
        90.0f64,
    );
    (*g).mm305_min_altitude = positive_or(
        (*g).mm305_min_altitude,
        (*gd).mm305_min_altitude,
    );
    (*g).mm305_max_altitude = positive_or(
        (*g).mm305_max_altitude,
        (*gd).mm305_max_altitude,
    );
    if (*g).mm305_max_altitude < (*g).mm305_min_altitude {
        (*g).mm305_max_altitude = (*g).mm305_min_altitude;
    }
    (*g).mm305_target_mach = positive_or(
        (*g).mm305_target_mach,
        (*gd).mm305_target_mach,
    );
    (*g).mm305_mach_half_width = positive_or(
        (*g).mm305_mach_half_width,
        (*gd).mm305_mach_half_width,
    );
    (*g).hac_acquisition_altitude = positive_or(
        (*g).hac_acquisition_altitude,
        (*gd).hac_acquisition_altitude,
    );
    (*g).hac_acquisition_altitude = fmin(
        (*g).hac_acquisition_altitude,
        (*g).taem_interface_altitude,
    );
    (*g).hac_acquisition_mach = positive_or(
        (*g).hac_acquisition_mach,
        (*gd).hac_acquisition_mach,
    );
    (*g).hac_radius = positive_or((*g).hac_radius, (*gd).hac_radius);
    (*g).hac_look_ahead_angle = finite_or(
        (*g).hac_look_ahead_angle,
        (*gd).hac_look_ahead_angle,
    );
    (*g).hac_look_ahead_angle = clampd((*g).hac_look_ahead_angle, 0.0f64, 180.0f64);
    (*g).final_approach_distance = positive_or(
        (*g).final_approach_distance,
        (*gd).final_approach_distance,
    );
    (*g).final_alignment_speed = positive_or(
        (*g).final_alignment_speed,
        (*gd).final_alignment_speed,
    );
    (*g).final_alignment_speed = fmax((*g).final_alignment_speed, (*v).touchdown_speed);
    (*g).final_glide_slope = descent_angle_or(
        (*g).final_glide_slope,
        (*gd).final_glide_slope,
    );
    (*g).taem_glide_slope = descent_angle_or(
        (*g).taem_glide_slope,
        (*gd).taem_glide_slope,
    );
    (*g).gear_deployment_altitude = nonnegative_or(
        (*g).gear_deployment_altitude,
        (*gd).gear_deployment_altitude,
    );
    (*g).flare_altitude = nonnegative_or((*g).flare_altitude, (*gd).flare_altitude);
    (*g).touchdown_aim_along = nonnegative_or((*g).touchdown_aim_along, 0.0f64);
    if (*g).touchdown_sink_rate.is_finite() as i32 == 0
        || !((*g).touchdown_sink_rate < 0.0f64)
    {
        (*g).touchdown_sink_rate = (*gd).touchdown_sink_rate;
    }
    (*g).guidance_rate = positive_or((*g).guidance_rate, (*gd).guidance_rate);
    (*g).prediction_interval = positive_or(
        (*g).prediction_interval,
        (*gd).prediction_interval,
    );
    (*g).s_turn_minimum_leg_duration = nonnegative_or(
        (*g).s_turn_minimum_leg_duration,
        (*gd).s_turn_minimum_leg_duration,
    );
    (*g).entry_roll_rate = positive_or((*g).entry_roll_rate, (*gd).entry_roll_rate);
    (*g).entry_roll_acceleration = positive_or(
        (*g).entry_roll_acceleration,
        (*gd).entry_roll_acceleration,
    );
    (*g).taem_roll_rate = positive_or((*g).taem_roll_rate, (*gd).taem_roll_rate);
    (*g).approach_roll_rate = positive_or(
        (*g).approach_roll_rate,
        (*gd).approach_roll_rate,
    );
    (*g).minimum_planning_lead_time = nonnegative_or(
        (*g).minimum_planning_lead_time,
        (*gd).minimum_planning_lead_time,
    );
    let mut a: *mut CalibrationSettings = &raw mut (*c).calibration;
    let mut ad: *const CalibrationSettings = &raw mut d.calibration;
    (*a).minimum_dynamic_pressure = nonnegative_or(
        (*a).minimum_dynamic_pressure,
        (*ad).minimum_dynamic_pressure,
    );
    (*a).maximum_sample_dynamic_pressure = positive_or(
        (*a).maximum_sample_dynamic_pressure,
        (*ad).maximum_sample_dynamic_pressure,
    );
    (*a).maximum_sample_dynamic_pressure = fmax(
        (*a).maximum_sample_dynamic_pressure,
        (*a).minimum_dynamic_pressure,
    );
    (*a).maximum_sideslip = nonnegative_or(
        (*a).maximum_sideslip,
        (*ad).maximum_sideslip,
    );
    (*a).minimum_calibration_radar_altitude = nonnegative_or(
        (*a).minimum_calibration_radar_altitude,
        (*ad).minimum_calibration_radar_altitude,
    );
    (*a).maximum_calibration_dynamic_pressure = positive_or(
        (*a).maximum_calibration_dynamic_pressure,
        (*ad).maximum_calibration_dynamic_pressure,
    );
    (*a).maximum_calibration_dynamic_pressure = fmax(
        (*a).maximum_calibration_dynamic_pressure,
        (*a).minimum_dynamic_pressure,
    );
    (*a).maximum_calibration_g_load = positive_or(
        (*a).maximum_calibration_g_load,
        (*ad).maximum_calibration_g_load,
    );
    (*a).maximum_calibration_sink_rate = positive_or(
        (*a).maximum_calibration_sink_rate,
        (*ad).maximum_calibration_sink_rate,
    );
    (*a).abort_stall_fraction = fraction_or(
        (*a).abort_stall_fraction,
        (*ad).abort_stall_fraction,
    );
    (*a).minimum_angle_of_attack = clampd(
        finite_or((*a).minimum_angle_of_attack, (*ad).minimum_angle_of_attack),
        0.0f64,
        (*v).maximum_angle_of_attack,
    );
    (*a).maximum_angle_of_attack = clampd(
        finite_or((*a).maximum_angle_of_attack, (*ad).maximum_angle_of_attack),
        (*a).minimum_angle_of_attack,
        (*v).maximum_angle_of_attack,
    );
    (*a).angle_of_attack_step = positive_or(
        (*a).angle_of_attack_step,
        (*ad).angle_of_attack_step,
    );
    (*a).settling_duration = nonnegative_or(
        (*a).settling_duration,
        (*ad).settling_duration,
    );
    (*a).sampling_duration = nonnegative_or(
        (*a).sampling_duration,
        (*ad).sampling_duration,
    );
    (*a).trajectory_learning_rate = fraction_or(
        (*a).trajectory_learning_rate,
        (*ad).trajectory_learning_rate,
    );
}
unsafe extern "C" fn obj(
    mut d: *const JsonDoc,
    mut p: ::core::ffi::c_int,
    mut k: *const ::core::ffi::c_char,
) -> ::core::ffi::c_int {
    return json_object_get(d, p, k);
}
unsafe extern "C" fn num(
    mut d: *const JsonDoc,
    mut p: ::core::ffi::c_int,
    mut k: *const ::core::ffi::c_char,
    mut f: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return json_number(d, obj(d, p, k), f);
}
unsafe extern "C" fn boo(
    mut d: *const JsonDoc,
    mut p: ::core::ffi::c_int,
    mut k: *const ::core::ffi::c_char,
    mut f: bool,
) -> bool {
    return json_boolean(d, obj(d, p, k), f);
}
unsafe extern "C" fn strv(
    mut d: *const JsonDoc,
    mut p: ::core::ffi::c_int,
    mut k: *const ::core::ffi::c_char,
    mut out: *mut ::core::ffi::c_char,
    mut n: size_t,
) {
    let mut i: ::core::ffi::c_int = obj(d, p, k);
    if i >= 0 as ::core::ffi::c_int {
        json_string(d, i, out, n);
    }
}
#[no_mangle]
pub unsafe extern "C" fn landing_configuration_from_json(
    mut c: *mut LandingConfiguration,
    mut d: *const JsonDoc,
    mut root: ::core::ffi::c_int,
) -> bool {
    if root < 0 as ::core::ffi::c_int
        || (*(*d).tokens.offset(root as isize)).type_0 as ::core::ffi::c_uint
            != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return false_0 != 0;
    }
    let mut n: LandingConfiguration = *c;
    let mut x: ::core::ffi::c_int = obj(
        d,
        root,
        b"connection\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if x >= 0 as ::core::ffi::c_int {
        strv(
            d,
            x,
            b"serialPort\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut n.connection.serial_port as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        );
        strv(
            d,
            x,
            b"rpcHost\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut n.connection.rpc_host as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        );
        n.connection.rpc_port = json_integer(
            d,
            obj(d, x, b"rpcPort\0" as *const u8 as *const ::core::ffi::c_char),
            n.connection.rpc_port as ::core::ffi::c_long,
        ) as ::core::ffi::c_int;
        n.connection.baud_rate = json_integer(
            d,
            obj(d, x, b"baudRate\0" as *const u8 as *const ::core::ffi::c_char),
            n.connection.baud_rate as ::core::ffi::c_long,
        ) as ::core::ffi::c_int;
        n.connection.timeout_ms = json_integer(
            d,
            obj(d, x, b"timeoutMs\0" as *const u8 as *const ::core::ffi::c_char),
            n.connection.timeout_ms as ::core::ffi::c_long,
        ) as ::core::ffi::c_int;
        strv(
            d,
            x,
            b"clientName\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut n.connection.client_name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        );
        let mut legacy: ::core::ffi::c_int = obj(
            d,
            x,
            b"address\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if obj(d, x, b"serialPort\0" as *const u8 as *const ::core::ffi::c_char)
            < 0 as ::core::ffi::c_int && legacy >= 0 as ::core::ffi::c_int
        {
            let mut candidate: [::core::ffi::c_char; 256] = [
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
            ];
            json_string(
                d,
                legacy,
                &raw mut candidate as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            );
            if candidate[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
                == '/' as i32
            {
                copystr(
                    &raw mut n.connection.serial_port as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                    &raw mut candidate as *mut ::core::ffi::c_char,
                );
            }
        }
    }
    x = obj(d, root, b"site\0" as *const u8 as *const ::core::ffi::c_char);
    if x >= 0 as ::core::ffi::c_int {
        strv(
            d,
            x,
            b"name\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut n.site.name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        );
        n.site.latitude = num(
            d,
            x,
            b"latitude\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.latitude,
        );
        n.site.longitude = num(
            d,
            x,
            b"longitude\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.longitude,
        );
        n.site.altitude = num(
            d,
            x,
            b"altitude\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.altitude,
        );
        n.site.runway_heading = num(
            d,
            x,
            b"runwayHeading\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.runway_heading,
        );
        n.site.runway_length = num(
            d,
            x,
            b"runwayLength\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.runway_length,
        );
        n.site.runway_width = num(
            d,
            x,
            b"runwayWidth\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.runway_width,
        );
        n.site.allow_reciprocal_runway = boo(
            d,
            x,
            b"allowReciprocalRunway\0" as *const u8 as *const ::core::ffi::c_char,
            n.site.allow_reciprocal_runway,
        );
    }
    x = obj(d, root, b"vehicle\0" as *const u8 as *const ::core::ffi::c_char);
    if x >= 0 as ::core::ffi::c_int {
        let mut v: *mut VehicleProfile = &raw mut n.vehicle;
        strv(
            d,
            x,
            b"modelId\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut (*v).model_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        );
        (*v).estimated_lift_to_drag = num(
            d,
            x,
            b"estimatedLiftToDrag\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).estimated_lift_to_drag,
        );
        (*v).estimated_ballistic_coefficient = num(
            d,
            x,
            b"estimatedBallisticCoefficient\0" as *const u8
                as *const ::core::ffi::c_char,
            (*v).estimated_ballistic_coefficient,
        );
        (*v).entry_angle_of_attack = num(
            d,
            x,
            b"entryAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).entry_angle_of_attack,
        );
        (*v).maximum_angle_of_attack = num(
            d,
            x,
            b"maximumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).maximum_angle_of_attack,
        );
        (*v).terminal_maximum_lift_angle_of_attack = num(
            d,
            x,
            b"terminalMaximumLiftAngleOfAttack\0" as *const u8
                as *const ::core::ffi::c_char,
            (*v).terminal_maximum_lift_angle_of_attack,
        );
        (*v).maximum_bank_angle = num(
            d,
            x,
            b"maximumBankAngle\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).maximum_bank_angle,
        );
        (*v).maximum_dynamic_pressure = num(
            d,
            x,
            b"maximumDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).maximum_dynamic_pressure,
        );
        (*v).maximum_g_load = num(
            d,
            x,
            b"maximumGLoad\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).maximum_g_load,
        );
        (*v).minimum_safe_speed = num(
            d,
            x,
            b"minimumSafeSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).minimum_safe_speed,
        );
        (*v).final_approach_speed = num(
            d,
            x,
            b"finalApproachSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).final_approach_speed,
        );
        (*v).touchdown_speed = num(
            d,
            x,
            b"touchdownSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).touchdown_speed,
        );
        (*v).allow_powered_approach = boo(
            d,
            x,
            b"allowPoweredApproach\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).allow_powered_approach,
        );
        (*v).maximum_approach_throttle = num(
            d,
            x,
            b"maximumApproachThrottle\0" as *const u8 as *const ::core::ffi::c_char,
            (*v).maximum_approach_throttle,
        );
        (*v).airbrake_action_group = json_integer(
            d,
            obj(
                d,
                x,
                b"airbrakeActionGroup\0" as *const u8 as *const ::core::ffi::c_char,
            ),
            (*v).airbrake_action_group as ::core::ffi::c_long,
        ) as ::core::ffi::c_int as ::core::ffi::c_uint;
    }
    x = obj(d, root, b"guidance\0" as *const u8 as *const ::core::ffi::c_char);
    if x >= 0 as ::core::ffi::c_int {
        let mut g: *mut GuidanceSettings = &raw mut n.guidance;
        (*g).target_deorbit_capture_radius = num(
            d,
            x,
            b"targetDeorbitCaptureRadius\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).target_deorbit_capture_radius,
        );
        (*g).entry_interface_altitude_margin = num(
            d,
            x,
            b"entryInterfaceAltitudeMargin\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).entry_interface_altitude_margin,
        );
        (*g).target_entry_range = num(
            d,
            x,
            b"targetEntryRange\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).target_entry_range,
        );
        (*g).entry_range_reference_speed = num(
            d,
            x,
            b"entryRangeReferenceSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).entry_range_reference_speed,
        );
        (*g).target_entry_flight_path_angle = num(
            d,
            x,
            b"targetEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).target_entry_flight_path_angle,
        );
        (*g).maximum_entry_flight_path_angle = num(
            d,
            x,
            b"maximumEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).maximum_entry_flight_path_angle,
        );
        (*g).target_post_burn_periapsis_altitude = num(
            d,
            x,
            b"targetPostBurnPeriapsisAltitude\0" as *const u8
                as *const ::core::ffi::c_char,
            (*g).target_post_burn_periapsis_altitude,
        );
        (*g).deorbit_maximum_throttle = num(
            d,
            x,
            b"deorbitMaximumThrottle\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_maximum_throttle,
        );
        (*g).deorbit_throttle_ramp_duration = num(
            d,
            x,
            b"deorbitThrottleRampDuration\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_throttle_ramp_duration,
        );
        (*g).deorbit_timing_uncertainty = num(
            d,
            x,
            b"deorbitTimingUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_timing_uncertainty,
        );
        (*g).deorbit_thrust_uncertainty_fraction = num(
            d,
            x,
            b"deorbitThrustUncertaintyFraction\0" as *const u8
                as *const ::core::ffi::c_char,
            (*g).deorbit_thrust_uncertainty_fraction,
        );
        (*g).deorbit_delta_v_uncertainty = num(
            d,
            x,
            b"deorbitDeltaVUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_delta_v_uncertainty,
        );
        (*g).deorbit_mass_uncertainty_fraction = num(
            d,
            x,
            b"deorbitMassUncertaintyFraction\0" as *const u8
                as *const ::core::ffi::c_char,
            (*g).deorbit_mass_uncertainty_fraction,
        );
        (*g).deorbit_position_uncertainty = num(
            d,
            x,
            b"deorbitPositionUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_position_uncertainty,
        );
        (*g).deorbit_velocity_uncertainty = num(
            d,
            x,
            b"deorbitVelocityUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_velocity_uncertainty,
        );
        (*g).deorbit_pointing_uncertainty = num(
            d,
            x,
            b"deorbitPointingUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).deorbit_pointing_uncertainty,
        );
        (*g).deorbit_atmosphere_uncertainty_fraction = num(
            d,
            x,
            b"deorbitAtmosphereUncertaintyFraction\0" as *const u8
                as *const ::core::ffi::c_char,
            (*g).deorbit_atmosphere_uncertainty_fraction,
        );
        (*g).deorbit_robust_minimum_pass_fraction = num(
            d,
            x,
            b"deorbitRobustMinimumPassFraction\0" as *const u8
                as *const ::core::ffi::c_char,
            (*g).deorbit_robust_minimum_pass_fraction,
        );
        (*g).taem_interface_altitude = num(
            d,
            x,
            b"taemInterfaceAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_interface_altitude,
        );
        (*g).taem_interface_range = num(
            d,
            x,
            b"taemInterfaceRange\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_interface_range,
        );
        (*g).taem_force_handoff_speed = num(
            d,
            x,
            b"taemForceHandoffSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_force_handoff_speed,
        );
        (*g).mm304_handoff_radius = num(
            d,
            x,
            b"mm304HandoffRadius\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm304_handoff_radius,
        );
        (*g).mm304_perpendicular_heading_half_width = num(
            d,
            x,
            b"mm304PerpendicularHeadingHalfWidth\0" as *const u8
                as *const ::core::ffi::c_char,
            (*g).mm304_perpendicular_heading_half_width,
        );
        (*g).mm304_handoff_along_track = num(
            d,
            x,
            b"mm304HandoffAlongTrack\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm304_handoff_along_track,
        );
        (*g).mm304_handoff_cross_track = num(
            d,
            x,
            b"mm304HandoffCrossTrack\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm304_handoff_cross_track,
        );
        (*g).mm305_min_altitude = num(
            d,
            x,
            b"mm305MinimumAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm305_min_altitude,
        );
        (*g).mm305_max_altitude = num(
            d,
            x,
            b"mm305MaximumAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm305_max_altitude,
        );
        (*g).mm305_target_mach = num(
            d,
            x,
            b"mm305TargetMach\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm305_target_mach,
        );
        (*g).mm305_mach_half_width = num(
            d,
            x,
            b"mm305MachHalfWidth\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).mm305_mach_half_width,
        );
        (*g).hac_acquisition_altitude = num(
            d,
            x,
            b"hacAcquisitionAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).hac_acquisition_altitude,
        );
        (*g).hac_acquisition_mach = num(
            d,
            x,
            b"hacAcquisitionMach\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).hac_acquisition_mach,
        );
        (*g).hac_radius = num(
            d,
            x,
            b"hacRadius\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).hac_radius,
        );
        (*g).hac_look_ahead_angle = num(
            d,
            x,
            b"hacLookAheadAngle\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).hac_look_ahead_angle,
        );
        (*g).final_approach_distance = num(
            d,
            x,
            b"finalApproachDistance\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).final_approach_distance,
        );
        (*g).final_alignment_speed = num(
            d,
            x,
            b"finalAlignmentSpeed\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).final_alignment_speed,
        );
        (*g).final_glide_slope = num(
            d,
            x,
            b"finalGlideSlope\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).final_glide_slope,
        );
        (*g).taem_glide_slope = num(
            d,
            x,
            b"taemGlideSlope\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_glide_slope,
        );
        (*g).gear_deployment_altitude = num(
            d,
            x,
            b"gearDeploymentAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).gear_deployment_altitude,
        );
        (*g).flare_altitude = num(
            d,
            x,
            b"flareAltitude\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).flare_altitude,
        );
        (*g).touchdown_aim_along = num(
            d,
            x,
            b"touchdownAimAlong\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).touchdown_aim_along,
        );
        (*g).touchdown_sink_rate = num(
            d,
            x,
            b"touchdownSinkRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).touchdown_sink_rate,
        );
        (*g).guidance_rate = num(
            d,
            x,
            b"guidanceRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).guidance_rate,
        );
        (*g).prediction_interval = num(
            d,
            x,
            b"predictionInterval\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).prediction_interval,
        );
        (*g).s_turn_minimum_leg_duration = num(
            d,
            x,
            b"sTurnMinimumLegDuration\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).s_turn_minimum_leg_duration,
        );
        (*g).entry_roll_rate = num(
            d,
            x,
            b"entryRollRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).entry_roll_rate,
        );
        (*g).entry_roll_acceleration = num(
            d,
            x,
            b"entryRollAcceleration\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).entry_roll_acceleration,
        );
        (*g).taem_roll_rate = num(
            d,
            x,
            b"taemRollRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_roll_rate,
        );
        (*g).approach_roll_rate = num(
            d,
            x,
            b"approachRollRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).approach_roll_rate,
        );
        (*g).minimum_planning_lead_time = num(
            d,
            x,
            b"minimumPlanningLeadTime\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).minimum_planning_lead_time,
        );
        (*g).use_time_warp = boo(
            d,
            x,
            b"useTimeWarp\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).use_time_warp,
        );
        (*g).taem_energy_law = boo(
            d,
            x,
            b"taemEnergyLaw\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_energy_law,
        );
        (*g).taem_hac_law = boo(
            d,
            x,
            b"taemHeadingAlignmentCone\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).taem_hac_law,
        );
        (*g).sts_trajectory_delivery = boo(
            d,
            x,
            b"stsTrajectoryDelivery\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).sts_trajectory_delivery,
        );
        (*g).terminal_speedbrake = boo(
            d,
            x,
            b"terminalSpeedbrake\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).terminal_speedbrake,
        );
        (*g).adaptive_flare = boo(
            d,
            x,
            b"adaptiveFlare\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).adaptive_flare,
        );
        (*g).rollout_parachutes = boo(
            d,
            x,
            b"rolloutParachutes\0" as *const u8 as *const ::core::ffi::c_char,
            (*g).rollout_parachutes,
        );
    }
    x = obj(d, root, b"calibration\0" as *const u8 as *const ::core::ffi::c_char);
    if x >= 0 as ::core::ffi::c_int {
        let mut a: *mut CalibrationSettings = &raw mut n.calibration;
        (*a).enable_passive_calibration = boo(
            d,
            x,
            b"enablePassiveCalibration\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).enable_passive_calibration,
        );
        (*a).enable_trajectory_calibration = boo(
            d,
            x,
            b"enableTrajectoryCalibration\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).enable_trajectory_calibration,
        );
        (*a).auto_apply_in_flight = boo(
            d,
            x,
            b"autoApplyInFlight\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).auto_apply_in_flight,
        );
        (*a).minimum_dynamic_pressure = num(
            d,
            x,
            b"minimumDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).minimum_dynamic_pressure,
        );
        (*a).maximum_sample_dynamic_pressure = num(
            d,
            x,
            b"maximumSampleDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).maximum_sample_dynamic_pressure,
        );
        (*a).maximum_sideslip = num(
            d,
            x,
            b"maximumSideslip\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).maximum_sideslip,
        );
        (*a).minimum_calibration_radar_altitude = num(
            d,
            x,
            b"minimumCalibrationRadarAltitude\0" as *const u8
                as *const ::core::ffi::c_char,
            (*a).minimum_calibration_radar_altitude,
        );
        (*a).maximum_calibration_dynamic_pressure = num(
            d,
            x,
            b"maximumCalibrationDynamicPressure\0" as *const u8
                as *const ::core::ffi::c_char,
            (*a).maximum_calibration_dynamic_pressure,
        );
        (*a).maximum_calibration_g_load = num(
            d,
            x,
            b"maximumCalibrationGLoad\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).maximum_calibration_g_load,
        );
        (*a).maximum_calibration_sink_rate = num(
            d,
            x,
            b"maximumCalibrationSinkRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).maximum_calibration_sink_rate,
        );
        (*a).abort_stall_fraction = num(
            d,
            x,
            b"abortStallFraction\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).abort_stall_fraction,
        );
        (*a).minimum_angle_of_attack = num(
            d,
            x,
            b"minimumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).minimum_angle_of_attack,
        );
        (*a).maximum_angle_of_attack = num(
            d,
            x,
            b"maximumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).maximum_angle_of_attack,
        );
        (*a).angle_of_attack_step = num(
            d,
            x,
            b"angleOfAttackStep\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).angle_of_attack_step,
        );
        (*a).settling_duration = num(
            d,
            x,
            b"settlingDuration\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).settling_duration,
        );
        (*a).sampling_duration = num(
            d,
            x,
            b"samplingDuration\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).sampling_duration,
        );
        (*a).trajectory_learning_rate = num(
            d,
            x,
            b"trajectoryLearningRate\0" as *const u8 as *const ::core::ffi::c_char,
            (*a).trajectory_learning_rate,
        );
    }
    if n.guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw mut n.vehicle.model_id as *mut ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        && obj(
            d,
            obj(d, root, b"vehicle\0" as *const u8 as *const ::core::ffi::c_char),
            b"terminalMaximumLiftAngleOfAttack\0" as *const u8
                as *const ::core::ffi::c_char,
        ) < 0 as ::core::ffi::c_int
    {
        n.vehicle.terminal_maximum_lift_angle_of_attack = n
            .vehicle
            .maximum_angle_of_attack;
    }
    landing_configuration_normalize(&raw mut n);
    *c = n;
    return true_0 != 0;
}
unsafe extern "C" fn model_vec3_json(mut w: *mut JsonWriter, mut v: Vector3) {
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    jw_number(w, v.x);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, v.y);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, v.z);
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn planet_model_replay_json(
    mut w: *mut JsonWriter,
    mut p: *const PlanetModel,
) {
    if w.is_null() || p.is_null() {
        if !w.is_null() {
            jw_null(w);
        }
        return;
    }
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"name\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*p).name as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"radius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gravitationalParameter\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).gravitational_parameter);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rotationalSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).rotational_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"atmosphereDepth\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).atmosphere_depth);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"surfaceDensity\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).surface_density);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"atmosphereAdiabaticIndex\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).atmosphere_adiabatic_index);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"northAxis\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    model_vec3_json(w, (*p).north_axis);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"primeMeridianAtEpoch\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    model_vec3_json(w, (*p).prime_meridian_at_epoch);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"epochUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).epoch_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"atmosphereSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    let mut count: size_t = (*p).atmosphere_sample_count;
    if count > LANDER_ATMOSPHERE_SAMPLE_MAX as size_t {
        count = LANDER_ATMOSPHERE_SAMPLE_MAX as size_t;
    }
    let mut i: size_t = 0 as size_t;
    while i < count {
        if i != 0 {
            jw_char(w, ',' as i32 as ::core::ffi::c_char);
        }
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"altitude\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).atmosphere_altitude[i as usize]);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"pressure\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).atmosphere_pressure[i as usize]);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"density\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).atmosphere_density[i as usize]);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
        i = i.wrapping_add(1);
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn landing_configuration_json(
    mut w: *mut JsonWriter,
    mut c: *const LandingConfiguration,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"connection\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"serialPort\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*c).connection.serial_port as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcHost\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*c).connection.rpc_host as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcPort\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).connection.rpc_port as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"baudRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).connection.baud_rate as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"timeoutMs\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).connection.timeout_ms as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"clientName\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*c).connection.client_name as *const ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"site\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"name\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*c).site.name as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"latitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).site.latitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"longitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).site.longitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"altitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).site.altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).site.runway_heading);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayLength\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).site.runway_length);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayWidth\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).site.runway_width);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"allowReciprocalRunway\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).site.allow_reciprocal_runway);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    let mut v: *const VehicleProfile = &raw const (*c).vehicle;
    jw_string(w, b"vehicle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"modelId\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*v).model_id as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedLiftToDrag\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).estimated_lift_to_drag);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"estimatedBallisticCoefficient\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).estimated_ballistic_coefficient);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).entry_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalMaximumLiftAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).terminal_maximum_lift_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumBankAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_bank_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumGLoad\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_g_load);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"minimumSafeSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).minimum_safe_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalApproachSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).final_approach_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"touchdownSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).touchdown_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"allowPoweredApproach\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*v).allow_powered_approach);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumApproachThrottle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_approach_throttle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakeActionGroup\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*v).airbrake_action_group as ::core::ffi::c_longlong);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    let mut g: *const GuidanceSettings = &raw const (*c).guidance;
    jw_string(w, b"guidance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"targetDeorbitCaptureRadius\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).target_deorbit_capture_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryInterfaceAltitudeMargin\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).entry_interface_altitude_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetEntryRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).target_entry_range);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryRangeReferenceSpeed\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).entry_range_reference_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"targetEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).target_entry_flight_path_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).maximum_entry_flight_path_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"targetPostBurnPeriapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).target_post_burn_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"deorbitMaximumThrottle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_maximum_throttle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitThrottleRampDuration\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_throttle_ramp_duration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitTimingUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_timing_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitThrustUncertaintyFraction\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_thrust_uncertainty_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitDeltaVUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_delta_v_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitMassUncertaintyFraction\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_mass_uncertainty_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitPositionUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_position_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitVelocityUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_velocity_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitPointingUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_pointing_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitAtmosphereUncertaintyFraction\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_atmosphere_uncertainty_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"deorbitRobustMinimumPassFraction\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).deorbit_robust_minimum_pass_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemInterfaceAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).taem_interface_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemInterfaceRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).taem_interface_range);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemForceHandoffSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).taem_force_handoff_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm304HandoffRadius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm304_handoff_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"mm304PerpendicularHeadingHalfWidth\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm304_perpendicular_heading_half_width);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm304HandoffAlongTrack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm304_handoff_along_track);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm304HandoffCrossTrack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm304_handoff_cross_track);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm305MinimumAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm305_min_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm305MaximumAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm305_max_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm305TargetMach\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm305_target_mach);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mm305MachHalfWidth\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).mm305_mach_half_width);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacAcquisitionAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).hac_acquisition_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacAcquisitionMach\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).hac_acquisition_mach);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacRadius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).hac_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacLookAheadAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).hac_look_ahead_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalApproachDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).final_approach_distance);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalAlignmentSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).final_alignment_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalGlideSlope\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).final_glide_slope);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemGlideSlope\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).taem_glide_slope);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gearDeploymentAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).gear_deployment_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"flareAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).flare_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"touchdownAimAlong\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).touchdown_aim_along);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"touchdownSinkRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).touchdown_sink_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"guidanceRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).guidance_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictionInterval\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).prediction_interval);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"sTurnMinimumLegDuration\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).s_turn_minimum_leg_duration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryRollRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).entry_roll_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryRollAcceleration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).entry_roll_acceleration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemRollRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).taem_roll_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"approachRollRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).approach_roll_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"useTimeWarp\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).use_time_warp);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemEnergyLaw\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).taem_energy_law);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"taemHeadingAlignmentCone\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).taem_hac_law);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"stsTrajectoryDelivery\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).sts_trajectory_delivery);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalSpeedbrake\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).terminal_speedbrake);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"adaptiveFlare\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).adaptive_flare);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rolloutParachutes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*g).rollout_parachutes);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"minimumPlanningLeadTime\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*g).minimum_planning_lead_time);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    let mut a: *const CalibrationSettings = &raw const (*c).calibration;
    jw_string(w, b"calibration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"enablePassiveCalibration\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).enable_passive_calibration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"enableTrajectoryCalibration\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).enable_trajectory_calibration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autoApplyInFlight\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).auto_apply_in_flight);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"minimumDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).minimum_dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumSampleDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).maximum_sample_dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumSideslip\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).maximum_sideslip);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"minimumCalibrationRadarAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).minimum_calibration_radar_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumCalibrationDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).maximum_calibration_dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumCalibrationGLoad\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).maximum_calibration_g_load);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumCalibrationSinkRate\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).maximum_calibration_sink_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"abortStallFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).abort_stall_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"minimumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).minimum_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).maximum_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"angleOfAttackStep\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).angle_of_attack_step);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"settlingDuration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).settling_duration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"samplingDuration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).sampling_duration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectoryLearningRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).trajectory_learning_rate);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn telemetry_init(mut t: *mut Telemetry) {
    memset(
        t as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Telemetry>() as size_t,
    );
    copystr(
        &raw mut (*t).vessel_name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"\xE2\x80\x94\0" as *const u8 as *const ::core::ffi::c_char,
    );
    (*t).speed_of_sound = 0.0f64;
    (*t).navball_speed_mode = SPEED_UNCHANGED;
    (*t).bank_effectiveness = 1.0f64;
    (*t).trajectory_lift_scale = (*t).bank_effectiveness;
    (*t).trajectory_drag_scale = (*t).trajectory_lift_scale;
    (*t).trajectory_density_scale = (*t).trajectory_drag_scale;
    copystr(
        &raw mut (*t).vessel_situation as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"Unknown\0" as *const u8 as *const ::core::ffi::c_char,
    );
}
#[no_mangle]
pub unsafe extern "C" fn guidance_command_init(mut c: *mut GuidanceCommand) {
    memset(
        c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<GuidanceCommand>() as size_t,
    );
    (*c).navball_speed_mode = SPEED_UNCHANGED;
    (*c).control_profile = PROFILE_ORBITAL;
}
#[no_mangle]
pub unsafe extern "C" fn calibration_snapshot_init(mut c: *mut CalibrationSnapshot) {
    memset(
        c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<CalibrationSnapshot>() as size_t,
    );
    (*c).state = CAL_IDLE;
    (*c).confidence = 0.2f64;
    (*c).planning_confidence = 0.18f64;
    copystr(
        &raw mut (*c).status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"Passive calibration is ready.\0" as *const u8 as *const ::core::ffi::c_char,
    );
}
#[no_mangle]
pub unsafe extern "C" fn landing_snapshot_init(
    mut s: *mut LandingSnapshot,
    mut p: *const VehicleProfile,
) {
    memset(
        s as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<LandingSnapshot>() as size_t,
    );
    (*s).connection_status = CONN_DISCONNECTED;
    (*s).phase = PHASE_IDLE;
    telemetry_init(&raw mut (*s).telemetry);
    guidance_command_init(&raw mut (*s).command);
    trajectory_init(&raw mut (*s).actual_trajectory);
    trajectory_init(&raw mut (*s).reference_trajectory);
    trajectory_init(&raw mut (*s).predicted_trajectory);
    copystr(
        &raw mut (*s).status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Not connected\0" as *const u8 as *const ::core::ffi::c_char,
    );
    calibration_snapshot_init(&raw mut (*s).calibration);
    if !p.is_null() {
        (*s).adaptive_vehicle_profile = *p;
    }
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_init(mut t: *mut Trajectory) {
    memset(
        t as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Trajectory>() as size_t,
    );
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_clear(mut t: *mut Trajectory) {
    free((*t).points as *mut ::core::ffi::c_void);
    memset(
        t as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Trajectory>() as size_t,
    );
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_append(
    mut t: *mut Trajectory,
    mut p: TrajectoryPoint,
) -> bool {
    if (*t).count == (*t).capacity {
        let mut n: size_t = if (*t).capacity != 0 {
            (*t).capacity.wrapping_mul(2 as size_t)
        } else {
            128 as size_t
        };
        let mut q: *mut TrajectoryPoint = realloc(
            (*t).points as *mut ::core::ffi::c_void,
            n.wrapping_mul(::core::mem::size_of::<TrajectoryPoint>() as size_t),
        ) as *mut TrajectoryPoint;
        if q.is_null() {
            return false_0 != 0;
        }
        (*t).points = q;
        (*t).capacity = n;
    }
    let fresh0 = (*t).count;
    (*t).count = (*t).count.wrapping_add(1);
    *(*t).points.offset(fresh0 as isize) = p;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_copy(
    mut d: *mut Trajectory,
    mut s: *const Trajectory,
) -> bool {
    trajectory_clear(d);
    if (*s).count == 0 {
        return true_0 != 0;
    }
    (*d).points = malloc(
        (*s).count.wrapping_mul(::core::mem::size_of::<TrajectoryPoint>() as size_t),
    ) as *mut TrajectoryPoint;
    if (*d).points.is_null() {
        return false_0 != 0;
    }
    memcpy(
        (*d).points as *mut ::core::ffi::c_void,
        (*s).points as *const ::core::ffi::c_void,
        (*s).count.wrapping_mul(::core::mem::size_of::<TrajectoryPoint>() as size_t),
    );
    (*d).capacity = (*s).count;
    (*d).count = (*d).capacity;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_plan_clear(mut p: *mut DeorbitPlan) {
    if p.is_null() {
        return;
    }
    trajectory_clear(&raw mut (*p).trajectory);
    memset(
        p as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
}
unsafe extern "C" fn nullable_number(
    mut w: *mut JsonWriter,
    mut has: bool,
    mut v: ::core::ffi::c_double,
) {
    if has {
        jw_number(w, v);
    } else {
        jw_null(w);
    };
}
unsafe extern "C" fn vec3_json(
    mut w: *mut JsonWriter,
    mut a: *const ::core::ffi::c_double,
) {
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(0 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(1 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(2 as ::core::ffi::c_int as isize));
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn vec4_json(
    mut w: *mut JsonWriter,
    mut a: *const ::core::ffi::c_double,
) {
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(0 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(1 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(2 as ::core::ffi::c_int as isize));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, *a.offset(3 as ::core::ffi::c_int as isize));
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn trajectory_json(
    mut w: *mut JsonWriter,
    mut t: *const Trajectory,
) {
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    let mut i: size_t = 0 as size_t;
    while i < (*t).count {
        if i != 0 {
            jw_char(w, ',' as i32 as ::core::ffi::c_char);
        }
        let mut p: *const TrajectoryPoint = (*t).points.offset(i as isize)
            as *mut TrajectoryPoint;
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"id\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        let mut id: [::core::ffi::c_char; 40] = [0; 40];
        snprintf(
            &raw mut id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 40]>() as size_t,
            b"c-%zu-%.0f\0" as *const u8 as *const ::core::ffi::c_char,
            i,
            (*p).ut * 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        jw_string(w, &raw mut id as *mut ::core::ffi::c_char);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"ut\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).ut);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"latitude\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).latitude);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"longitude\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).longitude);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"altitude\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).altitude);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"speed\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*p).speed);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_string(w, phase_string((*p).phase));
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"kind\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_string(
            w,
            if (*p).kind as ::core::ffi::c_uint
                == TRAJ_ACTUAL as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                b"actual\0" as *const u8 as *const ::core::ffi::c_char
            } else if (*p).kind as ::core::ffi::c_uint
                == TRAJ_REFERENCE as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                b"reference\0" as *const u8 as *const ::core::ffi::c_char
            } else {
                b"planned\0" as *const u8 as *const ::core::ffi::c_char
            },
        );
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
        i = i.wrapping_add(1);
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn vehicle_json(mut w: *mut JsonWriter, mut v: *const VehicleProfile) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"modelId\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*v).model_id as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedLiftToDrag\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).estimated_lift_to_drag);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"estimatedBallisticCoefficient\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).estimated_ballistic_coefficient);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).entry_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalMaximumLiftAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).terminal_maximum_lift_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumBankAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_bank_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"maximumGLoad\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_g_load);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"minimumSafeSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).minimum_safe_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalApproachSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).final_approach_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"touchdownSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).touchdown_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"allowPoweredApproach\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*v).allow_powered_approach);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"maximumApproachThrottle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*v).maximum_approach_throttle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakeActionGroup\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*v).airbrake_action_group as ::core::ffi::c_longlong);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn command_json(
    mut w: *mut JsonWriter,
    mut c: *const GuidanceCommand,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetPitch\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).target_pitch);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetAoA\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*c).has_target_aoa, (*c).target_aoa);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).target_heading);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetRoll\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).target_roll);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetThrottle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).target_throttle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"wheelSteering\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).wheel_steering);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gear\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).gear);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"brakes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).brakes);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).airbrakes);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"useInertialDirection\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).use_inertial_direction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"inertialDirection\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"x\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).inertial_direction.x);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"y\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).inertial_direction.y);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"z\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).inertial_direction.z);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autopilotEngaged\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).autopilot_engaged);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"headingControlEnabled\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).heading_control_enabled);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacControlTuning\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).hac_control_tuning);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPitchTuning\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).terminal_pitch_tuning);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalRollTrimEnabled\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).terminal_roll_trim_enabled);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"navballSpeedMode\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, speed_mode_string((*c).navball_speed_mode));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlProfile\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, profile_string((*c).control_profile));
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn physics_vector_json(
    mut w: *mut JsonWriter,
    mut valid: bool,
    mut v: Vector3,
) {
    if !valid {
        jw_null(w);
        return;
    }
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    jw_number(w, v.x);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, v.y);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(w, v.z);
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn telemetry_json(mut w: *mut JsonWriter, mut t: *const Telemetry) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsReferenceFrame\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"body-non-rotating-canonical-xzy\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsSampleValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).physics_sample_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"liftVector\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    physics_vector_json(w, (*t).has_force_vectors, (*t).lift_vector);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dragVector\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    physics_vector_json(w, (*t).has_force_vectors, (*t).drag_vector);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"centerOfMassRootReferenceFrame\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, b"root-part-krpc\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"centerOfMassRoot\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    physics_vector_json(w, (*t).has_center_of_mass_root, (*t).center_of_mass_root);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"centerOfMass\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    physics_vector_json(w, (*t).has_center_of_mass, (*t).center_of_mass);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedPhysicsObservedSeconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_physics_observed_seconds);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedPhysicsFallbackSeconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_physics_fallback_seconds);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedShadowGuidance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).predicted_shadow_guidance);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTerminalPolicyFeasible\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).predicted_terminal_policy_feasible);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTerminalSurvivabilityStressScore\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_terminal_survivability_stress_score);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedPhysicsRelativeUncertainty\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_physics_relative_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedUncertaintyScenarios\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_uncertainty_scenarios as ::core::ffi::c_double);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedRawPublishedPositionDelta30s\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_raw_published_position_delta_30s);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedRawPublishedPositionDelta60s\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_raw_published_position_delta_60s);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedRawPublishedPositionDelta120s\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_raw_published_position_delta_120s);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dryMass\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).dry_mass);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsConfidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsMassFlow\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_mass_flow);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsModelResidual\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_model_residual);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsModelResidualConfidence\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_model_residual_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsCertifiedUncertainty\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_certified_uncertainty);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsForceResidualPerQ\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    physics_vector_json(w, true_0 != 0, (*t).physics_force_residual_per_q);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsForceResidualSigmaPerQ\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    physics_vector_json(w, true_0 != 0, (*t).physics_force_residual_sigma_per_q);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsForceResidualConfidence\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_force_residual_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsAirbrakeModelAvailable\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).physics_airbrake_model_available);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsAirbrakeModelConfidence\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_airbrake_model_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_samples as ::core::ffi::c_double);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"physicsLiveSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_live_samples as ::core::ffi::c_double);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsAirbrakeDragAccel\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).physics_airbrake_drag_accel);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsAngularAuthority\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if i != 0 {
            jw_char(w, ',' as i32 as ::core::ffi::c_char);
        }
        jw_number(w, (*t).physics_authority[i as usize]);
        i += 1;
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"physicsAuthorityConfidence\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 3 as ::core::ffi::c_int {
        if i_0 != 0 {
            jw_char(w, ',' as i32 as ::core::ffi::c_char);
        }
        jw_number(w, (*t).physics_authority_confidence[i_0 as usize]);
        i_0 += 1;
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"ut\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"vesselName\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*t).vessel_name as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"latitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).latitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"longitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).longitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"meanAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).mean_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"radarAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).radar_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"verticalSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).vertical_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"horizontalSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).horizontal_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"surfaceSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).surface_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trueAirSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).true_air_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"atmosphericDensity\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).atmospheric_density);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"speedOfSound\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).speed_of_sound);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mach\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).mach);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"heading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).heading);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"groundTrackHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).ground_track_heading);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"courseToSiteError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).course_to_site_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"pitch\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).pitch);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"roll\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).roll);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"pitchRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).pitch_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rollRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).roll_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"headingRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).heading_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"angleOfAttackRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_angle_of_attack_rate, (*t).angle_of_attack_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bodyPitchRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_body_pitch_rate, (*t).body_pitch_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bodyRollRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_body_roll_rate, (*t).body_roll_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bodyYawRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_body_yaw_rate, (*t).body_yaw_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"courseRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_course_rate, (*t).course_rate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autopilotError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).autopilot_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"commandPitchError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_command_pitch_error, (*t).command_pitch_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"commandRollError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_command_roll_error, (*t).command_roll_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"commandHeadingError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_command_heading_error, (*t).command_heading_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"attitudeQuaternion\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_attitude_quaternion {
        vec4_json(
            w,
            &raw const (*t).attitude_quaternion as *const ::core::ffi::c_double,
        );
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"attitudeReferenceFrame\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).attitude_reference_frame[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(
            w,
            &raw const (*t).attitude_reference_frame as *const ::core::ffi::c_char,
        );
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autopilotPitchPIDGains\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_pid_gains {
        vec3_json(
            w,
            &raw const (*t).autopilot_pitch_pid_gains as *const ::core::ffi::c_double,
        );
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autopilotRollPIDGains\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_pid_gains {
        vec3_json(
            w,
            &raw const (*t).autopilot_roll_pid_gains as *const ::core::ffi::c_double,
        );
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autopilotYawPIDGains\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_pid_gains {
        vec3_json(
            w,
            &raw const (*t).autopilot_yaw_pid_gains as *const ::core::ffi::c_double,
        );
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"navballSpeedMode\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, speed_mode_string((*t).navball_speed_mode));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"angleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"sideslip\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).sideslip);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dynamicPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"staticPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).static_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gForce\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).g_force);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"stallFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).stall_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"liftForce\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).lift_force);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dragForce\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).drag_force);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mass\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).mass);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"availableThrust\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).available_thrust);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"currentThrust\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).current_thrust);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"apoapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).apoapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"periapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"orbitPeriod\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).orbit_period);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"throttle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).throttle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlPitch\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_controls, (*t).control_pitch);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlRoll\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_controls, (*t).control_roll);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlYaw\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_controls, (*t).control_yaw);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"availablePitchTorque\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_torque, (*t).available_pitch_torque);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"availableRollTorque\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_torque, (*t).available_roll_torque);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"availableYawTorque\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_torque, (*t).available_yaw_torque);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"pitchMomentOfInertia\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_inertia, (*t).pitch_moment_of_inertia);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rollMomentOfInertia\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_inertia, (*t).roll_moment_of_inertia);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"yawMomentOfInertia\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_inertia, (*t).yaw_moment_of_inertia);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"loopWallDeltaMilliseconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_loop_wall_delta, (*t).loop_wall_delta_ms);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"telemetryLatencyMilliseconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_telemetry_latency, (*t).telemetry_latency_ms);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"guidanceComputeMilliseconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_guidance_compute, (*t).guidance_compute_ms);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"applyLatencyMilliseconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_apply_latency, (*t).apply_latency_ms);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"controlLoopMilliseconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*t).has_control_loop, (*t).control_loop_ms);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcReadCalls\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_rpc_budget {
        jw_integer(w, (*t).rpc_read_calls as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcReadWireRequests\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_rpc_budget {
        jw_integer(w, (*t).rpc_read_wire_requests as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcApplyCalls\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_rpc_budget {
        jw_integer(w, (*t).rpc_apply_calls as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcApplyWireRequests\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_rpc_budget {
        jw_integer(w, (*t).rpc_apply_wire_requests as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcTotalCalls\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_rpc_budget {
        jw_integer(w, (*t).rpc_total_calls as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rpcTotalWireRequests\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_rpc_budget {
        jw_integer(w, (*t).rpc_total_wire_requests as ::core::ffi::c_longlong);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gear\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).gear);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"brakes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).brakes);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).has_airbrakes {
        jw_bool(w, (*t).airbrakes);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hasMainGearGrounded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).has_main_gear_grounded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"mainGearGrounded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).main_gear_grounded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hasNoseGearGrounded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).has_nose_gear_grounded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"noseGearGrounded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*t).nose_gear_grounded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"vesselSituation\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*t).vessel_situation as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rangeToSite\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).range_to_site);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bearingToSite\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).bearing_to_site);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"headingError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).heading_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayAlongTrack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).runway_along_track);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayCrossTrack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).runway_cross_track);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"flightPathAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).flight_path_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedLiftToDrag\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).estimated_lift_to_drag);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"estimatedBallisticCoefficient\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).estimated_ballistic_coefficient);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"aerodynamicConfidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).aerodynamic_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"calibratedBestGlideAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).calibrated_best_glide_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"calibratedStallSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).calibrated_stall_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedMissDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*t).predicted_miss_distance.is_finite() as i32 != 0,
        (*t).predicted_miss_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedTAEMDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).predicted_taem_distance.is_finite() as i32 != 0 {
        jw_number(w, (*t).predicted_taem_distance);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).predicted_taem_range_error.is_finite() as i32 != 0 {
        jw_number(w, (*t).predicted_taem_range_error);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedEntryRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*t).predicted_entry_range.is_finite() as i32 != 0,
        (*t).predicted_entry_range,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*t).predicted_entry_flight_path_angle.is_finite() as i32 != 0,
        (*t).predicted_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedTAEMSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).predicted_taem_speed.is_finite() as i32 != 0 {
        jw_number(w, (*t).predicted_taem_speed);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTAEMEnergyError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*t).predicted_taem_energy_error.is_finite() as i32 != 0 {
        jw_number(w, (*t).predicted_taem_energy_error);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedPeakDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*t).predicted_peak_dynamic_pressure.is_finite() as i32 != 0,
        (*t).predicted_peak_dynamic_pressure,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedPeakGLoad\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*t).predicted_peak_g_load.is_finite() as i32 != 0,
        (*t).predicted_peak_g_load,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedSTurnReversals\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).predicted_s_turn_reversals);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectoryDensityScale\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_density_scale);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectoryDragScale\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_drag_scale);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectoryLiftScale\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_lift_scale);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bankEffectiveness\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).bank_effectiveness);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"trajectoryCalibrationConfidence\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_calibration_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"trajectoryAltitudeResidual\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_altitude_residual);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"trajectorySpeedResidual\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_speed_residual);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"trajectoryRangeResidual\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).trajectory_range_residual);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"energyExcessRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*t).energy_excess_range);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn plan_json(mut w: *mut JsonWriter, mut p: *const DeorbitPlan) {
    if p.is_null() {
        jw_null(w);
        return;
    }
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"createdUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).created_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).burn_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"deltaV\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).delta_v);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnNormalAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).burn_normal_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedBurnDuration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).estimated_burn_duration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedTAEMDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_taem_distance.is_finite() as i32 != 0,
        (*p).predicted_taem_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_taem_range_error.is_finite() as i32 != 0,
        (*p).predicted_taem_range_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedClosestDistance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_closest_distance.is_finite() as i32 != 0,
        (*p).predicted_closest_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedEntryRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_entry_range.is_finite() as i32 != 0,
        (*p).predicted_entry_range,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_entry_flight_path_angle.is_finite() as i32 != 0,
        (*p).predicted_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedPostBurnPeriapsisAltitude\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).predicted_post_burn_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"nominalCaptureAchieved\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).nominal_capture_achieved);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).robustness_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalModelSnapshotId\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).terminal_model_snapshot_id as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalReplayEvaluated\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_replay_evaluated);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalInletQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_inlet_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalNativePathCommitted\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_native_path_committed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalFinalApproachCaptured\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_final_approach_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalGuidanceAborted\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_guidance_aborted);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalInletEnergyMargin\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).terminal_inlet_energy_margin.is_finite() as i32 != 0,
        (*p).terminal_inlet_energy_margin,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessScenarios\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).robustness_scenarios as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessPassed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).robustness_passed as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessUnsafe\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).robustness_unsafe as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessPassFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).robustness_pass_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recoveryPassed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).recovery_passed as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recoveryPassFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).recovery_pass_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"executionQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).execution_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"executionDegraded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).execution_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCaseClosestDistance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_closest_distance.is_finite() as i32 != 0,
        (*p).worst_case_closest_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCaseTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_taem_range_error.is_finite() as i32 != 0,
        (*p).worst_case_taem_range_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCaseEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_entry_flight_path_angle.is_finite() as i32 != 0,
        (*p).worst_case_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCasePostBurnPeriapsisAltitude\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_post_burn_periapsis_altitude.is_finite() as i32 != 0,
        (*p).worst_case_post_burn_periapsis_altitude,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCasePeakDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_peak_dynamic_pressure.is_finite() as i32 != 0,
        (*p).worst_case_peak_dynamic_pressure,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"worstCasePeakGLoad\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_peak_g_load.is_finite() as i32 != 0,
        (*p).worst_case_peak_g_load,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffCaptureQualified\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).live_cutoff_capture_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffPeriapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).live_cutoff_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffClosestDistance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).live_cutoff_closest_distance.is_finite() as i32 != 0,
        (*p).live_cutoff_closest_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).live_cutoff_entry_flight_path_angle.is_finite() as i32 != 0,
        (*p).live_cutoff_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"achievedStateVerified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).achieved_state_verified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"achievedStateCaptureQualified\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).achieved_state_capture_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"achievedPostBurnPeriapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).achieved_post_burn_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetCaptureAchieved\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).target_capture_achieved);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"confidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectory\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    trajectory_json(w, &raw const (*p).trajectory);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"notes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    if (*p).note[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*p).note as *const ::core::ffi::c_char);
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn plan_log_json(mut w: *mut JsonWriter, mut p: *const DeorbitPlan) {
    if p.is_null() {
        jw_null(w);
        return;
    }
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"createdUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).created_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).burn_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"deltaV\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).delta_v);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnNormalAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).burn_normal_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedBurnDuration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).estimated_burn_duration);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"planningMass\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).planning_mass);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"planningAvailableThrust\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).planning_available_thrust);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedTAEMDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_taem_distance.is_finite() as i32 != 0,
        (*p).predicted_taem_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_taem_range_error.is_finite() as i32 != 0,
        (*p).predicted_taem_range_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedClosestDistance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_closest_distance.is_finite() as i32 != 0,
        (*p).predicted_closest_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedEntryRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_entry_range.is_finite() as i32 != 0,
        (*p).predicted_entry_range,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).predicted_entry_flight_path_angle.is_finite() as i32 != 0,
        (*p).predicted_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedPostBurnPeriapsisAltitude\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).predicted_post_burn_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"nominalCaptureAchieved\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).nominal_capture_achieved);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).robustness_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalModelSnapshotId\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).terminal_model_snapshot_id as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalReplayEvaluated\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_replay_evaluated);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalInletQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_inlet_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalNativePathCommitted\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_native_path_committed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalFinalApproachCaptured\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_final_approach_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalGuidanceAborted\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).terminal_guidance_aborted);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalInletEnergyMargin\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).terminal_inlet_energy_margin.is_finite() as i32 != 0,
        (*p).terminal_inlet_energy_margin,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessScenarios\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).robustness_scenarios as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessPassed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).robustness_passed as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessUnsafe\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).robustness_unsafe as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"robustnessPassFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).robustness_pass_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recoveryPassed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).recovery_passed as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recoveryPassFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).recovery_pass_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"executionQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).execution_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"executionDegraded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).execution_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryStageQualified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).entry_stage_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryStageScenarios\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).entry_stage_scenarios as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryStagePassed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).entry_stage_passed as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryStagePassFraction\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).entry_stage_pass_fraction);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCaseClosestDistance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_closest_distance.is_finite() as i32 != 0,
        (*p).worst_case_closest_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCaseTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_taem_range_error.is_finite() as i32 != 0,
        (*p).worst_case_taem_range_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCaseEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_entry_flight_path_angle.is_finite() as i32 != 0,
        (*p).worst_case_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCasePostBurnPeriapsisAltitude\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_post_burn_periapsis_altitude.is_finite() as i32 != 0,
        (*p).worst_case_post_burn_periapsis_altitude,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"worstCasePeakDynamicPressure\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_peak_dynamic_pressure.is_finite() as i32 != 0,
        (*p).worst_case_peak_dynamic_pressure,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"worstCasePeakGLoad\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).worst_case_peak_g_load.is_finite() as i32 != 0,
        (*p).worst_case_peak_g_load,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffCaptureQualified\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).live_cutoff_capture_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffPeriapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).live_cutoff_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffClosestDistance\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).live_cutoff_closest_distance.is_finite() as i32 != 0,
        (*p).live_cutoff_closest_distance,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"liveCutoffEntryFlightPathAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*p).live_cutoff_entry_flight_path_angle.is_finite() as i32 != 0,
        (*p).live_cutoff_entry_flight_path_angle,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"achievedStateVerified\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).achieved_state_verified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"achievedStateCaptureQualified\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).achieved_state_capture_qualified);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"achievedPostBurnPeriapsisAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).achieved_post_burn_periapsis_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetCaptureAchieved\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*p).target_capture_achieved);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"confidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*p).confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectoryPointCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*p).trajectory.count as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"notes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    if (*p).note[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*p).note as *const ::core::ffi::c_char);
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn applied_command_json(
    mut w: *mut JsonWriter,
    mut a: *const KRPCApplyResult,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"applied\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).applied);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"autopilotEngaged\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).autopilot_engaged);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"referenceFrame\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*a).reference_frame[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*a).reference_frame as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"speedMode\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*a).speed_mode[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*a).speed_mode as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlProfile\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*a).control_profile[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*a).control_profile as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetPitch\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).target_pitch);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).target_heading);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetRoll\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).target_roll);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"throttle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).throttle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"wheelSteering\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*a).wheel_steering);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gear\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).gear);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"brakes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).brakes);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakes\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*a).airbrakes);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"actuatorFeedback\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*a).has_actuator_feedback {
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"pitch\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_pitch);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"roll\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_roll);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"yaw\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_yaw);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlDiagnostics\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*a).has_control_diagnostics {
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"revision\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        if (*a).control_revision[0 as ::core::ffi::c_int as usize] != 0 {
            jw_string(w, &raw const (*a).control_revision as *const ::core::ffi::c_char);
        } else {
            jw_null(w);
        }
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"targetAoA\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_target_aoa);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"measuredAoA\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_measured_aoa);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"aoaRate\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_aoa_rate);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"rollRate\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_roll_rate);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"bodyPitchRate\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_body_pitch_rate);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(
            w,
            b"bodyPitchRateAvailable\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_bool(w, (*a).control_body_pitch_rate_available);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"bodyRollRate\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_body_roll_rate);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(
            w,
            b"bodyRollRateAvailable\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_bool(w, (*a).control_body_roll_rate_available);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"bodyYawRate\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_body_yaw_rate);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(
            w,
            b"bodyYawRateAvailable\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_bool(w, (*a).control_body_yaw_rate_available);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"pitchError\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_pitch_error);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"pitchTrim\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_pitch_trim);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"pitchAuthority\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_pitch_authority);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"pitchAeroFraction\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_pitch_aero_fraction);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"rollAuthority\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_roll_authority);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"rollRawAuthority\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_roll_raw_authority);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"rollAeroFraction\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_roll_aero_fraction);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"targetRollRate\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_target_roll_rate);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"betaYawGain\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_beta_yaw_gain);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"betaRollCoupling\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_beta_roll_coupling);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"betaConfidence\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_beta_confidence);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"yawCommand\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*a).control_yaw_command);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn command_saturated(mut s: *const LandingSnapshot) -> bool {
    if !(*s).command_applied {
        return false_0 != 0;
    }
    let mut c: *const GuidanceCommand = &raw const (*s).command;
    let mut a: *const KRPCApplyResult = &raw const (*s).applied_command;
    if (*c).target_throttle != (*a).throttle
        || (*c).wheel_steering != (*a).wheel_steering
    {
        return true_0 != 0;
    }
    if (*c).autopilot_engaged as ::core::ffi::c_int != 0 && !(*c).use_inertial_direction
        && ((*c).target_pitch != (*a).target_pitch
            || norm_signed_deg((*c).target_heading - (*a).target_heading) != 0.0f64
            || norm_signed_deg((*c).target_roll - (*a).target_roll) != 0.0f64)
    {
        return true_0 != 0;
    }
    return (*c).gear as ::core::ffi::c_int != (*a).gear as ::core::ffi::c_int
        || (*c).brakes as ::core::ffi::c_int != (*a).brakes as ::core::ffi::c_int
        || (*c).airbrakes as ::core::ffi::c_int != (*a).airbrakes as ::core::ffi::c_int;
}
unsafe extern "C" fn entry_exec_json(
    mut w: *mut JsonWriter,
    mut e: *const EntryExecTelemetry,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"initialized\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).initialized);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"complete\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).entry_complete);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, entry_phase_string((*e).active_phase));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"lastTransition\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, entry_transition_reason_string((*e).last_transition_reason));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"transitionCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*e).transition_count as ::core::ffi::c_longlong);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn entry_feedback_json(
    mut w: *mut JsonWriter,
    mut f: *const EntryGuidanceFeedback,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"valid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*f).valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"lateralValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*f).lateral_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"ut\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0 && (*f).ut.is_finite() as i32 != 0,
        (*f).ut,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"confidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0 && (*f).confidence.is_finite() as i32 != 0,
        (*f).confidence,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"measuredDrag\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).measured_drag_accel.is_finite() as i32 != 0,
        (*f).measured_drag_accel,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"referenceDrag\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).reference_drag_accel.is_finite() as i32 != 0,
        (*f).reference_drag_accel,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dragError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).drag_error_accel.is_finite() as i32 != 0,
        (*f).drag_error_accel,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedAchievableRange\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).predicted_achievable_range.is_finite() as i32 != 0,
        (*f).predicted_achievable_range,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"requiredRangeToGo\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).required_range_to_go.is_finite() as i32 != 0,
        (*f).required_range_to_go,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"energyMargin\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).energy_margin.is_finite() as i32 != 0,
        (*f).energy_margin,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rangeErrorScale\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).range_error_scale.is_finite() as i32 != 0,
        (*f).range_error_scale,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"requiredVerticalLift\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).required_vertical_lift_accel.is_finite() as i32 != 0,
        (*f).required_vertical_lift_accel,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"crossrangeError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).lateral_valid as ::core::ffi::c_int != 0
            && (*f).crossrange_error.is_finite() as i32 != 0,
        (*f).crossrange_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"crossrangeRate\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).lateral_valid as ::core::ffi::c_int != 0
            && (*f).crossrange_rate.is_finite() as i32 != 0,
        (*f).crossrange_rate,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"projectedCrossrangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).lateral_valid as ::core::ffi::c_int != 0
            && (*f).projected_crossrange_error.is_finite() as i32 != 0,
        (*f).projected_crossrange_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"crossrangeCorridor\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).lateral_valid as ::core::ffi::c_int != 0
            && (*f).crossrange_corridor.is_finite() as i32 != 0,
        (*f).crossrange_corridor,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"energyBankMagnitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).lateral_valid as ::core::ffi::c_int != 0
            && (*f).energy_bank_magnitude_deg.is_finite() as i32 != 0,
        (*f).energy_bank_magnitude_deg,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"commandedBank\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0
            && (*f).commanded_bank_deg.is_finite() as i32 != 0,
        (*f).commanded_bank_deg,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bankSign\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*f).valid as ::core::ffi::c_int != 0 && (*f).bank_sign.is_finite() as i32 != 0,
        (*f).bank_sign,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"reversalRequested\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*f).reversal_requested);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn taem_exec_json(
    mut w: *mut JsonWriter,
    mut e: *const TaemExecTelemetry,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"initialized\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).initialized);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"ownershipLatched\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).ownership_latched);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"complete\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).taem_complete);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, taem_phase_string((*e).active_phase));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"lastTransition\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, taem_transition_reason_string((*e).last_transition_reason));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"transitionCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*e).transition_count as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recoveryActive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).recovery_active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recoveryReason\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, taem_recovery_reason_string((*e).recovery_reason));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalEvaluationValid\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_evaluation.valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPolicyFeasible\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_evaluation.feasible);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalBlockReason\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        taem_terminal_block_reason_string((*e).terminal_evaluation.block_reason),
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalContract\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"valid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_contract.valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"pathCommitted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_contract.path_committed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rangeToGo\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.range_to_go);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rangeMargin\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.range_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dynamicPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dynamicPressureMargin\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.dynamic_pressure_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"speedbrakeRequired\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_contract.speedbrake_required);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"speedbrakeAvailable\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_contract.speedbrake_available);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"speedbrakeDynamicPressureMargin\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.speedbrake_dynamic_pressure_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"altitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"altitudeMargin\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.altitude_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"flightPathAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.flight_path_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"flightPathAngleMargin\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.flight_path_angle_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"specificEnergy\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.specific_energy);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"specificEnergyMargin\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.specific_energy_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"responseTimeAvailable\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.response_time_available);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"responseTimeRequired\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*e).terminal_contract.response_time_required);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"attitudeResponseQualified\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*e).terminal_contract.attitude_response_qualified);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn active_runway_json(
    mut w: *mut JsonWriter,
    mut s: *const LandingSnapshot,
) {
    if !(*s).has_active_runway {
        jw_null(w);
        return;
    }
    let mut site: *const LandingSite = &raw const (*s).active_runway;
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"configuredSiteName\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*site).name as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"latitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*site).latitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"longitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*site).longitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"altitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*site).altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*site).runway_heading);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayLength\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*site).runway_length);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"runwayWidth\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*site).runway_width);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalApproachDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).active_final_distance);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalGlideSlope\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).active_final_slope);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn snapshot_json(
    mut w: *mut JsonWriter,
    mut s: *const LandingSnapshot,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"schemaVersion\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, 2 as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recordType\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, b"snapshot\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"logSequence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).log_sequence as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"tickSequence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).tick_sequence as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"wallMonotonicSeconds\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).wall_monotonic_seconds);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"simulationDt\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).simulation_dt);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"connectionStatus\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, connection_status_string((*s).connection_status));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, phase_string((*s).phase));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"telemetry\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    telemetry_json(w, &raw const (*s).telemetry);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"vehicleState\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_vehicle_state {
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"ut\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.ut);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"position\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_char(w, '[' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.position.x);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.position.y);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.position.z);
        jw_char(w, ']' as i32 as ::core::ffi::c_char);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"velocity\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_char(w, '[' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.velocity.x);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.velocity.y);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.velocity.z);
        jw_char(w, ']' as i32 as ::core::ffi::c_char);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"mass\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.mass);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"attitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_attitude_quaternion {
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"quaternion\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        vec4_json(
            w,
            &raw const (*s).attitude_quaternion as *const ::core::ffi::c_double,
        );
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"referenceFrame\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_string(
            w,
            &raw const (*s).attitude_reference_frame as *const ::core::ffi::c_char,
        );
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"guidanceState\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"activeRunway\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    active_runway_json(w, s);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnStarted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_has_burn_started);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnCompleted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_burn_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"atmosphereCrossed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_atmosphere_crossed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_final_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakesDeployed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_airbrakes_deployed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacSideSelected\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_side_selected);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"deliveredDeltaV\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_delivered_delta_v);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnActiveElapsed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_burn_active_elapsed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"sTurnSign\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_s_turn_sign);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacSide\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_side);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCompleted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacProgressValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_progress_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacArcRemaining\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_remaining);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"minimumTurnRadius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_minimum_turn_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryLegElapsed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_entry_leg_elapsed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacTransitionActive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_transition_active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacRadius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCircuitCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).guidance_hac_circuit_count as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCircuitGlideSlope\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_circuit_slope);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalPredictionValid\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_prediction_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalCandidateValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathCommitted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_committed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathSelected\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(
        w,
        (*s).guidance_terminal_committed as ::core::ffi::c_int != 0
            || (*s).guidance_hac_side_selected as ::core::ffi::c_int != 0,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(
        w,
        (*s).guidance_hac_captured as ::core::ffi::c_int != 0
            && !(*s).guidance_hac_transition_active,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathComplete\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathRemaining\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_remaining);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalCandidateRadius\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalCandidateAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalCandidateSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalReferenceFPA\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_reference_fpa);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalBlend\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_mix);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacTransitionProgress\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_transition_progress);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"commandedCourseRateEstimate\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_commanded_course_rate_estimate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"energyLaw\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"active\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_taem_energy_law_active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dragLevel\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_taem_energy_law_active as ::core::ffi::c_int != 0
            && (*s).guidance_taem_energy_drag_level.is_finite() as i32 != 0,
        (*s).guidance_taem_energy_drag_level,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"pathToGo\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_taem_energy_law_active as ::core::ffi::c_int != 0
            && (*s).guidance_taem_energy_path_to_go.is_finite() as i32 != 0,
        (*s).guidance_taem_energy_path_to_go,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"requiredPath\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_taem_energy_law_active as ::core::ffi::c_int != 0
            && (*s).guidance_taem_energy_required_path.is_finite() as i32 != 0,
        (*s).guidance_taem_energy_required_path,
    );
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryFeedback\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    entry_feedback_json(w, &raw const (*s).guidance_entry_feedback);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_plan_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTerminalReady\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_plan_terminal_ready);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTargetBank\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_target_bank,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTargetAoA\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_target_aoa,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTargetHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_target_heading,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanSegmentRemaining\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_segment_remaining,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanCost\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*s).guidance_entry_plan_valid, (*s).guidance_entry_plan_cost);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_taem_range_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTAEMSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid as ::core::ffi::c_int != 0
            && (*s).guidance_entry_plan_taem_speed.is_finite() as i32 != 0,
        (*s).guidance_entry_plan_taem_speed,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanTAEMEnergyError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid as ::core::ffi::c_int != 0
            && (*s).guidance_entry_plan_taem_energy_error.is_finite() as i32 != 0,
        (*s).guidance_entry_plan_taem_energy_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanPredictedReversals\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(
        w,
        (*s).guidance_entry_plan_predicted_reversals as ::core::ffi::c_longlong,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalScheduled\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_reversal_scheduled);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalIsFinal\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_reversal_is_final);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryReversalTimeRemaining\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_reversal_scheduled,
        (*s).guidance_entry_reversal_time_remaining,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_reversal_scheduled,
        (*s).guidance_entry_reversal_range,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalSign\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_reversal_scheduled,
        (*s).guidance_entry_reversal_sign,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryExecutive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    entry_exec_json(w, &raw const (*s).guidance_entry_exec);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemExecutive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    taem_exec_json(w, &raw const (*s).guidance_taem_exec);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"decisionReason\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).decision_reason[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*s).decision_reason as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"decisionTrace\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, phase_string((*s).phase));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"controlProfile\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, profile_string((*s).command.control_profile));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"automationEngaged\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).automation_engaged);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"paused\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).paused);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnStarted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_has_burn_started);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnCompleted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_burn_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"atmosphereCrossed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_atmosphere_crossed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_final_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"deliveredDeltaV\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_delivered_delta_v);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryLegElapsed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_entry_leg_elapsed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"inputs\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rangeToSite\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.range_to_site);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"courseToSiteError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.course_to_site_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"flightPathAngle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.flight_path_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"energyExcessRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.energy_excess_range);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedMissDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.predicted_miss_distance);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"predictedTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.predicted_taem_range_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"dynamicPressure\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.dynamic_pressure);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"gForce\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.g_force);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"angleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"roll\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.roll);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"heading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).telemetry.heading);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"status\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*s).status_message as *const ::core::ffi::c_char);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"command\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    command_json(w, &raw const (*s).command);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"commandLifecycle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"computedWallSeconds\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).command_computed_wall_seconds);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"applyAttempted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).command_apply_attempted);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"applyStartedWallSeconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).command_apply_attempted,
        (*s).command_apply_started_wall_seconds,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"applyFinishedWallSeconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).command_apply_attempted,
        (*s).command_apply_finished_wall_seconds,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"applied\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).command_applied);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"saturatedOrNormalized\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, command_saturated(s));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"appliedCommand\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).command_apply_attempted {
        applied_command_json(w, &raw const (*s).applied_command);
    } else {
        jw_null(w);
    }
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"plan\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    plan_json(w, (*s).plan);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"actualTrajectory\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    trajectory_json(w, &raw const (*s).actual_trajectory);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"referenceTrajectory\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    trajectory_json(w, &raw const (*s).reference_trajectory);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"predictedTrajectory\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    trajectory_json(w, &raw const (*s).predicted_trajectory);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"statusMessage\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*s).status_message as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"warningMessage\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_warning {
        jw_string(w, &raw const (*s).warning_message as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"lastError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_error {
        jw_string(w, &raw const (*s).last_error as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"automationEngaged\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).automation_engaged);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"paused\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).paused);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"calibration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    let mut c: *const CalibrationSnapshot = &raw const (*s).calibration;
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"state\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        if (*c).state as ::core::ffi::c_uint
            == CAL_IDLE as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            b"Idle\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            cal_state_string((*c).state)
        },
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"active\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"progress\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).progress);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).target_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"acceptedSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).accepted_samples as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rejectedSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).rejected_samples as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"confidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"planningConfidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).planning_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bestGlideAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).best_glide_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedStallSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).estimated_stall_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"status\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*c).status as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"warning\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*c).has_warning {
        jw_string(w, &raw const (*c).warning as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"adaptiveVehicleProfile\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    vehicle_json(w, &raw const (*s).adaptive_vehicle_profile);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn snapshot_log_json(
    mut w: *mut JsonWriter,
    mut s: *const LandingSnapshot,
) {
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"schemaVersion\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, 3 as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"recordType\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, b"snapshot\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"logSequence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).log_sequence as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"tickSequence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).tick_sequence as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"wallMonotonicSeconds\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).wall_monotonic_seconds);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"simulationDt\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).simulation_dt);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"connectionStatus\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, connection_status_string((*s).connection_status));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"phase\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, phase_string((*s).phase));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"telemetry\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    telemetry_json(w, &raw const (*s).telemetry);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"vehicleState\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_vehicle_state {
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"ut\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.ut);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"position\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_char(w, '[' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.position.x);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.position.y);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.position.z);
        jw_char(w, ']' as i32 as ::core::ffi::c_char);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"velocity\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_char(w, '[' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.velocity.x);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.velocity.y);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.velocity.z);
        jw_char(w, ']' as i32 as ::core::ffi::c_char);
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"mass\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_number(w, (*s).vehicle_state.mass);
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"attitude\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_attitude_quaternion {
        jw_char(w, '{' as i32 as ::core::ffi::c_char);
        jw_string(w, b"quaternion\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        vec4_json(
            w,
            &raw const (*s).attitude_quaternion as *const ::core::ffi::c_double,
        );
        jw_char(w, ',' as i32 as ::core::ffi::c_char);
        jw_string(w, b"referenceFrame\0" as *const u8 as *const ::core::ffi::c_char);
        jw_char(w, ':' as i32 as ::core::ffi::c_char);
        jw_string(
            w,
            &raw const (*s).attitude_reference_frame as *const ::core::ffi::c_char,
        );
        jw_char(w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"guidanceState\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnStarted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_has_burn_started);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnCompleted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_burn_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"atmosphereCrossed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_atmosphere_crossed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"finalCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_final_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"airbrakesDeployed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_airbrakes_deployed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacSideSelected\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_side_selected);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"deliveredDeltaV\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_delivered_delta_v);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"burnActiveElapsed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_burn_active_elapsed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"sTurnSign\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_s_turn_sign);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacSide\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_side);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_captured);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCompleted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacProgressValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_progress_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacTransitionActive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_transition_active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacArcRemaining\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_remaining);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacRadius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCircuitCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).guidance_hac_circuit_count as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacCircuitGlideSlope\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_circuit_slope);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalReentryAfterUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_reentry_after_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalPredictionValid\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_prediction_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalCandidateValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathCommitted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_committed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathSelected\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(
        w,
        (*s).guidance_terminal_committed as ::core::ffi::c_int != 0
            || (*s).guidance_hac_side_selected as ::core::ffi::c_int != 0,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathCaptured\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(
        w,
        (*s).guidance_hac_captured as ::core::ffi::c_int != 0
            && !(*s).guidance_hac_transition_active,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathComplete\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_completed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalPathTransitionActive\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_transition_active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalPathRemaining\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_remaining);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalCandidateRadius\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"terminalCandidateAltitude\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_altitude);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalCandidateSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateKind\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).guidance_terminal_candidate_kind as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateGeometryDegraded\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_geometry_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateEnergyDegraded\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_energy_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateShellDegraded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_shell_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateSide\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_side);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateSlope\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_slope);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateCurveLength\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_curve_length);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateLeadLength\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_lead_length);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateArcRemaining\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_arc_remaining);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateFinalDistance\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_final_distance);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateQuality\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_quality);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateArrivalUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_arrival_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateCourse\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_course);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidatePeakLateral\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_peak_lateral);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidatePeakRateRatio\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_peak_rate_ratio);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateExitSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_exit_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateTrackingEvaluated\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_tracking_evaluated);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateTrackingError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_tracking_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateTrackingCourseError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_tracking_course_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateTrackingEndpointError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_tracking_endpoint_error);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateTrackingMargin\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_candidate_tracking_margin);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidatePathDegraded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_path_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"candidateControlDegraded\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_control_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateRateDegraded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_rate_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"candidateEndDegraded\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_terminal_candidate_end_degraded);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalReferenceFPA\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_reference_fpa);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"terminalBlend\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_terminal_mix);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacTransitionProgress\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_transition_progress);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionControlPoints\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 8 as ::core::ffi::c_int {
        if i != 0 {
            jw_char(w, ',' as i32 as ::core::ffi::c_char);
        }
        jw_number(w, (*s).guidance_hac_transition_points[i as usize]);
        i += 1;
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionHeadingCone\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_transition_heading_cone);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"hacTransitionLeadCurve\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_hac_transition_lead_curve);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionLeadPoints\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 6 as ::core::ffi::c_int {
        if i_0 != 0 {
            jw_char(w, ',' as i32 as ::core::ffi::c_char);
        }
        jw_number(w, (*s).guidance_hac_transition_lead_points[i_0 as usize]);
        i_0 += 1;
    }
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionConeCenter\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '[' as i32 as ::core::ffi::c_char);
    jw_number(
        w,
        (*s).guidance_hac_transition_cone_center[0 as ::core::ffi::c_int as usize],
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_number(
        w,
        (*s).guidance_hac_transition_cone_center[1 as ::core::ffi::c_int as usize],
    );
    jw_char(w, ']' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionConeStartAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_transition_cone_start_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionConeEndAngle\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_transition_cone_end_angle);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"hacTransitionConeArcLength\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_hac_transition_cone_arc_length);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"commandedCourseRateEstimate\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_commanded_course_rate_estimate);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"minimumTurnRadius\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_minimum_turn_radius);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryLegElapsed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).guidance_entry_leg_elapsed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanValid\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_plan_valid);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTerminalReady\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_plan_terminal_ready);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanUT\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*s).guidance_entry_plan_valid, (*s).guidance_entry_plan_ut);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTargetBank\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_target_bank,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTargetAoA\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_target_aoa,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTargetHeading\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_target_heading,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanSegmentRemaining\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_segment_remaining,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanCost\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(w, (*s).guidance_entry_plan_valid, (*s).guidance_entry_plan_cost);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanTAEMRangeError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid,
        (*s).guidance_entry_plan_taem_range_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryPlanTAEMSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid as ::core::ffi::c_int != 0
            && (*s).guidance_entry_plan_taem_speed.is_finite() as i32 != 0,
        (*s).guidance_entry_plan_taem_speed,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanTAEMEnergyError\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_plan_valid as ::core::ffi::c_int != 0
            && (*s).guidance_entry_plan_taem_energy_error.is_finite() as i32 != 0,
        (*s).guidance_entry_plan_taem_energy_error,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryPlanPredictedReversals\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(
        w,
        (*s).guidance_entry_plan_predicted_reversals as ::core::ffi::c_longlong,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalScheduled\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_reversal_scheduled);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalIsFinal\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).guidance_entry_reversal_is_final);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"entryReversalTimeRemaining\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_reversal_scheduled,
        (*s).guidance_entry_reversal_time_remaining,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalRange\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_reversal_scheduled,
        (*s).guidance_entry_reversal_range,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryReversalSign\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).guidance_entry_reversal_scheduled,
        (*s).guidance_entry_reversal_sign,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"entryExecutive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    entry_exec_json(w, &raw const (*s).guidance_entry_exec);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"taemExecutive\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    taem_exec_json(w, &raw const (*s).guidance_taem_exec);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"decisionReason\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).decision_reason[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(w, &raw const (*s).decision_reason as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"command\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    command_json(w, &raw const (*s).command);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"commandLifecycle\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"computedWallSeconds\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*s).command_computed_wall_seconds);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"applyAttempted\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).command_apply_attempted);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"applyStartedWallSeconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).command_apply_attempted,
        (*s).command_apply_started_wall_seconds,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(
        w,
        b"applyFinishedWallSeconds\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    nullable_number(
        w,
        (*s).command_apply_attempted,
        (*s).command_apply_finished_wall_seconds,
    );
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"applied\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).command_applied);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"saturatedOrNormalized\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, command_saturated(s));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"appliedCommand\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).command_apply_attempted {
        applied_command_json(w, &raw const (*s).applied_command);
    } else {
        jw_null(w);
    }
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"plan\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    plan_log_json(w, (*s).plan);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"trajectoryState\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"actualPointCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).actual_trajectory.count as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"referencePointCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*s).reference_trajectory.count as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"planPointCount\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(
        w,
        (if !(*s).plan.is_null() { (*(*s).plan).trajectory.count } else { 0 as size_t })
            as ::core::ffi::c_longlong,
    );
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"statusMessage\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*s).status_message as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"warningMessage\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_warning {
        jw_string(w, &raw const (*s).warning_message as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"lastError\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*s).has_error {
        jw_string(w, &raw const (*s).last_error as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"automationEngaged\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).automation_engaged);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"paused\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*s).paused);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"calibration\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    let mut c: *const CalibrationSnapshot = &raw const (*s).calibration;
    jw_char(w, '{' as i32 as ::core::ffi::c_char);
    jw_string(w, b"state\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, cal_state_string((*c).state));
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"active\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_bool(w, (*c).active);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"progress\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).progress);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"targetAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).target_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"acceptedSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).accepted_samples as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"rejectedSamples\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_integer(w, (*c).rejected_samples as ::core::ffi::c_longlong);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"confidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"planningConfidence\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).planning_confidence);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"bestGlideAngleOfAttack\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).best_glide_angle_of_attack);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"estimatedStallSpeed\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_number(w, (*c).estimated_stall_speed);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"status\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    jw_string(w, &raw const (*c).status as *const ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"warning\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    if (*c).has_warning {
        jw_string(w, &raw const (*c).warning as *const ::core::ffi::c_char);
    } else {
        jw_null(w);
    }
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
    jw_char(w, ',' as i32 as ::core::ffi::c_char);
    jw_string(w, b"adaptiveVehicleProfile\0" as *const u8 as *const ::core::ffi::c_char);
    jw_char(w, ':' as i32 as ::core::ffi::c_char);
    vehicle_json(w, &raw const (*s).adaptive_vehicle_profile);
    jw_char(w, '}' as i32 as ::core::ffi::c_char);
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
