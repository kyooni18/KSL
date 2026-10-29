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
    fn jw_init(w: *mut JsonWriter);
    fn jw_free(w: *mut JsonWriter);
    fn jw_raw(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_char(w: *mut JsonWriter, c: ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_null(w: *mut JsonWriter);
    fn landing_configuration_default() -> LandingConfiguration;
    fn landing_configuration_normalize(c: *mut LandingConfiguration);
    fn landing_configuration_from_json(
        c: *mut LandingConfiguration,
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
    ) -> bool;
    fn landing_configuration_json(w: *mut JsonWriter, c: *const LandingConfiguration);
    fn landing_snapshot_init(s: *mut LandingSnapshot, p: *const VehicleProfile);
    fn trajectory_clear(t: *mut Trajectory);
    fn snapshot_json(w: *mut JsonWriter, s: *const LandingSnapshot);
    fn landing_controller_create(
        cfg: *const LandingConfiguration,
        callback: SnapshotCallback,
        context: *mut ::core::ffi::c_void,
    ) -> *mut LandingController;
    fn landing_controller_destroy(c: *mut LandingController);
    fn landing_controller_update_configuration(
        c: *mut LandingController,
        cfg: *const LandingConfiguration,
    );
    fn landing_controller_configuration(
        c: *mut LandingController,
    ) -> LandingConfiguration;
    fn landing_controller_connect(c: *mut LandingController);
    fn landing_controller_disconnect(c: *mut LandingController);
    fn landing_controller_create_plan(c: *mut LandingController);
    fn landing_controller_engage(c: *mut LandingController);
    fn landing_controller_engage_reentry(c: *mut LandingController);
    fn landing_controller_engage_hac_test(c: *mut LandingController);
    fn landing_controller_engage_final_test(c: *mut LandingController);
    fn landing_controller_start_calibration(c: *mut LandingController);
    fn landing_controller_stop_calibration(c: *mut LandingController, apply: bool);
    fn landing_controller_reset_calibration(c: *mut LandingController);
    fn landing_controller_set_paused(c: *mut LandingController, paused: bool);
    fn landing_controller_abort(c: *mut LandingController);
    fn landing_controller_set_gear(c: *mut LandingController, deployed: bool);
    fn landing_controller_set_brakes(c: *mut LandingController, enabled: bool);
    fn landing_controller_save_checkpoint(
        c: *mut LandingController,
        name: *const ::core::ffi::c_char,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn landing_controller_shutdown(c: *mut LandingController);
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
pub type SnapshotCallback = Option<
    unsafe extern "C" fn(*const LandingSnapshot, *mut ::core::ffi::c_void) -> (),
>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Output {
    pub mutex: pthread_mutex_t,
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const EXIT_FAILURE: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const EXIT_SUCCESS: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn emit_writer(mut out: *mut Output, mut w: *mut JsonWriter) {
    if (*w).failed as ::core::ffi::c_int != 0 || (*w).data.is_null() {
        return;
    }
    pthread_mutex_lock(&raw mut (*out).mutex);
    fwrite((*w).data as *const ::core::ffi::c_void, 1 as size_t, (*w).length, stdout);
    fputc('\n' as i32, stdout);
    fflush(stdout);
    pthread_mutex_unlock(&raw mut (*out).mutex);
}
unsafe extern "C" fn snapshot_callback(
    mut snapshot: *const LandingSnapshot,
    mut context: *mut ::core::ffi::c_void,
) {
    let mut out: *mut Output = context as *mut Output;
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_raw(
        &raw mut w,
        b"{\"type\":\"snapshot\",\"snapshot\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    snapshot_json(&raw mut w, snapshot);
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    emit_writer(out, &raw mut w);
    jw_free(&raw mut w);
}
unsafe extern "C" fn emit_ready(
    mut out: *mut Output,
    mut configuration: *const LandingConfiguration,
) {
    let mut snapshot: LandingSnapshot = LandingSnapshot {
        connection_status: CONN_DISCONNECTED,
        phase: PHASE_IDLE,
        telemetry: Telemetry {
            ut: 0.,
            vessel_name: [0; 128],
            latitude: 0.,
            longitude: 0.,
            mean_altitude: 0.,
            radar_altitude: 0.,
            vertical_speed: 0.,
            horizontal_speed: 0.,
            surface_speed: 0.,
            true_air_speed: 0.,
            atmospheric_density: 0.,
            speed_of_sound: 0.,
            mach: 0.,
            heading: 0.,
            ground_track_heading: 0.,
            course_to_site_error: 0.,
            pitch: 0.,
            roll: 0.,
            pitch_rate: 0.,
            roll_rate: 0.,
            heading_rate: 0.,
            angle_of_attack_rate: 0.,
            has_angle_of_attack_rate: false,
            has_body_pitch_rate: false,
            has_body_roll_rate: false,
            has_body_yaw_rate: false,
            has_course_rate: false,
            body_pitch_rate: 0.,
            body_roll_rate: 0.,
            body_yaw_rate: 0.,
            course_rate: 0.,
            autopilot_error: 0.,
            has_attitude_quaternion: false,
            attitude_quaternion: [0.; 4],
            attitude_reference_frame: [0; 48],
            has_command_pitch_error: false,
            has_command_roll_error: false,
            has_command_heading_error: false,
            command_pitch_error: 0.,
            command_roll_error: 0.,
            command_heading_error: 0.,
            has_pid_gains: false,
            autopilot_pitch_pid_gains: [0.; 3],
            autopilot_roll_pid_gains: [0.; 3],
            autopilot_yaw_pid_gains: [0.; 3],
            navball_speed_mode: SPEED_UNCHANGED,
            attitude_response: AttitudeResponseModel {
                pitch_valid: false,
                roll_valid: false,
                pitch_natural_frequency_s_inv: 0.,
                pitch_damping_ratio: 0.,
                roll_natural_frequency_s_inv: 0.,
                roll_damping_ratio: 0.,
                maximum_pitch_rate_deg_s: 0.,
                maximum_roll_rate_deg_s: 0.,
                maximum_pitch_accel_deg_s2: 0.,
                maximum_roll_accel_deg_s2: 0.,
            },
            angle_of_attack: 0.,
            sideslip: 0.,
            dynamic_pressure: 0.,
            static_pressure: 0.,
            g_force: 0.,
            stall_fraction: 0.,
            lift_force: 0.,
            drag_force: 0.,
            stall_fraction_is_measured: false,
            has_force_vectors: false,
            has_center_of_mass: false,
            has_center_of_mass_root: false,
            physics_sample_valid: false,
            center_of_mass_root: Vector3 { x: 0., y: 0., z: 0. },
            lift_vector: Vector3 { x: 0., y: 0., z: 0. },
            drag_vector: Vector3 { x: 0., y: 0., z: 0. },
            center_of_mass: Vector3 { x: 0., y: 0., z: 0. },
            dry_mass: 0.,
            physics_confidence: 0.,
            physics_authority: [0.; 3],
            physics_authority_confidence: [0.; 3],
            physics_mass_flow: 0.,
            physics_model_residual: 0.,
            physics_model_residual_confidence: 0.,
            physics_certified_uncertainty: 0.,
            physics_force_residual_per_q: Vector3 { x: 0., y: 0., z: 0. },
            physics_force_residual_sigma_per_q: Vector3 { x: 0., y: 0., z: 0. },
            physics_force_residual_confidence: 0.,
            physics_airbrake_model_available: false,
            physics_airbrake_model_confidence: 0.,
            physics_airbrake_drag_accel: 0.,
            physics_samples: 0,
            physics_live_samples: 0,
            predicted_physics_observed_seconds: 0.,
            predicted_physics_fallback_seconds: 0.,
            predicted_shadow_guidance: false,
            predicted_terminal_policy_feasible: false,
            predicted_terminal_survivability_stress_score: 0.,
            predicted_physics_relative_uncertainty: 0.,
            predicted_uncertainty_scenarios: 0,
            predicted_raw_published_position_delta_30s: 0.,
            predicted_raw_published_position_delta_60s: 0.,
            predicted_raw_published_position_delta_120s: 0.,
            mass: 0.,
            available_thrust: 0.,
            current_thrust: 0.,
            apoapsis_altitude: 0.,
            periapsis_altitude: 0.,
            orbit_period: 0.,
            throttle: 0.,
            has_controls: false,
            control_pitch: 0.,
            control_roll: 0.,
            control_yaw: 0.,
            has_torque: false,
            available_pitch_torque: 0.,
            available_roll_torque: 0.,
            available_yaw_torque: 0.,
            has_inertia: false,
            pitch_moment_of_inertia: 0.,
            roll_moment_of_inertia: 0.,
            yaw_moment_of_inertia: 0.,
            has_loop_wall_delta: false,
            has_telemetry_latency: false,
            has_guidance_compute: false,
            has_apply_latency: false,
            has_control_loop: false,
            has_warp: false,
            warp_rails: false,
            physics_warp_factor: 0,
            warp_rate: 0.,
            loop_wall_delta_ms: 0.,
            telemetry_latency_ms: 0.,
            guidance_compute_ms: 0.,
            apply_latency_ms: 0.,
            control_loop_ms: 0.,
            has_rpc_budget: false,
            rpc_read_calls: 0,
            rpc_read_wire_requests: 0,
            rpc_apply_calls: 0,
            rpc_apply_wire_requests: 0,
            rpc_total_calls: 0,
            rpc_total_wire_requests: 0,
            gear: false,
            brakes: false,
            has_airbrakes: false,
            airbrakes: false,
            has_sas_state: false,
            sas_enabled: false,
            has_control_surface_state: false,
            control_surface_state_ut: 0.,
            control_surface_count: 0,
            control_surfaces: [TelemetryControlSurface {
                part_name: [0; 64],
                role: [0; 16],
                deployed: false,
                deflection_override: false,
                override_deflection: 0.,
            }; 16],
            has_main_gear_grounded: false,
            main_gear_grounded: false,
            has_nose_gear_grounded: false,
            nose_gear_grounded: false,
            vessel_situation: [0; 64],
            range_to_site: 0.,
            bearing_to_site: 0.,
            heading_error: 0.,
            runway_along_track: 0.,
            runway_cross_track: 0.,
            flight_path_angle: 0.,
            estimated_lift_to_drag: 0.,
            estimated_ballistic_coefficient: 0.,
            aerodynamic_confidence: 0.,
            calibrated_best_glide_angle_of_attack: 0.,
            calibrated_stall_speed: 0.,
            predicted_miss_distance: 0.,
            predicted_taem_distance: 0.,
            predicted_taem_range_error: 0.,
            predicted_entry_range: 0.,
            predicted_entry_flight_path_angle: 0.,
            predicted_taem_speed: 0.,
            predicted_taem_energy_error: 0.,
            predicted_peak_dynamic_pressure: 0.,
            predicted_peak_g_load: 0.,
            predicted_s_turn_reversals: 0.,
            trajectory_density_scale: 0.,
            trajectory_drag_scale: 0.,
            trajectory_lift_scale: 0.,
            bank_effectiveness: 0.,
            trajectory_calibration_confidence: 0.,
            trajectory_altitude_residual: 0.,
            trajectory_speed_residual: 0.,
            trajectory_range_residual: 0.,
            energy_excess_range: 0.,
        },
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
        plan: ::core::ptr::null_mut::<DeorbitPlan>(),
        actual_trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        reference_trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        predicted_trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        status_message: [0; 384],
        warning_message: [0; 384],
        last_error: [0; 512],
        has_warning: false,
        has_error: false,
        automation_engaged: false,
        paused: false,
        calibration: CalibrationSnapshot {
            state: CAL_IDLE,
            active: false,
            progress: 0.,
            target_angle_of_attack: 0.,
            accepted_samples: 0,
            rejected_samples: 0,
            confidence: 0.,
            planning_confidence: 0.,
            best_glide_angle_of_attack: 0.,
            estimated_stall_speed: 0.,
            status: [0; 256],
            warning: [0; 256],
            has_warning: false,
        },
        adaptive_vehicle_profile: VehicleProfile {
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
        tick_sequence: 0,
        log_sequence: 0,
        wall_monotonic_seconds: 0.,
        simulation_dt: 0.,
        has_vehicle_state: false,
        vehicle_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        has_attitude_quaternion: false,
        attitude_quaternion: [0.; 4],
        attitude_reference_frame: [0; 48],
        command_apply_attempted: false,
        command_applied: false,
        command_computed_wall_seconds: 0.,
        command_apply_started_wall_seconds: 0.,
        command_apply_finished_wall_seconds: 0.,
        applied_command: KRPCApplyResult {
            applied: false,
            autopilot_engaged: false,
            gear: false,
            brakes: false,
            airbrakes: false,
            target_pitch: 0.,
            target_heading: 0.,
            target_roll: 0.,
            throttle: 0.,
            wheel_steering: 0.,
            has_actuator_feedback: false,
            control_pitch: 0.,
            control_roll: 0.,
            control_yaw: 0.,
            has_control_diagnostics: false,
            control_body_pitch_rate_available: false,
            control_body_roll_rate_available: false,
            control_body_yaw_rate_available: false,
            control_target_aoa: 0.,
            control_measured_aoa: 0.,
            control_aoa_rate: 0.,
            control_roll_rate: 0.,
            control_body_pitch_rate: 0.,
            control_body_roll_rate: 0.,
            control_body_yaw_rate: 0.,
            control_pitch_error: 0.,
            control_pitch_trim: 0.,
            control_pitch_authority: 0.,
            control_pitch_aero_fraction: 0.,
            control_roll_authority: 0.,
            control_roll_raw_authority: 0.,
            control_roll_aero_fraction: 0.,
            control_target_roll_rate: 0.,
            control_beta_yaw_gain: 0.,
            control_beta_roll_coupling: 0.,
            control_beta_confidence: 0.,
            control_yaw_command: 0.,
            control_revision: [0; 48],
            reference_frame: [0; 24],
            speed_mode: [0; 24],
            control_profile: [0; 24],
        },
        guidance_has_burn_started: false,
        guidance_burn_completed: false,
        guidance_atmosphere_crossed: false,
        guidance_final_captured: false,
        guidance_airbrakes_deployed: false,
        guidance_hac_side_selected: false,
        has_active_runway: false,
        active_runway: LandingSite {
            name: [0; 128],
            latitude: 0.,
            longitude: 0.,
            altitude: 0.,
            runway_heading: 0.,
            runway_length: 0.,
            runway_width: 0.,
            allow_reciprocal_runway: false,
        },
        active_final_distance: 0.,
        active_final_slope: 0.,
        guidance_delivered_delta_v: 0.,
        guidance_burn_active_elapsed: 0.,
        guidance_s_turn_sign: 0.,
        guidance_hac_side: 0.,
        guidance_entry_leg_elapsed: 0.,
        guidance_entry_plan_valid: false,
        guidance_entry_plan_terminal_ready: false,
        guidance_entry_reversal_scheduled: false,
        guidance_entry_reversal_is_final: false,
        guidance_entry_plan_ut: 0.,
        guidance_entry_plan_target_bank: 0.,
        guidance_entry_plan_target_aoa: 0.,
        guidance_entry_plan_target_heading: 0.,
        guidance_entry_plan_segment_remaining: 0.,
        guidance_entry_plan_cost: 0.,
        guidance_entry_plan_taem_range_error: 0.,
        guidance_entry_plan_taem_speed: 0.,
        guidance_entry_plan_taem_energy_error: 0.,
        guidance_entry_reversal_time_remaining: 0.,
        guidance_entry_reversal_range: 0.,
        guidance_entry_reversal_sign: 0.,
        guidance_entry_plan_predicted_reversals: 0,
        guidance_entry_feedback: EntryGuidanceFeedback {
            valid: false,
            lateral_valid: false,
            reversal_requested: false,
            ut: 0.,
            confidence: 0.,
            measured_drag_accel: 0.,
            reference_drag_accel: 0.,
            drag_error_accel: 0.,
            predicted_achievable_range: 0.,
            required_range_to_go: 0.,
            energy_margin: 0.,
            range_error_scale: 0.,
            required_vertical_lift_accel: 0.,
            crossrange_error: 0.,
            crossrange_rate: 0.,
            projected_crossrange_error: 0.,
            crossrange_corridor: 0.,
            energy_bank_magnitude_deg: 0.,
            commanded_bank_deg: 0.,
            bank_sign: 0.,
        },
        guidance_entry_exec: EntryExecTelemetry {
            initialized: false,
            entry_complete: false,
            active_phase: 0 as EntryPhase,
            last_transition_reason: ENTRY_TRANSITION_NONE,
            last_transition_from: 0 as EntryPhase,
            last_transition_to: 0 as EntryPhase,
            last_transition_ut: 0.,
            last_transition_velocity: 0.,
            last_transition_completed_entry: false,
            transition_count: 0,
        },
        guidance_taem_exec: TaemExecTelemetry {
            initialized: false,
            ownership_latched: false,
            taem_complete: false,
            active_phase: TAEM_PHASE_PATH_ACQUISITION,
            ownership_ut: 0.,
            ownership_velocity: 0.,
            last_transition_reason: TAEM_TRANSITION_NONE,
            last_transition_from: TAEM_PHASE_PATH_ACQUISITION,
            last_transition_to: TAEM_PHASE_PATH_ACQUISITION,
            last_transition_ut: 0.,
            last_transition_velocity: 0.,
            last_transition_completed_taem: false,
            transition_count: 0,
            recovery_active: false,
            recovery_reason: TAEM_RECOVERY_NONE,
            last_recovery_reason: TAEM_RECOVERY_NONE,
            last_recovery_ut: 0.,
            recovery_count: 0,
            terminal_contract: TaemTerminalContract {
                valid: false,
                path_committed: false,
                range_to_go: 0.,
                range_margin: 0.,
                dynamic_pressure: 0.,
                dynamic_pressure_margin: 0.,
                speedbrake_required: false,
                speedbrake_available: false,
                speedbrake_dynamic_pressure_margin: 0.,
                altitude: 0.,
                altitude_margin: 0.,
                flight_path_angle: 0.,
                flight_path_angle_margin: 0.,
                specific_energy: 0.,
                specific_energy_margin: 0.,
                response_time_available: 0.,
                response_time_required: 0.,
                attitude_response_qualified: false,
            },
            terminal_evaluation: TaemTerminalEvaluation {
                valid: false,
                feasible: false,
                block_reason: TAEM_TERMINAL_BLOCK_NONE,
            },
        },
        guidance_taem_energy_law_active: false,
        guidance_taem_energy_drag_level: 0.,
        guidance_taem_energy_path_to_go: 0.,
        guidance_taem_energy_required_path: 0.,
        guidance_hac_captured: false,
        guidance_hac_completed: false,
        guidance_hac_progress_valid: false,
        guidance_hac_transition_active: false,
        guidance_terminal_prediction_valid: false,
        guidance_terminal_candidate_valid: false,
        guidance_terminal_committed: false,
        guidance_terminal_candidate_radius: 0.,
        guidance_terminal_candidate_altitude: 0.,
        guidance_terminal_candidate_speed: 0.,
        guidance_terminal_candidate_kind: 0,
        guidance_terminal_candidate_geometry_degraded: false,
        guidance_terminal_candidate_energy_degraded: false,
        guidance_terminal_candidate_shell_degraded: false,
        guidance_terminal_candidate_side: 0.,
        guidance_terminal_candidate_slope: 0.,
        guidance_terminal_candidate_curve_length: 0.,
        guidance_terminal_candidate_lead_length: 0.,
        guidance_terminal_candidate_arc_remaining: 0.,
        guidance_terminal_candidate_final_distance: 0.,
        guidance_terminal_candidate_quality: 0.,
        guidance_terminal_candidate_arrival_ut: 0.,
        guidance_terminal_candidate_course: 0.,
        guidance_terminal_candidate_peak_lateral: 0.,
        guidance_terminal_candidate_peak_rate_ratio: 0.,
        guidance_terminal_candidate_exit_speed: 0.,
        guidance_terminal_candidate_tracking_error: 0.,
        guidance_terminal_candidate_tracking_course_error: 0.,
        guidance_terminal_candidate_tracking_endpoint_error: 0.,
        guidance_terminal_candidate_tracking_margin: 0.,
        guidance_terminal_candidate_tracking_evaluated: false,
        guidance_terminal_candidate_path_degraded: false,
        guidance_terminal_candidate_control_degraded: false,
        guidance_terminal_candidate_rate_degraded: false,
        guidance_terminal_candidate_end_degraded: false,
        guidance_terminal_reference_fpa: 0.,
        guidance_terminal_mix: 0.,
        guidance_hac_remaining: 0.,
        guidance_hac_radius: 0.,
        guidance_minimum_turn_radius: 0.,
        guidance_hac_transition_progress: 0.,
        guidance_commanded_course_rate_estimate: 0.,
        guidance_hac_transition_points: [0.; 8],
        guidance_hac_transition_heading_cone: false,
        guidance_hac_transition_lead_curve: false,
        guidance_hac_transition_lead_points: [0.; 6],
        guidance_hac_transition_cone_center: [0.; 2],
        guidance_hac_transition_cone_start_angle: 0.,
        guidance_hac_transition_cone_end_angle: 0.,
        guidance_hac_transition_cone_arc_length: 0.,
        guidance_hac_circuit_count: 0,
        guidance_hac_circuit_slope: 0.,
        guidance_terminal_reentry_after_ut: 0.,
        decision_reason: [0; 384],
    };
    landing_snapshot_init(&raw mut snapshot, &raw const (*configuration).vehicle);
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_raw(
        &raw mut w,
        b"{\"type\":\"ready\",\"protocolVersion\":1,\"configuration\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    landing_configuration_json(&raw mut w, configuration);
    jw_raw(&raw mut w, b",\"snapshot\":\0" as *const u8 as *const ::core::ffi::c_char);
    snapshot_json(&raw mut w, &raw mut snapshot);
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    emit_writer(out, &raw mut w);
    jw_free(&raw mut w);
    trajectory_clear(&raw mut snapshot.actual_trajectory);
    trajectory_clear(&raw mut snapshot.reference_trajectory);
}
unsafe extern "C" fn emit_response(
    mut out: *mut Output,
    mut id: *const ::core::ffi::c_char,
    mut configuration_result: bool,
    mut configuration: *const LandingConfiguration,
) {
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_raw(
        &raw mut w,
        b"{\"type\":\"response\",\"id\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(
        &raw mut w,
        if !id.is_null() { id } else { b"\0" as *const u8 as *const ::core::ffi::c_char },
    );
    jw_raw(
        &raw mut w,
        b",\"ok\":true,\"result\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if configuration_result {
        jw_raw(
            &raw mut w,
            b"{\"configuration\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        landing_configuration_json(&raw mut w, configuration);
        jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    } else {
        jw_raw(&raw mut w, b"{}\0" as *const u8 as *const ::core::ffi::c_char);
    }
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    emit_writer(out, &raw mut w);
    jw_free(&raw mut w);
}
unsafe extern "C" fn emit_error(
    mut out: *mut Output,
    mut id: *const ::core::ffi::c_char,
    mut message: *const ::core::ffi::c_char,
) {
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_raw(
        &raw mut w,
        b"{\"type\":\"response\",\"id\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !id.is_null() {
        jw_string(&raw mut w, id);
    } else {
        jw_null(&raw mut w);
    }
    jw_raw(
        &raw mut w,
        b",\"ok\":false,\"error\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(
        &raw mut w,
        if !message.is_null() {
            message
        } else {
            b"Unknown backend error\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    emit_writer(out, &raw mut w);
    jw_free(&raw mut w);
}
unsafe fn main_0() -> ::core::ffi::c_int {
    let mut output: Output = Output {
        mutex: core::mem::zeroed(),
    };
    pthread_mutex_init(
        &raw mut output.mutex,
        ::core::ptr::null::<pthread_mutexattr_t>(),
    );
    let mut configuration: LandingConfiguration = landing_configuration_default();
    landing_configuration_normalize(&raw mut configuration);
    let mut controller: *mut LandingController = landing_controller_create(
        &raw mut configuration,
        Some(
            snapshot_callback
                as unsafe extern "C" fn(
                    *const LandingSnapshot,
                    *mut ::core::ffi::c_void,
                ) -> (),
        ),
        &raw mut output as *mut ::core::ffi::c_void,
    );
    if controller.is_null() {
        fprintf(
            stderr,
            b"Failed to allocate C landing controller.\n\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return EXIT_FAILURE;
    }
    emit_ready(&raw mut output, &raw mut configuration);
    let mut line: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    let mut line_capacity: size_t = 0 as size_t;
    while getline(&raw mut line, &raw mut line_capacity, stdin) >= 0 as __ssize_t {
        let mut tokens: [JsonToken; 8192] = [JsonToken {
            type_0: JSON_UNDEFINED,
            start: 0,
            end: 0,
            size: 0,
            parent: 0,
        }; 8192];
        let mut doc: JsonDoc = JsonDoc {
            text: ::core::ptr::null::<::core::ffi::c_char>(),
            tokens: ::core::ptr::null_mut::<JsonToken>(),
            count: 0,
        };
        if json_parse(
            line,
            &raw mut tokens as *mut JsonToken,
            8192 as ::core::ffi::c_int,
            &raw mut doc,
        ) < 1 as ::core::ffi::c_int
            || (*doc.tokens.offset(0 as ::core::ffi::c_int as isize)).type_0
                as ::core::ffi::c_uint
                != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            emit_error(
                &raw mut output,
                ::core::ptr::null::<::core::ffi::c_char>(),
                b"Invalid backend command JSON\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        } else {
            let mut id: [::core::ffi::c_char; 128] = ::core::mem::transmute::<
                [u8; 128],
                [::core::ffi::c_char; 128],
            >(
                *b"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            );
            let mut method: [::core::ffi::c_char; 96] = ::core::mem::transmute::<
                [u8; 96],
                [::core::ffi::c_char; 96],
            >(
                *b"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            );
            let mut id_index: ::core::ffi::c_int = json_object_get(
                &raw mut doc,
                0 as ::core::ffi::c_int,
                b"id\0" as *const u8 as *const ::core::ffi::c_char,
            );
            let mut method_index: ::core::ffi::c_int = json_object_get(
                &raw mut doc,
                0 as ::core::ffi::c_int,
                b"method\0" as *const u8 as *const ::core::ffi::c_char,
            );
            if id_index >= 0 as ::core::ffi::c_int {
                json_string(
                    &raw mut doc,
                    id_index,
                    &raw mut id as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                );
            }
            if method_index < 0 as ::core::ffi::c_int
                || !json_string(
                    &raw mut doc,
                    method_index,
                    &raw mut method as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
                )
            {
                emit_error(
                    &raw mut output,
                    if id[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
                        &raw mut id as *mut ::core::ffi::c_char
                    } else {
                        ::core::ptr::null_mut::<::core::ffi::c_char>()
                    },
                    b"Backend command requires method\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
            } else {
                let mut config_index: ::core::ffi::c_int = json_object_get(
                    &raw mut doc,
                    0 as ::core::ffi::c_int,
                    b"configuration\0" as *const u8 as *const ::core::ffi::c_char,
                );
                if config_index >= 0 as ::core::ffi::c_int {
                    let mut requested: LandingConfiguration = configuration;
                    if !landing_configuration_from_json(
                        &raw mut requested,
                        &raw mut doc,
                        config_index,
                    ) {
                        emit_error(
                            &raw mut output,
                            if id[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
                                != 0
                            {
                                &raw mut id as *mut ::core::ffi::c_char
                            } else {
                                ::core::ptr::null_mut::<::core::ffi::c_char>()
                            },
                            b"Invalid landing configuration\0" as *const u8
                                as *const ::core::ffi::c_char,
                        );
                        continue;
                    } else {
                        configuration = requested;
                        landing_controller_update_configuration(
                            controller,
                            &raw mut configuration,
                        );
                    }
                }
                if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"getConfiguration\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    configuration = landing_controller_configuration(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        true_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"updateConfiguration\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    if config_index < 0 as ::core::ffi::c_int {
                        emit_error(
                            &raw mut output,
                            &raw mut id as *mut ::core::ffi::c_char,
                            b"updateConfiguration requires configuration\0" as *const u8
                                as *const ::core::ffi::c_char,
                        );
                    } else {
                        configuration = landing_controller_configuration(controller);
                        emit_response(
                            &raw mut output,
                            &raw mut id as *mut ::core::ffi::c_char,
                            true_0 != 0,
                            &raw mut configuration,
                        );
                    }
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"connect\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_connect(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        true_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"disconnect\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_disconnect(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"createPlan\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_create_plan(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"engage\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_engage(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"engageReentry\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_engage_reentry(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"engageHACTest\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_engage_hac_test(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"engageFinalTest\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_engage_final_test(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"startCalibration\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_start_calibration(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"stopCalibration\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    let mut apply: bool = json_boolean(
                        &raw mut doc,
                        json_object_get(
                            &raw mut doc,
                            0 as ::core::ffi::c_int,
                            b"applyResults\0" as *const u8 as *const ::core::ffi::c_char,
                        ),
                        false_0 != 0,
                    );
                    landing_controller_stop_calibration(controller, apply);
                    if apply {
                        configuration = landing_controller_configuration(controller);
                    }
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"resetCalibration\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_reset_calibration(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"setPaused\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    let mut value: bool = json_boolean(
                        &raw mut doc,
                        json_object_get(
                            &raw mut doc,
                            0 as ::core::ffi::c_int,
                            b"value\0" as *const u8 as *const ::core::ffi::c_char,
                        ),
                        false_0 != 0,
                    );
                    landing_controller_set_paused(controller, value);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"abort\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_abort(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"setGear\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    let mut value_0: bool = json_boolean(
                        &raw mut doc,
                        json_object_get(
                            &raw mut doc,
                            0 as ::core::ffi::c_int,
                            b"value\0" as *const u8 as *const ::core::ffi::c_char,
                        ),
                        false_0 != 0,
                    );
                    landing_controller_set_gear(controller, value_0);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"setBrakes\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    let mut value_1: bool = json_boolean(
                        &raw mut doc,
                        json_object_get(
                            &raw mut doc,
                            0 as ::core::ffi::c_int,
                            b"value\0" as *const u8 as *const ::core::ffi::c_char,
                        ),
                        false_0 != 0,
                    );
                    landing_controller_set_brakes(controller, value_1);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"saveCheckpoint\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    let mut name: [::core::ffi::c_char; 128] = [
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
                    ];
                    json_string(
                        &raw mut doc,
                        json_object_get(
                            &raw mut doc,
                            0 as ::core::ffi::c_int,
                            b"name\0" as *const u8 as *const ::core::ffi::c_char,
                        ),
                        &raw mut name as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                    );
                    let mut error: [::core::ffi::c_char; 512] = [
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
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
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
                    if !landing_controller_save_checkpoint(
                        controller,
                        &raw mut name as *mut ::core::ffi::c_char,
                        &raw mut error as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
                    ) {
                        emit_error(
                            &raw mut output,
                            if id[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
                                != 0
                            {
                                &raw mut id as *mut ::core::ffi::c_char
                            } else {
                                ::core::ptr::null_mut::<::core::ffi::c_char>()
                            },
                            if error[0 as ::core::ffi::c_int as usize]
                                as ::core::ffi::c_int != 0
                            {
                                &raw mut error as *mut ::core::ffi::c_char
                                    as *const ::core::ffi::c_char
                            } else {
                                b"Could not save KSP checkpoint\0" as *const u8
                                    as *const ::core::ffi::c_char
                            },
                        );
                    } else {
                        emit_response(
                            &raw mut output,
                            &raw mut id as *mut ::core::ffi::c_char,
                            false_0 != 0,
                            &raw mut configuration,
                        );
                    }
                } else if strcmp(
                    &raw mut method as *mut ::core::ffi::c_char,
                    b"shutdown\0" as *const u8 as *const ::core::ffi::c_char,
                ) == 0
                {
                    landing_controller_shutdown(controller);
                    emit_response(
                        &raw mut output,
                        &raw mut id as *mut ::core::ffi::c_char,
                        false_0 != 0,
                        &raw mut configuration,
                    );
                    break;
                } else {
                    let mut message: [::core::ffi::c_char; 192] = [0; 192];
                    snprintf(
                        &raw mut message as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
                        b"Unknown backend method: %s\0" as *const u8
                            as *const ::core::ffi::c_char,
                        &raw mut method as *mut ::core::ffi::c_char,
                    );
                    emit_error(
                        &raw mut output,
                        if id[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
                            != 0
                        {
                            &raw mut id as *mut ::core::ffi::c_char
                        } else {
                            ::core::ptr::null_mut::<::core::ffi::c_char>()
                        },
                        &raw mut message as *mut ::core::ffi::c_char,
                    );
                }
            }
        }
    }
    free(line as *mut ::core::ffi::c_void);
    landing_controller_destroy(controller);
    pthread_mutex_destroy(&raw mut output.mutex);
    return EXIT_SUCCESS;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub fn main() {
    unsafe { ::std::process::exit(main_0() as i32) }
}
#[repr(C)]
pub struct LandingController {
    _private: [u8; 0],
}
