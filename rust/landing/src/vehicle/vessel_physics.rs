use crate::sys::*;
extern "C" {
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn ceil(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn copysign(
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
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_low_q_protective_aoa_floor(
        dynamic_pressure: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vector3;
    fn vadd(a: Vector3, b: Vector3) -> Vector3;
    fn vsub(a: Vector3, b: Vector3) -> Vector3;
    fn vscale(a: Vector3, s: ::core::ffi::c_double) -> Vector3;
    fn vdot(a: Vector3, b: Vector3) -> ::core::ffi::c_double;
    fn vcross(a: Vector3, b: Vector3) -> Vector3;
    fn vmag(a: Vector3) -> ::core::ffi::c_double;
    fn vnorm(a: Vector3, fallback: Vector3) -> Vector3;
    fn vproject_plane(a: Vector3, normal: Vector3) -> Vector3;
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
    fn planet_atmospheric_density(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_speed_of_sound(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn aerodynamic_force_factors_mach(
        mach: ::core::ffi::c_double,
        angle_of_attack: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
        lift_factor: *mut ::core::ffi::c_double,
        drag_factor: *mut ::core::ffi::c_double,
    );
}
pub type size_t = usize;
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
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
unsafe extern "C" fn finite_vector(mut v: Vector3) -> bool {
    return v.x.is_finite() as i32 != 0 && v.y.is_finite() as i32 != 0
        && v.z.is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_init(mut m: *mut VesselPhysicsModel) {
    memset(
        m as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<VesselPhysicsModel>() as size_t,
    );
    (*m).airbrakes = -(1 as ::core::ffi::c_int);
}
unsafe extern "C" fn vessel_physics_reset_transients(mut m: *mut VesselPhysicsModel) {
    if m.is_null() {
        return;
    }
    let mut samples: [VesselAeroSample; 192] = [VesselAeroSample {
        q: 0.,
        mach: 0.,
        aoa: 0.,
        beta: 0.,
        mass: 0.,
        force_per_q: Vector3 { x: 0., y: 0., z: 0. },
        gear: false,
        brakes: false,
        airbrakes: 0,
        observations: 0,
        trust: 0.,
        last_observation_ut: 0.,
    }; 192];
    let mut count: ::core::ffi::c_uint = (*m).count;
    let mut next: ::core::ffi::c_uint = (*m).next;
    let mut accepted: ::core::ffi::c_uint = (*m).accepted;
    let mut rejected: ::core::ffi::c_uint = (*m).rejected;
    let mut certified_uncertainty: ::core::ffi::c_double = (*m).certified_uncertainty;
    let mut certified_confidence: ::core::ffi::c_double = (*m).certified_confidence;
    memcpy(
        &raw mut samples as *mut VesselAeroSample as *mut ::core::ffi::c_void,
        &raw mut (*m).samples as *mut VesselAeroSample as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[VesselAeroSample; 192]>() as size_t,
    );
    vessel_physics_init(m);
    memcpy(
        &raw mut (*m).samples as *mut VesselAeroSample as *mut ::core::ffi::c_void,
        &raw mut samples as *mut VesselAeroSample as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[VesselAeroSample; 192]>() as size_t,
    );
    (*m).count = count;
    (*m).next = next;
    (*m).accepted = accepted;
    (*m).rejected = rejected;
    (*m).certified_uncertainty = certified_uncertainty;
    (*m).certified_confidence = certified_confidence;
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_import_sample(
    mut m: *mut VesselPhysicsModel,
    mut input: *const VesselAeroSample,
) {
    if m.is_null() || input.is_null() || (*input).q.is_finite() as i32 == 0
        || (*input).q <= 0.0f64 || (*input).mach.is_finite() as i32 == 0
        || (*input).aoa.is_finite() as i32 == 0 || (*input).beta.is_finite() as i32 == 0
        || (*input).mass.is_finite() as i32 == 0
        || (*input).mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).airbrakes != 0 as ::core::ffi::c_int
            && (*input).airbrakes != 1 as ::core::ffi::c_int
        || !finite_vector((*input).force_per_q)
    {
        return;
    }
    let mut sample: VesselAeroSample = *input;
    sample.trust = clampd(
        if sample.trust.is_finite() as i32 != 0 {
            sample.trust
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        },
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    if sample.trust <= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        return;
    }
    if sample.observations == 0 as ::core::ffi::c_uint {
        sample.observations = 1 as ::core::ffi::c_uint;
    }
    if (*m).count == 0 as ::core::ffi::c_uint && (*m).airbrakes < 0 as ::core::ffi::c_int
    {
        (*m).gear = sample.gear;
        (*m).brakes = sample.brakes;
        (*m).airbrakes = sample.airbrakes;
    }
    if (*m).count < VESSEL_AERO_SAMPLES as ::core::ffi::c_uint {
        let fresh0 = (*m).count;
        (*m).count = (*m).count.wrapping_add(1);
        (*m).samples[fresh0 as usize] = sample;
        (*m).next = (*m).count.wrapping_rem(VESSEL_AERO_SAMPLES as ::core::ffi::c_uint);
    }
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_gravity(
    mut pos: Vector3,
    mut p: *const PlanetModel,
) -> Vector3 {
    let mut r: ::core::ffi::c_double = fmax(
        vmag(pos),
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    return vscale(pos, -(*p).gravitational_parameter / (r * r * r));
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_environment(
    mut p: *const PlanetModel,
    mut cal: *const TrajectoryCalibrationModel,
    mut altitude: ::core::ffi::c_double,
    mut density: *mut ::core::ffi::c_double,
    mut sound: *mut ::core::ffi::c_double,
) {
    *density = planet_atmospheric_density(p, altitude)
        * clampd((*cal).density_scale, 0.35f64, 2.5f64);
    let mut scale: ::core::ffi::c_double = (*cal).speed_of_sound_scale;
    if scale.is_finite() as i32 == 0
        || scale <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        scale = 1 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut profile: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        altitude,
    );
    *sound = if profile > 1 as ::core::ffi::c_int as ::core::ffi::c_double {
        profile * clampd(scale, 0.7f64, 1.3f64)
    } else {
        clampd(
            (*cal).speed_of_sound,
            180 as ::core::ffi::c_int as ::core::ffi::c_double,
            450 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    };
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_conservative_stall_fraction(
    mut true_air_speed: ::core::ffi::c_double,
    mut angle_of_attack: ::core::ffi::c_double,
    mut dynamic_pressure: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if v.is_null() || true_air_speed.is_finite() as i32 == 0
        || angle_of_attack.is_finite() as i32 == 0
        || dynamic_pressure.is_finite() as i32 == 0
        || (*v).minimum_safe_speed.is_finite() as i32 == 0
        || (*v).minimum_safe_speed <= 1.0f64
        || (*v).maximum_angle_of_attack.is_finite() as i32 == 0
        || (*v).maximum_angle_of_attack <= 0.0f64
    {
        return 1.0f64;
    }
    let mut speed_margin: ::core::ffi::c_double = clampd(
        ((*v).minimum_safe_speed * 1.15f64 - true_air_speed)
            / fmax(10.0f64, (*v).minimum_safe_speed * 0.35f64),
        0.0f64,
        1.0f64,
    );
    let mut protective: ::core::ffi::c_double = entry_low_q_protective_aoa_floor(
        dynamic_pressure,
        v,
    );
    let mut thin_air: ::core::ffi::c_double = clampd(
        (700.0f64 - fmax(0.0f64, dynamic_pressure)) / 350.0f64,
        0.0f64,
        1.0f64,
    );
    let mut tracking_margin: ::core::ffi::c_double = thin_air * 0.35f64
        * fmax(0.0f64, (*v).maximum_angle_of_attack - protective);
    let mut onset: ::core::ffi::c_double = fmax(
        (*v).maximum_angle_of_attack * 0.75f64,
        protective + tracking_margin,
    );
    let mut aoa_margin: ::core::ffi::c_double = clampd(
        (fabs(angle_of_attack) - onset)
            / fmax(1e-6f64, (*v).maximum_angle_of_attack - onset),
        0.0f64,
        1.0f64,
    );
    return fmax(speed_margin, aoa_margin);
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_burn_mass(
    mut m: *const VesselPhysicsModel,
    mut mass: ::core::ffi::c_double,
    mut thrust: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if m.is_null()
        || (*m).propellant_per_impulse
            <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*m).dry_mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || dt <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || mass <= (*m).dry_mass
    {
        return mass;
    }
    return fmax(
        (*m).dry_mass,
        mass
            - (*m).propellant_per_impulse
                * fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, thrust) * dt,
    );
}
#[no_mangle]
pub unsafe extern "C" fn landing_snapshot_set_sample(
    mut s: *mut LandingSnapshot,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
) {
    (*s).telemetry = *t;
    (*s).has_vehicle_state = !state.is_null() && (*t).ut.is_finite() as i32 != 0
        && fabs((*state).ut - (*t).ut) < 1e-6f64;
    if (*s).has_vehicle_state {
        (*s).vehicle_state = *state;
    }
    (*s).has_attitude_quaternion = (*t).has_attitude_quaternion;
    memcpy(
        &raw mut (*s).attitude_quaternion as *mut ::core::ffi::c_double
            as *mut ::core::ffi::c_void,
        &raw const (*t).attitude_quaternion as *const ::core::ffi::c_double
            as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 4]>() as size_t,
    );
    snprintf(
        &raw mut (*s).attitude_reference_frame as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        &raw const (*t).attitude_reference_frame as *const ::core::ffi::c_char,
    );
}
unsafe extern "C" fn physics_diagnostics(
    mut m: *mut VesselPhysicsModel,
    mut t: *mut Telemetry,
) {
    let mut specific: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut confidence: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    vessel_physics_aero(
        m,
        (*t).dynamic_pressure,
        (*t).mach,
        (*t).angle_of_attack,
        (*t).sideslip,
        (*t).mass,
        &raw mut specific,
        &raw mut confidence,
    );
    (*m).confidence = confidence;
    (*t).physics_confidence = confidence;
    (*t).physics_samples = (*m).count;
    (*t).physics_mass_flow = (*m).mass_flow;
    (*t).physics_live_samples = (*m).live_observations;
    (*t).physics_model_residual = (*m).model_residual;
    (*t).physics_model_residual_confidence = (*m).model_residual_confidence;
    (*t).physics_certified_uncertainty = (*m).certified_uncertainty;
    (*t).physics_force_residual_per_q = (*m).force_residual_per_q;
    (*t).physics_force_residual_sigma_per_q = v3(
        sqrt(
            fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                (*m).force_residual_variance_per_q.x,
            ),
        ),
        sqrt(
            fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                (*m).force_residual_variance_per_q.y,
            ),
        ),
        sqrt(
            fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                (*m).force_residual_variance_per_q.z,
            ),
        ),
    );
    (*t).physics_force_residual_confidence = (*m).force_residual_confidence;
    let mut airbrake_specific: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut airbrake_confidence: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*t).physics_airbrake_model_available = (*t).has_airbrakes as ::core::ffi::c_int != 0
        && vessel_physics_aero_config(
            m,
            (*t).dynamic_pressure,
            (*t).mach,
            (*t).angle_of_attack,
            (*t).sideslip,
            (*t).mass,
            (*t).gear,
            (*t).brakes,
            1 as ::core::ffi::c_int,
            &raw mut airbrake_specific,
            &raw mut airbrake_confidence,
        ) as ::core::ffi::c_int != 0;
    (*t).physics_airbrake_model_confidence = if (*t).physics_airbrake_model_available
        as ::core::ffi::c_int != 0
    {
        airbrake_confidence
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    (*t).physics_airbrake_drag_accel = if (*t).physics_airbrake_model_available
        as ::core::ffi::c_int != 0 && airbrake_specific.x.is_finite() as i32 != 0
    {
        fmax(0.0f64, airbrake_specific.x)
    } else {
        0.0f64
    };
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        (*t).physics_authority[i as usize] = (*m).authority[i as usize];
        (*t).physics_authority_confidence[i as usize] = (*m)
            .authority_confidence[i as usize];
        i += 1;
    }
}
unsafe extern "C" fn physics_control_axis_isolated(
    mut control: *const ::core::ffi::c_double,
    mut axis: ::core::ffi::c_int,
) -> bool {
    if control.is_null() || axis < 0 as ::core::ffi::c_int
        || axis >= 3 as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    let mut selected: ::core::ffi::c_double = fabs(*control.offset(axis as isize));
    if selected.is_finite() as i32 == 0 || selected <= 0.15f64 {
        return false_0 != 0;
    }
    let mut cross_limit: ::core::ffi::c_double = selected * 0.05f64;
    let mut other: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while other < 3 as ::core::ffi::c_int {
        if other != axis {
            if (*control.offset(other as isize)).is_finite() as i32 == 0
                || fabs(*control.offset(other as isize)) > cross_limit
            {
                return false_0 != 0;
            }
        }
        other += 1;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_observe(
    mut m: *mut VesselPhysicsModel,
    mut t: *mut Telemetry,
    mut s: *const VehicleState,
    mut p: *const PlanetModel,
) {
    if (*t).ut.is_finite() as i32 == 0 || (*s).ut.is_finite() as i32 == 0
        || fabs((*s).ut - (*t).ut) > 1e-6f64 || (*s).mass.is_finite() as i32 == 0
        || (*s).mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || !finite_vector((*s).position) || !finite_vector((*s).velocity)
    {
        return;
    }
    if !(*t).physics_sample_valid {
        physics_diagnostics(m, t);
        return;
    }
    if (*m).has_state as ::core::ffi::c_int != 0 && (*s).ut < (*m).state.ut {
        vessel_physics_reset_transients(m);
    }
    let mut dt: ::core::ffi::c_double = if (*m).has_state as ::core::ffi::c_int != 0 {
        (*s).ut - (*m).state.ut
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    if (*m).has_state as ::core::ffi::c_int != 0
        && dt == 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        physics_diagnostics(m, t);
        return;
    }
    if (*m).has_state as ::core::ffi::c_int != 0
        && dt > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && fabs((*s).mass - (*m).state.mass) > 0.02f64 * (*m).state.mass
    {
        vessel_physics_reset_transients(m);
        dt = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    if (*m).has_state as ::core::ffi::c_int != 0 && dt > 0.02f64
        && dt < 2 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*t).current_thrust > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*m).thrust > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        let mut flow: ::core::ffi::c_double = ((*m).state.mass - (*s).mass) / dt;
        if flow >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
            && flow < (*s).mass * 0.01f64
        {
            (*m).mass_flow += 0.2f64 * (flow - (*m).mass_flow);
            let mut consumption: ::core::ffi::c_double = flow
                / fmax(
                    0.5f64 * ((*m).thrust + (*t).current_thrust),
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                );
            (*m).propellant_per_impulse
                += 0.2f64 * (consumption - (*m).propellant_per_impulse);
        }
    }
    let mut torque: [::core::ffi::c_double; 3] = [
        (*t).available_pitch_torque,
        (*t).available_roll_torque,
        (*t).available_yaw_torque,
    ];
    let mut inertia: [::core::ffi::c_double; 3] = [
        (*t).pitch_moment_of_inertia,
        (*t).roll_moment_of_inertia,
        (*t).yaw_moment_of_inertia,
    ];
    let mut rate: [::core::ffi::c_double; 3] = [
        (*t).body_pitch_rate,
        (*t).body_roll_rate,
        (*t).body_yaw_rate,
    ];
    let mut control: [::core::ffi::c_double; 3] = [
        (*t).control_pitch,
        (*t).control_roll,
        (*t).control_yaw,
    ];
    let mut valid: [bool; 3] = [
        (*t).has_body_pitch_rate,
        (*t).has_body_roll_rate,
        (*t).has_body_yaw_rate,
    ];
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        (*m).torque[i as usize] = if (*t).has_torque as ::core::ffi::c_int != 0
            && torque[i as usize].is_finite() as i32 != 0
        {
            torque[i as usize]
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        (*m).inertia[i as usize] = if (*t).has_inertia as ::core::ffi::c_int != 0
            && inertia[i as usize].is_finite() as i32 != 0
        {
            inertia[i as usize]
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        if (*t).has_torque as ::core::ffi::c_int != 0
            && (*t).has_inertia as ::core::ffi::c_int != 0
            && torque[i as usize].is_finite() as i32 != 0
            && fabs(torque[i as usize]) > 1e-6f64
            && inertia[i as usize] > 0 as ::core::ffi::c_int as ::core::ffi::c_double
            && inertia[i as usize].is_finite() as i32 != 0
            && (*m).authority_confidence[i as usize] < 0.3f64
        {
            (*m).authority[i as usize] = clampd(
                fabs(torque[i as usize]) / inertia[i as usize] * RAD2DEG,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                10000 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            (*m).authority_confidence[i as usize] = 0.25f64;
        }
        if valid[i as usize] as ::core::ffi::c_int != 0
            && (*m).previous_rate_valid[i as usize] as ::core::ffi::c_int != 0
            && (*t).has_controls as ::core::ffi::c_int != 0 && dt > 0.02f64
            && dt < 1 as ::core::ffi::c_int as ::core::ffi::c_double
            && physics_control_axis_isolated(
                &raw mut (*m).previous_control as *mut ::core::ffi::c_double
                    as *const ::core::ffi::c_double,
                i,
            ) as ::core::ffi::c_int != 0
        {
            let mut measured: ::core::ffi::c_double = (rate[i as usize]
                - (*m).previous_rate[i as usize]) / dt
                / (*m).previous_control[i as usize];
            if measured.is_finite() as i32 != 0
                && measured > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                && measured < 10000 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                (*m).authority[i as usize]
                    += 0.15f64 * (measured - (*m).authority[i as usize]);
                (*m).authority_confidence[i as usize] = fmin(
                    0.85f64,
                    (*m).authority_confidence[i as usize] + 0.025f64,
                );
            }
        }
        (*m).previous_rate[i as usize] = rate[i as usize];
        (*m).previous_control[i as usize] = control[i as usize];
        (*m).previous_rate_valid[i as usize] = valid[i as usize] as ::core::ffi::c_int
            != 0 && rate[i as usize].is_finite() as i32 != 0
            && (*t).has_controls as ::core::ffi::c_int != 0;
        (*t).physics_authority[i as usize] = (*m).authority[i as usize];
        (*t).physics_authority_confidence[i as usize] = (*m)
            .authority_confidence[i as usize];
        i += 1;
    }
    (*m).state = *s;
    (*m).has_state = true_0 != 0;
    (*m).thrust = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        (*t).current_thrust,
    );
    (*m).available_thrust = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        (*t).available_thrust,
    );
    (*m).throttle = (*t).throttle;
    (*m).dry_mass = if (*t).dry_mass.is_finite() as i32 != 0
        && (*t).dry_mass > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*t).dry_mass <= (*s).mass
    {
        (*t).dry_mass
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    (*m).authority_q = (*t).dynamic_pressure;
    (*m).sideslip = (*t).sideslip;
    (*m).gear = (*t).gear;
    (*m).brakes = (*t).brakes;
    (*m).airbrakes = if (*t).has_airbrakes as ::core::ffi::c_int != 0 {
        if (*t).airbrakes as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        }
    } else {
        -(1 as ::core::ffi::c_int)
    };
    (*m).has_center_of_mass = (*t).has_center_of_mass as ::core::ffi::c_int != 0
        && finite_vector((*t).center_of_mass) as ::core::ffi::c_int != 0;
    if (*m).has_center_of_mass {
        (*m).center_of_mass = (*t).center_of_mass;
    }
    (*m).has_center_of_mass_root = (*t).has_center_of_mass_root as ::core::ffi::c_int
        != 0 && finite_vector((*t).center_of_mass_root) as ::core::ffi::c_int != 0;
    if (*m).has_center_of_mass_root {
        (*m).center_of_mass_root = (*t).center_of_mass_root;
    }
    (*m).lift_vector = (*t).lift_vector;
    (*m).drag_vector = (*t).drag_vector;
    let mut air: Vector3 = vsub(
        (*s).velocity,
        vcross(planet_rotation_vector(p), (*s).position),
    );
    if (*t).has_force_vectors as ::core::ffi::c_int != 0
        && (*t).has_airbrakes as ::core::ffi::c_int != 0
        && finite_vector((*t).lift_vector) as ::core::ffi::c_int != 0
        && finite_vector((*t).drag_vector) as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure > 0.0f64
        && (*t).dynamic_pressure.is_finite() as i32 != 0
        && (*t).mach.is_finite() as i32 != 0
        && (*t).angle_of_attack.is_finite() as i32 != 0
        && (*t).sideslip.is_finite() as i32 != 0 && vmag(air) > DBL_MIN
    {
        let mut dir: Vector3 = vnorm(
            air,
            v3(
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut up: Vector3 = vnorm(
            vproject_plane((*s).position, dir),
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut side: Vector3 = vnorm(
            vcross(dir, up),
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut bank: ::core::ffi::c_double = (*t).roll * DEG2RAD;
        let mut lift: Vector3 = vadd(vscale(up, cos(bank)), vscale(side, sin(bank)));
        let mut lateral: Vector3 = vsub(vscale(side, cos(bank)), vscale(up, sin(bank)));
        let mut f: Vector3 = vadd((*t).lift_vector, (*t).drag_vector);
        let mut sample: VesselAeroSample = VesselAeroSample {
            q: (*t).dynamic_pressure,
            mach: (*t).mach,
            aoa: (*t).angle_of_attack,
            beta: (*t).sideslip,
            mass: (*s).mass,
            force_per_q: vscale(
                v3(-vdot(f, dir), vdot(f, lift), vdot(f, lateral)),
                1 as ::core::ffi::c_int as ::core::ffi::c_double / (*t).dynamic_pressure,
            ),
            gear: (*t).gear,
            brakes: (*t).brakes,
            airbrakes: if (*t).airbrakes as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            observations: 1 as ::core::ffi::c_uint,
            trust: 1.0f64,
            last_observation_ut: (*t).ut,
        };
        if sample.force_per_q.x >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
            && finite_vector(sample.force_per_q) as ::core::ffi::c_int != 0
        {
            (*m).accepted = (*m).accepted.wrapping_add(1);
            let mut independent: bool = !(*m).has_last_residual_ut
                || (*t).ut < (*m).last_residual_ut
                || (*t).ut - (*m).last_residual_ut >= 0.5f64;
            if independent {
                (*m).live_observations = (*m).live_observations.wrapping_add(1);
                (*m).last_residual_ut = (*t).ut;
                (*m).has_last_residual_ut = true_0 != 0;
                let mut expected: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
                let mut certified_confidence: ::core::ffi::c_double = 0
                    as ::core::ffi::c_int as ::core::ffi::c_double;
                if vessel_physics_aero_config(
                    m,
                    (*t).dynamic_pressure,
                    (*t).mach,
                    (*t).angle_of_attack,
                    (*t).sideslip,
                    (*s).mass,
                    (*t).gear,
                    (*t).brakes,
                    if (*t).airbrakes as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                    &raw mut expected,
                    &raw mut certified_confidence,
                ) {
                    let mut actual: Vector3 = vscale(
                        sample.force_per_q,
                        (*t).dynamic_pressure
                            / fmax(
                                (*s).mass,
                                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                            ),
                    );
                    let mut scale: ::core::ffi::c_double = fmax(0.05f64, vmag(actual));
                    let mut residual: ::core::ffi::c_double = clampd(
                        vmag(vsub(actual, expected)) / scale,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        2.0f64,
                    );
                    let mut alpha: ::core::ffi::c_double = 0.10f64
                        * clampd(certified_confidence, 0.15f64, 1.0f64);
                    if (*m).model_residual_confidence
                        <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
                    {
                        (*m).model_residual = residual;
                    } else {
                        (*m).model_residual += alpha * (residual - (*m).model_residual);
                    }
                    (*m).model_residual_confidence = clampd(
                        (*m).model_residual_confidence + 0.035f64 * certified_confidence,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        1 as ::core::ffi::c_int as ::core::ffi::c_double,
                    );
                    let mut expected_per_q: Vector3 = vscale(
                        expected,
                        (*s).mass / fmax((*t).dynamic_pressure, 1e-9f64),
                    );
                    let mut raw: Vector3 = vsub(sample.force_per_q, expected_per_q);
                    let mut basis: ::core::ffi::c_double = fmax(
                        vmag(expected_per_q),
                        1e-6f64,
                    );
                    let mut limits: Vector3 = v3(
                        fmax(0.35f64 * fabs(expected_per_q.x), 0.05f64 * basis),
                        fmax(0.35f64 * fabs(expected_per_q.y), 0.08f64 * basis),
                        0.20f64 * basis,
                    );
                    let mut bounded: Vector3 = v3(
                        clampd(raw.x, -limits.x, limits.x),
                        clampd(raw.y, -limits.y, limits.y),
                        clampd(raw.z, -limits.z, limits.z),
                    );
                    let mut previous: Vector3 = (*m).force_residual_per_q;
                    let mut innovation: Vector3 = vsub(bounded, previous);
                    let mut residual_alpha: ::core::ffi::c_double = 0.06f64
                        * clampd(certified_confidence, 0.15f64, 1.0f64);
                    (*m).force_residual_per_q = vadd(
                        previous,
                        vscale(innovation, residual_alpha),
                    );
                    let mut variance_alpha: ::core::ffi::c_double = clampd(
                        0.08f64 * certified_confidence,
                        0.02f64,
                        0.10f64,
                    );
                    let mut innovation_sq: Vector3 = v3(
                        innovation.x * innovation.x,
                        innovation.y * innovation.y,
                        innovation.z * innovation.z,
                    );
                    (*m).force_residual_variance_per_q = vadd(
                        vscale(
                            (*m).force_residual_variance_per_q,
                            1.0f64 - variance_alpha,
                        ),
                        vscale(innovation_sq, variance_alpha),
                    );
                    (*m).force_residual_confidence = clampd(
                        (*m).force_residual_confidence + 0.025f64 * certified_confidence,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        0.95f64,
                    );
                }
            }
        } else {
            (*m).rejected = (*m).rejected.wrapping_add(1);
        }
    } else {
        (*m).rejected = (*m).rejected.wrapping_add(1);
    }
    physics_diagnostics(m, t);
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_aero_config(
    mut m: *const VesselPhysicsModel,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut beta: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut gear: bool,
    mut brakes: bool,
    mut airbrakes: ::core::ffi::c_int,
    mut specific: *mut Vector3,
    mut confidence: *mut ::core::ffi::c_double,
) -> bool {
    if specific.is_null() || confidence.is_null() {
        return false_0 != 0;
    }
    *specific = v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    *confidence = 0.0f64;
    if m.is_null()
        || airbrakes != 0 as ::core::ffi::c_int && airbrakes != 1 as ::core::ffi::c_int
        || !(q > 0.0f64) || !(mass > 0.0f64) || q.is_finite() as i32 == 0
        || mach.is_finite() as i32 == 0 || aoa.is_finite() as i32 == 0
        || beta.is_finite() as i32 == 0 || mass.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut estimate_weight: ::core::ffi::c_double = 0.0f64;
    let mut locality_weight: ::core::ffi::c_double = 0.0f64;
    let mut confidence_weight: ::core::ffi::c_double = 0.0f64;
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < (*m).count {
        let mut sample: *const VesselAeroSample = (&raw const (*m).samples
            as *const VesselAeroSample)
            .offset(i as isize) as *const VesselAeroSample;
        if !((*sample).gear as ::core::ffi::c_int != gear as ::core::ffi::c_int
            || (*sample).brakes as ::core::ffi::c_int != brakes as ::core::ffi::c_int
            || (*sample).airbrakes != airbrakes || !((*sample).q > 0.0f64))
        {
            let mut dq: ::core::ffi::c_double = log(q / (*sample).q) / log(2.0f64);
            let mut dm: ::core::ffi::c_double = (mach - (*sample).mach) / 0.5f64;
            let mut da: ::core::ffi::c_double = (aoa - (*sample).aoa) / 10.0f64;
            let mut db: ::core::ffi::c_double = (beta - (*sample).beta) / 10.0f64;
            let mut radius_squared: ::core::ffi::c_double = dq * dq + dm * dm + da * da
                + db * db;
            if radius_squared < 1.0f64 {
                let mut trust: ::core::ffi::c_double = clampd(
                    if (*sample).trust.is_finite() as i32 != 0 {
                        (*sample).trust
                    } else {
                        0.0f64
                    },
                    0.0f64,
                    1.0f64,
                );
                if trust > 0.0f64 {
                    let mut radius: ::core::ffi::c_double = sqrt(
                        fmax(0.0f64, radius_squared),
                    );
                    let mut compact: ::core::ffi::c_double = 1.0f64 - radius;
                    let mut kernel: ::core::ffi::c_double = compact * compact;
                    let mut independent: ::core::ffi::c_double = (if (*sample)
                        .observations != 0
                    {
                        (*sample).observations
                    } else {
                        1 as ::core::ffi::c_uint
                    }) as ::core::ffi::c_double;
                    let mut base_weight: ::core::ffi::c_double = kernel * independent;
                    let mut weight: ::core::ffi::c_double = base_weight * trust;
                    *specific = vadd(*specific, vscale((*sample).force_per_q, weight));
                    estimate_weight += weight;
                    locality_weight += base_weight;
                    confidence_weight += base_weight * trust * compact;
                }
            }
        }
        i = i.wrapping_add(1);
    }
    if !(estimate_weight > 0.0f64) || !(locality_weight > 0.0f64) {
        return false_0 != 0;
    }
    *specific = vscale(*specific, q / (mass * estimate_weight));
    *confidence = clampd(confidence_weight / locality_weight, 0.0f64, 1.0f64);
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_aero(
    mut m: *const VesselPhysicsModel,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut beta: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut specific: *mut Vector3,
    mut confidence: *mut ::core::ffi::c_double,
) -> bool {
    if m.is_null() || (*m).airbrakes < 0 as ::core::ffi::c_int {
        if !specific.is_null() {
            *specific = v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
        }
        if !confidence.is_null() {
            *confidence = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        }
        return false_0 != 0;
    }
    return vessel_physics_aero_config(
        m,
        q,
        mach,
        aoa,
        beta,
        mass,
        (*m).gear,
        (*m).brakes,
        (*m).airbrakes,
        specific,
        confidence,
    );
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_force_config(
    mut m: *const VesselPhysicsModel,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut beta: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut gear: bool,
    mut brakes: bool,
    mut airbrakes: ::core::ffi::c_int,
    mut fallback: AerodynamicModel,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut confidence: *mut ::core::ffi::c_double,
    mut observed: *mut bool,
) -> Vector3 {
    let mut specific: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    *observed = false_0 != 0;
    *confidence = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    if q.is_finite() as i32 == 0 || q < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || mach.is_finite() as i32 == 0 || aoa.is_finite() as i32 == 0
        || mass.is_finite() as i32 == 0
        || mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    let mut stress_drag: ::core::ffi::c_double = if !cal.is_null()
        && (*cal).stress_drag_scale > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        clampd((*cal).stress_drag_scale, 0.5f64, 1.5f64)
    } else {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut stress_lift: ::core::ffi::c_double = if !cal.is_null()
        && (*cal).stress_lift_scale > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        clampd((*cal).stress_lift_scale, 0.5f64, 1.5f64)
    } else {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    *observed = vessel_physics_aero_config(
        m,
        q,
        mach,
        aoa,
        beta,
        mass,
        gear,
        brakes,
        airbrakes,
        &raw mut specific,
        confidence,
    );
    if *observed {
        return v3(
            specific.x * stress_drag,
            specific.y * stress_lift,
            specific.z * stress_lift,
        );
    }
    let mut lf: ::core::ffi::c_double = 0.;
    let mut df: ::core::ffi::c_double = 0.;
    aerodynamic_force_factors_mach(mach, aoa, v, &raw mut lf, &raw mut df);
    let mut drag_scale: ::core::ffi::c_double = if !cal.is_null() {
        clampd(
            (*cal).drag_scale,
            0.3f64,
            3 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    } else {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut lift_scale: ::core::ffi::c_double = if !cal.is_null() {
        clampd(
            (*cal).lift_scale,
            0.3f64,
            3 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    } else {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut base: ::core::ffi::c_double = q
        / fmax(
            20 as ::core::ffi::c_int as ::core::ffi::c_double,
            fallback.ballistic_coefficient,
        ) * drag_scale * stress_drag;
    *confidence = 0.05f64;
    return v3(
        base * df,
        base
            * fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                fallback.lift_to_drag,
            ) * lift_scale * stress_lift * lf,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_force_best_estimate_config(
    mut m: *const VesselPhysicsModel,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut beta: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut gear: bool,
    mut brakes: bool,
    mut airbrakes: ::core::ffi::c_int,
    mut fallback: AerodynamicModel,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut confidence: *mut ::core::ffi::c_double,
    mut observed: *mut bool,
    mut relative_uncertainty: *mut ::core::ffi::c_double,
) -> Vector3 {
    let mut prior: Vector3 = vessel_physics_force_config(
        m,
        q,
        mach,
        aoa,
        beta,
        mass,
        gear,
        brakes,
        airbrakes,
        fallback,
        cal,
        v,
        confidence,
        observed,
    );
    let mut certified: ::core::ffi::c_double = if !m.is_null()
        && (*m).certified_uncertainty > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*m).certified_uncertainty.is_finite() as i32 != 0
    {
        clampd((*m).certified_uncertainty, 0.06f64, 0.90f64)
    } else {
        0.45f64
    };
    if !relative_uncertainty.is_null() {
        *relative_uncertainty = if *observed as ::core::ffi::c_int != 0 {
            certified
        } else {
            fmax(certified, 0.45f64)
        };
    }
    if m.is_null() || !*observed || q <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*m).force_residual_confidence <= 0.02f64
    {
        return prior;
    }
    let mut gain: ::core::ffi::c_double = clampd(
        ((*m).force_residual_confidence - 0.02f64) / 0.55f64,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.78f64,
    );
    let mut correction: Vector3 = vscale(
        (*m).force_residual_per_q,
        q / fmax(mass, 1.0f64) * gain,
    );
    let mut basis: ::core::ffi::c_double = fmax(vmag(prior), 0.02f64);
    let mut limits: Vector3 = v3(
        fmax(0.30f64 * fabs(prior.x), 0.04f64 * basis),
        fmax(0.30f64 * fabs(prior.y), 0.06f64 * basis),
        0.20f64 * basis,
    );
    correction.x = clampd(correction.x, -limits.x, limits.x);
    correction.y = clampd(correction.y, -limits.y, limits.y);
    correction.z = clampd(correction.z, -limits.z, limits.z);
    let mut best: Vector3 = vadd(prior, correction);
    best.x = fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, best.x);
    if !relative_uncertainty.is_null() {
        let mut sigma_per_q: Vector3 = v3(
            sqrt(
                fmax(
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    (*m).force_residual_variance_per_q.x,
                ),
            ),
            sqrt(
                fmax(
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    (*m).force_residual_variance_per_q.y,
                ),
            ),
            sqrt(
                fmax(
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    (*m).force_residual_variance_per_q.z,
                ),
            ),
        );
        let mut sigma_rel: ::core::ffi::c_double = vmag(
            vscale(sigma_per_q, q / fmax(mass, 1.0f64)),
        ) / fmax(vmag(best), 0.05f64);
        let mut prior_term: ::core::ffi::c_double = certified
            * (1.0f64 - 0.25f64 * gain);
        *relative_uncertainty = clampd(
            hypot(
                prior_term,
                clampd(
                    sigma_rel,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0.80f64,
                ) * gain,
            ),
            0.06f64,
            0.90f64,
        );
    }
    return best;
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_force(
    mut m: *const VesselPhysicsModel,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut beta: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut fallback: AerodynamicModel,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut confidence: *mut ::core::ffi::c_double,
    mut observed: *mut bool,
) -> Vector3 {
    let mut gear: bool = if !m.is_null() {
        (*m).gear as ::core::ffi::c_int
    } else {
        false_0
    } != 0;
    let mut brakes: bool = if !m.is_null() {
        (*m).brakes as ::core::ffi::c_int
    } else {
        false_0
    } != 0;
    let mut airbrakes: ::core::ffi::c_int = if !m.is_null()
        && (*m).airbrakes >= 0 as ::core::ffi::c_int
    {
        (*m).airbrakes
    } else {
        0 as ::core::ffi::c_int
    };
    return vessel_physics_force_config(
        m,
        q,
        mach,
        aoa,
        beta,
        mass,
        gear,
        brakes,
        airbrakes,
        fallback,
        cal,
        v,
        confidence,
        observed,
    );
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_acceleration(
    mut position: Vector3,
    mut air: Vector3,
    mut p: *const PlanetModel,
    mut force: Vector3,
    mut bank: ::core::ffi::c_double,
) -> Vector3 {
    let mut dir: Vector3 = vnorm(
        air,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut up: Vector3 = vnorm(
        vproject_plane(position, dir),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut side: Vector3 = vnorm(
        vcross(dir, up),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut b: ::core::ffi::c_double = bank * DEG2RAD;
    let mut lift: Vector3 = vadd(vscale(up, cos(b)), vscale(side, sin(b)));
    let mut lateral: Vector3 = vsub(vscale(side, cos(b)), vscale(up, sin(b)));
    return vadd(
        vessel_physics_gravity(position, p),
        vadd(
            vscale(dir, -force.x),
            vadd(vscale(lift, force.y), vscale(lateral, force.z)),
        ),
    );
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_axis_step(
    mut m: *const VesselPhysicsModel,
    mut axis: ::core::ffi::c_int,
    mut target: ::core::ffi::c_double,
    mut q: ::core::ffi::c_double,
    mut limit: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut angle: *mut ::core::ffi::c_double,
    mut rate: *mut ::core::ffi::c_double,
) {
    if m.is_null() || axis < 0 as ::core::ffi::c_int || axis > 2 as ::core::ffi::c_int
        || dt <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || dt.is_finite() as i32 == 0 || q.is_finite() as i32 == 0
        || target.is_finite() as i32 == 0 || limit.is_finite() as i32 == 0
        || (*angle).is_finite() as i32 == 0 || (*rate).is_finite() as i32 == 0
    {
        return;
    }
    let mut physical_authority: ::core::ffi::c_double = (*m).authority[axis as usize];
    if (*m).authority_q > 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && q < (*m).authority_q
    {
        physical_authority
            *= clampd(
                q / (*m).authority_q,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
    }
    let mut rate_limit: ::core::ffi::c_double = fabs(limit);
    let mut closed_loop_authority: ::core::ffi::c_double = fmin(
        fmax(physical_authority, 0.0f64),
        fmax(0.5f64, rate_limit / 0.75f64 * 2.0f64),
    );
    if closed_loop_authority <= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        return;
    }
    let mut steps: ::core::ffi::c_uint = ceil(
        fmin(dt, 120 as ::core::ffi::c_int as ::core::ffi::c_double) / 0.05f64,
    ) as ::core::ffi::c_uint;
    let mut h: ::core::ffi::c_double = dt
        / fmax(
            steps as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < steps {
        let mut error: ::core::ffi::c_double = norm_signed_deg(target - *angle);
        let mut desired: ::core::ffi::c_double = copysign(
            fmin(
                rate_limit,
                sqrt(
                    2 as ::core::ffi::c_int as ::core::ffi::c_double
                        * closed_loop_authority * fabs(error),
                ),
            ),
            error,
        );
        let mut previous: ::core::ffi::c_double = *rate;
        *rate
            += clampd(
                desired - *rate,
                -closed_loop_authority * h,
                closed_loop_authority * h,
            );
        *angle = norm_signed_deg(*angle + (*rate + previous) * 0.5f64 * h);
        i = i.wrapping_add(1);
    }
}
pub const VESSEL_AERO_SAMPLES: ::core::ffi::c_int = 192 as ::core::ffi::c_int;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
