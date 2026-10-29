use crate::sys::*;
extern "C" {
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmod(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn copysign(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn nextafter(
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
pub struct EntryExecutive {
    pub initialized: bool,
    pub entry_complete: bool,
    pub phase: EntryPhase,
    pub last_transition_reason: EntryTransitionReason,
    pub last_transition_from: EntryPhase,
    pub last_transition_to: EntryPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_entry: bool,
    pub transition_count: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralState {
    pub initialized: bool,
    pub bank_sign: ::core::ffi::c_double,
    pub leg_captured: bool,
    pub leg_captured_ut: ::core::ffi::c_double,
    pub reversal_armed: bool,
    pub last_reversal_ut: ::core::ffi::c_double,
    pub command_bank_deg: ::core::ffi::c_double,
    pub command_bank_rate_deg_s: ::core::ffi::c_double,
    pub has_previous_sideslip: bool,
    pub previous_sideslip_deg: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryEnergyState {
    pub initialized: bool,
    pub integral_m_s: ::core::ffi::c_double,
    pub last_ut: ::core::ffi::c_double,
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
pub struct TaemExecutive {
    pub initialized: bool,
    pub ownership_latched: bool,
    pub taem_complete: bool,
    pub phase: TaemPhase,
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
pub type TerminalVerticalStage = ::core::ffi::c_uint;
pub const TERMINAL_GROUND: TerminalVerticalStage = 5;
pub const TERMINAL_TOUCHDOWN_FLARE: TerminalVerticalStage = 4;
pub const TERMINAL_INNER_FINAL: TerminalVerticalStage = 3;
pub const TERMINAL_PREFLARE: TerminalVerticalStage = 2;
pub const TERMINAL_OUTER_FINAL: TerminalVerticalStage = 1;
pub const TERMINAL_TRAJECTORY_CAPTURE: TerminalVerticalStage = 0;
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
pub struct FinalAlignmentContract {
    pub valid: bool,
    pub distance_m: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
    pub speed_mps: ::core::ffi::c_double,
    pub slope_deg: ::core::ffi::c_double,
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
pub struct TaemPoint2 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFixedHacGeometry {
    pub entry: TaemPoint2,
    pub center: TaemPoint2,
    pub exit: TaemPoint2,
    pub radius_m: ::core::ffi::c_double,
    pub side: ::core::ffi::c_double,
    pub capture_course_deg: ::core::ffi::c_double,
    pub arc_sweep_rad: ::core::ffi::c_double,
    pub arc_length_m: ::core::ffi::c_double,
    pub final_length_m: ::core::ffi::c_double,
    pub total_length_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRoute {
    pub valid: bool,
    pub count: size_t,
    pub lead_count: size_t,
    pub arc_count: size_t,
    pub rollout_count: size_t,
    pub length_m: ::core::ffi::c_double,
    pub side: ::core::ffi::c_double,
    pub hac: TaemFixedHacGeometry,
    pub p0_along_m: ::core::ffi::c_double,
    pub p0_cross_m: ::core::ffi::c_double,
    pub p1_along_m: ::core::ffi::c_double,
    pub p1_cross_m: ::core::ffi::c_double,
    pub p2_along_m: ::core::ffi::c_double,
    pub p2_cross_m: ::core::ffi::c_double,
    pub p3_along_m: ::core::ffi::c_double,
    pub p3_cross_m: ::core::ffi::c_double,
    pub capture_control: [TaemPoint2; 6],
    pub spiral_exit_radius_m: ::core::ffi::c_double,
    pub spiral_coefficient_m: ::core::ffi::c_double,
    pub spiral_exit_heading_rad: ::core::ffi::c_double,
    pub spiral_arc_lut: [::core::ffi::c_double; 129],
    pub unload_length_m: ::core::ffi::c_double,
    pub settle_length_m: ::core::ffi::c_double,
    pub lead_length_m: ::core::ffi::c_double,
    pub lead_arc_fraction_lut: [::core::ffi::c_float; 129],
    pub rollout_length_m: ::core::ffi::c_double,
    pub alignment_along_m: ::core::ffi::c_double,
    pub initial_course_deg: ::core::ffi::c_double,
    pub runway_heading_deg: ::core::ffi::c_double,
    pub profile_start_altitude_m: ::core::ffi::c_double,
    pub profile_start_fpa_deg: ::core::ffi::c_double,
    pub profile_final_altitude_m: ::core::ffi::c_double,
    pub profile_final_slope_deg: ::core::ffi::c_double,
    pub profile_total_length_m: ::core::ffi::c_double,
    pub profile_midpoint_offset_m: ::core::ffi::c_double,
    pub profile_local_offset_m: ::core::ffi::c_double,
    pub profile_local_start_fraction: ::core::ffi::c_double,
    pub profile_local_peak_fraction: ::core::ffi::c_double,
    pub profile_local_end_fraction: ::core::ffi::c_double,
    pub profile_initial_sag_m: ::core::ffi::c_double,
    pub profile_initial_sag_length_m: ::core::ffi::c_double,
    pub profile_tabulated: bool,
    pub profile_generation_aoa_deg: ::core::ffi::c_double,
    pub profile_altitude_lut: [::core::ffi::c_float; 257],
    pub profile_fpa_lut: [::core::ffi::c_float; 257],
    pub profile_speed_tabulated: bool,
    pub profile_speed_lut: [::core::ffi::c_float; 257],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AdaptiveTrim {
    pub weight: ::core::ffi::c_double,
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub xx: ::core::ffi::c_double,
    pub xy: ::core::ffi::c_double,
    pub yy: ::core::ffi::c_double,
    pub lo: ::core::ffi::c_double,
    pub hi: ::core::ffi::c_double,
    pub last_ut: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub pressure: ::core::ffi::c_double,
    pub clock_valid: bool,
    pub regime_valid: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RobustPID {
    pub kp: ::core::ffi::c_double,
    pub ki: ::core::ffi::c_double,
    pub kd: ::core::ffi::c_double,
    pub integral_limit: ::core::ffi::c_double,
    pub derivative_tc: ::core::ffi::c_double,
    pub integral: ::core::ffi::c_double,
    pub previous_error: ::core::ffi::c_double,
    pub filtered_derivative: ::core::ffi::c_double,
    pub has_previous: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SlewLimiter {
    pub value: ::core::ffi::c_double,
    pub has_value: bool,
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
pub struct LowPass {
    pub tc: ::core::ffi::c_double,
    pub value: ::core::ffi::c_double,
    pub has_value: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SpeedbrakeController {
    pub integral: ::core::ffi::c_double,
    pub deployed: bool,
    pub initialized: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlRegime {
    pub command_rate_scale: ::core::ffi::c_double,
    pub command_accel_scale: ::core::ffi::c_double,
    pub response_scale: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryTerminalDemand {
    pub remaining: ::core::ffi::c_double,
    pub early: ::core::ffi::c_double,
    pub projected_taem_range_error: ::core::ffi::c_double,
    pub range_norm: ::core::ffi::c_double,
    pub energy_norm: ::core::ffi::c_double,
    pub raw_score: ::core::ffi::c_double,
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
pub struct TaemInterfaceTarget {
    pub valid: bool,
    pub energy_qualified: bool,
    pub along_track: ::core::ffi::c_double,
    pub cross_track: ::core::ffi::c_double,
    pub course: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub speed: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub specific_energy: ::core::ffi::c_double,
    pub selected_ut: ::core::ffi::c_double,
    pub arrival_ut: ::core::ffi::c_double,
    pub quality_score: ::core::ffi::c_double,
    pub acquisition_lead: ::core::ffi::c_double,
    pub remaining_path: ::core::ffi::c_double,
    pub response_time: ::core::ffi::c_double,
    pub hac_radius: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemHandoffContract {
    pub horizontal_radius_m: ::core::ffi::c_double,
    pub perpendicular_heading_half_width_deg: ::core::ffi::c_double,
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
pub type EntrySupervisionMode = ::core::ffi::c_uint;
pub const ENTRY_SUPERVISION_INFEASIBLE: EntrySupervisionMode = 3;
pub const ENTRY_SUPERVISION_CONSERVATIVE_FALLBACK: EntrySupervisionMode = 2;
pub const ENTRY_SUPERVISION_BOUNDED_CORRECTION: EntrySupervisionMode = 1;
pub const ENTRY_SUPERVISION_PASS_THROUGH: EntrySupervisionMode = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemEnergyLawState {
    pub initialized: bool,
    pub loop_0: bool,
    pub capture_forecast: bool,
    pub leg: ::core::ffi::c_int,
    pub side: ::core::ffi::c_int,
    pub radius: ::core::ffi::c_double,
    pub r_turn: ::core::ffi::c_double,
    pub x_turn: ::core::ffi::c_double,
    pub turn_cx: ::core::ffi::c_double,
    pub turn_cy: ::core::ffi::c_double,
    pub k: ::core::ffi::c_double,
    pub s_req: ::core::ffi::c_double,
    pub s_go: ::core::ffi::c_double,
    pub last_geo_ut: ::core::ffi::c_double,
    pub last_pred_ut: ::core::ffi::c_double,
    pub admit_pending: bool,
    pub admit_since_ut: ::core::ffi::c_double,
    pub low_energy_pending: bool,
    pub low_energy_since_ut: ::core::ffi::c_double,
    pub trace_leg: ::core::ffi::c_int,
    pub trace_side: ::core::ffi::c_int,
    pub trace_radius: ::core::ffi::c_double,
    pub trace_loop: bool,
    pub hac_left_rad: ::core::ffi::c_double,
    pub hac_last_ang: ::core::ffi::c_double,
    pub weave_on: bool,
    pub weave_sign: ::core::ffi::c_int,
    pub weave_brg: ::core::ffi::c_double,
    pub station_slope_deg: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemHacLiveAero {
    pub drag: [::core::ffi::c_double; 15],
    pub lift: [::core::ffi::c_double; 15],
    pub weight: [::core::ffi::c_double; 15],
    pub last_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemHacReference {
    pub valid: bool,
    pub ds: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub built_ut: ::core::ffi::c_double,
    pub aoa: [::core::ffi::c_double; 41],
    pub cl: [::core::ffi::c_double; 41],
    pub e: [::core::ffi::c_double; 480],
    pub h: [::core::ffi::c_double; 480],
    pub v: [::core::ffi::c_double; 480],
    pub h_eq: [::core::ffi::c_double; 480],
    pub v_eq: [::core::ffi::c_double; 480],
    pub s_gate: ::core::ffi::c_double,
    pub e_gate: ::core::ffi::c_double,
    pub h_gate: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemHacLawState {
    pub initialized: bool,
    pub final_line: bool,
    pub phase: ::core::ffi::c_int,
    pub side: ::core::ffi::c_int,
    pub s_turn_sign: ::core::ffi::c_int,
    pub drag_sign: ::core::ffi::c_int,
    pub drag_away_sign: ::core::ffi::c_int,
    pub drag_away: bool,
    pub drag_flown: bool,
    pub drag_aoa_cmd: ::core::ffi::c_double,
    pub drag_bank_cmd: ::core::ffi::c_double,
    pub trace: bool,
    pub drag_required: ::core::ffi::c_double,
    pub rf: ::core::ffi::c_double,
    pub psi: ::core::ffi::c_double,
    pub last_ang: ::core::ffi::c_double,
    pub s_go: ::core::ffi::c_double,
    pub e_ref: ::core::ffi::c_double,
    pub h_ref: ::core::ffi::c_double,
    pub v_ref: ::core::ffi::c_double,
    pub e_err: ::core::ffi::c_double,
    pub energy_level: ::core::ffi::c_double,
    pub tangent_x: ::core::ffi::c_double,
    pub tangent_y: ::core::ffi::c_double,
    pub tangent_dist: ::core::ffi::c_double,
    pub turn_cost: ::core::ffi::c_double,
    pub sdot: ::core::ffi::c_double,
    pub last_sgo: ::core::ffi::c_double,
    pub last_sgo_ut: ::core::ffi::c_double,
    pub eta_live: ::core::ffi::c_double,
    pub last_plan_ut: ::core::ffi::c_double,
    pub last_side_ut: ::core::ffi::c_double,
    pub last_ut: ::core::ffi::c_double,
    pub speedbrake_integral: ::core::ffi::c_double,
    pub station_slope_deg: ::core::ffi::c_double,
    pub admit_pending: bool,
    pub admit_since_ut: ::core::ffi::c_double,
    pub low_energy_pending: bool,
    pub low_energy_since_ut: ::core::ffi::c_double,
    pub ref_0: TaemHacReference,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AdaptiveFlareState {
    pub sxx: [[::core::ffi::c_double; 3]; 3],
    pub sxy: [::core::ffi::c_double; 3],
    pub last_aoa: ::core::ffi::c_double,
    pub last_gamma: ::core::ffi::c_double,
    pub last_ut: ::core::ffi::c_double,
    pub has_last: bool,
    pub tau: ::core::ffi::c_double,
    pub bias: ::core::ffi::c_double,
    pub gain: ::core::ffi::c_double,
    pub glide_sink: ::core::ffi::c_double,
    pub aim_shift: ::core::ffi::c_double,
    pub weight: ::core::ffi::c_double,
    pub plan_valid: bool,
    pub active: bool,
    pub arc_accel: ::core::ffi::c_double,
    pub sgs_height: ::core::ffi::c_double,
    pub cost: ::core::ffi::c_double,
    pub pred_speed: ::core::ffi::c_double,
    pub pred_sink: ::core::ffi::c_double,
    pub pred_along: ::core::ffi::c_double,
    pub pred_peak_n: ::core::ffi::c_double,
    pub pred_climb: ::core::ffi::c_double,
    pub plan_ut: ::core::ffi::c_double,
}
pub type TerminalPathKind = ::core::ffi::c_uint;
pub const TERMINAL_PATH_HAC: TerminalPathKind = 2;
pub const TERMINAL_PATH_DIRECT: TerminalPathKind = 1;
pub const TERMINAL_PATH_NONE: TerminalPathKind = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GuidanceMachine {
    pub phase: GuidancePhase,
    pub automation_engaged: bool,
    pub paused: bool,
    pub aborted: bool,
    pub entry_terminal_model: *const TerminalModel,
    pub diagnostic_shadow: bool,
    pub diagnostic_stop_at_taem: bool,
    pub diagnostic_entry_previous_range_m: ::core::ffi::c_double,
    pub diagnostic_entry_milestone_mask: ::core::ffi::c_uint,
    pub diagnostic_entry_has_previous_range: bool,
    pub s_turn_sign: ::core::ffi::c_double,
    pub s_turn_leg_started_ut: ::core::ffi::c_double,
    pub has_s_turn_leg_started: bool,
    pub s_turn_reversal_requested_ut: ::core::ffi::c_double,
    pub has_s_turn_reversal_requested: bool,
    pub entry_exec: EntryExecutive,
    pub entry_lateral: EntryLateralState,
    pub entry_energy: EntryEnergyState,
    pub taem_exec: TaemExecutive,
    pub entry_alpha_has_target: bool,
    pub entry_alpha_has_modulation: bool,
    pub entry_drag_ratio_valid: bool,
    pub entry_alpha_target: ::core::ffi::c_double,
    pub entry_alpha_modulation: ::core::ffi::c_double,
    pub entry_alpha_drag_boost: ::core::ffi::c_double,
    pub entry_drag_velocity_ratio: ::core::ffi::c_double,
    pub entry_alpha_deficit_filter: ::core::ffi::c_double,
    pub entry_alpha_ceiling_filter: ::core::ffi::c_double,
    pub entry_alpha_filter_valid: bool,
    pub entry_bank_authority_acquired: bool,
    pub entry_supervision_valid: bool,
    pub entry_supervision_boundary_missed: bool,
    pub entry_supervision_mode: EntrySupervisionMode,
    pub entry_policy_valid: bool,
    pub entry_policy_bank_deg: ::core::ffi::c_double,
    pub entry_policy_aoa_deg: ::core::ffi::c_double,
    pub entry_policy_ut: ::core::ffi::c_double,
    pub entry_energy_short_latched: bool,
    pub entry_handoff_target_range: ::core::ffi::c_double,
    pub entry_hac_energy_excess: ::core::ffi::c_double,
    pub entry_hac_energy_ut: ::core::ffi::c_double,
    pub taem_runway_end_scored: bool,
    pub taem_runway_end_score_ut: ::core::ffi::c_double,
    pub entry_supervision_ut: ::core::ffi::c_double,
    pub entry_planning_deferred: bool,
    pub entry_planning_needed: bool,
    pub entry_lateral_infeasible: bool,
    pub entry_committed_infeasible: bool,
    pub entry_s_turn_plan: EntryControlPlan,
    pub entry_predictor_models_valid: bool,
    pub entry_predictor_envelope: AerodynamicEnvelope,
    pub entry_predictor_calibration: TrajectoryCalibrationModel,
    pub mm305_route: TaemRoute,
    pub mm305_route_cursor: size_t,
    pub mm305_route_committed: bool,
    pub mm305_hac_exit_reached: bool,
    pub mm305_direct_approach_degraded: bool,
    pub mm305_best_effort_route_active: bool,
    pub mm305_runway_alignment_active: bool,
    pub mm305_model_snapshot_id: uint64_t,
    pub mm305_acquisition_route: TaemRoute,
    pub mm305_acquisition_route_cursor: size_t,
    pub mm305_acquisition_route_valid: bool,
    pub mm305_acquisition_route_ut: ::core::ffi::c_double,
    pub mm305_acquisition_sweep_rad: ::core::ffi::c_double,
    pub mm305_async_planning: bool,
    pub mm305_planning_needed: bool,
    pub mm305_plan_request_ut: ::core::ffi::c_double,
    pub mm305_last_plan_attempt_ut: ::core::ffi::c_double,
    pub mm305_last_route_ut: ::core::ffi::c_double,
    pub mm305_plan_failures: ::core::ffi::c_uint,
    pub mm305_replans: ::core::ffi::c_uint,
    pub mm305_lift_scale: ::core::ffi::c_double,
    pub mm305_drag_scale: ::core::ffi::c_double,
    pub mm305_command_bank_rad: ::core::ffi::c_double,
    pub mm305_command_aoa_rad: ::core::ffi::c_double,
    pub mm305_command_ut: ::core::ffi::c_double,
    pub mm305_command_valid: bool,
    pub terminal_landing_warning_latched: bool,
    pub mm305_trim: AdaptiveTrim,
    pub mm305_admission_needed: bool,
    pub mm305_admission_request_ut: ::core::ffi::c_double,
    pub mm305_admission_last_attempt_ut: ::core::ffi::c_double,
    pub mm305_admission_valid: bool,
    pub mm305_admission_found: bool,
    pub mm305_admission_result_ut: ::core::ffi::c_double,
    pub mm305_admission_snapshot_id: uint64_t,
    pub mm305_admission_route: TaemRoute,
    pub mm305_admission_runway_end: ::core::ffi::c_int,
    pub mm305_admission_side: ::core::ffi::c_double,
    pub mm305_admission_solve_wall_s: ::core::ffi::c_double,
    pub mm305_admission_lift_scale: ::core::ffi::c_double,
    pub mm305_admission_drag_scale: ::core::ffi::c_double,
    pub mm305_admission_scale_mach: ::core::ffi::c_double,
    pub mm305_admission_rejections: ::core::ffi::c_uint,
    pub taem_interface_target: TaemInterfaceTarget,
    pub taem_interface_captured: bool,
    pub taem_safety_handoff: bool,
    pub taem_interface_diagnostic_ut: ::core::ffi::c_double,
    pub terminal_path_kind: TerminalPathKind,
    pub terminal_path_committed: bool,
    pub runway_end_preview_valid: bool,
    pub runway_end_committed: bool,
    pub runway_end_index: ::core::ffi::c_int,
    pub runway_end_primary_path: ::core::ffi::c_double,
    pub runway_end_reciprocal_path: ::core::ffi::c_double,
    pub terminal_final_handoff_latched: bool,
    pub terminal_final_handoff_distance: ::core::ffi::c_double,
    pub terminal_final_handoff_slope_deg: ::core::ffi::c_double,
    pub terminal_final_handoff_speed_mps: ::core::ffi::c_double,
    pub hac_side: ::core::ffi::c_double,
    pub hac_side_selected: bool,
    pub delivered_delta_v: ::core::ffi::c_double,
    pub burn_command_started_ut: ::core::ffi::c_double,
    pub burn_active_elapsed: ::core::ffi::c_double,
    pub burn_progress_watch_ut: ::core::ffi::c_double,
    pub burn_progress_watch_delta_v: ::core::ffi::c_double,
    pub burn_thrust_loss_elapsed: ::core::ffi::c_double,
    pub has_burn_command_started: bool,
    pub has_burn_progress_watch: bool,
    pub deorbit_burn_completed: bool,
    pub atmospheric_interface_crossed: bool,
    pub has_previous_ut: bool,
    pub previous_ut: ::core::ffi::c_double,
    pub final_approach_captured: bool,
    pub airbrakes_deployed: bool,
    pub terminal_final_low_energy: bool,
    pub hac_progress_valid: bool,
    pub hac_captured: bool,
    pub hac_completed: bool,
    pub terminal_region_entered: bool,
    pub terminal_glide_mode: bool,
    pub terminal_final_test_mode: bool,
    pub terminal_rehearsal_mode: bool,
    pub terminal_test_capture_active: bool,
    pub hac_capture_lost_duration: ::core::ffi::c_double,
    pub terminal_energy_mismatch_duration: ::core::ffi::c_double,
    pub terminal_reentry_after_ut: ::core::ffi::c_double,
    pub hac_remaining: ::core::ffi::c_double,
    pub hac_radius: ::core::ffi::c_double,
    pub minimum_turn_radius: ::core::ffi::c_double,
    pub terminal_test_glide_slope: ::core::ffi::c_double,
    pub terminal_test_final_approach_distance: ::core::ffi::c_double,
    pub final_invalid_duration: ::core::ffi::c_double,
    pub terminal_test_revolution_remaining: ::core::ffi::c_double,
    pub terminal_test_preflare_altitude: ::core::ffi::c_double,
    pub terminal_test_preflare_target_speed: ::core::ffi::c_double,
    pub terminal_test_preflare_min_speed: ::core::ffi::c_double,
    pub terminal_vertical_stage: TerminalVerticalStage,
    pub terminal_vertical_stage_valid: bool,
    pub gear_command_latched: bool,
    pub ground_contact_latched: bool,
    pub flare_sink_captured: bool,
    pub terminal_response_sample_valid: bool,
    pub terminal_preflare_plan_valid: bool,
    pub terminal_energy_sample_valid: bool,
    pub terminal_stage_started_ut: ::core::ffi::c_double,
    pub terminal_stage_good_duration: ::core::ffi::c_double,
    pub ground_contact_duration: ::core::ffi::c_double,
    pub terminal_previous_aoa: ::core::ffi::c_double,
    pub terminal_previous_vertical_speed: ::core::ffi::c_double,
    pub terminal_response_sample_ut: ::core::ffi::c_double,
    pub terminal_previous_specific_energy: ::core::ffi::c_double,
    pub terminal_previous_speed: ::core::ffi::c_double,
    pub terminal_energy_sample_ut: ::core::ffi::c_double,
    pub terminal_energy_loss_accel_ema: ::core::ffi::c_double,
    pub terminal_speed_loss_accel_ema: ::core::ffi::c_double,
    pub terminal_lift_area_ema: ::core::ffi::c_double,
    pub terminal_drag_area_ema: ::core::ffi::c_double,
    pub terminal_aero_sample_ut: ::core::ffi::c_double,
    pub terminal_aero_sample_valid: bool,
    pub terminal_predicted_touchdown_speed: ::core::ffi::c_double,
    pub terminal_speedbrake_cda: ::core::ffi::c_double,
    pub terminal_speedbrake_fraction: ::core::ffi::c_double,
    pub terminal_split_fin_cda: ::core::ffi::c_double,
    pub terminal_vs_prev: ::core::ffi::c_double,
    pub terminal_vs_prev_ut: ::core::ffi::c_double,
    pub terminal_vs_rate_ema: ::core::ffi::c_double,
    pub terminal_lift_q: [::core::ffi::c_double; 21],
    pub terminal_drag_q: [::core::ffi::c_double; 21],
    pub terminal_aero_seen: [::core::ffi::c_uchar; 21],
    pub terminal_aero_mach: [::core::ffi::c_double; 21],
    pub terminal_aoa_cmd_last: ::core::ffi::c_double,
    pub terminal_aoa_cmd_ut: ::core::ffi::c_double,
    pub taem_speedbrake_integral: ::core::ffi::c_double,
    pub taem_speedbrake_ut: ::core::ffi::c_double,
    pub taem_speedbrake_fraction: ::core::ffi::c_double,
    pub terminal_pull_latch: bool,
    pub terminal_arc_started: bool,
    pub terminal_aim_valid: bool,
    pub terminal_aim_along: ::core::ffi::c_double,
    pub terminal_aim_shift: ::core::ffi::c_double,
    pub terminal_predicted_touchdown_along: ::core::ffi::c_double,
    pub terminal_aim_ut: ::core::ffi::c_double,
    pub terminal_lift_ratio: ::core::ffi::c_double,
    pub terminal_positive_aoa_rate_ema: ::core::ffi::c_double,
    pub terminal_sink_accel_ema: ::core::ffi::c_double,
    pub terminal_pitch_response_delay_ema: ::core::ffi::c_double,
    pub preflare_trigger_altitude: ::core::ffi::c_double,
    pub preflare_target_aoa: ::core::ffi::c_double,
    pub preflare_target_sink: ::core::ffi::c_double,
    pub preflare_minimum_speed: ::core::ffi::c_double,
    pub preflare_reference_speed: ::core::ffi::c_double,
    pub preflare_predicted_height_loss: ::core::ffi::c_double,
    pub preflare_predicted_kinetic_margin: ::core::ffi::c_double,
    pub preflare_effective_accel: ::core::ffi::c_double,
    pub preflare_response_time: ::core::ffi::c_double,
    pub preflare_plan_source_ut: ::core::ffi::c_double,
    pub attitude_recovery: bool,
    pub control_bad_duration: ::core::ffi::c_double,
    pub control_good_duration: ::core::ffi::c_double,
    pub recovery_duration: ::core::ffi::c_double,
    pub recovery_heading: ::core::ffi::c_double,
    pub recovery_aoa: ::core::ffi::c_double,
    pub recovery_pitch_trim: ::core::ffi::c_double,
    pub previous_relative_roll_rate: ::core::ffi::c_double,
    pub roll_oscillation_score: ::core::ffi::c_double,
    pub roll_rate_excess_duration: ::core::ffi::c_double,
    pub has_previous_relative_roll_rate: bool,
    pub previous_sideslip: ::core::ffi::c_double,
    pub entry_coordination_beta_rate: ::core::ffi::c_double,
    pub has_previous_sideslip: bool,
    pub entry_predictive_score: ::core::ffi::c_double,
    pub entry_reference_speed: ::core::ffi::c_double,
    pub entry_reference_altitude: ::core::ffi::c_double,
    pub entry_reference_range: ::core::ffi::c_double,
    pub has_entry_predictive_score: bool,
    pub entry_control_plan_valid: bool,
    pub entry_control_terminal_ready: bool,
    pub control_plan_sequence: uint64_t,
    pub entry_control_plan_ut: ::core::ffi::c_double,
    pub entry_control_bank: ::core::ffi::c_double,
    pub entry_control_aoa: ::core::ffi::c_double,
    pub entry_control_heading: ::core::ffi::c_double,
    pub entry_control_turn_radius: ::core::ffi::c_double,
    pub entry_control_segment_until_ut: ::core::ffi::c_double,
    pub entry_control_cost: ::core::ffi::c_double,
    pub entry_control_taem_range_error: ::core::ffi::c_double,
    pub entry_control_taem_speed: ::core::ffi::c_double,
    pub entry_control_taem_energy_error: ::core::ffi::c_double,
    pub entry_reversal_scheduled: bool,
    pub entry_reversal_is_final: bool,
    pub entry_final_reversal_pending: bool,
    pub entry_final_reversal_completed: bool,
    pub entry_continuation_bootstrap: bool,
    pub entry_target_side_latched: bool,
    pub entry_target_side: ::core::ffi::c_double,
    pub entry_reversal_ut: ::core::ffi::c_double,
    pub entry_reversal_range: ::core::ffi::c_double,
    pub entry_reversal_sign: ::core::ffi::c_double,
    pub entry_reversal_bank: ::core::ffi::c_double,
    pub entry_control_reversals: ::core::ffi::c_uint,
    pub entry_feedback: EntryGuidanceFeedback,
    pub entry_lateral_bank_magnitude: ::core::ffi::c_double,
    pub entry_geometry_bank: ::core::ffi::c_double,
    pub entry_vertical_bank_magnitude: ::core::ffi::c_double,
    pub entry_demand_bank: ::core::ffi::c_double,
    pub entry_lateral_required_bank: ::core::ffi::c_double,
    pub abort_reason: [::core::ffi::c_char; 384],
    pub entry_energy_pid: RobustPID,
    pub taem_altitude_pid: RobustPID,
    pub final_altitude_pid: RobustPID,
    pub speed_pid: RobustPID,
    pub flare_sink_pid: RobustPID,
    pub speedbrake_controller: SpeedbrakeController,
    pub pitch_limiter: JerkLimiter,
    pub roll_limiter: JerkLimiter,
    pub heading_limiter: JerkLimiter,
    pub throttle_limiter: SlewLimiter,
    pub last_stabilized_phase: GuidancePhase,
    pub has_last_stabilized_phase: bool,
    pub taem_energy_law: TaemEnergyLawState,
    pub taem_hac_law: TaemHacLawState,
    pub taem_hac_live_aero: TaemHacLiveAero,
    pub adaptive_flare: AdaptiveFlareState,
    pub jet_assist_throttle: ::core::ffi::c_double,
    pub mm304: crate::mm304::Mm304State,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct StockKspCurveKey {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub in_tangent: ::core::ffi::c_double,
    pub out_tangent: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const LANDER_ATMOSPHERE_SAMPLE_MAX: ::core::ffi::c_int = 1024 as ::core::ffi::c_int;
#[no_mangle]
pub unsafe extern "C" fn clampd(
    mut v: ::core::ffi::c_double,
    mut lo: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return fmin(fmax(v, lo), hi);
}
#[no_mangle]
pub unsafe extern "C" fn norm_deg(
    mut a: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = fmod(a, 360.0f64);
    return if r < 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        r + 360.0f64
    } else {
        r
    };
}
#[no_mangle]
pub unsafe extern "C" fn norm_signed_deg(
    mut a: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = norm_deg(a);
    return if r > 180.0f64 { r - 360.0f64 } else { r };
}
#[no_mangle]
pub unsafe extern "C" fn taem_handoff_contract(
    mut s: *const GuidanceSettings,
) -> TaemHandoffContract {
    let mut out: TaemHandoffContract = TaemHandoffContract {
        horizontal_radius_m: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        perpendicular_heading_half_width_deg: 0.,
    };
    if s.is_null() {
        return out;
    }
    out.horizontal_radius_m = (*s).mm304_handoff_radius;
    out.perpendicular_heading_half_width_deg = (*s)
        .mm304_perpendicular_heading_half_width;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn mm304_handoff_station(
    mut settings: *const GuidanceSettings,
    mut along_m: *mut ::core::ffi::c_double,
    mut cross_m: *mut ::core::ffi::c_double,
) {
    if !along_m.is_null() {
        *along_m = if !settings.is_null() {
            (*settings).mm304_handoff_along_track
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
    }
    if !cross_m.is_null() {
        *cross_m = if !settings.is_null() {
            (*settings).mm304_handoff_cross_track
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
    }
}
#[no_mangle]
pub unsafe extern "C" fn entry_s_turn_effective_minimum_leg(
    mut true_air_speed: ::core::ffi::c_double,
    mut taem_speed: ::core::ffi::c_double,
    mut settings: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    return if !settings.is_null() {
        fmax(0.0f64, (*settings).s_turn_minimum_leg_duration)
    } else {
        0.0f64
    };
}
#[no_mangle]
pub unsafe extern "C" fn v3(
    mut x: ::core::ffi::c_double,
    mut y: ::core::ffi::c_double,
    mut z: ::core::ffi::c_double,
) -> Vector3 {
    let mut r: Vector3 = Vector3 { x: x, y: y, z: z };
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn vadd(mut a: Vector3, mut b: Vector3) -> Vector3 {
    return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
#[no_mangle]
pub unsafe extern "C" fn vsub(mut a: Vector3, mut b: Vector3) -> Vector3 {
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
#[no_mangle]
pub unsafe extern "C" fn vscale(
    mut a: Vector3,
    mut s: ::core::ffi::c_double,
) -> Vector3 {
    return v3(a.x * s, a.y * s, a.z * s);
}
#[no_mangle]
pub unsafe extern "C" fn vdot(mut a: Vector3, mut b: Vector3) -> ::core::ffi::c_double {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
#[no_mangle]
pub unsafe extern "C" fn vcross(mut a: Vector3, mut b: Vector3) -> Vector3 {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
#[no_mangle]
pub unsafe extern "C" fn vmag(mut a: Vector3) -> ::core::ffi::c_double {
    return hypot(hypot(a.x, a.y), a.z);
}
#[no_mangle]
pub unsafe extern "C" fn vnorm(mut a: Vector3, mut fallback: Vector3) -> Vector3 {
    let mut m: ::core::ffi::c_double = vmag(a);
    return if m.is_finite() as i32 != 0 && m > 0.0f64 {
        vscale(a, 1.0f64 / m)
    } else {
        fallback
    };
}
#[no_mangle]
pub unsafe extern "C" fn vproject_plane(mut a: Vector3, mut normal: Vector3) -> Vector3 {
    return vsub(a, vscale(normal, vdot(a, normal)));
}
#[no_mangle]
pub unsafe extern "C" fn vrotate(
    mut a: Vector3,
    mut axis: Vector3,
    mut radians: ::core::ffi::c_double,
) -> Vector3 {
    axis = vnorm(
        axis,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut c: ::core::ffi::c_double = cos(radians);
    let mut s: ::core::ffi::c_double = sin(radians);
    return vadd(
        vadd(vscale(a, c), vscale(vcross(axis, a), s)),
        vscale(
            axis,
            vdot(axis, a) * (1 as ::core::ffi::c_int as ::core::ffi::c_double - c),
        ),
    );
}
#[no_mangle]
pub unsafe extern "C" fn great_circle_distance(
    mut a: GeoPoint,
    mut b: GeoPoint,
    mut radius: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut lat1: ::core::ffi::c_double = a.latitude * DEG2RAD;
    let mut lat2: ::core::ffi::c_double = b.latitude * DEG2RAD;
    let mut dlat: ::core::ffi::c_double = (b.latitude - a.latitude) * DEG2RAD;
    let mut dlon: ::core::ffi::c_double = norm_signed_deg(b.longitude - a.longitude)
        * DEG2RAD;
    let mut h: ::core::ffi::c_double = sin(
        dlat / 2 as ::core::ffi::c_int as ::core::ffi::c_double,
    ) * sin(dlat / 2 as ::core::ffi::c_int as ::core::ffi::c_double)
        + cos(lat1) * cos(lat2)
            * sin(dlon / 2 as ::core::ffi::c_int as ::core::ffi::c_double)
            * sin(dlon / 2 as ::core::ffi::c_int as ::core::ffi::c_double);
    return radius * 2 as ::core::ffi::c_int as ::core::ffi::c_double
        * atan2(
            sqrt(h),
            sqrt(
                fmax(
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    1 as ::core::ffi::c_int as ::core::ffi::c_double - h,
                ),
            ),
        );
}
#[no_mangle]
pub unsafe extern "C" fn initial_bearing(
    mut a: GeoPoint,
    mut b: GeoPoint,
) -> ::core::ffi::c_double {
    let mut lat1: ::core::ffi::c_double = a.latitude * DEG2RAD;
    let mut lat2: ::core::ffi::c_double = b.latitude * DEG2RAD;
    let mut dlon: ::core::ffi::c_double = norm_signed_deg(b.longitude - a.longitude)
        * DEG2RAD;
    let mut y: ::core::ffi::c_double = sin(dlon) * cos(lat2);
    let mut x: ::core::ffi::c_double = cos(lat1) * sin(lat2)
        - sin(lat1) * cos(lat2) * cos(dlon);
    return norm_deg(atan2(y, x) * RAD2DEG);
}
#[no_mangle]
pub unsafe extern "C" fn destination_point(
    mut a: GeoPoint,
    mut bearing: ::core::ffi::c_double,
    mut distance: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
) -> GeoPoint {
    let mut ad: ::core::ffi::c_double = distance / radius;
    let mut th: ::core::ffi::c_double = bearing * DEG2RAD;
    let mut lat1: ::core::ffi::c_double = a.latitude * DEG2RAD;
    let mut lon1: ::core::ffi::c_double = a.longitude * DEG2RAD;
    let mut lat2: ::core::ffi::c_double = asin(
        sin(lat1) * cos(ad) + cos(lat1) * sin(ad) * cos(th),
    );
    let mut lon2: ::core::ffi::c_double = lon1
        + atan2(sin(th) * sin(ad) * cos(lat1), cos(ad) - sin(lat1) * sin(lat2));
    let mut p: GeoPoint = GeoPoint {
        latitude: lat2 * RAD2DEG,
        longitude: norm_signed_deg(lon2 * RAD2DEG),
        altitude: a.altitude,
    };
    return p;
}
#[no_mangle]
pub unsafe extern "C" fn runway_approach_aimpoint(
    mut site: *const LandingSite,
    mut radius: ::core::ffi::c_double,
    mut distance_before_threshold: ::core::ffi::c_double,
) -> GeoPoint {
    let mut threshold: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    return destination_point(
        threshold,
        norm_deg((*site).runway_heading + 180.0f64),
        fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            distance_before_threshold,
        ),
        radius,
    );
}
#[no_mangle]
pub unsafe extern "C" fn runway_reciprocal_site(
    mut primary: *const LandingSite,
    mut radius: ::core::ffi::c_double,
) -> LandingSite {
    let mut out: LandingSite = *primary;
    let mut threshold: GeoPoint = GeoPoint {
        latitude: (*primary).latitude,
        longitude: (*primary).longitude,
        altitude: (*primary).altitude,
    };
    let mut reciprocal: GeoPoint = destination_point(
        threshold,
        (*primary).runway_heading,
        fmax(0.0f64, (*primary).runway_length),
        radius,
    );
    out.latitude = reciprocal.latitude;
    out.longitude = reciprocal.longitude;
    out.altitude = reciprocal.altitude;
    out.runway_heading = norm_deg((*primary).runway_heading + 180.0f64);
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn telemetry_reframe_runway(
    mut t: *mut Telemetry,
    mut site: *const LandingSite,
    mut radius: ::core::ffi::c_double,
) {
    if t.is_null() || site.is_null() {
        return;
    }
    let mut current: GeoPoint = GeoPoint {
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        altitude: (*t).mean_altitude,
    };
    let mut threshold: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    (*t).range_to_site = great_circle_distance(current, threshold, radius);
    (*t).bearing_to_site = initial_bearing(current, threshold);
    (*t).heading_error = norm_signed_deg((*t).bearing_to_site - (*t).heading);
    (*t).course_to_site_error = norm_signed_deg(
        (*t).bearing_to_site - (*t).ground_track_heading,
    );
    runway_coordinates(
        current,
        threshold,
        (*site).runway_heading,
        radius,
        &raw mut (*t).runway_along_track,
        &raw mut (*t).runway_cross_track,
    );
}
#[no_mangle]
pub unsafe extern "C" fn local_offsets(
    mut origin: GeoPoint,
    mut p: GeoPoint,
    mut radius: ::core::ffi::c_double,
    mut east: *mut ::core::ffi::c_double,
    mut north: *mut ::core::ffi::c_double,
) {
    let mut distance: ::core::ffi::c_double = great_circle_distance(origin, p, radius);
    let mut bearing: ::core::ffi::c_double = initial_bearing(origin, p) * DEG2RAD;
    *east = distance * sin(bearing);
    *north = distance * cos(bearing);
}
#[no_mangle]
pub unsafe extern "C" fn local_point(
    mut origin: GeoPoint,
    mut east: ::core::ffi::c_double,
    mut north: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
) -> GeoPoint {
    let mut distance: ::core::ffi::c_double = hypot(east, north);
    let mut bearing: ::core::ffi::c_double = if distance > 0.0f64 {
        norm_deg(atan2(east, north) * RAD2DEG)
    } else {
        0.0f64
    };
    let mut p: GeoPoint = destination_point(origin, bearing, distance, radius);
    p.altitude = altitude;
    return p;
}
#[no_mangle]
pub unsafe extern "C" fn surface_course(
    mut pos: Vector3,
    mut vel: Vector3,
    mut omega: Vector3,
    mut north_axis: Vector3,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut up: Vector3 = vnorm(
        pos,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut surface: Vector3 = vsub(vel, vcross(omega, pos));
    let mut horizontal: Vector3 = vproject_plane(surface, up);
    if !(vmag(horizontal) > 0.0f64) {
        return norm_deg(fallback);
    }
    let mut north: Vector3 = vnorm(
        vsub(north_axis, vscale(up, vdot(north_axis, up))),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut east: Vector3 = vnorm(
        vcross(north, up),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    if !(vmag(north) > 0.0f64) || !(vmag(east) > 0.0f64) {
        return norm_deg(fallback);
    }
    return norm_deg(atan2(vdot(horizontal, east), vdot(horizontal, north)) * RAD2DEG);
}
#[no_mangle]
pub unsafe extern "C" fn runway_coordinates(
    mut point: GeoPoint,
    mut site: GeoPoint,
    mut heading: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
    mut along: *mut ::core::ffi::c_double,
    mut cross: *mut ::core::ffi::c_double,
) {
    let mut lat1: ::core::ffi::c_double = site.latitude * DEG2RAD;
    let mut lat2: ::core::ffi::c_double = point.latitude * DEG2RAD;
    let mut dlon: ::core::ffi::c_double = norm_signed_deg(
        point.longitude - site.longitude,
    ) * DEG2RAD;
    let mut cos_delta: ::core::ffi::c_double = sin(lat1) * sin(lat2)
        + cos(lat1) * cos(lat2) * cos(dlon);
    let mut delta: ::core::ffi::c_double = acos(clampd(cos_delta, -1.0f64, 1.0f64));
    let mut bearing: ::core::ffi::c_double = atan2(
        sin(dlon) * cos(lat2),
        cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon),
    );
    let mut rel: ::core::ffi::c_double = bearing - heading * DEG2RAD;
    rel = atan2(sin(rel), cos(rel));
    if !along.is_null() {
        *along = atan2(sin(delta) * cos(rel), cos(delta)) * radius;
    }
    if !cross.is_null() {
        *cross = asin(clampd(sin(delta) * sin(rel), -1.0f64, 1.0f64)) * radius;
    }
}
#[no_mangle]
pub unsafe extern "C" fn taem_interface_line_heading(
    mut runway_along: ::core::ffi::c_double,
    mut runway_cross: ::core::ffi::c_double,
    mut runway_heading: ::core::ffi::c_double,
    mut q: *const TaemInterfaceTarget,
) -> ::core::ffi::c_double {
    if q.is_null() || !(*q).valid || runway_along.is_finite() as i32 == 0
        || runway_cross.is_finite() as i32 == 0 || runway_heading.is_finite() as i32 == 0
        || (*q).along_track.is_finite() as i32 == 0
        || (*q).cross_track.is_finite() as i32 == 0
        || (*q).course.is_finite() as i32 == 0
    {
        return if runway_heading.is_finite() as i32 != 0 {
            norm_deg(runway_heading)
        } else {
            0.0f64
        };
    }
    let mut rel: ::core::ffi::c_double = norm_signed_deg((*q).course - runway_heading)
        * DEG2RAD;
    let mut tangent_along: ::core::ffi::c_double = cos(rel);
    let mut tangent_cross: ::core::ffi::c_double = sin(rel);
    let mut normal_along: ::core::ffi::c_double = -tangent_cross;
    let mut normal_cross: ::core::ffi::c_double = tangent_along;
    let mut da: ::core::ffi::c_double = runway_along - (*q).along_track;
    let mut dc: ::core::ffi::c_double = runway_cross - (*q).cross_track;
    let mut along_error: ::core::ffi::c_double = da * tangent_along + dc * tangent_cross;
    let mut line_error: ::core::ffi::c_double = da * normal_along + dc * normal_cross;
    let mut scale: ::core::ffi::c_double = if (*q).acquisition_lead > 0.0f64 {
        (*q).acquisition_lead
    } else {
        fabs(along_error)
    };
    let mut intercept: ::core::ffi::c_double = -atan2(line_error, scale) * RAD2DEG;
    return norm_deg((*q).course + intercept);
}
#[no_mangle]
pub unsafe extern "C" fn taem_course_rate_bank_command(
    mut horizontal_speed: ::core::ffi::c_double,
    mut lift_accel: ::core::ffi::c_double,
    mut bank_effectiveness: ::core::ffi::c_double,
    mut bank_limit: ::core::ffi::c_double,
    mut current_course: ::core::ffi::c_double,
    mut desired_course: ::core::ffi::c_double,
    mut response_time: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if horizontal_speed.is_finite() as i32 == 0 || !(horizontal_speed > 0.0f64)
        || lift_accel.is_finite() as i32 == 0 || !(lift_accel > 0.0f64)
        || bank_effectiveness.is_finite() as i32 == 0 || !(bank_effectiveness > 0.0f64)
        || bank_limit.is_finite() as i32 == 0 || !(bank_limit > 0.0f64)
        || current_course.is_finite() as i32 == 0
        || desired_course.is_finite() as i32 == 0
        || response_time.is_finite() as i32 == 0 || !(response_time > 0.0f64)
    {
        return 0.0f64;
    }
    let mut limit: ::core::ffi::c_double = fmin(
        fabs(bank_limit),
        nextafter(90.0f64, 0.0f64),
    );
    let mut effective_limit: ::core::ffi::c_double = fmin(
        limit * bank_effectiveness,
        nextafter(90.0f64, 0.0f64),
    );
    let mut max_lateral: ::core::ffi::c_double = lift_accel
        * sin(effective_limit * DEG2RAD);
    if !(max_lateral > 0.0f64) {
        return 0.0f64;
    }
    let mut max_course_rate: ::core::ffi::c_double = max_lateral / horizontal_speed;
    let mut error: ::core::ffi::c_double = norm_signed_deg(
        desired_course - current_course,
    ) * DEG2RAD;
    let mut requested_rate: ::core::ffi::c_double = error / response_time;
    let mut course_rate: ::core::ffi::c_double = clampd(
        requested_rate,
        -max_course_rate,
        max_course_rate,
    );
    let mut lateral: ::core::ffi::c_double = horizontal_speed * course_rate;
    let mut ratio: ::core::ffi::c_double = clampd(
        lateral / lift_accel,
        -sin(effective_limit * DEG2RAD),
        sin(effective_limit * DEG2RAD),
    );
    let mut bank: ::core::ffi::c_double = asin(ratio) / bank_effectiveness * RAD2DEG;
    return clampd(bank, -limit, limit);
}
#[no_mangle]
pub unsafe extern "C" fn bank_for_point_capture(
    mut speed: ::core::ffi::c_double,
    mut lift_accel: ::core::ffi::c_double,
    mut bank_effectiveness: ::core::ffi::c_double,
    mut course_error: ::core::ffi::c_double,
    mut distance: ::core::ffi::c_double,
    mut maximum_bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(speed > 0.0f64) || !(lift_accel > 0.0f64) || !(distance > 0.0f64)
        || !(maximum_bank > 0.0f64) || !(bank_effectiveness > 0.0f64)
    {
        return 0.0f64;
    }
    let mut eff: ::core::ffi::c_double = bank_effectiveness;
    let mut limit: ::core::ffi::c_double = fmin(
        maximum_bank,
        nextafter(90.0f64, 0.0f64),
    );
    let mut curvature: ::core::ffi::c_double = 2.0f64
        * sin(norm_signed_deg(course_error) * DEG2RAD) / distance;
    let mut lateral: ::core::ffi::c_double = speed * speed * curvature;
    let mut effective_limit: ::core::ffi::c_double = fmin(
        limit * eff,
        nextafter(90.0f64, 0.0f64),
    ) * DEG2RAD;
    let mut ratio: ::core::ffi::c_double = clampd(
        lateral / lift_accel,
        -sin(effective_limit),
        sin(effective_limit),
    );
    return clampd(asin(ratio) / eff * RAD2DEG, -limit, limit);
}
#[no_mangle]
pub unsafe extern "C" fn robust_pid_init(
    mut p: *mut RobustPID,
    mut kp: ::core::ffi::c_double,
    mut ki: ::core::ffi::c_double,
    mut kd: ::core::ffi::c_double,
    mut limit: ::core::ffi::c_double,
    mut tc: ::core::ffi::c_double,
) {
    memset(
        p as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<RobustPID>() as size_t,
    );
    (*p).kp = kp;
    (*p).ki = ki;
    (*p).kd = kd;
    (*p).integral_limit = fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, limit);
    (*p).derivative_tc = fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, tc);
}
#[no_mangle]
pub unsafe extern "C" fn robust_pid_reset(mut p: *mut RobustPID) {
    (*p).integral = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*p).previous_error = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*p).filtered_derivative = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*p).has_previous = false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn robust_pid_update(
    mut p: *mut RobustPID,
    mut error: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut lo: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(dt > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || error.is_finite() as i32 == 0
    {
        return clampd((*p).kp * error, lo, hi);
    }
    let mut raw: ::core::ffi::c_double = if (*p).has_previous as ::core::ffi::c_int != 0
    {
        (error - (*p).previous_error) / dt
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut alpha: ::core::ffi::c_double = if (*p).derivative_tc
        <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    } else {
        dt / (dt + (*p).derivative_tc)
    };
    (*p).filtered_derivative += alpha * (raw - (*p).filtered_derivative);
    (*p).previous_error = error;
    (*p).has_previous = true_0 != 0;
    let mut candidate: ::core::ffi::c_double = clampd(
        (*p).integral + error * dt,
        -(*p).integral_limit,
        (*p).integral_limit,
    );
    let mut pd: ::core::ffi::c_double = (*p).kp * error
        + (*p).kd * (*p).filtered_derivative;
    let mut out: ::core::ffi::c_double = pd + (*p).ki * candidate;
    let mut limited: ::core::ffi::c_double = clampd(out, lo, hi);
    if out == limited {
        (*p).integral = candidate;
    } else if out > hi {
        if error < 0 as ::core::ffi::c_int as ::core::ffi::c_double {
            (*p).integral = candidate;
        } else if (*p).ki != 0 as ::core::ffi::c_int as ::core::ffi::c_double && pd < hi
        {
            (*p).integral = clampd(
                (hi - pd) / (*p).ki,
                -(*p).integral_limit,
                (*p).integral_limit,
            );
        }
    } else if error > 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        (*p).integral = candidate;
    } else if (*p).ki != 0 as ::core::ffi::c_int as ::core::ffi::c_double && pd > lo {
        (*p).integral = clampd(
            (lo - pd) / (*p).ki,
            -(*p).integral_limit,
            (*p).integral_limit,
        );
    }
    return clampd(pd + (*p).ki * (*p).integral, lo, hi);
}
#[no_mangle]
pub unsafe extern "C" fn lowpass_update(
    mut f: *mut LowPass,
    mut value: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(*f).has_value {
        (*f).value = value;
        (*f).has_value = true_0 != 0;
        return value;
    }
    if value.is_finite() as i32 == 0 || dt.is_finite() as i32 == 0 || !(dt > 0.0f64) {
        return (*f).value;
    }
    if (*f).tc.is_finite() as i32 == 0 {
        return (*f).value;
    }
    if !((*f).tc > 0.0f64) {
        (*f).value = value;
        return value;
    }
    let mut alpha: ::core::ffi::c_double = dt / (dt + (*f).tc);
    (*f).value += alpha * (value - (*f).value);
    return (*f).value;
}
#[no_mangle]
pub unsafe extern "C" fn lowpass_reset(mut f: *mut LowPass) {
    (*f).has_value = false_0 != 0;
    (*f).value = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
}
#[no_mangle]
pub unsafe extern "C" fn jerk_update(
    mut j: *mut JerkLimiter,
    mut target: ::core::ffi::c_double,
    mut max_rate: ::core::ffi::c_double,
    mut max_accel: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if j.is_null() {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if target.is_finite() as i32 == 0 || dt.is_finite() as i32 == 0 || !(dt > 0.0f64) {
        return if (*j).has_value as ::core::ffi::c_int != 0 {
            (*j).value
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
    }
    if !(*j).has_value {
        (*j).has_value = true_0 != 0;
        (*j).value = target;
        (*j).rate = 0.0f64;
        return target;
    }
    if max_rate.is_finite() as i32 == 0 || max_accel.is_finite() as i32 == 0 {
        return (*j).value;
    }
    if !(max_rate > 0.0f64) || !(max_accel > 0.0f64) {
        (*j).rate = 0.0f64;
        return (*j).value;
    }
    let mut error: ::core::ffi::c_double = target - (*j).value;
    let mut stopping: ::core::ffi::c_double = sqrt(2.0f64 * max_accel * fabs(error));
    let mut desired: ::core::ffi::c_double = copysign(
        fmin(max_rate, fmin(stopping, fabs(error) / dt)),
        error,
    );
    (*j).rate += clampd(desired - (*j).rate, -max_accel * dt, max_accel * dt);
    (*j).rate = clampd((*j).rate, -max_rate, max_rate);
    (*j).value += (*j).rate * dt;
    return (*j).value;
}
#[no_mangle]
pub unsafe extern "C" fn jerk_angle_update(
    mut j: *mut JerkLimiter,
    mut target: ::core::ffi::c_double,
    mut max_rate: ::core::ffi::c_double,
    mut max_accel: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut norm: ::core::ffi::c_double = norm_deg(target);
    if !(*j).has_value {
        (*j).has_value = true_0 != 0;
        (*j).value = norm;
        (*j).rate = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        return norm;
    }
    let mut error: ::core::ffi::c_double = norm_signed_deg(norm - (*j).value);
    let mut next: ::core::ffi::c_double = jerk_update(
        j,
        (*j).value + error,
        max_rate,
        max_accel,
        dt,
    );
    (*j).value = norm_deg(next);
    return (*j).value;
}
#[no_mangle]
pub unsafe extern "C" fn burn_fraction(
    mut elapsed: ::core::ffi::c_double,
    mut remaining: ::core::ffi::c_double,
    mut max_accel: ::core::ffi::c_double,
    mut ramp: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if remaining <= GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    ramp = fmax(0.5f64, ramp);
    let mut x: ::core::ffi::c_double = clampd(
        elapsed / ramp,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut smooth: ::core::ffi::c_double = x * x
        * (3 as ::core::ffi::c_int as ::core::ffi::c_double
            - 2 as ::core::ffi::c_int as ::core::ffi::c_double * x);
    let mut tdv: ::core::ffi::c_double = fmax(
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
        max_accel * 1.25f64,
    );
    let mut taper: ::core::ffi::c_double = if remaining >= tdv {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    } else {
        fmax(
            0.18f64,
            sqrt(fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, remaining) / tdv),
        )
    };
    return clampd(
        fmin(smooth, taper),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
}
#[no_mangle]
pub unsafe extern "C" fn burn_estimated_duration(
    mut dv: ::core::ffi::c_double,
    mut max_accel: ::core::ffi::c_double,
    mut ramp: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if dv <= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut acc: ::core::ffi::c_double = fmax(0.05f64, max_accel);
    let mut remaining: ::core::ffi::c_double = dv;
    let mut elapsed: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut step: ::core::ffi::c_double = 0.025f64;
    while remaining > GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS
        && elapsed < 300 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        remaining = fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            remaining - acc * burn_fraction(elapsed, remaining, acc, ramp) * step,
        );
        elapsed += step;
    }
    return elapsed;
}
#[no_mangle]
pub unsafe extern "C" fn flight_control_regime(
    mut mach: ::core::ffi::c_double,
    mut true_air_speed: ::core::ffi::c_double,
    mut dynamic_pressure: ::core::ffi::c_double,
    mut minimum_safe_speed: ::core::ffi::c_double,
) -> FlightControlRegime {
    let mut speed_ratio: ::core::ffi::c_double = true_air_speed
        / fmax(minimum_safe_speed, 1.0f64);
    let mut x: ::core::ffi::c_double = clampd(
        (speed_ratio - 0.95f64) / 1.05f64,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut low_speed: ::core::ffi::c_double = 0.45f64
        + 0.55f64 * x * x
            * (3 as ::core::ffi::c_int as ::core::ffi::c_double
                - 2 as ::core::ffi::c_int as ::core::ffi::c_double * x);
    let mut r: FlightControlRegime = FlightControlRegime {
        command_rate_scale: 0.,
        command_accel_scale: 0.,
        response_scale: 0.,
    };
    r.command_rate_scale = low_speed;
    r.command_accel_scale = sqrt(low_speed);
    r.response_scale = low_speed;
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn speedbrake_controller_reset(
    mut c: *mut SpeedbrakeController,
    mut deployed: bool,
) {
    if c.is_null() {
        return;
    }
    (*c).integral = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*c).deployed = deployed;
    (*c).initialized = true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn speedbrake_controller_update(
    mut c: *mut SpeedbrakeController,
    mut dynamic_pressure: ::core::ffi::c_double,
    mut target_dynamic_pressure: ::core::ffi::c_double,
    mut specific_energy_error: ::core::ffi::c_double,
    mut energy_scale: ::core::ffi::c_double,
    mut maximum_dynamic_pressure: ::core::ffi::c_double,
    mut inhibit: bool,
    mut dt: ::core::ffi::c_double,
) -> bool {
    if c.is_null() {
        return false_0 != 0;
    }
    if !(*c).initialized {
        speedbrake_controller_reset(c, false_0 != 0);
    }
    if inhibit as ::core::ffi::c_int != 0 || dynamic_pressure.is_finite() as i32 == 0
        || target_dynamic_pressure.is_finite() as i32 == 0
        || target_dynamic_pressure <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        || specific_energy_error.is_finite() as i32 == 0
        || energy_scale.is_finite() as i32 == 0
        || energy_scale <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        (*c).integral
            *= exp(-fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, dt) / 2.0f64);
        (*c).deployed = false_0 != 0;
        return false_0 != 0;
    }
    let mut qerr: ::core::ffi::c_double = (dynamic_pressure - target_dynamic_pressure)
        / fmax(target_dynamic_pressure, 1.0f64);
    let mut eerr: ::core::ffi::c_double = specific_energy_error
        / fmax(energy_scale, 1.0f64);
    qerr = clampd(qerr, -2.0f64, 2.0f64);
    eerr = clampd(eerr, -2.0f64, 2.0f64);
    if dt > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && dt.is_finite() as i32 != 0
    {
        (*c).integral += clampd(qerr, -1.0f64, 1.0f64) * dt / 6.0f64;
        (*c).integral = clampd((*c).integral, -1.2f64, 1.2f64);
    }
    let mut drive: ::core::ffi::c_double = qerr + 0.38f64 * eerr
        + 0.22f64 * (*c).integral;
    if maximum_dynamic_pressure > 0.0f64 && dynamic_pressure >= maximum_dynamic_pressure
    {
        (*c).deployed = false_0 != 0;
        return false_0 != 0;
    }
    if (*c).deployed {
        if drive < -0.08f64 {
            (*c).deployed = false_0 != 0;
        }
    } else if drive > 0.08f64 {
        (*c).deployed = true_0 != 0;
    }
    return (*c).deployed;
}
#[no_mangle]
pub unsafe extern "C" fn planet_rotation_vector(mut p: *const PlanetModel) -> Vector3 {
    return vscale((*p).north_axis, (*p).rotational_speed);
}
#[no_mangle]
pub unsafe extern "C" fn planet_surface_gravity(
    mut p: *const PlanetModel,
) -> ::core::ffi::c_double {
    if p.is_null() || (*p).radius.is_finite() as i32 == 0 || !((*p).radius > 0.0f64)
        || (*p).gravitational_parameter.is_finite() as i32 == 0
        || !((*p).gravitational_parameter > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return (*p).gravitational_parameter / ((*p).radius * (*p).radius);
}
unsafe extern "C" fn atmosphere_curve_sample(
    mut p: *const PlanetModel,
    mut values: *const ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
    mut logarithmic: bool,
) -> ::core::ffi::c_double {
    if p.is_null() || values.is_null() || (*p).atmosphere_sample_count < 2 as size_t {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut count: size_t = (*p).atmosphere_sample_count;
    if count > LANDER_ATMOSPHERE_SAMPLE_MAX as size_t {
        count = LANDER_ATMOSPHERE_SAMPLE_MAX as size_t;
    }
    let mut h: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        altitude,
    );
    if h <= (*p).atmosphere_altitude[0 as ::core::ffi::c_int as usize] {
        return fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            *values.offset(0 as ::core::ffi::c_int as isize),
        );
    }
    if h >= (*p).atmosphere_altitude[count.wrapping_sub(1 as size_t) as usize] {
        return fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            *values.offset(count.wrapping_sub(1 as size_t) as isize),
        );
    }
    let mut lo: size_t = 0 as size_t;
    let mut hi: size_t = count.wrapping_sub(1 as size_t);
    while hi.wrapping_sub(lo) > 1 as size_t {
        let mut mid: size_t = lo
            .wrapping_add(hi.wrapping_sub(lo).wrapping_div(2 as size_t));
        if h <= (*p).atmosphere_altitude[mid as usize] {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let mut a: ::core::ffi::c_double = (*p).atmosphere_altitude[lo as usize];
    let mut b: ::core::ffi::c_double = (*p).atmosphere_altitude[hi as usize];
    let mut altitude_span: ::core::ffi::c_double = b - a;
    if !(altitude_span > 0.0f64) || altitude_span.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut f: ::core::ffi::c_double = clampd((h - a) / altitude_span, 0.0f64, 1.0f64);
    let mut av: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        *values.offset(lo as isize),
    );
    let mut bv: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        *values.offset(hi as isize),
    );
    if logarithmic as ::core::ffi::c_int != 0
        && av > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && bv > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return exp(log(av) + f * (log(bv) - log(av)));
    }
    return av + (bv - av) * f;
}
#[no_mangle]
pub unsafe extern "C" fn planet_atmospheric_density(
    mut p: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if p.is_null() || altitude.is_finite() as i32 == 0
        || (*p).atmosphere_depth.is_finite() as i32 == 0
        || (*p).atmosphere_depth < 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if altitude >= (*p).atmosphere_depth {
        return 0.0f64;
    }
    if (*p).atmosphere_sample_count < 2 as size_t {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return atmosphere_curve_sample(
        p,
        &raw const (*p).atmosphere_density as *const ::core::ffi::c_double,
        fmax(0.0f64, altitude),
        true_0 != 0,
    );
}
#[no_mangle]
pub unsafe extern "C" fn planet_atmospheric_pressure(
    mut p: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if p.is_null() || altitude.is_finite() as i32 == 0
        || (*p).atmosphere_depth.is_finite() as i32 == 0
        || (*p).atmosphere_depth < 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if altitude >= (*p).atmosphere_depth {
        return 0.0f64;
    }
    if (*p).atmosphere_sample_count < 2 as size_t {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return atmosphere_curve_sample(
        p,
        &raw const (*p).atmosphere_pressure as *const ::core::ffi::c_double,
        fmax(0.0f64, altitude),
        true_0 != 0,
    );
}
#[no_mangle]
pub unsafe extern "C" fn planet_atmospheric_speed_of_sound(
    mut p: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if p.is_null() || altitude.is_finite() as i32 == 0
        || (*p).atmosphere_depth.is_finite() as i32 == 0
        || (*p).atmosphere_depth < 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if altitude >= (*p).atmosphere_depth {
        return 0.0f64;
    }
    let mut pressure: ::core::ffi::c_double = planet_atmospheric_pressure(p, altitude);
    let mut density: ::core::ffi::c_double = planet_atmospheric_density(p, altitude);
    let mut gamma: ::core::ffi::c_double = (*p).atmosphere_adiabatic_index;
    if !(pressure > 0.0f64) || !(density > 0.0f64) || gamma.is_finite() as i32 == 0
        || !(gamma > 1.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return sqrt(gamma * pressure / density);
}
#[no_mangle]
pub unsafe extern "C" fn hac_guidance_compute_radius(
    mut current: GeoPoint,
    mut true_air_speed: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut gravity: ::core::ffi::c_double,
    mut hac_radius: ::core::ffi::c_double,
) -> HACGuidance {
    hac_radius = fmax(1000 as ::core::ffi::c_int as ::core::ffi::c_double, hac_radius);
    let mut h: ::core::ffi::c_double = (*site).runway_heading * DEG2RAD;
    let mut ae: ::core::ffi::c_double = sin(h);
    let mut an: ::core::ffi::c_double = cos(h);
    let mut re: ::core::ffi::c_double = cos(h);
    let mut rn: ::core::ffi::c_double = -sin(h);
    let mut fe: ::core::ffi::c_double = -ae * (*s).final_approach_distance;
    let mut fn_0: ::core::ffi::c_double = -an * (*s).final_approach_distance;
    let mut ce: ::core::ffi::c_double = fe + re * side * hac_radius;
    let mut cn: ::core::ffi::c_double = fn_0 + rn * side * hac_radius;
    let mut sp: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut e: ::core::ffi::c_double = 0.;
    let mut n: ::core::ffi::c_double = 0.;
    local_offsets(sp, current, radius, &raw mut e, &raw mut n);
    let mut ca: ::core::ffi::c_double = atan2(n - cn, e - ce);
    let mut fa: ::core::ffi::c_double = atan2(fn_0 - cn, fe - ce);
    let mut orient: ::core::ffi::c_double = -side;
    let mut configured: ::core::ffi::c_double = (*s).hac_look_ahead_angle * DEG2RAD;
    let mut speed: ::core::ffi::c_double = clampd(
        true_air_speed * 7 as ::core::ffi::c_int as ::core::ffi::c_double
            / fmax(hac_radius, 1 as ::core::ffi::c_int as ::core::ffi::c_double),
        6 as ::core::ffi::c_int as ::core::ffi::c_double * DEG2RAD,
        34 as ::core::ffi::c_int as ::core::ffi::c_double * DEG2RAD,
    );
    let mut look: ::core::ffi::c_double = fmax(configured * 0.65f64, speed);
    let mut ta: ::core::ffi::c_double = ca + orient * look;
    let mut te: ::core::ffi::c_double = ce + cos(ta) * hac_radius;
    let mut tn: ::core::ffi::c_double = cn + sin(ta) * hac_radius;
    let mut target: ::core::ffi::c_double = norm_deg(atan2(te - e, tn - n) * RAD2DEG);
    let mut raw: ::core::ffi::c_double = if orient
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        fmod(
            fa - ca + 2 as ::core::ffi::c_int as ::core::ffi::c_double * LANDER_PI,
            2 as ::core::ffi::c_int as ::core::ffi::c_double * LANDER_PI,
        )
    } else {
        fmod(
            ca - fa + 2 as ::core::ffi::c_int as ::core::ffi::c_double * LANDER_PI,
            2 as ::core::ffi::c_int as ::core::ffi::c_double * LANDER_PI,
        )
    };
    let mut arc: ::core::ffi::c_double = raw * hac_radius;
    let mut actual: ::core::ffi::c_double = hypot(e - ce, n - cn);
    let mut radial: ::core::ffi::c_double = actual - hac_radius;
    let mut capture: ::core::ffi::c_double = fmin(fabs(radial), hac_radius * 1.5f64);
    let mut final_alt: ::core::ffi::c_double = (*s).final_approach_distance
        * tan((*s).final_glide_slope * DEG2RAD);
    let mut desired: ::core::ffi::c_double = (*site).altitude + final_alt
        + (arc + capture * 0.65f64) * tan((*s).taem_glide_slope * DEG2RAD);
    let mut eta: ::core::ffi::c_double = norm_signed_deg(target - course) * DEG2RAD;
    let mut chord: ::core::ffi::c_double = fmax(hypot(te - e, tn - n), 1.0f64);
    let mut lateral: ::core::ffi::c_double = 2 as ::core::ffi::c_int
        as ::core::ffi::c_double * true_air_speed * true_air_speed / chord
        * sin(clampd(eta, -LANDER_PI * 0.5f64, LANDER_PI * 0.5f64));
    let mut tangent: ::core::ffi::c_double = norm_deg(
        atan2(-orient * sin(ca), orient * cos(ca)) * RAD2DEG,
    );
    let mut err: ::core::ffi::c_double = norm_signed_deg(tangent - course);
    let mut g: HACGuidance = HACGuidance {
        heading: target,
        bank: clampd(
            atan2(lateral, fmax(gravity, 0.01f64)) * RAD2DEG,
            -(70 as ::core::ffi::c_int) as ::core::ffi::c_double,
            70 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
        desired_altitude: desired,
        distance_final: hypot(e - fe, n - fn_0),
        arc_remaining: arc,
        radial_error: radial,
        course_error: err,
        angle: ca,
        lateral_acceleration: lateral,
    };
    return g;
}
#[no_mangle]
pub unsafe extern "C" fn hac_guidance_compute(
    mut current: GeoPoint,
    mut true_air_speed: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut gravity: ::core::ffi::c_double,
) -> HACGuidance {
    return hac_guidance_compute_radius(
        current,
        true_air_speed,
        course,
        site,
        s,
        radius,
        side,
        gravity,
        (*s).hac_radius,
    );
}
#[no_mangle]
pub unsafe extern "C" fn hac_guidance_score_radius(
    mut current: GeoPoint,
    mut true_air_speed: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut gravity: ::core::ffi::c_double,
    mut hac_radius: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut g: HACGuidance = hac_guidance_compute_radius(
        current,
        true_air_speed,
        course,
        site,
        s,
        radius,
        side,
        gravity,
        hac_radius,
    );
    return fabs(g.radial_error)
        / fmax(hac_radius, 1 as ::core::ffi::c_int as ::core::ffi::c_double) * 2.4f64
        + fabs(g.course_error) / 45 as ::core::ffi::c_int as ::core::ffi::c_double
            * 1.4f64
        + g.arc_remaining
            / fmax(
                hac_radius * 300 as ::core::ffi::c_int as ::core::ffi::c_double
                    * DEG2RAD,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ) * 0.45f64
        + fmin(
            g.distance_final
                / fmax(
                    hac_radius * 3 as ::core::ffi::c_int as ::core::ffi::c_double,
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            2 as ::core::ffi::c_int as ::core::ffi::c_double,
        ) * 0.15f64;
}
#[no_mangle]
pub unsafe extern "C" fn hac_entry_capture_geometry_ready(
    mut g: *const HACGuidance,
    mut hac_radius: ::core::ffi::c_double,
) -> bool {
    if g.is_null() || hac_radius.is_finite() as i32 == 0
        || hac_radius <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*g).radial_error.is_finite() as i32 == 0
        || (*g).course_error.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut radial_limit: ::core::ffi::c_double = fmax(
        500.0f64,
        fmin(5000.0f64, hac_radius * 0.30f64),
    );
    return fabs((*g).radial_error) <= radial_limit && fabs((*g).course_error) <= 45.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn hac_guidance_score(
    mut current: GeoPoint,
    mut true_air_speed: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut gravity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return hac_guidance_score_radius(
        current,
        true_air_speed,
        course,
        site,
        s,
        radius,
        side,
        gravity,
        (*s).hac_radius,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_alignment_maneuver_geometry(
    mut altitude: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
    mut minimum_turn_radius: ::core::ffi::c_double,
    mut alignment_turn_deg: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut out_remaining: *mut ::core::ffi::c_double,
    mut out_slope: *mut ::core::ffi::c_double,
    mut out_final_altitude: *mut ::core::ffi::c_double,
) -> bool {
    if site.is_null() || s.is_null() || altitude.is_finite() as i32 == 0
        || radius.is_finite() as i32 == 0 || minimum_turn_radius.is_finite() as i32 == 0
        || alignment_turn_deg.is_finite() as i32 == 0 || !(radius > 0.0f64)
        || !(minimum_turn_radius > 0.0f64) || minimum_turn_radius > radius
    {
        return false_0 != 0;
    }
    let mut turn_angle: ::core::ffi::c_double = fabs(alignment_turn_deg) * DEG2RAD;
    let mut path: ::core::ffi::c_double = turn_angle * radius
        + (*s).final_approach_distance;
    if !(path > 0.0f64) || path.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut final_alt: ::core::ffi::c_double = (*site).altitude
        + (*s).final_approach_distance * tan((*s).final_glide_slope * DEG2RAD);
    let mut height: ::core::ffi::c_double = altitude - final_alt;
    if !(height > 0.0f64) || height.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut slope: ::core::ffi::c_double = atan2(height, path) * RAD2DEG;
    if !(slope > 0.0f64) || !(slope < 90.0f64) {
        return false_0 != 0;
    }
    if !out_remaining.is_null() {
        *out_remaining = path;
    }
    if !out_slope.is_null() {
        *out_slope = slope;
    }
    if !out_final_altitude.is_null() {
        *out_final_altitude = final_alt;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_alignment_maneuver_feasible(
    mut altitude: ::core::ffi::c_double,
    mut speed: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
    mut minimum_turn_radius: ::core::ffi::c_double,
    mut drag: ::core::ffi::c_double,
    mut alignment_turn_deg: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut out_remaining: *mut ::core::ffi::c_double,
    mut out_slope: *mut ::core::ffi::c_double,
) -> bool {
    if p.is_null() || site.is_null() || v.is_null() || s.is_null()
        || speed.is_finite() as i32 == 0 || drag.is_finite() as i32 == 0 || drag < 0.0f64
        || speed < (*v).minimum_safe_speed
    {
        return false_0 != 0;
    }
    let mut path: ::core::ffi::c_double = 0.0f64;
    let mut slope: ::core::ffi::c_double = 0.0f64;
    let mut final_alt: ::core::ffi::c_double = 0.0f64;
    if !taem_alignment_maneuver_geometry(
        altitude,
        radius,
        minimum_turn_radius,
        alignment_turn_deg,
        site,
        s,
        &raw mut path,
        &raw mut slope,
        &raw mut final_alt,
    ) {
        return false_0 != 0;
    }
    let mut target: ::core::ffi::c_double = fmax(
        (*v).final_approach_speed,
        (*v).minimum_safe_speed,
    );
    let mut energy: ::core::ffi::c_double = entry_remaining_specific_energy(
        (*site).latitude,
        altitude,
        speed,
        (*site).latitude,
        final_alt,
        target,
        p,
    );
    let mut height: ::core::ffi::c_double = altitude - final_alt;
    let mut air_path: ::core::ffi::c_double = hypot(path, height);
    let mut loss: ::core::ffi::c_double = drag * air_path;
    if energy.is_finite() as i32 == 0 || loss.is_finite() as i32 == 0 || energy < loss {
        return false_0 != 0;
    }
    if !out_remaining.is_null() {
        *out_remaining = path;
    }
    if !out_slope.is_null() {
        *out_slope = slope;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_approach_valid(
    mut current: GeoPoint,
    mut range_to_site: ::core::ffi::c_double,
    mut runway_along: ::core::ffi::c_double,
    mut runway_cross: ::core::ffi::c_double,
    mut course: ::core::ffi::c_double,
    mut true_air_speed: ::core::ffi::c_double,
    mut horizontal_speed: ::core::ffi::c_double,
    mut vertical_speed: ::core::ffi::c_double,
    mut flight_path_angle: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> bool {
    let mut distance: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        -runway_along,
    );
    let mut desired: ::core::ffi::c_double = (*site).altitude
        + distance * tan((*s).final_glide_slope * DEG2RAD);
    let mut sink: ::core::ffi::c_double = -horizontal_speed
        * tan((*s).final_glide_slope * DEG2RAD);
    let mut altitude_tolerance: ::core::ffi::c_double = fmax(
        180.0f64,
        distance * 0.08f64,
    );
    let mut sink_tolerance: ::core::ffi::c_double = fmax(15.0f64, fabs(sink) * 0.35f64);
    let mut speed_ceiling: ::core::ffi::c_double = fmax(
        (*v).final_approach_speed * 2.0f64,
        (*v).minimum_safe_speed * 2.4f64,
    );
    let mut shallow_limit: ::core::ffi::c_double = -fmax(
        3.0f64,
        (*s).final_glide_slope - 8.0f64,
    );
    return range_to_site.is_finite() as i32 != 0
        && range_to_site < (*s).final_approach_distance * 1.35f64
        && runway_along >= -(*s).final_approach_distance * 1.22f64
        && runway_along < (*site).runway_length
        && fabs(runway_cross)
            < fmax(
                (*site).runway_width * 0.5f64,
                fmin(
                    400 as ::core::ffi::c_int as ::core::ffi::c_double,
                    distance * 0.07f64,
                ),
            )
        && fabs(norm_signed_deg((*site).runway_heading - course))
            < 12 as ::core::ffi::c_int as ::core::ffi::c_double
        && true_air_speed >= (*v).minimum_safe_speed && true_air_speed <= speed_ceiling
        && fabs(current.altitude - desired) < altitude_tolerance
        && flight_path_angle > -((*s).final_glide_slope + 8.0f64)
        && flight_path_angle < shallow_limit
        && fabs(vertical_speed - sink) < sink_tolerance;
}
#[no_mangle]
pub unsafe extern "C" fn entry_guidance_start_altitude(
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if p.is_null() || s.is_null() {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    return fmax(
        (*s).taem_interface_altitude + 1000.0f64,
        (*p).atmosphere_depth - (*s).entry_interface_altitude_margin - 500.0f64,
    );
}
#[no_mangle]
pub unsafe extern "C" fn entry_taem_handoff_altitude_bounds(
    mut s: *const GuidanceSettings,
    mut minimum: *mut ::core::ffi::c_double,
    mut maximum: *mut ::core::ffi::c_double,
) {
    let mut lo: ::core::ffi::c_double = if !s.is_null()
        && (*s).mm305_min_altitude.is_finite() as i32 != 0
        && (*s).mm305_min_altitude > 0.0f64
    {
        (*s).mm305_min_altitude
    } else {
        15000.0f64
    };
    let mut hi: ::core::ffi::c_double = if !s.is_null()
        && (*s).mm305_max_altitude.is_finite() as i32 != 0
        && (*s).mm305_max_altitude >= lo
    {
        (*s).mm305_max_altitude
    } else {
        25000.0f64
    };
    if !minimum.is_null() {
        *minimum = lo;
    }
    if !maximum.is_null() {
        *maximum = hi;
    }
}
#[no_mangle]
pub unsafe extern "C" fn entry_taem_handoff_geometry_ready(
    mut altitude: ::core::ffi::c_double,
    mut runway_along_track: ::core::ffi::c_double,
    mut vertical_speed: ::core::ffi::c_double,
    mut horizontal_speed: ::core::ffi::c_double,
    mut s: *const GuidanceSettings,
) -> bool {
    if s.is_null() || altitude.is_finite() as i32 == 0
        || runway_along_track.is_finite() as i32 == 0
        || vertical_speed.is_finite() as i32 == 0
        || horizontal_speed.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut minimum: ::core::ffi::c_double = 0.0f64;
    let mut maximum: ::core::ffi::c_double = 0.0f64;
    entry_taem_handoff_altitude_bounds(s, &raw mut minimum, &raw mut maximum);
    let mut station_along: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut station_cross: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    mm304_handoff_station(s, &raw mut station_along, &raw mut station_cross);
    let mut station_lead: ::core::ffi::c_double = clampd(
        fabs(horizontal_speed) * 2.0f64,
        180.0f64,
        2400.0f64,
    );
    return altitude >= minimum && altitude <= maximum
        && runway_along_track < station_along - station_lead && vertical_speed < 0.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn entry_s_turn_bank_authority_available(
    mut dynamic_pressure: ::core::ffi::c_double,
    mut true_air_speed: ::core::ffi::c_double,
    mut stall_fraction: ::core::ffi::c_double,
    mut g_force: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
) -> bool {
    if v.is_null() || dynamic_pressure.is_finite() as i32 == 0
        || true_air_speed.is_finite() as i32 == 0
        || stall_fraction.is_finite() as i32 == 0 || g_force.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    return dynamic_pressure > DBL_MIN && true_air_speed >= (*v).minimum_safe_speed
        && dynamic_pressure <= (*v).maximum_dynamic_pressure
        && g_force <= (*v).maximum_g_load && stall_fraction < 1.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn entry_bank_authority_limit(
    mut true_air_speed: ::core::ffi::c_double,
    mut dynamic_pressure: ::core::ffi::c_double,
    mut g_force: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
    mut maximum_bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if v.is_null() || true_air_speed.is_finite() as i32 == 0
        || dynamic_pressure.is_finite() as i32 == 0 || g_force.is_finite() as i32 == 0
        || maximum_bank.is_finite() as i32 == 0
    {
        return 0.0f64;
    }
    if dynamic_pressure <= DBL_MIN || true_air_speed < (*v).minimum_safe_speed
        || dynamic_pressure > (*v).maximum_dynamic_pressure
        || g_force > (*v).maximum_g_load
    {
        return 0.0f64;
    }
    let mut bank: ::core::ffi::c_double = fmin(
        fabs(maximum_bank),
        (*v).maximum_bank_angle,
    );
    let mut authority_q: ::core::ffi::c_double = fmax(
        250.0f64,
        (*v).maximum_dynamic_pressure * 0.008f64,
    );
    let mut authority_fraction: ::core::ffi::c_double = clampd(
        dynamic_pressure / authority_q,
        0.0f64,
        1.0f64,
    );
    return bank * authority_fraction * authority_fraction;
}
#[no_mangle]
pub unsafe extern "C" fn entry_taem_range_target(
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if p.is_null() || s.is_null() {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut terminal_reserve: ::core::ffi::c_double = fmax(
        (*s).final_approach_distance * 1.5f64,
        (*s).final_approach_distance + fmax(0.0f64, (*s).hac_radius),
    );
    return fmax((*s).taem_interface_range, terminal_reserve);
}
#[no_mangle]
pub unsafe extern "C" fn mm305_acquisition_ready(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> bool {
    if t.is_null() || p.is_null() || cfg.is_null() {
        return false_0 != 0;
    }
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    if (*t).mean_altitude.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).range_to_site.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0
        || (*t).g_force.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        (*t).mean_altitude,
    );
    let mut mach: ::core::ffi::c_double = if (*t).mach.is_finite() as i32 != 0
        && (*t).mach > 0.0f64
    {
        (*t).mach
    } else if sound.is_finite() as i32 != 0 && sound > DBL_MIN {
        (*t).true_air_speed / sound
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut mach_min: ::core::ffi::c_double = (*s).mm305_target_mach
        - (*s).mm305_mach_half_width;
    let mut mach_max: ::core::ffi::c_double = (*s).mm305_target_mach
        + (*s).mm305_mach_half_width;
    if (*s).hac_acquisition_mach.is_finite() as i32 != 0
        && (*s).hac_acquisition_mach > 0.0f64
    {
        mach_min = fmin(mach_min, (*s).hac_acquisition_mach);
    }
    let mut min_altitude: ::core::ffi::c_double = (*s).mm305_min_altitude;
    if (*s).hac_acquisition_altitude.is_finite() as i32 != 0
        && (*s).hac_acquisition_altitude > 0.0f64
    {
        min_altitude = fmin(min_altitude, (*s).hac_acquisition_altitude);
    }
    let mut maximum_range: ::core::ffi::c_double = entry_taem_range_target(p, s);
    let mut minimum_range: ::core::ffi::c_double = fmax(
        (*s).final_approach_distance * 1.5f64,
        (*s).final_approach_distance + fmax(0.0f64, (*s).hac_radius),
    );
    let mut closure: ::core::ffi::c_double = if (*t).course_to_site_error.is_finite()
        as i32 != 0
    {
        cos((*t).course_to_site_error * DEG2RAD)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut structural: bool = (*t).dynamic_pressure <= (*v).maximum_dynamic_pressure
        && (*t).g_force <= (*v).maximum_g_load
        && (!(*t).stall_fraction_is_measured
            || (*t).stall_fraction.is_finite() as i32 != 0
                && (*t).stall_fraction < 1.0f64);
    let mut acquisition_region: bool = maximum_range.is_finite() as i32 != 0
        && maximum_range > minimum_range && (*t).range_to_site >= minimum_range
        && (*t).range_to_site <= maximum_range && closure.is_finite() as i32 != 0
        && closure > 0.0f64;
    return (*t).mean_altitude >= min_altitude
        && (*t).mean_altitude <= (*s).mm305_max_altitude && mach.is_finite() as i32 != 0
        && mach >= mach_min && mach <= mach_max && (*t).vertical_speed < 0.0f64
        && (*t).true_air_speed >= (*v).minimum_safe_speed
        && acquisition_region as ::core::ffi::c_int != 0
        && structural as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn runway_landing_intercept_ratio(
    mut along: ::core::ffi::c_double,
    mut cross: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut settings: *const GuidanceSettings,
    mut ratio: *mut ::core::ffi::c_double,
) -> bool {
    if site.is_null() || settings.is_null() || ratio.is_null() || !(along < 0.0f64)
        || cross.is_finite() as i32 == 0 || !((*site).runway_length > 0.0f64)
        || !((*site).runway_width > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut target: ::core::ffi::c_double = clampd(
        (*settings).touchdown_aim_along,
        0.0f64,
        (*site).runway_length,
    );
    let mut distance: ::core::ffi::c_double = -along;
    let mut half_width: ::core::ffi::c_double = 0.5f64 * (*site).runway_width;
    let mut lower: ::core::ffi::c_double = fmax(
        (-half_width - cross) / distance,
        (-half_width - cross) / (distance + (*site).runway_length),
    );
    let mut upper: ::core::ffi::c_double = fmin(
        (half_width - cross) / distance,
        (half_width - cross) / (distance + (*site).runway_length),
    );
    if lower > upper {
        return false_0 != 0;
    }
    *ratio = clampd(-cross / (distance + target), lower, upper);
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn final_alignment_contract(
    mut distance: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> FinalAlignmentContract {
    let mut out: FinalAlignmentContract = FinalAlignmentContract {
        valid: 0 as ::core::ffi::c_int != 0,
        distance_m: 0.,
        altitude_m: 0.,
        speed_mps: 0.,
        slope_deg: 0.,
    };
    if site.is_null() || v.is_null() || s.is_null() || distance.is_finite() as i32 == 0
        || distance < 500.0f64
    {
        return out;
    }
    let mut base_slope: ::core::ffi::c_double = clampd(
        (*s).final_glide_slope,
        3.0f64,
        45.0f64,
    );
    let mut height: ::core::ffi::c_double = fmin(
        distance * tan(base_slope * DEG2RAD),
        4000.0f64,
    );
    out.distance_m = distance;
    out.slope_deg = clampd(atan2(height, distance) * RAD2DEG, 15.0f64, base_slope);
    out.altitude_m = (*site).altitude + distance * tan(out.slope_deg * DEG2RAD);
    out.speed_mps = fmax(
        (*s).final_alignment_speed,
        fmax((*v).minimum_safe_speed, (*v).touchdown_speed),
    ) + 0.005f64 * clampd(distance - 8000.0f64, 0.0f64, 1000.0f64);
    out.valid = out.altitude_m.is_finite() as i32 != 0
        && out.speed_mps.is_finite() as i32 != 0 && out.speed_mps > 0.0f64;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn final_alignment_distance_for_state(
    mut g: *const GuidanceMachine,
    mut altitude: ::core::ffi::c_double,
    mut speed: ::core::ffi::c_double,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if site.is_null() || v.is_null() || s.is_null() {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if !g.is_null() && (*g).terminal_final_handoff_latched as ::core::ffi::c_int != 0
        && (*g).terminal_final_handoff_distance.is_finite() as i32 != 0
        && (*g).terminal_final_handoff_distance >= 500.0f64
    {
        return (*g).terminal_final_handoff_distance;
    }
    let mut base: ::core::ffi::c_double = fmax(500.0f64, (*s).final_approach_distance);
    if (*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        return base;
    }
    if altitude.is_finite() as i32 == 0 || speed.is_finite() as i32 == 0 {
        return base;
    }
    let mut slope: ::core::ffi::c_double = (*s).final_glide_slope;
    if !(slope > 1.0f64 && slope < 60.0f64) {
        return base;
    }
    let mut nominal_height: ::core::ffi::c_double = base * tan(slope * DEG2RAD);
    let mut alignment_speed: ::core::ffi::c_double = fmax(
        (*s).final_alignment_speed,
        (*v).minimum_safe_speed,
    );
    let mut altitude_load: ::core::ffi::c_double = clampd(
        fmax(0.0f64, altitude - (*site).altitude - nominal_height) / 3000.0f64,
        0.0f64,
        1.0f64,
    );
    let mut speed_load: ::core::ffi::c_double = clampd(
        fmax(0.0f64, speed - alignment_speed) / 250.0f64,
        0.0f64,
        1.0f64,
    );
    return base
        + fmax(altitude_load, speed_load) * fmax(0.0f64, fmax(base, 9000.0f64) - base);
}
#[no_mangle]
pub unsafe extern "C" fn rotating_specific_energy(
    mut latitude: ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
    mut air_relative_speed: ::core::ffi::c_double,
    mut p: *const PlanetModel,
) -> ::core::ffi::c_double {
    if p.is_null() || latitude.is_finite() as i32 == 0 || fabs(latitude) > 90.0f64
        || altitude.is_finite() as i32 == 0 || air_relative_speed.is_finite() as i32 == 0
        || air_relative_speed < 0.0f64 || (*p).radius.is_finite() as i32 == 0
        || !((*p).radius > 0.0f64)
        || (*p).gravitational_parameter.is_finite() as i32 == 0
        || !((*p).gravitational_parameter > 0.0f64)
        || (*p).rotational_speed.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut radius: ::core::ffi::c_double = (*p).radius + altitude;
    if radius.is_finite() as i32 == 0 || !(radius > 0.0f64) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut transverse: ::core::ffi::c_double = radius * cos(latitude * DEG2RAD);
    let mut centrifugal: ::core::ffi::c_double = 0.5f64 * (*p).rotational_speed
        * (*p).rotational_speed * transverse * transverse;
    return 0.5f64 * air_relative_speed * air_relative_speed
        - (*p).gravitational_parameter / radius - centrifugal;
}
#[no_mangle]
pub unsafe extern "C" fn entry_remaining_specific_energy(
    mut latitude: ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
    mut air_relative_speed: ::core::ffi::c_double,
    mut target_latitude: ::core::ffi::c_double,
    mut target_altitude: ::core::ffi::c_double,
    mut target_speed: ::core::ffi::c_double,
    mut p: *const PlanetModel,
) -> ::core::ffi::c_double {
    let mut current: ::core::ffi::c_double = rotating_specific_energy(
        latitude,
        altitude,
        air_relative_speed,
        p,
    );
    let mut target: ::core::ffi::c_double = rotating_specific_energy(
        target_latitude,
        target_altitude,
        target_speed,
        p,
    );
    return current - target;
}
#[no_mangle]
pub unsafe extern "C" fn entry_altitude_target_for_speed(
    mut reference_altitude: ::core::ffi::c_double,
    mut reference_speed: ::core::ffi::c_double,
    mut true_air_speed: ::core::ffi::c_double,
    mut target_altitude: ::core::ffi::c_double,
    mut target_speed: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if reference_altitude.is_finite() as i32 == 0
        || reference_speed.is_finite() as i32 == 0
        || true_air_speed.is_finite() as i32 == 0
        || target_altitude.is_finite() as i32 == 0
        || target_speed.is_finite() as i32 == 0
    {
        return target_altitude;
    }
    let mut span: ::core::ffi::c_double = reference_speed - target_speed;
    if span.is_finite() as i32 == 0 || !(span > 0.0f64) {
        return target_altitude;
    }
    let mut remaining: ::core::ffi::c_double = clampd(
        (true_air_speed - target_speed) / span,
        0.0f64,
        1.0f64,
    );
    let mut smooth: ::core::ffi::c_double = remaining * remaining
        * (3.0f64 - 2.0f64 * remaining);
    return target_altitude + (reference_altitude - target_altitude) * smooth;
}
#[no_mangle]
pub unsafe extern "C" fn entry_altitude_target_for_range(
    mut reference_altitude: ::core::ffi::c_double,
    mut reference_range: ::core::ffi::c_double,
    mut range: ::core::ffi::c_double,
    mut target_altitude: ::core::ffi::c_double,
    mut target_range: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if reference_altitude.is_finite() as i32 == 0
        || reference_range.is_finite() as i32 == 0 || range.is_finite() as i32 == 0
        || target_altitude.is_finite() as i32 == 0
        || target_range.is_finite() as i32 == 0
    {
        return target_altitude;
    }
    let mut span: ::core::ffi::c_double = reference_range - target_range;
    if span.is_finite() as i32 == 0 || !(span > 0.0f64) {
        return target_altitude;
    }
    let mut remaining: ::core::ffi::c_double = clampd(
        (range - target_range) / span,
        0.0f64,
        1.0f64,
    );
    return target_altitude + (reference_altitude - target_altitude) * remaining;
}
#[no_mangle]
pub unsafe extern "C" fn entry_thermal_protection_aoa_floor(
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if v.is_null() {
        return 0.0f64;
    }
    return clampd((*v).entry_angle_of_attack, 0.0f64, (*v).maximum_angle_of_attack);
}
#[no_mangle]
pub unsafe extern "C" fn entry_low_q_protective_aoa_floor(
    mut dynamic_pressure: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    return entry_thermal_protection_aoa_floor(v);
}
#[no_mangle]
pub unsafe extern "C" fn entry_terminal_turn_aoa_floor(
    mut true_air_speed: ::core::ffi::c_double,
    mut dynamic_pressure: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    return entry_thermal_protection_aoa_floor(v);
}
#[no_mangle]
pub unsafe extern "C" fn entry_final_s_turn_aoa_ceiling(
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut nominal: ::core::ffi::c_double = if !v.is_null()
        && (*v).maximum_angle_of_attack.is_finite() as i32 != 0
    {
        (*v).maximum_angle_of_attack
    } else {
        28.0f64
    };
    return fmax(nominal, 35.0f64);
}
#[no_mangle]
pub unsafe extern "C" fn entry_terminal_demand(
    mut latitude: ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
    mut range: ::core::ffi::c_double,
    mut horizontal_speed: ::core::ffi::c_double,
    mut course_error: ::core::ffi::c_double,
    mut vertical_speed: ::core::ffi::c_double,
    mut true_air_speed: ::core::ffi::c_double,
    mut entry_reference_speed: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
) -> EntryTerminalDemand {
    let mut d: EntryTerminalDemand = EntryTerminalDemand {
        remaining: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        early: 0.,
        projected_taem_range_error: 0.,
        range_norm: 0.,
        energy_norm: 0.,
        raw_score: 0.,
    };
    if p.is_null() || v.is_null() || site.is_null() || s.is_null() {
        return d;
    }
    let mut height: ::core::ffi::c_double = fmax(
        0.0f64,
        altitude - (*s).taem_interface_altitude,
    );
    let mut sink: ::core::ffi::c_double = fmax(0.0f64, -vertical_speed);
    let mut radius: ::core::ffi::c_double = fmax(DBL_MIN, (*p).radius + altitude);
    let mut gravity: ::core::ffi::c_double = (*p).gravitational_parameter
        / (radius * radius);
    let mut curvature_accel: ::core::ffi::c_double = horizontal_speed * horizontal_speed
        / radius;
    let mut down_accel: ::core::ffi::c_double = fmax(0.0f64, gravity - curvature_accel);
    let mut time_to_taem: ::core::ffi::c_double = 0.0f64;
    if height > 0.0f64 {
        if down_accel > DBL_EPSILON {
            time_to_taem = (sqrt(sink * sink + 2.0f64 * down_accel * height) - sink)
                / down_accel;
        } else if sink > DBL_EPSILON {
            time_to_taem = height / sink;
        } else {
            time_to_taem = ::core::f32::INFINITY as ::core::ffi::c_double;
        }
    }
    let mut closing_speed: ::core::ffi::c_double = horizontal_speed
        * cos(norm_signed_deg(course_error) * DEG2RAD);
    let mut projected_remaining: ::core::ffi::c_double = if time_to_taem.is_finite()
        as i32 != 0
    {
        fmax(0.0f64, range) - closing_speed * time_to_taem
    } else {
        fmax(0.0f64, range)
    };
    d.projected_taem_range_error = projected_remaining - entry_taem_range_target(p, s);
    return d;
}
unsafe extern "C" fn stock_ksp_curve_eval(
    mut keys: *const StockKspCurveKey,
    mut count: size_t,
    mut x: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if keys.is_null() || count == 0 as size_t || x.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if x <= (*keys.offset(0 as ::core::ffi::c_int as isize)).x {
        return (*keys.offset(0 as ::core::ffi::c_int as isize)).y;
    }
    if x >= (*keys.offset(count.wrapping_sub(1 as size_t) as isize)).x {
        return (*keys.offset(count.wrapping_sub(1 as size_t) as isize)).y;
    }
    let mut i: size_t = 0 as size_t;
    while i.wrapping_add(1 as size_t) < count {
        let mut a: *const StockKspCurveKey = keys.offset(i as isize)
            as *const StockKspCurveKey;
        let mut b: *const StockKspCurveKey = keys
            .offset(i.wrapping_add(1 as size_t) as isize) as *const StockKspCurveKey;
        if x > (*b).x {
            i = i.wrapping_add(1);
        } else {
            let mut h: ::core::ffi::c_double = (*b).x - (*a).x;
            if !(h > DBL_MIN) {
                return (*a).y;
            }
            let mut u: ::core::ffi::c_double = clampd((x - (*a).x) / h, 0.0f64, 1.0f64);
            let mut u2: ::core::ffi::c_double = u * u;
            let mut u3: ::core::ffi::c_double = u2 * u;
            let mut h00: ::core::ffi::c_double = 2.0f64 * u3 - 3.0f64 * u2 + 1.0f64;
            let mut h10: ::core::ffi::c_double = u3 - 2.0f64 * u2 + u;
            let mut h01: ::core::ffi::c_double = -2.0f64 * u3 + 3.0f64 * u2;
            let mut h11: ::core::ffi::c_double = u3 - u2;
            return h00 * (*a).y + h10 * h * (*a).out_tangent + h01 * (*b).y
                + h11 * h * (*b).in_tangent;
        }
    }
    return (*keys.offset(count.wrapping_sub(1 as size_t) as isize)).y;
}
unsafe extern "C" fn stock_ksp_lifting_surface_coefficients(
    mut mach: ::core::ffi::c_double,
    mut angle_of_attack: ::core::ffi::c_double,
    mut cl: *mut ::core::ffi::c_double,
    mut cd: *mut ::core::ffi::c_double,
) {
    static mut lift_curve: [StockKspCurveKey; 5] = [
        StockKspCurveKey {
            x: 0.0f64,
            y: 0.0f64,
            in_tangent: 0.0f64,
            out_tangent: 1.965926f64,
        },
        StockKspCurveKey {
            x: 0.258819f64,
            y: 0.5114774f64,
            in_tangent: 1.990092f64,
            out_tangent: 1.905806f64,
        },
        StockKspCurveKey {
            x: 0.5f64,
            y: 0.9026583f64,
            in_tangent: 0.7074468f64,
            out_tangent: -0.7074468f64,
        },
        StockKspCurveKey {
            x: 0.7071068f64,
            y: 0.5926583f64,
            in_tangent: -2.087948f64,
            out_tangent: -1.990095f64,
        },
        StockKspCurveKey {
            x: 1.0f64,
            y: 0.0f64,
            in_tangent: -2.014386f64,
            out_tangent: -2.014386f64,
        },
    ];
    static mut lift_mach: [StockKspCurveKey; 5] = [
        StockKspCurveKey {
            x: 0.0f64,
            y: 1.0f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
        StockKspCurveKey {
            x: 0.3f64,
            y: 0.5f64,
            in_tangent: -1.671345f64,
            out_tangent: -0.8273422f64,
        },
        StockKspCurveKey {
            x: 1.0f64,
            y: 0.125f64,
            in_tangent: -0.0005291355f64,
            out_tangent: -0.02625772f64,
        },
        StockKspCurveKey {
            x: 5.0f64,
            y: 0.0625f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
        StockKspCurveKey {
            x: 25.0f64,
            y: 0.05f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
    ];
    static mut drag_curve: [StockKspCurveKey; 5] = [
        StockKspCurveKey {
            x: 0.0f64,
            y: 0.01f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
        StockKspCurveKey {
            x: 0.3420201f64,
            y: 0.06f64,
            in_tangent: 0.1750731f64,
            out_tangent: 0.1750731f64,
        },
        StockKspCurveKey {
            x: 0.5f64,
            y: 0.24f64,
            in_tangent: 2.60928f64,
            out_tangent: 2.60928f64,
        },
        StockKspCurveKey {
            x: 0.7071068f64,
            y: 1.7f64,
            in_tangent: 3.349777f64,
            out_tangent: 3.349777f64,
        },
        StockKspCurveKey {
            x: 1.0f64,
            y: 2.4f64,
            in_tangent: 1.387938f64,
            out_tangent: 0.0f64,
        },
    ];
    static mut drag_mach: [StockKspCurveKey; 9] = [
        StockKspCurveKey {
            x: 0.0f64,
            y: 0.35f64,
            in_tangent: 0.0f64,
            out_tangent: -0.8463008f64,
        },
        StockKspCurveKey {
            x: 0.15f64,
            y: 0.125f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
        StockKspCurveKey {
            x: 0.9f64,
            y: 0.275f64,
            in_tangent: 0.541598f64,
            out_tangent: 0.541598f64,
        },
        StockKspCurveKey {
            x: 1.1f64,
            y: 0.75f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
        StockKspCurveKey {
            x: 1.4f64,
            y: 0.4f64,
            in_tangent: -0.3626955f64,
            out_tangent: -0.3626955f64,
        },
        StockKspCurveKey {
            x: 1.6f64,
            y: 0.35f64,
            in_tangent: -0.1545923f64,
            out_tangent: -0.1545923f64,
        },
        StockKspCurveKey {
            x: 2.0f64,
            y: 0.3f64,
            in_tangent: -0.09013031f64,
            out_tangent: -0.09013031f64,
        },
        StockKspCurveKey {
            x: 5.0f64,
            y: 0.22f64,
            in_tangent: 0.0f64,
            out_tangent: 0.0f64,
        },
        StockKspCurveKey {
            x: 25.0f64,
            y: 0.3f64,
            in_tangent: 0.0006807274f64,
            out_tangent: 0.0f64,
        },
    ];
    let mut signed_aoa: ::core::ffi::c_double = if angle_of_attack.is_finite() as i32
        != 0
    {
        angle_of_attack
    } else {
        0.0f64
    };
    let mut flow: ::core::ffi::c_double = sin(
        clampd(fabs(signed_aoa), 0.0f64, 90.0f64) * DEG2RAD,
    );
    let mut m: ::core::ffi::c_double = fmax(
        0.0f64,
        if mach.is_finite() as i32 != 0 { mach } else { 0.0f64 },
    );
    let mut lift: ::core::ffi::c_double = stock_ksp_curve_eval(
        &raw const lift_curve as *const StockKspCurveKey,
        (::core::mem::size_of::<[StockKspCurveKey; 5]>() as size_t)
            .wrapping_div(::core::mem::size_of::<StockKspCurveKey>() as size_t),
        flow,
    )
        * stock_ksp_curve_eval(
            &raw const lift_mach as *const StockKspCurveKey,
            (::core::mem::size_of::<[StockKspCurveKey; 5]>() as size_t)
                .wrapping_div(::core::mem::size_of::<StockKspCurveKey>() as size_t),
            m,
        );
    let mut drag: ::core::ffi::c_double = stock_ksp_curve_eval(
        &raw const drag_curve as *const StockKspCurveKey,
        (::core::mem::size_of::<[StockKspCurveKey; 5]>() as size_t)
            .wrapping_div(::core::mem::size_of::<StockKspCurveKey>() as size_t),
        flow,
    )
        * stock_ksp_curve_eval(
            &raw const drag_mach as *const StockKspCurveKey,
            (::core::mem::size_of::<[StockKspCurveKey; 9]>() as size_t)
                .wrapping_div(::core::mem::size_of::<StockKspCurveKey>() as size_t),
            m,
        );
    if !cl.is_null() {
        *cl = copysign(fmax(0.0f64, lift), signed_aoa);
    }
    if !cd.is_null() {
        *cd = fmax(0.0f64, drag);
    }
}
#[no_mangle]
pub unsafe extern "C" fn aerodynamic_force_factors_mach(
    mut mach: ::core::ffi::c_double,
    mut angle_of_attack: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
    mut lift_factor: *mut ::core::ffi::c_double,
    mut drag_factor: *mut ::core::ffi::c_double,
) {
    let mut reference_aoa: ::core::ffi::c_double = if !v.is_null()
        && (*v).entry_angle_of_attack.is_finite() as i32 != 0
    {
        fabs((*v).entry_angle_of_attack)
    } else {
        0.0f64
    };
    let mut raw_lift: ::core::ffi::c_double = 0.0f64;
    let mut raw_drag: ::core::ffi::c_double = 0.0f64;
    let mut ref_lift: ::core::ffi::c_double = 0.0f64;
    let mut ref_drag: ::core::ffi::c_double = 0.0f64;
    stock_ksp_lifting_surface_coefficients(
        mach,
        angle_of_attack,
        &raw mut raw_lift,
        &raw mut raw_drag,
    );
    stock_ksp_lifting_surface_coefficients(
        5.0f64,
        reference_aoa,
        &raw mut ref_lift,
        &raw mut ref_drag,
    );
    let mut lf: ::core::ffi::c_double = if fabs(ref_lift) > DBL_MIN {
        raw_lift / fabs(ref_lift)
    } else {
        raw_lift
    };
    let mut df: ::core::ffi::c_double = if ref_drag > DBL_MIN {
        raw_drag / ref_drag
    } else {
        raw_drag
    };
    if !lift_factor.is_null() {
        *lift_factor = if lf.is_finite() as i32 != 0 { lf } else { 0.0f64 };
    }
    if !drag_factor.is_null() {
        *drag_factor = if df.is_finite() as i32 != 0 && df > DBL_MIN {
            df
        } else {
            DBL_MIN
        };
    }
}
#[no_mangle]
pub unsafe extern "C" fn aerodynamic_best_glide_aoa(
    mut mach: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if v.is_null() || mach.is_finite() as i32 == 0
        || !((*v).maximum_angle_of_attack > 0.0f64)
    {
        return 0.0f64;
    }
    let mut hi: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    if mach < 1.0f64
        && (*v).terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*v).terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        hi = fmin(hi, (*v).terminal_maximum_lift_angle_of_attack);
    }
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let phi: ::core::ffi::c_double = (sqrt(5.0f64) as ::core::ffi::c_double - 1.0f64)
        * 0.5f64;
    let mut x1: ::core::ffi::c_double = hi - phi * (hi - lo);
    let mut x2: ::core::ffi::c_double = lo + phi * (hi - lo);
    let mut l1: ::core::ffi::c_double = 0.0f64;
    let mut d1: ::core::ffi::c_double = 0.0f64;
    let mut l2: ::core::ffi::c_double = 0.0f64;
    let mut d2: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(mach, x1, v, &raw mut l1, &raw mut d1);
    aerodynamic_force_factors_mach(mach, x2, v, &raw mut l2, &raw mut d2);
    let mut f1: ::core::ffi::c_double = if d1 > DBL_MIN && l1.is_finite() as i32 != 0
        && d1.is_finite() as i32 != 0
    {
        fabs(l1) / d1
    } else {
        0.0f64
    };
    let mut f2: ::core::ffi::c_double = if d2 > DBL_MIN && l2.is_finite() as i32 != 0
        && d2.is_finite() as i32 != 0
    {
        fabs(l2) / d2
    } else {
        0.0f64
    };
    let mut resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON) * fmax(1.0f64, hi);
    let mut guard: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while hi - lo > resolution
        && {
            let fresh0 = guard;
            guard = guard.wrapping_add(1);
            fresh0 < (4 as ::core::ffi::c_int * DBL_MANT_DIG) as ::core::ffi::c_uint
        }
    {
        if f1 < f2 {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + phi * (hi - lo);
            aerodynamic_force_factors_mach(mach, x2, v, &raw mut l2, &raw mut d2);
            f2 = if d2 > DBL_MIN && l2.is_finite() as i32 != 0
                && d2.is_finite() as i32 != 0
            {
                fabs(l2) / d2
            } else {
                0.0f64
            };
        } else {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - phi * (hi - lo);
            aerodynamic_force_factors_mach(mach, x1, v, &raw mut l1, &raw mut d1);
            f1 = if d1 > DBL_MIN && l1.is_finite() as i32 != 0
                && d1.is_finite() as i32 != 0
            {
                fabs(l1) / d1
            } else {
                0.0f64
            };
        }
    }
    return clampd(0.5f64 * (lo + hi), 0.0f64, (*v).maximum_angle_of_attack);
}
#[no_mangle]
pub unsafe extern "C" fn aerodynamic_force_factors(
    mut angle_of_attack: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
    mut lift_factor: *mut ::core::ffi::c_double,
    mut drag_factor: *mut ::core::ffi::c_double,
) {
    aerodynamic_force_factors_mach(5.0f64, angle_of_attack, v, lift_factor, drag_factor);
}
pub const GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS: ::core::ffi::c_double = 0.08f64;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const DBL_MANT_DIG: ::core::ffi::c_int = __DBL_MANT_DIG__;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MANT_DIG__: ::core::ffi::c_int = 53 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
#[repr(C)]
pub struct TerminalModel {
    _private: [u8; 0],
}
