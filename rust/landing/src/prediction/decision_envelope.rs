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
    fn pow(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn ceil(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn taem_handoff_contract(settings: *const GuidanceSettings) -> TaemHandoffContract;
    fn world_fixed_to_inertial(
        w: *const KerbinWorld,
        fixed: Vec3,
        ut: ::core::ffi::c_double,
    ) -> Vec3;
    fn world_atmosphere_sample_state(
        w: *const KerbinWorld,
        position_i: Vec3,
        ut: ::core::ffi::c_double,
    ) -> AtmosphereSample;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_taem_range_target(
        planet: *const PlanetModel,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
    fn final_alignment_contract(
        distance_m: ::core::ffi::c_double,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
    ) -> FinalAlignmentContract;
    fn final_alignment_distance_for_state(
        guidance: *const GuidanceMachine,
        altitude_m: ::core::ffi::c_double,
        speed_mps: ::core::ffi::c_double,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
    fn rotating_specific_energy(
        latitude: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
        air_relative_speed: ::core::ffi::c_double,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn entry_remaining_specific_energy(
        latitude: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
        air_relative_speed: ::core::ffi::c_double,
        target_latitude: ::core::ffi::c_double,
        target_altitude: ::core::ffi::c_double,
        target_speed: ::core::ffi::c_double,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn telemetry_reframe_runway(
        telemetry: *mut Telemetry,
        site: *const LandingSite,
        radius: ::core::ffi::c_double,
    );
    fn taem_alignment_maneuver_geometry(
        altitude: ::core::ffi::c_double,
        radius: ::core::ffi::c_double,
        minimum_turn_radius: ::core::ffi::c_double,
        alignment_turn_deg: ::core::ffi::c_double,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
        remaining_path: *mut ::core::ffi::c_double,
        glide_slope: *mut ::core::ffi::c_double,
        final_altitude: *mut ::core::ffi::c_double,
    ) -> bool;
    fn planet_surface_gravity(p: *const PlanetModel) -> ::core::ffi::c_double;
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
        lift_factor_0: *mut ::core::ffi::c_double,
        drag_factor: *mut ::core::ffi::c_double,
    );
    fn v3_add(a: Vec3, b: Vec3) -> Vec3;
    fn v3_sub(a: Vec3, b: Vec3) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_dot(a: Vec3, b: Vec3) -> ::core::ffi::c_double;
    fn v3_cross(a: Vec3, b: Vec3) -> Vec3;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
    fn v3_normalized(a: Vec3) -> Vec3;
    fn v3_rotate_axis(v: Vec3, axis: Vec3, angle_rad: ::core::ffi::c_double) -> Vec3;
    fn aero_apply_mach_observation(
        a: *mut AeroTable,
        mach: ::core::ffi::c_double,
        lift_scale: ::core::ffi::c_double,
        drag_scale: ::core::ffi::c_double,
    );
    fn aero_force_per_q(
        a: *const AeroTable,
        q: ::core::ffi::c_double,
        mach: ::core::ffi::c_double,
        alpha_deg: ::core::ffi::c_double,
        lift_per_q: *mut ::core::ffi::c_double,
        drag_per_q: *mut ::core::ffi::c_double,
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
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
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
pub struct SpeedbrakeController {
    pub integral: ::core::ffi::c_double,
    pub deployed: bool,
    pub initialized: bool,
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
pub struct TaemInterfaceCapture {
    pub valid: bool,
    pub ready: bool,
    pub veto: ::core::ffi::c_uint,
    pub along: ::core::ffi::c_double,
    pub cross: ::core::ffi::c_double,
    pub course_error: ::core::ffi::c_double,
    pub altitude_error: ::core::ffi::c_double,
    pub energy_margin: ::core::ffi::c_double,
    pub turn_margin: ::core::ffi::c_double,
    pub path_length_m: ::core::ffi::c_double,
    pub required_time_s: ::core::ffi::c_double,
    pub available_time_s: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
    pub energy_available: ::core::ffi::c_double,
    pub energy_drag_work: ::core::ffi::c_double,
    pub energy_uncertainty: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EnergyPathEnvelope {
    pub valid: bool,
    pub feasible: bool,
    pub energy: DecisionMargin,
    pub available_specific_energy: ::core::ffi::c_double,
    pub modeled_drag_work: ::core::ffi::c_double,
    pub uncertainty_specific_energy: ::core::ffi::c_double,
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
pub struct UnpoweredPathProjection {
    pub valid: bool,
    pub reference_reachable: bool,
    pub projected_ground_path_m: ::core::ffi::c_double,
    pub terminal_latitude_deg: ::core::ffi::c_double,
    pub integration_error_j_kg: ::core::ffi::c_double,
    pub ground_path_m: ::core::ffi::c_double,
    pub air_path_m: ::core::ffi::c_double,
    pub modeled_drag_work: ::core::ffi::c_double,
    pub terminal_speed_mps: ::core::ffi::c_double,
    pub travel_time_s: ::core::ffi::c_double,
    pub target_aoa_deg: ::core::ffi::c_double,
}
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
pub type TerminalPathKind = ::core::ffi::c_uint;
pub const TERMINAL_PATH_HAC: TerminalPathKind = 2;
pub const TERMINAL_PATH_DIRECT: TerminalPathKind = 1;
pub const TERMINAL_PATH_NONE: TerminalPathKind = 0;
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
pub struct TerminalModel {
    pub schema_version: uint32_t,
    pub snapshot_id: uint64_t,
    pub atmosphere_source_digest: uint64_t,
    pub aero_source_digest: uint64_t,
    pub aero_book_source_digest: uint64_t,
    pub attitude_source_digest: uint64_t,
    pub replay_validated: bool,
    pub planet_frame_aligned: bool,
    pub world: KerbinWorld,
    pub aero: AeroTable,
    pub attitude: AttitudeModel,
    pub vehicle: VehicleProfile,
    pub site: LandingSite,
    pub guidance: GuidanceSettings,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AttitudeModel {
    pub aoa_rad: ::core::ffi::c_double,
    pub bank_rad: ::core::ffi::c_double,
    pub aoa_rate_rad_s: ::core::ffi::c_double,
    pub bank_rate_rad_s: ::core::ffi::c_double,
    pub requested_aoa_rad: ::core::ffi::c_double,
    pub requested_bank_rad: ::core::ffi::c_double,
    pub cmd_aoa_rad: ::core::ffi::c_double,
    pub cmd_bank_rad: ::core::ffi::c_double,
    pub pitch_wn: ::core::ffi::c_double,
    pub pitch_zeta: ::core::ffi::c_double,
    pub roll_wn: ::core::ffi::c_double,
    pub roll_zeta: ::core::ffi::c_double,
    pub max_pitch_rate_rad_s: ::core::ffi::c_double,
    pub max_roll_rate_rad_s: ::core::ffi::c_double,
    pub max_pitch_accel_rad_s2: ::core::ffi::c_double,
    pub max_roll_accel_rad_s2: ::core::ffi::c_double,
    pub pitch_full_authority_q_pa: ::core::ffi::c_double,
    pub roll_full_authority_q_pa: ::core::ffi::c_double,
    pub direct: bool,
    pub direct_body_coordinates: bool,
    pub body_rates_initialized: bool,
    pub body_pitch_rate_rad_s: ::core::ffi::c_double,
    pub body_roll_rate_rad_s: ::core::ffi::c_double,
    pub body_yaw_rate_rad_s: ::core::ffi::c_double,
    pub input_pitch: ::core::ffi::c_double,
    pub input_roll: ::core::ffi::c_double,
    pub input_yaw: ::core::ffi::c_double,
    pub sideslip_rad: ::core::ffi::c_double,
    pub sideslip_rate_rad_s: ::core::ffi::c_double,
    pub direct_min_q_pa: ::core::ffi::c_double,
    pub direct_pitch_accel_per_kpa: ::core::ffi::c_double,
    pub direct_pitch_accel_max: ::core::ffi::c_double,
    pub direct_pitch_stiffness_per_kpa: ::core::ffi::c_double,
    pub direct_trim_aoa_rad: ::core::ffi::c_double,
    pub direct_pitch_damping_ratio: ::core::ffi::c_double,
    pub direct_roll_accel_per_kpa: ::core::ffi::c_double,
    pub direct_roll_accel_max: ::core::ffi::c_double,
    pub direct_roll_damping_s_inv: ::core::ffi::c_double,
    pub direct_yaw_accel_per_kpa: ::core::ffi::c_double,
    pub direct_yaw_accel_max: ::core::ffi::c_double,
    pub direct_yaw_stiffness_per_kpa: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroTable {
    pub mach: [::core::ffi::c_double; 40],
    pub alpha_deg: [::core::ffi::c_double; 40],
    pub cl: [[::core::ffi::c_double; 40]; 40],
    pub cd: [[::core::ffi::c_double; 40]; 40],
    pub mach_count: size_t,
    pub alpha_count: size_t,
    pub reference_area_m2: ::core::ffi::c_double,
    pub book: [AeroBookPoint; 2048],
    pub book_count: size_t,
    pub book_enabled: bool,
    pub lift_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroBookPoint {
    pub q_pa: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub alpha_deg: ::core::ffi::c_double,
    pub lift_per_q_m2: ::core::ffi::c_double,
    pub drag_per_q_m2: ::core::ffi::c_double,
    pub support: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KerbinWorld {
    pub radius_m: ::core::ffi::c_double,
    pub mu_m3_s2: ::core::ffi::c_double,
    pub rotation_rate_rad_s: ::core::ffi::c_double,
    pub atmosphere_top_m: ::core::ffi::c_double,
    pub rotation_phase_rad_at_ut0: ::core::ffi::c_double,
    pub orbital_rate_rad_s: ::core::ffi::c_double,
    pub solar_phase_rad_at_ut0: ::core::ffi::c_double,
    pub stock_spatial_atmosphere: bool,
    pub atmosphere: AtmosphereTable,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereTable {
    pub p: [AtmospherePoint; 2048],
    pub count: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmospherePoint {
    pub altitude_m: ::core::ffi::c_double,
    pub sample: AtmosphereSample,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereSample {
    pub density_kg_m3: ::core::ffi::c_double,
    pub pressure_pa: ::core::ffi::c_double,
    pub temperature_k: ::core::ffi::c_double,
    pub speed_of_sound_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PathForceScale {
    pub base: ::core::ffi::c_double,
    pub lbase: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GuidanceAttitudeLimits {
    pub rate_deg_s: ::core::ffi::c_double,
    pub accel_deg_s2: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlAuthorityEnvelope {
    pub valid: bool,
    pub survivable: bool,
    pub controllable: bool,
    pub speed: DecisionMargin,
    pub dynamic_pressure: DecisionMargin,
    pub load: DecisionMargin,
    pub stall: DecisionMargin,
    pub stall_observed: bool,
    pub minimum_speed_mps: ::core::ffi::c_double,
    pub maximum_lift_aoa_deg: ::core::ffi::c_double,
    pub maximum_normal_accel_mps2: ::core::ffi::c_double,
    pub maximum_lateral_accel_mps2: ::core::ffi::c_double,
    pub control_response_time_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HandoffSetEnvelope {
    pub valid: bool,
    pub position_ready: bool,
    pub heading_ready: bool,
    pub inside: bool,
    pub control: ControlAuthorityEnvelope,
    pub position: DecisionMargin,
    pub heading: DecisionMargin,
    pub center_distance_m: ::core::ffi::c_double,
    pub heading_error_deg: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
    pub path_to_set_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VerticalRecoveryEnvelope {
    pub valid: bool,
    pub reachable: bool,
    pub height: DecisionMargin,
    pub response_height_m: ::core::ffi::c_double,
    pub braking_height_m: ::core::ffi::c_double,
    pub required_height_m: ::core::ffi::c_double,
    pub sink_after_response_mps: ::core::ffi::c_double,
    pub recovery_time_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemSpeedEnvelope {
    pub valid: bool,
    pub feasible: bool,
    pub minimum_speed_mps: ::core::ffi::c_double,
    pub maximum_speed_mps: ::core::ffi::c_double,
    pub density_kg_m3: ::core::ffi::c_double,
    pub dynamic_pressure_limit_pa: ::core::ffi::c_double,
    pub load_pressure_limit_pa: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TargetCaptureEnvelope {
    pub valid: bool,
    pub reachable: bool,
    pub control: ControlAuthorityEnvelope,
    pub capture_time: DecisionMargin,
    pub forward_m: ::core::ffi::c_double,
    pub cross_m: ::core::ffi::c_double,
    pub range_m: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
    pub path_length_m: ::core::ffi::c_double,
    pub time_available_s: ::core::ffi::c_double,
    pub required_time_s: ::core::ffi::c_double,
    pub lateral_capture_time_s: ::core::ffi::c_double,
    pub heading_capture_time_s: ::core::ffi::c_double,
    pub course_error_deg: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PathFeasibilityEnvelope {
    pub valid: bool,
    pub feasible: bool,
    pub path_minimum: DecisionMargin,
    pub path_maximum: DecisionMargin,
    pub slope_minimum: DecisionMargin,
    pub slope_maximum: DecisionMargin,
    pub lateral_authority: DecisionMargin,
    pub course_rate: DecisionMargin,
    pub endpoint_rate: DecisionMargin,
    pub worst_normalized_margin: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RunwayCaptureEnvelope {
    pub valid: bool,
    pub reachable: bool,
    pub runway_time: DecisionMargin,
    pub vertical_recovery: DecisionMargin,
    pub speed: DecisionMargin,
    pub dynamic_pressure: DecisionMargin,
    pub load: DecisionMargin,
    pub runway_remaining: DecisionMargin,
    pub ground_range_m: ::core::ffi::c_double,
    pub time_available_s: ::core::ffi::c_double,
    pub lateral_capture_time_s: ::core::ffi::c_double,
    pub heading_capture_time_s: ::core::ffi::c_double,
    pub control_response_time_s: ::core::ffi::c_double,
    pub cross_rate_mps: ::core::ffi::c_double,
    pub lateral_accel_mps2: ::core::ffi::c_double,
    pub course_rate_deg_s: ::core::ffi::c_double,
    pub maximum_lift_aoa_deg: ::core::ffi::c_double,
    pub maximum_normal_accel_mps2: ::core::ffi::c_double,
    pub sink_rate_mps: ::core::ffi::c_double,
    pub vertical_recovery_accel_mps2: ::core::ffi::c_double,
    pub response_down_accel_mps2: ::core::ffi::c_double,
    pub vertical_recovery_height_m: ::core::ffi::c_double,
    pub recoverable_sink_rate_mps: ::core::ffi::c_double,
    pub recoverable_fpa_deg: ::core::ffi::c_double,
}
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MANT_DIG__: ::core::ffi::c_int = 53 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
#[inline]
unsafe extern "C" fn taem_route_station_at_index(
    mut route: *const TaemRoute,
    mut index: size_t,
) -> ::core::ffi::c_double {
    if route.is_null() || !(*route).valid || (*route).count < 2 as size_t
        || (*route).lead_count == 0 as size_t || (*route).arc_count == 0 as size_t
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if index >= (*route).count {
        index = (*route).count.wrapping_sub(1 as size_t);
    }
    if index <= (*route).lead_count {
        return (*route).lead_length_m * index as ::core::ffi::c_double
            / (*route).lead_count as ::core::ffi::c_double;
    }
    if index <= (*route).lead_count.wrapping_add((*route).arc_count) {
        return (*route).lead_length_m
            + (*route).hac.arc_length_m
                * index.wrapping_sub((*route).lead_count) as ::core::ffi::c_double
                / (*route).arc_count as ::core::ffi::c_double;
    }
    if (*route).rollout_count == 0 as size_t {
        return (*route).lead_length_m + (*route).hac.arc_length_m;
    }
    return (*route).lead_length_m + (*route).hac.arc_length_m
        + (*route).rollout_length_m
            * index.wrapping_sub((*route).lead_count).wrapping_sub((*route).arc_count)
                as ::core::ffi::c_double
            / (*route).rollout_count as ::core::ffi::c_double;
}
#[inline]
unsafe extern "C" fn taem_route_remaining_at_index(
    mut route: *const TaemRoute,
    mut index: size_t,
) -> ::core::ffi::c_double {
    let mut station: ::core::ffi::c_double = taem_route_station_at_index(route, index);
    if station.is_finite() as i32 == 0 || (*route).length_m.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return fmax(0.0f64, (*route).length_m - station);
}
#[no_mangle]
pub unsafe extern "C" fn entry_taem_interface_capture(
    mut q: *const TaemInterfaceTarget,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> TaemInterfaceCapture {
    let mut out: TaemInterfaceCapture = TaemInterfaceCapture {
        valid: 0 as ::core::ffi::c_int != 0,
        ready: false,
        veto: 0,
        along: 0.,
        cross: 0.,
        course_error: 0.,
        altitude_error: 0.,
        energy_margin: 0.,
        turn_margin: 0.,
        path_length_m: 0.,
        required_time_s: 0.,
        available_time_s: 0.,
        minimum_turn_radius_m: 0.,
        energy_available: 0.,
        energy_drag_work: 0.,
        energy_uncertainty: 0.,
    };
    if q.is_null() || t.is_null() || p.is_null() || cfg.is_null() {
        return out;
    }
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    if !(*q).valid || (*q).along_track.is_finite() as i32 == 0
        || (*q).cross_track.is_finite() as i32 == 0
        || (*q).course.is_finite() as i32 == 0 || (*q).altitude.is_finite() as i32 == 0
        || (*q).speed.is_finite() as i32 == 0
        || (*q).speed <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*q).flight_path_angle.is_finite() as i32 == 0
        || (*q).specific_energy.is_finite() as i32 == 0
        || (*q).acquisition_lead.is_finite() as i32 == 0
        || (*q).acquisition_lead <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*q).remaining_path.is_finite() as i32 == 0
        || (*q).remaining_path < (*q).acquisition_lead
        || (*q).response_time.is_finite() as i32 == 0
        || (*q).response_time <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*t).latitude.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
        || (*t).mean_altitude.is_finite() as i32 == 0
        || (*t).flight_path_angle.is_finite() as i32 == 0
        || course.is_finite() as i32 == 0
        || (*t).horizontal_speed.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0
        || (*t).g_force.is_finite() as i32 == 0
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction.is_finite() as i32 == 0
    {
        return out;
    }
    let mut h: ::core::ffi::c_double = norm_signed_deg(
        (*q).course - (*cfg).site.runway_heading,
    ) * DEG2RAD;
    let mut da: ::core::ffi::c_double = (*q).along_track - (*t).runway_along_track;
    let mut dc: ::core::ffi::c_double = (*q).cross_track - (*t).runway_cross_track;
    out.along = da * cos(h) + dc * sin(h);
    out.cross = -da * sin(h) + dc * cos(h);
    out.course_error = norm_signed_deg((*q).course - course);
    let mut horizontal: ::core::ffi::c_double = (*t).horizontal_speed;
    let mut response: ::core::ffi::c_double = (*q).response_time;
    let mut control: ControlAuthorityEnvelope = decision_control_authority_envelope(
        ::core::ptr::null::<GuidanceMachine>(),
        t,
        p,
        cfg,
    );
    if !control.valid || !control.controllable {
        return out;
    }
    let mut minimum_turn_radius: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut path_length: ::core::ffi::c_double = decision_target_path_length(
        ::core::ptr::null::<GuidanceMachine>(),
        t,
        course,
        (*q).along_track,
        (*q).cross_track,
        (*q).course,
        ::core::f32::NAN as ::core::ffi::c_double,
        p,
        cfg,
        &raw mut minimum_turn_radius,
    );
    let mut slope: ::core::ffi::c_double = -(*q).flight_path_angle * DEG2RAD;
    let mut slope_valid: bool = (*q).flight_path_angle < 0.0f64
        && (*q).flight_path_angle > -90.0f64 && slope.is_finite() as i32 != 0
        && cos(slope) > DBL_EPSILON;
    let mut corridor_altitude: ::core::ffi::c_double = if slope_valid
        as ::core::ffi::c_int != 0
    {
        (*q).altitude + out.along * tan(slope)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.altitude_error = if corridor_altitude.is_finite() as i32 != 0 {
        (*t).mean_altitude - corridor_altitude
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.energy_margin = ::core::f32::NAN as ::core::ffi::c_double;
    let mut handoff_set: HandoffSetEnvelope = decision_mm304_handoff_set_envelope(
        ::core::ptr::null::<GuidanceMachine>(),
        t,
        course,
        (*q).along_track,
        (*q).cross_track,
        p,
        cfg,
    );
    let mut capture_path: ::core::ffi::c_double = if handoff_set.valid
        as ::core::ffi::c_int != 0
    {
        handoff_set.path_to_set_m
    } else {
        path_length
    };
    let mut required_time: ::core::ffi::c_double = if capture_path.is_finite() as i32
        != 0 && horizontal > DBL_MIN
    {
        response + capture_path / horizontal
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut available_time: ::core::ffi::c_double = if (*q).arrival_ut.is_finite() as i32
        != 0
    {
        (*q).arrival_ut - (*t).ut
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.path_length_m = capture_path;
    out.required_time_s = required_time;
    out.available_time_s = available_time;
    out.minimum_turn_radius_m = minimum_turn_radius;
    out.turn_margin = if required_time.is_finite() as i32 != 0
        && available_time.is_finite() as i32 != 0
    {
        (available_time - required_time) * horizontal
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.valid = out.along.is_finite() as i32 != 0 && out.cross.is_finite() as i32 != 0
        && out.turn_margin.is_finite() as i32 != 0
        && minimum_turn_radius.is_finite() as i32 != 0
        && slope_valid as ::core::ffi::c_int != 0;
    if !out.valid {
        return out;
    }
    let mut handoff: TaemHandoffContract = taem_handoff_contract(s);
    let mut position_error: ::core::ffi::c_double = hypot(out.along, out.cross);
    let mut inside_spatial: bool = position_error.is_finite() as i32 != 0
        && position_error <= handoff.horizontal_radius_m;
    if !inside_spatial {
        out.veto |= 2 as ::core::ffi::c_uint;
    }
    let mut runway_offset: ::core::ffi::c_double = fabs(
        norm_signed_deg(course - (*cfg).site.runway_heading),
    );
    let mut heading_contract_ready: bool = fabs(runway_offset - 90.0f64)
        <= handoff.perpendicular_heading_half_width_deg;
    if !heading_contract_ready {
        out.veto |= 4 as ::core::ffi::c_uint;
    }
    if (*t).true_air_speed < (*v).minimum_safe_speed {
        out.veto |= 1 as ::core::ffi::c_uint;
    }
    if !((*t).vertical_speed < 0.0f64) {
        out.veto |= 32 as ::core::ffi::c_uint;
    }
    if (*t).mean_altitude < (*s).mm305_min_altitude
        || (*t).mean_altitude > (*s).mm305_max_altitude
    {
        out.veto |= 8 as ::core::ffi::c_uint;
    }
    let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        (*t).mean_altitude,
    );
    let mut live_mach: ::core::ffi::c_double = if (*t).mach.is_finite() as i32 != 0
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
    if live_mach.is_finite() as i32 == 0 || live_mach < mach_min || live_mach > mach_max
    {
        out.veto |= 1 as ::core::ffi::c_uint;
    }
    let mut terminal_radius: ::core::ffi::c_double = fmax(
        (*s).hac_radius,
        minimum_turn_radius,
    );
    let mut maneuver_path: ::core::ffi::c_double = 0.0f64;
    let mut maneuver_slope: ::core::ffi::c_double = 0.0f64;
    let mut final_altitude: ::core::ffi::c_double = 0.0f64;
    let mut maneuver_geometry: bool = taem_alignment_maneuver_geometry(
        (*t).mean_altitude,
        terminal_radius,
        minimum_turn_radius,
        fabs(norm_signed_deg((*q).course - (*cfg).site.runway_heading)),
        &raw const (*cfg).site,
        s,
        &raw mut maneuver_path,
        &raw mut maneuver_slope,
        &raw mut final_altitude,
    ) as ::core::ffi::c_int != 0 && (*t).true_air_speed >= (*v).minimum_safe_speed;
    let mut final_speed: ::core::ffi::c_double = fmax(
        (*s).final_alignment_speed,
        fmax((*v).final_approach_speed, (*v).minimum_safe_speed),
    );
    let mut available_energy: ::core::ffi::c_double = entry_remaining_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        (*cfg).site.latitude,
        final_altitude,
        final_speed,
        p,
    );
    let mut maneuver_projection: UnpoweredPathProjection = UnpoweredPathProjection {
        valid: 0 as ::core::ffi::c_int != 0,
        reference_reachable: false,
        projected_ground_path_m: 0.,
        terminal_latitude_deg: 0.,
        integration_error_j_kg: 0.,
        ground_path_m: 0.,
        air_path_m: 0.,
        modeled_drag_work: 0.,
        terminal_speed_mps: 0.,
        travel_time_s: 0.,
        target_aoa_deg: 0.,
    };
    if maneuver_geometry {
        maneuver_projection = decision_unpowered_path_projection(
            ::core::ptr::null::<GuidanceMachine>(),
            t,
            p,
            cfg,
            final_altitude,
            maneuver_path,
        );
    }
    let mut maneuver_energy: EnergyPathEnvelope = EnergyPathEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        feasible: false,
        energy: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        available_specific_energy: 0.,
        modeled_drag_work: 0.,
        uncertainty_specific_energy: 0.,
    };
    if maneuver_projection.valid {
        maneuver_energy = decision_energy_path_envelope(
            t,
            p,
            available_energy,
            maneuver_projection.modeled_drag_work,
            maneuver_projection.air_path_m,
        );
    }
    out.energy_margin = if maneuver_energy.valid as ::core::ffi::c_int != 0 {
        maneuver_energy.energy.margin
    } else {
        0.0f64
    };
    out.energy_available = if maneuver_energy.valid as ::core::ffi::c_int != 0 {
        maneuver_energy.available_specific_energy
    } else {
        0.0f64
    };
    out.energy_drag_work = if maneuver_energy.valid as ::core::ffi::c_int != 0 {
        maneuver_energy.modeled_drag_work
    } else {
        0.0f64
    };
    out.energy_uncertainty = if maneuver_energy.valid as ::core::ffi::c_int != 0 {
        maneuver_energy.uncertainty_specific_energy
    } else {
        0.0f64
    };
    if maneuver_geometry as ::core::ffi::c_int != 0
        && (!maneuver_energy.valid || out.energy_margin < 0.0f64
            || out.turn_margin < 0.0f64)
    {
        out.veto |= 16 as ::core::ffi::c_uint;
    }
    if (*t).dynamic_pressure > (*v).maximum_dynamic_pressure
        || (*t).g_force > (*v).maximum_g_load
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction >= 1.0f64
    {
        out.veto |= 64 as ::core::ffi::c_uint;
    }
    out.ready = out.veto == 0 as ::core::ffi::c_uint;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn entry_mm305_acquisition_capture(
    mut g: *const GuidanceMachine,
    mut q: *const TaemInterfaceTarget,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> TaemInterfaceCapture {
    let mut out: TaemInterfaceCapture = TaemInterfaceCapture {
        valid: 0 as ::core::ffi::c_int != 0,
        ready: false,
        veto: 0,
        along: 0.,
        cross: 0.,
        course_error: 0.,
        altitude_error: 0.,
        energy_margin: 0.,
        turn_margin: 0.,
        path_length_m: 0.,
        required_time_s: 0.,
        available_time_s: 0.,
        minimum_turn_radius_m: 0.,
        energy_available: 0.,
        energy_drag_work: 0.,
        energy_uncertainty: 0.,
    };
    if q.is_null() || t.is_null() || p.is_null() || cfg.is_null() || !(*q).valid
        || (*q).along_track.is_finite() as i32 == 0
        || (*q).cross_track.is_finite() as i32 == 0
        || (*q).course.is_finite() as i32 == 0 || (*t).latitude.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
        || (*t).horizontal_speed.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).mean_altitude.is_finite() as i32 == 0 || course.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0
        || (*t).g_force.is_finite() as i32 == 0
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction.is_finite() as i32 == 0
    {
        return out;
    }
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut control: ControlAuthorityEnvelope = decision_control_authority_envelope(
        g,
        t,
        p,
        cfg,
    );
    if !control.valid || !control.controllable
        || !(control.maximum_lateral_accel_mps2 > DBL_MIN)
        || !((*t).horizontal_speed > DBL_MIN)
    {
        return out;
    }
    let mut qh: ::core::ffi::c_double = norm_signed_deg(
        (*q).course - (*cfg).site.runway_heading,
    ) * DEG2RAD;
    let mut da: ::core::ffi::c_double = (*q).along_track - (*t).runway_along_track;
    let mut dc: ::core::ffi::c_double = (*q).cross_track - (*t).runway_cross_track;
    out.along = da * cos(qh) + dc * sin(qh);
    out.cross = -da * sin(qh) + dc * cos(qh);
    out.course_error = norm_signed_deg((*q).course - course);
    out.altitude_error = (*t).mean_altitude - (*s).taem_interface_altitude;
    out.minimum_turn_radius_m = (*t).horizontal_speed * (*t).horizontal_speed
        / control.maximum_lateral_accel_mps2;
    let mut path_length: ::core::ffi::c_double = decision_target_path_length(
        g,
        t,
        course,
        (*q).along_track,
        (*q).cross_track,
        (*q).course,
        control.maximum_lateral_accel_mps2,
        p,
        cfg,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    out.path_length_m = if path_length.is_finite() as i32 != 0 {
        path_length
    } else {
        hypot(da, dc)
    };
    out.required_time_s = if out.path_length_m.is_finite() as i32 != 0 {
        control.control_response_time_s + out.path_length_m / (*t).horizontal_speed
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.available_time_s = if (*q).arrival_ut.is_finite() as i32 != 0 {
        (*q).arrival_ut - (*t).ut
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.turn_margin = if out.required_time_s.is_finite() as i32 != 0
        && out.available_time_s.is_finite() as i32 != 0
    {
        (out.available_time_s - out.required_time_s) * (*t).horizontal_speed
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut final_altitude: ::core::ffi::c_double = (*cfg).site.altitude
        + (*s).final_approach_distance * tan((*s).final_glide_slope * DEG2RAD);
    let mut final_speed: ::core::ffi::c_double = fmax(
        (*s).final_alignment_speed,
        fmax((*v).final_approach_speed, (*v).minimum_safe_speed),
    );
    out.energy_margin = entry_remaining_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        (*cfg).site.latitude,
        final_altitude,
        final_speed,
        p,
    );
    out.energy_available = out.energy_margin;
    out.valid = out.along.is_finite() as i32 != 0 && out.cross.is_finite() as i32 != 0
        && out.minimum_turn_radius_m.is_finite() as i32 != 0
        && out.energy_margin.is_finite() as i32 != 0;
    if !out.valid {
        return out;
    }
    let mut range: ::core::ffi::c_double = hypot(
        (*t).runway_along_track,
        (*t).runway_cross_track,
    );
    let mut max_range: ::core::ffi::c_double = entry_taem_range_target(p, s);
    let mut min_range: ::core::ffi::c_double = fmax(
        (*s).final_approach_distance * 1.5f64,
        (*s).final_approach_distance + fmax(0.0f64, (*s).hac_radius),
    );
    if range.is_finite() as i32 == 0 || max_range.is_finite() as i32 == 0
        || range < min_range || range > max_range
    {
        out.veto |= 2 as ::core::ffi::c_uint;
    }
    let mut runway: ::core::ffi::c_double = (*cfg).site.runway_heading * DEG2RAD;
    let mut de: ::core::ffi::c_double = -(*t).runway_along_track * sin(runway)
        - (*t).runway_cross_track * cos(runway);
    let mut dn: ::core::ffi::c_double = -(*t).runway_along_track * cos(runway)
        + (*t).runway_cross_track * sin(runway);
    let mut site_course: ::core::ffi::c_double = norm_deg(atan2(de, dn) * RAD2DEG);
    let mut closing_error: ::core::ffi::c_double = norm_signed_deg(site_course - course);
    if closing_error.is_finite() as i32 == 0 || cos(closing_error * DEG2RAD) <= 0.0f64 {
        out.veto |= 4 as ::core::ffi::c_uint;
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
    if (*t).true_air_speed < (*v).minimum_safe_speed || mach.is_finite() as i32 == 0
        || mach < (*s).mm305_target_mach - (*s).mm305_mach_half_width
        || mach > (*s).mm305_target_mach + (*s).mm305_mach_half_width
    {
        out.veto |= 1 as ::core::ffi::c_uint;
    }
    if (*t).mean_altitude < (*s).mm305_min_altitude
        || (*t).mean_altitude > (*s).mm305_max_altitude
    {
        out.veto |= 8 as ::core::ffi::c_uint;
    }
    if !(out.energy_margin > 0.0f64) {
        out.veto |= 16 as ::core::ffi::c_uint;
    }
    if !((*t).vertical_speed < 0.0f64) {
        out.veto |= 32 as ::core::ffi::c_uint;
    }
    if (*t).dynamic_pressure > (*v).maximum_dynamic_pressure
        || (*t).g_force > (*v).maximum_g_load
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction >= 1.0f64
    {
        out.veto |= 64 as ::core::ffi::c_uint;
    }
    out.ready = out.veto == 0 as ::core::ffi::c_uint;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn entry_mm305_admission_prefilter_veto(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> ::core::ffi::c_uint {
    if t.is_null() || p.is_null() || cfg.is_null()
        || (*t).mean_altitude.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0
        || (*t).g_force.is_finite() as i32 == 0
    {
        return 0xffffffff as ::core::ffi::c_uint;
    }
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut veto: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
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
    if (*t).true_air_speed < (*v).minimum_safe_speed || mach.is_finite() as i32 == 0
        || mach < (*s).mm305_target_mach - (*s).mm305_mach_half_width
        || mach > (*s).mm305_target_mach + (*s).mm305_mach_half_width
    {
        veto |= 1 as ::core::ffi::c_uint;
    }
    if (*t).mean_altitude < (*s).mm305_min_altitude
        || (*t).mean_altitude > (*s).mm305_max_altitude
    {
        veto |= 8 as ::core::ffi::c_uint;
    }
    let mut range: ::core::ffi::c_double = hypot(
        (*t).runway_along_track,
        (*t).runway_cross_track,
    );
    let mut max_range: ::core::ffi::c_double = entry_taem_range_target(p, s);
    if range.is_finite() as i32 == 0 || max_range.is_finite() as i32 == 0
        || range > max_range
    {
        veto |= 2 as ::core::ffi::c_uint;
    }
    if !((*t).vertical_speed < 0.0f64) {
        veto |= 32 as ::core::ffi::c_uint;
    }
    if (*t).course_to_site_error.is_finite() as i32 != 0
        && fabs((*t).course_to_site_error) > 90.0f64
    {
        veto |= 128 as ::core::ffi::c_uint;
    }
    if (*t).dynamic_pressure > (*v).maximum_dynamic_pressure
        || (*t).g_force > (*v).maximum_g_load
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction >= 1.0f64
    {
        veto |= 64 as ::core::ffi::c_uint;
    }
    return veto;
}
#[no_mangle]
pub unsafe extern "C" fn entry_mm305_admission_envelope(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> TaemInterfaceCapture {
    let mut out: TaemInterfaceCapture = TaemInterfaceCapture {
        valid: 0 as ::core::ffi::c_int != 0,
        ready: false,
        veto: 0,
        along: 0.,
        cross: 0.,
        course_error: 0.,
        altitude_error: 0.,
        energy_margin: 0.,
        turn_margin: 0.,
        path_length_m: 0.,
        required_time_s: 0.,
        available_time_s: 0.,
        minimum_turn_radius_m: 0.,
        energy_available: 0.,
        energy_drag_work: 0.,
        energy_uncertainty: 0.,
    };
    if t.is_null() || p.is_null() || cfg.is_null()
        || (*t).latitude.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
        || (*t).horizontal_speed.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).mean_altitude.is_finite() as i32 == 0 || course.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0
        || (*t).g_force.is_finite() as i32 == 0
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction.is_finite() as i32 == 0
    {
        return out;
    }
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut control: ControlAuthorityEnvelope = decision_control_authority_envelope(
        g,
        t,
        p,
        cfg,
    );
    if !control.valid || !(control.maximum_normal_accel_mps2 > 0.0f64)
        || !(control.maximum_lateral_accel_mps2 > DBL_MIN)
        || !((*t).horizontal_speed > DBL_MIN)
    {
        return out;
    }
    let mut alignment_distance: ::core::ffi::c_double = final_alignment_distance_for_state(
        g,
        (*t).mean_altitude,
        (*t).true_air_speed,
        &raw const (*cfg).site,
        v,
        s,
    );
    let mut alignment: FinalAlignmentContract = final_alignment_contract(
        alignment_distance,
        &raw const (*cfg).site,
        v,
        s,
    );
    if !alignment.valid {
        return out;
    }
    let mut final_along: ::core::ffi::c_double = -alignment.distance_m;
    let mut final_cross: ::core::ffi::c_double = 0.0f64;
    let mut final_course: ::core::ffi::c_double = (*cfg).site.runway_heading;
    let mut da: ::core::ffi::c_double = final_along - (*t).runway_along_track;
    let mut dc: ::core::ffi::c_double = final_cross - (*t).runway_cross_track;
    let mut minimum_path: ::core::ffi::c_double = hypot(da, dc);
    let mut minimum_turn_radius: ::core::ffi::c_double = (*t).horizontal_speed
        * (*t).horizontal_speed / control.maximum_lateral_accel_mps2;
    if minimum_path.is_finite() as i32 == 0
        || minimum_turn_radius.is_finite() as i32 == 0 || !(minimum_turn_radius > 0.0f64)
    {
        return out;
    }
    let mut final_altitude: ::core::ffi::c_double = alignment.altitude_m;
    let mut final_speed: ::core::ffi::c_double = alignment.speed_mps;
    let mut available_energy: ::core::ffi::c_double = entry_remaining_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        (*cfg).site.latitude,
        final_altitude,
        final_speed,
        p,
    );
    let mut projection: UnpoweredPathProjection = decision_unpowered_path_projection(
        g,
        t,
        p,
        cfg,
        final_altitude,
        minimum_path,
    );
    if projection.valid {
        available_energy = entry_remaining_specific_energy(
            (*t).latitude,
            (*t).mean_altitude,
            (*t).true_air_speed,
            projection.terminal_latitude_deg,
            final_altitude,
            final_speed,
            p,
        );
    }
    let mut energy: EnergyPathEnvelope = EnergyPathEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        feasible: false,
        energy: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        available_specific_energy: 0.,
        modeled_drag_work: 0.,
        uncertainty_specific_energy: 0.,
    };
    if projection.valid as ::core::ffi::c_int != 0
        && available_energy.is_finite() as i32 != 0
    {
        energy = decision_energy_path_envelope(
            t,
            p,
            available_energy,
            projection.modeled_drag_work,
            projection.air_path_m,
        );
    }
    out.along = final_along - (*t).runway_along_track;
    out.cross = final_cross - (*t).runway_cross_track;
    out.course_error = norm_signed_deg(final_course - course);
    out.altitude_error = (*t).mean_altitude
        - clampd((*t).mean_altitude, (*s).mm305_min_altitude, (*s).mm305_max_altitude);
    out.minimum_turn_radius_m = minimum_turn_radius;
    out.path_length_m = minimum_path;
    out.required_time_s = control.control_response_time_s
        + minimum_path / (*t).horizontal_speed;
    out.available_time_s = if projection.valid as ::core::ffi::c_int != 0 {
        projection.travel_time_s
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.turn_margin = if minimum_path.is_finite() as i32 != 0 {
        0.0f64
    } else {
        -::core::f32::INFINITY as ::core::ffi::c_double
    };
    out.energy_available = if energy.valid as ::core::ffi::c_int != 0 {
        energy.available_specific_energy
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.energy_drag_work = if energy.valid as ::core::ffi::c_int != 0 {
        energy.modeled_drag_work
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.energy_uncertainty = if energy.valid as ::core::ffi::c_int != 0 {
        energy.uncertainty_specific_energy
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.energy_margin = if energy.valid as ::core::ffi::c_int != 0 {
        energy.energy.margin - projection.integration_error_j_kg
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    out.valid = out.along.is_finite() as i32 != 0 && out.cross.is_finite() as i32 != 0
        && out.minimum_turn_radius_m.is_finite() as i32 != 0
        && energy.valid as ::core::ffi::c_int != 0;
    if !out.valid {
        out.veto |= 16 as ::core::ffi::c_uint;
        return out;
    }
    let mut range: ::core::ffi::c_double = hypot(
        (*t).runway_along_track,
        (*t).runway_cross_track,
    );
    let mut maximum_entry_range: ::core::ffi::c_double = entry_taem_range_target(p, s);
    if range.is_finite() as i32 == 0 || maximum_entry_range.is_finite() as i32 == 0
        || range > maximum_entry_range
    {
        out.veto |= 2 as ::core::ffi::c_uint;
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
    if (*t).true_air_speed < (*v).minimum_safe_speed || mach.is_finite() as i32 == 0
        || mach < (*s).mm305_target_mach - (*s).mm305_mach_half_width
        || mach > (*s).mm305_target_mach + (*s).mm305_mach_half_width
    {
        out.veto |= 1 as ::core::ffi::c_uint;
    }
    if (*t).mean_altitude < (*s).mm305_min_altitude
        || (*t).mean_altitude > (*s).mm305_max_altitude
    {
        out.veto |= 8 as ::core::ffi::c_uint;
    }
    if !(out.energy_margin > 0.0f64) || !projection.reference_reachable {
        out.veto |= 16 as ::core::ffi::c_uint;
    }
    if !((*t).vertical_speed < 0.0f64) {
        out.veto |= 32 as ::core::ffi::c_uint;
    }
    if (*t).course_to_site_error.is_finite() as i32 != 0
        && fabs((*t).course_to_site_error) > 90.0f64
    {
        out.veto |= 128 as ::core::ffi::c_uint;
    }
    if (*t).dynamic_pressure > (*v).maximum_dynamic_pressure
        || (*t).g_force > (*v).maximum_g_load
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction >= 1.0f64
    {
        out.veto |= 64 as ::core::ffi::c_uint;
    }
    out.ready = out.veto == 0 as ::core::ffi::c_uint;
    return out;
}
unsafe extern "C" fn measured_or_modeled_lift_accel(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if t.is_null() || v.is_null() {
        return 0.0f64;
    }
    if (*t).mass > 0.0f64 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).lift_force > 0.0f64
    {
        return (*t).lift_force / (*t).mass;
    }
    let mut ballistic: ::core::ffi::c_double = if (*t)
        .estimated_ballistic_coefficient
        .is_finite() as i32 != 0 && (*t).estimated_ballistic_coefficient > 0.0f64
    {
        (*t).estimated_ballistic_coefficient
    } else {
        (*v).estimated_ballistic_coefficient
    };
    let mut lift_to_drag: ::core::ffi::c_double = if (*t)
        .estimated_lift_to_drag
        .is_finite() as i32 != 0 && (*t).estimated_lift_to_drag > 0.0f64
    {
        (*t).estimated_lift_to_drag
    } else {
        (*v).estimated_lift_to_drag
    };
    if !(ballistic > 0.0f64) || !(lift_to_drag > 0.0f64)
        || !((*t).dynamic_pressure > 0.0f64)
        || (*t).dynamic_pressure.is_finite() as i32 == 0
    {
        return 0.0f64;
    }
    let mut factor: ::core::ffi::c_double = lift_factor(
        (*t).mach,
        hypot((*t).angle_of_attack, (*t).sideslip),
        v,
    );
    return (*t).dynamic_pressure / ballistic * lift_to_drag * factor;
}
unsafe extern "C" fn lift_factor(
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut lift: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(
        mach,
        aoa,
        v,
        &raw mut lift,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    return fabs(lift);
}
unsafe extern "C" fn maximum_lift_aoa(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if t.is_null() || v.is_null() || !((*v).maximum_angle_of_attack > 0.0f64) {
        return 0.0f64;
    }
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let mut hi: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    let phi: ::core::ffi::c_double = (sqrt(5.0f64) as ::core::ffi::c_double - 1.0f64)
        * 0.5f64;
    let mut x1: ::core::ffi::c_double = hi - phi * (hi - lo);
    let mut x2: ::core::ffi::c_double = lo + phi * (hi - lo);
    let mut f1: ::core::ffi::c_double = lift_factor((*t).mach, x1, v);
    let mut f2: ::core::ffi::c_double = lift_factor((*t).mach, x2, v);
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
            f2 = lift_factor((*t).mach, x2, v);
        } else {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - phi * (hi - lo);
            f1 = lift_factor((*t).mach, x1, v);
        }
    }
    let mut aoa: ::core::ffi::c_double = 0.5f64 * (lo + hi);
    if (*t).mach < 1.0f64 && (*v).terminal_maximum_lift_angle_of_attack > 0.0f64 {
        aoa = fmin(aoa, (*v).terminal_maximum_lift_angle_of_attack);
    }
    return aoa;
}
unsafe extern "C" fn maximum_normal_accel(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut gravity: ::core::ffi::c_double,
    mut target_aoa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut current: ::core::ffi::c_double = measured_or_modeled_lift_accel(t, v);
    if t.is_null() || v.is_null() || !(current > 0.0f64) {
        return current;
    }
    let mut current_factor: ::core::ffi::c_double = lift_factor(
        (*t).mach,
        hypot((*t).angle_of_attack, (*t).sideslip),
        v,
    );
    let mut target_factor: ::core::ffi::c_double = lift_factor((*t).mach, target_aoa, v);
    let mut predicted: ::core::ffi::c_double = current;
    if current_factor > 0.25f64 * target_factor && target_factor.is_finite() as i32 != 0
    {
        predicted = current * target_factor / current_factor;
    } else {
        let mut stall: ::core::ffi::c_double = if (*t).calibrated_stall_speed.is_finite()
            as i32 != 0 && (*t).calibrated_stall_speed > 0.0f64
        {
            (*t).calibrated_stall_speed
        } else {
            (*v).minimum_safe_speed
        };
        if stall > 0.0f64 && (*t).true_air_speed.is_finite() as i32 != 0
            && (*t).true_air_speed > 0.0f64
        {
            predicted = gravity * ((*t).true_air_speed / stall)
                * ((*t).true_air_speed / stall);
        }
    }
    let mut structural: ::core::ffi::c_double = if (*v).maximum_g_load > 0.0f64 {
        (*v).maximum_g_load * gravity
    } else {
        predicted
    };
    return fmin(fmax(current, predicted), structural);
}
#[no_mangle]
pub unsafe extern "C" fn decision_guidance_attitude_limits(
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut phase: GuidancePhase,
    mut pitch_axis: bool,
) -> GuidanceAttitudeLimits {
    let mut limits: GuidanceAttitudeLimits = GuidanceAttitudeLimits {
        rate_deg_s: fabs(if !s.is_null() { (*s).entry_roll_rate } else { 0.0f64 }),
        accel_deg_s2: fabs(
            if !s.is_null() { (*s).entry_roll_acceleration } else { 0.0f64 },
        ),
    };
    if pitch_axis {
        if !t.is_null() && (*t).attitude_response.pitch_valid as ::core::ffi::c_int != 0
        {
            limits.rate_deg_s = fmin(
                limits.rate_deg_s,
                fabs((*t).attitude_response.maximum_pitch_rate_deg_s),
            );
            limits.accel_deg_s2 = fmin(
                limits.accel_deg_s2,
                fabs((*t).attitude_response.maximum_pitch_accel_deg_s2),
            );
        }
    } else {
        if phase as ::core::ffi::c_uint
            == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
            || phase as ::core::ffi::c_uint
                == PHASE_HEADING_ALIGNMENT as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            limits.rate_deg_s = fabs(
                if !s.is_null() { (*s).taem_roll_rate } else { 0.0f64 },
            );
            limits.accel_deg_s2 = fabs(
                if !s.is_null() {
                    (*s).entry_roll_acceleration * 0.85f64
                } else {
                    0.0f64
                },
            );
        } else if phase as ::core::ffi::c_uint
            == PHASE_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint
            || phase as ::core::ffi::c_uint
                == PHASE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            limits.rate_deg_s = fabs(
                if !s.is_null() { (*s).approach_roll_rate } else { 0.0f64 },
            );
            limits.accel_deg_s2 = fabs(
                if !s.is_null() {
                    (*s).entry_roll_acceleration * 0.55f64
                } else {
                    0.0f64
                },
            );
        }
        if !t.is_null() && (*t).attitude_response.roll_valid as ::core::ffi::c_int != 0 {
            limits.rate_deg_s = fmin(
                limits.rate_deg_s,
                fabs((*t).attitude_response.maximum_roll_rate_deg_s),
            );
            limits.accel_deg_s2 = fmin(
                limits.accel_deg_s2,
                fabs((*t).attitude_response.maximum_roll_accel_deg_s2),
            );
        }
    }
    limits.rate_deg_s = if limits.rate_deg_s.is_finite() as i32 != 0
        && limits.rate_deg_s > 0.0f64
    {
        limits.rate_deg_s
    } else {
        1.0f64
    };
    limits.accel_deg_s2 = if limits.accel_deg_s2.is_finite() as i32 != 0
        && limits.accel_deg_s2 > 0.0f64
    {
        limits.accel_deg_s2
    } else {
        1.0f64
    };
    return limits;
}
unsafe extern "C" fn control_response_time(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut target_aoa: ::core::ffi::c_double,
    mut final_0: bool,
) -> ::core::ffi::c_double {
    if t.is_null() || s.is_null() || (*t).roll.is_finite() as i32 == 0
        || (*t).roll_rate.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut roll_rate: ::core::ffi::c_double = (*s).approach_roll_rate;
    let mut roll_accel: ::core::ffi::c_double = (*s).entry_roll_acceleration;
    if (*t).attitude_response.roll_valid {
        roll_rate = (*t).attitude_response.maximum_roll_rate_deg_s;
        roll_accel = (*t).attitude_response.maximum_roll_accel_deg_s2;
    }
    let mut roll_time: ::core::ffi::c_double = decision_axis_capture_time(
        -norm_signed_deg((*t).roll),
        (*t).roll_rate,
        roll_accel,
        roll_rate,
    );
    let mut pitch_time: ::core::ffi::c_double = decision_pitch_capture_time(
        t,
        target_aoa,
    );
    if final_0 {
        let mut pitch: GuidanceAttitudeLimits = decision_guidance_attitude_limits(
            t,
            s,
            PHASE_FINAL,
            true_0 != 0,
        );
        let mut roll: GuidanceAttitudeLimits = decision_guidance_attitude_limits(
            t,
            s,
            PHASE_FINAL,
            false_0 != 0,
        );
        let mut aoa_rate: ::core::ffi::c_double = if (*t).has_angle_of_attack_rate
            as ::core::ffi::c_int != 0
        {
            (*t).angle_of_attack_rate
        } else {
            0.0f64
        };
        let mut pitch_reference: ::core::ffi::c_double = decision_axis_capture_time(
            target_aoa - (*t).angle_of_attack,
            aoa_rate,
            pitch.accel_deg_s2,
            pitch.rate_deg_s,
        );
        let mut roll_reference: ::core::ffi::c_double = decision_axis_capture_time(
            -norm_signed_deg((*t).roll),
            (*t).roll_rate,
            roll.accel_deg_s2,
            roll.rate_deg_s,
        );
        if !g.is_null() && (*g).pitch_limiter.has_value as ::core::ffi::c_int != 0 {
            pitch_reference = fmax(
                pitch_reference,
                decision_axis_capture_time(
                    target_aoa - (*g).pitch_limiter.value,
                    (*g).pitch_limiter.rate,
                    pitch.accel_deg_s2,
                    pitch.rate_deg_s,
                ),
            );
        }
        if !g.is_null() && (*g).roll_limiter.has_value as ::core::ffi::c_int != 0 {
            roll_reference = fmax(
                roll_reference,
                decision_axis_capture_time(
                    -norm_signed_deg((*g).roll_limiter.value),
                    (*g).roll_limiter.rate,
                    roll.accel_deg_s2,
                    roll.rate_deg_s,
                ),
            );
        }
        pitch_time = fmax(pitch_time, pitch_reference);
        roll_time = fmax(roll_time, roll_reference);
    }
    return fmax(pitch_time, roll_time);
}
#[no_mangle]
pub unsafe extern "C" fn decision_final_approach_response_time(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut target_aoa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return control_response_time(g, t, s, target_aoa, true_0 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn decision_control_authority_envelope(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> ControlAuthorityEnvelope {
    let mut out: ControlAuthorityEnvelope = ControlAuthorityEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        survivable: false,
        controllable: false,
        speed: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        dynamic_pressure: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        load: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        stall: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        stall_observed: false,
        minimum_speed_mps: 0.,
        maximum_lift_aoa_deg: 0.,
        maximum_normal_accel_mps2: 0.,
        maximum_lateral_accel_mps2: 0.,
        control_response_time_s: 0.,
    };
    if t.is_null() || p.is_null() || cfg.is_null() {
        return out;
    }
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
    if !(gravity > 0.0f64) {
        return out;
    }
    let mut minimum_speed: ::core::ffi::c_double = (*v).minimum_safe_speed;
    if (*t).calibrated_stall_speed.is_finite() as i32 != 0
        && (*t).calibrated_stall_speed > 0.0f64
    {
        minimum_speed = fmax(minimum_speed, (*t).calibrated_stall_speed);
    }
    out.speed = decision_margin((*t).true_air_speed, minimum_speed);
    out.dynamic_pressure = decision_margin(
        (*v).maximum_dynamic_pressure,
        (*t).dynamic_pressure,
    );
    out.load = decision_margin((*v).maximum_g_load, (*t).g_force);
    out.stall_observed = (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
        && (*t).stall_fraction.is_finite() as i32 != 0;
    if out.stall_observed {
        out.stall = decision_margin(1.0f64, (*t).stall_fraction);
    }
    out.minimum_speed_mps = minimum_speed;
    out.maximum_lift_aoa_deg = maximum_lift_aoa(t, v);
    out.maximum_normal_accel_mps2 = maximum_normal_accel(
        t,
        v,
        gravity,
        out.maximum_lift_aoa_deg,
    );
    let mut bank_effectiveness: ::core::ffi::c_double = if (*t)
        .bank_effectiveness
        .is_finite() as i32 != 0 && (*t).bank_effectiveness > 0.0f64
    {
        (*t).bank_effectiveness
    } else {
        1.0f64
    };
    let mut bank_limit_rad: ::core::ffi::c_double = fabs((*v).maximum_bank_angle)
        * bank_effectiveness * DEG2RAD;
    bank_limit_rad = fmin(bank_limit_rad, nextafter(LANDER_PI * 0.5f64, 0.0f64));
    out.maximum_lateral_accel_mps2 = out.maximum_normal_accel_mps2
        * fabs(sin(bank_limit_rad));
    let mut final_0: bool = !g.is_null()
        && ((*g).final_approach_captured as ::core::ffi::c_int != 0
            || (*g).phase as ::core::ffi::c_uint
                == PHASE_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).phase as ::core::ffi::c_uint
                == PHASE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).terminal_vertical_stage_valid as ::core::ffi::c_int != 0);
    out.control_response_time_s = control_response_time(
        g,
        t,
        s,
        out.maximum_lift_aoa_deg,
        final_0,
    );
    out.valid = out.speed.valid as ::core::ffi::c_int != 0
        && out.dynamic_pressure.valid as ::core::ffi::c_int != 0
        && out.load.valid as ::core::ffi::c_int != 0
        && out.maximum_lift_aoa_deg.is_finite() as i32 != 0
        && out.maximum_normal_accel_mps2.is_finite() as i32 != 0
        && out.maximum_lateral_accel_mps2.is_finite() as i32 != 0
        && out.control_response_time_s.is_finite() as i32 != 0;
    out.survivable = out.valid as ::core::ffi::c_int != 0 && out.speed.margin >= 0.0f64
        && out.dynamic_pressure.margin >= 0.0f64 && out.load.margin >= 0.0f64
        && (!out.stall_observed
            || out.stall.valid as ::core::ffi::c_int != 0 && out.stall.margin >= 0.0f64);
    out.controllable = out.survivable as ::core::ffi::c_int != 0
        && out.maximum_normal_accel_mps2 > 0.0f64
        && out.maximum_lateral_accel_mps2 > 0.0f64;
    return out;
}
unsafe extern "C" fn mod_two_pi(mut x: ::core::ffi::c_double) -> ::core::ffi::c_double {
    let mut two_pi: ::core::ffi::c_double = 2.0f64 * LANDER_PI;
    let mut y: ::core::ffi::c_double = fmod(x, two_pi);
    return if y < 0.0f64 { y + two_pi } else { y };
}
unsafe extern "C" fn dubins_lsl(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut d: ::core::ffi::c_double,
    mut out: *mut ::core::ffi::c_double,
) -> bool {
    let mut tmp: ::core::ffi::c_double = d + sin(a) - sin(b);
    let mut p2: ::core::ffi::c_double = 2.0f64 + d * d - 2.0f64 * cos(a - b)
        + 2.0f64 * d * (sin(a) - sin(b));
    if p2 < 0.0f64 {
        return false_0 != 0;
    }
    let mut p: ::core::ffi::c_double = sqrt(fmax(0.0f64, p2));
    let mut x: ::core::ffi::c_double = atan2(cos(b) - cos(a), tmp);
    *out = mod_two_pi(-a + x) + p + mod_two_pi(b - x);
    return true_0 != 0;
}
unsafe extern "C" fn dubins_rsr(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut d: ::core::ffi::c_double,
    mut out: *mut ::core::ffi::c_double,
) -> bool {
    let mut tmp: ::core::ffi::c_double = d - sin(a) + sin(b);
    let mut p2: ::core::ffi::c_double = 2.0f64 + d * d - 2.0f64 * cos(a - b)
        + 2.0f64 * d * (-sin(a) + sin(b));
    if p2 < 0.0f64 {
        return false_0 != 0;
    }
    let mut p: ::core::ffi::c_double = sqrt(fmax(0.0f64, p2));
    let mut x: ::core::ffi::c_double = atan2(cos(a) - cos(b), tmp);
    *out = mod_two_pi(a - x) + p + mod_two_pi(-b + x);
    return true_0 != 0;
}
unsafe extern "C" fn dubins_lsr(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut d: ::core::ffi::c_double,
    mut out: *mut ::core::ffi::c_double,
) -> bool {
    let mut p2: ::core::ffi::c_double = -2.0f64 + d * d + 2.0f64 * cos(a - b)
        + 2.0f64 * d * (sin(a) + sin(b));
    if p2 < 0.0f64 {
        return false_0 != 0;
    }
    let mut p: ::core::ffi::c_double = sqrt(fmax(0.0f64, p2));
    let mut x: ::core::ffi::c_double = atan2(-cos(a) - cos(b), d + sin(a) + sin(b))
        - atan2(-2.0f64, p);
    *out = mod_two_pi(-a + x) + p + mod_two_pi(-b + x);
    return true_0 != 0;
}
unsafe extern "C" fn dubins_rsl(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut d: ::core::ffi::c_double,
    mut out: *mut ::core::ffi::c_double,
) -> bool {
    let mut p2: ::core::ffi::c_double = d * d - 2.0f64 + 2.0f64 * cos(a - b)
        - 2.0f64 * d * (sin(a) + sin(b));
    if p2 < 0.0f64 {
        return false_0 != 0;
    }
    let mut p: ::core::ffi::c_double = sqrt(fmax(0.0f64, p2));
    let mut x: ::core::ffi::c_double = atan2(cos(a) + cos(b), d - sin(a) - sin(b))
        - atan2(2.0f64, p);
    *out = mod_two_pi(a - x) + p + mod_two_pi(b - x);
    return true_0 != 0;
}
unsafe extern "C" fn dubins_rlr(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut d: ::core::ffi::c_double,
    mut out: *mut ::core::ffi::c_double,
) -> bool {
    let mut x: ::core::ffi::c_double = (6.0f64 - d * d + 2.0f64 * cos(a - b)
        + 2.0f64 * d * (sin(a) - sin(b))) / 8.0f64;
    if x < -1.0f64 || x > 1.0f64 {
        return false_0 != 0;
    }
    let mut p: ::core::ffi::c_double = mod_two_pi(
        2.0f64 * LANDER_PI - acos(clampd(x, -1.0f64, 1.0f64)),
    );
    let mut t: ::core::ffi::c_double = mod_two_pi(
        a - atan2(cos(a) - cos(b), d - sin(a) + sin(b)) + p * 0.5f64,
    );
    let mut q: ::core::ffi::c_double = mod_two_pi(a - b - t + p);
    *out = t + p + q;
    return true_0 != 0;
}
unsafe extern "C" fn dubins_lrl(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut d: ::core::ffi::c_double,
    mut out: *mut ::core::ffi::c_double,
) -> bool {
    let mut x: ::core::ffi::c_double = (6.0f64 - d * d + 2.0f64 * cos(a - b)
        + 2.0f64 * d * (-sin(a) + sin(b))) / 8.0f64;
    if x < -1.0f64 || x > 1.0f64 {
        return false_0 != 0;
    }
    let mut p: ::core::ffi::c_double = mod_two_pi(
        2.0f64 * LANDER_PI - acos(clampd(x, -1.0f64, 1.0f64)),
    );
    let mut t: ::core::ffi::c_double = mod_two_pi(
        -a - atan2(cos(a) - cos(b), d + sin(a) - sin(b)) + p * 0.5f64,
    );
    let mut q: ::core::ffi::c_double = mod_two_pi(b - a - t + p);
    *out = t + p + q;
    return true_0 != 0;
}
unsafe extern "C" fn shortest_dubins_path(
    mut de: ::core::ffi::c_double,
    mut dn: ::core::ffi::c_double,
    mut start_course: ::core::ffi::c_double,
    mut end_course: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if de.is_finite() as i32 == 0 || dn.is_finite() as i32 == 0
        || start_course.is_finite() as i32 == 0 || end_course.is_finite() as i32 == 0
        || !(radius > 0.0f64) || radius.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut distance: ::core::ffi::c_double = hypot(de, dn);
    if distance <= DBL_EPSILON
        && fabs(norm_signed_deg((end_course - start_course) * RAD2DEG))
            <= sqrt(DBL_EPSILON) * RAD2DEG
    {
        return 0.0f64;
    }
    let mut d: ::core::ffi::c_double = distance / radius;
    let mut theta: ::core::ffi::c_double = atan2(de, dn);
    let mut a: ::core::ffi::c_double = mod_two_pi(start_course - theta);
    let mut b: ::core::ffi::c_double = mod_two_pi(end_course - theta);
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut candidate: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    if dubins_lsl(a, b, d, &raw mut candidate) {
        best = fmin(best, candidate);
    }
    if dubins_rsr(a, b, d, &raw mut candidate) {
        best = fmin(best, candidate);
    }
    if dubins_lsr(a, b, d, &raw mut candidate) {
        best = fmin(best, candidate);
    }
    if dubins_rsl(a, b, d, &raw mut candidate) {
        best = fmin(best, candidate);
    }
    if dubins_rlr(a, b, d, &raw mut candidate) {
        best = fmin(best, candidate);
    }
    if dubins_lrl(a, b, d, &raw mut candidate) {
        best = fmin(best, candidate);
    }
    return if best.is_finite() as i32 != 0 {
        best * radius
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn decision_target_path_geometry(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut target_along_m: ::core::ffi::c_double,
    mut target_cross_m: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
    mut lateral_accel_limit_mps2: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut control_out: *mut ControlAuthorityEnvelope,
    mut range_out: *mut ::core::ffi::c_double,
    mut forward_out: *mut ::core::ffi::c_double,
    mut cross_out: *mut ::core::ffi::c_double,
    mut course_error_out: *mut ::core::ffi::c_double,
    mut minimum_turn_radius_out: *mut ::core::ffi::c_double,
    mut path_length_out: *mut ::core::ffi::c_double,
) -> bool {
    if t.is_null() || p.is_null() || cfg.is_null() || course_deg.is_finite() as i32 == 0
        || target_along_m.is_finite() as i32 == 0
        || target_cross_m.is_finite() as i32 == 0
        || target_course_deg.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut control: ControlAuthorityEnvelope = decision_control_authority_envelope(
        g,
        t,
        p,
        cfg,
    );
    if !control.valid || !((*t).horizontal_speed > DBL_MIN)
        || (*t).horizontal_speed.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut lateral: ::core::ffi::c_double = control.maximum_lateral_accel_mps2;
    if lateral_accel_limit_mps2.is_finite() as i32 != 0 {
        lateral = fmin(lateral, fmax(0.0f64, lateral_accel_limit_mps2));
    }
    if !(lateral > DBL_MIN) || lateral.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut da: ::core::ffi::c_double = target_along_m - (*t).runway_along_track;
    let mut dc: ::core::ffi::c_double = target_cross_m - (*t).runway_cross_track;
    let mut runway: ::core::ffi::c_double = (*cfg).site.runway_heading * DEG2RAD;
    let mut de: ::core::ffi::c_double = da * sin(runway) + dc * cos(runway);
    let mut dn: ::core::ffi::c_double = da * cos(runway) - dc * sin(runway);
    let mut target_course: ::core::ffi::c_double = target_course_deg * DEG2RAD;
    let mut minimum_turn: ::core::ffi::c_double = (*t).horizontal_speed
        * (*t).horizontal_speed / lateral;
    let mut path: ::core::ffi::c_double = shortest_dubins_path(
        de,
        dn,
        course_deg * DEG2RAD,
        target_course,
        minimum_turn,
    );
    if path.is_finite() as i32 == 0 || minimum_turn.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    if !control_out.is_null() {
        *control_out = control;
    }
    if !range_out.is_null() {
        *range_out = hypot(de, dn);
    }
    if !forward_out.is_null() {
        *forward_out = de * sin(target_course) + dn * cos(target_course);
    }
    if !cross_out.is_null() {
        *cross_out = -de * cos(target_course) + dn * sin(target_course);
    }
    if !course_error_out.is_null() {
        *course_error_out = norm_signed_deg(target_course_deg - course_deg);
    }
    if !minimum_turn_radius_out.is_null() {
        *minimum_turn_radius_out = minimum_turn;
    }
    if !path_length_out.is_null() {
        *path_length_out = path;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn decision_runway_end_path_score(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut site: *const LandingSite,
) -> ::core::ffi::c_double {
    if g.is_null() || t.is_null() || p.is_null() || cfg.is_null() || site.is_null() {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut candidate: LandingConfiguration = *cfg;
    candidate.site = *site;
    let mut framed: Telemetry = *t;
    telemetry_reframe_runway(&raw mut framed, site, (*p).radius);
    let mut course: ::core::ffi::c_double = if framed.ground_track_heading.is_finite()
        as i32 != 0
    {
        framed.ground_track_heading
    } else {
        framed.heading
    };
    if course.is_finite() as i32 == 0 {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut minimum_turn_radius: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut path: ::core::ffi::c_double = decision_target_path_length(
        g,
        &raw mut framed,
        course,
        -candidate.guidance.final_approach_distance,
        0.0f64,
        candidate.site.runway_heading,
        ::core::f32::NAN as ::core::ffi::c_double,
        p,
        &raw mut candidate,
        &raw mut minimum_turn_radius,
    );
    return if path.is_finite() as i32 != 0 {
        path
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
#[no_mangle]
pub unsafe extern "C" fn decision_target_path_length(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut target_along_m: ::core::ffi::c_double,
    mut target_cross_m: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
    mut lateral_accel_limit_mps2: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut minimum_turn_radius_m: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut path: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    if !decision_target_path_geometry(
        g,
        t,
        course_deg,
        target_along_m,
        target_cross_m,
        target_course_deg,
        lateral_accel_limit_mps2,
        p,
        cfg,
        ::core::ptr::null_mut::<ControlAuthorityEnvelope>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        minimum_turn_radius_m,
        &raw mut path,
    ) {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return path;
}
unsafe extern "C" fn decision_target_capture_envelope_impl(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut target_along_m: ::core::ffi::c_double,
    mut target_cross_m: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
    mut available_time_s: ::core::ffi::c_double,
    mut lateral_accel_limit_mps2: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> TargetCaptureEnvelope {
    let mut out: TargetCaptureEnvelope = TargetCaptureEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        reachable: false,
        control: ControlAuthorityEnvelope {
            valid: false,
            survivable: false,
            controllable: false,
            speed: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            dynamic_pressure: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            load: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            stall: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            stall_observed: false,
            minimum_speed_mps: 0.,
            maximum_lift_aoa_deg: 0.,
            maximum_normal_accel_mps2: 0.,
            maximum_lateral_accel_mps2: 0.,
            control_response_time_s: 0.,
        },
        capture_time: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        forward_m: 0.,
        cross_m: 0.,
        range_m: 0.,
        minimum_turn_radius_m: 0.,
        path_length_m: 0.,
        time_available_s: 0.,
        required_time_s: 0.,
        lateral_capture_time_s: 0.,
        heading_capture_time_s: 0.,
        course_error_deg: 0.,
    };
    if t.is_null() || p.is_null() || cfg.is_null() || course_deg.is_finite() as i32 == 0
        || target_along_m.is_finite() as i32 == 0
        || target_cross_m.is_finite() as i32 == 0
        || target_course_deg.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
    {
        return out;
    }
    if !decision_target_path_geometry(
        g,
        t,
        course_deg,
        target_along_m,
        target_cross_m,
        target_course_deg,
        lateral_accel_limit_mps2,
        p,
        cfg,
        &raw mut out.control,
        &raw mut out.range_m,
        &raw mut out.forward_m,
        &raw mut out.cross_m,
        &raw mut out.course_error_deg,
        &raw mut out.minimum_turn_radius_m,
        &raw mut out.path_length_m,
    ) {
        return out;
    }
    let mut lateral: ::core::ffi::c_double = (*t).horizontal_speed
        * (*t).horizontal_speed / out.minimum_turn_radius_m;
    out.required_time_s = if out.path_length_m.is_finite() as i32 != 0 {
        out.control.control_response_time_s + out.path_length_m / (*t).horizontal_speed
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    out.time_available_s = available_time_s;
    out.capture_time = decision_margin(available_time_s, out.required_time_s);
    let mut course_rate: ::core::ffi::c_double = lateral / (*t).horizontal_speed;
    out.heading_capture_time_s = if course_rate > 0.0f64 {
        out.control.control_response_time_s
            + fabs(out.course_error_deg * DEG2RAD) / course_rate
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut cross_rate: ::core::ffi::c_double = (*t).horizontal_speed
        * sin(norm_signed_deg(course_deg - target_course_deg) * DEG2RAD);
    out.lateral_capture_time_s = decision_bounded_capture_time(
        out.cross_m,
        cross_rate,
        lateral,
        out.control.control_response_time_s,
    );
    out.valid = out.control.valid as ::core::ffi::c_int != 0
        && out.capture_time.valid as ::core::ffi::c_int != 0
        && out.path_length_m.is_finite() as i32 != 0
        && out.required_time_s.is_finite() as i32 != 0;
    out.reachable = out.valid as ::core::ffi::c_int != 0
        && out.control.controllable as ::core::ffi::c_int != 0
        && out.capture_time.margin >= 0.0f64;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_target_capture_envelope(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut target_along_m: ::core::ffi::c_double,
    mut target_cross_m: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
    mut available_time_s: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> TargetCaptureEnvelope {
    return decision_target_capture_envelope_impl(
        g,
        t,
        course_deg,
        target_along_m,
        target_cross_m,
        target_course_deg,
        available_time_s,
        ::core::f32::NAN as ::core::ffi::c_double,
        p,
        cfg,
    );
}
#[no_mangle]
pub unsafe extern "C" fn decision_target_capture_envelope_with_lateral_accel(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut target_along_m: ::core::ffi::c_double,
    mut target_cross_m: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
    mut available_time_s: ::core::ffi::c_double,
    mut lateral_accel_limit_mps2: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> TargetCaptureEnvelope {
    return decision_target_capture_envelope_impl(
        g,
        t,
        course_deg,
        target_along_m,
        target_cross_m,
        target_course_deg,
        available_time_s,
        lateral_accel_limit_mps2,
        p,
        cfg,
    );
}
unsafe extern "C" fn finite_positive(
    mut value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 && value > 0.0f64 { value } else { 0.0f64 };
}
#[no_mangle]
pub unsafe extern "C" fn decision_margin(
    mut available: ::core::ffi::c_double,
    mut required: ::core::ffi::c_double,
) -> DecisionMargin {
    let mut out: DecisionMargin = DecisionMargin {
        valid: false,
        available: 0.,
        required: 0.,
        margin: ::core::f32::NAN as ::core::ffi::c_double,
        normalized_margin: ::core::f32::NAN as ::core::ffi::c_double,
    };
    if available.is_finite() as i32 == 0 || required.is_finite() as i32 == 0 {
        return out;
    }
    out.available = available;
    out.required = required;
    out.margin = available - required;
    if out.margin.is_finite() as i32 == 0 {
        return out;
    }
    let mut scale: ::core::ffi::c_double = fmax(fabs(available), fabs(required));
    out.normalized_margin = if scale > DBL_MIN {
        out.margin / scale
    } else {
        out.margin
    };
    out.valid = out.normalized_margin.is_finite() as i32 != 0;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_vertical_recovery_envelope(
    mut available_height_m: ::core::ffi::c_double,
    mut sink_mps: ::core::ffi::c_double,
    mut target_sink_mps: ::core::ffi::c_double,
    mut response_time_s: ::core::ffi::c_double,
    mut response_down_accel_mps2: ::core::ffi::c_double,
    mut recovery_up_accel_mps2: ::core::ffi::c_double,
) -> VerticalRecoveryEnvelope {
    let mut out: VerticalRecoveryEnvelope = VerticalRecoveryEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        reachable: false,
        height: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        response_height_m: 0.,
        braking_height_m: 0.,
        required_height_m: 0.,
        sink_after_response_mps: 0.,
        recovery_time_s: 0.,
    };
    out.height = decision_margin(
        ::core::f32::NAN as ::core::ffi::c_double,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    if available_height_m.is_finite() as i32 == 0 || sink_mps.is_finite() as i32 == 0
        || sink_mps < 0.0f64 || target_sink_mps.is_finite() as i32 == 0
        || target_sink_mps < 0.0f64 || response_time_s.is_finite() as i32 == 0
        || response_time_s < 0.0f64 || response_down_accel_mps2.is_finite() as i32 == 0
        || response_down_accel_mps2 < 0.0f64
        || recovery_up_accel_mps2.is_finite() as i32 == 0
        || recovery_up_accel_mps2 < 0.0f64
    {
        return out;
    }
    let mut sink: ::core::ffi::c_double = sink_mps
        + response_down_accel_mps2 * response_time_s;
    let mut response_height: ::core::ffi::c_double = sink_mps * response_time_s
        + 0.5f64 * response_down_accel_mps2 * response_time_s * response_time_s;
    let mut braking_height: ::core::ffi::c_double = 0.0f64;
    let mut braking_time: ::core::ffi::c_double = 0.0f64;
    if sink > target_sink_mps {
        if !(recovery_up_accel_mps2 > 0.0f64) {
            return out;
        }
        braking_height = (sink - target_sink_mps) * (sink + target_sink_mps)
            / (2.0f64 * recovery_up_accel_mps2);
        braking_time = (sink - target_sink_mps) / recovery_up_accel_mps2;
    }
    out.response_height_m = response_height;
    out.braking_height_m = braking_height;
    out.required_height_m = response_height + braking_height;
    out.sink_after_response_mps = sink;
    out.recovery_time_s = response_time_s + braking_time;
    out.height = decision_margin(available_height_m, out.required_height_m);
    out.valid = out.height.valid as ::core::ffi::c_int != 0
        && sink.is_finite() as i32 != 0 && out.recovery_time_s.is_finite() as i32 != 0;
    out.reachable = out.valid as ::core::ffi::c_int != 0 && out.height.margin >= 0.0f64;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_mm304_course_contract_ready(
    mut course_deg: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
    mut cfg: *const LandingConfiguration,
) -> bool {
    if cfg.is_null() || course_deg.is_finite() as i32 == 0
        || target_course_deg.is_finite() as i32 == 0
        || (*cfg).site.runway_heading.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut handoff: TaemHandoffContract = taem_handoff_contract(
        &raw const (*cfg).guidance,
    );
    if !(handoff.perpendicular_heading_half_width_deg >= 0.0f64) {
        return false_0 != 0;
    }
    let mut tolerance: ::core::ffi::c_double = handoff
        .perpendicular_heading_half_width_deg;
    let mut resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON) * 180.0f64;
    let mut actual_relative: ::core::ffi::c_double = fabs(
        norm_signed_deg(course_deg - (*cfg).site.runway_heading),
    );
    let mut target_relative: ::core::ffi::c_double = fabs(
        norm_signed_deg(target_course_deg - (*cfg).site.runway_heading),
    );
    let mut target_error: ::core::ffi::c_double = fabs(
        norm_signed_deg(target_course_deg - course_deg),
    );
    return actual_relative.is_finite() as i32 != 0
        && target_relative.is_finite() as i32 != 0
        && target_error.is_finite() as i32 != 0
        && fabs(actual_relative - 90.0f64) <= tolerance + resolution
        && fabs(target_relative - 90.0f64) <= tolerance + resolution
        && target_error <= tolerance + resolution;
}
#[no_mangle]
pub unsafe extern "C" fn decision_mm304_handoff_set_envelope(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut center_along_m: ::core::ffi::c_double,
    mut center_cross_m: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> HandoffSetEnvelope {
    let mut out: HandoffSetEnvelope = HandoffSetEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        position_ready: false,
        heading_ready: false,
        inside: false,
        control: ControlAuthorityEnvelope {
            valid: false,
            survivable: false,
            controllable: false,
            speed: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            dynamic_pressure: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            load: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            stall: DecisionMargin {
                valid: false,
                available: 0.,
                required: 0.,
                margin: 0.,
                normalized_margin: 0.,
            },
            stall_observed: false,
            minimum_speed_mps: 0.,
            maximum_lift_aoa_deg: 0.,
            maximum_normal_accel_mps2: 0.,
            maximum_lateral_accel_mps2: 0.,
            control_response_time_s: 0.,
        },
        position: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        heading: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        center_distance_m: 0.,
        heading_error_deg: 0.,
        minimum_turn_radius_m: 0.,
        path_to_set_m: 0.,
    };
    if t.is_null() || p.is_null() || cfg.is_null() || course_deg.is_finite() as i32 == 0
        || center_along_m.is_finite() as i32 == 0
        || center_cross_m.is_finite() as i32 == 0
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
    {
        return out;
    }
    let mut contract: TaemHandoffContract = taem_handoff_contract(
        &raw const (*cfg).guidance,
    );
    if !(contract.horizontal_radius_m > 0.0f64)
        || !(contract.perpendicular_heading_half_width_deg >= 0.0f64)
    {
        return out;
    }
    out.control = decision_control_authority_envelope(g, t, p, cfg);
    let mut da: ::core::ffi::c_double = center_along_m - (*t).runway_along_track;
    let mut dc: ::core::ffi::c_double = center_cross_m - (*t).runway_cross_track;
    out.center_distance_m = hypot(da, dc);
    out.position = decision_margin(contract.horizontal_radius_m, out.center_distance_m);
    out.position_ready = out.position.valid as ::core::ffi::c_int != 0
        && out.position.margin >= 0.0f64;
    let perpendicular: ::core::ffi::c_double = 90.0f64;
    let mut center_a: ::core::ffi::c_double = norm_deg(
        (*cfg).site.runway_heading - perpendicular,
    );
    let mut center_b: ::core::ffi::c_double = norm_deg(
        (*cfg).site.runway_heading + perpendicular,
    );
    let mut error_a: ::core::ffi::c_double = fabs(
        norm_signed_deg(course_deg - center_a),
    );
    let mut error_b: ::core::ffi::c_double = fabs(
        norm_signed_deg(course_deg - center_b),
    );
    let mut center_error: ::core::ffi::c_double = fmin(error_a, error_b);
    out.heading_error_deg = fmax(
        0.0f64,
        center_error - contract.perpendicular_heading_half_width_deg,
    );
    out.heading = decision_margin(
        contract.perpendicular_heading_half_width_deg,
        center_error,
    );
    out.heading_ready = out.heading.valid as ::core::ffi::c_int != 0
        && out.heading.margin >= 0.0f64;
    let mut lateral: ::core::ffi::c_double = out.control.maximum_lateral_accel_mps2;
    out.minimum_turn_radius_m = if out.control.valid as ::core::ffi::c_int != 0
        && lateral > DBL_MIN && (*t).horizontal_speed > DBL_MIN
    {
        (*t).horizontal_speed * (*t).horizontal_speed / lateral
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut position_excess: ::core::ffi::c_double = fmax(
        0.0f64,
        out.center_distance_m - contract.horizontal_radius_m,
    );
    let mut turn_path: ::core::ffi::c_double = if out.minimum_turn_radius_m.is_finite()
        as i32 != 0
    {
        out.minimum_turn_radius_m * out.heading_error_deg * DEG2RAD
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    out.path_to_set_m = if turn_path.is_finite() as i32 != 0 {
        position_excess + turn_path
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    out.inside = out.position_ready as ::core::ffi::c_int != 0
        && out.heading_ready as ::core::ffi::c_int != 0;
    out.valid = out.position.valid as ::core::ffi::c_int != 0
        && out.heading.valid as ::core::ffi::c_int != 0
        && out.control.valid as ::core::ffi::c_int != 0
        && out.path_to_set_m.is_finite() as i32 != 0;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_taem_speed_envelope(
    mut v: *const VehicleProfile,
    mut p: *const PlanetModel,
    mut altitude_m: ::core::ffi::c_double,
) -> TaemSpeedEnvelope {
    let mut out: TaemSpeedEnvelope = TaemSpeedEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        feasible: false,
        minimum_speed_mps: 0.,
        maximum_speed_mps: 0.,
        density_kg_m3: 0.,
        dynamic_pressure_limit_pa: 0.,
        load_pressure_limit_pa: 0.,
        minimum_turn_radius_m: 0.,
    };
    if v.is_null() || p.is_null() || altitude_m.is_finite() as i32 == 0
        || !((*v).minimum_safe_speed > 0.0f64) || !((*v).final_approach_speed > 0.0f64)
        || !((*v).maximum_dynamic_pressure > 0.0f64) || !((*v).maximum_g_load > 0.0f64)
        || !((*v).estimated_ballistic_coefficient > 0.0f64)
        || !((*v).estimated_lift_to_drag > 0.0f64)
    {
        return out;
    }
    let mut density: ::core::ffi::c_double = planet_atmospheric_density(p, altitude_m);
    let mut radius: ::core::ffi::c_double = (*p).radius + altitude_m;
    if !(density > DBL_MIN) || !(radius > DBL_MIN)
        || !((*p).gravitational_parameter > 0.0f64)
    {
        return out;
    }
    let mut gravity: ::core::ffi::c_double = (*p).gravitational_parameter
        / (radius * radius);
    let mut ld: ::core::ffi::c_double = fabs((*v).estimated_lift_to_drag);
    let mut beta: ::core::ffi::c_double = (*v).estimated_ballistic_coefficient;
    let mut aerodynamic_ratio: ::core::ffi::c_double = sqrt(1.0f64 + ld * ld);
    let mut load_q: ::core::ffi::c_double = (*v).maximum_g_load * gravity * beta
        / aerodynamic_ratio;
    let mut q_limit: ::core::ffi::c_double = fmin((*v).maximum_dynamic_pressure, load_q);
    if !(q_limit > 0.0f64) {
        return out;
    }
    let mut minimum: ::core::ffi::c_double = fmax(
        (*v).minimum_safe_speed,
        (*v).final_approach_speed,
    );
    let mut maximum: ::core::ffi::c_double = sqrt(2.0f64 * q_limit / density);
    let mut effective_bank: ::core::ffi::c_double = fmin(
        fabs((*v).maximum_bank_angle),
        nextafter(90.0f64, 0.0f64),
    );
    let mut lateral_per_speed2: ::core::ffi::c_double = 0.5f64 * density / beta * ld
        * fabs(sin(effective_bank * DEG2RAD));
    let mut minimum_turn: ::core::ffi::c_double = if lateral_per_speed2 > DBL_MIN {
        1.0f64 / lateral_per_speed2
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    out.valid = minimum.is_finite() as i32 != 0 && maximum.is_finite() as i32 != 0
        && minimum_turn.is_finite() as i32 != 0;
    out.feasible = out.valid as ::core::ffi::c_int != 0 && maximum >= minimum;
    out.minimum_speed_mps = minimum;
    out.maximum_speed_mps = maximum;
    out.density_kg_m3 = density;
    out.dynamic_pressure_limit_pa = (*v).maximum_dynamic_pressure;
    out.load_pressure_limit_pa = load_q;
    out.minimum_turn_radius_m = minimum_turn;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_bounded_capture_time(
    mut position: ::core::ffi::c_double,
    mut velocity: ::core::ffi::c_double,
    mut acceleration_limit: ::core::ffi::c_double,
    mut response_time: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if position.is_finite() as i32 == 0 || velocity.is_finite() as i32 == 0
        || acceleration_limit.is_finite() as i32 == 0
        || response_time.is_finite() as i32 == 0 || acceleration_limit <= 0.0f64
        || response_time < 0.0f64
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut stop_time: ::core::ffi::c_double = fabs(velocity) / acceleration_limit;
    let mut stop_displacement: ::core::ffi::c_double = copysign(
        velocity * velocity / (2.0f64 * acceleration_limit),
        velocity,
    );
    let mut residual: ::core::ffi::c_double = position + velocity * response_time
        + stop_displacement;
    let mut translate_time: ::core::ffi::c_double = 2.0f64
        * sqrt(fabs(residual) / acceleration_limit);
    return response_time + stop_time + translate_time;
}
#[no_mangle]
pub unsafe extern "C" fn decision_axis_capture_time(
    mut error_deg: ::core::ffi::c_double,
    mut rate_deg_s: ::core::ffi::c_double,
    mut acceleration_limit_deg_s2: ::core::ffi::c_double,
    mut rate_limit_deg_s: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if error_deg.is_finite() as i32 == 0 || rate_deg_s.is_finite() as i32 == 0
        || acceleration_limit_deg_s2.is_finite() as i32 == 0
        || rate_limit_deg_s.is_nan() as i32 != 0 || acceleration_limit_deg_s2 <= 0.0f64
        || rate_limit_deg_s <= 0.0f64
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut distance: ::core::ffi::c_double = fabs(error_deg);
    let mut direction: ::core::ffi::c_double = 0.;
    if distance > 0.0f64 {
        direction = copysign(1.0f64, error_deg);
    } else if rate_deg_s != 0.0f64 {
        direction = -copysign(1.0f64, rate_deg_s);
    } else {
        return 0.0f64
    }
    let mut initial_rate: ::core::ffi::c_double = direction * rate_deg_s;
    let mut acceleration: ::core::ffi::c_double = acceleration_limit_deg_s2;
    let mut rate_limit: ::core::ffi::c_double = if if rate_limit_deg_s.is_infinite() {
        if rate_limit_deg_s.is_sign_positive() { 1 } else { -1 }
    } else {
        0
    } != 0
    {
        ::core::f32::INFINITY as ::core::ffi::c_double
    } else {
        fmax(rate_limit_deg_s, fabs(initial_rate))
    };
    let mut stopping_distance: ::core::ffi::c_double = if initial_rate > 0.0f64 {
        initial_rate * initial_rate / (2.0f64 * acceleration)
    } else {
        0.0f64
    };
    if stopping_distance > distance {
        let mut reverse_rate_sq: ::core::ffi::c_double = initial_rate * initial_rate
            * 0.5f64 - acceleration * distance;
        let mut reverse_rate: ::core::ffi::c_double = sqrt(
            fmax(0.0f64, reverse_rate_sq),
        );
        return (initial_rate + 2.0f64 * reverse_rate) / acceleration;
    }
    let mut peak_rate_sq: ::core::ffi::c_double = acceleration * distance
        + initial_rate * initial_rate * 0.5f64;
    let mut peak_rate: ::core::ffi::c_double = sqrt(fmax(0.0f64, peak_rate_sq));
    if peak_rate <= rate_limit {
        return (2.0f64 * peak_rate - initial_rate) / acceleration;
    }
    let mut accelerate_distance: ::core::ffi::c_double = (rate_limit * rate_limit
        - initial_rate * initial_rate) / (2.0f64 * acceleration);
    let mut brake_distance: ::core::ffi::c_double = rate_limit * rate_limit
        / (2.0f64 * acceleration);
    let mut cruise_distance: ::core::ffi::c_double = fmax(
        0.0f64,
        distance - accelerate_distance - brake_distance,
    );
    return (rate_limit - initial_rate) / acceleration + cruise_distance / rate_limit
        + rate_limit / acceleration;
}
#[no_mangle]
pub unsafe extern "C" fn decision_pitch_capture_time(
    mut t: *const Telemetry,
    mut target_aoa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || (*t).angle_of_attack.is_finite() as i32 == 0
        || target_aoa.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut accel_limit: ::core::ffi::c_double = 0.0f64;
    let mut rate_limit: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    if (*t).attitude_response.pitch_valid {
        accel_limit = (*t).attitude_response.maximum_pitch_accel_deg_s2;
        rate_limit = (*t).attitude_response.maximum_pitch_rate_deg_s;
    } else if (*t).physics_authority[0 as ::core::ffi::c_int as usize].is_finite() as i32
        != 0 && (*t).physics_authority[0 as ::core::ffi::c_int as usize] > 0.0f64
        && (*t)
            .physics_authority_confidence[0 as ::core::ffi::c_int as usize]
            .is_finite() as i32 != 0
        && (*t).physics_authority_confidence[0 as ::core::ffi::c_int as usize] > 0.0f64
    {
        accel_limit = (*t).physics_authority[0 as ::core::ffi::c_int as usize];
    } else if (*t).has_torque as ::core::ffi::c_int != 0
        && (*t).has_inertia as ::core::ffi::c_int != 0
        && (*t).available_pitch_torque.is_finite() as i32 != 0
        && (*t).available_pitch_torque > 0.0f64
        && (*t).pitch_moment_of_inertia.is_finite() as i32 != 0
        && (*t).pitch_moment_of_inertia > 0.0f64
    {
        accel_limit = (*t).available_pitch_torque / (*t).pitch_moment_of_inertia
            * RAD2DEG;
    } else {
        accel_limit = 1.8f64;
        rate_limit = 4.5f64;
    }
    if !(accel_limit > DBL_MIN) || accel_limit.is_finite() as i32 == 0 {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return decision_axis_capture_time(
        target_aoa - (*t).angle_of_attack,
        if (*t).has_angle_of_attack_rate as ::core::ffi::c_int != 0 {
            (*t).angle_of_attack_rate
        } else {
            0.0f64
        },
        accel_limit,
        rate_limit,
    );
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const DBL_MANT_DIG: ::core::ffi::c_int = __DBL_MANT_DIG__;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
unsafe extern "C" fn path_atmosphere(
    mut model: *const TerminalModel,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
    mut latitude: ::core::ffi::c_double,
    mut longitude: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
    mut density: *mut ::core::ffi::c_double,
    mut sound_speed: *mut ::core::ffi::c_double,
) -> bool {
    let mut rho: ::core::ffi::c_double = 0.;
    let mut sound: ::core::ffi::c_double = 0.;
    if !model.is_null() {
        let mut lat: ::core::ffi::c_double = latitude * DEG2RAD;
        let mut lon: ::core::ffi::c_double = longitude * DEG2RAD;
        let mut radius: ::core::ffi::c_double = (*model).world.radius_m + altitude;
        let mut fixed: Vec3 = Vec3 {
            x: radius * cos(lat) * cos(lon),
            y: radius * cos(lat) * sin(lon),
            z: radius * sin(lat),
        };
        let mut inertial: Vec3 = world_fixed_to_inertial(
            &raw const (*model).world,
            fixed,
            ut,
        );
        let mut atmosphere: AtmosphereSample = world_atmosphere_sample_state(
            &raw const (*model).world,
            inertial,
            ut,
        );
        rho = atmosphere.density_kg_m3;
        sound = atmosphere.speed_of_sound_mps;
    } else {
        rho = planet_atmospheric_density(p, altitude);
        sound = planet_atmospheric_speed_of_sound(p, altitude);
    }
    *density = rho;
    *sound_speed = sound;
    return rho.is_finite() as i32 != 0 && rho >= 0.0f64 && sound.is_finite() as i32 != 0
        && sound > 0.0f64;
}
unsafe extern "C" fn path_force_scale(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
) -> PathForceScale {
    let mut l0: ::core::ffi::c_double = 0.0f64;
    let mut d0: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        hypot((*t).angle_of_attack, (*t).sideslip),
        v,
        &raw mut l0,
        &raw mut d0,
    );
    let mut beta: ::core::ffi::c_double = if (*t)
        .estimated_ballistic_coefficient
        .is_finite() as i32 != 0 && (*t).estimated_ballistic_coefficient > 0.0f64
    {
        (*t).estimated_ballistic_coefficient
    } else {
        (*v).estimated_ballistic_coefficient
    };
    let mut ld: ::core::ffi::c_double = if (*t).estimated_lift_to_drag.is_finite() as i32
        != 0 && (*t).estimated_lift_to_drag > 0.0f64
    {
        (*t).estimated_lift_to_drag
    } else {
        (*v).estimated_lift_to_drag
    };
    let mut scale: PathForceScale = PathForceScale {
        base: 1.0f64 / beta,
        lbase: 1.0f64 / beta * ld,
    };
    if (*t).mass > 0.0f64 && (*t).dynamic_pressure > 0.0f64
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force > 0.0f64
        && (*t).drag_force.is_finite() as i32 != 0 && (*t).drag_force > 0.0f64
    {
        if (*t).drag_force >= 0.0f64 && (*t).drag_force.is_finite() as i32 != 0
            && d0 > DBL_MIN
        {
            scale.base = (*t).drag_force / ((*t).mass * (*t).dynamic_pressure * d0);
        }
        if (*t).lift_force >= 0.0f64 && (*t).lift_force.is_finite() as i32 != 0
            && fabs(l0) > DBL_MIN
        {
            scale.lbase = (*t).lift_force
                / ((*t).mass * (*t).dynamic_pressure * fabs(l0));
        }
    }
    return scale;
}
unsafe extern "C" fn path_incidence_accelerations(
    mut model: *const TerminalModel,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut scale: PathForceScale,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut lift: *mut ::core::ffi::c_double,
    mut drag: *mut ::core::ffi::c_double,
) -> bool {
    if !model.is_null() {
        let mut lq: ::core::ffi::c_double = 0.0f64;
        let mut dq: ::core::ffi::c_double = 0.0f64;
        aero_force_per_q(
            &raw const (*model).aero,
            q,
            mach,
            aoa,
            &raw mut lq,
            &raw mut dq,
        );
        *lift = q * lq / (*t).mass;
        *drag = q * dq / (*t).mass;
    } else {
        let mut lf: ::core::ffi::c_double = 0.0f64;
        let mut df: ::core::ffi::c_double = 0.0f64;
        aerodynamic_force_factors_mach(mach, aoa, v, &raw mut lf, &raw mut df);
        *lift = q * scale.lbase * lf;
        *drag = q * scale.base * df;
    }
    return (*lift).is_finite() as i32 != 0 && (*drag).is_finite() as i32 != 0
        && *drag >= 0.0f64;
}
unsafe extern "C" fn path_aerodynamic_accelerations(
    mut model: *const TerminalModel,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut altitude: ::core::ffi::c_double,
    mut speed: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut latitude: ::core::ffi::c_double,
    mut longitude: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
    mut lift: *mut ::core::ffi::c_double,
    mut drag: *mut ::core::ffi::c_double,
) -> bool {
    if t.is_null() || p.is_null() || cfg.is_null() || lift.is_null() || drag.is_null()
        || speed.is_finite() as i32 == 0 || !(speed > 0.0f64)
        || aoa.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut rho: ::core::ffi::c_double = 0.;
    let mut sound: ::core::ffi::c_double = 0.;
    if !path_atmosphere(
        model,
        t,
        p,
        altitude,
        latitude,
        longitude,
        ut,
        &raw mut rho,
        &raw mut sound,
    ) {
        return false_0 != 0;
    }
    let mut scale: PathForceScale = if !model.is_null() {
        PathForceScale {
            base: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            lbase: 0.,
        }
    } else {
        path_force_scale(t, &raw const (*cfg).vehicle)
    };
    return path_incidence_accelerations(
        model,
        t,
        &raw const (*cfg).vehicle,
        scale,
        0.5f64 * rho * speed * speed,
        speed / sound,
        aoa,
        lift,
        drag,
    );
}
unsafe extern "C" fn path_observed_model(
    mut base: *const TerminalModel,
    mut t: *const Telemetry,
) -> *mut TerminalModel {
    let mut model: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    if model.is_null() {
        return ::core::ptr::null_mut::<TerminalModel>();
    }
    *model = *base;
    let mut lift_q: ::core::ffi::c_double = 0.0f64;
    let mut drag_q: ::core::ffi::c_double = 0.0f64;
    aero_force_per_q(
        &raw const (*base).aero,
        (*t).dynamic_pressure,
        (*t).mach,
        hypot((*t).angle_of_attack, (*t).sideslip),
        &raw mut lift_q,
        &raw mut drag_q,
    );
    let mut ls: ::core::ffi::c_double = if lift_q > DBL_MIN
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force >= 0.0f64
    {
        (*t).lift_force / ((*t).dynamic_pressure * lift_q)
    } else {
        1.0f64
    };
    let mut ds: ::core::ffi::c_double = if drag_q > DBL_MIN
        && (*t).drag_force.is_finite() as i32 != 0 && (*t).drag_force > 0.0f64
    {
        (*t).drag_force / ((*t).dynamic_pressure * drag_q)
    } else {
        1.0f64
    };
    aero_apply_mach_observation(&raw mut (*model).aero, (*t).mach, ls, ds);
    return model;
}
#[no_mangle]
pub unsafe extern "C" fn decision_unpowered_aerodynamic_accelerations(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut altitude: ::core::ffi::c_double,
    mut speed: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut lift: *mut ::core::ffi::c_double,
    mut drag: *mut ::core::ffi::c_double,
) -> bool {
    if t.is_null() {
        return false_0 != 0;
    }
    let mut base: *const TerminalModel = if !g.is_null() {
        (*g).entry_terminal_model as *const TerminalModel
    } else {
        ::core::ptr::null::<TerminalModel>()
    };
    let mut observed: *mut TerminalModel = if !base.is_null()
        && (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure > 0.0f64
    {
        path_observed_model(base, t)
    } else {
        ::core::ptr::null_mut::<TerminalModel>()
    };
    if !base.is_null() && (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure > 0.0f64 && observed.is_null()
    {
        return false_0 != 0;
    }
    let mut valid: bool = path_aerodynamic_accelerations(
        if !observed.is_null() { observed as *const TerminalModel } else { base },
        t,
        p,
        cfg,
        altitude,
        speed,
        aoa,
        (*t).latitude,
        (*t).longitude,
        (*t).ut,
        lift,
        drag,
    );
    free(observed as *mut ::core::ffi::c_void);
    return valid;
}
unsafe extern "C" fn path_axis_step(
    mut target: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut limits: GuidanceAttitudeLimits,
    mut angle: *mut ::core::ffi::c_double,
    mut rate: *mut ::core::ffi::c_double,
) {
    let mut error: ::core::ffi::c_double = target - *angle;
    let mut wanted: ::core::ffi::c_double = copysign(
        fmin(limits.rate_deg_s, sqrt(2.0f64 * limits.accel_deg_s2 * fabs(error))),
        error,
    );
    let mut next: ::core::ffi::c_double = *rate
        + clampd(wanted - *rate, -limits.accel_deg_s2 * dt, limits.accel_deg_s2 * dt);
    let mut delta: ::core::ffi::c_double = 0.5f64 * (*rate + next) * dt;
    if delta * error >= 0.0f64 && fabs(delta) > fabs(error) {
        *angle = target;
        *rate = 0.0f64;
    } else {
        *angle += delta;
        *rate = next;
    };
}
unsafe extern "C" fn path_acceleration(
    mut model: *const TerminalModel,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut position: Vec3,
    mut velocity: Vec3,
    mut aoa: ::core::ffi::c_double,
    mut bank: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
    mut accel: *mut Vec3,
    mut drag_power: *mut ::core::ffi::c_double,
) -> bool {
    let mut r: ::core::ffi::c_double = v3_norm(position);
    let mut speed: ::core::ffi::c_double = v3_norm(velocity);
    if !(r > 0.0f64) || !(speed > 0.0f64) {
        return false_0 != 0;
    }
    let mut up: Vec3 = v3_scale(position, 1.0f64 / r);
    let mut flow: Vec3 = v3_scale(velocity, 1.0f64 / speed);
    let mut lift: ::core::ffi::c_double = 0.0f64;
    let mut drag: ::core::ffi::c_double = 0.0f64;
    let mut latitude: ::core::ffi::c_double = asin(position.z / r) * RAD2DEG;
    let mut longitude: ::core::ffi::c_double = atan2(position.y, position.x) * RAD2DEG;
    if !path_aerodynamic_accelerations(
        model,
        t,
        p,
        cfg,
        r - (*p).radius,
        speed,
        aoa,
        latitude,
        longitude,
        ut,
        &raw mut lift,
        &raw mut drag,
    ) {
        return false_0 != 0;
    }
    let mut right: Vec3 = v3_normalized(v3_cross(flow, up));
    let mut lift_axis: Vec3 = v3_normalized(v3_cross(right, flow));
    lift_axis = v3_rotate_axis(lift_axis, flow, bank * DEG2RAD);
    let mut omega: Vec3 = Vec3 {
        x: 0.0f64,
        y: 0.0f64,
        z: (*p).rotational_speed,
    };
    *accel = v3_add(
        v3_scale(position, -(*p).gravitational_parameter / (r * r * r)),
        v3_add(
            v3_scale(v3_cross(omega, v3_cross(omega, position)), -1.0f64),
            v3_add(
                v3_scale(v3_cross(omega, velocity), -2.0f64),
                v3_add(v3_scale(flow, -drag), v3_scale(lift_axis, lift)),
            ),
        ),
    );
    *drag_power = drag * speed;
    return (*accel).x.is_finite() as i32 != 0 && (*accel).y.is_finite() as i32 != 0
        && (*accel).z.is_finite() as i32 != 0;
}
unsafe extern "C" fn path_reference_integrate(
    mut model: *const TerminalModel,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut target_altitude: ::core::ffi::c_double,
    mut path: ::core::ffi::c_double,
    mut resolution: ::core::ffi::c_uint,
) -> UnpoweredPathProjection {
    let mut out: UnpoweredPathProjection = UnpoweredPathProjection {
        valid: 0 as ::core::ffi::c_int != 0,
        reference_reachable: false,
        projected_ground_path_m: 0.,
        terminal_latitude_deg: 0.,
        integration_error_j_kg: 0.,
        ground_path_m: 0.,
        air_path_m: 0.,
        modeled_drag_work: 0.,
        terminal_speed_mps: 0.,
        travel_time_s: 0.,
        target_aoa_deg: 0.,
    };
    let mut lat: ::core::ffi::c_double = (*t).latitude * DEG2RAD;
    let mut r: ::core::ffi::c_double = (*p).radius + (*t).mean_altitude;
    let mut lon: ::core::ffi::c_double = (*t).longitude * DEG2RAD;
    let mut position: Vec3 = Vec3 {
        x: r * cos(lat) * cos(lon),
        y: r * cos(lat) * sin(lon),
        z: r * sin(lat),
    };
    let mut up: Vec3 = v3_normalized(position);
    let mut north: Vec3 = Vec3 {
        x: -sin(lat) * cos(lon),
        y: -sin(lat) * sin(lon),
        z: cos(lat),
    };
    let mut east: Vec3 = Vec3 {
        x: -sin(lon),
        y: cos(lon),
        z: 0.0f64,
    };
    let mut course: ::core::ffi::c_double = if (*t).ground_track_heading.is_finite()
        as i32 != 0
    {
        (*t).ground_track_heading * DEG2RAD
    } else {
        0.0f64
    };
    let mut horizontal: Vec3 = v3_add(
        v3_scale(north, cos(course)),
        v3_scale(east, sin(course)),
    );
    let mut gamma: ::core::ffi::c_double = (*t).flight_path_angle * DEG2RAD;
    let mut velocity: Vec3 = v3_scale(
        v3_add(v3_scale(horizontal, cos(gamma)), v3_scale(up, sin(gamma))),
        (*t).true_air_speed,
    );
    let mut aoa: ::core::ffi::c_double = (*t).angle_of_attack;
    let mut aoa_rate: ::core::ffi::c_double = if (*t).has_angle_of_attack_rate
        as ::core::ffi::c_int != 0
    {
        (*t).angle_of_attack_rate
    } else {
        0.0f64
    };
    let mut bank: ::core::ffi::c_double = (*t).roll;
    let mut bank_rate: ::core::ffi::c_double = (*t).roll_rate;
    let mut pitch: GuidanceAttitudeLimits = decision_guidance_attitude_limits(
        t,
        &raw const (*cfg).guidance,
        PHASE_ENTRY_ENERGY,
        true_0 != 0,
    );
    let mut roll: GuidanceAttitudeLimits = decision_guidance_attitude_limits(
        t,
        &raw const (*cfg).guidance,
        PHASE_ENTRY_ENERGY,
        false_0 != 0,
    );
    let mut step: ::core::ffi::c_double = 8.0f64 / resolution as ::core::ffi::c_double;
    let mut time: ::core::ffi::c_double = 0.0f64;
    let mut ground: ::core::ffi::c_double = 0.0f64;
    let mut air: ::core::ffi::c_double = 0.0f64;
    let mut work: ::core::ffi::c_double = 0.0f64;
    let mut initial_energy: ::core::ffi::c_double = rotating_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        p,
    );
    let mut target_aoa: ::core::ffi::c_double = aoa;
    let mut force_scale: PathForceScale = if !model.is_null() {
        PathForceScale {
            base: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            lbase: 0.,
        }
    } else {
        path_force_scale(t, &raw const (*cfg).vehicle)
    };
    let mut n: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while n < resolution.wrapping_mul(256 as ::core::ffi::c_uint) {
        let mut altitude: ::core::ffi::c_double = v3_norm(position) - (*p).radius;
        let mut speed: ::core::ffi::c_double = v3_norm(velocity);
        if altitude < target_altitude || !(speed > 0.0f64) {
            return out;
        }
        let mut latitude: ::core::ffi::c_double = asin(position.z / v3_norm(position))
            * RAD2DEG;
        let mut longitude: ::core::ffi::c_double = atan2(position.y, position.x)
            * RAD2DEG;
        let mut density: ::core::ffi::c_double = 0.;
        let mut sound: ::core::ffi::c_double = 0.;
        if !path_atmosphere(
            model,
            t,
            p,
            altitude,
            latitude,
            longitude,
            (*t).ut + time,
            &raw mut density,
            &raw mut sound,
        ) {
            return out;
        }
        let mut upper: ::core::ffi::c_double = (*cfg).vehicle.maximum_angle_of_attack;
        if speed / sound < 1.0f64
            && (*cfg).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64
        {
            upper = fmin(upper, (*cfg).vehicle.terminal_maximum_lift_angle_of_attack);
        }
        up = v3_normalized(position);
        let mut live_gamma: ::core::ffi::c_double = asin(
            clampd(v3_dot(velocity, up) / speed, -1.0f64, 1.0f64),
        );
        let mut desired_gamma: ::core::ffi::c_double = atan2(
            target_altitude - altitude,
            fmax(0.0f64, path - ground),
        );
        let mut response: ::core::ffi::c_double = decision_pitch_capture_time(t, upper);
        if response.is_finite() as i32 == 0 {
            return out;
        }
        response = fmax(response, step);
        let mut lat_now: ::core::ffi::c_double = asin(up.z);
        let mut gravity: ::core::ffi::c_double = (*p).gravitational_parameter
            / pow((*p).radius + altitude, 2.0f64)
            - (*p).rotational_speed * (*p).rotational_speed * ((*p).radius + altitude)
                * pow(cos(lat_now), 2.0f64);
        let mut required: ::core::ffi::c_double = fmax(
            0.0f64,
            ((gravity - speed * speed / ((*p).radius + altitude)) * cos(live_gamma)
                + speed * (desired_gamma - live_gamma) / response)
                / fmax(DBL_EPSILON, cos(bank * DEG2RAD)),
        );
        let mut best_drag: ::core::ffi::c_double = ::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut max_lift: ::core::ffi::c_double = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut max_lift_aoa: ::core::ffi::c_double = upper;
        let mut previous_lift: ::core::ffi::c_double = 0.0f64;
        let mut search_q: ::core::ffi::c_double = 0.5f64 * density * speed * speed;
        let mut search_mach: ::core::ffi::c_double = speed / sound;
        let mut j: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
        while j <= ceil(upper) as ::core::ffi::c_uint {
            let mut candidate: ::core::ffi::c_double = fmin(
                j as ::core::ffi::c_double,
                upper,
            );
            let mut lift: ::core::ffi::c_double = 0.0f64;
            let mut drag: ::core::ffi::c_double = 0.0f64;
            if !path_incidence_accelerations(
                model,
                t,
                &raw const (*cfg).vehicle,
                force_scale,
                search_q,
                search_mach,
                candidate,
                &raw mut lift,
                &raw mut drag,
            ) {
                return out;
            }
            if lift > max_lift {
                max_lift = lift;
                max_lift_aoa = candidate;
            }
            if j != 0 && previous_lift < required && lift >= required {
                let mut lo: ::core::ffi::c_double = candidate - 1.0f64;
                let mut hi: ::core::ffi::c_double = candidate;
                let mut k: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
                while k < 16 as ::core::ffi::c_uint {
                    let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
                    let mut ml: ::core::ffi::c_double = 0.0f64;
                    let mut md: ::core::ffi::c_double = 0.0f64;
                    if !path_incidence_accelerations(
                        model,
                        t,
                        &raw const (*cfg).vehicle,
                        force_scale,
                        search_q,
                        search_mach,
                        mid,
                        &raw mut ml,
                        &raw mut md,
                    ) {
                        return out;
                    }
                    if ml < required {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                    k = k.wrapping_add(1);
                }
                candidate = hi;
                if !path_incidence_accelerations(
                    model,
                    t,
                    &raw const (*cfg).vehicle,
                    force_scale,
                    search_q,
                    search_mach,
                    candidate,
                    &raw mut lift,
                    &raw mut drag,
                ) {
                    return out;
                }
            }
            if lift >= required && drag < best_drag {
                best_drag = drag;
                target_aoa = candidate;
            }
            previous_lift = lift;
            j = j.wrapping_add(1);
        }
        if best_drag.is_finite() as i32 == 0 {
            target_aoa = max_lift_aoa;
        }
        path_axis_step(target_aoa, step, pitch, &raw mut aoa, &raw mut aoa_rate);
        path_axis_step(0.0f64, step, roll, &raw mut bank, &raw mut bank_rate);
        let mut accel: Vec3 = Vec3 { x: 0., y: 0., z: 0. };
        let mut mid_accel: Vec3 = Vec3 { x: 0., y: 0., z: 0. };
        let mut power: ::core::ffi::c_double = 0.;
        let mut mid_power: ::core::ffi::c_double = 0.;
        if !path_acceleration(
            model,
            t,
            p,
            cfg,
            position,
            velocity,
            aoa,
            bank,
            (*t).ut + time,
            &raw mut accel,
            &raw mut power,
        ) {
            return out;
        }
        let mut mid_position: Vec3 = v3_add(position, v3_scale(velocity, 0.5f64 * step));
        let mut mid_velocity: Vec3 = v3_add(velocity, v3_scale(accel, 0.5f64 * step));
        if !path_acceleration(
            model,
            t,
            p,
            cfg,
            mid_position,
            mid_velocity,
            aoa,
            bank,
            (*t).ut + time + 0.5f64 * step,
            &raw mut mid_accel,
            &raw mut mid_power,
        ) {
            return out;
        }
        let mut next_position: Vec3 = v3_add(position, v3_scale(mid_velocity, step));
        let mut next_velocity: Vec3 = v3_add(velocity, v3_scale(mid_accel, step));
        let mut next_altitude: ::core::ffi::c_double = v3_norm(next_position)
            - (*p).radius;
        let mut dt: ::core::ffi::c_double = step;
        let mut arrived: bool = next_altitude <= target_altitude;
        if arrived {
            let mut fraction: ::core::ffi::c_double = (altitude - target_altitude)
                / (altitude - next_altitude);
            dt *= fraction;
            next_position = v3_add(
                position,
                v3_scale(v3_sub(next_position, position), fraction),
            );
            next_velocity = v3_add(
                velocity,
                v3_scale(v3_sub(next_velocity, velocity), fraction),
            );
            next_position = v3_scale(
                v3_normalized(next_position),
                (*p).radius + target_altitude,
            );
        }
        up = v3_normalized(mid_position);
        let mut radial: ::core::ffi::c_double = v3_dot(mid_velocity, up);
        ground
            += sqrt(fmax(0.0f64, v3_dot(mid_velocity, mid_velocity) - radial * radial))
                * (*p).radius / v3_norm(mid_position) * dt;
        air += v3_norm(mid_velocity) * dt;
        work += mid_power * dt;
        time += dt;
        position = next_position;
        velocity = next_velocity;
        if arrived {
            let mut final_lat: ::core::ffi::c_double = asin(
                position.z / v3_norm(position),
            ) * RAD2DEG;
            let mut energy: ::core::ffi::c_double = rotating_specific_energy(
                final_lat,
                target_altitude,
                v3_norm(velocity),
                p,
            );
            out.valid = work.is_finite() as i32 != 0 && energy.is_finite() as i32 != 0
                && work >= 0.0f64;
            out.reference_reachable = ground >= path;
            out.projected_ground_path_m = ground;
            out.terminal_latitude_deg = final_lat;
            out.integration_error_j_kg = fabs(initial_energy - energy - work);
            out.ground_path_m = path;
            out.air_path_m = air;
            out.modeled_drag_work = work;
            out.terminal_speed_mps = v3_norm(velocity);
            out.travel_time_s = time;
            out.target_aoa_deg = target_aoa;
            return out;
        }
        n = n.wrapping_add(1);
    }
    return out;
}
unsafe extern "C" fn path_reference_project(
    mut model: *const TerminalModel,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut target_altitude: ::core::ffi::c_double,
    mut path: ::core::ffi::c_double,
    mut relative_tolerance: ::core::ffi::c_double,
) -> UnpoweredPathProjection {
    let mut out: UnpoweredPathProjection = UnpoweredPathProjection {
        valid: 0 as ::core::ffi::c_int != 0,
        reference_reachable: false,
        projected_ground_path_m: 0.,
        terminal_latitude_deg: 0.,
        integration_error_j_kg: 0.,
        ground_path_m: 0.,
        air_path_m: 0.,
        modeled_drag_work: 0.,
        terminal_speed_mps: 0.,
        travel_time_s: 0.,
        target_aoa_deg: 0.,
    };
    if t.is_null() || p.is_null() || cfg.is_null()
        || relative_tolerance.is_finite() as i32 == 0 || !(relative_tolerance > 0.0f64)
        || target_altitude.is_finite() as i32 == 0 || path.is_finite() as i32 == 0
        || path < 0.0f64 || (*t).mean_altitude.is_finite() as i32 == 0
        || (*t).latitude.is_finite() as i32 == 0 || fabs((*t).latitude) > 90.0f64
        || (*t).longitude.is_finite() as i32 == 0 || (*t).ut.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0
        || (*t).dynamic_pressure < 0.0f64 || (*t).true_air_speed.is_finite() as i32 == 0
        || !((*t).true_air_speed > 0.0f64)
        || (*t).angle_of_attack.is_finite() as i32 == 0
        || (*t).sideslip.is_finite() as i32 == 0 || (*t).roll.is_finite() as i32 == 0
        || (*t).flight_path_angle.is_finite() as i32 == 0
        || !((*p).radius + (*t).mean_altitude > 0.0f64)
        || !((*p).radius + target_altitude > 0.0f64)
        || !((*p).gravitational_parameter > 0.0f64)
        || (*p).rotational_speed.is_finite() as i32 == 0
        || !((*cfg).vehicle.estimated_ballistic_coefficient > 0.0f64)
        || !model.is_null() && (!((*t).mass > 0.0f64) || !(*model).replay_validated)
    {
        return out;
    }
    let mut tolerance: ::core::ffi::c_double = relative_tolerance
        * fmax(
            1.0f64,
            fabs(
                entry_remaining_specific_energy(
                    (*t).latitude,
                    (*t).mean_altitude,
                    (*t).true_air_speed,
                    (*cfg).site.latitude,
                    target_altitude,
                    0.0f64,
                    p,
                ),
            ),
        );
    let mut n: ::core::ffi::c_uint = 32 as ::core::ffi::c_uint;
    out = path_reference_integrate(model, t, p, cfg, target_altitude, path, n);
    while out.valid as ::core::ffi::c_int != 0 && n < 4096 as ::core::ffi::c_uint {
        n = n.wrapping_mul(2 as ::core::ffi::c_uint);
        let mut finer: UnpoweredPathProjection = path_reference_integrate(
            model,
            t,
            p,
            cfg,
            target_altitude,
            path,
            n,
        );
        if !finer.valid {
            return finer;
        }
        finer.integration_error_j_kg = fmax(
            finer.integration_error_j_kg,
            fabs(finer.modeled_drag_work - out.modeled_drag_work),
        );
        out = finer;
        if out.integration_error_j_kg <= tolerance {
            return out;
        }
    }
    out.valid = false_0 != 0;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_unpowered_path_projection_with_tolerance(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut altitude: ::core::ffi::c_double,
    mut path: ::core::ffi::c_double,
    mut tolerance: ::core::ffi::c_double,
) -> UnpoweredPathProjection {
    let mut invalid: UnpoweredPathProjection = UnpoweredPathProjection {
        valid: 0 as ::core::ffi::c_int != 0,
        reference_reachable: false,
        projected_ground_path_m: 0.,
        terminal_latitude_deg: 0.,
        integration_error_j_kg: 0.,
        ground_path_m: 0.,
        air_path_m: 0.,
        modeled_drag_work: 0.,
        terminal_speed_mps: 0.,
        travel_time_s: 0.,
        target_aoa_deg: 0.,
    };
    let mut base: *const TerminalModel = if !g.is_null() {
        (*g).entry_terminal_model as *const TerminalModel
    } else {
        ::core::ptr::null::<TerminalModel>()
    };
    let mut observed: *mut TerminalModel = if !base.is_null() && !t.is_null()
        && (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure > 0.0f64
    {
        path_observed_model(base, t)
    } else {
        ::core::ptr::null_mut::<TerminalModel>()
    };
    if !base.is_null() && !t.is_null()
        && (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure > 0.0f64 && observed.is_null()
    {
        return invalid;
    }
    let mut result: UnpoweredPathProjection = path_reference_project(
        if !observed.is_null() { observed as *const TerminalModel } else { base },
        t,
        p,
        cfg,
        altitude,
        path,
        tolerance,
    );
    free(observed as *mut ::core::ffi::c_void);
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn decision_unpowered_path_projection(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut target_altitude: ::core::ffi::c_double,
    mut path: ::core::ffi::c_double,
) -> UnpoweredPathProjection {
    return decision_unpowered_path_projection_with_tolerance(
        g,
        t,
        p,
        cfg,
        target_altitude,
        path,
        1e-4f64,
    );
}
/// Experimental: apply active-regime uncertainty at Mach >= 1 too. Off by
/// default (the C build's `-DMM305_SUPERSONIC_ACTIVE_UNCERTAINTY=1`); enable
/// with the `mm305-supersonic-active-uncertainty` cargo feature.
pub const MM305_SUPERSONIC_ACTIVE_UNCERTAINTY: ::core::ffi::c_int =
    cfg!(feature = "mm305-supersonic-active-uncertainty") as ::core::ffi::c_int;
#[no_mangle]
pub unsafe extern "C" fn decision_energy_path_envelope(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut available_specific_energy: ::core::ffi::c_double,
    mut modeled_drag_work: ::core::ffi::c_double,
    mut path_length_m: ::core::ffi::c_double,
) -> EnergyPathEnvelope {
    let mut out: EnergyPathEnvelope = EnergyPathEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        feasible: false,
        energy: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        available_specific_energy: 0.,
        modeled_drag_work: 0.,
        uncertainty_specific_energy: 0.,
    };
    if t.is_null() || p.is_null() || available_specific_energy.is_finite() as i32 == 0
        || modeled_drag_work.is_finite() as i32 == 0
        || path_length_m.is_finite() as i32 == 0 || path_length_m < 0.0f64
        || modeled_drag_work < 0.0f64
    {
        return out;
    }
    let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
    if !(gravity > 0.0f64) || gravity.is_finite() as i32 == 0 {
        return out;
    }
    let mut uncertainty: ::core::ffi::c_double = 0.0f64;
    if (*t).trajectory_altitude_residual.is_finite() as i32 != 0 {
        uncertainty += gravity * fabs((*t).trajectory_altitude_residual);
    }
    if (*t).trajectory_speed_residual.is_finite() as i32 != 0 {
        let mut dv: ::core::ffi::c_double = fabs((*t).trajectory_speed_residual);
        uncertainty += fabs((*t).true_air_speed) * dv + 0.5f64 * dv * dv;
    }
    if (*t).trajectory_range_residual.is_finite() as i32 != 0 && path_length_m > 0.0f64 {
        let mut mean_drag_loss: ::core::ffi::c_double = modeled_drag_work
            / path_length_m;
        uncertainty += fabs(mean_drag_loss * (*t).trajectory_range_residual);
    }
    let mut relative_uncertainty: ::core::ffi::c_double = 0.0f64;
    if (*t).physics_certified_uncertainty.is_finite() as i32 != 0
        && (*t).physics_certified_uncertainty > 0.0f64
    {
        relative_uncertainty = (*t).physics_certified_uncertainty;
    }
    if (*t).predicted_physics_relative_uncertainty.is_finite() as i32 != 0
        && (*t).predicted_physics_relative_uncertainty > 0.0f64
    {
        relative_uncertainty = fmax(
            relative_uncertainty,
            (*t).predicted_physics_relative_uncertainty,
        );
    }
    if (*t).mach.is_finite() as i32 != 0
        && ((*t).mach < 1.0f64 || MM305_SUPERSONIC_ACTIVE_UNCERTAINTY != 0)
        && relative_uncertainty > 0.08f64
    {
        let mut active_unc: ::core::ffi::c_double = 0.0f64;
        let mut active_evidence: bool = false_0 != 0;
        if (*t).aerodynamic_confidence.is_finite() as i32 != 0
            && (*t).aerodynamic_confidence > 0.0f64
        {
            active_unc = fmax(active_unc, 1.0f64 - (*t).aerodynamic_confidence);
            active_evidence = true_0 != 0;
        }
        if (*t).physics_model_residual.is_finite() as i32 != 0
            && (*t).physics_model_residual >= 0.0f64
            && (*t).physics_model_residual_confidence.is_finite() as i32 != 0
            && (*t).physics_model_residual_confidence > 0.0f64
        {
            active_unc = fmax(active_unc, (*t).physics_model_residual);
            active_evidence = true_0 != 0;
        }
        relative_uncertainty = fmin(
            relative_uncertainty,
            if active_evidence as ::core::ffi::c_int != 0 {
                clampd(active_unc, 0.0f64, 0.35f64)
            } else {
                0.08f64
            },
        );
    }
    uncertainty += modeled_drag_work * relative_uncertainty;
    out.available_specific_energy = available_specific_energy;
    out.modeled_drag_work = modeled_drag_work;
    out.uncertainty_specific_energy = uncertainty;
    out.energy = decision_margin(
        available_specific_energy,
        modeled_drag_work + uncertainty,
    );
    out.valid = out.energy.valid;
    out.feasible = out.valid as ::core::ffi::c_int != 0 && out.energy.margin >= 0.0f64;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_path_feasibility_envelope(
    mut path_length_m: ::core::ffi::c_double,
    mut minimum_path_m: ::core::ffi::c_double,
    mut maximum_path_m: ::core::ffi::c_double,
    mut slope_deg: ::core::ffi::c_double,
    mut minimum_slope_deg: ::core::ffi::c_double,
    mut maximum_slope_deg: ::core::ffi::c_double,
    mut required_lateral_accel_mps2: ::core::ffi::c_double,
    mut available_lateral_accel_mps2: ::core::ffi::c_double,
    mut peak_course_rate_ratio: ::core::ffi::c_double,
    mut endpoint_course_rate_deg_s: ::core::ffi::c_double,
    mut endpoint_course_rate_limit_deg_s: ::core::ffi::c_double,
) -> PathFeasibilityEnvelope {
    let mut out: PathFeasibilityEnvelope = PathFeasibilityEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        feasible: false,
        path_minimum: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        path_maximum: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        slope_minimum: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        slope_maximum: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        lateral_authority: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        course_rate: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        endpoint_rate: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        worst_normalized_margin: 0.,
    };
    if path_length_m.is_finite() as i32 == 0 || minimum_path_m.is_finite() as i32 == 0
        || maximum_path_m.is_finite() as i32 == 0 || slope_deg.is_finite() as i32 == 0
        || minimum_slope_deg.is_finite() as i32 == 0
        || maximum_slope_deg.is_finite() as i32 == 0
        || required_lateral_accel_mps2.is_finite() as i32 == 0
        || available_lateral_accel_mps2.is_finite() as i32 == 0
        || peak_course_rate_ratio.is_finite() as i32 == 0
        || endpoint_course_rate_deg_s.is_finite() as i32 == 0
        || endpoint_course_rate_limit_deg_s.is_finite() as i32 == 0
        || minimum_path_m < 0.0f64 || maximum_path_m < minimum_path_m
        || maximum_slope_deg < minimum_slope_deg || available_lateral_accel_mps2 < 0.0f64
        || endpoint_course_rate_limit_deg_s < 0.0f64
    {
        return out;
    }
    out.path_minimum = decision_margin(path_length_m, minimum_path_m);
    out.path_maximum = decision_margin(maximum_path_m, path_length_m);
    out.slope_minimum = decision_margin(slope_deg, minimum_slope_deg);
    out.slope_maximum = decision_margin(maximum_slope_deg, slope_deg);
    out.lateral_authority = decision_margin(
        available_lateral_accel_mps2,
        required_lateral_accel_mps2,
    );
    out.course_rate = decision_margin(1.0f64, peak_course_rate_ratio);
    out.endpoint_rate = decision_margin(
        endpoint_course_rate_limit_deg_s,
        endpoint_course_rate_deg_s,
    );
    let mut margins: [*const DecisionMargin; 7] = [
        &raw mut out.path_minimum as *const DecisionMargin,
        &raw mut out.path_maximum as *const DecisionMargin,
        &raw mut out.slope_minimum as *const DecisionMargin,
        &raw mut out.slope_maximum as *const DecisionMargin,
        &raw mut out.lateral_authority as *const DecisionMargin,
        &raw mut out.course_rate as *const DecisionMargin,
        &raw mut out.endpoint_rate as *const DecisionMargin,
    ];
    out.worst_normalized_margin = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[*const DecisionMargin; 7]>() as usize)
            .wrapping_div(::core::mem::size_of::<*const DecisionMargin>() as usize)
    {
        if !(*margins[i as usize]).valid {
            return out;
        }
        out.worst_normalized_margin = fmin(
            out.worst_normalized_margin,
            (*margins[i as usize]).normalized_margin,
        );
        i = i.wrapping_add(1);
    }
    out.valid = out.worst_normalized_margin.is_finite() as i32 != 0;
    out.feasible = out.valid as ::core::ffi::c_int != 0
        && out.worst_normalized_margin >= 0.0f64;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn decision_runway_capture_envelope(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course_deg: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> RunwayCaptureEnvelope {
    let mut out: RunwayCaptureEnvelope = RunwayCaptureEnvelope {
        valid: 0 as ::core::ffi::c_int != 0,
        reachable: false,
        runway_time: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        vertical_recovery: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        speed: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        dynamic_pressure: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        load: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        runway_remaining: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        ground_range_m: 0.,
        time_available_s: 0.,
        lateral_capture_time_s: 0.,
        heading_capture_time_s: 0.,
        control_response_time_s: 0.,
        cross_rate_mps: 0.,
        lateral_accel_mps2: 0.,
        course_rate_deg_s: 0.,
        maximum_lift_aoa_deg: 0.,
        maximum_normal_accel_mps2: 0.,
        sink_rate_mps: 0.,
        vertical_recovery_accel_mps2: 0.,
        response_down_accel_mps2: 0.,
        vertical_recovery_height_m: 0.,
        recoverable_sink_rate_mps: 0.,
        recoverable_fpa_deg: 0.,
    };
    if t.is_null() || p.is_null() || cfg.is_null() || course_deg.is_finite() as i32 == 0
    {
        return out;
    }
    let mut site: *const LandingSite = &raw const (*cfg).site;
    let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
    let mut horizontal: ::core::ffi::c_double = finite_positive((*t).horizontal_speed);
    if !(gravity > 0.0f64) || !(horizontal > 0.0f64)
        || (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
    {
        return out;
    }
    let mut authority: ControlAuthorityEnvelope = decision_control_authority_envelope(
        g,
        t,
        p,
        cfg,
    );
    if !authority.valid {
        return out;
    }
    let mut course_error_rad: ::core::ffi::c_double = norm_signed_deg(
        course_deg - (*site).runway_heading,
    ) * DEG2RAD;
    let mut cross_rate: ::core::ffi::c_double = horizontal * sin(course_error_rad);
    let mut max_lift_aoa: ::core::ffi::c_double = authority.maximum_lift_aoa_deg;
    let mut normal_accel: ::core::ffi::c_double = authority.maximum_normal_accel_mps2;
    let mut lateral_accel: ::core::ffi::c_double = authority.maximum_lateral_accel_mps2;
    let mut response_time: ::core::ffi::c_double = authority.control_response_time_s;
    let mut lateral_time: ::core::ffi::c_double = decision_bounded_capture_time(
        (*t).runway_cross_track,
        cross_rate,
        lateral_accel,
        response_time,
    );
    let mut course_rate_rad: ::core::ffi::c_double = if horizontal > 0.0f64 {
        lateral_accel / horizontal
    } else {
        0.0f64
    };
    let mut heading_time: ::core::ffi::c_double = if course_rate_rad > 0.0f64 {
        response_time + fabs(course_error_rad) / course_rate_rad
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut runway_forward: ::core::ffi::c_double = (*site).runway_length
        - (*t).runway_along_track;
    let mut direct_ground_range: ::core::ffi::c_double = hypot(
        fmax(0.0f64, runway_forward),
        (*t).runway_cross_track,
    );
    let mut fixed_hac: bool = !g.is_null()
        && (*g).mm305_route_committed as ::core::ffi::c_int != 0
        && (*g).mm305_route.valid as ::core::ffi::c_int != 0
        && !(*g).mm305_hac_exit_reached && !(*g).final_approach_captured;
    let mut committed_path: ::core::ffi::c_double = 0.0f64;
    if fixed_hac {
        let mut route_remaining: ::core::ffi::c_double = taem_route_remaining_at_index(
            &raw const (*g).mm305_route,
            (*g).mm305_route_cursor,
        );
        if route_remaining.is_finite() as i32 != 0 {
            committed_path = route_remaining
                + fmax(0.0f64, (*cfg).guidance.final_approach_distance);
        }
    }
    let mut ground_range: ::core::ffi::c_double = if fixed_hac as ::core::ffi::c_int != 0
        && committed_path.is_finite() as i32 != 0 && committed_path > 0.0f64
    {
        committed_path
    } else {
        direct_ground_range
    };
    let mut time_available: ::core::ffi::c_double = ground_range / horizontal;
    let mut time_required: ::core::ffi::c_double = fmax(lateral_time, heading_time);
    let mut sink: ::core::ffi::c_double = fmax(0.0f64, -(*t).vertical_speed);
    if (*t).flight_path_angle.is_finite() as i32 == 0
        || (*t).roll.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
    {
        return out;
    }
    let mut vertical_projection: ::core::ffi::c_double = cos(
        (*t).flight_path_angle * DEG2RAD,
    );
    let mut vertical_accel: ::core::ffi::c_double = fmax(
        0.0f64,
        normal_accel * vertical_projection - gravity,
    );
    let mut current_lift: ::core::ffi::c_double = measured_or_modeled_lift_accel(
        t,
        &raw const (*cfg).vehicle,
    );
    let mut response_down: ::core::ffi::c_double = fmax(
        0.0f64,
        gravity - current_lift * cos((*t).roll * DEG2RAD) * vertical_projection,
    );
    let mut radar: ::core::ffi::c_double = if (*t).radar_altitude.is_finite() as i32 != 0
        && (*t).radar_altitude >= 0.0f64
    {
        (*t).radar_altitude
    } else {
        (*t).mean_altitude - (*site).altitude
    };
    let mut recovery: VerticalRecoveryEnvelope = decision_vertical_recovery_envelope(
        radar,
        sink,
        0.0f64,
        response_time,
        response_down,
        vertical_accel,
    );
    let mut recovery_height: ::core::ffi::c_double = if recovery.valid
        as ::core::ffi::c_int != 0
    {
        recovery.required_height_m
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut recoverable_sink: ::core::ffi::c_double = 0.0f64;
    if vertical_accel > 0.0f64 && radar >= 0.0f64 {
        let mut radicand: ::core::ffi::c_double = vertical_accel
            * (vertical_accel + response_down) * response_time * response_time
            + 2.0f64 * vertical_accel * radar;
        recoverable_sink = fmax(
            0.0f64,
            sqrt(radicand) - (vertical_accel + response_down) * response_time,
        );
    }
    out.runway_time = decision_margin(time_available, time_required);
    out.vertical_recovery = recovery.height;
    out.speed = authority.speed;
    out.dynamic_pressure = authority.dynamic_pressure;
    out.load = authority.load;
    out.runway_remaining = decision_margin(runway_forward, 0.0f64);
    out.ground_range_m = ground_range;
    out.time_available_s = time_available;
    out.lateral_capture_time_s = lateral_time;
    out.heading_capture_time_s = heading_time;
    out.control_response_time_s = response_time;
    out.cross_rate_mps = cross_rate;
    out.lateral_accel_mps2 = lateral_accel;
    out.course_rate_deg_s = course_rate_rad * RAD2DEG;
    out.maximum_lift_aoa_deg = max_lift_aoa;
    out.maximum_normal_accel_mps2 = normal_accel;
    out.sink_rate_mps = sink;
    out.vertical_recovery_accel_mps2 = vertical_accel;
    out.response_down_accel_mps2 = response_down;
    out.vertical_recovery_height_m = recovery_height;
    out.recoverable_sink_rate_mps = recoverable_sink;
    out.recoverable_fpa_deg = if horizontal > 0.0f64 {
        atan2(recoverable_sink, horizontal) * RAD2DEG
    } else {
        0.0f64
    };
    out.valid = out.runway_time.valid as ::core::ffi::c_int != 0
        && out.vertical_recovery.valid as ::core::ffi::c_int != 0
        && out.speed.valid as ::core::ffi::c_int != 0
        && out.dynamic_pressure.valid as ::core::ffi::c_int != 0
        && out.load.valid as ::core::ffi::c_int != 0
        && out.runway_remaining.valid as ::core::ffi::c_int != 0
        && lateral_time.is_finite() as i32 != 0 && heading_time.is_finite() as i32 != 0;
    out.reachable = out.valid as ::core::ffi::c_int != 0
        && authority.controllable as ::core::ffi::c_int != 0
        && out.runway_time.margin >= 0.0f64 && out.vertical_recovery.margin >= 0.0f64
        && out.speed.margin >= 0.0f64 && out.dynamic_pressure.margin >= 0.0f64
        && out.load.margin >= 0.0f64 && out.runway_remaining.margin >= 0.0f64;
    return out;
}
