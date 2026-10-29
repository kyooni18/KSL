use crate::sys::*;
extern "C" {
    fn entry_exec_update(
        exec: *mut EntryExecutive,
        observation: *const EntryExecObservation,
        profile: *const EntryExecProfile,
    ) -> bool;
    fn entry_phase_string(phase: EntryPhase) -> *const ::core::ffi::c_char;
    fn entry_lateral_default_limits() -> EntryLateralLimits;
    fn entry_lateral_update(
        state: *mut EntryLateralState,
        input: *const EntryLateralInput,
        limits: *const EntryLateralLimits,
    ) -> EntryLateralOutput;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn entry_mm305_admission_envelope(
        guidance: *const GuidanceMachine,
        telemetry: *const Telemetry,
        course: ::core::ffi::c_double,
        planet: *const PlanetModel,
        configuration: *const LandingConfiguration,
    ) -> TaemInterfaceCapture;
    fn entry_mm305_admission_prefilter_veto(
        telemetry: *const Telemetry,
        planet: *const PlanetModel,
        configuration: *const LandingConfiguration,
    ) -> ::core::ffi::c_uint;
    fn aero_force_per_q(
        a: *const AeroTable,
        q: ::core::ffi::c_double,
        mach: ::core::ffi::c_double,
        alpha_deg: ::core::ffi::c_double,
        lift_per_q: *mut ::core::ffi::c_double,
        drag_per_q: *mut ::core::ffi::c_double,
    );
    fn entry_drag_reference_default_config() -> EntryDragReferenceConfig;
    fn entry_drag_reference_compute(
        input: *const EntryDragReferenceInput,
        config: *const EntryDragReferenceConfig,
    ) -> EntryDragReferenceOutput;
    fn entry_drag_reference_exec_profile(
        input: *const EntryDragReferenceInput,
        config: *const EntryDragReferenceConfig,
        reference: *const EntryDragReferenceOutput,
    ) -> EntryExecProfile;
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
    fn entry_remaining_specific_energy(
        latitude: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
        air_relative_speed: ::core::ffi::c_double,
        target_latitude: ::core::ffi::c_double,
        target_altitude: ::core::ffi::c_double,
        target_speed: ::core::ffi::c_double,
        planet: *const PlanetModel,
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
    fn great_circle_distance(
        a: GeoPoint,
        b: GeoPoint,
        radius: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn initial_bearing(a: GeoPoint, b: GeoPoint) -> ::core::ffi::c_double;
    fn runway_approach_aimpoint(
        site: *const LandingSite,
        radius: ::core::ffi::c_double,
        distance_before_threshold: ::core::ffi::c_double,
    ) -> GeoPoint;
    fn runway_reciprocal_site(
        primary: *const LandingSite,
        radius: ::core::ffi::c_double,
    ) -> LandingSite;
    fn planet_surface_gravity(p: *const PlanetModel) -> ::core::ffi::c_double;
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
    fn entry_taem_speed_target(
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn decision_target_path_length(
        guidance: *const GuidanceMachine,
        telemetry: *const Telemetry,
        course_deg: ::core::ffi::c_double,
        target_along_m: ::core::ffi::c_double,
        target_cross_m: ::core::ffi::c_double,
        target_course_deg: ::core::ffi::c_double,
        lateral_accel_limit_mps2: ::core::ffi::c_double,
        planet: *const PlanetModel,
        configuration: *const LandingConfiguration,
        minimum_turn_radius_m: *mut ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn entry_alpha_schedule_default(
        schedule: *mut EntryAlphaSchedule,
        vehicle: *const VehicleProfile,
        transition_velocity: ::core::ffi::c_double,
    );
    fn entry_alpha_schedule_interpolate(
        schedule: *const EntryAlphaSchedule,
        relative_velocity: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn entry_alpha_command(
        schedule: *const EntryAlphaSchedule,
        input: *const EntryAlphaInput,
    ) -> EntryAlphaResult;
    fn controlled_roll_rate(t: *const Telemetry) -> ::core::ffi::c_double;
    fn controlled_aoa_rate(t: *const Telemetry) -> ::core::ffi::c_double;
    fn result_make(
        phase: GuidancePhase,
        c: GuidanceCommand,
        status: *const ::core::ffi::c_char,
        warning: *const ::core::ffi::c_char,
    ) -> GuidanceResult;
    fn dynamic_bank_limit(
        t: *const Telemetry,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn atmospheric(
        t: *const Telemetry,
        heading: ::core::ffi::c_double,
        roll: ::core::ffi::c_double,
        v: *const VehicleProfile,
        throttle: ::core::ffi::c_double,
        air: bool,
        profile: ControlProfile,
    ) -> GuidanceCommand;
    fn stabilized(
        g: *mut GuidanceMachine,
        r: GuidanceResult,
        t: *const Telemetry,
        v: *const VehicleProfile,
        s: *const GuidanceSettings,
        dt: ::core::ffi::c_double,
    ) -> GuidanceResult;
    fn mm305_energy_law_handoff_reach(
        model: *const TerminalModel,
        altitude_above_site_m: ::core::ffi::c_double,
        speed_mps: ::core::ffi::c_double,
        mass_kg: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn mm305_hac_handoff_state(
        model: *const TerminalModel,
        live: *const TaemHacLiveAero,
        mass_kg: ::core::ffi::c_double,
        mach: ::core::ffi::c_double,
        altitude_above_site_m: *mut ::core::ffi::c_double,
        speed_mps: *mut ::core::ffi::c_double,
        station_range_m: *mut ::core::ffi::c_double,
    ) -> bool;
    fn mm305_hac_reference_energy(
        model: *const TerminalModel,
        live: *const TaemHacLiveAero,
        mass_kg: ::core::ffi::c_double,
        station_range_m: ::core::ffi::c_double,
        energy_m: *mut ::core::ffi::c_double,
    ) -> bool;
    fn mm305_hac_observe_aero(g: *mut GuidanceMachine, t: *const Telemetry);
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
pub struct EntryExecObservation {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub checkpoint_restart: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryExecProfile {
    pub has_temperature_velocity_gate: bool,
    pub temperature_end_velocity: ::core::ffi::c_double,
    pub equilibrium_intercept_valid: bool,
    pub equilibrium_intercept: bool,
    pub has_constant_drag_velocity_gate: bool,
    pub constant_drag_end_velocity: ::core::ffi::c_double,
    pub has_phase_floor: bool,
    pub phase_floor: EntryPhase,
}
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
pub struct EntryLateralLongitudinalDemand {
    pub valid: bool,
    pub confidence: ::core::ffi::c_double,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub drag_error_accel: ::core::ffi::c_double,
    pub predicted_range_error: ::core::ffi::c_double,
    pub range_error_scale: ::core::ffi::c_double,
    pub required_vertical_lift_accel: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralLimits {
    pub maximum_bank_deg: ::core::ffi::c_double,
    pub maximum_roll_rate_deg_s: ::core::ffi::c_double,
    pub maximum_roll_accel_deg_s2: ::core::ffi::c_double,
    pub minimum_leg_duration_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralInput {
    pub ut: ::core::ffi::c_double,
    pub dt: ::core::ffi::c_double,
    pub relative_speed: ::core::ffi::c_double,
    pub measured_bank_deg: ::core::ffi::c_double,
    pub measured_bank_rate_deg_s: ::core::ffi::c_double,
    pub bank_effectiveness: ::core::ffi::c_double,
    pub lift_accel: ::core::ffi::c_double,
    pub has_measured_lift: bool,
    pub has_simulated_lift_observation: bool,
    pub authority_confidence: ::core::ffi::c_double,
    pub course_to_site_error_deg: ::core::ffi::c_double,
    pub has_crossrange_error: bool,
    pub crossrange_error_m: ::core::ffi::c_double,
    pub has_crossrange_rate: bool,
    pub crossrange_error_rate_mps: ::core::ffi::c_double,
    pub has_crossrange_accel: bool,
    pub crossrange_error_accel_mps2: ::core::ffi::c_double,
    pub crossrange_uncertainty_m: ::core::ffi::c_double,
    pub crossrange_arrest_prediction: bool,
    pub has_crossrange_course_alignment: bool,
    pub crossrange_course_alignment: ::core::ffi::c_double,
    pub has_sideslip: bool,
    pub sideslip_deg: ::core::ffi::c_double,
    pub maximum_sideslip_deg: ::core::ffi::c_double,
    pub longitudinal: EntryLateralLongitudinalDemand,
    pub has_bank_magnitude: bool,
    pub bank_magnitude_deg: ::core::ffi::c_double,
    pub has_azimuth_error: bool,
    pub azimuth_error_deg: ::core::ffi::c_double,
    pub azimuth_deadband_deg: ::core::ffi::c_double,
    pub has_time_to_go: bool,
    pub time_to_go_s: ::core::ffi::c_double,
    pub has_point_miss: bool,
    pub point_miss_m: ::core::ffi::c_double,
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
pub struct EntryLateralOutput {
    pub valid: bool,
    pub reversal_requested: bool,
    pub leg_captured: bool,
    pub reversal_armed: bool,
    pub degraded_authority: bool,
    pub bank_sign: ::core::ffi::c_double,
    pub bank_magnitude_deg: ::core::ffi::c_double,
    pub energy_bank_magnitude_deg: ::core::ffi::c_double,
    pub raw_target_bank_deg: ::core::ffi::c_double,
    pub target_bank_deg: ::core::ffi::c_double,
    pub target_bank_rate_deg_s: ::core::ffi::c_double,
    pub corridor_metric: ::core::ffi::c_double,
    pub course_corridor_deg: ::core::ffi::c_double,
    pub crossrange_corridor_m: ::core::ffi::c_double,
    pub projected_crossrange_error_m: ::core::ffi::c_double,
    pub reversal_response_time_s: ::core::ffi::c_double,
    pub required_lateral_lift_accel: ::core::ffi::c_double,
    pub zero_effort_crossrange_m: ::core::ffi::c_double,
    pub lateral_demand_binding: bool,
    pub expected_course_rate_deg_s: ::core::ffi::c_double,
    pub sideslip_rate_deg_s: ::core::ffi::c_double,
    pub coordination_roll_rate_limit_deg_s: ::core::ffi::c_double,
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
pub type TerminalVerticalStage = ::core::ffi::c_uint;
pub const TERMINAL_GROUND: TerminalVerticalStage = 5;
pub const TERMINAL_TOUCHDOWN_FLARE: TerminalVerticalStage = 4;
pub const TERMINAL_INNER_FINAL: TerminalVerticalStage = 3;
pub const TERMINAL_PREFLARE: TerminalVerticalStage = 2;
pub const TERMINAL_OUTER_FINAL: TerminalVerticalStage = 1;
pub const TERMINAL_TRAJECTORY_CAPTURE: TerminalVerticalStage = 0;
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
pub type EntryAlphaPhase = ::core::ffi::c_uint;
pub const ENTRY_ALPHA_PHASE_COUNT: EntryAlphaPhase = 5;
pub const ENTRY_ALPHA_TRANSITION: EntryAlphaPhase = 4;
pub const ENTRY_ALPHA_CONSTANT_DRAG: EntryAlphaPhase = 3;
pub const ENTRY_ALPHA_EQUILIBRIUM_GLIDE: EntryAlphaPhase = 2;
pub const ENTRY_ALPHA_TEMPERATURE_CONTROL: EntryAlphaPhase = 1;
pub const ENTRY_ALPHA_PRE_ENTRY: EntryAlphaPhase = 0;
pub type EntryAlphaLimitReason = ::core::ffi::c_uint;
pub const ENTRY_ALPHA_LIMIT_INVALID_INPUT: EntryAlphaLimitReason = 8;
pub const ENTRY_ALPHA_LIMIT_COMMAND_RATE: EntryAlphaLimitReason = 7;
pub const ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE: EntryAlphaLimitReason = 6;
pub const ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION: EntryAlphaLimitReason = 5;
pub const ENTRY_ALPHA_LIMIT_G_LOAD: EntryAlphaLimitReason = 4;
pub const ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE: EntryAlphaLimitReason = 3;
pub const ENTRY_ALPHA_LIMIT_STALL_MARGIN: EntryAlphaLimitReason = 2;
pub const ENTRY_ALPHA_LIMIT_MAX_AOA: EntryAlphaLimitReason = 1;
pub const ENTRY_ALPHA_LIMIT_NONE: EntryAlphaLimitReason = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryAlphaSchedule {
    pub point_count: size_t,
    pub relative_velocity: [::core::ffi::c_double; 8],
    pub alpha: [::core::ffi::c_double; 8],
    pub phase_modulation_scale: [::core::ffi::c_double; 5],
    pub drag_error_gain: ::core::ffi::c_double,
    pub maximum_modulation: ::core::ffi::c_double,
    pub modulation_rate_limit: ::core::ffi::c_double,
    pub target_rate_limit: ::core::ffi::c_double,
    pub minimum_protective_aoa: ::core::ffi::c_double,
    pub minimum_aero_confidence: ::core::ffi::c_double,
    pub dynamic_pressure_protect_ratio: ::core::ffi::c_double,
    pub g_load_protect_ratio: ::core::ffi::c_double,
    pub stall_fraction_limit: ::core::ffi::c_double,
    pub calibrated_stall_speed_margin: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryAlphaInput {
    pub relative_velocity: ::core::ffi::c_double,
    pub phase: EntryAlphaPhase,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub measured_drag_accel: ::core::ffi::c_double,
    pub current_aoa: ::core::ffi::c_double,
    pub current_aoa_rate: ::core::ffi::c_double,
    pub has_previous_target: bool,
    pub previous_target_aoa: ::core::ffi::c_double,
    pub has_previous_modulation: bool,
    pub previous_modulation: ::core::ffi::c_double,
    pub dt: ::core::ffi::c_double,
    pub maximum_aoa: ::core::ffi::c_double,
    pub stall_margin_aoa_limit: ::core::ffi::c_double,
    pub stall_fraction: ::core::ffi::c_double,
    pub stall_fraction_is_measured: bool,
    pub calibrated_stall_speed: ::core::ffi::c_double,
    pub minimum_safe_speed: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub maximum_dynamic_pressure: ::core::ffi::c_double,
    pub g_load: ::core::ffi::c_double,
    pub maximum_g_load: ::core::ffi::c_double,
    pub aero_confidence: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryAlphaResult {
    pub valid: bool,
    pub degraded: bool,
    pub nominal_aoa: ::core::ffi::c_double,
    pub requested_modulation: ::core::ffi::c_double,
    pub modulation: ::core::ffi::c_double,
    pub unconstrained_target_aoa: ::core::ffi::c_double,
    pub target_aoa: ::core::ffi::c_double,
    pub limiting_reason: EntryAlphaLimitReason,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryDragReferenceConfig {
    pub entry_velocity_ratio: ::core::ffi::c_double,
    pub temperature_end_progress: ::core::ffi::c_double,
    pub equilibrium_end_progress: ::core::ffi::c_double,
    pub constant_end_progress: ::core::ffi::c_double,
    pub temperature_start_drag_ratio: ::core::ffi::c_double,
    pub equilibrium_blend: ::core::ffi::c_double,
    pub equilibrium_anchor_weight: ::core::ffi::c_double,
    pub range_gain: ::core::ffi::c_double,
    pub minimum_range_authority: ::core::ffi::c_double,
    pub minimum_drag_scale: ::core::ffi::c_double,
    pub maximum_drag_scale: ::core::ffi::c_double,
    pub range_error_scale_fraction: ::core::ffi::c_double,
    pub minimum_range_error_scale: ::core::ffi::c_double,
    pub vertical_drag_gain: ::core::ffi::c_double,
    pub vertical_range_gain: ::core::ffi::c_double,
    pub maximum_vertical_lift_correction: ::core::ffi::c_double,
    pub maximum_drag_g_fraction: ::core::ffi::c_double,
    pub local_drag_authority_multiplier: ::core::ffi::c_double,
    pub minimum_local_drag_confidence: ::core::ffi::c_double,
    pub minimum_drag_accel: ::core::ffi::c_double,
    pub prediction_steps: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryDragReferenceInput {
    pub phase: EntryPhase,
    pub relative_velocity: ::core::ffi::c_double,
    pub latitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub state: *const VehicleState,
    pub measured_drag_accel: ::core::ffi::c_double,
    pub modeled_drag_accel: ::core::ffi::c_double,
    pub aero_confidence: ::core::ffi::c_double,
    pub has_incidence: bool,
    pub angle_of_attack: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub range_to_site: ::core::ffi::c_double,
    pub taem_range: ::core::ffi::c_double,
    pub taem_latitude: ::core::ffi::c_double,
    pub taem_altitude: ::core::ffi::c_double,
    pub taem_velocity: ::core::ffi::c_double,
    pub planet: *const PlanetModel,
    pub vehicle: *const VehicleProfile,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryDragReferenceOutput {
    pub valid: bool,
    pub active: bool,
    pub degraded: bool,
    pub confidence: ::core::ffi::c_double,
    pub entry_anchor_velocity: ::core::ffi::c_double,
    pub profile_progress: ::core::ffi::c_double,
    pub range_authority: ::core::ffi::c_double,
    pub required_average_drag_accel: ::core::ffi::c_double,
    pub equilibrium_drag_accel: ::core::ffi::c_double,
    pub constant_drag_accel: ::core::ffi::c_double,
    pub taem_drag_accel: ::core::ffi::c_double,
    pub nominal_drag_accel: ::core::ffi::c_double,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub minimum_achievable_drag_accel: ::core::ffi::c_double,
    pub drag_floor_active: bool,
    pub observed_drag_accel: ::core::ffi::c_double,
    pub drag_error_accel: ::core::ffi::c_double,
    pub drag_error_fraction: ::core::ffi::c_double,
    pub available_range_to_taem: ::core::ffi::c_double,
    pub unshaped_predicted_range: ::core::ffi::c_double,
    pub predicted_range_to_taem: ::core::ffi::c_double,
    pub unshaped_range_error: ::core::ffi::c_double,
    pub predicted_range_error: ::core::ffi::c_double,
    pub range_error_scale: ::core::ffi::c_double,
    pub drag_range_scale: ::core::ffi::c_double,
    pub reference_drag_dv: ::core::ffi::c_double,
    pub predicted_range_d_drag: ::core::ffi::c_double,
    pub required_vertical_lift_accel: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const MM304_PLANNED_MAX_BANK_DEG: ::core::ffi::c_double = 40.0f64;
pub const MM304_ENERGY_SHORT_CROSSRANGE_TOLERANCE_M: ::core::ffi::c_double = 15000.0f64;
#[no_mangle]
pub unsafe extern "C" fn guidance_accept_entry_plan(
    mut g: *mut GuidanceMachine,
    mut request: *const GuidanceMachine,
    mut result: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
) -> bool {
    if g.is_null() || request.is_null() || result.is_null() || t.is_null() {
        return false_0 != 0;
    }
    if (*result).phase as ::core::ffi::c_uint != (*request).phase as ::core::ffi::c_uint
        || !(*g).automation_engaged || (*g).paused as ::core::ffi::c_int != 0
        || (*g).aborted as ::core::ffi::c_int != 0
        || (*g).phase as ::core::ffi::c_uint
            != PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*g).control_plan_sequence != (*request).control_plan_sequence
    {
        return false_0 != 0;
    }
    let mut age: ::core::ffi::c_double = (*t).ut - (*request).previous_ut;
    if age.is_finite() as i32 == 0 || age < 0.0f64 {
        return false_0 != 0;
    }
    (*g).entry_supervision_ut = (*request).previous_ut;
    (*g).entry_supervision_valid = (*result).entry_supervision_valid;
    (*g).entry_supervision_mode = (*result).entry_supervision_mode;
    (*g).entry_committed_infeasible = (*result).entry_committed_infeasible;
    (*g).entry_supervision_boundary_missed = (*result).entry_supervision_boundary_missed;
    if (*result).entry_s_turn_plan.valid {
        (*g).entry_s_turn_plan.valid = true_0 != 0;
        (*g).entry_s_turn_plan.terminal_ready = (*result)
            .entry_s_turn_plan
            .terminal_ready;
        (*g).entry_s_turn_plan.closest_distance = (*result)
            .entry_s_turn_plan
            .closest_distance;
        (*g).entry_s_turn_plan.taem_range_error = (*result)
            .entry_s_turn_plan
            .taem_range_error;
        (*g).entry_s_turn_plan.taem_speed = (*result).entry_s_turn_plan.taem_speed;
        (*g).entry_s_turn_plan.taem_energy_error = (*result)
            .entry_s_turn_plan
            .taem_energy_error;
        (*g).entry_s_turn_plan.predicted_reversals = (*result)
            .entry_s_turn_plan
            .predicted_reversals;
        (*g).entry_s_turn_plan.cost = (*result).entry_s_turn_plan.cost;
    }
    if (*result).entry_policy_valid as ::core::ffi::c_int != 0
        && (*result).entry_policy_bank_deg.is_finite() as i32 != 0
    {
        (*g).entry_policy_valid = true_0 != 0;
        (*g).entry_policy_bank_deg = (*result).entry_policy_bank_deg;
        (*g).entry_policy_aoa_deg = (*result).entry_policy_aoa_deg;
        (*g).entry_policy_ut = (*result).entry_policy_ut;
    }
    if crate::mm304::live::accept(&mut (*g).mm304, &(*result).mm304) && !(*g).diagnostic_shadow {
        let mut pl = (*g).mm304.plan;
        fprintf(
            stderr,
            b"MM304 plan #%u %s route=%s: ut=%.1f sigma=%+.3f bank=%.1f aoaBias=%+.1f E=%+.0f target=%.0f band=%.0f unc=%.0f Emaxrange=%+.0f Emaxbank=%+.0f limit=%s forecasts=%u revPred=%u conv=%d env=%d handoff along=%.0f cross=%.0f hdgErr=%+.1f h=%.0f V=%.0f spiral=%.0f spiralAtMaxRange=%+.0f/%.0fdeg\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            pl.sequence,
            crate::mm304::telemetry::decision_cname(pl.decision),
            if pl.route == crate::mm304::state::MM304_ROUTE_SHORT {
                b"short\0" as *const u8 as *const ::core::ffi::c_char
            } else {
                b"spiral\0" as *const u8 as *const ::core::ffi::c_char
            },
            pl.ut,
            pl.sigma,
            pl.bank,
            pl.aoa_bias,
            pl.predicted_energy,
            pl.target_energy,
            pl.terminal.band,
            pl.uncertainty,
            pl.energy_at_min,
            pl.energy_at_max,
            crate::mm304::telemetry::limiting_cname(pl.limiting),
            pl.forecasts,
            pl.reversals_predicted,
            pl.converged as ::core::ffi::c_int,
            pl.in_envelope as ::core::ffi::c_int,
            pl.terminal.along,
            pl.terminal.cross,
            pl.terminal.heading_error,
            pl.terminal.altitude,
            pl.terminal.speed,
            pl.terminal.spiral_turn,
            pl.spiral_at_max_range,
            pl.spiral_turn_at_max_range,
        );
    }
    (*g).entry_planning_needed = false_0 != 0;
    return true_0 != 0;
}
pub const MM304_COORDINATION_BETA_SCALE_DEG: ::core::ffi::c_double = 5.0f64;
unsafe extern "C" fn entry_alpha_phase_for_exec(
    mut phase: EntryPhase,
) -> EntryAlphaPhase {
    match phase as ::core::ffi::c_uint {
        1 => return ENTRY_ALPHA_PRE_ENTRY,
        2 => return ENTRY_ALPHA_TEMPERATURE_CONTROL,
        3 => return ENTRY_ALPHA_EQUILIBRIUM_GLIDE,
        4 => return ENTRY_ALPHA_CONSTANT_DRAG,
        5 => return ENTRY_ALPHA_TRANSITION,
        _ => return ENTRY_ALPHA_PRE_ENTRY,
    };
}
unsafe extern "C" fn entry_alpha_transition_velocity(
    mut s: *const GuidanceSettings,
    mut taem_velocity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if !s.is_null() {
        fmax(taem_velocity, (*s).taem_force_handoff_speed)
    } else {
        taem_velocity
    };
}
#[no_mangle]
pub unsafe extern "C" fn live_lift_accel(
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).mass.is_finite() as i32 != 0 && (*t).mass > DBL_MIN
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force >= 0.0f64
    {
        return (*t).lift_force / (*t).mass;
    }
    let mut incidence: ::core::ffi::c_double = hypot(
        (*t).angle_of_attack,
        (*t).sideslip,
    );
    let mut lf: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        incidence,
        v,
        &raw mut lf,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    if aero.ballistic_coefficient.is_finite() as i32 == 0
        || aero.ballistic_coefficient <= DBL_MIN
        || aero.lift_to_drag.is_finite() as i32 == 0 || aero.lift_to_drag < 0.0f64
    {
        return 0.0f64;
    }
    return fmax(0.0f64, (*t).dynamic_pressure) / aero.ballistic_coefficient
        * aero.lift_to_drag * fabs(lf);
}
#[no_mangle]
pub unsafe extern "C" fn guidance_feedback_lift_accel(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if !g.is_null() && (*g).diagnostic_shadow as ::core::ffi::c_int != 0 && !t.is_null()
        && (*t).mass.is_finite() as i32 != 0 && (*t).mass > DBL_MIN
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force >= 0.0f64
    {
        return (*t).lift_force / (*t).mass;
    }
    return live_lift_accel(t, aero, v);
}
#[no_mangle]
pub unsafe extern "C" fn live_drag_accel(
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).mass.is_finite() as i32 != 0 && (*t).mass > DBL_MIN
        && (*t).drag_force.is_finite() as i32 != 0 && (*t).drag_force >= 0.0f64
    {
        return (*t).drag_force / (*t).mass;
    }
    let mut incidence: ::core::ffi::c_double = hypot(
        (*t).angle_of_attack,
        (*t).sideslip,
    );
    let mut df: ::core::ffi::c_double = 1.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        incidence,
        v,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut df,
    );
    if aero.ballistic_coefficient.is_finite() as i32 == 0
        || aero.ballistic_coefficient <= DBL_MIN
    {
        return 0.0f64;
    }
    return fmax(0.0f64, (*t).dynamic_pressure) / aero.ballistic_coefficient
        * fmax(0.0f64, df);
}
#[no_mangle]
pub unsafe extern "C" fn entry_measured_drag_accel(
    mut t: *const Telemetry,
) -> ::core::ffi::c_double {
    if t.is_null() || !(*t).physics_sample_valid || (*t).mass.is_finite() as i32 == 0
        || (*t).mass <= DBL_MIN || (*t).drag_force.is_finite() as i32 == 0
        || (*t).drag_force < 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return (*t).drag_force / (*t).mass;
}
unsafe extern "C" fn entry_diagnostic_milestone_due(
    mut g: *mut GuidanceMachine,
    mut range_m: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    static mut milestone_m: [::core::ffi::c_double; 4] = [
        220000.0f64,
        170000.0f64,
        140000.0f64,
        100000.0f64,
    ];
    let mut trace: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TRACE_ENTRY_MILESTONES\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if g.is_null() || !(*g).diagnostic_shadow || trace.is_null()
        || strcmp(trace, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            != 0 as ::core::ffi::c_int || range_m.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut previous: ::core::ffi::c_double = if (*g).diagnostic_entry_has_previous_range
        as ::core::ffi::c_int != 0
    {
        (*g).diagnostic_entry_previous_range_m
    } else {
        range_m
    };
    (*g).diagnostic_entry_previous_range_m = range_m;
    (*g).diagnostic_entry_has_previous_range = true_0 != 0;
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while (i as usize)
        < (::core::mem::size_of::<[::core::ffi::c_double; 4]>() as usize)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
    {
        let mut bit: ::core::ffi::c_uint = (1 as ::core::ffi::c_uint) << i;
        if (*g).diagnostic_entry_milestone_mask & bit == 0
            && previous > milestone_m[i as usize] && range_m <= milestone_m[i as usize]
        {
            (*g).diagnostic_entry_milestone_mask |= bit;
            return milestone_m[i as usize];
        }
        i = i.wrapping_add(1);
    }
    return ::core::f32::NAN as ::core::ffi::c_double;
}
unsafe extern "C" fn entry_aoa_staircase_test(
    mut mach: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut spec: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_ENTRY_AOA_STAIRCASE_TEST\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    if spec.is_null() || *spec == 0 || mach.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut p: *const ::core::ffi::c_char = spec;
    while !p.is_null() && *p as ::core::ffi::c_int != 0 {
        let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
            ::core::ffi::c_char,
        >();
        let mut threshold: ::core::ffi::c_double = strtod(p, &raw mut end);
        if end == p as *mut ::core::ffi::c_char
            || *end as ::core::ffi::c_int != ':' as i32
        {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        p = end.offset(1 as ::core::ffi::c_int as isize);
        let mut aoa: ::core::ffi::c_double = strtod(p, &raw mut end);
        if end == p as *mut ::core::ffi::c_char {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        if mach >= threshold {
            return aoa;
        }
        p = if *end as ::core::ffi::c_int == ',' as i32 {
            end.offset(1 as ::core::ffi::c_int as isize)
        } else {
            ::core::ptr::null_mut::<::core::ffi::c_char>()
        };
    }
    return ::core::f32::NAN as ::core::ffi::c_double;
}
unsafe extern "C" fn entry_max_lift_to_drag_aoa(
    mut m: *const TerminalModel,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut ceiling: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if m.is_null() || mach.is_finite() as i32 == 0 || ceiling.is_finite() as i32 == 0
        || ceiling <= 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut best: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut best_ld: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut a: ::core::ffi::c_double = 4.0f64;
    while a <= ceiling + 1e-9f64 {
        let mut lq: ::core::ffi::c_double = 0.0f64;
        let mut dq: ::core::ffi::c_double = 0.0f64;
        aero_force_per_q(
            &raw const (*m).aero,
            fmax(q, 1.0f64),
            mach,
            a,
            &raw mut lq,
            &raw mut dq,
        );
        if dq > DBL_MIN && lq / dq > best_ld {
            best_ld = lq / dq;
            best = a;
        }
        a += 0.5f64;
    }
    return best;
}
unsafe extern "C" fn entry_program_contract_hold(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
    mut dt: ::core::ffi::c_double,
    mut warning: *const ::core::ffi::c_char,
) -> GuidanceResult {
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut bank_limit: ::core::ffi::c_double = dynamic_bank_limit(t, v);
    let mut have_previous: bool = (*g).entry_control_plan_valid as ::core::ffi::c_int
        != 0 && (*g).entry_control_bank.is_finite() as i32 != 0
        && (*g).entry_control_aoa.is_finite() as i32 != 0
        && (*g).entry_control_heading.is_finite() as i32 != 0;
    let mut bank: ::core::ffi::c_double = if have_previous as ::core::ffi::c_int != 0 {
        (*g).entry_control_bank
    } else {
        norm_signed_deg((*t).roll)
    };
    let mut aoa: ::core::ffi::c_double = if have_previous as ::core::ffi::c_int != 0 {
        (*g).entry_control_aoa
    } else {
        (*t).angle_of_attack
    };
    let mut heading: ::core::ffi::c_double = if have_previous as ::core::ffi::c_int != 0
    {
        (*g).entry_control_heading
    } else if (*t).ground_track_heading.is_finite() as i32 != 0 {
        (*t).ground_track_heading
    } else {
        (*t).heading
    };
    if bank.is_finite() as i32 == 0 {
        bank = 0.0f64;
    }
    if aoa.is_finite() as i32 == 0 {
        aoa = (*v).entry_angle_of_attack;
    }
    if heading.is_finite() as i32 == 0 {
        heading = (*cfg).site.runway_heading;
    }
    bank = clampd(bank, -bank_limit, bank_limit);
    aoa = clampd(aoa, 0.0f64, (*v).maximum_angle_of_attack);
    (*g).entry_control_terminal_ready = false_0 != 0;
    (*g).entry_planning_needed = true_0 != 0;
    (*g).airbrakes_deployed = false_0 != 0;
    let mut c: GuidanceCommand = atmospheric(
        t,
        heading,
        bank,
        v,
        0.0f64,
        false_0 != 0,
        PROFILE_ENTRY,
    );
    c.heading_control_enabled = false_0 != 0;
    c.has_target_aoa = true_0 != 0;
    c.target_aoa = aoa;
    c.target_pitch = (*t).flight_path_angle + aoa;
    let mut milestone: ::core::ffi::c_double = entry_diagnostic_milestone_due(
        g,
        (*t).range_to_site,
    );
    if milestone.is_finite() as i32 != 0 {
        fprintf(
            stderr,
            b"MM304_SHADOW_MILESTONE hold=1 milestone=%.0f ut=%.2f h=%.0f V=%.1f M=%.2f fpa=%.2f range=%.0f status=\"%s\" previousBank=%.1f previousAoA=%.1f feedbackValid=%d lateralValid=%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            milestone,
            (*t).ut,
            (*t).mean_altitude,
            (*t).true_air_speed,
            (*t).mach,
            (*t).flight_path_angle,
            (*t).range_to_site,
            if !warning.is_null() {
                warning
            } else {
                b"\0" as *const u8 as *const ::core::ffi::c_char
            },
            bank,
            aoa,
            if (*g).entry_feedback.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if (*g).entry_feedback.lateral_valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
        );
    }
    return stabilized(
        g,
        result_make(
            PHASE_ENTRY_ENERGY,
            c,
            b"MM304 feedback model unavailable; retaining the bounded current Entry command.\0"
                as *const u8 as *const ::core::ffi::c_char,
            warning,
        ),
        t,
        v,
        s,
        dt,
    );
}
#[no_mangle]
pub unsafe extern "C" fn entry_program_guidance(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    (*g).entry_reversal_scheduled = false_0 != 0;
    (*g).entry_reversal_is_final = false_0 != 0;
    (*g).entry_reversal_ut = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).entry_reversal_range = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).entry_reversal_sign = 0.0f64;
    (*g).entry_reversal_bank = 0.0f64;
    (*g).entry_final_reversal_pending = false_0 != 0;
    (*g).entry_final_reversal_completed = false_0 != 0;
    (*g).entry_lateral_infeasible = false_0 != 0;
    (*g).entry_geometry_bank = 0.0f64;
    (*g).entry_lateral_required_bank = 0.0f64;
    (*g).taem_interface_target = TaemInterfaceTarget {
        valid: 0 as ::core::ffi::c_int != 0,
        energy_qualified: false,
        along_track: 0.,
        cross_track: 0.,
        course: 0.,
        altitude: 0.,
        speed: 0.,
        flight_path_angle: 0.,
        specific_energy: 0.,
        selected_ut: 0.,
        arrival_ut: 0.,
        quality_score: 0.,
        acquisition_lead: 0.,
        remaining_path: 0.,
        response_time: 0.,
        hac_radius: 0.,
    };
    let mut taem_velocity: ::core::ffi::c_double = entry_taem_speed_target(v, s, p);
    if taem_velocity.is_finite() as i32 == 0 || taem_velocity <= 0.0f64 {
        return entry_program_contract_hold(
            g,
            t,
            cfg,
            dt,
            b"MM304 has no finite TAEM velocity target from the active vehicle/planet model.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    let mut phase: EntryPhase = (if (*g).entry_exec.initialized as ::core::ffi::c_int
        != 0
    {
        (*g).entry_exec.phase as ::core::ffi::c_uint
    } else {
        ENTRY_PHASE_PREENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
    }) as EntryPhase;
    let mut drag_cfg: EntryDragReferenceConfig = entry_drag_reference_default_config();
    let mut measured_drag: ::core::ffi::c_double = entry_measured_drag_accel(t);
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
    let mut alignment_altitude: ::core::ffi::c_double = alignment.altitude_m;
    let mut alignment_speed: ::core::ffi::c_double = alignment.speed_mps;
    let mut ground_radius: ::core::ffi::c_double = (*p).radius + (*cfg).site.altitude;
    let mut alignment_point: GeoPoint = runway_approach_aimpoint(
        &raw const (*cfg).site,
        ground_radius,
        alignment.distance_m,
    );
    let mut current_point: GeoPoint = GeoPoint {
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        altitude: (*t).mean_altitude,
    };
    let mut alignment_range: ::core::ffi::c_double = great_circle_distance(
        current_point,
        alignment_point,
        ground_radius,
    );
    let mut delivery_site: LandingSite = (*cfg).site;
    let mut sts_delivery: bool = (*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    let mut hac_delivery: bool = (*s).taem_hac_law as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    if (sts_delivery as ::core::ffi::c_int != 0
        || hac_delivery as ::core::ffi::c_int != 0)
        && (*cfg).site.allow_reciprocal_runway as ::core::ffi::c_int != 0
    {
        let mut base: LandingSite = if (*g).runway_end_preview_valid
            as ::core::ffi::c_int != 0
            && (*g).runway_end_index == 1 as ::core::ffi::c_int
        {
            runway_reciprocal_site(&raw const (*cfg).site, ground_radius)
        } else {
            (*cfg).site
        };
        let mut ends: [LandingSite; 2] = [
            base,
            runway_reciprocal_site(&raw mut base, ground_radius),
        ];
        let mut aims: [GeoPoint; 2] = [GeoPoint {
            latitude: 0.,
            longitude: 0.,
            altitude: 0.,
        }; 2];
        let mut turn: [::core::ffi::c_double; 2] = [0.; 2];
        let mut range: [::core::ffi::c_double; 2] = [0.; 2];
        let mut e: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while e < 2 as ::core::ffi::c_int {
            aims[e as usize] = runway_approach_aimpoint(
                (&raw mut ends as *mut LandingSite).offset(e as isize)
                    as *mut LandingSite,
                ground_radius,
                alignment.distance_m,
            );
            range[e as usize] = great_circle_distance(
                current_point,
                aims[e as usize],
                ground_radius,
            );
            turn[e as usize] = fabs(
                norm_signed_deg(
                    ends[e as usize].runway_heading
                        - initial_bearing(current_point, aims[e as usize]),
                ),
            );
            e += 1;
        }
        let mut end: ::core::ffi::c_int = if (*g).runway_end_preview_valid
            as ::core::ffi::c_int != 0
        {
            (*g).runway_end_index
        } else {
            0 as ::core::ffi::c_int
        };
        if range[end as usize] > entry_taem_range_target(p, s)
            || !(*g).runway_end_preview_valid
        {
            let mut best: ::core::ffi::c_int = if !(*g).runway_end_preview_valid {
                if turn[1 as ::core::ffi::c_int as usize]
                    < turn[0 as ::core::ffi::c_int as usize]
                {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                }
            } else if turn[end as usize] > 90.0f64
                && turn[(1 as ::core::ffi::c_int - end) as usize] < turn[end as usize]
            {
                1 as ::core::ffi::c_int - end
            } else {
                end
            };
            if best != end || !(*g).runway_end_preview_valid {
                fprintf(
                    stderr,
                    b"MM304 delivery runway end: UT %.1f RW%s turn09=%.1f turn27=%.1f range=%.0f\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*t).ut,
                    if best == 1 as ::core::ffi::c_int {
                        b"27\0" as *const u8 as *const ::core::ffi::c_char
                    } else {
                        b"09\0" as *const u8 as *const ::core::ffi::c_char
                    },
                    turn[0 as ::core::ffi::c_int as usize],
                    turn[1 as ::core::ffi::c_int as usize],
                    range[best as usize],
                );
            }
            end = best;
            (*g).runway_end_index = end;
            (*g).runway_end_preview_valid = true_0 != 0;
        }
        alignment_point = aims[end as usize];
        alignment_range = range[end as usize];
        delivery_site = ends[end as usize];
    }
    if !alignment.valid || alignment_range.is_finite() as i32 == 0
        || alignment_altitude.is_finite() as i32 == 0
    {
        return entry_program_contract_hold(
            g,
            t,
            cfg,
            dt,
            b"MM304 alignment energy contract has invalid geometry.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    let mut energy_range: ::core::ffi::c_double = alignment_range;
    if (*t).true_air_speed <= (*s).taem_force_handoff_speed
        && alignment_range <= entry_taem_range_target(p, s)
        && (*t).runway_along_track.is_finite() as i32 != 0
        && (*t).runway_cross_track.is_finite() as i32 != 0
        && course.is_finite() as i32 != 0
    {
        let mut turn_path: ::core::ffi::c_double = decision_target_path_length(
            g,
            t,
            course,
            -alignment.distance_m,
            0.0f64,
            (*cfg).site.runway_heading,
            ::core::f32::NAN as ::core::ffi::c_double,
            p,
            cfg,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
        if turn_path.is_finite() as i32 != 0 {
            energy_range = fmax(energy_range, turn_path);
        }
    }
    let mut drag_in: EntryDragReferenceInput = EntryDragReferenceInput {
        phase: phase,
        relative_velocity: (*t).true_air_speed,
        latitude: (*t).latitude,
        altitude: (*t).mean_altitude,
        state: state,
        measured_drag_accel: measured_drag,
        modeled_drag_accel: live_drag_accel(t, aero, v),
        aero_confidence: clampd(
            fmax(aero.confidence, (*t).aerodynamic_confidence),
            0.0f64,
            1.0f64,
        ),
        has_incidence: (*t).angle_of_attack.is_finite() as i32 != 0
            && (*t).sideslip.is_finite() as i32 != 0,
        angle_of_attack: (*t).angle_of_attack,
        sideslip: (*t).sideslip,
        range_to_site: energy_range,
        taem_range: 0.0f64,
        taem_latitude: (*cfg).site.latitude,
        taem_altitude: alignment_altitude,
        taem_velocity: alignment_speed,
        planet: p,
        vehicle: v,
    };
    let mut hac_h: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut hac_v: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut hac_range: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0
        && !(*g).entry_terminal_model.is_null()
    {
        mm305_hac_observe_aero(g, t);
        let mut e_ref: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
        (*g).entry_hac_energy_excess = if mm305_hac_reference_energy(
            (*g).entry_terminal_model as *const TerminalModel,
            &raw mut (*g).taem_hac_live_aero,
            (*t).mass,
            fmax(0.0f64, energy_range),
            &raw mut e_ref,
        ) as ::core::ffi::c_int != 0
        {
            (*t).mean_altitude - (*cfg).site.altitude
                + (*t).true_air_speed * (*t).true_air_speed
                    / (2.0f64 * planet_surface_gravity(p)) - e_ref
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
        (*g).entry_hac_energy_ut = (*t).ut;
        if mm305_hac_handoff_state(
            (*g).entry_terminal_model as *const TerminalModel,
            &raw mut (*g).taem_hac_live_aero,
            (*t).mass,
            (*s).mm305_target_mach,
            &raw mut hac_h,
            &raw mut hac_v,
            &raw mut hac_range,
        ) as ::core::ffi::c_int != 0 && (*t).true_air_speed > hac_v
        {
            drag_in.taem_velocity = hac_v;
            drag_in.taem_altitude = hac_h + (*cfg).site.altitude;
            let mut target: ::core::ffi::c_double = hac_range + alignment.distance_m;
            if !(*g).diagnostic_shadow
                && !(fabs(target - (*g).entry_handoff_target_range) < 2000.0f64)
            {
                fprintf(
                    stderr,
                    b"MM304 HAC handoff target: UT %.1f Mach %.2f h %.0f m V %.0f m/s station range %.1f km, site %.1f km\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*t).ut,
                    (*s).mm305_target_mach,
                    hac_h,
                    hac_v,
                    hac_range / 1000.0f64,
                    target / 1000.0f64,
                );
            }
            (*g).entry_handoff_target_range = target;
            drag_in.taem_range = hac_range;
        }
    } else if (*cfg).guidance.taem_energy_law as ::core::ffi::c_int != 0
        && !(*g).entry_terminal_model.is_null()
    {
        let mut interface_altitude: ::core::ffi::c_double = (*s).taem_interface_altitude;
        let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
            p,
            interface_altitude,
        );
        let mut handoff_speed: ::core::ffi::c_double = ((*s).mm305_target_mach
            + (if sts_delivery as ::core::ffi::c_int != 0 {
                0.0f64
            } else {
                (*s).mm305_mach_half_width
            })) * sound;
        let mut reach: ::core::ffi::c_double = mm305_energy_law_handoff_reach(
            (*g).entry_terminal_model as *const TerminalModel,
            interface_altitude - (*cfg).site.altitude,
            handoff_speed,
            (*t).mass,
        );
        if handoff_speed.is_finite() as i32 != 0 && handoff_speed > 0.0f64
            && reach.is_finite() as i32 != 0 && (*t).true_air_speed > handoff_speed
        {
            drag_in.taem_velocity = handoff_speed;
            drag_in.taem_altitude = interface_altitude;
            (*g).entry_handoff_target_range = reach + alignment.distance_m;
            drag_in.taem_range = reach;
        }
    }
    let mut drag: EntryDragReferenceOutput = entry_drag_reference_compute(
        &raw mut drag_in,
        &raw mut drag_cfg,
    );
    if !drag.valid {
        return entry_program_contract_hold(
            g,
            t,
            cfg,
            dt,
            b"MM304 drag-reference state is invalid; Entry ownership is retained.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    let mut phase_in: EntryDragReferenceInput = drag_in;
    phase_in.taem_velocity = taem_velocity;
    phase_in.taem_altitude = (*s).taem_interface_altitude;
    let mut exec_profile: EntryExecProfile = entry_drag_reference_exec_profile(
        &raw mut phase_in,
        &raw mut drag_cfg,
        &raw mut drag,
    );
    let mut checkpoint_restart: bool = !(*g).entry_exec.initialized
        && (*g).entry_continuation_bootstrap as ::core::ffi::c_int != 0;
    let mut obs: EntryExecObservation = EntryExecObservation {
        ut: (*t).ut,
        relative_velocity: (*t).true_air_speed,
        altitude: (*t).mean_altitude,
        dynamic_pressure: fmax(0.0f64, (*t).dynamic_pressure),
        checkpoint_restart: checkpoint_restart,
    };
    if !entry_exec_update(
        &raw mut (*g).entry_exec,
        &raw mut obs,
        &raw mut exec_profile,
    ) {
        return entry_program_contract_hold(
            g,
            t,
            cfg,
            dt,
            b"MM304 executive rejected the measured drag-reference state.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if (*g).entry_exec.phase as ::core::ffi::c_uint
        != drag_in.phase as ::core::ffi::c_uint
    {
        drag_in.phase = (*g).entry_exec.phase;
        drag = entry_drag_reference_compute(&raw mut drag_in, &raw mut drag_cfg);
        if !drag.valid {
            return entry_program_contract_hold(
                g,
                t,
                cfg,
                dt,
                b"MM304 phase transition invalidated the drag-reference state.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
    }
    let mut admission: TaemInterfaceCapture = TaemInterfaceCapture {
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
    let mut quick_veto: ::core::ffi::c_uint = entry_mm305_admission_prefilter_veto(
        t,
        p,
        cfg,
    );
    if (*g).diagnostic_shadow as ::core::ffi::c_int != 0
        && (*g).diagnostic_stop_at_taem as ::core::ffi::c_int != 0 || quick_veto != 0
    {
        admission.veto = quick_veto;
    } else {
        admission = entry_mm305_admission_envelope(g, t, course, p, cfg);
    }
    let mut diagnostic_milestone: ::core::ffi::c_double = entry_diagnostic_milestone_due(
        g,
        (*t).range_to_site,
    );
    let mut handoff_ready: bool = admission.valid as ::core::ffi::c_int != 0
        && admission.ready as ::core::ffi::c_int != 0;
    let mut transition_velocity: ::core::ffi::c_double = entry_alpha_transition_velocity(
        s,
        taem_velocity,
    );
    let mut measured_drag_accel: ::core::ffi::c_double = if measured_drag.is_finite()
        as i32 != 0
    {
        measured_drag
    } else {
        live_drag_accel(t, aero, v)
    };
    let mut alpha_schedule: EntryAlphaSchedule = EntryAlphaSchedule {
        point_count: 0,
        relative_velocity: [0.; 8],
        alpha: [0.; 8],
        phase_modulation_scale: [0.; 5],
        drag_error_gain: 0.,
        maximum_modulation: 0.,
        modulation_rate_limit: 0.,
        target_rate_limit: 0.,
        minimum_protective_aoa: 0.,
        minimum_aero_confidence: 0.,
        dynamic_pressure_protect_ratio: 0.,
        g_load_protect_ratio: 0.,
        stall_fraction_limit: 0.,
        calibrated_stall_speed_margin: 0.,
    };
    entry_alpha_schedule_default(&raw mut alpha_schedule, v, transition_velocity);
    let mut yaw_use: ::core::ffi::c_double = if (*t).has_controls as ::core::ffi::c_int
        != 0 && (*t).control_yaw.is_finite() as i32 != 0
    {
        clampd(fabs((*t).control_yaw), 0.0f64, 1.0f64)
    } else {
        0.0f64
    };
    let mut alpha_ceiling: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    if yaw_use > 0.0f64 && (*t).angle_of_attack.is_finite() as i32 != 0 {
        let mut nominal: ::core::ffi::c_double = entry_alpha_schedule_interpolate(
            &raw mut alpha_schedule,
            (*t).true_air_speed,
        );
        let mut hold: ::core::ffi::c_double = if nominal.is_finite() as i32 != 0 {
            fmin((*t).angle_of_attack, nominal)
        } else {
            (*t).angle_of_attack
        };
        alpha_ceiling = alpha_ceiling + (hold - alpha_ceiling) * yaw_use;
    }
    alpha_ceiling = fmax(alpha_ceiling, alpha_schedule.minimum_protective_aoa);
    let mut alpha_in: EntryAlphaInput = EntryAlphaInput {
        relative_velocity: (*t).true_air_speed,
        phase: entry_alpha_phase_for_exec((*g).entry_exec.phase),
        reference_drag_accel: drag.reference_drag_accel,
        measured_drag_accel: measured_drag_accel,
        current_aoa: (*t).angle_of_attack,
        current_aoa_rate: controlled_aoa_rate(t),
        has_previous_target: (*g).entry_alpha_has_target,
        previous_target_aoa: (*g).entry_alpha_target,
        has_previous_modulation: (*g).entry_alpha_has_modulation,
        previous_modulation: (*g).entry_alpha_modulation,
        dt: fmax(DBL_EPSILON, dt),
        maximum_aoa: alpha_ceiling,
        stall_margin_aoa_limit: ::core::f32::NAN as ::core::ffi::c_double,
        stall_fraction: (*t).stall_fraction,
        stall_fraction_is_measured: (*t).stall_fraction_is_measured,
        calibrated_stall_speed: (*t).calibrated_stall_speed,
        minimum_safe_speed: (*v).minimum_safe_speed,
        dynamic_pressure: (*t).dynamic_pressure,
        maximum_dynamic_pressure: (*v).maximum_dynamic_pressure,
        g_load: (*t).g_force,
        maximum_g_load: (*v).maximum_g_load,
        aero_confidence: clampd(
            fmax(drag.confidence, (*t).aerodynamic_confidence),
            0.0f64,
            1.0f64,
        ),
    };
    let mut alpha: EntryAlphaResult = entry_alpha_command(
        &raw mut alpha_schedule,
        &raw mut alpha_in,
    );
    if !alpha.valid {
        return entry_program_contract_hold(
            g,
            t,
            cfg,
            dt,
            b"MM304 AoA schedule is invalid; Entry ownership is retained.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*g).entry_alpha_has_target = true_0 != 0;
    (*g).entry_alpha_target = alpha.target_aoa;
    (*g).entry_alpha_has_modulation = true_0 != 0;
    (*g).entry_alpha_modulation = alpha.modulation;
    let mut limits: EntryLateralLimits = entry_lateral_default_limits();
    limits.maximum_bank_deg = fmin(dynamic_bank_limit(t, v), MM304_PLANNED_MAX_BANK_DEG);
    limits.maximum_roll_rate_deg_s = (*s).entry_roll_rate;
    limits.maximum_roll_accel_deg_s2 = (*s).entry_roll_acceleration;
    let mut reserve: ::core::ffi::c_double = fmax(DBL_EPSILON, 1.0f64 - yaw_use);
    limits.maximum_roll_rate_deg_s *= reserve;
    limits.maximum_roll_accel_deg_s2 *= reserve;
    let mut lateral: EntryLateralInput = EntryLateralInput {
        ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        dt: 0.,
        relative_speed: 0.,
        measured_bank_deg: 0.,
        measured_bank_rate_deg_s: 0.,
        bank_effectiveness: 0.,
        lift_accel: 0.,
        has_measured_lift: false,
        has_simulated_lift_observation: false,
        authority_confidence: 0.,
        course_to_site_error_deg: 0.,
        has_crossrange_error: false,
        crossrange_error_m: 0.,
        has_crossrange_rate: false,
        crossrange_error_rate_mps: 0.,
        has_crossrange_accel: false,
        crossrange_error_accel_mps2: 0.,
        crossrange_uncertainty_m: 0.,
        crossrange_arrest_prediction: false,
        has_crossrange_course_alignment: false,
        crossrange_course_alignment: 0.,
        has_sideslip: false,
        sideslip_deg: 0.,
        maximum_sideslip_deg: 0.,
        longitudinal: EntryLateralLongitudinalDemand {
            valid: false,
            confidence: 0.,
            reference_drag_accel: 0.,
            drag_error_accel: 0.,
            predicted_range_error: 0.,
            range_error_scale: 0.,
            required_vertical_lift_accel: 0.,
        },
        has_bank_magnitude: false,
        bank_magnitude_deg: 0.,
        has_azimuth_error: false,
        azimuth_error_deg: 0.,
        azimuth_deadband_deg: 0.,
        has_time_to_go: false,
        time_to_go_s: 0.,
        has_point_miss: false,
        point_miss_m: 0.,
    };
    lateral.ut = (*t).ut;
    lateral.dt = fmax(DBL_EPSILON, dt);
    lateral.relative_speed = (*t).true_air_speed;
    lateral.measured_bank_deg = norm_signed_deg((*t).roll);
    lateral.measured_bank_rate_deg_s = controlled_roll_rate(t);
    lateral.bank_effectiveness = if (*t).bank_effectiveness.is_finite() as i32 != 0
        && (*t).bank_effectiveness > 0.0f64
    {
        (*t).bank_effectiveness
    } else {
        1.0f64
    };
    lateral.lift_accel = guidance_feedback_lift_accel(g, t, aero, v);
    lateral.has_measured_lift = (*t).physics_sample_valid as ::core::ffi::c_int != 0
        && (*t).mass.is_finite() as i32 != 0 && (*t).mass > DBL_MIN
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force > 0.0f64;
    lateral.has_simulated_lift_observation = (*g).diagnostic_shadow as ::core::ffi::c_int
        != 0 && (*t).mass.is_finite() as i32 != 0 && (*t).mass > DBL_MIN
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force > 0.0f64;
    lateral.authority_confidence = clampd(
        fmax(
            (*t).aerodynamic_confidence,
            (*t).physics_authority_confidence[1 as ::core::ffi::c_int as usize],
        ),
        0.0f64,
        1.0f64,
    );
    lateral.course_to_site_error_deg = if (*t).course_to_site_error.is_finite() as i32
        != 0
    {
        (*t).course_to_site_error
    } else {
        0.0f64
    };
    lateral.has_crossrange_error = (*t).runway_cross_track.is_finite() as i32 != 0;
    lateral.crossrange_error_m = if lateral.has_crossrange_error as ::core::ffi::c_int
        != 0
    {
        -(*t).runway_cross_track
    } else {
        0.0f64
    };
    let mut arrest_test: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_ENTRY_LATERAL_ARREST_TEST\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    lateral.crossrange_arrest_prediction = arrest_test.is_null()
        || strcmp(arrest_test, b"0\0" as *const u8 as *const ::core::ffi::c_char)
            != 0 as ::core::ffi::c_int;
    let mut map_radius: ::core::ffi::c_double = (*p).radius
        + fmax(0.0f64, (*t).mean_altitude);
    let mut map_scale: ::core::ffi::c_double = if map_radius > DBL_EPSILON {
        (*p).radius / map_radius
    } else {
        1.0f64
    };
    let mut map_speed: ::core::ffi::c_double = fmax(0.0f64, (*t).horizontal_speed)
        * map_scale;
    let mut course_relative: ::core::ffi::c_double = norm_signed_deg(
        course - (*cfg).site.runway_heading,
    ) * DEG2RAD;
    if lateral.has_crossrange_error as ::core::ffi::c_int != 0
        && map_speed.is_finite() as i32 != 0 && course_relative.is_finite() as i32 != 0
    {
        lateral.has_crossrange_rate = true_0 != 0;
        lateral.crossrange_error_rate_mps = -map_speed * sin(course_relative);
        lateral.has_crossrange_course_alignment = true_0 != 0;
        lateral.crossrange_course_alignment = cos(course_relative);
        let mut effective_bank: ::core::ffi::c_double = clampd(
            lateral.measured_bank_deg * lateral.bank_effectiveness,
            -nextafter(90.0f64, 0.0f64),
            nextafter(90.0f64, 0.0f64),
        );
        let mut course_rate_rad: ::core::ffi::c_double = if map_speed > DBL_EPSILON {
            lateral.lift_accel * sin(effective_bank * DEG2RAD) / map_speed
        } else {
            0.0f64
        };
        lateral.has_crossrange_accel = true_0 != 0;
        lateral.crossrange_error_accel_mps2 = -map_speed * cos(course_relative)
            * course_rate_rad;
    }
    lateral.crossrange_uncertainty_m = fmax(
        if (*t).trajectory_range_residual.is_finite() as i32 != 0 {
            fabs((*t).trajectory_range_residual)
        } else {
            0.0f64
        },
        if (*t).predicted_raw_published_position_delta_30s.is_finite() as i32 != 0 {
            fabs((*t).predicted_raw_published_position_delta_30s)
        } else {
            0.0f64
        },
    );
    if (*g).entry_policy_valid as ::core::ffi::c_int != 0
        && (*g).entry_policy_bank_deg.is_finite() as i32 != 0
        && (*g).entry_policy_bank_deg <= 0.5f64
    {
        lateral.crossrange_uncertainty_m = fmax(
            lateral.crossrange_uncertainty_m,
            MM304_ENERGY_SHORT_CROSSRANGE_TOLERANCE_M,
        );
    }
    lateral.has_sideslip = (*t).sideslip.is_finite() as i32 != 0
        && (*cfg).calibration.maximum_sideslip.is_finite() as i32 != 0
        && (*cfg).calibration.maximum_sideslip > 0.0f64;
    lateral.sideslip_deg = if lateral.has_sideslip as ::core::ffi::c_int != 0 {
        (*t).sideslip
    } else {
        0.0f64
    };
    lateral.maximum_sideslip_deg = if lateral.has_sideslip as ::core::ffi::c_int != 0 {
        (*cfg).calibration.maximum_sideslip
    } else {
        0.0f64
    };
    lateral.longitudinal = EntryLateralLongitudinalDemand {
        valid: drag.valid,
        confidence: drag.confidence,
        reference_drag_accel: drag.reference_drag_accel,
        drag_error_accel: drag.drag_error_accel,
        predicted_range_error: drag.predicted_range_error,
        range_error_scale: drag.range_error_scale,
        required_vertical_lift_accel: drag.required_vertical_lift_accel,
    };
    if (*g).entry_policy_valid as ::core::ffi::c_int != 0
        && (*g).entry_policy_bank_deg.is_finite() as i32 != 0
        && lateral.lift_accel.is_finite() as i32 != 0 && lateral.lift_accel > 0.0f64
    {
        let mut eff: ::core::ffi::c_double = clampd(
            (*g).entry_policy_bank_deg * lateral.bank_effectiveness,
            0.0f64,
            nextafter(90.0f64, 0.0f64),
        );
        lateral.longitudinal.required_vertical_lift_accel = lateral.lift_accel
            * cos(eff * DEG2RAD);
        lateral.longitudinal.valid = true_0 != 0;
    }
    lateral.has_bank_magnitude = false_0 != 0;
    if (*t).has_controls as ::core::ffi::c_int != 0
        && (*t).sideslip.is_finite() as i32 != 0
        && (*t).control_yaw.is_finite() as i32 != 0
    {
        let mut signed_beta: ::core::ffi::c_double = norm_signed_deg((*t).sideslip);
        let mut beta: ::core::ffi::c_double = fabs(signed_beta);
        let mut yaw_use_0: ::core::ffi::c_double = clampd(
            fabs((*t).control_yaw),
            0.0f64,
            1.0f64,
        );
        let mut corrective_yaw: bool = signed_beta * (*t).control_yaw > 0.0f64;
        let mut outward_beta_rate: ::core::ffi::c_double = (if signed_beta >= 0.0f64 {
            1.0f64
        } else {
            -1.0f64
        }) * (*g).entry_coordination_beta_rate;
        let mut direct_scale: ::core::ffi::c_double = 1.0f64;
        if corrective_yaw as ::core::ffi::c_int != 0 && yaw_use_0 > DBL_EPSILON
            && beta > DBL_EPSILON
        {
            let mut beta_load: ::core::ffi::c_double = clampd(
                beta / MM304_COORDINATION_BETA_SCALE_DEG,
                0.0f64,
                1.0f64,
            );
            direct_scale = 1.0f64 - beta_load * yaw_use_0;
        }
        let mut predictive_scale: ::core::ffi::c_double = 1.0f64;
        if corrective_yaw as ::core::ffi::c_int != 0 && outward_beta_rate > DBL_EPSILON
            && beta < MM304_COORDINATION_BETA_SCALE_DEG
        {
            let mut rate_limit: ::core::ffi::c_double = fmax(
                DBL_EPSILON,
                fabs(limits.maximum_roll_rate_deg_s),
            );
            let mut accel_limit: ::core::ffi::c_double = fmax(
                DBL_EPSILON,
                fabs(limits.maximum_roll_accel_deg_s2),
            );
            let mut bank_span: ::core::ffi::c_double = fabs(lateral.measured_bank_deg);
            let mut rollout_time: ::core::ffi::c_double = bank_span / rate_limit
                + fabs(lateral.measured_bank_rate_deg_s) / accel_limit;
            let mut beta_margin: ::core::ffi::c_double = MM304_COORDINATION_BETA_SCALE_DEG
                - beta;
            let mut beta_escape_time: ::core::ffi::c_double = beta_margin
                / outward_beta_rate;
            if rollout_time > DBL_EPSILON {
                predictive_scale = clampd(
                    beta_escape_time / rollout_time,
                    0.0f64,
                    1.0f64,
                );
            }
        }
        limits.maximum_bank_deg *= fmin(direct_scale, predictive_scale);
    }
    lateral.has_azimuth_error = (*t).course_to_site_error.is_finite() as i32 != 0;
    lateral.azimuth_error_deg = if lateral.has_azimuth_error as ::core::ffi::c_int != 0 {
        (*t).course_to_site_error
    } else {
        0.0f64
    };
    if sts_delivery as ::core::ffi::c_int != 0
        && (*t).ground_track_heading.is_finite() as i32 != 0
    {
        lateral.has_azimuth_error = true_0 != 0;
        lateral.azimuth_error_deg = norm_signed_deg(
            initial_bearing(current_point, alignment_point) - (*t).ground_track_heading,
        );
    }
    lateral.azimuth_deadband_deg = atan2(
        lateral.crossrange_uncertainty_m,
        fmax(alignment_range, DBL_EPSILON),
    ) * RAD2DEG;
    let mut mean_speed: ::core::ffi::c_double = 0.5f64
        * (map_speed + fmax(0.0f64, alignment_speed));
    lateral.has_time_to_go = mean_speed.is_finite() as i32 != 0
        && mean_speed > DBL_EPSILON && alignment_range.is_finite() as i32 != 0;
    lateral.time_to_go_s = if lateral.has_time_to_go as ::core::ffi::c_int != 0 {
        alignment_range / mean_speed
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    lateral.has_point_miss = (*t).course_to_site_error.is_finite() as i32 != 0
        && alignment_range.is_finite() as i32 != 0;
    lateral.point_miss_m = if lateral.has_point_miss as ::core::ffi::c_int != 0 {
        alignment_range * sin((*t).course_to_site_error * DEG2RAD)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut track: ::core::ffi::c_double = (*t).ground_track_heading * DEG2RAD;
    let mut lat: ::core::ffi::c_double = (*t).latitude * DEG2RAD;
    let mut lon: ::core::ffi::c_double = (*t).longitude * DEG2RAD;
    if lateral.has_time_to_go as ::core::ffi::c_int != 0 && track.is_finite() as i32 != 0
        && (*t).horizontal_speed.is_finite() as i32 != 0
        && (*p).rotational_speed.is_finite() as i32 != 0
    {
        let mut up: Vector3 = v3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat));
        let mut east: Vector3 = v3(-sin(lon), cos(lon), 0.0f64);
        let mut north: Vector3 = vcross(up, east);
        let mut r: ::core::ffi::c_double = (*p).radius + (*t).mean_altitude;
        let mut v_surface: Vector3 = vscale(
            vadd(vscale(north, cos(track)), vscale(east, sin(track))),
            (*t).horizontal_speed,
        );
        let mut v_inertial: Vector3 = vadd(
            v_surface,
            vscale(east, (*p).rotational_speed * r * cos(lat)),
        );
        let mut n: Vector3 = vnorm(
            vcross(up, v_inertial),
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut slat: ::core::ffi::c_double = alignment_point.latitude * DEG2RAD;
        let mut slon: ::core::ffi::c_double = alignment_point.longitude * DEG2RAD
            + (*p).rotational_speed * lateral.time_to_go_s;
        let mut site: Vector3 = v3(
            cos(slat) * cos(slon),
            cos(slat) * sin(slon),
            sin(slat),
        );
        let mut side: ::core::ffi::c_double = vdot(n, site);
        if side.is_finite() as i32 != 0 && vmag(n) > 0.0f64 {
            lateral.has_point_miss = true_0 != 0;
            lateral.point_miss_m = -ground_radius * asin(clampd(side, -1.0f64, 1.0f64));
        }
    }
    let mut previous_side: ::core::ffi::c_double = if (*g).s_turn_sign >= 0.0f64 {
        1.0f64
    } else {
        -1.0f64
    };
    let mut mm_count: usize = (alpha_schedule.point_count as usize).min(8);
    let mut mm_ceiling: ::core::ffi::c_double = fmin(alpha_ceiling, (*v).maximum_angle_of_attack);
    let mut mm_stair_env: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_ENTRY_AOA_STAIRCASE_TEST\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut mm_stair: Option<&str> = if mm_stair_env.is_null() {
        None
    } else {
        ::core::ffi::CStr::from_ptr(mm_stair_env).to_str().ok().filter(|x| !x.is_empty())
    };
    crate::mm304::live::set_schedule(
        &mut (*g).mm304,
        &alpha_schedule.relative_velocity[..mm_count],
        &alpha_schedule.alpha[..mm_count],
        alpha_schedule.target_rate_limit,
        alpha_schedule.minimum_protective_aoa,
        mm_ceiling,
        mm_stair,
    );
    let mut mm_max_ld: ::core::ffi::c_double = crate::mm304::predict::max_lift_to_drag_aoa(
        (*g).entry_terminal_model as *const crate::prediction::predictor::TerminalModel,
        (*t).dynamic_pressure,
        (*t).mach,
        mm_ceiling,
    );
    // MM304 delivers to MM305's HAC reference point: the extended centreline at the
    // final-alignment distance plus the HAC station range from the threshold.
    let mut hac_point: GeoPoint = if hac_range.is_finite() as i32 != 0 {
        runway_approach_aimpoint(&raw mut delivery_site, ground_radius, alignment.distance_m + hac_range)
    } else if (*g).mm304.target_valid {
        GeoPoint {
            latitude: (*g).mm304.target_latitude,
            longitude: (*g).mm304.target_longitude,
            altitude: alignment_point.altitude,
        }
    } else {
        alignment_point
    };
    let mut mm_in: crate::mm304::live::LiveInput = crate::mm304::live::LiveInput {
        ut: (*t).ut,
        mach: (*t).mach,
        speed: (*t).true_air_speed,
        ground_speed: map_speed,
        altitude: (*t).mean_altitude,
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        course: if course.is_finite() as i32 != 0 { course } else { (*t).ground_track_heading },
        site_altitude: (*cfg).site.altitude,
        surface_gravity: planet_surface_gravity(p),
        radius: ground_radius,
        lift_accel: lateral.lift_accel,
        target_latitude: hac_point.latitude,
        target_longitude: hac_point.longitude,
        station_range: 0.0f64,
        handoff_speed: if hac_v.is_finite() as i32 != 0 { hac_v } else { taem_velocity },
        max_ld_aoa: mm_max_ld,
        bank_limit: limits.maximum_bank_deg,
        roll_rate: limits.maximum_roll_rate_deg_s,
        cross_uncertainty: fmax(
            lateral.crossrange_uncertainty_m,
            if (*g).mm304.residual.miss.is_finite() as i32 != 0 {
                fabs((*g).mm304.residual.miss)
            } else {
                0.0f64
            },
        ),
        measured_bank: lateral.measured_bank_deg,
    };
    let mut mm = crate::mm304::live::tick(&mut (*g).mm304, &mm_in);
    (*g).mm304.live_terminal = if (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0
        && !(*g).entry_terminal_model.is_null()
    {
        crate::mm304::terminal::evaluate(
            &crate::mm304::terminal::Handoff {
                ut: (*t).ut,
                point: crate::prediction::predictor::GeoPoint {
                    latitude: (*t).latitude,
                    longitude: (*t).longitude,
                    altitude: (*t).mean_altitude,
                },
                airspeed: (*t).true_air_speed,
                ground_speed: map_speed,
                course: mm_in.course,
                flight_path_angle: (*t).flight_path_angle,
                mass: (*t).mass,
            },
            &crate::mm304::terminal::Site {
                latitude: (*cfg).site.latitude,
                longitude: (*cfg).site.longitude,
                altitude: (*cfg).site.altitude,
                runway_heading: (*cfg).site.runway_heading,
                radius: (*p).radius,
            },
            (*g).entry_terminal_model as *const ::core::ffi::c_void,
            &raw const (*g).taem_hac_live_aero as *const ::core::ffi::c_void,
            (*s).mm305_min_altitude,
            (*s).mm305_max_altitude,
        )
    } else {
        crate::mm304::state::Mm304State::ZERO.live_terminal
    };
    // The deorbit witness ranks burns by this excess: MM305's least-energy route,
    // the requirement every handoff must meet (the spiral is taken when reachable).
    if (*g).mm304.live_terminal.valid && (*g).mm304.live_terminal.short_error.is_finite() {
        (*g).entry_hac_energy_excess = (*g).mm304.live_terminal.short_error;
        (*g).entry_hac_energy_ut = (*t).ut;
    }
    let mut mm_law = (*g).mm304.law;
    let mut lateral_out: EntryLateralOutput = EntryLateralOutput {
        valid: mm.aoa.is_finite() && mm.bank.is_finite(),
        reversal_requested: mm.reversal,
        leg_captured: false,
        reversal_armed: false,
        degraded_authority: false,
        bank_sign: mm_law.bank_sign,
        bank_magnitude_deg: fabs(mm.bank),
        energy_bank_magnitude_deg: crate::mm304::law::plan_attitude(
            if (*g).mm304.plan.valid { (*g).mm304.plan.sigma } else { 0.0f64 },
            mm.nominal_aoa,
            mm_max_ld,
            limits.maximum_bank_deg,
        ).1,
        raw_target_bank_deg: mm.bank,
        target_bank_deg: mm.bank,
        target_bank_rate_deg_s: 0.0f64,
        corridor_metric: 0.0f64,
        course_corridor_deg: 0.0f64,
        crossrange_corridor_m: mm_in.cross_uncertainty,
        projected_crossrange_error_m: mm_law.miss_hold,
        reversal_response_time_s: 0.0f64,
        required_lateral_lift_accel: mm_law.accel_required,
        zero_effort_crossrange_m: mm_law.miss,
        lateral_demand_binding: mm_law.regime == crate::mm304::state::MM304_REGIME_LATERAL,
        expected_course_rate_deg_s: 0.0f64,
        sideslip_rate_deg_s: 0.0f64,
        coordination_roll_rate_limit_deg_s: limits.maximum_roll_rate_deg_s,
    };
    if !lateral_out.valid {
        return entry_program_contract_hold(
            g,
            t,
            cfg,
            dt,
            b"MM304 lateral allocator is invalid; Entry ownership is retained.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    let mut side_0: ::core::ffi::c_double = if lateral_out.bank_sign >= 0.0f64 {
        1.0f64
    } else {
        -1.0f64
    };
    if lateral_out.reversal_requested as ::core::ffi::c_int != 0
        && side_0 * previous_side < 0.0f64
    {
        (*g).entry_control_reversals = (*g).entry_control_reversals.wrapping_add(1);
        (*g).entry_continuation_bootstrap = false_0 != 0;
        if !(*g).diagnostic_shadow {
            fprintf(
                stderr,
                b"MM304 feedback reversal: UT %.2f side %+.0f -> %+.0f azimuth error %+.1f deg deadband %.1f deg V %.0f m/s.\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                previous_side,
                side_0,
                lateral.azimuth_error_deg,
                lateral.azimuth_deadband_deg,
                (*t).true_air_speed,
            );
        }
    }
    (*g).s_turn_sign = side_0;
    let mut bank: ::core::ffi::c_double = lateral_out.target_bank_deg;
    let mut bank_resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON)
        * fmax(1.0f64, limits.maximum_bank_deg);
    let mut bank_at_limit: bool = limits.maximum_bank_deg > DBL_EPSILON
        && fabs(bank) >= limits.maximum_bank_deg - bank_resolution;
    let mut structural_clear: bool = (*t).dynamic_pressure.is_finite() as i32 != 0
        && (*t).g_force.is_finite() as i32 != 0
        && (*t).dynamic_pressure < (*v).maximum_dynamic_pressure
        && (*t).g_force < (*v).maximum_g_load;
    let mut aoa: ::core::ffi::c_double = mm.aoa;
    if alpha.limiting_reason as ::core::ffi::c_uint
        == ENTRY_ALPHA_LIMIT_STALL_MARGIN as ::core::ffi::c_int as ::core::ffi::c_uint
        || alpha.limiting_reason as ::core::ffi::c_uint
            == ENTRY_ALPHA_LIMIT_G_LOAD as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        aoa = fmin(aoa, alpha.target_aoa);
    }
    aoa = clampd(aoa, 0.0f64, (*v).maximum_angle_of_attack);
    if !(*g).diagnostic_shadow {
        let (mm_range, _) = crate::mm304::predict::range_bearing(
            crate::prediction::predictor::GeoPoint {
                latitude: (*t).latitude,
                longitude: (*t).longitude,
                altitude: (*t).mean_altitude,
            },
            crate::prediction::predictor::GeoPoint {
                latitude: hac_point.latitude,
                longitude: hac_point.longitude,
                altitude: 0.0f64,
            },
            ground_radius,
        );
        crate::mm304::telemetry::publish(crate::mm304::telemetry::Published {
            state: (*g).mm304,
            ut: (*t).ut,
            aoa_cmd: aoa,
            nominal_aoa: mm.nominal_aoa,
            bank_cmd: bank,
            max_ld_aoa: mm_max_ld,
            range: mm_range,
        });
    }
    let mut heading: ::core::ffi::c_double = if course.is_finite() as i32 != 0 {
        norm_deg(course)
    } else {
        norm_deg((*t).heading)
    };
    (*g).entry_feedback = EntryGuidanceFeedback {
        valid: true_0 != 0,
        lateral_valid: true_0 != 0,
        reversal_requested: lateral_out.reversal_requested,
        ut: (*t).ut,
        confidence: fmin(drag.confidence, lateral.authority_confidence),
        measured_drag_accel: drag.observed_drag_accel,
        reference_drag_accel: drag.reference_drag_accel,
        drag_error_accel: drag.drag_error_accel,
        predicted_achievable_range: drag.predicted_range_to_taem,
        required_range_to_go: drag.available_range_to_taem,
        energy_margin: drag.predicted_range_error,
        range_error_scale: drag.range_error_scale,
        required_vertical_lift_accel: drag.required_vertical_lift_accel,
        crossrange_error: if lateral.has_crossrange_error as ::core::ffi::c_int != 0 {
            lateral.crossrange_error_m
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        },
        crossrange_rate: if lateral.has_crossrange_rate as ::core::ffi::c_int != 0 {
            lateral.crossrange_error_rate_mps
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        },
        projected_crossrange_error: lateral_out.projected_crossrange_error_m,
        crossrange_corridor: lateral_out.crossrange_corridor_m,
        energy_bank_magnitude_deg: lateral_out.energy_bank_magnitude_deg,
        commanded_bank_deg: bank,
        bank_sign: side_0,
    };
    if diagnostic_milestone.is_finite() as i32 != 0 {
        fprintf(
            stderr,
            b"MM304_SHADOW_MILESTONE hold=0 milestone=%.0f ut=%.2f phase=%s h=%.0f V=%.1f M=%.2f q=%.0f fpa=%.2f range=%.0f predRange=%.0f reqRange=%.0f rangeErr=%+.0f Dtelemetry=%.3f Dmodel=%.3f Dref=%.3f bankEnergy=%.1f objective=siteBearing azErr=%+.2f xte=%+.0f bankCmd=%+.1f roll=%+.1f aoaNom=%.1f aoaCmd=%.1f aoa=%.1f d/e/a/l=%d%d%d%d degraded=%d%d%d%d sat=%d%d%d mm305Ready=%d veto=0x%x\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            diagnostic_milestone,
            (*t).ut,
            entry_phase_string((*g).entry_exec.phase),
            (*t).mean_altitude,
            (*t).true_air_speed,
            (*t).mach,
            (*t).dynamic_pressure,
            (*t).flight_path_angle,
            (*t).range_to_site,
            drag.predicted_range_to_taem,
            drag.available_range_to_taem,
            drag.predicted_range_error,
            drag.observed_drag_accel,
            drag_in.modeled_drag_accel,
            drag.reference_drag_accel,
            lateral_out.bank_magnitude_deg,
            lateral.azimuth_error_deg,
            -(*t).runway_cross_track,
            bank,
            norm_signed_deg((*t).roll),
            alpha.nominal_aoa,
            aoa,
            (*t).angle_of_attack,
            if drag.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if drag.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if alpha.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if lateral_out.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if drag.degraded as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if drag.degraded as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if alpha.degraded as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if lateral_out.degraded_authority as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            0 as ::core::ffi::c_int,
            if bank_at_limit as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if structural_clear as ::core::ffi::c_int != 0 {
                0 as ::core::ffi::c_int
            } else {
                1 as ::core::ffi::c_int
            },
            if handoff_ready as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            admission.veto,
        );
    }
    (*g).entry_lateral_bank_magnitude = lateral_out.bank_magnitude_deg;
    (*g).entry_vertical_bank_magnitude = lateral_out.bank_magnitude_deg;
    (*g).entry_demand_bank = side_0 * lateral_out.bank_magnitude_deg;
    (*g).entry_control_plan_valid = true_0 != 0;
    (*g).entry_control_terminal_ready = handoff_ready;
    (*g).entry_control_plan_ut = (*t).ut;
    (*g).entry_control_bank = bank;
    (*g).entry_control_aoa = aoa;
    (*g).entry_control_heading = heading;
    (*g).entry_control_turn_radius = live_turn_radius(t, aero, v, bank);
    (*g).entry_control_cost = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).entry_control_taem_range_error = drag.predicted_range_error;
    let mut qualified_inlet: bool = (*g).mm305_admission_valid as ::core::ffi::c_int != 0
        && (*g).mm305_admission_found as ::core::ffi::c_int != 0
        && (*g).mm305_admission_route.valid as ::core::ffi::c_int != 0
        && (*g).mm305_admission_route.profile_speed_tabulated as ::core::ffi::c_int != 0
        && (*t).ut >= (*g).mm305_admission_result_ut
        && (*t).ut - (*g).mm305_admission_result_ut <= MM305_ADMISSION_MAX_AGE_S;
    (*g).entry_control_taem_speed = (if qualified_inlet as ::core::ffi::c_int != 0 {
        (*g).mm305_admission_route.profile_speed_lut[0 as ::core::ffi::c_int as usize]
    } else {
        ::core::f32::NAN
    }) as ::core::ffi::c_double;
    (*g).entry_control_taem_energy_error = entry_remaining_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        (*cfg).site.latitude,
        alignment_altitude,
        alignment_speed,
        p,
    );
    (*g).entry_control_segment_until_ut = (*t).ut;
    if (*g).entry_supervision_ut.is_finite() as i32 == 0
        || (*t).ut - (*g).entry_supervision_ut
            >= fmax(DBL_EPSILON, (*s).prediction_interval)
    {
        (*g).entry_planning_needed = true_0 != 0;
    }
    (*g).airbrakes_deployed = false_0 != 0;
    let mut c: GuidanceCommand = atmospheric(
        t,
        heading,
        bank,
        v,
        0.0f64,
        false_0 != 0,
        PROFILE_ENTRY,
    );
    c.heading_control_enabled = false_0 != 0;
    c.has_target_aoa = true_0 != 0;
    c.target_aoa = aoa;
    c.target_pitch = (*t).flight_path_angle + aoa;
    let mut status: [::core::ffi::c_char; 448] = [0; 448];
    snprintf(
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 448]>() as size_t,
        b"MM304 %s: plan %s sigma %+.2f, HAC energy %+.0f m (target %.0f, band %.0f, unc %.0f), bank %+.1f deg, miss %+.1f km, AoA %.1f deg (sched %.1f), %u rev, MM305 %s.\0"
            as *const u8 as *const ::core::ffi::c_char,
        entry_phase_string((*g).entry_exec.phase),
        crate::mm304::telemetry::decision_cname((*g).mm304.plan.decision),
        (*g).mm304.plan.sigma,
        (*g).mm304.plan.predicted_energy,
        (*g).mm304.plan.target_energy,
        (*g).mm304.plan.terminal.band,
        (*g).mm304.plan.uncertainty,
        bank,
        mm_law.miss / 1000.0f64,
        aoa,
        mm.nominal_aoa,
        mm_law.reversals,
        if handoff_ready as ::core::ffi::c_int != 0 {
            b"admissible\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"not yet admissible\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    let mut admission_warning: [::core::ffi::c_char; 384] = [0; 384];
    let mut warning: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    if drag.degraded as ::core::ffi::c_int != 0
        || alpha.degraded as ::core::ffi::c_int != 0
        || lateral_out.degraded_authority as ::core::ffi::c_int != 0
    {
        warning = b"MM304 is using degraded aerodynamic feedback; commands remain inside measured/configured authority and no fallback trajectory is active.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if drag.predicted_range_error < 0.0f64
        && drag.drag_range_scale <= drag_cfg.minimum_drag_scale + 1e-6f64
    {
        warning = b"MM304 maximum-range drag profile cannot reach the required Final alignment path; preserve energy and correct lateral displacement before MM305.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    let mut inlet_band: bool = (*t).mean_altitude >= (*s).mm305_min_altitude
        && (*t).mean_altitude <= (*s).mm305_max_altitude
        && (*t).mach.is_finite() as i32 != 0
        && fabs((*t).mach - (*s).mm305_target_mach) <= (*s).mm305_mach_half_width;
    if inlet_band as ::core::ffi::c_int != 0 && !admission.valid {
        warning = b"MM305 energy projection is invalid or unconverged; MM304 retains ownership. Verify native atmosphere, force evidence and response limits.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if inlet_band as ::core::ffi::c_int != 0
        && admission.veto & 16 as ::core::ffi::c_uint != 0
    {
        snprintf(
            &raw mut admission_warning as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"MM305 alignment reserve is not certified (%+.0f J/kg, drag %.0f, uncertainty %.0f). MM304 retains ownership; preserve energy and reduce lateral displacement before the inlet.\0"
                as *const u8 as *const ::core::ffi::c_char,
            admission.energy_margin,
            admission.energy_drag_work,
            admission.energy_uncertainty,
        );
        warning = &raw mut admission_warning as *mut ::core::ffi::c_char;
    }
    fn entry_trace_last_ut_tls() -> *mut ::core::ffi::c_double {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<::core::ffi::c_double> = const { ::core::cell::UnsafeCell::new(-::core::f32::INFINITY
        as ::core::ffi::c_double) };
        }
        CELL.with(|c| c.get())
    }
    let mut entry_trace: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_ENTRY_TRACE\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !entry_trace.is_null()
        && strcmp(entry_trace, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
        && ((*t).ut < (*entry_trace_last_ut_tls()) || (*t).ut - (*entry_trace_last_ut_tls()) >= 5.0f64)
    {
        fprintf(
            stderr,
            b"ENTRY_TRACE ut=%.3f h=%.1f V=%.2f q=%.2f fpa=%.2f D=%.3f Dref=%.3f dD=%+.3f Rach=%.1f Rreq=%.1f margin=%+.1f along=%+.1f xtk=%+.1f azerr=%+.1f hdotref=%+.1f Benergy=%.2f Bcmd=%+.2f roll=%+.2f aoa=%.2f admissionE=%+.0f Eavail=%.0f Edrag=%.0f Eunc=%.0f path=%.0f reversal=%u handoff=%d veto=%u eh=%+.0f D0=%.2f sat=%d%d%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            (*t).mean_altitude,
            (*t).true_air_speed,
            (*t).dynamic_pressure,
            (*t).flight_path_angle,
            drag.observed_drag_accel,
            drag.reference_drag_accel,
            drag.drag_error_accel,
            drag.predicted_range_to_taem,
            drag.available_range_to_taem,
            drag.predicted_range_error,
            (*t).runway_along_track,
            (*t).runway_cross_track,
            lateral.azimuth_error_deg,
            ::core::f32::NAN as ::core::ffi::c_double,
            lateral_out.bank_magnitude_deg,
            bank,
            norm_signed_deg((*t).roll),
            aoa,
            admission.energy_margin,
            admission.energy_available,
            admission.energy_drag_work,
            admission.energy_uncertainty,
            admission.path_length_m,
            (*g).entry_control_reversals,
            if handoff_ready as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            admission.veto,
            ::core::f32::NAN as ::core::ffi::c_double,
            drag.constant_drag_accel,
            0 as ::core::ffi::c_int,
            if bank_at_limit as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if structural_clear as ::core::ffi::c_int != 0 {
                0 as ::core::ffi::c_int
            } else {
                1 as ::core::ffi::c_int
            },
        );
        (*entry_trace_last_ut_tls()) = (*t).ut;
    }
    return stabilized(
        g,
        result_make(
            PHASE_ENTRY_ENERGY,
            c,
            &raw mut status as *mut ::core::ffi::c_char,
            warning,
        ),
        t,
        v,
        s,
        dt,
    );
}
#[no_mangle]
pub unsafe extern "C" fn live_turn_radius(
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut bank_deg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut lift: ::core::ffi::c_double = live_lift_accel(t, aero, v);
    let mut eff: ::core::ffi::c_double = if (*t).bank_effectiveness.is_finite() as i32
        != 0 && (*t).bank_effectiveness > 0.0f64
    {
        (*t).bank_effectiveness
    } else {
        1.0f64
    };
    let mut lateral: ::core::ffi::c_double = lift
        * fabs(
            sin(
                clampd(fabs(bank_deg) * eff, 0.0f64, nextafter(90.0f64, 0.0f64))
                    * DEG2RAD,
            ),
        );
    return if lateral > DBL_EPSILON {
        (*t).true_air_speed * (*t).true_air_speed / lateral
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const MM305_ADMISSION_MAX_AGE_S: ::core::ffi::c_double = 10.0f64;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
