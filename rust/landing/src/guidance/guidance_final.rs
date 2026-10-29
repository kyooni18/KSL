use crate::sys::*;
extern "C" {
    fn taem_exec_owns_vehicle(exec: *const TaemExecutive) -> bool;
    fn taem_exec_evaluate_terminal_contract(
        contract: *const TaemTerminalContract,
    ) -> TaemTerminalEvaluation;
    fn taem_terminal_block_reason_string(
        reason: TaemTerminalBlockReason,
    ) -> *const ::core::ffi::c_char;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn pow(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn floor(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmod(
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
    fn world_lla(
        w: *const KerbinWorld,
        position_i: Vec3,
        ut: ::core::ffi::c_double,
    ) -> LLA;
    fn world_atmosphere_sample_state(
        w: *const KerbinWorld,
        position_i: Vec3,
        ut: ::core::ffi::c_double,
    ) -> AtmosphereSample;
    fn aero_compute(
        w: *const KerbinWorld,
        a: *const AeroTable,
        position_i: Vec3,
        velocity_i: Vec3,
        ut: ::core::ffi::c_double,
        mass_kg: ::core::ffi::c_double,
        aoa_rad: ::core::ffi::c_double,
        bank_rad: ::core::ffi::c_double,
    ) -> AeroForces;
    fn taem_geometry_state(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *mut TaemGeometryState,
    ) -> bool;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_remaining_specific_energy(
        latitude: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
        air_relative_speed: ::core::ffi::c_double,
        target_latitude: ::core::ffi::c_double,
        target_altitude: ::core::ffi::c_double,
        target_speed: ::core::ffi::c_double,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn robust_pid_reset(p: *mut RobustPID);
    fn guidance_command_init(c: *mut GuidanceCommand);
    fn trajectory_copy(dst: *mut Trajectory, src: *const Trajectory) -> bool;
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
        lift_factor: *mut ::core::ffi::c_double,
        drag_factor: *mut ::core::ffi::c_double,
    );
    fn guidance_machine_init(g: *mut GuidanceMachine);
    fn guidance_set_engaged(g: *mut GuidanceMachine, engaged: bool);
    fn decision_vertical_recovery_envelope(
        available_height_m: ::core::ffi::c_double,
        sink_mps: ::core::ffi::c_double,
        target_sink_mps: ::core::ffi::c_double,
        response_time_s: ::core::ffi::c_double,
        response_down_accel_mps2: ::core::ffi::c_double,
        recovery_up_accel_mps2: ::core::ffi::c_double,
    ) -> VerticalRecoveryEnvelope;
    fn decision_guidance_attitude_limits(
        telemetry: *const Telemetry,
        settings: *const GuidanceSettings,
        phase: GuidancePhase,
        pitch_axis: bool,
    ) -> GuidanceAttitudeLimits;
    fn decision_final_approach_response_time(
        machine: *const GuidanceMachine,
        telemetry: *const Telemetry,
        settings: *const GuidanceSettings,
        target_aoa_deg: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn decision_margin(
        available: ::core::ffi::c_double,
        required: ::core::ffi::c_double,
    ) -> DecisionMargin;
    fn decision_axis_capture_time(
        error_deg: ::core::ffi::c_double,
        rate_deg_s: ::core::ffi::c_double,
        acceleration_limit_deg_s2: ::core::ffi::c_double,
        rate_limit_deg_s: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn decision_runway_capture_envelope(
        guidance: *const GuidanceMachine,
        telemetry: *const Telemetry,
        course_deg: ::core::ffi::c_double,
        planet: *const PlanetModel,
        configuration: *const LandingConfiguration,
    ) -> RunwayCaptureEnvelope;
    fn decision_energy_path_envelope(
        telemetry: *const Telemetry,
        planet: *const PlanetModel,
        available_specific_energy: ::core::ffi::c_double,
        modeled_drag_work: ::core::ffi::c_double,
        path_length_m: ::core::ffi::c_double,
    ) -> EnergyPathEnvelope;
    fn hac_response_lead_time(
        t: *const Telemetry,
        s: *const GuidanceSettings,
        nominal_bank: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn result_make(
        phase: GuidancePhase,
        c: GuidanceCommand,
        status: *const ::core::ffi::c_char,
        warning: *const ::core::ffi::c_char,
    ) -> GuidanceResult;
    fn reset_limiters(g: *mut GuidanceMachine);
    fn reset_taem_handoff_state(g: *mut GuidanceMachine);
    fn terminal_lateral_bank_limit(
        t: *const Telemetry,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn terminal_speedbrake_closed(g: *mut GuidanceMachine);
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
    fn guidance_feedback_lift_accel(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn live_drag_accel(
        t: *const Telemetry,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn entry_program_guidance(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        dt: ::core::ffi::c_double,
    ) -> GuidanceResult;
    fn terminal_expected_energy_loss_accel(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn taem_exec_enter(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        s: *const GuidanceSettings,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        checkpoint_resume: bool,
    ) -> bool;
    fn taem_exec_sync(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        s: *const GuidanceSettings,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
    );
    fn terminal_fpa_force_aoa(
        t: *const Telemetry,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
        reference_fpa: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_quadratic_drag_energy_step(
        speed: ::core::ffi::c_double,
        drag: ::core::ffi::c_double,
        air_distance: ::core::ffi::c_double,
        potential_drop: ::core::ffi::c_double,
        next_speed: *mut ::core::ffi::c_double,
        work: *mut ::core::ffi::c_double,
    ) -> bool;
    fn terminal_projected_drag_work_state_anchored(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
        s: *const GuidanceSettings,
        target_aoa: ::core::ffi::c_double,
        ground_path: ::core::ffi::c_double,
        slope_deg: ::core::ffi::c_double,
        end_speed_out: *mut ::core::ffi::c_double,
        end_altitude_out: *mut ::core::ffi::c_double,
        inherited_anchor: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_projected_drag_work_state(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
        s: *const GuidanceSettings,
        target_aoa: ::core::ffi::c_double,
        ground_path: ::core::ffi::c_double,
        slope_deg: ::core::ffi::c_double,
        end_speed_out: *mut ::core::ffi::c_double,
        end_altitude_out: *mut ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_projected_lift_accel_at_aoa(
        t: *const Telemetry,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
        aoa: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn runway_landing_intercept_ratio(
        along: ::core::ffi::c_double,
        cross: ::core::ffi::c_double,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
        ratio: *mut ::core::ffi::c_double,
    ) -> bool;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalPreflarePlan {
    pub feasible: bool,
    pub reject_reason: *const ::core::ffi::c_char,
    pub trigger_altitude: ::core::ffi::c_double,
    pub target_aoa: ::core::ffi::c_double,
    pub target_sink: ::core::ffi::c_double,
    pub minimum_speed: ::core::ffi::c_double,
    pub reference_speed: ::core::ffi::c_double,
    pub predicted_height_loss: ::core::ffi::c_double,
    pub predicted_kinetic_margin: ::core::ffi::c_double,
    pub effective_accel: ::core::ffi::c_double,
    pub response_time: ::core::ffi::c_double,
    pub plan_source_ut: ::core::ffi::c_double,
    pub height: DecisionMargin,
    pub degraded_floor: bool,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalFlarePlan {
    pub arrest_accel: ::core::ffi::c_double,
    pub sgs_height: ::core::ffi::c_double,
    pub sgs_sink: ::core::ffi::c_double,
    pub final_accel: ::core::ffi::c_double,
    pub float_height: ::core::ffi::c_double,
    pub float_sink: ::core::ffi::c_double,
    pub lift_ratio: ::core::ffi::c_double,
    pub aoa_max: ::core::ffi::c_double,
    pub g0: ::core::ffi::c_double,
    pub planet: *const PlanetModel,
    pub site_altitude: ::core::ffi::c_double,
    pub preflare_trigger_altitude: ::core::ffi::c_double,
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
pub struct GuidanceAttitudeLimits {
    pub rate_deg_s: ::core::ffi::c_double,
    pub accel_deg_s2: ::core::ffi::c_double,
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
pub struct LLA {
    pub lat_rad: ::core::ffi::c_double,
    pub lon_rad: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroForces {
    pub lift_n: ::core::ffi::c_double,
    pub drag_n: ::core::ffi::c_double,
    pub side_n: ::core::ffi::c_double,
    pub force_i: Vec3,
    pub mach: ::core::ffi::c_double,
    pub dynamic_pressure_pa: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalDynamicState {
    pub position_i_m: Vec3,
    pub velocity_i_mps: Vec3,
    pub attitude: AttitudeModel,
    pub mass_kg: ::core::ffi::c_double,
    pub ut_s: ::core::ffi::c_double,
    pub trim_aoa_ceiling_rad: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemGeometryState {
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub altitude_above_runway_m: ::core::ffi::c_double,
    pub ground_speed_mps: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub heading_error_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RolloutSteeringDemand {
    pub steering: ::core::ffi::c_double,
    pub lateral_rate: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AfLawOut {
    pub cmd: ::core::ffi::c_double,
    pub accel: ::core::ffi::c_double,
    pub need: ::core::ffi::c_double,
    pub on_arc: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AfState {
    pub h: ::core::ffi::c_double,
    pub sink: ::core::ffi::c_double,
    pub V: ::core::ffi::c_double,
    pub gamma: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub cmd: ::core::ffi::c_double,
    pub x: ::core::ffi::c_double,
    pub engaged: bool,
    pub glide_sink: ::core::ffi::c_double,
    pub aoa_rate: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AfParams {
    pub arc_accel: ::core::ffi::c_double,
    pub sgs_height: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AfModel {
    pub lq: [::core::ffi::c_double; 121],
    pub dq: [::core::ffi::c_double; 121],
    pub n: ::core::ffi::c_int,
    pub aoa_max: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub g0: ::core::ffi::c_double,
    pub rho: ::core::ffi::c_double,
    pub sound: ::core::ffi::c_double,
    pub tau: ::core::ffi::c_double,
    pub bias: ::core::ffi::c_double,
    pub gain: ::core::ffi::c_double,
    pub v_floor: ::core::ffi::c_double,
    pub zone_lo: ::core::ffi::c_double,
    pub zone_hi: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AfOutcome {
    pub landed: bool,
    pub speed: ::core::ffi::c_double,
    pub sink: ::core::ffi::c_double,
    pub along: ::core::ffi::c_double,
    pub peak_n: ::core::ffi::c_double,
    pub climb: ::core::ffi::c_double,
    pub cost: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const LANDER_ATMOSPHERE_SAMPLE_MAX: ::core::ffi::c_int = 1024 as ::core::ffi::c_int;
pub const TERMINAL_LINE_CAPTURE_BANK_DEG: ::core::ffi::c_double = 10.0f64;
#[no_mangle]
pub unsafe extern "C" fn terminal_outer_glide_slope(
    mut g: *const GuidanceMachine,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if (*g).terminal_glide_mode as ::core::ffi::c_int != 0
        && (*g).terminal_test_glide_slope
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return (*g).terminal_test_glide_slope;
    }
    if (*g).terminal_final_handoff_latched as ::core::ffi::c_int != 0
        && (*g).terminal_final_handoff_slope_deg.is_finite() as i32 != 0
        && (*g).terminal_final_handoff_slope_deg >= 3.0f64
        && (*g).terminal_final_handoff_slope_deg <= 45.0f64
    {
        return (*g).terminal_final_handoff_slope_deg;
    }
    return (*s).final_glide_slope;
}
unsafe extern "C" fn terminal_final_distance(
    mut g: *const GuidanceMachine,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if (*g).terminal_glide_mode as ::core::ffi::c_int != 0
        && (*g).terminal_test_final_approach_distance
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return (*g).terminal_test_final_approach_distance;
    }
    if (*g).terminal_final_handoff_latched as ::core::ffi::c_int != 0
        && (*g).terminal_final_handoff_distance.is_finite() as i32 != 0
        && (*g).terminal_final_handoff_distance >= 500.0f64
    {
        return (*g).terminal_final_handoff_distance;
    }
    return (*s).final_approach_distance;
}
unsafe extern "C" fn terminal_final_alignment_speed(
    mut g: *const GuidanceMachine,
    mut s: *const GuidanceSettings,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut fallback: ::core::ffi::c_double = fmax(
        (*v).final_approach_speed,
        (*v).minimum_safe_speed,
    );
    let mut alignment: ::core::ffi::c_double = if !s.is_null()
        && (*s).final_alignment_speed.is_finite() as i32 != 0
        && (*s).final_alignment_speed > 0.0f64
    {
        (*s).final_alignment_speed
    } else {
        fallback
    };
    if !g.is_null() && (*g).terminal_final_handoff_latched as ::core::ffi::c_int != 0
        && (*g).terminal_final_handoff_speed_mps.is_finite() as i32 != 0
        && (*g).terminal_final_handoff_speed_mps > 0.0f64
    {
        alignment = (*g).terminal_final_handoff_speed_mps;
    }
    return fmax(alignment, fmax((*v).touchdown_speed, 1.0f64));
}
unsafe extern "C" fn terminal_final_speed_target(
    mut g: *const GuidanceMachine,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut distance: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut final_distance: ::core::ffi::c_double = fmax(
        1.0f64,
        terminal_final_distance(g, s),
    );
    let mut alignment: ::core::ffi::c_double = terminal_final_alignment_speed(g, s, v);
    let mut touchdown_0: ::core::ffi::c_double = clampd(
        (*v).touchdown_speed,
        1.0f64,
        alignment,
    );
    let mut fraction: ::core::ffi::c_double = clampd(
        distance / final_distance,
        0.0f64,
        1.0f64,
    );
    return touchdown_0 + (alignment - touchdown_0) * fraction;
}
unsafe extern "C" fn terminal_observe_response(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut dt: ::core::ffi::c_double,
) {
    if !(*g).terminal_response_sample_valid {
        (*g).terminal_response_sample_valid = true_0 != 0;
        (*g).terminal_previous_aoa = (*t).angle_of_attack;
        (*g).terminal_previous_vertical_speed = (*t).vertical_speed;
        (*g).terminal_response_sample_ut = (*t).ut;
        return;
    }
    let mut sample_dt: ::core::ffi::c_double = (*t).ut
        - (*g).terminal_response_sample_ut;
    if !(sample_dt >= 0.04f64 && sample_dt <= 0.75f64) {
        (*g).terminal_previous_aoa = (*t).angle_of_attack;
        (*g).terminal_previous_vertical_speed = (*t).vertical_speed;
        (*g).terminal_response_sample_ut = (*t).ut;
        return;
    }
    let mut aoa_rate: ::core::ffi::c_double = ((*t).angle_of_attack
        - (*g).terminal_previous_aoa) / sample_dt;
    let mut sink_accel: ::core::ffi::c_double = ((*t).vertical_speed
        - (*g).terminal_previous_vertical_speed) / sample_dt;
    if aoa_rate.is_finite() as i32 != 0 && aoa_rate > 0.15f64 {
        let mut observed: ::core::ffi::c_double = clampd(aoa_rate, 0.25f64, 4.0f64);
        (*g).terminal_positive_aoa_rate_ema = 0.88f64
            * (*g).terminal_positive_aoa_rate_ema + 0.12f64 * observed;
        if sink_accel.is_finite() as i32 != 0 && sink_accel > 0.1f64 {
            let mut accel: ::core::ffi::c_double = clampd(sink_accel, 0.1f64, 6.0f64);
            (*g).terminal_sink_accel_ema = if (*g).terminal_sink_accel_ema
                > 0 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                0.90f64 * (*g).terminal_sink_accel_ema + 0.10f64 * accel
            } else {
                accel
            };
        }
    }
    (*g).terminal_previous_aoa = (*t).angle_of_attack;
    (*g).terminal_previous_vertical_speed = (*t).vertical_speed;
    (*g).terminal_response_sample_ut = (*t).ut;
}
unsafe extern "C" fn terminal_low_energy_speed_floor(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut v_td: ::core::ffi::c_double = if (*v).touchdown_speed > 1.0f64 {
        (*v).touchdown_speed
    } else {
        65.0f64
    };
    let mut rho: ::core::ffi::c_double = if (*t).atmospheric_density.is_finite() as i32
        != 0 && (*t).atmospheric_density > 0.0f64
    {
        (*t).atmospheric_density
    } else {
        planet_atmospheric_density(p, (*t).mean_altitude)
    };
    let mut L: ::core::ffi::c_double = 0.0f64;
    let mut D: ::core::ffi::c_double = 0.0f64;
    terminal_aero_q(g, t, v, final_lift_aoa_limit(t, v), &raw mut L, &raw mut D);
    let mut ratio: ::core::ffi::c_double = if (*g).terminal_lift_ratio > 0.0f64 {
        (*g).terminal_lift_ratio
    } else {
        1.0f64
    };
    let mut speed_1g: ::core::ffi::c_double = if rho > 0.0f64 && L > 0.0f64 {
        sqrt(2.0f64 * (*t).mass * planet_surface_gravity(p) / (rho * L * ratio))
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut floor_0: ::core::ffi::c_double = if speed_1g.is_finite() as i32 != 0 {
        fmax(v_td, 1.25f64 * speed_1g)
    } else {
        (*v).minimum_safe_speed
    };
    if (*t).calibrated_stall_speed.is_finite() as i32 != 0
        && (*t).calibrated_stall_speed > 0.0f64
    {
        floor_0 = fmax(floor_0, (*t).calibrated_stall_speed);
    }
    return fmin(floor_0, (*v).minimum_safe_speed);
}
unsafe extern "C" fn terminal_preflare_plan(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
) -> TerminalPreflarePlan {
    let mut plan: TerminalPreflarePlan = terminal_preflare_plan_floor(
        g,
        t,
        p,
        aero,
        cfg,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    if plan.feasible as ::core::ffi::c_int != 0 || plan.reject_reason.is_null()
        || strcmp(
            plan.reject_reason,
            b"preflare-speed-aoa-or-stall-gate\0" as *const u8
                as *const ::core::ffi::c_char,
        ) != 0 as ::core::ffi::c_int
    {
        return plan;
    }
    let mut floor_0: ::core::ffi::c_double = terminal_low_energy_speed_floor(
        g,
        t,
        p,
        &raw const (*cfg).vehicle,
    );
    if !(floor_0 < plan.minimum_speed - 0.5f64) {
        return plan;
    }
    let mut degraded: TerminalPreflarePlan = terminal_preflare_plan_floor(
        g,
        t,
        p,
        aero,
        cfg,
        floor_0,
    );
    if !degraded.feasible {
        return plan;
    }
    degraded.degraded_floor = true_0 != 0;
    return degraded;
}
unsafe extern "C" fn terminal_preflare_plan_floor(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut speed_floor: ::core::ffi::c_double,
) -> TerminalPreflarePlan {
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut plan: TerminalPreflarePlan = TerminalPreflarePlan {
        feasible: 0 as ::core::ffi::c_int != 0,
        reject_reason: ::core::ptr::null::<::core::ffi::c_char>(),
        trigger_altitude: 0.,
        target_aoa: 0.,
        target_sink: 0.,
        minimum_speed: 0.,
        reference_speed: 0.,
        predicted_height_loss: 0.,
        predicted_kinetic_margin: 0.,
        effective_accel: 0.,
        response_time: 0.,
        plan_source_ut: 0.,
        height: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        degraded_floor: false,
    };
    plan.reject_reason = b"uninitialized\0" as *const u8 as *const ::core::ffi::c_char;
    let mut slope: ::core::ffi::c_double = terminal_outer_glide_slope(g, s);
    let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
    if !(gravity > 0.0f64) || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
    {
        plan.reject_reason = b"gravity-or-vertical-speed-invalid\0" as *const u8
            as *const ::core::ffi::c_char;
        return plan;
    }
    let mut sink: ::core::ffi::c_double = fmax(0.0f64, -(*t).vertical_speed);
    let mut target_sink: ::core::ffi::c_double = clampd(
        (*t).horizontal_speed * tan(8.0f64 * DEG2RAD),
        6.0f64,
        14.0f64,
    );
    let mut recovery: RunwayCaptureEnvelope = decision_runway_capture_envelope(
        g,
        t,
        (*cfg).site.runway_heading,
        p,
        cfg,
    );
    if !recovery.valid {
        plan.reject_reason = b"capture-envelope-invalid\0" as *const u8
            as *const ::core::ffi::c_char;
        return plan;
    }
    let mut minimum_speed: ::core::ffi::c_double = if speed_floor.is_finite() as i32 != 0
    {
        speed_floor
    } else {
        (*v).minimum_safe_speed
    };
    if (*t).calibrated_stall_speed.is_finite() as i32 != 0
        && (*t).calibrated_stall_speed > 0.0f64
    {
        minimum_speed = fmax(minimum_speed, (*t).calibrated_stall_speed);
    }
    let mut current_lf: ::core::ffi::c_double = 0.0f64;
    let mut current_df: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        (*t).angle_of_attack,
        v,
        &raw mut current_lf,
        &raw mut current_df,
    );
    if !(current_df > DBL_EPSILON) {
        plan.reject_reason = b"aero-force-factors-invalid\0" as *const u8
            as *const ::core::ffi::c_char;
        return plan;
    }
    let mut measured_lift: ::core::ffi::c_double = guidance_feedback_lift_accel(
        g,
        t,
        aero,
        v,
    );
    let mut measured_drag: ::core::ffi::c_double = live_drag_accel(t, aero, v);
    if !(measured_lift > 0.0f64) || !(measured_drag >= 0.0f64) {
        plan.reject_reason = b"measured-aero-invalid\0" as *const u8
            as *const ::core::ffi::c_char;
        return plan;
    }
    let mut vertical_projection: ::core::ffi::c_double = cos(
        (*t).flight_path_angle * DEG2RAD,
    );
    let mut target_aoa: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut modeled_up: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut response_time: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut required_height: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut trigger: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut predicted_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut best_trigger: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut vertical: VerticalRecoveryEnvelope = VerticalRecoveryEnvelope {
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
    let mut fallback_trigger: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_required_height: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_pull_time: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_target_drag: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_air_path: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_modeled_up: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_response_time: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fallback_predicted_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut maximum_aoa: ::core::ffi::c_double = clampd(
        recovery.maximum_lift_aoa_deg,
        0.0f64,
        (*v).maximum_angle_of_attack,
    );
    let mut candidate: ::core::ffi::c_double = maximum_aoa;
    while candidate >= 0.5f64 {
        let mut target_df: ::core::ffi::c_double = 0.0f64;
        aerodynamic_force_factors_mach(
            (*t).mach,
            candidate,
            v,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            &raw mut target_df,
        );
        if !(!(target_df >= 0.0f64) || target_df.is_finite() as i32 == 0) {
            let mut projected_lift: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
                t,
                aero,
                v,
                candidate,
            );
            if recovery.maximum_normal_accel_mps2.is_finite() as i32 != 0
                && recovery.maximum_normal_accel_mps2 > 0.0f64
            {
                projected_lift = fmin(
                    projected_lift,
                    recovery.maximum_normal_accel_mps2,
                );
            }
            let mut candidate_up: ::core::ffi::c_double = fmax(
                0.0f64,
                projected_lift * vertical_projection - gravity,
            );
            if !(!(candidate_up > 0.0f64) || candidate_up.is_finite() as i32 == 0) {
                let mut candidate_response: ::core::ffi::c_double = decision_final_approach_response_time(
                    g,
                    t,
                    s,
                    candidate,
                );
                if !(candidate_response.is_finite() as i32 == 0) {
                    let mut response_down: ::core::ffi::c_double = fmax(
                        0.0f64,
                        0.5f64 * (recovery.response_down_accel_mps2 - candidate_up),
                    );
                    let mut candidate_vertical: VerticalRecoveryEnvelope = decision_vertical_recovery_envelope(
                        (*t).radar_altitude - (*s).flare_altitude,
                        sink,
                        target_sink,
                        candidate_response,
                        response_down,
                        candidate_up,
                    );
                    if candidate_vertical.valid {
                        let mut candidate_required: ::core::ffi::c_double = candidate_vertical
                            .required_height_m;
                        let mut candidate_trigger: ::core::ffi::c_double = (*s)
                            .flare_altitude + candidate_required;
                        let mut candidate_pull: ::core::ffi::c_double = candidate_vertical
                            .recovery_time_s;
                        let mut candidate_drag: ::core::ffi::c_double = measured_drag
                            * (target_df / current_df);
                        let mut candidate_air_path: ::core::ffi::c_double = fmax(
                            0.0f64,
                            (*t).true_air_speed * candidate_pull,
                        );
                        let mut response_path: ::core::ffi::c_double = fmax(
                            0.0f64,
                            (*t).true_air_speed * candidate_response,
                        );
                        let mut response_speed: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        let mut response_work: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        let mut candidate_speed: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        let mut candidate_work: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        if !(!terminal_quadratic_drag_energy_step(
                            (*t).true_air_speed,
                            measured_drag,
                            response_path,
                            gravity * candidate_vertical.response_height_m,
                            &raw mut response_speed,
                            &raw mut response_work,
                        )
                            || !terminal_quadratic_drag_energy_step(
                                response_speed,
                                candidate_drag,
                                fmax(
                                    0.0f64,
                                    response_speed * (candidate_pull - candidate_response),
                                ),
                                gravity * candidate_vertical.braking_height_m,
                                &raw mut candidate_speed,
                                &raw mut candidate_work,
                            ))
                        {
                            candidate_work += response_work;
                            if candidate >= maximum_aoa - 0.25f64 {
                                fallback_trigger = candidate_trigger;
                                fallback_required_height = candidate_required;
                                fallback_pull_time = candidate_pull;
                                fallback_target_drag = candidate_drag;
                                fallback_air_path = candidate_air_path;
                                fallback_modeled_up = candidate_up;
                                fallback_response_time = candidate_response;
                                fallback_predicted_speed = candidate_speed;
                            }
                            if !(candidate_vertical.height.margin < 0.0f64
                                || candidate_speed < minimum_speed)
                            {
                                let mut candidate_reference: ::core::ffi::c_double = minimum_speed;
                                let mut net_kinetic_loss_rate: ::core::ffi::c_double = fmax(
                                    0.0f64,
                                    measured_drag * (*t).true_air_speed - gravity * sink,
                                );
                                let mut energy_trigger: ::core::ffi::c_double = (*s)
                                    .flare_altitude;
                                if candidate_speed > candidate_reference
                                    && net_kinetic_loss_rate > DBL_EPSILON && sink > 0.5f64
                                {
                                    let mut excess: ::core::ffi::c_double = 0.5f64
                                        * (candidate_speed * candidate_speed
                                            - candidate_reference * candidate_reference);
                                    let mut delay: ::core::ffi::c_double = excess
                                        / net_kinetic_loss_rate;
                                    energy_trigger = fmax(
                                        (*s).flare_altitude,
                                        (*t).radar_altitude - sink * delay,
                                    );
                                }
                                candidate_trigger = fmax(candidate_trigger, energy_trigger);
                                let mut delay_to_trigger: ::core::ffi::c_double = if sink
                                    > 0.5f64
                                {
                                    fmax(
                                        0.0f64,
                                        ((*t).radar_altitude - candidate_trigger) / sink,
                                    )
                                } else {
                                    0.0f64
                                };
                                let mut adjusted_v2: ::core::ffi::c_double = candidate_speed
                                    * candidate_speed
                                    - 2.0f64 * net_kinetic_loss_rate * delay_to_trigger;
                                if !(adjusted_v2.is_finite() as i32 == 0
                                    || adjusted_v2 < minimum_speed * minimum_speed)
                                {
                                    let mut adjusted_speed: ::core::ffi::c_double = sqrt(
                                        fmax(0.0f64, adjusted_v2),
                                    );
                                    if !(candidate_trigger >= best_trigger - 1e-6f64) {
                                        best_trigger = candidate_trigger;
                                        target_aoa = candidate;
                                        modeled_up = candidate_up;
                                        response_time = candidate_response;
                                        vertical = candidate_vertical;
                                        required_height = candidate_required;
                                        trigger = candidate_trigger;
                                        predicted_speed = adjusted_speed;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        candidate -= 0.5f64;
    }
    if target_aoa.is_finite() as i32 == 0 {
        plan.trigger_altitude = fallback_trigger;
        plan.minimum_speed = minimum_speed;
        if !getenv(
                b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        {
            fprintf(
                stderr,
                b"FINAL_TEST_PREFLARE speedGate predictedSpeed=%.6f minimumSpeed=%.6f currentSpeed=%.6f aoa=%.6f maxAoA=%.6f stallMeasured=%d stallFraction=%.6f trigger=%.6f requiredHeight=%.6f pullTime=%.6f targetDrag=%.6f airPath=%.6f modeledUp=%.6f responseTime=%.6f targetAoA=%.6f sink=%.6f targetSink=%.6f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                fallback_predicted_speed,
                minimum_speed,
                (*t).true_air_speed,
                (*t).angle_of_attack,
                (*v).maximum_angle_of_attack,
                if (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                (*t).stall_fraction,
                fallback_trigger,
                fallback_required_height,
                fallback_pull_time,
                fallback_target_drag,
                fallback_air_path,
                fallback_modeled_up,
                fallback_response_time,
                maximum_aoa,
                sink,
                target_sink,
            );
        }
        plan.reject_reason = b"preflare-speed-aoa-or-stall-gate\0" as *const u8
            as *const ::core::ffi::c_char;
        return plan;
    }
    plan.height = vertical.height;
    let mut predicted_v2: ::core::ffi::c_double = predicted_speed * predicted_speed;
    let mut preflare_ground: ::core::ffi::c_double = trigger
        / fmax(tan(slope * DEG2RAD), 1e-3f64);
    let mut reference_speed: ::core::ffi::c_double = if (*g).terminal_glide_mode
        as ::core::ffi::c_int != 0
    {
        fmax(terminal_final_speed_target(g, v, s, preflare_ground), minimum_speed)
    } else {
        fmax((*v).final_approach_speed, minimum_speed)
    };
    let mut margin_v2: ::core::ffi::c_double = predicted_v2
        - minimum_speed * minimum_speed;
    if trigger.is_finite() as i32 == 0 || predicted_v2.is_finite() as i32 == 0
        || (*t).true_air_speed < minimum_speed
        || (*t).angle_of_attack > (*v).maximum_angle_of_attack
        || (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
            && (*t).stall_fraction >= 1.0f64
    {
        plan.trigger_altitude = trigger;
        plan.minimum_speed = minimum_speed;
        plan.reject_reason = b"preflare-speed-aoa-or-stall-gate\0" as *const u8
            as *const ::core::ffi::c_char;
        return plan;
    }
    plan.feasible = true_0 != 0;
    plan.trigger_altitude = trigger;
    plan.target_aoa = target_aoa;
    plan.target_sink = -target_sink;
    plan.minimum_speed = minimum_speed;
    plan.reference_speed = reference_speed;
    plan.predicted_height_loss = required_height;
    plan.predicted_kinetic_margin = 0.5f64 * margin_v2;
    plan.effective_accel = modeled_up;
    plan.response_time = response_time;
    plan.plan_source_ut = (*t).ut;
    plan.reject_reason = b"none\0" as *const u8 as *const ::core::ffi::c_char;
    return plan;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_store_preflare_plan(
    mut g: *mut GuidanceMachine,
    mut p: *const TerminalPreflarePlan,
) {
    if !p.is_null() && (*p).feasible as ::core::ffi::c_int != 0
        && (*p).degraded_floor as ::core::ffi::c_int != 0
    {
        (*g).terminal_final_low_energy = true_0 != 0;
        p = ::core::ptr::null::<TerminalPreflarePlan>();
    }
    (*g).terminal_preflare_plan_valid = !p.is_null()
        && (*p).feasible as ::core::ffi::c_int != 0;
    if p.is_null() || !(*p).feasible {
        return;
    }
    (*g).preflare_trigger_altitude = (*p).trigger_altitude;
    (*g).preflare_target_aoa = (*p).target_aoa;
    (*g).preflare_target_sink = (*p).target_sink;
    (*g).preflare_minimum_speed = (*p).minimum_speed;
    (*g).preflare_reference_speed = (*p).reference_speed;
    (*g).preflare_predicted_height_loss = (*p).predicted_height_loss;
    (*g).preflare_predicted_kinetic_margin = (*p).predicted_kinetic_margin;
    (*g).preflare_effective_accel = (*p).effective_accel;
    (*g).preflare_response_time = (*p).response_time;
    (*g).preflare_plan_source_ut = (*p).plan_source_ut;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_set_stage(
    mut g: *mut GuidanceMachine,
    mut stage: TerminalVerticalStage,
    mut ut: ::core::ffi::c_double,
) {
    if (*g).terminal_vertical_stage_valid as ::core::ffi::c_int != 0
        && (*g).terminal_vertical_stage as ::core::ffi::c_uint
            == stage as ::core::ffi::c_uint
    {
        return;
    }
    if (*g).terminal_vertical_stage_valid as ::core::ffi::c_int != 0
        && (stage as ::core::ffi::c_uint)
            < (*g).terminal_vertical_stage as ::core::ffi::c_uint
    {
        return;
    }
    (*g).terminal_vertical_stage = stage;
    (*g).terminal_vertical_stage_valid = true_0 != 0;
    (*g).terminal_stage_started_ut = ut;
    (*g).terminal_stage_good_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).flare_sink_captured = false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_outer_gate(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut out_plan: *mut TerminalPreflarePlan,
) -> bool {
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut slope: ::core::ffi::c_double = terminal_outer_glide_slope(g, s);
    let mut distance: ::core::ffi::c_double = fmax(0.0f64, -(*t).runway_along_track);
    let mut plan: TerminalPreflarePlan = terminal_preflare_plan(g, t, p, aero, cfg);
    if !out_plan.is_null() {
        *out_plan = plan;
    }
    if !plan.feasible || !plan.height.valid || plan.height.margin < 0.0f64 {
        return false_0 != 0;
    }
    let mut envelope: RunwayCaptureEnvelope = decision_runway_capture_envelope(
        g,
        t,
        course,
        p,
        cfg,
    );
    if !envelope.valid {
        return false_0 != 0;
    }
    let mut tangent: ::core::ffi::c_double = tan(slope * DEG2RAD);
    if !(tangent > 0.0f64) || !((*t).horizontal_speed > 0.0f64) {
        return false_0 != 0;
    }
    let mut preflare_ground: ::core::ffi::c_double = plan.trigger_altitude / tangent;
    let mut ground_to_preflare: ::core::ffi::c_double = distance - preflare_ground;
    if !(ground_to_preflare > 0.0f64) {
        return false_0 != 0;
    }
    let mut time_to_preflare: ::core::ffi::c_double = ground_to_preflare
        / (*t).horizontal_speed;
    let mut runway_aligned: bool = terminal_preflare_alignment_valid(g, t, course, cfg);
    if !runway_aligned {
        if !(*g).diagnostic_shadow
            && !getenv(
                    b"KSP_LANDER_GATE_TRACE\0" as *const u8 as *const ::core::ffi::c_char,
                )
                .is_null()
        {
            fprintf(
                stderr,
                b"GATE_TRACE ut=%.2f not-aligned hac=%d\n\0" as *const u8
                    as *const ::core::ffi::c_char,
                (*t).ut,
                if (*g).hac_completed as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
            );
        }
        return false_0 != 0;
    }
    let mut lateral_time: ::core::ffi::c_double = 0.0f64;
    let mut capture_time: DecisionMargin = decision_margin(
        time_to_preflare,
        lateral_time,
    );
    if !(*g).diagnostic_shadow
        && !getenv(b"KSP_LANDER_GATE_TRACE\0" as *const u8 as *const ::core::ffi::c_char)
            .is_null()
    {
        fprintf(
            stderr,
            b"GATE_TRACE ut=%.2f along=%.0f runway=%.1f vrec=%.1f speed=%.1f q=%.1f load=%.2f capture=%d/%.1f kin=%.0f hac=%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            (*t).runway_along_track,
            envelope.runway_remaining.margin,
            envelope.vertical_recovery.margin,
            envelope.speed.margin,
            envelope.dynamic_pressure.margin,
            envelope.load.margin,
            if capture_time.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            capture_time.margin,
            plan.predicted_kinetic_margin,
            if (*g).hac_completed as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
        );
    }
    return (*t).runway_along_track < 0.0f64 && envelope.runway_remaining.margin >= 0.0f64
        && envelope.vertical_recovery.margin >= 0.0f64 && envelope.speed.margin >= 0.0f64
        && envelope.dynamic_pressure.margin >= 0.0f64 && envelope.load.margin >= 0.0f64
        && capture_time.valid as ::core::ffi::c_int != 0 && capture_time.margin >= 0.0f64
        && plan.predicted_kinetic_margin >= 0.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_outer_capture_admissible(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut out_plan: *mut TerminalPreflarePlan,
) -> bool {
    return terminal_outer_gate(g, t, course, p, aero, cfg, out_plan);
}
#[no_mangle]
pub unsafe extern "C" fn terminal_delivery_contract(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut plan: *const TerminalPreflarePlan,
) -> TaemTerminalContract {
    let mut contract: TaemTerminalContract = TaemTerminalContract {
        valid: 0 as ::core::ffi::c_int != 0,
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
    };
    if g.is_null() || t.is_null() || p.is_null() || cfg.is_null() || plan.is_null()
        || !(*plan).feasible
    {
        return contract;
    }
    let mut site: *const LandingSite = &raw const (*cfg).site;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut envelope: RunwayCaptureEnvelope = decision_runway_capture_envelope(
        g,
        t,
        course,
        p,
        cfg,
    );
    if !envelope.valid {
        return contract;
    }
    let mut slope: ::core::ffi::c_double = terminal_outer_glide_slope(g, s);
    let mut tangent: ::core::ffi::c_double = tan(slope * DEG2RAD);
    let mut distance: ::core::ffi::c_double = fmax(0.0f64, -(*t).runway_along_track);
    if !(tangent > 0.0f64) || !((*t).horizontal_speed > 0.0f64) {
        return contract;
    }
    let mut preflare_ground: ::core::ffi::c_double = (*plan).trigger_altitude / tangent;
    let mut ground_to_gate: ::core::ffi::c_double = distance - preflare_ground;
    if !(ground_to_gate > 0.0f64) {
        return contract;
    }
    let mut response_available: ::core::ffi::c_double = ground_to_gate
        / (*t).horizontal_speed;
    if (*plan).response_time.is_finite() as i32 == 0 {
        return contract;
    }
    let mut runway_aligned: bool = terminal_preflare_alignment_valid(g, t, course, cfg);
    let mut response_required: ::core::ffi::c_double = if runway_aligned
        as ::core::ffi::c_int != 0
    {
        0.0f64
    } else {
        fmax(envelope.lateral_capture_time_s, envelope.heading_capture_time_s)
    };
    let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
    if !(gravity > 0.0f64) {
        return contract;
    }
    let mut height_to_gate: ::core::ffi::c_double = fmax(
        0.0f64,
        (*t).radar_altitude - (*plan).trigger_altitude,
    );
    let mut air_path: ::core::ffi::c_double = hypot(ground_to_gate, height_to_gate);
    let mut minimum_speed: ::core::ffi::c_double = (*plan).minimum_speed;
    let mut gate_speed: ::core::ffi::c_double = fmax(
        fmax((*plan).reference_speed, (*v).final_approach_speed),
        minimum_speed,
    );
    let mut aero: AerodynamicModel = AerodynamicModel {
        lift_to_drag: (*v).estimated_lift_to_drag,
        ballistic_coefficient: if (*t).estimated_ballistic_coefficient > 0.0f64 {
            (*t).estimated_ballistic_coefficient
        } else {
            (*v).estimated_ballistic_coefficient
        },
        confidence: if (*t).aerodynamic_confidence > 0.0f64 {
            (*t).aerodynamic_confidence
        } else {
            1.0f64
        },
    };
    let mut live_loss: ::core::ffi::c_double = terminal_expected_energy_loss_accel(
        g,
        t,
        aero,
        v,
    );
    if live_loss.is_finite() as i32 == 0 || live_loss < 0.0f64 {
        live_loss = if (*t).mass > 0.0f64 && (*t).drag_force.is_finite() as i32 != 0
            && (*t).drag_force >= 0.0f64
        {
            (*t).drag_force / (*t).mass
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
    }
    if live_loss.is_finite() as i32 == 0 {
        return contract;
    }
    let mut predicted_gate_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut predicted_drag_work: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    if !terminal_quadratic_drag_energy_step(
        (*t).true_air_speed,
        live_loss,
        air_path,
        gravity * height_to_gate,
        &raw mut predicted_gate_speed,
        &raw mut predicted_drag_work,
    ) {
        return contract;
    }
    let mut proj_gate_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut proj_gate_alt: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut conditioning_aoa: ::core::ffi::c_double = terminal_fpa_force_aoa(
        t,
        p,
        aero,
        v,
        -slope,
    );
    let mut proj_work: ::core::ffi::c_double = terminal_projected_drag_work_state_anchored(
        g,
        t,
        p,
        aero,
        v,
        s,
        conditioning_aoa,
        ground_to_gate,
        slope,
        &raw mut proj_gate_speed,
        &raw mut proj_gate_alt,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
    if proj_work.is_finite() as i32 != 0 && proj_gate_speed.is_finite() as i32 != 0
        && proj_gate_speed > 0.0f64
    {
        predicted_gate_speed = fmin(predicted_gate_speed, proj_gate_speed);
    }
    let mut predicted_gate_v2: ::core::ffi::c_double = predicted_gate_speed
        * predicted_gate_speed;
    let mut planned_exit_v2: ::core::ffi::c_double = minimum_speed * minimum_speed
        + 2.0f64 * (*plan).predicted_kinetic_margin;
    if planned_exit_v2.is_finite() as i32 == 0 || planned_exit_v2 < 0.0f64 {
        return contract;
    }
    let mut low_energy_margin: ::core::ffi::c_double = (*plan).predicted_kinetic_margin;
    let mut gate_energy: ::core::ffi::c_double = 0.5f64 * gate_speed * gate_speed;
    let mut planned_exit_energy: ::core::ffi::c_double = 0.5f64 * planned_exit_v2;
    let mut high_energy_margin: ::core::ffi::c_double = gate_energy
        - planned_exit_energy;
    let mut predicted_touchdown_speed: ::core::ffi::c_double = terminal_delivery_touchdown_speed(
        g,
        t,
        p,
        cfg,
        (*plan).trigger_altitude,
    );
    let mut touchdown_target: ::core::ffi::c_double = if (*v).touchdown_speed > 1.0f64 {
        (*v).touchdown_speed
    } else {
        65.0f64
    };
    let mut touchdown_energy_margin: ::core::ffi::c_double = if predicted_touchdown_speed
        .is_finite() as i32 != 0
    {
        0.5f64
            * (predicted_touchdown_speed * predicted_touchdown_speed
                - touchdown_target * touchdown_target)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut delivery_diag: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !delivery_diag.is_null()
        && strcmp(delivery_diag, b"2\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
        && ((*g).mm305_runway_alignment_active as ::core::ffi::c_int != 0
            || (*g).mm305_hac_exit_reached as ::core::ffi::c_int != 0
            || (*g).hac_completed as ::core::ffi::c_int != 0
            || (*g).final_approach_captured as ::core::ffi::c_int != 0)
    {
        fprintf(
            stderr,
            b"FINAL_DELIVERY_ENERGY ut=%.2f distance=%.1f along=%.1f h=%.1f radar=%.1f V=%.2f fpa=%.2f trigger=%.1f preflareGround=%.1f groundToGate=%.1f minV=%.2f refV=%.2f predictedExitV=%.2f predictedGateV=%.2f predictedTouchdownV=%.2f touchdownMargin=%.1f lowMargin=%.1f highMargin=%.1f liveDrag=%.4f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            distance,
            (*t).runway_along_track,
            (*t).mean_altitude - (*site).altitude,
            (*t).radar_altitude,
            (*t).true_air_speed,
            (*t).flight_path_angle,
            (*plan).trigger_altitude,
            preflare_ground,
            ground_to_gate,
            minimum_speed,
            gate_speed,
            sqrt(fmax(0.0f64, planned_exit_v2)),
            predicted_gate_speed,
            predicted_touchdown_speed,
            touchdown_energy_margin,
            low_energy_margin,
            high_energy_margin,
            live_loss,
        );
    }
    let mut energy_roundoff: ::core::ffi::c_double = 1e-9f64 * fmax(gate_energy, 1.0f64);
    if high_energy_margin < 0.0f64 && high_energy_margin >= -energy_roundoff {
        high_energy_margin = 0.0f64;
    }
    if !getenv(
            b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null() && (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
        && !(*g).terminal_glide_mode
    {
        static mut final_energy_diag_printed: bool = false_0 != 0;
        if !final_energy_diag_printed {
            fprintf(
                stderr,
                b"FINAL_TEST_ENERGY preTriggerSpeed=%.6f plannedExitSpeed=%.6f minimumSpeed=%.6f referenceGateSpeed=%.6f predictedTouchdownSpeed=%.6f touchdownMargin=%.6f lowMargin=%.6f highMargin=%.6f liveDragAccel=%.6f airPath=%.6f heightToGate=%.6f groundToGate=%.6f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if predicted_gate_v2 >= 0.0f64 {
                    sqrt(predicted_gate_v2)
                } else {
                    ::core::f32::NAN as ::core::ffi::c_double
                },
                sqrt(planned_exit_v2),
                minimum_speed,
                gate_speed,
                predicted_touchdown_speed,
                touchdown_energy_margin,
                low_energy_margin,
                high_energy_margin,
                live_loss,
                air_path,
                height_to_gate,
                ground_to_gate,
            );
            final_energy_diag_printed = true_0 != 0;
        }
    }
    let mut down_fpa: ::core::ffi::c_double = fmax(0.0f64, -(*t).flight_path_angle);
    let mut fpa_margin: ::core::ffi::c_double = envelope.recoverable_fpa_deg - down_fpa;
    let mut current_specific: ::core::ffi::c_double = gravity
        * ((*t).mean_altitude - (*site).altitude)
        + 0.5f64 * (*t).true_air_speed * (*t).true_air_speed;
    if response_available.is_finite() as i32 == 0
        || response_required.is_finite() as i32 == 0
        || low_energy_margin.is_finite() as i32 == 0
        || high_energy_margin.is_finite() as i32 == 0
        || touchdown_energy_margin.is_finite() as i32 == 0
        || current_specific.is_finite() as i32 == 0 || fpa_margin.is_finite() as i32 == 0
    {
        return contract;
    }
    contract.valid = true_0 != 0;
    contract.path_committed = (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
        || (*g).terminal_path_committed as ::core::ffi::c_int != 0
            && ((*g).mm305_route_committed as ::core::ffi::c_int != 0
                || (*g).terminal_path_kind as ::core::ffi::c_uint
                    == TERMINAL_PATH_DIRECT as ::core::ffi::c_int
                        as ::core::ffi::c_uint);
    contract.range_to_go = distance;
    contract.range_margin = (response_available - response_required)
        * (*t).horizontal_speed;
    contract.dynamic_pressure = (*t).dynamic_pressure;
    contract.dynamic_pressure_margin = (*v).maximum_dynamic_pressure
        - (*t).dynamic_pressure;
    contract.speedbrake_required = false_0 != 0;
    contract.speedbrake_available = false_0 != 0;
    contract.speedbrake_dynamic_pressure_margin = (*v).maximum_dynamic_pressure
        - (*t).dynamic_pressure;
    contract.altitude = (*t).mean_altitude;
    contract.altitude_margin = fmin(
        envelope.vertical_recovery.margin,
        if (*plan).height.valid as ::core::ffi::c_int != 0 {
            (*plan).height.margin
        } else {
            -::core::f32::INFINITY as ::core::ffi::c_double
        },
    );
    contract.flight_path_angle = (*t).flight_path_angle;
    contract.flight_path_angle_margin = fpa_margin;
    contract.specific_energy = current_specific;
    contract.specific_energy_margin = fmin(
        fmin(low_energy_margin, high_energy_margin),
        touchdown_energy_margin,
    );
    contract.response_time_available = response_available;
    contract.response_time_required = response_required;
    contract.attitude_response_qualified = (*plan).effective_accel > 0.0f64
        && envelope.vertical_recovery_accel_mps2 > 0.0f64;
    return contract;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_final_conditioning_reachable(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut energy_out: *mut EnergyPathEnvelope,
    mut end_speed_out: *mut ::core::ffi::c_double,
    mut slope_out: *mut ::core::ffi::c_double,
) -> bool {
    if !energy_out.is_null() {
        *energy_out = EnergyPathEnvelope {
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
    }
    if !end_speed_out.is_null() {
        *end_speed_out = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if !slope_out.is_null() {
        *slope_out = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if g.is_null() || t.is_null() || p.is_null() || cfg.is_null() {
        return false_0 != 0;
    }
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut capture: RunwayCaptureEnvelope = decision_runway_capture_envelope(
        g,
        t,
        course,
        p,
        cfg,
    );
    let mut capture_valid_except_vertical: bool = capture.runway_time.valid
        as ::core::ffi::c_int != 0 && capture.speed.valid as ::core::ffi::c_int != 0
        && capture.dynamic_pressure.valid as ::core::ffi::c_int != 0
        && capture.load.valid as ::core::ffi::c_int != 0
        && capture.runway_remaining.valid as ::core::ffi::c_int != 0
        && capture.lateral_capture_time_s.is_finite() as i32 != 0
        && capture.heading_capture_time_s.is_finite() as i32 != 0;
    let mut capture_reachable_except_vertical: bool = capture_valid_except_vertical
        as ::core::ffi::c_int != 0 && capture.runway_time.margin >= 0.0f64
        && capture.speed.margin >= 0.0f64 && capture.dynamic_pressure.margin >= 0.0f64
        && capture.load.margin >= 0.0f64 && capture.runway_remaining.margin >= 0.0f64;
    if !capture_reachable_except_vertical {
        let mut trace: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_FINAL_CONDITIONING_DIAGNOSTICS\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if !trace.is_null()
            && strcmp(trace, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int
            && (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
        {
            fprintf(
                stderr,
                b"FINAL_CONDITIONING reason=capture ut=%.3f x=%.1f cross=%.1f h=%.1f sink=%.2f v=%.2f runwayTime=%.2f/%.2f/%.2f lateral=%.2f heading=%.2f speed=%.2f q=%.1f load=%.2f runway=%.1f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*t).runway_along_track,
                (*t).runway_cross_track,
                (*t).radar_altitude,
                (*t).vertical_speed,
                (*t).true_air_speed,
                capture.time_available_s,
                capture.lateral_capture_time_s,
                capture.runway_time.margin,
                capture.lateral_capture_time_s,
                capture.heading_capture_time_s,
                capture.speed.margin,
                capture.dynamic_pressure.margin,
                capture.load.margin,
                capture.runway_remaining.margin,
            );
        }
        return false_0 != 0;
    }
    let mut radar: ::core::ffi::c_double = if (*t).radar_altitude.is_finite() as i32 != 0
    {
        (*t).radar_altitude
    } else {
        (*t).mean_altitude - (*cfg).site.altitude
    };
    let mut height: ::core::ffi::c_double = fmax(0.0f64, radar - (*s).flare_altitude);
    let mut ground: ::core::ffi::c_double = fmax(0.0f64, -(*t).runway_along_track);
    if !(height > 0.0f64) || !(ground > 1.0f64) {
        return false_0 != 0;
    }
    let mut slope: ::core::ffi::c_double = atan2(height, ground) * RAD2DEG;
    if slope.is_finite() as i32 == 0 || slope > 35.0f64 {
        let mut trace_0: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_FINAL_CONDITIONING_DIAGNOSTICS\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if !trace_0.is_null()
            && strcmp(trace_0, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int
            && (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
        {
            fprintf(
                stderr,
                b"FINAL_CONDITIONING reason=slope ut=%.3f x=%.1f h=%.1f ground=%.1f slope=%.2f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*t).runway_along_track,
                height,
                ground,
                slope,
            );
        }
        return false_0 != 0;
    }
    slope = fmax(0.0f64, slope);
    let mut conditioning_aoa: ::core::ffi::c_double = terminal_fpa_force_aoa(
        t,
        p,
        aero,
        v,
        -slope,
    );
    let mut end_speed: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut end_altitude: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut work: ::core::ffi::c_double = terminal_projected_drag_work_state(
        g,
        t,
        p,
        aero,
        v,
        s,
        conditioning_aoa,
        ground,
        slope,
        &raw mut end_speed,
        &raw mut end_altitude,
    );
    let mut gate_altitude: ::core::ffi::c_double = (*cfg).site.altitude
        + (*s).flare_altitude;
    let mut minimum_speed: ::core::ffi::c_double = fmax((*v).minimum_safe_speed, 1.0f64);
    let mut available: ::core::ffi::c_double = entry_remaining_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        (*cfg).site.latitude,
        gate_altitude,
        minimum_speed,
        p,
    );
    if work.is_finite() as i32 == 0 || work < 0.0f64 || available.is_finite() as i32 == 0
        || available < 0.0f64 || end_speed.is_finite() as i32 == 0
        || end_speed < minimum_speed
    {
        let mut trace_1: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_FINAL_CONDITIONING_DIAGNOSTICS\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        if !trace_1.is_null()
            && strcmp(trace_1, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int
            && (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
        {
            fprintf(
                stderr,
                b"FINAL_CONDITIONING reason=projection ut=%.3f x=%.1f cross=%.1f h=%.1f sink=%.2f v=%.2f ground=%.1f height=%.1f slope=%.2f aoa=%.2f work=%.1f endSpeed=%.2f minimum=%.2f available=%.1f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*t).runway_along_track,
                (*t).runway_cross_track,
                (*t).radar_altitude,
                (*t).vertical_speed,
                (*t).true_air_speed,
                ground,
                height,
                slope,
                conditioning_aoa,
                work,
                end_speed,
                minimum_speed,
                available,
            );
        }
        return false_0 != 0;
    }
    let mut energy: EnergyPathEnvelope = decision_energy_path_envelope(
        t,
        p,
        available,
        work,
        ground,
    );
    if !energy_out.is_null() {
        *energy_out = energy;
    }
    if !end_speed_out.is_null() {
        *end_speed_out = end_speed;
    }
    if !slope_out.is_null() {
        *slope_out = slope;
    }
    let mut trace_2: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_FINAL_CONDITIONING_DIAGNOSTICS\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    if !trace_2.is_null()
        && strcmp(trace_2, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
        && (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
    {
        static mut last_trace_ut: ::core::ffi::c_double = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        if last_trace_ut.is_finite() as i32 == 0 || (*t).ut - last_trace_ut >= 1.0f64 {
            fprintf(
                stderr,
                b"FINAL_CONDITIONING reason=energy ut=%.3f x=%.1f cross=%.1f h=%.1f sink=%.2f v=%.2f ground=%.1f height=%.1f slope=%.2f aoa=%.2f work=%.1f endSpeed=%.2f minimum=%.2f available=%.1f margin=%.1f feasible=%d\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*t).runway_along_track,
                (*t).runway_cross_track,
                (*t).radar_altitude,
                (*t).vertical_speed,
                (*t).true_air_speed,
                ground,
                height,
                slope,
                conditioning_aoa,
                work,
                end_speed,
                minimum_speed,
                available,
                energy.energy.margin,
                if energy.feasible as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
            );
            last_trace_ut = (*t).ut;
        }
    }
    return energy.valid as ::core::ffi::c_int != 0
        && energy.feasible as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_begin_final_test(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut message: *mut ::core::ffi::c_char,
    mut message_size: size_t,
) -> bool {
    if g.is_null() || t.is_null() || p.is_null() || cfg.is_null() {
        if !message.is_null() && message_size != 0 {
            snprintf(
                message,
                message_size,
                b"Final-only test initialization received an invalid argument.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    if !((*t).radar_altitude > 0.0f64) || !((*t).true_air_speed > 0.0f64)
        || (*t).runway_along_track >= 0.0f64
    {
        if !message.is_null() && message_size != 0 {
            snprintf(
                message,
                message_size,
                b"Final-only test requires an airborne pre-threshold runway-referenced state.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    guidance_machine_init(g);
    guidance_set_engaged(g, true_0 != 0);
    (*g).terminal_glide_mode = false_0 != 0;
    (*g).terminal_final_test_mode = true_0 != 0;
    (*g).terminal_rehearsal_mode = false_0 != 0;
    (*g).terminal_test_capture_active = false_0 != 0;
    (*g).terminal_test_glide_slope = 0.0f64;
    (*g).terminal_test_final_approach_distance = 0.0f64;
    (*g).has_burn_command_started = true_0 != 0;
    (*g).deorbit_burn_completed = true_0 != 0;
    (*g).atmospheric_interface_crossed = true_0 != 0;
    (*g).terminal_region_entered = true_0 != 0;
    (*g).terminal_path_kind = TERMINAL_PATH_NONE;
    (*g).terminal_path_committed = true_0 != 0;
    (*g).hac_completed = true_0 != 0;
    (*g).hac_captured = false_0 != 0;
    (*g).hac_progress_valid = false_0 != 0;
    (*g).hac_side_selected = false_0 != 0;
    (*g).hac_radius = 0.0f64;
    if !taem_exec_enter(g, t, &raw const (*cfg).guidance, p, aero, cfg, true_0 != 0) {
        guidance_machine_init(g);
        if !message.is_null() && message_size != 0 {
            snprintf(
                message,
                message_size,
                b"Final-only test could not reconstruct TAEM final-intercept ownership.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    if !getenv(
            b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        let mut e: RunwayCaptureEnvelope = decision_runway_capture_envelope(
            g,
            t,
            course,
            p,
            cfg,
        );
        fprintf(
            stderr,
            b"FINAL_TEST_CAPTURE valid=%d reachable=%d runwayTime=%.6f/%.6f/%.6f vertical=%.6f/%.6f/%.6f speed=%.6f/%.6f/%.6f q=%.6f/%.6f/%.6f load=%.6f/%.6f/%.6f runwayRemaining=%.6f/%.6f/%.6f groundRange=%.6f timeAvailable=%.6f lateralTime=%.6f headingTime=%.6f responseTime=%.6f crossRate=%.6f lateralAccel=%.6f courseRate=%.6f maxLiftAoA=%.6f maxNormalAccel=%.6f sink=%.6f verticalRecoveryAccel=%.6f responseDownAccel=%.6f recoveryHeight=%.6f recoverableSink=%.6f recoverableFPA=%.6f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            if e.valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if e.reachable as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            e.runway_time.available,
            e.runway_time.required,
            e.runway_time.margin,
            e.vertical_recovery.available,
            e.vertical_recovery.required,
            e.vertical_recovery.margin,
            e.speed.available,
            e.speed.required,
            e.speed.margin,
            e.dynamic_pressure.available,
            e.dynamic_pressure.required,
            e.dynamic_pressure.margin,
            e.load.available,
            e.load.required,
            e.load.margin,
            e.runway_remaining.available,
            e.runway_remaining.required,
            e.runway_remaining.margin,
            e.ground_range_m,
            e.time_available_s,
            e.lateral_capture_time_s,
            e.heading_capture_time_s,
            e.control_response_time_s,
            e.cross_rate_mps,
            e.lateral_accel_mps2,
            e.course_rate_deg_s,
            e.maximum_lift_aoa_deg,
            e.maximum_normal_accel_mps2,
            e.sink_rate_mps,
            e.vertical_recovery_accel_mps2,
            e.response_down_accel_mps2,
            e.vertical_recovery_height_m,
            e.recoverable_sink_rate_mps,
            e.recoverable_fpa_deg,
        );
    }
    let mut plan: TerminalPreflarePlan = TerminalPreflarePlan {
        feasible: 0 as ::core::ffi::c_int != 0,
        reject_reason: ::core::ptr::null::<::core::ffi::c_char>(),
        trigger_altitude: 0.,
        target_aoa: 0.,
        target_sink: 0.,
        minimum_speed: 0.,
        reference_speed: 0.,
        predicted_height_loss: 0.,
        predicted_kinetic_margin: 0.,
        effective_accel: 0.,
        response_time: 0.,
        plan_source_ut: 0.,
        height: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        degraded_floor: false,
    };
    let mut approach: bool = terminal_outer_gate(
        g,
        t,
        course,
        p,
        aero,
        cfg,
        &raw mut plan,
    );
    let mut contract: TaemTerminalContract = terminal_delivery_contract(
        g,
        t,
        course,
        p,
        cfg,
        &raw mut plan,
    );
    let mut evaluation: TaemTerminalEvaluation = taem_exec_evaluate_terminal_contract(
        &raw mut contract,
    );
    let mut conditioning_energy: EnergyPathEnvelope = EnergyPathEnvelope {
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
    let mut conditioning_end_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut conditioning_slope: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut conditioning: bool = terminal_final_conditioning_reachable(
        g,
        t,
        course,
        p,
        aero,
        cfg,
        &raw mut conditioning_energy,
        &raw mut conditioning_end_speed,
        &raw mut conditioning_slope,
    );
    let mut delivered: bool = approach as ::core::ffi::c_int != 0
        && evaluation.valid as ::core::ffi::c_int != 0
        && evaluation.feasible as ::core::ffi::c_int != 0;
    if !delivered && !conditioning {
        let mut reject: *const ::core::ffi::c_char = if !plan.reject_reason.is_null() {
            plan.reject_reason
        } else {
            b"none\0" as *const u8 as *const ::core::ffi::c_char
        };
        let mut block: *const ::core::ffi::c_char = taem_terminal_block_reason_string(
            evaluation.block_reason,
        );
        if !getenv(
                b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        {
            fprintf(
                stderr,
                b"FINAL_TEST_ADMISSION result=reject approach=%d preflare=%s terminal=%s conditioning=0 rangeMargin=%.6f qMargin=%.6f altitudeMargin=%.6f fpaMargin=%.6f energyMargin=%.6f responseAvailable=%.6f responseRequired=%.6f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if approach as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                reject,
                block,
                contract.range_margin,
                contract.dynamic_pressure_margin,
                contract.altitude_margin,
                contract.flight_path_angle_margin,
                contract.specific_energy_margin,
                contract.response_time_available,
                contract.response_time_required,
            );
        }
        if !message.is_null() && message_size != 0 {
            snprintf(
                message,
                message_size,
                b"Final-only snapshot rejected: no physically reachable runway-line conditioning path remains (approach=%d preflare=%s terminal=%s).\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if approach as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                reject,
                block,
            );
        }
        guidance_machine_init(g);
        return false_0 != 0;
    }
    (*g).terminal_glide_mode = true_0 != 0;
    if !getenv(
            b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        if delivered {
            fprintf(
                stderr,
                b"FINAL_TEST_ADMISSION result=accept approach=1 preflare=ok terminal=ok conditioning=1 rangeMargin=%.6f qMargin=%.6f altitudeMargin=%.6f fpaMargin=%.6f energyMargin=%.6f responseAvailable=%.6f responseRequired=%.6f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                contract.range_margin,
                contract.dynamic_pressure_margin,
                contract.altitude_margin,
                contract.flight_path_angle_margin,
                contract.specific_energy_margin,
                contract.response_time_available,
                contract.response_time_required,
            );
        } else {
            fprintf(
                stderr,
                b"FINAL_TEST_ADMISSION result=condition approach=%d preflare=%s terminal=%s conditioning=1 slope=%.6f projectedEndSpeed=%.6f energyMargin=%.6f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if approach as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if !plan.reject_reason.is_null() {
                    plan.reject_reason
                } else {
                    b"none\0" as *const u8 as *const ::core::ffi::c_char
                },
                taem_terminal_block_reason_string(evaluation.block_reason),
                conditioning_slope,
                conditioning_end_speed,
                conditioning_energy.energy.margin,
            );
        }
    }
    (*g).phase = PHASE_TAEM;
    if !message.is_null() && message_size != 0 {
        snprintf(
            message,
            message_size,
            if delivered as ::core::ffi::c_int != 0 {
                b"Final-only post-HAC test armed from runway-aligned recorded state; TAEM final-intercept contract is feasible.\0"
                    as *const u8 as *const ::core::ffi::c_char
            } else {
                b"Final-only post-HAC test armed from runway-aligned recorded state; TAEM runway alignment will condition energy until the unchanged final-intercept contract becomes feasible.\0"
                    as *const u8 as *const ::core::ffi::c_char
            },
        );
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_pavement_intercept_rate(
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
    mut rate: *mut ::core::ffi::c_double,
) -> bool {
    if t.is_null() || cfg.is_null() || rate.is_null()
        || !((*cfg).guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
            || (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0)
        || strcmp(
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0 || !((*t).runway_along_track < 0.0f64)
        || !((*t).horizontal_speed > 0.0f64) || !((*cfg).site.runway_length > 0.0f64)
        || !((*cfg).site.runway_width > 0.0f64)
        || (*t).runway_cross_track.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut ratio: ::core::ffi::c_double = 0.;
    if !runway_landing_intercept_ratio(
        (*t).runway_along_track,
        (*t).runway_cross_track,
        &raw const (*cfg).site,
        &raw const (*cfg).guidance,
        &raw mut ratio,
    ) {
        return false_0 != 0;
    }
    *rate = (*t).horizontal_speed * ratio / sqrt(1.0f64 + ratio * ratio);
    return (*rate).is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_preflare_alignment_valid(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut cfg: *const LandingConfiguration,
) -> bool {
    let mut site: *const LandingSite = &raw const (*cfg).site;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    if (*t).runway_along_track.is_finite() as i32 == 0
        || (*t).runway_cross_track.is_finite() as i32 == 0
        || course.is_finite() as i32 == 0
        || (*t).horizontal_speed.is_finite() as i32 == 0
        || !((*t).horizontal_speed > 0.0f64)
        || (*site).runway_width.is_finite() as i32 == 0
        || !((*site).runway_width > 0.0f64)
        || (*site).runway_length.is_finite() as i32 == 0
        || !((*site).runway_length > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut error: ::core::ffi::c_double = norm_signed_deg(
        course - (*site).runway_heading,
    ) * DEG2RAD;
    let mut cross_rate: ::core::ffi::c_double = (*t).horizontal_speed * sin(error);
    let mut along_rate: ::core::ffi::c_double = (*t).horizontal_speed * cos(error);
    if !(along_rate > 0.0f64) {
        return false_0 != 0;
    }
    let mut time_to_threshold: ::core::ffi::c_double = fmax(
        0.0f64,
        -(*t).runway_along_track,
    ) / along_rate;
    let mut projected_cross: ::core::ffi::c_double = (*t).runway_cross_track
        + cross_rate * time_to_threshold;
    let mut projected_runway_end_cross: ::core::ffi::c_double = projected_cross
        + (*site).runway_length * (cross_rate / along_rate);
    let mut half_width: ::core::ffi::c_double = (*site).runway_width * 0.5f64;
    let mut minimum: ::core::ffi::c_double = if !g.is_null()
        && (*g).terminal_preflare_plan_valid as ::core::ffi::c_int != 0
    {
        (*g).preflare_minimum_speed
    } else {
        (*v).minimum_safe_speed
    };
    return (*t).runway_along_track < (*site).runway_length
        && fabs(projected_cross) <= half_width
        && fabs(projected_runway_end_cross) <= half_width
        && (*t).true_air_speed >= minimum;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_runway_contact_position(
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
) -> bool {
    let mut site: *const LandingSite = &raw const (*cfg).site;
    return (*t).runway_along_track >= -40.0f64
        && (*t).runway_along_track < (*site).runway_length + 80.0f64
        && fabs((*t).runway_cross_track) < fmax((*site).runway_width * 0.75f64, 45.0f64);
}
unsafe extern "C" fn terminal_update_ground_latch(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
    mut dt: ::core::ffi::c_double,
) -> bool {
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    if (*g).ground_contact_latched {
        return true_0 != 0;
    }
    let mut over_runway: bool = terminal_runway_contact_position(t, cfg);
    let mut landed_on_runway: bool = strcasecmp(
        &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0 && over_runway as ::core::ffi::c_int != 0
        && (*t).gear as ::core::ffi::c_int != 0;
    let mut flare_stage: bool = (*g).terminal_vertical_stage_valid as ::core::ffi::c_int
        != 0
        && (*g).terminal_vertical_stage as ::core::ffi::c_uint
            >= TERMINAL_TOUCHDOWN_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut near_contact: bool = flare_stage as ::core::ffi::c_int != 0
        && (*t).gear as ::core::ffi::c_int != 0 && over_runway as ::core::ffi::c_int != 0
        && terminal_wheel_height(t, cfg) < 0.6f64 && (*t).vertical_speed > -3.0f64
        && fabs((*t).vertical_speed) < 3.0f64
        && (*t).surface_speed < fmax(190.0f64, (*v).final_approach_speed * 1.65f64);
    let mut contact_evidence: bool = landed_on_runway as ::core::ffi::c_int != 0
        || near_contact as ::core::ffi::c_int != 0;
    if contact_evidence {
        (*g).ground_contact_duration
            += fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, dt);
    } else {
        (*g).ground_contact_duration = fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            (*g).ground_contact_duration
                - fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, dt) * 2.0f64,
        );
    }
    if (*g).ground_contact_duration >= 0.25f64 {
        (*g).ground_contact_latched = true_0 != 0;
        (*g).gear_command_latched = true_0 != 0;
        terminal_set_stage(g, TERMINAL_GROUND, (*t).ut);
        return true_0 != 0;
    }
    return false_0 != 0;
}
pub const GUIDANCE_HARD_AOA_LIMIT_DEG: ::core::ffi::c_double = 46.0f64;
pub const GUIDANCE_RECOVERY_AOA_THREAT_DEG: ::core::ffi::c_double = 40.0f64;
pub const GUIDANCE_ATTITUDE_LOOKAHEAD_S: ::core::ffi::c_double = 0.75f64;
pub const SPEEDBRAKE_CAP: ::core::ffi::c_double = 0.50f64;
pub const SPEEDBRAKE_NOMINAL: ::core::ffi::c_double = 0.25f64;
pub const SPEEDBRAKE_DEPLOY_SLEW: ::core::ffi::c_double = 0.05f64;
pub const SPEEDBRAKE_STOW_SLEW: ::core::ffi::c_double = 0.20f64;
unsafe extern "C" fn terminal_geometry_fpa(
    mut height: ::core::ffi::c_double,
    mut distance: ::core::ffi::c_double,
    mut float_height: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return clampd(
        -atan2(fmax(0.0f64, height - float_height), fmax(120.0f64, distance)),
        -25.0f64 * DEG2RAD,
        -4.0f64 * DEG2RAD,
    );
}
unsafe extern "C" fn terminal_final_flare_height(
    mut float_height: ::core::ffi::c_double,
    mut float_sink: ::core::ffi::c_double,
    mut final_accel: ::core::ffi::c_double,
    mut sgs_sink: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return float_height
        + fmax(0.0f64, sgs_sink * sgs_sink - float_sink * float_sink)
            / (2.0f64 * final_accel);
}
unsafe extern "C" fn terminal_sink_profile(
    mut height: ::core::ffi::c_double,
    mut float_height: ::core::ffi::c_double,
    mut float_sink: ::core::ffi::c_double,
    mut arrest_accel: ::core::ffi::c_double,
    mut sgs_height: ::core::ffi::c_double,
    mut sgs_sink: ::core::ffi::c_double,
    mut final_accel: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if height > sgs_height {
        return sqrt(sgs_sink * sgs_sink + 2.0f64 * arrest_accel * (height - sgs_height));
    }
    if height
        > terminal_final_flare_height(float_height, float_sink, final_accel, sgs_sink)
    {
        return sgs_sink;
    }
    if height > float_height {
        return sqrt(
            float_sink * float_sink + 2.0f64 * final_accel * (height - float_height),
        );
    }
    return float_sink;
}
unsafe extern "C" fn terminal_learn_aero(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
) {
    let mut q: ::core::ffi::c_double = (*t).dynamic_pressure;
    if !(q > 200.0f64) || (*t).lift_force.is_finite() as i32 == 0
        || (*t).drag_force.is_finite() as i32 == 0
    {
        return;
    }
    let mut lf: ::core::ffi::c_double = 0.0f64;
    let mut df: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        (*t).angle_of_attack,
        v,
        &raw mut lf,
        &raw mut df,
    );
    let mut tc: Telemetry = *t;
    if (*t).has_airbrakes as ::core::ffi::c_int != 0
        && (*t).airbrakes as ::core::ffi::c_int != 0
    {
        let mut k: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
        if (*t).physics_airbrake_model_available as ::core::ffi::c_int != 0
            && (*t).physics_airbrake_drag_accel > 0.0f64 && (*t).mass > 1.0f64
        {
            k = (*t).physics_airbrake_drag_accel * (*t).mass / q;
        } else {
            let mut L: ::core::ffi::c_double = 0.0f64;
            let mut D: ::core::ffi::c_double = 0.0f64;
            terminal_aero_q(g, t, v, (*t).angle_of_attack, &raw mut L, &raw mut D);
            if D > 0.0f64 {
                k = (*t).drag_force / q - D;
            }
        }
        if k.is_finite() as i32 != 0 && k > 0.0f64 {
            (*g).terminal_speedbrake_cda = if (*g).terminal_speedbrake_cda > 0.0f64 {
                (*g).terminal_speedbrake_cda * 0.9f64 + k * 0.1f64
            } else {
                k
            };
        }
        if !((*g).terminal_speedbrake_cda > 0.0f64) {
            return;
        }
        tc.drag_force = (*t).drag_force - (*g).terminal_speedbrake_cda * q;
        t = &raw mut tc;
    } else if (*g).terminal_speedbrake_fraction > 0.0f64 {
        let mut f: ::core::ffi::c_double = (*g).terminal_speedbrake_fraction;
        if f >= SPEEDBRAKE_NOMINAL {
            let mut L_0: ::core::ffi::c_double = 0.0f64;
            let mut D_0: ::core::ffi::c_double = 0.0f64;
            terminal_aero_q(g, t, v, (*t).angle_of_attack, &raw mut L_0, &raw mut D_0);
            let mut k_0: ::core::ffi::c_double = if D_0 > 0.0f64 {
                ((*t).drag_force / q - D_0) / f
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            };
            if k_0.is_finite() as i32 != 0 && k_0 > 0.0f64 {
                (*g).terminal_split_fin_cda = if (*g).terminal_split_fin_cda > 0.0f64 {
                    (*g).terminal_split_fin_cda * 0.9f64 + k_0 * 0.1f64
                } else {
                    k_0
                };
            }
        }
        if !((*g).terminal_split_fin_cda > 0.0f64) {
            return;
        }
        tc.drag_force = (*t).drag_force - (*g).terminal_split_fin_cda * f * q;
        t = &raw mut tc;
    }
    if lf > 0.05f64 && (*t).lift_force > 0.0f64 {
        let mut k_1: ::core::ffi::c_double = (*t).lift_force / (q * lf);
        (*g).terminal_lift_area_ema = if (*g).terminal_lift_area_ema > 0.0f64 {
            (*g).terminal_lift_area_ema * 0.9f64 + k_1 * 0.1f64
        } else {
            k_1
        };
    }
    if df > 0.05f64 && (*t).drag_force > 0.0f64 {
        let mut k_2: ::core::ffi::c_double = (*t).drag_force / (q * df);
        (*g).terminal_drag_area_ema = if (*g).terminal_drag_area_ema > 0.0f64 {
            (*g).terminal_drag_area_ema * 0.9f64 + k_2 * 0.1f64
        } else {
            k_2
        };
    }
    let mut a: ::core::ffi::c_double = (*t).angle_of_attack;
    if (*t).mach > 0.8f64 || !(a >= 0.0f64 && a <= 20.0f64) {
        return;
    }
    let mut i: ::core::ffi::c_int = floor(a) as ::core::ffi::c_int;
    let mut w: ::core::ffi::c_double = a - i as ::core::ffi::c_double;
    let mut k_3: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while k_3 < 2 as ::core::ffi::c_int {
        let mut j: ::core::ffi::c_int = i + k_3;
        if !(j < 0 as ::core::ffi::c_int || j > 20 as ::core::ffi::c_int) {
            let mut wt: ::core::ffi::c_double = if k_3 != 0 { w } else { 1.0f64 - w };
            if !(wt < 0.2f64) {
                let mut L_1: ::core::ffi::c_double = (*t).lift_force / q;
                let mut D_1: ::core::ffi::c_double = (*t).drag_force / q;
                let mut e: ::core::ffi::c_double = 0.25f64 * wt;
                if (*g).terminal_aero_seen[j as usize] == 0
                    || fabs((*t).mach - (*g).terminal_aero_mach[j as usize]) > 0.04f64
                {
                    (*g).terminal_lift_q[j as usize] = L_1;
                    (*g).terminal_drag_q[j as usize] = D_1;
                    (*g).terminal_aero_seen[j as usize] = 1 as ::core::ffi::c_uchar;
                } else {
                    (*g).terminal_lift_q[j as usize]
                        += e * (L_1 - (*g).terminal_lift_q[j as usize]);
                    (*g).terminal_drag_q[j as usize]
                        += e * (D_1 - (*g).terminal_drag_q[j as usize]);
                }
                (*g).terminal_aero_mach[j as usize] = (*t).mach;
            }
        }
        k_3 += 1;
    }
}
unsafe extern "C" fn terminal_aero_q(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut aoa: ::core::ffi::c_double,
    mut L: *mut ::core::ffi::c_double,
    mut D: *mut ::core::ffi::c_double,
) {
    let mut i: ::core::ffi::c_int = floor(aoa) as ::core::ffi::c_int;
    let mut w: ::core::ffi::c_double = aoa - i as ::core::ffi::c_double;
    if (*t).mach <= 0.8f64 && i >= 0 as ::core::ffi::c_int
        && i < 20 as ::core::ffi::c_int
        && (*g).terminal_aero_seen[i as usize] as ::core::ffi::c_int != 0
        && (*g).terminal_aero_seen[(i + 1 as ::core::ffi::c_int) as usize]
            as ::core::ffi::c_int != 0
        && fabs((*t).mach - (*g).terminal_aero_mach[i as usize]) <= 0.06f64
        && fabs(
            (*t).mach - (*g).terminal_aero_mach[(i + 1 as ::core::ffi::c_int) as usize],
        ) <= 0.06f64
    {
        *L = (*g).terminal_lift_q[i as usize] * (1.0f64 - w)
            + (*g).terminal_lift_q[(i + 1 as ::core::ffi::c_int) as usize] * w;
        *D = (*g).terminal_drag_q[i as usize] * (1.0f64 - w)
            + (*g).terminal_drag_q[(i + 1 as ::core::ffi::c_int) as usize] * w;
        return;
    }
    let mut lf: ::core::ffi::c_double = 0.0f64;
    let mut df: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach((*t).mach, aoa, v, &raw mut lf, &raw mut df);
    *L = (if (*g).terminal_lift_area_ema > 0.0f64 {
        (*g).terminal_lift_area_ema
    } else {
        100.0f64
    }) * lf;
    *D = (if (*g).terminal_drag_area_ema > 0.0f64 {
        (*g).terminal_drag_area_ema
    } else {
        50.0f64
    }) * df;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_observe_landing_aero(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut gravity: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) {
    if g.is_null() || t.is_null() || v.is_null() || (*t).ut.is_finite() as i32 == 0
        || (*t).angle_of_attack.is_finite() as i32 == 0
        || (*t).mach.is_finite() as i32 == 0
        || (*t).dynamic_pressure.is_finite() as i32 == 0 || !(gravity > 0.0f64)
    {
        return;
    }
    if (*g).terminal_aero_sample_valid as ::core::ffi::c_int != 0
        && (*t).ut == (*g).terminal_aero_sample_ut
    {
        return;
    }
    terminal_learn_aero(g, t, v);
    if !((*g).terminal_lift_ratio > 0.0f64)
        || (*g).terminal_lift_ratio.is_finite() as i32 == 0
    {
        (*g).terminal_lift_ratio = 1.0f64;
    }
    let mut L: ::core::ffi::c_double = 0.0f64;
    let mut D: ::core::ffi::c_double = 0.0f64;
    terminal_aero_q(g, t, v, (*t).angle_of_attack, &raw mut L, &raw mut D);
    let mut modeled: ::core::ffi::c_double = L * fmax((*t).dynamic_pressure, 1.0f64);
    if modeled > 0.2f64 * (*t).mass * gravity && (*t).lift_force.is_finite() as i32 != 0
        && (*t).lift_force > 0.0f64
    {
        (*g).terminal_lift_ratio
            += clampd(dt / 2.0f64, 0.0f64, 1.0f64)
                * ((*t).lift_force / modeled - (*g).terminal_lift_ratio);
    }
    (*g).terminal_aero_sample_ut = (*t).ut;
    (*g).terminal_aero_sample_valid = true_0 != 0;
}
unsafe extern "C" fn terminal_wheel_height(
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
) -> ::core::ffi::c_double {
    if (*t).mean_altitude.is_finite() as i32 == 0
        || (*t).flight_path_angle.is_finite() as i32 == 0
        || (*t).angle_of_attack.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut height: ::core::ffi::c_double = (*t).mean_altitude - (*cfg).site.altitude;
    if (*t).radar_altitude.is_finite() as i32 != 0 && (*t).radar_altitude >= 0.0f64
        && terminal_runway_contact_position(t, cfg) as ::core::ffi::c_int != 0
    {
        height = (*t).radar_altitude;
    } else if (*cfg).guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 && (*t).radar_altitude.is_finite() as i32 != 0
        && (*t).radar_altitude >= 0.0f64
    {
        height = fmin(height, (*t).radar_altitude);
    }
    let mut pitch: ::core::ffi::c_double = ((*t).flight_path_angle
        + (*t).angle_of_attack) * DEG2RAD;
    return fmax(0.0f64, height - 3.1535f64 * cos(pitch) - 0.9058f64 * sin(pitch));
}
unsafe extern "C" fn terminal_flare_plan(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut trigger_altitude: ::core::ffi::c_double,
) -> TerminalFlarePlan {
    let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
    let mut limit: ::core::ffi::c_double = final_lift_aoa_limit(
        t,
        &raw const (*cfg).vehicle,
    );
    let mut L: ::core::ffi::c_double = 0.0f64;
    let mut D: ::core::ffi::c_double = 0.0f64;
    terminal_aero_q(g, t, &raw const (*cfg).vehicle, limit, &raw mut L, &raw mut D);
    let mut pull: ::core::ffi::c_double = L * fmax((*t).dynamic_pressure, 1.0f64)
        * (*g).terminal_lift_ratio / fmax((*t).mass, 1.0f64)
        - gravity * cos((*t).flight_path_angle * DEG2RAD);
    return TerminalFlarePlan {
        arrest_accel: clampd(0.4f64 * pull, 0.12f64 * gravity, 0.35f64 * gravity),
        sgs_height: 12.0f64,
        sgs_sink: 3.5f64,
        final_accel: 0.1f64 * gravity,
        float_height: 1.5f64,
        float_sink: 1.0f64,
        lift_ratio: (*g).terminal_lift_ratio,
        aoa_max: limit,
        g0: gravity,
        planet: p,
        site_altitude: (*cfg).site.altitude,
        preflare_trigger_altitude: trigger_altitude,
    };
}
#[no_mangle]
pub unsafe extern "C" fn terminal_delivery_touchdown_speed(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut trigger_altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if g.is_null() || t.is_null() || p.is_null() || cfg.is_null()
        || (*t).true_air_speed.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).mach.is_finite() as i32 == 0
        || (*t).angle_of_attack.is_finite() as i32 == 0 || !((*t).mass > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut forecast: GuidanceMachine = *g;
    let mut dt: ::core::ffi::c_double = if forecast.terminal_aero_sample_valid
        as ::core::ffi::c_int != 0
    {
        clampd((*t).ut - forecast.terminal_aero_sample_ut, 0.0f64, 1.0f64)
    } else {
        0.1f64
    };
    terminal_observe_landing_aero(
        &raw mut forecast,
        t,
        &raw const (*cfg).vehicle,
        planet_surface_gravity(p),
        dt,
    );
    let mut plan: TerminalFlarePlan = terminal_flare_plan(
        &raw mut forecast,
        t,
        p,
        cfg,
        trigger_altitude,
    );
    return terminal_predict_touchdown_speed(
        &raw mut forecast,
        t,
        &raw const (*cfg).vehicle,
        terminal_wheel_height(t, cfg),
        (*t).true_air_speed,
        fmax(0.0f64, -(*t).vertical_speed),
        false_0 != 0,
        &raw mut plan,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
}
unsafe extern "C" fn terminal_aoa_for_lift(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut lift: ::core::ffi::c_double,
    mut aoa_max: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut q: ::core::ffi::c_double = fmax((*t).dynamic_pressure, 1.0f64);
    let mut best: ::core::ffi::c_double = aoa_max;
    let mut best_l: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut a: ::core::ffi::c_double = 0.0f64;
    while a <= aoa_max + 1e-6f64 {
        let mut L: ::core::ffi::c_double = 0.0f64;
        let mut D: ::core::ffi::c_double = 0.0f64;
        terminal_aero_q(g, t, v, a, &raw mut L, &raw mut D);
        if L * q >= lift {
            return a;
        }
        if L > best_l {
            best_l = L;
            best = a;
        }
        a += 0.1f64;
    }
    return best;
}
unsafe extern "C" fn terminal_learned_max_lift_to_drag(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut aoa_max: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut best: ::core::ffi::c_double = 0.0f64;
    let mut a: ::core::ffi::c_double = 1.0f64;
    while a <= aoa_max + 1e-6f64 {
        let mut L: ::core::ffi::c_double = 0.0f64;
        let mut D: ::core::ffi::c_double = 0.0f64;
        terminal_aero_q(g, t, v, a, &raw mut L, &raw mut D);
        if D > 1e-6f64 && L / D > best {
            best = L / D;
        }
        a += 0.5f64;
    }
    return clampd(best, 1.5f64, 10.0f64);
}
unsafe extern "C" fn terminal_predict_touchdown_speed(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut height: ::core::ffi::c_double,
    mut V: ::core::ffi::c_double,
    mut sink: ::core::ffi::c_double,
    mut latched: bool,
    mut p: *const TerminalFlarePlan,
    mut ground_range: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !ground_range.is_null() {
        *ground_range = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if height.is_finite() as i32 == 0 || V.is_finite() as i32 == 0
        || sink.is_finite() as i32 == 0 || !((*p).arrest_accel > 0.0f64)
        || !((*p).lift_ratio > 0.0f64) || !((*p).g0 > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut tc: Telemetry = *t;
    let mut m: ::core::ffi::c_double = fmax((*t).mass, 1.0f64);
    let mut sound: ::core::ffi::c_double = if (*t).mach.is_finite() as i32 != 0
        && (*t).mach > 0.05f64 && (*t).true_air_speed.is_finite() as i32 != 0
    {
        (*t).true_air_speed / (*t).mach
    } else {
        340.0f64
    };
    let mut rho_now: ::core::ffi::c_double = if (*t).atmospheric_density.is_finite()
        as i32 != 0 && (*t).atmospheric_density > 0.0f64
    {
        (*t).atmospheric_density
    } else {
        2.0f64 * fmax((*t).dynamic_pressure, 1.0f64) / fmax(V * V, 1.0f64)
    };
    let mut rho_model_now: ::core::ffi::c_double = if !(*p).planet.is_null() {
        planet_atmospheric_density((*p).planet, (*t).mean_altitude)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut sound_model_now: ::core::ffi::c_double = if !(*p).planet.is_null() {
        planet_atmospheric_speed_of_sound((*p).planet, (*t).mean_altitude)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut sin_glide: ::core::ffi::c_double = clampd(
        sink / fmax(V, 1.0f64),
        -0.95f64,
        0.95f64,
    );
    let mut travelled: ::core::ffi::c_double = 0.0f64;
    let DT: ::core::ffi::c_double = 0.25f64;
    let TAU: ::core::ffi::c_double = 1.5f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 1200 as ::core::ffi::c_int && height > 0.0f64 && V > 5.0f64 {
        if !latched {
            let mut pull: ::core::ffi::c_double = fmax(
                0.0f64,
                sink * sink - (*p).sgs_sink * (*p).sgs_sink,
            ) / (2.0f64 * (*p).arrest_accel);
            let mut target_latch: ::core::ffi::c_double = fmax(
                pull + (*p).sgs_height + sink * 1.5f64,
                if (*p).preflare_trigger_altitude.is_finite() as i32 != 0 {
                    (*p).preflare_trigger_altitude
                } else {
                    0.0f64
                },
            );
            if height <= target_latch {
                latched = true_0 != 0;
            }
        }
        let mut accel_up: ::core::ffi::c_double = 0.0f64;
        if latched {
            let mut ref_0: ::core::ffi::c_double = terminal_sink_profile(
                height,
                (*p).float_height,
                (*p).float_sink,
                (*p).arrest_accel,
                (*p).sgs_height,
                (*p).sgs_sink,
                (*p).final_accel,
            );
            accel_up = clampd(
                (sink - ref_0) / TAU
                    + (if sink > (*p).sgs_sink && height > (*p).sgs_height {
                        (*p).arrest_accel
                    } else {
                        0.0f64
                    }),
                -0.3f64 * (*p).g0,
                1.5f64 * (*p).g0,
            );
        } else {
            sink = V * sin_glide;
        }
        let mut gamma: ::core::ffi::c_double = -asin(
            clampd(sink / V, -0.95f64, 0.95f64),
        );
        let mut rho: ::core::ffi::c_double = rho_now;
        let mut a: ::core::ffi::c_double = sound;
        if rho_model_now > 0.0f64 && sound_model_now > 0.0f64 {
            rho
                *= planet_atmospheric_density((*p).planet, (*p).site_altitude + height)
                    / rho_model_now;
            a
                *= planet_atmospheric_speed_of_sound(
                    (*p).planet,
                    (*p).site_altitude + height,
                ) / sound_model_now;
        }
        tc.mach = V / a;
        tc.dynamic_pressure = 0.5f64 * rho * V * V;
        let mut lift: ::core::ffi::c_double = m * (accel_up + (*p).g0 * cos(gamma))
            / fmax((*p).lift_ratio, 0.3f64);
        let mut aoa: ::core::ffi::c_double = terminal_aoa_for_lift(
            g,
            &raw mut tc,
            v,
            fmax(0.0f64, lift),
            (*p).aoa_max,
        );
        let mut L: ::core::ffi::c_double = 0.0f64;
        let mut D: ::core::ffi::c_double = 0.0f64;
        terminal_aero_q(g, &raw mut tc, v, aoa, &raw mut L, &raw mut D);
        travelled += V * cos(gamma) * DT;
        V += (-D * tc.dynamic_pressure / m - (*p).g0 * sin(gamma)) * DT;
        if latched {
            sink -= accel_up * DT;
        }
        height -= sink * DT;
        i += 1;
    }
    if !ground_range.is_null() && height <= 0.0f64 && V.is_finite() as i32 != 0
        && V > 0.0f64
    {
        *ground_range = travelled;
    }
    return if height <= 0.0f64 && V.is_finite() as i32 != 0 && V > 0.0f64 {
        V
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
}
unsafe extern "C" fn terminal_live_equilibrium_slope(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut g0: ::core::ffi::c_double,
    mut aoa_max: ::core::ffi::c_double,
    mut distance: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut q: ::core::ffi::c_double = (*t).dynamic_pressure;
    if !(q > 0.0f64) || !((*t).mass > 0.0f64) || !(aoa_max > 0.0f64)
        || !(distance > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut ratio: ::core::ffi::c_double = if (*g).terminal_lift_ratio > 0.0f64
        && (*g).terminal_lift_ratio.is_finite() as i32 != 0
    {
        (*g).terminal_lift_ratio
    } else {
        1.0f64
    };
    let mut V: ::core::ffi::c_double = (*t).true_air_speed;
    let mut v_a: ::core::ffi::c_double = fmax(
        (*v).final_approach_speed,
        (*v).minimum_safe_speed,
    );
    if !(V > 0.0f64) || !(v_a > 0.0f64) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut kinetic: ::core::ffi::c_double = (V * V - v_a * v_a)
        / (2.0f64 * g0 * distance);
    let mut slope: ::core::ffi::c_double = if (*t).flight_path_angle.is_finite() as i32
        != 0
    {
        fabs((*t).flight_path_angle) * DEG2RAD
    } else {
        0.0f64
    };
    let mut it: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while it < 8 as ::core::ffi::c_int {
        let mut need: ::core::ffi::c_double = (*t).mass * g0 * cos(slope);
        let mut lo: ::core::ffi::c_double = 0.0f64;
        let mut hi: ::core::ffi::c_double = aoa_max;
        let mut L: ::core::ffi::c_double = 0.0f64;
        let mut D: ::core::ffi::c_double = 0.0f64;
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < 30 as ::core::ffi::c_int {
            let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
            terminal_aero_q(g, t, v, mid, &raw mut L, &raw mut D);
            if L * q * ratio < need {
                lo = mid;
            } else {
                hi = mid;
            }
            i += 1;
        }
        terminal_aero_q(g, t, v, 0.5f64 * (lo + hi), &raw mut L, &raw mut D);
        if !(L > 0.0f64) || !(D > 0.0f64) {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        let mut next: ::core::ffi::c_double = atan(
            fmax(0.0f64, D / (L * ratio) - kinetic),
        );
        if fabs(next - slope) < 1e-5f64 {
            slope = next;
            break;
        } else {
            slope = next;
            it += 1;
        }
    }
    return slope * RAD2DEG;
}
unsafe extern "C" fn terminal_landing_guidance(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut ref_0: *const Trajectory,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut site: *const LandingSite = &raw const (*cfg).site;
    let g0: ::core::ffi::c_double = planet_surface_gravity(p) as ::core::ffi::c_double;
    let H_FLOAT: ::core::ffi::c_double = 1.5f64;
    let S_FLOAT: ::core::ffi::c_double = 1.0f64;
    let FINAL_FLARE_ACCEL: ::core::ffi::c_double = 0.1f64 * g0;
    let TOUCHDOWN_ALONG: ::core::ffi::c_double = 75.0f64;
    let SGS_HEIGHT: ::core::ffi::c_double = 12.0f64;
    let SGS_SINK: ::core::ffi::c_double = 3.5f64;
    let TOUCHDOWN_PITCH: ::core::ffi::c_double = 8.0f64;
    let TOUCHDOWN_ATTITUDE_START: ::core::ffi::c_double = 26.0f64;
    let TOUCHDOWN_ATTITUDE_FULL: ::core::ffi::c_double = 5.0f64;
    let FLOOR_SETTLE_G: ::core::ffi::c_double = 0.97f64;
    let mut height: ::core::ffi::c_double = terminal_wheel_height(t, cfg);
    let mut V: ::core::ffi::c_double = fmax(1.0f64, (*t).true_air_speed);
    let mut Vh: ::core::ffi::c_double = fmax(1.0f64, (*t).horizontal_speed);
    let mut along: ::core::ffi::c_double = (*t).runway_along_track;
    let mut gamma: ::core::ffi::c_double = (*t).flight_path_angle * DEG2RAD;
    let mut aoa_max: ::core::ffi::c_double = final_lift_aoa_limit(t, v);
    let mut v_td: ::core::ffi::c_double = if (*v).touchdown_speed > 1.0f64 {
        (*v).touchdown_speed
    } else {
        65.0f64
    };
    terminal_observe_landing_aero(g, t, v, g0, dt);
    af_observe(g, t);
    let mut stage: TerminalVerticalStage = (*g).terminal_vertical_stage;
    if (stage as ::core::ffi::c_uint)
        < TERMINAL_OUTER_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        stage = TERMINAL_OUTER_FINAL;
    }
    if stage as ::core::ffi::c_uint
        == TERMINAL_OUTER_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint
        && !(*g).terminal_pull_latch && !(*g).terminal_glide_mode
    {
        let mut live_slope: ::core::ffi::c_double = terminal_live_equilibrium_slope(
            g,
            t,
            v,
            g0,
            aoa_max,
            fmax(0.0f64, -along),
        );
        if live_slope.is_finite() as i32 != 0 && live_slope >= 3.0f64
            && live_slope <= 45.0f64
        {
            (*g).terminal_final_handoff_latched = true_0 != 0;
            (*g).terminal_final_handoff_slope_deg = live_slope;
        }
    }
    if (*g).terminal_vs_prev_ut > 0.0f64 && (*t).ut > (*g).terminal_vs_prev_ut + 1e-3f64
        && (*t).ut < (*g).terminal_vs_prev_ut + 2.0f64
    {
        let mut rate: ::core::ffi::c_double = ((*t).vertical_speed
            - (*g).terminal_vs_prev) / ((*t).ut - (*g).terminal_vs_prev_ut);
        if rate.is_finite() as i32 != 0 {
            (*g).terminal_vs_rate_ema
                += 0.3f64
                    * (clampd(rate, -30.0f64, 30.0f64) - (*g).terminal_vs_rate_ema);
        }
    }
    (*g).terminal_vs_prev = (*t).vertical_speed;
    (*g).terminal_vs_prev_ut = (*t).ut;
    let mut lateral_scale: ::core::ffi::c_double = clampd(
        height / 60.0f64,
        0.0f64,
        1.0f64,
    );
    let mut track_error: ::core::ffi::c_double = norm_signed_deg(
        course - (*site).runway_heading,
    ) * DEG2RAD;
    let mut cross_rate: ::core::ffi::c_double = Vh * sin(track_error);
    let ALIGN_STATION: ::core::ffi::c_double = -2800.0f64;
    let mut align_remaining: ::core::ffi::c_double = fmax(0.0f64, ALIGN_STATION - along);
    let mut align_time: ::core::ffi::c_double = fmax(6.0f64, align_remaining / Vh);
    let mut omega: ::core::ffi::c_double = clampd(2.2f64 / align_time, 0.06f64, 0.18f64);
    let mut bank_limit: ::core::ffi::c_double = fmin(
        fmin(30.0f64, terminal_lateral_bank_limit(t, v)),
        3.0f64 + 27.0f64 * lateral_scale,
    );
    let mut lift_accel: ::core::ffi::c_double = if (*t).lift_force.is_finite() as i32
        != 0 && (*t).lift_force > 0.0f64 && (*t).mass > 1.0f64
    {
        (*t).lift_force / (*t).mass
    } else {
        g0
    };
    lift_accel = fmax(0.3f64 * g0, lift_accel);
    let mut later_bank: ::core::ffi::c_double = fmin(
        bank_limit,
        TERMINAL_LINE_CAPTURE_BANK_DEG,
    );
    let mut stop_accel: ::core::ffi::c_double = 0.6f64 * g0 * tan(later_bank * DEG2RAD);
    let mut y: ::core::ffi::c_double = clampd(
        (*t).runway_cross_track,
        -2000.0f64,
        2000.0f64,
    );
    let mut roll_limits: GuidanceAttitudeLimits = decision_guidance_attitude_limits(
        t,
        s,
        PHASE_FINAL,
        false_0 != 0,
    );
    let mut roll_now: ::core::ffi::c_double = if (*g).roll_limiter.has_value
        as ::core::ffi::c_int != 0
    {
        norm_signed_deg((*g).roll_limiter.value)
    } else {
        norm_signed_deg((*t).roll)
    };
    let mut roll_rate_now: ::core::ffi::c_double = if (*g).roll_limiter.has_value
        as ::core::ffi::c_int != 0
    {
        (*g).roll_limiter.rate
    } else if (*t).roll_rate.is_finite() as i32 != 0 {
        (*t).roll_rate
    } else {
        0.0f64
    };
    let mut reversal_time: ::core::ffi::c_double = decision_axis_capture_time(
        2.0f64 * later_bank,
        0.0f64,
        roll_limits.accel_deg_s2,
        roll_limits.rate_deg_s,
    );
    let RATE_CLOSURE_TAU: ::core::ffi::c_double = 2.0f64;
    if (*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        let mut need_accel: ::core::ffi::c_double = 0.5f64 * omega * omega * fabs(y);
        let mut need_bank: ::core::ffi::c_double = fmin(
            later_bank,
            atan(need_accel / g0) * RAD2DEG,
        );
        let mut need_reversal: ::core::ffi::c_double = decision_axis_capture_time(
            2.0f64 * need_bank,
            0.0f64,
            roll_limits.accel_deg_s2,
            roll_limits.rate_deg_s,
        );
        if need_reversal.is_finite() as i32 != 0 {
            reversal_time = need_reversal;
        }
    }
    let mut omega_limit: ::core::ffi::c_double = 0.35f64
        / (RATE_CLOSURE_TAU
            + (if reversal_time.is_finite() as i32 != 0 {
                reversal_time
            } else {
                30.0f64
            }));
    let mut rate_gain: ::core::ffi::c_double = fmin(
        fmin(omega, omega_limit),
        sqrt(2.0f64 * stop_accel / fmax(fabs(y), 1.0f64)),
    );
    let mut desired_rate: ::core::ffi::c_double = -y * rate_gain;
    let mut capture_rate: ::core::ffi::c_double = desired_rate;
    let mut pavement_intercept: bool = terminal_pavement_intercept_rate(
        t,
        cfg,
        &raw mut desired_rate,
    );
    if pavement_intercept as ::core::ffi::c_int != 0
        && capture_rate * desired_rate > 0.0f64
        && fabs(capture_rate) > fabs(desired_rate)
    {
        desired_rate = capture_rate;
    }
    let mut present_accel: ::core::ffi::c_double = lift_accel
        * sin(clampd(roll_now, -60.0f64, 60.0f64) * DEG2RAD);
    let mut lateral_limit: ::core::ffi::c_double = lift_accel
        * sin(bank_limit * DEG2RAD);
    let mut lateral_accel: ::core::ffi::c_double = 0.0f64;
    let mut it: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while it < 3 as ::core::ffi::c_int {
        let mut bank_goal: ::core::ffi::c_double = asin(
            clampd(lateral_accel / lift_accel, -1.0f64, 1.0f64),
        ) * RAD2DEG;
        let mut swing: ::core::ffi::c_double = decision_axis_capture_time(
            bank_goal - roll_now,
            roll_rate_now,
            roll_limits.accel_deg_s2,
            roll_limits.rate_deg_s,
        );
        if swing.is_finite() as i32 == 0 {
            swing = reversal_time;
        }
        if swing.is_finite() as i32 == 0 {
            swing = 10.0f64;
        }
        lateral_accel = clampd(
            (desired_rate - cross_rate - 0.5f64 * present_accel * swing)
                / (0.5f64 * swing + RATE_CLOSURE_TAU),
            -lateral_limit,
            lateral_limit,
        );
        it += 1;
    }
    let mut bank: ::core::ffi::c_double = clampd(
        asin(clampd(lateral_accel / lift_accel, -1.0f64, 1.0f64)) * RAD2DEG,
        -bank_limit,
        bank_limit,
    );
    let mut lift_div: ::core::ffi::c_double = cos(bank * DEG2RAD);
    let mut runway_length: ::core::ffi::c_double = if (*site).runway_length > 100.0f64 {
        (*site).runway_length
    } else {
        2500.0f64
    };
    let mut touchdown_target: ::core::ffi::c_double = clampd(
        if (*s).touchdown_aim_along > 0.0f64 {
            (*s).touchdown_aim_along
        } else {
            0.15f64 * runway_length
        },
        TOUCHDOWN_ALONG,
        600.0f64,
    );
    let mut ld_float: ::core::ffi::c_double = 0.8f64
        * terminal_learned_max_lift_to_drag(g, t, v, aoa_max);
    let mut planned_preflare_trigger: ::core::ffi::c_double = if (*g)
        .terminal_preflare_plan_valid as ::core::ffi::c_int != 0
        && (*g).preflare_trigger_altitude.is_finite() as i32 != 0
        && (*g).preflare_trigger_altitude > 0.0f64
    {
        (*g).preflare_trigger_altitude
    } else {
        0.0f64
    };
    let mut flare_plan: TerminalFlarePlan = terminal_flare_plan(
        g,
        t,
        p,
        cfg,
        planned_preflare_trigger,
    );
    let mut arrest_accel: ::core::ffi::c_double = flare_plan.arrest_accel;
    let mut lift_q_max: ::core::ffi::c_double = 0.0f64;
    let mut drag_q_max: ::core::ffi::c_double = 0.0f64;
    terminal_aero_q(g, t, v, aoa_max, &raw mut lift_q_max, &raw mut drag_q_max);
    let mut rho: ::core::ffi::c_double = if (*t).atmospheric_density.is_finite() as i32
        != 0 && (*t).atmospheric_density > 0.0f64
    {
        (*t).atmospheric_density
    } else {
        2.0f64 * fmax((*t).dynamic_pressure, 1.0f64) / (V * V)
    };
    let mut speed_1g: ::core::ffi::c_double = sqrt(
        2.0f64 * (*t).mass * g0
            / fmax(1e-6f64, rho * lift_q_max * (*g).terminal_lift_ratio),
    );
    let mut sink_now: ::core::ffi::c_double = fmax(0.0f64, -(*t).vertical_speed);
    let mut pull_height: ::core::ffi::c_double = fmax(
        0.0f64,
        sink_now * sink_now - SGS_SINK * SGS_SINK,
    ) / (2.0f64 * arrest_accel);
    let mut energy_height: ::core::ffi::c_double = fmax(0.0f64, V * V - v_td * v_td)
        / (2.0f64 * g0);
    let mut handoff_slope: ::core::ffi::c_double = terminal_outer_glide_slope(g, s);
    let mut glide_tan: ::core::ffi::c_double = fmax(
        tan(4.0f64 * DEG2RAD),
        fabs(
            tan(
                if (*g).terminal_aim_valid as ::core::ffi::c_int != 0 {
                    terminal_geometry_fpa(
                        height,
                        fmax(120.0f64, (*g).terminal_aim_along - along),
                        0.0f64,
                    )
                } else if gamma.is_finite() as i32 != 0 && fabs(gamma) > 1e-3f64 {
                    gamma
                } else {
                    -handoff_slope * DEG2RAD
                },
            ),
        ),
    );
    let mut arc_distance: ::core::ffi::c_double = Vh * sink_now / arrest_accel * 0.6f64;
    let mut float_distance: ::core::ffi::c_double = ld_float
        * (energy_height + SGS_HEIGHT);
    let mut preflare_height: ::core::ffi::c_double = pull_height + SGS_HEIGHT;
    let AIM_BIAS: ::core::ffi::c_double = 150.0f64;
    let mut aim: ::core::ffi::c_double = touchdown_target + AIM_BIAS - arc_distance
        - float_distance + preflare_height / glide_tan;
    aim = clampd(aim, touchdown_target - 6000.0f64, touchdown_target + AIM_BIAS);
    let mut glide_owns: bool = !(*g).terminal_pull_latch || !(*g).terminal_arc_started;
    if (*s).taem_hac_law as ::core::ffi::c_int != 0 && !(*s).adaptive_flare
        && glide_owns as ::core::ffi::c_int != 0
        && (*g).terminal_aim_valid as ::core::ffi::c_int != 0
        && (*g).terminal_predicted_touchdown_along.is_finite() as i32 != 0
    {
        let mut err: ::core::ffi::c_double = clampd(
            touchdown_target - (*g).terminal_predicted_touchdown_along,
            -300.0f64,
            300.0f64,
        );
        (*g).terminal_aim_shift = clampd(
            (*g).terminal_aim_shift + clampd(dt, 0.0f64, 0.5f64) * err / 6.0f64,
            -1500.0f64,
            1500.0f64,
        );
        aim = clampd(
            aim + (*g).terminal_aim_shift,
            touchdown_target - 6000.0f64,
            touchdown_target + AIM_BIAS + fmax(0.0f64, (*g).terminal_aim_shift),
        );
    }
    if (*s).adaptive_flare as ::core::ffi::c_int != 0 && !(*g).terminal_pull_latch {
        let mut F: *mut AdaptiveFlareState = &raw mut (*g).adaptive_flare;
        if (*F).plan_valid as ::core::ffi::c_int != 0
            && (*F).pred_along.is_finite() as i32 != 0 && (*F).cost < 1e5f64
        {
            let mut err_0: ::core::ffi::c_double = clampd(
                touchdown_target - (*F).pred_along,
                -300.0f64,
                300.0f64,
            );
            (*F).aim_shift = clampd(
                (*F).aim_shift + clampd(dt, 0.0f64, 0.5f64) * err_0 / 6.0f64,
                -1500.0f64,
                1500.0f64,
            );
        }
        aim = clampd(
            aim + (*F).aim_shift,
            touchdown_target - 6000.0f64,
            touchdown_target + AIM_BIAS + fmax(0.0f64, (*F).aim_shift),
        );
    }
    if glide_owns {
        if !(*g).terminal_aim_valid {
            let mut seed_distance: ::core::ffi::c_double = height
                / fmax(tan(4.0f64 * DEG2RAD), tan(handoff_slope * DEG2RAD));
            let mut seed_aim: ::core::ffi::c_double = clampd(
                along + seed_distance,
                touchdown_target - 6000.0f64,
                touchdown_target,
            );
            if (*g).terminal_final_low_energy {
                seed_aim = aim;
            }
            (*g).terminal_aim_along = seed_aim;
            (*g).terminal_aim_valid = true_0 != 0;
        } else {
            let mut tau: ::core::ffi::c_double = 4.0f64;
            let mut a: ::core::ffi::c_double = clampd(dt / (tau + dt), 0.0f64, 1.0f64);
            (*g).terminal_aim_along += a * (aim - (*g).terminal_aim_along);
        }
        (*g).terminal_aim_ut = (*t).ut;
    }
    let mut aim_distance: ::core::ffi::c_double = fmax(
        120.0f64,
        (*g).terminal_aim_along - along,
    );
    let mut glide_fpa: ::core::ffi::c_double = terminal_geometry_fpa(
        height,
        aim_distance,
        0.0f64,
    );
    let mut aoa_pull: ::core::ffi::c_double = aoa_max;
    let mut La: ::core::ffi::c_double = 0.0f64;
    let mut Da: ::core::ffi::c_double = 0.0f64;
    let mut Lb: ::core::ffi::c_double = 0.0f64;
    let mut Db: ::core::ffi::c_double = 0.0f64;
    let mut a0: ::core::ffi::c_double = clampd((*t).angle_of_attack, 0.0f64, aoa_max);
    terminal_aero_q(g, t, v, fmax(0.0f64, a0 - 1.0f64), &raw mut La, &raw mut Da);
    terminal_aero_q(
        g,
        t,
        v,
        fmin(aoa_max + 1.0f64, a0 + 1.0f64),
        &raw mut Lb,
        &raw mut Db,
    );
    let mut slope: ::core::ffi::c_double = (Lb - La)
        / (fmin(aoa_max + 1.0f64, a0 + 1.0f64) - fmax(0.0f64, a0 - 1.0f64));
    let mut needed: ::core::ffi::c_double = (*t).mass * (g0 * cos(gamma) + arrest_accel)
        / fmax(0.3f64, lift_div);
    if slope > 1e-3f64 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).dynamic_pressure > 100.0f64
    {
        aoa_pull = clampd(
            a0 + (needed - (*t).lift_force) / ((*t).dynamic_pressure * slope),
            0.0f64,
            aoa_max,
        );
    }
    let FLARE_SLEW: ::core::ffi::c_double = 5.0f64;
    let mut response: ::core::ffi::c_double = clampd(
        1.5f64 + fmax(0.0f64, aoa_pull - (*t).angle_of_attack) / FLARE_SLEW,
        1.5f64,
        8.0f64,
    );
    if (*g).terminal_preflare_plan_valid as ::core::ffi::c_int != 0
        && (*g).preflare_response_time.is_finite() as i32 != 0
        && (*g).preflare_response_time > 0.0f64
    {
        response = fmax(response, (*g).preflare_response_time);
    }
    let mut response_height: ::core::ffi::c_double = sink_now * response;
    let mut preflare_latch_height: ::core::ffi::c_double = fmax(
        preflare_height + response_height,
        planned_preflare_trigger,
    );
    if !(*s).adaptive_flare && height <= preflare_latch_height {
        (*g).terminal_pull_latch = true_0 != 0;
    }
    let mut station_speed: ::core::ffi::c_double = if (*g).terminal_final_low_energy
        as ::core::ffi::c_int != 0
    {
        0.0f64
    } else {
        terminal_final_speed_target(g, v, s, fmax(0.0f64, -along))
    };
    let mut speed_ref: ::core::ffi::c_double = fmax(
        station_speed,
        1.6f64 * fmax(speed_1g, v_td),
    );
    let mut predicted_touchdown_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut predicted_ground_range: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    if glide_owns {
        predicted_touchdown_speed = terminal_predict_touchdown_speed(
            g,
            t,
            v,
            height,
            V,
            sink_now,
            false_0 != 0,
            &raw mut flare_plan,
            &raw mut predicted_ground_range,
        );
        speed_ref = fmax(
            station_speed,
            fmax(
                1.5f64 * fmax(speed_1g, v_td),
                sqrt(
                    fmax(
                        0.0f64,
                        V * V + v_td * v_td
                            - predicted_touchdown_speed * predicted_touchdown_speed,
                    ),
                ),
            ),
        );
    }
    (*g).terminal_predicted_touchdown_along = if predicted_ground_range.is_finite()
        as i32 != 0
    {
        along + predicted_ground_range
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut range_short: bool = ((*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
        || (*s).taem_hac_law as ::core::ffi::c_int != 0)
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 && predicted_ground_range.is_finite() as i32 != 0
        && predicted_ground_range < -along + touchdown_target;
    if range_short {
        speed_ref = fmax(
            1.5f64 * fmax(speed_1g, v_td),
            sqrt(
                fmax(
                    0.0f64,
                    V * V + v_td * v_td
                        - predicted_touchdown_speed * predicted_touchdown_speed,
                ),
            ),
        );
    }
    (*g).terminal_predicted_touchdown_speed = predicted_touchdown_speed;
    let mut speedbrake_target: ::core::ffi::c_double = 0.0f64;
    if (*s).terminal_speedbrake as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure > 0.0f64
    {
        let mut extra_drag: ::core::ffi::c_double = 0.0f64;
        if glide_owns {
            if speed_ref.is_finite() as i32 != 0 && V > speed_ref {
                extra_drag = (*t).mass * (V * V - speed_ref * speed_ref)
                    / (2.0f64 * aim_distance);
            }
        } else {
            let mut to_target: ::core::ffi::c_double = touchdown_target
                + 0.35f64 * runway_length - along;
            let mut fin_drag: ::core::ffi::c_double = (*g).terminal_split_fin_cda
                * (*g).terminal_speedbrake_fraction * (*t).dynamic_pressure;
            let mut clean_drag: ::core::ffi::c_double = fmax(
                0.0f64,
                (*t).drag_force - fin_drag,
            );
            let mut excess_energy: ::core::ffi::c_double = V * V - v_td * v_td
                + 2.0f64 * g0 * fmax(0.0f64, height);
            extra_drag = if to_target > 0.0f64 {
                (*t).mass * excess_energy / (2.0f64 * fmax(120.0f64, to_target))
                    - clean_drag
            } else {
                ::core::f32::INFINITY as ::core::ffi::c_double
            };
        }
        if extra_drag > 0.0f64 {
            speedbrake_target = if (*g).terminal_split_fin_cda > 0.0f64 {
                extra_drag / ((*t).dynamic_pressure * (*g).terminal_split_fin_cda)
            } else {
                SPEEDBRAKE_NOMINAL
            };
        }
        speedbrake_target = clampd(speedbrake_target, 0.0f64, SPEEDBRAKE_CAP);
    }
    let mut step: ::core::ffi::c_double = clampd(dt, 0.0f64, 1.0f64);
    (*g).terminal_speedbrake_fraction = if (*s).terminal_speedbrake as ::core::ffi::c_int
        != 0
    {
        clampd(
            speedbrake_target,
            (*g).terminal_speedbrake_fraction - SPEEDBRAKE_STOW_SLEW * step,
            (*g).terminal_speedbrake_fraction + SPEEDBRAKE_DEPLOY_SLEW * step,
        )
    } else {
        0.0f64
    };
    let mut glide_speed_error: ::core::ffi::c_double = V - speed_ref;
    if (*s).terminal_speedbrake as ::core::ffi::c_int != 0 && glide_speed_error > 0.0f64
        && (*g).terminal_speedbrake_fraction < SPEEDBRAKE_CAP
    {
        glide_speed_error = 0.0f64;
    }
    if glide_owns as ::core::ffi::c_int != 0 && speed_ref.is_finite() as i32 != 0 {
        glide_fpa = clampd(
            glide_fpa + clampd(0.8f64 * glide_speed_error, -12.0f64, 6.0f64) * DEG2RAD,
            -35.0f64 * DEG2RAD,
            -3.0f64 * DEG2RAD,
        );
    }
    if ((*g).terminal_final_low_energy as ::core::ffi::c_int != 0
        || range_short as ::core::ffi::c_int != 0)
        && glide_owns as ::core::ffi::c_int != 0
    {
        let mut best_glide: ::core::ffi::c_double = -atan(
            1.0f64 / terminal_learned_max_lift_to_drag(g, t, v, aoa_max),
        );
        let mut floor_0: ::core::ffi::c_double = if (*g).terminal_final_low_energy
            as ::core::ffi::c_int != 0
        {
            v_td
        } else {
            fmax(v_td, (*v).final_approach_speed)
        };
        let mut stretch: ::core::ffi::c_double = clampd(
            (V - floor_0) / (0.15f64 * v_td),
            0.0f64,
            1.0f64,
        );
        glide_fpa += stretch * (fmax(glide_fpa, best_glide) - glide_fpa);
    }
    let mut long_landing: bool = along > touchdown_target + 0.35f64 * runway_length;
    let mut float_height: ::core::ffi::c_double = if long_landing as ::core::ffi::c_int
        != 0
    {
        0.0f64
    } else {
        H_FLOAT
    };
    let mut float_sink: ::core::ffi::c_double = if long_landing as ::core::ffi::c_int
        != 0
    {
        1.8f64
    } else {
        S_FLOAT
    };
    let mut flaring: bool = (*g).terminal_pull_latch;
    let mut sink: ::core::ffi::c_double = -(*t).vertical_speed;
    let mut reference_fpa: ::core::ffi::c_double = glide_fpa;
    let mut sink_rate_ff: ::core::ffi::c_double = 0.0f64;
    let mut arc_accel: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut settle_sink: ::core::ffi::c_double = float_sink;
    if flaring as ::core::ffi::c_int != 0 && height > SGS_HEIGHT {
        let PREFLARE_TAU: ::core::ffi::c_double = 1.5f64;
        let ARC_RESIDUAL_LAG: ::core::ffi::c_double = 1.0f64;
        let mut arc_lag: ::core::ffi::c_double = fmin(
            response,
            fmax(ARC_RESIDUAL_LAG, fabs(aoa_pull - (*t).angle_of_attack) / FLARE_SLEW),
        );
        let mut room: ::core::ffi::c_double = fmax(
            0.0f64,
            height - SGS_HEIGHT - fmax(0.0f64, sink) * arc_lag,
        );
        let mut design_sink: ::core::ffi::c_double = sqrt(
            SGS_SINK * SGS_SINK + 2.0f64 * arrest_accel * room,
        );
        let mut glide_sink: ::core::ffi::c_double = fmax(SGS_SINK, -V * sin(glide_fpa));
        let mut onset_room: ::core::ffi::c_double = fmax(
            0.0f64,
            height - SGS_HEIGHT - fmax(0.0f64, sink) * (response + 1.0f64),
        );
        let mut onset_sink: ::core::ffi::c_double = sqrt(
            SGS_SINK * SGS_SINK + 2.0f64 * arrest_accel * onset_room,
        );
        let mut sts_gentle: bool = (*s).sts_trajectory_delivery as ::core::ffi::c_int
            != 0
            && strcmp(
                &raw const (*v).model_id as *const ::core::ffi::c_char,
                b"STS\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0;
        if onset_sink < sink || sink <= SGS_SINK {
            (*g).terminal_arc_started = true_0 != 0;
        }
        let mut on_arc: bool = (*g).terminal_arc_started as ::core::ffi::c_int != 0
            || sts_gentle as ::core::ffi::c_int != 0 && sink < 0.92f64 * glide_sink;
        if sink > SGS_SINK && !on_arc && sts_gentle as ::core::ffi::c_int != 0 {
            arc_accel = (sink * sink - SGS_SINK * SGS_SINK)
                / (2.0f64 * fmax(room, 1.0f64));
            let mut arc_goal: ::core::ffi::c_double = fmax(
                SGS_SINK,
                sink - arc_accel * PREFLARE_TAU,
            );
            reference_fpa = fmax(
                glide_fpa,
                -asin(clampd(arc_goal / V, 0.0f64, 0.95f64)),
            );
        } else if sink > SGS_SINK && !on_arc {
            arc_accel = ::core::f32::NAN as ::core::ffi::c_double;
            reference_fpa = glide_fpa;
        } else {
            let mut arc_sink: ::core::ffi::c_double = sink;
            if (*t).lift_force.is_finite() as i32 != 0 {
                arc_sink = fmax(
                    0.0f64,
                    sink
                        - ARC_RESIDUAL_LAG
                            * fmax(
                                0.0f64,
                                (*t).lift_force * cos(bank * DEG2RAD)
                                    / fmax((*t).mass, 1.0f64) - g0 * cos(gamma),
                            ),
                );
            }
            arc_accel = if sink > SGS_SINK {
                fmax(0.0f64, arrest_accel + (arc_sink - design_sink) / PREFLARE_TAU)
            } else {
                (sink - SGS_SINK) / PREFLARE_TAU
            };
            let mut arc_goal_0: ::core::ffi::c_double = if sink > SGS_SINK {
                fmax(SGS_SINK, sink - arc_accel * PREFLARE_TAU)
            } else {
                SGS_SINK
            };
            reference_fpa = fmax(
                glide_fpa,
                -asin(clampd(arc_goal_0 / V, 0.0f64, 0.95f64)),
            );
        }
    } else if flaring {
        (*g).terminal_arc_started = true_0 != 0;
        let NO_PUSH_HEIGHT: ::core::ffi::c_double = 3.0f64 * H_FLOAT;
        let FLARE_TAU: ::core::ffi::c_double = 1.0f64;
        let FLARE_LEAD: ::core::ffi::c_double = 1.0f64;
        let mut lead_height: ::core::ffi::c_double = fmax(
            0.0f64,
            height - fmax(0.0f64, sink) * FLARE_LEAD,
        );
        let mut profile_sink: ::core::ffi::c_double = terminal_sink_profile(
            lead_height,
            float_height,
            float_sink,
            arrest_accel,
            SGS_HEIGHT,
            SGS_SINK,
            FINAL_FLARE_ACCEL,
        );
        let mut reference_sink: ::core::ffi::c_double = if height > NO_PUSH_HEIGHT {
            profile_sink
        } else {
            fmin(profile_sink, fmax(float_sink, sink))
        };
        if (*s).taem_energy_law as ::core::ffi::c_int != 0
            && (*s).terminal_speedbrake as ::core::ffi::c_int != 0 && along < 0.0f64
            && speed_1g.is_finite() as i32 != 0 && V > speed_1g
        {
            let mut reserve: ::core::ffi::c_double = ld_float
                * (fmax(0.0f64, height)
                    + fmax(0.0f64, V * V - speed_1g * speed_1g) / (2.0f64 * g0));
            if reserve > -along {
                reference_sink = fmin(
                    reference_sink,
                    fmax(0.0f64, lead_height) * Vh / -along,
                );
            }
        }
        let SETTLE_SINK_MAX: ::core::ffi::c_double = 2.0f64;
        let mut decel: ::core::ffi::c_double = if (*t).drag_force.is_finite() as i32 != 0
            && (*t).mass > 1.0f64
        {
            (*t).drag_force / (*t).mass
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
        let mut settling: bool = false_0 != 0;
        if decel.is_finite() as i32 != 0 && decel > 0.1f64
            && speed_1g.is_finite() as i32 != 0 && height > 0.0f64
        {
            let mut float_time: ::core::ffi::c_double = (V - 1.05f64 * speed_1g) / decel;
            let mut settle: ::core::ffi::c_double = if float_time > 0.0f64 {
                height / float_time
            } else {
                ::core::f32::INFINITY as ::core::ffi::c_double
            };
            if fmin(settle, SETTLE_SINK_MAX) > reference_sink {
                reference_sink = fmin(settle, SETTLE_SINK_MAX);
                settling = true_0 != 0;
            }
        }
        settle_sink = reference_sink;
        let mut flare_top: ::core::ffi::c_double = terminal_final_flare_height(
            float_height,
            float_sink,
            FINAL_FLARE_ACCEL,
            SGS_SINK,
        );
        let mut ff: ::core::ffi::c_double = if lead_height > float_height
            && lead_height <= flare_top && sink > float_sink
        {
            FINAL_FLARE_ACCEL
        } else {
            0.0f64
        };
        let FLARE_RESPONSE_LEAD: ::core::ffi::c_double = 0.5f64;
        let mut lead_sink: ::core::ffi::c_double = sink;
        if (*t).lift_force.is_finite() as i32 != 0 && (*t).mass > 1.0f64 {
            lead_sink = sink
                + FLARE_RESPONSE_LEAD
                    * (g0 * cos(gamma)
                        - (*t).lift_force * cos(bank * DEG2RAD) / (*t).mass);
        }
        arc_accel = ff + (lead_sink - reference_sink) / FLARE_TAU;
        if settling {
            arc_accel = fmax(arc_accel, -0.1f64 * g0);
        }
        reference_fpa = fmax(glide_fpa, -asin(fmin(0.95f64, reference_sink / V)));
    }
    let mut reference_sink_0: ::core::ffi::c_double = V * sin(reference_fpa);
    let mut measured_gamma_rate: ::core::ffi::c_double = if (*t).lift_force.is_finite()
        as i32 != 0
    {
        ((*t).lift_force * cos(bank * DEG2RAD) / fmax((*t).mass, 1.0f64)
            - g0 * cos(gamma)) / V
    } else {
        0.0f64
    };
    let mut lead_gamma: ::core::ffi::c_double = gamma + 1.5f64 * measured_gamma_rate;
    let mut vertical_accel: ::core::ffi::c_double = if arc_accel.is_finite() as i32 != 0
    {
        clampd(arc_accel, -0.3f64 * g0, 1.5f64 * g0)
    } else {
        clampd(
            V * (sink_rate_ff + 0.5f64 * (reference_fpa - lead_gamma)),
            -0.3f64 * g0,
            1.5f64 * g0,
        )
    };
    let mut lift_required: ::core::ffi::c_double = (*t).mass
        * (vertical_accel + g0 * cos(gamma)) / lift_div / (*g).terminal_lift_ratio;
    let mut aoa: ::core::ffi::c_double = clampd(
        terminal_aoa_for_lift(g, t, v, fmax(0.0f64, lift_required), aoa_max),
        0.0f64,
        aoa_max,
    );
    if flaring as ::core::ffi::c_int != 0 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).dynamic_pressure > 100.0f64
    {
        let mut q: ::core::ffi::c_double = (*t).dynamic_pressure;
        let mut La_0: ::core::ffi::c_double = 0.0f64;
        let mut Da_0: ::core::ffi::c_double = 0.0f64;
        let mut Lb_0: ::core::ffi::c_double = 0.0f64;
        let mut Db_0: ::core::ffi::c_double = 0.0f64;
        let mut a0_0: ::core::ffi::c_double = clampd(
            (*t).angle_of_attack,
            0.0f64,
            aoa_max,
        );
        terminal_aero_q(
            g,
            t,
            v,
            fmax(0.0f64, a0_0 - 1.0f64),
            &raw mut La_0,
            &raw mut Da_0,
        );
        terminal_aero_q(
            g,
            t,
            v,
            fmin(aoa_max + 1.0f64, a0_0 + 1.0f64),
            &raw mut Lb_0,
            &raw mut Db_0,
        );
        let mut slope_0: ::core::ffi::c_double = (Lb_0 - La_0)
            / (fmin(aoa_max + 1.0f64, a0_0 + 1.0f64) - fmax(0.0f64, a0_0 - 1.0f64));
        if slope_0 > 1e-3f64 {
            let mut needed_0: ::core::ffi::c_double = (*t).mass
                * (vertical_accel + g0 * cos(gamma)) / lift_div;
            aoa = clampd(
                a0_0 + (needed_0 - (*t).lift_force) / (q * slope_0),
                0.0f64,
                aoa_max,
            );
        }
    }
    let mut af_flying: bool = false_0 != 0;
    if (*s).adaptive_flare as ::core::ffi::c_int != 0 && height < 1500.0f64
        && V > 20.0f64 && gamma.is_finite() as i32 != 0
    {
        let mut F_0: *mut AdaptiveFlareState = &raw mut (*g).adaptive_flare;
        let mut M: AfModel = AfModel {
            lq: [0.; 121],
            dq: [0.; 121],
            n: 0,
            aoa_max: 0.,
            mass: 0.,
            g0: 0.,
            rho: 0.,
            sound: 0.,
            tau: 0.,
            bias: 0.,
            gain: 0.,
            v_floor: 0.,
            zone_lo: 0.,
            zone_hi: 0.,
        };
        af_model(
            g,
            t,
            v,
            g0,
            aoa_max,
            v_td,
            100.0f64,
            touchdown_target + 500.0f64,
            &raw mut M,
        );
        let mut s0: AfState = AfState {
            h: height,
            sink: sink,
            V: V,
            gamma: gamma,
            aoa: (*t).angle_of_attack,
            cmd: if (*g).terminal_aoa_cmd_ut > 0.0f64 {
                (*g).terminal_aoa_cmd_last
            } else {
                (*t).angle_of_attack
            },
            x: along,
            engaged: (*g).terminal_pull_latch,
            glide_sink: if (*F_0).active as ::core::ffi::c_int != 0 {
                (*F_0).glide_sink
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
            aoa_rate: if (*t).has_angle_of_attack_rate as ::core::ffi::c_int != 0 {
                (*t).angle_of_attack_rate
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            },
        };
        if !(*F_0).plan_valid || (*t).ut - (*F_0).plan_ut >= 0.2f64 {
            af_solve(F_0, &raw mut M, &raw mut s0, (*t).ut);
        }
        if (*F_0).plan_valid {
            let mut P: AfParams = AfParams {
                arc_accel: (*F_0).arc_accel,
                sgs_height: (*F_0).sgs_height,
            };
            let mut o: AfLawOut = af_law(&raw mut M, &raw mut P, &raw mut s0, 0.0f64);
            if !(*g).terminal_pull_latch && o.on_arc as ::core::ffi::c_int != 0 {
                (*g).terminal_pull_latch = true_0 != 0;
                (*F_0).active = true_0 != 0;
                (*F_0).glide_sink = sink;
            }
            if (*g).terminal_pull_latch {
                aoa = o.cmd;
                af_flying = true_0 != 0;
                flaring = true_0 != 0;
            }
            if !getenv(
                    b"KSP_LANDER_AFLARE_TRACE\0" as *const u8
                        as *const ::core::ffi::c_char,
                )
                .is_null()
            {
                fprintf(
                    stderr,
                    b"AFLARE ut=%.2f h=%.1f x=%.0f V=%.1f sink=%.2f aoa=%.2f cmd=%.2f acc=%.2f on=%d fly=%d tau=%.2f bias=%.2f gain=%.2f shift=%.0f w=%.1f arc=%.2f sgs=%.0f cost=%.2f pV=%.1f pS=%.2f pX=%.0f pN=%.2f pC=%.2f\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*t).ut,
                    height,
                    along,
                    V,
                    sink,
                    (*t).angle_of_attack,
                    o.cmd,
                    o.accel,
                    o.on_arc as ::core::ffi::c_int,
                    af_flying as ::core::ffi::c_int,
                    (*F_0).tau,
                    (*F_0).bias,
                    (*F_0).gain,
                    (*F_0).aim_shift,
                    (*F_0).weight,
                    (*F_0).arc_accel / g0,
                    (*F_0).sgs_height,
                    (*F_0).cost,
                    (*F_0).pred_speed,
                    (*F_0).pred_sink,
                    (*F_0).pred_along,
                    (*F_0).pred_peak_n,
                    (*F_0).pred_climb,
                );
            }
        }
    }
    let mut touchdown_attitude_floor: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut touchdown_aoa_floor: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    if !af_flying && flaring as ::core::ffi::c_int != 0
        && height <= TOUCHDOWN_ATTITUDE_START
    {
        let mut u: ::core::ffi::c_double = clampd(
            (TOUCHDOWN_ATTITUDE_START - height)
                / (TOUCHDOWN_ATTITUDE_START - TOUCHDOWN_ATTITUDE_FULL),
            0.0f64,
            1.0f64,
        );
        u = u * u * (3.0f64 - 2.0f64 * u);
        touchdown_attitude_floor = TOUCHDOWN_PITCH * u;
        touchdown_aoa_floor = clampd(
            touchdown_attitude_floor - (*t).flight_path_angle,
            0.0f64,
            aoa_max,
        );
        let mut floor_lift_cap: ::core::ffi::c_double = terminal_aoa_for_lift(
            g,
            t,
            v,
            1.0f64 * (*t).mass * g0 / fmax(0.3f64, lift_div)
                / fmax(0.3f64, (*g).terminal_lift_ratio),
            aoa_max,
        );
        touchdown_aoa_floor = fmin(touchdown_aoa_floor, fmax(aoa, floor_lift_cap));
        if sink < settle_sink && (*t).lift_force.is_finite() as i32 != 0
            && (*t).dynamic_pressure > 100.0f64
        {
            let mut La_1: ::core::ffi::c_double = 0.0f64;
            let mut Da_1: ::core::ffi::c_double = 0.0f64;
            let mut Lb_1: ::core::ffi::c_double = 0.0f64;
            let mut Db_1: ::core::ffi::c_double = 0.0f64;
            let mut a0_1: ::core::ffi::c_double = clampd(
                (*t).angle_of_attack,
                0.0f64,
                aoa_max,
            );
            terminal_aero_q(
                g,
                t,
                v,
                fmax(0.0f64, a0_1 - 1.0f64),
                &raw mut La_1,
                &raw mut Da_1,
            );
            terminal_aero_q(
                g,
                t,
                v,
                fmin(aoa_max + 1.0f64, a0_1 + 1.0f64),
                &raw mut Lb_1,
                &raw mut Db_1,
            );
            let mut slope_1: ::core::ffi::c_double = (Lb_1 - La_1)
                / (fmin(aoa_max + 1.0f64, a0_1 + 1.0f64) - fmax(0.0f64, a0_1 - 1.0f64));
            if slope_1 > 1e-3f64 {
                let mut settle_0: ::core::ffi::c_double = (*t).mass * FLOOR_SETTLE_G * g0
                    * cos(gamma) / fmax(0.3f64, lift_div);
                let mut measured_cap: ::core::ffi::c_double = a0_1
                    + (settle_0 - (*t).lift_force) / ((*t).dynamic_pressure * slope_1);
                touchdown_aoa_floor = fmin(touchdown_aoa_floor, fmax(aoa, measured_cap));
            }
        }
        aoa = fmax(aoa, touchdown_aoa_floor);
    }
    if stage as ::core::ffi::c_uint
        == TERMINAL_TOUCHDOWN_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
        || flaring as ::core::ffi::c_int != 0 && height <= 15.0f64
    {
        stage = TERMINAL_TOUCHDOWN_FLARE;
    } else {
        stage = (if flaring as ::core::ffi::c_int != 0 {
            TERMINAL_PREFLARE as ::core::ffi::c_int
        } else {
            TERMINAL_OUTER_FINAL as ::core::ffi::c_int
        }) as TerminalVerticalStage;
    }
    if (*g).terminal_vertical_stage as ::core::ffi::c_uint
        != stage as ::core::ffi::c_uint
    {
        terminal_set_stage(g, stage, (*t).ut);
    }
    if (*g).terminal_vertical_stage as ::core::ffi::c_uint
        == TERMINAL_TOUCHDOWN_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        (*g).flare_sink_captured = true_0 != 0;
    }
    flaring = (*g).terminal_vertical_stage as ::core::ffi::c_uint
        != TERMINAL_TRAJECTORY_CAPTURE as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*g).terminal_vertical_stage as ::core::ffi::c_uint
            != TERMINAL_OUTER_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut aoa_command_elapsed: ::core::ffi::c_double = (*t).ut
        - (*g).terminal_aoa_cmd_ut;
    if (*g).terminal_aoa_cmd_ut > 0.0f64 && aoa_command_elapsed.is_finite() as i32 != 0
        && aoa_command_elapsed >= 0.0f64 && aoa_command_elapsed < 2.0f64
    {
        let mut rate_0: ::core::ffi::c_double = if flaring as ::core::ffi::c_int != 0 {
            FLARE_SLEW
        } else {
            6.0f64
        };
        let mut allowance: ::core::ffi::c_double = rate_0 * aoa_command_elapsed;
        aoa = clampd(
            aoa,
            (*g).terminal_aoa_cmd_last - allowance,
            (*g).terminal_aoa_cmd_last + allowance,
        );
    } else {
        aoa = (*t).angle_of_attack;
    }
    (*g).terminal_aoa_cmd_last = aoa;
    (*g).terminal_aoa_cmd_ut = (*t).ut;
    let mut intercept: ::core::ffi::c_double = clampd(
        atan2(-(*t).runway_cross_track, fmax(400.0f64, V * 6.0f64)) * RAD2DEG,
        -20.0f64,
        20.0f64,
    );
    if pavement_intercept {
        intercept = asin(clampd(desired_rate / Vh, -1.0f64, 1.0f64)) * RAD2DEG;
    }
    let mut target_course: ::core::ffi::c_double = norm_deg(
        (*site).runway_heading + intercept,
    );
    if height < 90.0f64 {
        (*g).gear_command_latched = true_0 != 0;
    }
    terminal_speedbrake_closed(g);
    let mut c: GuidanceCommand = atmospheric(
        t,
        target_course,
        bank,
        v,
        0.0f64,
        (*g).airbrakes_deployed,
        (if flaring as ::core::ffi::c_int != 0 {
            PROFILE_FLARE as ::core::ffi::c_int
        } else {
            PROFILE_APPROACH as ::core::ffi::c_int
        }) as ControlProfile,
    );
    c.speedbrake_fraction = (*g).terminal_speedbrake_fraction;
    c.terminal_roll_trim_enabled = (*s).sts_trajectory_delivery as ::core::ffi::c_int
        != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    c.has_target_aoa = true_0 != 0;
    c.target_aoa = aoa;
    c.target_pitch = (*t).flight_path_angle + aoa;
    c.gear = (*t).gear as ::core::ffi::c_int != 0
        || (*g).gear_command_latched as ::core::ffi::c_int != 0;
    if flaring as ::core::ffi::c_int != 0 && height <= 15.0f64 {
        c.heading_control_enabled = true_0 != 0;
        c.target_heading = (*site).runway_heading;
    }
    let mut phase: GuidancePhase = (if (*g).terminal_vertical_stage
        as ::core::ffi::c_uint
        == TERMINAL_TOUCHDOWN_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        PHASE_FLARE as ::core::ffi::c_int
    } else {
        PHASE_FINAL as ::core::ffi::c_int
    }) as GuidancePhase;
    (*g).phase = phase;
    let mut status: [::core::ffi::c_char; 256] = [0; 256];
    snprintf(
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"%s: wheel height %.0f m, along %.0f m, cross %+.0f m, speed %.0f / %.0f m/s, sink %.1f / %.1f m/s, AoA %.1f deg, aim %.0f m.\0"
            as *const u8 as *const ::core::ffi::c_char,
        if phase as ::core::ffi::c_uint
            == PHASE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            b"Touchdown flare\0" as *const u8 as *const ::core::ffi::c_char
        } else if flaring as ::core::ffi::c_int != 0 {
            b"Preflare\0" as *const u8 as *const ::core::ffi::c_char
        } else if range_short as ::core::ffi::c_int != 0 {
            b"Range-stretch glide\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"Final glide\0" as *const u8 as *const ::core::ffi::c_char
        },
        height,
        along,
        (*t).runway_cross_track,
        V,
        speed_ref,
        (*t).vertical_speed,
        reference_sink_0,
        aoa,
        (*g).terminal_aim_along,
    );
    let mut warn: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    if long_landing {
        warn = b"Long landing: floating would leave too little runway; landing firmly now.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if V < v_td * 0.9f64 {
        warn = b"Airspeed is below the touchdown target; the flare will be firm.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if along < 0.0f64 && flaring as ::core::ffi::c_int != 0 && V < v_td * 1.1f64 {
        warn = b"Energy is low for the runway threshold.\0" as *const u8
            as *const ::core::ffi::c_char;
    }
    let mut r: GuidanceResult = result_make(
        phase,
        c,
        &raw mut status as *mut ::core::ffi::c_char,
        warn,
    );
    trajectory_copy(&raw mut r.reference, ref_0);
    r = stabilized(g, r, t, v, s, dt);
    if (*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        let mut shaped: ::core::ffi::c_double = clampd(
            c.target_aoa,
            0.0f64,
            fmin((*v).maximum_angle_of_attack, GUIDANCE_HARD_AOA_LIMIT_DEG),
        );
        r.command.target_aoa = shaped;
        r.command.target_pitch = (*t).flight_path_angle + shaped;
        (*g).pitch_limiter.has_value = false_0 != 0;
    }
    if !(*g).diagnostic_shadow
        && !getenv(
                b"KSP_LANDER_FINAL_TRACE\0" as *const u8 as *const ::core::ffi::c_char,
            )
            .is_null()
    {
        fprintf(
            stderr,
            b"FINAL_TRACE g=%p ut=%.2f h=%.1f x=%.0f V=%.1f Vref=%.1f sink=%.1f latch=%d arc=%d pfh=%.0f trigger=%.0f plan=%d planSpeed=%.1f resp=%.1f arr=%.2f aim=%.0f glide=%.1f ref=%.1f acc=%.2f aoa=%.1f tdPitch=%.1f tdAoA=%.1f reqAoA=%.1f cmdAoA=%.1f ld=%.2f long=%d low=%d short=%d tdRange=%.0f vtd=%.1f sb=%.2f sbOn=%d finCdA=%.2f v1g=%.1f aoaMax=%.1f lr=%.2f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            g as *mut ::core::ffi::c_void,
            (*t).ut,
            height,
            along,
            V,
            speed_ref,
            sink_now,
            (*g).terminal_pull_latch as ::core::ffi::c_int,
            (*g).terminal_arc_started as ::core::ffi::c_int,
            preflare_height,
            (*g).preflare_trigger_altitude,
            if (*g).terminal_preflare_plan_valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            (*g).preflare_reference_speed,
            response,
            arrest_accel,
            (*g).terminal_aim_along,
            glide_fpa * RAD2DEG,
            reference_fpa * RAD2DEG,
            vertical_accel,
            aoa,
            touchdown_attitude_floor,
            touchdown_aoa_floor,
            aoa,
            r.command.target_aoa,
            ld_float,
            long_landing as ::core::ffi::c_int,
            (*g).terminal_final_low_energy as ::core::ffi::c_int,
            range_short as ::core::ffi::c_int,
            predicted_ground_range,
            predicted_touchdown_speed,
            (*g).terminal_speedbrake_fraction,
            if (*s).terminal_speedbrake as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            (*g).terminal_split_fin_cda,
            speed_1g,
            aoa_max,
            (*g).terminal_lift_ratio,
        );
    }
    let mut lat_diag: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_FINAL_LATERAL_DIAGNOSTICS\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    if !lat_diag.is_null()
        && strcmp(lat_diag, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int && fmod((*t).ut, 1.0f64) < dt * 0.999f64
    {
        fprintf(
            stderr,
            b"FINAL_LAT ut=%.3f x=%.1f cross=%.1f course=%.2f track=%.2f runway=%.2f trackErr=%.2f crossRate=%.2f omega=%.3f lat=%.3f bankLimit=%.2f rawBank=%.2f targetHeading=%.2f targetRoll=%.2f actualRoll=%.2f rollRate=%.2f omegaLim=%.3f rateGain=%.3f reversal=%.2f laterBank=%.2f desired=%.2f pave=%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            along,
            (*t).runway_cross_track,
            course,
            (*t).ground_track_heading,
            (*site).runway_heading,
            track_error * RAD2DEG,
            cross_rate,
            omega,
            lateral_accel,
            bank_limit,
            bank,
            target_course,
            r.command.target_roll,
            (*t).roll,
            (*t).roll_rate,
            omega_limit,
            rate_gain,
            reversal_time,
            later_bank,
            desired_rate,
            if pavement_intercept as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
        );
    }
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_approach_sequence(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut ref_0: *const Trajectory,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    if strcasecmp(
        &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0 && !terminal_runway_contact_position(t, cfg)
    {
        return terminal_abort(
            g,
            b"Touchdown occurred outside the runway contact envelope for the selected end.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if strcasecmp(
        &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0 && !(*t).gear
    {
        return terminal_abort(
            g,
            b"Touchdown occurred without confirmed landing-gear deployment.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if !(*g).terminal_vertical_stage_valid {
        terminal_set_stage(g, TERMINAL_OUTER_FINAL, (*t).ut);
    }
    terminal_observe_response(g, t, dt);
    if terminal_update_ground_latch(g, t, cfg, dt) {
        return touchdown(g, t, cfg, ref_0);
    }
    return terminal_landing_guidance(g, t, course, p, cfg, ref_0, dt);
}
#[no_mangle]
pub unsafe extern "C" fn terminal_native_delivery_admissible(
    mut m: *const TerminalModel,
    mut state: *const TerminalDynamicState,
    mut seed: *const GuidanceMachine,
    mut evaluation: *mut TaemTerminalEvaluation,
) -> bool {
    let mut contract: TaemTerminalContract = TaemTerminalContract {
        valid: 0 as ::core::ffi::c_int != 0,
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
    };
    if !evaluation.is_null() {
        *evaluation = taem_exec_evaluate_terminal_contract(&raw mut contract);
    }
    if m.is_null() || state.is_null() || seed.is_null()
        || (*m).world.atmosphere.count == 0 as size_t
        || (*m).world.atmosphere.count > LANDER_ATMOSPHERE_SAMPLE_MAX as size_t
    {
        return false_0 != 0;
    }
    let mut geometry: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    if !taem_geometry_state(m, state, &raw mut geometry) {
        return false_0 != 0;
    }
    let mut planet: PlanetModel = PlanetModel {
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
    planet.radius = (*m).world.radius_m;
    planet.gravitational_parameter = (*m).world.mu_m3_s2;
    planet.rotational_speed = (*m).world.rotation_rate_rad_s;
    planet.atmosphere_depth = (*m).world.atmosphere_top_m;
    planet.atmosphere_adiabatic_index = 1.4f64;
    planet.north_axis = Vector3 {
        x: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        z: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    planet.prime_meridian_at_epoch = Vector3 {
        x: cos((*m).world.rotation_phase_rad_at_ut0),
        y: sin((*m).world.rotation_phase_rad_at_ut0),
        z: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    planet.atmosphere_sample_count = (*m).world.atmosphere.count;
    let mut i: size_t = 0 as size_t;
    while i < planet.atmosphere_sample_count {
        let mut point: *const AtmospherePoint = (&raw const (*m).world.atmosphere.p
            as *const AtmospherePoint)
            .offset(i as isize) as *const AtmospherePoint;
        planet.atmosphere_altitude[i as usize] = (*point).altitude_m;
        planet.atmosphere_pressure[i as usize] = (*point).sample.pressure_pa;
        planet.atmosphere_density[i as usize] = (*point).sample.density_kg_m3;
        i = i.wrapping_add(1);
    }
    planet.surface_density = planet.atmosphere_density[0 as ::core::ffi::c_int as usize];
    let mut lla: LLA = world_lla(
        &raw const (*m).world,
        (*state).position_i_m,
        (*state).ut_s,
    );
    let mut air: AtmosphereSample = world_atmosphere_sample_state(
        &raw const (*m).world,
        (*state).position_i_m,
        (*state).ut_s,
    );
    let mut forces: AeroForces = aero_compute(
        &raw const (*m).world,
        &raw const (*m).aero,
        (*state).position_i_m,
        (*state).velocity_i_mps,
        (*state).ut_s,
        (*state).mass_kg,
        (*state).attitude.aoa_rad,
        (*state).attitude.bank_rad,
    );
    let mut t: Telemetry = Telemetry {
        ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
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
    };
    t.ut = (*state).ut_s;
    t.mass = (*state).mass_kg;
    t.latitude = lla.lat_rad * RAD2DEG;
    t.longitude = lla.lon_rad * RAD2DEG;
    t.mean_altitude = lla.altitude_m;
    t.radar_altitude = fmax(0.0f64, geometry.altitude_above_runway_m);
    t.runway_along_track = geometry.runway_along_m;
    t.runway_cross_track = geometry.runway_cross_m;
    t.true_air_speed = geometry.airspeed_mps;
    t.horizontal_speed = geometry.ground_speed_mps;
    t.vertical_speed = t.true_air_speed * sin(geometry.flight_path_angle_deg * DEG2RAD);
    t.flight_path_angle = geometry.flight_path_angle_deg;
    t.ground_track_heading = geometry.course_deg;
    t.heading = t.ground_track_heading;
    t.angle_of_attack = (*state).attitude.aoa_rad * RAD2DEG;
    t.pitch = t.flight_path_angle + t.angle_of_attack;
    t.roll = (*state).attitude.bank_rad * RAD2DEG;
    t.roll_rate = (*state).attitude.bank_rate_rad_s * RAD2DEG;
    t.mach = forces.mach;
    t.dynamic_pressure = forces.dynamic_pressure_pa;
    t.atmospheric_density = air.density_kg_m3;
    t.speed_of_sound = air.speed_of_sound_mps;
    t.lift_force = forces.lift_n;
    t.drag_force = forces.drag_n;
    t.g_force = fabs(forces.lift_n) / fmax(t.mass, 1.0f64) / 9.80665f64;
    t.estimated_ballistic_coefficient = (*m).vehicle.estimated_ballistic_coefficient;
    t.aerodynamic_confidence = 1.0f64;
    snprintf(
        &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"flying\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut cfg: LandingConfiguration = LandingConfiguration {
        connection: ConnectionConfiguration {
            serial_port: [0; 256],
            rpc_host: [0; 256],
            rpc_port: 0,
            baud_rate: 0,
            timeout_ms: 0,
            client_name: [0; 128],
        },
        site: (*m).site,
        vehicle: (*m).vehicle,
        guidance: (*m).guidance,
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
    let mut guidance: GuidanceMachine = *seed;
    guidance.terminal_glide_mode = !(*seed).terminal_final_test_mode;
    guidance.terminal_test_glide_slope = 0.0f64;
    guidance.terminal_test_final_approach_distance = 0.0f64;
    guidance.terminal_final_handoff_latched = true_0 != 0;
    guidance.terminal_final_handoff_distance = (*m).guidance.final_approach_distance;
    guidance.terminal_final_handoff_slope_deg = (*m).guidance.final_glide_slope;
    guidance.terminal_final_handoff_speed_mps = (*m).guidance.final_alignment_speed;
    guidance.mm305_route_committed = true_0 != 0;
    guidance.terminal_path_committed = guidance.mm305_route_committed;
    terminal_observe_landing_aero(
        &raw mut guidance,
        &raw mut t,
        &raw mut cfg.vehicle,
        planet_surface_gravity(&raw mut planet),
        0.1f64,
    );
    let mut aero: AerodynamicModel = AerodynamicModel {
        lift_to_drag: cfg.vehicle.estimated_lift_to_drag,
        ballistic_coefficient: cfg.vehicle.estimated_ballistic_coefficient,
        confidence: 1.0f64,
    };
    let mut plan: TerminalPreflarePlan = TerminalPreflarePlan {
        feasible: 0 as ::core::ffi::c_int != 0,
        reject_reason: ::core::ptr::null::<::core::ffi::c_char>(),
        trigger_altitude: 0.,
        target_aoa: 0.,
        target_sink: 0.,
        minimum_speed: 0.,
        reference_speed: 0.,
        predicted_height_loss: 0.,
        predicted_kinetic_margin: 0.,
        effective_accel: 0.,
        response_time: 0.,
        plan_source_ut: 0.,
        height: DecisionMargin {
            valid: false,
            available: 0.,
            required: 0.,
            margin: 0.,
            normalized_margin: 0.,
        },
        degraded_floor: false,
    };
    let mut approach: bool = terminal_outer_capture_admissible(
        &raw mut guidance,
        &raw mut t,
        geometry.course_deg,
        &raw mut planet,
        aero,
        &raw mut cfg,
        &raw mut plan,
    );
    contract = terminal_delivery_contract(
        &raw mut guidance,
        &raw mut t,
        geometry.course_deg,
        &raw mut planet,
        &raw mut cfg,
        &raw mut plan,
    );
    let mut verdict: TaemTerminalEvaluation = taem_exec_evaluate_terminal_contract(
        &raw mut contract,
    );
    if !evaluation.is_null() {
        *evaluation = verdict;
    }
    return approach as ::core::ffi::c_int != 0
        && verdict.valid as ::core::ffi::c_int != 0
        && verdict.feasible as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn final_lift_aoa_limit(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut limit: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    if (*t).mach < 1.0f64 && (*v).terminal_maximum_lift_angle_of_attack > 0.0f64 {
        limit = fmin(limit, (*v).terminal_maximum_lift_angle_of_attack);
    }
    return limit;
}
unsafe extern "C" fn runway_rollout_steering(
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
) -> RolloutSteeringDemand {
    let mut heading_error: ::core::ffi::c_double = norm_signed_deg(
        (*cfg).site.runway_heading - (*t).heading,
    );
    let mut course_error: ::core::ffi::c_double = norm_signed_deg(
        (*t).ground_track_heading - (*cfg).site.runway_heading,
    );
    let mut lateral_rate: ::core::ffi::c_double = (*t).surface_speed
        * sin(course_error * DEG2RAD);
    if lateral_rate.is_finite() as i32 == 0 {
        lateral_rate = 0.0f64;
    }
    let mut rate_weight: ::core::ffi::c_double = clampd(
        ((*t).surface_speed - 8.0f64) / 35.0f64,
        0.0f64,
        1.0f64,
    );
    let mut authority: ::core::ffi::c_double = clampd(
        55.0f64 / fmax((*t).surface_speed, 15.0f64),
        0.25f64,
        1.0f64,
    );
    let mut correction: ::core::ffi::c_double = (heading_error * 0.09f64
        - (*t).runway_cross_track * 0.0033f64 - lateral_rate * 0.036f64 * rate_weight)
        * authority;
    let mut steer: ::core::ffi::c_double = clampd(correction, -0.40f64, 0.40f64);
    return RolloutSteeringDemand {
        steering: steer,
        lateral_rate: lateral_rate,
    };
}
#[no_mangle]
pub unsafe extern "C" fn touchdown(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
    mut ref_0: *const Trajectory,
) -> GuidanceResult {
    let mut steering: RolloutSteeringDemand = runway_rollout_steering(t, cfg);
    let mut steer: ::core::ffi::c_double = steering.steering;
    let mut lateral_rate: ::core::ffi::c_double = steering.lateral_rate;
    let mut c: GuidanceCommand = GuidanceCommand {
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
    guidance_command_init(&raw mut c);
    c.gear = true_0 != 0;
    c.airbrakes = false_0 != 0;
    c.wheel_steering = steer;
    c.heading_control_enabled = true_0 != 0;
    c.target_heading = (*cfg).site.runway_heading;
    c.navball_speed_mode = SPEED_SURFACE;
    c.control_profile = PROFILE_ROLLOUT;
    let mut sts_delivery: bool = (*cfg).guidance.sts_trajectory_delivery
        as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    let mut nose_gear_contact: bool = (*t).has_nose_gear_grounded as ::core::ffi::c_int
        != 0 && (*t).nose_gear_grounded as ::core::ffi::c_int != 0
        || (*t).pitch <= 0.5f64
        || sts_delivery as ::core::ffi::c_int != 0
            && ((*g).phase as ::core::ffi::c_uint
                == PHASE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint
                || (*g).phase as ::core::ffi::c_uint
                    == PHASE_COMPLETE as ::core::ffi::c_int as ::core::ffi::c_uint);
    let mut status: [::core::ffi::c_char; 160] = [0; 160];
    let mut phase: GuidancePhase = PHASE_IDLE;
    if (*t).surface_speed.is_finite() as i32 == 0 || (*t).surface_speed < 0.0f64 {
        return terminal_abort(
            g,
            b"Rollout speed is invalid; stopping cannot be confirmed.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if (*t).surface_speed < 1.5f64 {
        if (*t).runway_along_track.is_finite() as i32 == 0
            || (*t).runway_cross_track.is_finite() as i32 == 0
            || !((*cfg).site.runway_length > 0.0f64)
            || !((*cfg).site.runway_width > 0.0f64) || (*t).runway_along_track < 0.0f64
            || (*t).runway_along_track > (*cfg).site.runway_length
            || fabs((*t).runway_cross_track) > (*cfg).site.runway_width * 0.5f64
        {
            return terminal_abort(
                g,
                b"Rollout stopped outside the selected runway bounds.\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
        phase = PHASE_COMPLETE;
        c.wheel_steering = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        c.brakes = true_0 != 0;
        reset_limiters(g);
        snprintf(
            &raw mut status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
            b"Landing complete.\0" as *const u8 as *const ::core::ffi::c_char,
        );
    } else if !nose_gear_contact {
        phase = PHASE_TOUCHDOWN;
        c.brakes = false_0 != 0;
        c.has_target_aoa = false_0 != 0;
        snprintf(
            &raw mut status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
            b"Touchdown: nose lowering, pitch %.1f deg, sink %.1f m/s.\0" as *const u8
                as *const ::core::ffi::c_char,
            (*t).pitch,
            (*t).vertical_speed,
        );
    } else {
        phase = PHASE_ROLLOUT;
        c.has_target_aoa = false_0 != 0;
        c.target_pitch = 0.0f64;
        c.brakes = (*t).surface_speed
            < fmax(120.0f64, (*cfg).vehicle.touchdown_speed * 1.35f64);
        snprintf(
            &raw mut status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
            b"Rollout: %.1f m/s, steering %+.2f, lateral %.1f m/s.\0" as *const u8
                as *const ::core::ffi::c_char,
            (*t).surface_speed,
            steer,
            lateral_rate,
        );
    }
    (*g).phase = phase;
    (*g).previous_ut = (*t).ut;
    (*g).has_previous_ut = true_0 != 0;
    let mut r: GuidanceResult = result_make(
        phase,
        c,
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::ptr::null::<::core::ffi::c_char>(),
    );
    trajectory_copy(&raw mut r.reference, ref_0);
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_abort(
    mut g: *mut GuidanceMachine,
    mut reason: *const ::core::ffi::c_char,
) -> GuidanceResult {
    snprintf(
        &raw mut (*g).abort_reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        reason,
    );
    let mut c: GuidanceCommand = GuidanceCommand {
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
    guidance_command_init(&raw mut c);
    return result_make(PHASE_ABORT, c, reason, reason);
}
unsafe extern "C" fn observed_signed_roll_rate(
    mut t: *const Telemetry,
) -> ::core::ffi::c_double {
    let mut euler: ::core::ffi::c_double = (*t).roll_rate;
    if !(*t).has_body_roll_rate || (*t).body_roll_rate.is_finite() as i32 == 0 {
        return euler;
    }
    let mut c: ::core::ffi::c_double = cos(norm_signed_deg((*t).roll) * DEG2RAD);
    let mut euler_weight: ::core::ffi::c_double = c * c;
    let mut weighted_euler: ::core::ffi::c_double = euler * euler_weight;
    return if fabs((*t).body_roll_rate) >= fabs(weighted_euler) {
        (*t).body_roll_rate
    } else {
        weighted_euler
    };
}
#[no_mangle]
pub unsafe extern "C" fn control_recovery_needed(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut v: *const VehicleProfile,
    mut dt: ::core::ffi::c_double,
) -> bool {
    let mut hac_control_tuning: bool = (*g).terminal_region_entered as ::core::ffi::c_int
        != 0
        && ((*g).phase as ::core::ffi::c_uint
            == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).hac_side_selected as ::core::ffi::c_int != 0)
        && (!(*g).terminal_glide_mode || !(*g).terminal_test_capture_active
            || (*g).phase as ::core::ffi::c_uint
                == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint);
    let mut signed_roll_rate: ::core::ffi::c_double = observed_signed_roll_rate(t);
    let mut roll_rate: ::core::ffi::c_double = fabs(signed_roll_rate);
    let mut pitch_rate: ::core::ffi::c_double = fmax(
        fabs((*t).pitch_rate),
        if (*t).has_body_pitch_rate as ::core::ffi::c_int != 0 {
            fabs((*t).body_pitch_rate)
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        },
    );
    let mut signed_pitch_rate: ::core::ffi::c_double = if (*t).has_body_pitch_rate
        as ::core::ffi::c_int != 0 && (*t).body_pitch_rate.is_finite() as i32 != 0
    {
        (*t).body_pitch_rate
    } else {
        (*t).pitch_rate
    };
    let mut body_yaw_rate: ::core::ffi::c_double = if (*t).has_body_yaw_rate
        as ::core::ffi::c_int != 0
    {
        fabs((*t).body_yaw_rate)
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut sideslip: ::core::ffi::c_double = fabs((*t).sideslip);
    let mut signed_aoa: ::core::ffi::c_double = (*t).angle_of_attack;
    let mut outward_aoa_rate: ::core::ffi::c_double = 0.0f64;
    if (*t).has_angle_of_attack_rate as ::core::ffi::c_int != 0
        && (*t).angle_of_attack_rate.is_finite() as i32 != 0
    {
        outward_aoa_rate = (if signed_aoa >= 0.0f64 { 1.0f64 } else { -1.0f64 })
            * (*t).angle_of_attack_rate;
    }
    let mut projected_abs_aoa: ::core::ffi::c_double = fabs(signed_aoa)
        + fmax(0.0f64, outward_aoa_rate) * GUIDANCE_ATTITUDE_LOOKAHEAD_S;
    let mut attitude_envelope_threat: bool = projected_abs_aoa
        >= fmax(GUIDANCE_RECOVERY_AOA_THREAT_DEG, (*v).maximum_angle_of_attack + 2.0f64);
    let mut beta_rate: ::core::ffi::c_double = 0.0f64;
    if (*g).has_previous_sideslip as ::core::ffi::c_int != 0 && dt > 1e-4f64 {
        beta_rate = norm_signed_deg((*t).sideslip - (*g).previous_sideslip) / dt;
    }
    (*g).entry_coordination_beta_rate = beta_rate;
    (*g).previous_sideslip = (*t).sideslip;
    (*g).has_previous_sideslip = true_0 != 0;
    let mut roll_saturated: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && fabs((*t).control_roll) >= 0.35f64;
    let mut pitch_saturated: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && fabs((*t).control_pitch) >= 0.35f64;
    let mut yaw_saturated: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && fabs((*t).control_yaw) >= 0.20f64;
    let mut saturated: bool = roll_saturated as ::core::ffi::c_int != 0
        || pitch_saturated as ::core::ffi::c_int != 0
        || !hac_control_tuning && yaw_saturated as ::core::ffi::c_int != 0;
    let mut nominal_roll_rate: ::core::ffi::c_double = (*s).entry_roll_rate;
    if (*g).phase as ::core::ffi::c_uint
        == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*g).phase as ::core::ffi::c_uint
            == PHASE_HEADING_ALIGNMENT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        nominal_roll_rate = (*s).taem_roll_rate;
    } else if (*g).phase as ::core::ffi::c_uint
        == PHASE_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*g).phase as ::core::ffi::c_uint
            == PHASE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        nominal_roll_rate = (*s).approach_roll_rate;
    }
    let mut bad_roll_rate: ::core::ffi::c_double = fmax(
        12 as ::core::ffi::c_int as ::core::ffi::c_double,
        nominal_roll_rate * 3 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut extreme_roll_rate: ::core::ffi::c_double = fmax(
        24 as ::core::ffi::c_int as ::core::ffi::c_double,
        nominal_roll_rate * 6 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut native_route_tracking: bool = (*g).phase as ::core::ffi::c_uint
        == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*g).mm305_route_committed as ::core::ffi::c_int != 0
        && (*g).mm305_route.valid as ::core::ffi::c_int != 0
        && !(*g).mm305_hac_exit_reached;
    let mut previous_roll_target: ::core::ffi::c_double = if (*g).roll_limiter.has_value
        as ::core::ffi::c_int != 0 || native_route_tracking as ::core::ffi::c_int != 0
    {
        norm_signed_deg((*g).roll_limiter.value)
    } else {
        norm_signed_deg((*t).roll)
    };
    let mut roll_error: ::core::ffi::c_double = norm_signed_deg(
        previous_roll_target - norm_signed_deg((*t).roll),
    );
    let mut reference_roll_rate: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    if !native_route_tracking && (*g).roll_limiter.has_value as ::core::ffi::c_int != 0
        && (*g).roll_limiter.rate.is_finite() as i32 != 0
    {
        reference_roll_rate = (*g).roll_limiter.rate;
    }
    let mut fixed_hac_bank_captured: bool = (*g).mm305_route_committed
        as ::core::ffi::c_int != 0 && (*g).mm305_route.valid as ::core::ffi::c_int != 0
        && !(*g).mm305_hac_exit_reached && fabs(roll_error) <= 1.5f64;
    let mut relative_roll_rate: ::core::ffi::c_double = if fixed_hac_bank_captured
        as ::core::ffi::c_int != 0
    {
        signed_roll_rate
    } else {
        signed_roll_rate - reference_roll_rate
    };
    let mut energetic_crossing_rate: ::core::ffi::c_double = fmax(
        8.0f64,
        nominal_roll_rate * 1.30f64,
    );
    let mut sustained_excess_rate: ::core::ffi::c_double = fmax(
        12.0f64,
        nominal_roll_rate * 2.0f64,
    );
    let mut commanded_roll_motion: bool = fabs(roll_error) > 1.5f64
        && roll_error * signed_roll_rate
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut roll_rate_braking: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && fabs((*t).control_roll) > 0.03f64
        && (*t).control_roll * signed_roll_rate
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut energetic_crossing: bool = (*g).has_previous_relative_roll_rate
        as ::core::ffi::c_int != 0
        && (*g).previous_relative_roll_rate * relative_roll_rate
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && fabs((*g).previous_relative_roll_rate) > energetic_crossing_rate
        && fabs(relative_roll_rate) > energetic_crossing_rate;
    (*g).roll_oscillation_score = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        (*g).roll_oscillation_score - dt * 0.35f64,
    );
    if energetic_crossing {
        (*g).roll_oscillation_score += 1.0f64;
    }
    let mut excess_roll_motion: bool = fabs(relative_roll_rate) > sustained_excess_rate
        && !commanded_roll_motion && !roll_rate_braking;
    if excess_roll_motion {
        (*g).roll_rate_excess_duration += dt;
    } else {
        (*g).roll_rate_excess_duration = fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            (*g).roll_rate_excess_duration - dt * 1.5f64,
        );
    }
    (*g).previous_relative_roll_rate = relative_roll_rate;
    (*g).has_previous_relative_roll_rate = true_0 != 0;
    let mut oscillatory_roll: bool = (*g).roll_oscillation_score >= 1.8f64
        || (*g).roll_rate_excess_duration >= 1.25f64;
    let mut aero_accel: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    if (*t).mass > 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*t).lift_force.is_finite() as i32 != 0
        && (*t).drag_force.is_finite() as i32 != 0
    {
        aero_accel = hypot((*t).lift_force, (*t).drag_force) / (*t).mass;
    }
    let mut aero_loaded: bool = aero_accel > 0.02f64;
    let mut bank_excess: ::core::ffi::c_double = fabs(norm_signed_deg((*t).roll))
        - (*v).maximum_bank_angle;
    let mut bank_departure: bool = bank_excess
        > fmax(15.0f64, (*v).maximum_bank_angle * 0.22f64);
    let mut beta_growth: ::core::ffi::c_double = (*t).sideslip * beta_rate;
    let mut beta_growth_fraction: ::core::ffi::c_double = beta_growth
        / ((*t).sideslip * (*t).sideslip + 25.0f64);
    let mut signed_body_yaw_rate: ::core::ffi::c_double = if (*t).has_body_yaw_rate
        as ::core::ffi::c_int != 0 && (*t).body_yaw_rate.is_finite() as i32 != 0
    {
        (*t).body_yaw_rate
    } else {
        (*t).heading_rate
    };
    let mut corrective_beta_yaw: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && fabs((*t).control_yaw) > 0.08f64
        && (*t).sideslip * (*t).control_yaw
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut yaw_rate_braking: bool = corrective_beta_yaw as ::core::ffi::c_int != 0
        && (*t).control_yaw * signed_body_yaw_rate
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut beta_runaway: bool = sideslip
        > 10 as ::core::ffi::c_int as ::core::ffi::c_double
        && beta_growth_fraction > 0.12f64
        && (saturated as ::core::ffi::c_int != 0
            || bank_departure as ::core::ffi::c_int != 0
            || body_yaw_rate > 6 as ::core::ffi::c_int as ::core::ffi::c_double)
        && !yaw_rate_braking;
    let mut uncoordinated_yaw: bool = !hac_control_tuning
        && (body_yaw_rate > 20 as ::core::ffi::c_int as ::core::ffi::c_double
            || sideslip > 20 as ::core::ffi::c_int as ::core::ffi::c_double
                && fabs((*t).heading_rate)
                    > 12 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut extreme_yaw: bool = !hac_control_tuning
        && (body_yaw_rate > 35 as ::core::ffi::c_int as ::core::ffi::c_double
            || sideslip > 30 as ::core::ffi::c_int as ::core::ffi::c_double
                && fabs((*t).heading_rate)
                    > 20 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut yaw_saturated_departure: bool = !hac_control_tuning
        && yaw_saturated as ::core::ffi::c_int != 0
        && (body_yaw_rate > 10 as ::core::ffi::c_int as ::core::ffi::c_double
            || sideslip > 12 as ::core::ffi::c_int as ::core::ffi::c_double
                && fabs((*t).heading_rate)
                    > 8 as ::core::ffi::c_int as ::core::ffi::c_double)
        && !yaw_rate_braking;
    let mut emergency_rate: bool = roll_rate
        > 75 as ::core::ffi::c_int as ::core::ffi::c_double
        || pitch_rate > 45 as ::core::ffi::c_int as ::core::ffi::c_double
        || aero_loaded as ::core::ffi::c_int != 0
            && extreme_yaw as ::core::ffi::c_int != 0
        || bank_departure as ::core::ffi::c_int != 0
            && sideslip > 25 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut extreme_roll_motion: bool = roll_rate > extreme_roll_rate
        && !roll_rate_braking;
    let mut extreme_rate: bool = emergency_rate as ::core::ffi::c_int != 0
        || aero_loaded as ::core::ffi::c_int != 0
            && (extreme_roll_motion as ::core::ffi::c_int != 0
                || pitch_rate > 35 as ::core::ffi::c_int as ::core::ffi::c_double
                || extreme_yaw as ::core::ffi::c_int != 0);
    let mut divergent_roll: bool = !commanded_roll_motion && !roll_rate_braking
        && (roll_rate > bad_roll_rate
            || roll_saturated as ::core::ffi::c_int != 0
                && roll_rate
                    > nominal_roll_rate
                        * 2 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut target_aoa: ::core::ffi::c_double = if (*g).pitch_limiter.has_value
        as ::core::ffi::c_int != 0
    {
        clampd(
            (*g).pitch_limiter.value,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            (*v).maximum_angle_of_attack,
        )
    } else {
        (*t).angle_of_attack
    };
    let mut aoa_error: ::core::ffi::c_double = target_aoa - (*t).angle_of_attack;
    let mut corrective_pitch_saturation: bool = pitch_saturated as ::core::ffi::c_int
        != 0 && fabs(aoa_error) > 3 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*t).has_controls as ::core::ffi::c_int != 0
        && aoa_error * (*t).control_pitch
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut pitch_rate_braking: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && fabs((*t).control_pitch) > 0.08f64
        && (*t).control_pitch * signed_pitch_rate
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut pitch_departure: bool = pitch_rate
        > 18 as ::core::ffi::c_int as ::core::ffi::c_double
        || pitch_saturated as ::core::ffi::c_int != 0
            && pitch_rate > 10 as ::core::ffi::c_int as ::core::ffi::c_double
            && !corrective_pitch_saturation && !pitch_rate_braking;
    let mut bad: bool = attitude_envelope_threat as ::core::ffi::c_int != 0
        || beta_runaway as ::core::ffi::c_int != 0
        || aero_loaded as ::core::ffi::c_int != 0
            && (oscillatory_roll as ::core::ffi::c_int != 0
                || extreme_rate as ::core::ffi::c_int != 0
                || divergent_roll as ::core::ffi::c_int != 0
                || pitch_departure as ::core::ffi::c_int != 0
                || uncoordinated_yaw as ::core::ffi::c_int != 0
                || yaw_saturated_departure as ::core::ffi::c_int != 0);
    (*g).control_bad_duration = if bad as ::core::ffi::c_int != 0 {
        (*g).control_bad_duration + dt
    } else {
        fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            (*g).control_bad_duration - dt * 1.5f64,
        )
    };
    let mut recovery_trigger: bool = !(*g).attitude_recovery
        && (attitude_envelope_threat as ::core::ffi::c_int != 0
            || extreme_rate as ::core::ffi::c_int != 0
            || oscillatory_roll as ::core::ffi::c_int != 0
            || (*g).control_bad_duration >= 0.35f64);
    if recovery_trigger {
        let mut diag_env: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_HAC_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if !diag_env.is_null()
            && strcmp(diag_env, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
        {
            fprintf(
                stderr,
                b"CONTROL RECOVERY trigger: UT %.2f phase %d alt %.0f q %.3f V %.1f roll %.2f rollRate %.2f bodyRoll %.2f pitchRate %.2f bodyPitch %.2f beta %.2f betaRate %.2f bodyYaw %.2f headingRate %.2f controls %.3f/%.3f/%.3f aeroAccel %.4f loaded=%d badDur %.2f relRate %.2f refRate %.2f rollErr %.2f aoa %.2f targetAoA %.2f flags bad=%d beta=%d oscillatory=%d extreme=%d divergent=%d pitch=%d yaw=%d yawSat=%d emergency=%d braking=%d rollSat=%d pitchSat=%d yawSatCmd=%d\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*g).phase as ::core::ffi::c_uint,
                (*t).mean_altitude,
                (*t).dynamic_pressure,
                (*t).true_air_speed,
                (*t).roll,
                signed_roll_rate,
                if (*t).has_body_roll_rate as ::core::ffi::c_int != 0 {
                    (*t).body_roll_rate
                } else {
                    ::core::f32::NAN as ::core::ffi::c_double
                },
                pitch_rate,
                if (*t).has_body_pitch_rate as ::core::ffi::c_int != 0 {
                    (*t).body_pitch_rate
                } else {
                    ::core::f32::NAN as ::core::ffi::c_double
                },
                (*t).sideslip,
                beta_rate,
                body_yaw_rate,
                (*t).heading_rate,
                if (*t).has_controls as ::core::ffi::c_int != 0 {
                    (*t).control_roll
                } else {
                    ::core::f32::NAN as ::core::ffi::c_double
                },
                if (*t).has_controls as ::core::ffi::c_int != 0 {
                    (*t).control_pitch
                } else {
                    ::core::f32::NAN as ::core::ffi::c_double
                },
                if (*t).has_controls as ::core::ffi::c_int != 0 {
                    (*t).control_yaw
                } else {
                    ::core::f32::NAN as ::core::ffi::c_double
                },
                aero_accel,
                aero_loaded as ::core::ffi::c_int,
                (*g).control_bad_duration,
                relative_roll_rate,
                reference_roll_rate,
                roll_error,
                (*t).angle_of_attack,
                target_aoa,
                bad as ::core::ffi::c_int,
                beta_runaway as ::core::ffi::c_int,
                oscillatory_roll as ::core::ffi::c_int,
                extreme_rate as ::core::ffi::c_int,
                divergent_roll as ::core::ffi::c_int,
                pitch_departure as ::core::ffi::c_int,
                uncoordinated_yaw as ::core::ffi::c_int,
                yaw_saturated_departure as ::core::ffi::c_int,
                emergency_rate as ::core::ffi::c_int,
                roll_rate_braking as ::core::ffi::c_int,
                roll_saturated as ::core::ffi::c_int,
                pitch_saturated as ::core::ffi::c_int,
                yaw_saturated as ::core::ffi::c_int,
            );
        }
        (*g).attitude_recovery = true_0 != 0;
        (*g).control_good_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*g).recovery_duration = (*g).control_good_duration;
        (*g).recovery_pitch_trim = if (*t).has_controls as ::core::ffi::c_int != 0
            && (*t).control_pitch.is_finite() as i32 != 0
        {
            (*t).control_pitch
        } else {
            0.0f64
        };
        (*g).recovery_heading = norm_deg((*t).ground_track_heading);
        if (*g).terminal_region_entered as ::core::ffi::c_int != 0
            && (*g).terminal_glide_mode as ::core::ffi::c_int != 0
        {
            (*g).recovery_aoa = clampd(
                fmax(4.0f64, (*t).angle_of_attack + 1.5f64),
                4.0f64,
                10.0f64,
            );
        } else {
            (*g).recovery_aoa = clampd((*v).entry_angle_of_attack, 10.0f64, 18.0f64);
        }
        reset_limiters(g);
        (*g).attitude_recovery = true_0 != 0;
    }
    if !(*g).attitude_recovery {
        return false_0 != 0;
    }
    (*g).recovery_duration += dt;
    let mut recovery_aoa: ::core::ffi::c_double = if (*g).recovery_aoa
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        (*g).recovery_aoa
    } else {
        clampd(
            (*v).entry_angle_of_attack,
            10 as ::core::ffi::c_int as ::core::ffi::c_double,
            18 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    };
    let mut pitch_captured: bool = fabs((*t).angle_of_attack - recovery_aoa) < 4.0f64
        && (*t).angle_of_attack >= 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut destabilizing_roll_saturation: bool = roll_saturated as ::core::ffi::c_int
        != 0 && !roll_rate_braking;
    if (*t).has_controls as ::core::ffi::c_int != 0
        && (*t).control_pitch.is_finite() as i32 != 0
    {
        (*g).recovery_pitch_trim
            += dt / (3.0f64 + dt) * ((*t).control_pitch - (*g).recovery_pitch_trim);
    }
    let mut pitch_off_trim: bool = (*t).has_controls as ::core::ffi::c_int != 0
        && (fabs((*t).control_pitch) >= 0.95f64
            || fabs((*t).control_pitch - (*g).recovery_pitch_trim) >= 0.35f64);
    let mut destabilizing_pitch_saturation: bool = pitch_saturated as ::core::ffi::c_int
        != 0 && pitch_off_trim as ::core::ffi::c_int != 0 && !corrective_pitch_saturation
        && !pitch_rate_braking;
    let mut destabilizing_yaw_saturation: bool = !hac_control_tuning
        && yaw_saturated as ::core::ffi::c_int != 0
        && (sideslip > 8 as ::core::ffi::c_int as ::core::ffi::c_double
            || body_yaw_rate > 5 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut stable: bool = pitch_captured as ::core::ffi::c_int != 0
        && roll_rate < 5 as ::core::ffi::c_int as ::core::ffi::c_double
        && pitch_rate < 5 as ::core::ffi::c_int as ::core::ffi::c_double
        && fabs(norm_signed_deg((*t).roll))
            < 10 as ::core::ffi::c_int as ::core::ffi::c_double
        && (!aero_loaded
            || sideslip < 8 as ::core::ffi::c_int as ::core::ffi::c_double
                && !destabilizing_roll_saturation && !destabilizing_pitch_saturation
                && !destabilizing_yaw_saturation);
    if !(*g).diagnostic_shadow
        && !getenv(
                b"KSP_LANDER_RECOVERY_TRACE\0" as *const u8 as *const ::core::ffi::c_char,
            )
            .is_null()
    {
        fprintf(
            stderr,
            b"RECOVERY_TRACE ut=%.2f aoa=%.2f rec=%.1f tgt=%.2f roll=%.2f rr=%.2f pr=%.2f beta=%.2f ctl=%.3f/%.3f/%.3f cap=%d psat=%d corr=%d brake=%d rsat=%d rbrake=%d ysat=%d good=%.2f dur=%.2f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            (*t).angle_of_attack,
            recovery_aoa,
            target_aoa,
            (*t).roll,
            roll_rate,
            pitch_rate,
            sideslip,
            (*t).control_pitch,
            (*t).control_roll,
            (*t).control_yaw,
            pitch_captured as ::core::ffi::c_int,
            pitch_saturated as ::core::ffi::c_int,
            corrective_pitch_saturation as ::core::ffi::c_int,
            pitch_rate_braking as ::core::ffi::c_int,
            roll_saturated as ::core::ffi::c_int,
            roll_rate_braking as ::core::ffi::c_int,
            destabilizing_yaw_saturation as ::core::ffi::c_int,
            (*g).control_good_duration,
            (*g).recovery_duration,
        );
    }
    (*g).control_good_duration = if stable as ::core::ffi::c_int != 0 {
        (*g).control_good_duration + dt
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    if (*g).control_good_duration >= 2 as ::core::ffi::c_int as ::core::ffi::c_double {
        (*g).attitude_recovery = false_0 != 0;
        (*g).control_bad_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*g).roll_oscillation_score = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*g).roll_rate_excess_duration = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        (*g).has_previous_relative_roll_rate = false_0 != 0;
        (*g).recovery_aoa = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_entry_recovery_available(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> bool {
    if g.is_null() || (*g).terminal_region_entered as ::core::ffi::c_int != 0
        || (*g).terminal_rehearsal_mode as ::core::ffi::c_int != 0
        || (*g).final_approach_captured as ::core::ffi::c_int != 0
        || (*t).true_air_speed < (*v).minimum_safe_speed * 2.0f64
    {
        return false_0 != 0;
    }
    let mut response: ::core::ffi::c_double = fmax(
        4.0f64,
        hac_response_lead_time(t, s, 0.0f64),
    );
    let mut recovery_height: ::core::ffi::c_double = fmax(
        fmax((*g).terminal_test_preflare_altitude, (*s).flare_altitude * 8.0f64)
            * 3.0f64,
        fmax(0.0f64, -(*t).vertical_speed) * (response + 4.0f64),
    );
    let mut runway_margin: ::core::ffi::c_double = -(*s).final_approach_distance
        - (*t).runway_along_track;
    let mut recovery_distance: ::core::ffi::c_double = (*t).horizontal_speed * response
        * 1.5f64;
    return (*t).radar_altitude > recovery_height && runway_margin > recovery_distance;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_invalidate_frozen_path(mut g: *mut GuidanceMachine) {
    (*g).terminal_path_kind = TERMINAL_PATH_NONE;
    (*g).terminal_path_committed = false_0 != 0;
    (*g).terminal_final_handoff_latched = false_0 != 0;
    (*g).terminal_final_handoff_distance = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).terminal_final_handoff_slope_deg = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).terminal_final_handoff_speed_mps = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).hac_side_selected = false_0 != 0;
    (*g).hac_captured = false_0 != 0;
    (*g).hac_completed = false_0 != 0;
    (*g).hac_progress_valid = false_0 != 0;
    (*g).mm305_route = TaemRoute {
        valid: 0 as ::core::ffi::c_int != 0,
        count: 0,
        lead_count: 0,
        arc_count: 0,
        rollout_count: 0,
        length_m: 0.,
        side: 0.,
        hac: TaemFixedHacGeometry {
            entry: TaemPoint2 { x: 0., y: 0. },
            center: TaemPoint2 { x: 0., y: 0. },
            exit: TaemPoint2 { x: 0., y: 0. },
            radius_m: 0.,
            side: 0.,
            capture_course_deg: 0.,
            arc_sweep_rad: 0.,
            arc_length_m: 0.,
            final_length_m: 0.,
            total_length_m: 0.,
        },
        p0_along_m: 0.,
        p0_cross_m: 0.,
        p1_along_m: 0.,
        p1_cross_m: 0.,
        p2_along_m: 0.,
        p2_cross_m: 0.,
        p3_along_m: 0.,
        p3_cross_m: 0.,
        capture_control: [TaemPoint2 { x: 0., y: 0. }; 6],
        spiral_exit_radius_m: 0.,
        spiral_coefficient_m: 0.,
        spiral_exit_heading_rad: 0.,
        spiral_arc_lut: [0.; 129],
        unload_length_m: 0.,
        settle_length_m: 0.,
        lead_length_m: 0.,
        lead_arc_fraction_lut: [0.; 129],
        rollout_length_m: 0.,
        alignment_along_m: 0.,
        initial_course_deg: 0.,
        runway_heading_deg: 0.,
        profile_start_altitude_m: 0.,
        profile_start_fpa_deg: 0.,
        profile_final_altitude_m: 0.,
        profile_final_slope_deg: 0.,
        profile_total_length_m: 0.,
        profile_midpoint_offset_m: 0.,
        profile_local_offset_m: 0.,
        profile_local_start_fraction: 0.,
        profile_local_peak_fraction: 0.,
        profile_local_end_fraction: 0.,
        profile_initial_sag_m: 0.,
        profile_initial_sag_length_m: 0.,
        profile_tabulated: false,
        profile_generation_aoa_deg: 0.,
        profile_altitude_lut: [0.; 257],
        profile_fpa_lut: [0.; 257],
        profile_speed_tabulated: false,
        profile_speed_lut: [0.; 257],
    };
    (*g).mm305_route_cursor = 0 as size_t;
    (*g).mm305_route_committed = false_0 != 0;
    (*g).mm305_hac_exit_reached = false_0 != 0;
    (*g).mm305_runway_alignment_active = false_0 != 0;
    (*g).mm305_model_snapshot_id = 0 as uint64_t;
    (*g).hac_remaining = 0.0f64;
    (*g).hac_radius = 0.0f64;
    (*g).hac_capture_lost_duration = 0.0f64;
    (*g).terminal_energy_mismatch_duration = 0.0f64;
    (*g).terminal_test_capture_active = (*g).terminal_glide_mode;
    (*g).terminal_energy_sample_valid = false_0 != 0;
    (*g).terminal_energy_loss_accel_ema = 0.0f64;
    (*g).terminal_speed_loss_accel_ema = 0.0f64;
    (*g).terminal_preflare_plan_valid = false_0 != 0;
    (*g).final_invalid_duration = 0.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_invalidate_frozen_path_preserve_energy(
    mut g: *mut GuidanceMachine,
) {
    let mut energy_loss: ::core::ffi::c_double = (*g).terminal_energy_loss_accel_ema;
    let mut speed_loss: ::core::ffi::c_double = (*g).terminal_speed_loss_accel_ema;
    terminal_invalidate_frozen_path(g);
    (*g).terminal_energy_loss_accel_ema = energy_loss;
    (*g).terminal_speed_loss_accel_ema = speed_loss;
    (*g).terminal_energy_sample_valid = false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_return_to_entry(
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
    if taem_exec_owns_vehicle(&raw mut (*g).taem_exec) {
        terminal_invalidate_frozen_path_preserve_energy(g);
        (*g).terminal_region_entered = true_0 != 0;
        (*g).terminal_glide_mode = true_0 != 0;
        (*g).attitude_recovery = false_0 != 0;
        (*g).control_bad_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*g).control_good_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*g).recovery_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*g).airbrakes_deployed = false_0 != 0;
        robust_pid_reset(&raw mut (*g).taem_altitude_pid);
        robust_pid_reset(&raw mut (*g).speed_pid);
        taem_exec_sync(g, t, s, p, aero, cfg);
        (*g).phase = PHASE_TAEM;
        let mut c: GuidanceCommand = atmospheric(
            t,
            (*t).ground_track_heading,
            0.0f64,
            v,
            0.0f64,
            false_0 != 0,
            PROFILE_TAEM,
        );
        c.heading_control_enabled = false_0 != 0;
        c.has_target_aoa = true_0 != 0;
        c.target_aoa = clampd(
            (*v).entry_angle_of_attack,
            10.0f64,
            fmin(18.0f64, (*v).maximum_angle_of_attack),
        );
        c.target_pitch = (*t).flight_path_angle + c.target_aoa;
        return stabilized(
            g,
            result_make(
                PHASE_TAEM,
                c,
                b"TAEM terminal path invalidated; replanning energy and runway geometry without releasing MM305 ownership.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                b"The previous terminal path was discarded, but TAEM remains the sole terminal-area guidance owner.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            ),
            t,
            v,
            s,
            dt,
        );
    }
    terminal_invalidate_frozen_path(g);
    (*g).terminal_region_entered = false_0 != 0;
    (*g).terminal_glide_mode = false_0 != 0;
    (*g).terminal_rehearsal_mode = false_0 != 0;
    (*g).terminal_test_capture_active = false_0 != 0;
    (*g).terminal_reentry_after_ut = 0.0f64;
    reset_taem_handoff_state(g);
    (*g).attitude_recovery = false_0 != 0;
    (*g).control_bad_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).control_good_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).recovery_duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).entry_control_plan_valid = false_0 != 0;
    (*g).entry_control_terminal_ready = false_0 != 0;
    (*g).entry_control_segment_until_ut = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).entry_reversal_scheduled = false_0 != 0;
    (*g).entry_reversal_ut = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).entry_reversal_range = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).entry_reversal_sign = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).entry_reversal_bank = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).has_entry_predictive_score = false_0 != 0;
    (*g).entry_predictive_score = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).airbrakes_deployed = false_0 != 0;
    (*g).entry_reversal_is_final = false_0 != 0;
    (*g).entry_final_reversal_pending = false_0 != 0;
    (*g).entry_final_reversal_completed = false_0 != 0;
    (*g).has_s_turn_reversal_requested = false_0 != 0;
    (*g).has_s_turn_leg_started = false_0 != 0;
    if fabs(norm_signed_deg((*t).roll))
        >= 4 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        (*g).s_turn_sign = (if (*t).roll
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        }) as ::core::ffi::c_double;
    }
    robust_pid_reset(&raw mut (*g).taem_altitude_pid);
    robust_pid_reset(&raw mut (*g).speed_pid);
    (*g).phase = PHASE_ENTRY_ENERGY;
    let mut r: GuidanceResult = entry_program_guidance(
        g,
        t,
        state,
        course,
        p,
        aero,
        cfg,
        dt,
    );
    snprintf(
        &raw mut r.status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Pre-TAEM recovery: discarded advisory terminal geometry; MM304 must re-qualify the interface contract before TAEM ownership.\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    r.has_warning = true_0 != 0;
    snprintf(
        &raw mut r.warning as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Previous advisory terminal preview discarded before TAEM ownership; an explicit MM304 recovery frame is permitted.\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    return r;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const AF_TABLE_STEP: ::core::ffi::c_double = 0.25f64;
pub const AF_TABLE_MAX: ::core::ffi::c_int = 121 as ::core::ffi::c_int;
pub const AF_FLOAT_HEIGHT: ::core::ffi::c_double = 1.5f64;
pub const AF_FLOAT_SINK: ::core::ffi::c_double = 1.0f64;
pub const AF_SGS_SINK: ::core::ffi::c_double = 3.0f64;
pub const AF_SURGE_T: ::core::ffi::c_double = 0.5f64;
pub const AF_SLEW: ::core::ffi::c_double = 3.0f64;
pub const AF_TD_PITCH: ::core::ffi::c_double = 8.0f64;
pub const AF_FLOOR_START: ::core::ffi::c_double = 10.0f64;
pub const AF_FLOOR_FULL: ::core::ffi::c_double = 3.0f64;
pub const AF_MAX_ONSET: ::core::ffi::c_double = 600.0f64;
unsafe extern "C" fn af_response(
    mut F: *const AdaptiveFlareState,
    mut tau: *mut ::core::ffi::c_double,
    mut bias: *mut ::core::ffi::c_double,
    mut gain: *mut ::core::ffi::c_double,
) {
    let prior: [::core::ffi::c_double; 3] = [1.0f64 / 0.8f64, 0.0f64, 0.0f64];
    let lam: [::core::ffi::c_double; 3] = [5.0f64, 3.0f64, 0.1f64];
    let mut A: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    let mut b: [::core::ffi::c_double; 3] = [0.; 3];
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while j < 3 as ::core::ffi::c_int {
            A[i as usize][j as usize] = (*F).sxx[i as usize][j as usize];
            j += 1;
        }
        A[i as usize][i as usize] += lam[i as usize];
        b[i as usize] = (*F).sxy[i as usize] + lam[i as usize] * prior[i as usize];
        i += 1;
    }
    let mut c: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while c < 3 as ::core::ffi::c_int {
        let mut piv: ::core::ffi::c_int = c;
        let mut r: ::core::ffi::c_int = c + 1 as ::core::ffi::c_int;
        while r < 3 as ::core::ffi::c_int {
            if fabs(A[r as usize][c as usize]) > fabs(A[piv as usize][c as usize]) {
                piv = r;
            }
            r += 1;
        }
        if piv != c {
            let mut k: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while k < 3 as ::core::ffi::c_int {
                let mut t: ::core::ffi::c_double = A[c as usize][k as usize];
                A[c as usize][k as usize] = A[piv as usize][k as usize];
                A[piv as usize][k as usize] = t;
                k += 1;
            }
            let mut t_0: ::core::ffi::c_double = b[c as usize];
            b[c as usize] = b[piv as usize];
            b[piv as usize] = t_0;
        }
        if fabs(A[c as usize][c as usize]) < 1e-9f64 {
            *tau = 1.0f64;
            *bias = 0.0f64;
            *gain = 1.0f64;
            return;
        }
        let mut r_0: ::core::ffi::c_int = c + 1 as ::core::ffi::c_int;
        while r_0 < 3 as ::core::ffi::c_int {
            let mut f: ::core::ffi::c_double = A[r_0 as usize][c as usize]
                / A[c as usize][c as usize];
            let mut k_0: ::core::ffi::c_int = c;
            while k_0 < 3 as ::core::ffi::c_int {
                A[r_0 as usize][k_0 as usize] -= f * A[c as usize][k_0 as usize];
                k_0 += 1;
            }
            b[r_0 as usize] -= f * b[c as usize];
            r_0 += 1;
        }
        c += 1;
    }
    let mut x: [::core::ffi::c_double; 3] = [0.; 3];
    let mut c_0: ::core::ffi::c_int = 2 as ::core::ffi::c_int;
    while c_0 >= 0 as ::core::ffi::c_int {
        let mut s: ::core::ffi::c_double = b[c_0 as usize];
        let mut k_1: ::core::ffi::c_int = c_0 + 1 as ::core::ffi::c_int;
        while k_1 < 3 as ::core::ffi::c_int {
            s -= A[c_0 as usize][k_1 as usize] * x[k_1 as usize];
            k_1 += 1;
        }
        x[c_0 as usize] = s / A[c_0 as usize][c_0 as usize];
        c_0 -= 1;
    }
    let mut inv_tau: ::core::ffi::c_double = clampd(
        x[0 as ::core::ffi::c_int as usize],
        1.0f64 / 4.0f64,
        1.0f64 / 0.2f64,
    );
    *tau = 1.0f64 / inv_tau;
    *gain = 1.0f64;
    *bias = clampd(x[2 as ::core::ffi::c_int as usize] * *tau, -4.0f64, 4.0f64);
}
unsafe extern "C" fn af_observe(mut g: *mut GuidanceMachine, mut t: *const Telemetry) {
    let mut F: *mut AdaptiveFlareState = &raw mut (*g).adaptive_flare;
    let mut aoa: ::core::ffi::c_double = (*t).angle_of_attack;
    let mut gam: ::core::ffi::c_double = (*t).flight_path_angle;
    if (*t).ut.is_finite() as i32 == 0 || aoa.is_finite() as i32 == 0
        || gam.is_finite() as i32 == 0
    {
        return;
    }
    if (*F).has_last {
        let mut dt: ::core::ffi::c_double = (*t).ut - (*F).last_ut;
        if dt < 0.02f64 {
            return;
        }
        if dt <= 1.0f64 && (*g).terminal_aoa_cmd_ut > 0.0f64
            && (*t).ut - (*g).terminal_aoa_cmd_ut <= 1.5f64
            && (*t).dynamic_pressure > 1000.0f64 && fabs((*t).roll) < 45.0f64
            && (!(*t).has_main_gear_grounded || !(*t).main_gear_grounded)
        {
            let mut decay: ::core::ffi::c_double = exp(-dt / 5.0f64);
            let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while i < 3 as ::core::ffi::c_int {
                (*F).sxy[i as usize] *= decay;
                let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while j < 3 as ::core::ffi::c_int {
                    (*F).sxx[i as usize][j as usize] *= decay;
                    j += 1;
                }
                i += 1;
            }
            (*F).weight *= decay;
            let mut y: ::core::ffi::c_double = (aoa - (*F).last_aoa) / dt;
            let mut r: [::core::ffi::c_double; 3] = [
                (*g).terminal_aoa_cmd_last - (*F).last_aoa,
                0.0f64,
                1.0f64,
            ];
            if y.is_finite() as i32 != 0 && fabs(y) < 20.0f64 {
                let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while i_0 < 3 as ::core::ffi::c_int {
                    (*F).sxy[i_0 as usize] += dt * r[i_0 as usize] * y;
                    let mut j_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                    while j_0 < 3 as ::core::ffi::c_int {
                        (*F).sxx[i_0 as usize][j_0 as usize]
                            += dt * r[i_0 as usize] * r[j_0 as usize];
                        j_0 += 1;
                    }
                    i_0 += 1;
                }
                (*F).weight += dt;
            }
        }
    }
    (*F).last_aoa = aoa;
    (*F).last_gamma = gam;
    (*F).last_ut = (*t).ut;
    (*F).has_last = true_0 != 0;
    af_response(F, &raw mut (*F).tau, &raw mut (*F).bias, &raw mut (*F).gain);
}
unsafe extern "C" fn af_model(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut g0: ::core::ffi::c_double,
    mut aoa_max: ::core::ffi::c_double,
    mut v_td: ::core::ffi::c_double,
    mut zone_lo: ::core::ffi::c_double,
    mut zone_hi: ::core::ffi::c_double,
    mut M: *mut AfModel,
) {
    let mut F: *const AdaptiveFlareState = &raw const (*g).adaptive_flare;
    (*M).aoa_max = aoa_max;
    (*M).n = fmin(
        AF_TABLE_MAX as ::core::ffi::c_double,
        floor(aoa_max / AF_TABLE_STEP) + 1 as ::core::ffi::c_int as ::core::ffi::c_double,
    ) as ::core::ffi::c_int;
    let mut ratio: ::core::ffi::c_double = if (*g).terminal_lift_ratio > 0.0f64
        && (*g).terminal_lift_ratio.is_finite() as i32 != 0
    {
        (*g).terminal_lift_ratio
    } else {
        1.0f64
    };
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < (*M).n {
        let mut L: ::core::ffi::c_double = 0.0f64;
        let mut D: ::core::ffi::c_double = 0.0f64;
        terminal_aero_q(
            g,
            t,
            v,
            i as ::core::ffi::c_double * AF_TABLE_STEP,
            &raw mut L,
            &raw mut D,
        );
        (*M).lq[i as usize] = L * ratio;
        (*M).dq[i as usize] = D;
        i += 1;
    }
    (*M).mass = fmax((*t).mass, 1.0f64);
    (*M).g0 = g0;
    let mut V: ::core::ffi::c_double = fmax(1.0f64, (*t).true_air_speed);
    (*M).rho = if (*t).atmospheric_density.is_finite() as i32 != 0
        && (*t).atmospheric_density > 0.0f64
    {
        (*t).atmospheric_density
    } else {
        2.0f64 * fmax((*t).dynamic_pressure, 1.0f64) / (V * V)
    };
    (*M).sound = if (*t).mach.is_finite() as i32 != 0 && (*t).mach > 0.05f64 {
        V / (*t).mach
    } else {
        340.0f64
    };
    (*M).tau = if (*F).tau > 0.0f64 { (*F).tau } else { 1.0f64 };
    (*M).bias = (*F).bias;
    (*M).gain = if (*F).gain > 0.0f64 { (*F).gain } else { 1.0f64 };
    (*M).v_floor = fmax(66.0f64, 0.88f64 * v_td);
    (*M).zone_lo = zone_lo;
    (*M).zone_hi = zone_hi;
}
unsafe extern "C" fn af_aero(
    mut M: *const AfModel,
    mut aoa: ::core::ffi::c_double,
    mut lq: *mut ::core::ffi::c_double,
    mut dq: *mut ::core::ffi::c_double,
) {
    let mut u: ::core::ffi::c_double = clampd(
        aoa,
        0.0f64,
        ((*M).n - 1 as ::core::ffi::c_int) as ::core::ffi::c_double * AF_TABLE_STEP,
    ) / AF_TABLE_STEP;
    let mut i: ::core::ffi::c_int = floor(u) as ::core::ffi::c_int;
    if i >= (*M).n - 1 as ::core::ffi::c_int {
        i = (*M).n - 2 as ::core::ffi::c_int;
    }
    if i < 0 as ::core::ffi::c_int {
        i = 0 as ::core::ffi::c_int;
    }
    let mut w: ::core::ffi::c_double = u - i as ::core::ffi::c_double;
    *lq = (*M).lq[i as usize] * (1.0f64 - w)
        + (*M).lq[(i + 1 as ::core::ffi::c_int) as usize] * w;
    *dq = (*M).dq[i as usize] * (1.0f64 - w)
        + (*M).dq[(i + 1 as ::core::ffi::c_int) as usize] * w;
}
unsafe extern "C" fn af_aoa_for(
    mut M: *const AfModel,
    mut lq: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut best: ::core::ffi::c_double = 0.0f64;
    let mut bestl: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < (*M).n {
        if (*M).lq[i as usize] >= lq {
            if i == 0 as ::core::ffi::c_int {
                return 0.0f64;
            }
            let mut f: ::core::ffi::c_double = (lq
                - (*M).lq[(i - 1 as ::core::ffi::c_int) as usize])
                / fmax(
                    1e-9f64,
                    (*M).lq[i as usize] - (*M).lq[(i - 1 as ::core::ffi::c_int) as usize],
                );
            return ((i - 1 as ::core::ffi::c_int) as ::core::ffi::c_double
                + clampd(f, 0.0f64, 1.0f64)) * AF_TABLE_STEP;
        }
        if (*M).lq[i as usize] > bestl {
            bestl = (*M).lq[i as usize];
            best = i as ::core::ffi::c_double * AF_TABLE_STEP;
        }
        i += 1;
    }
    return best;
}
unsafe extern "C" fn af_profile(
    mut P: *const AfParams,
    mut g0: ::core::ffi::c_double,
    mut h: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    h = fmax(0.0f64, h);
    if h > (*P).sgs_height {
        return sqrt(
            AF_SGS_SINK * AF_SGS_SINK + 2.0f64 * (*P).arc_accel * (h - (*P).sgs_height),
        );
    }
    if h <= AF_FLOAT_HEIGHT {
        return AF_FLOAT_SINK;
    }
    return AF_FLOAT_SINK
        + (AF_SGS_SINK - AF_FLOAT_SINK) * (h - AF_FLOAT_HEIGHT)
            / fmax(0.1f64, (*P).sgs_height - AF_FLOAT_HEIGHT);
}
unsafe extern "C" fn af_law(
    mut M: *const AfModel,
    mut P: *const AfParams,
    mut s: *const AfState,
    mut dt: ::core::ffi::c_double,
) -> AfLawOut {
    let mut o: AfLawOut = AfLawOut {
        cmd: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        accel: 0.,
        need: 0.,
        on_arc: false,
    };
    let mut V: ::core::ffi::c_double = fmax((*s).V, 1.0f64);
    let mut q: ::core::ffi::c_double = 0.5f64 * (*M).rho * V * V;
    let mut m: ::core::ffi::c_double = (*M).mass;
    let mut g0: ::core::ffi::c_double = (*M).g0;
    let mut T: ::core::ffi::c_double = clampd((*M).tau, 0.3f64, 3.0f64);
    let mut model_rate: ::core::ffi::c_double = ((*M).gain * (*s).cmd + (*M).bias
        - (*s).aoa) / fmax((*M).tau, 0.1f64);
    let mut surge: ::core::ffi::c_double = if (*s).h < (*P).sgs_height + 5.0f64
        && (*s).aoa_rate.is_finite() as i32 != 0 && model_rate.is_finite() as i32 != 0
    {
        clampd(((*s).aoa_rate - model_rate) * AF_SURGE_T, -2.0f64, 2.0f64)
    } else {
        0.0f64
    };
    let mut lq: ::core::ffi::c_double = 0.0f64;
    let mut dq: ::core::ffi::c_double = 0.0f64;
    af_aero(M, (*s).aoa + surge, &raw mut lq, &raw mut dq);
    let mut a_now: ::core::ffi::c_double = lq * q / m - g0 * cos((*s).gamma);
    let mut a_lead: ::core::ffi::c_double = 0.6f64 * fmax(0.0f64, a_now);
    let mut h_l: ::core::ffi::c_double = fmax(
        0.0f64,
        (*s).h - (*s).sink * T + 0.5f64 * a_lead * T * T,
    );
    let mut s_l: ::core::ffi::c_double = (*s).sink - a_lead * T;
    let mut ref_0: ::core::ffi::c_double = af_profile(P, g0, h_l);
    let mut ref_lo: ::core::ffi::c_double = af_profile(
        P,
        g0,
        fmax(0.0f64, h_l - 0.5f64),
    );
    let mut ff: ::core::ffi::c_double = ref_0 * (ref_0 - ref_lo) / 0.5f64;
    if (*s).engaged as ::core::ffi::c_int != 0 && (*s).glide_sink.is_finite() as i32 != 0
        && ref_0 > (*s).glide_sink
    {
        ref_0 = (*s).glide_sink;
        ff = 0.0f64;
    }
    let TC: ::core::ffi::c_double = clampd(T, 0.8f64, 3.0f64) as ::core::ffi::c_double;
    let mut a_d: ::core::ffi::c_double = 0.;
    o.on_arc = (*s).engaged as ::core::ffi::c_int != 0 || (*s).h <= (*P).sgs_height
        || (*s).h < AF_MAX_ONSET && s_l >= ref_0;
    a_d = if o.on_arc as ::core::ffi::c_int != 0 {
        ff + (s_l - ref_0) / TC
    } else {
        0.0f64
    };
    if (*s).h <= (*P).sgs_height {
        let mut floating: bool = (*s).h > 5.0f64
            && (*s).sink < 0.5f64 * af_profile(P, g0, (*s).h);
        let mut floor_a: ::core::ffi::c_double = if floating as ::core::ffi::c_int != 0 {
            -0.05f64 * g0
        } else {
            0.0f64
        };
        floor_a = fmin(floor_a, -0.08f64 * g0 * clampd(surge, 0.0f64, 1.0f64));
        a_d = fmax(a_d, floor_a);
    }
    a_d = clampd(a_d, -0.12f64 * g0, 1.2f64 * g0);
    o.accel = a_d;
    let mut need: ::core::ffi::c_double = af_aoa_for(
        M,
        m * (a_d + g0 * cos((*s).gamma)) / q,
    );
    o.need = need;
    let mut cmd: ::core::ffi::c_double = (need - (*M).bias) / (*M).gain;
    if (*s).h < AF_FLOOR_START {
        let mut u: ::core::ffi::c_double = clampd(
            (AF_FLOOR_START - (*s).h) / (AF_FLOOR_START - AF_FLOOR_FULL),
            0.0f64,
            1.0f64,
        );
        u = u * u * (3.0f64 - 2.0f64 * u);
        let mut floor_aoa: ::core::ffi::c_double = fmin(
            AF_TD_PITCH * u - (*s).gamma * RAD2DEG,
            af_aoa_for(M, m * g0 / q),
        );
        cmd = fmax(cmd, (floor_aoa - (*M).bias) / (*M).gain);
    }
    cmd = clampd(cmd, 0.0f64, (*M).aoa_max + 1.0f64);
    if (*s).cmd.is_finite() as i32 != 0 && dt > 0.0f64 {
        cmd = clampd(cmd, (*s).cmd - AF_SLEW * dt, (*s).cmd + AF_SLEW * dt);
    }
    o.cmd = cmd;
    return o;
}
unsafe extern "C" fn af_predict(
    mut M: *const AfModel,
    mut P: *const AfParams,
    mut s: AfState,
) -> AfOutcome {
    let mut r: AfOutcome = AfOutcome {
        landed: 0 as ::core::ffi::c_int != 0,
        speed: 0.,
        sink: 0.,
        along: 0.,
        peak_n: 0.,
        climb: 0.,
        cost: 0.,
    };
    let DT: ::core::ffi::c_double = 0.1f64;
    let mut peak: ::core::ffi::c_double = 0.0f64;
    let mut climb: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut started: bool = false_0 != 0;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 450 as ::core::ffi::c_int {
        if s.h <= 0.0f64 {
            r.landed = true_0 != 0;
            break;
        } else {
            if s.V < 0.6f64 * (*M).v_floor {
                break;
            }
            let mut o: AfLawOut = af_law(M, P, &raw mut s, DT);
            s.aoa_rate = ::core::f32::NAN as ::core::ffi::c_double;
            if o.on_arc as ::core::ffi::c_int != 0 && !started {
                started = true_0 != 0;
                s.engaged = true_0 != 0;
                s.glide_sink = s.sink;
            }
            s.cmd = o.cmd;
            let mut V: ::core::ffi::c_double = s.V;
            let mut q: ::core::ffi::c_double = 0.5f64 * (*M).rho * V * V;
            s.aoa += ((*M).gain * s.cmd + (*M).bias - s.aoa) / (*M).tau * DT;
            s.aoa = clampd(s.aoa, -2.0f64, (*M).aoa_max + 3.0f64);
            let mut lq: ::core::ffi::c_double = 0.0f64;
            let mut dq: ::core::ffi::c_double = 0.0f64;
            af_aero(M, s.aoa, &raw mut lq, &raw mut dq);
            let mut L: ::core::ffi::c_double = lq * q;
            let mut D: ::core::ffi::c_double = dq * q;
            let mut gdot: ::core::ffi::c_double = (L / (*M).mass
                - (*M).g0 * cos(s.gamma)) / V;
            s.V += (-D / (*M).mass - (*M).g0 * sin(s.gamma)) * DT;
            s.gamma += gdot * DT;
            s.sink = -s.V * sin(s.gamma);
            s.h -= s.sink * DT;
            s.x += s.V * cos(s.gamma) * DT;
            peak = fmax(peak, L / ((*M).mass * (*M).g0));
            if started {
                climb = fmax(climb, -s.sink);
            }
            i += 1;
        }
    }
    r.speed = s.V;
    r.sink = s.sink;
    r.along = s.x;
    r.peak_n = peak;
    r.climb = if climb.is_finite() as i32 != 0 { climb } else { 0.0f64 };
    if !r.landed {
        r.cost = 1e6f64;
        return r;
    }
    let mut c: ::core::ffi::c_double = 0.0f64;
    c
        += 30.0f64
            * pow(
                fmax(0.0f64, r.sink - 1.5f64),
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
            )
            + 300.0f64
                * pow(
                    fmax(0.0f64, r.sink - 2.5f64),
                    2 as ::core::ffi::c_int as ::core::ffi::c_double,
                );
    c
        += 200.0f64
            * pow(
                fmax(0.0f64, r.climb - 0.3f64),
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
    if r.speed < (*M).v_floor {
        c
            += pow(
                (*M).v_floor - r.speed,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
    }
    if r.along < 0.0f64 {
        c
            += 2000.0f64
                + pow(
                    r.along / 20.0f64,
                    2 as ::core::ffi::c_int as ::core::ffi::c_double,
                );
    }
    if r.along < (*M).zone_lo {
        c
            += pow(
                ((*M).zone_lo - r.along) / 50.0f64,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
    }
    if r.along > (*M).zone_hi {
        c
            += pow(
                (r.along - (*M).zone_hi) / 200.0f64,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
    }
    c
        += 5.0f64
            * pow(
                fmax(0.0f64, r.peak_n - 1.8f64),
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
    r.cost = c;
    return r;
}
static mut AF_ARC: [::core::ffi::c_double; 6] = [
    0.06f64,
    0.10f64,
    0.14f64,
    0.18f64,
    0.24f64,
    0.30f64,
];
static mut AF_SGS: [::core::ffi::c_double; 3] = [10.0f64, 14.0f64, 18.0f64];
unsafe extern "C" fn af_solve(
    mut F: *mut AdaptiveFlareState,
    mut M: *const AfModel,
    mut s0: *const AfState,
    mut ut: ::core::ffi::c_double,
) {
    if (*F).plan_valid as ::core::ffi::c_int != 0
        && (*s0).engaged as ::core::ffi::c_int != 0
    {
        return;
    }
    let mut W: AfModel = *M;
    W.tau = fmin(3.0f64, 1.3f64 * (*M).tau);
    W.gain = fmax(0.4f64, 0.8f64 * (*M).gain);
    W.bias = (*M).bias - 0.3f64;
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut bp: AfParams = AfParams {
        arc_accel: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        sgs_height: 0.,
    };
    let mut bo: AfOutcome = AfOutcome {
        landed: 0 as ::core::ffi::c_int != 0,
        speed: 0.,
        sink: 0.,
        along: 0.,
        peak_n: 0.,
        climb: 0.,
        cost: 0.,
    };
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[::core::ffi::c_double; 6]>() as usize)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
    {
        let mut j: size_t = 0 as size_t;
        while j
            < (::core::mem::size_of::<[::core::ffi::c_double; 3]>() as usize)
                .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
        {
            let mut P: AfParams = AfParams {
                arc_accel: AF_ARC[i as usize] * (*M).g0,
                sgs_height: AF_SGS[j as usize],
            };
            let mut o: AfOutcome = af_predict(M, &raw mut P, *s0);
            let mut w: AfOutcome = af_predict(&raw mut W, &raw mut P, *s0);
            if w.cost > o.cost {
                o = w;
            }
            let mut c: ::core::ffi::c_double = o.cost;
            if (*F).plan_valid as ::core::ffi::c_int != 0
                && fabs(P.arc_accel - (*F).arc_accel) < 1e-6f64
                && fabs(P.sgs_height - (*F).sgs_height) < 1e-6f64
            {
                c = c * 0.9f64 - 0.05f64;
            }
            if c < best {
                best = c;
                bp = P;
                bo = o;
            }
            j = j.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    if best.is_finite() as i32 == 0 {
        return;
    }
    (*F).plan_valid = true_0 != 0;
    (*F).arc_accel = bp.arc_accel;
    (*F).sgs_height = bp.sgs_height;
    (*F).cost = bo.cost;
    (*F).pred_speed = bo.speed;
    (*F).pred_sink = bo.sink;
    (*F).pred_along = bo.along;
    (*F).pred_peak_n = bo.peak_n;
    (*F).pred_climb = bo.climb;
    (*F).plan_ut = ut;
}
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
