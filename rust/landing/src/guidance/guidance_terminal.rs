use crate::sys::*;
extern "C" {
    fn entry_exec_accept_taem_handoff(
        exec: *mut EntryExecutive,
        observation: *const EntryExecObservation,
    ) -> bool;
    fn entry_exec_force_taem_handoff(
        exec: *mut EntryExecutive,
        observation: *const EntryExecObservation,
    ) -> bool;
    fn taem_exec_owns_vehicle(exec: *const TaemExecutive) -> bool;
    fn taem_exec_deliver_low_energy(
        exec: *mut TaemExecutive,
        observation: *const TaemExecObservation,
    ) -> bool;
    fn taem_exec_evaluate_terminal_contract(
        contract: *const TaemTerminalContract,
    ) -> TaemTerminalEvaluation;
    fn taem_terminal_block_reason_string(
        reason: TaemTerminalBlockReason,
    ) -> *const ::core::ffi::c_char;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn round(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn decision_runway_end_path_score(
        guidance: *const GuidanceMachine,
        telemetry: *const Telemetry,
        planet: *const PlanetModel,
        configuration: *const LandingConfiguration,
        site: *const LandingSite,
    ) -> ::core::ffi::c_double;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_guidance_start_altitude(
        planet: *const PlanetModel,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
    fn entry_taem_handoff_altitude_bounds(
        settings: *const GuidanceSettings,
        minimum: *mut ::core::ffi::c_double,
        maximum: *mut ::core::ffi::c_double,
    );
    fn entry_taem_range_target(
        planet: *const PlanetModel,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
    fn entry_low_q_protective_aoa_floor(
        dynamic_pressure: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn entry_terminal_demand(
        latitude: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
        range: ::core::ffi::c_double,
        horizontal_speed: ::core::ffi::c_double,
        course_error: ::core::ffi::c_double,
        vertical_speed: ::core::ffi::c_double,
        true_air_speed: ::core::ffi::c_double,
        entry_reference_speed: ::core::ffi::c_double,
        planet: *const PlanetModel,
        vehicle: *const VehicleProfile,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
    ) -> EntryTerminalDemand;
    fn v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vector3;
    fn vadd(a: Vector3, b: Vector3) -> Vector3;
    fn vscale(a: Vector3, s: ::core::ffi::c_double) -> Vector3;
    fn vcross(a: Vector3, b: Vector3) -> Vector3;
    fn vnorm(a: Vector3, fallback: Vector3) -> Vector3;
    fn surface_course(
        pos: Vector3,
        vel: Vector3,
        omega: Vector3,
        north: Vector3,
        fallback: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn runway_reciprocal_site(
        primary: *const LandingSite,
        radius: ::core::ffi::c_double,
    ) -> LandingSite;
    fn telemetry_reframe_runway(
        telemetry: *mut Telemetry,
        site: *const LandingSite,
        radius: ::core::ffi::c_double,
    );
    fn robust_pid_reset(p: *mut RobustPID);
    fn burn_fraction(
        elapsed: ::core::ffi::c_double,
        remaining: ::core::ffi::c_double,
        max_accel: ::core::ffi::c_double,
        ramp: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn guidance_command_init(c: *mut GuidanceCommand);
    fn phase_string(p: GuidancePhase) -> *const ::core::ffi::c_char;
    fn trajectory_init(t: *mut Trajectory);
    fn trajectory_clear(t: *mut Trajectory);
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
    fn planet_surface_gravity(p: *const PlanetModel) -> ::core::ffi::c_double;
    fn planet_atmospheric_speed_of_sound(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn guidance_abort(g: *mut GuidanceMachine);
    fn guidance_result_clear(r: *mut GuidanceResult);
    fn terminal_runway_line_path_guidance(
        t: *const Telemetry,
        site: *const LandingSite,
        s: *const GuidanceSettings,
        v: *const VehicleProfile,
        gravity: ::core::ffi::c_double,
        course: ::core::ffi::c_double,
    ) -> HACGuidance;
    fn result_make(
        phase: GuidancePhase,
        c: GuidanceCommand,
        status: *const ::core::ffi::c_char,
        warning: *const ::core::ffi::c_char,
    ) -> GuidanceResult;
    fn reset_limiters(g: *mut GuidanceMachine);
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
    fn entry_capture(
        t: *const Telemetry,
        state: *const VehicleState,
        v: *const VehicleProfile,
    ) -> GuidanceCommand;
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
    fn terminal_glide_initialize(
        g: *mut GuidanceMachine,
        v: *const VehicleProfile,
        s: *const GuidanceSettings,
    );
    fn terminal_energy_observe(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        p: *const PlanetModel,
    );
    fn taem_exec_enter(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        s: *const GuidanceSettings,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        checkpoint_resume: bool,
    ) -> bool;
    fn taem_exec_sync_with_contract(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        s: *const GuidanceSettings,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        terminal_contract: *const TaemTerminalContract,
        has_final_approach_override: bool,
        final_approach_override: bool,
    ) -> bool;
    fn terminal_fpa_force_aoa(
        t: *const Telemetry,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
        reference_fpa: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn taem_guidance_native(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        model: *const TerminalModel,
        dt: ::core::ffi::c_double,
    ) -> GuidanceResult;
    fn terminal_observe_landing_aero(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        v: *const VehicleProfile,
        gravity: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    );
    fn terminal_store_preflare_plan(
        g: *mut GuidanceMachine,
        p: *const TerminalPreflarePlan,
    );
    fn terminal_set_stage(
        g: *mut GuidanceMachine,
        stage: TerminalVerticalStage,
        ut: ::core::ffi::c_double,
    );
    fn terminal_outer_gate(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        out_plan: *mut TerminalPreflarePlan,
    ) -> bool;
    fn terminal_delivery_contract(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        cfg: *const LandingConfiguration,
        plan: *const TerminalPreflarePlan,
    ) -> TaemTerminalContract;
    fn terminal_final_conditioning_reachable(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        energy_out: *mut EnergyPathEnvelope,
        end_speed_out: *mut ::core::ffi::c_double,
        slope_out: *mut ::core::ffi::c_double,
    ) -> bool;
    fn terminal_runway_contact_position(
        t: *const Telemetry,
        cfg: *const LandingConfiguration,
    ) -> bool;
    fn touchdown(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        cfg: *const LandingConfiguration,
        ref_0: *const Trajectory,
    ) -> GuidanceResult;
    fn terminal_abort(
        g: *mut GuidanceMachine,
        reason: *const ::core::ffi::c_char,
    ) -> GuidanceResult;
    fn terminal_approach_sequence(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        ref_0: *const Trajectory,
        dt: ::core::ffi::c_double,
    ) -> GuidanceResult;
    fn control_recovery_needed(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        s: *const GuidanceSettings,
        v: *const VehicleProfile,
        dt: ::core::ffi::c_double,
    ) -> bool;
    fn terminal_entry_recovery_available(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        v: *const VehicleProfile,
        s: *const GuidanceSettings,
    ) -> bool;
    fn terminal_invalidate_frozen_path(g: *mut GuidanceMachine);
    fn terminal_return_to_entry(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        course: ::core::ffi::c_double,
        p: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        dt: ::core::ffi::c_double,
    ) -> GuidanceResult;
    fn mm305_plan(
        model: *const TerminalModel,
        request: *const Mm305PlanRequest,
    ) -> Mm305PlanResult;
    fn guidance_mm305_admission_request(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        cfg: *const LandingConfiguration,
        model: *const TerminalModel,
    ) -> Mm305PlanRequest;
    fn guidance_mm305_admission_accept(
        g: *mut GuidanceMachine,
        result: *const Mm305PlanResult,
    );
    fn guidance_mm305_admission_qualified(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        physically_admissible: bool,
        model_snapshot_id: uint64_t,
    ) -> bool;
    fn guidance_mm305_revalidate_admission(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        cfg: *const LandingConfiguration,
        model: *const TerminalModel,
    ) -> bool;
    fn guidance_mm305_adopt_admission_route(
        g: *mut GuidanceMachine,
        cfg: *const LandingConfiguration,
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
pub struct EntryExecObservation {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub checkpoint_restart: bool,
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
pub struct TaemExecObservation {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub checkpoint_restart: bool,
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
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
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
pub type TaemPlanStatus = ::core::ffi::c_uint;
pub const TAEM_PLAN_QUALIFIED: TaemPlanStatus = 3;
pub const TAEM_PLAN_SEARCH_EXHAUSTED: TaemPlanStatus = 2;
pub const TAEM_PLAN_UNQUALIFIED: TaemPlanStatus = 1;
pub const TAEM_PLAN_INFEASIBLE: TaemPlanStatus = 0;
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
pub struct Mm305PlanResult {
    pub valid: bool,
    pub found: bool,
    pub request_ut: ::core::ffi::c_double,
    pub initial_state: TerminalDynamicState,
    pub model_snapshot_id: uint64_t,
    pub candidate: TaemFixedHacCandidate,
    pub solve_wall_s: ::core::ffi::c_double,
    pub scale_mach: ::core::ffi::c_double,
    pub lift_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
    pub diagnostic: [::core::ffi::c_char; 768],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFixedHacCandidate {
    pub status: TaemPlanStatus,
    pub side: ::core::ffi::c_double,
    pub runway_end: ::core::ffi::c_int,
    pub route_built: bool,
    pub final_tail_qualified: bool,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub quality_score: ::core::ffi::c_double,
    pub route: TaemRoute,
    pub replay: TerminalSolverResult,
    pub reason: *const ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalSolverResult {
    pub status: TerminalSolverStatus,
    pub path_constraints_ok: bool,
    pub final_interface_qualified: bool,
    pub final_state: TerminalDynamicState,
    pub final_geometry: TaemGeometryState,
    pub elapsed_s: ::core::ffi::c_double,
    pub minimum_speed_mps: ::core::ffi::c_double,
    pub maximum_dynamic_pressure_pa: ::core::ffi::c_double,
    pub maximum_load_g: ::core::ffi::c_double,
    pub maximum_cross_track_m: ::core::ffi::c_double,
    pub maximum_course_error_deg: ::core::ffi::c_double,
    pub final_route_course_error_deg: ::core::ffi::c_double,
    pub maximum_altitude_error_m: ::core::ffi::c_double,
    pub maximum_lateral_authority_shortfall_mps2: ::core::ffi::c_double,
    pub maximum_vertical_authority_shortfall_mps2: ::core::ffi::c_double,
    pub turn_burden_integral_s: ::core::ffi::c_double,
    pub maximum_turn_authority_fraction: ::core::ffi::c_double,
    pub maximum_commanded_aoa_deg: ::core::ffi::c_double,
    pub bank_target_reversals: ::core::ffi::c_uint,
    pub failure_cross_track_m: ::core::ffi::c_double,
    pub failure_course_error_deg: ::core::ffi::c_double,
    pub failure_required_lateral_accel_mps2: ::core::ffi::c_double,
    pub failure_available_lateral_accel_mps2: ::core::ffi::c_double,
    pub failure_required_vertical_lift_mps2: ::core::ffi::c_double,
    pub failure_delivered_vertical_lift_mps2: ::core::ffi::c_double,
    pub failure_target_altitude_m: ::core::ffi::c_double,
    pub failure_target_flight_path_angle_deg: ::core::ffi::c_double,
    pub failure_route_index: size_t,
    pub energy_start_j_kg: ::core::ffi::c_double,
    pub energy_end_j_kg: ::core::ffi::c_double,
    pub drag_work_j_kg: ::core::ffi::c_double,
    pub energy_closure_residual_j_kg: ::core::ffi::c_double,
    pub final_route_index: size_t,
    pub final_target_altitude_m: ::core::ffi::c_double,
    pub final_target_flight_path_angle_deg: ::core::ffi::c_double,
    pub reason: *const ::core::ffi::c_char,
}
pub type TerminalSolverStatus = ::core::ffi::c_uint;
pub const TERMINAL_SOLVER_INVALID_INPUT: TerminalSolverStatus = 4;
pub const TERMINAL_SOLVER_SEARCH_EXHAUSTED: TerminalSolverStatus = 3;
pub const TERMINAL_SOLVER_UNQUALIFIED: TerminalSolverStatus = 2;
pub const TERMINAL_SOLVER_INFEASIBLE: TerminalSolverStatus = 1;
pub const TERMINAL_SOLVER_QUALIFIED: TerminalSolverStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Mm305PlanRequest {
    pub valid: bool,
    pub request_ut: ::core::ffi::c_double,
    pub model_snapshot_id: uint64_t,
    pub state: TerminalDynamicState,
    pub final_guidance: GuidanceMachine,
    pub hac_radius_m: ::core::ffi::c_double,
    pub final_approach_distance_m: ::core::ffi::c_double,
    pub search_both_ends: bool,
    pub upstream_end: ::core::ffi::c_int,
    pub restrict_side: bool,
    pub side: ::core::ffi::c_double,
    pub lift_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
    pub scale_mach: ::core::ffi::c_double,
    pub seed_route_valid: bool,
    pub seed_route: TaemRoute,
    pub seed_runway_end: ::core::ffi::c_int,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const TERMINAL_BACKSTOP_HEIGHT_M: ::core::ffi::c_double = 300.0f64;
pub const TERMINAL_BACKSTOP_TIME_S: ::core::ffi::c_double = 12.0f64;
pub const TERMINAL_BACKSTOP_COURSE_DEG: ::core::ffi::c_double = 15.0f64;
pub const DEORBIT_PERIAPSIS_TAPER_SPAN_M: ::core::ffi::c_double = 1500.0f64;
unsafe extern "C" fn mm305_physical_handoff_due(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> bool {
    if t.is_null() || p.is_null() || cfg.is_null()
        || (*t).mean_altitude.is_finite() as i32 == 0
        || (*t).vertical_speed.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0 || (*t).vertical_speed >= 0.0f64
        || (*t).true_air_speed <= 0.0f64
        || (*t).mean_altitude <= (*cfg).site.altitude + 100.0f64
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
    } else if sound.is_finite() as i32 != 0 && sound > 0.0f64 {
        (*t).true_air_speed / sound
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut sts_delivery: bool = (*cfg).guidance.sts_trajectory_delivery
        as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    let mut centre: bool = sts_delivery as ::core::ffi::c_int != 0
        || (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0;
    return mach.is_finite() as i32 != 0
        && (*t).mean_altitude <= (*cfg).guidance.mm305_max_altitude
        && mach
            <= (*cfg).guidance.mm305_target_mach
                + (if centre as ::core::ffi::c_int != 0 {
                    0.0f64
                } else {
                    (*cfg).guidance.mm305_mach_half_width
                });
}
unsafe extern "C" fn terminal_final_test_guidance(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    if (*g).final_approach_captured as ::core::ffi::c_int != 0
        && (*g).taem_exec.taem_complete as ::core::ffi::c_int != 0
    {
        if !(*g).terminal_pull_latch {
            let mut fresh_plan: TerminalPreflarePlan = TerminalPreflarePlan {
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
            if terminal_outer_gate(g, t, course, p, aero, cfg, &raw mut fresh_plan)
                as ::core::ffi::c_int != 0
                && fresh_plan.feasible as ::core::ffi::c_int != 0
            {
                terminal_store_preflare_plan(g, &raw mut fresh_plan);
            }
        }
        let mut ref_0: Trajectory = Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        };
        trajectory_init(&raw mut ref_0);
        let mut result: GuidanceResult = terminal_approach_sequence(
            g,
            t,
            course,
            p,
            aero,
            cfg,
            &raw mut ref_0,
            dt,
        );
        trajectory_clear(&raw mut ref_0);
        return result;
    }
    let mut approach_plan: TerminalPreflarePlan = TerminalPreflarePlan {
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
        &raw mut approach_plan,
    );
    let mut contract: TaemTerminalContract = terminal_delivery_contract(
        g,
        t,
        course,
        p,
        cfg,
        &raw mut approach_plan,
    );
    let mut evaluation: TaemTerminalEvaluation = taem_exec_evaluate_terminal_contract(
        &raw mut contract,
    );
    if !approach || !evaluation.valid || !evaluation.feasible {
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
        let mut projected_end_speed: ::core::ffi::c_double = ::core::f32::NAN
            as ::core::ffi::c_double;
        let mut conditioning_slope: ::core::ffi::c_double = ::core::f32::NAN
            as ::core::ffi::c_double;
        if !terminal_final_conditioning_reachable(
            g,
            t,
            course,
            p,
            aero,
            cfg,
            &raw mut conditioning_energy,
            &raw mut projected_end_speed,
            &raw mut conditioning_slope,
        ) {
            let mut reason: [::core::ffi::c_char; 320] = [0; 320];
            snprintf(
                &raw mut reason as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 320]>() as size_t,
                b"Post-HAC final alignment cannot meet the priced delivery contract: approach=%d preflare=%s terminal=%s.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if approach as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if !approach_plan.reject_reason.is_null() {
                    approach_plan.reject_reason
                } else {
                    b"none\0" as *const u8 as *const ::core::ffi::c_char
                },
                taem_terminal_block_reason_string(evaluation.block_reason),
            );
            return terminal_abort(g, &raw mut reason as *mut ::core::ffi::c_char);
        }
        let mut line: HACGuidance = terminal_runway_line_path_guidance(
            t,
            &raw const (*cfg).site,
            s,
            v,
            planet_surface_gravity(p),
            course,
        );
        let mut conditioning_aoa: ::core::ffi::c_double = terminal_fpa_force_aoa(
            t,
            p,
            aero,
            v,
            -conditioning_slope,
        );
        let mut command: GuidanceCommand = atmospheric(
            t,
            line.heading,
            line.bank,
            v,
            0.0f64,
            false_0 != 0,
            PROFILE_TAEM,
        );
        command.heading_control_enabled = true_0 != 0;
        command.has_target_aoa = true_0 != 0;
        command.target_aoa = conditioning_aoa;
        command.target_pitch = (*t).flight_path_angle + conditioning_aoa;
        (*g).gear_command_latched = (*g).gear_command_latched as ::core::ffi::c_int != 0
            || (*t).gear as ::core::ffi::c_int != 0
            || (*t).radar_altitude <= (*s).gear_deployment_altitude;
        command.gear = (*t).gear as ::core::ffi::c_int != 0
            || (*g).gear_command_latched as ::core::ffi::c_int != 0;
        terminal_speedbrake_closed(g);
        let mut result_0: GuidanceResult = stabilized(
            g,
            result_make(
                PHASE_TAEM,
                command,
                ::core::ptr::null::<::core::ffi::c_char>(),
                ::core::ptr::null::<::core::ffi::c_char>(),
            ),
            t,
            v,
            s,
            dt,
        );
        (*g).phase = PHASE_TAEM;
        snprintf(
            &raw mut result_0.status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Post-HAC final alignment energy conditioning: V %.1f m/s, projected %.1f m/s at flare boundary, minimum %.1f m/s, slope %.1f deg, AoA %.1f deg, energy margin %+.0f J/kg.\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).true_air_speed,
            projected_end_speed,
            (*v).minimum_safe_speed,
            conditioning_slope,
            conditioning_aoa,
            conditioning_energy.energy.margin,
        );
        return result_0;
    }
    taem_exec_sync_with_contract(
        g,
        t,
        s,
        p,
        aero,
        cfg,
        &raw mut contract,
        false_0 != 0,
        false_0 != 0,
    );
    if !(*g).final_approach_captured && (*t).runway_along_track < 0.0f64
        && (*g).taem_exec.phase as ::core::ffi::c_uint
            == TAEM_PHASE_FINAL_INTERCEPT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        taem_exec_sync_with_contract(
            g,
            t,
            s,
            p,
            aero,
            cfg,
            &raw mut contract,
            true_0 != 0,
            true_0 != 0,
        );
        if (*g).taem_exec.taem_complete {
            (*g).final_approach_captured = true_0 != 0;
            if approach_plan.degraded_floor {
                (*g).terminal_final_low_energy = true_0 != 0;
            }
            terminal_store_preflare_plan(g, &raw mut approach_plan);
            terminal_set_stage(g, TERMINAL_OUTER_FINAL, (*t).ut);
            robust_pid_reset(&raw mut (*g).final_altitude_pid);
            robust_pid_reset(&raw mut (*g).speed_pid);
            robust_pid_reset(&raw mut (*g).flare_sink_pid);
        }
    }
    let mut ref_1: Trajectory = Trajectory {
        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
        count: 0,
        capacity: 0,
    };
    trajectory_init(&raw mut ref_1);
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
    if (*g).final_approach_captured as ::core::ffi::c_int != 0
        && (*g).taem_exec.taem_complete as ::core::ffi::c_int != 0
    {
        result_1 = terminal_approach_sequence(
            g,
            t,
            course,
            p,
            aero,
            cfg,
            &raw mut ref_1,
            dt,
        );
    } else {
        let mut line_0: HACGuidance = terminal_runway_line_path_guidance(
            t,
            &raw const (*cfg).site,
            s,
            v,
            planet_surface_gravity(p),
            course,
        );
        let mut command_0: GuidanceCommand = atmospheric(
            t,
            line_0.heading,
            line_0.bank,
            v,
            0.0f64,
            true_0 != 0,
            PROFILE_TAEM,
        );
        command_0.heading_control_enabled = true_0 != 0;
        (*g).phase = PHASE_TAEM;
        result_1 = stabilized(
            g,
            result_make(
                PHASE_TAEM,
                command_0,
                b"Post-HAC final alignment: runway-line capture and TAEM final-intercept admission.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                ::core::ptr::null::<::core::ffi::c_char>(),
            ),
            t,
            v,
            s,
            dt,
        );
    }
    trajectory_clear(&raw mut ref_1);
    return result_1;
}
unsafe extern "C" fn terminal_guidance(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut dt: ::core::ffi::c_double,
    mut terminal_model: *const TerminalModel,
) -> GuidanceResult {
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    terminal_observe_landing_aero(g, t, v, planet_surface_gravity(p), dt);
    if (*g).terminal_final_test_mode {
        (*g).terminal_glide_mode = false_0 != 0;
        let mut result: GuidanceResult = terminal_final_test_guidance(
            g,
            t,
            course,
            p,
            aero,
            cfg,
            dt,
        );
        (*g).terminal_glide_mode = true_0 != 0;
        return result;
    }
    let mut reported_landed: bool = strcasecmp(
        &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0;
    if strcmp(
        &raw const (*v).model_id as *const ::core::ffi::c_char,
        b"STS\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0 && (*g).final_approach_captured as ::core::ffi::c_int != 0
        && (*t).gear as ::core::ffi::c_int != 0
        && (reported_landed as ::core::ffi::c_int != 0
            || (*t).has_main_gear_grounded as ::core::ffi::c_int != 0
                && (*t).main_gear_grounded as ::core::ffi::c_int != 0)
    {
        if !terminal_runway_contact_position(t, cfg) {
            return terminal_abort(
                g,
                b"Touchdown occurred outside the runway contact envelope for the selected end.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        (*g).ground_contact_latched = true_0 != 0;
        (*g).gear_command_latched = true_0 != 0;
        (*g).attitude_recovery = false_0 != 0;
        terminal_set_stage(g, TERMINAL_GROUND, (*t).ut);
    }
    if (*g).final_approach_captured as ::core::ffi::c_int != 0
        && ((*g).phase as ::core::ffi::c_uint
            == PHASE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).phase as ::core::ffi::c_uint
                == PHASE_COMPLETE as ::core::ffi::c_int as ::core::ffi::c_uint
            || (*g).ground_contact_latched as ::core::ffi::c_int != 0)
    {
        if reported_landed as ::core::ffi::c_int != 0
            && !terminal_runway_contact_position(t, cfg)
        {
            return terminal_abort(
                g,
                b"Touchdown occurred outside the runway contact envelope for the selected end.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        if reported_landed as ::core::ffi::c_int != 0 && !(*t).gear {
            return terminal_abort(
                g,
                b"Touchdown occurred without confirmed landing-gear deployment.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        let mut reference: Trajectory = Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        };
        trajectory_init(&raw mut reference);
        let mut result_0: GuidanceResult = touchdown(g, t, cfg, &raw mut reference);
        trajectory_clear(&raw mut reference);
        return result_0;
    }
    if (*g).final_approach_captured as ::core::ffi::c_int != 0 && !reported_landed
        && (*cfg).site.runway_length > 0.0f64
        && (*t).runway_along_track > (*cfg).site.runway_length
    {
        return terminal_abort(
            g,
            b"Long landing: the vehicle passed the runway end still airborne.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if (*g).final_approach_captured as ::core::ffi::c_int != 0 && !reported_landed
        && (*t).radar_altitude.is_finite() as i32 != 0 && (*t).radar_altitude < 0.0f64
        && !terminal_runway_contact_position(t, cfg)
    {
        let mut reason: [::core::ffi::c_char; 256] = [0; 256];
        snprintf(
            &raw mut reason as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"Ground impact off the runway surface (runway along %.0f m, cross %.0f m, sink %.1f m/s).\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).runway_along_track,
            (*t).runway_cross_track,
            -(*t).vertical_speed,
        );
        return terminal_abort(g, &raw mut reason as *mut ::core::ffi::c_char);
    }
    let mut recovery_was_active: bool = (*g).attitude_recovery;
    if control_recovery_needed(g, t, s, v, dt) {
        if !recovery_was_active
            && ((*g).terminal_path_committed as ::core::ffi::c_int != 0
                || (*g).hac_side_selected as ::core::ffi::c_int != 0)
        {
            terminal_invalidate_frozen_path(g);
        }
        if (*g).recovery_duration >= 15.0f64 {
            return terminal_abort(
                g,
                b"Control departure: attitude recovery exceeded 15 s without stabilizing.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        if (*t).radar_altitude < fmax(300.0f64, -(*t).vertical_speed * 8.0f64) {
            let mut reason_0: [::core::ffi::c_char; 256] = [0; 256];
            snprintf(
                &raw mut reason_0 as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"Control departure: insufficient height remains to complete attitude recovery (radar %.0f m, runway along %.0f m, cross %.0f m, phase %s).\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).radar_altitude,
                (*t).runway_along_track,
                (*t).runway_cross_track,
                phase_string((*g).phase),
            );
            return terminal_abort(g, &raw mut reason_0 as *mut ::core::ffi::c_char);
        }
        (*g).phase = PHASE_ATTITUDE_RECOVERY;
        let mut command: GuidanceCommand = atmospheric(
            t,
            (*g).recovery_heading,
            0.0f64,
            v,
            0.0f64,
            false_0 != 0,
            PROFILE_RECOVERY,
        );
        command.heading_control_enabled = false_0 != 0;
        let mut aoa: ::core::ffi::c_double = if (*g).recovery_aoa > 0.0f64 {
            (*g).recovery_aoa
        } else {
            clampd((*v).entry_angle_of_attack, 10.0f64, 18.0f64)
        };
        command.has_target_aoa = true_0 != 0;
        command.target_aoa = aoa;
        command.target_pitch = clampd((*t).flight_path_angle + aoa, -12.0f64, 25.0f64);
        return stabilized(
            g,
            result_make(
                (*g).phase,
                command,
                b"Attitude recovery: unloading and leveling wings; terminal progression inhibited.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                b"Sustained saturation/attitude error or excessive roll rate.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            ),
            t,
            v,
            s,
            dt,
        );
    }
    if recovery_was_active as ::core::ffi::c_int != 0 && !(*g).attitude_recovery {
        if terminal_entry_recovery_available(g, t, v, s) {
            return terminal_return_to_entry(g, t, state, course, p, aero, cfg, dt);
        }
        terminal_invalidate_frozen_path(g);
    }
    terminal_energy_observe(g, t, p);
    if !(*g).terminal_region_entered && !(*g).final_approach_captured {
        (*g).phase = PHASE_ENTRY_ENERGY;
        let mut entry: GuidanceResult = entry_program_guidance(
            g,
            t,
            state,
            course,
            p,
            aero,
            cfg,
            dt,
        );
        let mut handoff_due: bool = mm305_physical_handoff_due(t, p, cfg);
        let mut model_available: bool = !terminal_model.is_null()
            && (*terminal_model).replay_validated as ::core::ffi::c_int != 0;
        let mut forced_handoff: bool = handoff_due as ::core::ffi::c_int != 0
            && (model_available as ::core::ffi::c_int != 0
                || (*g).diagnostic_shadow as ::core::ffi::c_int != 0
                    && (*g).diagnostic_stop_at_taem as ::core::ffi::c_int != 0);
        let mut physical_handoff: bool = forced_handoff as ::core::ffi::c_int != 0
            || (*g).entry_exec.phase as ::core::ffi::c_uint
                == ENTRY_PHASE_TRANSITION as ::core::ffi::c_int as ::core::ffi::c_uint
                && (*g).entry_control_plan_ut == (*t).ut
                && (*g).entry_control_terminal_ready as ::core::ffi::c_int != 0;
        let mut admission_route_qualified: bool = false_0 != 0;
        if physical_handoff as ::core::ffi::c_int != 0 && !(*g).diagnostic_shadow {
            let mut snapshot: uint64_t = if !terminal_model.is_null()
                && (*terminal_model).replay_validated as ::core::ffi::c_int != 0
            {
                (*terminal_model).snapshot_id
            } else {
                0 as uint64_t
            };
            if snapshot != 0 as uint64_t {
                admission_route_qualified = guidance_mm305_admission_qualified(
                    g,
                    t,
                    true_0 != 0,
                    snapshot,
                );
                if (*g).mm305_admission_valid as ::core::ffi::c_int != 0
                    && (*g).mm305_admission_found as ::core::ffi::c_int != 0
                    && (*g).mm305_admission_result_ut != (*t).ut
                {
                    admission_route_qualified = guidance_mm305_revalidate_admission(
                        g,
                        t,
                        state,
                        cfg,
                        terminal_model,
                    );
                }
                if !admission_route_qualified
                    && (*g).mm305_admission_needed as ::core::ffi::c_int != 0
                    && !(*g).mm305_async_planning
                {
                    let mut request: Mm305PlanRequest = guidance_mm305_admission_request(
                        g,
                        t,
                        state,
                        cfg,
                        terminal_model,
                    );
                    if request.valid {
                        let mut result_1: Mm305PlanResult = mm305_plan(
                            terminal_model,
                            &raw mut request,
                        );
                        guidance_mm305_admission_accept(g, &raw mut result_1);
                    } else {
                        (*g).mm305_admission_needed = false_0 != 0;
                    }
                    admission_route_qualified = guidance_mm305_admission_qualified(
                        g,
                        t,
                        true_0 != 0,
                        snapshot,
                    );
                }
            }
        } else if !physical_handoff && !(*g).diagnostic_shadow {
            guidance_mm305_admission_qualified(g, t, false_0 != 0, 0 as uint64_t);
        }
        // Under the HAC law MM305 flies to the HAC energy reference, so it is admitted on
        // that requirement: the MM304 terminal score of the live state, energy inside
        // MM305's window [0, band] and altitude inside the MM305 band.
        if physical_handoff as ::core::ffi::c_int != 0 && !admission_route_qualified
            && !(*g).diagnostic_shadow && (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0
            && model_available as ::core::ffi::c_int != 0
            && (*g).mm304.live_terminal.admissible && (*g).mm304.live_terminal.ut == (*t).ut
        {
            admission_route_qualified = true_0 != 0;
            fprintf(
                stderr,
                b"MM305 admitted on HAC energy: UT %.1f M %.2f h %.0f m along %.1f km cross %.1f km heading error %+.1f deg, energy %+.0f m in [0, %.0f]\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*t).mach,
                (*t).mean_altitude,
                (*g).mm304.live_terminal.along / 1000.0f64,
                (*g).mm304.live_terminal.cross / 1000.0f64,
                (*g).mm304.live_terminal.heading_error,
                (*g).mm304.live_terminal.energy_error,
                (*g).mm304.live_terminal.band,
            );
        }
        let mut diagnostic_boundary: bool = (*g).diagnostic_shadow as ::core::ffi::c_int
            != 0 && (*g).diagnostic_stop_at_taem as ::core::ffi::c_int != 0;
        if !physical_handoff || !diagnostic_boundary && !admission_route_qualified {
            if handoff_due as ::core::ffi::c_int != 0 && !model_available
                && !(*g).diagnostic_shadow
            {
                entry.has_warning = true_0 != 0;
                snprintf(
                    &raw mut entry.warning as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                    b"MM305 physical boundary reached, but no validated native model is available; retaining MM304.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
            } else if physical_handoff as ::core::ffi::c_int != 0
                && !admission_route_qualified
            {
                entry.has_warning = true_0 != 0;
                snprintf(
                    &raw mut entry.warning as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                    b"MM305 acquisition/energy envelope unqualified: no native trajectory proves Final delivery. Retaining MM304; preserve attainable range and lift.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
            }
            let mut low: ::core::ffi::c_double = 0.0f64;
            let mut high: ::core::ffi::c_double = 0.0f64;
            entry_taem_handoff_altitude_bounds(s, &raw mut low, &raw mut high);
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
            let mut minimum_mach: ::core::ffi::c_double = (*s).mm305_target_mach
                - (*s).mm305_mach_half_width;
            let mut handoff_lost: bool = (*t).true_air_speed < (*v).minimum_safe_speed
                || (*t).mean_altitude < (*cfg).site.altitude + 100.0f64;
            if handoff_lost {
                entry.has_warning = true_0 != 0;
                snprintf(
                    &raw mut entry.warning as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                    b"MM304 recovery margin exhausted; continuing the best available entry command.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
            }
            return entry;
        }
        let mut before_handoff: GuidanceMachine = *g;
        let mut handoff_observation: EntryExecObservation = EntryExecObservation {
            ut: (*t).ut,
            relative_velocity: (*t).true_air_speed,
            altitude: (*t).mean_altitude,
            dynamic_pressure: (*t).dynamic_pressure,
            checkpoint_restart: false,
        };
        let mut accepted: bool = if forced_handoff as ::core::ffi::c_int != 0 {
            entry_exec_force_taem_handoff(
                &raw mut (*g).entry_exec,
                &raw mut handoff_observation,
            ) as ::core::ffi::c_int
        } else {
            entry_exec_accept_taem_handoff(
                &raw mut (*g).entry_exec,
                &raw mut handoff_observation,
            ) as ::core::ffi::c_int
        } != 0;
        if !accepted {
            entry.has_warning = true_0 != 0;
            snprintf(
                &raw mut entry.warning as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                b"MM304 executive could not complete the MM305 ownership handoff.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
            return entry;
        }
        (*g).taem_interface_captured = true_0 != 0;
        let mut loss: ::core::ffi::c_double = (*g).terminal_energy_loss_accel_ema;
        let mut speed_loss: ::core::ffi::c_double = (*g).terminal_speed_loss_accel_ema;
        let mut rehearsal: bool = (*g).terminal_rehearsal_mode;
        terminal_glide_initialize(g, v, s);
        (*g).terminal_energy_loss_accel_ema = loss;
        (*g).terminal_speed_loss_accel_ema = speed_loss;
        (*g).terminal_rehearsal_mode = rehearsal;
        (*g).terminal_region_entered = true_0 != 0;
        (*g).terminal_test_capture_active = false_0 != 0;
        (*g).taem_safety_handoff = false_0 != 0;
        (*g).terminal_path_kind = TERMINAL_PATH_NONE;
        (*g).terminal_path_committed = false_0 != 0;
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
        (*g).mm305_direct_approach_degraded = false_0 != 0;
        (*g).mm305_model_snapshot_id = 0 as uint64_t;
        (*g).mm305_acquisition_route = TaemRoute {
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
        (*g).mm305_acquisition_route_cursor = 0 as size_t;
        (*g).mm305_acquisition_route_valid = false_0 != 0;
        (*g).mm305_acquisition_route_ut = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        (*g).taem_energy_law = TaemEnergyLawState {
            initialized: 0 as ::core::ffi::c_int != 0,
            loop_0: false,
            capture_forecast: false,
            leg: 0,
            side: 0,
            radius: 0.,
            r_turn: 0.,
            x_turn: 0.,
            turn_cx: 0.,
            turn_cy: 0.,
            k: 0.,
            s_req: 0.,
            s_go: 0.,
            last_geo_ut: 0.,
            last_pred_ut: 0.,
            admit_pending: false,
            admit_since_ut: 0.,
            low_energy_pending: false,
            low_energy_since_ut: 0.,
            trace_leg: 0,
            trace_side: 0,
            trace_radius: 0.,
            trace_loop: false,
            hac_left_rad: 0.,
            hac_last_ang: 0.,
            weave_on: false,
            weave_sign: 0,
            weave_brg: 0.,
            station_slope_deg: 0.,
        };
        (*g).taem_hac_law = TaemHacLawState {
            initialized: 0 as ::core::ffi::c_int != 0,
            final_line: false,
            phase: 0,
            side: 0,
            s_turn_sign: 0,
            drag_sign: 0,
            drag_away_sign: 0,
            drag_away: false,
            drag_flown: false,
            drag_aoa_cmd: 0.,
            drag_bank_cmd: 0.,
            trace: false,
            drag_required: 0.,
            rf: 0.,
            psi: 0.,
            last_ang: 0.,
            s_go: 0.,
            e_ref: 0.,
            h_ref: 0.,
            v_ref: 0.,
            e_err: 0.,
            energy_level: 0.,
            tangent_x: 0.,
            tangent_y: 0.,
            tangent_dist: 0.,
            turn_cost: 0.,
            sdot: 0.,
            last_sgo: 0.,
            last_sgo_ut: 0.,
            eta_live: 0.,
            last_plan_ut: 0.,
            last_side_ut: 0.,
            last_ut: 0.,
            speedbrake_integral: 0.,
            station_slope_deg: 0.,
            admit_pending: false,
            admit_since_ut: 0.,
            low_energy_pending: false,
            low_energy_since_ut: 0.,
            ref_0: TaemHacReference {
                valid: false,
                ds: 0.,
                mass: 0.,
                built_ut: 0.,
                aoa: [0.; 41],
                cl: [0.; 41],
                e: [0.; 480],
                h: [0.; 480],
                v: [0.; 480],
                h_eq: [0.; 480],
                v_eq: [0.; 480],
                s_gate: 0.,
                e_gate: 0.,
                h_gate: 0.,
            },
        };
        (*g).mm305_acquisition_sweep_rad = ::core::f32::NAN as ::core::ffi::c_double;
        (*g).mm305_planning_needed = false_0 != 0;
        (*g).mm305_plan_request_ut = ::core::f32::NAN as ::core::ffi::c_double;
        (*g).mm305_last_plan_attempt_ut = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        (*g).mm305_last_route_ut = -::core::f32::INFINITY as ::core::ffi::c_double;
        (*g).mm305_plan_failures = 0 as ::core::ffi::c_uint;
        (*g).mm305_replans = 0 as ::core::ffi::c_uint;
        (*g).mm305_lift_scale = if (*g).mm305_admission_lift_scale.is_finite() as i32
            != 0 && (*g).mm305_admission_lift_scale > 0.0f64
        {
            (*g).mm305_admission_lift_scale
        } else {
            1.0f64
        };
        (*g).mm305_drag_scale = if (*g).mm305_admission_drag_scale.is_finite() as i32
            != 0 && (*g).mm305_admission_drag_scale > 0.0f64
        {
            (*g).mm305_admission_drag_scale
        } else {
            1.0f64
        };
        (*g).hac_side_selected = false_0 != 0;
        (*g).hac_captured = false_0 != 0;
        (*g).hac_completed = false_0 != 0;
        (*g).hac_progress_valid = false_0 != 0;
        (*g).hac_remaining = 0.0f64;
        (*g).hac_radius = (*s).hac_radius;
        (*g).hac_side = 0.0f64;
        (*g).phase = PHASE_TAEM;
        if !taem_exec_enter(g, t, s, p, aero, cfg, false_0 != 0) {
            *g = before_handoff;
            entry.has_warning = true_0 != 0;
            snprintf(
                &raw mut entry.warning as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                b"MM305 executive initialization failed; MM304 ownership retained.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
            return entry;
        }
        if !(*g).diagnostic_shadow {
            if admission_route_qualified as ::core::ffi::c_int != 0
                && !guidance_mm305_adopt_admission_route(g, cfg) && !forced_handoff
            {
                *g = before_handoff;
                entry.has_warning = true_0 != 0;
                snprintf(
                    &raw mut entry.warning as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                    b"Qualified MM305 route could not be adopted; MM304 ownership retained.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
                return entry;
            }
            (*g).mm305_admission_needed = false_0 != 0;
            if !(*g).mm305_route_committed && !(*cfg).guidance.taem_energy_law
                && !(*cfg).guidance.taem_hac_law
            {
                (*g).mm305_planning_needed = true_0 != 0;
                (*g).mm305_plan_request_ut = (*t).ut;
                (*g).mm305_last_plan_attempt_ut = (*t).ut;
            }
            if forced_handoff {
                fprintf(
                    stderr,
                    b"MM304->MM305 physical handoff: UT %.2f h %.0f m Mach %.2f HAC energy excess %.0f m routeQualified=%d routeCommitted=%d; native MM305 owns recovery.\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*t).ut,
                    (*t).mean_altitude,
                    (*t).mach,
                    (*g).entry_hac_energy_excess,
                    if admission_route_qualified as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                    if (*g).mm305_route_committed as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                );
            }
        }
        if (*g).diagnostic_shadow as ::core::ffi::c_int != 0
            && (*g).diagnostic_stop_at_taem as ::core::ffi::c_int != 0
        {
            (*g).phase = PHASE_TAEM;
            entry.phase = PHASE_TAEM;
            entry.has_warning = false_0 != 0;
            entry.warning[0 as ::core::ffi::c_int as usize] = '\0' as i32
                as ::core::ffi::c_char;
            snprintf(
                &raw mut entry.status as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                b"MM304 diagnostic shadow captured the MM305 ownership boundary.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
            return entry;
        }
        guidance_result_clear(&raw mut entry);
    }
    if (*g).hac_completed as ::core::ffi::c_int != 0
        || (*g).final_approach_captured as ::core::ffi::c_int != 0
    {
        return terminal_final_test_guidance(g, t, course, p, aero, cfg, dt);
    }
    if !(*g).diagnostic_shadow
        && taem_exec_owns_vehicle(&raw mut (*g).taem_exec) as ::core::ffi::c_int != 0
        && (*t).radar_altitude.is_finite() as i32 != 0
        && (*t).vertical_speed.is_finite() as i32 != 0
        && (*t).runway_along_track < 0.0f64
        && (*t).radar_altitude
            < fmax(
                TERMINAL_BACKSTOP_HEIGHT_M,
                TERMINAL_BACKSTOP_TIME_S * fmax(0.0f64, -(*t).vertical_speed),
            )
        && fabs((*t).runway_cross_track)
            <= fmax(150.0f64, 2.0f64 * (*cfg).site.runway_width)
        && fabs(norm_signed_deg(course - (*cfg).site.runway_heading))
            <= TERMINAL_BACKSTOP_COURSE_DEG
    {
        let mut observation: TaemExecObservation = TaemExecObservation {
            ut: (*t).ut,
            relative_velocity: (*t).true_air_speed,
            checkpoint_restart: false_0 != 0,
        };
        if taem_exec_deliver_low_energy(&raw mut (*g).taem_exec, &raw mut observation) {
            (*g).hac_captured = true_0 != 0;
            (*g).hac_completed = (*g).hac_captured;
            (*g).mm305_hac_exit_reached = (*g).hac_completed;
            (*g).final_approach_captured = true_0 != 0;
            (*g).terminal_final_low_energy = true_0 != 0;
            terminal_set_stage(g, TERMINAL_OUTER_FINAL, (*t).ut);
            robust_pid_reset(&raw mut (*g).final_altitude_pid);
            robust_pid_reset(&raw mut (*g).speed_pid);
            robust_pid_reset(&raw mut (*g).flare_sink_pid);
            fprintf(
                stderr,
                b"MM305 terminal backstop: Final takes over at radar %.0f m, sink %.1f m/s, along %.0f m.\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).radar_altitude,
                -(*t).vertical_speed,
                (*t).runway_along_track,
            );
            return terminal_final_test_guidance(g, t, course, p, aero, cfg, dt);
        }
    }
    if !taem_exec_owns_vehicle(&raw mut (*g).taem_exec) {
        return terminal_abort(
            g,
            b"MM305 lost TAEM executive ownership before the HAC exit.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    let mut result_2: GuidanceResult = taem_guidance_native(
        g,
        t,
        state,
        course,
        p,
        aero,
        cfg,
        terminal_model,
        dt,
    );
    if (*g).mm305_hac_exit_reached as ::core::ffi::c_int != 0
        && result_2.phase as ::core::ffi::c_uint
            != PHASE_ABORT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        guidance_result_clear(&raw mut result_2);
        return terminal_final_test_guidance(g, t, course, p, aero, cfg, dt);
    }
    return result_2;
}
unsafe extern "C" fn terminal_guidance_selected(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut dt: ::core::ffi::c_double,
    mut terminal_model: *const TerminalModel,
) -> GuidanceResult {
    let mut handoff_end_selected: bool = false_0 != 0;
    let mut near_terminal_preview: bool = (*t).true_air_speed.is_finite() as i32 != 0
        && (*t).true_air_speed <= (*cfg).guidance.taem_force_handoff_speed
        && (*t).range_to_site.is_finite() as i32 != 0
        && (*t).range_to_site <= entry_taem_range_target(p, &raw const (*cfg).guidance);
    if (*cfg).site.allow_reciprocal_runway as ::core::ffi::c_int != 0
        && !(*g).runway_end_committed && !(*g).runway_end_preview_valid
        && !(*g).terminal_region_entered
        && (near_terminal_preview as ::core::ffi::c_int != 0
            || (*g).taem_interface_captured as ::core::ffi::c_int != 0
            || (*g).entry_exec.entry_complete as ::core::ffi::c_int != 0)
    {
        let mut primary: LandingSite = (*cfg).site;
        let mut reciprocal: LandingSite = runway_reciprocal_site(
            &raw mut primary,
            (*p).radius,
        );
        let mut primary_path: ::core::ffi::c_double = decision_runway_end_path_score(
            g,
            t,
            p,
            cfg,
            &raw mut primary,
        );
        let mut reciprocal_path: ::core::ffi::c_double = decision_runway_end_path_score(
            g,
            t,
            p,
            cfg,
            &raw mut reciprocal,
        );
        if primary_path.is_finite() as i32 != 0
            || reciprocal_path.is_finite() as i32 != 0
        {
            let mut best_end: ::core::ffi::c_int = if primary_path.is_finite() as i32
                == 0
                || reciprocal_path.is_finite() as i32 != 0
                    && reciprocal_path < primary_path
            {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            };
            (*g).runway_end_index = best_end;
            (*g).runway_end_preview_valid = true_0 != 0;
            handoff_end_selected = true_0 != 0;
        }
    }
    let mut selected: LandingConfiguration = *cfg;
    let mut framed: Telemetry = *t;
    if (*cfg).site.allow_reciprocal_runway as ::core::ffi::c_int != 0
        && (*g).runway_end_preview_valid as ::core::ffi::c_int != 0
        && (*g).runway_end_index == 1 as ::core::ffi::c_int
    {
        selected.site = runway_reciprocal_site(&raw const (*cfg).site, (*p).radius);
    }
    telemetry_reframe_runway(&raw mut framed, &raw mut selected.site, (*p).radius);
    let mut entry_ref: ::core::ffi::c_double = if (*g).entry_reference_speed > 0.0f64 {
        (*g).entry_reference_speed
    } else {
        framed.true_air_speed
    };
    let mut demand: EntryTerminalDemand = entry_terminal_demand(
        framed.latitude,
        framed.mean_altitude,
        framed.range_to_site,
        framed.horizontal_speed,
        framed.course_to_site_error,
        framed.vertical_speed,
        framed.true_air_speed,
        entry_ref,
        p,
        &raw mut selected.vehicle,
        &raw mut selected.site,
        &raw mut selected.guidance,
    );
    framed.energy_excess_range = -demand.projected_taem_range_error;
    let mut result: GuidanceResult = terminal_guidance(
        g,
        &raw mut framed,
        state,
        course,
        p,
        aero,
        &raw mut selected,
        dt,
        terminal_model,
    );
    if handoff_end_selected as ::core::ffi::c_int != 0
        && (*g).terminal_region_entered as ::core::ffi::c_int != 0
    {
        (*g).runway_end_committed = true_0 != 0;
    }
    if (*g).terminal_path_committed {
        (*g).runway_end_committed = true_0 != 0;
    }
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_update_impl(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut plan: *const DeorbitPlan,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut terminal_model: *const TerminalModel,
) -> GuidanceResult {
    let mut dt: ::core::ffi::c_double = if (*g).has_previous_ut as ::core::ffi::c_int
        != 0
    {
        clampd(
            (*t).ut - (*g).previous_ut,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    } else {
        0.1f64
    };
    (*g).previous_ut = (*t).ut;
    (*g).has_previous_ut = true_0 != 0;
    if (*g).aborted {
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
        return result_make(
            PHASE_ABORT,
            c,
            if (*g).abort_reason[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int
                != 0
            {
                &raw mut (*g).abort_reason as *mut ::core::ffi::c_char
                    as *const ::core::ffi::c_char
            } else {
                b"Automation aborted. Manual control restored.\0" as *const u8
                    as *const ::core::ffi::c_char
            },
            ::core::ptr::null::<::core::ffi::c_char>(),
        );
    }
    if !(*g).automation_engaged {
        (*g).phase = PHASE_IDLE;
        reset_limiters(g);
        let mut c_0: GuidanceCommand = GuidanceCommand {
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
        guidance_command_init(&raw mut c_0);
        if !plan.is_null() {
            if (*plan).execution_qualified {
                return result_make(
                    PHASE_IDLE,
                    c_0,
                    if (*plan).execution_degraded as ::core::ffi::c_int != 0 {
                        b"Entry-stage-qualified orbital plan is ready. Engage guidance to begin execution.\0"
                            as *const u8 as *const ::core::ffi::c_char
                    } else {
                        b"Strict full-route plan is ready. Engage guidance to begin execution.\0"
                            as *const u8 as *const ::core::ffi::c_char
                    },
                    if (*plan).execution_degraded as ::core::ffi::c_int != 0 {
                        b"The finite burn and safe entry envelope qualified; downstream MM304/TAEM shadow capture remains separately reported.\0"
                            as *const u8 as *const ::core::ffi::c_char
                    } else {
                        ::core::ptr::null::<::core::ffi::c_char>()
                    },
                );
            }
            return result_make(
                PHASE_IDLE,
                c_0,
                b"Best available orbital plan is selected under warning; finite-burn or atmospheric-entry qualification did not pass.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                b"Live execution may proceed for testing, but qualification failures remain visible and downstream transition/handoff gates are not bypassed.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return result_make(
            PHASE_IDLE,
            c_0,
            b"Guidance is not engaged.\0" as *const u8 as *const ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        );
    }
    if (*g).paused {
        (*g).phase = PHASE_PAUSED;
        let mut c_1: GuidanceCommand = GuidanceCommand {
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
        guidance_command_init(&raw mut c_1);
        return result_make(
            PHASE_PAUSED,
            c_1,
            b"Guidance paused. Attitude hold released.\0" as *const u8
                as *const ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        );
    }
    let mut v: *const VehicleProfile = &raw const (*cfg).vehicle;
    let mut s: *const GuidanceSettings = &raw const (*cfg).guidance;
    let mut course: ::core::ffi::c_double = surface_course(
        (*state).position,
        (*state).velocity,
        planet_rotation_vector(p),
        (*p).north_axis,
        (*t).heading,
    );
    if (*g).terminal_glide_mode {
        (*g).jet_assist_throttle = 0.0f64;
        let mut terminal: GuidanceResult = terminal_guidance_selected(
            g,
            t,
            state,
            course,
            p,
            aero,
            cfg,
            dt,
            terminal_model,
        );
        terminal.command.jet_assist_throttle = (*g).jet_assist_throttle;
        return terminal;
    }
    if plan.is_null() {
        (*g).phase = PHASE_PLANNING;
        let mut c_2: GuidanceCommand = GuidanceCommand {
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
        guidance_command_init(&raw mut c_2);
        return result_make(
            PHASE_PLANNING,
            c_2,
            b"A deorbit plan is required before engagement.\0" as *const u8
                as *const ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        );
    }
    let mut entry_alt: ::core::ffi::c_double = entry_guidance_start_altitude(p, s);
    if !(*g).deorbit_burn_completed && (*t).mean_altitude <= entry_alt
        && (*t).vertical_speed < 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        (*g).deorbit_burn_completed = true_0 != 0;
    }
    let mut peri_rem: ::core::ffi::c_double = (*t).periapsis_altitude
        - (*plan).predicted_post_burn_periapsis_altitude;
    let mut has_peri: bool = (*plan).predicted_post_burn_periapsis_altitude
        > 10000 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*plan).predicted_post_burn_periapsis_altitude < (*p).atmosphere_depth
        && (*t).periapsis_altitude.is_finite() as i32 != 0
        && (*t).periapsis_altitude > -(*p).radius * 0.5f64;
    let mut peri_hit: bool = (*g).has_burn_command_started as ::core::ffi::c_int != 0
        && has_peri as ::core::ffi::c_int != 0
        && peri_rem <= 300 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut state_hit: bool = (*g).has_burn_command_started as ::core::ffi::c_int != 0
        && (*plan).live_cutoff_capture_qualified as ::core::ffi::c_int != 0 && !has_peri;
    let mut dvtarget: ::core::ffi::c_double = (*plan).delta_v
        + (if has_peri as ::core::ffi::c_int != 0 {
            8 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        }) as ::core::ffi::c_double;
    if (*g).has_burn_command_started as ::core::ffi::c_int != 0
        && ((*g).delivered_delta_v >= dvtarget || peri_hit as ::core::ffi::c_int != 0
            || state_hit as ::core::ffi::c_int != 0)
    {
        (*g).deorbit_burn_completed = true_0 != 0;
    }
    let mut still: bool = (*g).has_burn_command_started as ::core::ffi::c_int != 0
        && !(*g).deorbit_burn_completed && (*g).delivered_delta_v < dvtarget && !peri_hit
        && !state_hit;
    let mut burnstart: ::core::ffi::c_double = (*plan).burn_ut
        - (*plan).estimated_burn_duration * 0.5f64;
    let mut open: ::core::ffi::c_double = ((*t).ut >= burnstart) as ::core::ffi::c_int
        as ::core::ffi::c_double;
    if !(*g).deorbit_burn_completed
        && (*t).mean_altitude
            > entry_alt - 4500 as ::core::ffi::c_int as ::core::ffi::c_double
        && ((*t).vertical_speed >= -(20 as ::core::ffi::c_int) as ::core::ffi::c_double
            || still as ::core::ffi::c_int != 0 || open != 0.)
    {
        let mut c_3: GuidanceCommand = GuidanceCommand {
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
        guidance_command_init(&raw mut c_3);
        c_3.autopilot_engaged = true_0 != 0;
        c_3.use_inertial_direction = true_0 != 0;
        c_3.thrust_alignment = true_0 != 0;
        c_3.inertial_direction = vnorm(
            vadd(
                vnorm(
                    vscale(
                        (*state).velocity,
                        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                    ),
                    v3(
                        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ),
                ),
                vscale(
                    vnorm(
                        vcross((*state).position, (*state).velocity),
                        v3(
                            0 as ::core::ffi::c_int as ::core::ffi::c_double,
                            0 as ::core::ffi::c_int as ::core::ffi::c_double,
                            1 as ::core::ffi::c_int as ::core::ffi::c_double,
                        ),
                    ),
                    tan((*plan).burn_normal_angle * DEG2RAD),
                ),
            ),
            v3(
                -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        c_3.navball_speed_mode = SPEED_ORBIT;
        c_3.control_profile = PROFILE_ORBITAL;
        if (*t).ut < burnstart - 12 as ::core::ffi::c_int as ::core::ffi::c_double {
            (*g).phase = PHASE_COAST;
            let mut st: [::core::ffi::c_char; 128] = [0; 128];
            let mut sec: ::core::ffi::c_int = fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                round(burnstart - (*t).ut),
            ) as ::core::ffi::c_int;
            snprintf(
                &raw mut st as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                b"Coasting while acquiring retrograde. T\xE2\x88\x92%02d:%02d.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                sec / 60 as ::core::ffi::c_int,
                sec % 60 as ::core::ffi::c_int,
            );
            return result_make(
                (*g).phase,
                c_3,
                &raw mut st as *mut ::core::ffi::c_char,
                ::core::ptr::null::<::core::ffi::c_char>(),
            );
        }
        if (*t).ut < burnstart {
            (*g).phase = PHASE_BURN_SETUP;
            return result_make(
                (*g).phase,
                c_3,
                b"Aligning retrograde for the deorbit burn.\0" as *const u8
                    as *const ::core::ffi::c_char,
                ::core::ptr::null::<::core::ffi::c_char>(),
            );
        }
        if (*g).delivered_delta_v < dvtarget && !peri_hit && !state_hit {
            (*g).phase = PHASE_DEORBIT_BURN;
            let mut maxthr: ::core::ffi::c_double = (*s).deorbit_maximum_throttle;
            let mut avail: ::core::ffi::c_double = fmax(
                0.05f64,
                (*t).available_thrust * maxthr
                    / fmax((*t).mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double),
            );
            let mut meas: ::core::ffi::c_double = fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                (*t).current_thrust
                    / fmax((*t).mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double),
            );
            let mut align: ::core::ffi::c_double = fabs((*t).autopilot_error);
            let mut aligned: bool = align
                <= 10 as ::core::ffi::c_int as ::core::ffi::c_double;
            if !aligned && !(*g).has_burn_command_started
                && (*t).ut
                    > burnstart
                        + fmax(
                            30 as ::core::ffi::c_int as ::core::ffi::c_double,
                            (*plan).estimated_burn_duration,
                        )
            {
                guidance_abort(g);
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
                return result_make(
                    PHASE_ABORT,
                    safe,
                    b"Deorbit burn aborted because retrograde alignment missed the burn window.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    b"The vehicle did not achieve the required retrograde alignment in time. Manual control restored; replan from the current orbit.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
            }
            if aligned as ::core::ffi::c_int != 0 && !(*g).has_burn_command_started {
                (*g).burn_command_started_ut = (*t).ut;
                (*g).has_burn_command_started = true_0 != 0;
                (*g).burn_active_elapsed = 0 as ::core::ffi::c_int
                    as ::core::ffi::c_double;
                (*g).burn_progress_watch_ut = (*t).ut;
                (*g).burn_progress_watch_delta_v = (*g).delivered_delta_v;
                (*g).has_burn_progress_watch = true_0 != 0;
            }
            if aligned as ::core::ffi::c_int != 0
                && (*g).has_burn_command_started as ::core::ffi::c_int != 0
            {
                (*g).delivered_delta_v
                    += meas
                        * fmax(
                            0 as ::core::ffi::c_int as ::core::ffi::c_double,
                            cos(align * DEG2RAD),
                        ) * cos((*plan).burn_normal_angle * DEG2RAD) * dt;
                (*g).burn_active_elapsed += dt;
                if !(*g).has_burn_progress_watch
                    || (*g).delivered_delta_v - (*g).burn_progress_watch_delta_v
                        >= 0.25f64
                {
                    (*g).burn_progress_watch_ut = (*t).ut;
                    (*g).burn_progress_watch_delta_v = (*g).delivered_delta_v;
                    (*g).has_burn_progress_watch = true_0 != 0;
                }
            }
            let mut rem: ::core::ffi::c_double = fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                dvtarget - (*g).delivered_delta_v,
            );
            let mut fraction: ::core::ffi::c_double = burn_fraction(
                (*g).burn_active_elapsed,
                rem,
                avail,
                (*s).deorbit_throttle_ramp_duration,
            );
            let mut authority: ::core::ffi::c_double = clampd(
                (10 as ::core::ffi::c_int as ::core::ffi::c_double - align)
                    / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            let mut pa: ::core::ffi::c_double = if has_peri as ::core::ffi::c_int != 0 {
                clampd(
                    (peri_rem - 250 as ::core::ffi::c_int as ::core::ffi::c_double)
                        / DEORBIT_PERIAPSIS_TAPER_SPAN_M,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                )
            } else {
                1 as ::core::ffi::c_int as ::core::ffi::c_double
            };
            c_3.target_throttle = if rem < 0.08f64 {
                0 as ::core::ffi::c_int as ::core::ffi::c_double
            } else if aligned as ::core::ffi::c_int != 0 {
                maxthr * fmin(fraction, pa) * authority
            } else {
                0 as ::core::ffi::c_int as ::core::ffi::c_double
            };
            let mut expected_accel: ::core::ffi::c_double = fmax(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                (*t).available_thrust * c_3.target_throttle
                    / fmax((*t).mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double),
            );
            let mut thrust_lost: bool = aligned as ::core::ffi::c_int != 0
                && (*g).has_burn_command_started as ::core::ffi::c_int != 0
                && (*g).has_burn_progress_watch as ::core::ffi::c_int != 0
                && rem > 2 as ::core::ffi::c_int as ::core::ffi::c_double
                && expected_accel > 0.02f64
                && meas < fmax(0.01f64, expected_accel * 0.10f64);
            (*g).burn_thrust_loss_elapsed = if thrust_lost as ::core::ffi::c_int != 0 {
                (*g).burn_thrust_loss_elapsed + dt
            } else {
                0.0f64
            };
            let mut stalled: bool = (*g).burn_thrust_loss_elapsed
                > 6 as ::core::ffi::c_int as ::core::ffi::c_double;
            if stalled {
                guidance_abort(g);
                let mut safe_0: GuidanceCommand = GuidanceCommand {
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
                guidance_command_init(&raw mut safe_0);
                return result_make(
                    PHASE_ABORT,
                    safe_0,
                    b"Deorbit burn aborted after sustained loss of thrust/delta-v progress.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    b"The burn stopped making measurable progress despite a meaningful commanded thrust level. Manual control restored; do not continue entry on the stale plan.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
            }
            let mut st_0: [::core::ffi::c_char; 320] = [0; 320];
            snprintf(
                &raw mut st_0 as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 320]>() as size_t,
                b"Deorbit burn: %.1f / %.1f m/s, throttle %.0f%%.%s%s\0" as *const u8
                    as *const ::core::ffi::c_char,
                (*g).delivered_delta_v,
                (*plan).delta_v,
                c_3.target_throttle * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
                if has_peri as ::core::ffi::c_int != 0 {
                    b" Periapsis closure active.\0" as *const u8
                        as *const ::core::ffi::c_char
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
                if (*plan).live_cutoff_capture_qualified as ::core::ffi::c_int != 0 {
                    b" Cutoff-now trajectory is inside the entry corridor.\0"
                        as *const u8 as *const ::core::ffi::c_char
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
            return result_make(
                (*g).phase,
                c_3,
                &raw mut st_0 as *mut ::core::ffi::c_char,
                if (*t).available_thrust
                    < 1 as ::core::ffi::c_int as ::core::ffi::c_double
                {
                    b"No usable thrust is available.\0" as *const u8
                        as *const ::core::ffi::c_char
                } else if align > 10 as ::core::ffi::c_int as ::core::ffi::c_double {
                    b"Holding throttle until retrograde alignment is stable.\0"
                        as *const u8 as *const ::core::ffi::c_char
                } else {
                    ::core::ptr::null::<::core::ffi::c_char>()
                },
            );
        }
    }
    if !(*g).deorbit_burn_completed {
        (*g).phase = PHASE_COAST;
        let mut c_4: GuidanceCommand = GuidanceCommand {
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
        guidance_command_init(&raw mut c_4);
        c_4.autopilot_engaged = true_0 != 0;
        c_4.use_inertial_direction = true_0 != 0;
        c_4.thrust_alignment = true_0 != 0;
        c_4.inertial_direction = vnorm(
            vadd(
                vnorm(
                    vscale(
                        (*state).velocity,
                        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                    ),
                    v3(
                        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ),
                ),
                vscale(
                    vnorm(
                        vcross((*state).position, (*state).velocity),
                        v3(
                            0 as ::core::ffi::c_int as ::core::ffi::c_double,
                            0 as ::core::ffi::c_int as ::core::ffi::c_double,
                            1 as ::core::ffi::c_int as ::core::ffi::c_double,
                        ),
                    ),
                    tan((*plan).burn_normal_angle * DEG2RAD),
                ),
            ),
            v3(
                -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        c_4.navball_speed_mode = SPEED_ORBIT;
        c_4.control_profile = PROFILE_ORBITAL;
        return result_make(
            (*g).phase,
            c_4,
            b"Holding retrograde until the deorbit burn completes.\0" as *const u8
                as *const ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        );
    }
    if !(*g).atmospheric_interface_crossed {
        let mut signed_entry_roll: ::core::ffi::c_double = norm_signed_deg((*t).roll);
        let mut entry_roll: ::core::ffi::c_double = fabs(signed_entry_roll);
        let mut capture_aoa: ::core::ffi::c_double = entry_low_q_protective_aoa_floor(
            (*t).dynamic_pressure,
            v,
        );
        let mut entry_pitch_error: ::core::ffi::c_double = fabs(
            (*t).angle_of_attack - capture_aoa,
        );
        let mut entry_heading_error: ::core::ffi::c_double = fabs(
            norm_signed_deg((*t).ground_track_heading - (*t).heading),
        );
        let mut entry_attitude_ready: bool = entry_roll
            <= 18 as ::core::ffi::c_int as ::core::ffi::c_double
            && entry_pitch_error <= 4 as ::core::ffi::c_int as ::core::ffi::c_double
            && entry_heading_error <= 15 as ::core::ffi::c_int as ::core::ffi::c_double
            && fabs((*t).roll_rate) <= 8 as ::core::ffi::c_int as ::core::ffi::c_double
            && fabs((*t).pitch_rate) <= 8 as ::core::ffi::c_int as ::core::ffi::c_double
            && fabs((*t).heading_rate)
                <= 8 as ::core::ffi::c_int as ::core::ffi::c_double;
        let mut entry_reached: bool = (*t).mean_altitude <= entry_alt
            && (*t).vertical_speed < 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        if entry_reached as ::core::ffi::c_int != 0
            && entry_attitude_ready as ::core::ffi::c_int != 0
        {
            (*g).atmospheric_interface_crossed = true_0 != 0;
            if entry_roll >= 5 as ::core::ffi::c_int as ::core::ffi::c_double {
                (*g).s_turn_sign = (if signed_entry_roll
                    < 0 as ::core::ffi::c_int as ::core::ffi::c_double
                {
                    -(1 as ::core::ffi::c_int)
                } else {
                    1 as ::core::ffi::c_int
                }) as ::core::ffi::c_double;
            }
        } else {
            (*g).phase = PHASE_ENTRY_INTERFACE;
            let mut st_1: *const ::core::ffi::c_char = if entry_reached
                as ::core::ffi::c_int != 0
            {
                b"Atmospheric interface reached. Holding prograde heading, entry AoA and wings-level attitude before MM304 guidance.\0"
                    as *const u8 as *const ::core::ffi::c_char
            } else {
                b"Burn complete. Using RCS/direct control to capture prograde heading, entry AoA and wings-level attitude.\0"
                    as *const u8 as *const ::core::ffi::c_char
            };
            return result_make(
                (*g).phase,
                entry_capture(t, state, v),
                st_1,
                if entry_reached as ::core::ffi::c_int != 0 && !entry_attitude_ready {
                    b"MM304 guidance is inhibited until entry heading/AoA/roll attitude is stabilized.\0"
                        as *const u8 as *const ::core::ffi::c_char
                } else {
                    ::core::ptr::null::<::core::ffi::c_char>()
                },
            );
        }
    }
    return terminal_guidance_selected(
        g,
        t,
        state,
        course,
        p,
        aero,
        cfg,
        dt,
        terminal_model,
    );
}
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
