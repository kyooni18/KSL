use crate::sys::*;
extern "C" {
    fn taem_exec_initialize(
        exec: *mut TaemExecutive,
        observation: *const TaemExecObservation,
        inputs: *const TaemExecInputs,
    ) -> bool;
    fn taem_exec_update(
        exec: *mut TaemExecutive,
        observation: *const TaemExecObservation,
        inputs: *const TaemExecInputs,
    ) -> bool;
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
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn expm1(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log1p(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn floor(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmod(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn nextafter(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn remainder(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn round(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_state_validate(
        state: *const TerminalDynamicState,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn world_atmosphere_sample(
        w: *const KerbinWorld,
        altitude_m: ::core::ffi::c_double,
    ) -> AtmosphereSample;
    fn aero_coefficients(
        a: *const AeroTable,
        mach: ::core::ffi::c_double,
        alpha_deg: ::core::ffi::c_double,
        cl: *mut ::core::ffi::c_double,
        cd: *mut ::core::ffi::c_double,
    );
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
    fn taem_hac_geometry_sweep(
        model: *const TerminalModel,
        radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        geometry: *mut TaemFixedHacGeometry,
    ) -> bool;
    fn taem_fixed_hac_turn_reachability(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        radius_m: ::core::ffi::c_double,
        reachability: *mut TaemReachability,
    ) -> bool;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_bank_authority_limit(
        true_air_speed: ::core::ffi::c_double,
        dynamic_pressure: ::core::ffi::c_double,
        g_force: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
        maximum_bank: ::core::ffi::c_double,
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
    fn local_point(
        origin: GeoPoint,
        east: ::core::ffi::c_double,
        north: ::core::ffi::c_double,
        radius: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
    ) -> GeoPoint;
    fn runway_reciprocal_site(
        primary: *const LandingSite,
        radius: ::core::ffi::c_double,
    ) -> LandingSite;
    fn robust_pid_reset(p: *mut RobustPID);
    fn trajectory_clear(t: *mut Trajectory);
    fn trajectory_append(t: *mut Trajectory, p: TrajectoryPoint) -> bool;
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
    fn aerodynamic_best_glide_aoa(
        mach: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn guidance_machine_init(g: *mut GuidanceMachine);
    fn guidance_set_engaged(g: *mut GuidanceMachine, engaged: bool);
    fn decision_pitch_capture_time(
        telemetry: *const Telemetry,
        target_aoa_deg: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn decision_guidance_attitude_limits(
        telemetry: *const Telemetry,
        settings: *const GuidanceSettings,
        phase: GuidancePhase,
        pitch_axis: bool,
    ) -> GuidanceAttitudeLimits;
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
    fn taem_tracker_update(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        reference: *const TaemPathReference,
        dt_s: ::core::ffi::c_double,
    ) -> TaemTrackerOutput;
    fn taem_route_build_hac(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        hac_radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_build_hac_shortest(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        hac_radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        peak_curvature_limit_per_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_build_hac_length(
        model: *const TerminalModel,
        start: *const TaemGeometryState,
        hac_radius_m: ::core::ffi::c_double,
        side: ::core::ffi::c_double,
        sweep_abs_rad: ::core::ffi::c_double,
        point_spacing_m: ::core::ffi::c_double,
        maximum_lead_curvature_per_m: ::core::ffi::c_double,
        peak_curvature_limit_per_m: ::core::ffi::c_double,
        target_lead_length_m: ::core::ffi::c_double,
        route: *mut TaemRoute,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn taem_route_limit_initial_vertical_authority(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        route: *mut TaemRoute,
    ) -> bool;
    fn taem_route_planned_speed(
        route: *const TaemRoute,
        station_m: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn taem_route_point_at_index(
        route: *const TaemRoute,
        index: size_t,
        point: *mut TaemRoutePoint,
    ) -> bool;
    fn taem_route_reference(
        route: *const TaemRoute,
        state: *const TaemGeometryState,
        cursor: *mut size_t,
        reference: *mut TaemPathReference,
        point_index: *mut size_t,
    ) -> bool;
    fn hac_response_lead_time(
        t: *const Telemetry,
        s: *const GuidanceSettings,
        nominal_bank: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
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
    fn terminal_lateral_bank_limit(
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
    fn live_drag_accel(
        t: *const Telemetry,
        aero: AerodynamicModel,
        v: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn terminal_observe_landing_aero(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        v: *const VehicleProfile,
        gravity: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    );
    fn terminal_delivery_touchdown_speed(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        p: *const PlanetModel,
        cfg: *const LandingConfiguration,
        trigger_altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn terminal_native_delivery_admissible(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        seed: *const GuidanceMachine,
        evaluation: *mut TaemTerminalEvaluation,
    ) -> bool;
    fn terminal_store_preflare_plan(
        g: *mut GuidanceMachine,
        p: *const TerminalPreflarePlan,
    );
    fn terminal_set_stage(
        g: *mut GuidanceMachine,
        stage: TerminalVerticalStage,
        ut: ::core::ffi::c_double,
    );
    fn runway_landing_intercept_ratio(
        along: ::core::ffi::c_double,
        cross: ::core::ffi::c_double,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
        ratio: *mut ::core::ffi::c_double,
    ) -> bool;
    fn terminal_pavement_intercept_rate(
        t: *const Telemetry,
        cfg: *const LandingConfiguration,
        rate: *mut ::core::ffi::c_double,
    ) -> bool;
    fn terminal_preflare_alignment_valid(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        cfg: *const LandingConfiguration,
    ) -> bool;
    fn terminal_outer_capture_admissible(
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
    fn terminal_abort(
        g: *mut GuidanceMachine,
        reason: *const ::core::ffi::c_char,
    ) -> GuidanceResult;
    fn taem_fixed_hac_candidate_preferred(
        model: *const TerminalModel,
        trial: *const TaemFixedHacCandidate,
        best: *const TaemFixedHacCandidate,
    ) -> bool;
    fn taem_fixed_hac_search(
        model: *const TerminalModel,
        initial: *const TerminalDynamicState,
        hac_radius_m: ::core::ffi::c_double,
        route_spacing_m: ::core::ffi::c_double,
        dt_s: ::core::ffi::c_double,
        maximum_elapsed_s: ::core::ffi::c_double,
    ) -> TaemFixedHacSearch;
    fn taem_fixed_hac_evaluate_route(
        model: *const TerminalModel,
        initial: *const TerminalDynamicState,
        route: *const TaemRoute,
        runway_end: ::core::ffi::c_int,
        dt_s: ::core::ffi::c_double,
        maximum_elapsed_s: ::core::ffi::c_double,
    ) -> TaemFixedHacCandidate;
    fn terminal_solver_generate_profile(
        model: *const TerminalModel,
        initial: *const TerminalDynamicState,
        route: *mut TaemRoute,
        dt_s: ::core::ffi::c_double,
        maximum_elapsed_s: ::core::ffi::c_double,
    ) -> TerminalProfileResult;
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
pub struct TaemExecObservation {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub checkpoint_restart: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemExecInputs {
    pub mm304_complete: bool,
    pub resume_mm305: bool,
    pub terminal_path_selected: bool,
    pub terminal_path_captured: bool,
    pub final_intercept_ready: bool,
    pub final_approach_ready: bool,
    pub terminal_contract: TaemTerminalContract,
    pub off_nominal_recovery_active: bool,
    pub off_nominal_recovery_reason: TaemRecoveryReason,
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
pub struct TaemRoutePoint {
    pub along_m: ::core::ffi::c_double,
    pub cross_m: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub distance_from_start_m: ::core::ffi::c_double,
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
pub const TEL_LEG_FINAL: C2RustUnnamed_0 = 4;
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
pub struct GuidanceAttitudeLimits {
    pub rate_deg_s: ::core::ffi::c_double,
    pub accel_deg_s2: ::core::ffi::c_double,
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
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
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
pub struct TaemReachability {
    pub valid: bool,
    pub lateral_authority_ok: bool,
    pub energy_qualified: bool,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub lateral_margin_mps2: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
    pub roll_settling_time_s: ::core::ffi::c_double,
    pub roll_settling_distance_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalControl {
    pub angle_of_attack_rad: ::core::ffi::c_double,
    pub bank_rad: ::core::ffi::c_double,
    pub dt_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemPathReference {
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub course_deg: ::core::ffi::c_double,
    pub curvature_right_per_m: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
    pub flight_path_angle_deg: ::core::ffi::c_double,
    pub vertical_curvature_per_m: ::core::ffi::c_double,
    pub station_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTrackerOutput {
    pub valid: bool,
    pub lateral_authority_ok: bool,
    pub vertical_authority_ok: bool,
    pub control: TerminalControl,
    pub cross_track_error_m: ::core::ffi::c_double,
    pub course_error_deg: ::core::ffi::c_double,
    pub required_lateral_accel_mps2: ::core::ffi::c_double,
    pub available_lateral_accel_mps2: ::core::ffi::c_double,
    pub required_vertical_lift_mps2: ::core::ffi::c_double,
    pub delivered_vertical_lift_mps2: ::core::ffi::c_double,
    pub response_time_s: ::core::ffi::c_double,
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
pub struct TelSegment {
    pub length: ::core::ffi::c_double,
    pub radius: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelPoint {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub h: ::core::ffi::c_double,
    pub v: ::core::ffi::c_double,
    pub g: ::core::ffi::c_double,
    pub psi: ::core::ffi::c_double,
    pub m: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub aoa_rate: ::core::ffi::c_double,
    pub bank: ::core::ffi::c_double,
    pub ground_speed: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelContext {
    pub model: *const TerminalModel,
    pub d_station: ::core::ffi::c_double,
    pub h_station: ::core::ffi::c_double,
    pub v_station: ::core::ffi::c_double,
    pub e_station: ::core::ffi::c_double,
    pub aoa_max: ::core::ffi::c_double,
    pub slope_deg: ::core::ffi::c_double,
    pub final_line: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HacCtx {
    pub c: TelContext,
    pub aero: *const AeroTable,
    pub live: *const TaemHacLiveAero,
    pub mass: ::core::ffi::c_double,
    pub aoa_supersonic: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HacPrediction {
    pub valid: bool,
    pub s_go: ::core::ffi::c_double,
    pub psi_t: ::core::ffi::c_double,
    pub tx: ::core::ffi::c_double,
    pub ty: ::core::ffi::c_double,
    pub tangent_dist: ::core::ffi::c_double,
    pub acq_turn: ::core::ffi::c_double,
    pub acq_arc: ::core::ffi::c_double,
    pub hac_arc: ::core::ffi::c_double,
    pub turn_cost: ::core::ffi::c_double,
    pub e_ref: ::core::ffi::c_double,
}
pub const TAEM_HAC_PHASE_ACQ: C2RustUnnamed_1 = 2;
pub const TAEM_HAC_PHASE_HAC: C2RustUnnamed_1 = 3;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Mm305AcquisitionChoice {
    pub valid: bool,
    pub route: TaemRoute,
    pub sweep_rad: ::core::ffi::c_double,
    pub profile_class: Mm305AcquisitionProfileClass,
    pub speed_error_mps: ::core::ffi::c_double,
    pub energy_metric: ::core::ffi::c_double,
    pub circle_penalty: ::core::ffi::c_double,
    pub profile_reason: *const ::core::ffi::c_char,
    pub replay_quality: ::core::ffi::c_double,
}
pub type Mm305AcquisitionProfileClass = ::core::ffi::c_uint;
pub const MM305_ACQ_PROFILE_INVALID: Mm305AcquisitionProfileClass = 3;
pub const MM305_ACQ_PROFILE_TOO_LONG: Mm305AcquisitionProfileClass = 2;
pub const MM305_ACQ_PROFILE_TOO_SHORT: Mm305AcquisitionProfileClass = 1;
pub const MM305_ACQ_PROFILE_FEASIBLE: Mm305AcquisitionProfileClass = 0;
pub type Mm305AcquisitionRouteKind = ::core::ffi::c_uint;
pub const MM305_ACQ_ROUTE_ENERGY: Mm305AcquisitionRouteKind = 2;
pub const MM305_ACQ_ROUTE_SHORTEST: Mm305AcquisitionRouteKind = 1;
pub const MM305_ACQ_ROUTE_NORMAL: Mm305AcquisitionRouteKind = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Mm305AcquisitionSeed {
    pub valid: bool,
    pub radius_m: ::core::ffi::c_double,
    pub sweep_rad: ::core::ffi::c_double,
    pub target_lead_m: ::core::ffi::c_double,
    pub cheap_score: ::core::ffi::c_double,
    pub kind: Mm305AcquisitionRouteKind,
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
pub struct TerminalProfileResult {
    pub valid: bool,
    pub aoa_deg: ::core::ffi::c_double,
    pub exit_speed_mps: ::core::ffi::c_double,
    pub exit_fpa_deg: ::core::ffi::c_double,
    pub reason: *const ::core::ffi::c_char,
}
pub const MM305_ACQ_NATIVE_SEEDS: C2RustUnnamed = 18;
pub const MM305_ACQ_RADIUS_SEEDS: C2RustUnnamed = 1;
pub const MM305_ACQ_RADIUS_COUNT: C2RustUnnamed = 7;
pub const MM305_ACQ_GLOBAL_SEEDS: C2RustUnnamed = 4;
pub const MM305_ACQ_RADIUS_SHORTEST_SEEDS: C2RustUnnamed = 1;
pub type C2RustUnnamed = ::core::ffi::c_uint;
pub const TEL_LEG_TURN: C2RustUnnamed_0 = 1;
pub const TEL_LEG_DOWN: C2RustUnnamed_0 = 2;
pub const TEL_LEG_OUT: C2RustUnnamed_0 = 0;
pub const TEL_LEG_HAC: C2RustUnnamed_0 = 3;
pub const TAEM_HAC_PHASE_DRAG: C2RustUnnamed_1 = 0;
pub const TAEM_HAC_PHASE_PREFINAL: C2RustUnnamed_1 = 4;
pub const TAEM_HAC_PHASE_STURN: C2RustUnnamed_1 = 1;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFixedHacSearch {
    pub status: TaemPlanStatus,
    pub selected_candidate: ::core::ffi::c_int,
    pub candidate_count: ::core::ffi::c_int,
    pub candidates: [TaemFixedHacCandidate; 4],
    pub reason: *const ::core::ffi::c_char,
}
pub type C2RustUnnamed_0 = ::core::ffi::c_uint;
pub type C2RustUnnamed_1 = ::core::ffi::c_uint;
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const TAEM_ROUTE_PROFILE_LUT_POINTS: ::core::ffi::c_int = 257 as ::core::ffi::c_int;
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
#[inline]
unsafe extern "C" fn adaptive_trim_fit(
    mut m: *const AdaptiveTrim,
    mut a: *mut ::core::ffi::c_double,
    mut b: *mut ::core::ffi::c_double,
    mut noise: *mut ::core::ffi::c_double,
) -> bool {
    if m.is_null() || (*m).weight < 2.0f64 || (*m).hi - (*m).lo < 3.0f64 {
        return false_0 != 0;
    }
    let mut variance: ::core::ffi::c_double = (*m).xx / (*m).weight
        - pow((*m).x / (*m).weight, 2 as ::core::ffi::c_int as ::core::ffi::c_double);
    if variance < 1.0f64 {
        return false_0 != 0;
    }
    *b = ((*m).xy / (*m).weight - (*m).x * (*m).y / ((*m).weight * (*m).weight))
        / variance;
    *a = (*m).y / (*m).weight - *b * (*m).x / (*m).weight;
    let mut residual: ::core::ffi::c_double = (*m).yy / (*m).weight
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * *a * (*m).y / (*m).weight
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * *b * (*m).xy / (*m).weight
        + *a * *a
        + 2 as ::core::ffi::c_int as ::core::ffi::c_double * *a * *b * (*m).x
            / (*m).weight + *b * *b * (*m).xx / (*m).weight;
    *noise = sqrt(fmax(0.0f64, residual));
    return (*a).is_finite() as i32 != 0 && (*b).is_finite() as i32 != 0
        && (*noise).is_finite() as i32 != 0 && *noise < 0.1f64;
}
#[inline]
unsafe extern "C" fn adaptive_trim_observe(
    mut m: *mut AdaptiveTrim,
    mut ut: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut pressure: ::core::ffi::c_double,
    mut incidence: ::core::ffi::c_double,
    mut input: ::core::ffi::c_double,
    mut settled: bool,
) {
    if m.is_null() || ut.is_finite() as i32 == 0 {
        return;
    }
    let mut dt: ::core::ffi::c_double = if (*m).clock_valid as ::core::ffi::c_int != 0 {
        ut - (*m).last_ut
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    (*m).last_ut = ut;
    (*m).clock_valid = true_0 != 0;
    if dt <= 0 as ::core::ffi::c_int as ::core::ffi::c_double || dt > 2.0f64 {
        (*m).weight = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*m).regime_valid = false_0 != 0;
        return;
    }
    let mut decay: ::core::ffi::c_double = exp(-dt / 30.0f64);
    (*m).weight *= decay;
    (*m).x *= decay;
    (*m).y *= decay;
    (*m).xx *= decay;
    (*m).xy *= decay;
    (*m).yy *= decay;
    if !settled || mach.is_finite() as i32 == 0 || pressure.is_finite() as i32 == 0
        || pressure <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || incidence.is_finite() as i32 == 0
        || fabs(incidence) > 180 as ::core::ffi::c_int as ::core::ffi::c_double
        || input.is_finite() as i32 == 0 || fabs(input) >= 0.95f64
    {
        return;
    }
    if (*m).regime_valid as ::core::ffi::c_int != 0
        && (fabs(mach - (*m).mach) > 0.35f64
            || pressure / (*m).pressure
                > 2 as ::core::ffi::c_int as ::core::ffi::c_double
            || pressure / (*m).pressure < 0.5f64)
    {
        (*m).weight = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*m).regime_valid = false_0 != 0;
    }
    if !(*m).regime_valid || (*m).weight < 0.05f64 {
        (*m).yy = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        (*m).xy = (*m).yy;
        (*m).xx = (*m).xy;
        (*m).y = (*m).xx;
        (*m).x = (*m).y;
        (*m).weight = (*m).x;
        (*m).hi = incidence;
        (*m).lo = (*m).hi;
        (*m).mach = mach;
        (*m).pressure = pressure;
        (*m).regime_valid = true_0 != 0;
    }
    (*m).weight += dt;
    (*m).x += dt * incidence;
    (*m).y += dt * input;
    (*m).xx += dt * incidence * incidence;
    (*m).xy += dt * incidence * input;
    (*m).yy += dt * input * input;
    (*m).lo = fmin((*m).lo, incidence);
    (*m).hi = fmax((*m).hi, incidence);
}
#[inline]
unsafe extern "C" fn adaptive_trim_input(
    mut m: *const AdaptiveTrim,
    mut incidence: ::core::ffi::c_double,
    mut input: *mut ::core::ffi::c_double,
) -> bool {
    if m.is_null() || (*m).weight < 0.05f64 || incidence.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut a: ::core::ffi::c_double = 0.;
    let mut b: ::core::ffi::c_double = 0.;
    let mut noise: ::core::ffi::c_double = 0.;
    if adaptive_trim_fit(m, &raw mut a, &raw mut b, &raw mut noise) {
        *input = a + b * incidence;
    } else {
        if fabs(incidence - (*m).x / (*m).weight) > 1.0f64 {
            return false_0 != 0;
        }
        *input = (*m).y / (*m).weight;
    }
    *input = fmax(-0.75f64, fmin(0.75f64, *input));
    return (*input).is_finite() as i32 != 0;
}
#[inline]
unsafe extern "C" fn adaptive_trim_ceiling(
    mut m: *const AdaptiveTrim,
    mut configured_initial: ::core::ffi::c_double,
    mut configured_maximum: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut initial: ::core::ffi::c_double = fmax(
        0.0f64,
        fmin(
            if configured_initial > 0 as ::core::ffi::c_int as ::core::ffi::c_double {
                configured_initial
            } else {
                configured_maximum
            },
            configured_maximum,
        ),
    );
    let mut a: ::core::ffi::c_double = 0.;
    let mut b: ::core::ffi::c_double = 0.;
    let mut noise: ::core::ffi::c_double = 0.;
    if !adaptive_trim_fit(m, &raw mut a, &raw mut b, &raw mut noise) {
        return initial;
    }
    let mut span: ::core::ffi::c_double = (*m).hi - (*m).lo;
    let mut upper: ::core::ffi::c_double = fmin(configured_maximum, (*m).hi + span);
    let mut variance: ::core::ffi::c_double = (*m).xx / (*m).weight
        - pow((*m).x / (*m).weight, 2 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut mean: ::core::ffi::c_double = (*m).x / (*m).weight;
    let mut ceiling: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut incidence: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    while incidence <= upper + 1e-9f64 {
        let mut uncertainty: ::core::ffi::c_double = (noise + 0.03f64)
            * sqrt(
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + pow(
                        incidence - mean,
                        2 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ) / variance,
            );
        if fabs(a + b * incidence)
            + 2 as ::core::ffi::c_int as ::core::ffi::c_double * uncertainty <= 0.75f64
        {
            ceiling = incidence;
        }
        incidence += 0.25f64;
    }
    return fmin(configured_maximum, fmax(0.25f64, ceiling));
}
pub const TERMINAL_LINE_CAPTURE_BANK_DEG: ::core::ffi::c_double = 10.0f64;
pub const TAEM_HAC_REF_N: ::core::ffi::c_int = 480 as ::core::ffi::c_int;
pub const TAEM_HAC_KAPPA_BINS: ::core::ffi::c_int = 15 as ::core::ffi::c_int;
pub const TAEM_HAC_REF_MACH_N: ::core::ffi::c_int = 41 as ::core::ffi::c_int;
pub const M_PI: ::core::ffi::c_double = 3.14159265358979323846f64;
pub const MM305_MINIMUM_GLIDE_SPEED_MPS: ::core::ffi::c_double = 80.0f64;
#[no_mangle]
pub unsafe extern "C" fn mm305_energy_law_handoff_reach(
    mut model: *const TerminalModel,
    mut altitude_above_site_m: ::core::ffi::c_double,
    mut speed_mps: ::core::ffi::c_double,
    mut mass_kg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if model.is_null() || altitude_above_site_m.is_finite() as i32 == 0
        || !(altitude_above_site_m > 0.0f64) || speed_mps.is_finite() as i32 == 0
        || !(speed_mps > 0.0f64) || mass_kg.is_finite() as i32 == 0
        || !(mass_kg > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut geo: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    geo.runway_along_m = -::core::f32::INFINITY as ::core::ffi::c_double;
    geo.altitude_above_runway_m = altitude_above_site_m;
    geo.airspeed_mps = speed_mps;
    geo.ground_speed_mps = speed_mps;
    let mut c: TelContext = TelContext {
        model: ::core::ptr::null::<TerminalModel>(),
        d_station: 0.,
        h_station: 0.,
        v_station: 0.,
        e_station: 0.,
        aoa_max: 0.,
        slope_deg: 0.,
        final_line: false,
    };
    let mut p: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    tel_setup(
        model,
        &raw mut geo,
        mass_kg,
        ::core::f32::NAN as ::core::ffi::c_double,
        0.0f64,
        &raw mut c,
        &raw mut p,
    );
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    let mut cl: ::core::ffi::c_double = 0.;
    let mut cd: ::core::ffi::c_double = 0.;
    tel_air(&raw mut c, p.h, p.v, &raw mut q, &raw mut mach);
    p.aoa = tel_best_ld_aoa(&raw mut c, mach);
    aero_coefficients(&raw const (*model).aero, mach, p.aoa, &raw mut cl, &raw mut cd);
    p.g = if cl > 0.0f64 && cd > 0.0f64 { -atan(cd / cl) } else { 0.0f64 };
    let mut straight: [TelSegment; 1] = [
        TelSegment {
            length: ::core::f32::INFINITY as ::core::ffi::c_double,
            radius: 0.0f64,
        },
    ];
    let mut reach: ::core::ffi::c_double = tel_predict_range(
        &raw mut c,
        &raw mut p,
        1.0f64,
        ::core::f32::INFINITY as ::core::ffi::c_double,
        &raw mut straight as *mut TelSegment,
        1 as ::core::ffi::c_int,
    );
    return if reach.is_finite() as i32 != 0 && reach > 0.0f64 {
        reach
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
}
unsafe extern "C" fn mm305_hac_cached_reference(
    mut model: *const TerminalModel,
    mut live: *const TaemHacLiveAero,
    mut mass_kg: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut x: *mut HacCtx,
) -> *const TaemHacReference {
    fn ref_0_tls() -> *mut TaemHacReference {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<TaemHacReference> = const { ::core::cell::UnsafeCell::new(TaemHacReference {
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
    }) };
        }
        CELL.with(|c| c.get())
    }
    fn ref_model_tls() -> *mut *const TerminalModel {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<*const TerminalModel> = const { ::core::cell::UnsafeCell::new(::core::ptr::null::<TerminalModel>()) };
        }
        CELL.with(|c| c.get())
    }
    fn ref_mass_tls() -> *mut ::core::ffi::c_double {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<::core::ffi::c_double> = const { ::core::cell::UnsafeCell::new(0.) };
        }
        CELL.with(|c| c.get())
    }
    fn ref_live_ut_tls() -> *mut ::core::ffi::c_double {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<::core::ffi::c_double> = const { ::core::cell::UnsafeCell::new(0.) };
        }
        CELL.with(|c| c.get())
    }
    let mut geo: TaemGeometryState = TaemGeometryState {
        runway_along_m: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        runway_cross_m: 0.,
        altitude_above_runway_m: 0.,
        ground_speed_mps: 0.,
        airspeed_mps: 0.,
        course_deg: 0.,
        flight_path_angle_deg: 0.,
        heading_error_deg: 0.,
        curvature_right_per_m: 0.,
    };
    geo.runway_along_m = -::core::f32::INFINITY as ::core::ffi::c_double;
    geo.altitude_above_runway_m = 20000.0f64;
    geo.ground_speed_mps = mach * 300.0f64;
    geo.airspeed_mps = geo.ground_speed_mps;
    let mut pt: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    hac_context(
        x,
        model,
        &raw const (*model).aero,
        live,
        &raw mut geo,
        mass_kg,
        ::core::f32::NAN as ::core::ffi::c_double,
        0.0f64,
        &raw mut pt,
    );
    let mut live_ut: ::core::ffi::c_double = if !live.is_null() {
        (*live).last_ut
    } else {
        0.0f64
    };
    if !(*ref_0_tls()).valid || (*ref_model_tls()) != model || fabs((*ref_mass_tls()) - mass_kg) > 0.01f64 * mass_kg
        || fabs(live_ut - (*ref_live_ut_tls())) > HAC_REF_REBUILD_S * 5.0f64
    {
        hac_reference_build(&raw mut (*ref_0_tls()), x, live_ut);
        (*ref_model_tls()) = model;
        (*ref_mass_tls()) = mass_kg;
        (*ref_live_ut_tls()) = live_ut;
    }
    return &raw mut (*ref_0_tls());
}
#[no_mangle]
pub unsafe extern "C" fn mm305_hac_handoff_state(
    mut model: *const TerminalModel,
    mut live: *const TaemHacLiveAero,
    mut mass_kg: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut altitude_above_site_m: *mut ::core::ffi::c_double,
    mut speed_mps: *mut ::core::ffi::c_double,
    mut station_range_m: *mut ::core::ffi::c_double,
) -> bool {
    if model.is_null() || mass_kg.is_finite() as i32 == 0 || !(mass_kg > 0.0f64)
        || mach.is_finite() as i32 == 0 || !(mach > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut x: HacCtx = HacCtx {
        c: TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        },
        aero: ::core::ptr::null::<AeroTable>(),
        live: ::core::ptr::null::<TaemHacLiveAero>(),
        mass: 0.,
        aoa_supersonic: 0.,
    };
    let mut R: *const TaemHacReference = mm305_hac_cached_reference(
        model,
        live,
        mass_kg,
        mach,
        &raw mut x,
    );
    let mut prev_mach: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < TAEM_HAC_REF_N {
        let mut q: ::core::ffi::c_double = 0.;
        let mut m: ::core::ffi::c_double = 0.;
        tel_air(
            &raw mut x.c,
            (*R).h_eq[i as usize],
            (*R).v_eq[i as usize],
            &raw mut q,
            &raw mut m,
        );
        if i > 0 as ::core::ffi::c_int && m >= mach && prev_mach < mach {
            let mut f: ::core::ffi::c_double = (mach - prev_mach)
                / fmax(1e-9f64, m - prev_mach);
            if !altitude_above_site_m.is_null() {
                *altitude_above_site_m = (*R)
                    .h_eq[(i - 1 as ::core::ffi::c_int) as usize]
                    + f
                        * ((*R).h_eq[i as usize]
                            - (*R).h_eq[(i - 1 as ::core::ffi::c_int) as usize]);
            }
            if !speed_mps.is_null() {
                *speed_mps = (*R).v_eq[(i - 1 as ::core::ffi::c_int) as usize]
                    + f
                        * ((*R).v_eq[i as usize]
                            - (*R).v_eq[(i - 1 as ::core::ffi::c_int) as usize]);
            }
            if !station_range_m.is_null() {
                *station_range_m = ((i - 1 as ::core::ffi::c_int)
                    as ::core::ffi::c_double + f) * (*R).ds;
            }
            return true_0 != 0;
        }
        prev_mach = m;
        i += 1;
    }
    return false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_hac_handoff_energy_error(
    mut model: *const TerminalModel,
    mut live: *const TaemHacLiveAero,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut error_m: *mut ::core::ffi::c_double,
    mut band_m: *mut ::core::ffi::c_double,
) -> bool {
    if model.is_null() || geo.is_null() || mass_kg.is_finite() as i32 == 0
        || !(mass_kg > 0.0f64) || (*geo).runway_along_m.is_finite() as i32 == 0
        || (*geo).runway_cross_m.is_finite() as i32 == 0
        || (*geo).heading_error_deg.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut x: HacCtx = HacCtx {
        c: TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        },
        aero: ::core::ptr::null::<AeroTable>(),
        live: ::core::ptr::null::<TaemHacLiveAero>(),
        mass: 0.,
        aoa_supersonic: 0.,
    };
    let mut R: *const TaemHacReference = mm305_hac_cached_reference(
        model,
        live,
        mass_kg,
        2.5f64,
        &raw mut x,
    );
    if !(*R).valid {
        return false_0 != 0;
    }
    let mut s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    hac_context(
        &raw mut x,
        model,
        &raw const (*model).aero,
        live,
        geo,
        mass_kg,
        ::core::f32::NAN as ::core::ffi::c_double,
        0.0f64,
        &raw mut s,
    );
    fn H_tls() -> *mut TaemHacLawState {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<TaemHacLawState> = const { ::core::cell::UnsafeCell::new(TaemHacLawState {
        initialized: false,
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
    }) };
        }
        CELL.with(|c| c.get())
    }
    (*H_tls()).ref_0 = *R;
    (*H_tls()).rf = clampd(
        x.c.v_station * x.c.v_station / (TEL_G0 * tan(HAC_BANK_NOMINAL * DEG2RAD)),
        HAC_RF_MIN,
        HAC_RF_MAX,
    );
    let mut s_go: ::core::ffi::c_double = 0.;
    let mut e_tgt: ::core::ffi::c_double = 0.;
    let mut bearing: ::core::ffi::c_double = 0.;
    hac_drag_path(
        &raw mut (*H_tls()),
        &raw mut x,
        &raw mut s,
        0 as ::core::ffi::c_int,
        &raw mut s_go,
        &raw mut e_tgt,
        &raw mut bearing,
    );
    if e_tgt.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    if !error_m.is_null() {
        *error_m = hac_energy(&raw mut s) - e_tgt;
    }
    if !band_m.is_null() {
        *band_m = hac_band_high(&raw mut x, e_tgt);
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_hac_spiral_energy_error(
    mut model: *const TerminalModel,
    mut live: *const TaemHacLiveAero,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut error_m: *mut ::core::ffi::c_double,
    mut band_m: *mut ::core::ffi::c_double,
    mut psi_out: *mut ::core::ffi::c_double,
) -> bool {
    let (mut tx, mut ty, mut tc) = (0.0f64, 0.0f64, 0.0f64);
    mm305_hac_spiral_tangent(model, live, geo, mass_kg, error_m, band_m, psi_out, &mut tx, &mut ty, &mut tc)
}
#[no_mangle]
pub unsafe extern "C" fn mm305_hac_spiral_tangent(
    mut model: *const TerminalModel,
    mut live: *const TaemHacLiveAero,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut error_m: *mut ::core::ffi::c_double,
    mut band_m: *mut ::core::ffi::c_double,
    mut psi_out: *mut ::core::ffi::c_double,
    mut tangent_x: *mut ::core::ffi::c_double,
    mut tangent_y: *mut ::core::ffi::c_double,
    mut tangent_course: *mut ::core::ffi::c_double,
) -> bool {
    if model.is_null() || geo.is_null() || mass_kg.is_finite() as i32 == 0
        || !(mass_kg > 0.0f64) || (*geo).runway_along_m.is_finite() as i32 == 0
        || (*geo).runway_cross_m.is_finite() as i32 == 0
        || (*geo).heading_error_deg.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut x: HacCtx = HacCtx {
        c: TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        },
        aero: ::core::ptr::null::<AeroTable>(),
        live: ::core::ptr::null::<TaemHacLiveAero>(),
        mass: 0.,
        aoa_supersonic: 0.,
    };
    let mut R: *const TaemHacReference = mm305_hac_cached_reference(
        model,
        live,
        mass_kg,
        2.5f64,
        &raw mut x,
    );
    if !(*R).valid {
        return false_0 != 0;
    }
    let mut s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    hac_context(
        &raw mut x,
        model,
        &raw const (*model).aero,
        live,
        geo,
        mass_kg,
        ::core::f32::NAN as ::core::ffi::c_double,
        0.0f64,
        &raw mut s,
    );
    fn H_tls() -> *mut TaemHacLawState {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<TaemHacLawState> = const { ::core::cell::UnsafeCell::new(TaemHacLawState {
        initialized: false,
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
    }) };
        }
        CELL.with(|c| c.get())
    }
    (*H_tls()).ref_0 = *R;
    (*H_tls()).rf = clampd(
        x.c.v_station * x.c.v_station / (TEL_G0 * tan(HAC_BANK_NOMINAL * DEG2RAD)),
        HAC_RF_MIN,
        HAC_RF_MAX,
    );
    // The spiral side: the side whose MM305 route turns furthest around the HAC.
    let mut e_tgt: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut spiral: HacPrediction = HacPrediction {
        valid: false,
        s_go: 0.,
        psi_t: -1.0f64,
        tx: 0.,
        ty: 0.,
        tangent_dist: 0.,
        acq_turn: 0.,
        acq_arc: 0.,
        hac_arc: 0.,
        turn_cost: 0.,
        e_ref: 0.,
    };
    let mut spiral_side: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut spiral_rf: ::core::ffi::c_double = 0.0f64;
    let mut side: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while side <= 1 as ::core::ffi::c_int {
        let mut rf: ::core::ffi::c_double = (*H_tls()).rf;
        let mut pred: HacPrediction = hac_size_side(&raw mut x, R, side, &raw mut s, &raw mut rf);
        if pred.valid && pred.psi_t <= 2.0f64 * TEL_PI - 0.05f64 && pred.psi_t > spiral.psi_t {
            spiral = pred;
            spiral_side = side;
            spiral_rf = rf;
        }
        side += 2 as ::core::ffi::c_int;
    }
    let mut spiral_course: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if spiral.valid {
        let mut d_station: ::core::ffi::c_double = x.c.d_station;
        let mut ang: ::core::ffi::c_double = atan2(
            spiral.ty - spiral_side as ::core::ffi::c_double * spiral_rf,
            spiral.tx + d_station,
        );
        spiral_course = ang + spiral_side as ::core::ffi::c_double * 0.5f64 * TEL_PI;
        // The requirement is what MM305's drag phase will fly: its drag path held
        // to the spiral side, on the HAC radius MM305's side sizing gives it, with
        // the same supersonic allowance.
        (*H_tls()).rf = spiral_rf;
        let mut s_go: ::core::ffi::c_double = 0.;
        let mut bearing: ::core::ffi::c_double = 0.;
        hac_drag_path(&raw mut (*H_tls()), &raw mut x, &raw mut s, spiral_side, &raw mut s_go, &raw mut e_tgt, &raw mut bearing);
        if !psi_out.is_null() {
            *psi_out = spiral.psi_t;
        }
        // Tangent point of the spiral-side route and its direction of travel
        // (MM305 frame: x along, y = -cross, angle counter-clockwise from the runway).
        *tangent_x = spiral.tx;
        *tangent_y = spiral.ty;
        *tangent_course = spiral_course;
    }
    if e_tgt.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    if !error_m.is_null() {
        *error_m = hac_energy(&raw mut s) - e_tgt;
    }
    if !band_m.is_null() {
        *band_m = hac_band_high(&raw mut x, e_tgt);
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_hac_reference_energy(
    mut model: *const TerminalModel,
    mut live: *const TaemHacLiveAero,
    mut mass_kg: ::core::ffi::c_double,
    mut station_range_m: ::core::ffi::c_double,
    mut energy_m: *mut ::core::ffi::c_double,
) -> bool {
    if model.is_null() || mass_kg.is_finite() as i32 == 0 || !(mass_kg > 0.0f64)
        || station_range_m.is_finite() as i32 == 0 || !(station_range_m >= 0.0f64)
        || energy_m.is_null()
    {
        return false_0 != 0;
    }
    let mut x: HacCtx = HacCtx {
        c: TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        },
        aero: ::core::ptr::null::<AeroTable>(),
        live: ::core::ptr::null::<TaemHacLiveAero>(),
        mass: 0.,
        aoa_supersonic: 0.,
    };
    let mut R: *const TaemHacReference = mm305_hac_cached_reference(
        model,
        live,
        mass_kg,
        2.5f64,
        &raw mut x,
    );
    if !(*R).valid
        || station_range_m
            > (TAEM_HAC_REF_N - 1 as ::core::ffi::c_int) as ::core::ffi::c_double
                * (*R).ds
    {
        return false_0 != 0;
    }
    hac_reference_at(
        R,
        station_range_m,
        energy_m,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    return (*energy_m).is_finite() as i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_hac_observe_aero(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
) {
    if !g.is_null() && !t.is_null() && !(*g).entry_terminal_model.is_null() {
        hac_live_aero_observe(
            &raw mut (*g).taem_hac_live_aero,
            &raw const (*(*g).entry_terminal_model).aero,
            t,
            0.0f64,
        );
    }
}
#[no_mangle]
pub static mut terminal_energy_commit_tolerance: ::core::ffi::c_double = ::core::f32::NAN
    as ::core::ffi::c_double;
#[no_mangle]
pub unsafe extern "C" fn terminal_test_speed_floor(
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    return if !v.is_null() {
        fmax(65.0f64, (*v).touchdown_speed * 0.88f64)
    } else {
        65.0f64
    };
}
#[no_mangle]
pub unsafe extern "C" fn terminal_projected_lift_accel_at_aoa(
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut aoa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || v.is_null() || aoa.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut target: ::core::ffi::c_double = clampd(
        aoa,
        0.0f64,
        (*v).maximum_angle_of_attack,
    );
    let mut lf: ::core::ffi::c_double = 0.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        target,
        v,
        &raw mut lf,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    let mut modeled: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if (*t).dynamic_pressure.is_finite() as i32 != 0 && (*t).dynamic_pressure >= 0.0f64
        && aero.ballistic_coefficient.is_finite() as i32 != 0
        && aero.ballistic_coefficient > DBL_MIN
        && aero.lift_to_drag.is_finite() as i32 != 0 && aero.lift_to_drag >= 0.0f64
    {
        modeled = (*t).dynamic_pressure / aero.ballistic_coefficient * aero.lift_to_drag
            * fabs(lf);
    }
    let mut incidence: ::core::ffi::c_double = hypot(
        (*t).angle_of_attack,
        (*t).sideslip,
    );
    let mut measured: ::core::ffi::c_double = if (*t).mass > DBL_MIN
        && (*t).lift_force.is_finite() as i32 != 0 && (*t).lift_force >= 0.0f64
    {
        (*t).lift_force / (*t).mass
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut same_incidence: ::core::ffi::c_double = sqrt(DBL_EPSILON)
        * fmax(1.0f64, fabs(incidence));
    if measured.is_finite() as i32 != 0 && fabs(target - incidence) <= same_incidence {
        return measured;
    }
    return if modeled.is_finite() as i32 != 0 && modeled >= 0.0f64 {
        modeled
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
}
unsafe extern "C" fn guidance_begin_mm305_restart(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut test_mode: bool,
    mut message: *mut ::core::ffi::c_char,
    mut message_size: size_t,
) -> bool {
    if g.is_null() || t.is_null() || planet.is_null() || cfg.is_null()
        || course.is_finite() as i32 == 0 || !((*t).radar_altitude > 0.0f64)
        || !((*t).true_air_speed > 0.0f64) || (*t).ut.is_finite() as i32 == 0
        || (*t).mean_altitude.is_finite() as i32 == 0
    {
        if !message.is_null() && message_size != 0 {
            snprintf(
                message,
                message_size,
                b"MM305 restart requires a valid airborne state and guidance configuration.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    guidance_machine_init(g);
    guidance_set_engaged(g, true_0 != 0);
    terminal_glide_initialize(g, &raw const (*cfg).vehicle, &raw const (*cfg).guidance);
    (*g).has_burn_command_started = true_0 != 0;
    (*g).deorbit_burn_completed = true_0 != 0;
    (*g).atmospheric_interface_crossed = true_0 != 0;
    (*g).terminal_region_entered = true_0 != 0;
    (*g).terminal_test_capture_active = test_mode;
    (*g).hac_radius = (*cfg).guidance.hac_radius;
    (*g).hac_side = 0.0f64;
    (*g).terminal_final_handoff_latched = false_0 != 0;
    (*g).terminal_final_handoff_distance = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).terminal_final_handoff_slope_deg = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).terminal_final_handoff_speed_mps = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).terminal_path_committed = false_0 != 0;
    (*g).hac_completed = false_0 != 0;
    (*g).final_approach_captured = false_0 != 0;
    (*g).terminal_final_low_energy = false_0 != 0;
    if !taem_exec_enter(
        g,
        t,
        &raw const (*cfg).guidance,
        planet,
        aero,
        cfg,
        true_0 != 0,
    ) {
        guidance_machine_init(g);
        if !message.is_null() && message_size != 0 {
            snprintf(
                message,
                message_size,
                b"MM305 live state could not initialize TAEM ownership.\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    (*g).phase = PHASE_TAEM;
    if !message.is_null() && message_size != 0 {
        snprintf(
            message,
            message_size,
            if test_mode as ::core::ffi::c_int != 0 {
                b"MM305 checkpoint accepted; HAC selection and Final delivery remain subject to the live guidance gates.\0"
                    as *const u8 as *const ::core::ffi::c_char
            } else {
                b"MM305 continuation adopted from the live state; route selection and Final delivery remain subject to the normal guidance gates.\0"
                    as *const u8 as *const ::core::ffi::c_char
            },
        );
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_begin_mm305_continuation(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut message: *mut ::core::ffi::c_char,
    mut message_size: size_t,
) -> bool {
    return guidance_begin_mm305_restart(
        g,
        t,
        course,
        planet,
        aero,
        cfg,
        false_0 != 0,
        message,
        message_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn guidance_begin_hac_test(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut message: *mut ::core::ffi::c_char,
    mut message_size: size_t,
) -> bool {
    return guidance_begin_mm305_restart(
        g,
        t,
        course,
        planet,
        aero,
        cfg,
        true_0 != 0,
        message,
        message_size,
    );
}
unsafe extern "C" fn mm305_trim_observe(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
) {
    if g.is_null() || t.is_null() || (*g).diagnostic_shadow as ::core::ffi::c_int != 0 {
        return;
    }
    let mut rate: ::core::ffi::c_double = if (*t).has_angle_of_attack_rate
        as ::core::ffi::c_int != 0 && (*t).angle_of_attack_rate.is_finite() as i32 != 0
    {
        (*t).angle_of_attack_rate
    } else {
        (*t).pitch_rate
    };
    let mut error: ::core::ffi::c_double = (*g).mm305_command_aoa_rad * RAD2DEG
        - (*t).angle_of_attack;
    let mut settled: bool = (*g).mm305_command_valid as ::core::ffi::c_int != 0
        && (*t).has_controls as ::core::ffi::c_int != 0 && rate.is_finite() as i32 != 0
        && fabs(rate) < 0.3f64 && error.is_finite() as i32 != 0 && fabs(error) < 0.5f64;
    adaptive_trim_observe(
        &raw mut (*g).mm305_trim,
        (*t).ut,
        (*t).mach,
        (*t).dynamic_pressure,
        (*t).angle_of_attack,
        (*t).control_pitch,
        settled,
    );
}
unsafe extern "C" fn terminal_live_state(
    mut g: *const GuidanceMachine,
    mut state: *const VehicleState,
    mut telemetry: *const Telemetry,
    mut model: *const TerminalModel,
) -> TerminalDynamicState {
    let mut attitude: AttitudeModel = (*model).attitude;
    attitude.aoa_rad = (*telemetry).angle_of_attack * DEG2RAD;
    attitude.bank_rad = (*telemetry).roll * DEG2RAD;
    attitude.aoa_rate_rad_s = (if (*telemetry).has_angle_of_attack_rate
        as ::core::ffi::c_int != 0
        && (*telemetry).angle_of_attack_rate.is_finite() as i32 != 0
    {
        (*telemetry).angle_of_attack_rate
    } else {
        (*telemetry).pitch_rate
    }) * DEG2RAD;
    attitude.bank_rate_rad_s = (*telemetry).roll_rate * DEG2RAD;
    attitude.requested_aoa_rad = attitude.aoa_rad;
    attitude.cmd_aoa_rad = attitude.requested_aoa_rad;
    attitude.requested_bank_rad = attitude.bank_rad;
    attitude.cmd_bank_rad = attitude.requested_bank_rad;
    if !g.is_null() && (*g).mm305_command_valid as ::core::ffi::c_int != 0
        && (*g).mm305_command_ut.is_finite() as i32 != 0
        && (*telemetry).ut - (*g).mm305_command_ut >= 0.0f64
        && (*telemetry).ut - (*g).mm305_command_ut <= 1.0f64
    {
        attitude.requested_bank_rad = (*g).mm305_command_bank_rad;
        attitude.cmd_bank_rad = attitude.requested_bank_rad;
        attitude.requested_aoa_rad = (*g).mm305_command_aoa_rad;
        attitude.cmd_aoa_rad = attitude.requested_aoa_rad;
    }
    let mut trim_ceiling: ::core::ffi::c_double = adaptive_trim_ceiling(
        if !g.is_null() {
            &raw const (*g).mm305_trim
        } else {
            ::core::ptr::null::<AdaptiveTrim>()
        },
        (*model).vehicle.terminal_maximum_lift_angle_of_attack,
        (*model).vehicle.maximum_angle_of_attack,
    );
    return TerminalDynamicState {
        position_i_m: Vec3 {
            x: (*state).position.x,
            y: (*state).position.y,
            z: (*state).position.z,
        },
        velocity_i_mps: Vec3 {
            x: (*state).velocity.x,
            y: (*state).velocity.y,
            z: (*state).velocity.z,
        },
        attitude: attitude,
        mass_kg: (*state).mass,
        ut_s: (*state).ut,
        trim_aoa_ceiling_rad: if trim_ceiling.is_finite() as i32 != 0 {
            trim_ceiling * DEG2RAD
        } else {
            0.0f64
        },
    };
}
unsafe extern "C" fn mm305_reciprocal_model(
    mut model: *const TerminalModel,
    mut out: *mut TerminalModel,
) {
    *out = *model;
    (*out).site = runway_reciprocal_site(
        &raw const (*model).site,
        (*model).world.radius_m,
    );
}
unsafe extern "C" fn mm305_model_for_end(
    mut model: *const TerminalModel,
    mut runway_end: ::core::ffi::c_int,
) -> *const TerminalModel {
    fn reciprocal_tls() -> *mut TerminalModel {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<TerminalModel> = const { ::core::cell::UnsafeCell::new(TerminalModel {
        schema_version: 0,
        snapshot_id: 0,
        atmosphere_source_digest: 0,
        aero_source_digest: 0,
        aero_book_source_digest: 0,
        attitude_source_digest: 0,
        replay_validated: false,
        planet_frame_aligned: false,
        world: KerbinWorld {
            radius_m: 0.,
            mu_m3_s2: 0.,
            rotation_rate_rad_s: 0.,
            atmosphere_top_m: 0.,
            rotation_phase_rad_at_ut0: 0.,
            orbital_rate_rad_s: 0.,
            solar_phase_rad_at_ut0: 0.,
            stock_spatial_atmosphere: false,
            atmosphere: AtmosphereTable {
                p: [AtmospherePoint {
                    altitude_m: 0.,
                    sample: AtmosphereSample {
                        density_kg_m3: 0.,
                        pressure_pa: 0.,
                        temperature_k: 0.,
                        speed_of_sound_mps: 0.,
                    },
                }; 2048],
                count: 0,
            },
        },
        aero: AeroTable {
            mach: [0.; 40],
            alpha_deg: [0.; 40],
            cl: [[0.; 40]; 40],
            cd: [[0.; 40]; 40],
            mach_count: 0,
            alpha_count: 0,
            reference_area_m2: 0.,
            book: [AeroBookPoint {
                q_pa: 0.,
                mach: 0.,
                alpha_deg: 0.,
                lift_per_q_m2: 0.,
                drag_per_q_m2: 0.,
                support: 0.,
            }; 2048],
            book_count: 0,
            book_enabled: false,
            lift_scale: 0.,
            drag_scale: 0.,
        },
        attitude: AttitudeModel {
            aoa_rad: 0.,
            bank_rad: 0.,
            aoa_rate_rad_s: 0.,
            bank_rate_rad_s: 0.,
            requested_aoa_rad: 0.,
            requested_bank_rad: 0.,
            cmd_aoa_rad: 0.,
            cmd_bank_rad: 0.,
            pitch_wn: 0.,
            pitch_zeta: 0.,
            roll_wn: 0.,
            roll_zeta: 0.,
            max_pitch_rate_rad_s: 0.,
            max_roll_rate_rad_s: 0.,
            max_pitch_accel_rad_s2: 0.,
            max_roll_accel_rad_s2: 0.,
            pitch_full_authority_q_pa: 0.,
            roll_full_authority_q_pa: 0.,
            direct: false,
            direct_body_coordinates: false,
            body_rates_initialized: false,
            body_pitch_rate_rad_s: 0.,
            body_roll_rate_rad_s: 0.,
            body_yaw_rate_rad_s: 0.,
            input_pitch: 0.,
            input_roll: 0.,
            input_yaw: 0.,
            sideslip_rad: 0.,
            sideslip_rate_rad_s: 0.,
            direct_min_q_pa: 0.,
            direct_pitch_accel_per_kpa: 0.,
            direct_pitch_accel_max: 0.,
            direct_pitch_stiffness_per_kpa: 0.,
            direct_trim_aoa_rad: 0.,
            direct_pitch_damping_ratio: 0.,
            direct_roll_accel_per_kpa: 0.,
            direct_roll_accel_max: 0.,
            direct_roll_damping_s_inv: 0.,
            direct_yaw_accel_per_kpa: 0.,
            direct_yaw_accel_max: 0.,
            direct_yaw_stiffness_per_kpa: 0.,
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
    }) };
        }
        CELL.with(|c| c.get())
    }
    fn reciprocal_snapshot_id_tls() -> *mut uint64_t {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<uint64_t> = const { ::core::cell::UnsafeCell::new(0) };
        }
        CELL.with(|c| c.get())
    }
    fn reciprocal_valid_tls() -> *mut bool {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<bool> = const { ::core::cell::UnsafeCell::new(false) };
        }
        CELL.with(|c| c.get())
    }
    if runway_end != 1 as ::core::ffi::c_int {
        return model;
    }
    if !(*reciprocal_valid_tls()) || (*reciprocal_snapshot_id_tls()) != (*model).snapshot_id
        || (*reciprocal_tls()).site.latitude
            != runway_reciprocal_site(&raw const (*model).site, (*model).world.radius_m)
                .latitude
    {
        mm305_reciprocal_model(model, &raw mut (*reciprocal_tls()));
        (*reciprocal_snapshot_id_tls()) = (*model).snapshot_id;
        (*reciprocal_valid_tls()) = true_0 != 0;
    }
    return &raw mut (*reciprocal_tls());
}
unsafe extern "C" fn mm305_scale_model(
    mut m: *mut TerminalModel,
    mut observed_mach: ::core::ffi::c_double,
    mut lift_scale: ::core::ffi::c_double,
    mut drag_scale: ::core::ffi::c_double,
) {
    aero_apply_mach_observation(
        &raw mut (*m).aero,
        observed_mach,
        lift_scale,
        drag_scale,
    );
}
unsafe extern "C" fn mm305_wall_seconds() -> ::core::ffi::c_double {
    let mut ts: timespec = timespec { tv_sec: 0, tv_nsec: 0 };
    if clock_gettime(CLOCK_MONOTONIC, &raw mut ts) != 0 as ::core::ffi::c_int {
        return 0.0f64;
    }
    return ts.tv_sec as ::core::ffi::c_double
        + ts.tv_nsec as ::core::ffi::c_double / 1e9f64;
}
unsafe extern "C" fn mm305_candidate_ok(mut c: *const TaemFixedHacCandidate) -> bool {
    return (*c).status as ::core::ffi::c_uint
        == TAEM_PLAN_UNQUALIFIED as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*c).route_built as ::core::ffi::c_int != 0
        && (*c).replay.path_constraints_ok as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn mm305_candidate_delivery_ok(
    mut model: *const TerminalModel,
    mut request: *const Mm305PlanRequest,
    mut c: *mut TaemFixedHacCandidate,
) -> bool {
    if !mm305_candidate_ok(c) {
        return false_0 != 0;
    }
    let mut delivery: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    if delivery.is_null() {
        return false_0 != 0;
    }
    *delivery = *model;
    (*delivery).guidance.final_approach_distance = -(*c).route.alignment_along_m;
    (*delivery).guidance.final_glide_slope = (*c).route.profile_final_slope_deg;
    let mut evaluation: TaemTerminalEvaluation = TaemTerminalEvaluation {
        valid: 0 as ::core::ffi::c_int != 0,
        feasible: false,
        block_reason: TAEM_TERMINAL_BLOCK_NONE,
    };
    (*c).replay.final_interface_qualified = terminal_native_delivery_admissible(
        delivery,
        &raw mut (*c).replay.final_state,
        &raw const (*request).final_guidance,
        &raw mut evaluation,
    );
    free(delivery as *mut ::core::ffi::c_void);
    if !(*c).replay.final_interface_qualified {
        (*c).reason = taem_terminal_block_reason_string(evaluation.block_reason);
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_plan_result_fresh(
    mut result: *const Mm305PlanResult,
    mut t: *const Telemetry,
) -> bool {
    if result.is_null() || t.is_null() || !(*result).valid
        || (*t).ut.is_finite() as i32 == 0
        || (*result).request_ut.is_finite() as i32 == 0
        || (*result).initial_state.ut_s.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut age: ::core::ffi::c_double = (*t).ut - (*result).request_ut;
    return age >= 0.0f64 && age <= 1.0f64
        && fabs((*result).initial_state.ut_s - (*result).request_ut) <= 1e-6f64;
}
unsafe extern "C" fn mm305_final_alignment_speed_for_distance(
    mut model: *const TerminalModel,
    mut distance: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if !model.is_null() {
        final_alignment_contract(
                distance,
                &raw const (*model).site,
                &raw const (*model).vehicle,
                &raw const (*model).guidance,
            )
            .speed_mps
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
}
#[no_mangle]
pub unsafe extern "C" fn mm305_final_alignment_distance(
    mut g: *const GuidanceMachine,
    mut model: *const TerminalModel,
    mut state: *const TerminalDynamicState,
) -> ::core::ffi::c_double {
    if model.is_null() {
        return ::core::f32::NAN as ::core::ffi::c_double;
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
    let mut altitude: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut speed: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if !state.is_null()
        && taem_geometry_state(model, state, &raw mut geometry) as ::core::ffi::c_int
            != 0
    {
        altitude = geometry.altitude_above_runway_m + (*model).site.altitude;
        speed = geometry.airspeed_mps;
    }
    return final_alignment_distance_for_state(
        g,
        altitude,
        speed,
        &raw const (*model).site,
        &raw const (*model).vehicle,
        &raw const (*model).guidance,
    );
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_adopt_best_effort_route(
    mut g: *mut GuidanceMachine,
    mut model: *const TerminalModel,
    mut current_airspeed_mps: ::core::ffi::c_double,
) -> bool {
    if g.is_null() || model.is_null()
        || (*g).phase as ::core::ffi::c_uint
            != PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*g).mm305_route_committed as ::core::ffi::c_int != 0
        || !(*g).mm305_acquisition_route_valid || !(*g).mm305_acquisition_route.valid
        || (*g).mm305_acquisition_route.count < 3 as size_t
        || (*g).mm305_acquisition_route.alignment_along_m.is_finite() as i32 == 0
        || current_airspeed_mps.is_finite() as i32 == 0
        || !(current_airspeed_mps > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut route: TaemRoute = (*g).mm305_acquisition_route;
    let mut final_distance: ::core::ffi::c_double = -route.alignment_along_m;
    if !(final_distance >= 500.0f64) || final_distance.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut final_slope: ::core::ffi::c_double = route.profile_final_slope_deg;
    if !(final_slope >= 3.0f64 && final_slope <= 45.0f64)
        || final_slope.is_finite() as i32 == 0
    {
        final_slope = (*model).guidance.final_glide_slope;
    }
    let mut speed_floor: ::core::ffi::c_double = fmax(
        MM305_MINIMUM_GLIDE_SPEED_MPS,
        fmax((*model).vehicle.minimum_safe_speed, (*model).vehicle.touchdown_speed),
    );
    let mut speed_start: ::core::ffi::c_double = fmax(speed_floor, current_airspeed_mps);
    let mut speed_finish: ::core::ffi::c_double = fmax(
        speed_floor,
        mm305_final_alignment_speed_for_distance(model, final_distance),
    );
    if speed_start.is_finite() as i32 == 0 || speed_finish.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    route.profile_speed_tabulated = true_0 != 0;
    let mut i: size_t = 0 as size_t;
    while i < TAEM_ROUTE_PROFILE_LUT_POINTS as size_t {
        let mut fraction: ::core::ffi::c_double = i as ::core::ffi::c_double
            / (TAEM_ROUTE_PROFILE_LUT_POINTS - 1 as ::core::ffi::c_int)
                as ::core::ffi::c_double;
        route.profile_speed_lut[i as usize] = (speed_start
            + fraction * (speed_finish - speed_start)) as ::core::ffi::c_float;
        i = i.wrapping_add(1);
    }
    (*g).mm305_route = route;
    (*g).mm305_route_cursor = if (*g).mm305_acquisition_route_cursor < route.count {
        (*g).mm305_acquisition_route_cursor
    } else {
        0 as size_t
    };
    (*g).mm305_route_committed = true_0 != 0;
    (*g).mm305_best_effort_route_active = true_0 != 0;
    (*g).mm305_direct_approach_degraded = false_0 != 0;
    (*g).mm305_model_snapshot_id = (*model).snapshot_id;
    (*g).mm305_last_route_ut = (*g).mm305_acquisition_route_ut;
    (*g).mm305_acquisition_route_valid = false_0 != 0;
    (*g).mm305_planning_needed = false_0 != 0;
    (*g).mm305_hac_exit_reached = false_0 != 0;
    (*g).mm305_runway_alignment_active = false_0 != 0;
    (*g).runway_end_committed = true_0 != 0;
    (*g).hac_side = route.side;
    (*g).hac_side_selected = true_0 != 0;
    (*g).hac_radius = route.hac.radius_m;
    (*g).hac_remaining = taem_route_remaining_at_index(
        &raw mut route,
        (*g).mm305_route_cursor,
    );
    (*g).hac_progress_valid = (*g).hac_remaining.is_finite() as i32 != 0;
    (*g).hac_captured = true_0 != 0;
    (*g).hac_completed = false_0 != 0;
    (*g).terminal_path_kind = TERMINAL_PATH_HAC;
    (*g).terminal_path_committed = true_0 != 0;
    (*g).terminal_final_handoff_latched = true_0 != 0;
    (*g).terminal_final_handoff_distance = final_distance;
    (*g).terminal_final_handoff_slope_deg = final_slope;
    (*g).terminal_final_handoff_speed_mps = mm305_final_alignment_speed_for_distance(
        model,
        final_distance,
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_apply_final_alignment(
    mut out: *mut TerminalModel,
    mut base: *const TerminalModel,
    mut final_distance_m: ::core::ffi::c_double,
) {
    if out.is_null() || base.is_null() {
        return;
    }
    if !(final_distance_m >= 500.0f64) || final_distance_m.is_finite() as i32 == 0 {
        return;
    }
    let mut contract: FinalAlignmentContract = final_alignment_contract(
        final_distance_m,
        &raw const (*base).site,
        &raw const (*base).vehicle,
        &raw const (*base).guidance,
    );
    if !contract.valid {
        return;
    }
    (*out).guidance.final_approach_distance = contract.distance_m;
    (*out).guidance.final_glide_slope = contract.slope_deg;
    (*out).guidance.final_alignment_speed = contract.speed_mps;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_prepare_planning_model(
    mut out: *mut TerminalModel,
    mut base: *const TerminalModel,
    mut final_distance_m: ::core::ffi::c_double,
    mut scale_mach: ::core::ffi::c_double,
    mut lift_scale: ::core::ffi::c_double,
    mut drag_scale: ::core::ffi::c_double,
    mut trim_aoa_ceiling_rad: ::core::ffi::c_double,
) {
    if out.is_null() || base.is_null() {
        return;
    }
    *out = *base;
    mm305_apply_final_alignment(out, base, final_distance_m);
    mm305_scale_model(out, scale_mach, lift_scale, drag_scale);
    if trim_aoa_ceiling_rad > 0.0f64 && trim_aoa_ceiling_rad.is_finite() as i32 != 0 {
        (*out).vehicle.maximum_angle_of_attack = fmin(
            (*out).vehicle.maximum_angle_of_attack,
            trim_aoa_ceiling_rad * RAD2DEG,
        );
    }
}
#[no_mangle]
pub unsafe extern "C" fn mm305_plan(
    mut model: *const TerminalModel,
    mut request: *const Mm305PlanRequest,
) -> Mm305PlanResult {
    let mut out: Mm305PlanResult = Mm305PlanResult {
        valid: false,
        found: false,
        request_ut: 0.,
        initial_state: TerminalDynamicState {
            position_i_m: Vec3 { x: 0., y: 0., z: 0. },
            velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
            attitude: AttitudeModel {
                aoa_rad: 0.,
                bank_rad: 0.,
                aoa_rate_rad_s: 0.,
                bank_rate_rad_s: 0.,
                requested_aoa_rad: 0.,
                requested_bank_rad: 0.,
                cmd_aoa_rad: 0.,
                cmd_bank_rad: 0.,
                pitch_wn: 0.,
                pitch_zeta: 0.,
                roll_wn: 0.,
                roll_zeta: 0.,
                max_pitch_rate_rad_s: 0.,
                max_roll_rate_rad_s: 0.,
                max_pitch_accel_rad_s2: 0.,
                max_roll_accel_rad_s2: 0.,
                pitch_full_authority_q_pa: 0.,
                roll_full_authority_q_pa: 0.,
                direct: false,
                direct_body_coordinates: false,
                body_rates_initialized: false,
                body_pitch_rate_rad_s: 0.,
                body_roll_rate_rad_s: 0.,
                body_yaw_rate_rad_s: 0.,
                input_pitch: 0.,
                input_roll: 0.,
                input_yaw: 0.,
                sideslip_rad: 0.,
                sideslip_rate_rad_s: 0.,
                direct_min_q_pa: 0.,
                direct_pitch_accel_per_kpa: 0.,
                direct_pitch_accel_max: 0.,
                direct_pitch_stiffness_per_kpa: 0.,
                direct_trim_aoa_rad: 0.,
                direct_pitch_damping_ratio: 0.,
                direct_roll_accel_per_kpa: 0.,
                direct_roll_accel_max: 0.,
                direct_roll_damping_s_inv: 0.,
                direct_yaw_accel_per_kpa: 0.,
                direct_yaw_accel_max: 0.,
                direct_yaw_stiffness_per_kpa: 0.,
            },
            mass_kg: 0.,
            ut_s: 0.,
            trim_aoa_ceiling_rad: 0.,
        },
        model_snapshot_id: 0,
        candidate: TaemFixedHacCandidate {
            status: TAEM_PLAN_INFEASIBLE,
            side: 0.,
            runway_end: 0,
            route_built: false,
            final_tail_qualified: false,
            required_lateral_accel_mps2: 0.,
            available_lateral_accel_mps2: 0.,
            quality_score: 0.,
            route: TaemRoute {
                valid: false,
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
            },
            replay: TerminalSolverResult {
                status: TERMINAL_SOLVER_QUALIFIED,
                path_constraints_ok: false,
                final_interface_qualified: false,
                final_state: TerminalDynamicState {
                    position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                    velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                    attitude: AttitudeModel {
                        aoa_rad: 0.,
                        bank_rad: 0.,
                        aoa_rate_rad_s: 0.,
                        bank_rate_rad_s: 0.,
                        requested_aoa_rad: 0.,
                        requested_bank_rad: 0.,
                        cmd_aoa_rad: 0.,
                        cmd_bank_rad: 0.,
                        pitch_wn: 0.,
                        pitch_zeta: 0.,
                        roll_wn: 0.,
                        roll_zeta: 0.,
                        max_pitch_rate_rad_s: 0.,
                        max_roll_rate_rad_s: 0.,
                        max_pitch_accel_rad_s2: 0.,
                        max_roll_accel_rad_s2: 0.,
                        pitch_full_authority_q_pa: 0.,
                        roll_full_authority_q_pa: 0.,
                        direct: false,
                        direct_body_coordinates: false,
                        body_rates_initialized: false,
                        body_pitch_rate_rad_s: 0.,
                        body_roll_rate_rad_s: 0.,
                        body_yaw_rate_rad_s: 0.,
                        input_pitch: 0.,
                        input_roll: 0.,
                        input_yaw: 0.,
                        sideslip_rad: 0.,
                        sideslip_rate_rad_s: 0.,
                        direct_min_q_pa: 0.,
                        direct_pitch_accel_per_kpa: 0.,
                        direct_pitch_accel_max: 0.,
                        direct_pitch_stiffness_per_kpa: 0.,
                        direct_trim_aoa_rad: 0.,
                        direct_pitch_damping_ratio: 0.,
                        direct_roll_accel_per_kpa: 0.,
                        direct_roll_accel_max: 0.,
                        direct_roll_damping_s_inv: 0.,
                        direct_yaw_accel_per_kpa: 0.,
                        direct_yaw_accel_max: 0.,
                        direct_yaw_stiffness_per_kpa: 0.,
                    },
                    mass_kg: 0.,
                    ut_s: 0.,
                    trim_aoa_ceiling_rad: 0.,
                },
                final_geometry: TaemGeometryState {
                    runway_along_m: 0.,
                    runway_cross_m: 0.,
                    altitude_above_runway_m: 0.,
                    ground_speed_mps: 0.,
                    airspeed_mps: 0.,
                    course_deg: 0.,
                    flight_path_angle_deg: 0.,
                    heading_error_deg: 0.,
                    curvature_right_per_m: 0.,
                },
                elapsed_s: 0.,
                minimum_speed_mps: 0.,
                maximum_dynamic_pressure_pa: 0.,
                maximum_load_g: 0.,
                maximum_cross_track_m: 0.,
                maximum_course_error_deg: 0.,
                final_route_course_error_deg: 0.,
                maximum_altitude_error_m: 0.,
                maximum_lateral_authority_shortfall_mps2: 0.,
                maximum_vertical_authority_shortfall_mps2: 0.,
                turn_burden_integral_s: 0.,
                maximum_turn_authority_fraction: 0.,
                maximum_commanded_aoa_deg: 0.,
                bank_target_reversals: 0,
                failure_cross_track_m: 0.,
                failure_course_error_deg: 0.,
                failure_required_lateral_accel_mps2: 0.,
                failure_available_lateral_accel_mps2: 0.,
                failure_required_vertical_lift_mps2: 0.,
                failure_delivered_vertical_lift_mps2: 0.,
                failure_target_altitude_m: 0.,
                failure_target_flight_path_angle_deg: 0.,
                failure_route_index: 0,
                energy_start_j_kg: 0.,
                energy_end_j_kg: 0.,
                drag_work_j_kg: 0.,
                energy_closure_residual_j_kg: 0.,
                final_route_index: 0,
                final_target_altitude_m: 0.,
                final_target_flight_path_angle_deg: 0.,
                reason: ::core::ptr::null::<::core::ffi::c_char>(),
            },
            reason: ::core::ptr::null::<::core::ffi::c_char>(),
        },
        solve_wall_s: 0.,
        scale_mach: 0.,
        lift_scale: 0.,
        drag_scale: 0.,
        diagnostic: [0; 768],
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Mm305PlanResult>() as size_t,
    );
    if model.is_null() || request.is_null() || !(*request).valid
        || !(*model).replay_validated
    {
        snprintf(
            &raw mut out.diagnostic as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
            b"MM305 plan request invalid\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return out;
    }
    let mut started: ::core::ffi::c_double = mm305_wall_seconds();
    out.request_ut = (*request).request_ut;
    out.initial_state = (*request).state;
    out.model_snapshot_id = (*request).model_snapshot_id;
    out.scale_mach = (*request).scale_mach;
    out.lift_scale = (*request).lift_scale;
    out.drag_scale = (*request).drag_scale;
    let mut scaled: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    let mut reciprocal: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    if scaled.is_null() || reciprocal.is_null() {
        free(scaled as *mut ::core::ffi::c_void);
        free(reciprocal as *mut ::core::ffi::c_void);
        snprintf(
            &raw mut out.diagnostic as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
            b"MM305 plan allocation failed\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return out;
    }
    mm305_prepare_planning_model(
        scaled,
        model,
        (*request).final_approach_distance_m,
        (*request).scale_mach,
        (*request).lift_scale,
        (*request).drag_scale,
        (*request).state.trim_aoa_ceiling_rad,
    );
    mm305_reciprocal_model(scaled, reciprocal);
    if (*request).seed_route_valid as ::core::ffi::c_int != 0
        && fabs(
            (*request).seed_route.alignment_along_m
                + (*scaled).guidance.final_approach_distance,
        ) <= 500.0f64
        && (!(*request).restrict_side
            || (*request).seed_route.side * (*request).side > 0.0f64)
    {
        let mut seed_model: *const TerminalModel = if (*request).seed_runway_end
            == 1 as ::core::ffi::c_int
        {
            reciprocal
        } else {
            scaled
        };
        let mut seed: TaemFixedHacCandidate = taem_fixed_hac_evaluate_route(
            seed_model,
            &raw const (*request).state,
            &raw const (*request).seed_route,
            (*request).seed_runway_end,
            0.5f64,
            420.0f64,
        );
        if mm305_candidate_delivery_ok(seed_model, request, &raw mut seed) {
            out.valid = true_0 != 0;
            out.found = true_0 != 0;
            out.candidate = seed;
            snprintf(
                &raw mut out.diagnostic as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
                b"MM305 provisional route promoted after native replay scale=%.3f/%.3f\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*request).lift_scale,
                (*request).drag_scale,
            );
            free(scaled as *mut ::core::ffi::c_void);
            free(reciprocal as *mut ::core::ffi::c_void);
            out.solve_wall_s = mm305_wall_seconds() - started;
            return out;
        }
        let mut seed_diag: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if !seed_diag.is_null()
            && strcmp(seed_diag, b"2\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int
        {
            fprintf(
                stderr,
                b"MM305 seed route rejected: side=%+.0f radius=%.0f sweep=%.1f length=%.0f reason=%s elapsed=%.1f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                seed.side,
                seed.route.hac.radius_m,
                seed.route.hac.arc_sweep_rad * RAD2DEG,
                seed.route.length_m,
                if !seed.reason.is_null() {
                    seed.reason
                } else {
                    b"unknown\0" as *const u8 as *const ::core::ffi::c_char
                },
                seed.replay.elapsed_s,
            );
        }
    }
    let radius_factors: [::core::ffi::c_double; 2] = [1.0f64, 1.5f64];
    let mut search: TaemFixedHacSearch = TaemFixedHacSearch {
        status: TAEM_PLAN_INFEASIBLE,
        selected_candidate: 0,
        candidate_count: 0,
        candidates: [TaemFixedHacCandidate {
            status: TAEM_PLAN_INFEASIBLE,
            side: 0.,
            runway_end: 0,
            route_built: false,
            final_tail_qualified: false,
            required_lateral_accel_mps2: 0.,
            available_lateral_accel_mps2: 0.,
            quality_score: 0.,
            route: TaemRoute {
                valid: false,
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
            },
            replay: TerminalSolverResult {
                status: TERMINAL_SOLVER_QUALIFIED,
                path_constraints_ok: false,
                final_interface_qualified: false,
                final_state: TerminalDynamicState {
                    position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                    velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                    attitude: AttitudeModel {
                        aoa_rad: 0.,
                        bank_rad: 0.,
                        aoa_rate_rad_s: 0.,
                        bank_rate_rad_s: 0.,
                        requested_aoa_rad: 0.,
                        requested_bank_rad: 0.,
                        cmd_aoa_rad: 0.,
                        cmd_bank_rad: 0.,
                        pitch_wn: 0.,
                        pitch_zeta: 0.,
                        roll_wn: 0.,
                        roll_zeta: 0.,
                        max_pitch_rate_rad_s: 0.,
                        max_roll_rate_rad_s: 0.,
                        max_pitch_accel_rad_s2: 0.,
                        max_roll_accel_rad_s2: 0.,
                        pitch_full_authority_q_pa: 0.,
                        roll_full_authority_q_pa: 0.,
                        direct: false,
                        direct_body_coordinates: false,
                        body_rates_initialized: false,
                        body_pitch_rate_rad_s: 0.,
                        body_roll_rate_rad_s: 0.,
                        body_yaw_rate_rad_s: 0.,
                        input_pitch: 0.,
                        input_roll: 0.,
                        input_yaw: 0.,
                        sideslip_rad: 0.,
                        sideslip_rate_rad_s: 0.,
                        direct_min_q_pa: 0.,
                        direct_pitch_accel_per_kpa: 0.,
                        direct_pitch_accel_max: 0.,
                        direct_pitch_stiffness_per_kpa: 0.,
                        direct_trim_aoa_rad: 0.,
                        direct_pitch_damping_ratio: 0.,
                        direct_roll_accel_per_kpa: 0.,
                        direct_roll_accel_max: 0.,
                        direct_roll_damping_s_inv: 0.,
                        direct_yaw_accel_per_kpa: 0.,
                        direct_yaw_accel_max: 0.,
                        direct_yaw_stiffness_per_kpa: 0.,
                    },
                    mass_kg: 0.,
                    ut_s: 0.,
                    trim_aoa_ceiling_rad: 0.,
                },
                final_geometry: TaemGeometryState {
                    runway_along_m: 0.,
                    runway_cross_m: 0.,
                    altitude_above_runway_m: 0.,
                    ground_speed_mps: 0.,
                    airspeed_mps: 0.,
                    course_deg: 0.,
                    flight_path_angle_deg: 0.,
                    heading_error_deg: 0.,
                    curvature_right_per_m: 0.,
                },
                elapsed_s: 0.,
                minimum_speed_mps: 0.,
                maximum_dynamic_pressure_pa: 0.,
                maximum_load_g: 0.,
                maximum_cross_track_m: 0.,
                maximum_course_error_deg: 0.,
                final_route_course_error_deg: 0.,
                maximum_altitude_error_m: 0.,
                maximum_lateral_authority_shortfall_mps2: 0.,
                maximum_vertical_authority_shortfall_mps2: 0.,
                turn_burden_integral_s: 0.,
                maximum_turn_authority_fraction: 0.,
                maximum_commanded_aoa_deg: 0.,
                bank_target_reversals: 0,
                failure_cross_track_m: 0.,
                failure_course_error_deg: 0.,
                failure_required_lateral_accel_mps2: 0.,
                failure_available_lateral_accel_mps2: 0.,
                failure_required_vertical_lift_mps2: 0.,
                failure_delivered_vertical_lift_mps2: 0.,
                failure_target_altitude_m: 0.,
                failure_target_flight_path_angle_deg: 0.,
                failure_route_index: 0,
                energy_start_j_kg: 0.,
                energy_end_j_kg: 0.,
                drag_work_j_kg: 0.,
                energy_closure_residual_j_kg: 0.,
                final_route_index: 0,
                final_target_altitude_m: 0.,
                final_target_flight_path_angle_deg: 0.,
                reason: ::core::ptr::null::<::core::ffi::c_char>(),
            },
            reason: ::core::ptr::null::<::core::ffi::c_char>(),
        }; 4],
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    memset(
        &raw mut search as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemFixedHacSearch>() as size_t,
    );
    let mut best_candidate: TaemFixedHacCandidate = TaemFixedHacCandidate {
        status: TAEM_PLAN_INFEASIBLE,
        side: 0.,
        runway_end: 0,
        route_built: false,
        final_tail_qualified: false,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        quality_score: 0.,
        route: TaemRoute {
            valid: false,
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
        },
        replay: TerminalSolverResult {
            status: TERMINAL_SOLVER_QUALIFIED,
            path_constraints_ok: false,
            final_interface_qualified: false,
            final_state: TerminalDynamicState {
                position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                attitude: AttitudeModel {
                    aoa_rad: 0.,
                    bank_rad: 0.,
                    aoa_rate_rad_s: 0.,
                    bank_rate_rad_s: 0.,
                    requested_aoa_rad: 0.,
                    requested_bank_rad: 0.,
                    cmd_aoa_rad: 0.,
                    cmd_bank_rad: 0.,
                    pitch_wn: 0.,
                    pitch_zeta: 0.,
                    roll_wn: 0.,
                    roll_zeta: 0.,
                    max_pitch_rate_rad_s: 0.,
                    max_roll_rate_rad_s: 0.,
                    max_pitch_accel_rad_s2: 0.,
                    max_roll_accel_rad_s2: 0.,
                    pitch_full_authority_q_pa: 0.,
                    roll_full_authority_q_pa: 0.,
                    direct: false,
                    direct_body_coordinates: false,
                    body_rates_initialized: false,
                    body_pitch_rate_rad_s: 0.,
                    body_roll_rate_rad_s: 0.,
                    body_yaw_rate_rad_s: 0.,
                    input_pitch: 0.,
                    input_roll: 0.,
                    input_yaw: 0.,
                    sideslip_rad: 0.,
                    sideslip_rate_rad_s: 0.,
                    direct_min_q_pa: 0.,
                    direct_pitch_accel_per_kpa: 0.,
                    direct_pitch_accel_max: 0.,
                    direct_pitch_stiffness_per_kpa: 0.,
                    direct_trim_aoa_rad: 0.,
                    direct_pitch_damping_ratio: 0.,
                    direct_roll_accel_per_kpa: 0.,
                    direct_roll_accel_max: 0.,
                    direct_roll_damping_s_inv: 0.,
                    direct_yaw_accel_per_kpa: 0.,
                    direct_yaw_accel_max: 0.,
                    direct_yaw_stiffness_per_kpa: 0.,
                },
                mass_kg: 0.,
                ut_s: 0.,
                trim_aoa_ceiling_rad: 0.,
            },
            final_geometry: TaemGeometryState {
                runway_along_m: 0.,
                runway_cross_m: 0.,
                altitude_above_runway_m: 0.,
                ground_speed_mps: 0.,
                airspeed_mps: 0.,
                course_deg: 0.,
                flight_path_angle_deg: 0.,
                heading_error_deg: 0.,
                curvature_right_per_m: 0.,
            },
            elapsed_s: 0.,
            minimum_speed_mps: 0.,
            maximum_dynamic_pressure_pa: 0.,
            maximum_load_g: 0.,
            maximum_cross_track_m: 0.,
            maximum_course_error_deg: 0.,
            final_route_course_error_deg: 0.,
            maximum_altitude_error_m: 0.,
            maximum_lateral_authority_shortfall_mps2: 0.,
            maximum_vertical_authority_shortfall_mps2: 0.,
            turn_burden_integral_s: 0.,
            maximum_turn_authority_fraction: 0.,
            maximum_commanded_aoa_deg: 0.,
            bank_target_reversals: 0,
            failure_cross_track_m: 0.,
            failure_course_error_deg: 0.,
            failure_required_lateral_accel_mps2: 0.,
            failure_available_lateral_accel_mps2: 0.,
            failure_required_vertical_lift_mps2: 0.,
            failure_delivered_vertical_lift_mps2: 0.,
            failure_target_altitude_m: 0.,
            failure_target_flight_path_angle_deg: 0.,
            failure_route_index: 0,
            energy_start_j_kg: 0.,
            energy_end_j_kg: 0.,
            drag_work_j_kg: 0.,
            energy_closure_residual_j_kg: 0.,
            final_route_index: 0,
            final_target_altitude_m: 0.,
            final_target_flight_path_angle_deg: 0.,
            reason: ::core::ptr::null::<::core::ffi::c_char>(),
        },
        reason: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut have_best: bool = false_0 != 0;
    let mut reasons: [::core::ffi::c_char; 768] = ::core::mem::transmute::<
        [u8; 768],
        [::core::ffi::c_char; 768],
    >(
        *b"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
    );
    let mut reasons_used: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut end_models: [*const TerminalModel; 2] = [
        (if (*request).search_both_ends as ::core::ffi::c_int != 0
            || (*request).upstream_end == 0 as ::core::ffi::c_int
        {
            scaled
        } else {
            reciprocal
        }) as *const TerminalModel,
        reciprocal as *const TerminalModel,
    ];
    let mut end_ids: [::core::ffi::c_int; 2] = [
        if (*request).search_both_ends as ::core::ffi::c_int != 0 {
            0 as ::core::ffi::c_int
        } else {
            (*request).upstream_end
        },
        1 as ::core::ffi::c_int,
    ];
    let mut end_count: ::core::ffi::c_int = if (*request).search_both_ends
        as ::core::ffi::c_int != 0
    {
        2 as ::core::ffi::c_int
    } else {
        1 as ::core::ffi::c_int
    };
    let mut end_pass: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while end_pass < end_count && !have_best {
        let mut end_model: *const TerminalModel = end_models[end_pass as usize];
        let mut runway_end: ::core::ffi::c_int = end_ids[end_pass as usize];
        let mut r: size_t = 0 as size_t;
        while r
            < (::core::mem::size_of::<[::core::ffi::c_double; 2]>() as usize)
                .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
        {
            let mut radius: ::core::ffi::c_double = (*request).hac_radius_m
                * radius_factors[r as usize];
            if radius >= 3000.0f64 {
                search = taem_fixed_hac_search(
                    end_model,
                    &raw const (*request).state,
                    radius,
                    200.0f64,
                    0.5f64,
                    420.0f64,
                );
                let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while i < search.candidate_count {
                    search.candidates[i as usize].runway_end = runway_end;
                    i += 1;
                }
                let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while i_0 < search.candidate_count
                    && reasons_used >= 0 as ::core::ffi::c_int
                    && (reasons_used as size_t)
                        < ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as usize
                {
                    let mut c: *const TaemFixedHacCandidate = (&raw mut search.candidates
                        as *mut TaemFixedHacCandidate)
                        .offset(i_0 as isize) as *mut TaemFixedHacCandidate;
                    reasons_used
                        += snprintf(
                            (&raw mut reasons as *mut ::core::ffi::c_char)
                                .offset(reasons_used as isize),
                            (::core::mem::size_of::<[::core::ffi::c_char; 768]>()
                                as size_t)
                                .wrapping_sub(reasons_used as size_t),
                            b" [%s %c r%.0f] %s t%.1f;\0" as *const u8
                                as *const ::core::ffi::c_char,
                            if runway_end == 1 as ::core::ffi::c_int {
                                b"RECIP\0" as *const u8 as *const ::core::ffi::c_char
                            } else {
                                b"CONF\0" as *const u8 as *const ::core::ffi::c_char
                            },
                            if (*c).side > 0.0f64 { 'R' as i32 } else { 'L' as i32 },
                            radius,
                            if !(*c).reason.is_null() {
                                (*c).reason
                            } else {
                                b"unknown\0" as *const u8 as *const ::core::ffi::c_char
                            },
                            (*c).replay.elapsed_s,
                        );
                    i_0 += 1;
                }
                let mut plan_diag: *const ::core::ffi::c_char = getenv(
                    b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
                if !plan_diag.is_null()
                    && strcmp(
                        plan_diag,
                        b"2\0" as *const u8 as *const ::core::ffi::c_char,
                    ) == 0 as ::core::ffi::c_int
                {
                    fprintf(
                        stderr,
                        b"MM305 search pass: end=%s radius=%.0f selected=%d candidates=%d\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        if runway_end == 1 as ::core::ffi::c_int {
                            b"RECIP\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"CONF\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        radius,
                        search.selected_candidate,
                        search.candidate_count,
                    );
                    let mut di: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                    while di < search.candidate_count {
                        let mut dc: *const TaemFixedHacCandidate = (&raw mut search
                            .candidates as *mut TaemFixedHacCandidate)
                            .offset(di as isize) as *mut TaemFixedHacCandidate;
                        fprintf(
                            stderr,
                            b" [%c status=%d len=%.0f %s]\0" as *const u8
                                as *const ::core::ffi::c_char,
                            if (*dc).side > 0.0f64 { 'R' as i32 } else { 'L' as i32 },
                            (*dc).status as ::core::ffi::c_int,
                            if (*dc).route_built as ::core::ffi::c_int != 0 {
                                (*dc).route.length_m
                            } else {
                                ::core::f32::NAN as ::core::ffi::c_double
                            },
                            if !(*dc).reason.is_null() {
                                (*dc).reason
                            } else {
                                b"unknown\0" as *const u8 as *const ::core::ffi::c_char
                            },
                        );
                        di += 1;
                    }
                    fprintf(stderr, b"\n\0" as *const u8 as *const ::core::ffi::c_char);
                }
                let mut i_1: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while i_1 < search.candidate_count {
                    let mut c_0: *mut TaemFixedHacCandidate = (&raw mut search.candidates
                        as *mut TaemFixedHacCandidate)
                        .offset(i_1 as isize) as *mut TaemFixedHacCandidate;
                    if mm305_candidate_delivery_ok(end_model, request, c_0) {
                        if !((*request).restrict_side as ::core::ffi::c_int != 0
                            && (*c_0).side * (*request).side <= 0.0f64)
                        {
                            if !have_best
                                || taem_fixed_hac_candidate_preferred(
                                    end_model,
                                    c_0,
                                    &raw mut best_candidate,
                                ) as ::core::ffi::c_int != 0
                            {
                                best_candidate = *c_0;
                                have_best = true_0 != 0;
                            }
                        }
                    }
                    i_1 += 1;
                }
            }
            r = r.wrapping_add(1);
        }
        end_pass += 1;
    }
    out.valid = true_0 != 0;
    if have_best {
        out.found = true_0 != 0;
        out.candidate = best_candidate;
    }
    snprintf(
        &raw mut out.diagnostic as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
        b"MM305 route %s scale=%.3f/%.3f:%s\0" as *const u8
            as *const ::core::ffi::c_char,
        if out.found as ::core::ffi::c_int != 0 {
            b"found\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"rejected\0" as *const u8 as *const ::core::ffi::c_char
        },
        (*request).lift_scale,
        (*request).drag_scale,
        &raw mut reasons as *mut ::core::ffi::c_char,
    );
    free(scaled as *mut ::core::ffi::c_void);
    free(reciprocal as *mut ::core::ffi::c_void);
    out.solve_wall_s = mm305_wall_seconds() - started;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn mm305_revalidate_plan(
    mut model: *const TerminalModel,
    mut request: *const Mm305PlanRequest,
    mut previous: *const Mm305PlanResult,
) -> Mm305PlanResult {
    let mut out: Mm305PlanResult = Mm305PlanResult {
        valid: 0 as ::core::ffi::c_int != 0,
        found: false,
        request_ut: 0.,
        initial_state: TerminalDynamicState {
            position_i_m: Vec3 { x: 0., y: 0., z: 0. },
            velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
            attitude: AttitudeModel {
                aoa_rad: 0.,
                bank_rad: 0.,
                aoa_rate_rad_s: 0.,
                bank_rate_rad_s: 0.,
                requested_aoa_rad: 0.,
                requested_bank_rad: 0.,
                cmd_aoa_rad: 0.,
                cmd_bank_rad: 0.,
                pitch_wn: 0.,
                pitch_zeta: 0.,
                roll_wn: 0.,
                roll_zeta: 0.,
                max_pitch_rate_rad_s: 0.,
                max_roll_rate_rad_s: 0.,
                max_pitch_accel_rad_s2: 0.,
                max_roll_accel_rad_s2: 0.,
                pitch_full_authority_q_pa: 0.,
                roll_full_authority_q_pa: 0.,
                direct: false,
                direct_body_coordinates: false,
                body_rates_initialized: false,
                body_pitch_rate_rad_s: 0.,
                body_roll_rate_rad_s: 0.,
                body_yaw_rate_rad_s: 0.,
                input_pitch: 0.,
                input_roll: 0.,
                input_yaw: 0.,
                sideslip_rad: 0.,
                sideslip_rate_rad_s: 0.,
                direct_min_q_pa: 0.,
                direct_pitch_accel_per_kpa: 0.,
                direct_pitch_accel_max: 0.,
                direct_pitch_stiffness_per_kpa: 0.,
                direct_trim_aoa_rad: 0.,
                direct_pitch_damping_ratio: 0.,
                direct_roll_accel_per_kpa: 0.,
                direct_roll_accel_max: 0.,
                direct_roll_damping_s_inv: 0.,
                direct_yaw_accel_per_kpa: 0.,
                direct_yaw_accel_max: 0.,
                direct_yaw_stiffness_per_kpa: 0.,
            },
            mass_kg: 0.,
            ut_s: 0.,
            trim_aoa_ceiling_rad: 0.,
        },
        model_snapshot_id: 0,
        candidate: TaemFixedHacCandidate {
            status: TAEM_PLAN_INFEASIBLE,
            side: 0.,
            runway_end: 0,
            route_built: false,
            final_tail_qualified: false,
            required_lateral_accel_mps2: 0.,
            available_lateral_accel_mps2: 0.,
            quality_score: 0.,
            route: TaemRoute {
                valid: false,
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
            },
            replay: TerminalSolverResult {
                status: TERMINAL_SOLVER_QUALIFIED,
                path_constraints_ok: false,
                final_interface_qualified: false,
                final_state: TerminalDynamicState {
                    position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                    velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                    attitude: AttitudeModel {
                        aoa_rad: 0.,
                        bank_rad: 0.,
                        aoa_rate_rad_s: 0.,
                        bank_rate_rad_s: 0.,
                        requested_aoa_rad: 0.,
                        requested_bank_rad: 0.,
                        cmd_aoa_rad: 0.,
                        cmd_bank_rad: 0.,
                        pitch_wn: 0.,
                        pitch_zeta: 0.,
                        roll_wn: 0.,
                        roll_zeta: 0.,
                        max_pitch_rate_rad_s: 0.,
                        max_roll_rate_rad_s: 0.,
                        max_pitch_accel_rad_s2: 0.,
                        max_roll_accel_rad_s2: 0.,
                        pitch_full_authority_q_pa: 0.,
                        roll_full_authority_q_pa: 0.,
                        direct: false,
                        direct_body_coordinates: false,
                        body_rates_initialized: false,
                        body_pitch_rate_rad_s: 0.,
                        body_roll_rate_rad_s: 0.,
                        body_yaw_rate_rad_s: 0.,
                        input_pitch: 0.,
                        input_roll: 0.,
                        input_yaw: 0.,
                        sideslip_rad: 0.,
                        sideslip_rate_rad_s: 0.,
                        direct_min_q_pa: 0.,
                        direct_pitch_accel_per_kpa: 0.,
                        direct_pitch_accel_max: 0.,
                        direct_pitch_stiffness_per_kpa: 0.,
                        direct_trim_aoa_rad: 0.,
                        direct_pitch_damping_ratio: 0.,
                        direct_roll_accel_per_kpa: 0.,
                        direct_roll_accel_max: 0.,
                        direct_roll_damping_s_inv: 0.,
                        direct_yaw_accel_per_kpa: 0.,
                        direct_yaw_accel_max: 0.,
                        direct_yaw_stiffness_per_kpa: 0.,
                    },
                    mass_kg: 0.,
                    ut_s: 0.,
                    trim_aoa_ceiling_rad: 0.,
                },
                final_geometry: TaemGeometryState {
                    runway_along_m: 0.,
                    runway_cross_m: 0.,
                    altitude_above_runway_m: 0.,
                    ground_speed_mps: 0.,
                    airspeed_mps: 0.,
                    course_deg: 0.,
                    flight_path_angle_deg: 0.,
                    heading_error_deg: 0.,
                    curvature_right_per_m: 0.,
                },
                elapsed_s: 0.,
                minimum_speed_mps: 0.,
                maximum_dynamic_pressure_pa: 0.,
                maximum_load_g: 0.,
                maximum_cross_track_m: 0.,
                maximum_course_error_deg: 0.,
                final_route_course_error_deg: 0.,
                maximum_altitude_error_m: 0.,
                maximum_lateral_authority_shortfall_mps2: 0.,
                maximum_vertical_authority_shortfall_mps2: 0.,
                turn_burden_integral_s: 0.,
                maximum_turn_authority_fraction: 0.,
                maximum_commanded_aoa_deg: 0.,
                bank_target_reversals: 0,
                failure_cross_track_m: 0.,
                failure_course_error_deg: 0.,
                failure_required_lateral_accel_mps2: 0.,
                failure_available_lateral_accel_mps2: 0.,
                failure_required_vertical_lift_mps2: 0.,
                failure_delivered_vertical_lift_mps2: 0.,
                failure_target_altitude_m: 0.,
                failure_target_flight_path_angle_deg: 0.,
                failure_route_index: 0,
                energy_start_j_kg: 0.,
                energy_end_j_kg: 0.,
                drag_work_j_kg: 0.,
                energy_closure_residual_j_kg: 0.,
                final_route_index: 0,
                final_target_altitude_m: 0.,
                final_target_flight_path_angle_deg: 0.,
                reason: ::core::ptr::null::<::core::ffi::c_char>(),
            },
            reason: ::core::ptr::null::<::core::ffi::c_char>(),
        },
        solve_wall_s: 0.,
        scale_mach: 0.,
        lift_scale: 0.,
        drag_scale: 0.,
        diagnostic: [0; 768],
    };
    if model.is_null() || request.is_null() || !(*request).valid || previous.is_null()
        || !(*previous).found
        || (*request).model_snapshot_id != (*previous).model_snapshot_id
    {
        return out;
    }
    let mut started: ::core::ffi::c_double = mm305_wall_seconds();
    let mut scaled: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    let mut reciprocal: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    if scaled.is_null() || reciprocal.is_null() {
        free(scaled as *mut ::core::ffi::c_void);
        free(reciprocal as *mut ::core::ffi::c_void);
        return out;
    }
    mm305_prepare_planning_model(
        scaled,
        model,
        (*request).final_approach_distance_m,
        (*request).scale_mach,
        (*request).lift_scale,
        (*request).drag_scale,
        (*request).state.trim_aoa_ceiling_rad,
    );
    mm305_reciprocal_model(scaled, reciprocal);
    let mut end: *const TerminalModel = if (*previous).candidate.runway_end
        == 1 as ::core::ffi::c_int
    {
        reciprocal
    } else {
        scaled
    };
    out.valid = true_0 != 0;
    out.request_ut = (*request).request_ut;
    out.initial_state = (*request).state;
    out.model_snapshot_id = (*request).model_snapshot_id;
    out.scale_mach = (*request).scale_mach;
    out.lift_scale = (*request).lift_scale;
    out.drag_scale = (*request).drag_scale;
    out.candidate = taem_fixed_hac_evaluate_route(
        end,
        &raw const (*request).state,
        &raw const (*previous).candidate.route,
        (*previous).candidate.runway_end,
        0.5f64,
        420.0f64,
    );
    out.found = mm305_candidate_delivery_ok(end, request, &raw mut out.candidate);
    free(scaled as *mut ::core::ffi::c_void);
    free(reciprocal as *mut ::core::ffi::c_void);
    out.solve_wall_s = mm305_wall_seconds() - started;
    snprintf(
        &raw mut out.diagnostic as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
        b"MM305 current-state route revalidation %s: %s\0" as *const u8
            as *const ::core::ffi::c_char,
        if out.found as ::core::ffi::c_int != 0 {
            b"passed\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"rejected\0" as *const u8 as *const ::core::ffi::c_char
        },
        if !out.candidate.reason.is_null() {
            out.candidate.reason
        } else {
            b"invalid\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    return out;
}
unsafe extern "C" fn mm305_request_from_state(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut cfg: *const LandingConfiguration,
    mut model: *const TerminalModel,
) -> Mm305PlanRequest {
    let mut r: Mm305PlanRequest = Mm305PlanRequest {
        valid: false,
        request_ut: 0.,
        model_snapshot_id: 0,
        state: TerminalDynamicState {
            position_i_m: Vec3 { x: 0., y: 0., z: 0. },
            velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
            attitude: AttitudeModel {
                aoa_rad: 0.,
                bank_rad: 0.,
                aoa_rate_rad_s: 0.,
                bank_rate_rad_s: 0.,
                requested_aoa_rad: 0.,
                requested_bank_rad: 0.,
                cmd_aoa_rad: 0.,
                cmd_bank_rad: 0.,
                pitch_wn: 0.,
                pitch_zeta: 0.,
                roll_wn: 0.,
                roll_zeta: 0.,
                max_pitch_rate_rad_s: 0.,
                max_roll_rate_rad_s: 0.,
                max_pitch_accel_rad_s2: 0.,
                max_roll_accel_rad_s2: 0.,
                pitch_full_authority_q_pa: 0.,
                roll_full_authority_q_pa: 0.,
                direct: false,
                direct_body_coordinates: false,
                body_rates_initialized: false,
                body_pitch_rate_rad_s: 0.,
                body_roll_rate_rad_s: 0.,
                body_yaw_rate_rad_s: 0.,
                input_pitch: 0.,
                input_roll: 0.,
                input_yaw: 0.,
                sideslip_rad: 0.,
                sideslip_rate_rad_s: 0.,
                direct_min_q_pa: 0.,
                direct_pitch_accel_per_kpa: 0.,
                direct_pitch_accel_max: 0.,
                direct_pitch_stiffness_per_kpa: 0.,
                direct_trim_aoa_rad: 0.,
                direct_pitch_damping_ratio: 0.,
                direct_roll_accel_per_kpa: 0.,
                direct_roll_accel_max: 0.,
                direct_roll_damping_s_inv: 0.,
                direct_yaw_accel_per_kpa: 0.,
                direct_yaw_accel_max: 0.,
                direct_yaw_stiffness_per_kpa: 0.,
            },
            mass_kg: 0.,
            ut_s: 0.,
            trim_aoa_ceiling_rad: 0.,
        },
        final_guidance: GuidanceMachine {
            phase: PHASE_IDLE,
            automation_engaged: false,
            paused: false,
            aborted: false,
            entry_terminal_model: ::core::ptr::null::<TerminalModel>(),
            diagnostic_shadow: false,
            diagnostic_stop_at_taem: false,
            diagnostic_entry_previous_range_m: 0.,
            diagnostic_entry_milestone_mask: 0,
            diagnostic_entry_has_previous_range: false,
            s_turn_sign: 0.,
            s_turn_leg_started_ut: 0.,
            has_s_turn_leg_started: false,
            s_turn_reversal_requested_ut: 0.,
            has_s_turn_reversal_requested: false,
            entry_exec: EntryExecutive {
                initialized: false,
                entry_complete: false,
                phase: 0 as EntryPhase,
                last_transition_reason: ENTRY_TRANSITION_NONE,
                last_transition_from: 0 as EntryPhase,
                last_transition_to: 0 as EntryPhase,
                last_transition_ut: 0.,
                last_transition_velocity: 0.,
                last_transition_completed_entry: false,
                transition_count: 0,
            },
            entry_lateral: EntryLateralState {
                initialized: false,
                bank_sign: 0.,
                leg_captured: false,
                leg_captured_ut: 0.,
                reversal_armed: false,
                last_reversal_ut: 0.,
                command_bank_deg: 0.,
                command_bank_rate_deg_s: 0.,
                has_previous_sideslip: false,
                previous_sideslip_deg: 0.,
            },
            entry_energy: EntryEnergyState {
                initialized: false,
                integral_m_s: 0.,
                last_ut: 0.,
            },
            taem_exec: TaemExecutive {
                initialized: false,
                ownership_latched: false,
                taem_complete: false,
                phase: TAEM_PHASE_PATH_ACQUISITION,
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
            entry_alpha_has_target: false,
            entry_alpha_has_modulation: false,
            entry_drag_ratio_valid: false,
            entry_alpha_target: 0.,
            entry_alpha_modulation: 0.,
            entry_alpha_drag_boost: 0.,
            entry_drag_velocity_ratio: 0.,
            entry_alpha_deficit_filter: 0.,
            entry_alpha_ceiling_filter: 0.,
            entry_alpha_filter_valid: false,
            entry_bank_authority_acquired: false,
            entry_supervision_valid: false,
            entry_supervision_boundary_missed: false,
            entry_supervision_mode: ENTRY_SUPERVISION_PASS_THROUGH,
            entry_policy_valid: false,
            entry_policy_bank_deg: 0.,
            entry_policy_aoa_deg: 0.,
            entry_policy_ut: 0.,
            entry_energy_short_latched: false,
            entry_handoff_target_range: 0.,
            entry_hac_energy_excess: 0.,
            entry_hac_energy_ut: 0.,
            taem_runway_end_scored: false,
            taem_runway_end_score_ut: 0.,
            entry_supervision_ut: 0.,
            entry_planning_deferred: false,
            entry_planning_needed: false,
            entry_lateral_infeasible: false,
            entry_committed_infeasible: false,
            entry_s_turn_plan: EntryControlPlan {
                valid: false,
                terminal_ready: false,
                plan_id: 0,
                plan_version: 0,
                parent_plan_id: 0,
                parent_plan_version: 0,
                planned_ut: 0.,
                target_bank: 0.,
                target_aoa: 0.,
                target_heading: 0.,
                bank_cap: 0.,
                target_turn_radius: 0.,
                segment_duration: 0.,
                cost: 0.,
                taem_range_error: 0.,
                taem_speed: 0.,
                taem_energy_error: 0.,
                closest_distance: 0.,
                has_planned_reversal: false,
                planned_reversal_is_final: false,
                final_heading_lock: false,
                planned_reversal_ut: 0.,
                planned_reversal_range: 0.,
                planned_reversal_sign: 0.,
                predicted_reversals: 0,
            },
            entry_predictor_models_valid: false,
            entry_predictor_envelope: AerodynamicEnvelope {
                regimes: [AerodynamicModel {
                    lift_to_drag: 0.,
                    ballistic_coefficient: 0.,
                    confidence: 0.,
                }; 4],
            },
            entry_predictor_calibration: TrajectoryCalibrationModel {
                physics: ::core::ptr::null::<VesselPhysicsModel>(),
                density_scale: 0.,
                drag_scale: 0.,
                lift_scale: 0.,
                bank_effectiveness: 0.,
                speed_of_sound: 0.,
                speed_of_sound_scale: 0.,
                stress_drag_scale: 0.,
                stress_lift_scale: 0.,
                forecast_drag_scale: 0.,
                forecast_lift_scale: 0.,
                forecast_exact_policy_bank: false,
                altitude_residual: 0.,
                speed_residual: 0.,
                range_residual: 0.,
                confidence: 0.,
                accepted_samples: 0,
            },
            mm305_route: TaemRoute {
                valid: false,
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
            },
            mm305_route_cursor: 0,
            mm305_route_committed: false,
            mm305_hac_exit_reached: false,
            mm305_direct_approach_degraded: false,
            mm305_best_effort_route_active: false,
            mm305_runway_alignment_active: false,
            mm305_model_snapshot_id: 0,
            mm305_acquisition_route: TaemRoute {
                valid: false,
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
            },
            mm305_acquisition_route_cursor: 0,
            mm305_acquisition_route_valid: false,
            mm305_acquisition_route_ut: 0.,
            mm305_acquisition_sweep_rad: 0.,
            mm305_async_planning: false,
            mm305_planning_needed: false,
            mm305_plan_request_ut: 0.,
            mm305_last_plan_attempt_ut: 0.,
            mm305_last_route_ut: 0.,
            mm305_plan_failures: 0,
            mm305_replans: 0,
            mm305_lift_scale: 0.,
            mm305_drag_scale: 0.,
            mm305_command_bank_rad: 0.,
            mm305_command_aoa_rad: 0.,
            mm305_command_ut: 0.,
            mm305_command_valid: false,
            terminal_landing_warning_latched: false,
            mm305_trim: AdaptiveTrim {
                weight: 0.,
                x: 0.,
                y: 0.,
                xx: 0.,
                xy: 0.,
                yy: 0.,
                lo: 0.,
                hi: 0.,
                last_ut: 0.,
                mach: 0.,
                pressure: 0.,
                clock_valid: false,
                regime_valid: false,
            },
            mm305_admission_needed: false,
            mm305_admission_request_ut: 0.,
            mm305_admission_last_attempt_ut: 0.,
            mm305_admission_valid: false,
            mm305_admission_found: false,
            mm305_admission_result_ut: 0.,
            mm305_admission_snapshot_id: 0,
            mm305_admission_route: TaemRoute {
                valid: false,
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
            },
            mm305_admission_runway_end: 0,
            mm305_admission_side: 0.,
            mm305_admission_solve_wall_s: 0.,
            mm305_admission_lift_scale: 0.,
            mm305_admission_drag_scale: 0.,
            mm305_admission_scale_mach: 0.,
            mm305_admission_rejections: 0,
            taem_interface_target: TaemInterfaceTarget {
                valid: false,
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
            },
            taem_interface_captured: false,
            taem_safety_handoff: false,
            taem_interface_diagnostic_ut: 0.,
            terminal_path_kind: TERMINAL_PATH_NONE,
            terminal_path_committed: false,
            runway_end_preview_valid: false,
            runway_end_committed: false,
            runway_end_index: 0,
            runway_end_primary_path: 0.,
            runway_end_reciprocal_path: 0.,
            terminal_final_handoff_latched: false,
            terminal_final_handoff_distance: 0.,
            terminal_final_handoff_slope_deg: 0.,
            terminal_final_handoff_speed_mps: 0.,
            hac_side: 0.,
            hac_side_selected: false,
            delivered_delta_v: 0.,
            burn_command_started_ut: 0.,
            burn_active_elapsed: 0.,
            burn_progress_watch_ut: 0.,
            burn_progress_watch_delta_v: 0.,
            burn_thrust_loss_elapsed: 0.,
            has_burn_command_started: false,
            has_burn_progress_watch: false,
            deorbit_burn_completed: false,
            atmospheric_interface_crossed: false,
            has_previous_ut: false,
            previous_ut: 0.,
            final_approach_captured: false,
            airbrakes_deployed: false,
            terminal_final_low_energy: false,
            hac_progress_valid: false,
            hac_captured: false,
            hac_completed: false,
            terminal_region_entered: false,
            terminal_glide_mode: false,
            terminal_final_test_mode: false,
            terminal_rehearsal_mode: false,
            terminal_test_capture_active: false,
            hac_capture_lost_duration: 0.,
            terminal_energy_mismatch_duration: 0.,
            terminal_reentry_after_ut: 0.,
            hac_remaining: 0.,
            hac_radius: 0.,
            minimum_turn_radius: 0.,
            terminal_test_glide_slope: 0.,
            terminal_test_final_approach_distance: 0.,
            final_invalid_duration: 0.,
            terminal_test_revolution_remaining: 0.,
            terminal_test_preflare_altitude: 0.,
            terminal_test_preflare_target_speed: 0.,
            terminal_test_preflare_min_speed: 0.,
            terminal_vertical_stage: TERMINAL_TRAJECTORY_CAPTURE,
            terminal_vertical_stage_valid: false,
            gear_command_latched: false,
            ground_contact_latched: false,
            flare_sink_captured: false,
            terminal_response_sample_valid: false,
            terminal_preflare_plan_valid: false,
            terminal_energy_sample_valid: false,
            terminal_stage_started_ut: 0.,
            terminal_stage_good_duration: 0.,
            ground_contact_duration: 0.,
            terminal_previous_aoa: 0.,
            terminal_previous_vertical_speed: 0.,
            terminal_response_sample_ut: 0.,
            terminal_previous_specific_energy: 0.,
            terminal_previous_speed: 0.,
            terminal_energy_sample_ut: 0.,
            terminal_energy_loss_accel_ema: 0.,
            terminal_speed_loss_accel_ema: 0.,
            terminal_lift_area_ema: 0.,
            terminal_drag_area_ema: 0.,
            terminal_aero_sample_ut: 0.,
            terminal_aero_sample_valid: false,
            terminal_predicted_touchdown_speed: 0.,
            terminal_speedbrake_cda: 0.,
            terminal_speedbrake_fraction: 0.,
            terminal_split_fin_cda: 0.,
            terminal_vs_prev: 0.,
            terminal_vs_prev_ut: 0.,
            terminal_vs_rate_ema: 0.,
            terminal_lift_q: [0.; 21],
            terminal_drag_q: [0.; 21],
            terminal_aero_seen: [0; 21],
            terminal_aero_mach: [0.; 21],
            terminal_aoa_cmd_last: 0.,
            terminal_aoa_cmd_ut: 0.,
            taem_speedbrake_integral: 0.,
            taem_speedbrake_ut: 0.,
            taem_speedbrake_fraction: 0.,
            terminal_pull_latch: false,
            terminal_arc_started: false,
            terminal_aim_valid: false,
            terminal_aim_along: 0.,
            terminal_aim_shift: 0.,
            terminal_predicted_touchdown_along: 0.,
            terminal_aim_ut: 0.,
            terminal_lift_ratio: 0.,
            terminal_positive_aoa_rate_ema: 0.,
            terminal_sink_accel_ema: 0.,
            terminal_pitch_response_delay_ema: 0.,
            preflare_trigger_altitude: 0.,
            preflare_target_aoa: 0.,
            preflare_target_sink: 0.,
            preflare_minimum_speed: 0.,
            preflare_reference_speed: 0.,
            preflare_predicted_height_loss: 0.,
            preflare_predicted_kinetic_margin: 0.,
            preflare_effective_accel: 0.,
            preflare_response_time: 0.,
            preflare_plan_source_ut: 0.,
            attitude_recovery: false,
            control_bad_duration: 0.,
            control_good_duration: 0.,
            recovery_duration: 0.,
            recovery_heading: 0.,
            recovery_aoa: 0.,
            recovery_pitch_trim: 0.,
            previous_relative_roll_rate: 0.,
            roll_oscillation_score: 0.,
            roll_rate_excess_duration: 0.,
            has_previous_relative_roll_rate: false,
            previous_sideslip: 0.,
            entry_coordination_beta_rate: 0.,
            has_previous_sideslip: false,
            entry_predictive_score: 0.,
            entry_reference_speed: 0.,
            entry_reference_altitude: 0.,
            entry_reference_range: 0.,
            has_entry_predictive_score: false,
            entry_control_plan_valid: false,
            entry_control_terminal_ready: false,
            control_plan_sequence: 0,
            entry_control_plan_ut: 0.,
            entry_control_bank: 0.,
            entry_control_aoa: 0.,
            entry_control_heading: 0.,
            entry_control_turn_radius: 0.,
            entry_control_segment_until_ut: 0.,
            entry_control_cost: 0.,
            entry_control_taem_range_error: 0.,
            entry_control_taem_speed: 0.,
            entry_control_taem_energy_error: 0.,
            entry_reversal_scheduled: false,
            entry_reversal_is_final: false,
            entry_final_reversal_pending: false,
            entry_final_reversal_completed: false,
            entry_continuation_bootstrap: false,
            entry_target_side_latched: false,
            entry_target_side: 0.,
            entry_reversal_ut: 0.,
            entry_reversal_range: 0.,
            entry_reversal_sign: 0.,
            entry_reversal_bank: 0.,
            entry_control_reversals: 0,
            entry_feedback: EntryGuidanceFeedback {
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
            entry_lateral_bank_magnitude: 0.,
            entry_geometry_bank: 0.,
            entry_vertical_bank_magnitude: 0.,
            entry_demand_bank: 0.,
            entry_lateral_required_bank: 0.,
            abort_reason: [0; 384],
            entry_energy_pid: RobustPID {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                integral_limit: 0.,
                derivative_tc: 0.,
                integral: 0.,
                previous_error: 0.,
                filtered_derivative: 0.,
                has_previous: false,
            },
            taem_altitude_pid: RobustPID {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                integral_limit: 0.,
                derivative_tc: 0.,
                integral: 0.,
                previous_error: 0.,
                filtered_derivative: 0.,
                has_previous: false,
            },
            final_altitude_pid: RobustPID {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                integral_limit: 0.,
                derivative_tc: 0.,
                integral: 0.,
                previous_error: 0.,
                filtered_derivative: 0.,
                has_previous: false,
            },
            speed_pid: RobustPID {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                integral_limit: 0.,
                derivative_tc: 0.,
                integral: 0.,
                previous_error: 0.,
                filtered_derivative: 0.,
                has_previous: false,
            },
            flare_sink_pid: RobustPID {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                integral_limit: 0.,
                derivative_tc: 0.,
                integral: 0.,
                previous_error: 0.,
                filtered_derivative: 0.,
                has_previous: false,
            },
            speedbrake_controller: SpeedbrakeController {
                integral: 0.,
                deployed: false,
                initialized: false,
            },
            pitch_limiter: JerkLimiter {
                value: 0.,
                rate: 0.,
                has_value: false,
            },
            roll_limiter: JerkLimiter {
                value: 0.,
                rate: 0.,
                has_value: false,
            },
            heading_limiter: JerkLimiter {
                value: 0.,
                rate: 0.,
                has_value: false,
            },
            throttle_limiter: SlewLimiter {
                value: 0.,
                has_value: false,
            },
            last_stabilized_phase: PHASE_IDLE,
            has_last_stabilized_phase: false,
            taem_energy_law: TaemEnergyLawState {
                initialized: false,
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
            },
            taem_hac_law: TaemHacLawState {
                initialized: false,
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
            },
            taem_hac_live_aero: TaemHacLiveAero {
                drag: [0.; 15],
                lift: [0.; 15],
                weight: [0.; 15],
                last_ut: 0.,
            },
            adaptive_flare: AdaptiveFlareState {
                sxx: [[0.; 3]; 3],
                sxy: [0.; 3],
                last_aoa: 0.,
                last_gamma: 0.,
                last_ut: 0.,
                has_last: false,
                tau: 0.,
                bias: 0.,
                gain: 0.,
                glide_sink: 0.,
                aim_shift: 0.,
                weight: 0.,
                plan_valid: false,
                active: false,
                arc_accel: 0.,
                sgs_height: 0.,
                cost: 0.,
                pred_speed: 0.,
                pred_sink: 0.,
                pred_along: 0.,
                pred_peak_n: 0.,
                pred_climb: 0.,
                plan_ut: 0.,
            },
            jet_assist_throttle: 0.,
        mm304: crate::mm304::Mm304State::ZERO,
        },
        hac_radius_m: 0.,
        final_approach_distance_m: 0.,
        search_both_ends: false,
        upstream_end: 0,
        restrict_side: false,
        side: 0.,
        lift_scale: 0.,
        drag_scale: 0.,
        scale_mach: 0.,
        seed_route_valid: false,
        seed_route: TaemRoute {
            valid: false,
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
        },
        seed_runway_end: 0,
    };
    memset(
        &raw mut r as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Mm305PlanRequest>() as size_t,
    );
    if g.is_null() || t.is_null() || state.is_null() || cfg.is_null() || model.is_null()
        || !(*model).replay_validated
    {
        return r;
    }
    r.state = terminal_live_state(g, state, t, model);
    if !terminal_state_validate(
        &raw mut r.state,
        ::core::ptr::null_mut::<::core::ffi::c_char>(),
        0 as size_t,
    ) {
        return r;
    }
    r.request_ut = (*t).ut;
    r.final_guidance = *g;
    r.model_snapshot_id = (*model).snapshot_id;
    r.hac_radius_m = if (*g).taem_interface_target.valid as ::core::ffi::c_int != 0
        && (*g).taem_interface_target.hac_radius.is_finite() as i32 != 0
        && (*g).taem_interface_target.hac_radius > 0.0f64
    {
        (*g).taem_interface_target.hac_radius
    } else {
        (*g).hac_radius
    };
    if !(r.hac_radius_m > 0.0f64) {
        r.hac_radius_m = (*cfg).guidance.hac_radius;
    }
    r.final_approach_distance_m = mm305_final_alignment_distance(
        g,
        model,
        &raw mut r.state,
    );
    if !(r.final_approach_distance_m >= 500.0f64)
        || r.final_approach_distance_m.is_finite() as i32 == 0
    {
        r.final_approach_distance_m = (*cfg).guidance.final_approach_distance;
    }
    r.search_both_ends = (*cfg).site.allow_reciprocal_runway as ::core::ffi::c_int != 0
        && !(*g).mm305_route_committed;
    r.upstream_end = if (*g).runway_end_preview_valid as ::core::ffi::c_int != 0
        && (*g).runway_end_index == 1 as ::core::ffi::c_int
    {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
    r.restrict_side = (*g).mm305_route_committed as ::core::ffi::c_int != 0
        || (*g).mm305_replans > 0 as ::core::ffi::c_uint;
    r.side = (*g).hac_side;
    r.lift_scale = if (*g).mm305_lift_scale.is_finite() as i32 != 0
        && (*g).mm305_lift_scale > 0.0f64
    {
        (*g).mm305_lift_scale
    } else {
        1.0f64
    };
    r.drag_scale = if (*g).mm305_drag_scale.is_finite() as i32 != 0
        && (*g).mm305_drag_scale > 0.0f64
    {
        (*g).mm305_drag_scale
    } else {
        1.0f64
    };
    let mut observed: AeroForces = aero_compute(
        &raw const (*model).world,
        &raw const (*model).aero,
        r.state.position_i_m,
        r.state.velocity_i_mps,
        r.state.ut_s,
        r.state.mass_kg,
        r.state.attitude.aoa_rad,
        r.state.attitude.bank_rad,
    );
    r.scale_mach = observed.mach;
    if (*g).mm305_acquisition_route_valid as ::core::ffi::c_int != 0
        && (*g).mm305_acquisition_route.valid as ::core::ffi::c_int != 0
    {
        r.seed_route_valid = true_0 != 0;
        r.seed_route = (*g).mm305_acquisition_route;
        r.seed_runway_end = r.upstream_end;
    }
    r.valid = true_0 != 0;
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_plan_request(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut cfg: *const LandingConfiguration,
    mut model: *const TerminalModel,
) -> Mm305PlanRequest {
    if g.is_null() || !(*g).mm305_planning_needed {
        let mut r: Mm305PlanRequest = Mm305PlanRequest {
            valid: false,
            request_ut: 0.,
            model_snapshot_id: 0,
            state: TerminalDynamicState {
                position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                attitude: AttitudeModel {
                    aoa_rad: 0.,
                    bank_rad: 0.,
                    aoa_rate_rad_s: 0.,
                    bank_rate_rad_s: 0.,
                    requested_aoa_rad: 0.,
                    requested_bank_rad: 0.,
                    cmd_aoa_rad: 0.,
                    cmd_bank_rad: 0.,
                    pitch_wn: 0.,
                    pitch_zeta: 0.,
                    roll_wn: 0.,
                    roll_zeta: 0.,
                    max_pitch_rate_rad_s: 0.,
                    max_roll_rate_rad_s: 0.,
                    max_pitch_accel_rad_s2: 0.,
                    max_roll_accel_rad_s2: 0.,
                    pitch_full_authority_q_pa: 0.,
                    roll_full_authority_q_pa: 0.,
                    direct: false,
                    direct_body_coordinates: false,
                    body_rates_initialized: false,
                    body_pitch_rate_rad_s: 0.,
                    body_roll_rate_rad_s: 0.,
                    body_yaw_rate_rad_s: 0.,
                    input_pitch: 0.,
                    input_roll: 0.,
                    input_yaw: 0.,
                    sideslip_rad: 0.,
                    sideslip_rate_rad_s: 0.,
                    direct_min_q_pa: 0.,
                    direct_pitch_accel_per_kpa: 0.,
                    direct_pitch_accel_max: 0.,
                    direct_pitch_stiffness_per_kpa: 0.,
                    direct_trim_aoa_rad: 0.,
                    direct_pitch_damping_ratio: 0.,
                    direct_roll_accel_per_kpa: 0.,
                    direct_roll_accel_max: 0.,
                    direct_roll_damping_s_inv: 0.,
                    direct_yaw_accel_per_kpa: 0.,
                    direct_yaw_accel_max: 0.,
                    direct_yaw_stiffness_per_kpa: 0.,
                },
                mass_kg: 0.,
                ut_s: 0.,
                trim_aoa_ceiling_rad: 0.,
            },
            final_guidance: GuidanceMachine {
                phase: PHASE_IDLE,
                automation_engaged: false,
                paused: false,
                aborted: false,
                entry_terminal_model: ::core::ptr::null::<TerminalModel>(),
                diagnostic_shadow: false,
                diagnostic_stop_at_taem: false,
                diagnostic_entry_previous_range_m: 0.,
                diagnostic_entry_milestone_mask: 0,
                diagnostic_entry_has_previous_range: false,
                s_turn_sign: 0.,
                s_turn_leg_started_ut: 0.,
                has_s_turn_leg_started: false,
                s_turn_reversal_requested_ut: 0.,
                has_s_turn_reversal_requested: false,
                entry_exec: EntryExecutive {
                    initialized: false,
                    entry_complete: false,
                    phase: 0 as EntryPhase,
                    last_transition_reason: ENTRY_TRANSITION_NONE,
                    last_transition_from: 0 as EntryPhase,
                    last_transition_to: 0 as EntryPhase,
                    last_transition_ut: 0.,
                    last_transition_velocity: 0.,
                    last_transition_completed_entry: false,
                    transition_count: 0,
                },
                entry_lateral: EntryLateralState {
                    initialized: false,
                    bank_sign: 0.,
                    leg_captured: false,
                    leg_captured_ut: 0.,
                    reversal_armed: false,
                    last_reversal_ut: 0.,
                    command_bank_deg: 0.,
                    command_bank_rate_deg_s: 0.,
                    has_previous_sideslip: false,
                    previous_sideslip_deg: 0.,
                },
                entry_energy: EntryEnergyState {
                    initialized: false,
                    integral_m_s: 0.,
                    last_ut: 0.,
                },
                taem_exec: TaemExecutive {
                    initialized: false,
                    ownership_latched: false,
                    taem_complete: false,
                    phase: TAEM_PHASE_PATH_ACQUISITION,
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
                entry_alpha_has_target: false,
                entry_alpha_has_modulation: false,
                entry_drag_ratio_valid: false,
                entry_alpha_target: 0.,
                entry_alpha_modulation: 0.,
                entry_alpha_drag_boost: 0.,
                entry_drag_velocity_ratio: 0.,
                entry_alpha_deficit_filter: 0.,
                entry_alpha_ceiling_filter: 0.,
                entry_alpha_filter_valid: false,
                entry_bank_authority_acquired: false,
                entry_supervision_valid: false,
                entry_supervision_boundary_missed: false,
                entry_supervision_mode: ENTRY_SUPERVISION_PASS_THROUGH,
                entry_policy_valid: false,
                entry_policy_bank_deg: 0.,
                entry_policy_aoa_deg: 0.,
                entry_policy_ut: 0.,
                entry_energy_short_latched: false,
                entry_handoff_target_range: 0.,
                entry_hac_energy_excess: 0.,
                entry_hac_energy_ut: 0.,
                taem_runway_end_scored: false,
                taem_runway_end_score_ut: 0.,
                entry_supervision_ut: 0.,
                entry_planning_deferred: false,
                entry_planning_needed: false,
                entry_lateral_infeasible: false,
                entry_committed_infeasible: false,
                entry_s_turn_plan: EntryControlPlan {
                    valid: false,
                    terminal_ready: false,
                    plan_id: 0,
                    plan_version: 0,
                    parent_plan_id: 0,
                    parent_plan_version: 0,
                    planned_ut: 0.,
                    target_bank: 0.,
                    target_aoa: 0.,
                    target_heading: 0.,
                    bank_cap: 0.,
                    target_turn_radius: 0.,
                    segment_duration: 0.,
                    cost: 0.,
                    taem_range_error: 0.,
                    taem_speed: 0.,
                    taem_energy_error: 0.,
                    closest_distance: 0.,
                    has_planned_reversal: false,
                    planned_reversal_is_final: false,
                    final_heading_lock: false,
                    planned_reversal_ut: 0.,
                    planned_reversal_range: 0.,
                    planned_reversal_sign: 0.,
                    predicted_reversals: 0,
                },
                entry_predictor_models_valid: false,
                entry_predictor_envelope: AerodynamicEnvelope {
                    regimes: [AerodynamicModel {
                        lift_to_drag: 0.,
                        ballistic_coefficient: 0.,
                        confidence: 0.,
                    }; 4],
                },
                entry_predictor_calibration: TrajectoryCalibrationModel {
                    physics: ::core::ptr::null::<VesselPhysicsModel>(),
                    density_scale: 0.,
                    drag_scale: 0.,
                    lift_scale: 0.,
                    bank_effectiveness: 0.,
                    speed_of_sound: 0.,
                    speed_of_sound_scale: 0.,
                    stress_drag_scale: 0.,
                    stress_lift_scale: 0.,
                    forecast_drag_scale: 0.,
                    forecast_lift_scale: 0.,
                    forecast_exact_policy_bank: false,
                    altitude_residual: 0.,
                    speed_residual: 0.,
                    range_residual: 0.,
                    confidence: 0.,
                    accepted_samples: 0,
                },
                mm305_route: TaemRoute {
                    valid: false,
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
                },
                mm305_route_cursor: 0,
                mm305_route_committed: false,
                mm305_hac_exit_reached: false,
                mm305_direct_approach_degraded: false,
                mm305_best_effort_route_active: false,
                mm305_runway_alignment_active: false,
                mm305_model_snapshot_id: 0,
                mm305_acquisition_route: TaemRoute {
                    valid: false,
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
                },
                mm305_acquisition_route_cursor: 0,
                mm305_acquisition_route_valid: false,
                mm305_acquisition_route_ut: 0.,
                mm305_acquisition_sweep_rad: 0.,
                mm305_async_planning: false,
                mm305_planning_needed: false,
                mm305_plan_request_ut: 0.,
                mm305_last_plan_attempt_ut: 0.,
                mm305_last_route_ut: 0.,
                mm305_plan_failures: 0,
                mm305_replans: 0,
                mm305_lift_scale: 0.,
                mm305_drag_scale: 0.,
                mm305_command_bank_rad: 0.,
                mm305_command_aoa_rad: 0.,
                mm305_command_ut: 0.,
                mm305_command_valid: false,
                terminal_landing_warning_latched: false,
                mm305_trim: AdaptiveTrim {
                    weight: 0.,
                    x: 0.,
                    y: 0.,
                    xx: 0.,
                    xy: 0.,
                    yy: 0.,
                    lo: 0.,
                    hi: 0.,
                    last_ut: 0.,
                    mach: 0.,
                    pressure: 0.,
                    clock_valid: false,
                    regime_valid: false,
                },
                mm305_admission_needed: false,
                mm305_admission_request_ut: 0.,
                mm305_admission_last_attempt_ut: 0.,
                mm305_admission_valid: false,
                mm305_admission_found: false,
                mm305_admission_result_ut: 0.,
                mm305_admission_snapshot_id: 0,
                mm305_admission_route: TaemRoute {
                    valid: false,
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
                },
                mm305_admission_runway_end: 0,
                mm305_admission_side: 0.,
                mm305_admission_solve_wall_s: 0.,
                mm305_admission_lift_scale: 0.,
                mm305_admission_drag_scale: 0.,
                mm305_admission_scale_mach: 0.,
                mm305_admission_rejections: 0,
                taem_interface_target: TaemInterfaceTarget {
                    valid: false,
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
                },
                taem_interface_captured: false,
                taem_safety_handoff: false,
                taem_interface_diagnostic_ut: 0.,
                terminal_path_kind: TERMINAL_PATH_NONE,
                terminal_path_committed: false,
                runway_end_preview_valid: false,
                runway_end_committed: false,
                runway_end_index: 0,
                runway_end_primary_path: 0.,
                runway_end_reciprocal_path: 0.,
                terminal_final_handoff_latched: false,
                terminal_final_handoff_distance: 0.,
                terminal_final_handoff_slope_deg: 0.,
                terminal_final_handoff_speed_mps: 0.,
                hac_side: 0.,
                hac_side_selected: false,
                delivered_delta_v: 0.,
                burn_command_started_ut: 0.,
                burn_active_elapsed: 0.,
                burn_progress_watch_ut: 0.,
                burn_progress_watch_delta_v: 0.,
                burn_thrust_loss_elapsed: 0.,
                has_burn_command_started: false,
                has_burn_progress_watch: false,
                deorbit_burn_completed: false,
                atmospheric_interface_crossed: false,
                has_previous_ut: false,
                previous_ut: 0.,
                final_approach_captured: false,
                airbrakes_deployed: false,
                terminal_final_low_energy: false,
                hac_progress_valid: false,
                hac_captured: false,
                hac_completed: false,
                terminal_region_entered: false,
                terminal_glide_mode: false,
                terminal_final_test_mode: false,
                terminal_rehearsal_mode: false,
                terminal_test_capture_active: false,
                hac_capture_lost_duration: 0.,
                terminal_energy_mismatch_duration: 0.,
                terminal_reentry_after_ut: 0.,
                hac_remaining: 0.,
                hac_radius: 0.,
                minimum_turn_radius: 0.,
                terminal_test_glide_slope: 0.,
                terminal_test_final_approach_distance: 0.,
                final_invalid_duration: 0.,
                terminal_test_revolution_remaining: 0.,
                terminal_test_preflare_altitude: 0.,
                terminal_test_preflare_target_speed: 0.,
                terminal_test_preflare_min_speed: 0.,
                terminal_vertical_stage: TERMINAL_TRAJECTORY_CAPTURE,
                terminal_vertical_stage_valid: false,
                gear_command_latched: false,
                ground_contact_latched: false,
                flare_sink_captured: false,
                terminal_response_sample_valid: false,
                terminal_preflare_plan_valid: false,
                terminal_energy_sample_valid: false,
                terminal_stage_started_ut: 0.,
                terminal_stage_good_duration: 0.,
                ground_contact_duration: 0.,
                terminal_previous_aoa: 0.,
                terminal_previous_vertical_speed: 0.,
                terminal_response_sample_ut: 0.,
                terminal_previous_specific_energy: 0.,
                terminal_previous_speed: 0.,
                terminal_energy_sample_ut: 0.,
                terminal_energy_loss_accel_ema: 0.,
                terminal_speed_loss_accel_ema: 0.,
                terminal_lift_area_ema: 0.,
                terminal_drag_area_ema: 0.,
                terminal_aero_sample_ut: 0.,
                terminal_aero_sample_valid: false,
                terminal_predicted_touchdown_speed: 0.,
                terminal_speedbrake_cda: 0.,
                terminal_speedbrake_fraction: 0.,
                terminal_split_fin_cda: 0.,
                terminal_vs_prev: 0.,
                terminal_vs_prev_ut: 0.,
                terminal_vs_rate_ema: 0.,
                terminal_lift_q: [0.; 21],
                terminal_drag_q: [0.; 21],
                terminal_aero_seen: [0; 21],
                terminal_aero_mach: [0.; 21],
                terminal_aoa_cmd_last: 0.,
                terminal_aoa_cmd_ut: 0.,
                taem_speedbrake_integral: 0.,
                taem_speedbrake_ut: 0.,
                taem_speedbrake_fraction: 0.,
                terminal_pull_latch: false,
                terminal_arc_started: false,
                terminal_aim_valid: false,
                terminal_aim_along: 0.,
                terminal_aim_shift: 0.,
                terminal_predicted_touchdown_along: 0.,
                terminal_aim_ut: 0.,
                terminal_lift_ratio: 0.,
                terminal_positive_aoa_rate_ema: 0.,
                terminal_sink_accel_ema: 0.,
                terminal_pitch_response_delay_ema: 0.,
                preflare_trigger_altitude: 0.,
                preflare_target_aoa: 0.,
                preflare_target_sink: 0.,
                preflare_minimum_speed: 0.,
                preflare_reference_speed: 0.,
                preflare_predicted_height_loss: 0.,
                preflare_predicted_kinetic_margin: 0.,
                preflare_effective_accel: 0.,
                preflare_response_time: 0.,
                preflare_plan_source_ut: 0.,
                attitude_recovery: false,
                control_bad_duration: 0.,
                control_good_duration: 0.,
                recovery_duration: 0.,
                recovery_heading: 0.,
                recovery_aoa: 0.,
                recovery_pitch_trim: 0.,
                previous_relative_roll_rate: 0.,
                roll_oscillation_score: 0.,
                roll_rate_excess_duration: 0.,
                has_previous_relative_roll_rate: false,
                previous_sideslip: 0.,
                entry_coordination_beta_rate: 0.,
                has_previous_sideslip: false,
                entry_predictive_score: 0.,
                entry_reference_speed: 0.,
                entry_reference_altitude: 0.,
                entry_reference_range: 0.,
                has_entry_predictive_score: false,
                entry_control_plan_valid: false,
                entry_control_terminal_ready: false,
                control_plan_sequence: 0,
                entry_control_plan_ut: 0.,
                entry_control_bank: 0.,
                entry_control_aoa: 0.,
                entry_control_heading: 0.,
                entry_control_turn_radius: 0.,
                entry_control_segment_until_ut: 0.,
                entry_control_cost: 0.,
                entry_control_taem_range_error: 0.,
                entry_control_taem_speed: 0.,
                entry_control_taem_energy_error: 0.,
                entry_reversal_scheduled: false,
                entry_reversal_is_final: false,
                entry_final_reversal_pending: false,
                entry_final_reversal_completed: false,
                entry_continuation_bootstrap: false,
                entry_target_side_latched: false,
                entry_target_side: 0.,
                entry_reversal_ut: 0.,
                entry_reversal_range: 0.,
                entry_reversal_sign: 0.,
                entry_reversal_bank: 0.,
                entry_control_reversals: 0,
                entry_feedback: EntryGuidanceFeedback {
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
                entry_lateral_bank_magnitude: 0.,
                entry_geometry_bank: 0.,
                entry_vertical_bank_magnitude: 0.,
                entry_demand_bank: 0.,
                entry_lateral_required_bank: 0.,
                abort_reason: [0; 384],
                entry_energy_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                taem_altitude_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                final_altitude_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                speed_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                flare_sink_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                speedbrake_controller: SpeedbrakeController {
                    integral: 0.,
                    deployed: false,
                    initialized: false,
                },
                pitch_limiter: JerkLimiter {
                    value: 0.,
                    rate: 0.,
                    has_value: false,
                },
                roll_limiter: JerkLimiter {
                    value: 0.,
                    rate: 0.,
                    has_value: false,
                },
                heading_limiter: JerkLimiter {
                    value: 0.,
                    rate: 0.,
                    has_value: false,
                },
                throttle_limiter: SlewLimiter {
                    value: 0.,
                    has_value: false,
                },
                last_stabilized_phase: PHASE_IDLE,
                has_last_stabilized_phase: false,
                taem_energy_law: TaemEnergyLawState {
                    initialized: false,
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
                },
                taem_hac_law: TaemHacLawState {
                    initialized: false,
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
                },
                taem_hac_live_aero: TaemHacLiveAero {
                    drag: [0.; 15],
                    lift: [0.; 15],
                    weight: [0.; 15],
                    last_ut: 0.,
                },
                adaptive_flare: AdaptiveFlareState {
                    sxx: [[0.; 3]; 3],
                    sxy: [0.; 3],
                    last_aoa: 0.,
                    last_gamma: 0.,
                    last_ut: 0.,
                    has_last: false,
                    tau: 0.,
                    bias: 0.,
                    gain: 0.,
                    glide_sink: 0.,
                    aim_shift: 0.,
                    weight: 0.,
                    plan_valid: false,
                    active: false,
                    arc_accel: 0.,
                    sgs_height: 0.,
                    cost: 0.,
                    pred_speed: 0.,
                    pred_sink: 0.,
                    pred_along: 0.,
                    pred_peak_n: 0.,
                    pred_climb: 0.,
                    plan_ut: 0.,
                },
                jet_assist_throttle: 0.,
        mm304: crate::mm304::Mm304State::ZERO,
            },
            hac_radius_m: 0.,
            final_approach_distance_m: 0.,
            search_both_ends: false,
            upstream_end: 0,
            restrict_side: false,
            side: 0.,
            lift_scale: 0.,
            drag_scale: 0.,
            scale_mach: 0.,
            seed_route_valid: false,
            seed_route: TaemRoute {
                valid: false,
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
            },
            seed_runway_end: 0,
        };
        memset(
            &raw mut r as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<Mm305PlanRequest>() as size_t,
        );
        return r;
    }
    return mm305_request_from_state(g, t, state, cfg, model);
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_admission_request(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut cfg: *const LandingConfiguration,
    mut model: *const TerminalModel,
) -> Mm305PlanRequest {
    if g.is_null() || !(*g).mm305_admission_needed {
        let mut r: Mm305PlanRequest = Mm305PlanRequest {
            valid: false,
            request_ut: 0.,
            model_snapshot_id: 0,
            state: TerminalDynamicState {
                position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                attitude: AttitudeModel {
                    aoa_rad: 0.,
                    bank_rad: 0.,
                    aoa_rate_rad_s: 0.,
                    bank_rate_rad_s: 0.,
                    requested_aoa_rad: 0.,
                    requested_bank_rad: 0.,
                    cmd_aoa_rad: 0.,
                    cmd_bank_rad: 0.,
                    pitch_wn: 0.,
                    pitch_zeta: 0.,
                    roll_wn: 0.,
                    roll_zeta: 0.,
                    max_pitch_rate_rad_s: 0.,
                    max_roll_rate_rad_s: 0.,
                    max_pitch_accel_rad_s2: 0.,
                    max_roll_accel_rad_s2: 0.,
                    pitch_full_authority_q_pa: 0.,
                    roll_full_authority_q_pa: 0.,
                    direct: false,
                    direct_body_coordinates: false,
                    body_rates_initialized: false,
                    body_pitch_rate_rad_s: 0.,
                    body_roll_rate_rad_s: 0.,
                    body_yaw_rate_rad_s: 0.,
                    input_pitch: 0.,
                    input_roll: 0.,
                    input_yaw: 0.,
                    sideslip_rad: 0.,
                    sideslip_rate_rad_s: 0.,
                    direct_min_q_pa: 0.,
                    direct_pitch_accel_per_kpa: 0.,
                    direct_pitch_accel_max: 0.,
                    direct_pitch_stiffness_per_kpa: 0.,
                    direct_trim_aoa_rad: 0.,
                    direct_pitch_damping_ratio: 0.,
                    direct_roll_accel_per_kpa: 0.,
                    direct_roll_accel_max: 0.,
                    direct_roll_damping_s_inv: 0.,
                    direct_yaw_accel_per_kpa: 0.,
                    direct_yaw_accel_max: 0.,
                    direct_yaw_stiffness_per_kpa: 0.,
                },
                mass_kg: 0.,
                ut_s: 0.,
                trim_aoa_ceiling_rad: 0.,
            },
            final_guidance: GuidanceMachine {
                phase: PHASE_IDLE,
                automation_engaged: false,
                paused: false,
                aborted: false,
                entry_terminal_model: ::core::ptr::null::<TerminalModel>(),
                diagnostic_shadow: false,
                diagnostic_stop_at_taem: false,
                diagnostic_entry_previous_range_m: 0.,
                diagnostic_entry_milestone_mask: 0,
                diagnostic_entry_has_previous_range: false,
                s_turn_sign: 0.,
                s_turn_leg_started_ut: 0.,
                has_s_turn_leg_started: false,
                s_turn_reversal_requested_ut: 0.,
                has_s_turn_reversal_requested: false,
                entry_exec: EntryExecutive {
                    initialized: false,
                    entry_complete: false,
                    phase: 0 as EntryPhase,
                    last_transition_reason: ENTRY_TRANSITION_NONE,
                    last_transition_from: 0 as EntryPhase,
                    last_transition_to: 0 as EntryPhase,
                    last_transition_ut: 0.,
                    last_transition_velocity: 0.,
                    last_transition_completed_entry: false,
                    transition_count: 0,
                },
                entry_lateral: EntryLateralState {
                    initialized: false,
                    bank_sign: 0.,
                    leg_captured: false,
                    leg_captured_ut: 0.,
                    reversal_armed: false,
                    last_reversal_ut: 0.,
                    command_bank_deg: 0.,
                    command_bank_rate_deg_s: 0.,
                    has_previous_sideslip: false,
                    previous_sideslip_deg: 0.,
                },
                entry_energy: EntryEnergyState {
                    initialized: false,
                    integral_m_s: 0.,
                    last_ut: 0.,
                },
                taem_exec: TaemExecutive {
                    initialized: false,
                    ownership_latched: false,
                    taem_complete: false,
                    phase: TAEM_PHASE_PATH_ACQUISITION,
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
                entry_alpha_has_target: false,
                entry_alpha_has_modulation: false,
                entry_drag_ratio_valid: false,
                entry_alpha_target: 0.,
                entry_alpha_modulation: 0.,
                entry_alpha_drag_boost: 0.,
                entry_drag_velocity_ratio: 0.,
                entry_alpha_deficit_filter: 0.,
                entry_alpha_ceiling_filter: 0.,
                entry_alpha_filter_valid: false,
                entry_bank_authority_acquired: false,
                entry_supervision_valid: false,
                entry_supervision_boundary_missed: false,
                entry_supervision_mode: ENTRY_SUPERVISION_PASS_THROUGH,
                entry_policy_valid: false,
                entry_policy_bank_deg: 0.,
                entry_policy_aoa_deg: 0.,
                entry_policy_ut: 0.,
                entry_energy_short_latched: false,
                entry_handoff_target_range: 0.,
                entry_hac_energy_excess: 0.,
                entry_hac_energy_ut: 0.,
                taem_runway_end_scored: false,
                taem_runway_end_score_ut: 0.,
                entry_supervision_ut: 0.,
                entry_planning_deferred: false,
                entry_planning_needed: false,
                entry_lateral_infeasible: false,
                entry_committed_infeasible: false,
                entry_s_turn_plan: EntryControlPlan {
                    valid: false,
                    terminal_ready: false,
                    plan_id: 0,
                    plan_version: 0,
                    parent_plan_id: 0,
                    parent_plan_version: 0,
                    planned_ut: 0.,
                    target_bank: 0.,
                    target_aoa: 0.,
                    target_heading: 0.,
                    bank_cap: 0.,
                    target_turn_radius: 0.,
                    segment_duration: 0.,
                    cost: 0.,
                    taem_range_error: 0.,
                    taem_speed: 0.,
                    taem_energy_error: 0.,
                    closest_distance: 0.,
                    has_planned_reversal: false,
                    planned_reversal_is_final: false,
                    final_heading_lock: false,
                    planned_reversal_ut: 0.,
                    planned_reversal_range: 0.,
                    planned_reversal_sign: 0.,
                    predicted_reversals: 0,
                },
                entry_predictor_models_valid: false,
                entry_predictor_envelope: AerodynamicEnvelope {
                    regimes: [AerodynamicModel {
                        lift_to_drag: 0.,
                        ballistic_coefficient: 0.,
                        confidence: 0.,
                    }; 4],
                },
                entry_predictor_calibration: TrajectoryCalibrationModel {
                    physics: ::core::ptr::null::<VesselPhysicsModel>(),
                    density_scale: 0.,
                    drag_scale: 0.,
                    lift_scale: 0.,
                    bank_effectiveness: 0.,
                    speed_of_sound: 0.,
                    speed_of_sound_scale: 0.,
                    stress_drag_scale: 0.,
                    stress_lift_scale: 0.,
                    forecast_drag_scale: 0.,
                    forecast_lift_scale: 0.,
                    forecast_exact_policy_bank: false,
                    altitude_residual: 0.,
                    speed_residual: 0.,
                    range_residual: 0.,
                    confidence: 0.,
                    accepted_samples: 0,
                },
                mm305_route: TaemRoute {
                    valid: false,
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
                },
                mm305_route_cursor: 0,
                mm305_route_committed: false,
                mm305_hac_exit_reached: false,
                mm305_direct_approach_degraded: false,
                mm305_best_effort_route_active: false,
                mm305_runway_alignment_active: false,
                mm305_model_snapshot_id: 0,
                mm305_acquisition_route: TaemRoute {
                    valid: false,
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
                },
                mm305_acquisition_route_cursor: 0,
                mm305_acquisition_route_valid: false,
                mm305_acquisition_route_ut: 0.,
                mm305_acquisition_sweep_rad: 0.,
                mm305_async_planning: false,
                mm305_planning_needed: false,
                mm305_plan_request_ut: 0.,
                mm305_last_plan_attempt_ut: 0.,
                mm305_last_route_ut: 0.,
                mm305_plan_failures: 0,
                mm305_replans: 0,
                mm305_lift_scale: 0.,
                mm305_drag_scale: 0.,
                mm305_command_bank_rad: 0.,
                mm305_command_aoa_rad: 0.,
                mm305_command_ut: 0.,
                mm305_command_valid: false,
                terminal_landing_warning_latched: false,
                mm305_trim: AdaptiveTrim {
                    weight: 0.,
                    x: 0.,
                    y: 0.,
                    xx: 0.,
                    xy: 0.,
                    yy: 0.,
                    lo: 0.,
                    hi: 0.,
                    last_ut: 0.,
                    mach: 0.,
                    pressure: 0.,
                    clock_valid: false,
                    regime_valid: false,
                },
                mm305_admission_needed: false,
                mm305_admission_request_ut: 0.,
                mm305_admission_last_attempt_ut: 0.,
                mm305_admission_valid: false,
                mm305_admission_found: false,
                mm305_admission_result_ut: 0.,
                mm305_admission_snapshot_id: 0,
                mm305_admission_route: TaemRoute {
                    valid: false,
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
                },
                mm305_admission_runway_end: 0,
                mm305_admission_side: 0.,
                mm305_admission_solve_wall_s: 0.,
                mm305_admission_lift_scale: 0.,
                mm305_admission_drag_scale: 0.,
                mm305_admission_scale_mach: 0.,
                mm305_admission_rejections: 0,
                taem_interface_target: TaemInterfaceTarget {
                    valid: false,
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
                },
                taem_interface_captured: false,
                taem_safety_handoff: false,
                taem_interface_diagnostic_ut: 0.,
                terminal_path_kind: TERMINAL_PATH_NONE,
                terminal_path_committed: false,
                runway_end_preview_valid: false,
                runway_end_committed: false,
                runway_end_index: 0,
                runway_end_primary_path: 0.,
                runway_end_reciprocal_path: 0.,
                terminal_final_handoff_latched: false,
                terminal_final_handoff_distance: 0.,
                terminal_final_handoff_slope_deg: 0.,
                terminal_final_handoff_speed_mps: 0.,
                hac_side: 0.,
                hac_side_selected: false,
                delivered_delta_v: 0.,
                burn_command_started_ut: 0.,
                burn_active_elapsed: 0.,
                burn_progress_watch_ut: 0.,
                burn_progress_watch_delta_v: 0.,
                burn_thrust_loss_elapsed: 0.,
                has_burn_command_started: false,
                has_burn_progress_watch: false,
                deorbit_burn_completed: false,
                atmospheric_interface_crossed: false,
                has_previous_ut: false,
                previous_ut: 0.,
                final_approach_captured: false,
                airbrakes_deployed: false,
                terminal_final_low_energy: false,
                hac_progress_valid: false,
                hac_captured: false,
                hac_completed: false,
                terminal_region_entered: false,
                terminal_glide_mode: false,
                terminal_final_test_mode: false,
                terminal_rehearsal_mode: false,
                terminal_test_capture_active: false,
                hac_capture_lost_duration: 0.,
                terminal_energy_mismatch_duration: 0.,
                terminal_reentry_after_ut: 0.,
                hac_remaining: 0.,
                hac_radius: 0.,
                minimum_turn_radius: 0.,
                terminal_test_glide_slope: 0.,
                terminal_test_final_approach_distance: 0.,
                final_invalid_duration: 0.,
                terminal_test_revolution_remaining: 0.,
                terminal_test_preflare_altitude: 0.,
                terminal_test_preflare_target_speed: 0.,
                terminal_test_preflare_min_speed: 0.,
                terminal_vertical_stage: TERMINAL_TRAJECTORY_CAPTURE,
                terminal_vertical_stage_valid: false,
                gear_command_latched: false,
                ground_contact_latched: false,
                flare_sink_captured: false,
                terminal_response_sample_valid: false,
                terminal_preflare_plan_valid: false,
                terminal_energy_sample_valid: false,
                terminal_stage_started_ut: 0.,
                terminal_stage_good_duration: 0.,
                ground_contact_duration: 0.,
                terminal_previous_aoa: 0.,
                terminal_previous_vertical_speed: 0.,
                terminal_response_sample_ut: 0.,
                terminal_previous_specific_energy: 0.,
                terminal_previous_speed: 0.,
                terminal_energy_sample_ut: 0.,
                terminal_energy_loss_accel_ema: 0.,
                terminal_speed_loss_accel_ema: 0.,
                terminal_lift_area_ema: 0.,
                terminal_drag_area_ema: 0.,
                terminal_aero_sample_ut: 0.,
                terminal_aero_sample_valid: false,
                terminal_predicted_touchdown_speed: 0.,
                terminal_speedbrake_cda: 0.,
                terminal_speedbrake_fraction: 0.,
                terminal_split_fin_cda: 0.,
                terminal_vs_prev: 0.,
                terminal_vs_prev_ut: 0.,
                terminal_vs_rate_ema: 0.,
                terminal_lift_q: [0.; 21],
                terminal_drag_q: [0.; 21],
                terminal_aero_seen: [0; 21],
                terminal_aero_mach: [0.; 21],
                terminal_aoa_cmd_last: 0.,
                terminal_aoa_cmd_ut: 0.,
                taem_speedbrake_integral: 0.,
                taem_speedbrake_ut: 0.,
                taem_speedbrake_fraction: 0.,
                terminal_pull_latch: false,
                terminal_arc_started: false,
                terminal_aim_valid: false,
                terminal_aim_along: 0.,
                terminal_aim_shift: 0.,
                terminal_predicted_touchdown_along: 0.,
                terminal_aim_ut: 0.,
                terminal_lift_ratio: 0.,
                terminal_positive_aoa_rate_ema: 0.,
                terminal_sink_accel_ema: 0.,
                terminal_pitch_response_delay_ema: 0.,
                preflare_trigger_altitude: 0.,
                preflare_target_aoa: 0.,
                preflare_target_sink: 0.,
                preflare_minimum_speed: 0.,
                preflare_reference_speed: 0.,
                preflare_predicted_height_loss: 0.,
                preflare_predicted_kinetic_margin: 0.,
                preflare_effective_accel: 0.,
                preflare_response_time: 0.,
                preflare_plan_source_ut: 0.,
                attitude_recovery: false,
                control_bad_duration: 0.,
                control_good_duration: 0.,
                recovery_duration: 0.,
                recovery_heading: 0.,
                recovery_aoa: 0.,
                recovery_pitch_trim: 0.,
                previous_relative_roll_rate: 0.,
                roll_oscillation_score: 0.,
                roll_rate_excess_duration: 0.,
                has_previous_relative_roll_rate: false,
                previous_sideslip: 0.,
                entry_coordination_beta_rate: 0.,
                has_previous_sideslip: false,
                entry_predictive_score: 0.,
                entry_reference_speed: 0.,
                entry_reference_altitude: 0.,
                entry_reference_range: 0.,
                has_entry_predictive_score: false,
                entry_control_plan_valid: false,
                entry_control_terminal_ready: false,
                control_plan_sequence: 0,
                entry_control_plan_ut: 0.,
                entry_control_bank: 0.,
                entry_control_aoa: 0.,
                entry_control_heading: 0.,
                entry_control_turn_radius: 0.,
                entry_control_segment_until_ut: 0.,
                entry_control_cost: 0.,
                entry_control_taem_range_error: 0.,
                entry_control_taem_speed: 0.,
                entry_control_taem_energy_error: 0.,
                entry_reversal_scheduled: false,
                entry_reversal_is_final: false,
                entry_final_reversal_pending: false,
                entry_final_reversal_completed: false,
                entry_continuation_bootstrap: false,
                entry_target_side_latched: false,
                entry_target_side: 0.,
                entry_reversal_ut: 0.,
                entry_reversal_range: 0.,
                entry_reversal_sign: 0.,
                entry_reversal_bank: 0.,
                entry_control_reversals: 0,
                entry_feedback: EntryGuidanceFeedback {
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
                entry_lateral_bank_magnitude: 0.,
                entry_geometry_bank: 0.,
                entry_vertical_bank_magnitude: 0.,
                entry_demand_bank: 0.,
                entry_lateral_required_bank: 0.,
                abort_reason: [0; 384],
                entry_energy_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                taem_altitude_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                final_altitude_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                speed_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                flare_sink_pid: RobustPID {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    integral_limit: 0.,
                    derivative_tc: 0.,
                    integral: 0.,
                    previous_error: 0.,
                    filtered_derivative: 0.,
                    has_previous: false,
                },
                speedbrake_controller: SpeedbrakeController {
                    integral: 0.,
                    deployed: false,
                    initialized: false,
                },
                pitch_limiter: JerkLimiter {
                    value: 0.,
                    rate: 0.,
                    has_value: false,
                },
                roll_limiter: JerkLimiter {
                    value: 0.,
                    rate: 0.,
                    has_value: false,
                },
                heading_limiter: JerkLimiter {
                    value: 0.,
                    rate: 0.,
                    has_value: false,
                },
                throttle_limiter: SlewLimiter {
                    value: 0.,
                    has_value: false,
                },
                last_stabilized_phase: PHASE_IDLE,
                has_last_stabilized_phase: false,
                taem_energy_law: TaemEnergyLawState {
                    initialized: false,
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
                },
                taem_hac_law: TaemHacLawState {
                    initialized: false,
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
                },
                taem_hac_live_aero: TaemHacLiveAero {
                    drag: [0.; 15],
                    lift: [0.; 15],
                    weight: [0.; 15],
                    last_ut: 0.,
                },
                adaptive_flare: AdaptiveFlareState {
                    sxx: [[0.; 3]; 3],
                    sxy: [0.; 3],
                    last_aoa: 0.,
                    last_gamma: 0.,
                    last_ut: 0.,
                    has_last: false,
                    tau: 0.,
                    bias: 0.,
                    gain: 0.,
                    glide_sink: 0.,
                    aim_shift: 0.,
                    weight: 0.,
                    plan_valid: false,
                    active: false,
                    arc_accel: 0.,
                    sgs_height: 0.,
                    cost: 0.,
                    pred_speed: 0.,
                    pred_sink: 0.,
                    pred_along: 0.,
                    pred_peak_n: 0.,
                    pred_climb: 0.,
                    plan_ut: 0.,
                },
                jet_assist_throttle: 0.,
        mm304: crate::mm304::Mm304State::ZERO,
            },
            hac_radius_m: 0.,
            final_approach_distance_m: 0.,
            search_both_ends: false,
            upstream_end: 0,
            restrict_side: false,
            side: 0.,
            lift_scale: 0.,
            drag_scale: 0.,
            scale_mach: 0.,
            seed_route_valid: false,
            seed_route: TaemRoute {
                valid: false,
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
            },
            seed_runway_end: 0,
        };
        memset(
            &raw mut r as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<Mm305PlanRequest>() as size_t,
        );
        return r;
    }
    let mut r_0: Mm305PlanRequest = mm305_request_from_state(g, t, state, cfg, model);
    r_0.restrict_side = false_0 != 0;
    r_0.drag_scale = 1.0f64;
    r_0.lift_scale = r_0.drag_scale;
    if r_0.valid as ::core::ffi::c_int != 0
        && (*t).physics_sample_valid as ::core::ffi::c_int != 0 && (*t).mass > 0.0f64
        && (*t).dynamic_pressure > 0.0f64
    {
        let mut lift_q: ::core::ffi::c_double = 0.0f64;
        let mut drag_q: ::core::ffi::c_double = 0.0f64;
        aero_force_per_q(
            &raw const (*model).aero,
            (*t).dynamic_pressure,
            (*t).mach,
            hypot((*t).angle_of_attack, (*t).sideslip),
            &raw mut lift_q,
            &raw mut drag_q,
        );
        if lift_q > DBL_MIN && (*t).lift_force.is_finite() as i32 != 0
            && (*t).lift_force >= 0.0f64
        {
            r_0.lift_scale = (*t).lift_force / ((*t).dynamic_pressure * lift_q);
        }
        if drag_q > DBL_MIN && (*t).drag_force.is_finite() as i32 != 0
            && (*t).drag_force >= 0.0f64
        {
            r_0.drag_scale = (*t).drag_force / ((*t).dynamic_pressure * drag_q);
        }
        r_0.scale_mach = (*t).mach;
    }
    return r_0;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_admission_accept(
    mut g: *mut GuidanceMachine,
    mut result: *const Mm305PlanResult,
) {
    if g.is_null() || result.is_null() || !(*result).valid {
        return;
    }
    (*g).mm305_admission_needed = false_0 != 0;
    (*g).mm305_admission_valid = true_0 != 0;
    (*g).mm305_admission_found = (*result).found as ::core::ffi::c_int != 0
        && mm305_candidate_ok(&raw const (*result).candidate) as ::core::ffi::c_int != 0
        && (*result).candidate.replay.final_interface_qualified as ::core::ffi::c_int
            != 0;
    (*g).mm305_admission_result_ut = (*result).request_ut;
    (*g).mm305_admission_snapshot_id = (*result).model_snapshot_id;
    (*g).mm305_admission_solve_wall_s = (*result).solve_wall_s;
    (*g).mm305_admission_lift_scale = (*result).lift_scale;
    (*g).mm305_admission_drag_scale = (*result).drag_scale;
    (*g).mm305_admission_scale_mach = (*result).scale_mach;
    if (*g).mm305_admission_found {
        (*g).mm305_admission_route = (*result).candidate.route;
        (*g).mm305_admission_runway_end = (*result).candidate.runway_end;
        (*g).mm305_admission_side = (*result).candidate.side;
    } else {
        (*g).mm305_admission_route = TaemRoute {
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
        (*g).mm305_admission_rejections = (*g)
            .mm305_admission_rejections
            .wrapping_add(1);
        fprintf(
            stderr,
            b"MM305_ADMISSION rejected: %s\n\0" as *const u8
                as *const ::core::ffi::c_char,
            &raw const (*result).diagnostic as *const ::core::ffi::c_char,
        );
    };
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_admission_qualified(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut physically_admissible: bool,
    mut model_snapshot_id: uint64_t,
) -> bool {
    if g.is_null() || t.is_null() || (*t).ut.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    if !physically_admissible {
        (*g).mm305_admission_needed = false_0 != 0;
        return false_0 != 0;
    }
    let mut age: ::core::ffi::c_double = (*t).ut - (*g).mm305_admission_result_ut;
    if (*g).mm305_admission_valid as ::core::ffi::c_int != 0
        && (*g).mm305_admission_found as ::core::ffi::c_int != 0
        && (*g).mm305_admission_snapshot_id == model_snapshot_id && age >= -1e-6f64
        && age <= MM305_ADMISSION_MAX_AGE_S
    {
        return true_0 != 0;
    }
    let mut rejected_recently: bool = (*g).mm305_admission_valid as ::core::ffi::c_int
        != 0 && !(*g).mm305_admission_found
        && (*g).mm305_admission_last_attempt_ut.is_finite() as i32 != 0
        && (*t).ut - (*g).mm305_admission_last_attempt_ut < MM305_ADMISSION_RETRY_S;
    if !(*g).mm305_admission_needed && !rejected_recently {
        (*g).mm305_admission_needed = true_0 != 0;
        (*g).mm305_admission_request_ut = (*t).ut;
        (*g).mm305_admission_last_attempt_ut = (*t).ut;
    }
    return false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_revalidate_admission(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
    mut cfg: *const LandingConfiguration,
    mut model: *const TerminalModel,
) -> bool {
    if g.is_null() || t.is_null() || state.is_null() || cfg.is_null() || model.is_null()
        || !(*g).mm305_admission_valid || !(*g).mm305_admission_found
        || !(*g).mm305_admission_route.valid
        || (*g).mm305_admission_snapshot_id != (*model).snapshot_id
        || (*state).ut != (*t).ut
    {
        return false_0 != 0;
    }
    if (*g).mm305_admission_result_ut == (*t).ut {
        return true_0 != 0;
    }
    let mut request: *mut Mm305PlanRequest = malloc(
        ::core::mem::size_of::<Mm305PlanRequest>() as size_t,
    ) as *mut Mm305PlanRequest;
    let mut previous: *mut Mm305PlanResult = calloc(
        1 as size_t,
        ::core::mem::size_of::<Mm305PlanResult>() as size_t,
    ) as *mut Mm305PlanResult;
    if request.is_null() || previous.is_null() {
        free(request as *mut ::core::ffi::c_void);
        free(previous as *mut ::core::ffi::c_void);
        return false_0 != 0;
    }
    let mut needed: bool = (*g).mm305_admission_needed;
    (*g).mm305_admission_needed = true_0 != 0;
    *request = guidance_mm305_admission_request(g, t, state, cfg, model);
    (*g).mm305_admission_needed = needed;
    (*request).final_approach_distance_m = -(*g).mm305_admission_route.alignment_along_m;
    (*previous).found = true_0 != 0;
    (*previous).valid = (*previous).found;
    (*previous).model_snapshot_id = (*g).mm305_admission_snapshot_id;
    (*previous).candidate.route = (*g).mm305_admission_route;
    (*previous).candidate.runway_end = (*g).mm305_admission_runway_end;
    let mut current: Mm305PlanResult = mm305_revalidate_plan(model, request, previous);
    free(request as *mut ::core::ffi::c_void);
    free(previous as *mut ::core::ffi::c_void);
    if !current.valid {
        return false_0 != 0;
    }
    (*g).mm305_admission_last_attempt_ut = (*t).ut;
    guidance_mm305_admission_accept(g, &raw mut current);
    return (*g).mm305_admission_found;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_adopt_admission_route(
    mut g: *mut GuidanceMachine,
    mut cfg: *const LandingConfiguration,
) -> bool {
    if g.is_null() || cfg.is_null() || !(*g).mm305_admission_valid
        || !(*g).mm305_admission_found || !(*g).mm305_admission_route.valid
    {
        return false_0 != 0;
    }
    let mut result: Mm305PlanResult = Mm305PlanResult {
        valid: false,
        found: false,
        request_ut: 0.,
        initial_state: TerminalDynamicState {
            position_i_m: Vec3 { x: 0., y: 0., z: 0. },
            velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
            attitude: AttitudeModel {
                aoa_rad: 0.,
                bank_rad: 0.,
                aoa_rate_rad_s: 0.,
                bank_rate_rad_s: 0.,
                requested_aoa_rad: 0.,
                requested_bank_rad: 0.,
                cmd_aoa_rad: 0.,
                cmd_bank_rad: 0.,
                pitch_wn: 0.,
                pitch_zeta: 0.,
                roll_wn: 0.,
                roll_zeta: 0.,
                max_pitch_rate_rad_s: 0.,
                max_roll_rate_rad_s: 0.,
                max_pitch_accel_rad_s2: 0.,
                max_roll_accel_rad_s2: 0.,
                pitch_full_authority_q_pa: 0.,
                roll_full_authority_q_pa: 0.,
                direct: false,
                direct_body_coordinates: false,
                body_rates_initialized: false,
                body_pitch_rate_rad_s: 0.,
                body_roll_rate_rad_s: 0.,
                body_yaw_rate_rad_s: 0.,
                input_pitch: 0.,
                input_roll: 0.,
                input_yaw: 0.,
                sideslip_rad: 0.,
                sideslip_rate_rad_s: 0.,
                direct_min_q_pa: 0.,
                direct_pitch_accel_per_kpa: 0.,
                direct_pitch_accel_max: 0.,
                direct_pitch_stiffness_per_kpa: 0.,
                direct_trim_aoa_rad: 0.,
                direct_pitch_damping_ratio: 0.,
                direct_roll_accel_per_kpa: 0.,
                direct_roll_accel_max: 0.,
                direct_roll_damping_s_inv: 0.,
                direct_yaw_accel_per_kpa: 0.,
                direct_yaw_accel_max: 0.,
                direct_yaw_stiffness_per_kpa: 0.,
            },
            mass_kg: 0.,
            ut_s: 0.,
            trim_aoa_ceiling_rad: 0.,
        },
        model_snapshot_id: 0,
        candidate: TaemFixedHacCandidate {
            status: TAEM_PLAN_INFEASIBLE,
            side: 0.,
            runway_end: 0,
            route_built: false,
            final_tail_qualified: false,
            required_lateral_accel_mps2: 0.,
            available_lateral_accel_mps2: 0.,
            quality_score: 0.,
            route: TaemRoute {
                valid: false,
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
            },
            replay: TerminalSolverResult {
                status: TERMINAL_SOLVER_QUALIFIED,
                path_constraints_ok: false,
                final_interface_qualified: false,
                final_state: TerminalDynamicState {
                    position_i_m: Vec3 { x: 0., y: 0., z: 0. },
                    velocity_i_mps: Vec3 { x: 0., y: 0., z: 0. },
                    attitude: AttitudeModel {
                        aoa_rad: 0.,
                        bank_rad: 0.,
                        aoa_rate_rad_s: 0.,
                        bank_rate_rad_s: 0.,
                        requested_aoa_rad: 0.,
                        requested_bank_rad: 0.,
                        cmd_aoa_rad: 0.,
                        cmd_bank_rad: 0.,
                        pitch_wn: 0.,
                        pitch_zeta: 0.,
                        roll_wn: 0.,
                        roll_zeta: 0.,
                        max_pitch_rate_rad_s: 0.,
                        max_roll_rate_rad_s: 0.,
                        max_pitch_accel_rad_s2: 0.,
                        max_roll_accel_rad_s2: 0.,
                        pitch_full_authority_q_pa: 0.,
                        roll_full_authority_q_pa: 0.,
                        direct: false,
                        direct_body_coordinates: false,
                        body_rates_initialized: false,
                        body_pitch_rate_rad_s: 0.,
                        body_roll_rate_rad_s: 0.,
                        body_yaw_rate_rad_s: 0.,
                        input_pitch: 0.,
                        input_roll: 0.,
                        input_yaw: 0.,
                        sideslip_rad: 0.,
                        sideslip_rate_rad_s: 0.,
                        direct_min_q_pa: 0.,
                        direct_pitch_accel_per_kpa: 0.,
                        direct_pitch_accel_max: 0.,
                        direct_pitch_stiffness_per_kpa: 0.,
                        direct_trim_aoa_rad: 0.,
                        direct_pitch_damping_ratio: 0.,
                        direct_roll_accel_per_kpa: 0.,
                        direct_roll_accel_max: 0.,
                        direct_roll_damping_s_inv: 0.,
                        direct_yaw_accel_per_kpa: 0.,
                        direct_yaw_accel_max: 0.,
                        direct_yaw_stiffness_per_kpa: 0.,
                    },
                    mass_kg: 0.,
                    ut_s: 0.,
                    trim_aoa_ceiling_rad: 0.,
                },
                final_geometry: TaemGeometryState {
                    runway_along_m: 0.,
                    runway_cross_m: 0.,
                    altitude_above_runway_m: 0.,
                    ground_speed_mps: 0.,
                    airspeed_mps: 0.,
                    course_deg: 0.,
                    flight_path_angle_deg: 0.,
                    heading_error_deg: 0.,
                    curvature_right_per_m: 0.,
                },
                elapsed_s: 0.,
                minimum_speed_mps: 0.,
                maximum_dynamic_pressure_pa: 0.,
                maximum_load_g: 0.,
                maximum_cross_track_m: 0.,
                maximum_course_error_deg: 0.,
                final_route_course_error_deg: 0.,
                maximum_altitude_error_m: 0.,
                maximum_lateral_authority_shortfall_mps2: 0.,
                maximum_vertical_authority_shortfall_mps2: 0.,
                turn_burden_integral_s: 0.,
                maximum_turn_authority_fraction: 0.,
                maximum_commanded_aoa_deg: 0.,
                bank_target_reversals: 0,
                failure_cross_track_m: 0.,
                failure_course_error_deg: 0.,
                failure_required_lateral_accel_mps2: 0.,
                failure_available_lateral_accel_mps2: 0.,
                failure_required_vertical_lift_mps2: 0.,
                failure_delivered_vertical_lift_mps2: 0.,
                failure_target_altitude_m: 0.,
                failure_target_flight_path_angle_deg: 0.,
                failure_route_index: 0,
                energy_start_j_kg: 0.,
                energy_end_j_kg: 0.,
                drag_work_j_kg: 0.,
                energy_closure_residual_j_kg: 0.,
                final_route_index: 0,
                final_target_altitude_m: 0.,
                final_target_flight_path_angle_deg: 0.,
                reason: ::core::ptr::null::<::core::ffi::c_char>(),
            },
            reason: ::core::ptr::null::<::core::ffi::c_char>(),
        },
        solve_wall_s: 0.,
        scale_mach: 0.,
        lift_scale: 0.,
        drag_scale: 0.,
        diagnostic: [0; 768],
    };
    memset(
        &raw mut result as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Mm305PlanResult>() as size_t,
    );
    result.found = true_0 != 0;
    result.valid = result.found;
    result.request_ut = (*g).mm305_admission_result_ut;
    result.lift_scale = (*g).mm305_admission_lift_scale;
    result.drag_scale = (*g).mm305_admission_drag_scale;
    result.scale_mach = (*g).mm305_admission_scale_mach;
    result.model_snapshot_id = (*g).mm305_admission_snapshot_id;
    result.solve_wall_s = (*g).mm305_admission_solve_wall_s;
    result.candidate.route = (*g).mm305_admission_route;
    result.candidate.runway_end = (*g).mm305_admission_runway_end;
    result.candidate.side = (*g).mm305_admission_side;
    result.candidate.status = TAEM_PLAN_UNQUALIFIED;
    result.candidate.route_built = true_0 != 0;
    result.candidate.replay.path_constraints_ok = true_0 != 0;
    result.candidate.replay.final_interface_qualified = true_0 != 0;
    let mut adopted: bool = guidance_mm305_accept_plan(g, &raw mut result, cfg);
    if adopted {
        (*g).mm305_last_plan_attempt_ut = (*g).mm305_admission_result_ut;
    }
    return adopted;
}
#[no_mangle]
pub unsafe extern "C" fn guidance_mm305_accept_plan(
    mut g: *mut GuidanceMachine,
    mut result: *const Mm305PlanResult,
    mut cfg: *const LandingConfiguration,
) -> bool {
    if g.is_null() || result.is_null() || !(*result).valid || cfg.is_null() {
        return false_0 != 0;
    }
    if (*g).mm305_best_effort_route_active {
        return false_0 != 0;
    }
    (*g).mm305_planning_needed = false_0 != 0;
    if (*g).phase as ::core::ffi::c_uint
        != PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*g).mm305_hac_exit_reached as ::core::ffi::c_int != 0
        || (*g).hac_completed as ::core::ffi::c_int != 0
        || (*g).final_approach_captured as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    if !(*result).found || !mm305_candidate_ok(&raw const (*result).candidate)
        || !(*result).candidate.route.valid
        || !(*result).candidate.replay.final_interface_qualified
    {
        (*g).mm305_plan_failures = (*g).mm305_plan_failures.wrapping_add(1);
        fprintf(
            stderr,
            b"%s\n\0" as *const u8 as *const ::core::ffi::c_char,
            &raw const (*result).diagnostic as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut candidate: *const TaemFixedHacCandidate = &raw const (*result).candidate;
    let mut final_distance: ::core::ffi::c_double = -(*candidate)
        .route
        .alignment_along_m;
    if !(final_distance >= 500.0f64) || final_distance.is_finite() as i32 == 0 {
        final_distance = (*cfg).guidance.final_approach_distance;
    }
    let mut final_slope: ::core::ffi::c_double = (*candidate)
        .route
        .profile_final_slope_deg;
    if !(final_slope >= 3.0f64 && final_slope <= 45.0f64)
        || final_slope.is_finite() as i32 == 0
    {
        final_slope = (*cfg).guidance.final_glide_slope;
    }
    let mut handoff_model: TerminalModel = TerminalModel {
        schema_version: 0 as uint32_t,
        snapshot_id: 0,
        atmosphere_source_digest: 0,
        aero_source_digest: 0,
        aero_book_source_digest: 0,
        attitude_source_digest: 0,
        replay_validated: false,
        planet_frame_aligned: false,
        world: KerbinWorld {
            radius_m: 0.,
            mu_m3_s2: 0.,
            rotation_rate_rad_s: 0.,
            atmosphere_top_m: 0.,
            rotation_phase_rad_at_ut0: 0.,
            orbital_rate_rad_s: 0.,
            solar_phase_rad_at_ut0: 0.,
            stock_spatial_atmosphere: false,
            atmosphere: AtmosphereTable {
                p: [AtmospherePoint {
                    altitude_m: 0.,
                    sample: AtmosphereSample {
                        density_kg_m3: 0.,
                        pressure_pa: 0.,
                        temperature_k: 0.,
                        speed_of_sound_mps: 0.,
                    },
                }; 2048],
                count: 0,
            },
        },
        aero: AeroTable {
            mach: [0.; 40],
            alpha_deg: [0.; 40],
            cl: [[0.; 40]; 40],
            cd: [[0.; 40]; 40],
            mach_count: 0,
            alpha_count: 0,
            reference_area_m2: 0.,
            book: [AeroBookPoint {
                q_pa: 0.,
                mach: 0.,
                alpha_deg: 0.,
                lift_per_q_m2: 0.,
                drag_per_q_m2: 0.,
                support: 0.,
            }; 2048],
            book_count: 0,
            book_enabled: false,
            lift_scale: 0.,
            drag_scale: 0.,
        },
        attitude: AttitudeModel {
            aoa_rad: 0.,
            bank_rad: 0.,
            aoa_rate_rad_s: 0.,
            bank_rate_rad_s: 0.,
            requested_aoa_rad: 0.,
            requested_bank_rad: 0.,
            cmd_aoa_rad: 0.,
            cmd_bank_rad: 0.,
            pitch_wn: 0.,
            pitch_zeta: 0.,
            roll_wn: 0.,
            roll_zeta: 0.,
            max_pitch_rate_rad_s: 0.,
            max_roll_rate_rad_s: 0.,
            max_pitch_accel_rad_s2: 0.,
            max_roll_accel_rad_s2: 0.,
            pitch_full_authority_q_pa: 0.,
            roll_full_authority_q_pa: 0.,
            direct: false,
            direct_body_coordinates: false,
            body_rates_initialized: false,
            body_pitch_rate_rad_s: 0.,
            body_roll_rate_rad_s: 0.,
            body_yaw_rate_rad_s: 0.,
            input_pitch: 0.,
            input_roll: 0.,
            input_yaw: 0.,
            sideslip_rad: 0.,
            sideslip_rate_rad_s: 0.,
            direct_min_q_pa: 0.,
            direct_pitch_accel_per_kpa: 0.,
            direct_pitch_accel_max: 0.,
            direct_pitch_stiffness_per_kpa: 0.,
            direct_trim_aoa_rad: 0.,
            direct_pitch_damping_ratio: 0.,
            direct_roll_accel_per_kpa: 0.,
            direct_roll_accel_max: 0.,
            direct_roll_damping_s_inv: 0.,
            direct_yaw_accel_per_kpa: 0.,
            direct_yaw_accel_max: 0.,
            direct_yaw_stiffness_per_kpa: 0.,
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
    };
    handoff_model.guidance = (*cfg).guidance;
    handoff_model.vehicle = (*cfg).vehicle;
    let mut final_speed: ::core::ffi::c_double = mm305_final_alignment_speed_for_distance(
        &raw mut handoff_model,
        final_distance,
    );
    let mut replan: bool = (*g).mm305_route_committed;
    (*g).mm305_route = (*candidate).route;
    (*g).mm305_best_effort_route_active = false_0 != 0;
    (*g).mm305_direct_approach_degraded = false_0 != 0;
    (*g).runway_end_index = (*candidate).runway_end;
    (*g).runway_end_preview_valid = true_0 != 0;
    (*g).runway_end_committed = true_0 != 0;
    fprintf(
        stderr,
        b"MM305_ROUTE {\"runwayEnd\":%d,\"side\":%.17g,\"radius\":%.17g,\"sweep\":%.17g,\"leadLength\":%.17g,\"arcLength\":%.17g,\"entry\":[%.17g,%.17g],\"center\":[%.17g,%.17g],\"exit\":[%.17g,%.17g],\"finalDistance\":%.17g,\"finalSlope\":%.3f,\"finalSpeed\":%.1f,\"replan\":%s,\"scaleMach\":%.3f,\"liftScale\":%.4f,\"dragScale\":%.4f,\"solveWall\":%.3f}\n\0"
            as *const u8 as *const ::core::ffi::c_char,
        (*candidate).runway_end,
        (*candidate).side,
        (*candidate).route.hac.radius_m,
        (*candidate).route.hac.arc_sweep_rad,
        (*candidate).route.lead_length_m,
        (*candidate).route.hac.arc_length_m,
        (*candidate).route.hac.entry.x,
        (*candidate).route.hac.entry.y,
        (*candidate).route.hac.center.x,
        (*candidate).route.hac.center.y,
        (*candidate).route.hac.exit.x,
        (*candidate).route.hac.exit.y,
        final_distance,
        final_slope,
        final_speed,
        if replan as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        (*result).scale_mach,
        (*result).lift_scale,
        (*result).drag_scale,
        (*result).solve_wall_s,
    );
    (*g).mm305_route_cursor = 0 as size_t;
    (*g).mm305_route_committed = true_0 != 0;
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
    (*g).mm305_acquisition_route_ut = -::core::f32::INFINITY as ::core::ffi::c_double;
    (*g).mm305_acquisition_sweep_rad = ::core::f32::NAN as ::core::ffi::c_double;
    (*g).mm305_hac_exit_reached = false_0 != 0;
    (*g).mm305_runway_alignment_active = false_0 != 0;
    (*g).mm305_model_snapshot_id = (*result).model_snapshot_id;
    (*g).mm305_last_route_ut = (*result).request_ut;
    if replan {
        (*g).mm305_replans = (*g).mm305_replans.wrapping_add(1);
    }
    (*g).hac_side = (*candidate).side;
    (*g).hac_side_selected = true_0 != 0;
    (*g).hac_radius = (*candidate).route.hac.radius_m;
    (*g).hac_remaining = (*candidate).route.length_m;
    (*g).hac_progress_valid = true_0 != 0;
    (*g).terminal_path_kind = TERMINAL_PATH_HAC;
    (*g).terminal_path_committed = true_0 != 0;
    (*g).terminal_final_handoff_latched = true_0 != 0;
    (*g).terminal_final_handoff_distance = final_distance;
    (*g).terminal_final_handoff_slope_deg = final_slope;
    (*g).terminal_final_handoff_speed_mps = final_speed;
    (*g).hac_completed = false_0 != 0;
    (*g).hac_captured = true_0 != 0;
    return true_0 != 0;
}
unsafe extern "C" fn mm305_observe_force_scale(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut model: *const TerminalModel,
    mut current: *const TerminalDynamicState,
    mut dt: ::core::ffi::c_double,
) {
    if (*t).angle_of_attack.is_finite() as i32 == 0 || (*t).angle_of_attack < 3.0f64 {
        return;
    }
    if !(*t).physics_sample_valid || !((*t).mass > 0.0f64)
        || !((*t).dynamic_pressure > 200.0f64)
    {
        return;
    }
    let mut modeled: AeroForces = aero_compute(
        &raw const (*model).world,
        &raw const (*model).aero,
        (*current).position_i_m,
        (*current).velocity_i_mps,
        (*current).ut_s,
        (*current).mass_kg,
        (*current).attitude.aoa_rad,
        (*current).attitude.bank_rad,
    );
    if !(modeled.lift_n > 1.0f64) || !(modeled.drag_n > 1.0f64) {
        return;
    }
    let mut lift_raw: ::core::ffi::c_double = fabs((*t).lift_force) / modeled.lift_n;
    let mut drag_raw: ::core::ffi::c_double = fabs((*t).drag_force) / modeled.drag_n;
    let mut lift_ratio: ::core::ffi::c_double = lift_raw;
    let mut drag_ratio: ::core::ffi::c_double = drag_raw;
    let mut a: ::core::ffi::c_double = clampd(dt / (8.0f64 + dt), 0.0f64, 1.0f64);
    let mut first_lift: bool = !((*g).mm305_lift_scale > 0.0f64)
        || (*g).mm305_lift_scale.is_finite() as i32 == 0;
    let mut first_drag: bool = !((*g).mm305_drag_scale > 0.0f64)
        || (*g).mm305_drag_scale.is_finite() as i32 == 0;
    if first_lift {
        (*g).mm305_lift_scale = lift_ratio;
    } else {
        (*g).mm305_lift_scale += a * (lift_ratio - (*g).mm305_lift_scale);
    }
    if first_drag {
        (*g).mm305_drag_scale = drag_ratio;
    } else {
        (*g).mm305_drag_scale += a * (drag_ratio - (*g).mm305_drag_scale);
    }
    let mut diag: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !diag.is_null()
        && (first_lift as ::core::ffi::c_int != 0
            || first_drag as ::core::ffi::c_int != 0
            || strcmp(diag, b"aero\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int)
    {
        fprintf(
            stderr,
            b"MM305 aero sample: ut=%.2f mach=%.3f aoa=%.2f raw=%.3f/%.3f clamped=%.3f/%.3f filtered=%.3f/%.3f measured=%.0f/%.0f modeled=%.0f/%.0f q=%.0f sb=%.3f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*current).ut_s,
            modeled.mach,
            (*t).angle_of_attack,
            lift_raw,
            drag_raw,
            lift_ratio,
            drag_ratio,
            (*g).mm305_lift_scale,
            (*g).mm305_drag_scale,
            fabs((*t).lift_force),
            fabs((*t).drag_force),
            modeled.lift_n,
            modeled.drag_n,
            (*t).dynamic_pressure,
            (*g).taem_speedbrake_fraction,
        );
    }
}
unsafe extern "C" fn mm305_acquisition_circle_penalty(
    mut route: *const TaemRoute,
) -> ::core::ffi::c_double {
    if route.is_null() || !(*route).valid {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return log1p(
        fmax(0.0f64, (*route).lead_length_m) / fmax(1.0f64, (*route).hac.arc_length_m),
    );
}
unsafe extern "C" fn mm305_acquisition_profile_class(
    mut profile: *const TerminalProfileResult,
) -> Mm305AcquisitionProfileClass {
    if profile.is_null() {
        return MM305_ACQ_PROFILE_INVALID;
    }
    if (*profile).valid {
        return MM305_ACQ_PROFILE_FEASIBLE;
    }
    let mut reason: *const ::core::ffi::c_char = if !(*profile).reason.is_null() {
        (*profile).reason
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
    if !strstr(
            reason,
            b"too short to descend\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
        || !strstr(
                reason,
                b"too short to dissipate\0" as *const u8 as *const ::core::ffi::c_char,
            )
            .is_null()
        || !strstr(
                reason,
                b"retains excess energy\0" as *const u8 as *const ::core::ffi::c_char,
            )
            .is_null()
    {
        return MM305_ACQ_PROFILE_TOO_SHORT;
    }
    if !strstr(
            reason,
            b"too long for the available energy\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
        || !strstr(
                reason,
                b"too slow for Final handoff\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        || !strstr(
                reason,
                b"propagation time before route completion\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
    {
        return MM305_ACQ_PROFILE_TOO_LONG;
    }
    return MM305_ACQ_PROFILE_INVALID;
}
unsafe extern "C" fn mm305_acquisition_seed_consider(
    mut seeds: *mut Mm305AcquisitionSeed,
    mut capacity: size_t,
    mut radius_m: ::core::ffi::c_double,
    mut sweep_rad: ::core::ffi::c_double,
    mut kind: Mm305AcquisitionRouteKind,
    mut target_lead_m: ::core::ffi::c_double,
    mut cheap_score: ::core::ffi::c_double,
) {
    if seeds.is_null() || capacity == 0 as size_t || radius_m.is_finite() as i32 == 0
        || sweep_rad.is_finite() as i32 == 0 || cheap_score.is_finite() as i32 == 0
    {
        return;
    }
    let mut i: size_t = 0 as size_t;
    while i < capacity {
        if (*seeds.offset(i as isize)).valid {
            if fabs((*seeds.offset(i as isize)).radius_m - radius_m) <= 1.0f64
                && fabs((*seeds.offset(i as isize)).sweep_rad - sweep_rad) <= 1e-9f64
                && (*seeds.offset(i as isize)).kind as ::core::ffi::c_uint
                    == kind as ::core::ffi::c_uint
            {
                if cheap_score < (*seeds.offset(i as isize)).cheap_score {
                    (*seeds.offset(i as isize)).target_lead_m = target_lead_m;
                    (*seeds.offset(i as isize)).cheap_score = cheap_score;
                }
                return;
            }
        }
        i = i.wrapping_add(1);
    }
    let mut slot: size_t = capacity;
    let mut worst: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < capacity {
        if !(*seeds.offset(i_0 as isize)).valid {
            slot = i_0;
            break;
        } else {
            if (*seeds.offset(i_0 as isize)).cheap_score > worst {
                worst = (*seeds.offset(i_0 as isize)).cheap_score;
                slot = i_0;
            }
            i_0 = i_0.wrapping_add(1);
        }
    }
    if slot >= capacity
        || (*seeds.offset(slot as isize)).valid as ::core::ffi::c_int != 0
            && cheap_score >= (*seeds.offset(slot as isize)).cheap_score
    {
        return;
    }
    *seeds.offset(slot as isize) = Mm305AcquisitionSeed {
        valid: true_0 != 0,
        radius_m: radius_m,
        sweep_rad: sweep_rad,
        target_lead_m: target_lead_m,
        cheap_score: cheap_score,
        kind: kind,
    };
}
unsafe extern "C" fn mm305_acquisition_choice_better(
    mut candidate: *const Mm305AcquisitionChoice,
    mut best: *const Mm305AcquisitionChoice,
) -> bool {
    if candidate.is_null() || !(*candidate).valid {
        return false_0 != 0;
    }
    if (*candidate).profile_class as ::core::ffi::c_uint
        == MM305_ACQ_PROFILE_FEASIBLE as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*candidate).replay_quality == 0.0f64
    {
        return false_0 != 0;
    }
    if (*best).profile_class as ::core::ffi::c_uint
        == MM305_ACQ_PROFILE_FEASIBLE as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*best).replay_quality == 0.0f64
    {
        return true_0 != 0;
    }
    if (*candidate).profile_class as ::core::ffi::c_uint
        != (*best).profile_class as ::core::ffi::c_uint
    {
        return ((*candidate).profile_class as ::core::ffi::c_uint)
            < (*best).profile_class as ::core::ffi::c_uint;
    }
    if (*candidate).profile_class as ::core::ffi::c_uint
        == MM305_ACQ_PROFILE_FEASIBLE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        if (*candidate).speed_error_mps < (*best).speed_error_mps - 2.0f64 {
            return true_0 != 0;
        }
        if (*best).speed_error_mps < (*candidate).speed_error_mps - 2.0f64 {
            return false_0 != 0;
        }
        if (*candidate).replay_quality < (*best).replay_quality {
            return true_0 != 0;
        }
        if (*candidate).replay_quality > (*best).replay_quality {
            return false_0 != 0;
        }
    }
    if (*candidate).energy_metric < (*best).energy_metric - 0.05f64 {
        return true_0 != 0;
    }
    if (*best).energy_metric < (*candidate).energy_metric - 0.05f64 {
        return false_0 != 0;
    }
    if (*candidate).circle_penalty < (*best).circle_penalty - 0.02f64 {
        return true_0 != 0;
    }
    if (*best).circle_penalty < (*candidate).circle_penalty - 0.02f64 {
        return false_0 != 0;
    }
    return (*candidate).route.length_m < (*best).route.length_m;
}
unsafe extern "C" fn mm305_make_acquisition_route_variant(
    mut g: *const GuidanceMachine,
    mut model: *const TerminalModel,
    mut current: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
    mut hac_radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut selected_sweep: ::core::ffi::c_double,
    mut kind: Mm305AcquisitionRouteKind,
    mut target_lead_m: ::core::ffi::c_double,
    mut out: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    if model.is_null() || current.is_null() || geometry.is_null() || out.is_null() {
        return false_0 != 0;
    }
    let mut reachability: TaemReachability = TaemReachability {
        valid: false,
        lateral_authority_ok: false,
        energy_qualified: false,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        lateral_margin_mps2: 0.,
        minimum_turn_radius_m: 0.,
        roll_settling_time_s: 0.,
        roll_settling_distance_m: 0.,
    };
    if !taem_fixed_hac_turn_reachability(
        model,
        current,
        geometry,
        hac_radius,
        &raw mut reachability,
    ) || !reachability.valid || !(reachability.available_lateral_accel_mps2 > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut live_curvature: ::core::ffi::c_double = 0.95f64
        * reachability.available_lateral_accel_mps2
        / fmax((*geometry).ground_speed_mps * (*geometry).ground_speed_mps, 1.0f64);
    let mut rho_live: ::core::ffi::c_double = world_atmosphere_sample(
            &raw const (*model).world,
            (*model).site.altitude + (*geometry).altitude_above_runway_m,
        )
        .density_kg_m3;
    let mut sweep_geometry: TaemFixedHacGeometry = TaemFixedHacGeometry {
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
    };
    if !taem_hac_geometry_sweep(
        model,
        hac_radius,
        side,
        selected_sweep,
        &raw mut sweep_geometry,
    ) {
        return false_0 != 0;
    }
    let mut terminal_height: ::core::ffi::c_double = sweep_geometry.final_length_m
        * tan((*model).guidance.final_glide_slope * DEG2RAD);
    let mut hac_height: ::core::ffi::c_double = terminal_height
        + sweep_geometry.arc_length_m
            * tan(fmin((*model).guidance.taem_glide_slope, 12.0f64) * DEG2RAD);
    let mut rho_join: ::core::ffi::c_double = world_atmosphere_sample(
            &raw const (*model).world,
            (*model).site.altitude + fmax(0.0f64, hac_height),
        )
        .density_kg_m3;
    let mut density_gain: ::core::ffi::c_double = if rho_live > 0.0f64
        && rho_join > rho_live
    {
        rho_join / rho_live
    } else {
        1.0f64
    };
    let mut peak_curvature_limit: ::core::ffi::c_double = fmax(
        live_curvature,
        live_curvature * density_gain,
    );
    let mut spacing: ::core::ffi::c_double = clampd(
        0.5f64 * (*geometry).ground_speed_mps,
        250.0f64,
        700.0f64,
    );
    let mut route: TaemRoute = TaemRoute {
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
    let mut built: bool = false_0 != 0;
    if kind as ::core::ffi::c_uint
        == MM305_ACQ_ROUTE_NORMAL as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        built = taem_route_build_hac(
            model,
            geometry,
            hac_radius,
            side,
            selected_sweep,
            spacing,
            live_curvature,
            &raw mut route,
            reason,
            reason_size,
        );
    } else if kind as ::core::ffi::c_uint
        == MM305_ACQ_ROUTE_SHORTEST as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        built = taem_route_build_hac_shortest(
            model,
            geometry,
            hac_radius,
            side,
            selected_sweep,
            spacing,
            live_curvature,
            peak_curvature_limit,
            &raw mut route,
            reason,
            reason_size,
        );
    } else if kind as ::core::ffi::c_uint
        == MM305_ACQ_ROUTE_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        && target_lead_m.is_finite() as i32 != 0 && target_lead_m > 0.0f64
    {
        built = taem_route_build_hac_length(
            model,
            geometry,
            hac_radius,
            side,
            selected_sweep,
            spacing,
            live_curvature,
            peak_curvature_limit,
            target_lead_m,
            &raw mut route,
            reason,
            reason_size,
        );
    }
    if !built {
        return false_0 != 0;
    }
    if !taem_route_limit_initial_vertical_authority(
        model,
        current,
        geometry,
        &raw mut route,
    ) {
        return false_0 != 0;
    }
    *out = route;
    return true_0 != 0;
}
unsafe extern "C" fn mm305_make_acquisition_route(
    mut g: *const GuidanceMachine,
    mut model: *const TerminalModel,
    mut current: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
    mut hac_radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut selected_sweep: ::core::ffi::c_double,
    mut out: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    if mm305_make_acquisition_route_variant(
        g,
        model,
        current,
        geometry,
        hac_radius,
        side,
        selected_sweep,
        MM305_ACQ_ROUTE_SHORTEST,
        ::core::f32::NAN as ::core::ffi::c_double,
        out,
        reason,
        reason_size,
    ) {
        return true_0 != 0;
    }
    return mm305_make_acquisition_route_variant(
        g,
        model,
        current,
        geometry,
        hac_radius,
        side,
        selected_sweep,
        MM305_ACQ_ROUTE_NORMAL,
        ::core::f32::NAN as ::core::ffi::c_double,
        out,
        reason,
        reason_size,
    );
}
unsafe extern "C" fn mm305_install_acquisition_route(
    mut g: *mut GuidanceMachine,
    mut model: *const TerminalModel,
    mut current: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
    mut hac_radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut selected_sweep: ::core::ffi::c_double,
    mut prebuilt: *const TaemRoute,
) -> bool {
    if g.is_null() {
        return false_0 != 0;
    }
    let mut route: TaemRoute = TaemRoute {
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
    let mut reason: [::core::ffi::c_char; 160] = [
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
    ];
    if !prebuilt.is_null() && (*prebuilt).valid as ::core::ffi::c_int != 0 {
        route = *prebuilt;
    } else if !mm305_make_acquisition_route(
        g,
        model,
        current,
        geometry,
        hac_radius,
        side,
        selected_sweep,
        &raw mut route,
        &raw mut reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
    ) {
        return false_0 != 0
    }
    (*g).mm305_acquisition_route = route;
    (*g).mm305_acquisition_route_cursor = 0 as size_t;
    (*g).mm305_acquisition_route_valid = true_0 != 0;
    (*g).mm305_acquisition_route_ut = (*current).ut_s;
    (*g).mm305_acquisition_sweep_rad = selected_sweep;
    return true_0 != 0;
}
unsafe extern "C" fn mm305_route_to_reference_trajectory(
    mut out: *mut Trajectory,
    mut route: *const TaemRoute,
    mut model: *const TerminalModel,
    mut telemetry: *const Telemetry,
) {
    if out.is_null() || route.is_null() || !(*route).valid
        || (*route).count < 2 as size_t || model.is_null() || telemetry.is_null()
    {
        return;
    }
    trajectory_clear(out);
    let mut origin: GeoPoint = GeoPoint {
        latitude: (*model).site.latitude,
        longitude: (*model).site.longitude,
        altitude: (*model).site.altitude,
    };
    let mut heading: ::core::ffi::c_double = (*model).site.runway_heading * DEG2RAD;
    let mut sh: ::core::ffi::c_double = sin(heading);
    let mut ch: ::core::ffi::c_double = cos(heading);
    let mut max_points: size_t = 120 as size_t;
    let mut step: size_t = if (*route).count <= max_points {
        1 as size_t
    } else {
        (*route)
            .count
            .wrapping_sub(1 as size_t)
            .wrapping_div(max_points.wrapping_sub(1 as size_t))
            .wrapping_add(
                (if (*route)
                    .count
                    .wrapping_sub(1 as size_t)
                    .wrapping_rem(max_points.wrapping_sub(1 as size_t)) != 0
                {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                }) as size_t,
            )
    };
    let mut end_speed: ::core::ffi::c_double = fmax(
        (*model).guidance.final_alignment_speed,
        (*model).vehicle.minimum_safe_speed,
    );
    let mut index: size_t = 0 as size_t;
    while index < (*route).count {
        let mut rp: TaemRoutePoint = TaemRoutePoint {
            along_m: 0.,
            cross_m: 0.,
            course_deg: 0.,
            curvature_right_per_m: 0.,
            altitude_m: 0.,
            flight_path_angle_deg: 0.,
            distance_from_start_m: 0.,
        };
        if taem_route_point_at_index(route, index, &raw mut rp) {
            let mut east: ::core::ffi::c_double = rp.along_m * sh + rp.cross_m * ch;
            let mut north: ::core::ffi::c_double = rp.along_m * ch - rp.cross_m * sh;
            let mut altitude: ::core::ffi::c_double = if rp.altitude_m.is_finite() as i32
                != 0
            {
                rp.altitude_m
            } else {
                (*model).site.altitude + (*telemetry).radar_altitude
            };
            let mut p: GeoPoint = local_point(
                origin,
                east,
                north,
                (*model).world.radius_m,
                altitude,
            );
            let mut f: ::core::ffi::c_double = if (*route).length_m > 0.0f64 {
                clampd(rp.distance_from_start_m / (*route).length_m, 0.0f64, 1.0f64)
            } else {
                0.0f64
            };
            let mut speed: ::core::ffi::c_double = (*telemetry).true_air_speed
                + f * (end_speed - (*telemetry).true_air_speed);
            let mut average_speed: ::core::ffi::c_double = fmax(
                1.0f64,
                0.5f64 * ((*telemetry).true_air_speed + speed),
            );
            trajectory_append(
                out,
                TrajectoryPoint {
                    ut: (*telemetry).ut + rp.distance_from_start_m / average_speed,
                    latitude: p.latitude,
                    longitude: p.longitude,
                    altitude: p.altitude,
                    speed: speed,
                    phase: PHASE_TAEM,
                    kind: TRAJ_REFERENCE,
                },
            );
        }
        index = index.wrapping_add(step);
    }
    if (*route).count > 1 as size_t
        && (*route).count.wrapping_sub(1 as size_t).wrapping_rem(step) != 0 as size_t
    {
        let mut rp_0: TaemRoutePoint = TaemRoutePoint {
            along_m: 0.,
            cross_m: 0.,
            course_deg: 0.,
            curvature_right_per_m: 0.,
            altitude_m: 0.,
            flight_path_angle_deg: 0.,
            distance_from_start_m: 0.,
        };
        if taem_route_point_at_index(
            route,
            (*route).count.wrapping_sub(1 as size_t),
            &raw mut rp_0,
        ) {
            let mut east_0: ::core::ffi::c_double = rp_0.along_m * sh
                + rp_0.cross_m * ch;
            let mut north_0: ::core::ffi::c_double = rp_0.along_m * ch
                - rp_0.cross_m * sh;
            let mut altitude_0: ::core::ffi::c_double = if rp_0.altitude_m.is_finite()
                as i32 != 0
            {
                rp_0.altitude_m
            } else {
                (*model).site.altitude + (*telemetry).radar_altitude
            };
            let mut p_0: GeoPoint = local_point(
                origin,
                east_0,
                north_0,
                (*model).world.radius_m,
                altitude_0,
            );
            trajectory_append(
                out,
                TrajectoryPoint {
                    ut: (*telemetry).ut
                        + (*route).length_m
                            / fmax(
                                1.0f64,
                                0.5f64 * ((*telemetry).true_air_speed + end_speed),
                            ),
                    latitude: p_0.latitude,
                    longitude: p_0.longitude,
                    altitude: p_0.altitude,
                    speed: end_speed,
                    phase: PHASE_TAEM,
                    kind: TRAJ_REFERENCE,
                },
            );
        }
    }
}
unsafe extern "C" fn mm305_acquisition_command(
    mut g: *mut GuidanceMachine,
    mut base_model: *const TerminalModel,
    mut model: *const TerminalModel,
    mut current: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
    mut hac_radius: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut demand: *mut TaemTrackerOutput,
    mut reference: *mut TaemPathReference,
    mut path_remaining_out: *mut ::core::ffi::c_double,
    mut required_drag_out: *mut ::core::ffi::c_double,
    mut available_drag_out: *mut ::core::ffi::c_double,
) -> bool {
    if g.is_null() || model.is_null() || current.is_null() || geometry.is_null() {
        return false_0 != 0;
    }
    let mut final_base: *const TerminalModel = if !base_model.is_null() {
        base_model
    } else {
        model
    };
    let mut final_distance: ::core::ffi::c_double = mm305_final_alignment_distance(
        g,
        final_base,
        current,
    );
    let mut acquisition_model: TerminalModel = *model;
    mm305_apply_final_alignment(&raw mut acquisition_model, final_base, final_distance);
    if (*current).trim_aoa_ceiling_rad > 0.0f64 {
        acquisition_model.vehicle.maximum_angle_of_attack = fmin(
            acquisition_model.vehicle.maximum_angle_of_attack,
            (*current).trim_aoa_ceiling_rad * RAD2DEG,
        );
    }
    let mut planning_model: *const TerminalModel = &raw mut acquisition_model;
    let mut side: ::core::ffi::c_double = if (*g).mm305_acquisition_route_valid
        as ::core::ffi::c_int != 0
    {
        (*g).mm305_acquisition_route.side
    } else if (*geometry).runway_cross_m >= 0.0f64 {
        1.0f64
    } else {
        -1.0f64
    };
    let mut alignment_height: ::core::ffi::c_double = (*planning_model)
        .guidance
        .final_approach_distance
        * tan((*planning_model).guidance.final_glide_slope * DEG2RAD);
    let mut alignment_speed: ::core::ffi::c_double = fmax(
        (*planning_model).guidance.final_alignment_speed,
        (*planning_model).vehicle.minimum_safe_speed,
    );
    let mut radius: ::core::ffi::c_double = (*planning_model).world.radius_m
        + (*planning_model).site.altitude
        + fmax(0.0f64, (*geometry).altitude_above_runway_m);
    let mut gravity: ::core::ffi::c_double = (*planning_model).world.mu_m3_s2
        / fmax(radius * radius, 1.0f64);
    let mut energy_excess: ::core::ffi::c_double = 0.5f64
        * ((*geometry).airspeed_mps * (*geometry).airspeed_mps
            - alignment_speed * alignment_speed)
        + gravity * ((*geometry).altitude_above_runway_m - alignment_height);
    energy_excess = fmax(0.0f64, energy_excess);
    let mut modeled: AeroForces = aero_compute(
        &raw const (*planning_model).world,
        &raw const (*planning_model).aero,
        (*current).position_i_m,
        (*current).velocity_i_mps,
        (*current).ut_s,
        (*current).mass_kg,
        (*current).attitude.aoa_rad,
        (*current).attitude.bank_rad,
    );
    let mut available_drag: ::core::ffi::c_double = if (*current).mass_kg > 1.0f64
        && modeled.drag_n > 0.0f64
    {
        modeled.drag_n / (*current).mass_kg
    } else {
        0.0f64
    };
    let mut planning_drag: ::core::ffi::c_double = fmax(0.75f64, available_drag);
    let mut required_path: ::core::ffi::c_double = energy_excess / planning_drag;
    let mut joined_fixed_hac: bool = (*g).mm305_acquisition_route_valid
        as ::core::ffi::c_int != 0
        && (*g).mm305_acquisition_route.valid as ::core::ffi::c_int != 0
        && (*g).mm305_acquisition_route_cursor >= (*g).mm305_acquisition_route.lead_count
        && (*g).mm305_acquisition_route_cursor < (*g).mm305_acquisition_route.count;
    let mut refresh: bool = !(*g).mm305_acquisition_route_valid
        || !(*g).mm305_acquisition_route.valid
        || !joined_fixed_hac
            && (*current).ut_s - (*g).mm305_acquisition_route_ut >= 4.0f64;
    if refresh {
        let mut global_seeds: [Mm305AcquisitionSeed; 4] = [
            Mm305AcquisitionSeed {
                valid: 0 as ::core::ffi::c_int != 0,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
        ];
        let mut radius_seeds: [[Mm305AcquisitionSeed; 1]; 7] = [
            [
                Mm305AcquisitionSeed {
                    valid: 0 as ::core::ffi::c_int != 0,
                    radius_m: 0.,
                    sweep_rad: 0.,
                    target_lead_m: 0.,
                    cheap_score: 0.,
                    kind: MM305_ACQ_ROUTE_NORMAL,
                },
            ],
            [Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            }; 1],
            [Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            }; 1],
            [Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            }; 1],
            [Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            }; 1],
            [Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            }; 1],
            [Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            }; 1],
        ];
        let mut radius_shortest: [Mm305AcquisitionSeed; 7] = [
            Mm305AcquisitionSeed {
                valid: 0 as ::core::ffi::c_int != 0,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
        ];
        let mut native_seeds: [Mm305AcquisitionSeed; 18] = [
            Mm305AcquisitionSeed {
                valid: 0 as ::core::ffi::c_int != 0,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
            Mm305AcquisitionSeed {
                valid: false,
                radius_m: 0.,
                sweep_rad: 0.,
                target_lead_m: 0.,
                cheap_score: 0.,
                kind: MM305_ACQ_ROUTE_NORMAL,
            },
        ];
        let mut selected: Mm305AcquisitionChoice = Mm305AcquisitionChoice {
            valid: 0 as ::core::ffi::c_int != 0,
            route: TaemRoute {
                valid: false,
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
            },
            sweep_rad: 0.,
            profile_class: MM305_ACQ_PROFILE_FEASIBLE,
            speed_error_mps: 0.,
            energy_metric: 0.,
            circle_penalty: 0.,
            profile_reason: ::core::ptr::null::<::core::ffi::c_char>(),
            replay_quality: 0.,
        };
        let radius_floor: ::core::ffi::c_double = 3000.0f64;
        let radius_factors: [::core::ffi::c_double; 7] = [
            1.50f64,
            1.00f64,
            0.75f64,
            0.60f64,
            0.50f64,
            0.36f64,
            0.25f64,
        ];
        let cheap_scale: ::core::ffi::c_double = fmax(required_path, 5000.0f64)
            as ::core::ffi::c_double;
        let mut previous_radius: ::core::ffi::c_double = ::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut ri: size_t = 0 as size_t;
        while ri < MM305_ACQ_RADIUS_COUNT as ::core::ffi::c_int as size_t {
            let mut candidate_radius: ::core::ffi::c_double = fmax(
                radius_floor,
                hac_radius * radius_factors[ri as usize],
            );
            if !(fabs(candidate_radius - previous_radius) <= 1.0f64) {
                previous_radius = candidate_radius;
                let mut sweep_deg: ::core::ffi::c_double = 15.0f64;
                while sweep_deg <= 270.0f64 + 1e-6f64 {
                    let mut sweep: ::core::ffi::c_double = sweep_deg * DEG2RAD;
                    let mut normal: TaemRoute = TaemRoute {
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
                    let mut reason: [::core::ffi::c_char; 160] = [
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
                    ];
                    let mut have_normal: bool = mm305_make_acquisition_route_variant(
                        g,
                        planning_model,
                        current,
                        geometry,
                        candidate_radius,
                        side,
                        sweep,
                        MM305_ACQ_ROUTE_NORMAL,
                        ::core::f32::NAN as ::core::ffi::c_double,
                        &raw mut normal,
                        &raw mut reason as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
                    );
                    let mut energy_lead: ::core::ffi::c_double = ::core::f32::NAN
                        as ::core::ffi::c_double;
                    if have_normal {
                        let mut nonlead: ::core::ffi::c_double = normal.length_m
                            - normal.lead_length_m;
                        energy_lead = fmax(100.0f64, required_path - nonlead);
                    }
                    let mut current_block_13: u64;
                    let mut variant: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                    while variant < 3 as ::core::ffi::c_int {
                        let mut kind: Mm305AcquisitionRouteKind = variant
                            as Mm305AcquisitionRouteKind;
                        let mut target_lead: ::core::ffi::c_double = if kind
                            as ::core::ffi::c_uint
                            == MM305_ACQ_ROUTE_ENERGY as ::core::ffi::c_int
                                as ::core::ffi::c_uint
                        {
                            energy_lead
                        } else {
                            ::core::f32::NAN as ::core::ffi::c_double
                        };
                        if !(kind as ::core::ffi::c_uint
                            == MM305_ACQ_ROUTE_ENERGY as ::core::ffi::c_int
                                as ::core::ffi::c_uint
                            && target_lead.is_finite() as i32 == 0)
                        {
                            let mut trial: TaemRoute = TaemRoute {
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
                            if kind as ::core::ffi::c_uint
                                == MM305_ACQ_ROUTE_NORMAL as ::core::ffi::c_int
                                    as ::core::ffi::c_uint
                                && have_normal as ::core::ffi::c_int != 0
                            {
                                trial = normal;
                                current_block_13 = 17281240262373992796;
                            } else if !mm305_make_acquisition_route_variant(
                                g,
                                planning_model,
                                current,
                                geometry,
                                candidate_radius,
                                side,
                                sweep,
                                kind,
                                target_lead,
                                &raw mut trial,
                                &raw mut reason as *mut ::core::ffi::c_char,
                                ::core::mem::size_of::<[::core::ffi::c_char; 160]>()
                                    as size_t,
                            ) {
                                current_block_13 = 18386322304582297246;
                            } else {
                                current_block_13 = 17281240262373992796;
                            }
                            match current_block_13 {
                                18386322304582297246 => {}
                                _ => {
                                    let mut circle: ::core::ffi::c_double = mm305_acquisition_circle_penalty(
                                        &raw mut trial,
                                    );
                                    let mut length_error: ::core::ffi::c_double = fabs(
                                        trial.length_m - required_path,
                                    ) / cheap_scale;
                                    let mut cheap_score: ::core::ffi::c_double = length_error
                                        + 0.03f64 * circle;
                                    mm305_acquisition_seed_consider(
                                        &raw mut global_seeds as *mut Mm305AcquisitionSeed,
                                        MM305_ACQ_GLOBAL_SEEDS as ::core::ffi::c_int as size_t,
                                        candidate_radius,
                                        sweep,
                                        kind,
                                        target_lead,
                                        cheap_score,
                                    );
                                    mm305_acquisition_seed_consider(
                                        &raw mut *(&raw mut radius_seeds
                                            as *mut [Mm305AcquisitionSeed; 1])
                                            .offset(ri as isize) as *mut Mm305AcquisitionSeed,
                                        MM305_ACQ_RADIUS_SEEDS as ::core::ffi::c_int as size_t,
                                        candidate_radius,
                                        sweep,
                                        kind,
                                        target_lead,
                                        cheap_score,
                                    );
                                    mm305_acquisition_seed_consider(
                                        (&raw mut radius_shortest as *mut Mm305AcquisitionSeed)
                                            .offset(ri as isize) as *mut Mm305AcquisitionSeed,
                                        MM305_ACQ_RADIUS_SHORTEST_SEEDS as ::core::ffi::c_int
                                            as size_t,
                                        candidate_radius,
                                        sweep,
                                        kind,
                                        target_lead,
                                        trial.length_m / cheap_scale,
                                    );
                                }
                            }
                        }
                        variant += 1;
                    }
                    sweep_deg += 15.0f64;
                }
            }
            ri = ri.wrapping_add(1);
        }
        let mut i: size_t = 0 as size_t;
        while i < MM305_ACQ_GLOBAL_SEEDS as ::core::ffi::c_int as size_t {
            if global_seeds[i as usize].valid {
                mm305_acquisition_seed_consider(
                    &raw mut native_seeds as *mut Mm305AcquisitionSeed,
                    MM305_ACQ_NATIVE_SEEDS as ::core::ffi::c_int as size_t,
                    global_seeds[i as usize].radius_m,
                    global_seeds[i as usize].sweep_rad,
                    global_seeds[i as usize].kind,
                    global_seeds[i as usize].target_lead_m,
                    global_seeds[i as usize].cheap_score,
                );
            }
            i = i.wrapping_add(1);
        }
        let mut ri_0: size_t = 0 as size_t;
        while ri_0 < MM305_ACQ_RADIUS_COUNT as ::core::ffi::c_int as size_t {
            let mut i_0: size_t = 0 as size_t;
            while i_0 < MM305_ACQ_RADIUS_SEEDS as ::core::ffi::c_int as size_t {
                if radius_seeds[ri_0 as usize][i_0 as usize].valid {
                    mm305_acquisition_seed_consider(
                        &raw mut native_seeds as *mut Mm305AcquisitionSeed,
                        MM305_ACQ_NATIVE_SEEDS as ::core::ffi::c_int as size_t,
                        radius_seeds[ri_0 as usize][i_0 as usize].radius_m,
                        radius_seeds[ri_0 as usize][i_0 as usize].sweep_rad,
                        radius_seeds[ri_0 as usize][i_0 as usize].kind,
                        radius_seeds[ri_0 as usize][i_0 as usize].target_lead_m,
                        radius_seeds[ri_0 as usize][i_0 as usize].cheap_score,
                    );
                }
                i_0 = i_0.wrapping_add(1);
            }
            if radius_shortest[ri_0 as usize].valid {
                mm305_acquisition_seed_consider(
                    &raw mut native_seeds as *mut Mm305AcquisitionSeed,
                    MM305_ACQ_NATIVE_SEEDS as ::core::ffi::c_int as size_t,
                    radius_shortest[ri_0 as usize].radius_m,
                    radius_shortest[ri_0 as usize].sweep_rad,
                    radius_shortest[ri_0 as usize].kind,
                    radius_shortest[ri_0 as usize].target_lead_m,
                    radius_shortest[ri_0 as usize].cheap_score,
                );
            }
            ri_0 = ri_0.wrapping_add(1);
        }
        let mut i_1: size_t = 0 as size_t;
        while i_1 < MM305_ACQ_NATIVE_SEEDS as ::core::ffi::c_int as size_t {
            let mut seed: *const Mm305AcquisitionSeed = (&raw mut native_seeds
                as *mut Mm305AcquisitionSeed)
                .offset(i_1 as isize) as *mut Mm305AcquisitionSeed;
            if (*seed).valid {
                let mut trial_0: TaemRoute = TaemRoute {
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
                let mut reason_0: [::core::ffi::c_char; 160] = [
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
                ];
                if mm305_make_acquisition_route_variant(
                    g,
                    planning_model,
                    current,
                    geometry,
                    (*seed).radius_m,
                    side,
                    (*seed).sweep_rad,
                    (*seed).kind,
                    (*seed).target_lead_m,
                    &raw mut trial_0,
                    &raw mut reason_0 as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
                ) {
                    let mut profile_class: Mm305AcquisitionProfileClass = MM305_ACQ_PROFILE_INVALID;
                    let mut predicted_speed: ::core::ffi::c_double = ::core::f32::NAN
                        as ::core::ffi::c_double;
                    let mut replay_quality: ::core::ffi::c_double = ::core::f32::NAN
                        as ::core::ffi::c_double;
                    let mut classification_reason: *const ::core::ffi::c_char = b"provisional geometry only; background replay pending\0"
                        as *const u8 as *const ::core::ffi::c_char;
                    if (*g).diagnostic_shadow {
                        let mut profile_route: TaemRoute = trial_0;
                        let mut profile: TerminalProfileResult = terminal_solver_generate_profile(
                            planning_model,
                            current,
                            &raw mut profile_route,
                            0.75f64,
                            420.0f64,
                        );
                        profile_class = mm305_acquisition_profile_class(
                            &raw mut profile,
                        );
                        predicted_speed = profile.exit_speed_mps;
                        classification_reason = profile.reason;
                        let mut replay: TaemFixedHacCandidate = taem_fixed_hac_evaluate_route(
                            planning_model,
                            current,
                            &raw mut trial_0,
                            0 as ::core::ffi::c_int,
                            0.5f64,
                            420.0f64,
                        );
                        if mm305_candidate_ok(&raw mut replay) {
                            profile_class = MM305_ACQ_PROFILE_FEASIBLE;
                            predicted_speed = replay.replay.final_geometry.airspeed_mps;
                            classification_reason = b"native replay qualified provisional route\0"
                                as *const u8 as *const ::core::ffi::c_char;
                            replay_quality = replay.quality_score;
                        } else if profile.valid {
                            profile_class = MM305_ACQ_PROFILE_INVALID;
                            predicted_speed = replay.replay.final_geometry.airspeed_mps;
                            classification_reason = if !replay.reason.is_null() {
                                replay.reason
                            } else {
                                b"native replay did not satisfy Final alignment\0"
                                    as *const u8 as *const ::core::ffi::c_char
                            };
                        }
                    }
                    let mut speed_is_prediction: bool = profile_class
                        as ::core::ffi::c_uint
                        == MM305_ACQ_PROFILE_FEASIBLE as ::core::ffi::c_int
                            as ::core::ffi::c_uint
                        || predicted_speed > 0.0f64 && !classification_reason.is_null()
                            && !strstr(
                                    classification_reason,
                                    b"Final handoff\0" as *const u8
                                        as *const ::core::ffi::c_char,
                                )
                                .is_null();
                    let mut speed_error: ::core::ffi::c_double = if speed_is_prediction
                        as ::core::ffi::c_int != 0
                        && predicted_speed.is_finite() as i32 != 0
                    {
                        fabs(predicted_speed - alignment_speed)
                    } else {
                        ::core::f32::NAN as ::core::ffi::c_double
                    };
                    let mut candidate: Mm305AcquisitionChoice = Mm305AcquisitionChoice {
                        valid: true_0 != 0,
                        route: trial_0,
                        sweep_rad: (*seed).sweep_rad,
                        profile_class: profile_class,
                        speed_error_mps: speed_error,
                        energy_metric: fabs(trial_0.length_m - required_path)
                            / cheap_scale,
                        circle_penalty: mm305_acquisition_circle_penalty(
                            &raw mut trial_0,
                        ),
                        profile_reason: classification_reason,
                        replay_quality: replay_quality,
                    };
                    if mm305_acquisition_choice_better(
                        &raw mut candidate,
                        &raw mut selected,
                    ) {
                        selected = candidate;
                    }
                }
            }
            i_1 = i_1.wrapping_add(1);
        }
        if !selected.valid
            || !mm305_install_acquisition_route(
                g,
                model,
                current,
                geometry,
                selected.route.hac.radius_m,
                side,
                selected.sweep_rad,
                &raw mut selected.route,
            )
        {
            (*g).mm305_acquisition_route_valid = false_0 != 0;
            return false_0 != 0;
        }
        let mut diag: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if !diag.is_null()
            && strcmp(diag, b"2\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
        {
            fprintf(
                stderr,
                b"MM305 acquisition selected: radius=%.0f sweep=%.0f path=%.0f lead=%.0f arc=%.0f native=%s exitErr=%s%.1f circlePenalty=%.3f reason=%s\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                selected.route.hac.radius_m,
                selected.sweep_rad * RAD2DEG,
                selected.route.length_m,
                selected.route.lead_length_m,
                selected.route.hac.arc_length_m,
                if selected.profile_class as ::core::ffi::c_uint
                    == MM305_ACQ_PROFILE_FEASIBLE as ::core::ffi::c_int
                        as ::core::ffi::c_uint
                {
                    b"feasible\0" as *const u8 as *const ::core::ffi::c_char
                } else if selected.profile_class as ::core::ffi::c_uint
                    == MM305_ACQ_PROFILE_TOO_SHORT as ::core::ffi::c_int
                        as ::core::ffi::c_uint
                {
                    b"too-short\0" as *const u8 as *const ::core::ffi::c_char
                } else if selected.profile_class as ::core::ffi::c_uint
                    == MM305_ACQ_PROFILE_TOO_LONG as ::core::ffi::c_int
                        as ::core::ffi::c_uint
                {
                    b"too-long\0" as *const u8 as *const ::core::ffi::c_char
                } else {
                    b"invalid\0" as *const u8 as *const ::core::ffi::c_char
                },
                if selected.speed_error_mps.is_finite() as i32 != 0 {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                } else {
                    b"n/a \0" as *const u8 as *const ::core::ffi::c_char
                },
                if selected.speed_error_mps.is_finite() as i32 != 0 {
                    selected.speed_error_mps
                } else {
                    0.0f64
                },
                selected.circle_penalty,
                if !selected.profile_reason.is_null() {
                    selected.profile_reason
                } else {
                    b"unknown\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
        }
    }
    let mut selected_sweep: ::core::ffi::c_double = (*g).mm305_acquisition_sweep_rad;
    let mut selected_path: ::core::ffi::c_double = (*g).mm305_acquisition_route.length_m;
    let mut have_route: bool = (*g).mm305_acquisition_route_valid as ::core::ffi::c_int
        != 0
        && taem_route_reference(
            &raw mut (*g).mm305_acquisition_route,
            geometry,
            &raw mut (*g).mm305_acquisition_route_cursor,
            reference,
            ::core::ptr::null_mut::<size_t>(),
        ) as ::core::ffi::c_int != 0;
    if !have_route {
        return false_0 != 0;
    }
    let mut route_remaining: ::core::ffi::c_double = taem_route_remaining_at_index(
        &raw mut (*g).mm305_acquisition_route,
        (*g).mm305_acquisition_route_cursor,
    );
    if route_remaining.is_finite() as i32 != 0 && route_remaining > 0.0f64 {
        selected_path = route_remaining;
    }
    (*reference).altitude_m = ::core::f32::NAN as ::core::ffi::c_double;
    (*reference).vertical_curvature_per_m = 0.0f64;
    let mut route_fpa: ::core::ffi::c_double = (*reference).flight_path_angle_deg;
    let mut measured_fpa: ::core::ffi::c_double = (*geometry).flight_path_angle_deg;
    let mut full: TaemTrackerOutput = taem_tracker_update(
        model,
        current,
        geometry,
        reference,
        dt,
    );
    if !full.valid {
        return false_0 != 0;
    }
    if route_fpa > measured_fpa
        && (!full.vertical_authority_ok
            || full.required_vertical_lift_mps2
                > full.delivered_vertical_lift_mps2 + 0.25f64)
    {
        let mut best: TaemPathReference = *reference;
        best.flight_path_angle_deg = measured_fpa;
        let mut best_demand: TaemTrackerOutput = taem_tracker_update(
            model,
            current,
            geometry,
            &raw mut best,
            dt,
        );
        if !best_demand.valid {
            return false_0 != 0;
        }
        let mut lo: ::core::ffi::c_double = measured_fpa;
        let mut hi: ::core::ffi::c_double = route_fpa;
        let mut i_2: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_2 < 8 as ::core::ffi::c_int {
            let mut candidate_0: ::core::ffi::c_double = 0.5f64 * (lo + hi);
            let mut trial_1: TaemPathReference = *reference;
            trial_1.flight_path_angle_deg = candidate_0;
            let mut td: TaemTrackerOutput = taem_tracker_update(
                model,
                current,
                geometry,
                &raw mut trial_1,
                dt,
            );
            let mut ok: bool = td.valid as ::core::ffi::c_int != 0
                && td.vertical_authority_ok as ::core::ffi::c_int != 0
                && td.required_vertical_lift_mps2
                    <= td.delivered_vertical_lift_mps2 + 0.25f64;
            if ok {
                lo = candidate_0;
                best = trial_1;
                best_demand = td;
            } else {
                hi = candidate_0;
            }
            i_2 += 1;
        }
        *reference = best;
        full = best_demand;
    }
    if !full.valid {
        return false_0 != 0;
    }
    let mut full_lateral: ::core::ffi::c_double = fabs(full.required_lateral_accel_mps2);
    let mut lateral_limit: ::core::ffi::c_double = 0.80f64
        * fmax(0.0f64, full.available_lateral_accel_mps2);
    if full.lateral_authority_ok as ::core::ffi::c_int != 0
        && full_lateral <= lateral_limit + 1e-6f64
    {
        *demand = full;
    } else {
        let mut shaped: TaemPathReference = *reference;
        let mut best_0: TaemTrackerOutput = full;
        let mut lo_0: ::core::ffi::c_double = 0.0f64;
        let mut hi_0: ::core::ffi::c_double = 1.0f64;
        let mut i_3: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_3 < 7 as ::core::ffi::c_int {
            let mut scale: ::core::ffi::c_double = 0.5f64 * (lo_0 + hi_0);
            let mut trial_2: TaemPathReference = *reference;
            trial_2.curvature_right_per_m = (*reference).curvature_right_per_m * scale;
            let mut td_0: TaemTrackerOutput = taem_tracker_update(
                model,
                current,
                geometry,
                &raw mut trial_2,
                dt,
            );
            let mut req: ::core::ffi::c_double = fabs(td_0.required_lateral_accel_mps2);
            let mut cap: ::core::ffi::c_double = 0.80f64
                * fmax(0.0f64, td_0.available_lateral_accel_mps2);
            let mut ok_0: bool = td_0.valid as ::core::ffi::c_int != 0
                && td_0.lateral_authority_ok as ::core::ffi::c_int != 0
                && req <= cap + 1e-6f64;
            if ok_0 {
                lo_0 = scale;
                shaped = trial_2;
                best_0 = td_0;
            } else {
                hi_0 = scale;
            }
            i_3 += 1;
        }
        *reference = shaped;
        *demand = best_0;
    }
    if !path_remaining_out.is_null() {
        *path_remaining_out = selected_path;
    }
    if !required_drag_out.is_null() {
        *required_drag_out = energy_excess / fmax(selected_path, 1000.0f64);
    }
    if !available_drag_out.is_null() {
        *available_drag_out = available_drag;
    }
    let mut diag_0: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !diag_0.is_null()
        && strcmp(diag_0, b"2\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
    {
        fprintf(
            stderr,
            b"MM305 acquisition route: radius=%.0f sweep=%.0f path=%.0f E=%.0f reqD=%.2f availD=%.2f joined=%d cursor=%zu/%zu xt=%.0f courseErr=%.1f V=%.1f h=%.0f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*g).mm305_acquisition_route.hac.radius_m,
            selected_sweep * RAD2DEG,
            selected_path,
            energy_excess,
            energy_excess / fmax(selected_path, 1000.0f64),
            available_drag,
            if joined_fixed_hac as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            (*g).mm305_acquisition_route_cursor,
            (*g).mm305_acquisition_route.count,
            (*demand).cross_track_error_m,
            (*demand).course_error_deg,
            (*geometry).airspeed_mps,
            (*geometry).altitude_above_runway_m,
        );
    }
    return (*demand).valid;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_direct_runway_fallback_window(
    mut g: *const GuidanceMachine,
    mut runway_along_m: ::core::ffi::c_double,
    mut horizontal_speed: ::core::ffi::c_double,
    mut final_station_distance: ::core::ffi::c_double,
    mut capture: *const RunwayCaptureEnvelope,
) -> bool {
    if runway_along_m.is_finite() as i32 == 0 || horizontal_speed.is_finite() as i32 == 0
        || !(horizontal_speed > 0.0f64) || final_station_distance.is_finite() as i32 == 0
        || !(final_station_distance > 0.0f64)
    {
        return false_0 != 0;
    }
    if runway_along_m >= -final_station_distance {
        return true_0 != 0;
    }
    if g.is_null() || (*g).mm305_route_committed as ::core::ffi::c_int != 0
        || capture.is_null() || !(*capture).valid
        || (*capture).reachable as ::core::ffi::c_int != 0
        || (*capture).control_response_time_s.is_finite() as i32 == 0
        || !((*capture).control_response_time_s > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut capture_time: ::core::ffi::c_double = fmax(
        (*capture).control_response_time_s,
        fmax((*capture).heading_capture_time_s, (*capture).lateral_capture_time_s),
    );
    let mut capture_distance: ::core::ffi::c_double = horizontal_speed * capture_time;
    return capture_distance.is_finite() as i32 != 0 && capture_distance > 0.0f64
        && runway_along_m >= -final_station_distance - capture_distance;
}
unsafe extern "C" fn taem_energy_law_line_bank(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut cfg: *const LandingConfiguration,
    mut course: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let g0: ::core::ffi::c_double = 9.81f64;
    let mut Vh: ::core::ffi::c_double = fmax(1.0f64, (*t).horizontal_speed);
    let mut track_error: ::core::ffi::c_double = norm_signed_deg(
        course - (*cfg).site.runway_heading,
    ) * DEG2RAD;
    let mut cross_rate: ::core::ffi::c_double = Vh * sin(track_error);
    let mut along: ::core::ffi::c_double = (*t).runway_along_track;
    let mut align_time: ::core::ffi::c_double = fmax(
        6.0f64,
        fmax(0.0f64, -2800.0f64 - along) / Vh,
    );
    let mut omega: ::core::ffi::c_double = clampd(2.2f64 / align_time, 0.06f64, 0.18f64);
    let mut bank_limit: ::core::ffi::c_double = fmin(
        30.0f64,
        terminal_lateral_bank_limit(t, &raw const (*cfg).vehicle),
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
        &raw const (*cfg).guidance,
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
    let mut omega_limit: ::core::ffi::c_double = 0.5f64
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
    terminal_pavement_intercept_rate(t, cfg, &raw mut desired_rate);
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
            swing = if reversal_time.is_finite() as i32 != 0 {
                reversal_time
            } else {
                10.0f64
            };
        }
        lateral_accel = clampd(
            (desired_rate - cross_rate - 0.5f64 * present_accel * swing)
                / (0.5f64 * swing + RATE_CLOSURE_TAU),
            -lateral_limit,
            lateral_limit,
        );
        it += 1;
    }
    return clampd(
        asin(clampd(lateral_accel / lift_accel, -1.0f64, 1.0f64)) * RAD2DEG,
        -bank_limit,
        bank_limit,
    );
}
unsafe extern "C" fn taem_law_final_handoff(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut geometry: *const TaemGeometryState,
    mut law_energy_short: bool,
    mut admit_pending: *mut bool,
    mut admit_since_ut: *mut ::core::ffi::c_double,
    mut low_energy_pending: *mut bool,
    mut low_energy_since_ut: *mut ::core::ffi::c_double,
) -> *const ::core::ffi::c_char {
    let mut final_blocked: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut final_plan: TerminalPreflarePlan = TerminalPreflarePlan {
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
    let mut final_approach: bool = terminal_outer_capture_admissible(
        g,
        t,
        course,
        planet,
        aero,
        cfg,
        &raw mut final_plan,
    );
    let mut final_contract: TaemTerminalContract = terminal_delivery_contract(
        g,
        t,
        course,
        planet,
        cfg,
        &raw mut final_plan,
    );
    let mut final_eval: TaemTerminalEvaluation = taem_exec_evaluate_terminal_contract(
        &raw mut final_contract,
    );
    let mut admissible: bool = final_approach as ::core::ffi::c_int != 0
        && final_eval.valid as ::core::ffi::c_int != 0
        && final_eval.feasible as ::core::ffi::c_int != 0;
    if !admissible {
        *admit_pending = false_0 != 0;
        final_blocked = if !final_approach {
            if !final_plan.reject_reason.is_null()
                && strcmp(
                    final_plan.reject_reason,
                    b"none\0" as *const u8 as *const ::core::ffi::c_char,
                ) != 0 as ::core::ffi::c_int
            {
                final_plan.reject_reason
            } else {
                b"runway capture gate\0" as *const u8 as *const ::core::ffi::c_char
            }
        } else {
            taem_terminal_block_reason_string(final_eval.block_reason)
        };
    } else if !*admit_pending {
        *admit_pending = true_0 != 0;
        *admit_since_ut = (*t).ut;
    }
    taem_exec_sync_with_contract(
        g,
        t,
        &raw const (*cfg).guidance,
        planet,
        aero,
        cfg,
        &raw mut final_contract,
        false_0 != 0,
        false_0 != 0,
    );
    let mut energy_short: bool = !admissible
        && law_energy_short as ::core::ffi::c_int != 0
        && (*geometry).runway_along_m < 0.0f64
        && terminal_preflare_alignment_valid(g, t, course, cfg) as ::core::ffi::c_int
            != 0;
    if !energy_short {
        *low_energy_pending = false_0 != 0;
    } else if !*low_energy_pending {
        *low_energy_pending = true_0 != 0;
        *low_energy_since_ut = (*t).ut;
    }
    (*g).jet_assist_throttle = 0.0f64;
    if energy_short as ::core::ffi::c_int != 0
        && (*t).ut - *low_energy_since_ut >= TEL_LOW_ENERGY_DWELL_S
    {
        let mut deficit: ::core::ffi::c_double = if final_contract
            .specific_energy_margin
            .is_finite() as i32 != 0
        {
            fmax(0.0f64, -final_contract.specific_energy_margin)
        } else {
            0.0f64
        };
        (*g).jet_assist_throttle = clampd(
            deficit / JET_ASSIST_FULL_DEFICIT_J_KG,
            JET_ASSIST_MIN_THROTTLE,
            1.0f64,
        );
    }
    if energy_short as ::core::ffi::c_int != 0
        && (*t).ut - *low_energy_since_ut >= TEL_LOW_ENERGY_DWELL_S
    {
        let mut observation: TaemExecObservation = TaemExecObservation {
            ut: (*t).ut,
            relative_velocity: (*t).true_air_speed,
            checkpoint_restart: false_0 != 0,
        };
        if taem_exec_deliver_low_energy(&raw mut (*g).taem_exec, &raw mut observation) {
            (*g).taem_exec.terminal_contract = final_contract;
            (*g).taem_exec.terminal_evaluation = final_eval;
            (*g).mm305_hac_exit_reached = true_0 != 0;
            (*g).hac_completed = true_0 != 0;
            (*g).hac_captured = true_0 != 0;
            (*g).final_approach_captured = true_0 != 0;
            (*g).terminal_final_low_energy = true_0 != 0;
            if final_plan.feasible {
                terminal_store_preflare_plan(g, &raw mut final_plan);
            }
            terminal_set_stage(g, TERMINAL_OUTER_FINAL, (*t).ut);
            robust_pid_reset(&raw mut (*g).final_altitude_pid);
            robust_pid_reset(&raw mut (*g).speed_pid);
            robust_pid_reset(&raw mut (*g).flare_sink_pid);
            if !(*g).diagnostic_shadow {
                fprintf(
                    stderr,
                    b"MM305 low-energy Final delivery: V %.1f m/s, h %.0f m, along %.0f m, preflare %s.\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*geometry).airspeed_mps,
                    (*geometry).altitude_above_runway_m,
                    (*geometry).runway_along_m,
                    if !final_plan.reject_reason.is_null() {
                        final_plan.reject_reason
                    } else {
                        b"none\0" as *const u8 as *const ::core::ffi::c_char
                    },
                );
            }
        }
    }
    if admissible as ::core::ffi::c_int != 0 && (*t).ut - *admit_since_ut >= 1.0f64 {
        (*g).taem_exec.terminal_contract = final_contract;
        (*g).taem_exec.terminal_evaluation = final_eval;
        (*g).terminal_predicted_touchdown_speed = terminal_delivery_touchdown_speed(
            g,
            t,
            planet,
            cfg,
            final_plan.trigger_altitude,
        );
        (*g).mm305_hac_exit_reached = true_0 != 0;
        (*g).hac_completed = true_0 != 0;
        (*g).hac_captured = true_0 != 0;
        taem_exec_sync_with_contract(
            g,
            t,
            &raw const (*cfg).guidance,
            planet,
            aero,
            cfg,
            &raw mut final_contract,
            false_0 != 0,
            false_0 != 0,
        );
    }
    return final_blocked;
}
unsafe extern "C" fn taem_energy_law_guidance(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut control_model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
    mut current: *const TerminalDynamicState,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    let mut aoa: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut bank: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut L: *mut TaemEnergyLawState = &raw mut (*g).taem_energy_law;
    if !taem_energy_law_step(
        L,
        control_model,
        geometry,
        (*current).mass_kg,
        (*t).angle_of_attack,
        (*t).ut,
        (*t).roll,
        &raw mut aoa,
        &raw mut bank,
    ) {
        return terminal_abort(
            g,
            b"MM305 energy law could not produce a bounded command.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if (*L).station_slope_deg.is_finite() as i32 != 0 {
        if !(*g).terminal_final_handoff_latched {
            (*g).terminal_final_handoff_distance = (*control_model)
                .guidance
                .final_approach_distance;
            (*g).terminal_final_handoff_speed_mps = (*control_model)
                .guidance
                .final_alignment_speed;
        }
        (*g).terminal_final_handoff_latched = true_0 != 0;
        (*g).terminal_final_handoff_slope_deg = (*L).station_slope_deg;
    }
    fn trace_last_ut_tls() -> *mut ::core::ffi::c_double {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<::core::ffi::c_double> = const { ::core::cell::UnsafeCell::new(-::core::f32::INFINITY
        as ::core::ffi::c_double) };
        }
        CELL.with(|c| c.get())
    }
    if !(*g).diagnostic_shadow
        && ((*trace_last_ut_tls()).is_finite() as i32 == 0 || (*t).ut < (*trace_last_ut_tls())
            || (*t).ut - (*trace_last_ut_tls()) >= 2.0f64)
    {
        (*trace_last_ut_tls()) = (*t).ut;
        fprintf(
            stderr,
            b"TAEM_ENERGY ut=%.1f along=%+.0f cross=%+.0f h=%.0f V=%.1f M=%.2f fpa=%.1f hdgErr=%+.1f k=%.2f loop=%d leg=%d side=%+d R=%.0f xTurn=%.0f sGo=%.0f sReq=%.0f slope=%.1f aoa=%.1f/%.1f bank=%+.1f/%+.1f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            (*geometry).runway_along_m,
            (*geometry).runway_cross_m,
            (*geometry).altitude_above_runway_m,
            (*geometry).airspeed_mps,
            (*t).mach,
            (*geometry).flight_path_angle_deg,
            (*geometry).heading_error_deg,
            (*L).k,
            (*L).loop_0 as ::core::ffi::c_int,
            (*L).leg,
            (*L).side,
            (*L).radius,
            (*L).x_turn,
            (*L).s_go,
            (*L).s_req,
            (*L).station_slope_deg,
            (*t).angle_of_attack,
            aoa,
            (*t).roll,
            bank,
        );
    }
    let mut heading: ::core::ffi::c_double = course;
    let mut line_c: TelContext = TelContext {
        model: ::core::ptr::null::<TerminalModel>(),
        d_station: 0.,
        h_station: 0.,
        v_station: 0.,
        e_station: 0.,
        aoa_max: 0.,
        slope_deg: 0.,
        final_line: false,
    };
    let mut line_s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    tel_setup(
        control_model,
        geometry,
        (*current).mass_kg,
        (*t).angle_of_attack,
        (*t).roll,
        &raw mut line_c,
        &raw mut line_s,
    );
    if (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int
        && tel_final_line_ready(&raw mut line_c, &raw mut line_s) as ::core::ffi::c_int
            != 0
    {
        bank = taem_energy_law_line_bank(g, t, cfg, course);
        heading = norm_deg(
            (*cfg).site.runway_heading
                + clampd(
                    atan2(
                        -(*t).runway_cross_track,
                        fmax(400.0f64, (*t).true_air_speed * 6.0f64),
                    ) * RAD2DEG,
                    -20.0f64,
                    20.0f64,
                ),
        );
        let mut intercept_rate: ::core::ffi::c_double = 0.;
        if terminal_pavement_intercept_rate(t, cfg, &raw mut intercept_rate) {
            heading = norm_deg(
                (*cfg).site.runway_heading
                    + asin(
                        clampd(
                            intercept_rate / fmax(1.0f64, (*t).horizontal_speed),
                            -1.0f64,
                            1.0f64,
                        ),
                    ) * RAD2DEG,
            );
        }
    }
    let mut command: GuidanceCommand = atmospheric(
        t,
        heading,
        bank,
        &raw const (*cfg).vehicle,
        0.0f64,
        false_0 != 0,
        PROFILE_TAEM,
    );
    command.heading_control_enabled = false_0 != 0;
    command.hac_control_tuning = true_0 != 0;
    command.terminal_pitch_tuning = true_0 != 0;
    command.has_target_aoa = true_0 != 0;
    command.target_aoa = clampd(aoa, 0.0f64, (*cfg).vehicle.maximum_angle_of_attack);
    command.target_pitch = (*t).flight_path_angle + command.target_aoa;
    command.gear = (*t).gear as ::core::ffi::c_int != 0
        || (*t).radar_altitude <= (*cfg).guidance.gear_deployment_altitude;
    (*g).terminal_path_committed = true_0 != 0;
    (*g).terminal_path_kind = TERMINAL_PATH_DIRECT;
    (*g).mm305_acquisition_route_valid = false_0 != 0;
    let mut station: ::core::ffi::c_double = fmax(
        500.0f64,
        (*control_model).guidance.final_approach_distance,
    );
    let mut final_blocked: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    if (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int
        && (*geometry).runway_along_m >= -station - 2000.0f64
    {
        final_blocked = taem_law_final_handoff(
            g,
            t,
            course,
            planet,
            aero,
            cfg,
            geometry,
            (*L).k <= TEL_K_SHORT,
            &raw mut (*L).admit_pending,
            &raw mut (*L).admit_since_ut,
            &raw mut (*L).low_energy_pending,
            &raw mut (*L).low_energy_since_ut,
        );
    }
    static mut legs: [*const ::core::ffi::c_char; 5] = [
        b"centreline\0" as *const u8 as *const ::core::ffi::c_char,
        b"turnaround\0" as *const u8 as *const ::core::ffi::c_char,
        b"downwind\0" as *const u8 as *const ::core::ffi::c_char,
        b"HAC\0" as *const u8 as *const ::core::ffi::c_char,
        b"Final line\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    let mut status: [::core::ffi::c_char; 256] = [0; 256];
    let mut n: ::core::ffi::c_int = snprintf(
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"MM305 energy law RW%02.0f: %s %s, drag level %.2f, path to go %.1f km.\0"
            as *const u8 as *const ::core::ffi::c_char,
        norm_deg((*control_model).site.runway_heading) / 10.0f64,
        if (*L).loop_0 as ::core::ffi::c_int != 0 {
            b"overhead loop\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"straight-in\0" as *const u8 as *const ::core::ffi::c_char
        },
        legs[(if (*L).leg < 0 as ::core::ffi::c_int || (*L).leg > 4 as ::core::ffi::c_int
        {
            0 as ::core::ffi::c_int
        } else {
            (*L).leg
        }) as usize],
        (*L).k,
        (*L).s_go / 1000.0f64,
    );
    if !final_blocked.is_null() && n > 0 as ::core::ffi::c_int
        && (n as size_t) < ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as usize
    {
        snprintf(
            (&raw mut status as *mut ::core::ffi::c_char).offset(n as isize),
            (::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t)
                .wrapping_sub(n as size_t),
            b" Final blocked: %s.\0" as *const u8 as *const ::core::ffi::c_char,
            final_blocked,
        );
    }
    let mut r: GuidanceResult = stabilized(
        g,
        result_make(
            PHASE_TAEM,
            command,
            &raw mut status as *mut ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        ),
        t,
        &raw const (*cfg).vehicle,
        &raw const (*cfg).guidance,
        dt,
    );
    if !(*g).diagnostic_shadow
        && (*cfg).guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
        && fabs((*t).ut - (*L).last_pred_ut) < 1e-6f64
    {
        let mut display: TelContext = TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        };
        let mut point: TelPoint = TelPoint {
            x: 0.,
            y: 0.,
            h: 0.,
            v: 0.,
            g: 0.,
            psi: 0.,
            m: 0.,
            aoa: 0.,
            aoa_rate: 0.,
            bank: 0.,
            ground_speed: 0.,
        };
        tel_setup(
            control_model,
            geometry,
            (*current).mass_kg,
            (*t).angle_of_attack,
            (*t).roll,
            &raw mut display,
            &raw mut point,
        );
        tel_predict_pattern_trace(
            L,
            &raw mut display,
            &raw mut point,
            (*L).k,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            &raw mut r.reference,
            (*t).ut,
        );
    }
    (*g).phase = PHASE_TAEM;
    return r;
}
unsafe extern "C" fn taem_hac_law_guidance(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut control_model: *const TerminalModel,
    mut recorded: *const AeroTable,
    mut geometry: *const TaemGeometryState,
    mut current: *const TerminalDynamicState,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    let mut H: *mut TaemHacLawState = &raw mut (*g).taem_hac_law;
    let mut x: HacCtx = HacCtx {
        c: TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        },
        aero: ::core::ptr::null::<AeroTable>(),
        live: ::core::ptr::null::<TaemHacLiveAero>(),
        mass: 0.,
        aoa_supersonic: 0.,
    };
    let mut s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    hac_context(
        &raw mut x,
        control_model,
        recorded,
        &raw mut (*g).taem_hac_live_aero,
        geometry,
        (*current).mass_kg,
        (*t).angle_of_attack,
        (*t).roll,
        &raw mut s,
    );
    x.aoa_supersonic = (*cfg).vehicle.maximum_angle_of_attack;
    if !(*H).initialized {
        hac_initialize(H, &raw mut x, &raw mut s, (*t).ut);
    }
    (*H).trace = !(*g).diagnostic_shadow;
    hac_live_aero_observe(
        &raw mut (*g).taem_hac_live_aero,
        recorded,
        t,
        (*g).taem_speedbrake_fraction,
    );
    let mut drag_accel: ::core::ffi::c_double = if (*t).physics_sample_valid
        as ::core::ffi::c_int != 0 && (*t).mass > 0.0f64
        && (*t).drag_force.is_finite() as i32 != 0
    {
        fabs((*t).drag_force) / (*t).mass
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    if drag_accel.is_finite() as i32 == 0 {
        let mut q: ::core::ffi::c_double = 0.;
        let mut mach: ::core::ffi::c_double = 0.;
        let mut lift: ::core::ffi::c_double = 0.;
        let mut drag: ::core::ffi::c_double = 0.;
        tel_forces(
            &raw mut x.c,
            &raw mut s,
            s.aoa,
            &raw mut q,
            &raw mut mach,
            &raw mut lift,
            &raw mut drag,
        );
        drag_accel = drag / s.m;
    }
    let mut acq: HacPrediction = hac_update(
        H,
        &raw mut x,
        &raw mut s,
        (*t).ut,
        drag_accel,
    );
    let mut bank: ::core::ffi::c_double = 0.;
    let mut aoa: ::core::ffi::c_double = 0.;
    if (*H).phase == TAEM_HAC_PHASE_DRAG as ::core::ffi::c_int {
        let mut left: ::core::ffi::c_double = 0.;
        hac_drag_command(H, &raw mut x, &raw mut s, &raw mut aoa, &raw mut left);
        bank = -left;
        let mut step: ::core::ffi::c_double = clampd(dt, 0.0f64, 1.0f64);
        if (*H).drag_aoa_cmd.is_finite() as i32 == 0 {
            (*H).drag_aoa_cmd = (*t).angle_of_attack;
        }
        if (*H).drag_bank_cmd.is_finite() as i32 == 0 {
            (*H).drag_bank_cmd = (*t).roll;
        }
        (*H).drag_aoa_cmd = clampd(
            aoa,
            (*H).drag_aoa_cmd - HAC_DRAG_AOA_SLEW * step,
            (*H).drag_aoa_cmd + HAC_DRAG_AOA_SLEW * step,
        );
        let mut roll_slew: ::core::ffi::c_double = fmax(
            1.0f64,
            (*cfg).guidance.taem_roll_rate,
        ) * step;
        (*H).drag_bank_cmd = clampd(
            bank,
            (*H).drag_bank_cmd - roll_slew,
            (*H).drag_bank_cmd + roll_slew,
        );
        aoa = (*H).drag_aoa_cmd;
        bank = (*H).drag_bank_cmd;
    } else {
        bank = -hac_lateral_left(H, &raw mut x, &raw mut s, &raw mut acq);
        aoa = hac_vertical(H, &raw mut x, &raw mut s, bank, drag_accel);
    }
    if aoa.is_finite() as i32 == 0 || bank.is_finite() as i32 == 0 {
        return terminal_abort(
            g,
            b"MM305 HAC guidance could not produce a bounded command.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*H).station_slope_deg = x.c.slope_deg;
    if !(*g).terminal_final_handoff_latched {
        (*g).terminal_final_handoff_distance = (*control_model)
            .guidance
            .final_approach_distance;
        (*g).terminal_final_handoff_speed_mps = (*control_model)
            .guidance
            .final_alignment_speed;
    }
    (*g).terminal_final_handoff_latched = true_0 != 0;
    (*g).terminal_final_handoff_slope_deg = (*H).station_slope_deg;
    static mut phases: [*const ::core::ffi::c_char; 5] = [
        b"drag energy\0" as *const u8 as *const ::core::ffi::c_char,
        b"S-turn\0" as *const u8 as *const ::core::ffi::c_char,
        b"acquisition\0" as *const u8 as *const ::core::ffi::c_char,
        b"heading alignment\0" as *const u8 as *const ::core::ffi::c_char,
        b"prefinal\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    fn trace_last_ut_tls() -> *mut ::core::ffi::c_double {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<::core::ffi::c_double> = const { ::core::cell::UnsafeCell::new(-::core::f32::INFINITY
        as ::core::ffi::c_double) };
        }
        CELL.with(|c| c.get())
    }
    if !(*g).diagnostic_shadow
        && ((*trace_last_ut_tls()).is_finite() as i32 == 0 || (*t).ut < (*trace_last_ut_tls())
            || (*t).ut - (*trace_last_ut_tls()) >= 2.0f64)
    {
        (*trace_last_ut_tls()) = (*t).ut;
        fprintf(
            stderr,
            b"TAEM_HAC ut=%.1f phase=%s side=%+d RF=%.0f psi=%.0f sGo=%.0f tan=%.0f along=%+.0f cross=%+.0f h=%.0f href=%.0f V=%.1f Vref=%.1f M=%.2f E=%.0f Eref=%.0f dE=%+.0f turn=%.0f eta=%.2f kD=%.2f kL=%.2f aoa=%.1f/%.1f bank=%+.1f/%+.1f sb=%.2f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            phases[(*H).phase as usize],
            (*H).side,
            (*H).rf,
            (*H).psi * RAD2DEG,
            (*H).s_go,
            (*H).tangent_dist,
            (*geometry).runway_along_m,
            (*geometry).runway_cross_m,
            s.h,
            (*H).h_ref,
            s.v,
            (*H).v_ref,
            (*t).mach,
            hac_energy(&raw mut s),
            (*H).e_ref,
            (*H).e_err,
            (*H).turn_cost,
            if (*H).phase == TAEM_HAC_PHASE_DRAG as ::core::ffi::c_int {
                (*H).drag_required
            } else {
                (*H).eta_live
            },
            hac_kappa(&raw mut (*g).taem_hac_live_aero, (*t).mach, false_0 != 0),
            hac_kappa(&raw mut (*g).taem_hac_live_aero, (*t).mach, true_0 != 0),
            (*t).angle_of_attack,
            aoa,
            (*t).roll,
            bank,
            (*g).taem_speedbrake_fraction,
        );
    }
    let mut heading: ::core::ffi::c_double = course;
    if (*H).final_line {
        bank = taem_energy_law_line_bank(g, t, cfg, course);
        heading = norm_deg(
            (*cfg).site.runway_heading
                + clampd(
                    atan2(
                        -(*t).runway_cross_track,
                        fmax(400.0f64, (*t).true_air_speed * 6.0f64),
                    ) * RAD2DEG,
                    -20.0f64,
                    20.0f64,
                ),
        );
        let mut intercept_rate: ::core::ffi::c_double = 0.;
        if terminal_pavement_intercept_rate(t, cfg, &raw mut intercept_rate) {
            heading = norm_deg(
                (*cfg).site.runway_heading
                    + asin(
                        clampd(
                            intercept_rate / fmax(1.0f64, (*t).horizontal_speed),
                            -1.0f64,
                            1.0f64,
                        ),
                    ) * RAD2DEG,
            );
        }
    }
    let mut command: GuidanceCommand = atmospheric(
        t,
        heading,
        bank,
        &raw const (*cfg).vehicle,
        0.0f64,
        false_0 != 0,
        PROFILE_TAEM,
    );
    command.heading_control_enabled = false_0 != 0;
    command.hac_control_tuning = true_0 != 0;
    command.terminal_pitch_tuning = true_0 != 0;
    command.has_target_aoa = true_0 != 0;
    command.target_aoa = clampd(aoa, 0.0f64, (*cfg).vehicle.maximum_angle_of_attack);
    command.target_pitch = (*t).flight_path_angle + command.target_aoa;
    command.gear = (*t).gear as ::core::ffi::c_int != 0
        || (*t).radar_altitude <= (*cfg).guidance.gear_deployment_altitude;
    let mut sb_dt: ::core::ffi::c_double = if (*g).taem_speedbrake_ut > 0.0f64
        && (*t).ut > (*g).taem_speedbrake_ut
    {
        fmin((*t).ut - (*g).taem_speedbrake_ut, 1.0f64)
    } else {
        0.0f64
    };
    (*g).taem_speedbrake_ut = (*t).ut;
    let mut sb_target: ::core::ffi::c_double = if (*cfg).guidance.terminal_speedbrake
        as ::core::ffi::c_int != 0
    {
        hac_speedbrake(H, (*t).mach, sb_dt)
    } else {
        0.0f64
    };
    (*g).taem_speedbrake_fraction = clampd(
        sb_target,
        (*g).taem_speedbrake_fraction - SPEEDBRAKE_STOW_SLEW * sb_dt,
        (*g).taem_speedbrake_fraction + SPEEDBRAKE_DEPLOY_SLEW * sb_dt,
    );
    command.speedbrake_fraction = (*g).taem_speedbrake_fraction;
    (*g).terminal_path_committed = true_0 != 0;
    (*g).terminal_path_kind = TERMINAL_PATH_DIRECT;
    (*g).mm305_acquisition_route_valid = false_0 != 0;
    (*g).hac_side_selected = true_0 != 0;
    (*g).hac_side = (*H).side as ::core::ffi::c_double;
    (*g).hac_radius = (*H).rf;
    (*g).hac_captured = (*H).phase >= TAEM_HAC_PHASE_HAC as ::core::ffi::c_int;
    let mut station: ::core::ffi::c_double = fmax(
        500.0f64,
        (*control_model).guidance.final_approach_distance,
    );
    let mut final_blocked: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    if (*H).final_line as ::core::ffi::c_int != 0
        && (*geometry).runway_along_m >= -station - 2000.0f64
    {
        final_blocked = taem_law_final_handoff(
            g,
            t,
            course,
            planet,
            aero,
            cfg,
            geometry,
            (*H).energy_level <= TEL_K_SHORT,
            &raw mut (*H).admit_pending,
            &raw mut (*H).admit_since_ut,
            &raw mut (*H).low_energy_pending,
            &raw mut (*H).low_energy_since_ut,
        );
    }
    let mut status: [::core::ffi::c_char; 256] = [0; 256];
    let mut n: ::core::ffi::c_int = snprintf(
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"MM305 HAC RW%02.0f: %s, %s HAC RF %.1f km turn %.0f deg, range %.1f km, energy %+.1f km.\0"
            as *const u8 as *const ::core::ffi::c_char,
        norm_deg((*control_model).site.runway_heading) / 10.0f64,
        phases[(*H).phase as usize],
        if (*H).psi > TEL_PI {
            b"overhead\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"straight-in\0" as *const u8 as *const ::core::ffi::c_char
        },
        (*H).rf / 1000.0f64,
        (*H).psi * RAD2DEG,
        (*H).s_go / 1000.0f64,
        (*H).e_err / 1000.0f64,
    );
    if !final_blocked.is_null() && n > 0 as ::core::ffi::c_int
        && (n as size_t) < ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as usize
    {
        snprintf(
            (&raw mut status as *mut ::core::ffi::c_char).offset(n as isize),
            (::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t)
                .wrapping_sub(n as size_t),
            b" Final blocked: %s.\0" as *const u8 as *const ::core::ffi::c_char,
            final_blocked,
        );
    }
    let mut r: GuidanceResult = stabilized(
        g,
        result_make(
            PHASE_TAEM,
            command,
            &raw mut status as *mut ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        ),
        t,
        &raw const (*cfg).vehicle,
        &raw const (*cfg).guidance,
        dt,
    );
    if !(*g).diagnostic_shadow && r.reference.count == 0 as size_t {
        let mut hd: ::core::ffi::c_double = (*control_model).site.runway_heading
            * DEG2RAD;
        let mut origin: GeoPoint = GeoPoint {
            latitude: (*control_model).site.latitude,
            longitude: (*control_model).site.longitude,
            altitude: (*control_model).site.altitude,
        };
        let mut D: ::core::ffi::c_double = x.c.d_station;
        let mut psi0: ::core::ffi::c_double = (*H).psi;
        let mut line: HacPrediction = HacPrediction {
            valid: 0 as ::core::ffi::c_int != 0,
            s_go: 0.,
            psi_t: 0.,
            tx: 0.,
            ty: 0.,
            tangent_dist: 0.,
            acq_turn: 0.,
            acq_arc: 0.,
            hac_arc: 0.,
            turn_cost: 0.,
            e_ref: 0.,
        };
        if (*H).phase <= TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int {
            line = hac_predict_acquisition(
                &raw mut x,
                &raw mut (*H).ref_0,
                (*H).side,
                (*H).rf,
                &raw mut s,
            );
            if line.valid {
                psi0 = line.psi_t;
            }
        }
        let mut arc0: ::core::ffi::c_double = hac_arc_length((*H).rf, psi0);
        let n_line: ::core::ffi::c_int = if line.valid as ::core::ffi::c_int != 0 {
            12 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        };
        let n_arc: ::core::ffi::c_int = 48 as ::core::ffi::c_int;
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i <= n_line + n_arc {
            let mut px: ::core::ffi::c_double = 0.;
            let mut py: ::core::ffi::c_double = 0.;
            let mut s_ref: ::core::ffi::c_double = 0.;
            if i < n_line {
                let mut f: ::core::ffi::c_double = i as ::core::ffi::c_double
                    / n_line as ::core::ffi::c_double;
                px = s.x + f * (line.tx - s.x);
                py = s.y + f * (line.ty - s.y);
                s_ref = arc0 + hypot(line.tx - px, line.ty - py);
            } else {
                let mut psi: ::core::ffi::c_double = psi0
                    * (1.0f64
                        - (i - n_line) as ::core::ffi::c_double
                            / n_arc as ::core::ffi::c_double);
                let mut r_s: ::core::ffi::c_double = hac_radius_at((*H).rf, psi);
                let mut ang: ::core::ffi::c_double = hac_exit_angle((*H).side)
                    - (*H).side as ::core::ffi::c_double * psi;
                px = -D + r_s * cos(ang);
                py = (*H).side as ::core::ffi::c_double * (*H).rf + r_s * sin(ang);
                s_ref = hac_arc_length((*H).rf, psi);
            }
            let mut h_r: ::core::ffi::c_double = 0.;
            let mut v_r: ::core::ffi::c_double = 0.;
            hac_reference_at(
                &raw mut (*H).ref_0,
                s_ref,
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                &raw mut h_r,
                &raw mut v_r,
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
            );
            let mut pt: GeoPoint = local_point(
                origin,
                px * sin(hd) - py * cos(hd),
                px * cos(hd) + py * sin(hd),
                (*control_model).world.radius_m,
                (*control_model).site.altitude + h_r,
            );
            trajectory_append(
                &raw mut r.reference,
                TrajectoryPoint {
                    ut: (*t).ut,
                    latitude: pt.latitude,
                    longitude: pt.longitude,
                    altitude: pt.altitude,
                    speed: v_r,
                    phase: PHASE_TAEM,
                    kind: TRAJ_REFERENCE,
                },
            );
            i += 1;
        }
    }
    (*g).phase = PHASE_TAEM;
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn taem_guidance_native(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut vehicle_state: *const VehicleState,
    mut course: ::core::ffi::c_double,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut model: *const TerminalModel,
    mut dt: ::core::ffi::c_double,
) -> GuidanceResult {
    if g.is_null() || t.is_null() || vehicle_state.is_null() || cfg.is_null()
        || model.is_null() || !(*model).replay_validated
    {
        return terminal_abort(
            g,
            b"MM305 native fixed-HAC model is unavailable; no legacy HAC fallback is permitted.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if (*g).terminal_final_test_mode as ::core::ffi::c_int != 0
        && (*g).hac_completed as ::core::ffi::c_int != 0
    {
        return result_make(
            PHASE_TAEM,
            atmospheric(
                t,
                course,
                0.0f64,
                &raw const (*cfg).vehicle,
                0.0f64,
                false_0 != 0,
                PROFILE_TAEM,
            ),
            b"MM305 native HAC exit reached; awaiting the existing numeric Final contract.\0"
                as *const u8 as *const ::core::ffi::c_char,
            ::core::ptr::null::<::core::ffi::c_char>(),
        );
    }
    mm305_trim_observe(g, t);
    let mut current: TerminalDynamicState = terminal_live_state(
        g,
        vehicle_state,
        t,
        model,
    );
    if !terminal_state_validate(
        &raw mut current,
        ::core::ptr::null_mut::<::core::ffi::c_char>(),
        0 as size_t,
    ) {
        return terminal_abort(
            g,
            b"MM305 native state could not be projected into the fixed runway frame.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if (*g).mm305_route_committed as ::core::ffi::c_int != 0
        && (*g).mm305_model_snapshot_id != (*model).snapshot_id
    {
        return terminal_abort(
            g,
            b"The native MM305 model snapshot changed after route commitment.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    mm305_observe_force_scale(g, t, model, &raw mut current, dt);
    let replan_interval_s: ::core::ffi::c_double = 20.0f64;
    let retry_interval_s: ::core::ffi::c_double = 2.0f64;
    if !(*g).mm305_route_committed && !(*cfg).guidance.taem_energy_law
        && !(*cfg).guidance.taem_hac_law
    {
        let mut aero_calibrated: bool = (*g).mm305_lift_scale.is_finite() as i32 != 0
            && (*g).mm305_lift_scale > 0.0f64
            && (*g).mm305_drag_scale.is_finite() as i32 != 0
            && (*g).mm305_drag_scale > 0.0f64;
        let mut backoff: ::core::ffi::c_double = retry_interval_s;
        let mut due: bool = aero_calibrated as ::core::ffi::c_int != 0
            && ((*g).mm305_last_plan_attempt_ut.is_finite() as i32 == 0
                || (*t).ut - (*g).mm305_last_plan_attempt_ut >= backoff);
        if due as ::core::ffi::c_int != 0 && !(*g).mm305_planning_needed {
            (*g).mm305_planning_needed = true_0 != 0;
            (*g).mm305_last_plan_attempt_ut = (*t).ut;
            (*g).mm305_plan_request_ut = (*t).ut;
        }
        if (*g).mm305_planning_needed as ::core::ffi::c_int != 0
            && !(*g).mm305_async_planning
        {
            let mut request: Mm305PlanRequest = guidance_mm305_plan_request(
                g,
                t,
                vehicle_state,
                cfg,
                model,
            );
            let mut result: Mm305PlanResult = mm305_plan(model, &raw mut request);
            if !guidance_mm305_accept_plan(g, &raw mut result, cfg) {
                (*g).mm305_planning_needed = false_0 != 0;
            }
        }
    }
    let mut end: ::core::ffi::c_int = if (*g).mm305_route_committed as ::core::ffi::c_int
        != 0 && (*g).runway_end_index == 1 as ::core::ffi::c_int
    {
        1 as ::core::ffi::c_int
    } else if (*g).runway_end_preview_valid as ::core::ffi::c_int != 0
        && (*g).runway_end_index == 1 as ::core::ffi::c_int
    {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
    let mut end_choice_open: bool = !(*g).taem_energy_law.initialized
        || (*g).taem_energy_law.leg == TEL_LEG_OUT as ::core::ffi::c_int
        || (*g).taem_energy_law.leg == TEL_LEG_DOWN as ::core::ffi::c_int;
    if (*g).taem_energy_law.initialized as ::core::ffi::c_int != 0
        && end_choice_open as ::core::ffi::c_int != 0
    {
        let mut active: *const TerminalModel = mm305_model_for_end(model, end);
        let mut choice_geo: TaemGeometryState = TaemGeometryState {
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
        end_choice_open = taem_geometry_state(
            active,
            &raw mut current,
            &raw mut choice_geo,
        );
        if end_choice_open {
            let mut choice_c: TelContext = TelContext {
                model: ::core::ptr::null::<TerminalModel>(),
                d_station: 0.,
                h_station: 0.,
                v_station: 0.,
                e_station: 0.,
                aoa_max: 0.,
                slope_deg: 0.,
                final_line: false,
            };
            let mut choice_s: TelPoint = TelPoint {
                x: 0.,
                y: 0.,
                h: 0.,
                v: 0.,
                g: 0.,
                psi: 0.,
                m: 0.,
                aoa: 0.,
                aoa_rate: 0.,
                bank: 0.,
                ground_speed: 0.,
            };
            tel_setup(
                active,
                &raw mut choice_geo,
                current.mass_kg,
                (*t).angle_of_attack,
                (*t).roll,
                &raw mut choice_c,
                &raw mut choice_s,
            );
            end_choice_open = tel_plan_open(
                &raw mut (*g).taem_energy_law,
                &raw mut choice_c,
                &raw mut choice_s,
            ) as ::core::ffi::c_int != 0
                && choice_s.h + choice_s.v * choice_s.v / (2.0f64 * TEL_G0)
                    > choice_c.e_station;
        }
    }
    if (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0
        && !(*g).mm305_route_committed
        && (*cfg).site.allow_reciprocal_runway as ::core::ffi::c_int != 0
    {
        let mut H: *mut TaemHacLawState = &raw mut (*g).taem_hac_law;
        let mut open: bool = !(*H).initialized
            || !(*H).drag_flown && (*H).phase <= TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int
                && (*H).tangent_dist > HAC_SELECT_FREEZE_M;
        if open as ::core::ffi::c_int != 0
            && (!(*g).taem_runway_end_scored
                || (*t).ut - (*g).taem_runway_end_score_ut >= HAC_UPDATE_S)
        {
            let mut first_choice: bool = !(*g).taem_runway_end_scored;
            (*g).taem_runway_end_scored = true_0 != 0;
            (*g).taem_runway_end_score_ut = (*t).ut;
            let mut score: [::core::ffi::c_double; 2] = [
                ::core::f32::INFINITY as ::core::ffi::c_double,
                ::core::f32::INFINITY as ::core::ffi::c_double,
            ];
            let mut feasible: [bool; 2] = [false_0 != 0, false_0 != 0];
            let mut approach_side: [bool; 2] = [false_0 != 0, false_0 != 0];
            let mut e: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while e < 2 as ::core::ffi::c_int {
                let mut trial: TerminalModel = TerminalModel {
                    schema_version: 0,
                    snapshot_id: 0,
                    atmosphere_source_digest: 0,
                    aero_source_digest: 0,
                    aero_book_source_digest: 0,
                    attitude_source_digest: 0,
                    replay_validated: false,
                    planet_frame_aligned: false,
                    world: KerbinWorld {
                        radius_m: 0.,
                        mu_m3_s2: 0.,
                        rotation_rate_rad_s: 0.,
                        atmosphere_top_m: 0.,
                        rotation_phase_rad_at_ut0: 0.,
                        orbital_rate_rad_s: 0.,
                        solar_phase_rad_at_ut0: 0.,
                        stock_spatial_atmosphere: false,
                        atmosphere: AtmosphereTable {
                            p: [AtmospherePoint {
                                altitude_m: 0.,
                                sample: AtmosphereSample {
                                    density_kg_m3: 0.,
                                    pressure_pa: 0.,
                                    temperature_k: 0.,
                                    speed_of_sound_mps: 0.,
                                },
                            }; 2048],
                            count: 0,
                        },
                    },
                    aero: AeroTable {
                        mach: [0.; 40],
                        alpha_deg: [0.; 40],
                        cl: [[0.; 40]; 40],
                        cd: [[0.; 40]; 40],
                        mach_count: 0,
                        alpha_count: 0,
                        reference_area_m2: 0.,
                        book: [AeroBookPoint {
                            q_pa: 0.,
                            mach: 0.,
                            alpha_deg: 0.,
                            lift_per_q_m2: 0.,
                            drag_per_q_m2: 0.,
                            support: 0.,
                        }; 2048],
                        book_count: 0,
                        book_enabled: false,
                        lift_scale: 0.,
                        drag_scale: 0.,
                    },
                    attitude: AttitudeModel {
                        aoa_rad: 0.,
                        bank_rad: 0.,
                        aoa_rate_rad_s: 0.,
                        bank_rate_rad_s: 0.,
                        requested_aoa_rad: 0.,
                        requested_bank_rad: 0.,
                        cmd_aoa_rad: 0.,
                        cmd_bank_rad: 0.,
                        pitch_wn: 0.,
                        pitch_zeta: 0.,
                        roll_wn: 0.,
                        roll_zeta: 0.,
                        max_pitch_rate_rad_s: 0.,
                        max_roll_rate_rad_s: 0.,
                        max_pitch_accel_rad_s2: 0.,
                        max_roll_accel_rad_s2: 0.,
                        pitch_full_authority_q_pa: 0.,
                        roll_full_authority_q_pa: 0.,
                        direct: false,
                        direct_body_coordinates: false,
                        body_rates_initialized: false,
                        body_pitch_rate_rad_s: 0.,
                        body_roll_rate_rad_s: 0.,
                        body_yaw_rate_rad_s: 0.,
                        input_pitch: 0.,
                        input_roll: 0.,
                        input_yaw: 0.,
                        sideslip_rad: 0.,
                        sideslip_rate_rad_s: 0.,
                        direct_min_q_pa: 0.,
                        direct_pitch_accel_per_kpa: 0.,
                        direct_pitch_accel_max: 0.,
                        direct_pitch_stiffness_per_kpa: 0.,
                        direct_trim_aoa_rad: 0.,
                        direct_pitch_damping_ratio: 0.,
                        direct_roll_accel_per_kpa: 0.,
                        direct_roll_accel_max: 0.,
                        direct_roll_damping_s_inv: 0.,
                        direct_yaw_accel_per_kpa: 0.,
                        direct_yaw_accel_max: 0.,
                        direct_yaw_stiffness_per_kpa: 0.,
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
                };
                let mut trial_end: *const TerminalModel = mm305_model_for_end(model, e);
                mm305_prepare_planning_model(
                    &raw mut trial,
                    trial_end,
                    mm305_final_alignment_distance(g, trial_end, &raw mut current),
                    (*t).mach,
                    1.0f64,
                    1.0f64,
                    current.trim_aoa_ceiling_rad,
                );
                let mut trial_geometry: TaemGeometryState = TaemGeometryState {
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
                if taem_geometry_state(
                    &raw mut trial,
                    &raw mut current,
                    &raw mut trial_geometry,
                ) {
                    approach_side[e as usize] = trial_geometry.runway_along_m < 0.0f64;
                    score[e as usize] = taem_hac_law_frame_score(
                        &raw mut trial,
                        &raw const (*model).aero,
                        &raw mut (*g).taem_hac_live_aero,
                        &raw mut trial_geometry,
                        current.mass_kg,
                        (*t).angle_of_attack,
                        (*t).roll,
                        (&raw mut feasible as *mut bool).offset(e as isize) as *mut bool,
                    );
                }
                e += 1;
            }
            let mut other: ::core::ffi::c_int = 1 as ::core::ffi::c_int - end;
            if first_choice as ::core::ffi::c_int != 0 && (*t).mach > HAC_DRAG_EXIT_MACH
                && approach_side[end as usize] as ::core::ffi::c_int != 0
            {
                other = end;
            }
            if if first_choice as ::core::ffi::c_int != 0 {
                (score[other as usize] < score[end as usize]) as ::core::ffi::c_int
            } else {
                (!feasible[end as usize]
                    && feasible[other as usize] as ::core::ffi::c_int != 0
                    && score[other as usize] < score[end as usize]) as ::core::ffi::c_int
            } != 0
            {
                end = other;
            }
            let mut changed: bool = end != (*g).runway_end_index;
            if changed as ::core::ffi::c_int != 0
                && (*H).initialized as ::core::ffi::c_int != 0
            {
                *H = TaemHacLawState {
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
                (*g).terminal_final_handoff_latched = false_0 != 0;
            }
            (*g).runway_end_index = end;
            (*g).runway_end_preview_valid = true_0 != 0;
            (*g).runway_end_committed = true_0 != 0;
            if !(*g).diagnostic_shadow
                && (first_choice as ::core::ffi::c_int != 0
                    || changed as ::core::ffi::c_int != 0)
            {
                fprintf(
                    stderr,
                    b"MM305 HAC runway end: UT %.1f RW%s end=%d cost09=%.0f cost27=%.0f\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*t).ut,
                    if end == 1 as ::core::ffi::c_int {
                        b"27\0" as *const u8 as *const ::core::ffi::c_char
                    } else {
                        b"09\0" as *const u8 as *const ::core::ffi::c_char
                    },
                    end,
                    score[0 as ::core::ffi::c_int as usize],
                    score[1 as ::core::ffi::c_int as usize],
                );
            }
        }
    }
    if (*cfg).guidance.taem_energy_law as ::core::ffi::c_int != 0
        && !(*cfg).guidance.taem_hac_law && !(*g).mm305_route_committed
        && (*cfg).site.allow_reciprocal_runway as ::core::ffi::c_int != 0
        && (if (*cfg).guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0 {
            end_choice_open as ::core::ffi::c_int
        } else {
            (!(*g).taem_energy_law.initialized && !(*g).taem_runway_end_scored)
                as ::core::ffi::c_int
        }) != 0
        && (!(*g).taem_runway_end_scored
            || (*t).ut - (*g).taem_runway_end_score_ut >= TEL_UPDATE_S)
    {
        let mut first_choice_0: bool = !(*g).taem_runway_end_scored;
        (*g).taem_runway_end_scored = true_0 != 0;
        (*g).taem_runway_end_score_ut = (*t).ut;
        let mut score_0: [::core::ffi::c_double; 2] = [
            ::core::f32::INFINITY as ::core::ffi::c_double,
            ::core::f32::INFINITY as ::core::ffi::c_double,
        ];
        let mut first_bank: [::core::ffi::c_double; 2] = [
            ::core::f32::NAN as ::core::ffi::c_double,
            ::core::f32::NAN as ::core::ffi::c_double,
        ];
        let mut delivery_feasible: [bool; 2] = [false_0 != 0, false_0 != 0];
        let mut e_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while e_0 < 2 as ::core::ffi::c_int {
            let mut trial_0: TerminalModel = TerminalModel {
                schema_version: 0,
                snapshot_id: 0,
                atmosphere_source_digest: 0,
                aero_source_digest: 0,
                aero_book_source_digest: 0,
                attitude_source_digest: 0,
                replay_validated: false,
                planet_frame_aligned: false,
                world: KerbinWorld {
                    radius_m: 0.,
                    mu_m3_s2: 0.,
                    rotation_rate_rad_s: 0.,
                    atmosphere_top_m: 0.,
                    rotation_phase_rad_at_ut0: 0.,
                    orbital_rate_rad_s: 0.,
                    solar_phase_rad_at_ut0: 0.,
                    stock_spatial_atmosphere: false,
                    atmosphere: AtmosphereTable {
                        p: [AtmospherePoint {
                            altitude_m: 0.,
                            sample: AtmosphereSample {
                                density_kg_m3: 0.,
                                pressure_pa: 0.,
                                temperature_k: 0.,
                                speed_of_sound_mps: 0.,
                            },
                        }; 2048],
                        count: 0,
                    },
                },
                aero: AeroTable {
                    mach: [0.; 40],
                    alpha_deg: [0.; 40],
                    cl: [[0.; 40]; 40],
                    cd: [[0.; 40]; 40],
                    mach_count: 0,
                    alpha_count: 0,
                    reference_area_m2: 0.,
                    book: [AeroBookPoint {
                        q_pa: 0.,
                        mach: 0.,
                        alpha_deg: 0.,
                        lift_per_q_m2: 0.,
                        drag_per_q_m2: 0.,
                        support: 0.,
                    }; 2048],
                    book_count: 0,
                    book_enabled: false,
                    lift_scale: 0.,
                    drag_scale: 0.,
                },
                attitude: AttitudeModel {
                    aoa_rad: 0.,
                    bank_rad: 0.,
                    aoa_rate_rad_s: 0.,
                    bank_rate_rad_s: 0.,
                    requested_aoa_rad: 0.,
                    requested_bank_rad: 0.,
                    cmd_aoa_rad: 0.,
                    cmd_bank_rad: 0.,
                    pitch_wn: 0.,
                    pitch_zeta: 0.,
                    roll_wn: 0.,
                    roll_zeta: 0.,
                    max_pitch_rate_rad_s: 0.,
                    max_roll_rate_rad_s: 0.,
                    max_pitch_accel_rad_s2: 0.,
                    max_roll_accel_rad_s2: 0.,
                    pitch_full_authority_q_pa: 0.,
                    roll_full_authority_q_pa: 0.,
                    direct: false,
                    direct_body_coordinates: false,
                    body_rates_initialized: false,
                    body_pitch_rate_rad_s: 0.,
                    body_roll_rate_rad_s: 0.,
                    body_yaw_rate_rad_s: 0.,
                    input_pitch: 0.,
                    input_roll: 0.,
                    input_yaw: 0.,
                    sideslip_rad: 0.,
                    sideslip_rate_rad_s: 0.,
                    direct_min_q_pa: 0.,
                    direct_pitch_accel_per_kpa: 0.,
                    direct_pitch_accel_max: 0.,
                    direct_pitch_stiffness_per_kpa: 0.,
                    direct_trim_aoa_rad: 0.,
                    direct_pitch_damping_ratio: 0.,
                    direct_roll_accel_per_kpa: 0.,
                    direct_roll_accel_max: 0.,
                    direct_roll_damping_s_inv: 0.,
                    direct_yaw_accel_per_kpa: 0.,
                    direct_yaw_accel_max: 0.,
                    direct_yaw_stiffness_per_kpa: 0.,
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
            };
            let mut trial_end_0: *const TerminalModel = mm305_model_for_end(model, e_0);
            mm305_prepare_planning_model(
                &raw mut trial_0,
                trial_end_0,
                mm305_final_alignment_distance(g, trial_end_0, &raw mut current),
                (*t).mach,
                if (*g).mm305_lift_scale.is_finite() as i32 != 0
                    && (*g).mm305_lift_scale > 0.0f64
                {
                    (*g).mm305_lift_scale
                } else {
                    1.0f64
                },
                if (*g).mm305_drag_scale.is_finite() as i32 != 0
                    && (*g).mm305_drag_scale > 0.0f64
                {
                    (*g).mm305_drag_scale
                } else {
                    1.0f64
                },
                current.trim_aoa_ceiling_rad,
            );
            let mut trial_geometry_0: TaemGeometryState = TaemGeometryState {
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
            if taem_geometry_state(
                &raw mut trial_0,
                &raw mut current,
                &raw mut trial_geometry_0,
            ) {
                score_0[e_0 as usize] = taem_energy_law_frame_score(
                    &raw mut trial_0,
                    &raw mut trial_geometry_0,
                    current.mass_kg,
                    (*t).angle_of_attack,
                    (*t).roll,
                    (&raw mut first_bank as *mut ::core::ffi::c_double)
                        .offset(e_0 as isize) as *mut ::core::ffi::c_double,
                    (&raw mut delivery_feasible as *mut bool).offset(e_0 as isize)
                        as *mut bool,
                );
            }
            e_0 += 1;
        }
        if score_0[0 as ::core::ffi::c_int as usize].is_finite() as i32 != 0
            || score_0[1 as ::core::ffi::c_int as usize].is_finite() as i32 != 0
        {
            let mut other_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int - end;
            let mut margin: ::core::ffi::c_double = if first_choice_0
                as ::core::ffi::c_int != 0
            {
                0.0f64
            } else {
                TEL_REPLAN_MARGIN
            };
            if !first_choice_0 && first_bank[other_0 as usize].is_finite() as i32 != 0 {
                let mut limits: GuidanceAttitudeLimits = decision_guidance_attitude_limits(
                    t,
                    &raw const (*cfg).guidance,
                    PHASE_TAEM,
                    false_0 != 0,
                );
                let mut capture: ::core::ffi::c_double = decision_axis_capture_time(
                    first_bank[other_0 as usize] - (*t).roll,
                    (*t).roll_rate,
                    limits.accel_deg_s2,
                    limits.rate_deg_s,
                );
                if capture.is_finite() as i32 != 0 {
                    margin += (*t).horizontal_speed * capture;
                }
            }
            if (first_choice_0 as ::core::ffi::c_int != 0
                || delivery_feasible[other_0 as usize] as ::core::ffi::c_int != 0)
                && score_0[other_0 as usize] < score_0[end as usize] - margin
            {
                end = other_0;
            }
            let mut changed_0: bool = end != (*g).runway_end_index;
            if changed_0 as ::core::ffi::c_int != 0
                && (*g).taem_energy_law.initialized as ::core::ffi::c_int != 0
            {
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
                (*g).terminal_final_handoff_latched = false_0 != 0;
            }
            (*g).runway_end_index = end;
            (*g).runway_end_preview_valid = true_0 != 0;
            (*g).runway_end_committed = true_0 != 0;
            if !(*g).diagnostic_shadow
                && (first_choice_0 as ::core::ffi::c_int != 0
                    || changed_0 as ::core::ffi::c_int != 0)
            {
                fprintf(
                    stderr,
                    b"MM305 energy-law runway end: UT %.1f RW%s end=%d score09=%.0f score27=%.0f\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*t).ut,
                    if end == 1 as ::core::ffi::c_int {
                        b"27\0" as *const u8 as *const ::core::ffi::c_char
                    } else {
                        b"09\0" as *const u8 as *const ::core::ffi::c_char
                    },
                    end,
                    score_0[0 as ::core::ffi::c_int as usize],
                    score_0[1 as ::core::ffi::c_int as usize],
                );
            }
        }
    }
    let mut end_model: *const TerminalModel = mm305_model_for_end(model, end);
    let mut live_model: TerminalModel = TerminalModel {
        schema_version: 0,
        snapshot_id: 0,
        atmosphere_source_digest: 0,
        aero_source_digest: 0,
        aero_book_source_digest: 0,
        attitude_source_digest: 0,
        replay_validated: false,
        planet_frame_aligned: false,
        world: KerbinWorld {
            radius_m: 0.,
            mu_m3_s2: 0.,
            rotation_rate_rad_s: 0.,
            atmosphere_top_m: 0.,
            rotation_phase_rad_at_ut0: 0.,
            orbital_rate_rad_s: 0.,
            solar_phase_rad_at_ut0: 0.,
            stock_spatial_atmosphere: false,
            atmosphere: AtmosphereTable {
                p: [AtmospherePoint {
                    altitude_m: 0.,
                    sample: AtmosphereSample {
                        density_kg_m3: 0.,
                        pressure_pa: 0.,
                        temperature_k: 0.,
                        speed_of_sound_mps: 0.,
                    },
                }; 2048],
                count: 0,
            },
        },
        aero: AeroTable {
            mach: [0.; 40],
            alpha_deg: [0.; 40],
            cl: [[0.; 40]; 40],
            cd: [[0.; 40]; 40],
            mach_count: 0,
            alpha_count: 0,
            reference_area_m2: 0.,
            book: [AeroBookPoint {
                q_pa: 0.,
                mach: 0.,
                alpha_deg: 0.,
                lift_per_q_m2: 0.,
                drag_per_q_m2: 0.,
                support: 0.,
            }; 2048],
            book_count: 0,
            book_enabled: false,
            lift_scale: 0.,
            drag_scale: 0.,
        },
        attitude: AttitudeModel {
            aoa_rad: 0.,
            bank_rad: 0.,
            aoa_rate_rad_s: 0.,
            bank_rate_rad_s: 0.,
            requested_aoa_rad: 0.,
            requested_bank_rad: 0.,
            cmd_aoa_rad: 0.,
            cmd_bank_rad: 0.,
            pitch_wn: 0.,
            pitch_zeta: 0.,
            roll_wn: 0.,
            roll_zeta: 0.,
            max_pitch_rate_rad_s: 0.,
            max_roll_rate_rad_s: 0.,
            max_pitch_accel_rad_s2: 0.,
            max_roll_accel_rad_s2: 0.,
            pitch_full_authority_q_pa: 0.,
            roll_full_authority_q_pa: 0.,
            direct: false,
            direct_body_coordinates: false,
            body_rates_initialized: false,
            body_pitch_rate_rad_s: 0.,
            body_roll_rate_rad_s: 0.,
            body_yaw_rate_rad_s: 0.,
            input_pitch: 0.,
            input_roll: 0.,
            input_yaw: 0.,
            sideslip_rad: 0.,
            sideslip_rate_rad_s: 0.,
            direct_min_q_pa: 0.,
            direct_pitch_accel_per_kpa: 0.,
            direct_pitch_accel_max: 0.,
            direct_pitch_stiffness_per_kpa: 0.,
            direct_trim_aoa_rad: 0.,
            direct_pitch_damping_ratio: 0.,
            direct_roll_accel_per_kpa: 0.,
            direct_roll_accel_max: 0.,
            direct_roll_damping_s_inv: 0.,
            direct_yaw_accel_per_kpa: 0.,
            direct_yaw_accel_max: 0.,
            direct_yaw_stiffness_per_kpa: 0.,
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
    };
    let mut live_lift_scale: ::core::ffi::c_double = if (*g).mm305_lift_scale.is_finite()
        as i32 != 0 && (*g).mm305_lift_scale > 0.0f64
    {
        (*g).mm305_lift_scale
    } else {
        1.0f64
    };
    let mut live_drag_scale: ::core::ffi::c_double = if (*g).mm305_drag_scale.is_finite()
        as i32 != 0 && (*g).mm305_drag_scale > 0.0f64
    {
        (*g).mm305_drag_scale
    } else {
        1.0f64
    };
    let mut live_final_distance: ::core::ffi::c_double = mm305_final_alignment_distance(
        g,
        end_model,
        &raw mut current,
    );
    mm305_prepare_planning_model(
        &raw mut live_model,
        end_model,
        live_final_distance,
        (*t).mach,
        live_lift_scale,
        live_drag_scale,
        current.trim_aoa_ceiling_rad,
    );
    if (*g).terminal_final_handoff_latched {
        if (*g).terminal_final_handoff_slope_deg.is_finite() as i32 != 0
            && (*g).terminal_final_handoff_slope_deg >= 3.0f64
            && (*g).terminal_final_handoff_slope_deg <= 45.0f64
        {
            live_model.guidance.final_glide_slope = (*g)
                .terminal_final_handoff_slope_deg;
        }
        if (*g).terminal_final_handoff_speed_mps.is_finite() as i32 != 0
            && (*g).terminal_final_handoff_speed_mps > 0.0f64
        {
            live_model.guidance.final_alignment_speed = (*g)
                .terminal_final_handoff_speed_mps;
        }
    }
    let mut control_model: *const TerminalModel = &raw mut live_model;
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
    if !taem_geometry_state(control_model, &raw mut current, &raw mut geometry) {
        return terminal_abort(
            g,
            b"MM305 native state could not be projected into the fixed runway frame.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if geometry.altitude_above_runway_m <= 0.0f64 {
        return terminal_abort(
            g,
            b"MM305 crossed runway elevation before reaching the fixed-HAC exit.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0
        && !(*g).mm305_route_committed
    {
        return taem_hac_law_guidance(
            g,
            t,
            course,
            planet,
            aero,
            cfg,
            control_model,
            &raw const (*model).aero,
            &raw mut geometry,
            &raw mut current,
            dt,
        );
    }
    if (*cfg).guidance.taem_energy_law as ::core::ffi::c_int != 0
        && !(*g).mm305_route_committed
    {
        return taem_energy_law_guidance(
            g,
            t,
            course,
            planet,
            aero,
            cfg,
            control_model,
            &raw mut geometry,
            &raw mut current,
            dt,
        );
    }
    let glide_speed_floor_mps: ::core::ffi::c_double = MM305_MINIMUM_GLIDE_SPEED_MPS;
    let mut direct_range: ::core::ffi::c_double = hypot(
        geometry.runway_along_m,
        geometry.runway_cross_m,
    );
    let mut handoff_distance: ::core::ffi::c_double = fmax(
        500.0f64,
        (*control_model).guidance.final_approach_distance,
    );
    let mut handoff_speed: ::core::ffi::c_double = fmax(
        (*control_model).guidance.final_alignment_speed,
        fmax(
            glide_speed_floor_mps,
            fmax(
                (*control_model).vehicle.minimum_safe_speed,
                (*control_model).vehicle.touchdown_speed,
            ),
        ),
    );
    let mut direct_slope: ::core::ffi::c_double = atan2(
        fmax(0.0f64, geometry.altitude_above_runway_m),
        fmax(1.0f64, direct_range),
    ) * RAD2DEG;
    let mut direct_capture: RunwayCaptureEnvelope = decision_runway_capture_envelope(
        g,
        t,
        course,
        planet,
        cfg,
    );
    let mut inside_final_station: bool = geometry.runway_along_m >= -handoff_distance;
    let mut direct_window: bool = terminal_direct_runway_fallback_window(
        g,
        geometry.runway_along_m,
        geometry.ground_speed_mps,
        handoff_distance,
        &raw mut direct_capture,
    );
    let mut preemptive_direct_window: bool = direct_window as ::core::ffi::c_int != 0
        && !inside_final_station;
    let mut local_gravity: ::core::ffi::c_double = (*control_model).world.mu_m3_s2
        / pow(
            (*control_model).world.radius_m + (*control_model).site.altitude
                + geometry.altitude_above_runway_m,
            2.0f64,
        );
    let mut direct_line: HACGuidance = HACGuidance {
        heading: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        bank: 0.,
        desired_altitude: 0.,
        distance_final: 0.,
        arc_remaining: 0.,
        radial_error: 0.,
        course_error: 0.,
        angle: 0.,
        lateral_acceleration: 0.,
    };
    let mut direct_target_aoa: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut direct_glide_aoa: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut direct_lateral_aoa: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut projected_runway_speed: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut direct_projection_valid: bool = false_0 != 0;
    if direct_window as ::core::ffi::c_int != 0
        || (*g).mm305_direct_approach_degraded as ::core::ffi::c_int != 0
    {
        direct_line = terminal_runway_line_path_guidance(
            t,
            &raw const (*cfg).site,
            &raw const (*cfg).guidance,
            &raw const (*cfg).vehicle,
            local_gravity,
            course,
        );
        direct_target_aoa = terminal_direct_runway_target_aoa(
            t,
            planet,
            aero,
            &raw const (*cfg).vehicle,
            -direct_slope,
            direct_line.lateral_acceleration,
            direct_line.bank,
            &raw mut direct_glide_aoa,
            &raw mut direct_lateral_aoa,
        );
        if direct_target_aoa.is_finite() as i32 == 0 {
            direct_target_aoa = aerodynamic_best_glide_aoa(
                (*t).mach,
                &raw const (*cfg).vehicle,
            );
        }
        let mut projected_drag_work: ::core::ffi::c_double = terminal_projected_drag_work_state(
            g,
            t,
            planet,
            aero,
            &raw const (*cfg).vehicle,
            &raw const (*cfg).guidance,
            direct_target_aoa,
            direct_range,
            direct_slope,
            &raw mut projected_runway_speed,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
        direct_projection_valid = projected_drag_work.is_nan() as i32 == 0
            && projected_runway_speed.is_finite() as i32 != 0
            && projected_runway_speed >= 0.0f64;
    }
    let mut direct_energy_degraded: bool = inside_final_station as ::core::ffi::c_int
        != 0 && (!direct_projection_valid || projected_runway_speed < handoff_speed);
    let mut direct_degraded: bool = direct_energy_degraded as ::core::ffi::c_int != 0
        || preemptive_direct_window as ::core::ffi::c_int != 0;
    let mut direct_degradation_started: bool = direct_degraded as ::core::ffi::c_int != 0
        && !(*g).mm305_direct_approach_degraded;
    let mut best_effort_warning: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut best_effort_started: bool = false_0 != 0;
    if direct_degraded as ::core::ffi::c_int != 0
        && guidance_mm305_adopt_best_effort_route(g, model, geometry.airspeed_mps)
            as ::core::ffi::c_int != 0
    {
        best_effort_started = true_0 != 0;
        best_effort_warning = b"Native route qualification did not finish in the capture window; MM305 is holding its best route from the measured entry state. Final handoff still requires measured alignment and energy checks.\0"
            as *const u8 as *const ::core::ffi::c_char;
        if !(*g).diagnostic_shadow {
            fprintf(
                stderr,
                b"MM305 best-effort route held: UT=%.2f cursor=%zu/%zu along=%+.0f cross=%+.0f final=%+.0f radius=%.0f sweep=%.1f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                (*g).mm305_route_cursor,
                (*g).mm305_route.count,
                geometry.runway_along_m,
                geometry.runway_cross_m,
                (*g).mm305_route.alignment_along_m,
                (*g).mm305_route.hac.radius_m,
                (*g).mm305_route.hac.arc_sweep_rad * RAD2DEG,
            );
        }
    }
    if (*g).mm305_best_effort_route_active {
        best_effort_warning = b"Native route qualification was unavailable; MM305 is following its best measured-state route. Final handoff still requires measured alignment and energy checks.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if direct_degraded as ::core::ffi::c_int != 0 && !best_effort_started
        && !(*g).mm305_route_committed && !(*g).mm305_best_effort_route_active
    {
        (*g).mm305_direct_approach_degraded = true_0 != 0;
    }
    if (*g).mm305_direct_approach_degraded as ::core::ffi::c_int != 0
        && !(*g).mm305_best_effort_route_active
    {
        let mut line: HACGuidance = direct_line;
        let mut aoa: ::core::ffi::c_double = if direct_projection_valid
            as ::core::ffi::c_int != 0 && projected_runway_speed >= glide_speed_floor_mps
        {
            direct_target_aoa
        } else {
            fmax(direct_glide_aoa, direct_lateral_aoa)
        };
        if aoa.is_finite() as i32 == 0 {
            aoa = aerodynamic_best_glide_aoa((*t).mach, &raw const (*cfg).vehicle);
        }
        let mut direct_trace: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_DIRECT_TRACE\0" as *const u8 as *const ::core::ffi::c_char,
        );
        fn direct_trace_last_ut_tls() -> *mut ::core::ffi::c_double {
            ::std::thread_local! {
                static CELL: ::core::cell::UnsafeCell<::core::ffi::c_double> = const { ::core::cell::UnsafeCell::new(-::core::f32::INFINITY
            as ::core::ffi::c_double) };
            }
            CELL.with(|c| c.get())
        }
        if !direct_trace.is_null()
            && strcmp(direct_trace, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
            && ((*direct_trace_last_ut_tls()).is_finite() as i32 == 0
                || (*t).ut < (*direct_trace_last_ut_tls())
                || (*t).ut - (*direct_trace_last_ut_tls()) >= 5.0f64)
        {
            fprintf(
                stderr,
                b"TAEM_DIRECT_TRACE ut=%.2f along=%+.0f cross=%+.0f range=%.0f h=%.0f V=%.1f fpa=%.1f course=%.1f target=%.1f courseErr=%+.1f bankCmd=%+.1f latAccel=%+.1f roll=%+.1f rollRate=%+.1f aoa=%.1f aoaCmd=%.1f beta=%+.1f projectedV=%.1f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                geometry.runway_along_m,
                geometry.runway_cross_m,
                direct_range,
                geometry.altitude_above_runway_m,
                geometry.ground_speed_mps,
                (*t).flight_path_angle,
                course,
                line.heading,
                line.course_error,
                line.bank,
                line.lateral_acceleration,
                (*t).roll,
                (*t).roll_rate,
                (*t).angle_of_attack,
                aoa,
                (*t).sideslip,
                projected_runway_speed,
            );
            (*direct_trace_last_ut_tls()) = (*t).ut;
        }
        if direct_degradation_started as ::core::ffi::c_int != 0
            && !(*g).diagnostic_shadow
        {
            fprintf(
                stderr,
                b"TAEM direct capture opened: UT=%.2f along=%+.0f cross=%+.0f range=%.0f h=%.0f V=%.1f course=%.1f target=%.1f bank=%+.1f captureTime=%.1f headingTime=%.1f lateralTime=%.1f response=%.1f available=%.1f projectedV=%.1f slope=%.1f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).ut,
                geometry.runway_along_m,
                geometry.runway_cross_m,
                direct_range,
                geometry.altitude_above_runway_m,
                geometry.ground_speed_mps,
                course,
                line.heading,
                line.bank,
                fmax(
                    direct_capture.control_response_time_s,
                    fmax(
                        direct_capture.heading_capture_time_s,
                        direct_capture.lateral_capture_time_s,
                    ),
                ),
                direct_capture.heading_capture_time_s,
                direct_capture.lateral_capture_time_s,
                direct_capture.control_response_time_s,
                direct_capture.time_available_s,
                projected_runway_speed,
                direct_slope,
            );
        }
        let mut command: GuidanceCommand = atmospheric(
            t,
            line.heading,
            line.bank,
            &raw const (*cfg).vehicle,
            0.0f64,
            false_0 != 0,
            PROFILE_TAEM,
        );
        command.heading_control_enabled = false_0 != 0;
        command.hac_control_tuning = true_0 != 0;
        command.terminal_pitch_tuning = true_0 != 0;
        command.has_target_aoa = true_0 != 0;
        command.target_aoa = clampd(aoa, 0.0f64, (*cfg).vehicle.maximum_angle_of_attack);
        command.target_pitch = (*t).flight_path_angle + command.target_aoa;
        command.gear = (*t).gear as ::core::ffi::c_int != 0
            || (*t).radar_altitude <= (*cfg).guidance.gear_deployment_altitude;
        (*g).mm305_route_committed = false_0 != 0;
        (*g).terminal_path_committed = true_0 != 0;
        (*g).terminal_path_kind = TERMINAL_PATH_DIRECT;
        (*g).mm305_acquisition_route_valid = false_0 != 0;
        (*g).hac_captured = false_0 != 0;
        (*g).hac_side_selected = false_0 != 0;
        let mut final_plan: TerminalPreflarePlan = TerminalPreflarePlan {
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
        let mut final_approach: bool = terminal_outer_capture_admissible(
            g,
            t,
            course,
            planet,
            aero,
            cfg,
            &raw mut final_plan,
        );
        let mut final_contract: TaemTerminalContract = terminal_delivery_contract(
            g,
            t,
            course,
            planet,
            cfg,
            &raw mut final_plan,
        );
        let mut final_eval: TaemTerminalEvaluation = taem_exec_evaluate_terminal_contract(
            &raw mut final_contract,
        );
        if final_approach as ::core::ffi::c_int != 0
            && final_eval.valid as ::core::ffi::c_int != 0
            && final_eval.feasible as ::core::ffi::c_int != 0
        {
            (*g).taem_exec.terminal_contract = final_contract;
            (*g).taem_exec.terminal_evaluation = final_eval;
            (*g).terminal_predicted_touchdown_speed = terminal_delivery_touchdown_speed(
                g,
                t,
                planet,
                cfg,
                final_plan.trigger_altitude,
            );
            (*g).mm305_hac_exit_reached = true_0 != 0;
            (*g).hac_completed = true_0 != 0;
            (*g).hac_captured = true_0 != 0;
        }
        let mut direct_warning: [::core::ffi::c_char; 256] = [0; 256];
        if direct_degradation_started as ::core::ffi::c_int != 0
            && preemptive_direct_window as ::core::ffi::c_int != 0
        {
            snprintf(
                &raw mut direct_warning as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"No committed HAC route remains inside the measured runway-capture response window; selecting direct runway alignment as degraded guidance.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        } else if direct_degradation_started as ::core::ffi::c_int != 0
            && direct_projection_valid as ::core::ffi::c_int != 0
        {
            snprintf(
                &raw mut direct_warning as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"Inside the Final station, projected runway speed %.1f m/s is below the configured Final handoff speed %.1f m/s.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                projected_runway_speed,
                handoff_speed,
            );
        } else if direct_degradation_started {
            snprintf(
                &raw mut direct_warning as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"Inside the Final station, runway energy projection is unavailable; selecting the direct runway line as degraded guidance.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        } else {
            direct_warning[0 as ::core::ffi::c_int as usize] = '\0' as i32
                as ::core::ffi::c_char;
        }
        let mut direct_status: [::core::ffi::c_char; 256] = [0; 256];
        if direct_projection_valid {
            snprintf(
                &raw mut direct_status as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"%s; projected runway speed %.1f m/s, Final target %.1f m/s, glide floor %.1f m/s%s.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if preemptive_direct_window as ::core::ffi::c_int != 0 {
                    b"Unreachable-capture direct runway alignment\0" as *const u8
                        as *const ::core::ffi::c_char
                } else {
                    b"Energy-degraded direct runway alignment\0" as *const u8
                        as *const ::core::ffi::c_char
                },
                projected_runway_speed,
                handoff_speed,
                glide_speed_floor_mps,
                if projected_runway_speed < glide_speed_floor_mps {
                    b"; prioritizing live-model turn/glide incidence\0" as *const u8
                        as *const ::core::ffi::c_char
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
        } else {
            snprintf(
                &raw mut direct_status as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"%s; projection unavailable, using the live-model best-glide incidence with %.1f m/s glide floor.\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if preemptive_direct_window as ::core::ffi::c_int != 0 {
                    b"Unreachable-capture direct runway alignment\0" as *const u8
                        as *const ::core::ffi::c_char
                } else {
                    b"Energy-degraded direct runway alignment\0" as *const u8
                        as *const ::core::ffi::c_char
                },
                glide_speed_floor_mps,
            );
        }
        let mut degraded: GuidanceResult = stabilized(
            g,
            result_make(
                PHASE_TAEM,
                command,
                &raw mut direct_status as *mut ::core::ffi::c_char,
                &raw mut direct_warning as *mut ::core::ffi::c_char,
            ),
            t,
            &raw const (*cfg).vehicle,
            &raw const (*cfg).guidance,
            dt,
        );
        (*g).phase = PHASE_TAEM;
        return degraded;
    }
    let mut reference: TaemPathReference = TaemPathReference {
        runway_along_m: 0.,
        runway_cross_m: 0.,
        course_deg: 0.,
        curvature_right_per_m: 0.,
        altitude_m: 0.,
        flight_path_angle_deg: 0.,
        vertical_curvature_per_m: 0.,
        station_m: 0.,
    };
    let mut demand: TaemTrackerOutput = TaemTrackerOutput {
        valid: false,
        lateral_authority_ok: false,
        vertical_authority_ok: false,
        control: TerminalControl {
            angle_of_attack_rad: 0.,
            bank_rad: 0.,
            dt_s: 0.,
        },
        cross_track_error_m: 0.,
        course_error_deg: 0.,
        required_lateral_accel_mps2: 0.,
        available_lateral_accel_mps2: 0.,
        required_vertical_lift_mps2: 0.,
        delivered_vertical_lift_mps2: 0.,
        response_time_s: 0.,
    };
    let mut status: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    terminal_observe_landing_aero(
        g,
        t,
        &raw const (*cfg).vehicle,
        planet_surface_gravity(planet),
        dt,
    );
    if (*g).mm305_route_committed as ::core::ffi::c_int != 0
        && (*g).mm305_runway_alignment_active as ::core::ffi::c_int != 0
    {
        reference = taem_alignment_reference(control_model, &raw mut geometry);
        demand = taem_tracker_update(
            control_model,
            &raw mut current,
            &raw mut geometry,
            &raw mut reference,
            dt,
        );
        if !demand.valid {
            return terminal_abort(
                g,
                b"MM305 runway alignment could not produce a bounded control command.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        if taem_alignment_exhausted(control_model, &raw mut geometry) {
            return terminal_abort(
                g,
                b"MM305 missed the Final alignment point; late runway-line capture is not permitted.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        if taem_alignment_ready(control_model, &raw mut current, &raw mut geometry) {
            let mut final_plan_0: TerminalPreflarePlan = TerminalPreflarePlan {
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
            let mut final_approach_0: bool = terminal_outer_capture_admissible(
                g,
                t,
                course,
                planet,
                aero,
                cfg,
                &raw mut final_plan_0,
            );
            let mut final_contract_0: TaemTerminalContract = terminal_delivery_contract(
                g,
                t,
                course,
                planet,
                cfg,
                &raw mut final_plan_0,
            );
            let mut final_eval_0: TaemTerminalEvaluation = taem_exec_evaluate_terminal_contract(
                &raw mut final_contract_0,
            );
            (*g).taem_exec.terminal_contract = final_contract_0;
            (*g).taem_exec.terminal_evaluation = final_eval_0;
            (*g).terminal_predicted_touchdown_speed = terminal_delivery_touchdown_speed(
                g,
                t,
                planet,
                cfg,
                final_plan_0.trigger_altitude,
            );
            if final_approach_0 as ::core::ffi::c_int != 0
                && final_eval_0.valid as ::core::ffi::c_int != 0
                && final_eval_0.feasible as ::core::ffi::c_int != 0
            {
                fprintf(
                    stderr,
                    b"MM305_LIVE_EXIT model=%llu side=%+.0f radius=%.0f sweep=%.1f lead=%.0f cursor=%zu/%zu refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f cmdBank=%.2f cmdAoa=%.2f reason=final-contract-ready\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*g).mm305_model_snapshot_id as ::core::ffi::c_ulonglong,
                    (*g).mm305_route.side,
                    (*g).mm305_route.hac.radius_m,
                    (*g).mm305_route.hac.arc_sweep_rad * RAD2DEG,
                    (*g).mm305_route.lead_length_m,
                    (*g).mm305_route_cursor,
                    (*g).mm305_route.count,
                    demand.course_error_deg,
                    geometry.heading_error_deg,
                    geometry.runway_along_m,
                    geometry.runway_cross_m,
                    geometry.altitude_above_runway_m,
                    geometry.airspeed_mps,
                    geometry.flight_path_angle_deg,
                    current.attitude.bank_rad * RAD2DEG,
                    current.attitude.aoa_rad * RAD2DEG,
                    demand.control.bank_rad * RAD2DEG,
                    demand.control.angle_of_attack_rad * RAD2DEG,
                );
                (*g).mm305_hac_exit_reached = true_0 != 0;
                (*g).hac_completed = true_0 != 0;
                (*g).hac_remaining = 0.0f64;
            }
        }
        status = if (*g).mm305_hac_exit_reached as ::core::ffi::c_int != 0 {
            b"MM305 delivered a settled runway-line state to Final.\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"MM305 is settling bank and course on the runway line after HAC.\0"
                as *const u8 as *const ::core::ffi::c_char
        };
    } else if (*g).mm305_route_committed {
        if !taem_route_reference(
            &raw mut (*g).mm305_route,
            &raw mut geometry,
            &raw mut (*g).mm305_route_cursor,
            &raw mut reference,
            ::core::ptr::null_mut::<size_t>(),
        ) {
            return terminal_abort(
                g,
                b"Committed MM305 route could not produce a tracker reference.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        let mut route_remaining: ::core::ffi::c_double = taem_route_remaining_at_index(
            &raw mut (*g).mm305_route,
            (*g).mm305_route_cursor,
        );
        if route_remaining.is_finite() as i32 == 0 {
            return terminal_abort(
                g,
                b"Committed MM305 route station became non-finite.\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
        (*g).hac_remaining = route_remaining;
        (*g).hac_progress_valid = true_0 != 0;
        (*g).hac_captured = true_0 != 0;
        let mut tracker_reference: TaemPathReference = reference;
        let mut tracker_plan_speed: ::core::ffi::c_double = taem_route_planned_speed(
            &raw mut (*g).mm305_route,
            reference.station_m,
        );
        if tracker_plan_speed.is_finite() as i32 != 0 && tracker_plan_speed > 1.0f64
            && reference.altitude_m.is_finite() as i32 != 0
            && route_remaining > 3500.0f64
        {
            let mut dh: ::core::ffi::c_double = (*control_model).site.altitude
                + geometry.altitude_above_runway_m - reference.altitude_m;
            let mut equivalent: ::core::ffi::c_double = sqrt(
                fmax(
                    0.0f64,
                    geometry.airspeed_mps * geometry.airspeed_mps + 2.0f64 * 9.81f64 * dh,
                ),
            );
            let mut ratio: ::core::ffi::c_double = equivalent / tracker_plan_speed
                - 1.0f64;
            if ratio < -0.03f64 {
                let mut strength: ::core::ffi::c_double = clampd(
                    (-ratio - 0.03f64) / 0.15f64,
                    0.0f64,
                    1.0f64,
                );
                let mut fade: ::core::ffi::c_double = clampd(
                    (route_remaining - 3500.0f64) / 5000.0f64,
                    0.0f64,
                    1.0f64,
                );
                let mut deficit_height: ::core::ffi::c_double = fmax(
                    0.0f64,
                    (tracker_plan_speed * tracker_plan_speed - equivalent * equivalent)
                        / (2.0f64 * 9.81f64),
                );
                let mut altitude_relief: ::core::ffi::c_double = fmin(
                    700.0f64,
                    0.70f64 * deficit_height,
                ) * fade;
                tracker_reference.altitude_m -= altitude_relief;
                tracker_reference.flight_path_angle_deg = clampd(
                    reference.flight_path_angle_deg - 8.0f64 * strength * fade,
                    -35.0f64,
                    -2.0f64,
                );
                if !(*g).diagnostic_shadow {
                    let mut diag: *const ::core::ffi::c_char = getenv(
                        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8
                            as *const ::core::ffi::c_char,
                    );
                    if !diag.is_null()
                        && strcmp(
                            diag,
                            b"2\0" as *const u8 as *const ::core::ffi::c_char,
                        ) == 0 as ::core::ffi::c_int
                    {
                        fprintf(
                            stderr,
                            b"MM305 energy recovery: ratio=%+.1f%% remaining=%.0f fpa %.1f->%.1f altRelief=%.0f\n\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            100.0f64 * ratio,
                            route_remaining,
                            reference.flight_path_angle_deg,
                            tracker_reference.flight_path_angle_deg,
                            altitude_relief,
                        );
                    }
                }
            }
        }
        demand = taem_tracker_update(
            control_model,
            &raw mut current,
            &raw mut geometry,
            &raw mut tracker_reference,
            dt,
        );
        if !demand.valid {
            return terminal_abort(
                g,
                b"Live MM305 tracker could not produce a bounded control command.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        let mut emergency_cross_track_m: ::core::ffi::c_double = fmax(
            3000.0f64,
            0.25f64 * (*g).mm305_route.hac.radius_m,
        );
        if fabs(demand.cross_track_error_m) > emergency_cross_track_m
            || fabs(demand.course_error_deg) > 45.0f64
        {
            if geometry.altitude_above_runway_m < 1500.0f64 {
                return terminal_abort(
                    g,
                    b"Live MM305 tracking diverged below the height needed to re-plan.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                );
            }
            if !(*g).diagnostic_shadow {
                fprintf(
                    stderr,
                    b"MM305 route released: cross %.0f m course %.1f deg; re-planning.\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    demand.cross_track_error_m,
                    demand.course_error_deg,
                );
            }
            (*g).mm305_route_committed = false_0 != 0;
            (*g).mm305_best_effort_route_active = false_0 != 0;
            (*g).terminal_path_committed = false_0 != 0;
            (*g).hac_captured = false_0 != 0;
            (*g).mm305_acquisition_route_valid = false_0 != 0;
            (*g).mm305_acquisition_route_cursor = 0 as size_t;
            (*g).mm305_planning_needed = true_0 != 0;
            (*g).mm305_last_plan_attempt_ut = (*t).ut;
            (*g).mm305_plan_request_ut = (*t).ut;
        }
        let mut plan_speed_now: ::core::ffi::c_double = taem_route_planned_speed(
            &raw mut (*g).mm305_route,
            reference.station_m,
        );
        let mut energy_ratio: ::core::ffi::c_double = ::core::f32::NAN
            as ::core::ffi::c_double;
        if plan_speed_now.is_finite() as i32 != 0 && plan_speed_now > 1.0f64
            && reference.altitude_m.is_finite() as i32 != 0
        {
            let mut dh_0: ::core::ffi::c_double = (*control_model).site.altitude
                + geometry.altitude_above_runway_m - reference.altitude_m;
            energy_ratio = sqrt(
                fmax(
                    0.0f64,
                    geometry.airspeed_mps * geometry.airspeed_mps
                        + 2.0f64 * 9.81f64 * dh_0,
                ),
            ) / plan_speed_now - 1.0f64;
        }
        let mut energy_high: bool = energy_ratio.is_finite() as i32 != 0
            && energy_ratio > 0.10f64;
        let mut energy_low: bool = energy_ratio.is_finite() as i32 != 0
            && energy_ratio < -0.06f64;
        let mut energy_off_plan: bool = (energy_high as ::core::ffi::c_int != 0
            || energy_low as ::core::ffi::c_int != 0)
            && route_remaining.is_finite() as i32 != 0;
        let mut energy_replan_interval: ::core::ffi::c_double = if energy_low
            as ::core::ffi::c_int != 0
        {
            10.0f64
        } else {
            replan_interval_s
        };
        if (*g).mm305_route_committed as ::core::ffi::c_int != 0
            && !(*g).mm305_best_effort_route_active
            && (*g).mm305_async_planning as ::core::ffi::c_int != 0
            && !(*g).mm305_planning_needed
            && (*t).ut - (*g).mm305_last_plan_attempt_ut >= energy_replan_interval
            && (energy_off_plan as ::core::ffi::c_int != 0
                || fabs(demand.cross_track_error_m) > 1000.0f64
                || fabs(demand.course_error_deg) > 20.0f64
                || reference.flight_path_angle_deg.is_finite() as i32 != 0
                    && fabs(
                        geometry.flight_path_angle_deg - reference.flight_path_angle_deg,
                    ) > 6.0f64)
        {
            let mut remaining: ::core::ffi::c_double = taem_route_remaining_at_index(
                &raw mut (*g).mm305_route,
                (*g).mm305_route_cursor,
            );
            let mut minimum_remaining: ::core::ffi::c_double = if energy_low
                as ::core::ffi::c_int != 0
            {
                fmax(4500.0f64, 0.20f64 * (*g).mm305_route.hac.arc_length_m)
            } else {
                fmax(8000.0f64, 0.35f64 * (*g).mm305_route.hac.arc_length_m)
            };
            if remaining.is_finite() as i32 != 0 && remaining > minimum_remaining {
                if energy_off_plan as ::core::ffi::c_int != 0 && !(*g).diagnostic_shadow
                {
                    fprintf(
                        stderr,
                        b"MM305 energy off plan by %+.0f%% with %.0f m left; re-planning.\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        100.0f64 * energy_ratio,
                        remaining,
                    );
                }
                (*g).mm305_planning_needed = true_0 != 0;
                (*g).mm305_last_plan_attempt_ut = (*t).ut;
                (*g).mm305_plan_request_ut = (*t).ut;
            }
        }
        let mut exit_distance: ::core::ffi::c_double = hypot(
            geometry.runway_along_m - (*g).mm305_route.alignment_along_m,
            geometry.runway_cross_m,
        );
        let mut alignment_capture: ::core::ffi::c_double = taem_alignment_capture_distance(
            control_model,
            &raw mut geometry,
        );
        if (*g).mm305_route_committed as ::core::ffi::c_int != 0
            && route_remaining.is_finite() as i32 != 0
            && route_remaining <= alignment_capture
            && exit_distance <= alignment_capture + 500.0f64
            && fabs(demand.course_error_deg) <= 12.0f64
        {
            fprintf(
                stderr,
                b"MM305_ALIGNMENT_ROUTE_END model=%llu side=%+.0f radius=%.0f sweep=%.1f lead=%.0f cursor=%zu/%zu refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f cmdBank=%.2f cmdAoa=%.2f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*g).mm305_model_snapshot_id as ::core::ffi::c_ulonglong,
                (*g).mm305_route.side,
                (*g).mm305_route.hac.radius_m,
                (*g).mm305_route.hac.arc_sweep_rad * 180.0f64
                    / 3.14159265358979323846f64,
                (*g).mm305_route.lead_length_m,
                (*g).mm305_route_cursor,
                (*g).mm305_route.count,
                demand.course_error_deg,
                geometry.heading_error_deg,
                geometry.runway_along_m,
                geometry.runway_cross_m,
                geometry.altitude_above_runway_m,
                geometry.airspeed_mps,
                geometry.flight_path_angle_deg,
                current.attitude.bank_rad * RAD2DEG,
                current.attitude.aoa_rad * RAD2DEG,
                demand.control.bank_rad * RAD2DEG,
                demand.control.angle_of_attack_rad * RAD2DEG,
            );
            (*g).mm305_runway_alignment_active = true_0 != 0;
            (*g).hac_captured = true_0 != 0;
            (*g).hac_remaining = 0.0f64;
        }
        status = if (*g).mm305_runway_alignment_active as ::core::ffi::c_int != 0 {
            b"MM305 native route reached the Final alignment station; verifying settled handoff state.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else if (*g).mm305_best_effort_route_active as ::core::ffi::c_int != 0 {
            b"MM305 is smoothly tracking its best measured-state alignment route; native qualification was unavailable.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"MM305 native tracker is following the committed route through HAC and alignment rollout.\0"
                as *const u8 as *const ::core::ffi::c_char
        };
    }
    let mut acquisition_path_remaining: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut acquisition_required_drag: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut acquisition_available_drag: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    if !(*g).mm305_route_committed {
        let mut hac_radius: ::core::ffi::c_double = if (*g).hac_radius >= 3000.0f64 {
            (*g).hac_radius
        } else {
            (*cfg).guidance.hac_radius
        };
        if !mm305_acquisition_command(
            g,
            end_model,
            control_model,
            &raw mut current,
            &raw mut geometry,
            hac_radius,
            dt,
            &raw mut demand,
            &raw mut reference,
            &raw mut acquisition_path_remaining,
            &raw mut acquisition_required_drag,
            &raw mut acquisition_available_drag,
        ) {
            return terminal_abort(
                g,
                b"MM305 acquisition law could not produce a bounded control command.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        if geometry.altitude_above_runway_m < 1000.0f64
            && (*g).mm305_plan_failures > 0 as ::core::ffi::c_uint
        {
            return terminal_abort(
                g,
                b"MM305 found no qualified route before the height needed to fly one.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        status = b"MM305 acquisition: tracking the displayed provisional TAEM route while qualification runs.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    let mut diagnostics: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TAEM_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !diagnostics.is_null()
        && strcmp(diagnostics, b"2\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
    {
        fprintf(
            stderr,
            b"TAEM live trace: ceil=%.1f latReq=%.2f latAvail=%.2f ut=%.2f committed=%d idx=%zu along=%.0f cross=%.0f h=%.0f course=%.1f ref=%.1f kappa=%.3g xt=%.0f crsErr=%.1f fpa=%.1f refFpa=%.1f bank=%.1f cmdBank=%.1f aoa=%.1f cmdAoa=%.1f scales=%.3f/%.3f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            current.trim_aoa_ceiling_rad * RAD2DEG,
            demand.required_lateral_accel_mps2,
            demand.available_lateral_accel_mps2,
            current.ut_s,
            (*g).mm305_route_committed as ::core::ffi::c_int,
            (*g).mm305_route_cursor,
            geometry.runway_along_m,
            geometry.runway_cross_m,
            geometry.altitude_above_runway_m,
            geometry.course_deg,
            reference.course_deg,
            reference.curvature_right_per_m,
            demand.cross_track_error_m,
            demand.course_error_deg,
            geometry.flight_path_angle_deg,
            reference.flight_path_angle_deg,
            current.attitude.bank_rad * RAD2DEG,
            demand.control.bank_rad * RAD2DEG,
            current.attitude.aoa_rad * RAD2DEG,
            demand.control.angle_of_attack_rad * RAD2DEG,
            (*g).mm305_lift_scale,
            (*g).mm305_drag_scale,
        );
    }
    let mut command_0: GuidanceCommand = atmospheric(
        t,
        reference.course_deg,
        demand.control.bank_rad * RAD2DEG,
        &raw const (*cfg).vehicle,
        0.0f64,
        false_0 != 0,
        PROFILE_TAEM,
    );
    command_0.heading_control_enabled = false_0 != 0;
    command_0.has_target_aoa = true_0 != 0;
    command_0.target_aoa = demand.control.angle_of_attack_rad * RAD2DEG;
    command_0.target_pitch = (*t).flight_path_angle + command_0.target_aoa;
    let sb_cap: ::core::ffi::c_double = SPEEDBRAKE_CAP;
    let sb_acquisition_nominal: ::core::ffi::c_double = SPEEDBRAKE_NOMINAL;
    let sb_deploy_slew_per_s: ::core::ffi::c_double = SPEEDBRAKE_DEPLOY_SLEW;
    let sb_stow_slew_per_s: ::core::ffi::c_double = SPEEDBRAKE_STOW_SLEW;
    let sb_stow_remaining_m: ::core::ffi::c_double = 500.0f64;
    let sb_fade_m: ::core::ffi::c_double = 2500.0f64;
    let mut sb_dt: ::core::ffi::c_double = if (*g).taem_speedbrake_ut > 0.0f64
        && (*t).ut > (*g).taem_speedbrake_ut
    {
        fmin((*t).ut - (*g).taem_speedbrake_ut, 1.0f64)
    } else {
        0.0f64
    };
    (*g).taem_speedbrake_ut = (*t).ut;
    let mut planned_speed: ::core::ffi::c_double = if (*g).mm305_route_committed
        as ::core::ffi::c_int != 0
    {
        taem_route_planned_speed(&raw mut (*g).mm305_route, reference.station_m)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut sb_remaining: ::core::ffi::c_double = if (*g).mm305_route_committed
        as ::core::ffi::c_int != 0
    {
        taem_route_remaining_at_index(&raw mut (*g).mm305_route, (*g).mm305_route_cursor)
    } else {
        acquisition_path_remaining
    };
    let mut sb_geometric_fade: ::core::ffi::c_double = if sb_remaining.is_finite() as i32
        != 0
    {
        clampd((sb_remaining - sb_stow_remaining_m) / sb_fade_m, 0.0f64, 1.0f64)
    } else {
        0.0f64
    };
    let mut sb_target: ::core::ffi::c_double = 0.0f64;
    let mut sb_limit: ::core::ffi::c_double = sb_cap * sb_geometric_fade;
    if (*g).mm305_route_committed as ::core::ffi::c_int != 0
        && planned_speed.is_finite() as i32 != 0 && planned_speed > 1.0f64
        && reference.altitude_m.is_finite() as i32 != 0
    {
        let mut surplus_height: ::core::ffi::c_double = (*control_model).site.altitude
            + geometry.altitude_above_runway_m - reference.altitude_m;
        let mut equivalent_0: ::core::ffi::c_double = sqrt(
            fmax(
                0.0f64,
                geometry.airspeed_mps * geometry.airspeed_mps
                    + 2.0f64 * 9.81f64 * surplus_height,
            ),
        );
        let mut ratio_0: ::core::ffi::c_double = equivalent_0 / planned_speed - 1.0f64;
        let mut extended_final: ::core::ffi::c_double = clampd(
            ((*control_model).guidance.final_approach_distance - 8000.0f64) / 2000.0f64,
            0.0f64,
            1.0f64,
        );
        let mut committed_cap: ::core::ffi::c_double = sb_cap
            * (1.0f64 - 0.40f64 * extended_final);
        let mut energy_deadband: ::core::ffi::c_double = 0.01f64
            + 0.03f64 * extended_final;
        let mut brake_error: ::core::ffi::c_double = fmax(
            0.0f64,
            ratio_0 - energy_deadband,
        );
        let mut proportional_gain: ::core::ffi::c_double = 2.5f64
            - 0.8f64 * extended_final;
        let mut integral_gain: ::core::ffi::c_double = 0.04f64
            * (1.0f64 - 0.50f64 * extended_final);
        sb_limit = committed_cap * sb_geometric_fade;
        if brake_error > 0.0f64 {
            (*g).taem_speedbrake_integral = clampd(
                (*g).taem_speedbrake_integral + integral_gain * brake_error * sb_dt,
                0.0f64,
                sb_limit,
            );
        } else {
            (*g).taem_speedbrake_integral = fmax(
                0.0f64,
                (*g).taem_speedbrake_integral - 0.14f64 * sb_dt,
            );
        }
        sb_target = clampd(
            proportional_gain * brake_error + (*g).taem_speedbrake_integral,
            0.0f64,
            sb_limit,
        );
        if !diagnostics.is_null()
            && strcmp(diagnostics, b"2\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
        {
            fprintf(
                stderr,
                b"TAEM speedbrake: V=%.1f Veq=%.1f plan=%.1f dh=%.0f limit=%.2f target=%.2f actual=%.2f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                geometry.airspeed_mps,
                equivalent_0,
                planned_speed,
                surplus_height,
                sb_limit,
                sb_target,
                (*g).taem_speedbrake_fraction,
            );
        }
    } else if !(*g).mm305_route_committed
        && acquisition_required_drag.is_finite() as i32 != 0
        && acquisition_available_drag.is_finite() as i32 != 0
    {
        let mut ratio_1: ::core::ffi::c_double = acquisition_required_drag
            / fmax(acquisition_available_drag, 0.25f64) - 1.0f64;
        let mut energy_high_0: ::core::ffi::c_double = clampd(
            ratio_1 / 0.15f64,
            0.0f64,
            1.0f64,
        );
        sb_limit = sb_cap;
        let mut sb_base: ::core::ffi::c_double = sb_acquisition_nominal * energy_high_0;
        (*g).taem_speedbrake_integral = clampd(
            (*g).taem_speedbrake_integral + 0.04f64 * ratio_1 * sb_dt,
            -sb_base,
            sb_limit - sb_base,
        );
        sb_target = clampd(
            sb_base + 0.35f64 * ratio_1 + (*g).taem_speedbrake_integral,
            0.0f64,
            sb_limit,
        );
        if !diagnostics.is_null()
            && strcmp(diagnostics, b"2\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int && !(*g).diagnostic_shadow
        {
            fprintf(
                stderr,
                b"TAEM acquisition speedbrake: reqD=%.2f availD=%.2f path=%.0f ratio=%+.2f target=%.2f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                acquisition_required_drag,
                acquisition_available_drag,
                acquisition_path_remaining,
                ratio_1,
                sb_target,
            );
        }
    } else {
        (*g).taem_speedbrake_integral = 0.0f64;
    }
    let mut unpowered: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_UNPOWERED_ONLY\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !unpowered.is_null()
        && strcmp(unpowered, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
    {
        (*g).taem_speedbrake_integral = 0.0f64;
        (*g).taem_speedbrake_fraction = 0.0f64;
    } else {
        let mut sb_open_step: ::core::ffi::c_double = sb_deploy_slew_per_s * sb_dt;
        let mut sb_close_step: ::core::ffi::c_double = sb_stow_slew_per_s * sb_dt;
        (*g).taem_speedbrake_fraction = clampd(
            sb_target,
            (*g).taem_speedbrake_fraction - sb_close_step,
            (*g).taem_speedbrake_fraction + sb_open_step,
        );
    }
    command_0.speedbrake_fraction = (*g).taem_speedbrake_fraction;
    let mut result_0: GuidanceResult = stabilized(
        g,
        result_make(PHASE_TAEM, command_0, status, best_effort_warning),
        t,
        &raw const (*cfg).vehicle,
        &raw const (*cfg).guidance,
        dt,
    );
    let mut display_route: *const TaemRoute = if (*g).mm305_route_committed
        as ::core::ffi::c_int != 0
    {
        &raw mut (*g).mm305_route
    } else if (*g).mm305_acquisition_route_valid as ::core::ffi::c_int != 0 {
        &raw mut (*g).mm305_acquisition_route
    } else {
        ::core::ptr::null_mut::<TaemRoute>()
    };
    if !display_route.is_null() {
        mm305_route_to_reference_trajectory(
            &raw mut result_0.reference,
            display_route,
            control_model,
            t,
        );
    }
    result_0.command.has_pitch_trim_feedforward = adaptive_trim_input(
        &raw mut (*g).mm305_trim,
        result_0.command.target_aoa,
        &raw mut result_0.command.pitch_trim_feedforward,
    );
    (*g).mm305_command_bank_rad = result_0.command.target_roll * DEG2RAD;
    (*g).mm305_command_aoa_rad = result_0.command.target_aoa * DEG2RAD;
    (*g).mm305_command_ut = (*t).ut;
    (*g).mm305_command_valid = true_0 != 0;
    (*g).phase = PHASE_TAEM;
    return result_0;
}
pub const SPEEDBRAKE_CAP: ::core::ffi::c_double = 0.50f64;
pub const SPEEDBRAKE_NOMINAL: ::core::ffi::c_double = 0.25f64;
pub const SPEEDBRAKE_DEPLOY_SLEW: ::core::ffi::c_double = 0.05f64;
pub const SPEEDBRAKE_STOW_SLEW: ::core::ffi::c_double = 0.20f64;
#[no_mangle]
pub unsafe extern "C" fn terminal_glide_initialize(
    mut g: *mut GuidanceMachine,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) {
    (*g).terminal_glide_mode = true_0 != 0;
    (*g).terminal_path_kind = TERMINAL_PATH_NONE;
    (*g).terminal_energy_sample_valid = false_0 != 0;
    (*g).terminal_previous_specific_energy = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).terminal_previous_speed = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).terminal_energy_sample_ut = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).terminal_energy_loss_accel_ema = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).terminal_speed_loss_accel_ema = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*g).terminal_test_preflare_altitude = clampd(
        fmax(500.0f64, (*s).flare_altitude * 8.0f64),
        450.0f64,
        750.0f64,
    );
    (*g).terminal_test_preflare_target_speed = fmax(
        (*v).final_approach_speed * 1.12f64,
        (*v).minimum_safe_speed * 1.40f64,
    );
    (*g).terminal_test_preflare_min_speed = fmax(
        (*v).final_approach_speed * 1.03f64,
        (*v).minimum_safe_speed * 1.25f64,
    );
    (*g).terminal_test_glide_slope = (*s).taem_glide_slope;
    (*g).terminal_test_final_approach_distance = (*s).final_approach_distance;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_energy_observe(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
) {
    if g.is_null() || t.is_null() || p.is_null() || !(*g).terminal_glide_mode {
        return;
    }
    let mut specific: ::core::ffi::c_double = rotating_specific_energy(
        (*t).latitude,
        (*t).mean_altitude,
        (*t).true_air_speed,
        p,
    );
    if specific.is_finite() as i32 == 0 || (*t).ut.is_finite() as i32 == 0
        || (*t).true_air_speed.is_finite() as i32 == 0
    {
        return;
    }
    if (*g).terminal_energy_sample_valid {
        let mut dt: ::core::ffi::c_double = (*t).ut - (*g).terminal_energy_sample_ut;
        if dt >= 0.08f64 && dt <= 2.5f64 {
            let mut mean_speed: ::core::ffi::c_double = fmax(
                30.0f64,
                0.5f64 * ((*g).terminal_previous_speed + (*t).true_air_speed),
            );
            let mut energy_equiv: ::core::ffi::c_double = ((*g)
                .terminal_previous_specific_energy - specific) / dt / mean_speed;
            let mut speed_loss: ::core::ffi::c_double = ((*g).terminal_previous_speed
                - (*t).true_air_speed) / dt;
            if energy_equiv.is_finite() as i32 != 0
                && energy_equiv >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
                && energy_equiv < 60 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                let mut a: ::core::ffi::c_double = clampd(
                    dt / (2.2f64 + dt),
                    0.03f64,
                    0.45f64,
                );
                (*g).terminal_energy_loss_accel_ema = if (*g)
                    .terminal_energy_loss_accel_ema
                    > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                {
                    (*g).terminal_energy_loss_accel_ema
                        + (energy_equiv - (*g).terminal_energy_loss_accel_ema) * a
                } else {
                    energy_equiv
                };
            }
            if speed_loss.is_finite() as i32 != 0
                && speed_loss >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
                && speed_loss < 60 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                let mut a_0: ::core::ffi::c_double = clampd(
                    dt / (1.8f64 + dt),
                    0.03f64,
                    0.50f64,
                );
                (*g).terminal_speed_loss_accel_ema = if (*g)
                    .terminal_speed_loss_accel_ema
                    > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                {
                    (*g).terminal_speed_loss_accel_ema
                        + (speed_loss - (*g).terminal_speed_loss_accel_ema) * a_0
                } else {
                    speed_loss
                };
            }
        }
    }
    (*g).terminal_previous_specific_energy = specific;
    (*g).terminal_previous_speed = (*t).true_air_speed;
    (*g).terminal_energy_sample_ut = (*t).ut;
    (*g).terminal_energy_sample_valid = true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_expected_energy_loss_accel(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut live: ::core::ffi::c_double = live_drag_accel(t, aero, v);
    if live.is_finite() as i32 == 0 || live < 0.0f64 {
        return ::core::f32::NAN as ::core::ffi::c_double;
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
    let mut modeled: ::core::ffi::c_double = fmax(0.0f64, (*t).dynamic_pressure)
        / fmax(20.0f64, aero.ballistic_coefficient) * fmax(0.0f64, df);
    if modeled.is_finite() as i32 == 0 || modeled < 0.0f64 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    live = fmax(live, modeled);
    if !g.is_null() && (*g).terminal_energy_loss_accel_ema.is_finite() as i32 != 0
        && (*g).terminal_energy_loss_accel_ema >= 0.0f64
    {
        return fmax(live, (*g).terminal_energy_loss_accel_ema);
    }
    return live;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_drag_projection_anchor(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    if g.is_null() || t.is_null() || p.is_null() || v.is_null()
        || aero.ballistic_coefficient.is_finite() as i32 == 0
        || !(aero.ballistic_coefficient > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut df: ::core::ffi::c_double = 1.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        hypot((*t).angle_of_attack, (*t).sideslip),
        v,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut df,
    );
    let mut density: ::core::ffi::c_double = planet_atmospheric_density(
        p,
        (*t).mean_altitude,
    );
    let mut modeled: ::core::ffi::c_double = 0.5f64 * density * (*t).true_air_speed
        * (*t).true_air_speed / aero.ballistic_coefficient * df;
    let mut observed: ::core::ffi::c_double = terminal_expected_energy_loss_accel(
        g,
        t,
        aero,
        v,
    );
    if modeled.is_finite() as i32 == 0 || modeled < 0.0f64
        || observed.is_finite() as i32 == 0 || observed < 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut anchor: ::core::ffi::c_double = if modeled > DBL_MIN {
        observed / modeled
    } else {
        1.0f64
    };
    let mut diag: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_HAC_DIAGNOSTICS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    static mut anchor_diag_count: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    if !diag.is_null()
        && strcmp(diag, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
        && {
            let fresh1 = anchor_diag_count;
            anchor_diag_count = anchor_diag_count + 1;
            fresh1 < 12 as ::core::ffi::c_int
        }
    {
        fprintf(
            stderr,
            b"HAC_DRAG_ANCHOR speed=%.6f aoa=%.6f mach=%.6f density=%.9f q=%.6f polarDf=%.9f dragForce=%.6f liftForce=%.6f mass=%.6f forceVectors=%d physicsValid=%d modeled=%.9f observed=%.9f anchor=%.9f ema=%.9f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).true_air_speed,
            (*t).angle_of_attack,
            (*t).mach,
            density,
            (*t).dynamic_pressure,
            df,
            (*t).drag_force,
            (*t).lift_force,
            (*t).mass,
            if (*t).has_force_vectors as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if (*t).physics_sample_valid as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            modeled,
            observed,
            anchor,
            (*g).terminal_energy_loss_accel_ema,
        );
    }
    return anchor;
}
unsafe extern "C" fn taem_exec_inputs_live_with_contract(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut terminal_contract: *const TaemTerminalContract,
    mut has_final_approach_override: bool,
    mut final_approach_override: bool,
) -> TaemExecInputs {
    let mut inputs: TaemExecInputs = TaemExecInputs {
        mm304_complete: 0 as ::core::ffi::c_int != 0,
        resume_mm305: false,
        terminal_path_selected: false,
        terminal_path_captured: false,
        final_intercept_ready: false,
        final_approach_ready: false,
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
        off_nominal_recovery_active: false,
        off_nominal_recovery_reason: TAEM_RECOVERY_NONE,
    };
    inputs.mm304_complete = (*g).entry_exec.entry_complete;
    let mut native_route: bool = (*g).mm305_route_committed as ::core::ffi::c_int != 0
        && (*g).mm305_route.valid as ::core::ffi::c_int != 0;
    let mut direct_route: bool = (*g).mm305_direct_approach_degraded
        as ::core::ffi::c_int != 0
        && (*g).terminal_path_kind as ::core::ffi::c_uint
            == TERMINAL_PATH_DIRECT as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*g).terminal_path_committed as ::core::ffi::c_int != 0;
    let mut energy_law_route: bool = !cfg.is_null()
        && (*cfg).guidance.taem_energy_law as ::core::ffi::c_int != 0
        && (*g).taem_energy_law.initialized as ::core::ffi::c_int != 0
        && (*g).taem_energy_law.leg == TEL_LEG_FINAL as ::core::ffi::c_int;
    let mut hac_law_route: bool = !cfg.is_null()
        && (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0
        && (*g).taem_hac_law.initialized as ::core::ffi::c_int != 0
        && (*g).taem_hac_law.final_line as ::core::ffi::c_int != 0;
    inputs.terminal_path_selected = (*g).terminal_final_test_mode as ::core::ffi::c_int
        != 0 || native_route as ::core::ffi::c_int != 0
        || direct_route as ::core::ffi::c_int != 0
        || energy_law_route as ::core::ffi::c_int != 0
        || hac_law_route as ::core::ffi::c_int != 0;
    inputs.terminal_path_captured = inputs.terminal_path_selected;
    inputs.final_intercept_ready = (*g).terminal_final_test_mode as ::core::ffi::c_int
        != 0
        || (native_route as ::core::ffi::c_int != 0
            || direct_route as ::core::ffi::c_int != 0
            || energy_law_route as ::core::ffi::c_int != 0
            || hac_law_route as ::core::ffi::c_int != 0)
            && (*g).mm305_hac_exit_reached as ::core::ffi::c_int != 0;
    inputs.final_approach_ready = if has_final_approach_override as ::core::ffi::c_int
        != 0
    {
        final_approach_override as ::core::ffi::c_int
    } else {
        (*g).final_approach_captured as ::core::ffi::c_int
    } != 0;
    if !terminal_contract.is_null() {
        inputs.terminal_contract = *terminal_contract;
    }
    inputs.off_nominal_recovery_active = (*g).attitude_recovery;
    inputs.off_nominal_recovery_reason = (if (*g).attitude_recovery as ::core::ffi::c_int
        != 0
    {
        TAEM_RECOVERY_ATTITUDE as ::core::ffi::c_int
    } else {
        TAEM_RECOVERY_NONE as ::core::ffi::c_int
    }) as TaemRecoveryReason;
    return inputs;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_enter(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut checkpoint_resume: bool,
) -> bool {
    if (*g).taem_exec.initialized as ::core::ffi::c_int != 0
        && (*g).taem_exec.ownership_latched as ::core::ffi::c_int != 0
    {
        return true_0 != 0;
    }
    let mut observation: TaemExecObservation = TaemExecObservation {
        ut: (*t).ut,
        relative_velocity: (*t).true_air_speed,
        checkpoint_restart: checkpoint_resume,
    };
    let mut inputs: TaemExecInputs = taem_exec_inputs_live_with_contract(
        g,
        t,
        s,
        p,
        aero,
        cfg,
        ::core::ptr::null::<TaemTerminalContract>(),
        false_0 != 0,
        false_0 != 0,
    );
    inputs.resume_mm305 = checkpoint_resume;
    if checkpoint_resume {
        inputs.mm304_complete = false_0 != 0;
    }
    return taem_exec_initialize(
        &raw mut (*g).taem_exec,
        &raw mut observation,
        &raw mut inputs,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_sync_with_contract(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
    mut terminal_contract: *const TaemTerminalContract,
    mut has_final_approach_override: bool,
    mut final_approach_override: bool,
) -> bool {
    if !(*g).taem_exec.initialized || !(*g).taem_exec.ownership_latched {
        return false_0 != 0;
    }
    let mut observation: TaemExecObservation = TaemExecObservation {
        ut: (*t).ut,
        relative_velocity: (*t).true_air_speed,
        checkpoint_restart: false_0 != 0,
    };
    let mut inputs: TaemExecInputs = taem_exec_inputs_live_with_contract(
        g,
        t,
        s,
        p,
        aero,
        cfg,
        terminal_contract,
        has_final_approach_override,
        final_approach_override,
    );
    return taem_exec_update(
        &raw mut (*g).taem_exec,
        &raw mut observation,
        &raw mut inputs,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_sync(
    mut g: *mut GuidanceMachine,
    mut t: *const Telemetry,
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut cfg: *const LandingConfiguration,
) {
    taem_exec_sync_with_contract(
        g,
        t,
        s,
        p,
        aero,
        cfg,
        ::core::ptr::null::<TaemTerminalContract>(),
        false_0 != 0,
        false_0 != 0,
    );
}
#[no_mangle]
pub unsafe extern "C" fn terminal_fpa_force_aoa(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut reference_fpa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || p.is_null() || v.is_null() || reference_fpa.is_finite() as i32 == 0
        || !((*t).true_air_speed > DBL_MIN)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut upper: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    if (*t).mach < 1.0f64
        && (*v).terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*v).terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        upper = fmin(upper, (*v).terminal_maximum_lift_angle_of_attack);
    }
    upper = fmax(0.0f64, upper);
    let mut radius: ::core::ffi::c_double = (*p).radius + (*t).mean_altitude;
    if !(radius > DBL_MIN) || !((*p).gravitational_parameter > 0.0f64) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut speed: ::core::ffi::c_double = (*t).true_air_speed;
    let mut gravity: ::core::ffi::c_double = (*p).gravitational_parameter
        / (radius * radius);
    let mut curvature: ::core::ffi::c_double = (*t).horizontal_speed
        * (*t).horizontal_speed / radius;
    let mut gamma: ::core::ffi::c_double = (*t).flight_path_angle * DEG2RAD;
    let mut bank: ::core::ffi::c_double = norm_signed_deg((*t).roll) * DEG2RAD;
    let mut vertical_lift_factor: ::core::ffi::c_double = cos(bank);
    let mut base: ::core::ffi::c_double = (gravity - curvature) * cos(gamma);
    let mut lift0: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
        t,
        aero,
        v,
        0.0f64,
    );
    let mut lift1: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
        t,
        aero,
        v,
        upper,
    );
    if lift0.is_finite() as i32 == 0 || lift1.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut rate0: ::core::ffi::c_double = (lift0 * vertical_lift_factor - base) / speed;
    let mut rate1: ::core::ffi::c_double = (lift1 * vertical_lift_factor - base) / speed;
    let mut error: ::core::ffi::c_double = (reference_fpa - (*t).flight_path_angle)
        * DEG2RAD;
    let mut response: ::core::ffi::c_double = decision_pitch_capture_time(t, upper);
    let mut desired_rate: ::core::ffi::c_double = 0.;
    if response.is_finite() as i32 != 0 && response > DBL_MIN {
        desired_rate = error / response;
    } else {
        desired_rate = if error >= 0.0f64 {
            fmax(rate0, rate1)
        } else {
            fmin(rate0, rate1)
        };
    }
    let mut minimum_rate: ::core::ffi::c_double = fmin(rate0, rate1);
    let mut maximum_rate: ::core::ffi::c_double = fmax(rate0, rate1);
    desired_rate = clampd(desired_rate, minimum_rate, maximum_rate);
    if fabs(rate1 - rate0) <= DBL_EPSILON {
        return if fabs(desired_rate - rate0) <= sqrt(DBL_EPSILON) {
            0.0f64
        } else {
            upper
        };
    }
    let mut increasing: bool = rate1 > rate0;
    if increasing as ::core::ffi::c_int != 0 && desired_rate <= rate0
        || !increasing && desired_rate >= rate0
    {
        return 0.0f64;
    }
    if increasing as ::core::ffi::c_int != 0 && desired_rate >= rate1
        || !increasing && desired_rate <= rate1
    {
        return upper;
    }
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let mut hi: ::core::ffi::c_double = upper;
    let mut resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON) * fmax(1.0f64, upper);
    let mut guard: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while hi - lo > resolution
        && {
            let fresh2 = guard;
            guard = guard.wrapping_add(1);
            fresh2 < (4 as ::core::ffi::c_int * DBL_MANT_DIG) as ::core::ffi::c_uint
        }
    {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        let mut lift: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
            t,
            aero,
            v,
            mid,
        );
        if lift.is_finite() as i32 == 0 {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        let mut rate: ::core::ffi::c_double = (lift * vertical_lift_factor - base)
            / speed;
        if increasing as ::core::ffi::c_int != 0 && rate < desired_rate
            || !increasing && rate > desired_rate
        {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return clampd(0.5f64 * (lo + hi), 0.0f64, upper);
}
#[no_mangle]
pub unsafe extern "C" fn fixed_hac_lead_drag_aoa(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut target_speed: ::core::ffi::c_double,
    mut remaining_distance: ::core::ffi::c_double,
    mut reference_slope_deg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || p.is_null() || v.is_null() || !(remaining_distance > DBL_MIN)
        || !(target_speed > 0.0f64) || !((*t).true_air_speed > target_speed)
        || reference_slope_deg.is_finite() as i32 == 0
        || fabs(reference_slope_deg) >= 90.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut radius: ::core::ffi::c_double = (*p).radius + (*t).mean_altitude;
    if !(radius > DBL_MIN) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut gravity: ::core::ffi::c_double = (*p).gravitational_parameter
        / (radius * radius);
    let mut speed: ::core::ffi::c_double = (*t).true_air_speed;
    let mut required: ::core::ffi::c_double = (speed * speed
        - target_speed * target_speed) / (2.0f64 * remaining_distance);
    required += gravity * sin(reference_slope_deg * DEG2RAD);
    if !(required > 0.0f64) || required.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut upper: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    let mut drag0: ::core::ffi::c_double = fixed_hac_projected_drag_accel(
        t,
        aero,
        v,
        0.0f64,
    );
    let mut drag1: ::core::ffi::c_double = fixed_hac_projected_drag_accel(
        t,
        aero,
        v,
        upper,
    );
    if drag0.is_finite() as i32 == 0 || drag1.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if drag0 >= required {
        return 0.0f64;
    }
    if drag1 <= required {
        return upper;
    }
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let mut hi: ::core::ffi::c_double = upper;
    let mut resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON) * fmax(1.0f64, upper);
    let mut guard: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while hi - lo > resolution
        && {
            let fresh3 = guard;
            guard = guard.wrapping_add(1);
            fresh3 < (4 as ::core::ffi::c_int * DBL_MANT_DIG) as ::core::ffi::c_uint
        }
    {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        let mut drag: ::core::ffi::c_double = fixed_hac_projected_drag_accel(
            t,
            aero,
            v,
            mid,
        );
        if drag.is_finite() as i32 == 0 {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        if drag < required {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return clampd(0.5f64 * (lo + hi), 0.0f64, upper);
}
#[no_mangle]
pub unsafe extern "C" fn fixed_hac_projected_drag_accel(
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut aoa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || v.is_null() || !(aero.ballistic_coefficient > 0.0f64)
        || (*t).true_air_speed.is_finite() as i32 == 0 || !((*t).true_air_speed > 0.0f64)
        || (*t).atmospheric_density.is_finite() as i32 == 0
        || (*t).atmospheric_density < 0.0f64 || aoa.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut current_incidence: ::core::ffi::c_double = fmax(
        0.0f64,
        hypot((*t).angle_of_attack, (*t).sideslip),
    );
    let mut current_df: ::core::ffi::c_double = 1.0f64;
    let mut target_df: ::core::ffi::c_double = 1.0f64;
    aerodynamic_force_factors_mach(
        (*t).mach,
        current_incidence,
        v,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut current_df,
    );
    aerodynamic_force_factors_mach(
        (*t).mach,
        clampd(aoa, 0.0f64, (*v).maximum_angle_of_attack),
        v,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut target_df,
    );
    let mut modeled_current: ::core::ffi::c_double = 0.5f64 * (*t).atmospheric_density
        * (*t).true_air_speed * (*t).true_air_speed / aero.ballistic_coefficient
        * fmax(0.0f64, current_df);
    let mut modeled_target: ::core::ffi::c_double = 0.5f64 * (*t).atmospheric_density
        * (*t).true_air_speed * (*t).true_air_speed / aero.ballistic_coefficient
        * fmax(0.0f64, target_df);
    if modeled_current.is_finite() as i32 == 0 || modeled_target.is_finite() as i32 == 0
        || modeled_current < 0.0f64 || modeled_target < 0.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut measured: ::core::ffi::c_double = live_drag_accel(t, aero, v);
    let mut anchor: ::core::ffi::c_double = if modeled_current > DBL_MIN
        && measured.is_finite() as i32 != 0 && measured >= 0.0f64
    {
        measured / modeled_current
    } else {
        1.0f64
    };
    return if anchor.is_finite() as i32 != 0 && anchor >= 0.0f64 {
        modeled_target * anchor
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
}
#[no_mangle]
pub unsafe extern "C" fn terminal_required_aoa_for_lateral_limit(
    mut t: *const Telemetry,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut lateral_accel: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut maximum_aoa: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if t.is_null() || v.is_null() || lateral_accel.is_finite() as i32 == 0
        || bank_deg.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    maximum_aoa = clampd(maximum_aoa, 0.0f64, (*v).maximum_angle_of_attack);
    let mut required_lateral: ::core::ffi::c_double = fabs(lateral_accel);
    if required_lateral <= DBL_EPSILON {
        return 0.0f64;
    }
    let mut effectiveness: ::core::ffi::c_double = if (*t).bank_effectiveness.is_finite()
        as i32 != 0 && (*t).bank_effectiveness > 0.0f64
    {
        (*t).bank_effectiveness
    } else {
        1.0f64
    };
    let mut effective_bank: ::core::ffi::c_double = fmin(
        fabs(bank_deg * effectiveness),
        nextafter(90.0f64, 0.0f64),
    ) * DEG2RAD;
    let mut bank_component: ::core::ffi::c_double = fabs(sin(effective_bank));
    if bank_component <= DBL_MIN {
        return maximum_aoa;
    }
    let mut required_lift: ::core::ffi::c_double = required_lateral / bank_component;
    let mut lift0: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
        t,
        aero,
        v,
        0.0f64,
    );
    let mut lift1: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
        t,
        aero,
        v,
        maximum_aoa,
    );
    if lift0.is_finite() as i32 == 0 || lift1.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if lift0 >= required_lift {
        return 0.0f64;
    }
    if lift1 <= required_lift {
        return maximum_aoa;
    }
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let mut hi: ::core::ffi::c_double = maximum_aoa;
    let mut resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON)
        * fmax(1.0f64, maximum_aoa);
    let mut guard: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while hi - lo > resolution
        && {
            let fresh4 = guard;
            guard = guard.wrapping_add(1);
            fresh4 < (4 as ::core::ffi::c_int * DBL_MANT_DIG) as ::core::ffi::c_uint
        }
    {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        let mut lift: ::core::ffi::c_double = terminal_projected_lift_accel_at_aoa(
            t,
            aero,
            v,
            mid,
        );
        if lift.is_finite() as i32 == 0 {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        if lift < required_lift {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return clampd(0.5f64 * (lo + hi), 0.0f64, maximum_aoa);
}
#[no_mangle]
pub unsafe extern "C" fn terminal_direct_runway_target_aoa(
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut reference_fpa: ::core::ffi::c_double,
    mut lateral_accel: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut glide_aoa_out: *mut ::core::ffi::c_double,
    mut lateral_aoa_out: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !glide_aoa_out.is_null() {
        *glide_aoa_out = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if !lateral_aoa_out.is_null() {
        *lateral_aoa_out = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if t.is_null() || p.is_null() || v.is_null() || reference_fpa.is_finite() as i32 == 0
        || lateral_accel.is_finite() as i32 == 0 || bank_deg.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut upper: ::core::ffi::c_double = (*v).maximum_angle_of_attack;
    if (*t).mach < 1.0f64
        && (*v).terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*v).terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        upper = fmin(upper, (*v).terminal_maximum_lift_angle_of_attack);
    }
    upper = fmax(0.0f64, upper);
    let mut glide: ::core::ffi::c_double = if (*t)
        .calibrated_best_glide_angle_of_attack
        .is_finite() as i32 != 0 && (*t).calibrated_best_glide_angle_of_attack >= 0.0f64
    {
        clampd((*t).calibrated_best_glide_angle_of_attack, 0.0f64, upper)
    } else {
        aerodynamic_best_glide_aoa((*t).mach, v)
    };
    glide = clampd(
        if glide.is_finite() as i32 != 0 { glide } else { 0.0f64 },
        0.0f64,
        upper,
    );
    let mut vertical: ::core::ffi::c_double = terminal_fpa_force_aoa(
        t,
        p,
        aero,
        v,
        reference_fpa,
    );
    if vertical.is_finite() as i32 == 0 {
        vertical = glide;
    }
    let mut lateral: ::core::ffi::c_double = terminal_required_aoa_for_lateral_limit(
        t,
        aero,
        v,
        lateral_accel,
        bank_deg,
        upper,
    );
    if lateral.is_finite() as i32 == 0 {
        lateral = glide;
    }
    if !glide_aoa_out.is_null() {
        *glide_aoa_out = glide;
    }
    if !lateral_aoa_out.is_null() {
        *lateral_aoa_out = lateral;
    }
    return clampd(fmax(glide, fmax(vertical, lateral)), 0.0f64, upper);
}
unsafe extern "C" fn terminal_atmosphere_vertical_resolution(
    mut p: *const PlanetModel,
    mut lower_altitude: ::core::ffi::c_double,
    mut upper_altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if p.is_null() || (*p).atmosphere_sample_count < 2 as size_t {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    lower_altitude = fmax(0.0f64, lower_altitude);
    upper_altitude = fmax(lower_altitude, upper_altitude);
    let mut resolution: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut i: size_t = 1 as size_t;
    while i < (*p).atmosphere_sample_count {
        let mut a: ::core::ffi::c_double = (*p)
            .atmosphere_altitude[i.wrapping_sub(1 as size_t) as usize];
        let mut b: ::core::ffi::c_double = (*p).atmosphere_altitude[i as usize];
        if a.is_finite() as i32 == 0 || b.is_finite() as i32 == 0 || !(b > a) {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        if !(b < lower_altitude || a > upper_altitude) {
            resolution = fmin(resolution, b - a);
        }
        i = i.wrapping_add(1);
    }
    return resolution;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_quadratic_drag_energy_step(
    mut speed: ::core::ffi::c_double,
    mut drag: ::core::ffi::c_double,
    mut air_distance: ::core::ffi::c_double,
    mut potential_drop: ::core::ffi::c_double,
    mut next_speed: *mut ::core::ffi::c_double,
    mut work: *mut ::core::ffi::c_double,
) -> bool {
    if next_speed.is_null() || work.is_null() || speed.is_finite() as i32 == 0
        || !(speed > 0.0f64) || drag.is_finite() as i32 == 0 || drag < 0.0f64
        || air_distance.is_finite() as i32 == 0 || air_distance < 0.0f64
        || potential_drop.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut y: ::core::ffi::c_double = speed * speed;
    let mut x: ::core::ffi::c_double = 2.0f64 * drag * air_distance / y;
    if x.is_finite() as i32 == 0 || x < 0.0f64 {
        return false_0 != 0;
    }
    let mut attenuation: ::core::ffi::c_double = exp(-x);
    let mut response: ::core::ffi::c_double = if x > DBL_EPSILON {
        -expm1(-x) / x
    } else {
        1.0f64
    };
    let mut next_y: ::core::ffi::c_double = y * attenuation
        + 2.0f64 * potential_drop * response;
    if next_y.is_finite() as i32 == 0 || !(next_y > 0.0f64) {
        return false_0 != 0;
    }
    let mut integrated_work: ::core::ffi::c_double = 0.5f64 * y * -expm1(-x)
        + potential_drop * (1.0f64 - response);
    if integrated_work.is_finite() as i32 == 0 || integrated_work < 0.0f64 {
        return false_0 != 0;
    }
    *next_speed = sqrt(next_y);
    *work = integrated_work;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_projected_drag_work_state_anchored(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut target_aoa: ::core::ffi::c_double,
    mut ground_path: ::core::ffi::c_double,
    mut slope_deg: ::core::ffi::c_double,
    mut end_speed_out: *mut ::core::ffi::c_double,
    mut end_altitude_out: *mut ::core::ffi::c_double,
    mut inherited_anchor: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !end_speed_out.is_null() {
        *end_speed_out = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if !end_altitude_out.is_null() {
        *end_altitude_out = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if g.is_null() || t.is_null() || p.is_null() || v.is_null() || s.is_null()
        || ground_path.is_finite() as i32 == 0 || ground_path < 0.0f64
        || slope_deg.is_finite() as i32 == 0 || target_aoa.is_finite() as i32 == 0
        || aero.ballistic_coefficient.is_finite() as i32 == 0
        || !(aero.ballistic_coefficient > 0.0f64)
        || (*t).true_air_speed.is_finite() as i32 == 0 || !((*t).true_air_speed > 0.0f64)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if ground_path == 0.0f64 {
        if !end_speed_out.is_null() {
            *end_speed_out = (*t).true_air_speed;
        }
        if !end_altitude_out.is_null() {
            *end_altitude_out = (*t).mean_altitude;
        }
        return 0.0f64;
    }
    let mut slope: ::core::ffi::c_double = slope_deg * DEG2RAD;
    let mut cosine: ::core::ffi::c_double = cos(slope);
    if !(slope >= 0.0f64 && slope < 0.5f64 * M_PI) || !(cosine > 0.0f64) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut tangent: ::core::ffi::c_double = tan(slope);
    let mut path_factor: ::core::ffi::c_double = 1.0f64 / cosine;
    let mut anchor: ::core::ffi::c_double = if inherited_anchor.is_nan() as i32 != 0 {
        terminal_drag_projection_anchor(g, t, p, aero, v)
    } else {
        inherited_anchor
    };
    if anchor.is_finite() as i32 == 0 || anchor < 0.0f64 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut response_time: ::core::ffi::c_double = hac_response_lead_time(t, s, 0.0f64);
    if response_time.is_finite() as i32 == 0 || response_time < 0.0f64 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut response_distance: ::core::ffi::c_double = (*t).true_air_speed
        * response_time;
    let mut end_altitude: ::core::ffi::c_double = fmax(
        0.0f64,
        (*t).mean_altitude - ground_path * tangent,
    );
    let mut vertical_resolution: ::core::ffi::c_double = terminal_atmosphere_vertical_resolution(
        p,
        end_altitude,
        (*t).mean_altitude,
    );
    if vertical_resolution.is_nan() as i32 != 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut atmosphere_ground_resolution: ::core::ffi::c_double = if vertical_resolution
        .is_finite() as i32 != 0 && tangent > DBL_MIN
    {
        vertical_resolution / tangent
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut response_resolution: ::core::ffi::c_double = fmax(
        (*t).true_air_speed / fmax(1.0f64, (*s).guidance_rate),
        sqrt(DBL_EPSILON) * fmax(1.0f64, ground_path),
    );
    let mut response_resolved: bool = response_distance > response_resolution;
    let mut segment_length: ::core::ffi::c_double = ground_path;
    if response_resolved {
        segment_length = fmin(segment_length, response_distance);
    }
    if atmosphere_ground_resolution.is_finite() as i32 != 0
        && atmosphere_ground_resolution > DBL_MIN
    {
        segment_length = fmin(segment_length, atmosphere_ground_resolution);
    }
    if !(segment_length > 0.0f64) || segment_length.is_finite() as i32 == 0 {
        segment_length = ground_path;
    }
    let mut steps: size_t = ceil(ground_path / segment_length) as size_t;
    if steps < 1 as size_t {
        steps = 1 as size_t;
    }
    let mut initial_aoa: ::core::ffi::c_double = clampd(
        fabs((*t).angle_of_attack),
        0.0f64,
        (*v).maximum_angle_of_attack,
    );
    let mut ds: ::core::ffi::c_double = ground_path / steps as ::core::ffi::c_double;
    let mut speed: ::core::ffi::c_double = (*t).true_air_speed;
    let mut altitude: ::core::ffi::c_double = (*t).mean_altitude;
    let mut work: ::core::ffi::c_double = 0.0f64;
    let mut i: size_t = 0 as size_t;
    while i < steps {
        let mut sample_altitude: ::core::ffi::c_double = fmax(
            0.0f64,
            altitude - 0.5f64 * ds * tangent,
        );
        let mut density: ::core::ffi::c_double = planet_atmospheric_density(
            p,
            sample_altitude,
        );
        if density.is_finite() as i32 == 0 || density < 0.0f64 {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
            p,
            sample_altitude,
        );
        let mut mach: ::core::ffi::c_double = if sound.is_finite() as i32 != 0
            && sound > DBL_MIN
        {
            speed / sound
        } else {
            (*t).mach
        };
        let mut distance_mid: ::core::ffi::c_double = (i as ::core::ffi::c_double
            + 0.5f64) * ds;
        let mut response_progress: ::core::ffi::c_double = if response_resolved
            as ::core::ffi::c_int != 0
        {
            clampd(distance_mid / response_distance, 0.0f64, 1.0f64)
        } else {
            1.0f64
        };
        response_progress = response_progress * response_progress
            * (3.0f64 - 2.0f64 * response_progress);
        let mut projected_aoa: ::core::ffi::c_double = initial_aoa
            + (target_aoa - initial_aoa) * response_progress;
        let mut df: ::core::ffi::c_double = 1.0f64;
        aerodynamic_force_factors_mach(
            mach,
            projected_aoa,
            v,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            &raw mut df,
        );
        if df.is_finite() as i32 == 0 || df < 0.0f64 {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        let mut drag: ::core::ffi::c_double = 0.5f64 * density * speed * speed
            / aero.ballistic_coefficient * df * anchor;
        if drag.is_finite() as i32 == 0 || drag < 0.0f64 {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        let mut air_ds: ::core::ffi::c_double = ds * path_factor;
        let mut next_altitude: ::core::ffi::c_double = fmax(
            0.0f64,
            altitude - ds * tangent,
        );
        let mut potential_drop: ::core::ffi::c_double = rotating_specific_energy(
            (*t).latitude,
            altitude,
            0.0f64,
            p,
        ) - rotating_specific_energy((*t).latitude, next_altitude, 0.0f64, p);
        let mut next_speed: ::core::ffi::c_double = ::core::f32::NAN
            as ::core::ffi::c_double;
        let mut segment_work: ::core::ffi::c_double = ::core::f32::NAN
            as ::core::ffi::c_double;
        if !terminal_quadratic_drag_energy_step(
            speed,
            drag,
            air_ds,
            potential_drop,
            &raw mut next_speed,
            &raw mut segment_work,
        ) {
            if !end_speed_out.is_null() {
                *end_speed_out = 0.0f64;
            }
            if !end_altitude_out.is_null() {
                *end_altitude_out = altitude;
            }
            return ::core::f32::INFINITY as ::core::ffi::c_double;
        }
        work += segment_work;
        speed = next_speed;
        altitude = next_altitude;
        i = i.wrapping_add(1);
    }
    if !end_speed_out.is_null() {
        *end_speed_out = speed;
    }
    if !end_altitude_out.is_null() {
        *end_altitude_out = altitude;
    }
    return work;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_projected_drag_work_state(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut target_aoa: ::core::ffi::c_double,
    mut ground_path: ::core::ffi::c_double,
    mut slope_deg: ::core::ffi::c_double,
    mut end_speed_out: *mut ::core::ffi::c_double,
    mut end_altitude_out: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return terminal_projected_drag_work_state_anchored(
        g,
        t,
        p,
        aero,
        v,
        s,
        target_aoa,
        ground_path,
        slope_deg,
        end_speed_out,
        end_altitude_out,
        ::core::f32::NAN as ::core::ffi::c_double,
    );
}
#[no_mangle]
pub unsafe extern "C" fn terminal_projected_drag_work(
    mut g: *const GuidanceMachine,
    mut t: *const Telemetry,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut target_aoa: ::core::ffi::c_double,
    mut ground_path: ::core::ffi::c_double,
    mut slope_deg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return terminal_projected_drag_work_state(
        g,
        t,
        p,
        aero,
        v,
        s,
        target_aoa,
        ground_path,
        slope_deg,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
}
#[no_mangle]
pub unsafe extern "C" fn terminal_path_slope_bounds(
    mut t: *const Telemetry,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut minimum: *mut ::core::ffi::c_double,
    mut maximum: *mut ::core::ffi::c_double,
) {
    let mut taem: ::core::ffi::c_double = clampd(
        if !s.is_null() { (*s).taem_glide_slope } else { 12.0f64 },
        6.0f64,
        30.0f64,
    );
    let mut final_0: ::core::ffi::c_double = clampd(
        if !s.is_null() { (*s).final_glide_slope } else { 20.0f64 },
        taem,
        35.0f64,
    );
    let mut speed_reference: ::core::ffi::c_double = if !v.is_null() {
        fmax((*v).minimum_safe_speed, (*v).final_approach_speed)
    } else {
        0.0f64
    };
    let mut speed_scale: ::core::ffi::c_double = fmax(speed_reference, DBL_MIN);
    let mut hot: ::core::ffi::c_double = if !t.is_null() {
        clampd(((*t).true_air_speed - speed_reference) / speed_scale, 0.0f64, 1.0f64)
    } else {
        0.0f64
    };
    let mut cool_lo: ::core::ffi::c_double = fmax(taem, final_0 * 0.90f64);
    let mut hot_lo: ::core::ffi::c_double = clampd(taem - 4.0f64, 5.0f64, cool_lo);
    let mut lo: ::core::ffi::c_double = cool_lo + (hot_lo - cool_lo) * hot;
    let mut ld: ::core::ffi::c_double = if !t.is_null()
        && (*t).estimated_lift_to_drag.is_finite() as i32 != 0
        && (*t).estimated_lift_to_drag > 0.0f64
    {
        (*t).estimated_lift_to_drag
    } else if !v.is_null() {
        (*v).estimated_lift_to_drag
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    if ld.is_finite() as i32 != 0 && ld > 0.0f64 {
        lo = fmax(
            lo,
            clampd(atan(1.0f64 / fmax(ld, 2.0f64)) * RAD2DEG, 5.0f64, 45.0f64),
        );
    }
    let mut hi: ::core::ffi::c_double = clampd(
        fmax(fmax(final_0 + 12.0f64, taem + 12.0f64), lo + 8.0f64),
        lo + 4.0f64,
        50.0f64,
    );
    if !minimum.is_null() {
        *minimum = lo;
    }
    if !maximum.is_null() {
        *maximum = hi;
    }
}
#[inline]
unsafe extern "C" fn taem_alignment_target_height(
    mut model: *const TerminalModel,
) -> ::core::ffi::c_double {
    return (*model).guidance.final_approach_distance
        * tan((*model).guidance.final_glide_slope / 57.29577951308232f64);
}
#[inline]
unsafe extern "C" fn taem_alignment_capture_distance(
    mut model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
) -> ::core::ffi::c_double {
    let mut speed: ::core::ffi::c_double = if !geometry.is_null() {
        fmax(0.0f64, (*geometry).ground_speed_mps)
    } else {
        0.0f64
    };
    let mut response_distance: ::core::ffi::c_double = 30.0f64 * speed;
    return fmax(3000.0f64, fmin(6000.0f64, response_distance));
}
#[inline]
unsafe extern "C" fn taem_alignment_reference(
    mut model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
) -> TaemPathReference {
    let radians_per_degree: ::core::ffi::c_double = 0.017453292519943295f64;
    let mut station: ::core::ffi::c_double = -(*model).guidance.final_approach_distance;
    let mut remaining: ::core::ffi::c_double = station - (*geometry).runway_along_m;
    let mut effective_remaining: ::core::ffi::c_double = fmax(2000.0f64, remaining);
    let mut course_offset: ::core::ffi::c_double = atan2(
        -2.0f64 * (*geometry).runway_cross_m,
        effective_remaining,
    ) / radians_per_degree;
    course_offset = fmax(-20.0f64, fmin(20.0f64, course_offset));
    let mut final_slope: ::core::ffi::c_double = (*model).guidance.final_glide_slope;
    let mut target_height: ::core::ffi::c_double = taem_alignment_target_height(model);
    let mut upstream: ::core::ffi::c_double = fmax(0.0f64, remaining);
    let mut glide_height: ::core::ffi::c_double = target_height
        + upstream * tan(final_slope * radians_per_degree);
    let mut height_error: ::core::ffi::c_double = glide_height
        - (*geometry).altitude_above_runway_m;
    let mut correction: ::core::ffi::c_double = atan2(height_error, effective_remaining)
        / radians_per_degree;
    correction = fmax(-6.0f64, fmin(6.0f64, correction));
    let mut fpa: ::core::ffi::c_double = -final_slope + correction;
    return TaemPathReference {
        runway_along_m: (*geometry).runway_along_m,
        runway_cross_m: 0.0f64,
        course_deg: (*model).site.runway_heading + course_offset,
        curvature_right_per_m: 0.0f64,
        altitude_m: ::core::f32::NAN as ::core::ffi::c_double,
        flight_path_angle_deg: fpa,
        vertical_curvature_per_m: 0.0f64,
        station_m: 0.0f64,
    };
}
#[inline]
unsafe extern "C" fn taem_alignment_ready(
    mut model: *const TerminalModel,
    mut state: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
) -> bool {
    let degrees_per_radian: ::core::ffi::c_double = 57.29577951308232f64;
    let mut station: ::core::ffi::c_double = -(*model).guidance.final_approach_distance;
    let mut target_height: ::core::ffi::c_double = taem_alignment_target_height(model);
    let mut target_speed: ::core::ffi::c_double = fmax(
        (*model).guidance.final_alignment_speed,
        (*model).vehicle.touchdown_speed,
    );
    return fabs((*geometry).runway_cross_m) < 0.5f64 * (*model).site.runway_width
        && fabs((*geometry).heading_error_deg) < 2.0f64
        && fabs((*state).attitude.bank_rad * degrees_per_radian) < 5.0f64
        && fabs((*state).attitude.bank_rate_rad_s * degrees_per_radian) < 3.0f64
        && fabs((*geometry).runway_along_m - station) <= 250.0f64
        && fabs((*geometry).altitude_above_runway_m - target_height) <= 150.0f64
        && fabs((*geometry).airspeed_mps - target_speed) <= 10.0f64
        && fabs((*geometry).flight_path_angle_deg + (*model).guidance.final_glide_slope)
            <= 3.0f64;
}
#[inline]
unsafe extern "C" fn taem_alignment_exhausted(
    mut model: *const TerminalModel,
    mut geometry: *const TaemGeometryState,
) -> bool {
    let mut station: ::core::ffi::c_double = -(*model).guidance.final_approach_distance;
    if (*geometry).flight_path_angle_deg < -(*model).guidance.final_glide_slope - 6.0f64
    {
        return (*geometry).runway_along_m > station;
    }
    return (*geometry).runway_along_m > station + 500.0f64
        || (*geometry).altitude_above_runway_m < 300.0f64;
}
pub const MM305_ADMISSION_MAX_AGE_S: ::core::ffi::c_double = 10.0f64;
pub const MM305_ADMISSION_RETRY_S: ::core::ffi::c_double = 5.0f64;
pub const TEL_AOA_SUP_MIN: ::core::ffi::c_double = 8.0f64;
pub const TEL_AOA_MIN: ::core::ffi::c_double = 1.0f64;
pub const TEL_Q_LO: ::core::ffi::c_double = 3500.0f64;
pub const TEL_Q_HI: ::core::ffi::c_double = 11000.0f64;
pub const TEL_BLEND: ::core::ffi::c_double = 8000.0f64;
pub const TEL_AIM: ::core::ffi::c_double = 9000.0f64;
pub const TEL_BANK_MAX: ::core::ffi::c_double = 50.0f64;
pub const TEL_BANK_SUP: ::core::ffi::c_double = 45.0f64;
pub const TEL_BANK_TURN: ::core::ffi::c_double = 55.0f64;
pub const TEL_BANK_ARC: ::core::ffi::c_double = 60.0f64;
pub const TEL_R_MIN: ::core::ffi::c_double = 2200.0f64;
pub const TEL_R_SPIRAL_MIN: ::core::ffi::c_double = 1600.0f64;
pub const TEL_R_MAX: ::core::ffi::c_double = 10000.0f64;
pub const TEL_R_TURN: ::core::ffi::c_double = 2600.0f64;
pub const TEL_K_PLAN: ::core::ffi::c_double = 0.5f64;
pub const TEL_MODE_FREEZE: ::core::ffi::c_double = 8000.0f64;
pub const TEL_UPDATE_S: ::core::ffi::c_double = 2.0f64;
pub const TEL_REPLAN_MARGIN: ::core::ffi::c_double = 2000.0f64;
pub const TEL_LOW_ENERGY_DWELL_S: ::core::ffi::c_double = 3.0f64;
pub const TEL_K_SHORT: ::core::ffi::c_double = 0.05f64;
pub const TEL_FINAL_ENERGY_SPAN_M: ::core::ffi::c_double = 400.0f64;
pub const JET_ASSIST_FULL_DEFICIT_J_KG: ::core::ffi::c_double = 2000.0f64;
pub const JET_ASSIST_MIN_THROTTLE: ::core::ffi::c_double = 0.25f64;
pub const TEL_PI: ::core::ffi::c_double = 3.14159265358979323846f64;
pub const TEL_G0: ::core::ffi::c_double = 9.81f64;
unsafe extern "C" fn tel_delivery_policy(mut c: *const TelContext) -> bool {
    return !c.is_null() && !(*c).model.is_null()
        && (*(*c).model).guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*(*c).model).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
}
unsafe extern "C" fn tel_wrap(mut a: ::core::ffi::c_double) -> ::core::ffi::c_double {
    return remainder(a, 2.0f64 * TEL_PI);
}
unsafe extern "C" fn tel_gravity(
    mut c: *const TelContext,
    mut h: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = (*(*c).model).world.radius_m
        + (*(*c).model).site.altitude + h;
    return (*(*c).model).world.mu_m3_s2 / (r * r);
}
unsafe extern "C" fn tel_air(
    mut c: *const TelContext,
    mut h: ::core::ffi::c_double,
    mut v: ::core::ffi::c_double,
    mut q: *mut ::core::ffi::c_double,
    mut mach: *mut ::core::ffi::c_double,
) {
    let mut a: AtmosphereSample = world_atmosphere_sample(
        &raw const (*(*c).model).world,
        (*(*c).model).site.altitude + h,
    );
    *q = 0.5f64 * a.density_kg_m3 * v * v;
    *mach = if a.speed_of_sound_mps > 1.0f64 {
        v / a.speed_of_sound_mps
    } else {
        0.0f64
    };
}
unsafe extern "C" fn tel_forces(
    mut c: *const TelContext,
    mut p: *const TelPoint,
    mut aoa: ::core::ffi::c_double,
    mut q: *mut ::core::ffi::c_double,
    mut mach: *mut ::core::ffi::c_double,
    mut lift: *mut ::core::ffi::c_double,
    mut drag: *mut ::core::ffi::c_double,
) {
    let mut cl: ::core::ffi::c_double = 0.;
    let mut cd: ::core::ffi::c_double = 0.;
    tel_air(c, (*p).h, (*p).v, q, mach);
    aero_coefficients(
        &raw const (*(*c).model).aero,
        *mach,
        aoa,
        &raw mut cl,
        &raw mut cd,
    );
    *lift = cl * *q * (*(*c).model).aero.reference_area_m2;
    *drag = cd * *q * (*(*c).model).aero.reference_area_m2;
}
unsafe extern "C" fn tel_lift_aoa(
    mut c: *const TelContext,
    mut p: *const TelPoint,
    mut lift_needed: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut lo: ::core::ffi::c_double = -5.0f64;
    let mut hi: ::core::ffi::c_double = (*c).aoa_max;
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    let mut lift: ::core::ffi::c_double = 0.;
    let mut drag: ::core::ffi::c_double = 0.;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 30 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        tel_forces(c, p, mid, &raw mut q, &raw mut mach, &raw mut lift, &raw mut drag);
        if lift < lift_needed {
            lo = mid;
        } else {
            hi = mid;
        }
        i += 1;
    }
    return 0.5f64 * (lo + hi);
}
unsafe extern "C" fn tel_best_ld_aoa(
    mut c: *const TelContext,
    mut mach: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let step: ::core::ffi::c_double = 0.25f64;
    let mut best_aoa: ::core::ffi::c_double = (*c).aoa_max;
    let mut best_ld: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut a: ::core::ffi::c_double = TEL_AOA_MIN;
    while a <= (*c).aoa_max + 1e-9f64 {
        let mut cl: ::core::ffi::c_double = 0.;
        let mut cd: ::core::ffi::c_double = 0.;
        aero_coefficients(
            &raw const (*(*c).model).aero,
            mach,
            a,
            &raw mut cl,
            &raw mut cd,
        );
        if cd > 0.0f64 {
            let mut ld: ::core::ffi::c_double = cl / cd;
            if ld > best_ld {
                best_ld = ld;
                best_aoa = a;
            }
        }
        a += step;
    }
    return best_aoa;
}
unsafe extern "C" fn tel_vertical(
    mut c: *const TelContext,
    mut p: *const TelPoint,
    mut bank_deg: ::core::ffi::c_double,
    mut k: ::core::ffi::c_double,
    mut along: ::core::ffi::c_double,
    mut live: bool,
) -> ::core::ffi::c_double {
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    let mut lift: ::core::ffi::c_double = 0.;
    let mut drag: ::core::ffi::c_double = 0.;
    tel_air(c, (*p).h, (*p).v, &raw mut q, &raw mut mach);
    if mach > 1.05f64 {
        let mut best: ::core::ffi::c_double = tel_best_ld_aoa(c, mach);
        if (*p).g < -32.0f64 * DEG2RAD {
            return (*c).aoa_max;
        }
        return best + (fmin(best, TEL_AOA_SUP_MIN) - best) * k;
    }
    let mut best_0: ::core::ffi::c_double = tel_best_ld_aoa(c, mach);
    let mut best_q: ::core::ffi::c_double = 0.;
    let mut best_mach: ::core::ffi::c_double = 0.;
    let mut best_lift: ::core::ffi::c_double = 0.;
    let mut best_drag: ::core::ffi::c_double = 0.;
    tel_forces(
        c,
        p,
        best_0,
        &raw mut best_q,
        &raw mut best_mach,
        &raw mut best_lift,
        &raw mut best_drag,
    );
    let mut vertical_load: ::core::ffi::c_double = (*p).m * tel_gravity(c, (*p).h)
        * fmax(0.0f64, cos((*p).g));
    let mut bank_cos: ::core::ffi::c_double = fmax(
        0.3f64,
        cos(fabs(bank_deg) * DEG2RAD),
    );
    let mut q_range: ::core::ffi::c_double = if best_lift > 0.0f64 {
        best_q * vertical_load / (bank_cos * best_lift)
    } else {
        TEL_Q_LO
    };
    let mut safe_speed: ::core::ffi::c_double = (*(*c).model).vehicle.minimum_safe_speed;
    if (*c).final_line {
        safe_speed = fmax(safe_speed, (*(*c).model).vehicle.final_approach_speed);
    }
    if (*p).v > 0.0f64 && safe_speed > 0.0f64 {
        q_range = fmax(
            q_range,
            q * safe_speed * safe_speed / ((*p).v * (*p).v * bank_cos),
        );
    }
    let mut q_ref: ::core::ffi::c_double = if tel_delivery_policy(c)
        as ::core::ffi::c_int != 0
    {
        q_range + (fmax(q_range, TEL_Q_HI) - q_range) * k
    } else {
        TEL_Q_LO + (TEL_Q_HI - TEL_Q_LO) * k
    };
    if along < TEL_BLEND && (!live || k > TEL_K_SHORT) {
        let mut q_sta: ::core::ffi::c_double = 0.;
        let mut m_sta: ::core::ffi::c_double = 0.;
        tel_air(c, (*p).h, (*c).v_station, &raw mut q_sta, &raw mut m_sta);
        let mut w: ::core::ffi::c_double = fmax(0.0f64, along) / TEL_BLEND;
        q_ref = w * q_ref + (1.0f64 - w) * q_sta;
    }
    let mut gravity: ::core::ffi::c_double = tel_gravity(c, (*p).h);
    let mut a1: ::core::ffi::c_double = tel_lift_aoa(c, p, (*p).m * gravity);
    tel_forces(c, p, a1, &raw mut q, &raw mut mach, &raw mut lift, &raw mut drag);
    let mut ld: ::core::ffi::c_double = if drag > 0.0f64 {
        fmax(0.3f64, lift / drag)
    } else {
        1.0f64
    };
    let mut g_cmd: ::core::ffi::c_double = -atan(1.0f64 / ld)
        - 0.6f64 * clampd((q_ref - q) / q_ref, -0.6f64, 0.6f64);
    if along > 0.0f64 && along < TEL_AIM {
        let mut g_aim: ::core::ffi::c_double = fmin(
            -atan2((*p).h - (*c).h_station, fmax(500.0f64, along)),
            -atan(1.0f64 / ld),
        );
        let mut w_0: ::core::ffi::c_double = fmin(1.0f64, (TEL_AIM - along) / 3000.0f64);
        g_cmd = (1.0f64 - w_0) * g_cmd + w_0 * g_aim;
    } else if live as ::core::ffi::c_int != 0 && along <= 0.0f64
        && (k > TEL_K_SHORT || !tel_delivery_policy(c))
    {
        let mut ogs: ::core::ffi::c_double = atan((*c).h_station / (*c).d_station);
        let mut dist: ::core::ffi::c_double = fmax(1000.0f64, -(*p).x);
        g_cmd = -ogs - 0.5f64 * ((*p).h - dist * tan(ogs)) / dist;
        g_cmd = clampd(g_cmd, -35.0f64 * DEG2RAD, -10.0f64 * DEG2RAD);
    }
    let mut turn_cos: ::core::ffi::c_double = fmax(
        0.3f64,
        cos(fmin(TEL_BANK_MAX, (*(*c).model).vehicle.maximum_bank_angle) * DEG2RAD),
    );
    let mut turn_speed: ::core::ffi::c_double = safe_speed
        / sqrt((if (*p).v < safe_speed { turn_cos } else { bank_cos }));
    if tel_delivery_policy(c) as ::core::ffi::c_int != 0 && safe_speed > 0.0f64
        && (*p).v < turn_speed
    {
        let mut actual_q: ::core::ffi::c_double = 0.;
        let mut actual_mach: ::core::ffi::c_double = 0.;
        let mut actual_lift: ::core::ffi::c_double = 0.;
        let mut actual_drag: ::core::ffi::c_double = 0.;
        tel_forces(
            c,
            p,
            (*p).aoa,
            &raw mut actual_q,
            &raw mut actual_mach,
            &raw mut actual_lift,
            &raw mut actual_drag,
        );
        let mut recover: ::core::ffi::c_double = actual_drag / (*p).m
            + (turn_speed - (*p).v) / 4.0f64;
        g_cmd = fmin(g_cmd, -asin(clampd(recover / gravity, 0.0f64, 1.0f64)));
    }
    g_cmd = clampd(g_cmd, -35.0f64 * DEG2RAD, 6.0f64 * DEG2RAD);
    let mut gdot: ::core::ffi::c_double = clampd(
        (g_cmd - (*p).g) / 4.0f64,
        -0.08f64,
        0.08f64,
    );
    let mut cphi: ::core::ffi::c_double = fmax(0.3f64, cos(fabs(bank_deg) * DEG2RAD));
    let mut lift_req: ::core::ffi::c_double = (*p).m
        * (gravity * cos((*p).g) + (*p).v * gdot) / cphi;
    return clampd(tel_lift_aoa(c, p, lift_req), TEL_AOA_MIN, (*c).aoa_max);
}
unsafe extern "C" fn tel_servo(
    mut v: *mut ::core::ffi::c_double,
    mut r: *mut ::core::ffi::c_double,
    mut cmd: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) {
    let rate_max: ::core::ffi::c_double = 5.8f64;
    let accel_max: ::core::ffi::c_double = 14.0f64;
    let wn: ::core::ffi::c_double = 1.5f64;
    let mut want: ::core::ffi::c_double = clampd(wn * (cmd - *v), -rate_max, rate_max);
    let mut stop: ::core::ffi::c_double = sqrt(2.0f64 * accel_max * fabs(cmd - *v));
    want = clampd(want, -stop, stop);
    let mut acc: ::core::ffi::c_double = clampd((want - *r) / dt, -accel_max, accel_max);
    *r += acc * dt;
    *v += *r * dt;
}
unsafe extern "C" fn tel_predict_range(
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut k: ::core::ffi::c_double,
    mut along: ::core::ffi::c_double,
    mut segs: *const TelSegment,
    mut nseg: ::core::ffi::c_int,
) -> ::core::ffi::c_double {
    return tel_predict_range_ex(
        c,
        s,
        k,
        along,
        segs,
        nseg,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
}
unsafe extern "C" fn tel_predict_range_ex(
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut k: ::core::ffi::c_double,
    mut along: ::core::ffi::c_double,
    mut segs: *const TelSegment,
    mut nseg: ::core::ffi::c_int,
    mut bank_over_deg: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut p: TelPoint = *s;
    p.x = 0.0f64;
    p.y = 0.0f64;
    p.psi = 0.0f64;
    let mut dt: ::core::ffi::c_double = 1.0f64;
    let mut dist: ::core::ffi::c_double = 0.0f64;
    let mut used: ::core::ffi::c_double = 0.0f64;
    let mut t: ::core::ffi::c_double = 0.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut rr: ::core::ffi::c_double = (*(*c).model).world.radius_m
        + (*(*c).model).site.altitude;
    while t < 1500.0f64 {
        let mut e: ::core::ffi::c_double = p.h + p.v * p.v / (2.0f64 * TEL_G0);
        if e <= (*c).e_station || p.h < 0.0f64 {
            break;
        }
        while i < nseg && used >= (*segs.offset(i as isize)).length {
            used -= (*segs.offset(i as isize)).length;
            i += 1;
        }
        let mut rad: ::core::ffi::c_double = if i < nseg {
            (*segs.offset(i as isize)).radius
        } else {
            0.0f64
        };
        let mut gnd: ::core::ffi::c_double = p.v * cos(p.g);
        let mut need: ::core::ffi::c_double = if rad > 0.0f64 {
            atan(gnd * gnd / (TEL_G0 * rad)) * RAD2DEG
        } else {
            0.0f64
        };
        if !bank_over_deg.is_null() && i > 0 as ::core::ffi::c_int && i < nseg {
            *bank_over_deg = fmax(*bank_over_deg, need - TEL_BANK_TURN);
        }
        let mut bank: ::core::ffi::c_double = fmin(TEL_BANK_MAX, need);
        let mut aoa_cmd: ::core::ffi::c_double = tel_vertical(
            c,
            &raw mut p,
            bank,
            k,
            along - dist,
            false_0 != 0,
        );
        tel_servo(&raw mut p.aoa, &raw mut p.aoa_rate, aoa_cmd, dt);
        let mut q: ::core::ffi::c_double = 0.;
        let mut mach: ::core::ffi::c_double = 0.;
        let mut lift: ::core::ffi::c_double = 0.;
        let mut drag: ::core::ffi::c_double = 0.;
        tel_forces(
            c,
            &raw mut p,
            p.aoa,
            &raw mut q,
            &raw mut mach,
            &raw mut lift,
            &raw mut drag,
        );
        let mut v: ::core::ffi::c_double = fmax(p.v, 1.0f64);
        let mut grav: ::core::ffi::c_double = tel_gravity(c, p.h);
        let mut phi: ::core::ffi::c_double = bank * DEG2RAD;
        let mut dv: ::core::ffi::c_double = -drag / p.m - grav * sin(p.g);
        let mut dg: ::core::ffi::c_double = lift * cos(phi) / (p.m * v)
            - (grav / v - v / (rr + p.h)) * cos(p.g);
        let mut d: ::core::ffi::c_double = v * cos(p.g) * dt;
        p.v += dv * dt;
        p.g += dg * dt;
        p.h += v * sin(p.g) * dt;
        dist += d;
        used += d;
        t += dt;
    }
    return dist;
}
unsafe extern "C" fn tel_segments(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut out: *mut TelSegment,
) -> ::core::ffi::c_int {
    let mut path: [TelSegment; 4] = [
        TelSegment {
            length: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radius: 0.,
        },
        TelSegment {
            length: 0.,
            radius: 0.,
        },
        TelSegment {
            length: 0.,
            radius: 0.,
        },
        TelSegment {
            length: 0.,
            radius: 0.,
        },
    ];
    let mut n: ::core::ffi::c_int = tel_segments_path(
        L,
        c,
        s,
        &raw mut path as *mut TelSegment,
    );
    let mut turn: ::core::ffi::c_double = 0.;
    let mut tx: ::core::ffi::c_double = 0.;
    let mut ty: ::core::ffi::c_double = 0.;
    let mut hac_arc: ::core::ffi::c_double = 0.;
    if (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int
        && tel_hac_tangent(
            L,
            c,
            (*s).x,
            (*s).y,
            &raw mut tx,
            &raw mut ty,
            &raw mut hac_arc,
        ) as ::core::ffi::c_int != 0
    {
        turn = fabs(tel_wrap(atan2(ty - (*s).y, tx - (*s).x) - (*s).psi));
    } else if (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int && !(*L).loop_0 {
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < n {
            *out.offset(i as isize) = path[i as usize];
            i += 1;
        }
        return n;
    } else if (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int
        || (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int
    {
        turn = fabs((*s).psi)
            + fabs(
                atan2(
                    (*s).y,
                    fmax(3000.0f64, 0.4f64 * fmax(0.0f64, -(*c).d_station - (*s).x)),
                ),
            );
    } else {
        let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_0 < n {
            *out.offset(i_0 as isize) = path[i_0 as usize];
            i_0 += 1;
        }
        return n;
    }
    let mut gnd: ::core::ffi::c_double = (*s).v * cos((*s).g);
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut lim: ::core::ffi::c_double = (if mach > 1.1f64 {
        TEL_BANK_SUP
    } else {
        TEL_BANK_MAX
    }) * DEG2RAD;
    let mut rad: ::core::ffi::c_double = gnd * gnd / (TEL_G0 * tan(lim));
    let mut arc: ::core::ffi::c_double = fmin(
        rad * turn,
        path[0 as ::core::ffi::c_int as usize].length,
    );
    let mut m: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    if arc > 100.0f64 {
        let fresh5 = m;
        m = m + 1;
        *out.offset(fresh5 as isize) = TelSegment {
            length: arc,
            radius: rad,
        };
    }
    let fresh6 = m;
    m = m + 1;
    *out.offset(fresh6 as isize) = TelSegment {
        length: path[0 as ::core::ffi::c_int as usize].length - arc,
        radius: 0.0f64,
    };
    let mut i_1: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
    while i_1 < n {
        let fresh7 = m;
        m = m + 1;
        *out.offset(fresh7 as isize) = path[i_1 as usize];
        i_1 += 1;
    }
    return m;
}
unsafe extern "C" fn tel_hac_tangent(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut x: ::core::ffi::c_double,
    mut y: ::core::ffi::c_double,
    mut tx: *mut ::core::ffi::c_double,
    mut ty: *mut ::core::ffi::c_double,
    mut arc: *mut ::core::ffi::c_double,
) -> bool {
    let mut r: ::core::ffi::c_double = (*L).radius;
    let mut cx: ::core::ffi::c_double = -(*c).d_station;
    let mut cy: ::core::ffi::c_double = (*L).side as ::core::ffi::c_double * r;
    let mut dx: ::core::ffi::c_double = x - cx;
    let mut dy: ::core::ffi::c_double = y - cy;
    let mut d: ::core::ffi::c_double = hypot(dx, dy);
    if d <= r * 1.02f64 {
        return false_0 != 0;
    }
    let mut ang: ::core::ffi::c_double = atan2(dy, dx)
        + (*L).side as ::core::ffi::c_double * acos(r / d);
    *tx = cx + r * cos(ang);
    *ty = cy + r * sin(ang);
    let mut end: ::core::ffi::c_double = -(*L).side as ::core::ffi::c_double * TEL_PI
        / 2.0f64;
    *arc = fmod((*L).side as ::core::ffi::c_double * (end - ang), 2.0f64 * TEL_PI);
    if *arc < 0.0f64 {
        *arc += 2.0f64 * TEL_PI;
    }
    *arc *= r;
    return true_0 != 0;
}
unsafe extern "C" fn tel_hac_remaining(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut x: ::core::ffi::c_double,
    mut y: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = (*L).radius;
    let mut cx: ::core::ffi::c_double = -(*c).d_station;
    let mut cy: ::core::ffi::c_double = (*L).side as ::core::ffi::c_double * r;
    let mut ang: ::core::ffi::c_double = atan2(y - cy, x - cx);
    let mut end: ::core::ffi::c_double = -(*L).side as ::core::ffi::c_double * TEL_PI
        / 2.0f64;
    let mut rem: ::core::ffi::c_double = fmod(
        (*L).side as ::core::ffi::c_double * (end - ang),
        2.0f64 * TEL_PI,
    );
    if rem < 0.0f64 {
        rem += 2.0f64 * TEL_PI;
    }
    if rem > 1.9f64 * TEL_PI {
        rem = 0.0f64;
    }
    return r * rem;
}
unsafe extern "C" fn tel_mean_turn_radius(
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> ::core::ffi::c_double {
    let mut gnd: ::core::ffi::c_double = (*s).v * cos((*s).g);
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut lim: ::core::ffi::c_double = (if mach > 1.1f64 {
        TEL_BANK_SUP
    } else {
        TEL_BANK_MAX
    }) * DEG2RAD;
    let mut v2: ::core::ffi::c_double = 0.5f64
        * (gnd * gnd + (*c).v_station * (*c).v_station);
    return v2 / (TEL_G0 * tan(lim));
}
unsafe extern "C" fn tel_dubins_to_station(
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut r: ::core::ffi::c_double,
    mut out: *mut TelSegment,
) -> ::core::ffi::c_int {
    let mut ex: ::core::ffi::c_double = -(*c).d_station;
    let mut ey: ::core::ffi::c_double = 0.0f64;
    let mut epsi: ::core::ffi::c_double = 0.0f64;
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut a1: ::core::ffi::c_double = 0.0f64;
    let mut st: ::core::ffi::c_double = 0.0f64;
    let mut a2: ::core::ffi::c_double = 0.0f64;
    let mut d1: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while d1 <= 1 as ::core::ffi::c_int {
        let mut current_block_14: u64;
        let mut d2: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
        while d2 <= 1 as ::core::ffi::c_int {
            let mut c1x: ::core::ffi::c_double = (*s).x
                - d1 as ::core::ffi::c_double * r * sin((*s).psi);
            let mut c1y: ::core::ffi::c_double = (*s).y
                + d1 as ::core::ffi::c_double * r * cos((*s).psi);
            let mut c2x: ::core::ffi::c_double = ex
                - d2 as ::core::ffi::c_double * r * sin(epsi);
            let mut c2y: ::core::ffi::c_double = ey
                + d2 as ::core::ffi::c_double * r * cos(epsi);
            let mut dx: ::core::ffi::c_double = c2x - c1x;
            let mut dy: ::core::ffi::c_double = c2y - c1y;
            let mut d: ::core::ffi::c_double = hypot(dx, dy);
            let mut phi: ::core::ffi::c_double = atan2(dy, dx);
            let mut theta: ::core::ffi::c_double = 0.;
            let mut straight: ::core::ffi::c_double = 0.;
            if d1 == d2 {
                theta = phi;
                straight = d;
                current_block_14 = 13536709405535804910;
            } else if d < 2.0f64 * r {
                current_block_14 = 14155750587950065367;
            } else {
                theta = phi + d1 as ::core::ffi::c_double * asin(2.0f64 * r / d);
                straight = sqrt(d * d - 4.0f64 * r * r);
                current_block_14 = 13536709405535804910;
            }
            match current_block_14 {
                13536709405535804910 => {
                    let mut arc1: ::core::ffi::c_double = fmod(
                        d1 as ::core::ffi::c_double * (theta - (*s).psi),
                        2.0f64 * TEL_PI,
                    );
                    if arc1 < 0.0f64 {
                        arc1 += 2.0f64 * TEL_PI;
                    }
                    let mut arc2: ::core::ffi::c_double = fmod(
                        d2 as ::core::ffi::c_double * (epsi - theta),
                        2.0f64 * TEL_PI,
                    );
                    if arc2 < 0.0f64 {
                        arc2 += 2.0f64 * TEL_PI;
                    }
                    let mut total: ::core::ffi::c_double = r * (arc1 + arc2) + straight;
                    if total < best {
                        best = total;
                        a1 = r * arc1;
                        st = straight;
                        a2 = r * arc2;
                    }
                }
                _ => {}
            }
            d2 += 2 as ::core::ffi::c_int;
        }
        d1 += 2 as ::core::ffi::c_int;
    }
    if best.is_finite() as i32 == 0 {
        *out.offset(0 as ::core::ffi::c_int as isize) = TelSegment {
            length: hypot(ex - (*s).x, ey - (*s).y),
            radius: 0.0f64,
        };
        return 1 as ::core::ffi::c_int;
    }
    *out.offset(0 as ::core::ffi::c_int as isize) = TelSegment {
        length: a1,
        radius: r,
    };
    *out.offset(1 as ::core::ffi::c_int as isize) = TelSegment {
        length: st,
        radius: 0.0f64,
    };
    *out.offset(2 as ::core::ffi::c_int as isize) = TelSegment {
        length: a2,
        radius: r,
    };
    return 3 as ::core::ffi::c_int;
}
unsafe extern "C" fn tel_segments_path(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut out: *mut TelSegment,
) -> ::core::ffi::c_int {
    let mut r: ::core::ffi::c_double = (*L).radius;
    let mut D: ::core::ffi::c_double = (*c).d_station;
    let mut tx: ::core::ffi::c_double = 0.;
    let mut ty: ::core::ffi::c_double = 0.;
    let mut arc: ::core::ffi::c_double = 0.;
    if (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int {
        *out.offset(0 as ::core::ffi::c_int as isize) = TelSegment {
            length: hypot(fmax(0.0f64, -D - (*s).x), (*s).y),
            radius: 0.0f64,
        };
        return 1 as ::core::ffi::c_int;
    }
    if !(*L).loop_0 {
        return tel_dubins_to_station(c, s, tel_mean_turn_radius(c, s), out);
    }
    if (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int {
        let mut q: TaemEnergyLawState = *L;
        q.side = if (*s).y <= 0.0f64 {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        };
        let mut rt: ::core::ffi::c_double = TEL_R_TURN;
        let mut ex: ::core::ffi::c_double = fmax((*L).x_turn, (*s).x);
        let mut ey: ::core::ffi::c_double = (*s).y
            + 2.0f64 * q.side as ::core::ffi::c_double * rt;
        *out.offset(0 as ::core::ffi::c_int as isize) = TelSegment {
            length: fmax(0.0f64, (*L).x_turn - (*s).x),
            radius: 0.0f64,
        };
        *out.offset(1 as ::core::ffi::c_int as isize) = TelSegment {
            length: TEL_PI * rt,
            radius: rt,
        };
        if tel_hac_tangent(
            &raw mut q,
            c,
            ex,
            ey,
            &raw mut tx,
            &raw mut ty,
            &raw mut arc,
        ) {
            *out.offset(2 as ::core::ffi::c_int as isize) = TelSegment {
                length: hypot(tx - ex, ty - ey),
                radius: 0.0f64,
            };
            *out.offset(3 as ::core::ffi::c_int as isize) = TelSegment {
                length: arc,
                radius: r,
            };
        } else {
            *out.offset(2 as ::core::ffi::c_int as isize) = TelSegment {
                length: 0.0f64,
                radius: 0.0f64,
            };
            *out.offset(3 as ::core::ffi::c_int as isize) = TelSegment {
                length: tel_hac_remaining(&raw mut q, c, ex, ey),
                radius: r,
            };
        }
        return 4 as ::core::ffi::c_int;
    }
    if (*L).leg == TEL_LEG_TURN as ::core::ffi::c_int
        || (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int
    {
        if !tel_hac_tangent(
            L,
            c,
            (*s).x,
            (*s).y,
            &raw mut tx,
            &raw mut ty,
            &raw mut arc,
        ) {
            *out.offset(0 as ::core::ffi::c_int as isize) = TelSegment {
                length: tel_hac_remaining(L, c, (*s).x, (*s).y),
                radius: r,
            };
            return 1 as ::core::ffi::c_int;
        }
        let mut rem: ::core::ffi::c_double = 0.0f64;
        if (*L).leg == TEL_LEG_TURN as ::core::ffi::c_int {
            let mut brg: ::core::ffi::c_double = atan2(ty - (*s).y, tx - (*s).x);
            rem = fmod(
                (*L).side as ::core::ffi::c_double * (brg - (*s).psi),
                2.0f64 * TEL_PI,
            );
            if rem < 0.0f64 {
                rem += 2.0f64 * TEL_PI;
            }
            if rem > 1.9f64 * TEL_PI {
                rem = 0.0f64;
            }
        }
        let mut n: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        if rem * (*L).r_turn > 50.0f64 {
            let fresh8 = n;
            n = n + 1;
            *out.offset(fresh8 as isize) = TelSegment {
                length: rem * (*L).r_turn,
                radius: (*L).r_turn,
            };
        }
        let fresh9 = n;
        n = n + 1;
        *out.offset(fresh9 as isize) = TelSegment {
            length: fmax(
                0.0f64,
                hypot(tx - (*s).x, ty - (*s).y) - rem * (*L).r_turn * 0.6f64,
            ),
            radius: 0.0f64,
        };
        let fresh10 = n;
        n = n + 1;
        *out.offset(fresh10 as isize) = TelSegment {
            length: arc,
            radius: r,
        };
        return n;
    }
    *out.offset(0 as ::core::ffi::c_int as isize) = TelSegment {
        length: fmax(0.0f64, (*L).hac_left_rad) * r,
        radius: r,
    };
    return 1 as ::core::ffi::c_int;
}
unsafe extern "C" fn tel_sum(
    mut s: *const TelSegment,
    mut n: ::core::ffi::c_int,
) -> ::core::ffi::c_double {
    let mut a: ::core::ffi::c_double = 0.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < n {
        a += (*s.offset(i as isize)).length;
        i += 1;
    }
    return a;
}
unsafe extern "C" fn tel_solve_k(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut s_req: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut segs: [TelSegment; 5] = [TelSegment {
        length: 0.,
        radius: 0.,
    }; 5];
    let mut n: ::core::ffi::c_int = tel_segments(
        L,
        c,
        s,
        &raw mut segs as *mut TelSegment,
    );
    let mut s_go: ::core::ffi::c_double = tel_sum(&raw mut segs as *mut TelSegment, n);
    let mut r_lo: ::core::ffi::c_double = if (*L).capture_forecast as ::core::ffi::c_int
        != 0
    {
        tel_predict_pattern(
            L,
            c,
            s,
            0.0f64,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        )
    } else {
        tel_predict_range(c, s, 0.0f64, s_go, &raw mut segs as *mut TelSegment, n)
    };
    let mut r_hi: ::core::ffi::c_double = if (*L).capture_forecast as ::core::ffi::c_int
        != 0
    {
        tel_predict_pattern(
            L,
            c,
            s,
            1.0f64,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        )
    } else {
        tel_predict_range(c, s, 1.0f64, s_go, &raw mut segs as *mut TelSegment, n)
    };
    if r_hi >= s_go {
        *s_req = r_hi;
        return 1.0f64;
    }
    if r_lo <= s_go {
        *s_req = r_lo;
        return 0.0f64;
    }
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let mut hi: ::core::ffi::c_double = 1.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 7 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        let mut reach: ::core::ffi::c_double = if (*L).capture_forecast
            as ::core::ffi::c_int != 0
        {
            tel_predict_pattern(
                L,
                c,
                s,
                mid,
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
            )
        } else {
            tel_predict_range(c, s, mid, s_go, &raw mut segs as *mut TelSegment, n)
        };
        if reach > s_go {
            lo = mid;
        } else {
            hi = mid;
        }
        i += 1;
    }
    *s_req = s_go;
    return 0.5f64 * (lo + hi);
}
unsafe extern "C" fn tel_pattern_excess(
    mut L: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> ::core::ffi::c_double {
    let mut segs: [TelSegment; 5] = [TelSegment {
        length: 0.,
        radius: 0.,
    }; 5];
    let mut n: ::core::ffi::c_int = tel_segments(
        L,
        c,
        s,
        &raw mut segs as *mut TelSegment,
    );
    return (if (*L).capture_forecast as ::core::ffi::c_int != 0 {
        tel_predict_pattern(
            L,
            c,
            s,
            TEL_K_PLAN,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        )
    } else {
        tel_predict_range(c, s, TEL_K_PLAN, 1e9f64, &raw mut segs as *mut TelSegment, n)
    }) - tel_sum(&raw mut segs as *mut TelSegment, n);
}
unsafe extern "C" fn tel_size_hac(
    mut L: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut set_turn: bool,
) {
    let mut v_hac: ::core::ffi::c_double = clampd(
        (*s).v * cos((*s).g),
        130.0f64,
        170.0f64,
    );
    let mut r_fly: ::core::ffi::c_double = v_hac * v_hac
        / (TEL_G0 * tan(35.0f64 * DEG2RAD));
    static mut radii: [::core::ffi::c_double; 13] = [
        TEL_R_MIN,
        2600.0f64,
        3000.0f64,
        3500.0f64,
        4000.0f64,
        4500.0f64,
        5000.0f64,
        5500.0f64,
        6000.0f64,
        7000.0f64,
        8000.0f64,
        9000.0f64,
        TEL_R_MAX,
    ];
    let mut best_r: ::core::ffi::c_double = (*L).radius;
    let mut best_e: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[::core::ffi::c_double; 13]>() as usize)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
    {
        if !(i > 0 as size_t && radii[i as usize] <= r_fly) {
            (*L).radius = fmax(radii[i as usize], r_fly);
            if set_turn {
                (*L).x_turn = -(*c).d_station;
            }
            let mut ext: ::core::ffi::c_double = tel_pattern_excess(L, c, s);
            if fabs(ext) < fabs(best_e) {
                best_e = ext;
                best_r = (*L).radius;
            }
            if set_turn as ::core::ffi::c_int != 0 && ext <= 0.0f64 {
                break;
            }
        }
        i = i.wrapping_add(1);
    }
    (*L).radius = best_r;
    if set_turn {
        (*L).x_turn = -(*c).d_station + (*L).radius + 500.0f64
            + fmax(0.0f64, best_e) / 2.0f64;
    }
}
unsafe extern "C" fn tel_entry_arc(
    mut L: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut rem: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = (*L).radius;
    if rem > TEL_PI {
        return rem;
    }
    let mut none: [TelSegment; 1] = [
        TelSegment {
            length: rem * r,
            radius: r,
        },
    ];
    let mut excess: ::core::ffi::c_double = tel_predict_range(
        c,
        s,
        1.0f64,
        1e9f64,
        &raw mut none as *mut TelSegment,
        1 as ::core::ffi::c_int,
    ) - rem * r;
    let mut vg: ::core::ffi::c_double = (*s).v;
    let mut r_small: ::core::ffi::c_double = fmax(
        TEL_R_SPIRAL_MIN,
        vg * vg / (TEL_G0 * tan(TEL_BANK_TURN * DEG2RAD)),
    );
    let mut turn: ::core::ffi::c_double = rem + 2.0f64 * TEL_PI;
    let mut small: [TelSegment; 1] = [
        TelSegment {
            length: turn * r_small,
            radius: r_small,
        },
    ];
    let mut shortfall: ::core::ffi::c_double = turn * r_small
        - tel_predict_range(
            c,
            s,
            0.0f64,
            1e9f64,
            &raw mut small as *mut TelSegment,
            1 as ::core::ffi::c_int,
        );
    let mut mid: [TelSegment; 1] = [
        TelSegment {
            length: turn * r,
            radius: r,
        },
    ];
    let mut reach: ::core::ffi::c_double = tel_predict_range(
        c,
        s,
        TEL_K_PLAN,
        1e9f64,
        &raw mut mid as *mut TelSegment,
        1 as ::core::ffi::c_int,
    );
    let mut circle: bool = excess
        > fmax(0.0f64, shortfall) + (if rem > 0.35f64 { 2000.0f64 } else { 0.0f64 });
    if !getenv(b"KSP_LANDER_TEL_TRACE\0" as *const u8 as *const ::core::ffi::c_char)
        .is_null()
    {
        fprintf(
            stderr,
            b"TEL_ENTRY rem=%.2f r=%.0f excess=%.0f shortfall=%.0f r_small=%.0f reach=%.0f circle=%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            rem,
            r,
            excess,
            shortfall,
            r_small,
            reach,
            circle as ::core::ffi::c_int,
        );
    }
    if !circle {
        return rem;
    }
    if rem <= 0.35f64 {
        (*L).radius = clampd(reach / turn, r_small, TEL_R_MAX);
    }
    return turn;
}
unsafe extern "C" fn tel_line_lookahead(
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut mach: ::core::ffi::c_double,
    mut gnd: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut look: ::core::ffi::c_double = clampd(
        0.4f64 * fmax(0.0f64, -(*c).d_station - (*s).x),
        3000.0f64,
        12000.0f64,
    );
    if mach > 1.1f64 {
        look = fmax(look, 45.0f64 * gnd);
    }
    return look;
}
unsafe extern "C" fn tel_effective_station(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut mach: ::core::ffi::c_double,
    mut gnd: ::core::ffi::c_double,
) -> TelContext {
    let mut e: TelContext = *c;
    if (*L).loop_0 as ::core::ffi::c_int != 0
        && (!tel_delivery_policy(c) || (*L).leg != TEL_LEG_FINAL as ::core::ffi::c_int)
        || (*L).leg != TEL_LEG_OUT as ::core::ffi::c_int
            && (*L).leg != TEL_LEG_FINAL as ::core::ffi::c_int
    {
        return e;
    }
    let mut x_cap: ::core::ffi::c_double = if (*L).leg
        == TEL_LEG_FINAL as ::core::ffi::c_int
    {
        (*s).x
    } else {
        (*s).x + tel_line_lookahead(c, s, mach, gnd)
    };
    if !(x_cap > -(*c).d_station) {
        return e;
    }
    e.d_station = fmax(500.0f64, -x_cap);
    e.h_station = e.d_station * tan(e.slope_deg * DEG2RAD);
    e.e_station = e.h_station + e.v_station * e.v_station / (2.0f64 * TEL_G0);
    return e;
}
unsafe extern "C" fn tel_line_aligned(
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> bool {
    if c.is_null() || s.is_null() || !((*c).d_station > 0.0f64)
        || (*s).x.is_finite() as i32 == 0 || (*s).y.is_finite() as i32 == 0
        || (*s).psi.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    return fabs((*s).y) <= fmin(2000.0f64, 0.25f64 * (*c).d_station)
        && fabs(tel_wrap((*s).psi)) <= 15.0f64 * DEG2RAD;
}
unsafe extern "C" fn tel_final_line_ready(
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> bool {
    return tel_line_aligned(c, s) as ::core::ffi::c_int != 0
        && -(*c).d_station - (*s).x < 6000.0f64;
}
unsafe extern "C" fn tel_lateral(
    mut L: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> ::core::ffi::c_double {
    let mut bank_cmd: ::core::ffi::c_double = 0.;
    let mut D: ::core::ffi::c_double = (*c).d_station;
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut gnd: ::core::ffi::c_double = if (*s).ground_speed > 0.0f64 {
        (*s).ground_speed
    } else {
        (*s).v * cos((*s).g)
    };
    if (*L).loop_0 as ::core::ffi::c_int != 0
        && (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int && (*s).x >= (*L).x_turn
    {
        (*L).leg = TEL_LEG_TURN as ::core::ffi::c_int;
        (*L).side = if (*s).y <= 0.0f64 {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        };
        (*L).r_turn = fmax(TEL_R_TURN, gnd * gnd / (TEL_G0 * tan(45.0f64 * DEG2RAD)));
    }
    let mut r: ::core::ffi::c_double = (*L).radius;
    let mut bank_ff: ::core::ffi::c_double = 0.0f64;
    let mut course_cmd: ::core::ffi::c_double = 0.;
    let mut side: ::core::ffi::c_int = (*L).side;
    if (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int
        || (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int
    {
        let mut look: ::core::ffi::c_double = tel_line_lookahead(c, s, mach, gnd);
        course_cmd = if (*L).capture_forecast as ::core::ffi::c_int != 0 && !(*L).loop_0
            && fabs((*s).y) > 6000.0f64
        {
            atan2(-(*s).y, -D - (*s).x)
        } else {
            atan2(-(*s).y, look)
        };
        if !(*L).loop_0 && tel_final_line_ready(c, s) as ::core::ffi::c_int != 0 {
            (*L).leg = TEL_LEG_FINAL as ::core::ffi::c_int;
        }
        if tel_delivery_policy(c) as ::core::ffi::c_int != 0
            && (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int && (*s).x < 0.0f64
            && (*(*c).model).site.runway_length > 0.0f64
            && (*(*c).model).site.runway_width > 0.0f64
        {
            let mut ratio: ::core::ffi::c_double = 0.;
            if runway_landing_intercept_ratio(
                (*s).x,
                -(*s).y,
                &raw const (*(*c).model).site,
                &raw const (*(*c).model).guidance,
                &raw mut ratio,
            ) {
                course_cmd = -atan(ratio);
            }
        }
    } else if (*L).leg == TEL_LEG_TURN as ::core::ffi::c_int
        || (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int
    {
        let mut tx: ::core::ffi::c_double = 0.;
        let mut ty: ::core::ffi::c_double = 0.;
        let mut arc: ::core::ffi::c_double = 0.;
        if !tel_hac_tangent(
            L,
            c,
            (*s).x,
            (*s).y,
            &raw mut tx,
            &raw mut ty,
            &raw mut arc,
        ) {
            (*L).leg = TEL_LEG_HAC as ::core::ffi::c_int;
            let mut cx: ::core::ffi::c_double = -D;
            let mut cy: ::core::ffi::c_double = side as ::core::ffi::c_double * r;
            let mut ang: ::core::ffi::c_double = atan2((*s).y - cy, (*s).x - cx);
            let mut end: ::core::ffi::c_double = -side as ::core::ffi::c_double * TEL_PI
                / 2.0f64;
            let mut rem: ::core::ffi::c_double = fmod(
                side as ::core::ffi::c_double * (end - ang),
                2.0f64 * TEL_PI,
            );
            if rem < 0.0f64 {
                rem += 2.0f64 * TEL_PI;
            }
            if rem > 1.9f64 * TEL_PI {
                let mut full: [TelSegment; 1] = [
                    TelSegment {
                        length: rem * r,
                        radius: r,
                    },
                ];
                if tel_predict_range(
                    c,
                    s,
                    TEL_K_PLAN,
                    1e9f64,
                    &raw mut full as *mut TelSegment,
                    1 as ::core::ffi::c_int,
                ) < 0.5f64 * rem * r
                {
                    rem = 0.0f64;
                }
            }
            (*L).hac_left_rad = tel_entry_arc(L, c, s, rem);
            (*L).hac_last_ang = ::core::f32::NAN as ::core::ffi::c_double;
            course_cmd = (*s).psi;
        } else {
            let mut brg: ::core::ffi::c_double = atan2(ty - (*s).y, tx - (*s).x);
            course_cmd = brg;
            if (*L).leg == TEL_LEG_TURN as ::core::ffi::c_int {
                let mut rem_0: ::core::ffi::c_double = tel_wrap(brg - (*s).psi);
                if (*L).side as ::core::ffi::c_double * rem_0 < 0.0f64
                    || fabs(rem_0) > 0.35f64
                {
                    bank_ff = side as ::core::ffi::c_double
                        * atan(gnd * gnd / (TEL_G0 * (*L).r_turn)) * RAD2DEG;
                    course_cmd = (*s).psi;
                } else {
                    (*L).leg = TEL_LEG_DOWN as ::core::ffi::c_int;
                }
            }
            if (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int
                && hypot(tx - (*s).x, ty - (*s).y) < fmax(300.0f64, 2.0f64 * gnd)
            {
                (*L).leg = TEL_LEG_HAC as ::core::ffi::c_int;
                (*L).hac_left_rad = tel_entry_arc(
                    L,
                    c,
                    s,
                    arc / fmax(1.0f64, (*L).radius),
                );
                (*L).hac_last_ang = ::core::f32::NAN as ::core::ffi::c_double;
            }
        }
    }
    if (*L).leg == TEL_LEG_HAC as ::core::ffi::c_int {
        r = (*L).radius;
        let mut cx_0: ::core::ffi::c_double = -D;
        let mut cy_0: ::core::ffi::c_double = side as ::core::ffi::c_double * r;
        let mut ang_0: ::core::ffi::c_double = atan2((*s).y - cy_0, (*s).x - cx_0);
        if (*L).hac_last_ang.is_finite() as i32 != 0 {
            (*L).hac_left_rad
                -= side as ::core::ffi::c_double * tel_wrap(ang_0 - (*L).hac_last_ang);
        }
        (*L).hac_last_ang = ang_0;
        let mut r_err: ::core::ffi::c_double = hypot((*s).x - cx_0, (*s).y - cy_0) - r;
        course_cmd = ang_0
            + side as ::core::ffi::c_double
                * (TEL_PI / 2.0f64 + clampd(r_err / 1500.0f64, -0.6f64, 0.6f64));
        bank_ff = side as ::core::ffi::c_double * atan(gnd * gnd / (TEL_G0 * r))
            * RAD2DEG;
        if (*L).hac_left_rad < 0.3f64 && fabs(tel_wrap((*s).psi)) < 0.35f64 {
            (*L).leg = TEL_LEG_FINAL as ::core::ffi::c_int;
            bank_ff = 0.0f64;
            course_cmd = atan2(-(*s).y, 3000.0f64);
        }
    }
    let mut E: ::core::ffi::c_double = (*L).s_req - (*L).s_go;
    let mut tx_0: ::core::ffi::c_double = -D;
    let mut ty_0: ::core::ffi::c_double = 0.0f64;
    let mut arc_0: ::core::ffi::c_double = 0.0f64;
    let mut leg_ok: bool = (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int
        || (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int && !(*L).loop_0;
    if (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int
        && !tel_hac_tangent(
            L,
            c,
            (*s).x,
            (*s).y,
            &raw mut tx_0,
            &raw mut ty_0,
            &raw mut arc_0,
        )
    {
        leg_ok = false_0 != 0;
    }
    let mut d: ::core::ffi::c_double = hypot(tx_0 - (*s).x, ty_0 - (*s).y);
    if (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int {
        d = fmax(0.0f64, -D - 6000.0f64 - (*s).x);
    }
    if leg_ok as ::core::ffi::c_int != 0 && (*L).k > 0.999f64 && E > 300.0f64
        && mach < 0.95f64 && d > 3000.0f64
    {
        let mut brg_0: ::core::ffi::c_double = atan2(ty_0 - (*s).y, tx_0 - (*s).x);
        if !(*L).weave_on {
            (*L).weave_on = true_0 != 0;
            (*L).weave_sign = 1 as ::core::ffi::c_int;
            (*L).weave_brg = brg_0;
        }
        let mut ux: ::core::ffi::c_double = cos((*L).weave_brg);
        let mut uy: ::core::ffi::c_double = sin((*L).weave_brg);
        let mut off: ::core::ffi::c_double = -((*s).x - tx_0) * uy
            + ((*s).y - ty_0) * ux;
        let mut ymax: ::core::ffi::c_double = clampd(0.25f64 * d, 800.0f64, 2500.0f64);
        if (*L).weave_sign > 0 as ::core::ffi::c_int && off > ymax {
            (*L).weave_sign = -(1 as ::core::ffi::c_int);
        } else if (*L).weave_sign < 0 as ::core::ffi::c_int && off < -ymax {
            (*L).weave_sign = 1 as ::core::ffi::c_int;
        }
        let mut psi_w: ::core::ffi::c_double = fmin(
            acos(d / (d + E)),
            50.0f64 * DEG2RAD,
        );
        course_cmd = brg_0 + (*L).weave_sign as ::core::ffi::c_double * psi_w;
        bank_ff = 0.0f64;
    } else {
        (*L).weave_on = false_0 != 0;
    }
    let mut cerr: ::core::ffi::c_double = tel_wrap(course_cmd - (*s).psi);
    let mut omega: ::core::ffi::c_double = clampd(cerr / 6.0f64, -0.12f64, 0.12f64);
    let mut bank_left: ::core::ffi::c_double = bank_ff
        + atan(gnd * omega / TEL_G0) * RAD2DEG;
    let mut lim: ::core::ffi::c_double = if mach > 1.1f64 {
        TEL_BANK_SUP
    } else {
        TEL_BANK_MAX
    };
    if (*L).leg == TEL_LEG_HAC as ::core::ffi::c_int
        || (*L).leg == TEL_LEG_TURN as ::core::ffi::c_int && bank_ff != 0.0f64
    {
        lim = TEL_BANK_ARC;
    }
    lim = fmin(lim, (*(*c).model).vehicle.maximum_bank_angle);
    let mut lift: ::core::ffi::c_double = 0.;
    let mut drag: ::core::ffi::c_double = 0.;
    tel_forces(c, s, (*s).aoa, &raw mut q, &raw mut mach, &raw mut lift, &raw mut drag);
    if tel_delivery_policy(c) {
        lim = entry_bank_authority_limit(
            (*s).v,
            q,
            lift / ((*s).m * TEL_G0),
            &raw const (*(*c).model).vehicle,
            lim,
        );
    }
    bank_cmd = -clampd(bank_left, -lim, lim);
    return bank_cmd;
}
unsafe extern "C" fn tel_predict_pattern_trace(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut k: ::core::ffi::c_double,
    mut bank_over_deg: *mut ::core::ffi::c_double,
    mut trace: *mut Trajectory,
    mut ut: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut route: TaemEnergyLawState = *L;
    route.k = k;
    route.s_go = 0.0f64;
    route.s_req = route.s_go;
    let mut p: TelPoint = *s;
    let mut segs: [TelSegment; 5] = [TelSegment {
        length: 0.,
        radius: 0.,
    }; 5];
    let mut initial: ::core::ffi::c_double = tel_sum(
        &raw mut segs as *mut TelSegment,
        tel_segments(L, c, s, &raw mut segs as *mut TelSegment),
    );
    let dt: ::core::ffi::c_double = 1.0f64;
    let mut rr: ::core::ffi::c_double = (*(*c).model).world.radius_m
        + (*(*c).model).site.altitude;
    let mut delivered: bool = false_0 != 0;
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < 1500 as ::core::ffi::c_uint {
        if !trace.is_null()
            && i.wrapping_rem(10 as ::core::ffi::c_uint) == 0 as ::core::ffi::c_uint
            && p.h >= 0.0f64 && p.v >= 1.0f64
        {
            let mut heading: ::core::ffi::c_double = (*(*c).model).site.runway_heading
                * DEG2RAD;
            let mut origin: GeoPoint = GeoPoint {
                latitude: (*(*c).model).site.latitude,
                longitude: (*(*c).model).site.longitude,
                altitude: (*(*c).model).site.altitude,
            };
            let mut point: GeoPoint = local_point(
                origin,
                p.x * sin(heading) - p.y * cos(heading),
                p.x * cos(heading) + p.y * sin(heading),
                (*(*c).model).world.radius_m,
                (*(*c).model).site.altitude + p.h,
            );
            trajectory_append(
                trace,
                TrajectoryPoint {
                    ut: ut + i as ::core::ffi::c_double * dt,
                    latitude: point.latitude,
                    longitude: point.longitude,
                    altitude: point.altitude,
                    speed: p.v,
                    phase: PHASE_TAEM,
                    kind: TRAJ_REFERENCE,
                },
            );
        }
        if p.h < 0.0f64 || p.v < 1.0f64
            || !tel_delivery_policy(c)
                && p.h + p.v * p.v / (2.0f64 * TEL_G0) <= (*c).e_station
        {
            break;
        }
        if tel_delivery_policy(c) as ::core::ffi::c_int != 0
            && route.leg == TEL_LEG_FINAL as ::core::ffi::c_int && p.x >= -(*c).d_station
            && tel_final_line_ready(c, &raw mut p) as ::core::ffi::c_int != 0
        {
            delivered = true_0 != 0;
            break;
        } else {
            let mut along: ::core::ffi::c_double = tel_sum(
                &raw mut segs as *mut TelSegment,
                tel_segments_path(
                    &raw mut route,
                    c,
                    &raw mut p,
                    &raw mut segs as *mut TelSegment,
                ),
            );
            p.ground_speed = p.v * cos(p.g);
            let mut command: ::core::ffi::c_double = tel_lateral(
                &raw mut route,
                c,
                &raw mut p,
            );
            p.bank += clampd(command - p.bank, -6.0f64 * dt, 6.0f64 * dt);
            if !bank_over_deg.is_null() && route.leg == TEL_LEG_HAC as ::core::ffi::c_int
            {
                let mut gnd: ::core::ffi::c_double = p.v * cos(p.g);
                let mut need: ::core::ffi::c_double = atan(
                    gnd * gnd / (TEL_G0 * route.radius),
                ) * RAD2DEG;
                *bank_over_deg = fmax(*bank_over_deg, need - TEL_BANK_TURN);
            }
            let mut vertical_along: ::core::ffi::c_double = if route.leg
                == TEL_LEG_FINAL as ::core::ffi::c_int
            {
                -(*c).d_station - p.x
            } else {
                along
            };
            let mut vertical: TelContext = *c;
            vertical.final_line = route.leg == TEL_LEG_FINAL as ::core::ffi::c_int;
            let mut target: ::core::ffi::c_double = tel_vertical(
                &raw mut vertical,
                &raw mut p,
                p.bank,
                k,
                vertical_along,
                true_0 != 0,
            );
            tel_servo(&raw mut p.aoa, &raw mut p.aoa_rate, target, dt);
            let mut q: ::core::ffi::c_double = 0.;
            let mut mach: ::core::ffi::c_double = 0.;
            let mut lift: ::core::ffi::c_double = 0.;
            let mut drag: ::core::ffi::c_double = 0.;
            tel_forces(
                c,
                &raw mut p,
                p.aoa,
                &raw mut q,
                &raw mut mach,
                &raw mut lift,
                &raw mut drag,
            );
            let mut v: ::core::ffi::c_double = fmax(p.v, 1.0f64);
            let mut grav: ::core::ffi::c_double = tel_gravity(c, p.h);
            let mut phi: ::core::ffi::c_double = p.bank * DEG2RAD;
            let mut gnd_0: ::core::ffi::c_double = v * cos(p.g);
            let mut dv: ::core::ffi::c_double = -drag / p.m - grav * sin(p.g);
            let mut dg: ::core::ffi::c_double = lift * cos(phi) / (p.m * v)
                - (grav / v - v / (rr + p.h)) * cos(p.g);
            let mut dpsi: ::core::ffi::c_double = -lift * sin(phi)
                / (p.m * fmax(1.0f64, gnd_0));
            p.x += gnd_0 * cos(p.psi) * dt;
            p.y += gnd_0 * sin(p.psi) * dt;
            p.h += v * sin(p.g) * dt;
            p.v += dv * dt;
            p.g += dg * dt;
            p.psi = tel_wrap(p.psi + dpsi * dt);
            i = i.wrapping_add(1);
        }
    }
    if !tel_delivery_policy(c) {
        let mut remaining: ::core::ffi::c_double = tel_sum(
            &raw mut segs as *mut TelSegment,
            tel_segments(&raw mut route, c, &raw mut p, &raw mut segs as *mut TelSegment),
        );
        if route.leg == TEL_LEG_FINAL as ::core::ffi::c_int {
            remaining = -(*c).d_station - p.x + fabs(p.y);
        }
        return initial - remaining;
    }
    let mut slope: ::core::ffi::c_double = tan((*c).slope_deg * DEG2RAD);
    if !(slope > 0.0f64) {
        slope = (*c).h_station / (*c).d_station;
    }
    if !(slope > 0.0f64) {
        slope = 1.0f64;
    }
    let mut energy_margin: ::core::ffi::c_double = p.h + p.v * p.v / (2.0f64 * TEL_G0)
        - (*c).e_station;
    let mut height_margin: ::core::ffi::c_double = p.h - (*c).h_station;
    let mut speed_margin: ::core::ffi::c_double = (p.v * p.v
        - (*c).v_station * (*c).v_station) / (2.0f64 * TEL_G0);
    let mut reserve: ::core::ffi::c_double = fmin(
        energy_margin,
        fmin(height_margin, speed_margin),
    ) / slope;
    let mut remaining_0: ::core::ffi::c_double = tel_sum(
        &raw mut segs as *mut TelSegment,
        tel_segments(&raw mut route, c, &raw mut p, &raw mut segs as *mut TelSegment),
    );
    if delivered {
        remaining_0 = fabs(p.y);
    }
    return initial - remaining_0 + fmin(0.0f64, reserve)
        + (if delivered as ::core::ffi::c_int != 0 {
            fmax(0.0f64, reserve)
        } else {
            0.0f64
        });
}
unsafe extern "C" fn tel_predict_pattern(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut k: ::core::ffi::c_double,
    mut bank_over_deg: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return tel_predict_pattern_trace(
        L,
        c,
        s,
        k,
        bank_over_deg,
        ::core::ptr::null_mut::<Trajectory>(),
        0.0f64,
    );
}
unsafe extern "C" fn tel_plan_open(
    mut L: *const TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> bool {
    let mut D: ::core::ffi::c_double = (*c).d_station;
    let mut tx: ::core::ffi::c_double = 0.;
    let mut ty: ::core::ffi::c_double = 0.;
    let mut arc: ::core::ffi::c_double = 0.;
    if (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int {
        if (*L).loop_0 {
            return (*s).x < (*L).x_turn - 6000.0f64;
        }
        let mut remaining: ::core::ffi::c_double = hypot(-D - (*s).x, (*s).y);
        return (*s).x < -D - TEL_MODE_FREEZE
            || fabs((*s).y) > fmax(6000.0f64, 0.75f64 * D)
                && remaining > D + TEL_MODE_FREEZE;
    }
    if (*L).leg == TEL_LEG_DOWN as ::core::ffi::c_int {
        return tel_hac_tangent(
            L,
            c,
            (*s).x,
            (*s).y,
            &raw mut tx,
            &raw mut ty,
            &raw mut arc,
        ) as ::core::ffi::c_int != 0 && hypot(tx - (*s).x, ty - (*s).y) > 5000.0f64;
    }
    return false_0 != 0;
}
unsafe extern "C" fn tel_score_with_delivery(
    mut P: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
    mut delivery_feasible: *mut bool,
) -> ::core::ffi::c_double {
    let mut segs: [TelSegment; 5] = [TelSegment {
        length: 0.,
        radius: 0.,
    }; 5];
    let mut n: ::core::ffi::c_int = tel_segments(
        P,
        c,
        s,
        &raw mut segs as *mut TelSegment,
    );
    let mut path: ::core::ffi::c_double = tel_sum(&raw mut segs as *mut TelSegment, n);
    let mut r_max: ::core::ffi::c_double = if (*P).capture_forecast as ::core::ffi::c_int
        != 0
    {
        tel_predict_pattern(
            P,
            c,
            s,
            0.0f64,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        )
    } else {
        tel_predict_range(c, s, 0.0f64, 1e9f64, &raw mut segs as *mut TelSegment, n)
    };
    let mut r_min: ::core::ffi::c_double = if (*P).capture_forecast as ::core::ffi::c_int
        != 0
    {
        tel_predict_pattern(
            P,
            c,
            s,
            1.0f64,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        )
    } else {
        tel_predict_range(c, s, 1.0f64, 1e9f64, &raw mut segs as *mut TelSegment, n)
    };
    let mut over: ::core::ffi::c_double = 0.0f64;
    let mut r_mid: ::core::ffi::c_double = if (*P).capture_forecast as ::core::ffi::c_int
        != 0
    {
        tel_predict_pattern(P, c, s, TEL_K_PLAN, &raw mut over)
    } else {
        tel_predict_range_ex(
            c,
            s,
            TEL_K_PLAN,
            1e9f64,
            &raw mut segs as *mut TelSegment,
            n,
            &raw mut over,
        )
    };
    if !delivery_feasible.is_null() {
        *delivery_feasible = r_max >= path && r_min <= path;
    }
    let mut stretch: bool = (*P).loop_0 as ::core::ffi::c_int != 0
        && (*P).leg == TEL_LEG_OUT as ::core::ffi::c_int;
    let mut path_lo: ::core::ffi::c_double = path;
    let mut path_hi: ::core::ffi::c_double = path;
    if (*P).loop_0 as ::core::ffi::c_int != 0
        && (*P).leg == TEL_LEG_DOWN as ::core::ffi::c_int && n > 0 as ::core::ffi::c_int
        && segs[(n - 1 as ::core::ffi::c_int) as usize].radius > 0.0f64
    {
        let mut theta: ::core::ffi::c_double = segs[(n - 1 as ::core::ffi::c_int)
                as usize]
            .length / segs[(n - 1 as ::core::ffi::c_int) as usize].radius;
        path_lo = path
            - theta
                * fmax(
                    0.0f64,
                    segs[(n - 1 as ::core::ffi::c_int) as usize].radius - TEL_R_MIN,
                ) * 0.5f64;
        path_hi = path
            + theta
                * fmax(
                    0.0f64,
                    TEL_R_MAX - segs[(n - 1 as ::core::ffi::c_int) as usize].radius,
                ) * 0.5f64;
    }
    let mut outside: ::core::ffi::c_double = fmax(0.0f64, path_lo - r_max)
        + (if stretch as ::core::ffi::c_int != 0 {
            0.0f64
        } else {
            fmax(0.0f64, r_min - path_hi)
        });
    return outside
        + 0.15f64
            * (if stretch as ::core::ffi::c_int != 0 {
                fmax(0.0f64, path - r_mid)
            } else {
                fabs(r_mid - path)
            }) + (if stretch as ::core::ffi::c_int != 0 { 1000.0f64 } else { 0.0f64 })
        + 1000.0f64 * fmax(0.0f64, over);
}
unsafe extern "C" fn tel_score(
    mut P: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
) -> ::core::ffi::c_double {
    return tel_score_with_delivery(P, c, s, ::core::ptr::null_mut::<bool>());
}
unsafe extern "C" fn tel_plan(
    mut L: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
) {
    static mut radii: [::core::ffi::c_double; 10] = [
        TEL_R_MIN,
        3000.0f64,
        3500.0f64,
        4000.0f64,
        5000.0f64,
        6000.0f64,
        7000.0f64,
        8000.0f64,
        9000.0f64,
        TEL_R_MAX,
    ];
    let mut D: ::core::ffi::c_double = (*c).d_station;
    let mut tx: ::core::ffi::c_double = 0.;
    let mut ty: ::core::ffi::c_double = 0.;
    let mut arc: ::core::ffi::c_double = 0.;
    let mut v_hac: ::core::ffi::c_double = clampd(
        (*s).v * cos((*s).g),
        130.0f64,
        170.0f64,
    );
    let mut r_fly: ::core::ffi::c_double = v_hac * v_hac
        / (TEL_G0 * tan(35.0f64 * DEG2RAD));
    let mut cur: TaemEnergyLawState = *L;
    let mut best: TaemEnergyLawState = *L;
    let mut direct_ready: bool = tel_line_aligned(c, s);
    let mut outbound: bool = fabs(tel_wrap((*s).psi)) > 0.5f64 * TEL_PI;
    let mut patterns_only: bool = tel_delivery_policy(c) as ::core::ffi::c_int != 0
        && !direct_ready || outbound as ::core::ffi::c_int != 0;
    let mut cur_score: ::core::ffi::c_double = if patterns_only as ::core::ffi::c_int
        != 0 && !cur.loop_0
    {
        ::core::f32::INFINITY as ::core::ffi::c_double
    } else {
        tel_score(&raw mut cur, c, s)
    };
    let mut best_score: ::core::ffi::c_double = cur_score;
    let mut p: TaemEnergyLawState = *L;
    p.loop_0 = false_0 != 0;
    p.leg = TEL_LEG_OUT as ::core::ffi::c_int;
    if !outbound
        && (if tel_delivery_policy(c) as ::core::ffi::c_int != 0 {
            direct_ready as ::core::ffi::c_int
        } else {
            ((*s).x < -D - TEL_MODE_FREEZE
                || (*L).capture_forecast as ::core::ffi::c_int != 0
                    && fabs((*s).y) > 6000.0f64) as ::core::ffi::c_int
        }) != 0
    {
        let mut sc: ::core::ffi::c_double = tel_score(&raw mut p, c, s);
        if sc < best_score {
            best_score = sc;
            best = p;
        }
    }
    let mut side: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while side <= 1 as ::core::ffi::c_int {
        let mut i: size_t = 0 as size_t;
        while i
            < (::core::mem::size_of::<[::core::ffi::c_double; 10]>() as usize)
                .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
        {
            let mut forecast_radius: bool = tel_delivery_policy(c) as ::core::ffi::c_int
                != 0 && (*L).capture_forecast as ::core::ffi::c_int != 0;
            if !(!forecast_radius && i > 0 as size_t && radii[i as usize] <= r_fly) {
                p = *L;
                p.loop_0 = true_0 != 0;
                p.leg = TEL_LEG_DOWN as ::core::ffi::c_int;
                p.side = side;
                p.radius = if forecast_radius as ::core::ffi::c_int != 0 {
                    radii[i as usize]
                } else {
                    fmax(radii[i as usize], r_fly)
                };
                if tel_hac_tangent(
                    &raw mut p,
                    c,
                    (*s).x,
                    (*s).y,
                    &raw mut tx,
                    &raw mut ty,
                    &raw mut arc,
                ) {
                    if !(hypot(tx - (*s).x, ty - (*s).y) < 6000.0f64) {
                        let mut sc_0: ::core::ffi::c_double = tel_score(
                            &raw mut p,
                            c,
                            s,
                        );
                        if sc_0 < best_score {
                            best_score = sc_0;
                            best = p;
                        }
                    }
                }
            }
            i = i.wrapping_add(1);
        }
        side += 2 as ::core::ffi::c_int;
    }
    if (*s).x < -D - 6000.0f64 {
        p = *L;
        p.loop_0 = true_0 != 0;
        p.leg = TEL_LEG_OUT as ::core::ffi::c_int;
        p.radius = fmax(TEL_R_MIN, r_fly);
        p.x_turn = -D;
        let mut sc_1: ::core::ffi::c_double = tel_score(&raw mut p, c, s);
        if sc_1 < best_score {
            best_score = sc_1;
            best = p;
        }
    }
    if best_score < cur_score - TEL_REPLAN_MARGIN {
        *L = best;
    }
    if (*L).loop_0 as ::core::ffi::c_int != 0
        && (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int
    {
        tel_size_hac(L, c, s, true_0 != 0);
    }
}
unsafe extern "C" fn tel_equilibrium_slope(
    mut c: *const TelContext,
    mut mass_kg: ::core::ffi::c_double,
    mut seed_deg: ::core::ffi::c_double,
    mut approach_speed: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut slope: ::core::ffi::c_double = if seed_deg.is_finite() as i32 != 0
        && seed_deg > 0.0f64
    {
        seed_deg * DEG2RAD
    } else {
        atan(0.5f64)
    };
    if !(mass_kg > 0.0f64) || !((*c).v_station > 0.0f64) || !((*c).d_station > 0.0f64) {
        return slope * RAD2DEG;
    }
    let mut v_a: ::core::ffi::c_double = if approach_speed.is_finite() as i32 != 0
        && approach_speed > 0.0f64
    {
        fmin(approach_speed, (*c).v_station)
    } else {
        (*c).v_station
    };
    let mut kinetic: ::core::ffi::c_double = ((*c).v_station * (*c).v_station
        - v_a * v_a) / (2.0f64 * TEL_G0 * (*c).d_station);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 8 as ::core::ffi::c_int {
        let mut p: TelPoint = TelPoint {
            x: 0.,
            y: 0.,
            h: 0.5f64 * (*c).d_station * tan(slope),
            v: sqrt(0.5f64 * ((*c).v_station * (*c).v_station + v_a * v_a)),
            g: 0.,
            psi: 0.,
            m: mass_kg,
            aoa: 0.,
            aoa_rate: 0.,
            bank: 0.,
            ground_speed: 0.,
        };
        let mut weight: ::core::ffi::c_double = mass_kg * tel_gravity(c, p.h);
        let mut aoa: ::core::ffi::c_double = tel_lift_aoa(
            c,
            &raw mut p,
            weight * cos(slope),
        );
        let mut q: ::core::ffi::c_double = 0.;
        let mut mach: ::core::ffi::c_double = 0.;
        let mut lift: ::core::ffi::c_double = 0.;
        let mut drag: ::core::ffi::c_double = 0.;
        tel_forces(
            c,
            &raw mut p,
            aoa,
            &raw mut q,
            &raw mut mach,
            &raw mut lift,
            &raw mut drag,
        );
        if !(lift > 0.0f64) || !(drag > 0.0f64) {
            break;
        }
        let mut next: ::core::ffi::c_double = atan(fmax(0.0f64, drag / lift - kinetic));
        if fabs(next - slope) < 1e-5f64 {
            slope = next;
            break;
        } else {
            slope = next;
            i += 1;
        }
    }
    return clampd(slope * RAD2DEG, 3.0f64, 45.0f64);
}
unsafe extern "C" fn tel_setup(
    mut model: *const TerminalModel,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut aoa_deg: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut out_c: *mut TelContext,
    mut out_s: *mut TelPoint,
) {
    let mut c: TelContext = TelContext {
        model: model,
        d_station: fmax(500.0f64, (*model).guidance.final_approach_distance),
        h_station: 0.,
        v_station: fmax(
            (*model).guidance.final_alignment_speed,
            (*model).vehicle.minimum_safe_speed,
        ),
        e_station: 0.,
        aoa_max: fmax(TEL_AOA_SUP_MIN, (*model).vehicle.maximum_angle_of_attack),
        slope_deg: 0.,
        final_line: false,
    };
    if (*model).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64 {
        c.aoa_max = fmin(
            c.aoa_max,
            (*model).vehicle.terminal_maximum_lift_angle_of_attack,
        );
    }
    c.slope_deg = tel_equilibrium_slope(
        &raw mut c,
        mass_kg,
        (*model).guidance.final_glide_slope,
        fmax((*model).vehicle.final_approach_speed, (*model).vehicle.minimum_safe_speed),
    );
    if (*model).guidance.final_glide_slope > 0.0f64 {
        c.slope_deg = fmin(c.slope_deg, (*model).guidance.final_glide_slope);
    }
    c.h_station = c.d_station * tan(c.slope_deg * DEG2RAD);
    c.e_station = c.h_station + c.v_station * c.v_station / (2.0f64 * TEL_G0);
    let mut s: TelPoint = TelPoint {
        x: (*geo).runway_along_m,
        y: -(*geo).runway_cross_m,
        h: (*geo).altitude_above_runway_m,
        v: (*geo).airspeed_mps,
        g: (*geo).flight_path_angle_deg * DEG2RAD,
        psi: -(*geo).heading_error_deg * DEG2RAD,
        m: mass_kg,
        aoa: if aoa_deg.is_finite() as i32 != 0 { aoa_deg } else { 10.0f64 },
        aoa_rate: 0.0f64,
        bank: if bank_deg.is_finite() as i32 != 0 { bank_deg } else { 0.0f64 },
        ground_speed: (*geo).ground_speed_mps,
    };
    *out_c = c;
    *out_s = s;
}
unsafe extern "C" fn tel_initialize(
    mut L: *mut TaemEnergyLawState,
    mut c: *const TelContext,
    mut s: *const TelPoint,
) {
    let mut D: ::core::ffi::c_double = (*c).d_station;
    *L = TaemEnergyLawState {
        initialized: true_0 != 0,
        loop_0: false,
        capture_forecast: tel_delivery_policy(c) as ::core::ffi::c_int != 0
            || fabs((*s).y) > 6000.0f64,
        leg: TEL_LEG_OUT as ::core::ffi::c_int,
        side: 1 as ::core::ffi::c_int,
        radius: 5000.0f64,
        r_turn: 0.,
        x_turn: -D,
        turn_cx: 0.,
        turn_cy: 0.,
        k: 0.5f64,
        s_req: 0.,
        s_go: 0.,
        last_geo_ut: -::core::f32::INFINITY as ::core::ffi::c_double,
        last_pred_ut: -::core::f32::INFINITY as ::core::ffi::c_double,
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
}
unsafe extern "C" fn taem_energy_law_frame_score(
    mut model: *const TerminalModel,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut aoa_deg: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut first_bank: *mut ::core::ffi::c_double,
    mut delivery_feasible: *mut bool,
) -> ::core::ffi::c_double {
    if model.is_null() || geo.is_null() || !(mass_kg > 0.0f64)
        || (*geo).runway_along_m.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut c: TelContext = TelContext {
        model: ::core::ptr::null::<TerminalModel>(),
        d_station: 0.,
        h_station: 0.,
        v_station: 0.,
        e_station: 0.,
        aoa_max: 0.,
        slope_deg: 0.,
        final_line: false,
    };
    let mut s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    tel_setup(model, geo, mass_kg, aoa_deg, bank_deg, &raw mut c, &raw mut s);
    let mut L: TaemEnergyLawState = TaemEnergyLawState {
        initialized: false,
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
    tel_initialize(&raw mut L, &raw mut c, &raw mut s);
    tel_plan(&raw mut L, &raw mut c, &raw mut s);
    let mut score: ::core::ffi::c_double = tel_score_with_delivery(
        &raw mut L,
        &raw mut c,
        &raw mut s,
        delivery_feasible,
    );
    if !first_bank.is_null() {
        *first_bank = tel_lateral(&raw mut L, &raw mut c, &raw mut s);
    }
    return if score.is_finite() as i32 != 0 {
        score
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn taem_energy_law_step(
    mut L: *mut TaemEnergyLawState,
    mut model: *const TerminalModel,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut aoa_deg: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut aoa_cmd: *mut ::core::ffi::c_double,
    mut bank_cmd: *mut ::core::ffi::c_double,
) -> bool {
    if L.is_null() || model.is_null() || geo.is_null() || !(mass_kg > 0.0f64)
        || (*geo).runway_along_m.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut c: TelContext = TelContext {
        model: ::core::ptr::null::<TerminalModel>(),
        d_station: 0.,
        h_station: 0.,
        v_station: 0.,
        e_station: 0.,
        aoa_max: 0.,
        slope_deg: 0.,
        final_line: false,
    };
    let mut s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    tel_setup(model, geo, mass_kg, aoa_deg, bank_deg, &raw mut c, &raw mut s);
    (*L).station_slope_deg = c.slope_deg;
    let mut D: ::core::ffi::c_double = c.d_station;
    let mut initial_plan: bool = !(*L).initialized;
    if initial_plan {
        tel_initialize(L, &raw mut c, &raw mut s);
    }
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw mut c, s.h, s.v, &raw mut q, &raw mut mach);
    let mut gnd: ::core::ffi::c_double = (*geo).ground_speed_mps;
    if tel_delivery_policy(&raw mut c) as ::core::ffi::c_int != 0
        && initial_plan as ::core::ffi::c_int != 0
        || ut >= (*L).last_geo_ut + TEL_UPDATE_S
            && tel_plan_open(L, &raw mut c, &raw mut s) as ::core::ffi::c_int != 0
    {
        (*L).last_geo_ut = ut;
        tel_plan(L, &raw mut c, &raw mut s);
    } else if (*L).loop_0 as ::core::ffi::c_int != 0
        && (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int
        && ut >= (*L).last_geo_ut + TEL_UPDATE_S
    {
        (*L).last_geo_ut = ut;
        let mut ext: ::core::ffi::c_double = tel_pattern_excess(
            L,
            &raw mut c,
            &raw mut s,
        );
        (*L).x_turn = fmax(
            -D + (*L).radius + 500.0f64,
            fmax(s.x, (*L).x_turn + 0.5f64 * ext),
        );
    }
    if (*L).loop_0 as ::core::ffi::c_int != 0
        && (*L).leg == TEL_LEG_OUT as ::core::ffi::c_int && s.x >= (*L).x_turn
    {
        (*L).leg = TEL_LEG_TURN as ::core::ffi::c_int;
        (*L).side = if s.y <= 0.0f64 {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        };
        (*L).r_turn = fmax(TEL_R_TURN, gnd * gnd / (TEL_G0 * tan(45.0f64 * DEG2RAD)));
    }
    if (!tel_delivery_policy(&raw mut c) || !(*L).capture_forecast)
        && (*L).leg == TEL_LEG_HAC as ::core::ffi::c_int && (*L).hac_left_rad > 0.5f64
        && ut >= (*L).last_geo_ut + TEL_UPDATE_S
    {
        (*L).last_geo_ut = ut;
        let mut arc1: [TelSegment; 1] = [
            TelSegment {
                length: (*L).hac_left_rad * (*L).radius,
                radius: (*L).radius,
            },
        ];
        let mut need: ::core::ffi::c_double = tel_predict_range(
            &raw mut c,
            &raw mut s,
            TEL_K_PLAN,
            1e9f64,
            &raw mut arc1 as *mut TelSegment,
            1 as ::core::ffi::c_int,
        );
        let mut r_fly: ::core::ffi::c_double = gnd * gnd
            / (TEL_G0 * tan(TEL_BANK_TURN * DEG2RAD));
        let mut r_floor: ::core::ffi::c_double = if (*L).hac_left_rad > 1.5f64 * TEL_PI {
            TEL_R_SPIRAL_MIN
        } else {
            fmin(TEL_R_MIN, (*L).radius)
        };
        let mut r_new: ::core::ffi::c_double = clampd(
            need / (*L).hac_left_rad,
            fmax(r_floor, r_fly),
            TEL_R_MAX,
        );
        (*L).radius = clampd(r_new, (*L).radius - 250.0f64, (*L).radius + 250.0f64);
    }
    let mut ce: TelContext = tel_effective_station(L, &raw mut c, &raw mut s, mach, gnd);
    let mut segs: [TelSegment; 5] = [TelSegment {
        length: 0.,
        radius: 0.,
    }; 5];
    let mut nseg: ::core::ffi::c_int = tel_segments(
        L,
        &raw mut ce,
        &raw mut s,
        &raw mut segs as *mut TelSegment,
    );
    (*L).s_go = tel_sum(&raw mut segs as *mut TelSegment, nseg);
    if !getenv(b"KSP_LANDER_TEL_TRACE\0" as *const u8 as *const ::core::ffi::c_char)
        .is_null()
        && ((*L).leg != (*L).trace_leg || (*L).side != (*L).trace_side
            || fabs((*L).radius - (*L).trace_radius) > 1.0f64
            || (*L).loop_0 as ::core::ffi::c_int
                != (*L).trace_loop as ::core::ffi::c_int)
    {
        (*L).trace_leg = (*L).leg;
        (*L).trace_side = (*L).side;
        (*L).trace_radius = (*L).radius;
        (*L).trace_loop = (*L).loop_0;
        fprintf(
            stderr,
            b"TEL_PLAN ut=%.1f loop=%d leg=%d side=%+d R=%.0f D=%.0f x=%.0f y=%.0f psi=%.1f xturn=%.0f segs:\0"
                as *const u8 as *const ::core::ffi::c_char,
            ut,
            (*L).loop_0 as ::core::ffi::c_int,
            (*L).leg,
            (*L).side,
            (*L).radius,
            D,
            s.x,
            s.y,
            s.psi * RAD2DEG,
            (*L).x_turn,
        );
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < nseg {
            fprintf(
                stderr,
                b" [%.0f r%.0f]\0" as *const u8 as *const ::core::ffi::c_char,
                segs[i as usize].length,
                segs[i as usize].radius,
            );
            i += 1;
        }
        fprintf(stderr, b"\n\0" as *const u8 as *const ::core::ffi::c_char);
    }
    if ut >= (*L).last_pred_ut + TEL_UPDATE_S {
        (*L).last_pred_ut = ut;
        (*L).k = tel_solve_k(L, &raw mut ce, &raw mut s, &raw mut (*L).s_req);
        if (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int && s.x > -c.d_station {
            let mut d_ref: ::core::ffi::c_double = fmax(500.0f64, -s.x);
            let mut e_ref: ::core::ffi::c_double = d_ref * tan(c.slope_deg * DEG2RAD)
                + c.v_station * c.v_station / (2.0f64 * TEL_G0);
            let mut e_now: ::core::ffi::c_double = s.h + s.v * s.v / (2.0f64 * TEL_G0);
            (*L).k = clampd(
                TEL_K_PLAN + (e_now - e_ref) / TEL_FINAL_ENERGY_SPAN_M,
                0.0f64,
                1.0f64,
            );
            (*L).s_req = (*L).s_go;
        }
    }
    *bank_cmd = tel_lateral(L, &raw mut c, &raw mut s);
    let mut along_v: ::core::ffi::c_double = if (*L).leg
        == TEL_LEG_FINAL as ::core::ffi::c_int
    {
        -D - s.x
    } else {
        (*L).s_go
    };
    c.final_line = (*L).leg == TEL_LEG_FINAL as ::core::ffi::c_int;
    *aoa_cmd = tel_vertical(
        &raw mut c,
        &raw mut s,
        *bank_cmd,
        (*L).k,
        along_v,
        true_0 != 0,
    );
    return (*aoa_cmd).is_finite() as i32 != 0 && (*bank_cmd).is_finite() as i32 != 0;
}
pub const HAC_R2: ::core::ffi::c_double = 93.0f64;
pub const HAC_RF_MIN: ::core::ffi::c_double = 2200.0f64;
pub const HAC_RF_MAX: ::core::ffi::c_double = 9000.0f64;
pub const HAC_BANK_NOMINAL: ::core::ffi::c_double = 30.0f64;
pub const HAC_RF_RATE: ::core::ffi::c_double = 15.0f64;
pub const HAC_LD_FRACTION: ::core::ffi::c_double = 0.9f64;
pub const HAC_STATION_MARGIN_M: ::core::ffi::c_double = 300.0f64;
pub const HAC_BANK_SIZING: ::core::ffi::c_double = 55.0f64;
pub const HAC_OGS_FULL_M: ::core::ffi::c_double = 1500.0f64;
pub const HAC_OGS_BLEND_M: ::core::ffi::c_double = 5000.0f64;
pub const HAC_Q_BLEND_M: ::core::ffi::c_double = 12000.0f64;
pub const HAC_REF_DS: ::core::ffi::c_double = 400.0f64;
pub const HAC_REF_REBUILD_S: ::core::ffi::c_double = 2.0f64;
pub const HAC_KAPPA_REACH: ::core::ffi::c_double = 1.5f64;
pub const HAC_KAPPA_TAU_S: ::core::ffi::c_double = 6.0f64;
pub const HAC_GPHI: ::core::ffi::c_double = 2.5f64;
pub const HAC_BANK_OUTWARD: ::core::ffi::c_double = 30.0f64;
pub const HAC_RERRLM: ::core::ffi::c_double = 2100.0f64;
pub const HAC_STURN_MIN_RANGE: ::core::ffi::c_double = 15000.0f64;
pub const HAC_STURN_MAX_PSI: ::core::ffi::c_double = 200.0f64 * DEG2RAD;
pub const HAC_SELECT_FREEZE_M: ::core::ffi::c_double = 10000.0f64;
pub const HAC_SELECT_MARGIN_M: ::core::ffi::c_double = 1500.0f64;
pub const HAC_UPDATE_S: ::core::ffi::c_double = 2.0f64;
pub const HAC_GATE_MACH: ::core::ffi::c_double = 0.95f64;
pub const HAC_DRAG_EXIT_MACH: ::core::ffi::c_double = 1.0f64;
pub const HAC_DRAG_MIN_RUN: ::core::ffi::c_double = 3000.0f64;
pub const HAC_DRAG_MAX_LOAD: ::core::ffi::c_double = 2.5f64;
pub const HAC_DRAG_BANK_MAX: ::core::ffi::c_double = 70.0f64;
pub const HAC_DRAG_GAMMA_TAU: ::core::ffi::c_double = 6.0f64;
pub const HAC_DRAG_DEADBAND_DEG: ::core::ffi::c_double = 15.0f64;
pub const HAC_DRAG_STURN_DEG: ::core::ffi::c_double = 45.0f64;
pub const HAC_DRAG_LOOKAHEAD: ::core::ffi::c_double = 15000.0f64;
pub const HAC_DRAG_AOA_SLEW: ::core::ffi::c_double = 2.0f64;
unsafe extern "C" fn hac_bin(mut mach: ::core::ffi::c_double) -> ::core::ffi::c_int {
    let mut i: ::core::ffi::c_int = floor(fmax(0.0f64, mach) / 0.25f64)
        as ::core::ffi::c_int;
    return if i < 0 as ::core::ffi::c_int {
        0 as ::core::ffi::c_int
    } else if i >= TAEM_HAC_KAPPA_BINS {
        TAEM_HAC_KAPPA_BINS - 1 as ::core::ffi::c_int
    } else {
        i
    };
}
unsafe extern "C" fn hac_kappa(
    mut A: *const TaemHacLiveAero,
    mut mach: ::core::ffi::c_double,
    mut lift: bool,
) -> ::core::ffi::c_double {
    if A.is_null() {
        return 1.0f64;
    }
    let mut b: ::core::ffi::c_int = hac_bin(mach);
    let mut r: *const ::core::ffi::c_double = if lift as ::core::ffi::c_int != 0 {
        &raw const (*A).lift as *const ::core::ffi::c_double
    } else {
        &raw const (*A).drag as *const ::core::ffi::c_double
    };
    if (*A).weight[b as usize] > 0.0f64 && *r.offset(b as isize) > 0.0f64 {
        return *r.offset(b as isize);
    }
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut value: ::core::ffi::c_double = 1.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < TAEM_HAC_KAPPA_BINS {
        if (*A).weight[i as usize] > 0.0f64 && *r.offset(i as isize) > 0.0f64 {
            if !(((i as ::core::ffi::c_double + 0.5f64) * 0.25f64 < 1.0f64)
                as ::core::ffi::c_int != (mach < 1.0f64) as ::core::ffi::c_int)
            {
                let mut dm: ::core::ffi::c_double = fabs(
                    mach - (i as ::core::ffi::c_double + 0.5f64) * 0.25f64,
                );
                if dm < best {
                    best = dm;
                    value = *r.offset(i as isize);
                }
            }
        }
        i += 1;
    }
    if best.is_finite() as i32 == 0 {
        return 1.0f64;
    }
    return 1.0f64 + (value - 1.0f64) * exp(-best / HAC_KAPPA_REACH);
}
unsafe extern "C" fn hac_live_aero_observe(
    mut A: *mut TaemHacLiveAero,
    mut aero: *const AeroTable,
    mut t: *const Telemetry,
    mut speedbrake: ::core::ffi::c_double,
) {
    if A.is_null() || aero.is_null() || t.is_null() || !(*t).physics_sample_valid
        || !((*t).mass > 0.0f64)
    {
        return;
    }
    let mut dt: ::core::ffi::c_double = if (*A).last_ut.is_finite() as i32 != 0
        && (*t).ut > (*A).last_ut
    {
        fmin((*t).ut - (*A).last_ut, 1.0f64)
    } else {
        0.0f64
    };
    (*A).last_ut = (*t).ut;
    if !(dt > 0.0f64) || !((*t).dynamic_pressure > 300.0f64)
        || (*t).mach.is_finite() as i32 == 0
        || (*t).angle_of_attack.is_finite() as i32 == 0 || (*t).angle_of_attack < 1.0f64
        || speedbrake > 0.02f64
    {
        return;
    }
    let mut cl: ::core::ffi::c_double = 0.;
    let mut cd: ::core::ffi::c_double = 0.;
    aero_coefficients(aero, (*t).mach, (*t).angle_of_attack, &raw mut cl, &raw mut cd);
    let mut qs: ::core::ffi::c_double = (*t).dynamic_pressure
        * (*aero).reference_area_m2;
    if !(cl > 0.02f64) || !(cd > 1e-3f64) || !(qs > 0.0f64) {
        return;
    }
    let mut rl: ::core::ffi::c_double = fabs((*t).lift_force) / (qs * cl);
    let mut rd: ::core::ffi::c_double = fabs((*t).drag_force) / (qs * cd);
    if rl.is_finite() as i32 == 0 || rd.is_finite() as i32 == 0 || !(rl > 0.0f64)
        || !(rd > 0.0f64)
    {
        return;
    }
    let mut b: ::core::ffi::c_int = hac_bin((*t).mach);
    let mut a: ::core::ffi::c_double = dt / (HAC_KAPPA_TAU_S + dt);
    if !((*A).weight[b as usize] > 0.0f64) {
        (*A).lift[b as usize] = rl;
        (*A).drag[b as usize] = rd;
    } else {
        (*A).lift[b as usize] += a * (rl - (*A).lift[b as usize]);
        (*A).drag[b as usize] += a * (rd - (*A).drag[b as usize]);
    }
    (*A).weight[b as usize] = fmin(
        1.0f64,
        (*A).weight[b as usize] + dt / HAC_KAPPA_TAU_S,
    );
    if !getenv(b"KSP_LANDER_HAC_AERO_TRACE\0" as *const u8 as *const ::core::ffi::c_char)
        .is_null()
    {
        fprintf(
            stderr,
            b"HAC_AERO ut=%.1f M=%.2f aoa=%.1f bin=%d rawL=%.3f rawD=%.3f L=%.3f D=%.3f w=%.2f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).ut,
            (*t).mach,
            (*t).angle_of_attack,
            b,
            rl,
            rd,
            (*A).lift[b as usize],
            (*A).drag[b as usize],
            (*A).weight[b as usize],
        );
    }
}
unsafe extern "C" fn hac_aero(
    mut x: *const HacCtx,
    mut mach: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut cl: *mut ::core::ffi::c_double,
    mut cd: *mut ::core::ffi::c_double,
) {
    aero_coefficients((*x).aero, mach, aoa, cl, cd);
    *cl *= hac_kappa((*x).live, mach, true_0 != 0);
    *cd *= hac_kappa((*x).live, mach, false_0 != 0);
}
unsafe extern "C" fn hac_lift_aoa_to(
    mut x: *const HacCtx,
    mut mach: ::core::ffi::c_double,
    mut q: ::core::ffi::c_double,
    mut lift_needed: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut area: ::core::ffi::c_double = (*(*x).aero).reference_area_m2;
    let mut lo: ::core::ffi::c_double = -5.0f64;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 30 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        let mut cl: ::core::ffi::c_double = 0.;
        let mut cd: ::core::ffi::c_double = 0.;
        hac_aero(x, mach, mid, &raw mut cl, &raw mut cd);
        if cl * q * area < lift_needed {
            lo = mid;
        } else {
            hi = mid;
        }
        i += 1;
    }
    return 0.5f64 * (lo + hi);
}
unsafe extern "C" fn hac_best_ld_aoa(
    mut x: *const HacCtx,
    mut mach: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut best_aoa: ::core::ffi::c_double = hi;
    let mut best: ::core::ffi::c_double = -::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut a: ::core::ffi::c_double = TEL_AOA_MIN;
    while a <= hi + 1e-9f64 {
        let mut cl: ::core::ffi::c_double = 0.;
        let mut cd: ::core::ffi::c_double = 0.;
        hac_aero(x, mach, a, &raw mut cl, &raw mut cd);
        if cd > 1e-4f64 && cl / cd > best {
            best = cl / cd;
            best_aoa = a;
        }
        a += 0.25f64;
    }
    return best_aoa;
}
unsafe extern "C" fn hac_lift_aoa(
    mut x: *const HacCtx,
    mut mach: ::core::ffi::c_double,
    mut q: ::core::ffi::c_double,
    mut lift_needed: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return hac_lift_aoa_to(x, mach, q, lift_needed, (*x).c.aoa_max);
}
unsafe extern "C" fn hac_drag_aoa_ceiling(
    mut x: *const HacCtx,
    mut mach: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut sup: ::core::ffi::c_double = fmax((*x).c.aoa_max, (*x).aoa_supersonic);
    return (*x).c.aoa_max
        + (sup - (*x).c.aoa_max) * clampd((mach - 1.0f64) / 0.5f64, 0.0f64, 1.0f64);
}
unsafe extern "C" fn hac_reference_incidence(
    mut R: *mut TaemHacReference,
    mut x: *const HacCtx,
) {
    let mut m: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while m < TAEM_HAC_REF_MACH_N {
        let mut mach: ::core::ffi::c_double = 0.1f64 * m as ::core::ffi::c_double;
        let mut best: ::core::ffi::c_double = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut best_aoa: ::core::ffi::c_double = (*x).c.aoa_max;
        let mut ld: [::core::ffi::c_double; 64] = [0.; 64];
        let mut n: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        let mut a: ::core::ffi::c_double = 2.0f64;
        while a <= (*x).c.aoa_max + 1e-9f64 && n < 64 as ::core::ffi::c_int {
            let mut sum: ::core::ffi::c_double = 0.0f64;
            let mut k: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
            while k <= 1 as ::core::ffi::c_int {
                let mut cl: ::core::ffi::c_double = 0.;
                let mut cd: ::core::ffi::c_double = 0.;
                hac_aero(
                    x,
                    mach,
                    a + 0.5f64 * k as ::core::ffi::c_double,
                    &raw mut cl,
                    &raw mut cd,
                );
                sum += if cd > 1e-4f64 { cl / cd } else { 0.0f64 };
                k += 1;
            }
            ld[n as usize] = sum / 3.0f64;
            if ld[n as usize] > best {
                best = ld[n as usize];
                best_aoa = a;
            }
            a += 0.5f64;
            n += 1;
        }
        let mut aoa: ::core::ffi::c_double = best_aoa;
        let mut i: ::core::ffi::c_int = round((best_aoa - 2.0f64) / 0.5f64)
            as ::core::ffi::c_int;
        while i > 0 as ::core::ffi::c_int
            && ld[(i - 1 as ::core::ffi::c_int) as usize] >= HAC_LD_FRACTION * best
        {
            i -= 1;
            aoa -= 0.5f64;
        }
        let mut cl_0: ::core::ffi::c_double = 0.;
        let mut cd_0: ::core::ffi::c_double = 0.;
        hac_aero(x, mach, aoa, &raw mut cl_0, &raw mut cd_0);
        (*R).aoa[m as usize] = aoa;
        (*R).cl[m as usize] = fmax(0.02f64, cl_0);
        m += 1;
    }
}
unsafe extern "C" fn hac_q_ref(
    mut x: *const HacCtx,
    mut R: *const TaemHacReference,
    mut mach: ::core::ffi::c_double,
    mut s: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut u: ::core::ffi::c_double = clampd(
        mach,
        0.0f64,
        0.1f64 * (TAEM_HAC_REF_MACH_N - 1 as ::core::ffi::c_int) as ::core::ffi::c_double,
    ) * 10.0f64;
    let mut i: ::core::ffi::c_int = floor(u) as ::core::ffi::c_int;
    if i > TAEM_HAC_REF_MACH_N - 2 as ::core::ffi::c_int {
        i = TAEM_HAC_REF_MACH_N - 2 as ::core::ffi::c_int;
    }
    let mut cl: ::core::ffi::c_double = (*R).cl[i as usize]
        + (u - i as ::core::ffi::c_double)
            * ((*R).cl[(i + 1 as ::core::ffi::c_int) as usize] - (*R).cl[i as usize]);
    let mut q: ::core::ffi::c_double = (*x).mass * TEL_G0
        / (cl * (*(*x).aero).reference_area_m2);
    if mach < 1.0f64 {
        let mut q_sta: ::core::ffi::c_double = 0.;
        let mut m_sta: ::core::ffi::c_double = 0.;
        tel_air(
            &raw const (*x).c,
            (*x).c.h_station,
            (*x).c.v_station,
            &raw mut q_sta,
            &raw mut m_sta,
        );
        q = q_sta + (q - q_sta) * clampd(s / HAC_Q_BLEND_M, 0.0f64, 1.0f64);
    }
    return q;
}
unsafe extern "C" fn hac_split_ref(
    mut x: *const HacCtx,
    mut R: *const TaemHacReference,
    mut e: ::core::ffi::c_double,
    mut s: ::core::ffi::c_double,
    mut h: *mut ::core::ffi::c_double,
    mut v: *mut ::core::ffi::c_double,
) {
    let mut floor_v: ::core::ffi::c_double = 1.3f64
        * (*(*x).c.model).vehicle.minimum_safe_speed;
    let mut lo: ::core::ffi::c_double = 0.0f64;
    let mut hi: ::core::ffi::c_double = fmax(1.0f64, e);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 40 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        let mut vv: ::core::ffi::c_double = sqrt(
            fmax(1.0f64, 2.0f64 * TEL_G0 * (e - mid)),
        );
        let mut q: ::core::ffi::c_double = 0.;
        let mut mach: ::core::ffi::c_double = 0.;
        tel_air(&raw const (*x).c, mid, vv, &raw mut q, &raw mut mach);
        if q > hac_q_ref(x, R, mach, s) && vv > floor_v {
            lo = mid;
        } else {
            hi = mid;
        }
        i += 1;
    }
    *h = 0.5f64 * (lo + hi);
    let mut w: ::core::ffi::c_double = clampd(
        (HAC_OGS_BLEND_M - s) / (HAC_OGS_BLEND_M - HAC_OGS_FULL_M),
        0.0f64,
        1.0f64,
    );
    if w > 0.0f64 {
        let mut h_line: ::core::ffi::c_double = (*x).c.h_station
            + s * tan((*x).c.slope_deg * DEG2RAD);
        *h = fmin(
            w * h_line + (1.0f64 - w) * *h,
            e - floor_v * floor_v / (2.0f64 * TEL_G0),
        );
    }
    *v = sqrt(fmax(1.0f64, 2.0f64 * TEL_G0 * (e - *h)));
}
unsafe extern "C" fn hac_eta(
    mut x: *const HacCtx,
    mut R: *const TaemHacReference,
    mut e: ::core::ffi::c_double,
    mut s: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut h: ::core::ffi::c_double = 0.;
    let mut v: ::core::ffi::c_double = 0.;
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    hac_split_ref(x, R, e, s, &raw mut h, &raw mut v);
    tel_air(&raw const (*x).c, h, v, &raw mut q, &raw mut mach);
    let mut cphi: ::core::ffi::c_double = cos(fmin(fabs(bank_deg), 70.0f64) * DEG2RAD);
    let mut aoa: ::core::ffi::c_double = hac_lift_aoa(
        x,
        mach,
        q,
        (*x).mass * tel_gravity(&raw const (*x).c, h) / cphi,
    );
    let mut cl: ::core::ffi::c_double = 0.;
    let mut cd: ::core::ffi::c_double = 0.;
    hac_aero(x, mach, aoa, &raw mut cl, &raw mut cd);
    return if cl > 1e-3f64 { cd / cl / cphi } else { 3.0f64 };
}
unsafe extern "C" fn hac_reference_build(
    mut R: *mut TaemHacReference,
    mut x: *const HacCtx,
    mut ut: ::core::ffi::c_double,
) {
    (*R).ds = HAC_REF_DS;
    (*R).mass = (*x).mass;
    (*R).built_ut = ut;
    hac_reference_incidence(R, x);
    (*R).e[0 as ::core::ffi::c_int as usize] = (*x).c.e_station + HAC_STATION_MARGIN_M;
    (*R).h[0 as ::core::ffi::c_int as usize] = (*x).c.h_station;
    (*R).v[0 as ::core::ffi::c_int as usize] = sqrt(
        2.0f64 * TEL_G0
            * ((*R).e[0 as ::core::ffi::c_int as usize]
                - (*R).h[0 as ::core::ffi::c_int as usize]),
    );
    let mut i: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
    while i < TAEM_HAC_REF_N {
        let mut s: ::core::ffi::c_double = (i - 1 as ::core::ffi::c_int)
            as ::core::ffi::c_double * (*R).ds;
        let mut e: ::core::ffi::c_double = (*R)
            .e[(i - 1 as ::core::ffi::c_int) as usize];
        let mut k1: ::core::ffi::c_double = hac_eta(x, R, e, s, 0.0f64);
        let mut k2: ::core::ffi::c_double = hac_eta(
            x,
            R,
            e + 0.5f64 * (*R).ds * k1,
            s + 0.5f64 * (*R).ds,
            0.0f64,
        );
        (*R).e[i as usize] = e + (*R).ds * k2;
        hac_split_ref(
            x,
            R,
            (*R).e[i as usize],
            s + (*R).ds,
            (&raw mut (*R).h as *mut ::core::ffi::c_double).offset(i as isize)
                as *mut ::core::ffi::c_double,
            (&raw mut (*R).v as *mut ::core::ffi::c_double).offset(i as isize)
                as *mut ::core::ffi::c_double,
        );
        i += 1;
    }
    memcpy(
        &raw mut (*R).h_eq as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
        &raw mut (*R).h as *mut ::core::ffi::c_double as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 480]>() as size_t,
    );
    memcpy(
        &raw mut (*R).v_eq as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
        &raw mut (*R).v as *mut ::core::ffi::c_double as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 480]>() as size_t,
    );
    (*R).s_gate = 0.0f64;
    (*R).e_gate = (*R).e[0 as ::core::ffi::c_int as usize];
    (*R).h_gate = (*R).h[0 as ::core::ffi::c_int as usize];
    let mut i_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
    while i_0 < TAEM_HAC_REF_N {
        let mut q: ::core::ffi::c_double = 0.;
        let mut m0: ::core::ffi::c_double = 0.;
        let mut m1: ::core::ffi::c_double = 0.;
        tel_air(
            &raw const (*x).c,
            (*R).h[(i_0 - 1 as ::core::ffi::c_int) as usize],
            (*R).v[(i_0 - 1 as ::core::ffi::c_int) as usize],
            &raw mut q,
            &raw mut m0,
        );
        tel_air(
            &raw const (*x).c,
            (*R).h[i_0 as usize],
            (*R).v[i_0 as usize],
            &raw mut q,
            &raw mut m1,
        );
        if m1 >= HAC_GATE_MACH && m0 < HAC_GATE_MACH {
            let mut f: ::core::ffi::c_double = (HAC_GATE_MACH - m0)
                / fmax(1e-9f64, m1 - m0);
            (*R).s_gate = ((i_0 - 1 as ::core::ffi::c_int) as ::core::ffi::c_double + f)
                * (*R).ds;
            (*R).e_gate = (*R).e[(i_0 - 1 as ::core::ffi::c_int) as usize]
                + f
                    * ((*R).e[i_0 as usize]
                        - (*R).e[(i_0 - 1 as ::core::ffi::c_int) as usize]);
            (*R).h_gate = (*R).h[(i_0 - 1 as ::core::ffi::c_int) as usize]
                + f
                    * ((*R).h[i_0 as usize]
                        - (*R).h[(i_0 - 1 as ::core::ffi::c_int) as usize]);
            let mut slope: ::core::ffi::c_double = ::core::f32::INFINITY
                as ::core::ffi::c_double;
            let mut v_gate: ::core::ffi::c_double = (*R)
                .v[(i_0 - 1 as ::core::ffi::c_int) as usize]
                + f
                    * ((*R).v[i_0 as usize]
                        - (*R).v[(i_0 - 1 as ::core::ffi::c_int) as usize]);
            let mut j: ::core::ffi::c_int = if i_0 / 2 as ::core::ffi::c_int
                > 1 as ::core::ffi::c_int
            {
                i_0 / 2 as ::core::ffi::c_int
            } else {
                1 as ::core::ffi::c_int
            };
            while j < i_0 {
                slope = fmin(
                    slope,
                    ((*R).e[j as usize] - (*R).e[(j - 1 as ::core::ffi::c_int) as usize])
                        / (*R).ds,
                );
                j += 1;
            }
            if slope.is_finite() as i32 == 0 {
                slope = ((*R).e[i_0 as usize]
                    - (*R).e[(i_0 - 1 as ::core::ffi::c_int) as usize]) / (*R).ds;
            }
            let mut k: ::core::ffi::c_int = i_0;
            while k < TAEM_HAC_REF_N {
                (*R).e[k as usize] = (*R).e_gate
                    + slope * (k as ::core::ffi::c_double * (*R).ds - (*R).s_gate);
                (*R).h[k as usize] = (*R).h_gate + ((*R).e[k as usize] - (*R).e_gate);
                (*R).v[k as usize] = v_gate;
                k += 1;
            }
            break;
        } else {
            i_0 += 1;
        }
    }
    (*R).valid = true_0 != 0;
    if !getenv(b"KSP_LANDER_HAC_REF_TRACE\0" as *const u8 as *const ::core::ffi::c_char)
        .is_null()
    {
        fn dumped_tls() -> *mut ::core::ffi::c_int {
            ::std::thread_local! {
                static CELL: ::core::cell::UnsafeCell<::core::ffi::c_int> = const { ::core::cell::UnsafeCell::new(0 as ::core::ffi::c_int) };
            }
            CELL.with(|c| c.get())
        }
        let fresh0 = (*dumped_tls());
        (*dumped_tls()) = (*dumped_tls()) + 1;
        if fresh0 == 0 {
            let mut i_1: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while i_1 < TAEM_HAC_REF_N {
                let mut q_0: ::core::ffi::c_double = 0.;
                let mut mach: ::core::ffi::c_double = 0.;
                tel_air(
                    &raw const (*x).c,
                    (*R).h[i_1 as usize],
                    (*R).v[i_1 as usize],
                    &raw mut q_0,
                    &raw mut mach,
                );
                fprintf(
                    stderr,
                    b"HAC_REF s=%.0f e=%.0f h=%.0f v=%.1f M=%.2f q=%.0f\n\0" as *const u8
                        as *const ::core::ffi::c_char,
                    i_1 as ::core::ffi::c_double * (*R).ds,
                    (*R).e[i_1 as usize],
                    (*R).h[i_1 as usize],
                    (*R).v[i_1 as usize],
                    mach,
                    q_0,
                );
                i_1
                    += if i_1 < 40 as ::core::ffi::c_int {
                        2 as ::core::ffi::c_int
                    } else {
                        20 as ::core::ffi::c_int
                    };
            }
        }
    }
}
unsafe extern "C" fn hac_reference_at(
    mut R: *const TaemHacReference,
    mut s: ::core::ffi::c_double,
    mut e: *mut ::core::ffi::c_double,
    mut h: *mut ::core::ffi::c_double,
    mut v: *mut ::core::ffi::c_double,
    mut dhds: *mut ::core::ffi::c_double,
    mut deds: *mut ::core::ffi::c_double,
) {
    let mut u: ::core::ffi::c_double = s / (*R).ds;
    let mut i: ::core::ffi::c_int = floor(u) as ::core::ffi::c_int;
    if i < 0 as ::core::ffi::c_int {
        i = 0 as ::core::ffi::c_int;
    }
    if i > TAEM_HAC_REF_N - 2 as ::core::ffi::c_int {
        i = TAEM_HAC_REF_N - 2 as ::core::ffi::c_int;
    }
    let mut f: ::core::ffi::c_double = u - i as ::core::ffi::c_double;
    let mut de: ::core::ffi::c_double = ((*R).e[(i + 1 as ::core::ffi::c_int) as usize]
        - (*R).e[i as usize]) / (*R).ds;
    let mut dh: ::core::ffi::c_double = ((*R).h[(i + 1 as ::core::ffi::c_int) as usize]
        - (*R).h[i as usize]) / (*R).ds;
    if !e.is_null() {
        *e = (*R).e[i as usize]
            + f * ((*R).e[(i + 1 as ::core::ffi::c_int) as usize] - (*R).e[i as usize]);
    }
    if !h.is_null() {
        *h = (*R).h[i as usize]
            + f * ((*R).h[(i + 1 as ::core::ffi::c_int) as usize] - (*R).h[i as usize]);
    }
    if !v.is_null() {
        *v = (*R).v[i as usize]
            + f * ((*R).v[(i + 1 as ::core::ffi::c_int) as usize] - (*R).v[i as usize]);
    }
    if !dhds.is_null() {
        *dhds = dh;
    }
    if !deds.is_null() {
        *deds = de;
    }
}
unsafe extern "C" fn hac_radius_at(
    mut rf: ::core::ffi::c_double,
    mut psi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return rf + HAC_R2 * psi * psi;
}
unsafe extern "C" fn hac_arc_length(
    mut rf: ::core::ffi::c_double,
    mut psi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return rf * psi + HAC_R2 * psi * psi * psi / 3.0f64;
}
unsafe extern "C" fn hac_exit_angle(
    mut side: ::core::ffi::c_int,
) -> ::core::ffi::c_double {
    return -side as ::core::ffi::c_double * TEL_PI / 2.0f64;
}
unsafe extern "C" fn hac_point_psi(
    mut side: ::core::ffi::c_int,
    mut rf: ::core::ffi::c_double,
    mut D: ::core::ffi::c_double,
    mut px: ::core::ffi::c_double,
    mut py: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut ang: ::core::ffi::c_double = atan2(
        py - side as ::core::ffi::c_double * rf,
        px + D,
    );
    let mut psi: ::core::ffi::c_double = fmod(
        side as ::core::ffi::c_double * (hac_exit_angle(side) - ang),
        2.0f64 * TEL_PI,
    );
    return if psi < 0.0f64 { psi + 2.0f64 * TEL_PI } else { psi };
}
unsafe extern "C" fn hac_spiral_tangent(
    mut side: ::core::ffi::c_int,
    mut rf: ::core::ffi::c_double,
    mut D: ::core::ffi::c_double,
    mut px: ::core::ffi::c_double,
    mut py: ::core::ffi::c_double,
    mut tx: *mut ::core::ffi::c_double,
    mut ty: *mut ::core::ffi::c_double,
    mut psi_t: *mut ::core::ffi::c_double,
) -> bool {
    let mut cx: ::core::ffi::c_double = -D;
    let mut cy: ::core::ffi::c_double = side as ::core::ffi::c_double * rf;
    let mut dx: ::core::ffi::c_double = px - cx;
    let mut dy: ::core::ffi::c_double = py - cy;
    let mut d: ::core::ffi::c_double = hypot(dx, dy);
    let mut psi: ::core::ffi::c_double = hac_point_psi(side, rf, D, px, py);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 8 as ::core::ffi::c_int {
        let mut r: ::core::ffi::c_double = hac_radius_at(rf, psi);
        if d <= r * 1.001f64 {
            return false_0 != 0;
        }
        let mut ang: ::core::ffi::c_double = atan2(dy, dx)
            + side as ::core::ffi::c_double * acos(r / d);
        let mut next: ::core::ffi::c_double = fmod(
            side as ::core::ffi::c_double * (hac_exit_angle(side) - ang),
            2.0f64 * TEL_PI,
        );
        if next < 0.0f64 {
            next += 2.0f64 * TEL_PI;
        }
        psi = next;
        *tx = cx + r * cos(ang);
        *ty = cy + r * sin(ang);
        i += 1;
    }
    *psi_t = psi;
    return true_0 != 0;
}
unsafe extern "C" fn hac_turn_cost(
    mut x: *const HacCtx,
    mut R: *const TaemHacReference,
    mut rf: ::core::ffi::c_double,
    mut psi_t: ::core::ffi::c_double,
    mut acq_arc: ::core::ffi::c_double,
    mut acq_bank_deg: ::core::ffi::c_double,
    mut s_acq: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut cost: ::core::ffi::c_double = 0.0f64;
    let n: ::core::ffi::c_int = 12 as ::core::ffi::c_int;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < n {
        let mut p0: ::core::ffi::c_double = psi_t * i as ::core::ffi::c_double
            / n as ::core::ffi::c_double;
        let mut p1: ::core::ffi::c_double = psi_t
            * (i + 1 as ::core::ffi::c_int) as ::core::ffi::c_double
            / n as ::core::ffi::c_double;
        let mut pm: ::core::ffi::c_double = 0.5f64 * (p0 + p1);
        let mut sm: ::core::ffi::c_double = hac_arc_length(rf, pm);
        let mut e: ::core::ffi::c_double = 0.;
        let mut v: ::core::ffi::c_double = 0.;
        hac_reference_at(
            R,
            sm,
            &raw mut e,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            &raw mut v,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
        let mut bank: ::core::ffi::c_double = fmin(
            60.0f64,
            atan(v * v / (TEL_G0 * hac_radius_at(rf, pm))) * RAD2DEG,
        );
        cost
            += fmax(0.0f64, hac_eta(x, R, e, sm, bank) - hac_eta(x, R, e, sm, 0.0f64))
                * (hac_arc_length(rf, p1) - hac_arc_length(rf, p0));
        i += 1;
    }
    if acq_arc > 0.0f64 {
        let mut e_0: ::core::ffi::c_double = 0.;
        hac_reference_at(
            R,
            s_acq,
            &raw mut e_0,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
        cost
            += fmax(
                0.0f64,
                hac_eta(x, R, e_0, s_acq, acq_bank_deg)
                    - hac_eta(x, R, e_0, s_acq, 0.0f64),
            ) * acq_arc;
    }
    return cost;
}
unsafe extern "C" fn hac_bank_limit(
    mut mach: ::core::ffi::c_double,
    mut phase: ::core::ffi::c_int,
) -> ::core::ffi::c_double {
    let mut sup: bool = mach > 1.0f64;
    if phase == TAEM_HAC_PHASE_HAC as ::core::ffi::c_int {
        return if sup as ::core::ffi::c_int != 0 { 55.0f64 } else { 60.0f64 };
    }
    return if sup as ::core::ffi::c_int != 0 { 45.0f64 } else { 50.0f64 };
}
unsafe extern "C" fn hac_predict_acquisition(
    mut x: *const HacCtx,
    mut R: *const TaemHacReference,
    mut side: ::core::ffi::c_int,
    mut rf: ::core::ffi::c_double,
    mut s: *const TelPoint,
) -> HacPrediction {
    let mut p: HacPrediction = HacPrediction {
        valid: 0 as ::core::ffi::c_int != 0,
        s_go: 0.,
        psi_t: 0.,
        tx: 0.,
        ty: 0.,
        tangent_dist: 0.,
        acq_turn: 0.,
        acq_arc: 0.,
        hac_arc: 0.,
        turn_cost: 0.,
        e_ref: 0.,
    };
    let mut D: ::core::ffi::c_double = (*x).c.d_station;
    let mut tx: ::core::ffi::c_double = 0.;
    let mut ty: ::core::ffi::c_double = 0.;
    let mut psi_t: ::core::ffi::c_double = 0.;
    if !hac_spiral_tangent(
        side,
        rf,
        D,
        (*s).x,
        (*s).y,
        &raw mut tx,
        &raw mut ty,
        &raw mut psi_t,
    ) {
        return p;
    }
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut vh: ::core::ffi::c_double = fmax(
        1.0f64,
        if (*s).ground_speed > 0.0f64 { (*s).ground_speed } else { (*s).v * cos((*s).g) },
    );
    let mut bank: ::core::ffi::c_double = 0.8f64
        * hac_bank_limit(mach, TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int);
    let mut r_acq: ::core::ffi::c_double = vh * vh / (TEL_G0 * tan(bank * DEG2RAD));
    let mut dist: ::core::ffi::c_double = hypot(tx - (*s).x, ty - (*s).y);
    let mut turn: ::core::ffi::c_double = tel_wrap(
        atan2(ty - (*s).y, tx - (*s).x) - (*s).psi,
    );
    let mut advance: ::core::ffi::c_double = r_acq
        * sin(fmin(fabs(turn), 0.5f64 * TEL_PI));
    p.valid = true_0 != 0;
    p.psi_t = psi_t;
    p.tx = tx;
    p.ty = ty;
    p.acq_turn = turn;
    p.acq_arc = r_acq * fabs(turn);
    p.tangent_dist = fmax(0.0f64, dist - advance);
    p.hac_arc = hac_arc_length(rf, psi_t);
    p.s_go = p.acq_arc + p.tangent_dist + p.hac_arc;
    p.turn_cost = hac_turn_cost(x, R, rf, psi_t, p.acq_arc, bank, p.s_go);
    let mut e: ::core::ffi::c_double = 0.;
    hac_reference_at(
        R,
        p.s_go,
        &raw mut e,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    p.e_ref = e + p.turn_cost;
    return p;
}
unsafe extern "C" fn hac_energy(mut s: *const TelPoint) -> ::core::ffi::c_double {
    return (*s).h + (*s).v * (*s).v / (2.0f64 * TEL_G0);
}
unsafe extern "C" fn hac_band_high(
    mut x: *const HacCtx,
    mut e_ref: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return 1000.0f64 + 0.06f64 * fmax(0.0f64, e_ref - (*x).c.e_station);
}
unsafe extern "C" fn hac_band_sturn(
    mut x: *const HacCtx,
    mut e_ref: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return 1500.0f64 + 0.10f64 * fmax(0.0f64, e_ref - (*x).c.e_station);
}
unsafe extern "C" fn hac_band_low(
    mut x: *const HacCtx,
    mut e_ref: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return 600.0f64 + 0.03f64 * fmax(0.0f64, e_ref - (*x).c.e_station);
}
unsafe extern "C" fn hac_energy_cost(
    mut e_err: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if e_err >= 0.0f64 { e_err } else { 2.5f64 * -e_err };
}
unsafe extern "C" fn hac_size_side(
    mut x: *const HacCtx,
    mut R: *const TaemHacReference,
    mut side: ::core::ffi::c_int,
    mut s: *const TelPoint,
    mut rf_out: *mut ::core::ffi::c_double,
) -> HacPrediction {
    static mut radii: [::core::ffi::c_double; 12] = [
        2200.0f64,
        2600.0f64,
        3000.0f64,
        3500.0f64,
        4000.0f64,
        4500.0f64,
        5000.0f64,
        5500.0f64,
        6000.0f64,
        7000.0f64,
        8000.0f64,
        9000.0f64,
    ];
    let mut best: HacPrediction = HacPrediction {
        valid: 0 as ::core::ffi::c_int != 0,
        s_go: 0.,
        psi_t: 0.,
        tx: 0.,
        ty: 0.,
        tangent_dist: 0.,
        acq_turn: 0.,
        acq_arc: 0.,
        hac_arc: 0.,
        turn_cost: 0.,
        e_ref: 0.,
    };
    let mut best_cost: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut e: ::core::ffi::c_double = hac_energy(s);
    let mut rf_min: ::core::ffi::c_double = fmax(
        HAC_RF_MIN,
        (*x).c.v_station * (*x).c.v_station / (TEL_G0 * tan(50.0f64 * DEG2RAD)),
    );
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[::core::ffi::c_double; 12]>() as usize)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
    {
        let mut rf: ::core::ffi::c_double = fmax(rf_min, radii[i as usize]);
        let mut p: HacPrediction = hac_predict_acquisition(x, R, side, rf, s);
        if p.valid {
            let mut cost: ::core::ffi::c_double = hac_energy_cost(e - p.e_ref);
            if cost < best_cost {
                best_cost = cost;
                best = p;
                *rf_out = rf;
            }
        }
        i = i.wrapping_add(1);
    }
    return best;
}
unsafe extern "C" fn hac_context(
    mut x: *mut HacCtx,
    mut control: *const TerminalModel,
    mut recorded: *const AeroTable,
    mut live: *const TaemHacLiveAero,
    mut geo: *const TaemGeometryState,
    mut mass: ::core::ffi::c_double,
    mut aoa_deg: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut s: *mut TelPoint,
) {
    tel_setup(control, geo, mass, aoa_deg, bank_deg, &raw mut (*x).c, s);
    (*x).aero = recorded;
    (*x).live = live;
    (*x).mass = mass;
    (*x).aoa_supersonic = (*x).c.aoa_max;
}
unsafe extern "C" fn hac_select(
    mut H: *mut TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut keep_bias: bool,
) -> ::core::ffi::c_double {
    let mut e: ::core::ffi::c_double = hac_energy(s);
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut best_side: ::core::ffi::c_int = if (*H).side != 0 {
        (*H).side
    } else {
        1 as ::core::ffi::c_int
    };
    let mut best_rf: ::core::ffi::c_double = (*H).rf;
    let (mut q, mut mach) = (0.0f64, 0.0f64);
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut supersonic: bool = mach > HAC_DRAG_EXIT_MACH;
    let mut side: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while side <= 1 as ::core::ffi::c_int {
        let mut rf: ::core::ffi::c_double = (*H).rf;
        let mut p: HacPrediction = hac_size_side(
            x,
            &raw mut (*H).ref_0,
            side,
            s,
            &raw mut rf,
        );
        if !(!p.valid || p.psi_t > 2.0f64 * TEL_PI - 0.05f64) {
            let mut e_side: ::core::ffi::c_double = p.e_ref;
            if supersonic {
                // Above the drag-phase exit Mach the side is the one whose held drag
                // path (the energy target the drag phase will fly) fits the energy.
                let mut held: TaemHacLawState = *H;
                held.rf = rf;
                let (mut s_go, mut e_drag, mut bearing) = (0.0f64, f64::NAN, 0.0f64);
                hac_drag_path(&raw const held, x, s, side, &raw mut s_go, &raw mut e_drag, &raw mut bearing);
                if e_drag.is_finite() {
                    e_side = e_drag;
                }
            }
            let mut cost: ::core::ffi::c_double = hac_energy_cost(e - e_side);
            if keep_bias as ::core::ffi::c_int != 0 && side != (*H).side {
                cost += HAC_SELECT_MARGIN_M;
            }
            if cost < best {
                best = cost;
                best_side = side;
                best_rf = rf;
            }
        }
        side += 2 as ::core::ffi::c_int;
    }
    if best.is_finite() as i32 != 0 {
        (*H).side = best_side;
        (*H).rf = best_rf;
    }
    return best;
}
unsafe extern "C" fn hac_initialize(
    mut H: *mut TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut ut: ::core::ffi::c_double,
) {
    *H = TaemHacLawState {
        initialized: true_0 != 0,
        final_line: false,
        phase: TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int,
        side: if (*s).y <= 0.0f64 {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        },
        s_turn_sign: 0,
        drag_sign: 0,
        drag_away_sign: 0,
        drag_away: false,
        drag_flown: false,
        drag_aoa_cmd: 0.,
        drag_bank_cmd: 0.,
        trace: false,
        drag_required: 0.,
        rf: clampd(
            (*x).c.v_station * (*x).c.v_station
                / (TEL_G0 * tan(HAC_BANK_NOMINAL * DEG2RAD)),
            HAC_RF_MIN,
            HAC_RF_MAX,
        ),
        psi: 0.,
        last_ang: ::core::f32::NAN as ::core::ffi::c_double,
        s_go: 0.,
        e_ref: 0.,
        h_ref: 0.,
        v_ref: 0.,
        e_err: 0.,
        energy_level: 0.,
        tangent_x: 0.,
        tangent_y: 0.,
        tangent_dist: ::core::f32::INFINITY as ::core::ffi::c_double,
        turn_cost: 0.,
        sdot: ::core::f32::NAN as ::core::ffi::c_double,
        last_sgo: ::core::f32::NAN as ::core::ffi::c_double,
        last_sgo_ut: -::core::f32::INFINITY as ::core::ffi::c_double,
        eta_live: ::core::f32::NAN as ::core::ffi::c_double,
        last_plan_ut: -::core::f32::INFINITY as ::core::ffi::c_double,
        last_side_ut: ut,
        last_ut: ut,
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
    hac_reference_build(&raw mut (*H).ref_0, x, ut);
    hac_select(H, x, s, false_0 != 0);
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    if mach > HAC_DRAG_EXIT_MACH {
        (*H).phase = TAEM_HAC_PHASE_DRAG as ::core::ffi::c_int;
        (*H).drag_flown = true_0 != 0;
        (*H).drag_aoa_cmd = (*s).aoa;
        (*H).drag_bank_cmd = ::core::f32::NAN as ::core::ffi::c_double;
    }
}
unsafe extern "C" fn hac_vertical(
    mut H: *const TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut bank_deg: ::core::ffi::c_double,
    mut drag_accel: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut e: ::core::ffi::c_double = hac_energy(s);
    let mut dhds: ::core::ffi::c_double = 0.;
    hac_reference_at(
        &raw const (*H).ref_0,
        (*H).s_go,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut dhds,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    let mut vh: ::core::ffi::c_double = (*s).v * cos((*s).g);
    let mut sdot: ::core::ffi::c_double = if (*H).sdot.is_finite() as i32 != 0 {
        fmin(0.0f64, (*H).sdot)
    } else {
        -vh
    };
    let mut hdot: ::core::ffi::c_double = (*s).v * sin((*s).g);
    let mut hdot_ref: ::core::ffi::c_double = dhds * sdot;
    let mut wn: ::core::ffi::c_double = 0.06f64
        + 0.09f64 * clampd((20000.0f64 - (*s).h) / 17000.0f64, 0.0f64, 1.0f64);
    let mut z: ::core::ffi::c_double = 0.9f64;
    let mut damp: ::core::ffi::c_double = 2.0f64 * z * wn * (hdot_ref - hdot);
    let mut hdd_h: ::core::ffi::c_double = wn * wn * ((*H).h_ref - (*s).h) + damp;
    let mut hdd_hi: ::core::ffi::c_double = wn * wn
        * ((*H).e_ref + hac_band_high(x, (*H).e_ref) - e) + damp;
    let mut hdd_lo: ::core::ffi::c_double = wn * wn
        * ((*H).e_ref - hac_band_low(x, (*H).e_ref) - e) + damp;
    let mut hdd: ::core::ffi::c_double = hdd_h;
    if hdd > hdd_hi {
        hdd = hdd_hi;
    }
    if hdd < hdd_lo {
        hdd = hdd_lo;
    }
    let mut grav: ::core::ffi::c_double = tel_gravity(&raw const (*x).c, (*s).h);
    let mut v: ::core::ffi::c_double = fmax((*s).v, 1.0f64);
    let mut vdot: ::core::ffi::c_double = -drag_accel - grav * sin((*s).g);
    let mut gdot: ::core::ffi::c_double = (hdd - vdot * sin((*s).g))
        / (v * fmax(0.2f64, cos((*s).g)));
    let mut safe: ::core::ffi::c_double = (*(*x).c.model).vehicle.minimum_safe_speed;
    let mut cphi: ::core::ffi::c_double = fmax(0.3f64, cos(fabs(bank_deg) * DEG2RAD));
    let mut v_min: ::core::ffi::c_double = if safe > 0.0f64 {
        1.1f64 * safe / sqrt(cphi)
    } else {
        0.0f64
    };
    if v_min > 0.0f64 && (*s).v < v_min + 15.0f64 {
        let mut need: ::core::ffi::c_double = drag_accel
            + (v_min + 15.0f64 - (*s).v) / 4.0f64;
        let mut g_max: ::core::ffi::c_double = -asin(
            clampd(need / grav, 0.0f64, 1.0f64),
        );
        gdot = fmin(gdot, (g_max - (*s).g) / 3.0f64);
    }
    let mut q_cap: ::core::ffi::c_double = fmin(
        (*(*x).c.model).vehicle.maximum_dynamic_pressure * 0.5f64,
        2.0f64 * hac_q_ref(x, &raw const (*H).ref_0, mach, (*H).s_go),
    );
    if q > q_cap {
        let mut v_cap: ::core::ffi::c_double = (*s).v * sqrt(q_cap / q);
        let mut need_0: ::core::ffi::c_double = -((*s).v - v_cap) / 4.0f64 - drag_accel;
        let mut g_min: ::core::ffi::c_double = -asin(
            clampd(need_0 / grav, -1.0f64, 1.0f64),
        );
        gdot = fmax(gdot, (g_min - (*s).g) / 3.0f64);
    }
    if (*s).g < -35.0f64 * DEG2RAD {
        gdot = fmax(gdot, 0.0f64);
    }
    if (*s).g > 6.0f64 * DEG2RAD {
        gdot = fmin(gdot, 0.0f64);
    }
    gdot = clampd(gdot, -0.08f64, 0.08f64);
    let mut lift: ::core::ffi::c_double = (*s).m * (grav * cos((*s).g) + v * gdot)
        / cphi;
    let mut aoa: ::core::ffi::c_double = hac_lift_aoa(x, mach, q, lift);
    if (*H).e_err < -hac_band_low(x, (*H).e_ref) && (*s).v > v_min + 15.0f64 {
        aoa = fmin(aoa, tel_best_ld_aoa(&raw const (*x).c, mach));
    }
    return clampd(aoa, TEL_AOA_MIN, (*x).c.aoa_max);
}
unsafe extern "C" fn hac_lateral_left(
    mut H: *mut TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut acq: *const HacPrediction,
) -> ::core::ffi::c_double {
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut vh: ::core::ffi::c_double = fmax(
        1.0f64,
        if (*s).ground_speed > 0.0f64 { (*s).ground_speed } else { (*s).v * cos((*s).g) },
    );
    let mut D: ::core::ffi::c_double = (*x).c.d_station;
    let mut lim: ::core::ffi::c_double = hac_bank_limit(mach, (*H).phase);
    let mut bank: ::core::ffi::c_double = 0.0f64;
    if (*H).phase == TAEM_HAC_PHASE_STURN as ::core::ffi::c_int {
        let mut dpsi: ::core::ffi::c_double = if !acq.is_null()
            && (*acq).valid as ::core::ffi::c_int != 0
        {
            tel_wrap(atan2((*acq).ty - (*s).y, (*acq).tx - (*s).x) - (*s).psi)
        } else {
            0.0f64
        };
        bank = if fabs(dpsi) < 0.5f64 * TEL_PI {
            (*H).s_turn_sign as ::core::ffi::c_double * lim
        } else {
            0.0f64
        };
    } else if (*H).phase == TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int {
        let mut dpsi_0: ::core::ffi::c_double = if !acq.is_null()
            && (*acq).valid as ::core::ffi::c_int != 0
        {
            tel_wrap(atan2((*acq).ty - (*s).y, (*acq).tx - (*s).x) - (*s).psi)
        } else {
            tel_wrap(
                atan2((*H).side as ::core::ffi::c_double * (*H).rf - (*s).y, -D - (*s).x)
                    - (*s).psi,
            )
        };
        bank = clampd(HAC_GPHI * dpsi_0 * RAD2DEG, -lim, lim);
    } else if (*H).phase == TAEM_HAC_PHASE_HAC as ::core::ffi::c_int {
        let mut cx: ::core::ffi::c_double = -D;
        let mut cy: ::core::ffi::c_double = (*H).side as ::core::ffi::c_double * (*H).rf;
        let mut rx: ::core::ffi::c_double = (*s).x - cx;
        let mut ry: ::core::ffi::c_double = (*s).y - cy;
        let mut rc: ::core::ffi::c_double = hypot(rx, ry);
        let mut r: ::core::ffi::c_double = hac_radius_at((*H).rf, (*H).psi);
        let mut rdot: ::core::ffi::c_double = (rx * vh * cos((*s).psi)
            + ry * vh * sin((*s).psi)) / fmax(1.0f64, rc);
        let mut rdot_ref: ::core::ffi::c_double = -vh * 2.0f64 * HAC_R2 * (*H).psi
            / fmax(1.0f64, r);
        let mut acc: ::core::ffi::c_double = vh * vh / r + 0.0055f64 * (rc - r)
            + 0.222f64 * (rdot - rdot_ref);
        bank = (*H).side as ::core::ffi::c_double
            * clampd(
                atan(acc / (TEL_G0 * cos((*s).g))) * RAD2DEG,
                -HAC_BANK_OUTWARD,
                lim,
            );
    } else {
        let mut ydot: ::core::ffi::c_double = vh * sin((*s).psi);
        let mut acc_0: ::core::ffi::c_double = -(0.0225f64 * (*s).y + 0.219f64 * ydot);
        bank = atan(acc_0 / (TEL_G0 * cos((*s).g))) * RAD2DEG;
        lim = clampd(0.43f64 * fabs(bank) + 17.0f64, 30.0f64, 60.0f64);
        bank = clampd(bank, -lim, lim);
    }
    lim = fmin(lim, (*(*x).c.model).vehicle.maximum_bank_angle);
    let mut lift: ::core::ffi::c_double = 0.;
    let mut drag: ::core::ffi::c_double = 0.;
    tel_forces(
        &raw const (*x).c,
        s,
        (*s).aoa,
        &raw mut q,
        &raw mut mach,
        &raw mut lift,
        &raw mut drag,
    );
    lim = entry_bank_authority_limit(
        (*s).v,
        q,
        lift / ((*s).m * TEL_G0),
        &raw const (*(*x).c.model).vehicle,
        lim,
    );
    return clampd(bank, -lim, lim);
}
/// MM305's drag-phase energy target: the route of `side_hold` (±1) when it is
/// valid, else the least-energy side, plus the supersonic deceleration allowance.
/// Holding the selected side keeps the drag phase from shedding the energy a
/// spiral route needs.
unsafe extern "C" fn hac_drag_path(
    mut H: *const TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut side_hold: ::core::ffi::c_int,
    mut s_go: *mut ::core::ffi::c_double,
    mut e_tgt: *mut ::core::ffi::c_double,
    mut bearing: *mut ::core::ffi::c_double,
) {
    let mut D: ::core::ffi::c_double = (*x).c.d_station;
    let mut gx: ::core::ffi::c_double = -D - (*H).ref_0.s_gate;
    let mut v_gate: ::core::ffi::c_double = 0.;
    hac_reference_at(
        &raw const (*H).ref_0,
        (*H).ref_0.s_gate,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut v_gate,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
    );
    let mut sub: TelPoint = *s;
    if (*s).v > v_gate {
        let mut k: ::core::ffi::c_double = v_gate / (*s).v;
        sub.v *= k;
        sub.ground_speed *= k;
    }
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut best_e: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut bx: ::core::ffi::c_double = -D;
    let mut by: ::core::ffi::c_double = 0.0f64;
    let mut held: (f64, f64, f64, f64) = (f64::NAN, f64::NAN, f64::NAN, f64::NAN);
    let mut side: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while side <= 1 as ::core::ffi::c_int {
        let mut p: HacPrediction = hac_predict_acquisition(
            x,
            &raw const (*H).ref_0,
            side,
            (*H).rf,
            &raw mut sub,
        );
        let mut len: ::core::ffi::c_double = 0.;
        let mut e: ::core::ffi::c_double = 0.;
        let mut tx: ::core::ffi::c_double = 0.;
        let mut ty: ::core::ffi::c_double = 0.;
        if p.valid {
            len = p.s_go;
            e = p.e_ref;
            tx = p.tx;
            ty = p.ty;
        } else {
            len = (*H).rf + hac_arc_length((*H).rf, 2.0f64 * TEL_PI);
            hac_reference_at(
                &raw const (*H).ref_0,
                len,
                &raw mut e,
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
            );
            e
                += hac_turn_cost(
                    x,
                    &raw const (*H).ref_0,
                    (*H).rf,
                    2.0f64 * TEL_PI,
                    0.0f64,
                    0.0f64,
                    len,
                );
            tx = -D;
            ty = side as ::core::ffi::c_double * 2.0f64 * (*H).rf;
        }
        if best_e.is_finite() as i32 == 0 || e < best_e {
            best = len;
            best_e = e;
            bx = tx;
            by = ty;
        }
        if p.valid as ::core::ffi::c_int != 0 && side == side_hold {
            held = (len, e, tx, ty);
        }
        side += 2 as ::core::ffi::c_int;
    }
    if held.1.is_finite() {
        (best, best_e, bx, by) = held;
    }
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    if mach > 1.0f64 {
        let mut m_mid: ::core::ffi::c_double = 0.5f64 * (mach + 1.0f64);
        let mut cl: ::core::ffi::c_double = 0.;
        let mut cd: ::core::ffi::c_double = 0.;
        hac_aero(
            x,
            m_mid,
            hac_best_ld_aoa(x, m_mid, hac_drag_aoa_ceiling(x, m_mid)),
            &raw mut cl,
            &raw mut cd,
        );
        let mut eta_sup: ::core::ffi::c_double = if cl > 1e-3f64 {
            cd / cl
        } else {
            1.0f64
        };
        let mut v1: ::core::ffi::c_double = (*s).v / mach;
        let mut slope: ::core::ffi::c_double = 0.;
        hac_reference_at(
            &raw const (*H).ref_0,
            best,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            &raw mut slope,
        );
        let mut d: ::core::ffi::c_double = ((*s).v * (*s).v - v1 * v1)
            / (2.0f64 * TEL_G0 * fmax(0.3f64, eta_sup));
        best_e += fmax(0.0f64, eta_sup - slope) * d;
    }
    *s_go = best;
    *e_tgt = best_e;
    if (*s).x < gx {
        bx = fmin(-D, fmax(gx, (*s).x + HAC_DRAG_LOOKAHEAD));
        by = 0.0f64;
    }
    *bearing = atan2(by - (*s).y, bx - (*s).x);
}
unsafe extern "C" fn hac_drag_command(
    mut H: *mut TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut aoa_out: *mut ::core::ffi::c_double,
    mut bank_left_out: *mut ::core::ffi::c_double,
) {
    let mut q: ::core::ffi::c_double = 0.;
    let mut mach: ::core::ffi::c_double = 0.;
    tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
    let mut grav: ::core::ffi::c_double = tel_gravity(&raw const (*x).c, (*s).h);
    let mut v: ::core::ffi::c_double = fmax((*s).v, 1.0f64);
    let mut vh: ::core::ffi::c_double = fmax(
        1.0f64,
        if (*s).ground_speed > 0.0f64 { (*s).ground_speed } else { v * cos((*s).g) },
    );
    let mut area: ::core::ffi::c_double = (*(*x).aero).reference_area_m2;
    let mut s_go: ::core::ffi::c_double = 0.;
    let mut bearing: ::core::ffi::c_double = 0.;
    let mut e_tgt: ::core::ffi::c_double = 0.;
    let mut deds: ::core::ffi::c_double = 0.;
    hac_drag_path(H, x, s, (*H).side, &raw mut s_go, &raw mut e_tgt, &raw mut bearing);
    hac_reference_at(
        &raw mut (*H).ref_0,
        s_go,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut deds,
    );
    let mut run: ::core::ffi::c_double = fmax(
        HAC_DRAG_MIN_RUN,
        s_go - (*H).ref_0.s_gate,
    );
    let mut per_metre: ::core::ffi::c_double = grav * vh / v;
    let mut drag_req: ::core::ffi::c_double = fmax(
        0.0f64,
        deds + (hac_energy(s) - e_tgt) / run,
    ) * per_metre;
    let mut ceiling: ::core::ffi::c_double = hac_drag_aoa_ceiling(x, mach);
    let mut a_max: ::core::ffi::c_double = fmin(
        ceiling,
        hac_lift_aoa_to(x, mach, q, HAC_DRAG_MAX_LOAD * (*s).m * grav, ceiling),
    );
    let mut cl: ::core::ffi::c_double = 0.;
    let mut cd: ::core::ffi::c_double = 0.;
    hac_aero(x, mach, a_max, &raw mut cl, &raw mut cd);
    let mut drag_max: ::core::ffi::c_double = cd * q * area / (*s).m;
    let mut lo: ::core::ffi::c_double = TEL_AOA_MIN;
    let mut hi: ::core::ffi::c_double = fmax(TEL_AOA_MIN, a_max);
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 30 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (lo + hi);
        hac_aero(x, mach, mid, &raw mut cl, &raw mut cd);
        if cd * q * area / (*s).m < drag_req {
            lo = mid;
        } else {
            hi = mid;
        }
        i += 1;
    }
    let mut aoa: ::core::ffi::c_double = 0.5f64 * (lo + hi);
    if drag_req > drag_max {
        (*H).drag_away = true_0 != 0;
    } else if drag_req < 0.7f64 * drag_max {
        (*H).drag_away = false_0 != 0;
    }
    let mut dpsi_gate: ::core::ffi::c_double = tel_wrap(bearing - (*s).psi);
    if (*H).drag_away_sign == 0 {
        (*H).drag_away_sign = if dpsi_gate >= 0.0f64 {
            -(1 as ::core::ffi::c_int)
        } else {
            1 as ::core::ffi::c_int
        };
    }
    let mut swing: ::core::ffi::c_double = HAC_DRAG_STURN_DEG * DEG2RAD;
    if (*H).drag_away as ::core::ffi::c_int != 0
        && (*H).drag_away_sign as ::core::ffi::c_double * -dpsi_gate
            >= swing - 5.0f64 * DEG2RAD
    {
        (*H).drag_away_sign = -(*H).drag_away_sign;
    }
    let mut target: ::core::ffi::c_double = if (*H).drag_away as ::core::ffi::c_int != 0
    {
        bearing + (*H).drag_away_sign as ::core::ffi::c_double * swing
    } else {
        bearing
    };
    let mut dpsi: ::core::ffi::c_double = tel_wrap(target - (*s).psi);
    let mut shed: ::core::ffi::c_double = if drag_max > 0.0f64 {
        fmax(0.0f64, hac_energy(s) - e_tgt) * per_metre / drag_max
    } else {
        0.0f64
    };
    let mut g_ref: ::core::ffi::c_double = -atan2(
        fmax(0.0f64, (*s).h - (*H).ref_0.h_gate),
        fmax(run, shed),
    );
    let mut gdot: ::core::ffi::c_double = clampd(
        (g_ref - (*s).g) / HAC_DRAG_GAMMA_TAU,
        -0.08f64,
        0.08f64,
    );
    if (*s).g > 3.0f64 * DEG2RAD {
        gdot = fmin(gdot, 0.0f64);
    }
    if (*s).g < -35.0f64 * DEG2RAD {
        gdot = fmax(gdot, 0.0f64);
    }
    let mut lift_v: ::core::ffi::c_double = (*s).m * (grav * cos((*s).g) + v * gdot);
    hac_aero(x, mach, aoa, &raw mut cl, &raw mut cd);
    let mut lift: ::core::ffi::c_double = cl * q * area;
    let mut lim: ::core::ffi::c_double = fmin(
        HAC_DRAG_BANK_MAX,
        (*(*x).c.model).vehicle.maximum_bank_angle,
    );
    let mut steer: ::core::ffi::c_double = clampd(HAC_GPHI * dpsi * RAD2DEG, -lim, lim);
    let mut bank: ::core::ffi::c_double = if lift > lift_v && lift > 0.0f64 {
        acos(clampd(lift_v / lift, cos(lim * DEG2RAD), 1.0f64)) * RAD2DEG
    } else {
        0.0f64
    };
    if (*H).drag_sign == 0 || fabs(dpsi) * RAD2DEG > HAC_DRAG_DEADBAND_DEG {
        (*H).drag_sign = if dpsi >= 0.0f64 {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        };
        bank = fmax(bank, fabs(steer));
    }
    let mut need: ::core::ffi::c_double = lift_v / fmax(0.2f64, cos(bank * DEG2RAD));
    if lift < need {
        aoa = fmax(aoa, fmin(a_max, hac_lift_aoa_to(x, mach, q, need, ceiling)));
    }
    if hac_energy(s) < e_tgt && (*s).g > -30.0f64 * DEG2RAD {
        aoa = fmin(aoa, fmax(TEL_AOA_MIN, hac_best_ld_aoa(x, mach, ceiling)));
    }
    let mut lift_now: ::core::ffi::c_double = 0.;
    let mut drag_now: ::core::ffi::c_double = 0.;
    tel_forces(
        &raw const (*x).c,
        s,
        (*s).aoa,
        &raw mut q,
        &raw mut mach,
        &raw mut lift_now,
        &raw mut drag_now,
    );
    lim = entry_bank_authority_limit(
        (*s).v,
        q,
        lift_now / ((*s).m * TEL_G0),
        &raw const (*(*x).c.model).vehicle,
        lim,
    );
    (*H).drag_required = drag_req;
    *aoa_out = clampd(aoa, TEL_AOA_MIN, ceiling);
    *bank_left_out = (*H).drag_sign as ::core::ffi::c_double * clampd(bank, 0.0f64, lim);
}
unsafe extern "C" fn hac_update(
    mut H: *mut TaemHacLawState,
    mut x: *const HacCtx,
    mut s: *const TelPoint,
    mut ut: ::core::ffi::c_double,
    mut drag_accel: ::core::ffi::c_double,
) -> HacPrediction {
    let mut D: ::core::ffi::c_double = (*x).c.d_station;
    let mut e: ::core::ffi::c_double = hac_energy(s);
    let mut dt: ::core::ffi::c_double = if ut > (*H).last_ut {
        fmin(ut - (*H).last_ut, 1.0f64)
    } else {
        0.0f64
    };
    (*H).last_ut = ut;
    if !(*H).ref_0.valid || ut - (*H).ref_0.built_ut >= HAC_REF_REBUILD_S
        || fabs((*H).ref_0.mass - (*x).mass) > 0.01f64 * (*x).mass
    {
        hac_reference_build(&raw mut (*H).ref_0, x, ut);
    }
    let mut vh: ::core::ffi::c_double = fmax(1.0f64, (*s).v * cos((*s).g));
    let mut eta: ::core::ffi::c_double = drag_accel * (*s).v / (TEL_G0 * vh);
    if eta.is_finite() as i32 != 0 && dt > 0.0f64 {
        (*H).eta_live = if (*H).eta_live.is_finite() as i32 != 0 {
            (*H).eta_live + dt / (5.0f64 + dt) * (eta - (*H).eta_live)
        } else {
            eta
        };
    }
    let mut acq: HacPrediction = HacPrediction {
        valid: 0 as ::core::ffi::c_int != 0,
        s_go: 0.,
        psi_t: 0.,
        tx: 0.,
        ty: 0.,
        tangent_dist: 0.,
        acq_turn: 0.,
        acq_arc: 0.,
        hac_arc: 0.,
        turn_cost: 0.,
        e_ref: 0.,
    };
    if (*H).phase <= TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int {
        if ut - (*H).last_side_ut >= HAC_UPDATE_S
            && (*H).tangent_dist > HAC_SELECT_FREEZE_M
        {
            (*H).last_side_ut = ut;
            hac_select(H, x, s, true_0 != 0);
        }
        acq = hac_predict_acquisition(x, &raw mut (*H).ref_0, (*H).side, (*H).rf, s);
        if !acq.valid && (*H).phase != TAEM_HAC_PHASE_DRAG as ::core::ffi::c_int
            && hypot((*s).x + D, (*s).y - (*H).side as ::core::ffi::c_double * (*H).rf)
                < 0.9f64
                    * hac_radius_at(
                        (*H).rf,
                        hac_point_psi((*H).side, (*H).rf, D, (*s).x, (*s).y),
                    )
        {
            (*H).last_side_ut = ut;
            hac_select(H, x, s, false_0 != 0);
            acq = hac_predict_acquisition(x, &raw mut (*H).ref_0, (*H).side, (*H).rf, s);
        }
        let mut cx: ::core::ffi::c_double = -D;
        let mut cy: ::core::ffi::c_double = (*H).side as ::core::ffi::c_double * (*H).rf;
        let mut rc: ::core::ffi::c_double = hypot((*s).x - cx, (*s).y - cy);
        let mut psi_j: ::core::ffi::c_double = if acq.valid as ::core::ffi::c_int != 0 {
            acq.psi_t
        } else {
            hac_point_psi((*H).side, (*H).rf, D, (*s).x, (*s).y)
        };
        let mut along_hac: ::core::ffi::c_double = tel_wrap(
            (*s).psi
                - (atan2((*s).y - cy, (*s).x - cx)
                    + (*H).side as ::core::ffi::c_double * 0.5f64 * TEL_PI),
        );
        let mut vh_cap: ::core::ffi::c_double = fmax(
            1.0f64,
            if (*s).ground_speed > 0.0f64 {
                (*s).ground_speed
            } else {
                (*s).v * cos((*s).g)
            },
        );
        let mut inside: bool = (!acq.valid
            || rc < 1.1f64 * hac_radius_at((*H).rf, psi_j)
                && acq.tangent_dist < fmax(500.0f64, 4.0f64 * vh_cap))
            && fabs(along_hac) < TEL_PI / 3.0f64;
        if inside as ::core::ffi::c_int != 0
            && (*H).phase != TAEM_HAC_PHASE_DRAG as ::core::ffi::c_int
        {
            (*H).phase = TAEM_HAC_PHASE_HAC as ::core::ffi::c_int;
            (*H).psi = psi_j;
            if (*H).psi > 2.0f64 * TEL_PI - 0.3f64 {
                (*H).psi = 0.0f64;
            }
            (*H).last_ang = atan2((*s).y - cy, (*s).x - cx);
        } else if acq.valid {
            (*H).s_go = acq.s_go;
            (*H).turn_cost = acq.turn_cost;
            (*H).tangent_x = acq.tx;
            (*H).tangent_y = acq.ty;
            (*H).tangent_dist = acq.tangent_dist + acq.acq_arc;
            (*H).psi = acq.psi_t;
            (*H).e_ref = acq.e_ref;
        }
    }
    if (*H).phase == TAEM_HAC_PHASE_HAC as ::core::ffi::c_int {
        let mut cx_0: ::core::ffi::c_double = -D;
        let mut cy_0: ::core::ffi::c_double = (*H).side as ::core::ffi::c_double
            * (*H).rf;
        let mut ang: ::core::ffi::c_double = atan2((*s).y - cy_0, (*s).x - cx_0);
        if (*H).last_ang.is_finite() as i32 != 0 {
            (*H).psi = clampd(
                (*H).psi
                    - (*H).side as ::core::ffi::c_double * tel_wrap(ang - (*H).last_ang),
                0.0f64,
                (*H).psi,
            );
        }
        (*H).last_ang = ang;
        let mut e_hac: ::core::ffi::c_double = 0.;
        hac_reference_at(
            &raw mut (*H).ref_0,
            hac_arc_length((*H).rf, (*H).psi),
            &raw mut e_hac,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
        (*H).s_go = hac_arc_length((*H).rf, (*H).psi);
        (*H).turn_cost = hac_turn_cost(
            x,
            &raw mut (*H).ref_0,
            (*H).rf,
            (*H).psi,
            0.0f64,
            0.0f64,
            0.0f64,
        );
        (*H).e_ref = e_hac + (*H).turn_cost;
        (*H).tangent_dist = 0.0f64;
        let mut err: ::core::ffi::c_double = e - (*H).e_ref;
        if (*H).psi > 0.5f64 * TEL_PI && fabs(err) > 300.0f64 && dt > 0.0f64 {
            let mut deds: ::core::ffi::c_double = 0.;
            hac_reference_at(
                &raw mut (*H).ref_0,
                hac_arc_length((*H).rf, 0.5f64 * (*H).psi),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                ::core::ptr::null_mut::<::core::ffi::c_double>(),
                &raw mut deds,
            );
            let mut rf_target: ::core::ffi::c_double = (*H).rf
                + err / (fmax(0.05f64, deds) * (*H).psi);
            let mut rf_min: ::core::ffi::c_double = fmax(
                fmax(
                    HAC_RF_MIN,
                    (*x).c.v_station * (*x).c.v_station
                        / (TEL_G0 * tan(50.0f64 * DEG2RAD)),
                ),
                (*s).v * (*s).v / (TEL_G0 * tan(HAC_BANK_SIZING * DEG2RAD))
                    - HAC_R2 * (*H).psi * (*H).psi,
            );
            rf_target = clampd(rf_target, rf_min, HAC_RF_MAX);
            let mut rf_new: ::core::ffi::c_double = clampd(
                (*H).rf
                    + clampd(0.15f64 * (rf_target - (*H).rf), -HAC_RF_RATE, HAC_RF_RATE)
                        * dt,
                HAC_RF_MIN,
                HAC_RF_MAX,
            );
            let mut moved: ::core::ffi::c_double = fabs(rf_new - (*H).rf)
                / fmax(1.0f64, (*H).rf);
            (*H).rf = rf_new;
            let mut p: ::core::ffi::c_double = hac_point_psi(
                (*H).side,
                (*H).rf,
                D,
                (*s).x,
                (*s).y,
            );
            p += 2.0f64 * TEL_PI * round(((*H).psi - p) / (2.0f64 * TEL_PI));
            (*H).psi = clampd(p, 0.0f64, (*H).psi + moved);
            (*H).last_ang = atan2(
                (*s).y - (*H).side as ::core::ffi::c_double * (*H).rf,
                (*s).x - cx_0,
            );
        }
        let mut rc_0: ::core::ffi::c_double = hypot(
            (*s).x - cx_0,
            (*s).y - (*H).side as ::core::ffi::c_double * (*H).rf,
        );
        if rc_0 - hac_radius_at((*H).rf, (*H).psi) > HAC_RERRLM {
            (*H).phase = TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int;
            (*H).tangent_dist = 0.0f64;
        }
        if (*H).psi < 0.05f64
            || (*s).x >= -D - 500.0f64 && fabs(tel_wrap((*s).psi)) < 15.0f64 * DEG2RAD
                && fabs((*s).y) < 1500.0f64
        {
            (*H).phase = TAEM_HAC_PHASE_PREFINAL as ::core::ffi::c_int;
        }
    }
    if (*H).phase == TAEM_HAC_PHASE_PREFINAL as ::core::ffi::c_int {
        (*H).s_go = -D - (*s).x;
        (*H).turn_cost = 0.0f64;
        hac_reference_at(
            &raw mut (*H).ref_0,
            (*H).s_go,
            &raw mut (*H).e_ref,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
    }
    hac_split_ref(
        x,
        &raw mut (*H).ref_0,
        (*H).e_ref,
        fmax(0.0f64, (*H).s_go),
        &raw mut (*H).h_ref,
        &raw mut (*H).v_ref,
    );
    (*H).e_err = e - (*H).e_ref;
    (*H).energy_level = clampd(
        0.5f64 + (*H).e_err / (2.0f64 * hac_band_high(x, (*H).e_ref)),
        0.0f64,
        1.0f64,
    );
    if (*H).last_sgo.is_finite() as i32 != 0 && ut > (*H).last_sgo_ut {
        let mut rate: ::core::ffi::c_double = ((*H).s_go - (*H).last_sgo)
            / (ut - (*H).last_sgo_ut);
        if rate.is_finite() as i32 != 0 && fabs(rate) < 3000.0f64 {
            (*H).sdot = if (*H).sdot.is_finite() as i32 != 0 {
                (*H).sdot + 0.3f64 * (rate - (*H).sdot)
            } else {
                rate
            };
        }
    }
    (*H).last_sgo = (*H).s_go;
    (*H).last_sgo_ut = ut;
    if (*H).phase == TAEM_HAC_PHASE_DRAG as ::core::ffi::c_int {
        let mut q: ::core::ffi::c_double = 0.;
        let mut mach: ::core::ffi::c_double = 0.;
        tel_air(&raw const (*x).c, (*s).h, (*s).v, &raw mut q, &raw mut mach);
        let mut s_go: ::core::ffi::c_double = 0.;
        let mut bearing: ::core::ffi::c_double = 0.;
        let mut e_tgt: ::core::ffi::c_double = 0.;
        hac_drag_path(H, x, s, (*H).side, &raw mut s_go, &raw mut e_tgt, &raw mut bearing);
        (*H).s_go = s_go;
        (*H).e_ref = e_tgt;
        (*H).e_err = e - e_tgt;
        if mach <= HAC_DRAG_EXIT_MACH || e <= (*H).ref_0.e_gate {
            (*H).phase = TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int;
            if (*H).trace {
                fprintf(
                    stderr,
                    b"MM305 HAC drag-energy phase ends: UT %.1f Mach %.2f path %.1f km E %.0f m path E %.0f m\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    ut,
                    mach,
                    s_go / 1000.0f64,
                    e,
                    e_tgt,
                );
            }
        }
    }
    if (*H).phase == TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int
        && (*H).e_err > hac_band_sturn(x, (*H).e_ref) && (*H).s_go > HAC_STURN_MIN_RANGE
        && (*H).psi < HAC_STURN_MAX_PSI
    {
        (*H).phase = TAEM_HAC_PHASE_STURN as ::core::ffi::c_int;
        (*H).s_turn_sign = -(*H).side;
    } else if (*H).phase == TAEM_HAC_PHASE_STURN as ::core::ffi::c_int
        && ((*H).e_err < 0.6f64 * hac_band_sturn(x, (*H).e_ref)
            || (*H).s_go < HAC_STURN_MIN_RANGE)
    {
        (*H).phase = TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int;
    }
    (*H).final_line = (*H).phase == TAEM_HAC_PHASE_PREFINAL as ::core::ffi::c_int
        && tel_final_line_ready(&raw const (*x).c, s) as ::core::ffi::c_int != 0;
    return acq;
}
unsafe extern "C" fn hac_speedbrake(
    mut H: *mut TaemHacLawState,
    mut mach: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if mach > 0.95f64 || (*H).phase < TAEM_HAC_PHASE_ACQ as ::core::ffi::c_int
        || (*H).final_line as ::core::ffi::c_int != 0 || (*H).s_go < 1000.0f64
    {
        (*H).speedbrake_integral = 0.0f64;
        return 0.0f64;
    }
    let mut err: ::core::ffi::c_double = (*H).e_err - 200.0f64;
    (*H).speedbrake_integral = clampd(
        (*H).speedbrake_integral + 2.0e-5f64 * err * dt,
        0.0f64,
        SPEEDBRAKE_CAP,
    );
    return clampd(err / 2500.0f64 + (*H).speedbrake_integral, 0.0f64, SPEEDBRAKE_CAP);
}
unsafe extern "C" fn taem_hac_law_frame_score(
    mut model: *const TerminalModel,
    mut recorded: *const AeroTable,
    mut live: *const TaemHacLiveAero,
    mut geo: *const TaemGeometryState,
    mut mass_kg: ::core::ffi::c_double,
    mut aoa_deg: ::core::ffi::c_double,
    mut bank_deg: ::core::ffi::c_double,
    mut delivery_feasible: *mut bool,
) -> ::core::ffi::c_double {
    if model.is_null() || geo.is_null() || !(mass_kg > 0.0f64)
        || (*geo).runway_along_m.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut x: HacCtx = HacCtx {
        c: TelContext {
            model: ::core::ptr::null::<TerminalModel>(),
            d_station: 0.,
            h_station: 0.,
            v_station: 0.,
            e_station: 0.,
            aoa_max: 0.,
            slope_deg: 0.,
            final_line: false,
        },
        aero: ::core::ptr::null::<AeroTable>(),
        live: ::core::ptr::null::<TaemHacLiveAero>(),
        mass: 0.,
        aoa_supersonic: 0.,
    };
    let mut s: TelPoint = TelPoint {
        x: 0.,
        y: 0.,
        h: 0.,
        v: 0.,
        g: 0.,
        psi: 0.,
        m: 0.,
        aoa: 0.,
        aoa_rate: 0.,
        bank: 0.,
        ground_speed: 0.,
    };
    hac_context(
        &raw mut x,
        model,
        recorded,
        live,
        geo,
        mass_kg,
        aoa_deg,
        bank_deg,
        &raw mut s,
    );
    fn H_tls() -> *mut TaemHacLawState {
        ::std::thread_local! {
            static CELL: ::core::cell::UnsafeCell<TaemHacLawState> = const { ::core::cell::UnsafeCell::new(TaemHacLawState {
        initialized: false,
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
    }) };
        }
        CELL.with(|c| c.get())
    }
    hac_initialize(&raw mut (*H_tls()), &raw mut x, &raw mut s, 0.0f64);
    let mut p: HacPrediction = hac_predict_acquisition(
        &raw mut x,
        &raw mut (*H_tls()).ref_0,
        (*H_tls()).side,
        (*H_tls()).rf,
        &raw mut s,
    );
    if !p.valid {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut err: ::core::ffi::c_double = hac_energy(&raw mut s) - p.e_ref;
    if !getenv(b"KSP_LANDER_HAC_TRACE\0" as *const u8 as *const ::core::ffi::c_char)
        .is_null()
    {
        let mut e0: ::core::ffi::c_double = 0.;
        hac_reference_at(
            &raw mut (*H_tls()).ref_0,
            p.s_go,
            &raw mut e0,
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
            ::core::ptr::null_mut::<::core::ffi::c_double>(),
        );
        fprintf(
            stderr,
            b"HAC_FRAME x=%.0f y=%.0f psi=%.1f side=%+d RF=%.0f psiT=%.0f acqTurn=%.1f acqArc=%.0f tan=%.0f hacArc=%.0f sGo=%.0f Eref0=%.0f turn=%.0f E=%.0f err=%.0f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            s.x,
            s.y,
            s.psi * RAD2DEG,
            (*H_tls()).side,
            (*H_tls()).rf,
            p.psi_t * RAD2DEG,
            p.acq_turn * RAD2DEG,
            p.acq_arc,
            p.tangent_dist,
            p.hac_arc,
            p.s_go,
            e0,
            p.turn_cost,
            hac_energy(&raw mut s),
            err,
        );
    }
    if !delivery_feasible.is_null() {
        *delivery_feasible = err > -hac_band_low(&raw mut x, p.e_ref);
    }
    return hac_energy_cost(err);
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const DBL_MANT_DIG: ::core::ffi::c_int = __DBL_MANT_DIG__;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MANT_DIG__: ::core::ffi::c_int = 53 as ::core::ffi::c_int;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();

/// Offline MM305 mode-selection regression, ported from the former white-box C
/// test (`Validation/TaemEnergyGeometryTests.c`).  A polar inlet that is abeam
/// the runway is not a Final approach just because the along-track station lies
/// behind its current x coordinate.
#[cfg(test)]
mod taem_energy_geometry_tests {
    use super::*;
    use ::core::mem::zeroed;

    #[repr(C)]
    struct TerminalModelSourceFiles {
        atmosphere_csv: *const ::core::ffi::c_char,
        aero_csv: *const ::core::ffi::c_char,
        aero_book_csv: *const ::core::ffi::c_char,
        attitude_ini: *const ::core::ffi::c_char,
    }
    extern "C" {
        fn terminal_model_capture(
            model: *mut TerminalModel,
            files: *const TerminalModelSourceFiles,
            configuration: *const LandingConfiguration,
            snapshot_id: u64,
            reason: *mut ::core::ffi::c_char,
            reason_size: usize,
        ) -> bool;
        fn shuttle_sim_model_path(
            root: *const ::core::ffi::c_char,
            kind: ::core::ffi::c_int,
            buffer: *mut ::core::ffi::c_char,
            buffer_size: usize,
        ) -> *const ::core::ffi::c_char;
        fn landing_configuration_default() -> LandingConfiguration;
    }
    const SHUTTLE_SIM_MODEL_ATTITUDE: ::core::ffi::c_int = 3;
    const DEG: f64 = ::core::f64::consts::PI / 180.0;

    fn point(x: f64, y: f64) -> TelPoint {
        let mut p: TelPoint = unsafe { zeroed() };
        p.x = x;
        p.y = y;
        p
    }

    /// The 14:24 KST run reached this inlet before the operator's engine
    /// firings.  Maximum drag modulation must not be selected while the actual
    /// high-speed capture turn already leaves the station beyond glide range.
    unsafe fn recorded_capture_range() {
        let mut cfg: LandingConfiguration = landing_configuration_default();
        cfg.vehicle.maximum_bank_angle = 70.0;
        cfg.guidance.final_approach_distance = 9000.0;
        cfg.guidance.final_glide_slope = 20.0;
        cfg.guidance.final_alignment_speed = 160.0;
        let mut attitude = [0 as ::core::ffi::c_char; 512];
        let mut reason = [0 as ::core::ffi::c_char; 192];
        let files = TerminalModelSourceFiles {
            atmosphere_csv: b"ShuttleSim/reference-model/kerbin_atmosphere_identified.csv\0".as_ptr().cast(),
            aero_csv: b"ShuttleSim/reference-model/sts_aero_live20261010.csv\0".as_ptr().cast(),
            aero_book_csv: ::core::ptr::null(),
            attitude_ini: shuttle_sim_model_path(
                ::core::ptr::null(), SHUTTLE_SIM_MODEL_ATTITUDE, attitude.as_mut_ptr(), attitude.len(),
            ),
        };
        let model: *mut TerminalModel = Box::into_raw(Box::new(zeroed::<TerminalModel>()));
        assert!(terminal_model_capture(model, &files, &cfg, 1, reason.as_mut_ptr(), reason.len()));
        let mut c: TelContext = zeroed();
        c.model = model;
        c.d_station = 9000.0;
        c.v_station = 160.0;
        c.aoa_max = 15.0;
        c.h_station = c.d_station * (20.0 * DEG).tan();
        c.e_station = c.h_station + 160.0 * 160.0 / (2.0 * TEL_G0);
        let mut inlet: TelPoint = zeroed();
        inlet.x = -38498.0;
        inlet.y = 41727.0;
        inlet.h = 25789.0;
        inlet.v = 864.0;
        inlet.g = -6.788758 * DEG;
        inlet.psi = 28.984301 * DEG;
        inlet.m = 24073.0;
        inlet.aoa = 17.83;
        inlet.bank = 37.3;
        let mut route: TaemEnergyLawState = zeroed();
        route.initialized = true;
        route.loop_0 = true;
        route.capture_forecast = true;
        route.leg = TEL_LEG_DOWN as ::core::ffi::c_int;
        route.side = -1;
        route.radius = 4207.0;
        let mut segs = [TelSegment { length: 0.0, radius: 0.0 }; 5];
        let n = tel_segments(&route, &c, &inlet, segs.as_mut_ptr());
        let path = tel_sum(segs.as_ptr(), n);
        let mut predicted = 0.0;
        assert!(tel_solve_k(&route, &c, &inlet, &mut predicted) == 0.0);
        assert!(predicted < path - 20000.0);
        // Once abeam, a wide capture must point back toward the actual station,
        // rather than the always-forward centreline lookahead.
        route.loop_0 = false;
        route.leg = TEL_LEG_OUT as ::core::ffi::c_int;
        let mut abeam = inlet;
        abeam.x = 10000.0;
        abeam.y = 40000.0;
        abeam.v = 160.0;
        abeam.psi = -100.0 * DEG;
        assert!(tel_lateral(&mut route, &c, &abeam) > 0.0);
        assert!(route.leg == TEL_LEG_OUT as ::core::ffi::c_int);
        // 2026-10-14 live MM305 handoff: past the RW09 station abeam at 42 km
        // crossrange, Mach 2.85, flying away from the runway line. The
        // straight-in path must charge the turn back, not price the chord as |y|.
        let mut straight: TaemEnergyLawState = zeroed();
        straight.initialized = true;
        straight.leg = TEL_LEG_OUT as ::core::ffi::c_int;
        straight.loop_0 = false;
        straight.x_turn = -9000.0;
        straight.radius = 5000.0;
        let mut passed: TelPoint = zeroed();
        passed.x = 9014.0;
        passed.y = 37660.0;
        passed.h = 21043.0;
        passed.v = 592.0;
        passed.g = -13.8 * DEG;
        passed.psi = -19.5 * DEG;
        passed.m = 25670.0;
        passed.aoa = 14.8;
        passed.bank = 44.0;
        let n = tel_segments(&straight, &c, &passed, segs.as_mut_ptr());
        let passed_path = tel_sum(segs.as_ptr(), n);
        assert!(passed_path > (passed.x + c.d_station).hypot(passed.y) + 20000.0);
        (*model).guidance.sts_trajectory_delivery = true;
        let id = b"STS\0";
        ::core::ptr::copy_nonoverlapping(id.as_ptr().cast(), (*model).vehicle.model_id.as_mut_ptr(), id.len());
        tel_plan(&mut straight, &c, &passed);
        assert!(straight.loop_0); // price and fly a capture route, not a pursuit/Dubins mix
        let mut final_line: TaemEnergyLawState = zeroed();
        final_line.initialized = true;
        final_line.loop_0 = true;
        final_line.capture_forecast = true;
        final_line.leg = TEL_LEG_FINAL as ::core::ffi::c_int;
        final_line.radius = 2200.0;
        let mut low: TelPoint = zeroed();
        low.x = -9500.0;
        low.y = 0.0;
        low.h = 1680.0;
        low.v = 91.5;
        low.g = -19.3 * DEG;
        low.m = 25660.0;
        low.aoa = 4.8;
        let n = tel_segments(&final_line, &c, &low, segs.as_mut_ptr());
        let final_path = tel_sum(segs.as_ptr(), n);
        assert!(tel_predict_pattern(&final_line, &c, &low, 0.0, ::core::ptr::null_mut()) < final_path);
        c.slope_deg = 20.0;
        low.x = -5000.0;
        let priced = tel_effective_station(&final_line, &c, &low, 0.3, low.v);
        assert!(priced.d_station == 5000.0); // loop exits price their actual capture too
        // Live runway switch: near the centreline but pointing across it. The
        // frame scorer priced a HAC, while the old first-tick replan gate left
        // live guidance on an unplanned direct pursuit with the opposite bank.
        let mut switched: TaemGeometryState = zeroed();
        switched.runway_along_m = 33767.0;
        switched.runway_cross_m = -5006.0;
        switched.altitude_above_runway_m = 23459.0;
        switched.airspeed_mps = 768.7;
        switched.ground_speed_mps = 754.6;
        switched.flight_path_angle_deg = -11.0;
        switched.heading_error_deg = 105.8;
        let mut fresh: TaemEnergyLawState = zeroed();
        let (mut aoa, mut bank) = (0.0, 0.0);
        assert!(taem_energy_law_step(&mut fresh, model, &switched, 25660.0, 13.4, 26015.1, 44.8, &mut aoa, &mut bank));
        assert!(fresh.capture_forecast && fresh.loop_0);
        let mut slow_line: TelPoint = zeroed();
        slow_line.x = -7500.0;
        slow_line.y = 0.0;
        slow_line.h = 2800.0;
        slow_line.v = 108.0;
        slow_line.g = -5.0 * DEG;
        slow_line.m = 25660.0;
        slow_line.aoa = 5.5;
        c.final_line = false;
        let range_aoa = tel_vertical(&c, &slow_line, 0.0, 0.0, -1500.0, true);
        c.final_line = true;
        assert!(tel_vertical(&c, &slow_line, 0.0, 0.0, -1500.0, true) < range_aoa);
        let _ = taem_energy_law_frame_score;
        drop(Box::from_raw(model));
    }

    #[test]
    fn mm305_off_axis_and_final_mode_geometry() {
        unsafe {
            let mut c: TelContext = zeroed();
            c.d_station = 8000.0;
            let mut law: TaemEnergyLawState = zeroed();
            law.initialized = true;
            law.leg = TEL_LEG_OUT as ::core::ffi::c_int;
            law.loop_0 = false;
            law.x_turn = -8000.0;
            law.radius = 5000.0;
            let mut polar = point(973.0, 67957.0);
            polar.psi = 95.0 * DEG;
            assert!(tel_plan_open(&law, &c, &polar));
            assert!(!tel_final_line_ready(&c, &polar));
            polar.x = -1552.0;
            polar.y = 38725.0;
            polar.psi = 94.0 * DEG;
            assert!(tel_plan_open(&law, &c, &polar));
            assert!(!tel_final_line_ready(&c, &polar));
            // Even an in-range abeam approach does not shortcut the bounded turn.
            polar.x = -10000.0;
            polar.y = 3000.0;
            polar.psi = 5.0 * DEG;
            assert!(!tel_final_line_ready(&c, &polar));
            polar.y = 1000.0;
            polar.psi = 30.0 * DEG;
            assert!(!tel_final_line_ready(&c, &polar));
            polar.psi = 5.0 * DEG;
            assert!(tel_final_line_ready(&c, &polar));
            polar.x = 973.0;
            assert!(tel_final_line_ready(&c, &polar));
            polar.y = f64::NAN;
            assert!(!tel_final_line_ready(&c, &polar));
            // The test binary runs from the crate directory; the model files are
            // resolved relative to the repository root, as the C test was.
            ::std::env::set_current_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).unwrap();
            recorded_capture_range();
        }
    }
}
