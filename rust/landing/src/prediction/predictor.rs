use crate::sys::*;
extern "C" {
    fn taem_exec_owns_vehicle(exec: *const TaemExecutive) -> bool;
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn terminal_model_policy_matches(
        model: *const TerminalModel,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        guidance: *const GuidanceSettings,
    ) -> bool;
    fn terminal_model_planet_matches(
        model: *const TerminalModel,
        planet: *const PlanetModel,
    ) -> bool;
    fn aero_force_per_q(
        a: *const AeroTable,
        q: ::core::ffi::c_double,
        mach: ::core::ffi::c_double,
        alpha_deg: ::core::ffi::c_double,
        lift_per_q: *mut ::core::ffi::c_double,
        drag_per_q: *mut ::core::ffi::c_double,
    );
    fn decision_axis_capture_time(
        error_deg: ::core::ffi::c_double,
        rate_deg_s: ::core::ffi::c_double,
        acceleration_limit_deg_s2: ::core::ffi::c_double,
        rate_limit_deg_s: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn decision_taem_speed_envelope(
        vehicle: *const VehicleProfile,
        planet: *const PlanetModel,
        altitude_m: ::core::ffi::c_double,
    ) -> TaemSpeedEnvelope;
    fn vessel_physics_aero_config(
        m: *const VesselPhysicsModel,
        q: ::core::ffi::c_double,
        mach: ::core::ffi::c_double,
        aoa: ::core::ffi::c_double,
        beta: ::core::ffi::c_double,
        mass: ::core::ffi::c_double,
        gear: bool,
        brakes: bool,
        airbrakes: ::core::ffi::c_int,
        specific: *mut Vector3,
        confidence: *mut ::core::ffi::c_double,
    ) -> bool;
    fn vessel_physics_gravity(position: Vector3, p: *const PlanetModel) -> Vector3;
    fn vessel_physics_force_best_estimate_config(
        m: *const VesselPhysicsModel,
        q: ::core::ffi::c_double,
        mach: ::core::ffi::c_double,
        aoa: ::core::ffi::c_double,
        beta: ::core::ffi::c_double,
        mass: ::core::ffi::c_double,
        gear: bool,
        brakes: bool,
        airbrakes: ::core::ffi::c_int,
        fallback: AerodynamicModel,
        cal: *const TrajectoryCalibrationModel,
        v: *const VehicleProfile,
        confidence: *mut ::core::ffi::c_double,
        observed: *mut bool,
        relative_uncertainty: *mut ::core::ffi::c_double,
    ) -> Vector3;
    fn vessel_physics_acceleration(
        position: Vector3,
        air_velocity: Vector3,
        p: *const PlanetModel,
        specific_force: Vector3,
        bank: ::core::ffi::c_double,
    ) -> Vector3;
    fn vessel_physics_environment(
        p: *const PlanetModel,
        cal: *const TrajectoryCalibrationModel,
        altitude: ::core::ffi::c_double,
        density: *mut ::core::ffi::c_double,
        sound: *mut ::core::ffi::c_double,
    );
    fn vessel_physics_conservative_stall_fraction(
        true_air_speed: ::core::ffi::c_double,
        angle_of_attack: ::core::ffi::c_double,
        dynamic_pressure: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn vessel_physics_burn_mass(
        m: *const VesselPhysicsModel,
        mass: ::core::ffi::c_double,
        thrust: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn vessel_physics_axis_step(
        m: *const VesselPhysicsModel,
        axis: ::core::ffi::c_int,
        target: ::core::ffi::c_double,
        q: ::core::ffi::c_double,
        rate_limit: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
        angle: *mut ::core::ffi::c_double,
        rate: *mut ::core::ffi::c_double,
    );
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_guidance_start_altitude(
        planet: *const PlanetModel,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
    fn entry_s_turn_bank_authority_available(
        dynamic_pressure: ::core::ffi::c_double,
        true_air_speed: ::core::ffi::c_double,
        stall_fraction: ::core::ffi::c_double,
        g_force: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> bool;
    fn entry_bank_authority_limit(
        true_air_speed: ::core::ffi::c_double,
        dynamic_pressure: ::core::ffi::c_double,
        g_force: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
        maximum_bank: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn entry_taem_range_target(
        planet: *const PlanetModel,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
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
    fn entry_altitude_target_for_range(
        reference_altitude: ::core::ffi::c_double,
        reference_range: ::core::ffi::c_double,
        range: ::core::ffi::c_double,
        target_altitude: ::core::ffi::c_double,
        target_range: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
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
    fn vrotate(a: Vector3, axis: Vector3, radians: ::core::ffi::c_double) -> Vector3;
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
    fn local_point(
        origin: GeoPoint,
        east: ::core::ffi::c_double,
        north: ::core::ffi::c_double,
        radius: ::core::ffi::c_double,
        altitude: ::core::ffi::c_double,
    ) -> GeoPoint;
    fn surface_course(
        pos: Vector3,
        vel: Vector3,
        omega: Vector3,
        north: Vector3,
        fallback: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn runway_coordinates(
        point: GeoPoint,
        site: GeoPoint,
        heading: ::core::ffi::c_double,
        radius: ::core::ffi::c_double,
        along: *mut ::core::ffi::c_double,
        cross: *mut ::core::ffi::c_double,
    );
    fn taem_interface_line_heading(
        runway_along: ::core::ffi::c_double,
        runway_cross: ::core::ffi::c_double,
        runway_heading: ::core::ffi::c_double,
        target: *const TaemInterfaceTarget,
    ) -> ::core::ffi::c_double;
    fn taem_course_rate_bank_command(
        horizontal_speed: ::core::ffi::c_double,
        lift_accel: ::core::ffi::c_double,
        bank_effectiveness: ::core::ffi::c_double,
        bank_limit_0: ::core::ffi::c_double,
        current_course: ::core::ffi::c_double,
        desired_course: ::core::ffi::c_double,
        response_time: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn runway_reciprocal_site(
        primary: *const LandingSite,
        radius: ::core::ffi::c_double,
    ) -> LandingSite;
    fn jerk_angle_update(
        j: *mut JerkLimiter,
        target: ::core::ffi::c_double,
        max_rate: ::core::ffi::c_double,
        max_accel: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn burn_fraction(
        elapsed: ::core::ffi::c_double,
        remaining: ::core::ffi::c_double,
        max_accel: ::core::ffi::c_double,
        ramp: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn flight_control_regime(
        mach: ::core::ffi::c_double,
        true_air_speed: ::core::ffi::c_double,
        dynamic_pressure: ::core::ffi::c_double,
        minimum_safe_speed: ::core::ffi::c_double,
    ) -> FlightControlRegime;
    fn entry_s_turn_effective_minimum_leg(
        true_air_speed: ::core::ffi::c_double,
        taem_speed: ::core::ffi::c_double,
        settings: *const GuidanceSettings,
    ) -> ::core::ffi::c_double;
    fn speedbrake_controller_reset(c: *mut SpeedbrakeController, deployed: bool);
    fn speedbrake_controller_update(
        c: *mut SpeedbrakeController,
        dynamic_pressure: ::core::ffi::c_double,
        target_dynamic_pressure: ::core::ffi::c_double,
        specific_energy_error: ::core::ffi::c_double,
        energy_scale: ::core::ffi::c_double,
        maximum_dynamic_pressure: ::core::ffi::c_double,
        inhibit: bool,
        dt: ::core::ffi::c_double,
    ) -> bool;
    fn landing_configuration_default() -> LandingConfiguration;
    fn trajectory_init(t: *mut Trajectory);
    fn trajectory_clear(t: *mut Trajectory);
    fn trajectory_append(t: *mut Trajectory, p: TrajectoryPoint) -> bool;
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
    fn planet_surface_gravity(p: *const PlanetModel) -> ::core::ffi::c_double;
    fn planet_atmospheric_density(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_pressure(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_speed_of_sound(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn guidance_set_entry_predictor_models(
        g: *mut GuidanceMachine,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
    );
    fn guidance_initialize_reentry_continuation(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        planet: *const PlanetModel,
        cfg: *const LandingConfiguration,
        initial_s_turn_sign: ::core::ffi::c_double,
        late_terminal_test: bool,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
    );
    fn guidance_update_with_terminal_model(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        plan: *const DeorbitPlan,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        terminal_model: *const TerminalModel,
    ) -> GuidanceResult;
    fn guidance_result_clear(r: *mut GuidanceResult);
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
pub struct FlightControlRegime {
    pub command_rate_scale: ::core::ffi::c_double,
    pub command_accel_scale: ::core::ffi::c_double,
    pub response_scale: ::core::ffi::c_double,
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
pub struct EntryPrediction {
    pub trajectory: Trajectory,
    pub taem_distance: ::core::ffi::c_double,
    pub taem_range_error: ::core::ffi::c_double,
    pub closest_distance: ::core::ffi::c_double,
    pub entered_atmosphere: bool,
    pub reached_taem: bool,
    pub taem_ownership_boundary_missed: bool,
    pub taem_dynamic_interface_captured: bool,
    pub taem_dynamic_interface_target_valid: bool,
    pub entry_range: ::core::ffi::c_double,
    pub entry_flight_path_angle: ::core::ffi::c_double,
    pub entry_speed: ::core::ffi::c_double,
    pub entry_course_error: ::core::ffi::c_double,
    pub taem_speed: ::core::ffi::c_double,
    pub taem_flight_path_angle: ::core::ffi::c_double,
    pub taem_altitude: ::core::ffi::c_double,
    pub taem_desired_altitude: ::core::ffi::c_double,
    pub taem_along_track: ::core::ffi::c_double,
    pub taem_cross_track: ::core::ffi::c_double,
    pub taem_course: ::core::ffi::c_double,
    pub mm304_gate_recorded: bool,
    pub mm304_gate_range: ::core::ffi::c_double,
    pub mm304_gate_along_track: ::core::ffi::c_double,
    pub mm304_gate_cross_track: ::core::ffi::c_double,
    pub mm304_gate_course: ::core::ffi::c_double,
    pub mm304_gate_altitude: ::core::ffi::c_double,
    pub mm304_gate_speed: ::core::ffi::c_double,
    pub mm304_gate_flight_path_angle: ::core::ffi::c_double,
    pub taem_energy_error: ::core::ffi::c_double,
    pub taem_interface_error: ::core::ffi::c_double,
    pub taem_interface_energy_margin: ::core::ffi::c_double,
    pub peak_dynamic_pressure: ::core::ffi::c_double,
    pub peak_g_load: ::core::ffi::c_double,
    pub minimum_entry_speed: ::core::ffi::c_double,
    pub maximum_abs_angle_of_attack: ::core::ffi::c_double,
    pub minimum_bank_control_margin: ::core::ffi::c_double,
    pub minimum_aoa_control_margin: ::core::ffi::c_double,
    pub control_horizon_recorded: bool,
    pub control_horizon_speed: ::core::ffi::c_double,
    pub control_horizon_altitude: ::core::ffi::c_double,
    pub control_horizon_range: ::core::ffi::c_double,
    pub control_horizon_specific_energy: ::core::ffi::c_double,
    pub control_horizon_alignment_distance: ::core::ffi::c_double,
    pub has_first_s_turn_reversal: bool,
    pub first_s_turn_reversal_is_final: bool,
    pub first_s_turn_reversal_ut: ::core::ffi::c_double,
    pub first_s_turn_reversal_range: ::core::ffi::c_double,
    pub first_s_turn_reversal_sign: ::core::ffi::c_double,
    pub s_turn_reversals: ::core::ffi::c_uint,
    pub s_turn_max_bank: ::core::ffi::c_double,
    pub final_state: VehicleState,
    pub physics_observed_seconds: ::core::ffi::c_double,
    pub physics_fallback_seconds: ::core::ffi::c_double,
    pub shadow_guidance_used: bool,
    pub shadow_terminal_prediction_valid: bool,
    pub shadow_terminal_policy_feasible: bool,
    pub shadow_terminal_path_committed: bool,
    pub shadow_final_approach_captured: bool,
    pub shadow_aborted: bool,
    pub terminal_survivability_stress_score: ::core::ffi::c_double,
    pub physics_relative_uncertainty: ::core::ffi::c_double,
    pub uncertainty_scenarios: ::core::ffi::c_uint,
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
pub struct AtmosState {
    pub density: ::core::ffi::c_double,
    pub air_velocity: Vector3,
    pub speed: ::core::ffi::c_double,
    pub speed_of_sound: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub q: ::core::ffi::c_double,
    pub drag_accel: ::core::ffi::c_double,
    pub lift_accel: ::core::ffi::c_double,
    pub non_gravity: ::core::ffi::c_double,
    pub specific_force: Vector3,
    pub physics_confidence: ::core::ffi::c_double,
    pub physics_uncertainty: ::core::ffi::c_double,
    pub direct_aero: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroContext {
    pub p: *const PlanetModel,
    pub e: *const AerodynamicEnvelope,
    pub c: *const TrajectoryCalibrationModel,
    pub v: *const VehicleProfile,
    pub entry_table: *const TerminalModel,
    pub bank: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub ut: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PredictorGuidance {
    pub sign: ::core::ffi::c_double,
    pub leg_elapsed: ::core::ffi::c_double,
    pub established: bool,
    pub entry_loaded: bool,
    pub configured_leg_dwell_satisfied: bool,
    pub reversals: ::core::ffi::c_uint,
    pub limiter: JerkLimiter,
    pub has_first_reversal: bool,
    pub first_reversal_final: bool,
    pub final_heading_lock: bool,
    pub first_reversal_ut: ::core::ffi::c_double,
    pub first_reversal_range: ::core::ffi::c_double,
    pub first_reversal_sign: ::core::ffi::c_double,
    pub replay_event: bool,
    pub replay_event_final: bool,
    pub replay_event_ut: ::core::ffi::c_double,
    pub replay_event_sign: ::core::ffi::c_double,
    pub entry_score: ::core::ffi::c_double,
    pub has_entry_score: bool,
    pub actual_bank: ::core::ffi::c_double,
    pub actual_bank_rate: ::core::ffi::c_double,
    pub actual_aoa: ::core::ffi::c_double,
    pub actual_aoa_rate: ::core::ffi::c_double,
    pub entry_speed: ::core::ffi::c_double,
    pub has_interface_target: bool,
    pub enforce_terminal_delivery_budget: bool,
    pub interface_target: TaemInterfaceTarget,
    pub force_control: bool,
    pub forced_until_ut: ::core::ffi::c_double,
    pub control_horizon_ut: ::core::ffi::c_double,
    pub forced_bank: ::core::ffi::c_double,
    pub forced_aoa: ::core::ffi::c_double,
    pub speedbrake: SpeedbrakeController,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalShadowOutcome {
    pub terminal_prediction_valid: bool,
    pub terminal_policy_feasible: bool,
    pub terminal_path_committed: bool,
    pub final_approach_captured: bool,
    pub aborted: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryWitnessTracePoint {
    pub ut: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub speed: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub fpa: ::core::ffi::c_double,
    pub range: ::core::ffi::c_double,
    pub along: ::core::ffi::c_double,
    pub cross: ::core::ffi::c_double,
    pub course_to_site_error: ::core::ffi::c_double,
    pub commanded_bank: ::core::ffi::c_double,
    pub achieved_bank: ::core::ffi::c_double,
    pub commanded_aoa: ::core::ffi::c_double,
    pub achieved_aoa: ::core::ffi::c_double,
    pub energy_margin: ::core::ffi::c_double,
    pub phase: ::core::ffi::c_int,
    pub mm305_veto: ::core::ffi::c_uint,
    pub mm305_ready: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Candidate {
    pub burn_ut: ::core::ffi::c_double,
    pub dv: ::core::ffi::c_double,
    pub duration: ::core::ffi::c_double,
    pub peri: ::core::ffi::c_double,
    pub requested_periapsis_altitude: ::core::ffi::c_double,
    pub score: ::core::ffi::c_double,
    pub burn_normal_angle: ::core::ffi::c_double,
    pub after: VehicleState,
    pub pred: EntryPrediction,
    pub energy_margin: DeorbitEnergyMargin,
    pub tier: ::core::ffi::c_int,
    pub safety_violation: ::core::ffi::c_double,
    pub energy_deficit: ::core::ffi::c_double,
    pub energy_excess: ::core::ffi::c_double,
    pub entry_fpa_error: ::core::ffi::c_double,
    pub entry_course_error: ::core::ffi::c_double,
    pub entry_groundtrack_crossrange_error: ::core::ffi::c_double,
    pub entry_range_error: ::core::ffi::c_double,
    pub handoff_alignment_utilization: ::core::ffi::c_double,
    pub handoff_range_error: ::core::ffi::c_double,
    pub executable_witness_evaluated: bool,
    pub executable_witness_present: bool,
    pub executable_witness_capture: bool,
    pub executable_witness_handoff_valid: bool,
    pub executable_witness_boundary_missed: bool,
    pub executable_witness_aborted: bool,
    pub executable_witness_taem_error: ::core::ffi::c_double,
    pub executable_witness_range_error: ::core::ffi::c_double,
    pub executable_witness_energy_margin: ::core::ffi::c_double,
    pub executable_witness_alignment_error: ::core::ffi::c_double,
    pub executable_witness_closest_distance: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DeorbitEnergyMargin {
    pub valid: bool,
    pub impact_along_track: ::core::ffi::c_double,
    pub impact_cross_track: ::core::ffi::c_double,
    pub impact_range: ::core::ffi::c_double,
    pub impact_altitude: ::core::ffi::c_double,
    pub impact_speed: ::core::ffi::c_double,
    pub impact_runway_end: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RobustAssessment {
    pub scenarios: ::core::ffi::c_uint,
    pub passed: ::core::ffi::c_uint,
    pub unsafe_0: ::core::ffi::c_uint,
    pub recovery_passed: ::core::ffi::c_uint,
    pub entry_stage_passed: ::core::ffi::c_uint,
    pub pass_fraction: ::core::ffi::c_double,
    pub recovery_pass_fraction: ::core::ffi::c_double,
    pub worst_closest: ::core::ffi::c_double,
    pub worst_taem_error: ::core::ffi::c_double,
    pub worst_alignment_error: ::core::ffi::c_double,
    pub worst_entry_angle: ::core::ffi::c_double,
    pub min_periapsis: ::core::ffi::c_double,
    pub max_dynamic_pressure: ::core::ffi::c_double,
    pub max_g_load: ::core::ffi::c_double,
    pub entry_stage_pass_fraction: ::core::ffi::c_double,
    pub qualified: bool,
    pub recovery_qualified: bool,
    pub entry_stage_qualified: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct BurnStressCase {
    pub timing_offset: ::core::ffi::c_double,
    pub thrust_scale: ::core::ffi::c_double,
    pub dv_offset: ::core::ffi::c_double,
    pub mass_scale: ::core::ffi::c_double,
    pub position_offset: ::core::ffi::c_double,
    pub velocity_offset: ::core::ffi::c_double,
    pub radial_error: ::core::ffi::c_double,
    pub normal_error: ::core::ffi::c_double,
    pub atmosphere_fraction: ::core::ffi::c_double,
    pub aero_drag_scale: ::core::ffi::c_double,
    pub aero_lift_scale: ::core::ffi::c_double,
}
pub type DeorbitRejectReason = ::core::ffi::c_uint;
pub const DEORBIT_REJECT_COUNT: DeorbitRejectReason = 14;
pub const DEORBIT_REJECT_ENTRY_PREDICTION: DeorbitRejectReason = 13;
pub const DEORBIT_REJECT_POSTBURN_PERIAPSIS: DeorbitRejectReason = 12;
pub const DEORBIT_REJECT_FINITE_BURN: DeorbitRejectReason = 11;
pub const DEORBIT_REJECT_IMPULSIVE_PERIAPSIS: DeorbitRejectReason = 10;
pub const DEORBIT_REJECT_TARGET_FINITE_ROOT: DeorbitRejectReason = 9;
pub const DEORBIT_REJECT_TARGET_FINITE_BRACKET: DeorbitRejectReason = 8;
pub const DEORBIT_REJECT_TARGET_FINITE_SEED: DeorbitRejectReason = 7;
pub const DEORBIT_REJECT_TARGET_IMPULSE_ROOT: DeorbitRejectReason = 6;
pub const DEORBIT_REJECT_TARGET_IMPULSE_BOUND: DeorbitRejectReason = 5;
pub const DEORBIT_REJECT_TARGET_LOW_STATE: DeorbitRejectReason = 4;
pub const DEORBIT_REJECT_TARGET_ORBIT: DeorbitRejectReason = 3;
pub const DEORBIT_REJECT_TARGET_INPUT: DeorbitRejectReason = 2;
pub const DEORBIT_REJECT_START: DeorbitRejectReason = 1;
pub const DEORBIT_REJECT_NONE: DeorbitRejectReason = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DeorbitTimingProbe {
    pub valid: bool,
    pub closing: bool,
    pub burn_start: ::core::ffi::c_double,
    pub burn_ut: ::core::ffi::c_double,
    pub dv: ::core::ffi::c_double,
    pub duration: ::core::ffi::c_double,
    pub peri: ::core::ffi::c_double,
    pub normal_angle: ::core::ffi::c_double,
    pub entry_range: ::core::ffi::c_double,
    pub entry_fpa: ::core::ffi::c_double,
    pub entry_speed: ::core::ffi::c_double,
    pub entry_course_error: ::core::ffi::c_double,
    pub entry_crossrange: ::core::ffi::c_double,
    pub entry_miss: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
unsafe extern "C" fn aero_at(
    mut e: *const AerodynamicEnvelope,
    mut mach: ::core::ffi::c_double,
) -> AerodynamicModel {
    let mut a: [::core::ffi::c_double; 4] = [
        0.35f64,
        1.05f64,
        2.6f64,
        6 as ::core::ffi::c_int as ::core::ffi::c_double,
    ];
    let mut x: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        mach,
    );
    if x <= a[0 as ::core::ffi::c_int as usize] {
        return (*e).regimes[0 as ::core::ffi::c_int as usize];
    }
    if x >= a[3 as ::core::ffi::c_int as usize] {
        return (*e).regimes[3 as ::core::ffi::c_int as usize];
    }
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if x <= a[(i + 1 as ::core::ffi::c_int) as usize] {
            let mut f: ::core::ffi::c_double = (x - a[i as usize])
                / (a[(i + 1 as ::core::ffi::c_int) as usize] - a[i as usize]);
            let mut r: AerodynamicModel = AerodynamicModel {
                lift_to_drag: (*e).regimes[i as usize].lift_to_drag
                    + ((*e).regimes[(i + 1 as ::core::ffi::c_int) as usize].lift_to_drag
                        - (*e).regimes[i as usize].lift_to_drag) * f,
                ballistic_coefficient: (*e).regimes[i as usize].ballistic_coefficient
                    + ((*e)
                        .regimes[(i + 1 as ::core::ffi::c_int) as usize]
                        .ballistic_coefficient
                        - (*e).regimes[i as usize].ballistic_coefficient) * f,
                confidence: (*e).regimes[i as usize].confidence
                    + ((*e).regimes[(i + 1 as ::core::ffi::c_int) as usize].confidence
                        - (*e).regimes[i as usize].confidence) * f,
            };
            return r;
        }
        i += 1;
    }
    return (*e).regimes[3 as ::core::ffi::c_int as usize];
}
unsafe extern "C" fn scheduled_aoa(
    mut altitude: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
) -> ::core::ffi::c_double {
    let mut lower: ::core::ffi::c_double = fmax(
        5 as ::core::ffi::c_int as ::core::ffi::c_double,
        fmin(
            (*v).entry_angle_of_attack,
            10 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut f: ::core::ffi::c_double = clampd(
        altitude
            / fmax(
                (*p).atmosphere_depth,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    return lower + ((*v).entry_angle_of_attack - lower) * f;
}
unsafe extern "C" fn gravity(mut pos: Vector3, mut p: *const PlanetModel) -> Vector3 {
    return vessel_physics_gravity(pos, p);
}
unsafe extern "C" fn context_aoa(
    mut ctx: *const AeroContext,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if (*ctx).aoa.is_finite() as i32 != 0 {
        (*ctx).aoa
    } else {
        scheduled_aoa(altitude, (*ctx).p, (*ctx).v)
    };
}
unsafe extern "C" fn atmosphere(
    mut pos: Vector3,
    mut vel: Vector3,
    mut ctx: *const AeroContext,
) -> AtmosState {
    let mut alt: ::core::ffi::c_double = vmag(pos) - (*(*ctx).p).radius;
    let mut d: ::core::ffi::c_double = 0.;
    let mut sos: ::core::ffi::c_double = 0.;
    vessel_physics_environment((*ctx).p, (*ctx).c, alt, &raw mut d, &raw mut sos);
    let mut air: Vector3 = vsub(vel, vcross(planet_rotation_vector((*ctx).p), pos));
    let mut sp: ::core::ffi::c_double = vmag(air);
    let mut mach: ::core::ffi::c_double = sp
        / fmax(sos, 1 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut q: ::core::ffi::c_double = 0.5f64
        * fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, d) * sp * sp;
    let mut s: AtmosState = AtmosState {
        density: d,
        air_velocity: air,
        speed: sp,
        speed_of_sound: sos,
        mach: mach,
        q: q,
        drag_accel: 0.,
        lift_accel: 0.,
        non_gravity: 0.,
        specific_force: Vector3 { x: 0., y: 0., z: 0. },
        physics_confidence: 0.,
        physics_uncertainty: 0.,
        direct_aero: false,
    };
    s.specific_force = vessel_physics_force_best_estimate_config(
        (*(*ctx).c).physics,
        q,
        mach,
        context_aoa(ctx, alt),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        (*ctx).mass,
        (*ctx).gear,
        (*ctx).brakes,
        (*ctx).airbrakes,
        aero_at((*ctx).e, mach),
        (*ctx).c,
        (*ctx).v,
        &raw mut s.physics_confidence,
        &raw mut s.direct_aero,
        &raw mut s.physics_uncertainty,
    );
    if (*(*ctx).c).forecast_drag_scale > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        s.specific_force.x *= (*(*ctx).c).forecast_drag_scale;
    }
    if (*(*ctx).c).forecast_lift_scale > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        s.specific_force.y *= (*(*ctx).c).forecast_lift_scale;
    }
    if !(*ctx).entry_table.is_null() {
        let mut lift_per_q: ::core::ffi::c_double = 0.;
        let mut drag_per_q: ::core::ffi::c_double = 0.;
        aero_force_per_q(
            &raw const (*(*ctx).entry_table).aero,
            q,
            mach,
            context_aoa(ctx, alt),
            &raw mut lift_per_q,
            &raw mut drag_per_q,
        );
        let mut lift_stress: ::core::ffi::c_double = if (*(*ctx).c).stress_lift_scale
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            (*(*ctx).c).stress_lift_scale
        } else {
            1.0f64
        };
        let mut drag_stress: ::core::ffi::c_double = if (*(*ctx).c).stress_drag_scale
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            (*(*ctx).c).stress_drag_scale
        } else {
            1.0f64
        };
        s.specific_force = v3(
            q * drag_per_q * drag_stress / fmax((*ctx).mass, 1.0f64),
            q * lift_per_q * lift_stress / fmax((*ctx).mass, 1.0f64),
            0.0f64,
        );
    }
    s.drag_accel = s.specific_force.x;
    s.lift_accel = s.specific_force.y;
    s.non_gravity = vmag(s.specific_force);
    return s;
}
unsafe extern "C" fn total_accel(
    mut pos: Vector3,
    mut vel: Vector3,
    mut ctx: *const AeroContext,
) -> Vector3 {
    let mut s: AtmosState = atmosphere(pos, vel, ctx);
    let mut effectiveness: ::core::ffi::c_double = if s.direct_aero as ::core::ffi::c_int
        != 0
    {
        1 as ::core::ffi::c_int as ::core::ffi::c_double
    } else {
        clampd((*(*ctx).c).bank_effectiveness, 0.55f64, 1.35f64)
    };
    return vessel_physics_acceleration(
        pos,
        s.air_velocity,
        (*ctx).p,
        s.specific_force,
        (*ctx).bank * effectiveness,
    );
}
unsafe extern "C" fn rk4_gravity(
    mut s: VehicleState,
    mut dt: ::core::ffi::c_double,
    mut p: *const PlanetModel,
) -> VehicleState {
    let mut p1: Vector3 = s.velocity;
    let mut v1: Vector3 = gravity(s.position, p);
    let mut p2: Vector3 = vadd(s.position, vscale(p1, dt * 0.5f64));
    let mut u2: Vector3 = vadd(s.velocity, vscale(v1, dt * 0.5f64));
    let mut v2: Vector3 = gravity(p2, p);
    let mut p3: Vector3 = vadd(s.position, vscale(u2, dt * 0.5f64));
    let mut u3: Vector3 = vadd(s.velocity, vscale(v2, dt * 0.5f64));
    let mut v3a: Vector3 = gravity(p3, p);
    let mut p4: Vector3 = vadd(s.position, vscale(u3, dt));
    let mut u4: Vector3 = vadd(s.velocity, vscale(v3a, dt));
    let mut v4: Vector3 = gravity(p4, p);
    s.position = vadd(
        s.position,
        vscale(
            vadd(
                vadd(p1, vscale(u2, 2 as ::core::ffi::c_int as ::core::ffi::c_double)),
                vadd(vscale(u3, 2 as ::core::ffi::c_int as ::core::ffi::c_double), u4),
            ),
            dt / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    s.velocity = vadd(
        s.velocity,
        vscale(
            vadd(
                vadd(v1, vscale(v2, 2 as ::core::ffi::c_int as ::core::ffi::c_double)),
                vadd(vscale(v3a, 2 as ::core::ffi::c_int as ::core::ffi::c_double), v4),
            ),
            dt / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    s.ut += dt;
    return s;
}
unsafe extern "C" fn rk4_aero(
    mut s: VehicleState,
    mut dt: ::core::ffi::c_double,
    mut ctx: *mut AeroContext,
) -> VehicleState {
    (*ctx).ut = s.ut;
    (*ctx).mass = s.mass;
    let mut k1p: Vector3 = s.velocity;
    let mut k1v: Vector3 = total_accel(s.position, s.velocity, ctx);
    let mut p2: Vector3 = vadd(s.position, vscale(k1p, dt * 0.5f64));
    let mut u2: Vector3 = vadd(s.velocity, vscale(k1v, dt * 0.5f64));
    let mut k2p: Vector3 = u2;
    let mut k2v: Vector3 = total_accel(p2, u2, ctx);
    let mut p3: Vector3 = vadd(s.position, vscale(k2p, dt * 0.5f64));
    let mut u3: Vector3 = vadd(s.velocity, vscale(k2v, dt * 0.5f64));
    let mut k3p: Vector3 = u3;
    let mut k3v: Vector3 = total_accel(p3, u3, ctx);
    let mut p4: Vector3 = vadd(s.position, vscale(k3p, dt));
    let mut u4: Vector3 = vadd(s.velocity, vscale(k3v, dt));
    let mut k4p: Vector3 = u4;
    let mut k4v: Vector3 = total_accel(p4, u4, ctx);
    s.position = vadd(
        s.position,
        vscale(
            vadd(
                vadd(k1p, vscale(k2p, 2 as ::core::ffi::c_int as ::core::ffi::c_double)),
                vadd(vscale(k3p, 2 as ::core::ffi::c_int as ::core::ffi::c_double), k4p),
            ),
            dt / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    s.velocity = vadd(
        s.velocity,
        vscale(
            vadd(
                vadd(k1v, vscale(k2v, 2 as ::core::ffi::c_int as ::core::ffi::c_double)),
                vadd(vscale(k3v, 2 as ::core::ffi::c_int as ::core::ffi::c_double), k4v),
            ),
            dt / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    s.ut += dt;
    return s;
}
unsafe extern "C" fn body_basis(
    mut p: *const PlanetModel,
    mut ut: ::core::ffi::c_double,
    mut prime: *mut Vector3,
    mut east: *mut Vector3,
    mut north: *mut Vector3,
) {
    *north = vnorm(
        (*p).north_axis,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    *prime = vnorm(
        vproject_plane(
            vrotate(
                (*p).prime_meridian_at_epoch,
                *north,
                (*p).rotational_speed * (ut - (*p).epoch_ut),
            ),
            *north,
        ),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    *east = vnorm(
        vcross(*north, *prime),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
}
#[no_mangle]
pub unsafe extern "C" fn predictor_geo_point(
    mut pos: Vector3,
    mut p: *const PlanetModel,
    mut ut: ::core::ffi::c_double,
) -> GeoPoint {
    let mut prime: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut east: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut north: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    body_basis(p, ut, &raw mut prime, &raw mut east, &raw mut north);
    let mut up: Vector3 = vnorm(pos, prime);
    let mut sl: ::core::ffi::c_double = clampd(
        vdot(up, north),
        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut lat: ::core::ffi::c_double = asin(sl);
    let mut eq: Vector3 = vnorm(vsub(up, vscale(north, sl)), prime);
    let mut g: GeoPoint = GeoPoint {
        latitude: lat * RAD2DEG,
        longitude: norm_signed_deg(atan2(vdot(eq, east), vdot(eq, prime)) * RAD2DEG),
        altitude: vmag(pos) - (*p).radius,
    };
    return g;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_inertial_position(
    mut g: GeoPoint,
    mut p: *const PlanetModel,
    mut ut: ::core::ffi::c_double,
) -> Vector3 {
    let mut prime: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut east: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut north: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    body_basis(p, ut, &raw mut prime, &raw mut east, &raw mut north);
    let mut lat: ::core::ffi::c_double = g.latitude * DEG2RAD;
    let mut lon: ::core::ffi::c_double = g.longitude * DEG2RAD;
    let mut h: ::core::ffi::c_double = cos(lat);
    return vscale(
        vadd(
            vadd(vscale(prime, h * cos(lon)), vscale(east, h * sin(lon))),
            vscale(north, sin(lat)),
        ),
        (*p).radius + g.altitude,
    );
}
#[no_mangle]
pub unsafe extern "C" fn predictor_propagate_vacuum(
    mut s: VehicleState,
    mut target: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut step: ::core::ffi::c_double,
) -> VehicleState {
    if p.is_null() || target.is_finite() as i32 == 0 || s.ut.is_finite() as i32 == 0
        || target <= s.ut
    {
        return s;
    }
    step = clampd(step, 0.01f64, 120 as ::core::ffi::c_int as ::core::ffi::c_double);
    while s.ut < target {
        let mut dt: ::core::ffi::c_double = fmin(step, target - s.ut);
        s = rk4_gravity(s, dt, p);
    }
    return s;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_postburn_periapsis(
    mut s: VehicleState,
    mut p: *const PlanetModel,
) -> ::core::ffi::c_double {
    if p.is_null() || s.mass.is_finite() as i32 == 0
        || s.position.x.is_finite() as i32 == 0 || s.position.y.is_finite() as i32 == 0
        || s.position.z.is_finite() as i32 == 0 || s.velocity.x.is_finite() as i32 == 0
        || s.velocity.y.is_finite() as i32 == 0 || s.velocity.z.is_finite() as i32 == 0
    {
        return -DBL_MAX;
    }
    let mut r: ::core::ffi::c_double = vmag(s.position);
    if r.is_finite() as i32 == 0 || r <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*p).gravitational_parameter.is_finite() as i32 == 0
        || (*p).gravitational_parameter
            <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return -DBL_MAX;
    }
    let mut h: Vector3 = vcross(s.position, s.velocity);
    let mut h2: ::core::ffi::c_double = vdot(h, h);
    let mut e: ::core::ffi::c_double = 0.5f64 * vdot(s.velocity, s.velocity)
        - (*p).gravitational_parameter / r;
    if h2.is_finite() as i32 == 0 || e.is_finite() as i32 == 0 {
        return -DBL_MAX;
    }
    let mut ecc: ::core::ffi::c_double = sqrt(
        fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double
                + 2 as ::core::ffi::c_int as ::core::ffi::c_double * e * h2
                    / ((*p).gravitational_parameter * (*p).gravitational_parameter),
        ),
    );
    let mut pr: ::core::ffi::c_double = h2
        / fmax(
            (*p).gravitational_parameter
                * (1 as ::core::ffi::c_int as ::core::ffi::c_double + ecc),
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    return if pr.is_finite() as i32 != 0 { pr - (*p).radius } else { -DBL_MAX };
}
#[no_mangle]
pub unsafe extern "C" fn predictor_directed_taem_range_error(
    mut range: ::core::ffi::c_double,
    mut closest_distance: ::core::ffi::c_double,
    mut target_range: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if range.is_finite() as i32 == 0 || closest_distance.is_finite() as i32 == 0
        || target_range.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    range = fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, range);
    target_range = fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, target_range);
    closest_distance = clampd(
        closest_distance,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        range,
    );
    if closest_distance < target_range {
        let mut passed_distance: ::core::ffi::c_double = range + target_range
            - 2.0f64 * closest_distance;
        return -fmax(fabs(range - target_range), passed_distance);
    }
    return range - target_range;
}
unsafe extern "C" fn burn_direction(
    mut pos: Vector3,
    mut vel: Vector3,
    mut radial_error_deg: ::core::ffi::c_double,
    mut normal_error_deg: ::core::ffi::c_double,
) -> Vector3 {
    let mut retro: Vector3 = vnorm(
        vscale(vel, -(1 as ::core::ffi::c_int) as ::core::ffi::c_double),
        vnorm(
            vscale(pos, -(1 as ::core::ffi::c_int) as ::core::ffi::c_double),
            v3(
                -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        ),
    );
    let mut normal: Vector3 = vnorm(
        vcross(pos, vel),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut radial: Vector3 = vnorm(
        vproject_plane(
            vnorm(
                pos,
                v3(
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            ),
            retro,
        ),
        vnorm(
            vcross(normal, retro),
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        ),
    );
    let mut radial_error: ::core::ffi::c_double = tan(radial_error_deg * DEG2RAD);
    let mut normal_error: ::core::ffi::c_double = tan(normal_error_deg * DEG2RAD);
    return vnorm(
        vadd(retro, vadd(vscale(radial, radial_error), vscale(normal, normal_error))),
        retro,
    );
}
unsafe extern "C" fn burn_accel(
    mut pos: Vector3,
    mut vel: Vector3,
    mut p: *const PlanetModel,
    mut thrust_accel: ::core::ffi::c_double,
    mut radial_error_deg: ::core::ffi::c_double,
    mut normal_error_deg: ::core::ffi::c_double,
) -> Vector3 {
    return vadd(
        gravity(pos, p),
        vscale(
            burn_direction(pos, vel, radial_error_deg, normal_error_deg),
            thrust_accel,
        ),
    );
}
unsafe extern "C" fn retro_burn_scenario(
    mut s: VehicleState,
    mut dv: ::core::ffi::c_double,
    mut thrust: ::core::ffi::c_double,
    mut throttle: ::core::ffi::c_double,
    mut ramp: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut radial_error_deg: ::core::ffi::c_double,
    mut normal_error_deg: ::core::ffi::c_double,
    mut duration: *mut ::core::ffi::c_double,
    mut projected_delta_v: *mut ::core::ffi::c_double,
    mut physics: *const VesselPhysicsModel,
) -> VehicleState {
    let mut maxa: ::core::ffi::c_double = thrust
        * clampd(
            throttle,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ) / fmax(s.mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double);
    if maxa.is_finite() as i32 == 0 || dv.is_finite() as i32 == 0
        || ramp.is_finite() as i32 == 0
    {
        maxa = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    ramp = clampd(
        if ramp.is_finite() as i32 != 0 {
            ramp
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        },
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        60 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    if !duration.is_null() {
        *duration = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    if !projected_delta_v.is_null() {
        *projected_delta_v = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    if dv <= 0 as ::core::ffi::c_int as ::core::ffi::c_double || maxa <= 0.01f64 {
        return s;
    }
    let mut delivered: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut elapsed: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut step: ::core::ffi::c_double = 0.25f64;
    while delivered < dv - GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS {
        if !physics.is_null()
            && (*physics).dry_mass > 0 as ::core::ffi::c_int as ::core::ffi::c_double
            && s.mass <= (*physics).dry_mass
        {
            break;
        }
        maxa = thrust
            * clampd(
                throttle,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ) / fmax(s.mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double);
        let mut rem: ::core::ffi::c_double = fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv - delivered,
        );
        let mut f: ::core::ffi::c_double = burn_fraction(elapsed, rem, maxa, ramp);
        if f <= 0.0001f64 {
            elapsed += step;
        } else {
            let mut dt: ::core::ffi::c_double = fmin(
                step,
                fmax(0.02f64, (dv - delivered) / fmax(maxa * f, 0.01f64)),
            );
            let mut ta: ::core::ffi::c_double = maxa * f;
            if dt.is_finite() as i32 == 0 || !(dt > 0.0f64) || ta.is_finite() as i32 == 0
                || !(ta > 0.0f64)
            {
                break;
            }
            let mut previous_delivered: ::core::ffi::c_double = delivered;
            let mut retro: Vector3 = vnorm(
                vscale(s.velocity, -(1 as ::core::ffi::c_int) as ::core::ffi::c_double),
                v3(
                    -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            );
            let mut dir: Vector3 = burn_direction(
                s.position,
                s.velocity,
                radial_error_deg,
                normal_error_deg,
            );
            let mut projection: ::core::ffi::c_double = clampd(
                vdot(dir, retro),
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            let mut p1: Vector3 = s.velocity;
            let mut v1: Vector3 = burn_accel(
                s.position,
                s.velocity,
                p,
                ta,
                radial_error_deg,
                normal_error_deg,
            );
            let mut p2: Vector3 = vadd(s.position, vscale(p1, dt * 0.5f64));
            let mut u2: Vector3 = vadd(s.velocity, vscale(v1, dt * 0.5f64));
            let mut v2: Vector3 = burn_accel(
                p2,
                u2,
                p,
                ta,
                radial_error_deg,
                normal_error_deg,
            );
            let mut p3: Vector3 = vadd(s.position, vscale(u2, dt * 0.5f64));
            let mut u3: Vector3 = vadd(s.velocity, vscale(v2, dt * 0.5f64));
            let mut v3a: Vector3 = burn_accel(
                p3,
                u3,
                p,
                ta,
                radial_error_deg,
                normal_error_deg,
            );
            let mut p4: Vector3 = vadd(s.position, vscale(u3, dt));
            let mut u4: Vector3 = vadd(s.velocity, vscale(v3a, dt));
            let mut v4: Vector3 = burn_accel(
                p4,
                u4,
                p,
                ta,
                radial_error_deg,
                normal_error_deg,
            );
            s.position = vadd(
                s.position,
                vscale(
                    vadd(
                        vadd(
                            p1,
                            vscale(u2, 2 as ::core::ffi::c_int as ::core::ffi::c_double),
                        ),
                        vadd(
                            vscale(u3, 2 as ::core::ffi::c_int as ::core::ffi::c_double),
                            u4,
                        ),
                    ),
                    dt / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            );
            s.velocity = vadd(
                s.velocity,
                vscale(
                    vadd(
                        vadd(
                            v1,
                            vscale(v2, 2 as ::core::ffi::c_int as ::core::ffi::c_double),
                        ),
                        vadd(
                            vscale(
                                v3a,
                                2 as ::core::ffi::c_int as ::core::ffi::c_double,
                            ),
                            v4,
                        ),
                    ),
                    dt / 6 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            );
            s.ut += dt;
            s.mass = vessel_physics_burn_mass(
                physics,
                s.mass,
                thrust * throttle * f,
                dt,
            );
            delivered += ta * projection * dt;
            elapsed += dt;
            if s.ut.is_finite() as i32 == 0 || vmag(s.position).is_finite() as i32 == 0
                || vmag(s.velocity).is_finite() as i32 == 0
            {
                break;
            }
            if !(delivered > previous_delivered) {
                break;
            }
        }
    }
    if !duration.is_null() {
        *duration = elapsed;
    }
    if !projected_delta_v.is_null() {
        *projected_delta_v = delivered;
    }
    return s;
}
unsafe extern "C" fn bank_limit(
    mut a: *const AtmosState,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
    mut max_bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut g_load: ::core::ffi::c_double = (*a).non_gravity
        / fmax(planet_surface_gravity(p), 0.1f64);
    return entry_bank_authority_limit((*a).speed, (*a).q, g_load, v, max_bank);
}
unsafe extern "C" fn turn_radius_for_bank(
    mut a: *const AtmosState,
    mut c: *const TrajectoryCalibrationModel,
    mut bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if a.is_null() || c.is_null()
        || (*a).speed <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*a).lift_accel <= 0.005f64 || fabs(bank) < 0.05f64
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut effective: ::core::ffi::c_double = fabs(bank)
        * clampd((*c).bank_effectiveness, 0.35f64, 1.8f64) * DEG2RAD;
    effective = clampd(
        effective,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        89 as ::core::ffi::c_int as ::core::ffi::c_double * DEG2RAD,
    );
    let mut lateral: ::core::ffi::c_double = (*a).lift_accel * fabs(sin(effective));
    return if lateral > 0.005f64 {
        (*a).speed * (*a).speed / lateral
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn bank_for_turn_radius(
    mut a: *const AtmosState,
    mut c: *const TrajectoryCalibrationModel,
    mut radius: ::core::ffi::c_double,
    mut max_bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if a.is_null() || c.is_null() || radius.is_finite() as i32 == 0
        || radius <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*a).speed <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*a).lift_accel <= 0.005f64
    {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut needed: ::core::ffi::c_double = (*a).speed * (*a).speed / radius;
    let mut ratio: ::core::ffi::c_double = needed / fmax((*a).lift_accel, 0.005f64);
    if ratio <= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut eff: ::core::ffi::c_double = clampd(
        (*c).bank_effectiveness,
        0.35f64,
        1.8f64,
    );
    let mut limit: ::core::ffi::c_double = fabs(max_bank) * eff * DEG2RAD;
    limit = clampd(
        limit,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        89 as ::core::ffi::c_int as ::core::ffi::c_double * DEG2RAD,
    );
    let mut angle: ::core::ffi::c_double = asin(
        clampd(ratio, 0 as ::core::ffi::c_int as ::core::ffi::c_double, sin(limit)),
    ) / eff * RAD2DEG;
    return clampd(
        angle,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        fabs(max_bank),
    );
}
#[no_mangle]
pub unsafe extern "C" fn entry_taem_speed_target(
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
) -> ::core::ffi::c_double {
    if v.is_null() || s.is_null() || p.is_null() {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut envelope: TaemSpeedEnvelope = decision_taem_speed_envelope(
        v,
        p,
        (*s).taem_interface_altitude,
    );
    if !envelope.valid {
        return fmax((*v).minimum_safe_speed, (*v).final_approach_speed);
    }
    let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        (*s).taem_interface_altitude,
    );
    let mut nominal: ::core::ffi::c_double = if sound.is_finite() as i32 != 0
        && sound > DBL_MIN
    {
        sound * (*s).mm305_target_mach
    } else {
        envelope.maximum_speed_mps
    };
    if !envelope.feasible {
        return envelope.minimum_speed_mps;
    }
    return clampd(nominal, envelope.minimum_speed_mps, envelope.maximum_speed_mps);
}
#[no_mangle]
pub unsafe extern "C" fn hac_acquisition_speed_target(
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
) -> ::core::ffi::c_double {
    if v.is_null() || s.is_null() || p.is_null() {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut altitude: ::core::ffi::c_double = fmax(
        (*s).hac_acquisition_altitude,
        (*s).mm305_min_altitude,
    );
    let mut envelope: TaemSpeedEnvelope = decision_taem_speed_envelope(v, p, altitude);
    if !envelope.valid {
        return fmax((*v).minimum_safe_speed, (*v).final_approach_speed);
    }
    let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        altitude,
    );
    let mut lower_mach: ::core::ffi::c_double = fmax(
        0.0f64,
        (*s).mm305_target_mach - (*s).mm305_mach_half_width,
    );
    let mut target_mach: ::core::ffi::c_double = lower_mach / (1.0f64 - 0.01f64);
    target_mach = fmin(target_mach, (*s).mm305_target_mach);
    let mut nominal: ::core::ffi::c_double = if sound.is_finite() as i32 != 0
        && sound > DBL_MIN
    {
        sound * target_mach
    } else {
        envelope.minimum_speed_mps
    };
    if !envelope.feasible {
        return envelope.minimum_speed_mps;
    }
    return clampd(nominal, envelope.minimum_speed_mps, envelope.maximum_speed_mps);
}
unsafe extern "C" fn predictor_speedbrake_state(
    mut g: *mut PredictorGuidance,
    mut geo: GeoPoint,
    mut a: *const AtmosState,
    mut p: *const PlanetModel,
    mut c: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut mass: ::core::ffi::c_double,
    mut reached_taem: bool,
    mut dt: ::core::ffi::c_double,
) -> ::core::ffi::c_int {
    let mut unpowered: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_UNPOWERED_ONLY\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !unpowered.is_null()
        && strcmp(unpowered, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
    {
        speedbrake_controller_reset(&raw mut (*g).speedbrake, false_0 != 0);
        return 0 as ::core::ffi::c_int;
    }
    if !reached_taem {
        speedbrake_controller_reset(&raw mut (*g).speedbrake, false_0 != 0);
        return 0 as ::core::ffi::c_int;
    }
    let mut predicted_gear: bool = geo.altitude - (*site).altitude
        <= (*s).gear_deployment_altitude;
    let mut deployed_force: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
    let mut deployed_confidence: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut speedbrake_model_ready: bool = !c.is_null() && !(*c).physics.is_null()
        && vessel_physics_aero_config(
            (*c).physics,
            (*a).q,
            (*a).mach,
            (*g).actual_aoa,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass,
            predicted_gear,
            false_0 != 0,
            1 as ::core::ffi::c_int,
            &raw mut deployed_force,
            &raw mut deployed_confidence,
        ) as ::core::ffi::c_int != 0 && deployed_confidence >= 0.12f64;
    if !speedbrake_model_ready {
        speedbrake_controller_reset(&raw mut (*g).speedbrake, false_0 != 0);
        return 0 as ::core::ffi::c_int;
    }
    let mut target: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut range: ::core::ffi::c_double = great_circle_distance(
        geo,
        target,
        (*p).radius,
    );
    let mut taem_range: ::core::ffi::c_double = fmax(
        (*s).taem_interface_range,
        (*s).final_approach_distance
            + 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut blend: ::core::ffi::c_double = clampd(
        (range - (*s).final_approach_distance)
            / fmax(
                taem_range - (*s).final_approach_distance,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    blend = sqrt(blend);
    let mut taem_speed: ::core::ffi::c_double = entry_taem_speed_target(v, s, p);
    let mut final_speed: ::core::ffi::c_double = fmax(
        (*v).minimum_safe_speed * 1.08f64,
        (*v).final_approach_speed,
    );
    let mut target_speed: ::core::ffi::c_double = final_speed
        + (taem_speed - final_speed) * blend;
    let mut altitude_blend: ::core::ffi::c_double = clampd(
        range / fmax(taem_range, 1 as ::core::ffi::c_int as ::core::ffi::c_double),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut target_altitude: ::core::ffi::c_double = (*site).altitude
        + ((*s).taem_interface_altitude - (*site).altitude) * altitude_blend;
    let mut density: ::core::ffi::c_double = planet_atmospheric_density(
        p,
        target_altitude,
    ) * clampd((*c).density_scale, 0.35f64, 2.8f64);
    let mut target_q: ::core::ffi::c_double = 0.5f64
        * fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, density) * target_speed
        * target_speed;
    let mut current_energy: ::core::ffi::c_double = rotating_specific_energy(
        geo.latitude,
        geo.altitude,
        (*a).speed,
        p,
    );
    let mut target_energy: ::core::ffi::c_double = rotating_specific_energy(
        geo.latitude,
        target_altitude,
        target_speed,
        p,
    );
    let mut energy_scale: ::core::ffi::c_double = fmax(
        0.5f64 * target_speed * target_speed,
        planet_surface_gravity(p)
            * fmax(
                1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                target_altitude - (*site).altitude,
            ),
    );
    return if speedbrake_controller_update(
        &raw mut (*g).speedbrake,
        (*a).q,
        target_q,
        current_energy - target_energy,
        energy_scale,
        (*v).maximum_dynamic_pressure,
        false_0 != 0,
        dt,
    ) as ::core::ffi::c_int != 0
    {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
}
unsafe extern "C" fn entry_energy_to_taem(
    mut latitude: ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
    mut speed: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    let mut target_speed: ::core::ffi::c_double = entry_taem_speed_target(v, s, p);
    return fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        entry_remaining_specific_energy(
            latitude,
            altitude,
            speed,
            (*site).latitude,
            (*s).taem_interface_altitude,
            target_speed,
            p,
        ),
    );
}
unsafe extern "C" fn entry_required_average_drag(
    mut geo: GeoPoint,
    mut speed: ::core::ffi::c_double,
    mut range: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut interface_target: *const TaemInterfaceTarget,
) -> ::core::ffi::c_double {
    if !interface_target.is_null()
        && (*interface_target).valid as ::core::ffi::c_int != 0
        && (*interface_target).specific_energy.is_finite() as i32 != 0
        && (*interface_target).along_track.is_finite() as i32 != 0
        && (*interface_target).cross_track.is_finite() as i32 != 0
    {
        let mut rh: ::core::ffi::c_double = (*site).runway_heading * DEG2RAD;
        let mut e: ::core::ffi::c_double = (*interface_target).along_track * sin(rh)
            + (*interface_target).cross_track * cos(rh);
        let mut n: ::core::ffi::c_double = (*interface_target).along_track * cos(rh)
            - (*interface_target).cross_track * sin(rh);
        let mut origin: GeoPoint = GeoPoint {
            latitude: (*site).latitude,
            longitude: (*site).longitude,
            altitude: (*site).altitude,
        };
        let mut gate: GeoPoint = local_point(
            origin,
            e,
            n,
            (*p).radius,
            (*interface_target).altitude,
        );
        let mut distance: ::core::ffi::c_double = great_circle_distance(
            geo,
            gate,
            (*p).radius,
        );
        let mut current: ::core::ffi::c_double = rotating_specific_energy(
            geo.latitude,
            geo.altitude,
            speed,
            p,
        );
        if distance.is_finite() as i32 != 0 && current.is_finite() as i32 != 0 {
            return fmax(0.0f64, current - (*interface_target).specific_energy)
                / fmax(5000.0f64, distance);
        }
    }
    let mut reserve: ::core::ffi::c_double = fmax(
        entry_taem_range_target(p, s),
        (*s).hac_radius * 3.0f64,
    );
    let mut usable: ::core::ffi::c_double = fmax(
        5000 as ::core::ffi::c_int as ::core::ffi::c_double,
        range - reserve,
    );
    return entry_energy_to_taem(geo.latitude, geo.altitude, speed, p, v, site, s)
        / usable;
}
unsafe extern "C" fn predictor_change_sign(
    mut g: *mut PredictorGuidance,
    mut sign: ::core::ffi::c_double,
) {
    sign = (if sign >= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        1 as ::core::ffi::c_int
    } else {
        -(1 as ::core::ffi::c_int)
    }) as ::core::ffi::c_double;
    if (*g).sign == sign {
        return;
    }
    if (*g).sign != 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        (*g).reversals = (*g).reversals.wrapping_add(1);
    }
    (*g).sign = sign;
    (*g).leg_elapsed = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*g).established = false_0 != 0;
    (*g).configured_leg_dwell_satisfied = false_0 != 0;
}
unsafe extern "C" fn predictor_attitude_step(
    mut g: *mut PredictorGuidance,
    mut target_bank: ::core::ffi::c_double,
    mut target_aoa: ::core::ffi::c_double,
    mut a: *const AtmosState,
    mut v: *const VehicleProfile,
    mut set: *const GuidanceSettings,
    mut dt: ::core::ffi::c_double,
    mut physics: *const VesselPhysicsModel,
) {
    let mut measured_roll: bool = !physics.is_null()
        && (*physics).authority_confidence[1 as ::core::ffi::c_int as usize]
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*physics).authority[1 as ::core::ffi::c_int as usize].is_finite() as i32 != 0
        && (*physics).authority[1 as ::core::ffi::c_int as usize] > 1e-6f64;
    let mut measured_pitch: bool = !physics.is_null()
        && (*physics).authority_confidence[0 as ::core::ffi::c_int as usize]
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*physics).authority[0 as ::core::ffi::c_int as usize].is_finite() as i32 != 0
        && (*physics).authority[0 as ::core::ffi::c_int as usize] > 1e-6f64;
    if measured_roll {
        vessel_physics_axis_step(
            physics,
            1 as ::core::ffi::c_int,
            target_bank,
            (*a).q,
            (*set).entry_roll_rate,
            dt,
            &raw mut (*g).actual_bank,
            &raw mut (*g).actual_bank_rate,
        );
    } else {
        let mut regime: FlightControlRegime = flight_control_regime(
            (*a).mach,
            (*a).speed,
            (*a).q,
            (*v).minimum_safe_speed,
        );
        let mut bank_error: ::core::ffi::c_double = norm_signed_deg(
            target_bank - (*g).actual_bank,
        );
        let mut max_rr: ::core::ffi::c_double = fmax(
            0.8f64,
            (*set).entry_roll_rate * 1.12f64 * regime.response_scale,
        );
        let mut max_ra: ::core::ffi::c_double = fmax(
            0.45f64,
            (*set).entry_roll_acceleration * 1.15f64 * regime.response_scale,
        );
        let mut desired_rr: ::core::ffi::c_double = clampd(
            bank_error * 0.72f64,
            -max_rr,
            max_rr,
        );
        let mut rr_delta: ::core::ffi::c_double = clampd(
            desired_rr - (*g).actual_bank_rate,
            -max_ra * dt,
            max_ra * dt,
        );
        (*g).actual_bank_rate = clampd(
            (*g).actual_bank_rate + rr_delta,
            -max_rr * 1.35f64,
            max_rr * 1.35f64,
        );
        (*g).actual_bank = norm_signed_deg(
            (*g).actual_bank + (*g).actual_bank_rate * dt,
        );
    }
    if measured_pitch {
        vessel_physics_axis_step(
            physics,
            0 as ::core::ffi::c_int,
            target_aoa,
            (*a).q,
            (*set).entry_roll_rate,
            dt,
            &raw mut (*g).actual_aoa,
            &raw mut (*g).actual_aoa_rate,
        );
    } else {
        let mut aoa_error: ::core::ffi::c_double = target_aoa - (*g).actual_aoa;
        let mut max_ar: ::core::ffi::c_double = 4.2f64;
        let mut max_aa: ::core::ffi::c_double = 2.4f64;
        let mut desired_ar: ::core::ffi::c_double = clampd(
            aoa_error * 0.62f64,
            -max_ar,
            max_ar,
        );
        let mut ar_delta: ::core::ffi::c_double = clampd(
            desired_ar - (*g).actual_aoa_rate,
            -max_aa * dt,
            max_aa * dt,
        );
        (*g).actual_aoa_rate = clampd(
            (*g).actual_aoa_rate + ar_delta,
            -max_ar * 1.25f64,
            max_ar * 1.25f64,
        );
        (*g).actual_aoa = clampd(
            (*g).actual_aoa + (*g).actual_aoa_rate * dt,
            -(*v).maximum_angle_of_attack * 1.5f64,
            (*v).maximum_angle_of_attack * 1.5f64,
        );
    };
}
unsafe extern "C" fn predictor_entry_terminal_turn_reserve_radius(
    mut a: *const AtmosState,
    mut p: *const PlanetModel,
    mut c: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut set: *const GuidanceSettings,
    mut bank_limit_deg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut reserve: ::core::ffi::c_double = (*set).hac_radius;
    if a.is_null() || p.is_null() || c.is_null() || v.is_null() || set.is_null()
        || !((*a).speed > DBL_MIN) || !(bank_limit_deg > 0.0f64)
    {
        return reserve;
    }
    let mut live_radius: ::core::ffi::c_double = turn_radius_for_bank(
        a,
        c,
        bank_limit_deg,
    );
    let mut design_speed: ::core::ffi::c_double = fmax(
        entry_taem_speed_target(v, set, p),
        fmax((*v).minimum_safe_speed, (*v).final_approach_speed),
    );
    if live_radius.is_finite() as i32 != 0 && live_radius > 0.0f64
        && design_speed.is_finite() as i32 != 0 && design_speed > 0.0f64
    {
        let mut scale: ::core::ffi::c_double = design_speed / (*a).speed;
        let mut authority_radius: ::core::ffi::c_double = live_radius * scale * scale;
        if authority_radius.is_finite() as i32 != 0 && authority_radius > 0.0f64 {
            reserve = fmax(reserve, authority_radius);
        }
    }
    return reserve;
}
unsafe extern "C" fn predictor_interface_heading(
    mut g: *const PredictorGuidance,
    mut geo: GeoPoint,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut fallback_bearing: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if g.is_null() || !(*g).has_interface_target || !(*g).interface_target.valid {
        return fallback_bearing;
    }
    let mut site_geo: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut along: ::core::ffi::c_double = 0.0f64;
    let mut cross: ::core::ffi::c_double = 0.0f64;
    runway_coordinates(
        geo,
        site_geo,
        (*site).runway_heading,
        (*p).radius,
        &raw mut along,
        &raw mut cross,
    );
    return taem_interface_line_heading(
        along,
        cross,
        (*site).runway_heading,
        &raw const (*g).interface_target,
    );
}
unsafe extern "C" fn predictor_terminal_arc_ready(
    mut geo: GeoPoint,
    mut course: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut q: *const TaemInterfaceTarget,
    mut final_sign: ::core::ffi::c_double,
) -> bool {
    if p.is_null() || site.is_null() || q.is_null() || !(*q).valid
        || course.is_finite() as i32 == 0
    {
        return true_0 != 0;
    }
    let mut site_geo: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut along: ::core::ffi::c_double = 0.0f64;
    let mut cross: ::core::ffi::c_double = 0.0f64;
    runway_coordinates(
        geo,
        site_geo,
        (*site).runway_heading,
        (*p).radius,
        &raw mut along,
        &raw mut cross,
    );
    let mut da: ::core::ffi::c_double = (*q).along_track - along;
    let mut dc: ::core::ffi::c_double = (*q).cross_track - cross;
    if hypot(da, dc) < 1000.0f64 {
        return true_0 != 0;
    }
    let mut bearing: ::core::ffi::c_double = norm_deg(
        (*site).runway_heading + atan2(dc, da) * RAD2DEG,
    );
    let mut side: ::core::ffi::c_double = if final_sign >= 0.0f64 {
        1.0f64
    } else {
        -1.0f64
    };
    let mut gate_turn: ::core::ffi::c_double = norm_signed_deg(bearing - course);
    let mut tangent_turn: ::core::ffi::c_double = norm_signed_deg((*q).course - course);
    return side * gate_turn >= 1.0f64 && side * tangent_turn >= -0.5f64;
}
unsafe extern "C" fn entry_bank(
    mut g: *mut PredictorGuidance,
    mut s: VehicleState,
    mut geo: GeoPoint,
    mut a: *const AtmosState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut c: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut maxbank: ::core::ffi::c_double,
    mut dt: ::core::ffi::c_double,
    mut aoa_out: *mut ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut up: Vector3 = vnorm(
        s.position,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut air: Vector3 = vsub(
        s.velocity,
        vcross(planet_rotation_vector(p), s.position),
    );
    let mut hv: Vector3 = vproject_plane(air, up);
    let mut north: Vector3 = vnorm(
        vsub((*p).north_axis, vscale(up, vdot((*p).north_axis, up))),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut east: Vector3 = vnorm(
        vcross(north, up),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut target: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut tangent_lock: bool = (*g).final_heading_lock as ::core::ffi::c_int != 0
        && (*g).has_interface_target as ::core::ffi::c_int != 0
        && (*g).interface_target.valid as ::core::ffi::c_int != 0;
    let mut heading_target: GeoPoint = if (*g).final_heading_lock as ::core::ffi::c_int
        != 0 && !tangent_lock
    {
        runway_approach_aimpoint(site, (*p).radius, 500.0f64)
    } else {
        target
    };
    let mut course: ::core::ffi::c_double = if vmag(hv)
        > 1 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        norm_deg(atan2(vdot(hv, east), vdot(hv, north)) * RAD2DEG)
    } else {
        initial_bearing(geo, heading_target)
    };
    let mut bearing: ::core::ffi::c_double = initial_bearing(geo, heading_target);
    if (*g).has_interface_target as ::core::ffi::c_int != 0
        && (*g).interface_target.valid as ::core::ffi::c_int != 0
    {
        bearing = predictor_interface_heading(g, geo, p, site, set, bearing);
    }
    let mut he: ::core::ffi::c_double = norm_signed_deg(bearing - course);
    let mut range: ::core::ffi::c_double = great_circle_distance(
        geo,
        target,
        (*p).radius,
    );
    let mut grav: ::core::ffi::c_double = (*p).gravitational_parameter
        / pow(
            (*p).radius
                + fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, geo.altitude),
            2 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    if (*g).entry_speed <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*a).speed > 100 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        (*g).entry_speed = (*a).speed;
    }
    let mut delivery_target: *const TaemInterfaceTarget = if (*g).has_interface_target
        as ::core::ffi::c_int != 0
        && (*g).interface_target.valid as ::core::ffi::c_int != 0
    {
        &raw mut (*g).interface_target
    } else {
        ::core::ptr::null_mut::<TaemInterfaceTarget>()
    };
    let mut required_drag: ::core::ffi::c_double = entry_required_average_drag(
        geo,
        (*a).speed,
        range,
        p,
        v,
        site,
        set,
        delivery_target,
    );
    let mut drag_ratio: ::core::ffi::c_double = if required_drag > 1e-6f64 {
        (*a).drag_accel / required_drag
    } else {
        1.0f64
    };
    let mut drag_deficit: ::core::ffi::c_double = clampd(
        1.0f64 - drag_ratio,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut shared_bank_limit: ::core::ffi::c_double = bank_limit(a, p, v, maxbank);
    let mut env: ::core::ffi::c_double = fmin(60.0f64, shared_bank_limit);
    let mut minimum_radius: ::core::ffi::c_double = turn_radius_for_bank(a, c, env);
    let mut mag: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut forced_policy: bool = (*g).force_control as ::core::ffi::c_int != 0
        && s.ut < (*g).forced_until_ut;
    if forced_policy {
        env = shared_bank_limit;
    }
    if forced_policy {
        mag = clampd(
            fabs((*g).forced_bank),
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            env,
        );
        let mut lift: ::core::ffi::c_double = (*a).lift_accel;
        if !(*c).forecast_exact_policy_bank && lift > 0.0f64 && (*a).speed > 1.0f64
            && range > 0.0f64
        {
            let mut vertical: ::core::ffi::c_double = lift * cos(mag * DEG2RAD);
            let mut t_go: ::core::ffi::c_double = range / (*a).speed;
            let mut lateral: ::core::ffi::c_double = 2.0f64
                * fabs(range * sin(he * DEG2RAD)) / (t_go * t_go);
            if hypot(vertical, lateral) > lift {
                mag = clampd(
                    atan2(lateral, vertical) * RAD2DEG,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    env,
                );
                if (if he >= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
                    1 as ::core::ffi::c_int
                } else {
                    -(1 as ::core::ffi::c_int)
                }) as ::core::ffi::c_double != (*g).sign
                {
                    predictor_change_sign(
                        g,
                        (if he >= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
                            1 as ::core::ffi::c_int
                        } else {
                            -(1 as ::core::ffi::c_int)
                        }) as ::core::ffi::c_double,
                    );
                }
            }
        }
    } else if drag_deficit > 0.01f64 && minimum_radius.is_finite() as i32 != 0 {
        let mut radius_scale: ::core::ffi::c_double = 8.0f64
            - (8.0f64 - 1.20f64) * drag_deficit;
        mag = bank_for_turn_radius(a, c, minimum_radius * radius_scale, env);
    }
    if (*g).sign == 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        predictor_change_sign(
            g,
            (if he >= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
                1 as ::core::ffi::c_int
            } else {
                -(1 as ::core::ffi::c_int)
            }) as ::core::ffi::c_double,
        );
    }
    let mut effective_min_leg: ::core::ffi::c_double = entry_s_turn_effective_minimum_leg(
        (*a).speed,
        0.0f64,
        set,
    );
    let mut executable_min_leg: ::core::ffi::c_double = effective_min_leg;
    if (*g).established as ::core::ffi::c_int != 0
        && (*g).leg_elapsed >= (*set).s_turn_minimum_leg_duration
    {
        (*g).configured_leg_dwell_satisfied = true_0 != 0;
    }
    let mut configured_minleg: bool = (*g).configured_leg_dwell_satisfied;
    let mut minleg: bool = (*g).established as ::core::ffi::c_int != 0
        && (*g).leg_elapsed >= executable_min_leg;
    let mut reversal_target: ::core::ffi::c_double = -(*g).sign * fabs(mag);
    let mut reversal_error: ::core::ffi::c_double = norm_signed_deg(
        reversal_target - (*g).actual_bank,
    );
    let mut reversal_time: ::core::ffi::c_double = decision_axis_capture_time(
        reversal_error,
        (*g).actual_bank_rate,
        fabs((*set).entry_roll_acceleration),
        fabs((*set).entry_roll_rate),
    );
    if reversal_time.is_finite() as i32 == 0 {
        reversal_time = effective_min_leg;
    }
    let mut range_after_roll: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        range - (*a).speed * reversal_time,
    );
    let mut terminal_join_range: ::core::ffi::c_double = entry_taem_range_target(p, set);
    let mut corridor_fraction: ::core::ffi::c_double = clampd(
        (range_after_roll - terminal_join_range)
            / fmax(
                (*set).target_entry_range - terminal_join_range,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    corridor_fraction = corridor_fraction * corridor_fraction
        * (3.0f64 - 2.0f64 * corridor_fraction);
    let mut mission_entry_range: ::core::ffi::c_double = fmax(
        (*set).target_entry_range,
        terminal_join_range + 1000.0f64,
    );
    let mut mission_entry_altitude: ::core::ffi::c_double = entry_guidance_start_altitude(
        p,
        set,
    );
    let mut desired_range_altitude: ::core::ffi::c_double = entry_altitude_target_for_range(
        mission_entry_altitude,
        mission_entry_range,
        range,
        (*set).taem_interface_altitude,
        terminal_join_range,
    );
    let mut predicted_fpa: ::core::ffi::c_double = atan2(
        vdot(air, up),
        fmax(DBL_MIN, vmag(hv)),
    ) * RAD2DEG;
    let mut physical_terminal_radius: ::core::ffi::c_double = predictor_entry_terminal_turn_reserve_radius(
        a,
        p,
        c,
        v,
        set,
        shared_bank_limit,
    );
    let mut near_corridor: ::core::ffi::c_double = atan2(
        2.0f64 * physical_terminal_radius,
        fmax(mission_entry_range, DBL_MIN),
    ) * RAD2DEG;
    let mut far_corridor: ::core::ffi::c_double = fmax(
        near_corridor,
        (*set).hac_look_ahead_angle,
    );
    let mut heading_corridor: ::core::ffi::c_double = near_corridor
        + (far_corridor - near_corridor) * corridor_fraction;
    let mut interface_distance: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    if !delivery_target.is_null() {
        let mut ih: ::core::ffi::c_double = (*site).runway_heading * DEG2RAD;
        let mut ie: ::core::ffi::c_double = (*delivery_target).along_track * sin(ih)
            + (*delivery_target).cross_track * cos(ih);
        let mut in_0: ::core::ffi::c_double = (*delivery_target).along_track * cos(ih)
            - (*delivery_target).cross_track * sin(ih);
        let mut origin: GeoPoint = GeoPoint {
            latitude: (*site).latitude,
            longitude: (*site).longitude,
            altitude: (*site).altitude,
        };
        let mut interface_geo: GeoPoint = local_point(
            origin,
            ie,
            in_0,
            (*p).radius,
            (*delivery_target).altitude,
        );
        interface_distance = great_circle_distance(geo, interface_geo, (*p).radius);
    }
    if !forced_policy && minimum_radius.is_finite() as i32 != 0
        && fabs(he) > heading_corridor
    {
        let mut excess: ::core::ffi::c_double = clampd(
            (fabs(he) - heading_corridor) / 30.0f64,
            0.0f64,
            1.0f64,
        );
        let mut radius_scale_0: ::core::ffi::c_double = 6.0f64
            - (6.0f64 - 1.5f64) * excess;
        mag = fmax(
            mag,
            bank_for_turn_radius(a, c, minimum_radius * radius_scale_0, env),
        );
    }
    let mut terminal_reversal_reserve: ::core::ffi::c_double = 0.;
    let mut terminal_reversal_metric: ::core::ffi::c_double = 0.;
    if !delivery_target.is_null() {
        terminal_reversal_reserve = fmax(
            (*delivery_target).acquisition_lead,
            (*a).speed * (reversal_time + effective_min_leg),
        );
        terminal_reversal_metric = fmax(
            0.0f64,
            interface_distance - (*a).speed * reversal_time,
        );
    } else {
        terminal_reversal_reserve = terminal_join_range
            + (*a).speed * (reversal_time + effective_min_leg);
        terminal_reversal_metric = range_after_roll;
    }
    let mut remaining_taem_energy: ::core::ffi::c_double = 0.;
    if !delivery_target.is_null()
        && (*delivery_target).specific_energy.is_finite() as i32 != 0
    {
        let mut current_energy: ::core::ffi::c_double = rotating_specific_energy(
            geo.latitude,
            geo.altitude,
            (*a).speed,
            p,
        );
        remaining_taem_energy = if current_energy.is_finite() as i32 != 0 {
            fmax(0.0f64, current_energy - (*delivery_target).specific_energy)
        } else {
            ::core::f32::INFINITY as ::core::ffi::c_double
        };
    } else {
        remaining_taem_energy = entry_energy_to_taem(
            geo.latitude,
            geo.altitude,
            (*a).speed,
            p,
            v,
            site,
            set,
        );
    }
    let mut terminal_removable_energy: ::core::ffi::c_double = fmax(
        0.0f64,
        (*a).drag_accel,
    ) * terminal_reversal_reserve;
    let mut terminal_energy_ready: bool = remaining_taem_energy.is_finite() as i32 != 0
        && remaining_taem_energy <= terminal_removable_energy;
    let mut terminal_altitude_ready: bool = geo.altitude <= desired_range_altitude
        || (*a).speed <= (*set).taem_force_handoff_speed;
    let mut terminal_reversal_deadline: bool = terminal_reversal_metric
        <= terminal_reversal_reserve;
    let mut terminal_roll_has_delivery_time: bool = true_0 != 0;
    if (*g).enforce_terminal_delivery_budget as ::core::ffi::c_int != 0
        && !delivery_target.is_null()
        && predicted_fpa > (*delivery_target).flight_path_angle
    {
        let mut net_deceleration: ::core::ffi::c_double = (*a).drag_accel
            + grav * sin(predicted_fpa * DEG2RAD);
        let mut handoff_eta: ::core::ffi::c_double = 0.0f64;
        if (*a).speed > (*set).taem_force_handoff_speed {
            handoff_eta = if net_deceleration > DBL_MIN {
                ((*a).speed - (*set).taem_force_handoff_speed) / net_deceleration
            } else {
                ::core::f32::INFINITY as ::core::ffi::c_double
            };
        }
        let mut required_sink_delta: ::core::ffi::c_double = (*a).speed
            * fmax(
                0.0f64,
                sin(predicted_fpa * DEG2RAD)
                    - sin((*delivery_target).flight_path_angle * DEG2RAD),
            );
        let mut hold_bank: ::core::ffi::c_double = fmin(
            fabs((*g).actual_bank),
            nextafter(90.0f64, 0.0f64),
        ) * DEG2RAD;
        let mut sink_authority: ::core::ffi::c_double = grav
            - (*a).lift_accel * cos(hold_bank);
        let mut fpa_recovery_eta: ::core::ffi::c_double = if sink_authority > DBL_MIN {
            required_sink_delta / sink_authority
        } else {
            ::core::f32::INFINITY as ::core::ffi::c_double
        };
        terminal_roll_has_delivery_time = handoff_eta > reversal_time + fpa_recovery_eta;
    }
    let mut terminal_deadline_roll_ready: bool = terminal_reversal_deadline
        as ::core::ffi::c_int != 0
        && terminal_roll_has_delivery_time as ::core::ffi::c_int != 0;
    let mut terminal_final_sign: ::core::ffi::c_double = if fabs((*g).sign) > 0.1f64 {
        -(*g).sign
    } else if he >= 0.0f64 {
        1.0f64
    } else {
        -1.0f64
    };
    let mut terminal_arc_ready: bool = delivery_target.is_null()
        || predictor_terminal_arc_ready(
            geo,
            course,
            p,
            site,
            delivery_target,
            terminal_final_sign,
        ) as ::core::ffi::c_int != 0;
    let mut terminal_reversal_due: bool = terminal_deadline_roll_ready
        as ::core::ffi::c_int != 0 && terminal_energy_ready as ::core::ffi::c_int != 0
        && terminal_altitude_ready as ::core::ffi::c_int != 0
        && terminal_arc_ready as ::core::ffi::c_int != 0;
    let mut measured_reversal_due: bool = (*g).sign * he < -heading_corridor;
    let mut candidate_sign: ::core::ffi::c_double = if fabs((*g).sign) > 0.1f64 {
        -(*g).sign
    } else {
        0.0f64
    };
    let mut shaping_endpoint_ready: bool = delivery_target.is_null()
        || predictor_terminal_arc_ready(
            geo,
            course,
            p,
            site,
            delivery_target,
            candidate_sign,
        ) as ::core::ffi::c_int != 0;
    let mut target_relative: ::core::ffi::c_double = if !delivery_target.is_null() {
        norm_signed_deg((*delivery_target).course - (*site).runway_heading)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut final_side: ::core::ffi::c_double = if target_relative.is_finite() as i32
        != 0 && fabs(target_relative) > 1e-6f64
    {
        if target_relative >= 0.0f64 { 1.0f64 } else { -1.0f64 }
    } else {
        0.0f64
    };
    let mut live_setup_reversal_due: bool = !delivery_target.is_null()
        && candidate_sign != 0.0f64 && final_side != 0.0f64
        && candidate_sign * final_side > 0.0f64
        && shaping_endpoint_ready as ::core::ffi::c_int != 0;
    let mut divergence_limit: ::core::ffi::c_double = fmax(
        30.0f64,
        2.0f64 * heading_corridor,
    );
    let mut course_diverged: bool = (*g).sign * he < -divergence_limit;
    let mut held: bool = false_0 != 0;
    if !delivery_target.is_null() && measured_reversal_due as ::core::ffi::c_int != 0
        && !shaping_endpoint_ready
    {
        measured_reversal_due = false_0 != 0;
        held = true_0 != 0;
    }
    if !delivery_target.is_null()
        && terminal_reversal_deadline as ::core::ffi::c_int != 0 && !terminal_arc_ready
    {
        measured_reversal_due = false_0 != 0;
        held = true_0 != 0;
    }
    if held as ::core::ffi::c_int != 0 && course_diverged as ::core::ffi::c_int != 0
        && !(*g).final_heading_lock
    {
        measured_reversal_due = true_0 != 0;
        if !getenv(
                b"KSP_LANDER_SETUP_REVERSAL_TRACE\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        {
            fprintf(
                stderr,
                b"PREDICTOR_DIVERGENCE_REVERSAL ut=%.1f range=%.0f he=%.1f corridor=%.1f sign=%.0f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                s.ut,
                range,
                he,
                heading_corridor,
                (*g).sign,
            );
        }
    }
    if course_diverged as ::core::ffi::c_int != 0
        && !getenv(
                b"KSP_LANDER_SETUP_REVERSAL_TRACE\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
    {
        static mut last_trace_ut: ::core::ffi::c_double = -1e9f64;
        if fabs(s.ut - last_trace_ut) >= 30.0f64 {
            last_trace_ut = s.ut;
            fprintf(
                stderr,
                b"PREDICTOR_DIVERGED ut=%.1f range=%.0f he=%.1f sign=%.0f held=%d lock=%d replay=%d mag=%.1f minleg=%d\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                s.ut,
                range,
                he,
                (*g).sign,
                held as ::core::ffi::c_int,
                (*g).final_heading_lock as ::core::ffi::c_int,
                (*g).replay_event as ::core::ffi::c_int,
                mag,
                minleg as ::core::ffi::c_int,
            );
        }
    }
    let mut event_due: bool = (*g).replay_event as ::core::ffi::c_int != 0
        && s.ut >= (*g).replay_event_ut;
    let mut replay_arc_ready: bool = delivery_target.is_null()
        || predictor_terminal_arc_ready(
            geo,
            course,
            p,
            site,
            delivery_target,
            (*g).replay_event_sign,
        ) as ::core::ffi::c_int != 0;
    let mut replay_geometry_ready: bool = replay_arc_ready;
    let mut replay_due: bool = event_due as ::core::ffi::c_int != 0
        && replay_geometry_ready as ::core::ffi::c_int != 0;
    let mut deadline_action: bool = if !delivery_target.is_null() {
        terminal_reversal_due as ::core::ffi::c_int
    } else {
        terminal_deadline_roll_ready as ::core::ffi::c_int
    } != 0;
    let mut reverse_due: bool = if (*g).replay_event as ::core::ffi::c_int != 0 {
        replay_due as ::core::ffi::c_int
    } else {
        (live_setup_reversal_due as ::core::ffi::c_int != 0
            || measured_reversal_due as ::core::ffi::c_int != 0
            || deadline_action as ::core::ffi::c_int != 0) as ::core::ffi::c_int
    } != 0;
    let mut reversal_dwell_ready: bool = minleg as ::core::ffi::c_int != 0
        || !(*g).replay_event
            && (deadline_action as ::core::ffi::c_int != 0
                && configured_minleg as ::core::ffi::c_int != 0
                || live_setup_reversal_due as ::core::ffi::c_int != 0
                    && configured_minleg as ::core::ffi::c_int != 0);
    if !(*g).final_heading_lock
        && mag >= 4 as ::core::ffi::c_int as ::core::ffi::c_double
        && reversal_dwell_ready as ::core::ffi::c_int != 0
        && reverse_due as ::core::ffi::c_int != 0
    {
        let mut old_sign: ::core::ffi::c_double = (*g).sign;
        let mut tangent_final_available: bool = !delivery_target.is_null();
        let mut final_reversal: bool = if (*g).replay_event as ::core::ffi::c_int != 0 {
            ((*g).replay_event_final as ::core::ffi::c_int != 0
                && tangent_final_available as ::core::ffi::c_int != 0)
                as ::core::ffi::c_int
        } else {
            (terminal_reversal_due as ::core::ffi::c_int != 0
                && tangent_final_available as ::core::ffi::c_int != 0)
                as ::core::ffi::c_int
        } != 0;
        let mut event_sign: ::core::ffi::c_double = 0.;
        if (*g).replay_event {
            event_sign = (*g).replay_event_sign;
        } else if final_reversal as ::core::ffi::c_int != 0
            && tangent_final_available as ::core::ffi::c_int != 0
        {
            event_sign = terminal_final_sign;
        } else {
            event_sign = -(*g).sign;
        }
        predictor_change_sign(g, event_sign);
        (*g).replay_event = false_0 != 0;
        if final_reversal {
            (*g).final_heading_lock = true_0 != 0;
        }
        let mut side_changed: bool = old_sign
            != 0 as ::core::ffi::c_int as ::core::ffi::c_double && (*g).sign != old_sign;
        if !(*g).has_first_reversal
            && (side_changed as ::core::ffi::c_int != 0
                || final_reversal as ::core::ffi::c_int != 0)
        {
            (*g).has_first_reversal = true_0 != 0;
            (*g).first_reversal_final = final_reversal;
            (*g).first_reversal_ut = s.ut;
            (*g).first_reversal_range = range;
            (*g).first_reversal_sign = (*g).sign;
        }
    }
    let mut qr: ::core::ffi::c_double = (*a).q
        / fmax(
            (*v).maximum_dynamic_pressure,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    let mut gr: ::core::ffi::c_double = (*a).non_gravity / fmax(grav, 0.1f64)
        / fmax((*v).maximum_g_load, 0.1f64);
    let mut protective_aoa: ::core::ffi::c_double = entry_low_q_protective_aoa_floor(
        (*a).q,
        v,
    );
    let mut aoa: ::core::ffi::c_double = if forced_policy as ::core::ffi::c_int != 0 {
        clampd((*g).forced_aoa, protective_aoa, (*v).maximum_angle_of_attack)
    } else {
        (*v).entry_angle_of_attack
            + ((*v).maximum_angle_of_attack - (*v).entry_angle_of_attack) * drag_deficit
    };
    aoa = fmax(aoa, protective_aoa);
    if !forced_policy && (*a).mach > 4 as ::core::ffi::c_int as ::core::ffi::c_double
        && qr < 0.45f64 && geo.altitude <= desired_range_altitude
    {
        aoa = fmax(aoa, (*v).maximum_angle_of_attack - 1.0f64);
    }
    if qr > 0.92f64 {
        aoa = fmin(aoa + 1.0f64, (*v).maximum_angle_of_attack);
        mag = fmin(
            (*v).maximum_bank_angle,
            mag + 5 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    if gr > 0.92f64 {
        aoa
            -= clampd(
                (gr - 0.9f64) * 18 as ::core::ffi::c_int as ::core::ffi::c_double,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
                7 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
        mag *= 0.6f64;
    }
    aoa = clampd(
        aoa,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        (*v).maximum_angle_of_attack,
    );
    if !aoa_out.is_null() {
        *aoa_out = aoa;
    }
    mag = fmin(mag, env);
    let mut regime: FlightControlRegime = flight_control_regime(
        (*a).mach,
        (*a).speed,
        (*a).q,
        (*v).minimum_safe_speed,
    );
    let mut rs: ::core::ffi::c_double = fmax(0.42f64, regime.command_rate_scale);
    let mut as_0: ::core::ffi::c_double = fmax(0.30f64, regime.command_accel_scale);
    let mut command_mag: ::core::ffi::c_double = if forced_policy as ::core::ffi::c_int
        != 0
    {
        mag
    } else {
        mag / clampd((*c).bank_effectiveness, 0.55f64, 1.45f64)
    };
    let mut targetbank: ::core::ffi::c_double = 0.;
    if (*g).final_heading_lock as ::core::ffi::c_int != 0
        && (*g).has_interface_target as ::core::ffi::c_int != 0
        && (*g).interface_target.valid as ::core::ffi::c_int != 0
    {
        let mut response: ::core::ffi::c_double = fmax(
            4.0f64,
            (*g).interface_target.response_time,
        );
        targetbank = taem_course_rate_bank_command(
            (*a).speed,
            (*a).lift_accel,
            (*c).bank_effectiveness,
            env,
            course,
            bearing,
            response,
        );
        if targetbank * (*g).sign < 0.0f64 && fabs(targetbank) > 18.0f64 {
            targetbank = -(*g).sign * 18.0f64;
        }
    } else {
        targetbank = (*g).sign * fmin(env, command_mag);
    }
    let mut limited: ::core::ffi::c_double = norm_signed_deg(
        jerk_angle_update(
            &raw mut (*g).limiter,
            targetbank,
            (*set).entry_roll_rate * rs,
            (*set).entry_roll_acceleration * as_0,
            dt,
        ),
    );
    let mut predicted_g_force: ::core::ffi::c_double = (*a).non_gravity
        / fmax(planet_surface_gravity(p), 0.1f64);
    let mut predicted_stall_fraction: ::core::ffi::c_double = vessel_physics_conservative_stall_fraction(
        (*a).speed,
        (*g).actual_aoa,
        (*a).q,
        v,
    );
    if !(*g).entry_loaded && (*a).non_gravity >= 1.5f64 {
        (*g).entry_loaded = true_0 != 0;
    }
    let mut aerodynamic_authority: bool = entry_s_turn_bank_authority_available(
        (*a).q,
        (*a).speed,
        predicted_stall_fraction,
        predicted_g_force,
        v,
    );
    let mut loaded: bool = (*g).entry_loaded as ::core::ffi::c_int != 0
        || aerodynamic_authority as ::core::ffi::c_int != 0;
    let mut magnitude: ::core::ffi::c_double = fabs(limited);
    let mut threshold: ::core::ffi::c_double = fmin(
        12 as ::core::ffi::c_int as ::core::ffi::c_double,
        fmax(4 as ::core::ffi::c_int as ::core::ffi::c_double, magnitude * 0.35f64),
    );
    let mut meaningful: bool = loaded as ::core::ffi::c_int != 0
        && magnitude >= threshold;
    let mut captured: bool = meaningful as ::core::ffi::c_int != 0
        && (*g).sign * (*g).actual_bank
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && fabs((*g).actual_bank) >= threshold;
    if !captured {
        (*g).established = false_0 != 0;
        (*g).leg_elapsed = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    } else if !(*g).established {
        (*g).established = true_0 != 0;
        (*g).leg_elapsed = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    } else {
        (*g).leg_elapsed += dt;
    }
    return limited;
}
unsafe extern "C" fn predictor_local_log_density_gradient(
    mut p: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if p.is_null() || (*p).atmosphere_sample_count < 2 as size_t
        || altitude.is_finite() as i32 == 0
    {
        return 0.0f64;
    }
    let mut hi: size_t = 1 as size_t;
    while hi < (*p).atmosphere_sample_count
        && (*p).atmosphere_altitude[hi as usize] < altitude
    {
        hi = hi.wrapping_add(1);
    }
    if hi >= (*p).atmosphere_sample_count {
        hi = (*p).atmosphere_sample_count.wrapping_sub(1 as size_t);
    }
    let mut lo: size_t = hi.wrapping_sub(1 as size_t);
    let mut h0: ::core::ffi::c_double = (*p).atmosphere_altitude[lo as usize];
    let mut h1: ::core::ffi::c_double = (*p).atmosphere_altitude[hi as usize];
    let mut r0: ::core::ffi::c_double = (*p).atmosphere_density[lo as usize];
    let mut r1: ::core::ffi::c_double = (*p).atmosphere_density[hi as usize];
    if !(h1 > h0) || !(r0 > 0.0f64) || !(r1 > 0.0f64) {
        return 0.0f64;
    }
    return fabs(log(r1 / r0) / (h1 - h0));
}
unsafe extern "C" fn predictor_step_size(
    mut a: *const AtmosState,
    mut p: *const PlanetModel,
    mut v: *const VehicleProfile,
    mut set: *const GuidanceSettings,
    mut altitude: ::core::ffi::c_double,
    mut vertical_speed: ::core::ffi::c_double,
    mut entry_alt: ::core::ffi::c_double,
    mut entered: bool,
    mut reached_taem: bool,
    mut roll_rate: ::core::ffi::c_double,
    mut base: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(base > 0.0f64) || base.is_finite() as i32 == 0 {
        return 0.0f64;
    }
    let mut dt: ::core::ffi::c_double = base;
    if !a.is_null() && (*a).density > 0.0f64 {
        if (*a).non_gravity > DBL_EPSILON && (*a).speed > DBL_EPSILON {
            let mut acceleration_timescale: ::core::ffi::c_double = (*a).speed
                / (*a).non_gravity;
            if acceleration_timescale.is_finite() as i32 != 0
                && acceleration_timescale > 0.0f64
            {
                dt = fmin(dt, acceleration_timescale);
            }
        }
        let mut gradient: ::core::ffi::c_double = predictor_local_log_density_gradient(
            p,
            altitude,
        );
        if gradient > DBL_EPSILON && fabs(vertical_speed) > DBL_EPSILON {
            let mut density_timescale: ::core::ffi::c_double = 1.0f64
                / (gradient * fabs(vertical_speed));
            if density_timescale.is_finite() as i32 != 0 && density_timescale > 0.0f64 {
                dt = fmin(dt, density_timescale);
            }
        }
    }
    if fabs(roll_rate) > DBL_EPSILON {
        let mut roll_timescale: ::core::ffi::c_double = 1.0f64
            / (fabs(roll_rate) * DEG2RAD);
        if roll_timescale.is_finite() as i32 != 0 && roll_timescale > 0.0f64 {
            dt = fmin(dt, roll_timescale);
        }
    }
    if !entered && vertical_speed < 0.0f64 && altitude > entry_alt {
        let mut event_time: ::core::ffi::c_double = (altitude - entry_alt)
            / -vertical_speed;
        if event_time.is_finite() as i32 != 0 && event_time > 0.0f64 {
            dt = fmin(dt, event_time);
        }
    }
    let mut numerical_floor: ::core::ffi::c_double = base * sqrt(DBL_EPSILON);
    return fmax(numerical_floor, fmin(dt, base));
}
#[no_mangle]
pub unsafe extern "C" fn entry_prediction_clear(mut p: *mut EntryPrediction) {
    if p.is_null() {
        return;
    }
    trajectory_clear(&raw mut (*p).trajectory);
    memset(
        p as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
}
unsafe extern "C" fn predictor_simulate_entry_core(
    mut state: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env0: *const AerodynamicEnvelope,
    mut cal0: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut maxbank: ::core::ffi::c_double,
    mut initial_bank: ::core::ffi::c_double,
    mut initial_bank_rate: ::core::ffi::c_double,
    mut initial_aoa: ::core::ffi::c_double,
    mut initial_aoa_rate: ::core::ffi::c_double,
    mut initial_sign: ::core::ffi::c_double,
    mut initial_leg: ::core::ffi::c_double,
    mut initial_final_heading_lock: bool,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
    mut force_control: bool,
    mut forced_bank: ::core::ffi::c_double,
    mut forced_aoa: ::core::ffi::c_double,
    mut forced_duration: ::core::ffi::c_double,
    mut control_horizon_duration: ::core::ffi::c_double,
    mut stop_at_taem: bool,
    mut enforce_terminal_delivery_budget: bool,
    mut replay: *const EntryControlPlan,
    mut interface_target: *const TaemInterfaceTarget,
) -> EntryPrediction {
    let mut out: EntryPrediction = EntryPrediction {
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        taem_distance: 0.,
        taem_range_error: 0.,
        closest_distance: 0.,
        entered_atmosphere: false,
        reached_taem: false,
        taem_ownership_boundary_missed: false,
        taem_dynamic_interface_captured: false,
        taem_dynamic_interface_target_valid: false,
        entry_range: 0.,
        entry_flight_path_angle: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        taem_speed: 0.,
        taem_flight_path_angle: 0.,
        taem_altitude: 0.,
        taem_desired_altitude: 0.,
        taem_along_track: 0.,
        taem_cross_track: 0.,
        taem_course: 0.,
        mm304_gate_recorded: false,
        mm304_gate_range: 0.,
        mm304_gate_along_track: 0.,
        mm304_gate_cross_track: 0.,
        mm304_gate_course: 0.,
        mm304_gate_altitude: 0.,
        mm304_gate_speed: 0.,
        mm304_gate_flight_path_angle: 0.,
        taem_energy_error: 0.,
        taem_interface_error: 0.,
        taem_interface_energy_margin: 0.,
        peak_dynamic_pressure: 0.,
        peak_g_load: 0.,
        minimum_entry_speed: 0.,
        maximum_abs_angle_of_attack: 0.,
        minimum_bank_control_margin: 0.,
        minimum_aoa_control_margin: 0.,
        control_horizon_recorded: false,
        control_horizon_speed: 0.,
        control_horizon_altitude: 0.,
        control_horizon_range: 0.,
        control_horizon_specific_energy: 0.,
        control_horizon_alignment_distance: 0.,
        has_first_s_turn_reversal: false,
        first_s_turn_reversal_is_final: false,
        first_s_turn_reversal_ut: 0.,
        first_s_turn_reversal_range: 0.,
        first_s_turn_reversal_sign: 0.,
        s_turn_reversals: 0,
        s_turn_max_bank: 0.,
        final_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        physics_observed_seconds: 0.,
        physics_fallback_seconds: 0.,
        shadow_guidance_used: false,
        shadow_terminal_prediction_valid: false,
        shadow_terminal_policy_feasible: false,
        shadow_terminal_path_committed: false,
        shadow_final_approach_captured: false,
        shadow_aborted: false,
        terminal_survivability_stress_score: 0.,
        physics_relative_uncertainty: 0.,
        uncertainty_scenarios: 0,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
    trajectory_init(&raw mut out.trajectory);
    out.closest_distance = DBL_MAX;
    out.taem_distance = DBL_MAX;
    out.taem_altitude = ::core::f32::NAN as ::core::ffi::c_double;
    out.taem_desired_altitude = ::core::f32::NAN as ::core::ffi::c_double;
    out.minimum_entry_speed = DBL_MAX;
    out.minimum_bank_control_margin = ::core::f32::INFINITY as ::core::ffi::c_double;
    out.minimum_aoa_control_margin = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut uniform: AerodynamicEnvelope = AerodynamicEnvelope {
        regimes: [AerodynamicModel {
            lift_to_drag: 0.,
            ballistic_coefficient: 0.,
            confidence: 0.,
        }; 4],
    };
    if env0.is_null() {
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < 4 as ::core::ffi::c_int {
            uniform.regimes[i as usize] = aero;
            i += 1;
        }
        env0 = &raw mut uniform;
    }
    let mut dc: TrajectoryCalibrationModel = TrajectoryCalibrationModel {
        physics: ::core::ptr::null::<VesselPhysicsModel>(),
        density_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        drag_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        lift_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        bank_effectiveness: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_of_sound: 340 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_of_sound_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        stress_drag_scale: 0.,
        stress_lift_scale: 0.,
        forecast_drag_scale: 0.,
        forecast_lift_scale: 0.,
        forecast_exact_policy_bank: false,
        altitude_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        range_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        confidence: 0.05f64,
        accepted_samples: 0 as ::core::ffi::c_int,
    };
    if cal0.is_null() {
        cal0 = &raw mut dc;
    }
    let mut g: PredictorGuidance = PredictorGuidance {
        sign: 0.,
        leg_elapsed: 0.,
        established: false,
        entry_loaded: false,
        configured_leg_dwell_satisfied: false,
        reversals: 0,
        limiter: JerkLimiter {
            value: 0.,
            rate: 0.,
            has_value: false,
        },
        has_first_reversal: false,
        first_reversal_final: false,
        final_heading_lock: false,
        first_reversal_ut: 0.,
        first_reversal_range: 0.,
        first_reversal_sign: 0.,
        replay_event: false,
        replay_event_final: false,
        replay_event_ut: 0.,
        replay_event_sign: 0.,
        entry_score: 0.,
        has_entry_score: false,
        actual_bank: 0.,
        actual_bank_rate: 0.,
        actual_aoa: 0.,
        actual_aoa_rate: 0.,
        entry_speed: 0.,
        has_interface_target: false,
        enforce_terminal_delivery_budget: false,
        interface_target: TaemInterfaceTarget {
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
        force_control: false,
        forced_until_ut: 0.,
        control_horizon_ut: 0.,
        forced_bank: 0.,
        forced_aoa: 0.,
        speedbrake: SpeedbrakeController {
            integral: 0.,
            deployed: false,
            initialized: false,
        },
    };
    memset(
        &raw mut g as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PredictorGuidance>() as size_t,
    );
    g.sign = (if fabs(initial_sign) > 0.1f64 {
        if initial_sign >= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        }
    } else {
        0 as ::core::ffi::c_int
    }) as ::core::ffi::c_double;
    g.leg_elapsed = fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, initial_leg);
    g.established = initial_leg > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    g.final_heading_lock = initial_final_heading_lock;
    g.limiter.has_value = true_0 != 0;
    g.limiter.value = norm_deg(initial_bank);
    g.limiter.rate = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    g.actual_bank = norm_signed_deg(initial_bank);
    g.actual_bank_rate = clampd(
        initial_bank_rate,
        -(60 as ::core::ffi::c_int) as ::core::ffi::c_double,
        60 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    g.actual_aoa = if initial_aoa.is_finite() as i32 != 0 {
        clampd(
            initial_aoa,
            -(*v).maximum_angle_of_attack * 1.5f64,
            (*v).maximum_angle_of_attack * 1.5f64,
        )
    } else {
        scheduled_aoa(vmag(state.position) - (*p).radius, p, v)
    };
    g.actual_aoa_rate = if initial_aoa_rate.is_finite() as i32 != 0 {
        clampd(
            initial_aoa_rate,
            -(15 as ::core::ffi::c_int) as ::core::ffi::c_double,
            15 as ::core::ffi::c_int as ::core::ffi::c_double,
        )
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    g.force_control = force_control as ::core::ffi::c_int != 0
        && forced_duration > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    g.forced_until_ut = state.ut
        + fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, forced_duration);
    g.control_horizon_ut = state.ut
        + fmax(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            control_horizon_duration,
        );
    g.forced_bank = forced_bank;
    g.forced_aoa = forced_aoa;
    g.enforce_terminal_delivery_budget = enforce_terminal_delivery_budget;
    g.entry_loaded = initial_leg > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    g.configured_leg_dwell_satisfied = initial_leg + 1e-6f64
        >= (*set).s_turn_minimum_leg_duration;
    if !interface_target.is_null()
        && (*interface_target).valid as ::core::ffi::c_int != 0
    {
        g.has_interface_target = true_0 != 0;
        g.interface_target = *interface_target;
    }
    let mut interface_cfg: LandingConfiguration = LandingConfiguration {
        connection: ConnectionConfiguration {
            serial_port: [0; 256],
            rpc_host: [0; 256],
            rpc_port: 0,
            baud_rate: 0,
            timeout_ms: 0,
            client_name: [0; 128],
        },
        site: *site,
        vehicle: *v,
        guidance: *set,
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
    if g.sign * g.actual_bank <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || fabs(g.actual_bank) < 4 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        g.established = false_0 != 0;
        g.leg_elapsed = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    if !replay.is_null() && (*replay).has_planned_reversal as ::core::ffi::c_int != 0
        && (*replay).planned_reversal_ut.is_finite() as i32 != 0
        && fabs((*replay).planned_reversal_sign) > 0.1f64
    {
        g.replay_event = true_0 != 0;
        g.replay_event_ut = (*replay).planned_reversal_ut;
        g.replay_event_sign = (*replay).planned_reversal_sign;
        g.replay_event_final = (*replay).planned_reversal_is_final;
    }
    let mut target: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut entry_alt: ::core::ffi::c_double = entry_guidance_start_altitude(p, set);
    let mut elapsed: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut lastsample: ::core::ffi::c_double = -DBL_MAX;
    let mut mm304_gate_altitude: ::core::ffi::c_double = fmin(
        50000.0f64,
        entry_alt - 5000.0f64,
    );
    let mut initial_gear: bool = if !(*cal0).physics.is_null() {
        (*(*cal0).physics).gear as ::core::ffi::c_int
    } else {
        false_0
    } != 0;
    let mut initial_brakes: bool = if !(*cal0).physics.is_null() {
        (*(*cal0).physics).brakes as ::core::ffi::c_int
    } else {
        false_0
    } != 0;
    let mut initial_airbrakes: ::core::ffi::c_int = if !(*cal0).physics.is_null()
        && (*(*cal0).physics).airbrakes >= 0 as ::core::ffi::c_int
    {
        (*(*cal0).physics).airbrakes
    } else {
        0 as ::core::ffi::c_int
    };
    speedbrake_controller_reset(
        &raw mut g.speedbrake,
        initial_airbrakes != 0 as ::core::ffi::c_int,
    );
    let mut ctx: AeroContext = AeroContext {
        p: p,
        e: env0,
        c: cal0,
        v: v,
        entry_table: ::core::ptr::null::<TerminalModel>(),
        bank: g.actual_bank,
        aoa: g.actual_aoa,
        ut: state.ut,
        mass: state.mass,
        gear: initial_gear,
        brakes: initial_brakes,
        airbrakes: initial_airbrakes,
    };
    while elapsed < maxdur {
        let mut geo: GeoPoint = predictor_geo_point(state.position, p, state.ut);
        let mut range: ::core::ffi::c_double = great_circle_distance(
            geo,
            target,
            (*p).radius,
        );
        let mut at: AtmosState = atmosphere(
            state.position,
            state.velocity,
            &raw mut ctx,
        );
        let mut up: Vector3 = vnorm(
            state.position,
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut vs: ::core::ffi::c_double = vdot(at.air_velocity, up);
        let mut hs: ::core::ffi::c_double = vmag(vproject_plane(at.air_velocity, up));
        let mut fpa: ::core::ffi::c_double = atan2(vs, fmax(hs, 0.1f64)) * RAD2DEG;
        if force_control as ::core::ffi::c_int != 0 && !out.control_horizon_recorded
            && state.ut >= g.control_horizon_ut
        {
            out.control_horizon_recorded = true_0 != 0;
            out.control_horizon_speed = at.speed;
            out.control_horizon_altitude = geo.altitude;
            out.control_horizon_range = range;
            out.control_horizon_specific_energy = rotating_specific_energy(
                geo.latitude,
                geo.altitude,
                at.speed,
                p,
            );
            let mut along: ::core::ffi::c_double = 0 as ::core::ffi::c_int
                as ::core::ffi::c_double;
            let mut cross: ::core::ffi::c_double = 0 as ::core::ffi::c_int
                as ::core::ffi::c_double;
            runway_coordinates(
                geo,
                target,
                (*site).runway_heading,
                (*p).radius,
                &raw mut along,
                &raw mut cross,
            );
            out.control_horizon_alignment_distance = -along
                - (*set).final_approach_distance;
        }
        if !out.entered_atmosphere && geo.altitude <= entry_alt
            && vs < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            out.entered_atmosphere = true_0 != 0;
            out.entry_range = range;
            out.entry_flight_path_angle = fpa;
            out.entry_speed = at.speed;
            let mut course: ::core::ffi::c_double = surface_course(
                state.position,
                state.velocity,
                planet_rotation_vector(p),
                (*p).north_axis,
                initial_bearing(geo, target),
            );
            out.entry_course_error = norm_signed_deg(
                initial_bearing(geo, target) - course,
            );
        }
        if out.entered_atmosphere as ::core::ffi::c_int != 0 && !out.mm304_gate_recorded
            && geo.altitude <= mm304_gate_altitude && vs < 0.0f64
        {
            let mut gate_along: ::core::ffi::c_double = 0.0f64;
            let mut gate_cross: ::core::ffi::c_double = 0.0f64;
            runway_coordinates(
                geo,
                target,
                (*site).runway_heading,
                (*p).radius,
                &raw mut gate_along,
                &raw mut gate_cross,
            );
            let mut gate_course: ::core::ffi::c_double = surface_course(
                state.position,
                state.velocity,
                planet_rotation_vector(p),
                (*p).north_axis,
                initial_bearing(geo, target),
            );
            out.mm304_gate_recorded = true_0 != 0;
            out.mm304_gate_range = range;
            out.mm304_gate_along_track = gate_along;
            out.mm304_gate_cross_track = gate_cross;
            out.mm304_gate_course = gate_course;
            out.mm304_gate_altitude = geo.altitude;
            out.mm304_gate_speed = at.speed;
            out.mm304_gate_flight_path_angle = fpa;
        }
        if out.entered_atmosphere {
            out.closest_distance = fmin(out.closest_distance, range);
        }
        let mut ownership_along: ::core::ffi::c_double = 0.0f64;
        let mut ownership_cross: ::core::ffi::c_double = 0.0f64;
        let mut ownership_ready: bool = false_0 != 0;
        let mut ownership_boundary_missed: bool = false_0 != 0;
        if out.entered_atmosphere {
            runway_coordinates(
                geo,
                target,
                (*site).runway_heading,
                (*p).radius,
                &raw mut ownership_along,
                &raw mut ownership_cross,
            );
            let mut capture_course: ::core::ffi::c_double = surface_course(
                state.position,
                state.velocity,
                planet_rotation_vector(p),
                (*p).north_axis,
                initial_bearing(geo, target),
            );
            let mut local: AerodynamicModel = aero_at(env0, at.mach);
            let mut capture_state: Telemetry = Telemetry {
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
            capture_state.ut = state.ut;
            capture_state.latitude = geo.latitude;
            capture_state.longitude = geo.longitude;
            capture_state.mean_altitude = geo.altitude;
            capture_state.radar_altitude = geo.altitude - (*site).altitude;
            capture_state.true_air_speed = at.speed;
            capture_state.horizontal_speed = hs;
            capture_state.vertical_speed = vs;
            capture_state.flight_path_angle = fpa;
            capture_state.runway_along_track = ownership_along;
            capture_state.runway_cross_track = ownership_cross;
            capture_state.course_to_site_error = norm_signed_deg(
                initial_bearing(geo, target) - capture_course,
            );
            capture_state.dynamic_pressure = at.q;
            capture_state.g_force = fabs(at.lift_accel) / planet_surface_gravity(p);
            capture_state.bank_effectiveness = (*cal0).bank_effectiveness;
            capture_state.mach = at.mach;
            capture_state.atmospheric_density = at.density;
            capture_state.speed_of_sound = at.speed_of_sound;
            capture_state.angle_of_attack = g.actual_aoa;
            capture_state.roll = g.actual_bank;
            capture_state.roll_rate = g.actual_bank_rate;
            capture_state.angle_of_attack_rate = g.actual_aoa_rate;
            capture_state.has_angle_of_attack_rate = true_0 != 0;
            capture_state.mass = state.mass;
            capture_state.lift_force = fabs(at.lift_accel) * state.mass;
            capture_state.drag_force = fabs(at.drag_accel) * state.mass;
            capture_state.estimated_lift_to_drag = local.lift_to_drag;
            capture_state.estimated_ballistic_coefficient = local.ballistic_coefficient;
            capture_state.aerodynamic_confidence = local.confidence;
            if entry_mm305_admission_prefilter_veto(
                &raw mut capture_state,
                p,
                &raw mut interface_cfg,
            ) == 0 as ::core::ffi::c_uint
            {
                let mut capture: TaemInterfaceCapture = entry_mm305_admission_envelope(
                    ::core::ptr::null::<GuidanceMachine>(),
                    &raw mut capture_state,
                    capture_course,
                    p,
                    &raw mut interface_cfg,
                );
                ownership_ready = capture.valid as ::core::ffi::c_int != 0
                    && capture.ready as ::core::ffi::c_int != 0;
            }
            ownership_boundary_missed = !ownership_ready
                && geo.altitude < (*set).mm305_min_altitude && vs < 0.0f64;
        }
        if out.entered_atmosphere as ::core::ffi::c_int != 0 && !out.reached_taem {
            out.minimum_entry_speed = fmin(out.minimum_entry_speed, at.speed);
            out.maximum_abs_angle_of_attack = fmax(
                out.maximum_abs_angle_of_attack,
                fabs(g.actual_aoa),
            );
            if force_control as ::core::ffi::c_int != 0 && state.ut < g.forced_until_ut {
                let mut available_bank: ::core::ffi::c_double = bank_limit(
                    &raw mut at,
                    p,
                    v,
                    maxbank,
                );
                out.minimum_bank_control_margin = fmin(
                    out.minimum_bank_control_margin,
                    available_bank - fabs(g.forced_bank),
                );
                out.minimum_aoa_control_margin = fmin(
                    out.minimum_aoa_control_margin,
                    fmin(g.forced_aoa, (*v).maximum_angle_of_attack - g.forced_aoa),
                );
            }
        }
        out.peak_dynamic_pressure = fmax(out.peak_dynamic_pressure, at.q);
        out.peak_g_load = fmax(
            out.peak_g_load,
            at.non_gravity / fmax(planet_surface_gravity(p), 0.1f64),
        );
        if ownership_ready as ::core::ffi::c_int != 0 && !out.reached_taem {
            let mut target_speed: ::core::ffi::c_double = entry_taem_speed_target(
                v,
                set,
                p,
            );
            let mut target_altitude: ::core::ffi::c_double = (*set)
                .taem_interface_altitude;
            out.reached_taem = true_0 != 0;
            out.taem_distance = range;
            out.taem_range_error = range - entry_taem_range_target(p, set);
            out.taem_speed = at.speed;
            out.taem_flight_path_angle = fpa;
            out.taem_altitude = geo.altitude;
            out.taem_desired_altitude = target_altitude;
            out.taem_along_track = ownership_along;
            out.taem_cross_track = ownership_cross;
            out.taem_course = surface_course(
                state.position,
                state.velocity,
                planet_rotation_vector(p),
                (*p).north_axis,
                initial_bearing(geo, target),
            );
            out.taem_energy_error = entry_remaining_specific_energy(
                geo.latitude,
                geo.altitude,
                at.speed,
                (*site).latitude,
                target_altitude,
                target_speed,
                p,
            );
        }
        if stop_at_taem as ::core::ffi::c_int != 0
            && (out.reached_taem as ::core::ffi::c_int != 0
                || ownership_boundary_missed as ::core::ffi::c_int != 0)
        {
            out.taem_ownership_boundary_missed = ownership_boundary_missed;
            if include as ::core::ffi::c_int != 0
                && (out.trajectory.count == 0 as size_t
                    || fabs(
                        (*out
                            .trajectory
                            .points
                            .offset(
                                out.trajectory.count.wrapping_sub(1 as size_t) as isize,
                            ))
                            .ut - state.ut,
                    ) > 1e-6f64)
            {
                let mut stop_phase: GuidancePhase = (if out.reached_taem
                    as ::core::ffi::c_int != 0
                {
                    PHASE_TAEM as ::core::ffi::c_int
                } else {
                    PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                }) as GuidancePhase;
                let mut tp: TrajectoryPoint = TrajectoryPoint {
                    ut: state.ut,
                    latitude: geo.latitude,
                    longitude: geo.longitude,
                    altitude: geo.altitude,
                    speed: at.speed,
                    phase: stop_phase,
                    kind: TRAJ_PLANNED,
                };
                trajectory_append(&raw mut out.trajectory, tp);
            }
            break;
        } else {
            let mut phase: GuidancePhase = (if geo.altitude > entry_alt {
                PHASE_COAST as ::core::ffi::c_int
            } else {
                PHASE_ENTRY_ENERGY as ::core::ffi::c_int
            }) as GuidancePhase;
            if include as ::core::ffi::c_int != 0
                && state.ut - lastsample
                    >= 6 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                let mut tp_0: TrajectoryPoint = TrajectoryPoint {
                    ut: state.ut,
                    latitude: geo.latitude,
                    longitude: geo.longitude,
                    altitude: geo.altitude,
                    speed: at.speed,
                    phase: phase,
                    kind: TRAJ_PLANNED,
                };
                trajectory_append(&raw mut out.trajectory, tp_0);
                lastsample = state.ut;
            }
            if geo.altitude
                <= fmax(
                    (*site).altitude,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                ) && vs < 0 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                break;
            }
            if out.reached_taem as ::core::ffi::c_int != 0
                && range < 500 as ::core::ffi::c_int as ::core::ffi::c_double
                && geo.altitude < 1000 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                break;
            }
            if vmag(state.position) < (*p).radius * 0.8f64
                || vmag(state.position).is_finite() as i32 == 0
            {
                break;
            }
            let mut frac: ::core::ffi::c_double = clampd(
                geo.altitude
                    / fmax(
                        1 as ::core::ffi::c_int as ::core::ffi::c_double,
                        (*p).atmosphere_depth,
                    ),
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            let mut base_dt: ::core::ffi::c_double = if geo.altitude
                > (*p).atmosphere_depth
            {
                8 as ::core::ffi::c_int as ::core::ffi::c_double
            } else {
                clampd(0.25f64 + frac * 1.55f64, 0.25f64, 1.8f64)
            };
            let mut dt: ::core::ffi::c_double = predictor_step_size(
                &raw mut at,
                p,
                v,
                set,
                geo.altitude,
                vs,
                entry_alt,
                out.entered_atmosphere,
                false_0 != 0,
                g.limiter.rate,
                base_dt,
            );
            let mut aoa: ::core::ffi::c_double = ::core::f32::NAN
                as ::core::ffi::c_double;
            let mut local_0: AerodynamicModel = aero_at(env0, at.mach);
            let mut bank: ::core::ffi::c_double = 0 as ::core::ffi::c_int
                as ::core::ffi::c_double;
            let mut target_aoa: ::core::ffi::c_double = 0 as ::core::ffi::c_int
                as ::core::ffi::c_double;
            if !out.entered_atmosphere {
                let mut regime: FlightControlRegime = flight_control_regime(
                    at.mach,
                    at.speed,
                    at.q,
                    (*v).minimum_safe_speed,
                );
                let mut rate: ::core::ffi::c_double = (*set).entry_roll_rate
                    * fmax(0.55f64, regime.command_rate_scale);
                let mut acc: ::core::ffi::c_double = (*set).entry_roll_acceleration
                    * fmax(0.45f64, regime.command_accel_scale);
                bank = norm_signed_deg(
                    jerk_angle_update(
                        &raw mut g.limiter,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        rate,
                        acc,
                        dt,
                    ),
                );
                target_aoa = (*v).maximum_angle_of_attack;
            } else {
                bank = entry_bank(
                    &raw mut g,
                    state,
                    geo,
                    &raw mut at,
                    p,
                    local_0,
                    cal0,
                    v,
                    site,
                    set,
                    maxbank,
                    dt,
                    &raw mut aoa,
                );
                target_aoa = aoa;
            }
            predictor_attitude_step(
                &raw mut g,
                bank,
                target_aoa,
                &raw mut at,
                v,
                set,
                dt,
                (*cal0).physics,
            );
            if out.entered_atmosphere as ::core::ffi::c_int != 0 && !out.reached_taem {
                out.maximum_abs_angle_of_attack = fmax(
                    out.maximum_abs_angle_of_attack,
                    fabs(g.actual_aoa),
                );
            }
            ctx.bank = g.actual_bank;
            ctx.aoa = g.actual_aoa;
            ctx.ut = state.ut;
            ctx.airbrakes = predictor_speedbrake_state(
                &raw mut g,
                geo,
                &raw mut at,
                p,
                cal0,
                v,
                site,
                set,
                state.mass,
                false_0 != 0,
                dt,
            );
            ctx.brakes = false_0 != 0;
            ctx.gear = false_0 != 0;
            out.s_turn_max_bank = fmax(out.s_turn_max_bank, fabs(g.actual_bank));
            out.s_turn_reversals = g.reversals;
            if at.q > 1 as ::core::ffi::c_int as ::core::ffi::c_double {
                if at.direct_aero {
                    out.physics_observed_seconds += dt;
                } else {
                    out.physics_fallback_seconds += dt;
                }
            }
            state = rk4_aero(state, dt, &raw mut ctx);
            elapsed += dt;
        }
    }
    if force_control as ::core::ffi::c_int != 0 && !out.control_horizon_recorded {
        let mut geo_0: GeoPoint = predictor_geo_point(state.position, p, state.ut);
        let mut at_0: AtmosState = atmosphere(
            state.position,
            state.velocity,
            &raw mut ctx,
        );
        out.control_horizon_recorded = true_0 != 0;
        out.control_horizon_speed = at_0.speed;
        out.control_horizon_altitude = geo_0.altitude;
        out.control_horizon_range = great_circle_distance(geo_0, target, (*p).radius);
        out.control_horizon_specific_energy = rotating_specific_energy(
            geo_0.latitude,
            geo_0.altitude,
            at_0.speed,
            p,
        );
        let mut along_0: ::core::ffi::c_double = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        let mut cross_0: ::core::ffi::c_double = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        runway_coordinates(
            geo_0,
            target,
            (*site).runway_heading,
            (*p).radius,
            &raw mut along_0,
            &raw mut cross_0,
        );
        out.control_horizon_alignment_distance = -along_0
            - (*set).final_approach_distance;
    }
    out.has_first_s_turn_reversal = g.has_first_reversal;
    out.first_s_turn_reversal_is_final = g.first_reversal_final;
    out.first_s_turn_reversal_ut = g.first_reversal_ut;
    out.first_s_turn_reversal_range = g.first_reversal_range;
    out.first_s_turn_reversal_sign = g.first_reversal_sign;
    if out.minimum_entry_speed == DBL_MAX {
        out.minimum_entry_speed = ::core::f32::NAN as ::core::ffi::c_double;
    }
    if !out.reached_taem {
        let mut geo_1: GeoPoint = predictor_geo_point(state.position, p, state.ut);
        out.taem_distance = great_circle_distance(geo_1, target, (*p).radius);
        out.taem_range_error = if out.closest_distance.is_finite() as i32 != 0 {
            out.taem_distance - out.closest_distance
        } else {
            out.taem_distance
        };
        out.taem_speed = atmosphere(state.position, state.velocity, &raw mut ctx).speed;
        out.taem_energy_error = ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    out.final_state = state;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_entry_with_attitude(
    mut state: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut maxbank: ::core::ffi::c_double,
    mut initial_bank: ::core::ffi::c_double,
    mut initial_bank_rate: ::core::ffi::c_double,
    mut initial_aoa: ::core::ffi::c_double,
    mut initial_aoa_rate: ::core::ffi::c_double,
    mut initial_sign: ::core::ffi::c_double,
    mut initial_leg: ::core::ffi::c_double,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    return predictor_simulate_entry_core(
        state,
        p,
        aero,
        env,
        cal,
        v,
        site,
        set,
        maxbank,
        initial_bank,
        initial_bank_rate,
        initial_aoa,
        initial_aoa_rate,
        initial_sign,
        initial_leg,
        false_0 != 0,
        maxdur,
        include,
        false_0 != 0,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        false_0 != 0,
        false_0 != 0,
        ::core::ptr::null::<EntryControlPlan>(),
        ::core::ptr::null::<TaemInterfaceTarget>(),
    );
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_entry_with_rate(
    mut state: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut maxbank: ::core::ffi::c_double,
    mut initial_bank: ::core::ffi::c_double,
    mut initial_bank_rate: ::core::ffi::c_double,
    mut initial_sign: ::core::ffi::c_double,
    mut initial_leg: ::core::ffi::c_double,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    return predictor_simulate_entry_with_attitude(
        state,
        p,
        aero,
        env,
        cal,
        v,
        site,
        set,
        maxbank,
        initial_bank,
        initial_bank_rate,
        ::core::f32::NAN as ::core::ffi::c_double,
        ::core::f32::NAN as ::core::ffi::c_double,
        initial_sign,
        initial_leg,
        maxdur,
        include,
    );
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_entry_policy(
    mut state: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut bank_deg: ::core::ffi::c_double,
    mut aoa_deg: ::core::ffi::c_double,
    mut sign: ::core::ffi::c_double,
    mut maxdur: ::core::ffi::c_double,
) -> EntryPrediction {
    return predictor_simulate_entry_core(
        state,
        p,
        aero,
        env,
        cal,
        v,
        site,
        set,
        (*v).maximum_bank_angle,
        sign * bank_deg,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        aoa_deg,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        sign,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        false_0 != 0,
        maxdur,
        true_0 != 0,
        true_0 != 0,
        bank_deg,
        aoa_deg,
        maxdur,
        maxdur,
        false_0 != 0,
        false_0 != 0,
        ::core::ptr::null::<EntryControlPlan>(),
        ::core::ptr::null::<TaemInterfaceTarget>(),
    );
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_entry(
    mut state: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut v: *const VehicleProfile,
    mut site: *const LandingSite,
    mut set: *const GuidanceSettings,
    mut maxbank: ::core::ffi::c_double,
    mut initial_bank: ::core::ffi::c_double,
    mut initial_sign: ::core::ffi::c_double,
    mut initial_leg: ::core::ffi::c_double,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    return predictor_simulate_entry_with_rate(
        state,
        p,
        aero,
        env,
        cal,
        v,
        site,
        set,
        maxbank,
        initial_bank,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        initial_sign,
        initial_leg,
        maxdur,
        include,
    );
}
unsafe extern "C" fn predictor_normalized_upper_violation(
    mut value: ::core::ffi::c_double,
    mut limit: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if value.is_finite() as i32 == 0 || limit.is_finite() as i32 == 0
        || !(limit > 0.0f64)
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return fmax(0.0f64, value / limit - 1.0f64);
}
pub const GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS: ::core::ffi::c_double = 0.08f64;
unsafe extern "C" fn guidance_shadow_telemetry(
    mut base: *const Telemetry,
    mut state: VehicleState,
    mut response: *mut PredictorGuidance,
    mut ctx: *mut AeroContext,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut out_at: *mut AtmosState,
) -> Telemetry {
    let mut t: Telemetry = Telemetry {
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
    };
    memset(
        &raw mut t as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Telemetry>() as size_t,
    );
    if !base.is_null() {
        t = *base;
    }
    (*ctx).ut = state.ut;
    (*ctx).mass = state.mass;
    (*ctx).bank = (*response).actual_bank;
    (*ctx).aoa = (*response).actual_aoa;
    let mut at: AtmosState = atmosphere(state.position, state.velocity, ctx);
    if !out_at.is_null() {
        *out_at = at;
    }
    let mut geo: GeoPoint = predictor_geo_point(state.position, p, state.ut);
    let mut target: GeoPoint = GeoPoint {
        latitude: (*cfg).site.latitude,
        longitude: (*cfg).site.longitude,
        altitude: (*cfg).site.altitude,
    };
    let mut up: Vector3 = vnorm(
        state.position,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut vs: ::core::ffi::c_double = vdot(at.air_velocity, up);
    let mut hs: ::core::ffi::c_double = vmag(vproject_plane(at.air_velocity, up));
    let mut fpa: ::core::ffi::c_double = atan2(vs, fmax(hs, 0.1f64)) * RAD2DEG;
    let mut fallback: ::core::ffi::c_double = initial_bearing(geo, target);
    let mut course: ::core::ffi::c_double = surface_course(
        state.position,
        state.velocity,
        planet_rotation_vector(p),
        (*p).north_axis,
        fallback,
    );
    let mut along: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut cross: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    runway_coordinates(
        geo,
        target,
        (*cfg).site.runway_heading,
        (*p).radius,
        &raw mut along,
        &raw mut cross,
    );
    let mut turn_rate: ::core::ffi::c_double = if at.speed
        > 1 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        at.lift_accel * sin((*response).actual_bank * DEG2RAD) / at.speed * RAD2DEG
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    t.ut = state.ut;
    t.latitude = geo.latitude;
    t.longitude = geo.longitude;
    t.mean_altitude = geo.altitude;
    t.radar_altitude = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        geo.altitude - (*cfg).site.altitude,
    );
    t.vertical_speed = vs;
    t.horizontal_speed = hs;
    t.surface_speed = hs;
    t.true_air_speed = at.speed;
    t.atmospheric_density = at.density;
    t.speed_of_sound = at.speed_of_sound;
    t.mach = at.mach;
    t.heading = course;
    t.ground_track_heading = course;
    t.course_to_site_error = norm_signed_deg(fallback - course);
    t.pitch = fpa + (*response).actual_aoa;
    t.roll = (*response).actual_bank;
    t.pitch_rate = (*response).actual_aoa_rate;
    t.roll_rate = (*response).actual_bank_rate;
    t.heading_rate = turn_rate;
    t.has_angle_of_attack_rate = true_0 != 0;
    t.angle_of_attack_rate = (*response).actual_aoa_rate;
    t.has_body_pitch_rate = false_0 != 0;
    t.has_body_roll_rate = false_0 != 0;
    t.has_body_yaw_rate = false_0 != 0;
    t.has_course_rate = true_0 != 0;
    t.body_pitch_rate = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.body_roll_rate = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.body_yaw_rate = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.course_rate = turn_rate;
    t.autopilot_error = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.has_command_pitch_error = false_0 != 0;
    t.has_command_roll_error = false_0 != 0;
    t.has_command_heading_error = false_0 != 0;
    t.angle_of_attack = (*response).actual_aoa;
    t.sideslip = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.dynamic_pressure = at.q;
    t.static_pressure = planet_atmospheric_pressure(p, geo.altitude);
    t.g_force = at.non_gravity / fmax(planet_surface_gravity(p), 0.1f64);
    t.stall_fraction = vessel_physics_conservative_stall_fraction(
        t.true_air_speed,
        t.angle_of_attack,
        t.dynamic_pressure,
        &raw const (*cfg).vehicle,
    );
    t.stall_fraction_is_measured = false_0 != 0;
    if !base.is_null() && (*base).stall_fraction_is_measured as ::core::ffi::c_int != 0
        && (*response).actual_aoa >= (*base).angle_of_attack - 1.0f64
    {
        t.stall_fraction = fmax(t.stall_fraction, (*base).stall_fraction);
        t.stall_fraction_is_measured = true_0 != 0;
    }
    t.lift_force = fabs(at.lift_accel)
        * fmax(state.mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double);
    t.drag_force = fabs(at.drag_accel)
        * fmax(state.mass, 1 as ::core::ffi::c_int as ::core::ffi::c_double);
    t.has_force_vectors = false_0 != 0;
    t.physics_sample_valid = false_0 != 0;
    t.mass = state.mass;
    t.available_thrust = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.current_thrust = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.throttle = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.has_controls = false_0 != 0;
    t.control_pitch = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.control_roll = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.control_yaw = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    t.gear = (*ctx).gear;
    t.brakes = (*ctx).brakes;
    t.has_airbrakes = true_0 != 0;
    t.airbrakes = (*ctx).airbrakes != 0 as ::core::ffi::c_int;
    snprintf(
        &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"flying\0" as *const u8 as *const ::core::ffi::c_char,
    );
    t.range_to_site = great_circle_distance(geo, target, (*p).radius);
    t.bearing_to_site = fallback;
    t.heading_error = norm_signed_deg(fallback - course);
    t.runway_along_track = along;
    t.runway_cross_track = cross;
    t.flight_path_angle = fpa;
    t.aerodynamic_confidence = clampd(
        fmax(
            at.physics_confidence,
            if !base.is_null() {
                (*base).aerodynamic_confidence
            } else {
                0 as ::core::ffi::c_int as ::core::ffi::c_double
            },
        ),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    t.physics_confidence = at.physics_confidence;
    t.physics_certified_uncertainty = at.physics_uncertainty;
    if !(*ctx).c.is_null() {
        t.trajectory_density_scale = (*(*ctx).c).density_scale;
        t.trajectory_drag_scale = (*(*ctx).c).drag_scale;
        t.trajectory_lift_scale = (*(*ctx).c).lift_scale;
        t.bank_effectiveness = (*(*ctx).c).bank_effectiveness;
        t.trajectory_calibration_confidence = (*(*ctx).c).confidence;
        t.trajectory_altitude_residual = (*(*ctx).c).altitude_residual;
        t.trajectory_speed_residual = (*(*ctx).c).speed_residual;
        t.trajectory_range_residual = (*(*ctx).c).range_residual;
    }
    t.energy_excess_range = ::core::f32::NAN as ::core::ffi::c_double;
    return t;
}
unsafe extern "C" fn terminal_shadow_survives(
    mut pr: *const EntryPrediction,
    mut outcome: *const TerminalShadowOutcome,
    mut v: *const VehicleProfile,
) -> bool {
    if pr.is_null() || outcome.is_null() || v.is_null()
        || (*outcome).aborted as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    if (*pr).peak_dynamic_pressure > (*v).maximum_dynamic_pressure
        || (*pr).peak_g_load > (*v).maximum_g_load
    {
        return false_0 != 0;
    }
    if (*pr).minimum_entry_speed.is_finite() as i32 != 0
        && (*pr).minimum_entry_speed < (*v).minimum_safe_speed
    {
        return false_0 != 0;
    }
    return (*outcome).final_approach_captured as ::core::ffi::c_int != 0
        || (*outcome).terminal_path_committed as ::core::ffi::c_int != 0
        || (*outcome).terminal_policy_feasible as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn shadow_taem_range_target(
    mut shadow: *const GuidanceMachine,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
) -> ::core::ffi::c_double {
    let mut target: ::core::ffi::c_double = if !shadow.is_null() {
        (*shadow).entry_handoff_target_range
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    return if target.is_finite() as i32 != 0 && target > 0.0f64 {
        target
    } else {
        entry_taem_range_target(p, &raw const (*cfg).guidance)
    };
}
unsafe extern "C" fn predictor_simulate_guidance_shadow_core(
    mut state: VehicleState,
    mut telemetry: *const Telemetry,
    mut guidance: *const GuidanceMachine,
    mut plan: *const DeorbitPlan,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env0: *const AerodynamicEnvelope,
    mut cal0: *const TrajectoryCalibrationModel,
    mut cfg: *const LandingConfiguration,
    mut maxdur: ::core::ffi::c_double,
    mut include_trajectory: bool,
    mut require_terminal_start: bool,
    mut stop_at_taem: bool,
    mut terminal_model: *const TerminalModel,
    mut outcome: *mut TerminalShadowOutcome,
) -> EntryPrediction {
    let mut out: EntryPrediction = EntryPrediction {
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        taem_distance: 0.,
        taem_range_error: 0.,
        closest_distance: 0.,
        entered_atmosphere: false,
        reached_taem: false,
        taem_ownership_boundary_missed: false,
        taem_dynamic_interface_captured: false,
        taem_dynamic_interface_target_valid: false,
        entry_range: 0.,
        entry_flight_path_angle: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        taem_speed: 0.,
        taem_flight_path_angle: 0.,
        taem_altitude: 0.,
        taem_desired_altitude: 0.,
        taem_along_track: 0.,
        taem_cross_track: 0.,
        taem_course: 0.,
        mm304_gate_recorded: false,
        mm304_gate_range: 0.,
        mm304_gate_along_track: 0.,
        mm304_gate_cross_track: 0.,
        mm304_gate_course: 0.,
        mm304_gate_altitude: 0.,
        mm304_gate_speed: 0.,
        mm304_gate_flight_path_angle: 0.,
        taem_energy_error: 0.,
        taem_interface_error: 0.,
        taem_interface_energy_margin: 0.,
        peak_dynamic_pressure: 0.,
        peak_g_load: 0.,
        minimum_entry_speed: 0.,
        maximum_abs_angle_of_attack: 0.,
        minimum_bank_control_margin: 0.,
        minimum_aoa_control_margin: 0.,
        control_horizon_recorded: false,
        control_horizon_speed: 0.,
        control_horizon_altitude: 0.,
        control_horizon_range: 0.,
        control_horizon_specific_energy: 0.,
        control_horizon_alignment_distance: 0.,
        has_first_s_turn_reversal: false,
        first_s_turn_reversal_is_final: false,
        first_s_turn_reversal_ut: 0.,
        first_s_turn_reversal_range: 0.,
        first_s_turn_reversal_sign: 0.,
        s_turn_reversals: 0,
        s_turn_max_bank: 0.,
        final_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        physics_observed_seconds: 0.,
        physics_fallback_seconds: 0.,
        shadow_guidance_used: false,
        shadow_terminal_prediction_valid: false,
        shadow_terminal_policy_feasible: false,
        shadow_terminal_path_committed: false,
        shadow_final_approach_captured: false,
        shadow_aborted: false,
        terminal_survivability_stress_score: 0.,
        physics_relative_uncertainty: 0.,
        uncertainty_scenarios: 0,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
    trajectory_init(&raw mut out.trajectory);
    out.closest_distance = DBL_MAX;
    out.taem_distance = DBL_MAX;
    out.taem_altitude = ::core::f32::NAN as ::core::ffi::c_double;
    out.taem_desired_altitude = ::core::f32::NAN as ::core::ffi::c_double;
    out.minimum_entry_speed = DBL_MAX;
    out.minimum_bank_control_margin = ::core::f32::INFINITY as ::core::ffi::c_double;
    out.minimum_aoa_control_margin = ::core::f32::INFINITY as ::core::ffi::c_double;
    if !outcome.is_null() {
        memset(
            outcome as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<TerminalShadowOutcome>() as size_t,
        );
    }
    if telemetry.is_null() || guidance.is_null() || p.is_null() || cfg.is_null()
        || !(*guidance).automation_engaged
    {
        return out;
    }
    if require_terminal_start as ::core::ffi::c_int != 0
        && !taem_exec_owns_vehicle(&raw const (*guidance).taem_exec)
    {
        return out;
    }
    let mut uniform: AerodynamicEnvelope = AerodynamicEnvelope {
        regimes: [AerodynamicModel {
            lift_to_drag: 0.,
            ballistic_coefficient: 0.,
            confidence: 0.,
        }; 4],
    };
    if env0.is_null() {
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < 4 as ::core::ffi::c_int {
            uniform.regimes[i as usize] = aero;
            i += 1;
        }
        env0 = &raw mut uniform;
    }
    let mut dc: TrajectoryCalibrationModel = TrajectoryCalibrationModel {
        physics: ::core::ptr::null::<VesselPhysicsModel>(),
        density_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        drag_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        lift_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        bank_effectiveness: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_of_sound: 340 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_of_sound_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        stress_drag_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        stress_lift_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
        forecast_drag_scale: 0.,
        forecast_lift_scale: 0.,
        forecast_exact_policy_bank: false,
        altitude_residual: 0.,
        speed_residual: 0.,
        range_residual: 0.,
        confidence: 0.05f64,
        accepted_samples: 0,
    };
    if cal0.is_null() {
        cal0 = &raw mut dc;
    }
    let mut shadow: GuidanceMachine = *guidance;
    shadow.diagnostic_shadow = true_0 != 0;
    shadow.diagnostic_stop_at_taem = stop_at_taem;
    shadow.entry_planning_deferred = true_0 != 0;
    shadow.has_previous_ut = true_0 != 0;
    shadow.previous_ut = state.ut - 0.1f64;
    guidance_set_entry_predictor_models(&raw mut shadow, env0, cal0);
    let mut response: PredictorGuidance = PredictorGuidance {
        sign: 0.,
        leg_elapsed: 0.,
        established: false,
        entry_loaded: false,
        configured_leg_dwell_satisfied: false,
        reversals: 0,
        limiter: JerkLimiter {
            value: 0.,
            rate: 0.,
            has_value: false,
        },
        has_first_reversal: false,
        first_reversal_final: false,
        final_heading_lock: false,
        first_reversal_ut: 0.,
        first_reversal_range: 0.,
        first_reversal_sign: 0.,
        replay_event: false,
        replay_event_final: false,
        replay_event_ut: 0.,
        replay_event_sign: 0.,
        entry_score: 0.,
        has_entry_score: false,
        actual_bank: 0.,
        actual_bank_rate: 0.,
        actual_aoa: 0.,
        actual_aoa_rate: 0.,
        entry_speed: 0.,
        has_interface_target: false,
        enforce_terminal_delivery_budget: false,
        interface_target: TaemInterfaceTarget {
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
        force_control: false,
        forced_until_ut: 0.,
        control_horizon_ut: 0.,
        forced_bank: 0.,
        forced_aoa: 0.,
        speedbrake: SpeedbrakeController {
            integral: 0.,
            deployed: false,
            initialized: false,
        },
    };
    memset(
        &raw mut response as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PredictorGuidance>() as size_t,
    );
    response.actual_bank = norm_signed_deg((*telemetry).roll);
    response.actual_bank_rate = if (*telemetry).roll_rate.is_finite() as i32 != 0 {
        (*telemetry).roll_rate
    } else {
        0.0f64
    };
    response.actual_aoa = if (*telemetry).angle_of_attack.is_finite() as i32 != 0 {
        (*telemetry).angle_of_attack
    } else {
        (*cfg).vehicle.entry_angle_of_attack
    };
    response.actual_aoa_rate = if (*telemetry).has_angle_of_attack_rate
        as ::core::ffi::c_int != 0
        && (*telemetry).angle_of_attack_rate.is_finite() as i32 != 0
    {
        (*telemetry).angle_of_attack_rate
    } else {
        0.0f64
    };
    response.limiter.has_value = true_0 != 0;
    response.limiter.value = norm_deg(response.actual_bank);
    let mut initial_gear: bool = (*telemetry).gear;
    let mut initial_brakes: bool = (*telemetry).brakes;
    let mut initial_airbrakes: ::core::ffi::c_int = if (*telemetry).has_airbrakes
        as ::core::ffi::c_int != 0 && (*telemetry).airbrakes as ::core::ffi::c_int != 0
    {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
    let mut ctx: AeroContext = AeroContext {
        p: p,
        e: env0,
        c: cal0,
        v: &raw const (*cfg).vehicle,
        entry_table: ::core::ptr::null::<TerminalModel>(),
        bank: response.actual_bank,
        aoa: response.actual_aoa,
        ut: state.ut,
        mass: state.mass,
        gear: initial_gear,
        brakes: initial_brakes,
        airbrakes: initial_airbrakes,
    };
    let mut sts_entry_table: bool = (*cfg).guidance.taem_energy_law as ::core::ffi::c_int
        != 0
        && strcmp(
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        && terminal_model_policy_matches(
            terminal_model,
            &raw const (*cfg).site,
            &raw const (*cfg).vehicle,
            &raw const (*cfg).guidance,
        ) as ::core::ffi::c_int != 0
        && terminal_model_planet_matches(terminal_model, p) as ::core::ffi::c_int != 0;
    let mut target: GeoPoint = GeoPoint {
        latitude: (*cfg).site.latitude,
        longitude: (*cfg).site.longitude,
        altitude: (*cfg).site.altitude,
    };
    let mut elapsed: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut lastsample: ::core::ffi::c_double = -DBL_MAX;
    out.taem_interface_error = ::core::f32::INFINITY as ::core::ffi::c_double;
    out.taem_interface_energy_margin = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut entry_witness_trace_env: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TRACE_ENTRY_WITNESS\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut entry_witness_trace: bool = !entry_witness_trace_env.is_null()
        && strcmp(
            entry_witness_trace_env,
            b"1\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 as ::core::ffi::c_int;
    let mut entry_witness_phase_traced: bool = false_0 != 0;
    let mut entry_witness_history: [EntryWitnessTracePoint; 8] = [EntryWitnessTracePoint {
        ut: 0.,
        altitude: 0.,
        speed: 0.,
        mach: 0.,
        fpa: 0.,
        range: 0.,
        along: 0.,
        cross: 0.,
        course_to_site_error: 0.,
        commanded_bank: 0.,
        achieved_bank: 0.,
        commanded_aoa: 0.,
        achieved_aoa: 0.,
        energy_margin: 0.,
        phase: 0,
        mm305_veto: 0,
        mm305_ready: false,
    }; 8];
    let mut entry_witness_history_count: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    let mut entry_witness_history_next: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    let mut next_entry_witness_history_ut: ::core::ffi::c_double = state.ut;
    let mut trace_terminal_witness: bool = !getenv(
            b"KSP_LANDER_TRACE_TERMINAL_WITNESS\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null();
    let mut next_terminal_trace_ut: ::core::ffi::c_double = state.ut;
    let mut entry_alt: ::core::ffi::c_double = (*p).atmosphere_depth
        - (*cfg).guidance.entry_interface_altitude_margin;
    out.entered_atmosphere = (*telemetry).mean_altitude <= entry_alt
        && (*telemetry).vertical_speed < 0.0f64;
    let mut initial_terminal_owned: bool = taem_exec_owns_vehicle(
        &raw mut shadow.taem_exec,
    );
    let mut initial_degraded_safety_handoff: bool = initial_terminal_owned
        as ::core::ffi::c_int != 0
        && shadow.taem_safety_handoff as ::core::ffi::c_int != 0
        && !shadow.taem_interface_captured;
    out.reached_taem = initial_terminal_owned as ::core::ffi::c_int != 0
        && !(stop_at_taem as ::core::ffi::c_int != 0
            && initial_degraded_safety_handoff as ::core::ffi::c_int != 0);
    out.taem_ownership_boundary_missed = stop_at_taem as ::core::ffi::c_int != 0
        && initial_degraded_safety_handoff as ::core::ffi::c_int != 0;
    out.taem_dynamic_interface_captured = shadow.taem_interface_captured;
    out.taem_dynamic_interface_target_valid = shadow.taem_interface_target.valid;
    if out.entered_atmosphere {
        let mut initial_geo: GeoPoint = predictor_geo_point(state.position, p, state.ut);
        let mut initial_target: GeoPoint = GeoPoint {
            latitude: (*cfg).site.latitude,
            longitude: (*cfg).site.longitude,
            altitude: (*cfg).site.altitude,
        };
        out.entry_range = great_circle_distance(
            initial_geo,
            initial_target,
            (*p).radius,
        );
        out.entry_flight_path_angle = (*telemetry).flight_path_angle;
        out.entry_speed = (*telemetry).true_air_speed;
        out.entry_course_error = (*telemetry).course_to_site_error;
    }
    out.shadow_guidance_used = true_0 != 0;
    while elapsed < maxdur {
        ctx.entry_table = if sts_entry_table as ::core::ffi::c_int != 0
            && !taem_exec_owns_vehicle(&raw mut shadow.taem_exec) && !ctx.gear
            && ctx.airbrakes == 0
        {
            terminal_model
        } else {
            ::core::ptr::null::<TerminalModel>()
        };
        let mut at: AtmosState = AtmosState {
            density: 0.,
            air_velocity: Vector3 { x: 0., y: 0., z: 0. },
            speed: 0.,
            speed_of_sound: 0.,
            mach: 0.,
            q: 0.,
            drag_accel: 0.,
            lift_accel: 0.,
            non_gravity: 0.,
            specific_force: Vector3 { x: 0., y: 0., z: 0. },
            physics_confidence: 0.,
            physics_uncertainty: 0.,
            direct_aero: false,
        };
        let mut synthetic: Telemetry = guidance_shadow_telemetry(
            telemetry,
            state,
            &raw mut response,
            &raw mut ctx,
            p,
            cfg,
            &raw mut at,
        );
        let mut geo: GeoPoint = GeoPoint {
            latitude: synthetic.latitude,
            longitude: synthetic.longitude,
            altitude: synthetic.mean_altitude,
        };
        let mut range: ::core::ffi::c_double = synthetic.range_to_site;
        if !out.entered_atmosphere && geo.altitude <= entry_alt
            && synthetic.vertical_speed < 0.0f64
        {
            out.entered_atmosphere = true_0 != 0;
            out.entry_range = range;
            out.entry_flight_path_angle = synthetic.flight_path_angle;
            out.entry_speed = at.speed;
            out.entry_course_error = synthetic.course_to_site_error;
        }
        if out.entered_atmosphere as ::core::ffi::c_int != 0 && !out.mm304_gate_recorded
            && geo.altitude <= 50000.0f64 && synthetic.vertical_speed < 0.0f64
        {
            out.mm304_gate_recorded = true_0 != 0;
            out.mm304_gate_range = range;
            out.mm304_gate_along_track = synthetic.runway_along_track;
            out.mm304_gate_cross_track = synthetic.runway_cross_track;
            out.mm304_gate_course = synthetic.ground_track_heading;
            out.mm304_gate_altitude = geo.altitude;
            out.mm304_gate_speed = at.speed;
            out.mm304_gate_flight_path_angle = synthetic.flight_path_angle;
        }
        if out.entered_atmosphere {
            out.closest_distance = fmin(out.closest_distance, range);
        }
        if out.entered_atmosphere as ::core::ffi::c_int != 0 && !out.reached_taem {
            out.minimum_entry_speed = fmin(out.minimum_entry_speed, at.speed);
        }
        out.maximum_abs_angle_of_attack = fmax(
            out.maximum_abs_angle_of_attack,
            fabs(response.actual_aoa),
        );
        out.peak_dynamic_pressure = fmax(out.peak_dynamic_pressure, at.q);
        out.peak_g_load = fmax(out.peak_g_load, synthetic.g_force);
        if at.direct_aero {
            out.physics_observed_seconds += fmin(1.0f64, maxdur - elapsed);
        } else {
            out.physics_fallback_seconds += fmin(1.0f64, maxdur - elapsed);
        }
        if !out.reached_taem {
            out.taem_distance = range;
            out.taem_range_error = range
                - shadow_taem_range_target(&raw mut shadow, p, cfg);
            out.taem_speed = at.speed;
            out.taem_flight_path_angle = synthetic.flight_path_angle;
            out.taem_altitude = geo.altitude;
            out.taem_energy_error = entry_remaining_specific_energy(
                geo.latitude,
                geo.altitude,
                at.speed,
                (*cfg).site.latitude,
                (*cfg).guidance.taem_interface_altitude,
                entry_taem_speed_target(
                    &raw const (*cfg).vehicle,
                    &raw const (*cfg).guidance,
                    p,
                ),
                p,
            );
        } else if elapsed <= 1e-9f64 {
            out.taem_distance = range;
            out.taem_range_error = range
                - shadow_taem_range_target(&raw mut shadow, p, cfg);
            out.taem_speed = at.speed;
            out.taem_flight_path_angle = synthetic.flight_path_angle;
            out.taem_altitude = geo.altitude;
            out.taem_energy_error = entry_remaining_specific_energy(
                geo.latitude,
                geo.altitude,
                at.speed,
                (*cfg).site.latitude,
                (*cfg).guidance.taem_interface_altitude,
                entry_taem_speed_target(
                    &raw const (*cfg).vehicle,
                    &raw const (*cfg).guidance,
                    p,
                ),
                p,
            );
        }
        let mut previous_reversals: ::core::ffi::c_uint = shadow.entry_control_reversals;
        let mut r: GuidanceResult = guidance_update_with_terminal_model(
            &raw mut shadow,
            &raw mut synthetic,
            &raw mut state,
            plan,
            p,
            aero,
            cfg,
            terminal_model as *const TerminalModel,
        );
        if trace_terminal_witness as ::core::ffi::c_int != 0
            && taem_exec_owns_vehicle(&raw mut shadow.taem_exec) as ::core::ffi::c_int
                != 0 && state.ut >= next_terminal_trace_ut
        {
            fprintf(
                stderr,
                b"TERMINAL_WITNESS ut=%.1f phase=%d alt=%.0f V=%.1f M=%.2f range=%.0f along=%+.0f cross=%+.0f hdg=%.1f siteErr=%+.1f fpa=%.1f bank=%.1f/%.1f aoa=%.1f/%.1f leg=%d go=%.0f req=%.0f k=%.3f route=%d final=%d status=%.130s\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                state.ut,
                r.phase as ::core::ffi::c_int,
                geo.altitude,
                at.speed,
                at.mach,
                range,
                synthetic.runway_along_track,
                synthetic.runway_cross_track,
                synthetic.ground_track_heading,
                synthetic.course_to_site_error,
                synthetic.flight_path_angle,
                response.actual_bank,
                r.command.target_roll,
                response.actual_aoa,
                if r.command.has_target_aoa as ::core::ffi::c_int != 0 {
                    r.command.target_aoa
                } else {
                    response.actual_aoa
                },
                shadow.taem_energy_law.leg,
                shadow.taem_energy_law.s_go,
                shadow.taem_energy_law.s_req,
                shadow.taem_energy_law.k,
                if shadow.terminal_path_committed as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if shadow.final_approach_captured as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                &raw mut r.status as *mut ::core::ffi::c_char,
            );
            next_terminal_trace_ut = state.ut + 12.0f64;
        }
        if entry_witness_trace as ::core::ffi::c_int != 0
            && state.ut + 1e-6f64 >= next_entry_witness_history_ut
        {
            let mut admission: TaemInterfaceCapture = entry_mm305_admission_envelope(
                &raw mut shadow,
                &raw mut synthetic,
                synthetic.ground_track_heading,
                p,
                cfg,
            );
            let mut command_aoa: ::core::ffi::c_double = if r.command.has_target_aoa
                as ::core::ffi::c_int != 0
            {
                r.command.target_aoa
            } else {
                response.actual_aoa
            };
            entry_witness_history[entry_witness_history_next as usize] = EntryWitnessTracePoint {
                ut: state.ut,
                altitude: geo.altitude,
                speed: at.speed,
                mach: at.mach,
                fpa: synthetic.flight_path_angle,
                range: range,
                along: synthetic.runway_along_track,
                cross: synthetic.runway_cross_track,
                course_to_site_error: synthetic.course_to_site_error,
                commanded_bank: if r.command.target_roll.is_finite() as i32 != 0 {
                    norm_signed_deg(r.command.target_roll)
                } else {
                    response.actual_bank
                },
                achieved_bank: response.actual_bank,
                commanded_aoa: command_aoa,
                achieved_aoa: response.actual_aoa,
                energy_margin: shadow.entry_feedback.energy_margin,
                phase: r.phase as ::core::ffi::c_int,
                mm305_veto: admission.veto,
                mm305_ready: admission.ready,
            };
            entry_witness_history_next = entry_witness_history_next
                .wrapping_add(1 as ::core::ffi::c_uint)
                .wrapping_rem(8 as ::core::ffi::c_uint);
            if entry_witness_history_count < 8 as ::core::ffi::c_uint {
                entry_witness_history_count = entry_witness_history_count
                    .wrapping_add(1);
            }
            next_entry_witness_history_ut = state.ut + 45.0f64;
        }
        if entry_witness_trace as ::core::ffi::c_int != 0 && !entry_witness_phase_traced
            && r.phase as ::core::ffi::c_uint
                == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            fprintf(
                stderr,
                b"ENTRY_WITNESS first-entry ut=%.2f h=%.0f V=%.0f margin=%+.0f D=%.3f Dref=%.3f bank=%+.1f cross=%+.0f planning=%d\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                state.ut,
                geo.altitude,
                at.speed,
                shadow.entry_feedback.energy_margin,
                shadow.entry_feedback.measured_drag_accel,
                shadow.entry_feedback.reference_drag_accel,
                shadow.entry_feedback.commanded_bank_deg,
                shadow.entry_feedback.crossrange_error,
                if shadow.entry_planning_needed as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
            );
            entry_witness_phase_traced = true_0 != 0;
        }
        out.taem_dynamic_interface_target_valid = false_0 != 0;
        if !out.has_first_s_turn_reversal
            && shadow.entry_control_reversals != previous_reversals
        {
            out.has_first_s_turn_reversal = true_0 != 0;
            out.first_s_turn_reversal_ut = state.ut;
            out.first_s_turn_reversal_range = range;
            out.first_s_turn_reversal_sign = shadow.s_turn_sign;
            out.first_s_turn_reversal_is_final = false_0 != 0;
            if entry_witness_trace {
                fprintf(
                    stderr,
                    b"ENTRY_WITNESS reversal ut=%.2f V=%.1f h=%.0f cross=%.0f side=%+.0f count=%u\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    state.ut,
                    at.speed,
                    geo.altitude,
                    synthetic.runway_cross_track,
                    shadow.s_turn_sign,
                    shadow.entry_control_reversals,
                );
            }
        }
        out.s_turn_reversals = shadow
            .entry_control_reversals
            .wrapping_sub((*guidance).entry_control_reversals);
        let mut terminal_owned_now: bool = taem_exec_owns_vehicle(
            &raw mut shadow.taem_exec,
        );
        let mut noninterface_terminal_ownership: bool = terminal_owned_now
            as ::core::ffi::c_int != 0 && !shadow.taem_interface_captured;
        if stop_at_taem as ::core::ffi::c_int != 0
            && noninterface_terminal_ownership as ::core::ffi::c_int != 0
        {
            out.taem_ownership_boundary_missed = true_0 != 0;
        }
        if !out.reached_taem && terminal_owned_now as ::core::ffi::c_int != 0
            && !noninterface_terminal_ownership
        {
            out.reached_taem = true_0 != 0;
            out.taem_distance = range;
            let mut forced_boundary: bool = shadow.entry_exec.last_transition_reason
                as ::core::ffi::c_uint
                == ENTRY_TRANSITION_TAEM_FORCED_HANDOFF as ::core::ffi::c_int
                    as ::core::ffi::c_uint;
            let mut inlet: TaemInterfaceCapture = entry_mm305_admission_envelope(
                &raw mut shadow,
                &raw mut synthetic,
                synthetic.ground_track_heading,
                p,
                cfg,
            );
            let mut admissible_inlet: bool = inlet.valid as ::core::ffi::c_int != 0
                && inlet.ready as ::core::ffi::c_int != 0;
            if forced_boundary as ::core::ffi::c_int != 0 && !admissible_inlet {
                out.taem_ownership_boundary_missed = true_0 != 0;
            }
            out.taem_dynamic_interface_captured = shadow.taem_interface_captured
                as ::core::ffi::c_int != 0
                && admissible_inlet as ::core::ffi::c_int != 0;
            out.taem_dynamic_interface_target_valid = false_0 != 0;
            out.taem_interface_error = hypot(
                synthetic.runway_along_track - (*cfg).guidance.mm304_handoff_along_track,
                synthetic.runway_cross_track - (*cfg).guidance.mm304_handoff_cross_track,
            );
            out.taem_interface_energy_margin = inlet.energy_margin;
            if entry_witness_trace {
                fprintf(
                    stderr,
                    b"ENTRY_WITNESS physical-inlet ut=%.2f h=%.0f V=%.0f M=%.2f range=%.1fkm course=%.1f along=%.1fkm cross=%.1fkm ready=%d veto=0x%x energy=%+.0f J/kg Eavailable=%.0f Edrag=%.0f Euncertainty=%.0f path=%.1fkm turnRadius=%.1fkm forced=%d\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    state.ut,
                    geo.altitude,
                    at.speed,
                    at.mach,
                    range / 1000.0f64,
                    synthetic.ground_track_heading,
                    synthetic.runway_along_track / 1000.0f64,
                    synthetic.runway_cross_track / 1000.0f64,
                    if admissible_inlet as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                    inlet.veto,
                    inlet.energy_margin,
                    inlet.energy_available,
                    inlet.energy_drag_work,
                    inlet.energy_uncertainty,
                    inlet.path_length_m / 1000.0f64,
                    inlet.minimum_turn_radius_m / 1000.0f64,
                    if forced_boundary as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                );
            }
            out.taem_range_error = range
                - shadow_taem_range_target(&raw mut shadow, p, cfg);
            out.taem_speed = at.speed;
            out.taem_flight_path_angle = synthetic.flight_path_angle;
            out.taem_altitude = geo.altitude;
            out.taem_desired_altitude = (*cfg).guidance.taem_interface_altitude;
            out.taem_along_track = synthetic.runway_along_track;
            out.taem_cross_track = synthetic.runway_cross_track;
            out.taem_course = synthetic.ground_track_heading;
            out.taem_energy_error = entry_remaining_specific_energy(
                geo.latitude,
                geo.altitude,
                at.speed,
                (*cfg).site.latitude,
                (*cfg).guidance.taem_interface_altitude,
                entry_taem_speed_target(
                    &raw const (*cfg).vehicle,
                    &raw const (*cfg).guidance,
                    p,
                ),
                p,
            );
        }
        let mut policy_feasible: bool = shadow.taem_exec.terminal_evaluation.valid
            as ::core::ffi::c_int != 0
            && shadow.taem_exec.terminal_evaluation.feasible as ::core::ffi::c_int != 0;
        out.shadow_terminal_prediction_valid = shadow.mm305_route_committed
            as ::core::ffi::c_int != 0
            && shadow.mm305_route.valid as ::core::ffi::c_int != 0;
        out.shadow_terminal_policy_feasible = policy_feasible;
        out.shadow_terminal_path_committed = shadow.terminal_path_committed;
        out.shadow_final_approach_captured = shadow.final_approach_captured;
        out.shadow_aborted = shadow.aborted as ::core::ffi::c_int != 0
            || r.phase as ::core::ffi::c_uint
                == PHASE_ABORT as ::core::ffi::c_int as ::core::ffi::c_uint;
        if out.shadow_aborted as ::core::ffi::c_int != 0
            && entry_witness_trace as ::core::ffi::c_int != 0
        {
            fprintf(
                stderr,
                b"ENTRY_WITNESS shadow-abort ut=%.2f phase=%d h=%.0f V=%.1f mach=%.2f range=%.0f along=%.0f cross=%.0f course=%.1f status=\"%s\" warning=\"%s\" margin=%+.0f entryComplete=%d taemOwn=%d route=%d routeValid=%d evalValid=%d evalFeasible=%d\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                state.ut,
                r.phase as ::core::ffi::c_int,
                geo.altitude,
                at.speed,
                synthetic.mach,
                range,
                synthetic.runway_along_track,
                synthetic.runway_cross_track,
                synthetic.ground_track_heading,
                &raw mut r.status as *mut ::core::ffi::c_char,
                if r.has_warning as ::core::ffi::c_int != 0 {
                    &raw mut r.warning as *mut ::core::ffi::c_char
                        as *const ::core::ffi::c_char
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
                shadow.entry_feedback.energy_margin,
                if shadow.entry_exec.entry_complete as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if taem_exec_owns_vehicle(&raw mut shadow.taem_exec)
                    as ::core::ffi::c_int != 0
                {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if shadow.mm305_route_committed as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if shadow.mm305_route.valid as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if shadow.taem_exec.terminal_evaluation.valid as ::core::ffi::c_int != 0
                {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if shadow.taem_exec.terminal_evaluation.feasible as ::core::ffi::c_int
                    != 0
                {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
            );
            let mut first: ::core::ffi::c_uint = entry_witness_history_next
                .wrapping_add(8 as ::core::ffi::c_uint)
                .wrapping_sub(entry_witness_history_count)
                .wrapping_rem(8 as ::core::ffi::c_uint);
            let mut i_0: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
            while i_0 < entry_witness_history_count {
                let mut q: *const EntryWitnessTracePoint = (&raw mut entry_witness_history
                    as *mut EntryWitnessTracePoint)
                    .offset(
                        first.wrapping_add(i_0).wrapping_rem(8 as ::core::ffi::c_uint)
                            as isize,
                    ) as *mut EntryWitnessTracePoint;
                fprintf(
                    stderr,
                    b"ENTRY_WITNESS state ut=%.2f phase=%d h=%.0f V=%.1f M=%.2f fpa=%.2f range=%.0f along=%.0f cross=%.0f courseToSiteErr=%+.1f bank=%.1f/%.1f aoa=%.1f/%.1f entryFeedbackMargin=%+.0f mm305Ready=%d mm305Veto=0x%x\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*q).ut,
                    (*q).phase,
                    (*q).altitude,
                    (*q).speed,
                    (*q).mach,
                    (*q).fpa,
                    (*q).range,
                    (*q).along,
                    (*q).cross,
                    (*q).course_to_site_error,
                    (*q).commanded_bank,
                    (*q).achieved_bank,
                    (*q).commanded_aoa,
                    (*q).achieved_aoa,
                    (*q).energy_margin,
                    if (*q).mm305_ready as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                    (*q).mm305_veto,
                );
                i_0 = i_0.wrapping_add(1);
            }
        }
        let mut terminal_public_phase: bool = out.reached_taem as ::core::ffi::c_int != 0
            && r.phase as ::core::ffi::c_uint
                == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint;
        if include_trajectory as ::core::ffi::c_int != 0
            && (state.ut - lastsample >= 2.0f64
                || terminal_public_phase as ::core::ffi::c_int != 0)
        {
            let mut tp: TrajectoryPoint = TrajectoryPoint {
                ut: state.ut,
                latitude: geo.latitude,
                longitude: geo.longitude,
                altitude: geo.altitude,
                speed: at.speed,
                phase: r.phase,
                kind: TRAJ_PLANNED,
            };
            trajectory_append(&raw mut out.trajectory, tp);
            lastsample = state.ut;
        }
        if out.shadow_aborted as ::core::ffi::c_int != 0
            || stop_at_taem as ::core::ffi::c_int != 0
                && (out.taem_ownership_boundary_missed as ::core::ffi::c_int != 0
                    || terminal_public_phase as ::core::ffi::c_int != 0)
        {
            guidance_result_clear(&raw mut r);
            break;
        } else if geo.altitude
            <= (*cfg).site.altitude - 20 as ::core::ffi::c_int as ::core::ffi::c_double
            || at.speed < fmax(25.0f64, (*cfg).vehicle.touchdown_speed * 0.55f64)
        {
            guidance_result_clear(&raw mut r);
            break;
        } else {
            let mut target_bank: ::core::ffi::c_double = if r
                .command
                .target_roll
                .is_finite() as i32 != 0
            {
                norm_signed_deg(r.command.target_roll)
            } else {
                response.actual_bank
            };
            let mut target_aoa: ::core::ffi::c_double = response.actual_aoa;
            if r.command.use_inertial_direction as ::core::ffi::c_int != 0
                && vmag(r.command.inertial_direction) > 1e-9f64
            {
                let mut local_up: Vector3 = vnorm(
                    state.position,
                    v3(
                        1 as ::core::ffi::c_int as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ),
                );
                let mut desired_forward: Vector3 = vnorm(
                    r.command.inertial_direction,
                    state.velocity,
                );
                let mut air_velocity: Vector3 = vsub(
                    state.velocity,
                    vcross(planet_rotation_vector(p), state.position),
                );
                let mut air_forward: Vector3 = vnorm(air_velocity, desired_forward);
                let mut desired_fpa: ::core::ffi::c_double = atan2(
                    vdot(desired_forward, local_up),
                    fmax(vmag(vproject_plane(desired_forward, local_up)), 1e-9f64),
                ) * RAD2DEG;
                let mut air_fpa: ::core::ffi::c_double = atan2(
                    vdot(air_forward, local_up),
                    fmax(vmag(vproject_plane(air_forward, local_up)), 1e-9f64),
                ) * RAD2DEG;
                target_bank = 0.0f64;
                target_aoa = norm_signed_deg(desired_fpa - air_fpa);
            } else if r.command.has_target_aoa as ::core::ffi::c_int != 0
                && r.command.target_aoa.is_finite() as i32 != 0
            {
                target_aoa = r.command.target_aoa;
            } else if r.command.target_pitch.is_finite() as i32 != 0 {
                target_aoa = r.command.target_pitch - synthetic.flight_path_angle;
            }
            target_aoa = clampd(
                target_aoa,
                -(*cfg).vehicle.maximum_angle_of_attack,
                (*cfg).vehicle.maximum_angle_of_attack,
            );
            ctx.gear = r.command.gear;
            ctx.brakes = r.command.brakes;
            ctx.airbrakes = if r.command.airbrakes as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            };
            let mut dt: ::core::ffi::c_double = if taem_exec_owns_vehicle(
                &raw mut shadow.taem_exec,
            ) as ::core::ffi::c_int != 0
            {
                if geo.altitude > 25000 as ::core::ffi::c_int as ::core::ffi::c_double {
                    0.8f64
                } else if geo.altitude
                    > 8000 as ::core::ffi::c_int as ::core::ffi::c_double
                {
                    0.45f64
                } else {
                    0.25f64
                }
            } else {
                1.0f64 / clampd((*cfg).guidance.guidance_rate, 2.0f64, 30.0f64)
            };
            dt = fmin(dt, maxdur - elapsed);
            predictor_attitude_step(
                &raw mut response,
                target_bank,
                target_aoa,
                &raw mut at,
                &raw const (*cfg).vehicle,
                &raw const (*cfg).guidance,
                dt,
                (*cal0).physics,
            );
            ctx.bank = response.actual_bank;
            ctx.aoa = response.actual_aoa;
            ctx.ut = state.ut;
            ctx.mass = state.mass;
            state = rk4_aero(state, dt, &raw mut ctx);
            elapsed += dt;
            guidance_result_clear(&raw mut r);
        }
    }
    out.final_state = state;
    if out.closest_distance == DBL_MAX {
        out.closest_distance = great_circle_distance(
            predictor_geo_point(state.position, p, state.ut),
            target,
            (*p).radius,
        );
    }
    out.physics_relative_uncertainty = if (*telemetry).physics_certified_uncertainty
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        clampd((*telemetry).physics_certified_uncertainty, 0.06f64, 0.90f64)
    } else {
        0.45f64
    };
    if !outcome.is_null() {
        (*outcome).terminal_prediction_valid = out.shadow_terminal_prediction_valid;
        (*outcome).terminal_policy_feasible = out.shadow_terminal_policy_feasible;
        (*outcome).terminal_path_committed = out.shadow_terminal_path_committed;
        (*outcome).final_approach_captured = out.shadow_final_approach_captured;
        (*outcome).aborted = out.shadow_aborted;
    }
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_entry_guidance_shadow(
    mut state: VehicleState,
    mut telemetry: *const Telemetry,
    mut guidance: *const GuidanceMachine,
    mut plan: *const DeorbitPlan,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut cfg: *const LandingConfiguration,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    let mut empty: EntryPrediction = EntryPrediction {
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        taem_distance: 0.,
        taem_range_error: 0.,
        closest_distance: 0.,
        entered_atmosphere: false,
        reached_taem: false,
        taem_ownership_boundary_missed: false,
        taem_dynamic_interface_captured: false,
        taem_dynamic_interface_target_valid: false,
        entry_range: 0.,
        entry_flight_path_angle: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        taem_speed: 0.,
        taem_flight_path_angle: 0.,
        taem_altitude: 0.,
        taem_desired_altitude: 0.,
        taem_along_track: 0.,
        taem_cross_track: 0.,
        taem_course: 0.,
        mm304_gate_recorded: false,
        mm304_gate_range: 0.,
        mm304_gate_along_track: 0.,
        mm304_gate_cross_track: 0.,
        mm304_gate_course: 0.,
        mm304_gate_altitude: 0.,
        mm304_gate_speed: 0.,
        mm304_gate_flight_path_angle: 0.,
        taem_energy_error: 0.,
        taem_interface_error: 0.,
        taem_interface_energy_margin: 0.,
        peak_dynamic_pressure: 0.,
        peak_g_load: 0.,
        minimum_entry_speed: 0.,
        maximum_abs_angle_of_attack: 0.,
        minimum_bank_control_margin: 0.,
        minimum_aoa_control_margin: 0.,
        control_horizon_recorded: false,
        control_horizon_speed: 0.,
        control_horizon_altitude: 0.,
        control_horizon_range: 0.,
        control_horizon_specific_energy: 0.,
        control_horizon_alignment_distance: 0.,
        has_first_s_turn_reversal: false,
        first_s_turn_reversal_is_final: false,
        first_s_turn_reversal_ut: 0.,
        first_s_turn_reversal_range: 0.,
        first_s_turn_reversal_sign: 0.,
        s_turn_reversals: 0,
        s_turn_max_bank: 0.,
        final_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        physics_observed_seconds: 0.,
        physics_fallback_seconds: 0.,
        shadow_guidance_used: false,
        shadow_terminal_prediction_valid: false,
        shadow_terminal_policy_feasible: false,
        shadow_terminal_path_committed: false,
        shadow_final_approach_captured: false,
        shadow_aborted: false,
        terminal_survivability_stress_score: 0.,
        physics_relative_uncertainty: 0.,
        uncertainty_scenarios: 0,
    };
    memset(
        &raw mut empty as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
    trajectory_init(&raw mut empty.trajectory);
    if plan.is_null() {
        return empty;
    }
    let mut outcome: TerminalShadowOutcome = TerminalShadowOutcome {
        terminal_prediction_valid: false,
        terminal_policy_feasible: false,
        terminal_path_committed: false,
        final_approach_captured: false,
        aborted: false,
    };
    let mut out: EntryPrediction = predictor_simulate_guidance_shadow_core(
        state,
        telemetry,
        guidance,
        plan,
        p,
        aero,
        env,
        cal,
        cfg,
        maxdur,
        include,
        false_0 != 0,
        true_0 != 0,
        ::core::ptr::null::<TerminalModel>(),
        &raw mut outcome,
    );
    out.uncertainty_scenarios = if out.shadow_guidance_used as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_uint
    } else {
        0 as ::core::ffi::c_uint
    };
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_entry_guidance_shadow_with_terminal_model(
    mut state: VehicleState,
    mut telemetry: *const Telemetry,
    mut guidance: *const GuidanceMachine,
    mut plan: *const DeorbitPlan,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut cfg: *const LandingConfiguration,
    mut terminal_model: *const TerminalModel,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    let mut empty: EntryPrediction = EntryPrediction {
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        taem_distance: 0.,
        taem_range_error: 0.,
        closest_distance: 0.,
        entered_atmosphere: false,
        reached_taem: false,
        taem_ownership_boundary_missed: false,
        taem_dynamic_interface_captured: false,
        taem_dynamic_interface_target_valid: false,
        entry_range: 0.,
        entry_flight_path_angle: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        taem_speed: 0.,
        taem_flight_path_angle: 0.,
        taem_altitude: 0.,
        taem_desired_altitude: 0.,
        taem_along_track: 0.,
        taem_cross_track: 0.,
        taem_course: 0.,
        mm304_gate_recorded: false,
        mm304_gate_range: 0.,
        mm304_gate_along_track: 0.,
        mm304_gate_cross_track: 0.,
        mm304_gate_course: 0.,
        mm304_gate_altitude: 0.,
        mm304_gate_speed: 0.,
        mm304_gate_flight_path_angle: 0.,
        taem_energy_error: 0.,
        taem_interface_error: 0.,
        taem_interface_energy_margin: 0.,
        peak_dynamic_pressure: 0.,
        peak_g_load: 0.,
        minimum_entry_speed: 0.,
        maximum_abs_angle_of_attack: 0.,
        minimum_bank_control_margin: 0.,
        minimum_aoa_control_margin: 0.,
        control_horizon_recorded: false,
        control_horizon_speed: 0.,
        control_horizon_altitude: 0.,
        control_horizon_range: 0.,
        control_horizon_specific_energy: 0.,
        control_horizon_alignment_distance: 0.,
        has_first_s_turn_reversal: false,
        first_s_turn_reversal_is_final: false,
        first_s_turn_reversal_ut: 0.,
        first_s_turn_reversal_range: 0.,
        first_s_turn_reversal_sign: 0.,
        s_turn_reversals: 0,
        s_turn_max_bank: 0.,
        final_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        physics_observed_seconds: 0.,
        physics_fallback_seconds: 0.,
        shadow_guidance_used: false,
        shadow_terminal_prediction_valid: false,
        shadow_terminal_policy_feasible: false,
        shadow_terminal_path_committed: false,
        shadow_final_approach_captured: false,
        shadow_aborted: false,
        terminal_survivability_stress_score: 0.,
        physics_relative_uncertainty: 0.,
        uncertainty_scenarios: 0,
    };
    memset(
        &raw mut empty as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
    trajectory_init(&raw mut empty.trajectory);
    if plan.is_null() {
        return empty;
    }
    let mut outcome: TerminalShadowOutcome = TerminalShadowOutcome {
        terminal_prediction_valid: false,
        terminal_policy_feasible: false,
        terminal_path_committed: false,
        final_approach_captured: false,
        aborted: false,
    };
    let mut out: EntryPrediction = predictor_simulate_guidance_shadow_core(
        state,
        telemetry,
        guidance,
        plan,
        p,
        aero,
        env,
        cal,
        cfg,
        maxdur,
        include,
        false_0 != 0,
        true_0 != 0,
        terminal_model,
        &raw mut outcome,
    );
    out.uncertainty_scenarios = if out.shadow_guidance_used as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_uint
    } else {
        0 as ::core::ffi::c_uint
    };
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_terminal_shadow(
    mut state: VehicleState,
    mut telemetry: *const Telemetry,
    mut guidance: *const GuidanceMachine,
    mut plan: *const DeorbitPlan,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut cfg: *const LandingConfiguration,
    mut terminal_model: *const TerminalModel,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    let mut outcome: TerminalShadowOutcome = TerminalShadowOutcome {
        terminal_prediction_valid: false,
        terminal_policy_feasible: false,
        terminal_path_committed: false,
        final_approach_captured: false,
        aborted: false,
    };
    let mut out: EntryPrediction = predictor_simulate_guidance_shadow_core(
        state,
        telemetry,
        guidance,
        plan,
        p,
        aero,
        env,
        cal,
        cfg,
        maxdur,
        include,
        true_0 != 0,
        false_0 != 0,
        terminal_model,
        &raw mut outcome,
    );
    out.uncertainty_scenarios = if out.shadow_guidance_used as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_uint
    } else {
        0 as ::core::ffi::c_uint
    };
    out.terminal_survivability_stress_score = if out.shadow_guidance_used
        as ::core::ffi::c_int != 0
        && terminal_shadow_survives(
            &raw mut out,
            &raw mut outcome,
            &raw const (*cfg).vehicle,
        ) as ::core::ffi::c_int != 0
    {
        1.0f64
    } else {
        0.0f64
    };
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn predictor_simulate_terminal_shadow_ensemble(
    mut state: VehicleState,
    mut telemetry: *const Telemetry,
    mut guidance: *const GuidanceMachine,
    mut plan: *const DeorbitPlan,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut cfg: *const LandingConfiguration,
    mut terminal_model: *const TerminalModel,
    mut maxdur: ::core::ffi::c_double,
    mut include: bool,
) -> EntryPrediction {
    let mut nominal_outcome: TerminalShadowOutcome = TerminalShadowOutcome {
        terminal_prediction_valid: false,
        terminal_policy_feasible: false,
        terminal_path_committed: false,
        final_approach_captured: false,
        aborted: false,
    };
    let mut nominal: EntryPrediction = predictor_simulate_guidance_shadow_core(
        state,
        telemetry,
        guidance,
        plan,
        p,
        aero,
        env,
        cal,
        cfg,
        maxdur,
        include,
        true_0 != 0,
        false_0 != 0,
        terminal_model,
        &raw mut nominal_outcome,
    );
    if !nominal.shadow_guidance_used {
        return nominal;
    }
    let mut base_ctx: AeroContext = AeroContext {
        p: p,
        e: env,
        c: cal,
        v: &raw const (*cfg).vehicle,
        entry_table: ::core::ptr::null::<TerminalModel>(),
        bank: (*telemetry).roll,
        aoa: (*telemetry).angle_of_attack,
        ut: state.ut,
        mass: state.mass,
        gear: (*telemetry).gear,
        brakes: (*telemetry).brakes,
        airbrakes: ((*telemetry).has_airbrakes as ::core::ffi::c_int != 0
            && (*telemetry).airbrakes as ::core::ffi::c_int != 0) as ::core::ffi::c_int,
    };
    let mut initial: AtmosState = atmosphere(
        state.position,
        state.velocity,
        &raw mut base_ctx,
    );
    let mut sigma: ::core::ffi::c_double = clampd(
        initial.physics_uncertainty,
        0.08f64,
        0.40f64,
    );
    nominal.physics_relative_uncertainty = sigma;
    let mut stress_score: ::core::ffi::c_double = if terminal_shadow_survives(
        &raw mut nominal,
        &raw mut nominal_outcome,
        &raw const (*cfg).vehicle,
    ) as ::core::ffi::c_int != 0
    {
        0.50f64
    } else {
        0.0f64
    };
    let mut scenarios: ::core::ffi::c_uint = 1 as ::core::ffi::c_uint;
    let mut direction: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    while direction <= 1 as ::core::ffi::c_int {
        let mut stress: TrajectoryCalibrationModel = if !cal.is_null() {
            *cal
        } else {
            TrajectoryCalibrationModel {
                physics: ::core::ptr::null::<VesselPhysicsModel>(),
                density_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                drag_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                lift_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                bank_effectiveness: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                speed_of_sound: 340 as ::core::ffi::c_int as ::core::ffi::c_double,
                speed_of_sound_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                stress_drag_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                stress_lift_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
                forecast_drag_scale: 0.,
                forecast_lift_scale: 0.,
                forecast_exact_policy_bank: false,
                altitude_residual: 0.,
                speed_residual: 0.,
                range_residual: 0.,
                confidence: 0.05f64,
                accepted_samples: 0,
            }
        };
        let mut signed_sigma: ::core::ffi::c_double = direction as ::core::ffi::c_double
            * sigma;
        stress.stress_drag_scale = clampd(1.0f64 + signed_sigma, 0.60f64, 1.40f64);
        stress.stress_lift_scale = clampd(1.0f64 - signed_sigma, 0.60f64, 1.40f64);
        stress.density_scale = clampd(
            fmax(0.1f64, stress.density_scale) * (1.0f64 + 0.20f64 * signed_sigma),
            0.70f64,
            1.30f64,
        );
        let mut scenario_outcome: TerminalShadowOutcome = TerminalShadowOutcome {
            terminal_prediction_valid: false,
            terminal_policy_feasible: false,
            terminal_path_committed: false,
            final_approach_captured: false,
            aborted: false,
        };
        let mut scenario: EntryPrediction = predictor_simulate_guidance_shadow_core(
            state,
            telemetry,
            guidance,
            plan,
            p,
            aero,
            env,
            &raw mut stress,
            cfg,
            fmin(maxdur, 360.0f64),
            false_0 != 0,
            true_0 != 0,
            false_0 != 0,
            terminal_model,
            &raw mut scenario_outcome,
        );
        if terminal_shadow_survives(
            &raw mut scenario,
            &raw mut scenario_outcome,
            &raw const (*cfg).vehicle,
        ) {
            stress_score += 0.25f64;
        }
        scenarios = scenarios.wrapping_add(1);
        entry_prediction_clear(&raw mut scenario);
        direction += 2 as ::core::ffi::c_int;
    }
    nominal.terminal_survivability_stress_score = clampd(
        stress_score,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    nominal.uncertainty_scenarios = scenarios;
    return nominal;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_plan_preburn_state_compatible(
    mut plan: *const DeorbitPlan,
    mut live_mass: ::core::ffi::c_double,
    mut live_thrust: ::core::ffi::c_double,
    mut s: *const GuidanceSettings,
) -> bool {
    if plan.is_null() || s.is_null() || live_mass.is_finite() as i32 == 0
        || live_mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || live_thrust.is_finite() as i32 == 0
        || live_thrust < 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return false_0 != 0;
    }
    if (*plan).planning_mass > 1 as ::core::ffi::c_int as ::core::ffi::c_double {
        let mut limit: ::core::ffi::c_double = fmax(
            0.0f64,
            (*s).deorbit_mass_uncertainty_fraction,
        );
        if fabs(live_mass - (*plan).planning_mass)
            > (*plan).planning_mass * (limit + 1e-9f64)
        {
            return false_0 != 0;
        }
    }
    if (*plan).planning_available_thrust
        > 1 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        let mut limit_0: ::core::ffi::c_double = fmax(
            0.0f64,
            (*s).deorbit_thrust_uncertainty_fraction,
        );
        if fabs(live_thrust - (*plan).planning_available_thrust)
            > (*plan).planning_available_thrust * (limit_0 + 1e-9f64)
        {
            return false_0 != 0;
        }
    }
    return true_0 != 0;
}
unsafe extern "C" fn deorbit_alignment_position_error(
    mut p: *const EntryPrediction,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if p.is_null() || (*p).taem_interface_error.is_finite() as i32 == 0
        || (*p).taem_interface_error < 0.0f64
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return (*p).taem_interface_error;
}
unsafe extern "C" fn deorbit_alignment_course_error(
    mut p: *const EntryPrediction,
    mut site: *const LandingSite,
) -> ::core::ffi::c_double {
    if p.is_null() || site.is_null() || (*p).taem_course.is_finite() as i32 == 0 {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut runway_offset: ::core::ffi::c_double = fabs(
        norm_signed_deg((*p).taem_course - (*site).runway_heading),
    );
    let mut perpendicular_error: ::core::ffi::c_double = fabs(runway_offset - 90.0f64);
    let mut aligned_error: ::core::ffi::c_double = fmin(
        runway_offset,
        fabs(runway_offset - 180.0f64),
    );
    return fmin(perpendicular_error, aligned_error);
}
unsafe extern "C" fn deorbit_alignment_contract_utilization(
    mut p: *const EntryPrediction,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    let mut position: ::core::ffi::c_double = deorbit_alignment_position_error(p, s);
    let mut course: ::core::ffi::c_double = deorbit_alignment_course_error(p, site);
    let mut contract: TaemHandoffContract = taem_handoff_contract(s);
    if position.is_finite() as i32 == 0 || course.is_finite() as i32 == 0
        || !(contract.horizontal_radius_m > 0.0f64)
        || !(contract.perpendicular_heading_half_width_deg > 0.0f64)
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    return fmax(
        position / contract.horizontal_radius_m,
        course / contract.perpendicular_heading_half_width_deg,
    );
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_capture_qualified(
    mut p: *const EntryPrediction,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut peri: ::core::ffi::c_double,
    mut require_entry_target: bool,
) -> bool {
    if p.is_null() || v.is_null() || s.is_null() || !(*p).entered_atmosphere
        || !(*p).mm304_gate_recorded || (*p).entry_range.is_finite() as i32 == 0
        || (*p).entry_flight_path_angle.is_finite() as i32 == 0
        || (*p).entry_course_error.is_finite() as i32 == 0
        || (*p).peak_dynamic_pressure.is_finite() as i32 == 0
        || (*p).peak_g_load.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut safe: bool = (*p).entry_flight_path_angle
        >= (*s).maximum_entry_flight_path_angle
        && (*p).peak_dynamic_pressure <= (*v).maximum_dynamic_pressure
        && (*p).peak_g_load <= (*v).maximum_g_load;
    if !safe {
        return false_0 != 0;
    }
    if require_entry_target {
        return (*s).target_entry_range.is_finite() as i32 != 0
            && (*s).target_entry_flight_path_angle.is_finite() as i32 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_entry_stage_qualified(
    mut p: *const EntryPrediction,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut peri: ::core::ffi::c_double,
    mut site_altitude: ::core::ffi::c_double,
    mut atmosphere_depth: ::core::ffi::c_double,
    mut finite_burn_completed: bool,
) -> bool {
    if !finite_burn_completed || p.is_null() || v.is_null() || s.is_null()
        || !(*p).entered_atmosphere || peri.is_finite() as i32 == 0
        || site_altitude.is_finite() as i32 == 0
        || atmosphere_depth.is_finite() as i32 == 0
        || !(peri > site_altitude && peri < atmosphere_depth)
        || (*p).entry_range.is_finite() as i32 == 0
        || (*p).entry_flight_path_angle.is_finite() as i32 == 0
        || (*p).peak_dynamic_pressure.is_finite() as i32 == 0
        || (*p).peak_g_load.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    return (*p).entry_flight_path_angle >= (*s).maximum_entry_flight_path_angle
        && (*p).peak_dynamic_pressure <= (*v).maximum_dynamic_pressure
        && (*p).peak_g_load <= (*v).maximum_g_load;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_entry_stage_robustness_qualified(
    mut passed: ::core::ffi::c_uint,
    mut scenarios: ::core::ffi::c_uint,
    mut unsafe_0: ::core::ffi::c_uint,
    mut required_pass_fraction: ::core::ffi::c_double,
) -> bool {
    if scenarios == 0 as ::core::ffi::c_uint || passed > scenarios
        || unsafe_0 != 0 as ::core::ffi::c_uint
        || required_pass_fraction.is_finite() as i32 == 0
        || required_pass_fraction < 0.0f64 || required_pass_fraction > 1.0f64
    {
        return false_0 != 0;
    }
    return passed as ::core::ffi::c_double / scenarios as ::core::ffi::c_double + 1e-9f64
        >= required_pass_fraction;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_recovery_qualified(
    mut p: *const EntryPrediction,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut peri: ::core::ffi::c_double,
) -> bool {
    return deorbit_capture_qualified(p, site, v, s, peri, false_0 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_live_cutoff_capture_qualified(
    mut p: *const EntryPrediction,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut peri: ::core::ffi::c_double,
) -> bool {
    if !deorbit_recovery_qualified(p, site, v, s, peri) || !(*p).reached_taem
        || (*p).shadow_aborted as ::core::ffi::c_int != 0
        || (*p).taem_ownership_boundary_missed as ::core::ffi::c_int != 0
        || (*p).minimum_entry_speed.is_finite() as i32 == 0
        || (*p).minimum_entry_speed < (*v).minimum_safe_speed
        || (*p).taem_speed.is_finite() as i32 == 0
        || (*p).taem_speed < (*v).minimum_safe_speed
    {
        return false_0 != 0;
    }
    let mut interface_capture: bool = if (*p).taem_dynamic_interface_target_valid
        as ::core::ffi::c_int != 0
    {
        (*p).taem_dynamic_interface_captured as ::core::ffi::c_int
    } else {
        ((*p).closest_distance.is_finite() as i32 != 0 && (*p).closest_distance >= 0.0f64
            && (*p).closest_distance <= (*s).target_deorbit_capture_radius)
            as ::core::ffi::c_int
    } != 0;
    let mut alignment: ::core::ffi::c_double = deorbit_alignment_contract_utilization(
        p,
        site,
        s,
    );
    return interface_capture as ::core::ffi::c_int != 0
        && alignment.is_finite() as i32 != 0 && alignment <= 1.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_runway_capture_qualified(
    mut p: *const EntryPrediction,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut peri: ::core::ffi::c_double,
) -> bool {
    if !deorbit_recovery_qualified(p, site, v, s, peri) || !(*p).reached_taem
        || !(*p).taem_dynamic_interface_captured || !(*p).shadow_terminal_path_committed
        || !(*p).shadow_final_approach_captured
        || (*p).taem_interface_energy_margin.is_finite() as i32 == 0
        || !((*p).taem_interface_energy_margin >= 0.0f64)
        || (*p).shadow_aborted as ::core::ffi::c_int != 0
        || (*p).taem_ownership_boundary_missed as ::core::ffi::c_int != 0
        || (*p).minimum_entry_speed.is_finite() as i32 == 0
        || (*p).minimum_entry_speed < (*v).minimum_safe_speed
        || (*p).taem_speed.is_finite() as i32 == 0
        || (*p).taem_speed < (*v).minimum_safe_speed
        || (*s).target_deorbit_capture_radius.is_finite() as i32 == 0
        || !((*s).target_deorbit_capture_radius > 0.0f64)
        || (*s).mm304_perpendicular_heading_half_width.is_finite() as i32 == 0
        || !((*s).mm304_perpendicular_heading_half_width > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut minimum_taem_range: ::core::ffi::c_double = fmax(
        (*s).final_approach_distance * 1.5f64,
        (*s).final_approach_distance + fmax(0.0f64, (*s).hac_radius),
    );
    let mut maximum_taem_range: ::core::ffi::c_double = fmax(
        (*s).taem_interface_range,
        minimum_taem_range,
    );
    let mut position: ::core::ffi::c_double = deorbit_alignment_position_error(p, s);
    let mut course: ::core::ffi::c_double = deorbit_alignment_course_error(p, site);
    return (*p).closest_distance.is_finite() as i32 != 0
        && (*p).closest_distance <= (*s).target_deorbit_capture_radius
        && (*p).taem_distance.is_finite() as i32 != 0
        && (*p).taem_distance >= minimum_taem_range
        && (*p).taem_distance <= maximum_taem_range
        && (*p).taem_range_error.is_finite() as i32 != 0
        && fabs((*p).taem_range_error) <= (*s).target_deorbit_capture_radius
        && position.is_finite() as i32 != 0
        && position <= (*s).target_deorbit_capture_radius
        && course.is_finite() as i32 != 0
        && course <= (*s).mm304_perpendicular_heading_half_width;
}
#[no_mangle]
pub unsafe extern "C" fn reentry_guidance_shadow_recovery_qualified(
    mut p: *const EntryPrediction,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> bool {
    if p.is_null() || v.is_null() || s.is_null() || !(*p).shadow_guidance_used
        || !(*p).entered_atmosphere || !(*p).reached_taem
        || (*p).shadow_aborted as ::core::ffi::c_int != 0
        || (*p).taem_ownership_boundary_missed as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    if (*p).closest_distance.is_finite() as i32 == 0
        || (*p).taem_distance.is_finite() as i32 == 0
        || (*p).taem_range_error.is_finite() as i32 == 0
        || (*p).taem_speed.is_finite() as i32 == 0
        || (*p).peak_dynamic_pressure.is_finite() as i32 == 0
        || (*p).peak_g_load.is_finite() as i32 == 0
        || (*p).minimum_entry_speed.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut capture: bool = if (*p).taem_dynamic_interface_target_valid
        as ::core::ffi::c_int != 0
    {
        (*p).taem_dynamic_interface_captured as ::core::ffi::c_int
    } else {
        ((*p).closest_distance >= 0.0f64
            && (*p).closest_distance <= (*s).target_deorbit_capture_radius)
            as ::core::ffi::c_int
    } != 0;
    let mut load: bool = (*p).peak_dynamic_pressure <= (*v).maximum_dynamic_pressure
        && (*p).peak_g_load <= (*v).maximum_g_load;
    let mut speed: bool = (*p).minimum_entry_speed >= (*v).minimum_safe_speed
        && (*p).taem_speed >= (*v).minimum_safe_speed;
    return capture as ::core::ffi::c_int != 0 && load as ::core::ffi::c_int != 0
        && speed as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn deorbit_effective_entry_range(
    mut s: *const GuidanceSettings,
    mut p: *const PlanetModel,
    mut speed: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut ref_0: ::core::ffi::c_double = (*s).entry_range_reference_speed;
    if !(ref_0 > 0.0f64) || speed.is_finite() as i32 == 0 || !(speed > 0.0f64) {
        return (*s).target_entry_range;
    }
    let mut vt: ::core::ffi::c_double = fmax(1.0f64, (*s).mm305_target_mach) * 320.0f64;
    let mut g: ::core::ffi::c_double = (*p).gravitational_parameter
        / (((*p).radius + 60000.0f64) * ((*p).radius + 60000.0f64));
    let mut dh: ::core::ffi::c_double = fmax(
        0.0f64,
        (*p).atmosphere_depth - (*s).entry_interface_altitude_margin
            - (*s).taem_interface_altitude,
    );
    let mut pe: ::core::ffi::c_double = 2.0f64 * g * dh;
    let mut scale: ::core::ffi::c_double = (speed * speed - vt * vt + pe)
        / fmax(1.0f64, ref_0 * ref_0 - vt * vt + pe);
    return (*s).target_entry_range * clampd(scale, 0.7f64, 2.0f64);
}
unsafe extern "C" fn deorbit_passive_runway_margin(
    mut geo: GeoPoint,
    mut site: *const LandingSite,
    mut radius: ::core::ffi::c_double,
    mut m: *mut DeorbitEnergyMargin,
) {
    let mut site_geo: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut along: ::core::ffi::c_double = 0.0f64;
    let mut cross: ::core::ffi::c_double = 0.0f64;
    runway_coordinates(
        geo,
        site_geo,
        (*site).runway_heading,
        radius,
        &raw mut along,
        &raw mut cross,
    );
    (*m).impact_runway_end = 0 as ::core::ffi::c_int;
    if (*site).allow_reciprocal_runway {
        let mut reciprocal: LandingSite = runway_reciprocal_site(site, radius);
        let mut reciprocal_geo: GeoPoint = GeoPoint {
            latitude: reciprocal.latitude,
            longitude: reciprocal.longitude,
            altitude: reciprocal.altitude,
        };
        let mut reciprocal_along: ::core::ffi::c_double = 0.0f64;
        let mut reciprocal_cross: ::core::ffi::c_double = 0.0f64;
        runway_coordinates(
            geo,
            reciprocal_geo,
            reciprocal.runway_heading,
            radius,
            &raw mut reciprocal_along,
            &raw mut reciprocal_cross,
        );
        if reciprocal_along.is_finite() as i32 != 0 && reciprocal_along > along {
            along = reciprocal_along;
            cross = reciprocal_cross;
            (*m).impact_runway_end = 1 as ::core::ffi::c_int;
        }
    }
    (*m).impact_along_track = along;
    (*m).impact_cross_track = cross;
}
unsafe extern "C" fn deorbit_energy_margin_probe(
    mut after: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> DeorbitEnergyMargin {
    let mut m: DeorbitEnergyMargin = DeorbitEnergyMargin {
        valid: 0 as ::core::ffi::c_int != 0,
        impact_along_track: 0.,
        impact_cross_track: 0.,
        impact_range: 0.,
        impact_altitude: 0.,
        impact_speed: 0.,
        impact_runway_end: 0,
    };
    let mut passive: EntryPrediction = predictor_simulate_entry(
        after,
        p,
        aero,
        env,
        cal,
        v,
        site,
        s,
        0.0f64,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        4500 as ::core::ffi::c_int as ::core::ffi::c_double,
        false_0 != 0,
    );
    let mut final_0: VehicleState = passive.final_state;
    let mut geo: GeoPoint = predictor_geo_point(final_0.position, p, final_0.ut);
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING probe passive_s=%.0f entered=%d alt_km=%.2f vs=%.1f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            final_0.ut - after.ut,
            if passive.entered_atmosphere as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            predictor_geo_point(final_0.position, p, final_0.ut).altitude / 1000.0f64,
            vdot(
                final_0.velocity,
                vnorm(
                    final_0.position,
                    v3(
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        1 as ::core::ffi::c_int as ::core::ffi::c_double,
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ),
                ),
            ),
        );
    }
    let mut up: Vector3 = vnorm(
        final_0.position,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut surface_velocity: Vector3 = vsub(
        final_0.velocity,
        vcross(planet_rotation_vector(p), final_0.position),
    );
    let mut vertical: ::core::ffi::c_double = vdot(surface_velocity, up);
    let mut site_geo: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    deorbit_passive_runway_margin(geo, site, (*p).radius, &raw mut m);
    m.valid = passive.entered_atmosphere as ::core::ffi::c_int != 0
        && geo.altitude.is_finite() as i32 != 0
        && m.impact_along_track.is_finite() as i32 != 0
        && m.impact_cross_track.is_finite() as i32 != 0
        && vertical.is_finite() as i32 != 0
        && geo.altitude <= (*site).altitude + 750.0f64 && vertical < 0.0f64;
    m.impact_range = great_circle_distance(geo, site_geo, (*p).radius);
    m.impact_altitude = geo.altitude;
    m.impact_speed = vmag(surface_velocity);
    entry_prediction_clear(&raw mut passive);
    return m;
}
unsafe extern "C" fn deorbit_energy_margin_qualified(
    mut m: *const DeorbitEnergyMargin,
    mut s: *const GuidanceSettings,
    mut strict: bool,
) -> bool {
    return !m.is_null() && (*m).valid as ::core::ffi::c_int != 0
        && (*m).impact_along_track.is_finite() as i32 != 0
        && (*m).impact_along_track >= 0.0f64;
}
unsafe extern "C" fn candidate_clear(mut c: *mut Candidate) {
    entry_prediction_clear(&raw mut (*c).pred);
}
unsafe extern "C" fn deorbit_candidate_safety_violation(
    mut p: *const EntryPrediction,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if p.is_null() || v.is_null() || s.is_null() || !(*p).entered_atmosphere {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut q: ::core::ffi::c_double = predictor_normalized_upper_violation(
        (*p).peak_dynamic_pressure,
        (*v).maximum_dynamic_pressure,
    );
    let mut g: ::core::ffi::c_double = predictor_normalized_upper_violation(
        (*p).peak_g_load,
        (*v).maximum_g_load,
    );
    let mut fpa: ::core::ffi::c_double = fmax(
        0.0f64,
        (*s).maximum_entry_flight_path_angle - (*p).entry_flight_path_angle,
    );
    return fmax(q, fmax(g, fpa));
}
unsafe extern "C" fn tier(
    mut c: *mut Candidate,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_int {
    if c.is_null() {
        return 0 as ::core::ffi::c_int;
    }
    let mut safe: bool = deorbit_capture_qualified(
        &raw mut (*c).pred,
        site,
        v,
        s,
        (*c).peri,
        false_0 != 0,
    );
    let mut energy: bool = deorbit_energy_margin_qualified(
        &raw mut (*c).energy_margin,
        s,
        false_0 != 0,
    );
    let mut landing: bool = deorbit_runway_capture_qualified(
        &raw mut (*c).pred,
        site,
        v,
        s,
        (*c).peri,
    );
    if safe as ::core::ffi::c_int != 0 && energy as ::core::ffi::c_int != 0
        && (*c).pred.mm304_gate_recorded as ::core::ffi::c_int != 0
        && landing as ::core::ffi::c_int != 0
    {
        return 4 as ::core::ffi::c_int;
    }
    if safe as ::core::ffi::c_int != 0 && energy as ::core::ffi::c_int != 0
        && (*c).pred.mm304_gate_recorded as ::core::ffi::c_int != 0
    {
        return 3 as ::core::ffi::c_int;
    }
    if safe as ::core::ffi::c_int != 0 && energy as ::core::ffi::c_int != 0 {
        return 2 as ::core::ffi::c_int;
    }
    if safe {
        return 1 as ::core::ffi::c_int;
    }
    return 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn impulsive_deorbit_periapsis_plausible(
    mut coast: VehicleState,
    mut dv: ::core::ffi::c_double,
    mut normal_angle: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
) -> bool {
    let mut impulse: VehicleState = coast;
    let mut direction: Vector3 = burn_direction(
        coast.position,
        coast.velocity,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        normal_angle,
    );
    impulse.velocity = vadd(impulse.velocity, vscale(direction, dv));
    let mut peri: ::core::ffi::c_double = predictor_postburn_periapsis(impulse, p);
    return peri.is_finite() as i32 != 0 && peri < (*p).atmosphere_depth
        && peri > -(*p).radius;
}
unsafe extern "C" fn eval_candidate(
    mut out: *mut Candidate,
    mut coast: VehicleState,
    mut dv: ::core::ffi::c_double,
    mut normal_angle: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut reject: *mut DeorbitRejectReason,
) -> bool {
    memset(
        out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Candidate>() as size_t,
    );
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_IMPULSIVE_PERIAPSIS;
    }
    if !impulsive_deorbit_periapsis_plausible(coast, dv, normal_angle, p, s) {
        return false_0 != 0;
    }
    let mut dur: ::core::ffi::c_double = 0.0f64;
    let mut delivered: ::core::ffi::c_double = 0.0f64;
    let mut after: VehicleState = retro_burn_scenario(
        coast,
        dv,
        thrust,
        (*s).deorbit_maximum_throttle,
        (*s).deorbit_throttle_ramp_duration,
        p,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        normal_angle,
        &raw mut dur,
        &raw mut delivered,
        if !cal.is_null() {
            (*cal).physics
        } else {
            ::core::ptr::null::<VesselPhysicsModel>()
        },
    );
    let mut dv_tol: ::core::ffi::c_double = fmax(
        GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON) * fmax(1.0f64, fabs(dv)),
    );
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_FINITE_BURN;
    }
    if dur.is_finite() as i32 == 0 || delivered.is_finite() as i32 == 0
        || delivered + dv_tol < dv
    {
        return false_0 != 0;
    }
    let mut peri: ::core::ffi::c_double = predictor_postburn_periapsis(after, p);
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_POSTBURN_PERIAPSIS;
    }
    if peri.is_finite() as i32 == 0 || peri >= (*p).atmosphere_depth
        || peri <= -(*p).radius
    {
        return false_0 != 0;
    }
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_ENTRY_PREDICTION;
    }
    let mut pr: EntryPrediction = predictor_simulate_entry(
        after,
        p,
        aero,
        env,
        cal,
        v,
        site,
        s,
        (*v).maximum_bank_angle,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        4500 as ::core::ffi::c_int as ::core::ffi::c_double,
        false_0 != 0,
    );
    if !pr.entered_atmosphere || pr.entry_range.is_finite() as i32 == 0
        || pr.entry_flight_path_angle.is_finite() as i32 == 0
        || pr.entry_course_error.is_finite() as i32 == 0
        || pr.peak_dynamic_pressure.is_finite() as i32 == 0
        || pr.peak_g_load.is_finite() as i32 == 0
    {
        entry_prediction_clear(&raw mut pr);
        return false_0 != 0;
    }
    let mut energy_margin: DeorbitEnergyMargin = deorbit_energy_margin_probe(
        after,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
    );
    (*out).burn_ut = coast.ut + dur * 0.5f64;
    (*out).dv = dv;
    (*out).duration = dur;
    (*out).burn_normal_angle = normal_angle;
    (*out).after = after;
    (*out).peri = peri;
    (*out).requested_periapsis_altitude = (*s).target_post_burn_periapsis_altitude;
    (*out).energy_margin = energy_margin;
    (*out).pred = pr;
    (*out).safety_violation = deorbit_candidate_safety_violation(&raw mut pr, v, s);
    (*out).energy_deficit = if energy_margin.valid as ::core::ffi::c_int != 0 {
        fmax(0.0f64, -energy_margin.impact_along_track)
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    (*out).energy_excess = if energy_margin.valid as ::core::ffi::c_int != 0 {
        fmax(0.0f64, energy_margin.impact_along_track)
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    (*out).entry_fpa_error = fabs(
        pr.entry_flight_path_angle - (*s).target_entry_flight_path_angle,
    );
    (*out).entry_course_error = fabs(pr.entry_course_error);
    let mut entry_central_angle: ::core::ffi::c_double = pr.entry_range
        / fmax((*p).radius, DBL_MIN);
    let mut entry_cross_sine: ::core::ffi::c_double = sin(entry_central_angle)
        * sin(pr.entry_course_error * DEG2RAD);
    (*out).entry_groundtrack_crossrange_error = fabs(
        asin(clampd(entry_cross_sine, -1.0f64, 1.0f64)) * (*p).radius,
    );
    (*out).entry_range_error = fabs(pr.entry_range - (*s).target_entry_range);
    (*out).handoff_alignment_utilization = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    (*out).handoff_range_error = ::core::f32::INFINITY as ::core::ffi::c_double;
    if pr.reached_taem as ::core::ffi::c_int != 0 && !pr.taem_ownership_boundary_missed {
        (*out).handoff_alignment_utilization = deorbit_alignment_contract_utilization(
            &raw mut pr,
            site,
            s,
        );
        if pr.taem_range_error.is_finite() as i32 != 0 {
            (*out).handoff_range_error = fabs(pr.taem_range_error);
        }
    }
    (*out).tier = tier(out, site, v, s);
    (*out).score = if (*out).tier >= 2 as ::core::ffi::c_int {
        (*out).entry_fpa_error
    } else {
        (*out).safety_violation
    };
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_NONE;
    }
    return true_0 != 0;
}
unsafe extern "C" fn hard_unsafe(
    mut pr: *const EntryPrediction,
    mut peri: ::core::ffi::c_double,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut burn_completed: bool,
) -> bool {
    if !burn_completed || !(*pr).entered_atmosphere {
        return true_0 != 0;
    }
    if (*pr).entry_flight_path_angle < (*s).maximum_entry_flight_path_angle {
        return true_0 != 0;
    }
    if (*pr).peak_dynamic_pressure > (*v).maximum_dynamic_pressure {
        return true_0 != 0;
    }
    if (*pr).peak_g_load > (*v).maximum_g_load {
        return true_0 != 0;
    }
    return false_0 != 0;
}
unsafe extern "C" fn assess_candidate_robustness(
    mut c: *const Candidate,
    mut initial: VehicleState,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut authority_source: *const Telemetry,
    mut terminal_model: *const TerminalModel,
    mut full_terminal_stress: bool,
    mut trace: bool,
) -> RobustAssessment {
    let mut timing: ::core::ffi::c_double = (*s).deorbit_timing_uncertainty;
    let mut thrust_unc: ::core::ffi::c_double = (*s).deorbit_thrust_uncertainty_fraction;
    let mut dv_unc: ::core::ffi::c_double = (*s).deorbit_delta_v_uncertainty;
    let mut mass_unc: ::core::ffi::c_double = (*s).deorbit_mass_uncertainty_fraction;
    let mut pos_unc: ::core::ffi::c_double = (*s).deorbit_position_uncertainty;
    let mut vel_unc: ::core::ffi::c_double = (*s).deorbit_velocity_uncertainty;
    let mut pointing: ::core::ffi::c_double = (*s).deorbit_pointing_uncertainty;
    let mut atmos: ::core::ffi::c_double = (*s).deorbit_atmosphere_uncertainty_fraction;
    let mut aero_unc: ::core::ffi::c_double = 0.30f64;
    if !cal.is_null() && !(*cal).physics.is_null() {
        let mut m: *const VesselPhysicsModel = (*cal).physics;
        if (*m).certified_uncertainty > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            aero_unc = (*m).certified_uncertainty;
        }
        if (*m).model_residual_confidence > 0.20f64 {
            aero_unc = fmax(
                aero_unc,
                (*m).model_residual
                    * clampd(
                        (*m).model_residual_confidence,
                        0.25f64,
                        1 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ),
            );
        }
    }
    let mut aero_seconds: ::core::ffi::c_double = (*c).pred.physics_observed_seconds
        + (*c).pred.physics_fallback_seconds;
    if aero_seconds > 1.0f64
        && (*c).pred.physics_fallback_seconds
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double && aero_unc < 0.30f64
    {
        let mut fallback_fraction: ::core::ffi::c_double = clampd(
            (*c).pred.physics_fallback_seconds / aero_seconds,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        aero_unc += fallback_fraction * (0.30f64 - aero_unc);
    }
    aero_unc = clampd(aero_unc, 0.08f64, 0.40f64);
    let mut cases: [BurnStressCase; 15] = [
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: timing,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double - thrust_unc,
            dv_offset: -dv_unc,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + mass_unc,
            position_offset: pos_unc,
            velocity_offset: -vel_unc,
            radial_error: pointing,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: -timing,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + thrust_unc,
            dv_offset: dv_unc,
            mass_scale: fmax(
                0.5f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - mass_unc,
            ),
            position_offset: -pos_unc,
            velocity_offset: vel_unc,
            radial_error: -pointing,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: timing,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double - thrust_unc,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + mass_unc,
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: -vel_unc,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: pointing,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: -timing,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + thrust_unc,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: fmax(
                0.5f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - mass_unc,
            ),
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: vel_unc,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: -pointing,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: -dv_unc,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + mass_unc,
            position_offset: pos_unc,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: pointing,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: dv_unc,
            mass_scale: fmax(
                0.5f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - mass_unc,
            ),
            position_offset: -pos_unc,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: -pointing,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: -atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            position_offset: pos_unc,
            velocity_offset: vel_unc,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: pointing,
            atmosphere_fraction: atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            position_offset: -pos_unc,
            velocity_offset: -vel_unc,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: -pointing,
            atmosphere_fraction: -atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: timing,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + mass_unc,
            position_offset: pos_unc,
            velocity_offset: vel_unc,
            radial_error: -pointing,
            normal_error: pointing,
            atmosphere_fraction: atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: -timing,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: fmax(
                0.5f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - mass_unc,
            ),
            position_offset: -pos_unc,
            velocity_offset: -vel_unc,
            radial_error: pointing,
            normal_error: -pointing,
            atmosphere_fraction: -atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double - thrust_unc,
            dv_offset: -dv_unc,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + mass_unc,
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + thrust_unc,
            dv_offset: dv_unc,
            mass_scale: fmax(
                0.5f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - mass_unc,
            ),
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: -atmos,
            aero_drag_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_lift_scale: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + aero_unc,
            aero_lift_scale: fmax(
                0.50f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - aero_unc,
            ),
        },
        BurnStressCase {
            timing_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            thrust_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            dv_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double,
            position_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            velocity_offset: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            radial_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            normal_error: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            atmosphere_fraction: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            aero_drag_scale: fmax(
                0.50f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double - aero_unc,
            ),
            aero_lift_scale: 1 as ::core::ffi::c_int as ::core::ffi::c_double + aero_unc,
        },
    ];
    let mut r: RobustAssessment = RobustAssessment {
        scenarios: 0,
        passed: 0,
        unsafe_0: 0,
        recovery_passed: 0,
        entry_stage_passed: 0,
        pass_fraction: 0.,
        recovery_pass_fraction: 0.,
        worst_closest: 0.,
        worst_taem_error: 0.,
        worst_alignment_error: 0.,
        worst_entry_angle: 0.,
        min_periapsis: 0.,
        max_dynamic_pressure: 0.,
        max_g_load: 0.,
        entry_stage_pass_fraction: 0.,
        qualified: false,
        recovery_qualified: false,
        entry_stage_qualified: false,
    };
    memset(
        &raw mut r as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<RobustAssessment>() as size_t,
    );
    r.worst_closest = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    r.worst_taem_error = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    r.worst_alignment_error = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    r.worst_entry_angle = DBL_MAX;
    r.min_periapsis = DBL_MAX;
    let mut nominal_start: ::core::ffi::c_double = (*c).burn_ut - (*c).duration * 0.5f64;
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[BurnStressCase; 15]>() as usize)
            .wrapping_div(::core::mem::size_of::<BurnStressCase>() as usize)
    {
        let mut sc: BurnStressCase = cases[i as usize];
        let mut scenario_state: VehicleState = initial;
        let mut radial: Vector3 = vnorm(
            initial.position,
            v3(
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut along: Vector3 = vnorm(
            vproject_plane(initial.velocity, radial),
            vnorm(
                initial.velocity,
                v3(
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            ),
        );
        scenario_state.position = vadd(
            scenario_state.position,
            vscale(radial, sc.position_offset),
        );
        scenario_state.velocity = vadd(
            scenario_state.velocity,
            vscale(along, sc.velocity_offset),
        );
        scenario_state.mass = fmax(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            scenario_state.mass * sc.mass_scale,
        );
        let mut start: ::core::ffi::c_double = fmax(
            scenario_state.ut + 5 as ::core::ffi::c_int as ::core::ffi::c_double,
            nominal_start + sc.timing_offset,
        );
        let mut coast: VehicleState = predictor_propagate_vacuum(
            scenario_state,
            start,
            p,
            3 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        let mut scenario_dv: ::core::ffi::c_double = fmax(
            0.2f64,
            (*c).dv + sc.dv_offset,
        );
        let mut duration: ::core::ffi::c_double = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        let mut delivered: ::core::ffi::c_double = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        let mut after: VehicleState = retro_burn_scenario(
            coast,
            scenario_dv,
            thrust * fmax(0.05f64, sc.thrust_scale),
            (*s).deorbit_maximum_throttle,
            (*s).deorbit_throttle_ramp_duration,
            p,
            sc.radial_error,
            (*c).burn_normal_angle + sc.normal_error,
            &raw mut duration,
            &raw mut delivered,
            if !cal.is_null() {
                (*cal).physics
            } else {
                ::core::ptr::null::<VesselPhysicsModel>()
            },
        );
        let mut burn_completed: bool = duration.is_finite() as i32 != 0
            && delivered.is_finite() as i32 != 0 && delivered >= scenario_dv - 0.25f64;
        let mut peri: ::core::ffi::c_double = predictor_postburn_periapsis(after, p);
        let mut scenario_cal: TrajectoryCalibrationModel = *cal;
        scenario_cal.density_scale = clampd(
            (*cal).density_scale
                * (1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + sc.atmosphere_fraction),
            0.35f64,
            2.8f64,
        );
        scenario_cal.stress_drag_scale = if sc.aero_drag_scale
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            sc.aero_drag_scale
        } else {
            1 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        scenario_cal.stress_lift_scale = if sc.aero_lift_scale
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            sc.aero_lift_scale
        } else {
            1 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        let mut pr: EntryPrediction = EntryPrediction {
            trajectory: Trajectory {
                points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                count: 0,
                capacity: 0,
            },
            taem_distance: 0.,
            taem_range_error: 0.,
            closest_distance: 0.,
            entered_atmosphere: false,
            reached_taem: false,
            taem_ownership_boundary_missed: false,
            taem_dynamic_interface_captured: false,
            taem_dynamic_interface_target_valid: false,
            entry_range: 0.,
            entry_flight_path_angle: 0.,
            entry_speed: 0.,
            entry_course_error: 0.,
            taem_speed: 0.,
            taem_flight_path_angle: 0.,
            taem_altitude: 0.,
            taem_desired_altitude: 0.,
            taem_along_track: 0.,
            taem_cross_track: 0.,
            taem_course: 0.,
            mm304_gate_recorded: false,
            mm304_gate_range: 0.,
            mm304_gate_along_track: 0.,
            mm304_gate_cross_track: 0.,
            mm304_gate_course: 0.,
            mm304_gate_altitude: 0.,
            mm304_gate_speed: 0.,
            mm304_gate_flight_path_angle: 0.,
            taem_energy_error: 0.,
            taem_interface_error: 0.,
            taem_interface_energy_margin: 0.,
            peak_dynamic_pressure: 0.,
            peak_g_load: 0.,
            minimum_entry_speed: 0.,
            maximum_abs_angle_of_attack: 0.,
            minimum_bank_control_margin: 0.,
            minimum_aoa_control_margin: 0.,
            control_horizon_recorded: false,
            control_horizon_speed: 0.,
            control_horizon_altitude: 0.,
            control_horizon_range: 0.,
            control_horizon_specific_energy: 0.,
            control_horizon_alignment_distance: 0.,
            has_first_s_turn_reversal: false,
            first_s_turn_reversal_is_final: false,
            first_s_turn_reversal_ut: 0.,
            first_s_turn_reversal_range: 0.,
            first_s_turn_reversal_sign: 0.,
            s_turn_reversals: 0,
            s_turn_max_bank: 0.,
            final_state: VehicleState {
                ut: 0.,
                position: Vector3 { x: 0., y: 0., z: 0. },
                velocity: Vector3 { x: 0., y: 0., z: 0. },
                mass: 0.,
            },
            physics_observed_seconds: 0.,
            physics_fallback_seconds: 0.,
            shadow_guidance_used: false,
            shadow_terminal_prediction_valid: false,
            shadow_terminal_policy_feasible: false,
            shadow_terminal_path_committed: false,
            shadow_final_approach_captured: false,
            shadow_aborted: false,
            terminal_survivability_stress_score: 0.,
            physics_relative_uncertainty: 0.,
            uncertainty_scenarios: 0,
        };
        if full_terminal_stress as ::core::ffi::c_int != 0
            && burn_completed as ::core::ffi::c_int != 0
        {
            let mut stressed: Candidate = Candidate {
                burn_ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                dv: 0.,
                duration: 0.,
                peri: 0.,
                requested_periapsis_altitude: 0.,
                score: 0.,
                burn_normal_angle: 0.,
                after: VehicleState {
                    ut: 0.,
                    position: Vector3 { x: 0., y: 0., z: 0. },
                    velocity: Vector3 { x: 0., y: 0., z: 0. },
                    mass: 0.,
                },
                pred: EntryPrediction {
                    trajectory: Trajectory {
                        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                        count: 0,
                        capacity: 0,
                    },
                    taem_distance: 0.,
                    taem_range_error: 0.,
                    closest_distance: 0.,
                    entered_atmosphere: false,
                    reached_taem: false,
                    taem_ownership_boundary_missed: false,
                    taem_dynamic_interface_captured: false,
                    taem_dynamic_interface_target_valid: false,
                    entry_range: 0.,
                    entry_flight_path_angle: 0.,
                    entry_speed: 0.,
                    entry_course_error: 0.,
                    taem_speed: 0.,
                    taem_flight_path_angle: 0.,
                    taem_altitude: 0.,
                    taem_desired_altitude: 0.,
                    taem_along_track: 0.,
                    taem_cross_track: 0.,
                    taem_course: 0.,
                    mm304_gate_recorded: false,
                    mm304_gate_range: 0.,
                    mm304_gate_along_track: 0.,
                    mm304_gate_cross_track: 0.,
                    mm304_gate_course: 0.,
                    mm304_gate_altitude: 0.,
                    mm304_gate_speed: 0.,
                    mm304_gate_flight_path_angle: 0.,
                    taem_energy_error: 0.,
                    taem_interface_error: 0.,
                    taem_interface_energy_margin: 0.,
                    peak_dynamic_pressure: 0.,
                    peak_g_load: 0.,
                    minimum_entry_speed: 0.,
                    maximum_abs_angle_of_attack: 0.,
                    minimum_bank_control_margin: 0.,
                    minimum_aoa_control_margin: 0.,
                    control_horizon_recorded: false,
                    control_horizon_speed: 0.,
                    control_horizon_altitude: 0.,
                    control_horizon_range: 0.,
                    control_horizon_specific_energy: 0.,
                    control_horizon_alignment_distance: 0.,
                    has_first_s_turn_reversal: false,
                    first_s_turn_reversal_is_final: false,
                    first_s_turn_reversal_ut: 0.,
                    first_s_turn_reversal_range: 0.,
                    first_s_turn_reversal_sign: 0.,
                    s_turn_reversals: 0,
                    s_turn_max_bank: 0.,
                    final_state: VehicleState {
                        ut: 0.,
                        position: Vector3 { x: 0., y: 0., z: 0. },
                        velocity: Vector3 { x: 0., y: 0., z: 0. },
                        mass: 0.,
                    },
                    physics_observed_seconds: 0.,
                    physics_fallback_seconds: 0.,
                    shadow_guidance_used: false,
                    shadow_terminal_prediction_valid: false,
                    shadow_terminal_policy_feasible: false,
                    shadow_terminal_path_committed: false,
                    shadow_final_approach_captured: false,
                    shadow_aborted: false,
                    terminal_survivability_stress_score: 0.,
                    physics_relative_uncertainty: 0.,
                    uncertainty_scenarios: 0,
                },
                energy_margin: DeorbitEnergyMargin {
                    valid: false,
                    impact_along_track: 0.,
                    impact_cross_track: 0.,
                    impact_range: 0.,
                    impact_altitude: 0.,
                    impact_speed: 0.,
                    impact_runway_end: 0,
                },
                tier: 0,
                safety_violation: 0.,
                energy_deficit: 0.,
                energy_excess: 0.,
                entry_fpa_error: 0.,
                entry_course_error: 0.,
                entry_groundtrack_crossrange_error: 0.,
                entry_range_error: 0.,
                handoff_alignment_utilization: 0.,
                handoff_range_error: 0.,
                executable_witness_evaluated: false,
                executable_witness_present: false,
                executable_witness_capture: false,
                executable_witness_handoff_valid: false,
                executable_witness_boundary_missed: false,
                executable_witness_aborted: false,
                executable_witness_taem_error: 0.,
                executable_witness_range_error: 0.,
                executable_witness_energy_margin: 0.,
                executable_witness_alignment_error: 0.,
                executable_witness_closest_distance: 0.,
            };
            stressed.after = after;
            stressed.peri = peri;
            stressed.dv = scenario_dv;
            stressed.duration = duration;
            stressed.burn_ut = start + duration * 0.5f64;
            pr = deorbit_executable_policy_witness(
                &raw mut stressed,
                p,
                aero,
                env,
                &raw mut scenario_cal,
                site,
                v,
                s,
                authority_source,
                terminal_model,
                true_0 != 0,
                false_0 != 0,
            );
        } else {
            pr = predictor_simulate_entry(
                after,
                p,
                aero,
                env,
                &raw mut scenario_cal,
                v,
                site,
                s,
                (*v).maximum_bank_angle,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                4500 as ::core::ffi::c_int as ::core::ffi::c_double,
                false_0 != 0,
            );
        }
        let mut base_pass: bool = burn_completed as ::core::ffi::c_int != 0
            && deorbit_runway_capture_qualified(&raw mut pr, site, v, s, peri)
                as ::core::ffi::c_int != 0;
        let mut base_recovery: bool = base_pass;
        let mut energy_margin: DeorbitEnergyMargin = DeorbitEnergyMargin {
            valid: 0 as ::core::ffi::c_int != 0,
            impact_along_track: 0.,
            impact_cross_track: 0.,
            impact_range: 0.,
            impact_altitude: 0.,
            impact_speed: 0.,
            impact_runway_end: 0,
        };
        if base_pass as ::core::ffi::c_int != 0
            || base_recovery as ::core::ffi::c_int != 0
        {
            energy_margin = deorbit_energy_margin_probe(
                after,
                p,
                aero,
                env,
                &raw mut scenario_cal,
                site,
                v,
                s,
            );
        }
        let mut pass: bool = base_pass as ::core::ffi::c_int != 0
            && deorbit_energy_margin_qualified(&raw mut energy_margin, s, true_0 != 0)
                as ::core::ffi::c_int != 0;
        let mut recovery: bool = base_recovery as ::core::ffi::c_int != 0
            && deorbit_energy_margin_qualified(&raw mut energy_margin, s, false_0 != 0)
                as ::core::ffi::c_int != 0;
        let mut entry_stage: bool = deorbit_entry_stage_qualified(
            &raw mut pr,
            v,
            s,
            peri,
            (*site).altitude,
            (*p).atmosphere_depth,
            burn_completed,
        );
        let mut unsafe_0: bool = hard_unsafe(&raw mut pr, peri, v, s, burn_completed);
        let mut align_metric: ::core::ffi::c_double = deorbit_alignment_contract_utilization(
            &raw mut pr,
            site,
            s,
        );
        let mut align_pos: ::core::ffi::c_double = deorbit_alignment_position_error(
            &raw mut pr,
            s,
        );
        let mut align_course: ::core::ffi::c_double = deorbit_alignment_course_error(
            &raw mut pr,
            site,
        );
        if trace {
            fprintf(
                stderr,
                b"stress[%zu] pass=%d recovery=%d unsafe=%d timing=%+.1f dv=%+.1f atm=%+.2f aeroD=%.2f aeroL=%.2f entry=%.1fkm/%.2fdeg gate=%d %.1fkm x=%.1f y=%.1f course=%.1f FPA=%.2f V=%.0f align=%.1fkm/%.1fdeg h=%.1fkm V=%.0f reached=%d min=%.0f rev=%u closest=%.1fkm Pe=%.1fkm q=%.1fkPa g=%.2f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                i,
                if pass as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if recovery as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                if unsafe_0 as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                sc.timing_offset,
                sc.dv_offset,
                sc.atmosphere_fraction,
                scenario_cal.stress_drag_scale,
                scenario_cal.stress_lift_scale,
                pr.entry_range / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.entry_flight_path_angle,
                if pr.mm304_gate_recorded as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                pr.mm304_gate_range
                    / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.mm304_gate_along_track
                    / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.mm304_gate_cross_track
                    / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.mm304_gate_course,
                pr.mm304_gate_flight_path_angle,
                pr.mm304_gate_speed,
                align_pos / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                align_course,
                pr.taem_altitude / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.taem_speed,
                if pr.reached_taem as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                pr.minimum_entry_speed,
                pr.s_turn_reversals,
                pr.closest_distance
                    / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                peri / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.peak_dynamic_pressure
                    / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
                pr.peak_g_load,
            );
            fprintf(
                stderr,
                b"  energy-margin impact valid=%d along=%+.1fkm cross=%+.1fkm range=%.1fkm V=%.0f\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if energy_margin.valid as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                energy_margin.impact_along_track / 1000.0f64,
                energy_margin.impact_cross_track / 1000.0f64,
                energy_margin.impact_range / 1000.0f64,
                energy_margin.impact_speed,
            );
        }
        r.scenarios = r.scenarios.wrapping_add(1);
        if pass {
            r.passed = r.passed.wrapping_add(1);
        }
        if recovery {
            r.recovery_passed = r.recovery_passed.wrapping_add(1);
        }
        if entry_stage {
            r.entry_stage_passed = r.entry_stage_passed.wrapping_add(1);
        }
        if unsafe_0 {
            r.unsafe_0 = r.unsafe_0.wrapping_add(1);
        }
        if pr.closest_distance.is_finite() as i32 != 0 {
            r.worst_closest = fmax(r.worst_closest, pr.closest_distance);
        }
        if pr.taem_range_error.is_finite() as i32 != 0 {
            r.worst_taem_error = fmax(r.worst_taem_error, fabs(pr.taem_range_error));
        }
        if align_metric.is_finite() as i32 != 0 {
            r.worst_alignment_error = fmax(r.worst_alignment_error, align_metric);
        }
        if pr.entered_atmosphere as ::core::ffi::c_int != 0
            && pr.entry_flight_path_angle.is_finite() as i32 != 0
        {
            r.worst_entry_angle = fmin(r.worst_entry_angle, pr.entry_flight_path_angle);
        }
        if peri.is_finite() as i32 != 0 {
            r.min_periapsis = fmin(r.min_periapsis, peri);
        }
        r.max_dynamic_pressure = fmax(r.max_dynamic_pressure, pr.peak_dynamic_pressure);
        r.max_g_load = fmax(r.max_g_load, pr.peak_g_load);
        entry_prediction_clear(&raw mut pr);
        i = i.wrapping_add(1);
    }
    if r.worst_entry_angle == DBL_MAX {
        r.worst_entry_angle = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    if r.min_periapsis == DBL_MAX {
        r.min_periapsis = -DBL_MAX;
    }
    r.pass_fraction = if r.scenarios != 0 {
        r.passed as ::core::ffi::c_double / r.scenarios as ::core::ffi::c_double
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    r.recovery_pass_fraction = if r.scenarios != 0 {
        r.recovery_passed as ::core::ffi::c_double / r.scenarios as ::core::ffi::c_double
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    r.entry_stage_pass_fraction = if r.scenarios != 0 {
        r.entry_stage_passed as ::core::ffi::c_double
            / r.scenarios as ::core::ffi::c_double
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    r.qualified = (*c).executable_witness_capture as ::core::ffi::c_int != 0
        && r.pass_fraction + 1e-9f64 >= (*s).deorbit_robust_minimum_pass_fraction
        && r.unsafe_0 == 0 as ::core::ffi::c_uint;
    r.recovery_qualified = (*c).executable_witness_capture as ::core::ffi::c_int != 0
        && deorbit_energy_margin_qualified(
            &raw const (*c).energy_margin,
            s,
            false_0 != 0,
        ) as ::core::ffi::c_int != 0
        && r.recovery_pass_fraction + 1e-9f64
            >= (*s).deorbit_robust_minimum_pass_fraction
        && r.unsafe_0 == 0 as ::core::ffi::c_uint;
    r.entry_stage_qualified = deorbit_entry_stage_robustness_qualified(
        r.entry_stage_passed,
        r.scenarios,
        r.unsafe_0,
        (*s).deorbit_robust_minimum_pass_fraction,
    );
    return r;
}
unsafe extern "C" fn deorbit_finite_burn_periapsis(
    mut coast: VehicleState,
    mut dv: ::core::ffi::c_double,
    mut normal_angle: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut cal: *const TrajectoryCalibrationModel,
) -> ::core::ffi::c_double {
    let mut duration: ::core::ffi::c_double = 0.0f64;
    let mut delivered: ::core::ffi::c_double = 0.0f64;
    let mut after: VehicleState = retro_burn_scenario(
        coast,
        dv,
        thrust,
        (*s).deorbit_maximum_throttle,
        (*s).deorbit_throttle_ramp_duration,
        p,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        normal_angle,
        &raw mut duration,
        &raw mut delivered,
        if !cal.is_null() {
            (*cal).physics
        } else {
            ::core::ptr::null::<VesselPhysicsModel>()
        },
    );
    let mut tolerance: ::core::ffi::c_double = fmax(
        GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON) * fmax(1.0f64, fabs(dv)),
    );
    if duration.is_finite() as i32 == 0 || delivered.is_finite() as i32 == 0
        || delivered + tolerance < dv
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return predictor_postburn_periapsis(after, p);
}
unsafe extern "C" fn deorbit_solve_target_periapsis(
    mut coast: VehicleState,
    mut normal_angle: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut cal: *const TrajectoryCalibrationModel,
    mut out_dv: *mut ::core::ffi::c_double,
    mut reject: *mut DeorbitRejectReason,
) -> bool {
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_TARGET_INPUT;
    }
    if out_dv.is_null() || p.is_null() || s.is_null() || !(thrust > 0.0f64)
        || (*s).target_post_burn_periapsis_altitude.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut low: ::core::ffi::c_double = 0.0f64;
    let mut speed: ::core::ffi::c_double = vmag(coast.velocity);
    if !(speed > 0.0f64) || speed.is_finite() as i32 == 0 {
        if !reject.is_null() {
            *reject = DEORBIT_REJECT_TARGET_ORBIT;
        }
        return false_0 != 0;
    }
    let mut low_peri: ::core::ffi::c_double = deorbit_finite_burn_periapsis(
        coast,
        low,
        normal_angle,
        p,
        s,
        thrust,
        cal,
    );
    let mut target: ::core::ffi::c_double = (*s).target_post_burn_periapsis_altitude;
    if low_peri.is_finite() as i32 == 0 {
        if !reject.is_null() {
            *reject = DEORBIT_REJECT_TARGET_LOW_STATE;
        }
        return false_0 != 0;
    }
    if low_peri <= target {
        *out_dv = 0.0f64;
        if !reject.is_null() {
            *reject = DEORBIT_REJECT_NONE;
        }
        return true_0 != 0;
    }
    let mut impulse_low: ::core::ffi::c_double = 0.0f64;
    let mut impulse_high: ::core::ffi::c_double = speed;
    let mut direction: Vector3 = burn_direction(
        coast.position,
        coast.velocity,
        0.0f64,
        normal_angle,
    );
    let mut impulse: VehicleState = coast;
    impulse.velocity = vadd(impulse.velocity, vscale(direction, impulse_high));
    let mut impulse_high_peri: ::core::ffi::c_double = predictor_postburn_periapsis(
        impulse,
        p,
    );
    if impulse_high_peri.is_finite() as i32 == 0 || impulse_high_peri > target {
        if !reject.is_null() {
            *reject = DEORBIT_REJECT_TARGET_IMPULSE_BOUND;
        }
        return false_0 != 0;
    }
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 48 as ::core::ffi::c_int {
        let mut mid: ::core::ffi::c_double = 0.5f64 * (impulse_low + impulse_high);
        impulse = coast;
        impulse.velocity = vadd(coast.velocity, vscale(direction, mid));
        let mut peri: ::core::ffi::c_double = predictor_postburn_periapsis(impulse, p);
        if peri.is_finite() as i32 == 0 {
            if !reject.is_null() {
                *reject = DEORBIT_REJECT_TARGET_IMPULSE_ROOT;
            }
            return false_0 != 0;
        }
        if peri > target {
            impulse_low = mid;
        } else {
            impulse_high = mid;
        }
        i += 1;
    }
    let mut high: ::core::ffi::c_double = impulse_high;
    let mut high_peri: ::core::ffi::c_double = deorbit_finite_burn_periapsis(
        coast,
        high,
        normal_angle,
        p,
        s,
        thrust,
        cal,
    );
    if high_peri.is_finite() as i32 == 0 {
        if !reject.is_null() {
            *reject = DEORBIT_REJECT_TARGET_FINITE_SEED;
        }
        return false_0 != 0;
    }
    while high_peri > target {
        low = high;
        if high >= speed {
            if !reject.is_null() {
                *reject = DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            }
            return false_0 != 0;
        }
        high = fmin(speed, high * 2.0f64);
        if !(high > low) {
            if !reject.is_null() {
                *reject = DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            }
            return false_0 != 0;
        }
        high_peri = deorbit_finite_burn_periapsis(
            coast,
            high,
            normal_angle,
            p,
            s,
            thrust,
            cal,
        );
        if high_peri.is_finite() as i32 == 0 {
            if !reject.is_null() {
                *reject = DEORBIT_REJECT_TARGET_FINITE_BRACKET;
            }
            return false_0 != 0;
        }
    }
    let dv_bracket_tolerance: ::core::ffi::c_double = fmin(
        0.01f64,
        0.125f64 * GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
    ) as ::core::ffi::c_double;
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 48 as ::core::ffi::c_int && high - low > dv_bracket_tolerance {
        let mut mid_0: ::core::ffi::c_double = 0.5f64 * (low + high);
        let mut peri_0: ::core::ffi::c_double = deorbit_finite_burn_periapsis(
            coast,
            mid_0,
            normal_angle,
            p,
            s,
            thrust,
            cal,
        );
        if peri_0.is_finite() as i32 == 0 {
            if !reject.is_null() {
                *reject = DEORBIT_REJECT_TARGET_FINITE_ROOT;
            }
            return false_0 != 0;
        }
        if peri_0 > target {
            low = mid_0;
        } else {
            high = mid_0;
        }
        i_0 += 1;
    }
    *out_dv = 0.5f64 * (low + high);
    let mut solved: bool = (*out_dv).is_finite() as i32 != 0 && *out_dv >= 0.0f64;
    if solved as ::core::ffi::c_int != 0 && !reject.is_null() {
        *reject = DEORBIT_REJECT_NONE;
    }
    return solved;
}
unsafe extern "C" fn deorbit_evaluate_start(
    mut out: *mut Candidate,
    mut initial: VehicleState,
    mut burn_start: ::core::ffi::c_double,
    mut dv_offset: ::core::ffi::c_double,
    mut normal_angle: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut reject: *mut DeorbitRejectReason,
) -> bool {
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_START;
    }
    if out.is_null() || burn_start.is_finite() as i32 == 0 || burn_start < initial.ut {
        return false_0 != 0;
    }
    let mut coast: VehicleState = predictor_propagate_vacuum(
        initial,
        burn_start,
        p,
        12 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut dv: ::core::ffi::c_double = 0.0f64;
    if !deorbit_solve_target_periapsis(
        coast,
        normal_angle,
        p,
        s,
        thrust,
        cal,
        &raw mut dv,
        reject,
    ) {
        return false_0 != 0;
    }
    dv = fmax(0.0f64, dv + dv_offset);
    return eval_candidate(
        out,
        coast,
        dv,
        normal_angle,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        thrust,
        reject,
    );
}
unsafe extern "C" fn copy_control_authority_evidence(
    mut destination: *mut Telemetry,
    mut source: *const Telemetry,
) {
    if destination.is_null() || source.is_null() {
        return;
    }
    (*destination).attitude_response = (*source).attitude_response;
    memcpy(
        &raw mut (*destination).physics_authority as *mut ::core::ffi::c_double
            as *mut ::core::ffi::c_void,
        &raw const (*source).physics_authority as *const ::core::ffi::c_double
            as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 3]>() as size_t,
    );
    memcpy(
        &raw mut (*destination).physics_authority_confidence
            as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
        &raw const (*source).physics_authority_confidence as *const ::core::ffi::c_double
            as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 3]>() as size_t,
    );
    (*destination).has_torque = (*source).has_torque;
    (*destination).available_pitch_torque = (*source).available_pitch_torque;
    (*destination).available_roll_torque = (*source).available_roll_torque;
    (*destination).available_yaw_torque = (*source).available_yaw_torque;
    (*destination).has_inertia = (*source).has_inertia;
    (*destination).pitch_moment_of_inertia = (*source).pitch_moment_of_inertia;
    (*destination).roll_moment_of_inertia = (*source).roll_moment_of_inertia;
    (*destination).yaw_moment_of_inertia = (*source).yaw_moment_of_inertia;
}
unsafe extern "C" fn deorbit_executable_policy_witness(
    mut c: *const Candidate,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut authority_source: *const Telemetry,
    mut terminal_model: *const TerminalModel,
    mut run_terminal: bool,
    mut include_trajectory: bool,
) -> EntryPrediction {
    let mut empty: EntryPrediction = EntryPrediction {
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        taem_distance: 0.,
        taem_range_error: 0.,
        closest_distance: 0.,
        entered_atmosphere: false,
        reached_taem: false,
        taem_ownership_boundary_missed: false,
        taem_dynamic_interface_captured: false,
        taem_dynamic_interface_target_valid: false,
        entry_range: 0.,
        entry_flight_path_angle: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        taem_speed: 0.,
        taem_flight_path_angle: 0.,
        taem_altitude: 0.,
        taem_desired_altitude: 0.,
        taem_along_track: 0.,
        taem_cross_track: 0.,
        taem_course: 0.,
        mm304_gate_recorded: false,
        mm304_gate_range: 0.,
        mm304_gate_along_track: 0.,
        mm304_gate_cross_track: 0.,
        mm304_gate_course: 0.,
        mm304_gate_altitude: 0.,
        mm304_gate_speed: 0.,
        mm304_gate_flight_path_angle: 0.,
        taem_energy_error: 0.,
        taem_interface_error: 0.,
        taem_interface_energy_margin: 0.,
        peak_dynamic_pressure: 0.,
        peak_g_load: 0.,
        minimum_entry_speed: 0.,
        maximum_abs_angle_of_attack: 0.,
        minimum_bank_control_margin: 0.,
        minimum_aoa_control_margin: 0.,
        control_horizon_recorded: false,
        control_horizon_speed: 0.,
        control_horizon_altitude: 0.,
        control_horizon_range: 0.,
        control_horizon_specific_energy: 0.,
        control_horizon_alignment_distance: 0.,
        has_first_s_turn_reversal: false,
        first_s_turn_reversal_is_final: false,
        first_s_turn_reversal_ut: 0.,
        first_s_turn_reversal_range: 0.,
        first_s_turn_reversal_sign: 0.,
        s_turn_reversals: 0,
        s_turn_max_bank: 0.,
        final_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        physics_observed_seconds: 0.,
        physics_fallback_seconds: 0.,
        shadow_guidance_used: false,
        shadow_terminal_prediction_valid: false,
        shadow_terminal_policy_feasible: false,
        shadow_terminal_path_committed: false,
        shadow_final_approach_captured: false,
        shadow_aborted: false,
        terminal_survivability_stress_score: 0.,
        physics_relative_uncertainty: 0.,
        uncertainty_scenarios: 0,
    };
    memset(
        &raw mut empty as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
    trajectory_init(&raw mut empty.trajectory);
    if c.is_null() || p.is_null() || env.is_null() || cal.is_null() || site.is_null()
        || v.is_null() || s.is_null()
    {
        return empty;
    }
    let mut cfg: LandingConfiguration = landing_configuration_default();
    cfg.site = *site;
    cfg.vehicle = *v;
    cfg.guidance = *s;
    let mut response: PredictorGuidance = PredictorGuidance {
        sign: 0.,
        leg_elapsed: 0.,
        established: false,
        entry_loaded: false,
        configured_leg_dwell_satisfied: false,
        reversals: 0,
        limiter: JerkLimiter {
            value: 0.,
            rate: 0.,
            has_value: false,
        },
        has_first_reversal: false,
        first_reversal_final: false,
        final_heading_lock: false,
        first_reversal_ut: 0.,
        first_reversal_range: 0.,
        first_reversal_sign: 0.,
        replay_event: false,
        replay_event_final: false,
        replay_event_ut: 0.,
        replay_event_sign: 0.,
        entry_score: 0.,
        has_entry_score: false,
        actual_bank: 0.,
        actual_bank_rate: 0.,
        actual_aoa: 0.,
        actual_aoa_rate: 0.,
        entry_speed: 0.,
        has_interface_target: false,
        enforce_terminal_delivery_budget: false,
        interface_target: TaemInterfaceTarget {
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
        force_control: false,
        forced_until_ut: 0.,
        control_horizon_ut: 0.,
        forced_bank: 0.,
        forced_aoa: 0.,
        speedbrake: SpeedbrakeController {
            integral: 0.,
            deployed: false,
            initialized: false,
        },
    };
    memset(
        &raw mut response as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PredictorGuidance>() as size_t,
    );
    response.actual_bank = 0.0f64;
    response.actual_aoa = 0.0f64;
    let mut ctx: AeroContext = AeroContext {
        p: p,
        e: env,
        c: cal,
        v: &raw mut cfg.vehicle,
        entry_table: ::core::ptr::null::<TerminalModel>(),
        bank: 0.0f64,
        aoa: 0.0f64,
        ut: (*c).after.ut,
        mass: (*c).after.mass,
        gear: if !(*cal).physics.is_null() {
            (*(*cal).physics).gear as ::core::ffi::c_int
        } else {
            false_0
        } != 0,
        brakes: if !(*cal).physics.is_null() {
            (*(*cal).physics).brakes as ::core::ffi::c_int
        } else {
            false_0
        } != 0,
        airbrakes: 0 as ::core::ffi::c_int,
    };
    let mut seed: Telemetry = guidance_shadow_telemetry(
        ::core::ptr::null::<Telemetry>(),
        (*c).after,
        &raw mut response,
        &raw mut ctx,
        p,
        &raw mut cfg,
        ::core::ptr::null_mut::<AtmosState>(),
    );
    copy_control_authority_evidence(&raw mut seed, authority_source);
    let mut guidance: GuidanceMachine = GuidanceMachine {
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
    };
    guidance_initialize_reentry_continuation(
        &raw mut guidance,
        &raw mut seed,
        p,
        &raw mut cfg,
        1.0f64,
        false_0 != 0,
        env,
        cal,
    );
    let mut executed: DeorbitPlan = DeorbitPlan {
        created_ut: 0.,
        burn_ut: 0.,
        delta_v: 0.,
        estimated_burn_duration: 0.,
        predicted_taem_distance: 0.,
        predicted_taem_range_error: 0.,
        predicted_closest_distance: 0.,
        planning_mass: 0.,
        planning_available_thrust: 0.,
        burn_normal_angle: 0.,
        predicted_entry_range: 0.,
        predicted_entry_flight_path_angle: 0.,
        predicted_post_burn_periapsis_altitude: 0.,
        nominal_capture_achieved: false,
        robustness_qualified: false,
        target_capture_achieved: false,
        terminal_model_snapshot_id: 0,
        terminal_replay_evaluated: false,
        terminal_inlet_qualified: false,
        terminal_native_path_committed: false,
        terminal_final_approach_captured: false,
        terminal_guidance_aborted: false,
        terminal_inlet_energy_margin: 0.,
        robustness_scenarios: 0,
        robustness_passed: 0,
        robustness_unsafe: 0,
        robustness_pass_fraction: 0.,
        worst_case_closest_distance: 0.,
        worst_case_taem_range_error: 0.,
        execution_qualified: false,
        execution_degraded: false,
        entry_stage_qualified: false,
        entry_stage_scenarios: 0,
        entry_stage_passed: 0,
        entry_stage_pass_fraction: 0.,
        recovery_passed: 0,
        recovery_pass_fraction: 0.,
        worst_case_entry_flight_path_angle: 0.,
        worst_case_post_burn_periapsis_altitude: 0.,
        worst_case_peak_dynamic_pressure: 0.,
        worst_case_peak_g_load: 0.,
        live_cutoff_capture_qualified: false,
        live_cutoff_periapsis_altitude: 0.,
        live_cutoff_closest_distance: 0.,
        live_cutoff_entry_flight_path_angle: 0.,
        achieved_state_verified: false,
        achieved_state_capture_qualified: false,
        achieved_post_burn_periapsis_altitude: 0.,
        confidence: 0.,
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        note: [0; 384],
    };
    memset(
        &raw mut executed as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
    trajectory_init(&raw mut executed.trajectory);
    executed.created_ut = (*c).after.ut;
    executed.burn_ut = (*c).burn_ut;
    executed.delta_v = (*c).dv;
    executed.estimated_burn_duration = (*c).duration;
    executed.predicted_post_burn_periapsis_altitude = (*c).peri;
    executed.live_cutoff_capture_qualified = true_0 != 0;
    executed.execution_qualified = true_0 != 0;
    executed.achieved_state_verified = true_0 != 0;
    executed.achieved_state_capture_qualified = true_0 != 0;
    let mut witness: EntryPrediction = EntryPrediction {
        trajectory: Trajectory {
            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
            count: 0,
            capacity: 0,
        },
        taem_distance: 0.,
        taem_range_error: 0.,
        closest_distance: 0.,
        entered_atmosphere: false,
        reached_taem: false,
        taem_ownership_boundary_missed: false,
        taem_dynamic_interface_captured: false,
        taem_dynamic_interface_target_valid: false,
        entry_range: 0.,
        entry_flight_path_angle: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        taem_speed: 0.,
        taem_flight_path_angle: 0.,
        taem_altitude: 0.,
        taem_desired_altitude: 0.,
        taem_along_track: 0.,
        taem_cross_track: 0.,
        taem_course: 0.,
        mm304_gate_recorded: false,
        mm304_gate_range: 0.,
        mm304_gate_along_track: 0.,
        mm304_gate_cross_track: 0.,
        mm304_gate_course: 0.,
        mm304_gate_altitude: 0.,
        mm304_gate_speed: 0.,
        mm304_gate_flight_path_angle: 0.,
        taem_energy_error: 0.,
        taem_interface_error: 0.,
        taem_interface_energy_margin: 0.,
        peak_dynamic_pressure: 0.,
        peak_g_load: 0.,
        minimum_entry_speed: 0.,
        maximum_abs_angle_of_attack: 0.,
        minimum_bank_control_margin: 0.,
        minimum_aoa_control_margin: 0.,
        control_horizon_recorded: false,
        control_horizon_speed: 0.,
        control_horizon_altitude: 0.,
        control_horizon_range: 0.,
        control_horizon_specific_energy: 0.,
        control_horizon_alignment_distance: 0.,
        has_first_s_turn_reversal: false,
        first_s_turn_reversal_is_final: false,
        first_s_turn_reversal_ut: 0.,
        first_s_turn_reversal_range: 0.,
        first_s_turn_reversal_sign: 0.,
        s_turn_reversals: 0,
        s_turn_max_bank: 0.,
        final_state: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        physics_observed_seconds: 0.,
        physics_fallback_seconds: 0.,
        shadow_guidance_used: false,
        shadow_terminal_prediction_valid: false,
        shadow_terminal_policy_feasible: false,
        shadow_terminal_path_committed: false,
        shadow_final_approach_captured: false,
        shadow_aborted: false,
        terminal_survivability_stress_score: 0.,
        physics_relative_uncertainty: 0.,
        uncertainty_scenarios: 0,
    };
    let mut sts_table_witness: bool = (*s).taem_energy_law as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    if (run_terminal as ::core::ffi::c_int != 0
        || sts_table_witness as ::core::ffi::c_int != 0) && !terminal_model.is_null()
        && (*terminal_model).replay_validated as ::core::ffi::c_int != 0
        && (*terminal_model).planet_frame_aligned as ::core::ffi::c_int != 0
    {
        let mut outcome: TerminalShadowOutcome = TerminalShadowOutcome {
            terminal_prediction_valid: false,
            terminal_policy_feasible: false,
            terminal_path_committed: false,
            final_approach_captured: false,
            aborted: false,
        };
        witness = predictor_simulate_guidance_shadow_core(
            (*c).after,
            &raw mut seed,
            &raw mut guidance,
            &raw mut executed,
            p,
            aero,
            env,
            cal,
            &raw mut cfg,
            4500.0f64,
            include_trajectory,
            false_0 != 0,
            !run_terminal,
            terminal_model,
            &raw mut outcome,
        );
        witness.uncertainty_scenarios = if witness.shadow_guidance_used
            as ::core::ffi::c_int != 0
        {
            1 as ::core::ffi::c_uint
        } else {
            0 as ::core::ffi::c_uint
        };
    } else {
        witness = predictor_simulate_entry_guidance_shadow(
            (*c).after,
            &raw mut seed,
            &raw mut guidance,
            &raw mut executed,
            p,
            aero,
            env,
            cal,
            &raw mut cfg,
            4500.0f64,
            include_trajectory,
        );
    }
    trajectory_clear(&raw mut executed.trajectory);
    return witness;
}
unsafe extern "C" fn deorbit_record_execution_witness(
    mut c: *mut Candidate,
    mut witness: *const EntryPrediction,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) {
    if c.is_null() || witness.is_null() || site.is_null() || v.is_null() || s.is_null() {
        return;
    }
    let mut executable: bool = (*witness).shadow_guidance_used as ::core::ffi::c_int != 0
        && (*witness).entered_atmosphere as ::core::ffi::c_int != 0
        && (*witness).mm304_gate_recorded as ::core::ffi::c_int != 0
        && !(*witness).shadow_aborted;
    (*c).executable_witness_evaluated = true_0 != 0;
    (*c).executable_witness_present = (*witness).shadow_guidance_used;
    (*c).executable_witness_capture = executable as ::core::ffi::c_int != 0
        && deorbit_runway_capture_qualified(witness, site, v, s, (*c).peri)
            as ::core::ffi::c_int != 0;
    (*c).executable_witness_handoff_valid = (*witness).reached_taem as ::core::ffi::c_int
        != 0 && !(*witness).taem_ownership_boundary_missed;
    (*c).executable_witness_boundary_missed = (*witness).taem_ownership_boundary_missed;
    (*c).executable_witness_aborted = (*witness).shadow_aborted;
    (*c).executable_witness_taem_error = if (*witness).taem_interface_error.is_finite()
        as i32 != 0
    {
        (*witness).taem_interface_error
    } else if (*witness).taem_range_error.is_finite() as i32 != 0 {
        fabs((*witness).taem_range_error)
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    (*c).executable_witness_range_error = if (*witness).taem_range_error.is_finite()
        as i32 != 0
    {
        (*witness).taem_range_error
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    (*c).executable_witness_alignment_error = deorbit_alignment_contract_utilization(
        witness,
        site,
        s,
    );
    (*c).executable_witness_closest_distance = if (*witness).closest_distance.is_finite()
        as i32 != 0
    {
        (*witness).closest_distance
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn deorbit_witness_closing(
    mut w: *const EntryPrediction,
    mut site: *const LandingSite,
) -> bool {
    if w.is_null() || site.is_null() || (*w).taem_along_track.is_finite() as i32 == 0
        || (*w).taem_cross_track.is_finite() as i32 == 0
        || (*w).taem_course.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut relative: ::core::ffi::c_double = ((*w).taem_course - (*site).runway_heading)
        * DEG2RAD;
    return (*w).taem_along_track * cos(relative) + (*w).taem_cross_track * sin(relative)
        < 0.0f64;
}
unsafe extern "C" fn deorbit_witness_range_debt(
    mut w: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if w.is_null() || (*w).taem_distance.is_finite() as i32 == 0
        || !((*w).taem_distance >= 0.0f64)
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut minimum: ::core::ffi::c_double = fmax(
        (*s).final_approach_distance * 1.5f64,
        (*s).final_approach_distance + fmax(0.0f64, (*s).hac_radius),
    );
    let mut maximum: ::core::ffi::c_double = entry_taem_range_target(p, s);
    return fmax(
        0.0f64,
        fmax(minimum - (*w).taem_distance, (*w).taem_distance - maximum),
    );
}
unsafe extern "C" fn deorbit_witness_inlet_geometry(
    mut w: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
) -> bool {
    if w.is_null() || !(*w).reached_taem || (*w).taem_altitude.is_finite() as i32 == 0
        || (*w).taem_speed.is_finite() as i32 == 0 || (*w).taem_speed <= 0.0f64
        || (*w).taem_altitude < (*s).mm305_min_altitude
        || (*w).taem_altitude > (*s).mm305_max_altitude
        || (*w).taem_course.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut heading_offset: ::core::ffi::c_double = fabs(
        norm_signed_deg((*w).taem_course - (*site).runway_heading),
    );
    return deorbit_witness_range_debt(w, p, s) <= 0.0f64
        && fabs(heading_offset - 90.0f64) <= (*s).mm304_perpendicular_heading_half_width
        && deorbit_witness_closing(w, site) as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn deorbit_witness_terminal_trial(
    mut candidate: *const Candidate,
    mut w: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
) -> bool {
    return !candidate.is_null() && !w.is_null()
        && deorbit_entry_stage_qualified(
            w,
            v,
            s,
            (*candidate).peri,
            (*site).altitude,
            (*p).atmosphere_depth,
            true_0 != 0,
        ) as ::core::ffi::c_int != 0 && (*w).reached_taem as ::core::ffi::c_int != 0
        && (*w).taem_altitude.is_finite() as i32 != 0
        && (*w).taem_altitude >= (*s).mm305_min_altitude
        && (*w).taem_altitude <= (*s).mm305_max_altitude
        && deorbit_witness_range_debt(w, p, s) <= 0.0f64
        && deorbit_witness_closing(w, site) as ::core::ffi::c_int != 0;
}
unsafe extern "C" fn deorbit_guided_terminal_miss(
    mut candidate: *const Candidate,
    mut w: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
) -> ::core::ffi::c_double {
    if !(*candidate).executable_witness_evaluated || !(*w).shadow_guidance_used
        || (*w).shadow_aborted as ::core::ffi::c_int != 0
        || (*w).final_state.ut.is_finite() as i32 == 0
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut end: GeoPoint = predictor_geo_point(
        (*w).final_state.position,
        p,
        (*w).final_state.ut,
    );
    if end.altitude.is_finite() as i32 == 0 || end.altitude > (*site).altitude + 750.0f64
    {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut target: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut miss: ::core::ffi::c_double = great_circle_distance(
        end,
        target,
        (*p).radius,
    );
    return if miss.is_finite() as i32 != 0 {
        miss
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
}
unsafe extern "C" fn deorbit_sts_handoff_target(
    mut model: *const TerminalModel,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut mass: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if model.is_null() || p.is_null() || site.is_null() || v.is_null() || s.is_null() {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if (*s).taem_hac_law {
        let mut live: TaemHacLiveAero = TaemHacLiveAero {
            drag: [
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
                0.,
            ],
            lift: [0.; 15],
            weight: [0.; 15],
            last_ut: 0.,
        };
        let mut h: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
        let mut speed: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
        let mut station: ::core::ffi::c_double = ::core::f32::NAN
            as ::core::ffi::c_double;
        if !mm305_hac_handoff_state(
            model,
            &raw mut live,
            mass,
            (*s).mm305_target_mach,
            &raw mut h,
            &raw mut speed,
            &raw mut station,
        ) {
            return ::core::f32::NAN as ::core::ffi::c_double;
        }
        return station
            + final_alignment_distance_for_state(
                ::core::ptr::null::<GuidanceMachine>(),
                h + (*site).altitude,
                speed,
                site,
                v,
                s,
            );
    }
    let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        (*s).taem_interface_altitude,
    );
    let mut reach: ::core::ffi::c_double = mm305_energy_law_handoff_reach(
        model as *const TerminalModel,
        (*s).taem_interface_altitude - (*site).altitude,
        (*s).mm305_target_mach * sound,
        mass,
    );
    return if reach.is_finite() as i32 != 0 {
        reach + (*s).final_approach_distance
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
}
unsafe extern "C" fn deorbit_guided_candidate_better(
    mut a: *const Candidate,
    mut aw: *const EntryPrediction,
    mut b: *const Candidate,
    mut bw: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut sts_handoff_target: ::core::ffi::c_double,
) -> bool {
    let mut a_safe: bool = deorbit_entry_stage_qualified(
        aw,
        v,
        s,
        (*a).peri,
        (*site).altitude,
        (*p).atmosphere_depth,
        true_0 != 0,
    );
    let mut b_safe: bool = deorbit_entry_stage_qualified(
        bw,
        v,
        s,
        (*b).peri,
        (*site).altitude,
        (*p).atmosphere_depth,
        true_0 != 0,
    );
    if a_safe as ::core::ffi::c_int != b_safe as ::core::ffi::c_int {
        return a_safe;
    }
    let mut a_capture: bool = a_safe as ::core::ffi::c_int != 0
        && deorbit_runway_capture_qualified(aw, site, v, s, (*a).peri)
            as ::core::ffi::c_int != 0
        && deorbit_energy_margin_qualified(&raw const (*a).energy_margin, s, true_0 != 0)
            as ::core::ffi::c_int != 0;
    let mut b_capture: bool = b_safe as ::core::ffi::c_int != 0
        && deorbit_runway_capture_qualified(bw, site, v, s, (*b).peri)
            as ::core::ffi::c_int != 0
        && deorbit_energy_margin_qualified(&raw const (*b).energy_margin, s, true_0 != 0)
            as ::core::ffi::c_int != 0;
    if a_capture as ::core::ffi::c_int != b_capture as ::core::ffi::c_int {
        return a_capture;
    }
    let mut a_inlet: bool = (*aw).taem_dynamic_interface_captured as ::core::ffi::c_int
        != 0 && !(*aw).taem_ownership_boundary_missed;
    let mut b_inlet: bool = (*bw).taem_dynamic_interface_captured as ::core::ffi::c_int
        != 0 && !(*bw).taem_ownership_boundary_missed;
    if a_inlet as ::core::ffi::c_int != b_inlet as ::core::ffi::c_int {
        return a_inlet;
    }
    if ((*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
        || (*s).taem_hac_law as ::core::ffi::c_int != 0)
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 && sts_handoff_target.is_finite() as i32 != 0
        && (*aw).reached_taem as ::core::ffi::c_int != 0
        && (*bw).reached_taem as ::core::ffi::c_int != 0
        && (*aw).taem_distance.is_finite() as i32 != 0
        && (*bw).taem_distance.is_finite() as i32 != 0
    {
        let mut ae: ::core::ffi::c_double = fabs(
            (*aw).taem_distance - sts_handoff_target,
        );
        let mut be: ::core::ffi::c_double = fabs(
            (*bw).taem_distance - sts_handoff_target,
        );
        if fabs(ae - be) > 1.0f64 {
            return ae < be;
        }
    }
    let mut a_geometry: bool = deorbit_witness_inlet_geometry(aw, p, site, s);
    let mut b_geometry: bool = deorbit_witness_inlet_geometry(bw, p, site, s);
    if a_geometry as ::core::ffi::c_int != b_geometry as ::core::ffi::c_int {
        return a_geometry;
    }
    if !a_geometry {
        let mut ad: ::core::ffi::c_double = deorbit_witness_range_debt(aw, p, s);
        let mut bd: ::core::ffi::c_double = deorbit_witness_range_debt(bw, p, s);
        if ad < bd {
            return true_0 != 0;
        }
        if ad > bd {
            return false_0 != 0;
        }
        let mut ac: bool = deorbit_witness_closing(aw, site);
        let mut bc: bool = deorbit_witness_closing(bw, site);
        if ac as ::core::ffi::c_int != bc as ::core::ffi::c_int {
            return ac;
        }
    }
    let mut ae_0: bool = (*aw).taem_interface_energy_margin.is_finite() as i32 != 0
        && (*aw).taem_interface_energy_margin >= 0.0f64;
    let mut be_0: bool = (*bw).taem_interface_energy_margin.is_finite() as i32 != 0
        && (*bw).taem_interface_energy_margin >= 0.0f64;
    if ae_0 as ::core::ffi::c_int != be_0 as ::core::ffi::c_int {
        return ae_0;
    }
    if !ae_0 && (*aw).taem_interface_energy_margin.is_finite() as i32 != 0
        && (*bw).taem_interface_energy_margin.is_finite() as i32 != 0
        && fabs((*aw).taem_interface_energy_margin - (*bw).taem_interface_energy_margin)
            > 1.0f64
    {
        return (*aw).taem_interface_energy_margin > (*bw).taem_interface_energy_margin;
    }
    if a_safe as ::core::ffi::c_int != 0 && b_safe as ::core::ffi::c_int != 0
        && (*a).executable_witness_evaluated as ::core::ffi::c_int != 0
        && (*b).executable_witness_evaluated as ::core::ffi::c_int != 0
    {
        let mut am: ::core::ffi::c_double = deorbit_guided_terminal_miss(a, aw, p, site);
        let mut bm: ::core::ffi::c_double = deorbit_guided_terminal_miss(b, bw, p, site);
        if am.is_finite() as i32 != 0 && bm.is_finite() as i32 != 0
            && fabs(am - bm) > 1.0f64
        {
            return am < bm;
        }
    }
    let mut nominal: ::core::ffi::c_double = entry_taem_range_target(p, s);
    let mut ad_0: ::core::ffi::c_double = fabs((*aw).taem_distance - nominal);
    let mut bd_0: ::core::ffi::c_double = fabs((*bw).taem_distance - nominal);
    if ad_0 < bd_0 {
        return true_0 != 0;
    }
    if ad_0 > bd_0 {
        return false_0 != 0;
    }
    return (*a).burn_ut < (*b).burn_ut;
}
unsafe extern "C" fn deorbit_timing_probe_entry(
    mut out: *mut DeorbitTimingProbe,
    mut state: VehicleState,
    mut p: *const PlanetModel,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
) -> bool {
    if out.is_null() || p.is_null() || site.is_null() || s.is_null() {
        return false_0 != 0;
    }
    let mut target: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut entry_alt: ::core::ffi::c_double = entry_guidance_start_altitude(p, s);
    let mut elapsed: ::core::ffi::c_double = 0.0f64;
    while elapsed < 4500.0f64 {
        let mut geo: GeoPoint = predictor_geo_point(state.position, p, state.ut);
        let mut up: Vector3 = vnorm(
            state.position,
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        );
        let mut air: Vector3 = vsub(
            state.velocity,
            vcross(planet_rotation_vector(p), state.position),
        );
        let mut speed: ::core::ffi::c_double = vmag(air);
        let mut vertical: ::core::ffi::c_double = vdot(air, up);
        if geo.altitude <= entry_alt && vertical < 0.0f64 {
            let mut horizontal: ::core::ffi::c_double = vmag(vproject_plane(air, up));
            let mut bearing: ::core::ffi::c_double = initial_bearing(geo, target);
            let mut course: ::core::ffi::c_double = surface_course(
                state.position,
                state.velocity,
                planet_rotation_vector(p),
                (*p).north_axis,
                bearing,
            );
            let mut signed_error: ::core::ffi::c_double = norm_signed_deg(
                bearing - course,
            );
            let mut range: ::core::ffi::c_double = great_circle_distance(
                geo,
                target,
                (*p).radius,
            );
            let mut central: ::core::ffi::c_double = range / fmax((*p).radius, DBL_MIN);
            let mut cross_sine: ::core::ffi::c_double = sin(central)
                * sin(signed_error * DEG2RAD);
            (*out).entry_range = range;
            (*out).entry_fpa = atan2(vertical, fmax(horizontal, 0.1f64)) * RAD2DEG;
            (*out).entry_speed = speed;
            (*out).entry_course_error = fabs(signed_error);
            (*out).entry_crossrange = fabs(
                asin(clampd(cross_sine, -1.0f64, 1.0f64)) * (*p).radius,
            );
            (*out).entry_miss = hypot(
                (*out).entry_crossrange,
                range - deorbit_effective_entry_range(s, p, (*out).entry_speed),
            );
            (*out).closing = (*out).entry_course_error < 90.0f64;
            (*out).valid = (*out).entry_range.is_finite() as i32 != 0
                && (*out).entry_fpa.is_finite() as i32 != 0
                && (*out).entry_speed.is_finite() as i32 != 0
                && (*out).entry_course_error.is_finite() as i32 != 0
                && (*out).entry_crossrange.is_finite() as i32 != 0
                && (*out).entry_miss.is_finite() as i32 != 0;
            return (*out).valid;
        }
        if geo.altitude.is_finite() as i32 == 0 || geo.altitude < -1000.0f64 {
            return false_0 != 0;
        }
        let mut excess: ::core::ffi::c_double = geo.altitude - entry_alt;
        let mut dt: ::core::ffi::c_double = if excess > 10000.0f64 {
            8.0f64
        } else if excess > 2000.0f64 {
            2.0f64
        } else {
            0.25f64
        };
        state = rk4_gravity(state, dt, p);
        elapsed += dt;
    }
    return false_0 != 0;
}
unsafe extern "C" fn deorbit_timing_probe_start(
    mut out: *mut DeorbitTimingProbe,
    mut initial: VehicleState,
    mut burn_start: ::core::ffi::c_double,
    mut normal_angle: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut reject: *mut DeorbitRejectReason,
) -> bool {
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_START;
    }
    if out.is_null() || burn_start.is_finite() as i32 == 0 || burn_start < initial.ut {
        return false_0 != 0;
    }
    memset(
        out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitTimingProbe>() as size_t,
    );
    let mut coast: VehicleState = predictor_propagate_vacuum(
        initial,
        burn_start,
        p,
        12.0f64,
    );
    let mut dv: ::core::ffi::c_double = 0.0f64;
    if !deorbit_solve_target_periapsis(
        coast,
        normal_angle,
        p,
        s,
        thrust,
        cal,
        &raw mut dv,
        reject,
    ) {
        return false_0 != 0;
    }
    let mut duration: ::core::ffi::c_double = 0.0f64;
    let mut delivered: ::core::ffi::c_double = 0.0f64;
    let mut after: VehicleState = retro_burn_scenario(
        coast,
        dv,
        thrust,
        (*s).deorbit_maximum_throttle,
        (*s).deorbit_throttle_ramp_duration,
        p,
        0.0f64,
        normal_angle,
        &raw mut duration,
        &raw mut delivered,
        if !cal.is_null() {
            (*cal).physics
        } else {
            ::core::ptr::null::<VesselPhysicsModel>()
        },
    );
    let mut tolerance: ::core::ffi::c_double = fmax(
        GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS,
        sqrt(DBL_EPSILON) * fmax(1.0f64, fabs(dv)),
    );
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_FINITE_BURN;
    }
    if duration.is_finite() as i32 == 0 || delivered.is_finite() as i32 == 0
        || delivered + tolerance < dv
    {
        return false_0 != 0;
    }
    let mut peri: ::core::ffi::c_double = predictor_postburn_periapsis(after, p);
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_POSTBURN_PERIAPSIS;
    }
    if peri.is_finite() as i32 == 0 || peri >= (*p).atmosphere_depth
        || peri <= -(*p).radius
    {
        return false_0 != 0;
    }
    (*out).burn_start = burn_start;
    (*out).normal_angle = normal_angle;
    (*out).burn_ut = coast.ut + duration * 0.5f64;
    (*out).dv = dv;
    (*out).duration = duration;
    (*out).peri = peri;
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_ENTRY_PREDICTION;
    }
    if !deorbit_timing_probe_entry(out, after, p, site, s) {
        return false_0 != 0;
    }
    if !reject.is_null() {
        *reject = DEORBIT_REJECT_NONE;
    }
    return true_0 != 0;
}
unsafe extern "C" fn deorbit_timing_probe_better(
    mut a: *const DeorbitTimingProbe,
    mut b: *const DeorbitTimingProbe,
    mut hasb: bool,
    mut s: *const GuidanceSettings,
) -> bool {
    if a.is_null() || !(*a).valid {
        return false_0 != 0;
    }
    if !hasb || b.is_null() || !(*b).valid {
        return true_0 != 0;
    }
    if (*a).closing as ::core::ffi::c_int != (*b).closing as ::core::ffi::c_int {
        return (*a).closing;
    }
    let mut a_capture: bool = (*a).entry_miss <= (*s).target_deorbit_capture_radius;
    let mut b_capture: bool = (*b).entry_miss <= (*s).target_deorbit_capture_radius;
    if a_capture as ::core::ffi::c_int != b_capture as ::core::ffi::c_int {
        return a_capture;
    }
    if a_capture as ::core::ffi::c_int != 0 && (*a).burn_start != (*b).burn_start
        && fabs((*a).burn_start - (*b).burn_start) > 1200.0f64
    {
        return (*a).burn_start < (*b).burn_start;
    }
    if (*a).entry_miss < (*b).entry_miss {
        return true_0 != 0;
    }
    if (*a).entry_miss > (*b).entry_miss {
        return false_0 != 0;
    }
    if (*a).entry_crossrange < (*b).entry_crossrange {
        return true_0 != 0;
    }
    if (*a).entry_crossrange > (*b).entry_crossrange {
        return false_0 != 0;
    }
    let mut af: ::core::ffi::c_double = fabs(
        (*a).entry_fpa - (*s).target_entry_flight_path_angle,
    );
    let mut bf: ::core::ffi::c_double = fabs(
        (*b).entry_fpa - (*s).target_entry_flight_path_angle,
    );
    if af < bf {
        return true_0 != 0;
    }
    if af > bf {
        return false_0 != 0;
    }
    if (*a).entry_course_error < (*b).entry_course_error {
        return true_0 != 0;
    }
    if (*a).entry_course_error > (*b).entry_course_error {
        return false_0 != 0;
    }
    return (*a).burn_start < (*b).burn_start;
}
unsafe extern "C" fn deorbit_search_horizon(
    mut state: VehicleState,
    mut period: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut s: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    let mut search_horizon: ::core::ffi::c_double = period;
    let mut angular_momentum: Vector3 = vcross(state.position, state.velocity);
    let mut angular_momentum_magnitude: ::core::ffi::c_double = vmag(angular_momentum);
    if angular_momentum_magnitude.is_finite() as i32 != 0
        && angular_momentum_magnitude > DBL_EPSILON
        && (*p).rotational_speed.is_finite() as i32 != 0
    {
        let mut orbit_normal: Vector3 = vscale(
            angular_momentum,
            1.0f64 / angular_momentum_magnitude,
        );
        let mut north: Vector3 = vnorm((*p).north_axis, v3(0.0f64, 0.0f64, 1.0f64));
        let mut orbital_rate: ::core::ffi::c_double = 2.0f64 * acos(-1.0f64) / period;
        let mut ground_relative_rate: ::core::ffi::c_double = orbital_rate
            - (*p).rotational_speed * vdot(orbit_normal, north);
        if ground_relative_rate.is_finite() as i32 != 0
            && fabs(ground_relative_rate) > DBL_EPSILON * fmax(1.0f64, orbital_rate)
        {
            let mut target_phase_period: ::core::ffi::c_double = 2.0f64 * acos(-1.0f64)
                / fabs(ground_relative_rate);
            if target_phase_period.is_finite() as i32 != 0
                && target_phase_period > 0.0f64
            {
                search_horizon = fmax(period, target_phase_period);
            }
        }
        let mut radius: ::core::ffi::c_double = vmag(state.position);
        let mut energy: ::core::ffi::c_double = 0.5f64
            * vdot(state.velocity, state.velocity)
            - (*p).gravitational_parameter / fmax(radius, DBL_MIN);
        let mut eccentricity: Vector3 = vsub(
            vscale(
                vcross(state.velocity, angular_momentum),
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    / (*p).gravitational_parameter,
            ),
            vnorm(
                state.position,
                v3(
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            ),
        );
        let mut semimajor: ::core::ffi::c_double = if energy
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            -(*p).gravitational_parameter
                / (2 as ::core::ffi::c_int as ::core::ffi::c_double * energy)
        } else {
            ::core::f32::INFINITY as ::core::ffi::c_double
        };
        let mut apsis_span: ::core::ffi::c_double = 2 as ::core::ffi::c_int
            as ::core::ffi::c_double * semimajor * vmag(eccentricity);
        if fabs((*p).rotational_speed) > DBL_EPSILON
            && ((*p).radius * vmag(vcross(orbit_normal, north))
                > (*s).target_deorbit_capture_radius
                || apsis_span > (*s).target_deorbit_capture_radius)
        {
            let mut rotation_period: ::core::ffi::c_double = 2.0f64 * acos(-1.0f64)
                / fabs((*p).rotational_speed);
            if rotation_period.is_finite() as i32 != 0 && rotation_period > 0.0f64 {
                search_horizon = fmax(search_horizon, rotation_period);
            }
        }
    }
    return search_horizon;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_plan_create_with_terminal_model(
    mut out: *mut DeorbitPlan,
    mut state: VehicleState,
    mut period: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut authority_source: *const Telemetry,
    mut terminal_model: *const TerminalModel,
) -> bool {
    if out.is_null() || p.is_null() || env.is_null() || cal.is_null() || site.is_null()
        || v.is_null() || s.is_null() || period.is_finite() as i32 == 0
        || !(period > 0.0f64) || !(thrust > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut native_terminal: bool = terminal_model_policy_matches(
        terminal_model,
        site,
        v,
        s,
    ) as ::core::ffi::c_int != 0
        && terminal_model_planet_matches(terminal_model, p) as ::core::ffi::c_int != 0;
    let mut entry_model: *const TerminalModel = if native_terminal as ::core::ffi::c_int
        != 0 && (*s).taem_energy_law as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*v).model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        terminal_model
    } else {
        ::core::ptr::null::<TerminalModel>()
    };
    memset(
        out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
    trajectory_init(&raw mut (*out).trajectory);
    let mut sts_handoff_target: ::core::ffi::c_double = if !entry_model.is_null()
        && ((*s).sts_trajectory_delivery as ::core::ffi::c_int != 0
            || (*s).taem_hac_law as ::core::ffi::c_int != 0)
    {
        deorbit_sts_handoff_target(entry_model, p, site, v, s, state.mass)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    if !getenv(
            b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"DEORBIT_STS_HANDOFF_TARGET %.1fkm\n\0" as *const u8
                as *const ::core::ffi::c_char,
            sts_handoff_target / 1000.0f64,
        );
    }
    let mut lead: ::core::ffi::c_double = fmax(0.0f64, (*s).minimum_planning_lead_time);
    let mut search_horizon: ::core::ffi::c_double = deorbit_search_horizon(
        state,
        period,
        p,
        s,
    );
    let time_samples: ::core::ffi::c_int = 361 as ::core::ffi::c_int;
    let mut coarse_step: ::core::ffi::c_double = search_horizon
        / (time_samples - 1 as ::core::ffi::c_int) as ::core::ffi::c_double;
    let mut rejection_counts: [::core::ffi::c_uint; 14] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_uint,
        0,
        0,
        0,
        0,
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
    let mut timing_best: DeorbitTimingProbe = DeorbitTimingProbe {
        valid: 0 as ::core::ffi::c_int != 0,
        closing: false,
        burn_start: 0.,
        burn_ut: 0.,
        dv: 0.,
        duration: 0.,
        peri: 0.,
        normal_angle: 0.,
        entry_range: 0.,
        entry_fpa: 0.,
        entry_speed: 0.,
        entry_course_error: 0.,
        entry_crossrange: 0.,
        entry_miss: 0.,
    };
    let mut timing_valid: bool = false_0 != 0;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < time_samples {
        let mut offset: ::core::ffi::c_double = lead
            + search_horizon * i as ::core::ffi::c_double
                / (time_samples - 1 as ::core::ffi::c_int) as ::core::ffi::c_double;
        let mut probe: DeorbitTimingProbe = DeorbitTimingProbe {
            valid: 0 as ::core::ffi::c_int != 0,
            closing: false,
            burn_start: 0.,
            burn_ut: 0.,
            dv: 0.,
            duration: 0.,
            peri: 0.,
            normal_angle: 0.,
            entry_range: 0.,
            entry_fpa: 0.,
            entry_speed: 0.,
            entry_course_error: 0.,
            entry_crossrange: 0.,
            entry_miss: 0.,
        };
        let mut rejection: DeorbitRejectReason = DEORBIT_REJECT_NONE;
        if !deorbit_timing_probe_start(
            &raw mut probe,
            state,
            state.ut + offset,
            0.0f64,
            p,
            cal,
            site,
            s,
            thrust,
            &raw mut rejection,
        ) {
            if rejection as ::core::ffi::c_uint
                > DEORBIT_REJECT_NONE as ::core::ffi::c_int as ::core::ffi::c_uint
                && (rejection as ::core::ffi::c_uint)
                    < DEORBIT_REJECT_COUNT as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                rejection_counts[rejection as usize] = rejection_counts[rejection
                        as usize]
                    .wrapping_add(1);
            }
        } else {
            if !getenv(
                    b"KSP_LANDER_TRACE_DEORBIT_CANDIDATES\0" as *const u8
                        as *const ::core::ffi::c_char,
                )
                .is_null()
            {
                fprintf(
                    stderr,
                    b"DEORBIT_TIMING sample=%d start=%.2f burn=%.2f dv=%.2f entry=%.1fkm cross=%.1fkm miss=%.1fkm courseErr=%.2f FPA=%.2f closing=%d\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    i,
                    probe.burn_start,
                    probe.burn_ut,
                    probe.dv,
                    probe.entry_range / 1000.0f64,
                    probe.entry_crossrange / 1000.0f64,
                    probe.entry_miss / 1000.0f64,
                    probe.entry_course_error,
                    probe.entry_fpa,
                    if probe.closing as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                );
            }
            if deorbit_timing_probe_better(
                &raw mut probe,
                &raw mut timing_best,
                timing_valid,
                s,
            ) {
                timing_best = probe;
                timing_valid = true_0 != 0;
            }
        }
        i += 1;
    }
    if !timing_valid {
        if !getenv(
                b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        {
            fprintf(
                stderr,
                b"Deorbit timing search rejected all burn starts: start=%u target-input=%u target-orbit=%u target-low-state=%u target-impulse-bound=%u target-impulse-root=%u target-finite-seed=%u target-finite-bracket=%u target-finite-root=%u impulsive-periapsis=%u finite-burn=%u postburn-periapsis=%u entry-prediction=%u.\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                rejection_counts[DEORBIT_REJECT_START as ::core::ffi::c_int as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_INPUT as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_ORBIT as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_LOW_STATE as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_IMPULSE_BOUND
                    as ::core::ffi::c_int as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_IMPULSE_ROOT as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_FINITE_SEED as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_FINITE_BRACKET
                    as ::core::ffi::c_int as usize],
                rejection_counts[DEORBIT_REJECT_TARGET_FINITE_ROOT as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_IMPULSIVE_PERIAPSIS as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_FINITE_BURN as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_POSTBURN_PERIAPSIS as ::core::ffi::c_int
                    as usize],
                rejection_counts[DEORBIT_REJECT_ENTRY_PREDICTION as ::core::ffi::c_int
                    as usize],
            );
        }
        return false_0 != 0;
    }
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"coarse-search\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    let mut time_step: ::core::ffi::c_double = coarse_step;
    let mut iteration: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while iteration < 9 as ::core::ffi::c_int {
        let mut iteration_best: DeorbitTimingProbe = timing_best;
        let mut direction: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
        while direction <= 1 as ::core::ffi::c_int {
            let mut start: ::core::ffi::c_double = timing_best.burn_start
                + direction as ::core::ffi::c_double * time_step;
            if !(start < state.ut + lead || start > state.ut + lead + search_horizon) {
                let mut probe_0: DeorbitTimingProbe = DeorbitTimingProbe {
                    valid: 0 as ::core::ffi::c_int != 0,
                    closing: false,
                    burn_start: 0.,
                    burn_ut: 0.,
                    dv: 0.,
                    duration: 0.,
                    peri: 0.,
                    normal_angle: 0.,
                    entry_range: 0.,
                    entry_fpa: 0.,
                    entry_speed: 0.,
                    entry_course_error: 0.,
                    entry_crossrange: 0.,
                    entry_miss: 0.,
                };
                if deorbit_timing_probe_start(
                    &raw mut probe_0,
                    state,
                    start,
                    0.0f64,
                    p,
                    cal,
                    site,
                    s,
                    thrust,
                    ::core::ptr::null_mut::<DeorbitRejectReason>(),
                ) {
                    if deorbit_timing_probe_better(
                        &raw mut probe_0,
                        &raw mut iteration_best,
                        true_0 != 0,
                        s,
                    ) {
                        iteration_best = probe_0;
                    }
                }
            }
            direction += 2 as ::core::ffi::c_int;
        }
        if deorbit_timing_probe_better(
            &raw mut iteration_best,
            &raw mut timing_best,
            true_0 != 0,
            s,
        ) {
            timing_best = iteration_best;
        }
        time_step *= 0.5f64;
        iteration += 1;
    }
    if !getenv(
            b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"DEORBIT_TIMING_SELECTED start=%.3f burn=%.3f dv=%.2f entry=%.1fkm cross=%.1fkm miss=%.1fkm courseErr=%.2f FPA=%.2f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            timing_best.burn_start,
            timing_best.burn_ut,
            timing_best.dv,
            timing_best.entry_range / 1000.0f64,
            timing_best.entry_crossrange / 1000.0f64,
            timing_best.entry_miss / 1000.0f64,
            timing_best.entry_course_error,
            timing_best.entry_fpa,
        );
    }
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"timing-refine\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    let mut angle_step: ::core::ffi::c_double = 45.0f64;
    let mut iteration_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while iteration_0 < 12 as ::core::ffi::c_int {
        let mut improved: DeorbitTimingProbe = timing_best;
        let mut sign: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
        while sign <= 1 as ::core::ffi::c_int {
            let mut angle: ::core::ffi::c_double = clampd(
                timing_best.normal_angle + sign as ::core::ffi::c_double * angle_step,
                -45.0f64,
                45.0f64,
            );
            let mut probe_1: DeorbitTimingProbe = DeorbitTimingProbe {
                valid: 0 as ::core::ffi::c_int != 0,
                closing: false,
                burn_start: 0.,
                burn_ut: 0.,
                dv: 0.,
                duration: 0.,
                peri: 0.,
                normal_angle: 0.,
                entry_range: 0.,
                entry_fpa: 0.,
                entry_speed: 0.,
                entry_course_error: 0.,
                entry_crossrange: 0.,
                entry_miss: 0.,
            };
            if deorbit_timing_probe_start(
                &raw mut probe_1,
                state,
                timing_best.burn_start,
                angle,
                p,
                cal,
                site,
                s,
                thrust,
                ::core::ptr::null_mut::<DeorbitRejectReason>(),
            ) as ::core::ffi::c_int != 0 && probe_1.closing as ::core::ffi::c_int != 0
                && probe_1.entry_miss < improved.entry_miss
            {
                improved = probe_1;
            }
            sign += 2 as ::core::ffi::c_int;
        }
        timing_best = improved;
        angle_step *= 0.5f64;
        iteration_0 += 1;
    }
    let mut selected: Candidate = Candidate {
        burn_ut: 0.,
        dv: 0.,
        duration: 0.,
        peri: 0.,
        requested_periapsis_altitude: 0.,
        score: 0.,
        burn_normal_angle: 0.,
        after: VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        },
        pred: EntryPrediction {
            trajectory: Trajectory {
                points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                count: 0,
                capacity: 0,
            },
            taem_distance: 0.,
            taem_range_error: 0.,
            closest_distance: 0.,
            entered_atmosphere: false,
            reached_taem: false,
            taem_ownership_boundary_missed: false,
            taem_dynamic_interface_captured: false,
            taem_dynamic_interface_target_valid: false,
            entry_range: 0.,
            entry_flight_path_angle: 0.,
            entry_speed: 0.,
            entry_course_error: 0.,
            taem_speed: 0.,
            taem_flight_path_angle: 0.,
            taem_altitude: 0.,
            taem_desired_altitude: 0.,
            taem_along_track: 0.,
            taem_cross_track: 0.,
            taem_course: 0.,
            mm304_gate_recorded: false,
            mm304_gate_range: 0.,
            mm304_gate_along_track: 0.,
            mm304_gate_cross_track: 0.,
            mm304_gate_course: 0.,
            mm304_gate_altitude: 0.,
            mm304_gate_speed: 0.,
            mm304_gate_flight_path_angle: 0.,
            taem_energy_error: 0.,
            taem_interface_error: 0.,
            taem_interface_energy_margin: 0.,
            peak_dynamic_pressure: 0.,
            peak_g_load: 0.,
            minimum_entry_speed: 0.,
            maximum_abs_angle_of_attack: 0.,
            minimum_bank_control_margin: 0.,
            minimum_aoa_control_margin: 0.,
            control_horizon_recorded: false,
            control_horizon_speed: 0.,
            control_horizon_altitude: 0.,
            control_horizon_range: 0.,
            control_horizon_specific_energy: 0.,
            control_horizon_alignment_distance: 0.,
            has_first_s_turn_reversal: false,
            first_s_turn_reversal_is_final: false,
            first_s_turn_reversal_ut: 0.,
            first_s_turn_reversal_range: 0.,
            first_s_turn_reversal_sign: 0.,
            s_turn_reversals: 0,
            s_turn_max_bank: 0.,
            final_state: VehicleState {
                ut: 0.,
                position: Vector3 { x: 0., y: 0., z: 0. },
                velocity: Vector3 { x: 0., y: 0., z: 0. },
                mass: 0.,
            },
            physics_observed_seconds: 0.,
            physics_fallback_seconds: 0.,
            shadow_guidance_used: false,
            shadow_terminal_prediction_valid: false,
            shadow_terminal_policy_feasible: false,
            shadow_terminal_path_committed: false,
            shadow_final_approach_captured: false,
            shadow_aborted: false,
            terminal_survivability_stress_score: 0.,
            physics_relative_uncertainty: 0.,
            uncertainty_scenarios: 0,
        },
        energy_margin: DeorbitEnergyMargin {
            valid: false,
            impact_along_track: 0.,
            impact_cross_track: 0.,
            impact_range: 0.,
            impact_altitude: 0.,
            impact_speed: 0.,
            impact_runway_end: 0,
        },
        tier: 0,
        safety_violation: 0.,
        energy_deficit: 0.,
        energy_excess: 0.,
        entry_fpa_error: 0.,
        entry_course_error: 0.,
        entry_groundtrack_crossrange_error: 0.,
        entry_range_error: 0.,
        handoff_alignment_utilization: 0.,
        handoff_range_error: 0.,
        executable_witness_evaluated: false,
        executable_witness_present: false,
        executable_witness_capture: false,
        executable_witness_handoff_valid: false,
        executable_witness_boundary_missed: false,
        executable_witness_aborted: false,
        executable_witness_taem_error: 0.,
        executable_witness_range_error: 0.,
        executable_witness_energy_margin: 0.,
        executable_witness_alignment_error: 0.,
        executable_witness_closest_distance: 0.,
    };
    memset(
        &raw mut selected as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<Candidate>() as size_t,
    );
    trajectory_init(&raw mut selected.pred.trajectory);
    if !deorbit_evaluate_start(
        &raw mut selected,
        state,
        timing_best.burn_start,
        0.0f64,
        timing_best.normal_angle,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        thrust,
        ::core::ptr::null_mut::<DeorbitRejectReason>(),
    ) {
        candidate_clear(&raw mut selected);
        return false_0 != 0;
    }
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"angle-search\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    let mut selected_inlet: EntryPrediction = deorbit_executable_policy_witness(
        &raw mut selected,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        authority_source,
        entry_model,
        false_0 != 0,
        false_0 != 0,
    );
    if native_terminal as ::core::ffi::c_int != 0
        && deorbit_witness_terminal_trial(
            &raw mut selected,
            &raw mut selected_inlet,
            p,
            site,
            v,
            s,
        ) as ::core::ffi::c_int != 0
    {
        let mut full: EntryPrediction = deorbit_executable_policy_witness(
            &raw mut selected,
            p,
            aero,
            env,
            cal,
            site,
            v,
            s,
            authority_source,
            terminal_model,
            true_0 != 0,
            false_0 != 0,
        );
        deorbit_record_execution_witness(&raw mut selected, &raw mut full, site, v, s);
        entry_prediction_clear(&raw mut selected_inlet);
        selected_inlet = full;
    }
    let mut inlet_candidates_evaluated: ::core::ffi::c_uint = 1 as ::core::ffi::c_uint;
    let mut scan_dt: [::core::ffi::c_double; 16] = [0.; 16];
    let mut scan_range: [::core::ffi::c_double; 16] = [0.; 16];
    let mut scan_count: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    if selected_inlet.taem_distance.is_finite() as i32 != 0
        && selected_inlet.reached_taem as ::core::ffi::c_int != 0
    {
        scan_dt[scan_count as usize] = 0.0f64;
        let fresh0 = scan_count;
        scan_count = scan_count + 1;
        scan_range[fresh0 as usize] = selected_inlet.taem_distance;
    }
    let mut scan_origin: ::core::ffi::c_double = timing_best.burn_start;
    let mut inlet_step: ::core::ffi::c_double = (*s).target_deorbit_capture_radius
        / fmax(1.0f64, timing_best.entry_speed);
    if inlet_step.is_finite() as i32 == 0 || inlet_step <= 0.0f64 {
        inlet_step = 0.0f64;
    }
    let mut last_offset: ::core::ffi::c_int = 2 as ::core::ffi::c_int;
    let mut offset_0: ::core::ffi::c_int = -(2 as ::core::ffi::c_int);
    while offset_0 <= last_offset && inlet_step > 0.0f64 {
        if !(offset_0 == 0 as ::core::ffi::c_int) {
            let mut start_0: ::core::ffi::c_double = timing_best.burn_start
                + offset_0 as ::core::ffi::c_double * inlet_step;
            if !(start_0 < state.ut + lead || start_0 > state.ut + lead + search_horizon)
            {
                let mut nearby: Candidate = Candidate {
                    burn_ut: 0.,
                    dv: 0.,
                    duration: 0.,
                    peri: 0.,
                    requested_periapsis_altitude: 0.,
                    score: 0.,
                    burn_normal_angle: 0.,
                    after: VehicleState {
                        ut: 0.,
                        position: Vector3 { x: 0., y: 0., z: 0. },
                        velocity: Vector3 { x: 0., y: 0., z: 0. },
                        mass: 0.,
                    },
                    pred: EntryPrediction {
                        trajectory: Trajectory {
                            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                            count: 0,
                            capacity: 0,
                        },
                        taem_distance: 0.,
                        taem_range_error: 0.,
                        closest_distance: 0.,
                        entered_atmosphere: false,
                        reached_taem: false,
                        taem_ownership_boundary_missed: false,
                        taem_dynamic_interface_captured: false,
                        taem_dynamic_interface_target_valid: false,
                        entry_range: 0.,
                        entry_flight_path_angle: 0.,
                        entry_speed: 0.,
                        entry_course_error: 0.,
                        taem_speed: 0.,
                        taem_flight_path_angle: 0.,
                        taem_altitude: 0.,
                        taem_desired_altitude: 0.,
                        taem_along_track: 0.,
                        taem_cross_track: 0.,
                        taem_course: 0.,
                        mm304_gate_recorded: false,
                        mm304_gate_range: 0.,
                        mm304_gate_along_track: 0.,
                        mm304_gate_cross_track: 0.,
                        mm304_gate_course: 0.,
                        mm304_gate_altitude: 0.,
                        mm304_gate_speed: 0.,
                        mm304_gate_flight_path_angle: 0.,
                        taem_energy_error: 0.,
                        taem_interface_error: 0.,
                        taem_interface_energy_margin: 0.,
                        peak_dynamic_pressure: 0.,
                        peak_g_load: 0.,
                        minimum_entry_speed: 0.,
                        maximum_abs_angle_of_attack: 0.,
                        minimum_bank_control_margin: 0.,
                        minimum_aoa_control_margin: 0.,
                        control_horizon_recorded: false,
                        control_horizon_speed: 0.,
                        control_horizon_altitude: 0.,
                        control_horizon_range: 0.,
                        control_horizon_specific_energy: 0.,
                        control_horizon_alignment_distance: 0.,
                        has_first_s_turn_reversal: false,
                        first_s_turn_reversal_is_final: false,
                        first_s_turn_reversal_ut: 0.,
                        first_s_turn_reversal_range: 0.,
                        first_s_turn_reversal_sign: 0.,
                        s_turn_reversals: 0,
                        s_turn_max_bank: 0.,
                        final_state: VehicleState {
                            ut: 0.,
                            position: Vector3 { x: 0., y: 0., z: 0. },
                            velocity: Vector3 { x: 0., y: 0., z: 0. },
                            mass: 0.,
                        },
                        physics_observed_seconds: 0.,
                        physics_fallback_seconds: 0.,
                        shadow_guidance_used: false,
                        shadow_terminal_prediction_valid: false,
                        shadow_terminal_policy_feasible: false,
                        shadow_terminal_path_committed: false,
                        shadow_final_approach_captured: false,
                        shadow_aborted: false,
                        terminal_survivability_stress_score: 0.,
                        physics_relative_uncertainty: 0.,
                        uncertainty_scenarios: 0,
                    },
                    energy_margin: DeorbitEnergyMargin {
                        valid: false,
                        impact_along_track: 0.,
                        impact_cross_track: 0.,
                        impact_range: 0.,
                        impact_altitude: 0.,
                        impact_speed: 0.,
                        impact_runway_end: 0,
                    },
                    tier: 0,
                    safety_violation: 0.,
                    energy_deficit: 0.,
                    energy_excess: 0.,
                    entry_fpa_error: 0.,
                    entry_course_error: 0.,
                    entry_groundtrack_crossrange_error: 0.,
                    entry_range_error: 0.,
                    handoff_alignment_utilization: 0.,
                    handoff_range_error: 0.,
                    executable_witness_evaluated: false,
                    executable_witness_present: false,
                    executable_witness_capture: false,
                    executable_witness_handoff_valid: false,
                    executable_witness_boundary_missed: false,
                    executable_witness_aborted: false,
                    executable_witness_taem_error: 0.,
                    executable_witness_range_error: 0.,
                    executable_witness_energy_margin: 0.,
                    executable_witness_alignment_error: 0.,
                    executable_witness_closest_distance: 0.,
                };
                memset(
                    &raw mut nearby as *mut ::core::ffi::c_void,
                    0 as ::core::ffi::c_int,
                    ::core::mem::size_of::<Candidate>() as size_t,
                );
                trajectory_init(&raw mut nearby.pred.trajectory);
                if !deorbit_evaluate_start(
                    &raw mut nearby,
                    state,
                    start_0,
                    0.0f64,
                    timing_best.normal_angle,
                    p,
                    aero,
                    env,
                    cal,
                    site,
                    v,
                    s,
                    thrust,
                    ::core::ptr::null_mut::<DeorbitRejectReason>(),
                ) {
                    candidate_clear(&raw mut nearby);
                } else {
                    let mut nearby_inlet: EntryPrediction = deorbit_executable_policy_witness(
                        &raw mut nearby,
                        p,
                        aero,
                        env,
                        cal,
                        site,
                        v,
                        s,
                        authority_source,
                        entry_model,
                        false_0 != 0,
                        false_0 != 0,
                    );
                    if native_terminal as ::core::ffi::c_int != 0
                        && deorbit_witness_terminal_trial(
                            &raw mut nearby,
                            &raw mut nearby_inlet,
                            p,
                            site,
                            v,
                            s,
                        ) as ::core::ffi::c_int != 0
                    {
                        let mut full_0: EntryPrediction = deorbit_executable_policy_witness(
                            &raw mut nearby,
                            p,
                            aero,
                            env,
                            cal,
                            site,
                            v,
                            s,
                            authority_source,
                            terminal_model,
                            true_0 != 0,
                            false_0 != 0,
                        );
                        deorbit_record_execution_witness(
                            &raw mut nearby,
                            &raw mut full_0,
                            site,
                            v,
                            s,
                        );
                        entry_prediction_clear(&raw mut nearby_inlet);
                        nearby_inlet = full_0;
                    }
                    inlet_candidates_evaluated = inlet_candidates_evaluated
                        .wrapping_add(1);
                    if scan_count < 16 as ::core::ffi::c_int
                        && nearby_inlet.taem_distance.is_finite() as i32 != 0
                        && nearby_inlet.reached_taem as ::core::ffi::c_int != 0
                    {
                        scan_dt[scan_count as usize] = offset_0 as ::core::ffi::c_double
                            * inlet_step;
                        let fresh1 = scan_count;
                        scan_count = scan_count + 1;
                        scan_range[fresh1 as usize] = nearby_inlet.taem_distance;
                    }
                    if offset_0 == last_offset && last_offset < 8 as ::core::ffi::c_int
                        && sts_handoff_target.is_finite() as i32 != 0
                        && nearby_inlet.reached_taem as ::core::ffi::c_int != 0
                        && nearby_inlet.taem_distance.is_finite() as i32 != 0
                        && nearby_inlet.taem_distance > sts_handoff_target
                    {
                        last_offset += 1;
                    }
                    let mut better: bool = deorbit_guided_candidate_better(
                        &raw mut nearby,
                        &raw mut nearby_inlet,
                        &raw mut selected,
                        &raw mut selected_inlet,
                        p,
                        site,
                        v,
                        s,
                        sts_handoff_target,
                    );
                    if !getenv(
                            b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                                as *const ::core::ffi::c_char,
                        )
                        .is_null()
                    {
                        fprintf(
                            stderr,
                            b"DEORBIT_INLET_CANDIDATE dt=%+.1fs burn=%.1f tier=%d safe=%d range=%.1fkm course=%.1fdeg closing=%d geometry=%d energy=%+.0fJ/kg boundaryMiss=%d chosen=%d\n\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            offset_0 as ::core::ffi::c_double * inlet_step,
                            nearby.burn_ut,
                            nearby.tier,
                            if deorbit_entry_stage_qualified(
                                &raw mut nearby_inlet,
                                v,
                                s,
                                nearby.peri,
                                (*site).altitude,
                                (*p).atmosphere_depth,
                                true_0 != 0,
                            ) as ::core::ffi::c_int != 0
                            {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                            nearby_inlet.taem_distance / 1000.0f64,
                            nearby_inlet.taem_course,
                            if deorbit_witness_closing(&raw mut nearby_inlet, site)
                                as ::core::ffi::c_int != 0
                            {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                            if deorbit_witness_inlet_geometry(
                                &raw mut nearby_inlet,
                                p,
                                site,
                                s,
                            ) as ::core::ffi::c_int != 0
                            {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                            nearby_inlet.taem_interface_energy_margin,
                            if nearby_inlet.taem_ownership_boundary_missed
                                as ::core::ffi::c_int != 0
                            {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                            if better as ::core::ffi::c_int != 0 {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                        );
                    }
                    if better {
                        candidate_clear(&raw mut selected);
                        entry_prediction_clear(&raw mut selected_inlet);
                        selected = nearby;
                        selected_inlet = nearby_inlet;
                    } else {
                        candidate_clear(&raw mut nearby);
                        entry_prediction_clear(&raw mut nearby_inlet);
                    }
                }
            }
        }
        offset_0 += 1;
    }
    let mut nominal: ::core::ffi::c_double = if sts_handoff_target.is_finite() as i32
        != 0
    {
        sts_handoff_target
    } else {
        entry_taem_range_target(p, s)
    };
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < scan_count {
        let mut j: ::core::ffi::c_int = i_0 + 1 as ::core::ffi::c_int;
        while j < scan_count {
            if scan_dt[j as usize] < scan_dt[i_0 as usize] {
                let mut t: ::core::ffi::c_double = scan_dt[i_0 as usize];
                scan_dt[i_0 as usize] = scan_dt[j as usize];
                scan_dt[j as usize] = t;
                t = scan_range[i_0 as usize];
                scan_range[i_0 as usize] = scan_range[j as usize];
                scan_range[j as usize] = t;
            }
            j += 1;
        }
        i_0 += 1;
    }
    let mut bracket: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    let mut i_1: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while (i_1 + 1 as ::core::ffi::c_int) < scan_count {
        if (scan_range[i_1 as usize] - nominal)
            * (scan_range[(i_1 + 1 as ::core::ffi::c_int) as usize] - nominal) < 0.0f64
            && scan_dt[(i_1 + 1 as ::core::ffi::c_int) as usize] - scan_dt[i_1 as usize]
                <= 1.5f64 * inlet_step
        {
            bracket = i_1;
            break;
        } else {
            i_1 += 1;
        }
    }
    let mut lo_dt: ::core::ffi::c_double = if bracket >= 0 as ::core::ffi::c_int {
        scan_dt[bracket as usize]
    } else {
        0.0f64
    };
    let mut lo_r: ::core::ffi::c_double = if bracket >= 0 as ::core::ffi::c_int {
        scan_range[bracket as usize]
    } else {
        0.0f64
    };
    let mut hi_dt: ::core::ffi::c_double = if bracket >= 0 as ::core::ffi::c_int {
        scan_dt[(bracket + 1 as ::core::ffi::c_int) as usize]
    } else {
        0.0f64
    };
    let mut hi_r: ::core::ffi::c_double = if bracket >= 0 as ::core::ffi::c_int {
        scan_range[(bracket + 1 as ::core::ffi::c_int) as usize]
    } else {
        0.0f64
    };
    let mut it: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while bracket >= 0 as ::core::ffi::c_int && it < 3 as ::core::ffi::c_int {
        let mut dt: ::core::ffi::c_double = lo_dt
            + (nominal - lo_r) * (hi_dt - lo_dt) / (hi_r - lo_r);
        dt = clampd(
            dt,
            lo_dt + 0.1f64 * (hi_dt - lo_dt),
            hi_dt - 0.1f64 * (hi_dt - lo_dt),
        );
        let mut start_1: ::core::ffi::c_double = scan_origin + dt;
        if start_1 < state.ut + lead || start_1 > state.ut + lead + search_horizon {
            break;
        }
        let mut refined: Candidate = Candidate {
            burn_ut: 0.,
            dv: 0.,
            duration: 0.,
            peri: 0.,
            requested_periapsis_altitude: 0.,
            score: 0.,
            burn_normal_angle: 0.,
            after: VehicleState {
                ut: 0.,
                position: Vector3 { x: 0., y: 0., z: 0. },
                velocity: Vector3 { x: 0., y: 0., z: 0. },
                mass: 0.,
            },
            pred: EntryPrediction {
                trajectory: Trajectory {
                    points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                    count: 0,
                    capacity: 0,
                },
                taem_distance: 0.,
                taem_range_error: 0.,
                closest_distance: 0.,
                entered_atmosphere: false,
                reached_taem: false,
                taem_ownership_boundary_missed: false,
                taem_dynamic_interface_captured: false,
                taem_dynamic_interface_target_valid: false,
                entry_range: 0.,
                entry_flight_path_angle: 0.,
                entry_speed: 0.,
                entry_course_error: 0.,
                taem_speed: 0.,
                taem_flight_path_angle: 0.,
                taem_altitude: 0.,
                taem_desired_altitude: 0.,
                taem_along_track: 0.,
                taem_cross_track: 0.,
                taem_course: 0.,
                mm304_gate_recorded: false,
                mm304_gate_range: 0.,
                mm304_gate_along_track: 0.,
                mm304_gate_cross_track: 0.,
                mm304_gate_course: 0.,
                mm304_gate_altitude: 0.,
                mm304_gate_speed: 0.,
                mm304_gate_flight_path_angle: 0.,
                taem_energy_error: 0.,
                taem_interface_error: 0.,
                taem_interface_energy_margin: 0.,
                peak_dynamic_pressure: 0.,
                peak_g_load: 0.,
                minimum_entry_speed: 0.,
                maximum_abs_angle_of_attack: 0.,
                minimum_bank_control_margin: 0.,
                minimum_aoa_control_margin: 0.,
                control_horizon_recorded: false,
                control_horizon_speed: 0.,
                control_horizon_altitude: 0.,
                control_horizon_range: 0.,
                control_horizon_specific_energy: 0.,
                control_horizon_alignment_distance: 0.,
                has_first_s_turn_reversal: false,
                first_s_turn_reversal_is_final: false,
                first_s_turn_reversal_ut: 0.,
                first_s_turn_reversal_range: 0.,
                first_s_turn_reversal_sign: 0.,
                s_turn_reversals: 0,
                s_turn_max_bank: 0.,
                final_state: VehicleState {
                    ut: 0.,
                    position: Vector3 { x: 0., y: 0., z: 0. },
                    velocity: Vector3 { x: 0., y: 0., z: 0. },
                    mass: 0.,
                },
                physics_observed_seconds: 0.,
                physics_fallback_seconds: 0.,
                shadow_guidance_used: false,
                shadow_terminal_prediction_valid: false,
                shadow_terminal_policy_feasible: false,
                shadow_terminal_path_committed: false,
                shadow_final_approach_captured: false,
                shadow_aborted: false,
                terminal_survivability_stress_score: 0.,
                physics_relative_uncertainty: 0.,
                uncertainty_scenarios: 0,
            },
            energy_margin: DeorbitEnergyMargin {
                valid: false,
                impact_along_track: 0.,
                impact_cross_track: 0.,
                impact_range: 0.,
                impact_altitude: 0.,
                impact_speed: 0.,
                impact_runway_end: 0,
            },
            tier: 0,
            safety_violation: 0.,
            energy_deficit: 0.,
            energy_excess: 0.,
            entry_fpa_error: 0.,
            entry_course_error: 0.,
            entry_groundtrack_crossrange_error: 0.,
            entry_range_error: 0.,
            handoff_alignment_utilization: 0.,
            handoff_range_error: 0.,
            executable_witness_evaluated: false,
            executable_witness_present: false,
            executable_witness_capture: false,
            executable_witness_handoff_valid: false,
            executable_witness_boundary_missed: false,
            executable_witness_aborted: false,
            executable_witness_taem_error: 0.,
            executable_witness_range_error: 0.,
            executable_witness_energy_margin: 0.,
            executable_witness_alignment_error: 0.,
            executable_witness_closest_distance: 0.,
        };
        memset(
            &raw mut refined as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<Candidate>() as size_t,
        );
        trajectory_init(&raw mut refined.pred.trajectory);
        if !deorbit_evaluate_start(
            &raw mut refined,
            state,
            start_1,
            0.0f64,
            timing_best.normal_angle,
            p,
            aero,
            env,
            cal,
            site,
            v,
            s,
            thrust,
            ::core::ptr::null_mut::<DeorbitRejectReason>(),
        ) {
            candidate_clear(&raw mut refined);
            break;
        } else {
            let mut refined_inlet: EntryPrediction = deorbit_executable_policy_witness(
                &raw mut refined,
                p,
                aero,
                env,
                cal,
                site,
                v,
                s,
                authority_source,
                entry_model,
                false_0 != 0,
                false_0 != 0,
            );
            if native_terminal as ::core::ffi::c_int != 0
                && deorbit_witness_terminal_trial(
                    &raw mut refined,
                    &raw mut refined_inlet,
                    p,
                    site,
                    v,
                    s,
                ) as ::core::ffi::c_int != 0
            {
                let mut full_1: EntryPrediction = deorbit_executable_policy_witness(
                    &raw mut refined,
                    p,
                    aero,
                    env,
                    cal,
                    site,
                    v,
                    s,
                    authority_source,
                    terminal_model,
                    true_0 != 0,
                    false_0 != 0,
                );
                deorbit_record_execution_witness(
                    &raw mut refined,
                    &raw mut full_1,
                    site,
                    v,
                    s,
                );
                entry_prediction_clear(&raw mut refined_inlet);
                refined_inlet = full_1;
            }
            inlet_candidates_evaluated = inlet_candidates_evaluated.wrapping_add(1);
            let mut r: ::core::ffi::c_double = if refined_inlet.reached_taem
                as ::core::ffi::c_int != 0
            {
                refined_inlet.taem_distance
            } else {
                ::core::f32::NAN as ::core::ffi::c_double
            };
            let mut better_0: bool = deorbit_guided_candidate_better(
                &raw mut refined,
                &raw mut refined_inlet,
                &raw mut selected,
                &raw mut selected_inlet,
                p,
                site,
                v,
                s,
                sts_handoff_target,
            );
            if !getenv(
                    b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                        as *const ::core::ffi::c_char,
                )
                .is_null()
            {
                fprintf(
                    stderr,
                    b"DEORBIT_INLET_REFINE dt=%+.1fs burn=%.1f range=%.1fkm geometry=%d chosen=%d\n\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    dt,
                    refined.burn_ut,
                    r / 1000.0f64,
                    if deorbit_witness_inlet_geometry(&raw mut refined_inlet, p, site, s)
                        as ::core::ffi::c_int != 0
                    {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                    if better_0 as ::core::ffi::c_int != 0 {
                        1 as ::core::ffi::c_int
                    } else {
                        0 as ::core::ffi::c_int
                    },
                );
            }
            if better_0 {
                candidate_clear(&raw mut selected);
                entry_prediction_clear(&raw mut selected_inlet);
                selected = refined;
                selected_inlet = refined_inlet;
            } else {
                candidate_clear(&raw mut refined);
                entry_prediction_clear(&raw mut refined_inlet);
            }
            if r.is_finite() as i32 == 0 {
                break;
            }
            if (r - nominal) * (lo_r - nominal) > 0.0f64 {
                lo_dt = dt;
                lo_r = r;
            } else {
                hi_dt = dt;
                hi_r = r;
            }
            if fabs(r - nominal) < 5000.0f64 {
                break;
            }
            it += 1;
        }
    }
    if !getenv(
            b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"DEORBIT_INLET_SELECTED samples=%u burn=%.1f range=%.1fkm energy=%+.0fJ/kg geometry=%d\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            inlet_candidates_evaluated,
            selected.burn_ut,
            selected_inlet.taem_distance / 1000.0f64,
            selected_inlet.taem_interface_energy_margin,
            if deorbit_witness_inlet_geometry(&raw mut selected_inlet, p, site, s)
                as ::core::ffi::c_int != 0
            {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
        );
    }
    entry_prediction_clear(&raw mut selected_inlet);
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"inlet-retime\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"detailed-witness\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    let mut detailed: EntryPrediction = deorbit_executable_policy_witness(
        &raw mut selected,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        authority_source,
        terminal_model,
        native_terminal,
        true_0 != 0,
    );
    deorbit_record_execution_witness(&raw mut selected, &raw mut detailed, site, v, s);
    let mut nominal_capture: bool = native_terminal as ::core::ffi::c_int != 0
        && selected.executable_witness_capture as ::core::ffi::c_int != 0
        && deorbit_energy_margin_qualified(
            &raw mut selected.energy_margin,
            s,
            true_0 != 0,
        ) as ::core::ffi::c_int != 0;
    let mut selected_robust: RobustAssessment = assess_candidate_robustness(
        &raw mut selected,
        state,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        thrust,
        authority_source,
        terminal_model,
        nominal_capture,
        false_0 != 0,
    );
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"robustness\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    if !getenv(b"KSP_LANDER_TRACE_STRESS\0" as *const u8 as *const ::core::ffi::c_char)
        .is_null()
    {
        let mut traced: RobustAssessment = assess_candidate_robustness(
            &raw mut selected,
            state,
            p,
            aero,
            env,
            cal,
            site,
            v,
            s,
            thrust,
            authority_source,
            terminal_model,
            nominal_capture,
            true_0 != 0,
        );
    }
    selected_robust.qualified = selected.executable_witness_capture as ::core::ffi::c_int
        != 0
        && selected_robust.pass_fraction + 1e-9f64
            >= (*s).deorbit_robust_minimum_pass_fraction
        && selected_robust.unsafe_0 == 0 as ::core::ffi::c_uint;
    selected_robust.recovery_qualified = selected.executable_witness_capture
        as ::core::ffi::c_int != 0
        && deorbit_energy_margin_qualified(
            &raw mut selected.energy_margin,
            s,
            false_0 != 0,
        ) as ::core::ffi::c_int != 0
        && selected_robust.recovery_pass_fraction + 1e-9f64
            >= (*s).deorbit_robust_minimum_pass_fraction
        && selected_robust.unsafe_0 == 0 as ::core::ffi::c_uint;
    (*out).created_ut = state.ut;
    (*out).burn_ut = selected.burn_ut;
    (*out).delta_v = selected.dv;
    (*out).burn_normal_angle = selected.burn_normal_angle;
    (*out).estimated_burn_duration = selected.duration;
    (*out).planning_mass = state.mass;
    (*out).planning_available_thrust = thrust;
    (*out).predicted_taem_distance = detailed.taem_distance;
    (*out).predicted_taem_range_error = detailed.taem_range_error;
    (*out).predicted_closest_distance = detailed.closest_distance;
    (*out).predicted_entry_range = detailed.entry_range;
    (*out).predicted_entry_flight_path_angle = detailed.entry_flight_path_angle;
    (*out).predicted_post_burn_periapsis_altitude = selected.peri;
    (*out).terminal_model_snapshot_id = if native_terminal as ::core::ffi::c_int != 0 {
        (*terminal_model).snapshot_id
    } else {
        0 as uint64_t
    };
    (*out).terminal_replay_evaluated = native_terminal as ::core::ffi::c_int != 0
        && detailed.shadow_guidance_used as ::core::ffi::c_int != 0;
    (*out).terminal_inlet_qualified = (*out).terminal_replay_evaluated
        as ::core::ffi::c_int != 0 && detailed.reached_taem as ::core::ffi::c_int != 0
        && detailed.taem_dynamic_interface_captured as ::core::ffi::c_int != 0
        && !detailed.taem_ownership_boundary_missed;
    (*out).terminal_native_path_committed = (*out).terminal_replay_evaluated
        as ::core::ffi::c_int != 0
        && detailed.shadow_terminal_path_committed as ::core::ffi::c_int != 0;
    (*out).terminal_final_approach_captured = (*out).terminal_replay_evaluated
        as ::core::ffi::c_int != 0
        && detailed.shadow_final_approach_captured as ::core::ffi::c_int != 0;
    (*out).terminal_guidance_aborted = (*out).terminal_replay_evaluated
        as ::core::ffi::c_int != 0 && detailed.shadow_aborted as ::core::ffi::c_int != 0;
    (*out).terminal_inlet_energy_margin = if (*out).terminal_replay_evaluated
        as ::core::ffi::c_int != 0
    {
        detailed.taem_interface_energy_margin
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut terminal_end: GeoPoint = predictor_geo_point(
        detailed.final_state.position,
        p,
        detailed.final_state.ut,
    );
    let mut runway_geo: GeoPoint = GeoPoint {
        latitude: (*site).latitude,
        longitude: (*site).longitude,
        altitude: (*site).altitude,
    };
    let mut terminal_end_range: ::core::ffi::c_double = great_circle_distance(
        terminal_end,
        runway_geo,
        (*p).radius,
    );
    if !getenv(
            b"KSP_LANDER_TRACE_DEORBIT_SEARCH\0" as *const u8
                as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"DEORBIT_NATIVE_TERMINAL snapshot=%llu replay=%d inlet=%d committed=%d final=%d aborted=%d energy=%+.0f J/kg strict=%d endUT=%.1f endH=%.0f endSpeed=%.1f endRange=%.1fkm\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*out).terminal_model_snapshot_id as ::core::ffi::c_ulonglong,
            if (*out).terminal_replay_evaluated as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if (*out).terminal_inlet_qualified as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if (*out).terminal_native_path_committed as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if (*out).terminal_final_approach_captured as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if (*out).terminal_guidance_aborted as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            (*out).terminal_inlet_energy_margin,
            if nominal_capture as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            detailed.final_state.ut,
            terminal_end.altitude,
            vmag(detailed.final_state.velocity),
            terminal_end_range / 1000.0f64,
        );
    }
    (*out).nominal_capture_achieved = nominal_capture;
    (*out).robustness_qualified = selected_robust.qualified as ::core::ffi::c_int != 0
        && nominal_capture as ::core::ffi::c_int != 0;
    (*out).target_capture_achieved = (*out).robustness_qualified;
    (*out).robustness_scenarios = selected_robust.scenarios;
    (*out).robustness_passed = selected_robust.passed;
    (*out).robustness_unsafe = selected_robust.unsafe_0;
    (*out).robustness_pass_fraction = selected_robust.pass_fraction;
    (*out).recovery_passed = selected_robust.recovery_passed;
    (*out).recovery_pass_fraction = selected_robust.recovery_pass_fraction;
    (*out).entry_stage_passed = selected_robust.entry_stage_passed;
    (*out).entry_stage_scenarios = selected_robust.scenarios;
    (*out).entry_stage_pass_fraction = selected_robust.entry_stage_pass_fraction;
    let mut nominal_entry_stage: bool = deorbit_entry_stage_qualified(
        &raw mut detailed,
        v,
        s,
        selected.peri,
        (*site).altitude,
        (*p).atmosphere_depth,
        true_0 != 0,
    );
    (*out).entry_stage_qualified = nominal_entry_stage as ::core::ffi::c_int != 0
        && selected_robust.entry_stage_qualified as ::core::ffi::c_int != 0;
    (*out).execution_qualified = (*out).entry_stage_qualified;
    (*out).execution_degraded = (*out).execution_qualified as ::core::ffi::c_int != 0
        && !(*out).target_capture_achieved;
    (*out).worst_case_closest_distance = selected_robust.worst_closest;
    (*out).worst_case_taem_range_error = selected_robust.worst_taem_error;
    (*out).worst_case_entry_flight_path_angle = selected_robust.worst_entry_angle;
    (*out).worst_case_post_burn_periapsis_altitude = selected_robust.min_periapsis;
    (*out).worst_case_peak_dynamic_pressure = selected_robust.max_dynamic_pressure;
    (*out).worst_case_peak_g_load = selected_robust.max_g_load;
    let mut model_confidence: ::core::ffi::c_double = 0.0f64;
    let mut i_2: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_2 < 4 as ::core::ffi::c_int {
        model_confidence += (*env).regimes[i_2 as usize].confidence;
        i_2 += 1;
    }
    model_confidence /= 4.0f64;
    let mut evidence_confidence: ::core::ffi::c_double = fmin(
        model_confidence,
        fmin((*cal).confidence, selected_robust.pass_fraction),
    );
    if selected_robust.unsafe_0 != 0 || !nominal_entry_stage {
        evidence_confidence = 0.0f64;
    }
    (*out).confidence = clampd(evidence_confidence, 0.0f64, 1.0f64);
    (*out).trajectory = detailed.trajectory;
    let mut selected_align_pos: ::core::ffi::c_double = deorbit_alignment_position_error(
        &raw mut detailed,
        s,
    );
    let mut selected_align_course: ::core::ffi::c_double = deorbit_alignment_course_error(
        &raw mut detailed,
        site,
    );
    snprintf(
        &raw mut (*out).note as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Entry %.0fkm FPA %.2fdeg, impact %+.1f/%+.1fkm, alignment %.1fkm/%.1fdeg at %.1fkm/%.0fmps, inlet %.1fkm margin %+.0fkJ/kg, Pe %.1fkm; %u guided candidates. Strict %u/%u (%.0f%%), recovery %u/%u (%.0f%%)%s%s\0"
            as *const u8 as *const ::core::ffi::c_char,
        detailed.entry_range / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track / 1000.0f64,
        selected.energy_margin.impact_cross_track / 1000.0f64,
        selected_align_pos / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        selected_align_course,
        detailed.taem_altitude / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        detailed.taem_speed,
        detailed.taem_distance / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        detailed.taem_interface_energy_margin
            / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        selected.peri / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
        inlet_candidates_evaluated,
        selected_robust.passed,
        selected_robust.scenarios,
        selected_robust.pass_fraction
            * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
        selected_robust.recovery_passed,
        selected_robust.scenarios,
        selected_robust.recovery_pass_fraction
            * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
        if selected_robust.unsafe_0 != 0 {
            b"; unsafe stress case detected\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
        if (*out).target_capture_achieved as ::core::ffi::c_int != 0 {
            b".\0" as *const u8 as *const ::core::ffi::c_char
        } else if (*out).execution_qualified as ::core::ffi::c_int != 0 {
            b"; entry-stage qualified; downstream MM304/TAEM shadow capture not achieved.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"; entry-stage execution acceptance not met.\0" as *const u8
                as *const ::core::ffi::c_char
        },
    );
    if selected.pred.mm304_gate_recorded as ::core::ffi::c_int != 0
        && detailed.mm304_gate_recorded as ::core::ffi::c_int != 0
    {
        fprintf(
            stderr,
            b"Deorbit 50km policy witness: reduced along/cross %+.1f/%+.1fkm V %.1f; executable %+.1f/%+.1fkm V %.1f; delta %+.1f/%+.1fkm dV %+.1f.\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            selected.pred.mm304_gate_along_track / 1000.0f64,
            selected.pred.mm304_gate_cross_track / 1000.0f64,
            selected.pred.mm304_gate_speed,
            detailed.mm304_gate_along_track / 1000.0f64,
            detailed.mm304_gate_cross_track / 1000.0f64,
            detailed.mm304_gate_speed,
            (detailed.mm304_gate_along_track - selected.pred.mm304_gate_along_track)
                / 1000.0f64,
            (detailed.mm304_gate_cross_track - selected.pred.mm304_gate_cross_track)
                / 1000.0f64,
            detailed.mm304_gate_speed - selected.pred.mm304_gate_speed,
        );
    } else {
        fprintf(
            stderr,
            b"Deorbit 50km policy witness incomplete: reduced=%d executable=%d shadow=%d abort=%d.\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            if selected.pred.mm304_gate_recorded as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if detailed.mm304_gate_recorded as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if detailed.shadow_guidance_used as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
            if detailed.shadow_aborted as ::core::ffi::c_int != 0 {
                1 as ::core::ffi::c_int
            } else {
                0 as ::core::ffi::c_int
            },
        );
    }
    fprintf(
        stderr,
        b"Deorbit selected: tier %d burn %.1f dv %.1f Pe %.1fkm entry %.1fkm FPA %.2fdeg marginImpact %+.1f/%+.1fkm align %.1fkm courseDebt %.1fdeg h %.1fkm V %.0f miss %.1fkm TAEMrange %.1fkm rangeError %+.1fkm capture=%d shadow=%d entered=%d gate=%d taem=%d abort=%d boundaryMiss=%d minEntryV=%.0f safeV=%.0f energyValid=%d nominal %d robust %u/%u recovery %u/%u exec %d.\n\0"
            as *const u8 as *const ::core::ffi::c_char,
        selected.tier,
        selected.burn_ut,
        selected.dv,
        selected.peri / 1000.0f64,
        detailed.entry_range / 1000.0f64,
        detailed.entry_flight_path_angle,
        selected.energy_margin.impact_along_track / 1000.0f64,
        selected.energy_margin.impact_cross_track / 1000.0f64,
        selected_align_pos / 1000.0f64,
        selected_align_course,
        detailed.taem_altitude / 1000.0f64,
        detailed.taem_speed,
        detailed.closest_distance / 1000.0f64,
        detailed.taem_distance / 1000.0f64,
        detailed.taem_range_error / 1000.0f64,
        if deorbit_runway_capture_qualified(&raw mut detailed, site, v, s, selected.peri)
            as ::core::ffi::c_int != 0
        {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if detailed.shadow_guidance_used as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if detailed.entered_atmosphere as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if detailed.mm304_gate_recorded as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if detailed.reached_taem as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if detailed.shadow_aborted as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if detailed.taem_ownership_boundary_missed as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        detailed.minimum_entry_speed,
        (*v).minimum_safe_speed,
        if selected.energy_margin.valid as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        if nominal_capture as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
        selected_robust.passed,
        selected_robust.scenarios,
        selected_robust.recovery_passed,
        selected_robust.scenarios,
        if (*out).execution_qualified as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        },
    );
    if !getenv(
            b"KSP_LANDER_TRACE_PLAN_TIMING\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"PLAN_TIMING stage=%s cpu=%.3fs\n\0" as *const u8
                as *const ::core::ffi::c_char,
            b"done\0" as *const u8 as *const ::core::ffi::c_char,
            clock() as ::core::ffi::c_double / CLOCKS_PER_SEC as ::core::ffi::c_double,
        );
    }
    candidate_clear(&raw mut selected);
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_plan_create_with_telemetry(
    mut out: *mut DeorbitPlan,
    mut state: VehicleState,
    mut period: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
    mut authority_source: *const Telemetry,
) -> bool {
    return deorbit_plan_create_with_terminal_model(
        out,
        state,
        period,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        thrust,
        authority_source,
        ::core::ptr::null::<TerminalModel>(),
    );
}
#[no_mangle]
pub unsafe extern "C" fn deorbit_plan_create(
    mut out: *mut DeorbitPlan,
    mut state: VehicleState,
    mut period: ::core::ffi::c_double,
    mut p: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut env: *const AerodynamicEnvelope,
    mut cal: *const TrajectoryCalibrationModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut s: *const GuidanceSettings,
    mut thrust: ::core::ffi::c_double,
) -> bool {
    return deorbit_plan_create_with_telemetry(
        out,
        state,
        period,
        p,
        aero,
        env,
        cal,
        site,
        v,
        s,
        thrust,
        ::core::ptr::null::<Telemetry>(),
    );
}
pub const DBL_MAX: ::core::ffi::c_double = __DBL_MAX__;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MAX__: ::core::ffi::c_double = 1.7976931348623157e+308f64;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const NULL_0: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();

/// Offline regression of the deorbit window search, ported from the former
/// white-box C test (`Validation/DeorbitWindowTests.c`): it inspects the cheap
/// orbital timing implementation without a live link or the expensive
/// atmospheric certification search.
#[cfg(test)]
mod deorbit_window_tests {
    use super::*;
    use ::core::mem::zeroed;

    extern "C" {
        fn destination_point(a: GeoPoint, bearing: f64, distance: f64, radius: f64) -> GeoPoint;
    }

    #[test]
    fn deorbit_window_search_and_witness_selection() {
        unsafe {
            let mut p: PlanetModel = zeroed();
            p.radius = 600000.0;
            p.gravitational_parameter = 3.5316e12;
            p.rotational_speed = 0.0002915709;
            p.atmosphere_depth = 70000.0;
            p.north_axis = v3(0.0, 0.0, 1.0);
            p.prime_meridian_at_epoch = v3(1.0, 0.0, 0.0);
            let mut cfg: LandingConfiguration = landing_configuration_default();
            let west = destination_point(
                GeoPoint {
                    latitude: cfg.site.latitude,
                    longitude: cfg.site.longitude,
                    altitude: cfg.site.altitude,
                },
                270.0,
                5000.0,
                p.radius,
            );
            let mut runway_margin: DeorbitEnergyMargin = zeroed();
            deorbit_passive_runway_margin(west, &cfg.site, p.radius, &mut runway_margin);
            assert!(runway_margin.impact_runway_end == 1 && runway_margin.impact_along_track > 7000.0);
            cfg.site.allow_reciprocal_runway = false;
            deorbit_passive_runway_margin(west, &cfg.site, p.radius, &mut runway_margin);
            assert!(runway_margin.impact_runway_end == 0 && runway_margin.impact_along_track < -4000.0);
            cfg.site.allow_reciprocal_runway = true;
            cfg.guidance.target_post_burn_periapsis_altitude = 50000.0;
            cfg.guidance.target_deorbit_capture_radius = 15000.0;
            let mut state: VehicleState = zeroed();
            state.position = v3(686000.0, 0.0, 0.0);
            state.velocity = v3(0.0, (p.gravitational_parameter / 686000.0).sqrt(), 0.0);
            state.mass = 26000.0;
            let period = 2.0 * (-1.0f64).acos() * (686000.0f64.powi(3) / p.gravitational_parameter).sqrt();
            let horizon = deorbit_search_horizon(state, period, &p, &cfg.guidance);
            assert!(horizon < 2.0 * period);
            let mut inclined = state;
            inclined.velocity = v3(0.0, state.velocity.y * 0.5, state.velocity.y * 0.75f64.sqrt());
            assert!(deorbit_search_horizon(inclined, period, &p, &cfg.guidance) > horizon);
            let mut eccentric = state;
            eccentric.velocity = vscale(state.velocity, 1.1);
            assert!(deorbit_search_horizon(eccentric, period, &p, &cfg.guidance) > horizon);
            let mut early: DeorbitTimingProbe = zeroed();
            early.valid = true;
            early.closing = true;
            early.entry_miss = 10000.0;
            early.burn_start = 100.0;
            let mut late: DeorbitTimingProbe = zeroed();
            late.valid = true;
            late.closing = true;
            late.entry_miss = 1.0;
            late.burn_start = 10000.0;
            assert!(deorbit_timing_probe_better(&early, &late, true, &cfg.guidance));
            let cal: TrajectoryCalibrationModel = zeroed();
            let mut straight: DeorbitTimingProbe = zeroed();
            let mut steered: DeorbitTimingProbe = zeroed();
            assert!(deorbit_timing_probe_start(
                &mut straight, state, 100.0, 0.0, &p, &cal, &cfg.site, &cfg.guidance, 60000.0,
                ::core::ptr::null_mut()
            ));
            assert!(deorbit_timing_probe_start(
                &mut steered, state, 100.0, 5.0, &p, &cal, &cfg.site, &cfg.guidance, 60000.0,
                ::core::ptr::null_mut()
            ));
            assert!((steered.peri - 50000.0).abs() < 100.0);
            assert!((steered.entry_crossrange - straight.entry_crossrange).abs() > 1.0);
            let mut best: DeorbitTimingProbe = zeroed();
            let mut found = false;
            for i in 0..=360 {
                let mut probe: DeorbitTimingProbe = zeroed();
                if deorbit_timing_probe_start(
                    &mut probe, state,
                    cfg.guidance.minimum_planning_lead_time + horizon * i as f64 / 360.0,
                    0.0, &p, &cal, &cfg.site, &cfg.guidance, 60000.0, ::core::ptr::null_mut(),
                ) && deorbit_timing_probe_better(&probe, &best, found, &cfg.guidance)
                {
                    best = probe;
                    found = true;
                }
            }
            assert!(found && best.burn_start <= horizon + cfg.guidance.minimum_planning_lead_time);
            // A nearby deorbit opportunity may have a worse cheap orbital miss
            // but a physically usable and still-closing MM305 acquisition range.
            cfg.guidance.target_deorbit_capture_radius = 100000.0;
            cfg.guidance.taem_interface_range = 96500.0;
            cfg.guidance.hac_radius = 12000.0;
            cfg.guidance.final_approach_distance = 8000.0;
            let mut previous: Candidate = zeroed();
            previous.peri = 50000.0;
            previous.burn_ut = 100.0;
            let mut improved: Candidate = zeroed();
            improved.peri = 50000.0;
            improved.burn_ut = 143.0;
            let mut poor: EntryPrediction = zeroed();
            poor.entered_atmosphere = true;
            poor.reached_taem = true;
            poor.entry_flight_path_angle = -2.0;
            poor.entry_range = 1019000.0;
            poor.peak_dynamic_pressure = 6000.0;
            poor.peak_g_load = 2.0;
            poor.taem_altitude = 25750.0;
            poor.taem_speed = 863.0;
            poor.taem_distance = 153500.0;
            poor.taem_along_track = 8000.0;
            poor.taem_cross_track = -153300.0;
            poor.taem_course = 185.0;
            poor.taem_interface_energy_margin = -270000.0;
            poor.taem_ownership_boundary_missed = true;
            let mut usable = poor;
            usable.taem_distance = 71400.0;
            usable.taem_along_track = 1400.0;
            usable.taem_cross_track = -71400.0;
            usable.taem_interface_energy_margin = -246000.0;
            assert!(deorbit_witness_inlet_geometry(&usable, &p, &cfg.site, &cfg.guidance));
            assert!(!deorbit_witness_inlet_geometry(&poor, &p, &cfg.site, &cfg.guidance));
            let nan = f64::NAN;
            assert!(deorbit_guided_candidate_better(
                &improved, &usable, &previous, &poor, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            let mut overshot = usable;
            overshot.taem_distance = 90000.0;
            overshot.taem_along_track = 4000.0;
            overshot.taem_cross_track = 89900.0;
            overshot.taem_course = 195.0;
            assert!(!deorbit_witness_closing(&overshot, &cfg.site));
            assert!(!deorbit_guided_candidate_better(
                &improved, &overshot, &previous, &usable, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            let mut too_close = usable;
            too_close.taem_distance = 8000.0;
            too_close.taem_along_track = 1000.0;
            too_close.taem_cross_track = -8000.0;
            assert!(!deorbit_witness_inlet_geometry(&too_close, &p, &cfg.site, &cfg.guidance));
            assert!(!deorbit_guided_candidate_better(
                &improved, &too_close, &previous, &usable, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            let mut unsafe_prediction = usable;
            unsafe_prediction.entry_flight_path_angle = -15.0;
            unsafe_prediction.taem_dynamic_interface_captured = true;
            unsafe_prediction.taem_ownership_boundary_missed = false;
            assert!(!deorbit_guided_candidate_better(
                &improved, &unsafe_prediction, &previous, &usable, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            // The 34-degree return's course fails the perpendicular inlet
            // heuristic, but a safe closing inlet still deserves a native MM305
            // continuation.
            let mut oblique = usable;
            oblique.taem_course = 60.0;
            oblique.taem_along_track = -40000.0;
            oblique.taem_cross_track = -40000.0;
            assert!(!deorbit_witness_inlet_geometry(&oblique, &p, &cfg.site, &cfg.guidance));
            assert!(deorbit_witness_terminal_trial(&improved, &oblique, &p, &cfg.site, &cfg.vehicle, &cfg.guidance));
            assert!(!deorbit_witness_terminal_trial(
                &improved, &unsafe_prediction, &p, &cfg.site, &cfg.vehicle, &cfg.guidance
            ));
            // Among two completed uncertified attempts, prefer the nearer
            // ground endpoint even if the other candidate better matches the
            // nominal range.
            let mut near = usable;
            let mut nominal = usable;
            near.shadow_guidance_used = true;
            nominal.shadow_guidance_used = true;
            near.taem_distance = 70000.0;
            nominal.taem_distance = 96500.0;
            near.final_state.ut = 1000.0;
            nominal.final_state.ut = 1000.0;
            near.final_state.position = predictor_inertial_position(
                GeoPoint { latitude: cfg.site.latitude, longitude: cfg.site.longitude + 0.1, altitude: 50.0 },
                &p, 1000.0,
            );
            nominal.final_state.position = predictor_inertial_position(
                GeoPoint { latitude: cfg.site.latitude, longitude: cfg.site.longitude + 1.0, altitude: 50.0 },
                &p, 1000.0,
            );
            previous.executable_witness_evaluated = true;
            improved.executable_witness_evaluated = true;
            // The nearer ground endpoint cannot compensate for a missed MM305
            // ownership boundary or a larger terminal energy deficit.
            nominal.taem_dynamic_interface_captured = true;
            nominal.taem_ownership_boundary_missed = false;
            assert!(deorbit_guided_candidate_better(
                &previous, &nominal, &improved, &near, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            nominal.taem_dynamic_interface_captured = false;
            nominal.taem_interface_energy_margin = -200000.0;
            assert!(deorbit_guided_candidate_better(
                &previous, &nominal, &improved, &near, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            nominal.taem_interface_energy_margin = near.taem_interface_energy_margin;
            assert!(deorbit_guided_candidate_better(
                &improved, &near, &previous, &nominal, &p, &cfg.site, &cfg.vehicle, &cfg.guidance, nan
            ));
            // A closer point high above KSC is not a completed approach.
            near.final_state.position = predictor_inertial_position(
                GeoPoint { latitude: cfg.site.latitude, longitude: cfg.site.longitude, altitude: 15000.0 },
                &p, 1000.0,
            );
            assert!(!deorbit_guided_terminal_miss(&improved, &near, &p, &cfg.site).is_finite());
        }
    }
}
