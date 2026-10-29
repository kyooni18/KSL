use crate::sys::*;
extern "C" {
    fn jw_init(w: *mut JsonWriter);
    fn jw_free(w: *mut JsonWriter);
    fn jw_raw(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_char(w: *mut JsonWriter, c: ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_integer(w: *mut JsonWriter, value: ::core::ffi::c_longlong);
    fn jw_bool(w: *mut JsonWriter, value: bool);
    fn jw_null(w: *mut JsonWriter);
    fn entry_exec_telemetry(exec: *const EntryExecutive) -> EntryExecTelemetry;
    fn taem_exec_telemetry(exec: *const TaemExecutive) -> TaemExecTelemetry;
    fn taem_exec_owns_vehicle(exec: *const TaemExecutive) -> bool;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn async_prediction_request_due(
        latest_ut: ::core::ffi::c_double,
        last_request_ut: ::core::ffi::c_double,
        now_wall: ::core::ffi::c_double,
        last_completion_wall: ::core::ffi::c_double,
        previous_solve_wall_seconds: ::core::ffi::c_double,
        configured_interval: ::core::ffi::c_double,
    ) -> bool;
    fn async_prediction_result_fresh(
        request_identity: *const AsyncPredictionIdentity,
        current_identity: *const AsyncPredictionIdentity,
        request_ut: ::core::ffi::c_double,
        latest_ut: ::core::ffi::c_double,
        configured_interval: ::core::ffi::c_double,
    ) -> bool;
    fn planner_log_open(
        directory: *const ::core::ffi::c_char,
        stamp: *const ::core::ffi::c_char,
        vessel: *const ::core::ffi::c_char,
        session_id: *const ::core::ffi::c_char,
        native_build: *const ::core::ffi::c_char,
        vehicle_peer_path: *const ::core::ffi::c_char,
        vehicle_peer_available: bool,
        configuration: *const LandingConfiguration,
        max_segment_bytes: size_t,
    ) -> *mut PlannerLog;
    fn planner_log_close(log_0: *mut PlannerLog);
    fn planner_log_record(
        log_0: *mut PlannerLog,
        sample: *const PlannerLogSample,
    ) -> PlannerLogDecision;
    fn planner_log_current_path(log_0: *const PlannerLog) -> *const ::core::ffi::c_char;
    fn vessel_physics_init(m: *mut VesselPhysicsModel);
    fn vessel_physics_observe(
        m: *mut VesselPhysicsModel,
        t: *mut Telemetry,
        s: *const VehicleState,
        p: *const PlanetModel,
    );
    fn vessel_physics_derive_envelope(
        m: *mut VesselPhysicsModel,
        baseline: *const VehicleProfile,
        envelope: *mut AerodynamicEnvelope,
        planning: *mut AerodynamicModel,
    ) -> bool;
    fn landing_snapshot_set_sample(
        snapshot: *mut LandingSnapshot,
        t: *const Telemetry,
        state: *const VehicleState,
    );
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn entry_taem_range_target(
        planet: *const PlanetModel,
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
    fn v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vector3;
    fn vdot(a: Vector3, b: Vector3) -> ::core::ffi::c_double;
    fn vmag(a: Vector3) -> ::core::ffi::c_double;
    fn vnorm(a: Vector3, fallback: Vector3) -> Vector3;
    fn great_circle_distance(
        a: GeoPoint,
        b: GeoPoint,
        radius: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn initial_bearing(a: GeoPoint, b: GeoPoint) -> ::core::ffi::c_double;
    fn destination_point(
        a: GeoPoint,
        bearing: ::core::ffi::c_double,
        distance: ::core::ffi::c_double,
        radius: ::core::ffi::c_double,
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
    fn runway_reciprocal_site(
        primary: *const LandingSite,
        radius: ::core::ffi::c_double,
    ) -> LandingSite;
    fn landing_configuration_default() -> LandingConfiguration;
    fn landing_configuration_normalize(c: *mut LandingConfiguration);
    fn landing_configuration_json(w: *mut JsonWriter, c: *const LandingConfiguration);
    fn planet_model_replay_json(w: *mut JsonWriter, p: *const PlanetModel);
    fn telemetry_init(t: *mut Telemetry);
    fn guidance_command_init(c: *mut GuidanceCommand);
    fn calibration_snapshot_init(c: *mut CalibrationSnapshot);
    fn landing_snapshot_init(s: *mut LandingSnapshot, p: *const VehicleProfile);
    fn phase_string(p: GuidancePhase) -> *const ::core::ffi::c_char;
    fn profile_string(p: ControlProfile) -> *const ::core::ffi::c_char;
    fn trajectory_init(t: *mut Trajectory);
    fn trajectory_clear(t: *mut Trajectory);
    fn trajectory_append(t: *mut Trajectory, p: TrajectoryPoint) -> bool;
    fn trajectory_copy(dst: *mut Trajectory, src: *const Trajectory) -> bool;
    fn deorbit_plan_clear(p: *mut DeorbitPlan);
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
    fn planet_surface_gravity(p: *const PlanetModel) -> ::core::ffi::c_double;
    fn planet_atmospheric_density(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_speed_of_sound(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn predictor_postburn_periapsis(
        state: VehicleState,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn entry_taem_speed_target(
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
        planet: *const PlanetModel,
    ) -> ::core::ffi::c_double;
    fn predictor_simulate_entry_policy(
        state: VehicleState,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
        vehicle: *const VehicleProfile,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
        bank_deg: ::core::ffi::c_double,
        aoa_deg: ::core::ffi::c_double,
        sign: ::core::ffi::c_double,
        max_duration: ::core::ffi::c_double,
    ) -> EntryPrediction;
    fn predictor_simulate_entry_with_attitude(
        state: VehicleState,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
        vehicle: *const VehicleProfile,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
        max_bank: ::core::ffi::c_double,
        initial_bank: ::core::ffi::c_double,
        initial_bank_rate: ::core::ffi::c_double,
        initial_angle_of_attack: ::core::ffi::c_double,
        initial_angle_of_attack_rate: ::core::ffi::c_double,
        initial_bank_sign: ::core::ffi::c_double,
        initial_leg_elapsed: ::core::ffi::c_double,
        max_duration: ::core::ffi::c_double,
        include_trajectory: bool,
    ) -> EntryPrediction;
    fn predictor_simulate_entry_guidance_shadow_with_terminal_model(
        state: VehicleState,
        telemetry: *const Telemetry,
        guidance: *const GuidanceMachine,
        plan: *const DeorbitPlan,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
        cfg: *const LandingConfiguration,
        terminal_model: *const TerminalModel,
        max_duration: ::core::ffi::c_double,
        include_trajectory: bool,
    ) -> EntryPrediction;
    fn predictor_simulate_terminal_shadow_ensemble(
        state: VehicleState,
        telemetry: *const Telemetry,
        guidance: *const GuidanceMachine,
        plan: *const DeorbitPlan,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
        cfg: *const LandingConfiguration,
        terminal_model: *const TerminalModel,
        max_duration: ::core::ffi::c_double,
        include_trajectory: bool,
    ) -> EntryPrediction;
    fn entry_prediction_clear(p: *mut EntryPrediction);
    fn reentry_guidance_shadow_recovery_qualified(
        prediction: *const EntryPrediction,
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
    ) -> bool;
    fn deorbit_plan_create_with_terminal_model(
        out: *mut DeorbitPlan,
        state: VehicleState,
        orbit_period: ::core::ffi::c_double,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        env: *const AerodynamicEnvelope,
        cal: *const TrajectoryCalibrationModel,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
        available_thrust: ::core::ffi::c_double,
        authority_source: *const Telemetry,
        terminal_model: *const TerminalModel,
    ) -> bool;
    fn deorbit_plan_preburn_state_compatible(
        plan: *const DeorbitPlan,
        live_mass: ::core::ffi::c_double,
        live_available_thrust: ::core::ffi::c_double,
        settings: *const GuidanceSettings,
    ) -> bool;
    fn deorbit_capture_qualified(
        p: *const EntryPrediction,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
        postburn_periapsis: ::core::ffi::c_double,
        require_entry_corridor: bool,
    ) -> bool;
    fn deorbit_live_cutoff_capture_qualified(
        p: *const EntryPrediction,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        settings: *const GuidanceSettings,
        postburn_periapsis: ::core::ffi::c_double,
    ) -> bool;
    fn guidance_accept_entry_plan(
        g: *mut GuidanceMachine,
        request: *const GuidanceMachine,
        result: *const GuidanceMachine,
        t: *const Telemetry,
        cfg: *const LandingConfiguration,
    ) -> bool;
    fn guidance_machine_init(g: *mut GuidanceMachine);
    fn guidance_set_engaged(g: *mut GuidanceMachine, engaged: bool);
    fn guidance_set_paused(g: *mut GuidanceMachine, paused: bool);
    fn guidance_abort(g: *mut GuidanceMachine);
    fn guidance_reset_plan(g: *mut GuidanceMachine);
    fn guidance_entry_leg_elapsed(
        g: *const GuidanceMachine,
        ut: ::core::ffi::c_double,
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
    fn guidance_begin_mm305_continuation(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        message: *mut ::core::ffi::c_char,
        message_size: size_t,
    ) -> bool;
    fn guidance_begin_hac_test(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        message: *mut ::core::ffi::c_char,
        message_size: size_t,
    ) -> bool;
    fn guidance_begin_final_test(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        course: ::core::ffi::c_double,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
        message: *mut ::core::ffi::c_char,
        message_size: size_t,
    ) -> bool;
    fn guidance_update(
        g: *mut GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        plan: *const DeorbitPlan,
        planet: *const PlanetModel,
        aero: AerodynamicModel,
        cfg: *const LandingConfiguration,
    ) -> GuidanceResult;
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
    fn reference_trajectory(
        out: *mut Trajectory,
        site: *const LandingSite,
        settings: *const GuidanceSettings,
        radius: ::core::ffi::c_double,
        hac_side: ::core::ffi::c_double,
    );
    fn adaptive_calibrator_init(
        c: *mut AdaptiveFlightCalibrator,
        profile: *const VehicleProfile,
    );
    fn adaptive_calibrator_reset(
        c: *mut AdaptiveFlightCalibrator,
        profile: *const VehicleProfile,
    );
    fn adaptive_calibrator_update(
        c: *mut AdaptiveFlightCalibrator,
        t: *const Telemetry,
        settings: *const CalibrationSettings,
        baseline: *const VehicleProfile,
        command: *const GuidanceCommand,
        dt: ::core::ffi::c_double,
        allow_adaptation: bool,
        active: bool,
        sampling: bool,
        run_state: CalibrationRunState,
        progress: ::core::ffi::c_double,
        target_aoa: ::core::ffi::c_double,
        run_status: *const ::core::ffi::c_char,
        warning: *const ::core::ffi::c_char,
        current: *mut AerodynamicModel,
        planning: *mut AerodynamicModel,
        envelope: *mut AerodynamicEnvelope,
        recommended: *mut VehicleProfile,
        snapshot: *mut CalibrationSnapshot,
    );
    fn glide_calibration_init(g: *mut GlideCalibrationMachine);
    fn glide_calibration_start(
        g: *mut GlideCalibrationMachine,
        t: *const Telemetry,
        settings: *const CalibrationSettings,
    );
    fn glide_calibration_stop(
        g: *mut GlideCalibrationMachine,
        status: *const ::core::ffi::c_char,
    );
    fn glide_calibration_update(
        g: *mut GlideCalibrationMachine,
        t: *const Telemetry,
        profile: *const VehicleProfile,
        settings: *const CalibrationSettings,
        sampling: *mut bool,
        finished: *mut bool,
        progress: *mut ::core::ffi::c_double,
        target_aoa: *mut ::core::ffi::c_double,
    ) -> GuidanceResult;
    fn trajectory_calibrator_init(c: *mut InFlightTrajectoryCalibrator);
    fn trajectory_calibrator_clear(c: *mut InFlightTrajectoryCalibrator);
    fn trajectory_calibrator_set_forecast(
        c: *mut InFlightTrajectoryCalibrator,
        forecast: *const Trajectory,
    );
    fn trajectory_calibrator_update(
        c: *mut InFlightTrajectoryCalibrator,
        t: *const Telemetry,
        state: *const VehicleState,
        planet: *const PlanetModel,
        site: *const LandingSite,
        env: *const AerodynamicEnvelope,
        vehicle: *const VehicleProfile,
        command: *const GuidanceCommand,
        settings: *const CalibrationSettings,
        allow_adaptation: bool,
        dt: ::core::ffi::c_double,
    ) -> TrajectoryCalibrationModel;
    fn krpc_warp_with_progress(
        s: *mut KRPCSession,
        ut: ::core::ffi::c_double,
        progress: KRPCWarpProgress,
        context: *mut ::core::ffi::c_void,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_shutdown_burn_engines(
        s: *mut KRPCSession,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_read_physics_warp(
        s: *mut KRPCSession,
        factor: *mut ::core::ffi::c_int,
        rate: *mut ::core::ffi::c_double,
        rails: *mut bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_set_physics_warp(
        s: *mut KRPCSession,
        factor: ::core::ffi::c_int,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_session_open(
        cfg: *const LandingConfiguration,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> *mut KRPCSession;
    fn krpc_session_close(s: *mut KRPCSession);
    fn krpc_session_planet(s: *const KRPCSession) -> *const PlanetModel;
    fn krpc_session_vessel(s: *const KRPCSession) -> *const ::core::ffi::c_char;
    fn krpc_session_transport(s: *const KRPCSession) -> *const ::core::ffi::c_char;
    fn krpc_session_library_version(s: *const KRPCSession) -> *const ::core::ffi::c_char;
    fn krpc_session_transport_budget(s: *const KRPCSession) -> KRPCTransportBudget;
    fn krpc_session_is_simulator(s: *const KRPCSession) -> bool;
    fn krpc_session_physics_structure_id(
        s: *const KRPCSession,
    ) -> *const ::core::ffi::c_char;
    fn krpc_session_physics_environment_id(
        s: *const KRPCSession,
    ) -> *const ::core::ffi::c_char;
    fn krpc_session_physics_storage(s: *const KRPCSession) -> *const ::core::ffi::c_char;
    fn krpc_session_physics_history_count(s: *const KRPCSession) -> ::core::ffi::c_uint;
    fn krpc_session_seed_physics(s: *const KRPCSession, model: *mut VesselPhysicsModel);
    fn krpc_read_telemetry(
        s: *mut KRPCSession,
        cfg: *const LandingConfiguration,
        t: *mut Telemetry,
        state: *mut VehicleState,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_apply(
        s: *mut KRPCSession,
        command: *const GuidanceCommand,
        airbrake_group: ::core::ffi::c_uint,
        phase: *const ::core::ffi::c_char,
        status: *const ::core::ffi::c_char,
        warning: *const ::core::ffi::c_char,
        hud_predicted_trajectory: *const Trajectory,
        hud_reference_trajectory: *const Trajectory,
        result: *mut KRPCApplyResult,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_neutralize(s: *mut KRPCSession);
    fn krpc_safe(s: *mut KRPCSession);
    fn krpc_set_gear(
        s: *mut KRPCSession,
        value: bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_set_brakes(
        s: *mut KRPCSession,
        value: bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_save_game(
        s: *mut KRPCSession,
        name: *const ::core::ffi::c_char,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn vehicle_log_encoder_init(encoder: *mut VehicleLogEncoder);
    fn vehicle_log_decide(
        encoder: *const VehicleLogEncoder,
        sample: *const VehicleLogSample,
        force_event: bool,
        force_keyframe: bool,
    ) -> VehicleLogDecision;
    fn vehicle_log_commit(
        encoder: *mut VehicleLogEncoder,
        sample: *const VehicleLogSample,
        decision: *const VehicleLogDecision,
    );
    fn vehicle_log_record_json(
        writer: *mut JsonWriter,
        session_id: *const ::core::ffi::c_char,
        record_sequence: uint64_t,
        sample: *const VehicleLogSample,
        decision: *const VehicleLogDecision,
    );
    fn shuttle_sim_model_profile() -> *const ::core::ffi::c_char;
    fn shuttle_sim_model_vehicle_id() -> *const ::core::ffi::c_char;
    fn shuttle_sim_model_matches_vehicle(
        model_id: *const ::core::ffi::c_char,
        reason: *mut ::core::ffi::c_char,
        size: size_t,
    ) -> bool;
    fn shuttle_sim_resolve_vehicle_identity(
        live_name: *const ::core::ffi::c_char,
        configured_model_id: *const ::core::ffi::c_char,
    ) -> *const ::core::ffi::c_char;
    fn shuttle_sim_model_path(
        root: *const ::core::ffi::c_char,
        kind: ShuttleSimModelFile,
        buffer: *mut ::core::ffi::c_char,
        buffer_size: size_t,
    ) -> *const ::core::ffi::c_char;
    fn guidance_mm305_plan_request(
        g: *const GuidanceMachine,
        t: *const Telemetry,
        state: *const VehicleState,
        cfg: *const LandingConfiguration,
        model: *const TerminalModel,
    ) -> Mm305PlanRequest;
    fn mm305_plan(
        model: *const TerminalModel,
        request: *const Mm305PlanRequest,
    ) -> Mm305PlanResult;
    fn mm305_revalidate_plan(
        model: *const TerminalModel,
        request: *const Mm305PlanRequest,
        previous: *const Mm305PlanResult,
    ) -> Mm305PlanResult;
    fn mm305_plan_result_fresh(
        result: *const Mm305PlanResult,
        telemetry: *const Telemetry,
    ) -> bool;
    fn guidance_mm305_accept_plan(
        g: *mut GuidanceMachine,
        result: *const Mm305PlanResult,
        cfg: *const LandingConfiguration,
    ) -> bool;
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
    fn mm305_hac_handoff_energy_error(
        model: *const TerminalModel,
        live: *const TaemHacLiveAero,
        geo: *const TaemGeometryState,
        mass_kg: ::core::ffi::c_double,
        error_m: *mut ::core::ffi::c_double,
        band_m: *mut ::core::ffi::c_double,
    ) -> bool;
    fn terminal_model_capture(
        model: *mut TerminalModel,
        files: *const TerminalModelSourceFiles,
        configuration: *const LandingConfiguration,
        snapshot_id: uint64_t,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn terminal_model_align_planet_frame(
        model: *mut TerminalModel,
        planet: *const PlanetModel,
        reason: *mut ::core::ffi::c_char,
        reason_size: size_t,
    ) -> bool;
    fn terminal_model_policy_matches(
        model: *const TerminalModel,
        site: *const LandingSite,
        vehicle: *const VehicleProfile,
        guidance: *const GuidanceSettings,
    ) -> bool;
}
pub type size_t = usize;
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
pub type TerminalVerticalStage = ::core::ffi::c_uint;
pub const TERMINAL_GROUND: TerminalVerticalStage = 5;
pub const TERMINAL_TOUCHDOWN_FLARE: TerminalVerticalStage = 4;
pub const TERMINAL_INNER_FINAL: TerminalVerticalStage = 3;
pub const TERMINAL_PREFLARE: TerminalVerticalStage = 2;
pub const TERMINAL_OUTER_FINAL: TerminalVerticalStage = 1;
pub const TERMINAL_TRAJECTORY_CAPTURE: TerminalVerticalStage = 0;
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
pub struct LandingController {
    pub mutex: pthread_mutex_t,
    pub thread: pthread_t,
    pub thread_started: bool,
    pub stop_thread: bool,
    pub prediction_thread: pthread_t,
    pub prediction_thread_started: bool,
    pub mm305_planner_thread: pthread_t,
    pub mm305_planner_started: bool,
    pub stop_prediction_thread: bool,
    pub prediction_generation: uint64_t,
    pub last_terminal_prediction_request_ut: ::core::ffi::c_double,
    pub last_terminal_prediction_completion_ut: ::core::ffi::c_double,
    pub last_terminal_prediction_completion_wall: ::core::ffi::c_double,
    pub last_terminal_prediction_solve_wall: ::core::ffi::c_double,
    pub configuration: LandingConfiguration,
    pub session: *mut KRPCSession,
    pub guidance: GuidanceMachine,
    pub terminal_model: TerminalModel,
    pub terminal_model_valid: bool,
    pub vehicle_identity: [::core::ffi::c_char; 128],
    pub terminal_model_capture_sequence: uint64_t,
    pub physics: VesselPhysicsModel,
    pub adaptive: AdaptiveFlightCalibrator,
    pub glide: GlideCalibrationMachine,
    pub aerodynamics: AerodynamicModel,
    pub planning_aerodynamics: AerodynamicModel,
    pub envelope: AerodynamicEnvelope,
    pub trajectory_calibrator: InFlightTrajectoryCalibrator,
    pub trajectory_calibration: TrajectoryCalibrationModel,
    pub last_command: GuidanceCommand,
    pub adaptive_profile: VehicleProfile,
    pub plan: DeorbitPlan,
    pub has_plan: bool,
    pub plan_stale: bool,
    pub snapshot: LandingSnapshot,
    pub dynamic_trajectory: Trajectory,
    pub stabilized_trajectory: Trajectory,
    pub forecast_entry_plan_identity_valid: bool,
    pub forecast_entry_plan_id: uint64_t,
    pub forecast_entry_plan_version: uint64_t,
    pub prediction_lineage_discontinuity: bool,
    pub latest_telemetry: Telemetry,
    pub latest_state: VehicleState,
    pub has_latest: bool,
    pub last_trajectory_ut: ::core::ffi::c_double,
    pub last_prediction_ut: ::core::ffi::c_double,
    pub has_prediction_cache: bool,
    pub cached_physics_observed_seconds: ::core::ffi::c_double,
    pub cached_physics_fallback_seconds: ::core::ffi::c_double,
    pub cached_predicted_miss_distance: ::core::ffi::c_double,
    pub cached_predicted_taem_distance: ::core::ffi::c_double,
    pub cached_predicted_taem_range_error: ::core::ffi::c_double,
    pub cached_predicted_entry_range: ::core::ffi::c_double,
    pub cached_predicted_entry_flight_path_angle: ::core::ffi::c_double,
    pub cached_predicted_taem_speed: ::core::ffi::c_double,
    pub cached_predicted_taem_energy_error: ::core::ffi::c_double,
    pub cached_predicted_peak_dynamic_pressure: ::core::ffi::c_double,
    pub cached_predicted_peak_g_load: ::core::ffi::c_double,
    pub cached_predicted_s_turn_reversals: ::core::ffi::c_double,
    pub cached_predicted_shadow_guidance: bool,
    pub cached_predicted_terminal_policy_feasible: bool,
    pub cached_predicted_terminal_survivability_stress_score: ::core::ffi::c_double,
    pub cached_predicted_physics_relative_uncertainty: ::core::ffi::c_double,
    pub cached_predicted_uncertainty_scenarios: ::core::ffi::c_uint,
    pub cached_predicted_raw_published_position_delta_30s: ::core::ffi::c_double,
    pub cached_predicted_raw_published_position_delta_60s: ::core::ffi::c_double,
    pub cached_predicted_raw_published_position_delta_120s: ::core::ffi::c_double,
    pub cached_prediction_ut: ::core::ffi::c_double,
    pub cached_prediction_sign: ::core::ffi::c_double,
    pub last_tick_ut: ::core::ffi::c_double,
    pub has_last_tick_ut: bool,
    pub last_tick_wall: ::core::ffi::c_double,
    pub has_last_tick_wall: bool,
    pub time_warp_issued: bool,
    pub warp_step_ut: ::core::ffi::c_double,
    pub warp_last_roll_rate: ::core::ffi::c_double,
    pub warp_roll_reversals: ::core::ffi::c_int,
    pub warp_reversal_window_ut: ::core::ffi::c_double,
    pub burn_engines_shut_down: bool,
    pub forecast_history: [ForecastHistorySample; 64],
    pub forecast_history_count: ::core::ffi::c_int,
    pub forecast_drag_scale: ::core::ffi::c_double,
    pub forecast_lift_scale: ::core::ffi::c_double,
    pub tick_sequence: uint64_t,
    pub log_sequence: uint64_t,
    pub vehicle_record_sequence: uint64_t,
    pub callback: SnapshotCallback,
    pub callback_context: *mut ::core::ffi::c_void,
    pub sim_publication_valid: bool,
    pub publication_event_pending: bool,
    pub sim_last_publish_wall: ::core::ffi::c_double,
    pub sim_last_publish_ut: ::core::ffi::c_double,
    pub sim_last_publish_phase: GuidancePhase,
    pub sim_last_publish_connection: ConnectionStatus,
    pub sim_last_publish_engaged: bool,
    pub sim_last_publish_paused: bool,
    pub flight_log: *mut FILE,
    pub planner_log: *mut PlannerLog,
    pub vehicle_log_encoder: VehicleLogEncoder,
    pub log_session_id: [::core::ffi::c_char; 192],
    pub log_native_build: [::core::ffi::c_char; 96],
    pub log_planner_path: [::core::ffi::c_char; 1700],
    pub log_actual_trajectory_dirty: bool,
    pub log_reference_trajectory_dirty: bool,
    pub log_plan_trajectory_dirty: bool,
    pub log_raw_plan_trajectory_dirty: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogEncoder {
    pub has_last: bool,
    pub last: VehicleLogSample,
    pub last_emit_wall: ::core::ffi::c_double,
    pub last_keyframe_wall: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogSample {
    pub tick_sequence: uint64_t,
    pub wall_monotonic_seconds: ::core::ffi::c_double,
    pub ut: ::core::ffi::c_double,
    pub latitude: ::core::ffi::c_double,
    pub longitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub radar_altitude: ::core::ffi::c_double,
    pub position: [::core::ffi::c_double; 3],
    pub velocity: [::core::ffi::c_double; 3],
    pub true_air_speed: ::core::ffi::c_double,
    pub horizontal_speed: ::core::ffi::c_double,
    pub vertical_speed: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub pitch: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub heading: ::core::ffi::c_double,
    pub angle_of_attack: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub pitch_rate: ::core::ffi::c_double,
    pub roll_rate: ::core::ffi::c_double,
    pub yaw_rate: ::core::ffi::c_double,
    pub course_rate: ::core::ffi::c_double,
    pub coordinate_pitch_rate: ::core::ffi::c_double,
    pub coordinate_roll_rate: ::core::ffi::c_double,
    pub heading_rate: ::core::ffi::c_double,
    pub angle_of_attack_rate: ::core::ffi::c_double,
    pub body_pitch_rate: ::core::ffi::c_double,
    pub body_roll_rate: ::core::ffi::c_double,
    pub body_yaw_rate: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub static_pressure: ::core::ffi::c_double,
    pub atmospheric_density: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub stall_fraction: ::core::ffi::c_double,
    pub stall_fraction_is_measured: bool,
    pub lift_force: ::core::ffi::c_double,
    pub drag_force: ::core::ffi::c_double,
    pub g_force: ::core::ffi::c_double,
    pub energy_excess_range: ::core::ffi::c_double,
    pub has_force_vectors: bool,
    pub lift_vector: [::core::ffi::c_double; 3],
    pub drag_vector: [::core::ffi::c_double; 3],
    pub aero_acceleration: [::core::ffi::c_double; 3],
    pub physics_confidence: ::core::ffi::c_double,
    pub physics_model_residual: ::core::ffi::c_double,
    pub physics_model_residual_confidence: ::core::ffi::c_double,
    pub physics_certified_uncertainty: ::core::ffi::c_double,
    pub physics_force_residual_per_q: [::core::ffi::c_double; 3],
    pub physics_force_residual_sigma_per_q: [::core::ffi::c_double; 3],
    pub physics_force_residual_confidence: ::core::ffi::c_double,
    pub physics_airbrake_model_available: bool,
    pub physics_airbrake_model_confidence: ::core::ffi::c_double,
    pub physics_airbrake_drag_accel: ::core::ffi::c_double,
    pub physics_samples: ::core::ffi::c_uint,
    pub physics_live_samples: ::core::ffi::c_uint,
    pub mass: ::core::ffi::c_double,
    pub available_thrust: ::core::ffi::c_double,
    pub current_thrust: ::core::ffi::c_double,
    pub has_dry_mass: bool,
    pub dry_mass: ::core::ffi::c_double,
    pub has_center_of_mass: bool,
    pub center_of_mass: [::core::ffi::c_double; 3],
    pub has_center_of_mass_root: bool,
    pub center_of_mass_root: [::core::ffi::c_double; 3],
    pub has_torque: bool,
    pub available_torque: [::core::ffi::c_double; 3],
    pub has_inertia: bool,
    pub moment_of_inertia: [::core::ffi::c_double; 3],
    pub target_pitch: ::core::ffi::c_double,
    pub target_heading: ::core::ffi::c_double,
    pub target_roll: ::core::ffi::c_double,
    pub target_throttle: ::core::ffi::c_double,
    pub has_target_aoa: bool,
    pub target_aoa: ::core::ffi::c_double,
    pub target_gear: bool,
    pub target_brakes: bool,
    pub target_airbrakes: bool,
    pub gear: bool,
    pub brakes: bool,
    pub has_airbrakes: bool,
    pub airbrakes: bool,
    pub has_sas_state: bool,
    pub sas_enabled: bool,
    pub has_control_state: bool,
    pub control_state_pitch: ::core::ffi::c_double,
    pub control_state_roll: ::core::ffi::c_double,
    pub control_state_yaw: ::core::ffi::c_double,
    pub control_state_throttle: ::core::ffi::c_double,
    pub command_applied: bool,
    pub has_actuator_feedback: bool,
    pub has_control_diagnostics: bool,
    pub control_pitch: ::core::ffi::c_double,
    pub control_roll: ::core::ffi::c_double,
    pub control_yaw: ::core::ffi::c_double,
    pub control_pitch_error: ::core::ffi::c_double,
    pub control_pitch_trim: ::core::ffi::c_double,
    pub control_pitch_authority: ::core::ffi::c_double,
    pub control_pitch_aero_fraction: ::core::ffi::c_double,
    pub has_control_surface_state: bool,
    pub control_surface_state_ut: ::core::ffi::c_double,
    pub control_surface_count: size_t,
    pub control_surfaces: [VehicleLogSurfaceSample; 16],
    pub has_loop_wall_delta: bool,
    pub has_telemetry_latency: bool,
    pub has_guidance_compute: bool,
    pub has_apply_latency: bool,
    pub has_control_loop: bool,
    pub has_warp: bool,
    pub warp_rate: ::core::ffi::c_double,
    pub loop_wall_delta_ms: ::core::ffi::c_double,
    pub telemetry_latency_ms: ::core::ffi::c_double,
    pub guidance_compute_ms: ::core::ffi::c_double,
    pub apply_latency_ms: ::core::ffi::c_double,
    pub control_loop_ms: ::core::ffi::c_double,
    pub automation_engaged: bool,
    pub paused: bool,
    pub constraint_active: bool,
    pub phase: [::core::ffi::c_char; 40],
    pub control_profile: [::core::ffi::c_char; 32],
    pub vessel_situation: [::core::ffi::c_char; 64],
    pub has_plan_identity: bool,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub parent_plan_id: uint64_t,
    pub parent_plan_version: uint64_t,
    pub range_to_site: ::core::ffi::c_double,
    pub bearing_to_site: ::core::ffi::c_double,
    pub runway_along_track: ::core::ffi::c_double,
    pub runway_cross_track: ::core::ffi::c_double,
    pub specific_mechanical_energy: ::core::ffi::c_double,
    pub entry_plan_valid: bool,
    pub entry_reversal_scheduled: bool,
    pub entry_reversal_time_remaining: ::core::ffi::c_double,
    pub entry_reversal_ut: ::core::ffi::c_double,
    pub entry_reversal_range: ::core::ffi::c_double,
    pub entry_reversal_sign: ::core::ffi::c_double,
    pub event: [::core::ffi::c_char; 512],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogSurfaceSample {
    pub part_name: [::core::ffi::c_char; 64],
    pub role: [::core::ffi::c_char; 16],
    pub deployed: bool,
    pub deflection_override: bool,
    pub override_deflection: ::core::ffi::c_double,
}
pub type SnapshotCallback = Option<
    unsafe extern "C" fn(*const LandingSnapshot, *mut ::core::ffi::c_void) -> (),
>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ForecastHistorySample {
    pub state: VehicleState,
    pub aoa: ::core::ffi::c_double,
    pub bank: ::core::ffi::c_double,
    pub q: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct InFlightTrajectoryCalibrator {
    pub forecast: Trajectory,
    pub model: TrajectoryCalibrationModel,
    pub previous_ut: ::core::ffi::c_double,
    pub has_previous: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GlideCalibrationMachine {
    pub active: bool,
    pub state: CalibrationRunState,
    pub start_ut: ::core::ffi::c_double,
    pub state_start_ut: ::core::ffi::c_double,
    pub initial_heading: ::core::ffi::c_double,
    pub angles: [::core::ffi::c_double; 16],
    pub angle_count: ::core::ffi::c_int,
    pub angle_index: ::core::ffi::c_int,
    pub status: [::core::ffi::c_char; 256],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AdaptiveFlightCalibrator {
    pub ld: [::core::ffi::c_double; 4],
    pub beta: [::core::ffi::c_double; 4],
    pub confidence: [::core::ffi::c_double; 4],
    pub accepted: ::core::ffi::c_int,
    pub rejected: ::core::ffi::c_int,
    pub best_glide_aoa: ::core::ffi::c_double,
    pub stall_speed: ::core::ffi::c_double,
    pub recommended: VehicleProfile,
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
pub struct KRPCTransportBudget {
    pub read_calls: ::core::ffi::c_uint,
    pub read_wire_requests: ::core::ffi::c_uint,
    pub apply_calls: ::core::ffi::c_uint,
    pub apply_wire_requests: ::core::ffi::c_uint,
    pub total_calls: ::core::ffi::c_uint,
    pub total_wire_requests: ::core::ffi::c_uint,
    pub read_seconds: ::core::ffi::c_double,
    pub apply_seconds: ::core::ffi::c_double,
}
pub type KRPCWarpProgress = Option<
    unsafe extern "C" fn(
        *mut ::core::ffi::c_void,
        ::core::ffi::c_double,
        ::core::ffi::c_double,
        ::core::ffi::c_int,
        ::core::ffi::c_double,
    ) -> (),
>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlannerLogDecision {
    pub emit_scalar: bool,
    pub emit_trajectory: bool,
    pub forced: bool,
    pub durable: bool,
    pub phase_changed: bool,
    pub plan_changed: bool,
    pub constraint_changed: bool,
    pub predictor_discontinuity: bool,
    pub time_rewind: bool,
    pub executable_lineage: bool,
    pub event_flags: ::core::ffi::c_uint,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub parent_plan_id: uint64_t,
    pub parent_plan_version: uint64_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlannerLogSample {
    pub tick_sequence: uint64_t,
    pub ut: ::core::ffi::c_double,
    pub phase: GuidancePhase,
    pub telemetry: *const Telemetry,
    pub snapshot: *const LandingSnapshot,
    pub plan: *const EntryControlPlan,
    pub raw_prediction: *const Trajectory,
    pub published_prediction: *const Trajectory,
    pub calibration: *const TrajectoryCalibrationModel,
    pub envelope: *const AerodynamicEnvelope,
    pub constraint_active: bool,
    pub predictor_discontinuity: bool,
    pub explicit_force: bool,
    pub event_reason: *const ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleLogDecision {
    pub emit: bool,
    pub keyframe: bool,
    pub durable: bool,
    pub groups: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TerminalModelSourceFiles {
    pub atmosphere_csv: *const ::core::ffi::c_char,
    pub aero_csv: *const ::core::ffi::c_char,
    pub aero_book_csv: *const ::core::ffi::c_char,
    pub attitude_ini: *const ::core::ffi::c_char,
}
pub type ShuttleSimModelFile = ::core::ffi::c_uint;
pub const SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR: ShuttleSimModelFile = 4;
pub const SHUTTLE_SIM_MODEL_ATTITUDE: ShuttleSimModelFile = 3;
pub const SHUTTLE_SIM_MODEL_FORCE_BOOK: ShuttleSimModelFile = 2;
pub const SHUTTLE_SIM_MODEL_AERO: ShuttleSimModelFile = 1;
pub const SHUTTLE_SIM_MODEL_ATMOSPHERE: ShuttleSimModelFile = 0;
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
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
pub type TerminalSolverStatus = ::core::ffi::c_uint;
pub const TERMINAL_SOLVER_INVALID_INPUT: TerminalSolverStatus = 4;
pub const TERMINAL_SOLVER_SEARCH_EXHAUSTED: TerminalSolverStatus = 3;
pub const TERMINAL_SOLVER_UNQUALIFIED: TerminalSolverStatus = 2;
pub const TERMINAL_SOLVER_INFEASIBLE: TerminalSolverStatus = 1;
pub const TERMINAL_SOLVER_QUALIFIED: TerminalSolverStatus = 0;
pub type TaemPlanStatus = ::core::ffi::c_uint;
pub const TAEM_PLAN_QUALIFIED: TaemPlanStatus = 3;
pub const TAEM_PLAN_SEARCH_EXHAUSTED: TaemPlanStatus = 2;
pub const TAEM_PLAN_UNQUALIFIED: TaemPlanStatus = 1;
pub const TAEM_PLAN_INFEASIBLE: TaemPlanStatus = 0;
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
pub struct AsyncPredictionIdentity {
    pub guidance_phase: ::core::ffi::c_int,
    pub taem_phase: ::core::ffi::c_int,
    pub taem_transition_count: ::core::ffi::c_uint,
    pub control_plan_sequence: uint64_t,
    pub plan_id: uint64_t,
    pub plan_version: uint64_t,
    pub terminal_path_kind: ::core::ffi::c_int,
    pub terminal_path_committed: bool,
    pub hac_side_selected: bool,
    pub hac_captured: bool,
    pub final_approach_captured: bool,
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const EOF: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const LANDING_CONTROL_SURFACE_MAX: ::core::ffi::c_int = 16 as ::core::ffi::c_int;
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
pub const MM304_PLANNED_MAX_BANK_DEG: ::core::ffi::c_double = 40.0f64;
pub const FORECAST_FIT_WINDOW_S: ::core::ffi::c_double = 30.0f64;
pub const FORECAST_FIT_MIN_Q_PA: ::core::ffi::c_double = 100.0f64;
pub const FORECAST_SCALE_MIN: ::core::ffi::c_double = 0.5f64;
pub const FORECAST_SCALE_MAX: ::core::ffi::c_double = 2.0f64;
pub const FORECAST_HISTORY_MAX: ::core::ffi::c_int = 64 as ::core::ffi::c_int;
unsafe extern "C" fn monotonic_seconds() -> ::core::ffi::c_double {
    let mut ts: timespec = timespec { tv_sec: 0, tv_nsec: 0 };
    clock_gettime(CLOCK_MONOTONIC, &raw mut ts);
    return ts.tv_sec as ::core::ffi::c_double
        + ts.tv_nsec as ::core::ffi::c_double / 1e9f64;
}
unsafe extern "C" fn sleep_seconds(mut s: ::core::ffi::c_double) {
    if s <= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        return;
    }
    let mut ts: timespec = timespec {
        tv_sec: s as __time_t,
        tv_nsec: ((s - floor(s)) * 1e9f64) as __syscall_slong_t,
    };
    while nanosleep(&raw mut ts, &raw mut ts) < 0 as ::core::ffi::c_int
        && *__errno_location() == EINTR
    {}
}
unsafe extern "C" fn terminal_model_path(
    mut environment_name: *const ::core::ffi::c_char,
    mut kind: ShuttleSimModelFile,
    mut buffer: *mut ::core::ffi::c_char,
    mut buffer_size: size_t,
) -> *const ::core::ffi::c_char {
    let mut configured: *const ::core::ffi::c_char = getenv(environment_name);
    if !configured.is_null()
        && *configured.offset(0 as ::core::ffi::c_int as isize) as ::core::ffi::c_int
            != 0
    {
        return configured;
    }
    return shuttle_sim_model_path(
        ::core::ptr::null::<::core::ffi::c_char>(),
        kind,
        buffer,
        buffer_size,
    );
}
unsafe extern "C" fn controller_capture_terminal_model(
    mut c: *mut LandingController,
    mut cfg: *const LandingConfiguration,
) -> bool {
    let mut reason: [::core::ffi::c_char; 256] = [0; 256];
    if !shuttle_sim_model_matches_vehicle(
        &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char,
        &raw mut reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
    ) {
        (*c).terminal_model_valid = false_0 != 0;
        fprintf(
            stderr,
            b"MM305 native model unavailable: %s\n\0" as *const u8
                as *const ::core::ffi::c_char,
            &raw mut reason as *mut ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut atmosphere: [::core::ffi::c_char; 512] = [0; 512];
    let mut aero: [::core::ffi::c_char; 512] = [0; 512];
    let mut book: [::core::ffi::c_char; 512] = [0; 512];
    let mut attitude: [::core::ffi::c_char; 512] = [0; 512];
    let mut files: TerminalModelSourceFiles = TerminalModelSourceFiles {
        atmosphere_csv: terminal_model_path(
            b"KSP_LANDER_TERMINAL_ATMOSPHERE\0" as *const u8
                as *const ::core::ffi::c_char,
            SHUTTLE_SIM_MODEL_ATMOSPHERE,
            &raw mut atmosphere as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        ),
        aero_csv: terminal_model_path(
            b"KSP_LANDER_TERMINAL_AERO\0" as *const u8 as *const ::core::ffi::c_char,
            SHUTTLE_SIM_MODEL_AERO,
            &raw mut aero as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        ),
        aero_book_csv: terminal_model_path(
            b"KSP_LANDER_TERMINAL_AERO_BOOK\0" as *const u8
                as *const ::core::ffi::c_char,
            SHUTTLE_SIM_MODEL_FORCE_BOOK,
            &raw mut book as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        ),
        attitude_ini: terminal_model_path(
            b"KSP_LANDER_TERMINAL_ATTITUDE\0" as *const u8 as *const ::core::ffi::c_char,
            SHUTTLE_SIM_MODEL_ATTITUDE,
            &raw mut attitude as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        ),
    };
    if files.atmosphere_csv.is_null() || files.aero_csv.is_null()
        || files.aero_book_csv.is_null() || files.attitude_ini.is_null()
    {
        (*c).terminal_model_valid = false_0 != 0;
        fprintf(
            stderr,
            b"MM305 native model unavailable: model path too long\n\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    (*c).terminal_model_capture_sequence = (*c)
        .terminal_model_capture_sequence
        .wrapping_add(1);
    let mut snapshot_id: uint64_t = (*c).terminal_model_capture_sequence;
    if snapshot_id == 0 {
        (*c).terminal_model_capture_sequence = (*c)
            .terminal_model_capture_sequence
            .wrapping_add(1);
        snapshot_id = (*c).terminal_model_capture_sequence;
    }
    (*c).terminal_model_valid = terminal_model_capture(
        &raw mut (*c).terminal_model,
        &raw mut files,
        cfg,
        snapshot_id,
        &raw mut reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
    );
    if (*c).terminal_model_valid {
        (*c).terminal_model_valid = terminal_model_align_planet_frame(
            &raw mut (*c).terminal_model,
            krpc_session_planet((*c).session),
            &raw mut reason as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        );
    }
    if !(*c).terminal_model_valid {
        fprintf(
            stderr,
            b"MM305 native model unavailable: %s\n\0" as *const u8
                as *const ::core::ffi::c_char,
            &raw mut reason as *mut ::core::ffi::c_char,
        );
    }
    return (*c).terminal_model_valid;
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_create(
    mut cfg: *const LandingConfiguration,
    mut cb: SnapshotCallback,
    mut ctx: *mut ::core::ffi::c_void,
) -> *mut LandingController {
    let mut c: *mut LandingController = calloc(
        1 as size_t,
        ::core::mem::size_of::<LandingController>() as size_t,
    ) as *mut LandingController;
    if c.is_null() {
        return ::core::ptr::null_mut::<LandingController>();
    }
    pthread_mutex_init(&raw mut (*c).mutex, ::core::ptr::null::<pthread_mutexattr_t>());
    (*c).configuration = if !cfg.is_null() {
        *cfg
    } else {
        landing_configuration_default()
    };
    landing_configuration_normalize(&raw mut (*c).configuration);
    guidance_machine_init(&raw mut (*c).guidance);
    adaptive_calibrator_init(
        &raw mut (*c).adaptive,
        &raw mut (*c).configuration.vehicle,
    );
    glide_calibration_init(&raw mut (*c).glide);
    trajectory_calibrator_init(&raw mut (*c).trajectory_calibrator);
    trajectory_init(&raw mut (*c).dynamic_trajectory);
    trajectory_init(&raw mut (*c).stabilized_trajectory);
    landing_snapshot_init(&raw mut (*c).snapshot, &raw mut (*c).configuration.vehicle);
    (*c).adaptive_profile = (*c).configuration.vehicle;
    (*c).aerodynamics = AerodynamicModel {
        lift_to_drag: (*c).configuration.vehicle.estimated_lift_to_drag,
        ballistic_coefficient: (*c)
            .configuration
            .vehicle
            .estimated_ballistic_coefficient,
        confidence: 0.2f64,
    };
    (*c).planning_aerodynamics = (*c).aerodynamics;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 4 as ::core::ffi::c_int {
        (*c).envelope.regimes[i as usize] = (*c).aerodynamics;
        i += 1;
    }
    (*c).trajectory_calibration = TrajectoryCalibrationModel {
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
        altitude_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        range_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        confidence: 0.05f64,
        accepted_samples: 0 as ::core::ffi::c_int,
    };
    guidance_command_init(&raw mut (*c).last_command);
    (*c).cached_prediction_ut = -1e300f64;
    (*c).last_prediction_ut = (*c).cached_prediction_ut;
    (*c).last_trajectory_ut = (*c).last_prediction_ut;
    (*c).callback = cb;
    (*c).callback_context = ctx;
    return c;
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_destroy(mut c: *mut LandingController) {
    if c.is_null() {
        return;
    }
    landing_controller_shutdown(c);
    pthread_mutex_lock(&raw mut (*c).mutex);
    clear_snapshot_dynamic(&raw mut (*c).snapshot);
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    trajectory_calibrator_clear(&raw mut (*c).trajectory_calibrator);
    if (*c).has_plan {
        deorbit_plan_clear(&raw mut (*c).plan);
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    pthread_mutex_destroy(&raw mut (*c).mutex);
    free(c as *mut ::core::ffi::c_void);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_configuration(
    mut c: *mut LandingController,
) -> LandingConfiguration {
    pthread_mutex_lock(&raw mut (*c).mutex);
    let mut r: LandingConfiguration = (*c).configuration;
    pthread_mutex_unlock(&raw mut (*c).mutex);
    return r;
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_update_configuration(
    mut c: *mut LandingController,
    mut cfg: *const LandingConfiguration,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    let mut before: LandingConfiguration = (*c).configuration;
    let mut next: LandingConfiguration = *cfg;
    let mut before_vehicle: VehicleProfile = before.vehicle;
    landing_configuration_normalize(&raw mut next);
    let mut changed: bool = !configuration_equal(&raw mut before, &raw mut next);
    let mut planning_changed: bool = !planning_configuration_equal(
        &raw mut before,
        &raw mut next,
    );
    if !changed {
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    (*c).configuration = next;
    if !(*c).session.is_null() && !(*c).guidance.terminal_region_entered {
        controller_capture_terminal_model(c, &raw mut (*c).configuration);
    }
    if !vehicle_profile_equal(
        &raw mut before_vehicle,
        &raw mut (*c).configuration.vehicle,
    ) && !(*c).glide.active
    {
        reset_models(c, &raw mut (*c).configuration.vehicle);
        if !(*c).session.is_null() {
            krpc_session_seed_physics((*c).session, &raw mut (*c).physics);
            apply_certified_aero_prior(c);
        }
    }
    if (*c).has_plan as ::core::ffi::c_int != 0
        && planning_changed as ::core::ffi::c_int != 0
    {
        (*c).plan_stale = true_0 != 0;
        set_warning(
            c,
            b"Planning-relevant configuration changed after planning. Create a new deorbit plan before engagement.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    append_configuration_log(c);
    log_state_event(
        c,
        if planning_changed as ::core::ffi::c_int != 0 {
            b"event=configuration-changed; replan_required=true\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"event=configuration-changed; replan_required=false\0" as *const u8
                as *const ::core::ffi::c_char
        },
        true_0 != 0,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_connect(mut c: *mut LandingController) {
    stop_thread(c);
    pthread_mutex_lock(&raw mut (*c).mutex);
    disconnect_locked(c, false_0 != 0);
    reset_snapshot(
        c,
        if !getenv(b"KSP_LANDER_SIMULATOR\0" as *const u8 as *const ::core::ffi::c_char)
            .is_null()
        {
            b"Opening ShuttleSim lockstep backend\xE2\x80\xA6\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"Opening native kRPC C-Nano link\xE2\x80\xA6\0" as *const u8
                as *const ::core::ffi::c_char
        },
    );
    (*c).snapshot.connection_status = CONN_CONNECTING;
    publish(c);
    let mut cfg: LandingConfiguration = (*c).configuration;
    pthread_mutex_unlock(&raw mut (*c).mutex);
    let mut err: [::core::ffi::c_char; 768] = [0; 768];
    let mut s: *mut KRPCSession = krpc_session_open(
        &raw mut cfg,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
    );
    pthread_mutex_lock(&raw mut (*c).mutex);
    if s.is_null() {
        (*c).snapshot.connection_status = CONN_FAILED;
        (*c).snapshot.phase = PHASE_FAULT;
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Native kRPC C-Nano connection failed.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        set_error(c, &raw mut err as *mut ::core::ffi::c_char);
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    (*c).session = s;
    snprintf(
        &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        shuttle_sim_resolve_vehicle_identity(
            krpc_session_vessel(s),
            &raw mut (*c).configuration.vehicle.model_id as *mut ::core::ffi::c_char,
        ),
    );
    start_log(c);
    reset_models(c, &raw mut (*c).configuration.vehicle);
    krpc_session_seed_physics(s, &raw mut (*c).physics);
    apply_certified_aero_prior(c);
    controller_capture_terminal_model(c, &raw mut (*c).configuration);
    guidance_machine_init(&raw mut (*c).guidance);
    glide_calibration_init(&raw mut (*c).glide);
    (*c).has_prediction_cache = false_0 != 0;
    (*c).time_warp_issued = false_0 != 0;
    (*c).burn_engines_shut_down = false_0 != 0;
    (*c).snapshot.connection_status = CONN_CONNECTED;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"%s connected to %s around %s with %u %s aerodynamic cells.\0" as *const u8
            as *const ::core::ffi::c_char,
        if krpc_session_is_simulator(s) as ::core::ffi::c_int != 0 {
            b"ShuttleSim lockstep\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"Native kRPC C-Nano\0" as *const u8 as *const ::core::ffi::c_char
        },
        krpc_session_vessel(s),
        &raw const (*(krpc_session_planet
            as unsafe extern "C" fn(*const KRPCSession) -> *const PlanetModel)(s))
            .name as *const ::core::ffi::c_char,
        (*c).physics.count,
        if shuttle_sim_model_matches_vehicle(
            &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        ) as ::core::ffi::c_int != 0
        {
            b"certified\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"self-learned\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
    if !shuttle_sim_model_matches_vehicle(
        &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char,
        ::core::ptr::null_mut::<::core::ffi::c_char>(),
        0 as size_t,
    ) {
        let mut warning: [::core::ffi::c_char; 320] = [0; 320];
        snprintf(
            &raw mut warning as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 320]>() as size_t,
            b"UNCERTIFIED craft '%s': no certified aerodynamic profile matches it (profile %s describes %s). Flying from configured seeds; aerodynamics learn only from this craft's flight data, and no certified terminal model is used.\0"
                as *const u8 as *const ::core::ffi::c_char,
            &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char,
            if !shuttle_sim_model_profile().is_null() {
                shuttle_sim_model_profile()
            } else {
                b"unknown\0" as *const u8 as *const ::core::ffi::c_char
            },
            if !shuttle_sim_model_vehicle_id().is_null() {
                shuttle_sim_model_vehicle_id()
            } else {
                b"unknown\0" as *const u8 as *const ::core::ffi::c_char
            },
        );
        set_warning(c, &raw mut warning as *mut ::core::ffi::c_char);
    }
    (*c).snapshot.adaptive_vehicle_profile = (*c).adaptive_profile;
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
    start_thread(c);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_disconnect(mut c: *mut LandingController) {
    stop_thread(c);
    pthread_mutex_lock(&raw mut (*c).mutex);
    disconnect_locked(c, true_0 != 0);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_create_plan(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if (*c).session.is_null() {
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).glide.active {
        set_warning(
            c,
            b"Stop the glide calibration before creating a deorbit plan.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        log_state_event(
            c,
            b"event=planning-blocked; reason=calibration-active\0" as *const u8
                as *const ::core::ffi::c_char,
            false_0 != 0,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut err: [::core::ffi::c_char; 768] = [0; 768];
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
    let mut s: VehicleState = VehicleState {
        ut: 0.,
        position: Vector3 { x: 0., y: 0., z: 0. },
        velocity: Vector3 { x: 0., y: 0., z: 0. },
        mass: 0.,
    };
    if !(*c).terminal_model_valid
        || !terminal_model_policy_matches(
            &raw mut (*c).terminal_model,
            &raw mut cfg.site,
            &raw mut cfg.vehicle,
            &raw mut cfg.guidance,
        )
    {
        controller_capture_terminal_model(c, &raw mut cfg);
    }
    if !read_live_sample(
        c,
        &raw mut cfg,
        &raw mut t,
        &raw mut s,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
    ) {
        (*c).snapshot.phase = PHASE_FAULT;
        set_error(c, &raw mut err as *mut ::core::ffi::c_char);
        log_state_event(
            c,
            b"event=planning-failed; reason=telemetry-read\0" as *const u8
                as *const ::core::ffi::c_char,
            true_0 != 0,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    (*c).latest_telemetry = t;
    (*c).latest_state = s;
    (*c).has_latest = true_0 != 0;
    landing_snapshot_set_sample(&raw mut (*c).snapshot, &raw mut t, &raw mut s);
    (*c).snapshot.phase = PHASE_PLANNING;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Searching orbital opportunities and stress-testing finite-burn atmospheric entry\xE2\x80\xA6\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
    publish(c);
    let mut planet: *const PlanetModel = krpc_session_planet((*c).session);
    if t.apoapsis_altitude <= (*planet).atmosphere_depth
        || t.periapsis_altitude <= (*planet).atmosphere_depth
    {
        (*c).snapshot.phase = PHASE_FAULT;
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Planning rejected.\0" as *const u8 as *const ::core::ffi::c_char,
        );
        set_error(
            c,
            b"Initial apoapsis and periapsis must both be outside the active planet atmosphere.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        log_state_event(
            c,
            b"event=planning-failed; reason=orbit-inside-atmosphere\0" as *const u8
                as *const ::core::ffi::c_char,
            true_0 != 0,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut p: DeorbitPlan = DeorbitPlan {
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
    if !deorbit_plan_create_with_terminal_model(
        &raw mut p,
        s,
        t.orbit_period,
        krpc_session_planet((*c).session),
        (*c).planning_aerodynamics,
        &raw mut (*c).envelope,
        &raw mut (*c).trajectory_calibration,
        &raw mut cfg.site,
        &raw mut cfg.vehicle,
        &raw mut cfg.guidance,
        t.available_thrust,
        &raw mut t,
        if (*c).terminal_model_valid as ::core::ffi::c_int != 0 {
            &raw mut (*c).terminal_model
        } else {
            ::core::ptr::null_mut::<TerminalModel>()
        },
    ) {
        (*c).snapshot.phase = PHASE_FAULT;
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"No usable deorbit solution was found.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        set_error(
            c,
            b"The current orbit, landing site, thrust or calibrated aerodynamic profile did not produce a safe finite-burn atmospheric-entry solution across the searched opportunities.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        log_state_event(
            c,
            b"event=planning-failed; reason=no-executable-deorbit-solution\0"
                as *const u8 as *const ::core::ffi::c_char,
            true_0 != 0,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).has_plan {
        deorbit_plan_clear(&raw mut (*c).plan);
    }
    (*c).plan = p;
    (*c).has_plan = true_0 != 0;
    (*c).plan_stale = false_0 != 0;
    (*c).time_warp_issued = false_0 != 0;
    (*c).burn_engines_shut_down = false_0 != 0;
    (*c).log_plan_trajectory_dirty = true_0 != 0;
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    trajectory_copy(&raw mut (*c).dynamic_trajectory, &raw mut p.trajectory);
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    trajectory_copy(&raw mut (*c).stabilized_trajectory, &raw mut p.trajectory);
    guidance_reset_plan(&raw mut (*c).guidance);
    t.predicted_miss_distance = p.predicted_closest_distance;
    t.predicted_entry_range = p.predicted_entry_range;
    t.predicted_entry_flight_path_angle = p.predicted_entry_flight_path_angle;
    trajectory_calibrator_set_forecast(
        &raw mut (*c).trajectory_calibrator,
        &raw mut p.trajectory,
    );
    landing_snapshot_set_sample(
        &raw mut (*c).snapshot,
        &raw mut t,
        &raw mut (*c).latest_state,
    );
    (*c).snapshot.phase = PHASE_IDLE;
    if p.target_capture_achieved {
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Robust full-route plan ready. %u/%u strict recovery scenarios pass (%.0f%%).\0"
                as *const u8 as *const ::core::ffi::c_char,
            p.robustness_passed,
            p.robustness_scenarios,
            p.robustness_pass_fraction
                * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    } else if p.execution_qualified {
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Entry-stage qualified. %u/%u burn/entry stress cases pass (%.0f%%); downstream MM304/TAEM shadow capture remains advisory.\0"
                as *const u8 as *const ::core::ffi::c_char,
            p.entry_stage_passed,
            p.entry_stage_scenarios,
            p.entry_stage_pass_fraction
                * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    } else {
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Best available plan selected under warning: finite-burn or atmospheric-entry qualification did not pass.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    set_warning(
        c,
        if p.target_capture_achieved as ::core::ffi::c_int != 0 {
            ::core::ptr::null::<::core::ffi::c_char>()
        } else if p.execution_qualified as ::core::ffi::c_int != 0 {
            b"Orbital burn and safe atmospheric entry are qualified. Review the separate MM304/TAEM shadow result; it is not the orbital-stage admission gate.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"Best available orbital plan is unqualified. Live execution may continue for testing under warning; runtime abort protections and all downstream transition/handoff gates remain active.\0"
                as *const u8 as *const ::core::ffi::c_char
        },
    );
    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
    reference_trajectory(
        &raw mut (*c).snapshot.reference_trajectory,
        &raw mut cfg.site,
        &raw mut cfg.guidance,
        (*krpc_session_planet((*c).session)).radius,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    (*c).log_reference_trajectory_dirty = true_0 != 0;
    let mut plan_reason: [::core::ffi::c_char; 256] = [0; 256];
    snprintf(
        &raw mut plan_reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"event=plan-created; execution-qualified=%s; strict-capture=%s; entry-stage=%s; entry-stage-pass=%.6g; downstream-shadow-capture=%s\0"
            as *const u8 as *const ::core::ffi::c_char,
        if p.execution_qualified as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        if p.target_capture_achieved as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        if p.entry_stage_qualified as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        p.entry_stage_pass_fraction,
        if p.nominal_capture_achieved as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    log_state_event(c, &raw mut plan_reason as *mut ::core::ffi::c_char, true_0 != 0);
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_engage(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    if (*c).glide.active {
        set_warning(
            c,
            b"Stop calibration before engaging landing guidance.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    } else if !(*c).has_plan {
        set_warning(
            c,
            b"Create and review a deorbit plan before engaging guidance.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    } else if (*c).plan_stale {
        set_warning(
            c,
            b"Engagement blocked because planning-relevant configuration changed. Create a new deorbit plan.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    } else if (*c).has_latest as ::core::ffi::c_int != 0 && !(*c).session.is_null()
        && (*c).latest_telemetry.ut
            > (*c).plan.burn_ut + (*c).plan.estimated_burn_duration * 0.5f64
        && (*c).latest_telemetry.mean_altitude
            > (*krpc_session_planet((*c).session)).atmosphere_depth
    {
        set_warning(
            c,
            b"Engagement blocked because the selected deorbit-burn window has passed. Create a new plan.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    } else {
        guidance_set_engaged(&raw mut (*c).guidance, true_0 != 0);
        (*c).snapshot.automation_engaged = true_0 != 0;
        (*c).snapshot.paused = false_0 != 0;
        set_warning(
            c,
            if !(*c).plan.execution_qualified {
                b"Best available unqualified plan engaged under warning. Runtime abort protections and all downstream transition/handoff gates remain active.\0"
                    as *const u8 as *const ::core::ffi::c_char
            } else if (*c).plan.execution_degraded as ::core::ffi::c_int != 0 {
                b"Guarded recovery execution engaged. Live cutoff and achieved-state verification will supervise the burn and entry handoff.\0"
                    as *const u8 as *const ::core::ffi::c_char
            } else {
                ::core::ptr::null::<::core::ffi::c_char>()
            },
        );
    }
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_engage_reentry(
    mut c: *mut LandingController,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    if (*c).session.is_null() {
        set_warning(
            c,
            b"Connect to KSP before starting reentry-only guidance.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).glide.active {
        set_warning(
            c,
            b"Stop calibration before starting reentry-only guidance.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut err: [::core::ffi::c_char; 768] = [
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
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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
    let mut state: VehicleState = VehicleState {
        ut: 0.,
        position: Vector3 { x: 0., y: 0., z: 0. },
        velocity: Vector3 { x: 0., y: 0., z: 0. },
        mass: 0.,
    };
    if !read_live_sample(
        c,
        &raw mut cfg,
        &raw mut t,
        &raw mut state,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
    ) {
        set_error(c, &raw mut err as *mut ::core::ffi::c_char);
        (*c).snapshot.phase = PHASE_FAULT;
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    let mut sim_shadow: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_SIM_SHADOW\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut simulator_fast_qualification: bool = krpc_session_is_simulator((*c).session)
        as ::core::ffi::c_int != 0
        && !(!sim_shadow.is_null()
            && strcmp(sim_shadow, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int);
    let mut airborne: bool = strcasecmp(
        &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) != 0
        && strcasecmp(
            &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
            b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0
        && strcasecmp(
            &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
            b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0;
    let mut in_atmosphere: bool = planet_atmospheric_density(p, t.mean_altitude)
        > DBL_EPSILON;
    let mut descending: bool = t.vertical_speed < 0.0f64;
    if !airborne || !descending && in_atmosphere as ::core::ffi::c_int != 0 {
        set_warning(
            c,
            b"Reentry continuation requires an airborne vehicle already descending once atmospheric forces are present.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if !in_atmosphere {
        let mut periapsis_enters_atmosphere: bool = t.periapsis_altitude.is_finite()
            as i32 != 0 && t.periapsis_altitude > cfg.site.altitude
            && t.periapsis_altitude < (*p).atmosphere_depth;
        if !periapsis_enters_atmosphere {
            set_warning(
                c,
                b"Reentry continuation requires an osculating periapsis inside the atmosphere and above the surface.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
            publish(c);
            pthread_mutex_unlock(&raw mut (*c).mutex);
            return;
        }
    }
    let mut initial_roll: ::core::ffi::c_double = norm_signed_deg(t.roll);
    let mut initial_s_turn_sign: ::core::ffi::c_double = 0.;
    if fabs(initial_roll) > DBL_EPSILON {
        initial_s_turn_sign = if initial_roll < 0.0f64 { -1.0f64 } else { 1.0f64 };
    } else if fabs(t.runway_cross_track) > DBL_EPSILON {
        initial_s_turn_sign = if t.runway_cross_track > 0.0f64 {
            -1.0f64
        } else {
            1.0f64
        };
    } else {
        let mut course_error: ::core::ffi::c_double = norm_signed_deg(
            t.ground_track_heading - cfg.site.runway_heading,
        );
        initial_s_turn_sign = if course_error < 0.0f64 { -1.0f64 } else { 1.0f64 };
    }
    let mut live_course: ::core::ffi::c_double = surface_course(
        state.position,
        state.velocity,
        planet_rotation_vector(p),
        (*p).north_axis,
        t.heading,
    );
    let mut terminal_restart_range: ::core::ffi::c_double = entry_taem_range_target(
        p,
        &raw mut cfg.guidance,
    );
    let mut restart_sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        p,
        t.mean_altitude,
    );
    let mut restart_mach: ::core::ffi::c_double = if t.mach.is_finite() as i32 != 0
        && t.mach > 0.0f64
    {
        t.mach
    } else if restart_sound.is_finite() as i32 != 0 && restart_sound > 0.0f64 {
        t.true_air_speed / restart_sound
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    let mut late_terminal_restart: bool = in_atmosphere as ::core::ffi::c_int != 0
        && descending as ::core::ffi::c_int != 0 && live_course.is_finite() as i32 != 0
        && t.range_to_site.is_finite() as i32 != 0
        && restart_mach.is_finite() as i32 != 0
        && t.mean_altitude <= cfg.guidance.mm305_max_altitude
        && restart_mach
            <= cfg.guidance.mm305_target_mach + cfg.guidance.mm305_mach_half_width
        && t.range_to_site <= terminal_restart_range
        && t.true_air_speed > cfg.vehicle.minimum_safe_speed;
    if late_terminal_restart {
        let mut status: [::core::ffi::c_char; 384] = [
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
        ];
        if !guidance_begin_mm305_continuation(
            &raw mut (*c).guidance,
            &raw mut t,
            live_course,
            p,
            (*c).aerodynamics,
            &raw mut cfg,
            &raw mut status as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        ) {
            set_warning(
                c,
                if status[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
                    &raw mut status as *mut ::core::ffi::c_char
                        as *const ::core::ffi::c_char
                } else {
                    b"The live terminal state could not initialize MM305 continuation.\0"
                        as *const u8 as *const ::core::ffi::c_char
                },
            );
            publish(c);
            pthread_mutex_unlock(&raw mut (*c).mutex);
            return;
        }
        if (*c).has_plan {
            deorbit_plan_clear(&raw mut (*c).plan);
            (*c).has_plan = false_0 != 0;
        }
        (*c).plan_stale = false_0 != 0;
        (*c).time_warp_issued = true_0 != 0;
        (*c).has_prediction_cache = false_0 != 0;
        (*c).last_prediction_ut = -1e300f64;
        trajectory_clear(&raw mut (*c).dynamic_trajectory);
        trajectory_init(&raw mut (*c).dynamic_trajectory);
        trajectory_clear(&raw mut (*c).stabilized_trajectory);
        trajectory_init(&raw mut (*c).stabilized_trajectory);
        trajectory_clear(&raw mut (*c).trajectory_calibrator.forecast);
        trajectory_init(&raw mut (*c).trajectory_calibrator.forecast);
        (*c).latest_telemetry = t;
        (*c).latest_state = state;
        (*c).has_latest = true_0 != 0;
        landing_snapshot_set_sample(
            &raw mut (*c).snapshot,
            &raw mut t,
            &raw mut (*c).latest_state,
        );
        (*c).snapshot.plan = ::core::ptr::null_mut::<DeorbitPlan>();
        (*c).snapshot.phase = PHASE_TAEM;
        (*c).snapshot.automation_engaged = true_0 != 0;
        (*c).snapshot.paused = false_0 != 0;
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut status as *mut ::core::ffi::c_char,
        );
        set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
        set_warning(c, ::core::ptr::null::<::core::ffi::c_char>());
        log_state_event(
            c,
            b"event=reentry-continuation-armed; late-terminal-restart=true; mm305-ownership-adopted=true\0"
                as *const u8 as *const ::core::ffi::c_char,
            true_0 != 0,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut qualification_guidance: GuidanceMachine = GuidanceMachine {
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
        &raw mut qualification_guidance,
        &raw mut t,
        p,
        &raw mut cfg,
        initial_s_turn_sign,
        false_0 != 0,
        &raw mut (*c).envelope,
        &raw mut (*c).trajectory_calibration,
    );
    let mut qualification_plan: DeorbitPlan = DeorbitPlan {
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
        &raw mut qualification_plan as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
    trajectory_init(&raw mut qualification_plan.trajectory);
    qualification_plan.created_ut = t.ut;
    qualification_plan.burn_ut = t.ut;
    qualification_plan.predicted_post_burn_periapsis_altitude = if t
        .periapsis_altitude
        .is_finite() as i32 != 0
    {
        t.periapsis_altitude
    } else {
        cfg.site.altitude
    };
    qualification_plan.live_cutoff_capture_qualified = true_0 != 0;
    qualification_plan.execution_qualified = true_0 != 0;
    qualification_plan.achieved_state_verified = true_0 != 0;
    qualification_plan.achieved_state_capture_qualified = true_0 != 0;
    let mut prediction: EntryPrediction = EntryPrediction {
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
        &raw mut prediction as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryPrediction>() as size_t,
    );
    trajectory_init(&raw mut prediction.trajectory);
    if simulator_fast_qualification {
        prediction.entered_atmosphere = in_atmosphere;
        prediction.entry_range = t.range_to_site;
        prediction.entry_flight_path_angle = t.flight_path_angle;
        prediction.entry_speed = t.true_air_speed;
        prediction.taem_distance = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.taem_range_error = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.closest_distance = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.taem_speed = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.taem_flight_path_angle = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.minimum_entry_speed = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.peak_dynamic_pressure = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.peak_g_load = ::core::f32::NAN as ::core::ffi::c_double;
        prediction.final_state = state;
    } else {
        let mut gravity: ::core::ffi::c_double = planet_surface_gravity(p);
        let mut height: ::core::ffi::c_double = fmax(
            0.0f64,
            t.mean_altitude - cfg.site.altitude,
        );
        let mut horizontal_time: ::core::ffi::c_double = if t.horizontal_speed
            > DBL_EPSILON
        {
            fmax(0.0f64, t.range_to_site) / t.horizontal_speed
        } else {
            0.0f64
        };
        let mut vertical_time: ::core::ffi::c_double = if -t.vertical_speed > DBL_EPSILON
        {
            height / -t.vertical_speed
        } else if gravity > DBL_EPSILON {
            sqrt(2.0f64 * height / gravity)
        } else {
            0.0f64
        };
        let mut terminal_path: ::core::ffi::c_double = 1.5f64 * LANDER_PI
            * cfg.guidance.hac_radius + cfg.guidance.final_approach_distance;
        let mut terminal_time: ::core::ffi::c_double = terminal_path
            / fmax(cfg.vehicle.minimum_safe_speed, DBL_EPSILON);
        let mut qualification_horizon: ::core::ffi::c_double = fmax(
            horizontal_time,
            vertical_time,
        ) + terminal_time;
        let mut entry_model: *const TerminalModel = if cfg.guidance.taem_energy_law
            as ::core::ffi::c_int != 0
            && strcmp(
                &raw mut cfg.vehicle.model_id as *mut ::core::ffi::c_char,
                b"STS\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0 && (*c).terminal_model_valid as ::core::ffi::c_int != 0
        {
            &raw mut (*c).terminal_model
        } else {
            ::core::ptr::null_mut::<TerminalModel>()
        };
        prediction = predictor_simulate_entry_guidance_shadow_with_terminal_model(
            state,
            &raw mut t,
            &raw mut qualification_guidance,
            &raw mut qualification_plan,
            p,
            (*c).planning_aerodynamics,
            &raw mut (*c).envelope,
            &raw mut (*c).trajectory_calibration,
            &raw mut cfg,
            entry_model as *const TerminalModel,
            qualification_horizon,
            true_0 != 0,
        );
        trajectory_clear(&raw mut qualification_plan.trajectory);
    }
    let mut shadow_qualified: bool = !simulator_fast_qualification
        && reentry_guidance_shadow_recovery_qualified(
            &raw mut prediction,
            &raw mut cfg.vehicle,
            &raw mut cfg.guidance,
        ) as ::core::ffi::c_int != 0;
    let mut recovery: bool = simulator_fast_qualification as ::core::ffi::c_int != 0
        || shadow_qualified as ::core::ffi::c_int != 0;
    let mut continue_policy: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_CONTINUE_UNQUALIFIED\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut warning_continuation: bool = !recovery && !continue_policy.is_null()
        && strcmp(continue_policy, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int;
    let mut continuation_warning: [::core::ffi::c_char; 768] = [
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
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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
    if !recovery {
        let mut msg: [::core::ffi::c_char; 768] = [0; 768];
        snprintf(
            &raw mut msg as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
            b"Reentry rejected by closed-loop shadow: entered=%s, MM305=%s, closest=%.1f km, range error=%+.1f km, q=%.1f kPa, g=%.2f.\0"
                as *const u8 as *const ::core::ffi::c_char,
            if prediction.entered_atmosphere as ::core::ffi::c_int != 0 {
                b"true\0" as *const u8 as *const ::core::ffi::c_char
            } else {
                b"false\0" as *const u8 as *const ::core::ffi::c_char
            },
            if prediction.reached_taem as ::core::ffi::c_int != 0 {
                b"true\0" as *const u8 as *const ::core::ffi::c_char
            } else {
                b"false\0" as *const u8 as *const ::core::ffi::c_char
            },
            prediction.closest_distance / 1000.0f64,
            prediction.taem_range_error / 1000.0f64,
            prediction.peak_dynamic_pressure / 1000.0f64,
            prediction.peak_g_load,
        );
        snprintf(
            &raw mut continuation_warning as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut msg as *mut ::core::ffi::c_char,
        );
        if !warning_continuation {
            entry_prediction_clear(&raw mut prediction);
            set_warning(c, &raw mut msg as *mut ::core::ffi::c_char);
            publish(c);
            pthread_mutex_unlock(&raw mut (*c).mutex);
            return;
        }
    }
    let mut plan: DeorbitPlan = DeorbitPlan {
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
        &raw mut plan as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
    trajectory_init(&raw mut plan.trajectory);
    plan.created_ut = t.ut;
    plan.burn_ut = t.ut;
    plan.delta_v = 0.0f64;
    plan.estimated_burn_duration = 0.0f64;
    plan.predicted_taem_distance = prediction.taem_distance;
    plan.predicted_taem_range_error = prediction.taem_range_error;
    plan.predicted_closest_distance = prediction.closest_distance;
    plan.predicted_entry_range = if shadow_qualified as ::core::ffi::c_int != 0 {
        prediction.entry_range
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.predicted_entry_flight_path_angle = if shadow_qualified as ::core::ffi::c_int
        != 0
    {
        prediction.entry_flight_path_angle
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.predicted_post_burn_periapsis_altitude = t.periapsis_altitude;
    plan.nominal_capture_achieved = shadow_qualified;
    plan.robustness_qualified = shadow_qualified;
    plan.target_capture_achieved = shadow_qualified;
    plan.robustness_scenarios = (if shadow_qualified as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    }) as ::core::ffi::c_uint;
    plan.robustness_passed = (if shadow_qualified as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    }) as ::core::ffi::c_uint;
    plan.robustness_unsafe = 0 as ::core::ffi::c_uint;
    plan.robustness_pass_fraction = if shadow_qualified as ::core::ffi::c_int != 0 {
        1.0f64
    } else {
        0.0f64
    };
    plan.execution_qualified = shadow_qualified;
    plan.execution_degraded = !shadow_qualified;
    plan.recovery_passed = (if shadow_qualified as ::core::ffi::c_int != 0 {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    }) as ::core::ffi::c_uint;
    plan.recovery_pass_fraction = if shadow_qualified as ::core::ffi::c_int != 0 {
        1.0f64
    } else {
        0.0f64
    };
    plan.worst_case_closest_distance = if shadow_qualified as ::core::ffi::c_int != 0 {
        prediction.closest_distance
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.worst_case_taem_range_error = if shadow_qualified as ::core::ffi::c_int != 0 {
        fabs(prediction.taem_range_error)
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.worst_case_entry_flight_path_angle = if shadow_qualified as ::core::ffi::c_int
        != 0
    {
        prediction.entry_flight_path_angle
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.worst_case_post_burn_periapsis_altitude = if shadow_qualified
        as ::core::ffi::c_int != 0
    {
        t.periapsis_altitude
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.worst_case_peak_dynamic_pressure = if shadow_qualified as ::core::ffi::c_int
        != 0
    {
        prediction.peak_dynamic_pressure
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.worst_case_peak_g_load = if shadow_qualified as ::core::ffi::c_int != 0 {
        prediction.peak_g_load
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.live_cutoff_capture_qualified = shadow_qualified;
    plan.live_cutoff_periapsis_altitude = t.periapsis_altitude;
    plan.live_cutoff_closest_distance = if shadow_qualified as ::core::ffi::c_int != 0 {
        prediction.closest_distance
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.live_cutoff_entry_flight_path_angle = if shadow_qualified as ::core::ffi::c_int
        != 0
    {
        prediction.entry_flight_path_angle
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    plan.achieved_state_verified = shadow_qualified;
    plan.achieved_state_capture_qualified = shadow_qualified;
    plan.achieved_post_burn_periapsis_altitude = t.periapsis_altitude;
    plan.confidence = clampd((*c).planning_aerodynamics.confidence, 0.0f64, 1.0f64);
    guidance_initialize_reentry_continuation(
        &raw mut (*c).guidance,
        &raw mut t,
        p,
        &raw mut cfg,
        initial_s_turn_sign,
        false_0 != 0,
        &raw mut (*c).envelope,
        &raw mut (*c).trajectory_calibration,
    );
    trajectory_copy(&raw mut plan.trajectory, &raw mut prediction.trajectory);
    snprintf(
        &raw mut plan.note as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        if simulator_fast_qualification as ::core::ffi::c_int != 0 {
            b"Offline downstream-controller diagnostic; no shadow qualification was performed.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else if warning_continuation as ::core::ffi::c_int != 0 {
            b"Live warning continuation; production shadow rejected this state; qualification remains false.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"Reentry continuation qualified by production shadow executive from live state.\0"
                as *const u8 as *const ::core::ffi::c_char
        },
    );
    if shadow_qualified {
        prediction_cache_store(c, &raw mut prediction);
        (*c).cached_prediction_ut = t.ut;
        (*c).cached_prediction_sign = initial_s_turn_sign;
        prediction_cache_apply(c, &raw mut t, &raw mut cfg);
    } else {
        (*c).has_prediction_cache = false_0 != 0;
        t.predicted_miss_distance = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_taem_distance = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_taem_range_error = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_entry_range = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_entry_flight_path_angle = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_taem_speed = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_taem_energy_error = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_peak_dynamic_pressure = ::core::f32::NAN as ::core::ffi::c_double;
        t.predicted_peak_g_load = ::core::f32::NAN as ::core::ffi::c_double;
    }
    entry_prediction_clear(&raw mut prediction);
    if (*c).has_plan {
        deorbit_plan_clear(&raw mut (*c).plan);
    }
    (*c).plan = plan;
    (*c).has_plan = true_0 != 0;
    (*c).plan_stale = false_0 != 0;
    (*c).time_warp_issued = true_0 != 0;
    (*c).log_plan_trajectory_dirty = true_0 != 0;
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    trajectory_copy(&raw mut (*c).dynamic_trajectory, &raw mut (*c).plan.trajectory);
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    trajectory_copy(&raw mut (*c).stabilized_trajectory, &raw mut (*c).plan.trajectory);
    trajectory_calibrator_set_forecast(
        &raw mut (*c).trajectory_calibrator,
        &raw mut (*c).plan.trajectory,
    );
    (*c).latest_telemetry = t;
    (*c).latest_state = state;
    (*c).has_latest = true_0 != 0;
    landing_snapshot_set_sample(&raw mut (*c).snapshot, &raw mut t, &raw mut state);
    (*c).last_prediction_ut = -::core::f32::INFINITY as ::core::ffi::c_double;
    t.predicted_miss_distance = (*c).plan.predicted_closest_distance;
    t.predicted_taem_distance = (*c).plan.predicted_taem_distance;
    t.predicted_taem_range_error = (*c).plan.predicted_taem_range_error;
    t.predicted_entry_range = (*c).plan.predicted_entry_range;
    t.predicted_entry_flight_path_angle = (*c).plan.predicted_entry_flight_path_angle;
    landing_snapshot_set_sample(
        &raw mut (*c).snapshot,
        &raw mut t,
        &raw mut (*c).latest_state,
    );
    (*c).snapshot.phase = (*c).guidance.phase;
    (*c).snapshot.automation_engaged = true_0 != 0;
    (*c).snapshot.paused = false_0 != 0;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        if simulator_fast_qualification as ::core::ffi::c_int != 0 {
            b"Offline downstream-controller diagnostic armed without shadow qualification.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else if warning_continuation as ::core::ffi::c_int != 0 {
            b"Live reentry continuation armed under warning without shadow qualification.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else if in_atmosphere as ::core::ffi::c_int != 0 {
            b"Reentry-only guidance resumed from live atmospheric state after shadow qualification.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"Reentry-only continuation armed from live post-deorbit state after shadow qualification.\0"
                as *const u8 as *const ::core::ffi::c_char
        },
    );
    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
    set_warning(
        c,
        if simulator_fast_qualification as ::core::ffi::c_int != 0 {
            b"This offline diagnostic skips shadow admission and cannot demonstrate qualification.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            (if warning_continuation as ::core::ffi::c_int != 0 {
                &raw mut continuation_warning as *mut ::core::ffi::c_char
            } else {
                ::core::ptr::null_mut::<::core::ffi::c_char>()
            }) as *const ::core::ffi::c_char
        },
    );
    reference_trajectory(
        &raw mut (*c).snapshot.reference_trajectory,
        &raw mut cfg.site,
        &raw mut cfg.guidance,
        (*p).radius,
        initial_s_turn_sign,
    );
    (*c).log_reference_trajectory_dirty = true_0 != 0;
    log_state_event(
        c,
        if simulator_fast_qualification as ::core::ffi::c_int != 0 {
            b"event=offline-reentry-diagnostic-armed; simulator-fast-qualification=true; shadow-qualified=false\0"
                as *const u8 as *const ::core::ffi::c_char
        } else if warning_continuation as ::core::ffi::c_int != 0 {
            b"event=reentry-warning-continuation-armed; shadow-qualified=false; shadow-rejected=true; authorized-warning-policy=true\0"
                as *const u8 as *const ::core::ffi::c_char
        } else if in_atmosphere as ::core::ffi::c_int != 0 {
            b"event=reentry-continuation-armed; atmospheric-restart=true; shadow-qualified=true\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"event=reentry-continuation-armed; deorbit-planner-skipped=true; shadow-qualified=true\0"
                as *const u8 as *const ::core::ffi::c_char
        },
        true_0 != 0,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_engage_hac_test(
    mut c: *mut LandingController,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    if (*c).session.is_null() {
        set_warning(
            c,
            b"Connect to KSP before starting the HAC-only landing test.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).glide.active {
        set_warning(
            c,
            b"Stop calibration before starting the HAC-only landing test.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut err: [::core::ffi::c_char; 768] = [
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
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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
    let mut state: VehicleState = VehicleState {
        ut: 0.,
        position: Vector3 { x: 0., y: 0., z: 0. },
        velocity: Vector3 { x: 0., y: 0., z: 0. },
        mass: 0.,
    };
    if !read_live_sample(
        c,
        &raw mut cfg,
        &raw mut t,
        &raw mut state,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
    ) {
        set_error(c, &raw mut err as *mut ::core::ffi::c_char);
        (*c).snapshot.phase = PHASE_FAULT;
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    if planet_atmospheric_density(p, t.mean_altitude) <= DBL_EPSILON {
        set_warning(
            c,
            b"HAC-only landing test requires an atmospheric flight state.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if strcasecmp(
        &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0
        || strcasecmp(
            &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
            b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        || strcasecmp(
            &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
            b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        set_warning(
            c,
            b"HAC-only landing test requires an airborne vehicle.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut course: ::core::ffi::c_double = surface_course(
        state.position,
        state.velocity,
        planet_rotation_vector(p),
        (*p).north_axis,
        t.heading,
    );
    let mut status: [::core::ffi::c_char; 384] = [
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
    ];
    if !guidance_begin_hac_test(
        &raw mut (*c).guidance,
        &raw mut t,
        course,
        p,
        (*c).aerodynamics,
        &raw mut cfg,
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
    ) {
        set_warning(
            c,
            if status[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
                &raw mut status as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
            } else {
                b"The current state cannot initialize a HAC-only landing test.\0"
                    as *const u8 as *const ::core::ffi::c_char
            },
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).has_plan {
        deorbit_plan_clear(&raw mut (*c).plan);
        (*c).has_plan = false_0 != 0;
    }
    (*c).plan_stale = false_0 != 0;
    (*c).time_warp_issued = true_0 != 0;
    (*c).has_prediction_cache = false_0 != 0;
    (*c).last_prediction_ut = -1e300f64;
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    trajectory_init(&raw mut (*c).dynamic_trajectory);
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    trajectory_init(&raw mut (*c).stabilized_trajectory);
    trajectory_clear(&raw mut (*c).trajectory_calibrator.forecast);
    trajectory_init(&raw mut (*c).trajectory_calibrator.forecast);
    (*c).latest_telemetry = t;
    (*c).latest_state = state;
    (*c).has_latest = true_0 != 0;
    landing_snapshot_set_sample(
        &raw mut (*c).snapshot,
        &raw mut t,
        &raw mut (*c).latest_state,
    );
    (*c).snapshot.plan = ::core::ptr::null_mut::<DeorbitPlan>();
    (*c).snapshot.phase = PHASE_TAEM;
    (*c).snapshot.automation_engaged = true_0 != 0;
    (*c).snapshot.paused = false_0 != 0;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"%s MM305 terminal guidance owns the vehicle; path acquisition remains uncommitted until the live geometry, energy, and authority gates pass.\0"
            as *const u8 as *const ::core::ffi::c_char,
        &raw mut status as *mut ::core::ffi::c_char,
    );
    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
    set_warning(
        c,
        b"HAC/MM305 test mode is active. This is a terminal-guidance test, not a validated reentry continuation.\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    trajectory_clear(&raw mut (*c).snapshot.reference_trajectory);
    (*c).log_reference_trajectory_dirty = true_0 != 0;
    log_state_event(
        c,
        b"event=hac-only-test-armed; deorbit-skipped=true; entry-skipped=true; mm305-energy-staging=true\0"
            as *const u8 as *const ::core::ffi::c_char,
        true_0 != 0,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_engage_final_test(
    mut c: *mut LandingController,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    if (*c).session.is_null() {
        set_warning(
            c,
            b"Connect to ShuttleSim before starting the final-only landing test.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if !krpc_session_is_simulator((*c).session)
        && getenv(
                b"KSP_LANDER_FINAL_TEST_LIVE\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        || getenv(
                b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                    as *const ::core::ffi::c_char,
            )
            .is_null()
        || strcmp(
            getenv(
                b"KSP_LANDER_FINAL_APPROACH_TEST\0" as *const u8
                    as *const ::core::ffi::c_char,
            ),
            b"1\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0
    {
        set_warning(
            c,
            b"Final-only landing test is simulator-only and requires KSP_LANDER_FINAL_APPROACH_TEST=1.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).glide.active {
        set_warning(
            c,
            b"Stop calibration before starting the final-only landing test.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut err: [::core::ffi::c_char; 768] = [
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
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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
    let mut state: VehicleState = VehicleState {
        ut: 0.,
        position: Vector3 { x: 0., y: 0., z: 0. },
        velocity: Vector3 { x: 0., y: 0., z: 0. },
        mass: 0.,
    };
    if !read_live_sample(
        c,
        &raw mut cfg,
        &raw mut t,
        &raw mut state,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
    ) {
        set_error(c, &raw mut err as *mut ::core::ffi::c_char);
        (*c).snapshot.phase = PHASE_FAULT;
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    (*c).aerodynamics = envelope_at_controller(&raw mut (*c).envelope, t.mach);
    t.estimated_lift_to_drag = (*c).aerodynamics.lift_to_drag;
    t.estimated_ballistic_coefficient = (*c).aerodynamics.ballistic_coefficient;
    t.aerodynamic_confidence = (*c).aerodynamics.confidence;
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    if planet_atmospheric_density(p, t.mean_altitude) <= DBL_EPSILON {
        set_warning(
            c,
            b"Final-only landing test requires an atmospheric flight state.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if strcasecmp(
        &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0
        || strcasecmp(
            &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
            b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        || strcasecmp(
            &raw mut t.vessel_situation as *mut ::core::ffi::c_char,
            b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        set_warning(
            c,
            b"Final-only landing test requires an airborne vehicle.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut course: ::core::ffi::c_double = surface_course(
        state.position,
        state.velocity,
        planet_rotation_vector(p),
        (*p).north_axis,
        t.heading,
    );
    let mut status: [::core::ffi::c_char; 512] = [
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
    if !guidance_begin_final_test(
        &raw mut (*c).guidance,
        &raw mut t,
        course,
        p,
        (*c).aerodynamics,
        &raw mut cfg,
        &raw mut status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
    ) {
        set_warning(
            c,
            if status[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
                &raw mut status as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
            } else {
                b"The recorded state cannot initialize the final-only landing test.\0"
                    as *const u8 as *const ::core::ffi::c_char
            },
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).has_plan {
        deorbit_plan_clear(&raw mut (*c).plan);
        (*c).has_plan = false_0 != 0;
    }
    (*c).plan_stale = false_0 != 0;
    (*c).time_warp_issued = true_0 != 0;
    (*c).has_prediction_cache = false_0 != 0;
    (*c).last_prediction_ut = -1e300f64;
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    trajectory_init(&raw mut (*c).dynamic_trajectory);
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    trajectory_init(&raw mut (*c).stabilized_trajectory);
    trajectory_clear(&raw mut (*c).trajectory_calibrator.forecast);
    trajectory_init(&raw mut (*c).trajectory_calibrator.forecast);
    (*c).latest_telemetry = t;
    (*c).latest_state = state;
    (*c).has_latest = true_0 != 0;
    landing_snapshot_set_sample(
        &raw mut (*c).snapshot,
        &raw mut t,
        &raw mut (*c).latest_state,
    );
    (*c).snapshot.plan = ::core::ptr::null_mut::<DeorbitPlan>();
    (*c).snapshot.phase = PHASE_TAEM;
    (*c).snapshot.automation_engaged = true_0 != 0;
    (*c).snapshot.paused = false_0 != 0;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut status as *mut ::core::ffi::c_char,
    );
    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
    set_warning(c, ::core::ptr::null::<::core::ffi::c_char>());
    trajectory_clear(&raw mut (*c).snapshot.reference_trajectory);
    (*c).log_reference_trajectory_dirty = true_0 != 0;
    log_state_event(
        c,
        b"event=final-only-test-armed; post-hac=true; hac-planning-skipped=true; entry-skipped=true; s-turn-skipped=true\0"
            as *const u8 as *const ::core::ffi::c_char,
        true_0 != 0,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_start_calibration(
    mut c: *mut LandingController,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if (*c).session.is_null() {
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if (*c).guidance.automation_engaged {
        set_warning(
            c,
            b"Pause or abort landing guidance before starting calibration.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    if !(*c).has_latest {
        set_warning(
            c,
            b"Wait for the first telemetry sample before starting calibration.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    let mut t: *mut Telemetry = &raw mut (*c).latest_telemetry;
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut v: *mut VehicleProfile = &raw mut cfg.vehicle;
    let mut cal: *mut CalibrationSettings = &raw mut cfg.calibration;
    let mut warning: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    if strcasecmp(
        &raw mut (*t).vessel_situation as *mut ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0
        || strcasecmp(
            &raw mut (*t).vessel_situation as *mut ::core::ffi::c_char,
            b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
        || strcasecmp(
            &raw mut (*t).vessel_situation as *mut ::core::ffi::c_char,
            b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0
    {
        warning = b"Calibration mode requires an airborne craft.\0" as *const u8
            as *const ::core::ffi::c_char;
    } else if (*t).radar_altitude < (*cal).minimum_calibration_radar_altitude {
        warning = b"Current radar altitude is below the configured calibration envelope.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if (*t).true_air_speed < (*v).minimum_safe_speed {
        warning = b"Current airspeed is below the vehicle minimum-safe-speed model.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if (*t).dynamic_pressure < (*cal).minimum_dynamic_pressure {
        warning = b"Dynamic pressure is below the configured calibration measurement domain.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if (*t).dynamic_pressure > (*cal).maximum_calibration_dynamic_pressure {
        warning = b"Dynamic pressure exceeds the configured calibration safety domain.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if (*t).g_force > (*cal).maximum_calibration_g_load {
        warning = b"Load factor exceeds the configured calibration safety domain.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if -(*t).vertical_speed > (*cal).maximum_calibration_sink_rate {
        warning = b"Sink rate exceeds the configured calibration safety domain.\0"
            as *const u8 as *const ::core::ffi::c_char;
    } else if (*t).angle_of_attack < (*cal).minimum_angle_of_attack
        || (*t).angle_of_attack > (*cal).maximum_angle_of_attack
    {
        warning = b"Current angle of attack is outside the configured calibration sweep domain.\0"
            as *const u8 as *const ::core::ffi::c_char;
    }
    if !warning.is_null() {
        set_warning(c, warning);
        publish(c);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return;
    }
    guidance_set_engaged(&raw mut (*c).guidance, false_0 != 0);
    glide_calibration_start(&raw mut (*c).glide, t, cal);
    (*c).snapshot.phase = PHASE_CALIBRATION;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Starting wings-level glide calibration inside the configured measurement envelope.\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    set_warning(c, ::core::ptr::null::<::core::ffi::c_char>());
    (*c).snapshot.calibration.active = true_0 != 0;
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_stop_calibration(
    mut c: *mut LandingController,
    mut apply: bool,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    glide_calibration_stop(
        &raw mut (*c).glide,
        if apply as ::core::ffi::c_int != 0 {
            b"Flight-test calibration stopped and the reviewed profile was explicitly applied.\0"
                as *const u8 as *const ::core::ffi::c_char
        } else {
            b"Flight-test calibration stopped. The shadow profile remains available for review.\0"
                as *const u8 as *const ::core::ffi::c_char
        },
    );
    if !(*c).session.is_null() {
        krpc_safe((*c).session);
    }
    if apply {
        (*c).configuration.vehicle = (*c).adaptive_profile;
        apply_certified_aero_prior(c);
        if (*c).has_plan {
            (*c).plan_stale = true_0 != 0;
            set_warning(
                c,
                b"The explicitly applied flight-test profile changed the vehicle model. Create a new deorbit plan before engagement.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
    }
    (*c).snapshot.phase = PHASE_IDLE;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        if apply as ::core::ffi::c_int != 0 {
            b"Reviewed calibration profile applied; replan required.\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"Flight-test calibration stopped. Certified flight model unchanged.\0"
                as *const u8 as *const ::core::ffi::c_char
        },
    );
    (*c).snapshot.calibration.active = false_0 != 0;
    (*c).snapshot.calibration.state = CAL_STOPPED;
    (*c).snapshot.adaptive_vehicle_profile = (*c).adaptive_profile;
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_reset_calibration(
    mut c: *mut LandingController,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    glide_calibration_stop(
        &raw mut (*c).glide,
        b"Live flight-test reconstruction reset.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    if !(*c).session.is_null() {
        krpc_safe((*c).session);
    }
    reset_models(c, &raw mut (*c).configuration.vehicle);
    if !(*c).session.is_null() {
        krpc_session_seed_physics((*c).session, &raw mut (*c).physics);
        apply_certified_aero_prior(c);
    }
    calibration_snapshot_init(&raw mut (*c).snapshot.calibration);
    snprintf(
        &raw mut (*c).snapshot.calibration.status as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"Shadow flight-test calibration reset. The stored prior-flight aerodynamic data book remains active.\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    (*c).snapshot.phase = PHASE_IDLE;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Shadow calibration reset; certified vessel prior restored.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_set_paused(
    mut c: *mut LandingController,
    mut p: bool,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    guidance_set_paused(&raw mut (*c).guidance, p);
    (*c).snapshot.paused = p;
    if p as ::core::ffi::c_int != 0 && !(*c).session.is_null() {
        krpc_safe((*c).session);
    }
    log_state_event(
        c,
        if p as ::core::ffi::c_int != 0 {
            b"event=operator-pause; safe_control_release=true\0" as *const u8
                as *const ::core::ffi::c_char
        } else {
            b"event=operator-resume\0" as *const u8 as *const ::core::ffi::c_char
        },
        true_0 != 0,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_abort(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).prediction_generation = (*c).prediction_generation.wrapping_add(1);
    guidance_abort(&raw mut (*c).guidance);
    glide_calibration_stop(
        &raw mut (*c).glide,
        b"Calibration aborted. Manual control restored.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    if !(*c).session.is_null() {
        krpc_safe((*c).session);
    }
    (*c).snapshot.automation_engaged = false_0 != 0;
    (*c).snapshot.paused = false_0 != 0;
    (*c).snapshot.calibration.active = false_0 != 0;
    (*c).snapshot.phase = PHASE_ABORT;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Automation aborted. Manual control restored.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    log_state_event(
        c,
        b"event=operator-abort; safe_control_release=true\0" as *const u8
            as *const ::core::ffi::c_char,
        true_0 != 0,
    );
    publish(c);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_set_gear(
    mut c: *mut LandingController,
    mut v: bool,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if !(*c).session.is_null() {
        let mut e: [::core::ffi::c_char; 512] = [0; 512];
        if !krpc_set_gear(
            (*c).session,
            v,
            &raw mut e as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        ) {
            set_warning(c, &raw mut e as *mut ::core::ffi::c_char);
            publish(c);
        }
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_set_brakes(
    mut c: *mut LandingController,
    mut v: bool,
) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if !(*c).session.is_null() {
        let mut e: [::core::ffi::c_char; 512] = [0; 512];
        if !krpc_set_brakes(
            (*c).session,
            v,
            &raw mut e as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        ) {
            set_warning(c, &raw mut e as *mut ::core::ffi::c_char);
            publish(c);
        }
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_save_checkpoint(
    mut c: *mut LandingController,
    mut name: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut ok: bool = false_0 != 0;
    pthread_mutex_lock(&raw mut (*c).mutex);
    if (*c).session.is_null() {
        snprintf(
            error,
            error_size,
            b"kRPC is not connected\0" as *const u8 as *const ::core::ffi::c_char,
        );
    } else {
        ok = krpc_save_game((*c).session, name, error, error_size);
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    return ok;
}
#[no_mangle]
pub unsafe extern "C" fn landing_controller_shutdown(mut c: *mut LandingController) {
    if c.is_null() {
        return;
    }
    stop_thread(c);
    pthread_mutex_lock(&raw mut (*c).mutex);
    disconnect_locked(c, false_0 != 0);
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
unsafe extern "C" fn entry_policy_handoff_geometry(
    mut f: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut out: *mut TaemGeometryState,
) -> bool {
    if f.is_null() || p.is_null() || cfg.is_null() || out.is_null() {
        return false_0 != 0;
    }
    let mut site: GeoPoint = GeoPoint {
        latitude: (*cfg).site.latitude,
        longitude: (*cfg).site.longitude,
        altitude: (*cfg).site.altitude,
    };
    let mut mach_handoff: ::core::ffi::c_double = (*cfg).guidance.mm305_target_mach
        + (if ((*cfg).guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
            || (*cfg).guidance.taem_hac_law as ::core::ffi::c_int != 0)
            && strcmp(
                &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
                b"STS\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
        {
            0.0f64
        } else {
            (*cfg).guidance.mm305_mach_half_width
        });
    let mut prev_excess: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut i: size_t = 0 as size_t;
    while i < (*f).trajectory.count {
        let mut pt: *const TrajectoryPoint = (*f).trajectory.points.offset(i as isize)
            as *mut TrajectoryPoint;
        let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
            p,
            (*pt).altitude,
        );
        if sound > 0.0f64 {
            let mut excess: ::core::ffi::c_double = (*pt).speed - mach_handoff * sound;
            if i > 0 as size_t && excess <= 0.0f64
                && (*pt).altitude <= (*cfg).guidance.mm305_max_altitude
            {
                let mut q: *const TrajectoryPoint = (*f)
                    .trajectory
                    .points
                    .offset(i.wrapping_sub(1 as size_t) as isize)
                    as *mut TrajectoryPoint;
                let mut k: ::core::ffi::c_double = if prev_excess.is_finite() as i32 != 0
                    && prev_excess > 0.0f64
                {
                    prev_excess / (prev_excess - excess)
                } else {
                    1.0f64
                };
                let mut a: GeoPoint = GeoPoint {
                    latitude: (*q).latitude,
                    longitude: (*q).longitude,
                    altitude: (*q).altitude,
                };
                let mut b: GeoPoint = GeoPoint {
                    latitude: (*pt).latitude,
                    longitude: (*pt).longitude,
                    altitude: (*pt).altitude,
                };
                let mut ground: ::core::ffi::c_double = great_circle_distance(
                    a,
                    b,
                    (*p).radius,
                );
                if !(ground > 1.0f64) {
                    return false_0 != 0;
                }
                let mut course: ::core::ffi::c_double = initial_bearing(a, b);
                let mut at: GeoPoint = destination_point(
                    a,
                    course,
                    k * ground,
                    (*p).radius,
                );
                at.altitude = (*q).altitude + k * ((*pt).altitude - (*q).altitude);
                let mut speed: ::core::ffi::c_double = (*q).speed
                    + k * ((*pt).speed - (*q).speed);
                let mut fpa: ::core::ffi::c_double = atan2(
                    (*pt).altitude - (*q).altitude,
                    ground,
                );
                let mut along: ::core::ffi::c_double = 0.;
                let mut cross: ::core::ffi::c_double = 0.;
                runway_coordinates(
                    at,
                    site,
                    (*cfg).site.runway_heading,
                    (*p).radius,
                    &raw mut along,
                    &raw mut cross,
                );
                *out = TaemGeometryState {
                    runway_along_m: along,
                    runway_cross_m: cross,
                    altitude_above_runway_m: at.altitude - (*cfg).site.altitude,
                    ground_speed_mps: speed * cos(fpa),
                    airspeed_mps: speed,
                    course_deg: course,
                    flight_path_angle_deg: fpa * RAD2DEG,
                    heading_error_deg: norm_signed_deg(
                        course - (*cfg).site.runway_heading,
                    ),
                    curvature_right_per_m: 0.,
                };
                return true_0 != 0;
            }
            prev_excess = excess;
        }
        i = i.wrapping_add(1);
    }
    return false_0 != 0;
}
unsafe extern "C" fn entry_policy_handoff_miss(
    mut f: *const EntryPrediction,
    mut p: *const PlanetModel,
    mut cfg: *const LandingConfiguration,
    mut g: *const GuidanceMachine,
    mut model: *const TerminalModel,
    mut mass: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut geo: TaemGeometryState = TaemGeometryState {
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
    let mut error: ::core::ffi::c_double = 0.;
    let mut band: ::core::ffi::c_double = 0.;
    if g.is_null() || model.is_null()
        || !entry_policy_handoff_geometry(f, p, cfg, &raw mut geo)
        || !mm305_hac_handoff_energy_error(
            model,
            &raw const (*g).taem_hac_live_aero,
            &raw mut geo,
            mass,
            &raw mut error,
            &raw mut band,
        )
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return band - error;
}
unsafe extern "C" fn entry_supervision_horizon(
    mut settings: *const GuidanceSettings,
) -> ::core::ffi::c_double {
    if settings.is_null() || (*settings).prediction_interval.is_finite() as i32 == 0
        || (*settings).prediction_interval <= 0.0f64
    {
        return 0.0f64;
    }
    return (*settings).prediction_interval;
}
unsafe extern "C" fn terminal_prediction_identity(
    mut g: *const GuidanceMachine,
) -> AsyncPredictionIdentity {
    let mut id: AsyncPredictionIdentity = AsyncPredictionIdentity {
        guidance_phase: 0 as ::core::ffi::c_int,
        taem_phase: 0,
        taem_transition_count: 0,
        control_plan_sequence: 0,
        plan_id: 0,
        plan_version: 0,
        terminal_path_kind: 0,
        terminal_path_committed: false,
        hac_side_selected: false,
        hac_captured: false,
        final_approach_captured: false,
    };
    if g.is_null() {
        return id;
    }
    id.guidance_phase = (*g).phase as ::core::ffi::c_int;
    id.taem_phase = (*g).taem_exec.phase as ::core::ffi::c_int;
    id.taem_transition_count = (*g).taem_exec.transition_count;
    id.control_plan_sequence = (*g).control_plan_sequence;
    id.terminal_path_kind = (*g).terminal_path_kind as ::core::ffi::c_int;
    id.terminal_path_committed = (*g).terminal_path_committed;
    id.hac_side_selected = (*g).hac_side_selected;
    id.hac_captured = (*g).hac_captured;
    id.final_approach_captured = (*g).final_approach_captured;
    return id;
}
unsafe extern "C" fn terminal_prediction_commit(
    mut c: *mut LandingController,
    mut pr: *mut EntryPrediction,
    mut request_ut: ::core::ffi::c_double,
    mut sign: ::core::ffi::c_double,
) {
    if c.is_null() || pr.is_null() || !(*pr).shadow_guidance_used {
        return;
    }
    prediction_cache_store(c, pr);
    (*c).cached_prediction_ut = request_ut;
    (*c).cached_prediction_sign = sign;
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    (*c).dynamic_trajectory = (*pr).trajectory;
    (*pr).trajectory = Trajectory {
        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
        count: 0,
        capacity: 0,
    };
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    if !trajectory_copy(
        &raw mut (*c).stabilized_trajectory,
        &raw mut (*c).dynamic_trajectory,
    ) {
        trajectory_init(&raw mut (*c).stabilized_trajectory);
    }
    trajectory_calibrator_set_forecast(
        &raw mut (*c).trajectory_calibrator,
        &raw mut (*c).dynamic_trajectory,
    );
    (*c).cached_predicted_raw_published_position_delta_30s = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*c).cached_predicted_raw_published_position_delta_60s = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*c).cached_predicted_raw_published_position_delta_120s = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    (*c).log_raw_plan_trajectory_dirty = true_0 != 0;
    if (*c).has_plan {
        (*c).plan.predicted_closest_distance = (*c).cached_predicted_miss_distance;
        (*c).plan.predicted_taem_distance = (*c).cached_predicted_taem_distance;
        (*c).plan.predicted_taem_range_error = (*c).cached_predicted_taem_range_error;
        (*c).plan.predicted_entry_range = (*c).cached_predicted_entry_range;
        (*c).plan.predicted_entry_flight_path_angle = (*c)
            .cached_predicted_entry_flight_path_angle;
        if !trajectory_equal(
            &raw mut (*c).plan.trajectory,
            &raw mut (*c).stabilized_trajectory,
        ) {
            trajectory_copy(
                &raw mut (*c).plan.trajectory,
                &raw mut (*c).stabilized_trajectory,
            );
            (*c).log_plan_trajectory_dirty = true_0 != 0;
        }
    }
}
unsafe extern "C" fn mm305_admission_job(mut c: *mut LandingController) -> bool {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if (*c).stop_prediction_thread as ::core::ffi::c_int != 0 || (*c).session.is_null()
        || !(*c).has_latest || !(*c).terminal_model_valid
        || !(*c).guidance.automation_engaged
        || (*c).guidance.paused as ::core::ffi::c_int != 0
        || (*c).guidance.aborted as ::core::ffi::c_int != 0
        || (*c).guidance.phase as ::core::ffi::c_uint
            != PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        || !(*c).guidance.mm305_async_planning || !(*c).guidance.mm305_admission_needed
    {
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return false_0 != 0;
    }
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut model: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    if model.is_null() {
        (*c).guidance.mm305_admission_needed = false_0 != 0;
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return true_0 != 0;
    }
    *model = (*c).terminal_model;
    let mut request: Mm305PlanRequest = guidance_mm305_admission_request(
        &raw mut (*c).guidance,
        &raw mut (*c).latest_telemetry,
        &raw mut (*c).latest_state,
        &raw mut cfg,
        model,
    );
    let mut token: ::core::ffi::c_double = (*c).guidance.mm305_admission_request_ut;
    let mut generation: uint64_t = (*c).prediction_generation;
    if !request.valid {
        (*c).guidance.mm305_admission_needed = false_0 != 0;
        pthread_mutex_unlock(&raw mut (*c).mutex);
        free(model as *mut ::core::ffi::c_void);
        return true_0 != 0;
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    let mut result: Mm305PlanResult = mm305_plan(model, &raw mut request);
    free(model as *mut ::core::ffi::c_void);
    pthread_mutex_lock(&raw mut (*c).mutex);
    let mut current: bool = !(*c).stop_prediction_thread
        && generation == (*c).prediction_generation
        && (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*c).guidance.mm305_admission_needed as ::core::ffi::c_int != 0
        && (*c).guidance.mm305_admission_request_ut == token
        && (*c).terminal_model_valid as ::core::ffi::c_int != 0
        && (*c).terminal_model.snapshot_id == result.model_snapshot_id;
    if current {
        guidance_mm305_admission_accept(&raw mut (*c).guidance, &raw mut result);
        let mut event: [::core::ffi::c_char; 256] = [0; 256];
        snprintf(
            &raw mut event as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"event=mm305-admission; request-ut=%.3f; found=%d; solve-wall=%.3f; rejections=%u\0"
                as *const u8 as *const ::core::ffi::c_char,
            request.request_ut,
            result.found as ::core::ffi::c_int,
            result.solve_wall_s,
            (*c).guidance.mm305_admission_rejections,
        );
        log_state_event(c, &raw mut event as *mut ::core::ffi::c_char, true_0 != 0);
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    return true_0 != 0;
}
unsafe extern "C" fn mm305_planning_job(mut c: *mut LandingController) -> bool {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if (*c).stop_prediction_thread as ::core::ffi::c_int != 0 || (*c).session.is_null()
        || !(*c).has_latest || !(*c).terminal_model_valid
        || !(*c).guidance.automation_engaged
        || (*c).guidance.paused as ::core::ffi::c_int != 0
        || (*c).guidance.aborted as ::core::ffi::c_int != 0
        || (*c).guidance.phase as ::core::ffi::c_uint
            != PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        || !(*c).guidance.mm305_async_planning || !(*c).guidance.mm305_planning_needed
    {
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return false_0 != 0;
    }
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut model: *mut TerminalModel = malloc(
        ::core::mem::size_of::<TerminalModel>() as size_t,
    ) as *mut TerminalModel;
    if model.is_null() {
        (*c).guidance.mm305_planning_needed = false_0 != 0;
        pthread_mutex_unlock(&raw mut (*c).mutex);
        return true_0 != 0;
    }
    *model = (*c).terminal_model;
    let mut request: Mm305PlanRequest = guidance_mm305_plan_request(
        &raw mut (*c).guidance,
        &raw mut (*c).latest_telemetry,
        &raw mut (*c).latest_state,
        &raw mut cfg,
        model,
    );
    let mut token: ::core::ffi::c_double = (*c).guidance.mm305_plan_request_ut;
    let mut generation: uint64_t = (*c).prediction_generation;
    if !request.valid {
        (*c).guidance.mm305_planning_needed = false_0 != 0;
        (*c).guidance.mm305_plan_failures = (*c)
            .guidance
            .mm305_plan_failures
            .wrapping_add(1);
        pthread_mutex_unlock(&raw mut (*c).mutex);
        free(model as *mut ::core::ffi::c_void);
        return true_0 != 0;
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    let mut result: Mm305PlanResult = mm305_plan(model, &raw mut request);
    pthread_mutex_lock(&raw mut (*c).mutex);
    let mut current: bool = !(*c).stop_prediction_thread
        && generation == (*c).prediction_generation
        && (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*c).guidance.mm305_planning_needed as ::core::ffi::c_int != 0
        && (*c).guidance.mm305_plan_request_ut == token
        && (*c).terminal_model_valid as ::core::ffi::c_int != 0
        && (*c).terminal_model.snapshot_id == result.model_snapshot_id;
    if current as ::core::ffi::c_int != 0 && result.found as ::core::ffi::c_int != 0
        && !mm305_plan_result_fresh(&raw mut result, &raw mut (*c).latest_telemetry)
    {
        cfg = effective_config(c);
        let mut fresh: Mm305PlanRequest = guidance_mm305_plan_request(
            &raw mut (*c).guidance,
            &raw mut (*c).latest_telemetry,
            &raw mut (*c).latest_state,
            &raw mut cfg,
            model,
        );
        pthread_mutex_unlock(&raw mut (*c).mutex);
        result = mm305_revalidate_plan(model, &raw mut fresh, &raw mut result);
        pthread_mutex_lock(&raw mut (*c).mutex);
        current = !(*c).stop_prediction_thread
            && generation == (*c).prediction_generation
            && (*c).guidance.phase as ::core::ffi::c_uint
                == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
            && (*c).guidance.mm305_planning_needed as ::core::ffi::c_int != 0
            && (*c).guidance.mm305_plan_request_ut == token
            && (*c).terminal_model_valid as ::core::ffi::c_int != 0
            && (*c).terminal_model.snapshot_id == result.model_snapshot_id;
    }
    if current {
        let mut stale: bool = result.found as ::core::ffi::c_int != 0
            && !mm305_plan_result_fresh(&raw mut result, &raw mut (*c).latest_telemetry);
        let mut adopted: bool = false_0 != 0;
        if stale {
            (*c).guidance.mm305_planning_needed = false_0 != 0;
            (*c).guidance.mm305_plan_failures = (*c)
                .guidance
                .mm305_plan_failures
                .wrapping_add(1);
            snprintf(
                &raw mut result.diagnostic as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
                b"MM305 route rejected: current-state qualification expired before adoption\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        } else {
            adopted = guidance_mm305_accept_plan(
                &raw mut (*c).guidance,
                &raw mut result,
                &raw mut cfg,
            );
        }
        let mut event: [::core::ffi::c_char; 256] = [0; 256];
        snprintf(
            &raw mut event as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            b"event=mm305-plan; request-ut=%.3f; found=%d; adopted=%d; solve-wall=%.3f; replans=%u\0"
                as *const u8 as *const ::core::ffi::c_char,
            result.request_ut,
            result.found as ::core::ffi::c_int,
            adopted as ::core::ffi::c_int,
            result.solve_wall_s,
            (*c).guidance.mm305_replans,
        );
        log_state_event(c, &raw mut event as *mut ::core::ffi::c_char, true_0 != 0);
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    free(model as *mut ::core::ffi::c_void);
    return true_0 != 0;
}
unsafe extern "C" fn mm305_planner_worker(
    mut arg: *mut ::core::ffi::c_void,
) -> *mut ::core::ffi::c_void {
    let mut c: *mut LandingController = arg as *mut LandingController;
    loop {
        pthread_mutex_lock(&raw mut (*c).mutex);
        let mut stop: bool = (*c).stop_prediction_thread;
        pthread_mutex_unlock(&raw mut (*c).mutex);
        if stop {
            break;
        }
        if !mm305_planning_job(c) && !mm305_admission_job(c) {
            sleep_seconds(0.005f64);
        }
    }
    return NULL;
}
unsafe extern "C" fn forecast_observables(
    mut s: *const VehicleState,
    mut speed: *mut ::core::ffi::c_double,
    mut radial: *mut ::core::ffi::c_double,
) {
    *speed = vmag((*s).velocity);
    *radial = vdot(
        (*s).velocity,
        vnorm(
            (*s).position,
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        ),
    );
}
unsafe extern "C" fn forecast_fit_scales(
    mut h: *const ForecastHistorySample,
    mut n: ::core::ffi::c_int,
    mut now: *const VehicleState,
    mut planet: *const PlanetModel,
    mut aero: AerodynamicModel,
    mut envelope: *const AerodynamicEnvelope,
    mut calibration: *const TrajectoryCalibrationModel,
    mut cfg: *const LandingConfiguration,
    mut drag_scale: *mut ::core::ffi::c_double,
    mut lift_scale: *mut ::core::ffi::c_double,
) {
    if n < 2 as ::core::ffi::c_int {
        return;
    }
    let mut j: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
    let mut i: ::core::ffi::c_int = n - 1 as ::core::ffi::c_int;
    while i >= 0 as ::core::ffi::c_int {
        if (*now).ut - (*h.offset(i as isize)).state.ut >= FORECAST_FIT_WINDOW_S {
            j = i;
            break;
        } else {
            i -= 1;
        }
    }
    if j < 0 as ::core::ffi::c_int {
        return;
    }
    let mut bank: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut aoa: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut side: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut m: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut i_0: ::core::ffi::c_int = j;
    while i_0 < n {
        if !((*h.offset(i_0 as isize)).q >= FORECAST_FIT_MIN_Q_PA) {
            return;
        }
        if fabs((*h.offset(i_0 as isize)).bank) > 5.0f64 {
            let mut s: ::core::ffi::c_double = if (*h.offset(i_0 as isize)).bank
                >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                1.0f64
            } else {
                -1.0f64
            };
            if side != 0.0f64 && s != side {
                return;
            }
            side = s;
        }
        bank += fabs((*h.offset(i_0 as isize)).bank);
        aoa += (*h.offset(i_0 as isize)).aoa;
        m += 1;
        i_0 += 1;
    }
    bank /= m as ::core::ffi::c_double;
    aoa /= m as ::core::ffi::c_double;
    if side == 0.0f64 {
        side = 1.0f64;
    }
    let mut duration: ::core::ffi::c_double = (*now).ut
        - (*h.offset(j as isize)).state.ut;
    let mut k: [::core::ffi::c_double; 2] = [*drag_scale, *lift_scale];
    let mut pred: [[::core::ffi::c_double; 2]; 3] = [[0.; 2]; 3];
    let mut c: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while c < 3 as ::core::ffi::c_int {
        let mut cal: TrajectoryCalibrationModel = *calibration;
        cal.forecast_drag_scale = k[0 as ::core::ffi::c_int as usize]
            * (if c == 1 as ::core::ffi::c_int { 1.1f64 } else { 1.0f64 });
        cal.forecast_lift_scale = k[1 as ::core::ffi::c_int as usize]
            * (if c == 2 as ::core::ffi::c_int { 1.1f64 } else { 1.0f64 });
        cal.forecast_exact_policy_bank = true_0 != 0;
        let mut f: EntryPrediction = predictor_simulate_entry_policy(
            (*h.offset(j as isize)).state,
            planet,
            aero,
            envelope,
            &raw mut cal,
            &raw const (*cfg).vehicle,
            &raw const (*cfg).site,
            &raw const (*cfg).guidance,
            bank,
            aoa,
            side,
            duration,
        );
        let mut end: VehicleState = f.final_state;
        entry_prediction_clear(&raw mut f);
        if fabs(end.ut - (*now).ut) > 2.0f64 {
            return;
        }
        forecast_observables(
            &raw mut end,
            (&raw mut *(&raw mut pred as *mut [::core::ffi::c_double; 2])
                .offset(c as isize) as *mut ::core::ffi::c_double)
                .offset(0 as ::core::ffi::c_int as isize) as *mut ::core::ffi::c_double,
            (&raw mut *(&raw mut pred as *mut [::core::ffi::c_double; 2])
                .offset(c as isize) as *mut ::core::ffi::c_double)
                .offset(1 as ::core::ffi::c_int as isize) as *mut ::core::ffi::c_double,
        );
        c += 1;
    }
    let mut act: [::core::ffi::c_double; 2] = [0.; 2];
    forecast_observables(
        now,
        (&raw mut act as *mut ::core::ffi::c_double)
            .offset(0 as ::core::ffi::c_int as isize) as *mut ::core::ffi::c_double,
        (&raw mut act as *mut ::core::ffi::c_double)
            .offset(1 as ::core::ffi::c_int as isize) as *mut ::core::ffi::c_double,
    );
    let mut r0: ::core::ffi::c_double = act[0 as ::core::ffi::c_int as usize]
        - pred[0 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize];
    let mut r1: ::core::ffi::c_double = act[1 as ::core::ffi::c_int as usize]
        - pred[0 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize];
    let mut d: ::core::ffi::c_double = log(1.1f64);
    let mut a: ::core::ffi::c_double = (pred[1 as ::core::ffi::c_int
        as usize][0 as ::core::ffi::c_int as usize]
        - pred[0 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize]) / d;
    let mut b: ::core::ffi::c_double = (pred[2 as ::core::ffi::c_int
        as usize][0 as ::core::ffi::c_int as usize]
        - pred[0 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize]) / d;
    let mut cc: ::core::ffi::c_double = (pred[1 as ::core::ffi::c_int
        as usize][1 as ::core::ffi::c_int as usize]
        - pred[0 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize]) / d;
    let mut dd: ::core::ffi::c_double = (pred[2 as ::core::ffi::c_int
        as usize][1 as ::core::ffi::c_int as usize]
        - pred[0 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize]) / d;
    let mut det: ::core::ffi::c_double = a * dd - b * cc;
    if !(fabs(det) > 1e-6f64) {
        return;
    }
    let mut x0: ::core::ffi::c_double = (dd * r0 - b * r1) / det;
    let mut x1: ::core::ffi::c_double = (-cc * r0 + a * r1) / det;
    x0 = clampd(0.5f64 * x0, -0.2f64, 0.2f64);
    x1 = clampd(0.5f64 * x1, -0.2f64, 0.2f64);
    *drag_scale = clampd(
        k[0 as ::core::ffi::c_int as usize] * exp(x0),
        FORECAST_SCALE_MIN,
        FORECAST_SCALE_MAX,
    );
    *lift_scale = clampd(
        k[1 as ::core::ffi::c_int as usize] * exp(x1),
        FORECAST_SCALE_MIN,
        FORECAST_SCALE_MAX,
    );
    if !getenv(
            b"KSP_LANDER_TRACE_FORECAST_FIT\0" as *const u8 as *const ::core::ffi::c_char,
        )
        .is_null()
    {
        fprintf(
            stderr,
            b"FORECAST_FIT ut=%.1f window=%.0fs bank=%.1f aoa=%.1f dv=%+.2f dvr=%+.2f -> drag x%.3f lift x%.3f\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*now).ut,
            duration,
            bank,
            aoa,
            r0,
            r1,
            *drag_scale,
            *lift_scale,
        );
    }
}
unsafe extern "C" fn terminal_prediction_worker(
    mut arg: *mut ::core::ffi::c_void,
) -> *mut ::core::ffi::c_void {
    let mut c: *mut LandingController = arg as *mut LandingController;
    loop {
        let mut state: VehicleState = VehicleState {
            ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        };
        let mut telemetry: Telemetry = Telemetry {
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
        telemetry_init(&raw mut telemetry);
        let mut cfg: LandingConfiguration = LandingConfiguration {
            connection: ConnectionConfiguration {
                serial_port: [
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
                ],
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
        let mut aero: AerodynamicModel = AerodynamicModel {
            lift_to_drag: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ballistic_coefficient: 0.,
            confidence: 0.,
        };
        let mut envelope: AerodynamicEnvelope = AerodynamicEnvelope {
            regimes: [
                AerodynamicModel {
                    lift_to_drag: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    ballistic_coefficient: 0.,
                    confidence: 0.,
                },
                AerodynamicModel {
                    lift_to_drag: 0.,
                    ballistic_coefficient: 0.,
                    confidence: 0.,
                },
                AerodynamicModel {
                    lift_to_drag: 0.,
                    ballistic_coefficient: 0.,
                    confidence: 0.,
                },
                AerodynamicModel {
                    lift_to_drag: 0.,
                    ballistic_coefficient: 0.,
                    confidence: 0.,
                },
            ],
        };
        let mut physics: VesselPhysicsModel = VesselPhysicsModel {
            state: VehicleState {
                ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                position: Vector3 { x: 0., y: 0., z: 0. },
                velocity: Vector3 { x: 0., y: 0., z: 0. },
                mass: 0.,
            },
            center_of_mass: Vector3 { x: 0., y: 0., z: 0. },
            lift_vector: Vector3 { x: 0., y: 0., z: 0. },
            drag_vector: Vector3 { x: 0., y: 0., z: 0. },
            center_of_mass_root: Vector3 { x: 0., y: 0., z: 0. },
            has_center_of_mass_root: false,
            has_state: false,
            has_center_of_mass: false,
            gear: false,
            brakes: false,
            airbrakes: 0,
            thrust: 0.,
            available_thrust: 0.,
            throttle: 0.,
            mass_flow: 0.,
            propellant_per_impulse: 0.,
            authority: [0.; 3],
            authority_confidence: [0.; 3],
            previous_rate: [0.; 3],
            previous_control: [0.; 3],
            torque: [0.; 3],
            inertia: [0.; 3],
            previous_rate_valid: [false; 3],
            authority_q: 0.,
            sideslip: 0.,
            confidence: 0.,
            model_residual: 0.,
            model_residual_confidence: 0.,
            certified_uncertainty: 0.,
            certified_confidence: 0.,
            last_residual_ut: 0.,
            has_last_residual_ut: false,
            force_residual_per_q: Vector3 { x: 0., y: 0., z: 0. },
            force_residual_variance_per_q: Vector3 { x: 0., y: 0., z: 0. },
            force_residual_confidence: 0.,
            dry_mass: 0.,
            count: 0,
            next: 0,
            accepted: 0,
            rejected: 0,
            live_observations: 0,
            samples: [VesselAeroSample {
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
            }; 192],
        };
        let mut calibration: TrajectoryCalibrationModel = TrajectoryCalibrationModel {
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
        };
        let mut terminal_model: TerminalModel = TerminalModel {
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
        let mut terminal_model_valid: bool = false_0 != 0;
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
        let mut entry_request: GuidanceMachine = GuidanceMachine {
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
        let mut entry_job: bool = false_0 != 0;
        let mut entry_planning_job: bool = false_0 != 0;
        let mut generation: uint64_t = 0 as uint64_t;
        let mut plan: DeorbitPlan = DeorbitPlan {
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
            &raw mut plan as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<DeorbitPlan>() as size_t,
        );
        trajectory_init(&raw mut plan.trajectory);
        let mut run: bool = false_0 != 0;
        let mut has_plan: bool = false_0 != 0;
        let mut request_ut: ::core::ffi::c_double = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        let mut sign: ::core::ffi::c_double = 0 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        let mut request_identity: AsyncPredictionIdentity = AsyncPredictionIdentity {
            guidance_phase: 0 as ::core::ffi::c_int,
            taem_phase: 0,
            taem_transition_count: 0,
            control_plan_sequence: 0,
            plan_id: 0,
            plan_version: 0,
            terminal_path_kind: 0,
            terminal_path_committed: false,
            hac_side_selected: false,
            hac_captured: false,
            final_approach_captured: false,
        };
        let mut history: [ForecastHistorySample; 64] = [ForecastHistorySample {
            state: VehicleState {
                ut: 0.,
                position: Vector3 { x: 0., y: 0., z: 0. },
                velocity: Vector3 { x: 0., y: 0., z: 0. },
                mass: 0.,
            },
            aoa: 0.,
            bank: 0.,
            q: 0.,
        }; 64];
        let mut history_count: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        let mut forecast_drag: ::core::ffi::c_double = 1.0f64;
        let mut forecast_lift: ::core::ffi::c_double = 1.0f64;
        pthread_mutex_lock(&raw mut (*c).mutex);
        if (*c).stop_prediction_thread {
            pthread_mutex_unlock(&raw mut (*c).mutex);
            deorbit_plan_clear(&raw mut plan);
            break;
        } else {
            if !(*c).session.is_null() && (*c).has_latest as ::core::ffi::c_int != 0
                && (*c).guidance.automation_engaged as ::core::ffi::c_int != 0
                && !(*c).guidance.paused && !(*c).guidance.aborted
                && (*c).guidance.phase as ::core::ffi::c_uint
                    != PHASE_ATTITUDE_RECOVERY as ::core::ffi::c_int
                        as ::core::ffi::c_uint
                && (taem_exec_owns_vehicle(&raw mut (*c).guidance.taem_exec)
                    as ::core::ffi::c_int != 0
                    || (*c).guidance.phase as ::core::ffi::c_uint
                        == PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                            as ::core::ffi::c_uint
                        && ((*c).guidance.entry_planning_needed as ::core::ffi::c_int
                            != 0
                            || (*c).guidance.entry_final_reversal_pending
                                as ::core::ffi::c_int != 0
                            || (*c).guidance.entry_final_reversal_completed
                                as ::core::ffi::c_int != 0))
            {
                if (*c).latest_telemetry.ut
                    < fmax(
                        (*c).last_terminal_prediction_request_ut,
                        (*c).last_terminal_prediction_completion_ut,
                    ) - 1.0f64
                {
                    (*c).last_terminal_prediction_request_ut = -1e300f64;
                    (*c).last_terminal_prediction_completion_ut = -1e300f64;
                    (*c).last_terminal_prediction_completion_wall = -1e300f64;
                    (*c).last_terminal_prediction_solve_wall = 0.0f64;
                }
                let mut cadence_anchor: ::core::ffi::c_double = fmax(
                    (*c).last_terminal_prediction_request_ut,
                    (*c).last_terminal_prediction_completion_ut,
                );
                let mut now_wall: ::core::ffi::c_double = monotonic_seconds();
                if krpc_session_is_simulator((*c).session) as ::core::ffi::c_int != 0
                    && (*c).last_terminal_prediction_completion_ut > -1e299f64
                {
                    now_wall = (*c).last_terminal_prediction_completion_wall
                        + ((*c).latest_telemetry.ut
                            - (*c).last_terminal_prediction_completion_ut);
                }
                if async_prediction_request_due(
                    (*c).latest_telemetry.ut,
                    cadence_anchor,
                    now_wall,
                    (*c).last_terminal_prediction_completion_wall,
                    (*c).last_terminal_prediction_solve_wall,
                    (*c).configuration.guidance.prediction_interval,
                ) {
                    state = (*c).latest_state;
                    telemetry = (*c).latest_telemetry;
                    history_count = (*c).forecast_history_count;
                    memcpy(
                        &raw mut history as *mut ForecastHistorySample
                            as *mut ::core::ffi::c_void,
                        &raw mut (*c).forecast_history as *mut ForecastHistorySample
                            as *const ::core::ffi::c_void,
                        (history_count as size_t)
                            .wrapping_mul(
                                ::core::mem::size_of::<ForecastHistorySample>() as size_t,
                            ),
                    );
                    if (*c).forecast_drag_scale > 0.0f64 {
                        forecast_drag = (*c).forecast_drag_scale;
                    }
                    if (*c).forecast_lift_scale > 0.0f64 {
                        forecast_lift = (*c).forecast_lift_scale;
                    }
                    cfg = effective_config(c);
                    planet = *krpc_session_planet((*c).session);
                    aero = (*c).aerodynamics;
                    envelope = (*c).envelope;
                    physics = (*c).physics;
                    calibration = (*c).trajectory_calibration;
                    calibration.physics = &raw mut physics;
                    if (*c).terminal_model_valid {
                        terminal_model = (*c).terminal_model;
                        terminal_model_valid = true_0 != 0;
                    }
                    guidance = (*c).guidance;
                    guidance.mm305_async_planning = true_0 != 0;
                    guidance.mm305_planning_needed = false_0 != 0;
                    entry_request = guidance;
                    entry_request.previous_ut = telemetry.ut;
                    entry_job = guidance.phase as ::core::ffi::c_uint
                        == PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                            as ::core::ffi::c_uint;
                    entry_planning_job = entry_job as ::core::ffi::c_int != 0
                        && guidance.entry_planning_needed as ::core::ffi::c_int != 0;
                    generation = (*c).prediction_generation;
                    if guidance.entry_predictor_models_valid {
                        guidance.entry_predictor_calibration.physics = &raw mut physics;
                    }
                    if (*c).has_plan {
                        has_plan = terminal_prediction_plan_copy(
                            &raw mut (*c).plan,
                            &raw mut plan,
                        );
                    }
                    request_identity = terminal_prediction_identity(
                        &raw mut (*c).guidance,
                    );
                    request_ut = telemetry.ut;
                    if fabs((*c).guidance.s_turn_sign) > DBL_EPSILON {
                        sign = (if (*c).guidance.s_turn_sign
                            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
                        {
                            1 as ::core::ffi::c_int
                        } else {
                            -(1 as ::core::ffi::c_int)
                        }) as ::core::ffi::c_double;
                    } else {
                        let mut roll: ::core::ffi::c_double = norm_signed_deg(
                            (*c).last_command.target_roll,
                        );
                        if fabs(roll) > DBL_EPSILON {
                            sign = (if roll
                                >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
                            {
                                1 as ::core::ffi::c_int
                            } else {
                                -(1 as ::core::ffi::c_int)
                            }) as ::core::ffi::c_double;
                        }
                    }
                    (*c).last_terminal_prediction_request_ut = request_ut;
                    run = true_0 != 0;
                }
            }
            pthread_mutex_unlock(&raw mut (*c).mutex);
            if !run {
                deorbit_plan_clear(&raw mut plan);
                sleep_seconds(0.05f64);
            } else {
                let mut solve_started: ::core::ffi::c_double = monotonic_seconds();
                if !getenv(
                        b"KSP_LANDER_ENTRY_WORKER_TRACE\0" as *const u8
                            as *const ::core::ffi::c_char,
                    )
                    .is_null()
                {
                    fprintf(
                        stderr,
                        b"ENTRY_WORKER start ut=%.3f entry=%d planning=%d h=%.1f V=%.1f sequence=%llu\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        request_ut,
                        entry_job as ::core::ffi::c_int,
                        entry_planning_job as ::core::ffi::c_int,
                        telemetry.mean_altitude,
                        telemetry.true_air_speed,
                        entry_request.control_plan_sequence as ::core::ffi::c_ulonglong,
                    );
                }
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
                if entry_job {
                    guidance.entry_planning_deferred = false_0 != 0;
                    guidance.diagnostic_shadow = true_0 != 0;
                    let mut proposal: GuidanceResult = guidance_update(
                        &raw mut guidance,
                        &raw mut telemetry,
                        &raw mut state,
                        if has_plan as ::core::ffi::c_int != 0 {
                            &raw mut plan
                        } else {
                            ::core::ptr::null_mut::<DeorbitPlan>()
                        },
                        &raw mut planet,
                        aero,
                        &raw mut cfg,
                    );
                    guidance_result_clear(&raw mut proposal);
                    let mut proposal_finished: ::core::ffi::c_double = monotonic_seconds();
                    if !getenv(
                            b"KSP_LANDER_ENTRY_WORKER_TRACE\0" as *const u8
                                as *const ::core::ffi::c_char,
                        )
                        .is_null()
                    {
                        fprintf(
                            stderr,
                            b"ENTRY_WORKER stage proposal wall=%.3f\n\0" as *const u8
                                as *const ::core::ffi::c_char,
                            proposal_finished - solve_started,
                        );
                    }
                    guidance.entry_planning_deferred = true_0 != 0;
                    let mut supervision_horizon: ::core::ffi::c_double = entry_supervision_horizon(
                        &raw mut cfg.guidance,
                    );
                    let mut supervision_terminal_model: *const TerminalModel = if terminal_model_valid
                        as ::core::ffi::c_int != 0
                    {
                        &raw mut terminal_model
                    } else {
                        ::core::ptr::null_mut::<TerminalModel>()
                    };
                    pr = predictor_simulate_entry_guidance_shadow_with_terminal_model(
                        state,
                        &raw mut telemetry,
                        &raw mut guidance,
                        if has_plan as ::core::ffi::c_int != 0 {
                            &raw mut plan
                        } else {
                            ::core::ptr::null_mut::<DeorbitPlan>()
                        },
                        &raw mut planet,
                        aero,
                        &raw mut envelope,
                        &raw mut calibration,
                        &raw mut cfg,
                        supervision_terminal_model as *const TerminalModel,
                        supervision_horizon,
                        false_0 != 0,
                    );
                    if !getenv(
                            b"KSP_LANDER_ENTRY_WORKER_TRACE\0" as *const u8
                                as *const ::core::ffi::c_char,
                        )
                        .is_null()
                    {
                        fprintf(
                            stderr,
                            b"ENTRY_WORKER stage supervision wall=%.3f horizon=%.3f terminal=%d\n\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            monotonic_seconds() - proposal_finished,
                            supervision_horizon,
                            if !supervision_terminal_model.is_null() {
                                1 as ::core::ffi::c_int
                            } else {
                                0 as ::core::ffi::c_int
                            },
                        );
                    }
                    guidance.entry_s_turn_plan.terminal_ready = pr.reached_taem
                        as ::core::ffi::c_int != 0
                        && pr.taem_dynamic_interface_captured as ::core::ffi::c_int != 0
                        && pr.shadow_terminal_policy_feasible as ::core::ffi::c_int != 0;
                    guidance.entry_committed_infeasible = pr
                        .taem_ownership_boundary_missed;
                    guidance.entry_supervision_valid = pr.shadow_guidance_used
                        as ::core::ffi::c_int != 0 && !pr.shadow_aborted
                        && pr.peak_dynamic_pressure
                            <= cfg.vehicle.maximum_dynamic_pressure
                        && pr.peak_g_load <= cfg.vehicle.maximum_g_load;
                    guidance.entry_supervision_ut = request_ut;
                    guidance.entry_supervision_boundary_missed = pr
                        .taem_ownership_boundary_missed;
                    guidance.entry_supervision_mode = (if guidance
                        .entry_supervision_valid as ::core::ffi::c_int != 0
                    {
                        ENTRY_SUPERVISION_PASS_THROUGH as ::core::ffi::c_int
                    } else {
                        ENTRY_SUPERVISION_INFEASIBLE as ::core::ffi::c_int
                    }) as EntrySupervisionMode;
                    let mut target: ::core::ffi::c_double = guidance
                        .entry_handoff_target_range;
                    if (cfg.guidance.taem_energy_law as ::core::ffi::c_int != 0
                        || cfg.guidance.taem_hac_law as ::core::ffi::c_int != 0)
                        && target.is_finite() as i32 != 0
                        && telemetry.range_to_site.is_finite() as i32 != 0
                    {
                        forecast_fit_scales(
                            &raw mut history as *mut ForecastHistorySample,
                            history_count,
                            &raw mut state,
                            &raw mut planet,
                            aero,
                            &raw mut envelope,
                            &raw mut calibration,
                            &raw mut cfg,
                            &raw mut forecast_drag,
                            &raw mut forecast_lift,
                        );
                        calibration.forecast_drag_scale = forecast_drag;
                        calibration.forecast_lift_scale = forecast_lift;
                        let mut aoa: ::core::ffi::c_double = if guidance
                            .entry_control_plan_valid as ::core::ffi::c_int != 0
                            && guidance.entry_control_aoa.is_finite() as i32 != 0
                        {
                            guidance.entry_control_aoa
                        } else if guidance.entry_alpha_has_target as ::core::ffi::c_int
                            != 0 && guidance.entry_alpha_target.is_finite() as i32 != 0
                        {
                            guidance.entry_alpha_target
                        } else {
                            telemetry.angle_of_attack
                        };
                        let mut side: ::core::ffi::c_double = if guidance.s_turn_sign
                            >= 0.0f64
                        {
                            1.0f64
                        } else {
                            -1.0f64
                        };
                        let mut hi: ::core::ffi::c_double = fmin(
                            MM304_PLANNED_MAX_BANK_DEG,
                            cfg.vehicle.maximum_bank_angle,
                        );
                        let mut lo: ::core::ffi::c_double = 0.0f64;
                        let mut f_lo: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        let mut f_hi: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        let mut sound_floor: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
                            &raw mut planet,
                            cfg.guidance.mm305_max_altitude,
                        );
                        let mut mach_handoff: ::core::ffi::c_double = cfg
                            .guidance
                            .mm305_target_mach
                            + (if (cfg.guidance.sts_trajectory_delivery
                                as ::core::ffi::c_int != 0
                                || cfg.guidance.taem_hac_law as ::core::ffi::c_int != 0)
                                && strcmp(
                                    &raw mut cfg.vehicle.model_id as *mut ::core::ffi::c_char,
                                    b"STS\0" as *const u8 as *const ::core::ffi::c_char,
                                ) == 0
                            {
                                0.0f64
                            } else {
                                cfg.guidance.mm305_mach_half_width
                            });
                        let mut bound: ::core::ffi::c_double = 2.0f64
                            * telemetry.range_to_site
                            / fmax(1.0f64, mach_handoff * sound_floor);
                        let mut iteration: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                        loop {
                            let mut bank: ::core::ffi::c_double = if iteration
                                == 0 as ::core::ffi::c_int
                            {
                                lo
                            } else if iteration == 1 as ::core::ffi::c_int {
                                hi
                            } else {
                                0.5f64 * (lo + hi)
                            };
                            let mut f: EntryPrediction = predictor_simulate_entry_policy(
                                state,
                                &raw mut planet,
                                aero,
                                &raw mut envelope,
                                &raw mut calibration,
                                &raw mut cfg.vehicle,
                                &raw mut cfg.site,
                                &raw mut cfg.guidance,
                                bank,
                                aoa,
                                side,
                                bound,
                            );
                            let mut miss: ::core::ffi::c_double = ::core::f32::NAN
                                as ::core::ffi::c_double;
                            if cfg.guidance.taem_hac_law {
                                miss = entry_policy_handoff_miss(
                                    &raw mut f,
                                    &raw mut planet,
                                    &raw mut cfg,
                                    &raw mut guidance,
                                    if terminal_model_valid as ::core::ffi::c_int != 0 {
                                        &raw mut terminal_model
                                    } else {
                                        ::core::ptr::null_mut::<TerminalModel>()
                                    },
                                    state.mass,
                                );
                            } else {
                                let mut geo: TaemGeometryState = TaemGeometryState {
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
                                if entry_policy_handoff_geometry(
                                    &raw mut f,
                                    &raw mut planet,
                                    &raw mut cfg,
                                    &raw mut geo,
                                ) {
                                    miss = hypot(geo.runway_along_m, geo.runway_cross_m)
                                        - target;
                                }
                            }
                            entry_prediction_clear(&raw mut f);
                            if iteration == 0 as ::core::ffi::c_int {
                                f_lo = miss;
                                if !(miss < 0.0f64) {
                                    break;
                                }
                            } else if iteration == 1 as ::core::ffi::c_int {
                                f_hi = miss;
                                if !(miss > 0.0f64) {
                                    break;
                                }
                            } else {
                                if miss.is_finite() as i32 == 0 {
                                    break;
                                }
                                if miss > 0.0f64 {
                                    hi = bank;
                                    f_hi = miss;
                                } else {
                                    lo = bank;
                                    f_lo = miss;
                                }
                                if hi - lo
                                    <= cfg.guidance.entry_roll_rate
                                        / fmax(1.0f64, cfg.guidance.guidance_rate)
                                {
                                    break;
                                }
                            }
                            iteration += 1;
                        }
                        let mut solved: ::core::ffi::c_double = if f_lo.is_finite()
                            as i32 != 0 && !(f_lo < 0.0f64)
                        {
                            0.0f64
                        } else if f_hi.is_finite() as i32 != 0 && !(f_hi > 0.0f64) {
                            hi
                        } else {
                            0.5f64 * (lo + hi)
                        };
                        let mut policy_aoa: ::core::ffi::c_double = ::core::f32::NAN
                            as ::core::ffi::c_double;
                        let mut best_miss: ::core::ffi::c_double = f_lo;
                        if f_lo.is_finite() as i32 != 0 || f_hi.is_finite() as i32 != 0 {
                            guidance.entry_policy_valid = true_0 != 0;
                            guidance.entry_policy_bank_deg = solved;
                            guidance.entry_policy_aoa_deg = policy_aoa;
                            guidance.entry_policy_ut = request_ut;
                            fprintf(
                                stderr,
                                b"MM304 live replan: ut=%.1f target %.1f km bank %.1f deg aoa %.1f deg (short at 0 deg %+.1f km, at max %+.1f km, best %+.1f km; HAC law: energy height)\n\0"
                                    as *const u8 as *const ::core::ffi::c_char,
                                request_ut,
                                target / 1000.0f64,
                                solved,
                                policy_aoa,
                                f_lo / 1000.0f64,
                                f_hi / 1000.0f64,
                                best_miss / 1000.0f64,
                            );
                        }
                    }
                } else {
                    pr = predictor_simulate_terminal_shadow_ensemble(
                        state,
                        &raw mut telemetry,
                        &raw mut guidance,
                        if has_plan as ::core::ffi::c_int != 0 {
                            &raw mut plan
                        } else {
                            ::core::ptr::null_mut::<DeorbitPlan>()
                        },
                        &raw mut planet,
                        aero,
                        &raw mut envelope,
                        &raw mut calibration,
                        &raw mut cfg,
                        if terminal_model_valid as ::core::ffi::c_int != 0 {
                            &raw mut terminal_model
                        } else {
                            ::core::ptr::null_mut::<TerminalModel>()
                        },
                        600 as ::core::ffi::c_int as ::core::ffi::c_double,
                        true_0 != 0,
                    );
                }
                let mut solve_finished: ::core::ffi::c_double = monotonic_seconds();
                if !getenv(
                        b"KSP_LANDER_ENTRY_WORKER_TRACE\0" as *const u8
                            as *const ::core::ffi::c_char,
                    )
                    .is_null()
                {
                    fprintf(
                        stderr,
                        b"ENTRY_WORKER finish ut=%.3f wall=%.3f reached=%d captured=%d aborted=%d final_ut=%.3f\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        request_ut,
                        solve_finished - solve_started,
                        pr.reached_taem as ::core::ffi::c_int,
                        pr.taem_dynamic_interface_captured as ::core::ffi::c_int,
                        pr.shadow_aborted as ::core::ffi::c_int,
                        pr.final_state.ut,
                    );
                }
                deorbit_plan_clear(&raw mut plan);
                pthread_mutex_lock(&raw mut (*c).mutex);
                if entry_job {
                    (*c).forecast_drag_scale = forecast_drag;
                    (*c).forecast_lift_scale = forecast_lift;
                }
                if (*c).has_latest as ::core::ffi::c_int != 0
                    && (*c).latest_telemetry.ut >= request_ut - 1.0f64
                {
                    (*c).last_terminal_prediction_completion_ut = fmax(
                        request_ut,
                        (*c).latest_telemetry.ut,
                    );
                }
                (*c).last_terminal_prediction_completion_wall = solve_finished;
                (*c).last_terminal_prediction_solve_wall = fmax(
                    0.0f64,
                    solve_finished - solve_started,
                );
                let mut current_identity: AsyncPredictionIdentity = terminal_prediction_identity(
                    &raw mut (*c).guidance,
                );
                let mut accept: bool = !entry_job && !(*c).stop_prediction_thread
                    && !(*c).session.is_null()
                    && (*c).has_latest as ::core::ffi::c_int != 0
                    && generation == (*c).prediction_generation
                    && taem_exec_owns_vehicle(&raw mut (*c).guidance.taem_exec)
                        as ::core::ffi::c_int != 0
                    && request_ut > (*c).cached_prediction_ut
                    && async_prediction_result_fresh(
                        &raw mut request_identity,
                        &raw mut current_identity,
                        request_ut,
                        (*c).latest_telemetry.ut,
                        (*c).configuration.guidance.prediction_interval,
                    ) as ::core::ffi::c_int != 0;
                let mut entry_result_current: bool = entry_job as ::core::ffi::c_int != 0
                    && !(*c).stop_prediction_thread && !(*c).session.is_null()
                    && (*c).has_latest as ::core::ffi::c_int != 0
                    && generation == (*c).prediction_generation
                    && (*c).guidance.phase as ::core::ffi::c_uint
                        == entry_request.phase as ::core::ffi::c_uint
                    && (*c).guidance.control_plan_sequence
                        == entry_request.control_plan_sequence
                    && (*c).guidance.s_turn_sign == entry_request.s_turn_sign
                    && (*c).guidance.entry_control_reversals
                        == entry_request.entry_control_reversals
                    && (*c).guidance.entry_reversal_scheduled as ::core::ffi::c_int
                        == entry_request.entry_reversal_scheduled as ::core::ffi::c_int
                    && (!(*c).guidance.entry_reversal_scheduled
                        || (*c).guidance.entry_reversal_ut
                            == entry_request.entry_reversal_ut
                            && (*c).guidance.entry_reversal_sign
                                == entry_request.entry_reversal_sign
                            && (*c).guidance.entry_reversal_is_final
                                as ::core::ffi::c_int
                                == entry_request.entry_reversal_is_final
                                    as ::core::ffi::c_int)
                    && (*c).guidance.entry_final_reversal_pending as ::core::ffi::c_int
                        == entry_request.entry_final_reversal_pending
                            as ::core::ffi::c_int
                    && (*c).guidance.entry_final_reversal_completed as ::core::ffi::c_int
                        == entry_request.entry_final_reversal_completed
                            as ::core::ffi::c_int
                    && memcmp(
                        &raw mut (*c).guidance.taem_interface_target
                            as *const ::core::ffi::c_void,
                        &raw mut entry_request.taem_interface_target
                            as *const ::core::ffi::c_void,
                        ::core::mem::size_of::<TaemInterfaceTarget>() as size_t,
                    ) == 0 as ::core::ffi::c_int;
                if entry_result_current {
                    let mut adopted: bool = guidance_accept_entry_plan(
                        &raw mut (*c).guidance,
                        &raw mut entry_request,
                        &raw mut guidance,
                        &raw mut (*c).latest_telemetry,
                        &raw mut cfg,
                    );
                    if !getenv(
                            b"KSP_LANDER_ENTRY_WORKER_TRACE\0" as *const u8
                                as *const ::core::ffi::c_char,
                        )
                        .is_null()
                    {
                        fprintf(
                            stderr,
                            b"ENTRY_WORKER adoption request_ut=%.3f live_ut=%.3f adopted=%d\n\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            request_ut,
                            (*c).latest_telemetry.ut,
                            adopted as ::core::ffi::c_int,
                        );
                    }
                    if adopted as ::core::ffi::c_int != 0
                        && request_ut > (*c).cached_prediction_ut
                    {
                        terminal_prediction_commit(c, &raw mut pr, request_ut, sign);
                        let mut live_plan: *const EntryControlPlan = current_entry_control_plan(
                            c,
                        );
                        (*c).forecast_entry_plan_identity_valid = !live_plan.is_null()
                            && (*live_plan).valid as ::core::ffi::c_int != 0
                            && (*live_plan).plan_id != 0 as uint64_t;
                        (*c).forecast_entry_plan_id = if (*c)
                            .forecast_entry_plan_identity_valid as ::core::ffi::c_int
                            != 0
                        {
                            (*live_plan).plan_id
                        } else {
                            0 as uint64_t
                        };
                        (*c).forecast_entry_plan_version = if (*c)
                            .forecast_entry_plan_identity_valid as ::core::ffi::c_int
                            != 0
                        {
                            (*live_plan).plan_version
                        } else {
                            0 as uint64_t
                        };
                    }
                } else if accept {
                    terminal_prediction_commit(c, &raw mut pr, request_ut, sign);
                }
                pthread_mutex_unlock(&raw mut (*c).mutex);
                entry_prediction_clear(&raw mut pr);
            }
        }
    }
    return NULL;
}
unsafe extern "C" fn start_prediction_worker(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if !(*c).prediction_thread_started {
        (*c).stop_prediction_thread = false_0 != 0;
        (*c).last_terminal_prediction_completion_ut = -1e300f64;
        (*c).last_terminal_prediction_request_ut = (*c)
            .last_terminal_prediction_completion_ut;
        (*c).last_terminal_prediction_completion_wall = -1e300f64;
        (*c).last_terminal_prediction_solve_wall = 0.0f64;
        let mut attr: pthread_attr_t = core::mem::zeroed();
        pthread_attr_init(&raw mut attr);
        pthread_attr_setstacksize(
            &raw mut attr,
            (8 as ::core::ffi::c_int * 1024 as ::core::ffi::c_int
                * 1024 as ::core::ffi::c_int) as size_t,
        );
        if pthread_create(
            &raw mut (*c).prediction_thread,
            &raw mut attr,
            Some(
                terminal_prediction_worker
                    as unsafe extern "C" fn(
                        *mut ::core::ffi::c_void,
                    ) -> *mut ::core::ffi::c_void,
            ),
            c as *mut ::core::ffi::c_void,
        ) == 0 as ::core::ffi::c_int
        {
            (*c).prediction_thread_started = true_0 != 0;
        }
        if (*c).prediction_thread_started as ::core::ffi::c_int != 0
            && pthread_create(
                &raw mut (*c).mm305_planner_thread,
                &raw mut attr,
                Some(
                    mm305_planner_worker
                        as unsafe extern "C" fn(
                            *mut ::core::ffi::c_void,
                        ) -> *mut ::core::ffi::c_void,
                ),
                c as *mut ::core::ffi::c_void,
            ) == 0 as ::core::ffi::c_int
        {
            (*c).mm305_planner_started = true_0 != 0;
        }
        pthread_attr_destroy(&raw mut attr);
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
unsafe extern "C" fn stop_prediction_worker(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    let mut join: bool = (*c).prediction_thread_started;
    (*c).stop_prediction_thread = true_0 != 0;
    if !(*c).session.is_null() {
        krpc_safe((*c).session);
    }
    let mut join_planner: bool = (*c).mm305_planner_started;
    pthread_mutex_unlock(&raw mut (*c).mutex);
    if join {
        pthread_join(
            (*c).prediction_thread,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_void>(),
        );
    }
    if join_planner {
        pthread_join(
            (*c).mm305_planner_thread,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_void>(),
        );
    }
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).mm305_planner_started = false_0 != 0;
    (*c).prediction_thread_started = false_0 != 0;
    (*c).stop_prediction_thread = false_0 != 0;
    (*c).last_terminal_prediction_completion_ut = -1e300f64;
    (*c).last_terminal_prediction_request_ut = (*c)
        .last_terminal_prediction_completion_ut;
    (*c).last_terminal_prediction_completion_wall = -1e300f64;
    (*c).last_terminal_prediction_solve_wall = 0.0f64;
    pthread_mutex_unlock(&raw mut (*c).mutex);
}
unsafe extern "C" fn durable_flush_file(mut f: *mut FILE) {
    if f.is_null() {
        return;
    }
    fflush(f);
    let mut fd: ::core::ffi::c_int = fileno(f);
    if fd >= 0 as ::core::ffi::c_int {
        fsync(fd);
    }
}
unsafe extern "C" fn durable_flush_log(mut c: *mut LandingController) {
    durable_flush_file((*c).flight_log);
}
unsafe extern "C" fn close_log(mut c: *mut LandingController) {
    if !(*c).flight_log.is_null() {
        durable_flush_log(c);
        fclose((*c).flight_log);
        (*c).flight_log = ::core::ptr::null_mut::<FILE>();
    }
    if !(*c).planner_log.is_null() {
        planner_log_close((*c).planner_log);
        (*c).planner_log = ::core::ptr::null_mut::<PlannerLog>();
    }
}
unsafe extern "C" fn write_log_line(
    mut c: *mut LandingController,
    mut w: *mut JsonWriter,
) -> bool {
    if (*c).flight_log.is_null() || w.is_null() || (*w).failed as ::core::ffi::c_int != 0
        || (*w).data.is_null()
    {
        return false_0 != 0;
    }
    return fwrite(
        (*w).data as *const ::core::ffi::c_void,
        1 as size_t,
        (*w).length,
        (*c).flight_log,
    ) as size_t == (*w).length && fputc('\n' as i32, (*c).flight_log) != EOF;
}
unsafe extern "C" fn append_log_metadata(
    mut c: *mut LandingController,
    mut started: *const ::core::ffi::c_char,
) {
    if (*c).flight_log.is_null() || (*c).session.is_null() {
        return;
    }
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    let mut campaign_identity: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_CAMPAIGN_IDENTITY\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_raw(
        &raw mut w,
        b"{\"schemaVersion\":5,\"sessionSchemaVersion\":1,\"recordType\":\"sessionStart\",\"stream\":\"vehicle\",\"startedUTC\":\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, started);
    jw_raw(&raw mut w, b",\"sessionId\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, &raw mut (*c).log_session_id as *mut ::core::ffi::c_char);
    jw_raw(
        &raw mut w,
        b",\"campaignIdentity\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(
        &raw mut w,
        if !campaign_identity.is_null() && *campaign_identity as ::core::ffi::c_int != 0
        {
            campaign_identity
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    jw_raw(
        &raw mut w,
        b",\"peer\":{\"stream\":\"planner\",\"available\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_bool(&raw mut w, !(*c).planner_log.is_null());
    jw_raw(&raw mut w, b",\"path\":\0" as *const u8 as *const ::core::ffi::c_char);
    if !(*c).planner_log.is_null() {
        jw_string(&raw mut w, planner_log_current_path((*c).planner_log));
    } else if (*c).log_planner_path[0 as ::core::ffi::c_int as usize] != 0 {
        jw_string(
            &raw mut w,
            &raw mut (*c).log_planner_path as *mut ::core::ffi::c_char,
        );
    } else {
        jw_null(&raw mut w);
    }
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    jw_raw(&raw mut w, b",\"vessel\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, krpc_session_vessel((*c).session));
    jw_raw(&raw mut w, b",\"planet\":\0" as *const u8 as *const ::core::ffi::c_char);
    planet_model_replay_json(&raw mut w, p);
    jw_raw(
        &raw mut w,
        b",\"runtime\":{\"transport\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, krpc_session_transport((*c).session));
    jw_raw(
        &raw mut w,
        b",\"clientLibrary\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, krpc_session_library_version((*c).session));
    jw_raw(
        &raw mut w,
        b",\"nativeBuild\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, &raw mut (*c).log_native_build as *mut ::core::ffi::c_char);
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    jw_raw(
        &raw mut w,
        b",\"modelIdentity\":{\"vehicleModelId\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char);
    jw_raw(&raw mut w, b",\"certified\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_raw(
        &raw mut w,
        if shuttle_sim_model_matches_vehicle(
            &raw mut (*c).vehicle_identity as *mut ::core::ffi::c_char,
            ::core::ptr::null_mut::<::core::ffi::c_char>(),
            0 as size_t,
        ) as ::core::ffi::c_int != 0
        {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    jw_raw(
        &raw mut w,
        b",\"physicsContext\":{\"structureId\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, krpc_session_physics_structure_id((*c).session));
    jw_raw(
        &raw mut w,
        b",\"environmentId\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, krpc_session_physics_environment_id((*c).session));
    jw_raw(&raw mut w, b",\"storage\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(&raw mut w, krpc_session_physics_storage((*c).session));
    jw_raw(
        &raw mut w,
        b",\"historicalSamplesLoaded\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_integer(
        &raw mut w,
        krpc_session_physics_history_count((*c).session) as ::core::ffi::c_longlong,
    );
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    jw_raw(
        &raw mut w,
        b",\"encoding\":{\"policy\":\"adaptive delta with semantic-event forcing\",\"periodicKeyframeSeconds\":5,\"nominalHz\":10,\"thinAtmosphereHz\":5,\"rapidTransientHz\":20}\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    jw_raw(
        &raw mut w,
        b",\"configuration\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    landing_configuration_json(&raw mut w, &raw mut (*c).configuration);
    jw_char(&raw mut w, '}' as i32 as ::core::ffi::c_char);
    if write_log_line(c, &raw mut w) {
        durable_flush_log(c);
    }
    jw_free(&raw mut w);
}
unsafe extern "C" fn vehicle_log_sample(
    mut c: *const LandingController,
    mut event: *const ::core::ffi::c_char,
) -> VehicleLogSample {
    let mut s: VehicleLogSample = VehicleLogSample {
        tick_sequence: 0 as uint64_t,
        wall_monotonic_seconds: 0.,
        ut: 0.,
        latitude: 0.,
        longitude: 0.,
        altitude: 0.,
        radar_altitude: 0.,
        position: [0.; 3],
        velocity: [0.; 3],
        true_air_speed: 0.,
        horizontal_speed: 0.,
        vertical_speed: 0.,
        flight_path_angle: 0.,
        pitch: 0.,
        roll: 0.,
        heading: 0.,
        angle_of_attack: 0.,
        sideslip: 0.,
        pitch_rate: 0.,
        roll_rate: 0.,
        yaw_rate: 0.,
        course_rate: 0.,
        coordinate_pitch_rate: 0.,
        coordinate_roll_rate: 0.,
        heading_rate: 0.,
        angle_of_attack_rate: 0.,
        body_pitch_rate: 0.,
        body_roll_rate: 0.,
        body_yaw_rate: 0.,
        dynamic_pressure: 0.,
        static_pressure: 0.,
        atmospheric_density: 0.,
        mach: 0.,
        stall_fraction: 0.,
        stall_fraction_is_measured: false,
        lift_force: 0.,
        drag_force: 0.,
        g_force: 0.,
        energy_excess_range: 0.,
        has_force_vectors: false,
        lift_vector: [0.; 3],
        drag_vector: [0.; 3],
        aero_acceleration: [0.; 3],
        physics_confidence: 0.,
        physics_model_residual: 0.,
        physics_model_residual_confidence: 0.,
        physics_certified_uncertainty: 0.,
        physics_force_residual_per_q: [0.; 3],
        physics_force_residual_sigma_per_q: [0.; 3],
        physics_force_residual_confidence: 0.,
        physics_airbrake_model_available: false,
        physics_airbrake_model_confidence: 0.,
        physics_airbrake_drag_accel: 0.,
        physics_samples: 0,
        physics_live_samples: 0,
        mass: 0.,
        available_thrust: 0.,
        current_thrust: 0.,
        has_dry_mass: false,
        dry_mass: 0.,
        has_center_of_mass: false,
        center_of_mass: [0.; 3],
        has_center_of_mass_root: false,
        center_of_mass_root: [0.; 3],
        has_torque: false,
        available_torque: [0.; 3],
        has_inertia: false,
        moment_of_inertia: [0.; 3],
        target_pitch: 0.,
        target_heading: 0.,
        target_roll: 0.,
        target_throttle: 0.,
        has_target_aoa: false,
        target_aoa: 0.,
        target_gear: false,
        target_brakes: false,
        target_airbrakes: false,
        gear: false,
        brakes: false,
        has_airbrakes: false,
        airbrakes: false,
        has_sas_state: false,
        sas_enabled: false,
        has_control_state: false,
        control_state_pitch: 0.,
        control_state_roll: 0.,
        control_state_yaw: 0.,
        control_state_throttle: 0.,
        command_applied: false,
        has_actuator_feedback: false,
        has_control_diagnostics: false,
        control_pitch: 0.,
        control_roll: 0.,
        control_yaw: 0.,
        control_pitch_error: 0.,
        control_pitch_trim: 0.,
        control_pitch_authority: 0.,
        control_pitch_aero_fraction: 0.,
        has_control_surface_state: false,
        control_surface_state_ut: 0.,
        control_surface_count: 0,
        control_surfaces: [VehicleLogSurfaceSample {
            part_name: [0; 64],
            role: [0; 16],
            deployed: false,
            deflection_override: false,
            override_deflection: 0.,
        }; 16],
        has_loop_wall_delta: false,
        has_telemetry_latency: false,
        has_guidance_compute: false,
        has_apply_latency: false,
        has_control_loop: false,
        has_warp: false,
        warp_rate: 0.,
        loop_wall_delta_ms: 0.,
        telemetry_latency_ms: 0.,
        guidance_compute_ms: 0.,
        apply_latency_ms: 0.,
        control_loop_ms: 0.,
        automation_engaged: false,
        paused: false,
        constraint_active: false,
        phase: [0; 40],
        control_profile: [0; 32],
        vessel_situation: [0; 64],
        has_plan_identity: false,
        plan_id: 0,
        plan_version: 0,
        parent_plan_id: 0,
        parent_plan_version: 0,
        range_to_site: 0.,
        bearing_to_site: 0.,
        runway_along_track: 0.,
        runway_cross_track: 0.,
        specific_mechanical_energy: 0.,
        entry_plan_valid: false,
        entry_reversal_scheduled: false,
        entry_reversal_time_remaining: 0.,
        entry_reversal_ut: 0.,
        entry_reversal_range: 0.,
        entry_reversal_sign: 0.,
        event: [0; 512],
    };
    let mut q: *const LandingSnapshot = &raw const (*c).snapshot;
    let mut t: *const Telemetry = &raw const (*q).telemetry;
    let mut p: *const PlanetModel = if !(*c).session.is_null() {
        krpc_session_planet((*c).session)
    } else {
        ::core::ptr::null::<PlanetModel>()
    };
    s.tick_sequence = (*q).tick_sequence;
    s.wall_monotonic_seconds = (*q).wall_monotonic_seconds;
    s.ut = (*t).ut;
    s.latitude = (*t).latitude;
    s.longitude = (*t).longitude;
    s.altitude = (*t).mean_altitude;
    s.radar_altitude = (*t).radar_altitude;
    if (*q).has_vehicle_state {
        s.position[0 as ::core::ffi::c_int as usize] = (*q).vehicle_state.position.x;
        s.position[1 as ::core::ffi::c_int as usize] = (*q).vehicle_state.position.y;
        s.position[2 as ::core::ffi::c_int as usize] = (*q).vehicle_state.position.z;
        s.velocity[0 as ::core::ffi::c_int as usize] = (*q).vehicle_state.velocity.x;
        s.velocity[1 as ::core::ffi::c_int as usize] = (*q).vehicle_state.velocity.y;
        s.velocity[2 as ::core::ffi::c_int as usize] = (*q).vehicle_state.velocity.z;
    }
    s.true_air_speed = (*t).true_air_speed;
    s.horizontal_speed = (*t).horizontal_speed;
    s.vertical_speed = (*t).vertical_speed;
    s.flight_path_angle = (*t).flight_path_angle;
    s.pitch = (*t).pitch;
    s.roll = (*t).roll;
    s.heading = (*t).heading;
    s.angle_of_attack = (*t).angle_of_attack;
    s.sideslip = (*t).sideslip;
    s.pitch_rate = if (*t).has_body_pitch_rate as ::core::ffi::c_int != 0 {
        (*t).body_pitch_rate
    } else {
        (*t).pitch_rate
    };
    s.roll_rate = if (*t).has_body_roll_rate as ::core::ffi::c_int != 0 {
        (*t).body_roll_rate
    } else {
        (*t).roll_rate
    };
    s.yaw_rate = if (*t).has_body_yaw_rate as ::core::ffi::c_int != 0 {
        (*t).body_yaw_rate
    } else {
        (*t).heading_rate
    };
    s.course_rate = if (*t).has_course_rate as ::core::ffi::c_int != 0 {
        (*t).course_rate
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    s.coordinate_pitch_rate = (*t).pitch_rate;
    s.coordinate_roll_rate = (*t).roll_rate;
    s.heading_rate = (*t).heading_rate;
    s.angle_of_attack_rate = if (*t).has_angle_of_attack_rate as ::core::ffi::c_int != 0
    {
        (*t).angle_of_attack_rate
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    s.body_pitch_rate = if (*t).has_body_pitch_rate as ::core::ffi::c_int != 0 {
        (*t).body_pitch_rate
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    s.body_roll_rate = if (*t).has_body_roll_rate as ::core::ffi::c_int != 0 {
        (*t).body_roll_rate
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    s.body_yaw_rate = if (*t).has_body_yaw_rate as ::core::ffi::c_int != 0 {
        (*t).body_yaw_rate
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    s.dynamic_pressure = (*t).dynamic_pressure;
    s.static_pressure = (*t).static_pressure;
    s.atmospheric_density = (*t).atmospheric_density;
    s.mach = (*t).mach;
    s.stall_fraction = (*t).stall_fraction;
    s.stall_fraction_is_measured = (*t).stall_fraction_is_measured;
    s.lift_force = (*t).lift_force;
    s.drag_force = (*t).drag_force;
    s.g_force = (*t).g_force;
    s.energy_excess_range = (*t).energy_excess_range;
    s.has_force_vectors = (*t).has_force_vectors;
    if (*t).has_force_vectors {
        s.lift_vector[0 as ::core::ffi::c_int as usize] = (*t).lift_vector.x;
        s.lift_vector[1 as ::core::ffi::c_int as usize] = (*t).lift_vector.y;
        s.lift_vector[2 as ::core::ffi::c_int as usize] = (*t).lift_vector.z;
        s.drag_vector[0 as ::core::ffi::c_int as usize] = (*t).drag_vector.x;
        s.drag_vector[1 as ::core::ffi::c_int as usize] = (*t).drag_vector.y;
        s.drag_vector[2 as ::core::ffi::c_int as usize] = (*t).drag_vector.z;
        if (*t).mass > 1 as ::core::ffi::c_int as ::core::ffi::c_double {
            s.aero_acceleration[0 as ::core::ffi::c_int as usize] = ((*t).lift_vector.x
                + (*t).drag_vector.x) / (*t).mass;
            s.aero_acceleration[1 as ::core::ffi::c_int as usize] = ((*t).lift_vector.y
                + (*t).drag_vector.y) / (*t).mass;
            s.aero_acceleration[2 as ::core::ffi::c_int as usize] = ((*t).lift_vector.z
                + (*t).drag_vector.z) / (*t).mass;
        }
    }
    s.physics_confidence = (*t).physics_confidence;
    s.physics_model_residual = (*t).physics_model_residual;
    s.physics_model_residual_confidence = (*t).physics_model_residual_confidence;
    s.physics_certified_uncertainty = (*t).physics_certified_uncertainty;
    s.physics_force_residual_per_q[0 as ::core::ffi::c_int as usize] = (*t)
        .physics_force_residual_per_q
        .x;
    s.physics_force_residual_per_q[1 as ::core::ffi::c_int as usize] = (*t)
        .physics_force_residual_per_q
        .y;
    s.physics_force_residual_per_q[2 as ::core::ffi::c_int as usize] = (*t)
        .physics_force_residual_per_q
        .z;
    s.physics_force_residual_sigma_per_q[0 as ::core::ffi::c_int as usize] = (*t)
        .physics_force_residual_sigma_per_q
        .x;
    s.physics_force_residual_sigma_per_q[1 as ::core::ffi::c_int as usize] = (*t)
        .physics_force_residual_sigma_per_q
        .y;
    s.physics_force_residual_sigma_per_q[2 as ::core::ffi::c_int as usize] = (*t)
        .physics_force_residual_sigma_per_q
        .z;
    s.physics_force_residual_confidence = (*t).physics_force_residual_confidence;
    s.physics_airbrake_model_available = (*t).physics_airbrake_model_available;
    s.physics_airbrake_model_confidence = (*t).physics_airbrake_model_confidence;
    s.physics_airbrake_drag_accel = (*t).physics_airbrake_drag_accel;
    s.physics_samples = (*t).physics_samples;
    s.physics_live_samples = (*t).physics_live_samples;
    s.mass = (*t).mass;
    s.has_dry_mass = (*t).dry_mass.is_finite() as i32 != 0
        && (*t).dry_mass > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    s.dry_mass = (*t).dry_mass;
    s.available_thrust = (*t).available_thrust;
    s.current_thrust = (*t).current_thrust;
    s.has_center_of_mass = (*t).has_center_of_mass;
    if (*t).has_center_of_mass {
        s.center_of_mass[0 as ::core::ffi::c_int as usize] = (*t).center_of_mass.x;
        s.center_of_mass[1 as ::core::ffi::c_int as usize] = (*t).center_of_mass.y;
        s.center_of_mass[2 as ::core::ffi::c_int as usize] = (*t).center_of_mass.z;
    }
    s.has_center_of_mass_root = (*t).has_center_of_mass_root;
    if (*t).has_center_of_mass_root {
        s.center_of_mass_root[0 as ::core::ffi::c_int as usize] = (*t)
            .center_of_mass_root
            .x;
        s.center_of_mass_root[1 as ::core::ffi::c_int as usize] = (*t)
            .center_of_mass_root
            .y;
        s.center_of_mass_root[2 as ::core::ffi::c_int as usize] = (*t)
            .center_of_mass_root
            .z;
    }
    s.has_torque = (*t).has_torque;
    s.available_torque[0 as ::core::ffi::c_int as usize] = (*t).available_pitch_torque;
    s.available_torque[1 as ::core::ffi::c_int as usize] = (*t).available_roll_torque;
    s.available_torque[2 as ::core::ffi::c_int as usize] = (*t).available_yaw_torque;
    s.has_inertia = (*t).has_inertia;
    s.moment_of_inertia[0 as ::core::ffi::c_int as usize] = (*t).pitch_moment_of_inertia;
    s.moment_of_inertia[1 as ::core::ffi::c_int as usize] = (*t).roll_moment_of_inertia;
    s.moment_of_inertia[2 as ::core::ffi::c_int as usize] = (*t).yaw_moment_of_inertia;
    s.target_pitch = (*q).command.target_pitch;
    s.target_heading = (*q).command.target_heading;
    s.target_roll = (*q).command.target_roll;
    s.target_throttle = (*q).command.target_throttle;
    s.has_target_aoa = (*q).command.has_target_aoa;
    s.target_aoa = (*q).command.target_aoa;
    s.target_gear = (*q).command.gear;
    s.target_brakes = (*q).command.brakes;
    s.target_airbrakes = (*q).command.airbrakes;
    s.gear = (*t).gear;
    s.brakes = (*t).brakes;
    s.has_airbrakes = (*t).has_airbrakes;
    s.airbrakes = (*t).airbrakes;
    s.has_sas_state = (*t).has_sas_state;
    s.sas_enabled = (*t).sas_enabled;
    s.has_control_state = (*t).has_controls;
    s.control_state_pitch = (*t).control_pitch;
    s.control_state_roll = (*t).control_roll;
    s.control_state_yaw = (*t).control_yaw;
    s.control_state_throttle = (*t).throttle;
    s.command_applied = (*q).command_applied;
    s.has_actuator_feedback = (*q).command_apply_attempted as ::core::ffi::c_int != 0
        && (*q).applied_command.has_control_diagnostics as ::core::ffi::c_int != 0;
    s.has_control_diagnostics = (*q).command_apply_attempted as ::core::ffi::c_int != 0
        && (*q).applied_command.has_control_diagnostics as ::core::ffi::c_int != 0;
    s.control_pitch = (*q).applied_command.control_pitch;
    s.control_roll = (*q).applied_command.control_roll;
    s.control_yaw = (*q).applied_command.control_yaw;
    s.control_pitch_error = (*q).applied_command.control_pitch_error;
    s.control_pitch_trim = (*q).applied_command.control_pitch_trim;
    s.control_pitch_authority = (*q).applied_command.control_pitch_authority;
    s.control_pitch_aero_fraction = (*q).applied_command.control_pitch_aero_fraction;
    s.has_control_surface_state = (*t).has_control_surface_state;
    s.control_surface_state_ut = (*t).control_surface_state_ut;
    s.control_surface_count = if (*t).control_surface_count
        > LANDING_CONTROL_SURFACE_MAX as size_t
    {
        LANDING_CONTROL_SURFACE_MAX as size_t
    } else {
        (*t).control_surface_count
    };
    let mut i: size_t = 0 as size_t;
    while i < s.control_surface_count {
        snprintf(
            &raw mut (*(&raw mut s.control_surfaces as *mut VehicleLogSurfaceSample)
                .offset(i as isize))
                .part_name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw const (*(&raw const (*t).control_surfaces
                as *const TelemetryControlSurface)
                .offset(i as isize))
                .part_name as *const ::core::ffi::c_char,
        );
        snprintf(
            &raw mut (*(&raw mut s.control_surfaces as *mut VehicleLogSurfaceSample)
                .offset(i as isize))
                .role as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 16]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw const (*(&raw const (*t).control_surfaces
                as *const TelemetryControlSurface)
                .offset(i as isize))
                .role as *const ::core::ffi::c_char,
        );
        s.control_surfaces[i as usize].deployed = (*t)
            .control_surfaces[i as usize]
            .deployed;
        s.control_surfaces[i as usize].deflection_override = (*t)
            .control_surfaces[i as usize]
            .deflection_override;
        s.control_surfaces[i as usize].override_deflection = (*t)
            .control_surfaces[i as usize]
            .override_deflection;
        i = i.wrapping_add(1);
    }
    s.has_warp = (*t).has_warp;
    s.warp_rate = (*t).warp_rate;
    s.has_loop_wall_delta = (*t).has_loop_wall_delta;
    s.has_telemetry_latency = (*t).has_telemetry_latency;
    s.has_guidance_compute = (*t).has_guidance_compute;
    s.has_apply_latency = (*t).has_apply_latency;
    s.has_control_loop = (*t).has_control_loop;
    s.loop_wall_delta_ms = (*t).loop_wall_delta_ms;
    s.telemetry_latency_ms = (*t).telemetry_latency_ms;
    s.guidance_compute_ms = (*t).guidance_compute_ms;
    s.apply_latency_ms = (*t).apply_latency_ms;
    s.control_loop_ms = (*t).control_loop_ms;
    s.automation_engaged = (*q).automation_engaged;
    s.paused = (*q).paused;
    s.constraint_active = (*t).dynamic_pressure
        >= (*c).configuration.vehicle.maximum_dynamic_pressure
        || (*t).g_force >= (*c).configuration.vehicle.maximum_g_load
        || (*t).stall_fraction >= 1.0f64;
    snprintf(
        &raw mut s.phase as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 40]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        phase_string((*q).phase),
    );
    snprintf(
        &raw mut s.control_profile as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 32]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        profile_string((*q).command.control_profile),
    );
    snprintf(
        &raw mut s.vessel_situation as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        &raw const (*t).vessel_situation as *const ::core::ffi::c_char,
    );
    s.range_to_site = (*t).range_to_site;
    s.bearing_to_site = (*t).bearing_to_site;
    s.runway_along_track = (*t).runway_along_track;
    s.runway_cross_track = (*t).runway_cross_track;
    s.specific_mechanical_energy = if !p.is_null() {
        rotating_specific_energy(
            (*t).latitude,
            (*t).mean_altitude,
            (*t).true_air_speed,
            p,
        )
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    s.entry_plan_valid = (*q).guidance_entry_plan_valid;
    s.entry_reversal_scheduled = (*q).guidance_entry_reversal_scheduled;
    s.entry_reversal_time_remaining = (*q).guidance_entry_reversal_time_remaining;
    s.entry_reversal_ut = if (*q).guidance_entry_reversal_scheduled as ::core::ffi::c_int
        != 0
    {
        (*t).ut + (*q).guidance_entry_reversal_time_remaining
    } else {
        ::core::f32::NAN as ::core::ffi::c_double
    };
    s.entry_reversal_range = (*q).guidance_entry_reversal_range;
    s.entry_reversal_sign = (*q).guidance_entry_reversal_sign;
    let mut plan: *const EntryControlPlan = current_entry_control_plan(c);
    s.has_plan_identity = !plan.is_null() && (*plan).plan_id > 0 as uint64_t
        && (*plan).plan_version > 0 as uint64_t;
    if s.has_plan_identity {
        s.plan_id = (*plan).plan_id;
        s.plan_version = (*plan).plan_version;
        s.parent_plan_id = (*plan).parent_plan_id;
        s.parent_plan_version = (*plan).parent_plan_version;
    }
    if !event.is_null() && *event as ::core::ffi::c_int != 0 {
        snprintf(
            &raw mut s.event as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            event,
        );
    }
    return s;
}
unsafe extern "C" fn append_vehicle_log(
    mut c: *mut LandingController,
    mut event: *const ::core::ffi::c_char,
    mut force_event: bool,
    mut force_keyframe: bool,
) -> bool {
    if (*c).flight_log.is_null() {
        return false_0 != 0;
    }
    let mut s: VehicleLogSample = vehicle_log_sample(c, event);
    let mut d: VehicleLogDecision = vehicle_log_decide(
        &raw mut (*c).vehicle_log_encoder,
        &raw mut s,
        force_event,
        force_keyframe,
    );
    if !d.emit {
        return true_0 != 0;
    }
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    vehicle_log_record_json(
        &raw mut w,
        &raw mut (*c).log_session_id as *mut ::core::ffi::c_char,
        (*c).vehicle_record_sequence.wrapping_add(1 as uint64_t),
        &raw mut s,
        &raw mut d,
    );
    let mut ok: bool = write_log_line(c, &raw mut w);
    jw_free(&raw mut w);
    if ok {
        (*c).vehicle_record_sequence = (*c).vehicle_record_sequence.wrapping_add(1);
        vehicle_log_commit(&raw mut (*c).vehicle_log_encoder, &raw mut s, &raw mut d);
        if d.durable {
            durable_flush_log(c);
        } else {
            fflush((*c).flight_log);
        }
    }
    return ok;
}
unsafe extern "C" fn current_entry_control_plan(
    mut c: *const LandingController,
) -> *const EntryControlPlan {
    if (*c).guidance.entry_s_turn_plan.valid {
        return &raw const (*c).guidance.entry_s_turn_plan;
    }
    return ::core::ptr::null::<EntryControlPlan>();
}
unsafe extern "C" fn append_planner_log(
    mut c: *mut LandingController,
    mut event: *const ::core::ffi::c_char,
    mut explicit_force: bool,
) {
    if (*c).planner_log.is_null() || !(*c).snapshot.has_vehicle_state {
        return;
    }
    let mut t: *const Telemetry = &raw mut (*c).snapshot.telemetry;
    let mut constraint_active: bool = (*t).dynamic_pressure
        >= (*c).configuration.vehicle.maximum_dynamic_pressure
        || (*t).g_force >= (*c).configuration.vehicle.maximum_g_load
        || (*t).stall_fraction >= 1.0f64;
    let mut prediction_matches_plan: bool = !(*c).prediction_lineage_discontinuity
        && entry_forecast_matches_guidance(c) as ::core::ffi::c_int != 0;
    let mut s: PlannerLogSample = PlannerLogSample {
        tick_sequence: (*c).snapshot.tick_sequence,
        ut: (*t).ut,
        phase: (*c).snapshot.phase,
        telemetry: t,
        snapshot: &raw mut (*c).snapshot,
        plan: current_entry_control_plan(c),
        raw_prediction: if prediction_matches_plan as ::core::ffi::c_int != 0 {
            &raw mut (*c).dynamic_trajectory
        } else {
            ::core::ptr::null_mut::<Trajectory>()
        },
        published_prediction: if prediction_matches_plan as ::core::ffi::c_int != 0 {
            &raw mut (*c).stabilized_trajectory
        } else {
            ::core::ptr::null_mut::<Trajectory>()
        },
        calibration: &raw mut (*c).trajectory_calibration,
        envelope: &raw mut (*c).envelope,
        constraint_active: constraint_active,
        predictor_discontinuity: !prediction_matches_plan,
        explicit_force: explicit_force,
        event_reason: event,
    };
    planner_log_record((*c).planner_log, &raw mut s);
}
unsafe extern "C" fn start_log(mut c: *mut LandingController) {
    close_log(c);
    (*c).tick_sequence = 0 as uint64_t;
    (*c).log_sequence = 0 as uint64_t;
    (*c).vehicle_record_sequence = 0 as uint64_t;
    (*c).snapshot.tick_sequence = 0 as uint64_t;
    (*c).snapshot.log_sequence = 0 as uint64_t;
    (*c).log_native_build[0 as ::core::ffi::c_int as usize] = 0 as ::core::ffi::c_char;
    (*c).log_planner_path[0 as ::core::ffi::c_int as usize] = 0 as ::core::ffi::c_char;
    vehicle_log_encoder_init(&raw mut (*c).vehicle_log_encoder);
    (*c).log_actual_trajectory_dirty = false_0 != 0;
    (*c).log_reference_trajectory_dirty = false_0 != 0;
    (*c).log_plan_trajectory_dirty = false_0 != 0;
    (*c).log_raw_plan_trajectory_dirty = false_0 != 0;
    let mut root: [::core::ffi::c_char; 1200] = [
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
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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
    let mut dir: [::core::ffi::c_char; 1300] = [0; 1300];
    let mut path: [::core::ffi::c_char; 1800] = [0; 1800];
    let mut configured: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_ROOT\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !configured.is_null() && *configured as ::core::ffi::c_int != 0 {
        snprintf(
            &raw mut root as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1200]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            configured,
        );
    } else if getcwd(
            &raw mut root as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1200]>() as size_t,
        )
        .is_null()
    {
        return
    }
    if access(
        b"Configuration/default.json\0" as *const u8 as *const ::core::ffi::c_char,
        R_OK,
    ) != 0 as ::core::ffi::c_int
        && access(
            b"../Configuration/default.json\0" as *const u8
                as *const ::core::ffi::c_char,
            R_OK,
        ) == 0 as ::core::ffi::c_int
    {
        let mut parent: [::core::ffi::c_char; 1200] = [0; 1200];
        if !realpath(
                b"..\0" as *const u8 as *const ::core::ffi::c_char,
                &raw mut parent as *mut ::core::ffi::c_char,
            )
            .is_null()
        {
            snprintf(
                &raw mut root as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 1200]>() as size_t,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                &raw mut parent as *mut ::core::ffi::c_char,
            );
        }
    }
    snprintf(
        &raw mut dir as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1300]>() as size_t,
        b"%s/FlightLogs\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut root as *mut ::core::ffi::c_char,
    );
    mkdir(&raw mut dir as *mut ::core::ffi::c_char, 0o700 as __mode_t);
    let mut now: time_t = time(::core::ptr::null_mut::<time_t>());
    let mut tm: tm = ::core::mem::zeroed();
    gmtime_r(&raw mut now, &raw mut tm);
    let mut stamp: [::core::ffi::c_char; 64] = [0; 64];
    strftime(
        &raw mut stamp as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"%Y-%m-%dT%H-%M-%SZ\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut tm,
    );
    let mut vessel: [::core::ffi::c_char; 128] = [0; 128];
    snprintf(
        &raw mut vessel as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        krpc_session_vessel((*c).session),
    );
    let mut p: *mut ::core::ffi::c_char = &raw mut vessel as *mut ::core::ffi::c_char;
    while *p != 0 {
        if *p as ::core::ffi::c_int == '/' as i32
            || *p as ::core::ffi::c_int == '\\' as i32
            || *p as ::core::ffi::c_int == ':' as i32
            || *p as ::core::ffi::c_int == ' ' as i32
        {
            *p = '-' as i32 as ::core::ffi::c_char;
        }
        p = p.offset(1);
    }
    let mut safe_vessel: *const ::core::ffi::c_char = if vessel[0 as ::core::ffi::c_int
        as usize] as ::core::ffi::c_int != 0
    {
        &raw mut vessel as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
    } else {
        b"vessel\0" as *const u8 as *const ::core::ffi::c_char
    };
    let mut nonce: ::core::ffi::c_ulonglong = (monotonic_seconds() * 1000000.0f64)
        as ::core::ffi::c_ulonglong;
    snprintf(
        &raw mut (*c).log_session_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"%s-%ld-%llu\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut stamp as *mut ::core::ffi::c_char,
        getpid() as ::core::ffi::c_long,
        nonce,
    );
    snprintf(
        &raw mut (*c).log_native_build as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        b"%s %s\0" as *const u8 as *const ::core::ffi::c_char,
        concat!(env!("LANDING_BUILD_DATE"), "\0").as_ptr() as *const ::core::ffi::c_char,
        concat!(env!("LANDING_BUILD_TIME"), "\0").as_ptr() as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut path as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1800]>() as size_t,
        b"%s/%s-%s-vehicle.jsonl\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut dir as *mut ::core::ffi::c_char,
        &raw mut stamp as *mut ::core::ffi::c_char,
        safe_vessel,
    );
    (*c).flight_log = fopen(
        &raw mut path as *mut ::core::ffi::c_char,
        b"w\0" as *const u8 as *const ::core::ffi::c_char,
    ) as *mut FILE;
    snprintf(
        &raw mut (*c).log_planner_path as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1700]>() as size_t,
        b"%s/%s-%s-planner.jsonl\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut dir as *mut ::core::ffi::c_char,
        &raw mut stamp as *mut ::core::ffi::c_char,
        safe_vessel,
    );
    (*c).planner_log = planner_log_open(
        &raw mut dir as *mut ::core::ffi::c_char,
        &raw mut stamp as *mut ::core::ffi::c_char,
        safe_vessel,
        &raw mut (*c).log_session_id as *mut ::core::ffi::c_char,
        &raw mut (*c).log_native_build as *mut ::core::ffi::c_char,
        &raw mut path as *mut ::core::ffi::c_char,
        !(*c).flight_log.is_null(),
        &raw mut (*c).configuration,
        0 as size_t,
    );
    if !(*c).flight_log.is_null() {
        append_log_metadata(c, &raw mut stamp as *mut ::core::ffi::c_char);
    }
    if (*c).flight_log.is_null() && (*c).planner_log.is_null() {
        set_warning(
            c,
            b"Both forensic log streams failed to open. Guidance can continue, but this session cannot count as acceptance evidence.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    } else if (*c).flight_log.is_null() {
        set_warning(
            c,
            b"Vehicle forensic stream failed to open. Planner logging may continue, but this session cannot count as acceptance evidence.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    } else if (*c).planner_log.is_null() {
        set_warning(
            c,
            b"Planner forensic stream failed to open. Vehicle logging may continue, but this session cannot count as acceptance evidence.\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
}
unsafe extern "C" fn append_log(mut c: *mut LandingController) {
    append_vehicle_log(
        c,
        ::core::ptr::null::<::core::ffi::c_char>(),
        false_0 != 0,
        false_0 != 0,
    );
    append_planner_log(c, ::core::ptr::null::<::core::ffi::c_char>(), false_0 != 0);
    (*c).log_actual_trajectory_dirty = false_0 != 0;
    (*c).log_reference_trajectory_dirty = false_0 != 0;
    (*c).log_plan_trajectory_dirty = false_0 != 0;
    (*c).log_raw_plan_trajectory_dirty = false_0 != 0;
}
unsafe extern "C" fn append_configuration_log(mut c: *mut LandingController) {
    if (*c).flight_log.is_null() && (*c).planner_log.is_null() {
        return;
    }
    (*c).snapshot.wall_monotonic_seconds = monotonic_seconds();
    append_vehicle_log(
        c,
        b"event=configuration-changed\0" as *const u8 as *const ::core::ffi::c_char,
        true_0 != 0,
        false_0 != 0,
    );
    append_planner_log(
        c,
        b"event=configuration-changed\0" as *const u8 as *const ::core::ffi::c_char,
        true_0 != 0,
    );
}
unsafe extern "C" fn log_state_event(
    mut c: *mut LandingController,
    mut reason: *const ::core::ffi::c_char,
    mut durable: bool,
) {
    if (*c).flight_log.is_null() && (*c).planner_log.is_null() {
        return;
    }
    (*c).log_sequence = (*c).log_sequence.wrapping_add(1);
    (*c).snapshot.log_sequence = (*c).log_sequence;
    (*c).snapshot.wall_monotonic_seconds = monotonic_seconds();
    snprintf(
        &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if !reason.is_null() {
            reason
        } else {
            b"event=state-change\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    append_vehicle_log(
        c,
        if !reason.is_null() {
            reason
        } else {
            b"event=state-change\0" as *const u8 as *const ::core::ffi::c_char
        },
        true_0 != 0,
        false_0 != 0,
    );
    append_planner_log(
        c,
        if !reason.is_null() {
            reason
        } else {
            b"event=state-change\0" as *const u8 as *const ::core::ffi::c_char
        },
        true_0 != 0,
    );
    if durable {
        durable_flush_log(c);
    }
}
unsafe extern "C" fn clear_snapshot_dynamic(mut s: *mut LandingSnapshot) {
    trajectory_clear(&raw mut (*s).actual_trajectory);
    trajectory_clear(&raw mut (*s).reference_trajectory);
    trajectory_clear(&raw mut (*s).predicted_trajectory);
}
unsafe extern "C" fn reset_snapshot(
    mut c: *mut LandingController,
    mut status: *const ::core::ffi::c_char,
) {
    clear_snapshot_dynamic(&raw mut (*c).snapshot);
    landing_snapshot_init(&raw mut (*c).snapshot, &raw mut (*c).configuration.vehicle);
    if !status.is_null() {
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            status,
        );
    }
}
unsafe extern "C" fn reset_models(
    mut c: *mut LandingController,
    mut p: *const VehicleProfile,
) {
    vessel_physics_init(&raw mut (*c).physics);
    (*c).adaptive_profile = *p;
    adaptive_calibrator_reset(&raw mut (*c).adaptive, p);
    (*c).aerodynamics = AerodynamicModel {
        lift_to_drag: (*p).estimated_lift_to_drag,
        ballistic_coefficient: (*p).estimated_ballistic_coefficient,
        confidence: 0.2f64,
    };
    (*c).planning_aerodynamics = (*c).aerodynamics;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 4 as ::core::ffi::c_int {
        (*c).envelope.regimes[i as usize] = (*c).aerodynamics;
        i += 1;
    }
    trajectory_calibrator_clear(&raw mut (*c).trajectory_calibrator);
    trajectory_calibrator_init(&raw mut (*c).trajectory_calibrator);
    (*c).trajectory_calibration = TrajectoryCalibrationModel {
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
        altitude_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        range_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        confidence: 0.05f64,
        accepted_samples: 0 as ::core::ffi::c_int,
    };
    guidance_command_init(&raw mut (*c).last_command);
    (*c).snapshot.adaptive_vehicle_profile = *p;
}
unsafe extern "C" fn stop_thread(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    let mut join: bool = (*c).thread_started;
    (*c).stop_thread = true_0 != 0;
    pthread_mutex_unlock(&raw mut (*c).mutex);
    if join {
        pthread_join((*c).thread, ::core::ptr::null_mut::<*mut ::core::ffi::c_void>());
    }
    pthread_mutex_lock(&raw mut (*c).mutex);
    (*c).thread_started = false_0 != 0;
    (*c).stop_thread = false_0 != 0;
    pthread_mutex_unlock(&raw mut (*c).mutex);
    stop_prediction_worker(c);
}
unsafe extern "C" fn disconnect_locked(
    mut c: *mut LandingController,
    mut should_publish: bool,
) {
    vessel_physics_init(&raw mut (*c).physics);
    if !(*c).flight_log.is_null() || !(*c).planner_log.is_null() {
        (*c).log_sequence = (*c).log_sequence.wrapping_add(1);
        (*c).snapshot.log_sequence = (*c).log_sequence;
        (*c).snapshot.wall_monotonic_seconds = monotonic_seconds();
        snprintf(
            &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"event=disconnect; safe_control_release=true\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        snprintf(
            &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"Disconnecting and releasing control.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        append_vehicle_log(
            c,
            &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
            true_0 != 0,
            false_0 != 0,
        );
        append_planner_log(
            c,
            &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
            true_0 != 0,
        );
    }
    close_log(c);
    if !(*c).session.is_null() {
        krpc_session_close((*c).session);
        (*c).session = ::core::ptr::null_mut::<KRPCSession>();
    }
    if (*c).has_plan {
        deorbit_plan_clear(&raw mut (*c).plan);
        (*c).has_plan = false_0 != 0;
    }
    (*c).plan_stale = false_0 != 0;
    (*c).has_latest = false_0 != 0;
    (*c).has_prediction_cache = false_0 != 0;
    (*c).cached_prediction_ut = -1e300f64;
    (*c).cached_prediction_sign = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    trajectory_init(&raw mut (*c).dynamic_trajectory);
    trajectory_clear(&raw mut (*c).stabilized_trajectory);
    trajectory_init(&raw mut (*c).stabilized_trajectory);
    guidance_machine_init(&raw mut (*c).guidance);
    glide_calibration_init(&raw mut (*c).glide);
    trajectory_calibrator_clear(&raw mut (*c).trajectory_calibrator);
    trajectory_calibrator_init(&raw mut (*c).trajectory_calibrator);
    (*c).trajectory_calibration = TrajectoryCalibrationModel {
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
        altitude_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        range_residual: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        confidence: 0.05f64,
        accepted_samples: 0 as ::core::ffi::c_int,
    };
    guidance_command_init(&raw mut (*c).last_command);
    (*c).has_last_tick_ut = false_0 != 0;
    (*c).has_last_tick_wall = false_0 != 0;
    (*c).time_warp_issued = false_0 != 0;
    (*c).last_trajectory_ut = -1e300f64;
    (*c).last_prediction_ut = -1e300f64;
    if should_publish {
        reset_snapshot(c, b"Disconnected\0" as *const u8 as *const ::core::ffi::c_char);
        publish(c);
    }
}
unsafe extern "C" fn actual_trajectory_update(
    mut c: *mut LandingController,
    mut t: *const Telemetry,
) {
    if (*t).ut - (*c).last_trajectory_ut
        < 1 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return;
    }
    (*c).last_trajectory_ut = (*t).ut;
    let mut p: TrajectoryPoint = TrajectoryPoint {
        ut: (*t).ut,
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        altitude: (*t).mean_altitude,
        speed: (*t).true_air_speed,
        phase: (*c).snapshot.phase,
        kind: TRAJ_ACTUAL,
    };
    if !trajectory_append(&raw mut (*c).snapshot.actual_trajectory, p) {
        return;
    }
    (*c).log_actual_trajectory_dirty = true_0 != 0;
    if (*c).snapshot.actual_trajectory.count > 3600 as size_t {
        let mut excess: size_t = (*c)
            .snapshot
            .actual_trajectory
            .count
            .wrapping_sub(3600 as size_t);
        memmove(
            (*c).snapshot.actual_trajectory.points as *mut ::core::ffi::c_void,
            (*c).snapshot.actual_trajectory.points.offset(excess as isize)
                as *const ::core::ffi::c_void,
            (3600 as size_t)
                .wrapping_mul(::core::mem::size_of::<TrajectoryPoint>() as size_t),
        );
        (*c).snapshot.actual_trajectory.count = 3600 as size_t;
    }
}
unsafe extern "C" fn prediction_blend(
    mut previous: ::core::ffi::c_double,
    mut raw: ::core::ffi::c_double,
    mut alpha: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if previous.is_finite() as i32 == 0 || raw.is_finite() as i32 == 0 {
        return raw;
    }
    return previous
        + (raw - previous)
            * clampd(
                alpha,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
}
unsafe extern "C" fn prediction_cache_store(
    mut c: *mut LandingController,
    mut pr: *const EntryPrediction,
) {
    (*c).has_prediction_cache = true_0 != 0;
    (*c).cached_predicted_miss_distance = (*pr).closest_distance;
    (*c).cached_predicted_taem_distance = (*pr).taem_distance;
    (*c).cached_predicted_taem_range_error = (*pr).taem_range_error;
    (*c).cached_predicted_entry_range = (*pr).entry_range;
    (*c).cached_predicted_entry_flight_path_angle = (*pr).entry_flight_path_angle;
    (*c).cached_predicted_taem_speed = (*pr).taem_speed;
    (*c).cached_predicted_taem_energy_error = (*pr).taem_energy_error;
    (*c).cached_predicted_peak_dynamic_pressure = (*pr).peak_dynamic_pressure;
    (*c).cached_predicted_peak_g_load = (*pr).peak_g_load;
    (*c).cached_physics_observed_seconds = (*pr).physics_observed_seconds;
    (*c).cached_physics_fallback_seconds = (*pr).physics_fallback_seconds;
    (*c).cached_predicted_s_turn_reversals = (*pr).s_turn_reversals
        as ::core::ffi::c_double;
    (*c).cached_predicted_shadow_guidance = (*pr).shadow_guidance_used;
    (*c).cached_predicted_terminal_policy_feasible = (*pr)
        .shadow_terminal_policy_feasible;
    (*c).cached_predicted_terminal_survivability_stress_score = (*pr)
        .terminal_survivability_stress_score;
    (*c).cached_predicted_physics_relative_uncertainty = (*pr)
        .physics_relative_uncertainty;
    (*c).cached_predicted_uncertainty_scenarios = (*pr).uncertainty_scenarios;
}
unsafe extern "C" fn prediction_cache_store_filtered(
    mut c: *mut LandingController,
    mut pr: *const EntryPrediction,
    mut alpha: ::core::ffi::c_double,
) {
    if !(*c).has_prediction_cache || alpha >= 0.999f64 {
        prediction_cache_store(c, pr);
        return;
    }
    (*c).cached_predicted_miss_distance = prediction_blend(
        (*c).cached_predicted_miss_distance,
        (*pr).closest_distance,
        alpha,
    );
    (*c).cached_predicted_taem_distance = prediction_blend(
        (*c).cached_predicted_taem_distance,
        (*pr).taem_distance,
        alpha,
    );
    (*c).cached_predicted_taem_range_error = prediction_blend(
        (*c).cached_predicted_taem_range_error,
        (*pr).taem_range_error,
        alpha,
    );
    (*c).cached_predicted_entry_range = prediction_blend(
        (*c).cached_predicted_entry_range,
        (*pr).entry_range,
        alpha,
    );
    (*c).cached_predicted_entry_flight_path_angle = prediction_blend(
        (*c).cached_predicted_entry_flight_path_angle,
        (*pr).entry_flight_path_angle,
        alpha,
    );
    (*c).cached_predicted_taem_speed = prediction_blend(
        (*c).cached_predicted_taem_speed,
        (*pr).taem_speed,
        alpha,
    );
    (*c).cached_predicted_taem_energy_error = prediction_blend(
        (*c).cached_predicted_taem_energy_error,
        (*pr).taem_energy_error,
        alpha,
    );
    (*c).cached_predicted_peak_dynamic_pressure = prediction_blend(
        (*c).cached_predicted_peak_dynamic_pressure,
        (*pr).peak_dynamic_pressure,
        alpha,
    );
    (*c).cached_predicted_peak_g_load = prediction_blend(
        (*c).cached_predicted_peak_g_load,
        (*pr).peak_g_load,
        alpha,
    );
    (*c).cached_physics_observed_seconds = (*pr).physics_observed_seconds;
    (*c).cached_physics_fallback_seconds = (*pr).physics_fallback_seconds;
    (*c).cached_predicted_s_turn_reversals = (*pr).s_turn_reversals
        as ::core::ffi::c_double;
    (*c).cached_predicted_shadow_guidance = (*pr).shadow_guidance_used;
    (*c).cached_predicted_terminal_policy_feasible = (*pr)
        .shadow_terminal_policy_feasible;
    (*c).cached_predicted_terminal_survivability_stress_score = prediction_blend(
        (*c).cached_predicted_terminal_survivability_stress_score,
        (*pr).terminal_survivability_stress_score,
        alpha,
    );
    (*c).cached_predicted_physics_relative_uncertainty = prediction_blend(
        (*c).cached_predicted_physics_relative_uncertainty,
        (*pr).physics_relative_uncertainty,
        alpha,
    );
    (*c).cached_predicted_uncertainty_scenarios = (*pr).uncertainty_scenarios;
}
unsafe extern "C" fn prediction_cache_apply(
    mut c: *const LandingController,
    mut t: *mut Telemetry,
    mut cfg: *const LandingConfiguration,
) {
    if !(*c).has_prediction_cache {
        return;
    }
    (*t).predicted_physics_observed_seconds = (*c).cached_physics_observed_seconds;
    (*t).predicted_physics_fallback_seconds = (*c).cached_physics_fallback_seconds;
    (*t).predicted_miss_distance = (*c).cached_predicted_miss_distance;
    (*t).predicted_taem_distance = (*c).cached_predicted_taem_distance;
    (*t).predicted_taem_range_error = (*c).cached_predicted_taem_range_error;
    (*t).predicted_entry_range = (*c).cached_predicted_entry_range;
    (*t).predicted_entry_flight_path_angle = (*c)
        .cached_predicted_entry_flight_path_angle;
    (*t).predicted_taem_speed = (*c).cached_predicted_taem_speed;
    (*t).predicted_taem_energy_error = (*c).cached_predicted_taem_energy_error;
    (*t).predicted_peak_dynamic_pressure = (*c).cached_predicted_peak_dynamic_pressure;
    (*t).predicted_peak_g_load = (*c).cached_predicted_peak_g_load;
    (*t).predicted_s_turn_reversals = (*c).cached_predicted_s_turn_reversals;
    (*t).predicted_shadow_guidance = (*c).cached_predicted_shadow_guidance;
    (*t).predicted_terminal_policy_feasible = (*c)
        .cached_predicted_terminal_policy_feasible;
    (*t).predicted_terminal_survivability_stress_score = (*c)
        .cached_predicted_terminal_survivability_stress_score;
    (*t).predicted_physics_relative_uncertainty = (*c)
        .cached_predicted_physics_relative_uncertainty;
    (*t).predicted_uncertainty_scenarios = (*c).cached_predicted_uncertainty_scenarios;
    (*t).predicted_raw_published_position_delta_30s = (*c)
        .cached_predicted_raw_published_position_delta_30s;
    (*t).predicted_raw_published_position_delta_60s = (*c)
        .cached_predicted_raw_published_position_delta_60s;
    (*t).predicted_raw_published_position_delta_120s = (*c)
        .cached_predicted_raw_published_position_delta_120s;
}
unsafe extern "C" fn dynamic_prediction(
    mut c: *mut LandingController,
    mut t: *mut Telemetry,
    mut state: *const VehicleState,
    mut cfg: *const LandingConfiguration,
) {
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    let mut terminal_shadow: bool = taem_exec_owns_vehicle(
        &raw mut (*c).guidance.taem_exec,
    );
    let mut postburn_verify: bool = (*c).has_plan as ::core::ffi::c_int != 0
        && (*c).guidance.deorbit_burn_completed as ::core::ffi::c_int != 0
        && !(*c).plan.achieved_state_verified
        && planet_atmospheric_density(p, (*t).mean_altitude) <= DBL_EPSILON;
    if (*c).guidance.phase as ::core::ffi::c_uint
        == PHASE_ATTITUDE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint
        || terminal_shadow as ::core::ffi::c_int != 0
        || (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_ENTRY_INTERFACE as ::core::ffi::c_int as ::core::ffi::c_uint
            && !postburn_verify
    {
        prediction_cache_apply(c, t, cfg);
        return;
    }
    let mut burn_cutoff_predict: bool = (*c).has_plan as ::core::ffi::c_int != 0
        && (*c).guidance.has_burn_command_started as ::core::ffi::c_int != 0
        && !(*c).guidance.deorbit_burn_completed
        && planet_atmospheric_density(p, (*t).mean_altitude) <= DBL_EPSILON;
    let mut sts_periapsis_closure: bool = (*cfg).guidance.taem_energy_law
        as ::core::ffi::c_int != 0
        && strcmp(
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 && burn_cutoff_predict as ::core::ffi::c_int != 0
        && (*c).plan.predicted_post_burn_periapsis_altitude
            > 10000 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).plan.predicted_post_burn_periapsis_altitude < (*p).atmosphere_depth
        && (*t).periapsis_altitude.is_finite() as i32 != 0
        && (*t).periapsis_altitude > -(*p).radius * 0.5f64;
    if sts_periapsis_closure {
        prediction_cache_apply(c, t, cfg);
        return;
    }
    if !(*c).guidance.automation_engaged && !postburn_verify {
        prediction_cache_apply(c, t, cfg);
        return;
    }
    let mut prediction_interval: ::core::ffi::c_double = if burn_cutoff_predict
        as ::core::ffi::c_int != 0
    {
        fmin((*cfg).guidance.prediction_interval, 0.75f64)
    } else {
        (*cfg).guidance.prediction_interval
    };
    if !postburn_verify && (*t).ut - (*c).last_prediction_ut < prediction_interval {
        prediction_cache_apply(c, t, cfg);
        return;
    }
    if planet_atmospheric_density(p, (*t).mean_altitude) <= DBL_EPSILON
        && !burn_cutoff_predict && !postburn_verify
    {
        prediction_cache_apply(c, t, cfg);
        return;
    }
    (*c).last_prediction_ut = (*t).ut;
    let mut signed_roll: ::core::ffi::c_double = norm_signed_deg(
        (*c).last_command.target_roll,
    );
    let mut sign: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut entry_state: bool = (*c).guidance.phase as ::core::ffi::c_uint
        == PHASE_ENTRY_INTERFACE as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_ATTITUDE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint;
    if (*c).guidance.automation_engaged as ::core::ffi::c_int != 0
        && entry_state as ::core::ffi::c_int != 0
        && fabs((*c).guidance.s_turn_sign) > DBL_EPSILON
    {
        sign = (if (*c).guidance.s_turn_sign
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        }) as ::core::ffi::c_double;
    } else if (*c).last_command.autopilot_engaged as ::core::ffi::c_int != 0
        && fabs(signed_roll) > DBL_EPSILON
    {
        sign = (if signed_roll >= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        }) as ::core::ffi::c_double;
    }
    if sign == 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).guidance.deorbit_burn_completed as ::core::ffi::c_int != 0
    {
        sign = (if (*c).guidance.s_turn_sign
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            1 as ::core::ffi::c_int
        } else {
            -(1 as ::core::ffi::c_int)
        }) as ::core::ffi::c_double;
    }
    let mut prediction_aero: AerodynamicModel = (*c).aerodynamics;
    let mut prediction_envelope: AerodynamicEnvelope = (*c).envelope;
    let mut initial_roll_rate: ::core::ffi::c_double = if (*t).roll_rate.is_finite()
        as i32 != 0
    {
        (*t).roll_rate
    } else {
        0.0f64
    };
    let mut initial_aoa_rate: ::core::ffi::c_double = if (*t).has_angle_of_attack_rate
        as ::core::ffi::c_int != 0 && (*t).angle_of_attack_rate.is_finite() as i32 != 0
    {
        (*t).angle_of_attack_rate
    } else {
        0.0f64
    };
    let mut leg_elapsed: ::core::ffi::c_double = guidance_entry_leg_elapsed(
        &raw mut (*c).guidance,
        (*t).ut,
    );
    (*c).forecast_entry_plan_identity_valid = false_0 != 0;
    (*c).forecast_entry_plan_id = 0 as uint64_t;
    (*c).forecast_entry_plan_version = 0 as uint64_t;
    let mut pr: EntryPrediction = predictor_simulate_entry_with_attitude(
        *state,
        p,
        prediction_aero,
        &raw mut prediction_envelope,
        &raw mut (*c).trajectory_calibration,
        &raw const (*cfg).vehicle,
        &raw const (*cfg).site,
        &raw const (*cfg).guidance,
        (*cfg).vehicle.maximum_bank_angle,
        (*t).roll,
        initial_roll_rate,
        (*t).angle_of_attack,
        initial_aoa_rate,
        sign,
        leg_elapsed,
        (if burn_cutoff_predict as ::core::ffi::c_int != 0
            || postburn_verify as ::core::ffi::c_int != 0
        {
            4500 as ::core::ffi::c_int
        } else {
            2500 as ::core::ffi::c_int
        }) as ::core::ffi::c_double,
        true_0 != 0,
    );
    trajectory_clear(&raw mut (*c).dynamic_trajectory);
    (*c).dynamic_trajectory = pr.trajectory;
    pr.trajectory = Trajectory {
        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
        count: 0,
        capacity: 0,
    };
    (*c).log_raw_plan_trajectory_dirty = true_0 != 0;
    let mut filter_dt: ::core::ffi::c_double = if (*c).has_prediction_cache
        as ::core::ffi::c_int != 0 && (*c).cached_prediction_ut > -1e200f64
        && (*t).ut > (*c).cached_prediction_ut
    {
        (*t).ut - (*c).cached_prediction_ut
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut entry_alt: ::core::ffi::c_double = (*p).atmosphere_depth
        - (*cfg).guidance.entry_interface_altitude_margin;
    let mut remaining: ::core::ffi::c_double = clampd(
        ((*t).mean_altitude - (*cfg).guidance.taem_interface_altitude)
            / fmax(
                entry_alt - (*cfg).guidance.taem_interface_altitude,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut tau: ::core::ffi::c_double = 1.25f64 + 2.75f64 * remaining;
    let mut side_changed: bool = (*c).has_prediction_cache as ::core::ffi::c_int != 0
        && fabs((*c).cached_prediction_sign) > 0.1f64 && fabs(sign) > 0.1f64
        && (*c).cached_prediction_sign * sign
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    if side_changed {
        tau = fmax(0.75f64, tau * 0.55f64);
    }
    let mut alpha: ::core::ffi::c_double = if filter_dt
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        1.0f64 - exp(-clampd(filter_dt, 0.01f64, 10.0f64) / tau)
    } else {
        1.0f64
    };
    alpha = clampd(alpha, 0.10f64, 1.0f64);
    let mut display_pr: EntryPrediction = pr;
    if !pr.reached_taem {
        display_pr.taem_distance = ::core::f32::NAN as ::core::ffi::c_double;
        display_pr.taem_range_error = ::core::f32::NAN as ::core::ffi::c_double;
        display_pr.taem_speed = ::core::f32::NAN as ::core::ffi::c_double;
        display_pr.taem_energy_error = entry_remaining_specific_energy(
            (*t).latitude,
            (*t).mean_altitude,
            (*t).true_air_speed,
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
    if postburn_verify as ::core::ffi::c_int != 0
        || terminal_shadow as ::core::ffi::c_int != 0
    {
        prediction_cache_store(c, &raw mut display_pr);
        alpha = 1.0f64;
    } else {
        prediction_cache_store_filtered(c, &raw mut display_pr, alpha);
    }
    (*c).cached_prediction_ut = (*t).ut;
    (*c).cached_prediction_sign = sign;
    prediction_cache_apply(c, t, cfg);
    trajectory_calibrator_set_forecast(
        &raw mut (*c).trajectory_calibrator,
        &raw mut (*c).dynamic_trajectory,
    );
    if postburn_verify as ::core::ffi::c_int != 0
        || terminal_shadow as ::core::ffi::c_int != 0
    {
        trajectory_clear(&raw mut (*c).stabilized_trajectory);
        trajectory_init(&raw mut (*c).stabilized_trajectory);
        trajectory_copy(
            &raw mut (*c).stabilized_trajectory,
            &raw mut (*c).dynamic_trajectory,
        );
    } else if !stabilize_forecast_trajectory(
        &raw mut (*c).stabilized_trajectory,
        &raw mut (*c).dynamic_trajectory,
        clampd(alpha * 1.25f64, 0.12f64, 1.0f64),
    ) {
        trajectory_copy(
            &raw mut (*c).stabilized_trajectory,
            &raw mut (*c).dynamic_trajectory,
        );
    }
    (*c).cached_predicted_raw_published_position_delta_30s = trajectory_position_delta_at(
        &raw mut (*c).dynamic_trajectory,
        &raw mut (*c).stabilized_trajectory,
        (*t).ut + 30.0f64,
        (*p).radius,
    );
    (*c).cached_predicted_raw_published_position_delta_60s = trajectory_position_delta_at(
        &raw mut (*c).dynamic_trajectory,
        &raw mut (*c).stabilized_trajectory,
        (*t).ut + 60.0f64,
        (*p).radius,
    );
    (*c).cached_predicted_raw_published_position_delta_120s = trajectory_position_delta_at(
        &raw mut (*c).dynamic_trajectory,
        &raw mut (*c).stabilized_trajectory,
        (*t).ut + 120.0f64,
        (*p).radius,
    );
    (*t).predicted_raw_published_position_delta_30s = (*c)
        .cached_predicted_raw_published_position_delta_30s;
    (*t).predicted_raw_published_position_delta_60s = (*c)
        .cached_predicted_raw_published_position_delta_60s;
    (*t).predicted_raw_published_position_delta_120s = (*c)
        .cached_predicted_raw_published_position_delta_120s;
    if (*c).has_plan {
        (*c).plan.predicted_closest_distance = (*c).cached_predicted_miss_distance;
        (*c).plan.predicted_taem_distance = (*c).cached_predicted_taem_distance;
        (*c).plan.predicted_taem_range_error = (*c).cached_predicted_taem_range_error;
        (*c).plan.predicted_entry_range = (*c).cached_predicted_entry_range;
        (*c).plan.predicted_entry_flight_path_angle = (*c)
            .cached_predicted_entry_flight_path_angle;
        if !trajectory_equal(
            &raw mut (*c).plan.trajectory,
            &raw mut (*c).stabilized_trajectory,
        ) {
            trajectory_copy(
                &raw mut (*c).plan.trajectory,
                &raw mut (*c).stabilized_trajectory,
            );
            (*c).log_plan_trajectory_dirty = true_0 != 0;
        }
        let mut peri: ::core::ffi::c_double = if (*c).plan.achieved_state_verified
            as ::core::ffi::c_int != 0
        {
            (*c).plan.achieved_post_burn_periapsis_altitude
        } else {
            (*c).plan.predicted_post_burn_periapsis_altitude
        };
        if burn_cutoff_predict {
            let mut cutoff_peri: ::core::ffi::c_double = predictor_postburn_periapsis(
                *state,
                p,
            );
            (*c).plan.live_cutoff_periapsis_altitude = cutoff_peri;
            (*c).plan.live_cutoff_closest_distance = pr.closest_distance;
            (*c).plan.live_cutoff_entry_flight_path_angle = pr.entry_flight_path_angle;
            (*c).plan.live_cutoff_capture_qualified = deorbit_live_cutoff_capture_qualified(
                &raw mut pr,
                &raw const (*cfg).site,
                &raw const (*cfg).vehicle,
                &raw const (*cfg).guidance,
                cutoff_peri,
            );
        } else if !(*c).guidance.has_burn_command_started {
            (*c).plan.live_cutoff_capture_qualified = false_0 != 0;
        }
        if postburn_verify {
            peri = predictor_postburn_periapsis(*state, p);
            (*c).plan.achieved_state_verified = true_0 != 0;
            (*c).plan.achieved_post_burn_periapsis_altitude = peri;
        }
        let mut live_capture: bool = deorbit_capture_qualified(
            &raw mut pr,
            &raw const (*cfg).site,
            &raw const (*cfg).vehicle,
            &raw const (*cfg).guidance,
            peri,
            (*t).mean_altitude > entry_alt,
        );
        if postburn_verify {
            (*c).plan.achieved_state_capture_qualified = live_capture;
        }
        (*c).plan.target_capture_achieved = (*c).plan.robustness_qualified
            as ::core::ffi::c_int != 0
            && (!(*c).plan.achieved_state_verified
                || (*c).plan.achieved_state_capture_qualified as ::core::ffi::c_int
                    != 0);
        let mut robust: ::core::ffi::c_double = (*c).plan.robustness_pass_fraction
            * (if (*c).plan.robustness_unsafe != 0 { 0.45f64 } else { 1.0f64 });
        let mut trajectory_term: ::core::ffi::c_double = if (*cfg)
            .calibration
            .enable_trajectory_calibration as ::core::ffi::c_int != 0
        {
            (*c).trajectory_calibration.confidence
        } else {
            (*c).planning_aerodynamics.confidence
        };
        (*c).plan.confidence = clampd(
            (*c).planning_aerodynamics.confidence * 0.25f64 + trajectory_term * 0.25f64
                + robust * 0.25f64
                + (if live_capture as ::core::ffi::c_int != 0 {
                    0.20f64
                } else {
                    0.05f64
                })
                + (if (*c).plan.achieved_state_verified as ::core::ffi::c_int != 0 {
                    0.05f64
                } else {
                    0 as ::core::ffi::c_int as ::core::ffi::c_double
                }),
            0.03f64,
            0.97f64,
        );
    }
    entry_prediction_clear(&raw mut pr);
}
unsafe extern "C" fn terminal_prediction_plan_copy(
    mut src: *const DeorbitPlan,
    mut dst: *mut DeorbitPlan,
) -> bool {
    if dst.is_null() {
        return false_0 != 0;
    }
    memset(
        dst as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
    trajectory_init(&raw mut (*dst).trajectory);
    if src.is_null() {
        return true_0 != 0;
    }
    *dst = *src;
    (*dst).trajectory = Trajectory {
        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
        count: 0,
        capacity: 0,
    };
    if trajectory_copy(&raw mut (*dst).trajectory, &raw const (*src).trajectory) {
        return true_0 != 0;
    }
    trajectory_clear(&raw mut (*dst).trajectory);
    memset(
        dst as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<DeorbitPlan>() as size_t,
    );
    trajectory_init(&raw mut (*dst).trajectory);
    return false_0 != 0;
}
unsafe extern "C" fn forecast_history_append(
    mut c: *mut LandingController,
    mut t: *const Telemetry,
    mut s: *const VehicleState,
) {
    let mut n: ::core::ffi::c_int = (*c).forecast_history_count;
    if n > 0 as ::core::ffi::c_int
        && (*s).ut
            < (*c).forecast_history[(n - 1 as ::core::ffi::c_int) as usize].state.ut
    {
        n = 0 as ::core::ffi::c_int;
    }
    if n > 0 as ::core::ffi::c_int
        && (*s).ut
            - (*c).forecast_history[(n - 1 as ::core::ffi::c_int) as usize].state.ut
            < 1.0f64
    {
        return;
    }
    if n == FORECAST_HISTORY_MAX {
        memmove(
            &raw mut (*c).forecast_history as *mut ForecastHistorySample
                as *mut ::core::ffi::c_void,
            (&raw mut (*c).forecast_history as *mut ForecastHistorySample)
                .offset(1 as ::core::ffi::c_int as isize) as *const ::core::ffi::c_void,
            ((n - 1 as ::core::ffi::c_int) as size_t)
                .wrapping_mul(::core::mem::size_of::<ForecastHistorySample>() as size_t),
        );
        n -= 1;
    }
    (*c).forecast_history[n as usize] = ForecastHistorySample {
        state: *s,
        aoa: (*t).angle_of_attack,
        bank: (*t).roll,
        q: (*t).dynamic_pressure,
    };
    (*c).forecast_history_count = n + 1 as ::core::ffi::c_int;
}
unsafe extern "C" fn achieved_state_warning(
    mut c: *mut LandingController,
    mut buf: *mut ::core::ffi::c_char,
    mut n: size_t,
) -> *const ::core::ffi::c_char {
    if !(*c).has_plan || !(*c).plan.achieved_state_verified
        || (*c).plan.achieved_state_capture_qualified as ::core::ffi::c_int != 0
    {
        return ::core::ptr::null::<::core::ffi::c_char>();
    }
    snprintf(
        buf,
        n,
        b"Post-burn achieved-state verification is outside the planned entry corridor (actual Pe %.1f km). Entry guidance is continuing from live state; monitor the trajectory and be ready to take manual control.\0"
            as *const u8 as *const ::core::ffi::c_char,
        (*c).plan.achieved_post_burn_periapsis_altitude
            / 1000 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    return buf;
}
unsafe extern "C" fn environment_warning(
    mut c: *mut LandingController,
    mut t: *const Telemetry,
    mut buf: *mut ::core::ffi::c_char,
    mut n: size_t,
) -> *const ::core::ffi::c_char {
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    if strcasecmp(
        &raw const (*p).name as *const ::core::ffi::c_char,
        b"Kerbin\0" as *const u8 as *const ::core::ffi::c_char,
    ) != 0
    {
        snprintf(
            buf,
            n,
            b"The active planet is %s; guidance is using its live atmosphere/gravity model rather than Kerbin-specific geometric thresholds.\0"
                as *const u8 as *const ::core::ffi::c_char,
            &raw const (*p).name as *const ::core::ffi::c_char,
        );
        return buf;
    }
    if (*t).physics_model_residual_confidence > 0.0f64
        && (*t).physics_model_residual > (*t).physics_certified_uncertainty
    {
        snprintf(
            buf,
            n,
            b"Live aerodynamic response is outside the certified prior-flight model uncertainty (residual %.0f%%).\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*t).physics_model_residual * 100.0f64,
        );
        return buf;
    }
    if (*t).stall_fraction_is_measured as ::core::ffi::c_int != 0
        && (*t).stall_fraction >= 1.0f64
    {
        snprintf(
            buf,
            n,
            b"The measured aerodynamic state is at or beyond stall.\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return buf;
    }
    return ::core::ptr::null::<::core::ffi::c_char>();
}
unsafe extern "C" fn should_time_warp(
    mut c: *const LandingController,
    mut r: *const GuidanceResult,
    mut t: *const Telemetry,
) -> bool {
    if !(*c).session.is_null()
        && krpc_session_is_simulator((*c).session) as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    if (*c).time_warp_issued as ::core::ffi::c_int != 0
        || !(*c).configuration.guidance.use_time_warp
        || (*r).phase as ::core::ffi::c_uint
            != PHASE_COAST as ::core::ffi::c_int as ::core::ffi::c_uint || !(*c).has_plan
        || !(*c).guidance.automation_engaged
        || (*c).guidance.paused as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    let mut burn_start: ::core::ffi::c_double = (*c).plan.burn_ut
        - (*c).plan.estimated_burn_duration * 0.5f64;
    let mut lead: ::core::ffi::c_double = fmax(
        0.0f64,
        (*c).configuration.guidance.minimum_planning_lead_time,
    );
    let mut target: ::core::ffi::c_double = burn_start - lead;
    return target > (*t).ut + lead;
}
unsafe extern "C" fn publish_warp_progress(
    mut context: *mut ::core::ffi::c_void,
    mut ut: ::core::ffi::c_double,
    mut target: ::core::ffi::c_double,
    mut factor: ::core::ffi::c_int,
    mut rate: ::core::ffi::c_double,
) {
    let mut c: *mut LandingController = context as *mut LandingController;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Coast warp: UT %.3f, %.1f s to target, factor %d (%.1fx). Flight telemetry resumes after warp.\0"
            as *const u8 as *const ::core::ffi::c_char,
        ut,
        fmax(0.0f64, target - ut),
        factor,
        rate,
    );
    publish(c);
}
unsafe extern "C" fn perform_time_warp(
    mut c: *mut LandingController,
    mut t: *const Telemetry,
    mut err: *mut ::core::ffi::c_char,
    mut err_size: size_t,
) -> bool {
    let mut burn_start: ::core::ffi::c_double = (*c).plan.burn_ut
        - (*c).plan.estimated_burn_duration * 0.5f64;
    let mut lead: ::core::ffi::c_double = fmax(
        0.0f64,
        (*c).configuration.guidance.minimum_planning_lead_time,
    );
    let mut target: ::core::ffi::c_double = burn_start - lead;
    if target <= (*t).ut + lead {
        (*c).time_warp_issued = true_0 != 0;
        return true_0 != 0;
    }
    krpc_neutralize((*c).session);
    (*c).time_warp_issued = true_0 != 0;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Time-warping to %.0f seconds before the planned deorbit burn for attitude acquisition.\0"
            as *const u8 as *const ::core::ffi::c_char,
        lead,
    );
    let mut reason: [::core::ffi::c_char; 192] = [0; 192];
    snprintf(
        &raw mut reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"event=time-warp-start; from_ut=%.9g; target_ut=%.9g\0" as *const u8
            as *const ::core::ffi::c_char,
        (*t).ut,
        target,
    );
    log_state_event(c, &raw mut reason as *mut ::core::ffi::c_char, true_0 != 0);
    publish(c);
    if !krpc_warp_with_progress(
        (*c).session,
        target,
        Some(
            publish_warp_progress
                as unsafe extern "C" fn(
                    *mut ::core::ffi::c_void,
                    ::core::ffi::c_double,
                    ::core::ffi::c_double,
                    ::core::ffi::c_int,
                    ::core::ffi::c_double,
                ) -> (),
        ),
        c as *mut ::core::ffi::c_void,
        err,
        err_size,
    ) {
        return false_0 != 0;
    }
    (*c).has_last_tick_ut = false_0 != 0;
    (*c).has_last_tick_wall = false_0 != 0;
    (*c).last_prediction_ut = -1e300f64;
    (*c).last_trajectory_ut = -1e300f64;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Time warp complete. Reacquiring live state before burn setup.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"event=time-warp-complete; target_ut=%.9g\0" as *const u8
            as *const ::core::ffi::c_char,
        target,
    );
    log_state_event(c, &raw mut reason as *mut ::core::ffi::c_char, true_0 != 0);
    publish(c);
    return true_0 != 0;
}
unsafe extern "C" fn apply_command_logged(
    mut c: *mut LandingController,
    mut cmd: *const GuidanceCommand,
    mut group: ::core::ffi::c_uint,
    mut phase: GuidancePhase,
    mut status: *const ::core::ffi::c_char,
    mut warning: *const ::core::ffi::c_char,
    mut reference: *const Trajectory,
    mut out: *mut KRPCApplyResult,
    mut started: *mut ::core::ffi::c_double,
    mut finished: *mut ::core::ffi::c_double,
    mut err: *mut ::core::ffi::c_char,
    mut err_size: size_t,
) -> bool {
    *started = monotonic_seconds();
    let mut predicted: *const Trajectory = if phase as ::core::ffi::c_uint
        == PHASE_ENTRY_INTERFACE as ::core::ffi::c_int as ::core::ffi::c_uint
        || phase as ::core::ffi::c_uint
            == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        || phase as ::core::ffi::c_uint
            == PHASE_ATTITUDE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        &raw mut (*c).stabilized_trajectory
    } else {
        ::core::ptr::null_mut::<Trajectory>()
    };
    let mut ok: bool = krpc_apply(
        (*c).session,
        cmd,
        group,
        phase_string(phase),
        status,
        warning,
        predicted,
        reference,
        out,
        err,
        err_size,
    );
    *finished = monotonic_seconds();
    return ok;
}
pub const WARP_ENTRY_ALTITUDE_M: ::core::ffi::c_double = 72000.0f64;
pub const WARP_STEP_HOLD_S: ::core::ffi::c_double = 3.0f64;
pub const WARP_AOA_ERROR_DEG: ::core::ffi::c_double = 5.0f64;
pub const WARP_BANK_ERROR_DEG: ::core::ffi::c_double = 15.0f64;
pub const WARP_BODY_RATE_DEG_S: ::core::ffi::c_double = 8.0f64;
pub const WARP_WOBBLE_REVERSALS: ::core::ffi::c_int = 3 as ::core::ffi::c_int;
pub const WARP_WOBBLE_WINDOW_S: ::core::ffi::c_double = 6.0f64;
unsafe extern "C" fn warp_allowed_factor(
    mut phase: GuidancePhase,
    mut t: *const Telemetry,
) -> ::core::ffi::c_int {
    if phase as ::core::ffi::c_uint
        == PHASE_ENTRY_INTERFACE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return if (*t).mean_altitude > WARP_ENTRY_ALTITUDE_M {
            3 as ::core::ffi::c_int
        } else {
            1 as ::core::ffi::c_int
        };
    }
    if phase as ::core::ffi::c_uint
        == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return if (*t).mean_altitude > WARP_ENTRY_ALTITUDE_M {
            3 as ::core::ffi::c_int
        } else {
            1 as ::core::ffi::c_int
        };
    }
    return 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn govern_physics_warp(
    mut c: *mut LandingController,
    mut t: *const Telemetry,
    mut r: *const GuidanceResult,
) {
    if !(*t).has_warp || (*t).warp_rails as ::core::ffi::c_int != 0
        || krpc_session_is_simulator((*c).session) as ::core::ffi::c_int != 0
    {
        return;
    }
    let mut factor: ::core::ffi::c_int = (*t).physics_warp_factor;
    if factor <= 0 as ::core::ffi::c_int {
        (*c).warp_roll_reversals = 0 as ::core::ffi::c_int;
        return;
    }
    let mut cmd: *const GuidanceCommand = &raw const (*r).command;
    let mut aoa_error: ::core::ffi::c_double = if (*cmd).has_target_aoa
        as ::core::ffi::c_int != 0
    {
        fabs((*t).angle_of_attack - (*cmd).target_aoa)
    } else {
        0.0f64
    };
    let mut bank_error: ::core::ffi::c_double = fabs(
        fmod((*t).roll - (*cmd).target_roll + 540.0f64, 360.0f64) - 180.0f64,
    );
    let mut rate: ::core::ffi::c_double = 0.0f64;
    if (*t).has_body_pitch_rate {
        rate = fmax(rate, fabs((*t).body_pitch_rate));
    }
    if (*t).has_body_roll_rate {
        rate = fmax(rate, fabs((*t).body_roll_rate));
    }
    if (*t).has_body_yaw_rate {
        rate = fmax(rate, fabs((*t).body_yaw_rate));
    }
    if (*t).has_body_roll_rate as ::core::ffi::c_int != 0
        && fabs((*t).body_roll_rate) > 2.0f64
    {
        if (*c).warp_last_roll_rate * (*t).body_roll_rate < 0.0f64 {
            if (*t).ut - (*c).warp_reversal_window_ut > WARP_WOBBLE_WINDOW_S {
                (*c).warp_reversal_window_ut = (*t).ut;
                (*c).warp_roll_reversals = 0 as ::core::ffi::c_int;
            }
            (*c).warp_roll_reversals += 1;
        }
        (*c).warp_last_roll_rate = (*t).body_roll_rate;
    }
    let mut allowed: ::core::ffi::c_int = warp_allowed_factor((*r).phase, t);
    let mut why: *const ::core::ffi::c_char = ::core::ptr::null::<::core::ffi::c_char>();
    if factor > allowed {
        why = if allowed != 0 {
            b"phase/altitude limit\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"phase does not allow warp\0" as *const u8 as *const ::core::ffi::c_char
        };
    } else if aoa_error > WARP_AOA_ERROR_DEG {
        why = b"AoA tracking error\0" as *const u8 as *const ::core::ffi::c_char;
    } else if bank_error > WARP_BANK_ERROR_DEG {
        why = b"bank tracking error\0" as *const u8 as *const ::core::ffi::c_char;
    } else if rate > WARP_BODY_RATE_DEG_S {
        why = b"body rate\0" as *const u8 as *const ::core::ffi::c_char;
    } else if (*c).warp_roll_reversals >= WARP_WOBBLE_REVERSALS {
        why = b"roll wobble\0" as *const u8 as *const ::core::ffi::c_char;
    }
    if why.is_null() {
        return;
    }
    let mut next: ::core::ffi::c_int = if allowed == 0 as ::core::ffi::c_int {
        0 as ::core::ffi::c_int
    } else {
        factor - 1 as ::core::ffi::c_int
    };
    if allowed > 0 as ::core::ffi::c_int
        && (*t).ut - (*c).warp_step_ut < WARP_STEP_HOLD_S
    {
        return;
    }
    let mut err: [::core::ffi::c_char; 256] = [
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
    if !krpc_set_physics_warp(
        (*c).session,
        next,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
    ) {
        return;
    }
    (*c).warp_step_ut = (*t).ut;
    (*c).warp_roll_reversals = 0 as ::core::ffi::c_int;
    let mut reason: [::core::ffi::c_char; 256] = [0; 256];
    snprintf(
        &raw mut reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"event=physics-warp-reduced; from=%dx; to=%dx; reason=%s; aoa_error=%.2f; bank_error=%.2f; body_rate=%.2f; altitude=%.0f\0"
            as *const u8 as *const ::core::ffi::c_char,
        factor + 1 as ::core::ffi::c_int,
        next + 1 as ::core::ffi::c_int,
        why,
        aoa_error,
        bank_error,
        rate,
        (*t).mean_altitude,
    );
    log_state_event(c, &raw mut reason as *mut ::core::ffi::c_char, true_0 != 0);
    snprintf(
        &raw mut (*c).snapshot.warning_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Time warp reduced to %dx (%s).\0" as *const u8 as *const ::core::ffi::c_char,
        next + 1 as ::core::ffi::c_int,
        why,
    );
}
unsafe extern "C" fn shut_down_burn_engines(mut c: *mut LandingController) {
    if (*c).burn_engines_shut_down as ::core::ffi::c_int != 0
        || !(*c).guidance.deorbit_burn_completed
        || !(*c).guidance.has_burn_command_started
    {
        return;
    }
    let mut err: [::core::ffi::c_char; 256] = [
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
    if !krpc_shutdown_burn_engines(
        (*c).session,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
    ) {
        let mut reason: [::core::ffi::c_char; 384] = [0; 384];
        snprintf(
            &raw mut reason as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"event=oms-shutdown-failed; error=%s\0" as *const u8
                as *const ::core::ffi::c_char,
            &raw mut err as *mut ::core::ffi::c_char,
        );
        log_state_event(c, &raw mut reason as *mut ::core::ffi::c_char, true_0 != 0);
        return;
    }
    (*c).burn_engines_shut_down = true_0 != 0;
    log_state_event(
        c,
        b"event=oms-shutdown; reason=deorbit-burn-complete\0" as *const u8
            as *const ::core::ffi::c_char,
        true_0 != 0,
    );
}
unsafe extern "C" fn tick_locked(mut c: *mut LandingController) {
    let mut read_budget: KRPCTransportBudget = KRPCTransportBudget {
        read_calls: 0,
        read_wire_requests: 0,
        apply_calls: 0,
        apply_wire_requests: 0,
        total_calls: 0,
        total_wire_requests: 0,
        read_seconds: 0.,
        apply_seconds: 0.,
    };
    let mut dt: ::core::ffi::c_double = 0.;
    let mut compute: ::core::ffi::c_double = 0.;
    let mut was_cal: bool = false;
    let mut sampling: bool = false;
    let mut finished: bool = false;
    let mut progress: ::core::ffi::c_double = 0.;
    let mut target_aoa: ::core::ffi::c_double = 0.;
    let mut glide: GuidanceResult = GuidanceResult {
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
    let mut nominal_learning: bool = false;
    let mut cal_snap: CalibrationSnapshot = CalibrationSnapshot {
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
    };
    let mut shadow_current: AerodynamicModel = AerodynamicModel {
        lift_to_drag: 0.,
        ballistic_coefficient: 0.,
        confidence: 0.,
    };
    let mut shadow_planning: AerodynamicModel = AerodynamicModel {
        lift_to_drag: 0.,
        ballistic_coefficient: 0.,
        confidence: 0.,
    };
    let mut shadow_envelope: AerodynamicEnvelope = AerodynamicEnvelope {
        regimes: [AerodynamicModel {
            lift_to_drag: 0.,
            ballistic_coefficient: 0.,
            confidence: 0.,
        }; 4],
    };
    let mut planet: *const PlanetModel = ::core::ptr::null::<PlanetModel>();
    let mut using_glide: bool = false;
    let mut r: GuidanceResult = GuidanceResult {
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
    let mut cmd: GuidanceCommand = GuidanceCommand {
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
    let mut hud_status: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut hud_warning: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut native_mm305_route: bool = false;
    let mut native_route_remaining: ::core::ffi::c_double = 0.;
    let mut native_route_station: ::core::ffi::c_double = 0.;
    let mut native_route_progress: ::core::ffi::c_double = 0.;
    let mut lift_accel: ::core::ffi::c_double = 0.;
    let mut bank_for_rate: ::core::ffi::c_double = 0.;
    let mut warning: *const ::core::ffi::c_char = ::core::ptr::null::<
        ::core::ffi::c_char,
    >();
    let mut current_block: u64;
    if (*c).session.is_null() {
        return;
    }
    let mut err: [::core::ffi::c_char; 768] = [
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
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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
    let mut wall_start: ::core::ffi::c_double = monotonic_seconds();
    let mut cfg: LandingConfiguration = effective_config(c);
    let mut tele_start: ::core::ffi::c_double = monotonic_seconds();
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
    telemetry_init(&raw mut t);
    let mut state: VehicleState = VehicleState {
        ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        position: Vector3 { x: 0., y: 0., z: 0. },
        velocity: Vector3 { x: 0., y: 0., z: 0. },
        mass: 0.,
    };
    let mut have_telemetry: bool = false_0 != 0;
    let mut apply_attempted: bool = false_0 != 0;
    let mut apply_succeeded: bool = false_0 != 0;
    let mut applied: KRPCApplyResult = KRPCApplyResult {
        applied: 0 as ::core::ffi::c_int != 0,
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
    };
    let mut apply_started: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut apply_finished: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut command_computed: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    if read_live_sample(
        c,
        &raw mut cfg,
        &raw mut t,
        &raw mut state,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
    ) {
        have_telemetry = true_0 != 0;
        read_budget = krpc_session_transport_budget((*c).session);
        t.has_rpc_budget = true_0 != 0;
        t.rpc_read_calls = read_budget.read_calls;
        t.rpc_read_wire_requests = read_budget.read_wire_requests;
        t.rpc_total_calls = read_budget.total_calls;
        t.rpc_total_wire_requests = read_budget.total_wire_requests;
        t.has_telemetry_latency = true_0 != 0;
        t.telemetry_latency_ms = (monotonic_seconds() - tele_start)
            * 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
        dt = if (*c).has_last_tick_ut as ::core::ffi::c_int != 0 {
            clampd(
                t.ut - (*c).last_tick_ut,
                0.01f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            )
        } else {
            0.1f64
        };
        (*c).last_tick_ut = t.ut;
        (*c).has_last_tick_ut = true_0 != 0;
        if (*c).has_last_tick_wall {
            t.has_loop_wall_delta = true_0 != 0;
            t.loop_wall_delta_ms = (wall_start - (*c).last_tick_wall)
                * 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
        }
        (*c).last_tick_wall = wall_start;
        (*c).has_last_tick_wall = true_0 != 0;
        compute = monotonic_seconds();
        (*c).tick_sequence = (*c).tick_sequence.wrapping_add(1);
        (*c).snapshot.tick_sequence = (*c).tick_sequence;
        (*c).prediction_lineage_discontinuity = false_0 != 0;
        was_cal = (*c).glide.active;
        sampling = false_0 != 0;
        finished = false_0 != 0;
        progress = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        target_aoa = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        glide = glide_calibration_update(
            &raw mut (*c).glide,
            &raw mut t,
            &raw mut cfg.vehicle,
            &raw mut (*c).configuration.calibration,
            &raw mut sampling,
            &raw mut finished,
            &raw mut progress,
            &raw mut target_aoa,
        );
        nominal_learning = !telemetry_warped(&raw mut t) && !(*c).guidance.aborted
            && !(*c).guidance.attitude_recovery
            && (*c).snapshot.phase as ::core::ffi::c_uint
                != PHASE_ABORT as ::core::ffi::c_int as ::core::ffi::c_uint
            && (*c).snapshot.phase as ::core::ffi::c_uint
                != PHASE_FAULT as ::core::ffi::c_int as ::core::ffi::c_uint
            && (*c).snapshot.phase as ::core::ffi::c_uint
                != PHASE_ATTITUDE_RECOVERY as ::core::ffi::c_int as ::core::ffi::c_uint;
        cal_snap = CalibrationSnapshot {
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
        };
        shadow_current = AerodynamicModel {
            lift_to_drag: 0.,
            ballistic_coefficient: 0.,
            confidence: 0.,
        };
        shadow_planning = AerodynamicModel {
            lift_to_drag: 0.,
            ballistic_coefficient: 0.,
            confidence: 0.,
        };
        shadow_envelope = AerodynamicEnvelope {
            regimes: [AerodynamicModel {
                lift_to_drag: 0.,
                ballistic_coefficient: 0.,
                confidence: 0.,
            }; 4],
        };
        adaptive_calibrator_update(
            &raw mut (*c).adaptive,
            &raw mut t,
            &raw mut (*c).configuration.calibration,
            &raw mut (*c).configuration.vehicle,
            &raw mut (*c).last_command,
            dt,
            nominal_learning,
            (*c).glide.active,
            sampling,
            (*c).glide.state,
            progress,
            target_aoa,
            if (*c).glide.state as ::core::ffi::c_uint
                == CAL_IDLE as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                ::core::ptr::null_mut::<::core::ffi::c_char>()
            } else {
                &raw mut glide.status as *mut ::core::ffi::c_char
            },
            if glide.has_warning as ::core::ffi::c_int != 0 {
                &raw mut glide.warning as *mut ::core::ffi::c_char
            } else {
                ::core::ptr::null_mut::<::core::ffi::c_char>()
            },
            &raw mut shadow_current,
            &raw mut shadow_planning,
            &raw mut shadow_envelope,
            &raw mut (*c).adaptive_profile,
            &raw mut cal_snap,
        );
        if !(*c).glide.active {
            snprintf(
                &raw mut cal_snap.status as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"Shadow flight-test reconstruction: certified prior-flight aerodynamics remain frozen for this session.\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        (*c).aerodynamics = envelope_at_controller(&raw mut (*c).envelope, t.mach);
        (*c).trajectory_calibration = trajectory_calibrator_update(
            &raw mut (*c).trajectory_calibrator,
            &raw mut t,
            &raw mut state,
            krpc_session_planet((*c).session),
            &raw mut cfg.site,
            &raw mut (*c).envelope,
            &raw mut cfg.vehicle,
            &raw mut (*c).last_command,
            &raw mut (*c).configuration.calibration,
            nominal_learning,
            dt,
        );
        (*c).trajectory_calibration.physics = &raw mut (*c).physics;
        t.estimated_lift_to_drag = (*c).aerodynamics.lift_to_drag;
        t.estimated_ballistic_coefficient = (*c).aerodynamics.ballistic_coefficient;
        t.aerodynamic_confidence = (*c).aerodynamics.confidence;
        t.calibrated_best_glide_angle_of_attack = cal_snap.best_glide_angle_of_attack;
        t.calibrated_stall_speed = cal_snap.estimated_stall_speed;
        t.trajectory_density_scale = (*c).trajectory_calibration.density_scale;
        t.trajectory_drag_scale = (*c).trajectory_calibration.drag_scale;
        t.trajectory_lift_scale = (*c).trajectory_calibration.lift_scale;
        t.bank_effectiveness = (*c).trajectory_calibration.bank_effectiveness;
        t.trajectory_calibration_confidence = (*c).trajectory_calibration.confidence;
        t.trajectory_altitude_residual = (*c).trajectory_calibration.altitude_residual;
        t.trajectory_speed_residual = (*c).trajectory_calibration.speed_residual;
        t.trajectory_range_residual = (*c).trajectory_calibration.range_residual;
        t.predicted_miss_distance = if (*c).has_plan as ::core::ffi::c_int != 0 {
            (*c).plan.predicted_closest_distance
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        t.predicted_taem_distance = if (*c).has_plan as ::core::ffi::c_int != 0 {
            (*c).plan.predicted_taem_distance
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        t.predicted_taem_range_error = if (*c).has_plan as ::core::ffi::c_int != 0 {
            (*c).plan.predicted_taem_range_error
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        planet = krpc_session_planet((*c).session);
        t.energy_excess_range = ::core::f32::NAN as ::core::ffi::c_double;
        (*c).guidance.entry_planning_deferred = true_0 != 0;
        guidance_set_entry_predictor_models(
            &raw mut (*c).guidance,
            &raw mut (*c).envelope,
            &raw mut (*c).trajectory_calibration,
        );
        if (*c).has_plan as ::core::ffi::c_int != 0 && !(*c).glide.active
            && (*c).guidance.automation_engaged as ::core::ffi::c_int != 0
            && !(*c).guidance.has_burn_command_started
            && !(*c).guidance.deorbit_burn_completed
            && !deorbit_plan_preburn_state_compatible(
                &raw mut (*c).plan,
                t.mass,
                t.available_thrust,
                &raw mut cfg.guidance,
            )
        {
            let mut mass_delta: ::core::ffi::c_double = if (*c).plan.planning_mass
                > 1 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                fabs(t.mass - (*c).plan.planning_mass) / (*c).plan.planning_mass
            } else {
                0 as ::core::ffi::c_int as ::core::ffi::c_double
            };
            let mut thrust_delta: ::core::ffi::c_double = if (*c)
                .plan
                .planning_available_thrust
                > 1 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                fabs(t.available_thrust - (*c).plan.planning_available_thrust)
                    / (*c).plan.planning_available_thrust
            } else {
                0 as ::core::ffi::c_int as ::core::ffi::c_double
            };
            (*c).plan_stale = true_0 != 0;
            if mass_delta > cfg.guidance.deorbit_mass_uncertainty_fraction + 1e-9f64 {
                snprintf(
                    &raw mut (*c).guidance.abort_reason as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                    b"Deorbit plan invalidated before burn: live vessel mass changed from %.0f kg to %.0f kg (%.1f%%), outside the certified +/-%.1f%% planning envelope. Replan from the current live state.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*c).plan.planning_mass,
                    t.mass,
                    mass_delta * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
                    cfg.guidance.deorbit_mass_uncertainty_fraction
                        * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
                );
            } else {
                snprintf(
                    &raw mut (*c).guidance.abort_reason as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                    b"Deorbit plan invalidated before burn: available thrust changed from %.0f N to %.0f N (%.1f%%), outside the certified +/-%.1f%% planning envelope. Replan from the current live state.\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    (*c).plan.planning_available_thrust,
                    t.available_thrust,
                    thrust_delta * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
                    cfg.guidance.deorbit_thrust_uncertainty_fraction
                        * 100 as ::core::ffi::c_int as ::core::ffi::c_double,
                );
            }
            let mut drift_reason: [::core::ffi::c_char; 256] = [0; 256];
            snprintf(
                &raw mut drift_reason as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
                b"event=plan-invalidated; reason=preburn-state-drift; mass=%.9g; planning-mass=%.9g; thrust=%.9g; planning-thrust=%.9g\0"
                    as *const u8 as *const ::core::ffi::c_char,
                t.mass,
                (*c).plan.planning_mass,
                t.available_thrust,
                (*c).plan.planning_available_thrust,
            );
            log_state_event(
                c,
                &raw mut drift_reason as *mut ::core::ffi::c_char,
                true_0 != 0,
            );
            guidance_abort(&raw mut (*c).guidance);
        }
        actual_trajectory_update(c, &raw mut t);
        dynamic_prediction(c, &raw mut t, &raw mut state, &raw mut cfg);
        (*c).latest_telemetry = t;
        (*c).latest_state = state;
        (*c).has_latest = true_0 != 0;
        forecast_history_append(c, &raw mut t, &raw mut state);
        using_glide = (*c).glide.active;
        (*c).guidance.mm305_async_planning = (*c).prediction_thread_started
            as ::core::ffi::c_int != 0
            && (*c).terminal_model_valid as ::core::ffi::c_int != 0;
        r = if using_glide as ::core::ffi::c_int != 0 {
            glide
        } else {
            guidance_update_with_terminal_model(
                &raw mut (*c).guidance,
                &raw mut t,
                &raw mut state,
                if (*c).has_plan as ::core::ffi::c_int != 0 {
                    &raw mut (*c).plan
                } else {
                    ::core::ptr::null_mut::<DeorbitPlan>()
                },
                planet,
                (*c).aerodynamics,
                &raw mut cfg,
                if (*c).terminal_model_valid as ::core::ffi::c_int != 0 {
                    &raw mut (*c).terminal_model
                } else {
                    ::core::ptr::null_mut::<TerminalModel>()
                },
            )
        };
        if using_glide {
            glide.reference = Trajectory {
                points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                count: 0,
                capacity: 0,
            };
        }
        if !using_glide
            && r.phase as ::core::ffi::c_uint
                == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
            && (*c).guidance.entry_feedback.valid as ::core::ffi::c_int != 0
            && (*c).guidance.entry_feedback.energy_margin.is_finite() as i32 != 0
        {
            t.energy_excess_range = (*c).guidance.entry_feedback.energy_margin;
        }
        t.has_guidance_compute = true_0 != 0;
        t.guidance_compute_ms = (monotonic_seconds() - compute)
            * 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
        command_computed = monotonic_seconds();
        if should_time_warp(c, &raw mut r, &raw mut t) {
            if !perform_time_warp(
                c,
                &raw mut t,
                &raw mut err as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
            ) {
                guidance_result_clear(&raw mut r);
                guidance_result_clear(&raw mut glide);
            } else {
                guidance_result_clear(&raw mut r);
                guidance_result_clear(&raw mut glide);
                return;
            }
        } else {
            cmd = r.command;
            (*c).last_command = cmd;
            hud_status = if using_glide as ::core::ffi::c_int != 0 {
                &raw mut glide.status as *mut ::core::ffi::c_char
            } else {
                &raw mut r.status as *mut ::core::ffi::c_char
            };
            hud_warning = if if using_glide as ::core::ffi::c_int != 0 {
                glide.has_warning as ::core::ffi::c_int
            } else {
                r.has_warning as ::core::ffi::c_int
            } != 0
            {
                (if using_glide as ::core::ffi::c_int != 0 {
                    &raw mut glide.warning as *mut ::core::ffi::c_char
                } else {
                    &raw mut r.warning as *mut ::core::ffi::c_char
                }) as *const ::core::ffi::c_char
            } else {
                b"\0" as *const u8 as *const ::core::ffi::c_char
            };
            if cmd.autopilot_engaged as ::core::ffi::c_int != 0
                && !cmd.use_inertial_direction
            {
                t.has_command_heading_error = true_0 != 0;
                t.has_command_roll_error = t.has_command_heading_error;
                t.has_command_pitch_error = t.has_command_roll_error;
                t.command_pitch_error = if cmd.has_target_aoa as ::core::ffi::c_int != 0
                {
                    cmd.target_aoa - t.angle_of_attack
                } else {
                    cmd.target_pitch - t.pitch
                };
                t.command_roll_error = norm_signed_deg(cmd.target_roll - t.roll);
                t.command_heading_error = norm_signed_deg(
                    cmd.target_heading - t.heading,
                );
            }
            if using_glide {
                apply_attempted = true_0 != 0;
                if !apply_command_logged(
                    c,
                    &raw mut cmd,
                    cfg.vehicle.airbrake_action_group,
                    PHASE_CALIBRATION,
                    hud_status,
                    hud_warning,
                    ::core::ptr::null::<Trajectory>(),
                    &raw mut applied,
                    &raw mut apply_started,
                    &raw mut apply_finished,
                    &raw mut err as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
                ) {
                    guidance_result_clear(&raw mut r);
                    guidance_result_clear(&raw mut glide);
                    current_block = 11056270165857487203;
                } else {
                    apply_succeeded = true_0 != 0;
                    t.has_apply_latency = true_0 != 0;
                    t.apply_latency_ms = (apply_finished - apply_started)
                        * 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
                    current_block = 5722677567366458307;
                }
            } else if was_cal as ::core::ffi::c_int != 0
                && finished as ::core::ffi::c_int != 0
            {
                krpc_safe((*c).session);
                current_block = 5722677567366458307;
            } else if r.phase as ::core::ffi::c_uint
                == PHASE_ABORT as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                krpc_safe((*c).session);
                current_block = 5722677567366458307;
            } else if (*c).guidance.automation_engaged as ::core::ffi::c_int != 0
                && !(*c).guidance.paused
            {
                apply_attempted = true_0 != 0;
                if !apply_command_logged(
                    c,
                    &raw mut cmd,
                    cfg.vehicle.airbrake_action_group,
                    r.phase,
                    hud_status,
                    hud_warning,
                    &raw mut r.reference,
                    &raw mut applied,
                    &raw mut apply_started,
                    &raw mut apply_finished,
                    &raw mut err as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 768]>() as size_t,
                ) {
                    guidance_result_clear(&raw mut r);
                    guidance_result_clear(&raw mut glide);
                    current_block = 11056270165857487203;
                } else {
                    apply_succeeded = true_0 != 0;
                    t.has_apply_latency = true_0 != 0;
                    t.apply_latency_ms = (apply_finished - apply_started)
                        * 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
                    current_block = 5722677567366458307;
                }
            } else {
                current_block = 5722677567366458307;
            }
            match current_block {
                11056270165857487203 => {}
                _ => {
                    if apply_attempted as ::core::ffi::c_int != 0
                        && apply_succeeded as ::core::ffi::c_int != 0
                    {
                        let mut apply_budget: KRPCTransportBudget = krpc_session_transport_budget(
                            (*c).session,
                        );
                        t.has_rpc_budget = true_0 != 0;
                        t.rpc_apply_calls = apply_budget.apply_calls;
                        t.rpc_apply_wire_requests = apply_budget.apply_wire_requests;
                        t.rpc_total_calls = apply_budget.total_calls;
                        t.rpc_total_wire_requests = apply_budget.total_wire_requests;
                    }
                    t.has_control_loop = true_0 != 0;
                    t.control_loop_ms = (monotonic_seconds() - wall_start)
                        * 1000 as ::core::ffi::c_int as ::core::ffi::c_double;
                    (*c).latest_telemetry = t;
                    if !using_glide && !entry_forecast_matches_guidance(c) {
                        (*c).prediction_lineage_discontinuity = true_0 != 0;
                        (*c).last_prediction_ut = -1e300f64;
                        let mut no_forecast: Trajectory = Trajectory {
                            points: ::core::ptr::null_mut::<TrajectoryPoint>(),
                            count: 0,
                            capacity: 0,
                        };
                        trajectory_calibrator_set_forecast(
                            &raw mut (*c).trajectory_calibrator,
                            &raw mut no_forecast,
                        );
                        if (*c).guidance.phase as ::core::ffi::c_uint
                            != PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                                as ::core::ffi::c_uint
                        {
                            (*c).forecast_entry_plan_identity_valid = false_0 != 0;
                            (*c).forecast_entry_plan_id = 0 as uint64_t;
                            (*c).forecast_entry_plan_version = 0 as uint64_t;
                            trajectory_clear(&raw mut (*c).dynamic_trajectory);
                            trajectory_clear(&raw mut (*c).stabilized_trajectory);
                        }
                    }
                    native_mm305_route = (*c).guidance.mm305_route_committed
                        as ::core::ffi::c_int != 0
                        && (*c).guidance.mm305_route.valid as ::core::ffi::c_int != 0;
                    native_route_remaining = if native_mm305_route as ::core::ffi::c_int
                        != 0
                    {
                        taem_route_remaining_at_index(
                            &raw mut (*c).guidance.mm305_route,
                            (*c).guidance.mm305_route_cursor,
                        )
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_prediction_valid = native_mm305_route;
                    (*c).snapshot.guidance_terminal_candidate_valid = native_mm305_route;
                    (*c).snapshot.guidance_terminal_committed = (*c)
                        .guidance
                        .terminal_path_committed;
                    (*c).snapshot.guidance_terminal_candidate_radius = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.hac.radius_m
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_altitude = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.profile_final_altitude_m
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_speed = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        t.true_air_speed
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_kind = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        TERMINAL_PATH_HAC as ::core::ffi::c_int
                    } else {
                        TERMINAL_PATH_NONE as ::core::ffi::c_int
                    };
                    (*c).snapshot.guidance_terminal_candidate_geometry_degraded = false_0
                        != 0;
                    (*c).snapshot.guidance_terminal_candidate_energy_degraded = false_0
                        != 0;
                    (*c).snapshot.guidance_terminal_candidate_shell_degraded = false_0
                        != 0;
                    (*c).snapshot.guidance_terminal_candidate_side = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.side
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_slope = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.profile_final_slope_deg
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_curve_length = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.length_m
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_lead_length = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.lead_length_m
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_arc_remaining = if native_route_remaining
                        .is_finite() as i32 != 0
                    {
                        native_route_remaining
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_final_distance = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.hac.final_length_m
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_quality = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_arrival_ut = ::core::f32::NAN
                        as ::core::ffi::c_double;
                    (*c).snapshot.guidance_terminal_candidate_course = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.runway_heading_deg
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_candidate_peak_lateral = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_peak_rate_ratio = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_exit_speed = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_tracking_error = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_tracking_course_error = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_tracking_endpoint_error = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_tracking_margin = 0.0f64;
                    (*c).snapshot.guidance_terminal_candidate_tracking_evaluated = false_0
                        != 0;
                    (*c).snapshot.guidance_terminal_candidate_path_degraded = (*c)
                        .guidance
                        .taem_safety_handoff;
                    (*c).snapshot.guidance_terminal_candidate_control_degraded = false_0
                        != 0;
                    (*c).snapshot.guidance_terminal_candidate_rate_degraded = false_0
                        != 0;
                    (*c).snapshot.guidance_terminal_candidate_end_degraded = false_0
                        != 0;
                    native_route_station = if native_mm305_route as ::core::ffi::c_int
                        != 0
                    {
                        taem_route_station_at_index(
                            &raw mut (*c).guidance.mm305_route,
                            (*c).guidance.mm305_route_cursor,
                        )
                    } else {
                        0.0f64
                    };
                    native_route_progress = if native_mm305_route as ::core::ffi::c_int
                        != 0 && native_route_station.is_finite() as i32 != 0
                        && (*c).guidance.mm305_route.length_m > 1.0f64
                    {
                        clampd(
                            native_route_station / (*c).guidance.mm305_route.length_m,
                            0.0f64,
                            1.0f64,
                        )
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_reference_fpa = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.mm305_route.profile_final_slope_deg
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_terminal_mix = if native_mm305_route
                        as ::core::ffi::c_int != 0
                    {
                        1.0f64
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_taem_energy_law_active = cfg
                        .guidance
                        .taem_energy_law as ::core::ffi::c_int != 0
                        && r.phase as ::core::ffi::c_uint
                            == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
                        && (*c).guidance.taem_energy_law.initialized
                            as ::core::ffi::c_int != 0;
                    (*c).snapshot.guidance_taem_energy_drag_level = (*c)
                        .guidance
                        .taem_energy_law
                        .k;
                    (*c).snapshot.guidance_taem_energy_path_to_go = (*c)
                        .guidance
                        .taem_energy_law
                        .s_go;
                    (*c).snapshot.guidance_taem_energy_required_path = (*c)
                        .guidance
                        .taem_energy_law
                        .s_req;
                    (*c).snapshot.has_active_runway = (*c)
                        .guidance
                        .runway_end_preview_valid as ::core::ffi::c_int != 0
                        || (*c).guidance.runway_end_committed as ::core::ffi::c_int != 0;
                    (*c).snapshot.active_runway = cfg.site;
                    if (*c).snapshot.has_active_runway as ::core::ffi::c_int != 0
                        && cfg.site.allow_reciprocal_runway as ::core::ffi::c_int != 0
                        && (*c).guidance.runway_end_index == 1 as ::core::ffi::c_int
                    {
                        (*c).snapshot.active_runway = runway_reciprocal_site(
                            &raw mut cfg.site,
                            (*krpc_session_planet((*c).session)).radius,
                        );
                    }
                    (*c).snapshot.active_final_distance = if (*c)
                        .guidance
                        .terminal_final_handoff_latched as ::core::ffi::c_int != 0
                        && (*c).guidance.terminal_final_handoff_distance.is_finite()
                            as i32 != 0
                        && (*c).guidance.terminal_final_handoff_distance > 0.0f64
                    {
                        (*c).guidance.terminal_final_handoff_distance
                    } else {
                        cfg.guidance.final_approach_distance
                    };
                    (*c).snapshot.active_final_slope = if (*c)
                        .guidance
                        .terminal_final_handoff_latched as ::core::ffi::c_int != 0
                        && (*c).guidance.terminal_final_handoff_slope_deg.is_finite()
                            as i32 != 0
                        && (*c).guidance.terminal_final_handoff_slope_deg > 0.0f64
                    {
                        (*c).guidance.terminal_final_handoff_slope_deg
                    } else {
                        cfg.guidance.final_glide_slope
                    };
                    (*c).snapshot.tick_sequence = (*c).tick_sequence;
                    (*c).log_sequence = (*c).log_sequence.wrapping_add(1);
                    (*c).snapshot.log_sequence = (*c).log_sequence;
                    (*c).snapshot.wall_monotonic_seconds = wall_start;
                    (*c).snapshot.simulation_dt = dt;
                    (*c).snapshot.has_vehicle_state = true_0 != 0;
                    (*c).snapshot.vehicle_state = state;
                    (*c).snapshot.has_attitude_quaternion = t.has_attitude_quaternion;
                    if t.has_attitude_quaternion {
                        memcpy(
                            &raw mut (*c).snapshot.attitude_quaternion
                                as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
                            &raw mut t.attitude_quaternion as *mut ::core::ffi::c_double
                                as *const ::core::ffi::c_void,
                            ::core::mem::size_of::<[::core::ffi::c_double; 4]>()
                                as size_t,
                        );
                    }
                    snprintf(
                        &raw mut (*c).snapshot.attitude_reference_frame
                            as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
                        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                        &raw mut t.attitude_reference_frame as *mut ::core::ffi::c_char,
                    );
                    shut_down_burn_engines(c);
                    govern_physics_warp(c, &raw mut t, &raw mut r);
                    (*c).snapshot.command_computed_wall_seconds = command_computed;
                    (*c).snapshot.command_apply_attempted = apply_attempted;
                    (*c).snapshot.command_apply_started_wall_seconds = apply_started;
                    (*c).snapshot.command_apply_finished_wall_seconds = apply_finished;
                    (*c).snapshot.command_applied = apply_succeeded as ::core::ffi::c_int
                        != 0 && applied.applied as ::core::ffi::c_int != 0;
                    (*c).snapshot.applied_command = applied;
                    (*c).snapshot.guidance_has_burn_started = (*c)
                        .guidance
                        .has_burn_command_started;
                    (*c).snapshot.guidance_burn_completed = (*c)
                        .guidance
                        .deorbit_burn_completed;
                    (*c).snapshot.guidance_atmosphere_crossed = (*c)
                        .guidance
                        .atmospheric_interface_crossed;
                    (*c).snapshot.guidance_final_captured = (*c)
                        .guidance
                        .final_approach_captured;
                    (*c).snapshot.guidance_airbrakes_deployed = (*c)
                        .guidance
                        .airbrakes_deployed;
                    (*c).snapshot.guidance_hac_side_selected = (*c)
                        .guidance
                        .hac_side_selected;
                    (*c).snapshot.guidance_delivered_delta_v = (*c)
                        .guidance
                        .delivered_delta_v;
                    (*c).snapshot.guidance_burn_active_elapsed = (*c)
                        .guidance
                        .burn_active_elapsed;
                    (*c).snapshot.guidance_s_turn_sign = (*c).guidance.s_turn_sign;
                    (*c).snapshot.guidance_hac_side = (*c).guidance.hac_side;
                    (*c).snapshot.guidance_entry_leg_elapsed = guidance_entry_leg_elapsed(
                        &raw mut (*c).guidance,
                        t.ut,
                    );
                    (*c).snapshot.guidance_entry_exec = entry_exec_telemetry(
                        &raw mut (*c).guidance.entry_exec,
                    );
                    (*c).snapshot.guidance_taem_exec = taem_exec_telemetry(
                        &raw mut (*c).guidance.taem_exec,
                    );
                    (*c).snapshot.guidance_hac_captured = (*c).guidance.hac_captured;
                    (*c).snapshot.guidance_hac_completed = (*c).guidance.hac_completed;
                    (*c).snapshot.guidance_hac_progress_valid = (*c)
                        .guidance
                        .hac_progress_valid;
                    (*c).snapshot.guidance_hac_transition_active = false_0 != 0;
                    (*c).snapshot.guidance_hac_remaining = (*c).guidance.hac_remaining;
                    (*c).snapshot.guidance_hac_radius = (*c).guidance.hac_radius;
                    (*c).snapshot.guidance_hac_circuit_count = 0 as ::core::ffi::c_uint;
                    (*c).snapshot.guidance_hac_circuit_slope = 0.0f64;
                    (*c).snapshot.guidance_terminal_reentry_after_ut = (*c)
                        .guidance
                        .terminal_reentry_after_ut;
                    (*c).snapshot.guidance_minimum_turn_radius = (*c)
                        .guidance
                        .minimum_turn_radius;
                    (*c).snapshot.guidance_hac_transition_progress = native_route_progress;
                    let mut i: size_t = 0 as size_t;
                    while i < 8 as size_t {
                        (*c).snapshot.guidance_hac_transition_points[i as usize] = 0.0f64;
                        i = i.wrapping_add(1);
                    }
                    (*c).snapshot.guidance_hac_transition_heading_cone = false_0 != 0;
                    (*c).snapshot.guidance_hac_transition_lead_curve = false_0 != 0;
                    let mut i_0: size_t = 0 as size_t;
                    while i_0 < 6 as size_t {
                        (*c)
                            .snapshot
                            .guidance_hac_transition_lead_points[i_0 as usize] = 0.0f64;
                        i_0 = i_0.wrapping_add(1);
                    }
                    (*c)
                        .snapshot
                        .guidance_hac_transition_cone_center[0 as ::core::ffi::c_int
                        as usize] = 0.0f64;
                    (*c)
                        .snapshot
                        .guidance_hac_transition_cone_center[1 as ::core::ffi::c_int
                        as usize] = 0.0f64;
                    (*c).snapshot.guidance_hac_transition_cone_start_angle = 0.0f64;
                    (*c).snapshot.guidance_hac_transition_cone_end_angle = 0.0f64;
                    (*c).snapshot.guidance_hac_transition_cone_arc_length = 0.0f64;
                    lift_accel = if t.mass
                        > 1 as ::core::ffi::c_int as ::core::ffi::c_double
                        && t.lift_force.is_finite() as i32 != 0
                        && t.lift_force
                            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                    {
                        t.lift_force / t.mass
                    } else {
                        0 as ::core::ffi::c_int as ::core::ffi::c_double
                    };
                    bank_for_rate = clampd(
                        norm_signed_deg(cmd.target_roll)
                            * clampd(t.bank_effectiveness, 0.35f64, 1.8f64),
                        -(89 as ::core::ffi::c_int) as ::core::ffi::c_double,
                        89 as ::core::ffi::c_int as ::core::ffi::c_double,
                    );
                    (*c).snapshot.guidance_commanded_course_rate_estimate = if t
                        .true_air_speed
                        > 1 as ::core::ffi::c_int as ::core::ffi::c_double
                        && lift_accel > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                    {
                        lift_accel * sin(bank_for_rate * DEG2RAD) / t.true_air_speed
                            * RAD2DEG
                    } else {
                        0 as ::core::ffi::c_int as ::core::ffi::c_double
                    };
                    (*c).snapshot.guidance_entry_plan_valid = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .valid;
                    (*c).snapshot.guidance_entry_plan_terminal_ready = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .valid as ::core::ffi::c_int != 0
                        && (*c).guidance.entry_s_turn_plan.terminal_ready
                            as ::core::ffi::c_int != 0;
                    (*c).snapshot.guidance_entry_plan_ut = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .planned_ut;
                    (*c).snapshot.guidance_entry_plan_target_bank = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .target_bank;
                    (*c).snapshot.guidance_entry_plan_target_aoa = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .target_aoa;
                    (*c).snapshot.guidance_entry_plan_target_heading = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .target_heading;
                    (*c).snapshot.guidance_entry_plan_segment_remaining = if (*c)
                        .guidance
                        .entry_s_turn_plan
                        .valid as ::core::ffi::c_int != 0
                    {
                        fmax(
                            0.0f64,
                            (*c).guidance.entry_s_turn_plan.planned_ut
                                + (*c).guidance.entry_s_turn_plan.segment_duration - t.ut,
                        )
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_entry_plan_cost = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .cost;
                    (*c).snapshot.guidance_entry_plan_taem_range_error = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .taem_range_error;
                    (*c).snapshot.guidance_entry_plan_taem_speed = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .taem_speed;
                    (*c).snapshot.guidance_entry_plan_taem_energy_error = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .taem_energy_error;
                    (*c).snapshot.guidance_entry_plan_predicted_reversals = (*c)
                        .guidance
                        .entry_s_turn_plan
                        .predicted_reversals;
                    (*c).snapshot.guidance_entry_feedback = (*c).guidance.entry_feedback;
                    (*c).snapshot.guidance_entry_reversal_scheduled = (*c)
                        .guidance
                        .entry_reversal_scheduled;
                    (*c).snapshot.guidance_entry_reversal_is_final = (*c)
                        .guidance
                        .entry_reversal_is_final;
                    (*c).snapshot.guidance_entry_reversal_time_remaining = if (*c)
                        .guidance
                        .entry_reversal_scheduled as ::core::ffi::c_int != 0
                    {
                        (*c).guidance.entry_reversal_ut - t.ut
                    } else {
                        0.0f64
                    };
                    (*c).snapshot.guidance_entry_reversal_range = (*c)
                        .guidance
                        .entry_reversal_range;
                    (*c).snapshot.guidance_entry_reversal_sign = (*c)
                        .guidance
                        .entry_reversal_sign;
                    (*c).snapshot.connection_status = CONN_CONNECTED;
                    (*c).snapshot.phase = (if using_glide as ::core::ffi::c_int != 0 {
                        PHASE_CALIBRATION as ::core::ffi::c_int as ::core::ffi::c_uint
                    } else {
                        r.phase as ::core::ffi::c_uint
                    }) as GuidancePhase;
                    landing_snapshot_set_sample(
                        &raw mut (*c).snapshot,
                        &raw mut t,
                        &raw mut (*c).latest_state,
                    );
                    (*c).snapshot.command = cmd;
                    (*c).snapshot.status_message[0 as ::core::ffi::c_int as usize] = 0
                        as ::core::ffi::c_char;
                    snprintf(
                        &raw mut (*c).snapshot.status_message
                            as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                        if using_glide as ::core::ffi::c_int != 0 {
                            &raw mut r.status as *mut ::core::ffi::c_char
                        } else {
                            &raw mut r.status as *mut ::core::ffi::c_char
                        },
                    );
                    snprintf(
                        &raw mut (*c).snapshot.decision_reason
                            as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
                        b"phase=%s; profile=%s; automation=%s; paused=%s; burn_started=%s; burn_completed=%s; atmosphere_crossed=%s; final_captured=%s; delivered_dv=%.9g; entry_leg_elapsed=%.9g\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        phase_string((*c).snapshot.phase),
                        profile_string(cmd.control_profile),
                        if (*c).guidance.automation_engaged as ::core::ffi::c_int != 0 {
                            b"true\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"false\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        if (*c).guidance.paused as ::core::ffi::c_int != 0 {
                            b"true\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"false\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        if (*c).guidance.has_burn_command_started as ::core::ffi::c_int
                            != 0
                        {
                            b"true\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"false\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        if (*c).guidance.deorbit_burn_completed as ::core::ffi::c_int
                            != 0
                        {
                            b"true\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"false\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        if (*c).guidance.atmospheric_interface_crossed
                            as ::core::ffi::c_int != 0
                        {
                            b"true\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"false\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        if (*c).guidance.final_approach_captured as ::core::ffi::c_int
                            != 0
                        {
                            b"true\0" as *const u8 as *const ::core::ffi::c_char
                        } else {
                            b"false\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        (*c).guidance.delivered_delta_v,
                        (*c).snapshot.guidance_entry_leg_elapsed,
                    );
                    replace_reference_trajectory(c, &raw mut r.reference);
                    replace_predicted_trajectory(c, &raw mut (*c).stabilized_trajectory);
                    warning = if r.has_warning as ::core::ffi::c_int != 0 {
                        &raw mut r.warning as *mut ::core::ffi::c_char
                            as *const ::core::ffi::c_char
                    } else {
                        achieved_state_warning(
                            c,
                            &raw mut err as *mut ::core::ffi::c_char,
                            ::core::mem::size_of::<[::core::ffi::c_char; 768]>()
                                as size_t,
                        )
                    };
                    if warning.is_null() && (*c).has_plan as ::core::ffi::c_int != 0
                        && (*c).plan.execution_degraded as ::core::ffi::c_int != 0
                        && (*c).guidance.automation_engaged as ::core::ffi::c_int != 0
                        && !(*c).guidance.deorbit_burn_completed
                    {
                        warning = b"Guarded recovery plan is active because the preferred strict corridor was not available; live cutoff and achieved-state verification remain armed.\0"
                            as *const u8 as *const ::core::ffi::c_char;
                    }
                    if warning.is_null() {
                        warning = environment_warning(
                            c,
                            &raw mut t,
                            &raw mut err as *mut ::core::ffi::c_char,
                            ::core::mem::size_of::<[::core::ffi::c_char; 768]>()
                                as size_t,
                        );
                    }
                    set_warning(c, warning);
                    set_error(c, ::core::ptr::null::<::core::ffi::c_char>());
                    (*c).snapshot.automation_engaged = (*c).guidance.automation_engaged;
                    (*c).snapshot.paused = (*c).guidance.paused;
                    (*c).snapshot.calibration = cal_snap;
                    (*c).snapshot.adaptive_vehicle_profile = (*c).adaptive_profile;
                    (*c).snapshot.plan = if (*c).has_plan as ::core::ffi::c_int != 0 {
                        &raw mut (*c).plan
                    } else {
                        ::core::ptr::null_mut::<DeorbitPlan>()
                    };
                    append_log(c);
                    publish(c);
                    guidance_result_clear(&raw mut r);
                    guidance_result_clear(&raw mut glide);
                    return;
                }
            }
        }
    }
    krpc_safe((*c).session);
    guidance_abort(&raw mut (*c).guidance);
    glide_calibration_stop(
        &raw mut (*c).glide,
        b"Calibration stopped after a kRPC or control error.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    (*c).snapshot.tick_sequence = (*c).tick_sequence;
    (*c).log_sequence = (*c).log_sequence.wrapping_add(1);
    (*c).snapshot.log_sequence = (*c).log_sequence;
    (*c).snapshot.wall_monotonic_seconds = wall_start;
    (*c).snapshot.command_computed_wall_seconds = command_computed;
    (*c).snapshot.command_apply_attempted = apply_attempted;
    (*c).snapshot.command_apply_started_wall_seconds = apply_started;
    (*c).snapshot.command_apply_finished_wall_seconds = apply_finished;
    (*c).snapshot.command_applied = false_0 != 0;
    (*c).snapshot.applied_command = applied;
    if have_telemetry {
        landing_snapshot_set_sample(
            &raw mut (*c).snapshot,
            &raw mut t,
            &raw mut (*c).latest_state,
        );
        (*c).snapshot.tick_sequence = (*c).tick_sequence;
        (*c).snapshot.has_vehicle_state = true_0 != 0;
        (*c).snapshot.vehicle_state = state;
        (*c).snapshot.has_attitude_quaternion = t.has_attitude_quaternion;
        if t.has_attitude_quaternion {
            memcpy(
                &raw mut (*c).snapshot.attitude_quaternion as *mut ::core::ffi::c_double
                    as *mut ::core::ffi::c_void,
                &raw mut t.attitude_quaternion as *mut ::core::ffi::c_double
                    as *const ::core::ffi::c_void,
                ::core::mem::size_of::<[::core::ffi::c_double; 4]>() as size_t,
            );
        }
        snprintf(
            &raw mut (*c).snapshot.attitude_reference_frame as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut t.attitude_reference_frame as *mut ::core::ffi::c_char,
        );
    }
    (*c).snapshot.phase = PHASE_FAULT;
    (*c).snapshot.connection_status = CONN_FAILED;
    snprintf(
        &raw mut (*c).snapshot.status_message as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"Guidance stopped after a kRPC or control error.\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
        b"event=fault; telemetry_available=%s; command_apply_attempted=%s; error=%s\0"
            as *const u8 as *const ::core::ffi::c_char,
        if have_telemetry as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        if apply_attempted as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        if err[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
            &raw mut err as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
        } else {
            b"unknown\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    set_error(
        c,
        if err[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
            &raw mut err as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
        } else {
            b"Unknown kRPC or control failure\0" as *const u8
                as *const ::core::ffi::c_char
        },
    );
    (*c).snapshot.automation_engaged = false_0 != 0;
    (*c).snapshot.paused = false_0 != 0;
    (*c).snapshot.calibration.active = false_0 != 0;
    append_vehicle_log(
        c,
        &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
        true_0 != 0,
        false_0 != 0,
    );
    append_planner_log(
        c,
        &raw mut (*c).snapshot.decision_reason as *mut ::core::ffi::c_char,
        true_0 != 0,
    );
    durable_flush_log(c);
    publish(c);
}
unsafe extern "C" fn control_thread(
    mut arg: *mut ::core::ffi::c_void,
) -> *mut ::core::ffi::c_void {
    let mut c: *mut LandingController = arg as *mut LandingController;
    loop {
        let mut start: ::core::ffi::c_double = monotonic_seconds();
        pthread_mutex_lock(&raw mut (*c).mutex);
        if (*c).stop_thread {
            pthread_mutex_unlock(&raw mut (*c).mutex);
            break;
        } else {
            tick_locked(c);
            let mut hz: ::core::ffi::c_double = clampd(
                (*c).configuration.guidance.guidance_rate,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
                30 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            let mut final_gear_contact_window: bool = (*c).guidance.automation_engaged
                as ::core::ffi::c_int != 0 && (*c).has_latest as ::core::ffi::c_int != 0
                && (*c).latest_telemetry.radar_altitude.is_finite() as i32 != 0
                && (*c).latest_telemetry.radar_altitude <= 100.0f64
                && ((*c).guidance.phase as ::core::ffi::c_uint
                    == PHASE_FINAL as ::core::ffi::c_int as ::core::ffi::c_uint
                    || (*c).guidance.phase as ::core::ffi::c_uint
                        == PHASE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint
                    || (*c).guidance.phase as ::core::ffi::c_uint
                        == PHASE_TOUCHDOWN as ::core::ffi::c_int as ::core::ffi::c_uint
                    || (*c).guidance.phase as ::core::ffi::c_uint
                        == PHASE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint);
            if final_gear_contact_window {
                hz = 30.0f64;
            }
            let mut fast_sim: bool = !(*c).session.is_null()
                && krpc_session_is_simulator((*c).session) as ::core::ffi::c_int != 0
                && (*c).guidance.automation_engaged as ::core::ffi::c_int != 0
                && !(*c).guidance.paused;
            let mut cadence_anchor: ::core::ffi::c_double = fmax(
                (*c).last_terminal_prediction_request_ut,
                (*c).last_terminal_prediction_completion_ut,
            );
            let mut request_outstanding: bool = (*c).last_terminal_prediction_request_ut
                > (*c).last_terminal_prediction_completion_ut + 1e-6f64;
            let mut sim_wall: ::core::ffi::c_double = if fast_sim as ::core::ffi::c_int
                != 0 && (*c).last_terminal_prediction_completion_ut > -1e299f64
                && (*c).has_latest as ::core::ffi::c_int != 0
            {
                (*c).last_terminal_prediction_completion_wall
                    + ((*c).latest_telemetry.ut
                        - (*c).last_terminal_prediction_completion_ut)
            } else {
                monotonic_seconds()
            };
            let mut request_due: bool = (*c).has_latest as ::core::ffi::c_int != 0
                && async_prediction_request_due(
                    (*c).latest_telemetry.ut,
                    cadence_anchor,
                    sim_wall,
                    (*c).last_terminal_prediction_completion_wall,
                    (*c).last_terminal_prediction_solve_wall,
                    (*c).configuration.guidance.prediction_interval,
                ) as ::core::ffi::c_int != 0;
            let mut sim_plan_barrier: bool = fast_sim as ::core::ffi::c_int != 0
                && (*c).guidance.phase as ::core::ffi::c_uint
                    == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
                && ((*c).guidance.entry_planning_needed as ::core::ffi::c_int != 0
                    || (*c).guidance.entry_final_reversal_pending as ::core::ffi::c_int
                        != 0
                    || (*c).guidance.entry_final_reversal_completed as ::core::ffi::c_int
                        != 0)
                && (request_outstanding as ::core::ffi::c_int != 0
                    || request_due as ::core::ffi::c_int != 0);
            let mut sim_mm305_barrier: bool = fast_sim as ::core::ffi::c_int != 0
                && (*c).guidance.mm305_async_planning as ::core::ffi::c_int != 0
                && ((*c).guidance.phase as ::core::ffi::c_uint
                    == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
                    && (*c).guidance.mm305_planning_needed as ::core::ffi::c_int != 0
                    || (*c).guidance.phase as ::core::ffi::c_uint
                        == PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                            as ::core::ffi::c_uint
                        && (*c).guidance.mm305_admission_needed as ::core::ffi::c_int
                            != 0);
            let mut barrier_ut: ::core::ffi::c_double = if (*c).has_latest
                as ::core::ffi::c_int != 0
            {
                (*c).latest_telemetry.ut
            } else {
                -::core::f32::INFINITY as ::core::ffi::c_double
            };
            let mut barrier_generation: uint64_t = (*c).prediction_generation;
            pthread_mutex_unlock(&raw mut (*c).mutex);
            if sim_plan_barrier {
                loop {
                    sleep_seconds(0.001f64);
                    pthread_mutex_lock(&raw mut (*c).mutex);
                    let mut stop: bool = (*c).stop_thread as ::core::ffi::c_int != 0
                        || (*c).session.is_null()
                        || !krpc_session_is_simulator((*c).session)
                        || !(*c).guidance.automation_engaged
                        || (*c).guidance.paused as ::core::ffi::c_int != 0
                        || (*c).guidance.phase as ::core::ffi::c_uint
                            != PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                                as ::core::ffi::c_uint
                        || (*c).prediction_generation != barrier_generation;
                    let mut completed: bool = (*c).last_terminal_prediction_completion_ut
                        >= barrier_ut - 1e-6f64;
                    pthread_mutex_unlock(&raw mut (*c).mutex);
                    if stop as ::core::ffi::c_int != 0
                        || completed as ::core::ffi::c_int != 0
                    {
                        break;
                    }
                }
            } else if sim_mm305_barrier {
                let mut started: ::core::ffi::c_double = monotonic_seconds();
                loop {
                    sleep_seconds(0.001f64);
                    pthread_mutex_lock(&raw mut (*c).mutex);
                    let mut pending: bool = (*c).guidance.phase as ::core::ffi::c_uint
                        == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
                        && (*c).guidance.mm305_planning_needed as ::core::ffi::c_int != 0
                        || (*c).guidance.phase as ::core::ffi::c_uint
                            == PHASE_ENTRY_ENERGY as ::core::ffi::c_int
                                as ::core::ffi::c_uint
                            && (*c).guidance.mm305_admission_needed as ::core::ffi::c_int
                                != 0;
                    let mut done: bool = (*c).stop_thread as ::core::ffi::c_int != 0
                        || (*c).session.is_null() || !(*c).guidance.automation_engaged
                        || (*c).guidance.paused as ::core::ffi::c_int != 0 || !pending
                        || (*c).prediction_generation != barrier_generation;
                    pthread_mutex_unlock(&raw mut (*c).mutex);
                    if done as ::core::ffi::c_int != 0
                        || monotonic_seconds() - started > 180.0f64
                    {
                        break;
                    }
                }
            } else if !fast_sim {
                sleep_seconds(
                    fmax(
                        0 as ::core::ffi::c_int as ::core::ffi::c_double,
                        1.0f64 / hz - (monotonic_seconds() - start),
                    ),
                );
            }
        }
    }
    return NULL;
}
unsafe extern "C" fn start_thread(mut c: *mut LandingController) {
    pthread_mutex_lock(&raw mut (*c).mutex);
    if !(*c).thread_started {
        (*c).stop_thread = false_0 != 0;
        let mut attr: pthread_attr_t = core::mem::zeroed();
        pthread_attr_init(&raw mut attr);
        pthread_attr_setstacksize(
            &raw mut attr,
            (8 as ::core::ffi::c_int * 1024 as ::core::ffi::c_int
                * 1024 as ::core::ffi::c_int) as size_t,
        );
        if pthread_create(
            &raw mut (*c).thread,
            &raw mut attr,
            Some(
                control_thread
                    as unsafe extern "C" fn(
                        *mut ::core::ffi::c_void,
                    ) -> *mut ::core::ffi::c_void,
            ),
            c as *mut ::core::ffi::c_void,
        ) == 0 as ::core::ffi::c_int
        {
            (*c).thread_started = true_0 != 0;
        }
        pthread_attr_destroy(&raw mut attr);
    }
    pthread_mutex_unlock(&raw mut (*c).mutex);
    start_prediction_worker(c);
}
unsafe extern "C" fn effective_config(
    mut c: *const LandingController,
) -> LandingConfiguration {
    let mut r: LandingConfiguration = (*c).configuration;
    if r.calibration.auto_apply_in_flight {
        r.vehicle.minimum_safe_speed = (*c).adaptive_profile.minimum_safe_speed;
        r.vehicle.final_approach_speed = (*c).adaptive_profile.final_approach_speed;
        r.vehicle.touchdown_speed = (*c).adaptive_profile.touchdown_speed;
    }
    landing_configuration_normalize(&raw mut r);
    return r;
}
unsafe extern "C" fn envelope_at_controller(
    mut e: *const AerodynamicEnvelope,
    mut mach: ::core::ffi::c_double,
) -> AerodynamicModel {
    let anchor: [::core::ffi::c_double; 4] = [
        0.35f64,
        1.05f64,
        2.6f64,
        6 as ::core::ffi::c_int as ::core::ffi::c_double,
    ];
    let mut x: ::core::ffi::c_double = fmax(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        mach,
    );
    if x <= anchor[0 as ::core::ffi::c_int as usize] {
        return (*e).regimes[0 as ::core::ffi::c_int as usize];
    }
    if x >= anchor[3 as ::core::ffi::c_int as usize] {
        return (*e).regimes[3 as ::core::ffi::c_int as usize];
    }
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if x <= anchor[(i + 1 as ::core::ffi::c_int) as usize] {
            let mut f: ::core::ffi::c_double = (x - anchor[i as usize])
                / fmax(
                    anchor[(i + 1 as ::core::ffi::c_int) as usize] - anchor[i as usize],
                    1e-9f64,
                );
            return AerodynamicModel {
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
        }
        i += 1;
    }
    return (*e).regimes[3 as ::core::ffi::c_int as usize];
}
unsafe extern "C" fn apply_certified_aero_prior(mut c: *mut LandingController) {
    let mut planning: AerodynamicModel = AerodynamicModel {
        lift_to_drag: 0.,
        ballistic_coefficient: 0.,
        confidence: 0.,
    };
    vessel_physics_derive_envelope(
        &raw mut (*c).physics,
        &raw mut (*c).configuration.vehicle,
        &raw mut (*c).envelope,
        &raw mut planning,
    );
    (*c).planning_aerodynamics = planning;
    (*c).aerodynamics = if (*c).has_latest as ::core::ffi::c_int != 0 {
        envelope_at_controller(&raw mut (*c).envelope, (*c).latest_telemetry.mach)
    } else {
        planning
    };
}
unsafe extern "C" fn vehicle_profile_equal(
    mut a: *const VehicleProfile,
    mut b: *const VehicleProfile,
) -> bool {
    return strcmp(
        &raw const (*a).model_id as *const ::core::ffi::c_char,
        &raw const (*b).model_id as *const ::core::ffi::c_char,
    ) == 0 && (*a).estimated_lift_to_drag == (*b).estimated_lift_to_drag
        && (*a).estimated_ballistic_coefficient == (*b).estimated_ballistic_coefficient
        && (*a).entry_angle_of_attack == (*b).entry_angle_of_attack
        && (*a).maximum_angle_of_attack == (*b).maximum_angle_of_attack
        && (*a).terminal_maximum_lift_angle_of_attack
            == (*b).terminal_maximum_lift_angle_of_attack
        && (*a).maximum_bank_angle == (*b).maximum_bank_angle
        && (*a).maximum_dynamic_pressure == (*b).maximum_dynamic_pressure
        && (*a).maximum_g_load == (*b).maximum_g_load
        && (*a).minimum_safe_speed == (*b).minimum_safe_speed
        && (*a).final_approach_speed == (*b).final_approach_speed
        && (*a).touchdown_speed == (*b).touchdown_speed
        && (*a).allow_powered_approach as ::core::ffi::c_int
            == (*b).allow_powered_approach as ::core::ffi::c_int
        && (*a).maximum_approach_throttle == (*b).maximum_approach_throttle
        && (*a).airbrake_action_group == (*b).airbrake_action_group;
}
unsafe extern "C" fn configuration_equal(
    mut a: *const LandingConfiguration,
    mut b: *const LandingConfiguration,
) -> bool {
    let mut wa: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    let mut wb: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut wa);
    jw_init(&raw mut wb);
    landing_configuration_json(&raw mut wa, a);
    landing_configuration_json(&raw mut wb, b);
    let mut same: bool = !wa.failed && !wb.failed && !wa.data.is_null()
        && !wb.data.is_null() && strcmp(wa.data, wb.data) == 0;
    jw_free(&raw mut wa);
    jw_free(&raw mut wb);
    return same;
}
unsafe extern "C" fn planning_configuration_equal(
    mut a: *const LandingConfiguration,
    mut b: *const LandingConfiguration,
) -> bool {
    let mut aa: LandingConfiguration = *a;
    let mut bb: LandingConfiguration = *b;
    memset(
        &raw mut aa.connection as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<ConnectionConfiguration>() as size_t,
    );
    memset(
        &raw mut bb.connection as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<ConnectionConfiguration>() as size_t,
    );
    memset(
        &raw mut aa.calibration as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<CalibrationSettings>() as size_t,
    );
    memset(
        &raw mut bb.calibration as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<CalibrationSettings>() as size_t,
    );
    bb.guidance.use_time_warp = false_0 != 0;
    aa.guidance.use_time_warp = bb.guidance.use_time_warp;
    return configuration_equal(&raw mut aa, &raw mut bb);
}
unsafe extern "C" fn entry_forecast_matches_guidance(
    mut c: *const LandingController,
) -> bool {
    if c.is_null() {
        return true_0 != 0;
    }
    if (*c).forecast_entry_plan_identity_valid {
        if (*c).guidance.phase as ::core::ffi::c_uint
            != PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            return false_0 != 0;
        }
        return (*c).guidance.entry_control_plan_valid as ::core::ffi::c_int != 0
            && (*c).guidance.entry_s_turn_plan.valid as ::core::ffi::c_int != 0
            && (*c).forecast_entry_plan_id == (*c).guidance.entry_s_turn_plan.plan_id
            && (*c).forecast_entry_plan_version
                == (*c).guidance.entry_s_turn_plan.plan_version;
    }
    if (*c).guidance.phase as ::core::ffi::c_uint
        != PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return true_0 != 0;
    }
    return !((*c).guidance.entry_control_plan_valid as ::core::ffi::c_int != 0
        && (*c).guidance.entry_s_turn_plan.valid as ::core::ffi::c_int != 0);
}
unsafe extern "C" fn telemetry_warped(mut t: *const Telemetry) -> bool {
    return (*t).has_warp as ::core::ffi::c_int != 0
        && ((*t).warp_rails as ::core::ffi::c_int != 0 || (*t).warp_rate > 1.0001f64);
}
unsafe extern "C" fn read_live_sample(
    mut c: *mut LandingController,
    mut cfg: *mut LandingConfiguration,
    mut t: *mut Telemetry,
    mut state: *mut VehicleState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !krpc_read_telemetry((*c).session, cfg, t, state, error, error_size) {
        return false_0 != 0;
    }
    let mut warp_factor: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    let mut warp_rate: ::core::ffi::c_double = 1.0f64;
    let mut warp_rails: bool = false_0 != 0;
    if krpc_read_physics_warp(
        (*c).session,
        &raw mut warp_factor,
        &raw mut warp_rate,
        &raw mut warp_rails,
        ::core::ptr::null_mut::<::core::ffi::c_char>(),
        0 as size_t,
    ) {
        (*t).has_warp = true_0 != 0;
        (*t).physics_warp_factor = warp_factor;
        (*t).warp_rate = warp_rate;
        (*t).warp_rails = warp_rails;
    }
    let mut p: *const PlanetModel = krpc_session_planet((*c).session);
    if !telemetry_warped(t) {
        vessel_physics_observe(&raw mut (*c).physics, t, state, p);
    }
    (*c).trajectory_calibration.physics = &raw mut (*c).physics;
    (*c).latest_telemetry = *t;
    (*c).latest_state = *state;
    (*c).has_latest = true_0 != 0;
    landing_snapshot_set_sample(&raw mut (*c).snapshot, t, state);
    return true_0 != 0;
}
unsafe extern "C" fn publish(mut c: *mut LandingController) {
    (*c).snapshot.plan = if (*c).has_plan as ::core::ffi::c_int != 0 {
        &raw mut (*c).plan
    } else {
        ::core::ptr::null_mut::<DeorbitPlan>()
    };
    if (*c).callback.is_none() {
        return;
    }
    if !(*c).session.is_null()
        && krpc_session_is_simulator((*c).session) as ::core::ffi::c_int != 0
    {
        let mut hz: ::core::ffi::c_double = 12.0f64;
        let mut e: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_SIM_PUBLISH_HZ\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if !e.is_null() && *e as ::core::ffi::c_int != 0 {
            let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                ::core::ffi::c_char,
            >();
            let mut v: ::core::ffi::c_double = strtod(e, &raw mut end);
            if end != e as *mut ::core::ffi::c_char && *end == 0
                && v.is_finite() as i32 != 0
                && v >= 1 as ::core::ffi::c_int as ::core::ffi::c_double
                && v <= 120 as ::core::ffi::c_int as ::core::ffi::c_double
            {
                hz = v;
            }
        }
        let mut now: ::core::ffi::c_double = monotonic_seconds();
        let mut current_ut: ::core::ffi::c_double = (*c).snapshot.telemetry.ut;
        let mut first_live: bool = current_ut.is_finite() as i32 != 0
            && current_ut > 0.0f64
            && ((*c).sim_last_publish_ut.is_finite() as i32 == 0
                || (*c).sim_last_publish_ut <= 0.0f64);
        let mut force: bool = !(*c).sim_publication_valid
            || first_live as ::core::ffi::c_int != 0
            || (*c).publication_event_pending as ::core::ffi::c_int != 0
            || (*c).snapshot.phase as ::core::ffi::c_uint
                != (*c).sim_last_publish_phase as ::core::ffi::c_uint
            || (*c).snapshot.connection_status as ::core::ffi::c_uint
                != (*c).sim_last_publish_connection as ::core::ffi::c_uint
            || (*c).snapshot.automation_engaged as ::core::ffi::c_int
                != (*c).sim_last_publish_engaged as ::core::ffi::c_int
            || (*c).snapshot.paused as ::core::ffi::c_int
                != (*c).sim_last_publish_paused as ::core::ffi::c_int;
        if !force && now - (*c).sim_last_publish_wall < 1.0f64 / hz {
            return;
        }
        (*c).sim_publication_valid = true_0 != 0;
        (*c).sim_last_publish_wall = now;
        (*c).sim_last_publish_ut = current_ut;
        (*c).sim_last_publish_phase = (*c).snapshot.phase;
        (*c).sim_last_publish_connection = (*c).snapshot.connection_status;
        (*c).sim_last_publish_engaged = (*c).snapshot.automation_engaged;
        (*c).sim_last_publish_paused = (*c).snapshot.paused;
    } else {
        (*c).sim_publication_valid = false_0 != 0;
    }
    (*c).publication_event_pending = false_0 != 0;
    (*c)
        .callback
        .expect(
            "non-null function pointer",
        )(&raw mut (*c).snapshot, (*c).callback_context);
}
unsafe extern "C" fn set_warning(
    mut c: *mut LandingController,
    mut msg: *const ::core::ffi::c_char,
) {
    let mut present: bool = !msg.is_null() && *msg as ::core::ffi::c_int != 0;
    (*c).publication_event_pending = ((*c).publication_event_pending
        as ::core::ffi::c_int
        | ((*c).snapshot.has_warning as ::core::ffi::c_int
            != present as ::core::ffi::c_int
            || present as ::core::ffi::c_int != 0
                && strncmp(
                    &raw mut (*c).snapshot.warning_message as *mut ::core::ffi::c_char,
                    msg,
                    (::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t)
                        .wrapping_sub(1 as size_t),
                ) != 0 as ::core::ffi::c_int) as ::core::ffi::c_int) != 0;
    (*c).snapshot.has_warning = present;
    if (*c).snapshot.has_warning {
        snprintf(
            &raw mut (*c).snapshot.warning_message as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 384]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            msg,
        );
    } else {
        (*c).snapshot.warning_message[0 as ::core::ffi::c_int as usize] = 0
            as ::core::ffi::c_char;
    };
}
unsafe extern "C" fn set_error(
    mut c: *mut LandingController,
    mut msg: *const ::core::ffi::c_char,
) {
    let mut present: bool = !msg.is_null() && *msg as ::core::ffi::c_int != 0;
    (*c).publication_event_pending = ((*c).publication_event_pending
        as ::core::ffi::c_int
        | ((*c).snapshot.has_error as ::core::ffi::c_int != present as ::core::ffi::c_int
            || present as ::core::ffi::c_int != 0
                && strncmp(
                    &raw mut (*c).snapshot.last_error as *mut ::core::ffi::c_char,
                    msg,
                    (::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t)
                        .wrapping_sub(1 as size_t),
                ) != 0 as ::core::ffi::c_int) as ::core::ffi::c_int) != 0;
    (*c).snapshot.has_error = present;
    if (*c).snapshot.has_error {
        snprintf(
            &raw mut (*c).snapshot.last_error as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            msg,
        );
    } else {
        (*c).snapshot.last_error[0 as ::core::ffi::c_int as usize] = 0
            as ::core::ffi::c_char;
    };
}
unsafe extern "C" fn trajectory_equal(
    mut a: *const Trajectory,
    mut b: *const Trajectory,
) -> bool {
    if (*a).count != (*b).count {
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*a).count {
        let mut x: *const TrajectoryPoint = (*a).points.offset(i as isize)
            as *mut TrajectoryPoint;
        let mut y: *const TrajectoryPoint = (*b).points.offset(i as isize)
            as *mut TrajectoryPoint;
        if (*x).ut != (*y).ut || (*x).latitude != (*y).latitude
            || (*x).longitude != (*y).longitude || (*x).altitude != (*y).altitude
            || (*x).speed != (*y).speed
            || (*x).phase as ::core::ffi::c_uint != (*y).phase as ::core::ffi::c_uint
            || (*x).kind as ::core::ffi::c_uint != (*y).kind as ::core::ffi::c_uint
        {
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    return true_0 != 0;
}
unsafe extern "C" fn replace_reference_trajectory(
    mut c: *mut LandingController,
    mut next: *const Trajectory,
) -> bool {
    let mut source: *const Trajectory = next;
    if !c.is_null()
        && (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_ENTRY_ENERGY as ::core::ffi::c_int as ::core::ffi::c_uint
        && (source.is_null() || (*source).count < 2 as size_t)
        && (*c).has_plan as ::core::ffi::c_int != 0
        && (*c).plan.trajectory.count >= 2 as size_t
    {
        source = &raw mut (*c).plan.trajectory;
    }
    if !c.is_null()
        && (*c).guidance.phase as ::core::ffi::c_uint
            == PHASE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*c).configuration.guidance.sts_trajectory_delivery as ::core::ffi::c_int != 0
        && (source.is_null() || (*source).count < 2 as size_t)
        && (*c).snapshot.reference_trajectory.count >= 2 as size_t
        && (*c).snapshot.telemetry.ut
            - (*(*c)
                .snapshot
                .reference_trajectory
                .points
                .offset(0 as ::core::ffi::c_int as isize))
                .ut >= 0.0f64
        && (*c).snapshot.telemetry.ut
            - (*(*c)
                .snapshot
                .reference_trajectory
                .points
                .offset(0 as ::core::ffi::c_int as isize))
                .ut <= 2.5f64
    {
        source = &raw mut (*c).snapshot.reference_trajectory;
    }
    if source.is_null() {
        return false_0 != 0;
    }
    if trajectory_equal(&raw mut (*c).snapshot.reference_trajectory, source) {
        return true_0 != 0;
    }
    if !trajectory_copy(&raw mut (*c).snapshot.reference_trajectory, source) {
        return false_0 != 0;
    }
    (*c).log_reference_trajectory_dirty = true_0 != 0;
    return true_0 != 0;
}
unsafe extern "C" fn replace_predicted_trajectory(
    mut c: *mut LandingController,
    mut next: *const Trajectory,
) -> bool {
    if !c.is_null()
        && ((*c).prediction_lineage_discontinuity as ::core::ffi::c_int != 0
            || !entry_forecast_matches_guidance(c))
    {
        trajectory_clear(&raw mut (*c).snapshot.predicted_trajectory);
        return true_0 != 0;
    }
    if trajectory_equal(&raw mut (*c).snapshot.predicted_trajectory, next) {
        return true_0 != 0;
    }
    return trajectory_copy(&raw mut (*c).snapshot.predicted_trajectory, next);
}
unsafe extern "C" fn trajectory_sample_at(
    mut t: *const Trajectory,
    mut ut: ::core::ffi::c_double,
    mut out: *mut TrajectoryPoint,
) -> bool {
    if t.is_null() || out.is_null() || (*t).count == 0 {
        return false_0 != 0;
    }
    if ut < (*(*t).points.offset(0 as ::core::ffi::c_int as isize)).ut
        || ut > (*(*t).points.offset((*t).count.wrapping_sub(1 as size_t) as isize)).ut
    {
        return false_0 != 0;
    }
    if ut <= (*(*t).points.offset(0 as ::core::ffi::c_int as isize)).ut {
        *out = *(*t).points.offset(0 as ::core::ffi::c_int as isize);
        return true_0 != 0;
    }
    if ut >= (*(*t).points.offset((*t).count.wrapping_sub(1 as size_t) as isize)).ut {
        *out = *(*t).points.offset((*t).count.wrapping_sub(1 as size_t) as isize);
        return true_0 != 0;
    }
    let mut lo: size_t = 0 as size_t;
    let mut hi: size_t = (*t).count.wrapping_sub(1 as size_t);
    while hi.wrapping_sub(lo) > 1 as size_t {
        let mut mid: size_t = lo
            .wrapping_add(hi.wrapping_sub(lo).wrapping_div(2 as size_t));
        if (*(*t).points.offset(mid as isize)).ut >= ut {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let mut a: *const TrajectoryPoint = (*t).points.offset(lo as isize)
        as *mut TrajectoryPoint;
    let mut b: *const TrajectoryPoint = (*t).points.offset(hi as isize)
        as *mut TrajectoryPoint;
    let mut f: ::core::ffi::c_double = clampd(
        (ut - (*a).ut) / fmax((*b).ut - (*a).ut, 1e-9f64),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    *out = *a;
    (*out).ut = ut;
    (*out).latitude = (*a).latitude + ((*b).latitude - (*a).latitude) * f;
    (*out).longitude = norm_signed_deg(
        (*a).longitude + norm_signed_deg((*b).longitude - (*a).longitude) * f,
    );
    (*out).altitude = (*a).altitude + ((*b).altitude - (*a).altitude) * f;
    (*out).speed = (*a).speed + ((*b).speed - (*a).speed) * f;
    return true_0 != 0;
}
unsafe extern "C" fn trajectory_position_delta_at(
    mut a: *const Trajectory,
    mut b: *const Trajectory,
    mut ut: ::core::ffi::c_double,
    mut radius: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut pa: TrajectoryPoint = TrajectoryPoint {
        ut: 0.,
        latitude: 0.,
        longitude: 0.,
        altitude: 0.,
        speed: 0.,
        phase: PHASE_IDLE,
        kind: TRAJ_PLANNED,
    };
    let mut pb: TrajectoryPoint = TrajectoryPoint {
        ut: 0.,
        latitude: 0.,
        longitude: 0.,
        altitude: 0.,
        speed: 0.,
        phase: PHASE_IDLE,
        kind: TRAJ_PLANNED,
    };
    if !trajectory_sample_at(a, ut, &raw mut pa)
        || !trajectory_sample_at(b, ut, &raw mut pb)
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut ga: GeoPoint = GeoPoint {
        latitude: pa.latitude,
        longitude: pa.longitude,
        altitude: pa.altitude,
    };
    let mut gb: GeoPoint = GeoPoint {
        latitude: pb.latitude,
        longitude: pb.longitude,
        altitude: pb.altitude,
    };
    let mut horizontal: ::core::ffi::c_double = great_circle_distance(ga, gb, radius);
    return hypot(horizontal, pa.altitude - pb.altitude);
}
unsafe extern "C" fn stabilize_forecast_trajectory(
    mut stable: *mut Trajectory,
    mut raw: *const Trajectory,
    mut alpha: ::core::ffi::c_double,
) -> bool {
    if stable.is_null() || raw.is_null() {
        return false_0 != 0;
    }
    if (*stable).count == 0 || alpha >= 0.999f64 {
        return trajectory_copy(stable, raw);
    }
    let mut next: Trajectory = Trajectory {
        points: ::core::ptr::null_mut::<TrajectoryPoint>(),
        count: 0,
        capacity: 0,
    };
    trajectory_init(&raw mut next);
    alpha = clampd(
        alpha,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut i: size_t = 0 as size_t;
    while i < (*raw).count {
        let mut point: TrajectoryPoint = *(*raw).points.offset(i as isize);
        let mut old: TrajectoryPoint = TrajectoryPoint {
            ut: 0.,
            latitude: 0.,
            longitude: 0.,
            altitude: 0.,
            speed: 0.,
            phase: PHASE_IDLE,
            kind: TRAJ_PLANNED,
        };
        if trajectory_sample_at(stable, point.ut, &raw mut old) {
            point.latitude = old.latitude + (point.latitude - old.latitude) * alpha;
            point.longitude = norm_signed_deg(
                old.longitude + norm_signed_deg(point.longitude - old.longitude) * alpha,
            );
            point.speed = old.speed + (point.speed - old.speed) * alpha;
        }
        if !trajectory_append(&raw mut next, point) {
            trajectory_clear(&raw mut next);
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    trajectory_clear(stable);
    *stable = next;
    return true_0 != 0;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
#[repr(C)]
pub struct KRPCSession {
    _private: [u8; 0],
}
#[repr(C)]
pub struct PlannerLog {
    _private: [u8; 0],
}
