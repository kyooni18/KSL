use crate::sys::*;
extern "C" {
    fn jw_init(w: *mut JsonWriter);
    fn jw_free(w: *mut JsonWriter);
    fn jw_raw(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_number(w: *mut JsonWriter, value: ::core::ffi::c_double);
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn round(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn krpc_cnano_client_open(
        configuration: *const LandingConfiguration,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> *mut KrpcCNanoClient;
    fn krpc_cnano_client_close(client: *mut KrpcCNanoClient);
    fn krpc_cnano_client_planet(client: *const KrpcCNanoClient) -> *const PlanetModel;
    fn krpc_cnano_client_vessel(
        client: *const KrpcCNanoClient,
    ) -> *const ::core::ffi::c_char;
    fn krpc_cnano_client_transport_name(
        client: *const KrpcCNanoClient,
    ) -> *const ::core::ffi::c_char;
    fn krpc_cnano_client_library_version(
        client: *const KrpcCNanoClient,
    ) -> *const ::core::ffi::c_char;
    fn krpc_cnano_client_flight_key(
        client: *const KrpcCNanoClient,
    ) -> *const ::core::ffi::c_char;
    fn krpc_cnano_client_read(
        client: *mut KrpcCNanoClient,
        configuration: *const LandingConfiguration,
        telemetry: *mut Telemetry,
        state: *mut VehicleState,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_direct_controls(
        client: *mut KrpcCNanoClient,
        pitch: ::core::ffi::c_double,
        roll: ::core::ffi::c_double,
        yaw: ::core::ffi::c_double,
        throttle: ::core::ffi::c_double,
        wheel_steering: ::core::ffi::c_double,
        rcs: bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_autopilot(
        client: *mut KrpcCNanoClient,
        command: *const GuidanceCommand,
        inertial_up_reference: *const Vector3,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_surface_allocation(
        client: *mut KrpcCNanoClient,
        allocation: KrpcSurfaceAllocation,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_speedbrake(
        client: *mut KrpcCNanoClient,
        fraction: ::core::ffi::c_double,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_surface_layout(
        client: *const KrpcCNanoClient,
        buffer: *mut ::core::ffi::c_char,
        buffer_size: size_t,
    ) -> *const ::core::ffi::c_char;
    fn krpc_cnano_client_set_gear(
        client: *mut KrpcCNanoClient,
        value: bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_deploy_parachutes(
        client: *mut KrpcCNanoClient,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_shutdown_burn_engines(
        client: *mut KrpcCNanoClient,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_physics_warp(
        client: *mut KrpcCNanoClient,
        factor: *mut ::core::ffi::c_int,
        rate: *mut ::core::ffi::c_double,
        rails: *mut bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_physics_warp(
        client: *mut KrpcCNanoClient,
        factor: ::core::ffi::c_int,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_brakes(
        client: *mut KrpcCNanoClient,
        value: bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_airbrakes(
        client: *mut KrpcCNanoClient,
        group: ::core::ffi::c_uint,
        value: bool,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_set_speed_mode(
        client: *mut KrpcCNanoClient,
        mode: NavballSpeedMode,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_warp_with_progress(
        client: *mut KrpcCNanoClient,
        ut: ::core::ffi::c_double,
        progress: KRPCWarpProgress,
        context: *mut ::core::ffi::c_void,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_save(
        client: *mut KrpcCNanoClient,
        name: *const ::core::ffi::c_char,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn krpc_cnano_client_neutralize(client: *mut KrpcCNanoClient);
    fn krpc_cnano_client_safe(client: *mut KrpcCNanoClient);
    fn krpc_cnano_client_budget(client: *const KrpcCNanoClient) -> KrpcCNanoBudget;
    fn flight_control_init(
        state: *mut FlightControlState,
        nominal_dt: ::core::ffi::c_double,
    );
    fn flight_control_reset_transients(state: *mut FlightControlState);
    fn flight_control_airframe_error(
        attitude_quaternion: *const ::core::ffi::c_double,
        air_direction: Vector3,
        radial_up: Vector3,
        target_aoa_deg: ::core::ffi::c_double,
        target_bank_deg: ::core::ffi::c_double,
        use_yaw_error: bool,
        error: *mut FlightControlAttitudeError,
    ) -> bool;
    fn flight_control_step_attitude(
        state: *mut FlightControlState,
        telemetry: *const Telemetry,
        command: *const GuidanceCommand,
        attitude_error: *const FlightControlAttitudeError,
        sample_dt: ::core::ffi::c_double,
        output: *mut FlightControlOutput,
    ) -> bool;
    fn flight_control_step(
        state: *mut FlightControlState,
        telemetry: *const Telemetry,
        command: *const GuidanceCommand,
        sample_dt: ::core::ffi::c_double,
        output: *mut FlightControlOutput,
    ) -> bool;
    fn flight_control_rollout_parachutes_ready(
        telemetry: *const Telemetry,
        command: *const GuidanceCommand,
    ) -> bool;
    fn flight_control_revision() -> *const ::core::ffi::c_char;
    fn physics_store_open(
        options: *const PhysicsStoreOptions,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> *mut PhysicsStore;
    fn physics_store_close(store: *mut PhysicsStore);
    fn physics_store_flush(
        store: *mut PhysicsStore,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn physics_store_structure_id(
        store: *const PhysicsStore,
    ) -> *const ::core::ffi::c_char;
    fn physics_store_environment_id(
        store: *const PhysicsStore,
    ) -> *const ::core::ffi::c_char;
    fn physics_store_flight_key(
        store: *const PhysicsStore,
    ) -> *const ::core::ffi::c_char;
    fn physics_store_load_history(
        store: *mut PhysicsStore,
        planet: *const PlanetModel,
        out_samples: *mut VesselAeroSample,
        maximum: size_t,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> size_t;
    fn physics_store_observe(
        store: *mut PhysicsStore,
        observation: *const PhysicsStoreObservation,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn shuttle_sim_load_planet(
        atmosphere_path: *const ::core::ffi::c_char,
        planet: *mut PlanetModel,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn shuttle_sim_decode_telemetry(
        packet: *const ::core::ffi::c_char,
        planet: *const PlanetModel,
        previous: *const Telemetry,
        telemetry: *mut Telemetry,
        state: *mut VehicleState,
        error: *mut ::core::ffi::c_char,
        error_size: size_t,
    ) -> bool;
    fn shuttle_sim_prepare_guidance_telemetry(
        telemetry: *mut Telemetry,
        configuration: *const LandingConfiguration,
        planet: *const PlanetModel,
    );
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
    fn vessel_physics_import_sample(
        m: *mut VesselPhysicsModel,
        sample: *const VesselAeroSample,
    );
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vector3;
    fn vsub(a: Vector3, b: Vector3) -> Vector3;
    fn vdot(a: Vector3, b: Vector3) -> ::core::ffi::c_double;
    fn vcross(a: Vector3, b: Vector3) -> Vector3;
    fn vmag(a: Vector3) -> ::core::ffi::c_double;
    fn vnorm(a: Vector3, fallback: Vector3) -> Vector3;
    fn vproject_plane(a: Vector3, normal: Vector3) -> Vector3;
    fn telemetry_init(t: *mut Telemetry);
    fn guidance_command_init(c: *mut GuidanceCommand);
    fn speed_mode_string(s: NavballSpeedMode) -> *const ::core::ffi::c_char;
    fn profile_string(p: ControlProfile) -> *const ::core::ffi::c_char;
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __va_list_tag {
    pub gp_offset: ::core::ffi::c_uint,
    pub fp_offset: ::core::ffi::c_uint,
    pub overflow_arg_area: *mut ::core::ffi::c_void,
    pub reg_save_area: *mut ::core::ffi::c_void,
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
pub type __uint16_t = u16;
pub type __uint32_t = u32;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub struct KRPCSession {
    pub client: *mut KrpcCNanoClient,
    pub flight_control: FlightControlState,
    pub physics_store: *mut PhysicsStore,
    pub physics_history: [VesselAeroSample; 192],
    pub physics_history_count: ::core::ffi::c_uint,
    pub physics_structure_id: [::core::ffi::c_char; 128],
    pub physics_environment_id: [::core::ffi::c_char; 128],
    pub physics_storage: [::core::ffi::c_char; 1024],
    pub runtime_transport: [::core::ffi::c_char; 192],
    pub runtime_library: [::core::ffi::c_char; 64],
    pub last_control_profile: [::core::ffi::c_char; 24],
    pub last_telemetry: Telemetry,
    pub has_last_telemetry: bool,
    pub rollout_parachutes_enabled: bool,
    pub rollout_parachutes_deployed: bool,
    pub last_state: VehicleState,
    pub has_last_state: bool,
    pub last_control_ut: ::core::ffi::c_double,
    pub has_last_control_ut: bool,
    pub taem_surface_roles_latched: bool,
    pub simulator: bool,
    pub sim_rx_fd: ::core::ffi::c_int,
    pub sim_tx_fd: ::core::ffi::c_int,
    pub sim_command_addr: sockaddr_in,
    pub sim_planet: PlanetModel,
    pub sim_vessel: [::core::ffi::c_char; 128],
    pub sim_need_packet: bool,
}
pub type in_addr_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlState {
    pub nominal_dt: ::core::ffi::c_double,
    pub has_sample: bool,
    pub last_ut: ::core::ffi::c_double,
    pub last_pitch: ::core::ffi::c_double,
    pub last_aoa: ::core::ffi::c_double,
    pub last_roll: ::core::ffi::c_double,
    pub last_heading: ::core::ffi::c_double,
    pub last_sideslip: ::core::ffi::c_double,
    pub pitch_rate: ::core::ffi::c_double,
    pub aoa_rate: ::core::ffi::c_double,
    pub roll_rate: ::core::ffi::c_double,
    pub heading_rate: ::core::ffi::c_double,
    pub sideslip_rate: ::core::ffi::c_double,
    pub last_control: [::core::ffi::c_double; 3],
    pub authority: [FlightControlAxisAuthority; 3],
    pub has_yaw_observation_frame: bool,
    pub yaw_observation_frame: ::core::ffi::c_int,
    pub beta: FlightControlBetaModel,
    pub pitch_trim: ::core::ffi::c_double,
    pub applied_pitch_trim_feedforward: ::core::ffi::c_double,
    pub terminal_pitch_integral: ::core::ffi::c_double,
    pub roll_trim: ::core::ffi::c_double,
    pub terminal_pitch_authority: ::core::ffi::c_double,
    pub terminal_roll_authority: ::core::ffi::c_double,
    pub has_terminal_pitch_authority: bool,
    pub has_terminal_roll_authority: bool,
    pub last_target_pitch_state: ::core::ffi::c_double,
    pub target_pitch_rate: ::core::ffi::c_double,
    pub has_last_target_pitch: bool,
    pub last_target_roll: ::core::ffi::c_double,
    pub target_roll_rate: ::core::ffi::c_double,
    pub has_last_target_roll: bool,
    pub has_quaternion_error_sample: bool,
    pub last_quaternion_pitch_error: ::core::ffi::c_double,
    pub last_quaternion_roll_error: ::core::ffi::c_double,
    pub last_quaternion_yaw_error: ::core::ffi::c_double,
    pub quaternion_pitch_error_rate: ::core::ffi::c_double,
    pub quaternion_roll_error_rate: ::core::ffi::c_double,
    pub quaternion_yaw_error_rate: ::core::ffi::c_double,
    pub pitch_response_scale: ::core::ffi::c_double,
    pub rcs_transonic_cutoff: bool,
    pub rcs_latched: bool,
    pub rcs_q_latched: bool,
    pub rcs_saturation_latched: bool,
    pub last_profile: ControlProfile,
    pub has_last_profile: bool,
    pub main_gear_contact_latched: bool,
    pub nose_gear_contact_latched: bool,
    pub main_gear_contact_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlBetaModel {
    pub mean_yaw_rate: ::core::ffi::c_double,
    pub mean_roll_rate: ::core::ffi::c_double,
    pub mean_beta_rate: ::core::ffi::c_double,
    pub yy: ::core::ffi::c_double,
    pub rr: ::core::ffi::c_double,
    pub yr: ::core::ffi::c_double,
    pub by: ::core::ffi::c_double,
    pub br: ::core::ffi::c_double,
    pub yaw_gain: ::core::ffi::c_double,
    pub roll_gain: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub initialized: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlAxisAuthority {
    pub torque_accel: ::core::ffi::c_double,
    pub reported_torque_accel: ::core::ffi::c_double,
    pub non_aero_confidence: ::core::ffi::c_double,
    pub controlled_accel: ::core::ffi::c_double,
    pub controlled_confidence: ::core::ffi::c_double,
    pub live_response_confidence: ::core::ffi::c_double,
    pub aero_per_q: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub drift_accel: ::core::ffi::c_double,
    pub last_rate: ::core::ffi::c_double,
    pub reported_initialized: bool,
    pub has_last_rate: bool,
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
pub struct PhysicsStoreOptions {
    pub path: *const ::core::ffi::c_char,
    pub model_id: *const ::core::ffi::c_char,
    pub vessel_name: *const ::core::ffi::c_char,
    pub structure_manifest_json: *const ::core::ffi::c_char,
    pub environment_manifest_json: *const ::core::ffi::c_char,
    pub session_id: *const ::core::ffi::c_char,
    pub flight_key: *const ::core::ffi::c_char,
    pub structure_variant: *const ::core::ffi::c_char,
    pub mode: PhysicsStoreMode,
}
pub type PhysicsStoreMode = ::core::ffi::c_uint;
pub const PHYSICS_STORE_READ_WRITE: PhysicsStoreMode = 1;
pub const PHYSICS_STORE_READ_ONLY: PhysicsStoreMode = 0;
pub type ShuttleSimModelFile = ::core::ffi::c_uint;
pub const SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR: ShuttleSimModelFile = 4;
pub const SHUTTLE_SIM_MODEL_ATTITUDE: ShuttleSimModelFile = 3;
pub const SHUTTLE_SIM_MODEL_FORCE_BOOK: ShuttleSimModelFile = 2;
pub const SHUTTLE_SIM_MODEL_AERO: ShuttleSimModelFile = 1;
pub const SHUTTLE_SIM_MODEL_ATMOSPHERE: ShuttleSimModelFile = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoBudget {
    pub read_calls: ::core::ffi::c_uint,
    pub write_calls: ::core::ffi::c_uint,
    pub total_calls: ::core::ffi::c_uint,
    pub read_wire_requests: ::core::ffi::c_uint,
    pub write_wire_requests: ::core::ffi::c_uint,
    pub total_wire_requests: ::core::ffi::c_uint,
    pub last_read_seconds: ::core::ffi::c_double,
    pub last_apply_seconds: ::core::ffi::c_double,
    pub read_calls_per_second: ::core::ffi::c_double,
    pub apply_calls_per_second: ::core::ffi::c_double,
    pub read_wire_requests_per_second: ::core::ffi::c_double,
    pub apply_wire_requests_per_second: ::core::ffi::c_double,
    pub bytes_read: size_t,
    pub bytes_written: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PhysicsStoreObservation {
    pub sample_valid: bool,
    pub body_non_rotating: bool,
    pub situation: *const ::core::ffi::c_char,
    pub ut: ::core::ffi::c_double,
    pub wall_time: ::core::ffi::c_double,
    pub q: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub beta: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub has_dry_mass: bool,
    pub dry_mass: ::core::ffi::c_double,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub rcs: ::core::ffi::c_int,
    pub has_center_of_mass_root: bool,
    pub center_of_mass_root: Vector3,
    pub has_moment_of_inertia: bool,
    pub moment_of_inertia: Vector3,
    pub position: Vector3,
    pub velocity: Vector3,
    pub lift: Vector3,
    pub drag: Vector3,
    pub control_profile: *const ::core::ffi::c_char,
    pub has_body_rates: bool,
    pub body_rates: Vector3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlDiagnostics {
    pub profile: ControlProfile,
    pub target_pitch_state: ::core::ffi::c_double,
    pub measured_pitch_state: ::core::ffi::c_double,
    pub pitch_error: ::core::ffi::c_double,
    pub effective_pitch_rate: ::core::ffi::c_double,
    pub body_pitch_rate: ::core::ffi::c_double,
    pub body_pitch_rate_available: bool,
    pub roll_error: ::core::ffi::c_double,
    pub effective_roll_rate: ::core::ffi::c_double,
    pub body_roll_rate: ::core::ffi::c_double,
    pub body_roll_rate_available: bool,
    pub target_roll_rate: ::core::ffi::c_double,
    pub commanded_roll_rate: ::core::ffi::c_double,
    pub relative_roll_rate: ::core::ffi::c_double,
    pub recovery_rate_first: bool,
    pub heading_error: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub sideslip_rate: ::core::ffi::c_double,
    pub body_yaw_rate: ::core::ffi::c_double,
    pub body_yaw_rate_available: bool,
    pub flight_path_angle: ::core::ffi::c_double,
    pub pitch_trim: ::core::ffi::c_double,
    pub steady_pitch_trim: ::core::ffi::c_double,
    pub terminal_pitch_integral: ::core::ffi::c_double,
    pub roll_trim: ::core::ffi::c_double,
    pub pitch_authority: ::core::ffi::c_double,
    pub pitch_raw_authority: ::core::ffi::c_double,
    pub pitch_guard_authority: ::core::ffi::c_double,
    pub pitch_aero_fraction: ::core::ffi::c_double,
    pub pitch_fast_command_limit: ::core::ffi::c_double,
    pub pitch_hold_seconds: ::core::ffi::c_double,
    pub pitch_rate_impulse_budget: ::core::ffi::c_double,
    pub roll_authority: ::core::ffi::c_double,
    pub roll_raw_authority: ::core::ffi::c_double,
    pub roll_guard_authority: ::core::ffi::c_double,
    pub roll_aero_fraction: ::core::ffi::c_double,
    pub roll_rate_limit: ::core::ffi::c_double,
    pub roll_command_limit: ::core::ffi::c_double,
    pub roll_hold_seconds: ::core::ffi::c_double,
    pub roll_rate_impulse_budget: ::core::ffi::c_double,
    pub roll_bandwidth_scale: ::core::ffi::c_double,
    pub pitch_bandwidth_scale: ::core::ffi::c_double,
    pub yaw_authority: ::core::ffi::c_double,
    pub yaw_aero_fraction: ::core::ffi::c_double,
    pub beta_yaw_gain: ::core::ffi::c_double,
    pub beta_roll_coupling: ::core::ffi::c_double,
    pub beta_confidence: ::core::ffi::c_double,
    pub sample_dt: ::core::ffi::c_double,
    pub attitude_error: ::core::ffi::c_double,
    pub quaternion_attitude_control: bool,
    pub rcs_transonic_cutoff: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlOutput {
    pub valid: bool,
    pub pitch: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub yaw: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub wheel_steering: ::core::ffi::c_double,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: bool,
    pub rcs_assist: ::core::ffi::c_double,
    pub rcs_requested: bool,
    pub diagnostics: FlightControlDiagnostics,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FlightControlAttitudeError {
    pub valid: bool,
    pub use_yaw_error: bool,
    pub pitch_error_deg: ::core::ffi::c_double,
    pub roll_error_deg: ::core::ffi::c_double,
    pub yaw_error_deg: ::core::ffi::c_double,
}
pub type KrpcSurfaceAllocation = ::core::ffi::c_uint;
pub const KRPC_SURFACE_ALLOCATION_SEPARATED_FLAP_PITCH: KrpcSurfaceAllocation = 2;
pub const KRPC_SURFACE_ALLOCATION_SEPARATED: KrpcSurfaceAllocation = 1;
pub const KRPC_SURFACE_ALLOCATION_MIXED: KrpcSurfaceAllocation = 0;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const UINT16_MAX: ::core::ffi::c_int = 65535 as ::core::ffi::c_int;
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
macro_rules! set_error {
    ($out:expr, $out_size:expr, $fmt:expr $(, $arg:expr)* $(,)?) => {{
        let out: *mut ::core::ffi::c_char = $out;
        let out_size: size_t = $out_size;
        let fmt: *const ::core::ffi::c_char = $fmt;
        if out.is_null() || out_size == 0 as size_t {
            $( let _ = $arg; )*
        } else {
            snprintf(out, out_size, fmt $(, $arg)*);
        }
    }};
}
unsafe extern "C" fn realtime_seconds() -> ::core::ffi::c_double {
    let mut ts: timespec = timespec { tv_sec: 0, tv_nsec: 0 };
    if clock_gettime(CLOCK_REALTIME, &raw mut ts) != 0 as ::core::ffi::c_int {
        return 0.0f64;
    }
    return ts.tv_sec as ::core::ffi::c_double
        + ts.tv_nsec as ::core::ffi::c_double / 1e9f64;
}
unsafe extern "C" fn canonical_to_raw(mut value: Vector3) -> Vector3 {
    return v3(value.x, value.z, value.y);
}
unsafe extern "C" fn project_root(
    mut out: *mut ::core::ffi::c_char,
    mut out_size: size_t,
) {
    let mut configured: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_ROOT\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !configured.is_null() && *configured as ::core::ffi::c_int != 0 {
        snprintf(
            out,
            out_size,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            configured,
        );
        return;
    }
    if getcwd(out, out_size).is_null() {
        snprintf(out, out_size, b".\0" as *const u8 as *const ::core::ffi::c_char);
        return;
    }
    if access(
        b"Configuration/default.json\0" as *const u8 as *const ::core::ffi::c_char,
        R_OK,
    ) == 0 as ::core::ffi::c_int
    {
        return;
    }
    if access(
        b"../Configuration/default.json\0" as *const u8 as *const ::core::ffi::c_char,
        R_OK,
    ) == 0 as ::core::ffi::c_int
    {
        let mut parent: [::core::ffi::c_char; 1024] = [0; 1024];
        if !realpath(
                b"..\0" as *const u8 as *const ::core::ffi::c_char,
                &raw mut parent as *mut ::core::ffi::c_char,
            )
            .is_null()
        {
            snprintf(
                out,
                out_size,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                &raw mut parent as *mut ::core::ffi::c_char,
            );
        }
    }
}
unsafe extern "C" fn build_structure_manifest(
    mut cfg: *const LandingConfiguration,
    mut vessel: *const ::core::ffi::c_char,
) -> *mut ::core::ffi::c_char {
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_raw(
        &raw mut w,
        b"{\"schema\":3,\"modelId\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_string(
        &raw mut w,
        &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
    );
    jw_raw(&raw mut w, b",\"vesselName\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_string(
        &raw mut w,
        if !vessel.is_null() {
            vessel
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    let mut variant: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_PHYSICS_VARIANT\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !variant.is_null() && *variant as ::core::ffi::c_int != 0 {
        jw_raw(
            &raw mut w,
            b",\"configuration\":\0" as *const u8 as *const ::core::ffi::c_char,
        );
        jw_string(&raw mut w, variant);
    }
    jw_raw(
        &raw mut w,
        b",\"parts\":[],\"geometryComplete\":false,\"geometry\":[]}\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    let mut result: *mut ::core::ffi::c_char = if !w.failed && !w.data.is_null() {
        strdup(w.data)
    } else {
        ::core::ptr::null_mut::<::core::ffi::c_char>()
    };
    jw_free(&raw mut w);
    return result;
}
unsafe extern "C" fn build_environment_manifest(
    mut planet: *const PlanetModel,
) -> *mut ::core::ffi::c_char {
    let mut w: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut w);
    jw_raw(
        &raw mut w,
        b"{\"schema\":3,\"planet\":{\"name\":\0" as *const u8
            as *const ::core::ffi::c_char,
    );
    jw_string(&raw mut w, &raw const (*planet).name as *const ::core::ffi::c_char);
    jw_raw(&raw mut w, b",\"radius\":\0" as *const u8 as *const ::core::ffi::c_char);
    jw_number(&raw mut w, (*planet).radius);
    jw_raw(
        &raw mut w,
        b",\"gravitationalParameter\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_number(&raw mut w, (*planet).gravitational_parameter);
    jw_raw(
        &raw mut w,
        b",\"rotationalSpeed\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_number(&raw mut w, (*planet).rotational_speed);
    jw_raw(
        &raw mut w,
        b",\"atmosphereDepth\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_number(&raw mut w, (*planet).atmosphere_depth);
    jw_raw(
        &raw mut w,
        b",\"surfaceDensity\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_number(&raw mut w, (*planet).surface_density);
    jw_raw(
        &raw mut w,
        b",\"atmosphereAdiabaticIndex\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_number(&raw mut w, (*planet).atmosphere_adiabatic_index);
    jw_raw(
        &raw mut w,
        b"},\"atmosphereCurveDigest\":null}\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut result: *mut ::core::ffi::c_char = if !w.failed && !w.data.is_null() {
        strdup(w.data)
    } else {
        ::core::ptr::null_mut::<::core::ffi::c_char>()
    };
    jw_free(&raw mut w);
    return result;
}
unsafe extern "C" fn open_physics_store(
    mut session: *mut KRPCSession,
    mut configured: *const LandingConfiguration,
) {
    let mut identified: LandingConfiguration = *configured;
    snprintf(
        &raw mut identified.vehicle.model_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        shuttle_sim_resolve_vehicle_identity(
            krpc_cnano_client_vessel((*session).client),
            &raw const (*configured).vehicle.model_id as *const ::core::ffi::c_char,
        ),
    );
    let mut cfg: *const LandingConfiguration = &raw mut identified;
    let mut planet: *const PlanetModel = krpc_cnano_client_planet((*session).client);
    let mut root: [::core::ffi::c_char; 1024] = [0; 1024];
    project_root(
        &raw mut root as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
    );
    let mut runtime: [::core::ffi::c_char; 1200] = [0; 1200];
    snprintf(
        &raw mut runtime as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1200]>() as size_t,
        b"%s/Runtime\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut root as *mut ::core::ffi::c_char,
    );
    mkdir(&raw mut runtime as *mut ::core::ffi::c_char, 0o700 as __mode_t);
    let mut physics_dir: [::core::ffi::c_char; 1300] = [0; 1300];
    snprintf(
        &raw mut physics_dir as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1300]>() as size_t,
        b"%s/Physics\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut runtime as *mut ::core::ffi::c_char,
    );
    mkdir(&raw mut physics_dir as *mut ::core::ffi::c_char, 0o700 as __mode_t);
    snprintf(
        &raw mut (*session).physics_storage as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
        b"%s/observations.sqlite3\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut physics_dir as *mut ::core::ffi::c_char,
    );
    let mut structure: *mut ::core::ffi::c_char = build_structure_manifest(
        cfg,
        krpc_cnano_client_vessel((*session).client),
    );
    let mut environment: *mut ::core::ffi::c_char = build_environment_manifest(planet);
    if structure.is_null() || environment.is_null() {
        free(structure as *mut ::core::ffi::c_void);
        free(environment as *mut ::core::ffi::c_void);
        return;
    }
    let mut options: PhysicsStoreOptions = PhysicsStoreOptions {
        path: &raw mut (*session).physics_storage as *mut ::core::ffi::c_char,
        model_id: &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
        vessel_name: krpc_cnano_client_vessel((*session).client),
        structure_manifest_json: structure,
        environment_manifest_json: environment,
        session_id: ::core::ptr::null::<::core::ffi::c_char>(),
        flight_key: krpc_cnano_client_flight_key((*session).client),
        structure_variant: getenv(
            b"KSP_LANDER_PHYSICS_VARIANT\0" as *const u8 as *const ::core::ffi::c_char,
        ),
        mode: PHYSICS_STORE_READ_WRITE,
    };
    let mut checkpoint: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_FORCE_REENTRY_TEST\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut unpowered: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_UNPOWERED_ONLY\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut checkpoint_attempt: bool = !checkpoint.is_null()
        && strcmp(checkpoint, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int && !unpowered.is_null()
        && strcmp(unpowered, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int;
    if checkpoint_attempt {
        options.flight_key = ::core::ptr::null::<::core::ffi::c_char>();
    }
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
    (*session).physics_store = physics_store_open(
        &raw mut options,
        &raw mut error as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
    );
    free(structure as *mut ::core::ffi::c_void);
    free(environment as *mut ::core::ffi::c_void);
    if (*session).physics_store.is_null() {
        snprintf(
            &raw mut (*session).physics_structure_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"model:%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
        );
        snprintf(
            &raw mut (*session).physics_environment_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"unavailable\0" as *const u8 as *const ::core::ffi::c_char,
        );
        let mut disabled: [::core::ffi::c_char; 1024] = [0; 1024];
        snprintf(
            &raw mut disabled as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
            b"disabled: %.900s\0" as *const u8 as *const ::core::ffi::c_char,
            if error[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0 {
                &raw mut error as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
            } else {
                b"open failed\0" as *const u8 as *const ::core::ffi::c_char
            },
        );
        snprintf(
            &raw mut (*session).physics_storage as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut disabled as *mut ::core::ffi::c_char,
        );
        return;
    }
    snprintf(
        &raw mut (*session).physics_structure_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        physics_store_structure_id((*session).physics_store),
    );
    snprintf(
        &raw mut (*session).physics_environment_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        physics_store_environment_id((*session).physics_store),
    );
    (*session).physics_history_count = physics_store_load_history(
        (*session).physics_store,
        planet,
        &raw mut (*session).physics_history as *mut VesselAeroSample,
        VESSEL_AERO_SAMPLES as size_t,
        &raw mut error as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
    ) as ::core::ffi::c_uint;
    if checkpoint_attempt {
        fprintf(
            stderr,
            b"Checkpoint aerodynamic prior: attempt %s, %u prior cells; current attempt held out.\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            physics_store_flight_key((*session).physics_store),
            (*session).physics_history_count,
        );
    }
}
unsafe extern "C" fn open_sim_physics_store(
    mut session: *mut KRPCSession,
    mut cfg: *const LandingConfiguration,
) {
    let mut root: [::core::ffi::c_char; 1024] = [0; 1024];
    project_root(
        &raw mut root as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
    );
    let mut runtime: [::core::ffi::c_char; 1200] = [0; 1200];
    snprintf(
        &raw mut runtime as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1200]>() as size_t,
        b"%s/Runtime\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut root as *mut ::core::ffi::c_char,
    );
    let mut physics_dir: [::core::ffi::c_char; 1300] = [0; 1300];
    snprintf(
        &raw mut physics_dir as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1300]>() as size_t,
        b"%s/Physics\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut runtime as *mut ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*session).physics_storage as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
        b"%s/observations.sqlite3\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut physics_dir as *mut ::core::ffi::c_char,
    );
    let mut structure: *mut ::core::ffi::c_char = build_structure_manifest(
        cfg,
        &raw mut (*session).sim_vessel as *mut ::core::ffi::c_char,
    );
    let mut environment: *mut ::core::ffi::c_char = build_environment_manifest(
        &raw mut (*session).sim_planet,
    );
    if structure.is_null() || environment.is_null() {
        free(structure as *mut ::core::ffi::c_void);
        free(environment as *mut ::core::ffi::c_void);
        return;
    }
    let mut options: PhysicsStoreOptions = PhysicsStoreOptions {
        path: &raw mut (*session).physics_storage as *mut ::core::ffi::c_char,
        model_id: &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
        vessel_name: &raw mut (*session).sim_vessel as *mut ::core::ffi::c_char,
        structure_manifest_json: structure,
        environment_manifest_json: environment,
        session_id: ::core::ptr::null::<::core::ffi::c_char>(),
        flight_key: b"shuttlesim-holdout\0" as *const u8 as *const ::core::ffi::c_char,
        structure_variant: ::core::ptr::null::<::core::ffi::c_char>(),
        mode: PHYSICS_STORE_READ_ONLY,
    };
    let mut err: [::core::ffi::c_char; 512] = [
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
    (*session).physics_store = physics_store_open(
        &raw mut options,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
    );
    free(structure as *mut ::core::ffi::c_void);
    free(environment as *mut ::core::ffi::c_void);
    if (*session).physics_store.is_null() {
        snprintf(
            &raw mut (*session).physics_structure_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"model:%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char,
        );
        snprintf(
            &raw mut (*session).physics_environment_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"sim-kerbin\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return;
    }
    snprintf(
        &raw mut (*session).physics_structure_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        physics_store_structure_id((*session).physics_store),
    );
    snprintf(
        &raw mut (*session).physics_environment_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        physics_store_environment_id((*session).physics_store),
    );
    (*session).physics_history_count = physics_store_load_history(
        (*session).physics_store,
        &raw mut (*session).sim_planet,
        &raw mut (*session).physics_history as *mut VesselAeroSample,
        VESSEL_AERO_SAMPLES as size_t,
        &raw mut err as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
    ) as ::core::ffi::c_uint;
}
unsafe extern "C" fn sim_load_force_book(mut s: *mut KRPCSession) {
    let mut root: [::core::ffi::c_char; 1024] = [0; 1024];
    project_root(
        &raw mut root as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
    );
    let mut path: [::core::ffi::c_char; 1400] = [0; 1400];
    if shuttle_sim_model_path(
            &raw mut root as *mut ::core::ffi::c_char,
            SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR,
            &raw mut path as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1400]>() as size_t,
        )
        .is_null()
    {
        return;
    }
    let mut configured: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_CERTIFIED_PRIOR\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !configured.is_null()
        && strcmp(configured, b"none\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
    {
        return;
    }
    let mut source: *const ::core::ffi::c_char = if !configured.is_null()
        && *configured as ::core::ffi::c_int != 0
    {
        configured
    } else {
        &raw mut path as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
    };
    let mut f: *mut FILE = fopen(
        source,
        b"r\0" as *const u8 as *const ::core::ffi::c_char,
    ) as *mut FILE;
    if f.is_null() {
        return;
    }
    let mut candidates: [VesselAeroSample; 256] = [VesselAeroSample {
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
    }; 256];
    let mut n: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    let mut line: [::core::ffi::c_char; 512] = [0; 512];
    while !fgets(
            &raw mut line as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as ::core::ffi::c_int,
            f,
        )
        .is_null() && n < 256 as ::core::ffi::c_uint
    {
        if !strstr(
                &raw mut line as *mut ::core::ffi::c_char,
                b"q_pa\0" as *const u8 as *const ::core::ffi::c_char,
            )
            .is_null()
        {
            continue;
        }
        let mut q: ::core::ffi::c_double = 0.;
        let mut mach: ::core::ffi::c_double = 0.;
        let mut aoa: ::core::ffi::c_double = 0.;
        let mut lift_per_q: ::core::ffi::c_double = 0.;
        let mut drag_per_q: ::core::ffi::c_double = 0.;
        let mut support: ::core::ffi::c_double = 1 as ::core::ffi::c_int
            as ::core::ffi::c_double;
        let mut got: ::core::ffi::c_int = sscanf(
            &raw mut line as *mut ::core::ffi::c_char,
            b"%lf,%lf,%lf,%lf,%lf,%lf\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut q,
            &raw mut mach,
            &raw mut aoa,
            &raw mut lift_per_q,
            &raw mut drag_per_q,
            &raw mut support,
        );
        if got < 5 as ::core::ffi::c_int || q.is_finite() as i32 == 0
            || q <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
            || mach.is_finite() as i32 == 0 || aoa.is_finite() as i32 == 0
            || lift_per_q.is_finite() as i32 == 0 || drag_per_q.is_finite() as i32 == 0
        {
            continue;
        }
        let mut obs: ::core::ffi::c_uint = fmax(
            1.0f64,
            round(if got >= 6 as ::core::ffi::c_int { support } else { 1.0f64 }),
        ) as ::core::ffi::c_uint;
        let fresh0 = n;
        n = n.wrapping_add(1);
        candidates[fresh0 as usize] = VesselAeroSample {
            q: q,
            mach: mach,
            aoa: aoa,
            beta: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mass: 40252.91796875f64,
            force_per_q: v3(
                fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, drag_per_q),
                fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, lift_per_q),
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
            gear: false_0 != 0,
            brakes: false_0 != 0,
            airbrakes: 0 as ::core::ffi::c_int,
            observations: obs,
            trust: clampd(
                0.72f64 + 0.035f64 * obs as ::core::ffi::c_double,
                0.72f64,
                1.0f64,
            ),
            last_observation_ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        };
    }
    fclose(f);
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < n {
        let mut j: ::core::ffi::c_uint = i.wrapping_add(1 as ::core::ffi::c_uint);
        while j < n {
            let mut swap: bool = candidates[j as usize].observations
                > candidates[i as usize].observations
                || candidates[j as usize].observations
                    == candidates[i as usize].observations
                    && candidates[j as usize].trust > candidates[i as usize].trust;
            if swap {
                let mut tmp: VesselAeroSample = candidates[i as usize];
                candidates[i as usize] = candidates[j as usize];
                candidates[j as usize] = tmp;
            }
            j = j.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    (*s).physics_history_count = if n < VESSEL_AERO_SAMPLES as ::core::ffi::c_uint {
        n
    } else {
        VESSEL_AERO_SAMPLES as ::core::ffi::c_uint
    };
    let mut i_0: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i_0 < (*s).physics_history_count {
        (*s).physics_history[i_0 as usize] = candidates[i_0 as usize];
        i_0 = i_0.wrapping_add(1);
    }
    snprintf(
        &raw mut (*s).physics_structure_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"model:STS-N:shuttlesim-force-book\0" as *const u8 as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*s).physics_environment_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"Kerbin:ShuttleSim-calibrated\0" as *const u8 as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*s).physics_storage as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        source,
    );
}
unsafe extern "C" fn sim_session_open(
    mut s: *mut KRPCSession,
    mut cfg: *const LandingConfiguration,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    (*s).simulator = true_0 != 0;
    (*s).sim_rx_fd = -(1 as ::core::ffi::c_int);
    (*s).sim_tx_fd = -(1 as ::core::ffi::c_int);
    (*s).sim_need_packet = true_0 != 0;
    let mut root: [::core::ffi::c_char; 1024] = [0; 1024];
    let mut atmosphere: [::core::ffi::c_char; 1400] = [0; 1400];
    project_root(
        &raw mut root as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
    );
    if shuttle_sim_model_path(
            &raw mut root as *mut ::core::ffi::c_char,
            SHUTTLE_SIM_MODEL_ATMOSPHERE,
            &raw mut atmosphere as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 1400]>() as size_t,
        )
        .is_null()
    {
        set_error!(
            error,
            error_size,
            b"ShuttleSim atmosphere path is too long\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut configured: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_TERMINAL_ATMOSPHERE\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !shuttle_sim_load_planet(
        if !configured.is_null() && *configured as ::core::ffi::c_int != 0 {
            configured
        } else {
            &raw mut atmosphere as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
        },
        &raw mut (*s).sim_planet,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    snprintf(
        &raw mut (*s).sim_vessel as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if (*cfg).vehicle.model_id[0 as ::core::ffi::c_int as usize]
            as ::core::ffi::c_int != 0
        {
            &raw const (*cfg).vehicle.model_id as *const ::core::ffi::c_char
        } else {
            b"STS-N\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    let mut ports: [::core::ffi::c_int; 2] = [
        8796 as ::core::ffi::c_int,
        8795 as ::core::ffi::c_int,
    ];
    let mut names: [*const ::core::ffi::c_char; 2] = [
        b"KSP_LANDER_SIM_TELEMETRY_PORT\0" as *const u8 as *const ::core::ffi::c_char,
        b"KSP_LANDER_SIM_COMMAND_PORT\0" as *const u8 as *const ::core::ffi::c_char,
    ];
    let mut i: size_t = 0 as size_t;
    while i
        < (::core::mem::size_of::<[::core::ffi::c_int; 2]>() as usize)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_int>() as usize)
    {
        let mut value: *const ::core::ffi::c_char = getenv(names[i as usize]);
        if !value.is_null() && *value as ::core::ffi::c_int != 0 {
            let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                ::core::ffi::c_char,
            >();
            *__errno_location() = 0 as ::core::ffi::c_int;
            let mut parsed: ::core::ffi::c_long = strtol(
                value,
                &raw mut end,
                10 as ::core::ffi::c_int,
            );
            if *__errno_location() != 0 || end == value as *mut ::core::ffi::c_char
                || *end as ::core::ffi::c_int != 0 || parsed < 1 as ::core::ffi::c_long
                || parsed > UINT16_MAX as ::core::ffi::c_long
            {
                set_error!(
                    error,
                    error_size,
                    b"Invalid ShuttleSim UDP port in %s\0" as *const u8
                        as *const ::core::ffi::c_char,
                    names[i as usize],
                );
                return false_0 != 0;
            }
            ports[i as usize] = parsed as ::core::ffi::c_int;
        }
        i = i.wrapping_add(1);
    }
    let mut rx_port: ::core::ffi::c_int = ports[0 as ::core::ffi::c_int as usize];
    let mut tx_port: ::core::ffi::c_int = ports[1 as ::core::ffi::c_int as usize];
    if rx_port == tx_port {
        set_error!(
            error,
            error_size,
            b"ShuttleSim command and telemetry ports must differ\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    (*s).sim_rx_fd = socket(
        AF_INET,
        SOCK_DGRAM as ::core::ffi::c_int,
        0 as ::core::ffi::c_int,
    );
    (*s).sim_tx_fd = socket(
        AF_INET,
        SOCK_DGRAM as ::core::ffi::c_int,
        0 as ::core::ffi::c_int,
    );
    if (*s).sim_rx_fd < 0 as ::core::ffi::c_int
        || (*s).sim_tx_fd < 0 as ::core::ffi::c_int
    {
        set_error!(
            error,
            error_size,
            b"Could not create ShuttleSim UDP sockets\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut rx: sockaddr_in = sockaddr_in {
        sin_family: 0 as sa_family_t,
        sin_port: 0,
        sin_addr: in_addr { s_addr: 0 },
        sin_zero: [0; 8],
    };
    rx.sin_family = AF_INET as sa_family_t;
    rx.sin_addr.s_addr = __bswap_32(0x7f000001 as ::core::ffi::c_int as __uint32_t)
        as in_addr_t;
    rx.sin_port = __bswap_16(rx_port as __uint16_t) as in_port_t;
    if bind(
        (*s).sim_rx_fd,
        &raw mut rx as *mut sockaddr,
        ::core::mem::size_of::<sockaddr_in>() as socklen_t,
    ) < 0 as ::core::ffi::c_int
    {
        set_error!(
            error,
            error_size,
            b"Could not bind ShuttleSim telemetry UDP %d: %s\0" as *const u8
                as *const ::core::ffi::c_char,
            rx_port,
            strerror(*__errno_location()),
        );
        return false_0 != 0;
    }
    let mut tv: timeval = timeval {
        tv_sec: 10 as __time_t,
        tv_usec: 0 as __suseconds_t,
    };
    setsockopt(
        (*s).sim_rx_fd,
        SOL_SOCKET,
        SO_RCVTIMEO,
        &raw mut tv as *const ::core::ffi::c_void,
        ::core::mem::size_of::<timeval>() as socklen_t,
    );
    memset(
        &raw mut (*s).sim_command_addr as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<sockaddr_in>() as size_t,
    );
    (*s).sim_command_addr.sin_family = AF_INET as sa_family_t;
    (*s).sim_command_addr.sin_port = __bswap_16(tx_port as __uint16_t) as in_port_t;
    inet_pton(
        AF_INET,
        b"127.0.0.1\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut (*s).sim_command_addr.sin_addr as *mut ::core::ffi::c_void,
    );
    flight_control_init(
        &raw mut (*s).flight_control,
        1.0f64
            / clampd(
                (*cfg).guidance.guidance_rate,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
                30 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
    );
    snprintf(
        &raw mut (*s).runtime_transport as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"shuttlesim-udp-lockstep\0" as *const u8 as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*s).runtime_library as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"ShuttleSim\0" as *const u8 as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*s).last_control_profile as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"entry\0" as *const u8 as *const ::core::ffi::c_char,
    );
    open_sim_physics_store(s, cfg);
    sim_load_force_book(s);
    return true_0 != 0;
}
unsafe extern "C" fn sim_read_telemetry(
    mut s: *mut KRPCSession,
    mut t: *mut Telemetry,
    mut state: *mut VehicleState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !(*s).sim_need_packet && (*s).has_last_telemetry as ::core::ffi::c_int != 0
        && (*s).has_last_state as ::core::ffi::c_int != 0
    {
        *t = (*s).last_telemetry;
        *state = (*s).last_state;
        return true_0 != 0;
    }
    let mut last_ut: ::core::ffi::c_double = if (*s).has_last_telemetry
        as ::core::ffi::c_int != 0
    {
        (*s).last_telemetry.ut
    } else {
        -::core::f32::INFINITY as ::core::ffi::c_double
    };
    loop {
        let mut buf: [::core::ffi::c_char; 8192] = [0; 8192];
        let mut n: ssize_t = recvfrom(
            (*s).sim_rx_fd,
            &raw mut buf as *mut ::core::ffi::c_char as *mut ::core::ffi::c_void,
            (::core::mem::size_of::<[::core::ffi::c_char; 8192]>() as size_t)
                .wrapping_sub(1 as size_t),
            0 as ::core::ffi::c_int,
            ::core::ptr::null_mut::<sockaddr>(),
            ::core::ptr::null_mut::<socklen_t>(),
        );
        if n <= 0 as ssize_t {
            set_error!(
                error,
                error_size,
                b"Timed out waiting for ShuttleSim telemetry: %s\0" as *const u8
                    as *const ::core::ffi::c_char,
                strerror(*__errno_location()),
            );
            return false_0 != 0;
        }
        buf[n as usize] = 0 as ::core::ffi::c_char;
        let mut candidate: Telemetry = Telemetry {
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
        let mut candidate_state: VehicleState = VehicleState {
            ut: 0.,
            position: Vector3 { x: 0., y: 0., z: 0. },
            velocity: Vector3 { x: 0., y: 0., z: 0. },
            mass: 0.,
        };
        telemetry_init(&raw mut candidate);
        memset(
            &raw mut candidate_state as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<VehicleState>() as size_t,
        );
        if !shuttle_sim_decode_telemetry(
            &raw mut buf as *mut ::core::ffi::c_char,
            &raw mut (*s).sim_planet,
            if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
                &raw mut (*s).last_telemetry
            } else {
                ::core::ptr::null_mut::<Telemetry>()
            },
            &raw mut candidate,
            &raw mut candidate_state,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if !(*s).has_last_telemetry || last_ut.is_finite() as i32 == 0
            || candidate.ut > last_ut + 1e-6f64
        {
            *t = candidate;
            *state = candidate_state;
            (*s).sim_need_packet = false_0 != 0;
            return true_0 != 0;
        }
        let mut diag: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_SIM_STEP_DIAG\0" as *const u8 as *const ::core::ffi::c_char,
        );
        if !diag.is_null()
            && strcmp(diag, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int
        {
            fprintf(
                stderr,
                b"ShuttleSim duplicate telemetry discarded: UT %.6f <= %.6f.\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                candidate.ut,
                last_ut,
            );
        }
    };
}
unsafe extern "C" fn sim_direct_control() -> bool {
    let mut v: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_SIM_DIRECT\0" as *const u8 as *const ::core::ffi::c_char,
    );
    return !v.is_null()
        && strcmp(v, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn sim_send_guidance(
    mut s: *mut KRPCSession,
    mut command: *const GuidanceCommand,
    mut step: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    return sim_send_guidance_inputs(
        s,
        command,
        ::core::ptr::null::<FlightControlOutput>(),
        step,
        error,
        error_size,
    );
}
unsafe extern "C" fn sim_send_guidance_inputs(
    mut s: *mut KRPCSession,
    mut command: *const GuidanceCommand,
    mut fc: *const FlightControlOutput,
    mut step: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut aoa: ::core::ffi::c_double = if (*s).has_last_telemetry as ::core::ffi::c_int
        != 0
    {
        (*s).last_telemetry.angle_of_attack
    } else {
        0.0f64
    };
    let mut bank: ::core::ffi::c_double = if (*s).has_last_telemetry
        as ::core::ffi::c_int != 0
    {
        (*s).last_telemetry.roll
    } else {
        0.0f64
    };
    let mut throttle: ::core::ffi::c_double = if !command.is_null()
        && (*command).control_profile as ::core::ffi::c_uint
            == PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        clampd((*command).target_throttle, 0.0f64, 1.0f64)
    } else {
        0.0f64
    };
    let mut gear: bool = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
        (*s).last_telemetry.gear as ::core::ffi::c_int
    } else {
        false_0
    } != 0;
    let mut brakes: bool = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
        (*s).last_telemetry.brakes as ::core::ffi::c_int
    } else {
        false_0
    } != 0;
    let mut airbrakes: bool = false_0 != 0;
    let mut wheel_steering: ::core::ffi::c_double = 0.0f64;
    let mut speedbrake: ::core::ffi::c_double = 0.0f64;
    if !command.is_null() {
        if (*command).has_target_aoa as ::core::ffi::c_int != 0
            && (*command).target_aoa.is_finite() as i32 != 0
        {
            aoa = (*command).target_aoa;
        }
        if (*command).target_roll.is_finite() as i32 != 0 {
            bank = (*command).target_roll;
        }
        gear = (*command).gear;
        brakes = (*command).brakes;
        airbrakes = (*command).airbrakes;
        wheel_steering = clampd((*command).wheel_steering, -1.0f64, 1.0f64);
        if (*command).speedbrake_fraction.is_finite() as i32 != 0 {
            speedbrake = clampd((*command).speedbrake_fraction, 0.0f64, 1.0f64);
        }
    }
    if !fc.is_null() {
        wheel_steering = clampd((*fc).wheel_steering, -1.0f64, 1.0f64);
    }
    let mut thrust_dir: [::core::ffi::c_char; 120] = ::core::mem::transmute::<
        [u8; 120],
        [::core::ffi::c_char; 120],
    >(
        *b"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
    );
    if !command.is_null()
        && (*command).control_profile as ::core::ffi::c_uint
            == PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*command).use_inertial_direction as ::core::ffi::c_int != 0
        && throttle > 0.0f64 && (*command).inertial_direction.x.is_finite() as i32 != 0
        && (*command).inertial_direction.y.is_finite() as i32 != 0
        && (*command).inertial_direction.z.is_finite() as i32 != 0
    {
        snprintf(
            &raw mut thrust_dir as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 120]>() as size_t,
            b",\"thrust_dir\":[%.9f,%.9f,%.9f]\0" as *const u8
                as *const ::core::ffi::c_char,
            (*command).inertial_direction.x,
            (*command).inertial_direction.y,
            (*command).inertial_direction.z,
        );
    }
    let mut inputs: [::core::ffi::c_char; 160] = ::core::mem::transmute::<
        [u8; 160],
        [::core::ffi::c_char; 160],
    >(
        *b"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
    );
    if !fc.is_null() {
        snprintf(
            &raw mut inputs as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 160]>() as size_t,
            b",\"pitch_input\":%.9f,\"roll_input\":%.9f,\"yaw_input\":%.9f\0"
                as *const u8 as *const ::core::ffi::c_char,
            clampd((*fc).pitch, -1.0f64, 1.0f64),
            clampd((*fc).roll, -1.0f64, 1.0f64),
            clampd((*fc).yaw, -1.0f64, 1.0f64),
        );
    }
    let mut msg: [::core::ffi::c_char; 800] = [0; 800];
    snprintf(
        &raw mut msg as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 800]>() as size_t,
        b"{\"type\":\"guidance\",\"aoa_deg\":%.9f,\"bank_deg\":%.9f,\"gear_down\":%s,\"brakes\":%s,\"airbrakes\":%s,\"speedbrake\":%.6f,\"wheel_steering\":%.9f,\"throttle\":%.9f%s%s,\"step\":%s}\0"
            as *const u8 as *const ::core::ffi::c_char,
        aoa,
        bank,
        if gear as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        if brakes as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        if airbrakes as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
        speedbrake,
        wheel_steering,
        throttle,
        &raw mut thrust_dir as *mut ::core::ffi::c_char,
        &raw mut inputs as *mut ::core::ffi::c_char,
        if step as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    let mut n: ssize_t = sendto(
        (*s).sim_tx_fd,
        &raw mut msg as *mut ::core::ffi::c_char as *const ::core::ffi::c_void,
        strlen(&raw mut msg as *mut ::core::ffi::c_char),
        0 as ::core::ffi::c_int,
        &raw mut (*s).sim_command_addr as *mut sockaddr,
        ::core::mem::size_of::<sockaddr_in>() as socklen_t,
    );
    if n < 0 as ssize_t {
        set_error!(
            error,
            error_size,
            b"Could not send ShuttleSim command: %s\0" as *const u8
                as *const ::core::ffi::c_char,
            strerror(*__errno_location()),
        );
        return false_0 != 0;
    }
    if step {
        (*s).sim_need_packet = true_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_open(
    mut cfg: *const LandingConfiguration,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> *mut KRPCSession {
    if cfg.is_null() {
        set_error!(
            error,
            error_size,
            b"Landing configuration is missing\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<KRPCSession>();
    }
    let mut session: *mut KRPCSession = calloc(
        1 as size_t,
        ::core::mem::size_of::<KRPCSession>() as size_t,
    ) as *mut KRPCSession;
    if session.is_null() {
        set_error!(
            error,
            error_size,
            b"Out of memory opening native kRPC session\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<KRPCSession>();
    }
    (*session).rollout_parachutes_enabled = (*cfg).guidance.rollout_parachutes;
    (*session).sim_tx_fd = -(1 as ::core::ffi::c_int);
    (*session).sim_rx_fd = (*session).sim_tx_fd;
    let mut sim: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_SIMULATOR\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !sim.is_null()
        && strcmp(sim, b"1\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int
    {
        if !sim_session_open(session, cfg, error, error_size) {
            krpc_session_close(session);
            return ::core::ptr::null_mut::<KRPCSession>();
        }
        return session;
    }
    (*session).client = krpc_cnano_client_open(cfg, error, error_size);
    if (*session).client.is_null() {
        free(session as *mut ::core::ffi::c_void);
        return ::core::ptr::null_mut::<KRPCSession>();
    }
    flight_control_init(
        &raw mut (*session).flight_control,
        1.0f64
            / clampd(
                (*cfg).guidance.guidance_rate,
                2 as ::core::ffi::c_int as ::core::ffi::c_double,
                30 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
    );
    snprintf(
        &raw mut (*session).runtime_transport as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        krpc_cnano_client_transport_name((*session).client),
    );
    snprintf(
        &raw mut (*session).runtime_library as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        krpc_cnano_client_library_version((*session).client),
    );
    snprintf(
        &raw mut (*session).last_control_profile as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"orbital\0" as *const u8 as *const ::core::ffi::c_char,
    );
    open_physics_store(session, cfg);
    return session;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_planet(
    mut s: *const KRPCSession,
) -> *const PlanetModel {
    if s.is_null() {
        return ::core::ptr::null::<PlanetModel>();
    }
    return if (*s).simulator as ::core::ffi::c_int != 0 {
        &raw const (*s).sim_planet
    } else if !(*s).client.is_null() {
        krpc_cnano_client_planet((*s).client)
    } else {
        ::core::ptr::null::<PlanetModel>()
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_vessel(
    mut s: *const KRPCSession,
) -> *const ::core::ffi::c_char {
    if s.is_null() {
        return b"\0" as *const u8 as *const ::core::ffi::c_char;
    }
    return if (*s).simulator as ::core::ffi::c_int != 0 {
        &raw const (*s).sim_vessel as *const ::core::ffi::c_char
    } else if !(*s).client.is_null() {
        krpc_cnano_client_vessel((*s).client)
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_transport(
    mut s: *const KRPCSession,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).runtime_transport as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_library_version(
    mut s: *const KRPCSession,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).runtime_library as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_is_simulator(mut s: *const KRPCSession) -> bool {
    return !s.is_null() && (*s).simulator as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_transport_budget(
    mut s: *const KRPCSession,
) -> KRPCTransportBudget {
    let mut out: KRPCTransportBudget = KRPCTransportBudget {
        read_calls: 0 as ::core::ffi::c_uint,
        read_wire_requests: 0,
        apply_calls: 0,
        apply_wire_requests: 0,
        total_calls: 0,
        total_wire_requests: 0,
        read_seconds: 0.,
        apply_seconds: 0.,
    };
    if s.is_null() || (*s).simulator as ::core::ffi::c_int != 0 || (*s).client.is_null()
    {
        return out;
    }
    let mut in_0: KrpcCNanoBudget = krpc_cnano_client_budget((*s).client);
    out.read_calls = in_0.read_calls;
    out.read_wire_requests = in_0.read_wire_requests;
    out.apply_calls = in_0.write_calls;
    out.apply_wire_requests = in_0.write_wire_requests;
    out.total_calls = in_0.total_calls;
    out.total_wire_requests = in_0.total_wire_requests;
    out.read_seconds = in_0.last_read_seconds;
    out.apply_seconds = in_0.last_apply_seconds;
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_physics_structure_id(
    mut s: *const KRPCSession,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).physics_structure_id as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_physics_environment_id(
    mut s: *const KRPCSession,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).physics_environment_id as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_physics_storage(
    mut s: *const KRPCSession,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).physics_storage as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_physics_history_count(
    mut s: *const KRPCSession,
) -> ::core::ffi::c_uint {
    return if !s.is_null() {
        (*s).physics_history_count
    } else {
        0 as ::core::ffi::c_uint
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_seed_physics(
    mut s: *const KRPCSession,
    mut model: *mut VesselPhysicsModel,
) {
    if s.is_null() || model.is_null() {
        return;
    }
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < (*s).physics_history_count {
        vessel_physics_import_sample(
            model,
            (&raw const (*s).physics_history as *const VesselAeroSample)
                .offset(i as isize) as *const VesselAeroSample,
        );
        i = i.wrapping_add(1);
    }
}
unsafe extern "C" fn store_observation(
    mut s: *mut KRPCSession,
    mut t: *const Telemetry,
    mut state: *const VehicleState,
) {
    if s.is_null() || (*s).physics_store.is_null() || t.is_null() || state.is_null() {
        return;
    }
    let mut o: PhysicsStoreObservation = PhysicsStoreObservation {
        sample_valid: 0 as ::core::ffi::c_int != 0,
        body_non_rotating: false,
        situation: ::core::ptr::null::<::core::ffi::c_char>(),
        ut: 0.,
        wall_time: 0.,
        q: 0.,
        mach: 0.,
        aoa: 0.,
        beta: 0.,
        roll: 0.,
        mass: 0.,
        has_dry_mass: false,
        dry_mass: 0.,
        gear: false,
        brakes: false,
        airbrakes: 0,
        rcs: 0,
        has_center_of_mass_root: false,
        center_of_mass_root: Vector3 { x: 0., y: 0., z: 0. },
        has_moment_of_inertia: false,
        moment_of_inertia: Vector3 { x: 0., y: 0., z: 0. },
        position: Vector3 { x: 0., y: 0., z: 0. },
        velocity: Vector3 { x: 0., y: 0., z: 0. },
        lift: Vector3 { x: 0., y: 0., z: 0. },
        drag: Vector3 { x: 0., y: 0., z: 0. },
        control_profile: ::core::ptr::null::<::core::ffi::c_char>(),
        has_body_rates: false,
        body_rates: Vector3 { x: 0., y: 0., z: 0. },
    };
    o.sample_valid = (*t).physics_sample_valid;
    o.body_non_rotating = true_0 != 0;
    o.situation = &raw const (*t).vessel_situation as *const ::core::ffi::c_char;
    o.ut = (*t).ut;
    o.wall_time = realtime_seconds();
    o.q = (*t).dynamic_pressure;
    o.mach = (*t).mach;
    o.aoa = (*t).angle_of_attack;
    o.beta = (*t).sideslip;
    o.roll = (*t).roll;
    o.mass = (*t).mass;
    o.has_dry_mass = (*t).dry_mass.is_finite() as i32 != 0
        && (*t).dry_mass > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    o.dry_mass = (*t).dry_mass;
    o.gear = (*t).gear;
    o.brakes = (*t).brakes;
    o.airbrakes = if (*t).has_airbrakes as ::core::ffi::c_int != 0 {
        if (*t).airbrakes as ::core::ffi::c_int != 0 {
            1 as ::core::ffi::c_int
        } else {
            0 as ::core::ffi::c_int
        }
    } else {
        -(1 as ::core::ffi::c_int)
    };
    o.rcs = -(1 as ::core::ffi::c_int);
    o.position = canonical_to_raw((*state).position);
    o.velocity = canonical_to_raw((*state).velocity);
    o.lift = canonical_to_raw((*t).lift_vector);
    o.drag = canonical_to_raw((*t).drag_vector);
    o.has_center_of_mass_root = (*t).has_center_of_mass_root;
    o.center_of_mass_root = (*t).center_of_mass_root;
    o.has_moment_of_inertia = (*t).has_inertia;
    o.moment_of_inertia = v3(
        (*t).pitch_moment_of_inertia,
        (*t).roll_moment_of_inertia,
        (*t).yaw_moment_of_inertia,
    );
    o.control_profile = &raw mut (*s).last_control_profile as *mut ::core::ffi::c_char;
    o.has_body_rates = (*t).has_body_pitch_rate as ::core::ffi::c_int != 0
        && (*t).has_body_roll_rate as ::core::ffi::c_int != 0
        && (*t).has_body_yaw_rate as ::core::ffi::c_int != 0;
    o.body_rates = v3((*t).body_pitch_rate, (*t).body_roll_rate, (*t).body_yaw_rate);
    let mut ignored: [::core::ffi::c_char; 256] = [0; 256];
    physics_store_observe(
        (*s).physics_store,
        &raw mut o,
        &raw mut ignored as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
    );
}
#[no_mangle]
pub unsafe extern "C" fn krpc_read_telemetry(
    mut s: *mut KRPCSession,
    mut cfg: *const LandingConfiguration,
    mut t: *mut Telemetry,
    mut state: *mut VehicleState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if s.is_null() {
        set_error!(
            error,
            error_size,
            b"Flight session is closed\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if (*s).simulator {
        if !sim_read_telemetry(s, t, state, error, error_size) {
            return false_0 != 0;
        }
        shuttle_sim_prepare_guidance_telemetry(t, cfg, &raw mut (*s).sim_planet);
        (*s).last_telemetry = *t;
        (*s).has_last_telemetry = true_0 != 0;
        (*s).last_state = *state;
        (*s).has_last_state = true_0 != 0;
        return true_0 != 0;
    }
    if (*s).client.is_null() {
        set_error!(
            error,
            error_size,
            b"Native C-Nano session is closed\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !krpc_cnano_client_read((*s).client, cfg, t, state, error, error_size) {
        return false_0 != 0;
    }
    (*s).last_telemetry = *t;
    (*s).has_last_telemetry = true_0 != 0;
    (*s).last_state = *state;
    (*s).has_last_state = true_0 != 0;
    store_observation(s, t, state);
    return true_0 != 0;
}
unsafe extern "C" fn fill_direct_result(
    mut result: *mut KRPCApplyResult,
    mut command: *const GuidanceCommand,
    mut fc: *const FlightControlOutput,
    mut applied_yaw: ::core::ffi::c_double,
) {
    memset(
        result as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<KRPCApplyResult>() as size_t,
    );
    (*result).applied = true_0 != 0;
    (*result).autopilot_engaged = false_0 != 0;
    (*result).gear = (*fc).gear;
    (*result).brakes = (*fc).brakes;
    (*result).airbrakes = (*fc).airbrakes;
    (*result).target_pitch = (*command).target_pitch;
    (*result).target_heading = (*command).target_heading;
    (*result).target_roll = (*command).target_roll;
    (*result).throttle = (*fc).throttle;
    (*result).wheel_steering = (*fc).wheel_steering;
    (*result).has_actuator_feedback = true_0 != 0;
    (*result).control_pitch = (*fc).pitch;
    (*result).control_roll = (*fc).roll;
    (*result).control_yaw = applied_yaw;
    (*result).has_control_diagnostics = true_0 != 0;
    (*result).control_body_pitch_rate_available = (*fc)
        .diagnostics
        .body_pitch_rate_available;
    (*result).control_body_roll_rate_available = (*fc)
        .diagnostics
        .body_roll_rate_available;
    (*result).control_body_yaw_rate_available = (*fc)
        .diagnostics
        .body_yaw_rate_available;
    (*result).control_target_aoa = (*fc).diagnostics.target_pitch_state;
    (*result).control_measured_aoa = (*fc).diagnostics.measured_pitch_state;
    (*result).control_aoa_rate = (*fc).diagnostics.effective_pitch_rate;
    (*result).control_roll_rate = (*fc).diagnostics.effective_roll_rate;
    (*result).control_body_pitch_rate = (*fc).diagnostics.body_pitch_rate;
    (*result).control_body_roll_rate = (*fc).diagnostics.body_roll_rate;
    (*result).control_body_yaw_rate = (*fc).diagnostics.body_yaw_rate;
    (*result).control_pitch_error = (*fc).diagnostics.pitch_error;
    (*result).control_pitch_trim = (*fc).diagnostics.pitch_trim;
    (*result).control_pitch_authority = (*fc).diagnostics.pitch_authority;
    (*result).control_pitch_aero_fraction = (*fc).diagnostics.pitch_aero_fraction;
    (*result).control_roll_authority = (*fc).diagnostics.roll_authority;
    (*result).control_roll_raw_authority = (*fc).diagnostics.roll_raw_authority;
    (*result).control_roll_aero_fraction = (*fc).diagnostics.roll_aero_fraction;
    (*result).control_target_roll_rate = (*fc).diagnostics.target_roll_rate;
    (*result).control_beta_yaw_gain = (*fc).diagnostics.beta_yaw_gain;
    (*result).control_beta_roll_coupling = (*fc).diagnostics.beta_roll_coupling;
    (*result).control_beta_confidence = (*fc).diagnostics.beta_confidence;
    (*result).control_yaw_command = (*fc).yaw;
    snprintf(
        &raw mut (*result).control_revision as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        flight_control_revision(),
    );
    snprintf(
        &raw mut (*result).reference_frame as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"native-direct\0" as *const u8 as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*result).speed_mode as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        speed_mode_string((*command).navball_speed_mode),
    );
    snprintf(
        &raw mut (*result).control_profile as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        profile_string((*command).control_profile),
    );
}
unsafe extern "C" fn fill_autopilot_result(
    mut result: *mut KRPCApplyResult,
    mut command: *const GuidanceCommand,
) {
    memset(
        result as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<KRPCApplyResult>() as size_t,
    );
    (*result).applied = true_0 != 0;
    (*result).autopilot_engaged = (*command).autopilot_engaged;
    (*result).gear = (*command).gear;
    (*result).brakes = (*command).brakes;
    (*result).airbrakes = (*command).airbrakes;
    (*result).target_pitch = (*command).target_pitch;
    (*result).target_heading = (*command).target_heading;
    (*result).target_roll = (*command).target_roll;
    (*result).throttle = (*command).target_throttle;
    (*result).wheel_steering = (*command).wheel_steering;
    snprintf(
        &raw mut (*result).control_revision as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
        b"cnano-autopilot\0" as *const u8 as *const ::core::ffi::c_char,
    );
    snprintf(
        &raw mut (*result).reference_frame as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if (*command).use_inertial_direction as ::core::ffi::c_int != 0 {
            b"body-non-rotating\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"surface\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    snprintf(
        &raw mut (*result).speed_mode as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        speed_mode_string((*command).navball_speed_mode),
    );
    snprintf(
        &raw mut (*result).control_profile as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        profile_string((*command).control_profile),
    );
}
unsafe extern "C" fn quat_rotation_matrix(
    mut qv: *const ::core::ffi::c_double,
    mut r: *mut [::core::ffi::c_double; 3],
) {
    let mut x: ::core::ffi::c_double = *qv.offset(0 as ::core::ffi::c_int as isize);
    let mut y: ::core::ffi::c_double = *qv.offset(1 as ::core::ffi::c_int as isize);
    let mut z: ::core::ffi::c_double = *qv.offset(2 as ::core::ffi::c_int as isize);
    let mut w: ::core::ffi::c_double = *qv.offset(3 as ::core::ffi::c_int as isize);
    let mut n: ::core::ffi::c_double = sqrt(x * x + y * y + z * z + w * w);
    if n <= 1e-12f64 {
        z = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
        y = z;
        x = y;
        w = 1 as ::core::ffi::c_int as ::core::ffi::c_double;
    } else {
        x /= n;
        y /= n;
        z /= n;
        w /= n;
    }
    (*r.offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize] = 1
        as ::core::ffi::c_int as ::core::ffi::c_double
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * (y * y + z * z);
    (*r.offset(0 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * y - z * w);
    (*r.offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * z + y * w);
    (*r.offset(1 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * y + z * w);
    (*r.offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize] = 1
        as ::core::ffi::c_int as ::core::ffi::c_double
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * (x * x + z * z);
    (*r.offset(1 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (y * z - x * w);
    (*r.offset(2 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (x * z - y * w);
    (*r.offset(2 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize] = 2
        as ::core::ffi::c_int as ::core::ffi::c_double * (y * z + x * w);
    (*r.offset(2 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize] = 1
        as ::core::ffi::c_int as ::core::ffi::c_double
        - 2 as ::core::ffi::c_int as ::core::ffi::c_double * (x * x + y * y);
}
unsafe extern "C" fn matrix_quaternion(
    mut m: *const [::core::ffi::c_double; 3],
    mut qx: *mut ::core::ffi::c_double,
    mut qy: *mut ::core::ffi::c_double,
    mut qz: *mut ::core::ffi::c_double,
    mut qw: *mut ::core::ffi::c_double,
) {
    let mut x: ::core::ffi::c_double = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut y: ::core::ffi::c_double = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut z: ::core::ffi::c_double = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut w: ::core::ffi::c_double = 1 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut tr: ::core::ffi::c_double = (*m
        .offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
        + (*m.offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
        + (*m
            .offset(2 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize];
    let mut scale: ::core::ffi::c_double = 0.;
    if tr > 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        scale = sqrt(
            fmax(1e-12f64, tr + 1 as ::core::ffi::c_int as ::core::ffi::c_double),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = 0.25f64 * scale;
        x = ((*m
            .offset(2 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[2 as ::core::ffi::c_int as usize]) / scale;
        y = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        z = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    0 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
    } else if (*m
        .offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
        > (*m.offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
        && (*m
            .offset(0 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
            > (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[2 as ::core::ffi::c_int as usize]
    {
        scale = sqrt(
            fmax(
                1e-12f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + (*m
                        .offset(
                            0 as ::core::ffi::c_int as isize,
                        ))[0 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            1 as ::core::ffi::c_int as isize,
                        ))[1 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            2 as ::core::ffi::c_int as isize,
                        ))[2 as ::core::ffi::c_int as usize],
            ),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = ((*m
            .offset(2 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[2 as ::core::ffi::c_int as usize]) / scale;
        x = 0.25f64 * scale;
        y = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        z = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
    } else if (*m
        .offset(1 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
        > (*m.offset(2 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
    {
        scale = sqrt(
            fmax(
                1e-12f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + (*m
                        .offset(
                            1 as ::core::ffi::c_int as isize,
                        ))[1 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            0 as ::core::ffi::c_int as isize,
                        ))[0 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            2 as ::core::ffi::c_int as isize,
                        ))[2 as ::core::ffi::c_int as usize],
            ),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        x = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[1 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    1 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        y = 0.25f64 * scale;
        z = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
    } else {
        scale = sqrt(
            fmax(
                1e-12f64,
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    + (*m
                        .offset(
                            2 as ::core::ffi::c_int as isize,
                        ))[2 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            0 as ::core::ffi::c_int as isize,
                        ))[0 as ::core::ffi::c_int as usize]
                    - (*m
                        .offset(
                            1 as ::core::ffi::c_int as isize,
                        ))[1 as ::core::ffi::c_int as usize],
            ),
        ) * 2 as ::core::ffi::c_int as ::core::ffi::c_double;
        w = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[0 as ::core::ffi::c_int as usize]
            - (*m
                .offset(
                    0 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
        x = ((*m
            .offset(0 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[0 as ::core::ffi::c_int as usize]) / scale;
        y = ((*m
            .offset(1 as ::core::ffi::c_int as isize))[2 as ::core::ffi::c_int as usize]
            + (*m
                .offset(
                    2 as ::core::ffi::c_int as isize,
                ))[1 as ::core::ffi::c_int as usize]) / scale;
        z = 0.25f64 * scale;
    }
    let mut n: ::core::ffi::c_double = sqrt(x * x + y * y + z * z + w * w);
    if n > 1e-12f64 {
        x /= n;
        y /= n;
        z /= n;
        w /= n;
    }
    if w < 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
    *qx = x;
    *qy = y;
    *qz = z;
    *qw = w;
}
unsafe extern "C" fn inertial_entry_capture_step(
    mut s: *mut KRPCSession,
    mut command: *const GuidanceCommand,
    mut out: *mut FlightControlOutput,
) -> bool {
    if s.is_null() || command.is_null() || out.is_null() || !(*s).has_last_telemetry
        || !(*s).has_last_state || !(*s).last_telemetry.has_attitude_quaternion
    {
        return false_0 != 0;
    }
    let mut forward: Vector3 = vnorm(
        canonical_to_raw((*command).inertial_direction),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut radial: Vector3 = vnorm(
        canonical_to_raw((*s).last_state.position),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut up: Vector3 = vnorm(
        vproject_plane(radial, forward),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut right: Vector3 = vnorm(
        vcross(up, forward),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut down: Vector3 = vnorm(
        vcross(right, forward),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut current: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    let mut desired: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    let mut rel: [[::core::ffi::c_double; 3]; 3] = [[0.; 3]; 3];
    quat_rotation_matrix(
        &raw mut (*s).last_telemetry.attitude_quaternion as *mut ::core::ffi::c_double
            as *const ::core::ffi::c_double,
        &raw mut current as *mut [::core::ffi::c_double; 3],
    );
    desired[0 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize] = right
        .x;
    desired[1 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize] = right
        .y;
    desired[2 as ::core::ffi::c_int as usize][0 as ::core::ffi::c_int as usize] = right
        .z;
    desired[0 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize] = forward
        .x;
    desired[1 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize] = forward
        .y;
    desired[2 as ::core::ffi::c_int as usize][1 as ::core::ffi::c_int as usize] = forward
        .z;
    desired[0 as ::core::ffi::c_int as usize][2 as ::core::ffi::c_int as usize] = down.x;
    desired[1 as ::core::ffi::c_int as usize][2 as ::core::ffi::c_int as usize] = down.y;
    desired[2 as ::core::ffi::c_int as usize][2 as ::core::ffi::c_int as usize] = down.z;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while j < 3 as ::core::ffi::c_int {
            rel[i as usize][j as usize] = 0 as ::core::ffi::c_int
                as ::core::ffi::c_double;
            let mut k: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
            while k < 3 as ::core::ffi::c_int {
                rel[i as usize][j as usize]
                    += current[k as usize][i as usize] * desired[k as usize][j as usize];
                k += 1;
            }
            j += 1;
        }
        i += 1;
    }
    let mut x: ::core::ffi::c_double = 0.;
    let mut y: ::core::ffi::c_double = 0.;
    let mut z: ::core::ffi::c_double = 0.;
    let mut w: ::core::ffi::c_double = 0.;
    matrix_quaternion(
        &raw mut rel as *mut [::core::ffi::c_double; 3]
            as *const [::core::ffi::c_double; 3],
        &raw mut x,
        &raw mut y,
        &raw mut z,
        &raw mut w,
    );
    let mut vn: ::core::ffi::c_double = sqrt(x * x + y * y + z * z);
    let mut angle: ::core::ffi::c_double = if vn > 1e-12f64 {
        2 as ::core::ffi::c_int as ::core::ffi::c_double
            * atan2(vn, fmax(0 as ::core::ffi::c_int as ::core::ffi::c_double, w))
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut pitch_error: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut roll_error: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut yaw_error: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    if vn > 1e-12f64 {
        let mut sangle: ::core::ffi::c_double = angle * RAD2DEG / vn;
        pitch_error = -x * sangle;
        roll_error = -y * sangle;
        yaw_error = -z * sangle;
    }
    let mut t: *const Telemetry = &raw mut (*s).last_telemetry;
    let mut pitch_rate: ::core::ffi::c_double = if (*t).has_body_pitch_rate
        as ::core::ffi::c_int != 0
    {
        (*t).body_pitch_rate
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut roll_rate: ::core::ffi::c_double = if (*t).has_body_roll_rate
        as ::core::ffi::c_int != 0
    {
        (*t).body_roll_rate
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut yaw_rate: ::core::ffi::c_double = if (*t).has_body_yaw_rate
        as ::core::ffi::c_int != 0
    {
        (*t).body_yaw_rate
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    let mut wanted_pitch: ::core::ffi::c_double = clampd(
        pitch_error / 7.0f64,
        -5.0f64,
        5.0f64,
    );
    let mut wanted_roll: ::core::ffi::c_double = clampd(
        roll_error / 7.0f64,
        -4.0f64,
        4.0f64,
    );
    let mut wanted_yaw: ::core::ffi::c_double = clampd(
        yaw_error / 7.0f64,
        -4.0f64,
        4.0f64,
    );
    memset(
        out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<FlightControlOutput>() as size_t,
    );
    (*out).pitch = clampd((wanted_pitch - pitch_rate) * 0.18f64, -0.45f64, 0.45f64);
    (*out).roll = clampd((wanted_roll - roll_rate) * 0.16f64, -0.35f64, 0.35f64);
    (*out).yaw = clampd((wanted_yaw - yaw_rate) * 0.16f64, -0.35f64, 0.35f64);
    (*out).throttle = (*command).target_throttle;
    (*out).wheel_steering = (*command).wheel_steering;
    (*out).gear = (*command).gear;
    (*out).brakes = (*command).brakes;
    (*out).airbrakes = (*command).airbrakes;
    (*out).rcs_assist = 1 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*out).rcs_requested = true_0 != 0;
    (*out).valid = true_0 != 0;
    (*out).diagnostics.body_pitch_rate_available = (*t).has_body_pitch_rate;
    (*out).diagnostics.effective_pitch_rate = pitch_rate;
    (*out).diagnostics.pitch_error = pitch_error;
    (*out).diagnostics.effective_roll_rate = roll_rate;
    (*out).diagnostics.target_roll_rate = wanted_roll;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_orbital_up_reference(
    mut command: *const GuidanceCommand,
    mut state: *const VehicleState,
    mut out: *mut Vector3,
) -> bool {
    if command.is_null() || state.is_null() || out.is_null()
        || !(*command).use_inertial_direction || vmag((*state).position) <= 1.0f64
        || vmag((*command).inertial_direction) <= 1e-9f64
    {
        return false_0 != 0;
    }
    let mut forward: Vector3 = vnorm(
        (*command).inertial_direction,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut radial: Vector3 = vnorm(
        (*state).position,
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut projected: Vector3 = vproject_plane(radial, forward);
    if vmag(projected) <= 1e-8f64 {
        let mut seed: Vector3 = v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        let mut best: ::core::ffi::c_double = fabs(vdot(seed, forward));
        let mut y: Vector3 = v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        let mut z: Vector3 = v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        let mut dy: ::core::ffi::c_double = fabs(vdot(y, forward));
        let mut dz: ::core::ffi::c_double = fabs(vdot(z, forward));
        if dy < best {
            seed = y;
            best = dy;
        }
        if dz < best {
            seed = z;
        }
        projected = vproject_plane(seed, forward);
    }
    if vmag(projected) <= 1e-8f64 {
        return false_0 != 0;
    }
    *out = vnorm(
        projected,
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_apply(
    mut s: *mut KRPCSession,
    mut command: *const GuidanceCommand,
    mut airbrake_group: ::core::ffi::c_uint,
    mut phase: *const ::core::ffi::c_char,
    mut status: *const ::core::ffi::c_char,
    mut warning: *const ::core::ffi::c_char,
    mut hud_predicted_trajectory: *const Trajectory,
    mut hud_reference_trajectory: *const Trajectory,
    mut result: *mut KRPCApplyResult,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if s.is_null() || command.is_null() {
        set_error!(
            error,
            error_size,
            b"Flight-control apply has invalid state\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if (*s).simulator {
        if !getenv(b"KSP_LANDER_APPLY_DIAG\0" as *const u8 as *const ::core::ffi::c_char)
            .is_null()
        {
            fprintf(
                stderr,
                b"KSP_APPLY phase=%s roll=%.3f aoa=%.3f heading=%.3f status=%s warning=%s\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if !phase.is_null() {
                    phase
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
                (*command).target_roll,
                if (*command).has_target_aoa as ::core::ffi::c_int != 0 {
                    (*command).target_aoa
                } else {
                    (*command).target_pitch
                },
                (*command).target_heading,
                if !status.is_null() {
                    status
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
                if !warning.is_null() {
                    warning
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
        }
        let mut direct: bool = sim_direct_control() as ::core::ffi::c_int != 0
            && (*command).control_profile as ::core::ffi::c_uint
                != PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint
            && !((*command).control_profile as ::core::ffi::c_uint
                == PROFILE_ENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
                && (*command).use_inertial_direction as ::core::ffi::c_int != 0)
            && (*s).has_last_telemetry as ::core::ffi::c_int != 0;
        if direct {
            let mut dt: ::core::ffi::c_double = 1.0f64
                / fmax(
                    2.0f64,
                    (if (*s).flight_control.nominal_dt
                        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                    {
                        1.0f64 / (*s).flight_control.nominal_dt
                    } else {
                        10.0f64
                    }),
                );
            if (*s).has_last_control_ut as ::core::ffi::c_int != 0
                && (*s).last_telemetry.ut > (*s).last_control_ut
            {
                dt = clampd(
                    (*s).last_telemetry.ut - (*s).last_control_ut,
                    0.01f64,
                    2.0f64,
                );
            }
            let mut fc: FlightControlOutput = FlightControlOutput {
                valid: false,
                pitch: 0.,
                roll: 0.,
                yaw: 0.,
                throttle: 0.,
                wheel_steering: 0.,
                gear: false,
                brakes: false,
                airbrakes: false,
                rcs_assist: 0.,
                rcs_requested: false,
                diagnostics: FlightControlDiagnostics {
                    profile: PROFILE_ORBITAL,
                    target_pitch_state: 0.,
                    measured_pitch_state: 0.,
                    pitch_error: 0.,
                    effective_pitch_rate: 0.,
                    body_pitch_rate: 0.,
                    body_pitch_rate_available: false,
                    roll_error: 0.,
                    effective_roll_rate: 0.,
                    body_roll_rate: 0.,
                    body_roll_rate_available: false,
                    target_roll_rate: 0.,
                    commanded_roll_rate: 0.,
                    relative_roll_rate: 0.,
                    recovery_rate_first: false,
                    heading_error: 0.,
                    sideslip: 0.,
                    sideslip_rate: 0.,
                    body_yaw_rate: 0.,
                    body_yaw_rate_available: false,
                    flight_path_angle: 0.,
                    pitch_trim: 0.,
                    steady_pitch_trim: 0.,
                    terminal_pitch_integral: 0.,
                    roll_trim: 0.,
                    pitch_authority: 0.,
                    pitch_raw_authority: 0.,
                    pitch_guard_authority: 0.,
                    pitch_aero_fraction: 0.,
                    pitch_fast_command_limit: 0.,
                    pitch_hold_seconds: 0.,
                    pitch_rate_impulse_budget: 0.,
                    roll_authority: 0.,
                    roll_raw_authority: 0.,
                    roll_guard_authority: 0.,
                    roll_aero_fraction: 0.,
                    roll_rate_limit: 0.,
                    roll_command_limit: 0.,
                    roll_hold_seconds: 0.,
                    roll_rate_impulse_budget: 0.,
                    roll_bandwidth_scale: 0.,
                    pitch_bandwidth_scale: 0.,
                    yaw_authority: 0.,
                    yaw_aero_fraction: 0.,
                    beta_yaw_gain: 0.,
                    beta_roll_coupling: 0.,
                    beta_confidence: 0.,
                    sample_dt: 0.,
                    attitude_error: 0.,
                    quaternion_attitude_control: false,
                    rcs_transonic_cutoff: false,
                },
            };
            if !flight_control_step(
                &raw mut (*s).flight_control,
                &raw mut (*s).last_telemetry,
                command,
                dt,
                &raw mut fc,
            ) || !fc.valid
            {
                set_error!(
                    error,
                    error_size,
                    b"Native atmospheric flight-control law rejected profile %s\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    profile_string((*command).control_profile),
                );
                return false_0 != 0;
            }
            if !sim_send_guidance_inputs(
                s,
                command,
                &raw mut fc,
                true_0 != 0,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
            (*s).last_control_ut = (*s).last_telemetry.ut;
            (*s).has_last_control_ut = true_0 != 0;
            if !result.is_null() {
                fill_direct_result(result, command, &raw mut fc, fc.yaw);
                snprintf(
                    &raw mut (*result).reference_frame as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
                    b"shuttlesim-direct\0" as *const u8 as *const ::core::ffi::c_char,
                );
            }
            snprintf(
                &raw mut (*s).last_control_profile as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                profile_string((*command).control_profile),
            );
            return true_0 != 0;
        }
        if !sim_send_guidance(s, command, true_0 != 0, error, error_size) {
            return false_0 != 0;
        }
        if !result.is_null() {
            memset(
                result as *mut ::core::ffi::c_void,
                0 as ::core::ffi::c_int,
                ::core::mem::size_of::<KRPCApplyResult>() as size_t,
            );
            (*result).applied = true_0 != 0;
            (*result).autopilot_engaged = true_0 != 0;
            (*result).gear = (*command).gear;
            (*result).brakes = (*command).brakes;
            (*result).airbrakes = false_0 != 0;
            (*result).target_pitch = (*command).target_pitch;
            (*result).target_heading = (*command).target_heading;
            (*result).target_roll = (*command).target_roll;
            (*result).throttle = if (*command).control_profile as ::core::ffi::c_uint
                == PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                clampd((*command).target_throttle, 0.0f64, 1.0f64)
            } else {
                0.0f64
            };
            snprintf(
                &raw mut (*result).reference_frame as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
                b"shuttlesim-high-level\0" as *const u8 as *const ::core::ffi::c_char,
            );
            snprintf(
                &raw mut (*result).speed_mode as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                speed_mode_string((*command).navball_speed_mode),
            );
            snprintf(
                &raw mut (*result).control_profile as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                profile_string((*command).control_profile),
            );
        }
        snprintf(
            &raw mut (*s).last_control_profile as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            profile_string((*command).control_profile),
        );
        return true_0 != 0;
    }
    if (*s).client.is_null() {
        set_error!(
            error,
            error_size,
            b"Native C-Nano apply has invalid state\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if (*command).control_profile as ::core::ffi::c_uint
        == PROFILE_TAEM as ::core::ffi::c_int as ::core::ffi::c_uint
        && !(*s).taem_surface_roles_latched
    {
        (*s).taem_surface_roles_latched = true_0 != 0;
        let mut layout: [::core::ffi::c_char; 48] = [0; 48];
        fprintf(
            stderr,
            b"Surface allocation: TAEM ownership -> separated elevator/aileron/rudder roles, body flap off (%s)\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            if *krpc_cnano_client_surface_layout(
                    (*s).client,
                    &raw mut layout as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
                )
                .offset(0 as ::core::ffi::c_int as isize) as ::core::ffi::c_int != 0
            {
                &raw mut layout as *mut ::core::ffi::c_char as *const ::core::ffi::c_char
            } else {
                b"unclassified; stock mixing kept\0" as *const u8
                    as *const ::core::ffi::c_char
            },
        );
    }
    if !krpc_cnano_client_set_surface_allocation(
        (*s).client,
        (if !(*s).taem_surface_roles_latched {
            KRPC_SURFACE_ALLOCATION_MIXED as ::core::ffi::c_int
        } else if (*command).terminal_roll_trim_enabled as ::core::ffi::c_int != 0
            && ((*command).control_profile as ::core::ffi::c_uint
                == PROFILE_APPROACH as ::core::ffi::c_int as ::core::ffi::c_uint
                || (*command).control_profile as ::core::ffi::c_uint
                    == PROFILE_FLARE as ::core::ffi::c_int as ::core::ffi::c_uint)
        {
            KRPC_SURFACE_ALLOCATION_SEPARATED_FLAP_PITCH as ::core::ffi::c_int
        } else {
            KRPC_SURFACE_ALLOCATION_SEPARATED as ::core::ffi::c_int
        }) as KrpcSurfaceAllocation,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    let mut atmospheric: bool = (*command).control_profile as ::core::ffi::c_uint
        != PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut inertial_entry: bool = atmospheric as ::core::ffi::c_int != 0
        && (*command).control_profile as ::core::ffi::c_uint
            == PROFILE_ENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
        && (*command).use_inertial_direction as ::core::ffi::c_int != 0;
    if atmospheric {
        if !(*s).has_last_telemetry {
            set_error!(
                error,
                error_size,
                b"Native flight control has no telemetry sample\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
        let mut dt_0: ::core::ffi::c_double = 1.0f64
            / fmax(
                2.0f64,
                (if (*s).flight_control.nominal_dt
                    > 0 as ::core::ffi::c_int as ::core::ffi::c_double
                {
                    1.0f64 / (*s).flight_control.nominal_dt
                } else {
                    10.0f64
                }),
            );
        if (*s).has_last_control_ut as ::core::ffi::c_int != 0
            && (*s).last_telemetry.ut > (*s).last_control_ut
        {
            dt_0 = clampd(
                (*s).last_telemetry.ut - (*s).last_control_ut,
                0.01f64,
                2.0f64,
            );
        }
        let mut fc_0: FlightControlOutput = FlightControlOutput {
            valid: false,
            pitch: 0.,
            roll: 0.,
            yaw: 0.,
            throttle: 0.,
            wheel_steering: 0.,
            gear: false,
            brakes: false,
            airbrakes: false,
            rcs_assist: 0.,
            rcs_requested: false,
            diagnostics: FlightControlDiagnostics {
                profile: PROFILE_ORBITAL,
                target_pitch_state: 0.,
                measured_pitch_state: 0.,
                pitch_error: 0.,
                effective_pitch_rate: 0.,
                body_pitch_rate: 0.,
                body_pitch_rate_available: false,
                roll_error: 0.,
                effective_roll_rate: 0.,
                body_roll_rate: 0.,
                body_roll_rate_available: false,
                target_roll_rate: 0.,
                commanded_roll_rate: 0.,
                relative_roll_rate: 0.,
                recovery_rate_first: false,
                heading_error: 0.,
                sideslip: 0.,
                sideslip_rate: 0.,
                body_yaw_rate: 0.,
                body_yaw_rate_available: false,
                flight_path_angle: 0.,
                pitch_trim: 0.,
                steady_pitch_trim: 0.,
                terminal_pitch_integral: 0.,
                roll_trim: 0.,
                pitch_authority: 0.,
                pitch_raw_authority: 0.,
                pitch_guard_authority: 0.,
                pitch_aero_fraction: 0.,
                pitch_fast_command_limit: 0.,
                pitch_hold_seconds: 0.,
                pitch_rate_impulse_budget: 0.,
                roll_authority: 0.,
                roll_raw_authority: 0.,
                roll_guard_authority: 0.,
                roll_aero_fraction: 0.,
                roll_rate_limit: 0.,
                roll_command_limit: 0.,
                roll_hold_seconds: 0.,
                roll_rate_impulse_budget: 0.,
                roll_bandwidth_scale: 0.,
                pitch_bandwidth_scale: 0.,
                yaw_authority: 0.,
                yaw_aero_fraction: 0.,
                beta_yaw_gain: 0.,
                beta_roll_coupling: 0.,
                beta_confidence: 0.,
                sample_dt: 0.,
                attitude_error: 0.,
                quaternion_attitude_control: false,
                rcs_transonic_cutoff: false,
            },
        };
        let mut valid: bool = false_0 != 0;
        if inertial_entry {
            valid = inertial_entry_capture_step(s, command, &raw mut fc_0);
        } else {
            let mut attitude_error: FlightControlAttitudeError = FlightControlAttitudeError {
                valid: 0 as ::core::ffi::c_int != 0,
                use_yaw_error: false,
                pitch_error_deg: 0.,
                roll_error_deg: 0.,
                yaw_error_deg: 0.,
            };
            let mut attitude_ptr: *const FlightControlAttitudeError = ::core::ptr::null::<
                FlightControlAttitudeError,
            >();
            let mut planet: *const PlanetModel = krpc_session_planet(s);
            let mut quaternion_eligible: bool = (*command).control_profile
                as ::core::ffi::c_uint
                != PROFILE_ROLLOUT as ::core::ffi::c_int as ::core::ffi::c_uint
                && (*s).has_last_state as ::core::ffi::c_int != 0
                && (*s).last_telemetry.has_attitude_quaternion as ::core::ffi::c_int != 0
                && !planet.is_null();
            if quaternion_eligible {
                let mut surface_velocity: Vector3 = vsub(
                    (*s).last_state.velocity,
                    vcross(planet_rotation_vector(planet), (*s).last_state.position),
                );
                let mut target_aoa: ::core::ffi::c_double = if (*command).has_target_aoa
                    as ::core::ffi::c_int != 0
                    && (*command).target_aoa.is_finite() as i32 != 0
                {
                    (*command).target_aoa
                } else {
                    (*command).target_pitch - (*s).last_telemetry.flight_path_angle
                };
                if flight_control_airframe_error(
                    &raw mut (*s).last_telemetry.attitude_quaternion
                        as *mut ::core::ffi::c_double as *const ::core::ffi::c_double,
                    canonical_to_raw(surface_velocity),
                    canonical_to_raw((*s).last_state.position),
                    target_aoa,
                    (*command).target_roll,
                    !(*command).heading_control_enabled,
                    &raw mut attitude_error,
                ) {
                    attitude_ptr = &raw mut attitude_error;
                }
            }
            valid = flight_control_step_attitude(
                &raw mut (*s).flight_control,
                &raw mut (*s).last_telemetry,
                command,
                attitude_ptr,
                dt_0,
                &raw mut fc_0,
            );
        }
        if !valid || !fc_0.valid {
            set_error!(
                error,
                error_size,
                b"Native %s flight-control law rejected profile %s\0" as *const u8
                    as *const ::core::ffi::c_char,
                if inertial_entry as ::core::ffi::c_int != 0 {
                    b"inertial-entry\0" as *const u8 as *const ::core::ffi::c_char
                } else {
                    b"atmospheric\0" as *const u8 as *const ::core::ffi::c_char
                },
                profile_string((*command).control_profile),
            );
            return false_0 != 0;
        }
        let mut unpowered: *const ::core::ffi::c_char = getenv(
            b"KSP_LANDER_UNPOWERED_ONLY\0" as *const u8 as *const ::core::ffi::c_char,
        );
        let mut unpowered_only: bool = !unpowered.is_null()
            && strcmp(unpowered, b"1\0" as *const u8 as *const ::core::ffi::c_char)
                == 0 as ::core::ffi::c_int;
        if unpowered_only {
            fc_0.throttle = 0.0f64;
        }
        let mut speedbrake_fraction: ::core::ffi::c_double = (*command)
            .speedbrake_fraction;
        let mut native_yaw: ::core::ffi::c_double = fc_0.yaw;
        if !krpc_cnano_client_set_direct_controls(
            (*s).client,
            fc_0.pitch,
            fc_0.roll,
            native_yaw,
            fc_0.throttle,
            fc_0.wheel_steering,
            fc_0.rcs_requested,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if !krpc_cnano_client_set_speedbrake(
            (*s).client,
            speedbrake_fraction,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if !result.is_null() {
            fill_direct_result(result, command, &raw mut fc_0, native_yaw);
            if fc_0.diagnostics.quaternion_attitude_control {
                snprintf(
                    &raw mut (*result).reference_frame as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
                    b"air-relative-quaternion\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
            }
        }
        (*s).last_control_ut = (*s).last_telemetry.ut;
        (*s).has_last_control_ut = true_0 != 0;
    } else {
        let mut radial_up: Vector3 = Vector3 { x: 0., y: 0., z: 0. };
        let mut up_reference: *const Vector3 = ::core::ptr::null::<Vector3>();
        if (*command).use_inertial_direction as ::core::ffi::c_int != 0
            && (*s).has_last_state as ::core::ffi::c_int != 0
            && krpc_orbital_up_reference(
                command,
                &raw mut (*s).last_state,
                &raw mut radial_up,
            ) as ::core::ffi::c_int != 0
        {
            up_reference = &raw mut radial_up;
        }
        if !krpc_cnano_client_set_autopilot(
            (*s).client,
            command,
            up_reference,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if !result.is_null() {
            fill_autopilot_result(result, command);
        }
    }
    if !krpc_cnano_client_set_gear((*s).client, (*command).gear, error, error_size)
        || !krpc_cnano_client_set_brakes(
            (*s).client,
            (*command).brakes,
            error,
            error_size,
        )
        || !krpc_cnano_client_set_airbrakes(
            (*s).client,
            airbrake_group,
            (*command).airbrakes,
            error,
            error_size,
        )
        || !krpc_cnano_client_set_speed_mode(
            (*s).client,
            (*command).navball_speed_mode,
            error,
            error_size,
        )
    {
        return false_0 != 0;
    }
    if (*s).rollout_parachutes_enabled as ::core::ffi::c_int != 0
        && !(*s).rollout_parachutes_deployed
        && (*s).has_last_telemetry as ::core::ffi::c_int != 0
        && flight_control_rollout_parachutes_ready(&raw mut (*s).last_telemetry, command)
            as ::core::ffi::c_int != 0
    {
        if !krpc_cnano_client_deploy_parachutes((*s).client, error, error_size) {
            return false_0 != 0;
        }
        (*s).rollout_parachutes_deployed = true_0 != 0;
        fprintf(
            stderr,
            b"Rollout parachutes deployment requested at UT %.3f, speed %.2f m/s.\n\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*s).last_telemetry.ut,
            (*s).last_telemetry.surface_speed,
        );
    }
    snprintf(
        &raw mut (*s).last_control_profile as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 24]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        profile_string((*command).control_profile),
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_neutralize(mut s: *mut KRPCSession) {
    if s.is_null() {
        return;
    }
    if (*s).simulator {
        let mut ignored: [::core::ffi::c_char; 128] = [0; 128];
        sim_send_guidance(
            s,
            ::core::ptr::null::<GuidanceCommand>(),
            false_0 != 0,
            &raw mut ignored as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        );
    } else if !(*s).client.is_null() {
        krpc_cnano_client_neutralize((*s).client);
    }
    flight_control_reset_transients(&raw mut (*s).flight_control);
    (*s).has_last_control_ut = false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_safe(mut s: *mut KRPCSession) {
    if s.is_null() {
        return;
    }
    if (*s).simulator {
        let mut ignored: [::core::ffi::c_char; 128] = [0; 128];
        sim_send_guidance(
            s,
            ::core::ptr::null::<GuidanceCommand>(),
            false_0 != 0,
            &raw mut ignored as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        );
    } else if !(*s).client.is_null() {
        krpc_cnano_client_safe((*s).client);
    }
    flight_control_reset_transients(&raw mut (*s).flight_control);
    (*s).has_last_control_ut = false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_set_gear(
    mut s: *mut KRPCSession,
    mut value: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if s.is_null() {
        return false_0 != 0;
    }
    if (*s).simulator {
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
        c.gear = value;
        c.brakes = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
            (*s).last_telemetry.brakes as ::core::ffi::c_int
        } else {
            false_0
        } != 0;
        c.has_target_aoa = true_0 != 0;
        c.target_aoa = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
            (*s).last_telemetry.angle_of_attack
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        c.target_roll = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
            (*s).last_telemetry.roll
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        return sim_send_guidance(s, &raw mut c, false_0 != 0, error, error_size);
    }
    return !(*s).client.is_null()
        && krpc_cnano_client_set_gear((*s).client, value, error, error_size)
            as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_set_brakes(
    mut s: *mut KRPCSession,
    mut value: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if s.is_null() {
        return false_0 != 0;
    }
    if (*s).simulator {
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
        c.gear = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
            (*s).last_telemetry.gear as ::core::ffi::c_int
        } else {
            false_0
        } != 0;
        c.brakes = value;
        c.has_target_aoa = true_0 != 0;
        c.target_aoa = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
            (*s).last_telemetry.angle_of_attack
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        c.target_roll = if (*s).has_last_telemetry as ::core::ffi::c_int != 0 {
            (*s).last_telemetry.roll
        } else {
            0 as ::core::ffi::c_int as ::core::ffi::c_double
        };
        return sim_send_guidance(s, &raw mut c, false_0 != 0, error, error_size);
    }
    return !(*s).client.is_null()
        && krpc_cnano_client_set_brakes((*s).client, value, error, error_size)
            as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_save_game(
    mut s: *mut KRPCSession,
    mut name: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !s.is_null() && (*s).simulator as ::core::ffi::c_int != 0 {
        return true_0 != 0;
    }
    return !s.is_null() && !(*s).client.is_null()
        && krpc_cnano_client_save((*s).client, name, error, error_size)
            as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_shutdown_burn_engines(
    mut s: *mut KRPCSession,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !s.is_null() && (*s).simulator as ::core::ffi::c_int != 0 {
        return true_0 != 0;
    }
    return !s.is_null() && !(*s).client.is_null()
        && krpc_cnano_client_shutdown_burn_engines((*s).client, error, error_size)
            as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_read_physics_warp(
    mut s: *mut KRPCSession,
    mut factor: *mut ::core::ffi::c_int,
    mut rate: *mut ::core::ffi::c_double,
    mut rails: *mut bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if factor.is_null() || rate.is_null() || rails.is_null() {
        return false_0 != 0;
    }
    if !s.is_null() && (*s).simulator as ::core::ffi::c_int != 0 {
        *factor = 0 as ::core::ffi::c_int;
        *rate = 1.0f64;
        *rails = false_0 != 0;
        return true_0 != 0;
    }
    return !s.is_null() && !(*s).client.is_null()
        && krpc_cnano_client_physics_warp(
            (*s).client,
            factor,
            rate,
            rails,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_set_physics_warp(
    mut s: *mut KRPCSession,
    mut factor: ::core::ffi::c_int,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !s.is_null() && (*s).simulator as ::core::ffi::c_int != 0 {
        return true_0 != 0;
    }
    return !s.is_null() && !(*s).client.is_null()
        && krpc_cnano_client_set_physics_warp((*s).client, factor, error, error_size)
            as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_warp(
    mut s: *mut KRPCSession,
    mut ut: ::core::ffi::c_double,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    return krpc_warp_with_progress(s, ut, None, NULL, error, error_size);
}
#[no_mangle]
pub unsafe extern "C" fn krpc_warp_with_progress(
    mut s: *mut KRPCSession,
    mut ut: ::core::ffi::c_double,
    mut progress: KRPCWarpProgress,
    mut context: *mut ::core::ffi::c_void,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !s.is_null() && (*s).simulator as ::core::ffi::c_int != 0 {
        set_error!(
            error,
            error_size,
            b"Time warp is not used in ShuttleSim lockstep\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut ok: bool = !s.is_null() && !(*s).client.is_null()
        && krpc_cnano_client_warp_with_progress(
            (*s).client,
            ut,
            progress,
            context,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0;
    if ok {
        flight_control_reset_transients(&raw mut (*s).flight_control);
        (*s).has_last_control_ut = false_0 != 0;
    }
    return ok;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_session_close(mut s: *mut KRPCSession) {
    if s.is_null() {
        return;
    }
    if !(*s).physics_store.is_null() {
        if !(*s).simulator {
            let mut ignored: [::core::ffi::c_char; 256] = [0; 256];
            physics_store_flush(
                (*s).physics_store,
                &raw mut ignored as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
            );
        }
        physics_store_close((*s).physics_store);
    }
    if (*s).sim_rx_fd >= 0 as ::core::ffi::c_int {
        close((*s).sim_rx_fd);
    }
    if (*s).sim_tx_fd >= 0 as ::core::ffi::c_int {
        close((*s).sim_tx_fd);
    }
    if !(*s).client.is_null() {
        krpc_cnano_client_close((*s).client);
    }
    free(s as *mut ::core::ffi::c_void);
}
pub const VESSEL_AERO_SAMPLES: ::core::ffi::c_int = 192 as ::core::ffi::c_int;
#[inline]
unsafe extern "C" fn __bswap_16(mut __bsx: __uint16_t) -> __uint16_t {
    return (__bsx as ::core::ffi::c_int >> 8 as ::core::ffi::c_int
        & 0xff as ::core::ffi::c_int
        | (__bsx as ::core::ffi::c_int & 0xff as ::core::ffi::c_int)
            << 8 as ::core::ffi::c_int) as __uint16_t;
}
#[inline]
unsafe extern "C" fn __bswap_32(mut __bsx: __uint32_t) -> __uint32_t {
    return (__bsx & 0xff000000 as __uint32_t) >> 24 as ::core::ffi::c_int
        | (__bsx & 0xff0000 as __uint32_t) >> 8 as ::core::ffi::c_int
        | (__bsx & 0xff00 as __uint32_t) << 8 as ::core::ffi::c_int
        | (__bsx & 0xff as __uint32_t) << 24 as ::core::ffi::c_int;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
#[repr(C)]
pub struct PhysicsStore {
    _private: [u8; 0],
}
#[repr(C)]
pub struct KrpcCNanoClient {
    _private: [u8; 0],
}
