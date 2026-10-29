use crate::sys::*;
extern "C" {
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn nextafter(
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
    fn fabsf(__x: ::core::ffi::c_float) -> ::core::ffi::c_float;
    fn vessel_physics_conservative_stall_fraction(
        true_air_speed: ::core::ffi::c_double,
        angle_of_attack: ::core::ffi::c_double,
        dynamic_pressure: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn norm_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn norm_signed_deg(a: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn great_circle_distance(
        a: GeoPoint,
        b: GeoPoint,
        radius: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn initial_bearing(a: GeoPoint, b: GeoPoint) -> ::core::ffi::c_double;
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
    fn lowpass_update(
        f: *mut LowPass,
        value: ::core::ffi::c_double,
        dt: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn lowpass_reset(f: *mut LowPass);
    fn telemetry_init(t: *mut Telemetry);
    fn planet_rotation_vector(p: *const PlanetModel) -> Vector3;
    fn planet_atmospheric_density(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn planet_atmospheric_speed_of_sound(
        p: *const PlanetModel,
        altitude: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn predictor_geo_point(
        position: Vector3,
        planet: *const PlanetModel,
        ut: ::core::ffi::c_double,
    ) -> GeoPoint;
    fn krpc_cnano_transport_config(
        storage: *mut KrpcCNanoTransport,
        ops: *const KrpcCNanoTransportOps,
        context: *mut ::core::ffi::c_void,
        timeout_ms: ::core::ffi::c_int,
    ) -> KrpcCNanoTransportConfig;
    fn krpc_cnano_transport_last_status(
        connection: krpc_connection_t,
    ) -> KrpcCNanoTransportStatus;
    fn krpc_cnano_transport_status_string(
        status: KrpcCNanoTransportStatus,
    ) -> *const ::core::ffi::c_char;
    fn krpc_cnano_transport_is_open(connection: krpc_connection_t) -> bool;
    fn krpc_cnano_transport_bytes_read(connection: krpc_connection_t) -> size_t;
    fn krpc_cnano_transport_bytes_written(connection: krpc_connection_t) -> size_t;
    fn krpc_cnano_transport_timeout_ms(
        connection: krpc_connection_t,
    ) -> ::core::ffi::c_int;
    fn krpc_cnano_transport_begin_deadline(connection: krpc_connection_t) -> bool;
    fn krpc_cnano_transport_end_deadline(connection: krpc_connection_t);
    fn krpc_cnano_posix_serial_init(
        serial: *mut KrpcCNanoPosixSerial,
        path: *const ::core::ffi::c_char,
        timeout_ms: ::core::ffi::c_int,
        baud_rate: ::core::ffi::c_int,
        configure_termios: bool,
    );
    fn krpc_cnano_posix_serial_ops() -> *const KrpcCNanoTransportOps;
    fn krpc_cnano_posix_tcp_init(
        tcp: *mut KrpcCNanoPosixTcp,
        host: *const ::core::ffi::c_char,
        port: ::core::ffi::c_int,
        timeout_ms: ::core::ffi::c_int,
    );
    fn krpc_cnano_posix_tcp_ops() -> *const KrpcCNanoTransportOps;
    fn krpc_get_error(error: krpc_error_t) -> *const ::core::ffi::c_char;
    fn krpc_get_error_message() -> *const ::core::ffi::c_char;
    fn krpc_open(
        connection: *mut krpc_connection_t,
        config: *const krpc_connection_config_t,
    ) -> krpc_error_t;
    fn krpc_close(connection: krpc_connection_t) -> krpc_error_t;
    fn pb_encode_tag_for_field(
        stream: *mut pb_ostream_t,
        field: *const pb_field_iter_t,
    ) -> bool;
    fn pb_encode_varint(stream: *mut pb_ostream_t, value: uint64_t) -> bool;
    fn krpc_init_result(result: *mut krpc_result_t) -> krpc_error_t;
    fn krpc_free_result(result: *mut krpc_result_t) -> krpc_error_t;
    fn krpc_get_return_value(
        result: *mut krpc_result_t,
        stream: *mut pb_istream_t,
    ) -> krpc_error_t;
    fn krpc_decode_double(
        stream: *mut pb_istream_t,
        value: *mut ::core::ffi::c_double,
    ) -> krpc_error_t;
    fn krpc_decode_float(
        stream: *mut pb_istream_t,
        value: *mut ::core::ffi::c_float,
    ) -> krpc_error_t;
    fn krpc_decode_int32(stream: *mut pb_istream_t, value: *mut int32_t) -> krpc_error_t;
    fn krpc_decode_bool(stream: *mut pb_istream_t, value: *mut bool) -> krpc_error_t;
    fn krpc_decode_string(
        stream: *mut pb_istream_t,
        value: *mut *mut ::core::ffi::c_char,
    ) -> krpc_error_t;
    fn krpc_decode_object(
        stream: *mut pb_istream_t,
        value: *mut krpc_object_t,
    ) -> krpc_error_t;
    fn krpc_decode_enum(
        stream: *mut pb_istream_t,
        value: *mut krpc_enum_t,
    ) -> krpc_error_t;
    fn krpc_decode_message_Tuple(
        stream: *mut pb_istream_t,
        value: *mut krpc_schema_Tuple,
    ) -> krpc_error_t;
    fn krpc_decode_message_List(
        stream: *mut pb_istream_t,
        value: *mut krpc_schema_List,
    ) -> krpc_error_t;
    fn krpc_calloc(num: size_t, size: size_t) -> *mut ::core::ffi::c_void;
    fn krpc_recalloc(
        ptr: *mut ::core::ffi::c_void,
        num: size_t,
        inc: size_t,
        size: size_t,
    ) -> *mut ::core::ffi::c_void;
    fn krpc_free(ptr: *mut ::core::ffi::c_void);
    fn krpc_call(
        call: *mut krpc_call_t,
        serviceId: uint32_t,
        procedureId: uint32_t,
        numArguments: size_t,
        arguments: *mut krpc_argument_t,
    ) -> krpc_error_t;
    fn krpc_add_argument(
        call: *mut krpc_call_t,
        position: uint32_t,
        encode: Option<
            unsafe extern "C" fn(
                *mut pb_ostream_t,
                *const pb_field_t,
                *const *mut ::core::ffi::c_void,
            ) -> bool,
        >,
        arg: *const ::core::ffi::c_void,
    ) -> krpc_error_t;
    fn krpc_encode_double(
        stream: *mut pb_ostream_t,
        value: ::core::ffi::c_double,
    ) -> krpc_error_t;
    fn krpc_encode_size_double(
        size: *mut size_t,
        value: ::core::ffi::c_double,
    ) -> krpc_error_t;
    fn krpc_encode_callback_double(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_float(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_int32(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_uint32(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_uint64(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_bool(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_string(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_object(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_callback_enum(
        stream: *mut pb_ostream_t,
        field: *const pb_field_t,
        arg: *const *mut ::core::ffi::c_void,
    ) -> bool;
    fn krpc_encode_message_Tuple(
        stream: *mut pb_ostream_t,
        value: *const krpc_schema_Tuple,
    ) -> krpc_error_t;
    fn krpc_cnano_invoke_batch(
        connection: krpc_connection_t,
        calls: *const krpc_schema_ProcedureCall,
        results: *mut krpc_result_t,
        result_failed: *mut bool,
        count: size_t,
    ) -> krpc_error_t;
    fn krpc_connect(
        connection: krpc_connection_t,
        clientName: *const ::core::ffi::c_char,
    ) -> krpc_error_t;
    fn krpc_invoke(
        connection: krpc_connection_t,
        result: *mut krpc_schema_ProcedureResult,
        call: *mut krpc_schema_ProcedureCall,
    ) -> krpc_error_t;
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
pub type __uint8_t = u8;
pub type __uint16_t = u16;
pub type __int32_t = i32;
pub type __uint32_t = u32;
pub type __int64_t = i64;
pub type __uint64_t = u64;
pub type __uint_least16_t = __uint16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type uint_least16_t = __uint_least16_t;
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
pub struct VehicleState {
    pub ut: ::core::ffi::c_double,
    pub position: Vector3,
    pub velocity: Vector3,
    pub mass: ::core::ffi::c_double,
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
pub struct LowPass {
    pub tc: ::core::ffi::c_double,
    pub value: ::core::ffi::c_double,
    pub has_value: bool,
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
pub type KrpcCNanoTransportStatus = ::core::ffi::c_uint;
pub const KRPC_CNANO_TRANSPORT_ZERO_PROGRESS: KrpcCNanoTransportStatus = 6;
pub const KRPC_CNANO_TRANSPORT_INVALID: KrpcCNanoTransportStatus = 5;
pub const KRPC_CNANO_TRANSPORT_CLOSED: KrpcCNanoTransportStatus = 4;
pub const KRPC_CNANO_TRANSPORT_IO_ERROR: KrpcCNanoTransportStatus = 3;
pub const KRPC_CNANO_TRANSPORT_TIMEOUT: KrpcCNanoTransportStatus = 2;
pub const KRPC_CNANO_TRANSPORT_EOF: KrpcCNanoTransportStatus = 1;
pub const KRPC_CNANO_TRANSPORT_OK: KrpcCNanoTransportStatus = 0;
pub type KrpcCNanoTransportProtocol = ::core::ffi::c_uint;
pub const KRPC_CNANO_PROTOCOL_TCP_RPC: KrpcCNanoTransportProtocol = 1;
pub const KRPC_CNANO_PROTOCOL_MULTIPLEXED: KrpcCNanoTransportProtocol = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransport {
    pub ops: *const KrpcCNanoTransportOps,
    pub context: *mut ::core::ffi::c_void,
    pub last_status: KrpcCNanoTransportStatus,
    pub bytes_read: size_t,
    pub bytes_written: size_t,
    pub timeout_ms: ::core::ffi::c_int,
    pub deadline_ns: int64_t,
    pub deadline_active: bool,
    pub is_open: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransportOps {
    pub open: KrpcCNanoTransportOpenFn,
    pub close: KrpcCNanoTransportCloseFn,
    pub read: KrpcCNanoTransportReadFn,
    pub write: KrpcCNanoTransportWriteFn,
    pub protocol: KrpcCNanoTransportProtocol,
}
pub type KrpcCNanoTransportWriteFn = Option<
    unsafe extern "C" fn(
        *mut ::core::ffi::c_void,
        *const uint8_t,
        size_t,
        *mut size_t,
        ::core::ffi::c_int,
    ) -> KrpcCNanoTransportStatus,
>;
pub type KrpcCNanoTransportReadFn = Option<
    unsafe extern "C" fn(
        *mut ::core::ffi::c_void,
        *mut uint8_t,
        size_t,
        *mut size_t,
        ::core::ffi::c_int,
    ) -> KrpcCNanoTransportStatus,
>;
pub type KrpcCNanoTransportCloseFn = Option<
    unsafe extern "C" fn(*mut ::core::ffi::c_void) -> KrpcCNanoTransportStatus,
>;
pub type KrpcCNanoTransportOpenFn = Option<
    unsafe extern "C" fn(*mut ::core::ffi::c_void) -> KrpcCNanoTransportStatus,
>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoTransportConfig {
    pub storage: *mut KrpcCNanoTransport,
    pub ops: *const KrpcCNanoTransportOps,
    pub context: *mut ::core::ffi::c_void,
    pub timeout_ms: ::core::ffi::c_int,
}
pub type krpc_connection_t = *mut KrpcCNanoTransport;
pub type krpc_connection_config_t = KrpcCNanoTransportConfig;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoPosixSerial {
    pub fd: ::core::ffi::c_int,
    pub timeout_ms: ::core::ffi::c_int,
    pub baud_rate: ::core::ffi::c_int,
    pub configure_termios: bool,
    pub path: [::core::ffi::c_char; 512],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoPosixTcp {
    pub fd: ::core::ffi::c_int,
    pub timeout_ms: ::core::ffi::c_int,
    pub port: ::core::ffi::c_int,
    pub host: [::core::ffi::c_char; 256],
}
pub type krpc_error_t = ::core::ffi::c_int;
pub const KRPC_ERROR_BUFFER_TOO_SMALL: krpc_error_t = -8;
pub const KRPC_ERROR_DECODING_FAILED: krpc_error_t = -7;
pub const KRPC_ERROR_ENCODING_FAILED: krpc_error_t = -6;
pub const KRPC_ERROR_RPC_FAILED: krpc_error_t = -5;
pub const KRPC_ERROR_NO_RESULTS: krpc_error_t = -4;
pub const KRPC_ERROR_CONNECTION_FAILED: krpc_error_t = -3;
pub const KRPC_ERROR_EOF: krpc_error_t = -2;
pub const KRPC_ERROR_IO: krpc_error_t = -1;
pub const KRPC_OK: krpc_error_t = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoClient {
    pub transport: KrpcCNanoTransport,
    pub serial: KrpcCNanoPosixSerial,
    pub tcp: KrpcCNanoPosixTcp,
    pub connection: krpc_connection_t,
    pub connected: bool,
    pub vessel: krpc_SpaceCenter_Vessel_t,
    pub control: krpc_SpaceCenter_Control_t,
    pub autopilot: krpc_SpaceCenter_AutoPilot_t,
    pub orbit: krpc_SpaceCenter_Orbit_t,
    pub body: krpc_SpaceCenter_CelestialBody_t,
    pub body_nonrotating: krpc_SpaceCenter_ReferenceFrame_t,
    pub vessel_frame: krpc_SpaceCenter_ReferenceFrame_t,
    pub root_frame_aoa_reported: bool,
    pub surface_frame: krpc_SpaceCenter_ReferenceFrame_t,
    pub flight_surface: krpc_SpaceCenter_Flight_t,
    pub flight_body: krpc_SpaceCenter_Flight_t,
    pub flight_vessel: krpc_SpaceCenter_Flight_t,
    pub planet: PlanetModel,
    pub vessel_name: [::core::ffi::c_char; 128],
    pub transport_name: [::core::ffi::c_char; 192],
    pub flight_key: [::core::ffi::c_char; 128],
    pub cached_situation: [::core::ffi::c_char; 64],
    pub burn_thrust_axis_body: Vector3,
    pub burn_thrust_moment_body: Vector3,
    pub burn_geometry_valid: bool,
    pub require_oms: bool,
    pub body_has_oxygen: bool,
    pub burn_resultant_fraction: ::core::ffi::c_double,
    pub engines: [CommandableEngine; 32],
    pub engine_count: size_t,
    pub commandable_thrust: ::core::ffi::c_double,
    pub surfaces: [CommandableSurface; 16],
    pub surface_count: size_t,
    pub surface_inventory_available: bool,
    pub surface_state_valid: bool,
    pub surface_state_ut: ::core::ffi::c_double,
    pub surface_roles_classified: bool,
    pub applied_surface_allocation: KrpcSurfaceAllocation,
    pub has_applied_surface_allocation: bool,
    pub fin_modules: [FinModuleFields; 4],
    pub fin_module_count: size_t,
    pub speedbrake_max_angle_deg: ::core::ffi::c_double,
    pub applied_speedbrake_fraction: ::core::ffi::c_double,
    pub has_applied_speedbrake: bool,
    pub main_wheel_grounded_fields: [krpc_SpaceCenter_PartField_t; 2],
    pub main_wheel_grounded_field_count: size_t,
    pub last_radar_altitude: ::core::ffi::c_double,
    pub cached_gear: bool,
    pub cached_brakes: bool,
    pub cached_airbrakes: bool,
    pub cached_rcs: bool,
    pub cached_sas: bool,
    pub cached_autopilot_engaged: bool,
    pub has_autopilot_engaged: bool,
    pub orbital_autotune_profile_active: bool,
    pub cached_command_throttle: ::core::ffi::c_double,
    pub cached_command_wheel: ::core::ffi::c_double,
    pub has_command_throttle: bool,
    pub has_command_wheel: bool,
    pub has_gear: bool,
    pub has_brakes: bool,
    pub has_airbrakes: bool,
    pub has_rcs: bool,
    pub has_sas: bool,
    pub pause_triplet_sent: bool,
    pub cached_speed_mode: NavballSpeedMode,
    pub has_speed_mode: bool,
    pub cached_speed_of_sound: ::core::ffi::c_double,
    pub cached_static_pressure: ::core::ffi::c_double,
    pub cached_g_force: ::core::ffi::c_double,
    pub cached_available_thrust: ::core::ffi::c_double,
    pub cached_current_thrust: ::core::ffi::c_double,
    pub cached_dry_mass: ::core::ffi::c_double,
    pub cached_torque: [::core::ffi::c_double; 3],
    pub cached_inertia: [::core::ffi::c_double; 3],
    pub cached_lift_vector: Vector3,
    pub cached_drag_vector: Vector3,
    pub cached_center_of_mass: Vector3,
    pub cached_center_of_mass_root: Vector3,
    pub cached_force_vectors_valid: bool,
    pub cached_center_of_mass_valid: bool,
    pub cached_center_of_mass_root_valid: bool,
    pub cached_apoapsis: ::core::ffi::c_double,
    pub cached_periapsis: ::core::ffi::c_double,
    pub cached_period: ::core::ffi::c_double,
    pub last_medium_ut: ::core::ffi::c_double,
    pub last_slow_ut: ::core::ffi::c_double,
    pub medium_valid: bool,
    pub slow_valid: bool,
    pub has_previous_attitude: bool,
    pub previous_ut: ::core::ffi::c_double,
    pub previous_pitch: ::core::ffi::c_double,
    pub previous_roll: ::core::ffi::c_double,
    pub previous_heading: ::core::ffi::c_double,
    pub previous_angle_of_attack: ::core::ffi::c_double,
    pub has_previous_course: bool,
    pub previous_course_ut: ::core::ffi::c_double,
    pub previous_course: ::core::ffi::c_double,
    pub pitch_rate_filter: LowPass,
    pub roll_rate_filter: LowPass,
    pub heading_rate_filter: LowPass,
    pub angle_of_attack_rate_filter: LowPass,
    pub course_rate_filter: LowPass,
    pub total_rpc_calls: ::core::ffi::c_uint,
    pub total_wire_requests: ::core::ffi::c_uint,
    pub last_read_wire_requests: ::core::ffi::c_uint,
    pub last_apply_wire_requests: ::core::ffi::c_uint,
    pub last_read_calls: ::core::ffi::c_uint,
    pub last_apply_calls: ::core::ffi::c_uint,
    pub last_read_seconds: ::core::ffi::c_double,
    pub last_apply_seconds: ::core::ffi::c_double,
}
pub type krpc_SpaceCenter_PartField_t = krpc_object_t;
pub type krpc_object_t = uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FinModuleFields {
    pub deploy: krpc_SpaceCenter_PartField_t,
    pub deploy_angle: krpc_SpaceCenter_PartField_t,
    pub deploy_invert: krpc_SpaceCenter_PartField_t,
    pub ignore_pitch: krpc_SpaceCenter_PartField_t,
    pub ignore_roll: krpc_SpaceCenter_PartField_t,
    pub ignore_yaw: krpc_SpaceCenter_PartField_t,
    pub range: krpc_SpaceCenter_PartField_t,
}
pub type KrpcSurfaceAllocation = ::core::ffi::c_uint;
pub const KRPC_SURFACE_ALLOCATION_SEPARATED_FLAP_PITCH: KrpcSurfaceAllocation = 2;
pub const KRPC_SURFACE_ALLOCATION_SEPARATED: KrpcSurfaceAllocation = 1;
pub const KRPC_SURFACE_ALLOCATION_MIXED: KrpcSurfaceAllocation = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CommandableSurface {
    pub object: krpc_SpaceCenter_ControlSurface_t,
    pub role: KrpcSurfaceRole,
    pub lateral_m: ::core::ffi::c_double,
    pub part_name: [::core::ffi::c_char; 64],
    pub sampled_deployed: bool,
    pub sampled_deflection_override: bool,
    pub sampled_override_deflection: ::core::ffi::c_float,
}
pub type KrpcSurfaceRole = ::core::ffi::c_uint;
pub const KRPC_SURFACE_ROLE_BODY_FLAP: KrpcSurfaceRole = 4;
pub const KRPC_SURFACE_ROLE_RUDDER: KrpcSurfaceRole = 3;
pub const KRPC_SURFACE_ROLE_AILERON: KrpcSurfaceRole = 2;
pub const KRPC_SURFACE_ROLE_ELEVATOR: KrpcSurfaceRole = 1;
pub const KRPC_SURFACE_ROLE_UNKNOWN: KrpcSurfaceRole = 0;
pub type krpc_SpaceCenter_ControlSurface_t = krpc_object_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CommandableEngine {
    pub object: krpc_SpaceCenter_Engine_t,
    pub part: krpc_SpaceCenter_Part_t,
    pub initially_active: bool,
    pub activated_by_client: bool,
    pub independent_owned: bool,
    pub available_thrust: ::core::ffi::c_float,
    pub role: EngineRole,
    pub selected: bool,
    pub airbreathing: bool,
    pub jet_assist_on: bool,
    pub jet_throttle: ::core::ffi::c_float,
}
pub type EngineRole = ::core::ffi::c_uint;
pub const ENGINE_ROLE_MAIN: EngineRole = 2;
pub const ENGINE_ROLE_OMS: EngineRole = 1;
pub const ENGINE_ROLE_OTHER: EngineRole = 0;
pub type krpc_SpaceCenter_Part_t = krpc_object_t;
pub type krpc_SpaceCenter_Engine_t = krpc_object_t;
pub type krpc_SpaceCenter_Flight_t = krpc_object_t;
pub type krpc_SpaceCenter_ReferenceFrame_t = krpc_object_t;
pub type krpc_SpaceCenter_CelestialBody_t = krpc_object_t;
pub type krpc_SpaceCenter_Orbit_t = krpc_object_t;
pub type krpc_SpaceCenter_AutoPilot_t = krpc_object_t;
pub type krpc_SpaceCenter_Control_t = krpc_object_t;
pub type krpc_SpaceCenter_Vessel_t = krpc_object_t;
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
pub type krpc_list_object_t = krpc_list_object_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_list_object_s {
    pub size: size_t,
    pub items: *mut krpc_object_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_result_t {
    pub message: krpc_schema_ProcedureResult,
    pub size: size_t,
    pub buffer: *mut uint8_t,
}
pub type krpc_schema_ProcedureResult = _krpc_schema_ProcedureResult;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_ProcedureResult {
    pub error: pb_callback_t,
    pub value: pb_callback_t,
}
pub type pb_callback_t = pb_callback_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_callback_s {
    pub funcs: C2RustUnnamed,
    pub arg: *mut ::core::ffi::c_void,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub decode: Option<
        unsafe extern "C" fn(
            *mut pb_istream_t,
            *const pb_field_t,
            *mut *mut ::core::ffi::c_void,
        ) -> bool,
    >,
    pub encode: Option<
        unsafe extern "C" fn(
            *mut pb_ostream_t,
            *const pb_field_t,
            *const *mut ::core::ffi::c_void,
        ) -> bool,
    >,
}
pub type pb_field_t = pb_field_iter_t;
pub type pb_field_iter_t = pb_field_iter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_field_iter_s {
    pub descriptor: *const pb_msgdesc_t,
    pub message: *mut ::core::ffi::c_void,
    pub index: pb_size_t,
    pub field_info_index: pb_size_t,
    pub required_field_index: pb_size_t,
    pub submessage_index: pb_size_t,
    pub tag: pb_size_t,
    pub data_size: pb_size_t,
    pub array_size: pb_size_t,
    pub type_0: pb_type_t,
    pub pField: *mut ::core::ffi::c_void,
    pub pData: *mut ::core::ffi::c_void,
    pub pSize: *mut ::core::ffi::c_void,
    pub submsg_desc: *const pb_msgdesc_t,
}
pub type pb_msgdesc_t = pb_msgdesc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_msgdesc_s {
    pub field_info: *const uint32_t,
    pub submsg_info: *const *const pb_msgdesc_t,
    pub default_value: *const pb_byte_t,
    pub field_callback: Option<
        unsafe extern "C" fn(
            *mut pb_istream_t,
            *mut pb_ostream_t,
            *const pb_field_iter_t,
        ) -> bool,
    >,
    pub field_count: pb_size_t,
    pub required_field_count: pb_size_t,
    pub largest_tag: pb_size_t,
}
pub type pb_size_t = uint_least16_t;
pub type pb_ostream_t = pb_ostream_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_ostream_s {
    pub callback: Option<
        unsafe extern "C" fn(*mut pb_ostream_t, *const pb_byte_t, size_t) -> bool,
    >,
    pub state: *mut ::core::ffi::c_void,
    pub max_size: size_t,
    pub bytes_written: size_t,
    pub errmsg: *const ::core::ffi::c_char,
}
pub type pb_byte_t = uint8_t;
pub type pb_istream_t = pb_istream_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pb_istream_s {
    pub callback: Option<
        unsafe extern "C" fn(*mut pb_istream_t, *mut pb_byte_t, size_t) -> bool,
    >,
    pub state: *mut ::core::ffi::c_void,
    pub bytes_left: size_t,
    pub errmsg: *const ::core::ffi::c_char,
}
pub type pb_type_t = pb_byte_t;
pub type krpc_schema_ProcedureCall = _krpc_schema_ProcedureCall;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_ProcedureCall {
    pub arguments: pb_callback_t,
    pub service_id: uint32_t,
    pub procedure_id: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_call_t {
    pub message: krpc_schema_ProcedureCall,
    pub numArguments: size_t,
    pub arguments: *mut krpc_argument_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_argument_t {
    pub message: krpc_schema_Argument,
}
pub type krpc_schema_Argument = _krpc_schema_Argument;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Argument {
    pub position: uint32_t,
    pub value: pb_callback_t,
}
pub type krpc_SpaceCenter_Module_t = krpc_object_t;
pub type krpc_schema_List = _krpc_schema_List;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_List {
    pub items: pb_callback_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State {
    pub capacity: size_t,
    pub value: *mut krpc_list_object_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_0 {
    pub capacity: size_t,
    pub value: *mut krpc_list_object_t,
}
pub type krpc_SpaceCenter_Parts_t = krpc_object_t;
pub type krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_tuple_double_double_double_s {
    pub e0: ::core::ffi::c_double,
    pub e1: ::core::ffi::c_double,
    pub e2: ::core::ffi::c_double,
}
pub type krpc_schema_Tuple = _krpc_schema_Tuple;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _krpc_schema_Tuple {
    pub items: pb_callback_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_1 {
    pub pos: size_t,
    pub value: *mut krpc_tuple_double_double_double_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_2 {
    pub pos: size_t,
    pub value: *mut krpc_tuple_double_double_double_t,
}
pub type krpc_SpaceCenter_Thruster_t = krpc_object_t;
pub type krpc_list_string_t = krpc_list_string_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_list_string_s {
    pub size: size_t,
    pub items: *mut *mut ::core::ffi::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_3 {
    pub capacity: size_t,
    pub value: *mut krpc_list_string_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_4 {
    pub capacity: size_t,
    pub value: *mut krpc_list_string_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ClientBatch {
    pub items: [BatchItem; 32],
    pub count: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct BatchItem {
    pub call: krpc_call_t,
    pub arguments: [krpc_argument_t; 3],
    pub object_arguments: [uint64_t; 3],
    pub uint_arguments: [uint32_t; 3],
    pub float_arguments: [::core::ffi::c_float; 3],
    pub double_arguments: [::core::ffi::c_double; 3],
    pub bool_arguments: [bool; 3],
    pub kind: BatchReturnKind,
    pub target: *mut ::core::ffi::c_void,
    pub required: bool,
}
pub type BatchReturnKind = ::core::ffi::c_uint;
pub const BATCH_TORQUE: BatchReturnKind = 7;
pub const BATCH_TUPLE4: BatchReturnKind = 6;
pub const BATCH_TUPLE3: BatchReturnKind = 5;
pub const BATCH_ENUM: BatchReturnKind = 4;
pub const BATCH_BOOL: BatchReturnKind = 3;
pub const BATCH_FLOAT: BatchReturnKind = 2;
pub const BATCH_DOUBLE: BatchReturnKind = 1;
pub const BATCH_NONE: BatchReturnKind = 0;
pub type krpc_tuple_tuple_double_double_double_tuple_double_double_double_t = krpc_tuple_tuple_double_double_double_tuple_double_double_double_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_tuple_tuple_double_double_double_tuple_double_double_double_s {
    pub e0: krpc_tuple_double_double_double_t,
    pub e1: krpc_tuple_double_double_double_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_5 {
    pub pos: size_t,
    pub value: *mut krpc_tuple_tuple_double_double_double_tuple_double_double_double_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_6 {
    pub pos: size_t,
    pub value: *mut krpc_tuple_tuple_double_double_double_tuple_double_double_double_t,
}
pub type krpc_tuple_double_double_double_double_t = krpc_tuple_double_double_double_double_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct krpc_tuple_double_double_double_double_s {
    pub e0: ::core::ffi::c_double,
    pub e1: ::core::ffi::c_double,
    pub e2: ::core::ffi::c_double,
    pub e3: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_7 {
    pub pos: size_t,
    pub value: *mut krpc_tuple_double_double_double_double_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct State_8 {
    pub pos: size_t,
    pub value: *mut krpc_tuple_double_double_double_double_t,
}
pub type krpc_enum_t = ::core::ffi::c_int;
pub const PROC_CELESTIALBODY_DENSITY_AT: C2RustUnnamed_0 = 241;
pub const PROC_CELESTIALBODY_PRESSURE_AT: C2RustUnnamed_0 = 242;
pub const PROC_ENGINE_SET_ACTIVE: C2RustUnnamed_0 = 817;
pub const PROC_ENGINE_SET_INDEPENDENT_THROTTLE: C2RustUnnamed_0 = 837;
pub const PROC_ENGINE_SET_THROTTLE: C2RustUnnamed_0 = 834;
pub const PROC_CONTROL_SET_RCS: C2RustUnnamed_0 = 362;
pub const PROC_CONTROL_SET_WHEEL_STEERING: C2RustUnnamed_0 = 424;
pub const PROC_CONTROL_SET_THROTTLE: C2RustUnnamed_0 = 400;
pub const PROC_CONTROL_SET_YAW: C2RustUnnamed_0 = 406;
pub const PROC_CONTROL_SET_ROLL: C2RustUnnamed_0 = 408;
pub const PROC_CONTROL_SET_PITCH: C2RustUnnamed_0 = 404;
pub const PROC_CONTROL_SET_SAS: C2RustUnnamed_0 = 356;
pub const PROC_AUTOPILOT_SET_ENGAGED: C2RustUnnamed_0 = 112;
pub const PROC_ORBIT_PERIOD: C2RustUnnamed_0 = 567;
pub const PROC_ORBIT_PERIAPSIS_ALTITUDE: C2RustUnnamed_0 = 562;
pub const PROC_ORBIT_APOAPSIS_ALTITUDE: C2RustUnnamed_0 = 561;
pub const PROC_CONTROL_SURFACE_DEFLECTION: C2RustUnnamed_0 = 780;
pub const PROC_CONTROL_SURFACE_DEFLECTION_OVERRIDE: C2RustUnnamed_0 = 778;
pub const PROC_CONTROL_SURFACE_DEPLOYED: C2RustUnnamed_0 = 776;
pub const PROC_UT: C2RustUnnamed_0 = 51;
pub type krpc_SpaceCenter_VesselSituation_t = ::core::ffi::c_uint;
pub const KRPC_SPACECENTER_VESSELSITUATION_DOCKED: krpc_SpaceCenter_VesselSituation_t = 7;
pub const KRPC_SPACECENTER_VESSELSITUATION_SPLASHED: krpc_SpaceCenter_VesselSituation_t = 6;
pub const KRPC_SPACECENTER_VESSELSITUATION_LANDED: krpc_SpaceCenter_VesselSituation_t = 5;
pub const KRPC_SPACECENTER_VESSELSITUATION_FLYING: krpc_SpaceCenter_VesselSituation_t = 4;
pub const KRPC_SPACECENTER_VESSELSITUATION_ESCAPING: krpc_SpaceCenter_VesselSituation_t = 3;
pub const KRPC_SPACECENTER_VESSELSITUATION_SUBORBITAL: krpc_SpaceCenter_VesselSituation_t = 2;
pub const KRPC_SPACECENTER_VESSELSITUATION_ORBITING: krpc_SpaceCenter_VesselSituation_t = 1;
pub const KRPC_SPACECENTER_VESSELSITUATION_PRELAUNCH: krpc_SpaceCenter_VesselSituation_t = 0;
pub const KRPC_SPACECENTER_SPEEDMODE_ORBIT: krpc_SpaceCenter_SpeedMode_t = 0;
pub const KRPC_SPACECENTER_SPEEDMODE_SURFACE: krpc_SpaceCenter_SpeedMode_t = 1;
pub const PROC_VESSEL_SITUATION: C2RustUnnamed_0 = 646;
pub const PROC_CONTROL_SPEED_MODE: C2RustUnnamed_0 = 359;
pub const PROC_AUTOPILOT_ENGAGED: C2RustUnnamed_0 = 111;
pub const PROC_CONTROL_SAS: C2RustUnnamed_0 = 355;
pub const PROC_CONTROL_RCS: C2RustUnnamed_0 = 361;
pub const PROC_CONTROL_GET_ACTION_GROUP: C2RustUnnamed_0 = 347;
pub const PROC_CONTROL_BRAKES: C2RustUnnamed_0 = 379;
pub const PROC_CONTROL_GEAR: C2RustUnnamed_0 = 371;
pub const PROC_FLIGHT_CENTER_OF_MASS: C2RustUnnamed_0 = 475;
pub const PROC_FLIGHT_DRAG: C2RustUnnamed_0 = 496;
pub const PROC_FLIGHT_LIFT: C2RustUnnamed_0 = 495;
pub const PROC_VESSEL_MOMENT_OF_INERTIA: C2RustUnnamed_0 = 682;
pub const PROC_VESSEL_AVAILABLE_TORQUE: C2RustUnnamed_0 = 684;
pub const PROC_VESSEL_THRUST: C2RustUnnamed_0 = 671;
pub const PROC_VESSEL_AVAILABLE_THRUST: C2RustUnnamed_0 = 672;
pub const PROC_VESSEL_DRY_MASS: C2RustUnnamed_0 = 670;
pub const PROC_FLIGHT_G_FORCE: C2RustUnnamed_0 = 463;
pub const PROC_FLIGHT_STATIC_PRESSURE: C2RustUnnamed_0 = 492;
pub const PROC_FLIGHT_SPEED_OF_SOUND: C2RustUnnamed_0 = 500;
pub const PROC_PART_FIELD_BOOL_VALUE: C2RustUnnamed_0 = 1064;
pub const PROC_VESSEL_ROTATION: C2RustUnnamed_0 = 639;
pub const PROC_AUTOPILOT_ERROR: C2RustUnnamed_0 = 138;
pub const PROC_CONTROL_THROTTLE: C2RustUnnamed_0 = 399;
pub const PROC_CONTROL_YAW: C2RustUnnamed_0 = 405;
pub const PROC_CONTROL_ROLL: C2RustUnnamed_0 = 407;
pub const PROC_CONTROL_PITCH: C2RustUnnamed_0 = 403;
pub const PROC_VESSEL_MASS: C2RustUnnamed_0 = 669;
pub const PROC_VESSEL_ANGULAR_VELOCITY: C2RustUnnamed_0 = 641;
pub const PROC_VESSEL_VELOCITY: C2RustUnnamed_0 = 638;
pub const PROC_VESSEL_POSITION: C2RustUnnamed_0 = 636;
pub const PROC_FLIGHT_DYNAMIC_PRESSURE: C2RustUnnamed_0 = 490;
pub const PROC_FLIGHT_SIDESLIP: C2RustUnnamed_0 = 507;
pub const PROC_FLIGHT_AOA: C2RustUnnamed_0 = 506;
pub const PROC_FLIGHT_ROLL: C2RustUnnamed_0 = 480;
pub const PROC_FLIGHT_PITCH: C2RustUnnamed_0 = 478;
pub const PROC_FLIGHT_HEADING: C2RustUnnamed_0 = 479;
pub const PROC_FLIGHT_TRUE_AIR_SPEED: C2RustUnnamed_0 = 503;
pub const PROC_FLIGHT_SURFACE_ALTITUDE: C2RustUnnamed_0 = 465;
pub const KRPC_SPACECENTER_WARPMODE_PHYSICS: krpc_SpaceCenter_WarpMode_t = 1;
pub type krpc_SpaceCenter_WarpMode_t = ::core::ffi::c_uint;
pub const KRPC_SPACECENTER_WARPMODE_NONE: krpc_SpaceCenter_WarpMode_t = 2;
pub const KRPC_SPACECENTER_WARPMODE_RAILS: krpc_SpaceCenter_WarpMode_t = 0;
pub type krpc_SpaceCenter_SpeedMode_t = ::core::ffi::c_uint;
pub const KRPC_SPACECENTER_SPEEDMODE_TARGET: krpc_SpaceCenter_SpeedMode_t = 2;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KrpcCNanoWarpDecision {
    pub factor: int32_t,
    pub predicted_rate: ::core::ffi::c_double,
    pub response_horizon_seconds: ::core::ffi::c_double,
    pub stopping_margin_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CNanoWarpState {
    pub ut: ::core::ffi::c_double,
    pub factor: int32_t,
    pub rate: ::core::ffi::c_double,
    pub latency_seconds: ::core::ffi::c_double,
}
pub type C2RustUnnamed_0 = ::core::ffi::c_uint;
pub const PROC_AUTOPILOT_SET_AUTOTUNE: C2RustUnnamed_0 = 159;
pub const DBL_MAX: ::core::ffi::c_double = __DBL_MAX__;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const RAD2DEG: ::core::ffi::c_double = 180.0f64 / LANDER_PI;
pub const LANDER_ATMOSPHERE_SAMPLE_MAX: ::core::ffi::c_int = 1024 as ::core::ffi::c_int;
pub const LANDING_CONTROL_SURFACE_MAX: ::core::ffi::c_int = 16 as ::core::ffi::c_int;
pub const CNANO_ENGINE_LIMIT: ::core::ffi::c_int = 32 as ::core::ffi::c_int;
pub const CNANO_ATMOSPHERE_SAMPLES: ::core::ffi::c_int = 71 as ::core::ffi::c_int;
pub const CNANO_MAIN_WHEEL_LIMIT: ::core::ffi::c_int = 2 as ::core::ffi::c_int;
pub const CNANO_CONTROL_SURFACE_LIMIT: ::core::ffi::c_int = 16 as ::core::ffi::c_int;
pub const CNANO_MEDIUM_PERIOD: ::core::ffi::c_double = 0.25f64;
pub const CNANO_SLOW_PERIOD: ::core::ffi::c_double = 1.0f64;
#[inline]
unsafe extern "C" fn krpc_decode_callback_item_list_string(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut state: *mut State_4 = *arg as *mut State_4;
    let mut i: size_t = (*(*state).value).size;
    (*(*state).value).size = (*(*state).value).size.wrapping_add(1);
    if (*state).capacity > 0 as size_t && (*(*state).value).size > (*state).capacity {
        (*(*state).value).items = krpc_recalloc(
            (*(*state).value).items as *mut ::core::ffi::c_void,
            (*state).capacity,
            KRPC_ALLOC_BLOCK_SIZE as size_t,
            ::core::mem::size_of::<*mut ::core::ffi::c_char>() as size_t,
        ) as *mut *mut ::core::ffi::c_char;
        (*state).capacity = (*state)
            .capacity
            .wrapping_add(KRPC_ALLOC_BLOCK_SIZE as size_t);
    }
    if krpc_decode_string(
        stream,
        (*(*state).value).items.offset(i as isize) as *mut *mut ::core::ffi::c_char,
    ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_decode_list_string(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_list_string_t,
) -> krpc_error_t {
    let mut state: State_3 = State_3 {
        capacity: 0 as size_t,
        value: value,
    };
    (*value).size = 0 as size_t;
    if (*value).items.is_null() {
        (*value).items = krpc_calloc(
            KRPC_ALLOC_BLOCK_SIZE as size_t,
            ::core::mem::size_of::<*mut ::core::ffi::c_char>() as size_t,
        ) as *mut *mut ::core::ffi::c_char;
        state.capacity = KRPC_ALLOC_BLOCK_SIZE as size_t;
    }
    let mut message: krpc_schema_List = _krpc_schema_List {
        items: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL_0,
        },
    };
    message.items.funcs.decode = Some(
        krpc_decode_callback_item_list_string
            as unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    message.items.arg = &raw mut state as *mut ::core::ffi::c_void;
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_decode_message_List(stream, &raw mut message);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_encode_callback_items_tuple_double_double_double(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    let mut value: *const krpc_tuple_double_double_double_t = *arg
        as *const krpc_tuple_double_double_double_t;
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut size: size_t = 0;
    if krpc_encode_size_double(&raw mut size, (*value).e0) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_double(stream, (*value).e0) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut size_0: size_t = 0;
    if krpc_encode_size_double(&raw mut size_0, (*value).e1) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size_0 as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_double(stream, (*value).e1) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut size_1: size_t = 0;
    if krpc_encode_size_double(&raw mut size_1, (*value).e2) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size_1 as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_double(stream, (*value).e2) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_encode_tuple_double_double_double(
    mut stream: *mut pb_ostream_t,
    mut value: *const krpc_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut message: krpc_schema_Tuple = _krpc_schema_Tuple {
        items: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL_0,
        },
    };
    message.items.funcs.encode = Some(
        krpc_encode_callback_items_tuple_double_double_double
            as unsafe extern "C" fn(
                *mut pb_ostream_t,
                *const pb_field_t,
                *const *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_ostream_t,
                *const pb_field_t,
                *const *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    message.items.arg = value as *mut krpc_tuple_double_double_double_t
        as *mut ::core::ffi::c_void;
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_message_Tuple(stream, &raw mut message);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_encode_size_tuple_double_double_double(
    mut size: *mut size_t,
    mut value: *const krpc_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut stream: pb_ostream_t = pb_ostream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        max_size: 0 as size_t,
        bytes_written: 0 as size_t,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_encode_tuple_double_double_double(&raw mut stream, value);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    *size = stream.bytes_written;
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_encode_callback_tuple_double_double_double(
    mut stream: *mut pb_ostream_t,
    mut field: *const pb_field_t,
    mut arg: *const *mut ::core::ffi::c_void,
) -> bool {
    if !pb_encode_tag_for_field(stream, field as *const pb_field_iter_t) {
        return false_0 != 0;
    }
    let mut value: *mut krpc_tuple_double_double_double_t = *arg
        as *mut krpc_tuple_double_double_double_t;
    let mut size: size_t = 0;
    if krpc_encode_size_tuple_double_double_double(&raw mut size, value)
        as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if !pb_encode_varint(stream, size as uint64_t) {
        return false_0 != 0;
    }
    if krpc_encode_tuple_double_double_double(stream, value) as ::core::ffi::c_int
        != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_decode_callback_item_tuple_double_double_double(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut state: *mut State_2 = *arg as *mut State_2;
    let mut value: *mut krpc_tuple_double_double_double_t = (*state).value;
    match (*state).pos {
        0 => {
            if krpc_decode_double(stream, &raw mut (*value).e0) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        1 => {
            if krpc_decode_double(stream, &raw mut (*value).e1) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        2 => {
            if krpc_decode_double(stream, &raw mut (*value).e2) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        _ => return false_0 != 0,
    }
    (*state).pos = (*state).pos.wrapping_add(1);
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_decode_tuple_double_double_double(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut message: krpc_schema_Tuple = _krpc_schema_Tuple {
        items: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL_0,
        },
    };
    message.items.funcs.decode = Some(
        krpc_decode_callback_item_tuple_double_double_double
            as unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    let mut state: State_1 = State_1 {
        pos: 0 as size_t,
        value: value,
    };
    message.items.arg = &raw mut state as *mut ::core::ffi::c_void;
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_decode_message_Tuple(stream, &raw mut message);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_decode_callback_item_tuple_double_double_double_double(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut state: *mut State_8 = *arg as *mut State_8;
    let mut value: *mut krpc_tuple_double_double_double_double_t = (*state).value;
    match (*state).pos {
        0 => {
            if krpc_decode_double(stream, &raw mut (*value).e0) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        1 => {
            if krpc_decode_double(stream, &raw mut (*value).e1) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        2 => {
            if krpc_decode_double(stream, &raw mut (*value).e2) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        3 => {
            if krpc_decode_double(stream, &raw mut (*value).e3) as ::core::ffi::c_int
                != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        _ => return false_0 != 0,
    }
    (*state).pos = (*state).pos.wrapping_add(1);
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_decode_tuple_double_double_double_double(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_tuple_double_double_double_double_t,
) -> krpc_error_t {
    let mut message: krpc_schema_Tuple = _krpc_schema_Tuple {
        items: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL_0,
        },
    };
    message.items.funcs.decode = Some(
        krpc_decode_callback_item_tuple_double_double_double_double
            as unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    let mut state: State_7 = State_7 {
        pos: 0 as size_t,
        value: value,
    };
    message.items.arg = &raw mut state as *mut ::core::ffi::c_void;
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_decode_message_Tuple(stream, &raw mut message);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_decode_callback_item_tuple_tuple_double_double_double_tuple_double_double_double(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut state: *mut State_6 = *arg as *mut State_6;
    let mut value: *mut krpc_tuple_tuple_double_double_double_tuple_double_double_double_t = (*state)
        .value;
    match (*state).pos {
        0 => {
            if krpc_decode_tuple_double_double_double(stream, &raw mut (*value).e0)
                as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        1 => {
            if krpc_decode_tuple_double_double_double(stream, &raw mut (*value).e1)
                as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            {
                return false_0 != 0;
            }
        }
        _ => return false_0 != 0,
    }
    (*state).pos = (*state).pos.wrapping_add(1);
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_decode_tuple_tuple_double_double_double_tuple_double_double_double(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_tuple_tuple_double_double_double_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut message: krpc_schema_Tuple = _krpc_schema_Tuple {
        items: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL_0,
        },
    };
    message.items.funcs.decode = Some(
        krpc_decode_callback_item_tuple_tuple_double_double_double_tuple_double_double_double
            as unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    let mut state: State_5 = State_5 {
        pos: 0 as size_t,
        value: value,
    };
    message.items.arg = &raw mut state as *mut ::core::ffi::c_void;
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_decode_message_Tuple(stream, &raw mut message);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Save(
    mut connection: krpc_connection_t,
    mut name: *const ::core::ffi::c_char,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        6 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_string
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut name as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_ActiveVessel(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        30 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_MaximumRailsWarpFactor(
    mut connection: krpc_connection_t,
    mut returnValue: *mut int32_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        60 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_int32(&raw mut _stream, returnValue);
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_PhysicsWarpFactor(
    mut connection: krpc_connection_t,
    mut returnValue: *mut int32_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        58 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_int32(&raw mut _stream, returnValue);
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_set_PhysicsWarpFactor(
    mut connection: krpc_connection_t,
    mut value: int32_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        59 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_int32
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_RailsWarpFactor(
    mut connection: krpc_connection_t,
    mut returnValue: *mut int32_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        56 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_int32(&raw mut _stream, returnValue);
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_set_RailsWarpFactor(
    mut connection: krpc_connection_t,
    mut value: int32_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        57 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_int32
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_UT(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_double,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        51 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_double(
            &raw mut _stream,
            returnValue,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_WarpMode(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_WarpMode_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        53 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_enum(
            &raw mut _stream,
            returnValue as *mut krpc_enum_t,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_WarpRate(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_float,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        54 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_float(&raw mut _stream, returnValue);
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_Reset(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        103 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_SetDirectionAndUp(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut direction: *const krpc_tuple_double_double_double_t,
    mut up: *const krpc_tuple_double_double_double_t,
    mut roll: ::core::ffi::c_float,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 4] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 4];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        105 as uint32_t,
        4 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_tuple_double_double_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        direction as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        2 as uint32_t,
        Some(
            krpc_encode_callback_tuple_double_double_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        up as *const ::core::ffi::c_void,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        3 as uint32_t,
        Some(
            krpc_encode_callback_float
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut roll as *const ::core::ffi::c_void,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_4: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    let mut error_5: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    let mut error_6: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_6;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_AutoTune(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        159 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_Engaged(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        112 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_Overshoot(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: *const krpc_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        167 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_tuple_double_double_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_ReferenceFrame(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        116 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_SoftStartTime(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: ::core::ffi::c_double,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        163 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_TargetDirection(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: *const krpc_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        126 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_tuple_double_double_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_TargetRoll(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: ::core::ffi::c_float,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        122 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_float
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_AutoPilot_set_TimeToPeak(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_AutoPilot_t,
    mut value: *const krpc_tuple_double_double_double_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        161 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_tuple_double_double_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_SurfacePosition(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_tuple_double_double_double_t,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
    mut latitude: ::core::ffi::c_double,
    mut longitude: ::core::ffi::c_double,
    mut referenceFrame: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 4] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 4];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        233 as uint32_t,
        4 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut latitude as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        2 as uint32_t,
        Some(
            krpc_encode_callback_double
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut longitude as *const ::core::ffi::c_void,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        3 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut referenceFrame as *const ::core::ffi::c_void,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_4: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    let mut error_5: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_6: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_6;
        }
        let mut error_7: krpc_error_t = krpc_decode_tuple_double_double_double(
            &raw mut _stream,
            returnValue,
        );
        if error_7 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_7;
        }
    }
    let mut error_8: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_8 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_8;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_AtmosphereDepth(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_double,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        264 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_double(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_EquatorialRadius(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_double,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        258 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_double(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_GravitationalParameter(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_double,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        252 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_double(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_HasAtmosphericOxygen(
    mut connection: krpc_connection_t,
    mut returnValue: *mut bool,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        265 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_bool(&raw mut _stream, returnValue);
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_Name(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        249 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_NonRotatingReferenceFrame(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_ReferenceFrame_t,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        270 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_CelestialBody_RotationalSpeed(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_double,
    mut instance: krpc_SpaceCenter_CelestialBody_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        255 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_double(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_SetActionGroup(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut group: uint32_t,
    mut state: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 3] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 3];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        348 as uint32_t,
        3 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_uint32
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut group as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        2 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut state as *const ::core::ffi::c_void,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_3: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_Brakes(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        380 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_Gear(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        372 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_Parachutes(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        388 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_RCS(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        362 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_SAS(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        356 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_SpeedMode(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: krpc_SpaceCenter_SpeedMode_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        360 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_enum
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Control_set_Throttle(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Control_t,
    mut value: ::core::ffi::c_float,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        400 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_float
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_ControlSurface_Part(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Part_t,
    mut instance: krpc_SpaceCenter_ControlSurface_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        765 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_ControlSurface_set_PitchEnabled(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_ControlSurface_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        767 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_ControlSurface_set_RollEnabled(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_ControlSurface_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        771 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_ControlSurface_set_YawEnabled(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_ControlSurface_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        769 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_Active(
    mut connection: krpc_connection_t,
    mut returnValue: *mut bool,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        816 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_bool(&raw mut _stream, returnValue);
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_set_Active(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Engine_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        817 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_AvailableThrust(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_float,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        819 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_float(&raw mut _stream, returnValue);
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_HasFuel(
    mut connection: krpc_connection_t,
    mut returnValue: *mut bool,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        831 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_bool(&raw mut _stream, returnValue);
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_set_IndependentThrottle(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Engine_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        837 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_Part(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Part_t,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        815 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_PropellantNames(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_string_t,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        828 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_set_Throttle(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_Engine_t,
    mut value: ::core::ffi::c_float,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        834 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_float
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_ThrottleLocked(
    mut connection: krpc_connection_t,
    mut returnValue: *mut bool,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        835 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_bool(&raw mut _stream, returnValue);
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Engine_Thrusters(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_object_t,
    mut instance: krpc_SpaceCenter_Engine_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        824 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_object(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Module_FieldList(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_object_t,
    mut instance: krpc_SpaceCenter_Module_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        935 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_object(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Module_Name(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_Module_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        932 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Orbit_Body(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_CelestialBody_t,
    mut instance: krpc_SpaceCenter_Orbit_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        558 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Part_Direction(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_tuple_double_double_double_t,
    mut instance: krpc_SpaceCenter_Part_t,
    mut referenceFrame: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        959 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut referenceFrame as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_4: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
        let mut error_5: krpc_error_t = krpc_decode_tuple_double_double_double(
            &raw mut _stream,
            returnValue,
        );
        if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_5;
        }
    }
    let mut error_6: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_6;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Part_Position(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_tuple_double_double_double_t,
    mut instance: krpc_SpaceCenter_Part_t,
    mut referenceFrame: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        956 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut referenceFrame as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_4: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
        let mut error_5: krpc_error_t = krpc_decode_tuple_double_double_double(
            &raw mut _stream,
            returnValue,
        );
        if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_5;
        }
    }
    let mut error_6: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_6;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Part_Modules(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_object_t,
    mut instance: krpc_SpaceCenter_Part_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1011 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_object(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Part_Name(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_Part_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        966 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Part_Tag(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_Part_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        969 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Part_Title(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_Part_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        967 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_PartField_set_BoolValue(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_PartField_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1065 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_PartField_FloatValue(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_float,
    mut instance: krpc_SpaceCenter_PartField_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1068 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_float(&raw mut _stream, returnValue);
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_PartField_set_FloatValue(
    mut connection: krpc_connection_t,
    mut instance: krpc_SpaceCenter_PartField_t,
    mut value: ::core::ffi::c_float,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1069 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_float
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_PartField_Name(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_PartField_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1058 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Parts_All(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_object_t,
    mut instance: krpc_SpaceCenter_Parts_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1079 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_object(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Parts_ControlSurfaces(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_object_t,
    mut instance: krpc_SpaceCenter_Parts_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1084 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_object(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Parts_Controlling(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Part_t,
    mut instance: krpc_SpaceCenter_Parts_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1081 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Parts_Engines(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_list_object_t,
    mut instance: krpc_SpaceCenter_Parts_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1088 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_list_object(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Parts_Root(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Part_t,
    mut instance: krpc_SpaceCenter_Parts_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1080 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Thruster_InitialThrustDirection(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_tuple_double_double_double_t,
    mut instance: krpc_SpaceCenter_Thruster_t,
    mut referenceFrame: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1316 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut referenceFrame as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_4: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
        let mut error_5: krpc_error_t = krpc_decode_tuple_double_double_double(
            &raw mut _stream,
            returnValue,
        );
        if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_5;
        }
    }
    let mut error_6: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_6;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Thruster_InitialThrustPosition(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_tuple_double_double_double_t,
    mut instance: krpc_SpaceCenter_Thruster_t,
    mut referenceFrame: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        1315 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut referenceFrame as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_4: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
        let mut error_5: krpc_error_t = krpc_decode_tuple_double_double_double(
            &raw mut _stream,
            returnValue,
        );
        if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_5;
        }
    }
    let mut error_6: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_6;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_Flight(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Flight_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
    mut referenceFrame: krpc_SpaceCenter_ReferenceFrame_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 2] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 2];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        627 as uint32_t,
        2 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        1 as uint32_t,
        Some(
            krpc_encode_callback_object
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut referenceFrame as *const ::core::ffi::c_void,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_2: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_4: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
        let mut error_5: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_5;
        }
    }
    let mut error_6: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_6 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_6;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_AutoPilot(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_AutoPilot_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        657 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_Control(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Control_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        655 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_MET(
    mut connection: krpc_connection_t,
    mut returnValue: *mut ::core::ffi::c_double,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        652 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_double(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_Name(
    mut connection: krpc_connection_t,
    mut returnValue: *mut *mut ::core::ffi::c_char,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        642 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_string(
            &raw mut _stream,
            returnValue,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_Orbit(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Orbit_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        654 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_Parts(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_Parts_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        668 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_ReferenceFrame(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_ReferenceFrame_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        698 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_SpaceCenter_Vessel_SurfaceReferenceFrame(
    mut connection: krpc_connection_t,
    mut returnValue: *mut krpc_SpaceCenter_ReferenceFrame_t,
    mut instance: krpc_SpaceCenter_Vessel_t,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        2 as uint32_t,
        700 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_uint64
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut instance as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_3: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
        let mut error_4: krpc_error_t = krpc_decode_object(
            &raw mut _stream,
            returnValue as *mut krpc_object_t,
        );
        if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_4;
        }
    }
    let mut error_5: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_5 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_5;
    }
    return KRPC_OK;
}
pub const KRPC_NULL: uint64_t = 0 as ::core::ffi::c_int as uint64_t;
unsafe extern "C" fn monotonic_seconds() -> ::core::ffi::c_double {
    let mut ts: timespec = timespec { tv_sec: 0, tv_nsec: 0 };
    if clock_gettime(CLOCK_MONOTONIC, &raw mut ts) != 0 as ::core::ffi::c_int {
        return 0.0f64;
    }
    return ts.tv_sec as ::core::ffi::c_double
        + ts.tv_nsec as ::core::ffi::c_double / 1e9f64;
}
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
unsafe extern "C" fn rpc_failed(
    mut client: *mut KrpcCNanoClient,
    mut code: krpc_error_t,
    mut name: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !client.is_null() {
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    }
    if code as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int {
        return false_0 != 0;
    }
    let mut message: *const ::core::ffi::c_char = krpc_get_error_message();
    let mut transport: KrpcCNanoTransportStatus = (if !client.is_null()
        && !(*client).connection.is_null()
    {
        krpc_cnano_transport_last_status((*client).connection) as ::core::ffi::c_uint
    } else {
        KRPC_CNANO_TRANSPORT_INVALID as ::core::ffi::c_int as ::core::ffi::c_uint
    }) as KrpcCNanoTransportStatus;
    set_error!(
        error,
        error_size,
        b"%s failed: %s%s%s (transport=%s)\0" as *const u8 as *const ::core::ffi::c_char,
        if !name.is_null() {
            name
        } else {
            b"kRPC\0" as *const u8 as *const ::core::ffi::c_char
        },
        krpc_get_error(code),
        if !message.is_null() && *message as ::core::ffi::c_int != 0 {
            b": \0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
        if !message.is_null() && *message as ::core::ffi::c_int != 0 {
            message
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
        krpc_cnano_transport_status_string(transport),
    );
    return true_0 != 0;
}
unsafe extern "C" fn tuple_to_canonical(
    mut value: krpc_tuple_double_double_double_t,
) -> Vector3 {
    return v3(value.e0, value.e2, value.e1);
}
unsafe extern "C" fn canonical_to_tuple(
    mut value: Vector3,
) -> krpc_tuple_double_double_double_t {
    let mut tuple: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0.,
        e1: 0.,
        e2: 0.,
    };
    tuple.e0 = value.x;
    tuple.e1 = value.z;
    tuple.e2 = value.y;
    return tuple;
}
unsafe extern "C" fn finite_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 { value } else { fallback };
}
unsafe extern "C" fn batch_add(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut kind: BatchReturnKind,
    mut target: *mut ::core::ffi::c_void,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh10 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh10 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = kind;
    (*item).target = target;
    (*item).required = required;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        0 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_add_object(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut instance: krpc_object_t,
    mut kind: BatchReturnKind,
    mut target: *mut ::core::ffi::c_void,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh9 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh9 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = kind;
    (*item).target = target;
    (*item).required = required;
    (*item).object_arguments[0 as ::core::ffi::c_int as usize] = instance as uint64_t;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        1 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            0 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(0 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_add_two_objects(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut first: krpc_object_t,
    mut second: krpc_object_t,
    mut kind: BatchReturnKind,
    mut target: *mut ::core::ffi::c_void,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh12 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh12 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = kind;
    (*item).target = target;
    (*item).required = required;
    (*item).object_arguments[0 as ::core::ffi::c_int as usize] = first as uint64_t;
    (*item).object_arguments[1 as ::core::ffi::c_int as usize] = second as uint64_t;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        2 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            0 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(0 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            1 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(1 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_add_object_uint(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut instance: krpc_object_t,
    mut value: uint32_t,
    mut kind: BatchReturnKind,
    mut target: *mut ::core::ffi::c_void,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh11 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh11 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = kind;
    (*item).target = target;
    (*item).required = required;
    (*item).object_arguments[0 as ::core::ffi::c_int as usize] = instance as uint64_t;
    (*item).uint_arguments[1 as ::core::ffi::c_int as usize] = value;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        2 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            0 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(0 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            1 as uint32_t,
            Some(
                krpc_encode_callback_uint32
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).uint_arguments as *mut uint32_t)
                .offset(1 as ::core::ffi::c_int as isize) as *mut uint32_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_add_object_double(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut instance: krpc_object_t,
    mut value: ::core::ffi::c_double,
    mut kind: BatchReturnKind,
    mut target: *mut ::core::ffi::c_void,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh6 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh6 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = kind;
    (*item).target = target;
    (*item).required = required;
    (*item).object_arguments[0 as ::core::ffi::c_int as usize] = instance as uint64_t;
    (*item).double_arguments[1 as ::core::ffi::c_int as usize] = value;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        2 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            0 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(0 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            1 as uint32_t,
            Some(
                krpc_encode_callback_double
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).double_arguments as *mut ::core::ffi::c_double)
                .offset(1 as ::core::ffi::c_int as isize) as *mut ::core::ffi::c_double
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_add_object_float(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut instance: krpc_object_t,
    mut value: ::core::ffi::c_float,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh8 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh8 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = BATCH_NONE;
    (*item).required = required;
    (*item).object_arguments[0 as ::core::ffi::c_int as usize] = instance as uint64_t;
    (*item).float_arguments[1 as ::core::ffi::c_int as usize] = value;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        2 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            0 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(0 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            1 as uint32_t,
            Some(
                krpc_encode_callback_float
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).float_arguments as *mut ::core::ffi::c_float)
                .offset(1 as ::core::ffi::c_int as isize) as *mut ::core::ffi::c_float
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_add_object_bool(
    mut batch: *mut ClientBatch,
    mut procedure: uint32_t,
    mut instance: krpc_object_t,
    mut value: bool,
    mut required: bool,
) -> bool {
    if batch.is_null() || (*batch).count >= KRPC_CNANO_BATCH_MAX as size_t {
        return false_0 != 0;
    }
    let fresh7 = (*batch).count;
    (*batch).count = (*batch).count.wrapping_add(1);
    let mut item: *mut BatchItem = (&raw mut (*batch).items as *mut BatchItem)
        .offset(fresh7 as isize) as *mut BatchItem;
    memset(
        item as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<BatchItem>() as size_t,
    );
    (*item).kind = BATCH_NONE;
    (*item).required = required;
    (*item).object_arguments[0 as ::core::ffi::c_int as usize] = instance as uint64_t;
    (*item).bool_arguments[1 as ::core::ffi::c_int as usize] = value;
    return krpc_call(
        &raw mut (*item).call,
        2 as uint32_t,
        procedure,
        2 as size_t,
        &raw mut (*item).arguments as *mut krpc_argument_t,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            0 as uint32_t,
            Some(
                krpc_encode_callback_uint64
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).object_arguments as *mut uint64_t)
                .offset(0 as ::core::ffi::c_int as isize) as *mut uint64_t
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && krpc_add_argument(
            &raw mut (*item).call,
            1 as uint32_t,
            Some(
                krpc_encode_callback_bool
                    as unsafe extern "C" fn(
                        *mut pb_ostream_t,
                        *const pb_field_t,
                        *const *mut ::core::ffi::c_void,
                    ) -> bool,
            ),
            (&raw mut (*item).bool_arguments as *mut bool)
                .offset(1 as ::core::ffi::c_int as isize) as *mut bool
                as *const ::core::ffi::c_void,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int;
}
unsafe extern "C" fn batch_decode_item(
    mut item: *const BatchItem,
    mut result: *mut krpc_result_t,
) -> krpc_error_t {
    let mut stream: pb_istream_t = pb_istream_s {
        callback: None,
        state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        bytes_left: 0,
        errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
    };
    if item.is_null() || result.is_null()
        || krpc_get_return_value(result, &raw mut stream) as ::core::ffi::c_int
            != KRPC_OK as ::core::ffi::c_int
    {
        return KRPC_ERROR_DECODING_FAILED;
    }
    match (*item).kind as ::core::ffi::c_uint {
        0 => return KRPC_OK,
        1 => {
            return krpc_decode_double(
                &raw mut stream,
                (*item).target as *mut ::core::ffi::c_double,
            );
        }
        2 => {
            return krpc_decode_float(
                &raw mut stream,
                (*item).target as *mut ::core::ffi::c_float,
            );
        }
        3 => return krpc_decode_bool(&raw mut stream, (*item).target as *mut bool),
        4 => return krpc_decode_enum(&raw mut stream, (*item).target as *mut krpc_enum_t),
        5 => {
            return krpc_decode_tuple_double_double_double(
                &raw mut stream,
                (*item).target as *mut krpc_tuple_double_double_double_t,
            );
        }
        6 => {
            return krpc_decode_tuple_double_double_double_double(
                &raw mut stream,
                (*item).target as *mut krpc_tuple_double_double_double_double_t,
            );
        }
        7 => {
            return krpc_decode_tuple_tuple_double_double_double_tuple_double_double_double(
                &raw mut stream,
                (*item).target
                    as *mut krpc_tuple_tuple_double_double_double_tuple_double_double_double_t,
            );
        }
        _ => return KRPC_ERROR_DECODING_FAILED,
    };
}
unsafe extern "C" fn batch_run(
    mut client: *mut KrpcCNanoClient,
    mut batch: *mut ClientBatch,
    mut label: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || batch.is_null() || (*batch).count == 0 as size_t {
        set_error!(
            error,
            error_size,
            b"%s batch is empty\0" as *const u8 as *const ::core::ffi::c_char,
            if !label.is_null() {
                label
            } else {
                b"C-Nano\0" as *const u8 as *const ::core::ffi::c_char
            },
        );
        return false_0 != 0;
    }
    let mut calls: [krpc_schema_ProcedureCall; 32] = [_krpc_schema_ProcedureCall {
        arguments: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
        },
        service_id: 0,
        procedure_id: 0,
    }; 32];
    let mut results: [krpc_result_t; 32] = [krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
        size: 0,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    }; 32];
    let mut failed: [bool; 32] = [false; 32];
    memset(
        &raw mut failed as *mut bool as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<[bool; 32]>() as size_t,
    );
    let mut initialized: size_t = 0 as size_t;
    let mut i: size_t = 0 as size_t;
    while i < (*batch).count {
        results[i as usize] = krpc_result_t {
            message: _krpc_schema_ProcedureResult {
                error: pb_callback_s {
                    funcs: C2RustUnnamed { decode: None },
                    arg: NULL,
                },
                value: pb_callback_s {
                    funcs: C2RustUnnamed { decode: None },
                    arg: NULL,
                },
            },
            size: 0 as size_t,
            buffer: ::core::ptr::null_mut::<uint8_t>(),
        };
        if krpc_init_result(
            (&raw mut results as *mut krpc_result_t).offset(i as isize)
                as *mut krpc_result_t,
        ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
        {
            set_error!(
                error,
                error_size,
                b"Failed to initialize %s batch result\0" as *const u8
                    as *const ::core::ffi::c_char,
                if !label.is_null() {
                    label
                } else {
                    b"C-Nano\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
            let mut j: size_t = 0 as size_t;
            while j < initialized {
                krpc_free_result(
                    (&raw mut results as *mut krpc_result_t).offset(j as isize)
                        as *mut krpc_result_t,
                );
                j = j.wrapping_add(1);
            }
            return false_0 != 0;
        }
        initialized = initialized.wrapping_add(1);
        calls[i as usize] = (*batch).items[i as usize].call.message;
        i = i.wrapping_add(1);
    }
    let mut code: krpc_error_t = krpc_cnano_invoke_batch(
        (*client).connection,
        &raw mut calls as *mut krpc_schema_ProcedureCall,
        &raw mut results as *mut krpc_result_t,
        &raw mut failed as *mut bool,
        (*batch).count,
    );
    (*client).total_rpc_calls = (*client)
        .total_rpc_calls
        .wrapping_add((*batch).count as ::core::ffi::c_uint);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if code as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int {
        let mut i_0: size_t = 0 as size_t;
        while i_0 < (*batch).count {
            if failed[i_0 as usize] {
                if (*batch).items[i_0 as usize].required {
                    set_error!(
                        error,
                        error_size,
                        b"%s batch procedure %u failed\0" as *const u8
                            as *const ::core::ffi::c_char,
                        if !label.is_null() {
                            label
                        } else {
                            b"RPC\0" as *const u8 as *const ::core::ffi::c_char
                        },
                        (*batch).items[i_0 as usize].call.message.procedure_id,
                    );
                    code = KRPC_ERROR_RPC_FAILED;
                    break;
                }
            } else {
                let mut decoded: krpc_error_t = batch_decode_item(
                    (&raw mut (*batch).items as *mut BatchItem).offset(i_0 as isize)
                        as *mut BatchItem,
                    (&raw mut results as *mut krpc_result_t).offset(i_0 as isize)
                        as *mut krpc_result_t,
                );
                if decoded as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                    && (*batch).items[i_0 as usize].required as ::core::ffi::c_int != 0
                {
                    code = decoded;
                    break;
                }
            }
            i_0 = i_0.wrapping_add(1);
        }
    }
    let mut i_1: size_t = 0 as size_t;
    while i_1 < initialized {
        krpc_free_result(
            (&raw mut results as *mut krpc_result_t).offset(i_1 as isize)
                as *mut krpc_result_t,
        );
        i_1 = i_1.wrapping_add(1);
    }
    if code as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        if error.is_null() || *error.offset(0 as ::core::ffi::c_int as isize) == 0 {
            set_error!(
                error,
                error_size,
                b"%s C-Nano batch failed: %s (transport=%s)\0" as *const u8
                    as *const ::core::ffi::c_char,
                if !label.is_null() {
                    label
                } else {
                    b"RPC\0" as *const u8 as *const ::core::ffi::c_char
                },
                krpc_get_error(code),
                krpc_cnano_transport_status_string(
                    krpc_cnano_transport_last_status((*client).connection),
                ),
            );
        }
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn reset_rate_filters(mut client: *mut KrpcCNanoClient) {
    (*client).has_previous_attitude = false_0 != 0;
    (*client).has_previous_course = false_0 != 0;
    lowpass_reset(&raw mut (*client).pitch_rate_filter);
    lowpass_reset(&raw mut (*client).roll_rate_filter);
    lowpass_reset(&raw mut (*client).heading_rate_filter);
    lowpass_reset(&raw mut (*client).angle_of_attack_rate_filter);
    lowpass_reset(&raw mut (*client).course_rate_filter);
}
unsafe extern "C" fn load_planet(
    mut client: *mut KrpcCNanoClient,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut planet: *mut PlanetModel = &raw mut (*client).planet;
    memset(
        planet as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<PlanetModel>() as size_t,
    );
    (*planet).atmosphere_adiabatic_index = 1.4f64;
    let mut name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_Name(
            (*client).connection,
            &raw mut name,
            (*client).body,
        ),
        b"krpc_SpaceCenter_CelestialBody_Name(client->connection, &name, client->body)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if !name.is_null() {
        snprintf(
            &raw mut (*planet).name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            name,
        );
        krpc_free(name as *mut ::core::ffi::c_void);
    } else {
        snprintf(
            &raw mut (*planet).name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
            b"Unknown\0" as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_EquatorialRadius(
            (*client).connection,
            &raw mut (*planet).radius,
            (*client).body,
        ),
        b"krpc_SpaceCenter_CelestialBody_EquatorialRadius(client->connection, &planet->radius, client->body)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_GravitationalParameter(
            (*client).connection,
            &raw mut (*planet).gravitational_parameter,
            (*client).body,
        ),
        b"krpc_SpaceCenter_CelestialBody_GravitationalParameter(client->connection, &planet->gravitational_parameter, client->body)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_RotationalSpeed(
            (*client).connection,
            &raw mut (*planet).rotational_speed,
            (*client).body,
        ),
        b"krpc_SpaceCenter_CelestialBody_RotationalSpeed(client->connection, &planet->rotational_speed, client->body)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_AtmosphereDepth(
            (*client).connection,
            &raw mut (*planet).atmosphere_depth,
            (*client).body,
        ),
        b"krpc_SpaceCenter_CelestialBody_AtmosphereDepth(client->connection, &planet->atmosphere_depth, client->body)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_UT((*client).connection, &raw mut (*planet).epoch_ut),
        b"krpc_SpaceCenter_UT(client->connection, &planet->epoch_ut)\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    let mut north: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut prime: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_SurfacePosition(
            (*client).connection,
            &raw mut north,
            (*client).body,
            90.0f64,
            0.0f64,
            (*client).body_nonrotating,
        ),
        b"krpc_SpaceCenter_CelestialBody_SurfacePosition(client->connection, &north, client->body, 90.0, 0.0, client->body_nonrotating)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_CelestialBody_SurfacePosition(
            (*client).connection,
            &raw mut prime,
            (*client).body,
            0.0f64,
            0.0f64,
            (*client).body_nonrotating,
        ),
        b"krpc_SpaceCenter_CelestialBody_SurfacePosition(client->connection, &prime, client->body, 0.0, 0.0, client->body_nonrotating)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*planet).north_axis = vnorm(
        tuple_to_canonical(north),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    (*planet).prime_meridian_at_epoch = vnorm(
        tuple_to_canonical(prime),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    if (*planet).atmosphere_depth > 0.0f64 {
        let mut batch: ClientBatch = ClientBatch {
            items: [
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
                BatchItem {
                    call: krpc_call_t {
                        message: _krpc_schema_ProcedureCall {
                            arguments: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                            service_id: 0,
                            procedure_id: 0,
                        },
                        numArguments: 0,
                        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                    },
                    arguments: [krpc_argument_t {
                        message: _krpc_schema_Argument {
                            position: 0,
                            value: pb_callback_s {
                                funcs: C2RustUnnamed { decode: None },
                                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                            },
                        },
                    }; 3],
                    object_arguments: [0; 3],
                    uint_arguments: [0; 3],
                    float_arguments: [0.; 3],
                    double_arguments: [0.; 3],
                    bool_arguments: [false; 3],
                    kind: BATCH_NONE,
                    target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                    required: false,
                },
            ],
            count: 0,
        };
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < CNANO_ATMOSPHERE_SAMPLES
            && (*planet).atmosphere_sample_count < LANDER_ATMOSPHERE_SAMPLE_MAX as size_t
        {
            let fresh5 = (*planet).atmosphere_sample_count;
            (*planet).atmosphere_sample_count = (*planet)
                .atmosphere_sample_count
                .wrapping_add(1);
            let mut n: size_t = fresh5;
            let mut altitude: ::core::ffi::c_double = (*planet).atmosphere_depth
                * i as ::core::ffi::c_double
                / (CNANO_ATMOSPHERE_SAMPLES - 1 as ::core::ffi::c_int)
                    as ::core::ffi::c_double;
            (*planet).atmosphere_altitude[n as usize] = altitude;
            if batch.count.wrapping_add(2 as size_t) > KRPC_CNANO_BATCH_MAX as size_t {
                if !batch_run(
                    client,
                    &raw mut batch,
                    b"atmosphere table\0" as *const u8 as *const ::core::ffi::c_char,
                    error,
                    error_size,
                ) {
                    return false_0 != 0;
                }
                memset(
                    &raw mut batch as *mut ::core::ffi::c_void,
                    0 as ::core::ffi::c_int,
                    ::core::mem::size_of::<ClientBatch>() as size_t,
                );
            }
            if !batch_add_object_double(
                &raw mut batch,
                PROC_CELESTIALBODY_PRESSURE_AT as ::core::ffi::c_int as uint32_t,
                (*client).body as krpc_object_t,
                altitude,
                BATCH_DOUBLE,
                (&raw mut (*planet).atmosphere_pressure as *mut ::core::ffi::c_double)
                    .offset(n as isize) as *mut ::core::ffi::c_double
                    as *mut ::core::ffi::c_void,
                true_0 != 0,
            )
                || !batch_add_object_double(
                    &raw mut batch,
                    PROC_CELESTIALBODY_DENSITY_AT as ::core::ffi::c_int as uint32_t,
                    (*client).body as krpc_object_t,
                    altitude,
                    BATCH_DOUBLE,
                    (&raw mut (*planet).atmosphere_density as *mut ::core::ffi::c_double)
                        .offset(n as isize) as *mut ::core::ffi::c_double
                        as *mut ::core::ffi::c_void,
                    true_0 != 0,
                )
            {
                set_error!(
                    error,
                    error_size,
                    b"Could not construct C-Nano atmosphere table batch\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
                return false_0 != 0;
            }
            i += 1;
        }
        if batch.count != 0
            && !batch_run(
                client,
                &raw mut batch,
                b"atmosphere table\0" as *const u8 as *const ::core::ffi::c_char,
                error,
                error_size,
            )
        {
            return false_0 != 0;
        }
        let mut n_0: size_t = 0 as size_t;
        while n_0 < (*planet).atmosphere_sample_count {
            (*planet).atmosphere_pressure[n_0 as usize] = fmax(
                0.0f64,
                finite_or((*planet).atmosphere_pressure[n_0 as usize], 0.0f64),
            );
            (*planet).atmosphere_density[n_0 as usize] = fmax(
                0.0f64,
                finite_or((*planet).atmosphere_density[n_0 as usize], 0.0f64),
            );
            n_0 = n_0.wrapping_add(1);
        }
    }
    (*planet).surface_density = if (*planet).atmosphere_sample_count != 0 {
        (*planet).atmosphere_density[0 as ::core::ffi::c_int as usize]
    } else {
        0.0f64
    };
    return true_0 != 0;
}
unsafe extern "C" fn refresh_burn_geometry(mut client: *mut KrpcCNanoClient) {
    let mut force: Vector3 = v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut moment: Vector3 = v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut selected_thrust: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut i: size_t = 0 as size_t;
    while i < (*client).engine_count {
        if (*client).engines[i as usize].selected {
            selected_thrust
                += (*client).engines[i as usize].available_thrust
                    as ::core::ffi::c_double;
        }
        i = i.wrapping_add(1);
    }
    let mut geometry_ok: bool = selected_thrust
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut part_fallback: bool = false_0 != 0;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*client).engine_count {
        let mut entry: *mut CommandableEngine = (&raw mut (*client).engines
            as *mut CommandableEngine)
            .offset(i_0 as isize) as *mut CommandableEngine;
        if (*entry).selected {
            let mut engine_force: Vector3 = v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            let mut engine_moment: Vector3 = v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            );
            let mut engine_ok: bool = true_0 != 0;
            let mut thrusters: krpc_list_object_t = krpc_list_object_s {
                size: 0 as size_t,
                items: ::core::ptr::null_mut::<krpc_object_t>(),
            };
            if krpc_SpaceCenter_Engine_Thrusters(
                (*client).connection,
                &raw mut thrusters,
                (*entry).object,
            ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                || thrusters.size == 0
            {
                engine_ok = false_0 != 0;
            }
            let mut j: size_t = 0 as size_t;
            while engine_ok as ::core::ffi::c_int != 0 && j < thrusters.size {
                let mut d: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
                    e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    e1: 0.,
                    e2: 0.,
                };
                let mut r: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
                    e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    e1: 0.,
                    e2: 0.,
                };
                let mut thruster: krpc_SpaceCenter_Thruster_t = *thrusters
                    .items
                    .offset(j as isize) as krpc_SpaceCenter_Thruster_t;
                if krpc_SpaceCenter_Thruster_InitialThrustDirection(
                    (*client).connection,
                    &raw mut d,
                    thruster,
                    (*client).vessel_frame,
                ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                    || krpc_SpaceCenter_Thruster_InitialThrustPosition(
                        (*client).connection,
                        &raw mut r,
                        thruster,
                        (*client).vessel_frame,
                    ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                {
                    engine_ok = false_0 != 0;
                    break;
                } else {
                    let mut direction: Vector3 = v3(d.e0, d.e1, d.e2);
                    if vmag(direction).is_finite() as i32 == 0
                        || vmag(direction) <= 1e-8f64
                    {
                        engine_ok = false_0 != 0;
                        break;
                    } else {
                        let mut f: Vector3 = vscale(
                            vnorm(
                                direction,
                                v3(
                                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                                ),
                            ),
                            ((*entry).available_thrust
                                / thrusters.size as ::core::ffi::c_float)
                                as ::core::ffi::c_double,
                        );
                        engine_force = vadd(engine_force, f);
                        engine_moment = vadd(
                            engine_moment,
                            vcross(v3(r.e0, r.e1, r.e2), f),
                        );
                        j = j.wrapping_add(1);
                    }
                }
            }
            krpc_free(thrusters.items as *mut ::core::ffi::c_void);
            thrusters.items = ::core::ptr::null_mut::<krpc_object_t>();
            thrusters.size = 0 as size_t;
            if !engine_ok && (*entry).part != KRPC_NULL {
                let mut d_0: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
                    e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    e1: 0.,
                    e2: 0.,
                };
                let mut r_0: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
                    e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    e1: 0.,
                    e2: 0.,
                };
                if krpc_SpaceCenter_Part_Direction(
                    (*client).connection,
                    &raw mut d_0,
                    (*entry).part,
                    (*client).vessel_frame,
                ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
                    && krpc_SpaceCenter_Part_Position(
                        (*client).connection,
                        &raw mut r_0,
                        (*entry).part,
                        (*client).vessel_frame,
                    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
                {
                    let mut direction_0: Vector3 = v3(d_0.e0, d_0.e1, d_0.e2);
                    if vmag(direction_0).is_finite() as i32 != 0
                        && vmag(direction_0) > 1e-8f64
                    {
                        engine_force = vscale(
                            vnorm(
                                direction_0,
                                v3(
                                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                                ),
                            ),
                            (*entry).available_thrust as ::core::ffi::c_double,
                        );
                        engine_moment = vcross(v3(r_0.e0, r_0.e1, r_0.e2), engine_force);
                        engine_ok = true_0 != 0;
                        part_fallback = true_0 != 0;
                    }
                }
            }
            if !engine_ok {
                geometry_ok = false_0 != 0;
            } else {
                force = vadd(force, engine_force);
                moment = vadd(moment, engine_moment);
            }
        }
        i_0 = i_0.wrapping_add(1);
    }
    (*client).burn_geometry_valid = geometry_ok as ::core::ffi::c_int != 0
        && vmag(force) > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*client).burn_thrust_axis_body = vnorm(
        force,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    (*client).burn_thrust_moment_body = moment;
    (*client).burn_resultant_fraction = if (*client).burn_geometry_valid
        as ::core::ffi::c_int != 0
    {
        vmag(force) / selected_thrust
    } else {
        1.0f64
    };
    (*client).commandable_thrust = if (*client).burn_geometry_valid as ::core::ffi::c_int
        != 0
    {
        vmag(force)
    } else {
        selected_thrust
    };
    fprintf(
        stderr,
        b"Selected burn thrust %.0f N; body axis %.4f/%.4f/%.4f; moment %.0f/%.0f/%.0f Nm; geometry=%s\n\0"
            as *const u8 as *const ::core::ffi::c_char,
        (*client).commandable_thrust,
        (*client).burn_thrust_axis_body.x,
        (*client).burn_thrust_axis_body.y,
        (*client).burn_thrust_axis_body.z,
        moment.x,
        moment.y,
        moment.z,
        if (*client).burn_geometry_valid as ::core::ffi::c_int != 0 {
            if part_fallback as ::core::ffi::c_int != 0 {
                b"part-orientation\0" as *const u8 as *const ::core::ffi::c_char
            } else {
                b"measured\0" as *const u8 as *const ::core::ffi::c_char
            }
        } else {
            b"unavailable\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
}
unsafe extern "C" fn inventory_engines(mut client: *mut KrpcCNanoClient) {
    (*client).engine_count = 0 as size_t;
    (*client).commandable_thrust = 0.0f64;
    (*client).body_has_oxygen = false_0 != 0;
    if krpc_SpaceCenter_CelestialBody_HasAtmosphericOxygen(
        (*client).connection,
        &raw mut (*client).body_has_oxygen,
        (*client).body,
    ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
    {
        (*client).body_has_oxygen = false_0 != 0;
    }
    let mut parts: krpc_SpaceCenter_Parts_t = KRPC_NULL;
    let mut engines: krpc_list_object_t = krpc_list_object_s {
        size: 0 as size_t,
        items: ::core::ptr::null_mut::<krpc_object_t>(),
    };
    let mut e: krpc_error_t = krpc_SpaceCenter_Vessel_Parts(
        (*client).connection,
        &raw mut parts,
        (*client).vessel,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int || parts == KRPC_NULL {
        return;
    }
    e = krpc_SpaceCenter_Parts_Engines((*client).connection, &raw mut engines, parts);
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return;
    }
    let mut i: size_t = 0 as size_t;
    while i < engines.size && (*client).engine_count < CNANO_ENGINE_LIMIT as size_t {
        let mut engine: krpc_SpaceCenter_Engine_t = *engines.items.offset(i as isize)
            as krpc_SpaceCenter_Engine_t;
        let mut locked: bool = true_0 != 0;
        let mut fuel: bool = false_0 != 0;
        let mut active: bool = false_0 != 0;
        let mut available: ::core::ffi::c_float = 0.0f32;
        let mut e_locked: krpc_error_t = krpc_SpaceCenter_Engine_ThrottleLocked(
            (*client).connection,
            &raw mut locked,
            engine,
        );
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        let mut e_fuel: krpc_error_t = krpc_SpaceCenter_Engine_HasFuel(
            (*client).connection,
            &raw mut fuel,
            engine,
        );
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        let mut e_available: krpc_error_t = krpc_SpaceCenter_Engine_AvailableThrust(
            (*client).connection,
            &raw mut available,
            engine,
        );
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        let mut e_active: krpc_error_t = krpc_SpaceCenter_Engine_Active(
            (*client).connection,
            &raw mut active,
            engine,
        );
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        if !(e_locked as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            || e_fuel as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            || e_available as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            || e_active as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            || locked as ::core::ffi::c_int != 0 || !fuel || !(available > 0.0f32))
        {
            let fresh4 = (*client).engine_count;
            (*client).engine_count = (*client).engine_count.wrapping_add(1);
            let mut entry: *mut CommandableEngine = (&raw mut (*client).engines
                as *mut CommandableEngine)
                .offset(fresh4 as isize) as *mut CommandableEngine;
            memset(
                entry as *mut ::core::ffi::c_void,
                0 as ::core::ffi::c_int,
                ::core::mem::size_of::<CommandableEngine>() as size_t,
            );
            let mut part: krpc_SpaceCenter_Part_t = KRPC_NULL;
            let mut name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                ::core::ffi::c_char,
            >();
            let mut title: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                ::core::ffi::c_char,
            >();
            let mut tag: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                ::core::ffi::c_char,
            >();
            if krpc_SpaceCenter_Engine_Part((*client).connection, &raw mut part, engine)
                as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
            {
                krpc_SpaceCenter_Part_Name((*client).connection, &raw mut name, part);
                krpc_SpaceCenter_Part_Title((*client).connection, &raw mut title, part);
                krpc_SpaceCenter_Part_Tag((*client).connection, &raw mut tag, part);
            }
            (*entry).role = engine_role_identify(name, title, tag);
            let mut propellants: krpc_list_string_t = krpc_list_string_s {
                size: 0 as size_t,
                items: ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
            };
            if krpc_SpaceCenter_Engine_PropellantNames(
                (*client).connection,
                &raw mut propellants,
                engine,
            ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
            {
                let mut p: size_t = 0 as size_t;
                while p < propellants.size {
                    (*entry).airbreathing = ((*entry).airbreathing as ::core::ffi::c_int
                        | engine_propellant_is_intake_air(
                            *propellants.items.offset(p as isize),
                        ) as ::core::ffi::c_int) != 0;
                    krpc_free(
                        *propellants.items.offset(p as isize) as *mut ::core::ffi::c_void,
                    );
                    p = p.wrapping_add(1);
                }
                krpc_free(propellants.items as *mut ::core::ffi::c_void);
                propellants.items = ::core::ptr::null_mut::<*mut ::core::ffi::c_char>();
                propellants.size = 0 as size_t;
            }
            fprintf(
                stderr,
                b"Engine inventory: name=%s title=%s role=%d airbreathing=%d thrust=%.0f N\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                if !name.is_null() {
                    name as *const ::core::ffi::c_char
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
                if !title.is_null() {
                    title as *const ::core::ffi::c_char
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
                (*entry).role as ::core::ffi::c_uint,
                if (*entry).airbreathing as ::core::ffi::c_int != 0 {
                    1 as ::core::ffi::c_int
                } else {
                    0 as ::core::ffi::c_int
                },
                available as ::core::ffi::c_double,
            );
            if !name.is_null() {
                krpc_free(name as *mut ::core::ffi::c_void);
            }
            if !title.is_null() {
                krpc_free(title as *mut ::core::ffi::c_void);
            }
            if !tag.is_null() {
                krpc_free(tag as *mut ::core::ffi::c_void);
            }
            (*entry).object = engine;
            (*entry).part = part;
            (*entry).initially_active = active;
            (*entry).available_thrust = available;
            (*client).commandable_thrust += available as ::core::ffi::c_double;
        }
        i = i.wrapping_add(1);
    }
    let mut has_oms: bool = false_0 != 0;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*client).engine_count {
        has_oms = (has_oms as ::core::ffi::c_int
            | ((*client).engines[i_0 as usize].role as ::core::ffi::c_uint
                == ENGINE_ROLE_OMS as ::core::ffi::c_int as ::core::ffi::c_uint)
                as ::core::ffi::c_int) != 0;
        i_0 = i_0.wrapping_add(1);
    }
    (*client).commandable_thrust = 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    let mut i_1: size_t = 0 as size_t;
    while i_1 < (*client).engine_count {
        let mut entry_0: *mut CommandableEngine = (&raw mut (*client).engines
            as *mut CommandableEngine)
            .offset(i_1 as isize) as *mut CommandableEngine;
        (*entry_0).selected = !(*entry_0).airbreathing
            && engine_role_selected(
                (*entry_0).role,
                has_oms as ::core::ffi::c_int != 0
                    || (*client).require_oms as ::core::ffi::c_int != 0,
            ) as ::core::ffi::c_int != 0;
        if (*entry_0).selected {
            (*client).commandable_thrust
                += (*entry_0).available_thrust as ::core::ffi::c_double;
        }
        i_1 = i_1.wrapping_add(1);
    }
    refresh_burn_geometry(client);
    krpc_free(engines.items as *mut ::core::ffi::c_void);
    engines.items = ::core::ptr::null_mut::<krpc_object_t>();
    engines.size = 0 as size_t;
}
unsafe extern "C" fn fin_module_fields(
    mut client: *mut KrpcCNanoClient,
    mut module: krpc_SpaceCenter_Module_t,
    mut out: *mut FinModuleFields,
) -> bool {
    memset(
        out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<FinModuleFields>() as size_t,
    );
    let mut fields: krpc_list_object_t = krpc_list_object_s {
        size: 0 as size_t,
        items: ::core::ptr::null_mut::<krpc_object_t>(),
    };
    let mut e: krpc_error_t = krpc_SpaceCenter_Module_FieldList(
        (*client).connection,
        &raw mut fields,
        module,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return false_0 != 0;
    }
    let mut k: size_t = 0 as size_t;
    while k < fields.size {
        let mut field: krpc_SpaceCenter_PartField_t = *fields.items.offset(k as isize);
        let mut name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
            ::core::ffi::c_char,
        >();
        e = krpc_SpaceCenter_PartField_Name((*client).connection, &raw mut name, field);
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int || name.is_null() {
            if !name.is_null() {
                krpc_free(name as *mut ::core::ffi::c_void);
            }
        } else {
            if strcmp(name, b"deploy\0" as *const u8 as *const ::core::ffi::c_char) == 0
            {
                (*out).deploy = field;
            } else if strcmp(
                name,
                b"deployAngle\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
            {
                (*out).deploy_angle = field;
            } else if strcmp(
                name,
                b"deployInvert\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
            {
                (*out).deploy_invert = field;
            } else if strcmp(
                name,
                b"ignorePitch\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
            {
                (*out).ignore_pitch = field;
            } else if strcmp(
                name,
                b"ignoreRoll\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
            {
                (*out).ignore_roll = field;
            } else if strcmp(
                name,
                b"ignoreYaw\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
            {
                (*out).ignore_yaw = field;
            } else if strcmp(
                name,
                b"ctrlSurfaceRange\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0
            {
                (*out).range = field;
            }
            krpc_free(name as *mut ::core::ffi::c_void);
        }
        k = k.wrapping_add(1);
    }
    krpc_free(fields.items as *mut ::core::ffi::c_void);
    fields.items = ::core::ptr::null_mut::<krpc_object_t>();
    fields.size = 0 as size_t;
    return (*out).deploy != 0 && (*out).deploy_angle != 0 && (*out).deploy_invert != 0
        && (*out).ignore_pitch != 0 && (*out).ignore_roll != 0 && (*out).ignore_yaw != 0
        && (*out).range != 0;
}
unsafe extern "C" fn inventory_fin_modules(mut client: *mut KrpcCNanoClient) {
    (*client).fin_module_count = 0 as size_t;
    (*client).has_applied_speedbrake = false_0 != 0;
    (*client).speedbrake_max_angle_deg = 0.0f64;
    let mut i: size_t = 0 as size_t;
    while i < (*client).surface_count {
        if !((*client).surfaces[i as usize].role as ::core::ffi::c_uint
            != KRPC_SURFACE_ROLE_RUDDER as ::core::ffi::c_int as ::core::ffi::c_uint)
        {
            let mut part: krpc_SpaceCenter_Part_t = KRPC_NULL;
            let mut modules: krpc_list_object_t = krpc_list_object_s {
                size: 0 as size_t,
                items: ::core::ptr::null_mut::<krpc_object_t>(),
            };
            let mut e: krpc_error_t = krpc_SpaceCenter_ControlSurface_Part(
                (*client).connection,
                &raw mut part,
                (*client).surfaces[i as usize].object,
            );
            (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
            (*client).total_wire_requests = (*client)
                .total_wire_requests
                .wrapping_add(1);
            if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                || part == KRPC_NULL)
            {
                e = krpc_SpaceCenter_Part_Modules(
                    (*client).connection,
                    &raw mut modules,
                    part,
                );
                (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
                (*client).total_wire_requests = (*client)
                    .total_wire_requests
                    .wrapping_add(1);
                if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int) {
                    let mut j: size_t = 0 as size_t;
                    while j < modules.size && (*client).fin_module_count < 4 as size_t {
                        let mut module: krpc_SpaceCenter_Module_t = *modules
                            .items
                            .offset(j as isize);
                        let mut name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                            ::core::ffi::c_char,
                        >();
                        e = krpc_SpaceCenter_Module_Name(
                            (*client).connection,
                            &raw mut name,
                            module,
                        );
                        (*client).total_rpc_calls = (*client)
                            .total_rpc_calls
                            .wrapping_add(1);
                        (*client).total_wire_requests = (*client)
                            .total_wire_requests
                            .wrapping_add(1);
                        let mut surface: bool = e as ::core::ffi::c_int
                            == KRPC_OK as ::core::ffi::c_int && !name.is_null()
                            && strcmp(
                                name,
                                b"ModuleControlSurface\0" as *const u8
                                    as *const ::core::ffi::c_char,
                            ) == 0 as ::core::ffi::c_int;
                        if !name.is_null() {
                            krpc_free(name as *mut ::core::ffi::c_void);
                        }
                        let mut fields: FinModuleFields = FinModuleFields {
                            deploy: 0,
                            deploy_angle: 0,
                            deploy_invert: 0,
                            ignore_pitch: 0,
                            ignore_roll: 0,
                            ignore_yaw: 0,
                            range: 0,
                        };
                        if !(!surface
                            || !fin_module_fields(client, module, &raw mut fields))
                        {
                            let mut range_deg: ::core::ffi::c_float = 0.0f32;
                            e = krpc_SpaceCenter_PartField_FloatValue(
                                (*client).connection,
                                &raw mut range_deg,
                                fields.range,
                            );
                            (*client).total_rpc_calls = (*client)
                                .total_rpc_calls
                                .wrapping_add(1);
                            (*client).total_wire_requests = (*client)
                                .total_wire_requests
                                .wrapping_add(1);
                            if e as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
                                && range_deg.is_finite() as i32 != 0 && range_deg > 0.0f32
                            {
                                (*client).speedbrake_max_angle_deg = if (*client)
                                    .speedbrake_max_angle_deg > 0.0f64
                                {
                                    fmin(
                                        (*client).speedbrake_max_angle_deg,
                                        range_deg as ::core::ffi::c_double,
                                    )
                                } else {
                                    range_deg as ::core::ffi::c_double
                                };
                            }
                            let fresh3 = (*client).fin_module_count;
                            (*client).fin_module_count = (*client)
                                .fin_module_count
                                .wrapping_add(1);
                            (*client).fin_modules[fresh3 as usize] = fields;
                        }
                        j = j.wrapping_add(1);
                    }
                    krpc_free(modules.items as *mut ::core::ffi::c_void);
                    modules.items = ::core::ptr::null_mut::<krpc_object_t>();
                    modules.size = 0 as size_t;
                }
            }
        }
        i = i.wrapping_add(1);
    }
    if (*client).fin_module_count != 2 as size_t
        || !((*client).speedbrake_max_angle_deg > 0.0f64)
    {
        (*client).fin_module_count = 0 as size_t;
        return;
    }
    let mut _e: krpc_error_t = krpc_SpaceCenter_PartField_set_BoolValue(
        (*client).connection,
        (*client).fin_modules[0 as ::core::ffi::c_int as usize].deploy_invert,
        0 as ::core::ffi::c_int != 0,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    let mut _e_0: krpc_error_t = krpc_SpaceCenter_PartField_set_BoolValue(
        (*client).connection,
        (*client).fin_modules[1 as ::core::ffi::c_int as usize].deploy_invert,
        1 as ::core::ffi::c_int != 0,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
}
unsafe extern "C" fn inventory_control_surfaces(mut client: *mut KrpcCNanoClient) {
    (*client).surface_count = 0 as size_t;
    (*client).surface_inventory_available = false_0 != 0;
    (*client).surface_state_valid = false_0 != 0;
    (*client).surface_roles_classified = false_0 != 0;
    (*client).has_applied_surface_allocation = false_0 != 0;
    let mut parts: krpc_SpaceCenter_Parts_t = KRPC_NULL;
    let mut surfaces: krpc_list_object_t = krpc_list_object_s {
        size: 0 as size_t,
        items: ::core::ptr::null_mut::<krpc_object_t>(),
    };
    let mut e: krpc_error_t = krpc_SpaceCenter_Vessel_Parts(
        (*client).connection,
        &raw mut parts,
        (*client).vessel,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int || parts == KRPC_NULL {
        return;
    }
    e = krpc_SpaceCenter_Parts_ControlSurfaces(
        (*client).connection,
        &raw mut surfaces,
        parts,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return;
    }
    (*client).surface_inventory_available = true_0 != 0;
    let mut elevons: size_t = 0 as size_t;
    let mut i: size_t = 0 as size_t;
    while i < surfaces.size
        && (*client).surface_count < CNANO_CONTROL_SURFACE_LIMIT as size_t
    {
        let mut surface: krpc_SpaceCenter_ControlSurface_t = *surfaces
            .items
            .offset(i as isize) as krpc_SpaceCenter_ControlSurface_t;
        let mut part: krpc_SpaceCenter_Part_t = KRPC_NULL;
        let mut position: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
            e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            e1: 0.,
            e2: 0.,
        };
        e = krpc_SpaceCenter_ControlSurface_Part(
            (*client).connection,
            &raw mut part,
            surface,
        );
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
            || part == KRPC_NULL)
        {
            e = krpc_SpaceCenter_Part_Position(
                (*client).connection,
                &raw mut position,
                part,
                (*client).vessel_frame,
            );
            (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
            (*client).total_wire_requests = (*client)
                .total_wire_requests
                .wrapping_add(1);
            if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                || position.e0.is_finite() as i32 == 0
                || position.e2.is_finite() as i32 == 0)
            {
                let fresh1 = (*client).surface_count;
                (*client).surface_count = (*client).surface_count.wrapping_add(1);
                let mut entry: *mut CommandableSurface = (&raw mut (*client).surfaces
                    as *mut CommandableSurface)
                    .offset(fresh1 as isize) as *mut CommandableSurface;
                memset(
                    entry as *mut ::core::ffi::c_void,
                    0 as ::core::ffi::c_int,
                    ::core::mem::size_of::<CommandableSurface>() as size_t,
                );
                (*entry).object = surface;
                (*entry).lateral_m = position.e0;
                let mut part_name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                    ::core::ffi::c_char,
                >();
                if krpc_SpaceCenter_Part_Name(
                    (*client).connection,
                    &raw mut part_name,
                    part,
                ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
                    && !part_name.is_null()
                {
                    snprintf(
                        &raw mut (*entry).part_name as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
                        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                        part_name,
                    );
                }
                if !part_name.is_null() {
                    krpc_free(part_name as *mut ::core::ffi::c_void);
                }
                if (*entry).part_name[0 as ::core::ffi::c_int as usize] == 0 {
                    snprintf(
                        &raw mut (*entry).part_name as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
                        b"control-surface-%zu\0" as *const u8
                            as *const ::core::ffi::c_char,
                        (*client).surface_count.wrapping_sub(1 as size_t),
                    );
                }
                if fabs(position.e0) < 0.5f64 {
                    (*entry).role = (if position.e2 < 0.0f64 {
                        KRPC_SURFACE_ROLE_RUDDER as ::core::ffi::c_int
                    } else {
                        KRPC_SURFACE_ROLE_BODY_FLAP as ::core::ffi::c_int
                    }) as KrpcSurfaceRole;
                } else {
                    (*entry).role = KRPC_SURFACE_ROLE_UNKNOWN;
                    elevons = elevons.wrapping_add(1);
                }
            }
        }
        i = i.wrapping_add(1);
    }
    krpc_free(surfaces.items as *mut ::core::ffi::c_void);
    surfaces.items = ::core::ptr::null_mut::<krpc_object_t>();
    surfaces.size = 0 as size_t;
    if elevons < 2 as size_t || elevons.wrapping_rem(2 as size_t) != 0 as size_t {
        return;
    }
    let mut spans: [::core::ffi::c_double; 16] = [0.; 16];
    let mut n: size_t = 0 as size_t;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*client).surface_count {
        if (*client).surfaces[i_0 as usize].role as ::core::ffi::c_uint
            == KRPC_SURFACE_ROLE_UNKNOWN as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            let fresh2 = n;
            n = n.wrapping_add(1);
            spans[fresh2 as usize] = fabs((*client).surfaces[i_0 as usize].lateral_m);
        }
        i_0 = i_0.wrapping_add(1);
    }
    let mut i_1: size_t = 1 as size_t;
    while i_1 < n {
        let mut j: size_t = i_1;
        while j > 0 as size_t
            && spans[j.wrapping_sub(1 as size_t) as usize] > spans[j as usize]
        {
            let mut t: ::core::ffi::c_double = spans[j as usize];
            spans[j as usize] = spans[j.wrapping_sub(1 as size_t) as usize];
            spans[j.wrapping_sub(1 as size_t) as usize] = t;
            j = j.wrapping_sub(1);
        }
        i_1 = i_1.wrapping_add(1);
    }
    let mut split: ::core::ffi::c_double = 0.5f64
        * (spans[n.wrapping_div(2 as size_t).wrapping_sub(1 as size_t) as usize]
            + spans[n.wrapping_div(2 as size_t) as usize]);
    if !(spans[n.wrapping_div(2 as size_t) as usize]
        - spans[n.wrapping_div(2 as size_t).wrapping_sub(1 as size_t) as usize]
        > 0.25f64)
    {
        return;
    }
    let mut i_2: size_t = 0 as size_t;
    while i_2 < (*client).surface_count {
        if (*client).surfaces[i_2 as usize].role as ::core::ffi::c_uint
            == KRPC_SURFACE_ROLE_UNKNOWN as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            (*client).surfaces[i_2 as usize].role = (if fabs(
                (*client).surfaces[i_2 as usize].lateral_m,
            ) < split
            {
                KRPC_SURFACE_ROLE_ELEVATOR as ::core::ffi::c_int
            } else {
                KRPC_SURFACE_ROLE_AILERON as ::core::ffi::c_int
            }) as KrpcSurfaceRole;
        }
        i_2 = i_2.wrapping_add(1);
    }
    (*client).surface_roles_classified = true_0 != 0;
    inventory_fin_modules(client);
}
unsafe extern "C" fn inventory_main_wheels(mut client: *mut KrpcCNanoClient) {
    (*client).main_wheel_grounded_field_count = 0 as size_t;
    let mut parts: krpc_SpaceCenter_Parts_t = KRPC_NULL;
    let mut all: krpc_list_object_t = krpc_list_object_s {
        size: 0 as size_t,
        items: ::core::ptr::null_mut::<krpc_object_t>(),
    };
    let mut e: krpc_error_t = krpc_SpaceCenter_Vessel_Parts(
        (*client).connection,
        &raw mut parts,
        (*client).vessel,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int || parts == KRPC_NULL {
        return;
    }
    e = krpc_SpaceCenter_Parts_All((*client).connection, &raw mut all, parts);
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return;
    }
    let mut i: size_t = 0 as size_t;
    while i < all.size
        && (*client).main_wheel_grounded_field_count < CNANO_MAIN_WHEEL_LIMIT as size_t
    {
        let mut part: krpc_SpaceCenter_Part_t = *all.items.offset(i as isize);
        let mut title: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
            ::core::ffi::c_char,
        >();
        e = krpc_SpaceCenter_Part_Title((*client).connection, &raw mut title, part);
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
        let mut is_main: bool = e as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
            && !title.is_null()
            && strcmp(
                title,
                b"OV-100 Main Gear\0" as *const u8 as *const ::core::ffi::c_char,
            ) == 0 as ::core::ffi::c_int;
        if !title.is_null() {
            krpc_free(title as *mut ::core::ffi::c_void);
        }
        if is_main {
            let mut modules: krpc_list_object_t = krpc_list_object_s {
                size: 0 as size_t,
                items: ::core::ptr::null_mut::<krpc_object_t>(),
            };
            e = krpc_SpaceCenter_Part_Modules(
                (*client).connection,
                &raw mut modules,
                part,
            );
            (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
            (*client).total_wire_requests = (*client)
                .total_wire_requests
                .wrapping_add(1);
            if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int) {
                let mut j: size_t = 0 as size_t;
                while j < modules.size
                    && (*client).main_wheel_grounded_field_count
                        < CNANO_MAIN_WHEEL_LIMIT as size_t
                {
                    let mut module: krpc_SpaceCenter_Module_t = *modules
                        .items
                        .offset(j as isize);
                    let mut name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                        ::core::ffi::c_char,
                    >();
                    let mut ne: krpc_error_t = krpc_SpaceCenter_Module_Name(
                        (*client).connection,
                        &raw mut name,
                        module,
                    );
                    (*client).total_rpc_calls = (*client)
                        .total_rpc_calls
                        .wrapping_add(1);
                    (*client).total_wire_requests = (*client)
                        .total_wire_requests
                        .wrapping_add(1);
                    let mut wheel_base: bool = ne as ::core::ffi::c_int
                        == KRPC_OK as ::core::ffi::c_int && !name.is_null()
                        && strcmp(
                            name,
                            b"ModuleWheelBase\0" as *const u8
                                as *const ::core::ffi::c_char,
                        ) == 0 as ::core::ffi::c_int;
                    if !name.is_null() {
                        krpc_free(name as *mut ::core::ffi::c_void);
                    }
                    if wheel_base {
                        let mut fields: krpc_list_object_t = krpc_list_object_s {
                            size: 0 as size_t,
                            items: ::core::ptr::null_mut::<krpc_object_t>(),
                        };
                        let mut fe: krpc_error_t = krpc_SpaceCenter_Module_FieldList(
                            (*client).connection,
                            &raw mut fields,
                            module,
                        );
                        (*client).total_rpc_calls = (*client)
                            .total_rpc_calls
                            .wrapping_add(1);
                        (*client).total_wire_requests = (*client)
                            .total_wire_requests
                            .wrapping_add(1);
                        if !(fe as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int) {
                            let mut k: size_t = 0 as size_t;
                            while k < fields.size
                                && (*client).main_wheel_grounded_field_count
                                    < CNANO_MAIN_WHEEL_LIMIT as size_t
                            {
                                let mut field: krpc_SpaceCenter_PartField_t = *fields
                                    .items
                                    .offset(k as isize);
                                let mut field_name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
                                    ::core::ffi::c_char,
                                >();
                                let mut fne: krpc_error_t = krpc_SpaceCenter_PartField_Name(
                                    (*client).connection,
                                    &raw mut field_name,
                                    field,
                                );
                                (*client).total_rpc_calls = (*client)
                                    .total_rpc_calls
                                    .wrapping_add(1);
                                (*client).total_wire_requests = (*client)
                                    .total_wire_requests
                                    .wrapping_add(1);
                                if fne as ::core::ffi::c_int
                                    == KRPC_OK as ::core::ffi::c_int && !field_name.is_null()
                                    && strcmp(
                                        field_name,
                                        b"isGrounded\0" as *const u8 as *const ::core::ffi::c_char,
                                    ) == 0 as ::core::ffi::c_int
                                {
                                    let fresh0 = (*client).main_wheel_grounded_field_count;
                                    (*client).main_wheel_grounded_field_count = (*client)
                                        .main_wheel_grounded_field_count
                                        .wrapping_add(1);
                                    (*client).main_wheel_grounded_fields[fresh0 as usize] = field;
                                }
                                if !field_name.is_null() {
                                    krpc_free(field_name as *mut ::core::ffi::c_void);
                                }
                                k = k.wrapping_add(1);
                            }
                            krpc_free(fields.items as *mut ::core::ffi::c_void);
                            fields.items = ::core::ptr::null_mut::<krpc_object_t>();
                            fields.size = 0 as size_t;
                        }
                    }
                    j = j.wrapping_add(1);
                }
                krpc_free(modules.items as *mut ::core::ffi::c_void);
                modules.items = ::core::ptr::null_mut::<krpc_object_t>();
                modules.size = 0 as size_t;
            }
        }
        i = i.wrapping_add(1);
    }
    krpc_free(all.items as *mut ::core::ffi::c_void);
    all.items = ::core::ptr::null_mut::<krpc_object_t>();
    all.size = 0 as size_t;
}
unsafe extern "C" fn allocate_client(
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> *mut KrpcCNanoClient {
    let mut client: *mut KrpcCNanoClient = calloc(
        1 as size_t,
        ::core::mem::size_of::<KrpcCNanoClient>() as size_t,
    ) as *mut KrpcCNanoClient;
    if client.is_null() {
        set_error!(
            error,
            error_size,
            b"Out of memory creating C-Nano client\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    (*client).last_radar_altitude = ::core::f32::NAN as ::core::ffi::c_double;
    (*client).last_medium_ut = -DBL_MAX;
    (*client).last_slow_ut = -DBL_MAX;
    (*client).pitch_rate_filter.tc = 0.32f64;
    (*client).roll_rate_filter.tc = 0.28f64;
    (*client).heading_rate_filter.tc = 0.38f64;
    (*client).angle_of_attack_rate_filter.tc = 0.32f64;
    (*client).course_rate_filter.tc = 0.30f64;
    return client;
}
unsafe extern "C" fn discard_setup_client(mut client: *mut KrpcCNanoClient) {
    if client.is_null() {
        return;
    }
    if !(*client).connection.is_null()
        && krpc_cnano_transport_is_open((*client).connection) as ::core::ffi::c_int != 0
    {
        krpc_close((*client).connection);
    }
    (*client).connected = false_0 != 0;
    free(client as *mut ::core::ffi::c_void);
}
unsafe extern "C" fn finish_client_open(
    mut client: *mut KrpcCNanoClient,
    mut configuration: *const LandingConfiguration,
    mut transport_config: *const KrpcCNanoTransportConfig,
    mut transport_name: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> *mut KrpcCNanoClient {
    if client.is_null() || configuration.is_null() || transport_config.is_null() {
        set_error!(
            error,
            error_size,
            b"C-Nano open arguments are incomplete\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    let mut open_error: krpc_error_t = krpc_open(
        &raw mut (*client).connection,
        transport_config as *const krpc_connection_config_t,
    );
    if open_error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        let mut transport_status: KrpcCNanoTransportStatus = (if !(*client)
            .connection
            .is_null()
        {
            krpc_cnano_transport_last_status((*client).connection) as ::core::ffi::c_uint
        } else {
            KRPC_CNANO_TRANSPORT_INVALID as ::core::ffi::c_int as ::core::ffi::c_uint
        }) as KrpcCNanoTransportStatus;
        set_error!(
            error,
            error_size,
            b"krpc_open failed: %s (transport=%s)\0" as *const u8
                as *const ::core::ffi::c_char,
            krpc_get_error(open_error),
            krpc_cnano_transport_status_string(transport_status),
        );
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    if rpc_failed(
        client,
        krpc_connect(
            (*client).connection,
            &raw const (*configuration).connection.client_name
                as *const ::core::ffi::c_char,
        ),
        b"krpc_connect\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    (*client).connected = true_0 != 0;
    (*client).require_oms = strcmp(
        &raw const (*configuration).vehicle.model_id as *const ::core::ffi::c_char,
        b"STS\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0
        || strcmp(
            &raw const (*configuration).vehicle.model_id as *const ::core::ffi::c_char,
            b"STS-N\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0;
    snprintf(
        &raw mut (*client).transport_name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if !transport_name.is_null() && *transport_name as ::core::ffi::c_int != 0 {
            transport_name
        } else {
            b"custom\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    if rpc_failed(
        client,
        krpc_SpaceCenter_ActiveVessel((*client).connection, &raw mut (*client).vessel),
        b"SpaceCenter.ActiveVessel\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) as ::core::ffi::c_int != 0 || (*client).vessel == KRPC_NULL
    {
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_Vessel_Control(
            (*client).connection,
            &raw mut (*client).control,
            (*client).vessel,
        ),
        b"Vessel.Control\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_AutoPilot(
                (*client).connection,
                &raw mut (*client).autopilot,
                (*client).vessel,
            ),
            b"Vessel.AutoPilot\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_Orbit(
                (*client).connection,
                &raw mut (*client).orbit,
                (*client).vessel,
            ),
            b"Vessel.Orbit\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Orbit_Body(
                (*client).connection,
                &raw mut (*client).body,
                (*client).orbit,
            ),
            b"Orbit.Body\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_CelestialBody_NonRotatingReferenceFrame(
                (*client).connection,
                &raw mut (*client).body_nonrotating,
                (*client).body,
            ),
            b"Body.NonRotatingReferenceFrame\0" as *const u8
                as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_ReferenceFrame(
                (*client).connection,
                &raw mut (*client).vessel_frame,
                (*client).vessel,
            ),
            b"Vessel.ReferenceFrame\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_SurfaceReferenceFrame(
                (*client).connection,
                &raw mut (*client).surface_frame,
                (*client).vessel,
            ),
            b"Vessel.SurfaceReferenceFrame\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_Flight(
                (*client).connection,
                &raw mut (*client).flight_surface,
                (*client).vessel,
                (*client).surface_frame,
            ),
            b"Vessel.Flight(surface)\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_Flight(
                (*client).connection,
                &raw mut (*client).flight_body,
                (*client).vessel,
                (*client).body_nonrotating,
            ),
            b"Vessel.Flight(body-non-rotating)\0" as *const u8
                as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
        || rpc_failed(
            client,
            krpc_SpaceCenter_Vessel_Flight(
                (*client).connection,
                &raw mut (*client).flight_vessel,
                (*client).vessel,
                (*client).vessel_frame,
            ),
            b"Vessel.Flight(vessel)\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) as ::core::ffi::c_int != 0
    {
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    let mut vessel_name: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    if !rpc_failed(
        client,
        krpc_SpaceCenter_Vessel_Name(
            (*client).connection,
            &raw mut vessel_name,
            (*client).vessel,
        ),
        b"Vessel.Name\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) && !vessel_name.is_null()
    {
        snprintf(
            &raw mut (*client).vessel_name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            vessel_name,
        );
        krpc_free(vessel_name as *mut ::core::ffi::c_void);
    } else if (*client).vessel_name[0 as ::core::ffi::c_int as usize] == 0 {
        snprintf(
            &raw mut (*client).vessel_name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"Active Vessel\0" as *const u8 as *const ::core::ffi::c_char,
        );
    }
    let mut parts: krpc_SpaceCenter_Parts_t = KRPC_NULL;
    let mut controlling: krpc_SpaceCenter_Part_t = KRPC_NULL;
    let mut root: krpc_SpaceCenter_Part_t = KRPC_NULL;
    let mut controlling_title: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    let mut root_title: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    if krpc_SpaceCenter_Vessel_Parts(
        (*client).connection,
        &raw mut parts,
        (*client).vessel,
    ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
    {
        if krpc_SpaceCenter_Parts_Controlling(
            (*client).connection,
            &raw mut controlling,
            parts,
        ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
            && controlling != KRPC_NULL
        {
            krpc_SpaceCenter_Part_Title(
                (*client).connection,
                &raw mut controlling_title,
                controlling,
            );
        }
        if krpc_SpaceCenter_Parts_Root((*client).connection, &raw mut root, parts)
            as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int && root != KRPC_NULL
        {
            krpc_SpaceCenter_Part_Title((*client).connection, &raw mut root_title, root);
        }
    }
    fprintf(
        stderr,
        b"Vessel '%s': controlling part '%s', root part '%s'.\n\0" as *const u8
            as *const ::core::ffi::c_char,
        &raw mut (*client).vessel_name as *mut ::core::ffi::c_char,
        if !controlling_title.is_null() {
            controlling_title as *const ::core::ffi::c_char
        } else {
            b"?\0" as *const u8 as *const ::core::ffi::c_char
        },
        if !root_title.is_null() {
            root_title as *const ::core::ffi::c_char
        } else {
            b"?\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    if !controlling_title.is_null() {
        krpc_free(controlling_title as *mut ::core::ffi::c_void);
    }
    if !root_title.is_null() {
        krpc_free(root_title as *mut ::core::ffi::c_void);
    }
    if !load_planet(client, error, error_size) {
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_AutoPilot_Reset((*client).connection, (*client).autopilot),
        b"AutoPilot.Reset\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        discard_setup_client(client);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    (*client).cached_autopilot_engaged = false_0 != 0;
    (*client).has_autopilot_engaged = true_0 != 0;
    let mut met: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut now_ut: ::core::ffi::c_double = (*client).planet.epoch_ut;
    let mut met_error: krpc_error_t = krpc_SpaceCenter_Vessel_MET(
        (*client).connection,
        &raw mut met,
        (*client).vessel,
    );
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    if met_error as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
        && met.is_finite() as i32 != 0 && met >= 0.0f64 && now_ut.is_finite() as i32 != 0
    {
        snprintf(
            &raw mut (*client).flight_key as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"launch-ut:%.3f\0" as *const u8 as *const ::core::ffi::c_char,
            now_ut - met,
        );
    }
    inventory_engines(client);
    inventory_control_surfaces(client);
    inventory_main_wheels(client);
    reset_rate_filters(client);
    return client;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_open_transport(
    mut configuration: *const LandingConfiguration,
    mut transport_config: *const KrpcCNanoTransportConfig,
    mut transport_name: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> *mut KrpcCNanoClient {
    if configuration.is_null() || transport_config.is_null() {
        set_error!(
            error,
            error_size,
            b"C-Nano custom transport configuration is missing\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    let mut client: *mut KrpcCNanoClient = allocate_client(error, error_size);
    if client.is_null() {
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    return finish_client_open(
        client,
        configuration,
        transport_config,
        transport_name,
        error,
        error_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_open(
    mut configuration: *const LandingConfiguration,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> *mut KrpcCNanoClient {
    if configuration.is_null() {
        set_error!(
            error,
            error_size,
            b"C-Nano configuration is missing\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    let mut client: *mut KrpcCNanoClient = allocate_client(error, error_size);
    if client.is_null() {
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    let mut rpc_host: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_RPC_HOST\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut rpc_port_text: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_RPC_PORT\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !rpc_host.is_null() && *rpc_host as ::core::ffi::c_int != 0
        || !rpc_port_text.is_null() && *rpc_port_text as ::core::ffi::c_int != 0
        || (*configuration).connection.rpc_host[0 as ::core::ffi::c_int as usize]
            as ::core::ffi::c_int != 0
    {
        if rpc_host.is_null() || *rpc_host == 0 {
            rpc_host = if (*configuration)
                .connection
                .rpc_host[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int != 0
            {
                &raw const (*configuration).connection.rpc_host
                    as *const ::core::ffi::c_char
            } else {
                b"127.0.0.1\0" as *const u8 as *const ::core::ffi::c_char
            };
        }
        let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
            ::core::ffi::c_char,
        >();
        let mut rpc_port: ::core::ffi::c_long = if !rpc_port_text.is_null()
            && *rpc_port_text as ::core::ffi::c_int != 0
        {
            strtol(rpc_port_text, &raw mut end, 10 as ::core::ffi::c_int)
        } else {
            (if (*configuration).connection.rpc_port > 0 as ::core::ffi::c_int {
                (*configuration).connection.rpc_port
            } else {
                50000 as ::core::ffi::c_int
            }) as ::core::ffi::c_long
        };
        if rpc_port < 1 as ::core::ffi::c_long || rpc_port > 65535 as ::core::ffi::c_long
            || !rpc_port_text.is_null() && *rpc_port_text as ::core::ffi::c_int != 0
                && (end.is_null() || *end as ::core::ffi::c_int != 0)
        {
            set_error!(
                error,
                error_size,
                b"Invalid kRPC RPC port: %s\0" as *const u8
                    as *const ::core::ffi::c_char,
                if !rpc_port_text.is_null() {
                    rpc_port_text
                } else {
                    b"\0" as *const u8 as *const ::core::ffi::c_char
                },
            );
            free(client as *mut ::core::ffi::c_void);
            return ::core::ptr::null_mut::<KrpcCNanoClient>();
        }
        krpc_cnano_posix_tcp_init(
            &raw mut (*client).tcp,
            rpc_host,
            rpc_port as ::core::ffi::c_int,
            (*configuration).connection.timeout_ms,
        );
        let mut config: KrpcCNanoTransportConfig = krpc_cnano_transport_config(
            &raw mut (*client).transport,
            krpc_cnano_posix_tcp_ops(),
            &raw mut (*client).tcp as *mut ::core::ffi::c_void,
            (*configuration).connection.timeout_ms,
        );
        let mut name: [::core::ffi::c_char; 192] = [0; 192];
        snprintf(
            &raw mut name as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
            b"tcp:%s:%ld\0" as *const u8 as *const ::core::ffi::c_char,
            rpc_host,
            rpc_port,
        );
        return finish_client_open(
            client,
            configuration,
            &raw mut config,
            &raw mut name as *mut ::core::ffi::c_char,
            error,
            error_size,
        );
    }
    let mut path: *const ::core::ffi::c_char = &raw const (*configuration)
        .connection
        .serial_port as *const ::core::ffi::c_char;
    let mut override_0: *const ::core::ffi::c_char = getenv(
        b"KSP_LANDER_SERIAL_PORT\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if !override_0.is_null() && *override_0 as ::core::ffi::c_int != 0 {
        path = override_0;
    }
    if path.is_null() || *path == 0 {
        set_error!(
            error,
            error_size,
            b"No kRPC endpoint is configured\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        free(client as *mut ::core::ffi::c_void);
        return ::core::ptr::null_mut::<KrpcCNanoClient>();
    }
    krpc_cnano_posix_serial_init(
        &raw mut (*client).serial,
        path,
        (*configuration).connection.timeout_ms,
        (*configuration).connection.baud_rate,
        true_0 != 0,
    );
    let mut config_0: KrpcCNanoTransportConfig = krpc_cnano_transport_config(
        &raw mut (*client).transport,
        krpc_cnano_posix_serial_ops(),
        &raw mut (*client).serial as *mut ::core::ffi::c_void,
        (*configuration).connection.timeout_ms,
    );
    let mut name_0: [::core::ffi::c_char; 192] = [0; 192];
    snprintf(
        &raw mut name_0 as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 192]>() as size_t,
        b"serial:%s\0" as *const u8 as *const ::core::ffi::c_char,
        path,
    );
    return finish_client_open(
        client,
        configuration,
        &raw mut config_0,
        &raw mut name_0 as *mut ::core::ffi::c_char,
        error,
        error_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_planet(
    mut client: *const KrpcCNanoClient,
) -> *const PlanetModel {
    return if !client.is_null() {
        &raw const (*client).planet
    } else {
        ::core::ptr::null::<PlanetModel>()
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_vessel(
    mut client: *const KrpcCNanoClient,
) -> *const ::core::ffi::c_char {
    return if !client.is_null() {
        &raw const (*client).vessel_name as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_transport_name(
    mut client: *const KrpcCNanoClient,
) -> *const ::core::ffi::c_char {
    return if !client.is_null() {
        &raw const (*client).transport_name as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_library_version(
    mut client: *const KrpcCNanoClient,
) -> *const ::core::ffi::c_char {
    return b"krpc-cnano-0.6.0\0" as *const u8 as *const ::core::ffi::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_flight_key(
    mut client: *const KrpcCNanoClient,
) -> *const ::core::ffi::c_char {
    return if !client.is_null() {
        &raw const (*client).flight_key as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
unsafe extern "C" fn situation_name(
    mut situation: krpc_SpaceCenter_VesselSituation_t,
) -> *const ::core::ffi::c_char {
    match situation as ::core::ffi::c_uint {
        0 => return b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        1 => return b"orbiting\0" as *const u8 as *const ::core::ffi::c_char,
        2 => return b"sub-orbital\0" as *const u8 as *const ::core::ffi::c_char,
        3 => return b"escaping\0" as *const u8 as *const ::core::ffi::c_char,
        4 => return b"flying\0" as *const u8 as *const ::core::ffi::c_char,
        5 => return b"landed\0" as *const u8 as *const ::core::ffi::c_char,
        6 => return b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        7 => return b"docked\0" as *const u8 as *const ::core::ffi::c_char,
        _ => return b"unknown\0" as *const u8 as *const ::core::ffi::c_char,
    };
}
unsafe extern "C" fn refresh_medium(
    mut client: *mut KrpcCNanoClient,
    mut airbrake_group: ::core::ffi::c_uint,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut speed_of_sound: ::core::ffi::c_float = (*client).cached_speed_of_sound
        as ::core::ffi::c_float;
    let mut static_pressure: ::core::ffi::c_float = (*client).cached_static_pressure
        as ::core::ffi::c_float;
    let mut g_force: ::core::ffi::c_float = (*client).cached_g_force
        as ::core::ffi::c_float;
    let mut dry_mass: ::core::ffi::c_float = (*client).cached_dry_mass
        as ::core::ffi::c_float;
    let mut available_thrust: ::core::ffi::c_float = (*client).cached_available_thrust
        as ::core::ffi::c_float;
    let mut current_thrust: ::core::ffi::c_float = (*client).cached_current_thrust
        as ::core::ffi::c_float;
    let mut torque: krpc_tuple_tuple_double_double_double_tuple_double_double_double_t = krpc_tuple_tuple_double_double_double_tuple_double_double_double_s {
        e0: krpc_tuple_double_double_double_s {
            e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            e1: 0.,
            e2: 0.,
        },
        e1: krpc_tuple_double_double_double_s {
            e0: 0.,
            e1: 0.,
            e2: 0.,
        },
    };
    let mut inertia: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut lift: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut drag: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut com: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut com_root: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut gear: bool = (*client).cached_gear;
    let mut brakes: bool = (*client).cached_brakes;
    let mut airbrakes: bool = (*client).cached_airbrakes;
    let mut rcs: bool = (*client).cached_rcs;
    let mut sas: bool = (*client).cached_sas;
    let mut autopilot: bool = (*client).cached_autopilot_engaged;
    let mut speed_mode: krpc_enum_t = KRPC_SPACECENTER_SPEEDMODE_SURFACE
        as ::core::ffi::c_int;
    let mut situation: krpc_enum_t = KRPC_SPACECENTER_VESSELSITUATION_FLYING
        as ::core::ffi::c_int;
    if airbrake_group < 1 as ::core::ffi::c_uint {
        airbrake_group = 1 as ::core::ffi::c_uint;
    }
    if airbrake_group > 10 as ::core::ffi::c_uint {
        airbrake_group = 10 as ::core::ffi::c_uint;
    }
    let mut batch: ClientBatch = ClientBatch {
        items: [
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
        ],
        count: 0,
    };
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_SPEED_OF_SOUND as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut speed_of_sound as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_STATIC_PRESSURE as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut static_pressure as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_G_FORCE as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut g_force as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_VESSEL_DRY_MASS as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_FLOAT,
        &raw mut dry_mass as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_VESSEL_AVAILABLE_THRUST as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_FLOAT,
        &raw mut available_thrust as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_VESSEL_THRUST as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_FLOAT,
        &raw mut current_thrust as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_VESSEL_AVAILABLE_TORQUE as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_TORQUE,
        &raw mut torque as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_VESSEL_MOMENT_OF_INERTIA as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut inertia as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_LIFT as ::core::ffi::c_int as uint32_t,
        (*client).flight_body as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut lift as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_DRAG as ::core::ffi::c_int as uint32_t,
        (*client).flight_body as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut drag as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_CENTER_OF_MASS as ::core::ffi::c_int as uint32_t,
        (*client).flight_body as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut com as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_FLIGHT_CENTER_OF_MASS as ::core::ffi::c_int as uint32_t,
        (*client).flight_vessel as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut com_root as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_CONTROL_GEAR as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_BOOL,
        &raw mut gear as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_CONTROL_BRAKES as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_BOOL,
        &raw mut brakes as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object_uint(
        &raw mut batch,
        PROC_CONTROL_GET_ACTION_GROUP as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        airbrake_group as uint32_t,
        BATCH_BOOL,
        &raw mut airbrakes as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_CONTROL_RCS as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_BOOL,
        &raw mut rcs as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_CONTROL_SAS as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_BOOL,
        &raw mut sas as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_AUTOPILOT_ENGAGED as ::core::ffi::c_int as uint32_t,
        (*client).autopilot as krpc_object_t,
        BATCH_BOOL,
        &raw mut autopilot as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_CONTROL_SPEED_MODE as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_ENUM,
        &raw mut speed_mode as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut batch,
        PROC_VESSEL_SITUATION as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_ENUM,
        &raw mut situation as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct medium C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_run(
        client,
        &raw mut batch,
        b"medium telemetry\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*client).cached_speed_of_sound = speed_of_sound as ::core::ffi::c_double;
    (*client).cached_static_pressure = static_pressure as ::core::ffi::c_double;
    (*client).cached_g_force = g_force as ::core::ffi::c_double;
    (*client).cached_dry_mass = dry_mass as ::core::ffi::c_double;
    (*client).cached_available_thrust = available_thrust as ::core::ffi::c_double;
    (*client).cached_current_thrust = current_thrust as ::core::ffi::c_double;
    (*client).cached_torque[0 as ::core::ffi::c_int as usize] = fmax(
        fabs(torque.e0.e0),
        fabs(torque.e1.e0),
    );
    (*client).cached_torque[1 as ::core::ffi::c_int as usize] = fmax(
        fabs(torque.e0.e1),
        fabs(torque.e1.e1),
    );
    (*client).cached_torque[2 as ::core::ffi::c_int as usize] = fmax(
        fabs(torque.e0.e2),
        fabs(torque.e1.e2),
    );
    (*client).cached_inertia[0 as ::core::ffi::c_int as usize] = fabs(inertia.e0);
    (*client).cached_inertia[1 as ::core::ffi::c_int as usize] = fabs(inertia.e1);
    (*client).cached_inertia[2 as ::core::ffi::c_int as usize] = fabs(inertia.e2);
    (*client).cached_lift_vector = tuple_to_canonical(lift);
    (*client).cached_drag_vector = tuple_to_canonical(drag);
    (*client).cached_force_vectors_valid = true_0 != 0;
    (*client).cached_center_of_mass = tuple_to_canonical(com);
    (*client).cached_center_of_mass_valid = true_0 != 0;
    (*client).cached_center_of_mass_root = tuple_to_canonical(com_root);
    (*client).cached_center_of_mass_root_valid = true_0 != 0;
    (*client).cached_gear = gear;
    (*client).has_gear = true_0 != 0;
    (*client).cached_brakes = brakes;
    (*client).has_brakes = true_0 != 0;
    (*client).cached_airbrakes = airbrakes;
    (*client).has_airbrakes = true_0 != 0;
    (*client).cached_rcs = rcs;
    (*client).has_rcs = true_0 != 0;
    (*client).cached_sas = sas;
    (*client).has_sas = true_0 != 0;
    (*client).cached_autopilot_engaged = autopilot;
    (*client).has_autopilot_engaged = true_0 != 0;
    (*client).cached_speed_mode = (if speed_mode
        == KRPC_SPACECENTER_SPEEDMODE_ORBIT as ::core::ffi::c_int
    {
        SPEED_ORBIT as ::core::ffi::c_int
    } else {
        SPEED_SURFACE as ::core::ffi::c_int
    }) as NavballSpeedMode;
    (*client).has_speed_mode = true_0 != 0;
    snprintf(
        &raw mut (*client).cached_situation as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        situation_name(situation as krpc_SpaceCenter_VesselSituation_t),
    );
    (*client).medium_valid = true_0 != 0;
    return true_0 != 0;
}
unsafe extern "C" fn refresh_control_surface_state(
    mut client: *mut KrpcCNanoClient,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !(*client).surface_inventory_available {
        (*client).surface_state_valid = false_0 != 0;
        return true_0 != 0;
    }
    let mut batch: ClientBatch = ClientBatch {
        items: [
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
        ],
        count: 0,
    };
    let mut sample_ut: ::core::ffi::c_double = 0.0f64;
    if !batch_add(
        &raw mut batch,
        PROC_UT as ::core::ffi::c_int as uint32_t,
        BATCH_DOUBLE,
        &raw mut sample_ut as *mut ::core::ffi::c_void,
        true_0 != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct control-surface telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*client).surface_count {
        if batch.count.wrapping_add(3 as size_t) > KRPC_CNANO_BATCH_MAX as size_t {
            if !batch_run(
                client,
                &raw mut batch,
                b"control-surface telemetry\0" as *const u8
                    as *const ::core::ffi::c_char,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
            memset(
                &raw mut batch as *mut ::core::ffi::c_void,
                0 as ::core::ffi::c_int,
                ::core::mem::size_of::<ClientBatch>() as size_t,
            );
        }
        let mut surface: *mut CommandableSurface = (&raw mut (*client).surfaces
            as *mut CommandableSurface)
            .offset(i as isize) as *mut CommandableSurface;
        if !batch_add_object(
            &raw mut batch,
            PROC_CONTROL_SURFACE_DEPLOYED as ::core::ffi::c_int as uint32_t,
            (*surface).object as krpc_object_t,
            BATCH_BOOL,
            &raw mut (*surface).sampled_deployed as *mut ::core::ffi::c_void,
            true_0 != 0,
        )
            || !batch_add_object(
                &raw mut batch,
                PROC_CONTROL_SURFACE_DEFLECTION_OVERRIDE as ::core::ffi::c_int
                    as uint32_t,
                (*surface).object as krpc_object_t,
                BATCH_BOOL,
                &raw mut (*surface).sampled_deflection_override
                    as *mut ::core::ffi::c_void,
                true_0 != 0,
            )
            || !batch_add_object(
                &raw mut batch,
                PROC_CONTROL_SURFACE_DEFLECTION as ::core::ffi::c_int as uint32_t,
                (*surface).object as krpc_object_t,
                BATCH_FLOAT,
                &raw mut (*surface).sampled_override_deflection
                    as *mut ::core::ffi::c_void,
                true_0 != 0,
            )
        {
            set_error!(
                error,
                error_size,
                b"Could not construct control-surface telemetry batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    if batch.count != 0
        && !batch_run(
            client,
            &raw mut batch,
            b"control-surface telemetry\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
    {
        return false_0 != 0;
    }
    (*client).surface_state_ut = sample_ut;
    (*client).surface_state_valid = true_0 != 0;
    return true_0 != 0;
}
unsafe extern "C" fn surface_role_name(
    mut role: KrpcSurfaceRole,
) -> *const ::core::ffi::c_char {
    match role as ::core::ffi::c_uint {
        1 => return b"elevator\0" as *const u8 as *const ::core::ffi::c_char,
        2 => return b"aileron\0" as *const u8 as *const ::core::ffi::c_char,
        3 => return b"rudder\0" as *const u8 as *const ::core::ffi::c_char,
        4 => return b"body-flap\0" as *const u8 as *const ::core::ffi::c_char,
        _ => return b"unknown\0" as *const u8 as *const ::core::ffi::c_char,
    };
}
unsafe extern "C" fn refresh_slow(
    mut client: *mut KrpcCNanoClient,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut apoapsis: ::core::ffi::c_double = (*client).cached_apoapsis;
    let mut periapsis: ::core::ffi::c_double = (*client).cached_periapsis;
    let mut period: ::core::ffi::c_double = (*client).cached_period;
    let mut batch: ClientBatch = ClientBatch {
        items: [
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
        ],
        count: 0,
    };
    if !batch_add_object(
        &raw mut batch,
        PROC_ORBIT_APOAPSIS_ALTITUDE as ::core::ffi::c_int as uint32_t,
        (*client).orbit as krpc_object_t,
        BATCH_DOUBLE,
        &raw mut apoapsis as *mut ::core::ffi::c_void,
        true_0 != 0,
    )
        || !batch_add_object(
            &raw mut batch,
            PROC_ORBIT_PERIAPSIS_ALTITUDE as ::core::ffi::c_int as uint32_t,
            (*client).orbit as krpc_object_t,
            BATCH_DOUBLE,
            &raw mut periapsis as *mut ::core::ffi::c_void,
            true_0 != 0,
        )
        || !batch_add_object(
            &raw mut batch,
            PROC_ORBIT_PERIOD as ::core::ffi::c_int as uint32_t,
            (*client).orbit as krpc_object_t,
            BATCH_DOUBLE,
            &raw mut period as *mut ::core::ffi::c_void,
            true_0 != 0,
        )
    {
        set_error!(
            error,
            error_size,
            b"Could not construct slow C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_run(
        client,
        &raw mut batch,
        b"slow telemetry\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*client).cached_apoapsis = apoapsis;
    (*client).cached_periapsis = periapsis;
    (*client).cached_period = period;
    (*client).slow_valid = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_read(
    mut client: *mut KrpcCNanoClient,
    mut configuration: *const LandingConfiguration,
    mut t: *mut Telemetry,
    mut state: *mut VehicleState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected || configuration.is_null() || t.is_null()
        || state.is_null()
    {
        set_error!(
            error,
            error_size,
            b"C-Nano telemetry read has invalid state\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut started: ::core::ffi::c_double = monotonic_seconds();
    let mut before: ::core::ffi::c_uint = (*client).total_rpc_calls;
    let mut wire_before: ::core::ffi::c_uint = (*client).total_wire_requests;
    telemetry_init(t);
    snprintf(
        &raw mut (*t).vessel_name as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut (*client).vessel_name as *mut ::core::ffi::c_char,
    );
    let mut ut: ::core::ffi::c_double = 0.0f64;
    let mut radar: ::core::ffi::c_double = 0.0f64;
    let mut true_air_speed: ::core::ffi::c_float = 0.0f32;
    let mut heading: ::core::ffi::c_float = 0.0f32;
    let mut pitch: ::core::ffi::c_float = 0.0f32;
    let mut roll: ::core::ffi::c_float = 0.0f32;
    let mut aoa: ::core::ffi::c_float = 0.0f32;
    let mut beta: ::core::ffi::c_float = 0.0f32;
    let mut q: ::core::ffi::c_float = 0.0f32;
    let mut mass: ::core::ffi::c_float = 0.0f32;
    let mut cp: ::core::ffi::c_float = 0.0f32;
    let mut cr: ::core::ffi::c_float = 0.0f32;
    let mut cy: ::core::ffi::c_float = 0.0f32;
    let mut throttle: ::core::ffi::c_float = 0.0f32;
    let mut autopilot_error: ::core::ffi::c_float = 180.0f32;
    let mut sas: bool = false_0 != 0;
    let mut position: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut velocity: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut angular: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
    };
    let mut rotation: krpc_tuple_double_double_double_double_t = krpc_tuple_double_double_double_double_s {
        e0: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        e1: 0.,
        e2: 0.,
        e3: 0.,
    };
    let mut main_gear_grounded: [bool; 2] = [false_0 != 0, false_0 != 0];
    let mut sample_main_gear: bool = (*client).main_wheel_grounded_field_count
        > 0 as size_t && (*client).last_radar_altitude.is_finite() as i32 != 0
        && (*client).last_radar_altitude <= 100.0f64;
    let mut fast: ClientBatch = ClientBatch {
        items: [
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
        ],
        count: 0,
    };
    if !batch_add(
        &raw mut fast,
        PROC_UT as ::core::ffi::c_int as uint32_t,
        BATCH_DOUBLE,
        &raw mut ut as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_SURFACE_ALTITUDE as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_DOUBLE,
        &raw mut radar as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_TRUE_AIR_SPEED as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut true_air_speed as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_HEADING as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut heading as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_PITCH as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut pitch as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_ROLL as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut roll as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_AOA as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut aoa as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_SIDESLIP as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut beta as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_FLIGHT_DYNAMIC_PRESSURE as ::core::ffi::c_int as uint32_t,
        (*client).flight_surface as krpc_object_t,
        BATCH_FLOAT,
        &raw mut q as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_two_objects(
        &raw mut fast,
        PROC_VESSEL_POSITION as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        (*client).body_nonrotating as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut position as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_two_objects(
        &raw mut fast,
        PROC_VESSEL_VELOCITY as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        (*client).body_nonrotating as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut velocity as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_two_objects(
        &raw mut fast,
        PROC_VESSEL_ANGULAR_VELOCITY as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        (*client).body_nonrotating as krpc_object_t,
        BATCH_TUPLE3,
        &raw mut angular as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_VESSEL_MASS as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        BATCH_FLOAT,
        &raw mut mass as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_CONTROL_PITCH as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_FLOAT,
        &raw mut cp as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_CONTROL_ROLL as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_FLOAT,
        &raw mut cr as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_CONTROL_YAW as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_FLOAT,
        &raw mut cy as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_CONTROL_THROTTLE as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_FLOAT,
        &raw mut throttle as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object(
        &raw mut fast,
        PROC_CONTROL_SAS as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        BATCH_BOOL,
        &raw mut sas as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if (*client).has_autopilot_engaged as ::core::ffi::c_int != 0
        && (*client).cached_autopilot_engaged as ::core::ffi::c_int != 0
    {
        if !batch_add_object(
            &raw mut fast,
            PROC_AUTOPILOT_ERROR as ::core::ffi::c_int as uint32_t,
            (*client).autopilot as krpc_object_t,
            BATCH_FLOAT,
            &raw mut autopilot_error as *mut ::core::ffi::c_void,
            1 as ::core::ffi::c_int != 0,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
    }
    if !batch_add_two_objects(
        &raw mut fast,
        PROC_VESSEL_ROTATION as ::core::ffi::c_int as uint32_t,
        (*client).vessel as krpc_object_t,
        (*client).body_nonrotating as krpc_object_t,
        BATCH_TUPLE4,
        &raw mut rotation as *mut ::core::ffi::c_void,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if sample_main_gear {
        let mut i: size_t = 0 as size_t;
        while i < (*client).main_wheel_grounded_field_count {
            if !batch_add_object(
                &raw mut fast,
                PROC_PART_FIELD_BOOL_VALUE as ::core::ffi::c_int as uint32_t,
                (*client).main_wheel_grounded_fields[i as usize],
                BATCH_BOOL,
                (&raw mut main_gear_grounded as *mut bool).offset(i as isize)
                    as *mut bool as *mut ::core::ffi::c_void,
                1 as ::core::ffi::c_int != 0,
            ) {
                set_error!(
                    error,
                    error_size,
                    b"Could not construct fast C-Nano telemetry batch\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
                return false_0 != 0;
            }
            i = i.wrapping_add(1);
        }
    }
    if !batch_run(
        client,
        &raw mut fast,
        b"fast telemetry\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*t).ut = ut;
    (*t).radar_altitude = radar;
    (*t).true_air_speed = true_air_speed as ::core::ffi::c_double;
    (*t).heading = heading as ::core::ffi::c_double;
    (*t).pitch = pitch as ::core::ffi::c_double;
    (*t).roll = roll as ::core::ffi::c_double;
    (*t).angle_of_attack = aoa as ::core::ffi::c_double;
    (*t).sideslip = beta as ::core::ffi::c_double;
    (*t).dynamic_pressure = q as ::core::ffi::c_double;
    (*t).mass = mass as ::core::ffi::c_double;
    if sample_main_gear {
        (*t).has_main_gear_grounded = true_0 != 0;
        let mut i_0: size_t = 0 as size_t;
        while i_0 < (*client).main_wheel_grounded_field_count {
            (*t).main_gear_grounded = (*t).main_gear_grounded as ::core::ffi::c_int != 0
                || main_gear_grounded[i_0 as usize] as ::core::ffi::c_int != 0;
            i_0 = i_0.wrapping_add(1);
        }
    }
    (*client).last_radar_altitude = radar;
    (*t).has_controls = true_0 != 0;
    (*t).control_pitch = cp as ::core::ffi::c_double;
    (*t).control_roll = cr as ::core::ffi::c_double;
    (*t).control_yaw = cy as ::core::ffi::c_double;
    (*t).throttle = throttle as ::core::ffi::c_double;
    (*t).has_sas_state = true_0 != 0;
    (*t).sas_enabled = sas;
    (*client).cached_sas = sas;
    (*client).has_sas = true_0 != 0;
    (*t).autopilot_error = fabs(autopilot_error as ::core::ffi::c_double);
    let mut pos: Vector3 = tuple_to_canonical(position);
    let mut vel: Vector3 = tuple_to_canonical(velocity);
    let mut geo: GeoPoint = predictor_geo_point(pos, &raw mut (*client).planet, ut);
    (*t).latitude = geo.latitude;
    (*t).longitude = geo.longitude;
    (*t).mean_altitude = geo.altitude;
    let mut up: Vector3 = vnorm(
        pos,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut surface_velocity: Vector3 = vsub(
        vel,
        vcross(planet_rotation_vector(&raw mut (*client).planet), pos),
    );
    (*t).vertical_speed = vdot(surface_velocity, up);
    (*t).horizontal_speed = vmag(vproject_plane(surface_velocity, up));
    (*t).surface_speed = vmag(surface_velocity);
    (*t).flight_path_angle = atan2(
        (*t).vertical_speed,
        fmax(0.1f64, (*t).horizontal_speed),
    ) * RAD2DEG;
    if (*t).true_air_speed > 1.0f64 && (*t).dynamic_pressure >= 0.0f64 {
        (*t).atmospheric_density = 2.0f64 * (*t).dynamic_pressure
            / ((*t).true_air_speed * (*t).true_air_speed);
    } else {
        (*t).atmospheric_density = planet_atmospheric_density(
            &raw mut (*client).planet,
            (*t).mean_altitude,
        );
    }
    let mut qx: ::core::ffi::c_double = rotation.e0;
    let mut qy: ::core::ffi::c_double = rotation.e1;
    let mut qz: ::core::ffi::c_double = rotation.e2;
    let mut qw: ::core::ffi::c_double = rotation.e3;
    let mut r00: ::core::ffi::c_double = 1.0f64 - 2.0f64 * (qy * qy + qz * qz);
    let mut r01: ::core::ffi::c_double = 2.0f64 * (qx * qy - qz * qw);
    let mut r02: ::core::ffi::c_double = 2.0f64 * (qx * qz + qy * qw);
    let mut r10: ::core::ffi::c_double = 2.0f64 * (qx * qy + qz * qw);
    let mut r11: ::core::ffi::c_double = 1.0f64 - 2.0f64 * (qx * qx + qz * qz);
    let mut r12: ::core::ffi::c_double = 2.0f64 * (qy * qz - qx * qw);
    let mut r20: ::core::ffi::c_double = 2.0f64 * (qx * qz - qy * qw);
    let mut r21: ::core::ffi::c_double = 2.0f64 * (qy * qz + qx * qw);
    let mut r22: ::core::ffi::c_double = 1.0f64 - 2.0f64 * (qx * qx + qy * qy);
    let mut body_x: ::core::ffi::c_double = r00 * angular.e0 + r10 * angular.e1
        + r20 * angular.e2;
    let mut body_y: ::core::ffi::c_double = r01 * angular.e0 + r11 * angular.e1
        + r21 * angular.e2;
    let mut body_z: ::core::ffi::c_double = r02 * angular.e0 + r12 * angular.e1
        + r22 * angular.e2;
    let mut sv_raw: [::core::ffi::c_double; 3] = [
        surface_velocity.x,
        surface_velocity.z,
        surface_velocity.y,
    ];
    let mut bx: ::core::ffi::c_double = r00 * sv_raw[0 as ::core::ffi::c_int as usize]
        + r10 * sv_raw[1 as ::core::ffi::c_int as usize]
        + r20 * sv_raw[2 as ::core::ffi::c_int as usize];
    let mut by: ::core::ffi::c_double = r01 * sv_raw[0 as ::core::ffi::c_int as usize]
        + r11 * sv_raw[1 as ::core::ffi::c_int as usize]
        + r21 * sv_raw[2 as ::core::ffi::c_int as usize];
    let mut bz: ::core::ffi::c_double = r02 * sv_raw[0 as ::core::ffi::c_int as usize]
        + r12 * sv_raw[1 as ::core::ffi::c_int as usize]
        + r22 * sv_raw[2 as ::core::ffi::c_int as usize];
    let mut speed: ::core::ffi::c_double = sqrt(bx * bx + by * by + bz * bz);
    if speed > 1.0f64 && speed.is_finite() as i32 != 0 {
        let mut own_aoa: ::core::ffi::c_double = asin(
            clampd(bz / speed, -1.0f64, 1.0f64),
        ) * RAD2DEG;
        let mut own_beta: ::core::ffi::c_double = asin(
            clampd(bx / speed, -1.0f64, 1.0f64),
        ) * RAD2DEG;
        if !(*client).root_frame_aoa_reported && speed > 50.0f64
            && fabs(own_aoa - (*t).angle_of_attack) > 1.0f64
        {
            (*client).root_frame_aoa_reported = true_0 != 0;
            fprintf(
                stderr,
                b"kRPC root-part incidence differs from the controlling part: kRPC AoA %.1f / sideslip %.1f, controlling-part AoA %.1f / sideslip %.1f; using the controlling part.\n\0"
                    as *const u8 as *const ::core::ffi::c_char,
                (*t).angle_of_attack,
                (*t).sideslip,
                own_aoa,
                own_beta,
            );
        }
        (*t).angle_of_attack = own_aoa;
        (*t).sideslip = own_beta;
    }
    (*t).has_body_yaw_rate = true_0 != 0;
    (*t).has_body_roll_rate = (*t).has_body_yaw_rate;
    (*t).has_body_pitch_rate = (*t).has_body_roll_rate;
    (*t).body_pitch_rate = -body_x * RAD2DEG;
    (*t).body_roll_rate = -body_y * RAD2DEG;
    (*t).body_yaw_rate = -body_z * RAD2DEG;
    (*t).has_attitude_quaternion = true_0 != 0;
    (*t).attitude_quaternion[0 as ::core::ffi::c_int as usize] = rotation.e0;
    (*t).attitude_quaternion[1 as ::core::ffi::c_int as usize] = rotation.e1;
    (*t).attitude_quaternion[2 as ::core::ffi::c_int as usize] = rotation.e2;
    (*t).attitude_quaternion[3 as ::core::ffi::c_int as usize] = rotation.e3;
    snprintf(
        &raw mut (*t).attitude_reference_frame as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
        b"body-non-rotating\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if (*client).has_previous_attitude {
        let mut dt: ::core::ffi::c_double = clampd(
            ut - (*client).previous_ut,
            0.01f64,
            1.0f64,
        );
        (*t).pitch_rate = lowpass_update(
            &raw mut (*client).pitch_rate_filter,
            norm_signed_deg((*t).pitch - (*client).previous_pitch) / dt,
            dt,
        );
        (*t).roll_rate = lowpass_update(
            &raw mut (*client).roll_rate_filter,
            norm_signed_deg((*t).roll - (*client).previous_roll) / dt,
            dt,
        );
        (*t).heading_rate = lowpass_update(
            &raw mut (*client).heading_rate_filter,
            norm_signed_deg((*t).heading - (*client).previous_heading) / dt,
            dt,
        );
        (*t).angle_of_attack_rate = lowpass_update(
            &raw mut (*client).angle_of_attack_rate_filter,
            norm_signed_deg((*t).angle_of_attack - (*client).previous_angle_of_attack)
                / dt,
            dt,
        );
        (*t).has_angle_of_attack_rate = true_0 != 0;
    }
    (*client).has_previous_attitude = true_0 != 0;
    (*client).previous_ut = ut;
    (*client).previous_pitch = (*t).pitch;
    (*client).previous_roll = (*t).roll;
    (*client).previous_heading = (*t).heading;
    (*client).previous_angle_of_attack = (*t).angle_of_attack;
    (*t).ground_track_heading = surface_course(
        pos,
        vel,
        planet_rotation_vector(&raw mut (*client).planet),
        (*client).planet.north_axis,
        (*t).heading,
    );
    if (*client).has_previous_course {
        let mut dt_0: ::core::ffi::c_double = ut - (*client).previous_course_ut;
        if dt_0 >= 0.01f64 && dt_0 <= 1.0f64 {
            (*t).course_rate = lowpass_update(
                &raw mut (*client).course_rate_filter,
                norm_signed_deg((*t).ground_track_heading - (*client).previous_course)
                    / dt_0,
                dt_0,
            );
            (*t).has_course_rate = true_0 != 0;
        } else {
            lowpass_reset(&raw mut (*client).course_rate_filter);
        }
    }
    (*client).has_previous_course = true_0 != 0;
    (*client).previous_course_ut = ut;
    (*client).previous_course = (*t).ground_track_heading;
    let mut burn_active: bool = (*client).cached_command_throttle > 0.0f64
        || (*client).cached_current_thrust > 0.0f64;
    if burn_active as ::core::ffi::c_int != 0 || !(*client).medium_valid
        || ut - (*client).last_medium_ut >= CNANO_MEDIUM_PERIOD
    {
        if !refresh_medium(
            client,
            (*configuration).vehicle.airbrake_action_group,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).last_medium_ut = ut;
    }
    if (*client).surface_inventory_available as ::core::ffi::c_int != 0
        && (!(*client).surface_state_valid || ut < (*client).surface_state_ut
            || ut - (*client).surface_state_ut >= CNANO_MEDIUM_PERIOD)
    {
        if !refresh_control_surface_state(client, error, error_size) {
            return false_0 != 0;
        }
    }
    if burn_active as ::core::ffi::c_int != 0 || !(*client).slow_valid
        || ut - (*client).last_slow_ut >= CNANO_SLOW_PERIOD
    {
        if !refresh_slow(client, error, error_size) {
            return false_0 != 0;
        }
        (*client).last_slow_ut = ut;
    }
    if burn_active as ::core::ffi::c_int != 0 && !(*client).burn_geometry_valid {
        refresh_burn_geometry(client);
    }
    (*t).speed_of_sound = if (*client).cached_speed_of_sound > 1.0f64 {
        (*client).cached_speed_of_sound
    } else {
        planet_atmospheric_speed_of_sound(&raw mut (*client).planet, (*t).mean_altitude)
    };
    let mut mach_reference: ::core::ffi::c_double = (*t).speed_of_sound;
    if !(mach_reference > 1.0f64)
        && (*client).planet.atmosphere_depth.is_finite() as i32 != 0
        && (*client).planet.atmosphere_depth > 1.0f64
    {
        mach_reference = planet_atmospheric_speed_of_sound(
            &raw mut (*client).planet,
            (*client).planet.atmosphere_depth - 1.0f64,
        );
    }
    (*t).mach = (*t).true_air_speed
        / fmax(
            1.0f64,
            (if mach_reference.is_finite() as i32 != 0 {
                mach_reference
            } else {
                1.0f64
            }),
        );
    (*t).static_pressure = (*client).cached_static_pressure;
    (*t).g_force = (*client).cached_g_force;
    (*t).dry_mass = (*client).cached_dry_mass;
    (*t).available_thrust = (*client).commandable_thrust;
    (*t).current_thrust = (*client).cached_current_thrust
        * (*client).burn_resultant_fraction;
    if throttle_feedback_requires_reassertion(
        (*client).has_command_throttle,
        (*client).cached_command_throttle,
        (*t).throttle,
        (*t).current_thrust,
    ) {
        (*client).has_command_throttle = false_0 != 0;
    }
    (*t).has_torque = (*client).cached_torque[0 as ::core::ffi::c_int as usize]
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*client).cached_torque[1 as ::core::ffi::c_int as usize]
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*client).cached_torque[2 as ::core::ffi::c_int as usize]
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*t).available_pitch_torque = (*client)
        .cached_torque[0 as ::core::ffi::c_int as usize];
    (*t).available_roll_torque = (*client)
        .cached_torque[1 as ::core::ffi::c_int as usize];
    (*t).available_yaw_torque = (*client)
        .cached_torque[2 as ::core::ffi::c_int as usize];
    (*t).has_inertia = (*client).cached_inertia[0 as ::core::ffi::c_int as usize]
        > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*client).cached_inertia[1 as ::core::ffi::c_int as usize]
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*client).cached_inertia[2 as ::core::ffi::c_int as usize]
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*t).pitch_moment_of_inertia = (*client)
        .cached_inertia[0 as ::core::ffi::c_int as usize];
    (*t).roll_moment_of_inertia = (*client)
        .cached_inertia[1 as ::core::ffi::c_int as usize];
    (*t).yaw_moment_of_inertia = (*client)
        .cached_inertia[2 as ::core::ffi::c_int as usize];
    (*t).apoapsis_altitude = (*client).cached_apoapsis;
    (*t).periapsis_altitude = (*client).cached_periapsis;
    (*t).orbit_period = (*client).cached_period;
    (*t).gear = (*client).cached_gear;
    (*t).brakes = (*client).cached_brakes;
    (*t).has_airbrakes = (*client).has_airbrakes;
    (*t).airbrakes = (*client).cached_airbrakes;
    (*t).has_control_surface_state = (*client).surface_inventory_available
        as ::core::ffi::c_int != 0
        && (*client).surface_state_valid as ::core::ffi::c_int != 0;
    if (*t).has_control_surface_state {
        (*t).control_surface_state_ut = (*client).surface_state_ut;
        (*t).control_surface_count = if (*client).surface_count
            < LANDING_CONTROL_SURFACE_MAX as size_t
        {
            (*client).surface_count
        } else {
            LANDING_CONTROL_SURFACE_MAX as size_t
        };
        let mut i_1: size_t = 0 as size_t;
        while i_1 < (*t).control_surface_count {
            let mut source: *const CommandableSurface = (&raw mut (*client).surfaces
                as *mut CommandableSurface)
                .offset(i_1 as isize) as *mut CommandableSurface;
            let mut target: *mut TelemetryControlSurface = (&raw mut (*t)
                .control_surfaces as *mut TelemetryControlSurface)
                .offset(i_1 as isize) as *mut TelemetryControlSurface;
            snprintf(
                &raw mut (*target).part_name as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                &raw const (*source).part_name as *const ::core::ffi::c_char,
            );
            snprintf(
                &raw mut (*target).role as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 16]>() as size_t,
                b"%s\0" as *const u8 as *const ::core::ffi::c_char,
                surface_role_name((*source).role),
            );
            (*target).deployed = (*source).sampled_deployed;
            (*target).deflection_override = (*source).sampled_deflection_override;
            (*target).override_deflection = (*source).sampled_override_deflection
                as ::core::ffi::c_double;
            i_1 = i_1.wrapping_add(1);
        }
    }
    (*t).navball_speed_mode = (if (*client).has_speed_mode as ::core::ffi::c_int != 0 {
        (*client).cached_speed_mode as ::core::ffi::c_uint
    } else {
        SPEED_UNCHANGED as ::core::ffi::c_int as ::core::ffi::c_uint
    }) as NavballSpeedMode;
    if (*client).cached_situation[0 as ::core::ffi::c_int as usize] != 0 {
        snprintf(
            &raw mut (*t).vessel_situation as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
            b"%s\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut (*client).cached_situation as *mut ::core::ffi::c_char,
        );
    }
    (*t).has_force_vectors = (*client).cached_force_vectors_valid;
    (*t).lift_vector = (*client).cached_lift_vector;
    (*t).drag_vector = (*client).cached_drag_vector;
    (*t).lift_force = vmag((*t).lift_vector);
    (*t).drag_force = vmag((*t).drag_vector);
    (*t).has_center_of_mass = (*client).cached_center_of_mass_valid;
    (*t).center_of_mass = (*client).cached_center_of_mass;
    (*t).has_center_of_mass_root = (*client).cached_center_of_mass_root_valid;
    (*t).center_of_mass_root = (*client).cached_center_of_mass_root;
    (*t).stall_fraction = vessel_physics_conservative_stall_fraction(
        (*t).true_air_speed,
        (*t).angle_of_attack,
        (*t).dynamic_pressure,
        &raw const (*configuration).vehicle,
    );
    (*t).stall_fraction_is_measured = false_0 != 0;
    (*t).physics_sample_valid = (*t).has_force_vectors as ::core::ffi::c_int != 0
        && (*t).dynamic_pressure.is_finite() as i32 != 0
        && (*t).dynamic_pressure >= 0.0f64 && (*t).mass.is_finite() as i32 != 0
        && (*t).mass > 0.0f64 && (*t).lift_force.is_finite() as i32 != 0
        && (*t).drag_force.is_finite() as i32 != 0;
    if (*t).physics_sample_valid {
        (*t).g_force = (*t).lift_force / ((*t).mass * 9.80665f64);
    }
    let mut current: GeoPoint = GeoPoint {
        latitude: (*t).latitude,
        longitude: (*t).longitude,
        altitude: (*t).mean_altitude,
    };
    let mut site: GeoPoint = GeoPoint {
        latitude: (*configuration).site.latitude,
        longitude: (*configuration).site.longitude,
        altitude: (*configuration).site.altitude,
    };
    (*t).range_to_site = great_circle_distance(current, site, (*client).planet.radius);
    (*t).bearing_to_site = initial_bearing(current, site);
    (*t).heading_error = norm_signed_deg((*t).bearing_to_site - (*t).heading);
    (*t).course_to_site_error = norm_signed_deg(
        (*t).bearing_to_site - (*t).ground_track_heading,
    );
    runway_coordinates(
        current,
        site,
        (*configuration).site.runway_heading,
        (*client).planet.radius,
        &raw mut (*t).runway_along_track,
        &raw mut (*t).runway_cross_track,
    );
    (*state).ut = ut;
    (*state).position = pos;
    (*state).velocity = vel;
    (*state).mass = (*t).mass;
    (*client).last_read_calls = (*client).total_rpc_calls.wrapping_sub(before);
    (*client).last_read_wire_requests = (*client)
        .total_wire_requests
        .wrapping_sub(wire_before);
    (*client).last_read_seconds = monotonic_seconds() - started;
    return true_0 != 0;
}
pub const KRPC_ALLOC_BLOCK_SIZE: ::core::ffi::c_int = 4 as ::core::ffi::c_int;
#[inline]
unsafe extern "C" fn krpc_decode_callback_item_list_object(
    mut stream: *mut pb_istream_t,
    mut field: *const pb_field_t,
    mut arg: *mut *mut ::core::ffi::c_void,
) -> bool {
    let mut state: *mut State_0 = *arg as *mut State_0;
    let mut i: size_t = (*(*state).value).size;
    (*(*state).value).size = (*(*state).value).size.wrapping_add(1);
    if (*state).capacity > 0 as size_t && (*(*state).value).size > (*state).capacity {
        (*(*state).value).items = krpc_recalloc(
            (*(*state).value).items as *mut ::core::ffi::c_void,
            (*state).capacity,
            KRPC_ALLOC_BLOCK_SIZE as size_t,
            ::core::mem::size_of::<krpc_object_t>() as size_t,
        ) as *mut krpc_object_t;
        (*state).capacity = (*state)
            .capacity
            .wrapping_add(KRPC_ALLOC_BLOCK_SIZE as size_t);
    }
    if krpc_decode_object(
        stream,
        (*(*state).value).items.offset(i as isize) as *mut krpc_object_t,
    ) as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn krpc_decode_list_object(
    mut stream: *mut pb_istream_t,
    mut value: *mut krpc_list_object_t,
) -> krpc_error_t {
    let mut state: State = State {
        capacity: 0 as size_t,
        value: value,
    };
    (*value).size = 0 as size_t;
    if (*value).items.is_null() {
        (*value).items = krpc_calloc(
            KRPC_ALLOC_BLOCK_SIZE as size_t,
            ::core::mem::size_of::<krpc_object_t>() as size_t,
        ) as *mut krpc_object_t;
        state.capacity = KRPC_ALLOC_BLOCK_SIZE as size_t;
    }
    let mut message: krpc_schema_List = _krpc_schema_List {
        items: pb_callback_s {
            funcs: C2RustUnnamed { decode: None },
            arg: NULL_0,
        },
    };
    message.items.funcs.decode = Some(
        krpc_decode_callback_item_list_object
            as unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
    )
        as Option<
            unsafe extern "C" fn(
                *mut pb_istream_t,
                *const pb_field_t,
                *mut *mut ::core::ffi::c_void,
            ) -> bool,
        >;
    message.items.arg = &raw mut state as *mut ::core::ffi::c_void;
    let mut error: krpc_error_t = KRPC_OK;
    error = krpc_decode_message_List(stream, &raw mut message);
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_KRPC_Paused(
    mut connection: krpc_connection_t,
    mut returnValue: *mut bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        1 as uint32_t,
        14 as uint32_t,
        0 as size_t,
        ::core::ptr::null_mut::<krpc_argument_t>(),
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_0: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut error_1: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    if !returnValue.is_null() {
        let mut _stream: pb_istream_t = pb_istream_s {
            callback: None,
            state: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            bytes_left: 0,
            errmsg: ::core::ptr::null::<::core::ffi::c_char>(),
        };
        let mut error_2: krpc_error_t = krpc_get_return_value(
            &raw mut _result,
            &raw mut _stream,
        );
        if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_2;
        }
        let mut error_3: krpc_error_t = krpc_decode_bool(&raw mut _stream, returnValue);
        if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
            return error_3;
        }
    }
    let mut error_4: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_4 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_4;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn krpc_KRPC_set_Paused(
    mut connection: krpc_connection_t,
    mut value: bool,
) -> krpc_error_t {
    let mut _call: krpc_call_t = krpc_call_t {
        message: _krpc_schema_ProcedureCall {
            arguments: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
            service_id: 0,
            procedure_id: 0,
        },
        numArguments: 0,
        arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
    };
    let mut _arguments: [krpc_argument_t; 1] = [krpc_argument_t {
        message: _krpc_schema_Argument {
            position: 0,
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
            },
        },
    }; 1];
    let mut error: krpc_error_t = krpc_call(
        &raw mut _call,
        1 as uint32_t,
        15 as uint32_t,
        1 as size_t,
        &raw mut _arguments as *mut krpc_argument_t,
    );
    if error as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error;
    }
    let mut error_0: krpc_error_t = krpc_add_argument(
        &raw mut _call,
        0 as uint32_t,
        Some(
            krpc_encode_callback_bool
                as unsafe extern "C" fn(
                    *mut pb_ostream_t,
                    *const pb_field_t,
                    *const *mut ::core::ffi::c_void,
                ) -> bool,
        ),
        &raw mut value as *const ::core::ffi::c_void,
    );
    if error_0 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_0;
    }
    let mut _result: krpc_result_t = krpc_result_t {
        message: _krpc_schema_ProcedureResult {
            error: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
            value: pb_callback_s {
                funcs: C2RustUnnamed { decode: None },
                arg: NULL_0,
            },
        },
        size: 0 as size_t,
        buffer: ::core::ptr::null_mut::<uint8_t>(),
    };
    let mut error_1: krpc_error_t = krpc_init_result(&raw mut _result);
    if error_1 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_1;
    }
    let mut error_2: krpc_error_t = krpc_invoke(
        connection,
        &raw mut _result.message,
        &raw mut _call.message,
    );
    if error_2 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_2;
    }
    let mut error_3: krpc_error_t = krpc_free_result(&raw mut _result);
    if error_3 as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int {
        return error_3;
    }
    return KRPC_OK;
}
#[inline]
unsafe extern "C" fn tolower(mut __c: ::core::ffi::c_int) -> ::core::ffi::c_int {
    return if __c >= -(128 as ::core::ffi::c_int) && __c < 256 as ::core::ffi::c_int {
        *(*__ctype_tolower_loc()).offset(__c as isize) as ::core::ffi::c_int
    } else {
        __c
    };
}
#[inline]
unsafe extern "C" fn thrust_alignment_rotate(
    mut v: Vector3,
    mut from: Vector3,
) -> Vector3 {
    let mut a: Vector3 = vnorm(
        from,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut b: Vector3 = v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut k: Vector3 = vcross(a, b);
    let mut c: ::core::ffi::c_double = clampd(
        vdot(a, b),
        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    if c < -(1 as ::core::ffi::c_int) as ::core::ffi::c_double + 1e-10f64 {
        return v3(v.x, -v.y, -v.z);
    }
    return vadd(
        v,
        vadd(
            vcross(k, v),
            vscale(
                vcross(k, vcross(k, v)),
                1 as ::core::ffi::c_int as ::core::ffi::c_double
                    / (1 as ::core::ffi::c_int as ::core::ffi::c_double + c),
            ),
        ),
    );
}
#[inline]
unsafe extern "C" fn thrust_alignment_map(
    mut body: Vector3,
    mut direction: Vector3,
    mut up: Vector3,
) -> Vector3 {
    let mut d: Vector3 = vnorm(
        direction,
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut u: Vector3 = vnorm(
        vproject_plane(up, d),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut right: Vector3 = vnorm(
        vcross(d, u),
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    return vadd(vscale(right, body.x), vadd(vscale(d, body.y), vscale(u, -body.z)));
}
#[inline]
unsafe extern "C" fn thrust_alignment_attitude(
    mut axis: Vector3,
    mut direction: Vector3,
    mut up: Vector3,
    mut nose: *mut Vector3,
    mut roof: *mut Vector3,
) {
    *nose = thrust_alignment_map(
        thrust_alignment_rotate(
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
            axis,
        ),
        direction,
        up,
    );
    *roof = thrust_alignment_map(
        thrust_alignment_rotate(
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
            ),
            axis,
        ),
        direction,
        up,
    );
}
pub const KRPC_CNANO_BATCH_MAX: ::core::ffi::c_uint = 32 as ::core::ffi::c_uint;
#[inline]
unsafe extern "C" fn throttle_feedback_requires_reassertion(
    mut has_command: bool,
    mut commanded_throttle: ::core::ffi::c_double,
    mut measured_global_throttle: ::core::ffi::c_double,
    mut measured_thrust_n: ::core::ffi::c_double,
) -> bool {
    if !has_command {
        return false_0 != 0;
    }
    if measured_global_throttle.is_finite() as i32 != 0
        && fabs(measured_global_throttle) > 1e-4f64
    {
        return true_0 != 0;
    }
    return commanded_throttle.is_finite() as i32 != 0 && commanded_throttle <= 1e-4f64
        && measured_thrust_n.is_finite() as i32 != 0 && measured_thrust_n > 1.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_save(
    mut client: *mut KrpcCNanoClient,
    mut name: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected || name.is_null() || *name == 0 {
        set_error!(
            error,
            error_size,
            b"Save name is empty\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_Save((*client).connection, name),
        b"krpc_SpaceCenter_Save(client->connection, name)\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn safe_flush_batch(
    mut client: *mut KrpcCNanoClient,
    mut batch: *mut ClientBatch,
) -> bool {
    if batch.is_null() || (*batch).count == 0 as size_t {
        return true_0 != 0;
    }
    let mut ignored: [::core::ffi::c_char; 256] = [
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
    let mut ok: bool = batch_run(
        client,
        batch,
        b"safe control\0" as *const u8 as *const ::core::ffi::c_char,
        &raw mut ignored as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
    );
    memset(
        batch as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<ClientBatch>() as size_t,
    );
    return ok;
}
unsafe extern "C" fn set_paused_triplet(
    mut client: *mut KrpcCNanoClient,
    mut paused: bool,
) {
    if client.is_null() || !(*client).connected
        || (*client).pause_triplet_sent as ::core::ffi::c_int != 0
    {
        return;
    }
    let mut _e: krpc_error_t = krpc_KRPC_set_Paused((*client).connection, paused);
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    let mut pause_gap: timespec = timespec {
        tv_sec: 0 as __time_t,
        tv_nsec: 200000000 as __syscall_slong_t,
    };
    nanosleep(&raw mut pause_gap, ::core::ptr::null_mut::<timespec>());
    let mut _e_0: krpc_error_t = krpc_KRPC_set_Paused((*client).connection, !paused);
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    nanosleep(&raw mut pause_gap, ::core::ptr::null_mut::<timespec>());
    let mut _e_1: krpc_error_t = krpc_KRPC_set_Paused((*client).connection, paused);
    (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
    (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    (*client).pause_triplet_sent = true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_neutralize(mut client: *mut KrpcCNanoClient) {
    let mut current_block: u64;
    if client.is_null() || !(*client).connected {
        return;
    }
    let mut batch: ClientBatch = ClientBatch {
        items: [
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
        ],
        count: 0,
    };
    if !batch_add_object_bool(
        &raw mut batch,
        PROC_AUTOPILOT_SET_ENGAGED as ::core::ffi::c_int as uint32_t,
        (*client).autopilot as krpc_object_t,
        0 as ::core::ffi::c_int != 0,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_bool(
        &raw mut batch,
        PROC_CONTROL_SET_SAS as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0 as ::core::ffi::c_int != 0,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_PITCH as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0.0f32,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_ROLL as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0.0f32,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_YAW as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0.0f32,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_THROTTLE as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0.0f32,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_WHEEL_STEERING as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0.0f32,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else if !batch_add_object_bool(
        &raw mut batch,
        PROC_CONTROL_SET_RCS as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        0 as ::core::ffi::c_int != 0,
        0 as ::core::ffi::c_int != 0,
    ) {
        safe_flush_batch(client, &raw mut batch);
    } else {
        let mut i: size_t = 0 as size_t;
        loop {
            if !(i < (*client).engine_count) {
                current_block = 5891011138178424807;
                break;
            }
            let mut engine: *mut CommandableEngine = (&raw mut (*client).engines
                as *mut CommandableEngine)
                .offset(i as isize) as *mut CommandableEngine;
            let mut needed: size_t = (if (*engine).independent_owned
                as ::core::ffi::c_int != 0
            {
                2 as ::core::ffi::c_uint
            } else {
                0 as ::core::ffi::c_uint
            })
                .wrapping_add(
                    (if (*engine).activated_by_client as ::core::ffi::c_int != 0
                        && !(*engine).initially_active
                    {
                        1 as ::core::ffi::c_uint
                    } else {
                        0 as ::core::ffi::c_uint
                    }),
                ) as size_t;
            if needed != 0
                && batch.count.wrapping_add(needed) > KRPC_CNANO_BATCH_MAX as size_t
                && !safe_flush_batch(client, &raw mut batch)
            {
                current_block = 6017496459565617652;
                break;
            }
            if (*engine).independent_owned {
                if !batch_add_object_float(
                    &raw mut batch,
                    PROC_ENGINE_SET_THROTTLE as ::core::ffi::c_int as uint32_t,
                    (*engine).object as krpc_object_t,
                    0.0f32,
                    0 as ::core::ffi::c_int != 0,
                ) {
                    safe_flush_batch(client, &raw mut batch);
                    current_block = 6017496459565617652;
                    break;
                } else if !batch_add_object_bool(
                    &raw mut batch,
                    PROC_ENGINE_SET_INDEPENDENT_THROTTLE as ::core::ffi::c_int
                        as uint32_t,
                    (*engine).object as krpc_object_t,
                    0 as ::core::ffi::c_int != 0,
                    0 as ::core::ffi::c_int != 0,
                ) {
                    safe_flush_batch(client, &raw mut batch);
                    current_block = 6017496459565617652;
                    break;
                } else {
                    (*engine).independent_owned = false_0 != 0;
                    (*engine).jet_assist_on = false_0 != 0;
                    (*engine).jet_throttle = 0.0f32;
                }
            }
            if (*engine).activated_by_client as ::core::ffi::c_int != 0
                && !(*engine).initially_active
            {
                if !batch_add_object_bool(
                    &raw mut batch,
                    PROC_ENGINE_SET_ACTIVE as ::core::ffi::c_int as uint32_t,
                    (*engine).object as krpc_object_t,
                    0 as ::core::ffi::c_int != 0,
                    0 as ::core::ffi::c_int != 0,
                ) {
                    safe_flush_batch(client, &raw mut batch);
                    current_block = 6017496459565617652;
                    break;
                } else {
                    (*engine).activated_by_client = false_0 != 0;
                }
            }
            i = i.wrapping_add(1);
        }
        match current_block {
            6017496459565617652 => {}
            _ => {
                safe_flush_batch(client, &raw mut batch);
            }
        }
    }
    (*client).cached_rcs = false_0 != 0;
    (*client).has_rcs = true_0 != 0;
    (*client).cached_sas = false_0 != 0;
    (*client).has_sas = true_0 != 0;
    (*client).cached_autopilot_engaged = false_0 != 0;
    (*client).has_autopilot_engaged = true_0 != 0;
    (*client).cached_command_throttle = 0.0f64;
    (*client).has_command_throttle = true_0 != 0;
    (*client).cached_command_wheel = 0.0f64;
    (*client).has_command_wheel = true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_safe(mut client: *mut KrpcCNanoClient) {
    if client.is_null() || !(*client).connected {
        return;
    }
    krpc_cnano_client_neutralize(client);
    set_paused_triplet(client, true_0 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_budget(
    mut client: *const KrpcCNanoClient,
) -> KrpcCNanoBudget {
    let mut budget: KrpcCNanoBudget = KrpcCNanoBudget {
        read_calls: 0 as ::core::ffi::c_uint,
        write_calls: 0,
        total_calls: 0,
        read_wire_requests: 0,
        write_wire_requests: 0,
        total_wire_requests: 0,
        last_read_seconds: 0.,
        last_apply_seconds: 0.,
        read_calls_per_second: 0.,
        apply_calls_per_second: 0.,
        read_wire_requests_per_second: 0.,
        apply_wire_requests_per_second: 0.,
        bytes_read: 0,
        bytes_written: 0,
    };
    if client.is_null() {
        return budget;
    }
    budget.read_calls = (*client).last_read_calls;
    budget.write_calls = (*client).last_apply_calls;
    budget.total_calls = (*client).total_rpc_calls;
    budget.read_wire_requests = (*client).last_read_wire_requests;
    budget.write_wire_requests = (*client).last_apply_wire_requests;
    budget.total_wire_requests = (*client).total_wire_requests;
    budget.last_read_seconds = (*client).last_read_seconds;
    budget.last_apply_seconds = (*client).last_apply_seconds;
    budget.read_calls_per_second = if (*client).last_read_seconds > 1e-6f64 {
        (*client).last_read_calls as ::core::ffi::c_double / (*client).last_read_seconds
    } else {
        0.0f64
    };
    budget.apply_calls_per_second = if (*client).last_apply_seconds > 1e-6f64 {
        (*client).last_apply_calls as ::core::ffi::c_double
            / (*client).last_apply_seconds
    } else {
        0.0f64
    };
    budget.read_wire_requests_per_second = if (*client).last_read_seconds > 1e-6f64 {
        (*client).last_read_wire_requests as ::core::ffi::c_double
            / (*client).last_read_seconds
    } else {
        0.0f64
    };
    budget.apply_wire_requests_per_second = if (*client).last_apply_seconds > 1e-6f64 {
        (*client).last_apply_wire_requests as ::core::ffi::c_double
            / (*client).last_apply_seconds
    } else {
        0.0f64
    };
    budget.bytes_read = if !(*client).connection.is_null() {
        krpc_cnano_transport_bytes_read((*client).connection)
    } else {
        0 as size_t
    };
    budget.bytes_written = if !(*client).connection.is_null() {
        krpc_cnano_transport_bytes_written((*client).connection)
    } else {
        0 as size_t
    };
    return budget;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_close(mut client: *mut KrpcCNanoClient) {
    if client.is_null() {
        return;
    }
    if (*client).connected {
        krpc_cnano_client_safe(client);
    }
    if !(*client).connection.is_null()
        && krpc_cnano_transport_is_open((*client).connection) as ::core::ffi::c_int != 0
    {
        krpc_close((*client).connection);
    }
    (*client).connected = false_0 != 0;
    free(client as *mut ::core::ffi::c_void);
}
unsafe extern "C" fn set_jet_assist(
    mut client: *mut KrpcCNanoClient,
    mut throttle: ::core::ffi::c_double,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut want: ::core::ffi::c_float = if throttle.is_finite() as i32 != 0 {
        clampd(throttle, 0.0f64, 1.0f64) as ::core::ffi::c_float
    } else {
        0.0f32
    };
    let mut i: size_t = 0 as size_t;
    while i < (*client).engine_count {
        let mut engine: *mut CommandableEngine = (&raw mut (*client).engines
            as *mut CommandableEngine)
            .offset(i as isize) as *mut CommandableEngine;
        if (*engine).airbreathing {
            let mut on: bool = want > 1e-3f32
                && (*client).body_has_oxygen as ::core::ffi::c_int != 0;
            let mut level: ::core::ffi::c_float = if on as ::core::ffi::c_int != 0 {
                want
            } else {
                0.0f32
            };
            if !((*engine).jet_assist_on as ::core::ffi::c_int
                == on as ::core::ffi::c_int
                && fabsf((*engine).jet_throttle - level) <= 0.02f32)
            {
                if on {
                    if !(*engine).jet_assist_on {
                        let mut active: bool = false_0 != 0;
                        if rpc_failed(
                            client,
                            krpc_SpaceCenter_Engine_Active(
                                (*client).connection,
                                &raw mut active,
                                (*engine).object,
                            ),
                            b"krpc_SpaceCenter_Engine_Active(client->connection, &active, engine->object)\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            error,
                            error_size,
                        ) {
                            return false_0 != 0;
                        }
                        if !active {
                            if rpc_failed(
                                client,
                                krpc_SpaceCenter_Engine_set_Active(
                                    (*client).connection,
                                    (*engine).object,
                                    1 as ::core::ffi::c_int != 0,
                                ),
                                b"krpc_SpaceCenter_Engine_set_Active(client->connection, engine->object, true)\0"
                                    as *const u8 as *const ::core::ffi::c_char,
                                error,
                                error_size,
                            ) {
                                return false_0 != 0;
                            }
                            (*engine).activated_by_client = true_0 != 0;
                        }
                        if rpc_failed(
                            client,
                            krpc_SpaceCenter_Engine_set_IndependentThrottle(
                                (*client).connection,
                                (*engine).object,
                                1 as ::core::ffi::c_int != 0,
                            ),
                            b"krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, true)\0"
                                as *const u8 as *const ::core::ffi::c_char,
                            error,
                            error_size,
                        ) {
                            return false_0 != 0;
                        }
                        (*engine).independent_owned = true_0 != 0;
                    }
                    if rpc_failed(
                        client,
                        krpc_SpaceCenter_Engine_set_Throttle(
                            (*client).connection,
                            (*engine).object,
                            level,
                        ),
                        b"krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, level)\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        error,
                        error_size,
                    ) {
                        return false_0 != 0;
                    }
                } else {
                    if rpc_failed(
                        client,
                        krpc_SpaceCenter_Engine_set_Throttle(
                            (*client).connection,
                            (*engine).object,
                            0.0f32,
                        ),
                        b"krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, 0.0f)\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        error,
                        error_size,
                    ) {
                        return false_0 != 0;
                    }
                    if rpc_failed(
                        client,
                        krpc_SpaceCenter_Engine_set_IndependentThrottle(
                            (*client).connection,
                            (*engine).object,
                            0 as ::core::ffi::c_int != 0,
                        ),
                        b"krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, false)\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        error,
                        error_size,
                    ) {
                        return false_0 != 0;
                    }
                    (*engine).independent_owned = false_0 != 0;
                }
                (*engine).jet_assist_on = on;
                (*engine).jet_throttle = level;
            }
        }
        i = i.wrapping_add(1);
    }
    return true_0 != 0;
}
unsafe extern "C" fn set_engine_throttle(
    mut client: *mut KrpcCNanoClient,
    mut throttle: ::core::ffi::c_double,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut powered: bool = throttle > 1e-4f64;
    let mut i: size_t = 0 as size_t;
    while i < (*client).engine_count {
        let mut engine: *mut CommandableEngine = (&raw mut (*client).engines
            as *mut CommandableEngine)
            .offset(i as isize) as *mut CommandableEngine;
        if !(*engine).airbreathing {
            if powered as ::core::ffi::c_int != 0 && !(*engine).selected {
                let mut active: bool = false_0 != 0;
                if krpc_SpaceCenter_Engine_Active(
                    (*client).connection,
                    &raw mut active,
                    (*engine).object,
                ) as ::core::ffi::c_int == KRPC_OK as ::core::ffi::c_int
                    && active as ::core::ffi::c_int != 0
                {
                    (*client).total_rpc_calls = (*client)
                        .total_rpc_calls
                        .wrapping_add(1);
                    (*client).total_wire_requests = (*client)
                        .total_wire_requests
                        .wrapping_add(1);
                    if rpc_failed(
                        client,
                        krpc_SpaceCenter_Engine_set_Active(
                            (*client).connection,
                            (*engine).object,
                            false_0 != 0,
                        ),
                        b"Excluded Engine.Active\0" as *const u8
                            as *const ::core::ffi::c_char,
                        error,
                        error_size,
                    ) {
                        return false_0 != 0;
                    }
                    fprintf(
                        stderr,
                        b"Burn engine exclusion: shut down non-selected engine %zu\n\0"
                            as *const u8 as *const ::core::ffi::c_char,
                        i,
                    );
                }
                if rpc_failed(
                    client,
                    krpc_SpaceCenter_Engine_set_IndependentThrottle(
                        (*client).connection,
                        (*engine).object,
                        true_0 != 0,
                    ),
                    b"Excluded Engine.IndependentThrottle\0" as *const u8
                        as *const ::core::ffi::c_char,
                    error,
                    error_size,
                ) as ::core::ffi::c_int != 0
                    || rpc_failed(
                        client,
                        krpc_SpaceCenter_Engine_set_Throttle(
                            (*client).connection,
                            (*engine).object,
                            0.0f32,
                        ),
                        b"Excluded Engine.Throttle\0" as *const u8
                            as *const ::core::ffi::c_char,
                        error,
                        error_size,
                    ) as ::core::ffi::c_int != 0
                {
                    return false_0 != 0;
                }
                (*engine).independent_owned = true_0 != 0;
            } else if powered as ::core::ffi::c_int != 0
                && (*engine).selected as ::core::ffi::c_int != 0
            {
                let mut active_0: bool = false_0 != 0;
                let mut fuel: bool = true_0 != 0;
                let mut e: krpc_error_t = krpc_SpaceCenter_Engine_Active(
                    (*client).connection,
                    &raw mut active_0,
                    (*engine).object,
                );
                (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
                (*client).total_wire_requests = (*client)
                    .total_wire_requests
                    .wrapping_add(1);
                if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int) {
                    e = krpc_SpaceCenter_Engine_HasFuel(
                        (*client).connection,
                        &raw mut fuel,
                        (*engine).object,
                    );
                    (*client).total_rpc_calls = (*client)
                        .total_rpc_calls
                        .wrapping_add(1);
                    (*client).total_wire_requests = (*client)
                        .total_wire_requests
                        .wrapping_add(1);
                    if !(e as ::core::ffi::c_int != KRPC_OK as ::core::ffi::c_int
                        || !fuel)
                    {
                        if !active_0 {
                            if rpc_failed(
                                client,
                                krpc_SpaceCenter_Engine_set_Active(
                                    (*client).connection,
                                    (*engine).object,
                                    true_0 != 0,
                                ),
                                b"Engine.Active\0" as *const u8
                                    as *const ::core::ffi::c_char,
                                error,
                                error_size,
                            ) {
                                return false_0 != 0;
                            }
                            (*engine).activated_by_client = true_0 != 0;
                        }
                        if rpc_failed(
                            client,
                            krpc_SpaceCenter_Engine_set_IndependentThrottle(
                                (*client).connection,
                                (*engine).object,
                                true_0 != 0,
                            ),
                            b"Engine.IndependentThrottle\0" as *const u8
                                as *const ::core::ffi::c_char,
                            error,
                            error_size,
                        ) as ::core::ffi::c_int != 0
                            || rpc_failed(
                                client,
                                krpc_SpaceCenter_Engine_set_Throttle(
                                    (*client).connection,
                                    (*engine).object,
                                    throttle as ::core::ffi::c_float,
                                ),
                                b"Engine.Throttle\0" as *const u8
                                    as *const ::core::ffi::c_char,
                                error,
                                error_size,
                            ) as ::core::ffi::c_int != 0
                        {
                            return false_0 != 0;
                        }
                        (*engine).independent_owned = true_0 != 0;
                    }
                }
            } else {
                if rpc_failed(
                    client,
                    krpc_SpaceCenter_Engine_set_Throttle(
                        (*client).connection,
                        (*engine).object,
                        0.0f32,
                    ),
                    b"krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, 0.0f)\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    error,
                    error_size,
                ) {
                    return false_0 != 0;
                }
                if rpc_failed(
                    client,
                    krpc_SpaceCenter_Engine_set_IndependentThrottle(
                        (*client).connection,
                        (*engine).object,
                        0 as ::core::ffi::c_int != 0,
                    ),
                    b"krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, false)\0"
                        as *const u8 as *const ::core::ffi::c_char,
                    error,
                    error_size,
                ) {
                    return false_0 != 0;
                }
                (*engine).independent_owned = false_0 != 0;
            }
        }
        i = i.wrapping_add(1);
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_direct_controls(
    mut client: *mut KrpcCNanoClient,
    mut pitch: ::core::ffi::c_double,
    mut roll: ::core::ffi::c_double,
    mut yaw: ::core::ffi::c_double,
    mut throttle: ::core::ffi::c_double,
    mut wheel_steering: ::core::ffi::c_double,
    mut rcs: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        set_error!(
            error,
            error_size,
            b"C-Nano client is not connected\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    (*client).pause_triplet_sent = false_0 != 0;
    let mut started: ::core::ffi::c_double = monotonic_seconds();
    let mut before: ::core::ffi::c_uint = (*client).total_rpc_calls;
    let mut wire_before: ::core::ffi::c_uint = (*client).total_wire_requests;
    pitch = clampd(
        pitch,
        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    roll = clampd(
        roll,
        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    yaw = clampd(
        yaw,
        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    throttle = clampd(
        throttle,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    wheel_steering = clampd(
        wheel_steering,
        -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut reset_orbital_profile: bool = (*client).orbital_autotune_profile_active;
    if reset_orbital_profile {
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_Reset((*client).connection, (*client).autopilot),
            b"krpc_SpaceCenter_AutoPilot_Reset(client->connection,client->autopilot)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).cached_autopilot_engaged = false_0 != 0;
        (*client).has_autopilot_engaged = true_0 != 0;
        (*client).orbital_autotune_profile_active = false_0 != 0;
    }
    let mut disengage_autopilot: bool = !reset_orbital_profile
        && (!(*client).has_autopilot_engaged
            || (*client).cached_autopilot_engaged as ::core::ffi::c_int != 0);
    let mut disable_sas: bool = !(*client).has_sas
        || (*client).cached_sas as ::core::ffi::c_int != 0;
    let mut change_throttle: bool = !(*client).has_command_throttle
        || fabs((*client).cached_command_throttle - throttle) > 1e-4f64;
    let mut change_wheel: bool = !(*client).has_command_wheel
        || fabs((*client).cached_command_wheel - wheel_steering) > 1e-3f64;
    let mut change_rcs: bool = !(*client).has_rcs
        || (*client).cached_rcs as ::core::ffi::c_int != rcs as ::core::ffi::c_int;
    let mut batch: ClientBatch = ClientBatch {
        items: [
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
            BatchItem {
                call: krpc_call_t {
                    message: _krpc_schema_ProcedureCall {
                        arguments: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                        service_id: 0,
                        procedure_id: 0,
                    },
                    numArguments: 0,
                    arguments: ::core::ptr::null_mut::<krpc_argument_t>(),
                },
                arguments: [krpc_argument_t {
                    message: _krpc_schema_Argument {
                        position: 0,
                        value: pb_callback_s {
                            funcs: C2RustUnnamed { decode: None },
                            arg: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                        },
                    },
                }; 3],
                object_arguments: [0; 3],
                uint_arguments: [0; 3],
                float_arguments: [0.; 3],
                double_arguments: [0.; 3],
                bool_arguments: [false; 3],
                kind: BATCH_NONE,
                target: ::core::ptr::null_mut::<::core::ffi::c_void>(),
                required: false,
            },
        ],
        count: 0,
    };
    if disengage_autopilot {
        if !batch_add_object_bool(
            &raw mut batch,
            PROC_AUTOPILOT_SET_ENGAGED as ::core::ffi::c_int as uint32_t,
            (*client).autopilot as krpc_object_t,
            0 as ::core::ffi::c_int != 0,
            1 as ::core::ffi::c_int != 0,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not construct direct-control C-Nano batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
    }
    if disable_sas {
        if !batch_add_object_bool(
            &raw mut batch,
            PROC_CONTROL_SET_SAS as ::core::ffi::c_int as uint32_t,
            (*client).control as krpc_object_t,
            0 as ::core::ffi::c_int != 0,
            1 as ::core::ffi::c_int != 0,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not construct direct-control C-Nano batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
    }
    if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_PITCH as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        pitch as ::core::ffi::c_float,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct direct-control C-Nano batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_ROLL as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        roll as ::core::ffi::c_float,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct direct-control C-Nano batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !batch_add_object_float(
        &raw mut batch,
        PROC_CONTROL_SET_YAW as ::core::ffi::c_int as uint32_t,
        (*client).control as krpc_object_t,
        yaw as ::core::ffi::c_float,
        1 as ::core::ffi::c_int != 0,
    ) {
        set_error!(
            error,
            error_size,
            b"Could not construct direct-control C-Nano batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if change_throttle {
        if !batch_add_object_float(
            &raw mut batch,
            PROC_CONTROL_SET_THROTTLE as ::core::ffi::c_int as uint32_t,
            (*client).control as krpc_object_t,
            0.0f32,
            1 as ::core::ffi::c_int != 0,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not construct direct-control C-Nano batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
    }
    if change_wheel {
        if !batch_add_object_float(
            &raw mut batch,
            PROC_CONTROL_SET_WHEEL_STEERING as ::core::ffi::c_int as uint32_t,
            (*client).control as krpc_object_t,
            wheel_steering as ::core::ffi::c_float,
            1 as ::core::ffi::c_int != 0,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not construct direct-control C-Nano batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
    }
    if change_rcs {
        if !batch_add_object_bool(
            &raw mut batch,
            PROC_CONTROL_SET_RCS as ::core::ffi::c_int as uint32_t,
            (*client).control as krpc_object_t,
            rcs,
            1 as ::core::ffi::c_int != 0,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not construct direct-control C-Nano batch\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
    }
    if !batch_run(
        client,
        &raw mut batch,
        b"direct control\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if disengage_autopilot {
        (*client).cached_autopilot_engaged = false_0 != 0;
        (*client).has_autopilot_engaged = true_0 != 0;
    }
    if disable_sas {
        (*client).cached_sas = false_0 != 0;
        (*client).has_sas = true_0 != 0;
    }
    if change_throttle {
        if !set_engine_throttle(client, throttle, error, error_size) {
            return false_0 != 0;
        }
        (*client).cached_command_throttle = throttle;
        (*client).has_command_throttle = true_0 != 0;
    }
    if !set_jet_assist(client, 0.0f64, error, error_size) {
        return false_0 != 0;
    }
    if change_wheel {
        (*client).cached_command_wheel = wheel_steering;
        (*client).has_command_wheel = true_0 != 0;
    }
    if change_rcs {
        (*client).cached_rcs = rcs;
        (*client).has_rcs = true_0 != 0;
    }
    (*client).last_apply_calls = (*client).total_rpc_calls.wrapping_sub(before);
    (*client).last_apply_wire_requests = (*client)
        .total_wire_requests
        .wrapping_sub(wire_before);
    (*client).last_apply_seconds = monotonic_seconds() - started;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_autopilot(
    mut client: *mut KrpcCNanoClient,
    mut command: *const GuidanceCommand,
    mut inertial_up_reference: *const Vector3,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected || command.is_null() {
        set_error!(
            error,
            error_size,
            b"C-Nano autopilot state is invalid\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    (*client).pause_triplet_sent = false_0 != 0;
    let mut started: ::core::ffi::c_double = monotonic_seconds();
    let mut before: ::core::ffi::c_uint = (*client).total_rpc_calls;
    let mut wire_before: ::core::ffi::c_uint = (*client).total_wire_requests;
    if !(*client).has_sas || (*client).cached_sas as ::core::ffi::c_int != 0 {
        if rpc_failed(
            client,
            krpc_SpaceCenter_Control_set_SAS(
                (*client).connection,
                (*client).control,
                0 as ::core::ffi::c_int != 0,
            ),
            b"krpc_SpaceCenter_Control_set_SAS(client->connection, client->control, false)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).cached_sas = false_0 != 0;
        (*client).has_sas = true_0 != 0;
    }
    let mut want_rcs: bool = (*command).control_profile as ::core::ffi::c_uint
        == PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint;
    if !(*client).has_rcs
        || (*client).cached_rcs as ::core::ffi::c_int != want_rcs as ::core::ffi::c_int
    {
        if rpc_failed(
            client,
            krpc_SpaceCenter_Control_set_RCS(
                (*client).connection,
                (*client).control,
                want_rcs,
            ),
            b"krpc_SpaceCenter_Control_set_RCS(client->connection, client->control, want_rcs)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).cached_rcs = want_rcs;
        (*client).has_rcs = true_0 != 0;
    }
    let mut frame: krpc_SpaceCenter_ReferenceFrame_t = if (*command)
        .use_inertial_direction as ::core::ffi::c_int != 0
    {
        (*client).body_nonrotating
    } else {
        (*client).surface_frame
    };
    let mut want_orbital_tuning: bool = (*command).control_profile as ::core::ffi::c_uint
        == PROFILE_ORBITAL as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut engaging_orbital: bool = want_orbital_tuning as ::core::ffi::c_int != 0
        && (*command).autopilot_engaged as ::core::ffi::c_int != 0
        && (!(*client).has_autopilot_engaged || !(*client).cached_autopilot_engaged);
    if engaging_orbital as ::core::ffi::c_int != 0
        && !(*client).orbital_autotune_profile_active
    {
        let mut time_to_peak: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
            e0: 0.8f64,
            e1: 5.0f64,
            e2: 0.8f64,
        };
        let mut overshoot: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
            e0: 0.05f64,
            e1: 0.01f64,
            e2: 0.05f64,
        };
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_AutoTune(
                (*client).connection,
                (*client).autopilot,
                1 as ::core::ffi::c_int != 0,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_AutoTune(client->connection, client->autopilot, true)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_TimeToPeak(
                (*client).connection,
                (*client).autopilot,
                &raw mut time_to_peak,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_TimeToPeak(client->connection, client->autopilot, &time_to_peak)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_Overshoot(
                (*client).connection,
                (*client).autopilot,
                &raw mut overshoot,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_Overshoot(client->connection, client->autopilot, &overshoot)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_SoftStartTime(
                (*client).connection,
                (*client).autopilot,
                0.5f64,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_SoftStartTime(client->connection, client->autopilot, 0.5)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).orbital_autotune_profile_active = true_0 != 0;
    } else if !want_orbital_tuning
        && (*client).orbital_autotune_profile_active as ::core::ffi::c_int != 0
    {
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_Reset((*client).connection, (*client).autopilot),
            b"krpc_SpaceCenter_AutoPilot_Reset(client->connection, client->autopilot)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).cached_autopilot_engaged = false_0 != 0;
        (*client).has_autopilot_engaged = true_0 != 0;
        (*client).orbital_autotune_profile_active = false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_AutoPilot_set_ReferenceFrame(
            (*client).connection,
            (*client).autopilot,
            frame,
        ),
        b"krpc_SpaceCenter_AutoPilot_set_ReferenceFrame(client->connection, client->autopilot, frame)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if (*command).use_inertial_direction {
        let mut target_nose: Vector3 = (*command).inertial_direction;
        let mut target_roof: Vector3 = if !inertial_up_reference.is_null() {
            *inertial_up_reference
        } else {
            v3(
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            )
        };
        if (*command).thrust_alignment as ::core::ffi::c_int != 0
            && (*client).burn_geometry_valid as ::core::ffi::c_int != 0
        {
            thrust_alignment_attitude(
                (*client).burn_thrust_axis_body,
                target_nose,
                target_roof,
                &raw mut target_nose,
                &raw mut target_roof,
            );
        }
        let mut direction: krpc_tuple_double_double_double_t = canonical_to_tuple(
            target_nose,
        );
        if !inertial_up_reference.is_null() {
            let mut normalized_up: Vector3 = vnorm(
                target_roof,
                v3(
                    1 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                ),
            );
            let mut up: krpc_tuple_double_double_double_t = canonical_to_tuple(
                normalized_up,
            );
            if rpc_failed(
                client,
                krpc_SpaceCenter_AutoPilot_SetDirectionAndUp(
                    (*client).connection,
                    (*client).autopilot,
                    &raw mut direction,
                    &raw mut up,
                    (*command).target_roll as ::core::ffi::c_float,
                ),
                b"krpc_SpaceCenter_AutoPilot_SetDirectionAndUp(client->connection, client->autopilot, &direction, &up, (float)command->target_roll)\0"
                    as *const u8 as *const ::core::ffi::c_char,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
        } else {
            if rpc_failed(
                client,
                krpc_SpaceCenter_AutoPilot_set_TargetDirection(
                    (*client).connection,
                    (*client).autopilot,
                    &raw mut direction,
                ),
                b"krpc_SpaceCenter_AutoPilot_set_TargetDirection(client->connection, client->autopilot, &direction)\0"
                    as *const u8 as *const ::core::ffi::c_char,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
            if rpc_failed(
                client,
                krpc_SpaceCenter_AutoPilot_set_TargetRoll(
                    (*client).connection,
                    (*client).autopilot,
                    (*command).target_roll as ::core::ffi::c_float,
                ),
                b"krpc_SpaceCenter_AutoPilot_set_TargetRoll(client->connection, client->autopilot, (float)command->target_roll)\0"
                    as *const u8 as *const ::core::ffi::c_char,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
        }
    } else {
        let mut pitch: ::core::ffi::c_double = clampd(
            (*command).target_pitch,
            -89.9f64,
            89.9f64,
        ) * DEG2RAD;
        let mut heading: ::core::ffi::c_double = norm_deg((*command).target_heading)
            * DEG2RAD;
        let mut cp: ::core::ffi::c_double = cos(pitch);
        let mut direction_0: krpc_tuple_double_double_double_t = krpc_tuple_double_double_double_s {
            e0: sin(pitch),
            e1: cp * cos(heading),
            e2: cp * sin(heading),
        };
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_TargetDirection(
                (*client).connection,
                (*client).autopilot,
                &raw mut direction_0,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_TargetDirection(client->connection, client->autopilot, &direction)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_TargetRoll(
                (*client).connection,
                (*client).autopilot,
                (*command).target_roll as ::core::ffi::c_float,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_TargetRoll(client->connection, client->autopilot, (float)command->target_roll)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
    }
    if !(*client).has_autopilot_engaged
        || (*client).cached_autopilot_engaged as ::core::ffi::c_int
            != (*command).autopilot_engaged as ::core::ffi::c_int
    {
        if rpc_failed(
            client,
            krpc_SpaceCenter_AutoPilot_set_Engaged(
                (*client).connection,
                (*client).autopilot,
                (*command).autopilot_engaged,
            ),
            b"krpc_SpaceCenter_AutoPilot_set_Engaged(client->connection, client->autopilot, command->autopilot_engaged)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        (*client).cached_autopilot_engaged = (*command).autopilot_engaged;
        (*client).has_autopilot_engaged = true_0 != 0;
    }
    let mut throttle: ::core::ffi::c_double = clampd(
        (*command).target_throttle,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    if !(*client).has_command_throttle
        || fabs((*client).cached_command_throttle - throttle) > 1e-4f64
    {
        if rpc_failed(
            client,
            krpc_SpaceCenter_Control_set_Throttle(
                (*client).connection,
                (*client).control,
                0.0f32,
            ),
            b"krpc_SpaceCenter_Control_set_Throttle(client->connection, client->control, 0.0f)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if !set_engine_throttle(client, throttle, error, error_size) {
            return false_0 != 0;
        }
        (*client).cached_command_throttle = throttle;
        (*client).has_command_throttle = true_0 != 0;
    }
    if !set_jet_assist(client, (*command).jet_assist_throttle, error, error_size) {
        return false_0 != 0;
    }
    (*client).cached_sas = false_0 != 0;
    (*client).has_sas = true_0 != 0;
    (*client).last_apply_calls = (*client).total_rpc_calls.wrapping_sub(before);
    (*client).last_apply_wire_requests = (*client)
        .total_wire_requests
        .wrapping_sub(wire_before);
    (*client).last_apply_seconds = monotonic_seconds() - started;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_surface_allocation(
    mut client: *mut KrpcCNanoClient,
    mut allocation: KrpcSurfaceAllocation,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        return false_0 != 0;
    }
    if !(*client).surface_roles_classified {
        return true_0 != 0;
    }
    if (*client).has_applied_surface_allocation as ::core::ffi::c_int != 0
        && (*client).applied_surface_allocation as ::core::ffi::c_uint
            == allocation as ::core::ffi::c_uint
    {
        return true_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*client).surface_count {
        let mut surface: *const CommandableSurface = (&raw mut (*client).surfaces
            as *mut CommandableSurface)
            .offset(i as isize) as *mut CommandableSurface;
        let mut pitch: bool = true_0 != 0;
        let mut roll: bool = true_0 != 0;
        let mut yaw: bool = true_0 != 0;
        if allocation as ::core::ffi::c_uint
            != KRPC_SURFACE_ALLOCATION_MIXED as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            pitch = (*surface).role as ::core::ffi::c_uint
                == KRPC_SURFACE_ROLE_ELEVATOR as ::core::ffi::c_int
                    as ::core::ffi::c_uint
                || allocation as ::core::ffi::c_uint
                    == KRPC_SURFACE_ALLOCATION_SEPARATED_FLAP_PITCH as ::core::ffi::c_int
                        as ::core::ffi::c_uint
                    && (*surface).role as ::core::ffi::c_uint
                        == KRPC_SURFACE_ROLE_BODY_FLAP as ::core::ffi::c_int
                            as ::core::ffi::c_uint;
            roll = (*surface).role as ::core::ffi::c_uint
                == KRPC_SURFACE_ROLE_AILERON as ::core::ffi::c_int
                    as ::core::ffi::c_uint;
            yaw = (*surface).role as ::core::ffi::c_uint
                == KRPC_SURFACE_ROLE_AILERON as ::core::ffi::c_int as ::core::ffi::c_uint
                || (*surface).role as ::core::ffi::c_uint
                    == KRPC_SURFACE_ROLE_RUDDER as ::core::ffi::c_int
                        as ::core::ffi::c_uint;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_ControlSurface_set_PitchEnabled(
                (*client).connection,
                (*surface).object,
                pitch,
            ),
            b"krpc_SpaceCenter_ControlSurface_set_PitchEnabled(client->connection, surface->object, pitch)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_ControlSurface_set_RollEnabled(
                (*client).connection,
                (*surface).object,
                roll,
            ),
            b"krpc_SpaceCenter_ControlSurface_set_RollEnabled(client->connection, surface->object, roll)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_ControlSurface_set_YawEnabled(
                (*client).connection,
                (*surface).object,
                yaw,
            ),
            b"krpc_SpaceCenter_ControlSurface_set_YawEnabled(client->connection, surface->object, yaw)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    let mut separated: bool = allocation as ::core::ffi::c_uint
        != KRPC_SURFACE_ALLOCATION_MIXED as ::core::ffi::c_int as ::core::ffi::c_uint;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*client).fin_module_count {
        if rpc_failed(
            client,
            krpc_SpaceCenter_PartField_set_BoolValue(
                (*client).connection,
                (*client).fin_modules[i_0 as usize].ignore_pitch,
                separated,
            ),
            b"krpc_SpaceCenter_PartField_set_BoolValue(client->connection, client->fin_modules[i].ignore_pitch, separated)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_PartField_set_BoolValue(
                (*client).connection,
                (*client).fin_modules[i_0 as usize].ignore_roll,
                separated,
            ),
            b"krpc_SpaceCenter_PartField_set_BoolValue(client->connection, client->fin_modules[i].ignore_roll, separated)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_PartField_set_BoolValue(
                (*client).connection,
                (*client).fin_modules[i_0 as usize].ignore_yaw,
                0 as ::core::ffi::c_int != 0,
            ),
            b"krpc_SpaceCenter_PartField_set_BoolValue(client->connection, client->fin_modules[i].ignore_yaw, false)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        i_0 = i_0.wrapping_add(1);
    }
    fprintf(
        stderr,
        b"Surface allocation applied: %s\n\0" as *const u8 as *const ::core::ffi::c_char,
        if allocation as ::core::ffi::c_uint
            == KRPC_SURFACE_ALLOCATION_MIXED as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            b"mixed\0" as *const u8 as *const ::core::ffi::c_char
        } else if allocation as ::core::ffi::c_uint
            == KRPC_SURFACE_ALLOCATION_SEPARATED as ::core::ffi::c_int
                as ::core::ffi::c_uint
        {
            b"separated (body flap off)\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"separated + body flap pitch\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
    (*client).applied_surface_allocation = allocation;
    (*client).has_applied_surface_allocation = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_speedbrake(
    mut client: *mut KrpcCNanoClient,
    mut fraction: ::core::ffi::c_double,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        return false_0 != 0;
    }
    if (*client).fin_module_count == 0 as size_t {
        return true_0 != 0;
    }
    fraction = if fraction.is_finite() as i32 != 0 {
        clampd(fraction, 0.0f64, 1.0f64)
    } else {
        0.0f64
    };
    fraction = round(fraction * 50.0f64) / 50.0f64;
    if (*client).has_applied_speedbrake as ::core::ffi::c_int != 0
        && fabs((*client).applied_speedbrake_fraction - fraction) < 1e-9f64
    {
        return true_0 != 0;
    }
    let mut deploy: bool = fraction > 0.0f64;
    let mut angle: ::core::ffi::c_float = (fraction * (*client).speedbrake_max_angle_deg)
        as ::core::ffi::c_float;
    let mut i: size_t = 0 as size_t;
    while i < (*client).fin_module_count {
        if rpc_failed(
            client,
            krpc_SpaceCenter_PartField_set_FloatValue(
                (*client).connection,
                (*client).fin_modules[i as usize].deploy_angle,
                angle,
            ),
            b"krpc_SpaceCenter_PartField_set_FloatValue(client->connection, client->fin_modules[i].deploy_angle, angle)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        if rpc_failed(
            client,
            krpc_SpaceCenter_PartField_set_BoolValue(
                (*client).connection,
                (*client).fin_modules[i as usize].deploy,
                deploy,
            ),
            b"krpc_SpaceCenter_PartField_set_BoolValue(client->connection, client->fin_modules[i].deploy, deploy)\0"
                as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        ) {
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    (*client).applied_speedbrake_fraction = fraction;
    (*client).has_applied_speedbrake = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_surface_layout(
    mut client: *const KrpcCNanoClient,
    mut buffer: *mut ::core::ffi::c_char,
    mut buffer_size: size_t,
) -> *const ::core::ffi::c_char {
    if buffer.is_null() || buffer_size == 0 as size_t {
        return b"\0" as *const u8 as *const ::core::ffi::c_char;
    }
    *buffer.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
        as ::core::ffi::c_char;
    if client.is_null() || !(*client).surface_roles_classified {
        return buffer;
    }
    let mut counts: [::core::ffi::c_uint; 5] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_uint,
        0,
        0,
        0,
        0,
    ];
    let mut i: size_t = 0 as size_t;
    while i < (*client).surface_count {
        if ((*client).surfaces[i as usize].role as ::core::ffi::c_uint)
            < 5 as ::core::ffi::c_uint
        {
            counts[(*client).surfaces[i as usize].role as usize] = counts[(*client)
                    .surfaces[i as usize]
                    .role as usize]
                .wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    snprintf(
        buffer,
        buffer_size,
        b"E%u A%u R%u B%u\0" as *const u8 as *const ::core::ffi::c_char,
        counts[KRPC_SURFACE_ROLE_ELEVATOR as ::core::ffi::c_int as usize],
        counts[KRPC_SURFACE_ROLE_AILERON as ::core::ffi::c_int as usize],
        counts[KRPC_SURFACE_ROLE_RUDDER as ::core::ffi::c_int as usize],
        counts[KRPC_SURFACE_ROLE_BODY_FLAP as ::core::ffi::c_int as usize],
    );
    return buffer;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_gear(
    mut client: *mut KrpcCNanoClient,
    mut value: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        return false_0 != 0;
    }
    if (*client).has_gear as ::core::ffi::c_int != 0
        && (*client).cached_gear as ::core::ffi::c_int == value as ::core::ffi::c_int
    {
        return true_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_Control_set_Gear(
            (*client).connection,
            (*client).control,
            value,
        ),
        b"krpc_SpaceCenter_Control_set_Gear(client->connection, client->control, value)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*client).cached_gear = value;
    (*client).has_gear = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_brakes(
    mut client: *mut KrpcCNanoClient,
    mut value: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        return false_0 != 0;
    }
    if (*client).has_brakes as ::core::ffi::c_int != 0
        && (*client).cached_brakes as ::core::ffi::c_int == value as ::core::ffi::c_int
    {
        return true_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_Control_set_Brakes(
            (*client).connection,
            (*client).control,
            value,
        ),
        b"krpc_SpaceCenter_Control_set_Brakes(client->connection, client->control, value)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*client).cached_brakes = value;
    (*client).has_brakes = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_airbrakes(
    mut client: *mut KrpcCNanoClient,
    mut group: ::core::ffi::c_uint,
    mut value: bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        return false_0 != 0;
    }
    if (*client).has_airbrakes as ::core::ffi::c_int != 0
        && (*client).cached_airbrakes as ::core::ffi::c_int
            == value as ::core::ffi::c_int
    {
        return true_0 != 0;
    }
    if group < 1 as ::core::ffi::c_uint {
        group = 1 as ::core::ffi::c_uint;
    }
    if group > 10 as ::core::ffi::c_uint {
        group = 10 as ::core::ffi::c_uint;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_Control_SetActionGroup(
            (*client).connection,
            (*client).control,
            group as uint32_t,
            value,
        ),
        b"krpc_SpaceCenter_Control_SetActionGroup(client->connection, client->control, group, value)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*client).cached_airbrakes = value;
    (*client).has_airbrakes = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_speed_mode(
    mut client: *mut KrpcCNanoClient,
    mut mode: NavballSpeedMode,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected
        || mode as ::core::ffi::c_uint
            == SPEED_UNCHANGED as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return true_0 != 0;
    }
    if (*client).has_speed_mode as ::core::ffi::c_int != 0
        && (*client).cached_speed_mode as ::core::ffi::c_uint
            == mode as ::core::ffi::c_uint
    {
        return true_0 != 0;
    }
    let mut native: krpc_SpaceCenter_SpeedMode_t = (if mode as ::core::ffi::c_uint
        == SPEED_ORBIT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        KRPC_SPACECENTER_SPEEDMODE_ORBIT as ::core::ffi::c_int
    } else {
        KRPC_SPACECENTER_SPEEDMODE_SURFACE as ::core::ffi::c_int
    }) as krpc_SpaceCenter_SpeedMode_t;
    if rpc_failed(
        client,
        krpc_SpaceCenter_Control_set_SpeedMode(
            (*client).connection,
            (*client).control,
            native,
        ),
        b"krpc_SpaceCenter_Control_set_SpeedMode(client->connection, client->control, native)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    (*client).cached_speed_mode = mode;
    (*client).has_speed_mode = true_0 != 0;
    return true_0 != 0;
}
static mut k_stock_rails_rate_by_factor: [::core::ffi::c_double; 8] = [
    1.0f64,
    5.0f64,
    10.0f64,
    50.0f64,
    100.0f64,
    1000.0f64,
    10000.0f64,
    100000.0f64,
];
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_deploy_parachutes(
    mut client: *mut KrpcCNanoClient,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_Control_set_Parachutes(
            (*client).connection,
            (*client).control,
            1 as ::core::ffi::c_int != 0,
        ),
        b"krpc_SpaceCenter_Control_set_Parachutes(client->connection,client->control,true)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_shutdown_burn_engines(
    mut client: *mut KrpcCNanoClient,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() {
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*client).engine_count {
        let mut engine: *mut CommandableEngine = (&raw mut (*client).engines
            as *mut CommandableEngine)
            .offset(i as isize) as *mut CommandableEngine;
        if !((*engine).airbreathing as ::core::ffi::c_int != 0 || !(*engine).selected) {
            if rpc_failed(
                client,
                krpc_SpaceCenter_Engine_set_Throttle(
                    (*client).connection,
                    (*engine).object,
                    0.0f32,
                ),
                b"krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, 0.0f)\0"
                    as *const u8 as *const ::core::ffi::c_char,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
            if rpc_failed(
                client,
                krpc_SpaceCenter_Engine_set_Active(
                    (*client).connection,
                    (*engine).object,
                    0 as ::core::ffi::c_int != 0,
                ),
                b"krpc_SpaceCenter_Engine_set_Active(client->connection, engine->object, false)\0"
                    as *const u8 as *const ::core::ffi::c_char,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
            (*engine).activated_by_client = false_0 != 0;
            fprintf(
                stderr,
                b"Deorbit complete: shut down burn engine %zu\n\0" as *const u8
                    as *const ::core::ffi::c_char,
                i,
            );
        }
        i = i.wrapping_add(1);
    }
    return true_0 != 0;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL_0: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
unsafe extern "C" fn warp_decision_from_rates(
    mut remaining_ut: ::core::ffi::c_double,
    mut response_horizon_seconds: ::core::ffi::c_double,
    mut maximum_factor: ::core::ffi::c_int,
    mut rates: *const ::core::ffi::c_double,
    mut rate_count: size_t,
    mut decision: *mut KrpcCNanoWarpDecision,
) -> bool {
    if decision.is_null() || rates.is_null() || rate_count == 0 as size_t
        || remaining_ut.is_finite() as i32 == 0 || remaining_ut < 0.0f64
        || response_horizon_seconds.is_finite() as i32 == 0
        || response_horizon_seconds <= 0.0f64 || maximum_factor < 0 as ::core::ffi::c_int
        || maximum_factor as size_t >= rate_count
    {
        return false_0 != 0;
    }
    *decision = KrpcCNanoWarpDecision {
        factor: 0 as int32_t,
        predicted_rate: *rates.offset(0 as ::core::ffi::c_int as isize),
        response_horizon_seconds: response_horizon_seconds,
        stopping_margin_ut: remaining_ut,
    };
    let mut factor: ::core::ffi::c_int = maximum_factor;
    while factor >= 1 as ::core::ffi::c_int {
        let mut rate: ::core::ffi::c_double = *rates.offset(factor as isize);
        if rate.is_finite() as i32 == 0 || rate <= 0.0f64 {
            return false_0 != 0;
        }
        let mut margin: ::core::ffi::c_double = remaining_ut
            - rate * response_horizon_seconds;
        if margin >= 0.0f64 {
            (*decision).factor = factor as int32_t;
            (*decision).predicted_rate = rate;
            (*decision).stopping_margin_ut = margin;
            break;
        } else {
            factor -= 1;
        }
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_warp_decision(
    mut remaining_ut: ::core::ffi::c_double,
    mut response_horizon_seconds: ::core::ffi::c_double,
    mut maximum_factor: ::core::ffi::c_int,
    mut decision: *mut KrpcCNanoWarpDecision,
) -> bool {
    return warp_decision_from_rates(
        remaining_ut,
        response_horizon_seconds,
        maximum_factor,
        &raw const k_stock_rails_rate_by_factor as *const ::core::ffi::c_double,
        (::core::mem::size_of::<[::core::ffi::c_double; 8]>() as size_t)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as size_t),
        decision,
    );
}
unsafe extern "C" fn set_rails_warp_factor_observed(
    mut client: *mut KrpcCNanoClient,
    mut requested: int32_t,
    mut observed: *mut int32_t,
    mut latency_seconds: *mut ::core::ffi::c_double,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || observed.is_null() || latency_seconds.is_null() {
        return false_0 != 0;
    }
    if !krpc_cnano_transport_begin_deadline((*client).connection) {
        set_error!(
            error,
            error_size,
            b"Could not start the configured rails-warp response deadline\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut started: ::core::ffi::c_double = monotonic_seconds();
    let mut set_result: krpc_error_t = krpc_SpaceCenter_set_RailsWarpFactor(
        (*client).connection,
        requested,
    );
    if rpc_failed(
        client,
        set_result,
        b"SpaceCenter.RailsWarpFactor(set)\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        krpc_cnano_transport_end_deadline((*client).connection);
        return false_0 != 0;
    }
    let mut read_result: krpc_error_t = krpc_SpaceCenter_RailsWarpFactor(
        (*client).connection,
        observed,
    );
    *latency_seconds = monotonic_seconds() - started;
    krpc_cnano_transport_end_deadline((*client).connection);
    if rpc_failed(
        client,
        read_result,
        b"SpaceCenter.RailsWarpFactor(readback)\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    let mut rate_count: size_t = (::core::mem::size_of::<[::core::ffi::c_double; 8]>()
        as size_t)
        .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as size_t);
    if *observed < 0 as int32_t || *observed as size_t >= rate_count
        || requested == 0 as int32_t && *observed != 0 as int32_t
        || requested > 0 as int32_t && *observed > requested
    {
        set_error!(
            error,
            error_size,
            b"KSP returned invalid rails-warp factor %d for request %d\0" as *const u8
                as *const ::core::ffi::c_char,
            *observed as ::core::ffi::c_int,
            requested as ::core::ffi::c_int,
        );
        return false_0 != 0;
    }
    if (*latency_seconds).is_finite() as i32 == 0 || *latency_seconds <= 0.0f64 {
        set_error!(
            error,
            error_size,
            b"Could not measure rails-warp command/response latency\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn read_warp_state(
    mut client: *mut KrpcCNanoClient,
    mut state: *mut CNanoWarpState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut attempt: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while attempt < 3 as ::core::ffi::c_int {
        if read_warp_state_once(client, state, error, error_size) {
            return true_0 != 0;
        }
        if client.is_null() || state.is_null() {
            return false_0 != 0;
        }
        let mut pause: timespec = timespec {
            tv_sec: 0 as __time_t,
            tv_nsec: 200000000 as __syscall_slong_t,
        };
        nanosleep(&raw mut pause, ::core::ptr::null_mut::<timespec>());
        attempt += 1;
    }
    return false_0 != 0;
}
unsafe extern "C" fn read_warp_state_once(
    mut client: *mut KrpcCNanoClient,
    mut state: *mut CNanoWarpState,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || state.is_null() {
        return false_0 != 0;
    }
    if !krpc_cnano_transport_begin_deadline((*client).connection) {
        set_error!(
            error,
            error_size,
            b"Could not start the configured rails-warp state deadline\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut started: ::core::ffi::c_double = monotonic_seconds();
    let mut rate: ::core::ffi::c_float = ::core::f32::NAN;
    let mut code: krpc_error_t = krpc_SpaceCenter_UT(
        (*client).connection,
        &raw mut (*state).ut,
    );
    if rpc_failed(
        client,
        code,
        b"SpaceCenter.UT(warp)\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        krpc_cnano_transport_end_deadline((*client).connection);
        return false_0 != 0;
    }
    code = krpc_SpaceCenter_RailsWarpFactor(
        (*client).connection,
        &raw mut (*state).factor,
    );
    if rpc_failed(
        client,
        code,
        b"SpaceCenter.RailsWarpFactor(warp-state)\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        krpc_cnano_transport_end_deadline((*client).connection);
        return false_0 != 0;
    }
    code = krpc_SpaceCenter_WarpRate((*client).connection, &raw mut rate);
    (*state).latency_seconds = monotonic_seconds() - started;
    krpc_cnano_transport_end_deadline((*client).connection);
    if rpc_failed(
        client,
        code,
        b"SpaceCenter.WarpRate\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    let mut rate_count: size_t = (::core::mem::size_of::<[::core::ffi::c_double; 8]>()
        as size_t)
        .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as size_t);
    (*state).rate = rate as ::core::ffi::c_double;
    if (*state).ut.is_finite() as i32 == 0 || (*state).factor < 0 as int32_t
        || (*state).factor as size_t >= rate_count
        || (*state).rate.is_finite() as i32 == 0 || (*state).rate <= 0.0f64
        || (*state).latency_seconds.is_finite() as i32 == 0
        || (*state).latency_seconds <= 0.0f64
    {
        set_error!(
            error,
            error_size,
            b"KSP returned invalid rails-warp state (ut=%.9g factor=%d rate=%.9g latency=%.9g)\0"
                as *const u8 as *const ::core::ffi::c_char,
            (*state).ut,
            (*state).factor as ::core::ffi::c_int,
            (*state).rate,
            (*state).latency_seconds,
        );
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn read_maximum_rails_factor(
    mut client: *mut KrpcCNanoClient,
    mut maximum_factor: *mut int32_t,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !krpc_cnano_transport_begin_deadline((*client).connection) {
        set_error!(
            error,
            error_size,
            b"Could not start the maximum-rails-factor response deadline\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut code: krpc_error_t = krpc_SpaceCenter_MaximumRailsWarpFactor(
        (*client).connection,
        maximum_factor,
    );
    krpc_cnano_transport_end_deadline((*client).connection);
    if rpc_failed(
        client,
        code,
        b"SpaceCenter.MaximumRailsWarpFactor\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    let mut rate_count: size_t = (::core::mem::size_of::<[::core::ffi::c_double; 8]>()
        as size_t)
        .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as size_t);
    if *maximum_factor < 0 as int32_t || *maximum_factor as size_t >= rate_count {
        set_error!(
            error,
            error_size,
            b"KSP returned unsupported maximum rails-warp factor %d\0" as *const u8
                as *const ::core::ffi::c_char,
            *maximum_factor as ::core::ffi::c_int,
        );
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn ut_resolution(
    mut ut: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut adjacent: ::core::ffi::c_double = nextafter(
        ut,
        ::core::f32::INFINITY as ::core::ffi::c_double,
    );
    let mut resolution: ::core::ffi::c_double = adjacent - ut;
    return if resolution.is_finite() as i32 != 0 && resolution > 0.0f64 {
        resolution
    } else {
        DBL_EPSILON
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_warp(
    mut client: *mut KrpcCNanoClient,
    mut ut: ::core::ffi::c_double,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    return krpc_cnano_client_warp_with_progress(
        client,
        ut,
        None,
        NULL,
        error,
        error_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_warp_with_progress(
    mut client: *mut KrpcCNanoClient,
    mut ut: ::core::ffi::c_double,
    mut progress: KRPCWarpProgress,
    mut context: *mut ::core::ffi::c_void,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || !(*client).connected || ut.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut timeout_ms: ::core::ffi::c_int = krpc_cnano_transport_timeout_ms(
        (*client).connection,
    );
    if timeout_ms <= 0 as ::core::ffi::c_int {
        set_error!(
            error,
            error_size,
            b"Rails warp requires a positive configured transport timeout\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut progress_timeout: ::core::ffi::c_double = timeout_ms as ::core::ffi::c_double
        / 1000.0f64;
    let mut observed_factor: int32_t = -(1 as int32_t);
    let mut stop_latency: ::core::ffi::c_double = 0.0f64;
    if !set_rails_warp_factor_observed(
        client,
        0 as int32_t,
        &raw mut observed_factor,
        &raw mut stop_latency,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    let mut factor_command_latency: ::core::ffi::c_double = stop_latency;
    let mut game_paused: bool = false_0 != 0;
    if krpc_KRPC_Paused((*client).connection, &raw mut game_paused) as ::core::ffi::c_int
        == KRPC_OK as ::core::ffi::c_int && game_paused as ::core::ffi::c_int != 0
    {
        let mut _e: krpc_error_t = krpc_KRPC_set_Paused(
            (*client).connection,
            0 as ::core::ffi::c_int != 0,
        );
        (*client).total_rpc_calls = (*client).total_rpc_calls.wrapping_add(1);
        (*client).total_wire_requests = (*client).total_wire_requests.wrapping_add(1);
    }
    let mut maximum_factor: int32_t = 0 as int32_t;
    if !read_maximum_rails_factor(client, &raw mut maximum_factor, error, error_size) {
        return false_0 != 0;
    }
    if maximum_factor < 1 as int32_t {
        set_error!(
            error,
            error_size,
            b"KSP reports that on-rails time warp is unavailable\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut state: CNanoWarpState = CNanoWarpState {
        ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        factor: 0,
        rate: 0.,
        latency_seconds: 0.,
    };
    if !read_warp_state(client, &raw mut state, error, error_size) {
        return false_0 != 0;
    }
    if ut <= state.ut {
        reset_rate_filters(client);
        return true_0 != 0;
    }
    let mut rates: [::core::ffi::c_double; 8] = [0.; 8];
    memcpy(
        &raw mut rates as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
        &raw const k_stock_rails_rate_by_factor as *const ::core::ffi::c_double
            as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 8]>() as size_t,
    );
    if (state.factor as size_t)
        < (::core::mem::size_of::<[::core::ffi::c_double; 8]>() as usize)
            .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
    {
        rates[state.factor as usize] = state.rate;
    }
    let mut last_progress_ut: ::core::ffi::c_double = state.ut;
    let mut last_progress_wall: ::core::ffi::c_double = monotonic_seconds();
    let mut last_publish_wall: ::core::ffi::c_double = last_progress_wall;
    if progress.is_some() {
        progress
            .expect(
                "non-null function pointer",
            )(context, state.ut, ut, state.factor as ::core::ffi::c_int, state.rate);
    }
    loop {
        let mut remaining: ::core::ffi::c_double = ut - state.ut;
        if remaining <= 0.0f64 {
            if state.factor != 0 as int32_t {
                let mut latency: ::core::ffi::c_double = 0.0f64;
                if !set_rails_warp_factor_observed(
                    client,
                    0 as int32_t,
                    &raw mut observed_factor,
                    &raw mut latency,
                    error,
                    error_size,
                ) {
                    return false_0 != 0;
                }
            }
            reset_rate_filters(client);
            return true_0 != 0;
        }
        if (state.factor as size_t)
            < (::core::mem::size_of::<[::core::ffi::c_double; 8]>() as usize)
                .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
        {
            rates[state.factor as usize] = state.rate;
        }
        let mut response_horizon: ::core::ffi::c_double = factor_command_latency
            + state.latency_seconds + stop_latency;
        let mut decision: KrpcCNanoWarpDecision = KrpcCNanoWarpDecision {
            factor: 0 as int32_t,
            predicted_rate: 0.,
            response_horizon_seconds: 0.,
            stopping_margin_ut: 0.,
        };
        if !warp_decision_from_rates(
            remaining,
            response_horizon,
            maximum_factor as ::core::ffi::c_int,
            &raw mut rates as *mut ::core::ffi::c_double,
            (::core::mem::size_of::<[::core::ffi::c_double; 8]>() as size_t)
                .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as size_t),
            &raw mut decision,
        ) {
            set_error!(
                error,
                error_size,
                b"Could not derive a rails-warp factor from remaining UT %.9g and response horizon %.9g\0"
                    as *const u8 as *const ::core::ffi::c_char,
                remaining,
                response_horizon,
            );
            return false_0 != 0;
        }
        if decision.factor == 0 as int32_t {
            if state.factor != 0 as int32_t {
                let mut latency_0: ::core::ffi::c_double = 0.0f64;
                if !set_rails_warp_factor_observed(
                    client,
                    0 as int32_t,
                    &raw mut observed_factor,
                    &raw mut latency_0,
                    error,
                    error_size,
                ) {
                    return false_0 != 0;
                }
                if latency_0 > stop_latency {
                    stop_latency = latency_0;
                }
            }
            reset_rate_filters(client);
            return true_0 != 0;
        }
        if state.factor != decision.factor {
            let mut latency_1: ::core::ffi::c_double = 0.0f64;
            if !set_rails_warp_factor_observed(
                client,
                decision.factor,
                &raw mut observed_factor,
                &raw mut latency_1,
                error,
                error_size,
            ) {
                return false_0 != 0;
            }
            if latency_1 > factor_command_latency {
                factor_command_latency = latency_1;
            }
            if observed_factor == 0 as int32_t {
                set_error!(
                    error,
                    error_size,
                    b"KSP declined requested rails-warp factor %d\0" as *const u8
                        as *const ::core::ffi::c_char,
                    decision.factor,
                );
                return false_0 != 0;
            }
            if observed_factor < decision.factor {
                maximum_factor = observed_factor;
            }
        }
        let mut next: CNanoWarpState = CNanoWarpState {
            ut: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            factor: 0,
            rate: 0.,
            latency_seconds: 0.,
        };
        if !read_warp_state(client, &raw mut next, error, error_size) {
            krpc_SpaceCenter_set_RailsWarpFactor((*client).connection, 0 as int32_t);
            return false_0 != 0;
        }
        let mut wall: ::core::ffi::c_double = monotonic_seconds();
        let mut resolution: ::core::ffi::c_double = ut_resolution(last_progress_ut);
        if next.ut > last_progress_ut + resolution {
            last_progress_ut = next.ut;
            last_progress_wall = wall;
        } else if wall - last_progress_wall >= progress_timeout {
            krpc_SpaceCenter_set_RailsWarpFactor((*client).connection, 0 as int32_t);
            set_error!(
                error,
                error_size,
                b"Rails warp made no measurable UT progress within the configured %.3f s transport timeout\0"
                    as *const u8 as *const ::core::ffi::c_char,
                progress_timeout,
            );
            return false_0 != 0;
        }
        if progress.is_some() && wall - last_publish_wall >= 1.0f64 {
            progress
                .expect(
                    "non-null function pointer",
                )(context, next.ut, ut, next.factor as ::core::ffi::c_int, next.rate);
            last_publish_wall = wall;
        }
        state = next;
    };
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_physics_warp(
    mut client: *mut KrpcCNanoClient,
    mut factor: *mut ::core::ffi::c_int,
    mut rate: *mut ::core::ffi::c_double,
    mut rails: *mut bool,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || factor.is_null() || rate.is_null() || rails.is_null() {
        return false_0 != 0;
    }
    let mut mode: krpc_SpaceCenter_WarpMode_t = KRPC_SPACECENTER_WARPMODE_RAILS;
    let mut warp_rate: ::core::ffi::c_float = 1.0f32;
    let mut physics: int32_t = 0 as int32_t;
    if rpc_failed(
        client,
        krpc_SpaceCenter_WarpMode((*client).connection, &raw mut mode),
        b"krpc_SpaceCenter_WarpMode(client->connection, &mode)\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_WarpRate((*client).connection, &raw mut warp_rate),
        b"krpc_SpaceCenter_WarpRate(client->connection, &warp_rate)\0" as *const u8
            as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_PhysicsWarpFactor((*client).connection, &raw mut physics),
        b"krpc_SpaceCenter_PhysicsWarpFactor(client->connection, &physics)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    *rails = mode as ::core::ffi::c_uint
        == KRPC_SPACECENTER_WARPMODE_RAILS as ::core::ffi::c_int as ::core::ffi::c_uint;
    *factor = if mode as ::core::ffi::c_uint
        == KRPC_SPACECENTER_WARPMODE_PHYSICS as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        physics as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
    *rate = if warp_rate.is_finite() as i32 != 0 && warp_rate > 0.0f32 {
        warp_rate as ::core::ffi::c_double
    } else {
        1.0f64
    };
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn krpc_cnano_client_set_physics_warp(
    mut client: *mut KrpcCNanoClient,
    mut factor: ::core::ffi::c_int,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if client.is_null() || factor < 0 as ::core::ffi::c_int
        || factor > 3 as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    if rpc_failed(
        client,
        krpc_SpaceCenter_set_PhysicsWarpFactor((*client).connection, factor as int32_t),
        b"krpc_SpaceCenter_set_PhysicsWarpFactor(client->connection, (int32_t)factor)\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn engine_role_contains(
    mut text: *const ::core::ffi::c_char,
    mut word: *const ::core::ffi::c_char,
) -> bool {
    if text.is_null() {
        return false_0 != 0;
    }
    while *text != 0 {
        let mut i: size_t = 0 as size_t;
        while *word.offset(i as isize) as ::core::ffi::c_int != 0
            && *text.offset(i as isize) as ::core::ffi::c_int != 0
            && ({
                let mut __res: ::core::ffi::c_int = 0;
                if ::core::mem::size_of::<::core::ffi::c_uchar>() as usize > 1 as usize {
                    if 0 != 0 {
                        let mut __c: ::core::ffi::c_int = *text.offset(i as isize)
                            as ::core::ffi::c_uchar as ::core::ffi::c_int;
                        __res = (if __c < -(128 as ::core::ffi::c_int)
                            || __c > 255 as ::core::ffi::c_int
                        {
                            __c as __int32_t
                        } else {
                            *(*__ctype_tolower_loc()).offset(__c as isize)
                        }) as ::core::ffi::c_int;
                    } else {
                        __res = tolower(
                            *text.offset(i as isize) as ::core::ffi::c_uchar
                                as ::core::ffi::c_int,
                        );
                    }
                } else {
                    __res = *(*__ctype_tolower_loc())
                        .offset(
                            *text.offset(i as isize) as ::core::ffi::c_uchar
                                as ::core::ffi::c_int as isize,
                        ) as ::core::ffi::c_int;
                }
                __res
            })
                == ({
                    let mut __res: ::core::ffi::c_int = 0;
                    if ::core::mem::size_of::<::core::ffi::c_uchar>() as usize
                        > 1 as usize
                    {
                        if 0 != 0 {
                            let mut __c: ::core::ffi::c_int = *word.offset(i as isize)
                                as ::core::ffi::c_uchar as ::core::ffi::c_int;
                            __res = (if __c < -(128 as ::core::ffi::c_int)
                                || __c > 255 as ::core::ffi::c_int
                            {
                                __c as __int32_t
                            } else {
                                *(*__ctype_tolower_loc()).offset(__c as isize)
                            }) as ::core::ffi::c_int;
                        } else {
                            __res = tolower(
                                *word.offset(i as isize) as ::core::ffi::c_uchar
                                    as ::core::ffi::c_int,
                            );
                        }
                    } else {
                        __res = *(*__ctype_tolower_loc())
                            .offset(
                                *word.offset(i as isize) as ::core::ffi::c_uchar
                                    as ::core::ffi::c_int as isize,
                            ) as ::core::ffi::c_int;
                    }
                    __res
                })
        {
            i = i.wrapping_add(1);
        }
        if *word.offset(i as isize) == 0 {
            return true_0 != 0;
        }
        text = text.offset(1);
    }
    return false_0 != 0;
}
#[inline]
unsafe extern "C" fn engine_role_identify(
    mut name: *const ::core::ffi::c_char,
    mut title: *const ::core::ffi::c_char,
    mut tag: *const ::core::ffi::c_char,
) -> EngineRole {
    if !tag.is_null()
        && (strcmp(tag, b"OMS\0" as *const u8 as *const ::core::ffi::c_char) == 0
            || strcmp(tag, b"oms\0" as *const u8 as *const ::core::ffi::c_char) == 0)
    {
        return ENGINE_ROLE_OMS;
    }
    if !tag.is_null()
        && (strcmp(tag, b"SSME\0" as *const u8 as *const ::core::ffi::c_char) == 0
            || strcmp(tag, b"ssme\0" as *const u8 as *const ::core::ffi::c_char) == 0)
    {
        return ENGINE_ROLE_MAIN;
    }
    if engine_role_contains(name, b"SSME\0" as *const u8 as *const ::core::ffi::c_char)
        as ::core::ffi::c_int != 0
        || engine_role_contains(
            title,
            b"space shuttle main engine\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
    {
        return ENGINE_ROLE_MAIN;
    }
    if engine_role_contains(
        title,
        b"orbital manoeuvr\0" as *const u8 as *const ::core::ffi::c_char,
    ) as ::core::ffi::c_int != 0
        || engine_role_contains(
            title,
            b"orbital maneuver\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        || engine_role_contains(
            title,
            b"terrier\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        || engine_role_contains(
            name,
            b"terrier\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
        || engine_role_contains(
            name,
            b"OMS\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0
    {
        return ENGINE_ROLE_OMS;
    }
    return ENGINE_ROLE_OTHER;
}
#[inline]
unsafe extern "C" fn engine_role_selected(
    mut role: EngineRole,
    mut has_oms: bool,
) -> bool {
    return if has_oms as ::core::ffi::c_int != 0 {
        (role as ::core::ffi::c_uint
            == ENGINE_ROLE_OMS as ::core::ffi::c_int as ::core::ffi::c_uint)
            as ::core::ffi::c_int
    } else {
        (role as ::core::ffi::c_uint
            != ENGINE_ROLE_MAIN as ::core::ffi::c_int as ::core::ffi::c_uint)
            as ::core::ffi::c_int
    } != 0;
}
#[inline]
unsafe extern "C" fn engine_propellant_is_intake_air(
    mut name: *const ::core::ffi::c_char,
) -> bool {
    return !name.is_null()
        && strcmp(name, b"IntakeAir\0" as *const u8 as *const ::core::ffi::c_char) == 0;
}
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const __DBL_MAX__: ::core::ffi::c_double = 1.7976931348623157e+308f64;

#[cfg(test)]
mod tests {
    use super::*;

    /// Deorbit thrust comes only from the orbital engines, never the mains.
    #[test]
    fn engine_roles_select_the_orbital_engines() {
        unsafe {
            assert_eq!(engine_role_identify(c"SSME".as_ptr(), c"Vector".as_ptr(), c"".as_ptr()), ENGINE_ROLE_MAIN);
            assert_eq!(
                engine_role_identify(c"benjee10.AJ10.v2".as_ptr(), c"KJ-10b Viking Orbital Manoeuvring Engine".as_ptr(), c"".as_ptr()),
                ENGINE_ROLE_OMS
            );
            assert_eq!(
                engine_role_identify(c"liquidEngine3".as_ptr(), c"RE-I5 'Terrier' Liquid Fuel Engine".as_ptr(), c"".as_ptr()),
                ENGINE_ROLE_OMS
            );
            assert_eq!(engine_role_identify(c"unknown".as_ptr(), c"unknown".as_ptr(), c"OMS".as_ptr()), ENGINE_ROLE_OMS);
            assert!(!engine_role_selected(ENGINE_ROLE_MAIN, true));
            assert!(!engine_role_selected(ENGINE_ROLE_OTHER, true));
            assert!(engine_role_selected(ENGINE_ROLE_OMS, true));
            assert!(!engine_role_selected(ENGINE_ROLE_MAIN, false));
            assert!(engine_role_selected(ENGINE_ROLE_OTHER, false));
            assert!(engine_propellant_is_intake_air(c"IntakeAir".as_ptr()));
            assert!(!engine_propellant_is_intake_air(c"LiquidFuel".as_ptr()));
            assert!(!engine_propellant_is_intake_air(core::ptr::null()));
        }
    }

    /// The commanded attitude points the vessel-frame thrust axis along the burn direction.
    #[test]
    fn thrust_alignment_points_the_engine_axis_along_the_burn() {
        unsafe {
            let axis = vnorm(v3(0.1, 0.93, 0.34), v3(0.0, 1.0, 0.0));
            let d = vnorm(v3(-0.8, 0.5, 0.2), v3(0.0, 1.0, 0.0));
            let u = vnorm(vproject_plane(v3(0.0, 0.0, 1.0), d), v3(0.0, 0.0, 1.0));
            let (mut nose, mut roof) = (v3(0.0, 0.0, 0.0), v3(0.0, 0.0, 0.0));
            thrust_alignment_attitude(axis, d, u, &mut nose, &mut roof);
            let right = vnorm(vcross(nose, roof), v3(1.0, 0.0, 0.0));
            let actual = vadd(vscale(right, axis.x), vadd(vscale(nose, axis.y), vscale(roof, -axis.z)));
            assert!(vmag(vsub(actual, d)) < 1e-9);
            assert!(vdot(nose, roof).abs() < 1e-9);
            thrust_alignment_attitude(v3(0.0, 1.0, 0.0), d, u, &mut nose, &mut roof);
            assert!(vmag(vsub(nose, d)) < 1e-9);
        }
    }
}
