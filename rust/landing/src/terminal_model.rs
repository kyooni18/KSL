use crate::sys::*;
extern "C" {
    fn world_seed_kerbin(w: *mut KerbinWorld);
    fn world_load_atmosphere_csv(
        w: *mut KerbinWorld,
        path: *const ::core::ffi::c_char,
    ) -> bool;
    fn aero_seed_stsn(a: *mut AeroTable);
    fn aero_load_csv(a: *mut AeroTable, path: *const ::core::ffi::c_char) -> bool;
    fn aero_load_book_csv(a: *mut AeroTable, path: *const ::core::ffi::c_char) -> bool;
    fn attitude_seed(a: *mut AttitudeModel);
    fn attitude_load_ini(
        a: *mut AttitudeModel,
        path: *const ::core::ffi::c_char,
    ) -> bool;
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn remainder(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
}
pub type size_t = usize;
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
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
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
pub struct AtmospherePoint {
    pub altitude_m: ::core::ffi::c_double,
    pub sample: AtmosphereSample,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereTable {
    pub p: [AtmospherePoint; 2048],
    pub count: size_t,
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
pub struct TerminalModelSourceFiles {
    pub atmosphere_csv: *const ::core::ffi::c_char,
    pub aero_csv: *const ::core::ffi::c_char,
    pub aero_book_csv: *const ::core::ffi::c_char,
    pub attitude_ini: *const ::core::ffi::c_char,
}
pub const ATM_TABLE_MAX: ::core::ffi::c_int = 2048 as ::core::ffi::c_int;
pub const AERO_MACH_MAX: ::core::ffi::c_int = 40 as ::core::ffi::c_int;
pub const AERO_ALPHA_MAX: ::core::ffi::c_int = 40 as ::core::ffi::c_int;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn digest_file(
    mut path: *const ::core::ffi::c_char,
    mut optional: bool,
    mut ok: *mut bool,
) -> uint64_t {
    *ok = false_0 != 0;
    if path.is_null() || *path == 0
        || optional as ::core::ffi::c_int != 0
            && strcmp(path, b"none\0" as *const u8 as *const ::core::ffi::c_char) == 0
    {
        *ok = optional;
        return 0 as uint64_t;
    }
    let mut f: *mut FILE = fopen(
        path,
        b"rb\0" as *const u8 as *const ::core::ffi::c_char,
    ) as *mut FILE;
    if f.is_null() {
        return 0 as uint64_t;
    }
    let mut h: uint64_t = 1469598103934665603 as uint64_t;
    let mut block: [::core::ffi::c_uchar; 4096] = [0; 4096];
    let mut n: size_t = 0;
    loop {
        n = fread(
            &raw mut block as *mut ::core::ffi::c_uchar as *mut ::core::ffi::c_void,
            1 as size_t,
            ::core::mem::size_of::<[::core::ffi::c_uchar; 4096]>() as size_t,
            f,
        ) as size_t;
        let mut i: size_t = 0 as size_t;
        while i < n {
            h ^= block[i as usize] as uint64_t;
            h = (h as ::core::ffi::c_ulong)
                .wrapping_mul(1099511628211 as ::core::ffi::c_ulong) as uint64_t
                as uint64_t;
            i = i.wrapping_add(1);
        }
        if !(n == ::core::mem::size_of::<[::core::ffi::c_uchar; 4096]>() as usize) {
            break;
        }
    }
    let mut read_ok: bool = ferror(f) == 0;
    if fclose(f) != 0 as ::core::ffi::c_int {
        read_ok = false_0 != 0;
    }
    if read_ok {
        *ok = true_0 != 0;
    }
    return if read_ok as ::core::ffi::c_int != 0 { h } else { 0 as uint64_t };
}
unsafe extern "C" fn finite_vec3(mut v: Vec3) -> bool {
    return v.x.is_finite() as i32 != 0 && v.y.is_finite() as i32 != 0
        && v.z.is_finite() as i32 != 0;
}
unsafe extern "C" fn reject(
    mut reason: *mut ::core::ffi::c_char,
    mut n: size_t,
    mut message: *const ::core::ffi::c_char,
) -> bool {
    if !reason.is_null() && n != 0 {
        snprintf(reason, n, b"%s\0" as *const u8 as *const ::core::ffi::c_char, message);
    }
    return false_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_model_capture(
    mut m: *mut TerminalModel,
    mut files: *const TerminalModelSourceFiles,
    mut cfg: *const LandingConfiguration,
    mut snapshot_id: uint64_t,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    if !reason.is_null() && reason_size != 0 {
        *reason.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
            as ::core::ffi::c_char;
    }
    if m.is_null() || files.is_null() || cfg.is_null() || snapshot_id == 0 {
        return reject(
            reason,
            reason_size,
            b"missing terminal model capture input\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    memset(
        m as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TerminalModel>() as size_t,
    );
    (*m).schema_version = 1 as uint32_t;
    (*m).snapshot_id = snapshot_id;
    world_seed_kerbin(&raw mut (*m).world);
    if !world_load_atmosphere_csv(&raw mut (*m).world, (*files).atmosphere_csv) {
        return reject(
            reason,
            reason_size,
            b"failed to load native atmosphere source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    aero_seed_stsn(&raw mut (*m).aero);
    if !aero_load_csv(&raw mut (*m).aero, (*files).aero_csv) {
        return reject(
            reason,
            reason_size,
            b"failed to load native aerodynamic source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    let mut book_ok: bool = true_0 != 0;
    if !(*files).aero_book_csv.is_null()
        && strcmp(
            (*files).aero_book_csv,
            b"none\0" as *const u8 as *const ::core::ffi::c_char,
        ) != 0 as ::core::ffi::c_int
    {
        if !aero_load_book_csv(&raw mut (*m).aero, (*files).aero_book_csv) {
            return reject(
                reason,
                reason_size,
                b"failed to load native force-book source\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
    }
    attitude_seed(&raw mut (*m).attitude);
    if !attitude_load_ini(&raw mut (*m).attitude, (*files).attitude_ini) {
        return reject(
            reason,
            reason_size,
            b"failed to load native attitude source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*m).vehicle = (*cfg).vehicle;
    (*m).site = (*cfg).site;
    (*m).guidance = (*cfg).guidance;
    (*m).atmosphere_source_digest = digest_file(
        (*files).atmosphere_csv,
        false_0 != 0,
        &raw mut book_ok,
    );
    if !book_ok {
        return reject(
            reason,
            reason_size,
            b"failed to hash native atmosphere source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*m).aero_source_digest = digest_file(
        (*files).aero_csv,
        false_0 != 0,
        &raw mut book_ok,
    );
    if !book_ok {
        return reject(
            reason,
            reason_size,
            b"failed to hash native aerodynamic source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*m).aero_book_source_digest = digest_file(
        (*files).aero_book_csv,
        true_0 != 0,
        &raw mut book_ok,
    );
    if !book_ok {
        return reject(
            reason,
            reason_size,
            b"failed to hash native force-book source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*m).attitude_source_digest = digest_file(
        (*files).attitude_ini,
        false_0 != 0,
        &raw mut book_ok,
    );
    if !book_ok {
        return reject(
            reason,
            reason_size,
            b"failed to hash native attitude source\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if !terminal_model_validate(m, reason, reason_size) {
        return false_0 != 0;
    }
    (*m).replay_validated = true_0 != 0;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_model_align_planet_frame(
    mut m: *mut TerminalModel,
    mut p: *const PlanetModel,
    mut reason: *mut ::core::ffi::c_char,
    mut n: size_t,
) -> bool {
    if !reason.is_null() && n != 0 {
        *reason.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
            as ::core::ffi::c_char;
    }
    if m.is_null() || p.is_null() || !(*m).replay_validated {
        return reject(
            reason,
            n,
            b"terminal model or live planet frame unavailable\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*m).planet_frame_aligned = false_0 != 0;
    let mut north_length: ::core::ffi::c_double = sqrt(
        (*p).north_axis.x * (*p).north_axis.x + (*p).north_axis.y * (*p).north_axis.y
            + (*p).north_axis.z * (*p).north_axis.z,
    );
    let mut prime_length: ::core::ffi::c_double = hypot(
        (*p).prime_meridian_at_epoch.x,
        (*p).prime_meridian_at_epoch.y,
    );
    if !positive_finite(north_length) || !positive_finite(prime_length)
        || (*p).epoch_ut.is_finite() as i32 == 0
        || (*p).rotational_speed.is_finite() as i32 == 0
        || fabs((*p).north_axis.x / north_length) > 1e-5f64
        || fabs((*p).north_axis.y / north_length) > 1e-5f64
        || (*p).north_axis.z / north_length < 0.999999f64
        || fabs((*p).prime_meridian_at_epoch.z) > 1e-5f64
    {
        return reject(
            reason,
            n,
            b"planet rotation cannot be represented in native +Z frame\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if !positive_finite((*p).radius) || !positive_finite((*p).gravitational_parameter)
        || fabs((*m).world.radius_m - (*p).radius) > 1e-4f64 * fmax(1.0f64, (*p).radius)
        || fabs((*m).world.mu_m3_s2 - (*p).gravitational_parameter)
            > 1e-4f64 * fmax(1.0f64, (*p).gravitational_parameter)
        || fabs((*m).world.rotation_rate_rad_s - (*p).rotational_speed) > 1e-7f64
    {
        return reject(
            reason,
            n,
            b"native terminal world does not match live planet dynamics\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    (*m).world.rotation_phase_rad_at_ut0 = atan2(
        (*p).prime_meridian_at_epoch.y,
        (*p).prime_meridian_at_epoch.x,
    ) - (*p).rotational_speed * (*p).epoch_ut;
    (*m).planet_frame_aligned = (*m).world.rotation_phase_rad_at_ut0.is_finite() as i32
        != 0;
    return (*m).planet_frame_aligned;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_model_planet_matches(
    mut m: *const TerminalModel,
    mut p: *const PlanetModel,
) -> bool {
    if m.is_null() || p.is_null() || !(*m).replay_validated || !(*m).planet_frame_aligned
        || (*p).epoch_ut.is_finite() as i32 == 0
        || (*p).rotational_speed.is_finite() as i32 == 0
        || (*p).prime_meridian_at_epoch.x.is_finite() as i32 == 0
        || (*p).prime_meridian_at_epoch.y.is_finite() as i32 == 0
        || !positive_finite((*p).radius)
        || !positive_finite((*p).gravitational_parameter)
        || fabs((*m).world.radius_m - (*p).radius) > 1e-4f64 * fmax(1.0f64, (*p).radius)
        || fabs((*m).world.mu_m3_s2 - (*p).gravitational_parameter)
            > 1e-4f64 * fmax(1.0f64, (*p).gravitational_parameter)
        || fabs((*m).world.rotation_rate_rad_s - (*p).rotational_speed) > 1e-7f64
    {
        return false_0 != 0;
    }
    let mut north_length: ::core::ffi::c_double = sqrt(
        (*p).north_axis.x * (*p).north_axis.x + (*p).north_axis.y * (*p).north_axis.y
            + (*p).north_axis.z * (*p).north_axis.z,
    );
    if !positive_finite(north_length) || fabs((*p).north_axis.x / north_length) > 1e-5f64
        || fabs((*p).north_axis.y / north_length) > 1e-5f64
        || (*p).north_axis.z / north_length < 0.999999f64
    {
        return false_0 != 0;
    }
    let mut prime_length: ::core::ffi::c_double = hypot(
        (*p).prime_meridian_at_epoch.x,
        (*p).prime_meridian_at_epoch.y,
    );
    if !positive_finite(prime_length) {
        return false_0 != 0;
    }
    let mut expected: ::core::ffi::c_double = atan2(
        (*p).prime_meridian_at_epoch.y,
        (*p).prime_meridian_at_epoch.x,
    ) - (*p).rotational_speed * (*p).epoch_ut;
    return expected.is_finite() as i32 != 0
        && (*m).world.rotation_phase_rad_at_ut0.is_finite() as i32 != 0
        && fabs(
            remainder(
                (*m).world.rotation_phase_rad_at_ut0 - expected,
                2.0f64 * acos(-1.0f64),
            ),
        ) < 1e-6f64;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_model_policy_matches(
    mut m: *const TerminalModel,
    mut site: *const LandingSite,
    mut v: *const VehicleProfile,
    mut g: *const GuidanceSettings,
) -> bool {
    if m.is_null() || site.is_null() || v.is_null() || g.is_null()
        || !(*m).replay_validated || !(*m).planet_frame_aligned
    {
        return false_0 != 0;
    }
    return memcmp(
        &raw const (*m).site as *const ::core::ffi::c_void,
        site as *const ::core::ffi::c_void,
        ::core::mem::size_of::<LandingSite>() as size_t,
    ) == 0
        && memcmp(
            &raw const (*m).vehicle as *const ::core::ffi::c_void,
            v as *const ::core::ffi::c_void,
            ::core::mem::size_of::<VehicleProfile>() as size_t,
        ) == 0
        && memcmp(
            &raw const (*m).guidance as *const ::core::ffi::c_void,
            g as *const ::core::ffi::c_void,
            ::core::mem::size_of::<GuidanceSettings>() as size_t,
        ) == 0;
}
unsafe extern "C" fn positive_finite(mut value: ::core::ffi::c_double) -> bool {
    return value.is_finite() as i32 != 0 && value > 0.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_model_validate(
    mut m: *const TerminalModel,
    mut reason: *mut ::core::ffi::c_char,
    mut n: size_t,
) -> bool {
    if !reason.is_null() && n != 0 {
        *reason.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
            as ::core::ffi::c_char;
    }
    if m.is_null() {
        return reject(
            reason,
            n,
            b"missing terminal model\0" as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if (*m).schema_version != 1 as uint32_t || (*m).snapshot_id == 0 {
        return reject(
            reason,
            n,
            b"terminal model snapshot identity is invalid\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if !positive_finite((*m).world.radius_m) || !positive_finite((*m).world.mu_m3_s2)
        || !positive_finite((*m).world.atmosphere_top_m)
        || (*m).world.atmosphere.count < 2 as size_t
        || (*m).world.atmosphere.count > ATM_TABLE_MAX as size_t
    {
        return reject(
            reason,
            n,
            b"terminal world snapshot is incomplete\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if (*m).world.rotation_rate_rad_s.is_finite() as i32 == 0
        || (*m).world.rotation_phase_rad_at_ut0.is_finite() as i32 == 0
    {
        return reject(
            reason,
            n,
            b"terminal world rotation is invalid\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    let mut i: size_t = 0 as size_t;
    while i < (*m).world.atmosphere.count {
        let mut p: *const AtmospherePoint = (&raw const (*m).world.atmosphere.p
            as *const AtmospherePoint)
            .offset(i as isize) as *const AtmospherePoint;
        if (*p).altitude_m.is_finite() as i32 == 0 || (*p).altitude_m < 0.0f64
            || (*p).sample.density_kg_m3.is_finite() as i32 == 0
            || (*p).sample.density_kg_m3 < 0.0f64
            || (*p).sample.pressure_pa.is_finite() as i32 == 0
            || (*p).sample.pressure_pa < 0.0f64
            || (*p).sample.temperature_k.is_finite() as i32 == 0
            || (*p).sample.temperature_k <= 0.0f64
            || (*p).sample.speed_of_sound_mps.is_finite() as i32 == 0
            || (*p).sample.speed_of_sound_mps <= 0.0f64
            || i != 0
                && (*p).altitude_m
                    <= (*m)
                        .world
                        .atmosphere
                        .p[i.wrapping_sub(1 as size_t) as usize]
                        .altitude_m
        {
            return reject(
                reason,
                n,
                b"terminal atmosphere table is invalid or unordered\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
        i = i.wrapping_add(1);
    }
    if (*m).aero.mach_count < 2 as size_t || (*m).aero.alpha_count < 2 as size_t
        || (*m).aero.mach_count > AERO_MACH_MAX as size_t
        || (*m).aero.alpha_count > AERO_ALPHA_MAX as size_t
        || !positive_finite((*m).aero.reference_area_m2)
    {
        return reject(
            reason,
            n,
            b"terminal aerodynamic snapshot is incomplete\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*m).aero.mach_count {
        if (*m).aero.mach[i_0 as usize].is_finite() as i32 == 0
            || i_0 != 0
                && (*m).aero.mach[i_0 as usize]
                    <= (*m).aero.mach[i_0.wrapping_sub(1 as size_t) as usize]
        {
            return reject(
                reason,
                n,
                b"terminal Mach grid is invalid or unordered\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
        i_0 = i_0.wrapping_add(1);
    }
    let mut i_1: size_t = 0 as size_t;
    while i_1 < (*m).aero.alpha_count {
        if (*m).aero.alpha_deg[i_1 as usize].is_finite() as i32 == 0
            || i_1 != 0
                && (*m).aero.alpha_deg[i_1 as usize]
                    <= (*m).aero.alpha_deg[i_1.wrapping_sub(1 as size_t) as usize]
        {
            return reject(
                reason,
                n,
                b"terminal incidence grid is invalid or unordered\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        }
        i_1 = i_1.wrapping_add(1);
    }
    let mut i_2: size_t = 0 as size_t;
    while i_2 < (*m).aero.mach_count {
        let mut j: size_t = 0 as size_t;
        while j < (*m).aero.alpha_count {
            if (*m).aero.cl[i_2 as usize][j as usize].is_finite() as i32 == 0
                || (*m).aero.cd[i_2 as usize][j as usize].is_finite() as i32 == 0
                || (*m).aero.cd[i_2 as usize][j as usize] < 0.0f64
            {
                return reject(
                    reason,
                    n,
                    b"terminal aerodynamic coefficient table is invalid\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
            }
            j = j.wrapping_add(1);
        }
        i_2 = i_2.wrapping_add(1);
    }
    if !positive_finite((*m).attitude.pitch_wn)
        || !positive_finite((*m).attitude.roll_wn)
        || !positive_finite((*m).attitude.pitch_zeta)
        || !positive_finite((*m).attitude.roll_zeta)
        || !positive_finite((*m).attitude.max_pitch_rate_rad_s)
        || !positive_finite((*m).attitude.max_roll_rate_rad_s)
        || !positive_finite((*m).attitude.max_pitch_accel_rad_s2)
        || !positive_finite((*m).attitude.max_roll_accel_rad_s2)
    {
        return reject(
            reason,
            n,
            b"terminal attitude response snapshot is incomplete\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if !positive_finite((*m).vehicle.touchdown_speed)
        || !positive_finite((*m).vehicle.maximum_angle_of_attack)
        || !positive_finite((*m).vehicle.maximum_bank_angle)
        || !positive_finite((*m).guidance.hac_radius)
        || !positive_finite((*m).guidance.final_approach_distance)
    {
        return reject(
            reason,
            n,
            b"terminal vehicle or runway policy is invalid\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if (*m).site.latitude.is_finite() as i32 == 0
        || (*m).site.longitude.is_finite() as i32 == 0
        || (*m).site.altitude.is_finite() as i32 == 0
        || (*m).site.runway_heading.is_finite() as i32 == 0
        || !positive_finite((*m).site.runway_length)
        || !positive_finite((*m).site.runway_width)
    {
        return reject(
            reason,
            n,
            b"terminal runway snapshot is invalid\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn terminal_state_validate(
    mut s: *const TerminalDynamicState,
    mut reason: *mut ::core::ffi::c_char,
    mut n: size_t,
) -> bool {
    if !reason.is_null() && n != 0 {
        *reason.offset(0 as ::core::ffi::c_int as isize) = '\0' as i32
            as ::core::ffi::c_char;
    }
    if s.is_null() {
        return reject(
            reason,
            n,
            b"missing terminal dynamic state\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if !finite_vec3((*s).position_i_m) || !finite_vec3((*s).velocity_i_mps)
        || !((*s).mass_kg > 0.0f64) || (*s).mass_kg.is_finite() as i32 == 0
        || (*s).ut_s.is_finite() as i32 == 0
    {
        return reject(
            reason,
            n,
            b"terminal dynamic state contains invalid position, velocity, mass, or time\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    if (*s).attitude.aoa_rad.is_finite() as i32 == 0
        || (*s).attitude.bank_rad.is_finite() as i32 == 0
        || (*s).attitude.aoa_rate_rad_s.is_finite() as i32 == 0
        || (*s).attitude.bank_rate_rad_s.is_finite() as i32 == 0
        || (*s).attitude.cmd_aoa_rad.is_finite() as i32 == 0
        || (*s).attitude.cmd_bank_rad.is_finite() as i32 == 0
        || (*s).attitude.requested_aoa_rad.is_finite() as i32 == 0
        || (*s).attitude.requested_bank_rad.is_finite() as i32 == 0
    {
        return reject(
            reason,
            n,
            b"terminal attitude state is invalid\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    return true_0 != 0;
}
