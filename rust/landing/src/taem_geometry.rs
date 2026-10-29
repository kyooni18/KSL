use crate::sys::*;
extern "C" {
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
    fn taem_vec3_dot(a: TaemVec3, b: TaemVec3) -> ::core::ffi::c_double;
    fn taem_vec3_norm(a: TaemVec3) -> ::core::ffi::c_double;
    fn taem_inertial_to_fixed(
        world: *const TaemFrameWorld,
        position_i: TaemVec3,
        velocity_i: TaemVec3,
        ut: ::core::ffi::c_double,
        position_b: *mut TaemVec3,
        velocity_b: *mut TaemVec3,
    ) -> bool;
    fn taem_local_north_east_up(
        position_b: TaemVec3,
        north: *mut TaemVec3,
        east: *mut TaemVec3,
        up: *mut TaemVec3,
    ) -> bool;
    fn taem_runway_frame_create(
        world: *const TaemFrameWorld,
        runway: *const TaemRunway,
        frame: *mut TaemRunwayFrame,
    ) -> bool;
    fn taem_runway_project(
        world: *const TaemFrameWorld,
        frame: *const TaemRunwayFrame,
        position_b: TaemVec3,
        coordinates: *mut TaemRunwayCoordinates,
    ) -> bool;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn remainder(
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
pub struct TaemVec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemFrameWorld {
    pub radius_m: ::core::ffi::c_double,
    pub mu_m3_s2: ::core::ffi::c_double,
    pub rotation_axis_i: TaemVec3,
    pub rotation_rate_rad_s: ::core::ffi::c_double,
    pub rotation_phase_rad_at_ut0: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRunway {
    pub latitude_rad: ::core::ffi::c_double,
    pub longitude_rad: ::core::ffi::c_double,
    pub elevation_m: ::core::ffi::c_double,
    pub heading_rad: ::core::ffi::c_double,
    pub length_m: ::core::ffi::c_double,
    pub width_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRunwayFrame {
    pub threshold_unit_b: TaemVec3,
    pub forward_b: TaemVec3,
    pub right_b: TaemVec3,
    pub elevation_m: ::core::ffi::c_double,
    pub length_m: ::core::ffi::c_double,
    pub width_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemRunwayCoordinates {
    pub position_b_m: TaemVec3,
    pub radius_from_center_m: ::core::ffi::c_double,
    pub altitude_above_datum_m: ::core::ffi::c_double,
    pub altitude_above_runway_m: ::core::ffi::c_double,
    pub runway_along_m: ::core::ffi::c_double,
    pub runway_cross_m: ::core::ffi::c_double,
    pub runway_vertical_m: ::core::ffi::c_double,
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
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
static mut taem_geometry_pi: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
unsafe extern "C" fn degrees(
    mut radians: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return radians * 180.0f64 / taem_geometry_pi;
}
unsafe extern "C" fn wrap_degrees(
    mut degrees_in: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return remainder(degrees_in, 360.0f64);
}
#[no_mangle]
pub unsafe extern "C" fn taem_geometry_world(
    mut m: *const TerminalModel,
    mut w: *mut TaemFrameWorld,
) -> bool {
    if m.is_null() || w.is_null() || !((*m).world.radius_m > 0.0f64)
        || !((*m).world.mu_m3_s2 > 0.0f64)
    {
        return false_0 != 0;
    }
    *w = TaemFrameWorld {
        radius_m: (*m).world.radius_m,
        mu_m3_s2: (*m).world.mu_m3_s2,
        rotation_axis_i: TaemVec3 {
            x: 0.0f64,
            y: 0.0f64,
            z: 1.0f64,
        },
        rotation_rate_rad_s: (*m).world.rotation_rate_rad_s,
        rotation_phase_rad_at_ut0: (*m).world.rotation_phase_rad_at_ut0,
    };
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_geometry_runway(
    mut m: *const TerminalModel,
    mut w: *mut TaemFrameWorld,
    mut f: *mut TaemRunwayFrame,
) -> bool {
    if m.is_null() || w.is_null() || f.is_null() || !taem_geometry_world(m, w) {
        return false_0 != 0;
    }
    let mut runway: TaemRunway = TaemRunway {
        latitude_rad: (*m).site.latitude * taem_geometry_pi / 180.0f64,
        longitude_rad: (*m).site.longitude * taem_geometry_pi / 180.0f64,
        elevation_m: (*m).site.altitude,
        heading_rad: (*m).site.runway_heading * taem_geometry_pi / 180.0f64,
        length_m: (*m).site.runway_length,
        width_m: (*m).site.runway_width,
    };
    return taem_runway_frame_create(w, &raw mut runway, f);
}
#[no_mangle]
pub unsafe extern "C" fn taem_geometry_state(
    mut m: *const TerminalModel,
    mut s: *const TerminalDynamicState,
    mut out: *mut TaemGeometryState,
) -> bool {
    if m.is_null() || s.is_null() || out.is_null() {
        return false_0 != 0;
    }
    let mut w: TaemFrameWorld = TaemFrameWorld {
        radius_m: 0.,
        mu_m3_s2: 0.,
        rotation_axis_i: TaemVec3 { x: 0., y: 0., z: 0. },
        rotation_rate_rad_s: 0.,
        rotation_phase_rad_at_ut0: 0.,
    };
    let mut runway: TaemRunwayFrame = TaemRunwayFrame {
        threshold_unit_b: TaemVec3 { x: 0., y: 0., z: 0. },
        forward_b: TaemVec3 { x: 0., y: 0., z: 0. },
        right_b: TaemVec3 { x: 0., y: 0., z: 0. },
        elevation_m: 0.,
        length_m: 0.,
        width_m: 0.,
    };
    if !taem_geometry_runway(m, &raw mut w, &raw mut runway) {
        return false_0 != 0;
    }
    let mut pi: TaemVec3 = TaemVec3 {
        x: (*s).position_i_m.x,
        y: (*s).position_i_m.y,
        z: (*s).position_i_m.z,
    };
    let mut vi: TaemVec3 = TaemVec3 {
        x: (*s).velocity_i_mps.x,
        y: (*s).velocity_i_mps.y,
        z: (*s).velocity_i_mps.z,
    };
    let mut pb: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    let mut vb: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    if !taem_inertial_to_fixed(&raw mut w, pi, vi, (*s).ut_s, &raw mut pb, &raw mut vb) {
        return false_0 != 0;
    }
    let mut rc: TaemRunwayCoordinates = TaemRunwayCoordinates {
        position_b_m: TaemVec3 { x: 0., y: 0., z: 0. },
        radius_from_center_m: 0.,
        altitude_above_datum_m: 0.,
        altitude_above_runway_m: 0.,
        runway_along_m: 0.,
        runway_cross_m: 0.,
        runway_vertical_m: 0.,
    };
    if !taem_runway_project(&raw mut w, &raw mut runway, pb, &raw mut rc) {
        return false_0 != 0;
    }
    let mut north: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    let mut east: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    let mut up: TaemVec3 = TaemVec3 { x: 0., y: 0., z: 0. };
    if !taem_local_north_east_up(pb, &raw mut north, &raw mut east, &raw mut up) {
        return false_0 != 0;
    }
    let mut vn: ::core::ffi::c_double = taem_vec3_dot(vb, north);
    let mut ve: ::core::ffi::c_double = taem_vec3_dot(vb, east);
    let mut vu: ::core::ffi::c_double = taem_vec3_dot(vb, up);
    let mut horizontal: ::core::ffi::c_double = hypot(vn, ve);
    if !(horizontal > 1e-6f64) || vu.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut course: ::core::ffi::c_double = degrees(atan2(ve, vn));
    let mut airspeed: ::core::ffi::c_double = taem_vec3_norm(vb);
    if airspeed.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut flow: AeroForces = aero_compute(
        &raw const (*m).world,
        &raw const (*m).aero,
        (*s).position_i_m,
        (*s).velocity_i_mps,
        (*s).ut_s,
        (*s).mass_kg,
        (*s).attitude.aoa_rad,
        0.0f64,
    );
    *out = TaemGeometryState {
        runway_along_m: rc.runway_along_m,
        runway_cross_m: rc.runway_cross_m,
        altitude_above_runway_m: rc.altitude_above_runway_m,
        ground_speed_mps: horizontal,
        airspeed_mps: airspeed,
        course_deg: wrap_degrees(course),
        flight_path_angle_deg: degrees(atan2(vu, horizontal)),
        heading_error_deg: wrap_degrees(course - (*m).site.runway_heading),
        curvature_right_per_m: flow.lift_n * sin((*s).attitude.bank_rad)
            / ((*s).mass_kg * horizontal * horizontal),
    };
    return true_0 != 0;
}
