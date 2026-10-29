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
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tan(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn taem_tracker_vertical_lift_demand(
        model: *const TerminalModel,
        state: *const TerminalDynamicState,
        geometry: *const TaemGeometryState,
        desired_gamma_rate_rad_s: ::core::ffi::c_double,
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
pub struct Point {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemSpiralSample {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub heading_rad: ::core::ffi::c_double,
    pub radius_m: ::core::ffi::c_double,
    pub ds_dpsi: ::core::ffi::c_double,
    pub curvature: ::core::ffi::c_double,
    pub curvature_rate: ::core::ffi::c_double,
    pub curvature_acceleration: ::core::ffi::c_double,
}
pub const TAEM_ROUTE_MAX_POINTS: ::core::ffi::c_int = 2048 as ::core::ffi::c_int;
pub const TAEM_ROUTE_LEAD_LUT_POINTS: ::core::ffi::c_int = 129 as ::core::ffi::c_int;
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
static mut planner_pi: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
unsafe extern "C" fn radians(
    mut degrees_0: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return degrees_0 * planner_pi / 180.0f64;
}
unsafe extern "C" fn degrees(
    mut radians_value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return radians_value * 180.0f64 / planner_pi;
}
unsafe extern "C" fn distance(mut a: Point, mut b: Point) -> ::core::ffi::c_double {
    return hypot(a.x - b.x, a.y - b.y);
}
unsafe extern "C" fn clamp(
    mut x: ::core::ffi::c_double,
    mut lo: ::core::ffi::c_double,
    mut hi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return fmax(lo, fmin(hi, x));
}
unsafe extern "C" fn vertical_profile(
    mut m: *const TerminalModel,
    mut start_altitude: ::core::ffi::c_double,
    mut start_fpa_deg: ::core::ffi::c_double,
    mut total_length: ::core::ffi::c_double,
    mut distance_from_start: ::core::ffi::c_double,
    mut midpoint_offset: ::core::ffi::c_double,
    mut local_offset: ::core::ffi::c_double,
    mut local_start_fraction: ::core::ffi::c_double,
    mut local_peak_fraction: ::core::ffi::c_double,
    mut local_end_fraction: ::core::ffi::c_double,
    mut initial_sag: ::core::ffi::c_double,
    mut initial_sag_length: ::core::ffi::c_double,
    mut altitude: *mut ::core::ffi::c_double,
    mut fpa_deg: *mut ::core::ffi::c_double,
    mut vertical_curvature_per_m: *mut ::core::ffi::c_double,
) {
    let mut x: ::core::ffi::c_double = clamp(
        distance_from_start / total_length,
        0.0f64,
        1.0f64,
    );
    let mut x2: ::core::ffi::c_double = x * x;
    let mut x3: ::core::ffi::c_double = x2 * x;
    let mut h0: ::core::ffi::c_double = start_altitude;
    let mut h1: ::core::ffi::c_double = (*m).site.altitude;
    let mut slope0: ::core::ffi::c_double = tan(radians(start_fpa_deg));
    let mut slope1: ::core::ffi::c_double = -tan(
        radians((*m).guidance.taem_glide_slope),
    );
    let mut h00: ::core::ffi::c_double = 2.0f64 * x3 - 3.0f64 * x2 + 1.0f64;
    let mut h10: ::core::ffi::c_double = x3 - 2.0f64 * x2 + x;
    let mut h01: ::core::ffi::c_double = -2.0f64 * x3 + 3.0f64 * x2;
    let mut h11: ::core::ffi::c_double = x3 - x2;
    let mut bow: ::core::ffi::c_double = 16.0f64 * x2 * (1.0f64 - x) * (1.0f64 - x);
    let mut bow_derivative_x: ::core::ffi::c_double = 32.0f64 * x * (1.0f64 - x)
        * (1.0f64 - 2.0f64 * x);
    let mut bow_second_derivative_x: ::core::ffi::c_double = 32.0f64
        * (1.0f64 - 6.0f64 * x + 6.0f64 * x2);
    let mut initial_u: ::core::ffi::c_double = if initial_sag_length > 0.0f64 {
        clamp(distance_from_start / initial_sag_length, 0.0f64, 1.0f64)
    } else {
        0.0f64
    };
    let mut initial_bow: ::core::ffi::c_double = 16.0f64 * initial_u * initial_u
        * (1.0f64 - initial_u) * (1.0f64 - initial_u);
    let mut initial_slope: ::core::ffi::c_double = if initial_sag_length > 0.0f64 {
        initial_sag * 32.0f64 * initial_u * (1.0f64 - initial_u)
            * (1.0f64 - 2.0f64 * initial_u) / initial_sag_length
    } else {
        0.0f64
    };
    let mut initial_second_slope: ::core::ffi::c_double = 0.0f64;
    if initial_sag_length > 0.0f64 && distance_from_start <= initial_sag_length {
        initial_second_slope = initial_sag * 32.0f64
            * (1.0f64 - 6.0f64 * initial_u + 6.0f64 * initial_u * initial_u)
            / (initial_sag_length * initial_sag_length);
    }
    let mut local_bow: ::core::ffi::c_double = 0.0f64;
    let mut local_slope: ::core::ffi::c_double = 0.0f64;
    let mut local_second_slope: ::core::ffi::c_double = 0.0f64;
    let mut local_fall_span: ::core::ffi::c_double = local_peak_fraction
        - local_start_fraction;
    let mut local_recovery_span: ::core::ffi::c_double = local_end_fraction
        - local_peak_fraction;
    if local_fall_span > 1e-6f64 && local_recovery_span > 1e-6f64 {
        if x >= local_start_fraction && x < local_peak_fraction {
            let mut u: ::core::ffi::c_double = clamp(
                (x - local_start_fraction) / local_fall_span,
                0.0f64,
                1.0f64,
            );
            let mut smooth: ::core::ffi::c_double = u * u * (3.0f64 - 2.0f64 * u);
            local_bow = smooth;
            local_slope = local_offset * 6.0f64 * u * (1.0f64 - u)
                / (local_fall_span * total_length);
            local_second_slope = local_offset * 6.0f64 * (1.0f64 - 2.0f64 * u)
                / (local_fall_span * local_fall_span * total_length * total_length);
        } else if x >= local_peak_fraction && x <= local_end_fraction {
            let mut u_0: ::core::ffi::c_double = clamp(
                (x - local_peak_fraction) / local_recovery_span,
                0.0f64,
                1.0f64,
            );
            let mut smooth_0: ::core::ffi::c_double = u_0 * u_0
                * (3.0f64 - 2.0f64 * u_0);
            local_bow = 1.0f64 - smooth_0;
            local_slope = -local_offset * 6.0f64 * u_0 * (1.0f64 - u_0)
                / (local_recovery_span * total_length);
            local_second_slope = -local_offset * 6.0f64 * (1.0f64 - 2.0f64 * u_0)
                / (local_recovery_span * local_recovery_span * total_length
                    * total_length);
        }
    }
    let mut height: ::core::ffi::c_double = h00 * h0 + h10 * total_length * slope0
        + h01 * h1 + h11 * total_length * slope1 + midpoint_offset * bow
        + local_offset * local_bow + initial_sag * initial_bow;
    let mut derivative_x: ::core::ffi::c_double = (6.0f64 * x2 - 6.0f64 * x) * h0
        + (3.0f64 * x2 - 4.0f64 * x + 1.0f64) * total_length * slope0
        + (-6.0f64 * x2 + 6.0f64 * x) * h1
        + (3.0f64 * x2 - 2.0f64 * x) * total_length * slope1;
    derivative_x += midpoint_offset * bow_derivative_x;
    let mut second_derivative_x: ::core::ffi::c_double = (12.0f64 * x - 6.0f64) * h0
        + (6.0f64 * x - 4.0f64) * total_length * slope0 + (-12.0f64 * x + 6.0f64) * h1
        + (6.0f64 * x - 2.0f64) * total_length * slope1;
    second_derivative_x += midpoint_offset * bow_second_derivative_x;
    let mut slope: ::core::ffi::c_double = derivative_x / total_length + initial_slope
        + local_slope;
    if !altitude.is_null() {
        *altitude = height;
    }
    if !fpa_deg.is_null() {
        *fpa_deg = degrees(atan(slope));
    }
    if !vertical_curvature_per_m.is_null() {
        let mut second_slope: ::core::ffi::c_double = second_derivative_x
            / (total_length * total_length) + initial_second_slope + local_second_slope;
        *vertical_curvature_per_m = second_slope / (1.0f64 + slope * slope);
    }
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_build_hac(
    mut m: *const TerminalModel,
    mut start: *const TaemGeometryState,
    mut hac_radius_m: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut sweep_abs_rad: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut maximum_lead_curvature: ::core::ffi::c_double,
    mut route: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    return build_hac(
        m,
        start,
        hac_radius_m,
        side,
        sweep_abs_rad,
        spacing,
        maximum_lead_curvature,
        ::core::f32::INFINITY as ::core::ffi::c_double,
        ::core::f32::NAN as ::core::ffi::c_double,
        route,
        reason,
        reason_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_build_hac_shortest(
    mut m: *const TerminalModel,
    mut start: *const TaemGeometryState,
    mut hac_radius_m: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut sweep_abs_rad: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut maximum_lead_curvature: ::core::ffi::c_double,
    mut peak_curvature_limit: ::core::ffi::c_double,
    mut route: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    if !(peak_curvature_limit > 0.0f64) || peak_curvature_limit.is_finite() as i32 == 0 {
        if !reason.is_null() && reason_size != 0 {
            snprintf(
                reason,
                reason_size,
                b"invalid route input\0" as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    return build_hac(
        m,
        start,
        hac_radius_m,
        side,
        sweep_abs_rad,
        spacing,
        maximum_lead_curvature,
        peak_curvature_limit,
        ::core::f32::NAN as ::core::ffi::c_double,
        route,
        reason,
        reason_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_build_hac_length(
    mut m: *const TerminalModel,
    mut start: *const TaemGeometryState,
    mut hac_radius_m: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut sweep_abs_rad: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut maximum_lead_curvature: ::core::ffi::c_double,
    mut peak_curvature_limit: ::core::ffi::c_double,
    mut target_lead_length_m: ::core::ffi::c_double,
    mut route: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    if !(peak_curvature_limit > 0.0f64) || peak_curvature_limit.is_finite() as i32 == 0
        || !(target_lead_length_m > 0.0f64)
        || target_lead_length_m.is_finite() as i32 == 0
    {
        if !reason.is_null() && reason_size != 0 {
            snprintf(
                reason,
                reason_size,
                b"invalid route input\0" as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return false_0 != 0;
    }
    return build_hac(
        m,
        start,
        hac_radius_m,
        side,
        sweep_abs_rad,
        spacing,
        maximum_lead_curvature,
        peak_curvature_limit,
        target_lead_length_m,
        route,
        reason,
        reason_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_blend_hac_length(
    mut m: *const TerminalModel,
    mut start: *const TaemGeometryState,
    mut a: *const TaemRoute,
    mut b: *const TaemRoute,
    mut spacing: ::core::ffi::c_double,
    mut initial_limit: ::core::ffi::c_double,
    mut peak_limit: ::core::ffi::c_double,
    mut target: ::core::ffi::c_double,
    mut route: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    if a.is_null() || b.is_null() || !(*a).valid || !(*b).valid || (*a).side != (*b).side
        || (*a).hac.radius_m != (*b).hac.radius_m
        || (*a).hac.arc_sweep_rad != (*b).hac.arc_sweep_rad
    {
        return false_0 != 0;
    }
    return build_hac(
        m,
        start,
        (*a).hac.radius_m,
        (*a).side,
        fabs((*a).hac.arc_sweep_rad),
        spacing,
        initial_limit,
        peak_limit,
        target,
        route,
        reason,
        reason_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_lead_peak_curvature(
    mut route: *const TaemRoute,
) -> ::core::ffi::c_double {
    if route.is_null() || !(*route).valid {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut copy: TaemRoute = *route;
    return capture_measure(&raw mut copy, false_0 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_build_fixed_hac(
    mut m: *const TerminalModel,
    mut start: *const TaemGeometryState,
    mut hac_radius_m: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut maximum_lead_curvature: ::core::ffi::c_double,
    mut route: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut reason_size: size_t,
) -> bool {
    return taem_route_build_hac(
        m,
        start,
        hac_radius_m,
        side,
        1.5f64 * planner_pi,
        spacing,
        maximum_lead_curvature,
        route,
        reason,
        reason_size,
    );
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_planned_speed(
    mut route: *const TaemRoute,
    mut station_m: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if route.is_null() || !(*route).profile_speed_tabulated
        || !((*route).profile_total_length_m > 0.0f64)
        || station_m.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let intervals: size_t = (TAEM_ROUTE_PROFILE_LUT_POINTS - 1 as ::core::ffi::c_int)
        as size_t;
    let mut u: ::core::ffi::c_double = clamp(
        station_m / (*route).profile_total_length_m,
        0.0f64,
        1.0f64,
    ) * intervals as ::core::ffi::c_double;
    let mut i: size_t = fmin(floor(u), intervals as ::core::ffi::c_double - 1.0f64)
        as size_t;
    let mut f: ::core::ffi::c_double = u - i as ::core::ffi::c_double;
    return (1.0f64 - f) * (*route).profile_speed_lut[i as usize] as ::core::ffi::c_double
        + f
            * (*route).profile_speed_lut[i.wrapping_add(1 as size_t) as usize]
                as ::core::ffi::c_double;
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_planned_altitude(
    mut route: *const TaemRoute,
    mut station_m: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if route.is_null() || !(*route).profile_tabulated
        || !((*route).profile_total_length_m > 0.0f64)
        || station_m.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let intervals: size_t = (TAEM_ROUTE_PROFILE_LUT_POINTS - 1 as ::core::ffi::c_int)
        as size_t;
    let mut u: ::core::ffi::c_double = clamp(
        station_m / (*route).profile_total_length_m,
        0.0f64,
        1.0f64,
    ) * intervals as ::core::ffi::c_double;
    let mut i: size_t = fmin(floor(u), intervals as ::core::ffi::c_double - 1.0f64)
        as size_t;
    let mut f: ::core::ffi::c_double = u - i as ::core::ffi::c_double;
    return (1.0f64 - f)
        * (*route).profile_altitude_lut[i as usize] as ::core::ffi::c_double
        + f
            * (*route).profile_altitude_lut[i.wrapping_add(1 as size_t) as usize]
                as ::core::ffi::c_double;
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_limit_initial_vertical_authority(
    mut m: *const TerminalModel,
    mut state: *const TerminalDynamicState,
    mut geometry: *const TaemGeometryState,
    mut route: *mut TaemRoute,
) -> bool {
    if m.is_null() || state.is_null() || geometry.is_null() || route.is_null()
        || !(*route).valid || !((*state).mass_kg > 1.0f64)
        || !((*geometry).airspeed_mps > 1.0f64)
        || !((*geometry).ground_speed_mps > 1.0f64)
        || !((*route).profile_total_length_m > 100.0f64)
    {
        return false_0 != 0;
    }
    (*route).profile_initial_sag_m = 0.0f64;
    (*route).profile_initial_sag_length_m = 0.0f64;
    let mut base_altitude: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut base_fpa: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut base_curvature: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut fresh0: TerminalModel = TerminalModel {
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
            altitude: (*route).profile_final_altitude_m,
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
            taem_glide_slope: (*route).profile_final_slope_deg,
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
    vertical_profile(
        &raw mut fresh0,
        (*route).profile_start_altitude_m,
        (*route).profile_start_fpa_deg,
        (*route).profile_total_length_m,
        0.0f64,
        (*route).profile_midpoint_offset_m,
        (*route).profile_local_offset_m,
        (*route).profile_local_start_fraction,
        (*route).profile_local_peak_fraction,
        (*route).profile_local_end_fraction,
        0.0f64,
        0.0f64,
        &raw mut base_altitude,
        &raw mut base_fpa,
        &raw mut base_curvature,
    );
    if base_curvature.is_finite() as i32 == 0 || base_fpa.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut altitude: ::core::ffi::c_double = sqrt(
        (*state).position_i_m.x * (*state).position_i_m.x
            + (*state).position_i_m.y * (*state).position_i_m.y
            + (*state).position_i_m.z * (*state).position_i_m.z,
    ) - (*m).world.radius_m;
    let mut local_radius: ::core::ffi::c_double = (*m).world.radius_m + altitude;
    if !(local_radius > 1.0f64) {
        return false_0 != 0;
    }
    let mut gravity: ::core::ffi::c_double = (*m).world.mu_m3_s2
        / (local_radius * local_radius);
    let mut aoa_max: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_angle_of_attack,
        (*m).aero.alpha_deg[(*m).aero.alpha_count.wrapping_sub(1 as size_t) as usize],
    );
    let mut flow: AeroForces = aero_compute(
        &raw const (*m).world,
        &raw const (*m).aero,
        (*state).position_i_m,
        (*state).velocity_i_mps,
        (*state).ut_s,
        (*state).mass_kg,
        0.0f64,
        0.0f64,
    );
    if flow.mach < 1.0f64
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        aoa_max = fmin(aoa_max, (*m).vehicle.terminal_maximum_lift_angle_of_attack);
    }
    if (*state).trim_aoa_ceiling_rad > 0.0f64 {
        aoa_max = fmin(aoa_max, degrees((*state).trim_aoa_ceiling_rad));
    }
    let mut g_limit: ::core::ffi::c_double = fmax(0.0f64, (*m).vehicle.maximum_g_load)
        * gravity;
    let mut lift_max: ::core::ffi::c_double = 0.0f64;
    let mut aoa: ::core::ffi::c_double = 0.0f64;
    while aoa <= aoa_max + 1e-9f64 {
        let mut f: AeroForces = aero_compute(
            &raw const (*m).world,
            &raw const (*m).aero,
            (*state).position_i_m,
            (*state).velocity_i_mps,
            (*state).ut_s,
            (*state).mass_kg,
            radians(aoa),
            0.0f64,
        );
        if f.lift_n.is_finite() as i32 != 0 {
            lift_max = fmax(lift_max, fmin(fabs(f.lift_n) / (*state).mass_kg, g_limit));
        }
        aoa += 0.5f64;
    }
    if !(lift_max > 1e-6f64) {
        return false_0 != 0;
    }
    let mut lateral_curvature: ::core::ffi::c_double = taem_route_curvature_at_station(
        route,
        0.0f64,
    );
    if lateral_curvature.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut lateral_required: ::core::ffi::c_double = fabs(
        (*geometry).ground_speed_mps * (*geometry).ground_speed_mps * lateral_curvature,
    );
    let mut bank_limit: ::core::ffi::c_double = radians(
        fmin((*m).vehicle.maximum_bank_angle, 80.0f64),
    );
    if lateral_required > lift_max * sin(bank_limit) + 1e-6f64 {
        return false_0 != 0;
    }
    let mut vertical_available: ::core::ffi::c_double = sqrt(
        fmax(0.0f64, lift_max * lift_max - lateral_required * lateral_required),
    );
    vertical_available *= 0.95f64;
    let mut radial_required: ::core::ffi::c_double = taem_tracker_vertical_lift_demand(
        m,
        state,
        geometry,
        0.0f64,
    );
    if radial_required.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut gamma_rate_max: ::core::ffi::c_double = (vertical_available
        - radial_required) / (*geometry).airspeed_mps;
    let mut curvature_max: ::core::ffi::c_double = gamma_rate_max
        / (*geometry).ground_speed_mps;
    if base_curvature <= curvature_max + 1e-9f64 {
        return true_0 != 0;
    }
    let mut pitch_response: ::core::ffi::c_double = if (*m).attitude.pitch_wn > 0.0f64
        && (*m).attitude.pitch_zeta > 0.0f64
    {
        4.0f64 / ((*m).attitude.pitch_wn * (*m).attitude.pitch_zeta)
    } else {
        4.0f64
    };
    let mut sag_length: ::core::ffi::c_double = (*geometry).ground_speed_mps
        * fmax(2.0f64, 2.0f64 * pitch_response);
    sag_length = fmax(
        1000.0f64,
        fmin(sag_length, fmin(6000.0f64, 0.30f64 * (*route).profile_total_length_m)),
    );
    if !(sag_length > 100.0f64) {
        return false_0 != 0;
    }
    let mut slope0: ::core::ffi::c_double = tan(radians((*route).profile_start_fpa_deg));
    let mut delta_second_slope: ::core::ffi::c_double = (curvature_max - base_curvature)
        * (1.0f64 + slope0 * slope0);
    let mut sag: ::core::ffi::c_double = delta_second_slope * sag_length * sag_length
        / 32.0f64;
    let mut drop_0: ::core::ffi::c_double = (*route).profile_start_altitude_m
        - (*route).profile_final_altitude_m;
    let mut sag_limit: ::core::ffi::c_double = fmax(
        50.0f64,
        0.20f64 * fmax(0.0f64, drop_0),
    );
    sag = fmax(-sag_limit, fmin(sag_limit, sag));
    (*route).profile_initial_sag_m = sag;
    (*route).profile_initial_sag_length_m = sag_length;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_curvature_at_station(
    mut route: *const TaemRoute,
    mut station_m: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut position: Point = Point { x: 0., y: 0. };
    let mut course: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut curvature: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if !route_sample_at_station(
        route,
        station_m,
        &raw mut position,
        &raw mut course,
        &raw mut curvature,
    ) {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return curvature;
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_sample(
    mut route: *const TaemRoute,
    mut station: ::core::ffi::c_double,
    mut point: *mut TaemRoutePoint,
) -> bool {
    let mut p: Point = Point { x: 0., y: 0. };
    let mut h: ::core::ffi::c_double = 0.;
    let mut k: ::core::ffi::c_double = 0.;
    if point.is_null()
        || !route_sample_at_station(route, station, &raw mut p, &raw mut h, &raw mut k)
    {
        return false_0 != 0;
    }
    *point = TaemRoutePoint {
        along_m: p.x,
        cross_m: p.y,
        course_deg: h,
        curvature_right_per_m: k,
        altitude_m: taem_route_planned_altitude(route, station),
        flight_path_angle_deg: ::core::f32::NAN as ::core::ffi::c_double,
        distance_from_start_m: station,
    };
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_point_at_index(
    mut route: *const TaemRoute,
    mut index: size_t,
    mut point: *mut TaemRoutePoint,
) -> bool {
    if route.is_null() || !(*route).valid || (*route).count < 2 as size_t
        || point.is_null() || index >= (*route).count
    {
        return false_0 != 0;
    }
    let mut station: ::core::ffi::c_double = route_station(route, index);
    let mut position: Point = Point { x: 0., y: 0. };
    let mut course: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut curvature: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if !route_sample_at_station(
        route,
        station,
        &raw mut position,
        &raw mut course,
        &raw mut curvature,
    ) {
        return false_0 != 0;
    }
    let mut altitude: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if (*route).profile_tabulated {
        altitude = taem_route_planned_altitude(route, station);
    } else if (*route).profile_total_length_m > 0.0f64 {
        let mut f: ::core::ffi::c_double = clamp(
            station / (*route).profile_total_length_m,
            0.0f64,
            1.0f64,
        );
        altitude = (*route).profile_start_altitude_m
            + f
                * ((*route).profile_final_altitude_m
                    - (*route).profile_start_altitude_m);
    }
    *point = TaemRoutePoint {
        along_m: position.x,
        cross_m: position.y,
        course_deg: course,
        curvature_right_per_m: curvature,
        altitude_m: altitude,
        flight_path_angle_deg: ::core::f32::NAN as ::core::ffi::c_double,
        distance_from_start_m: station,
    };
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_route_reference(
    mut route: *const TaemRoute,
    mut state: *const TaemGeometryState,
    mut cursor: *mut size_t,
    mut reference: *mut TaemPathReference,
    mut point_index: *mut size_t,
) -> bool {
    if route.is_null() || !(*route).valid || (*route).count < 3 as size_t
        || state.is_null() || cursor.is_null() || reference.is_null()
        || *cursor >= (*route).count
    {
        return false_0 != 0;
    }
    let mut start: size_t = *cursor;
    let mut lookahead: size_t = 16 as size_t;
    let mut end: size_t = if start.wrapping_add(lookahead) < (*route).count {
        start.wrapping_add(lookahead)
    } else {
        (*route).count.wrapping_sub(1 as size_t)
    };
    let mut best_d2: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut best_station: ::core::ffi::c_double = route_station(route, start);
    let mut best_segment: size_t = start;
    if start == (*route).count.wrapping_sub(1 as size_t) {
        let mut p: Point = route_position(route, start);
        let mut dx: ::core::ffi::c_double = (*state).runway_along_m - p.x;
        let mut dy: ::core::ffi::c_double = (*state).runway_cross_m - p.y;
        best_d2 = dx * dx + dy * dy;
    } else {
        let mut i: size_t = start;
        while i < end {
            let mut a: Point = route_position(route, i);
            let mut b: Point = route_position(route, i.wrapping_add(1 as size_t));
            let mut vx: ::core::ffi::c_double = b.x - a.x;
            let mut vy: ::core::ffi::c_double = b.y - a.y;
            let mut vv: ::core::ffi::c_double = vx * vx + vy * vy;
            if vv > 1e-12f64 {
                let mut wx: ::core::ffi::c_double = (*state).runway_along_m - a.x;
                let mut wy: ::core::ffi::c_double = (*state).runway_cross_m - a.y;
                let mut u: ::core::ffi::c_double = clamp(
                    (wx * vx + wy * vy) / vv,
                    0.0f64,
                    1.0f64,
                );
                let mut q: Point = Point {
                    x: a.x + u * vx,
                    y: a.y + u * vy,
                };
                let mut dx_0: ::core::ffi::c_double = (*state).runway_along_m - q.x;
                let mut dy_0: ::core::ffi::c_double = (*state).runway_cross_m - q.y;
                let mut d2: ::core::ffi::c_double = dx_0 * dx_0 + dy_0 * dy_0;
                if d2 < best_d2 {
                    let mut s0: ::core::ffi::c_double = route_station(route, i);
                    let mut s1: ::core::ffi::c_double = route_station(
                        route,
                        i.wrapping_add(1 as size_t),
                    );
                    best_d2 = d2;
                    best_station = s0 + u * (s1 - s0);
                    best_segment = i;
                }
            }
            i = i.wrapping_add(1);
        }
    }
    if best_d2.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut progress_index: size_t = best_segment;
    if best_segment.wrapping_add(1 as size_t) < (*route).count
        && best_station
            >= route_station(route, best_segment.wrapping_add(1 as size_t)) - 1e-6f64
    {
        progress_index = best_segment.wrapping_add(1 as size_t);
    }
    *cursor = progress_index;
    let mut center: Point = Point { x: 0., y: 0. };
    let mut course: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut curvature: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    if !route_sample_at_station(
        route,
        best_station,
        &raw mut center,
        &raw mut course,
        &raw mut curvature,
    ) {
        return false_0 != 0;
    }
    let mut altitude: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut fpa: ::core::ffi::c_double = ::core::f32::NAN as ::core::ffi::c_double;
    let mut vertical_curvature: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    if (*route).profile_tabulated {
        let intervals: size_t = (TAEM_ROUTE_PROFILE_LUT_POINTS - 1 as ::core::ffi::c_int)
            as size_t;
        let mut length: ::core::ffi::c_double = (*route).profile_total_length_m;
        let mut u_0: ::core::ffi::c_double = clamp(best_station / length, 0.0f64, 1.0f64)
            * intervals as ::core::ffi::c_double;
        let mut i_0: size_t = fmin(
            floor(u_0),
            intervals as ::core::ffi::c_double - 1.0f64,
        ) as size_t;
        let mut f: ::core::ffi::c_double = u_0 - i_0 as ::core::ffi::c_double;
        altitude = (1.0f64 - f)
            * (*route).profile_altitude_lut[i_0 as usize] as ::core::ffi::c_double
            + f
                * (*route).profile_altitude_lut[i_0.wrapping_add(1 as size_t) as usize]
                    as ::core::ffi::c_double;
        fpa = (1.0f64 - f)
            * (*route).profile_fpa_lut[i_0 as usize] as ::core::ffi::c_double
            + f
                * (*route).profile_fpa_lut[i_0.wrapping_add(1 as size_t) as usize]
                    as ::core::ffi::c_double;
        let mut ds: ::core::ffi::c_double = length / intervals as ::core::ffi::c_double;
        vertical_curvature = radians(
            ((*route).profile_fpa_lut[i_0.wrapping_add(1 as size_t) as usize]
                - (*route).profile_fpa_lut[i_0 as usize]) as ::core::ffi::c_double,
        ) / ds;
    } else {
        let mut fresh1: TerminalModel = TerminalModel {
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
                altitude: (*route).profile_final_altitude_m,
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
                taem_glide_slope: (*route).profile_final_slope_deg,
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
        vertical_profile(
            &raw mut fresh1,
            (*route).profile_start_altitude_m,
            (*route).profile_start_fpa_deg,
            (*route).profile_total_length_m,
            best_station,
            (*route).profile_midpoint_offset_m,
            (*route).profile_local_offset_m,
            (*route).profile_local_start_fraction,
            (*route).profile_local_peak_fraction,
            (*route).profile_local_end_fraction,
            (*route).profile_initial_sag_m,
            (*route).profile_initial_sag_length_m,
            &raw mut altitude,
            &raw mut fpa,
            &raw mut vertical_curvature,
        );
    }
    *reference = TaemPathReference {
        runway_along_m: center.x,
        runway_cross_m: center.y,
        course_deg: course,
        curvature_right_per_m: curvature,
        altitude_m: altitude,
        flight_path_angle_deg: fpa,
        vertical_curvature_per_m: vertical_curvature,
        station_m: best_station,
    };
    if !point_index.is_null() {
        *point_index = progress_index;
    }
    return true_0 != 0;
}
unsafe extern "C" fn bezier_eval(
    mut p: *const TaemPoint2,
    mut degree: ::core::ffi::c_int,
    mut t: ::core::ffi::c_double,
) -> Point {
    let mut q: [Point; 6] = [Point { x: 0., y: 0. }; 6];
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i <= degree {
        q[i as usize] = Point {
            x: (*p.offset(i as isize)).x,
            y: (*p.offset(i as isize)).y,
        };
        i += 1;
    }
    let mut n: ::core::ffi::c_int = degree;
    while n > 0 as ::core::ffi::c_int {
        let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_0 < n {
            q[i_0 as usize] = Point {
                x: (1 as ::core::ffi::c_int as ::core::ffi::c_double - t)
                    * q[i_0 as usize].x
                    + t * q[(i_0 + 1 as ::core::ffi::c_int) as usize].x,
                y: (1 as ::core::ffi::c_int as ::core::ffi::c_double - t)
                    * q[i_0 as usize].y
                    + t * q[(i_0 + 1 as ::core::ffi::c_int) as usize].y,
            };
            i_0 += 1;
        }
        n -= 1;
    }
    return q[0 as ::core::ffi::c_int as usize];
}
unsafe extern "C" fn capture_sample(
    mut r: *const TaemRoute,
    mut t: ::core::ffi::c_double,
    mut p: *mut Point,
    mut h: *mut ::core::ffi::c_double,
    mut k: *mut ::core::ffi::c_double,
) -> bool {
    let mut dp: [TaemPoint2; 5] = [TaemPoint2 { x: 0., y: 0. }; 5];
    let mut ddp: [TaemPoint2; 4] = [TaemPoint2 { x: 0., y: 0. }; 4];
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 5 as ::core::ffi::c_int {
        dp[i as usize] = TaemPoint2 {
            x: 5 as ::core::ffi::c_int as ::core::ffi::c_double
                * ((*r).capture_control[(i + 1 as ::core::ffi::c_int) as usize].x
                    - (*r).capture_control[i as usize].x),
            y: 5 as ::core::ffi::c_int as ::core::ffi::c_double
                * ((*r).capture_control[(i + 1 as ::core::ffi::c_int) as usize].y
                    - (*r).capture_control[i as usize].y),
        };
        i += 1;
    }
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 4 as ::core::ffi::c_int {
        ddp[i_0 as usize] = TaemPoint2 {
            x: 4 as ::core::ffi::c_int as ::core::ffi::c_double
                * (dp[(i_0 + 1 as ::core::ffi::c_int) as usize].x - dp[i_0 as usize].x),
            y: 4 as ::core::ffi::c_int as ::core::ffi::c_double
                * (dp[(i_0 + 1 as ::core::ffi::c_int) as usize].y - dp[i_0 as usize].y),
        };
        i_0 += 1;
    }
    let mut d: Point = bezier_eval(
        &raw mut dp as *mut TaemPoint2,
        4 as ::core::ffi::c_int,
        t,
    );
    let mut dd: Point = bezier_eval(
        &raw mut ddp as *mut TaemPoint2,
        3 as ::core::ffi::c_int,
        t,
    );
    let mut norm: ::core::ffi::c_double = hypot(d.x, d.y);
    if !(norm > 1e-6f64) {
        return false_0 != 0;
    }
    *p = bezier_eval(
        &raw const (*r).capture_control as *const TaemPoint2,
        5 as ::core::ffi::c_int,
        t,
    );
    *h = atan2(d.y, d.x);
    *k = (d.x * dd.y - d.y * dd.x) / (norm * norm * norm);
    return (*k).is_finite() as i32 != 0;
}
unsafe extern "C" fn capture_measure(
    mut r: *mut TaemRoute,
    mut lookup: bool,
) -> ::core::ffi::c_double {
    let mut previous: Point = Point {
        x: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: 0.,
    };
    let mut p: Point = Point { x: 0., y: 0. };
    let mut h: ::core::ffi::c_double = 0.;
    let mut k: ::core::ffi::c_double = 0.;
    let mut peak: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut length: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < TAEM_ROUTE_LEAD_LUT_POINTS {
        if !capture_sample(
            r,
            i as ::core::ffi::c_double
                / (TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int)
                    as ::core::ffi::c_double,
            &raw mut p,
            &raw mut h,
            &raw mut k,
        ) {
            return ::core::f32::INFINITY as ::core::ffi::c_double;
        }
        peak = fmax(peak, fabs(k));
        if i != 0 {
            length += distance(p, previous);
        }
        if lookup {
            (*r).lead_arc_fraction_lut[i as usize] = length as ::core::ffi::c_float;
        }
        previous = p;
        i += 1;
    }
    if lookup {
        (*r).lead_length_m = length;
        if !(length > 0 as ::core::ffi::c_int as ::core::ffi::c_double) {
            return ::core::f32::INFINITY as ::core::ffi::c_double;
        }
        let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_0 < TAEM_ROUTE_LEAD_LUT_POINTS {
            (*r).lead_arc_fraction_lut[i_0 as usize] = ((*r)
                .lead_arc_fraction_lut[i_0 as usize] as ::core::ffi::c_double / length)
                as ::core::ffi::c_float;
            i_0 += 1;
        }
    }
    return peak;
}
unsafe extern "C" fn capture_controls(
    mut r: *mut TaemRoute,
    mut p0: Point,
    mut p5: Point,
    mut h0: ::core::ffi::c_double,
    mut h1: ::core::ffi::c_double,
    mut k0: ::core::ffi::c_double,
    mut k1: ::core::ffi::c_double,
    mut v0: ::core::ffi::c_double,
    mut v1: ::core::ffi::c_double,
) {
    let mut d0: Point = Point {
        x: v0 * cos(h0),
        y: v0 * sin(h0),
    };
    let mut d1: Point = Point {
        x: v1 * cos(h1),
        y: v1 * sin(h1),
    };
    let mut a0: Point = Point {
        x: -k0 * v0 * v0 * sin(h0),
        y: k0 * v0 * v0 * cos(h0),
    };
    let mut a1: Point = Point {
        x: -k1 * v1 * v1 * sin(h1),
        y: k1 * v1 * v1 * cos(h1),
    };
    (*r).capture_control[0 as ::core::ffi::c_int as usize] = TaemPoint2 {
        x: p0.x,
        y: p0.y,
    };
    (*r).capture_control[1 as ::core::ffi::c_int as usize] = TaemPoint2 {
        x: p0.x + d0.x / 5 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: p0.y + d0.y / 5 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    (*r).capture_control[2 as ::core::ffi::c_int as usize] = TaemPoint2 {
        x: p0.x
            + 2 as ::core::ffi::c_int as ::core::ffi::c_double * d0.x
                / 5 as ::core::ffi::c_int as ::core::ffi::c_double
            + a0.x / 20 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: p0.y
            + 2 as ::core::ffi::c_int as ::core::ffi::c_double * d0.y
                / 5 as ::core::ffi::c_int as ::core::ffi::c_double
            + a0.y / 20 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    (*r).capture_control[3 as ::core::ffi::c_int as usize] = TaemPoint2 {
        x: p5.x
            - 2 as ::core::ffi::c_int as ::core::ffi::c_double * d1.x
                / 5 as ::core::ffi::c_int as ::core::ffi::c_double
            + a1.x / 20 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: p5.y
            - 2 as ::core::ffi::c_int as ::core::ffi::c_double * d1.y
                / 5 as ::core::ffi::c_int as ::core::ffi::c_double
            + a1.y / 20 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    (*r).capture_control[4 as ::core::ffi::c_int as usize] = TaemPoint2 {
        x: p5.x - d1.x / 5 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: p5.y - d1.y / 5 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    (*r).capture_control[5 as ::core::ffi::c_int as usize] = TaemPoint2 {
        x: p5.x,
        y: p5.y,
    };
}
unsafe extern "C" fn unload_heading(
    mut r: *const TaemRoute,
    mut s: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut u: ::core::ffi::c_double = clamp(
        s / (*r).unload_length_m,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    return (*r).spiral_exit_heading_rad
        + (*r).side / (*r).spiral_exit_radius_m * (*r).unload_length_m
            * (u - u * u * u + 0.5f64 * u * u * u * u);
}
unsafe extern "C" fn unload_delta(
    mut r: *const TaemRoute,
    mut s: ::core::ffi::c_double,
) -> Point {
    static mut x: [::core::ffi::c_double; 4] = [
        0.1834346424956498f64,
        0.5255324099163290f64,
        0.7966664774136267f64,
        0.9602898564975363f64,
    ];
    static mut w: [::core::ffi::c_double; 4] = [
        0.3626837833783620f64,
        0.3137066458778873f64,
        0.2223810344533745f64,
        0.1012285362903763f64,
    ];
    let mut p: Point = Point {
        x: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        y: 0.,
    };
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 4 as ::core::ffi::c_int {
        let mut sign: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
        while sign <= 1 as ::core::ffi::c_int {
            let mut h: ::core::ffi::c_double = unload_heading(
                r,
                0.5f64 * s
                    * (1 as ::core::ffi::c_int as ::core::ffi::c_double
                        + sign as ::core::ffi::c_double * x[i as usize]),
            );
            p.x += 0.5f64 * s * w[i as usize] * cos(h);
            p.y += 0.5f64 * s * w[i as usize] * sin(h);
            sign += 2 as ::core::ffi::c_int;
        }
        i += 1;
    }
    return p;
}
unsafe extern "C" fn route_sample_at_station(
    mut r: *const TaemRoute,
    mut station: ::core::ffi::c_double,
    mut p: *mut Point,
    mut course: *mut ::core::ffi::c_double,
    mut k: *mut ::core::ffi::c_double,
) -> bool {
    if r.is_null() || !(*r).valid || station.is_finite() as i32 == 0
        || !((*r).spiral_exit_radius_m
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
    {
        return false_0 != 0;
    }
    station = clamp(
        station,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        (*r).length_m,
    );
    let mut h: ::core::ffi::c_double = 0.;
    if station < (*r).lead_length_m {
        let mut f: ::core::ffi::c_double = station / (*r).lead_length_m;
        let mut i: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
        while i < TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int
            && ((*r).lead_arc_fraction_lut[i as usize] as ::core::ffi::c_double) < f
        {
            i += 1;
        }
        let mut a: ::core::ffi::c_double = (*r)
            .lead_arc_fraction_lut[(i - 1 as ::core::ffi::c_int) as usize]
            as ::core::ffi::c_double;
        let mut b: ::core::ffi::c_double = (*r).lead_arc_fraction_lut[i as usize]
            as ::core::ffi::c_double;
        let mut t: ::core::ffi::c_double = ((i - 1 as ::core::ffi::c_int)
            as ::core::ffi::c_double + (f - a) / fmax(b - a, 1e-12f64))
            / (TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int)
                as ::core::ffi::c_double;
        if !capture_sample(r, t, p, &raw mut h, k) {
            return false_0 != 0;
        }
    } else if station < (*r).lead_length_m + (*r).hac.arc_length_m {
        let mut remaining: ::core::ffi::c_double = (*r).hac.arc_length_m
            - (station - (*r).lead_length_m);
        let mut i_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
        while i_0 < TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int
            && (*r).spiral_arc_lut[i_0 as usize] < remaining
        {
            i_0 += 1;
        }
        let mut a_0: ::core::ffi::c_double = (*r)
            .spiral_arc_lut[(i_0 - 1 as ::core::ffi::c_int) as usize];
        let mut b_0: ::core::ffi::c_double = (*r).spiral_arc_lut[i_0 as usize];
        let mut sweep: ::core::ffi::c_double = fabs((*r).hac.arc_sweep_rad);
        let mut step: ::core::ffi::c_double = sweep
            / (TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int)
                as ::core::ffi::c_double;
        let mut psi: ::core::ffi::c_double = step
            * ((i_0 - 1 as ::core::ffi::c_int) as ::core::ffi::c_double
                + (remaining - a_0) / (b_0 - a_0));
        let mut q: TaemSpiralSample = TaemSpiralSample {
            x: 0.,
            y: 0.,
            heading_rad: 0.,
            radius_m: 0.,
            ds_dpsi: 0.,
            curvature: 0.,
            curvature_rate: 0.,
            curvature_acceleration: 0.,
        };
        let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while j < 2 as ::core::ffi::c_int {
            if !taem_spiral_sample(
                (*r).spiral_exit_radius_m,
                (*r).spiral_coefficient_m,
                psi,
                (*r).side,
                (*r).spiral_exit_heading_rad,
                (*r).hac.center.x,
                (*r).hac.center.y,
                &raw mut q,
            ) {
                return false_0 != 0;
            }
            psi = clamp(
                psi
                    - (taem_spiral_arc(
                        (*r).spiral_exit_radius_m,
                        (*r).spiral_coefficient_m,
                        psi,
                    ) - remaining) / q.ds_dpsi,
                0 as ::core::ffi::c_int as ::core::ffi::c_double,
                sweep,
            );
            j += 1;
        }
        if !taem_spiral_sample(
            (*r).spiral_exit_radius_m,
            (*r).spiral_coefficient_m,
            psi,
            (*r).side,
            (*r).spiral_exit_heading_rad,
            (*r).hac.center.x,
            (*r).hac.center.y,
            &raw mut q,
        ) {
            return false_0 != 0;
        }
        *p = Point { x: q.x, y: q.y };
        h = q.heading_rad;
        *k = q.curvature;
    } else {
        let mut s: ::core::ffi::c_double = station - (*r).lead_length_m
            - (*r).hac.arc_length_m;
        let mut u: ::core::ffi::c_double = clamp(
            s / (*r).unload_length_m,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
        let mut d: Point = unload_delta(r, fmin(s, (*r).unload_length_m));
        *p = Point {
            x: (*r).hac.exit.x + d.x
                + fmax(
                    0 as ::core::ffi::c_int as ::core::ffi::c_double,
                    s - (*r).unload_length_m,
                ),
            y: (*r).hac.exit.y + d.y,
        };
        h = unload_heading(r, s);
        *k = (*r).side / (*r).spiral_exit_radius_m
            * (1 as ::core::ffi::c_int as ::core::ffi::c_double
                - 3 as ::core::ffi::c_int as ::core::ffi::c_double * u * u
                + 2 as ::core::ffi::c_int as ::core::ffi::c_double * u * u * u);
    }
    *course = (*r).runway_heading_deg + degrees(h);
    return (*k).is_finite() as i32 != 0 && (*p).x.is_finite() as i32 != 0
        && (*course).is_finite() as i32 != 0;
}
unsafe extern "C" fn route_station(
    mut r: *const TaemRoute,
    mut index: size_t,
) -> ::core::ffi::c_double {
    return taem_route_station_at_index(r, index);
}
unsafe extern "C" fn route_position(
    mut r: *const TaemRoute,
    mut index: size_t,
) -> Point {
    let mut p: Point = Point {
        x: ::core::f32::NAN as ::core::ffi::c_double,
        y: ::core::f32::NAN as ::core::ffi::c_double,
    };
    let mut h: ::core::ffi::c_double = 0.;
    let mut k: ::core::ffi::c_double = 0.;
    route_sample_at_station(
        r,
        route_station(r, index),
        &raw mut p,
        &raw mut h,
        &raw mut k,
    );
    return p;
}
unsafe extern "C" fn build_hac(
    mut m: *const TerminalModel,
    mut start: *const TaemGeometryState,
    mut ri: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut sweep: ::core::ffi::c_double,
    mut spacing: ::core::ffi::c_double,
    mut initial_limit: ::core::ffi::c_double,
    mut peak_limit: ::core::ffi::c_double,
    mut target_length: ::core::ffi::c_double,
    mut r: *mut TaemRoute,
    mut reason: *mut ::core::ffi::c_char,
    mut size: size_t,
) -> bool {
    if !reason.is_null() && size != 0 {
        snprintf(
            reason,
            size,
            b"spiral geometry or acquisition is infeasible\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    if m.is_null() || start.is_null() || r.is_null()
        || !(ri > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || fabs(side) != 1 as ::core::ffi::c_int as ::core::ffi::c_double
        || !(sweep > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || sweep > 8 as ::core::ffi::c_int as ::core::ffi::c_double * planner_pi
        || !(spacing >= 100 as ::core::ffi::c_int as ::core::ffi::c_double)
        || !(initial_limit > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || (ri + sweep + spacing + initial_limit).is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    memset(
        r as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemRoute>() as size_t,
    );
    let mut vf: ::core::ffi::c_double = fmax(
        (*m).guidance.final_alignment_speed,
        (*m).vehicle.minimum_safe_speed,
    );
    let mut vi: ::core::ffi::c_double = (*start).ground_speed_mps;
    let mut gravity: ::core::ffi::c_double = (*m).world.mu_m3_s2
        / pow(
            (*m).world.radius_m + (*m).site.altitude,
            2 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    let mut bank: ::core::ffi::c_double = radians((*m).vehicle.maximum_bank_angle);
    if !(vf > 0 as ::core::ffi::c_int as ::core::ffi::c_double && vi > vf
        && gravity > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && bank > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && bank < planner_pi / 2 as ::core::ffi::c_int as ::core::ffi::c_double)
    {
        return false_0 != 0;
    }
    let mut rf: ::core::ffi::c_double = fmax(
        ri * vf * vf / (vi * vi),
        vf * vf / (gravity * tan(bank)),
    );
    rf = fmax(
        rf,
        ri
            / (1 as ::core::ffi::c_int as ::core::ffi::c_double
                + sweep * sweep / 12 as ::core::ffi::c_int as ::core::ffi::c_double),
    );
    if !(rf < ri) {
        return false_0 != 0;
    }
    (*r).side = side;
    (*r).spiral_exit_radius_m = rf;
    (*r).spiral_coefficient_m = (ri - rf)
        / pow(sweep, 4 as ::core::ffi::c_int as ::core::ffi::c_double);
    let mut wn: ::core::ffi::c_double = (*m).attitude.roll_wn;
    let mut zeta: ::core::ffi::c_double = (*m).attitude.roll_zeta;
    let mut rate: ::core::ffi::c_double = (*m).attitude.max_roll_rate_rad_s;
    let mut accel: ::core::ffi::c_double = (*m).attitude.max_roll_accel_rad_s2;
    if !(wn > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && zeta > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && rate > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && accel > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
    {
        return false_0 != 0;
    }
    /* With nominal constant vf, bank=atan(turn_ratio*f(t/T)), where
       |f'|<=1.5 and |f''|<=6. Bound both derivatives of atan, rather than
       scaling the bank angle itself (e23dd5c). */
    let mut turn_ratio: ::core::ffi::c_double = vf * vf / (gravity * rf);
    let mut settle: ::core::ffi::c_double = 4 as ::core::ffi::c_int
        as ::core::ffi::c_double / (wn * zeta);
    let mut unload_time: ::core::ffi::c_double = fmax(
        settle,
        fmax(
            1.5f64 * turn_ratio / rate,
            sqrt((6.0f64 * turn_ratio + 4.5f64 * pow(turn_ratio, 3.0f64)) / accel),
        ),
    );
    (*r).unload_length_m = vf * unload_time;
    (*r).settle_length_m = vf * settle;
    (*r).rollout_length_m = (*r).unload_length_m + (*r).settle_length_m;
    (*r).spiral_exit_heading_rad = -side * (*r).unload_length_m
        / (2 as ::core::ffi::c_int as ::core::ffi::c_double * rf);
    (*r).alignment_along_m = -(*m).guidance.final_approach_distance;
    let mut delta: Point = unload_delta(r, (*r).unload_length_m);
    (*r).hac.exit = TaemPoint2 {
        x: (*r).alignment_along_m - (*r).settle_length_m - delta.x,
        y: -delta.y,
    };
    let mut theta: ::core::ffi::c_double = (*r).spiral_exit_heading_rad
        - side * planner_pi / 2 as ::core::ffi::c_int as ::core::ffi::c_double;
    (*r).hac.center = TaemPoint2 {
        x: (*r).hac.exit.x - rf * cos(theta),
        y: (*r).hac.exit.y - rf * sin(theta),
    };
    let mut entry: TaemSpiralSample = TaemSpiralSample {
        x: 0.,
        y: 0.,
        heading_rad: 0.,
        radius_m: 0.,
        ds_dpsi: 0.,
        curvature: 0.,
        curvature_rate: 0.,
        curvature_acceleration: 0.,
    };
    if !taem_spiral_sample(
        rf,
        (*r).spiral_coefficient_m,
        sweep,
        side,
        (*r).spiral_exit_heading_rad,
        (*r).hac.center.x,
        (*r).hac.center.y,
        &raw mut entry,
    ) {
        return false_0 != 0;
    }
    (*r).hac.entry = TaemPoint2 {
        x: entry.x,
        y: entry.y,
    };
    (*r).hac.radius_m = ri;
    (*r).hac.side = side;
    (*r).hac.capture_course_deg = (*m).site.runway_heading + degrees(entry.heading_rad);
    (*r).hac.arc_sweep_rad = side * sweep;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < TAEM_ROUTE_LEAD_LUT_POINTS {
        let mut psi: ::core::ffi::c_double = sweep * i as ::core::ffi::c_double
            / (TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int)
                as ::core::ffi::c_double;
        let mut q: TaemSpiralSample = TaemSpiralSample {
            x: 0.,
            y: 0.,
            heading_rad: 0.,
            radius_m: 0.,
            ds_dpsi: 0.,
            curvature: 0.,
            curvature_rate: 0.,
            curvature_acceleration: 0.,
        };
        if !taem_spiral_sample(
            rf,
            (*r).spiral_coefficient_m,
            psi,
            side,
            (*r).spiral_exit_heading_rad,
            (*r).hac.center.x,
            (*r).hac.center.y,
            &raw mut q,
        ) || side * q.curvature <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            return false_0 != 0;
        }
        (*r).spiral_arc_lut[i as usize] = taem_spiral_arc(
            rf,
            (*r).spiral_coefficient_m,
            psi,
        );
        i += 1;
    }
    (*r).hac.arc_length_m = (*r)
        .spiral_arc_lut[(TAEM_ROUTE_LEAD_LUT_POINTS - 1 as ::core::ffi::c_int) as usize];
    let mut p0: Point = Point {
        x: (*start).runway_along_m,
        y: (*start).runway_cross_m,
    };
    let mut p5: Point = Point { x: entry.x, y: entry.y };
    let mut direct: ::core::ffi::c_double = distance(p0, p5);
    let mut best: ::core::ffi::c_double = ::core::f32::INFINITY as ::core::ffi::c_double;
    let mut controls: [TaemPoint2; 6] = [TaemPoint2 { x: 0., y: 0. }; 6];
    let mut found: bool = false_0 != 0;
    let mut k0: ::core::ffi::c_double = (*start).curvature_right_per_m;
    if k0.is_finite() as i32 == 0 || fabs(k0) > initial_limit {
        return false_0 != 0;
    }
    let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i_0 < 18 as ::core::ffi::c_int {
        let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while j < 18 as ::core::ffi::c_int {
            let mut v0: ::core::ffi::c_double = fmax(spacing, direct * 0.25f64)
                * pow(1.2f64, i_0 as ::core::ffi::c_double);
            let mut v1: ::core::ffi::c_double = fmax(spacing, direct * 0.25f64)
                * pow(1.2f64, j as ::core::ffi::c_double);
            capture_controls(
                r,
                p0,
                p5,
                radians((*start).course_deg - (*m).site.runway_heading),
                entry.heading_rad,
                k0,
                entry.curvature,
                v0,
                v1,
            );
            let mut peak: ::core::ffi::c_double = capture_measure(r, true_0 != 0);
            if !(peak.is_finite() as i32 == 0 || peak > peak_limit) {
                let mut score: ::core::ffi::c_double = if target_length.is_finite()
                    as i32 != 0
                {
                    fabs((*r).lead_length_m - target_length)
                } else if peak_limit.is_finite() as i32 != 0 {
                    (*r).lead_length_m
                } else {
                    peak
                };
                if score < best {
                    best = score;
                    memcpy(
                        &raw mut controls as *mut TaemPoint2 as *mut ::core::ffi::c_void,
                        &raw mut (*r).capture_control as *mut TaemPoint2
                            as *const ::core::ffi::c_void,
                        ::core::mem::size_of::<[TaemPoint2; 6]>() as size_t,
                    );
                    found = true_0 != 0;
                }
            }
            j += 1;
        }
        i_0 += 1;
    }
    if !found {
        return false_0 != 0;
    }
    memcpy(
        &raw mut (*r).capture_control as *mut TaemPoint2 as *mut ::core::ffi::c_void,
        &raw mut controls as *mut TaemPoint2 as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[TaemPoint2; 6]>() as size_t,
    );
    capture_measure(r, true_0 != 0);
    (*r).p0_along_m = p0.x;
    (*r).p0_cross_m = p0.y;
    (*r).p3_along_m = p5.x;
    (*r).p3_cross_m = p5.y;
    (*r).p1_along_m = controls[1 as ::core::ffi::c_int as usize].x;
    (*r).p1_cross_m = controls[1 as ::core::ffi::c_int as usize].y;
    (*r).p2_along_m = controls[4 as ::core::ffi::c_int as usize].x;
    (*r).p2_cross_m = controls[4 as ::core::ffi::c_int as usize].y;
    (*r).initial_course_deg = (*start).course_deg;
    (*r).runway_heading_deg = (*m).site.runway_heading;
    (*r).lead_count = fmax(
        2 as ::core::ffi::c_int as ::core::ffi::c_double,
        ceil((*r).lead_length_m / spacing),
    ) as size_t;
    (*r).arc_count = fmax(
        3 as ::core::ffi::c_int as ::core::ffi::c_double,
        ceil((*r).hac.arc_length_m / spacing),
    ) as size_t;
    (*r).rollout_count = fmax(
        2 as ::core::ffi::c_int as ::core::ffi::c_double,
        ceil((*r).rollout_length_m / spacing),
    ) as size_t;
    (*r).count = (1 as size_t)
        .wrapping_add((*r).lead_count)
        .wrapping_add((*r).arc_count)
        .wrapping_add((*r).rollout_count);
    if (*r).count > TAEM_ROUTE_MAX_POINTS as size_t {
        return false_0 != 0;
    }
    (*r).length_m = (*r).lead_length_m + (*r).hac.arc_length_m + (*r).rollout_length_m;
    (*r).hac.total_length_m = (*r).hac.arc_length_m + (*r).rollout_length_m;
    (*r).hac.final_length_m = (*r).rollout_length_m;
    (*r).profile_total_length_m = (*r).length_m;
    (*r).profile_start_altitude_m = (*m).site.altitude
        + (*start).altitude_above_runway_m;
    (*r).profile_start_fpa_deg = (*start).flight_path_angle_deg;
    (*r).profile_final_altitude_m = (*m).site.altitude
        + (*m).guidance.final_approach_distance
            * tan(radians((*m).guidance.final_glide_slope));
    (*r).profile_final_slope_deg = (*m).guidance.final_glide_slope;
    (*r).profile_local_start_fraction = 0.08f64;
    (*r).profile_local_peak_fraction = 0.24f64;
    (*r).profile_local_end_fraction = 0.40f64;
    (*r).valid = true_0 != 0;
    if !reason.is_null() && size != 0 {
        snprintf(
            reason,
            size,
            b"spiral geometry built; native qualification required\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    return true_0 != 0;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
#[inline]
unsafe extern "C" fn taem_spiral_sample(
    mut rf: ::core::ffi::c_double,
    mut a: ::core::ffi::c_double,
    mut psi: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut exit_heading: ::core::ffi::c_double,
    mut cx: ::core::ffi::c_double,
    mut cy: ::core::ffi::c_double,
    mut out: *mut TaemSpiralSample,
) -> bool {
    if out.is_null() || !(rf > 0.0f64) || !(a >= 0.0f64) || !(psi >= 0.0f64)
        || (rf + a + psi + exit_heading + cx + cy).is_finite() as i32 == 0
        || fabs(side) != 1.0f64
    {
        return false_0 != 0;
    }
    let mut r: ::core::ffi::c_double = rf + a * psi * psi * psi * psi;
    let mut rp: ::core::ffi::c_double = 4 as ::core::ffi::c_int as ::core::ffi::c_double
        * a * psi * psi * psi;
    let mut rpp: ::core::ffi::c_double = 12 as ::core::ffi::c_int
        as ::core::ffi::c_double * a * psi * psi;
    let mut rppp: ::core::ffi::c_double = 24 as ::core::ffi::c_int
        as ::core::ffi::c_double * a * psi;
    let mut rpppp: ::core::ffi::c_double = 24 as ::core::ffi::c_int
        as ::core::ffi::c_double * a;
    let mut theta: ::core::ffi::c_double = exit_heading
        - side
            * (acos(-1.0f64) / 2 as ::core::ffi::c_int as ::core::ffi::c_double + psi);
    let mut c: ::core::ffi::c_double = cos(theta);
    let mut s: ::core::ffi::c_double = sin(theta);
    let mut q: ::core::ffi::c_double = r * r + rp * rp;
    let mut root: ::core::ffi::c_double = sqrt(q);
    let mut n: ::core::ffi::c_double = r * r
        + 2 as ::core::ffi::c_int as ::core::ffi::c_double * rp * rp - r * rpp;
    let mut np: ::core::ffi::c_double = 2 as ::core::ffi::c_int as ::core::ffi::c_double
        * r * rp + 3 as ::core::ffi::c_int as ::core::ffi::c_double * rp * rpp
        - r * rppp;
    let mut npp: ::core::ffi::c_double = 2 as ::core::ffi::c_int as ::core::ffi::c_double
        * rp * rp + 2 as ::core::ffi::c_int as ::core::ffi::c_double * r * rpp
        + 3 as ::core::ffi::c_int as ::core::ffi::c_double * rpp * rpp
        + 2 as ::core::ffi::c_int as ::core::ffi::c_double * rp * rppp - r * rpppp;
    let mut b: ::core::ffi::c_double = r * rp + rp * rpp;
    let mut bp: ::core::ffi::c_double = rp * rp + r * rpp + rpp * rpp + rp * rppp;
    let mut kp: ::core::ffi::c_double = side
        * (np - 3 as ::core::ffi::c_int as ::core::ffi::c_double * n * b / q)
        / (q * root);
    let mut kpp: ::core::ffi::c_double = side
        * (npp
            - (6 as ::core::ffi::c_int as ::core::ffi::c_double * np * b
                + 3 as ::core::ffi::c_int as ::core::ffi::c_double * n * bp) / q
            + 15 as ::core::ffi::c_int as ::core::ffi::c_double * n * b * b / (q * q))
        / (q * root);
    *out = TaemSpiralSample {
        x: cx + r * c,
        y: cy + r * s,
        heading_rad: atan2(-rp * s + side * r * c, -rp * c - side * r * s),
        radius_m: r,
        ds_dpsi: root,
        curvature: side * n / (q * root),
        curvature_rate: -kp / root,
        curvature_acceleration: kpp / q - kp * b / (q * q),
    };
    return (*out).curvature_acceleration.is_finite() as i32 != 0
        && (*out).x.is_finite() as i32 != 0;
}
#[inline]
unsafe extern "C" fn taem_spiral_arc(
    mut rf: ::core::ffi::c_double,
    mut a: ::core::ffi::c_double,
    mut psi: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(rf > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || !(a >= 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || !(psi >= 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || (rf + a + psi).is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    static mut x: [::core::ffi::c_double; 4] = [
        0.1834346424956498f64,
        0.5255324099163290f64,
        0.7966664774136267f64,
        0.9602898564975363f64,
    ];
    static mut w: [::core::ffi::c_double; 4] = [
        0.3626837833783620f64,
        0.3137066458778873f64,
        0.2223810344533745f64,
        0.1012285362903763f64,
    ];
    let mut panels: ::core::ffi::c_int = ceil(psi / 0.25f64) as ::core::ffi::c_int;
    if panels < 1 as ::core::ffi::c_int {
        return 0 as ::core::ffi::c_int as ::core::ffi::c_double;
    }
    let mut sum: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut width: ::core::ffi::c_double = psi / panels as ::core::ffi::c_double;
    let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while j < panels {
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < 4 as ::core::ffi::c_int {
            let mut sign: ::core::ffi::c_int = -(1 as ::core::ffi::c_int);
            while sign <= 1 as ::core::ffi::c_int {
                let mut p: ::core::ffi::c_double = width
                    * (j as ::core::ffi::c_double + 0.5f64
                        + 0.5f64 * sign as ::core::ffi::c_double * x[i as usize]);
                let mut r: ::core::ffi::c_double = rf + a * p * p * p * p;
                let mut rp: ::core::ffi::c_double = 4 as ::core::ffi::c_int
                    as ::core::ffi::c_double * a * p * p * p;
                sum += 0.5f64 * width * w[i as usize] * hypot(r, rp);
                sign += 2 as ::core::ffi::c_int;
            }
            i += 1;
        }
        j += 1;
    }
    return sum;
}
