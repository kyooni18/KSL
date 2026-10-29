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
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn remainder(
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
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
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
static mut reachability_pi: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
unsafe extern "C" fn radians(
    mut degrees_0: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return degrees_0 * reachability_pi / 180.0f64;
}
unsafe extern "C" fn degrees(
    mut radians_value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return radians_value * 180.0f64 / reachability_pi;
}
#[no_mangle]
pub unsafe extern "C" fn taem_hac_geometry_sweep(
    mut m: *const TerminalModel,
    mut radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut sweep_abs_rad: ::core::ffi::c_double,
    mut out: *mut TaemFixedHacGeometry,
) -> bool {
    if m.is_null() || out.is_null() || !(radius >= 3000.0f64)
        || radius.is_finite() as i32 == 0 || side != -1.0f64 && side != 1.0f64
        || !(sweep_abs_rad > 0.0f64) || sweep_abs_rad.is_finite() as i32 == 0
        || sweep_abs_rad > 1.5f64 * reachability_pi * (1.0f64 + 1e-9f64)
        || !((*m).guidance.final_approach_distance > 0.0f64)
    {
        return false_0 != 0;
    }
    let mut final_distance: ::core::ffi::c_double = (*m)
        .guidance
        .final_approach_distance;
    let mut alignment_speed: ::core::ffi::c_double = fmax(
        (*m).guidance.final_alignment_speed,
        (*m).vehicle.minimum_safe_speed,
    );
    let mut roll_settle: ::core::ffi::c_double = if (*m).attitude.roll_wn > 0.0f64
        && (*m).attitude.roll_zeta > 0.0f64
    {
        4.0f64 / ((*m).attitude.roll_wn * (*m).attitude.roll_zeta)
    } else {
        6.0f64
    };
    let mut bank_limit: ::core::ffi::c_double = radians(
        fmin((*m).vehicle.maximum_bank_angle, 80.0f64),
    );
    let mut unload: ::core::ffi::c_double = if (*m).attitude.max_roll_rate_rad_s > 0.0f64
    {
        bank_limit / (*m).attitude.max_roll_rate_rad_s
    } else {
        4.0f64
    };
    let mut rollout_time: ::core::ffi::c_double = unload + 2.0f64 * roll_settle;
    let mut rollout_distance: ::core::ffi::c_double = alignment_speed
        * fmax(4.0f64, fmin(20.0f64, rollout_time));
    let mut exit: TaemPoint2 = TaemPoint2 {
        x: -(final_distance + rollout_distance),
        y: 0.0f64,
    };
    let mut center: TaemPoint2 = TaemPoint2 {
        x: exit.x,
        y: side * radius,
    };
    let mut end_angle: ::core::ffi::c_double = atan2(
        exit.y - center.y,
        exit.x - center.x,
    );
    let mut sweep_sign: ::core::ffi::c_double = side;
    let mut sweep: ::core::ffi::c_double = sweep_sign * sweep_abs_rad;
    let mut start_angle: ::core::ffi::c_double = end_angle - sweep;
    let mut entry: TaemPoint2 = TaemPoint2 {
        x: center.x + radius * cos(start_angle),
        y: center.y + radius * sin(start_angle),
    };
    let mut tangent_delta: ::core::ffi::c_double = remainder(
        start_angle + sweep_sign * reachability_pi / 2.0f64,
        2.0f64 * reachability_pi,
    );
    let mut capture_course: ::core::ffi::c_double = (*m).site.runway_heading
        + degrees(tangent_delta);
    *out = TaemFixedHacGeometry {
        entry: entry,
        center: center,
        exit: exit,
        radius_m: radius,
        side: side,
        capture_course_deg: capture_course,
        arc_sweep_rad: sweep,
        arc_length_m: fabs(sweep) * radius,
        final_length_m: final_distance + rollout_distance,
        total_length_m: fabs(sweep) * radius + final_distance + rollout_distance,
    };
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_geometry(
    mut m: *const TerminalModel,
    mut radius: ::core::ffi::c_double,
    mut side: ::core::ffi::c_double,
    mut out: *mut TaemFixedHacGeometry,
) -> bool {
    return taem_hac_geometry_sweep(m, radius, side, 1.5f64 * reachability_pi, out);
}
#[no_mangle]
pub unsafe extern "C" fn taem_fixed_hac_turn_reachability(
    mut m: *const TerminalModel,
    mut s: *const TerminalDynamicState,
    mut g: *const TaemGeometryState,
    mut radius: ::core::ffi::c_double,
    mut out: *mut TaemReachability,
) -> bool {
    if m.is_null() || s.is_null() || g.is_null() || out.is_null() || !(radius > 0.0f64)
        || !((*g).ground_speed_mps > 0.0f64) || !((*s).mass_kg > 0.0f64)
    {
        return false_0 != 0;
    }
    memset(
        out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemReachability>() as size_t,
    );
    let mut altitude: ::core::ffi::c_double = v3_norm((*s).position_i_m)
        - (*m).world.radius_m;
    let mut local_radius: ::core::ffi::c_double = (*m).world.radius_m + altitude;
    if !(local_radius > 0.0f64) || (*m).world.mu_m3_s2.is_finite() as i32 == 0
        || !((*s).mass_kg > 0.0f64) || (*m).aero.alpha_count < 2 as size_t
    {
        return false_0 != 0;
    }
    let mut gravity: ::core::ffi::c_double = (*m).world.mu_m3_s2
        / (local_radius * local_radius);
    let mut max_aoa: ::core::ffi::c_double = fmin(
        (*m).vehicle.maximum_angle_of_attack,
        (*m).aero.alpha_deg[(*m).aero.alpha_count.wrapping_sub(1 as size_t) as usize],
    );
    let mut flow: AeroForces = aero_compute(
        &raw const (*m).world,
        &raw const (*m).aero,
        (*s).position_i_m,
        (*s).velocity_i_mps,
        (*s).ut_s,
        (*s).mass_kg,
        0.0f64,
        0.0f64,
    );
    if flow.mach < 1.0f64
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack.is_finite() as i32 != 0
        && (*m).vehicle.terminal_maximum_lift_angle_of_attack > 0.0f64
    {
        max_aoa = fmin(max_aoa, (*m).vehicle.terminal_maximum_lift_angle_of_attack);
    }
    let mut lift_accel: ::core::ffi::c_double = 0.0f64;
    let mut aoa_deg: ::core::ffi::c_double = 0.0f64;
    while aoa_deg <= max_aoa + 1e-9f64 {
        let mut forces: AeroForces = aero_compute(
            &raw const (*m).world,
            &raw const (*m).aero,
            (*s).position_i_m,
            (*s).velocity_i_mps,
            (*s).ut_s,
            (*s).mass_kg,
            radians(aoa_deg),
            0.0f64,
        );
        if forces.lift_n.is_finite() as i32 != 0 {
            lift_accel = fmax(lift_accel, fmax(0.0f64, forces.lift_n) / (*s).mass_kg);
        }
        aoa_deg += 0.5f64;
    }
    let mut normal_accel: ::core::ffi::c_double = fmin(
        lift_accel,
        fmax(0.0f64, (*m).vehicle.maximum_g_load) * gravity,
    );
    let mut bank: ::core::ffi::c_double = fmin((*m).vehicle.maximum_bank_angle, 80.0f64)
        * reachability_pi / 180.0f64;
    let mut lateral: ::core::ffi::c_double = normal_accel * sin(bank);
    let mut required: ::core::ffi::c_double = (*g).ground_speed_mps
        * (*g).ground_speed_mps / radius;
    let mut wn: ::core::ffi::c_double = (*m).attitude.roll_wn;
    let mut zeta: ::core::ffi::c_double = (*m).attitude.roll_zeta;
    let mut settle: ::core::ffi::c_double = if wn > 0.0f64 && zeta > 0.0f64 {
        4.0f64 / (wn * zeta)
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    if lateral.is_finite() as i32 == 0 || required.is_finite() as i32 == 0
        || settle.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    (*out).valid = true_0 != 0;
    (*out).required_lateral_accel_mps2 = required;
    (*out).available_lateral_accel_mps2 = lateral;
    (*out).lateral_margin_mps2 = lateral - required;
    (*out).minimum_turn_radius_m = if lateral > 1e-9f64 {
        (*g).ground_speed_mps * (*g).ground_speed_mps / lateral
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    (*out).roll_settling_time_s = settle;
    (*out).roll_settling_distance_m = (*g).ground_speed_mps * settle;
    (*out).lateral_authority_ok = (*out).lateral_margin_mps2 >= 0.0f64;
    (*out).energy_qualified = false_0 != 0;
    return true_0 != 0;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
