use crate::sys::*;
extern "C" {
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn tanh(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
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
    fn entry_low_q_protective_aoa_floor(
        dynamic_pressure: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
    ) -> ::core::ffi::c_double;
    fn vscale(a: Vector3, s: ::core::ffi::c_double) -> Vector3;
    fn vmag(a: Vector3) -> ::core::ffi::c_double;
    fn vproject_plane(a: Vector3, normal: Vector3) -> Vector3;
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
    fn entry_alpha_schedule_default(
        schedule: *mut EntryAlphaSchedule,
        vehicle: *const VehicleProfile,
        transition_velocity: ::core::ffi::c_double,
    );
    fn entry_alpha_minimum_drag_tracking_aoa(
        schedule: *const EntryAlphaSchedule,
        phase: EntryAlphaPhase,
        relative_velocity: ::core::ffi::c_double,
        dynamic_pressure: ::core::ffi::c_double,
        maximum_dynamic_pressure: ::core::ffi::c_double,
        aero_confidence: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
}
pub type size_t = usize;
pub type EntryPhase = ::core::ffi::c_uint;
pub const ENTRY_PHASE_TRANSITION: EntryPhase = 5;
pub const ENTRY_PHASE_CONSTANT_DRAG: EntryPhase = 4;
pub const ENTRY_PHASE_EQUILIBRIUM_GLIDE: EntryPhase = 3;
pub const ENTRY_PHASE_TEMPERATURE_CONTROL: EntryPhase = 2;
pub const ENTRY_PHASE_PREENTRY: EntryPhase = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryExecProfile {
    pub has_temperature_velocity_gate: bool,
    pub temperature_end_velocity: ::core::ffi::c_double,
    pub equilibrium_intercept_valid: bool,
    pub equilibrium_intercept: bool,
    pub has_constant_drag_velocity_gate: bool,
    pub constant_drag_end_velocity: ::core::ffi::c_double,
    pub has_phase_floor: bool,
    pub phase_floor: EntryPhase,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
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
pub struct EntryDragReferenceConfig {
    pub entry_velocity_ratio: ::core::ffi::c_double,
    pub temperature_end_progress: ::core::ffi::c_double,
    pub equilibrium_end_progress: ::core::ffi::c_double,
    pub constant_end_progress: ::core::ffi::c_double,
    pub temperature_start_drag_ratio: ::core::ffi::c_double,
    pub equilibrium_blend: ::core::ffi::c_double,
    pub equilibrium_anchor_weight: ::core::ffi::c_double,
    pub range_gain: ::core::ffi::c_double,
    pub minimum_range_authority: ::core::ffi::c_double,
    pub minimum_drag_scale: ::core::ffi::c_double,
    pub maximum_drag_scale: ::core::ffi::c_double,
    pub range_error_scale_fraction: ::core::ffi::c_double,
    pub minimum_range_error_scale: ::core::ffi::c_double,
    pub vertical_drag_gain: ::core::ffi::c_double,
    pub vertical_range_gain: ::core::ffi::c_double,
    pub maximum_vertical_lift_correction: ::core::ffi::c_double,
    pub maximum_drag_g_fraction: ::core::ffi::c_double,
    pub local_drag_authority_multiplier: ::core::ffi::c_double,
    pub minimum_local_drag_confidence: ::core::ffi::c_double,
    pub minimum_drag_accel: ::core::ffi::c_double,
    pub prediction_steps: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryDragReferenceInput {
    pub phase: EntryPhase,
    pub relative_velocity: ::core::ffi::c_double,
    pub latitude: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub state: *const VehicleState,
    pub measured_drag_accel: ::core::ffi::c_double,
    pub modeled_drag_accel: ::core::ffi::c_double,
    pub aero_confidence: ::core::ffi::c_double,
    pub has_incidence: bool,
    pub angle_of_attack: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub range_to_site: ::core::ffi::c_double,
    pub taem_range: ::core::ffi::c_double,
    pub taem_latitude: ::core::ffi::c_double,
    pub taem_altitude: ::core::ffi::c_double,
    pub taem_velocity: ::core::ffi::c_double,
    pub planet: *const PlanetModel,
    pub vehicle: *const VehicleProfile,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryDragReferenceOutput {
    pub valid: bool,
    pub active: bool,
    pub degraded: bool,
    pub confidence: ::core::ffi::c_double,
    pub entry_anchor_velocity: ::core::ffi::c_double,
    pub profile_progress: ::core::ffi::c_double,
    pub range_authority: ::core::ffi::c_double,
    pub required_average_drag_accel: ::core::ffi::c_double,
    pub equilibrium_drag_accel: ::core::ffi::c_double,
    pub constant_drag_accel: ::core::ffi::c_double,
    pub taem_drag_accel: ::core::ffi::c_double,
    pub nominal_drag_accel: ::core::ffi::c_double,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub minimum_achievable_drag_accel: ::core::ffi::c_double,
    pub drag_floor_active: bool,
    pub observed_drag_accel: ::core::ffi::c_double,
    pub drag_error_accel: ::core::ffi::c_double,
    pub drag_error_fraction: ::core::ffi::c_double,
    pub available_range_to_taem: ::core::ffi::c_double,
    pub unshaped_predicted_range: ::core::ffi::c_double,
    pub predicted_range_to_taem: ::core::ffi::c_double,
    pub unshaped_range_error: ::core::ffi::c_double,
    pub predicted_range_error: ::core::ffi::c_double,
    pub range_error_scale: ::core::ffi::c_double,
    pub drag_range_scale: ::core::ffi::c_double,
    pub reference_drag_dv: ::core::ffi::c_double,
    pub predicted_range_d_drag: ::core::ffi::c_double,
    pub required_vertical_lift_accel: ::core::ffi::c_double,
}
pub type EntryAlphaPhase = ::core::ffi::c_uint;
pub const ENTRY_ALPHA_PHASE_COUNT: EntryAlphaPhase = 5;
pub const ENTRY_ALPHA_TRANSITION: EntryAlphaPhase = 4;
pub const ENTRY_ALPHA_CONSTANT_DRAG: EntryAlphaPhase = 3;
pub const ENTRY_ALPHA_EQUILIBRIUM_GLIDE: EntryAlphaPhase = 2;
pub const ENTRY_ALPHA_TEMPERATURE_CONTROL: EntryAlphaPhase = 1;
pub const ENTRY_ALPHA_PRE_ENTRY: EntryAlphaPhase = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryAlphaSchedule {
    pub point_count: size_t,
    pub relative_velocity: [::core::ffi::c_double; 8],
    pub alpha: [::core::ffi::c_double; 8],
    pub phase_modulation_scale: [::core::ffi::c_double; 5],
    pub drag_error_gain: ::core::ffi::c_double,
    pub maximum_modulation: ::core::ffi::c_double,
    pub modulation_rate_limit: ::core::ffi::c_double,
    pub target_rate_limit: ::core::ffi::c_double,
    pub minimum_protective_aoa: ::core::ffi::c_double,
    pub minimum_aero_confidence: ::core::ffi::c_double,
    pub dynamic_pressure_protect_ratio: ::core::ffi::c_double,
    pub g_load_protect_ratio: ::core::ffi::c_double,
    pub stall_fraction_limit: ::core::ffi::c_double,
    pub calibrated_stall_speed_margin: ::core::ffi::c_double,
}
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
unsafe extern "C" fn smoothstep01(
    mut x: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    x = clampd(x, 0.0f64, 1.0f64);
    return x * x * (3.0f64 - 2.0f64 * x);
}
unsafe extern "C" fn phase_valid(mut phase: EntryPhase) -> bool {
    return phase as ::core::ffi::c_uint
        >= ENTRY_PHASE_PREENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
        && phase as ::core::ffi::c_uint
            <= ENTRY_PHASE_TRANSITION as ::core::ffi::c_int as ::core::ffi::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn entry_drag_reference_default_config() -> EntryDragReferenceConfig {
    let mut c: EntryDragReferenceConfig = EntryDragReferenceConfig {
        entry_velocity_ratio: 0.,
        temperature_end_progress: 0.,
        equilibrium_end_progress: 0.,
        constant_end_progress: 0.,
        temperature_start_drag_ratio: 0.,
        equilibrium_blend: 0.,
        equilibrium_anchor_weight: 0.,
        range_gain: 0.,
        minimum_range_authority: 0.,
        minimum_drag_scale: 0.,
        maximum_drag_scale: 0.,
        range_error_scale_fraction: 0.,
        minimum_range_error_scale: 0.,
        vertical_drag_gain: 0.,
        vertical_range_gain: 0.,
        maximum_vertical_lift_correction: 0.,
        maximum_drag_g_fraction: 0.,
        local_drag_authority_multiplier: 0.,
        minimum_local_drag_confidence: 0.,
        minimum_drag_accel: 0.,
        prediction_steps: 0,
    };
    memset(
        &raw mut c as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryDragReferenceConfig>() as size_t,
    );
    c.entry_velocity_ratio = 3.40f64;
    c.temperature_end_progress = 0.30f64;
    c.equilibrium_end_progress = 0.58f64;
    c.constant_end_progress = 0.82f64;
    c.temperature_start_drag_ratio = 0.55f64;
    c.equilibrium_blend = 0.34f64;
    c.equilibrium_anchor_weight = 0.30f64;
    c.range_gain = 0.80f64;
    c.minimum_range_authority = 0.40f64;
    c.minimum_drag_scale = 0.55f64;
    c.maximum_drag_scale = 1.75f64;
    c.range_error_scale_fraction = 0.28f64;
    c.minimum_range_error_scale = 10000.0f64;
    c.vertical_drag_gain = 0.28f64;
    c.vertical_range_gain = 0.22f64;
    c.maximum_vertical_lift_correction = 0.72f64;
    c.maximum_drag_g_fraction = 0.72f64;
    c.local_drag_authority_multiplier = 1.85f64;
    c.minimum_local_drag_confidence = 0.20f64;
    c.minimum_drag_accel = 0.04f64;
    c.prediction_steps = 96 as ::core::ffi::c_uint;
    return c;
}
#[no_mangle]
pub unsafe extern "C" fn entry_drag_reference_config_valid(
    mut c: *const EntryDragReferenceConfig,
) -> bool {
    if c.is_null() {
        return false_0 != 0;
    }
    return (*c).entry_velocity_ratio.is_finite() as i32 != 0
        && (*c).entry_velocity_ratio > 1.05f64
        && (*c).temperature_end_progress.is_finite() as i32 != 0
        && (*c).temperature_end_progress
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).equilibrium_end_progress.is_finite() as i32 != 0
        && (*c).equilibrium_end_progress > (*c).temperature_end_progress
        && (*c).constant_end_progress.is_finite() as i32 != 0
        && (*c).constant_end_progress > (*c).equilibrium_end_progress
        && (*c).constant_end_progress < 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).temperature_start_drag_ratio.is_finite() as i32 != 0
        && (*c).temperature_start_drag_ratio
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).temperature_start_drag_ratio <= 1.5f64
        && (*c).equilibrium_blend.is_finite() as i32 != 0
        && (*c).equilibrium_blend >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).equilibrium_blend <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).equilibrium_anchor_weight.is_finite() as i32 != 0
        && (*c).equilibrium_anchor_weight
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).equilibrium_anchor_weight
            <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).range_gain.is_finite() as i32 != 0
        && (*c).range_gain >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).minimum_range_authority.is_finite() as i32 != 0
        && (*c).minimum_range_authority
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).minimum_range_authority
            <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).minimum_drag_scale.is_finite() as i32 != 0
        && (*c).minimum_drag_scale > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).maximum_drag_scale.is_finite() as i32 != 0
        && (*c).maximum_drag_scale >= (*c).minimum_drag_scale
        && (*c).range_error_scale_fraction.is_finite() as i32 != 0
        && (*c).range_error_scale_fraction
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).minimum_range_error_scale.is_finite() as i32 != 0
        && (*c).minimum_range_error_scale
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).vertical_drag_gain.is_finite() as i32 != 0
        && (*c).vertical_drag_gain >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).vertical_range_gain.is_finite() as i32 != 0
        && (*c).vertical_range_gain >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).maximum_vertical_lift_correction.is_finite() as i32 != 0
        && (*c).maximum_vertical_lift_correction
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).maximum_vertical_lift_correction
            < 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).maximum_drag_g_fraction.is_finite() as i32 != 0
        && (*c).maximum_drag_g_fraction
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).maximum_drag_g_fraction
            <= 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).local_drag_authority_multiplier.is_finite() as i32 != 0
        && (*c).local_drag_authority_multiplier >= 1.0f64
        && (*c).local_drag_authority_multiplier <= 5.0f64
        && (*c).minimum_local_drag_confidence.is_finite() as i32 != 0
        && (*c).minimum_local_drag_confidence
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).minimum_local_drag_confidence <= 1.0f64
        && (*c).minimum_drag_accel.is_finite() as i32 != 0
        && (*c).minimum_drag_accel > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*c).prediction_steps >= 16 as ::core::ffi::c_uint
        && (*c).prediction_steps <= 512 as ::core::ffi::c_uint;
}
unsafe extern "C" fn inertial_state_valid(mut state: *const VehicleState) -> bool {
    if state.is_null() {
        return true_0 != 0;
    }
    let mut radius: ::core::ffi::c_double = vmag((*state).position);
    return radius.is_finite() as i32 != 0 && radius > 0.0f64
        && (*state).velocity.x.is_finite() as i32 != 0
        && (*state).velocity.y.is_finite() as i32 != 0
        && (*state).velocity.z.is_finite() as i32 != 0
        && vmag((*state).velocity).is_finite() as i32 != 0;
}
unsafe extern "C" fn input_valid(mut i: *const EntryDragReferenceInput) -> bool {
    return !i.is_null() && phase_valid((*i).phase) as ::core::ffi::c_int != 0
        && !(*i).planet.is_null() && !(*i).vehicle.is_null()
        && inertial_state_valid((*i).state) as ::core::ffi::c_int != 0
        && (*i).relative_velocity.is_finite() as i32 != 0
        && (*i).relative_velocity >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*i).latitude.is_finite() as i32 != 0 && (*i).altitude.is_finite() as i32 != 0
        && (*i).range_to_site.is_finite() as i32 != 0
        && (*i).range_to_site >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*i).taem_range.is_finite() as i32 != 0
        && (*i).taem_range >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*i).taem_latitude.is_finite() as i32 != 0
        && (*i).taem_altitude.is_finite() as i32 != 0
        && (*i).taem_velocity.is_finite() as i32 != 0
        && (*i).taem_velocity > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (!(*i).has_incidence
            || (*i).angle_of_attack.is_finite() as i32 != 0
                && (*i).sideslip.is_finite() as i32 != 0)
        && (*(*i).planet).radius.is_finite() as i32 != 0
        && (*(*i).planet).radius > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*(*i).planet).gravitational_parameter.is_finite() as i32 != 0
        && (*(*i).planet).gravitational_parameter
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*(*i).vehicle).estimated_lift_to_drag.is_finite() as i32 != 0
        && (*(*i).vehicle).estimated_lift_to_drag
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*(*i).vehicle).estimated_ballistic_coefficient.is_finite() as i32 != 0
        && (*(*i).vehicle).estimated_ballistic_coefficient
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*(*i).vehicle).maximum_dynamic_pressure.is_finite() as i32 != 0
        && (*(*i).vehicle).maximum_dynamic_pressure
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*(*i).vehicle).maximum_g_load.is_finite() as i32 != 0
        && (*(*i).vehicle).maximum_g_load
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double;
}
unsafe extern "C" fn local_gravity(
    mut p: *const PlanetModel,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = fmax(1.0f64, (*p).radius + altitude);
    return (*p).gravitational_parameter / (r * r);
}
unsafe extern "C" fn entry_anchor_velocity(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
) -> ::core::ffi::c_double {
    return fmax(
        (*i).taem_velocity + 1.0f64,
        (*i).taem_velocity * (*c).entry_velocity_ratio,
    );
}
unsafe extern "C" fn progress_for_velocity(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut velocity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut entry: ::core::ffi::c_double = entry_anchor_velocity(i, c);
    let mut span: ::core::ffi::c_double = fmax(1.0f64, entry - (*i).taem_velocity);
    return clampd((entry - velocity) / span, 0.0f64, 1.0f64);
}
unsafe extern "C" fn velocity_for_progress(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut progress: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut entry: ::core::ffi::c_double = entry_anchor_velocity(i, c);
    return entry - clampd(progress, 0.0f64, 1.0f64) * (entry - (*i).taem_velocity);
}
unsafe extern "C" fn alpha_phase_for_entry_phase(
    mut phase: EntryPhase,
) -> EntryAlphaPhase {
    match phase as ::core::ffi::c_uint {
        1 => return ENTRY_ALPHA_PRE_ENTRY,
        2 => return ENTRY_ALPHA_TEMPERATURE_CONTROL,
        3 => return ENTRY_ALPHA_EQUILIBRIUM_GLIDE,
        4 => return ENTRY_ALPHA_CONSTANT_DRAG,
        5 => return ENTRY_ALPHA_TRANSITION,
        _ => {}
    }
    return ENTRY_ALPHA_PRE_ENTRY;
}
unsafe extern "C" fn alpha_phase_for_profile_velocity(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut velocity: ::core::ffi::c_double,
) -> EntryAlphaPhase {
    let mut p: ::core::ffi::c_double = progress_for_velocity(i, c, velocity);
    let mut projected: EntryAlphaPhase = ENTRY_ALPHA_TEMPERATURE_CONTROL;
    if p > (*c).constant_end_progress {
        projected = ENTRY_ALPHA_TRANSITION;
    } else if p > (*c).equilibrium_end_progress {
        projected = ENTRY_ALPHA_CONSTANT_DRAG;
    } else if p > (*c).temperature_end_progress {
        projected = ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
    }
    let mut current: EntryAlphaPhase = alpha_phase_for_entry_phase((*i).phase);
    return (if projected as ::core::ffi::c_uint > current as ::core::ffi::c_uint {
        projected as ::core::ffi::c_uint
    } else {
        current as ::core::ffi::c_uint
    }) as EntryAlphaPhase;
}
unsafe extern "C" fn maximum_reference_drag(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
) -> ::core::ffi::c_double {
    let mut g: ::core::ffi::c_double = local_gravity((*i).planet, (*i).taem_altitude);
    let mut by_g: ::core::ffi::c_double = (*c).maximum_drag_g_fraction
        * (*(*i).vehicle).maximum_g_load * g;
    let mut by_q: ::core::ffi::c_double = (*(*i).vehicle).maximum_dynamic_pressure
        / (*(*i).vehicle).estimated_ballistic_coefficient;
    let mut cap: ::core::ffi::c_double = by_g;
    if by_q.is_finite() as i32 != 0
        && by_q > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        cap = fmin(cap, by_q);
    }
    return fmax((*c).minimum_drag_accel, cap);
}
unsafe extern "C" fn achievable_profile_drag_cap(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut velocity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut structural: ::core::ffi::c_double = maximum_reference_drag(i, c);
    let mut confidence: ::core::ffi::c_double = clampd(
        if (*i).aero_confidence.is_finite() as i32 != 0 {
            (*i).aero_confidence
        } else {
            0.0f64
        },
        0.0f64,
        1.0f64,
    );
    let mut measured: bool = (*i).measured_drag_accel.is_finite() as i32 != 0
        && (*i).measured_drag_accel > 0.0f64;
    let mut modeled: bool = (*i).modeled_drag_accel.is_finite() as i32 != 0
        && (*i).modeled_drag_accel > 0.0f64;
    if !measured && (!modeled || confidence < (*c).minimum_local_drag_confidence) {
        return structural;
    }
    let mut achieved: ::core::ffi::c_double = if measured as ::core::ffi::c_int != 0 {
        (*i).measured_drag_accel
    } else {
        (*i).modeled_drag_accel
    };
    let mut beta: ::core::ffi::c_double = fmax(
        1.0f64,
        (*(*i).vehicle).estimated_ballistic_coefficient,
    );
    let mut rho_now: ::core::ffi::c_double = fmax(
        0.0f64,
        planet_atmospheric_density((*i).planet, (*i).altitude),
    );
    let mut ballistic_now: ::core::ffi::c_double = 0.5f64 * rho_now
        * (*i).relative_velocity * (*i).relative_velocity / beta;
    if ballistic_now.is_finite() as i32 == 0 || !(ballistic_now > 0.0f64) {
        return structural;
    }
    let mut calibration: ::core::ffi::c_double = clampd(
        achieved / ballistic_now,
        0.35f64,
        3.0f64,
    );
    let mut span: ::core::ffi::c_double = fmax(
        1.0f64,
        (*i).relative_velocity - (*i).taem_velocity,
    );
    let mut remaining: ::core::ffi::c_double = clampd(
        (velocity - (*i).taem_velocity) / span,
        0.0f64,
        1.0f64,
    );
    let mut altitude: ::core::ffi::c_double = (*i).taem_altitude
        + ((*i).altitude - (*i).taem_altitude) * smoothstep01(remaining);
    let mut rho: ::core::ffi::c_double = fmax(
        0.0f64,
        planet_atmospheric_density((*i).planet, altitude),
    );
    let mut projected: ::core::ffi::c_double = 0.5f64 * rho * velocity * velocity / beta
        * calibration * (*c).local_drag_authority_multiplier;
    if !(projected > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || projected.is_finite() as i32 == 0
    {
        return structural;
    }
    return clampd(projected, (*c).minimum_drag_accel, structural);
}
unsafe extern "C" fn achievable_profile_drag_floor(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut velocity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(*i).has_incidence {
        return (*c).minimum_drag_accel;
    }
    let mut confidence: ::core::ffi::c_double = clampd(
        if (*i).aero_confidence.is_finite() as i32 != 0 {
            (*i).aero_confidence
        } else {
            0.0f64
        },
        0.0f64,
        1.0f64,
    );
    let mut measured: bool = (*i).measured_drag_accel.is_finite() as i32 != 0
        && (*i).measured_drag_accel > 0.0f64;
    let mut modeled: bool = (*i).modeled_drag_accel.is_finite() as i32 != 0
        && (*i).modeled_drag_accel > 0.0f64;
    if !measured && (!modeled || confidence < (*c).minimum_local_drag_confidence) {
        return (*c).minimum_drag_accel;
    }
    let mut achieved: ::core::ffi::c_double = if measured as ::core::ffi::c_int != 0 {
        (*i).measured_drag_accel
    } else {
        (*i).modeled_drag_accel
    };
    let mut rho_now: ::core::ffi::c_double = fmax(
        0.0f64,
        planet_atmospheric_density((*i).planet, (*i).altitude),
    );
    let mut q_now: ::core::ffi::c_double = 0.5f64 * rho_now * (*i).relative_velocity
        * (*i).relative_velocity;
    if q_now.is_finite() as i32 == 0 || !(q_now > 0.0f64) {
        return (*c).minimum_drag_accel;
    }
    let mut sound_now: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        (*i).planet,
        (*i).altitude,
    );
    let mut mach_now: ::core::ffi::c_double = if sound_now > 1.0f64 {
        (*i).relative_velocity / sound_now
    } else {
        5.0f64
    };
    let mut current_incidence: ::core::ffi::c_double = hypot(
        (*i).angle_of_attack,
        (*i).sideslip,
    );
    let mut current_df: ::core::ffi::c_double = 1.0f64;
    let mut floor_df: ::core::ffi::c_double = 1.0f64;
    aerodynamic_force_factors_mach(
        mach_now,
        current_incidence,
        (*i).vehicle,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut current_df,
    );
    let mut alpha_schedule: EntryAlphaSchedule = EntryAlphaSchedule {
        point_count: 0,
        relative_velocity: [0.; 8],
        alpha: [0.; 8],
        phase_modulation_scale: [0.; 5],
        drag_error_gain: 0.,
        maximum_modulation: 0.,
        modulation_rate_limit: 0.,
        target_rate_limit: 0.,
        minimum_protective_aoa: 0.,
        minimum_aero_confidence: 0.,
        dynamic_pressure_protect_ratio: 0.,
        g_load_protect_ratio: 0.,
        stall_fraction_limit: 0.,
        calibrated_stall_speed_margin: 0.,
    };
    entry_alpha_schedule_default(
        &raw mut alpha_schedule,
        (*i).vehicle,
        (*i).taem_velocity,
    );
    let mut floor_phase: EntryAlphaPhase = alpha_phase_for_profile_velocity(
        i,
        c,
        (*i).relative_velocity,
    );
    let mut floor_aoa: ::core::ffi::c_double = entry_alpha_minimum_drag_tracking_aoa(
        &raw mut alpha_schedule,
        floor_phase,
        (*i).relative_velocity,
        q_now,
        (*(*i).vehicle).maximum_dynamic_pressure,
        confidence,
    );
    if floor_aoa.is_finite() as i32 == 0 {
        floor_aoa = entry_low_q_protective_aoa_floor(q_now, (*i).vehicle);
    }
    aerodynamic_force_factors_mach(
        mach_now,
        floor_aoa,
        (*i).vehicle,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut floor_df,
    );
    let mut floor_now: ::core::ffi::c_double = achieved * fmax(0.02f64, floor_df)
        / fmax(0.05f64, current_df);
    let mut span: ::core::ffi::c_double = fmax(
        1.0f64,
        (*i).relative_velocity - (*i).taem_velocity,
    );
    let mut remaining: ::core::ffi::c_double = clampd(
        (velocity - (*i).taem_velocity) / span,
        0.0f64,
        1.0f64,
    );
    let mut altitude: ::core::ffi::c_double = (*i).taem_altitude
        + ((*i).altitude - (*i).taem_altitude) * smoothstep01(remaining);
    let mut rho: ::core::ffi::c_double = fmax(
        0.0f64,
        planet_atmospheric_density((*i).planet, altitude),
    );
    let mut q: ::core::ffi::c_double = 0.5f64 * rho * velocity * velocity;
    if !(q > 0.0f64) || q.is_finite() as i32 == 0 {
        return (*c).minimum_drag_accel;
    }
    let mut sound: ::core::ffi::c_double = planet_atmospheric_speed_of_sound(
        (*i).planet,
        altitude,
    );
    let mut mach: ::core::ffi::c_double = if sound > 1.0f64 {
        velocity / sound
    } else {
        mach_now
    };
    let mut projected_phase: EntryAlphaPhase = alpha_phase_for_profile_velocity(
        i,
        c,
        velocity,
    );
    let mut projected_aoa: ::core::ffi::c_double = entry_alpha_minimum_drag_tracking_aoa(
        &raw mut alpha_schedule,
        projected_phase,
        velocity,
        q,
        (*(*i).vehicle).maximum_dynamic_pressure,
        confidence,
    );
    if projected_aoa.is_finite() as i32 == 0 {
        projected_aoa = entry_low_q_protective_aoa_floor(q, (*i).vehicle);
    }
    let mut projected_df: ::core::ffi::c_double = 1.0f64;
    aerodynamic_force_factors_mach(
        mach,
        projected_aoa,
        (*i).vehicle,
        ::core::ptr::null_mut::<::core::ffi::c_double>(),
        &raw mut projected_df,
    );
    let mut projected: ::core::ffi::c_double = floor_now * (q / q_now)
        * fmax(0.02f64, projected_df) / fmax(0.02f64, floor_df);
    return clampd(projected, (*c).minimum_drag_accel, maximum_reference_drag(i, c));
}
unsafe extern "C" fn equilibrium_vertical_lift(
    mut i: *const EntryDragReferenceInput,
    mut velocity: ::core::ffi::c_double,
    mut altitude: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut r: ::core::ffi::c_double = fmax(1.0f64, (*(*i).planet).radius + altitude);
    let mut g: ::core::ffi::c_double = local_gravity((*i).planet, altitude);
    let mut required: ::core::ffi::c_double = g - velocity * velocity / r;
    return clampd(required, 0.12f64 * g, 0.995f64 * g);
}
unsafe extern "C" fn equilibrium_drag_at(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut velocity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut lift: ::core::ffi::c_double = equilibrium_vertical_lift(
        i,
        velocity,
        (*i).taem_altitude,
    );
    let mut drag: ::core::ffi::c_double = lift
        / fmax(0.05f64, (*(*i).vehicle).estimated_lift_to_drag);
    return clampd(drag, (*c).minimum_drag_accel, maximum_reference_drag(i, c));
}
unsafe extern "C" fn taem_drag_at_target(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut density: ::core::ffi::c_double = planet_atmospheric_density(
        (*i).planet,
        (*i).taem_altitude,
    );
    if !(density > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        && !((*i).taem_altitude >= (*(*i).planet).atmosphere_depth)
    {
        return fallback;
    }
    let mut q: ::core::ffi::c_double = 0.5f64 * fmax(0.0f64, density)
        * (*i).taem_velocity * (*i).taem_velocity;
    let mut drag: ::core::ffi::c_double = q
        / fmax(1.0f64, (*(*i).vehicle).estimated_ballistic_coefficient);
    if !(drag > 0 as ::core::ffi::c_int as ::core::ffi::c_double)
        || drag.is_finite() as i32 == 0
    {
        return fallback;
    }
    return clampd(drag, (*c).minimum_drag_accel, maximum_reference_drag(i, c));
}
unsafe extern "C" fn profile_drag_at(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut constant_drag: ::core::ffi::c_double,
    mut taem_drag: ::core::ffi::c_double,
    mut velocity: ::core::ffi::c_double,
    mut scale: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut p: ::core::ffi::c_double = progress_for_velocity(i, c, velocity);
    let mut drag: ::core::ffi::c_double = constant_drag;
    if p <= (*c).temperature_end_progress {
        let mut x: ::core::ffi::c_double = p
            / fmax((*c).temperature_end_progress, 1e-6f64);
        let mut ratio: ::core::ffi::c_double = (*c).temperature_start_drag_ratio
            + (1.0f64 - (*c).temperature_start_drag_ratio) * smoothstep01(x);
        drag = constant_drag * ratio;
    } else if p <= (*c).equilibrium_end_progress {
        let mut x_0: ::core::ffi::c_double = (p - (*c).temperature_end_progress)
            / fmax(
                (*c).equilibrium_end_progress - (*c).temperature_end_progress,
                1e-6f64,
            );
        let mut window: ::core::ffi::c_double = sin(LANDER_PI * x_0);
        window *= window;
        let mut eq: ::core::ffi::c_double = equilibrium_drag_at(i, c, velocity);
        drag = constant_drag + (eq - constant_drag) * (*c).equilibrium_blend * window;
    } else if p <= (*c).constant_end_progress {
        drag = constant_drag;
    } else {
        let mut x_1: ::core::ffi::c_double = (p - (*c).constant_end_progress)
            / fmax(1.0f64 - (*c).constant_end_progress, 1e-6f64);
        drag = constant_drag + (taem_drag - constant_drag) * smoothstep01(x_1);
    }
    return clampd(
        drag * scale,
        (*c).minimum_drag_accel,
        achievable_profile_drag_cap(i, c, velocity),
    );
}
unsafe extern "C" fn feasible_profile_drag_at(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut constant_drag: ::core::ffi::c_double,
    mut taem_drag: ::core::ffi::c_double,
    mut velocity: ::core::ffi::c_double,
    mut scale: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut desired: ::core::ffi::c_double = profile_drag_at(
        i,
        c,
        constant_drag,
        taem_drag,
        velocity,
        scale,
    );
    let mut lower: ::core::ffi::c_double = achievable_profile_drag_floor(i, c, velocity);
    let mut upper: ::core::ffi::c_double = achievable_profile_drag_cap(i, c, velocity);
    if lower > upper {
        lower = upper;
    }
    return clampd(desired, lower, upper);
}
unsafe extern "C" fn predicted_range(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut constant_drag: ::core::ffi::c_double,
    mut taem_drag: ::core::ffi::c_double,
    mut scale: ::core::ffi::c_double,
    mut remaining_energy: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if remaining_energy <= 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        return 0.0f64;
    }
    let mut signed_kinetic: ::core::ffi::c_double = 0.5f64
        * ((*i).relative_velocity * (*i).relative_velocity
            - (*i).taem_velocity * (*i).taem_velocity);
    let mut range: ::core::ffi::c_double = 0.0f64;
    if (*i).relative_velocity > (*i).taem_velocity {
        let mut steps: ::core::ffi::c_uint = (*c).prediction_steps;
        let mut dv: ::core::ffi::c_double = ((*i).relative_velocity - (*i).taem_velocity)
            / steps as ::core::ffi::c_double;
        let mut n: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
        while n < steps {
            let mut velocity: ::core::ffi::c_double = (*i).taem_velocity
                + (n as ::core::ffi::c_double + 0.5f64) * dv;
            let mut drag: ::core::ffi::c_double = feasible_profile_drag_at(
                i,
                c,
                constant_drag,
                taem_drag,
                velocity,
                scale,
            );
            range += velocity / fmax((*c).minimum_drag_accel, drag) * dv;
            n = n.wrapping_add(1);
        }
    }
    let mut nonkinetic: ::core::ffi::c_double = remaining_energy
        - fmax(0.0f64, signed_kinetic);
    if nonkinetic != 0.0f64 {
        let mut current_drag: ::core::ffi::c_double = feasible_profile_drag_at(
            i,
            c,
            constant_drag,
            taem_drag,
            fmax((*i).relative_velocity, (*i).taem_velocity),
            scale,
        );
        let mut target_drag: ::core::ffi::c_double = feasible_profile_drag_at(
            i,
            c,
            constant_drag,
            taem_drag,
            (*i).taem_velocity,
            scale,
        );
        let mut representative: ::core::ffi::c_double = sqrt(
            fmax((*c).minimum_drag_accel, current_drag)
                * fmax((*c).minimum_drag_accel, target_drag),
        );
        range += nonkinetic / fmax((*c).minimum_drag_accel, representative);
    }
    return fmax(0.0f64, range);
}
unsafe extern "C" fn geometric_blend(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
    mut b_weight: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    a = fmax(a, 1e-9f64);
    b = fmax(b, 1e-9f64);
    b_weight = clampd(b_weight, 0.0f64, 1.0f64);
    return exp((1.0f64 - b_weight) * log(a) + b_weight * log(b));
}
#[no_mangle]
pub unsafe extern "C" fn entry_drag_reference_compute(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
) -> EntryDragReferenceOutput {
    let mut out: EntryDragReferenceOutput = EntryDragReferenceOutput {
        valid: false,
        active: false,
        degraded: false,
        confidence: 0.,
        entry_anchor_velocity: 0.,
        profile_progress: 0.,
        range_authority: 0.,
        required_average_drag_accel: 0.,
        equilibrium_drag_accel: 0.,
        constant_drag_accel: 0.,
        taem_drag_accel: 0.,
        nominal_drag_accel: 0.,
        reference_drag_accel: 0.,
        minimum_achievable_drag_accel: 0.,
        drag_floor_active: false,
        observed_drag_accel: 0.,
        drag_error_accel: 0.,
        drag_error_fraction: 0.,
        available_range_to_taem: 0.,
        unshaped_predicted_range: 0.,
        predicted_range_to_taem: 0.,
        unshaped_range_error: 0.,
        predicted_range_error: 0.,
        range_error_scale: 0.,
        drag_range_scale: 0.,
        reference_drag_dv: 0.,
        predicted_range_d_drag: 0.,
        required_vertical_lift_accel: 0.,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryDragReferenceOutput>() as size_t,
    );
    if !entry_drag_reference_config_valid(c) || !input_valid(i) {
        return out;
    }
    out.valid = true_0 != 0;
    out.active = (*i).phase as ::core::ffi::c_uint
        != ENTRY_PHASE_PREENTRY as ::core::ffi::c_int as ::core::ffi::c_uint;
    out.entry_anchor_velocity = entry_anchor_velocity(i, c);
    out.profile_progress = progress_for_velocity(i, c, (*i).relative_velocity);
    out.range_authority = (*c).minimum_range_authority
        + (1.0f64 - (*c).minimum_range_authority)
            * smoothstep01(
                out.profile_progress / fmax((*c).constant_end_progress, 1e-6f64),
            );
    let mut remaining_energy: ::core::ffi::c_double = entry_remaining_specific_energy(
        (*i).latitude,
        (*i).altitude,
        (*i).relative_velocity,
        (*i).taem_latitude,
        (*i).taem_altitude,
        (*i).taem_velocity,
        (*i).planet,
    );
    if remaining_energy.is_finite() as i32 == 0 {
        memset(
            &raw mut out as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<EntryDragReferenceOutput>() as size_t,
        );
        return out;
    }
    out.available_range_to_taem = fmax(0.0f64, (*i).range_to_site - (*i).taem_range);
    let mut sizing_range: ::core::ffi::c_double = fmax(
        1000.0f64,
        out.available_range_to_taem,
    );
    out.required_average_drag_accel = clampd(
        fmax(0.0f64, remaining_energy) / sizing_range,
        (*c).minimum_drag_accel,
        maximum_reference_drag(i, c),
    );
    let mut equilibrium_end_velocity: ::core::ffi::c_double = velocity_for_progress(
        i,
        c,
        (*c).equilibrium_end_progress,
    );
    let mut equilibrium_anchor: ::core::ffi::c_double = equilibrium_drag_at(
        i,
        c,
        equilibrium_end_velocity,
    );
    out.constant_drag_accel = geometric_blend(
        out.required_average_drag_accel,
        equilibrium_anchor,
        (*c).equilibrium_anchor_weight,
    );
    out.constant_drag_accel = clampd(
        out.constant_drag_accel,
        (*c).minimum_drag_accel,
        maximum_reference_drag(i, c),
    );
    out.equilibrium_drag_accel = equilibrium_drag_at(i, c, (*i).relative_velocity);
    out.taem_drag_accel = taem_drag_at_target(i, c, out.constant_drag_accel);
    out.minimum_achievable_drag_accel = achievable_profile_drag_floor(
        i,
        c,
        (*i).relative_velocity,
    );
    out.nominal_drag_accel = profile_drag_at(
        i,
        c,
        out.constant_drag_accel,
        out.taem_drag_accel,
        (*i).relative_velocity,
        1.0f64,
    );
    out.range_error_scale = fmax(
        (*c).minimum_range_error_scale,
        fmax(1000.0f64, out.available_range_to_taem) * (*c).range_error_scale_fraction,
    );
    out.unshaped_predicted_range = predicted_range(
        i,
        c,
        out.constant_drag_accel,
        out.taem_drag_accel,
        1.0f64,
        fmax(0.0f64, remaining_energy),
    );
    out.unshaped_range_error = out.unshaped_predicted_range
        - out.available_range_to_taem;
    let mut normalized: ::core::ffi::c_double = out.unshaped_range_error
        / out.range_error_scale;
    let mut exponent: ::core::ffi::c_double = (*c).range_gain * out.range_authority
        * tanh(normalized);
    out.drag_range_scale = clampd(
        exp(exponent),
        (*c).minimum_drag_scale,
        (*c).maximum_drag_scale,
    );
    out.reference_drag_accel = profile_drag_at(
        i,
        c,
        out.constant_drag_accel,
        out.taem_drag_accel,
        (*i).relative_velocity,
        out.drag_range_scale,
    );
    out.drag_floor_active = out.minimum_achievable_drag_accel
        > (*c).minimum_drag_accel + 1e-6f64
        && out.reference_drag_accel < out.minimum_achievable_drag_accel - 1e-6f64;
    out.reference_drag_accel = feasible_profile_drag_at(
        i,
        c,
        out.constant_drag_accel,
        out.taem_drag_accel,
        (*i).relative_velocity,
        out.drag_range_scale,
    );
    out.predicted_range_to_taem = predicted_range(
        i,
        c,
        out.constant_drag_accel,
        out.taem_drag_accel,
        out.drag_range_scale,
        fmax(0.0f64, remaining_energy),
    );
    out.predicted_range_error = out.predicted_range_to_taem
        - out.available_range_to_taem;
    let mut measured: bool = (*i).measured_drag_accel.is_finite() as i32 != 0
        && (*i).measured_drag_accel >= 0.0f64;
    let mut modeled: bool = (*i).modeled_drag_accel.is_finite() as i32 != 0
        && (*i).modeled_drag_accel >= 0.0f64;
    let mut aero: ::core::ffi::c_double = clampd(
        if (*i).aero_confidence.is_finite() as i32 != 0 {
            (*i).aero_confidence
        } else {
            0.0f64
        },
        0.0f64,
        1.0f64,
    );
    if measured {
        out.observed_drag_accel = (*i).measured_drag_accel;
        out.confidence = 0.50f64 + 0.50f64 * aero;
    } else if modeled {
        out.observed_drag_accel = (*i).modeled_drag_accel;
        out.confidence = 0.30f64 + 0.45f64 * aero;
        out.degraded = true_0 != 0;
    } else {
        out.observed_drag_accel = out.reference_drag_accel;
        out.confidence = 0.12f64 + 0.18f64 * aero;
        out.degraded = true_0 != 0;
    }
    if aero < 0.20f64 {
        out.degraded = true_0 != 0;
    }
    out.drag_error_accel = out.reference_drag_accel - out.observed_drag_accel;
    out.drag_error_fraction = out.drag_error_accel
        / fmax(0.25f64, fabs(out.reference_drag_accel));
    let mut delta: ::core::ffi::c_double = fmax(
        0.5f64,
        (*i).relative_velocity * 0.001f64,
    );
    let mut lo: ::core::ffi::c_double = fmax(
        (*i).taem_velocity,
        (*i).relative_velocity - delta,
    );
    let mut hi: ::core::ffi::c_double = (*i).relative_velocity + delta;
    if hi > lo {
        let mut dlo: ::core::ffi::c_double = feasible_profile_drag_at(
            i,
            c,
            out.constant_drag_accel,
            out.taem_drag_accel,
            lo,
            out.drag_range_scale,
        );
        let mut dhi: ::core::ffi::c_double = feasible_profile_drag_at(
            i,
            c,
            out.constant_drag_accel,
            out.taem_drag_accel,
            hi,
            out.drag_range_scale,
        );
        out.reference_drag_dv = (dhi - dlo) / (hi - lo);
    }
    out.predicted_range_d_drag = -out.predicted_range_to_taem
        / fmax((*c).minimum_drag_accel, out.reference_drag_accel);
    let mut support_speed: ::core::ffi::c_double = (*i).relative_velocity;
    let mut support_altitude: ::core::ffi::c_double = (*i).altitude;
    if !(*i).state.is_null() {
        let mut radius: ::core::ffi::c_double = vmag((*(*i).state).position);
        let mut up: Vector3 = vscale((*(*i).state).position, 1.0f64 / radius);
        support_speed = vmag(vproject_plane((*(*i).state).velocity, up));
        support_altitude = radius - (*(*i).planet).radius;
    }
    let mut vertical: ::core::ffi::c_double = equilibrium_vertical_lift(
        i,
        support_speed,
        support_altitude,
    );
    let mut range_fraction: ::core::ffi::c_double = out.predicted_range_error
        / out.range_error_scale;
    let mut correction: ::core::ffi::c_double = (*c).vertical_drag_gain
        * clampd(out.drag_error_fraction, -1.5f64, 1.5f64)
        + (*c).vertical_range_gain * clampd(range_fraction, -1.5f64, 1.5f64);
    correction = clampd(
        correction,
        -(*c).maximum_vertical_lift_correction,
        (*c).maximum_vertical_lift_correction,
    );
    out.required_vertical_lift_accel = vertical
        * clampd(1.0f64 - correction, 0.10f64, 1.50f64);
    if remaining_energy <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*i).relative_velocity <= (*i).taem_velocity
    {
        out.degraded = true_0 != 0;
    }
    out.confidence = clampd(out.confidence, 0.0f64, 1.0f64);
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn entry_drag_reference_exec_profile(
    mut i: *const EntryDragReferenceInput,
    mut c: *const EntryDragReferenceConfig,
    mut r: *const EntryDragReferenceOutput,
) -> EntryExecProfile {
    let mut p: EntryExecProfile = EntryExecProfile {
        has_temperature_velocity_gate: false,
        temperature_end_velocity: 0.,
        equilibrium_intercept_valid: false,
        equilibrium_intercept: false,
        has_constant_drag_velocity_gate: false,
        constant_drag_end_velocity: 0.,
        has_phase_floor: false,
        phase_floor: 0 as EntryPhase,
    };
    memset(
        &raw mut p as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryExecProfile>() as size_t,
    );
    if i.is_null() || c.is_null() || r.is_null() || !(*r).valid
        || !entry_drag_reference_config_valid(c) || !input_valid(i)
    {
        return p;
    }
    let mut temp_end: ::core::ffi::c_double = velocity_for_progress(
        i,
        c,
        (*c).temperature_end_progress,
    );
    let mut eq_end: ::core::ffi::c_double = velocity_for_progress(
        i,
        c,
        (*c).equilibrium_end_progress,
    );
    let mut const_end: ::core::ffi::c_double = velocity_for_progress(
        i,
        c,
        (*c).constant_end_progress,
    );
    p.has_temperature_velocity_gate = true_0 != 0;
    p.temperature_end_velocity = temp_end;
    p.equilibrium_intercept_valid = true_0 != 0;
    p.equilibrium_intercept = (*i).relative_velocity <= eq_end;
    p.has_constant_drag_velocity_gate = true_0 != 0;
    p.constant_drag_end_velocity = const_end;
    p.has_phase_floor = true_0 != 0;
    if (*i).relative_velocity <= const_end {
        p.phase_floor = ENTRY_PHASE_TRANSITION;
    } else if (*i).relative_velocity <= eq_end {
        p.phase_floor = ENTRY_PHASE_CONSTANT_DRAG;
    } else if (*i).relative_velocity <= temp_end {
        p.phase_floor = ENTRY_PHASE_EQUILIBRIUM_GLIDE;
    } else {
        p.phase_floor = ENTRY_PHASE_TEMPERATURE_CONTROL;
    }
    return p;
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
