use crate::sys::*;
extern "C" {
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
}
pub type size_t = usize;
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
pub type EntryAlphaPhase = ::core::ffi::c_uint;
pub const ENTRY_ALPHA_PHASE_COUNT: EntryAlphaPhase = 5;
pub const ENTRY_ALPHA_TRANSITION: EntryAlphaPhase = 4;
pub const ENTRY_ALPHA_CONSTANT_DRAG: EntryAlphaPhase = 3;
pub const ENTRY_ALPHA_EQUILIBRIUM_GLIDE: EntryAlphaPhase = 2;
pub const ENTRY_ALPHA_TEMPERATURE_CONTROL: EntryAlphaPhase = 1;
pub const ENTRY_ALPHA_PRE_ENTRY: EntryAlphaPhase = 0;
pub type EntryAlphaLimitReason = ::core::ffi::c_uint;
pub const ENTRY_ALPHA_LIMIT_INVALID_INPUT: EntryAlphaLimitReason = 8;
pub const ENTRY_ALPHA_LIMIT_COMMAND_RATE: EntryAlphaLimitReason = 7;
pub const ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE: EntryAlphaLimitReason = 6;
pub const ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION: EntryAlphaLimitReason = 5;
pub const ENTRY_ALPHA_LIMIT_G_LOAD: EntryAlphaLimitReason = 4;
pub const ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE: EntryAlphaLimitReason = 3;
pub const ENTRY_ALPHA_LIMIT_STALL_MARGIN: EntryAlphaLimitReason = 2;
pub const ENTRY_ALPHA_LIMIT_MAX_AOA: EntryAlphaLimitReason = 1;
pub const ENTRY_ALPHA_LIMIT_NONE: EntryAlphaLimitReason = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryAlphaInput {
    pub relative_velocity: ::core::ffi::c_double,
    pub phase: EntryAlphaPhase,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub measured_drag_accel: ::core::ffi::c_double,
    pub current_aoa: ::core::ffi::c_double,
    pub current_aoa_rate: ::core::ffi::c_double,
    pub has_previous_target: bool,
    pub previous_target_aoa: ::core::ffi::c_double,
    pub has_previous_modulation: bool,
    pub previous_modulation: ::core::ffi::c_double,
    pub dt: ::core::ffi::c_double,
    pub maximum_aoa: ::core::ffi::c_double,
    pub stall_margin_aoa_limit: ::core::ffi::c_double,
    pub stall_fraction: ::core::ffi::c_double,
    pub stall_fraction_is_measured: bool,
    pub calibrated_stall_speed: ::core::ffi::c_double,
    pub minimum_safe_speed: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub maximum_dynamic_pressure: ::core::ffi::c_double,
    pub g_load: ::core::ffi::c_double,
    pub maximum_g_load: ::core::ffi::c_double,
    pub aero_confidence: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryAlphaResult {
    pub valid: bool,
    pub degraded: bool,
    pub nominal_aoa: ::core::ffi::c_double,
    pub requested_modulation: ::core::ffi::c_double,
    pub modulation: ::core::ffi::c_double,
    pub unconstrained_target_aoa: ::core::ffi::c_double,
    pub target_aoa: ::core::ffi::c_double,
    pub limiting_reason: EntryAlphaLimitReason,
}
pub const ENTRY_ALPHA_MAX_SCHEDULE_POINTS: ::core::ffi::c_int = 8 as ::core::ffi::c_int;
unsafe extern "C" fn smoothstep01(
    mut x: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    x = clampd(x, 0.0f64, 1.0f64);
    return x * x * (3.0f64 - 2.0f64 * x);
}
unsafe extern "C" fn reason_priority(
    mut reason: EntryAlphaLimitReason,
) -> ::core::ffi::c_int {
    match reason as ::core::ffi::c_uint {
        8 => return 100 as ::core::ffi::c_int,
        2 => return 90 as ::core::ffi::c_int,
        4 => return 80 as ::core::ffi::c_int,
        5 => return 75 as ::core::ffi::c_int,
        1 => return 70 as ::core::ffi::c_int,
        3 => return 60 as ::core::ffi::c_int,
        7 => return 50 as ::core::ffi::c_int,
        6 => return 40 as ::core::ffi::c_int,
        0 => return 0 as ::core::ffi::c_int,
        _ => {}
    }
    return 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn set_reason(
    mut result: *mut EntryAlphaResult,
    mut reason: EntryAlphaLimitReason,
) {
    if !result.is_null()
        && reason_priority(reason) > reason_priority((*result).limiting_reason)
    {
        (*result).limiting_reason = reason;
    }
}
unsafe extern "C" fn negative_drag_modulation_scale(
    mut phase: EntryAlphaPhase,
) -> ::core::ffi::c_double {
    match phase as ::core::ffi::c_uint {
        0 => return 1.0f64,
        1 => return 0.85f64,
        2 => return 0.50f64,
        3 => return 0.20f64,
        4 => return 0.35f64,
        5 => return 1.0f64,
        _ => {}
    }
    return 1.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn entry_alpha_schedule_default(
    mut schedule: *mut EntryAlphaSchedule,
    mut vehicle: *const VehicleProfile,
    mut transition_velocity: ::core::ffi::c_double,
) {
    if schedule.is_null() {
        return;
    }
    memset(
        schedule as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryAlphaSchedule>() as size_t,
    );
    if vehicle.is_null() || transition_velocity.is_finite() as i32 == 0
        || transition_velocity <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return;
    }
    let trim: ::core::ffi::c_double = clampd(
        (*vehicle).entry_angle_of_attack,
        0.0f64,
        (*vehicle).maximum_angle_of_attack,
    ) as ::core::ffi::c_double;
    let v: ::core::ffi::c_double = fmax(
        transition_velocity,
        (*vehicle).minimum_safe_speed * 3.0f64,
    ) as ::core::ffi::c_double;
    (*schedule).point_count = 5 as size_t;
    (*schedule).relative_velocity[0 as ::core::ffi::c_int as usize] = v * 0.90f64;
    (*schedule).relative_velocity[1 as ::core::ffi::c_int as usize] = v;
    (*schedule).relative_velocity[2 as ::core::ffi::c_int as usize] = v * 1.25f64;
    (*schedule).relative_velocity[3 as ::core::ffi::c_int as usize] = v * 1.55f64;
    (*schedule).relative_velocity[4 as ::core::ffi::c_int as usize] = v * 1.85f64;
    let mut i: size_t = 0 as size_t;
    while i < 5 as size_t {
        (*schedule).alpha[i as usize] = trim;
        i = i.wrapping_add(1);
    }
    (*schedule)
        .phase_modulation_scale[ENTRY_ALPHA_PRE_ENTRY as ::core::ffi::c_int as usize] = 0.0f64;
    (*schedule)
        .phase_modulation_scale[ENTRY_ALPHA_TEMPERATURE_CONTROL as ::core::ffi::c_int
        as usize] = 0.35f64;
    (*schedule)
        .phase_modulation_scale[ENTRY_ALPHA_EQUILIBRIUM_GLIDE as ::core::ffi::c_int
        as usize] = 0.65f64;
    (*schedule)
        .phase_modulation_scale[ENTRY_ALPHA_CONSTANT_DRAG as ::core::ffi::c_int
        as usize] = 1.0f64;
    (*schedule)
        .phase_modulation_scale[ENTRY_ALPHA_TRANSITION as ::core::ffi::c_int as usize] = 0.45f64;
    (*schedule).drag_error_gain = 3.0f64;
    (*schedule).maximum_modulation = 4.0f64;
    (*schedule).modulation_rate_limit = 0.8f64;
    (*schedule).target_rate_limit = 2.0f64;
    (*schedule).minimum_protective_aoa = trim;
    (*schedule).minimum_aero_confidence = 0.20f64;
    (*schedule).dynamic_pressure_protect_ratio = 0.92f64;
    (*schedule).g_load_protect_ratio = 0.92f64;
    (*schedule).stall_fraction_limit = 0.12f64;
    (*schedule).calibrated_stall_speed_margin = 1.15f64;
}
#[no_mangle]
pub unsafe extern "C" fn entry_alpha_schedule_valid(
    mut schedule: *const EntryAlphaSchedule,
) -> bool {
    if schedule.is_null() || (*schedule).point_count < 2 as size_t
        || (*schedule).point_count > ENTRY_ALPHA_MAX_SCHEDULE_POINTS as size_t
    {
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*schedule).point_count {
        if (*schedule).relative_velocity[i as usize].is_finite() as i32 == 0
            || (*schedule).alpha[i as usize].is_finite() as i32 == 0
            || (*schedule).relative_velocity[i as usize]
                < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            return false_0 != 0;
        }
        if i > 0 as size_t
            && (*schedule).relative_velocity[i as usize]
                <= (*schedule).relative_velocity[i.wrapping_sub(1 as size_t) as usize]
        {
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    let mut i_0: size_t = 0 as size_t;
    while i_0 < ENTRY_ALPHA_PHASE_COUNT as ::core::ffi::c_int as size_t {
        if (*schedule).phase_modulation_scale[i_0 as usize].is_finite() as i32 == 0
            || (*schedule).phase_modulation_scale[i_0 as usize]
                < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            return false_0 != 0;
        }
        i_0 = i_0.wrapping_add(1);
    }
    return (*schedule).drag_error_gain.is_finite() as i32 != 0
        && (*schedule).maximum_modulation.is_finite() as i32 != 0
        && (*schedule).maximum_modulation
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).modulation_rate_limit.is_finite() as i32 != 0
        && (*schedule).modulation_rate_limit
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).target_rate_limit.is_finite() as i32 != 0
        && (*schedule).target_rate_limit
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).minimum_protective_aoa.is_finite() as i32 != 0
        && (*schedule).minimum_protective_aoa
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).minimum_aero_confidence.is_finite() as i32 != 0
        && (*schedule).minimum_aero_confidence
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).dynamic_pressure_protect_ratio.is_finite() as i32 != 0
        && (*schedule).dynamic_pressure_protect_ratio
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).dynamic_pressure_protect_ratio
            < 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).g_load_protect_ratio.is_finite() as i32 != 0
        && (*schedule).g_load_protect_ratio
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).g_load_protect_ratio
            < 1 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).stall_fraction_limit.is_finite() as i32 != 0
        && (*schedule).stall_fraction_limit
            >= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && (*schedule).calibrated_stall_speed_margin.is_finite() as i32 != 0
        && (*schedule).calibrated_stall_speed_margin >= 1.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn entry_alpha_schedule_interpolate(
    mut schedule: *const EntryAlphaSchedule,
    mut relative_velocity: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !entry_alpha_schedule_valid(schedule) || relative_velocity.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    if relative_velocity
        <= (*schedule).relative_velocity[0 as ::core::ffi::c_int as usize]
    {
        return (*schedule).alpha[0 as ::core::ffi::c_int as usize];
    }
    let mut last: size_t = (*schedule).point_count.wrapping_sub(1 as size_t);
    if relative_velocity >= (*schedule).relative_velocity[last as usize] {
        return (*schedule).alpha[last as usize];
    }
    let mut i: size_t = 0 as size_t;
    while i < last {
        let mut lo: ::core::ffi::c_double = (*schedule).relative_velocity[i as usize];
        let mut hi: ::core::ffi::c_double = (*schedule)
            .relative_velocity[i.wrapping_add(1 as size_t) as usize];
        if relative_velocity <= hi {
            let mut u: ::core::ffi::c_double = (relative_velocity - lo) / (hi - lo);
            return (*schedule).alpha[i as usize]
                + ((*schedule).alpha[i.wrapping_add(1 as size_t) as usize]
                    - (*schedule).alpha[i as usize]) * u;
        }
        i = i.wrapping_add(1);
    }
    return (*schedule).alpha[last as usize];
}
#[no_mangle]
pub unsafe extern "C" fn entry_alpha_minimum_drag_tracking_aoa(
    mut schedule: *const EntryAlphaSchedule,
    mut phase: EntryAlphaPhase,
    mut relative_velocity: ::core::ffi::c_double,
    mut dynamic_pressure: ::core::ffi::c_double,
    mut maximum_dynamic_pressure: ::core::ffi::c_double,
    mut aero_confidence: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !entry_alpha_schedule_valid(schedule)
        || (phase as ::core::ffi::c_uint) < 0 as ::core::ffi::c_uint
        || phase as ::core::ffi::c_uint
            >= ENTRY_ALPHA_PHASE_COUNT as ::core::ffi::c_int as ::core::ffi::c_uint
        || relative_velocity.is_finite() as i32 == 0
        || relative_velocity < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || dynamic_pressure.is_finite() as i32 == 0
        || dynamic_pressure < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || maximum_dynamic_pressure.is_finite() as i32 == 0
        || maximum_dynamic_pressure <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || aero_confidence.is_finite() as i32 == 0
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut nominal: ::core::ffi::c_double = entry_alpha_schedule_interpolate(
        schedule,
        relative_velocity,
    );
    if nominal.is_finite() as i32 == 0 {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut confidence_scale: ::core::ffi::c_double = 1.0f64;
    if aero_confidence < (*schedule).minimum_aero_confidence {
        confidence_scale = 0.0f64;
    } else if (*schedule).minimum_aero_confidence < 1.0f64 {
        confidence_scale = smoothstep01(
            (aero_confidence - (*schedule).minimum_aero_confidence)
                / fmax(1.0f64 - (*schedule).minimum_aero_confidence, 1e-6f64),
        );
    }
    let mut modulation: ::core::ffi::c_double = -1.5f64 * (*schedule).drag_error_gain
        * (*schedule).phase_modulation_scale[phase as usize] * confidence_scale;
    modulation *= negative_drag_modulation_scale(phase);
    modulation = clampd(
        modulation,
        -(*schedule).maximum_modulation,
        (*schedule).maximum_modulation,
    );
    let mut desired: ::core::ffi::c_double = clampd(
        nominal + modulation,
        nominal - (*schedule).maximum_modulation,
        nominal + (*schedule).maximum_modulation,
    );
    desired = fmax(desired, (*schedule).minimum_protective_aoa);
    let mut q_ratio: ::core::ffi::c_double = dynamic_pressure
        / fmax(maximum_dynamic_pressure, 1.0f64);
    if q_ratio > (*schedule).dynamic_pressure_protect_ratio {
        let mut severity: ::core::ffi::c_double = smoothstep01(
            (q_ratio - (*schedule).dynamic_pressure_protect_ratio)
                / fmax(1.0f64 - (*schedule).dynamic_pressure_protect_ratio, 1e-6f64),
        );
        desired = fmax(
            desired,
            nominal + (*schedule).maximum_modulation * 0.75f64 * severity,
        );
    }
    return fmax(0.0f64, desired);
}
unsafe extern "C" fn entry_alpha_input_valid(mut input: *const EntryAlphaInput) -> bool {
    if input.is_null()
        || ((*input).phase as ::core::ffi::c_uint) < 0 as ::core::ffi::c_uint
        || (*input).phase as ::core::ffi::c_uint
            >= ENTRY_ALPHA_PHASE_COUNT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return false_0 != 0;
    }
    if (*input).relative_velocity.is_finite() as i32 == 0
        || (*input).relative_velocity < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).current_aoa.is_finite() as i32 == 0
        || (*input).current_aoa_rate.is_finite() as i32 == 0
        || (*input).dt.is_finite() as i32 == 0
        || (*input).dt <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).maximum_aoa.is_finite() as i32 == 0
        || (*input).maximum_aoa <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).minimum_safe_speed.is_finite() as i32 == 0
        || (*input).minimum_safe_speed
            <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).dynamic_pressure.is_finite() as i32 == 0
        || (*input).dynamic_pressure < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).maximum_dynamic_pressure.is_finite() as i32 == 0
        || (*input).maximum_dynamic_pressure
            <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).g_load.is_finite() as i32 == 0
        || (*input).g_load < 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).maximum_g_load.is_finite() as i32 == 0
        || (*input).maximum_g_load <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || (*input).aero_confidence.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    if (*input).has_previous_target as ::core::ffi::c_int != 0
        && (*input).previous_target_aoa.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    if (*input).has_previous_modulation as ::core::ffi::c_int != 0
        && (*input).previous_modulation.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    if (*input).calibrated_stall_speed.is_finite() as i32 != 0
        && (*input).calibrated_stall_speed
            < 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn entry_alpha_command(
    mut schedule: *const EntryAlphaSchedule,
    mut input: *const EntryAlphaInput,
) -> EntryAlphaResult {
    let mut result: EntryAlphaResult = EntryAlphaResult {
        valid: 0 as ::core::ffi::c_int != 0,
        degraded: false,
        nominal_aoa: 0.,
        requested_modulation: 0.,
        modulation: 0.,
        unconstrained_target_aoa: 0.,
        target_aoa: 0.,
        limiting_reason: ENTRY_ALPHA_LIMIT_NONE,
    };
    result.limiting_reason = ENTRY_ALPHA_LIMIT_INVALID_INPUT;
    if !entry_alpha_schedule_valid(schedule) || !entry_alpha_input_valid(input) {
        if !input.is_null() && (*input).current_aoa.is_finite() as i32 != 0 {
            result.target_aoa = (*input).current_aoa;
        }
        return result;
    }
    result.nominal_aoa = entry_alpha_schedule_interpolate(
        schedule,
        (*input).relative_velocity,
    );
    if result.nominal_aoa.is_finite() as i32 == 0 {
        return result;
    }
    result.valid = true_0 != 0;
    result.limiting_reason = ENTRY_ALPHA_LIMIT_NONE;
    let mut phase_scale: ::core::ffi::c_double = (*schedule)
        .phase_modulation_scale[(*input).phase as usize];
    let mut confidence_scale: ::core::ffi::c_double = 1.0f64;
    if (*input).aero_confidence < (*schedule).minimum_aero_confidence {
        confidence_scale = 0.0f64;
        result.degraded = true_0 != 0;
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE);
    } else if (*schedule).minimum_aero_confidence < 1.0f64 {
        confidence_scale = smoothstep01(
            ((*input).aero_confidence - (*schedule).minimum_aero_confidence)
                / fmax(1.0f64 - (*schedule).minimum_aero_confidence, 1e-6f64),
        );
    }
    let mut drag_error: ::core::ffi::c_double = 0.0f64;
    if (*input).reference_drag_accel.is_finite() as i32 != 0
        && (*input).measured_drag_accel.is_finite() as i32 != 0
    {
        let mut scale: ::core::ffi::c_double = fmax(
            fabs((*input).reference_drag_accel),
            0.25f64,
        );
        drag_error = clampd(
            ((*input).reference_drag_accel - (*input).measured_drag_accel) / scale,
            -1.5f64,
            1.5f64,
        );
    } else {
        result.degraded = true_0 != 0;
        confidence_scale = 0.0f64;
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE);
    }
    let mut requested_modulation: ::core::ffi::c_double = (*schedule).drag_error_gain
        * drag_error * phase_scale * confidence_scale;
    if requested_modulation < 0.0f64 {
        requested_modulation *= negative_drag_modulation_scale((*input).phase);
    }
    result.requested_modulation = clampd(
        requested_modulation,
        -(*schedule).maximum_modulation,
        (*schedule).maximum_modulation,
    );
    let mut previous_mod: ::core::ffi::c_double = if (*input).has_previous_modulation
        as ::core::ffi::c_int != 0
    {
        (*input).previous_modulation
    } else {
        0.0f64
    };
    let mut modulation_step: ::core::ffi::c_double = (*schedule).modulation_rate_limit
        * (*input).dt;
    result.modulation = clampd(
        result.requested_modulation,
        previous_mod - modulation_step,
        previous_mod + modulation_step,
    );
    if fabs(result.modulation - result.requested_modulation) > 1e-9f64 {
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_COMMAND_RATE);
    }
    let mut desired: ::core::ffi::c_double = clampd(
        result.nominal_aoa + result.modulation,
        result.nominal_aoa - (*schedule).maximum_modulation,
        result.nominal_aoa + (*schedule).maximum_modulation,
    );
    let mut protective_floor: ::core::ffi::c_double = clampd(
        (*schedule).minimum_protective_aoa,
        0.0f64,
        (*input).maximum_aoa,
    );
    if desired < protective_floor {
        desired = protective_floor;
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION);
    }
    let mut q_ratio: ::core::ffi::c_double = (*input).dynamic_pressure
        / fmax((*input).maximum_dynamic_pressure, 1.0f64);
    if q_ratio > (*schedule).dynamic_pressure_protect_ratio {
        let mut severity: ::core::ffi::c_double = smoothstep01(
            (q_ratio - (*schedule).dynamic_pressure_protect_ratio)
                / fmax(1.0f64 - (*schedule).dynamic_pressure_protect_ratio, 1e-6f64),
        );
        desired = fmax(
            desired,
            result.nominal_aoa + (*schedule).maximum_modulation * 0.75f64 * severity,
        );
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE);
    }
    let mut g_ratio: ::core::ffi::c_double = (*input).g_load
        / fmax((*input).maximum_g_load, 0.1f64);
    if g_ratio > (*schedule).g_load_protect_ratio {
        let mut severity_0: ::core::ffi::c_double = smoothstep01(
            (g_ratio - (*schedule).g_load_protect_ratio)
                / fmax(1.0f64 - (*schedule).g_load_protect_ratio, 1e-6f64),
        );
        desired = fmin(
            desired,
            result.nominal_aoa - (*schedule).maximum_modulation * severity_0,
        );
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_G_LOAD);
    }
    result.unconstrained_target_aoa = desired;
    let mut target_reference: ::core::ffi::c_double = if (*input).has_previous_target
        as ::core::ffi::c_int != 0
    {
        (*input).previous_target_aoa
    } else {
        (*input).current_aoa
    };
    let mut direction: ::core::ffi::c_double = desired - target_reference;
    let mut rate_limit: ::core::ffi::c_double = (*schedule).target_rate_limit;
    if direction * (*input).current_aoa_rate < 0.0f64 {
        rate_limit
            *= clampd(
                1.0f64
                    - fabs((*input).current_aoa_rate)
                        / fmax((*schedule).target_rate_limit * 4.0f64, 1e-6f64),
                0.45f64,
                1.0f64,
            );
    }
    let mut target_step: ::core::ffi::c_double = rate_limit * (*input).dt;
    let mut target: ::core::ffi::c_double = clampd(
        desired,
        target_reference - target_step,
        target_reference + target_step,
    );
    if fabs(target - desired) > 1e-9f64 {
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_COMMAND_RATE);
    }
    if target > (*input).maximum_aoa {
        target = (*input).maximum_aoa;
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_MAX_AOA);
    }
    if target < 0.0f64 {
        target = 0.0f64;
    }
    let mut stall_speed_risk: bool = false_0 != 0;
    if (*input).calibrated_stall_speed.is_finite() as i32 != 0
        && (*input).calibrated_stall_speed
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        stall_speed_risk = (*input).relative_velocity
            < (*input).calibrated_stall_speed
                * (*schedule).calibrated_stall_speed_margin;
    }
    stall_speed_risk = stall_speed_risk as ::core::ffi::c_int != 0
        || (*input).relative_velocity < (*input).minimum_safe_speed * 1.10f64;
    let mut stall_fraction_risk: bool = (*input).stall_fraction_is_measured
        as ::core::ffi::c_int != 0
        && (*input).stall_fraction > (*schedule).stall_fraction_limit;
    if stall_fraction_risk as ::core::ffi::c_int != 0
        || stall_speed_risk as ::core::ffi::c_int != 0
    {
        let mut stall_cap: ::core::ffi::c_double = if (*input)
            .stall_margin_aoa_limit
            .is_finite() as i32 != 0
            && (*input).stall_margin_aoa_limit
                > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        {
            (*input).stall_margin_aoa_limit
        } else {
            fmin(result.nominal_aoa, fmax(4.0f64, (*input).maximum_aoa * 0.40f64))
        };
        stall_cap = clampd(stall_cap, 0.0f64, (*input).maximum_aoa);
        if target > stall_cap {
            target = stall_cap;
        }
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    } else if (*input).stall_margin_aoa_limit.is_finite() as i32 != 0
        && (*input).stall_margin_aoa_limit
            > 0 as ::core::ffi::c_int as ::core::ffi::c_double
        && target > (*input).stall_margin_aoa_limit
    {
        target = clampd((*input).stall_margin_aoa_limit, 0.0f64, (*input).maximum_aoa);
        set_reason(&raw mut result, ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    }
    result.target_aoa = target;
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn entry_alpha_limit_reason_string(
    mut reason: EntryAlphaLimitReason,
) -> *const ::core::ffi::c_char {
    match reason as ::core::ffi::c_uint {
        0 => return b"none\0" as *const u8 as *const ::core::ffi::c_char,
        1 => return b"maximum-aoa\0" as *const u8 as *const ::core::ffi::c_char,
        2 => return b"stall-margin\0" as *const u8 as *const ::core::ffi::c_char,
        3 => return b"dynamic-pressure\0" as *const u8 as *const ::core::ffi::c_char,
        4 => return b"g-load\0" as *const u8 as *const ::core::ffi::c_char,
        5 => return b"thermal-protection\0" as *const u8 as *const ::core::ffi::c_char,
        6 => return b"aero-confidence\0" as *const u8 as *const ::core::ffi::c_char,
        7 => return b"command-rate\0" as *const u8 as *const ::core::ffi::c_char,
        8 => return b"invalid-input\0" as *const u8 as *const ::core::ffi::c_char,
        _ => {}
    }
    return b"invalid-input\0" as *const u8 as *const ::core::ffi::c_char;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
