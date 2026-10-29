use crate::sys::*;
extern "C" {
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn ceil(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn copysign(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn nextafter(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
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
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralLongitudinalDemand {
    pub valid: bool,
    pub confidence: ::core::ffi::c_double,
    pub reference_drag_accel: ::core::ffi::c_double,
    pub drag_error_accel: ::core::ffi::c_double,
    pub predicted_range_error: ::core::ffi::c_double,
    pub range_error_scale: ::core::ffi::c_double,
    pub required_vertical_lift_accel: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralLimits {
    pub maximum_bank_deg: ::core::ffi::c_double,
    pub maximum_roll_rate_deg_s: ::core::ffi::c_double,
    pub maximum_roll_accel_deg_s2: ::core::ffi::c_double,
    pub minimum_leg_duration_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralInput {
    pub ut: ::core::ffi::c_double,
    pub dt: ::core::ffi::c_double,
    pub relative_speed: ::core::ffi::c_double,
    pub measured_bank_deg: ::core::ffi::c_double,
    pub measured_bank_rate_deg_s: ::core::ffi::c_double,
    pub bank_effectiveness: ::core::ffi::c_double,
    pub lift_accel: ::core::ffi::c_double,
    pub has_measured_lift: bool,
    pub has_simulated_lift_observation: bool,
    pub authority_confidence: ::core::ffi::c_double,
    pub course_to_site_error_deg: ::core::ffi::c_double,
    pub has_crossrange_error: bool,
    pub crossrange_error_m: ::core::ffi::c_double,
    pub has_crossrange_rate: bool,
    pub crossrange_error_rate_mps: ::core::ffi::c_double,
    pub has_crossrange_accel: bool,
    pub crossrange_error_accel_mps2: ::core::ffi::c_double,
    pub crossrange_uncertainty_m: ::core::ffi::c_double,
    pub crossrange_arrest_prediction: bool,
    pub has_crossrange_course_alignment: bool,
    pub crossrange_course_alignment: ::core::ffi::c_double,
    pub has_sideslip: bool,
    pub sideslip_deg: ::core::ffi::c_double,
    pub maximum_sideslip_deg: ::core::ffi::c_double,
    pub longitudinal: EntryLateralLongitudinalDemand,
    pub has_bank_magnitude: bool,
    pub bank_magnitude_deg: ::core::ffi::c_double,
    pub has_azimuth_error: bool,
    pub azimuth_error_deg: ::core::ffi::c_double,
    pub azimuth_deadband_deg: ::core::ffi::c_double,
    pub has_time_to_go: bool,
    pub time_to_go_s: ::core::ffi::c_double,
    pub has_point_miss: bool,
    pub point_miss_m: ::core::ffi::c_double,
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
pub struct EntryLateralOutput {
    pub valid: bool,
    pub reversal_requested: bool,
    pub leg_captured: bool,
    pub reversal_armed: bool,
    pub degraded_authority: bool,
    pub bank_sign: ::core::ffi::c_double,
    pub bank_magnitude_deg: ::core::ffi::c_double,
    pub energy_bank_magnitude_deg: ::core::ffi::c_double,
    pub raw_target_bank_deg: ::core::ffi::c_double,
    pub target_bank_deg: ::core::ffi::c_double,
    pub target_bank_rate_deg_s: ::core::ffi::c_double,
    pub corridor_metric: ::core::ffi::c_double,
    pub course_corridor_deg: ::core::ffi::c_double,
    pub crossrange_corridor_m: ::core::ffi::c_double,
    pub projected_crossrange_error_m: ::core::ffi::c_double,
    pub reversal_response_time_s: ::core::ffi::c_double,
    pub required_lateral_lift_accel: ::core::ffi::c_double,
    pub zero_effort_crossrange_m: ::core::ffi::c_double,
    pub lateral_demand_binding: bool,
    pub expected_course_rate_deg_s: ::core::ffi::c_double,
    pub sideslip_rate_deg_s: ::core::ffi::c_double,
    pub coordination_roll_rate_limit_deg_s: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralTurnInput {
    pub horizontal_speed_mps: ::core::ffi::c_double,
    pub current_course_deg: ::core::ffi::c_double,
    pub target_course_deg: ::core::ffi::c_double,
    pub current_along_m: ::core::ffi::c_double,
    pub current_cross_m: ::core::ffi::c_double,
    pub target_along_m: ::core::ffi::c_double,
    pub target_cross_m: ::core::ffi::c_double,
    pub measured_bank_deg: ::core::ffi::c_double,
    pub measured_bank_rate_deg_s: ::core::ffi::c_double,
    pub lift_accel_mps2: ::core::ffi::c_double,
    pub bank_effectiveness: ::core::ffi::c_double,
    pub maximum_bank_deg: ::core::ffi::c_double,
    pub maximum_roll_rate_deg_s: ::core::ffi::c_double,
    pub maximum_roll_accel_deg_s2: ::core::ffi::c_double,
    pub capture_radius_m: ::core::ffi::c_double,
    pub position_uncertainty_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryLateralTurnEnvelope {
    pub valid: bool,
    pub turn_sign: ::core::ffi::c_double,
    pub response_time_s: ::core::ffi::c_double,
    pub response_distance_m: ::core::ffi::c_double,
    pub response_heading_change_deg: ::core::ffi::c_double,
    pub projected_along_m: ::core::ffi::c_double,
    pub projected_cross_m: ::core::ffi::c_double,
    pub projected_course_deg: ::core::ffi::c_double,
    pub remaining_course_change_deg: ::core::ffi::c_double,
    pub minimum_turn_radius_m: ::core::ffi::c_double,
    pub ideal_turn_radius_m: ::core::ffi::c_double,
    pub radius_margin_m: ::core::ffi::c_double,
    pub endpoint_error_m: ::core::ffi::c_double,
    pub capture_margin_m: ::core::ffi::c_double,
    pub feasibility_margin_m: ::core::ffi::c_double,
}
pub type size_t = usize;
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const ENTRY_LATERAL_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const ENTRY_LATERAL_DEG2RAD: ::core::ffi::c_double = ENTRY_LATERAL_PI / 180.0f64;
pub const ENTRY_LATERAL_RAD2DEG: ::core::ffi::c_double = 180.0f64 / ENTRY_LATERAL_PI;
pub const ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG: ::core::ffi::c_double = 0.25f64;
unsafe extern "C" fn clamp_value(
    mut value: ::core::ffi::c_double,
    mut minimum: ::core::ffi::c_double,
    mut maximum: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return fmin(fmax(value, minimum), maximum);
}
unsafe extern "C" fn unit_sign(
    mut value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_sign_negative() as ::core::ffi::c_int != 0 {
        -1.0f64
    } else {
        1.0f64
    };
}
unsafe extern "C" fn finite_or(
    mut value: ::core::ffi::c_double,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if value.is_finite() as i32 != 0 { value } else { fallback };
}
unsafe extern "C" fn wrap_signed_deg(
    mut value: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    value = remainder(value, 360.0f64);
    if value <= -180.0f64 {
        value += 360.0f64;
    }
    if value > 180.0f64 {
        value -= 360.0f64;
    }
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn entry_lateral_geometry_turn_sign(
    mut current_course_deg: ::core::ffi::c_double,
    mut current_along_m: ::core::ffi::c_double,
    mut current_cross_m: ::core::ffi::c_double,
    mut target_along_m: ::core::ffi::c_double,
    mut target_cross_m: ::core::ffi::c_double,
    mut target_course_deg: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if current_course_deg.is_finite() as i32 == 0
        || current_along_m.is_finite() as i32 == 0
        || current_cross_m.is_finite() as i32 == 0
        || target_along_m.is_finite() as i32 == 0
        || target_cross_m.is_finite() as i32 == 0
        || target_course_deg.is_finite() as i32 == 0
    {
        return 0.0f64;
    }
    let mut da: ::core::ffi::c_double = target_along_m - current_along_m;
    let mut dc: ::core::ffi::c_double = target_cross_m - current_cross_m;
    let mut coordinate_scale: ::core::ffi::c_double = fmax(
        1.0f64,
        fmax(
            fmax(fabs(current_along_m), fabs(current_cross_m)),
            fmax(fabs(target_along_m), fabs(target_cross_m)),
        ),
    );
    let mut position_resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON)
        * coordinate_scale;
    let mut angular_resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON)
        * ENTRY_LATERAL_RAD2DEG;
    if hypot(da, dc) > position_resolution {
        let mut point_course: ::core::ffi::c_double = atan2(dc, da)
            * ENTRY_LATERAL_RAD2DEG;
        let mut point_turn: ::core::ffi::c_double = wrap_signed_deg(
            point_course - current_course_deg,
        );
        if fabs(point_turn) > angular_resolution {
            return unit_sign(point_turn);
        }
    }
    let mut terminal_turn: ::core::ffi::c_double = wrap_signed_deg(
        target_course_deg - current_course_deg,
    );
    if fabs(terminal_turn) > angular_resolution {
        return unit_sign(terminal_turn);
    }
    return 0.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn entry_lateral_crossrange_arrest_distance(
    mut rate: ::core::ffi::c_double,
    mut alignment: ::core::ffi::c_double,
    mut lift: ::core::ffi::c_double,
    mut bank: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if rate.is_finite() as i32 == 0 || alignment.is_finite() as i32 == 0
        || alignment <= 0.0f64 || alignment > 1.0f64 || lift.is_finite() as i32 == 0
        || lift <= 0.0f64 || bank.is_finite() as i32 == 0 || fabs(bank) >= 90.0f64
    {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    let mut authority: ::core::ffi::c_double = lift
        * fabs(sin(bank * ENTRY_LATERAL_DEG2RAD));
    if authority <= DBL_EPSILON {
        return ::core::f32::NAN as ::core::ffi::c_double;
    }
    return copysign(rate * rate / (authority * (1.0f64 + alignment)), rate);
}
unsafe extern "C" fn crossrange_arrest(
    mut in_0: *const EntryLateralInput,
    mut magnitude: ::core::ffi::c_double,
    mut response_time: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if !(*in_0).crossrange_arrest_prediction || !(*in_0).has_crossrange_rate
        || !(*in_0).has_crossrange_course_alignment
    {
        return 0.0f64;
    }
    let mut rate: ::core::ffi::c_double = (*in_0).crossrange_error_rate_mps;
    if (*in_0).has_crossrange_accel as ::core::ffi::c_int != 0
        && (*in_0).crossrange_error_accel_mps2.is_finite() as i32 != 0
    {
        rate += (*in_0).crossrange_error_accel_mps2 * response_time;
    }
    let mut distance: ::core::ffi::c_double = entry_lateral_crossrange_arrest_distance(
        rate,
        (*in_0).crossrange_course_alignment,
        (*in_0).lift_accel,
        magnitude * (*in_0).bank_effectiveness,
    );
    return if distance.is_finite() as i32 != 0 { distance } else { 0.0f64 };
}
#[no_mangle]
pub unsafe extern "C" fn entry_lateral_default_limits() -> EntryLateralLimits {
    let mut limits: EntryLateralLimits = EntryLateralLimits {
        maximum_bank_deg: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        maximum_roll_rate_deg_s: 0.,
        maximum_roll_accel_deg_s2: 0.,
        minimum_leg_duration_s: 0.,
    };
    return limits;
}
#[no_mangle]
pub unsafe extern "C" fn entry_lateral_state_init(
    mut state: *mut EntryLateralState,
    mut ut: ::core::ffi::c_double,
    mut bank_sign_hint: ::core::ffi::c_double,
    mut measured_bank_deg: ::core::ffi::c_double,
) {
    if state.is_null() {
        return;
    }
    memset(
        state as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryLateralState>() as size_t,
    );
    (*state).initialized = true_0 != 0;
    if bank_sign_hint.is_finite() as i32 != 0 && fabs(bank_sign_hint) > DBL_EPSILON {
        (*state).bank_sign = unit_sign(bank_sign_hint);
    } else if measured_bank_deg.is_finite() as i32 != 0
        && fabs(measured_bank_deg) > DBL_EPSILON
    {
        (*state).bank_sign = unit_sign(measured_bank_deg);
    } else {
        (*state).bank_sign = 1.0f64;
    }
    (*state).reversal_armed = false_0 != 0;
    (*state).last_reversal_ut = -::core::f32::INFINITY as ::core::ffi::c_double;
    (*state).command_bank_deg = finite_or(measured_bank_deg, 0.0f64);
    (*state).command_bank_rate_deg_s = 0.0f64;
    (*state).leg_captured_ut = finite_or(ut, 0.0f64);
}
unsafe extern "C" fn limits_valid(mut limits: *const EntryLateralLimits) -> bool {
    return !limits.is_null() && (*limits).maximum_bank_deg.is_finite() as i32 != 0
        && (*limits).maximum_bank_deg >= 0.0f64
        && (*limits).maximum_roll_rate_deg_s.is_finite() as i32 != 0
        && (*limits).maximum_roll_rate_deg_s > 0.0f64
        && (*limits).maximum_roll_accel_deg_s2.is_finite() as i32 != 0
        && (*limits).maximum_roll_accel_deg_s2 > 0.0f64
        && (*limits).minimum_leg_duration_s.is_finite() as i32 != 0
        && (*limits).minimum_leg_duration_s >= 0.0f64;
}
unsafe extern "C" fn choose_bank_sign(
    mut state: *const EntryLateralState,
    mut input: *const EntryLateralInput,
) -> ::core::ffi::c_double {
    if !state.is_null() && (*state).initialized as ::core::ffi::c_int != 0
        && fabs((*state).bank_sign) > DBL_EPSILON
    {
        return unit_sign((*state).bank_sign);
    }
    if (*input).has_azimuth_error as ::core::ffi::c_int != 0
        && (*input).azimuth_error_deg.is_finite() as i32 != 0
        && fabs((*input).azimuth_error_deg) > DBL_EPSILON
    {
        return unit_sign((*input).azimuth_error_deg);
    }
    if (*input).measured_bank_deg.is_finite() as i32 != 0
        && fabs((*input).measured_bank_deg) > DBL_EPSILON
    {
        return unit_sign((*input).measured_bank_deg);
    }
    if (*input).has_crossrange_error as ::core::ffi::c_int != 0
        && (*input).crossrange_error_m.is_finite() as i32 != 0
        && fabs((*input).crossrange_error_m) > DBL_EPSILON
    {
        return unit_sign((*input).crossrange_error_m);
    }
    if (*input).course_to_site_error_deg.is_finite() as i32 != 0
        && fabs((*input).course_to_site_error_deg) > DBL_EPSILON
    {
        return unit_sign((*input).course_to_site_error_deg);
    }
    return 1.0f64;
}
unsafe extern "C" fn lateral_demand(
    mut input: *const EntryLateralInput,
    mut accel: *mut ::core::ffi::c_double,
    mut zem: *mut ::core::ffi::c_double,
) -> bool {
    if (*input).has_time_to_go as ::core::ffi::c_int != 0
        && (*input).time_to_go_s.is_finite() as i32 != 0
        && (*input).time_to_go_s > DBL_EPSILON
        && (*input).has_point_miss as ::core::ffi::c_int != 0
        && (*input).point_miss_m.is_finite() as i32 != 0
    {
        let mut t: ::core::ffi::c_double = (*input).time_to_go_s;
        let mut miss: ::core::ffi::c_double = (*input).point_miss_m;
        let mut uncertainty: ::core::ffi::c_double = if (*input)
            .crossrange_uncertainty_m
            .is_finite() as i32 != 0
        {
            fmax(0.0f64, (*input).crossrange_uncertainty_m)
        } else {
            0.0f64
        };
        if fabs(miss) <= uncertainty {
            miss = 0.0f64;
        } else {
            miss -= copysign(uncertainty, miss);
        }
        *zem = miss;
        *accel = 2.0f64 * fabs(miss) / (t * t);
        return true_0 != 0;
    }
    if !(*input).has_time_to_go || (*input).time_to_go_s.is_finite() as i32 == 0
        || !((*input).time_to_go_s > DBL_EPSILON) || !(*input).has_crossrange_error
        || (*input).crossrange_error_m.is_finite() as i32 == 0
        || !(*input).has_crossrange_rate
        || (*input).crossrange_error_rate_mps.is_finite() as i32 == 0
        || !(*input).has_crossrange_course_alignment
        || (*input).crossrange_course_alignment.is_finite() as i32 == 0
        || !((*input).crossrange_course_alignment > DBL_EPSILON)
    {
        return false_0 != 0;
    }
    let mut t_0: ::core::ffi::c_double = (*input).time_to_go_s;
    let mut miss_0: ::core::ffi::c_double = (*input).crossrange_error_m
        + (*input).crossrange_error_rate_mps * t_0;
    let mut uncertainty_0: ::core::ffi::c_double = if (*input)
        .crossrange_uncertainty_m
        .is_finite() as i32 != 0
    {
        fmax(0.0f64, (*input).crossrange_uncertainty_m)
    } else {
        0.0f64
    };
    if fabs(miss_0) <= uncertainty_0 {
        miss_0 = 0.0f64;
    } else {
        miss_0 -= copysign(uncertainty_0, miss_0);
    }
    *zem = miss_0;
    *accel = 2.0f64 * fabs(miss_0) / (t_0 * t_0 * (*input).crossrange_course_alignment);
    return true_0 != 0;
}
unsafe extern "C" fn bank_magnitude(
    mut input: *const EntryLateralInput,
    mut limits: *const EntryLateralLimits,
    mut out: *mut ::core::ffi::c_double,
    mut energy_out: *mut ::core::ffi::c_double,
    mut lateral_accel: *mut ::core::ffi::c_double,
    mut zem: *mut ::core::ffi::c_double,
    mut binding: *mut bool,
) -> bool {
    *lateral_accel = ::core::f32::NAN as ::core::ffi::c_double;
    *zem = ::core::f32::NAN as ::core::ffi::c_double;
    *binding = false_0 != 0;
    *energy_out = 0.0f64;
    if out.is_null() || !(*input).longitudinal.valid
        || (*input).longitudinal.required_vertical_lift_accel.is_finite() as i32 == 0
        || (*input).longitudinal.required_vertical_lift_accel < 0.0f64
    {
        return false_0 != 0;
    }
    let mut measured_lift: ::core::ffi::c_double = finite_or(
        (*input).lift_accel,
        0.0f64,
    );
    let mut confidence: ::core::ffi::c_double = clamp_value(
        fmin(
            finite_or((*input).authority_confidence, 0.0f64),
            finite_or((*input).longitudinal.confidence, 0.0f64),
        ),
        0.0f64,
        1.0f64,
    );
    let mut available_lift: ::core::ffi::c_double = if (*input).has_measured_lift
        as ::core::ffi::c_int != 0
        || (*input).has_simulated_lift_observation as ::core::ffi::c_int != 0
    {
        measured_lift
    } else {
        measured_lift * confidence
    };
    if !(available_lift > DBL_EPSILON) {
        return false_0 != 0;
    }
    let mut effectiveness: ::core::ffi::c_double = finite_or(
        (*input).bank_effectiveness,
        1.0f64,
    );
    if !(effectiveness > DBL_EPSILON) {
        return false_0 != 0;
    }
    let mut bank_limit: ::core::ffi::c_double = fabs((*limits).maximum_bank_deg);
    let mut effective_bank_limit: ::core::ffi::c_double = bank_limit * effectiveness;
    if !(effective_bank_limit < 90.0f64) {
        effective_bank_limit = nextafter(90.0f64, 0.0f64);
    }
    let mut vertical: ::core::ffi::c_double = (*input)
        .longitudinal
        .required_vertical_lift_accel;
    let mut energy_bank: ::core::ffi::c_double = acos(
        clamp_value(
            vertical / available_lift,
            cos(effective_bank_limit * ENTRY_LATERAL_DEG2RAD),
            1.0f64,
        ),
    ) * ENTRY_LATERAL_RAD2DEG;
    *energy_out = clamp_value(energy_bank / effectiveness, 0.0f64, bank_limit);
    let mut effective_bank: ::core::ffi::c_double = 0.;
    let mut lateral: ::core::ffi::c_double = 0.0f64;
    if lateral_demand(input, &raw mut lateral, zem) {
        *lateral_accel = lateral;
    }
    if hypot(vertical, lateral) <= available_lift {
        effective_bank = acos(vertical / available_lift) * ENTRY_LATERAL_RAD2DEG;
    } else {
        effective_bank = atan2(lateral, vertical) * ENTRY_LATERAL_RAD2DEG;
        *binding = lateral > DBL_EPSILON;
    }
    effective_bank = fmin(effective_bank, effective_bank_limit);
    *out = clamp_value(effective_bank / effectiveness, 0.0f64, bank_limit);
    return true_0 != 0;
}
unsafe extern "C" fn update_bank_command(
    mut state: *mut EntryLateralState,
    mut target: ::core::ffi::c_double,
    mut limits: *const EntryLateralLimits,
    mut dt: ::core::ffi::c_double,
    mut measured_bank: ::core::ffi::c_double,
) {
    let mut bank_limit: ::core::ffi::c_double = fabs((*limits).maximum_bank_deg);
    let mut rate_limit: ::core::ffi::c_double = fabs((*limits).maximum_roll_rate_deg_s);
    let mut accel_limit: ::core::ffi::c_double = fabs(
        (*limits).maximum_roll_accel_deg_s2,
    );
    if !(dt > 0.0f64) {
        return;
    }
    target = clamp_value(target, -bank_limit, bank_limit);
    let mut error: ::core::ffi::c_double = target - (*state).command_bank_deg;
    let mut desired_rate: ::core::ffi::c_double = 0.0f64;
    if fabs(error) > DBL_EPSILON {
        let mut stopping_rate: ::core::ffi::c_double = sqrt(
            2.0f64 * accel_limit * fabs(error),
        );
        desired_rate = unit_sign(error) * fmin(rate_limit, stopping_rate);
        if measured_bank.is_finite() as i32 != 0 {
            let mut lead: ::core::ffi::c_double = unit_sign(error)
                * ((*state).command_bank_deg - measured_bank);
            let mut braking_reserve: ::core::ffi::c_double = rate_limit * rate_limit
                / (2.0f64 * accel_limit);
            let mut headroom: ::core::ffi::c_double = fmax(
                0.0f64,
                braking_reserve - fmax(0.0f64, lead),
            );
            desired_rate = unit_sign(error)
                * fmin(fabs(desired_rate), sqrt(2.0f64 * accel_limit * headroom));
        }
    }
    let mut rate_delta: ::core::ffi::c_double = desired_rate
        - (*state).command_bank_rate_deg_s;
    (*state).command_bank_rate_deg_s
        += clamp_value(rate_delta, -accel_limit * dt, accel_limit * dt);
    let mut next: ::core::ffi::c_double = (*state).command_bank_deg
        + (*state).command_bank_rate_deg_s * dt;
    if (target - (*state).command_bank_deg) * (target - next) <= 0.0f64 {
        next = target;
        (*state).command_bank_rate_deg_s = 0.0f64;
    }
    (*state).command_bank_deg = next;
}
#[no_mangle]
pub unsafe extern "C" fn entry_lateral_update(
    mut state: *mut EntryLateralState,
    mut input: *const EntryLateralInput,
    mut limits: *const EntryLateralLimits,
) -> EntryLateralOutput {
    let mut output: EntryLateralOutput = EntryLateralOutput {
        valid: false,
        reversal_requested: false,
        leg_captured: false,
        reversal_armed: false,
        degraded_authority: false,
        bank_sign: 0.,
        bank_magnitude_deg: 0.,
        energy_bank_magnitude_deg: 0.,
        raw_target_bank_deg: 0.,
        target_bank_deg: 0.,
        target_bank_rate_deg_s: 0.,
        corridor_metric: 0.,
        course_corridor_deg: 0.,
        crossrange_corridor_m: 0.,
        projected_crossrange_error_m: 0.,
        reversal_response_time_s: 0.,
        required_lateral_lift_accel: 0.,
        zero_effort_crossrange_m: 0.,
        lateral_demand_binding: false,
        expected_course_rate_deg_s: 0.,
        sideslip_rate_deg_s: 0.,
        coordination_roll_rate_limit_deg_s: 0.,
    };
    memset(
        &raw mut output as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryLateralOutput>() as size_t,
    );
    output.corridor_metric = ::core::f32::NAN as ::core::ffi::c_double;
    output.course_corridor_deg = ::core::f32::NAN as ::core::ffi::c_double;
    output.crossrange_corridor_m = ::core::f32::NAN as ::core::ffi::c_double;
    output.projected_crossrange_error_m = ::core::f32::NAN as ::core::ffi::c_double;
    output.reversal_response_time_s = ::core::f32::NAN as ::core::ffi::c_double;
    output.required_lateral_lift_accel = ::core::f32::NAN as ::core::ffi::c_double;
    output.zero_effort_crossrange_m = ::core::f32::NAN as ::core::ffi::c_double;
    if state.is_null() || input.is_null() || !limits_valid(limits)
        || (*input).ut.is_finite() as i32 == 0
        || (*input).relative_speed.is_finite() as i32 == 0
        || !((*input).relative_speed > 0.0f64) || (*input).dt.is_finite() as i32 == 0
        || (*input).dt < 0.0f64
    {
        return output;
    }
    if !(*state).initialized {
        entry_lateral_state_init(
            state,
            (*input).ut,
            choose_bank_sign(::core::ptr::null::<EntryLateralState>(), input),
            (*input).measured_bank_deg,
        );
        (*state).command_bank_rate_deg_s = clamp_value(
            finite_or((*input).measured_bank_rate_deg_s, 0.0f64),
            -fabs((*limits).maximum_roll_rate_deg_s),
            fabs((*limits).maximum_roll_rate_deg_s),
        );
    }
    let mut command_limits: EntryLateralLimits = *limits;
    let mut sideslip_rate: ::core::ffi::c_double = ::core::f32::NAN
        as ::core::ffi::c_double;
    let mut coordination_rate: ::core::ffi::c_double = fabs(
        (*limits).maximum_roll_rate_deg_s,
    );
    let mut sideslip_valid: bool = (*input).has_sideslip as ::core::ffi::c_int != 0
        && (*input).sideslip_deg.is_finite() as i32 != 0
        && (*input).maximum_sideslip_deg.is_finite() as i32 != 0
        && (*input).maximum_sideslip_deg > 0.0f64;
    if sideslip_valid {
        if (*state).has_previous_sideslip as ::core::ffi::c_int != 0
            && (*input).dt > DBL_EPSILON
        {
            sideslip_rate = wrap_signed_deg(
                (*input).sideslip_deg - (*state).previous_sideslip_deg,
            ) / (*input).dt;
            let mut beta: ::core::ffi::c_double = fabs((*input).sideslip_deg);
            let mut beta_sign: ::core::ffi::c_double = if beta > DBL_EPSILON {
                unit_sign((*input).sideslip_deg)
            } else if fabs(sideslip_rate) > DBL_EPSILON {
                unit_sign(sideslip_rate)
            } else {
                1.0f64
            };
            let mut outward_beta_rate: ::core::ffi::c_double = beta_sign * sideslip_rate;
            if outward_beta_rate > 0.0f64 {
                let mut beta_margin: ::core::ffi::c_double = fmax(
                    0.0f64,
                    (*input).maximum_sideslip_deg - beta,
                );
                let mut accel: ::core::ffi::c_double = fabs(
                    (*limits).maximum_roll_accel_deg_s2,
                );
                let mut safe_rate: ::core::ffi::c_double = accel * beta_margin
                    / outward_beta_rate;
                coordination_rate = fmin(coordination_rate, fmax(0.0f64, safe_rate));
            }
        }
        (*state).previous_sideslip_deg = (*input).sideslip_deg;
        (*state).has_previous_sideslip = true_0 != 0;
    } else {
        (*state).has_previous_sideslip = false_0 != 0;
    }
    let mut decel_floor: ::core::ffi::c_double = fmax(
        0.0f64,
        fabs((*state).command_bank_rate_deg_s)
            - fabs((*limits).maximum_roll_accel_deg_s2) * (*input).dt,
    );
    coordination_rate = fmax(coordination_rate, decel_floor);
    command_limits.maximum_roll_rate_deg_s = fmax(DBL_EPSILON, coordination_rate);
    (*state).bank_sign = choose_bank_sign(state, input);
    let mut magnitude: ::core::ffi::c_double = 0.0f64;
    let mut authority_valid: bool = false;
    if (*input).has_bank_magnitude {
        authority_valid = (*input).bank_magnitude_deg.is_finite() as i32 != 0
            && (*input).lift_accel.is_finite() as i32 != 0
            && (*input).lift_accel > 0.0f64;
        magnitude = if authority_valid as ::core::ffi::c_int != 0 {
            clamp_value(
                (*input).bank_magnitude_deg,
                0.0f64,
                fabs((*limits).maximum_bank_deg),
            )
        } else {
            0.0f64
        };
        output.energy_bank_magnitude_deg = magnitude;
        let mut lateral: ::core::ffi::c_double = 0.0f64;
        if authority_valid as ::core::ffi::c_int != 0
            && lateral_demand(
                input,
                &raw mut lateral,
                &raw mut output.zero_effort_crossrange_m,
            ) as ::core::ffi::c_int != 0
        {
            output.required_lateral_lift_accel = lateral;
            let mut vertical: ::core::ffi::c_double = (*input).lift_accel
                * cos(magnitude * ENTRY_LATERAL_DEG2RAD);
            if hypot(vertical, lateral) > (*input).lift_accel {
                magnitude = clamp_value(
                    atan2(lateral, vertical) * ENTRY_LATERAL_RAD2DEG,
                    0.0f64,
                    fabs((*limits).maximum_bank_deg),
                );
                output.lateral_demand_binding = true_0 != 0;
            }
        }
    } else {
        authority_valid = bank_magnitude(
            input,
            limits,
            &raw mut magnitude,
            &raw mut output.energy_bank_magnitude_deg,
            &raw mut output.required_lateral_lift_accel,
            &raw mut output.zero_effort_crossrange_m,
            &raw mut output.lateral_demand_binding,
        );
    }
    if !authority_valid {
        magnitude = 0.0f64;
        output.energy_bank_magnitude_deg = 0.0f64;
        output.degraded_authority = true_0 != 0;
    }
    let mut reversal_span: ::core::ffi::c_double = fabs((*input).measured_bank_deg)
        + magnitude;
    let mut reversal_dwell: ::core::ffi::c_double = fmax(
        (*limits).minimum_leg_duration_s,
        rest_to_rest_roll_time(
            reversal_span,
            command_limits.maximum_roll_rate_deg_s,
            command_limits.maximum_roll_accel_deg_s2,
        ),
    );
    let mut reversal_ready: bool = (*input).ut - (*state).last_reversal_ut
        >= reversal_dwell;
    let mut reversal_requested: bool = false_0 != 0;
    if (*input).has_azimuth_error as ::core::ffi::c_int != 0
        && (*input).azimuth_error_deg.is_finite() as i32 != 0
    {
        let mut error: ::core::ffi::c_double = (*input).azimuth_error_deg;
        let mut deadband: ::core::ffi::c_double = fmax(
            0.0f64,
            finite_or((*input).azimuth_deadband_deg, 0.0f64),
        );
        output.corridor_metric = if deadband > 0.0f64 {
            error / deadband
        } else {
            ::core::f32::NAN as ::core::ffi::c_double
        };
        output.course_corridor_deg = deadband;
        let mut demanded_sign: ::core::ffi::c_double = unit_sign(error);
        let mut crossrange_projection_valid: bool = authority_valid as ::core::ffi::c_int
            != 0 && magnitude > DBL_EPSILON
            && (*input).has_crossrange_error as ::core::ffi::c_int != 0
            && (*input).crossrange_error_m.is_finite() as i32 != 0
            && (*input).has_crossrange_rate as ::core::ffi::c_int != 0
            && (*input).crossrange_error_rate_mps.is_finite() as i32 != 0
            && (*input).has_crossrange_course_alignment as ::core::ffi::c_int != 0
            && (*input).crossrange_course_alignment.is_finite() as i32 != 0
            && fabs((*input).crossrange_course_alignment) > DBL_EPSILON;
        let mut crossrange_demand: ::core::ffi::c_double = 0.0f64;
        if crossrange_projection_valid {
            let mut opposite_target: ::core::ffi::c_double = -(*state).bank_sign
                * magnitude;
            let mut roll_span: ::core::ffi::c_double = fabs(
                opposite_target - finite_or((*input).measured_bank_deg, 0.0f64),
            );
            let mut response_time: ::core::ffi::c_double = rest_to_rest_roll_time(
                roll_span,
                fabs(command_limits.maximum_roll_rate_deg_s),
                fabs(command_limits.maximum_roll_accel_deg_s2),
            );
            let mut projected: ::core::ffi::c_double = (*input).crossrange_error_m
                + (*input).crossrange_error_rate_mps * response_time;
            if (*input).has_crossrange_accel as ::core::ffi::c_int != 0
                && (*input).crossrange_error_accel_mps2.is_finite() as i32 != 0
            {
                projected
                    += 0.5f64 * (*input).crossrange_error_accel_mps2 * response_time
                        * response_time;
            }
            projected += crossrange_arrest(input, magnitude, response_time);
            let mut uncertainty: ::core::ffi::c_double = if (*input)
                .crossrange_uncertainty_m
                .is_finite() as i32 != 0
            {
                fmax(0.0f64, (*input).crossrange_uncertainty_m)
            } else {
                0.0f64
            };
            let mut numerical_resolution: ::core::ffi::c_double = sqrt(DBL_EPSILON)
                * fmax(1.0f64, fmax(fabs((*input).crossrange_error_m), fabs(projected)));
            let mut corridor: ::core::ffi::c_double = uncertainty + numerical_resolution;
            output.reversal_response_time_s = response_time;
            output.projected_crossrange_error_m = projected;
            output.crossrange_corridor_m = corridor;
            output.corridor_metric = projected / fmax(corridor, numerical_resolution);
            if fabs(projected) > corridor {
                crossrange_demand = unit_sign(projected)
                    * unit_sign((*input).crossrange_course_alignment);
            }
        }
        let mut point_target_behind: bool = cos(error * ENTRY_LATERAL_DEG2RAD) <= 0.0f64;
        if point_target_behind {
            demanded_sign = (*state).bank_sign;
        } else if output.lateral_demand_binding as ::core::ffi::c_int != 0
            && fabs(output.zero_effort_crossrange_m) > DBL_EPSILON
        {
            demanded_sign = unit_sign(output.zero_effort_crossrange_m);
            crossrange_demand = demanded_sign;
        } else if crossrange_demand != 0. && crossrange_demand * demanded_sign < 0.0f64 {
            demanded_sign = crossrange_demand;
        }
        let mut captured_or_small: bool = (*state).leg_captured as ::core::ffi::c_int
            != 0 || magnitude < 5.0f64;
        if authority_valid as ::core::ffi::c_int != 0
            && (fabs(error) > deadband || crossrange_demand != 0.0f64)
            && demanded_sign * (*state).bank_sign < 0.0f64
        {
            (*state).reversal_armed = true_0 != 0;
            if captured_or_small as ::core::ffi::c_int != 0
                && reversal_ready as ::core::ffi::c_int != 0
            {
                (*state).bank_sign = demanded_sign;
                (*state).leg_captured = false_0 != 0;
                (*state).leg_captured_ut = (*input).ut;
                (*state).last_reversal_ut = (*input).ut;
                (*state).reversal_armed = false_0 != 0;
                reversal_requested = true_0 != 0;
            }
        } else {
            (*state).reversal_armed = false_0 != 0;
        }
    } else if authority_valid as ::core::ffi::c_int != 0 && magnitude > DBL_EPSILON
        && (*input).has_crossrange_error as ::core::ffi::c_int != 0
        && (*input).crossrange_error_m.is_finite() as i32 != 0
    {
        let mut opposite_target_0: ::core::ffi::c_double = -(*state).bank_sign
            * magnitude;
        let mut roll_span_0: ::core::ffi::c_double = fabs(
            opposite_target_0 - finite_or((*input).measured_bank_deg, 0.0f64),
        );
        let mut response_time_0: ::core::ffi::c_double = rest_to_rest_roll_time(
            roll_span_0,
            fabs(command_limits.maximum_roll_rate_deg_s),
            fabs(command_limits.maximum_roll_accel_deg_s2),
        );
        let mut projected_0: ::core::ffi::c_double = (*input).crossrange_error_m;
        if (*input).has_crossrange_rate as ::core::ffi::c_int != 0
            && (*input).crossrange_error_rate_mps.is_finite() as i32 != 0
        {
            projected_0 += (*input).crossrange_error_rate_mps * response_time_0;
        }
        if (*input).has_crossrange_accel as ::core::ffi::c_int != 0
            && (*input).crossrange_error_accel_mps2.is_finite() as i32 != 0
        {
            projected_0
                += 0.5f64 * (*input).crossrange_error_accel_mps2 * response_time_0
                    * response_time_0;
        }
        projected_0 += crossrange_arrest(input, magnitude, response_time_0);
        let mut uncertainty_0: ::core::ffi::c_double = if (*input)
            .crossrange_uncertainty_m
            .is_finite() as i32 != 0
        {
            fmax(0.0f64, (*input).crossrange_uncertainty_m)
        } else {
            0.0f64
        };
        let mut numerical_resolution_0: ::core::ffi::c_double = sqrt(DBL_EPSILON)
            * fmax(1.0f64, fmax(fabs((*input).crossrange_error_m), fabs(projected_0)));
        let mut corridor_0: ::core::ffi::c_double = uncertainty_0
            + numerical_resolution_0;
        output.reversal_response_time_s = response_time_0;
        output.projected_crossrange_error_m = projected_0;
        output.crossrange_corridor_m = corridor_0;
        output.corridor_metric = projected_0 / fmax(corridor_0, numerical_resolution_0);
        if fabs(projected_0) > corridor_0 {
            let mut demanded_sign_0: ::core::ffi::c_double = unit_sign(projected_0);
            if demanded_sign_0 * (*state).bank_sign < 0.0f64 {
                (*state).reversal_armed = true_0 != 0;
                if (*state).leg_captured as ::core::ffi::c_int != 0
                    && reversal_ready as ::core::ffi::c_int != 0
                {
                    (*state).bank_sign = demanded_sign_0;
                    (*state).leg_captured = false_0 != 0;
                    (*state).leg_captured_ut = (*input).ut;
                    (*state).last_reversal_ut = (*input).ut;
                    (*state).reversal_armed = false_0 != 0;
                    reversal_requested = true_0 != 0;
                }
            } else {
                (*state).reversal_armed = false_0 != 0;
            }
        } else {
            (*state).reversal_armed = false_0 != 0;
        }
    }
    let mut raw_target: ::core::ffi::c_double = (*state).bank_sign * magnitude;
    update_bank_command(
        state,
        raw_target,
        &raw mut command_limits,
        (*input).dt,
        (*input).measured_bank_deg,
    );
    if !(*state).leg_captured && magnitude > DBL_EPSILON
        && (*input).measured_bank_deg.is_finite() as i32 != 0
        && (*state).bank_sign * (*input).measured_bank_deg >= 0.9f64 * magnitude
    {
        (*state).leg_captured = true_0 != 0;
        (*state).leg_captured_ut = (*input).ut;
    }
    let mut effectiveness: ::core::ffi::c_double = finite_or(
        (*input).bank_effectiveness,
        1.0f64,
    );
    let mut effective_bank: ::core::ffi::c_double = (*state).command_bank_deg
        * effectiveness;
    effective_bank = clamp_value(
        effective_bank,
        -nextafter(90.0f64, 0.0f64),
        nextafter(90.0f64, 0.0f64),
    );
    let mut lateral_accel: ::core::ffi::c_double = fmax(
        0.0f64,
        finite_or((*input).lift_accel, 0.0f64),
    ) * sin(effective_bank * ENTRY_LATERAL_DEG2RAD);
    output.valid = true_0 != 0;
    output.reversal_requested = reversal_requested;
    output.leg_captured = (*state).leg_captured;
    output.reversal_armed = (*state).reversal_armed;
    output.bank_sign = (*state).bank_sign;
    output.bank_magnitude_deg = magnitude;
    output.raw_target_bank_deg = raw_target;
    output.target_bank_deg = (*state).command_bank_deg;
    output.target_bank_rate_deg_s = (*state).command_bank_rate_deg_s;
    output.expected_course_rate_deg_s = lateral_accel / (*input).relative_speed
        * ENTRY_LATERAL_RAD2DEG;
    output.sideslip_rate_deg_s = sideslip_rate;
    output.coordination_roll_rate_limit_deg_s = command_limits.maximum_roll_rate_deg_s;
    return output;
}
unsafe extern "C" fn rest_to_rest_roll_time(
    mut span_deg: ::core::ffi::c_double,
    mut rate_limit_deg_s: ::core::ffi::c_double,
    mut accel_limit_deg_s2: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    span_deg = fabs(span_deg);
    if !(span_deg > DBL_EPSILON) {
        return 0.0f64;
    }
    let mut triangular_span: ::core::ffi::c_double = rate_limit_deg_s * rate_limit_deg_s
        / accel_limit_deg_s2;
    if span_deg <= triangular_span {
        return 2.0f64 * sqrt(span_deg / accel_limit_deg_s2);
    }
    return span_deg / rate_limit_deg_s + rate_limit_deg_s / accel_limit_deg_s2;
}
#[no_mangle]
pub unsafe extern "C" fn entry_lateral_terminal_turn_envelope(
    mut input: *const EntryLateralTurnInput,
) -> EntryLateralTurnEnvelope {
    let mut out: EntryLateralTurnEnvelope = EntryLateralTurnEnvelope {
        valid: false,
        turn_sign: 0.,
        response_time_s: 0.,
        response_distance_m: 0.,
        response_heading_change_deg: 0.,
        projected_along_m: 0.,
        projected_cross_m: 0.,
        projected_course_deg: 0.,
        remaining_course_change_deg: 0.,
        minimum_turn_radius_m: 0.,
        ideal_turn_radius_m: 0.,
        radius_margin_m: 0.,
        endpoint_error_m: 0.,
        capture_margin_m: 0.,
        feasibility_margin_m: 0.,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryLateralTurnEnvelope>() as size_t,
    );
    out.response_time_s = ::core::f32::NAN as ::core::ffi::c_double;
    out.response_distance_m = ::core::f32::NAN as ::core::ffi::c_double;
    out.response_heading_change_deg = ::core::f32::NAN as ::core::ffi::c_double;
    out.projected_along_m = ::core::f32::NAN as ::core::ffi::c_double;
    out.projected_cross_m = ::core::f32::NAN as ::core::ffi::c_double;
    out.projected_course_deg = ::core::f32::NAN as ::core::ffi::c_double;
    out.remaining_course_change_deg = ::core::f32::NAN as ::core::ffi::c_double;
    out.minimum_turn_radius_m = ::core::f32::INFINITY as ::core::ffi::c_double;
    out.ideal_turn_radius_m = ::core::f32::INFINITY as ::core::ffi::c_double;
    out.radius_margin_m = -::core::f32::INFINITY as ::core::ffi::c_double;
    out.endpoint_error_m = ::core::f32::INFINITY as ::core::ffi::c_double;
    out.capture_margin_m = -::core::f32::INFINITY as ::core::ffi::c_double;
    out.feasibility_margin_m = -::core::f32::INFINITY as ::core::ffi::c_double;
    if input.is_null() || (*input).horizontal_speed_mps.is_finite() as i32 == 0
        || !((*input).horizontal_speed_mps > DBL_EPSILON)
        || (*input).current_course_deg.is_finite() as i32 == 0
        || (*input).target_course_deg.is_finite() as i32 == 0
        || (*input).current_along_m.is_finite() as i32 == 0
        || (*input).current_cross_m.is_finite() as i32 == 0
        || (*input).target_along_m.is_finite() as i32 == 0
        || (*input).target_cross_m.is_finite() as i32 == 0
        || (*input).measured_bank_deg.is_finite() as i32 == 0
        || (*input).measured_bank_rate_deg_s.is_finite() as i32 == 0
        || (*input).lift_accel_mps2.is_finite() as i32 == 0
        || !((*input).lift_accel_mps2 > DBL_EPSILON)
        || (*input).bank_effectiveness.is_finite() as i32 == 0
        || !((*input).bank_effectiveness > DBL_EPSILON)
        || (*input).maximum_bank_deg.is_finite() as i32 == 0
        || !((*input).maximum_bank_deg > DBL_EPSILON)
        || (*input).maximum_roll_rate_deg_s.is_finite() as i32 == 0
        || !((*input).maximum_roll_rate_deg_s > DBL_EPSILON)
        || (*input).maximum_roll_accel_deg_s2.is_finite() as i32 == 0
        || !((*input).maximum_roll_accel_deg_s2 > DBL_EPSILON)
        || (*input).capture_radius_m.is_finite() as i32 == 0
        || (*input).capture_radius_m < 0.0f64
        || (*input).position_uncertainty_m.is_finite() as i32 == 0
        || (*input).position_uncertainty_m < 0.0f64
    {
        return out;
    }
    let mut initial_turn: ::core::ffi::c_double = wrap_signed_deg(
        (*input).target_course_deg - (*input).current_course_deg,
    );
    let mut angular_resolution: ::core::ffi::c_double = ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG;
    if !(fabs(initial_turn) > angular_resolution) {
        return out;
    }
    let mut side: ::core::ffi::c_double = unit_sign(initial_turn);
    let mut bank_limit: ::core::ffi::c_double = fabs((*input).maximum_bank_deg);
    let mut target_bank: ::core::ffi::c_double = side * bank_limit;
    let mut rate_limit: ::core::ffi::c_double = fabs((*input).maximum_roll_rate_deg_s);
    let mut accel_limit: ::core::ffi::c_double = fabs(
        (*input).maximum_roll_accel_deg_s2,
    );
    let mut initial_rate: ::core::ffi::c_double = clamp_value(
        (*input).measured_bank_rate_deg_s,
        -rate_limit,
        rate_limit,
    );
    let mut stop_time: ::core::ffi::c_double = fabs(initial_rate) / accel_limit;
    let mut bank_after_stop: ::core::ffi::c_double = (*input).measured_bank_deg
        + 0.5f64 * initial_rate * stop_time;
    let mut response_time_bound: ::core::ffi::c_double = stop_time
        + rest_to_rest_roll_time(target_bank - bank_after_stop, rate_limit, accel_limit);
    let mut bank: ::core::ffi::c_double = clamp_value(
        (*input).measured_bank_deg,
        -bank_limit,
        bank_limit,
    );
    let mut bank_rate: ::core::ffi::c_double = initial_rate;
    let mut course_rad: ::core::ffi::c_double = (*input).current_course_deg
        * ENTRY_LATERAL_DEG2RAD;
    let mut along: ::core::ffi::c_double = (*input).current_along_m;
    let mut cross: ::core::ffi::c_double = (*input).current_cross_m;
    let mut elapsed: ::core::ffi::c_double = 0.0f64;
    let mut dt_nominal: ::core::ffi::c_double = ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG
        / rate_limit;
    if !(dt_nominal > 0.0f64) || response_time_bound.is_finite() as i32 == 0 {
        return out;
    }
    let mut max_steps: size_t = (ceil(response_time_bound / dt_nominal) as size_t)
        .wrapping_add(1 as size_t);
    let mut step: size_t = 0 as size_t;
    while step < max_steps && elapsed < response_time_bound {
        let mut remaining_time: ::core::ffi::c_double = response_time_bound - elapsed;
        let mut dt: ::core::ffi::c_double = fmin(dt_nominal, remaining_time);
        if !(dt > 0.0f64) {
            break;
        }
        let mut error: ::core::ffi::c_double = target_bank - bank;
        let mut desired_rate: ::core::ffi::c_double = 0.0f64;
        if fabs(error) > DBL_EPSILON {
            let mut stopping_rate: ::core::ffi::c_double = sqrt(
                2.0f64 * accel_limit * fabs(error),
            );
            desired_rate = unit_sign(error) * fmin(rate_limit, stopping_rate);
        }
        bank_rate
            += clamp_value(
                desired_rate - bank_rate,
                -accel_limit * dt,
                accel_limit * dt,
            );
        bank_rate = clamp_value(bank_rate, -rate_limit, rate_limit);
        let mut next_bank: ::core::ffi::c_double = bank + bank_rate * dt;
        if (target_bank - bank) * (target_bank - next_bank) <= 0.0f64 {
            next_bank = target_bank;
            bank_rate = 0.0f64;
        }
        next_bank = clamp_value(next_bank, -bank_limit, bank_limit);
        let mut average_bank: ::core::ffi::c_double = 0.5f64 * (bank + next_bank);
        let mut effective_bank: ::core::ffi::c_double = clamp_value(
            average_bank * (*input).bank_effectiveness,
            -nextafter(90.0f64, 0.0f64),
            nextafter(90.0f64, 0.0f64),
        );
        let mut lateral_accel: ::core::ffi::c_double = (*input).lift_accel_mps2
            * sin(effective_bank * ENTRY_LATERAL_DEG2RAD);
        let mut course_rate: ::core::ffi::c_double = lateral_accel
            / (*input).horizontal_speed_mps;
        let mut midpoint_course: ::core::ffi::c_double = course_rad
            + 0.5f64 * course_rate * dt;
        along += (*input).horizontal_speed_mps * cos(midpoint_course) * dt;
        cross += (*input).horizontal_speed_mps * sin(midpoint_course) * dt;
        course_rad += course_rate * dt;
        bank = next_bank;
        elapsed += dt;
        if bank == target_bank && fabs(bank_rate) <= DBL_EPSILON {
            break;
        }
        step = step.wrapping_add(1);
    }
    if fabs(target_bank - bank) > ENTRY_LATERAL_RESPONSE_BANK_STEP_DEG {
        return out;
    }
    let mut projected_course: ::core::ffi::c_double = wrap_signed_deg(
        course_rad * ENTRY_LATERAL_RAD2DEG,
    );
    let mut remaining_turn: ::core::ffi::c_double = wrap_signed_deg(
        (*input).target_course_deg - projected_course,
    );
    if side * remaining_turn <= angular_resolution {
        return out;
    }
    let mut effective_target_bank: ::core::ffi::c_double = fmin(
        bank_limit * (*input).bank_effectiveness,
        nextafter(90.0f64, 0.0f64),
    );
    let mut lateral_authority: ::core::ffi::c_double = (*input).lift_accel_mps2
        * fabs(sin(effective_target_bank * ENTRY_LATERAL_DEG2RAD));
    if !(lateral_authority > DBL_EPSILON) {
        return out;
    }
    let mut minimum_radius: ::core::ffi::c_double = (*input).horizontal_speed_mps
        * (*input).horizontal_speed_mps / lateral_authority;
    let mut psi0: ::core::ffi::c_double = projected_course * ENTRY_LATERAL_DEG2RAD;
    let mut psi1: ::core::ffi::c_double = (*input).target_course_deg
        * ENTRY_LATERAL_DEG2RAD;
    let mut ua: ::core::ffi::c_double = (sin(psi1) - sin(psi0)) / side;
    let mut uc: ::core::ffi::c_double = (cos(psi0) - cos(psi1)) / side;
    let mut norm2: ::core::ffi::c_double = ua * ua + uc * uc;
    if !(norm2 > DBL_EPSILON) || norm2.is_finite() as i32 == 0 {
        return out;
    }
    let mut da: ::core::ffi::c_double = (*input).target_along_m - along;
    let mut dc: ::core::ffi::c_double = (*input).target_cross_m - cross;
    let mut ideal_radius: ::core::ffi::c_double = (da * ua + dc * uc) / norm2;
    if ideal_radius.is_finite() as i32 == 0 {
        return out;
    }
    let mut endpoint_along: ::core::ffi::c_double = along + ideal_radius * ua;
    let mut endpoint_cross: ::core::ffi::c_double = cross + ideal_radius * uc;
    let mut endpoint_error: ::core::ffi::c_double = hypot(
        endpoint_along - (*input).target_along_m,
        endpoint_cross - (*input).target_cross_m,
    );
    let mut radius_margin: ::core::ffi::c_double = ideal_radius - minimum_radius;
    let mut capture_margin: ::core::ffi::c_double = (*input).capture_radius_m
        - (*input).position_uncertainty_m - endpoint_error;
    out.valid = true_0 != 0;
    out.turn_sign = side;
    out.response_time_s = elapsed;
    out.response_distance_m = (*input).horizontal_speed_mps * elapsed;
    out.response_heading_change_deg = wrap_signed_deg(
        projected_course - (*input).current_course_deg,
    );
    out.projected_along_m = along;
    out.projected_cross_m = cross;
    out.projected_course_deg = projected_course;
    out.remaining_course_change_deg = remaining_turn;
    out.minimum_turn_radius_m = minimum_radius;
    out.ideal_turn_radius_m = ideal_radius;
    out.radius_margin_m = radius_margin;
    out.endpoint_error_m = endpoint_error;
    out.capture_margin_m = capture_margin;
    out.feasibility_margin_m = fmin(radius_margin, capture_margin);
    return out;
}
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
