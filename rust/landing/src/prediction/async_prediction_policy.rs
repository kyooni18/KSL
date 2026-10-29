use crate::sys::*;
extern "C" {
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
}
pub type __uint64_t = u64;
pub type uint64_t = __uint64_t;
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
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn prediction_interval(
    mut configured_interval: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    return if configured_interval.is_finite() as i32 != 0 {
        fmax(configured_interval, 2.0f64)
    } else {
        2.0f64
    };
}
#[no_mangle]
pub unsafe extern "C" fn async_prediction_cooldown_seconds(
    mut configured_interval: ::core::ffi::c_double,
    mut previous_solve_wall_seconds: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut interval: ::core::ffi::c_double = prediction_interval(configured_interval);
    let mut solve: ::core::ffi::c_double = if previous_solve_wall_seconds.is_finite()
        as i32 != 0
    {
        fmax(0.0f64, previous_solve_wall_seconds)
    } else {
        0.0f64
    };
    return fmax(interval, fmin(solve, 8.0f64));
}
#[no_mangle]
pub unsafe extern "C" fn async_prediction_request_due(
    mut latest_ut: ::core::ffi::c_double,
    mut last_request_ut: ::core::ffi::c_double,
    mut now_wall: ::core::ffi::c_double,
    mut last_completion_wall: ::core::ffi::c_double,
    mut previous_solve_wall_seconds: ::core::ffi::c_double,
    mut configured_interval: ::core::ffi::c_double,
) -> bool {
    if latest_ut.is_finite() as i32 == 0 || now_wall.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    let mut interval: ::core::ffi::c_double = prediction_interval(configured_interval);
    let mut since_request: ::core::ffi::c_double = if last_request_ut.is_finite() as i32
        != 0
    {
        latest_ut - last_request_ut
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    let mut since_completion: ::core::ffi::c_double = if last_completion_wall.is_finite()
        as i32 != 0
    {
        now_wall - last_completion_wall
    } else {
        ::core::f32::INFINITY as ::core::ffi::c_double
    };
    return since_request + 1e-9f64 >= interval
        && since_completion + 1e-9f64
            >= async_prediction_cooldown_seconds(
                configured_interval,
                previous_solve_wall_seconds,
            );
}
#[no_mangle]
pub unsafe extern "C" fn async_prediction_identity_equal(
    mut a: *const AsyncPredictionIdentity,
    mut b: *const AsyncPredictionIdentity,
) -> bool {
    if a.is_null() || b.is_null() {
        return false_0 != 0;
    }
    return (*a).guidance_phase == (*b).guidance_phase
        && (*a).taem_phase == (*b).taem_phase
        && (*a).taem_transition_count == (*b).taem_transition_count
        && (*a).control_plan_sequence == (*b).control_plan_sequence
        && (*a).plan_id == (*b).plan_id && (*a).plan_version == (*b).plan_version
        && (*a).terminal_path_kind == (*b).terminal_path_kind
        && (*a).terminal_path_committed as ::core::ffi::c_int
            == (*b).terminal_path_committed as ::core::ffi::c_int
        && (*a).hac_side_selected as ::core::ffi::c_int
            == (*b).hac_side_selected as ::core::ffi::c_int
        && (*a).hac_captured as ::core::ffi::c_int
            == (*b).hac_captured as ::core::ffi::c_int
        && (*a).final_approach_captured as ::core::ffi::c_int
            == (*b).final_approach_captured as ::core::ffi::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn async_prediction_result_fresh(
    mut request_identity: *const AsyncPredictionIdentity,
    mut current_identity: *const AsyncPredictionIdentity,
    mut request_ut: ::core::ffi::c_double,
    mut latest_ut: ::core::ffi::c_double,
    mut configured_interval: ::core::ffi::c_double,
) -> bool {
    if !async_prediction_identity_equal(request_identity, current_identity)
        || request_ut.is_finite() as i32 == 0 || latest_ut.is_finite() as i32 == 0
        || latest_ut + 1e-6f64 < request_ut
    {
        return false_0 != 0;
    }
    let mut maximum_age: ::core::ffi::c_double = fmax(
        8.0f64,
        prediction_interval(configured_interval) * 4.0f64,
    );
    return latest_ut - request_ut <= maximum_age + 1e-6f64;
}
