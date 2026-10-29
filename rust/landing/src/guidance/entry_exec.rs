use crate::sys::*;
pub type EntryPhase = ::core::ffi::c_uint;
pub const ENTRY_PHASE_TRANSITION: EntryPhase = 5;
pub const ENTRY_PHASE_CONSTANT_DRAG: EntryPhase = 4;
pub const ENTRY_PHASE_EQUILIBRIUM_GLIDE: EntryPhase = 3;
pub const ENTRY_PHASE_TEMPERATURE_CONTROL: EntryPhase = 2;
pub const ENTRY_PHASE_PREENTRY: EntryPhase = 1;
pub type EntryTransitionReason = ::core::ffi::c_uint;
pub const ENTRY_TRANSITION_TAEM_FORCED_HANDOFF: EntryTransitionReason = 8;
pub const ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF: EntryTransitionReason = 7;
pub const ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY: EntryTransitionReason = 6;
pub const ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT: EntryTransitionReason = 5;
pub const ENTRY_TRANSITION_TEMPERATURE_VELOCITY: EntryTransitionReason = 4;
pub const ENTRY_TRANSITION_ATMOSPHERIC_CONTACT: EntryTransitionReason = 3;
pub const ENTRY_TRANSITION_RESTART_CLASSIFICATION: EntryTransitionReason = 2;
pub const ENTRY_TRANSITION_ENTRY_START: EntryTransitionReason = 1;
pub const ENTRY_TRANSITION_NONE: EntryTransitionReason = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryExecObservation {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub checkpoint_restart: bool,
}
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
pub struct EntryExecutive {
    pub initialized: bool,
    pub entry_complete: bool,
    pub phase: EntryPhase,
    pub last_transition_reason: EntryTransitionReason,
    pub last_transition_from: EntryPhase,
    pub last_transition_to: EntryPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_entry: bool,
    pub transition_count: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EntryExecTelemetry {
    pub initialized: bool,
    pub entry_complete: bool,
    pub active_phase: EntryPhase,
    pub last_transition_reason: EntryTransitionReason,
    pub last_transition_from: EntryPhase,
    pub last_transition_to: EntryPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_entry: bool,
    pub transition_count: ::core::ffi::c_uint,
}
pub type size_t = usize;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn phase_valid(mut phase: EntryPhase) -> bool {
    return phase as ::core::ffi::c_uint
        >= ENTRY_PHASE_PREENTRY as ::core::ffi::c_int as ::core::ffi::c_uint
        && phase as ::core::ffi::c_uint
            <= ENTRY_PHASE_TRANSITION as ::core::ffi::c_int as ::core::ffi::c_uint;
}
unsafe extern "C" fn observation_valid(
    mut observation: *const EntryExecObservation,
) -> bool {
    return !observation.is_null() && (*observation).ut.is_finite() as i32 != 0
        && (*observation).relative_velocity.is_finite() as i32 != 0
        && (*observation).relative_velocity >= 0.0f64
        && (*observation).altitude.is_finite() as i32 != 0
        && (*observation).dynamic_pressure.is_finite() as i32 != 0
        && (*observation).dynamic_pressure >= 0.0f64;
}
unsafe extern "C" fn profile_valid(mut profile: *const EntryExecProfile) -> bool {
    if profile.is_null() {
        return false_0 != 0;
    }
    if (*profile).has_temperature_velocity_gate as ::core::ffi::c_int != 0
        && ((*profile).temperature_end_velocity.is_finite() as i32 == 0
            || (*profile).temperature_end_velocity < 0.0f64)
    {
        return false_0 != 0;
    }
    if (*profile).has_constant_drag_velocity_gate as ::core::ffi::c_int != 0
        && ((*profile).constant_drag_end_velocity.is_finite() as i32 == 0
            || (*profile).constant_drag_end_velocity < 0.0f64)
    {
        return false_0 != 0;
    }
    if (*profile).has_phase_floor as ::core::ffi::c_int != 0
        && !phase_valid((*profile).phase_floor)
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn max_phase(mut a: EntryPhase, mut b: EntryPhase) -> EntryPhase {
    return (if a as ::core::ffi::c_uint > b as ::core::ffi::c_uint {
        a as ::core::ffi::c_uint
    } else {
        b as ::core::ffi::c_uint
    }) as EntryPhase;
}
unsafe extern "C" fn restart_phase(
    mut observation: *const EntryExecObservation,
    mut profile: *const EntryExecProfile,
) -> EntryPhase {
    let mut phase: EntryPhase = ENTRY_PHASE_PREENTRY;
    if (*profile).has_phase_floor {
        phase = max_phase(phase, (*profile).phase_floor);
    }
    if (*observation).dynamic_pressure > 0.0f64 {
        phase = max_phase(phase, ENTRY_PHASE_TEMPERATURE_CONTROL);
    }
    if (*profile).has_temperature_velocity_gate as ::core::ffi::c_int != 0
        && (*observation).relative_velocity <= (*profile).temperature_end_velocity
    {
        phase = max_phase(phase, ENTRY_PHASE_EQUILIBRIUM_GLIDE);
    }
    if (*profile).equilibrium_intercept_valid as ::core::ffi::c_int != 0
        && (*profile).equilibrium_intercept as ::core::ffi::c_int != 0
    {
        phase = max_phase(phase, ENTRY_PHASE_CONSTANT_DRAG);
    }
    if (*profile).has_constant_drag_velocity_gate as ::core::ffi::c_int != 0
        && (*observation).relative_velocity <= (*profile).constant_drag_end_velocity
    {
        phase = max_phase(phase, ENTRY_PHASE_TRANSITION);
    }
    return phase;
}
unsafe extern "C" fn record_transition(
    mut exec: *mut EntryExecutive,
    mut from: EntryPhase,
    mut to: EntryPhase,
    mut reason: EntryTransitionReason,
    mut observation: *const EntryExecObservation,
    mut completes_entry: bool,
) {
    (*exec).phase = to;
    (*exec).entry_complete = (*exec).entry_complete as ::core::ffi::c_int != 0
        || completes_entry as ::core::ffi::c_int != 0;
    (*exec).last_transition_reason = reason;
    (*exec).last_transition_from = from;
    (*exec).last_transition_to = to;
    (*exec).last_transition_ut = (*observation).ut;
    (*exec).last_transition_velocity = (*observation).relative_velocity;
    (*exec).last_transition_completed_entry = completes_entry;
    (*exec).transition_count = (*exec).transition_count.wrapping_add(1);
}
#[no_mangle]
pub unsafe extern "C" fn entry_exec_reset(mut exec: *mut EntryExecutive) {
    if exec.is_null() {
        return;
    }
    memset(
        exec as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryExecutive>() as size_t,
    );
    (*exec).phase = ENTRY_PHASE_PREENTRY;
    (*exec).last_transition_from = ENTRY_PHASE_PREENTRY;
    (*exec).last_transition_to = ENTRY_PHASE_PREENTRY;
    (*exec).last_transition_reason = ENTRY_TRANSITION_NONE;
    (*exec).last_transition_ut = ::core::f32::NAN as ::core::ffi::c_double;
    (*exec).last_transition_velocity = ::core::f32::NAN as ::core::ffi::c_double;
}
#[no_mangle]
pub unsafe extern "C" fn entry_exec_initialize(
    mut exec: *mut EntryExecutive,
    mut observation: *const EntryExecObservation,
    mut profile: *const EntryExecProfile,
) -> bool {
    if exec.is_null() || !observation_valid(observation) || !profile_valid(profile) {
        return false_0 != 0;
    }
    let mut next: EntryExecutive = EntryExecutive {
        initialized: false,
        entry_complete: false,
        phase: 0 as EntryPhase,
        last_transition_reason: ENTRY_TRANSITION_NONE,
        last_transition_from: 0 as EntryPhase,
        last_transition_to: 0 as EntryPhase,
        last_transition_ut: 0.,
        last_transition_velocity: 0.,
        last_transition_completed_entry: false,
        transition_count: 0,
    };
    entry_exec_reset(&raw mut next);
    next.initialized = true_0 != 0;
    if (*observation).checkpoint_restart {
        next.phase = restart_phase(observation, profile);
        next.entry_complete = false_0 != 0;
        next.last_transition_reason = ENTRY_TRANSITION_RESTART_CLASSIFICATION;
    } else {
        next.phase = ENTRY_PHASE_PREENTRY;
        next.entry_complete = false_0 != 0;
        next.last_transition_reason = ENTRY_TRANSITION_ENTRY_START;
    }
    next.last_transition_from = next.phase;
    next.last_transition_to = next.phase;
    next.last_transition_ut = (*observation).ut;
    next.last_transition_velocity = (*observation).relative_velocity;
    next.last_transition_completed_entry = next.entry_complete;
    *exec = next;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn entry_exec_update(
    mut exec: *mut EntryExecutive,
    mut observation: *const EntryExecObservation,
    mut profile: *const EntryExecProfile,
) -> bool {
    if exec.is_null() || !observation_valid(observation) || !profile_valid(profile) {
        return false_0 != 0;
    }
    if !(*exec).initialized {
        return entry_exec_initialize(exec, observation, profile);
    }
    if !phase_valid((*exec).phase) {
        return false_0 != 0;
    }
    if (*exec).entry_complete {
        return true_0 != 0;
    }
    let mut from: EntryPhase = (*exec).phase;
    match (*exec).phase as ::core::ffi::c_uint {
        1 => {
            if (*observation).dynamic_pressure > 0.0f64 {
                record_transition(
                    exec,
                    from,
                    ENTRY_PHASE_TEMPERATURE_CONTROL,
                    ENTRY_TRANSITION_ATMOSPHERIC_CONTACT,
                    observation,
                    false_0 != 0,
                );
            }
        }
        2 => {
            if (*profile).has_temperature_velocity_gate as ::core::ffi::c_int != 0
                && (*observation).relative_velocity
                    <= (*profile).temperature_end_velocity
            {
                record_transition(
                    exec,
                    from,
                    ENTRY_PHASE_EQUILIBRIUM_GLIDE,
                    ENTRY_TRANSITION_TEMPERATURE_VELOCITY,
                    observation,
                    false_0 != 0,
                );
            }
        }
        3 => {
            if (*profile).equilibrium_intercept_valid as ::core::ffi::c_int != 0
                && (*profile).equilibrium_intercept as ::core::ffi::c_int != 0
            {
                record_transition(
                    exec,
                    from,
                    ENTRY_PHASE_CONSTANT_DRAG,
                    ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT,
                    observation,
                    false_0 != 0,
                );
            }
        }
        4 => {
            if (*profile).has_constant_drag_velocity_gate as ::core::ffi::c_int != 0
                && (*observation).relative_velocity
                    <= (*profile).constant_drag_end_velocity
            {
                record_transition(
                    exec,
                    from,
                    ENTRY_PHASE_TRANSITION,
                    ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY,
                    observation,
                    false_0 != 0,
                );
            }
        }
        5 => {}
        _ => return false_0 != 0,
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn entry_exec_accept_taem_handoff(
    mut exec: *mut EntryExecutive,
    mut observation: *const EntryExecObservation,
) -> bool {
    if exec.is_null() || !observation_valid(observation) || !(*exec).initialized
        || (*exec).phase as ::core::ffi::c_uint
            != ENTRY_PHASE_TRANSITION as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return false_0 != 0;
    }
    if (*exec).entry_complete {
        return true_0 != 0;
    }
    record_transition(
        exec,
        (*exec).phase,
        (*exec).phase,
        ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF,
        observation,
        true_0 != 0,
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn entry_exec_force_taem_handoff(
    mut exec: *mut EntryExecutive,
    mut observation: *const EntryExecObservation,
) -> bool {
    if exec.is_null() || !observation_valid(observation)
        || (*exec).initialized as ::core::ffi::c_int != 0 && !phase_valid((*exec).phase)
    {
        return false_0 != 0;
    }
    if !(*exec).initialized {
        entry_exec_reset(exec);
        (*exec).initialized = true_0 != 0;
    }
    if (*exec).entry_complete {
        return true_0 != 0;
    }
    record_transition(
        exec,
        (*exec).phase,
        ENTRY_PHASE_TRANSITION,
        ENTRY_TRANSITION_TAEM_FORCED_HANDOFF,
        observation,
        true_0 != 0,
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn entry_exec_telemetry(
    mut exec: *const EntryExecutive,
) -> EntryExecTelemetry {
    let mut telemetry: EntryExecTelemetry = EntryExecTelemetry {
        initialized: false,
        entry_complete: false,
        active_phase: 0 as EntryPhase,
        last_transition_reason: ENTRY_TRANSITION_NONE,
        last_transition_from: 0 as EntryPhase,
        last_transition_to: 0 as EntryPhase,
        last_transition_ut: 0.,
        last_transition_velocity: 0.,
        last_transition_completed_entry: false,
        transition_count: 0,
    };
    memset(
        &raw mut telemetry as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<EntryExecTelemetry>() as size_t,
    );
    telemetry.active_phase = ENTRY_PHASE_PREENTRY;
    telemetry.last_transition_from = ENTRY_PHASE_PREENTRY;
    telemetry.last_transition_to = ENTRY_PHASE_PREENTRY;
    telemetry.last_transition_ut = ::core::f32::NAN as ::core::ffi::c_double;
    telemetry.last_transition_velocity = ::core::f32::NAN as ::core::ffi::c_double;
    if exec.is_null() {
        return telemetry;
    }
    telemetry.initialized = (*exec).initialized;
    telemetry.entry_complete = (*exec).entry_complete;
    telemetry.active_phase = (*exec).phase;
    telemetry.last_transition_reason = (*exec).last_transition_reason;
    telemetry.last_transition_from = (*exec).last_transition_from;
    telemetry.last_transition_to = (*exec).last_transition_to;
    telemetry.last_transition_ut = (*exec).last_transition_ut;
    telemetry.last_transition_velocity = (*exec).last_transition_velocity;
    telemetry.last_transition_completed_entry = (*exec).last_transition_completed_entry;
    telemetry.transition_count = (*exec).transition_count;
    return telemetry;
}
#[no_mangle]
pub unsafe extern "C" fn entry_phase_string(
    mut phase: EntryPhase,
) -> *const ::core::ffi::c_char {
    match phase as ::core::ffi::c_uint {
        1 => return b"Pre-entry\0" as *const u8 as *const ::core::ffi::c_char,
        2 => return b"Temperature Control\0" as *const u8 as *const ::core::ffi::c_char,
        3 => return b"Equilibrium Glide\0" as *const u8 as *const ::core::ffi::c_char,
        4 => return b"Constant Drag\0" as *const u8 as *const ::core::ffi::c_char,
        5 => return b"Transition\0" as *const u8 as *const ::core::ffi::c_char,
        _ => return b"Unknown Entry Phase\0" as *const u8 as *const ::core::ffi::c_char,
    };
}
#[no_mangle]
pub unsafe extern "C" fn entry_transition_reason_string(
    mut reason: EntryTransitionReason,
) -> *const ::core::ffi::c_char {
    match reason as ::core::ffi::c_uint {
        0 => return b"None\0" as *const u8 as *const ::core::ffi::c_char,
        1 => return b"Entry start\0" as *const u8 as *const ::core::ffi::c_char,
        2 => {
            return b"Checkpoint/restart classification\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        3 => return b"Atmospheric contact\0" as *const u8 as *const ::core::ffi::c_char,
        4 => {
            return b"Temperature-control velocity gate\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        5 => {
            return b"Equilibrium/constant-drag profile intersection\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        6 => {
            return b"Constant-drag transition velocity gate\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        7 => {
            return b"Qualified TAEM handoff\0" as *const u8 as *const ::core::ffi::c_char;
        }
        8 => {
            return b"MM305 physical boundary handoff\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        _ => {
            return b"Unknown Entry transition\0" as *const u8
                as *const ::core::ffi::c_char;
        }
    };
}
