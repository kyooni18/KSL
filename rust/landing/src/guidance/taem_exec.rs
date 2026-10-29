use crate::sys::*;
pub type TaemPhase = ::core::ffi::c_uint;
pub const TAEM_PHASE_FINAL_INTERCEPT: TaemPhase = 2;
pub const TAEM_PHASE_RUNWAY_ALIGNMENT: TaemPhase = 1;
pub const TAEM_PHASE_PATH_ACQUISITION: TaemPhase = 0;
pub type TaemTransitionReason = ::core::ffi::c_uint;
pub const TAEM_TRANSITION_LOW_ENERGY_DELIVERY: TaemTransitionReason = 7;
pub const TAEM_TRANSITION_TERMINAL_PATH_REPLAN: TaemTransitionReason = 6;
pub const TAEM_TRANSITION_FINAL_APPROACH_DELIVERY: TaemTransitionReason = 5;
pub const TAEM_TRANSITION_FINAL_INTERCEPT_GATE: TaemTransitionReason = 4;
pub const TAEM_TRANSITION_TERMINAL_PATH_CAPTURE: TaemTransitionReason = 3;
pub const TAEM_TRANSITION_RESTART_CLASSIFICATION: TaemTransitionReason = 2;
pub const TAEM_TRANSITION_MM304_HANDOFF: TaemTransitionReason = 1;
pub const TAEM_TRANSITION_NONE: TaemTransitionReason = 0;
pub type TaemRecoveryReason = ::core::ffi::c_uint;
pub const TAEM_RECOVERY_ENERGY_INFEASIBLE: TaemRecoveryReason = 4;
pub const TAEM_RECOVERY_CAPTURE_LOSS: TaemRecoveryReason = 3;
pub const TAEM_RECOVERY_PATH_REPLAN: TaemRecoveryReason = 2;
pub const TAEM_RECOVERY_ATTITUDE: TaemRecoveryReason = 1;
pub const TAEM_RECOVERY_NONE: TaemRecoveryReason = 0;
pub type TaemTerminalBlockReason = ::core::ffi::c_uint;
pub const TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE: TaemTerminalBlockReason = 10;
pub const TAEM_TERMINAL_BLOCK_FINITE_RESPONSE: TaemTerminalBlockReason = 9;
pub const TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY: TaemTerminalBlockReason = 8;
pub const TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE: TaemTerminalBlockReason = 7;
pub const TAEM_TERMINAL_BLOCK_ALTITUDE: TaemTerminalBlockReason = 6;
pub const TAEM_TERMINAL_BLOCK_SPEEDBRAKE: TaemTerminalBlockReason = 5;
pub const TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE: TaemTerminalBlockReason = 4;
pub const TAEM_TERMINAL_BLOCK_RANGE: TaemTerminalBlockReason = 3;
pub const TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED: TaemTerminalBlockReason = 2;
pub const TAEM_TERMINAL_BLOCK_CONTRACT_INVALID: TaemTerminalBlockReason = 1;
pub const TAEM_TERMINAL_BLOCK_NONE: TaemTerminalBlockReason = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTerminalContract {
    pub valid: bool,
    pub path_committed: bool,
    pub range_to_go: ::core::ffi::c_double,
    pub range_margin: ::core::ffi::c_double,
    pub dynamic_pressure: ::core::ffi::c_double,
    pub dynamic_pressure_margin: ::core::ffi::c_double,
    pub speedbrake_required: bool,
    pub speedbrake_available: bool,
    pub speedbrake_dynamic_pressure_margin: ::core::ffi::c_double,
    pub altitude: ::core::ffi::c_double,
    pub altitude_margin: ::core::ffi::c_double,
    pub flight_path_angle: ::core::ffi::c_double,
    pub flight_path_angle_margin: ::core::ffi::c_double,
    pub specific_energy: ::core::ffi::c_double,
    pub specific_energy_margin: ::core::ffi::c_double,
    pub response_time_available: ::core::ffi::c_double,
    pub response_time_required: ::core::ffi::c_double,
    pub attitude_response_qualified: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemTerminalEvaluation {
    pub valid: bool,
    pub feasible: bool,
    pub block_reason: TaemTerminalBlockReason,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemExecObservation {
    pub ut: ::core::ffi::c_double,
    pub relative_velocity: ::core::ffi::c_double,
    pub checkpoint_restart: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemExecInputs {
    pub mm304_complete: bool,
    pub resume_mm305: bool,
    pub terminal_path_selected: bool,
    pub terminal_path_captured: bool,
    pub final_intercept_ready: bool,
    pub final_approach_ready: bool,
    pub terminal_contract: TaemTerminalContract,
    pub off_nominal_recovery_active: bool,
    pub off_nominal_recovery_reason: TaemRecoveryReason,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemExecutive {
    pub initialized: bool,
    pub ownership_latched: bool,
    pub taem_complete: bool,
    pub phase: TaemPhase,
    pub ownership_ut: ::core::ffi::c_double,
    pub ownership_velocity: ::core::ffi::c_double,
    pub last_transition_reason: TaemTransitionReason,
    pub last_transition_from: TaemPhase,
    pub last_transition_to: TaemPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_taem: bool,
    pub transition_count: ::core::ffi::c_uint,
    pub recovery_active: bool,
    pub recovery_reason: TaemRecoveryReason,
    pub last_recovery_reason: TaemRecoveryReason,
    pub last_recovery_ut: ::core::ffi::c_double,
    pub recovery_count: ::core::ffi::c_uint,
    pub terminal_contract: TaemTerminalContract,
    pub terminal_evaluation: TaemTerminalEvaluation,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TaemExecTelemetry {
    pub initialized: bool,
    pub ownership_latched: bool,
    pub taem_complete: bool,
    pub active_phase: TaemPhase,
    pub ownership_ut: ::core::ffi::c_double,
    pub ownership_velocity: ::core::ffi::c_double,
    pub last_transition_reason: TaemTransitionReason,
    pub last_transition_from: TaemPhase,
    pub last_transition_to: TaemPhase,
    pub last_transition_ut: ::core::ffi::c_double,
    pub last_transition_velocity: ::core::ffi::c_double,
    pub last_transition_completed_taem: bool,
    pub transition_count: ::core::ffi::c_uint,
    pub recovery_active: bool,
    pub recovery_reason: TaemRecoveryReason,
    pub last_recovery_reason: TaemRecoveryReason,
    pub last_recovery_ut: ::core::ffi::c_double,
    pub recovery_count: ::core::ffi::c_uint,
    pub terminal_contract: TaemTerminalContract,
    pub terminal_evaluation: TaemTerminalEvaluation,
}
pub type size_t = usize;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn phase_valid(mut phase: TaemPhase) -> bool {
    return phase as ::core::ffi::c_uint
        >= TAEM_PHASE_PATH_ACQUISITION as ::core::ffi::c_int as ::core::ffi::c_uint
        && phase as ::core::ffi::c_uint
            <= TAEM_PHASE_FINAL_INTERCEPT as ::core::ffi::c_int as ::core::ffi::c_uint;
}
unsafe extern "C" fn recovery_reason_valid(mut reason: TaemRecoveryReason) -> bool {
    return reason as ::core::ffi::c_uint
        >= TAEM_RECOVERY_ATTITUDE as ::core::ffi::c_int as ::core::ffi::c_uint
        && reason as ::core::ffi::c_uint
            <= TAEM_RECOVERY_ENERGY_INFEASIBLE as ::core::ffi::c_int
                as ::core::ffi::c_uint;
}
unsafe extern "C" fn observation_valid(
    mut observation: *const TaemExecObservation,
) -> bool {
    return !observation.is_null() && (*observation).ut.is_finite() as i32 != 0
        && (*observation).relative_velocity.is_finite() as i32 != 0
        && (*observation).relative_velocity >= 0.0f64;
}
unsafe extern "C" fn inputs_valid(mut inputs: *const TaemExecInputs) -> bool {
    if inputs.is_null() {
        return false_0 != 0;
    }
    if (*inputs).off_nominal_recovery_active as ::core::ffi::c_int != 0
        && !recovery_reason_valid((*inputs).off_nominal_recovery_reason)
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn terminal_contract_clear(mut contract: *mut TaemTerminalContract) {
    if contract.is_null() {
        return;
    }
    memset(
        contract as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemTerminalContract>() as size_t,
    );
    (*contract).range_to_go = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).range_margin = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).dynamic_pressure = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).dynamic_pressure_margin = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).speedbrake_dynamic_pressure_margin = ::core::f32::NAN
        as ::core::ffi::c_double;
    (*contract).altitude = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).altitude_margin = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).flight_path_angle = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).flight_path_angle_margin = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).specific_energy = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).specific_energy_margin = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).response_time_available = ::core::f32::NAN as ::core::ffi::c_double;
    (*contract).response_time_required = ::core::f32::NAN as ::core::ffi::c_double;
}
unsafe extern "C" fn terminal_contract_numbers_valid(
    mut contract: *const TaemTerminalContract,
) -> bool {
    if contract.is_null() || !(*contract).valid {
        return false_0 != 0;
    }
    if (*contract).range_to_go.is_finite() as i32 == 0
        || (*contract).range_to_go < 0.0f64
        || (*contract).range_margin.is_finite() as i32 == 0
        || (*contract).dynamic_pressure.is_finite() as i32 == 0
        || (*contract).dynamic_pressure < 0.0f64
        || (*contract).dynamic_pressure_margin.is_finite() as i32 == 0
        || (*contract).speedbrake_dynamic_pressure_margin.is_finite() as i32 == 0
        || (*contract).altitude.is_finite() as i32 == 0
        || (*contract).altitude_margin.is_finite() as i32 == 0
        || (*contract).flight_path_angle.is_finite() as i32 == 0
        || (*contract).flight_path_angle_margin.is_finite() as i32 == 0
        || (*contract).specific_energy.is_finite() as i32 == 0
        || (*contract).specific_energy_margin.is_finite() as i32 == 0
        || (*contract).response_time_available.is_finite() as i32 == 0
        || (*contract).response_time_required.is_finite() as i32 == 0
        || (*contract).response_time_available < 0.0f64
        || (*contract).response_time_required < 0.0f64
    {
        return false_0 != 0;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_evaluate_terminal_contract(
    mut contract: *const TaemTerminalContract,
) -> TaemTerminalEvaluation {
    let mut evaluation: TaemTerminalEvaluation = TaemTerminalEvaluation {
        valid: false_0 != 0,
        feasible: false_0 != 0,
        block_reason: TAEM_TERMINAL_BLOCK_CONTRACT_INVALID,
    };
    if !terminal_contract_numbers_valid(contract) {
        return evaluation;
    }
    evaluation.valid = true_0 != 0;
    if !(*contract).path_committed {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED;
        return evaluation;
    }
    if (*contract).range_margin < 0.0f64 {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_RANGE;
        return evaluation;
    }
    if (*contract).dynamic_pressure_margin < 0.0f64 {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE;
        return evaluation;
    }
    if (*contract).speedbrake_required as ::core::ffi::c_int != 0
        && (!(*contract).speedbrake_available
            || (*contract).speedbrake_dynamic_pressure_margin < 0.0f64)
    {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_SPEEDBRAKE;
        return evaluation;
    }
    if (*contract).altitude_margin < 0.0f64 {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_ALTITUDE;
        return evaluation;
    }
    if (*contract).flight_path_angle_margin < 0.0f64 {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE;
        return evaluation;
    }
    if (*contract).specific_energy_margin < 0.0f64 {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY;
        return evaluation;
    }
    if (*contract).response_time_available < (*contract).response_time_required {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_FINITE_RESPONSE;
        return evaluation;
    }
    if !(*contract).attitude_response_qualified {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE;
        return evaluation;
    }
    evaluation.feasible = true_0 != 0;
    evaluation.block_reason = TAEM_TERMINAL_BLOCK_NONE;
    return evaluation;
}
unsafe extern "C" fn update_terminal_contract(
    mut exec: *mut TaemExecutive,
    mut inputs: *const TaemExecInputs,
) {
    (*exec).terminal_contract = (*inputs).terminal_contract;
    (*exec).terminal_evaluation = taem_exec_evaluate_terminal_contract(
        &raw const (*inputs).terminal_contract,
    );
}
unsafe extern "C" fn record_transition(
    mut exec: *mut TaemExecutive,
    mut from: TaemPhase,
    mut to: TaemPhase,
    mut reason: TaemTransitionReason,
    mut observation: *const TaemExecObservation,
    mut completes_taem: bool,
) {
    (*exec).phase = to;
    (*exec).taem_complete = (*exec).taem_complete as ::core::ffi::c_int != 0
        || completes_taem as ::core::ffi::c_int != 0;
    (*exec).last_transition_reason = reason;
    (*exec).last_transition_from = from;
    (*exec).last_transition_to = to;
    (*exec).last_transition_ut = (*observation).ut;
    (*exec).last_transition_velocity = (*observation).relative_velocity;
    (*exec).last_transition_completed_taem = completes_taem;
    (*exec).transition_count = (*exec).transition_count.wrapping_add(1);
}
unsafe extern "C" fn update_recovery(
    mut exec: *mut TaemExecutive,
    mut observation: *const TaemExecObservation,
    mut inputs: *const TaemExecInputs,
) {
    if (*inputs).off_nominal_recovery_active {
        if !(*exec).recovery_active
            || (*exec).recovery_reason as ::core::ffi::c_uint
                != (*inputs).off_nominal_recovery_reason as ::core::ffi::c_uint
        {
            (*exec).last_recovery_reason = (*inputs).off_nominal_recovery_reason;
            (*exec).last_recovery_ut = (*observation).ut;
            (*exec).recovery_count = (*exec).recovery_count.wrapping_add(1);
        }
        (*exec).recovery_active = true_0 != 0;
        (*exec).recovery_reason = (*inputs).off_nominal_recovery_reason;
        return;
    }
    (*exec).recovery_active = false_0 != 0;
    (*exec).recovery_reason = TAEM_RECOVERY_NONE;
}
unsafe extern "C" fn restart_phase(
    mut inputs: *const TaemExecInputs,
    mut terminal: *const TaemTerminalEvaluation,
) -> TaemPhase {
    if ((*inputs).final_intercept_ready as ::core::ffi::c_int != 0
        || (*inputs).final_approach_ready as ::core::ffi::c_int != 0)
        && !terminal.is_null() && (*terminal).feasible as ::core::ffi::c_int != 0
    {
        return TAEM_PHASE_FINAL_INTERCEPT;
    }
    if (*inputs).terminal_path_captured as ::core::ffi::c_int != 0
        || (*inputs).final_intercept_ready as ::core::ffi::c_int != 0
        || (*inputs).final_approach_ready as ::core::ffi::c_int != 0
    {
        return TAEM_PHASE_RUNWAY_ALIGNMENT;
    }
    return TAEM_PHASE_PATH_ACQUISITION;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_reset(mut exec: *mut TaemExecutive) {
    if exec.is_null() {
        return;
    }
    memset(
        exec as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemExecutive>() as size_t,
    );
    (*exec).phase = TAEM_PHASE_PATH_ACQUISITION;
    (*exec).last_transition_from = TAEM_PHASE_PATH_ACQUISITION;
    (*exec).last_transition_to = TAEM_PHASE_PATH_ACQUISITION;
    (*exec).last_transition_reason = TAEM_TRANSITION_NONE;
    (*exec).ownership_ut = ::core::f32::NAN as ::core::ffi::c_double;
    (*exec).ownership_velocity = ::core::f32::NAN as ::core::ffi::c_double;
    (*exec).last_transition_ut = ::core::f32::NAN as ::core::ffi::c_double;
    (*exec).last_transition_velocity = ::core::f32::NAN as ::core::ffi::c_double;
    (*exec).recovery_reason = TAEM_RECOVERY_NONE;
    (*exec).last_recovery_reason = TAEM_RECOVERY_NONE;
    (*exec).last_recovery_ut = ::core::f32::NAN as ::core::ffi::c_double;
    terminal_contract_clear(&raw mut (*exec).terminal_contract);
    (*exec).terminal_evaluation.valid = false_0 != 0;
    (*exec).terminal_evaluation.feasible = false_0 != 0;
    (*exec).terminal_evaluation.block_reason = TAEM_TERMINAL_BLOCK_CONTRACT_INVALID;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_initialize(
    mut exec: *mut TaemExecutive,
    mut observation: *const TaemExecObservation,
    mut inputs: *const TaemExecInputs,
) -> bool {
    if exec.is_null() || !observation_valid(observation) || !inputs_valid(inputs) {
        return false_0 != 0;
    }
    let mut normal_handoff: bool = (*inputs).mm304_complete;
    let mut restart_handoff: bool = (*observation).checkpoint_restart
        as ::core::ffi::c_int != 0 && (*inputs).resume_mm305 as ::core::ffi::c_int != 0;
    if !normal_handoff && !restart_handoff {
        return false_0 != 0;
    }
    let mut next: TaemExecutive = TaemExecutive {
        initialized: false,
        ownership_latched: false,
        taem_complete: false,
        phase: TAEM_PHASE_PATH_ACQUISITION,
        ownership_ut: 0.,
        ownership_velocity: 0.,
        last_transition_reason: TAEM_TRANSITION_NONE,
        last_transition_from: TAEM_PHASE_PATH_ACQUISITION,
        last_transition_to: TAEM_PHASE_PATH_ACQUISITION,
        last_transition_ut: 0.,
        last_transition_velocity: 0.,
        last_transition_completed_taem: false,
        transition_count: 0,
        recovery_active: false,
        recovery_reason: TAEM_RECOVERY_NONE,
        last_recovery_reason: TAEM_RECOVERY_NONE,
        last_recovery_ut: 0.,
        recovery_count: 0,
        terminal_contract: TaemTerminalContract {
            valid: false,
            path_committed: false,
            range_to_go: 0.,
            range_margin: 0.,
            dynamic_pressure: 0.,
            dynamic_pressure_margin: 0.,
            speedbrake_required: false,
            speedbrake_available: false,
            speedbrake_dynamic_pressure_margin: 0.,
            altitude: 0.,
            altitude_margin: 0.,
            flight_path_angle: 0.,
            flight_path_angle_margin: 0.,
            specific_energy: 0.,
            specific_energy_margin: 0.,
            response_time_available: 0.,
            response_time_required: 0.,
            attitude_response_qualified: false,
        },
        terminal_evaluation: TaemTerminalEvaluation {
            valid: false,
            feasible: false,
            block_reason: TAEM_TERMINAL_BLOCK_NONE,
        },
    };
    taem_exec_reset(&raw mut next);
    next.initialized = true_0 != 0;
    next.ownership_latched = true_0 != 0;
    next.ownership_ut = (*observation).ut;
    next.ownership_velocity = (*observation).relative_velocity;
    update_terminal_contract(&raw mut next, inputs);
    if restart_handoff {
        next.phase = restart_phase(inputs, &raw mut next.terminal_evaluation);
        next.taem_complete = (*inputs).final_approach_ready as ::core::ffi::c_int != 0
            && next.terminal_evaluation.feasible as ::core::ffi::c_int != 0
            && !(*inputs).off_nominal_recovery_active;
        next.last_transition_reason = TAEM_TRANSITION_RESTART_CLASSIFICATION;
    } else {
        next.phase = TAEM_PHASE_PATH_ACQUISITION;
        next.last_transition_reason = TAEM_TRANSITION_MM304_HANDOFF;
    }
    next.last_transition_from = next.phase;
    next.last_transition_to = next.phase;
    next.last_transition_ut = (*observation).ut;
    next.last_transition_velocity = (*observation).relative_velocity;
    next.last_transition_completed_taem = next.taem_complete;
    update_recovery(&raw mut next, observation, inputs);
    *exec = next;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_update(
    mut exec: *mut TaemExecutive,
    mut observation: *const TaemExecObservation,
    mut inputs: *const TaemExecInputs,
) -> bool {
    if exec.is_null() || !observation_valid(observation) || !inputs_valid(inputs) {
        return false_0 != 0;
    }
    if !(*exec).initialized {
        return taem_exec_initialize(exec, observation, inputs);
    }
    if !(*exec).ownership_latched || !phase_valid((*exec).phase) {
        return false_0 != 0;
    }
    update_terminal_contract(exec, inputs);
    update_recovery(exec, observation, inputs);
    if (*exec).taem_complete as ::core::ffi::c_int != 0
        || (*exec).recovery_active as ::core::ffi::c_int != 0
    {
        return true_0 != 0;
    }
    let mut from: TaemPhase = (*exec).phase;
    match (*exec).phase as ::core::ffi::c_uint {
        0 => {
            if (*inputs).terminal_path_selected as ::core::ffi::c_int != 0
                && (*inputs).terminal_path_captured as ::core::ffi::c_int != 0
            {
                record_transition(
                    exec,
                    from,
                    TAEM_PHASE_RUNWAY_ALIGNMENT,
                    TAEM_TRANSITION_TERMINAL_PATH_CAPTURE,
                    observation,
                    false_0 != 0,
                );
            }
        }
        1 => {
            if !(*inputs).terminal_path_selected || !(*inputs).terminal_path_captured {
                record_transition(
                    exec,
                    from,
                    TAEM_PHASE_PATH_ACQUISITION,
                    TAEM_TRANSITION_TERMINAL_PATH_REPLAN,
                    observation,
                    false_0 != 0,
                );
            } else if (*inputs).final_intercept_ready as ::core::ffi::c_int != 0
                && (*exec).terminal_evaluation.feasible as ::core::ffi::c_int != 0
            {
                record_transition(
                    exec,
                    from,
                    TAEM_PHASE_FINAL_INTERCEPT,
                    TAEM_TRANSITION_FINAL_INTERCEPT_GATE,
                    observation,
                    false_0 != 0,
                );
            }
        }
        2 => {
            if !(*inputs).final_approach_ready
                && (!(*inputs).terminal_path_selected
                    || !(*inputs).terminal_path_captured)
            {
                record_transition(
                    exec,
                    from,
                    TAEM_PHASE_PATH_ACQUISITION,
                    TAEM_TRANSITION_TERMINAL_PATH_REPLAN,
                    observation,
                    false_0 != 0,
                );
            } else if (*inputs).final_approach_ready as ::core::ffi::c_int != 0
                && (*exec).terminal_evaluation.feasible as ::core::ffi::c_int != 0
            {
                record_transition(
                    exec,
                    from,
                    TAEM_PHASE_FINAL_INTERCEPT,
                    TAEM_TRANSITION_FINAL_APPROACH_DELIVERY,
                    observation,
                    true_0 != 0,
                );
            }
        }
        _ => return false_0 != 0,
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_telemetry(
    mut exec: *const TaemExecutive,
) -> TaemExecTelemetry {
    let mut telemetry: TaemExecTelemetry = TaemExecTelemetry {
        initialized: false,
        ownership_latched: false,
        taem_complete: false,
        active_phase: TAEM_PHASE_PATH_ACQUISITION,
        ownership_ut: 0.,
        ownership_velocity: 0.,
        last_transition_reason: TAEM_TRANSITION_NONE,
        last_transition_from: TAEM_PHASE_PATH_ACQUISITION,
        last_transition_to: TAEM_PHASE_PATH_ACQUISITION,
        last_transition_ut: 0.,
        last_transition_velocity: 0.,
        last_transition_completed_taem: false,
        transition_count: 0,
        recovery_active: false,
        recovery_reason: TAEM_RECOVERY_NONE,
        last_recovery_reason: TAEM_RECOVERY_NONE,
        last_recovery_ut: 0.,
        recovery_count: 0,
        terminal_contract: TaemTerminalContract {
            valid: false,
            path_committed: false,
            range_to_go: 0.,
            range_margin: 0.,
            dynamic_pressure: 0.,
            dynamic_pressure_margin: 0.,
            speedbrake_required: false,
            speedbrake_available: false,
            speedbrake_dynamic_pressure_margin: 0.,
            altitude: 0.,
            altitude_margin: 0.,
            flight_path_angle: 0.,
            flight_path_angle_margin: 0.,
            specific_energy: 0.,
            specific_energy_margin: 0.,
            response_time_available: 0.,
            response_time_required: 0.,
            attitude_response_qualified: false,
        },
        terminal_evaluation: TaemTerminalEvaluation {
            valid: false,
            feasible: false,
            block_reason: TAEM_TERMINAL_BLOCK_NONE,
        },
    };
    memset(
        &raw mut telemetry as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<TaemExecTelemetry>() as size_t,
    );
    telemetry.active_phase = TAEM_PHASE_PATH_ACQUISITION;
    telemetry.last_transition_from = TAEM_PHASE_PATH_ACQUISITION;
    telemetry.last_transition_to = TAEM_PHASE_PATH_ACQUISITION;
    telemetry.ownership_ut = ::core::f32::NAN as ::core::ffi::c_double;
    telemetry.ownership_velocity = ::core::f32::NAN as ::core::ffi::c_double;
    telemetry.last_transition_ut = ::core::f32::NAN as ::core::ffi::c_double;
    telemetry.last_transition_velocity = ::core::f32::NAN as ::core::ffi::c_double;
    telemetry.last_recovery_ut = ::core::f32::NAN as ::core::ffi::c_double;
    terminal_contract_clear(&raw mut telemetry.terminal_contract);
    telemetry.terminal_evaluation.block_reason = TAEM_TERMINAL_BLOCK_CONTRACT_INVALID;
    if exec.is_null() {
        return telemetry;
    }
    telemetry.initialized = (*exec).initialized;
    telemetry.ownership_latched = (*exec).ownership_latched;
    telemetry.taem_complete = (*exec).taem_complete;
    telemetry.active_phase = (*exec).phase;
    telemetry.ownership_ut = (*exec).ownership_ut;
    telemetry.ownership_velocity = (*exec).ownership_velocity;
    telemetry.last_transition_reason = (*exec).last_transition_reason;
    telemetry.last_transition_from = (*exec).last_transition_from;
    telemetry.last_transition_to = (*exec).last_transition_to;
    telemetry.last_transition_ut = (*exec).last_transition_ut;
    telemetry.last_transition_velocity = (*exec).last_transition_velocity;
    telemetry.last_transition_completed_taem = (*exec).last_transition_completed_taem;
    telemetry.transition_count = (*exec).transition_count;
    telemetry.recovery_active = (*exec).recovery_active;
    telemetry.recovery_reason = (*exec).recovery_reason;
    telemetry.last_recovery_reason = (*exec).last_recovery_reason;
    telemetry.last_recovery_ut = (*exec).last_recovery_ut;
    telemetry.recovery_count = (*exec).recovery_count;
    telemetry.terminal_contract = (*exec).terminal_contract;
    telemetry.terminal_evaluation = (*exec).terminal_evaluation;
    return telemetry;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_owns_vehicle(mut exec: *const TaemExecutive) -> bool {
    return !exec.is_null() && (*exec).initialized as ::core::ffi::c_int != 0
        && (*exec).ownership_latched as ::core::ffi::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_exec_deliver_low_energy(
    mut exec: *mut TaemExecutive,
    mut observation: *const TaemExecObservation,
) -> bool {
    if !taem_exec_owns_vehicle(exec) || !observation_valid(observation)
        || !phase_valid((*exec).phase)
        || (*exec).recovery_active as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    if (*exec).taem_complete {
        return true_0 != 0;
    }
    record_transition(
        exec,
        (*exec).phase,
        TAEM_PHASE_FINAL_INTERCEPT,
        TAEM_TRANSITION_LOW_ENERGY_DELIVERY,
        observation,
        true_0 != 0,
    );
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn taem_phase_string(
    mut phase: TaemPhase,
) -> *const ::core::ffi::c_char {
    match phase as ::core::ffi::c_uint {
        0 => return b"Path Acquisition\0" as *const u8 as *const ::core::ffi::c_char,
        1 => return b"Runway Alignment\0" as *const u8 as *const ::core::ffi::c_char,
        2 => return b"Final Intercept\0" as *const u8 as *const ::core::ffi::c_char,
        _ => return b"Unknown TAEM Phase\0" as *const u8 as *const ::core::ffi::c_char,
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_transition_reason_string(
    mut reason: TaemTransitionReason,
) -> *const ::core::ffi::c_char {
    match reason as ::core::ffi::c_uint {
        0 => return b"None\0" as *const u8 as *const ::core::ffi::c_char,
        1 => {
            return b"MM304 qualified handoff\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        2 => {
            return b"Checkpoint/restart classification\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        3 => {
            return b"Terminal path captured\0" as *const u8 as *const ::core::ffi::c_char;
        }
        4 => {
            return b"Runway alignment/final intercept gate\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        5 => {
            return b"Final-approach delivery\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        6 => {
            return b"Terminal path lost/stale; replan inside TAEM\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        7 => {
            return b"Low-energy final-approach delivery\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        _ => {
            return b"Unknown TAEM transition\0" as *const u8
                as *const ::core::ffi::c_char;
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_recovery_reason_string(
    mut reason: TaemRecoveryReason,
) -> *const ::core::ffi::c_char {
    match reason as ::core::ffi::c_uint {
        0 => return b"None\0" as *const u8 as *const ::core::ffi::c_char,
        1 => {
            return b"Off-nominal attitude recovery\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        2 => {
            return b"Off-nominal terminal path replan\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        3 => {
            return b"Off-nominal terminal path capture loss\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        4 => {
            return b"Off-nominal terminal energy infeasible\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        _ => return b"Unknown TAEM recovery\0" as *const u8 as *const ::core::ffi::c_char,
    };
}
#[no_mangle]
pub unsafe extern "C" fn taem_terminal_block_reason_string(
    mut reason: TaemTerminalBlockReason,
) -> *const ::core::ffi::c_char {
    match reason as ::core::ffi::c_uint {
        0 => return b"None\0" as *const u8 as *const ::core::ffi::c_char,
        1 => {
            return b"Terminal contract invalid\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        2 => {
            return b"Terminal path not committed\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        3 => {
            return b"Insufficient runway-alignment range reserve\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        4 => {
            return b"Dynamic-pressure envelope exceeded\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        5 => {
            return b"Speedbrake authority/envelope unavailable\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        6 => {
            return b"Terminal altitude envelope missed\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        7 => {
            return b"Terminal flight-path-angle envelope missed\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        8 => {
            return b"Terminal specific-energy envelope missed\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        9 => {
            return b"Insufficient finite-response time\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        10 => {
            return b"Attitude response not qualified\0" as *const u8
                as *const ::core::ffi::c_char;
        }
        _ => {
            return b"Unknown terminal block reason\0" as *const u8
                as *const ::core::ffi::c_char;
        }
    };
}
