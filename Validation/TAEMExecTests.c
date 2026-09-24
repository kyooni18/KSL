#include "taem_exec.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static TaemExecObservation obs(double ut,double velocity){
    TaemExecObservation value={
        .ut=ut,
        .relative_velocity=velocity,
        .checkpoint_restart=false,
    };
    return value;
}

static TaemTerminalContract terminal_contract_nominal(void){
    TaemTerminalContract contract={
        .valid=true,
        .path_committed=true,
        .range_to_go=26000.0,
        .range_margin=6000.0,
        .dynamic_pressure=8000.0,
        .dynamic_pressure_margin=20000.0,
        .speedbrake_required=false,
        .speedbrake_available=true,
        .speedbrake_dynamic_pressure_margin=12000.0,
        .altitude=9000.0,
        .altitude_margin=1600.0,
        .flight_path_angle=-12.0,
        .flight_path_angle_margin=3.0,
        .specific_energy=520000.0,
        .specific_energy_margin=60000.0,
        .response_time_available=12.0,
        .response_time_required=6.0,
        .attitude_response_qualified=true,
    };
    return contract;
}

static TaemExecInputs nominal_handoff(void){
    TaemExecInputs inputs;
    memset(&inputs,0,sizeof(inputs));
    inputs.mm304_complete=true;
    inputs.terminal_contract=terminal_contract_nominal();
    return inputs;
}

static void test_terminal_contract_is_fail_closed(void){
    TaemTerminalContract contract=terminal_contract_nominal();
    TaemTerminalEvaluation e=taem_exec_evaluate_terminal_contract(&contract);
    assert(e.valid&&e.feasible);
    assert(e.block_reason==TAEM_TERMINAL_BLOCK_NONE);

    contract.path_committed=false;
    e=taem_exec_evaluate_terminal_contract(&contract);
    assert(e.valid&&!e.feasible);
    assert(e.block_reason==TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED);

    contract=terminal_contract_nominal();
    contract.specific_energy_margin=-1.0;
    e=taem_exec_evaluate_terminal_contract(&contract);
    assert(e.valid&&!e.feasible);
    assert(e.block_reason==TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY);

    contract=terminal_contract_nominal();
    contract.response_time_available=5.0;
    contract.response_time_required=6.0;
    e=taem_exec_evaluate_terminal_contract(&contract);
    assert(e.block_reason==TAEM_TERMINAL_BLOCK_FINITE_RESPONSE);

    contract=terminal_contract_nominal();
    contract.attitude_response_qualified=false;
    e=taem_exec_evaluate_terminal_contract(&contract);
    assert(e.block_reason==TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE);
}

static void test_mm305_has_only_forward_terminal_phases(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs=nominal_handoff();
    TaemExecObservation state=obs(100.0,590.0);

    assert(taem_exec_initialize(&exec,&state,&inputs));
    assert(taem_exec_owns_vehicle(&exec));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.last_transition_reason==TAEM_TRANSITION_MM304_HANDOFF);
    assert(!exec.taem_complete);

    inputs.terminal_path_selected=true;
    inputs.terminal_path_captured=true;
    state=obs(120.0,520.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(exec.last_transition_reason==TAEM_TRANSITION_TERMINAL_PATH_CAPTURE);

    inputs.final_intercept_ready=true;
    state=obs(145.0,420.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(exec.phase==TAEM_PHASE_FINAL_INTERCEPT);
    assert(exec.last_transition_reason==TAEM_TRANSITION_FINAL_INTERCEPT_GATE);

    inputs.final_approach_ready=true;
    state=obs(155.0,360.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(exec.taem_complete);
    assert(exec.last_transition_reason==TAEM_TRANSITION_FINAL_APPROACH_DELIVERY);
}

static void test_path_loss_replans_inside_mm305(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs=nominal_handoff();
    inputs.terminal_path_selected=true;
    inputs.terminal_path_captured=true;
    TaemExecObservation state=obs(200.0,500.0);

    assert(taem_exec_initialize(&exec,&state,&inputs));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    state=obs(201.0,495.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);

    inputs.terminal_path_selected=false;
    inputs.terminal_path_captured=false;
    state=obs(202.0,490.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.ownership_latched);
    assert(exec.last_transition_reason==TAEM_TRANSITION_TERMINAL_PATH_REPLAN);
}

static void test_handoff_and_restart_fail_closed(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs=nominal_handoff();
    inputs.mm304_complete=false;
    TaemExecObservation state=obs(300.0,500.0);
    assert(!taem_exec_initialize(&exec,&state,&inputs));
    assert(!taem_exec_owns_vehicle(&exec));

    taem_exec_reset(&exec);
    inputs.resume_mm305=true;
    inputs.terminal_path_selected=true;
    inputs.terminal_path_captured=true;
    state.checkpoint_restart=true;
    assert(taem_exec_initialize(&exec,&state,&inputs));
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(exec.last_transition_reason==TAEM_TRANSITION_RESTART_CLASSIFICATION);
}

static void test_recovery_holds_phase_and_ownership(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs=nominal_handoff();
    TaemExecObservation state=obs(400.0,470.0);
    assert(taem_exec_initialize(&exec,&state,&inputs));

    inputs.off_nominal_recovery_active=true;
    inputs.off_nominal_recovery_reason=TAEM_RECOVERY_ATTITUDE;
    inputs.terminal_path_selected=true;
    inputs.terminal_path_captured=true;
    state=obs(401.0,465.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(exec.recovery_active);
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.ownership_latched);

    inputs.off_nominal_recovery_active=false;
    inputs.off_nominal_recovery_reason=TAEM_RECOVERY_NONE;
    state=obs(402.0,460.0);
    assert(taem_exec_update(&exec,&state,&inputs));
    assert(!exec.recovery_active);
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);
}

static void test_invalid_observation_and_strings(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs=nominal_handoff();
    TaemExecObservation state=obs(500.0,450.0);
    assert(taem_exec_initialize(&exec,&state,&inputs));

    state.relative_velocity=NAN;
    assert(!taem_exec_update(&exec,&state,&inputs));

    assert(strcmp(taem_phase_string(TAEM_PHASE_PATH_ACQUISITION),
        "Path Acquisition")==0);
    assert(strcmp(taem_phase_string(TAEM_PHASE_RUNWAY_ALIGNMENT),
        "Runway Alignment")==0);
    assert(strcmp(taem_phase_string(TAEM_PHASE_FINAL_INTERCEPT),
        "Final Intercept")==0);
    assert(strcmp(taem_transition_reason_string(TAEM_TRANSITION_TERMINAL_PATH_REPLAN),
        "Terminal path lost/stale; replan inside TAEM")==0);
}

int main(void){
    test_terminal_contract_is_fail_closed();
    test_mm305_has_only_forward_terminal_phases();
    test_path_loss_replans_inside_mm305();
    test_handoff_and_restart_fail_closed();
    test_recovery_holds_phase_and_ownership();
    test_invalid_observation_and_strings();
    puts("TAEM executive tests passed.");
    return 0;
}
