#include "../CLanding/include/taem_exec.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static TaemTerminalContract feasible_contract(void){
    TaemTerminalContract c={
        .valid=true,
        .path_committed=true,
        .range_to_go=12000.0,
        .range_margin=2500.0,
        .dynamic_pressure=9000.0,
        .dynamic_pressure_margin=15000.0,
        .speedbrake_required=false,
        .speedbrake_available=true,
        .speedbrake_dynamic_pressure_margin=10000.0,
        .altitude=8000.0,
        .altitude_margin=1200.0,
        .flight_path_angle=-15.0,
        .flight_path_angle_margin=2.0,
        .specific_energy=420000.0,
        .specific_energy_margin=30000.0,
        .response_time_available=10.0,
        .response_time_required=5.0,
        .attitude_response_qualified=true,
    };
    return c;
}

static TaemExecObservation observation(double ut,double speed){
    return (TaemExecObservation){
        .ut=ut,
        .relative_velocity=speed,
        .checkpoint_restart=false,
    };
}

static void test_mm305_always_enters_path_acquisition(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs={0};
    inputs.mm304_complete=true;

    TaemExecObservation o=observation(1000.0,700.0);
    assert(taem_exec_initialize(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.last_transition_reason==TAEM_TRANSITION_MM304_HANDOFF);
    assert(taem_exec_owns_vehicle(&exec));
}

static void test_coarse_path_signals_cannot_bypass_terminal_contract(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs={0};
    inputs.mm304_complete=true;
    inputs.terminal_path_selected=true;
    inputs.terminal_path_captured=true;
    inputs.final_intercept_ready=true;
    inputs.final_approach_ready=true;

    TaemExecObservation o=observation(2000.0,420.0);
    assert(taem_exec_initialize(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    assert(!exec.terminal_evaluation.valid);

    o.ut+=1.0;
    assert(taem_exec_update(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(!exec.taem_complete);

    o.ut+=1.0;
    assert(taem_exec_update(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(!exec.taem_complete);
    assert(exec.terminal_evaluation.block_reason==
        TAEM_TERMINAL_BLOCK_CONTRACT_INVALID);
}

static void test_feasible_contract_allows_forward_delivery_only(void){
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecInputs inputs={0};
    inputs.mm304_complete=true;
    inputs.terminal_contract=feasible_contract();

    TaemExecObservation o=observation(3000.0,390.0);
    assert(taem_exec_initialize(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);

    inputs.terminal_path_selected=true;
    inputs.terminal_path_captured=true;
    o.ut+=1.0;
    assert(taem_exec_update(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_RUNWAY_ALIGNMENT);

    inputs.final_intercept_ready=true;
    o.ut+=1.0;
    assert(taem_exec_update(&exec,&o,&inputs));
    assert(exec.phase==TAEM_PHASE_FINAL_INTERCEPT);

    inputs.final_approach_ready=true;
    o.ut+=1.0;
    assert(taem_exec_update(&exec,&o,&inputs));
    assert(exec.taem_complete);
    assert(exec.last_transition_reason==
        TAEM_TRANSITION_FINAL_APPROACH_DELIVERY);
}

int main(void){
    test_mm305_always_enters_path_acquisition();
    test_coarse_path_signals_cannot_bypass_terminal_contract();
    test_feasible_contract_allows_forward_delivery_only();
    puts("TAEM production integration contract tests passed.");
    return 0;
}
