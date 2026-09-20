#include "taem_exec.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static TaemExecObservation obs(double ut, double velocity) {
    TaemExecObservation value = {
        .ut = ut,
        .relative_velocity = velocity,
        .checkpoint_restart = false,
    };
    return value;
}

static TaemExecProfile profile_default(void) {
    TaemExecProfile profile = {
        .s_turn_enabled = true,
    };
    return profile;
}

static TaemTerminalContract terminal_contract_nominal(void) {
    TaemTerminalContract contract = {
        .valid = true,
        .path_committed = true,
        .range_to_go = 26000.0,
        .range_margin = 6000.0,
        .dynamic_pressure = 8000.0,
        .dynamic_pressure_margin = 20000.0,
        .speedbrake_required = false,
        .speedbrake_available = true,
        .speedbrake_dynamic_pressure_margin = 12000.0,
        .altitude = 9000.0,
        .altitude_margin = 1600.0,
        .flight_path_angle = -12.0,
        .flight_path_angle_margin = 3.0,
        .specific_energy = 520000.0,
        .specific_energy_margin = 60000.0,
        .response_time_available = 12.0,
        .response_time_required = 6.0,
        .attitude_response_qualified = true,
    };
    return contract;
}

static TaemExecInputs nominal_handoff(void) {
    TaemExecInputs inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.mm304_complete = true;
    inputs.energy_valid = true;
    inputs.energy_excess = 10.0;
    inputs.terminal_feasibility_valid = true;
    inputs.nominal_terminal_path_feasible = true;
    inputs.terminal_contract = terminal_contract_nominal();
    return inputs;
}

static void test_terminal_contract_dimension_evaluation(void) {
    TaemTerminalContract contract = terminal_contract_nominal();
    TaemTerminalEvaluation evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_NONE);

    contract = terminal_contract_nominal();
    contract.valid = false;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(!evaluation.valid && !evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_CONTRACT_INVALID);

    contract = terminal_contract_nominal();
    contract.range_to_go = NAN;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(!evaluation.valid);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_CONTRACT_INVALID);

    contract = terminal_contract_nominal();
    contract.path_committed = false;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && !evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED);

    contract = terminal_contract_nominal();
    contract.range_margin = -1.0;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_RANGE);

    contract = terminal_contract_nominal();
    contract.dynamic_pressure_margin = -1.0;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE);

    contract = terminal_contract_nominal();
    contract.speedbrake_required = true;
    contract.speedbrake_available = false;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_SPEEDBRAKE);

    contract = terminal_contract_nominal();
    contract.speedbrake_required = true;
    contract.speedbrake_dynamic_pressure_margin = -1.0;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_SPEEDBRAKE);

    contract = terminal_contract_nominal();
    contract.altitude_margin = -1.0;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_ALTITUDE);

    contract = terminal_contract_nominal();
    contract.flight_path_angle_margin = -0.1;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE);

    contract = terminal_contract_nominal();
    contract.specific_energy_margin = -1.0;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY);

    contract = terminal_contract_nominal();
    contract.response_time_available = 5.0;
    contract.response_time_required = 6.0;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_FINITE_RESPONSE);

    contract = terminal_contract_nominal();
    contract.attitude_response_qualified = false;
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE);
}

static void test_normal_entry_to_acquisition_and_forward_progression(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    TaemExecObservation state = obs(100.0, 590.0);

    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(taem_exec_owns_vehicle(&exec));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(!exec.taem_complete);
    assert(exec.last_transition_reason == TAEM_TRANSITION_MM304_HANDOFF);
    assert(exec.transition_count == 0);
    assert(exec.terminal_evaluation.feasible);

    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;
    state = obs(120.0, 520.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(exec.last_transition_reason == TAEM_TRANSITION_TERMINAL_PATH_CAPTURE);

    inputs.final_intercept_ready = true;
    state = obs(150.0, 410.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_FINAL_INTERCEPT);
    assert(exec.last_transition_reason == TAEM_TRANSITION_FINAL_INTERCEPT_GATE);

    inputs.final_approach_ready = true;
    state = obs(160.0, 360.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_FINAL_INTERCEPT);
    assert(exec.taem_complete);
    assert(exec.last_transition_reason == TAEM_TRANSITION_FINAL_APPROACH_DELIVERY);
    assert(exec.last_transition_completed_taem);
    assert(exec.transition_count == 3);
}

static void test_mm304_contract_failure_cannot_create_taem_ownership(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    inputs.mm304_complete = false;
    TaemExecObservation state = obs(100.0, 500.0);

    assert(!taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(!taem_exec_owns_vehicle(&exec));
    assert(!exec.initialized);
}

static void test_high_energy_s_turn_preserves_terminal_alignment_energy(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    inputs.energy_excess = 150.0;
    inputs.nominal_terminal_path_feasible = false;
    inputs.terminal_contract.path_committed = false;
    TaemExecObservation state = obs(200.0, 650.0);

    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_S_TURN);
    assert(exec.last_transition_reason == TAEM_TRANSITION_EXCESS_ENERGY_S_TURN);

    /* Once a nominal terminal path can absorb the remaining energy, additional
       S-turn distance only spends runway-alignment energy, so exit immediately. */
    inputs.nominal_terminal_path_feasible = true;
    inputs.terminal_contract = terminal_contract_nominal();
    inputs.energy_excess = 40.0;
    state = obs(210.0, 620.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.last_transition_reason == TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE);

    /* Without a feasible terminal candidate, the signed surplus balance itself
       ends dissipation: zero means no extra path length is physically justified. */
    taem_exec_reset(&exec);
    inputs = nominal_handoff();
    inputs.energy_excess = 150.0;
    inputs.nominal_terminal_path_feasible = false;
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_S_TURN);
    inputs.energy_excess = 0.0;
    state = obs(220.0, 600.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.last_transition_reason == TAEM_TRANSITION_S_TURN_SURPLUS_EXHAUSTED);
}

static void test_s_turn_requires_energy_and_terminal_infeasibility(void) {
    TaemExecutive exec;
    TaemExecProfile profile = profile_default();
    TaemExecObservation state = obs(250.0, 620.0);

    TaemExecInputs inputs = nominal_handoff();
    inputs.energy_excess = 200.0;
    inputs.nominal_terminal_path_feasible = true;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);

    inputs.nominal_terminal_path_feasible = false;
    inputs.terminal_feasibility_valid = false;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
}

static void test_infeasible_contract_blocks_prefinal_and_al_handoffs(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    TaemExecObservation state = obs(270.0, 580.0);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));

    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;
    state = obs(271.0, 560.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);

    inputs.final_intercept_ready = true;
    inputs.terminal_contract.specific_energy_margin = -50.0;
    state = obs(272.0, 520.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(exec.terminal_evaluation.block_reason == TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY);

    inputs.terminal_contract = terminal_contract_nominal();
    state = obs(273.0, 500.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_FINAL_INTERCEPT);

    inputs.final_approach_ready = true;
    inputs.terminal_contract.response_time_available = 3.0;
    inputs.terminal_contract.response_time_required = 6.0;
    state = obs(274.0, 470.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_FINAL_INTERCEPT);
    assert(!exec.taem_complete);
    assert(exec.terminal_evaluation.block_reason == TAEM_TERMINAL_BLOCK_FINITE_RESPONSE);

    inputs.terminal_contract.response_time_available = 10.0;
    state = obs(275.0, 450.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.taem_complete);
}

static void test_latched_taem_replans_internally_on_terminal_path_loss(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    TaemExecObservation state = obs(300.0, 590.0);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));

    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;
    state = obs(310.0, 520.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    unsigned transitions = exec.transition_count;

    /* Losing the active terminal path after MM305 ownership is not permission to
       reacquire MM304. With no positive surplus, reacquire a terminal path inside
       TAEM rather than adding unnecessary distance. */
    inputs.mm304_complete = false;
    inputs.terminal_path_selected = false;
    inputs.terminal_path_captured = false;
    inputs.nominal_terminal_path_feasible = false;
    inputs.energy_excess = 0.0;
    inputs.terminal_contract.path_committed = false;
    state = obs(311.0, 515.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(taem_exec_owns_vehicle(&exec));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.transition_count == transitions + 1);

    /* If the same path loss exposes positive surplus, the unified TAEM planner
       owns a fresh S-turn rather than bouncing back to Entry. */
    inputs.energy_excess = 500.0;
    inputs.terminal_feasibility_valid = true;
    state = obs(312.0, 510.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(taem_exec_owns_vehicle(&exec));
    assert(exec.phase == TAEM_PHASE_S_TURN);
}

static void test_explicit_off_nominal_recovery_labeling(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    TaemExecObservation state = obs(400.0, 590.0);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));

    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;
    inputs.off_nominal_recovery_active = true;
    inputs.off_nominal_recovery_reason = TAEM_RECOVERY_PATH_REPLAN;
    state = obs(401.0, 585.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.recovery_active);
    assert(exec.recovery_reason == TAEM_RECOVERY_PATH_REPLAN);
    assert(exec.last_recovery_reason == TAEM_RECOVERY_PATH_REPLAN);
    assert(exec.recovery_count == 1);
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);

    state = obs(402.0, 580.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(exec.recovery_count == 1);
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);

    inputs.off_nominal_recovery_active = false;
    inputs.off_nominal_recovery_reason = TAEM_RECOVERY_NONE;
    state = obs(403.0, 575.0);
    assert(taem_exec_update(&exec, &state, &inputs, &profile));
    assert(!exec.recovery_active);
    assert(exec.recovery_reason == TAEM_RECOVERY_NONE);
    assert(exec.last_recovery_reason == TAEM_RECOVERY_PATH_REPLAN);
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
}

static void test_mid_taem_restart_initialization(void) {
    TaemExecProfile profile = profile_default();
    TaemExecObservation state = obs(500.0, 480.0);
    state.checkpoint_restart = true;
    TaemExecInputs inputs = nominal_handoff();
    inputs.mm304_complete = false;
    inputs.resume_mm305 = true;
    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;

    TaemExecutive exec;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(exec.last_transition_reason == TAEM_TRANSITION_RESTART_CLASSIFICATION);
    assert(exec.transition_count == 0);

    inputs.final_intercept_ready = true;
    state = obs(510.0, 400.0);
    state.checkpoint_restart = true;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_FINAL_INTERCEPT);

    inputs.final_approach_ready = true;
    state = obs(520.0, 350.0);
    state.checkpoint_restart = true;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_FINAL_INTERCEPT);
    assert(exec.taem_complete);

    /* Stale checkpoint flags cannot bypass a missing/expired candidate contract. */
    inputs.terminal_contract.path_committed = false;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(!exec.taem_complete);
    assert(exec.terminal_evaluation.block_reason == TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED);
}

static void test_latest_recorded_pre_taem_fixture_fails_closed(void) {
    /* Frozen from FlightLogs/2026-09-10T02-08-11Z-STS-N.jsonl near the final
       connected snapshot: terminal candidate existed, but terminalPathCommitted
       was false at 147.600 km range, 31.118 km altitude, 1500.782 m/s TAS,
       -4.583 deg FPA and 5.203 kPa q-bar. A restart with stale downstream flags
       must therefore reacquire Runway Alignment instead of skipping into final approach. */
    TaemTerminalContract contract = terminal_contract_nominal();
    contract.path_committed = false;
    contract.range_to_go = 147600.307160374;
    contract.dynamic_pressure = 5203.3623046875;
    contract.altitude = 31118.4956143626;
    contract.flight_path_angle = -4.58343760549857;
    contract.specific_energy =
        0.5 * 1500.78210449219 * 1500.78210449219 + 9.81 * 31118.4956143626;

    TaemTerminalEvaluation evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && !evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED);

    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    inputs.mm304_complete = false;
    inputs.resume_mm305 = true;
    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;
    inputs.final_intercept_ready = true;
    inputs.final_approach_ready = true;
    inputs.terminal_contract = contract;
    TaemExecObservation state = obs(67502.012275178, 1500.78210449219);
    state.checkpoint_restart = true;

    TaemExecutive exec;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    assert(taem_exec_owns_vehicle(&exec));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(!exec.taem_complete);
}

static void test_predictor_refresh_stability_and_invalid_inputs(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    inputs.energy_excess = 0.0; /* Isolate predictor-refresh stability from energy-management requests. */
    TaemExecObservation state = obs(600.0, 590.0);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));

    TaemExecutive before = exec;
    for (int i = 0; i < 50; ++i) {
        inputs.terminal_path_selected = (i % 2) == 0;
        inputs.terminal_feasibility_valid = (i % 3) != 0;
        inputs.nominal_terminal_path_feasible = (i % 5) != 0;
        state = obs(601.0 + i * 0.1, 589.0 - i * 0.1);
        assert(taem_exec_update(&exec, &state, &inputs, &profile));
        assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
        assert(exec.transition_count == before.transition_count);
    }

    TaemExecutive stable = exec;
    state.relative_velocity = NAN;
    assert(!taem_exec_update(&exec, &state, &inputs, &profile));
    assert(memcmp(&stable, &exec, sizeof(exec)) == 0);

    state = obs(610.0, 580.0);
    inputs.energy_valid = true;
    inputs.energy_excess = NAN;
    assert(!taem_exec_update(&exec, &state, &inputs, &profile));
    assert(memcmp(&stable, &exec, sizeof(exec)) == 0);
}

static void test_telemetry_and_strings(void) {
    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecProfile profile = profile_default();
    TaemExecInputs inputs = nominal_handoff();
    TaemExecObservation state = obs(700.0, 590.0);
    assert(taem_exec_initialize(&exec, &state, &inputs, &profile));
    TaemExecTelemetry telemetry = taem_exec_telemetry(&exec);
    assert(telemetry.initialized && telemetry.ownership_latched);
    assert(telemetry.active_phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(telemetry.ownership_ut == 700.0);
    assert(telemetry.ownership_velocity == 590.0);
    assert(telemetry.terminal_contract.valid);
    assert(telemetry.terminal_evaluation.feasible);
    assert(strcmp(taem_phase_string(TAEM_PHASE_RUNWAY_ALIGNMENT), "Runway Alignment") == 0);
    assert(strcmp(taem_transition_reason_string(TAEM_TRANSITION_TERMINAL_PATH_CAPTURE), "Terminal path captured") == 0);
    assert(strcmp(taem_transition_reason_string(TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE),
                  "Terminal path feasible; preserve alignment energy") == 0);
    assert(strcmp(taem_recovery_reason_string(TAEM_RECOVERY_CAPTURE_LOSS),
                  "Off-nominal terminal path capture loss") == 0);
    assert(strcmp(taem_transition_reason_string(TAEM_TRANSITION_TERMINAL_PATH_REPLAN),
                  "Terminal path lost/stale; replan inside TAEM") == 0);
    assert(strcmp(taem_terminal_block_reason_string(TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY),
                  "Terminal specific-energy envelope missed") == 0);
}

static void test_restart_defers_delivery_during_recovery(void) {
    TaemExecProfile profile = profile_default();
    for (int reason = TAEM_RECOVERY_ATTITUDE; reason <= TAEM_RECOVERY_ENERGY_INFEASIBLE; ++reason) {
        TaemExecutive exec;
        taem_exec_reset(&exec);
        TaemExecObservation observation = obs(100, 200);
        observation.checkpoint_restart = true;
        TaemExecInputs inputs = nominal_handoff();
        inputs.resume_mm305 = true;
        inputs.final_approach_ready = true;
        inputs.off_nominal_recovery_active = true;
        inputs.off_nominal_recovery_reason = (TaemRecoveryReason)reason;
        assert(taem_exec_initialize(&exec, &observation, &inputs, &profile));
        assert(exec.recovery_active);
        assert(!exec.taem_complete);
        assert(!exec.last_transition_completed_taem);
        observation.ut += 1;
        assert(taem_exec_update(&exec, &observation, &inputs, &profile));
        assert(!exec.taem_complete);
        inputs.off_nominal_recovery_active = false;
        observation.ut += 1;
        assert(taem_exec_update(&exec, &observation, &inputs, &profile));
        assert(exec.taem_complete);
        assert(!exec.recovery_active);
    }
}

int main(void) {
    test_restart_defers_delivery_during_recovery();
    test_terminal_contract_dimension_evaluation();
    test_normal_entry_to_acquisition_and_forward_progression();
    test_mm304_contract_failure_cannot_create_taem_ownership();
    test_high_energy_s_turn_preserves_terminal_alignment_energy();
    test_s_turn_requires_energy_and_terminal_infeasibility();
    test_infeasible_contract_blocks_prefinal_and_al_handoffs();
    test_latched_taem_replans_internally_on_terminal_path_loss();
    test_explicit_off_nominal_recovery_labeling();
    test_mid_taem_restart_initialization();
    test_latest_recorded_pre_taem_fixture_fails_closed();
    test_predictor_refresh_stability_and_invalid_inputs();
    test_telemetry_and_strings();
    puts("TAEM executive tests passed.");
    return 0;
}
