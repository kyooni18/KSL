#include "../CLanding/guidance.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_production_profile_can_select_optional_high_energy_s_turn(void) {
    LandingConfiguration cfg = landing_configuration_default();
    TaemExecProfile profile = taem_exec_profile_production(&cfg.guidance);

    assert(profile.s_turn_enabled);

    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecObservation observation = {
        .ut = 1000.0,
        .relative_velocity = cfg.guidance.taem_force_handoff_speed,
        .checkpoint_restart = false,
    };
    TaemExecInputs inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.mm304_complete = true;
    inputs.energy_valid = true;
    inputs.energy_excess = 1.0;
    inputs.terminal_feasibility_valid = true;
    inputs.nominal_terminal_path_feasible = false;

    assert(taem_exec_initialize(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_S_TURN);
}


static void test_production_high_energy_preview_cannot_skip_s_turn_above_shell(void) {
    LandingConfiguration cfg = landing_configuration_default();
    TaemExecProfile profile = taem_exec_profile_production(&cfg.guidance);
    GuidanceMachine g;
    memset(&g, 0, sizeof(g));
    g.entry_exec.entry_complete = true;
    g.terminal_candidate.valid = true;
    g.terminal_candidate.geometry_degraded = false;
    g.terminal_candidate.energy_degraded = false;

    double shell_min = 0.0, shell_max = 0.0;
    entry_taem_handoff_altitude_bounds(&cfg.guidance, &shell_min, &shell_max);
    Telemetry t;
    memset(&t, 0, sizeof(t));
    t.true_air_speed = cfg.guidance.taem_force_handoff_speed;
    t.energy_excess_range = 50000.0;
    t.mean_altitude = shell_max + 5000.0;

    TaemExecInputs inputs = taem_exec_inputs_live(&g, &t, &cfg.guidance);
    assert(inputs.energy_valid);
    assert(inputs.terminal_feasibility_valid);
    assert(!inputs.nominal_terminal_path_feasible);

    TaemExecutive exec;
    taem_exec_reset(&exec);
    TaemExecObservation observation = {
        .ut = 3000.0,
        .relative_velocity = t.true_air_speed,
        .checkpoint_restart = false,
    };
    assert(taem_exec_initialize(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_S_TURN);

    /* Once the live vehicle reaches the configured TAEM altitude shell, an
       actually nominal candidate may end the S-turn even if the coarse excess
       metric has not yet reached its lower fallback threshold. */
    t.mean_altitude = shell_max - 100.0;
    inputs = taem_exec_inputs_live(&g, &t, &cfg.guidance);
    assert(inputs.nominal_terminal_path_feasible);
    observation.ut += 1.0;
    assert(taem_exec_update(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.last_transition_reason == TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE);
}

static void test_recorded_20260910_140550_handoff_keeps_high_energy_s_turn(void) {
    LandingConfiguration cfg = landing_configuration_default();
    TaemExecProfile profile = taem_exec_profile_production(&cfg.guidance);
    GuidanceMachine g;
    memset(&g, 0, sizeof(g));
    g.entry_exec.entry_complete = true;
    g.terminal_candidate.valid = true;
    g.terminal_candidate.geometry_degraded = false;
    g.terminal_candidate.energy_degraded = false;

    /* FlightLogs/2026-09-10T14-05-50Z-STS-N.jsonl, seq 2202. The old
       build entered TAEM acquisition at 28.55 km / 1299.01 m/s with about
       +101 km of excess range and never captured a terminal path. Current
       MM305 must classify that same state as high-energy S-turn instead of
       accepting the nominal-looking preview while far above the TAEM shell. */
    Telemetry t;
    memset(&t, 0, sizeof(t));
    t.ut = 67508.4122751793;
    t.true_air_speed = 1299.01356944213;
    t.mean_altitude = 28547.9985856424;
    t.energy_excess_range = 101090.648698728;

    TaemExecInputs inputs = taem_exec_inputs_live(&g, &t, &cfg.guidance);
    assert(inputs.energy_valid);
    assert(inputs.terminal_feasibility_valid);
    assert(!inputs.nominal_terminal_path_feasible);

    TaemExecObservation observation = {
        .ut = t.ut,
        .relative_velocity = t.true_air_speed,
        .checkpoint_restart = false,
    };
    TaemExecutive exec;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_S_TURN);
    assert(exec.last_transition_reason == TAEM_TRANSITION_EXCESS_ENERGY_S_TURN);

    double shell_max = 0.0;
    entry_taem_handoff_altitude_bounds(&cfg.guidance, NULL, &shell_max);
    t.mean_altitude = shell_max - 100.0;
    inputs = taem_exec_inputs_live(&g, &t, &cfg.guidance);
    assert(inputs.nominal_terminal_path_feasible);
    observation.ut += 1.0;
    assert(taem_exec_update(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(exec.last_transition_reason == TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE);
}

static void test_coarse_production_signals_cannot_bypass_terminal_contract(void) {
    LandingConfiguration cfg = landing_configuration_default();
    TaemExecProfile profile = taem_exec_profile_production(&cfg.guidance);

    /* Reproduce the old production boundary: every coarse geometry/phase signal
       says "go", but no numeric terminal contract accompanies it. Regardless of
       how guidance.c later learns to publish a real contract, this legacy shape
       must never authorize final-intercept/final-approach delivery by itself. */
    TaemExecInputs inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.mm304_complete = true;
    inputs.energy_valid = true;
    inputs.energy_excess = 0.0;
    inputs.terminal_feasibility_valid = true;
    inputs.nominal_terminal_path_feasible = true;
    inputs.terminal_path_selected = true;
    inputs.terminal_path_captured = true;
    inputs.final_intercept_ready = true;
    inputs.final_approach_ready = true;
    assert(!inputs.terminal_contract.valid);

    TaemExecObservation observation = {
        .ut = 2000.0,
        .relative_velocity = 420.0,
        .checkpoint_restart = false,
    };
    TaemExecutive exec;
    taem_exec_reset(&exec);
    assert(taem_exec_initialize(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_PATH_ACQUISITION);

    observation.ut += 1.0;
    assert(taem_exec_update(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);

    observation.ut += 1.0;
    assert(taem_exec_update(&exec, &observation, &inputs, &profile));
    assert(exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(!exec.taem_complete);
    assert(exec.terminal_evaluation.block_reason == TAEM_TERMINAL_BLOCK_CONTRACT_INVALID);
}

static PlanetModel terminal_contract_test_planet(void) {
    PlanetModel p;
    memset(&p, 0, sizeof(p));
    p.radius = 600000.0;
    p.gravitational_parameter = 3.5316e12;
    p.rotational_speed = 2.0 * LANDER_PI / 21549.425;
    p.north_axis = v3(0, 0, 1);
    p.prime_meridian_at_epoch = v3(1, 0, 0);
    return p;
}

static Telemetry terminal_contract_test_state(const LandingConfiguration *cfg) {
    Telemetry t;
    memset(&t, 0, sizeof(t));
    const double distance = 7000.0;
    const double slope = cfg->guidance.final_glide_slope;
    const double height = distance * tan(slope * DEG2RAD);
    t.ut = 4000.0;
    t.true_air_speed = 170.0;
    t.horizontal_speed = 165.0;
    t.vertical_speed = -t.horizontal_speed * tan(slope * DEG2RAD);
    t.mean_altitude = cfg->site.altitude + height;
    t.radar_altitude = height;
    t.runway_along_track = -distance;
    t.runway_cross_track = 0.0;
    t.range_to_site = distance;
    t.flight_path_angle = -slope;
    t.angle_of_attack = 16.0;
    t.roll = 0.0;
    t.roll_rate = 0.0;
    t.dynamic_pressure = 10000.0;
    t.mass = 40000.0;
    t.drag_force = 40000.0;
    t.physics_airbrake_model_available = true;
    t.physics_airbrake_model_confidence = 0.8;
    t.physics_airbrake_drag_accel = 6.0;
    t.physics_certified_uncertainty = 0.15;
    return t;
}

static TerminalPreflarePlan terminal_contract_test_plan(void) {
    TerminalPreflarePlan plan = {0};
    plan.feasible = true;
    plan.trigger_altitude = 700.0;
    plan.target_aoa = 18.0;
    plan.target_sink = -9.0;
    plan.minimum_speed = 125.0;
    plan.reference_speed = 145.0;
    plan.predicted_height_loss = 450.0;
    plan.predicted_kinetic_margin = 5000.0;
    plan.effective_accel = 1.0;
    plan.response_delay = 1.2;
    plan.response_aoa_rate = 2.0;
    return plan;
}

static void test_live_terminal_contract_requires_measured_response_and_real_speedbrake(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = terminal_contract_test_planet();
    Telemetry t = terminal_contract_test_state(&cfg);
    TerminalPreflarePlan plan = terminal_contract_test_plan();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.terminal_path_committed = true;
    g.hac_side_selected = true;
    g.hac_captured = true;
    g.hac_completed = true;
    g.terminal_energy_loss_accel_ema = 1.0;

    TaemTerminalContract contract = terminal_delivery_contract(&g, &t, &p, &cfg, &plan, false);
    assert(contract.valid && contract.path_committed);
    assert(contract.range_margin > 0.0);
    assert(contract.dynamic_pressure_margin > 0.0);
    assert(contract.specific_energy_margin > 0.0);
    assert(contract.response_time_available > contract.response_time_required);
    assert(contract.speedbrake_required);
    TaemTerminalEvaluation evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && !evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE);

    g.terminal_sink_accel_ema = 0.8;
    contract = terminal_delivery_contract(&g, &t, &p, &cfg, &plan, false);
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_NONE);

    t.physics_airbrake_model_available = false;
    contract = terminal_delivery_contract(&g, &t, &p, &cfg, &plan, false);
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && !evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_SPEEDBRAKE);

    /* A nearby deployed-airbrake cell is not sufficient evidence by itself:
       its conservative drag magnitude must actually dispose of excess energy. */
    t.physics_airbrake_model_available = true;
    t.physics_airbrake_drag_accel = 2.0;
    contract = terminal_delivery_contract(&g, &t, &p, &cfg, &plan, false);
    evaluation = taem_exec_evaluate_terminal_contract(&contract);
    assert(evaluation.valid && !evaluation.feasible);
    assert(evaluation.block_reason == TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY);
}

static void test_production_sync_delivers_final_only_after_executive_contract(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = terminal_contract_test_planet();
    Telemetry t = terminal_contract_test_state(&cfg);
    TerminalPreflarePlan plan = terminal_contract_test_plan();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.entry_exec.entry_complete = true;
    g.terminal_path_committed = true;
    g.hac_side_selected = true;
    g.hac_captured = true;
    g.hac_completed = true;
    g.terminal_energy_loss_accel_ema = 1.0;
    g.terminal_sink_accel_ema = 0.8;

    TaemTerminalContract contract = terminal_delivery_contract(&g, &t, &p, &cfg, &plan, false);
    assert(taem_exec_evaluate_terminal_contract(&contract).feasible);
    TaemExecInputs inputs = taem_exec_inputs_live_with_contract(&g, &t, &cfg.guidance,
        &contract, false, false);
    TaemExecObservation observation = {
        .ut = t.ut,
        .relative_velocity = t.true_air_speed,
        .checkpoint_restart = false,
    };
    TaemExecProfile profile = taem_exec_profile_production(&cfg.guidance);
    assert(taem_exec_initialize(&g.taem_exec, &observation, &inputs, &profile));
    assert(g.taem_exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(!g.taem_exec.taem_complete);

    t.ut += 1.0;
    assert(taem_exec_sync_with_contract(&g, &t, &cfg.guidance, &contract, false, false));
    assert(g.taem_exec.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);
    assert(!g.taem_exec.taem_complete);
    t.ut += 1.0;
    assert(taem_exec_sync_with_contract(&g, &t, &cfg.guidance, &contract, false, false));
    assert(g.taem_exec.phase == TAEM_PHASE_FINAL_INTERCEPT);
    assert(!g.taem_exec.taem_complete);
    t.ut += 1.0;
    assert(taem_exec_sync_with_contract(&g, &t, &cfg.guidance, &contract, true, true));
    assert(g.taem_exec.taem_complete);
    assert(g.taem_exec.last_transition_reason == TAEM_TRANSITION_FINAL_APPROACH_DELIVERY);
}

int main(void) {
    test_production_profile_can_select_optional_high_energy_s_turn();
    test_production_high_energy_preview_cannot_skip_s_turn_above_shell();
    test_recorded_20260910_140550_handoff_keeps_high_energy_s_turn();
    test_coarse_production_signals_cannot_bypass_terminal_contract();
    test_live_terminal_contract_requires_measured_response_and_real_speedbrake();
    test_production_sync_delivers_final_only_after_executive_contract();
    puts("TAEM production integration contract tests passed.");
    return 0;
}
