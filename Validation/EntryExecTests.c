#include "entry_exec.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static EntryExecObservation obs(double ut, double velocity, double altitude, double dynamic_pressure) {
    EntryExecObservation value = {
        .ut = ut,
        .relative_velocity = velocity,
        .altitude = altitude,
        .dynamic_pressure = dynamic_pressure,
        .checkpoint_restart = false,
    };
    return value;
}

static EntryExecProfile profile_default(void) {
    EntryExecProfile profile;
    memset(&profile, 0, sizeof(profile));
    profile.has_temperature_velocity_gate = true;
    profile.temperature_end_velocity = 1800.0;
    profile.equilibrium_intercept_valid = true;
    profile.equilibrium_intercept = false;
    profile.has_constant_drag_velocity_gate = true;
    profile.constant_drag_end_velocity = 1000.0;
    return profile;
}

static void test_normal_five_phase_progression(void) {
    EntryExecutive exec;
    entry_exec_reset(&exec);
    EntryExecProfile profile = profile_default();
    EntryExecObservation state = obs(100.0, 2400.0, 70000.0, 0.2);

    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_PREENTRY);
    assert(!exec.entry_complete);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_ENTRY_START);
    assert(exec.transition_count == 0);

    state = obs(101.0, 2350.0, 68000.0, 1.6);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TEMPERATURE_CONTROL);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_ATMOSPHERIC_CONTACT);

    state = obs(140.0, 1790.0, 50000.0, 2.0);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_EQUILIBRIUM_GLIDE);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_TEMPERATURE_VELOCITY);

    profile.equilibrium_intercept = true;
    state = obs(160.0, 1500.0, 41000.0, 2.1);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_CONSTANT_DRAG);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT);

    profile.equilibrium_intercept = false;
    state = obs(190.0, 990.0, 30000.0, 1.8);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY);

    state = obs(220.0, 590.0, 18000.0, 1.4);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(!exec.entry_complete);
    assert(exec.transition_count == 4);
    assert(entry_exec_accept_taem_handoff(&exec, &state));
    assert(exec.entry_complete);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF);
    assert(exec.last_transition_completed_entry);
    assert(exec.last_transition_ut == 220.0);
    assert(exec.last_transition_velocity == 590.0);
    assert(exec.transition_count == 5);

    EntryExecTelemetry telemetry = entry_exec_telemetry(&exec);
    assert(telemetry.active_phase == ENTRY_PHASE_TRANSITION);
    assert(telemetry.entry_complete);
    assert(telemetry.last_transition_from == ENTRY_PHASE_TRANSITION);
    assert(telemetry.last_transition_to == ENTRY_PHASE_TRANSITION);
    assert(telemetry.last_transition_completed_entry);
    assert(strcmp(entry_phase_string(ENTRY_PHASE_EQUILIBRIUM_GLIDE), "Equilibrium Glide") == 0);
    assert(strcmp(entry_transition_reason_string(ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF),
                  "Qualified TAEM handoff") == 0);
}


static void test_nominal_phase_never_regresses(void) {
    EntryExecutive exec;
    entry_exec_reset(&exec);
    EntryExecProfile profile = profile_default();
    profile.has_phase_floor = true;
    profile.phase_floor = ENTRY_PHASE_EQUILIBRIUM_GLIDE;
    EntryExecObservation state = obs(30.0, 1700.0, 45000.0, 1.8);
    state.checkpoint_restart = true;
    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_EQUILIBRIUM_GLIDE);

    profile.has_phase_floor = true;
    profile.phase_floor = ENTRY_PHASE_PREENTRY;
    profile.equilibrium_intercept = false;
    state = obs(31.0, 2200.0, 46000.0, 0.4);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_EQUILIBRIUM_GLIDE);

    profile.equilibrium_intercept = true;
    state = obs(32.0, 1600.0, 43000.0, 1.7);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_CONSTANT_DRAG);

    profile.equilibrium_intercept = false;
    state = obs(33.0, 1900.0, 44000.0, 1.0);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_CONSTANT_DRAG);
}

static void test_checkpoint_restart_classification(void) {
    EntryExecutive exec;
    EntryExecProfile profile = profile_default();

    entry_exec_reset(&exec);
    profile.has_phase_floor = true;
    profile.phase_floor = ENTRY_PHASE_TEMPERATURE_CONTROL;
    EntryExecObservation state = obs(50.0, 2200.0, 50000.0, 1.0);
    state.checkpoint_restart = true;
    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TEMPERATURE_CONTROL);
    assert(!exec.entry_complete);

    entry_exec_reset(&exec);
    profile.phase_floor = ENTRY_PHASE_CONSTANT_DRAG;
    state = obs(60.0, 1250.0, 30000.0, 1.9);
    state.checkpoint_restart = true;
    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_CONSTANT_DRAG);
    assert(!exec.entry_complete);

    entry_exec_reset(&exec);
    profile.phase_floor = ENTRY_PHASE_TRANSITION;
    state = obs(70.0, 850.0, 22000.0, 1.6);
    state.checkpoint_restart = true;
    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(!exec.entry_complete);

    entry_exec_reset(&exec);
    profile.has_phase_floor = false;
    state = obs(80.0, 550.0, 15000.0, 1.4);
    state.checkpoint_restart = true;
    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(!exec.entry_complete);
    assert(!exec.last_transition_completed_entry);
}

static void test_invalid_inputs_are_transactional(void) {
    EntryExecutive exec;
    entry_exec_reset(&exec);
    EntryExecProfile profile = profile_default();
    EntryExecObservation state = obs(1.0, 2300.0, 65000.0, 0.2);
    assert(entry_exec_initialize(&exec, &state, &profile));
    EntryExecutive before = exec;

    state.relative_velocity = NAN;
    assert(!entry_exec_update(&exec, &state, &profile));
    assert(memcmp(&before, &exec, sizeof(exec)) == 0);

    state = obs(2.0, 2200.0, NAN, 0.4);
    assert(!entry_exec_update(&exec, &state, &profile));
    assert(memcmp(&before, &exec, sizeof(exec)) == 0);

    state = obs(2.0, 2200.0, 62000.0, NAN);
    assert(!entry_exec_update(&exec, &state, &profile));
    assert(memcmp(&before, &exec, sizeof(exec)) == 0);

    state = obs(2.0, 2200.0, 62000.0, 0.4);
    profile.temperature_end_velocity = NAN;
    assert(!entry_exec_update(&exec, &state, &profile));
    assert(memcmp(&before, &exec, sizeof(exec)) == 0);

    entry_exec_reset(&exec);
    state = obs(3.0, -1.0, 60000.0, 0.4);
    profile = profile_default();
    assert(!entry_exec_initialize(&exec, &state, &profile));
    assert(!exec.initialized);
    assert(exec.phase == ENTRY_PHASE_PREENTRY);
}

static void test_entry_complete_is_latched(void) {
    EntryExecutive exec;
    entry_exec_reset(&exec);
    EntryExecProfile profile = profile_default();
    profile.has_phase_floor = true;
    profile.phase_floor = ENTRY_PHASE_TRANSITION;
    EntryExecObservation state = obs(90.0, 590.0, 17000.0, 1.5);
    state.checkpoint_restart = true;
    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(!exec.entry_complete);
    assert(entry_exec_accept_taem_handoff(&exec, &state));
    assert(exec.entry_complete);
    EntryExecutive before = exec;

    profile.equilibrium_intercept = true;
    profile.temperature_end_velocity = 3000.0;
    state = obs(100.0, 900.0, 19000.0, 1.0);
    assert(entry_exec_update(&exec, &state, &profile));
    assert(memcmp(&before, &exec, sizeof(exec)) == 0);
}

static void test_transition_velocity_cannot_complete_entry(void) {
    EntryExecutive exec;
    entry_exec_reset(&exec);
    EntryExecProfile profile = profile_default();
    profile.has_phase_floor = true;
    profile.phase_floor = ENTRY_PHASE_TRANSITION;
    EntryExecObservation state = obs(200.0, 590.0, 18000.0, 1.5);
    state.checkpoint_restart = true;

    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(!exec.entry_complete);

    state.checkpoint_restart = false;
    state.ut = 201.0;
    state.relative_velocity = 300.0;
    assert(entry_exec_update(&exec, &state, &profile));
    assert(!exec.entry_complete);
    assert(!exec.last_transition_completed_entry);

    assert(entry_exec_accept_taem_handoff(&exec, &state));
    assert(exec.entry_complete);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF);
}


static void test_qualified_taem_handoff_completes_entry(void) {
    EntryExecutive exec;
    entry_exec_reset(&exec);
    EntryExecProfile profile = profile_default();
    profile.has_phase_floor = true;
    profile.phase_floor = ENTRY_PHASE_TRANSITION;
    EntryExecObservation state = obs(300.0, 700.0, 21000.0, 1.5);
    state.checkpoint_restart = true;

    assert(entry_exec_initialize(&exec, &state, &profile));
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(!exec.entry_complete);

    state.checkpoint_restart = false;
    state.ut = 301.0;
    state.relative_velocity = 610.0;
    assert(entry_exec_accept_taem_handoff(&exec, &state));
    assert(exec.entry_complete);
    assert(exec.phase == ENTRY_PHASE_TRANSITION);
    assert(exec.last_transition_reason == ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF);
    assert(exec.last_transition_completed_entry);
    assert(exec.last_transition_ut == 301.0);
    assert(exec.last_transition_velocity == 610.0);
    assert(strcmp(entry_transition_reason_string(ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF),
                  "Qualified TAEM handoff") == 0);
}
int main(void) {
    test_normal_five_phase_progression();
    test_nominal_phase_never_regresses();
    test_checkpoint_restart_classification();
    test_invalid_inputs_are_transactional();
    test_entry_complete_is_latched();
    test_transition_velocity_cannot_complete_entry();
    test_qualified_taem_handoff_completes_entry();
    puts("Entry executive tests passed.");
    return 0;
}
