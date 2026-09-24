#include "entry_exec.h"
#include "taem_exec.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static EntryExecProfile entry_profile(void) {
    EntryExecProfile p;
    memset(&p, 0, sizeof(p));
    p.has_temperature_velocity_gate = true;
    p.temperature_end_velocity = 2200.0;
    p.equilibrium_intercept_valid = true;
    p.equilibrium_intercept = false;
    p.has_constant_drag_velocity_gate = true;
    p.constant_drag_end_velocity = 1450.0;
    return p;
}

static EntryExecObservation entry_obs(double ut, double velocity, double dynamic_pressure,
                                      bool equilibrium_intercept) {
    (void)equilibrium_intercept;
    EntryExecObservation o;
    memset(&o, 0, sizeof(o));
    o.ut = ut;
    o.relative_velocity = velocity;
    o.altitude = 50000.0;
    o.dynamic_pressure = dynamic_pressure;
    return o;
}

static TaemExecInputs nominal_taem_inputs(bool mm304_complete) {
    TaemExecInputs i;
    memset(&i, 0, sizeof(i));
    i.mm304_complete = mm304_complete;
    i.terminal_path_selected = true;
    i.terminal_path_captured = false;
    return i;
}

static void test_mm304_to_mm305_is_one_way(void) {
    EntryExecutive entry;
    entry_exec_reset(&entry);
    EntryExecProfile ep = entry_profile();

    EntryExecObservation eo = entry_obs(100.0, 2500.0, 0.0, false);
    assert(entry_exec_initialize(&entry, &eo, &ep));
    assert(entry.phase == ENTRY_PHASE_PREENTRY);
    assert(!entry.entry_complete);

    eo = entry_obs(110.0, 2300.0, 0.10, false);
    assert(entry_exec_update(&entry, &eo, &ep));
    assert(entry.phase == ENTRY_PHASE_TEMPERATURE_CONTROL);

    eo = entry_obs(120.0, 2100.0, 0.10, false);
    assert(entry_exec_update(&entry, &eo, &ep));
    assert(entry.phase == ENTRY_PHASE_EQUILIBRIUM_GLIDE);

    ep.equilibrium_intercept = true;
    eo = entry_obs(130.0, 1650.0, 0.10, true);
    assert(entry_exec_update(&entry, &eo, &ep));
    assert(entry.phase == ENTRY_PHASE_CONSTANT_DRAG);

    eo = entry_obs(140.0, 1400.0, 0.10, true);
    assert(entry_exec_update(&entry, &eo, &ep));
    assert(entry.phase == ENTRY_PHASE_TRANSITION);
    assert(!entry.entry_complete);

    eo = entry_obs(150.0, 1290.0, 0.10, true);
    assert(entry_exec_update(&entry, &eo, &ep));
    assert(!entry.entry_complete);
    assert(entry_exec_accept_taem_handoff(&entry, &eo));
    assert(entry.entry_complete);

    TaemExecutive taem;
    taem_exec_reset(&taem);
    TaemExecInputs ti = nominal_taem_inputs(entry.entry_complete);
    TaemExecObservation to = {
        .ut = eo.ut,
        .relative_velocity = eo.relative_velocity,
        .checkpoint_restart = false,
    };
    assert(taem_exec_initialize(&taem, &to, &ti));
    assert(taem_exec_owns_vehicle(&taem));
    assert(taem.phase == TAEM_PHASE_PATH_ACQUISITION);

    /* A later/stale MM304-complete sample may disappear during predictor refresh,
       but MM305 ownership must remain latched.  Terminal-path loss is handled by
       TAEM itself; with terminal path loss it returns to TAEM path acquisition replan rather
       than oscillating back to Entry or freezing in a stale alignment state. */
    ti.mm304_complete = false;
    ti.terminal_path_selected = true;
    ti.terminal_path_captured = true;
    to.ut += 1.0;
    assert(taem_exec_update(&taem, &to, &ti));
    assert(taem_exec_owns_vehicle(&taem));
    assert(taem.phase == TAEM_PHASE_RUNWAY_ALIGNMENT);

    ti.terminal_path_selected = false;
    ti.terminal_path_captured = false;
    to.ut += 0.1;
    assert(taem_exec_update(&taem, &to, &ti));
    assert(taem_exec_owns_vehicle(&taem));
    assert(taem.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(taem.last_transition_reason == TAEM_TRANSITION_TERMINAL_PATH_REPLAN);
}

static void test_taem_handoff_latched(void) {
    TaemExecutive taem;
    taem_exec_reset(&taem);
    TaemExecObservation o = {
        .ut = 200.0,
        .relative_velocity = 1290.0,
        .checkpoint_restart = false,
    };

    TaemExecInputs inputs = nominal_taem_inputs(true);
    assert(taem_exec_initialize(&taem, &o, &inputs));
    assert(taem.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(taem_exec_owns_vehicle(&taem));
}

int main(void) {
    test_mm304_to_mm305_is_one_way();
    test_taem_handoff_latched();
    puts("Shuttle architecture contract tests passed.");
    return 0;
}
