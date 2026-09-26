#include "landing_api.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static PlanetModel test_planet(void) {
    PlanetModel p = {0};
    p.radius = 600000.0;
    p.gravitational_parameter = 3.5316e12;
    p.rotational_speed = 0.000291570900559802;
    p.atmosphere_depth = 70000.0;
    p.surface_density = 1.1399;
    p.north_axis = (Vector3){0.0, 0.0, 1.0};
    p.prime_meridian_at_epoch = (Vector3){1.0, 0.0, 0.0};
    return p;
}

int main(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel planet = test_planet();
    EntryPrediction entry = {0};
    entry.entered_atmosphere = true;
    entry.entry_range = 500000.0;
    entry.entry_flight_path_angle = -5.0;
    entry.peak_dynamic_pressure = 30000.0;
    entry.peak_g_load = 2.0;
    entry.mm304_gate_recorded = false;
    entry.reached_taem = false;
    entry.shadow_aborted = true;
    cfg.guidance.maximum_entry_flight_path_angle = -8.0;
    cfg.vehicle.maximum_dynamic_pressure = 100000.0;
    cfg.vehicle.maximum_g_load = 5.0;

    /* A later downstream shadow abort must not veto a safe orbital-stage entry. */
    assert(deorbit_entry_stage_qualified(&entry, &cfg.vehicle, &cfg.guidance,
        30000.0, 0.0, planet.atmosphere_depth, true));
    assert(!deorbit_runway_capture_qualified(&entry, &cfg.site, &cfg.vehicle,
        &cfg.guidance, 30000.0));
    assert(deorbit_entry_stage_robustness_qualified(13, 15, 0, 0.80));
    assert(!deorbit_entry_stage_robustness_qualified(12, 15, 0, 0.80));
    assert(!deorbit_entry_stage_robustness_qualified(15, 15, 1, 0.80));

    /* Each stage-local physical prerequisite independently rejects admission. */
    assert(!deorbit_entry_stage_qualified(&entry, &cfg.vehicle, &cfg.guidance,
        30000.0, 0.0, planet.atmosphere_depth, false));
    assert(!deorbit_entry_stage_qualified(&entry, &cfg.vehicle, &cfg.guidance,
        80000.0, 0.0, planet.atmosphere_depth, true));
    EntryPrediction invalid = entry;
    invalid.entered_atmosphere = false;
    assert(!deorbit_entry_stage_qualified(&invalid, &cfg.vehicle, &cfg.guidance,
        30000.0, 0.0, planet.atmosphere_depth, true));
    invalid = entry;
    invalid.entry_flight_path_angle = -10.0;
    assert(!deorbit_entry_stage_qualified(&invalid, &cfg.vehicle, &cfg.guidance,
        30000.0, 0.0, planet.atmosphere_depth, true));
    invalid = entry;
    invalid.peak_dynamic_pressure = cfg.vehicle.maximum_dynamic_pressure + 1.0;
    assert(!deorbit_entry_stage_qualified(&invalid, &cfg.vehicle, &cfg.guidance,
        30000.0, 0.0, planet.atmosphere_depth, true));
    invalid = entry;
    invalid.peak_g_load = cfg.vehicle.maximum_g_load + 1.0;
    assert(!deorbit_entry_stage_qualified(&invalid, &cfg.vehicle, &cfg.guidance,
        30000.0, 0.0, planet.atmosphere_depth, true));

    /* The same stage-qualified, non-captured plan enters normal orbital guidance. */
    DeorbitPlan plan = {0};
    plan.execution_qualified = true;
    plan.entry_stage_qualified = true;
    plan.execution_degraded = true;
    plan.target_capture_achieved = false;
    plan.burn_ut = 200.0;
    plan.delta_v = 100.0;
    plan.estimated_burn_duration = 20.0;
    plan.predicted_post_burn_periapsis_altitude = 30000.0;
    Telemetry t;
    telemetry_init(&t);
    t.ut = 100.0;
    t.mean_altitude = 86000.0;
    t.vertical_speed = 0.0;
    t.periapsis_altitude = 30000.0;
    t.mass = 50000.0;
    t.heading = 90.0;
    Vector3 position = {planet.radius + t.mean_altitude, 0.0, 0.0};
    Vector3 velocity = {0.0, 0.0, 2200.0};
    VehicleState state = {t.ut, position, velocity, t.mass};
    GuidanceMachine guidance;
    guidance_machine_init(&guidance);
    GuidanceResult result = guidance_update(&guidance, &t, &state, &plan,
        &planet, (AerodynamicModel){0}, &cfg);
    assert(result.phase == PHASE_IDLE);
    assert(strstr(result.status, "Entry-stage-qualified") != NULL);
    guidance_result_clear(&result);
    guidance_set_engaged(&guidance, true);
    result = guidance_update(&guidance, &t, &state, &plan,
        &planet, (AerodynamicModel){0}, &cfg);
    assert(result.phase == PHASE_COAST);
    assert(result.command.autopilot_engaged);
    guidance_result_clear(&result);

    puts("Deorbit stage admission tests passed.");
    return 0;
}
