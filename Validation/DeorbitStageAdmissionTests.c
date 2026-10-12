#include "landing_api.h"

#include <assert.h>
#include <math.h>
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

    /* Fourth STS polar live flight: the reduced shadow made a forced TAEM
       transfer 0.5 km from KSC, around 173 deg heading, with a very broad
       100 km orbital capture tolerance. Old runway qualification accepted
       this as a successful inlet despite leaving no HAC/final maneuver room. */
    EntryPrediction overflight = entry;
    overflight.mm304_gate_recorded = true;
    overflight.reached_taem = true;
    overflight.shadow_aborted = false;
    overflight.minimum_entry_speed = 310.0;
    overflight.taem_speed = 872.0;
    overflight.closest_distance = 485.0;
    overflight.taem_distance = 485.0;
    overflight.taem_range_error = 485.0 - 96500.0;
    overflight.taem_interface_error = 0.0; /* Former forced-boundary lie. */
    overflight.taem_course = 173.0;
    cfg.guidance.taem_interface_range = 96500.0;
    cfg.guidance.target_deorbit_capture_radius = 100000.0;
    cfg.guidance.mm304_perpendicular_heading_half_width = 30.0;
    cfg.guidance.final_approach_distance = 8000.0;
    cfg.guidance.hac_radius = 12000.0;
    assert(!deorbit_runway_capture_qualified(&overflight, &cfg.site,
        &cfg.vehicle, &cfg.guidance, 50000.0));
    overflight.taem_distance = 70000.0;
    overflight.taem_range_error = 70000.0 - 96500.0;
    assert(deorbit_runway_capture_qualified(&overflight, &cfg.site,
        &cfg.vehicle, &cfg.guidance, 50000.0));
    overflight.taem_ownership_boundary_missed = true;
    assert(!deorbit_runway_capture_qualified(&overflight, &cfg.site,
        &cfg.vehicle, &cfg.guidance, 50000.0));

    assert(deorbit_entry_stage_robustness_qualified(13, 15, 0, 0.80));
    /* Exactly the required fraction qualifies (1e-9 tolerance for 12/15 == 0.80). */
    assert(deorbit_entry_stage_robustness_qualified(12, 15, 0, 0.80));
    assert(!deorbit_entry_stage_robustness_qualified(11, 15, 0, 0.80));
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

    /* A measured Pe still 2 km above target must not suffer the former
       8 km throttle taper that stretched a 118 s planned burn to 215 s in
       the fourth live polar flight. Near the 300 m cutoff retain a smooth
       ramp instead of holding maximum throttle through the endpoint. */
    guidance_reset_plan(&guidance);
    guidance_set_engaged(&guidance, true);
    t.ut = 205.0;
    t.mass = 26000.0;
    t.available_thrust = 60000.0;
    t.current_thrust = 0.0;
    t.autopilot_error = 0.0;
    t.periapsis_altitude = plan.predicted_post_burn_periapsis_altitude + 2000.0;
    guidance.has_burn_command_started = true;
    guidance.burn_active_elapsed = 20.0;
    result = guidance_update(&guidance, &t, &state, &plan,
        &planet, (AerodynamicModel){0}, &cfg);
    assert(result.phase == PHASE_DEORBIT_BURN);
    assert(fabs(result.command.target_throttle - cfg.guidance.deorbit_maximum_throttle) < 1e-6);
    guidance_result_clear(&result);
    t.ut += 0.1;
    t.periapsis_altitude = plan.predicted_post_burn_periapsis_altitude + 700.0;
    result = guidance_update(&guidance, &t, &state, &plan,
        &planet, (AerodynamicModel){0}, &cfg);
    assert(result.phase == PHASE_DEORBIT_BURN);
    assert(result.command.target_throttle > 0.01);
    assert(result.command.target_throttle < cfg.guidance.deorbit_maximum_throttle * 0.5);
    guidance_result_clear(&result);

    /* A clock correction and one zero-thrust sample are not sustained loss.
       Recovering thrust resets the watch; a genuinely failed engine still aborts. */
    guidance_reset_plan(&guidance);
    guidance_set_engaged(&guidance,true);
    t.ut=200.0;t.periapsis_altitude=100000.0;
    t.mass=26000.0;t.available_thrust=60000.0;t.autopilot_error=0.0;
    cfg.guidance.deorbit_throttle_ramp_duration=1.0;
    for(unsigned i=0;i<10;i++){
        t.ut+=0.1;t.current_thrust=6000.0;
        result=guidance_update(&guidance,&t,&state,&plan,&planet,(AerodynamicModel){0},&cfg);
        assert(result.phase==PHASE_DEORBIT_BURN);
        guidance_result_clear(&result);
    }
    t.ut+=20.0;t.current_thrust=0.0;
    result=guidance_update(&guidance,&t,&state,&plan,&planet,(AerodynamicModel){0},&cfg);
    assert(result.phase==PHASE_DEORBIT_BURN);
    guidance_result_clear(&result);
    t.ut+=0.1;t.current_thrust=6000.0;
    result=guidance_update(&guidance,&t,&state,&plan,&planet,(AerodynamicModel){0},&cfg);
    assert(result.phase==PHASE_DEORBIT_BURN&&guidance.burn_thrust_loss_elapsed==0.0);
    guidance_result_clear(&result);
    for(unsigned i=0;i<70&&!guidance.aborted;i++){
        t.ut+=0.1;t.current_thrust=0.0;
        result=guidance_update(&guidance,&t,&state,&plan,&planet,(AerodynamicModel){0},&cfg);
        guidance_result_clear(&result);
    }
    assert(guidance.aborted);
    guidance_reset_plan(&guidance);
    assert(guidance.burn_thrust_loss_elapsed==0.0);

    puts("Deorbit stage admission tests passed.");
    return 0;
}
