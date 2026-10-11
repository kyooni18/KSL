#include "../CLanding/guidance/guidance_internal.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void init_native_route_case(GuidanceMachine *g, Telemetry *t,
        double bank_target, double roll_rate) {
    guidance_machine_init(g);
    g->phase = PHASE_TAEM;
    g->mm305_route.valid = true;
    g->mm305_route_committed = true;
    g->roll_limiter.value = bank_target;
    g->roll_limiter.has_value = false; /* committed route bypasses limiter */
    telemetry_init(t);
    t->mass = 40000.0;
    t->dynamic_pressure = 4000.0;
    t->lift_force = t->mass * 4.0;
    t->drag_force = t->mass * 2.0;
    t->roll = 0.0;
    t->roll_rate = roll_rate;
    t->has_body_roll_rate = true;
    t->body_roll_rate = roll_rate;
    t->angle_of_attack = 18.0;
}

static void test_commanded_native_capture_is_not_recovery(void) {
    LandingConfiguration cfg = landing_configuration_default();
    for (int direction = -1; direction <= 1; direction += 2) {
        GuidanceMachine g;
        Telemetry t;
        double target = 69.0 * direction;
        double rate = 12.45 * direction;
        init_native_route_case(&g, &t, target, rate);
        for (int i = 0; i < 8; ++i) {
            t.ut += .06;
            t.roll += rate * .06;
            assert(!control_recovery_needed(&g, &t, &cfg.guidance,
                &cfg.vehicle, .06));
        }
        assert(!g.attitude_recovery);
        assert(!g.roll_limiter.has_value);
    }
}

static void test_uncommanded_energetic_reversals_still_trigger(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    Telemetry t;
    init_native_route_case(&g, &t, 69.0, 12.0);
    g.previous_relative_roll_rate = 12.0;
    g.has_previous_relative_roll_rate = true;
    t.roll = 20.0;
    t.roll_rate = -12.0; /* wrong-way motion relative to +69 deg target */
    t.body_roll_rate = -12.0;
    assert(!control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06));
    t.roll_rate = 12.0;
    t.body_roll_rate = 12.0;
    assert(control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06));
    assert(g.attitude_recovery);
}

static void test_rejected_landing_brakes_then_terminates_failed(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.site.latitude = cfg.site.longitude = 0.0;
    cfg.site.runway_heading = 90.0;
    cfg.site.runway_length = 2500.0;
    cfg.site.allow_reciprocal_runway = false;
    PlanetModel planet = {.radius=600000.0,
        .gravitational_parameter=3.5316e12, .north_axis={0,0,1}};
    VehicleState state = {.position={600070,0,0}, .velocity={0,20,0}};
    AerodynamicModel aero = {.lift_to_drag=3.8,
        .ballistic_coefficient=700, .confidence=1};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.automation_engaged = g.terminal_glide_mode = true;
    g.terminal_region_entered = g.final_approach_captured = true;
    g.ground_contact_latched = true;
    g.phase = PHASE_ROLLOUT;
    Telemetry t;
    telemetry_init(&t);
    t.longitude = 2646.0 / planet.radius * RAD2DEG;
    t.mean_altitude = 70.0;
    t.mass = 25123.0;
    t.gear = true;
    t.heading = t.ground_track_heading = 90.0;
    t.surface_speed = t.horizontal_speed = t.true_air_speed = 20.0;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"landed");
    GuidanceResult r = guidance_update(&g,&t,&state,NULL,&planet,aero,&cfg);
    assert(r.phase == PHASE_ROLLOUT && r.has_warning);
    assert(r.command.brakes && r.command.target_throttle == 0.0);
    assert(!g.aborted && g.terminal_landing_warning_latched);
    assert(strstr(r.warning,"outside the runway"));
    guidance_result_clear(&r);

    /* Invalid speed cannot prove that a moving failed rollout has stopped. */
    t.surface_speed = NAN;
    t.ut += .1;
    r = guidance_update(&g,&t,&state,NULL,&planet,aero,&cfg);
    assert(r.phase == PHASE_ROLLOUT && r.command.brakes && !g.aborted);
    guidance_result_clear(&r);

    /* Even if later telemetry returns inside the runway, failure is latched. */
    t.longitude = 2400.0 / planet.radius * RAD2DEG;
    t.surface_speed = t.horizontal_speed = t.true_air_speed = 1.0;
    t.ut += .1;
    r = guidance_update(&g,&t,&state,NULL,&planet,aero,&cfg);
    assert(r.phase == PHASE_ABORT && r.has_warning);
    assert(r.command.brakes && r.command.gear && !r.command.airbrakes);
    assert(g.aborted && !g.automation_engaged);
    assert(strstr(r.status,"outside the runway"));
    guidance_result_clear(&r);
    t.ut += .1;
    r = guidance_update(&g,&t,&state,NULL,&planet,aero,&cfg);
    assert(r.phase == PHASE_ABORT);
    guidance_result_clear(&r);
}

static void test_rollout_completion_uses_runway_bounds(void) {
    LandingConfiguration cfg=landing_configuration_default();
    cfg.site.runway_length=2500.0;cfg.site.runway_width=70.0;
    /* The historical live stop at 2556 m was within the contact envelope but
       beyond the runway. Edge tolerances must not qualify mission success. */
    const double along[]={2400.0,2556.0,-20.0,2400.0,2400.0,2400.0};
    const double cross[]={12.0,12.0,0.0,40.0,0.0,0.0};
    const double speed[]={1.0,1.0,1.0,1.0,NAN,-1.0};
    for(size_t i=0;i<sizeof(along)/sizeof(along[0]);++i){
        GuidanceMachine g;guidance_machine_init(&g);
        Telemetry t;telemetry_init(&t);
        t.runway_along_track=along[i];t.runway_cross_track=cross[i];
        t.surface_speed=speed[i];
        Trajectory ref;trajectory_init(&ref);
        GuidanceResult r=touchdown(&g,&t,&cfg,&ref);
        assert(r.phase==(i==0?PHASE_COMPLETE:PHASE_ABORT));
        guidance_result_clear(&r);trajectory_clear(&ref);
    }
}

int main(void) {
    test_commanded_native_capture_is_not_recovery();
    test_uncommanded_energetic_reversals_still_trigger();
    test_rejected_landing_brakes_then_terminates_failed();
    test_rollout_completion_uses_runway_bounds();
    puts("MM305 recovery monitor tests passed.");
    return 0;
}
