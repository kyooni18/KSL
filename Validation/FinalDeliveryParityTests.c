#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "guidance_internal.h"
#include "mm305_planning.h"
#include "sim_telemetry.h"

/* Measured Sep 28 MM305 handoff neighborhood; no KSP connection or flight. */
static void test_touchdown_forecast_handoff_parity(void) {
    LandingConfiguration cfg=landing_configuration_default();
    cfg.site.altitude=70.0;
    cfg.site.runway_heading=90.0;
    cfg.vehicle.maximum_angle_of_attack=28.0;
    cfg.vehicle.terminal_maximum_lift_angle_of_attack=15.0;
    cfg.vehicle.touchdown_speed=75.0;
    cfg.guidance.final_approach_distance=9000.0;
    cfg.guidance.final_glide_slope=atan2(4000.0,9000.0)*RAD2DEG;
    cfg.guidance.final_alignment_speed=165.0;
    PlanetModel planet={0};
    char path[512],reason[256];
    assert(shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,path,sizeof(path)));
    assert(shuttle_sim_load_planet(path,&planet,reason,sizeof(reason)));
    Telemetry t={0};
    t.ut=22009.65;t.mean_altitude=4188.6;t.radar_altitude=4013.5;
    t.mass=43515.77;t.true_air_speed=168.86;
    t.flight_path_angle=-24.24;t.angle_of_attack=4.89;
    t.vertical_speed=-t.true_air_speed*sin(24.24*DEG2RAD);
    t.horizontal_speed=t.true_air_speed*cos(24.24*DEG2RAD);
    t.runway_along_track=-9239.6;t.runway_cross_track=16.7;
    t.heading=t.ground_track_heading=89.1;
    t.mach=.50976;t.dynamic_pressure=9524.39;t.atmospheric_density=.66804;
    t.lift_force=385162.4;t.drag_force=218940.2;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.terminal_final_handoff_latched=true;
    g.terminal_final_handoff_distance=9000.0;
    g.terminal_final_handoff_slope_deg=cfg.guidance.final_glide_slope;
    g.terminal_final_handoff_speed_mps=165.0;
    g.terminal_preflare_plan_valid=true;
    g.preflare_trigger_altitude=2229.6;
    g.terminal_vertical_stage=TERMINAL_OUTER_FINAL;
    g.terminal_vertical_stage_valid=true;

    /* Cold handoff and a handoff retaining MM305's observed aero history. */
    for(int pass=0;pass<2;++pass){
        GuidanceMachine before=g;
        double predicted=terminal_delivery_touchdown_speed(&g,&t,&planet,&cfg,
            g.preflare_trigger_altitude);
        assert(isfinite(predicted)&&predicted>0.0);
        assert(memcmp(&before,&g,sizeof(g))==0); /* forecast is read-only */
        Trajectory reference;
        trajectory_init(&reference);
        AerodynamicModel aero={.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=1};
        (void)terminal_approach_sequence(&g,&t,89.1,&planet,aero,&cfg,&reference,.1);
        trajectory_clear(&reference);
        assert(!g.terminal_pull_latch);
        assert(fabs(predicted-g.terminal_predicted_touchdown_speed)<1e-9);
        double lift_area=g.terminal_lift_area_ema,drag_area=g.terminal_drag_area_ema;
        terminal_observe_landing_aero(&g,&t,&cfg.vehicle,planet_surface_gravity(&planet),.1);
        assert(g.terminal_lift_area_ema==lift_area&&g.terminal_drag_area_ema==drag_area);
        if(pass==0){
            for(int tick=0;tick<20;++tick){
                t.ut+=.1;
                terminal_observe_landing_aero(&g,&t,&cfg.vehicle,
                    planet_surface_gravity(&planet),.1);
            }
        }
    }
    t.true_air_speed=NAN;
    assert(isnan(terminal_delivery_touchdown_speed(&g,&t,&planet,&cfg,2229.6)));
    t.true_air_speed=168.86;
    t.mean_altitude=NAN;
    assert(isnan(terminal_delivery_touchdown_speed(&g,&t,&planet,&cfg,2229.6)));
}

static void test_plan_age_is_qualification_age(void) {
    Mm305PlanResult result={.valid=true,.request_ut=100.0};
    result.initial_state.ut_s=100.0;
    Telemetry telemetry={.ut=100.5};
    assert(mm305_plan_result_fresh(&result,&telemetry));
    telemetry.ut=111.0; /* observed live search latency */
    assert(!mm305_plan_result_fresh(&result,&telemetry));
    result.request_ut=result.initial_state.ut_s=110.5; /* requalified state */
    assert(mm305_plan_result_fresh(&result,&telemetry));
    telemetry.ut=110.0;
    assert(!mm305_plan_result_fresh(&result,&telemetry));
    result.request_ut=NAN;
    assert(!mm305_plan_result_fresh(&result,&telemetry));

    /* A geometric-only planner result cannot bypass the Final delivery gate. */
    GuidanceMachine guidance;
    guidance_machine_init(&guidance);
    guidance.phase=PHASE_TAEM;
    LandingConfiguration cfg=landing_configuration_default();
    result.found=true;
    result.candidate.status=TAEM_PLAN_UNQUALIFIED;
    result.candidate.route_built=true;
    result.candidate.route.valid=true;
    result.candidate.route.alignment_along_m=-9000.0;
    result.candidate.route.profile_final_slope_deg=24.0;
    result.candidate.replay.path_constraints_ok=true;
    assert(!guidance_mm305_accept_plan(&guidance,&result,&cfg));
    assert(!guidance.mm305_route_committed);
    result.candidate.replay.final_interface_qualified=true;
    assert(guidance_mm305_accept_plan(&guidance,&result,&cfg));
    assert(guidance.mm305_route_committed);
}

int main(void) {
    test_touchdown_forecast_handoff_parity();
    test_plan_age_is_qualification_age();
    puts("Final forecast handoff parity and plan freshness tests passed.");
    return 0;
}
