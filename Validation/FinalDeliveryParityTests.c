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
    cfg.site.runway_length=2500.0;
    cfg.site.runway_width=70.0;
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

    /* Final admission follows measured ground-course projection, not the
       attitude or the optimistic capture-time estimate. */
    AerodynamicModel gate_aero={.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=1};
    TerminalPreflarePlan gate_plan={0};
    assert(terminal_preflare_alignment_valid(&g,&t,90.0,&cfg));
    assert(terminal_outer_capture_admissible(&g,&t,90.0,&planet,gate_aero,
        &cfg,&gate_plan));
    Telemetry divergent=t;
    divergent.runway_cross_track=6000.0;
    assert(!terminal_preflare_alignment_valid(&g,&divergent,100.0,&cfg));
    assert(!terminal_preflare_alignment_valid(&g,&divergent,307.0,&cfg));
    assert(!terminal_outer_capture_admissible(&g,&divergent,100.0,&planet,
        gate_aero,&cfg,NULL));
    assert(!terminal_outer_capture_admissible(&g,&divergent,307.0,&planet,
        gate_aero,&cfg,NULL));

    /* A 37-degree ground-track angle can still be arranged to cross the
       threshold centerline exactly. It must not enter Final because its
       projected lateral drift carries it far outside the runway by the end. */
    LandingConfiguration reciprocal=cfg;
    reciprocal.site.runway_heading=270.0;
    Telemetry oblique=t;
    const double oblique_course=307.0;
    double error=(oblique_course-reciprocal.site.runway_heading)*DEG2RAD;
    double along_rate=oblique.horizontal_speed*cos(error);
    double cross_rate=oblique.horizontal_speed*sin(error);
    double threshold_time=-oblique.runway_along_track/along_rate;
    oblique.runway_cross_track=-cross_rate*threshold_time;
    assert(fabs(oblique.runway_cross_track+cross_rate*threshold_time)<1e-6);
    assert(!terminal_preflare_alignment_valid(&g,&oblique,oblique_course,&reciprocal));

    /* Recorded v4 HAC-exit offset: centreline-rate closure projected beyond
       the far pavement edge and delayed handoff. A midpoint intercept fits
       both ends without widening the admission gate. */
    LandingConfiguration delivery=reciprocal;
    delivery.guidance.sts_trajectory_delivery=true;
    delivery.guidance.touchdown_aim_along=150.0;
    snprintf(delivery.vehicle.model_id,sizeof(delivery.vehicle.model_id),"STS");
    Telemetry exit=t;exit.runway_along_track=-8000.0;exit.runway_cross_track=114.0;
    exit.horizontal_speed=130.0;exit.true_air_speed=139.0;
    double intercept_rate=NAN;
    assert(terminal_pavement_intercept_rate(&exit,&delivery,&intercept_rate));
    double intercept_course=delivery.site.runway_heading+
        asin(intercept_rate/exit.horizontal_speed)*RAD2DEG;
    assert(terminal_preflare_alignment_valid(&g,&exit,intercept_course,&delivery));
    double ratio=tan((intercept_course-delivery.site.runway_heading)*DEG2RAD);
    assert(fabs(exit.runway_cross_track+ratio*(-exit.runway_along_track+
        delivery.guidance.touchdown_aim_along))<1e-8);
    /* Near threshold, a midpoint track leaves 25 m of cross error at the
       threshold. The closest feasible touchdown track leaves under 20 m,
       while respecting both runway edges. Check mirrored approaches too. */
    for(int sign=-1;sign<=1;sign+=2){
        double r=NAN;
        assert(runway_landing_intercept_ratio(-500.0,sign*30.0,
            &delivery.site,&delivery.guidance,&r));
        assert(fabs(sign*30.0+r*500.0)<20.0);
        assert(fabs(sign*30.0+r*3000.0)<=35.0+1e-10);
    }
    exit.runway_cross_track=1000.0;
    assert(!terminal_pavement_intercept_rate(&exit,&delivery,&intercept_rate));
    delivery.guidance.sts_trajectory_delivery=false;exit.runway_cross_track=114.0;
    assert(!terminal_pavement_intercept_rate(&exit,&delivery,&intercept_rate));

    /* Direct runway guidance must reserve enough modelled lift for its lateral
       turn even when the radial vertical solution wants a steeper dive. */
    Telemetry turn={0};
    turn.true_air_speed=732.0;turn.horizontal_speed=731.9;turn.mach=2.7;
    turn.mean_altitude=24787.0;turn.flight_path_angle=-9.3;
    turn.angle_of_attack=13.4;turn.roll=6.4;turn.dynamic_pressure=25000.0;
    turn.mass=25722.0;turn.lift_force=139693.7;
    AerodynamicModel turn_aero=gate_aero;
    turn_aero.lift_to_drag=1.2;
    double reference_fpa=-29.4*DEG2RAD;
    double glide_aoa=NAN,lateral_aoa=NAN;
    double turn_aoa=terminal_direct_runway_target_aoa(&turn,&planet,turn_aero,
        &cfg.vehicle,reference_fpa,-30.0,-45.0,&glide_aoa,&lateral_aoa);
    double vertical_aoa=terminal_fpa_force_aoa(&turn,&planet,turn_aero,
        &cfg.vehicle,reference_fpa);
    double lateral=terminal_required_aoa_for_lateral_limit(&turn,
        turn_aero,&cfg.vehicle,-30.0,-45.0,cfg.vehicle.maximum_angle_of_attack);
    assert(isfinite(turn_aoa)&&isfinite(glide_aoa)&&isfinite(lateral_aoa));
    assert(lateral_aoa>vertical_aoa);
    assert(fabs(turn_aoa-lateral)<1e-6);

    /* A finite touchdown-speed forecast does not prove runway range. The
       range-short correction is current-state guidance, not a latched degraded
       admission that may float a later healthy state below approach speed. */
    {
        GuidanceMachine range_g;guidance_machine_init(&range_g);
        Telemetry range_t=t;
        /* V5 first-Final vertical state, with the runway displaced beyond its
           finite predicted range to exercise speed-valid/range-short behavior. */
        range_t.ut=26289.79;range_t.mean_altitude=range_t.radar_altitude=2374.993;
        range_t.mass=25222.799;range_t.true_air_speed=124.128;
        range_t.horizontal_speed=119.066;range_t.vertical_speed=-35.085;
        range_t.flight_path_angle=-16.418;range_t.angle_of_attack=3.482;
        range_t.dynamic_pressure=6427.306;range_t.atmospheric_density=.842758;
        range_t.mach=.361222;range_t.lift_force=252746.914;range_t.drag_force=114253.696;
        range_t.runway_along_track=-50000.0;
        LandingConfiguration range_cfg=delivery;
        range_cfg.guidance.sts_trajectory_delivery=true;
        range_cfg.vehicle.terminal_maximum_lift_angle_of_attack=range_cfg.vehicle.maximum_angle_of_attack;
        Trajectory reference;trajectory_init(&reference);
        GuidanceResult range=terminal_approach_sequence(&range_g,&range_t,270.0,
            &planet,gate_aero,&range_cfg,&reference,.1);
        assert(strstr(range.status,"Range-stretch glide"));
        assert(!range_g.terminal_final_low_energy);
        guidance_result_clear(&range);trajectory_clear(&reference);
    }

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

static void test_direct_runway_fallback_uses_response_window(void) {
    GuidanceMachine guidance;
    guidance_machine_init(&guidance);
    RunwayCaptureEnvelope capture={
        .valid=true,
        .reachable=false,
        .control_response_time_s=2.0
    };
    /* With no separate heading/lateral solve in this fixture, the measured
       two-second response reserve opens 400 m before the station. */
    assert(terminal_direct_runway_fallback_window(&guidance,-8300.0,200.0,
        8000.0,&capture));
    assert(!terminal_direct_runway_fallback_window(&guidance,-8500.0,200.0,
        8000.0,&capture));
    capture.heading_capture_time_s=12.0;
    capture.lateral_capture_time_s=20.0;
    /* Full capture time opens 4 km before the station, while farther-out
       states remain on the provisional route. */
    assert(terminal_direct_runway_fallback_window(&guidance,-12000.0,200.0,
        8000.0,&capture));
    assert(!terminal_direct_runway_fallback_window(&guidance,-12500.0,200.0,
        8000.0,&capture));
    capture.heading_capture_time_s=0.0;
    capture.lateral_capture_time_s=0.0;
    capture.reachable=true;
    assert(!terminal_direct_runway_fallback_window(&guidance,-8300.0,200.0,
        8000.0,&capture));
    guidance.mm305_route_committed=true;
    capture.reachable=false;
    assert(!terminal_direct_runway_fallback_window(&guidance,-8300.0,200.0,
        8000.0,&capture));
    assert(terminal_direct_runway_fallback_window(&guidance,-7900.0,200.0,
        8000.0,&capture));
}

int main(void) {
    test_touchdown_forecast_handoff_parity();
    test_plan_age_is_qualification_age();
    test_direct_runway_fallback_uses_response_window();
    puts("Final forecast handoff parity and plan freshness tests passed.");
    return 0;
}
