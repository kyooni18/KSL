/* Reuse recorded fixtures; this target excludes the unrelated full-75km mission assertion. */
#define main legacy_predictor_suite_main
#include "EntryPredictorSupervisionTests.c"
#undef main
#include <time.h>

static double wall_seconds(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static void test_recorded_v32_worker_snapshot(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);cfg.guidance.taem_interface_altitude=20000;
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state={.ut=67559.443642,.mass=40251.26171875,
        .position={339204.517350027,527127.356748946,-12631.0361353068},
        .velocity={-1053.01402555878,554.862527415849,-43.027906415742}};
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,17.8966026306152);
    t.dynamic_pressure=5268.7880859375;t.mach=3.29755555116814;t.g_force=.82023149728775;
    t.lift_force=157192.423662449;t.drag_force=291594.060934389;t.roll=-.438079833984375;
    t.roll_rate=-.752621176305955;t.angle_of_attack_rate=-.570197246012215;
    GuidanceMachine g;guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1,false,&env,&cal);
    g.entry_planning_deferred=true;g.diagnostic_shadow=true;
    DeorbitPlan plan=shadow_continuation_plan(&t);
    GuidanceResult r=guidance_update(&g,&t,&state,&plan,&p,aero,&cfg);guidance_result_clear(&r);
    GuidanceMachine before=g,proposal=g;proposal.entry_planning_deferred=false;
    if(!isfinite(proposal.entry_s_turn_plan.cost))proposal.entry_control_plan_valid=false;
    double start=wall_seconds();
    r=guidance_update(&proposal,&t,&state,&plan,&p,aero,&cfg);guidance_result_clear(&r);
    double solve_ms=(wall_seconds()-start)*1000;
    proposal.entry_planning_deferred=true;
    PredictorPlannerTrace trace_before={0},trace_after={0};
    bool have_trace=predictor_last_planner_trace(&trace_before);
    start=wall_seconds();
    EntryPrediction a=predictor_simulate_entry_guidance_shadow(state,&t,&proposal,&plan,&p,aero,&env,&cal,&cfg,500,true);
    double replay_ms=(wall_seconds()-start)*1000;
    EntryPrediction b=predictor_simulate_entry_guidance_shadow(state,&t,&proposal,&plan,&p,aero,&env,&cal,&cfg,500,true);
    assert(a.shadow_guidance_used&&b.shadow_guidance_used);
    assert(fabs(a.final_state.ut-b.final_state.ut)<1e-9);
    assert(vmag(vsub(a.final_state.position,b.final_state.position))<1e-9);
    assert(a.reached_taem==b.reached_taem&&a.s_turn_reversals==b.s_turn_reversals);
    assert(memcmp(&g,&before,sizeof(g))==0);
    /* A replay cannot run a private search or change its parent's candidate trace.
       This recorded v32 snapshot is a parity fixture, not a nominal-capture fixture:
       degraded MM305 ownership may be public TAEM, but without the fixed interface
       capture it must remain explicitly marked as a missed MM304 boundary. */
    if(have_trace){assert(predictor_last_planner_trace(&trace_after));assert(memcmp(&trace_before,&trace_after,sizeof(trace_before))==0);}
    if(a.reached_taem)assert(!a.shadow_aborted);
    if(a.reached_taem&&!a.taem_dynamic_interface_captured)
        assert(a.taem_ownership_boundary_missed);
    printf("v32 worker snapshot: search %.2f ms, executable replay %.2f ms, TAEM %d, capture %d, boundary miss %d, reversals %u\n",
        solve_ms,replay_ms,a.reached_taem,a.taem_dynamic_interface_captured,a.taem_ownership_boundary_missed,a.s_turn_reversals);
    entry_prediction_clear(&a);entry_prediction_clear(&b);deorbit_plan_clear(&plan);
}
static void test_topology_uses_published_mm304_target(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);cfg.guidance.taem_interface_altitude=20000;
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state={.ut=67559.443642,.mass=40251.26171875,
        .position={339204.517350027,527127.356748946,-12631.0361353068},
        .velocity={-1053.01402555878,554.862527415849,-43.027906415742}};
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,17.8966026306152);
    t.dynamic_pressure=5268.7880859375;t.mach=3.29755555116814;t.g_force=.82023149728775;
    t.lift_force=157192.423662449;t.drag_force=291594.060934389;
    t.roll=-.438079833984375;t.roll_rate=-.752621176305955;
    GuidanceMachine g;guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1,false,&env,&cal);
    double station=-cfg.guidance.final_approach_distance;
    g.taem_interface_target=(TaemInterfaceTarget){.valid=true,.along_track=station,.cross_track=0.0,
        .course=45.0,.altitude=20000.0,.speed=820.0,.flight_path_angle=-12.0,
        .specific_energy=rotating_specific_energy(cfg.site.latitude,20000.0,820.0,&p),
        .selected_ut=t.ut,.arrival_ut=t.ut+120.0,.acquisition_lead=4305.0,
        .remaining_path=60000.0,.response_time=8.0};
    EntryTopologyPlan top=predictor_plan_entry_topology(state,&t,&g,&p,&env,&cal,&cfg);
    assert(isfinite(top.cost));
    assert(fabs(norm_signed_deg(top.outbound_course_offset-(-45.0)))<1e-9);
    assert(top.first_sign*top.outbound_course_offset<0.0);
    if(top.inlet.valid)assert(fabs(norm_signed_deg(top.inlet.course-45.0))<1e-9);
    /* The topology search must actually exercise its second/final event timing
       dimension.  This recorded published-target fixture need not be a valid capture,
       but it may not fall back to "terminal turn never started" merely because every
       search caller supplied NAN for the forced terminal delay. */
    assert((top.failure_flags&ENTRY_TOPOLOGY_FAILURE_TERMINAL_TURN)==0);
    /* The worker must preserve live MM304 ownership semantics.  A scheduled clock
       may not force the setup-side sign change; once setup geometry is proven, that
       released sign change may itself be the final inlet turn.  Therefore the
       terminal-turn event may coincide with the reversal, but it may never precede it. */
    if(isfinite(top.terminal_turn_ut)){
        assert(isfinite(top.reversal_ut));
        assert(top.terminal_turn_ut>=top.reversal_ut-1e-9);
    }
}

int main(void){
    test_committed_reversal_replay_uses_live_time_and_dwell_gates_only();
    test_committed_plan_above_autonomous_60deg_cap_uses_live_bank_authority();
    test_shared_conservative_stall_proxy();
    test_recorded_v27_supervisor_honors_dynamic_interface_contract();
    test_topology_uses_published_mm304_target();
    test_recorded_v32_worker_snapshot();
    puts("PASS: committed predictor policy and recorded worker replay");return 0;
}
