/* No transport or live flight: exercise the executable seam directly. */
#include "../CLanding/guidance.c"
#include <assert.h>
#include <time.h>

static double seconds(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static PlanetModel planet(void){
    return (PlanetModel){.radius=600000,.gravitational_parameter=3.5316e12,
        .rotational_speed=.000291570900559802,.atmosphere_depth=70000,
        .surface_density=1.1399,.atmosphere_adiabatic_index=1.4,
        .atmosphere_sample_count=2,
        .atmosphere_altitude={0.0,70000.0},
        .atmosphere_pressure={101325.0,0.1},
        .atmosphere_density={1.1399,0.0001},
        .north_axis={0,0,1},.prime_meridian_at_epoch={1,0,0}};
}
static LandingConfiguration configuration(void){
    LandingConfiguration cfg=landing_configuration_default();
    cfg.guidance.taem_interface_altitude=20000.0;
    cfg.vehicle.maximum_bank_angle=45.0;
    landing_configuration_normalize(&cfg);return cfg;
}
static Telemetry sample(void){
    Telemetry t;telemetry_init(&t);
    t.ut=100;t.mean_altitude=t.radar_altitude=41400;t.true_air_speed=1923;
    t.flight_path_angle=-7;t.vertical_speed=1923*sin(-7*DEG2RAD);
    t.horizontal_speed=1923*cos(-7*DEG2RAD);t.mass=40251;t.lift_force=t.mass*5;
    t.drag_force=t.mass*3;t.dynamic_pressure=3000;t.g_force=1;t.bank_effectiveness=1;
    t.attitude_response.pitch_valid=true;
    t.attitude_response.maximum_pitch_rate_deg_s=8.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=5.0;
    t.attitude_response.roll_valid=true;
    t.attitude_response.maximum_roll_rate_deg_s=18.0;
    t.attitude_response.maximum_roll_accel_deg_s2=15.0;
    t.angle_of_attack=18;t.ground_track_heading=t.heading=92.4;t.roll=25;
    t.runway_along_track=-303000;t.runway_cross_track=-18675;t.range_to_site=303575;
    t.latitude=-1;t.longitude=-95;t.mach=6;t.aerodynamic_confidence=.8;
    t.stall_fraction=0;t.stall_fraction_is_measured=true;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");return t;
}
static GuidanceMachine machine(Telemetry*t,PlanetModel*p,LandingConfiguration*cfg){
    GuidanceMachine g;guidance_machine_init(&g);g.automation_engaged=true;
    g.phase=PHASE_ENTRY_ENERGY;g.entry_planning_deferred=true;g.diagnostic_shadow=true;
    g.s_turn_sign=1;g.has_s_turn_leg_started=true;g.s_turn_leg_started_ut=t->ut-60;
    g.entry_reference_speed=2050;g.entry_reference_altitude=50000;g.entry_reference_range=440000;
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    entry_publish_taem_tangent_target(&g,t,t->ground_track_heading,p,aero,cfg);
    assert(g.taem_interface_target.valid);
    assert(fabs(norm_signed_deg(g.taem_interface_target.course-
        cfg->site.runway_heading))==90.0);
    return g;
}
static void test_commit_and_adoption(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);g.previous_ut=t.ut;g.control_plan_sequence=4;
    g.entry_s_turn_plan=(EntryControlPlan){.valid=true,.planned_ut=100,.segment_duration=80,
        .target_bank=25,.target_aoa=18,.plan_id=4,.plan_version=1};
    EntryControlPlan event=g.entry_s_turn_plan;event.has_planned_reversal=true;
    event.planned_reversal_ut=140;event.planned_reversal_sign=-1;event.planned_reversal_is_final=true;
    entry_program_commit_planned_reversal(&g,&event,t.ut,false);g.entry_s_turn_plan=event;
    for(int i=0;i<40;i++){
        EntryControlPlan later=event;later.planned_reversal_ut=170+i;later.target_bank=10+i;
        entry_program_commit_planned_reversal(&g,&later,101+i*.1,false);
        assert(g.entry_reversal_ut==140&&later.planned_reversal_ut==140);
        assert(g.entry_reversal_is_final);
    }
    EntryControlPlan ordinary=event;ordinary.planned_reversal_ut=120;ordinary.planned_reversal_is_final=false;
    entry_program_commit_planned_reversal(&g,&ordinary,101,false);
    assert(g.entry_reversal_ut==140&&ordinary.planned_reversal_is_final);
    GuidanceMachine request=g,result=g;result.entry_s_turn_plan.target_bank=35;
    result.entry_s_turn_plan.planned_ut=100;result.entry_supervision_valid=true;
    g.roll_limiter.value=17;g.pitch_limiter.value=22;g.entry_control_bank=27;
    result.roll_limiter.value=999;result.entry_control_bank=-60;t.ut=102;
    assert(guidance_accept_entry_plan(&g,&request,&result,&t,&cfg));
    assert(g.roll_limiter.value==17&&g.pitch_limiter.value==22&&g.entry_control_bank==27);
    assert(g.entry_reversal_ut==140&&g.entry_reversal_is_final);
    assert(!guidance_accept_entry_plan(&g,&request,&result,&t,&cfg)); /* obsolete lineage */
    g=request;g.paused=true;assert(!guidance_accept_entry_plan(&g,&request,&result,&t,&cfg));
    g=request;g.s_turn_sign=-1;assert(!guidance_accept_entry_plan(&g,&request,&result,&t,&cfg));
    g=request;g.taem_interface_target.cross_track+=10;
    assert(!guidance_accept_entry_plan(&g,&request,&result,&t,&cfg));
    g=request;t.ut=110;assert(guidance_accept_entry_plan(&g,&request,&result,&t,&cfg)); /* age alone is not invalidation */
    g=request;t.ut=102;result.entry_s_turn_plan.planned_reversal_ut=170;
    assert(!guidance_accept_entry_plan(&g,&request,&result,&t,&cfg)); /* unmatched forecast */
    result.entry_s_turn_plan.planned_reversal_ut=101;
    assert(!guidance_accept_entry_plan(&g,&request,&result,&t,&cfg)); /* deadline elapsed */
    g=request;g.entry_committed_infeasible=true;request=g;result=g;
    result.entry_s_turn_plan.planned_reversal_ut=170;
    assert(guidance_accept_entry_plan(&g,&request,&result,&t,&cfg));
    assert(g.entry_reversal_ut==170); /* explicit committed-trajectory evidence */
    g=request;g.entry_committed_infeasible=false;g.previous_ut=140;request=g;result=g;t.ut=142;
    assert(guidance_accept_entry_plan(&g,&request,&result,&t,&cfg));
    assert(g.entry_reversal_ut==140&&g.entry_control_reversals==0); /* overdue geometry gate, still owned */
}
static void test_propagated_shaping_event_timing_and_terminal_reachability(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);g.diagnostic_shadow=false;
    double final_relative=norm_signed_deg(g.taem_interface_target.course-cfg.site.runway_heading);
    double final_sign=final_relative>=0.0?1.0:-1.0;
    g.s_turn_sign=-final_sign;
    g.entry_reversal_scheduled=true;g.entry_reversal_is_final=false;
    g.entry_reversal_sign=final_sign;g.entry_reversal_bank=40;g.entry_reversal_ut=100;
    g.has_s_turn_reversal_requested=false;g.has_s_turn_leg_started=true;g.s_turn_leg_started_ut=40;

    /* A propagated event owns its own time. Actuator/capture lead is modeled in
       terminal reachability; it is not permission to execute the sign change
       before the event selected by trajectory propagation. */
    GuidanceMachine early=g;
    Telemetry early_t=t;early_t.ut=90.0;
    assert(early_t.ut<early.entry_reversal_ut);
    assert(!entry_program_execute_planned_reversal(&early,&early_t,&p,&cfg));
    assert(early.s_turn_sign==-final_sign&&early.entry_control_reversals==0&&
        early.entry_reversal_scheduled);

    /* At the propagated event, an ordinary shaping side change executes even
       when the fixed-station terminal turn is still physically unreachable. */
    t.ut=130.0;
    double endpoint_error=NAN,response_time=NAN;
    assert(!entry_program_final_reversal_geometry_ready(&g,&t,&cfg,final_sign));
    assert(!entry_program_rollthrough_endpoint_ready(&g,&t,&p,&cfg,final_sign,
        &endpoint_error,&response_time));
    assert(entry_program_execute_planned_reversal(&g,&t,&p,&cfg));
    assert(g.s_turn_sign==final_sign&&g.entry_control_reversals==1&&
        !g.entry_reversal_scheduled);
    assert(!g.entry_final_reversal_pending&&!g.entry_final_reversal_completed);

    /* Marking the same event final does not weaken the reachability contract. */
    GuidanceMachine final_gate=machine(&t,&p,&cfg);final_gate.diagnostic_shadow=false;
    final_gate.s_turn_sign=-final_sign;
    final_gate.entry_reversal_scheduled=true;final_gate.entry_reversal_is_final=true;
    final_gate.entry_reversal_sign=final_sign;final_gate.entry_reversal_bank=40;
    final_gate.entry_reversal_ut=100;
    final_gate.has_s_turn_reversal_requested=false;final_gate.has_s_turn_leg_started=true;
    final_gate.s_turn_leg_started_ut=40;final_gate.entry_supervision_valid=false;
    final_gate.entry_s_turn_plan.terminal_ready=false;
    assert(!entry_program_final_reversal_geometry_ready(&final_gate,&t,&cfg,final_sign));
    assert(!entry_program_rollthrough_endpoint_ready(&final_gate,&t,&p,&cfg,final_sign,
        &endpoint_error,&response_time));
    assert(!entry_program_execute_planned_reversal(&final_gate,&t,&p,&cfg));
    assert(final_gate.s_turn_sign==-final_sign&&final_gate.entry_reversal_scheduled);
}
static void test_same_side_nonterminal_event_cannot_masquerade_as_reversal(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);
    double terminal_relative=norm_signed_deg(g.taem_interface_target.course-cfg.site.runway_heading);
    double terminal_sign=terminal_relative>=0.0?1.0:-1.0;
    g.s_turn_sign=terminal_sign;g.entry_control_reversals=1;
    g.entry_reversal_scheduled=true;g.entry_reversal_is_final=false;
    g.entry_reversal_sign=terminal_sign;g.entry_reversal_bank=40;g.entry_reversal_ut=100;
    t.ut=130.0;
    t.ground_track_heading=t.heading=norm_deg(g.taem_interface_target.course-terminal_sign*60.0);
    t.roll=terminal_sign*12.0;
    t.runway_along_track=g.taem_interface_target.along_track-80000.0;
    t.runway_cross_track=g.taem_interface_target.cross_track;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);

    double endpoint_error=NAN,response_time=NAN;
    assert(!entry_program_final_reversal_geometry_ready(&g,&t,&cfg,terminal_sign));
    assert(!entry_program_rollthrough_endpoint_ready(&g,&t,&p,&cfg,terminal_sign,
        &endpoint_error,&response_time));
    assert(!entry_program_execute_planned_reversal(&g,&t,&p,&cfg));
    assert(g.entry_control_reversals==1&&g.entry_reversal_scheduled&&
        g.s_turn_sign==terminal_sign);

    /* Geometry demand may continue shaping the owned side, but it must remain
       inside the live bank envelope and is not itself a reversal permission. */
    double staging=entry_taem_gate_acquisition_bank(&g,&t,&p,aero,&cfg);
    double bank_limit=dynamic_bank_limit(&t,&cfg.vehicle);
    assert(isfinite(staging));
    assert(staging*g.s_turn_sign>=-sqrt(DBL_EPSILON));
    assert(fabs(staging)<=bank_limit+sqrt(DBL_EPSILON));
}

static void test_joint_lift_budget(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);
    g.taem_interface_target.valid=true;
    g.taem_interface_target.along_track=-cfg.guidance.final_approach_distance;
    g.taem_interface_target.cross_track=0.0;
    g.taem_interface_target.altitude=cfg.guidance.taem_interface_altitude;
    g.taem_interface_target.speed=600.0;
    g.taem_interface_target.course=norm_deg(cfg.site.runway_heading-90.0);
    g.taem_interface_target.flight_path_angle=-fabs(cfg.guidance.taem_glide_slope);
    g.taem_interface_target.response_time=0.0;
    g.taem_interface_target.acquisition_lead=0.0;
    double bank_limit=dynamic_bank_limit(&t,&cfg.vehicle);
    double far_ceiling=entry_program_vertical_bank_ceiling(&g,&t,&p,aero,&cfg);
    assert(isfinite(far_ceiling)&&far_ceiling>=0.0);
    assert(far_ceiling<=bank_limit+sqrt(DBL_EPSILON));
    t.runway_along_track=g.taem_interface_target.along_track-15000;
    t.runway_cross_track=g.taem_interface_target.cross_track-12000;
    t.mean_altitude=g.taem_interface_target.altitude+500;
    double near_ceiling=entry_program_vertical_bank_ceiling(&g,&t,&p,aero,&cfg);
    assert(isfinite(near_ceiling)&&near_ceiling>=0.0);
    assert(near_ceiling<=far_ceiling+sqrt(DBL_EPSILON));
    VehicleState state={.ut=t.ut,.mass=t.mass};
    GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
    assert(!g.entry_control_terminal_ready);
    guidance_result_clear(&r);
}
static void test_opposite_geometry_requires_a_committed_event(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);t.heading=t.ground_track_heading=115;
    double required=entry_taem_gate_required_bank(&g,&t,&p,aero,&cfg);
    double acquisition=entry_taem_gate_acquisition_bank(&g,&t,&p,aero,&cfg);
    assert(required*g.s_turn_sign>=-1e-9);
    assert(acquisition*g.s_turn_sign>=-1e-9);
    assert(!g.entry_reversal_scheduled&&g.entry_control_reversals==0&&g.s_turn_sign==1);
    g.entry_control_reversals=1;
    double reacquire=entry_taem_gate_acquisition_bank(&g,&t,&p,aero,&cfg);
    double count_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(fabs(acquisition),fabs(reacquire)));
    assert(fabs(reacquire-acquisition)<=count_resolution);
    g.entry_control_reversals=0;
    VehicleState state={.ut=t.ut,.mass=t.mass};
    GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
    assert(g.s_turn_sign==1&&!g.entry_reversal_scheduled);
    assert(g.entry_control_bank*g.s_turn_sign>=-1e-9);
    guidance_result_clear(&r);
}
static void test_deferred_control_and_final_arc(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);VehicleState state={.ut=t.ut,.mass=t.mass};
    double worst=0;
    for(int i=0;i<1000;i++){
        t.ut=100+i*.1;state.ut=t.ut;
        double start=seconds();
        GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
        worst=fmax(worst,seconds()-start);
        assert(g.entry_planning_needed&&isfinite(g.entry_control_bank));
        assert(fabs(g.entry_control_bank)<=dynamic_bank_limit(&t,&cfg.vehicle)+1e-9);
        guidance_result_clear(&r);
    }
    assert(worst<.1); /* planner unavailable for 100 simulated seconds, still 10 Hz */
    g.entry_reversal_scheduled=true;g.entry_reversal_is_final=true;
    g.entry_reversal_sign=-1;g.entry_reversal_ut=t.ut;g.entry_reversal_bank=25;
    g.has_s_turn_reversal_requested=false;g.has_s_turn_leg_started=true;g.s_turn_leg_started_ut=t.ut-60;
    /* Make this an unambiguous left-turn final event relative to whichever HAC
       inlet side the new geometry selected, then place the gate on that same
       turn side so the geometry gate—not an obsolete absolute heading—owns it. */
    t.ground_track_heading=t.heading=norm_deg(g.taem_interface_target.course+60.0);
    double gate_bearing=norm_deg(t.ground_track_heading-20.0);
    double gate_axis=norm_signed_deg(gate_bearing-cfg.site.runway_heading)*DEG2RAD;
    t.runway_along_track=g.taem_interface_target.along_track-100000.0*cos(gate_axis);
    t.runway_cross_track=g.taem_interface_target.cross_track-100000.0*sin(gate_axis);
    assert(entry_program_execute_planned_reversal(&g,&t,&p,&cfg));
    for(int i=0;i<100;i++){
        t.ut+=.1;state.ut=t.ut;t.roll=-25;
        GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
        assert(g.s_turn_sign==-1&&g.entry_control_bank<=0);
        assert(g.entry_control_reversals==1&&!g.entry_reversal_scheduled);
        guidance_result_clear(&r);
    }
    printf("deferred MM304 1000 ticks: worst %.3f ms; one final reversal, no opposite arc\n",worst*1000);
}

static void test_post_reversal_course_capture_survives_vertical_ceiling(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);VehicleState state={.ut=t.ut,.mass=t.mass};
    g.s_turn_sign=-1;g.entry_control_reversals=1;g.has_s_turn_leg_started=true;
    g.s_turn_leg_started_ut=t.ut-40;g.entry_reversal_scheduled=false;
    g.entry_final_reversal_pending=false;g.entry_final_reversal_completed=false;
    g.taem_interface_target.course=0.0;
    t.ground_track_heading=t.heading=80.0;t.roll=-20.0;
    t.true_air_speed=1000.0;t.horizontal_speed=990.0;t.vertical_speed=-140.0;
    t.flight_path_angle=atan2(t.vertical_speed,t.horizontal_speed)*RAD2DEG;
    t.mean_altitude=t.radar_altitude=g.taem_interface_target.altitude+500.0;
    t.runway_along_track=g.taem_interface_target.along_track-15000.0;
    t.runway_cross_track=g.taem_interface_target.cross_track-12000.0;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);
    state.ut=t.ut;
    double bank_limit=dynamic_bank_limit(&t,&cfg.vehicle);
    GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
    /* Vertical and lateral feasibility are coupled. Regardless of whether this
       synthetic state is feasible, guidance may not command the wrong side or
       escape the live bank envelope merely to preserve course capture. */
    assert(g.entry_control_reversals==1&&g.s_turn_sign==-1);
    assert(isfinite(g.entry_control_bank));
    assert(g.entry_control_bank*g.s_turn_sign>=-sqrt(DBL_EPSILON));
    assert(fabs(g.entry_control_bank)<=bank_limit+sqrt(DBL_EPSILON));
    assert(!g.taem_interface_captured);
    guidance_result_clear(&r);
    /* Reducing altitude tightens the same coupled problem. It still cannot
       authorize a wrong-side or out-of-envelope command. */
    t.ut+=.1;state.ut=t.ut;
    t.mean_altitude=t.radar_altitude=g.taem_interface_target.altitude-2500.0;
    t.ground_track_heading=t.heading=70.0;
    r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
    bank_limit=dynamic_bank_limit(&t,&cfg.vehicle);
    assert(isfinite(g.entry_control_bank));
    assert(g.entry_control_bank*g.s_turn_sign>=-sqrt(DBL_EPSILON));
    assert(fabs(g.entry_control_bank)<=bank_limit+sqrt(DBL_EPSILON));
    assert(!g.taem_interface_captured);
    guidance_result_clear(&r);
}

static void test_overdue_terminal_side_setup_preserves_energy(void){
    LandingConfiguration cfg=configuration();cfg.guidance.taem_force_handoff_speed=1300.0;cfg.vehicle.entry_angle_of_attack=18.0;PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);VehicleState state={.ut=t.ut,.mass=t.mass};
    g.diagnostic_shadow=false;
    g.s_turn_sign=-1;g.has_s_turn_leg_started=true;g.s_turn_leg_started_ut=t.ut-80.0;
    g.taem_interface_target.course=0.0;
    g.entry_reversal_scheduled=true;g.entry_reversal_is_final=false;
    g.entry_reversal_sign=-1.0;g.entry_reversal_ut=t.ut-30.0;g.entry_reversal_bank=35.0;
    t.ground_track_heading=t.heading=72.0;t.roll=-35.0;
    t.true_air_speed=1500.0;t.horizontal_speed=1495.0;t.vertical_speed=-120.0;
    t.flight_path_angle=atan2(t.vertical_speed,t.horizontal_speed)*RAD2DEG;
    t.dynamic_pressure=1000.0;t.g_force=1.2;t.stall_fraction=.02;t.stall_fraction_is_measured=true;
    t.runway_along_track=g.taem_interface_target.along_track-140000.0;
    t.runway_cross_track=g.taem_interface_target.cross_track-5000.0;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);state.ut=t.ut;
    assert(!entry_program_final_reversal_geometry_ready(&g,&t,&cfg,-1.0));
    GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
    /* A same-side committed event can be the terminal-turn setup gate rather than a
       literal bank-sign reversal. The energy manager may choose any incidence
       inside the active thermal/final-S-turn envelope; the test must not freeze
       a tuned efficient-turn AoA band. */
    assert(g.s_turn_sign==-1.0&&g.entry_reversal_scheduled);
    assert(isfinite(g.entry_control_aoa));
    assert(g.entry_control_aoa>=entry_thermal_protection_aoa_floor(&cfg.vehicle)-sqrt(DBL_EPSILON));
    assert(g.entry_control_aoa<=entry_final_s_turn_aoa_ceiling(&cfg.vehicle)+sqrt(DBL_EPSILON));
    guidance_result_clear(&r);
}

static void test_final_s_turn_drag_aoa_extension(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g=machine(&t,&p,&cfg);
    g.taem_interface_target.valid=true;
    g.taem_interface_target.along_track=-cfg.guidance.final_approach_distance;
    g.taem_interface_target.cross_track=0.0;
    g.taem_interface_target.altitude=cfg.guidance.taem_interface_altitude;
    g.taem_interface_target.speed=600.0;
    g.taem_interface_target.course=norm_deg(cfg.site.runway_heading-90.0);
    g.taem_interface_target.flight_path_angle=-fabs(cfg.guidance.taem_glide_slope);
    g.taem_interface_target.response_time=0.0;
    g.taem_interface_target.acquisition_lead=0.0;
    assert(cfg.vehicle.maximum_angle_of_attack<35.0);
    assert(fabs(entry_final_s_turn_aoa_ceiling(&cfg.vehicle)-35.0)<1e-9);
    g.entry_final_reversal_pending=true;g.entry_final_reversal_completed=false;
    t.true_air_speed=1750.0;t.horizontal_speed=1750.0;t.vertical_speed=0.0;t.flight_path_angle=0.0;
    t.dynamic_pressure=6000.0;t.g_force=1.1;t.stall_fraction=.01;t.stall_fraction_is_measured=true;
    /* Reproduce finalaoa2: the local planner claims undershoot while live energy
       and measured drag both show a large overshoot. Live evidence must win. */
    t.energy_excess_range=200000.0;
    EntryDragReferenceOutput drag={.valid=true,.predicted_range_error=-58500.0,.range_error_scale=40000.0,
        .drag_error_accel=5.0,.drag_error_fraction=.50};
    double extended=entry_program_final_s_turn_drag_aoa(&g,&t,&p,aero,&cfg,&drag,true);
    assert(isfinite(extended)&&extended>cfg.vehicle.maximum_angle_of_attack&&extended<=35.0+1e-9);
    assert(!isfinite(entry_program_final_s_turn_drag_aoa(&g,&t,&p,aero,&cfg,&drag,false)));
    t.true_air_speed=600.0;t.horizontal_speed=600.0;
    assert(!isfinite(entry_program_final_s_turn_drag_aoa(&g,&t,&p,aero,&cfg,&drag,true)));
    t.true_air_speed=850.0;t.horizontal_speed=850.0;
    assert(isfinite(entry_program_final_s_turn_drag_aoa(&g,&t,&p,aero,&cfg,&drag,true)));
    t.true_air_speed=1750.0;t.horizontal_speed=1750.0;

    /* The committed terminal-side reversal becomes final-S-turn energy management
       before the roll-through so high drag has enough distance to be useful. */
    g.entry_final_reversal_pending=false;g.entry_final_reversal_completed=false;
    g.entry_control_reversals=0;g.entry_reversal_scheduled=true;
    g.taem_interface_target.valid=true;g.taem_interface_target.course=0.0;
    t.ground_track_heading=t.heading=90.0;
    g.entry_reversal_sign=-1.0;g.entry_reversal_ut=t.ut+60.0;
    assert(entry_program_terminal_side_final_s_turn_armed(&g,&t,&cfg,t.ground_track_heading));
    g.entry_reversal_ut=t.ut+90.0;
    assert(!entry_program_terminal_side_final_s_turn_armed(&g,&t,&cfg,t.ground_track_heading));
    g.entry_reversal_ut=t.ut+60.0;g.entry_reversal_sign=1.0;
    assert(!entry_program_terminal_side_final_s_turn_armed(&g,&t,&cfg,t.ground_track_heading));

    /* The same overshoot must not buy drag by diving farther below the vertical corridor. */
    t.flight_path_angle=-30.0;
    assert(!isfinite(entry_program_final_s_turn_drag_aoa(&g,&t,&p,aero,&cfg,&drag,true)));

    /* The common reference shaper must pass an explicitly extended Entry command,
       while the ordinary vehicle profile itself remains unchanged. */
    t.flight_path_angle=0.0;t.angle_of_attack=cfg.vehicle.maximum_angle_of_attack;
    GuidanceCommand c=atmospheric(&t,t.heading,0.0,&cfg.vehicle,0.0,false,PROFILE_ENTRY);
    c.has_target_aoa=true;c.target_aoa=35.0;c.target_pitch=35.0;
    double seen=0.0;
    for(int i=0;i<8;i++){
        GuidanceResult r=stabilized(&g,result_make(PHASE_ENTRY_ENERGY,c,"test",NULL),
            &t,&cfg.vehicle,&cfg.guidance,.5);
        seen=fmax(seen,r.command.target_aoa);
        t.angle_of_attack=r.command.target_aoa;
        guidance_result_clear(&r);
    }
    assert(seen>cfg.vehicle.maximum_angle_of_attack+.5&&seen<=35.0+1e-9);
}

static void test_two_event_topology_installs_shaping_reversal(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);
    EntryTopologyPlan top={0};top.valid=true;top.planned_ut=t.ut;top.first_sign=1.0;
    top.first_bank=25.0;top.turn_bank=45.0;top.first_aoa=18.0;top.turn_aoa=16.0;
    top.reversal_ut=t.ut+40.0;top.terminal_turn_ut=t.ut+90.0;top.capture_ut=t.ut+140.0;
    top.reversal_along=t.runway_along_track+50000.0;top.reversal_cross=t.runway_cross_track+10000.0;
    top.reversal_altitude=32000.0;top.reversal_speed=1500.0;top.reversal_course=90.0;
    top.inlet=g.taem_interface_target;top.inlet.valid=true;top.inlet.course=0.0;
    assert(guidance_install_entry_topology(&g,&top,t.ut));
    assert(g.entry_topology.valid&&g.entry_reversal_scheduled);
    assert(!g.entry_reversal_is_final); /* first event shapes energy/crossrange only */
    assert(g.entry_reversal_sign==-1.0&&g.entry_s_turn_plan.predicted_reversals==1u);
    (void)p;
}

static EntryTopologyPlan replacement_topology(const GuidanceMachine*g,const Telemetry*t){
    EntryTopologyPlan top={0};
    top.valid=true;top.planned_ut=t->ut;top.first_sign=g->s_turn_sign;
    top.first_bank=30.0;top.turn_bank=45.0;top.first_aoa=18.0;top.turn_aoa=16.0;
    top.reversal_ut=t->ut+50.0;top.terminal_turn_ut=t->ut+100.0;top.capture_ut=t->ut+150.0;
    top.reversal_along=t->runway_along_track+40000.0;top.reversal_cross=t->runway_cross_track+12000.0;
    top.reversal_altitude=32000.0;top.reversal_speed=1450.0;top.reversal_course=90.0;
    top.glide_reserve=5000.0;top.inlet=g->taem_interface_target;top.inlet.valid=true;
    return top;
}

static void test_valid_topology_replaces_only_unexecuted_ordinary_reversal(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);g.diagnostic_shadow=false;
    g.entry_reversal_scheduled=true;g.entry_reversal_is_final=false;
    g.entry_reversal_ut=t.ut+120.0;g.entry_reversal_sign=-g.s_turn_sign;g.entry_reversal_bank=25.0;
    EntryTopologyPlan top=replacement_topology(&g,&t);
    assert(guidance_install_entry_topology(&g,&top,t.ut));
    assert(g.entry_topology.valid&&g.entry_reversal_scheduled&&!g.entry_reversal_is_final);
    assert(g.entry_reversal_ut==top.reversal_ut&&g.entry_reversal_sign==-top.first_sign);

    GuidanceMachine final_gate=machine(&t,&p,&cfg);
    final_gate.entry_reversal_scheduled=true;final_gate.entry_reversal_is_final=true;
    final_gate.entry_reversal_ut=t.ut+120.0;final_gate.entry_reversal_sign=-final_gate.s_turn_sign;
    top=replacement_topology(&final_gate,&t);
    assert(!guidance_install_entry_topology(&final_gate,&top,t.ut));

    GuidanceMachine executed=machine(&t,&p,&cfg);executed.entry_control_reversals=1;
    top=replacement_topology(&executed,&t);
    assert(!guidance_install_entry_topology(&executed,&top,t.ut));

    GuidanceMachine opposite=machine(&t,&p,&cfg);
    opposite.entry_reversal_scheduled=true;opposite.entry_reversal_is_final=false;
    opposite.entry_reversal_ut=t.ut+120.0;opposite.entry_reversal_sign=-opposite.s_turn_sign;
    top=replacement_topology(&opposite,&t);top.first_sign=-opposite.s_turn_sign;
    assert(!guidance_install_entry_topology(&opposite,&top,t.ut));
}

static void test_worker_topology_replaces_matching_live_ordinary_reversal(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine request=machine(&t,&p,&cfg);request.diagnostic_shadow=false;
    request.previous_ut=t.ut;request.control_plan_sequence=11;
    request.entry_s_turn_plan=(EntryControlPlan){.valid=true,.planned_ut=t.ut,.segment_duration=220.0,
        .target_bank=request.s_turn_sign*25.0,.target_aoa=18.0,.plan_id=11,.plan_version=1,
        .has_planned_reversal=true,.planned_reversal_is_final=false,.planned_reversal_ut=t.ut+120.0,
        .planned_reversal_sign=-request.s_turn_sign,.planned_reversal_range=250000.0,.predicted_reversals=1};
    request.entry_reversal_scheduled=true;request.entry_reversal_is_final=false;
    request.entry_reversal_ut=t.ut+120.0;request.entry_reversal_sign=-request.s_turn_sign;
    request.entry_reversal_bank=25.0;
    GuidanceMachine result=request;
    EntryTopologyPlan top=replacement_topology(&result,&t);
    assert(guidance_install_entry_topology(&result,&top,t.ut));
    result.entry_supervision_valid=true;result.entry_supervision_boundary_missed=false;
    GuidanceMachine live=request;t.ut+=2.0;
    assert(guidance_accept_entry_plan(&live,&request,&result,&t,&cfg));
    assert(live.entry_topology.valid&&live.entry_reversal_scheduled&&!live.entry_reversal_is_final);
    assert(live.entry_reversal_ut==top.reversal_ut&&live.entry_reversal_sign==-top.first_sign);

    GuidanceMachine stale=request;stale.entry_reversal_ut+=1.0;
    assert(!guidance_accept_entry_plan(&stale,&request,&result,&t,&cfg));
}

static void test_strict_taem_speed_does_not_compress_entry_alpha_schedule(void){
    LandingConfiguration cfg=configuration();
    cfg.vehicle.minimum_safe_speed=85.0;
    cfg.vehicle.entry_angle_of_attack=18.0;
    cfg.vehicle.maximum_angle_of_attack=28.0;
    cfg.guidance.taem_force_handoff_speed=1300.0;
    TaemInterfaceTarget q={0};q.valid=true;q.speed=500.0;
    double strict_taem_velocity=q.speed;
    double alpha_transition=entry_alpha_transition_velocity(&cfg.guidance,strict_taem_velocity);
    assert(strict_taem_velocity==500.0);
    assert(alpha_transition==1300.0);
    assert(q.speed==500.0); /* alpha scheduling must not relax the capture target */

    EntryAlphaSchedule compressed,decoupled;
    entry_alpha_schedule_default(&compressed,&cfg.vehicle,strict_taem_velocity);
    entry_alpha_schedule_default(&decoupled,&cfg.vehicle,alpha_transition);
    double compressed_alpha=entry_alpha_schedule_interpolate(&compressed,2106.0);
    double decoupled_alpha=entry_alpha_schedule_interpolate(&decoupled,2106.0);
    assert(compressed_alpha>26.9);
    assert(decoupled_alpha>24.8&&decoupled_alpha<25.3);
    assert(decoupled_alpha<compressed_alpha-1.5);
}

static void recorded_replay(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();
    AerodynamicModel aero={.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=.8};
    GuidanceMachine g={0};unsigned count=0,conflicts=0;double worst=0,first_conflict=NAN;
    double first_schedule=NAN,first_schedule_deadline=NAN,first_reversal=NAN;
    double values[33];
    while(scanf("%lf",&values[0])==1){
        for(int j=1;j<33;j++)assert(scanf("%lf",&values[j])==1);
        Telemetry t;telemetry_init(&t);int j=0;
        t.ut=values[j++];t.mean_altitude=values[j++];t.radar_altitude=t.mean_altitude;
        t.true_air_speed=values[j++];t.horizontal_speed=values[j++];t.vertical_speed=values[j++];
        t.flight_path_angle=values[j++];t.roll=values[j++];t.angle_of_attack=values[j++];
        t.pitch=values[j++];t.roll_rate=values[j++];t.angle_of_attack_rate=values[j++];
        t.has_angle_of_attack_rate=true;t.dynamic_pressure=values[j++];
        t.lift_force=values[j++];t.drag_force=values[j++];t.mass=values[j++];t.g_force=values[j++];
        t.stall_fraction=values[j++];t.stall_fraction_is_measured=values[j++]!=0;
        t.runway_along_track=values[j++];t.runway_cross_track=values[j++];
        t.ground_track_heading=values[j++];t.heading=values[j++];t.latitude=values[j++];t.longitude=values[j++];
        t.range_to_site=values[j++];t.mach=values[j++];t.bank_effectiveness=1;t.aerodynamic_confidence=.8;
        VehicleState state={.ut=t.ut,.mass=t.mass};
        state.position=v3(values[j],values[j+1],values[j+2]);j+=3;
        state.velocity=v3(values[j],values[j+1],values[j+2]);j+=3;
        double old_compute=values[j];
        if(!count){g=machine(&t,&p,&cfg);g.has_s_turn_leg_started=false;}
        unsigned reversals_before=g.entry_control_reversals;
        bool final_before=g.entry_final_reversal_pending||g.entry_final_reversal_completed;
        double start=seconds();
        GuidanceResult r=entry_program_guidance(&g,&t,&state,t.ground_track_heading,&p,aero,&cfg,.1);
        double wall=seconds()-start;worst=fmax(worst,wall);
        if(!isfinite(first_schedule)&&g.entry_reversal_scheduled){
            first_schedule=t.ut;first_schedule_deadline=g.entry_reversal_ut;
        }
        bool final_now=g.entry_final_reversal_pending||g.entry_final_reversal_completed;
        if(!isfinite(first_reversal)&&
           (g.entry_control_reversals>reversals_before||(!final_before&&final_now)))
            first_reversal=t.ut;
        assert(wall<.1&&isfinite(g.entry_control_bank)&&isfinite(g.entry_control_aoa));
        assert(fabs(g.entry_control_bank)<=dynamic_bank_limit(&t,&cfg.vehicle)+1e-9);
        if(g.entry_lateral_infeasible){if(!conflicts)first_conflict=t.ut;conflicts++;assert(!g.entry_control_terminal_ready);}
        if(t.ut>67423.4&&t.ut<67423.6)
            printf("early v32 divergence UT %.6f: executable bank %.2f, geometry conflict %d\n",
                t.ut,g.entry_control_bank,g.entry_lateral_infeasible);
        if(old_compute>1800&&t.ut>67559&&t.ut<67560)
            printf("recorded UT %.6f: old compute %.3f ms; replay %.3f ms; bank %.2f; conflict %d\n",
                t.ut,old_compute,wall*1000,g.entry_control_bank,g.entry_lateral_infeasible);
        guidance_result_clear(&r);count++;
    }
    assert(count>100);
    printf("recorded replay: %u MM304 samples, worst %.3f ms, %u explicit delivery conflicts (first UT %.3f), first schedule %.3f deadline %.3f, first reversal %.3f\n",
        count,worst*1000,conflicts,first_conflict,first_schedule,first_schedule_deadline,first_reversal);
}
static void test_pose_aware_setup_builds_terminal_crossrange(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    t.ground_track_heading=t.heading=90.0;
    GuidanceMachine g=machine(&t,&p,&cfg);
    double setup_along=NAN,setup_cross=NAN,final_side=0.0;
    assert(entry_taem_pose_setup_point(&g,&t,&p,&cfg,&setup_along,&setup_cross,&final_side));
    double radius=entry_taem_shaping_setup_radius(&g.taem_interface_target,&t,&p,&cfg);
    assert(isfinite(radius)&&radius>cfg.guidance.hac_radius);
    /* The setup radius is geometry-derived; do not compare it against the
       removed fixed HAC-entry heuristic. The invariant above is sufficient:
       pose setup must expand beyond the configured HAC radius when required. */
    assert(final_side*g.s_turn_sign<0.0);
    double setup_psi0=norm_signed_deg(t.ground_track_heading-cfg.site.runway_heading)*DEG2RAD;
    double setup_psi1=norm_signed_deg(g.taem_interface_target.course-cfg.site.runway_heading)*DEG2RAD;
    double expected_along=g.taem_interface_target.along_track-radius/final_side*(sin(setup_psi1)-sin(setup_psi0));
    double expected_cross=g.taem_interface_target.cross_track-radius/final_side*(cos(setup_psi0)-cos(setup_psi1));
    assert(fabs(setup_along-expected_along)<1e-6);
    assert(fabs(setup_cross-expected_cross)<1e-6);
    assert(setup_along<g.taem_interface_target.along_track);
    assert(g.s_turn_sign*(setup_cross-g.taem_interface_target.cross_track)>0.0);

    /* Reproduce the important live geometry near the 1.3 km/s energy target: about
       58 km upstream but with only a few km of crossrange. The pose-aware fallback
       must keep shaping on the owned side rather than chase the -8 km point directly. */
    t.mean_altitude=t.radar_altitude=27100.0;t.true_air_speed=1309.0;
    t.horizontal_speed=1295.0;t.vertical_speed=-190.0;t.flight_path_angle=-8.3;
    t.runway_along_track=-58000.0;t.runway_cross_track=6000.0;
    t.ground_track_heading=t.heading=86.0;t.dynamic_pressure=9000.0;
    t.lift_force=t.mass*7.0;t.drag_force=t.mass*4.0;t.bank_effectiveness=1.0;
    AerodynamicModel aero={.lift_to_drag=1.75,.ballistic_coefficient=700,.confidence=.9};
    assert(!entry_program_final_reversal_geometry_ready(&g,&t,&cfg,final_side));
    double bank=entry_taem_gate_required_bank(&g,&t,&p,aero,&cfg);
    assert(bank*g.s_turn_sign>0.0&&fabs(bank)>=20.0);
    assert(fabs(bank)<=dynamic_bank_limit(&t,&cfg.vehicle)+1e-9);
}

static void test_overdue_terminal_geometry_retains_lateral_authority(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);
    /* Reproduce the overdue terminal-geometry state near the old 1.3 km/s inlet,
       but keep it genuinely outside every flyable endpoint circle. The previous
       +16 km crossrange fixture is only ~12 km normal to a valid ~100 km arc after
       the MM304 radius-cap fix, so it is no longer a blocked state. */
    t.ut=200.0;t.mean_altitude=t.radar_altitude=26600.0;t.true_air_speed=1298.0;
    t.horizontal_speed=1280.0;t.ground_track_heading=t.heading=100.0;
    t.runway_along_track=-100000.0;t.runway_cross_track=-6000.0;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);
    t.dynamic_pressure=12000.0;t.lift_force=t.mass*5.0;t.drag_force=t.mass*3.0;
    t.g_force=1.4;t.stall_fraction=0.0;t.stall_fraction_is_measured=true;t.bank_effectiveness=.95;
    double path_distance=NAN;
    double path_fpa=entry_program_taem_path_fpa(&g,&t,&cfg,&path_distance);
    assert(isfinite(path_fpa)&&isfinite(path_distance));
    t.flight_path_angle=path_fpa-1.0;
    t.vertical_speed=t.true_air_speed*sin(t.flight_path_angle*DEG2RAD);
    g.entry_reversal_scheduled=true;g.entry_reversal_is_final=false;
    double terminal_side=norm_signed_deg(g.taem_interface_target.course-
        cfg.site.runway_heading)>=0.0?1.0:-1.0;
    g.entry_reversal_sign=terminal_side;g.entry_reversal_ut=t.ut-50.0;
    g.has_s_turn_leg_started=true;g.s_turn_leg_started_ut=t.ut-100.0;
    assert(entry_program_terminal_geometry_blocked(&g,&t,&cfg));
    double vertical=entry_program_vertical_bank_ceiling(&g,&t,&p,
        (AerodynamicModel){.lift_to_drag=2.0,.ballistic_coefficient=700,.confidence=.9},&cfg);
    assert(isfinite(vertical)&&vertical<cfg.vehicle.maximum_bank_angle-5.0);
    EntryControlPlan plan={.valid=true,.target_bank=vertical,.target_aoa=18.0,
        .bank_cap=cfg.vehicle.maximum_bank_angle};
    entry_program_apply_geometry_bank_demand(&g,&t,&p,
        (AerodynamicModel){.lift_to_drag=2.0,.ballistic_coefficient=700,.confidence=.9},&cfg,45.0,&plan);
    /* Geometry cannot spend vertical lift that the live altitude envelope
       has already reserved.  Retain the owned turn at the feasible ceiling. */
    assert(fabs(plan.target_bank)<=vertical+1e-9);
    assert(plan.target_bank*g.s_turn_sign>=0.0);
    assert(fabs(plan.target_bank)<=dynamic_bank_limit(&t,&cfg.vehicle)+1e-9);
}

static void test_post_shaping_parallel_stage_preserves_setup_crossrange(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);
    AerodynamicModel aero={.lift_to_drag=1.3,.ballistic_coefficient=700,.confidence=.9};
    /* The handoff target is now the strict perpendicular KSC09 inlet.  First
       exercise the high-speed state that the old future-HAC-radius shortcut
       incorrectly admitted: at ~1.1 km/s its measured-lift turn radius is far
       larger than the requested endpoint circle, so MM304 must keep staging. */
    g.taem_interface_target.valid=true;
    g.taem_interface_target.along_track=-cfg.guidance.final_approach_distance;
    g.taem_interface_target.cross_track=0.0;
    g.taem_interface_target.course=norm_deg(cfg.site.runway_heading-90.0);
    g.taem_interface_target.speed=640.3;
    g.taem_interface_target.response_time=8.0;
    g.s_turn_sign=-1.0;g.entry_control_reversals=1;
    g.entry_reversal_scheduled=false;g.entry_final_reversal_pending=false;
    g.entry_final_reversal_completed=false;
    t.mean_altitude=t.radar_altitude=24466.0;t.true_air_speed=1112.1;
    t.horizontal_speed=1107.0;t.vertical_speed=-145.0;t.flight_path_angle=-7.5;
    t.runway_along_track=-96300.0;t.runway_cross_track=25000.0;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);
    t.ground_track_heading=t.heading=96.0;t.dynamic_pressure=10000.0;
    t.lift_force=t.mass*8.0;t.drag_force=t.mass*5.0;t.g_force=1.2;
    t.bank_effectiveness=.97;t.stall_fraction=0.0;t.stall_fraction_is_measured=true;
    assert(!entry_program_final_reversal_geometry_ready(&g,&t,&cfg,-1.0));
    double lead_bank=NAN;
    assert(entry_program_post_shaping_parallel_stage(&g,&t,t.ground_track_heading,
        &p,aero,&cfg,&lead_bank));
    assert(isfinite(lead_bank));
    assert(lead_bank<=.1);
    assert(fabs(lead_bank)<=dynamic_bank_limit(&t,&cfg.vehicle)+1e-9);

    /* After deceleration, construct a physically flyable ~70 km perpendicular
       arc whose endpoint is exactly the fixed -8 km / zero-cross-track station.
       Admission starts the MM304 final turn only; it does not relax the <=1 km
       nominal TAEM capture contract. */
    t.true_air_speed=600.0;t.horizontal_speed=596.0;
    t.vertical_speed=-70.0;t.flight_path_angle=-6.7;
    t.runway_along_track=-78000.0;t.runway_cross_track=70000.0;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);
    t.ground_track_heading=t.heading=90.0;
    double bank=NAN;
    assert(entry_program_final_reversal_geometry_ready(&g,&t,&cfg,-1.0));
    assert(!entry_program_post_shaping_parallel_stage(&g,&t,t.ground_track_heading,
        &p,aero,&cfg,&bank));
}

static void test_v27_rollthrough_projection_closes_endpoint(void){
    LandingConfiguration cfg=configuration();PlanetModel p=planet();Telemetry t=sample();
    GuidanceMachine g=machine(&t,&p,&cfg);g.diagnostic_shadow=false;
    g.taem_interface_target.valid=true;
    g.taem_interface_target.along_track=-8000.0;g.taem_interface_target.cross_track=0.0;
    g.taem_interface_target.altitude=20000.0;g.taem_interface_target.speed=640.3;
    g.taem_interface_target.course=0.0;g.taem_interface_target.flight_path_angle=-12.0;
    g.s_turn_sign=1.0;g.entry_reversal_scheduled=true;g.entry_reversal_is_final=false;
    g.entry_reversal_sign=-1.0;g.entry_reversal_bank=50.0;g.entry_reversal_ut=100.0;
    g.has_s_turn_leg_started=true;g.s_turn_leg_started_ut=20.0;
    t.ut=130.0;t.mean_altitude=t.radar_altitude=26500.0;
    t.true_air_speed=550.0;t.horizontal_speed=546.15;
    t.flight_path_angle=-6.8;t.vertical_speed=t.true_air_speed*sin(t.flight_path_angle*DEG2RAD);
    t.mass=43729.0;t.lift_force=t.mass*16.0;t.drag_force=t.mass*8.0;
    t.dynamic_pressure=10500.0;t.g_force=1.25;t.bank_effectiveness=.96;
    t.stall_fraction=0.0;t.stall_fraction_is_measured=true;
    t.ground_track_heading=t.heading=105.0;t.course_rate=.23;
    t.roll=49.0;t.roll_rate=0.0;
    t.runway_along_track=-125000.0;t.runway_cross_track=75000.0;
    t.range_to_site=hypot(t.runway_along_track,t.runway_cross_track);

    /* The instantaneous measured circle is still outside the endpoint tube.
       The HAC-sized shaping offset is already satisfied, while projecting the
       +49 -> -bank roll transient independently closes the physical 0 deg
       endpoint before the instantaneous test does. */
    assert(!entry_program_final_reversal_geometry_ready(&g,&t,&cfg,-1.0));
    double built=NAN,required=NAN;
    assert(entry_taem_shaping_crossrange_ready(&g.taem_interface_target,&t,&p,&cfg,-1.0,&built,&required));
    double projected_error=INFINITY,lead=NAN;
    assert(entry_program_rollthrough_endpoint_ready(&g,&t,&p,&cfg,-1.0,&projected_error,&lead));
    assert(isfinite(projected_error)&&projected_error<=2500.0);
    assert(isfinite(lead)&&lead>12.0&&lead<30.1);
    assert(entry_program_execute_planned_reversal(&g,&t,&p,&cfg));
    assert(g.s_turn_sign<0.0&&g.entry_control_reversals==1&&!g.entry_reversal_scheduled);
}

int main(int argc,char**argv){
    if(argc==2&&strcmp(argv[1],"--replay")==0){recorded_replay();return 0;}
    test_commit_and_adoption();test_propagated_shaping_event_timing_and_terminal_reachability();test_same_side_nonterminal_event_cannot_masquerade_as_reversal();test_joint_lift_budget();test_opposite_geometry_requires_a_committed_event();test_deferred_control_and_final_arc();test_post_reversal_course_capture_survives_vertical_ceiling();test_overdue_terminal_side_setup_preserves_energy();test_final_s_turn_drag_aoa_extension();test_two_event_topology_installs_shaping_reversal();test_valid_topology_replaces_only_unexecuted_ordinary_reversal();test_worker_topology_replaces_matching_live_ordinary_reversal();test_strict_taem_speed_does_not_compress_entry_alpha_schedule();test_pose_aware_setup_builds_terminal_crossrange();test_overdue_terminal_geometry_retains_lateral_authority();test_post_shaping_parallel_stage_preserves_setup_crossrange();test_v27_rollthrough_projection_closes_endpoint();
    puts("PASS: MM304 offline seam contracts");return 0;
}
