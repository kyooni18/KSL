#include "landing.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static PlanetModel kerbin(void){
    PlanetModel p;memset(&p,0,sizeof(p));
    snprintf(p.name,sizeof(p.name),"Kerbin");
    p.radius=600000;p.gravitational_parameter=3.5316e12;
    p.rotational_speed=2*LANDER_PI/21549.425;p.atmosphere_depth=70000;
    p.surface_density=1.225;p.atmosphere_adiabatic_index=1.4;
    p.north_axis=v3(0,0,1);p.prime_meridian_at_epoch=v3(1,0,0);
    return p;
}

static void test_spherical_planet_geometry_and_gravity(void){
    PlanetModel p=kerbin();
    GeoPoint origin={-0.0486111111,-74.7283333333,70.0};
    GeoPoint remote=destination_point(origin,217.0,420000.0,p.radius);remote.altitude=53000.0;
    double east=0.0,north=0.0;local_offsets(origin,remote,p.radius,&east,&north);
    assert(fabs(hypot(east,north)-420000.0)<1e-3);
    assert(fabs(norm_signed_deg(atan2(east,north)*RAD2DEG-217.0))<1e-8);
    GeoPoint roundtrip=local_point(origin,east,north,p.radius,remote.altitude);
    assert(great_circle_distance(roundtrip,remote,p.radius)<1e-3);
    assert(fabs(roundtrip.altitude-remote.altitude)<1e-9);

    Vector3 surface=v3(p.radius,0,0),high=v3(p.radius+52820.0,0,0);
    double surface_g=vmag(vessel_physics_gravity(surface,&p));
    double high_g=vmag(vessel_physics_gravity(high,&p));
    assert(fabs(surface_g-p.gravitational_parameter/(p.radius*p.radius))<1e-10);
    assert(fabs(high_g-p.gravitational_parameter/((p.radius+52820.0)*(p.radius+52820.0)))<1e-10);
    assert(high_g<surface_g);
    assert(surface_g>9.7&&surface_g<9.9);
    assert(high_g>8.2&&high_g<8.5);
}

static PlanetModel recorded_1605_planet(void){
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;
    p.surface_density=1.1399229405107;
    p.epoch_ut=67555.1925486105;
    p.north_axis=v3(0,0,1);
    /* Reconstructed from the exact persisted inertial position plus the same
       frame's logged latitude/longitude. Future split logs persist this basis
       directly so replay no longer needs reconstruction. */
    p.prime_meridian_at_epoch=v3(-0.7496760227567124,0.6618050021748682,0);
    return p;
}

static void test_planet_replay_metadata_is_complete(void){
    PlanetModel p=recorded_1605_planet();
    p.atmosphere_sample_count=2;
    p.atmosphere_altitude[0]=0;p.atmosphere_pressure[0]=101325;p.atmosphere_density[0]=1.1399229405107;
    p.atmosphere_altitude[1]=70000;p.atmosphere_pressure[1]=0;p.atmosphere_density[1]=0;
    JsonWriter w;jw_init(&w);planet_model_replay_json(&w,&p);
    assert(!w.failed&&w.data);
    assert(strstr(w.data,"\"northAxis\":[0,0,1]"));
    assert(strstr(w.data,"\"primeMeridianAtEpoch\""));
    assert(strstr(w.data,"\"epochUT\":67555.1925486105"));
    assert(strstr(w.data,"\"atmosphereAdiabaticIndex\":1.4"));
    assert(strstr(w.data,"\"atmosphereSamples\":[{"));
    assert(strstr(w.data,"\"pressure\":101325"));
    jw_free(&w);
}

static VehicleState terminal_state_at_range(const PlanetModel*p,const LandingSite*site,double range){
    GeoPoint threshold={site->latitude,site->longitude,site->altitude};
    GeoPoint point=destination_point(threshold,norm_deg(site->runway_heading+180.0),range,p->radius);
    point.altitude=16500;
    Vector3 position=predictor_inertial_position(point,p,200);
    Vector3 up=vnorm(position,v3(1,0,0));
    Vector3 north=vnorm(vproject_plane(p->north_axis,up),v3(0,0,1));
    Vector3 east=vnorm(vcross(north,up),v3(0,1,0));
    double h=site->runway_heading*DEG2RAD,vertical=-30.0,speed=600.0;
    Vector3 direction=vnorm(vadd(vscale(north,cos(h)),vscale(east,sin(h))),east);
    Vector3 air=vadd(vscale(direction,sqrt(speed*speed-vertical*vertical)),vscale(up,vertical));
    VehicleState state={200,position,vadd(air,vcross(planet_rotation_vector(p),position)),20000};
    return state;
}

static VehicleState entry_state_at_range(const PlanetModel*p,const LandingSite*site,double range,double altitude,double speed){
    GeoPoint threshold={site->latitude,site->longitude,site->altitude};
    GeoPoint point=destination_point(threshold,norm_deg(site->runway_heading+180.0),range,p->radius);
    point.altitude=altitude;
    Vector3 position=predictor_inertial_position(point,p,200);
    Vector3 up=vnorm(position,v3(1,0,0));
    Vector3 north=vnorm(vproject_plane(p->north_axis,up),v3(0,0,1));
    Vector3 east=vnorm(vcross(north,up),v3(0,1,0));
    double h=site->runway_heading*DEG2RAD,vertical=-60.0;
    Vector3 direction=vnorm(vadd(vscale(north,cos(h)),vscale(east,sin(h))),east);
    Vector3 air=vadd(vscale(direction,sqrt(fmax(0.0,speed*speed-vertical*vertical))),vscale(up,vertical));
    VehicleState state={200,position,vadd(air,vcross(planet_rotation_vector(p),position)),20000};
    return state;
}

static VehicleState terminal_state(const PlanetModel*p,const LandingSite*site){
    return terminal_state_at_range(p,site,20000.0);
}

static void predictor_models(const LandingConfiguration*cfg,AerodynamicModel*aero,
        AerodynamicEnvelope*env,TrajectoryCalibrationModel*cal){
    *aero=(AerodynamicModel){.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=.6};
    for(int i=0;i<4;i++)env->regimes[i]=*aero;
    *cal=(TrajectoryCalibrationModel){
        .density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,
        .speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.2
    };
    (void)cfg;
}

static Telemetry shadow_entry_telemetry(const VehicleState*state,const PlanetModel*p,
        const LandingConfiguration*cfg,double initial_aoa){
    Telemetry t;telemetry_init(&t);
    GeoPoint geo=predictor_geo_point(state->position,p,state->ut);
    GeoPoint target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    Vector3 up=vnorm(state->position,v3(1,0,0));
    Vector3 air=vsub(state->velocity,vcross(planet_rotation_vector(p),state->position));
    double vertical=vdot(air,up),horizontal=vmag(vproject_plane(air,up));
    double fallback=initial_bearing(geo,target);
    double course=surface_course(state->position,state->velocity,planet_rotation_vector(p),p->north_axis,fallback);
    double along=0,cross=0;runway_coordinates(geo,target,cfg->site.runway_heading,p->radius,&along,&cross);
    t.ut=state->ut;t.latitude=geo.latitude;t.longitude=geo.longitude;t.mean_altitude=geo.altitude;
    t.radar_altitude=fmax(0.0,geo.altitude-cfg->site.altitude);t.vertical_speed=vertical;t.horizontal_speed=horizontal;
    t.true_air_speed=vmag(air);t.surface_speed=horizontal;t.flight_path_angle=atan2(vertical,fmax(horizontal,.1))*RAD2DEG;
    t.heading=course;t.ground_track_heading=course;t.bearing_to_site=fallback;t.heading_error=norm_signed_deg(fallback-course);
    t.course_to_site_error=t.heading_error;t.range_to_site=great_circle_distance(geo,target,p->radius);
    t.runway_along_track=along;t.runway_cross_track=cross;t.roll=0.0;t.roll_rate=0.0;
    t.angle_of_attack=initial_aoa;t.has_angle_of_attack_rate=true;t.angle_of_attack_rate=0.0;t.pitch=t.flight_path_angle+initial_aoa;
    t.mass=state->mass;t.bank_effectiveness=1.0;t.aerodynamic_confidence=.7;t.physics_confidence=.7;
    t.trajectory_density_scale=1.0;t.trajectory_drag_scale=1.0;t.trajectory_lift_scale=1.0;t.trajectory_calibration_confidence=.7;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    return t;
}

static DeorbitPlan shadow_continuation_plan(const Telemetry*t){
    DeorbitPlan plan;memset(&plan,0,sizeof(plan));trajectory_init(&plan.trajectory);
    plan.created_ut=t->ut;plan.burn_ut=t->ut-1.0;plan.delta_v=0.0;plan.estimated_burn_duration=0.0;
    plan.predicted_post_burn_periapsis_altitude=50000.0;
    plan.live_cutoff_capture_qualified=true;plan.execution_qualified=true;
    plan.achieved_state_verified=true;plan.achieved_state_capture_qualified=true;
    return plan;
}

static EntrySupervisionResult supervise_terminal(const PlanetModel*p,const LandingConfiguration*cfg,
        AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        VehicleState state,const EntryControlPlan*nominal,double bank_bound,double aoa_bound){
    return predictor_supervise_entry_control(state,p,aero,env,cal,&cfg->vehicle,&cfg->site,&cfg->guidance,
        0,0,10,0,1,false,0,0,false,false,nominal,bank_bound,aoa_bound,500);
}

static void test_assessment_flags_safety_and_terminal_feasibility(void){
    LandingConfiguration cfg=landing_configuration_default();
    EntryControlPlan plan={.valid=true,.target_bank=20,.target_aoa=18,.taem_speed=cfg.guidance.taem_force_handoff_speed};
    EntryPrediction pr;memset(&pr,0,sizeof(pr));
    pr.entered_atmosphere=true;pr.reached_taem=true;
    pr.peak_dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure*.8;
    pr.peak_g_load=cfg.vehicle.maximum_g_load*.8;
    pr.minimum_entry_speed=cfg.vehicle.minimum_safe_speed*2;
    pr.maximum_abs_angle_of_attack=18;
    pr.minimum_bank_control_margin=5;pr.minimum_aoa_control_margin=5;
    pr.taem_energy_error=1.0;

    EntryPredictionAssessment a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.valid&&a.safe&&a.terminal_feasible);

    pr.taem_ownership_boundary_missed=true;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.safe&&!a.terminal_feasible);
    pr.taem_ownership_boundary_missed=false;

    pr.taem_energy_error=-1.0;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.safe&&!a.terminal_feasible);
    pr.taem_energy_error=1.0;

    pr.peak_dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure*1.01;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.dynamic_pressure_violation&&!a.safe);
    pr.peak_dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure*.8;

    pr.peak_g_load=cfg.vehicle.maximum_g_load*1.01;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.g_load_violation&&!a.safe);
    pr.peak_g_load=cfg.vehicle.maximum_g_load*.8;

    pr.minimum_entry_speed=cfg.vehicle.minimum_safe_speed*.95;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.stall_risk&&!a.safe);
    pr.minimum_entry_speed=cfg.vehicle.minimum_safe_speed*2;

    pr.minimum_bank_control_margin=-.1;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.control_margin_violation&&!a.safe);
    pr.minimum_bank_control_margin=5;

    pr.reached_taem=false;
    a=predictor_assess_entry_prediction(&pr,&cfg.vehicle,45,&plan);
    assert(a.safe&&!a.terminal_feasible);
}

static void test_supervisor_passes_nominal_and_is_deterministic(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=terminal_state(&p,&cfg.site);
    EntryControlPlan nominal={
        .valid=true,.planned_ut=state.ut,.target_bank=0,.target_aoa=10,.target_heading=cfg.site.runway_heading,
        .bank_cap=0,.target_turn_radius=INFINITY,.segment_duration=24,.cost=0
    };

    EntrySupervisionResult a=supervise_terminal(&p,&cfg,aero,&env,&cal,state,&nominal,0,0);
    EntrySupervisionResult b=supervise_terminal(&p,&cfg,aero,&env,&cal,state,&nominal,0,0);
    assert(a.valid&&a.mode==ENTRY_SUPERVISION_PASS_THROUGH);
    assert(a.nominal_assessment.safe&&a.nominal_assessment.terminal_feasible);
    assert(b.valid&&b.mode==ENTRY_SUPERVISION_PASS_THROUGH);
    assert(fabs(a.plan.target_bank-nominal.target_bank)<1e-12);
    assert(fabs(a.plan.target_aoa-nominal.target_aoa)<1e-12);
    assert(fabs(norm_signed_deg(a.plan.target_heading-nominal.target_heading))<1e-12);
    assert(a.bank_correction==0&&a.aoa_correction==0);
    assert(a.mode==b.mode);
    assert(fabs(a.plan.target_bank-b.plan.target_bank)<1e-12);
    assert(fabs(a.plan.target_aoa-b.plan.target_aoa)<1e-12);
    assert(fabs(norm_signed_deg(a.plan.target_heading-b.plan.target_heading))<1e-12);
    assert(a.plan.predicted_reversals==b.plan.predicted_reversals);
    PredictorPlannerTrace trace;
    assert(predictor_last_planner_trace(&trace));
    assert(trace.valid&&trace.mode==ENTRY_SUPERVISION_PASS_THROUGH);
    assert(trace.candidate_count>=1&&trace.candidate_count<=PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES);
    assert(trace.selected_index>=0&&(unsigned)trace.selected_index<trace.candidate_count);
    assert(trace.candidates[trace.selected_index].selected);
    assert(trace.candidates[trace.selected_index].source==PREDICTOR_PLAN_CANDIDATE_NOMINAL);
    assert(trace.candidates[trace.selected_index].rejection_flags==PREDICTOR_CANDIDATE_REJECT_NONE);
    assert(trace.candidate_budget==PREDICTOR_PLANNER_TRACE_MAX_CANDIDATES);
    assert(trace.dropped_candidate_count==0&&trace.selection_converged);
    assert(fabs(trace.search_horizon_seconds-500.0)<1e-12);
    const PredictorPlannerCandidateTrace *selected=&trace.candidates[trace.selected_index];
    assert(selected->trajectory_sample_count>0&&selected->trajectory_sample_count<=PREDICTOR_PLANNER_TRACE_MAX_PATH_POINTS);
    assert(selected->trajectory_total_points>=selected->trajectory_sample_count);
    assert(fabs(trace.candidates[trace.selected_index].plan.target_bank-b.plan.target_bank)<1e-12);
    assert(fabs(trace.candidates[trace.selected_index].plan.target_aoa-b.plan.target_aoa)<1e-12);
}

static void test_supervisor_correction_is_bounded_and_preserves_lateral_semantics(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=terminal_state(&p,&cfg.site);
    EntryControlPlan nominal={
        .valid=true,.planned_ut=state.ut,.target_bank=0,
        .target_aoa=cfg.vehicle.maximum_angle_of_attack+1.0,.target_heading=cfg.site.runway_heading,
        .bank_cap=0,.target_turn_radius=INFINITY,.segment_duration=24,.cost=0,
        .has_planned_reversal=true,.planned_reversal_ut=state.ut+12,
        .planned_reversal_range=50000,.planned_reversal_sign=-1
    };
    EntrySupervisionResult corrected=supervise_terminal(&p,&cfg,aero,&env,&cal,state,&nominal,0,2);
    assert(corrected.valid&&corrected.mode==ENTRY_SUPERVISION_BOUNDED_CORRECTION);
    assert(corrected.nominal_assessment.control_margin_violation);
    assert(fabs(corrected.bank_correction)<1e-12);
    assert(fabs(corrected.aoa_correction)<=2.0+1e-12);
    assert(corrected.plan.target_aoa<=cfg.vehicle.maximum_angle_of_attack+1e-12);
    assert(fabs(corrected.plan.target_bank-nominal.target_bank)<1e-12);
    assert(fabs(norm_signed_deg(corrected.plan.target_heading-nominal.target_heading))<1e-12);
    assert(corrected.plan.has_planned_reversal==nominal.has_planned_reversal);
    assert(corrected.plan.planned_reversal_sign==nominal.planned_reversal_sign);
    PredictorPlannerTrace trace;assert(predictor_last_planner_trace(&trace));
    assert(trace.selected_index>=0&&trace.candidates[trace.selected_index].selected);
    assert(trace.candidates[trace.selected_index].rejection_flags==PREDICTOR_CANDIDATE_REJECT_NONE);
    bool saw_explicit_rejection=false;
    for(unsigned i=0;i<trace.candidate_count;i++)if((int)i!=trace.selected_index&&trace.candidates[i].rejection_flags!=PREDICTOR_CANDIDATE_REJECT_NONE)saw_explicit_rejection=true;
    assert(saw_explicit_rejection);
}

static void test_safe_nominal_keeps_ownership_when_terminal_proof_is_unavailable(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    /* Keep this case genuinely upstream of the one-way TAEM ownership boundary:
       the five-second supervision horizon should end while the vehicle is still
       well above V_TAEM, so lack of a terminal proof means "not observed yet",
       not a demonstrated handoff miss. */
    VehicleState state=entry_state_at_range(&p,&cfg.site,140000.0,30000.0,1800.0);
    EntryControlPlan nominal={
        .valid=true,.planned_ut=state.ut,.target_bank=0,.target_aoa=10,
        .target_heading=cfg.site.runway_heading,.segment_duration=12
    };
    EntrySupervisionResult result=predictor_supervise_entry_control(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        0,0,10,0,1,false,0,0,false,false,&nominal,0,0,5);
    assert(result.nominal_assessment.valid&&result.nominal_assessment.safe);
    assert(!result.nominal_assessment.terminal_feasible);
    assert(result.valid);
    if(result.mode==ENTRY_SUPERVISION_FALLBACK){
        assert(result.selected_assessment.terminal_feasible);
    }else{
        assert(result.mode==ENTRY_SUPERVISION_PASS_THROUGH);
        assert(fabs(result.plan.target_bank-nominal.target_bank)<1e-12);
        assert(fabs(result.plan.target_aoa-nominal.target_aoa)<1e-12);
    }
}

static void test_unsafe_nominal_cannot_escape_zero_correction_envelope(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=terminal_state(&p,&cfg.site);
    EntryControlPlan nominal={
        .valid=true,.planned_ut=state.ut,.target_bank=0,
        .target_aoa=cfg.vehicle.maximum_angle_of_attack+8.0,.target_heading=cfg.site.runway_heading,
        .segment_duration=24
    };
    EntrySupervisionResult result=supervise_terminal(&p,&cfg,aero,&env,&cal,state,&nominal,0,0);
    assert(result.nominal_assessment.valid&&result.nominal_assessment.control_margin_violation);
    /* A zero correction envelope is an absolute contract. The legacy broad
       planner may be advisory, but it cannot become an unbounded live command. */
    assert(!result.valid);
    assert(result.mode==ENTRY_SUPERVISION_INFEASIBLE);
    assert(fabs(result.bank_correction)<1e-12);
    assert(fabs(result.aoa_correction)<1e-12);
}

static void test_preburn_plan_state_guard_uses_certified_robustness_envelope(void){
    LandingConfiguration cfg=landing_configuration_default();
    DeorbitPlan plan;memset(&plan,0,sizeof(plan));
    plan.planning_mass=20000.0;
    plan.planning_available_thrust=820000.0;
    assert(deorbit_plan_preburn_state_compatible(&plan,20000.0,820000.0,&cfg.guidance));
    assert(deorbit_plan_preburn_state_compatible(&plan,20000.0*(1.0+cfg.guidance.deorbit_mass_uncertainty_fraction),820000.0,&cfg.guidance));
    assert(!deorbit_plan_preburn_state_compatible(&plan,20000.0*(1.0+cfg.guidance.deorbit_mass_uncertainty_fraction+.001),820000.0,&cfg.guidance));
    /* 03:04:46Z plan was certified at 20 t, but the settled pre-burn vessel was
       40.7 t. That state must never retain an executable deorbit plan. */
    assert(!deorbit_plan_preburn_state_compatible(&plan,40710.125,820000.0,&cfg.guidance));
    assert(deorbit_plan_preburn_state_compatible(&plan,20000.0,820000.0*(1.0-cfg.guidance.deorbit_thrust_uncertainty_fraction),&cfg.guidance));
    assert(!deorbit_plan_preburn_state_compatible(&plan,20000.0,820000.0*(1.0-cfg.guidance.deorbit_thrust_uncertainty_fraction-.001),&cfg.guidance));
}

static __attribute__((unused)) void test_recorded_0304_achieved_state_is_mm304_qualified_but_not_terminal_capture(void){
    LandingConfiguration cfg=landing_configuration_default();
    VehicleState state={
        64534.9916109235,
        {-621382.821157298,-292443.495989256,9.56258061856282},
        {954.642543517787,-2021.90037628325,0.0189350157342116},
        40274.37890625
    };
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;
    p.surface_density=1.1399229405107;
    p.epoch_ut=state.ut;
    Vector3 up=vnorm(state.position,v3(1,0,0));
    Vector3 equatorial=vnorm(vproject_plane(up,p.north_axis),v3(1,0,0));
    p.prime_meridian_at_epoch=vrotate(equatorial,p.north_axis,-117.095737926395*DEG2RAD);

    AerodynamicModel aero={.lift_to_drag=.379080762401409,.ballistic_coefficient=479.684293897776,.confidence=.9};
    AerodynamicEnvelope env;for(int i=0;i<4;i++)env.regimes[i]=aero;
    TrajectoryCalibrationModel cal={
        .density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,
        .speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.confidence=.05
    };
    EntryPrediction pr=predictor_simulate_entry_with_attitude(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        cfg.vehicle.maximum_bank_angle,.4404296875,-.0544158694975694,.589056730270386,0,1,0,4500,true);
    double peri=predictor_postburn_periapsis(state,&p);
    assert(pr.entered_atmosphere);
    assert(fabs(peri-49585.06)<150.0);
    assert(isfinite(pr.closest_distance));
    assert(pr.entry_range>900000.0&&pr.entry_range<1030000.0);
    assert(pr.entry_flight_path_angle>-1.9&&pr.entry_flight_path_angle<-1.6);
    assert(!pr.reached_taem);
    assert(deorbit_capture_qualified(&pr,&cfg.site,&cfg.vehicle,&cfg.guidance,peri,true));

    /* This state is still a coarse deorbit/MM304 delivery, not a terminal-capture
       proof. TAEM remains unreached, so exact nearest-runway replay distances are
       not part of the safety contract. */
    entry_prediction_clear(&pr);
}

static TaemTerminalContract shadow_terminal_contract(void){
    TaemTerminalContract c;memset(&c,0,sizeof(c));
    c.valid=true;c.path_committed=true;c.range_to_go=30000;c.range_margin=5000;
    c.dynamic_pressure=8000;c.dynamic_pressure_margin=15000;
    c.speedbrake_available=true;c.speedbrake_dynamic_pressure_margin=12000;
    c.altitude=15000;c.altitude_margin=2000;c.flight_path_angle=-5;c.flight_path_angle_margin=4;
    c.specific_energy=500000;c.specific_energy_margin=50000;
    c.response_time_available=20;c.response_time_required=5;c.attitude_response_qualified=true;
    return c;
}

static void test_forced_mm304_segment_honors_committed_aoa(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=entry_state_at_range(&p,&cfg.site,154000.0,37800.0,1959.0);
    EntryControlPlan plan;memset(&plan,0,sizeof(plan));
    plan.valid=true;plan.planned_ut=state.ut;plan.target_bank=-46.0;plan.target_aoa=18.0;
    plan.segment_duration=20.0;

    /* A committed MM304 segment is already vertically shaped by live guidance.
       The predictor must not silently replace its AoA with its own altitude-debt
       ceiling during that forced horizon; otherwise the forecast and flown plan
       immediately diverge again. Start well below the target so the distinction
       is visible in the propagated maximum AoA. */
    EntryPrediction pr=predictor_simulate_entry_control_plan(state,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,-46.0,0.0,8.0,0.0,-1.0,30.0,&plan,20.0,false);
    assert(pr.entered_atmosphere);
    assert(pr.maximum_abs_angle_of_attack>12.0);
    entry_prediction_clear(&pr);
}

static void test_shared_conservative_stall_proxy(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    double fast=cfg.vehicle.minimum_safe_speed*2.0,thin_q=68.7;
    double protective=entry_low_q_protective_aoa_floor(thin_q,&cfg.vehicle);
    double clean=vessel_physics_conservative_stall_fraction(fast,0.0,thin_q,&cfg.vehicle);
    double nominal=vessel_physics_conservative_stall_fraction(fast,protective,thin_q,&cfg.vehicle);
    double slow=vessel_physics_conservative_stall_fraction(cfg.vehicle.minimum_safe_speed*.90,0.0,thin_q,&cfg.vehicle);
    double high_aoa=vessel_physics_conservative_stall_fraction(fast,cfg.vehicle.maximum_angle_of_attack,thin_q,&cfg.vehicle);

    double tracking_q=370.0,tracking_floor=entry_low_q_protective_aoa_floor(tracking_q,&cfg.vehicle);
    double tracking=vessel_physics_conservative_stall_fraction(fast,tracking_floor+1.0,tracking_q,&cfg.vehicle);
    double high_q_onset=vessel_physics_conservative_stall_fraction(fast,cfg.vehicle.maximum_angle_of_attack*.75,2500.0,&cfg.vehicle);
    double high_q_mid=vessel_physics_conservative_stall_fraction(fast,cfg.vehicle.maximum_angle_of_attack*.875,2500.0,&cfg.vehicle);
    assert(fabs(clean)<1e-12);
    assert(fabs(nominal)<1e-12);
    assert(nominal<=0.12+1e-12);

    assert(tracking<=0.12+1e-12);

    assert(entry_s_turn_bank_authority_available(tracking_q,2090.0,tracking,0.1,&cfg.vehicle));
    assert(slow>0.5&&slow<=1.0);
    assert(fabs(high_aoa-1.0)<1e-12);
    assert(fabs(high_q_onset)<1e-12);
    assert(fabs(high_q_mid-.5)<1e-12);
}

static void test_exact_terminal_shadow_reuses_mm305_policy_and_reports_uncertainty(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=terminal_state_at_range(&p,&cfg.site,45000.0);
    Telemetry t;telemetry_init(&t);t.ut=state.ut;t.roll=0;t.angle_of_attack=15;t.true_air_speed=600;
    t.mass=state.mass;t.has_body_roll_rate=true;t.has_body_pitch_rate=true;t.physics_certified_uncertainty=.15;
    t.trajectory_density_scale=1;t.trajectory_drag_scale=1;t.trajectory_lift_scale=1;t.bank_effectiveness=1;

    GuidanceMachine g;guidance_machine_init(&g);g.automation_engaged=true;g.deorbit_burn_completed=true;
    g.atmospheric_interface_crossed=true;g.terminal_region_entered=true;g.terminal_glide_mode=true;g.phase=PHASE_TAEM;
    TaemExecObservation observation={.ut=state.ut,.relative_velocity=600,.checkpoint_restart=false};
    TaemExecInputs inputs;memset(&inputs,0,sizeof(inputs));inputs.mm304_complete=true;inputs.energy_valid=true;inputs.energy_excess=10;
    inputs.terminal_feasibility_valid=true;inputs.nominal_terminal_path_feasible=true;inputs.terminal_contract=shadow_terminal_contract();
    TaemExecProfile profile={.s_turn_enabled=true};
    assert(taem_exec_initialize(&g.taem_exec,&observation,&inputs,&profile));
    assert(taem_exec_owns_vehicle(&g.taem_exec));

    EntryPrediction pr=predictor_simulate_terminal_shadow_ensemble(state,&t,&g,NULL,&p,aero,&env,&cal,&cfg,60,true);
    assert(pr.shadow_guidance_used&&pr.reached_taem);
    assert(pr.final_state.ut>state.ut+1);
    assert(pr.trajectory.count>1);
    assert(pr.uncertainty_scenarios==3);
    assert(isfinite(pr.physics_relative_uncertainty)&&pr.physics_relative_uncertainty>=.08&&pr.physics_relative_uncertainty<=.40);
    assert(isfinite(pr.terminal_survivability_stress_score)&&pr.terminal_survivability_stress_score>=0&&pr.terminal_survivability_stress_score<=1);
    assert(taem_exec_owns_vehicle(&g.taem_exec)); /* cloned shadow may not mutate live ownership */
    entry_prediction_clear(&pr);
}

static void test_terminal_shadow_refuses_pre_latch_advisory_state(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=terminal_state_at_range(&p,&cfg.site,45000.0);Telemetry t;telemetry_init(&t);t.ut=state.ut;t.angle_of_attack=15;t.true_air_speed=600;t.mass=state.mass;
    GuidanceMachine g;guidance_machine_init(&g);g.automation_engaged=true;g.terminal_glide_mode=true;g.phase=PHASE_TAEM;
    assert(!taem_exec_owns_vehicle(&g.taem_exec));
    EntryPrediction pr=predictor_simulate_terminal_shadow(state,&t,&g,NULL,&p,aero,&env,&cal,&cfg,20,false);
    assert(!pr.shadow_guidance_used&&pr.uncertainty_scenarios==0);
    entry_prediction_clear(&pr);
}


static void test_entry_guidance_shadow_starts_from_shared_restart_policy(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);

    VehicleState coast=entry_state_at_range(&p,&cfg.site,1000000.0,75000.0,2100.0);
    Telemetry coast_t=shadow_entry_telemetry(&coast,&p,&cfg,20.2);
    GuidanceMachine coast_g;
    guidance_initialize_reentry_continuation(&coast_g,&coast_t,&p,&cfg,1.0,false,&env,&cal);
    assert(coast_g.phase==PHASE_ENTRY_INTERFACE&&!coast_g.atmospheric_interface_crossed);
    DeorbitPlan coast_plan=shadow_continuation_plan(&coast_t);
    EntryPrediction coast_pr=predictor_simulate_entry_guidance_shadow(coast,&coast_t,&coast_g,&coast_plan,
        &p,aero,&env,&cal,&cfg,6.0,true);
    assert(coast_pr.shadow_guidance_used&&!coast_pr.shadow_aborted);
    assert(!coast_pr.entered_atmosphere&&!coast_pr.reached_taem);
    assert(coast_pr.final_state.ut>coast.ut+1.0);
    assert(coast_pr.trajectory.count>=2);
    assert(coast_pr.trajectory.points[0].phase==PHASE_ENTRY_INTERFACE);
    assert(coast_g.phase==PHASE_ENTRY_INTERFACE); /* shadow clone cannot mutate live seed */
    entry_prediction_clear(&coast_pr);trajectory_clear(&coast_plan.trajectory);

    VehicleState loaded=entry_state_at_range(&p,&cfg.site,600000.0,65000.0,2100.0);
    Telemetry loaded_t=shadow_entry_telemetry(&loaded,&p,&cfg,20.2);
    GuidanceMachine loaded_g;
    guidance_initialize_reentry_continuation(&loaded_g,&loaded_t,&p,&cfg,1.0,false,&env,&cal);
    assert(loaded_g.phase==PHASE_ENTRY_ENERGY&&loaded_g.atmospheric_interface_crossed);
    DeorbitPlan loaded_plan=shadow_continuation_plan(&loaded_t);
    EntryPrediction loaded_pr=predictor_simulate_entry_guidance_shadow(loaded,&loaded_t,&loaded_g,&loaded_plan,
        &p,aero,&env,&cal,&cfg,12.0,true);
    assert(loaded_pr.shadow_guidance_used&&loaded_pr.entered_atmosphere&&!loaded_pr.shadow_aborted);
    assert(loaded_pr.final_state.ut>loaded.ut+1.0);
    assert(loaded_pr.trajectory.count>=2);
    assert(loaded_pr.maximum_abs_angle_of_attack>=entry_thermal_protection_aoa_floor(&cfg.vehicle)-1e-6);
    assert(loaded_g.phase==PHASE_ENTRY_ENERGY); /* no mutation of real continuation state */
    entry_prediction_clear(&loaded_pr);trajectory_clear(&loaded_plan.trajectory);
}

static void test_entry_guidance_shadow_does_not_false_handoff_at_speed_boundary(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    VehicleState late=entry_state_at_range(&p,&cfg.site,8000.0,23900.0,cfg.guidance.taem_force_handoff_speed-1.0);
    Telemetry t=shadow_entry_telemetry(&late,&p,&cfg,10.0);
    GuidanceMachine g;
    guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,-1.0,false,&env,&cal);
    DeorbitPlan plan=shadow_continuation_plan(&t);
    EntryPrediction pr=predictor_simulate_entry_guidance_shadow(late,&t,&g,&plan,
        &p,aero,&env,&cal,&cfg,60.0,true);
    assert(pr.shadow_guidance_used&&pr.entered_atmosphere);
    if(!pr.reached_taem&&!pr.taem_ownership_boundary_missed&&!pr.shadow_aborted)
        assert(pr.final_state.ut>=late.ut+59.0);
    if(pr.reached_taem)assert(pr.taem_speed<=cfg.guidance.taem_force_handoff_speed+1e-6);
    assert(g.phase==PHASE_ENTRY_ENERGY); /* qualification rollout is side-effect free */
    entry_prediction_clear(&pr);trajectory_clear(&plan.trajectory);
}

static void test_entry_guidance_shadow_honors_inertial_entry_capture_direction(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    /* DirectFix-like upper interface state: q is effectively zero, so live control
       uses inertial prograde/RCS capture and ignores the dormant max-AoA Euler
       fields.  The reduced shadow must carry the same low-incidence intent into
       first aerodynamic authority instead of slewing toward ~28 deg. */
    VehicleState state=entry_state_at_range(&p,&cfg.site,1000000.0,75000.0,2100.0);
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,0.0);
    GuidanceMachine g;
    guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1.0,false,&env,&cal);
    DeorbitPlan plan=shadow_continuation_plan(&t);
    EntryPrediction pr=predictor_simulate_entry_guidance_shadow(state,&t,&g,&plan,
        &p,aero,&env,&cal,&cfg,6.0,true);
    assert(pr.shadow_guidance_used&&!pr.entered_atmosphere&&!pr.shadow_aborted);
    assert(pr.maximum_abs_angle_of_attack<5.0);
    assert(pr.final_state.ut>state.ut+1.0);
    entry_prediction_clear(&pr);trajectory_clear(&plan.trajectory);
}

static void test_reentry_shadow_recovery_qualifier_uses_restart_contract(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    EntryPrediction pr;memset(&pr,0,sizeof(pr));trajectory_init(&pr.trajectory);
    pr.shadow_guidance_used=true;pr.entered_atmosphere=true;pr.reached_taem=true;
    pr.closest_distance=fmin(5000.0,cfg.guidance.target_deorbit_capture_radius*.5);
    pr.taem_distance=pr.closest_distance;pr.taem_range_error=0.0;
    pr.taem_speed=fmax(cfg.guidance.taem_force_handoff_speed,cfg.vehicle.minimum_safe_speed*2.0);
    pr.minimum_entry_speed=pr.taem_speed;
    pr.peak_dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure*.8;
    pr.peak_g_load=cfg.vehicle.maximum_g_load*.8;
    assert(reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));

    double legacy_closest=pr.closest_distance;
    pr.closest_distance=cfg.guidance.target_deorbit_capture_radius+11000.0;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.taem_dynamic_interface_captured=true;
    pr.taem_dynamic_interface_target_valid=true;
    /* A captured MM304 tangent state is sufficient for restart qualification;
       terminal path topology is deliberately owned by MM305 after handoff. */
    assert(reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.taem_terminal_candidate_valid=true;
    pr.taem_terminal_candidate_geometry_clean=false;
    assert(reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.taem_dynamic_interface_target_valid=false;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.taem_dynamic_interface_target_valid=true;pr.taem_dynamic_interface_captured=false;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.taem_dynamic_interface_captured=true;
    pr.closest_distance=legacy_closest;
    pr.taem_dynamic_interface_target_valid=false;pr.taem_terminal_candidate_valid=false;

    pr.shadow_aborted=true;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.shadow_aborted=false;pr.taem_ownership_boundary_missed=true;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.taem_ownership_boundary_missed=false;pr.reached_taem=false;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.reached_taem=true;pr.minimum_entry_speed=cfg.vehicle.minimum_safe_speed*.99;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.minimum_entry_speed=pr.taem_speed;pr.peak_dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure*1.2;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    pr.peak_dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure*.8;pr.peak_g_load=cfg.vehicle.maximum_g_load*1.2;
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    entry_prediction_clear(&pr);
}
static __attribute__((unused)) void test_legacy_75km_continuation_is_rejected_by_fixed_alignment_contract(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    /* Exact DirectFix75km inertial state from 07-22-54Z.  This trajectory was once
       accepted by the historical range/tangent handoff, but it cannot satisfy the
       current fixed (-8 km,0), perpendicular-course ownership contract.  It may enter
       degraded TAEM only at the explicit safety boundary; that transition must remain
       marked as a missed MM304 ownership boundary, without a nominal interface capture
       or recovery-qualified terminal solution. */
    VehicleState state={
        .ut=66898.4036422353,
        .position={255406.577946859,-624777.03857852,133.900133853931},
        .velocity={2082.2997622738,916.082985024232,-0.00892249000773412},
        .mass=40250.0
    };
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,-0.171449616551399);
    GuidanceMachine g;
    guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1.0,false,&env,&cal);
    DeorbitPlan plan=shadow_continuation_plan(&t);
    EntryPrediction pr=predictor_simulate_entry_guidance_shadow(state,&t,&g,&plan,
        &p,aero,&env,&cal,&cfg,900.0,true);
    bool saw_interface=false,saw_entry=false,saw_taem=false;
    for(size_t i=0;i<pr.trajectory.count;i++){
        GuidancePhase phase=pr.trajectory.points[i].phase;
        if(phase==PHASE_ENTRY_INTERFACE)saw_interface=true;
        if(phase==PHASE_ENTRY_ENERGY)saw_entry=true;
        if(phase==PHASE_TAEM)saw_taem=true;
    }
    assert(saw_interface);
    assert(saw_entry);
    assert(saw_taem);
    assert(pr.shadow_guidance_used&&pr.entered_atmosphere&&pr.reached_taem);
    assert(pr.taem_ownership_boundary_missed);
    assert(!pr.taem_dynamic_interface_captured);
    assert(!pr.taem_terminal_candidate_valid);
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    assert(g.phase==PHASE_ENTRY_INTERFACE); /* clone-only rollout */
    entry_prediction_clear(&pr);trajectory_clear(&plan.trajectory);
}

static void test_entry_guidance_shadow_low_speed_upstream_is_speed_handoff_not_capture_or_miss(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=entry_state_at_range(&p,&cfg.site,140000.0,30000.0,
        cfg.guidance.taem_force_handoff_speed-20.0);
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,12.0);
    GuidanceMachine g;
    guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1.0,false,&env,&cal);
    DeorbitPlan plan=shadow_continuation_plan(&t);
    EntryPrediction pr=predictor_simulate_entry_guidance_shadow(state,&t,&g,&plan,
        &p,aero,&env,&cal,&cfg,1.0,true);
    /* Crossing the historical V_TAEM threshold while still far upstream is not
       an ownership event.  Without an explicit fixed-interface capture the shadow
       must remain in MM304 and must not advertise recovery-qualified terminal state. */
    assert(pr.shadow_guidance_used&&pr.entered_atmosphere);
    assert(!pr.reached_taem);
    assert(!pr.taem_ownership_boundary_missed);
    assert(!pr.taem_dynamic_interface_target_valid&&!pr.taem_dynamic_interface_captured);
    assert(!reentry_guidance_shadow_recovery_qualified(&pr,&cfg.vehicle,&cfg.guidance));
    assert(pr.taem_range_error>60000.0);
    entry_prediction_clear(&pr);trajectory_clear(&plan.trajectory);
}
static void test_control_plan_shadow_cannot_continue_entry_past_vtaem(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    /* Mirrors the 16:05 split-log ownership failure: MM304 crossed V_TAEM at
       ~23.9 km altitude with only ~8 km to KSC. A predictor that keeps flying
       synthetic Entry after this point can manufacture a later HAC recovery that
       live entry_exec will never execute. */
    VehicleState state=entry_state_at_range(&p,&cfg.site,8000.0,23900.0,
        cfg.guidance.taem_force_handoff_speed-1.0);
    EntryControlPlan plan={
        .valid=true,.planned_ut=state.ut,.target_bank=-44,.target_aoa=14,
        .target_heading=76,.segment_duration=24
    };
    EntryPrediction pr=predictor_simulate_entry_control_plan(state,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,-58,0,9,0,-1,14,&plan,500,true);
    assert(pr.entered_atmosphere);
    assert(!pr.reached_taem);
    assert(pr.final_state.ut-state.ut<1e-9);
    assert(isinf(pr.taem_energy_error));
    entry_prediction_clear(&pr);
}

static __attribute__((unused)) void test_recorded_1605_supervisor_cannot_certify_post_vtaem_recovery(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    /* Exact canonical state from the 16:05 split stream near planner origin
       UT 67555.1925. The old shadow selected a bounded plan as terminal-feasible
       only by continuing synthetic MM304 Entry below the real V_TAEM boundary. */
    VehicleState state={
        .ut=67555.1925486105,
        .position={319182.591078865,543141.609661693,-5869.96635799898},
        .velocity={-1588.37512841323,719.689418301166,-91.155784475592},
        .mass=40251.265625
    };
    GeoPoint recorded_geo=predictor_geo_point(state.position,&p,state.ut);
    GeoPoint site_geo={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    assert(fabs(recorded_geo.latitude-(-0.533845869366966))<1e-6);
    assert(fabs(recorded_geo.longitude-(-79.0033715444139))<1e-6);
    assert(fabs(great_circle_distance(recorded_geo,site_geo,p.radius)-45054.8429445057)<2.0);
    EntryControlPlan nominal={
        .valid=true,.terminal_ready=true,.planned_ut=state.ut,
        .target_bank=-64.2322348268202,.target_aoa=9.093537109375,
        .target_heading=94.3571709363499,.bank_cap=46.4724713295503,
        .target_turn_radius=1026846.38996801,.segment_duration=75,
        .cost=204.309194905375
    };
    EntrySupervisionResult result=predictor_supervise_entry_control(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        58.2460021972656,-0.475741077583279,9.6878833770752,-0.0206667779380602,
        -1,false,0,75,true,false,&nominal,6,2,500);
    /* A selected result is acceptable only if it captures terminal ownership
       before the real MM304 speed boundary. The historical synthetic recovery
       must never reappear as a valid terminal proof. */
    assert(!result.valid);
    assert(result.mode==ENTRY_SUPERVISION_INFEASIBLE);
    assert(result.taem_ownership_boundary_missed);
    PredictorPlannerTrace trace;assert(predictor_last_planner_trace(&trace));
    for(unsigned i=0;i<trace.candidate_count;i++){
        const PredictorPlannerCandidateTrace*c=&trace.candidates[i];
        if(c->assessment.terminal_feasible){
            assert(c->reached_taem);
            assert(isfinite(c->plan.taem_speed));
            assert(c->plan.taem_speed<=cfg.guidance.taem_force_handoff_speed+1e-9);
        }
    }
}

static __attribute__((unused)) void test_recorded_1605_first_entry_policy_reaches_legal_handoff(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    /* First persisted MM304 planner origin from the same 16:05 flight. Under the
       current ownership contract this 75 s executable segment plus subsequent Entry
       propagation reaches the real altitude/upstream/speed handoff gate, so the
       supervisor must not preserve an obsolete future-boundary-miss warning. */
    VehicleState state={
        .ut=67000.7725484976,
        .position={448863.190251352,-495338.746667313,124.943901288233},
        .velocity={1657.39623618457,1590.461380034,-0.165514019007682},
        .mass=40267.36328125
    };
    GeoPoint recorded_geo=predictor_geo_point(state.position,&p,state.ut);
    GeoPoint site_geo={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    assert(fabs(recorded_geo.latitude-0.0107093307160464)<1e-6);
    assert(fabs(recorded_geo.longitude-(-177.118243599648))<1e-6);
    /* Tick 770 updates position but not the sparse guidance group, so the
       reconstructed rangeToSite at that exact tick can be stale. Validate the
       same-tick geometry from persisted inertial position + geodetic state. */
    assert(fabs(great_circle_distance(recorded_geo,site_geo,p.radius)-1072224.6806577505)<2.0);
    EntryControlPlan nominal={
        .valid=true,.terminal_ready=true,.planned_ut=state.ut,
        .target_bank=2.9,.target_aoa=23,.target_heading=90.0046370120484,
        .bank_cap=2.9,.target_turn_radius=INFINITY,.segment_duration=75,
        .cost=229.544290839511
    };
    EntryPrediction replay=predictor_simulate_entry_control_plan(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        .475006103515625,.0803847805387646,24.043514251709,1.38375369181716,
        1,0,&nominal,1200,true);
    assert(replay.reached_taem&&!replay.taem_ownership_boundary_missed);
    double handoff_low=0.0,handoff_high=0.0;
    entry_taem_handoff_altitude_bounds(&cfg.guidance,&handoff_low,&handoff_high);
    assert(replay.taem_altitude>=handoff_low-1e-6&&replay.taem_altitude<=handoff_high+1e-6);
    assert(replay.taem_speed<=cfg.guidance.taem_force_handoff_speed+1e-6);
    assert(replay.taem_distance>cfg.guidance.final_approach_distance);
    entry_prediction_clear(&replay);
    EntrySupervisionResult result=predictor_supervise_entry_control(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        .475006103515625,.0803847805387646,24.043514251709,1.38375369181716,
        1,false,0,75,true,false,&nominal,6,2,1200);
    assert(result.valid);
    assert(result.mode==ENTRY_SUPERVISION_PASS_THROUGH);
    assert(result.plan.terminal_ready);
    assert(!result.taem_ownership_boundary_missed);
    assert(result.plan.taem_speed<=cfg.guidance.taem_force_handoff_speed+1e-6);
}

static __attribute__((unused)) void test_recorded_1605_mm304_planner_scores_returned_segment_semantics(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    VehicleState early={
        .ut=67000.7725484976,
        .position={448863.190251352,-495338.746667313,124.943901288233},
        .velocity={1657.39623618457,1590.461380034,-0.165514019007682},
        .mass=40267.36328125
    };
    EntryControlPlan early_plan=predictor_plan_entry_control(early,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        .475006103515625,.0803847805387646,24.043514251709,1.38375369181716,1,false,0,0,false,false,1200);
    assert(early_plan.valid&&early_plan.terminal_ready);
    assert(early_plan.target_aoa>=entry_thermal_protection_aoa_floor(&cfg.vehicle)-1e-9);
    EntryPrediction early_replay=predictor_simulate_entry_control_plan(early,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        .475006103515625,.0803847805387646,24.043514251709,1.38375369181716,1,0,&early_plan,1200,false);
    assert(early_replay.reached_taem&&!early_replay.taem_ownership_boundary_missed);
    double handoff_low=0.0,handoff_high=0.0;
    entry_taem_handoff_altitude_bounds(&cfg.guidance,&handoff_low,&handoff_high);
    assert(early_replay.taem_altitude>=handoff_low-1e-6&&early_replay.taem_altitude<=handoff_high+1e-6);
    assert(early_replay.taem_speed<=cfg.guidance.taem_force_handoff_speed+1e-6);
    assert(early_replay.taem_distance>cfg.guidance.final_approach_distance);
    entry_prediction_clear(&early_replay);

    VehicleState mid={
        .ut=67225.9325485435,
        .position={653999.923827492,-32017.0969689317,-9.42869346360719},
        .velocity={59.8763258513993,2302.04379812822,-1.45976034890572},
        .mass=40231.265625
    };
    EntryControlPlan mid_plan=predictor_plan_entry_control(mid,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        4.35968017578125,.251041352163457,19.204252243042,.309056540628415,1,false,0,0,false,false,1200);
    /* The current shared ownership gate also keeps the later recorded checkpoint
       honest: its returned plan reaches the 15-18 km shell while still upstream,
       rather than inventing a post-site terminal acquisition. */
    assert(mid_plan.valid&&mid_plan.terminal_ready);
    assert(mid_plan.target_aoa>=entry_thermal_protection_aoa_floor(&cfg.vehicle)-1e-9);
    EntryPrediction mid_replay=predictor_simulate_entry_control_plan(mid,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        4.35968017578125,.251041352163457,19.204252243042,.309056540628415,1,0,&mid_plan,1200,false);
    assert(mid_replay.reached_taem&&!mid_replay.taem_ownership_boundary_missed);
    assert(mid_replay.taem_altitude>=handoff_low-1e-6&&mid_replay.taem_altitude<=handoff_high+1e-6);
    assert(mid_replay.taem_speed<=cfg.guidance.taem_force_handoff_speed+1e-6);
    assert(mid_replay.taem_distance>cfg.guidance.final_approach_distance);
    entry_prediction_clear(&mid_replay);
}

static __attribute__((unused)) void test_recorded_1852_geometry_deadline_pulls_nonfinal_reversal_forward(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;
    p.surface_density=1.1399229405107;
    p.epoch_ut=66898.4036422353;
    p.prime_meridian_at_epoch=v3(-0.610008941035229,0.792394530431072,0);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state={
        .ut=67541.2883298576,
        .position={330486.170722339,539083.286678898,-3040.2462706051},
        .velocity={-1717.1431877924,897.710806523861,-24.2141753341889},
        .mass=40251.26171875
    };
    EntryControlPlan plan=predictor_plan_entry_control(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        .90753173828125,.0250853144887711,10.6348552703857,-.733531765204076,
        1,true,cfg.guidance.s_turn_minimum_leg_duration,0,true,false,500);
    assert(plan.valid&&plan.has_planned_reversal);
    assert(!plan.planned_reversal_is_final);
    /* The failed flight carried the same reversal to only 4.6 km from KSC.
       The actuator-aware geometry deadline must pull it materially upstream. */
    assert(plan.planned_reversal_range>10000.0);
    assert(plan.planned_reversal_ut-state.ut<30.0);
}

static VehicleState recorded_0450_preentry_authority_state(void){
    VehicleState state={
        .ut=67407.4739548304,
        .position={537933.970441663,356343.786576681,-459.616247076497},
        .velocity={-1284.86778177951,1811.665899077,-9.75815585560505},
        .mass=40231.26171875
    };
    return state;
}

static void test_preentry_authority_predictor_dwell_matches_live_capture_contract(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;
    p.surface_density=1.1399229405107;
    p.epoch_ut=66898.5636422354;
    p.prime_meridian_at_epoch=v3(-0.610045906641322,0.792366071831806,0);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);

    /* 04-50 live discriminator: real PREENTRY bank work was already authoritative
       well below the 1.5 m/s2 executive transition. Keep the exact live envelope
       assertion alongside predictor replay so unit conventions cannot drift. */
    assert(entry_s_turn_bank_authority_available(950.749572753906,2033.02990722656,
        0.0,0.0773588716983795,&cfg.vehicle));

    VehicleState state=recorded_0450_preentry_authority_state();
    Vector3 radial=vnorm(state.position,v3(1,0,0));
    /* Lift the fixture slightly into the recorded q~600-700 Pa neighborhood so the
       reduced predictor remains comfortably below the 1.5 m/s2 sticky load gate while
       still inside the shared aerodynamic-authority envelope. */
    state.position=vadd(state.position,vscale(radial,2500.0));
    EntryControlPlan plan={
        .valid=true,.planned_ut=state.ut,.target_bank=23.5796630635964,
        .target_aoa=entry_thermal_protection_aoa_floor(&cfg.vehicle),
        .target_heading=90.3555394187047,.segment_duration=75.0,
        .has_planned_reversal=true,.planned_reversal_ut=state.ut-.5,
        .planned_reversal_range=217236.666433264,.planned_reversal_sign=-1.0
    };
    double minleg=cfg.guidance.s_turn_minimum_leg_duration;

    EntryPrediction before=predictor_simulate_entry_control_plan(state,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,21.3026428222656,0.0,
        entry_thermal_protection_aoa_floor(&cfg.vehicle),0.0,1.0,0.0,&plan,minleg-1.0,false);
    assert(!before.has_first_s_turn_reversal);
    entry_prediction_clear(&before);

    EntryPrediction after=predictor_simulate_entry_control_plan(state,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,21.3026428222656,0.0,
        entry_thermal_protection_aoa_floor(&cfg.vehicle),0.0,1.0,0.0,&plan,minleg+8.0,false);
    if(after.has_first_s_turn_reversal){
        assert(after.first_s_turn_reversal_ut>=state.ut+minleg-1.0);
        assert(after.first_s_turn_reversal_ut<=state.ut+minleg+3.0);
    }
    entry_prediction_clear(&after);

    /* Low-q PREENTRY remains non-authoritative even with a numerically captured bank. */
    VehicleState low=state;
    radial=vnorm(low.position,v3(1,0,0));
    low.position=vadd(low.position,vscale(radial,12000.0));
    plan.planned_ut=low.ut;plan.planned_reversal_ut=low.ut-.5;
    EntryPrediction low_replay=predictor_simulate_entry_control_plan(low,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,21.3026428222656,0.0,
        entry_thermal_protection_aoa_floor(&cfg.vehicle),0.0,1.0,0.0,&plan,minleg+8.0,false);
    assert(!low_replay.has_first_s_turn_reversal);
    entry_prediction_clear(&low_replay);

    /* PREENTRY aero authority is transient, unlike the sticky executive load phase.
       Start inside the useful envelope but climb quickly enough to leave it before the
       full dwell. The replay must reset rather than fire a stale due reversal. */
    VehicleState climbing=state;
    radial=vnorm(climbing.position,v3(1,0,0));
    Vector3 rotation=vcross(planet_rotation_vector(&p),climbing.position);
    Vector3 air=vsub(climbing.velocity,rotation);
    Vector3 horizontal=vproject_plane(air,radial);
    climbing.velocity=vadd(rotation,vadd(horizontal,vscale(radial,260.0)));
    plan.planned_ut=climbing.ut;plan.planned_reversal_ut=climbing.ut-.5;
    EntryPrediction climbing_replay=predictor_simulate_entry_control_plan(climbing,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,21.3026428222656,0.0,
        entry_thermal_protection_aoa_floor(&cfg.vehicle),0.0,1.0,0.0,&plan,minleg+8.0,false);
    assert(!climbing_replay.has_first_s_turn_reversal);
    entry_prediction_clear(&climbing_replay);
}

static __attribute__((unused)) void test_recorded_1852_post_site_vtaem_misses_live_capture_tube(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;
    p.surface_density=1.1399229405107;
    p.epoch_ut=66898.4036422353;
    p.prime_meridian_at_epoch=v3(-0.610008941035229,0.792394530431072,0);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    /* Exact vehicle keyframe at the failed 18:52:52Z V_TAEM handoff. The
       shuttle is already about 25 km outbound of the runway station. A loose
       predictor-only HAC intercept tube used to call this terminal-feasible,
       even though live terminal_capture_ready() rejects the same geometry. */
    VehicleState state={
        .ut=67592.8883298681,
        .position={245762.977046803,574302.946352253,-6715.88900844284},
        .velocity={-1391.00540220174,493.688595541218,-113.679261878071},
        .mass=40251.26171875
    };
    EntryControlPlan plan={
        .valid=true,.planned_ut=state.ut,.target_bank=-11.6423718852985,
        .target_aoa=20.817540001148,.target_heading=94.3236754387984,
        .segment_duration=20.0,.taem_speed=cfg.guidance.taem_force_handoff_speed
    };
    EntryPrediction pr=predictor_simulate_entry_control_plan(
        state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        -12.8083801269531,-1.45061537736031,19.0304412841797,.974774016326755,
        -1,0,&plan,20,false);
    assert(pr.entered_atmosphere);
    assert(!pr.reached_taem);
    assert(pr.taem_ownership_boundary_missed);
    entry_prediction_clear(&pr);
}

static __attribute__((unused)) void test_recorded_1852_210km_state_is_already_outside_recoverable_handoff_set(void){
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;p.surface_density=1.1399229405107;
    p.epoch_ut=66898.4036422353;p.prime_meridian_at_epoch=v3(-0.610008941035229,0.792394530431072,0);
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    /* Exact 18:52 vehicle/planner checkpoint while still ~211 km upstream. This
       state is already too late for any legal 15-18 km/upstream MM304->MM305
       handoff under the current control envelope. Earlier rolling MPC must avoid
       entering it; the predictor must classify it as a missed ownership boundary
       rather than fabricate a downstream recovery. */
    VehicleState state={
        .ut=67451.6483298393,
        .position={477550.753679665,429380.081153486,-761.701608414425},
        .velocity={-1490.50585010252,1535.36942445234,-17.8252195473654},
        .mass=40231.26171875
    };
    double bank=24.3129577636719,bank_rate=-2.68103578974802;
    double aoa=17.9312477111816,aoa_rate=-0.00358558097992476;
    EntryControlPlan plan=predictor_plan_entry_control(state,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,bank,bank_rate,aoa,aoa_rate,
        1.0,false,0.0,0.0,true,false,1200.0);
    if(plan.valid){
        EntryPrediction pr=predictor_simulate_entry_control_plan(state,&p,aero,&env,&cal,
            &cfg.vehicle,&cfg.site,&cfg.guidance,bank,bank_rate,aoa,aoa_rate,
            1.0,0.0,&plan,1200.0,false);
        assert(!pr.reached_taem&&pr.taem_ownership_boundary_missed);
        entry_prediction_clear(&pr);
    }
}

static void test_entry_guidance_shadow_records_mm304_50km_checkpoint(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);

    VehicleState state=entry_state_at_range(&p,&cfg.site,520000.0,52000.0,2100.0);
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,0.0);
    GuidanceMachine g;
    guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1.0,false,&env,&cal);
    DeorbitPlan plan=shadow_continuation_plan(&t);
    EntryPrediction pr=predictor_simulate_entry_guidance_shadow(state,&t,&g,&plan,
        &p,aero,&env,&cal,&cfg,90.0,false);

    assert(pr.shadow_guidance_used&&pr.entered_atmosphere&&!pr.shadow_aborted);
    assert(pr.mm304_gate_recorded);
    assert(isfinite(pr.mm304_gate_range)&&pr.mm304_gate_range>=0.0);
    assert(isfinite(pr.mm304_gate_along_track)&&isfinite(pr.mm304_gate_cross_track));
    assert(isfinite(pr.mm304_gate_course)&&isfinite(pr.mm304_gate_speed)&&pr.mm304_gate_speed>0.0);
    assert(isfinite(pr.mm304_gate_flight_path_angle));
    assert(pr.mm304_gate_altitude<=50000.0);

    entry_prediction_clear(&pr);trajectory_clear(&plan.trajectory);
}

int main(void){
    test_planet_replay_metadata_is_complete();

    test_spherical_planet_geometry_and_gravity();
    test_assessment_flags_safety_and_terminal_feasibility();
    test_supervisor_passes_nominal_and_is_deterministic();
    test_supervisor_correction_is_bounded_and_preserves_lateral_semantics();
    test_safe_nominal_keeps_ownership_when_terminal_proof_is_unavailable();
    test_unsafe_nominal_cannot_escape_zero_correction_envelope();
    test_preburn_plan_state_guard_uses_certified_robustness_envelope();
    test_forced_mm304_segment_honors_committed_aoa();
    test_control_plan_shadow_cannot_continue_entry_past_vtaem();
    test_preentry_authority_predictor_dwell_matches_live_capture_contract();
    test_shared_conservative_stall_proxy();
    test_exact_terminal_shadow_reuses_mm305_policy_and_reports_uncertainty();
    test_terminal_shadow_refuses_pre_latch_advisory_state();
    test_entry_guidance_shadow_starts_from_shared_restart_policy();
    test_entry_guidance_shadow_records_mm304_50km_checkpoint();
    test_entry_guidance_shadow_does_not_false_handoff_at_speed_boundary();

    test_entry_guidance_shadow_honors_inertial_entry_capture_direction();
    test_reentry_shadow_recovery_qualifier_uses_restart_contract();
    test_entry_guidance_shadow_low_speed_upstream_is_speed_handoff_not_capture_or_miss();
    puts("Entry predictor supervision tests passed.");
    return 0;
}
