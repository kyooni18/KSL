#define main legacy_predictor_suite_main
#include "../../Validation/EntryPredictorSupervisionTests.c"
#undef main

static double state_airspeed(VehicleState s,const PlanetModel*p){
    Vector3 air=vsub(s.velocity,vcross(planet_rotation_vector(p),s.position));
    return vmag(air);
}

static void run_horizon(double horizon){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    PlanetModel p=kerbin();
    p.rotational_speed=0.000291570900559802;
    p.surface_density=1.1399229405107;
    p.epoch_ut=66898.5636422354;
    p.prime_meridian_at_epoch=v3(-0.610045906641322,0.792366071831806,0);
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state=recorded_0450_preentry_authority_state();
    Telemetry t=shadow_entry_telemetry(&state,&p,&cfg,20.2);
    t.dynamic_pressure=950.749572753906;
    t.mach=6.70;
    t.g_force=.0773588716983795;
    t.speed_of_sound=t.true_air_speed/t.mach;
    t.atmospheric_density=2.0*t.dynamic_pressure/(t.true_air_speed*t.true_air_speed);
    t.aerodynamic_confidence=.7;
    t.physics_confidence=.7;
    GuidanceMachine g;
    guidance_initialize_reentry_continuation(&g,&t,&p,&cfg,1,false,&env,&cal);
    g.entry_planning_deferred=true;
    g.diagnostic_shadow=true;
    DeorbitPlan plan=shadow_continuation_plan(&t);

    EntryPrediction reduced=predictor_simulate_entry(state,&p,aero,&env,&cal,
        &cfg.vehicle,&cfg.site,&cfg.guidance,cfg.vehicle.maximum_bank_angle,
        t.roll,1.0,0.0,horizon,false);
    EntryPrediction shadow=predictor_simulate_entry_guidance_shadow(state,&t,&g,&plan,&p,aero,
        &env,&cal,&cfg,horizon,false);
    GeoPoint gr=predictor_geo_point(reduced.final_state.position,&p,reduced.final_state.ut);
    GeoPoint gs=predictor_geo_point(shadow.final_state.position,&p,shadow.final_state.ut);
    double horizontal=great_circle_distance(gr,gs,p.radius);
    printf("horizon=%.0f initial_h=%.1f initial_V=%.1f reduced_h=%.1f shadow_h=%.1f dH=%.1f horizontal=%.1f reduced_V=%.1f shadow_V=%.1f dV=%.1f reduced_t=%.2f shadow_t=%.2f\n",
        horizon,t.mean_altitude,t.true_air_speed,gr.altitude,gs.altitude,gr.altitude-gs.altitude,
        horizontal,state_airspeed(reduced.final_state,&p),state_airspeed(shadow.final_state,&p),
        state_airspeed(reduced.final_state,&p)-state_airspeed(shadow.final_state,&p),
        reduced.final_state.ut-state.ut,shadow.final_state.ut-state.ut);
    entry_prediction_clear(&reduced);entry_prediction_clear(&shadow);deorbit_plan_clear(&plan);
}

static void run_v27_margin(void){
    PlanetModel p=recorded_1605_planet();LandingConfiguration cfg=landing_configuration_default();
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;predictor_models(&cfg,&aero,&env,&cal);
    VehicleState state={.ut=67423.4636423423,.position={518300.646260972,377452.361008541,-4040.3237684788},
        .velocity={-1334.58063353126,1625.471375285,-81.0767413385289},.mass=40251.0};
    EntryControlPlan nominal={.valid=true,.planned_ut=67421.3836423419,.target_bank=0,.target_aoa=24.718481837413,
        .target_heading=96.0851259863373,.bank_cap=70,.target_turn_radius=INFINITY,.segment_duration=80.0799999996088,
        .cost=54.8387672026546};
    EntrySupervisionResult generic=predictor_supervise_entry_control(state,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        69.1799621582031,-0.182966979481971,24.4323921203613,-0.462323450724133,
        1,false,0,80.0799999996088,true,false,&nominal,6,2,500);
    printf("v27 generic valid=%d safe=%d terminal=%d ready=%d taemV=%.1f rangeErr=%.1f closest=%.1f qRatio=%.3f gRatio=%.3f minVRatio=%.3f bankMargin=%.3f aoaMargin=%.3f\n",
        generic.valid,generic.nominal_assessment.safe,generic.nominal_assessment.terminal_feasible,generic.plan.terminal_ready,
        generic.plan.taem_speed,generic.plan.taem_range_error,generic.plan.closest_distance,
        generic.nominal_assessment.dynamic_pressure_ratio,generic.nominal_assessment.g_load_ratio,
        generic.nominal_assessment.minimum_speed_ratio,generic.nominal_assessment.bank_margin,generic.nominal_assessment.aoa_margin);
}

int main(void){run_v27_margin();run_horizon(30);run_horizon(60);run_horizon(120);return 0;}
