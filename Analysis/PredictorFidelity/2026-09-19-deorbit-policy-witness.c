#define main entry_predictor_supervision_full_main
#include "../../Validation/EntryPredictorSupervisionTests.c"
#undef main

static double kepler_period(VehicleState state,const PlanetModel*p){
    double r=vmag(state.position),v2=vdot(state.velocity,state.velocity);
    double energy=.5*v2-p->gravitational_parameter/r;
    double a=-p->gravitational_parameter/(2.0*energy);
    return 2.0*LANDER_PI*sqrt(a*a*a/p->gravitational_parameter);
}

int main(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    PlanetModel p=recorded_1605_planet();
    VehicleState state={
        .ut=66520.4125093287,
        .position={-519319.101757897,-449419.285815064,-24.6917249450811},
        .velocity={1483.92378825283,-1715.21509885343,-0.230730010293286},
        .mass=40252.91796875
    };
    AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    predictor_models(&cfg,&aero,&env,&cal);
    double period=kepler_period(state,&p);
    const double measured_available_thrust=377000.0;
    DeorbitPlan plan;
    bool ok=deorbit_plan_create(&plan,state,period,&p,aero,&env,&cal,
        &cfg.site,&cfg.vehicle,&cfg.guidance,measured_available_thrust);
    printf("created=%d period=%.1f burnUT=%.1f dv=%.2f duration=%.2f entryRange=%.1fkm FPA=%.2f taemRange=%.1fkm closest=%.1fkm nominal=%d robust=%d execution=%d degraded=%d confidence=%.3f\n",
        ok?1:0,period,ok?plan.burn_ut:NAN,ok?plan.delta_v:NAN,ok?plan.estimated_burn_duration:NAN,
        ok?plan.predicted_entry_range/1000.0:NAN,ok?plan.predicted_entry_flight_path_angle:NAN,
        ok?plan.predicted_taem_distance/1000.0:NAN,ok?plan.predicted_closest_distance/1000.0:NAN,
        ok&&plan.nominal_capture_achieved,ok&&plan.robustness_qualified,ok&&plan.execution_qualified,
        ok&&plan.execution_degraded,ok?plan.confidence:NAN);
    if(ok){puts(plan.note);trajectory_clear(&plan.trajectory);}
    return ok?0:2;
}
