#include "../../CLanding/predictor.c"
#define main entry_predictor_supervision_full_main
#include "../../Validation/EntryPredictorSupervisionTests.c"
#undef main

static void direct_models(AerodynamicModel*aero,AerodynamicEnvelope*env,TrajectoryCalibrationModel*cal){
    *aero=(AerodynamicModel){.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=.6};
    for(int i=0;i<4;i++)env->regimes[i]=*aero;
    *cal=(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,
        .speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.2};
}
static PlanetModel replay_planet(void){
    PlanetModel p=kerbin();p.rotational_speed=0.000291570900559802;p.surface_density=1.1399229405107;
    p.epoch_ut=67555.1925486105;p.north_axis=v3(0,0,1);
    p.prime_meridian_at_epoch=v3(-0.7496760227567124,0.6618050021748682,0);return p;
}
static void print_gate(const char*name,const EntryPrediction*p){
    printf("%s shadow=%d abort=%d entered=%d gate=%d along=%+.3fkm cross=%+.3fkm V=%.1f h=%.1f FPA=%.2f reachedTAEM=%d miss=%d\n",
        name,p->shadow_guidance_used,p->shadow_aborted,p->entered_atmosphere,p->mm304_gate_recorded,
        p->mm304_gate_along_track/1000.0,p->mm304_gate_cross_track/1000.0,p->mm304_gate_speed,
        p->mm304_gate_altitude,p->mm304_gate_flight_path_angle,p->reached_taem,p->taem_ownership_boundary_missed);
}
int main(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    PlanetModel p=replay_planet();AerodynamicModel aero;AerodynamicEnvelope env;TrajectoryCalibrationModel cal;
    direct_models(&aero,&env,&cal);
    VehicleState pre={.ut=66520.4125093287,
        .position={-519319.101757897,-449419.285815064,-24.6917249450811},
        .velocity={1483.92378825283,-1715.21509885343,-0.230730010293286},.mass=40252.91796875};
    const double thrust=377000.0,dv=33.0;
    double duration=0.0,delivered=0.0;
    VehicleState after=retro_burn_scenario(pre,dv,thrust,cfg.guidance.deorbit_maximum_throttle,
        cfg.guidance.deorbit_throttle_ramp_duration,&p,0.0,0.0,&duration,&delivered,NULL);
    Candidate c={0};c.burn_ut=pre.ut+duration*.5;c.dv=dv;c.duration=duration;c.after=after;
    c.peri=predictor_postburn_periapsis(after,&p);
    printf("burn duration=%.3f delivered=%.3f peri=%.1fm afterUT=%.3f\n",duration,delivered,c.peri,after.ut);
    EntryPrediction reduced=predictor_simulate_entry(after,&p,aero,&env,&cal,&cfg.vehicle,&cfg.site,&cfg.guidance,
        cfg.vehicle.maximum_bank_angle,0,0,0,4500,false);
    EntryPrediction witness=deorbit_executable_policy_witness(&c,&p,aero,&env,&cal,&cfg.site,&cfg.vehicle,&cfg.guidance);
    print_gate("reduced",&reduced);print_gate("witness",&witness);
    if(reduced.mm304_gate_recorded&&witness.mm304_gate_recorded)
        printf("delta along=%+.3fkm cross=%+.3fkm dV=%+.1fm/s\n",
            (witness.mm304_gate_along_track-reduced.mm304_gate_along_track)/1000.0,
            (witness.mm304_gate_cross_track-reduced.mm304_gate_cross_track)/1000.0,
            witness.mm304_gate_speed-reduced.mm304_gate_speed);
    entry_prediction_clear(&reduced);entry_prediction_clear(&witness);return 0;
}
