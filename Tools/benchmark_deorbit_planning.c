/* Offline fixed-state timing/parity fixture. Never connects to KSP.
   --polar-plan replays a 2026-10-12 100 km polar-orbit initial state recorded
   before live deorbit planning. Atmospheric forces remain the offline model. */
#include "landing.h"
#include "sim_telemetry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int main(int argc,char **argv){
    LandingConfiguration cfg=landing_configuration_default();
    FILE *f=fopen("Configuration/sts-live.json","rb");
    if(!f)return 2;
    char text[65536];size_t size=fread(text,1,sizeof(text)-1,f);fclose(f);text[size]=0;
    JsonToken tokens[8192];JsonDoc doc;
    if(json_parse(text,tokens,8192,&doc)<0||!landing_configuration_from_json(&cfg,&doc,0))return 2;
    char path[512],error[512];PlanetModel p;
    if(!shuttle_sim_load_planet(shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,path,sizeof(path)),&p,error,sizeof(error))){
        fprintf(stderr,"%s\n",error);return 2;
    }
    bool polar=argc>1&&!strcmp(argv[1],"--polar-plan");
    bool plan=polar||(argc>1&&!strcmp(argv[1],"--plan"));
    double r=p.radius+100000.0,peri=p.radius+50000.0;
    double speed=sqrt(p.gravitational_parameter*(2.0/r-2.0/(r+peri)));
    if(plan)speed=sqrt(p.gravitational_parameter/r);
    VehicleState state={.position={r,0,0},.velocity={0,0,speed},.mass=22000.0,.ut=0.0};
    AerodynamicModel aero={cfg.vehicle.estimated_lift_to_drag,cfg.vehicle.estimated_ballistic_coefficient,0.05};
    AerodynamicEnvelope env;for(int i=0;i<4;i++)env.regimes[i]=aero;
    double thrust=60000.0;
    if(polar){
        /* FlightLogs/2026-10-12T10-25-20Z-STS-vehicle.jsonl, first keyframe.
           Real state/planet orientation and measured live aero envelope; no KSP
           access, mutating savepoint, or artificial circular-orbit assumption. */
        state=(VehicleState){.position={207985.161391343,-480859.502545376,464327.772627903},
            .velocity={-591.1344877022,1367.13071282428,1681.09862088477},
            .mass=26066.951171875,.ut=146933071.131715};
        p.epoch_ut=146933070.991715;
        p.prime_meridian_at_epoch=(Vector3){-.471671944124894,-.881774107765386,0};
        p.north_axis=(Vector3){0,0,1};
        env.regimes[0]=(AerodynamicModel){.217065034272743,198.411331724812,.934327984691704};
        env.regimes[1]=(AerodynamicModel){.721139538744275,596.650084087165,.899181345853369};
        env.regimes[2]=(AerodynamicModel){.759301463850623,643.198636310678,.95968166405498};
        env.regimes[3]=(AerodynamicModel){.465251228615833,359.753286896131,.914874410782777};
        aero=env.regimes[2];
        thrust=59195.8151910419;
    }
    TrajectoryCalibrationModel cal={.density_scale=1,.drag_scale=1,.lift_scale=1,
        .bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.05};
    clock_t start=clock();
    if(plan){
        DeorbitPlan result={0};
        bool ok=deorbit_plan_create(&result,state,2.0*M_PI*sqrt(r*r*r/p.gravitational_parameter),
            &p,aero,&env,&cal,&cfg.site,&cfg.vehicle,&cfg.guidance,thrust);
        printf("plan ok=%d execution=%d scenarios=%u passed=%u dv=%.17g burn=%.17g normal=%.17g peri=%.17g entry=%.17g fpa=%.17g\n",
            ok,result.execution_qualified,result.robustness_scenarios,result.robustness_passed,
            result.delta_v,result.burn_ut,result.burn_normal_angle,
            result.predicted_post_burn_periapsis_altitude,
            result.predicted_entry_range,result.predicted_entry_flight_path_angle);
        deorbit_plan_clear(&result);
    }else{
        EntryPrediction result=predictor_simulate_entry(state,&p,aero,NULL,NULL,
            &cfg.vehicle,&cfg.site,&cfg.guidance,0,0,0,0,4500,false);
        printf("passive entered=%d taem=%d elapsed=%.17g position=%.17g,%.17g,%.17g velocity=%.17g,%.17g,%.17g closest=%.17g\n",
            result.entered_atmosphere,result.reached_taem,result.final_state.ut-state.ut,
            result.final_state.position.x,result.final_state.position.y,result.final_state.position.z,
            result.final_state.velocity.x,result.final_state.velocity.y,result.final_state.velocity.z,result.closest_distance);
        entry_prediction_clear(&result);
    }
    fprintf(stderr,"cpu_seconds=%.6f\n",(double)(clock()-start)/CLOCKS_PER_SEC);
    return 0;
}
