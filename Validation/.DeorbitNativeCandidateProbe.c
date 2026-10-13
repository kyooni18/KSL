/* Offline fixed-state timing/parity fixture. Never connects to KSP.
   --polar-plan replays a 2026-10-12 100 km polar-orbit initial state recorded
   before live deorbit planning. Atmospheric forces remain the offline model. */
#include "../CLanding/prediction/predictor.c"
#include "sim_telemetry.h"
#include "terminal_model.h"
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
    bool polar=argc>1&&(!strcmp(argv[1],"--polar-plan")||
        !strcmp(argv[1],"--polar-terminal"));
    bool fourth=argc>1&&(!strcmp(argv[1],"--fourth-polar-plan")||
        !strcmp(argv[1],"--fourth-polar-terminal"));
    bool terminal=argc>1&&(!strcmp(argv[1],"--polar-terminal")||
        !strcmp(argv[1],"--fourth-polar-terminal"));
    bool plan=polar||fourth||(argc>1&&!strcmp(argv[1],"--plan"));
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
    if(fourth){
        /* Freeze the 2026-10-12 fourth live STS orbital state at UT
           146949660.947894, its KSP rotation frame, and first planner
           calibrated aero envelope. Reproduces the KSC-overflight plan without
           opening KSP or reading mutable game state. */
        state=(VehicleState){.position={265913.976923641,266230.434410646,590362.583042728},
            .velocity={-1338.3903380996,-1340.09579557642,1207.03270772642},
            .mass=25703.169921875,.ut=146949660.947894};
        p.epoch_ut=146949660.647894;
        p.prime_meridian_at_epoch=(Vector3){-.933582298018746,.358363074026938,0};
        p.north_axis=(Vector3){0,0,1};
        env.regimes[0]=(AerodynamicModel){.224200587877171,191.758167836936,.882985601234646};
        env.regimes[1]=(AerodynamicModel){.773070325204928,692.167061067764,.936993059592152};
        env.regimes[2]=(AerodynamicModel){.734959904003455,638.320313325158,.96515002259869};
        env.regimes[3]=(AerodynamicModel){.48723219035857,377.98466463495,.938729947916751};
        aero=env.regimes[2];
        thrust=59195.8164168286;
    }
    TerminalModel*terminal_model=NULL;
    if(terminal){
        char atmosphere[512],aero_path[512],force_book[512],attitude[512],reason[256];
        TerminalModelSourceFiles sources={
            shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,atmosphere,sizeof(atmosphere)),
            shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,aero_path,sizeof(aero_path)),
            shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,force_book,sizeof(force_book)),
            shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,attitude,sizeof(attitude))};
        terminal_model=calloc(1,sizeof(*terminal_model));
        if(!terminal_model||!terminal_model_capture(terminal_model,&sources,&cfg,11,reason,sizeof(reason))||
           !terminal_model_align_planet_frame(terminal_model,&p,reason,sizeof(reason))){
            fprintf(stderr,"native terminal capture/planet frame failed: %s\n",reason);return 2;
        }
    }
    TrajectoryCalibrationModel cal={.density_scale=1,.drag_scale=1,.lift_scale=1,
        .bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.confidence=.05};
    clock_t start=clock();
    if(plan){
        if(fourth&&terminal){
            /* Probe nearby degraded polar opportunities using real downstream
               guidance, not just their energy forecast. */
            double nominal=146957021.75;
            for(int off=0;off<=105;off+=15){
                Candidate c={0};trajectory_init(&c.pred.trajectory);
                if(!deorbit_evaluate_start(&c,state,nominal+off,0.0,23.5546875,&p,
                        aero,&env,&cal,&cfg.site,&cfg.vehicle,&cfg.guidance,thrust,NULL)){
                    printf("probe off=%d no-candidate\n",off);continue;
                }
                EntryPrediction entry=deorbit_executable_policy_witness(&c,&p,aero,&env,
                    &cal,&cfg.site,&cfg.vehicle,&cfg.guidance,NULL,NULL,false,false);
                EntryPrediction whole=deorbit_executable_policy_witness(&c,&p,aero,&env,
                    &cal,&cfg.site,&cfg.vehicle,&cfg.guidance,NULL,terminal_model,true,false);
                GeoPoint end=predictor_geo_point(whole.final_state.position,&p,whole.final_state.ut);
                GeoPoint runway={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
                double end_range=great_circle_distance(end,runway,p.radius);
                printf("NATIVE_CANDIDATE off=%+d burnUT=%.1f inlet=%.1fkm energy=%+.0fkJ/kg h=%.0fm V=%.0fmps course=%.0f final=%d route=%d abort=%d closest=%.1fkm endRange=%.1fkm endH=%.0fm endSpeed=%.0fmps endUT=%.1f\n",
                    off,c.burn_ut,entry.taem_distance/1000,
                    entry.taem_interface_energy_margin/1000,entry.taem_altitude,
                    entry.taem_speed,entry.taem_course,
                    whole.shadow_final_approach_captured,whole.shadow_terminal_path_committed,
                    whole.shadow_aborted,whole.closest_distance/1000,end_range/1000,
                    end.altitude,vmag(whole.final_state.velocity),whole.final_state.ut);
                fflush(stdout);
                entry_prediction_clear(&entry);entry_prediction_clear(&whole);candidate_clear(&c);
            }
            free(terminal_model);return 0;
        }
        DeorbitPlan result={0};
        bool ok=terminal?
            deorbit_plan_create_with_terminal_model(&result,state,
                2.0*M_PI*sqrt(r*r*r/p.gravitational_parameter),
                &p,aero,&env,&cal,&cfg.site,&cfg.vehicle,&cfg.guidance,
                thrust,NULL,terminal_model):
            deorbit_plan_create(&result,state,2.0*M_PI*sqrt(r*r*r/p.gravitational_parameter),
                &p,aero,&env,&cal,&cfg.site,&cfg.vehicle,&cfg.guidance,thrust);
        printf("plan ok=%d execution=%d scenarios=%u passed=%u dv=%.17g burn=%.17g normal=%.17g peri=%.17g entry=%.17g fpa=%.17g\n",
            ok,result.execution_qualified,result.robustness_scenarios,result.robustness_passed,
            result.delta_v,result.burn_ut,result.burn_normal_angle,
            result.predicted_post_burn_periapsis_altitude,
            result.predicted_entry_range,result.predicted_entry_flight_path_angle);
        printf("recovery=%d nominal=%d closest=%.1fkm TAEM=%.1fkm rangeError=%+.1fkm note=%s\n",
            result.target_capture_achieved?1:0,result.nominal_capture_achieved?1:0,
            result.predicted_closest_distance/1000.0,result.predicted_taem_distance/1000.0,
            result.predicted_taem_range_error/1000.0,result.note);
        printf("nativeTerminal snapshot=%llu replay=%d inlet=%d route=%d final=%d abort=%d energy=%+.1fkJ/kg strict=%d\n",
            (unsigned long long)result.terminal_model_snapshot_id,
            result.terminal_replay_evaluated?1:0,
            result.terminal_inlet_qualified?1:0,
            result.terminal_native_path_committed?1:0,
            result.terminal_final_approach_captured?1:0,
            result.terminal_guidance_aborted?1:0,
            result.terminal_inlet_energy_margin/1000.0,
            result.target_capture_achieved?1:0);
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
    free(terminal_model);
    fprintf(stderr,"cpu_seconds=%.6f\n",(double)(clock()-start)/CLOCKS_PER_SEC);
    return 0;
}
