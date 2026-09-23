#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "shuttlesim/sim.h"
#include "shuttlesim/scenario.h"
#include "shuttlesim/protocol.h"
#include "shuttlesim/math3.h"
#include "shuttlesim/replay.h"

static double monotonic_s(void){struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);return ts.tv_sec+ts.tv_nsec*1e-9;}
static void sleep_s(double sec){if(sec<=0)return;struct timespec ts;ts.tv_sec=(time_t)sec;ts.tv_nsec=(long)((sec-ts.tv_sec)*1e9);nanosleep(&ts,NULL);}
static void usage(const char *p){
    fprintf(stderr,
      "Usage: %s [options]\n"
      "  --scenario FILE          Scenario INI (default scenarios/orbit86km.ini)\n"
      "  --atmosphere FILE        Override Kerbin atmosphere CSV\n"
      "  --aero FILE              Override fallback STS-N Mach/AoA aero CSV\n"
      "  --aero-book FILE         Direct KSP force/q data book (log(q), Mach, AoA)\n"
      "  --attitude FILE          Override vehicle attitude-response INI\n"
      "  --dt SEC                 Physics step, default 0.02\n"
      "  --rate max|N             Wall-clock pacing; N=sim seconds / real second\n"
      "  --telemetry-hz N         JSONL publication rate in simulated time, default 10\n"
      "  --max-sim-time SEC       Stop limit, default 2400\n"
      "  --fixed-aoa DEG          Test driver; hold AoA command\n"
      "  --fixed-bank DEG         Test driver; hold bank command\n"
      "  --attitude-replay FILE   CSV: time_s,aoa_deg,bank_deg,gear_down\n"
      "  --replay-mode MODE       command (servo) or actual (open-loop physics), default command\n"
      "  --command-port PORT      UDP JSON command input, default 8795 (0 disables)\n"
      "  --telemetry-host HOST    UDP telemetry destination, default 127.0.0.1\n"
      "  --telemetry-port PORT    Guidance telemetry UDP destination, default 8796 (0 disables)\n"
      "  --web-telemetry-port PORT  Telemetry Web UDP destination, default 8797 (0 disables)\n"
      "  --record FILE            Also record telemetry JSONL\n"
      "  --quiet                  Suppress stdout telemetry; summary still printed\n"
      "  --gear-down              Start with gear down\n"
      "  --start-paused           Bind I/O but do not advance until resume command\n"
      "  --lockstep               Wait for a command with step=true after each telemetry frame\n",p);
}
int main(int argc,char **argv){
    const char *scenario_path=NULL,*atm_path=NULL,*aero_path=NULL,*aero_book_path=NULL,*attitude_path=NULL,*record_path=NULL,*thost="127.0.0.1",*replay_path=NULL,*replay_mode="command";
    double dt=0.02,rate=0.0,telemetry_hz=10.0,max_time=2400.0,fixed_aoa=0,fixed_bank=0;
    bool has_fixed_aoa=false,has_fixed_bank=false,quiet=false,start_gear=false,start_paused=false,lockstep=false;int command_port=8795,telemetry_port=8796,web_telemetry_port=8797;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--help")){usage(argv[0]);return 0;}
        else if(!strcmp(argv[i],"--scenario")&&i+1<argc)scenario_path=argv[++i];
        else if(!strcmp(argv[i],"--atmosphere")&&i+1<argc)atm_path=argv[++i];
        else if(!strcmp(argv[i],"--aero")&&i+1<argc)aero_path=argv[++i];
        else if(!strcmp(argv[i],"--aero-book")&&i+1<argc)aero_book_path=argv[++i];
        else if(!strcmp(argv[i],"--attitude")&&i+1<argc)attitude_path=argv[++i];
        else if(!strcmp(argv[i],"--dt")&&i+1<argc)dt=strtod(argv[++i],NULL);
        else if(!strcmp(argv[i],"--rate")&&i+1<argc){const char *r=argv[++i];rate=!strcmp(r,"max")?0.0:strtod(r,NULL);}
        else if(!strcmp(argv[i],"--telemetry-hz")&&i+1<argc)telemetry_hz=strtod(argv[++i],NULL);
        else if(!strcmp(argv[i],"--max-sim-time")&&i+1<argc)max_time=strtod(argv[++i],NULL);
        else if(!strcmp(argv[i],"--fixed-aoa")&&i+1<argc){fixed_aoa=strtod(argv[++i],NULL);has_fixed_aoa=true;}
        else if(!strcmp(argv[i],"--fixed-bank")&&i+1<argc){fixed_bank=strtod(argv[++i],NULL);has_fixed_bank=true;}
        else if(!strcmp(argv[i],"--attitude-replay")&&i+1<argc)replay_path=argv[++i];
        else if(!strcmp(argv[i],"--replay-mode")&&i+1<argc)replay_mode=argv[++i];
        else if(!strcmp(argv[i],"--command-port")&&i+1<argc)command_port=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--telemetry-host")&&i+1<argc)thost=argv[++i];
        else if(!strcmp(argv[i],"--telemetry-port")&&i+1<argc)telemetry_port=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--web-telemetry-port")&&i+1<argc)web_telemetry_port=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--record")&&i+1<argc)record_path=argv[++i];
        else if(!strcmp(argv[i],"--quiet"))quiet=true;
        else if(!strcmp(argv[i],"--gear-down"))start_gear=true;
        else if(!strcmp(argv[i],"--start-paused"))start_paused=true;
        else if(!strcmp(argv[i],"--lockstep"))lockstep=true;
        else {fprintf(stderr,"Unknown/incomplete option: %s\n",argv[i]);usage(argv[0]);return 2;}
    }
    if(dt<=0||dt>1||telemetry_hz<=0||max_time<=0||rate<0){fprintf(stderr,"Invalid numeric option.\n");return 2;}
    Scenario scenario;scenario_seed_86km(&scenario);
    if(scenario_path&&!scenario_load(scenario_path,&scenario)){fprintf(stderr,"Failed to load scenario: %s\n",scenario_path);return 3;}
    Simulation sim;sim_init(&sim,&scenario);sim.physics_dt_s=dt;sim.state.gear_down=start_gear;sim.paused=start_paused;
    if(atm_path&&!world_load_atmosphere_csv(&sim.world,atm_path)){fprintf(stderr,"Failed to load atmosphere: %s\n",atm_path);return 3;}
    if(aero_path&&!aero_load_csv(&sim.aero,aero_path)){fprintf(stderr,"Failed to load aero table: %s\n",aero_path);return 3;}
    if(aero_book_path&&!aero_load_book_csv(&sim.aero,aero_book_path)){fprintf(stderr,"Failed to load aero data book: %s\n",aero_book_path);return 3;}
    if(attitude_path&&!attitude_load_ini(&sim.state.attitude,attitude_path)){fprintf(stderr,"Failed to load attitude model: %s\n",attitude_path);return 3;}
    if(has_fixed_aoa||has_fixed_bank){double a=has_fixed_aoa?fixed_aoa:rad2deg(sim.state.attitude.cmd_aoa_rad);double b=has_fixed_bank?fixed_bank:rad2deg(sim.state.attitude.cmd_bank_rad);sim_set_attitude(&sim,a,b);}
    /* The first lockstep packet is a real state, not an uninitialized force
       sample. Refresh after loading the selected atmosphere and aero models,
       without advancing position, velocity, attitude or simulator time. */
    sim.state.aero=aero_compute(&sim.world,&sim.aero,sim.state.position_i_m,
        sim.state.velocity_i_mps,sim.state.ut,sim.state.mass_kg,
        sim.state.attitude.aoa_rad,sim.state.attitude.bank_rad);
    AttitudeReplay replay={0};
    if(replay_path&&!replay_load_csv(&replay,replay_path)){fprintf(stderr,"Failed to load attitude replay: %s\n",replay_path);return 3;}
    if(strcmp(replay_mode,"command")&&strcmp(replay_mode,"actual")){fprintf(stderr,"--replay-mode must be command or actual\n");replay_free(&replay);return 2;}
    Protocol proto;if(!protocol_open(&proto,command_port,thost,telemetry_port,web_telemetry_port)){fprintf(stderr,"Warning: UDP protocol unavailable; continuing stdout-only.\n");memset(&proto,0,sizeof(proto));proto.command_fd=proto.telemetry_fd=-1;}
    FILE *record=NULL;
    if(record_path){record=fopen(record_path,"w");if(!record){fprintf(stderr,"Failed to open record file: %s\n",record_path);protocol_close(&proto);replay_free(&replay);return 3;}}
    double wall0=monotonic_s(),next_pub=0.0,pub_period=1.0/telemetry_hz;char json[4096];
    const char *resend_env=getenv("SHUTTLESIM_LOCKSTEP_RESEND");
    bool lockstep_resend=resend_env&&strcmp(resend_env,"1")==0;
    if(lockstep&&!sim.paused){
        double wall=monotonic_s()-wall0;
        double observed=wall>1e-6?sim.state.sim_elapsed_s/wall:0;
        sim_build_telemetry_json(&sim,observed,json,sizeof(json));
        if(!quiet){puts(json);fflush(stdout);}
        if(record){fprintf(record,"%s\n",json);fflush(record);}
        protocol_send_telemetry(&proto,json,strlen(json));next_pub+=pub_period;
        bool advance=false;double resend_at=monotonic_s()+0.02;
        while(!advance){
            SimCommand lc;
            while(protocol_poll_command(&proto,&lc)){
                sim_apply_command(&sim,&lc);
                if(lc.step||lc.resume)advance=true;
            }
            double now=monotonic_s();
            if(lockstep_resend&&!advance&&now>=resend_at){
                protocol_send_telemetry(&proto,json,strlen(json));
                resend_at=now+0.02;
            }
            if(!advance)sleep_s(0.0002);
        }
    }
    while(sim.state.sim_elapsed_s<max_time){
        SimCommand cmd;while(protocol_poll_command(&proto,&cmd))sim_apply_command(&sim,&cmd);
        if(sim.paused){sleep_s(0.005);continue;}
        if(replay_path){
            ReplayPoint rp;
            if(replay_sample(&replay,sim.state.sim_elapsed_s,&rp)){
                sim.state.gear_down=rp.gear_down;
                if(!strcmp(replay_mode,"actual")){
                    sim.state.attitude.aoa_rad=deg2rad(rp.aoa_deg);
                    sim.state.attitude.bank_rad=deg2rad(rp.bank_deg);
                    sim.state.attitude.cmd_aoa_rad=sim.state.attitude.requested_aoa_rad=sim.state.attitude.aoa_rad;
                    sim.state.attitude.cmd_bank_rad=sim.state.attitude.requested_bank_rad=sim.state.attitude.bank_rad;
                    sim.state.attitude.aoa_rate_rad_s=0;
                    sim.state.attitude.bank_rate_rad_s=0;
                }else{
                    sim_set_attitude(&sim,rp.aoa_deg,rp.bank_deg);
                }
            }
        }
        sim_step(&sim,dt);
        if(sim.state.sim_elapsed_s+1e-12>=next_pub){
            double wall=monotonic_s()-wall0;double observed=wall>1e-6?sim.state.sim_elapsed_s/wall:0;
            sim_build_telemetry_json(&sim,observed,json,sizeof(json));
            if(!quiet){puts(json);fflush(stdout);}
            if(record){fprintf(record,"%s\n",json);fflush(record);}
            protocol_send_telemetry(&proto,json,strlen(json));next_pub+=pub_period;
            if(lockstep){
                bool advance=false;double resend_at=monotonic_s()+0.02;
                while(!advance){
                    SimCommand lc;
                    while(protocol_poll_command(&proto,&lc)){
                        sim_apply_command(&sim,&lc);
                        if(lc.step||lc.resume)advance=true;
                    }
                    double now=monotonic_s();
                    if(lockstep_resend&&!advance&&now>=resend_at){
                        protocol_send_telemetry(&proto,json,strlen(json));
                        resend_at=now+0.02;
                    }
                    if(!advance)sleep_s(0.0002);
                }
            }
        }
        if(rate>0){double target=sim.state.sim_elapsed_s/rate;double wall=monotonic_s()-wall0;if(target>wall)sleep_s(target-wall);}
        if(sim.state.on_ground){
            Vec3 surf=v3_sub(sim.state.velocity_i_mps,world_atmosphere_velocity_i(&sim.world,sim.state.position_i_m));
            Vec3 up=v3_normalized(sim.state.position_i_m);surf=v3_sub(surf,v3_scale(up,v3_dot(surf,up)));
            if(v3_norm(surf)<0.5)break;
        }
    }
    sim_build_summary_json(&sim,json,sizeof(json));fprintf(stderr,"SHUTTLESIM_SUMMARY %s\n",json);
    if(record) fclose(record);
    protocol_close(&proto);
    replay_free(&replay);
    return 0;
}
