#define _POSIX_C_SOURCE 200809L
#include "landing.h"
#include "json.h"
#include "sim_telemetry.h"

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static double wrap_deg(double x){while(x>180)x-=360;while(x<-180)x+=360;return x;}
static Vector3 vsub2(Vector3 a,Vector3 b){return (Vector3){a.x-b.x,a.y-b.y,a.z-b.z};}
static Vector3 vscale2(Vector3 a,double s){return (Vector3){a.x*s,a.y*s,a.z*s};}
static Vector3 vnorm2(Vector3 a){
    double n=sqrt(a.x*a.x+a.y*a.y+a.z*a.z); return n>1e-12?vscale2(a,1.0/n):(Vector3){0,0,0};
}
static Vector3 omega_cross(Vector3 p,double w){return (Vector3){-w*p.y,w*p.x,0};}

static int obj_get(const JsonDoc*d,int obj,const char*k){return obj>=0?json_object_get(d,obj,k):-1;}
static bool jbool(const JsonDoc*d,int obj,const char*k,bool def){int i=obj_get(d,obj,k);return i>=0?json_boolean(d,i,def):def;}

static const char* phase_name(GuidancePhase p){
    static const char*n[]={"IDLE","PLANNING","CALIBRATION","COAST","BURN_SETUP","DEORBIT_BURN",
      "ENTRY_INTERFACE","ENTRY_ENERGY","TAEM","HEADING_ALIGNMENT","FINAL","FLARE","TOUCHDOWN",
      "ROLLOUT","COMPLETE","PAUSED","ABORT","FAULT","ATTITUDE_RECOVERY"};
    return p>=0&&p<(int)(sizeof(n)/sizeof(n[0]))?n[p]:"?";
}

static bool load_config(const char*path,LandingConfiguration*c){
    *c=landing_configuration_default();
    FILE*f=fopen(path,"rb"); if(!f){landing_configuration_normalize(c);return false;}
    fseek(f,0,SEEK_END); long n=ftell(f); rewind(f);
    char*buf=calloc((size_t)n+1,1); if(!buf){fclose(f);return false;}
    fread(buf,1,(size_t)n,f); fclose(f);
    JsonToken*toks=calloc(16384,sizeof(*toks)); JsonDoc d;
    bool ok=toks&&json_parse(buf,toks,16384,&d)>0&&d.tokens[0].type==JSON_OBJECT&&landing_configuration_from_json(c,&d,0);
    free(toks);free(buf);landing_configuration_normalize(c);return ok;
}


static bool parse_packet(const char*buf,const PlanetModel*p,
                         Telemetry*t,VehicleState*s,bool*on_ground,bool*td_seen,bool*td_runway){
    char error[256];
    if(!shuttle_sim_decode_telemetry(buf,p,NULL,t,s,error,sizeof(error)))return false;

    /* Touchdown bookkeeping is runner-local.  All production telemetry fields
       come from the shared ShuttleSim transport adapter so rate frames,
       quaternion order, response-model validity and confidence semantics cannot
       silently diverge from the landing backend. */
    JsonToken tok[512];JsonDoc d;
    if(json_parse(buf,tok,512,&d)<1||d.tokens[0].type!=JSON_OBJECT)return false;
    int gr=obj_get(&d,0,"ground");if(gr<0)return false;

    *on_ground=jbool(&d,gr,"on_ground",false);
    *td_seen=jbool(&d,gr,"touchdown_seen",false);
    *td_runway=jbool(&d,gr,"on_runway_touchdown",false);
    return true;
}

static void fill_velocity_and_course(Telemetry*t,VehicleState*s,const PlanetModel*p,
                                     bool*have_prev,Vector3*prev_pos,double*prev_ut,double*prev_course){
    if(*have_prev&&t->ut>*prev_ut){
        double dt=t->ut-*prev_ut;
        if(!isfinite(s->velocity.x)||!isfinite(s->velocity.y)||!isfinite(s->velocity.z))
            s->velocity=vscale2(vsub2(s->position,*prev_pos),1.0/dt);
        Vector3 omega={0,0,p->rotational_speed};
        t->ground_track_heading=surface_course(s->position,s->velocity,omega,p->north_axis,t->heading);
        t->course_rate=wrap_deg(t->ground_track_heading-*prev_course)/dt;t->has_course_rate=true;
        *prev_course=t->ground_track_heading;
        GeoPoint here={t->latitude,t->longitude,t->mean_altitude};
        (void)here;
    }else{
        /* Tangential eastward seed; corrected on the next packet. */
        Vector3 up=vnorm2(s->position), east=vnorm2((Vector3){-up.y,up.x,0});
        Vector3 atm=omega_cross(s->position,p->rotational_speed);
        s->velocity=vscale2(east,t->true_air_speed);
        s->velocity=(Vector3){s->velocity.x+atm.x,s->velocity.y+atm.y,s->velocity.z+atm.z};
        *prev_course=t->heading;
    }
    *prev_pos=s->position;*prev_ut=t->ut;*have_prev=true;
}

static int open_receiver(int port){
    int fd=socket(AF_INET,SOCK_DGRAM,0);if(fd<0)return -1;
    int one=1;setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in a={0};a.sin_family=AF_INET;a.sin_addr.s_addr=inet_addr("127.0.0.1");a.sin_port=htons((unsigned short)port);
    if(bind(fd,(struct sockaddr*)&a,sizeof(a))<0){close(fd);return -1;}
    struct timeval tv={.tv_sec=5,.tv_usec=0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));return fd;
}
static int open_sender(void){return socket(AF_INET,SOCK_DGRAM,0);}
static void send_json(int fd,int port,const char*json){
    struct sockaddr_in a={0};a.sin_family=AF_INET;a.sin_port=htons((unsigned short)port);a.sin_addr.s_addr=inet_addr("127.0.0.1");
    sendto(fd,json,strlen(json),0,(struct sockaddr*)&a,sizeof(a));
}
static void send_command(int fd,int port,const GuidanceCommand*c,const Telemetry*t){
    double aoa=c->has_target_aoa?c->target_aoa:c->target_pitch-t->flight_path_angle;
    if(!isfinite(aoa))aoa=t->angle_of_attack;
    double bank=isfinite(c->target_roll)?c->target_roll:t->roll;
    char b[640];
    snprintf(b,sizeof(b),"{\"type\":\"attitude_command\",\"aoa_deg\":%.8g,\"bank_deg\":%.8g,\"gear_down\":%s,\"brakes\":%s,\"airbrakes\":%s,\"wheel_steering\":%.8g,\"throttle\":0,\"step\":true}",
             aoa,bank,c->gear?"true":"false",c->brakes?"true":"false",
             c->airbrakes?"true":"false",ss_clampd(c->wheel_steering,-1.0,1.0));
    send_json(fd,port,b);
}
static void send_resume(int fd,int port){send_json(fd,port,"{\"type\":\"resume\"}");}

int main(int argc,char**argv){
    const char*config_path=argc>1?argv[1]:"Configuration/default.json";
    int rx_port=getenv("SIM_GUIDANCE_RX_PORT")?atoi(getenv("SIM_GUIDANCE_RX_PORT")):8796;
    int tx_port=getenv("SIM_GUIDANCE_TX_PORT")?atoi(getenv("SIM_GUIDANCE_TX_PORT")):8795;
    LandingConfiguration cfg;if(!load_config(config_path,&cfg))
        fprintf(stderr,"guidance-runner: using built-in/default-normalized config (could not fully parse %s)\n",config_path);
    char default_atmosphere[512];
    const char *atmosphere=argc>2?argv[2]:shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,
        default_atmosphere,sizeof(default_atmosphere));
    if(!atmosphere){fprintf(stderr,"guidance-runner: atmosphere path too long\n");return 2;}
    PlanetModel planet;char error[256];
    if(!shuttle_sim_load_planet(atmosphere,&planet,error,sizeof(error))){
        fprintf(stderr,"guidance-runner: %s\n",error);return 2;
    }
    AerodynamicModel aero={cfg.vehicle.estimated_lift_to_drag,cfg.vehicle.estimated_ballistic_coefficient,.9};
    AerodynamicEnvelope envelope={0};for(int i=0;i<4;i++)envelope.regimes[i]=aero;
    TrajectoryCalibrationModel calibration={0};
    calibration.density_scale=1;calibration.drag_scale=1;calibration.lift_scale=1;calibration.bank_effectiveness=1;
    calibration.speed_of_sound=300;calibration.speed_of_sound_scale=1;
    calibration.stress_drag_scale=1;calibration.stress_lift_scale=1;calibration.confidence=.9;
    DeorbitPlan plan={0};plan.execution_qualified=true;plan.target_capture_achieved=true;plan.delta_v=0;plan.predicted_post_burn_periapsis_altitude=50000;
    GuidanceMachine g;guidance_machine_init(&g);
    bool guidance_initialized=false;

    int rx=open_receiver(rx_port),tx=open_sender();if(rx<0||tx<0){perror("guidance-runner socket");return 2;}
    fprintf(stderr,"guidance-runner: listening udp://127.0.0.1:%d -> commands %d\n",rx_port,tx_port);
    send_resume(tx,tx_port);

    bool have_prev=false;Vector3 prev_pos={0};double prev_ut=0,prev_course=90;
    GuidancePhase last_phase=(GuidancePhase)-1;unsigned ticks=0;double last_log_ut=-1e30;
    char buf[8192];
    for(;;){
        ssize_t n=recv(rx,buf,sizeof(buf)-1,0);
        if(n<0){if(errno==EAGAIN||errno==EWOULDBLOCK){fprintf(stderr,"guidance-runner: telemetry timeout\n");continue;}perror("recv");break;}
        buf[n]=0;Telemetry t;VehicleState s;bool on_ground=false,td_seen=false,td_runway=false;
        if(!parse_packet(buf,&planet,&t,&s,&on_ground,&td_seen,&td_runway))continue;
        fill_velocity_and_course(&t,&s,&planet,&have_prev,&prev_pos,&prev_ut,&prev_course);
        shuttle_sim_prepare_guidance_telemetry(&t,&cfg,&planet);
        if(!guidance_initialized){
            const char *final_test=getenv("KSP_LANDER_FINAL_APPROACH_TEST");
            if(final_test&&strcmp(final_test,"1")==0){
                char status[512]={0};
                double course=isfinite(t.heading)?t.heading:t.ground_track_heading;
                if(!guidance_begin_final_test(&g,&t,course,&planet,aero,&cfg,status,sizeof(status))){
                    fprintf(stderr,"guidance-runner: final-only admission rejected: %s\n",status[0]?status:"unknown reason");
                    send_resume(tx,tx_port);
                    break;
                }
                fprintf(stderr,"guidance-runner: final-only armed: %s\n",status);
            }else{
                guidance_initialize_reentry_continuation(&g,&t,&planet,&cfg,1.0,false,&envelope,&calibration);
            }
            guidance_initialized=true;
        }
        double entry_ref=g.entry_reference_speed>0?g.entry_reference_speed:t.true_air_speed;
        EntryTerminalDemand demand=entry_terminal_demand(t.latitude,t.mean_altitude,t.range_to_site,t.horizontal_speed,
            t.course_to_site_error,t.vertical_speed,t.true_air_speed,entry_ref,&planet,&cfg.vehicle,&cfg.site,&cfg.guidance);
        t.energy_excess_range=-demand.projected_taem_range_error;

        GuidanceResult r=guidance_update(&g,&t,&s,&plan,&planet,aero,&cfg);
        send_command(tx,tx_port,&r.command,&t);
        ticks++;
        if(r.phase!=last_phase||t.ut-last_log_ut>=10.0){
            fprintf(stderr,
              "SIM_GUIDANCE ut=%.2f phase=%s h=%.0f V=%.1f vs=%.1f range=%.0f along=%.0f cross=%.0f course=%.1f aoa=%.1f roll=%.1f cmdAoA=%.1f cmdRoll=%.1f excess=%.0f status=%s%s%s\n",
              t.ut,phase_name(r.phase),t.mean_altitude,t.true_air_speed,t.vertical_speed,t.range_to_site,
              t.runway_along_track,t.runway_cross_track,t.ground_track_heading,t.angle_of_attack,t.roll,
              r.command.has_target_aoa?r.command.target_aoa:r.command.target_pitch-t.flight_path_angle,
              r.command.target_roll,t.energy_excess_range,r.status,r.has_warning?" warning=":"",r.has_warning?r.warning:"");
            last_phase=r.phase;last_log_ut=t.ut;
        }
        if(r.phase==PHASE_ABORT||r.phase==PHASE_FAULT){
            fprintf(stderr,"SIM_GUIDANCE_STOP phase=%s status=%s warning=%s\n",phase_name(r.phase),r.status,r.has_warning?r.warning:"");
            guidance_result_clear(&r);break;
        }
        if(on_ground&&td_seen){
            fprintf(stderr,"SIM_TOUCHDOWN runway=%s along=%.2f cross=%.2f speed=%.2f phase=%s ticks=%u\n",
                    td_runway?"true":"false",t.runway_along_track,t.runway_cross_track,t.surface_speed,phase_name(r.phase),ticks);
            if(t.surface_speed<0.5||!td_runway){guidance_result_clear(&r);break;}
        }
        guidance_result_clear(&r);
    }
    close(rx);close(tx);return 0;
}
