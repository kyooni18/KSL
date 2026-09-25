#include "landing.h"
#include "flight_control.h"
#include "krpc_cnano_client.h"
#include "physics_store.h"
#include "sim_telemetry.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

struct KRPCSession {
    KrpcCNanoClient *client;
    FlightControlState flight_control;
    PhysicsStore *physics_store;
    VesselAeroSample physics_history[VESSEL_AERO_SAMPLES];
    unsigned physics_history_count;
    char physics_structure_id[128];
    char physics_environment_id[128];
    char physics_storage[1024];
    char runtime_transport[192];
    char runtime_library[64];
    char last_control_profile[24];
    Telemetry last_telemetry;
    bool has_last_telemetry;
    VehicleState last_state;
    bool has_last_state;
    double last_control_ut;
    bool has_last_control_ut;
    bool simulator;
    int sim_rx_fd, sim_tx_fd;
    struct sockaddr_in sim_command_addr;
    PlanetModel sim_planet;
    char sim_vessel[128];
    bool sim_need_packet;
};

static void set_error(char *out,size_t n,const char *fmt,...){
    if(!out||n==0)return;va_list ap;va_start(ap,fmt);vsnprintf(out,n,fmt,ap);va_end(ap);
}

static double realtime_seconds(void){
    struct timespec ts;if(clock_gettime(CLOCK_REALTIME,&ts)!=0)return 0.0;
    return (double)ts.tv_sec+(double)ts.tv_nsec/1e9;
}

static Vector3 canonical_to_raw(Vector3 value){return v3(value.x,value.z,value.y);}

static void project_root(char *out,size_t out_size){
    const char *configured=getenv("KSP_LANDER_ROOT");
    if(configured&&*configured){snprintf(out,out_size,"%s",configured);return;}
    if(!getcwd(out,out_size)){snprintf(out,out_size,".");return;}
    if(access("Configuration/default.json",R_OK)==0)return;
    if(access("../Configuration/default.json",R_OK)==0){char parent[1024];if(realpath("..",parent))snprintf(out,out_size,"%s",parent);}
}

static char *build_structure_manifest(const LandingConfiguration *cfg,const char *vessel){
    JsonWriter w;jw_init(&w);jw_raw(&w,"{\"schema\":3,\"modelId\":");jw_string(&w,cfg->vehicle.model_id);
    jw_raw(&w,",\"vesselName\":");jw_string(&w,vessel?vessel:"");
    /* Model ID is the authoritative manual identity. A later native parts
       inventory can add a geometry witness without changing that durable key. */
    jw_raw(&w,",\"parts\":[],\"geometryComplete\":false,\"geometry\":[]}");
    char *result=!w.failed&&w.data?strdup(w.data):NULL;jw_free(&w);return result;
}

static char *build_environment_manifest(const PlanetModel *planet){
    JsonWriter w;jw_init(&w);jw_raw(&w,"{\"schema\":3,\"planet\":{\"name\":");jw_string(&w,planet->name);
    jw_raw(&w,",\"radius\":");jw_number(&w,planet->radius);
    jw_raw(&w,",\"gravitationalParameter\":");jw_number(&w,planet->gravitational_parameter);
    jw_raw(&w,",\"rotationalSpeed\":");jw_number(&w,planet->rotational_speed);
    jw_raw(&w,",\"atmosphereDepth\":");jw_number(&w,planet->atmosphere_depth);
    jw_raw(&w,",\"surfaceDensity\":");jw_number(&w,planet->surface_density);
    jw_raw(&w,",\"atmosphereAdiabaticIndex\":");jw_number(&w,planet->atmosphere_adiabatic_index);
    jw_raw(&w,"},\"atmosphereCurveDigest\":null}");
    char *result=!w.failed&&w.data?strdup(w.data):NULL;jw_free(&w);return result;
}

static void open_physics_store(KRPCSession *session,const LandingConfiguration *cfg){
    const PlanetModel *planet=krpc_cnano_client_planet(session->client);
    char root[1024];project_root(root,sizeof(root));char runtime[1200];snprintf(runtime,sizeof(runtime),"%s/Runtime",root);mkdir(runtime,0700);
    char physics_dir[1300];snprintf(physics_dir,sizeof(physics_dir),"%s/Physics",runtime);mkdir(physics_dir,0700);
    snprintf(session->physics_storage,sizeof(session->physics_storage),"%s/observations.sqlite3",physics_dir);
    char *structure=build_structure_manifest(cfg,krpc_cnano_client_vessel(session->client));
    char *environment=build_environment_manifest(planet);
    if(!structure||!environment){free(structure);free(environment);return;}
    PhysicsStoreOptions options={
        .path=session->physics_storage,.model_id=cfg->vehicle.model_id,
        .vessel_name=krpc_cnano_client_vessel(session->client),
        .structure_manifest_json=structure,.environment_manifest_json=environment,
        .flight_key=krpc_cnano_client_flight_key(session->client),.mode=PHYSICS_STORE_READ_WRITE};
    /* A save reload is a new physical experiment, not a continuation of the
     * original launch. Keep the normal same-flight holdout, but in explicitly
     * forced unpowered checkpoint development exclude this native session only.
     * Earlier completed attempts remain prior evidence; current frames do not. */
    const char *checkpoint=getenv("KSP_LANDER_FORCE_REENTRY_TEST");
    const char *unpowered=getenv("KSP_LANDER_UNPOWERED_ONLY");
    bool checkpoint_attempt=checkpoint&&strcmp(checkpoint,"1")==0&&
        unpowered&&strcmp(unpowered,"1")==0;
    if(checkpoint_attempt)options.flight_key=NULL;
    char error[512]={0};session->physics_store=physics_store_open(&options,error,sizeof(error));
    free(structure);free(environment);
    if(!session->physics_store){
        snprintf(session->physics_structure_id,sizeof(session->physics_structure_id),"model:%s",cfg->vehicle.model_id);
        snprintf(session->physics_environment_id,sizeof(session->physics_environment_id),"unavailable");
        char disabled[sizeof(session->physics_storage)];snprintf(disabled,sizeof(disabled),"disabled: %.900s",error[0]?error:"open failed");
        snprintf(session->physics_storage,sizeof(session->physics_storage),"%s",disabled);return;
    }
    snprintf(session->physics_structure_id,sizeof(session->physics_structure_id),"%s",physics_store_structure_id(session->physics_store));
    snprintf(session->physics_environment_id,sizeof(session->physics_environment_id),"%s",physics_store_environment_id(session->physics_store));
    session->physics_history_count=(unsigned)physics_store_load_history(session->physics_store,planet,
        session->physics_history,VESSEL_AERO_SAMPLES,error,sizeof(error));
    if(checkpoint_attempt)fprintf(stderr,"Checkpoint aerodynamic prior: attempt %s, %u prior cells; current attempt held out.\n",
        physics_store_flight_key(session->physics_store),session->physics_history_count);
}



static void open_sim_physics_store(KRPCSession *session,const LandingConfiguration *cfg){
    char root[1024];project_root(root,sizeof(root));char runtime[1200];snprintf(runtime,sizeof(runtime),"%s/Runtime",root);
    char physics_dir[1300];snprintf(physics_dir,sizeof(physics_dir),"%s/Physics",runtime);
    snprintf(session->physics_storage,sizeof(session->physics_storage),"%s/observations.sqlite3",physics_dir);
    char *structure=build_structure_manifest(cfg,session->sim_vessel);
    char *environment=build_environment_manifest(&session->sim_planet);
    if(!structure||!environment){free(structure);free(environment);return;}
    PhysicsStoreOptions options={.path=session->physics_storage,.model_id=cfg->vehicle.model_id,
        .vessel_name=session->sim_vessel,.structure_manifest_json=structure,.environment_manifest_json=environment,
        .flight_key="shuttlesim-holdout",.mode=PHYSICS_STORE_READ_ONLY};
    char err[512]={0};session->physics_store=physics_store_open(&options,err,sizeof(err));
    free(structure);free(environment);
    if(!session->physics_store){
        snprintf(session->physics_structure_id,sizeof(session->physics_structure_id),"model:%s",cfg->vehicle.model_id);
        snprintf(session->physics_environment_id,sizeof(session->physics_environment_id),"sim-kerbin");
        return;
    }
    snprintf(session->physics_structure_id,sizeof(session->physics_structure_id),"%s",physics_store_structure_id(session->physics_store));
    snprintf(session->physics_environment_id,sizeof(session->physics_environment_id),"%s",physics_store_environment_id(session->physics_store));
    session->physics_history_count=(unsigned)physics_store_load_history(session->physics_store,&session->sim_planet,
        session->physics_history,VESSEL_AERO_SAMPLES,err,sizeof(err));
}


static void sim_load_force_book(KRPCSession *s){
    char root[1024];project_root(root,sizeof(root));char path[1400];
    snprintf(path,sizeof(path),"%s/ShuttleSim/data/fitted/stsn_force_book.csv",root);
    const char*configured=getenv("KSP_LANDER_TERMINAL_AERO_BOOK");
    if(configured&&strcmp(configured,"none")==0)return;
    const char*source=configured&&*configured?configured:path;
    FILE*f=fopen(source,"r");if(!f)return;
    VesselAeroSample candidates[256];unsigned n=0;char line[512];
    while(fgets(line,sizeof(line),f)&&n<256){
        if(strstr(line,"q_pa"))continue;
        double q,mach,aoa,lift_per_q,drag_per_q,support=1;
        int got=sscanf(line,"%lf,%lf,%lf,%lf,%lf,%lf",&q,&mach,&aoa,&lift_per_q,&drag_per_q,&support);
        if(got<5||!isfinite(q)||q<=1||!isfinite(mach)||!isfinite(aoa)||!isfinite(lift_per_q)||!isfinite(drag_per_q))continue;
        unsigned obs=(unsigned)fmax(1.0,round(got>=6?support:1.0));
        candidates[n++]=(VesselAeroSample){
            .q=q,.mach=mach,.aoa=aoa,.beta=0,.mass=40252.91796875,
            .force_per_q=v3(fmax(0,drag_per_q),fmax(0,lift_per_q),0),
            .gear=false,.brakes=false,.airbrakes=0,.observations=obs,
            .trust=clampd(.72+.035*(double)obs,.72,1.0),.last_observation_ut=0};
    }
    fclose(f);
    /* Guidance intentionally bounds its certified data book at 192 cells.
       Retain the strongest independent-flight cells when the simulator export
       is slightly denser; this drops only the weakest eight cells today. */
    for(unsigned i=0;i<n;i++)for(unsigned j=i+1;j<n;j++){
        bool swap=candidates[j].observations>candidates[i].observations||
            (candidates[j].observations==candidates[i].observations&&candidates[j].trust>candidates[i].trust);
        if(swap){VesselAeroSample tmp=candidates[i];candidates[i]=candidates[j];candidates[j]=tmp;}
    }
    s->physics_history_count=n<VESSEL_AERO_SAMPLES?n:VESSEL_AERO_SAMPLES;
    for(unsigned i=0;i<s->physics_history_count;i++)s->physics_history[i]=candidates[i];
    snprintf(s->physics_structure_id,sizeof(s->physics_structure_id),"model:STS-N:shuttlesim-force-book");
    snprintf(s->physics_environment_id,sizeof(s->physics_environment_id),"Kerbin:ShuttleSim-calibrated");
    snprintf(s->physics_storage,sizeof(s->physics_storage),"%s",source);
}

static bool sim_session_open(KRPCSession *s,const LandingConfiguration *cfg,char *error,size_t error_size){
    s->simulator=true;s->sim_rx_fd=-1;s->sim_tx_fd=-1;s->sim_need_packet=true;
    char root[1024], atmosphere[1400];project_root(root,sizeof(root));
    snprintf(atmosphere,sizeof(atmosphere),"%s/ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv",root);
    const char*configured=getenv("KSP_LANDER_TERMINAL_ATMOSPHERE");
    if(!shuttle_sim_load_planet(configured&&*configured?configured:atmosphere,
            &s->sim_planet,error,error_size))return false;
    snprintf(s->sim_vessel,sizeof(s->sim_vessel),"STS-N");
    int ports[]={8796,8795};
    const char*names[]={"KSP_LANDER_SIM_TELEMETRY_PORT","KSP_LANDER_SIM_COMMAND_PORT"};
    for(size_t i=0;i<sizeof(ports)/sizeof(ports[0]);i++){
        const char*value=getenv(names[i]);
        if(value&&*value){
            char*end=NULL;errno=0;long parsed=strtol(value,&end,10);
            if(errno||end==value||*end||parsed<1||parsed>UINT16_MAX){
                set_error(error,error_size,"Invalid ShuttleSim UDP port in %s",names[i]);return false;
            }
            ports[i]=(int)parsed;
        }
    }
    int rx_port=ports[0],tx_port=ports[1];
    if(rx_port==tx_port){set_error(error,error_size,"ShuttleSim command and telemetry ports must differ");return false;}
    s->sim_rx_fd=socket(AF_INET,SOCK_DGRAM,0);s->sim_tx_fd=socket(AF_INET,SOCK_DGRAM,0);
    if(s->sim_rx_fd<0||s->sim_tx_fd<0){set_error(error,error_size,"Could not create ShuttleSim UDP sockets");return false;}
    struct sockaddr_in rx={0};rx.sin_family=AF_INET;rx.sin_addr.s_addr=htonl(INADDR_LOOPBACK);rx.sin_port=htons((unsigned short)rx_port);
    if(bind(s->sim_rx_fd,(struct sockaddr*)&rx,sizeof(rx))<0){set_error(error,error_size,"Could not bind ShuttleSim telemetry UDP %d: %s",rx_port,strerror(errno));return false;}
    struct timeval tv={.tv_sec=10,.tv_usec=0};setsockopt(s->sim_rx_fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
    memset(&s->sim_command_addr,0,sizeof(s->sim_command_addr));s->sim_command_addr.sin_family=AF_INET;
    s->sim_command_addr.sin_port=htons((unsigned short)tx_port);inet_pton(AF_INET,"127.0.0.1",&s->sim_command_addr.sin_addr);
    flight_control_init(&s->flight_control,1.0/clampd(cfg->guidance.guidance_rate,2,30));
    snprintf(s->runtime_transport,sizeof(s->runtime_transport),"shuttlesim-udp-lockstep");
    snprintf(s->runtime_library,sizeof(s->runtime_library),"ShuttleSim");
    snprintf(s->last_control_profile,sizeof(s->last_control_profile),"entry");
    open_sim_physics_store(s,cfg);
    sim_load_force_book(s);
    return true;
}

static bool sim_read_telemetry(KRPCSession*s,Telemetry*t,VehicleState*state,char*error,size_t error_size){
    if(!s->sim_need_packet&&s->has_last_telemetry&&s->has_last_state){*t=s->last_telemetry;*state=s->last_state;return true;}

    /* ShuttleSim lockstep retransmits the current frame while waiting for a
       step command.  Those duplicate UDP packets can remain queued after the
       simulator advances, making the backend run multiple guidance ticks on
       one physics state.  Discard equal/past UT frames whenever a new packet is
       required, so one accepted telemetry packet corresponds to one sim step. */
    double last_ut=s->has_last_telemetry?s->last_telemetry.ut:-INFINITY;
    for(;;){
        char buf[8192];ssize_t n=recvfrom(s->sim_rx_fd,buf,sizeof(buf)-1,0,NULL,NULL);
        if(n<=0){set_error(error,error_size,"Timed out waiting for ShuttleSim telemetry: %s",strerror(errno));return false;}
        buf[n]=0;
        Telemetry candidate;VehicleState candidate_state;
        telemetry_init(&candidate);memset(&candidate_state,0,sizeof(candidate_state));
        if(!shuttle_sim_decode_telemetry(buf,&s->sim_planet,
                s->has_last_telemetry?&s->last_telemetry:NULL,
                &candidate,&candidate_state,error,error_size))return false;
        if(!s->has_last_telemetry||!isfinite(last_ut)||
           candidate.ut>last_ut+1e-6){
            *t=candidate;*state=candidate_state;s->sim_need_packet=false;return true;
        }
        const char*diag=getenv("KSP_LANDER_SIM_STEP_DIAG");
        if(diag&&strcmp(diag,"1")==0)
            fprintf(stderr,"ShuttleSim duplicate telemetry discarded: UT %.6f <= %.6f.\n",
                candidate.ut,last_ut);
    }
}


static bool sim_send_guidance(KRPCSession*s,const GuidanceCommand*command,bool step,char*error,size_t error_size){
    double aoa=s->has_last_telemetry?s->last_telemetry.angle_of_attack:0.0;
    double bank=s->has_last_telemetry?s->last_telemetry.roll:0.0;
    double throttle=command&&command->control_profile==PROFILE_ORBITAL?
        clampd(command->target_throttle,0.0,1.0):0.0;
    bool gear=s->has_last_telemetry?s->last_telemetry.gear:false,brakes=s->has_last_telemetry?s->last_telemetry.brakes:false;
    bool airbrakes=false;double wheel_steering=0.0;
    if(command){if(command->has_target_aoa&&isfinite(command->target_aoa))aoa=command->target_aoa;if(isfinite(command->target_roll))bank=command->target_roll;gear=command->gear;brakes=command->brakes;airbrakes=command->airbrakes;wheel_steering=clampd(command->wheel_steering,-1.0,1.0);}
    char msg[640];snprintf(msg,sizeof(msg),"{\"type\":\"guidance\",\"aoa_deg\":%.9f,\"bank_deg\":%.9f,\"gear_down\":%s,\"brakes\":%s,\"airbrakes\":%s,\"wheel_steering\":%.9f,\"throttle\":%.9f,\"step\":%s}",
        aoa,bank,gear?"true":"false",brakes?"true":"false",airbrakes?"true":"false",wheel_steering,throttle,step?"true":"false");
    ssize_t n=sendto(s->sim_tx_fd,msg,strlen(msg),0,(struct sockaddr*)&s->sim_command_addr,sizeof(s->sim_command_addr));
    if(n<0){set_error(error,error_size,"Could not send ShuttleSim command: %s",strerror(errno));return false;}
    if(step)s->sim_need_packet=true;return true;
}

KRPCSession *krpc_session_open(const LandingConfiguration *cfg,char *error,size_t error_size){
    if(!cfg){set_error(error,error_size,"Landing configuration is missing");return NULL;}
    KRPCSession *session=calloc(1,sizeof(*session));if(!session){set_error(error,error_size,"Out of memory opening native kRPC session");return NULL;}
    session->sim_rx_fd=session->sim_tx_fd=-1;
    const char *sim=getenv("KSP_LANDER_SIMULATOR");
    if(sim&&strcmp(sim,"1")==0){if(!sim_session_open(session,cfg,error,error_size)){krpc_session_close(session);return NULL;}return session;}
    session->client=krpc_cnano_client_open(cfg,error,error_size);if(!session->client){free(session);return NULL;}
    flight_control_init(&session->flight_control,1.0/clampd(cfg->guidance.guidance_rate,2,30));
    snprintf(session->runtime_transport,sizeof(session->runtime_transport),"%s",krpc_cnano_client_transport_name(session->client));
    snprintf(session->runtime_library,sizeof(session->runtime_library),"%s",krpc_cnano_client_library_version(session->client));
    snprintf(session->last_control_profile,sizeof(session->last_control_profile),"orbital");
    open_physics_store(session,cfg);
    return session;
}

const PlanetModel *krpc_session_planet(const KRPCSession *s){if(!s)return NULL;return s->simulator?&s->sim_planet:(s->client?krpc_cnano_client_planet(s->client):NULL);}
const char *krpc_session_vessel(const KRPCSession *s){if(!s)return "";return s->simulator?s->sim_vessel:(s->client?krpc_cnano_client_vessel(s->client):"");}
const char *krpc_session_transport(const KRPCSession *s){return s?s->runtime_transport:"";}
const char *krpc_session_library_version(const KRPCSession *s){return s?s->runtime_library:"";}
bool krpc_session_is_simulator(const KRPCSession *s){return s&&s->simulator;}
KRPCTransportBudget krpc_session_transport_budget(const KRPCSession *s){
    KRPCTransportBudget out={0};if(!s||s->simulator||!s->client)return out;
    KrpcCNanoBudget in=krpc_cnano_client_budget(s->client);
    out.read_calls=in.read_calls;out.read_wire_requests=in.read_wire_requests;
    out.apply_calls=in.write_calls;out.apply_wire_requests=in.write_wire_requests;
    out.total_calls=in.total_calls;out.total_wire_requests=in.total_wire_requests;
    out.read_seconds=in.last_read_seconds;out.apply_seconds=in.last_apply_seconds;
    return out;
}
const char *krpc_session_physics_structure_id(const KRPCSession *s){return s?s->physics_structure_id:"";}
const char *krpc_session_physics_environment_id(const KRPCSession *s){return s?s->physics_environment_id:"";}
const char *krpc_session_physics_storage(const KRPCSession *s){return s?s->physics_storage:"";}
unsigned krpc_session_physics_history_count(const KRPCSession *s){return s?s->physics_history_count:0;}
void krpc_session_seed_physics(const KRPCSession *s,VesselPhysicsModel *model){if(!s||!model)return;for(unsigned i=0;i<s->physics_history_count;i++)vessel_physics_import_sample(model,&s->physics_history[i]);}

static void store_observation(KRPCSession *s,const Telemetry *t,const VehicleState *state){
    if(!s||!s->physics_store||!t||!state)return;
    PhysicsStoreObservation o={0};
    o.sample_valid=t->physics_sample_valid;o.body_non_rotating=true;o.situation=t->vessel_situation;
    o.ut=t->ut;o.wall_time=realtime_seconds();o.q=t->dynamic_pressure;o.mach=t->mach;o.aoa=t->angle_of_attack;o.beta=t->sideslip;o.roll=t->roll;o.mass=t->mass;
    o.has_dry_mass=isfinite(t->dry_mass)&&t->dry_mass>0;o.dry_mass=t->dry_mass;o.gear=t->gear;o.brakes=t->brakes;o.airbrakes=t->has_airbrakes?(t->airbrakes?1:0):-1;o.rcs=-1;
    o.position=canonical_to_raw(state->position);o.velocity=canonical_to_raw(state->velocity);o.lift=canonical_to_raw(t->lift_vector);o.drag=canonical_to_raw(t->drag_vector);
    o.has_center_of_mass_root=t->has_center_of_mass_root;o.center_of_mass_root=t->center_of_mass_root;
    o.has_moment_of_inertia=t->has_inertia;o.moment_of_inertia=v3(t->pitch_moment_of_inertia,t->roll_moment_of_inertia,t->yaw_moment_of_inertia);
    o.control_profile=s->last_control_profile;o.has_body_rates=t->has_body_pitch_rate&&t->has_body_roll_rate&&t->has_body_yaw_rate;
    o.body_rates=v3(t->body_pitch_rate,t->body_roll_rate,t->body_yaw_rate);
    char ignored[256];(void)physics_store_observe(s->physics_store,&o,ignored,sizeof(ignored));
}

bool krpc_read_telemetry(KRPCSession *s,const LandingConfiguration *cfg,Telemetry *t,VehicleState *state,char *error,size_t error_size){
    if(!s){set_error(error,error_size,"Flight session is closed");return false;}
    if(s->simulator){
        if(!sim_read_telemetry(s,t,state,error,error_size))return false;
        shuttle_sim_prepare_guidance_telemetry(t,cfg,&s->sim_planet);
        s->last_telemetry=*t;s->has_last_telemetry=true;s->last_state=*state;s->has_last_state=true;return true;
    }
    if(!s->client){set_error(error,error_size,"Native C-Nano session is closed");return false;}
    if(!krpc_cnano_client_read(s->client,cfg,t,state,error,error_size))return false;
    s->last_telemetry=*t;s->has_last_telemetry=true;s->last_state=*state;s->has_last_state=true;store_observation(s,t,state);return true;
}

static void fill_direct_result(KRPCApplyResult *result,const GuidanceCommand *command,const FlightControlOutput *fc){
    memset(result,0,sizeof(*result));result->applied=true;result->autopilot_engaged=false;result->gear=fc->gear;result->brakes=fc->brakes;result->airbrakes=fc->airbrakes;
    result->target_pitch=command->target_pitch;result->target_heading=command->target_heading;result->target_roll=command->target_roll;result->throttle=fc->throttle;result->wheel_steering=fc->wheel_steering;
    result->has_actuator_feedback=true;result->control_pitch=fc->pitch;result->control_roll=fc->roll;result->control_yaw=fc->yaw;result->has_control_diagnostics=true;
    result->control_body_pitch_rate_available=fc->diagnostics.body_pitch_rate_available;result->control_body_roll_rate_available=fc->diagnostics.body_roll_rate_available;result->control_body_yaw_rate_available=fc->diagnostics.body_yaw_rate_available;result->control_target_aoa=fc->diagnostics.target_pitch_state;result->control_measured_aoa=fc->diagnostics.measured_pitch_state;
    result->control_aoa_rate=fc->diagnostics.effective_pitch_rate;result->control_roll_rate=fc->diagnostics.effective_roll_rate;result->control_body_pitch_rate=fc->diagnostics.body_pitch_rate;result->control_body_roll_rate=fc->diagnostics.body_roll_rate;result->control_body_yaw_rate=fc->diagnostics.body_yaw_rate;result->control_pitch_error=fc->diagnostics.pitch_error;result->control_pitch_trim=fc->diagnostics.pitch_trim;
    result->control_pitch_authority=fc->diagnostics.pitch_authority;result->control_pitch_aero_fraction=fc->diagnostics.pitch_aero_fraction;result->control_roll_authority=fc->diagnostics.roll_authority;result->control_roll_raw_authority=fc->diagnostics.roll_raw_authority;result->control_roll_aero_fraction=fc->diagnostics.roll_aero_fraction;
    result->control_target_roll_rate=fc->diagnostics.target_roll_rate;result->control_beta_yaw_gain=fc->diagnostics.beta_yaw_gain;result->control_beta_roll_coupling=fc->diagnostics.beta_roll_coupling;result->control_beta_confidence=fc->diagnostics.beta_confidence;result->control_yaw_command=fc->yaw;
    snprintf(result->control_revision,sizeof(result->control_revision),"%s",flight_control_revision());snprintf(result->reference_frame,sizeof(result->reference_frame),"native-direct");snprintf(result->speed_mode,sizeof(result->speed_mode),"%s",speed_mode_string(command->navball_speed_mode));snprintf(result->control_profile,sizeof(result->control_profile),"%s",profile_string(command->control_profile));
}

static void fill_autopilot_result(KRPCApplyResult *result,const GuidanceCommand *command){
    memset(result,0,sizeof(*result));result->applied=true;result->autopilot_engaged=command->autopilot_engaged;result->gear=command->gear;result->brakes=command->brakes;result->airbrakes=command->airbrakes;result->target_pitch=command->target_pitch;result->target_heading=command->target_heading;result->target_roll=command->target_roll;result->throttle=command->target_throttle;result->wheel_steering=command->wheel_steering;
    snprintf(result->control_revision,sizeof(result->control_revision),"cnano-autopilot");snprintf(result->reference_frame,sizeof(result->reference_frame),"%s",command->use_inertial_direction?"body-non-rotating":"surface");snprintf(result->speed_mode,sizeof(result->speed_mode),"%s",speed_mode_string(command->navball_speed_mode));snprintf(result->control_profile,sizeof(result->control_profile),"%s",profile_string(command->control_profile));
}

static void quat_rotation_matrix(const double qv[4],double r[3][3]){
    double x=qv[0],y=qv[1],z=qv[2],w=qv[3];double n=sqrt(x*x+y*y+z*z+w*w);if(n<=1e-12){x=y=z=0;w=1;}else{x/=n;y/=n;z/=n;w/=n;}
    r[0][0]=1-2*(y*y+z*z);r[0][1]=2*(x*y-z*w);r[0][2]=2*(x*z+y*w);
    r[1][0]=2*(x*y+z*w);r[1][1]=1-2*(x*x+z*z);r[1][2]=2*(y*z-x*w);
    r[2][0]=2*(x*z-y*w);r[2][1]=2*(y*z+x*w);r[2][2]=1-2*(x*x+y*y);
}

static void matrix_quaternion(const double m[3][3],double*qx,double*qy,double*qz,double*qw){
    double x=0,y=0,z=0,w=1,tr=m[0][0]+m[1][1]+m[2][2],scale;
    if(tr>0){scale=sqrt(fmax(1e-12,tr+1))*2;w=.25*scale;x=(m[2][1]-m[1][2])/scale;y=(m[0][2]-m[2][0])/scale;z=(m[1][0]-m[0][1])/scale;}
    else if(m[0][0]>m[1][1]&&m[0][0]>m[2][2]){scale=sqrt(fmax(1e-12,1+m[0][0]-m[1][1]-m[2][2]))*2;w=(m[2][1]-m[1][2])/scale;x=.25*scale;y=(m[0][1]+m[1][0])/scale;z=(m[0][2]+m[2][0])/scale;}
    else if(m[1][1]>m[2][2]){scale=sqrt(fmax(1e-12,1+m[1][1]-m[0][0]-m[2][2]))*2;w=(m[0][2]-m[2][0])/scale;x=(m[0][1]+m[1][0])/scale;y=.25*scale;z=(m[1][2]+m[2][1])/scale;}
    else{scale=sqrt(fmax(1e-12,1+m[2][2]-m[0][0]-m[1][1]))*2;w=(m[1][0]-m[0][1])/scale;x=(m[0][2]+m[2][0])/scale;y=(m[1][2]+m[2][1])/scale;z=.25*scale;}
    double n=sqrt(x*x+y*y+z*z+w*w);if(n>1e-12){x/=n;y/=n;z/=n;w/=n;}if(w<0){x=-x;y=-y;z=-z;w=-w;}*qx=x;*qy=y;*qz=z;*qw=w;
}

static bool inertial_entry_capture_step(KRPCSession*s,const GuidanceCommand*command,FlightControlOutput*out){
    if(!s||!command||!out||!s->has_last_telemetry||!s->has_last_state||!s->last_telemetry.has_attitude_quaternion)return false;
    Vector3 forward=vnorm(canonical_to_raw(command->inertial_direction),v3(0,1,0));
    Vector3 radial=vnorm(canonical_to_raw(s->last_state.position),v3(1,0,0));
    Vector3 up=vnorm(vproject_plane(radial,forward),v3(1,0,0));
    Vector3 right=vnorm(vcross(up,forward),v3(1,0,0));
    Vector3 down=vnorm(vcross(right,forward),v3(0,0,1));
    double current[3][3],desired[3][3],rel[3][3];quat_rotation_matrix(s->last_telemetry.attitude_quaternion,current);
    desired[0][0]=right.x;desired[1][0]=right.y;desired[2][0]=right.z;
    desired[0][1]=forward.x;desired[1][1]=forward.y;desired[2][1]=forward.z;
    desired[0][2]=down.x;desired[1][2]=down.y;desired[2][2]=down.z;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){rel[i][j]=0;for(int k=0;k<3;k++)rel[i][j]+=current[k][i]*desired[k][j];}
    double x,y,z,w;matrix_quaternion(rel,&x,&y,&z,&w);double vn=sqrt(x*x+y*y+z*z);double angle=vn>1e-12?2*atan2(vn,fmax(0,w)):0;
    double pitch_error=0,roll_error=0,yaw_error=0;if(vn>1e-12){double sangle=angle*RAD2DEG/vn;pitch_error=-x*sangle;roll_error=-y*sangle;yaw_error=-z*sangle;}
    const Telemetry*t=&s->last_telemetry;double pitch_rate=t->has_body_pitch_rate?t->body_pitch_rate:0,roll_rate=t->has_body_roll_rate?t->body_roll_rate:0,yaw_rate=t->has_body_yaw_rate?t->body_yaw_rate:0;
    double wanted_pitch=clampd(pitch_error/7.0,-5.0,5.0),wanted_roll=clampd(roll_error/7.0,-4.0,4.0),wanted_yaw=clampd(yaw_error/7.0,-4.0,4.0);
    memset(out,0,sizeof(*out));out->pitch=clampd((wanted_pitch-pitch_rate)*.18,-.45,.45);out->roll=clampd((wanted_roll-roll_rate)*.16,-.35,.35);out->yaw=clampd((wanted_yaw-yaw_rate)*.16,-.35,.35);out->throttle=command->target_throttle;out->wheel_steering=command->wheel_steering;out->gear=command->gear;out->brakes=command->brakes;out->airbrakes=command->airbrakes;out->rcs_assist=1;out->rcs_requested=true;out->valid=true;
    out->diagnostics.body_pitch_rate_available=t->has_body_pitch_rate;out->diagnostics.effective_pitch_rate=pitch_rate;out->diagnostics.pitch_error=pitch_error;out->diagnostics.effective_roll_rate=roll_rate;out->diagnostics.target_roll_rate=wanted_roll;
    return true;
}

bool krpc_orbital_up_reference(const GuidanceCommand *command,const VehicleState *state,Vector3 *out){
    if(!command||!state||!out||!command->use_inertial_direction||vmag(state->position)<=1.0||vmag(command->inertial_direction)<=1e-9)return false;
    Vector3 forward=vnorm(command->inertial_direction,v3(0,1,0));
    Vector3 radial=vnorm(state->position,v3(1,0,0));
    Vector3 projected=vproject_plane(radial,forward);
    if(vmag(projected)<=1e-8){
        Vector3 seed=v3(1,0,0);double best=fabs(vdot(seed,forward));
        Vector3 y=v3(0,1,0),z=v3(0,0,1);double dy=fabs(vdot(y,forward)),dz=fabs(vdot(z,forward));
        if(dy<best){seed=y;best=dy;}if(dz<best)seed=z;
        projected=vproject_plane(seed,forward);
    }
    if(vmag(projected)<=1e-8)return false;
    *out=vnorm(projected,v3(1,0,0));
    return true;
}

bool krpc_apply(KRPCSession *s,const GuidanceCommand *command,unsigned airbrake_group,const char *phase,const char *status,const char *warning,const Trajectory *hud_predicted_trajectory,const Trajectory *hud_reference_trajectory,KRPCApplyResult *result,char *error,size_t error_size){
    (void)phase;(void)status;(void)warning;(void)hud_predicted_trajectory;(void)hud_reference_trajectory;
    if(!s||!command){set_error(error,error_size,"Flight-control apply has invalid state");return false;}
    if(s->simulator){
        if(getenv("KSP_LANDER_APPLY_DIAG"))
            fprintf(stderr,"KSP_APPLY phase=%s roll=%.3f aoa=%.3f heading=%.3f status=%s warning=%s\n",
                phase?phase:"",command->target_roll,
                command->has_target_aoa?command->target_aoa:command->target_pitch,
                command->target_heading,status?status:"",warning?warning:"");
        if(!sim_send_guidance(s,command,true,error,error_size))return false;
        if(result){memset(result,0,sizeof(*result));result->applied=true;result->autopilot_engaged=true;result->gear=command->gear;result->brakes=command->brakes;result->airbrakes=false;
            result->target_pitch=command->target_pitch;result->target_heading=command->target_heading;result->target_roll=command->target_roll;
            result->throttle=command->control_profile==PROFILE_ORBITAL?
                clampd(command->target_throttle,0.0,1.0):0.0;
            snprintf(result->reference_frame,sizeof(result->reference_frame),"shuttlesim-high-level");snprintf(result->speed_mode,sizeof(result->speed_mode),"%s",speed_mode_string(command->navball_speed_mode));snprintf(result->control_profile,sizeof(result->control_profile),"%s",profile_string(command->control_profile));}
        snprintf(s->last_control_profile,sizeof(s->last_control_profile),"%s",profile_string(command->control_profile));return true;
    }
    if(!s->client){set_error(error,error_size,"Native C-Nano apply has invalid state");return false;}
    bool atmospheric=command->control_profile!=PROFILE_ORBITAL;
    bool inertial_entry=atmospheric&&command->control_profile==PROFILE_ENTRY&&command->use_inertial_direction;
    if(atmospheric){
        if(!s->has_last_telemetry){set_error(error,error_size,"Native flight control has no telemetry sample");return false;}
        double dt=1.0/fmax(2.0,s->flight_control.nominal_dt>0?1.0/s->flight_control.nominal_dt:10.0);
        if(s->has_last_control_ut&&s->last_telemetry.ut>s->last_control_ut)dt=clampd(s->last_telemetry.ut-s->last_control_ut,.01,2.0);
        FlightControlOutput fc;bool valid=inertial_entry?inertial_entry_capture_step(s,command,&fc):flight_control_step(&s->flight_control,&s->last_telemetry,command,dt,&fc);
        if(!valid||!fc.valid){set_error(error,error_size,"Native %s flight-control law rejected profile %s",inertial_entry?"inertial-entry":"atmospheric",profile_string(command->control_profile));return false;}
        /* Unpowered landing forbids main-engine energy recovery, not the brief
           attitude-only RCS authority needed to rotate from the retrograde deorbit
           burn attitude into entry attitude before the aerosurfaces have useful q.
           Keep throttle hard-zero; flight_control_step()/inertial_entry_capture_step()
           blend RCS out with aerodynamic authority and enforce the transonic cutoff. */
        const char *unpowered=getenv("KSP_LANDER_UNPOWERED_ONLY");
        if(unpowered&&strcmp(unpowered,"1")==0)fc.throttle=0.0;
        if(!krpc_cnano_client_set_direct_controls(s->client,fc.pitch,fc.roll,fc.yaw,fc.throttle,fc.wheel_steering,fc.rcs_requested,error,error_size))return false;
        if(result)fill_direct_result(result,command,&fc);
        s->last_control_ut=s->last_telemetry.ut;s->has_last_control_ut=true;
    }else{
        Vector3 radial_up;
        const Vector3 *up_reference=NULL;
        if(command->use_inertial_direction&&s->has_last_state&&
           krpc_orbital_up_reference(command,&s->last_state,&radial_up)){
            /* Define the complete orbital attitude geometrically for both
               retrograde and prograde capture: nose follows the inertial target
               while roof stays as radial-out as the nose direction permits.
               kRPC's separate TargetDirection + TargetRoll path has a 180-degree
               roll singularity; SetDirectionAndUp avoids that ambiguity. */
            up_reference=&radial_up;
        }
        if(!krpc_cnano_client_set_autopilot(s->client,command,up_reference,error,error_size))return false;
        if(result)fill_autopilot_result(result,command);
    }
    if(!krpc_cnano_client_set_gear(s->client,command->gear,error,error_size)||
       !krpc_cnano_client_set_brakes(s->client,command->brakes,error,error_size)||
       !krpc_cnano_client_set_airbrakes(s->client,airbrake_group,command->airbrakes,error,error_size)||
       !krpc_cnano_client_set_speed_mode(s->client,command->navball_speed_mode,error,error_size))return false;
    snprintf(s->last_control_profile,sizeof(s->last_control_profile),"%s",profile_string(command->control_profile));
    return true;
}

void krpc_safe(KRPCSession *s){if(!s)return;if(s->simulator){char ignored[128];(void)sim_send_guidance(s,NULL,false,ignored,sizeof(ignored));}else if(s->client)krpc_cnano_client_safe(s->client);flight_control_reset_transients(&s->flight_control);s->has_last_control_ut=false;}
bool krpc_set_gear(KRPCSession *s,bool value,char *error,size_t error_size){if(!s)return false;if(s->simulator){GuidanceCommand c;guidance_command_init(&c);c.gear=value;c.brakes=s->has_last_telemetry?s->last_telemetry.brakes:false;c.has_target_aoa=true;c.target_aoa=s->has_last_telemetry?s->last_telemetry.angle_of_attack:0;c.target_roll=s->has_last_telemetry?s->last_telemetry.roll:0;return sim_send_guidance(s,&c,false,error,error_size);}return s->client&&krpc_cnano_client_set_gear(s->client,value,error,error_size);}
bool krpc_set_brakes(KRPCSession *s,bool value,char *error,size_t error_size){if(!s)return false;if(s->simulator){GuidanceCommand c;guidance_command_init(&c);c.gear=s->has_last_telemetry?s->last_telemetry.gear:false;c.brakes=value;c.has_target_aoa=true;c.target_aoa=s->has_last_telemetry?s->last_telemetry.angle_of_attack:0;c.target_roll=s->has_last_telemetry?s->last_telemetry.roll:0;return sim_send_guidance(s,&c,false,error,error_size);}return s->client&&krpc_cnano_client_set_brakes(s->client,value,error,error_size);}
bool krpc_save_game(KRPCSession *s,const char *name,char *error,size_t error_size){if(s&&s->simulator){(void)name;(void)error;(void)error_size;return true;}return s&&s->client&&krpc_cnano_client_save(s->client,name,error,error_size);}
bool krpc_warp(KRPCSession *s,double ut,char *error,size_t error_size){if(s&&s->simulator){(void)ut;set_error(error,error_size,"Time warp is not used in ShuttleSim lockstep");return false;}bool ok=s&&s->client&&krpc_cnano_client_warp(s->client,ut,error,error_size);if(ok){flight_control_reset_transients(&s->flight_control);s->has_last_control_ut=false;}return ok;}

void krpc_session_close(KRPCSession *s){
    if(!s)return;
    if(s->physics_store){if(!s->simulator){char ignored[256];(void)physics_store_flush(s->physics_store,ignored,sizeof(ignored));}physics_store_close(s->physics_store);}
    if(s->sim_rx_fd>=0)close(s->sim_rx_fd);if(s->sim_tx_fd>=0)close(s->sim_tx_fd);
    if(s->client)krpc_cnano_client_close(s->client);free(s);
}
