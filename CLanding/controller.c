#include "landing.h"
#include "flight_log_codec.h"
#include "planner_log.h"
#include "async_prediction_policy.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

struct LandingController {
    pthread_mutex_t mutex;
    pthread_t thread;
    bool thread_started;
    bool stop_thread;
    pthread_t prediction_thread;
    bool prediction_thread_started;
    bool stop_prediction_thread;
    pthread_t terminal_preview_thread;
    bool terminal_preview_thread_started, stop_terminal_preview_thread;
    uint64_t prediction_generation;
    PredictorPlannerTrace entry_worker_trace;
    bool entry_worker_trace_valid;
    double last_terminal_prediction_request_ut, last_terminal_prediction_completion_ut;
    double last_terminal_prediction_completion_wall, last_terminal_prediction_solve_wall;
    LandingConfiguration configuration;
    bool runway_end_initialized;
    int runway_end_index; /* 0 = configured threshold/heading, 1 = reciprocal end */
    double runway_end_primary_score, runway_end_reciprocal_score;
    KRPCSession *session;
    GuidanceMachine guidance;
    VesselPhysicsModel physics;
    AdaptiveFlightCalibrator adaptive;
    GlideCalibrationMachine glide;
    AerodynamicModel aerodynamics;
    AerodynamicModel planning_aerodynamics;
    AerodynamicEnvelope envelope;
    InFlightTrajectoryCalibrator trajectory_calibrator;
    TrajectoryCalibrationModel trajectory_calibration;
    GuidanceCommand last_command;
    VehicleProfile adaptive_profile;
    DeorbitPlan plan;
    bool has_plan;
    bool plan_stale;
    LandingSnapshot snapshot;
    Trajectory dynamic_trajectory;
    Trajectory stabilized_trajectory;
    bool forecast_entry_plan_identity_valid;
    uint64_t forecast_entry_plan_id, forecast_entry_plan_version;
    bool prediction_lineage_discontinuity;
    Telemetry latest_telemetry;
    VehicleState latest_state;
    bool has_latest;
    double last_trajectory_ut;
    double last_prediction_ut;
    bool has_prediction_cache;
    double cached_physics_observed_seconds, cached_physics_fallback_seconds;
    double cached_predicted_miss_distance;
    double cached_predicted_taem_distance;
    double cached_predicted_taem_range_error;
    double cached_predicted_entry_range;
    double cached_predicted_entry_flight_path_angle;
    double cached_predicted_taem_speed;
    double cached_predicted_taem_energy_error;
    double cached_predicted_peak_dynamic_pressure;
    double cached_predicted_peak_g_load;
    double cached_predicted_s_turn_reversals;
    bool cached_predicted_shadow_guidance, cached_predicted_terminal_policy_feasible;
    double cached_predicted_terminal_survivability_stress_score, cached_predicted_physics_relative_uncertainty;
    unsigned cached_predicted_uncertainty_scenarios;
    double cached_predicted_raw_published_position_delta_30s;
    double cached_predicted_raw_published_position_delta_60s;
    double cached_predicted_raw_published_position_delta_120s;
    double cached_prediction_ut;
    double cached_prediction_sign;
    double last_tick_ut;
    bool has_last_tick_ut;
    double last_tick_wall;
    bool has_last_tick_wall;
    bool time_warp_issued;
    uint64_t tick_sequence, log_sequence, vehicle_record_sequence;
    SnapshotCallback callback;
    void *callback_context;
    FILE *flight_log;
    PlannerLog *planner_log;
    VehicleLogEncoder vehicle_log_encoder;
    char log_session_id[192];
    char log_native_build[96], log_planner_path[1700];
    bool log_actual_trajectory_dirty;
    bool log_reference_trajectory_dirty;
    bool log_plan_trajectory_dirty;
    bool log_raw_plan_trajectory_dirty;
};

static void start_prediction_worker(LandingController *c);
static void stop_prediction_worker(LandingController *c);

static double monotonic_seconds(void){struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);return (double)ts.tv_sec+(double)ts.tv_nsec/1e9;}
static void sleep_seconds(double s){if(s<=0)return;struct timespec ts={(time_t)s,(long)((s-floor(s))*1e9)};while(nanosleep(&ts,&ts)<0&&errno==EINTR){}}

static LandingSite controller_active_runway_site(const LandingController*c,double radius){
    LandingSite site=c->configuration.site;
    if(c->runway_end_initialized&&c->runway_end_index==1)
        site=runway_reciprocal_site(&site,radius);
    return site;
}

static void controller_select_runway_end(LandingController*c,LandingConfiguration*cfg,
        Telemetry*t,const PlanetModel*p){
    if(!c||!cfg||!t||!p)return;
    bool timeline_reset=!c->has_latest||
        (isfinite(c->latest_telemetry.ut)&&t->ut<c->latest_telemetry.ut-1.0);
    if(timeline_reset)c->runway_end_initialized=false;
    LandingSite primary=c->configuration.site;
    LandingSite reciprocal=runway_reciprocal_site(&primary,p->radius);
    if(!c->runway_end_initialized){
        c->runway_end_primary_score=runway_end_acquisition_score(t,&primary,&cfg->guidance,p->radius);
        c->runway_end_reciprocal_score=runway_end_acquisition_score(t,&reciprocal,&cfg->guidance,p->radius);
        c->runway_end_index=primary.allow_reciprocal_runway&&
            c->runway_end_reciprocal_score<c->runway_end_primary_score?1:0;
        c->runway_end_initialized=true;
        fprintf(stderr,"Runway end selected: heading %.1f deg, primary cost %.0f m, reciprocal cost %.0f m, reciprocal %s.\n",
            c->runway_end_index==1?reciprocal.runway_heading:primary.runway_heading,
            c->runway_end_primary_score,c->runway_end_reciprocal_score,
            primary.allow_reciprocal_runway?"allowed":"locked out");
    }
    cfg->site=c->runway_end_index==1?reciprocal:primary;
    telemetry_reframe_runway(t,&cfg->site,p->radius);
}
static LandingConfiguration effective_config(const LandingController*c){
    LandingConfiguration r=c->configuration;
    if(r.calibration.auto_apply_in_flight){
        /* Current-flight identification is advisory, like Shuttle flight-test
           reconstruction.  It may tighten operational speed margins, but it
           must not rewrite the certified aerodynamic data book while the same
           vehicle is depending on that model for entry guidance. */
        r.vehicle.minimum_safe_speed=c->adaptive_profile.minimum_safe_speed;
        r.vehicle.final_approach_speed=c->adaptive_profile.final_approach_speed;
        r.vehicle.touchdown_speed=c->adaptive_profile.touchdown_speed;
    }
    if(c->runway_end_initialized&&c->session){
        const PlanetModel*p=krpc_session_planet(c->session);
        if(p)r.site=controller_active_runway_site(c,p->radius);
    }
    landing_configuration_normalize(&r);return r;
}
static AerodynamicModel envelope_at_controller(const AerodynamicEnvelope*e,double mach){
    const double anchor[]={.35,1.05,2.6,6};double x=fmax(0,mach);
    if(x<=anchor[0])return e->regimes[0];if(x>=anchor[3])return e->regimes[3];
    for(int i=0;i<3;i++)if(x<=anchor[i+1]){double f=(x-anchor[i])/fmax(anchor[i+1]-anchor[i],1e-9);return (AerodynamicModel){e->regimes[i].lift_to_drag+(e->regimes[i+1].lift_to_drag-e->regimes[i].lift_to_drag)*f,e->regimes[i].ballistic_coefficient+(e->regimes[i+1].ballistic_coefficient-e->regimes[i].ballistic_coefficient)*f,e->regimes[i].confidence+(e->regimes[i+1].confidence-e->regimes[i].confidence)*f};}
    return e->regimes[3];
}
static void apply_certified_aero_prior(LandingController*c){
    AerodynamicModel planning;
    vessel_physics_derive_envelope(&c->physics,&c->configuration.vehicle,&c->envelope,&planning);
    c->planning_aerodynamics=planning;
    c->aerodynamics=c->has_latest?envelope_at_controller(&c->envelope,c->latest_telemetry.mach):planning;
}
static bool vehicle_profile_equal(const VehicleProfile*a,const VehicleProfile*b){return
    !strcmp(a->model_id,b->model_id)&&
    a->estimated_lift_to_drag==b->estimated_lift_to_drag&&
    a->estimated_ballistic_coefficient==b->estimated_ballistic_coefficient&&
    a->entry_angle_of_attack==b->entry_angle_of_attack&&
    a->maximum_angle_of_attack==b->maximum_angle_of_attack&&
    a->maximum_bank_angle==b->maximum_bank_angle&&
    a->maximum_dynamic_pressure==b->maximum_dynamic_pressure&&
    a->maximum_g_load==b->maximum_g_load&&
    a->minimum_safe_speed==b->minimum_safe_speed&&
    a->final_approach_speed==b->final_approach_speed&&
    a->touchdown_speed==b->touchdown_speed&&
    a->allow_powered_approach==b->allow_powered_approach&&
    a->maximum_approach_throttle==b->maximum_approach_throttle&&
    a->airbrake_action_group==b->airbrake_action_group;
}
static bool configuration_equal(const LandingConfiguration*a,const LandingConfiguration*b){JsonWriter wa,wb;jw_init(&wa);jw_init(&wb);landing_configuration_json(&wa,a);landing_configuration_json(&wb,b);bool same=!wa.failed&&!wb.failed&&wa.data&&wb.data&&!strcmp(wa.data,wb.data);jw_free(&wa);jw_free(&wb);return same;}
static bool planning_configuration_equal(const LandingConfiguration*a,const LandingConfiguration*b){LandingConfiguration aa=*a,bb=*b;memset(&aa.connection,0,sizeof(aa.connection));memset(&bb.connection,0,sizeof(bb.connection));memset(&aa.calibration,0,sizeof(aa.calibration));memset(&bb.calibration,0,sizeof(bb.calibration));return configuration_equal(&aa,&bb);}
static EntryControlPlan guidance_control_plan_snapshot(const GuidanceMachine*g,double ut,double remaining){
    EntryControlPlan p;memset(&p,0,sizeof(p));
    if(!g||!g->entry_control_plan_valid)return p;
    p.valid=true;p.terminal_ready=g->entry_control_terminal_ready;p.planned_ut=ut;
    p.target_bank=g->entry_control_bank;p.target_aoa=g->entry_control_aoa;p.target_heading=g->entry_control_heading;
    p.bank_cap=fabs(g->entry_control_bank);p.target_turn_radius=g->entry_control_turn_radius;
    p.segment_duration=fmax(0,remaining);p.cost=g->entry_control_cost;
    p.taem_range_error=g->entry_control_taem_range_error;p.taem_speed=g->entry_control_taem_speed;p.taem_energy_error=g->entry_control_taem_energy_error;
    p.has_planned_reversal=g->entry_reversal_scheduled;
    p.planned_reversal_is_final=g->entry_reversal_is_final;
    p.final_heading_lock=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    p.planned_reversal_ut=g->entry_reversal_ut;
    p.planned_reversal_range=g->entry_reversal_range;
    p.planned_reversal_sign=g->entry_reversal_sign;
    p.predicted_reversals=g->entry_control_reversals;
    if(g->entry_s_turn_plan.valid){
        p.plan_id=g->entry_s_turn_plan.plan_id;p.plan_version=g->entry_s_turn_plan.plan_version;
        p.parent_plan_id=g->entry_s_turn_plan.parent_plan_id;p.parent_plan_version=g->entry_s_turn_plan.parent_plan_version;
    }
    return p;
}

static bool entry_forecast_matches_guidance(const LandingController*c){
    if(!c)return true;
    /* A forecast tagged to an MM304 executable plan becomes stale not only when that
       plan id/version changes, but also when guidance leaves MM304 in the same tick. */
    if(c->forecast_entry_plan_identity_valid){
        if(c->guidance.phase!=PHASE_ENTRY_ENERGY)return false;
        return c->guidance.entry_control_plan_valid&&c->guidance.entry_s_turn_plan.valid&&
            c->forecast_entry_plan_id==c->guidance.entry_s_turn_plan.plan_id&&
            c->forecast_entry_plan_version==c->guidance.entry_s_turn_plan.plan_version;
    }
    if(c->guidance.phase!=PHASE_ENTRY_ENERGY)return true;
    return !(c->guidance.entry_control_plan_valid&&c->guidance.entry_s_turn_plan.valid);
}
/* All read paths publish an indivisible telemetry/state pair, including events. */
static bool read_live_sample(LandingController *c,LandingConfiguration *cfg,Telemetry *t,VehicleState *state,char *error,size_t error_size){
    if(!krpc_read_telemetry(c->session,cfg,t,state,error,error_size))return false;
    const PlanetModel*p=krpc_session_planet(c->session);
    controller_select_runway_end(c,cfg,t,p);
    vessel_physics_observe(&c->physics,t,state,p);
    c->trajectory_calibration.physics=&c->physics;
    c->latest_telemetry=*t;c->latest_state=*state;c->has_latest=true;
    landing_snapshot_set_sample(&c->snapshot,t,state);
    return true;
}
static void publish(LandingController*c){
    c->snapshot.plan=c->has_plan?&c->plan:NULL;
    if(!c->callback)return;
    /* Simulator lockstep can advance dozens/hundreds of guidance ticks per wall
       second. Keep controller execution unthrottled but cap expensive full JSON
       snapshot serialization for UI/log consumers. Phase transitions always pass. */
    static double sim_last_publish_wall=-INFINITY;
    static double sim_last_publish_ut=NAN;
    static GuidancePhase sim_last_phase=(GuidancePhase)-1;
    static ConnectionStatus sim_last_connection=(ConnectionStatus)-1;
    if(c->session&&krpc_session_is_simulator(c->session)){
        double hz=12.0;const char*e=getenv("KSP_LANDER_SIM_PUBLISH_HZ");
        if(e&&*e){double v=strtod(e,NULL);if(isfinite(v)&&v>=1&&v<=120)hz=v;}
        double now=monotonic_seconds();
        double current_ut=c->snapshot.telemetry.ut;
        bool first_live=isfinite(current_ut)&&current_ut>0.0&&(!isfinite(sim_last_publish_ut)||sim_last_publish_ut<=0.0);
        bool force=first_live||c->snapshot.phase!=sim_last_phase||c->snapshot.connection_status!=sim_last_connection;
        if(!force&&isfinite(sim_last_publish_wall)&&now-sim_last_publish_wall<1.0/hz)return;
        sim_last_publish_wall=now;sim_last_publish_ut=current_ut;
        sim_last_phase=c->snapshot.phase;sim_last_connection=c->snapshot.connection_status;
    }
    c->callback(&c->snapshot,c->callback_context);
}
static void set_warning(LandingController*c,const char*msg){c->snapshot.has_warning=msg&&*msg;if(c->snapshot.has_warning)snprintf(c->snapshot.warning_message,sizeof(c->snapshot.warning_message),"%s",msg);else c->snapshot.warning_message[0]=0;}
static void set_error(LandingController*c,const char*msg){c->snapshot.has_error=msg&&*msg;if(c->snapshot.has_error)snprintf(c->snapshot.last_error,sizeof(c->snapshot.last_error),"%s",msg);else c->snapshot.last_error[0]=0;}

static bool trajectory_equal(const Trajectory*a,const Trajectory*b){
    if(a->count!=b->count)return false;
    for(size_t i=0;i<a->count;i++){
        const TrajectoryPoint*x=&a->points[i],*y=&b->points[i];
        if(x->ut!=y->ut||x->latitude!=y->latitude||x->longitude!=y->longitude||x->altitude!=y->altitude||x->speed!=y->speed||x->phase!=y->phase||x->kind!=y->kind)return false;
    }
    return true;
}

static bool replace_reference_trajectory(LandingController*c,const Trajectory*next){
    const Trajectory*source=next;
    /* MM304 often has no phase-local reference yet even though reentry engagement
       already produced a real propagated continuation trajectory. Do not erase that
       PLAN preview on the first control tick: keep following the controller plan
       trajectory until guidance publishes a more specific reference. */
    if(c&&c->guidance.phase==PHASE_ENTRY_ENERGY&&
       (!source||source->count<2)&&c->has_plan&&c->plan.trajectory.count>=2)
        source=&c->plan.trajectory;
    if(!source)return false;
    if(trajectory_equal(&c->snapshot.reference_trajectory,source))return true;
    if(!trajectory_copy(&c->snapshot.reference_trajectory,source))return false;
    c->log_reference_trajectory_dirty=true;
    return true;
}

static bool replace_predicted_trajectory(LandingController*c,const Trajectory*next){
    /* A guidance plan may change after this tick's synchronous forecast. Never pair old
       geometry with the newly accepted plan/command. Suppress the path for this one
       transition interval; the next tick is forced to forecast the new executable plan. */
    if(c&&(c->prediction_lineage_discontinuity||!entry_forecast_matches_guidance(c))){
        trajectory_clear(&c->snapshot.predicted_trajectory);return true;
    }
    if(trajectory_equal(&c->snapshot.predicted_trajectory,next))return true;
    return trajectory_copy(&c->snapshot.predicted_trajectory,next);
}

static bool trajectory_sample_at(const Trajectory*t,double ut,TrajectoryPoint*out){
    if(!t||!out||!t->count)return false;
    if(ut<t->points[0].ut||ut>t->points[t->count-1].ut)return false;
    if(ut<=t->points[0].ut){*out=t->points[0];return true;}
    if(ut>=t->points[t->count-1].ut){*out=t->points[t->count-1];return true;}
    size_t lo=0,hi=t->count-1;
    while(hi-lo>1){size_t mid=lo+(hi-lo)/2;if(t->points[mid].ut>=ut)hi=mid;else lo=mid;}
    const TrajectoryPoint*a=&t->points[lo],*b=&t->points[hi];
    double f=clampd((ut-a->ut)/fmax(b->ut-a->ut,1e-9),0,1);
    *out=*a;out->ut=ut;out->latitude=a->latitude+(b->latitude-a->latitude)*f;out->longitude=norm_signed_deg(a->longitude+norm_signed_deg(b->longitude-a->longitude)*f);out->altitude=a->altitude+(b->altitude-a->altitude)*f;out->speed=a->speed+(b->speed-a->speed)*f;
    return true;
}

static double trajectory_position_delta_at(const Trajectory*a,const Trajectory*b,double ut,double radius){
    TrajectoryPoint pa,pb;if(!trajectory_sample_at(a,ut,&pa)||!trajectory_sample_at(b,ut,&pb))return NAN;
    GeoPoint ga={pa.latitude,pa.longitude,pa.altitude},gb={pb.latitude,pb.longitude,pb.altitude};
    double horizontal=great_circle_distance(ga,gb,radius);
    return hypot(horizontal,pa.altitude-pb.altitude);
}

static bool stabilize_forecast_trajectory(Trajectory*stable,const Trajectory*raw,double alpha){
    if(!stable||!raw)return false;
    if(!stable->count||alpha>=.999)return trajectory_copy(stable,raw);
    Trajectory next;trajectory_init(&next);alpha=clampd(alpha,0,1);
    for(size_t i=0;i<raw->count;i++){
        TrajectoryPoint point=raw->points[i],old;
        if(trajectory_sample_at(stable,point.ut,&old)){
            point.latitude=old.latitude+(point.latitude-old.latitude)*alpha;
            point.longitude=norm_signed_deg(old.longitude+norm_signed_deg(point.longitude-old.longitude)*alpha);
            /* Never low-pass the vertical forecast.  Altitude is a physical
               state constraint used to judge entry/TAEM descent, not a UI
               coordinate.  Blending it with the previous receding-horizon
               solution made an obsolete high trajectory linger for several
               replans, so the displayed predicted path appeared to creep
               upward even while each raw propagation was descending.  Keep
               lateral smoothing for map continuity, but publish the current
               predictor's altitude directly. */
            point.speed=old.speed+(point.speed-old.speed)*alpha;
        }
        if(!trajectory_append(&next,point)){trajectory_clear(&next);return false;}
    }
    trajectory_clear(stable);*stable=next;return true;
}

static void durable_flush_file(FILE*f){if(!f)return;fflush(f);int fd=fileno(f);if(fd>=0)(void)fsync(fd);}
static void durable_flush_log(LandingController*c){durable_flush_file(c->flight_log);}
static void close_log(LandingController*c){
    if(c->flight_log){durable_flush_log(c);fclose(c->flight_log);c->flight_log=NULL;}
    if(c->planner_log){planner_log_close(c->planner_log);c->planner_log=NULL;}
}
static bool write_log_line(LandingController*c,JsonWriter*w){
    if(!c->flight_log||!w||w->failed||!w->data)return false;
    return fwrite(w->data,1,w->length,c->flight_log)==w->length&&fputc('\n',c->flight_log)!=EOF;
}

static void append_log_metadata(LandingController*c,const char*started){
    if(!c->flight_log||!c->session)return;
    const PlanetModel*p=krpc_session_planet(c->session);JsonWriter w;jw_init(&w);
    const char*campaign_identity=getenv("KSP_LANDER_CAMPAIGN_IDENTITY");
    jw_raw(&w,"{\"schemaVersion\":4,\"sessionSchemaVersion\":1,\"recordType\":\"sessionStart\",\"stream\":\"vehicle\",\"startedUTC\":");jw_string(&w,started);
    jw_raw(&w,",\"sessionId\":");jw_string(&w,c->log_session_id);
    jw_raw(&w,",\"campaignIdentity\":");jw_string(&w,(campaign_identity&&*campaign_identity)?campaign_identity:"");
    jw_raw(&w,",\"peer\":{\"stream\":\"planner\",\"available\":");jw_bool(&w,c->planner_log!=NULL);jw_raw(&w,",\"path\":");if(c->planner_log)jw_string(&w,planner_log_current_path(c->planner_log));else if(c->log_planner_path[0])jw_string(&w,c->log_planner_path);else jw_null(&w);jw_char(&w,'}');
    jw_raw(&w,",\"vessel\":");jw_string(&w,krpc_session_vessel(c->session));
    jw_raw(&w,",\"planet\":");planet_model_replay_json(&w,p);
    jw_raw(&w,",\"runtime\":{\"transport\":");jw_string(&w,krpc_session_transport(c->session));jw_raw(&w,",\"clientLibrary\":");jw_string(&w,krpc_session_library_version(c->session));jw_raw(&w,",\"nativeBuild\":");jw_string(&w,c->log_native_build);jw_char(&w,'}');
    jw_raw(&w,",\"modelIdentity\":{\"vehicleModelId\":");jw_string(&w,c->configuration.vehicle.model_id);jw_char(&w,'}');
    jw_raw(&w,",\"physicsContext\":{\"structureId\":");jw_string(&w,krpc_session_physics_structure_id(c->session));jw_raw(&w,",\"environmentId\":");jw_string(&w,krpc_session_physics_environment_id(c->session));jw_raw(&w,",\"storage\":");jw_string(&w,krpc_session_physics_storage(c->session));jw_raw(&w,",\"historicalSamplesLoaded\":");jw_integer(&w,krpc_session_physics_history_count(c->session));jw_char(&w,'}');
    jw_raw(&w,",\"encoding\":{\"policy\":\"adaptive delta with semantic-event forcing\",\"periodicKeyframeSeconds\":5,\"nominalHz\":10,\"thinAtmosphereHz\":5,\"rapidTransientHz\":20}");
    jw_raw(&w,",\"configuration\":");landing_configuration_json(&w,&c->configuration);jw_char(&w,'}');
    if(write_log_line(c,&w))durable_flush_log(c);jw_free(&w);
}

static const EntryControlPlan*current_entry_control_plan(const LandingController*c);
static VehicleLogSample vehicle_log_sample(const LandingController*c,const char*event){
    VehicleLogSample s={0};const LandingSnapshot*q=&c->snapshot;const Telemetry*t=&q->telemetry;const PlanetModel*p=c->session?krpc_session_planet(c->session):NULL;
    s.tick_sequence=q->tick_sequence;s.wall_monotonic_seconds=q->wall_monotonic_seconds;s.ut=t->ut;
    s.latitude=t->latitude;s.longitude=t->longitude;s.altitude=t->mean_altitude;s.radar_altitude=t->radar_altitude;
    if(q->has_vehicle_state){s.position[0]=q->vehicle_state.position.x;s.position[1]=q->vehicle_state.position.y;s.position[2]=q->vehicle_state.position.z;s.velocity[0]=q->vehicle_state.velocity.x;s.velocity[1]=q->vehicle_state.velocity.y;s.velocity[2]=q->vehicle_state.velocity.z;}
    s.true_air_speed=t->true_air_speed;s.horizontal_speed=t->horizontal_speed;s.vertical_speed=t->vertical_speed;s.flight_path_angle=t->flight_path_angle;
    s.pitch=t->pitch;s.roll=t->roll;s.heading=t->heading;s.angle_of_attack=t->angle_of_attack;s.sideslip=t->sideslip;
    s.pitch_rate=t->has_body_pitch_rate?t->body_pitch_rate:t->pitch_rate;s.roll_rate=t->has_body_roll_rate?t->body_roll_rate:t->roll_rate;s.yaw_rate=t->has_body_yaw_rate?t->body_yaw_rate:t->heading_rate;s.course_rate=t->has_course_rate?t->course_rate:0;
    s.coordinate_pitch_rate=t->pitch_rate;s.coordinate_roll_rate=t->roll_rate;s.heading_rate=t->heading_rate;
    s.angle_of_attack_rate=t->has_angle_of_attack_rate?t->angle_of_attack_rate:NAN;
    s.body_pitch_rate=t->has_body_pitch_rate?t->body_pitch_rate:NAN;s.body_roll_rate=t->has_body_roll_rate?t->body_roll_rate:NAN;s.body_yaw_rate=t->has_body_yaw_rate?t->body_yaw_rate:NAN;
    s.dynamic_pressure=t->dynamic_pressure;s.static_pressure=t->static_pressure;s.atmospheric_density=t->atmospheric_density;s.mach=t->mach;s.stall_fraction=t->stall_fraction;s.stall_fraction_is_measured=t->stall_fraction_is_measured;s.lift_force=t->lift_force;s.drag_force=t->drag_force;s.g_force=t->g_force;s.energy_excess_range=t->energy_excess_range;
    s.has_force_vectors=t->has_force_vectors;if(t->has_force_vectors){s.lift_vector[0]=t->lift_vector.x;s.lift_vector[1]=t->lift_vector.y;s.lift_vector[2]=t->lift_vector.z;s.drag_vector[0]=t->drag_vector.x;s.drag_vector[1]=t->drag_vector.y;s.drag_vector[2]=t->drag_vector.z;if(t->mass>1){s.aero_acceleration[0]=(t->lift_vector.x+t->drag_vector.x)/t->mass;s.aero_acceleration[1]=(t->lift_vector.y+t->drag_vector.y)/t->mass;s.aero_acceleration[2]=(t->lift_vector.z+t->drag_vector.z)/t->mass;}}
    s.physics_confidence=t->physics_confidence;s.physics_model_residual=t->physics_model_residual;s.physics_model_residual_confidence=t->physics_model_residual_confidence;s.physics_certified_uncertainty=t->physics_certified_uncertainty;
    s.physics_force_residual_per_q[0]=t->physics_force_residual_per_q.x;s.physics_force_residual_per_q[1]=t->physics_force_residual_per_q.y;s.physics_force_residual_per_q[2]=t->physics_force_residual_per_q.z;
    s.physics_force_residual_sigma_per_q[0]=t->physics_force_residual_sigma_per_q.x;s.physics_force_residual_sigma_per_q[1]=t->physics_force_residual_sigma_per_q.y;s.physics_force_residual_sigma_per_q[2]=t->physics_force_residual_sigma_per_q.z;
    s.physics_force_residual_confidence=t->physics_force_residual_confidence;s.physics_airbrake_model_available=t->physics_airbrake_model_available;s.physics_airbrake_model_confidence=t->physics_airbrake_model_confidence;s.physics_airbrake_drag_accel=t->physics_airbrake_drag_accel;s.physics_samples=t->physics_samples;s.physics_live_samples=t->physics_live_samples;
    s.mass=t->mass;s.available_thrust=t->available_thrust;s.current_thrust=t->current_thrust;s.has_center_of_mass=t->has_center_of_mass;if(t->has_center_of_mass){s.center_of_mass[0]=t->center_of_mass.x;s.center_of_mass[1]=t->center_of_mass.y;s.center_of_mass[2]=t->center_of_mass.z;}
    s.target_pitch=q->command.target_pitch;s.target_heading=q->command.target_heading;s.target_roll=q->command.target_roll;s.target_throttle=q->command.target_throttle;s.has_target_aoa=q->command.has_target_aoa;s.target_aoa=q->command.target_aoa;s.target_gear=q->command.gear;s.target_brakes=q->command.brakes;s.target_airbrakes=q->command.airbrakes;
    s.gear=t->gear;s.brakes=t->brakes;s.has_airbrakes=t->has_airbrakes;s.airbrakes=t->airbrakes;
    s.has_control_state=t->has_controls;s.control_state_pitch=t->control_pitch;s.control_state_roll=t->control_roll;s.control_state_yaw=t->control_yaw;s.command_applied=q->command_applied;s.has_actuator_feedback=q->command_apply_attempted&&q->applied_command.has_control_diagnostics;s.control_pitch=q->applied_command.control_pitch;s.control_roll=q->applied_command.control_roll;s.control_yaw=q->applied_command.control_yaw;
    s.has_loop_wall_delta=t->has_loop_wall_delta;s.has_telemetry_latency=t->has_telemetry_latency;s.has_guidance_compute=t->has_guidance_compute;s.has_apply_latency=t->has_apply_latency;s.has_control_loop=t->has_control_loop;
    s.loop_wall_delta_ms=t->loop_wall_delta_ms;s.telemetry_latency_ms=t->telemetry_latency_ms;s.guidance_compute_ms=t->guidance_compute_ms;s.apply_latency_ms=t->apply_latency_ms;s.control_loop_ms=t->control_loop_ms;
    s.automation_engaged=q->automation_engaged;s.paused=q->paused;s.constraint_active=t->dynamic_pressure>=c->configuration.vehicle.maximum_dynamic_pressure||t->g_force>=c->configuration.vehicle.maximum_g_load||t->stall_fraction>=1.0;snprintf(s.phase,sizeof(s.phase),"%s",phase_string(q->phase));snprintf(s.control_profile,sizeof(s.control_profile),"%s",profile_string(q->command.control_profile));snprintf(s.vessel_situation,sizeof(s.vessel_situation),"%s",t->vessel_situation);
    s.range_to_site=t->range_to_site;s.bearing_to_site=t->bearing_to_site;s.runway_along_track=t->runway_along_track;s.runway_cross_track=t->runway_cross_track;s.specific_mechanical_energy=p?rotating_specific_energy(t->latitude,t->mean_altitude,t->true_air_speed,p):NAN;s.entry_plan_valid=q->guidance_entry_plan_valid;s.entry_reversal_scheduled=q->guidance_entry_reversal_scheduled;s.entry_reversal_time_remaining=q->guidance_entry_reversal_time_remaining;s.entry_reversal_ut=q->guidance_entry_reversal_scheduled?t->ut+q->guidance_entry_reversal_time_remaining:NAN;s.entry_reversal_range=q->guidance_entry_reversal_range;s.entry_reversal_sign=q->guidance_entry_reversal_sign;
    const EntryControlPlan*plan=current_entry_control_plan(c);s.has_plan_identity=plan&&plan->plan_id>0&&plan->plan_version>0;if(s.has_plan_identity){s.plan_id=plan->plan_id;s.plan_version=plan->plan_version;s.parent_plan_id=plan->parent_plan_id;s.parent_plan_version=plan->parent_plan_version;}
    if(event&&*event)snprintf(s.event,sizeof(s.event),"%s",event);return s;
}

static bool append_vehicle_log(LandingController*c,const char*event,bool force_event,bool force_keyframe){
    if(!c->flight_log)return false;VehicleLogSample s=vehicle_log_sample(c,event);VehicleLogDecision d=vehicle_log_decide(&c->vehicle_log_encoder,&s,force_event,force_keyframe);if(!d.emit)return true;
    JsonWriter w;jw_init(&w);vehicle_log_record_json(&w,c->log_session_id,c->vehicle_record_sequence+1,&s,&d);bool ok=write_log_line(c,&w);jw_free(&w);
    if(ok){c->vehicle_record_sequence++;vehicle_log_commit(&c->vehicle_log_encoder,&s,&d);if(d.durable)durable_flush_log(c);else fflush(c->flight_log);}return ok;
}

static const EntryControlPlan*current_entry_control_plan(const LandingController*c){
    if(c->guidance.taem_s_turn_plan.valid)return &c->guidance.taem_s_turn_plan;
    if(c->guidance.entry_s_turn_plan.valid)return &c->guidance.entry_s_turn_plan;
    return NULL;
}

static void append_planner_log(LandingController*c,const char*event,bool explicit_force){
    if(!c->planner_log||!c->snapshot.has_vehicle_state)return;PredictorPlannerTrace trace;bool have_trace=c->entry_worker_trace_valid&&c->guidance.phase==PHASE_ENTRY_ENERGY;
    if(have_trace)trace=c->entry_worker_trace;else have_trace=predictor_last_planner_trace(&trace);const Telemetry*t=&c->snapshot.telemetry;
    bool constraint_active=t->dynamic_pressure>=c->configuration.vehicle.maximum_dynamic_pressure||t->g_force>=c->configuration.vehicle.maximum_g_load||t->stall_fraction>=1.0;
    bool prediction_matches_plan=!c->prediction_lineage_discontinuity&&entry_forecast_matches_guidance(c);
    PlannerLogSample s={.tick_sequence=c->snapshot.tick_sequence,.ut=t->ut,.phase=c->snapshot.phase,
        .telemetry=t,.snapshot=&c->snapshot,.plan=current_entry_control_plan(c),.trace=have_trace?&trace:NULL,
        .raw_prediction=prediction_matches_plan?&c->dynamic_trajectory:NULL,
        .published_prediction=prediction_matches_plan?&c->stabilized_trajectory:NULL,
        .calibration=&c->trajectory_calibration,.envelope=&c->envelope,.constraint_active=constraint_active,
        .predictor_discontinuity=!prediction_matches_plan,.explicit_force=explicit_force,.event_reason=event};
    (void)planner_log_record(c->planner_log,&s);
}

static void start_log(LandingController*c){
    close_log(c);c->entry_worker_trace_valid=false;c->tick_sequence=0;c->log_sequence=0;c->vehicle_record_sequence=0;c->snapshot.tick_sequence=0;c->snapshot.log_sequence=0;c->log_native_build[0]=0;c->log_planner_path[0]=0;vehicle_log_encoder_init(&c->vehicle_log_encoder);c->log_actual_trajectory_dirty=false;c->log_reference_trajectory_dirty=false;c->log_plan_trajectory_dirty=false;c->log_raw_plan_trajectory_dirty=false;
    char root[1200]={0},dir[1300],path[1500];const char*configured=getenv("KSP_LANDER_ROOT");if(configured&&*configured)snprintf(root,sizeof(root),"%s",configured);else if(!getcwd(root,sizeof(root)))return;if(access("Configuration/default.json",R_OK)!=0&&access("../Configuration/default.json",R_OK)==0){char parent[1200];if(realpath("..",parent))snprintf(root,sizeof(root),"%s",parent);}snprintf(dir,sizeof(dir),"%s/FlightLogs",root);mkdir(dir,0700);
    time_t now=time(NULL);struct tm tm;gmtime_r(&now,&tm);char stamp[64];strftime(stamp,sizeof(stamp),"%Y-%m-%dT%H-%M-%SZ",&tm);char vessel[128];snprintf(vessel,sizeof(vessel),"%s",krpc_session_vessel(c->session));for(char*p=vessel;*p;p++)if(*p=='/'||*p=='\\'||*p==':'||*p==' ') *p='-';const char*safe_vessel=vessel[0]?vessel:"vessel";
    unsigned long long nonce=(unsigned long long)(monotonic_seconds()*1000000.0);snprintf(c->log_session_id,sizeof(c->log_session_id),"%s-%ld-%llu",stamp,(long)getpid(),nonce);snprintf(c->log_native_build,sizeof(c->log_native_build),"%s %s",__DATE__,__TIME__);
    snprintf(path,sizeof(path),"%s/%s-%s-vehicle.jsonl",dir,stamp,safe_vessel);c->flight_log=fopen(path,"w");
    snprintf(c->log_planner_path,sizeof(c->log_planner_path),"%s/%s-%s-planner.jsonl",dir,stamp,safe_vessel);
    c->planner_log=planner_log_open(dir,stamp,safe_vessel,c->log_session_id,c->log_native_build,path,c->flight_log!=NULL,&c->configuration,0);
    if(c->flight_log)append_log_metadata(c,stamp);
    if(!c->flight_log&&!c->planner_log)set_warning(c,"Both forensic log streams failed to open. Guidance can continue, but this session cannot count as acceptance evidence.");
    else if(!c->flight_log)set_warning(c,"Vehicle forensic stream failed to open. Planner logging may continue, but this session cannot count as acceptance evidence.");
    else if(!c->planner_log)set_warning(c,"Planner forensic stream failed to open. Vehicle logging may continue, but this session cannot count as acceptance evidence.");
}

static void append_log(LandingController*c){
    (void)append_vehicle_log(c,NULL,false,false);append_planner_log(c,NULL,false);c->log_actual_trajectory_dirty=false;c->log_reference_trajectory_dirty=false;c->log_plan_trajectory_dirty=false;c->log_raw_plan_trajectory_dirty=false;
}

static void append_configuration_log(LandingController*c){
    if(!c->flight_log&&!c->planner_log)return;c->snapshot.wall_monotonic_seconds=monotonic_seconds();(void)append_vehicle_log(c,"event=configuration-changed",true,false);append_planner_log(c,"event=configuration-changed",true);
}
static void log_state_event(LandingController*c,const char*reason,bool durable){
    if(!c->flight_log&&!c->planner_log)return;c->snapshot.log_sequence=++c->log_sequence;c->snapshot.wall_monotonic_seconds=monotonic_seconds();snprintf(c->snapshot.decision_reason,sizeof(c->snapshot.decision_reason),"%s",reason?reason:"event=state-change");(void)append_vehicle_log(c,reason?reason:"event=state-change",true,false);append_planner_log(c,reason?reason:"event=state-change",true);if(durable)durable_flush_log(c);
}

static void clear_snapshot_dynamic(LandingSnapshot*s){trajectory_clear(&s->actual_trajectory);trajectory_clear(&s->reference_trajectory);trajectory_clear(&s->predicted_trajectory);}
static void reset_snapshot(LandingController*c,const char*status){clear_snapshot_dynamic(&c->snapshot);landing_snapshot_init(&c->snapshot,&c->configuration.vehicle);if(status)snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s",status);}
static void reset_models(LandingController*c,const VehicleProfile*p){vessel_physics_init(&c->physics);c->adaptive_profile=*p;adaptive_calibrator_reset(&c->adaptive,p);c->aerodynamics=(AerodynamicModel){p->estimated_lift_to_drag,p->estimated_ballistic_coefficient,.2};c->planning_aerodynamics=c->aerodynamics;for(int i=0;i<4;i++)c->envelope.regimes[i]=c->aerodynamics;trajectory_calibrator_clear(&c->trajectory_calibrator);trajectory_calibrator_init(&c->trajectory_calibrator);c->trajectory_calibration=(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0};guidance_command_init(&c->last_command);c->snapshot.adaptive_vehicle_profile=*p;}

static void stop_thread(LandingController*c){
    pthread_mutex_lock(&c->mutex);
    bool join=c->thread_started;
    c->stop_thread=true;
    pthread_mutex_unlock(&c->mutex);
    if(join)pthread_join(c->thread,NULL);
    pthread_mutex_lock(&c->mutex);
    c->thread_started=false;
    c->stop_thread=false;
    pthread_mutex_unlock(&c->mutex);
    stop_prediction_worker(c);
}
static void disconnect_locked(LandingController*c,bool should_publish){vessel_physics_init(&c->physics);if(c->flight_log||c->planner_log){c->snapshot.log_sequence=++c->log_sequence;c->snapshot.wall_monotonic_seconds=monotonic_seconds();snprintf(c->snapshot.decision_reason,sizeof(c->snapshot.decision_reason),"event=disconnect; safe_control_release=true");snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Disconnecting and releasing control.");(void)append_vehicle_log(c,c->snapshot.decision_reason,true,false);append_planner_log(c,c->snapshot.decision_reason,true);}close_log(c);if(c->session){krpc_session_close(c->session);c->session=NULL;}if(c->has_plan){deorbit_plan_clear(&c->plan);c->has_plan=false;}c->plan_stale=false;c->has_latest=false;c->has_prediction_cache=false;c->cached_prediction_ut=-1e300;c->cached_prediction_sign=0;trajectory_clear(&c->dynamic_trajectory);trajectory_init(&c->dynamic_trajectory);trajectory_clear(&c->stabilized_trajectory);trajectory_init(&c->stabilized_trajectory);guidance_machine_init(&c->guidance);glide_calibration_init(&c->glide);trajectory_calibrator_clear(&c->trajectory_calibrator);trajectory_calibrator_init(&c->trajectory_calibrator);c->trajectory_calibration=(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0};guidance_command_init(&c->last_command);c->has_last_tick_ut=false;c->has_last_tick_wall=false;c->time_warp_issued=false;c->last_trajectory_ut=-1e300;c->last_prediction_ut=-1e300;if(should_publish){reset_snapshot(c,"Disconnected");publish(c);}}

static void actual_trajectory_update(LandingController*c,const Telemetry*t){if(t->ut-c->last_trajectory_ut<1)return;c->last_trajectory_ut=t->ut;TrajectoryPoint p={t->ut,t->latitude,t->longitude,t->mean_altitude,t->true_air_speed,c->snapshot.phase,TRAJ_ACTUAL};if(!trajectory_append(&c->snapshot.actual_trajectory,p))return;c->log_actual_trajectory_dirty=true;if(c->snapshot.actual_trajectory.count>3600){size_t excess=c->snapshot.actual_trajectory.count-3600;memmove(c->snapshot.actual_trajectory.points,c->snapshot.actual_trajectory.points+excess,3600*sizeof(TrajectoryPoint));c->snapshot.actual_trajectory.count=3600;}}

static double prediction_blend(double previous,double raw,double alpha){if(!isfinite(previous)||!isfinite(raw))return raw;return previous+(raw-previous)*clampd(alpha,0,1);}
static void prediction_cache_store(LandingController*c,const EntryPrediction*pr){
    c->has_prediction_cache=true;
    c->cached_predicted_miss_distance=pr->closest_distance;
    c->cached_predicted_taem_distance=pr->taem_distance;
    c->cached_predicted_taem_range_error=pr->taem_range_error;
    c->cached_predicted_entry_range=pr->entry_range;
    c->cached_predicted_entry_flight_path_angle=pr->entry_flight_path_angle;
    c->cached_predicted_taem_speed=pr->taem_speed;
    c->cached_predicted_taem_energy_error=pr->taem_energy_error;
    c->cached_predicted_peak_dynamic_pressure=pr->peak_dynamic_pressure;
    c->cached_predicted_peak_g_load=pr->peak_g_load;
    c->cached_physics_observed_seconds=pr->physics_observed_seconds;c->cached_physics_fallback_seconds=pr->physics_fallback_seconds;
    c->cached_predicted_s_turn_reversals=(double)pr->s_turn_reversals;
    c->cached_predicted_shadow_guidance=pr->shadow_guidance_used;
    c->cached_predicted_terminal_policy_feasible=pr->shadow_terminal_policy_feasible;
    c->cached_predicted_terminal_survivability_stress_score=pr->terminal_survivability_stress_score;
    c->cached_predicted_physics_relative_uncertainty=pr->physics_relative_uncertainty;
    c->cached_predicted_uncertainty_scenarios=pr->uncertainty_scenarios;
}
static void prediction_cache_store_filtered(LandingController*c,const EntryPrediction*pr,double alpha){
    if(!c->has_prediction_cache||alpha>=.999){prediction_cache_store(c,pr);return;}
    c->cached_predicted_miss_distance=prediction_blend(c->cached_predicted_miss_distance,pr->closest_distance,alpha);
    c->cached_predicted_taem_distance=prediction_blend(c->cached_predicted_taem_distance,pr->taem_distance,alpha);
    c->cached_predicted_taem_range_error=prediction_blend(c->cached_predicted_taem_range_error,pr->taem_range_error,alpha);
    c->cached_predicted_entry_range=prediction_blend(c->cached_predicted_entry_range,pr->entry_range,alpha);
    c->cached_predicted_entry_flight_path_angle=prediction_blend(c->cached_predicted_entry_flight_path_angle,pr->entry_flight_path_angle,alpha);
    c->cached_predicted_taem_speed=prediction_blend(c->cached_predicted_taem_speed,pr->taem_speed,alpha);
    c->cached_predicted_taem_energy_error=prediction_blend(c->cached_predicted_taem_energy_error,pr->taem_energy_error,alpha);
    c->cached_predicted_peak_dynamic_pressure=prediction_blend(c->cached_predicted_peak_dynamic_pressure,pr->peak_dynamic_pressure,alpha);
    c->cached_predicted_peak_g_load=prediction_blend(c->cached_predicted_peak_g_load,pr->peak_g_load,alpha);
    c->cached_physics_observed_seconds=pr->physics_observed_seconds;c->cached_physics_fallback_seconds=pr->physics_fallback_seconds;
    c->cached_predicted_s_turn_reversals=(double)pr->s_turn_reversals;
    c->cached_predicted_shadow_guidance=pr->shadow_guidance_used;
    c->cached_predicted_terminal_policy_feasible=pr->shadow_terminal_policy_feasible;
    c->cached_predicted_terminal_survivability_stress_score=prediction_blend(c->cached_predicted_terminal_survivability_stress_score,pr->terminal_survivability_stress_score,alpha);
    c->cached_predicted_physics_relative_uncertainty=prediction_blend(c->cached_predicted_physics_relative_uncertainty,pr->physics_relative_uncertainty,alpha);
    c->cached_predicted_uncertainty_scenarios=pr->uncertainty_scenarios;
}

static void prediction_cache_apply(const LandingController*c,Telemetry*t,const LandingConfiguration*cfg){
    (void)cfg;
    if(!c->has_prediction_cache)return;
    t->predicted_physics_observed_seconds=c->cached_physics_observed_seconds;t->predicted_physics_fallback_seconds=c->cached_physics_fallback_seconds;
    t->predicted_miss_distance=c->cached_predicted_miss_distance;
    t->predicted_taem_distance=c->cached_predicted_taem_distance;
    t->predicted_taem_range_error=c->cached_predicted_taem_range_error;
    t->predicted_entry_range=c->cached_predicted_entry_range;
    t->predicted_entry_flight_path_angle=c->cached_predicted_entry_flight_path_angle;
    t->predicted_taem_speed=c->cached_predicted_taem_speed;
    t->predicted_taem_energy_error=c->cached_predicted_taem_energy_error;
    t->predicted_peak_dynamic_pressure=c->cached_predicted_peak_dynamic_pressure;
    t->predicted_peak_g_load=c->cached_predicted_peak_g_load;
    t->predicted_s_turn_reversals=c->cached_predicted_s_turn_reversals;
    t->predicted_shadow_guidance=c->cached_predicted_shadow_guidance;
    t->predicted_terminal_policy_feasible=c->cached_predicted_terminal_policy_feasible;
    t->predicted_terminal_survivability_stress_score=c->cached_predicted_terminal_survivability_stress_score;
    t->predicted_physics_relative_uncertainty=c->cached_predicted_physics_relative_uncertainty;
    t->predicted_uncertainty_scenarios=c->cached_predicted_uncertainty_scenarios;
    t->predicted_raw_published_position_delta_30s=c->cached_predicted_raw_published_position_delta_30s;
    t->predicted_raw_published_position_delta_60s=c->cached_predicted_raw_published_position_delta_60s;
    t->predicted_raw_published_position_delta_120s=c->cached_predicted_raw_published_position_delta_120s;
}

static void dynamic_prediction(LandingController*c,Telemetry*t,const VehicleState*state,const LandingConfiguration*cfg){
    const PlanetModel*p=krpc_session_planet(c->session);
    bool terminal_shadow=taem_exec_owns_vehicle(&c->guidance.taem_exec);
    /* Once MM305 owns the vehicle, keep forecasting rather than freezing the last
       Entry path.  The terminal branch below runs a cloned guidance_update loop,
       so forecast policy and live TAEM/final policy are literally the same code. */
    /* Recovery is a real-time stabilization maneuver.  A full atmospheric
       rollout has no authority over the recovery command, yet it used to run
       every prediction interval and could hold the last direct-surface input
       for several wall-clock seconds.  Keep the last forecast visible while
       the fast control loop recovers the airframe. */
    /* MM305 terminal shadowing is diagnostic only and can take seconds because it
       recursively runs cloned terminal guidance. Never put that ensemble ahead of
       the live TAEM command tick; retain the last prediction for display/logging
       and let terminal_guidance() run its bounded internal planner synchronously. */
    bool postburn_verify=c->has_plan&&c->guidance.deorbit_burn_completed&&!c->plan.achieved_state_verified&&planet_atmospheric_density(p,t->mean_altitude)<=DBL_EPSILON;
    /* The post-burn vehicle normally remains in ENTRY_INTERFACE while it coasts
       from orbital altitude toward the MM304 entry gate.  Do not let the ordinary
       entry-phase prediction fast-path suppress the one achieved-state verification
       that certifies the actual burn, arms the strict checkpoint, and permits the
       guarded pre-entry warp. */
    if(c->guidance.phase==PHASE_ATTITUDE_RECOVERY||terminal_shadow||
       c->guidance.phase==PHASE_ENTRY_ENERGY||
       (c->guidance.phase==PHASE_ENTRY_INTERFACE&&!postburn_verify)){prediction_cache_apply(c,t,cfg);return;}
    bool burn_cutoff_predict=c->has_plan&&c->guidance.has_burn_command_started&&!c->guidance.deorbit_burn_completed&&planet_atmospheric_density(p,t->mean_altitude)<=DBL_EPSILON;
    /* Do not run the full atmospheric propagator while guidance is idle.
       In an in-atmosphere restart this control thread used to spend several
       wall-clock seconds solving a forecast while holding the controller mutex,
       delaying engageReentry and leaving the real vehicle neutral and free to
       roll. Static plan/cache data is sufficient until automation actually owns
       the vehicle; post-burn verification remains the only idle exception. */
    if(!c->guidance.automation_engaged&&!postburn_verify){prediction_cache_apply(c,t,cfg);return;}
    double prediction_interval=burn_cutoff_predict?fmin(cfg->guidance.prediction_interval,.75):cfg->guidance.prediction_interval;
    if(!postburn_verify&&t->ut-c->last_prediction_ut<prediction_interval){prediction_cache_apply(c,t,cfg);return;}
    if(planet_atmospheric_density(p,t->mean_altitude)<=DBL_EPSILON&&!burn_cutoff_predict&&!postburn_verify){prediction_cache_apply(c,t,cfg);return;}

    c->last_prediction_ut=t->ut;
    double signed_roll=norm_signed_deg(c->last_command.target_roll),sign=0;
    bool entry_state=c->guidance.phase==PHASE_ENTRY_INTERFACE||c->guidance.phase==PHASE_ENTRY_ENERGY||c->guidance.phase==PHASE_ATTITUDE_RECOVERY;
    if(c->guidance.automation_engaged&&entry_state&&fabs(c->guidance.s_turn_sign)>DBL_EPSILON)sign=c->guidance.s_turn_sign>=0?1:-1;
    else if(c->last_command.autopilot_engaged&&fabs(signed_roll)>DBL_EPSILON)sign=signed_roll>=0?1:-1;
    /* Guidance owns the S-turn branch even while the shaped bank command is
       still near zero. Letting the predictor infer a fresh branch from tiny
       course-error sign changes was the main source of +/-hundreds-of-km TAEM
       jumps in the 80 km replay. */
    if(sign==0&&c->guidance.deorbit_burn_completed)sign=c->guidance.s_turn_sign>=0?1:-1;

    AerodynamicModel prediction_aero=c->aerodynamics;
    AerodynamicEnvelope prediction_envelope=c->envelope;
    /* c->aerodynamics and c->envelope are already continuously learned from
       live force samples. Do not apply a second one-frame force anchor here:
       multiplying a fresh observation into every forecast created a positive
       feedback loop between forecast, residual calibration, and the next
       forecast. */
    double initial_roll_rate=isfinite(t->roll_rate)?t->roll_rate:0.0;
    double initial_aoa_rate=t->has_angle_of_attack_rate&&isfinite(t->angle_of_attack_rate)?
        t->angle_of_attack_rate:0.0;
    double leg_elapsed=guidance_entry_leg_elapsed(&c->guidance,t->ut);
    /* The live MM304 program owns bank/AoA/reversal state. Prediction is a
       read-only consumer of that already-selected command; it must never run the
       legacy broad optimizer or advance the shared plan timestamp first. */
    double commit_remaining=c->guidance.entry_control_plan_valid?
        fmax(0,c->guidance.entry_control_segment_until_ut-t->ut):0;
    EntryControlPlan control_plan={0};
    bool solve_entry_control=c->guidance.automation_engaged&&c->guidance.phase==PHASE_ENTRY_ENERGY;
    if(solve_entry_control&&c->guidance.entry_control_plan_valid)
        control_plan=guidance_control_plan_snapshot(&c->guidance,t->ut,commit_remaining);

    /* Forecast the exact guidance-owned plan. The predictor may assess the
       resulting trajectory for telemetry, but only entry_program_guidance() and
       predictor_supervise_entry_control() are allowed to change MM304 commands. */
    EntryControlPlan forecast_plan=guidance_terminal_control_plan(&c->guidance,control_plan);
    if(solve_entry_control&&forecast_plan.valid&&commit_remaining>0)
        forecast_plan.segment_duration=commit_remaining;
    bool forecast_has_entry_plan=solve_entry_control&&forecast_plan.valid&&c->guidance.entry_s_turn_plan.valid;
    c->forecast_entry_plan_identity_valid=forecast_has_entry_plan;
    c->forecast_entry_plan_id=forecast_has_entry_plan?c->guidance.entry_s_turn_plan.plan_id:0;
    c->forecast_entry_plan_version=forecast_has_entry_plan?c->guidance.entry_s_turn_plan.plan_version:0;
    EntryPrediction pr=solve_entry_control&&forecast_plan.valid?
        predictor_simulate_entry_control_plan_to_interface(*state,p,prediction_aero,&prediction_envelope,&c->trajectory_calibration,&cfg->vehicle,&cfg->site,&cfg->guidance,c->guidance.taem_interface_target.valid?&c->guidance.taem_interface_target:NULL,true,t->roll,initial_roll_rate,t->angle_of_attack,initial_aoa_rate,sign,leg_elapsed,&forecast_plan,2500,true):
        predictor_simulate_entry_with_attitude(*state,p,prediction_aero,&prediction_envelope,&c->trajectory_calibration,&cfg->vehicle,&cfg->site,&cfg->guidance,cfg->vehicle.maximum_bank_angle,t->roll,initial_roll_rate,t->angle_of_attack,initial_aoa_rate,sign,leg_elapsed,(burn_cutoff_predict||postburn_verify)?4500:2500,true);
    trajectory_clear(&c->dynamic_trajectory);c->dynamic_trajectory=pr.trajectory;pr.trajectory=(Trajectory){0};c->log_raw_plan_trajectory_dirty=true;
    double filter_dt=c->has_prediction_cache&&c->cached_prediction_ut>-1e200&&t->ut>c->cached_prediction_ut?t->ut-c->cached_prediction_ut:0;
    double entry_alt=p->atmosphere_depth-cfg->guidance.entry_interface_altitude_margin,remaining=clampd((t->mean_altitude-cfg->guidance.taem_interface_altitude)/fmax(entry_alt-cfg->guidance.taem_interface_altitude,1),0,1);
    double tau=1.25+2.75*remaining;
    bool side_changed=c->has_prediction_cache&&fabs(c->cached_prediction_sign)>.1&&fabs(sign)>.1&&c->cached_prediction_sign*sign<0;
    if(side_changed)tau=fmax(.75,tau*.55);
    double alpha=filter_dt>0?1.0-exp(-clampd(filter_dt,.01,10.0)/tau):1.0;
    alpha=clampd(alpha,.10,1.0);
    /* Burn-cutoff forecasts are conditional on continuing the burn. Once the
       burn completes, blending one of those states into the achieved-state
       forecast mixes two different initial conditions and can leave TAEM
       distance/range/speed internally inconsistent for many seconds. The
       first verified post-burn solve is a state-boundary event: replace the
       cache and displayed path atomically with that raw physical forecast. */
    EntryPrediction display_pr=pr;
    if(!pr.reached_taem){
        /* A propagated trajectory that never acquires TAEM does not have a
           meaningful TAEM distance, range error, or TAEM speed. Publishing
           the eventual crash/low-speed endpoint as "TAEM speed" produced the
           81 m/s contradiction seen in the flight log. Keep those fields
           explicitly unavailable while still publishing the exact live
           specific energy remaining to the configured TAEM energy state. */
        display_pr.taem_distance=NAN;
        display_pr.taem_range_error=NAN;
        display_pr.taem_speed=NAN;
        display_pr.taem_energy_error=entry_remaining_specific_energy(
            t->latitude,t->mean_altitude,t->true_air_speed,cfg->site.latitude,
            cfg->guidance.taem_interface_altitude,entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
    }
    if(postburn_verify||terminal_shadow){
        prediction_cache_store(c,&display_pr);
        alpha=1.0;
    }else prediction_cache_store_filtered(c,&display_pr,alpha);
    c->cached_prediction_ut=t->ut;c->cached_prediction_sign=sign;prediction_cache_apply(c,t,cfg);
    /* Keep the raw physical forecast for residual learning. The operator and
       plan get a time-aligned, continuity-preserving path so successive 1.5 s
       solves do not redraw the entire future trajectory onto another branch. */
    trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&c->dynamic_trajectory);
    if(postburn_verify||terminal_shadow){
        trajectory_clear(&c->stabilized_trajectory);
        trajectory_init(&c->stabilized_trajectory);
        trajectory_copy(&c->stabilized_trajectory,&c->dynamic_trajectory);
    }else if(!stabilize_forecast_trajectory(&c->stabilized_trajectory,&c->dynamic_trajectory,clampd(alpha*1.25,.12,1.0)))trajectory_copy(&c->stabilized_trajectory,&c->dynamic_trajectory);
    c->cached_predicted_raw_published_position_delta_30s=trajectory_position_delta_at(&c->dynamic_trajectory,&c->stabilized_trajectory,t->ut+30.0,p->radius);
    c->cached_predicted_raw_published_position_delta_60s=trajectory_position_delta_at(&c->dynamic_trajectory,&c->stabilized_trajectory,t->ut+60.0,p->radius);
    c->cached_predicted_raw_published_position_delta_120s=trajectory_position_delta_at(&c->dynamic_trajectory,&c->stabilized_trajectory,t->ut+120.0,p->radius);
    t->predicted_raw_published_position_delta_30s=c->cached_predicted_raw_published_position_delta_30s;
    t->predicted_raw_published_position_delta_60s=c->cached_predicted_raw_published_position_delta_60s;
    t->predicted_raw_published_position_delta_120s=c->cached_predicted_raw_published_position_delta_120s;

    if(c->has_plan){
        c->plan.predicted_closest_distance=c->cached_predicted_miss_distance;c->plan.predicted_taem_distance=c->cached_predicted_taem_distance;c->plan.predicted_taem_range_error=c->cached_predicted_taem_range_error;c->plan.predicted_entry_range=c->cached_predicted_entry_range;c->plan.predicted_entry_flight_path_angle=c->cached_predicted_entry_flight_path_angle;
        if(!trajectory_equal(&c->plan.trajectory,&c->stabilized_trajectory)){trajectory_copy(&c->plan.trajectory,&c->stabilized_trajectory);c->log_plan_trajectory_dirty=true;}
        double peri=c->plan.achieved_state_verified?c->plan.achieved_post_burn_periapsis_altitude:c->plan.predicted_post_burn_periapsis_altitude;
        if(burn_cutoff_predict){
            double cutoff_peri=predictor_postburn_periapsis(*state,p);
            c->plan.live_cutoff_periapsis_altitude=cutoff_peri;
            c->plan.live_cutoff_closest_distance=pr.closest_distance;
            c->plan.live_cutoff_entry_flight_path_angle=pr.entry_flight_path_angle;
            c->plan.live_cutoff_capture_qualified=deorbit_capture_qualified(&pr,&cfg->site,&cfg->vehicle,&cfg->guidance,cutoff_peri,true);
        }else if(!c->guidance.has_burn_command_started)c->plan.live_cutoff_capture_qualified=false;
        if(postburn_verify){peri=predictor_postburn_periapsis(*state,p);c->plan.achieved_state_verified=true;c->plan.achieved_post_burn_periapsis_altitude=peri;}
        bool live_capture=deorbit_capture_qualified(&pr,&cfg->site,&cfg->vehicle,&cfg->guidance,peri,t->mean_altitude>entry_alt);
        /* Achieved-state qualification is a burn-cutoff verification result,
           not a live-entry forecast bit. Re-evaluating it on every atmospheric
           prediction made a previously strict checkpoint flicker in and out of
           qualification as the S-turn forecast evolved. Freeze it once the
           post-burn state has actually been verified; live_capture remains a
           separate current-forecast confidence input. */
        if(postburn_verify)c->plan.achieved_state_capture_qualified=live_capture;
        c->plan.target_capture_achieved=c->plan.robustness_qualified&&(!c->plan.achieved_state_verified||c->plan.achieved_state_capture_qualified);
        double robust=c->plan.robustness_pass_fraction*(c->plan.robustness_unsafe?.45:1.0);
        /* Score the aerodynamic model actually used for planning, not merely
           the current Mach bin.  If trajectory adaptation is intentionally
           disabled, transfer that weight to the planning aero prior instead
           of permanently charging the plan its 0.05 cold-start confidence. */
        double trajectory_term=cfg->calibration.enable_trajectory_calibration?
            c->trajectory_calibration.confidence:c->planning_aerodynamics.confidence;
        c->plan.confidence=clampd(c->planning_aerodynamics.confidence*.25+trajectory_term*.25+robust*.25+(live_capture?.20:.05)+(c->plan.achieved_state_verified?.05:0),.03,.97);
    }
    entry_prediction_clear(&pr);
}

static bool terminal_prediction_plan_copy(const DeorbitPlan *src,DeorbitPlan *dst){
    if(!dst)return false;
    memset(dst,0,sizeof(*dst));
    trajectory_init(&dst->trajectory);
    if(!src)return true;
    *dst=*src;
    dst->trajectory=(Trajectory){0};
    if(trajectory_copy(&dst->trajectory,&src->trajectory))return true;
    trajectory_clear(&dst->trajectory);
    memset(dst,0,sizeof(*dst));
    trajectory_init(&dst->trajectory);
    return false;
}

static AsyncPredictionIdentity terminal_prediction_identity(const GuidanceMachine *g){
    AsyncPredictionIdentity id={0};
    if(!g)return id;
    id.guidance_phase=(int)g->phase;
    id.taem_phase=(int)g->taem_exec.phase;
    id.taem_transition_count=g->taem_exec.transition_count;
    id.control_plan_sequence=g->control_plan_sequence;
    if(g->taem_s_turn_plan.valid){
        id.plan_id=g->taem_s_turn_plan.plan_id;
        id.plan_version=g->taem_s_turn_plan.plan_version;
    }
    id.terminal_candidate_valid=g->terminal_candidate.valid;
    id.terminal_candidate_selected_ut=g->terminal_candidate.valid?g->terminal_candidate.selected_ut:NAN;
    id.terminal_path_kind=(int)g->terminal_path_kind;
    id.terminal_path_committed=g->terminal_path_committed;
    id.hac_side_selected=g->hac_side_selected;
    id.hac_captured=g->hac_captured;
    id.final_approach_captured=g->final_approach_captured;
    return id;
}

static void terminal_prediction_commit(LandingController *c,EntryPrediction *pr,double request_ut,double sign){
    if(!c||!pr||!pr->shadow_guidance_used)return;
    prediction_cache_store(c,pr);
    c->cached_prediction_ut=request_ut;
    c->cached_prediction_sign=sign;
    trajectory_clear(&c->dynamic_trajectory);
    c->dynamic_trajectory=pr->trajectory;
    pr->trajectory=(Trajectory){0};
    trajectory_clear(&c->stabilized_trajectory);
    if(!trajectory_copy(&c->stabilized_trajectory,&c->dynamic_trajectory))
        trajectory_init(&c->stabilized_trajectory);
    trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&c->dynamic_trajectory);
    c->cached_predicted_raw_published_position_delta_30s=0;
    c->cached_predicted_raw_published_position_delta_60s=0;
    c->cached_predicted_raw_published_position_delta_120s=0;
    c->log_raw_plan_trajectory_dirty=true;
    if(c->has_plan){
        c->plan.predicted_closest_distance=c->cached_predicted_miss_distance;
        c->plan.predicted_taem_distance=c->cached_predicted_taem_distance;
        c->plan.predicted_taem_range_error=c->cached_predicted_taem_range_error;
        c->plan.predicted_entry_range=c->cached_predicted_entry_range;
        c->plan.predicted_entry_flight_path_angle=c->cached_predicted_entry_flight_path_angle;
        if(!trajectory_equal(&c->plan.trajectory,&c->stabilized_trajectory)){
            trajectory_copy(&c->plan.trajectory,&c->stabilized_trajectory);
            c->log_plan_trajectory_dirty=true;
        }
    }
}

static void *terminal_prediction_worker(void *arg){
    LandingController *c=arg;
    for(;;){
        VehicleState state={0};
        Telemetry telemetry;telemetry_init(&telemetry);
        LandingConfiguration cfg={0};
        PlanetModel planet={0};
        AerodynamicModel aero={0};
        AerodynamicEnvelope envelope={0};
        VesselPhysicsModel physics={0};
        TrajectoryCalibrationModel calibration={0};
        GuidanceMachine guidance={0},entry_request={0};
        bool entry_job=false,entry_planning_job=false;uint64_t generation=0,request_tick_sequence=0;
        PredictorPlannerTrace entry_trace={0};bool have_entry_trace=false;
        DeorbitPlan plan;memset(&plan,0,sizeof(plan));trajectory_init(&plan.trajectory);
        bool run=false,has_plan=false;
        double request_ut=0,sign=0;
        AsyncPredictionIdentity request_identity={0};

        pthread_mutex_lock(&c->mutex);
        if(c->stop_prediction_thread){
            pthread_mutex_unlock(&c->mutex);
            deorbit_plan_clear(&plan);
            break;
        }
        if(c->session&&c->has_latest&&c->guidance.automation_engaged&&!c->guidance.paused&&!c->guidance.aborted&&
           c->guidance.phase!=PHASE_ATTITUDE_RECOVERY&&
           (taem_exec_owns_vehicle(&c->guidance.taem_exec)||
            (c->guidance.phase==PHASE_ENTRY_ENERGY&&
             (c->guidance.entry_planning_needed||c->guidance.entry_final_reversal_pending||
              c->guidance.entry_final_reversal_completed)))){
            if(c->latest_telemetry.ut<fmax(c->last_terminal_prediction_request_ut,c->last_terminal_prediction_completion_ut)-1.0){
                c->last_terminal_prediction_request_ut=-1e300;
                c->last_terminal_prediction_completion_ut=-1e300;
                c->last_terminal_prediction_completion_wall=-1e300;
                c->last_terminal_prediction_solve_wall=0.0;
            }
            double cadence_anchor=fmax(c->last_terminal_prediction_request_ut,c->last_terminal_prediction_completion_ut);
            double now_wall=monotonic_seconds();
            if(async_prediction_request_due(c->latest_telemetry.ut,cadence_anchor,now_wall,
                    c->last_terminal_prediction_completion_wall,c->last_terminal_prediction_solve_wall,
                    c->configuration.guidance.prediction_interval)){
                state=c->latest_state;
                telemetry=c->latest_telemetry;
                cfg=effective_config(c);
                planet=*krpc_session_planet(c->session);
                aero=c->aerodynamics;
                envelope=c->envelope;
                physics=c->physics;
                calibration=c->trajectory_calibration;
                calibration.physics=&physics;
                guidance=c->guidance;
                entry_request=guidance;
                entry_request.previous_ut=telemetry.ut;
                entry_job=guidance.phase==PHASE_ENTRY_ENERGY;
                entry_planning_job=entry_job&&guidance.entry_planning_needed;
                request_tick_sequence=c->tick_sequence;
                generation=c->prediction_generation;
                if(guidance.entry_predictor_models_valid)
                    guidance.entry_predictor_calibration.physics=&physics;
                if(c->has_plan)has_plan=terminal_prediction_plan_copy(&c->plan,&plan);
                request_identity=terminal_prediction_identity(&c->guidance);
                request_ut=telemetry.ut;
                if(fabs(c->guidance.s_turn_sign)>DBL_EPSILON)sign=c->guidance.s_turn_sign>=0?1:-1;
                else{
                    double roll=norm_signed_deg(c->last_command.target_roll);
                    if(fabs(roll)>DBL_EPSILON)sign=roll>=0?1:-1;
                }
                c->last_terminal_prediction_request_ut=request_ut;
                run=true;
            }
        }
        pthread_mutex_unlock(&c->mutex);

        if(!run){deorbit_plan_clear(&plan);sleep_seconds(.05);continue;}
        double solve_started=monotonic_seconds();
        EntryPrediction pr;
        bool had_entry_topology=guidance.entry_topology.valid;
        if(entry_job){
            /* Search and replay own only this immutable snapshot, outside the
               controller mutex. Live guidance keeps running fresh safety laws. */
            guidance.entry_planning_deferred=false;
            if(!isfinite(guidance.entry_s_turn_plan.cost)||
               (guidance.entry_lateral_infeasible&&!guidance.entry_reversal_scheduled))
                guidance.entry_control_plan_valid=false; /* initial fallback needs a real topology now */
            guidance.diagnostic_shadow=true;
            bool replaceable_ordinary_reversal=guidance.entry_reversal_scheduled&&
                !guidance.entry_reversal_is_final&&!guidance.entry_control_reversals&&
                !guidance.entry_final_reversal_pending&&!guidance.entry_final_reversal_completed;
            if(!guidance.entry_topology.valid&&
               (!guidance.entry_reversal_scheduled||replaceable_ordinary_reversal)&&
               !guidance.entry_control_reversals&&!guidance.entry_final_reversal_pending&&
               !guidance.entry_final_reversal_completed){
                /* The topology search runs before the worker's guidance_update(). Publish
                   the same fixed MM304 interface target first; otherwise an empty worker
                   snapshot silently falls back to +/-90 deg while live guidance is flying
                   a turnability-selected 45-90 deg outlet. That target mismatch made every
                   early topology solve geometrically impossible and left live MM304 on its
                   fallback shaping side until it was far past KSC. */
                double worker_course=isfinite(telemetry.ground_track_heading)?
                    telemetry.ground_track_heading:telemetry.heading;
                entry_publish_taem_tangent_target(&guidance,&telemetry,worker_course,&planet,aero,&cfg);
                EntryTopologyPlan topology=predictor_plan_entry_topology(state,&telemetry,&guidance,&planet,
                    &envelope,&calibration,&cfg);
                if(topology.valid)(void)guidance_install_entry_topology(&guidance,&topology,telemetry.ut);
            }
            GuidanceResult proposal=guidance_update(&guidance,&telemetry,&state,
                has_plan?&plan:NULL,&planet,aero,&cfg);
            guidance_result_clear(&proposal);
            have_entry_trace=entry_planning_job&&predictor_last_planner_trace(&entry_trace);
            if(have_entry_trace)entry_trace.producing_tick_sequence=request_tick_sequence;
            /* Publish candidate geometry immediately after the cheap planning pass.
               Full-flight shadow validation continues below before anything can be
               adopted for execution. This keeps the early MM304 map informative
               without coupling display latency to the expensive terminal forecast. */
            if(have_entry_trace){
                pthread_mutex_lock(&c->mutex);
                bool preview_current=!c->stop_prediction_thread&&c->session&&c->has_latest&&
                    generation==c->prediction_generation&&c->guidance.phase==entry_request.phase&&
                    c->guidance.control_plan_sequence==entry_request.control_plan_sequence&&
                    c->guidance.s_turn_sign==entry_request.s_turn_sign&&
                    c->guidance.entry_control_reversals==entry_request.entry_control_reversals&&
                    c->guidance.entry_reversal_scheduled==entry_request.entry_reversal_scheduled&&
                    c->guidance.entry_final_reversal_pending==entry_request.entry_final_reversal_pending&&
                    c->guidance.entry_final_reversal_completed==entry_request.entry_final_reversal_completed&&
                    memcmp(&c->guidance.taem_interface_target,&entry_request.taem_interface_target,
                        sizeof(c->guidance.taem_interface_target))==0;
                if(preview_current){
                    PredictorPlannerTrace preview=entry_trace;
                    preview.accepted_tick_sequence=c->tick_sequence;
                    preview.selected_index=-1;preview.selection_converged=false;
                    for(unsigned i=0;i<preview.candidate_count;i++)preview.candidates[i].selected=false;
                    c->entry_worker_trace=preview;c->entry_worker_trace_valid=true;
                }
                pthread_mutex_unlock(&c->mutex);
            }
            guidance.entry_planning_deferred=true;
            pr=predictor_simulate_entry_guidance_shadow(state,&telemetry,&guidance,
                has_plan?&plan:NULL,&planet,aero,&envelope,&calibration,&cfg,500,true);
            guidance.entry_s_turn_plan.terminal_ready=pr.reached_taem&&
                pr.taem_dynamic_interface_captured&&pr.taem_terminal_candidate_geometry_clean&&
                pr.shadow_terminal_policy_feasible;
            guidance.entry_committed_infeasible=pr.shadow_guidance_used&&
                pr.taem_ownership_boundary_missed&&pr.final_state.ut<=
                guidance.entry_s_turn_plan.planned_ut+guidance.entry_s_turn_plan.segment_duration;
            guidance.entry_supervision_valid=pr.shadow_guidance_used&&!pr.shadow_aborted&&
                pr.peak_dynamic_pressure<=cfg.vehicle.maximum_dynamic_pressure&&
                pr.peak_g_load<=cfg.vehicle.maximum_g_load&&
                !(pr.taem_ownership_boundary_missed&&pr.final_state.ut<=
                  guidance.entry_s_turn_plan.planned_ut+guidance.entry_s_turn_plan.segment_duration);
            guidance.entry_s_turn_plan.terminal_ready&=guidance.entry_supervision_valid;
            guidance.entry_s_turn_plan.closest_distance=pr.closest_distance;
            guidance.entry_s_turn_plan.taem_range_error=pr.reached_taem?pr.taem_range_error:NAN;
            guidance.entry_s_turn_plan.taem_speed=pr.reached_taem?pr.taem_speed:NAN;
            guidance.entry_s_turn_plan.taem_energy_error=pr.reached_taem?pr.taem_energy_error:NAN;
            guidance.entry_s_turn_plan.predicted_reversals=pr.s_turn_reversals;
            guidance.entry_supervision_boundary_missed=pr.taem_ownership_boundary_missed;
            guidance.entry_supervision_mode=guidance.entry_supervision_valid?
                ENTRY_SUPERVISION_PASS_THROUGH:ENTRY_SUPERVISION_INFEASIBLE;
            /* A new complete topology must survive the executable policy,
             * not merely the fast candidate propagator. Existing committed
             * plans retain their deadline; only first installation is gated. */
            if(guidance.entry_topology.valid&&!had_entry_topology&&
               (!pr.reached_taem||!pr.taem_dynamic_interface_captured||pr.shadow_aborted)){
                guidance.entry_s_turn_plan.valid=false;
                fprintf(stderr,"Entry topology withheld: executable shadow reached %d captured %d aborted %d missed %d.\n",
                    pr.reached_taem,pr.taem_dynamic_interface_captured,pr.shadow_aborted,pr.taem_ownership_boundary_missed);
            }
        }else pr=predictor_simulate_terminal_shadow_ensemble(state,&telemetry,&guidance,
            has_plan?&plan:NULL,&planet,aero,&envelope,&calibration,&cfg,600,true);
        double solve_finished=monotonic_seconds();
        deorbit_plan_clear(&plan);

        pthread_mutex_lock(&c->mutex);
        /* Coalesce to the freshest state and impose a wall-clock cooldown tied to
           the cost of the solve. A slow advisory forecast must never become a
           catch-up loop merely because KSP UT advanced while it was computing. */
        if(c->has_latest&&c->latest_telemetry.ut>=request_ut-1.0)
            c->last_terminal_prediction_completion_ut=fmax(request_ut,c->latest_telemetry.ut);
        c->last_terminal_prediction_completion_wall=solve_finished;
        c->last_terminal_prediction_solve_wall=fmax(0.0,solve_finished-solve_started);
        AsyncPredictionIdentity current_identity=terminal_prediction_identity(&c->guidance);
        bool accept=!entry_job&&!c->stop_prediction_thread&&c->session&&c->has_latest&&
            generation==c->prediction_generation&&
            taem_exec_owns_vehicle(&c->guidance.taem_exec)&&request_ut>c->cached_prediction_ut&&
            async_prediction_result_fresh(&request_identity,&current_identity,request_ut,
                c->latest_telemetry.ut,c->configuration.guidance.prediction_interval);
        bool entry_result_current=entry_job&&!c->stop_prediction_thread&&c->session&&c->has_latest&&
            generation==c->prediction_generation&&c->guidance.phase==entry_request.phase&&
            c->guidance.control_plan_sequence==entry_request.control_plan_sequence&&
            c->guidance.s_turn_sign==entry_request.s_turn_sign&&
            c->guidance.entry_control_reversals==entry_request.entry_control_reversals&&
            c->guidance.entry_reversal_scheduled==entry_request.entry_reversal_scheduled&&
            (!c->guidance.entry_reversal_scheduled||
             (c->guidance.entry_reversal_ut==entry_request.entry_reversal_ut&&
              c->guidance.entry_reversal_sign==entry_request.entry_reversal_sign&&
              c->guidance.entry_reversal_is_final==entry_request.entry_reversal_is_final))&&
            c->guidance.entry_final_reversal_pending==entry_request.entry_final_reversal_pending&&
            c->guidance.entry_final_reversal_completed==entry_request.entry_final_reversal_completed&&
            memcmp(&c->guidance.taem_interface_target,&entry_request.taem_interface_target,
                sizeof(c->guidance.taem_interface_target))==0;
        if(entry_result_current){
            bool tangent_forecast=entry_request.entry_final_reversal_pending||
                entry_request.entry_final_reversal_completed;
            bool adopted=!tangent_forecast&&guidance_accept_entry_plan(&c->guidance,&entry_request,&guidance,
                &c->latest_telemetry,&cfg);
            if((adopted||tangent_forecast)&&request_ut>c->cached_prediction_ut){
                terminal_prediction_commit(c,&pr,request_ut,sign);
                const EntryControlPlan*live_plan=current_entry_control_plan(c);
                c->forecast_entry_plan_identity_valid=live_plan&&live_plan->valid&&live_plan->plan_id!=0;
                c->forecast_entry_plan_id=c->forecast_entry_plan_identity_valid?live_plan->plan_id:0;
                c->forecast_entry_plan_version=c->forecast_entry_plan_identity_valid?live_plan->plan_version:0;
            }
            /* PLAN geometry is useful before an executable proposal is accepted. Publish
               the worker trace as an explicitly non-selected preview whenever its request
               lineage still matches live MM304. This changes telemetry only: rejected
               candidates never cross into guidance state or flight controls. A tangent-
               capture-only forecast has no new search trace and must not re-stamp an old one. */
            if(have_entry_trace){
                entry_trace.accepted_tick_sequence=c->tick_sequence;
                entry_trace.selected_index=-1;entry_trace.selection_converged=false;
                for(unsigned i=0;i<entry_trace.candidate_count;i++){
                    PredictorPlannerCandidateTrace*candidate=&entry_trace.candidates[i];
                    candidate->selected=false;
                    if(!adopted){
                        candidate->assessment.valid=false;
                        candidate->assessment.terminal_feasible=false;
                        candidate->plan.terminal_ready=false;
                        candidate->rejection_flags|=PREDICTOR_CANDIDATE_REJECT_POLICY_PRIORITY;
                    }
                }
                entry_trace.mode=guidance.entry_supervision_mode;
                c->entry_worker_trace=entry_trace;c->entry_worker_trace_valid=true;
            }else if(tangent_forecast)c->entry_worker_trace_valid=false;
        }else if(accept)terminal_prediction_commit(c,&pr,request_ut,sign);
        pthread_mutex_unlock(&c->mutex);
        entry_prediction_clear(&pr);
    }
    return NULL;
}


/* Short terminal candidate searches have their own native scheduling lane.
   A several-second full-flight advisory shadow must not delay the current
   terminal preview, and neither computation may run inside the control mutex. */
static void *terminal_preview_worker(void*context){
    LandingController*c=context;
    double next_wall=0.0;
    for(;;){
        GuidanceMachine request={0};Telemetry telemetry={0};PlanetModel planet={0};
        LandingConfiguration cfg={0};AerodynamicModel aero={0};
        VesselPhysicsModel physics={0};uint64_t generation=0;bool run=false;
        pthread_mutex_lock(&c->mutex);
        if(c->stop_terminal_preview_thread){pthread_mutex_unlock(&c->mutex);break;}
        GuidanceMachine*g=&c->guidance;
        cfg=effective_config(c);
        double refresh=cfg.guidance.prediction_interval;
        double poll_period=1.0/cfg.guidance.guidance_rate;
        if(c->session&&c->has_latest&&g->automation_engaged&&!g->paused&&!g->aborted&&
           !g->attitude_recovery&&!g->terminal_path_committed&&!g->final_approach_captured&&
           !(g->hac_side_selected&&!g->terminal_test_capture_active)&&
           (g->phase==PHASE_ENTRY_ENERGY||taem_exec_owns_vehicle(&g->taem_exec))&&
           c->latest_telemetry.ut>=g->terminal_reentry_after_ut&&
           c->latest_telemetry.vertical_speed<0.0&&
           monotonic_seconds()>=next_wall&&
           (!isfinite(g->terminal_prediction_ut)||
            c->latest_telemetry.ut-g->terminal_prediction_ut>=refresh)){
            request=*g;telemetry=c->latest_telemetry;planet=*krpc_session_planet(c->session);
            aero=c->aerodynamics;physics=c->physics;
            generation=c->prediction_generation;run=true;
        }
        pthread_mutex_unlock(&c->mutex);
        if(!run){sleep_seconds(poll_period);continue;}
        GuidanceMachine proposal=request;
        if(proposal.entry_predictor_models_valid)
            proposal.entry_predictor_calibration.physics=&physics;
        double started=monotonic_seconds();
        bool planned=guidance_plan_terminal_preview(&proposal,&telemetry,&planet,aero,&cfg);
        double elapsed=monotonic_seconds()-started;
        pthread_mutex_lock(&c->mutex);
        if(planned&&!c->stop_terminal_preview_thread&&c->session&&c->has_latest&&
           generation==c->prediction_generation)
            (void)guidance_accept_terminal_preview(&c->guidance,&request,&proposal,
                &c->latest_telemetry,&cfg);
        pthread_mutex_unlock(&c->mutex);
        next_wall=monotonic_seconds()+clampd(elapsed*.25,.05,.50);
    }
    return NULL;
}

static void start_prediction_worker(LandingController *c){
    pthread_mutex_lock(&c->mutex);
    /* ShuttleSim lockstep advances only after each applied guidance command.
       A wall-clock terminal-preview worker is therefore the wrong scheduler:
       at fast simulation rates hundreds of simulated seconds can elapse before
       a background proposal wins CPU time. Keep terminal planning synchronous
       with guidance_update() in simulator sessions. Live KSP retains the
       asynchronous worker so expensive searches stay off the control thread. */
    bool simulator=c->session&&krpc_session_is_simulator(c->session);
    if(!simulator&&!c->terminal_preview_thread_started){
        c->stop_terminal_preview_thread=false;
        if(pthread_create(&c->terminal_preview_thread,NULL,terminal_preview_worker,c)==0)
            c->terminal_preview_thread_started=true;
    }
    if(!c->prediction_thread_started){
        c->stop_prediction_thread=false;
        c->last_terminal_prediction_request_ut=c->last_terminal_prediction_completion_ut=-1e300;
        c->last_terminal_prediction_completion_wall=-1e300;
        c->last_terminal_prediction_solve_wall=0.0;
        if(pthread_create(&c->prediction_thread,NULL,terminal_prediction_worker,c)==0)
            c->prediction_thread_started=true;
    }
    pthread_mutex_unlock(&c->mutex);
}

static void stop_prediction_worker(LandingController *c){
    pthread_mutex_lock(&c->mutex);
    bool join=c->prediction_thread_started;
    bool join_preview=c->terminal_preview_thread_started;
    c->stop_prediction_thread=true;
    c->stop_terminal_preview_thread=true;
    /* The primary control thread is already joined before this function runs.
       Release KSP control now; never leave the last flight command applied while
       waiting several seconds for an advisory shadow prediction to finish. */
    if(c->session)krpc_safe(c->session);
    pthread_mutex_unlock(&c->mutex);
    if(join_preview)pthread_join(c->terminal_preview_thread,NULL);
    if(join)pthread_join(c->prediction_thread,NULL);
    pthread_mutex_lock(&c->mutex);
    c->prediction_thread_started=false;
    c->stop_prediction_thread=false;
    c->terminal_preview_thread_started=false;
    c->stop_terminal_preview_thread=false;
    c->last_terminal_prediction_request_ut=c->last_terminal_prediction_completion_ut=-1e300;
    c->last_terminal_prediction_completion_wall=-1e300;
    c->last_terminal_prediction_solve_wall=0.0;
    pthread_mutex_unlock(&c->mutex);
}

static const char* achieved_state_warning(LandingController*c,char*buf,size_t n){
    if(!c->has_plan||!c->plan.achieved_state_verified||c->plan.achieved_state_capture_qualified)return NULL;
    snprintf(buf,n,"Post-burn achieved-state verification is outside the planned entry corridor (actual Pe %.1f km). Entry guidance is continuing from live state; monitor the trajectory and be ready to take manual control.",c->plan.achieved_post_burn_periapsis_altitude/1000);
    return buf;
}

static const char *environment_warning(LandingController*c,const Telemetry*t,
        char*buf,size_t n){
    const PlanetModel*p=krpc_session_planet(c->session);
    if(strcasecmp(p->name,"Kerbin")){
        snprintf(buf,n,
            "The active planet is %s; guidance is using its live atmosphere/gravity model rather than Kerbin-specific geometric thresholds.",
            p->name);
        return buf;
    }
    if(t->physics_model_residual_confidence>0.0&&
       t->physics_model_residual>t->physics_certified_uncertainty){
        snprintf(buf,n,
            "Live aerodynamic response is outside the certified prior-flight model uncertainty (residual %.0f%%).",
            t->physics_model_residual*100.0);
        return buf;
    }
    if(t->stall_fraction_is_measured&&t->stall_fraction>=1.0){ /* decision-literal-ok: normalized stall boundary */
        snprintf(buf,n,"The measured aerodynamic state is at or beyond stall.");
        return buf;
    }
    return NULL;
}

static bool should_time_warp(const LandingController*c,const GuidanceResult*r,const Telemetry*t){
    if(c->time_warp_issued||!c->configuration.guidance.use_time_warp||r->phase!=PHASE_COAST||!c->has_plan||!c->guidance.automation_engaged||c->guidance.paused)return false;
    double burn_start=c->plan.burn_ut-c->plan.estimated_burn_duration*.5;
    double lead=fmax(0.0,c->configuration.guidance.minimum_planning_lead_time);
    double target=burn_start-lead;
    return target>t->ut+lead;
}

static bool perform_time_warp(LandingController*c,const Telemetry*t,char*err,size_t err_size){
    double burn_start=c->plan.burn_ut-c->plan.estimated_burn_duration*.5;
    double lead=fmax(0.0,c->configuration.guidance.minimum_planning_lead_time);
    double target=burn_start-lead;
    if(target<=t->ut+lead){c->time_warp_issued=true;return true;}
    krpc_safe(c->session);
    c->time_warp_issued=true;
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Time-warping to 100 seconds before the planned deorbit burn.");
    char reason[192];snprintf(reason,sizeof(reason),"event=time-warp-start; from_ut=%.9g; target_ut=%.9g",t->ut,target);log_state_event(c,reason,true);publish(c);
    if(!krpc_warp(c->session,target,err,err_size))return false;
    c->has_last_tick_ut=false;c->has_last_tick_wall=false;c->last_prediction_ut=-1e300;c->last_trajectory_ut=-1e300;
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Time warp complete. Reacquiring live state before burn setup.");
    snprintf(reason,sizeof(reason),"event=time-warp-complete; target_ut=%.9g",target);log_state_event(c,reason,true);publish(c);
    return true;
}

static bool apply_command_logged(LandingController*c,const GuidanceCommand*cmd,unsigned group,GuidancePhase phase,const char*status,const char*warning,const Trajectory*reference,KRPCApplyResult*out,double*started,double*finished,char*err,size_t err_size){
    *started=monotonic_seconds();
    const Trajectory*predicted=(phase==PHASE_ENTRY_INTERFACE||phase==PHASE_ENTRY_ENERGY||phase==PHASE_ATTITUDE_RECOVERY)?&c->stabilized_trajectory:NULL;
    bool ok=krpc_apply(c->session,cmd,group,phase_string(phase),status,warning,predicted,reference,out,err,err_size);*finished=monotonic_seconds();return ok;
}

static void tick_locked(LandingController*c){if(!c->session)return;char err[768]={0};double wall_start=monotonic_seconds();LandingConfiguration cfg=effective_config(c);double tele_start=monotonic_seconds();Telemetry t;telemetry_init(&t);VehicleState state={0};bool have_telemetry=false,apply_attempted=false,apply_succeeded=false;KRPCApplyResult applied={0};double apply_started=0,apply_finished=0,command_computed=0;if(!read_live_sample(c,&cfg,&t,&state,err,sizeof(err)))goto failure;have_telemetry=true;KRPCTransportBudget read_budget=krpc_session_transport_budget(c->session);t.has_rpc_budget=true;t.rpc_read_calls=read_budget.read_calls;t.rpc_read_wire_requests=read_budget.read_wire_requests;t.rpc_total_calls=read_budget.total_calls;t.rpc_total_wire_requests=read_budget.total_wire_requests;t.has_telemetry_latency=true;t.telemetry_latency_ms=(monotonic_seconds()-tele_start)*1000;double dt=c->has_last_tick_ut?clampd(t.ut-c->last_tick_ut,.01,1):.1;c->last_tick_ut=t.ut;c->has_last_tick_ut=true;if(c->has_last_tick_wall){t.has_loop_wall_delta=true;t.loop_wall_delta_ms=(wall_start-c->last_tick_wall)*1000;}c->last_tick_wall=wall_start;c->has_last_tick_wall=true;double compute=monotonic_seconds();
    c->tick_sequence++;c->snapshot.tick_sequence=c->tick_sequence;
    c->prediction_lineage_discontinuity=false;
    bool was_cal=c->glide.active,sampling=false,finished=false;double progress=0,target_aoa=0;GuidanceResult glide=glide_calibration_update(&c->glide,&t,&cfg.vehicle,&c->configuration.calibration,&sampling,&finished,&progress,&target_aoa);bool nominal_learning=!c->guidance.aborted&&!c->guidance.attitude_recovery&&c->snapshot.phase!=PHASE_ABORT&&c->snapshot.phase!=PHASE_FAULT&&c->snapshot.phase!=PHASE_ATTITUDE_RECOVERY;CalibrationSnapshot cal_snap;AerodynamicModel shadow_current,shadow_planning;AerodynamicEnvelope shadow_envelope;adaptive_calibrator_update(&c->adaptive,&t,&c->configuration.calibration,&c->configuration.vehicle,&c->last_command,dt,nominal_learning,c->glide.active,sampling,c->glide.state,progress,target_aoa,c->glide.state==CAL_IDLE?NULL:glide.status,glide.has_warning?glide.warning:NULL,&shadow_current,&shadow_planning,&shadow_envelope,&c->adaptive_profile,&cal_snap);(void)shadow_current;(void)shadow_planning;(void)shadow_envelope;if(!c->glide.active)snprintf(cal_snap.status,sizeof(cal_snap.status),"Shadow flight-test reconstruction: certified prior-flight aerodynamics remain frozen for this session.");c->aerodynamics=envelope_at_controller(&c->envelope,t.mach);c->trajectory_calibration=trajectory_calibrator_update(&c->trajectory_calibrator,&t,&state,krpc_session_planet(c->session),&cfg.site,&c->envelope,&cfg.vehicle,&c->last_command,&c->configuration.calibration,nominal_learning,dt);
    c->trajectory_calibration.physics=&c->physics;
    t.estimated_lift_to_drag=c->aerodynamics.lift_to_drag;t.estimated_ballistic_coefficient=c->aerodynamics.ballistic_coefficient;t.aerodynamic_confidence=c->aerodynamics.confidence;t.calibrated_best_glide_angle_of_attack=cal_snap.best_glide_angle_of_attack;t.calibrated_stall_speed=cal_snap.estimated_stall_speed;t.trajectory_density_scale=c->trajectory_calibration.density_scale;t.trajectory_drag_scale=c->trajectory_calibration.drag_scale;t.trajectory_lift_scale=c->trajectory_calibration.lift_scale;t.bank_effectiveness=c->trajectory_calibration.bank_effectiveness;t.trajectory_calibration_confidence=c->trajectory_calibration.confidence;t.trajectory_altitude_residual=c->trajectory_calibration.altitude_residual;t.trajectory_speed_residual=c->trajectory_calibration.speed_residual;t.trajectory_range_residual=c->trajectory_calibration.range_residual;t.predicted_miss_distance=c->has_plan?c->plan.predicted_closest_distance:0;t.predicted_taem_distance=c->has_plan?c->plan.predicted_taem_distance:0;t.predicted_taem_range_error=c->has_plan?c->plan.predicted_taem_range_error:0;const PlanetModel*planet=krpc_session_planet(c->session);double entry_ref=c->guidance.entry_reference_speed>0?c->guidance.entry_reference_speed:t.true_air_speed;EntryTerminalDemand local_demand=entry_terminal_demand(t.latitude,t.mean_altitude,t.range_to_site,t.horizontal_speed,t.course_to_site_error,t.vertical_speed,t.true_air_speed,entry_ref,planet,&cfg.vehicle,&cfg.site,&cfg.guidance);/* Preserve the historical sign: positive means excess downrange capability, negative means projected TAEM shortfall. */t.energy_excess_range=-local_demand.projected_taem_range_error;
    c->guidance.entry_planning_deferred=true;
    c->guidance.terminal_planning_deferred=
        !krpc_session_is_simulator(c->session)&&c->terminal_preview_thread_started;
    guidance_set_entry_predictor_models(&c->guidance,&c->envelope,&c->trajectory_calibration);
    if(c->has_plan&&!c->glide.active&&c->guidance.automation_engaged&&!c->guidance.has_burn_command_started&&!c->guidance.deorbit_burn_completed&&
       !deorbit_plan_preburn_state_compatible(&c->plan,t.mass,t.available_thrust,&cfg.guidance)){
        double mass_delta=c->plan.planning_mass>1?fabs(t.mass-c->plan.planning_mass)/c->plan.planning_mass:0;
        double thrust_delta=c->plan.planning_available_thrust>1?fabs(t.available_thrust-c->plan.planning_available_thrust)/c->plan.planning_available_thrust:0;
        c->plan_stale=true;
        if(mass_delta>cfg.guidance.deorbit_mass_uncertainty_fraction+1e-9)
            snprintf(c->guidance.abort_reason,sizeof(c->guidance.abort_reason),"Deorbit plan invalidated before burn: live vessel mass changed from %.0f kg to %.0f kg (%.1f%%), outside the certified +/-%.1f%% planning envelope. Replan from the current live state.",c->plan.planning_mass,t.mass,mass_delta*100,cfg.guidance.deorbit_mass_uncertainty_fraction*100);
        else
            snprintf(c->guidance.abort_reason,sizeof(c->guidance.abort_reason),"Deorbit plan invalidated before burn: available thrust changed from %.0f N to %.0f N (%.1f%%), outside the certified +/-%.1f%% planning envelope. Replan from the current live state.",c->plan.planning_available_thrust,t.available_thrust,thrust_delta*100,cfg.guidance.deorbit_thrust_uncertainty_fraction*100);
        char drift_reason[256];
        snprintf(drift_reason,sizeof(drift_reason),"event=plan-invalidated; reason=preburn-state-drift; mass=%.9g; planning-mass=%.9g; thrust=%.9g; planning-thrust=%.9g",t.mass,c->plan.planning_mass,t.available_thrust,c->plan.planning_available_thrust);
        log_state_event(c,drift_reason,true);
        guidance_abort(&c->guidance);
    }
    actual_trajectory_update(c,&t);dynamic_prediction(c,&t,&state,&cfg);c->latest_telemetry=t;c->latest_state=state;c->has_latest=true;bool using_glide=c->glide.active;GuidanceResult r=using_glide?glide:guidance_update(&c->guidance,&t,&state,c->has_plan?&c->plan:NULL,planet,c->aerodynamics,&cfg);if(using_glide)glide.reference=(Trajectory){0};t.has_guidance_compute=true;t.guidance_compute_ms=(monotonic_seconds()-compute)*1000;command_computed=monotonic_seconds();if(should_time_warp(c,&r,&t)){if(!perform_time_warp(c,&t,err,sizeof(err))){guidance_result_clear(&r);guidance_result_clear(&glide);goto failure;}guidance_result_clear(&r);guidance_result_clear(&glide);return;}GuidanceCommand cmd=r.command;c->last_command=cmd;const char*hud_status=using_glide?glide.status:r.status;const char*hud_warning=(using_glide?glide.has_warning:r.has_warning)?(using_glide?glide.warning:r.warning):"";if(cmd.autopilot_engaged&&!cmd.use_inertial_direction){t.has_command_pitch_error=t.has_command_roll_error=t.has_command_heading_error=true;t.command_pitch_error=cmd.has_target_aoa?cmd.target_aoa-t.angle_of_attack:cmd.target_pitch-t.pitch;t.command_roll_error=norm_signed_deg(cmd.target_roll-t.roll);t.command_heading_error=norm_signed_deg(cmd.target_heading-t.heading);}if(using_glide){apply_attempted=true;if(!apply_command_logged(c,&cmd,cfg.vehicle.airbrake_action_group,PHASE_CALIBRATION,hud_status,hud_warning,NULL,&applied,&apply_started,&apply_finished,err,sizeof(err))){guidance_result_clear(&r);guidance_result_clear(&glide);goto failure;}apply_succeeded=true;t.has_apply_latency=true;t.apply_latency_ms=(apply_finished-apply_started)*1000;}else if(was_cal&&finished)krpc_safe(c->session);else if(r.phase==PHASE_ABORT)krpc_safe(c->session);else if(c->guidance.automation_engaged&&!c->guidance.paused){apply_attempted=true;if(!apply_command_logged(c,&cmd,cfg.vehicle.airbrake_action_group,r.phase,hud_status,hud_warning,&r.reference,&applied,&apply_started,&apply_finished,err,sizeof(err))){guidance_result_clear(&r);guidance_result_clear(&glide);goto failure;}apply_succeeded=true;t.has_apply_latency=true;t.apply_latency_ms=(apply_finished-apply_started)*1000;}if(apply_attempted&&apply_succeeded){KRPCTransportBudget apply_budget=krpc_session_transport_budget(c->session);t.has_rpc_budget=true;t.rpc_apply_calls=apply_budget.apply_calls;t.rpc_apply_wire_requests=apply_budget.apply_wire_requests;t.rpc_total_calls=apply_budget.total_calls;t.rpc_total_wire_requests=apply_budget.total_wire_requests;}t.has_control_loop=true;t.control_loop_ms=(monotonic_seconds()-wall_start)*1000;c->latest_telemetry=t;
    /* dynamic_prediction() intentionally runs before guidance_update() because MM304
       consumes its prediction telemetry. If guidance accepted a different executable
       plan on this tick, do not block the next control cycle with a second atmospheric
       solve. Suppress this transition frame's unmatched path, prevent the next tick's
       calibrator from learning against the predecessor plan, and force an immediate refit. */
    if(!using_glide&&!entry_forecast_matches_guidance(c)){
        c->prediction_lineage_discontinuity=true;
        c->last_prediction_ut=-1e300;
        Trajectory no_forecast={0};trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&no_forecast);
        if(c->guidance.phase!=PHASE_ENTRY_ENERGY){
            c->forecast_entry_plan_identity_valid=false;c->forecast_entry_plan_id=0;c->forecast_entry_plan_version=0;
            trajectory_clear(&c->dynamic_trajectory);trajectory_clear(&c->stabilized_trajectory);
        }
    }
    c->snapshot.guidance_terminal_prediction_valid=c->guidance.terminal_prediction_valid;
    c->snapshot.guidance_terminal_candidate_valid=c->guidance.terminal_candidate.valid;
    c->snapshot.guidance_terminal_committed=c->guidance.terminal_path_committed;
    c->snapshot.guidance_terminal_candidate_radius=c->guidance.terminal_candidate.radius;
    c->snapshot.guidance_terminal_candidate_altitude=c->guidance.terminal_candidate.altitude;
    c->snapshot.guidance_terminal_candidate_speed=c->guidance.terminal_candidate.speed;
    c->snapshot.guidance_terminal_candidate_kind=c->guidance.terminal_candidate.kind;
    c->snapshot.guidance_terminal_candidate_geometry_degraded=c->guidance.terminal_candidate.geometry_degraded;
    c->snapshot.guidance_terminal_candidate_energy_degraded=c->guidance.terminal_candidate.energy_degraded;
    c->snapshot.guidance_terminal_candidate_shell_degraded=c->guidance.terminal_candidate.shell_degraded;
    c->snapshot.guidance_terminal_candidate_side=c->guidance.terminal_candidate.side;
    c->snapshot.guidance_terminal_candidate_slope=c->guidance.terminal_candidate.slope;
    c->snapshot.guidance_terminal_candidate_curve_length=c->guidance.terminal_candidate.join.length;
    c->snapshot.guidance_terminal_candidate_lead_length=c->guidance.terminal_candidate.join.lead_length;
    c->snapshot.guidance_terminal_candidate_arc_remaining=c->guidance.terminal_candidate.join.arc_remaining;
    c->snapshot.guidance_terminal_candidate_final_distance=c->guidance.terminal_candidate.final_distance;
    c->snapshot.guidance_terminal_candidate_quality=c->guidance.terminal_candidate.quality_score;
    c->snapshot.guidance_terminal_candidate_arrival_ut=c->guidance.terminal_candidate.arrival_ut;
    c->snapshot.guidance_terminal_candidate_course=c->guidance.terminal_candidate.course;
    c->snapshot.guidance_terminal_candidate_peak_lateral=c->guidance.terminal_candidate.join.peak_lateral;
    c->snapshot.guidance_terminal_candidate_peak_rate_ratio=c->guidance.terminal_candidate.join.peak_course_rate_ratio;
    c->snapshot.guidance_terminal_candidate_exit_speed=c->guidance.terminal_candidate.join.exit_speed;
    c->snapshot.guidance_terminal_candidate_path_degraded=c->guidance.taem_safety_handoff||c->guidance.terminal_candidate.join.degraded_path;
    c->snapshot.guidance_terminal_candidate_control_degraded=c->guidance.terminal_candidate.join.degraded_control;
    c->snapshot.guidance_terminal_candidate_rate_degraded=c->guidance.terminal_candidate.join.degraded_rate;
    c->snapshot.guidance_terminal_candidate_end_degraded=c->guidance.terminal_candidate.join.degraded_end;
    c->snapshot.guidance_terminal_reference_fpa=c->guidance.terminal_reference_fpa;
    c->snapshot.guidance_terminal_mix=c->guidance.terminal_mix;
    c->snapshot.tick_sequence=c->tick_sequence;c->snapshot.log_sequence=++c->log_sequence;c->snapshot.wall_monotonic_seconds=wall_start;c->snapshot.simulation_dt=dt;c->snapshot.has_vehicle_state=true;c->snapshot.vehicle_state=state;c->snapshot.has_attitude_quaternion=t.has_attitude_quaternion;if(t.has_attitude_quaternion)memcpy(c->snapshot.attitude_quaternion,t.attitude_quaternion,sizeof(t.attitude_quaternion));snprintf(c->snapshot.attitude_reference_frame,sizeof(c->snapshot.attitude_reference_frame),"%s",t.attitude_reference_frame);c->snapshot.command_computed_wall_seconds=command_computed;c->snapshot.command_apply_attempted=apply_attempted;c->snapshot.command_apply_started_wall_seconds=apply_started;c->snapshot.command_apply_finished_wall_seconds=apply_finished;c->snapshot.command_applied=apply_succeeded&&applied.applied;c->snapshot.applied_command=applied;c->snapshot.guidance_has_burn_started=c->guidance.has_burn_command_started;c->snapshot.guidance_burn_completed=c->guidance.deorbit_burn_completed;c->snapshot.guidance_atmosphere_crossed=c->guidance.atmospheric_interface_crossed;c->snapshot.guidance_final_captured=c->guidance.final_approach_captured;c->snapshot.guidance_airbrakes_deployed=c->guidance.airbrakes_deployed;c->snapshot.guidance_hac_side_selected=c->guidance.hac_side_selected;c->snapshot.guidance_delivered_delta_v=c->guidance.delivered_delta_v;c->snapshot.guidance_burn_active_elapsed=c->guidance.burn_active_elapsed;c->snapshot.guidance_s_turn_sign=c->guidance.s_turn_sign;c->snapshot.guidance_hac_side=c->guidance.hac_side;c->snapshot.guidance_entry_leg_elapsed=guidance_entry_leg_elapsed(&c->guidance,t.ut);c->snapshot.guidance_entry_exec=entry_exec_telemetry(&c->guidance.entry_exec);c->snapshot.guidance_taem_exec=taem_exec_telemetry(&c->guidance.taem_exec);c->snapshot.guidance_hac_captured=c->guidance.hac_captured;c->snapshot.guidance_hac_completed=c->guidance.hac_completed;c->snapshot.guidance_hac_progress_valid=c->guidance.hac_progress_valid;c->snapshot.guidance_hac_transition_active=c->guidance.hac_transition_active;c->snapshot.guidance_hac_remaining=c->guidance.hac_remaining;c->snapshot.guidance_hac_radius=c->guidance.hac_radius;c->snapshot.guidance_hac_circuit_count=c->guidance.hac_circuit_count;c->snapshot.guidance_hac_circuit_slope=c->guidance.hac_circuit_slope;c->snapshot.guidance_terminal_reentry_after_ut=c->guidance.terminal_reentry_after_ut;c->snapshot.guidance_minimum_turn_radius=c->guidance.minimum_turn_radius;c->snapshot.guidance_hac_transition_progress=c->guidance.hac_transition_progress;c->snapshot.guidance_hac_transition_points[0]=c->guidance.hac_transition_p0_e;c->snapshot.guidance_hac_transition_points[1]=c->guidance.hac_transition_p0_n;c->snapshot.guidance_hac_transition_points[2]=c->guidance.hac_transition_p1_e;c->snapshot.guidance_hac_transition_points[3]=c->guidance.hac_transition_p1_n;c->snapshot.guidance_hac_transition_points[4]=c->guidance.hac_transition_p2_e;c->snapshot.guidance_hac_transition_points[5]=c->guidance.hac_transition_p2_n;c->snapshot.guidance_hac_transition_points[6]=c->guidance.hac_transition_p3_e;c->snapshot.guidance_hac_transition_points[7]=c->guidance.hac_transition_p3_n;double lift_accel=t.mass>1&&isfinite(t.lift_force)&&t.lift_force>0?t.lift_force/t.mass:0;double bank_for_rate=clampd(norm_signed_deg(cmd.target_roll)*clampd(t.bank_effectiveness,.35,1.8),-89,89);c->snapshot.guidance_commanded_course_rate_estimate=t.true_air_speed>1&&lift_accel>0?lift_accel*sin(bank_for_rate*DEG2RAD)/t.true_air_speed*RAD2DEG:0;
    c->snapshot.guidance_entry_plan_valid=c->guidance.entry_s_turn_plan.valid;
    c->snapshot.guidance_entry_plan_terminal_ready=c->guidance.entry_s_turn_plan.valid&&c->guidance.entry_s_turn_plan.terminal_ready;
    c->snapshot.guidance_entry_plan_ut=c->guidance.entry_s_turn_plan.planned_ut;
    c->snapshot.guidance_entry_plan_target_bank=c->guidance.entry_s_turn_plan.target_bank;
    c->snapshot.guidance_entry_plan_target_aoa=c->guidance.entry_s_turn_plan.target_aoa;
    c->snapshot.guidance_entry_plan_target_heading=c->guidance.entry_s_turn_plan.target_heading;
    c->snapshot.guidance_entry_plan_segment_remaining=c->guidance.entry_s_turn_plan.valid?
        fmax(0.0,c->guidance.entry_s_turn_plan.planned_ut+c->guidance.entry_s_turn_plan.segment_duration-t.ut):0.0;
    c->snapshot.guidance_entry_plan_cost=c->guidance.entry_s_turn_plan.cost;
    c->snapshot.guidance_entry_plan_taem_range_error=c->guidance.entry_s_turn_plan.taem_range_error;
    c->snapshot.guidance_entry_plan_taem_speed=c->guidance.entry_s_turn_plan.taem_speed;
    c->snapshot.guidance_entry_plan_taem_energy_error=c->guidance.entry_s_turn_plan.taem_energy_error;
    c->snapshot.guidance_entry_plan_predicted_reversals=c->guidance.entry_s_turn_plan.predicted_reversals;
    c->snapshot.guidance_entry_reversal_scheduled=c->guidance.entry_reversal_scheduled;
    c->snapshot.guidance_entry_reversal_is_final=c->guidance.entry_reversal_is_final;
    c->snapshot.guidance_entry_reversal_time_remaining=c->guidance.entry_reversal_scheduled?c->guidance.entry_reversal_ut-t.ut:0.0;
    c->snapshot.guidance_entry_reversal_range=c->guidance.entry_reversal_range;
    c->snapshot.guidance_entry_reversal_sign=c->guidance.entry_reversal_sign;
    c->snapshot.connection_status=CONN_CONNECTED;c->snapshot.phase=using_glide?PHASE_CALIBRATION:r.phase;landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);c->snapshot.command=cmd;c->snapshot.status_message[0]=0;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s",using_glide?r.status:r.status);snprintf(c->snapshot.decision_reason,sizeof(c->snapshot.decision_reason),"phase=%s; profile=%s; automation=%s; paused=%s; burn_started=%s; burn_completed=%s; atmosphere_crossed=%s; final_captured=%s; delivered_dv=%.9g; entry_leg_elapsed=%.9g",phase_string(c->snapshot.phase),profile_string(cmd.control_profile),c->guidance.automation_engaged?"true":"false",c->guidance.paused?"true":"false",c->guidance.has_burn_command_started?"true":"false",c->guidance.deorbit_burn_completed?"true":"false",c->guidance.atmospheric_interface_crossed?"true":"false",c->guidance.final_approach_captured?"true":"false",c->guidance.delivered_delta_v,c->snapshot.guidance_entry_leg_elapsed);replace_reference_trajectory(c,&r.reference);replace_predicted_trajectory(c,&c->stabilized_trajectory);const char*warning=r.has_warning?r.warning:achieved_state_warning(c,err,sizeof(err));if(!warning&&c->has_plan&&c->plan.execution_degraded&&c->guidance.automation_engaged&&!c->guidance.deorbit_burn_completed)warning="Guarded recovery plan is active because the preferred strict corridor was not available; live cutoff and achieved-state verification remain armed.";if(!warning)warning=environment_warning(c,&t,err,sizeof(err));set_warning(c,warning);set_error(c,NULL);c->snapshot.automation_engaged=c->guidance.automation_engaged;c->snapshot.paused=c->guidance.paused;c->snapshot.calibration=cal_snap;c->snapshot.adaptive_vehicle_profile=c->adaptive_profile;c->snapshot.plan=c->has_plan?&c->plan:NULL;append_log(c);publish(c);guidance_result_clear(&r);guidance_result_clear(&glide);return;
failure:
    krpc_safe(c->session);guidance_abort(&c->guidance);glide_calibration_stop(&c->glide,"Calibration stopped after a kRPC or control error.");c->snapshot.tick_sequence=c->tick_sequence;c->snapshot.log_sequence=++c->log_sequence;c->snapshot.wall_monotonic_seconds=wall_start;c->snapshot.command_computed_wall_seconds=command_computed;c->snapshot.command_apply_attempted=apply_attempted;c->snapshot.command_apply_started_wall_seconds=apply_started;c->snapshot.command_apply_finished_wall_seconds=apply_finished;c->snapshot.command_applied=false;c->snapshot.applied_command=applied;if(have_telemetry){landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);c->snapshot.tick_sequence=c->tick_sequence;c->snapshot.has_vehicle_state=true;c->snapshot.vehicle_state=state;c->snapshot.has_attitude_quaternion=t.has_attitude_quaternion;if(t.has_attitude_quaternion)memcpy(c->snapshot.attitude_quaternion,t.attitude_quaternion,sizeof(t.attitude_quaternion));snprintf(c->snapshot.attitude_reference_frame,sizeof(c->snapshot.attitude_reference_frame),"%s",t.attitude_reference_frame);}c->snapshot.phase=PHASE_FAULT;c->snapshot.connection_status=CONN_FAILED;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Guidance stopped after a kRPC or control error.");snprintf(c->snapshot.decision_reason,sizeof(c->snapshot.decision_reason),"event=fault; telemetry_available=%s; command_apply_attempted=%s; error=%s",have_telemetry?"true":"false",apply_attempted?"true":"false",err[0]?err:"unknown");set_error(c,err[0]?err:"Unknown kRPC or control failure");c->snapshot.automation_engaged=false;c->snapshot.paused=false;c->snapshot.calibration.active=false;(void)append_vehicle_log(c,c->snapshot.decision_reason,true,false);append_planner_log(c,c->snapshot.decision_reason,true);durable_flush_log(c);publish(c);
}

static void *control_thread(void*arg){
    LandingController*c=arg;
    for(;;){
        double start=monotonic_seconds();
        pthread_mutex_lock(&c->mutex);
        if(c->stop_thread){pthread_mutex_unlock(&c->mutex);break;}
        tick_locked(c);
        double hz=clampd(c->configuration.guidance.guidance_rate,2,30);
        bool fast_sim=c->session&&krpc_session_is_simulator(c->session)&&
            c->guidance.automation_engaged&&!c->guidance.paused;
        /* ShuttleSim can advance hundreds of simulated seconds per wall second,
           while the production entry topology worker intentionally solves outside
           the controller mutex.  Without a barrier, MM304 can traverse its entire
           S-turn before one asynchronous topology result has a chance to return.
           At a planning boundary, freeze only simulator lockstep (not Guidance or
           the worker) until the request has completed. Live KSP remains fully
           asynchronous and unchanged. */
        bool sim_plan_barrier=fast_sim&&c->guidance.phase==PHASE_ENTRY_ENERGY&&
            c->guidance.entry_planning_needed&&!c->entry_worker_trace_valid;
        double barrier_ut=c->has_latest?c->latest_telemetry.ut:-INFINITY;
        double previous_completion=c->last_terminal_prediction_completion_ut;
        uint64_t barrier_generation=c->prediction_generation;
        pthread_mutex_unlock(&c->mutex);

        if(sim_plan_barrier){
            double deadline=monotonic_seconds()+15.0;
            for(;;){
                sleep_seconds(.001);
                pthread_mutex_lock(&c->mutex);
                bool stop=c->stop_thread||!c->session||
                    !krpc_session_is_simulator(c->session)||!c->guidance.automation_engaged||
                    c->guidance.paused||c->guidance.phase!=PHASE_ENTRY_ENERGY||
                    c->prediction_generation!=barrier_generation;
                bool completed=c->last_terminal_prediction_completion_ut>=barrier_ut-1e-6&&
                    (c->last_terminal_prediction_completion_ut>previous_completion+1e-6||
                     c->last_terminal_prediction_completion_wall>start);
                bool settled=!c->guidance.entry_planning_needed&&
                    !c->guidance.entry_final_reversal_pending;
                pthread_mutex_unlock(&c->mutex);
                if(stop||completed||settled||monotonic_seconds()>=deadline)break;
            }
        }else if(!fast_sim){
            sleep_seconds(fmax(0,1.0/hz-(monotonic_seconds()-start)));
        }
    }
    return NULL;
}
static void start_thread(LandingController*c){
    pthread_mutex_lock(&c->mutex);
    if(!c->thread_started){
        c->stop_thread=false;
        if(pthread_create(&c->thread,NULL,control_thread,c)==0)c->thread_started=true;
    }
    pthread_mutex_unlock(&c->mutex);
    start_prediction_worker(c);
}

LandingController *landing_controller_create(const LandingConfiguration*cfg,SnapshotCallback cb,void*ctx){LandingController*c=calloc(1,sizeof(*c));if(!c)return NULL;pthread_mutex_init(&c->mutex,NULL);c->configuration=cfg?*cfg:landing_configuration_default();landing_configuration_normalize(&c->configuration);guidance_machine_init(&c->guidance);adaptive_calibrator_init(&c->adaptive,&c->configuration.vehicle);glide_calibration_init(&c->glide);trajectory_calibrator_init(&c->trajectory_calibrator);trajectory_init(&c->dynamic_trajectory);trajectory_init(&c->stabilized_trajectory);landing_snapshot_init(&c->snapshot,&c->configuration.vehicle);c->adaptive_profile=c->configuration.vehicle;c->aerodynamics=(AerodynamicModel){c->configuration.vehicle.estimated_lift_to_drag,c->configuration.vehicle.estimated_ballistic_coefficient,.2};c->planning_aerodynamics=c->aerodynamics;for(int i=0;i<4;i++)c->envelope.regimes[i]=c->aerodynamics;c->trajectory_calibration=(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0};guidance_command_init(&c->last_command);c->last_trajectory_ut=c->last_prediction_ut=c->cached_prediction_ut=-1e300;c->callback=cb;c->callback_context=ctx;return c;}
void landing_controller_destroy(LandingController*c){if(!c)return;landing_controller_shutdown(c);pthread_mutex_lock(&c->mutex);clear_snapshot_dynamic(&c->snapshot);trajectory_clear(&c->dynamic_trajectory);trajectory_clear(&c->stabilized_trajectory);trajectory_calibrator_clear(&c->trajectory_calibrator);if(c->has_plan)deorbit_plan_clear(&c->plan);pthread_mutex_unlock(&c->mutex);pthread_mutex_destroy(&c->mutex);free(c);}
LandingConfiguration landing_controller_configuration(LandingController*c){pthread_mutex_lock(&c->mutex);LandingConfiguration r=c->configuration;pthread_mutex_unlock(&c->mutex);return r;}
void landing_controller_update_configuration(LandingController*c,const LandingConfiguration*cfg){pthread_mutex_lock(&c->mutex);c->prediction_generation++;LandingConfiguration before=c->configuration,next=*cfg;VehicleProfile before_vehicle=before.vehicle;landing_configuration_normalize(&next);bool changed=!configuration_equal(&before,&next),planning_changed=!planning_configuration_equal(&before,&next);if(!changed){pthread_mutex_unlock(&c->mutex);return;}c->configuration=next;if(!vehicle_profile_equal(&before_vehicle,&c->configuration.vehicle)&&!c->glide.active){reset_models(c,&c->configuration.vehicle);if(c->session){krpc_session_seed_physics(c->session,&c->physics);apply_certified_aero_prior(c);}}if(c->has_plan&&planning_changed){c->plan_stale=true;set_warning(c,"Planning-relevant configuration changed after planning. Create a new deorbit plan before engagement.");}append_configuration_log(c);log_state_event(c,planning_changed?"event=configuration-changed; replan_required=true":"event=configuration-changed; replan_required=false",true);publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_connect(LandingController*c){stop_thread(c);pthread_mutex_lock(&c->mutex);disconnect_locked(c,false);reset_snapshot(c,getenv("KSP_LANDER_SIMULATOR")?"Opening ShuttleSim lockstep backend…":"Opening native kRPC C-Nano serial link…");c->snapshot.connection_status=CONN_CONNECTING;publish(c);LandingConfiguration cfg=c->configuration;pthread_mutex_unlock(&c->mutex);char err[768];KRPCSession*s=krpc_session_open(&cfg,err,sizeof(err));pthread_mutex_lock(&c->mutex);if(!s){c->snapshot.connection_status=CONN_FAILED;c->snapshot.phase=PHASE_FAULT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Native kRPC C-Nano connection failed.");set_error(c,err);publish(c);pthread_mutex_unlock(&c->mutex);return;}c->session=s;start_log(c);reset_models(c,&c->configuration.vehicle);krpc_session_seed_physics(s,&c->physics);apply_certified_aero_prior(c);guidance_machine_init(&c->guidance);glide_calibration_init(&c->glide);c->has_prediction_cache=false;c->time_warp_issued=false;c->snapshot.connection_status=CONN_CONNECTED;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s connected to %s around %s with %u certified aerodynamic cells.",krpc_session_is_simulator(s)?"ShuttleSim lockstep":"Native kRPC C-Nano",krpc_session_vessel(s),krpc_session_planet(s)->name,c->physics.count);set_error(c,NULL);c->snapshot.adaptive_vehicle_profile=c->adaptive_profile;publish(c);pthread_mutex_unlock(&c->mutex);start_thread(c);}
void landing_controller_disconnect(LandingController*c){stop_thread(c);pthread_mutex_lock(&c->mutex);disconnect_locked(c,true);pthread_mutex_unlock(&c->mutex);}

void landing_controller_create_plan(LandingController*c){pthread_mutex_lock(&c->mutex);if(!c->session){pthread_mutex_unlock(&c->mutex);return;}if(c->glide.active){set_warning(c,"Stop the glide calibration before creating a deorbit plan.");log_state_event(c,"event=planning-blocked; reason=calibration-active",false);publish(c);pthread_mutex_unlock(&c->mutex);return;}LandingConfiguration cfg=effective_config(c);char err[768];Telemetry t;VehicleState s;if(!read_live_sample(c,&cfg,&t,&s,err,sizeof(err))){c->snapshot.phase=PHASE_FAULT;set_error(c,err);log_state_event(c,"event=planning-failed; reason=telemetry-read",true);publish(c);pthread_mutex_unlock(&c->mutex);return;}c->latest_telemetry=t;c->latest_state=s;c->has_latest=true;landing_snapshot_set_sample(&c->snapshot,&t,&s);c->snapshot.phase=PHASE_PLANNING;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Searching upcoming orbital landing opportunities and stress-testing burn execution…");set_error(c,NULL);publish(c);const PlanetModel*planet=krpc_session_planet(c->session);if(t.apoapsis_altitude<=planet->atmosphere_depth||t.periapsis_altitude<=planet->atmosphere_depth){c->snapshot.phase=PHASE_FAULT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Planning rejected.");set_error(c,"Initial apoapsis and periapsis must both be outside the active planet atmosphere.");log_state_event(c,"event=planning-failed; reason=orbit-inside-atmosphere",true);publish(c);pthread_mutex_unlock(&c->mutex);return;}DeorbitPlan p;if(!deorbit_plan_create(&p,s,t.orbit_period,krpc_session_planet(c->session),c->planning_aerodynamics,&c->envelope,&c->trajectory_calibration,&cfg.site,&cfg.vehicle,&cfg.guidance,t.available_thrust)){c->snapshot.phase=PHASE_FAULT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"No usable deorbit solution was found.");set_error(c,"The current orbit, landing site, thrust or calibrated aerodynamic profile did not produce atmospheric capture across the searched future opportunities.");log_state_event(c,"event=planning-failed; reason=no-executable-deorbit-solution",true);publish(c);pthread_mutex_unlock(&c->mutex);return;}if(c->has_plan)deorbit_plan_clear(&c->plan);c->plan=p;c->has_plan=true;c->plan_stale=false;c->time_warp_issued=false;c->log_plan_trajectory_dirty=true;trajectory_clear(&c->dynamic_trajectory);trajectory_copy(&c->dynamic_trajectory,&p.trajectory);trajectory_clear(&c->stabilized_trajectory);trajectory_copy(&c->stabilized_trajectory,&p.trajectory);guidance_reset_plan(&c->guidance);t.predicted_miss_distance=p.predicted_closest_distance;t.predicted_entry_range=p.predicted_entry_range;t.predicted_entry_flight_path_angle=p.predicted_entry_flight_path_angle;trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&p.trajectory);landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);c->snapshot.phase=PHASE_IDLE;if(p.target_capture_achieved)snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Robust plan ready. %u/%u strict stress cases pass (%.0f%%).",p.robustness_passed,p.robustness_scenarios,p.robustness_pass_fraction*100);else if(p.execution_qualified)snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Guarded recovery plan ready. %u/%u recovery cases pass (%.0f%%); strict corridor was not met.",p.recovery_passed,p.robustness_scenarios,p.recovery_pass_fraction*100);else snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Preview plan found, but no execution-qualified corridor was found.");set_warning(c,p.target_capture_achieved?NULL:p.execution_qualified?"This plan is executable under the broader guarded recovery envelope. Review its miss, entry angle, periapsis and recovery stress margin before engaging.":"Engagement is disabled for this preview plan. Replan for a later opportunity or revise the vehicle/site model.");set_error(c,NULL);reference_trajectory(&c->snapshot.reference_trajectory,&cfg.site,&cfg.guidance,krpc_session_planet(c->session)->radius,1);c->log_reference_trajectory_dirty=true;char plan_reason[256];snprintf(plan_reason,sizeof(plan_reason),"event=plan-created; execution-qualified=%s; strict-capture=%s; degraded=%s; robustness-pass=%.6g",p.execution_qualified?"true":"false",p.target_capture_achieved?"true":"false",p.execution_degraded?"true":"false",p.robustness_pass_fraction);log_state_event(c,plan_reason,true);publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_engage(LandingController*c){pthread_mutex_lock(&c->mutex);c->prediction_generation++;if(c->glide.active)set_warning(c,"Stop calibration before engaging landing guidance.");else if(!c->has_plan)set_warning(c,"Create and review a deorbit plan before engaging guidance.");else if(c->plan_stale)set_warning(c,"Engagement blocked because planning-relevant configuration changed. Create a new deorbit plan.");else if(!c->plan.execution_qualified)set_warning(c,"Engagement blocked because this plan is preview-only: neither the strict capture gate nor the guarded recovery envelope is satisfied. Replan for a later orbital opportunity.");else if(c->has_latest&&c->session&&c->latest_telemetry.ut>c->plan.burn_ut+c->plan.estimated_burn_duration*.5&&c->latest_telemetry.mean_altitude>krpc_session_planet(c->session)->atmosphere_depth)set_warning(c,"Engagement blocked because the planned deorbit-burn window has passed. Create a new plan.");else{guidance_set_engaged(&c->guidance,true);c->snapshot.automation_engaged=true;c->snapshot.paused=false;set_warning(c,c->plan.execution_degraded?"Guarded recovery execution engaged. Live cutoff and achieved-state verification will supervise the burn and entry handoff.":NULL);}publish(c);pthread_mutex_unlock(&c->mutex);}

void landing_controller_engage_reentry(LandingController*c){
    pthread_mutex_lock(&c->mutex);
    c->prediction_generation++;

    if(!c->session){
        set_warning(c,"Connect to KSP before starting reentry-only guidance.");
        publish(c);pthread_mutex_unlock(&c->mutex);return;
    }
    if(c->glide.active){
        set_warning(c,"Stop calibration before starting reentry-only guidance.");
        publish(c);pthread_mutex_unlock(&c->mutex);return;
    }

    LandingConfiguration cfg=effective_config(c);
    char err[768]={0};
    Telemetry t;
    VehicleState state;
    if(!read_live_sample(c,&cfg,&t,&state,err,sizeof(err))){
        set_error(c,err);c->snapshot.phase=PHASE_FAULT;
        publish(c);pthread_mutex_unlock(&c->mutex);return;
    }

    const PlanetModel*p=krpc_session_planet(c->session);
    bool airborne=strcasecmp(t.vessel_situation,"landed")&&
        strcasecmp(t.vessel_situation,"splashed")&&
        strcasecmp(t.vessel_situation,"pre-launch");
    bool in_atmosphere=planet_atmospheric_density(p,t.mean_altitude)>DBL_EPSILON;
    bool descending=t.vertical_speed<0.0;

    if(!airborne||(!descending&&in_atmosphere)){
        set_warning(c,
            "Reentry continuation requires an airborne vehicle already descending once atmospheric forces are present.");
        publish(c);pthread_mutex_unlock(&c->mutex);return;
    }

    if(!in_atmosphere){
        bool periapsis_enters_atmosphere=isfinite(t.periapsis_altitude)&&
            t.periapsis_altitude>cfg.site.altitude&&
            t.periapsis_altitude<p->atmosphere_depth;
        if(!periapsis_enters_atmosphere){
            set_warning(c,
                "Reentry continuation requires an osculating periapsis inside the atmosphere and above the surface.");
            publish(c);pthread_mutex_unlock(&c->mutex);return;
        }
    }

    double initial_roll=norm_signed_deg(t.roll);
    double initial_s_turn_sign;
    if(fabs(initial_roll)>DBL_EPSILON){
        initial_s_turn_sign=initial_roll<0.0?-1.0:1.0;
    }else if(fabs(t.runway_cross_track)>DBL_EPSILON){
        initial_s_turn_sign=t.runway_cross_track>0.0?-1.0:1.0;
    }else{
        double course_error=norm_signed_deg(
            t.ground_track_heading-cfg.site.runway_heading);
        initial_s_turn_sign=course_error<0.0?-1.0:1.0;
    }

    GuidanceMachine qualification_guidance;
    guidance_initialize_reentry_continuation(&qualification_guidance,&t,p,&cfg,
        initial_s_turn_sign,false,&c->envelope,&c->trajectory_calibration);

    /* Keep synchronous re-entry qualification on the same MM304 contract as
       the worker.  The topology solver consumes the published target when it
       is available; otherwise it has to fall back to a generic perpendicular
       outlet that may not match the worker's turnability-selected target. */
    double qualification_course=isfinite(t.ground_track_heading)?
        t.ground_track_heading:t.heading;
    entry_publish_taem_tangent_target(&qualification_guidance,&t,
        qualification_course,p,c->planning_aerodynamics,&cfg);

    EntryTopologyPlan topology=predictor_plan_entry_topology(state,&t,
        &qualification_guidance,p,&c->envelope,&c->trajectory_calibration,&cfg);
    if(topology.valid)
        (void)guidance_install_entry_topology(
            &qualification_guidance,&topology,t.ut);

    DeorbitPlan qualification_plan;
    memset(&qualification_plan,0,sizeof(qualification_plan));
    trajectory_init(&qualification_plan.trajectory);
    qualification_plan.created_ut=t.ut;
    qualification_plan.burn_ut=t.ut;
    qualification_plan.predicted_post_burn_periapsis_altitude=
        isfinite(t.periapsis_altitude)?t.periapsis_altitude:cfg.site.altitude;
    qualification_plan.live_cutoff_capture_qualified=true;
    qualification_plan.execution_qualified=true;
    qualification_plan.achieved_state_verified=true;
    qualification_plan.achieved_state_capture_qualified=true;

    double gravity=planet_surface_gravity(p);
    double height=fmax(0.0,t.mean_altitude-cfg.site.altitude);
    double horizontal_time=t.horizontal_speed>DBL_EPSILON?
        fmax(0.0,t.range_to_site)/t.horizontal_speed:0.0;
    double vertical_time=(-t.vertical_speed)>DBL_EPSILON?
        height/(-t.vertical_speed):
        (gravity>DBL_EPSILON?sqrt(2.0*height/gravity):0.0);
    double terminal_path=2.0*LANDER_PI*cfg.guidance.hac_radius+
        cfg.guidance.final_approach_distance;
    double terminal_time=terminal_path/
        fmax(cfg.vehicle.minimum_safe_speed,DBL_EPSILON);
    double qualification_horizon=fmax(horizontal_time,vertical_time)+terminal_time;

    EntryPrediction prediction=predictor_simulate_entry_guidance_shadow(
        state,&t,&qualification_guidance,&qualification_plan,
        p,c->planning_aerodynamics,&c->envelope,&c->trajectory_calibration,
        &cfg,qualification_horizon,true);
    trajectory_clear(&qualification_plan.trajectory);

    bool recovery=reentry_guidance_shadow_recovery_qualified(
        &prediction,&cfg.vehicle,&cfg.guidance);
    if(!recovery){
        char msg[768];
        snprintf(msg,sizeof(msg),
            "Reentry rejected: entered=%s, TAEM=%s, closest=%.1f km, range error=%.1f km, q=%.1f kPa, g=%.2f; topology=%s flags=0x%x bank=%.1f/%.1f AoA=%.1f/%.1f termUT=%.1f s geom=%.1f km capUT=%.1f s pos=%.1f km course=%.1f deg.",
            prediction.entered_atmosphere?"true":"false",
            prediction.reached_taem?"true":"false",
            prediction.closest_distance/1000.0,
            prediction.taem_range_error/1000.0,
            prediction.peak_dynamic_pressure/1000.0,
            prediction.peak_g_load,
            topology.valid?"valid":"invalid",topology.failure_flags,
            topology.first_bank,topology.turn_bank,
            topology.first_aoa,topology.turn_aoa,
            topology.terminal_turn_ut-state.ut,
            topology.terminal_turn_geometry_error/1000.0,
            topology.capture_ut-state.ut,
            topology.position_error/1000.0,
            topology.capture_course_error);
        entry_prediction_clear(&prediction);
        set_warning(c,msg);publish(c);pthread_mutex_unlock(&c->mutex);return;
    }

    DeorbitPlan plan;
    memset(&plan,0,sizeof(plan));
    trajectory_init(&plan.trajectory);
    plan.created_ut=t.ut;
    plan.burn_ut=t.ut;
    plan.delta_v=0.0;
    plan.estimated_burn_duration=0.0;
    plan.predicted_taem_distance=prediction.taem_distance;
    plan.predicted_taem_range_error=prediction.taem_range_error;
    plan.predicted_closest_distance=prediction.closest_distance;
    plan.predicted_entry_range=prediction.entry_range;
    plan.predicted_entry_flight_path_angle=prediction.entry_flight_path_angle;
    plan.predicted_post_burn_periapsis_altitude=t.periapsis_altitude;
    plan.nominal_capture_achieved=true;
    plan.robustness_qualified=true;
    plan.target_capture_achieved=true;
    plan.robustness_scenarios=1;
    plan.robustness_passed=1;
    plan.robustness_unsafe=0;
    plan.robustness_pass_fraction=1.0;
    plan.execution_qualified=true;
    plan.execution_degraded=false;
    plan.recovery_passed=1;
    plan.recovery_pass_fraction=1.0;
    plan.worst_case_closest_distance=prediction.closest_distance;
    plan.worst_case_taem_range_error=fabs(prediction.taem_range_error);
    plan.worst_case_entry_flight_path_angle=prediction.entry_flight_path_angle;
    plan.worst_case_post_burn_periapsis_altitude=t.periapsis_altitude;
    plan.worst_case_peak_dynamic_pressure=prediction.peak_dynamic_pressure;
    plan.worst_case_peak_g_load=prediction.peak_g_load;
    plan.live_cutoff_capture_qualified=true;
    plan.live_cutoff_periapsis_altitude=t.periapsis_altitude;
    plan.live_cutoff_closest_distance=prediction.closest_distance;
    plan.live_cutoff_entry_flight_path_angle=prediction.entry_flight_path_angle;
    plan.achieved_state_verified=true;
    plan.achieved_state_capture_qualified=true;
    plan.achieved_post_burn_periapsis_altitude=t.periapsis_altitude;
    plan.confidence=clampd(c->planning_aerodynamics.confidence,0.0,1.0); /* decision-literal-ok: confidence domain */

    guidance_initialize_reentry_continuation(&c->guidance,&t,p,&cfg,
        initial_s_turn_sign,false,&c->envelope,&c->trajectory_calibration);
    if(topology.valid)
        (void)guidance_install_entry_topology(&c->guidance,&topology,t.ut);

    trajectory_copy(&plan.trajectory,&prediction.trajectory);
    snprintf(plan.note,sizeof(plan.note),
        "Reentry continuation qualified by production shadow executive from live state.");
    prediction_cache_store(c,&prediction);
    c->cached_prediction_ut=t.ut;
    c->cached_prediction_sign=initial_s_turn_sign;
    prediction_cache_apply(c,&t,&cfg);
    entry_prediction_clear(&prediction);

    if(c->has_plan)deorbit_plan_clear(&c->plan);
    c->plan=plan;c->has_plan=true;c->plan_stale=false;
    c->time_warp_issued=true;c->log_plan_trajectory_dirty=true;
    trajectory_clear(&c->dynamic_trajectory);
    trajectory_copy(&c->dynamic_trajectory,&c->plan.trajectory);
    trajectory_clear(&c->stabilized_trajectory);
    trajectory_copy(&c->stabilized_trajectory,&c->plan.trajectory);
    trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&c->plan.trajectory);

    c->latest_telemetry=t;c->latest_state=state;c->has_latest=true;
    landing_snapshot_set_sample(&c->snapshot,&t,&state);
    c->last_prediction_ut=-INFINITY;
    t.predicted_miss_distance=c->plan.predicted_closest_distance;
    t.predicted_taem_distance=c->plan.predicted_taem_distance;
    t.predicted_taem_range_error=c->plan.predicted_taem_range_error;
    t.predicted_entry_range=c->plan.predicted_entry_range;
    t.predicted_entry_flight_path_angle=c->plan.predicted_entry_flight_path_angle;

    landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);
    c->snapshot.phase=c->guidance.phase;
    c->snapshot.automation_engaged=true;
    c->snapshot.paused=false;
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),
        in_atmosphere?
        "Reentry-only guidance resumed from live atmospheric state after shadow qualification.":
        "Reentry-only continuation armed from live post-deorbit state after shadow qualification.");
    set_error(c,NULL);set_warning(c,NULL);
    reference_trajectory(&c->snapshot.reference_trajectory,&cfg.site,
        &cfg.guidance,p->radius,initial_s_turn_sign);
    c->log_reference_trajectory_dirty=true;
    log_state_event(c,
        in_atmosphere?
        "event=reentry-continuation-armed; atmospheric-restart=true; shadow-qualified=true":
        "event=reentry-continuation-armed; deorbit-planner-skipped=true; shadow-qualified=true",
        true);
    publish(c);
    pthread_mutex_unlock(&c->mutex);
}

void landing_controller_engage_hac_test(LandingController*c){
    pthread_mutex_lock(&c->mutex);c->prediction_generation++;
    if(!c->session){set_warning(c,"Connect to KSP before starting the HAC-only landing test.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    if(c->glide.active){set_warning(c,"Stop calibration before starting the HAC-only landing test.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    LandingConfiguration cfg=effective_config(c);char err[768]={0};Telemetry t;VehicleState state;
    if(!read_live_sample(c,&cfg,&t,&state,err,sizeof(err))){set_error(c,err);c->snapshot.phase=PHASE_FAULT;publish(c);pthread_mutex_unlock(&c->mutex);return;}
    const PlanetModel*p=krpc_session_planet(c->session);
    if(planet_atmospheric_density(p,t.mean_altitude)<=DBL_EPSILON){set_warning(c,"HAC-only landing test requires an atmospheric flight state.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    if(!strcasecmp(t.vessel_situation,"landed")||!strcasecmp(t.vessel_situation,"splashed")||!strcasecmp(t.vessel_situation,"pre-launch")){set_warning(c,"HAC-only landing test requires an airborne vehicle.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,t.heading);
    char status[384]={0};
    if(!guidance_begin_hac_test(&c->guidance,&t,course,p,c->aerodynamics,&cfg,status,sizeof(status))){set_warning(c,status[0]?status:"The current state cannot initialize a HAC-only landing test.");publish(c);pthread_mutex_unlock(&c->mutex);return;}

    if(c->has_plan){deorbit_plan_clear(&c->plan);c->has_plan=false;}
    c->plan_stale=false;c->time_warp_issued=true;c->has_prediction_cache=false;c->last_prediction_ut=-1e300;
    trajectory_clear(&c->dynamic_trajectory);trajectory_init(&c->dynamic_trajectory);
    trajectory_clear(&c->stabilized_trajectory);trajectory_init(&c->stabilized_trajectory);
    trajectory_clear(&c->trajectory_calibrator.forecast);trajectory_init(&c->trajectory_calibrator.forecast);
    c->latest_telemetry=t;c->latest_state=state;c->has_latest=true;
    landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);c->snapshot.plan=NULL;c->snapshot.phase=PHASE_TAEM;c->snapshot.automation_engaged=true;c->snapshot.paused=false;
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s Entry/S-turn acquisition is bypassed; terminal HAC guidance owns the vehicle.",status);
    set_error(c,NULL);set_warning(c,"HAC-only test mode is active. This is a terminal-guidance test, not a validated reentry continuation.");
    reference_trajectory(&c->snapshot.reference_trajectory,&cfg.site,&cfg.guidance,p->radius,c->guidance.hac_side);c->log_reference_trajectory_dirty=true;
    log_state_event(c,"event=hac-only-test-armed; deorbit-skipped=true; entry-skipped=true; s-turn-skipped=true",true);
    publish(c);pthread_mutex_unlock(&c->mutex);
}

void landing_controller_start_calibration(LandingController*c){
    pthread_mutex_lock(&c->mutex);
    if(!c->session){pthread_mutex_unlock(&c->mutex);return;}
    if(c->guidance.automation_engaged){
        set_warning(c,"Pause or abort landing guidance before starting calibration.");
        publish(c);pthread_mutex_unlock(&c->mutex);return;
    }
    if(!c->has_latest){
        set_warning(c,"Wait for the first telemetry sample before starting calibration.");
        publish(c);pthread_mutex_unlock(&c->mutex);return;
    }

    Telemetry*t=&c->latest_telemetry;
    LandingConfiguration cfg=effective_config(c);
    VehicleProfile*v=&cfg.vehicle;
    CalibrationSettings*cal=&cfg.calibration;
    const char*warning=NULL;

    if(!strcasecmp(t->vessel_situation,"landed")||
       !strcasecmp(t->vessel_situation,"splashed")||
       !strcasecmp(t->vessel_situation,"pre-launch"))
        warning="Calibration mode requires an airborne craft.";
    else if(t->radar_altitude<cal->minimum_calibration_radar_altitude)
        warning="Current radar altitude is below the configured calibration envelope.";
    else if(t->true_air_speed<v->minimum_safe_speed)
        warning="Current airspeed is below the vehicle minimum-safe-speed model.";
    else if(t->dynamic_pressure<cal->minimum_dynamic_pressure)
        warning="Dynamic pressure is below the configured calibration measurement domain.";
    else if(t->dynamic_pressure>cal->maximum_calibration_dynamic_pressure)
        warning="Dynamic pressure exceeds the configured calibration safety domain.";
    else if(t->g_force>cal->maximum_calibration_g_load)
        warning="Load factor exceeds the configured calibration safety domain.";
    else if(-t->vertical_speed>cal->maximum_calibration_sink_rate)
        warning="Sink rate exceeds the configured calibration safety domain.";
    else if(t->angle_of_attack<cal->minimum_angle_of_attack||
            t->angle_of_attack>cal->maximum_angle_of_attack)
        warning="Current angle of attack is outside the configured calibration sweep domain.";

    if(warning){
        set_warning(c,warning);publish(c);pthread_mutex_unlock(&c->mutex);return;
    }

    guidance_set_engaged(&c->guidance,false);
    glide_calibration_start(&c->glide,t,cal);
    c->snapshot.phase=PHASE_CALIBRATION;
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),
        "Starting wings-level glide calibration inside the configured measurement envelope.");
    set_warning(c,NULL);c->snapshot.calibration.active=true;
    publish(c);pthread_mutex_unlock(&c->mutex);
}
void landing_controller_stop_calibration(LandingController*c,bool apply){pthread_mutex_lock(&c->mutex);glide_calibration_stop(&c->glide,apply?"Flight-test calibration stopped and the reviewed profile was explicitly applied.":"Flight-test calibration stopped. The shadow profile remains available for review.");if(c->session)krpc_safe(c->session);if(apply){c->configuration.vehicle=c->adaptive_profile;apply_certified_aero_prior(c);if(c->has_plan){c->plan_stale=true;set_warning(c,"The explicitly applied flight-test profile changed the vehicle model. Create a new deorbit plan before engagement.");}}c->snapshot.phase=PHASE_IDLE;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),apply?"Reviewed calibration profile applied; replan required.":"Flight-test calibration stopped. Certified flight model unchanged.");c->snapshot.calibration.active=false;c->snapshot.calibration.state=CAL_STOPPED;c->snapshot.adaptive_vehicle_profile=c->adaptive_profile;publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_reset_calibration(LandingController*c){pthread_mutex_lock(&c->mutex);glide_calibration_stop(&c->glide,"Live flight-test reconstruction reset.");if(c->session)krpc_safe(c->session);reset_models(c,&c->configuration.vehicle);if(c->session){krpc_session_seed_physics(c->session,&c->physics);apply_certified_aero_prior(c);}calibration_snapshot_init(&c->snapshot.calibration);snprintf(c->snapshot.calibration.status,sizeof(c->snapshot.calibration.status),"Shadow flight-test calibration reset. The stored prior-flight aerodynamic data book remains active.");c->snapshot.phase=PHASE_IDLE;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Shadow calibration reset; certified vessel prior restored.");publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_set_paused(LandingController*c,bool p){pthread_mutex_lock(&c->mutex);c->prediction_generation++;guidance_set_paused(&c->guidance,p);c->snapshot.paused=p;if(p&&c->session)krpc_safe(c->session);log_state_event(c,p?"event=operator-pause; safe_control_release=true":"event=operator-resume",true);publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_abort(LandingController*c){pthread_mutex_lock(&c->mutex);c->prediction_generation++;guidance_abort(&c->guidance);glide_calibration_stop(&c->glide,"Calibration aborted. Manual control restored.");if(c->session)krpc_safe(c->session);c->snapshot.automation_engaged=false;c->snapshot.paused=false;c->snapshot.calibration.active=false;c->snapshot.phase=PHASE_ABORT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Automation aborted. Manual control restored.");log_state_event(c,"event=operator-abort; safe_control_release=true",true);publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_set_gear(LandingController*c,bool v){pthread_mutex_lock(&c->mutex);if(c->session){char e[512];if(!krpc_set_gear(c->session,v,e,sizeof(e))){set_warning(c,e);publish(c);}}pthread_mutex_unlock(&c->mutex);}
void landing_controller_set_brakes(LandingController*c,bool v){pthread_mutex_lock(&c->mutex);if(c->session){char e[512];if(!krpc_set_brakes(c->session,v,e,sizeof(e))){set_warning(c,e);publish(c);}}pthread_mutex_unlock(&c->mutex);}
bool landing_controller_save_checkpoint(LandingController*c,const char*name,char*error,size_t error_size){bool ok=false;pthread_mutex_lock(&c->mutex);if(!c->session)snprintf(error,error_size,"kRPC is not connected");else ok=krpc_save_game(c->session,name,error,error_size);pthread_mutex_unlock(&c->mutex);return ok;}
void landing_controller_shutdown(LandingController*c){if(!c)return;stop_thread(c);pthread_mutex_lock(&c->mutex);disconnect_locked(c,false);pthread_mutex_unlock(&c->mutex);}
