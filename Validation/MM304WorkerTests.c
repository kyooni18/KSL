/* Exercise the production worker with an explicitly blocked replay, no transport. */
#define krpc_session_planet test_session_planet
#define predictor_simulate_entry_guidance_shadow blocked_entry_replay
#define predictor_plan_entry_topology scripted_entry_topology
#include "../CLanding/controller.c"
#undef krpc_session_planet
#undef predictor_simulate_entry_guidance_shadow
#undef predictor_plan_entry_topology
#include <assert.h>

static PlanetModel test_planet;
static pthread_mutex_t barrier_mutex=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t barrier_cond=PTHREAD_COND_INITIALIZER;
static bool entered,release_replay;
const PlanetModel *test_session_planet(const KRPCSession*s){(void)s;return &test_planet;}
EntryTopologyPlan scripted_entry_topology(VehicleState state,const Telemetry*t,const GuidanceMachine*g,
        const PlanetModel*p,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg){
    (void)state;(void)p;(void)env;(void)cal;(void)cfg;
    EntryTopologyPlan top={0};top.valid=true;top.planned_ut=t->ut;top.first_sign=g->s_turn_sign;
    top.first_bank=30;top.turn_bank=45;top.first_aoa=18;top.turn_aoa=16;
    top.reversal_ut=t->ut+60;top.terminal_turn_ut=t->ut+100;top.capture_ut=t->ut+160;
    top.reversal_along=t->runway_along_track+60000;top.reversal_cross=t->runway_cross_track+12000;
    top.reversal_altitude=36000;top.reversal_speed=1500;top.reversal_course=90;
    top.glide_reserve=5000;top.inlet=g->taem_interface_target;top.inlet.valid=true;
    return top;
}
EntryPrediction blocked_entry_replay(VehicleState state,const Telemetry*t,const GuidanceMachine*g,
    const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,
    const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,double duration,bool include){
    (void)t;(void)g;(void)plan;(void)p;(void)aero;(void)env;(void)cal;(void)cfg;(void)duration;(void)include;
    pthread_mutex_lock(&barrier_mutex);entered=true;pthread_cond_broadcast(&barrier_cond);
    while(!release_replay)pthread_cond_wait(&barrier_cond,&barrier_mutex);
    pthread_mutex_unlock(&barrier_mutex);
    EntryPrediction out={0};out.shadow_guidance_used=true;out.final_state=state;
    out.reached_taem=true;out.taem_dynamic_interface_captured=true;
    out.shadow_terminal_policy_feasible=true;
    out.closest_distance=0;out.taem_range_error=0;out.taem_speed=500;out.taem_energy_error=0;
    out.peak_dynamic_pressure=1000;out.peak_g_load=1;return out;
}
int main(void){
    LandingConfiguration cfg=landing_configuration_default();cfg.guidance.prediction_interval=2;cfg.guidance.taem_interface_altitude=20000;
    LandingController*c=landing_controller_create(&cfg,NULL,NULL);assert(c);
    test_planet=(PlanetModel){.radius=600000,.gravitational_parameter=3.5316e12,
        .rotational_speed=.000291570900559802,.atmosphere_depth=70000,.surface_density=1.1399,
        .north_axis={0,0,1},.prime_meridian_at_epoch={1,0,0}};
    Telemetry t;telemetry_init(&t);t.ut=100;t.mean_altitude=t.radar_altitude=41400;
    t.true_air_speed=1923;t.horizontal_speed=1910;t.vertical_speed=-150;t.flight_path_angle=-4.47;
    t.mass=40251;t.lift_force=t.mass*5;t.drag_force=t.mass*3;t.dynamic_pressure=3000;
    t.g_force=1;t.mach=6;t.angle_of_attack=18;t.ground_track_heading=t.heading=90;
    t.latitude=cfg.site.latitude;t.longitude=cfg.site.longitude-25;t.range_to_site=260000;
    t.runway_along_track=-260000;t.runway_cross_track=0;t.bank_effectiveness=1;t.aerodynamic_confidence=.8;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    GeoPoint geo={t.latitude,t.longitude,t.mean_altitude};
    Vector3 pos=predictor_inertial_position(geo,&test_planet,t.ut),up=vnorm(pos,v3(1,0,0));
    Vector3 east=vnorm(vcross(test_planet.north_axis,up),v3(0,1,0));
    VehicleState state={t.ut,pos,vadd(vadd(vscale(east,1910),vscale(up,-150)),
        vcross(planet_rotation_vector(&test_planet),pos)),t.mass};
    guidance_initialize_reentry_continuation(&c->guidance,&t,&test_planet,&cfg,1,false,&c->envelope,&c->trajectory_calibration);
    c->guidance.entry_planning_deferred=true;c->guidance.diagnostic_shadow=true;
    c->plan=(DeorbitPlan){.execution_qualified=true,.live_cutoff_capture_qualified=true};c->has_plan=true;
    GuidanceResult r=guidance_update(&c->guidance,&t,&state,&c->plan,&test_planet,c->aerodynamics,&cfg);
    guidance_result_clear(&r);
    /* Reproduce the production lockout seam: ordinary MM304 has already committed
       an unexecuted nonfinal fallback reversal before the slow worker finds a complete
       same-side topology. The worker must keep searching and replace that event. */
    c->guidance.entry_topology.valid=false;c->guidance.entry_control_reversals=0;
    c->guidance.entry_reversal_scheduled=true;c->guidance.entry_reversal_is_final=false;
    c->guidance.entry_reversal_ut=t.ut+120;c->guidance.entry_reversal_sign=-c->guidance.s_turn_sign;
    c->guidance.entry_reversal_bank=25;c->guidance.entry_reversal_range=180000;
    c->guidance.entry_s_turn_plan.valid=true;c->guidance.entry_s_turn_plan.planned_ut=t.ut;
    c->guidance.entry_s_turn_plan.segment_duration=220;c->guidance.entry_s_turn_plan.target_bank=c->guidance.s_turn_sign*25;
    c->guidance.entry_s_turn_plan.target_aoa=18;c->guidance.entry_s_turn_plan.has_planned_reversal=true;
    c->guidance.entry_s_turn_plan.planned_reversal_is_final=false;
    c->guidance.entry_s_turn_plan.planned_reversal_ut=c->guidance.entry_reversal_ut;
    c->guidance.entry_s_turn_plan.planned_reversal_sign=c->guidance.entry_reversal_sign;
    c->guidance.entry_s_turn_plan.planned_reversal_range=c->guidance.entry_reversal_range;
    double ordinary_reversal_ut=c->guidance.entry_reversal_ut;
    c->latest_telemetry=t;c->latest_state=state;c->has_latest=true;
    c->session=(KRPCSession*)c; /* only test_session_planet may inspect this sentinel */
    c->last_terminal_prediction_request_ut=c->last_terminal_prediction_completion_ut=-INFINITY;
    c->last_terminal_prediction_completion_wall=-INFINITY;
    pthread_t worker;assert(pthread_create(&worker,NULL,terminal_prediction_worker,c)==0);
    pthread_mutex_lock(&barrier_mutex);
    struct timespec deadline;clock_gettime(CLOCK_REALTIME,&deadline);deadline.tv_sec+=15;
    while(!entered)assert(pthread_cond_timedwait(&barrier_cond,&barrier_mutex,&deadline)==0);
    pthread_mutex_unlock(&barrier_mutex);
    uint64_t sequence=c->guidance.control_plan_sequence;double worst=0;
    for(int i=0;i<50;i++){
        double start=monotonic_seconds();pthread_mutex_lock(&c->mutex);
        t.ut+=.1;state.ut=t.ut;
        r=guidance_update(&c->guidance,&t,&state,&c->plan,&test_planet,c->aerodynamics,&cfg);
        c->latest_telemetry=t;c->latest_state=state;guidance_result_clear(&r);
        pthread_mutex_unlock(&c->mutex);worst=fmax(worst,monotonic_seconds()-start);
    }
    assert(worst<.1);
    pthread_mutex_lock(&barrier_mutex);release_replay=true;pthread_cond_broadcast(&barrier_cond);pthread_mutex_unlock(&barrier_mutex);
    double until=monotonic_seconds()+5;
    for(;;){
        pthread_mutex_lock(&c->mutex);bool completed=isfinite(c->last_terminal_prediction_completion_wall);
        pthread_mutex_unlock(&c->mutex);if(completed)break;
        assert(monotonic_seconds()<until);sleep_seconds(.001);
    }
    pthread_mutex_lock(&c->mutex);
    /* A multi-second solve is not stale merely because KSP UT advanced. If the live
       MM304 phase/side/sequence/target lineage is unchanged and its executable
       deadline is still ahead, the worker result must be adopted. */
    assert(c->guidance.control_plan_sequence>sequence);
    assert(c->guidance.entry_topology.valid);
    assert(c->guidance.entry_reversal_scheduled&&!c->guidance.entry_reversal_is_final);
    assert(c->guidance.entry_control_reversals==0);
    assert(c->guidance.entry_reversal_ut<ordinary_reversal_ut);
    assert(c->guidance.entry_reversal_sign==-c->guidance.entry_topology.first_sign);
    assert(c->has_prediction_cache);
    c->stop_prediction_thread=true;c->session=NULL;pthread_mutex_unlock(&c->mutex);
    pthread_join(worker,NULL);
    printf("PASS: blocked production worker permits 50 control ticks (worst %.3f ms); late same-side topology replaces unexecuted ordinary reversal\n",worst*1000);
    landing_controller_destroy(c);return 0;
}
