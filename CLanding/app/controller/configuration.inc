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
    a->terminal_maximum_lift_angle_of_attack==b->terminal_maximum_lift_angle_of_attack&&
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

