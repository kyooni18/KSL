/* Runway-frame state at the first forecast point inside the live MM305
   physical handoff (Mach <= target+half-width, altitude <= MM305 ceiling),
   interpolated between recorded samples; course and flight-path angle from
   the bracketing samples.  False when the forecast never gets there. */
static bool entry_policy_handoff_geometry(const EntryPrediction *f,const PlanetModel *p,
        const LandingConfiguration *cfg,TaemGeometryState *out){
    if(!f||!p||!cfg||!out)return false;
    GeoPoint site={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    /* Experimental STS hands over at the band centre (guidance_terminal.c). */
    double mach_handoff=cfg->guidance.mm305_target_mach+
        ((cfg->guidance.sts_trajectory_delivery||cfg->guidance.taem_hac_law)&&
         !strcmp(cfg->vehicle.model_id,"STS")?0.0:cfg->guidance.mm305_mach_half_width);
    double prev_excess=NAN;
    for(size_t i=0;i<f->trajectory.count;++i){
        const TrajectoryPoint *pt=&f->trajectory.points[i];
        double sound=planet_atmospheric_speed_of_sound(p,pt->altitude);
        if(!(sound>0.0))continue;
        double excess=pt->speed-mach_handoff*sound;
        if(i>0&&excess<=0.0&&pt->altitude<=cfg->guidance.mm305_max_altitude){
            const TrajectoryPoint *q=&f->trajectory.points[i-1];
            double k=isfinite(prev_excess)&&prev_excess>0.0?prev_excess/(prev_excess-excess):1.0;
            GeoPoint a={q->latitude,q->longitude,q->altitude},b={pt->latitude,pt->longitude,pt->altitude};
            double ground=great_circle_distance(a,b,p->radius);
            if(!(ground>1.0))return false;
            double course=initial_bearing(a,b);
            GeoPoint at=destination_point(a,course,k*ground,p->radius);
            at.altitude=q->altitude+k*(pt->altitude-q->altitude);
            double speed=q->speed+k*(pt->speed-q->speed);
            double fpa=atan2(pt->altitude-q->altitude,ground);
            double along,cross;
            runway_coordinates(at,site,cfg->site.runway_heading,p->radius,&along,&cross);
            *out=(TaemGeometryState){.runway_along_m=along,.runway_cross_m=cross,
                .altitude_above_runway_m=at.altitude-cfg->site.altitude,
                .ground_speed_mps=speed*cos(fpa),.airspeed_mps=speed,
                .course_deg=course,.flight_path_angle_deg=fpa*RAD2DEG,
                .heading_error_deg=norm_signed_deg(course-cfg->site.runway_heading)};
            return true;
        }
        prev_excess=excess;
    }
    return false;
}

/* MM304 cost of a forecast: the energy-height shortfall at the handoff
   against what MM305's HAC needs from that position and course plus the
   law's high band (+ : short).  Measured as MM305 will fly it, so crossrange
   and course error left at handoff are charged as HAC turn, not ignored by a
   straight-line site distance (live 2026-10-29: 49.7 km site target, 70 km
   HAC path flown, no energy left for the HAC). */
static double entry_policy_handoff_miss(const EntryPrediction *f,const PlanetModel *p,
        const LandingConfiguration *cfg,const GuidanceMachine *g,const TerminalModel *model,
        double mass){
    TaemGeometryState geo;
    double error,band;
    /* The model comes from the worker: a guidance copy's entry_terminal_model
       is bound only inside guidance_update and is NULL here, which left every
       live HAC replan without a forecast (live 2026-10-29). */
    if(!g||!model||!entry_policy_handoff_geometry(f,p,cfg,&geo)||
       !mm305_hac_handoff_energy_error(model,&g->taem_hac_live_aero,
           &geo,mass,&error,&band))
        return NAN;
    return band-error;
}

static double entry_supervision_horizon(const GuidanceSettings *settings){
    if(!settings||!isfinite(settings->prediction_interval)||
       settings->prediction_interval<=0.0)return 0.0;
    return settings->prediction_interval;
}

static AsyncPredictionIdentity terminal_prediction_identity(const GuidanceMachine *g){
    AsyncPredictionIdentity id={0};
    if(!g)return id;
    id.guidance_phase=(int)g->phase;
    id.taem_phase=(int)g->taem_exec.phase;
    id.taem_transition_count=g->taem_exec.transition_count;
    id.control_plan_sequence=g->control_plan_sequence;
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

/* MM305 route planning job.  Highest priority on the worker: it is not an
   advisory forecast but the route the tracker is waiting for.  Returns true
   when a job ran (successfully or not). */
/* MM304 -> MM305 qualification: the route MM305 would fly from the latest
   measured MM304 state.  Ownership stays with MM304 until it is found. */
static bool mm305_admission_job(LandingController *c){
    pthread_mutex_lock(&c->mutex);
    if(c->stop_prediction_thread||!c->session||!c->has_latest||!c->terminal_model_valid||
       !c->guidance.automation_engaged||c->guidance.paused||c->guidance.aborted||
       c->guidance.phase!=PHASE_ENTRY_ENERGY||!c->guidance.mm305_async_planning||
       !c->guidance.mm305_admission_needed){
        pthread_mutex_unlock(&c->mutex);
        return false;
    }
    LandingConfiguration cfg=effective_config(c);
    TerminalModel *model=malloc(sizeof(*model));
    if(!model){c->guidance.mm305_admission_needed=false;pthread_mutex_unlock(&c->mutex);return true;}
    *model=c->terminal_model;
    Mm305PlanRequest request=guidance_mm305_admission_request(&c->guidance,&c->latest_telemetry,
        &c->latest_state,&cfg,model);
    double token=c->guidance.mm305_admission_request_ut;
    uint64_t generation=c->prediction_generation;
    if(!request.valid){
        c->guidance.mm305_admission_needed=false;
        pthread_mutex_unlock(&c->mutex);
        free(model);
        return true;
    }
    pthread_mutex_unlock(&c->mutex);

    Mm305PlanResult result=mm305_plan(model,&request);
    free(model);

    pthread_mutex_lock(&c->mutex);
    bool current=!c->stop_prediction_thread&&generation==c->prediction_generation&&
        c->guidance.phase==PHASE_ENTRY_ENERGY&&c->guidance.mm305_admission_needed&&
        c->guidance.mm305_admission_request_ut==token&&
        c->terminal_model_valid&&c->terminal_model.snapshot_id==result.model_snapshot_id;
    if(current){
        guidance_mm305_admission_accept(&c->guidance,&result);
        char event[256];
        snprintf(event,sizeof(event),"event=mm305-admission; request-ut=%.3f; found=%d; solve-wall=%.3f; rejections=%u",
            request.request_ut,result.found,result.solve_wall_s,
            (unsigned)c->guidance.mm305_admission_rejections);
        log_state_event(c,event,true);
    }
    pthread_mutex_unlock(&c->mutex);
    return true;
}

static bool mm305_planning_job(LandingController *c){
    pthread_mutex_lock(&c->mutex);
    if(c->stop_prediction_thread||!c->session||!c->has_latest||!c->terminal_model_valid||
       !c->guidance.automation_engaged||c->guidance.paused||c->guidance.aborted||
       c->guidance.phase!=PHASE_TAEM||!c->guidance.mm305_async_planning||
       !c->guidance.mm305_planning_needed){
        pthread_mutex_unlock(&c->mutex);
        return false;
    }
    LandingConfiguration cfg=effective_config(c);
    TerminalModel *model=malloc(sizeof(*model));
    if(!model){c->guidance.mm305_planning_needed=false;pthread_mutex_unlock(&c->mutex);return true;}
    *model=c->terminal_model;
    Mm305PlanRequest request=guidance_mm305_plan_request(&c->guidance,&c->latest_telemetry,
        &c->latest_state,&cfg,model);
    double token=c->guidance.mm305_plan_request_ut;
    uint64_t generation=c->prediction_generation;
    if(!request.valid){
        /* Never leave the lockstep barrier waiting on an impossible request. */
        c->guidance.mm305_planning_needed=false;
        c->guidance.mm305_plan_failures++;
        pthread_mutex_unlock(&c->mutex);
        free(model);
        return true;
    }
    pthread_mutex_unlock(&c->mutex);

    Mm305PlanResult result=mm305_plan(model,&request);

    pthread_mutex_lock(&c->mutex);
    bool current=!c->stop_prediction_thread&&generation==c->prediction_generation&&
        c->guidance.phase==PHASE_TAEM&&c->guidance.mm305_planning_needed&&
        c->guidance.mm305_plan_request_ut==token&&
        c->terminal_model_valid&&c->terminal_model.snapshot_id==result.model_snapshot_id;
    if(current&&result.found&&!mm305_plan_result_fresh(&result,&c->latest_telemetry)){
        /* Search is expensive in live time. Qualify only the selected geometry
           again from the current state, outside the control-loop mutex. */
        cfg=effective_config(c);
        Mm305PlanRequest fresh=guidance_mm305_plan_request(&c->guidance,
            &c->latest_telemetry,&c->latest_state,&cfg,model);
        pthread_mutex_unlock(&c->mutex);
        result=mm305_revalidate_plan(model,&fresh,&result);
        pthread_mutex_lock(&c->mutex);
        current=!c->stop_prediction_thread&&generation==c->prediction_generation&&
            c->guidance.phase==PHASE_TAEM&&c->guidance.mm305_planning_needed&&
            c->guidance.mm305_plan_request_ut==token&&c->terminal_model_valid&&
            c->terminal_model.snapshot_id==result.model_snapshot_id;
    }
    if(current){
        bool stale=result.found&&!mm305_plan_result_fresh(&result,&c->latest_telemetry);
        bool adopted=false;
        if(stale){
            c->guidance.mm305_planning_needed=false;
            c->guidance.mm305_plan_failures++;
            snprintf(result.diagnostic,sizeof(result.diagnostic),
                "MM305 route rejected: current-state qualification expired before adoption");
        }else adopted=guidance_mm305_accept_plan(&c->guidance,&result,&cfg);
        char event[256];
        snprintf(event,sizeof(event),"event=mm305-plan; request-ut=%.3f; found=%d; adopted=%d; solve-wall=%.3f; replans=%u",
            result.request_ut,result.found,adopted,result.solve_wall_s,
            (unsigned)c->guidance.mm305_replans);
        log_state_event(c,event,true);
    }
    pthread_mutex_unlock(&c->mutex);
    free(model);
    return true;
}

/* MM305 route planning has its own thread: it must never queue behind an
   advisory forecast (a terminal shadow ensemble can take many seconds, and in
   simulator lockstep the whole simulation waits on the route). */
static void *mm305_planner_worker(void *arg){
    LandingController *c=arg;
    for(;;){
        pthread_mutex_lock(&c->mutex);
        bool stop=c->stop_prediction_thread;
        pthread_mutex_unlock(&c->mutex);
        if(stop)break;
        if(!mm305_planning_job(c)&&!mm305_admission_job(c))sleep_seconds(.005);
    }
    return NULL;
}

/* Inertial speed and radial rate: the two observables the drag and lift
   scales of a short re-flown forecast are fitted to. */
static void forecast_observables(const VehicleState *s,double *speed,double *radial){
    *speed=vmag(s->velocity);
    *radial=vdot(s->velocity,vnorm(s->position,v3(0,0,1)));
}

/* Re-fly the last ~30 s from the recorded state with the bank and AoA the
   vehicle actually held, and correct the forecast drag/lift scales so the
   re-flown forecast arrives at the measured speed and radial rate.  A
   forecast that disagrees with the flight it predicts may not keep steering
   the vehicle (live 2026-10-28: "short" for five minutes, then an overshoot). */
static void forecast_fit_scales(const ForecastHistorySample *h,int n,const VehicleState *now,
        const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *envelope,
        const TrajectoryCalibrationModel *calibration,const LandingConfiguration *cfg,
        double *drag_scale,double *lift_scale){
    if(n<2)return;
    int j=-1;
    for(int i=n-1;i>=0;--i)if(now->ut-h[i].state.ut>=FORECAST_FIT_WINDOW_S){j=i;break;}
    if(j<0)return;
    double bank=0,aoa=0,side=0;int m=0;
    for(int i=j;i<n;++i){
        if(!(h[i].q>=FORECAST_FIT_MIN_Q_PA))return;               /* fit in sensible air only */
        if(fabs(h[i].bank)>5.0){
            double s=h[i].bank>=0?1.0:-1.0;
            if(side!=0.0&&s!=side)return;                          /* a reversal inside the window */
            side=s;
        }
        bank+=fabs(h[i].bank);aoa+=h[i].aoa;m++;
    }
    bank/=m;aoa/=m;if(side==0.0)side=1.0;
    double duration=now->ut-h[j].state.ut;
    double k[2]={*drag_scale,*lift_scale},pred[3][2];
    for(int c=0;c<3;++c){
        TrajectoryCalibrationModel cal=*calibration;
        cal.forecast_drag_scale=k[0]*(c==1?1.1:1.0);
        cal.forecast_lift_scale=k[1]*(c==2?1.1:1.0);
        cal.forecast_exact_policy_bank=true;
        EntryPrediction f=predictor_simulate_entry_policy(h[j].state,planet,aero,envelope,&cal,
            &cfg->vehicle,&cfg->site,&cfg->guidance,bank,aoa,side,duration);
        VehicleState end=f.final_state;
        entry_prediction_clear(&f);
        if(fabs(end.ut-now->ut)>2.0)return;
        /* carry the forecast to the measurement time along its end velocity */
        forecast_observables(&end,&pred[c][0],&pred[c][1]);
    }
    double act[2];forecast_observables(now,&act[0],&act[1]);
    double r0=act[0]-pred[0][0],r1=act[1]-pred[0][1];
    double d=log(1.1);
    double a=(pred[1][0]-pred[0][0])/d,b=(pred[2][0]-pred[0][0])/d;
    double cc=(pred[1][1]-pred[0][1])/d,dd=(pred[2][1]-pred[0][1])/d;
    double det=a*dd-b*cc;
    if(!(fabs(det)>1e-6))return;
    double x0=( dd*r0-b*r1)/det,x1=(-cc*r0+a*r1)/det;              /* log-scale steps */
    x0=clampd(0.5*x0,-0.2,0.2);x1=clampd(0.5*x1,-0.2,0.2);
    *drag_scale=clampd(k[0]*exp(x0),FORECAST_SCALE_MIN,FORECAST_SCALE_MAX);
    *lift_scale=clampd(k[1]*exp(x1),FORECAST_SCALE_MIN,FORECAST_SCALE_MAX);
    if(getenv("KSP_LANDER_TRACE_FORECAST_FIT"))
        fprintf(stderr,"FORECAST_FIT ut=%.1f window=%.0fs bank=%.1f aoa=%.1f dv=%+.2f dvr=%+.2f -> drag x%.3f lift x%.3f\n",
            now->ut,duration,bank,aoa,r0,r1,*drag_scale,*lift_scale);
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
        TerminalModel terminal_model={0};bool terminal_model_valid=false;
        GuidanceMachine guidance={0},entry_request={0};
        bool entry_job=false,entry_planning_job=false;uint64_t generation=0;
        DeorbitPlan plan;memset(&plan,0,sizeof(plan));trajectory_init(&plan.trajectory);
        bool run=false,has_plan=false;
        double request_ut=0,sign=0;
        AsyncPredictionIdentity request_identity={0};
        ForecastHistorySample history[FORECAST_HISTORY_MAX];int history_count=0;
        double forecast_drag=1.0,forecast_lift=1.0;

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
            /* ShuttleSim lockstep: the cooldown elapses in UT, as it would live at 1x */
            if(krpc_session_is_simulator(c->session)&&c->last_terminal_prediction_completion_ut>-1e299)
                now_wall=c->last_terminal_prediction_completion_wall+
                    (c->latest_telemetry.ut-c->last_terminal_prediction_completion_ut);
            if(async_prediction_request_due(c->latest_telemetry.ut,cadence_anchor,now_wall,
                    c->last_terminal_prediction_completion_wall,c->last_terminal_prediction_solve_wall,
                    c->configuration.guidance.prediction_interval)){
                state=c->latest_state;
                telemetry=c->latest_telemetry;
                history_count=c->forecast_history_count;
                memcpy(history,c->forecast_history,(size_t)history_count*sizeof(*history));
                if(c->forecast_drag_scale>0.0)forecast_drag=c->forecast_drag_scale;
                if(c->forecast_lift_scale>0.0)forecast_lift=c->forecast_lift_scale;
                cfg=effective_config(c);
                planet=*krpc_session_planet(c->session);
                aero=c->aerodynamics;
                envelope=c->envelope;
                physics=c->physics;
                calibration=c->trajectory_calibration;
                calibration.physics=&physics;
                if(c->terminal_model_valid){terminal_model=c->terminal_model;terminal_model_valid=true;}
                guidance=c->guidance;
                /* A shadow forecast never plans MM305 routes: it flies the
                   committed route or the acquisition law.  (Planning inside
                   every ensemble member made each forecast cost minutes.) */
                guidance.mm305_async_planning=true;
                guidance.mm305_planning_needed=false;
                entry_request=guidance;
                entry_request.previous_ut=telemetry.ut;
                entry_job=guidance.phase==PHASE_ENTRY_ENERGY;
                entry_planning_job=entry_job&&guidance.entry_planning_needed;
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
        if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
            fprintf(stderr,"ENTRY_WORKER start ut=%.3f entry=%d planning=%d h=%.1f V=%.1f sequence=%llu\n",
                request_ut,entry_job,entry_planning_job,telemetry.mean_altitude,
                telemetry.true_air_speed,(unsigned long long)entry_request.control_plan_sequence);
        EntryPrediction pr;
        if(entry_job){
            /* Forecasting owns no control state. Recompute one command from this
               immutable measured snapshot, then replay the same closed-loop law for
               one prediction interval. */
            guidance.entry_planning_deferred=false;
            guidance.diagnostic_shadow=true;
            GuidanceResult proposal=guidance_update(&guidance,&telemetry,&state,
                has_plan?&plan:NULL,&planet,aero,&cfg);
            guidance_result_clear(&proposal);
            double proposal_finished=monotonic_seconds();
            if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
                fprintf(stderr,"ENTRY_WORKER stage proposal wall=%.3f\n",
                    proposal_finished-solve_started);

            guidance.entry_planning_deferred=true;
            double supervision_horizon=entry_supervision_horizon(&cfg.guidance);
            const TerminalModel*supervision_terminal_model=
                terminal_model_valid?&terminal_model:NULL;
            pr=predictor_simulate_entry_guidance_shadow_with_terminal_model(state,&telemetry,&guidance,
                has_plan?&plan:NULL,&planet,aero,&envelope,&calibration,&cfg,
                supervision_terminal_model,supervision_horizon,false);
            if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
                fprintf(stderr,"ENTRY_WORKER stage supervision wall=%.3f horizon=%.3f terminal=%d\n",
                    monotonic_seconds()-proposal_finished,supervision_horizon,
                    supervision_terminal_model?1:0);

            guidance.entry_s_turn_plan.terminal_ready=pr.reached_taem&&
                pr.taem_dynamic_interface_captured&&pr.shadow_terminal_policy_feasible;
            guidance.entry_committed_infeasible=pr.taem_ownership_boundary_missed;
            guidance.entry_supervision_valid=pr.shadow_guidance_used&&!pr.shadow_aborted&&
                pr.peak_dynamic_pressure<=cfg.vehicle.maximum_dynamic_pressure&&
                pr.peak_g_load<=cfg.vehicle.maximum_g_load;
            guidance.entry_supervision_ut=request_ut;
            guidance.entry_supervision_boundary_missed=pr.taem_ownership_boundary_missed;
            guidance.entry_supervision_mode=guidance.entry_supervision_valid?
                ENTRY_SUPERVISION_PASS_THROUGH:ENTRY_SUPERVISION_INFEASIBLE;
            /* Re-plan from live telemetry: solve the constant bank magnitude
               whose physics forecast from this measured state reaches the live
               MM305 handoff (Mach mm305TargetMach+halfWidth below the MM305
               ceiling) at the target site distance. More bank sheds more
               energy and hands off further out, so the miss is monotone. */
            double target=guidance.entry_handoff_target_range;
            if((cfg.guidance.taem_energy_law||cfg.guidance.taem_hac_law)&&isfinite(target)&&
               isfinite(telemetry.range_to_site)){
                forecast_fit_scales(history,history_count,&state,&planet,aero,&envelope,&calibration,&cfg,
                    &forecast_drag,&forecast_lift);
                calibration.forecast_drag_scale=forecast_drag;
                calibration.forecast_lift_scale=forecast_lift;
                /* Forecast the incidence MM304 actually commands (staircase,
                   drag boost and low-energy cap included), not the schedule. */
                double aoa=guidance.entry_control_plan_valid&&isfinite(guidance.entry_control_aoa)?
                    guidance.entry_control_aoa:
                    guidance.entry_alpha_has_target&&isfinite(guidance.entry_alpha_target)?
                    guidance.entry_alpha_target:telemetry.angle_of_attack;
                double side=guidance.s_turn_sign>=0.0?1.0:-1.0;
                double hi=fmin(MM304_PLANNED_MAX_BANK_DEG,cfg.vehicle.maximum_bank_angle);
                double lo=0.0,f_lo=NAN,f_hi=NAN;
                double sound_floor=planet_atmospheric_speed_of_sound(&planet,cfg.guidance.mm305_max_altitude);
                /* Experimental STS hands over at the band centre (guidance_terminal.c). */
                double mach_handoff=cfg.guidance.mm305_target_mach+
                    ((cfg.guidance.sts_trajectory_delivery||cfg.guidance.taem_hac_law)&&
                     !strcmp(cfg.vehicle.model_id,"STS")?0.0:cfg.guidance.mm305_mach_half_width);
                double bound=2.0*telemetry.range_to_site/fmax(1.0,mach_handoff*sound_floor);
                for(int iteration=0;;++iteration){
                    double bank=iteration==0?lo:iteration==1?hi:0.5*(lo+hi);
                    EntryPrediction f=predictor_simulate_entry_policy(state,&planet,aero,&envelope,
                        &calibration,&cfg.vehicle,&cfg.site,&cfg.guidance,bank,aoa,side,bound);
                    /* + : short.  HAC law: energy height (m) under MM305's HAC
                       need plus margin; otherwise handoff site distance past target. */
                    double miss=NAN;
                    if(cfg.guidance.taem_hac_law)
                        miss=entry_policy_handoff_miss(&f,&planet,&cfg,&guidance,
                            terminal_model_valid?&terminal_model:NULL,state.mass);
                    else{
                        TaemGeometryState geo;
                        if(entry_policy_handoff_geometry(&f,&planet,&cfg,&geo))
                            miss=hypot(geo.runway_along_m,geo.runway_cross_m)-target;
                    }
                    entry_prediction_clear(&f);
                    if(iteration==0){f_lo=miss;if(!(miss<0.0))break;continue;}
                    if(iteration==1){f_hi=miss;if(!(miss>0.0))break;continue;}
                    if(!isfinite(miss))break;
                    if(miss>0.0){hi=bank;f_hi=miss;}else{lo=bank;f_lo=miss;}
                    /* stop when the bracket is below the roll command resolution
                       the allocator can express in one guidance tick */
                    if(hi-lo<=cfg.guidance.entry_roll_rate/fmax(1.0,cfg.guidance.guidance_rate))break;
                }
                double solved=isfinite(f_lo)&&!(f_lo<0.0)?0.0:
                    isfinite(f_hi)&&!(f_hi>0.0)?hi:0.5*(lo+hi);
                double policy_aoa=NAN,best_miss=f_lo;
                if(isfinite(f_lo)||isfinite(f_hi)){
                    guidance.entry_policy_valid=true;
                    guidance.entry_policy_bank_deg=solved;
                    guidance.entry_policy_aoa_deg=policy_aoa;
                    guidance.entry_policy_ut=request_ut;
                    fprintf(stderr,"MM304 live replan: ut=%.1f target %.1f km bank %.1f deg aoa %.1f deg "
                        "(short at 0 deg %+.1f km, at max %+.1f km, best %+.1f km; HAC law: energy height)\n",request_ut,target/1000.0,
                        solved,policy_aoa,f_lo/1000.0,f_hi/1000.0,best_miss/1000.0);
                }
            }
        }else pr=predictor_simulate_terminal_shadow_ensemble(state,&telemetry,&guidance,
            has_plan?&plan:NULL,&planet,aero,&envelope,&calibration,&cfg,
            terminal_model_valid?&terminal_model:NULL,600,true);
        double solve_finished=monotonic_seconds();
        if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
            fprintf(stderr,"ENTRY_WORKER finish ut=%.3f wall=%.3f reached=%d captured=%d aborted=%d final_ut=%.3f\n",
                request_ut,solve_finished-solve_started,pr.reached_taem,
                pr.taem_dynamic_interface_captured,pr.shadow_aborted,pr.final_state.ut);
        deorbit_plan_clear(&plan);

        pthread_mutex_lock(&c->mutex);
        if(entry_job){c->forecast_drag_scale=forecast_drag;c->forecast_lift_scale=forecast_lift;}
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
            bool adopted=guidance_accept_entry_plan(&c->guidance,&entry_request,&guidance,
                &c->latest_telemetry,&cfg);
            if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
                fprintf(stderr,"ENTRY_WORKER adoption request_ut=%.3f live_ut=%.3f adopted=%d\n",
                    request_ut,c->latest_telemetry.ut,adopted);
            if(adopted&&request_ut>c->cached_prediction_ut){
                terminal_prediction_commit(c,&pr,request_ut,sign);
                const EntryControlPlan*live_plan=current_entry_control_plan(c);
                c->forecast_entry_plan_identity_valid=live_plan&&live_plan->valid&&live_plan->plan_id!=0;
                c->forecast_entry_plan_id=c->forecast_entry_plan_identity_valid?live_plan->plan_id:0;
                c->forecast_entry_plan_version=c->forecast_entry_plan_identity_valid?live_plan->plan_version:0;
            }
        }else if(accept)terminal_prediction_commit(c,&pr,request_ut,sign);
        pthread_mutex_unlock(&c->mutex);
        entry_prediction_clear(&pr);
    }
    return NULL;
}


static void start_prediction_worker(LandingController *c){
    pthread_mutex_lock(&c->mutex);
    if(!c->prediction_thread_started){
        c->stop_prediction_thread=false;
        c->last_terminal_prediction_request_ut=c->last_terminal_prediction_completion_ut=-1e300;
        c->last_terminal_prediction_completion_wall=-1e300;
        c->last_terminal_prediction_solve_wall=0.0;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 8 * 1024 * 1024);
        if(pthread_create(&c->prediction_thread,&attr,terminal_prediction_worker,c)==0)
            c->prediction_thread_started=true;
        if(c->prediction_thread_started&&
           pthread_create(&c->mm305_planner_thread,&attr,mm305_planner_worker,c)==0)
            c->mm305_planner_started=true;
        pthread_attr_destroy(&attr);
    }
    pthread_mutex_unlock(&c->mutex);
}

static void stop_prediction_worker(LandingController *c){
    pthread_mutex_lock(&c->mutex);
    bool join=c->prediction_thread_started;
    c->stop_prediction_thread=true;
    /* The primary control thread is already joined before this function runs.
       Release KSP control now; never leave the last flight command applied while
       waiting several seconds for an advisory shadow prediction to finish. */
    if(c->session)krpc_safe(c->session);
    bool join_planner=c->mm305_planner_started;
    pthread_mutex_unlock(&c->mutex);
    if(join)pthread_join(c->prediction_thread,NULL);
    if(join_planner)pthread_join(c->mm305_planner_thread,NULL);
    pthread_mutex_lock(&c->mutex);
    c->mm305_planner_started=false;
    c->prediction_thread_started=false;
    c->stop_prediction_thread=false;
    c->last_terminal_prediction_request_ut=c->last_terminal_prediction_completion_ut=-1e300;
    c->last_terminal_prediction_completion_wall=-1e300;
    c->last_terminal_prediction_solve_wall=0.0;
    pthread_mutex_unlock(&c->mutex);
}
