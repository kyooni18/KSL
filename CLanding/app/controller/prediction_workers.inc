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
        bool entry_job=false,entry_planning_job=false;uint64_t generation=0;
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
        bool had_entry_topology=guidance.entry_topology.valid;
        if(entry_job){
            /* Search and replay own only this immutable snapshot, outside the
               controller mutex. Live guidance keeps running fresh safety laws. */
            guidance.entry_planning_deferred=false;
            if(!isfinite(guidance.entry_s_turn_plan.cost)||
               (guidance.entry_lateral_infeasible&&!guidance.entry_reversal_scheduled))
                guidance.entry_control_plan_valid=false; /* initial fallback needs a real topology now */
            guidance.diagnostic_shadow=true;
            GuidanceResult proposal=guidance_update(&guidance,&telemetry,&state,
                has_plan?&plan:NULL,&planet,aero,&cfg);
            guidance_result_clear(&proposal);
            bool topology_search_needed=entry_topology_search_needed(&guidance);
            if(topology_search_needed){
                /* First let the bounded entry planner try the live state. If it
                   cannot propagate a reversal, search the complete two-event
                   route against the same published interface before accepting
                   a provisional same-side bank command. */
                double worker_course=isfinite(telemetry.ground_track_heading)?
                    telemetry.ground_track_heading:telemetry.heading;
                entry_publish_taem_tangent_target(&guidance,&telemetry,worker_course,&planet,aero,&cfg);
                EntryTopologyPlan topology=predictor_plan_entry_topology(state,&telemetry,&guidance,&planet,
                    &envelope,&calibration,&cfg);
                if(topology.valid&&guidance_install_entry_topology(&guidance,&topology,telemetry.ut)){
                    GuidanceResult topology_proposal=guidance_update(&guidance,&telemetry,&state,
                        has_plan?&plan:NULL,&planet,aero,&cfg);
                    guidance_result_clear(&topology_proposal);
                }
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
        if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
            fprintf(stderr,"ENTRY_WORKER finish ut=%.3f wall=%.3f topology=%d capture_ut=%.3f reached=%d captured=%d aborted=%d final_ut=%.3f\n",
                request_ut,solve_finished-solve_started,guidance.entry_topology.valid,
                guidance.entry_topology.capture_ut,pr.reached_taem,
                pr.taem_dynamic_interface_captured,pr.shadow_aborted,pr.final_state.ut);
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
            if(getenv("KSP_LANDER_ENTRY_WORKER_TRACE"))
                fprintf(stderr,"ENTRY_WORKER adoption request_ut=%.3f live_ut=%.3f adopted=%d topology=%d final_pending=%d\n",
                    request_ut,c->latest_telemetry.ut,adopted,c->guidance.entry_topology.valid,
                    c->guidance.entry_final_reversal_pending);
            if((adopted||tangent_forecast)&&request_ut>c->cached_prediction_ut){
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
        if(c->session&&c->has_latest&&guidance_terminal_preview_allowed(g)&&
           g->automation_engaged&&!g->paused&&!g->aborted&&
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

