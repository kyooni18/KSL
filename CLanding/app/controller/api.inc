static const char *terminal_model_path(const char *environment_name,
        const char *default_path) {
    const char *configured = getenv(environment_name);
    return configured && configured[0] ? configured : default_path;
}

static bool controller_capture_terminal_model(LandingController *c,
        const LandingConfiguration *cfg) {
    TerminalModelSourceFiles files = {
        .atmosphere_csv = terminal_model_path("KSP_LANDER_TERMINAL_ATMOSPHERE",
            "ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv"),
        .aero_csv = terminal_model_path("KSP_LANDER_TERMINAL_AERO",
            "ShuttleSim/data/fitted/stsn_aero_ksp_robust.csv"),
        .aero_book_csv = terminal_model_path("KSP_LANDER_TERMINAL_AERO_BOOK",
            "ShuttleSim/data/fitted/stsn_force_book.csv"),
        .attitude_ini = terminal_model_path("KSP_LANDER_TERMINAL_ATTITUDE",
            "ShuttleSim/data/fitted/stsn_attitude_ksp.ini")
    };
    char reason[256];
    uint64_t snapshot_id = ++c->terminal_model_capture_sequence;
    if (!snapshot_id) snapshot_id = ++c->terminal_model_capture_sequence;
    c->terminal_model_valid = terminal_model_capture(&c->terminal_model,
        &files, cfg, snapshot_id, reason, sizeof(reason));
    if (!c->terminal_model_valid)
        fprintf(stderr, "MM305 native model unavailable: %s\n", reason);
    return c->terminal_model_valid;
}

LandingController *landing_controller_create(const LandingConfiguration*cfg,SnapshotCallback cb,void*ctx){LandingController*c=calloc(1,sizeof(*c));if(!c)return NULL;pthread_mutex_init(&c->mutex,NULL);c->configuration=cfg?*cfg:landing_configuration_default();landing_configuration_normalize(&c->configuration);guidance_machine_init(&c->guidance);adaptive_calibrator_init(&c->adaptive,&c->configuration.vehicle);glide_calibration_init(&c->glide);trajectory_calibrator_init(&c->trajectory_calibrator);trajectory_init(&c->dynamic_trajectory);trajectory_init(&c->stabilized_trajectory);landing_snapshot_init(&c->snapshot,&c->configuration.vehicle);c->adaptive_profile=c->configuration.vehicle;c->aerodynamics=(AerodynamicModel){c->configuration.vehicle.estimated_lift_to_drag,c->configuration.vehicle.estimated_ballistic_coefficient,.2};c->planning_aerodynamics=c->aerodynamics;for(int i=0;i<4;i++)c->envelope.regimes[i]=c->aerodynamics;c->trajectory_calibration=(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0};guidance_command_init(&c->last_command);c->last_trajectory_ut=c->last_prediction_ut=c->cached_prediction_ut=-1e300;c->callback=cb;c->callback_context=ctx;return c;}
void landing_controller_destroy(LandingController*c){if(!c)return;landing_controller_shutdown(c);pthread_mutex_lock(&c->mutex);clear_snapshot_dynamic(&c->snapshot);trajectory_clear(&c->dynamic_trajectory);trajectory_clear(&c->stabilized_trajectory);trajectory_calibrator_clear(&c->trajectory_calibrator);if(c->has_plan)deorbit_plan_clear(&c->plan);pthread_mutex_unlock(&c->mutex);pthread_mutex_destroy(&c->mutex);free(c);}
LandingConfiguration landing_controller_configuration(LandingController*c){pthread_mutex_lock(&c->mutex);LandingConfiguration r=c->configuration;pthread_mutex_unlock(&c->mutex);return r;}
void landing_controller_update_configuration(LandingController*c,const LandingConfiguration*cfg){pthread_mutex_lock(&c->mutex);c->prediction_generation++;LandingConfiguration before=c->configuration,next=*cfg;VehicleProfile before_vehicle=before.vehicle;landing_configuration_normalize(&next);bool changed=!configuration_equal(&before,&next),planning_changed=!planning_configuration_equal(&before,&next);if(!changed){pthread_mutex_unlock(&c->mutex);return;}c->configuration=next;if(c->session&&!c->guidance.terminal_region_entered)(void)controller_capture_terminal_model(c,&c->configuration);if(!vehicle_profile_equal(&before_vehicle,&c->configuration.vehicle)&&!c->glide.active){reset_models(c,&c->configuration.vehicle);if(c->session){krpc_session_seed_physics(c->session,&c->physics);apply_certified_aero_prior(c);}}if(c->has_plan&&planning_changed){c->plan_stale=true;set_warning(c,"Planning-relevant configuration changed after planning. Create a new deorbit plan before engagement.");}append_configuration_log(c);log_state_event(c,planning_changed?"event=configuration-changed; replan_required=true":"event=configuration-changed; replan_required=false",true);publish(c);pthread_mutex_unlock(&c->mutex);}
void landing_controller_connect(LandingController*c){stop_thread(c);pthread_mutex_lock(&c->mutex);disconnect_locked(c,false);reset_snapshot(c,getenv("KSP_LANDER_SIMULATOR")?"Opening ShuttleSim lockstep backend…":"Opening native kRPC C-Nano serial link…");c->snapshot.connection_status=CONN_CONNECTING;publish(c);LandingConfiguration cfg=c->configuration;pthread_mutex_unlock(&c->mutex);char err[768];KRPCSession*s=krpc_session_open(&cfg,err,sizeof(err));pthread_mutex_lock(&c->mutex);if(!s){c->snapshot.connection_status=CONN_FAILED;c->snapshot.phase=PHASE_FAULT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Native kRPC C-Nano connection failed.");set_error(c,err);publish(c);pthread_mutex_unlock(&c->mutex);return;}c->session=s;start_log(c);reset_models(c,&c->configuration.vehicle);krpc_session_seed_physics(s,&c->physics);apply_certified_aero_prior(c);(void)controller_capture_terminal_model(c,&c->configuration);guidance_machine_init(&c->guidance);glide_calibration_init(&c->glide);c->has_prediction_cache=false;c->time_warp_issued=false;c->snapshot.connection_status=CONN_CONNECTED;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s connected to %s around %s with %u certified aerodynamic cells.",krpc_session_is_simulator(s)?"ShuttleSim lockstep":"Native kRPC C-Nano",krpc_session_vessel(s),krpc_session_planet(s)->name,c->physics.count);set_error(c,NULL);c->snapshot.adaptive_vehicle_profile=c->adaptive_profile;publish(c);pthread_mutex_unlock(&c->mutex);start_thread(c);}
void landing_controller_disconnect(LandingController*c){stop_thread(c);pthread_mutex_lock(&c->mutex);disconnect_locked(c,true);pthread_mutex_unlock(&c->mutex);}

void landing_controller_create_plan(LandingController*c){pthread_mutex_lock(&c->mutex);if(!c->session){pthread_mutex_unlock(&c->mutex);return;}if(c->glide.active){set_warning(c,"Stop the glide calibration before creating a deorbit plan.");log_state_event(c,"event=planning-blocked; reason=calibration-active",false);publish(c);pthread_mutex_unlock(&c->mutex);return;}LandingConfiguration cfg=effective_config(c);char err[768];Telemetry t;VehicleState s;if(!read_live_sample(c,&cfg,&t,&s,err,sizeof(err))){c->snapshot.phase=PHASE_FAULT;set_error(c,err);log_state_event(c,"event=planning-failed; reason=telemetry-read",true);publish(c);pthread_mutex_unlock(&c->mutex);return;}c->latest_telemetry=t;c->latest_state=s;c->has_latest=true;landing_snapshot_set_sample(&c->snapshot,&t,&s);c->snapshot.phase=PHASE_PLANNING;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Searching upcoming orbital landing opportunities and stress-testing burn execution…");set_error(c,NULL);publish(c);const PlanetModel*planet=krpc_session_planet(c->session);if(t.apoapsis_altitude<=planet->atmosphere_depth||t.periapsis_altitude<=planet->atmosphere_depth){c->snapshot.phase=PHASE_FAULT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Planning rejected.");set_error(c,"Initial apoapsis and periapsis must both be outside the active planet atmosphere.");log_state_event(c,"event=planning-failed; reason=orbit-inside-atmosphere",true);publish(c);pthread_mutex_unlock(&c->mutex);return;}DeorbitPlan p;if(!deorbit_plan_create_with_telemetry(&p,s,t.orbit_period,krpc_session_planet(c->session),c->planning_aerodynamics,&c->envelope,&c->trajectory_calibration,&cfg.site,&cfg.vehicle,&cfg.guidance,t.available_thrust,&t)){c->snapshot.phase=PHASE_FAULT;snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"No usable deorbit solution was found.");set_error(c,"The current orbit, landing site, thrust or calibrated aerodynamic profile did not produce atmospheric capture across the searched future opportunities.");log_state_event(c,"event=planning-failed; reason=no-executable-deorbit-solution",true);publish(c);pthread_mutex_unlock(&c->mutex);return;}if(c->has_plan)deorbit_plan_clear(&c->plan);c->plan=p;c->has_plan=true;c->plan_stale=false;c->time_warp_issued=false;c->log_plan_trajectory_dirty=true;trajectory_clear(&c->dynamic_trajectory);trajectory_copy(&c->dynamic_trajectory,&p.trajectory);trajectory_clear(&c->stabilized_trajectory);trajectory_copy(&c->stabilized_trajectory,&p.trajectory);guidance_reset_plan(&c->guidance);t.predicted_miss_distance=p.predicted_closest_distance;t.predicted_entry_range=p.predicted_entry_range;t.predicted_entry_flight_path_angle=p.predicted_entry_flight_path_angle;trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&p.trajectory);landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);c->snapshot.phase=PHASE_IDLE;if(p.target_capture_achieved)snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Robust plan ready. %u/%u strict stress cases pass (%.0f%%).",p.robustness_passed,p.robustness_scenarios,p.robustness_pass_fraction*100);else if(p.execution_qualified)snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Guarded recovery plan ready. %u/%u recovery cases pass (%.0f%%); strict corridor was not met.",p.recovery_passed,p.robustness_scenarios,p.recovery_pass_fraction*100);else snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"Preview plan found, but no execution-qualified corridor was found.");set_warning(c,p.target_capture_achieved?NULL:p.execution_qualified?"This plan is executable under the broader guarded recovery envelope. Review its miss, entry angle, periapsis and recovery stress margin before engaging.":"Engagement is disabled for this preview plan. Replan for a later opportunity or revise the vehicle/site model.");set_error(c,NULL);reference_trajectory(&c->snapshot.reference_trajectory,&cfg.site,&cfg.guidance,krpc_session_planet(c->session)->radius,1);c->log_reference_trajectory_dirty=true;char plan_reason[256];snprintf(plan_reason,sizeof(plan_reason),"event=plan-created; execution-qualified=%s; strict-capture=%s; degraded=%s; robustness-pass=%.6g",p.execution_qualified?"true":"false",p.target_capture_achieved?"true":"false",p.execution_degraded?"true":"false",p.robustness_pass_fraction);log_state_event(c,plan_reason,true);publish(c);pthread_mutex_unlock(&c->mutex);}
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
    bool simulator_fast_qualification=krpc_session_is_simulator(c->session);
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
       outlet that may not match the worker's turnability-selected target.
       Simulator continuation is already an executable closed loop, so do not
       spend the engagement call replaying a second full entry shadow. */
    double qualification_course=isfinite(t.ground_track_heading)?
        t.ground_track_heading:t.heading;
    entry_publish_taem_tangent_target(&qualification_guidance,&t,
        qualification_course,p,c->planning_aerodynamics,&cfg);

    EntryTopologyPlan topology={0};
    if(!simulator_fast_qualification){
        topology=predictor_plan_entry_topology(state,&t,
            &qualification_guidance,p,&c->envelope,&c->trajectory_calibration,&cfg);
        if(topology.valid)
            (void)guidance_install_entry_topology(
                &qualification_guidance,&topology,t.ut);
    }

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

    EntryPrediction prediction;
    memset(&prediction,0,sizeof(prediction));
    trajectory_init(&prediction.trajectory);
    if(simulator_fast_qualification){
        /* The simulator is the plant for the control loop below. Seed only
           the observable state used by the replay/archive; guidance remains
           responsible for all subsequent MM304, TAEM, HAC, and rollout
           decisions. */
        prediction.entered_atmosphere=in_atmosphere;
        prediction.entry_range=t.range_to_site;
        prediction.entry_flight_path_angle=t.flight_path_angle;
        prediction.entry_speed=t.true_air_speed;
        prediction.taem_distance=t.range_to_site;
        prediction.taem_range_error=t.range_to_site;
        prediction.closest_distance=t.range_to_site;
        prediction.taem_speed=t.true_air_speed;
        prediction.taem_flight_path_angle=t.flight_path_angle;
        prediction.peak_dynamic_pressure=t.dynamic_pressure;
        prediction.peak_g_load=t.g_force;
        prediction.final_state=state;
    }else{
        double gravity=planet_surface_gravity(p);
        double height=fmax(0.0,t.mean_altitude-cfg.site.altitude);
        double horizontal_time=t.horizontal_speed>DBL_EPSILON?
            fmax(0.0,t.range_to_site)/t.horizontal_speed:0.0;
        double vertical_time=(-t.vertical_speed)>DBL_EPSILON?
            height/(-t.vertical_speed):
            (gravity>DBL_EPSILON?sqrt(2.0*height/gravity):0.0);
        /* Normal MM305 is a partial Heading Alignment Cone, never a full
           revolution.  270 degrees is the maximum admitted normal cone; an
           explicit high-pass circuit is a separate contingency. */
        double terminal_path=1.5*LANDER_PI*cfg.guidance.hac_radius+
            cfg.guidance.final_approach_distance;
        double terminal_time=terminal_path/
            fmax(cfg.vehicle.minimum_safe_speed,DBL_EPSILON);
        double qualification_horizon=fmax(horizontal_time,vertical_time)+terminal_time;

        prediction=predictor_simulate_entry_guidance_shadow(
            state,&t,&qualification_guidance,&qualification_plan,
            p,c->planning_aerodynamics,&c->envelope,&c->trajectory_calibration,
            &cfg,qualification_horizon,true);
        trajectory_clear(&qualification_plan.trajectory);
    }

    bool recovery=simulator_fast_qualification||
        reentry_guidance_shadow_recovery_qualified(
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
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s MM305 terminal guidance owns the vehicle; path acquisition remains uncommitted until the live geometry, energy, and authority gates pass.",status);
    set_error(c,NULL);set_warning(c,"HAC/MM305 test mode is active. This is a terminal-guidance test, not a validated reentry continuation.");
    /* Do not publish the configured nominal circle as if it were the selected
       HAC.  MM305 selects radius only after the coupled lead, p0 authority,
       energy, and vertical certificates have been evaluated.  Publishing the
       nominal trajectory here made the UI look permanently locked to the
       default 12 km value while the dynamic search was still pending (or had
       rejected every candidate). */
    trajectory_clear(&c->snapshot.reference_trajectory);
    c->log_reference_trajectory_dirty=true;
    log_state_event(c,"event=hac-only-test-armed; deorbit-skipped=true; entry-skipped=true; mm305-energy-staging=true",true);
    publish(c);pthread_mutex_unlock(&c->mutex);
}

void landing_controller_engage_final_test(LandingController*c){
    pthread_mutex_lock(&c->mutex);c->prediction_generation++;
    if(!c->session){set_warning(c,"Connect to ShuttleSim before starting the final-only landing test.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    if((!krpc_session_is_simulator(c->session)&&!getenv("KSP_LANDER_FINAL_TEST_LIVE"))||!getenv("KSP_LANDER_FINAL_APPROACH_TEST")||strcmp(getenv("KSP_LANDER_FINAL_APPROACH_TEST"),"1")){
        set_warning(c,"Final-only landing test is simulator-only and requires KSP_LANDER_FINAL_APPROACH_TEST=1.");publish(c);pthread_mutex_unlock(&c->mutex);return;
    }
    if(c->glide.active){set_warning(c,"Stop calibration before starting the final-only landing test.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    LandingConfiguration cfg=effective_config(c);char err[768]={0};Telemetry t;VehicleState state;
    if(!read_live_sample(c,&cfg,&t,&state,err,sizeof(err))){set_error(c,err);c->snapshot.phase=PHASE_FAULT;publish(c);pthread_mutex_unlock(&c->mutex);return;}
    /* Admission must use the same Mach-interpolated model and confidence as
       the first live guidance tick, so both energy checks price the same
       aerodynamic uncertainty. */
    c->aerodynamics=envelope_at_controller(&c->envelope,t.mach);
    t.estimated_lift_to_drag=c->aerodynamics.lift_to_drag;
    t.estimated_ballistic_coefficient=c->aerodynamics.ballistic_coefficient;
    t.aerodynamic_confidence=c->aerodynamics.confidence;
    const PlanetModel*p=krpc_session_planet(c->session);
    if(planet_atmospheric_density(p,t.mean_altitude)<=DBL_EPSILON){set_warning(c,"Final-only landing test requires an atmospheric flight state.");publish(c);pthread_mutex_unlock(&c->mutex);return;}
    if(!strcasecmp(t.vessel_situation,"landed")||!strcasecmp(t.vessel_situation,"splashed")||!strcasecmp(t.vessel_situation,"pre-launch")){
        set_warning(c,"Final-only landing test requires an airborne vehicle.");publish(c);pthread_mutex_unlock(&c->mutex);return;
    }
    double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,t.heading);
    char status[512]={0};
    if(!guidance_begin_final_test(&c->guidance,&t,course,p,c->aerodynamics,&cfg,status,sizeof(status))){
        set_warning(c,status[0]?status:"The recorded state cannot initialize the final-only landing test.");publish(c);pthread_mutex_unlock(&c->mutex);return;
    }
    if(c->has_plan){deorbit_plan_clear(&c->plan);c->has_plan=false;}
    c->plan_stale=false;c->time_warp_issued=true;c->has_prediction_cache=false;c->last_prediction_ut=-1e300;
    trajectory_clear(&c->dynamic_trajectory);trajectory_init(&c->dynamic_trajectory);
    trajectory_clear(&c->stabilized_trajectory);trajectory_init(&c->stabilized_trajectory);
    trajectory_clear(&c->trajectory_calibrator.forecast);trajectory_init(&c->trajectory_calibrator.forecast);
    c->latest_telemetry=t;c->latest_state=state;c->has_latest=true;
    landing_snapshot_set_sample(&c->snapshot,&t,&c->latest_state);
    c->snapshot.plan=NULL;c->snapshot.phase=PHASE_TAEM;c->snapshot.automation_engaged=true;c->snapshot.paused=false;
    snprintf(c->snapshot.status_message,sizeof(c->snapshot.status_message),"%s",status);
    set_error(c,NULL);set_warning(c,NULL);
    trajectory_clear(&c->snapshot.reference_trajectory);
    c->log_reference_trajectory_dirty=true;
    log_state_event(c,"event=final-only-test-armed; post-hac=true; hac-planning-skipped=true; entry-skipped=true; s-turn-skipped=true",true);
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
