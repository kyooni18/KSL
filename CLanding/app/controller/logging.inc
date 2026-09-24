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
    if(c->guidance.entry_s_turn_plan.valid)return &c->guidance.entry_s_turn_plan;
    return NULL;
}

static void append_planner_log(LandingController*c,const char*event,bool explicit_force){
    if(!c->planner_log||!c->snapshot.has_vehicle_state)return;const Telemetry*t=&c->snapshot.telemetry;
    bool constraint_active=t->dynamic_pressure>=c->configuration.vehicle.maximum_dynamic_pressure||t->g_force>=c->configuration.vehicle.maximum_g_load||t->stall_fraction>=1.0;
    bool prediction_matches_plan=!c->prediction_lineage_discontinuity&&entry_forecast_matches_guidance(c);
    PlannerLogSample s={.tick_sequence=c->snapshot.tick_sequence,.ut=t->ut,.phase=c->snapshot.phase,
        .telemetry=t,.snapshot=&c->snapshot,.plan=current_entry_control_plan(c),
        .raw_prediction=prediction_matches_plan?&c->dynamic_trajectory:NULL,
        .published_prediction=prediction_matches_plan?&c->stabilized_trajectory:NULL,
        .calibration=&c->trajectory_calibration,.envelope=&c->envelope,.constraint_active=constraint_active,
        .predictor_discontinuity=!prediction_matches_plan,.explicit_force=explicit_force,.event_reason=event};
    (void)planner_log_record(c->planner_log,&s);
}

static void start_log(LandingController*c){
    close_log(c);c->tick_sequence=0;c->log_sequence=0;c->vehicle_record_sequence=0;c->snapshot.tick_sequence=0;c->snapshot.log_sequence=0;c->log_native_build[0]=0;c->log_planner_path[0]=0;vehicle_log_encoder_init(&c->vehicle_log_encoder);c->log_actual_trajectory_dirty=false;c->log_reference_trajectory_dirty=false;c->log_plan_trajectory_dirty=false;c->log_raw_plan_trajectory_dirty=false;
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
       read-only consumer of that already-selected command and must not mutate
       executable plan timing or synthesize an independent control policy. */
    double commit_remaining=c->guidance.entry_control_plan_valid?
        fmax(0,c->guidance.entry_control_segment_until_ut-t->ut):0;
    EntryControlPlan control_plan={0};
    bool solve_entry_control=c->guidance.automation_engaged&&c->guidance.phase==PHASE_ENTRY_ENERGY;
    if(solve_entry_control&&c->guidance.entry_control_plan_valid)
        control_plan=guidance_control_plan_snapshot(&c->guidance,t->ut,commit_remaining);

    /* Forecast the exact guidance-owned plan. The predictor evaluates the
       resulting trajectory for telemetry; executable MM304 changes come from
       entry_program_guidance() or a worker result that passes plan adoption checks. */
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
            c->plan.live_cutoff_capture_qualified=deorbit_live_cutoff_capture_qualified(&pr,&cfg->site,&cfg->vehicle,&cfg->guidance,cutoff_peri);
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

