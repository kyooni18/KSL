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
    actual_trajectory_update(c,&t);dynamic_prediction(c,&t,&state,&cfg);c->latest_telemetry=t;c->latest_state=state;c->has_latest=true;bool using_glide=c->glide.active;GuidanceResult r=using_glide?glide:guidance_update_with_terminal_model(&c->guidance,&t,&state,c->has_plan?&c->plan:NULL,planet,c->aerodynamics,&cfg,c->terminal_model_valid?&c->terminal_model:NULL);if(using_glide)glide.reference=(Trajectory){0};t.has_guidance_compute=true;t.guidance_compute_ms=(monotonic_seconds()-compute)*1000;command_computed=monotonic_seconds();if(should_time_warp(c,&r,&t)){if(!perform_time_warp(c,&t,err,sizeof(err))){guidance_result_clear(&r);guidance_result_clear(&glide);goto failure;}guidance_result_clear(&r);guidance_result_clear(&glide);return;}GuidanceCommand cmd=r.command;c->last_command=cmd;const char*hud_status=using_glide?glide.status:r.status;const char*hud_warning=(using_glide?glide.has_warning:r.has_warning)?(using_glide?glide.warning:r.warning):"";if(cmd.autopilot_engaged&&!cmd.use_inertial_direction){t.has_command_pitch_error=t.has_command_roll_error=t.has_command_heading_error=true;t.command_pitch_error=cmd.has_target_aoa?cmd.target_aoa-t.angle_of_attack:cmd.target_pitch-t.pitch;t.command_roll_error=norm_signed_deg(cmd.target_roll-t.roll);t.command_heading_error=norm_signed_deg(cmd.target_heading-t.heading);}if(using_glide){apply_attempted=true;if(!apply_command_logged(c,&cmd,cfg.vehicle.airbrake_action_group,PHASE_CALIBRATION,hud_status,hud_warning,NULL,&applied,&apply_started,&apply_finished,err,sizeof(err))){guidance_result_clear(&r);guidance_result_clear(&glide);goto failure;}apply_succeeded=true;t.has_apply_latency=true;t.apply_latency_ms=(apply_finished-apply_started)*1000;}else if(was_cal&&finished)krpc_safe(c->session);else if(r.phase==PHASE_ABORT)krpc_safe(c->session);else if(c->guidance.automation_engaged&&!c->guidance.paused){apply_attempted=true;if(!apply_command_logged(c,&cmd,cfg.vehicle.airbrake_action_group,r.phase,hud_status,hud_warning,&r.reference,&applied,&apply_started,&apply_finished,err,sizeof(err))){guidance_result_clear(&r);guidance_result_clear(&glide);goto failure;}apply_succeeded=true;t.has_apply_latency=true;t.apply_latency_ms=(apply_finished-apply_started)*1000;}if(apply_attempted&&apply_succeeded){KRPCTransportBudget apply_budget=krpc_session_transport_budget(c->session);t.has_rpc_budget=true;t.rpc_apply_calls=apply_budget.apply_calls;t.rpc_apply_wire_requests=apply_budget.apply_wire_requests;t.rpc_total_calls=apply_budget.total_calls;t.rpc_total_wire_requests=apply_budget.total_wire_requests;}t.has_control_loop=true;t.control_loop_ms=(monotonic_seconds()-wall_start)*1000;c->latest_telemetry=t;
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
    c->snapshot.guidance_terminal_candidate_tracking_error=c->guidance.terminal_candidate.tracking_error;
    c->snapshot.guidance_terminal_candidate_tracking_course_error=c->guidance.terminal_candidate.tracking_course_error;
    c->snapshot.guidance_terminal_candidate_tracking_endpoint_error=c->guidance.terminal_candidate.tracking_endpoint_error;
    c->snapshot.guidance_terminal_candidate_tracking_margin=c->guidance.terminal_candidate.tracking_margin;
    c->snapshot.guidance_terminal_candidate_tracking_evaluated=c->guidance.terminal_candidate.tracking_evaluated;
    c->snapshot.guidance_terminal_candidate_path_degraded=c->guidance.taem_safety_handoff||c->guidance.terminal_candidate.join.degraded_path;
    c->snapshot.guidance_terminal_candidate_control_degraded=c->guidance.terminal_candidate.join.degraded_control;
    c->snapshot.guidance_terminal_candidate_rate_degraded=c->guidance.terminal_candidate.join.degraded_rate;
    c->snapshot.guidance_terminal_candidate_end_degraded=c->guidance.terminal_candidate.join.degraded_end;
    c->snapshot.guidance_terminal_reference_fpa=c->guidance.terminal_reference_fpa;
    c->snapshot.guidance_terminal_mix=c->guidance.terminal_mix;
    c->snapshot.tick_sequence=c->tick_sequence;c->snapshot.log_sequence=++c->log_sequence;c->snapshot.wall_monotonic_seconds=wall_start;c->snapshot.simulation_dt=dt;c->snapshot.has_vehicle_state=true;c->snapshot.vehicle_state=state;c->snapshot.has_attitude_quaternion=t.has_attitude_quaternion;if(t.has_attitude_quaternion)memcpy(c->snapshot.attitude_quaternion,t.attitude_quaternion,sizeof(t.attitude_quaternion));snprintf(c->snapshot.attitude_reference_frame,sizeof(c->snapshot.attitude_reference_frame),"%s",t.attitude_reference_frame);c->snapshot.command_computed_wall_seconds=command_computed;c->snapshot.command_apply_attempted=apply_attempted;c->snapshot.command_apply_started_wall_seconds=apply_started;c->snapshot.command_apply_finished_wall_seconds=apply_finished;c->snapshot.command_applied=apply_succeeded&&applied.applied;c->snapshot.applied_command=applied;c->snapshot.guidance_has_burn_started=c->guidance.has_burn_command_started;c->snapshot.guidance_burn_completed=c->guidance.deorbit_burn_completed;c->snapshot.guidance_atmosphere_crossed=c->guidance.atmospheric_interface_crossed;c->snapshot.guidance_final_captured=c->guidance.final_approach_captured;c->snapshot.guidance_airbrakes_deployed=c->guidance.airbrakes_deployed;c->snapshot.guidance_hac_side_selected=c->guidance.hac_side_selected;c->snapshot.guidance_delivered_delta_v=c->guidance.delivered_delta_v;c->snapshot.guidance_burn_active_elapsed=c->guidance.burn_active_elapsed;c->snapshot.guidance_s_turn_sign=c->guidance.s_turn_sign;c->snapshot.guidance_hac_side=c->guidance.hac_side;c->snapshot.guidance_entry_leg_elapsed=guidance_entry_leg_elapsed(&c->guidance,t.ut);c->snapshot.guidance_entry_exec=entry_exec_telemetry(&c->guidance.entry_exec);c->snapshot.guidance_taem_exec=taem_exec_telemetry(&c->guidance.taem_exec);c->snapshot.guidance_hac_captured=c->guidance.hac_captured;c->snapshot.guidance_hac_completed=c->guidance.hac_completed;c->snapshot.guidance_hac_progress_valid=c->guidance.hac_progress_valid;c->snapshot.guidance_hac_transition_active=c->guidance.hac_transition_active;c->snapshot.guidance_hac_remaining=c->guidance.hac_remaining;c->snapshot.guidance_hac_radius=c->guidance.hac_radius;c->snapshot.guidance_hac_circuit_count=c->guidance.hac_circuit_count;c->snapshot.guidance_hac_circuit_slope=c->guidance.hac_circuit_slope;c->snapshot.guidance_terminal_reentry_after_ut=c->guidance.terminal_reentry_after_ut;c->snapshot.guidance_minimum_turn_radius=c->guidance.minimum_turn_radius;c->snapshot.guidance_hac_transition_progress=c->guidance.hac_transition_progress;c->snapshot.guidance_hac_transition_points[0]=c->guidance.hac_transition_p0_e;c->snapshot.guidance_hac_transition_points[1]=c->guidance.hac_transition_p0_n;c->snapshot.guidance_hac_transition_points[2]=c->guidance.hac_transition_p1_e;c->snapshot.guidance_hac_transition_points[3]=c->guidance.hac_transition_p1_n;c->snapshot.guidance_hac_transition_points[4]=c->guidance.hac_transition_p2_e;c->snapshot.guidance_hac_transition_points[5]=c->guidance.hac_transition_p2_n;c->snapshot.guidance_hac_transition_points[6]=c->guidance.hac_transition_p3_e;c->snapshot.guidance_hac_transition_points[7]=c->guidance.hac_transition_p3_n;c->snapshot.guidance_hac_transition_heading_cone=c->guidance.hac_transition_heading_cone;c->snapshot.guidance_hac_transition_lead_curve=c->guidance.hac_transition_lead_curve;c->snapshot.guidance_hac_transition_lead_points[0]=c->guidance.hac_transition_lead_start_e;c->snapshot.guidance_hac_transition_lead_points[1]=c->guidance.hac_transition_lead_start_n;c->snapshot.guidance_hac_transition_lead_points[2]=c->guidance.hac_transition_lead_p1_e;c->snapshot.guidance_hac_transition_lead_points[3]=c->guidance.hac_transition_lead_p1_n;c->snapshot.guidance_hac_transition_lead_points[4]=c->guidance.hac_transition_lead_p2_e;c->snapshot.guidance_hac_transition_lead_points[5]=c->guidance.hac_transition_lead_p2_n;c->snapshot.guidance_hac_transition_cone_center[0]=c->guidance.hac_transition_cone_center_e;c->snapshot.guidance_hac_transition_cone_center[1]=c->guidance.hac_transition_cone_center_n;c->snapshot.guidance_hac_transition_cone_start_angle=c->guidance.hac_transition_cone_start_angle;c->snapshot.guidance_hac_transition_cone_end_angle=c->guidance.hac_transition_cone_end_angle;c->snapshot.guidance_hac_transition_cone_arc_length=c->guidance.hac_transition_cone_arc_length;double lift_accel=t.mass>1&&isfinite(t.lift_force)&&t.lift_force>0?t.lift_force/t.mass:0;double bank_for_rate=clampd(norm_signed_deg(cmd.target_roll)*clampd(t.bank_effectiveness,.35,1.8),-89,89);c->snapshot.guidance_commanded_course_rate_estimate=t.true_air_speed>1&&lift_accel>0?lift_accel*sin(bank_for_rate*DEG2RAD)/t.true_air_speed*RAD2DEG:0;
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
        double cadence_anchor=fmax(c->last_terminal_prediction_request_ut,
            c->last_terminal_prediction_completion_ut);
        bool request_outstanding=c->last_terminal_prediction_request_ut>
            c->last_terminal_prediction_completion_ut+1e-6;
        bool request_due=c->has_latest&&async_prediction_request_due(
            c->latest_telemetry.ut,cadence_anchor,monotonic_seconds(),
            c->last_terminal_prediction_completion_wall,c->last_terminal_prediction_solve_wall,
            c->configuration.guidance.prediction_interval);
        bool sim_plan_barrier=fast_sim&&c->guidance.phase==PHASE_ENTRY_ENERGY&&
            (c->guidance.entry_planning_needed||c->guidance.entry_final_reversal_pending||
             c->guidance.entry_final_reversal_completed)&&
            (request_outstanding||request_due);
        double barrier_ut=c->has_latest?c->latest_telemetry.ut:-INFINITY;
        uint64_t barrier_generation=c->prediction_generation;
        pthread_mutex_unlock(&c->mutex);

        if(sim_plan_barrier){
            for(;;){
                sleep_seconds(.001);
                pthread_mutex_lock(&c->mutex);
                bool stop=c->stop_thread||!c->session||
                    !krpc_session_is_simulator(c->session)||!c->guidance.automation_engaged||
                    c->guidance.paused||c->guidance.phase!=PHASE_ENTRY_ENERGY||
                    c->prediction_generation!=barrier_generation;
                /* A displayed candidate is not an executable worker result.
                   Hold only an outstanding/due request, and do not let a wall
                   timeout advance hundreds of simulated seconds past it. The
                   worker credits completion at the frozen latest telemetry UT. */
                bool completed=c->last_terminal_prediction_completion_ut>=barrier_ut-1e-6;
                pthread_mutex_unlock(&c->mutex);
                if(stop||completed)break;
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

