
typedef struct {
    bool terminal_prediction_valid;
    bool terminal_policy_feasible;
    bool terminal_path_committed;
    bool final_approach_captured;
    bool aborted;
} TerminalShadowOutcome;

static Telemetry guidance_shadow_telemetry(const Telemetry*base,VehicleState state,
        PredictorGuidance*response,AeroContext*ctx,const PlanetModel*p,
        const LandingConfiguration*cfg,AtmosState*out_at){
    Telemetry t;memset(&t,0,sizeof(t));if(base)t=*base;
    ctx->ut=state.ut;ctx->mass=state.mass;ctx->bank=response->actual_bank;ctx->aoa=response->actual_aoa;
    AtmosState at=atmosphere(state.position,state.velocity,ctx);if(out_at)*out_at=at;
    GeoPoint geo=predictor_geo_point(state.position,p,state.ut),target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    Vector3 up=vnorm(state.position,v3(0,1,0));double vs=vdot(at.air_velocity,up);
    double hs=vmag(vproject_plane(at.air_velocity,up));double fpa=atan2(vs,fmax(hs,.1))*RAD2DEG;
    double fallback=initial_bearing(geo,target);
    double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,fallback);
    double along=0,cross=0;runway_coordinates(geo,target,cfg->site.runway_heading,p->radius,&along,&cross);
    double turn_rate=at.speed>1?at.lift_accel*sin(response->actual_bank*DEG2RAD)/at.speed*RAD2DEG:0;
    t.ut=state.ut;t.latitude=geo.latitude;t.longitude=geo.longitude;t.mean_altitude=geo.altitude;
    t.radar_altitude=fmax(0,geo.altitude-cfg->site.altitude);t.vertical_speed=vs;t.horizontal_speed=hs;
    t.surface_speed=hs;t.true_air_speed=at.speed;t.atmospheric_density=at.density;t.speed_of_sound=at.speed_of_sound;t.mach=at.mach;
    t.heading=course;t.ground_track_heading=course;t.course_to_site_error=norm_signed_deg(fallback-course);
    t.pitch=fpa+response->actual_aoa;t.roll=response->actual_bank;
    t.pitch_rate=response->actual_aoa_rate;t.roll_rate=response->actual_bank_rate;t.heading_rate=turn_rate;
    t.has_angle_of_attack_rate=true;t.angle_of_attack_rate=response->actual_aoa_rate;
    /* The reduced shadow integrates bank/AoA coordinate rates, not body-axis p/q.
       Do not masquerade these state derivatives as rigid-body angular rates. */
    t.has_body_pitch_rate=false;t.has_body_roll_rate=false;t.has_body_yaw_rate=false;t.has_course_rate=true;
    t.body_pitch_rate=0;t.body_roll_rate=0;t.body_yaw_rate=0;t.course_rate=turn_rate;t.autopilot_error=0;
    t.has_command_pitch_error=false;t.has_command_roll_error=false;t.has_command_heading_error=false;
    t.angle_of_attack=response->actual_aoa;t.sideslip=0;t.dynamic_pressure=at.q;
    t.static_pressure=planet_atmospheric_pressure(p,geo.altitude);
    t.g_force=at.non_gravity/fmax(planet_surface_gravity(p),.1);
    t.stall_fraction=vessel_physics_conservative_stall_fraction(t.true_air_speed,t.angle_of_attack,t.dynamic_pressure,&cfg->vehicle);
    t.stall_fraction_is_measured=false;
    /* A reduced attitude model cannot declare recovery from a measured stall
       while it still carries the observed incidence. Retain that safety evidence. */
    if(base&&base->stall_fraction_is_measured&&response->actual_aoa>=base->angle_of_attack-1.0){
        t.stall_fraction=fmax(t.stall_fraction,base->stall_fraction);
        t.stall_fraction_is_measured=true;
    }
    t.lift_force=fabs(at.lift_accel)*fmax(state.mass,1);t.drag_force=fabs(at.drag_accel)*fmax(state.mass,1);
    t.has_force_vectors=false;t.physics_sample_valid=false;t.mass=state.mass;t.available_thrust=0;t.current_thrust=0;t.throttle=0;
    t.has_controls=false;t.control_pitch=0;t.control_roll=0;t.control_yaw=0;
    t.gear=ctx->gear;t.brakes=ctx->brakes;t.has_airbrakes=true;t.airbrakes=ctx->airbrakes!=0;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    t.range_to_site=great_circle_distance(geo,target,p->radius);t.bearing_to_site=fallback;t.heading_error=norm_signed_deg(fallback-course);
    t.runway_along_track=along;t.runway_cross_track=cross;t.flight_path_angle=fpa;
    t.aerodynamic_confidence=clampd(fmax(at.physics_confidence,base?base->aerodynamic_confidence:0),0,1);
    t.physics_confidence=at.physics_confidence;t.physics_certified_uncertainty=at.physics_uncertainty;
    if(ctx->c){
        t.trajectory_density_scale=ctx->c->density_scale;t.trajectory_drag_scale=ctx->c->drag_scale;
        t.trajectory_lift_scale=ctx->c->lift_scale;t.bank_effectiveness=ctx->c->bank_effectiveness;
        t.trajectory_calibration_confidence=ctx->c->confidence;t.trajectory_altitude_residual=ctx->c->altitude_residual;
        t.trajectory_speed_residual=ctx->c->speed_residual;t.trajectory_range_residual=ctx->c->range_residual;
    }
    t.energy_excess_range=NAN;
    return t;
}

static bool terminal_shadow_survives(const EntryPrediction*pr,const TerminalShadowOutcome*outcome,
        const VehicleProfile*v){
    if(!pr||!outcome||!v||outcome->aborted)return false;
    if(pr->peak_dynamic_pressure>v->maximum_dynamic_pressure||
       pr->peak_g_load>v->maximum_g_load)return false;
    if(isfinite(pr->minimum_entry_speed)&&
       pr->minimum_entry_speed<v->minimum_safe_speed)return false;
    return outcome->final_approach_captured||outcome->terminal_path_committed||outcome->terminal_policy_feasible;
}

static EntryPrediction predictor_simulate_guidance_shadow_core(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const LandingConfiguration*cfg,
        double maxdur,bool include_trajectory,
        bool require_terminal_start,bool stop_at_taem,
        const TerminalModel*terminal_model,TerminalShadowOutcome*outcome){
    EntryPrediction out;memset(&out,0,sizeof(out));trajectory_init(&out.trajectory);
    out.closest_distance=DBL_MAX;out.taem_distance=DBL_MAX;
    out.taem_altitude=NAN;out.taem_desired_altitude=NAN;
    out.minimum_entry_speed=DBL_MAX;out.minimum_bank_control_margin=INFINITY;out.minimum_aoa_control_margin=INFINITY;
    if(outcome)memset(outcome,0,sizeof(*outcome));
    if(!telemetry||!guidance||!p||!cfg||!guidance->automation_engaged)return out;
    if(require_terminal_start&&!taem_exec_owns_vehicle(&guidance->taem_exec))return out;

    AerodynamicEnvelope uniform;if(!env0){for(int i=0;i<4;i++)uniform.regimes[i]=aero;env0=&uniform;}
    TrajectoryCalibrationModel dc={.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,
        .speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.confidence=.05};
    if(!cal0)cal0=&dc;
    GuidanceMachine shadow=*guidance;
    shadow.diagnostic_shadow=true;
    shadow.diagnostic_stop_at_taem=stop_at_taem;
    /* Prediction is advisory only. Replaying it must never invoke a second
       MM304 planner or mutate the executable Entry policy. */
    shadow.entry_planning_deferred=true;
    shadow.has_previous_ut=true;shadow.previous_ut=state.ut-.1;
    guidance_set_entry_predictor_models(&shadow,env0,cal0);
    PredictorGuidance response;memset(&response,0,sizeof(response));
    response.actual_bank=norm_signed_deg(telemetry->roll);
    response.actual_bank_rate=isfinite(telemetry->roll_rate)?telemetry->roll_rate:0.0;
    response.actual_aoa=isfinite(telemetry->angle_of_attack)?telemetry->angle_of_attack:cfg->vehicle.entry_angle_of_attack;
    response.actual_aoa_rate=telemetry->has_angle_of_attack_rate&&isfinite(telemetry->angle_of_attack_rate)?
        telemetry->angle_of_attack_rate:0.0;
    response.limiter.has_value=true;response.limiter.value=norm_deg(response.actual_bank);
    bool initial_gear=telemetry->gear,initial_brakes=telemetry->brakes;
    int initial_airbrakes=telemetry->has_airbrakes&&telemetry->airbrakes?1:0;
    AeroContext ctx={.p=p,.e=env0,.c=cal0,.v=&cfg->vehicle,.bank=response.actual_bank,.aoa=response.actual_aoa,
        .ut=state.ut,.mass=state.mass,.gear=initial_gear,.brakes=initial_brakes,.airbrakes=initial_airbrakes};
    GeoPoint target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double elapsed=0,lastsample=-DBL_MAX;
    out.taem_interface_error=INFINITY;
    out.taem_interface_energy_margin=INFINITY;
    const char*entry_witness_trace_env=getenv("KSP_LANDER_TRACE_ENTRY_WITNESS");
    bool entry_witness_trace=entry_witness_trace_env&&
        strcmp(entry_witness_trace_env,"1")==0;
    bool entry_witness_phase_traced=false;
    double entry_alt=p->atmosphere_depth-cfg->guidance.entry_interface_altitude_margin;
    out.entered_atmosphere=telemetry->mean_altitude<=entry_alt&&telemetry->vertical_speed<0.0;
    bool initial_terminal_owned=taem_exec_owns_vehicle(&shadow.taem_exec);
    bool initial_degraded_safety_handoff=initial_terminal_owned&&shadow.taem_safety_handoff&&
        !shadow.taem_interface_captured;
    /* A replay can begin after live guidance already latched degraded MM305 safety
       ownership. For stop-at-TAEM qualification that state is evidence that the
       fixed MM304 ownership boundary was missed, not a successful TAEM capture.
       The in-loop transition path already applies this rule; apply it at t=0 too. */
    out.reached_taem=initial_terminal_owned&&
        !(stop_at_taem&&initial_degraded_safety_handoff);
    out.taem_ownership_boundary_missed=stop_at_taem&&initial_degraded_safety_handoff;
    out.taem_dynamic_interface_captured=shadow.taem_interface_captured;
    out.taem_dynamic_interface_target_valid=shadow.taem_interface_target.valid;
    if(out.entered_atmosphere){
        GeoPoint initial_geo=predictor_geo_point(state.position,p,state.ut);
        GeoPoint initial_target={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        out.entry_range=great_circle_distance(initial_geo,initial_target,p->radius);
        out.entry_flight_path_angle=telemetry->flight_path_angle;
        out.entry_speed=telemetry->true_air_speed;
        out.entry_course_error=telemetry->course_to_site_error;
    }
    out.shadow_guidance_used=true;
    while(elapsed<maxdur){
        AtmosState at;Telemetry synthetic=guidance_shadow_telemetry(telemetry,state,&response,&ctx,p,cfg,&at);
        GeoPoint geo={synthetic.latitude,synthetic.longitude,synthetic.mean_altitude};
        double range=synthetic.range_to_site;
        if(!out.entered_atmosphere&&geo.altitude<=entry_alt&&synthetic.vertical_speed<0.0){
            out.entered_atmosphere=true;out.entry_range=range;out.entry_flight_path_angle=synthetic.flight_path_angle;
            out.entry_speed=at.speed;out.entry_course_error=synthetic.course_to_site_error;
        }
        /* 50 km is the explicit mission validation checkpoint.  Record the same
           runway-relative state from the executable-policy shadow so reduced
           prediction, shadow execution, and live KSP can be compared on one
           physical crossing instead of inferred from different terminal events. */
        if(out.entered_atmosphere&&!out.mm304_gate_recorded&&
           geo.altitude<=50000.0&&synthetic.vertical_speed<0.0){
            out.mm304_gate_recorded=true;out.mm304_gate_range=range;
            out.mm304_gate_along_track=synthetic.runway_along_track;
            out.mm304_gate_cross_track=synthetic.runway_cross_track;
            out.mm304_gate_course=synthetic.ground_track_heading;
            out.mm304_gate_altitude=geo.altitude;out.mm304_gate_speed=at.speed;
            out.mm304_gate_flight_path_angle=synthetic.flight_path_angle;
        }
        if(out.entered_atmosphere)out.closest_distance=fmin(out.closest_distance,range);
        if(out.entered_atmosphere&&!out.reached_taem)out.minimum_entry_speed=fmin(out.minimum_entry_speed,at.speed);
        out.maximum_abs_angle_of_attack=fmax(out.maximum_abs_angle_of_attack,fabs(response.actual_aoa));
        out.peak_dynamic_pressure=fmax(out.peak_dynamic_pressure,at.q);out.peak_g_load=fmax(out.peak_g_load,synthetic.g_force);
        if(at.direct_aero)out.physics_observed_seconds+=fmin(1.0,maxdur-elapsed);else out.physics_fallback_seconds+=fmin(1.0,maxdur-elapsed);
        if(!out.reached_taem){
            out.taem_distance=range;out.taem_range_error=range-entry_taem_range_target(p,&cfg->guidance);
            out.taem_speed=at.speed;out.taem_flight_path_angle=synthetic.flight_path_angle;out.taem_altitude=geo.altitude;
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,cfg->site.latitude,
                cfg->guidance.taem_interface_altitude,entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
        }else if(elapsed<=1e-9){
            out.taem_distance=range;out.taem_range_error=range-entry_taem_range_target(p,&cfg->guidance);
            out.taem_speed=at.speed;out.taem_flight_path_angle=synthetic.flight_path_angle;out.taem_altitude=geo.altitude;
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,cfg->site.latitude,
                cfg->guidance.taem_interface_altitude,entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
        }
        unsigned previous_reversals=shadow.entry_control_reversals;
        GuidanceResult r=guidance_update_with_terminal_model(&shadow,&synthetic,&state,plan,p,aero,cfg,terminal_model);
        if(entry_witness_trace&&!entry_witness_phase_traced&&r.phase==PHASE_ENTRY_ENERGY){
            fprintf(stderr,
                "ENTRY_WITNESS first-entry ut=%.2f h=%.0f V=%.0f margin=%+.0f D=%.3f Dref=%.3f bank=%+.1f cross=%+.0f planning=%d\n",
                state.ut,geo.altitude,at.speed,shadow.entry_feedback.energy_margin,
                shadow.entry_feedback.measured_drag_accel,
                shadow.entry_feedback.reference_drag_accel,
                shadow.entry_feedback.commanded_bank_deg,
                shadow.entry_feedback.crossrange_error,
                shadow.entry_planning_needed?1:0);
            entry_witness_phase_traced=true;
        }
        out.taem_dynamic_interface_target_valid=false;
        if(!out.has_first_s_turn_reversal&&shadow.entry_control_reversals!=previous_reversals){
            out.has_first_s_turn_reversal=true;
            out.first_s_turn_reversal_ut=state.ut;
            out.first_s_turn_reversal_range=range;
            out.first_s_turn_reversal_sign=shadow.s_turn_sign;
            out.first_s_turn_reversal_is_final=false;
            if(entry_witness_trace)
                fprintf(stderr,
                    "ENTRY_WITNESS reversal ut=%.2f V=%.1f h=%.0f cross=%.0f side=%+.0f count=%u\n",
                    state.ut,at.speed,geo.altitude,synthetic.runway_cross_track,
                    shadow.s_turn_sign,shadow.entry_control_reversals);
        }
        out.s_turn_reversals=shadow.entry_control_reversals-guidance->entry_control_reversals;
        bool terminal_owned_now=taem_exec_owns_vehicle(&shadow.taem_exec);
        bool noninterface_terminal_ownership=terminal_owned_now&&!shadow.taem_interface_captured;
        if(stop_at_taem&&noninterface_terminal_ownership)
            out.taem_ownership_boundary_missed=true;
        if(!out.reached_taem&&terminal_owned_now&&!noninterface_terminal_ownership){
            out.reached_taem=true;
            out.taem_distance=range;
            out.taem_dynamic_interface_captured=shadow.taem_interface_captured;
            out.taem_dynamic_interface_target_valid=false;
            out.taem_interface_error=0.0;
            out.taem_interface_energy_margin=shadow.entry_feedback.energy_margin;
            out.taem_range_error=range-entry_taem_range_target(p,&cfg->guidance);
            out.taem_speed=at.speed;
            out.taem_flight_path_angle=synthetic.flight_path_angle;
            out.taem_altitude=geo.altitude;
            out.taem_desired_altitude=cfg->guidance.taem_interface_altitude;
            out.taem_along_track=synthetic.runway_along_track;
            out.taem_cross_track=synthetic.runway_cross_track;
            out.taem_course=synthetic.ground_track_heading;
            out.taem_energy_error=entry_remaining_specific_energy(
                geo.latitude,geo.altitude,at.speed,cfg->site.latitude,
                cfg->guidance.taem_interface_altitude,
                entry_taem_speed_target(&cfg->vehicle,&cfg->guidance,p),p);
        }
        bool policy_feasible=shadow.taem_exec.terminal_evaluation.valid&&shadow.taem_exec.terminal_evaluation.feasible;
        out.shadow_terminal_prediction_valid=shadow.mm305_route_committed&&shadow.mm305_route.valid;
        out.shadow_terminal_policy_feasible=policy_feasible;
        out.shadow_terminal_path_committed=shadow.terminal_path_committed;
        out.shadow_final_approach_captured=shadow.final_approach_captured;
        out.shadow_aborted=shadow.aborted||r.phase==PHASE_ABORT;
        if(out.shadow_aborted&&entry_witness_trace){
            fprintf(stderr,
                "ENTRY_WITNESS shadow-abort ut=%.2f phase=%d h=%.0f V=%.1f mach=%.2f range=%.0f along=%.0f cross=%.0f course=%.1f status=\"%s\" warning=\"%s\" margin=%+.0f entryComplete=%d taemOwn=%d route=%d routeValid=%d evalValid=%d evalFeasible=%d\n",
                state.ut,(int)r.phase,geo.altitude,at.speed,synthetic.mach,range,
                synthetic.runway_along_track,synthetic.runway_cross_track,
                synthetic.ground_track_heading,r.status,r.has_warning?r.warning:"",
                shadow.entry_feedback.energy_margin,shadow.entry_exec.entry_complete?1:0,
                taem_exec_owns_vehicle(&shadow.taem_exec)?1:0,
                shadow.mm305_route_committed?1:0,shadow.mm305_route.valid?1:0,
                shadow.taem_exec.terminal_evaluation.valid?1:0,
                shadow.taem_exec.terminal_evaluation.feasible?1:0);
        }
        bool terminal_public_phase=out.reached_taem&&r.phase==PHASE_TAEM;
        if(include_trajectory&&(state.ut-lastsample>=2.0||terminal_public_phase)){
            TrajectoryPoint tp={state.ut,geo.latitude,geo.longitude,geo.altitude,at.speed,r.phase,TRAJ_PLANNED};
            trajectory_append(&out.trajectory,tp);lastsample=state.ut;
        }
        /* The ownership latch can become true inside the Entry guidance call that
           still returns its Entry-phase result for that tick. Continue one reduced
           integration step so qualification observes the same public PHASE_TAEM
           frame the live controller would publish next, then stop immediately. */
        if(out.shadow_aborted||
           (stop_at_taem&&(out.taem_ownership_boundary_missed||terminal_public_phase))){
            guidance_result_clear(&r);break;
        }
        if(geo.altitude<=cfg->site.altitude-20||at.speed<fmax(25.0,cfg->vehicle.touchdown_speed*.55)){
            guidance_result_clear(&r);break;
        }
        double target_bank=isfinite(r.command.target_roll)?norm_signed_deg(r.command.target_roll):response.actual_bank;
        double target_aoa=response.actual_aoa;
        if(r.command.use_inertial_direction&&vmag(r.command.inertial_direction)>1e-9){
            /* Live q<0.5 Entry Interface capture ignores the dormant Euler/AoA
               fields and uses a quaternion/RCS law to align body-forward with the
               commanded inertial prograde vector while keeping radial-up as the
               roll reference.  The reduced shadow has only bank/AoA states, so
               project that same attitude objective into aerodynamic coordinates:
               wings level, with incidence equal to the vertical-plane angle
               between inertial forward and the atmosphere-relative velocity. */
            Vector3 local_up=vnorm(state.position,v3(1,0,0));
            Vector3 desired_forward=vnorm(r.command.inertial_direction,state.velocity);
            Vector3 air_velocity=vsub(state.velocity,vcross(planet_rotation_vector(p),state.position));
            Vector3 air_forward=vnorm(air_velocity,desired_forward);
            double desired_fpa=atan2(vdot(desired_forward,local_up),
                fmax(vmag(vproject_plane(desired_forward,local_up)),1e-9))*RAD2DEG;
            double air_fpa=atan2(vdot(air_forward,local_up),
                fmax(vmag(vproject_plane(air_forward,local_up)),1e-9))*RAD2DEG;
            target_bank=0.0;
            target_aoa=norm_signed_deg(desired_fpa-air_fpa);
        }else if(r.command.has_target_aoa&&isfinite(r.command.target_aoa))target_aoa=r.command.target_aoa;
        else if(isfinite(r.command.target_pitch))target_aoa=r.command.target_pitch-synthetic.flight_path_angle;
        target_aoa=clampd(target_aoa,-cfg->vehicle.maximum_angle_of_attack,cfg->vehicle.maximum_angle_of_attack);
        ctx.gear=r.command.gear;ctx.brakes=r.command.brakes;ctx.airbrakes=r.command.airbrakes?1:0;
        double dt=taem_exec_owns_vehicle(&shadow.taem_exec)?
            (geo.altitude>25000?.8:(geo.altitude>8000?.45:.25)):
            1.0/clampd(cfg->guidance.guidance_rate,2.0,30.0);
        dt=fmin(dt,maxdur-elapsed);
        predictor_attitude_step(&response,target_bank,target_aoa,&at,&cfg->vehicle,&cfg->guidance,dt,cal0->physics);
        ctx.bank=response.actual_bank;ctx.aoa=response.actual_aoa;ctx.ut=state.ut;ctx.mass=state.mass;
        state=rk4_aero(state,dt,&ctx);elapsed+=dt;
        guidance_result_clear(&r);
    }
    out.final_state=state;
    if(out.closest_distance==DBL_MAX)out.closest_distance=great_circle_distance(predictor_geo_point(state.position,p,state.ut),target,p->radius);
    out.physics_relative_uncertainty=telemetry->physics_certified_uncertainty>0?clampd(telemetry->physics_certified_uncertainty,.06,.90):.45;
    if(outcome){
        outcome->terminal_prediction_valid=out.shadow_terminal_prediction_valid;
        outcome->terminal_policy_feasible=out.shadow_terminal_policy_feasible;
        outcome->terminal_path_committed=out.shadow_terminal_path_committed;
        outcome->final_approach_captured=out.shadow_final_approach_captured;
        outcome->aborted=out.shadow_aborted;
    }
    return out;
}

EntryPrediction predictor_simulate_entry_guidance_shadow(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,
        double maxdur,bool include){
    EntryPrediction empty;memset(&empty,0,sizeof(empty));trajectory_init(&empty.trajectory);
    if(!plan)return empty;
    TerminalShadowOutcome outcome;
    EntryPrediction out=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,plan,p,aero,env,cal,cfg,
        maxdur,include,false,true,NULL,&outcome);
    out.uncertainty_scenarios=out.shadow_guidance_used?1u:0u;
    return out;
}

EntryPrediction predictor_simulate_entry_guidance_shadow_with_terminal_model(
        VehicleState state,const Telemetry*telemetry,const GuidanceMachine*guidance,
        const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,
        const LandingConfiguration*cfg,const TerminalModel*terminal_model,
        double maxdur,bool include){
    EntryPrediction empty;memset(&empty,0,sizeof(empty));trajectory_init(&empty.trajectory);
    if(!plan)return empty;
    TerminalShadowOutcome outcome;
    /* The prediction worker owns planning. This replay certifies exactly the
       selected/provisional executable segment: it neither performs a private
       topology search nor rolls that segment into a different future plan. */
    EntryPrediction out=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,
        plan,p,aero,env,cal,cfg,maxdur,include,false,true,terminal_model,&outcome);
    out.uncertainty_scenarios=out.shadow_guidance_used?1u:0u;
    return out;
}

EntryPrediction predictor_simulate_terminal_shadow(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,
        const TerminalModel*terminal_model,double maxdur,bool include){
    TerminalShadowOutcome outcome;
    EntryPrediction out=predictor_simulate_guidance_shadow_core(state,telemetry,guidance,plan,p,aero,env,cal,cfg,
        maxdur,include,true,false,terminal_model,&outcome);
    out.uncertainty_scenarios=out.shadow_guidance_used?1u:0u;
    out.terminal_survivability_stress_score=out.shadow_guidance_used&&
        terminal_shadow_survives(&out,&outcome,&cfg->vehicle)?1.0:0.0;
    return out;
}

EntryPrediction predictor_simulate_terminal_shadow_ensemble(VehicleState state,const Telemetry*telemetry,
        const GuidanceMachine*guidance,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,
        const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const LandingConfiguration*cfg,
        const TerminalModel*terminal_model,double maxdur,bool include){
    TerminalShadowOutcome nominal_outcome;
    EntryPrediction nominal=predictor_simulate_guidance_shadow_core(
        state,telemetry,guidance,plan,p,aero,env,cal,cfg,maxdur,include,true,false,
        terminal_model,&nominal_outcome);
    if(!nominal.shadow_guidance_used)return nominal;
    AeroContext base_ctx={.p=p,.e=env,.c=cal,.v=&cfg->vehicle,.bank=telemetry->roll,.aoa=telemetry->angle_of_attack,
        .ut=state.ut,.mass=state.mass,.gear=telemetry->gear,.brakes=telemetry->brakes,.airbrakes=telemetry->has_airbrakes&&telemetry->airbrakes};
    AtmosState initial=atmosphere(state.position,state.velocity,&base_ctx);
    double sigma=clampd(initial.physics_uncertainty,.08,.40);nominal.physics_relative_uncertainty=sigma;
    /* Three deterministic sigma cases are a robustness stress score, not a
       calibrated survival probability. Keep the name explicit until held-out
       terminal outcomes support reliability/Brier calibration. */
    double stress_score=terminal_shadow_survives(&nominal,&nominal_outcome,&cfg->vehicle)?.50:0.0;
    unsigned scenarios=1;
    for(int direction=-1;direction<=1;direction+=2){
        TrajectoryCalibrationModel stress=cal?*cal:(TrajectoryCalibrationModel){.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.confidence=.05};
        double signed_sigma=direction*sigma;
        stress.stress_drag_scale=clampd(1.0+signed_sigma,.60,1.40);
        stress.stress_lift_scale=clampd(1.0-signed_sigma,.60,1.40);
        stress.density_scale=clampd(fmax(.1,stress.density_scale)*(1.0+.20*signed_sigma),.70,1.30);
        TerminalShadowOutcome scenario_outcome;
        EntryPrediction scenario=predictor_simulate_guidance_shadow_core(
            state,telemetry,guidance,plan,p,aero,env,&stress,cfg,fmin(maxdur,360.0),
            false,true,false,terminal_model,&scenario_outcome);
        if(terminal_shadow_survives(&scenario,&scenario_outcome,&cfg->vehicle))stress_score+=.25;
        scenarios++;entry_prediction_clear(&scenario);
    }
    nominal.terminal_survivability_stress_score=clampd(stress_score,0,1);nominal.uncertainty_scenarios=scenarios;
    return nominal;
}

bool deorbit_plan_preburn_state_compatible(const DeorbitPlan*plan,double live_mass,double live_thrust,const GuidanceSettings*s){
    if(!plan||!s||!isfinite(live_mass)||live_mass<=0||!isfinite(live_thrust)||live_thrust<0)return false;
    if(plan->planning_mass>1){
        double limit=fmax(0.0,s->deorbit_mass_uncertainty_fraction);
        if(fabs(live_mass-plan->planning_mass)>plan->planning_mass*(limit+1e-9))return false;
    }
    if(plan->planning_available_thrust>1){
        double limit=fmax(0.0,s->deorbit_thrust_uncertainty_fraction);
        if(fabs(live_thrust-plan->planning_available_thrust)>plan->planning_available_thrust*(limit+1e-9))return false;
    }
    return true;
}
static double deorbit_alignment_position_error(const EntryPrediction*p,const GuidanceSettings*s){
    (void)s;
    if(!p||!isfinite(p->taem_interface_error)||p->taem_interface_error<0.0)return INFINITY;
    return p->taem_interface_error;
}

static double deorbit_alignment_course_error(const EntryPrediction*p,const LandingSite*site){
    if(!p||!site||!isfinite(p->taem_course))return INFINITY;
    double runway_offset=fabs(norm_signed_deg(p->taem_course-site->runway_heading));
    double perpendicular_error=fabs(runway_offset-90.0);
    double aligned_error=fmin(runway_offset,fabs(runway_offset-180.0));
    return fmin(perpendicular_error,aligned_error);
}

static double deorbit_alignment_contract_utilization(const EntryPrediction*p,
        const LandingSite*site,const GuidanceSettings*s){
    double position=deorbit_alignment_position_error(p,s);
    double course=deorbit_alignment_course_error(p,site);
    TaemHandoffContract contract=taem_handoff_contract(s);
    if(!isfinite(position)||!isfinite(course)||
       !(contract.horizontal_radius_m>0.0)||
       !(contract.perpendicular_heading_half_width_deg>0.0))
        return INFINITY;
    return fmax(position/contract.horizontal_radius_m,
        course/contract.perpendicular_heading_half_width_deg);
}

bool deorbit_capture_qualified(const EntryPrediction*p,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double peri,bool require_entry_target){
    (void)site;(void)peri;
    if(!p||!v||!s||!p->entered_atmosphere||!p->mm304_gate_recorded||
       !isfinite(p->entry_range)||!isfinite(p->entry_flight_path_angle)||
       !isfinite(p->entry_course_error)||!isfinite(p->peak_dynamic_pressure)||
       !isfinite(p->peak_g_load))
        return false;

    bool safe=p->entry_flight_path_angle>=s->maximum_entry_flight_path_angle&&
        p->peak_dynamic_pressure<=v->maximum_dynamic_pressure&&
        p->peak_g_load<=v->maximum_g_load;
    if(!safe)return false;

    /*
     * target_entry_range/FPA/course are optimization objectives, not a second
     * safety corridor.  When a caller requests the entry target, require only
     * finite target data; candidate ordering below minimizes those errors.
     */
    if(require_entry_target)
        return isfinite(s->target_entry_range)&&
            isfinite(s->target_entry_flight_path_angle);
    return true;
}

bool deorbit_recovery_qualified(const EntryPrediction*p,const LandingSite*site,
        const VehicleProfile*v,const GuidanceSettings*s,double peri){
    return deorbit_capture_qualified(p,site,v,s,peri,false);
}

bool deorbit_live_cutoff_capture_qualified(const EntryPrediction*p,
        const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,
        double peri){
    if(!deorbit_recovery_qualified(p,site,v,s,peri)||!p->reached_taem||
       p->shadow_aborted||p->taem_ownership_boundary_missed||
       !isfinite(p->minimum_entry_speed)||
       p->minimum_entry_speed<v->minimum_safe_speed||
       !isfinite(p->taem_speed)||p->taem_speed<v->minimum_safe_speed)
        return false;

    bool interface_capture=p->taem_dynamic_interface_target_valid?
        p->taem_dynamic_interface_captured:
        (isfinite(p->closest_distance)&&p->closest_distance>=0.0&&
         p->closest_distance<=s->target_deorbit_capture_radius);
    double alignment=deorbit_alignment_contract_utilization(p,site,s);
    return interface_capture&&isfinite(alignment)&&alignment<=1.0;
}

bool deorbit_runway_capture_qualified(const EntryPrediction*p,
        const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s,
        double peri){
    if(!deorbit_recovery_qualified(p,site,v,s,peri)||!p->reached_taem||
       p->shadow_aborted||p->taem_ownership_boundary_missed||
       !isfinite(p->minimum_entry_speed)||
       p->minimum_entry_speed<v->minimum_safe_speed||
       !isfinite(p->taem_speed)||p->taem_speed<v->minimum_safe_speed||
       !isfinite(s->target_deorbit_capture_radius)||
       !(s->target_deorbit_capture_radius>0.0)||
       !isfinite(s->mm304_perpendicular_heading_half_width)||
       !(s->mm304_perpendicular_heading_half_width>0.0))
        return false;

    double position=deorbit_alignment_position_error(p,s);
    double course=deorbit_alignment_course_error(p,site);
    return isfinite(p->closest_distance)&&
        p->closest_distance<=s->target_deorbit_capture_radius&&
        isfinite(p->taem_range_error)&&
        fabs(p->taem_range_error)<=s->target_deorbit_capture_radius&&
        isfinite(position)&&position<=s->target_deorbit_capture_radius&&
        isfinite(course)&&
        course<=s->mm304_perpendicular_heading_half_width;
}

bool reentry_guidance_shadow_recovery_qualified(const EntryPrediction*p,
        const VehicleProfile*v,const GuidanceSettings*s){
    if(!p||!v||!s||!p->shadow_guidance_used||!p->entered_atmosphere||
       !p->reached_taem||p->shadow_aborted||
       p->taem_ownership_boundary_missed)
        return false;
    if(!isfinite(p->closest_distance)||!isfinite(p->taem_distance)||
       !isfinite(p->taem_range_error)||!isfinite(p->taem_speed)||
       !isfinite(p->peak_dynamic_pressure)||!isfinite(p->peak_g_load)||
       !isfinite(p->minimum_entry_speed))
        return false;

    bool capture=p->taem_dynamic_interface_target_valid?
        p->taem_dynamic_interface_captured:
        (p->closest_distance>=0.0&&
         p->closest_distance<=s->target_deorbit_capture_radius);
    bool load=p->peak_dynamic_pressure<=v->maximum_dynamic_pressure&&
        p->peak_g_load<=v->maximum_g_load;
    bool speed=p->minimum_entry_speed>=v->minimum_safe_speed&&
        p->taem_speed>=v->minimum_safe_speed;
    return capture&&load&&speed;
}

typedef struct {
    bool valid;
    double impact_along_track,impact_cross_track,impact_range,impact_altitude,impact_speed;
} DeorbitEnergyMargin;
typedef struct {
    double burn_ut,dv,duration,peri,requested_periapsis_altitude,score;
    VehicleState after;
    EntryPrediction pred;
    DeorbitEnergyMargin energy_margin;
    int tier;
    double safety_violation;
    double energy_deficit;
    double energy_excess;
    double entry_fpa_error;
    double entry_course_error;
    double entry_range_error;
    double handoff_alignment_utilization;
    double handoff_range_error;
    bool executable_witness_evaluated;
    bool executable_witness_present;
    bool executable_witness_capture;
    bool executable_witness_handoff_valid;
    bool executable_witness_boundary_missed;
    bool executable_witness_aborted;
    double executable_witness_taem_error;
    double executable_witness_range_error;
    double executable_witness_energy_margin;
    double executable_witness_alignment_error;
    double executable_witness_closest_distance;
} Candidate;

typedef enum {
    DEORBIT_REJECT_NONE=0,
    DEORBIT_REJECT_START,
    DEORBIT_REJECT_TARGET_INPUT,
    DEORBIT_REJECT_TARGET_ORBIT,
    DEORBIT_REJECT_TARGET_LOW_STATE,
    DEORBIT_REJECT_TARGET_IMPULSE_BOUND,
    DEORBIT_REJECT_TARGET_IMPULSE_ROOT,
    DEORBIT_REJECT_TARGET_FINITE_SEED,
    DEORBIT_REJECT_TARGET_FINITE_BRACKET,
    DEORBIT_REJECT_TARGET_FINITE_ROOT,
    DEORBIT_REJECT_IMPULSIVE_PERIAPSIS,
    DEORBIT_REJECT_FINITE_BURN,
    DEORBIT_REJECT_POSTBURN_PERIAPSIS,
    DEORBIT_REJECT_ENTRY_PREDICTION,
    DEORBIT_REJECT_COUNT
} DeorbitRejectReason;

static int execution_witness_status_compare(const Candidate*a,const Candidate*b){
    if(!a||!b)return 0;
    if(a->executable_witness_evaluated!=b->executable_witness_evaluated)
        return a->executable_witness_evaluated?-1:1;
    if(!a->executable_witness_evaluated)return 0;
    if(a->executable_witness_capture!=b->executable_witness_capture)
        return a->executable_witness_capture?-1:1;
    if(a->executable_witness_handoff_valid!=b->executable_witness_handoff_valid)
        return a->executable_witness_handoff_valid?-1:1;
    if(a->executable_witness_boundary_missed!=b->executable_witness_boundary_missed)
        return a->executable_witness_boundary_missed?1:-1;
    if(a->executable_witness_aborted!=b->executable_witness_aborted)
        return a->executable_witness_aborted?1:-1;
    if(a->executable_witness_present!=b->executable_witness_present)
        return a->executable_witness_present?-1:1;
    return 0;
}

static int execution_witness_geometry_compare(const Candidate*a,const Candidate*b){
    if(!a||!b||!a->executable_witness_evaluated||
       !b->executable_witness_evaluated)return 0;
    if(a->executable_witness_taem_error<b->executable_witness_taem_error)return -1;
    if(a->executable_witness_taem_error>b->executable_witness_taem_error)return 1;
    bool a_energy=isfinite(a->executable_witness_energy_margin);
    bool b_energy=isfinite(b->executable_witness_energy_margin);
    if(a_energy!=b_energy)return a_energy?-1:1;
    if(a_energy){
        double a_residual=fabs(a->executable_witness_energy_margin);
        double b_residual=fabs(b->executable_witness_energy_margin);
        if(a_residual<b_residual)return -1;
        if(a_residual>b_residual)return 1;
    }
    if(a->executable_witness_alignment_error<b->executable_witness_alignment_error)return -1;
    if(a->executable_witness_alignment_error>b->executable_witness_alignment_error)return 1;
    if(a->executable_witness_closest_distance<b->executable_witness_closest_distance)return -1;
    if(a->executable_witness_closest_distance>b->executable_witness_closest_distance)return 1;
    return 0;
}
