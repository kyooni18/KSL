static GuidanceResult entry_program_contract_hold(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double dt,const char*warning){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    double bank_limit=dynamic_bank_limit(t,v);
    bool have_previous=g->entry_control_plan_valid&&
        isfinite(g->entry_control_bank)&&isfinite(g->entry_control_aoa)&&
        isfinite(g->entry_control_heading);

    double bank=have_previous?g->entry_control_bank:norm_signed_deg(t->roll);
    double aoa=have_previous?g->entry_control_aoa:t->angle_of_attack;
    double heading=have_previous?g->entry_control_heading:
        (isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading);
    if(!isfinite(bank))bank=0.0;
    if(!isfinite(aoa))aoa=v->entry_angle_of_attack;
    if(!isfinite(heading))heading=cfg->site.runway_heading;
    bank=clampd(bank,-bank_limit,bank_limit);
    aoa=clampd(aoa,0.0,v->maximum_angle_of_attack);

    g->entry_control_terminal_ready=false;
    g->entry_planning_needed=true;
    g->airbrakes_deployed=false;

    GuidanceCommand c=atmospheric(t,heading,bank,v,0.0,false,PROFILE_ENTRY);
    c.heading_control_enabled=false;
    c.has_target_aoa=true;
    c.target_aoa=aoa;
    c.target_pitch=t->flight_path_angle+aoa;
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,
        "MM304 feedback model unavailable; retaining the bounded current Entry command.",
        warning),t,v,s,dt);
}

GuidanceResult entry_program_guidance(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    (void)state;

    /* Retire all event/waypoint ownership from live MM304.  These fields may
       survive in old checkpoints or forecast records, but they cannot command
       the vehicle or qualify the handoff. */
    g->entry_reversal_scheduled=false;
    g->entry_reversal_is_final=false;
    g->entry_reversal_ut=NAN;
    g->entry_reversal_range=NAN;
    g->entry_reversal_sign=0.0;
    g->entry_reversal_bank=0.0;
    g->entry_final_reversal_pending=false;
    g->entry_final_reversal_completed=false;
    g->entry_lateral_infeasible=false;
    g->entry_geometry_bank=0.0;
    g->entry_lateral_required_bank=0.0;
    g->taem_interface_target=(TaemInterfaceTarget){0};

    double taem_velocity=entry_taem_speed_target(v,s,p);
    if(!isfinite(taem_velocity)||taem_velocity<=0.0)
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 has no finite TAEM velocity target from the active vehicle/planet model.");

    EntryPhase phase=g->entry_exec.initialized?g->entry_exec.phase:ENTRY_PHASE_PREENTRY;
    EntryDragReferenceConfig drag_cfg=entry_drag_reference_default_config();
    double measured_drag=t->mass>DBL_MIN&&isfinite(t->drag_force)&&t->drag_force>=0.0?
        t->drag_force/t->mass:NAN;
    EntryDragReferenceInput drag_in={
        .phase=phase,
        .relative_velocity=t->true_air_speed,
        .latitude=t->latitude,
        .altitude=t->mean_altitude,
        .measured_drag_accel=measured_drag,
        .modeled_drag_accel=live_drag_accel(t,aero,v),
        .aero_confidence=clampd(fmax(aero.confidence,t->aerodynamic_confidence),0.0,1.0),
        .has_incidence=isfinite(t->angle_of_attack)&&isfinite(t->sideslip),
        .angle_of_attack=t->angle_of_attack,
        .sideslip=t->sideslip,
        .range_to_site=fmax(0.0,t->range_to_site),
        .taem_range=entry_taem_range_target(p,s),
        .taem_latitude=cfg->site.latitude,
        .taem_altitude=s->taem_interface_altitude,
        .taem_velocity=taem_velocity,
        .planet=p,
        .vehicle=v
    };
    EntryDragReferenceOutput drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
    if(!drag.valid)
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 drag-reference state is invalid; Entry ownership is retained.");

    EntryExecProfile exec_profile=entry_drag_reference_exec_profile(&drag_in,&drag_cfg,&drag);
    bool checkpoint_restart=!g->entry_exec.initialized&&g->entry_continuation_bootstrap;
    EntryExecObservation obs={
        .ut=t->ut,
        .relative_velocity=t->true_air_speed,
        .altitude=t->mean_altitude,
        .dynamic_pressure=fmax(0.0,t->dynamic_pressure),
        .checkpoint_restart=checkpoint_restart
    };
    if(!entry_exec_update(&g->entry_exec,&obs,&exec_profile))
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 executive rejected the measured drag-reference state.");

    if(g->entry_exec.phase!=drag_in.phase){
        drag_in.phase=g->entry_exec.phase;
        drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
        if(!drag.valid)
            return entry_program_contract_hold(g,t,cfg,dt,
                "MM304 phase transition invalidated the drag-reference state.");
    }

    TaemInterfaceCapture admission=entry_mm305_admission_envelope(g,t,course,p,cfg);
    bool handoff_ready=admission.valid&&admission.ready;
    if(handoff_ready&&g->entry_exec.phase==ENTRY_PHASE_TRANSITION&&
       !entry_exec_accept_taem_handoff(&g->entry_exec,&obs))
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 executive rejected an otherwise admissible MM305 handoff state.");

    /* Energy/range control: drag reference re-solved from the range still
       required, tracked with a damped law against *measured* drag and lift. */
    double transition_velocity=entry_alpha_transition_velocity(s,taem_velocity);
    double energy_bank_limit=dynamic_bank_limit(t,v);
    EntryEnergyConfig energy_cfg=entry_energy_default_config();
    EntryEnergyInput energy_in={
        .ut=t->ut,
        .relative_velocity=t->true_air_speed,
        .altitude=t->mean_altitude,
        .latitude=t->latitude,
        .flight_path_angle_deg=t->flight_path_angle,
        .vertical_speed=t->vertical_speed,
        .measured_drag_accel=isfinite(measured_drag)?measured_drag:
            live_drag_accel(t,aero,v),
        .measured_lift_accel=live_lift_accel(t,aero,v),
        .dynamic_pressure=fmax(0.0,t->dynamic_pressure),
        .range_to_go=fmax(0.0,t->range_to_site)-entry_taem_range_target(p,s),
        .taem_velocity=taem_velocity,
        .taem_altitude=s->taem_interface_altitude,
        .taem_latitude=cfg->site.latitude,
        .transition_velocity=fmax(transition_velocity,taem_velocity*1.05),
        .maximum_bank_deg=energy_bank_limit,
        .integrator_hold=g->entry_lateral.initialized&&!g->entry_lateral.leg_captured,
        .planet=p,
        .vehicle=v
    };
    EntryEnergyOutput energy=entry_energy_update(&g->entry_energy,&energy_in,&energy_cfg);
    if(!energy.valid)
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 energy controller state is invalid; Entry ownership is retained.");

    EntryAlphaSchedule alpha_schedule;
    entry_alpha_schedule_default(&alpha_schedule,v,transition_velocity);
    EntryAlphaInput alpha_in={
        .relative_velocity=t->true_air_speed,
        .phase=entry_alpha_phase_for_exec(g->entry_exec.phase),
        .reference_drag_accel=energy.reference_drag_accel,
        .measured_drag_accel=energy_in.measured_drag_accel,
        .current_aoa=t->angle_of_attack,
        .current_aoa_rate=controlled_aoa_rate(t),
        .has_previous_target=g->entry_alpha_has_target,
        .previous_target_aoa=g->entry_alpha_target,
        .has_previous_modulation=g->entry_alpha_has_modulation,
        .previous_modulation=g->entry_alpha_modulation,
        .dt=fmax(DBL_EPSILON,dt),
        .maximum_aoa=v->maximum_angle_of_attack,
        .stall_margin_aoa_limit=NAN,
        .stall_fraction=t->stall_fraction,
        .stall_fraction_is_measured=t->stall_fraction_is_measured,
        .calibrated_stall_speed=t->calibrated_stall_speed,
        .minimum_safe_speed=v->minimum_safe_speed,
        .dynamic_pressure=t->dynamic_pressure,
        .maximum_dynamic_pressure=v->maximum_dynamic_pressure,
        .g_load=t->g_force,
        .maximum_g_load=v->maximum_g_load,
        .aero_confidence=clampd(fmax(drag.confidence,t->aerodynamic_confidence),0.0,1.0)
    };
    EntryAlphaResult alpha=entry_alpha_command(&alpha_schedule,&alpha_in);
    if(!alpha.valid)
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 AoA schedule is invalid; Entry ownership is retained.");
    g->entry_alpha_has_target=true;
    g->entry_alpha_target=alpha.target_aoa;
    g->entry_alpha_has_modulation=true;
    g->entry_alpha_modulation=alpha.modulation;

    EntryLateralLimits limits=entry_lateral_default_limits();
    limits.maximum_bank_deg=dynamic_bank_limit(t,v);
    limits.maximum_roll_rate_deg_s=s->entry_roll_rate;
    limits.maximum_roll_accel_deg_s2=s->entry_roll_acceleration;

    EntryLateralInput lateral={0};
    lateral.ut=t->ut;
    lateral.dt=fmax(DBL_EPSILON,dt);
    lateral.relative_speed=t->true_air_speed;
    lateral.measured_bank_deg=norm_signed_deg(t->roll);
    lateral.measured_bank_rate_deg_s=controlled_roll_rate(t);
    lateral.bank_effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    lateral.lift_accel=live_lift_accel(t,aero,v);
    lateral.authority_confidence=clampd(
        fmax(t->aerodynamic_confidence,t->physics_authority_confidence[1]),0.0,1.0);
    lateral.course_to_site_error_deg=isfinite(t->course_to_site_error)?
        t->course_to_site_error:0.0;
    lateral.has_crossrange_error=isfinite(t->runway_cross_track);
    lateral.crossrange_error_m=lateral.has_crossrange_error?-t->runway_cross_track:0.0;

    double map_radius=p->radius+fmax(0.0,t->mean_altitude);
    double map_scale=map_radius>DBL_EPSILON?p->radius/map_radius:1.0;
    double map_speed=fmax(0.0,t->horizontal_speed)*map_scale;
    double course_relative=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    if(lateral.has_crossrange_error&&isfinite(map_speed)&&isfinite(course_relative)){
        lateral.has_crossrange_rate=true;
        lateral.crossrange_error_rate_mps=-map_speed*sin(course_relative);
        double effective_bank=clampd(lateral.measured_bank_deg*lateral.bank_effectiveness,
            -nextafter(90.0,0.0),nextafter(90.0,0.0));
        double course_rate_rad=map_speed>DBL_EPSILON?
            lateral.lift_accel*sin(effective_bank*DEG2RAD)/map_speed:0.0;
        lateral.has_crossrange_accel=true;
        lateral.crossrange_error_accel_mps2=-map_speed*cos(course_relative)*course_rate_rad;
    }
    lateral.crossrange_uncertainty_m=fmax(
        isfinite(t->trajectory_range_residual)?fabs(t->trajectory_range_residual):0.0,
        isfinite(t->predicted_raw_published_position_delta_30s)?
            fabs(t->predicted_raw_published_position_delta_30s):0.0);
    lateral.longitudinal=(EntryLateralLongitudinalDemand){
        .valid=drag.valid,
        .confidence=drag.confidence,
        .reference_drag_accel=drag.reference_drag_accel,
        .drag_error_accel=drag.drag_error_accel,
        .predicted_range_error=drag.predicted_range_error,
        .range_error_scale=drag.range_error_scale,
        .required_vertical_lift_accel=drag.required_vertical_lift_accel
    };

    lateral.has_bank_magnitude=true;
    lateral.bank_magnitude_deg=energy.bank_magnitude_deg;
    lateral.has_azimuth_error=isfinite(t->course_to_site_error);
    lateral.azimuth_error_deg=lateral.has_azimuth_error?t->course_to_site_error:0.0;
    {
        /* Velocity-dependent heading corridor: wide while energy is high,
           narrowing toward the TAEM interface so MM305 inherits a closing
           course (Shuttle reversal deadband idea, Kerbin-scaled). */
        double span=fmax(1.0,transition_velocity*1.6-taem_velocity);
        double x=clampd((t->true_air_speed-taem_velocity)/span,0.0,1.0);
        lateral.azimuth_deadband_deg=4.0+6.0*x;
        double off_nose=fabs(lateral.azimuth_error_deg);
        lateral.minimum_turn_bank_deg=20.0+25.0*clampd((off_nose-45.0)/45.0,0.0,1.0);
    }

    double previous_side=g->s_turn_sign>=0.0?1.0:-1.0;
    EntryLateralOutput lateral_out=entry_lateral_update(&g->entry_lateral,&lateral,&limits);
    if(!lateral_out.valid)
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 lateral allocator is invalid; Entry ownership is retained.");
    double side=lateral_out.bank_sign>=0.0?1.0:-1.0;
    if(lateral_out.reversal_requested&&side*previous_side<0.0){
        g->entry_control_reversals++;
        g->entry_continuation_bootstrap=false;
        if(!g->diagnostic_shadow)
            fprintf(stderr,
                "MM304 feedback reversal: UT %.2f side %+.0f -> %+.0f azimuth error %+.1f deg deadband %.1f deg V %.0f m/s.\n",
                t->ut,previous_side,side,lateral.azimuth_error_deg,
                lateral.azimuth_deadband_deg,t->true_air_speed);
    }
    g->s_turn_sign=side;

    double bank=clampd(lateral_out.target_bank_deg,-limits.maximum_bank_deg,limits.maximum_bank_deg);
    /* When bank alone cannot produce the demanded drag (bank at its limit),
       incidence is the remaining drag lever: ramp toward maximum AoA while
       q/load protection is clear, and ramp back as soon as bank has authority. */
    {
        double headroom=fmax(0.0,v->maximum_angle_of_attack-alpha.target_aoa);
        bool need_drag=energy.saturated_bank_limit&&!energy.protection_active&&
            energy.reference_drag_accel>energy_in.measured_drag_accel;
        double rate=1.0*fmax(0.0,dt);
        g->entry_alpha_drag_boost=need_drag?
            fmin(headroom,g->entry_alpha_drag_boost+rate):
            fmax(0.0,g->entry_alpha_drag_boost-rate);
        g->entry_alpha_drag_boost=fmin(g->entry_alpha_drag_boost,headroom);
    }
    double aoa=clampd(alpha.target_aoa+g->entry_alpha_drag_boost,0.0,v->maximum_angle_of_attack);
    double heading=isfinite(course)?norm_deg(course):norm_deg(t->heading);

    g->entry_feedback=(EntryGuidanceFeedback){
        .valid=true,
        .lateral_valid=true,
        .reversal_requested=lateral_out.reversal_requested,
        .ut=t->ut,
        .confidence=fmin(drag.confidence,lateral.authority_confidence),
        .measured_drag_accel=energy_in.measured_drag_accel,
        .reference_drag_accel=energy.reference_drag_accel,
        .drag_error_accel=energy.reference_drag_accel-energy_in.measured_drag_accel,
        .predicted_achievable_range=energy.predicted_range_m,
        .required_range_to_go=energy.required_range_m,
        .energy_margin=energy.range_error_m,
        .range_error_scale=fmax(1000.0,fabs(energy.required_range_m)),
        .required_vertical_lift_accel=energy.vertical_lift_command,
        .crossrange_error=lateral.has_crossrange_error?lateral.crossrange_error_m:NAN,
        .crossrange_rate=lateral.has_crossrange_rate?lateral.crossrange_error_rate_mps:NAN,
        .projected_crossrange_error=lateral_out.projected_crossrange_error_m,
        .crossrange_corridor=lateral_out.crossrange_corridor_m,
        .energy_bank_magnitude_deg=lateral_out.bank_magnitude_deg,
        .commanded_bank_deg=bank,
        .bank_sign=side
    };

    g->entry_lateral_bank_magnitude=lateral_out.bank_magnitude_deg;
    g->entry_vertical_bank_magnitude=lateral_out.bank_magnitude_deg;
    g->entry_demand_bank=side*lateral_out.bank_magnitude_deg;
    g->entry_control_plan_valid=true;
    g->entry_control_terminal_ready=handoff_ready;
    g->entry_control_plan_ut=t->ut;
    g->entry_control_bank=bank;
    g->entry_control_aoa=aoa;
    g->entry_control_heading=heading;
    g->entry_control_turn_radius=live_turn_radius(t,aero,v,bank);
    g->entry_control_cost=NAN;
    g->entry_control_taem_range_error=energy.range_error_m;
    g->entry_control_taem_speed=taem_velocity;
    g->entry_control_taem_energy_error=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,s->taem_interface_altitude,taem_velocity,p);
    g->entry_control_segment_until_ut=t->ut;
    if(!isfinite(g->entry_supervision_ut)||
       t->ut-g->entry_supervision_ut>=fmax(DBL_EPSILON,s->prediction_interval))
        g->entry_planning_needed=true;

    g->airbrakes_deployed=false;
    GuidanceCommand c=atmospheric(t,heading,bank,v,0.0,false,PROFILE_ENTRY);
    c.heading_control_enabled=false;
    c.has_target_aoa=true;
    c.target_aoa=aoa;
    c.target_pitch=t->flight_path_angle+aoa;

    char status[448];
    snprintf(status,sizeof(status),
        "MM304 %s closed loop: D %.2f/ref %.2f (err %+.2f) m/s2, range %.1f/%.1f km (margin %+.1f km), bank %+.1f deg, cross %+.1f km, AoA %.1f deg, MM305 %s.",
        entry_phase_string(g->entry_exec.phase),energy_in.measured_drag_accel,
        energy.reference_drag_accel,energy.reference_drag_accel-energy_in.measured_drag_accel,
        energy.predicted_range_m/1000.0,energy.required_range_m/1000.0,
        energy.range_error_m/1000.0,bank,
        lateral.has_crossrange_error?lateral.crossrange_error_m/1000.0:NAN,
        aoa,handoff_ready?"admissible":"not yet admissible");

    const char*warning=NULL;
    if(energy.degraded||alpha.degraded||lateral_out.degraded_authority)
        warning="MM304 is using degraded aerodynamic feedback; commands remain inside measured/configured authority and no fallback trajectory is active.";

    static _Thread_local double entry_trace_last_ut=-INFINITY;
    const char*entry_trace=getenv("KSP_LANDER_ENTRY_TRACE");
    if(entry_trace&&strcmp(entry_trace,"1")==0&&
       (t->ut<entry_trace_last_ut||t->ut-entry_trace_last_ut>=5.0)){
        fprintf(stderr,
            "ENTRY_TRACE ut=%.3f h=%.1f V=%.2f q=%.2f fpa=%.2f D=%.3f Dref=%.3f dD=%+.3f Rach=%.1f Rreq=%.1f margin=%+.1f along=%+.1f xtk=%+.1f azerr=%+.1f hdotref=%+.1f Benergy=%.2f Bcmd=%+.2f roll=%+.2f aoa=%.2f admissionE=%+.0f Eavail=%.0f Edrag=%.0f Eunc=%.0f path=%.0f reversal=%u handoff=%d veto=%u eh=%+.0f D0=%.2f sat=%d%d%d\n",
            t->ut,t->mean_altitude,t->true_air_speed,t->dynamic_pressure,
            t->flight_path_angle,energy_in.measured_drag_accel,energy.reference_drag_accel,
            energy.reference_drag_accel-energy_in.measured_drag_accel,energy.predicted_range_m,
            energy.required_range_m,energy.range_error_m,t->runway_along_track,t->runway_cross_track,
            lateral.azimuth_error_deg,energy.hdot_reference,
            lateral_out.bank_magnitude_deg,bank,norm_signed_deg(t->roll),aoa,
            admission.energy_margin,admission.energy_available,admission.energy_drag_work,
            admission.energy_uncertainty,admission.path_length_m,g->entry_control_reversals,
            handoff_ready?1:0,admission.veto,energy.altitude_error_m,energy.profile_drag_accel,
            energy.saturated_lift_up?1:0,energy.saturated_bank_limit?1:0,energy.protection_active?1:0);
        entry_trace_last_ut=t->ut;
    }

    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,status,warning),t,v,s,dt);
}

double live_turn_radius(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double bank_deg){
    double lift=live_lift_accel(t,aero,v);
    double eff=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    double lateral=lift*fabs(sin(clampd(fabs(bank_deg)*eff,0.0,nextafter(90.0,0.0))*DEG2RAD));
    return lateral>DBL_EPSILON?t->true_air_speed*t->true_air_speed/lateral:INFINITY;
}
