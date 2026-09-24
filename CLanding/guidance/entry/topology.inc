static GuidanceResult entry_topology_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const EntryTopologyPlan*top=&g->entry_topology;const TaemInterfaceTarget*q=&g->taem_interface_target;
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    bool turning=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    double time_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(fabs(top->terminal_turn_ut),fabs(top->reversal_ut)));
    bool separate_terminal_turn=isfinite(top->terminal_turn_ut)&&
        isfinite(top->reversal_ut)&&top->terminal_turn_ut>top->reversal_ut+time_resolution;
    bool started=false;
    if(!turning&&g->entry_reversal_scheduled)started=entry_program_execute_planned_reversal(g,t,p,cfg);
    turning=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    bool shaping_reversal_complete=separate_terminal_turn&&!turning&&
        g->s_turn_sign*top->first_sign<0.0&&t->ut+time_resolution>=top->reversal_ut;
    if(started&&!g->diagnostic_shadow)fprintf(stderr,
        "Entry topology REVERSAL: UT %.2f deadline %.2f x %.0f/%.0f y %.0f/%.0f h %.0f/%.0f V %.1f/%.1f course %.1f/%.1f -> %.1f\n",
        t->ut,top->reversal_ut,t->runway_along_track,top->reversal_along,t->runway_cross_track,top->reversal_cross,
        t->mean_altitude,top->reversal_altitude,t->true_air_speed,top->reversal_speed,course,top->reversal_course,
        separate_terminal_turn?cfg->site.runway_heading:q->course);

    /* A complete propagated topology has two distinct events: first reverse the
       energy-management leg and settle near runway-parallel flight, then begin the
       same-side 90 deg terminal turn when the measured turn circle reaches the
       fixed -8 km alignment station.  The old executor collapsed these into one
       final reversal, so every otherwise-useful two-event topology was rejected. */
    double desired_heading=course;
    double bank=top->first_sign*top->first_bank;
    if(shaping_reversal_complete){
        desired_heading=norm_deg(cfg->site.runway_heading);
        bank=entry_taem_tangent_capture_bank(g,t,aero,cfg,desired_heading);
        double dynamic_limit=dynamic_bank_limit(t,v);
        if(isfinite(dynamic_limit))
            bank=clampd(bank,-dynamic_limit,dynamic_limit);

        double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
        double final_side=target_relative>=0.0?1.0:-1.0;
        double endpoint_error=INFINITY,rollthrough_error=INFINITY,rollthrough_lead=NAN;
        bool endpoint_ready=entry_program_terminal_course_endpoint_ready(
            g,t,cfg,final_side,q->course,&endpoint_error,NULL);
        bool rollthrough_ready=entry_program_rollthrough_endpoint_ready(
            g,t,p,cfg,final_side,&rollthrough_error,&rollthrough_lead);
        ControlAuthorityEnvelope authority=
            decision_control_authority_envelope(g,t,p,cfg);

        /* A propagated terminal-turn timestamp no longer authorizes anything.
           Start the final MM304 turn only when the measured state can close the
           fixed point/course endpoint now, or after the physically projected
           roll-through transient, with live control authority still survivable. */
        if(authority.controllable&&(endpoint_ready||rollthrough_ready)){
            g->entry_final_reversal_pending=true;
            g->entry_final_reversal_completed=false;
            turning=true;
            if(!g->diagnostic_shadow)fprintf(stderr,
                "MM304 terminal turn START: UT %.2f endpoint=%d err %.0f m rollthrough=%d err %.0f m lead %.2fs course %.1f -> %.1f.\n",
                t->ut,endpoint_ready?1:0,endpoint_error,rollthrough_ready?1:0,
                rollthrough_error,rollthrough_lead,course,q->course);
        }
    }

    double target_course_error=norm_signed_deg(q->course-course);
    TaemHandoffContract handoff_contract=taem_handoff_contract(&cfg->guidance);
    if(turning&&!g->entry_topology_heading_locked&&
       fabs(target_course_error)<=handoff_contract.perpendicular_heading_half_width_deg){
        g->entry_topology_heading_locked=true;
        if(!g->diagnostic_shadow)fprintf(stderr,
            "MM304 perpendicular HEADING LOCK: UT %.2f course %.1f target %.1f x %.0f y %.0f.\n",
            t->ut,course,q->course,t->runway_along_track,t->runway_cross_track);
    }
    bool point_recovery_required=false;
    if(turning&&!g->entry_topology_heading_locked){
        /* Recompute the circle that satisfies the complete fixed-pose contract at
           every step: current course -> fixed -8 km point -> perpendicular outlet.
           Command the bank needed for that radius at the *current* speed/lift, so
           deceleration opens the bank instead of shrinking the flown radius inside
           the planned arc. */
        double turn_sign=-top->first_sign;
        double pose_error=INFINITY,pose_radius=INFINITY;
        (void)entry_program_terminal_course_endpoint_ready(g,t,cfg,turn_sign,q->course,
            &pose_error,&pose_radius);
        double limit=fmin(top->turn_bank,dynamic_bank_limit(t,v));
        double lift=live_lift_accel(t,aero,v);
        double eff=clampd(t->bank_effectiveness,.35,1.8);
        double horizontal=fmax(80.0,t->horizontal_speed);
        double bank_mag=0.0;
        if(isfinite(pose_radius)&&pose_radius>=1000.0&&isfinite(lift)&&lift>.01){
            double lateral=horizontal*horizontal/pose_radius;
            double max_ratio=sin(clampd(limit*eff,0.0,89.0)*DEG2RAD);
            bank_mag=asin(clampd(lateral/lift,0.0,max_ratio))/eff*RAD2DEG;
        }else{
            double fallback=taem_course_rate_bank_command(horizontal,lift,eff,limit,
                course,q->course,q->response_time);
            bank_mag=fabs(fallback);
        }
        bank=turn_sign*clampd(bank_mag,0.0,limit);
        desired_heading=q->course;
    }else if(turning){
        /* Once the +/-90 deg course is acquired, capture the fixed -8 km point, not
           an infinite tangent line. If the point is crossed diagonally, continue the
           owned turn with full demonstrated bank authority until it is reacquired. */
        desired_heading=entry_taem_tangent_capture_heading(g,t,cfg);
        double recovery_bearing=NAN;
        point_recovery_required=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
        (void)recovery_bearing;
        bank=entry_taem_tangent_capture_bank(g,t,aero,cfg,desired_heading);
        double correction_limit=dynamic_bank_limit(t,v);
        bank=clampd(bank,-correction_limit,correction_limit);
    }
    double terminal_floor=entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v);
    double aoa=fmax(t->true_air_speed>top->aoa_switch_speed?top->first_aoa:top->turn_aoa,terminal_floor);
    /* The topology executor must preserve the same post-reversal energy policy as
       ordinary MM304. Once control authority is established and the shuttle has
       entered the TAEM-energy band, carry only the protected high-L/D incidence
       needed by the terminal turn instead of burning speed at the thermal-entry
       incidence. This is still bounded by q/g/stall protection below. */
    bool preserve_terminal_energy=(shaping_reversal_complete||turning)&&
        t->true_air_speed<s->taem_force_handoff_speed+350.0&&
        t->dynamic_pressure>=700.0&&t->dynamic_pressure<v->maximum_dynamic_pressure*.85&&
        t->g_force<v->maximum_g_load*.85&&
        (!t->stall_fraction_is_measured||t->stall_fraction<.10);
    bool extended_final_turn=turning&&isfinite(top->turn_aoa)&&
        top->turn_aoa>v->maximum_angle_of_attack+1e-6&&
        t->dynamic_pressure<v->maximum_dynamic_pressure*.88&&t->g_force<v->maximum_g_load*.90&&
        (!t->stall_fraction_is_measured||t->stall_fraction<.08);
    if((preserve_terminal_energy||point_recovery_required)&&!extended_final_turn)
        aoa=fmin(aoa,fmax(15.0,terminal_floor));
    bank=clampd(bank,-dynamic_bank_limit(t,v),dynamic_bank_limit(t,v));
    bool safety_alpha=(t->stall_fraction_is_measured&&t->stall_fraction>.12)||t->g_force>v->maximum_g_load*.92;
    if(safety_alpha)aoa=fmin(aoa,fmax(6.0,t->angle_of_attack-2.0));
    double topology_aoa_ceiling=extended_final_turn?entry_final_s_turn_aoa_ceiling(v):v->maximum_angle_of_attack;
    aoa=clampd(aoa,0.0,topology_aoa_ceiling);
    g->entry_control_bank=bank;
    if(turning)(void)entry_program_tangent_reversal_captured(g,t,v);
    else entry_update_s_turn_leg_capture(g,t,bank,v);

    TaemInterfaceCapture capture=entry_dynamic_interface_capture(g,t,course,p,aero,cfg);
    bool mm305_ready=q->valid&&mm305_acquisition_ready(t,p,cfg);
    bool handoff_locked=mm305_ready&&fabs(norm_signed_deg(t->roll))<=18.0;
    g->entry_topology_capture_good_duration=handoff_locked?
        g->entry_topology_capture_good_duration+fmax(0.0,dt):0.0;
    EntryDragReferenceConfig drag_cfg=entry_drag_reference_default_config();
    drag_cfg.entry_velocity_ratio=4.5;
    double interface_speed=entry_taem_speed_target(v,s,p);
    EntryDragReferenceInput drag_in={.phase=g->entry_exec.initialized?g->entry_exec.phase:ENTRY_PHASE_PREENTRY,
        .relative_velocity=t->true_air_speed,.latitude=t->latitude,.altitude=t->mean_altitude,
        .measured_drag_accel=live_drag_accel(t,aero,v),.modeled_drag_accel=live_drag_accel(t,aero,v),
        .aero_confidence=clampd(aero.confidence,0.0,1.0),.has_incidence=true,
        .angle_of_attack=t->angle_of_attack,.sideslip=t->sideslip,
        .range_to_site=fmax(0.0,t->range_to_site),
        .taem_range=entry_taem_range_target(p,s),.taem_latitude=cfg->site.latitude,
        .taem_altitude=s->taem_interface_altitude,.taem_velocity=interface_speed,
        .planet=p,.vehicle=v};
    EntryDragReferenceOutput drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
    /* A propagated topology owns reversal geometry, but live longitudinal energy
       still owns how much drag the final S-turn must make. finalaoa4 selected a
       nominal 16 deg turn incidence while the measured Entry-to-TAEM excess was
       still about +200 km, so the topology path bypassed the explicitly allowed
       ~35 deg drag correction used by ordinary MM304. Apply the same live q/g/
       stall/path-guarded correction here after the topology drag reference exists. */
    double live_final_s_turn_aoa=entry_program_final_s_turn_drag_aoa(g,t,p,aero,cfg,&drag,
        turning||shaping_reversal_complete);
    if(turning&&isfinite(live_final_s_turn_aoa)){
        aoa=fmax(aoa,live_final_s_turn_aoa);
        aoa=clampd(aoa,0.0,entry_final_s_turn_aoa_ceiling(v));
    }
    EntryExecProfile profile=entry_drag_reference_exec_profile(&drag_in,&drag_cfg,&drag);
    bool handoff_qualified=mm305_ready&&g->entry_topology_capture_good_duration>=.75;
    EntryExecObservation obs={.ut=t->ut,.relative_velocity=t->true_air_speed,.altitude=t->mean_altitude,
        .dynamic_pressure=fmax(0.0,t->dynamic_pressure),.checkpoint_restart=false};
    bool executive_ok=entry_exec_update(&g->entry_exec,&obs,&profile);
    if(executive_ok&&handoff_qualified&&g->entry_exec.phase==ENTRY_PHASE_TRANSITION)
        executive_ok=entry_exec_accept_taem_handoff(&g->entry_exec,&obs);
    if(executive_ok&&g->entry_exec.entry_complete)g->taem_interface_captured=true;
    if(!g->diagnostic_shadow&&(t->ut-g->taem_interface_diagnostic_ut>=5.0||g->taem_interface_captured)){
        fprintf(stderr,"TAEM topology inlet: UT %.2f x %.0f y %.0f h %.0f V %.1f course %.1f -> %.1f bank %.1f roll %.1f veto %u along %.0f cross %.0f dh %.0f energy %.0f turn %.0f locked %.2fs ready %d\n",
            t->ut,t->runway_along_track,t->runway_cross_track,t->mean_altitude,t->true_air_speed,course,q->course,
            bank,norm_signed_deg(t->roll),capture.veto,capture.along,capture.cross,capture.altitude_error,
            capture.energy_margin,capture.turn_margin,g->entry_topology_capture_good_duration,g->taem_interface_captured);
        g->taem_interface_diagnostic_ut=t->ut;
    }
    const char*warning=NULL;
    if(!turning&&t->ut>top->reversal_ut+10.0){
        warning="Complete-path reversal setup was not reached inside its physical state tube; refusing a late near-point reversal.";
        g->entry_committed_infeasible=true;
        if(!g->diagnostic_shadow)fprintf(stderr,"Entry topology FAILED setup: UT %.2f deadline %.2f x %.0f y %.0f h %.0f V %.1f course %.1f\n",
            t->ut,top->reversal_ut,t->runway_along_track,t->runway_cross_track,t->mean_altitude,t->true_air_speed,course);
        guidance_abort(g);
    }else if(turning&&!g->taem_interface_captured){
        /* A forecast clock never releases MM304 ownership of the fixed alignment
           point. If capture takes longer than predicted, keep the live point/heading
           controller active (and flag the stale forecast) instead of aborting to
           straight flight. Only a real low-speed boundary makes this trajectory dead. */
        if(t->true_air_speed<v->minimum_safe_speed*1.6){
            warning="MM304 fixed-point capture became physically unrecoverable at low speed; handoff remains inhibited.";
            g->entry_committed_infeasible=true;
            if(!g->diagnostic_shadow)fprintf(stderr,"Entry topology FAILED inlet energy: UT %.2f deadline %.2f x %.0f y %.0f h %.0f V %.1f course %.1f veto %u [speed=%d spatial=%d course=%d altitude=%d maneuver=%d fpa=%d structural=%d HACradius=%d minAlt=%d].\n",
                t->ut,top->capture_ut,t->runway_along_track,t->runway_cross_track,t->mean_altitude,t->true_air_speed,course,capture.veto,
                (capture.veto&1u)!=0,(capture.veto&2u)!=0,(capture.veto&4u)!=0,(capture.veto&8u)!=0,
                (capture.veto&16u)!=0,(capture.veto&32u)!=0,(capture.veto&64u)!=0,
                (capture.veto&128u)!=0,(capture.veto&256u)!=0);
            guidance_abort(g);
        }else if(t->ut>top->capture_ut+30.0){
            warning=point_recovery_required?
                "MM304 capture forecast expired; continuing fixed-point recovery until the 1 km handoff contract is met.":
                "MM304 capture forecast expired; continuing fixed-point capture and requesting updated supervision.";
            g->entry_planning_needed=true;
        }
    }else if(!executive_ok)warning="Entry executive rejected the coupled path reference; handoff remains inhibited.";
    else if(safety_alpha)warning="Stall/load protection is overriding the propagated incidence; the trajectory requires live revalidation.";
    else if(g->entry_supervision_boundary_missed)warning="Latest advisory shadow did not certify this inlet; live capture remains mandatory.";
    g->entry_s_turn_plan.valid=true;
    g->entry_s_turn_plan.has_planned_reversal=!turning;
    g->entry_s_turn_plan.final_heading_lock=turning;
    g->entry_control_plan_valid=true;
    g->entry_control_terminal_ready=g->entry_s_turn_plan.terminal_ready;
    g->entry_control_plan_ut=t->ut;g->entry_control_aoa=aoa;g->entry_control_heading=desired_heading;
    g->entry_control_turn_radius=live_turn_radius(t,aero,v,bank);g->entry_control_cost=top->cost;
    g->entry_control_segment_until_ut=top->capture_ut+30.0;
    g->airbrakes_deployed=false;
    if(!turning&&t->ut-g->entry_supervision_ut>=15.0)g->entry_planning_needed=true;
    GuidanceCommand command=atmospheric(t,desired_heading,bank,v,0.0,false,PROFILE_ENTRY);
    command.heading_control_enabled=false;command.has_target_aoa=true;
    command.target_aoa=aoa;command.target_pitch=t->flight_path_angle+aoa;
    char status[384];
    if(!turning)snprintf(status,sizeof(status),"MM304 reachable S-turn: side %+.0f bank %+.1f alpha %.1f, plan %.1fs, reversal in %.1fs; inlet %.1f/%.1f km, course %.0f.",
        top->first_sign,bank,aoa,fmax(0.0,g->entry_control_segment_until_ut-t->ut),top->reversal_ut-t->ut,
        q->along_track/1000.0,q->cross_track/1000.0,q->course);
    else snprintf(status,sizeof(status),"MM304 %s: course %.1f -> %.1f ref %.1f, bank %+.1f roll %+.1f, gate %.1f km, h %.1f/%.1f V %.0f/%.0f, inlet veto %u.",
        g->entry_topology_heading_locked?"heading lock":"perpendicular turn",course,q->course,desired_heading,
        bank,norm_signed_deg(t->roll),drag_in.range_to_site/1000.0,t->mean_altitude/1000.0,q->altitude/1000.0,
        t->true_air_speed,q->speed,capture.veto);
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,command,status,warning),t,v,s,dt);
}

static double entry_alpha_transition_velocity(const GuidanceSettings*s,double strict_taem_velocity){
    /* The strict TAEM/HAC inlet speed is a capture contract, not an Entry AoA
       scheduling breakpoint. Reusing the <=500 m/s capture target here compressed
       the whole five-point alpha schedule below 1 km/s and forced max-alpha Entry
       while the orbiter was still above 2 km/s. Preserve the strict target for
       drag/capture accounting, but retain the established MM304 shaping speed as
       the aerodynamic schedule floor. */
    if(!s)return strict_taem_velocity;
    return fmax(strict_taem_velocity,s->taem_force_handoff_speed);
}

