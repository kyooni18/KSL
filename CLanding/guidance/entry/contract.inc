static GuidanceResult entry_program_contract_hold(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double dt,const char*warning){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    double bank_limit=dynamic_bank_limit(t,v);
    bool have_previous=g->entry_control_plan_valid&&
        isfinite(g->entry_control_bank)&&isfinite(g->entry_control_aoa)&&
        isfinite(g->entry_control_heading);

    /*
     * Invalid model data is not permission to switch controllers. Preserve the
     * last executable MM304 command, bounded by the live vehicle envelope. If
     * no executable command exists yet, preserve the measured aerodynamic
     * attitude instead. This is a zero-order continuity action, not a recovery
     * trajectory and it cannot satisfy or advance the handoff contract.
     */
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
        "MM304 model contract unavailable; retaining Entry ownership and the bounded current command.",
        warning),t,v,s,dt);
}

       GuidanceResult entry_program_guidance(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    g->entry_lateral_bank_magnitude=NAN;
    g->entry_geometry_bank=NAN;
    g->entry_vertical_bank_magnitude=NAN;
    g->entry_demand_bank=NAN;
    g->entry_lateral_required_bank=NAN;
    bool seed_target_side=entry_program_first_segment(g)&&
        !g->entry_target_side_latched;
    const char *side_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    bool side_diag_on=side_diag&&strcmp(side_diag,"1")==0;
    double seed_side_before=g->s_turn_sign;
    entry_publish_taem_tangent_target(g,t,course,p,aero,cfg);
    if(seed_target_side&&side_diag_on&&!g->diagnostic_shadow)
        fprintf(stderr,
            "MM304 first-side before seed: UT %.2f gside %+.0f latched %d target %.1f current %.1f along %.1f cross %.1f\n",
            t->ut,seed_side_before,g->entry_target_side_latched,
            g->taem_interface_target.course,course,t->runway_along_track,
            t->runway_cross_track);
    /* The first publish may be the operation that creates the fixed inlet.  Seed
       its mission side from that geometry before the candidate reaches the
       planner, then rebuild the advisory record so its course cannot flip on
       the next telemetry sample. */
    if(seed_target_side){
        entry_program_seed_initial_side(g,t,cfg);
        if(g->entry_target_side_latched){
            g->taem_interface_target.valid=false;
            entry_publish_taem_tangent_target(g,t,course,p,aero,cfg);
        }
        if(side_diag_on&&!g->diagnostic_shadow)
            fprintf(stderr,
                "MM304 first-side after seed: UT %.2f side %+.0f target %.1f current %.1f\n",
                t->ut,g->s_turn_sign,g->taem_interface_target.course,course);
    }
    if(g->entry_topology.valid)return entry_topology_guidance(g,t,course,p,aero,cfg,dt);
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double taem_velocity=q->valid&&isfinite(q->speed)&&q->speed>0.0?
        q->speed:s->taem_force_handoff_speed;
    if(!g->entry_drag_ratio_valid){
        g->entry_drag_velocity_ratio=clampd(t->true_air_speed/fmax(taem_velocity,1.0)*1.05,1.20,4.50);
        g->entry_drag_ratio_valid=true;
    }

    double drag_range=fmax(0.0,t->range_to_site);
    double drag_taem_range=entry_taem_range_target(p,s);
    double drag_taem_latitude=cfg->site.latitude;
    double drag_taem_altitude=s->taem_interface_altitude;
    if(q->valid&&isfinite(t->runway_along_track)&&isfinite(t->runway_cross_track)){
        double da=q->along_track-t->runway_along_track;
        double dc=q->cross_track-t->runway_cross_track;
        drag_range=hypot(da,dc);
        drag_taem_range=0.0;
        drag_taem_altitude=q->altitude;
        double rh=cfg->site.runway_heading*DEG2RAD;
        double e=q->along_track*sin(rh)+q->cross_track*cos(rh);
        double n=q->along_track*cos(rh)-q->cross_track*sin(rh);
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint gate=local_point(origin,e,n,p->radius,q->altitude);
        if(isfinite(gate.latitude))drag_taem_latitude=gate.latitude;
    }

    EntryPhase phase=g->entry_exec.initialized?g->entry_exec.phase:ENTRY_PHASE_PREENTRY;
    EntryDragReferenceConfig drag_cfg=entry_drag_reference_default_config();
    drag_cfg.entry_velocity_ratio=g->entry_drag_velocity_ratio;
    double measured_drag=t->mass>1&&isfinite(t->drag_force)&&t->drag_force>0?t->drag_force/t->mass:NAN;
    double modeled_drag=live_drag_accel(t,aero,v);
    EntryDragReferenceInput drag_in={
        .phase=phase,.relative_velocity=t->true_air_speed,.latitude=t->latitude,.altitude=t->mean_altitude,
        .measured_drag_accel=measured_drag,.modeled_drag_accel=modeled_drag,
        .aero_confidence=clampd(fmax(aero.confidence,t->aerodynamic_confidence),0.0,1.0),
        .has_incidence=isfinite(t->angle_of_attack)&&isfinite(t->sideslip),
        .angle_of_attack=t->angle_of_attack,.sideslip=t->sideslip,
        .range_to_site=drag_range,.taem_range=drag_taem_range,
        .taem_latitude=drag_taem_latitude,.taem_altitude=drag_taem_altitude,
        .taem_velocity=taem_velocity,.planet=p,.vehicle=v
    };
    EntryDragReferenceOutput drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
    if(!drag.valid)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 drag-reference contract is invalid; no alternate guidance controller was entered.");

    EntryExecProfile exec_profile=entry_drag_reference_exec_profile(&drag_in,&drag_cfg,&drag);
    /* TAEM readiness is a live handoff contract, not a mutable velocity gate in
       the Entry profile. The executive advances through its longitudinal phases,
       then accepts ownership transfer only through entry_exec_accept_taem_handoff. */
    TaemInterfaceCapture capture=entry_dynamic_interface_capture(g,t,course,p,aero,cfg);
    bool handoff_geometry_ready=capture.valid&&capture.ready;
    if(!g->diagnostic_shadow&&(t->ut-g->taem_interface_diagnostic_ut>=5.0||capture.ready)){
        const TaemInterfaceTarget*q=&g->taem_interface_target;
        char veto_reason[160];
        taem_capture_veto_reasons(capture.veto,veto_reason,sizeof(veto_reason));
        fprintf(stderr,"TAEM interface: UT %.2f live h %.0f V %.1f range %.0f course %.1f FPA %.1f target valid %d energy %d along %.0f cross %.0f h %.0f V %.1f course %.1f FPA %.1f lead %.0f capture valid %d veto %u reasons %s da %.0f dc %.0f dh %.0f dcourse %.1f energy %.0f turn %.0f ready %d\n",
            t->ut,t->mean_altitude,t->true_air_speed,t->range_to_site,course,t->flight_path_angle,
            q->valid,q->energy_qualified,q->along_track,q->cross_track,q->altitude,q->speed,q->course,q->flight_path_angle,q->acquisition_lead,
            capture.valid,capture.veto,veto_reason,capture.along,capture.cross,capture.altitude_error,capture.course_error,
            capture.energy_margin,capture.turn_margin,capture.ready);
        g->taem_interface_diagnostic_ut=t->ut;
    }
    bool checkpoint_restart=!g->entry_exec.initialized&&g->entry_continuation_bootstrap;
    EntryExecObservation obs={
        .ut=t->ut,.relative_velocity=t->true_air_speed,.altitude=t->mean_altitude,
        .dynamic_pressure=fmax(0.0,t->dynamic_pressure),.checkpoint_restart=checkpoint_restart
    };
    bool exec_ok=entry_exec_update(&g->entry_exec,&obs,&exec_profile);
    if(!exec_ok)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 executive rejected its reference profile; Entry ownership is retained without a controller bypass.");
    if(handoff_geometry_ready&&g->entry_exec.phase==ENTRY_PHASE_TRANSITION&&
       !entry_exec_accept_taem_handoff(&g->entry_exec,&obs))
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 handoff was rejected by the Entry executive; the fixed interface contract remains unsatisfied.");
    if(g->entry_exec.phase!=drag_in.phase){
        drag_in.phase=g->entry_exec.phase;
        drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
        if(!drag.valid)return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 phase transition invalidated the drag-reference contract; retaining the current MM304 command.");
    }
    if(g->entry_exec.entry_complete)g->taem_interface_captured=true;

    EntryAlphaSchedule alpha_schedule;
    double alpha_transition_velocity=entry_alpha_transition_velocity(s,taem_velocity);
    entry_alpha_schedule_default(&alpha_schedule,v,alpha_transition_velocity);
    EntryAlphaInput alpha_in={
        .relative_velocity=t->true_air_speed,.phase=entry_alpha_phase_for_exec(g->entry_exec.phase),
        .reference_drag_accel=drag.reference_drag_accel,.measured_drag_accel=drag.observed_drag_accel,
        .current_aoa=t->angle_of_attack,
        .current_aoa_rate=controlled_aoa_rate(t),
        .has_previous_target=g->entry_alpha_has_target,.previous_target_aoa=g->entry_alpha_target,
        .has_previous_modulation=g->entry_alpha_has_modulation,.previous_modulation=g->entry_alpha_modulation,
        .dt=fmax(.001,dt),.maximum_aoa=v->maximum_angle_of_attack,.stall_margin_aoa_limit=NAN,
        .stall_fraction=t->stall_fraction,.stall_fraction_is_measured=t->stall_fraction_is_measured,
        .calibrated_stall_speed=t->calibrated_stall_speed,
        .minimum_safe_speed=v->minimum_safe_speed,.dynamic_pressure=t->dynamic_pressure,
        .maximum_dynamic_pressure=v->maximum_dynamic_pressure,.g_load=t->g_force,
        .maximum_g_load=v->maximum_g_load,
        .aero_confidence=clampd(fmax(drag.confidence,t->aerodynamic_confidence),0.0,1.0)
    };
    EntryAlphaResult alpha=entry_alpha_command(&alpha_schedule,&alpha_in);
    if(!alpha.valid)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 alpha scheduling contract is invalid; retaining the bounded current MM304 command.");
    g->entry_alpha_has_target=true;g->entry_alpha_target=alpha.target_aoa;
    g->entry_alpha_has_modulation=true;g->entry_alpha_modulation=alpha.modulation;

    /* DRAGREF still provides the local longitudinal work request and ENTRYLATERAL
       still translates that request into an authority-aware fallback magnitude.
       Neither one owns S-turn side or reversal timing anymore. */
    EntryLateralLimits limits=entry_lateral_default_limits();
    limits.maximum_bank_deg=dynamic_bank_limit(t,v);
    limits.maximum_roll_rate_deg_s=s->entry_roll_rate;
    limits.maximum_roll_accel_deg_s2=s->entry_roll_acceleration;
    limits.minimum_leg_duration_s=s->s_turn_minimum_leg_duration;
    EntryLateralInput magnitude_in={0};
    magnitude_in.ut=t->ut;magnitude_in.dt=fmax(.001,dt);magnitude_in.relative_speed=t->true_air_speed;
    magnitude_in.measured_bank_deg=norm_signed_deg(t->roll);
    magnitude_in.measured_bank_rate_deg_s=controlled_roll_rate(t);
    magnitude_in.bank_effectiveness=t->bank_effectiveness>0?t->bank_effectiveness:1.0;
    magnitude_in.lift_accel=live_lift_accel(t,aero,v);
    magnitude_in.authority_confidence=clampd(fmax(t->aerodynamic_confidence,t->physics_authority_confidence[1]),0.0,1.0);
    magnitude_in.course_to_site_error_deg=0.0;
    magnitude_in.has_crossrange_error=false;
    magnitude_in.longitudinal=(EntryLateralLongitudinalDemand){
        .valid=drag.valid,.confidence=drag.confidence,.reference_drag_accel=drag.reference_drag_accel,
        .drag_error_accel=drag.drag_error_accel,.predicted_range_error=drag.predicted_range_error,
        .range_error_scale=drag.range_error_scale,.required_vertical_lift_accel=drag.required_vertical_lift_accel
    };
    if(!g->entry_lateral.initialized)
        entry_lateral_state_init(&g->entry_lateral,t->ut,g->s_turn_sign,magnitude_in.measured_bank_deg);
    EntryLateralOutput magnitude=entry_lateral_update(&g->entry_lateral,&magnitude_in,&limits);
    if(!magnitude.valid)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 lateral authority allocation is invalid; retaining the bounded current MM304 command.");

    /*
     * Admit the final MM304 turn from a physical signed envelope rather than
     * speed/heading/corridor thresholds. The envelope projects the measured
     * roll transient with the configured actuator limits, derives minimum turn
     * radius from measured lift and speed, solves the fixed-station circular
     * endpoint geometry, and consumes the explicit MM304 capture-radius
     * contract plus measured trajectory uncertainty.
     */
    if(!g->entry_reversal_scheduled&&!g->entry_final_reversal_pending&&
       !g->entry_final_reversal_completed&&q->valid&&isfinite(q->course)&&
       isfinite(course)&&isfinite(t->runway_along_track)&&
       isfinite(t->runway_cross_track)){
        double horizontal_speed=isfinite(t->horizontal_speed)&&t->horizontal_speed>DBL_MIN?
            t->horizontal_speed:t->surface_speed;
        double lift_accel=live_lift_accel(t,aero,v);
        double authority_confidence=clampd(
            fmax(t->aerodynamic_confidence,t->physics_authority_confidence[1]),0.0,1.0);
        double usable_bank=fmin(limits.maximum_bank_deg,magnitude.bank_magnitude_deg);
        EntryLateralTurnInput turn_input={
            .horizontal_speed_mps=horizontal_speed,
            .current_course_deg=norm_signed_deg(course-cfg->site.runway_heading),
            .target_course_deg=norm_signed_deg(q->course-cfg->site.runway_heading),
            .current_along_m=t->runway_along_track,
            .current_cross_m=t->runway_cross_track,
            .target_along_m=q->along_track,
            .target_cross_m=q->cross_track,
            .measured_bank_deg=norm_signed_deg(t->roll),
            .measured_bank_rate_deg_s=controlled_roll_rate(t),
            .lift_accel_mps2=lift_accel*authority_confidence,
            .bank_effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
                t->bank_effectiveness:1.0,
            .maximum_bank_deg=usable_bank,
            .maximum_roll_rate_deg_s=limits.maximum_roll_rate_deg_s,
            .maximum_roll_accel_deg_s2=limits.maximum_roll_accel_deg_s2,
            .capture_radius_m=s->mm304_handoff_radius,
            .position_uncertainty_m=isfinite(t->trajectory_range_residual)?
                fabs(t->trajectory_range_residual):0.0
        };
        EntryLateralTurnEnvelope turn=
            entry_lateral_terminal_turn_envelope(&turn_input);
        if(turn.valid&&turn.feasibility_margin_m>=0.0){
            EntryControlPlan heading_lock={0};
            heading_lock.valid=true;
            heading_lock.target_bank=turn.turn_sign*usable_bank;
            heading_lock.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));
            heading_lock.has_planned_reversal=true;
            heading_lock.planned_reversal_is_final=true;
            heading_lock.final_heading_lock=true;
            heading_lock.planned_reversal_ut=t->ut;
            heading_lock.planned_reversal_range=
                hypot(q->along_track-t->runway_along_track,q->cross_track-t->runway_cross_track);
            heading_lock.planned_reversal_sign=turn.turn_sign;
            entry_program_commit_planned_reversal(g,&heading_lock,t->ut,false);
            if(!g->diagnostic_shadow)
                fprintf(stderr,
                    "MM304 final turn admitted by physical envelope: UT %.2f sign %+.0f response %.2fs Rmin %.0f Rideal %.0f radiusMargin %.0f captureMargin %.0f endpointErr %.0f m.\n",
                    t->ut,turn.turn_sign,turn.response_time_s,
                    turn.minimum_turn_radius_m,turn.ideal_turn_radius_m,
                    turn.radius_margin_m,turn.capture_margin_m,turn.endpoint_error_m);
        }
    }

    /* A complete propagated topology remains preferred, but a geometry-only
       inlet must not deadlock the executable S-turn. Once the live setup-side
       crossrange contract is physically complete, schedule that measured sign
       change directly; the normal reversal executor still owns roll-through and
       promotion to the final fixed-point turn. */
    (void)entry_program_schedule_live_setup_reversal(g,t,p,cfg,
        magnitude.bank_magnitude_deg);
    (void)entry_program_release_nonfinal_reversal_now(g,t,p,cfg);
    bool reversal_executed=entry_program_execute_planned_reversal(g,t,p,cfg);
    bool tangent_capture_mode=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    if(tangent_capture_mode)entry_program_tangent_reversal_captured(g,t,v);
    bool final_setup_blocked=!tangent_capture_mode&&g->entry_reversal_scheduled&&
        g->entry_reversal_is_final&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,g->entry_reversal_sign);
    /* A nonfinal event whose requested sign already matches the owned S-turn side is
       not a literal reversal. It is a terminal-turn setup gate: hold that side until
       the measured-radius geometry becomes feasible. Once its deadline is reached,
       treat the wait as terminal setup for energy purposes instead of continuing to
       spend speed at entry incidence. */
    bool terminal_side_setup_blocked=!tangent_capture_mode&&g->entry_reversal_scheduled&&
        !g->entry_reversal_is_final&&g->entry_reversal_sign*g->s_turn_sign>0.0&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,g->entry_reversal_sign);
    bool terminal_geometry_blocked=entry_program_terminal_geometry_blocked(g,t,cfg);
    double parallel_stage_bank=0.0;
    bool post_shaping_parallel_stage=!tangent_capture_mode&&
        entry_program_post_shaping_parallel_stage(g,t,course,p,aero,cfg,&parallel_stage_bank);
    double geometry_bank=tangent_capture_mode?0.0:entry_taem_gate_acquisition_bank(g,t,p,aero,cfg);
    EntryControlPlan longitudinal_demand={0};
    longitudinal_demand.valid=true;
    longitudinal_demand.target_bank=(g->s_turn_sign>=0.0?1.0:-1.0)*magnitude.bank_magnitude_deg;
    longitudinal_demand.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));
    /* A missing worker topology is not permission to spend the entire entry
       energy budget at the upper end of the generic alpha schedule.  Before a
       real S-turn leg exists, preserve the lift-bearing trim through the fast
       portion of entry; the measured q/g/stall guards below still own safety.
       This keeps the alpha schedule contract unchanged while preventing the
       70 km live handoff from arriving at TAEM with no disposable energy. */
    bool topology_absent=!g->entry_topology.valid;
    bool fast_energy_leg=topology_absent&&isfinite(t->true_air_speed)&&
        t->true_air_speed>fmax(1400.0,taem_velocity*1.50)&&
        isfinite(t->dynamic_pressure)&&t->dynamic_pressure<v->maximum_dynamic_pressure*.80&&
        isfinite(t->g_force)&&t->g_force<v->maximum_g_load*.85&&
        (!t->stall_fraction_is_measured||t->stall_fraction<=.12);
    if(fast_energy_leg)
        longitudinal_demand.target_aoa=fmin(v->maximum_angle_of_attack,
            fmax(longitudinal_demand.target_aoa,v->entry_angle_of_attack+4.0));
    (void)entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&longitudinal_demand);
    /* The post-shaping runway-parallel segment is the final S-turn energy-dump
       lane even before the fixed-point 90 deg turn is geometrically flyable.
       finalaoa5 reached the -8 km station at ~1.6 km/s because this lane kept the
       old ~20 deg high-L/D incidence. The mission explicitly permits ~35 deg here. */
    bool final_s_turn_armed=tangent_capture_mode||post_shaping_parallel_stage||
        entry_program_terminal_side_final_s_turn_armed(g,t,cfg,course);
    double final_s_turn_drag_aoa=entry_program_final_s_turn_drag_aoa(g,t,p,aero,cfg,&drag,
        final_s_turn_armed);
    if(isfinite(final_s_turn_drag_aoa))
        longitudinal_demand.target_aoa=fmax(longitudinal_demand.target_aoa,final_s_turn_drag_aoa);
    double vertical_bank_magnitude=fabs(longitudinal_demand.target_bank);
    if(post_shaping_parallel_stage){
        longitudinal_demand.target_bank=parallel_stage_bank;
        longitudinal_demand.bank_cap=fabs(parallel_stage_bank);
        longitudinal_demand.target_turn_radius=fabs(parallel_stage_bank)>.25?
            live_turn_radius(t,aero,v,parallel_stage_bank):INFINITY;
    }else{
        entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank,&longitudinal_demand);
    }
    /*
     * Lateral acquisition is owned by entry_taem_gate_acquisition_bank() and
     * entry_program_apply_geometry_bank_demand(). That path computes the
     * required fixed-point capture bank from current geometry and then solves a
     * maximin allocation across vertical-lift, bounded-curvature capture-time,
     * and unpowered-energy margins. Do not overlay fixed course-rate/bank floors
     * or altitude/range windows here; such overlays bypass the same feasibility
     * model they are intended to protect.
     */
    double control_bank_magnitude=fabs(longitudinal_demand.target_bank);
    bool longitudinal_bank_replan=!tangent_capture_mode&&
        entry_program_longitudinal_bank_replan_due(g,&longitudinal_demand,t->ut,s,&limits);
    bool plan_expired=!g->entry_s_turn_plan.valid||!g->entry_control_plan_valid||
        !isfinite(g->entry_s_turn_plan.planned_ut)||g->entry_s_turn_plan.planned_ut>t->ut+1.0||
        !isfinite(g->entry_s_turn_plan.segment_duration)||
        t->ut+1e-6>=g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration;
    bool bank_authority_replan=entry_bank_authority_replan_due(g,t,v);
    bool vertical_authority_growth_replan=!bank_authority_replan&&
        entry_vertical_capture_growth_replan_due(g,t,p,aero,cfg);
    bool supervision_replan=entry_supervision_replan_due(g,t,s);
    bool alpha_safety_replan=false;
    if(g->entry_s_turn_plan.valid){
        /* Stall/g-load recovery is an execution-time AoA override below the nominal
           thermal floor. Re-solving the persistent S-turn topology cannot absorb
           that override because entry_program_plan_s_turn() deliberately reseeds
           target AoA at the thermal floor; doing so every frame creates a solve ->
           thermal-floor plan -> still-stall-limited solve loop. Bank safety still
           has its independent replan path, and the executable entry_control_aoa is
           refreshed from @ALPHA below. Dynamic-pressure protection, by contrast,
           raises the nominal target and therefore still requires a plan refresh. */
        if(alpha.limiting_reason==ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE&&
           g->entry_s_turn_plan.target_aoa<alpha.target_aoa-.25)
            alpha_safety_replan=true;
    }
    bool bank_safety_replan=g->entry_s_turn_plan.valid&&
        fabs(g->entry_s_turn_plan.target_bank)>limits.maximum_bank_deg+.25;
    bool plan_safety_replan=alpha_safety_replan||bank_safety_replan;
    bool solve_plan=state&&!tangent_capture_mode&&(reversal_executed||plan_expired||supervision_replan||
        plan_safety_replan||bank_authority_replan||vertical_authority_growth_replan||longitudinal_bank_replan);
    const char*plan_warning=NULL;

    g->entry_planning_needed=solve_plan;
    if(solve_plan&&!g->entry_planning_deferred){
        double planning_geometry_bank=post_shaping_parallel_stage?fabs(parallel_stage_bank):fabs(geometry_bank);
        EntryControlPlan planned=entry_program_plan_s_turn(g,t,state,p,aero,cfg,
            control_bank_magnitude,planning_geometry_bank,alpha.target_aoa);
        if(planned.valid){

            if(post_shaping_parallel_stage){
                /* Runway-parallel staging is not another S-turn leg. Keep the live
                   vertical-safe parallel command and do not schedule an ordinary
                   reversal away from the eventual inlet while its endpoint circle
                   is still maturing downstream. */
                planned.target_bank=parallel_stage_bank;
                planned.has_planned_reversal=false;
                planned.planned_reversal_is_final=false;
                planned.final_heading_lock=false;
            }
            double planned_sign=fabs(planned.target_bank)>=1.0?(planned.target_bank>=0?1.0:-1.0):g->s_turn_sign;
            bool initial_side_choice=g->entry_control_reversals==0&&!g->has_s_turn_leg_started&&
                !g->entry_reversal_scheduled&&!reversal_executed&&!g->entry_s_turn_plan.valid;
            if(initial_side_choice&&fabs(planned.target_bank)>=1.0)
                request_side(g,planned_sign,t->ut);
            else if(fabs(planned.target_bank)>=1.0&&planned_sign*g->s_turn_sign<0.0){
                /* Once the first leg exists, a new solve may resize it but cannot
                   invent an immediate opposite leg. Only its propagated reversal
                   milestone is allowed to change side. */
                planned.target_bank=g->s_turn_sign*fabs(planned.target_bank);
            }

            planned.planned_ut=t->ut;
            planned.segment_duration=fmax(s->s_turn_minimum_leg_duration,planned.segment_duration);
            if(bank_authority_replan||vertical_authority_growth_replan)
                planned.segment_duration=entry_authority_refresh_child_duration(g,
                    &g->entry_s_turn_plan,t->ut,planned.segment_duration,s);
            planned.final_heading_lock=false;
            control_plan_assign_lineage(g,&planned,&g->entry_s_turn_plan);
            /* A valid future boundary warning or safety-envelope replan requests a
               fresh solve without permission to postpone the durable reversal.
               Only a genuinely invalid supervisory trajectory may replace it. */
            bool terminal_geometry_durable=entry_program_terminal_geometry_blocked(g,t,cfg);
            bool replace_reversal=entry_program_replan_may_discard_reversal(
                plan_safety_replan,supervision_replan&&g->entry_committed_infeasible,
                g->entry_supervision_valid,terminal_geometry_durable);
            entry_program_commit_planned_reversal(g,&planned,t->ut,replace_reversal);
            g->entry_s_turn_plan=planned;
            g->entry_supervision_valid=false;g->entry_supervision_boundary_missed=false;
        }else{
            double fallback_sign=g->s_turn_sign>=0?1.0:-1.0;
            EntryControlPlan fallback={0};
            fallback.valid=true;fallback.terminal_ready=false;fallback.planned_ut=t->ut;
            fallback.target_bank=fallback_sign*control_bank_magnitude;
            fallback.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));fallback.target_heading=norm_deg(t->ground_track_heading);
            fallback.bank_cap=limits.maximum_bank_deg;
            fallback.target_turn_radius=live_turn_radius(t,aero,v,fallback.target_bank);
            fallback.segment_duration=fmax(5.0,s->prediction_interval*2.0);fallback.cost=INFINITY;
            fallback.taem_range_error=drag.predicted_range_error;fallback.taem_speed=NAN;
            fallback.taem_energy_error=t->predicted_taem_energy_error;
            entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&fallback);
            if(!post_shaping_parallel_stage)
                entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank,&fallback);
            control_plan_assign_lineage(g,&fallback,&g->entry_s_turn_plan);
            g->entry_s_turn_plan=fallback;
            plan_warning="MM304 trajectory planner has no finite S-turn solution at this checkpoint; holding the current side with longitudinal energy control and retrying shortly.";
            g->entry_supervision_valid=false;g->entry_supervision_boundary_missed=false;
        }
    }else if(!tangent_capture_mode&&(!state||g->entry_planning_deferred)&&(!g->entry_s_turn_plan.valid||plan_expired)){
        double fallback_sign=g->s_turn_sign>=0?1.0:-1.0;
        EntryControlPlan fallback={0};
        fallback.valid=true;fallback.planned_ut=t->ut;
        fallback.target_bank=fallback_sign*control_bank_magnitude;
        fallback.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));fallback.target_heading=norm_deg(t->ground_track_heading);
        fallback.bank_cap=limits.maximum_bank_deg;
        fallback.target_turn_radius=live_turn_radius(t,aero,v,fallback.target_bank);
        fallback.segment_duration=fmax(5.0,s->prediction_interval*2.0);fallback.cost=INFINITY;
        fallback.taem_range_error=drag.predicted_range_error;fallback.taem_speed=NAN;
        fallback.taem_energy_error=t->predicted_taem_energy_error;
        entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&fallback);
        if(!post_shaping_parallel_stage)
            entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank,&fallback);
        control_plan_assign_lineage(g,&fallback,&g->entry_s_turn_plan);
        g->entry_s_turn_plan=fallback;
        g->entry_supervision_boundary_missed=false;
        plan_warning="MM304 is awaiting a propagated plan; executing current energy and geometry demand on the owned side.";
    }

    /* Normalize recovered/persistent program metadata as well as newly solved plans.
       Emergency stall/g-load recovery is a live execution override, not a thermally
       unsafe trajectory contract for predictor replay or MM304->MM305 continuity. */
    if(g->entry_s_turn_plan.valid)
        g->entry_s_turn_plan.target_aoa=fmax(g->entry_s_turn_plan.target_aoa,
            entry_thermal_protection_aoa_floor(v));
    /* A terminal roll-through is still an MM304-owned executable segment.  If
       the preceding plan expired on the same tick that the final reversal was
       admitted, an async worker result can legitimately be rejected by the
       live-state identity check.  Do not let that race erase the capture
       contract: retain a short child segment with fresh lineage until the next
       predictor result arrives. */
    if(tangent_capture_mode&&!g->entry_s_turn_plan.valid){
        EntryControlPlan capture_plan={0};
        capture_plan.valid=true;
        capture_plan.planned_ut=t->ut;
        capture_plan.segment_duration=fmax(2.0,s->prediction_interval*2.0);
        capture_plan.target_bank=g->entry_control_bank;
        capture_plan.target_aoa=fmax(g->entry_control_aoa,
            entry_thermal_protection_aoa_floor(v));
        capture_plan.target_heading=g->taem_interface_target.valid&&
            isfinite(g->taem_interface_target.course)?g->taem_interface_target.course:
            (isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading);
        capture_plan.bank_cap=limits.maximum_bank_deg;
        capture_plan.target_turn_radius=live_turn_radius(t,aero,v,capture_plan.target_bank);
        capture_plan.final_heading_lock=true;
        capture_plan.terminal_ready=false;
        capture_plan.cost=INFINITY;
        capture_plan.predicted_reversals=g->entry_control_reversals;
        control_plan_assign_lineage(g,&capture_plan,NULL);
        g->entry_s_turn_plan=capture_plan;
    }
    EntryControlPlan nominal=g->entry_s_turn_plan;
    /* The executable policy recomputes coupled energy/geometry/alpha demand on
       every telemetry tick. A worker delay never freezes an old bank or AoA. */
    if(!tangent_capture_mode&&nominal.valid){
        nominal.target_bank=longitudinal_demand.target_bank;
        nominal.target_aoa=longitudinal_demand.target_aoa;
    }
    double lateral_requirement=(tangent_capture_mode||post_shaping_parallel_stage)?0.0:
        entry_taem_gate_required_bank(g,t,p,aero,cfg);
    g->entry_lateral_bank_magnitude=magnitude.bank_magnitude_deg;
    g->entry_geometry_bank=geometry_bank;
    g->entry_vertical_bank_magnitude=vertical_bank_magnitude;
    g->entry_demand_bank=longitudinal_demand.target_bank;
    g->entry_lateral_required_bank=lateral_requirement;
    bool geometry_saturated=!post_shaping_parallel_stage&&
        fabs(geometry_bank)>control_bank_magnitude+1.0;
    bool missing_reversal=lateral_requirement*g->s_turn_sign<-1.0&&!g->entry_reversal_scheduled;
    bool blocked_final=g->entry_reversal_scheduled&&g->entry_reversal_is_final&&
        t->ut>=g->entry_reversal_ut&&!entry_program_final_reversal_geometry_ready(g,t,cfg,g->entry_reversal_sign);
    /* A same-side geometry demand above the instantaneous vertical bank budget is
       saturation, not a topological contradiction. The executable law is already
       clamped to the feasible bank and can keep building crossrange while the async
       worker searches for the reversal. Only a missing required reversal or a due
       final event that still cannot execute makes the lateral program infeasible. */
    g->entry_lateral_infeasible=missing_reversal||blocked_final;
    if(g->entry_lateral_infeasible)nominal.terminal_ready=false;
    if(geometry_saturated&&!g->entry_lateral_infeasible)
        g->entry_planning_needed=true;
    nominal.segment_duration=nominal.valid?
        fmax(0.0,g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration-t->ut):0.0;
    nominal.has_planned_reversal=g->entry_reversal_scheduled;
    nominal.planned_reversal_is_final=g->entry_reversal_scheduled&&g->entry_reversal_is_final;
    nominal.final_heading_lock=tangent_capture_mode;
    nominal.planned_reversal_ut=g->entry_reversal_ut;
    nominal.planned_reversal_range=g->entry_reversal_range;
    nominal.planned_reversal_sign=g->entry_reversal_sign;
    nominal.predicted_reversals=g->entry_control_reversals+(g->entry_reversal_scheduled?1u:0u);
    if(tangent_capture_mode){
        const TaemInterfaceTarget*q=&g->taem_interface_target;
        nominal.valid=true;
        nominal.terminal_ready=capture.ready;
        nominal.planned_ut=t->ut;
        nominal.has_planned_reversal=false;
        nominal.planned_reversal_is_final=false;
        nominal.final_heading_lock=true;
        nominal.segment_duration=fmax(1.0,s->prediction_interval*2.0);
        nominal.bank_cap=limits.maximum_bank_deg;
        nominal.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));
        /* Tangent capture is the actual final S-turn. Preserve an explicit
           excess-energy drag command here instead of replacing it with the
           conservative alpha scheduler during the terminal roll-through. */
        if(isfinite(final_s_turn_drag_aoa))
            nominal.target_aoa=fmax(nominal.target_aoa,final_s_turn_drag_aoa);
        if(q->valid){
            double live_course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
            double terminal_course_error=isfinite(live_course)?norm_signed_deg(q->course-live_course):0.0;
            if(!g->entry_topology_heading_locked&&isfinite(live_course)&&fabs(terminal_course_error)<=8.0)
                g->entry_topology_heading_locked=true;
            /* During the physical terminal turn, acquire the published inlet course
               first.  Using the tangent-line intercept immediately can point 20-30 deg
               away from q->course when the orbiter is still far upstream; v14 then
               relaxed a correct reversal into ~76 deg course and bled below 600 m/s.
               Once the course is actually acquired, switch to the Frenet/line capture
               law for the fixed -8 km station. */
            nominal.target_heading=g->entry_topology_heading_locked?
                entry_taem_tangent_capture_heading(g,t,cfg):q->course;
            if(g->entry_final_reversal_pending){
                /* Do not let the point-capture law cancel the actual final roll-through.
                   Hold the committed opposite side until measured bank has crossed it. */
                double reversal_mag=fmax(4.0,fabs(g->entry_control_bank));
                nominal.target_bank=g->s_turn_sign*fmin(limits.maximum_bank_deg,reversal_mag);
            }else{
                double tangent_geometry_bank=entry_taem_tangent_capture_bank(g,t,aero,cfg,nominal.target_heading);
                double recovery_bearing=NAN;
                bool point_recovery_required=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
                (void)recovery_bearing;
                /* After a normal perpendicular-course capture, allow only a bounded
                   opposite-bank line correction. A missed fixed station is different:
                   MM304 must retain full demonstrated turning authority to reacquire
                   the point instead of continuing straight down the infinite tangent. */
                if(!point_recovery_required&&tangent_geometry_bank*g->s_turn_sign<0.0&&
                   fabs(tangent_geometry_bank)>18.0)
                    tangent_geometry_bank=-g->s_turn_sign*18.0;
                bool heading_lock_required=!g->entry_topology_heading_locked&&isfinite(live_course)&&
                    fabs(terminal_course_error)>8.0;
                nominal.target_bank=tangent_geometry_bank;
                (void)entry_program_altitude_capture(g,t,p,aero,cfg,&nominal.target_bank,&nominal.target_aoa);
                bool preserve_lateral=heading_lock_required||point_recovery_required;
                if(preserve_lateral&&fabs(tangent_geometry_bank)>.25){
                    /* Course/point capture is a hard MM304 handoff condition. Do not
                       let the vertical allocator erase the bank needed to satisfy it;
                       q/g/stall/dynamic-bank limits are already in the bank command. */
                    nominal.target_bank=tangent_geometry_bank;
                }else{
                    double vertical_ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
                    if(isfinite(vertical_ceiling)&&fabs(nominal.target_bank)>vertical_ceiling)
                        nominal.target_bank=copysign(vertical_ceiling,nominal.target_bank);
                    entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,tangent_geometry_bank,&nominal);
                }
                if(point_recovery_required&&t->true_air_speed<s->taem_force_handoff_speed+350.0){
                    double efficient_floor=entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v);
                    nominal.target_aoa=fmin(nominal.target_aoa,fmax(15.0,efficient_floor));
                }
                if(fabs(tangent_geometry_bank)>fabs(nominal.target_bank)+1.0)
                    g->entry_lateral_infeasible=true;
            }
            nominal.target_turn_radius=live_turn_radius(t,aero,v,nominal.target_bank);
        }else{
            /* A final reversal without its precomputed inlet must not reopen ordinary
               S-turn topology. Hold a bounded neutral recovery while MM304 remains owner. */
            nominal.target_heading=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
            nominal.target_bank=0.0;
            nominal.target_turn_radius=INFINITY;
        }
    }

    const char*supervision_warning=NULL;
    /* Supervision is a worker-side replay of this same executable policy. Its
       forecast cannot directly override the live MM304 bank/AoA command. */
    if(!tangent_capture_mode&&(!isfinite(g->entry_supervision_ut)||
       t->ut-g->entry_supervision_ut>=s->prediction_interval))g->entry_planning_needed=true;
    if(g->entry_lateral_infeasible){
        nominal.terminal_ready=false;g->entry_s_turn_plan.terminal_ready=false;
        if(!tangent_capture_mode)g->entry_planning_needed=true;
    }
    if(g->entry_lateral_infeasible)
        supervision_warning="MM304 lateral/vertical budgets conflict with current lift or an uncommitted reversal; tangent delivery is unproven and requires replanning.";
    double bank=nominal.target_bank,aoa=nominal.target_aoa;
    /* Below ~Mach 4, after the committed shaping reversal, TAEM energy reserve
       is an explicit mission requirement. The ordinary low-q belly-first floor
       was re-clamping the executable command to ~18 deg even when the terminal
       leg requested 15 deg, throwing away the speed needed by MM305. Retain the
       normal thermal floor everywhere else and retain every live q/g/stall
       protection here. */
    bool alpha_emergency_override=alpha.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN||
        alpha.limiting_reason==ENTRY_ALPHA_LIMIT_G_LOAD;
    bool terminal_energy_preservation=!tangent_capture_mode&&
        (g->entry_control_reversals>0||final_setup_blocked||terminal_side_setup_blocked||
         terminal_geometry_blocked)&&
        q->valid&&!capture.ready&&t->true_air_speed<s->taem_force_handoff_speed+350.0&&
        t->dynamic_pressure>=fmax(250.0,v->maximum_dynamic_pressure*.008)&&t->dynamic_pressure<v->maximum_dynamic_pressure*.85&&
        t->g_force<v->maximum_g_load*.85&&
        (!t->stall_fraction_is_measured||t->stall_fraction<.10);
    double protective_aoa=terminal_energy_preservation?
        fmax(15.0,entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v)):
        entry_low_q_protective_aoa_floor(t->dynamic_pressure,v);
    if(terminal_energy_preservation&&!isfinite(final_s_turn_drag_aoa))
        aoa=fmin(aoa,protective_aoa);
    if(alpha_emergency_override)aoa=fmin(aoa,alpha.target_aoa);
    else{
        aoa=fmax(aoa,protective_aoa);
        /* A replan is not a safety guarantee: it may be delayed, rejected by
           lineage checks, or fail to produce a usable topology. Enforce ALPHA's
           live q protection on execution as well as requesting a solve. */
        if(alpha.limiting_reason==ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE)
            aoa=fmax(aoa,alpha.target_aoa);
    }
    bank=clampd(bank,-limits.maximum_bank_deg,limits.maximum_bank_deg);
    double executable_aoa_ceiling=isfinite(final_s_turn_drag_aoa)?
        entry_final_s_turn_aoa_ceiling(v):v->maximum_angle_of_attack;
    aoa=clampd(aoa,0.0,executable_aoa_ceiling);

    /* `entry_s_turn_plan` is the persistent program. The entry_control_* fields
       are its current execution mirrors because controller.c publishes and replays
       the exact live command in trajectory forecasts. */
    g->entry_control_plan_valid=nominal.valid;
    g->entry_control_terminal_ready=nominal.terminal_ready;
    g->entry_control_plan_ut=t->ut;
    g->entry_control_bank=bank;g->entry_control_aoa=aoa;g->entry_control_heading=nominal.target_heading;
    g->entry_control_turn_radius=live_turn_radius(t,aero,v,bank);
    g->entry_control_cost=nominal.cost;
    g->entry_control_taem_range_error=nominal.taem_range_error;
    g->entry_control_taem_speed=nominal.taem_speed;
    g->entry_control_taem_energy_error=nominal.taem_energy_error;
    g->entry_control_segment_until_ut=tangent_capture_mode?t->ut+nominal.segment_duration:
        (g->entry_s_turn_plan.valid?g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration:t->ut);

    /* Reversal dwell starts only on a continuously captured, aerodynamically
       loaded S-turn leg; pre-entry roll excursions and wings-level segments do not count. */
    if(!tangent_capture_mode)entry_update_s_turn_leg_capture(g,t,bank,v);

    g->airbrakes_deployed=false;
    GuidanceCommand c=atmospheric(t,nominal.target_heading,bank,v,0,false,PROFILE_ENTRY);
    c.heading_control_enabled=false;c.has_target_aoa=true;c.target_aoa=aoa;c.target_pitch=t->flight_path_angle+aoa;
    char status[448];
    double plan_remaining=fmax(0.0,g->entry_control_segment_until_ut-t->ut);
    double reversal_remaining=g->entry_reversal_scheduled&&isfinite(g->entry_reversal_ut)?
        g->entry_reversal_ut-t->ut:NAN;
    double terminal_arc_error=NAN,terminal_arc_radius=NAN;
    if(g->taem_interface_target.valid){
        double arc_side=g->entry_reversal_scheduled?
            (g->entry_reversal_sign>=0.0?1.0:-1.0):
            (norm_signed_deg(g->taem_interface_target.course-cfg->site.runway_heading)>=0.0?1.0:-1.0);
        (void)entry_program_terminal_course_endpoint_ready(g,t,cfg,arc_side,
            g->taem_interface_target.course,&terminal_arc_error,&terminal_arc_radius);
    }
    if(tangent_capture_mode){
        const TaemInterfaceTarget*q=&g->taem_interface_target;
        double gate_distance=q->valid?hypot(q->along_track-t->runway_along_track,
            q->cross_track-t->runway_cross_track):NAN;
        snprintf(status,sizeof(status),"MM304 tangent capture: gate %.1f km, course %.1f -> %.1f deg, bank %+.1f deg (roll %+.1f), h %.1f/%.1f km, V %.0f/%.0f m/s, final reversal %s.",
            gate_distance/1000.0,t->ground_track_heading,nominal.target_heading,bank,norm_signed_deg(t->roll),
            t->mean_altitude/1000.0,q->valid?q->altitude/1000.0:NAN,t->true_air_speed,q->valid?q->speed:NAN,
            g->entry_final_reversal_pending?"rolling":"captured");
    }else snprintf(status,sizeof(status),"MM304 %s planned S-turn: Dref %.2f m/s2, dD %+.2f, bank %+.1f/Bvert %.1f/Bgeo %.1f/Bdem %.1f deg, roll %+.1f leg %d, Crate %+.2f dps Beff %.2f Rturn %.1f km, alpha %.1f deg, plan %.1fs, reversal %s%.1fs, arcErr %.1f km/R %.1f km.",
        entry_phase_string(g->entry_exec.phase),drag.reference_drag_accel,drag.drag_error_accel,
        bank,vertical_bank_magnitude,fabs(geometry_bank),fabs(longitudinal_demand.target_bank),
        norm_signed_deg(t->roll),g->has_s_turn_leg_started?1:0,
        t->has_course_rate?t->course_rate:t->heading_rate,t->bank_effectiveness,
        isfinite(nominal.target_turn_radius)?nominal.target_turn_radius/1000.0:NAN,
        aoa,plan_remaining,g->entry_reversal_scheduled?"in ":"none/",
        g->entry_reversal_scheduled?reversal_remaining:0.0,
        isfinite(terminal_arc_error)?terminal_arc_error/1000.0:NAN,
        isfinite(terminal_arc_radius)?terminal_arc_radius/1000.0:NAN);
    const char*warning=alpha_emergency_override?
        (alpha.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN?
            "MM304 thermal AoA floor temporarily overridden by explicit stall-margin recovery.":
            "MM304 thermal AoA floor temporarily overridden by explicit g-load recovery."):
        (supervision_warning?supervision_warning:plan_warning);
    if(!warning&&(drag.degraded||alpha.degraded||magnitude.degraded_authority))
        warning="MM304 planned S-turn is operating with degraded aerodynamic/reference authority; the current side is preserved while commands stay inside the demonstrated envelope.";
    /* Bounded entry-pipeline trace for ShuttleSim energy/authority diagnosis. */
    static double entry_trace_last_ut=-INFINITY;
    const char*entry_trace=getenv("KSP_LANDER_ENTRY_TRACE");
    if(entry_trace&&strcmp(entry_trace,"1")==0&&!g->diagnostic_shadow&&
       (t->ut<entry_trace_last_ut||t->ut-entry_trace_last_ut>=5.0)){
        double trace_distance=NAN;
        double trace_fpa=entry_program_taem_path_fpa(g,t,cfg,&trace_distance);
        double trace_ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
        fprintf(stderr,
            "ENTRY_TRACE ut=%.3f h=%.1f V=%.2f q=%.2f fpa=%.2f path_fpa=%.2f path=%.1f "
            "aoa=%.2f final_drag_aoa=%.2f Dref=%.3f Dobs=%.3f DreqV=%.3f lift=%.3f "
            "Braw=%.2f Bvert=%.2f Bgeo=%.2f Bdem=%.2f Bfinal=%.2f roll=%.2f cmd=%.2f "
            "ceiling=%.2f post_parallel=%d reversal=%u/%d\n",
            t->ut,t->mean_altitude,t->true_air_speed,t->dynamic_pressure,
            t->flight_path_angle,trace_fpa,trace_distance,t->angle_of_attack,
            final_s_turn_drag_aoa,drag.reference_drag_accel,drag.observed_drag_accel,
            drag.required_vertical_lift_accel,live_lift_accel(t,aero,v),
            magnitude.bank_magnitude_deg,vertical_bank_magnitude,fabs(geometry_bank),
            fabs(longitudinal_demand.target_bank),fabs(bank),norm_signed_deg(t->roll),
            c.target_roll,trace_ceiling,post_shaping_parallel_stage?1:0,
            g->entry_control_reversals,g->entry_reversal_scheduled?1:0);
        entry_trace_last_ut=t->ut;
    }
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,status,warning),t,v,s,dt);
}
       double live_turn_radius(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v,double bank_deg){
    double lift=live_lift_accel(t,aero,v),eff=clampd(t->bank_effectiveness,.35,1.8);
    double lateral=lift*fabs(sin(clampd(fabs(bank_deg)*eff,0,89)*DEG2RAD));
    return lateral>.005?t->true_air_speed*t->true_air_speed/lateral:INFINITY;
}
