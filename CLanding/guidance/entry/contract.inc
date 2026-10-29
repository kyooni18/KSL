static double entry_diagnostic_milestone_due(GuidanceMachine*g,double range_m){
    static const double milestone_m[]={220000.0,170000.0,140000.0,100000.0};
    const char*trace=getenv("KSP_LANDER_TRACE_ENTRY_MILESTONES");
    if(!g||!g->diagnostic_shadow||!trace||strcmp(trace,"1")!=0||!isfinite(range_m))
        return NAN;
    double previous=g->diagnostic_entry_has_previous_range?
        g->diagnostic_entry_previous_range_m:range_m;
    g->diagnostic_entry_previous_range_m=range_m;
    g->diagnostic_entry_has_previous_range=true;
    for(unsigned i=0;i<sizeof(milestone_m)/sizeof(milestone_m[0]);i++){
        unsigned bit=1U<<i;
        if(!(g->diagnostic_entry_milestone_mask&bit)&&previous>milestone_m[i]&&
           range_m<=milestone_m[i]){
            g->diagnostic_entry_milestone_mask|=bit;
            return milestone_m[i];
        }
    }
    return NAN;
}

/* KSP_LANDER_ENTRY_AOA_STAIRCASE_TEST: test-only incidence staircase for the
   aero-cell campaign; NAN when unset or below the last Mach threshold. */
static double entry_aoa_staircase_test(double mach){
    const char*spec=getenv("KSP_LANDER_ENTRY_AOA_STAIRCASE_TEST");
    if(!spec||!*spec||!isfinite(mach))return NAN;
    for(const char*p=spec;p&&*p;){
        char*end=NULL;
        double threshold=strtod(p,&end);
        if(end==p||*end!=':')return NAN;
        p=end+1;
        double aoa=strtod(p,&end);
        if(end==p)return NAN;
        if(mach>=threshold)return aoa;
        p=*end==','?end+1:NULL;
    }
    return NAN;
}

/* Max-L/D incidence at this Mach from the (live-fitted) aero table. */
static double entry_max_lift_to_drag_aoa(const TerminalModel*m,double q,double mach,
        double ceiling){
    if(!m||!isfinite(mach)||!isfinite(ceiling)||ceiling<=0.0)return NAN;
    double best=NAN,best_ld=-INFINITY;
    for(double a=4.0;a<=ceiling+1e-9;a+=0.5){
        double lq=0.0,dq=0.0;
        aero_force_per_q(&m->aero,fmax(q,1.0),mach,a,&lq,&dq);
        if(dq>DBL_MIN&&lq/dq>best_ld){best_ld=lq/dq;best=a;}
    }
    return best;
}

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
    double milestone=entry_diagnostic_milestone_due(g,t->range_to_site);
    if(isfinite(milestone))
        fprintf(stderr,
            "MM304_SHADOW_MILESTONE hold=1 milestone=%.0f ut=%.2f h=%.0f V=%.1f M=%.2f fpa=%.2f range=%.0f status=\"%s\" previousBank=%.1f previousAoA=%.1f feedbackValid=%d lateralValid=%d\n",
            milestone,t->ut,t->mean_altitude,t->true_air_speed,t->mach,
            t->flight_path_angle,t->range_to_site,warning?warning:"",
            bank,aoa,g->entry_feedback.valid?1:0,g->entry_feedback.lateral_valid?1:0);
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,
        "MM304 feedback model unavailable; retaining the bounded current Entry command.",
        warning),t,v,s,dt);
}

GuidanceResult entry_program_guidance(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;

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
    double measured_drag=entry_measured_drag_accel(t);
    /* Longitudinal energy belongs to the complete unpowered arrival, not a
       radial shell around the runway. Preserve the Final reserve while MM304
       controls entry. An inlet/route, when qualified, remains MM305's contract. */
    double alignment_distance=final_alignment_distance_for_state(g,t->mean_altitude,
        t->true_air_speed,&cfg->site,v,s);
    FinalAlignmentContract alignment=final_alignment_contract(alignment_distance,&cfg->site,v,s);
    double alignment_altitude=alignment.altitude_m;
    double alignment_speed=alignment.speed_mps;
    /* Entry can start on the far side of the planet. Use the same spherical
       runway aimpoint as downstream geometry instead of a tangent-plane chord. */
    double ground_radius=p->radius+cfg->site.altitude;
    GeoPoint alignment_point=runway_approach_aimpoint(&cfg->site,ground_radius,
        alignment.distance_m);
    GeoPoint current_point={t->latitude,t->longitude,t->mean_altitude};
    double alignment_range=great_circle_distance(current_point,alignment_point,ground_radius);
    bool sts_delivery=s->sts_trajectory_delivery&&!strcmp(v->model_id,"STS");
    /* The HAC law also wants the approach side: a straight-in cone instead
       of a 180+ deg overhead turn priced from a supersonic inlet. */
    bool hac_delivery=s->taem_hac_law&&!strcmp(v->model_id,"STS");
    if((sts_delivery||hac_delivery)&&cfg->site.allow_reciprocal_runway){
        /* Experimental STS: deliver to the approach side of the runway end
           TAEM will fly.  The 60 deg full test steered for the RW09 aimpoint
           while TAEM chose RW27, and handed over abeam at Mach 2.85 needing a
           reversal.  The end is the one reached with the smaller heading
           change onto its runway; it is re-chosen until the TAEM interface
           range and then held, and passed to TAEM as its preview. */
        /* cfg->site is already the preview end (guidance_terminal.c swaps
           it for end 1); index the configured end as 0. */
        LandingSite base=g->runway_end_preview_valid&&g->runway_end_index==1?
            runway_reciprocal_site(&cfg->site,ground_radius):cfg->site;
        LandingSite ends[2]={base,runway_reciprocal_site(&base,ground_radius)};
        GeoPoint aims[2];double turn[2],range[2];
        for(int e=0;e<2;++e){
            aims[e]=runway_approach_aimpoint(&ends[e],ground_radius,alignment.distance_m);
            range[e]=great_circle_distance(current_point,aims[e],ground_radius);
            turn[e]=fabs(norm_signed_deg(ends[e].runway_heading-
                initial_bearing(current_point,aims[e])));
        }
        int end=g->runway_end_preview_valid?g->runway_end_index:0;
        if(range[end]>entry_taem_range_target(p,s)||!g->runway_end_preview_valid){
            /* Keep the held end until reaching it would need more than a
               perpendicular turn (arrival from its departure side); near a
               perpendicular approach both ends cost alike and switching only
               moves the aimpoint across the runway every cycle. */
            int best=!g->runway_end_preview_valid?(turn[1]<turn[0]?1:0):
                (turn[end]>90.0&&turn[1-end]<turn[end]?1-end:end);
            if(best!=end||!g->runway_end_preview_valid)
                fprintf(stderr,"MM304 delivery runway end: UT %.1f RW%s turn09=%.1f turn27=%.1f range=%.0f\n",
                    t->ut,best==1?"27":"09",turn[0],turn[1],range[best]);
            end=best;
            g->runway_end_index=end;
            g->runway_end_preview_valid=true;
        }
        alignment_point=aims[end];
        alignment_range=range[end];
    }
    if(!alignment.valid||!isfinite(alignment_range)||!isfinite(alignment_altitude))
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 alignment energy contract has invalid geometry.");
    /* Near the terminal region the great-circle chord can be much shorter
       than the path a bank-limited orbiter must actually fly. Charge the
       bounded turn and course alignment to the drag budget while there is
       still speed to preserve. The runway-frame solver is local geometry, so
       retain the spherical distance on the far side of Kerbin. */
    double energy_range=alignment_range;
    if(t->true_air_speed<=s->taem_force_handoff_speed&&
       alignment_range<=entry_taem_range_target(p,s)&&
       isfinite(t->runway_along_track)&&isfinite(t->runway_cross_track)&&
       isfinite(course)){
        double turn_path=decision_target_path_length(g,t,course,
            -alignment.distance_m,0.0,cfg->site.runway_heading,NAN,p,cfg,NULL);
        if(isfinite(turn_path))energy_range=fmax(energy_range,turn_path);
    }
    EntryDragReferenceInput drag_in={
        .phase=phase,
        .relative_velocity=t->true_air_speed,
        .latitude=t->latitude,
        .altitude=t->mean_altitude,
        .state=state,
        .measured_drag_accel=measured_drag,
        .modeled_drag_accel=live_drag_accel(t,aero,v),
        .aero_confidence=clampd(fmax(aero.confidence,t->aerodynamic_confidence),0.0,1.0),
        .has_incidence=isfinite(t->angle_of_attack)&&isfinite(t->sideslip),
        .angle_of_attack=t->angle_of_attack,
        .sideslip=t->sideslip,
        .range_to_site=energy_range,
        .taem_range=0.0,
        .taem_latitude=cfg->site.latitude,
        .taem_altitude=alignment_altitude,
        .taem_velocity=alignment_speed,
        .planet=p,
        .vehicle=v
    };
    /* With the TAEM energy law, everything below the MM305 physical handoff
       (Mach mm305TargetMach+halfWidth) is flown by that law under its own
       polar.  Budget MM304 to deliver that handoff speed at the TAEM
       interface altitude where the law's mid-drag glide still reaches the
       Final station, instead of extrapolating MM304's drag profile down to
       Final speed (2026-10-14: handoff 100 km out needing 94 km of path with
       79 km of TAEM reach). */
    /* HAC law: MM304 delivers the point of MM305's own energy reference at
       the band-centre Mach (mm305_hac_handoff_state), not a straight-in
       reach at full drag: the 2026-10-21 live run handed over high and
       energy-rich on that reach and MM305 overran. */
    double hac_h=NAN,hac_v=NAN,hac_range=NAN;
    if(cfg->guidance.taem_hac_law&&g->entry_terminal_model){
        mm305_hac_observe_aero(g,t);
        /* Live energy against the reference at the measured path to the
           station (the handoff gate in guidance_terminal.c reads it). */
        double e_ref=NAN;
        g->entry_hac_energy_excess=mm305_hac_reference_energy(g->entry_terminal_model,
            &g->taem_hac_live_aero,t->mass,fmax(0.0,energy_range),&e_ref)?
            t->mean_altitude-cfg->site.altitude+
            t->true_air_speed*t->true_air_speed/(2.0*planet_surface_gravity(p))-e_ref:NAN;
        g->entry_hac_energy_ut=t->ut;
        if(mm305_hac_handoff_state(g->entry_terminal_model,&g->taem_hac_live_aero,t->mass,
               s->mm305_target_mach,&hac_h,&hac_v,&hac_range)&&t->true_air_speed>hac_v){
            drag_in.taem_velocity=hac_v;
            drag_in.taem_altitude=hac_h+cfg->site.altitude;
            double target=hac_range+alignment.distance_m;
            if(!g->diagnostic_shadow&&!(fabs(target-g->entry_handoff_target_range)<2000.0))
                fprintf(stderr,"MM304 HAC handoff target: UT %.1f Mach %.2f h %.0f m V %.0f m/s "
                    "station range %.1f km, site %.1f km\n",t->ut,s->mm305_target_mach,hac_h,hac_v,
                    hac_range/1000.0,target/1000.0);
            g->entry_handoff_target_range=target;
            drag_in.taem_range=hac_range;
        }
    }
    else if(cfg->guidance.taem_energy_law&&g->entry_terminal_model){
        double interface_altitude=s->taem_interface_altitude;
        double sound=planet_atmospheric_speed_of_sound(p,interface_altitude);
        /* Experimental STS budgets the handoff band centre, not its upper
           edge: the 60 deg full test arrived at Mach 2.85 too fast to turn. */
        double handoff_speed=(s->mm305_target_mach+
            (sts_delivery?0.0:s->mm305_mach_half_width))*sound;
        double reach=mm305_energy_law_handoff_reach(g->entry_terminal_model,
            interface_altitude-cfg->site.altitude,handoff_speed,t->mass);
        if(isfinite(handoff_speed)&&handoff_speed>0.0&&isfinite(reach)&&
           t->true_air_speed>handoff_speed){
            drag_in.taem_velocity=handoff_speed;
            drag_in.taem_altitude=interface_altitude;
            g->entry_handoff_target_range=reach+alignment.distance_m;
            drag_in.taem_range=reach;
        }
    }
    EntryDragReferenceOutput drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
    if(!drag.valid)
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 drag-reference state is invalid; Entry ownership is retained.");

    /* Phase/thermal sequencing still uses the configured inlet velocity band;
       those phase gates do not define the downstream energy budget. */
    EntryDragReferenceInput phase_in=drag_in;
    phase_in.taem_velocity=taem_velocity;
    phase_in.taem_altitude=s->taem_interface_altitude;
    EntryExecProfile exec_profile=entry_drag_reference_exec_profile(&phase_in,&drag_cfg,&drag);
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

    /* Full inlet energy integration is irrelevant outside the immutable
       kinematic/survivability gates. A stop-at-MM305 witness uses the physical
       boundary independently of energy or route qualification, so it need not
       perform the admission integration on every 0.1 s simulated FCS tick. */
    TaemInterfaceCapture admission={0};
    unsigned quick_veto=entry_mm305_admission_prefilter_veto(t,p,cfg);
    if((g->diagnostic_shadow&&g->diagnostic_stop_at_taem)||quick_veto)
        admission.veto=quick_veto;
    else
        admission=entry_mm305_admission_envelope(g,t,course,p,cfg);
    double diagnostic_milestone=entry_diagnostic_milestone_due(g,t->range_to_site);
    bool handoff_ready=admission.valid&&admission.ready;
    /* DRAGREF is the single MM304 longitudinal controller. It already
       re-solves the drag-v profile from remaining range and bounds that profile
       with measured aerodynamic authority; do not solve a second independent
       energy profile and overwrite its vertical-lift demand. */
    double transition_velocity=entry_alpha_transition_velocity(s,taem_velocity);
    double measured_drag_accel=isfinite(measured_drag)?measured_drag:
        live_drag_accel(t,aero,v);

    EntryAlphaSchedule alpha_schedule;
    entry_alpha_schedule_default(&alpha_schedule,v,transition_velocity);
    /* Rudder authority collapses as incidence rises and density falls.  Live
       polar entry with drag modulation lifting AoA from 22 to 28 degrees ran
       yaw to 100 percent while sideslip kept growing (departure at Mach 3.6).
       Yaw demand is measured against its physical limit (1.0), so the unused
       fraction is the reserve.  Blend the incidence ceiling from the vehicle
       maximum (full reserve) to the lift-bearing schedule value, never above
       the present incidence (no reserve): no tuned Mach/altitude gate. */
    double yaw_use=t->has_controls&&isfinite(t->control_yaw)?
        clampd(fabs(t->control_yaw),0.0,1.0):0.0;
    double alpha_ceiling=v->maximum_angle_of_attack;
    if(yaw_use>0.0&&isfinite(t->angle_of_attack)){
        double nominal=entry_alpha_schedule_interpolate(&alpha_schedule,t->true_air_speed);
        double hold=isfinite(nominal)?fmin(t->angle_of_attack,nominal):t->angle_of_attack;
        alpha_ceiling=alpha_ceiling+(hold-alpha_ceiling)*yaw_use;
    }
    alpha_ceiling=fmax(alpha_ceiling,alpha_schedule.minimum_protective_aoa);
    EntryAlphaInput alpha_in={
        .relative_velocity=t->true_air_speed,
        .phase=entry_alpha_phase_for_exec(g->entry_exec.phase),
        .reference_drag_accel=drag.reference_drag_accel,
        .measured_drag_accel=measured_drag_accel,
        .current_aoa=t->angle_of_attack,
        .current_aoa_rate=controlled_aoa_rate(t),
        .has_previous_target=g->entry_alpha_has_target,
        .previous_target_aoa=g->entry_alpha_target,
        .has_previous_modulation=g->entry_alpha_has_modulation,
        .previous_modulation=g->entry_alpha_modulation,
        .dt=fmax(DBL_EPSILON,dt),
        .maximum_aoa=alpha_ceiling,
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
    limits.maximum_bank_deg=fmin(dynamic_bank_limit(t,v),MM304_PLANNED_MAX_BANK_DEG);
    limits.maximum_roll_rate_deg_s=s->entry_roll_rate;
    limits.maximum_roll_accel_deg_s2=s->entry_roll_acceleration;
    /* Roll authority follows yaw reserve: the less rudder margin remains, the
       slower every bank exchange.  The reversal look-ahead is built from these
       limits, so a slower roll starts the reversal earlier (higher, faster)
       and spreads it over more time instead of rolling hard at high incidence. */
    {
        double reserve=fmax(DBL_EPSILON,1.0-yaw_use);
        limits.maximum_roll_rate_deg_s*=reserve;
        limits.maximum_roll_accel_deg_s2*=reserve;
    }

    EntryLateralInput lateral={0};
    lateral.ut=t->ut;
    lateral.dt=fmax(DBL_EPSILON,dt);
    lateral.relative_speed=t->true_air_speed;
    lateral.measured_bank_deg=norm_signed_deg(t->roll);
    lateral.measured_bank_rate_deg_s=controlled_roll_rate(t);
    lateral.bank_effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    lateral.lift_accel=guidance_feedback_lift_accel(g,t,aero,v);
    lateral.has_measured_lift=t->physics_sample_valid&&isfinite(t->mass)&&t->mass>DBL_MIN&&
        isfinite(t->lift_force)&&t->lift_force>0.0;
    /* Predict the feedback policy that will consume the future sensor force.
       Keep physics_sample_valid false and model confidence unchanged in the
       synthetic telemetry; uncertainty belongs to the force model, not a
       second attenuation of the controller's usable local observation. */
    lateral.has_simulated_lift_observation=g->diagnostic_shadow&&
        isfinite(t->mass)&&t->mass>DBL_MIN&&isfinite(t->lift_force)&&t->lift_force>0.0;
    lateral.authority_confidence=clampd(
        fmax(t->aerodynamic_confidence,t->physics_authority_confidence[1]),0.0,1.0);
    lateral.course_to_site_error_deg=isfinite(t->course_to_site_error)?
        t->course_to_site_error:0.0;
    lateral.has_crossrange_error=isfinite(t->runway_cross_track);
    lateral.crossrange_error_m=lateral.has_crossrange_error?-t->runway_cross_track:0.0;

    const char *arrest_test=getenv("KSP_LANDER_ENTRY_LATERAL_ARREST_TEST");
    /* Roll reversal alone does not arrest cross-track velocity.  On the
       34-degree return it still carried the vehicle north of KSC by MM305.
       Include the measured lift/bank stopping distance by default; retain
       explicit zero only for controlled offline comparison. */
    lateral.crossrange_arrest_prediction=!arrest_test||strcmp(arrest_test,"0")!=0;
    double map_radius=p->radius+fmax(0.0,t->mean_altitude);
    double map_scale=map_radius>DBL_EPSILON?p->radius/map_radius:1.0;
    double map_speed=fmax(0.0,t->horizontal_speed)*map_scale;
    double course_relative=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    if(lateral.has_crossrange_error&&isfinite(map_speed)&&isfinite(course_relative)){
        lateral.has_crossrange_rate=true;
        lateral.crossrange_error_rate_mps=-map_speed*sin(course_relative);
        lateral.has_crossrange_course_alignment=true;
        lateral.crossrange_course_alignment=cos(course_relative);
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
    /* Energy-short (the live replan is short even at 0 deg bank): nulling
       crossrange to zero spends range MM304 does not have.  Live 2026-10-16
       (10 deg): 40 deg of purely lateral bank from 57 to 44 km nulled 100 km
       of crossrange, overshot to +22 km and turned a surplus into a deficit;
       MM305 then flew the 21 km offset without difficulty.  Accept a
       TAEM-sized crossrange while short. */
    if(g->entry_policy_valid&&
       isfinite(g->entry_policy_bank_deg)&&g->entry_policy_bank_deg<=0.5)
        lateral.crossrange_uncertainty_m=fmax(lateral.crossrange_uncertainty_m,
            MM304_ENERGY_SHORT_CROSSRANGE_TOLERANCE_M);
    lateral.has_sideslip=isfinite(t->sideslip)&&
        isfinite(cfg->calibration.maximum_sideslip)&&cfg->calibration.maximum_sideslip>0.0;
    lateral.sideslip_deg=lateral.has_sideslip?t->sideslip:0.0;
    lateral.maximum_sideslip_deg=lateral.has_sideslip?cfg->calibration.maximum_sideslip:0.0;
    lateral.longitudinal=(EntryLateralLongitudinalDemand){
        .valid=drag.valid,
        .confidence=drag.confidence,
        .reference_drag_accel=drag.reference_drag_accel,
        .drag_error_accel=drag.drag_error_accel,
        .predicted_range_error=drag.predicted_range_error,
        .range_error_scale=drag.range_error_scale,
        .required_vertical_lift_accel=drag.required_vertical_lift_accel
    };

    /* A live physics replan supersedes the analytic drag-v range model for
       the energy share of the bank: fly the vertical lift of its solved bank. */
    if(g->entry_policy_valid&&isfinite(g->entry_policy_bank_deg)&&
       isfinite(lateral.lift_accel)&&lateral.lift_accel>0.0){
        double eff=clampd(g->entry_policy_bank_deg*lateral.bank_effectiveness,0.0,
            nextafter(90.0,0.0));
        lateral.longitudinal.required_vertical_lift_accel=lateral.lift_accel*cos(eff*DEG2RAD);
        lateral.longitudinal.valid=true;
    }
    /* Convert DRAGREF's vertical-lift demand to bank against measured
       lift inside the lateral allocator.  Supplying a separate precomputed bank
       magnitude here bypasses that contract. */
    lateral.has_bank_magnitude=false;
    /* 40 deg is MM304's nominal planning ceiling, not a promise that the
       airframe can coordinate that bank at every q/Mach point.  Coordination
       feasibility is predictive: compare the measured time for beta to reach
       the established 5 deg runaway scale with the time the airframe needs to
       unload its present bank.  If beta is escaping faster than bank can be
       removed, reduce the *planned* magnitude early.  The ordinary lateral
       allocator still applies the physical roll-rate/acceleration limits. */
    if(t->has_controls&&isfinite(t->sideslip)&&isfinite(t->control_yaw)){
        double signed_beta=norm_signed_deg(t->sideslip);
        double beta=fabs(signed_beta);
        double yaw_use=clampd(fabs(t->control_yaw),0.0,1.0);
        bool corrective_yaw=signed_beta*t->control_yaw>0.0;
        double outward_beta_rate=(signed_beta>=0.0?1.0:-1.0)*
            g->entry_coordination_beta_rate;

        double direct_scale=1.0;
        if(corrective_yaw&&yaw_use>DBL_EPSILON&&beta>DBL_EPSILON){
            double beta_load=clampd(beta/MM304_COORDINATION_BETA_SCALE_DEG,0.0,1.0);
            direct_scale=1.0-beta_load*yaw_use;
        }

        double predictive_scale=1.0;
        if(corrective_yaw&&outward_beta_rate>DBL_EPSILON&&
                beta<MM304_COORDINATION_BETA_SCALE_DEG){
            double rate_limit=fmax(DBL_EPSILON,fabs(limits.maximum_roll_rate_deg_s));
            double accel_limit=fmax(DBL_EPSILON,fabs(limits.maximum_roll_accel_deg_s2));
            double bank_span=fabs(lateral.measured_bank_deg);
            /* Conservative measured-state time to wings level: traverse the
               remaining bank at the allowed rate and reserve time to arrest the
               current roll rate.  This is rebuilt each guidance tick. */
            double rollout_time=bank_span/rate_limit+
                fabs(lateral.measured_bank_rate_deg_s)/accel_limit;
            double beta_margin=MM304_COORDINATION_BETA_SCALE_DEG-beta;
            double beta_escape_time=beta_margin/outward_beta_rate;
            if(rollout_time>DBL_EPSILON)
                predictive_scale=clampd(beta_escape_time/rollout_time,0.0,1.0);
        }

        limits.maximum_bank_deg*=fmin(direct_scale,predictive_scale);
    }
    lateral.has_azimuth_error=isfinite(t->course_to_site_error);
    lateral.azimuth_error_deg=lateral.has_azimuth_error?t->course_to_site_error:0.0;
    /* Experimental STS steers for the same aimpoint its energy targets. */
    if(sts_delivery&&isfinite(t->ground_track_heading)){
        lateral.has_azimuth_error=true;
        lateral.azimuth_error_deg=norm_signed_deg(
            initial_bearing(current_point,alignment_point)-t->ground_track_heading);
    }
    {
        /* Azimuth reversals respect only what navigation can resolve: the
           angle the crossrange uncertainty subtends at the capture point. */
        lateral.azimuth_deadband_deg=atan2(lateral.crossrange_uncertainty_m,
            fmax(alignment_range,DBL_EPSILON))*RAD2DEG;
        /* Speed falls from the present value toward the alignment contract;
           the mean of the two converts remaining range into time-to-go. */
        double mean_speed=0.5*(map_speed+fmax(0.0,alignment_speed));
        lateral.has_time_to_go=isfinite(mean_speed)&&mean_speed>DBL_EPSILON&&
            isfinite(alignment_range);
        lateral.time_to_go_s=lateral.has_time_to_go?alignment_range/mean_speed:NAN;
        /* Far from the site the approach is not along the runway line, so
           runway-frame crossrange is geometry, not miss. The miss is where
           the present ground track passes the capture point. */
        lateral.has_point_miss=isfinite(t->course_to_site_error)&&isfinite(alignment_range);
        lateral.point_miss_m=lateral.has_point_miss?
            alignment_range*sin(t->course_to_site_error*DEG2RAD):NAN;
        /* A straight ground track is wrong over long ranges: the vehicle
           follows a great circle of the inertial trajectory plane while the
           capture point rotates with the planet. Measure the miss as the
           capture point's angular distance from that plane at arrival. */
        double track=t->ground_track_heading*DEG2RAD;
        double lat=t->latitude*DEG2RAD,lon=t->longitude*DEG2RAD;
        if(lateral.has_time_to_go&&isfinite(track)&&isfinite(t->horizontal_speed)&&
           isfinite(p->rotational_speed)){
            Vector3 up=v3(cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat));
            Vector3 east=v3(-sin(lon),cos(lon),0.0);
            Vector3 north=vcross(up,east);
            double r=p->radius+t->mean_altitude;
            Vector3 v_surface=vscale(vadd(vscale(north,cos(track)),vscale(east,sin(track))),
                t->horizontal_speed);
            Vector3 v_inertial=vadd(v_surface,vscale(east,p->rotational_speed*r*cos(lat)));
            Vector3 n=vnorm(vcross(up,v_inertial),v3(0,0,0));
            double slat=alignment_point.latitude*DEG2RAD;
            double slon=alignment_point.longitude*DEG2RAD+p->rotational_speed*lateral.time_to_go_s;
            Vector3 site=v3(cos(slat)*cos(slon),cos(slat)*sin(slon),sin(slat));
            double side=vdot(n,site);
            if(isfinite(side)&&vmag(n)>0.0){
                lateral.has_point_miss=true;
                /* Plane normal points left of track; course error is positive right. */
                lateral.point_miss_m=-ground_radius*asin(clampd(side,-1.0,1.0));
            }
        }
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

    double bank=lateral_out.target_bank_deg;
    double bank_resolution=sqrt(DBL_EPSILON)*fmax(1.0,limits.maximum_bank_deg);
    /* Only the energy share of the bank speaks for drag; bank raised for a
       crossrange miss is not evidence that bank can no longer shed energy. */
    bool bank_at_limit=limits.maximum_bank_deg>DBL_EPSILON&&
        lateral_out.energy_bank_magnitude_deg>=limits.maximum_bank_deg-bank_resolution;
    bool structural_clear=isfinite(t->dynamic_pressure)&&isfinite(t->g_force)&&
        t->dynamic_pressure<v->maximum_dynamic_pressure&&t->g_force<v->maximum_g_load;
    /* If the bounded bank solution still cannot achieve DRAGREF's measured-
       authority drag request, incidence is the remaining drag lever. */
    {
        double headroom=fmax(0.0,v->maximum_angle_of_attack-alpha.target_aoa);
        bool need_drag=bank_at_limit&&structural_clear&&drag.drag_error_accel>0.0;
        /* Mirror the deorbit predictor (entry_bank): incidence rises with the
           measured drag deficit, alpha = trim + span*(1 - D/D_ref), not only
           after bank saturates.  Trim incidence alone left the bay-closed
           polar run (L/D 0.56 at Mach 6) with a +307 km surplus at 51 km that
           bank could not shed in time; it passed 18 km from KSC at 35 km. */
        double deficit=isfinite(measured_drag_accel)&&drag.reference_drag_accel>1e-6?
            clampd(1.0-measured_drag_accel/drag.reference_drag_accel,0.0,1.0):0.0;
        double ceiling=fmin(alpha_ceiling,v->maximum_angle_of_attack);
        /* Measured drag answers the incidence within one cycle and the yaw-
           reserve ceiling follows rudder use, so feeding either straight back
           produced a 2-degree target-AoA limit cycle at 62 km (10:07 polar
           retake).  Low-pass both before they set the boost goal. */
        if(!g->entry_alpha_filter_valid){
            g->entry_alpha_deficit_filter=deficit;g->entry_alpha_ceiling_filter=ceiling;
            g->entry_alpha_filter_valid=true;
        }else{
            double a=clampd(dt/8.0,0.0,1.0);
            g->entry_alpha_deficit_filter+=(deficit-g->entry_alpha_deficit_filter)*a;
            /* A tighter ceiling (more yaw demand) is honoured faster. */
            double b=clampd(dt/(ceiling<g->entry_alpha_ceiling_filter?2.0:8.0),0.0,1.0);
            g->entry_alpha_ceiling_filter+=(ceiling-g->entry_alpha_ceiling_filter)*b;
        }
        double proportional=structural_clear?
            fmax(0.0,g->entry_alpha_ceiling_filter-alpha.target_aoa)*g->entry_alpha_deficit_filter:0.0;
        double goal=need_drag?headroom:fmin(headroom,proportional);
        double rate=(need_drag?1.0:0.5)*fmax(0.0,dt);
        g->entry_alpha_drag_boost=g->entry_alpha_drag_boost<goal?
            fmin(goal,g->entry_alpha_drag_boost+rate):
            fmax(goal,g->entry_alpha_drag_boost-rate);
        g->entry_alpha_drag_boost=fmin(g->entry_alpha_drag_boost,headroom);
    }
    double aoa=clampd(alpha.target_aoa+g->entry_alpha_drag_boost,0.0,v->maximum_angle_of_attack);
    {
        /* Aero-cell campaign only (MM304 rewrite Phase 0): hold a Mach-keyed
           incidence staircase so 25-45 deg cells at Mach 4-8 get steady
           samples.  Format "mach:aoa,..." with descending Mach; each AoA is
           held while Mach >= its threshold, normal guidance below the last.
           Stall and g-load protection still win. */
        double test_aoa=entry_aoa_staircase_test(t->mach);
        if(isfinite(test_aoa)){
            if(alpha.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN||
               alpha.limiting_reason==ENTRY_ALPHA_LIMIT_G_LOAD)
                test_aoa=fmin(test_aoa,alpha.target_aoa);
            /* After attitude recovery the last MM304 command is stale; ramp
               from the measured incidence.  Pitch up at half rate: live
               flight A3 overshot 40 to 47 deg under full nose-down elevon
               at q < 60 Pa. */
            bool fresh=g->entry_control_plan_valid&&isfinite(g->entry_control_aoa)&&
                t->ut-g->entry_control_plan_ut<1.0;
            double previous=fresh?g->entry_control_aoa:t->angle_of_attack;
            double down=alpha_schedule.target_rate_limit*fmax(0.0,dt);
            if(isfinite(previous))test_aoa=clampd(test_aoa,previous-down,previous+0.5*down);
            aoa=clampd(test_aoa,0.0,v->maximum_angle_of_attack);
        }
    }
    {
        /* Low-energy logic (Shuttle ILEFLG): when even wings-level cannot
           reach the handoff, high incidence only throws range away (live
           L/D 0.16-0.19 at 40 deg against ~0.5 at 20 deg).  Cap AoA at the
           max-L/D incidence until the forecast at that incidence shows a
           real surplus again. */
        bool policy_fresh=g->entry_policy_valid&&isfinite(g->entry_policy_bank_deg)&&
            isfinite(g->entry_policy_ut)&&
            t->ut-g->entry_policy_ut<=3.0*fmax(s->prediction_interval,1.0);
        bool dragref_short=drag.predicted_range_error<0.0&&
            drag.drag_range_scale<=drag_cfg.minimum_drag_scale+1e-6;
        if((policy_fresh&&g->entry_policy_bank_deg<=0.5)||dragref_short)
            g->entry_energy_short_latched=true;
        else if(policy_fresh&&g->entry_policy_bank_deg>=10.0)
            g->entry_energy_short_latched=false;
        if(g->entry_energy_short_latched){
            double best=entry_max_lift_to_drag_aoa(g->entry_terminal_model,
                t->dynamic_pressure,t->mach,v->maximum_angle_of_attack);
            if(!isfinite(best))best=v->entry_angle_of_attack;
            best=fmax(best,alpha_schedule.minimum_protective_aoa);
            if(aoa>best){
                double previous=g->entry_control_plan_valid&&isfinite(g->entry_control_aoa)&&
                    t->ut-g->entry_control_plan_ut<1.0?g->entry_control_aoa:t->angle_of_attack;
                double step=alpha_schedule.target_rate_limit*fmax(0.0,dt);
                aoa=isfinite(previous)?fmax(best,fmin(aoa,previous-step)):best;
            }
        }
    }
    double heading=isfinite(course)?norm_deg(course):norm_deg(t->heading);

    g->entry_feedback=(EntryGuidanceFeedback){
        .valid=true,
        .lateral_valid=true,
        .reversal_requested=lateral_out.reversal_requested,
        .ut=t->ut,
        .confidence=fmin(drag.confidence,lateral.authority_confidence),
        .measured_drag_accel=drag.observed_drag_accel,
        .reference_drag_accel=drag.reference_drag_accel,
        .drag_error_accel=drag.drag_error_accel,
        .predicted_achievable_range=drag.predicted_range_to_taem,
        .required_range_to_go=drag.available_range_to_taem,
        .energy_margin=drag.predicted_range_error,
        .range_error_scale=drag.range_error_scale,
        .required_vertical_lift_accel=drag.required_vertical_lift_accel,
        .crossrange_error=lateral.has_crossrange_error?lateral.crossrange_error_m:NAN,
        .crossrange_rate=lateral.has_crossrange_rate?lateral.crossrange_error_rate_mps:NAN,
        .projected_crossrange_error=lateral_out.projected_crossrange_error_m,
        .crossrange_corridor=lateral_out.crossrange_corridor_m,
        .energy_bank_magnitude_deg=lateral_out.energy_bank_magnitude_deg,
        .commanded_bank_deg=bank,
        .bank_sign=side
    };

    if(isfinite(diagnostic_milestone))
        fprintf(stderr,
            "MM304_SHADOW_MILESTONE hold=0 milestone=%.0f ut=%.2f phase=%s h=%.0f V=%.1f M=%.2f q=%.0f fpa=%.2f range=%.0f predRange=%.0f reqRange=%.0f rangeErr=%+.0f Dtelemetry=%.3f Dmodel=%.3f Dref=%.3f bankEnergy=%.1f objective=siteBearing azErr=%+.2f xte=%+.0f bankCmd=%+.1f roll=%+.1f aoaNom=%.1f aoaCmd=%.1f aoa=%.1f d/e/a/l=%d%d%d%d degraded=%d%d%d%d sat=%d%d%d mm305Ready=%d veto=0x%x\n",
            diagnostic_milestone,t->ut,entry_phase_string(g->entry_exec.phase),
            t->mean_altitude,t->true_air_speed,t->mach,t->dynamic_pressure,
            t->flight_path_angle,t->range_to_site,drag.predicted_range_to_taem,
            drag.available_range_to_taem,drag.predicted_range_error,
            drag.observed_drag_accel,drag_in.modeled_drag_accel,
            drag.reference_drag_accel,lateral_out.bank_magnitude_deg,
            lateral.azimuth_error_deg,-t->runway_cross_track,bank,
            norm_signed_deg(t->roll),alpha.nominal_aoa,aoa,t->angle_of_attack,
            drag.valid?1:0,drag.valid?1:0,alpha.valid?1:0,lateral_out.valid?1:0,
            drag.degraded?1:0,drag.degraded?1:0,alpha.degraded?1:0,lateral_out.degraded_authority?1:0,
            0,bank_at_limit?1:0,structural_clear?0:1,
            handoff_ready?1:0,admission.veto);

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
    g->entry_control_taem_range_error=drag.predicted_range_error;
    /* Publish an inlet speed only when downstream propagation has actually
       qualified that inlet; the sequencing velocity is not an energy target. */
    bool qualified_inlet=g->mm305_admission_valid&&g->mm305_admission_found&&
        g->mm305_admission_route.valid&&g->mm305_admission_route.profile_speed_tabulated&&
        t->ut>=g->mm305_admission_result_ut&&
        t->ut-g->mm305_admission_result_ut<=MM305_ADMISSION_MAX_AGE_S;
    g->entry_control_taem_speed=qualified_inlet?
        g->mm305_admission_route.profile_speed_lut[0]:NAN;
    g->entry_control_taem_energy_error=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,alignment_altitude,alignment_speed,p);
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
        entry_phase_string(g->entry_exec.phase),drag.observed_drag_accel,
        drag.reference_drag_accel,drag.drag_error_accel,
        drag.predicted_range_to_taem/1000.0,drag.available_range_to_taem/1000.0,
        drag.predicted_range_error/1000.0,bank,
        lateral.has_crossrange_error?lateral.crossrange_error_m/1000.0:NAN,
        aoa,handoff_ready?"admissible":"not yet admissible");

    char admission_warning[384];
    const char*warning=NULL;
    if(drag.degraded||alpha.degraded||lateral_out.degraded_authority)
        warning="MM304 is using degraded aerodynamic feedback; commands remain inside measured/configured authority and no fallback trajectory is active.";
    if(drag.predicted_range_error<0.0&&
       drag.drag_range_scale<=drag_cfg.minimum_drag_scale+1e-6)
        warning="MM304 maximum-range drag profile cannot reach the required Final alignment path; preserve energy and correct lateral displacement before MM305.";
    bool inlet_band=t->mean_altitude>=s->mm305_min_altitude&&
        t->mean_altitude<=s->mm305_max_altitude&&
        isfinite(t->mach)&&fabs(t->mach-s->mm305_target_mach)<=s->mm305_mach_half_width;
    if(inlet_band&&!admission.valid)
        warning="MM305 energy projection is invalid or unconverged; MM304 retains ownership. Verify native atmosphere, force evidence and response limits.";
    else if(inlet_band&&(admission.veto&16u)){
        snprintf(admission_warning,sizeof(admission_warning),
            "MM305 alignment reserve is not certified (%+.0f J/kg, drag %.0f, uncertainty %.0f). MM304 retains ownership; preserve energy and reduce lateral displacement before the inlet.",
            admission.energy_margin,admission.energy_drag_work,admission.energy_uncertainty);
        warning=admission_warning;
    }

    static _Thread_local double entry_trace_last_ut=-INFINITY;
    const char*entry_trace=getenv("KSP_LANDER_ENTRY_TRACE");
    if(entry_trace&&strcmp(entry_trace,"1")==0&&
       (t->ut<entry_trace_last_ut||t->ut-entry_trace_last_ut>=5.0)){
        fprintf(stderr,
            "ENTRY_TRACE ut=%.3f h=%.1f V=%.2f q=%.2f fpa=%.2f D=%.3f Dref=%.3f dD=%+.3f Rach=%.1f Rreq=%.1f margin=%+.1f along=%+.1f xtk=%+.1f azerr=%+.1f hdotref=%+.1f Benergy=%.2f Bcmd=%+.2f roll=%+.2f aoa=%.2f admissionE=%+.0f Eavail=%.0f Edrag=%.0f Eunc=%.0f path=%.0f reversal=%u handoff=%d veto=%u eh=%+.0f D0=%.2f sat=%d%d%d\n",
            t->ut,t->mean_altitude,t->true_air_speed,t->dynamic_pressure,
            t->flight_path_angle,drag.observed_drag_accel,drag.reference_drag_accel,
            drag.drag_error_accel,drag.predicted_range_to_taem,
            drag.available_range_to_taem,drag.predicted_range_error,t->runway_along_track,t->runway_cross_track,
            lateral.azimuth_error_deg,NAN,
            lateral_out.bank_magnitude_deg,bank,norm_signed_deg(t->roll),aoa,
            admission.energy_margin,admission.energy_available,admission.energy_drag_work,
            admission.energy_uncertainty,admission.path_length_m,g->entry_control_reversals,
            handoff_ready?1:0,admission.veto,NAN,drag.constant_drag_accel,
            0,bank_at_limit?1:0,structural_clear?0:1);
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
