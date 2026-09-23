static EntryControlPlan entry_program_plan_s_turn(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double nominal_bank_magnitude,double geometry_bank_magnitude,double nominal_aoa){
    EntryControlPlan plan={0};plan.cost=INFINITY;
    if(!g||!t||!state||!p||!cfg)return plan;
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    if(!(v->maximum_bank_angle>0.0)||!(v->maximum_angle_of_attack>0.0)||
       !isfinite(nominal_bank_magnitude)||!isfinite(nominal_aoa))return plan;

    AerodynamicEnvelope fallback_env;TrajectoryCalibrationModel fallback_cal;
    const AerodynamicEnvelope*env=&g->entry_predictor_envelope;
    const TrajectoryCalibrationModel*cal=&g->entry_predictor_calibration;
    if(!g->entry_predictor_models_valid){
        entry_supervision_models(t,aero,&fallback_env,&fallback_cal);
        env=&fallback_env;cal=&fallback_cal;
    }

    double roll_rate=controlled_roll_rate(t);
    double aoa_rate=controlled_aoa_rate(t);
    double leg_elapsed=g->has_s_turn_leg_started?fmax(0.0,t->ut-g->s_turn_leg_started_ut):0.0;
    double selected_sign=g->s_turn_sign>=0.0?1.0:-1.0;
    double live_bank_limit=dynamic_bank_limit(t,v);
    if(!isfinite(live_bank_limit)||live_bank_limit<0.0)return plan;

    /*
     * MM304 already owns side, bank magnitude and alpha before this planner is
     * entered.  Do not run a second free-bank/free-alpha search merely to create
     * a provisional segment: build the executable segment directly from the live
     * DRAGREF/ENTRYLATERAL/@ALPHA demand, then let the propagated guidance shadow
     * and asynchronous two-event topology search decide future reversals.
     */
    double seeded_bank=clampd(fabs(nominal_bank_magnitude),0.0,live_bank_limit);
    double seeded_aoa=clampd(fmax(nominal_aoa,entry_thermal_protection_aoa_floor(v)),
        0.0,v->maximum_angle_of_attack);
    double measured_bank=isfinite(t->roll)?norm_signed_deg(t->roll):0.0;
    double bank_delta=fabs(norm_signed_deg(selected_sign*seeded_bank-measured_bank));
    double rate_time=s->entry_roll_rate>DBL_MIN?bank_delta/s->entry_roll_rate:0.0;
    double accel_time=s->entry_roll_acceleration>DBL_MIN?
        sqrt(2.0*bank_delta/s->entry_roll_acceleration):0.0;
    double target_speed=entry_taem_speed_target(v,s,p);
    double minimum_leg=entry_s_turn_effective_minimum_leg(t->true_air_speed,target_speed,s);

    plan.valid=true;
    plan.terminal_ready=false;
    plan.planned_ut=t->ut;
    plan.target_bank=selected_sign*seeded_bank;
    plan.target_aoa=seeded_aoa;
    plan.target_heading=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    plan.bank_cap=fabs(plan.target_bank);
    plan.target_turn_radius=live_turn_radius(t,aero,v,plan.target_bank);
    plan.segment_duration=fmin(1200.0,fmax(minimum_leg,fmax(rate_time,accel_time)));
    plan.has_planned_reversal=false;
    plan.planned_reversal_is_final=false;
    plan.final_heading_lock=false;
    plan.planned_reversal_ut=NAN;
    plan.planned_reversal_range=NAN;
    plan.planned_reversal_sign=0.0;

    (void)entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&plan);
    entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank_magnitude,&plan);

    /*
     * Reforecast the exact executable segment once.  This propagation may propose
     * the next shaping reversal, but it cannot certify terminal ownership; the
     * worker's full evolving-policy shadow remains the acceptance authority.
     */
    plan.segment_duration=fmax(minimum_leg,plan.segment_duration);
    GuidanceMachine event_contract=*g;
    if(g->entry_committed_infeasible){
        event_contract.entry_reversal_scheduled=false;
        event_contract.entry_reversal_is_final=false;
    }
    entry_program_commit_planned_reversal(&event_contract,&plan,t->ut,false);
    const TaemInterfaceTarget*interface_target=
        g->taem_interface_target.valid?&g->taem_interface_target:NULL;
    EntryPrediction shaped=predictor_simulate_entry_control_plan_to_interface(*state,p,aero,env,cal,v,
        &cfg->site,s,interface_target,true,measured_bank,roll_rate,t->angle_of_attack,aoa_rate,
        g->s_turn_sign,leg_elapsed,&plan,1200.0,false);
    double capture_hint=fmax(s->hac_radius*8.0,s->taem_interface_range*2.5);
    if(shaped.entered_atmosphere&&isfinite(shaped.closest_distance)){
        plan.terminal_ready=false;
        plan.taem_range_error=shaped.reached_taem?shaped.taem_range_error:
            shaped.closest_distance-capture_hint;
        plan.taem_speed=shaped.reached_taem?shaped.taem_speed:NAN;
        plan.taem_energy_error=shaped.reached_taem?shaped.taem_energy_error:NAN;
        plan.closest_distance=shaped.closest_distance;
        plan.predicted_reversals=shaped.s_turn_reversals;
        if(shaped.has_first_s_turn_reversal){
            double shaped_sign=fabs(plan.target_bank)>=1.0?
                (plan.target_bank>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
            plan.has_planned_reversal=true;
            plan.planned_reversal_ut=shaped.first_s_turn_reversal_ut;
            plan.planned_reversal_range=shaped.first_s_turn_reversal_range;
            plan.planned_reversal_sign=fabs(shaped.first_s_turn_reversal_sign)>.1?
                shaped.first_s_turn_reversal_sign:-shaped_sign;
            plan.planned_reversal_is_final=false;
            plan.final_heading_lock=false;
        }
    }
    entry_prediction_clear(&shaped);
    return plan;
}

static bool entry_supervision_replan_due(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    if(!g||!t||!s||!g->entry_s_turn_plan.valid||
       !isfinite(g->entry_supervision_ut)||!isfinite(g->entry_s_turn_plan.planned_ut))return false;
    /* Keep a future handoff-window miss observable even while the current segment
       remains safe. A miss reached inside the executable segment is an immediate
       invalidation. A farther-out warning is timed from the persistent executable
       plan, not from the supervisor refresh timestamp: entry_supervision_ut is
       refreshed every prediction_interval and therefore cannot be the dwell clock. */
    if(g->entry_supervision_boundary_missed){
        /* A proven miss invalidates an established plan promptly, but a replacement
           that is rejected again in the same tick must not spawn another child every
           control frame. Use the same age-based dwell as ordinary infeasible
           supervision so a fresh child gets one bounded execution window. */
        if(!g->entry_supervision_valid&&g->entry_supervision_mode==ENTRY_SUPERVISION_INFEASIBLE)
            return t->ut-g->entry_s_turn_plan.planned_ut>=
                fmax(5.0,s->prediction_interval*2.0);
        if(g->entry_supervision_valid)
            return t->ut-g->entry_s_turn_plan.planned_ut>=
                fmax(s->s_turn_minimum_leg_duration,s->prediction_interval);
        return false;
    }
    if(g->entry_supervision_valid||g->entry_supervision_mode!=ENTRY_SUPERVISION_INFEASIBLE)return false;
    return t->ut-g->entry_s_turn_plan.planned_ut>=fmax(5.0,s->prediction_interval*2.0);
}

/* A segment can be committed while thin-air/stall guards correctly deny usable
   bank authority. Once that physical authority first becomes available, the
   pre-leg segment may be reshaped immediately, but the child must not postpone
   the parent's already-scheduled ordinary refresh. Latch the acquisition so
   transient stall chatter cannot turn this into another level-triggered loop. */
static bool entry_bank_authority_replan_due(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v){
    if(!g||!t||!v)return false;
    bool available=entry_s_turn_bank_authority_available(t->dynamic_pressure,
        t->true_air_speed,t->stall_fraction,t->g_force,v);
    if(!available||g->entry_bank_authority_acquired)return false;
    g->entry_bank_authority_acquired=true;
    if(!g->entry_s_turn_plan.valid||!g->entry_control_plan_valid||g->has_s_turn_leg_started)
        return false;
    double parent_end=g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration;
    /* If the parent is already about to refresh, let normal expiry own the next
       solve rather than manufacturing a nearly-zero-lived child. */
    if(isfinite(parent_end)&&parent_end-t->ut<=1.0)return false;
    return true;
}

static double entry_bank_authority_child_duration(const EntryControlPlan*parent,
        double child_ut,double proposed_duration){
    double duration=fmax(0.0,proposed_duration);
    if(!parent||!parent->valid||!isfinite(parent->planned_ut)||
       !isfinite(parent->segment_duration)||!isfinite(child_ut))return duration;
    double parent_end=parent->planned_ut+parent->segment_duration;
    if(!isfinite(parent_end))return duration;
    return fmin(duration,fmax(0.0,parent_end-child_ut));
}

/* Bank authority grows quickly after the first useful-q edge. A persistent segment
   solved near that edge can become badly under-banked tens of seconds before its
   ordinary expiry even though the live vehicle has substantially more authority.
   Re-use the existing vertical-capture law as the demand oracle and only refresh
   when the newly executable same-side bank exceeds the committed demand by a
   material margin. The age gate makes this self-quenching rather than per-frame. */
static bool entry_vertical_capture_growth_replan_due(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->entry_s_turn_plan.valid||!g->entry_control_plan_valid)
        return false;
    const GuidanceSettings*s=&cfg->guidance;
    double age=t->ut-g->entry_s_turn_plan.planned_ut;
    double parent_end=g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    double refresh_period=fmax(fmax(0.0,s->prediction_interval),
        authority.valid?authority.control_response_time_s:0.0);
    if(!isfinite(age)||age+sqrt(DBL_EPSILON)<refresh_period||
       !isfinite(parent_end)||parent_end<=t->ut)
        return false;
    if(!entry_program_bank_capture_available(g,t,p,cfg))return false;

    double demanded_bank=g->entry_s_turn_plan.target_bank;
    double demanded_aoa=g->entry_s_turn_plan.target_aoa;
    GuidanceMachine probe=*g;
    if(!entry_program_altitude_capture(&probe,t,p,aero,cfg,
        &demanded_bank,&demanded_aoa))return false;
    double committed=fabs(g->entry_s_turn_plan.target_bank);
    double demanded=fabs(demanded_bank);
    double resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(committed,demanded));
    return demanded>committed+resolution;
}


static double entry_authority_refresh_child_duration(const GuidanceMachine*g,
        const EntryControlPlan*parent,double child_ut,double proposed_duration,
        const GuidanceSettings*s){
    double duration=entry_bank_authority_child_duration(parent,child_ut,proposed_duration);
    if(!g||!s||!g->entry_reversal_scheduled||!g->has_s_turn_leg_started||
       g->has_s_turn_reversal_requested||!isfinite(g->entry_reversal_ut))return duration;
    double executable_reversal=fmax(g->entry_reversal_ut,
        g->s_turn_leg_started_ut+s->s_turn_minimum_leg_duration);
    if(executable_reversal<=child_ut+1e-6)return duration;
    return fmin(duration,fmax(0.0,executable_reversal-child_ut));
}


/* The MM304 outlet is a pose constraint, not merely a waypoint. Before the final
   reversal, aim the owned S-turn leg at the start of the nominal terminal circle
   that would finish at the fixed TAEM point/course. This converts the required
   crossrange into an explicit setup target instead of relying on a fixed bearing
   lead. The live final-reversal gate still proves the measured-radius arc. */
static bool entry_taem_pose_setup_point(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double*setup_along,double*setup_cross,double*final_side){
    if(setup_along)*setup_along=NAN;if(setup_cross)*setup_cross=NAN;if(final_side)*final_side=0.0;
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!isfinite(q->course))return false;
    double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(fabs(target_relative)<30.0)return false;
    double side=target_relative>=0.0?1.0:-1.0;
    double radius=entry_taem_shaping_setup_radius(q,t,p,cfg);
    if(!isfinite(radius)||radius<1000.0)return false;
    double psi0=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    double psi1=target_relative*DEG2RAD;
    double along=q->along_track-radius/side*(sin(psi1)-sin(psi0));
    double cross=q->cross_track-radius/side*(cos(psi0)-cos(psi1));
    if(!isfinite(along)||!isfinite(cross))return false;
    if(setup_along)*setup_along=along;if(setup_cross)*setup_cross=cross;if(final_side)*final_side=side;
    return true;
}

static void entry_mm304_fixed_handoff_station(double*along,double*cross){
    /* MM304 ownership ends at the user-defined rear alignment point.  The
       instantaneous q/lift turn radius is an executability proof, not permission
       to translate the ownership point tens of kilometres away from KSC. */
    if(along)*along=-8000.0; /* decision-literal-ok: fixed MM304 handoff contract */
    if(cross)*cross=0.0;
}



       bool entry_taem_tangent_target_geometry(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg){
    if(!q||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->speed)||q->speed<=0.0)return false;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    double expected_along=NAN,expected_cross=NAN;
    entry_mm304_fixed_handoff_station(&expected_along,&expected_cross);
    double point_error=hypot(q->along_track-expected_along,
        q->cross_track-expected_cross);
    double runway_offset=fabs(norm_signed_deg(q->course-cfg->site.runway_heading));
    double perpendicular_error=fabs(runway_offset-90.0); /* decision-literal-ok: perpendicular runway geometry */
    return isfinite(point_error)&&
        point_error<=contract.horizontal_radius_m&&
        perpendicular_error<=contract.perpendicular_heading_half_width_deg;
}

static void entry_taem_turnability_target(const GuidanceMachine*g,const Telemetry*t,
        double reference_course,AerodynamicModel aero,const LandingConfiguration*cfg,
        double ordinary_target,double*target_speed,double*target_course){
    if(target_speed)*target_speed=ordinary_target;if(target_course)*target_course=NAN;
    if(!g||!t||!cfg||!isfinite(reference_course))return;
    (void)reference_course;(void)aero;
    /* Publish the centerline of the explicit perpendicular heading contract.
       Acceptance width remains solely in TaemHandoffContract. */
    double side=g->entry_target_side_latched&&fabs(g->entry_target_side)>DBL_EPSILON?
        (g->entry_target_side>=0.0?1.0:-1.0):
        (g->s_turn_sign>=0.0?-1.0:1.0);
    const double perpendicular=90.0; /* decision-literal-ok: perpendicular runway geometry */
    /* Both perpendicular outlets are legal MM304 states: their acceptance
       sectors are 330..030 and 150..210.  Before the side is latched, choose
       the outlet requiring the smaller measured course change.  This keeps an
       initially southbound vehicle on the southbound legal inlet instead of
       commanding a 180 degree reversal merely because the opposite side was
       the historical default.  Preserve the existing side only for the exact
       runway-tangent tie; the latch then makes the choice one-way. */
    if(!g->entry_target_side_latched){
        double live=norm_deg(reference_course);
        double positive=norm_deg(cfg->site.runway_heading+perpendicular);
        double negative=norm_deg(cfg->site.runway_heading-perpendicular);
        double positive_error=fabs(norm_signed_deg(positive-live));
        double negative_error=fabs(norm_signed_deg(negative-live));
        double resolution=sqrt(DBL_EPSILON)*fmax(1.0,
            fmax(positive_error,negative_error));
        if(positive_error<negative_error-resolution)side=1.0;
        else if(negative_error<positive_error-resolution)side=-1.0;
    }
    /* The owned side is also the signed bank/course-rate side in the runway
       frame: positive bank produces positive cross-track course rate. */
    if(target_course)*target_course=norm_deg(cfg->site.runway_heading+side*perpendicular);
}

void entry_publish_taem_tangent_target(GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||g->terminal_region_entered)return;
    /* Keep a geometry-only candidate refreshable.  Its fixed inlet remains the
       same, but ETA and energy qualification are state-dependent; freezing the
       first high-altitude projection preserved a billion-metre turn radius all
       the way into the atmosphere. */
    if(entry_taem_tangent_target_geometry(&g->taem_interface_target,cfg)&&
       g->taem_interface_target.energy_qualified)return;

    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double reference_course=isfinite(course)?norm_deg(course):
        (isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading));
    if(!isfinite(reference_course)||!(t->horizontal_speed>DBL_MIN))return;

    double target_course=NAN;
    double compatibility_speed=entry_taem_speed_target(v,s,p);
    entry_taem_turnability_target(g,t,reference_course,aero,cfg,compatibility_speed,
        NULL,&target_course);
    if(!isfinite(target_course))return;

    double slope=clampd(fabs(s->taem_glide_slope),
        DBL_EPSILON,nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
    double target_fpa=-slope;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!isfinite(authority.control_response_time_s)||
       authority.control_response_time_s<0.0)return;
    double response=authority.control_response_time_s;

    double final_altitude=cfg->site.altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);

    /*
     * MM304 owns a configured interface altitude.  Do not move that ownership
     * surface upward merely because a low-density instantaneous turn-radius
     * estimate grows; downstream maneuver feasibility decides whether the live
     * vehicle may actually cross the boundary.
     */
    double target_altitude=fmax(s->taem_interface_altitude,final_altitude);
    TaemSpeedEnvelope speed_envelope=
        decision_taem_speed_envelope(v,p,target_altitude);
    if(!speed_envelope.valid)return;
    double certified_ld=isfinite(aero.lift_to_drag)&&aero.lift_to_drag>0.0&&
        isfinite(aero.confidence)&&aero.confidence>0.0?
        aero.lift_to_drag:v->estimated_lift_to_drag;
    double radius_scale=v->estimated_lift_to_drag/fmax(certified_ld,DBL_MIN);
    /* Keep the ownership point fixed.  terminal_radius is retained only as an
       advisory downstream executability/energy measure; MM305 independently
       commits its configured runway-anchored HAC after the strict handoff. */
    double terminal_radius=fmax(s->hac_radius,
        speed_envelope.minimum_turn_radius_m*radius_scale);
    double target_along=NAN,target_cross=NAN;
    entry_mm304_fixed_handoff_station(&target_along,&target_cross);

    double rh=cfg->site.runway_heading*DEG2RAD;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double target_e=target_along*sin(rh)+target_cross*cos(rh);
    double target_n=target_along*cos(rh)-target_cross*sin(rh);
    GeoPoint point=local_point(origin,target_e,target_n,
        p->radius,target_altitude);

    HandoffSetEnvelope handoff_set=decision_mm304_handoff_set_envelope(
        g,t,reference_course,target_along,target_cross,p,cfg);
    double minimum_turn_radius=handoff_set.minimum_turn_radius_m;
    double approach_path=handoff_set.path_to_set_m;
    const char*decision_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    bool decision_diag_on=decision_diag&&strcmp(decision_diag,"1")==0;
    if(!handoff_set.valid||!isfinite(approach_path)){
        if(decision_diag_on)fprintf(stderr,
            "MM304 target reject: handoff-set geometry invalid at UT %.2f course %.1f.\n",
            t->ut,reference_course);
        return;
    }
    double relative=fabs(norm_signed_deg(target_course-cfg->site.runway_heading));
    double alignment_arc=relative*DEG2RAD*terminal_radius;
    double remaining=s->final_approach_distance+alignment_arc;
    double distance=hypot(target_along-t->runway_along_track,
        target_cross-t->runway_cross_track);

    /*
     * Keep longitudinal energy accounting independent from the instantaneous
     * lateral-authority proof. handoff_set.path_to_set_m deliberately inflates
     * the route by the turn radius available at the current q/lift. In the upper
     * atmosphere that radius can be thousands of kilometres even though the
     * orbiter will acquire most of its turning authority later in the descent.
     * Feeding that conservative lateral upper bound into the drag integrator made
     * a physically high-energy entry project to zero speed and prevented the
     * topology planner from ever being allowed to schedule its first reversal.
     *
     * Energy qualification therefore follows the finite geometric displacement
     * to the published inlet, plus the published terminal alignment arc. This is
     * not lateral certification: handoff_set, the propagated topology and the live
     * capture contract still independently prove that the path can actually be
     * flown before MM304 can release ownership.
     */
    double energy_path_to_inlet=distance;
    UnpoweredPathProjection arrival=decision_unpowered_path_projection(
        g,t,p,cfg,target_altitude,energy_path_to_inlet);
    if(!arrival.valid){
        if(decision_diag_on)fprintf(stderr,
            "MM304 target reject: energy projection invalid at UT %.2f energyPath %.0f proofPath %.0f Rmin %.0f.\n",
            t->ut,energy_path_to_inlet,approach_path,minimum_turn_radius);
        return;
    }

    /* The published inlet only has to prove that MM304 can deliver an
       unpowered vehicle into the actual MM305 engagement energy band. The
       complete TAEM/HAC/final maneuver is re-propagated by entry_topology_trial()
       and rechecked by entry_taem_interface_capture() at the ownership boundary.
       Requiring a second current-state projection all the way to final approach
       here duplicated that downstream proof and, worse, forced one geometric
       descent profile before the S-turn topology even existed. That circular
       gate prevented every reversal from being planned. */
    double mm305_min_speed=speed_envelope.minimum_speed_mps;
    double target_sound=planet_atmospheric_speed_of_sound(p,target_altitude);
    double mm305_min_mach=fmax(0.0,s->mm305_target_mach-s->mm305_mach_half_width);
    if(isfinite(target_sound)&&target_sound>DBL_MIN&&isfinite(mm305_min_mach))
        mm305_min_speed=fmax(mm305_min_speed,target_sound*mm305_min_mach);
    bool energy_qualified=arrival.valid&&
        arrival.terminal_speed_mps>=mm305_min_speed;
    double target_speed=clampd(compatibility_speed,
        speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps);
    /*
     * A geometry-only target is an acquisition preview, not a certification of
     * the exact fixed-pose turn. At the upper atmosphere the live lateral
     * authority can make handoff_set.path_to_set_m enormous (the instantaneous
     * minimum-turn radius tends toward infinity), even though the vehicle still
     * has a finite forward distance to the handoff station and a finite terminal
     * HAC reserve after that station. Feeding that conservative proof path into
     * arrival_ut makes the preview effectively millions of seconds away and
     * contaminates the first-leg planner. Keep the strict capture veto on the
     * live envelope, but give the advisory target a finite geometric ETA derived
     * from the current station distance plus the configured terminal reserve.
     */
    double geometry_path=isfinite(distance)&&distance>=0.0&&isfinite(remaining)&&remaining>=0.0?
        distance+remaining:approach_path;
    double arrival_time=energy_qualified?arrival.travel_time_s:
        response+geometry_path/fmax(t->horizontal_speed,DBL_MIN);
    if(!isfinite(arrival_time)||arrival_time<0.0)return;
    if(decision_diag_on){
        if(energy_qualified)
            fprintf(stderr,
                "MM304 target accept: inletV %.1f >= MM305min %.1f, guidanceV %.1f structural [%.1f, %.1f], energyPath %.0f proofPath %.0f Rmin %.0f dragWork %.0f travel %.1f.\n",
                arrival.terminal_speed_mps,mm305_min_speed,target_speed,
                speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                energy_path_to_inlet,approach_path,minimum_turn_radius,
                arrival.modeled_drag_work,arrival_time);
        else
            fprintf(stderr,
                "MM304 geometry target: inletV %.1f < MM305min %.1f, guidanceV %.1f structural [%.1f, %.1f], energyPath %.0f proofPath %.0f Rmin %.0f ETA %.1f.\n",
                arrival.valid?arrival.terminal_speed_mps:NAN,mm305_min_speed,target_speed,
                speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                energy_path_to_inlet,approach_path,minimum_turn_radius,arrival_time);
    }

    double lead=t->horizontal_speed*response;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);

    TaemInterfaceTarget next={0};
    next.valid=isfinite(point.latitude)&&isfinite(point.longitude)&&
        isfinite(target_altitude)&&isfinite(target_speed)&&target_speed>0.0&&
        isfinite(target_course)&&isfinite(arrival_time)&&
        contract.horizontal_radius_m>0.0&&
        contract.perpendicular_heading_half_width_deg>0.0;
    if(!next.valid)return;
    next.energy_qualified=energy_qualified;
    next.along_track=target_along;next.cross_track=target_cross;
    next.course=target_course;next.hac_radius=terminal_radius;
    next.altitude=target_altitude;next.speed=target_speed;next.flight_path_angle=target_fpa;
    next.specific_energy=rotating_specific_energy(point.latitude,target_altitude,target_speed,p);
    next.selected_ut=t->ut;next.arrival_ut=t->ut+arrival_time;
    next.quality_score=
        fabs(norm_signed_deg(next.course-reference_course))/
            contract.perpendicular_heading_half_width_deg+
        distance/contract.horizontal_radius_m;
    next.acquisition_lead=lead;
    next.remaining_path=fmax(lead,remaining);
    next.response_time=response;
    g->taem_interface_target=next;
}

static bool entry_taem_alignment_station_missed(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*point_bearing){
    if(point_bearing)*point_bearing=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double rel=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double tangent_along=cos(rel),tangent_cross=sin(rel);
    double from_along=t->runway_along_track-q->along_track;
    double from_cross=t->runway_cross_track-q->cross_track;
    double progress=from_along*tangent_along+from_cross*tangent_cross;
    double distance=hypot(from_along,from_cross);
    double to_along=-from_along,to_cross=-from_cross;
    double bearing=norm_deg(cfg->site.runway_heading+atan2(to_cross,to_along)*RAD2DEG);
    if(point_bearing)*point_bearing=bearing;

    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    if(!isfinite(distance)||distance<=contract.horizontal_radius_m)return false;

    double live=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(live))return isfinite(progress)&&progress>0.0;
    double live_rel=norm_signed_deg(live-cfg->site.runway_heading)*DEG2RAD;
    double forward_to_point=to_along*cos(live_rel)+to_cross*sin(live_rel);
    bool passed_tangent=isfinite(progress)&&progress>0.0;
    bool point_behind_velocity=isfinite(forward_to_point)&&forward_to_point<0.0;
    return passed_tangent||point_behind_velocity;
}

static double entry_taem_tangent_capture_heading(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg){
    if(!g||!t||!cfg)return 0.0;
    if(!g->taem_interface_target.valid||!isfinite(t->runway_along_track)||
       !isfinite(t->runway_cross_track)){
        double live=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        return isfinite(live)?norm_deg(live):0.0;
    }
    double point_bearing=NAN;
    if(entry_taem_alignment_station_missed(g,t,cfg,&point_bearing)){
        double live=isfinite(t->ground_track_heading)?
            norm_deg(t->ground_track_heading):norm_deg(t->heading);
        if(!isfinite(live)||!isfinite(point_bearing))
            return g->taem_interface_target.course;
        double point_turn=norm_signed_deg(point_bearing-live);
        double course_turn=norm_signed_deg(g->taem_interface_target.course-live);
        double resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        double side=fabs(course_turn)>resolution?
            (course_turn>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
        if(side*point_turn>0.0)return point_bearing;
        /* The point is aft of the committed turn. Continue on the geometric
           quarter-turn tangent until it moves onto the owned side; do not
           reverse S-turn ownership merely to chase an aft waypoint. */
        return norm_deg(live+side*90.0); /* decision-literal-ok: quarter-turn tangent geometry */
    }
    return taem_interface_line_heading(t->runway_along_track,t->runway_cross_track,
        cfg->site.runway_heading,&g->taem_interface_target);
}


static double entry_taem_tangent_capture_bank(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const LandingConfiguration*cfg,double desired_heading){
    if(!g||!t||!cfg||!g->taem_interface_target.valid)return 0.0;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    return taem_course_rate_bank_command(t->horizontal_speed,live_lift_accel(t,aero,&cfg->vehicle),
        t->bank_effectiveness>0?t->bank_effectiveness:1.0,dynamic_bank_limit(t,&cfg->vehicle),
        course,desired_heading,g->taem_interface_target.response_time);
}

static double entry_taem_minimum_bank_for_capture(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double target_along,double target_cross,
        double target_course,double side){
    if(!g||!t||!p||!cfg||!isfinite(target_along)||!isfinite(target_cross)||
       !isfinite(target_course)||!(t->horizontal_speed>DBL_MIN))return 0.0;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return 0.0;

    double distance=hypot(target_along-t->runway_along_track,
        target_cross-t->runway_cross_track);
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.controllable)return 0.0;

    double available_time=isfinite(q->arrival_ut)?
        fmax(0.0,q->arrival_ut-t->ut):
        authority.control_response_time_s+distance/t->horizontal_speed;
    if(!isfinite(available_time))return 0.0;

    double lift=live_lift_accel(t,aero,&cfg->vehicle);
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double limit=dynamic_bank_limit(t,&cfg->vehicle);
    if(!(lift>DBL_MIN)||!isfinite(limit)||limit<=0.0)return 0.0;

    double signed_side=side>=0.0?1.0:-1.0;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,limit);
    double max_effective=fmin(limit*effectiveness,nextafter(90.0,0.0));
    double max_lateral=lift*fabs(sin(max_effective*DEG2RAD));
    TargetCaptureEnvelope maximum=
        decision_target_capture_envelope_with_lateral_accel(
            g,t,course,target_along,target_cross,target_course,
            available_time,max_lateral,p,cfg);
    if(!maximum.valid||!maximum.reachable)return signed_side*limit;

    double low=0.0,high=limit;
    unsigned guard=0;
    while(high-low>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(low+high); /* decision-literal-ok: bisection midpoint */
        double effective=fmin(mid*effectiveness,nextafter(90.0,0.0));
        double lateral=lift*fabs(sin(effective*DEG2RAD));
        TargetCaptureEnvelope capture=
            decision_target_capture_envelope_with_lateral_accel(
                g,t,course,target_along,target_cross,target_course,
                available_time,lateral,p,cfg);
        if(capture.valid&&capture.reachable)high=mid;
        else low=mid;
    }
    return signed_side*high;
}

static double entry_taem_gate_required_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return 0.0;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return 0.0;

    TaemInterfaceCapture handoff=
        entry_taem_interface_capture(q,t,course,p,cfg);
    if(handoff.valid&&handoff.ready)return 0.0;

    double side=g->s_turn_sign>=0.0?1.0:-1.0;
    double target_along=q->along_track,target_cross=q->cross_track;
    double target_course=q->course,final_side=0.0;
    double setup_along=NAN,setup_cross=NAN;
    bool have_setup=entry_taem_pose_setup_point(g,t,p,cfg,
        &setup_along,&setup_cross,&final_side);
    bool shaping_leg=have_setup&&side*final_side<0.0&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,final_side);
    if(shaping_leg){
        target_along=setup_along;
        target_cross=setup_cross;
        target_course=course;
    }
    return entry_taem_minimum_bank_for_capture(g,t,p,aero,cfg,
        target_along,target_cross,target_course,side);
}

static double entry_taem_gate_acquisition_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid)return 0.0;
    double bank=entry_taem_gate_required_bank(g,t,p,aero,cfg);
    double side=g->s_turn_sign>=0.0?1.0:-1.0;
    if(bank*side>0.0)return bank;

    double recovery_bearing=NAN;
    bool missed=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
    (void)recovery_bearing;
    bool blocked=entry_program_terminal_geometry_blocked(g,t,cfg);
    if(!missed&&!blocked)return 0.0;

    /* Missed-point recovery and an overdue blocked endpoint are topological
       requirements, not invitations to invent a tuned bank floor. Request the
       full live envelope; the coupled maximin allocator decides what portion is
       executable after vertical and energy margins are included. */
    return side*dynamic_bank_limit(t,&cfg->vehicle);
}


/* After the first energy-management reversal, keep the acquired setup-side
   crossrange and carry it downstream on a runway-parallel leg until the measured
   endpoint arc becomes feasible. This prevents the shaping reversal and the final
   inlet turn from collapsing into one long maximum-bank maneuver. */
static bool entry_program_post_shaping_parallel_stage(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double*bank_out){
    if(bank_out)*bank_out=0.0;
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       g->entry_control_reversals==0||g->entry_final_reversal_pending||
       g->entry_final_reversal_completed||!isfinite(course)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(target_relative)||fabs(target_relative)<30.0)return false;
    double final_side=target_relative>=0.0?1.0:-1.0;
    double owned_side=g->s_turn_sign>=0.0?1.0:-1.0;
    if(owned_side*final_side<=0.0||t->runway_along_track>=q->along_track-1000.0)return false;
    if(entry_program_final_reversal_geometry_ready(g,t,cfg,final_side))return false;

    /* After the first shaping reversal, the runway-parallel segment is a
       deceleration lane, not proof that the final 90 deg arc is already flyable.
       Requiring crossrange equal to the entire current upstream chord made this
       state unreachable at entry speed, so MM304 simply held maximum bank until
       the safety handoff. Preserve a HAC-sized lateral offset instead; the strict
       endpoint test above still owns release into the final perpendicular turn. */
    double setup_side=-final_side;
    double built_cross=setup_side*(t->runway_cross_track-q->cross_track);
    if(!isfinite(built_cross)||built_cross<-500.0)return false;

    /* The setup proof was consumed when the first shaping reversal executed. Do
       not require the crossrange to remain frozen afterward: a real final turn
       must spend that offset while bending back toward the fixed station. The old
       runway-parallel hold drove commanded bank to zero for tens of seconds in
       finalaoa7, leaving course near 90 deg all the way to the safety boundary.

       Lead the fixed tangent line directly with a bounded intercept heading. This
       is a decelerating spiral/conic lead, not permission to hand off early: the
       <=1 km point/course/energy/HAC capture contract remains unchanged. */
    double desired=entry_taem_tangent_capture_heading(g,t,cfg);
    double turn_error=norm_signed_deg(desired-course);
    double bank=0.0;
    if(isfinite(desired)&&final_side*turn_error>0.75){
        double limit=dynamic_bank_limit(t,&cfg->vehicle);
        /* finalaoa13 proved that artificially capping this lead at 60 deg made
           coordination worse: beta grew past 50 deg and tripped the genuine yaw
           departure guard. The 70 deg vehicle envelope in finalaoa12 remained
           controllable, so let the measured q/g/vertical guards own the limit. */
        double vertical=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
        if(isfinite(vertical))limit=fmin(limit,fmax(0.0,vertical));
        if(limit>0.25)
            bank=taem_course_rate_bank_command(t->horizontal_speed,
                live_lift_accel(t,aero,&cfg->vehicle),
                t->bank_effectiveness>0?t->bank_effectiveness:1.0,limit,
                course,desired,q->response_time);
        if(bank*final_side<0.0)bank=0.0;
    }
    if(bank_out)*bank_out=bank;
    return true;
}

static bool entry_program_tangent_reversal_captured(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v){
    if(!g||!t||!v||!g->entry_final_reversal_pending)return g&&g->entry_final_reversal_completed;
    bool authority=entry_s_turn_bank_authority_available(t->dynamic_pressure,t->true_air_speed,
        t->stall_fraction,t->g_force,v);
    double actual=norm_signed_deg(t->roll);
    double roll_rate=controlled_roll_rate(t);
    double angle_resolution=sqrt(DBL_EPSILON)*fmax(1.0,fabs(actual));
    double rate_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,isfinite(roll_rate)?fabs(roll_rate):1.0);
    bool crossed=g->s_turn_sign*actual>angle_resolution;
    bool not_rolling_back=!isfinite(roll_rate)||
        g->s_turn_sign*roll_rate>=-rate_resolution;
    if(authority&&crossed&&not_rolling_back){
        g->entry_final_reversal_pending=false;
        g->entry_final_reversal_completed=true;
        return true;
    }
    return false;
}

       TaemInterfaceCapture entry_dynamic_interface_capture(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg){
    (void)aero;
    if(!g||!t||!p||!cfg)return (TaemInterfaceCapture){0};
    TaemInterfaceCapture out=entry_taem_interface_capture(
        &g->taem_interface_target,t,course,p,cfg);
    /* MM304 owns only this precomputed tangent-state contract.  A future HAC or
       spline preview is deliberately not an ownership prerequisite: MM305 must be
       free to select its own HAC after receiving the required position, altitude,
       speed and tangential course. */
    out.ready=out.valid&&out.veto==0;
    return out;
}

