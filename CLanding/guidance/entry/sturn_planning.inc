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
    double minimum_leg=entry_s_turn_effective_minimum_leg(t->true_air_speed,0.0,s);
    double roll_response=roll_capture_time(t,selected_sign*seeded_bank,
        s->entry_roll_rate,s->entry_roll_acceleration);
    if(!isfinite(roll_response))roll_response=minimum_leg;

    plan.valid=true;
    plan.terminal_ready=false;
    plan.planned_ut=t->ut;
    plan.target_bank=selected_sign*seeded_bank;
    plan.target_aoa=seeded_aoa;
    plan.target_heading=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    plan.bank_cap=fabs(plan.target_bank);
    plan.target_turn_radius=live_turn_radius(t,aero,v,plan.target_bank);
    plan.segment_duration=fmin(1200.0,fmax(minimum_leg,roll_response));
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




double entry_taem_tangent_target_hac_side(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg){
    if(!q||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->hac_radius)||!(q->hac_radius>0.0))
        return 0.0;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    double runway=cfg->site.runway_heading*DEG2RAD;
    double best_side=0.0,best_error=INFINITY;
    for(int turn=-1;turn<=1;turn+=2){
        HACPoint2 entry={0},center={0},exit={0};
        if(!hac_fixed_alignment_geometry(&entry,&center,&exit,&cfg->site,
                &cfg->guidance,q->hac_radius,q->course,(double)turn))continue;
        double expected_along=entry.e*sin(runway)+entry.n*cos(runway);
        double expected_cross=entry.e*cos(runway)-entry.n*sin(runway);
        double error=hypot(q->along_track-expected_along,
            q->cross_track-expected_cross);
        if(isfinite(error)&&error<best_error){
            best_error=error;
            best_side=(double)turn;
        }
    }
    return isfinite(best_error)&&best_error<=contract.horizontal_radius_m?
        best_side:0.0;
}

bool entry_taem_tangent_target_geometry(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg){
    if(!q||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->speed)||q->speed<=0.0||
       !isfinite(q->hac_radius)||!(q->hac_radius>0.0))return false;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    const double perpendicular=90.0; /* decision-literal: mathematical-numerical-requirement | MM304 inlet tangent is perpendicular to runway */
    double perpendicular_error=fabs(fabs(relative)-perpendicular);
    if(!isfinite(perpendicular_error)||
       perpendicular_error>contract.perpendicular_heading_half_width_deg)return false;

    /* A perpendicular inlet course can belong to either the short or long
       runway-anchored HAC. The point identifies which physical turn was selected. */
    return fabs(entry_taem_tangent_target_hac_side(q,cfg))==1.0;
}

static bool entry_taem_hac_inlet_target(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double reference_course,
        double hac_radius,double*target_along,double*target_cross,
        double*target_course,double*target_side){
    if(target_along)*target_along=NAN;
    if(target_cross)*target_cross=NAN;
    if(target_course)*target_course=NAN;
    if(target_side)*target_side=NAN;
    if(!g||!t||!p||!cfg||!(hac_radius>0.0)||!isfinite(reference_course)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;

    /*
     * Choose the inlet by the live bounded-curvature path, not by Euclidean
     * position alone.  The old selector could choose an inlet a few hundred
     * metres away whose tangent course was 180 degrees opposite the velocity,
     * then freeze that choice.  A shuttle already flying toward the opposite
     * inlet would consequently pass KSC while MM304 waited for an impossible
     * reversal.
     */
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||
       !(authority.maximum_lateral_accel_mps2>DBL_MIN)||
       !(t->horizontal_speed>DBL_MIN))return false;
    const double perpendicular=90.0; /* decision-literal: mathematical-numerical-requirement | perpendicular HAC inlet tangent */
    double runway=cfg->site.runway_heading*DEG2RAD;
    bool have=false;
    double best_score=INFINITY,best_along=NAN,best_cross=NAN,best_course=NAN,best_side=NAN;
    double retained_course=g->taem_interface_target.valid&&
        isfinite(g->taem_interface_target.course)?g->taem_interface_target.course:NAN;
    double retained_side=entry_taem_tangent_target_hac_side(
        &g->taem_interface_target,cfg);

    /*
     * A perpendicular inlet course does not determine the HAC turn side.  The
     * same course has both a short-arc and long-arc runway-anchored solution.
     * Evaluate all four physical inlet poses and let live bounded-curvature
     * reachability choose the one MM304 can actually deliver.
     */
    for(int direction=-1;direction<=1;direction+=2){
        double course=norm_deg(cfg->site.runway_heading+direction*perpendicular);
        for(int turn=-1;turn<=1;turn+=2){
            double side=(double)turn;
            HACPoint2 entry={0},center={0},exit={0};
            if(!hac_fixed_alignment_geometry(&entry,&center,&exit,&cfg->site,
                    &cfg->guidance,hac_radius,course,side))continue;
            double along=entry.e*sin(runway)+entry.n*cos(runway);
            double cross=entry.e*cos(runway)-entry.n*sin(runway);
            if(!isfinite(along)||!isfinite(cross))continue;

            /*
             * Evaluate the complete bounded-curvature connection to the candidate
             * pose. Euclidean distance plus a heading-debt scalar is not equivalent
             * to a Dubins path and can select an inlet behind the vehicle.
             */
            double candidate_minimum_radius=INFINITY;
            double score=decision_target_path_length(g,t,reference_course,
                along,cross,course,authority.maximum_lateral_accel_mps2,
                p,cfg,&candidate_minimum_radius);
            if(!isfinite(score)||!isfinite(candidate_minimum_radius))continue;

            bool retained=isfinite(retained_course)&&fabs(retained_side)==1.0&&
                fabs(norm_signed_deg(course-retained_course))<=
                    sqrt(DBL_EPSILON)*RAD2DEG&&
                side==retained_side;
            double scale=fmax(1.0,fmax(fabs(score),fabs(best_score)));
            bool tied=have&&isfinite(best_score)&&
                fabs(score-best_score)<=sqrt(DBL_EPSILON)*scale;
            if(!have||score<best_score||(tied&&retained)){
                have=true;best_score=score;best_along=along;best_cross=cross;
                best_course=course;best_side=side;
            }
        }
    }
    if(!have)return false;
    if(target_along)*target_along=best_along;
    if(target_cross)*target_cross=best_cross;
    if(target_course)*target_course=best_course;
    if(target_side)*target_side=best_side;
    return true;
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

    double compatibility_speed=hac_acquisition_speed_target(v,s,p);

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

    /* The published tangent is not the MM304->MM305 ownership surface.  It is
       the later Phase-1/Phase-2 acquisition target that MM305 flies toward after
       entering TAEM at the high-energy interface. */
    double target_altitude=s->hac_acquisition_altitude;
    if(!isfinite(target_altitude)||target_altitude<=final_altitude||
       target_altitude>=s->taem_interface_altitude)return;
    TaemSpeedEnvelope speed_envelope=
        decision_taem_speed_envelope(v,p,target_altitude);
    if(!speed_envelope.valid||!speed_envelope.feasible)return;
    double target_sound=planet_atmospheric_speed_of_sound(p,target_altitude);
    double target_speed=compatibility_speed;
    if(!isfinite(target_sound)||target_sound<=DBL_MIN||
       !isfinite(target_speed)||target_speed<=0.0)return;

    /*
     * Size the candidate HAC from the stock-KSP lift polar at the intended
     * MM305 state.  The previous envelope treated the profile reference L/D as
     * maximum lift and produced a ~30 km radius for STS-N at 16.5 km even while
     * the measured/modelled airframe had materially more turn authority.
     */
    double planning_ld=isfinite(aero.lift_to_drag)&&aero.lift_to_drag>0.0?
        aero.lift_to_drag:v->estimated_lift_to_drag;
    double planning_beta=isfinite(aero.ballistic_coefficient)&&
        aero.ballistic_coefficient>DBL_MIN?
        aero.ballistic_coefficient:v->estimated_ballistic_coefficient;
    double target_mach=target_speed/target_sound;
    double lift_aoa=v->maximum_angle_of_attack;
    if(target_mach<1.0&&isfinite(v->terminal_maximum_lift_angle_of_attack)&&
       v->terminal_maximum_lift_angle_of_attack>0.0)
        lift_aoa=fmin(lift_aoa,v->terminal_maximum_lift_angle_of_attack);
    double lift_factor=0.0;
    aerodynamic_force_factors_mach(target_mach,lift_aoa,v,&lift_factor,NULL);
    double target_q=.5*speed_envelope.density_kg_m3*target_speed*target_speed;
    double target_lift=target_q/planning_beta*planning_ld*fabs(lift_factor);
    double target_radius=p->radius+target_altitude;
    double target_gravity=p->gravitational_parameter/(target_radius*target_radius);
    double normal_limit=v->maximum_g_load*target_gravity;
    double target_bank=fmin(fabs(v->maximum_bank_angle),nextafter(90.0,0.0));
    double target_lateral=fmin(target_lift,normal_limit)*
        sin(target_bank*DEG2RAD);
    double modeled_minimum_radius=target_lateral>DBL_MIN?
        target_speed*target_speed/target_lateral:INFINITY;
    if(!isfinite(modeled_minimum_radius)||!(modeled_minimum_radius>0.0))
        modeled_minimum_radius=speed_envelope.minimum_turn_radius_m;

    /*
     * The configured HAC radius is a nominal design radius, not a substitute
     * for force-based authority.  Ensure the planned terminal radius is at least
     * the modeled minimum turn radius physically executable by the vehicle.
     */
    double terminal_radius=fmax(s->hac_radius,modeled_minimum_radius);
    if(!isfinite(terminal_radius)||!(terminal_radius>0.0))return;
    double target_along=NAN,target_cross=NAN,target_course=NAN,target_side=NAN;
    if(!entry_taem_hac_inlet_target(g,t,p,cfg,reference_course,
            terminal_radius,&target_along,&target_cross,&target_course,
            &target_side))return;
    (void)target_side;

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
    double mm305_max_speed=speed_envelope.maximum_speed_mps;
    bool energy_qualified=arrival.valid&&
        arrival.terminal_speed_mps>=mm305_min_speed&&
        arrival.terminal_speed_mps<=mm305_max_speed;
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
                "MM304 target accept: inletV %.1f inside MM305 [%.1f, %.1f], guidanceV %.1f structural [%.1f, %.1f], energyPath %.0f proofPath %.0f Rmin %.0f dragWork %.0f travel %.1f.\n",
                arrival.terminal_speed_mps,mm305_min_speed,mm305_max_speed,target_speed,
                speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                energy_path_to_inlet,approach_path,minimum_turn_radius,
                arrival.modeled_drag_work,arrival_time);
        else
            fprintf(stderr,
                "MM304 geometry target: inletV %.1f outside MM305 [%.1f, %.1f], guidanceV %.1f structural [%.1f, %.1f], energyPath %.0f proofPath %.0f Rmin %.0f ETA %.1f.\n",
                arrival.valid?arrival.terminal_speed_mps:NAN,mm305_min_speed,mm305_max_speed,target_speed,
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
    /* A provisional inlet intentionally suppresses the expensive propagated
       topology search.  The instant its unpowered arrival prediction enters
       the actual MM305 Mach/structural window is therefore a planning event: if
       we do not re-arm the worker here, the last provisional no-topology result
       can leave MM304 flying the fixed inlet with no route to it. */
    if(next.energy_qualified&&!g->entry_topology.valid)
        g->entry_planning_needed=true;
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
    double bank_limit=fmin(45.0,dynamic_bank_limit(t,&cfg->vehicle));
    return taem_course_rate_bank_command(t->horizontal_speed,live_lift_accel(t,aero,&cfg->vehicle),
        t->bank_effectiveness>0?t->bank_effectiveness:1.0,bank_limit,
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
    TaemInterfaceCapture out={0};
    if(!g||!t||!p||!cfg)return out;
    (void)aero;
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    const TaemInterfaceTarget*q=&g->taem_interface_target;

    /*
     * MM304 -> MM305 is the TAEM acquisition boundary, not HAC capture.  The
     * published q remains mandatory as a valid downstream target, but ownership
     * no longer requires the vehicle to be inside q's 1 km p0 tube or already
     * tangent to the HAC.  MM305 PATH_ACQUISITION owns that convergence.
     */
    if(!entry_taem_tangent_target_geometry(q,cfg)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(t->true_air_speed)||!isfinite(t->horizontal_speed)||
       !isfinite(t->vertical_speed)||!isfinite(t->mean_altitude)||
       !isfinite(course)||!isfinite(t->dynamic_pressure)||!isfinite(t->g_force)||
       (t->stall_fraction_is_measured&&!isfinite(t->stall_fraction)))
        return out;

    ControlAuthorityEnvelope control=decision_control_authority_envelope(g,t,p,cfg);
    if(!control.valid||!control.controllable||
       !(control.maximum_lateral_accel_mps2>DBL_MIN)||
       !(t->horizontal_speed>DBL_MIN))
        return out;

    double qh=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double da=q->along_track-t->runway_along_track;
    double dc=q->cross_track-t->runway_cross_track;
    out.along=da*cos(qh)+dc*sin(qh);
    out.cross=-da*sin(qh)+dc*cos(qh);
    out.course_error=norm_signed_deg(q->course-course);
    out.altitude_error=t->mean_altitude-s->taem_interface_altitude;

    double minimum_turn_radius=t->horizontal_speed*t->horizontal_speed/
        control.maximum_lateral_accel_mps2;
    double path_length=decision_target_path_length(g,t,course,
        q->along_track,q->cross_track,q->course,
        control.maximum_lateral_accel_mps2,p,cfg,NULL);
    out.minimum_turn_radius_m=minimum_turn_radius;
    out.path_length_m=isfinite(path_length)?path_length:hypot(da,dc);
    out.required_time_s=isfinite(out.path_length_m)?
        control.control_response_time_s+out.path_length_m/t->horizontal_speed:NAN;
    out.available_time_s=isfinite(q->arrival_ut)?q->arrival_ut-t->ut:NAN;
    out.turn_margin=isfinite(out.required_time_s)&&isfinite(out.available_time_s)?
        (out.available_time_s-out.required_time_s)*t->horizontal_speed:NAN;

    double final_altitude=cfg->site.altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);
    double final_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);
    out.energy_margin=entry_remaining_specific_energy(t->latitude,
        t->mean_altitude,t->true_air_speed,cfg->site.latitude,
        final_altitude,final_speed,p);
    out.energy_available=out.energy_margin;

    out.valid=isfinite(out.along)&&isfinite(out.cross)&&
        isfinite(out.minimum_turn_radius_m)&&isfinite(out.energy_margin);
    if(!out.valid)return out;

    /* NASA Shuttle TAEM begins as an acquisition region: enter while the orbiter
       is still energetic, within the terminal-area range, descending, and moving
       toward the landing site.  No arbitrary heading target is imposed here. */
    double acquisition_range=hypot(t->runway_along_track,t->runway_cross_track);
    double range_limit=entry_taem_range_target(p,s);
    if(!isfinite(acquisition_range)||!isfinite(range_limit)||
       acquisition_range>range_limit)
        out.veto|=2u;

    double runway=cfg->site.runway_heading*DEG2RAD;
    double de=-t->runway_along_track*sin(runway)-
        t->runway_cross_track*cos(runway);
    double dn=-t->runway_along_track*cos(runway)+
        t->runway_cross_track*sin(runway);
    double site_course=norm_deg(atan2(de,dn)*RAD2DEG);
    double closing_error=norm_signed_deg(site_course-course);
    if(!isfinite(closing_error)||cos(closing_error*DEG2RAD)<=0.0)
        out.veto|=4u;

    if(t->true_air_speed<v->minimum_safe_speed)out.veto|=1u;
    double sound=planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double live_mach=isfinite(t->mach)&&t->mach>0.0?t->mach:
        (isfinite(sound)&&sound>DBL_MIN?t->true_air_speed/sound:NAN);
    double mach_min=s->mm305_target_mach-s->mm305_mach_half_width;
    double mach_max=s->mm305_target_mach+s->mm305_mach_half_width;
    if(!isfinite(live_mach)||live_mach<mach_min||live_mach>mach_max)
        out.veto|=1u;
    if(t->mean_altitude<s->mm305_min_altitude||
       t->mean_altitude>s->mm305_max_altitude)
        out.veto|=8u;
    if(!(t->vertical_speed<0.0))out.veto|=32u;
    if(!(out.energy_margin>0.0))out.veto|=16u;
    if(t->dynamic_pressure>v->maximum_dynamic_pressure||
       t->g_force>v->maximum_g_load||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0))
        out.veto|=64u;

    out.ready=out.veto==0u;
    return out;
}

