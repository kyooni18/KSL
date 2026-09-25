static EntryBankAllocationSample entry_program_bank_allocation_sample(
        const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        const LandingConfiguration*cfg,const TaemInterfaceTarget*q,
        double course,double available_time,double lift,double drag,
        double effectiveness,double max_effective,double required_vertical,
        double available_energy,double bank){
    EntryBankAllocationSample out={0};
    double effective=fmin(bank*effectiveness,max_effective);
    double lateral=lift*fabs(sin(effective*DEG2RAD));
    TargetCaptureEnvelope capture=
        decision_target_capture_envelope_with_lateral_accel(
            g,t,course,q->along_track,q->cross_track,q->course,
            available_time,lateral,p,cfg);
    if(!capture.valid)return out;

    DecisionMargin vertical=decision_margin(
        lift*cos(effective*DEG2RAD),required_vertical);
    if(!vertical.valid)return out;

    double path_length=fmax(0.0,capture.path_length_m);
    EnergyPathEnvelope energy=decision_energy_path_envelope(
        t,p,available_energy,drag*path_length,path_length);
    if(!energy.valid||!energy.energy.valid)return out;

    out.valid=true;
    out.vertical_score=vertical.normalized_margin;
    out.lateral_score=fmin(capture.capture_time.normalized_margin,
        energy.energy.normalized_margin);
    out.capture_time_s=capture.required_time_s;
    return out;
}

/*
 * Allocate bank between MM304 vertical delivery and lateral fixed-point capture.
 *
 * The decision is a maximin balance between vertical lift margin, bounded-
 * curvature capture-time margin and unpowered energy-path margin.  The two
 * margin families are monotonic in opposite directions as bank grows, so the
 * optimum is their crossing; solve that crossing by bisection instead of
 * sampling a hand-tuned bank table.
 */
static void entry_program_apply_geometry_bank_demand(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double geometry_bank,EntryControlPlan*plan){
    if(!g||!t||!p||!cfg||!plan||!plan->valid||
       !g->taem_interface_target.valid)return;

    const VehicleProfile*v=&cfg->vehicle;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.controllable)return;

    double dynamic_limit=dynamic_bank_limit(t,v);
    double demand=clampd(fabs(geometry_bank),0.0,dynamic_limit);
    double vertical_ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
    if(!isfinite(vertical_ceiling))vertical_ceiling=dynamic_limit;
    vertical_ceiling=clampd(vertical_ceiling,0.0,dynamic_limit);
    /* A zero vertical ceiling means the current flight condition cannot support
       additional bank while preserving the predicted descent corridor. Do not
       turn a geometric demand into permission to violate that lift constraint;
       the measured envelope is reevaluated as q and available lift grow. */
    /* The vertical envelope is an authority limit, not just an input to the
       later maximin score. Letting the geometry demand remain above it lets
       the high-bank sample win the allocation even though its vertical lift
       margin is already negative; the next tick then has to erase the turn
       after the vehicle has spent the altitude reserve. Bound the demand
       before solving the lateral/vertical allocation so the command pipeline
       preserves the same contract it reports as Bvert. */
    demand=fmin(demand,vertical_ceiling);
    double current=fmin(fabs(plan->target_bank),demand);
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,demand);
    if(demand<=current+resolution)return;

    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!(t->horizontal_speed>DBL_MIN))return;

    double direct_range=hypot(q->along_track-t->runway_along_track,
        q->cross_track-t->runway_cross_track);
    double available_time=NAN;
    if(isfinite(q->arrival_ut)&&q->arrival_ut>t->ut)
        available_time=q->arrival_ut-t->ut;
    else if(isfinite(q->remaining_path)&&q->remaining_path>=0.0)
        available_time=q->remaining_path/t->horizontal_speed;
    else if(isfinite(direct_range))
        available_time=authority.control_response_time_s+
            direct_range/t->horizontal_speed;
    if(!isfinite(available_time))return;
    if(entry_program_terminal_geometry_blocked(g,t,cfg))
        available_time=0.0;

    double lift=live_lift_accel(t,aero,v);
    if(!(lift>DBL_MIN)||!isfinite(lift))return;
    double drag=fmax(0.0,live_drag_accel(t,aero,v));
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double max_effective=nextafter(90.0,0.0);
    double required_effective=fmin(vertical_ceiling*effectiveness,max_effective);
    double required_vertical=lift*cos(required_effective*DEG2RAD);

    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,q->altitude,q->speed,p);
    if(!isfinite(available_energy))return;

    double low=fmax(current,resolution),high=demand;
    EntryBankAllocationSample high_sample=entry_program_bank_allocation_sample(
        g,t,p,cfg,q,course,available_time,lift,drag,effectiveness,max_effective,
        required_vertical,available_energy,high);
    if(!high_sample.valid){
        /* Keep a physically bounded lateral demand when the shadow capture
           envelope is temporarily unavailable. Returning zero here turns a
           real station error into wings-level flight and makes the station
           recede faster than the MM305 range gate can recover. */
        double fallback_bank=fmin(demand,vertical_ceiling);
        if(fallback_bank>current+resolution){
            double sign=g->s_turn_sign>=0.0?1.0:-1.0;
            plan->target_bank=sign*fallback_bank;
            plan->bank_cap=fmax(plan->bank_cap,fallback_bank);
            plan->target_turn_radius=live_turn_radius(t,aero,v,plan->target_bank);
        }
        return;
    }

    EntryBankAllocationSample low_sample=entry_program_bank_allocation_sample(
        g,t,p,cfg,q,course,available_time,lift,drag,effectiveness,max_effective,
        required_vertical,available_energy,low);

    double best_bank=high;
    EntryBankAllocationSample best=high_sample;
    if(low_sample.valid&&low_sample.vertical_score<=low_sample.lateral_score){
        best_bank=low;best=low_sample;
    }else if(high_sample.vertical_score<high_sample.lateral_score){
        unsigned guard=0;
        while(high-low>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
            double mid=.5*(low+high); /* decision-literal-ok: bisection midpoint */
            EntryBankAllocationSample sample=entry_program_bank_allocation_sample(
                g,t,p,cfg,q,course,available_time,lift,drag,effectiveness,max_effective,
                required_vertical,available_energy,mid);
            if(!sample.valid||sample.vertical_score>sample.lateral_score){
                low=mid;low_sample=sample;
            }else{
                high=mid;high_sample=sample;
            }
        }
        double low_score=low_sample.valid?
            fmin(low_sample.vertical_score,low_sample.lateral_score):-INFINITY;
        double high_score=fmin(high_sample.vertical_score,high_sample.lateral_score);
        if(low_score>high_score){
            best_bank=low;best=low_sample;
        }else{
            best_bank=high;best=high_sample;
        }
    }
    (void)best;
    if(best_bank<=current+resolution)return;
    double sign=geometry_bank>=0.0?1.0:-1.0;
    plan->target_bank=sign*best_bank;
    plan->bank_cap=fmax(plan->bank_cap,best_bank);
    plan->target_turn_radius=live_turn_radius(t,aero,v,plan->target_bank);
}


/* MM304 executes a propagated S-turn program. Reversal timing is selected by
   trajectory propagation. Measured leg capture proves that the commanded side
   actually became an executable leg; no second altitude/corridor/dwell policy
   is allowed to move the propagated event. */
static bool entry_program_planned_reversal_due(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const VehicleProfile*v,const GuidanceSettings*s){
    (void)p;(void)v;(void)s;
    if(!g||!t||!g->entry_reversal_scheduled||!g->has_s_turn_leg_started||
       g->has_s_turn_reversal_requested||!isfinite(g->entry_reversal_ut))return false;
    double time_resolution=sqrt(DBL_EPSILON)*fmax(1.0,fabs(t->ut));
    return t->ut+time_resolution>=g->entry_reversal_ut;
}

/*
 * The propagated first-leg timestamp is advisory.  It is deliberately durable
 * against ordinary replans, but a live setup-side crossrange proof may mature
 * before that timestamp when the measured atmosphere differs from the shadow.
 * Release only a nonfinal, genuine side change after the configured leg dwell;
 * the final fixed-point/perpendicular turn still has its separate physical
 * reachability gate in entry_program_execute_planned_reversal().
 */
static bool entry_program_terminal_course_endpoint_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign,
        double target_course,double*endpoint_error_out,double*radius_out){
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(radius_out)*radius_out=INFINITY;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(target_course))return false;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return false;

    double side=final_sign>=0.0?1.0:-1.0;
    double tangent_turn=norm_signed_deg(target_course-course);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(side*tangent_turn<=angular_resolution)return false;

    double usable_bank=dynamic_bank_limit(t,&cfg->vehicle);
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double measured_lift=t->mass>DBL_MIN&&isfinite(t->lift_force)&&
        t->lift_force>0.0?t->lift_force/t->mass:NAN;
    if(!(usable_bank>0.0)||!isfinite(measured_lift)||
       !(measured_lift>DBL_MIN))return false;

    double effective_bank=fmin(usable_bank*effectiveness,
        nextafter(90.0,0.0))*DEG2RAD;
    double lateral=measured_lift*fabs(sin(effective_bank));
    if(!(lateral>DBL_MIN))return false;
    double minimum_radius=t->horizontal_speed*t->horizontal_speed/lateral;
    if(!isfinite(minimum_radius)||!(minimum_radius>0.0))return false;

    double psi0=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    double psi1=norm_signed_deg(target_course-cfg->site.runway_heading)*DEG2RAD;
    double ua=(sin(psi1)-sin(psi0))/side;
    double uc=(cos(psi0)-cos(psi1))/side;
    double norm2=ua*ua+uc*uc;
    if(!isfinite(norm2)||norm2<=DBL_EPSILON)return false;

    double ba=q->along_track-t->runway_along_track;
    double bc=q->cross_track-t->runway_cross_track;
    double ideal_radius=(ba*ua+bc*uc)/norm2;
    if(!isfinite(ideal_radius)||ideal_radius<minimum_radius)return false;

    double endpoint_along=t->runway_along_track+ideal_radius*ua;
    double endpoint_cross=t->runway_cross_track+ideal_radius*uc;
    double endpoint_error=hypot(endpoint_along-q->along_track,
        endpoint_cross-q->cross_track);

    /*
     * Admission tolerance is the distance that can be accumulated while the
     * roll actuator establishes the requested bank, plus the measured
     * trajectory-range residual.  There is no radius-percentage or fixed-metre
     * capture tube here.
     */
    double target_bank=side*usable_bank;
    double response_time=roll_capture_time(t,target_bank,
        cfg->guidance.entry_roll_rate,cfg->guidance.entry_roll_acceleration);
    if(!isfinite(response_time))return false;
    double model_residual=isfinite(t->trajectory_range_residual)?
        fabs(t->trajectory_range_residual):0.0;
    double tolerance=t->horizontal_speed*response_time+model_residual;

    if(endpoint_error_out)*endpoint_error_out=endpoint_error;
    if(radius_out)*radius_out=ideal_radius;
    return endpoint_error<=tolerance;
}

/* A bank sign change is not instantaneous.  The final-arc admission point must
   therefore lead the geometric circle by the distance/course accumulated while the
   orbiter rolls through zero and establishes useful opposite bank. v27 reached a
   good ~84 km endpoint circle at ~17 km crossrange, but waiting for instantaneous
   readiness let the +50 -> - bank transition carry it to ~25 km crossrange before
   the final turn had authority. Project that actuator transient in runway coordinates
   and admit the reversal when the post-roll state, rather than the current state,
   closes the fixed MM304 endpoint. */
static bool entry_program_rollthrough_endpoint_ready(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double final_sign,
        double*endpoint_error_out,double*lead_time_out){
    (void)p;
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(lead_time_out)*lead_time_out=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    double lift=t->mass>DBL_MIN&&isfinite(t->lift_force)&&t->lift_force>0.0?
        t->lift_force/t->mass:NAN;
    double bank_limit=dynamic_bank_limit(t,&cfg->vehicle);
    double side=final_sign>=0.0?1.0:-1.0;
    EntryLateralTurnInput turn_input={
        .horizontal_speed_mps=t->horizontal_speed,
        .current_course_deg=norm_signed_deg(course-cfg->site.runway_heading),
        .target_course_deg=norm_signed_deg(q->course-cfg->site.runway_heading),
        .current_along_m=t->runway_along_track,
        .current_cross_m=t->runway_cross_track,
        .target_along_m=q->along_track,
        .target_cross_m=q->cross_track,
        .measured_bank_deg=norm_signed_deg(t->roll),
        .measured_bank_rate_deg_s=controlled_roll_rate(t),
        .lift_accel_mps2=lift,
        .bank_effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
            t->bank_effectiveness:1.0,
        .maximum_bank_deg=bank_limit,
        .maximum_roll_rate_deg_s=fabs(cfg->guidance.entry_roll_rate),
        .maximum_roll_accel_deg_s2=fabs(cfg->guidance.entry_roll_acceleration),
        .capture_radius_m=fmax(cfg->guidance.mm304_handoff_radius,
            cfg->guidance.hac_radius*0.10),
        .position_uncertainty_m=isfinite(t->trajectory_range_residual)?
            fabs(t->trajectory_range_residual):0.0
    };
    EntryLateralTurnEnvelope turn=
        entry_lateral_terminal_turn_envelope(&turn_input);
    if(!turn.valid||turn.turn_sign*side<=0.0)return false;
    if(endpoint_error_out)*endpoint_error_out=turn.endpoint_error_m;
    if(lead_time_out)*lead_time_out=turn.response_time_s;
    return turn.feasibility_margin_m>=0.0;
}

static bool entry_program_final_reversal_geometry_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign){
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    /* Proximity alone never proves room for a final turn. */
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return false;
    double side=final_sign>=0.0?1.0:-1.0;
    double tangent_turn=norm_signed_deg(q->course-course);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(side*tangent_turn<=angular_resolution)return false;

    /* Forecast timestamps and replay-state proximity are hints, never authority.
       The final reversal is legal only when the measured state, measured lift and
       current actuator envelope can close a single monotonic arc onto the fixed
       MM304 point/course contract. */
    return entry_program_terminal_course_endpoint_ready(g,t,cfg,final_sign,q->course,NULL,NULL);
}


static bool entry_program_terminal_geometry_blocked(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg){
    if(!g||!t||!cfg||g->entry_final_reversal_pending||g->entry_final_reversal_completed||
       !g->entry_reversal_scheduled||!isfinite(g->entry_reversal_ut)||
       t->ut+1e-6<g->entry_reversal_ut||!g->taem_interface_target.valid)return false;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!isfinite(g->taem_interface_target.course))return false;
    double next_sign=g->entry_reversal_sign>=0.0?1.0:-1.0;
    double terminal_turn=norm_signed_deg(g->taem_interface_target.course-course);
    double resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    bool terminal_side=next_sign*terminal_turn>resolution;
    return terminal_side&&!entry_program_final_reversal_geometry_ready(g,t,cfg,next_sign);
}


/* Use one chord-aware setup radius everywhere MM304 reasons about the first S-turn
   reversal. The former pose target always used the nominal TAEM/HAC radius (~25 km
   in the current 640 m/s case), while the reversal gate correctly expanded the
   required radius toward ~96 km when the shuttle was still far upstream. That
   mismatch made guidance aim for only ~7 km crossrange, declare the timer overdue,
   then demand 50-70 deg bank late to satisfy a ~25 km crossrange gate. */
static double entry_taem_shaping_setup_radius(const TaemInterfaceTarget*q,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!q||!t||!p||!cfg||!q->valid||!isfinite(q->course)||
       !isfinite(q->along_track)||!isfinite(t->runway_along_track))return NAN;
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(relative)||fabs(relative)<30.0)return NAN;
    double offset=fabs(relative)*DEG2RAD;
    double reference_speed=isfinite(q->speed)&&q->speed>0.0?q->speed:
        fmax(220.0,cfg->guidance.taem_force_handoff_speed*.65);
    double course_rate=fmax(1.0,hac_course_rate_cap_for_speed(reference_speed))*DEG2RAD;
    double nominal_radius=reference_speed/fmax(course_rate,1e-6);
    double nominal_max=fmax(cfg->guidance.hac_radius*4.0,
        cfg->guidance.taem_interface_range*1.5);
    nominal_radius=clampd(nominal_radius,cfg->guidance.hac_radius,nominal_max);
    double upstream=fmax(0.0,q->along_track-t->runway_along_track);
    double exact_radius=upstream/fmax(.10,sin(offset));
    double maximum_radius=fmax(cfg->guidance.hac_radius,
        fmin(p->radius*.20,fmax(cfg->guidance.hac_radius*8.0,
            cfg->guidance.taem_interface_range*2.5)));
    return clampd(exact_radius,nominal_radius,maximum_radius);
}
/* The first sign change only has to establish a real setup-side offset before
   MM304 begins the long, decelerating terminal turn. Live finalaoa7 showed that
   a ~6.6 km proof was not itself the failure: the controller then stopped the
   turn and flew runway-parallel for most of the remaining descent. Keep the
   first proof HAC-sized so the turn starts early; post-shaping guidance below
   now continues a bounded tangent-line lead instead of freezing the heading. */
static double entry_taem_shaping_parallel_cross_requirement(const LandingConfiguration*cfg){
    if(!cfg)return NAN;
    double required=fmax(6000.0,fmin(12000.0,cfg->guidance.hac_radius*.55));
    return isfinite(required)?required:NAN;
}
bool entry_taem_shaping_crossrange_ready(const TaemInterfaceTarget*q,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double next_sign,
        double*built_out,double*required_out){
    if(built_out)*built_out=NAN;if(required_out)*required_out=NAN;
    if(!q||!t||!p||!cfg||!q->valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return true;
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(relative)||fabs(relative)<30.0)return true;
    double final_side=relative>=0.0?1.0:-1.0;
    if(next_sign*final_side<=0.0)return true;
    double required=entry_taem_shaping_parallel_cross_requirement(cfg);
    if(!isfinite(required))return true;
    double setup_side=-final_side;
    double built=setup_side*(t->runway_cross_track-q->cross_track);
    if(built_out)*built_out=built;if(required_out)*required_out=required;
    return isfinite(built)&&built>=required;
}

/* The propagated topology normally schedules the first MM304 sign change, but
   topology certification depends on an energy-qualified inlet. A geometry-only
   inlet can therefore create a deadlock: the live setup leg keeps building
   crossrange while the worker waits for an energy state that the unreversed leg
   itself is consuming. Once the measured setup-side crossrange proof is complete,
   schedule the opposite sign directly from that same physical contract. This is
   not a time/altitude shortcut: the configured leg dwell, measured crossrange,
   fixed TAEM point/course, and live bank envelope all have to be executable now. */
/* KSP_LANDER_SETUP_REVERSAL_TRACE=1 prints why the live setup reversal was
   refused, including inside planner shadow witnesses (tagged shadow=1).
   Lines are emitted only when the refusal reason changes per thread. */
static bool entry_program_schedule_live_setup_reversal(GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg,
        double reversal_bank_magnitude){
    if(!g||!t||!p||!cfg)return false;
    if(g->entry_reversal_scheduled&&
       (g->entry_reversal_is_final||!(g->entry_reversal_ut>t->ut+1e-6)))
        return false;
    if(g->entry_control_reversals!=0||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed||
       !g->has_s_turn_leg_started||!isfinite(g->s_turn_leg_started_ut)||
       !g->taem_interface_target.valid)
        return false;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(relative)||fabs(relative)<30.0)return false;
    double final_side=relative>=0.0?1.0:-1.0;
    double owned_side=g->s_turn_sign>=0.0?1.0:-1.0;
    if(owned_side*final_side>=0.0)return false;

    double minimum_dwell=fmax(cfg->guidance.s_turn_minimum_leg_duration,8.0);
    if(t->ut-g->s_turn_leg_started_ut<minimum_dwell)return false;

    double built=NAN,required=NAN;
    if(!entry_taem_shaping_crossrange_ready(q,t,p,cfg,final_side,&built,&required))
        return false;
    double distance=hypot(q->along_track-t->runway_along_track,
        q->cross_track-t->runway_cross_track);
    TaemHandoffContract handoff=taem_handoff_contract(&cfg->guidance);
    if(!isfinite(distance)||distance<=handoff.horizontal_radius_m)return false;

    double dynamic_limit=dynamic_bank_limit(t,&cfg->vehicle);
    if(!isfinite(dynamic_limit)||dynamic_limit<=0.0)return false;
    double bank=fmin(dynamic_limit,fmax(4.0,fabs(reversal_bank_magnitude)));

    EntryControlPlan event=g->entry_s_turn_plan;
    event.valid=true;
    event.planned_ut=t->ut;
    event.segment_duration=fmax(2.0,cfg->guidance.prediction_interval*2.0);
    event.target_bank=final_side*bank;
    if(!isfinite(event.target_aoa)||event.target_aoa<=0.0)
        event.target_aoa=fmax(g->entry_control_aoa,
            entry_thermal_protection_aoa_floor(&cfg->vehicle));
    event.target_heading=q->course;
    event.has_planned_reversal=true;
    event.planned_reversal_is_final=false;
    event.final_heading_lock=false;
    event.planned_reversal_ut=t->ut;
    event.planned_reversal_range=distance;
    event.planned_reversal_sign=final_side;
    event.predicted_reversals=fmax(event.predicted_reversals,1u);
    entry_program_commit_planned_reversal(g,&event,t->ut,false);
    if(!g->entry_reversal_scheduled||
       g->entry_reversal_is_final||
       g->entry_reversal_sign*final_side<=0.0||
       fabs(g->entry_reversal_ut-t->ut)>=1e-6)
        return false;

    g->entry_s_turn_plan=event;
    if(!g->diagnostic_shadow)fprintf(stderr,
        "MM304 live setup reversal scheduled: UT %.2f built %.0f/%.0f m distance %.0f m sign %+.0f bank %.1f.\n",
        t->ut,built,required,distance,final_side,bank);
    return true;
}

static bool entry_program_supervised_reversal_ready(const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s){
    if(!g||!t||!s||!g->entry_s_turn_plan.valid||!g->entry_s_turn_plan.terminal_ready||
       !g->entry_supervision_valid||g->entry_supervision_boundary_missed||
       !isfinite(g->entry_supervision_ut))return false;
    double age=t->ut-g->entry_supervision_ut;
    double max_age=fmax(12.0,6.0*fmax(.25,s->prediction_interval));
    return isfinite(age)&&age>=0.0&&age<=max_age;
}

static bool entry_program_execute_planned_reversal(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg)return false;
    const GuidanceSettings*s=&cfg->guidance;
    if(!entry_program_planned_reversal_due(g,t,p,&cfg->vehicle,s))return false;

    double next_sign=g->entry_reversal_sign>=0.0?1.0:-1.0;
    double old_sign=g->s_turn_sign>=0.0?1.0:-1.0;
    bool genuine_side_change=next_sign*old_sign<0.0;
    bool planned_final=g->entry_reversal_is_final;

    double live_course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    double target_turn=g->taem_interface_target.valid&&
        isfinite(g->taem_interface_target.course)&&isfinite(live_course)?
        norm_signed_deg(g->taem_interface_target.course-live_course):NAN;
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    double required_terminal_side=
        isfinite(target_turn)&&fabs(target_turn)>angular_resolution?
        (target_turn>=0.0?1.0:-1.0):0.0;
    bool terminal_direction=required_terminal_side!=0.0&&
        next_sign*required_terminal_side>0.0;

    double rollout_error=INFINITY,rollout_lead=NAN;
    bool rollout_ready=terminal_direction&&
        entry_program_rollthrough_endpoint_ready(g,t,p,cfg,next_sign,
            &rollout_error,&rollout_lead);
    bool geometry_ready=terminal_direction&&
        entry_program_final_reversal_geometry_ready(g,t,cfg,next_sign);
    bool supervised_ready=terminal_direction&&
        entry_program_supervised_reversal_ready(g,t,s);
    bool terminal_feasible=rollout_ready||geometry_ready||supervised_ready;

    /* A topology with no terminal-turn timestamp is the new acquisition-only
       MM304 route: its scheduled event is strictly the S-turn shaping reversal.
       Do not promote that reversal into the deleted MM304-owned HAC tangent turn. */
    bool acquisition_topology=g->entry_topology.valid&&
        !isfinite(g->entry_topology.terminal_turn_ut);
    bool terminal_side_event=!acquisition_topology&&!planned_final&&terminal_feasible;
    if(planned_final&&!g->diagnostic_shadow&&!terminal_feasible)return false;

    /* A same-side event is meaningful only for the legacy terminal-event path.
       Acquisition-only topology must execute a genuine bank-side reversal. */
    if(!planned_final&&!genuine_side_change&&!terminal_side_event)return false;

    if(!g->diagnostic_shadow&&(planned_final||terminal_side_event)&&terminal_feasible){
        fprintf(stderr,
            "MM304 terminal turn admitted by reachability: UT %.2f sign %+.0f endpoint=%d response=%d lead %.2fs error %.0f m shadow=%d.\n",
            t->ut,next_sign,geometry_ready?1:0,rollout_ready?1:0,
            rollout_lead,rollout_error,supervised_ready?1:0);
    }

    bool final_reversal=planned_final||terminal_side_event;
    EntryControlPlan previous_plan=g->entry_s_turn_plan;
    double reversal_bank=fmax(fabs(g->entry_control_bank),g->entry_reversal_bank);
    request_side(g,next_sign,t->ut);
    if(isfinite(reversal_bank)&&reversal_bank>DBL_EPSILON)
        g->entry_control_bank=next_sign*reversal_bank;
    if(final_reversal){
        /*
         * Terminal-turn ownership persists through roll/capture. The retained
         * child carries lineage and commands only; the fixed MM304 handoff
         * contract still decides when MM305 may take ownership.
         */
        g->entry_final_reversal_pending=true;
        g->entry_final_reversal_completed=false;
        g->entry_topology_heading_locked=false;
        EntryControlPlan capture_plan=previous_plan;
        capture_plan.valid=true;
        capture_plan.plan_id=0;capture_plan.plan_version=0;
        capture_plan.parent_plan_id=0;capture_plan.parent_plan_version=0;
        capture_plan.planned_ut=t->ut;
        capture_plan.segment_duration=fmax(2.0,s->prediction_interval*2.0);
        capture_plan.target_bank=g->entry_control_bank;
        if(!isfinite(capture_plan.target_aoa)||capture_plan.target_aoa<=0.0)
            capture_plan.target_aoa=fmax(g->entry_control_aoa,
                entry_thermal_protection_aoa_floor(&cfg->vehicle));
        capture_plan.target_heading=g->taem_interface_target.valid&&
            isfinite(g->taem_interface_target.course)?
            g->taem_interface_target.course:
            (isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading);
        capture_plan.has_planned_reversal=false;
        capture_plan.planned_reversal_is_final=false;
        capture_plan.planned_reversal_ut=NAN;capture_plan.planned_reversal_range=NAN;
        capture_plan.planned_reversal_sign=0.0;
        capture_plan.final_heading_lock=true;
        control_plan_assign_lineage(g,&capture_plan,previous_plan.plan_id?&previous_plan:NULL);
        g->entry_s_turn_plan=capture_plan;
    }else{
        g->entry_s_turn_plan.valid=false;
    }
    g->entry_control_segment_until_ut=t->ut;
    g->entry_reversal_scheduled=false;
    g->entry_reversal_is_final=false;
    if(genuine_side_change){
        g->entry_control_reversals++;
        g->entry_continuation_bootstrap=false;
    }
    return true;
}


/* Safety-envelope replans may resize the current bank/AoA command, but they
   are not evidence that an already accepted reversal deadline became unsafe.
   Keep anti-procrastination active for those replans: normal commit logic may
   still pull the same-side reversal earlier. Only an invalid supervisory
   trajectory may discard the committed milestone and install a later one. */
/* Routine model checkpoints may resize bank/AoA, but they cannot silently postpone
   an already accepted reversal. A materially infeasible-plan replan may replace it. */
       void entry_program_commit_planned_reversal(GuidanceMachine*g,EntryControlPlan*plan,
        double ut,bool replace_existing){
    if(!g||!plan)return;
    bool executable_leg=g->has_s_turn_leg_started||
        (isfinite(plan->target_bank)&&fabs(plan->target_bank)>=4.0);
    bool candidate=plan->valid&&plan->has_planned_reversal&&executable_leg&&
        isfinite(plan->planned_reversal_ut)&&plan->planned_reversal_ut>=ut-1e-6&&
        fabs(plan->planned_reversal_sign)>.1;
    if(replace_existing){
        g->entry_reversal_scheduled=false;g->entry_reversal_is_final=false;
    }
    if(candidate){
        double sign=plan->planned_reversal_sign>=0?1.0:-1.0;
        double event=fmax(ut,plan->planned_reversal_ut);
        bool accept=!g->entry_reversal_scheduled||
            (g->entry_reversal_sign*sign>=0.0&&event<g->entry_reversal_ut-1e-6&&
             (!g->entry_reversal_is_final||plan->planned_reversal_is_final));
        if(accept){
            g->entry_reversal_scheduled=true;
            g->entry_reversal_is_final=plan->planned_reversal_is_final;
            g->entry_reversal_ut=event;
            g->entry_reversal_range=plan->planned_reversal_range;
            g->entry_reversal_sign=sign;
            g->entry_reversal_bank=fabs(plan->target_bank);
        }
    }
    /* The persistent plan is the executable contract exposed to logging and
       predictor replay.  If anti-procrastination retained an older deadline,
       overwrite the rejected proposal's event metadata with that effective
       durable event; candidate trace still preserves the raw proposal. */
    plan->has_planned_reversal=g->entry_reversal_scheduled;
    plan->planned_reversal_is_final=g->entry_reversal_scheduled&&g->entry_reversal_is_final;
    if(g->entry_reversal_scheduled){
        plan->planned_reversal_ut=g->entry_reversal_ut;
        plan->planned_reversal_range=g->entry_reversal_range;
        plan->planned_reversal_sign=g->entry_reversal_sign;
        if(plan->predicted_reversals<1u)plan->predicted_reversals=1u;
    }else{
        plan->planned_reversal_ut=NAN;plan->planned_reversal_range=NAN;
        plan->planned_reversal_sign=0.0;
    }
}

/* A refreshed advisory target keeps the same fixed inlet geometry while its
 * arrival ETA is recomputed from live speed and authority.  Worker plans must
 * reject a changed inlet or energy qualification, but not a timestamp-only
 * refresh of that same target. */
static bool entry_interface_target_plan_identity_equal(
        const TaemInterfaceTarget*a,const TaemInterfaceTarget*b){
    if(!a||!b||a->valid!=b->valid||a->energy_qualified!=b->energy_qualified)
        return false;
    if(!a->valid)return true;
    return a->along_track==b->along_track&&a->cross_track==b->cross_track&&
        a->course==b->course&&a->altitude==b->altitude&&a->speed==b->speed&&
        a->flight_path_angle==b->flight_path_angle&&a->hac_radius==b->hac_radius;
}

/* Only plans cross the worker boundary. Never restore cloned executive, PID,
   limiter, capture, or reversal-execution state over newer live telemetry. */
bool guidance_accept_entry_plan(GuidanceMachine*g,const GuidanceMachine*request,
        const GuidanceMachine*result,const Telemetry*t,const LandingConfiguration*cfg){
    if(!g||!request||!result||!t||!cfg)return false;
    if(result->phase!=request->phase||
       !g->automation_engaged||g->paused||g->aborted||g->phase!=PHASE_ENTRY_ENERGY||
       g->control_plan_sequence!=request->control_plan_sequence||
       g->entry_control_reversals!=request->entry_control_reversals||
       g->entry_final_reversal_pending!=request->entry_final_reversal_pending||
       g->entry_final_reversal_completed!=request->entry_final_reversal_completed||
       !entry_interface_target_plan_identity_equal(&g->taem_interface_target,
           &request->taem_interface_target))
        return false;

    double age=t->ut-request->previous_ut;
    if(!isfinite(age)||age<0.0)return false;

    /*
     * Async prediction is advisory.  It may update feasibility and predicted
     * terminal-state evidence, but it must never install bank/AoA commands,
     * change s_turn_sign, schedule a reversal, or replace the live MM304 inlet.
     * Every executable command is recomputed from current telemetry in
     * entry_program_guidance().
     */
    if(result->entry_topology.valid)
        (void)guidance_install_entry_topology(g,&result->entry_topology,t->ut);

    g->entry_supervision_ut=request->previous_ut;
    g->entry_supervision_valid=result->entry_supervision_valid;
    g->entry_supervision_mode=result->entry_supervision_mode;
    g->entry_committed_infeasible=result->entry_committed_infeasible;
    g->entry_supervision_boundary_missed=result->entry_supervision_boundary_missed;

    if(result->entry_s_turn_plan.valid){
        g->entry_s_turn_plan.terminal_ready=result->entry_s_turn_plan.terminal_ready;
        g->entry_s_turn_plan.closest_distance=result->entry_s_turn_plan.closest_distance;
        g->entry_s_turn_plan.taem_range_error=result->entry_s_turn_plan.taem_range_error;
        g->entry_s_turn_plan.taem_speed=result->entry_s_turn_plan.taem_speed;
        g->entry_s_turn_plan.taem_energy_error=result->entry_s_turn_plan.taem_energy_error;
        g->entry_s_turn_plan.predicted_reversals=result->entry_s_turn_plan.predicted_reversals;
        g->entry_s_turn_plan.cost=result->entry_s_turn_plan.cost;
    }
    g->entry_planning_needed=false;
    return true;
}

static bool entry_program_first_segment(const GuidanceMachine*g){
    bool initial_entry=g&&(!g->entry_exec.initialized||g->entry_exec.phase==ENTRY_PHASE_PREENTRY);
    return initial_entry&&!g->entry_s_turn_plan.valid&&!g->has_s_turn_leg_started&&
        !g->entry_reversal_scheduled&&g->entry_control_reversals==0;
}

static void entry_program_seed_initial_side(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg){
    if(!entry_program_first_segment(g))return;
    /*
     * The published perpendicular course is the final inlet side, not
     * necessarily the side of the first energy-management leg.  When the
     * vehicle is upstream and its point bearing is closer to the runway
     * tangent than to the required inlet course, a direct turn cannot satisfy
     * both the fixed point and fixed course.  The executable topology therefore
     * starts on the opposite side and must later reverse into the inlet.
     *
     * Keep these two pieces of state separate: entry_target_side is the stable
     * final-course identity used by the target publisher, while s_turn_sign is
     * the currently owned aerodynamic leg.  The old code latched both from a
     * tiny point-bearing residual, which could erase the setup leg when the
     * vehicle was nearly on the runway tangent.
     */
    double sign=1.0;
    double target_side=1.0;
    if(t&&cfg&&g->taem_interface_target.valid&&
       isfinite(t->runway_along_track)&&isfinite(t->runway_cross_track)){
        double course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        double relative_course=norm_signed_deg(course-cfg->site.runway_heading);
        double target_course=norm_signed_deg(
            g->taem_interface_target.course-cfg->site.runway_heading);
        TaemHandoffContract handoff=taem_handoff_contract(&cfg->guidance);
        double target_distance=hypot(
            g->taem_interface_target.along_track-t->runway_along_track,
            g->taem_interface_target.cross_track-t->runway_cross_track);
        double target_course_error=fabs(norm_signed_deg(
            g->taem_interface_target.course-course));
        double final_turn=norm_signed_deg(target_course);
        double final_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        if(fabs(final_turn)>final_resolution)
            target_side=final_turn>=0.0?1.0:-1.0;

        bool inside_handoff=isfinite(target_distance)&&
            target_distance<=handoff.horizontal_radius_m&&
            isfinite(target_course_error)&&
            target_course_error<=handoff.perpendicular_heading_half_width_deg;
        double da=g->taem_interface_target.along_track-t->runway_along_track;
        double dc=g->taem_interface_target.cross_track-t->runway_cross_track;
        double point_course=norm_deg(cfg->site.runway_heading+
            atan2(dc,da)*RAD2DEG);
        double point_turn_signed=norm_signed_deg(point_course-course);
        bool perpendicular=fabs(fabs(final_turn)-90.0)<=
            handoff.perpendicular_heading_half_width_deg;
        bool upstream=isfinite(da)&&da>0.0;
        double setup_side=-target_side;
        double setup_cross=setup_side*
            (t->runway_cross_track-g->taem_interface_target.cross_track);
        double required_cross=entry_taem_shaping_parallel_cross_requirement(cfg);
        /* A perpendicular fixed-pose inlet needs real crossrange on the setup
           side before the final-side turn.  Use the same measured contract that
           later releases the setup reversal; point-bearing sign alone is
           degenerate on the runway tangent and can erase the entire first leg. */
        bool setup_required=!inside_handoff&&perpendicular&&upstream&&
            fabs(target_side)>DBL_EPSILON&&isfinite(setup_cross)&&
            isfinite(required_cross)&&setup_cross<required_cross;
        if(setup_required&&fabs(target_side)>DBL_EPSILON){
            /* A fixed-pose perpendicular inlet needs the opposite setup leg. */
            sign=-target_side;
        }else if(fabs(point_turn_signed)>sqrt(DBL_EPSILON)){
            /* The immediate MM304 objective is the fixed station.  Its signed
             * bearing, rather than the final 180-degree HAC course, determines
             * the first turn direction. */
            sign=point_turn_signed>=0.0?1.0:-1.0;
        }else{
            double geometry_sign=entry_lateral_geometry_turn_sign(
                relative_course,
                t->runway_along_track,t->runway_cross_track,
                g->taem_interface_target.along_track,
                g->taem_interface_target.cross_track,
                target_course);
            if(fabs(geometry_sign)>DBL_EPSILON)sign=geometry_sign;
        }
        if(inside_handoff&&fabs(target_side)>DBL_EPSILON)
            sign=target_side;
    }
    g->s_turn_sign=sign;
    g->entry_target_side=target_side;
    g->entry_target_side_latched=true;
    g->has_s_turn_reversal_requested=false;
}

