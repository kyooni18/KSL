static TaemSTurnProjection taem_s_turn_project_segment(const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        const LandingSite*site,double target_bank,double target_aoa,double duration){
    TaemSTurnProjection out={0};
    if(!t||!p||!v||!s||!site||!isfinite(duration)||duration<=0)return out;
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e=0.0,n=0.0;local_offsets(origin,current,p->radius,&e,&n);
    double bank=norm_signed_deg(t->roll);
    double bank_rate=controlled_roll_rate(t);
    if(!isfinite(bank_rate))bank_rate=0.0;
    double aoa=isfinite(t->angle_of_attack)?t->angle_of_attack:target_aoa;
    double aoa_rate=controlled_aoa_rate(t);
    if(!isfinite(aoa_rate))aoa_rate=0.0;
    double speed=fmax(1.0,t->true_air_speed);
    double altitude=t->mean_altitude;
    double gamma=clampd(t->flight_path_angle,-45.0,20.0)*DEG2RAD;
    double heading=course;
    double bank_eff=clampd(t->bank_effectiveness>0?t->bank_effectiveness:1.0,.35,1.8);
    GuidanceAttitudeLimits roll_limits=stabilized_attitude_limits(
        t,s,PHASE_TAEM,false);
    GuidanceAttitudeLimits pitch_limits=stabilized_attitude_limits(
        t,s,PHASE_TAEM,true);
    if(!(roll_limits.rate_deg_s>0.0)||!(roll_limits.accel_deg_s2>0.0)||
       !(pitch_limits.rate_deg_s>0.0)||!(pitch_limits.accel_deg_s2>0.0))
        return out;
    JerkLimiter roll_limiter={
        .value=norm_deg(bank),.rate=bank_rate,.has_value=true
    };
    JerkLimiter pitch_limiter={
        .value=aoa,.rate=aoa_rate,.has_value=true
    };
    double live_lift=fmax(.02,live_lift_accel(t,aero,v));
    double live_drag=fmax(.02,live_drag_accel(t,aero,v));
    double rho0=planet_atmospheric_density(p,t->mean_altitude);
    double density_scale=t->atmospheric_density>0&&rho0>1e-9?
        clampd(t->atmospheric_density/rho0,.35,2.8):clampd(t->trajectory_density_scale,.35,2.8);
    double sound0=t->speed_of_sound>1?t->speed_of_sound:planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double current_incidence=hypot(t->angle_of_attack,t->sideslip);
    double current_lf=0.0,current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,&current_lf,&current_df);
    double q0=fmax(1.0,t->dynamic_pressure);
    double peak_q=fmax(0.0,t->dynamic_pressure);
    double peak_load=fmax(0.0,t->g_force);
    double minimum_speed=speed;
    double minimum_altitude=altitude;
    double minimum_fpa=gamma*RAD2DEG;
    int steps=(int)ceil(duration/.20);if(steps<1)steps=1;if(steps>320)steps=320;
    double step=duration/(double)steps;
    for(int i=0;i<steps;i++){
        bank=norm_signed_deg(jerk_angle_update(&roll_limiter,target_bank,
            roll_limits.rate_deg_s,roll_limits.accel_deg_s2,step));
        bank=clampd(bank,-v->maximum_bank_angle,v->maximum_bank_angle);
        aoa=jerk_update(&pitch_limiter,target_aoa,
            pitch_limits.rate_deg_s,pitch_limits.accel_deg_s2,step);
        aoa=clampd(aoa,0.0,v->maximum_angle_of_attack);

        double rho=planet_atmospheric_density(p,altitude)*density_scale;
        double sound=planet_atmospheric_speed_of_sound(p,altitude);
        if(sound<=1)sound=sound0;
        double mach=speed/fmax(sound,1.0);
        double lf=0.0,df=1.0;
        aerodynamic_force_factors_mach(mach,fabs(aoa),v,&lf,&df);
        double q=.5*fmax(0.0,rho)*speed*speed;
        double q_ratio=q/q0;
        double lift=live_lift*q_ratio*fabs(lf)/fmax(.05,fabs(current_lf));
        double drag=live_drag*q_ratio*fmax(.05,df)/fmax(.05,current_df);
        if(!isfinite(lift)||lift<0)lift=live_lift;
        if(!isfinite(drag)||drag<.01)drag=live_drag;
        lift=clampd(lift,0.0,80.0);drag=clampd(drag,.01,60.0);
        double gravity=p->gravitational_parameter/
            fmax(1.0,(p->radius+altitude)*(p->radius+altitude));
        double effective_bank=clampd(bank*bank_eff,-89.0,89.0)*DEG2RAD;
        double vertical_lift=lift*cos(effective_bank);
        double lateral_lift=lift*sin(effective_bank);
        double gamma_rate=(vertical_lift-gravity*cos(gamma))/fmax(speed,30.0)+
            speed*cos(gamma)/fmax(p->radius+altitude,1.0);
        gamma=clampd(gamma+gamma_rate*step,-45.0*DEG2RAD,20.0*DEG2RAD);
        double horizontal=fmax(20.0,speed*cos(gamma));
        double course_rate=lateral_lift/horizontal;
        heading=norm_deg(heading+course_rate*RAD2DEG*step);
        double mid=heading-.5*course_rate*RAD2DEG*step;
        e+=horizontal*sin(mid*DEG2RAD)*step;
        n+=horizontal*cos(mid*DEG2RAD)*step;
        altitude+=speed*sin(gamma)*step;
        speed=fmax(1.0,speed+(-drag-gravity*sin(gamma))*step);
        minimum_speed=fmin(minimum_speed,speed);
        minimum_altitude=fmin(minimum_altitude,altitude);
        minimum_fpa=fmin(minimum_fpa,gamma*RAD2DEG);
        peak_q=fmax(peak_q,q);
        peak_load=fmax(peak_load,hypot(lift,drag)/fmax(gravity,.1));
    }
    GeoPoint end=local_point(origin,e,n,p->radius,altitude);
    runway_coordinates(end,origin,site->runway_heading,p->radius,&out.along_track,&out.cross_track);
    out.course=heading;out.altitude=altitude;out.speed=speed;
    out.minimum_speed=minimum_speed;
    out.minimum_altitude=minimum_altitude;
    out.minimum_flight_path_angle=minimum_fpa;
    out.specific_energy=rotating_specific_energy(end.latitude,altitude,speed,p);
    out.peak_dynamic_pressure_ratio=peak_q/fmax(v->maximum_dynamic_pressure,1.0);
    out.peak_g_ratio=peak_load/fmax(v->maximum_g_load,.1);
    out.valid=isfinite(out.along_track)&&isfinite(out.cross_track)&&isfinite(out.course)&&
        isfinite(out.altitude)&&isfinite(out.speed)&&isfinite(out.specific_energy)&&
        isfinite(out.minimum_flight_path_angle);
    return out;
}

static void taem_s_turn_target(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        const LandingConfiguration*cfg,double*target_along,double*target_cross,double*target_altitude,
        double*target_speed,double*target_course){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double shell_min=0.0,shell_max=0.0;entry_taem_handoff_altitude_bounds(s,&shell_min,&shell_max);
    double reserve=fmax(s->hac_radius*2.0,s->taem_interface_range*.65);
    *target_along=-s->final_approach_distance-reserve;
    *target_cross=0.0;
    *target_altitude=.5*(shell_min+shell_max);
    *target_speed=entry_taem_speed_target(v,s,p);
    *target_course=cfg->site.runway_heading;
    if(g->terminal_candidate.valid&&!g->terminal_candidate.geometry_degraded){
        const TerminalCandidate*c=&g->terminal_candidate;
        /* A geometry-valid but energy-degraded candidate is useful as a
           lateral rendezvous target only.  Its forecast altitude/speed belong
           to a path the energy certificate already says the shuttle cannot
           afford, so copying them into the S-turn objective caused MM305 to
           dissipate the very reserve needed to make that path feasible. */
        if(c->join.valid){
            double h=cfg->site.runway_heading*DEG2RAD;
            *target_along=c->join.lead_start.e*sin(h)+c->join.lead_start.n*cos(h);
            *target_cross=c->join.lead_start.e*cos(h)-c->join.lead_start.n*sin(h);
        }
        if(!c->energy_degraded){
            if(isfinite(c->altitude)&&c->altitude>cfg->site.altitude)*target_altitude=c->altitude;
            if(isfinite(c->speed)&&c->speed>v->minimum_safe_speed)*target_speed=c->speed;
        }
        if(isfinite(c->course))*target_course=c->course;
    }
    (void)t;
}

                
               
                            
                          
                         
                          
                        
                          
                       
                          
                

static bool taem_s_turn_rank_better(const TaemSTurnRank*a,
        const TaemSTurnRank*b){
    if(!a||!a->valid)return false;
    if(!b||!b->valid)return true;
    if(a->safety_violation<b->safety_violation)return true;
    if(a->safety_violation>b->safety_violation)return false;
    if(a->energy_deficit<b->energy_deficit)return true;
    if(a->energy_deficit>b->energy_deficit)return false;
    if(a->energy_excess<b->energy_excess)return true;
    if(a->energy_excess>b->energy_excess)return false;
    if(a->position_error<b->position_error)return true;
    if(a->position_error>b->position_error)return false;
    if(a->course_error<b->course_error)return true;
    if(a->course_error>b->course_error)return false;
    if(a->altitude_error<b->altitude_error)return true;
    if(a->altitude_error>b->altitude_error)return false;
    if(a->speed_error<b->speed_error)return true;
    if(a->speed_error>b->speed_error)return false;
    return a->control_effort<b->control_effort;
}

static EntryControlPlan taem_plan_s_turn_segment(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg,
        TaemSTurnProjection*chosen_projection){
    EntryControlPlan best={0};
    best.cost=INFINITY;
    best.planned_ut=t->ut;
    if(chosen_projection)memset(chosen_projection,0,
        sizeof(*chosen_projection));

    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    double target_along=0.0,target_cross=0.0,target_alt=0.0;
    double target_speed=0.0,target_course=0.0;
    taem_s_turn_target(g,t,p,cfg,&target_along,&target_cross,
        &target_alt,&target_speed,&target_course);

    double excess=0.0;
    if(!terminal_mm305_energy_excess_range(g,t,p,aero,cfg,&excess))
        excess=fmax(0.0,t->energy_excess_range);
    double minimum_leg=fmax(0.0,s->s_turn_minimum_leg_duration);
    double direct=hypot(t->runway_along_track-target_along,
        t->runway_cross_track-target_cross);

    /*
     * Receding-horizon terminal control must not confuse reversal dwell with
     * command hold time.  The former implementation used the 24 s minimum
     * S-turn leg as the shortest executable prediction, so one force snapshot
     * was extrapolated open-loop across a rapidly changing Mach/density regime.
     * Use the existing planner cadence as the shortest command horizon; keep
     * minimum_leg only as the side-reversal floor below.
     */
    double command_horizon=fmax(DBL_EPSILON,s->prediction_interval);
    double sign=g->s_turn_sign>=0.0?1.0:-1.0;

    bool inherited_reversal=g->entry_reversal_scheduled&&
        isfinite(g->entry_reversal_ut)&&
        g->entry_reversal_sign*sign<0.0;
    bool retained_reversal=g->taem_s_turn_plan.valid&&
        g->taem_s_turn_plan.has_planned_reversal&&
        isfinite(g->taem_s_turn_plan.planned_reversal_ut)&&
        g->taem_s_turn_plan.planned_reversal_sign*sign<0.0;

    /* The terminal target distance and calibrated minimum safe speed define a
       finite lookahead.  The command itself may be replanned at
       prediction_interval while the physical S-turn side remains latched. */
    double horizon=fmax(command_horizon,
        direct/fmax(v->minimum_safe_speed,DBL_MIN));
    if(inherited_reversal)
        horizon=fmin(horizon,fmax(command_horizon,
            g->entry_reversal_ut-t->ut));
    if(!isfinite(horizon)||horizon<command_horizon)return best;

    bool planned_reversal=excess>0.0||inherited_reversal||retained_reversal;
    double reversal_floor=g->has_s_turn_leg_started&&
        isfinite(g->s_turn_leg_started_ut)?
            g->s_turn_leg_started_ut+minimum_leg:
            t->ut+minimum_leg;

    double start_energy=rotating_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,p);
    double end_energy=rotating_specific_energy(
        cfg->site.latitude,target_alt,target_speed,p);
    double energy_span=fmax(fabs(start_energy-end_energy),DBL_MIN);
    double altitude_span=fmax(fabs(t->mean_altitude-target_alt),DBL_MIN);
    double speed_span=fmax(fabs(t->true_air_speed-target_speed),DBL_MIN);

    double max_bank=dynamic_bank_limit(t,v);
    max_bank=fmin(max_bank,v->maximum_bank_angle);
    if(!(max_bank>0.0))return best;

    /* MM305 is past the thermal-protection Entry regime.  Do not reuse the
       Entry AoA floor here: it is a hard 18 deg thermal attitude and forced
       terminal energy loss even when the downstream path was energy-short.
       Zero incidence is merely the numerical search bound; the propagated
       vertical envelope below prevents an unload from becoming a ballistic
       dive, and stabilized() enforces the real AoA slew/acceleration limits. */
    double aoa_floor=0.0;
    double aoa_ceiling=v->maximum_angle_of_attack;
    if(aoa_ceiling<aoa_floor)aoa_ceiling=aoa_floor;
    double minimum_path_slope=0.0,maximum_path_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_path_slope,&maximum_path_slope);
    if(!(maximum_path_slope>0.0)||maximum_path_slope<minimum_path_slope)return best;

    const int bank_samples=7; /* numerical control-space resolution */
    const int aoa_samples=5;
    const int duration_samples=(int)fmax(5.0,
        floor(sqrt((double)DBL_MANT_DIG)));
    TaemSTurnRank best_rank={0};

    for(int di=0;di<duration_samples;di++){
        double du=duration_samples>1?
            (double)di/(double)(duration_samples-1):0.0;
        /* Bias numerical resolution toward short, easily reversible commands
           while still spanning the complete physical horizon. */
        double leg_duration=command_horizon+
            (horizon-command_horizon)*du*du;
        for(int bi=0;bi<bank_samples;bi++){
            double bank=sign*max_bank*
                (double)(bi+1)/(double)bank_samples;
            for(int ai=0;ai<aoa_samples;ai++){
                double aoa=aoa_floor+
                    (aoa_ceiling-aoa_floor)*
                    (double)ai/(double)(aoa_samples-1);

                TaemSTurnProjection pr=taem_s_turn_project_segment(
                    t,course,p,aero,v,s,&cfg->site,
                    bank,aoa,leg_duration);
                if(!pr.valid)continue;

                TaemSTurnRank rank={0};
                rank.valid=true;
                double speed_violation=pr.minimum_speed>0.0?
                    fmax(0.0,v->minimum_safe_speed/pr.minimum_speed-1.0):
                    INFINITY;
                double vertical_violation=fmax(0.0,
                    (-pr.minimum_flight_path_angle-maximum_path_slope)/
                    fmax(maximum_path_slope,1.0));
                rank.safety_violation=fmax(vertical_violation,
                    fmax(speed_violation,
                        fmax(fmax(0.0,pr.peak_dynamic_pressure_ratio-1.0),
                            fmax(0.0,pr.peak_g_ratio-1.0))));
            double energy_delta=pr.specific_energy-end_energy;
            rank.energy_deficit=fmax(0.0,-energy_delta)/energy_span;
            rank.energy_excess=fmax(0.0,energy_delta)/energy_span;
            rank.position_error=hypot(
                pr.along_track-target_along,
                pr.cross_track-target_cross)/fmax(direct,DBL_MIN);
            rank.course_error=fabs(norm_signed_deg(
                pr.course-target_course))/180.0;
            rank.altitude_error=fabs(
                pr.altitude-target_alt)/altitude_span;
            rank.speed_error=fabs(
                pr.speed-target_speed)/speed_span;
            rank.control_effort=fmax(
                fabs(bank)/fmax(v->maximum_bank_angle,DBL_MIN),
                fabs(aoa)/fmax(v->maximum_angle_of_attack,DBL_MIN));

            if(!best.valid||taem_s_turn_rank_better(
                    &rank,&best_rank)){
                best_rank=rank;
                best.valid=true;
                best.terminal_ready=false;
                best.planned_ut=t->ut;
                best.target_bank=bank;
                best.target_aoa=aoa;
                best.target_heading=target_course;
                best.bank_cap=max_bank;
                best.target_turn_radius=live_turn_radius(
                    t,aero,v,bank);
                /* leg_duration is a prediction horizon, not an open-loop
                   command hold.  Execute one planner interval, then rebuild the
                   prediction from fresh measured aerodynamics. */
                best.segment_duration=command_horizon;
                best.cost=fmax(rank.safety_violation,
                    fmax(rank.energy_deficit,
                        fmax(rank.energy_excess,
                            fmax(rank.position_error,
                                fmax(rank.course_error,
                                    fmax(rank.altitude_error,
                                        rank.speed_error))))));
                best.taem_range_error=
                    pr.along_track-target_along;
                best.taem_speed=pr.speed;
                best.taem_energy_error=
                    pr.specific_energy-end_energy;
                best.closest_distance=hypot(
                    pr.along_track-target_along,
                    pr.cross_track-target_cross);
                best.has_planned_reversal=planned_reversal;
                best.planned_reversal_is_final=false;
                best.final_heading_lock=false;
                if(planned_reversal){
                    double projected_reversal=fmax(t->ut+leg_duration,reversal_floor);
                    if(retained_reversal)
                        projected_reversal=fmax(reversal_floor,
                            fmin(projected_reversal,
                                g->taem_s_turn_plan.planned_reversal_ut));
                    best.planned_reversal_ut=projected_reversal;
                }else best.planned_reversal_ut=NAN;
                best.planned_reversal_range=hypot(
                    pr.along_track,pr.cross_track);
                best.planned_reversal_sign=planned_reversal?
                    -sign:0.0;
                best.predicted_reversals=planned_reversal?1u:0u;
                if(chosen_projection)*chosen_projection=pr;
                }
            }
        }
    }

    return best;
}

static bool taem_s_turn_inherited_reversal_due(
        const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    const GuidanceSettings*s=cfg?&cfg->guidance:NULL;
    if(!g||!t||!p||!cfg||!s||!g->has_s_turn_leg_started||
       !isfinite(g->s_turn_leg_started_ut)||
       t->ut<g->s_turn_leg_started_ut)
        return false;

    double sign=g->s_turn_sign>=0.0?1.0:-1.0;
    bool committed=g->entry_reversal_scheduled&&
        isfinite(g->entry_reversal_ut)&&
        g->entry_reversal_sign*sign<0.0;
    if(committed&&t->ut>=g->entry_reversal_ut)return true;

    double excess=0.0;
    if(!terminal_mm305_energy_excess_range(g,t,p,aero,cfg,&excess))
        excess=fmax(0.0,t->energy_excess_range);
    if(!(excess>0.0))return false;
    double horizontal=fmax(guidance_lateral_speed(t),DBL_MIN);
    double full_leg=fmax(
        fmax(0.0,s->s_turn_minimum_leg_duration),
        excess/(2.0*horizontal)); /* decision-literal-ok: two symmetric legs */
    return t->ut-g->s_turn_leg_started_ut>=full_leg;
}

static bool taem_s_turn_reversal_due(
        const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s){
    if(!g||!t||!s||!g->taem_s_turn_plan.valid||
       !g->taem_s_turn_plan.has_planned_reversal||
       !isfinite(g->taem_s_turn_plan.planned_reversal_ut))
        return false;

    double dwell=t->ut-g->taem_s_turn_plan.planned_ut;
    if(g->has_s_turn_leg_started&&
       isfinite(g->s_turn_leg_started_ut)&&
       t->ut>=g->s_turn_leg_started_ut&&
       g->s_turn_sign*g->taem_s_turn_plan.target_bank>0.0)
        dwell=fmax(dwell,t->ut-g->s_turn_leg_started_ut);

    return dwell>=fmax(0.0,s->s_turn_minimum_leg_duration)&&
        t->ut>=g->taem_s_turn_plan.planned_reversal_ut;
}

       GuidanceResult taem_s_turn_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    /* If the physical Entry leg has already consumed its complete planned dwell,
       reverse before constructing the first MM305 child.  Otherwise a zero-time
       old-side child would leak one stale command across the ownership boundary. */
    if(!g->taem_s_turn_plan.valid&&taem_s_turn_inherited_reversal_due(g,t,p,aero,cfg))
        request_side(g,-g->s_turn_sign,t->ut);
    if(taem_s_turn_reversal_due(g,t,s)){
        double next_sign=g->taem_s_turn_plan.planned_reversal_sign>=0?1.0:-1.0;
        request_side(g,next_sign,t->ut);
        g->taem_s_turn_plan.valid=false;
    }
    bool plan_expired=!g->taem_s_turn_plan.valid||
        !isfinite(g->taem_s_turn_plan.planned_ut)||
        !isfinite(g->taem_s_turn_plan.segment_duration)||
        t->ut+1e-6>=g->taem_s_turn_plan.planned_ut+g->taem_s_turn_plan.segment_duration;
    TaemSTurnProjection projection={0};
    if(plan_expired){
        EntryControlPlan previous=g->taem_s_turn_plan;
        EntryControlPlan next=taem_plan_s_turn_segment(g,t,course,p,aero,cfg,&projection);
        const EntryControlPlan*parent=previous.plan_id?&previous:
            (g->entry_s_turn_plan.plan_id?&g->entry_s_turn_plan:NULL);
        control_plan_assign_lineage(g,&next,parent);
        g->taem_s_turn_plan=next;
        if(g->taem_s_turn_plan.valid){
            g->terminal_reference_bank=g->taem_s_turn_plan.target_bank;
            g->terminal_reference_aoa=g->taem_s_turn_plan.target_aoa;
            g->terminal_reference_heading=g->taem_s_turn_plan.target_heading;
            g->terminal_prediction_altitude=projection.valid?projection.altitude:t->mean_altitude;
            g->terminal_prediction_speed=projection.valid?projection.speed:t->true_air_speed;
            g->terminal_prediction_time=g->taem_s_turn_plan.segment_duration;
        }
    }
    if(!g->taem_s_turn_plan.valid){
        GuidanceCommand safe=atmospheric(t,t->ground_track_heading,0.0,v,0.0,false,PROFILE_TAEM);
        safe.heading_control_enabled=false;safe.has_target_aoa=true;
        safe.target_aoa=clampd(v->entry_angle_of_attack,8.0,18.0);
        safe.target_pitch=t->flight_path_angle+safe.target_aoa;
        return stabilized(g,result_make(PHASE_TAEM,safe,
            "TAEM S-turn planner unavailable; holding a bounded unloaded attitude while replanning.",
            "No finite planned terminal-energy segment is available on this frame."),t,v,s,dt);
    }

    EntryControlPlan*plan=&g->taem_s_turn_plan;
    GuidanceCommand c=atmospheric(t,plan->target_heading,plan->target_bank,v,0.0,false,PROFILE_TAEM);
    c.heading_control_enabled=false;c.has_target_aoa=true;c.target_aoa=plan->target_aoa;
    c.target_pitch=t->flight_path_angle+c.target_aoa;
    g->airbrakes_deployed=false;
    double until_reversal=plan->has_planned_reversal&&isfinite(plan->planned_reversal_ut)?
        fmax(0.0,plan->planned_reversal_ut-t->ut):fmax(0.0,plan->planned_ut+plan->segment_duration-t->ut);
    double display_excess=0.0;
    if(!terminal_mm305_energy_excess_range(g,t,p,aero,cfg,&display_excess))
        display_excess=fmax(0.0,t->energy_excess_range);
    char status[384];
    snprintf(status,sizeof(status),
        "TAEM planned S-turn: terminal excess %+.1f km, bank %+.1f deg, AoA %.1f deg, heading %.1f deg, plan %.1f s, reversal in %.1f s, forecast %.1f km / %.1f km / %.0f m/s.",
        display_excess/1000.0,plan->target_bank,plan->target_aoa,plan->target_heading,
        plan->segment_duration,until_reversal,g->terminal_prediction_altitude/1000.0,
        plan->closest_distance/1000.0,g->terminal_prediction_speed);
    return stabilized(g,result_make(PHASE_TAEM,c,status,NULL),t,v,s,dt);
}

/* HAC side convention: +1 is the south-side HAC for a southbound shuttle,
   -1 is the north-side HAC for a northbound shuttle.  The first turn relative
   to the runway is authoritative; cross-track is only a tie-breaker when the
   vehicle is already aligned with the runway. */
       double terminal_default_hac_side(const Telemetry*t,const LandingSite*site,double course){
    if(!site)return 1.0;
    double reference=isfinite(course)?course:
        (t&&isfinite(t->ground_track_heading)?t->ground_track_heading:
         (t&&isfinite(t->heading)?t->heading:site->runway_heading));
    double turn_delta=norm_signed_deg(site->runway_heading-reference);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(turn_delta<-angular_resolution)return 1.0;
    if(turn_delta>angular_resolution)return -1.0;
    if(t&&isfinite(t->runway_cross_track)&&
       fabs(t->runway_cross_track)>sqrt(DBL_EPSILON))
        return t->runway_cross_track>=0.0?1.0:-1.0;
    return 1.0;
}

bool guidance_begin_hac_test(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,char*message,size_t message_size){
    if(!g||!t||!p||!cfg){if(message&&message_size)snprintf(message,message_size,"HAC test initialization received an invalid argument.");return false;}
    (void)aero;
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double altitude_floor=fmax(1000.0,s->flare_altitude*8.0);
    double speed_floor=fmax(v->minimum_safe_speed*1.20,v->final_approach_speed*1.25);
    if(t->radar_altitude<=altitude_floor){if(message&&message_size)snprintf(message,message_size,"HAC-only test requires at least %.0f m radar altitude.",altitude_floor);return false;}
    if(t->true_air_speed<speed_floor){
        if(message&&message_size)snprintf(message,message_size,"HAC-only test requires at least %.0f m/s airspeed for recovery margin.",speed_floor);
        fprintf(stderr,"HAC_INIT_REJECT reason=speed-floor speed=%.9f minimum=%.9f\n",t->true_air_speed,speed_floor);
        return false;
    }
    if(t->dynamic_pressure>v->maximum_dynamic_pressure*1.05){if(message&&message_size)snprintf(message,message_size,"HAC-only test blocked because dynamic pressure is above the vehicle limit.");return false;}
    guidance_machine_init(g);guidance_set_engaged(g,true);
    g->terminal_glide_mode=true;
    g->terminal_rehearsal_mode=true;
    g->terminal_test_capture_active=true;
    g->terminal_fixed_hac_fixture_latched=false;
    
    g->has_burn_command_started=true;
    g->deorbit_burn_completed=true;
    g->atmospheric_interface_crossed=true;
    g->terminal_region_entered=true;
    /* The 15 km rehearsal begins too fast for the tight HAC that matches the
       shuttle's roughly 20 deg unpowered glide.  Do not lock a huge circle at
       the initial low-AoA state.  First capture the aerodynamic glide, then
       choose a one-revolution HAC from the measured turn authority once the
       vehicle has decelerated enough for a physically useful radius. */
    g->hac_side=terminal_default_hac_side(t,&cfg->site,course);g->hac_side_selected=false;g->hac_radius=s->hac_radius;
    g->hac_remaining=0;g->hac_previous_angle=0;g->hac_progress_valid=false;
    g->minimum_turn_radius=INFINITY;
    /* Preflare is the terminal energy gate.  HAC geometry is selected so the
       shuttle reaches this height with enough kinetic energy to execute the
       pull-up, rather than merely intersecting an arbitrary low-altitude
       final line.  Keep the values vehicle-relative so another shuttle can
       reuse the same guidance without inheriting STS-N-specific constants. */
    terminal_glide_initialize(g,v,s);
    g->terminal_test_upstream_staging=hac_variant_b_requested();
    g->terminal_test_upstream_staging_line_active=false;
    g->terminal_test_upstream_staging_y=NAN;
    g->terminal_test_upstream_staging_course=NAN;
    g->terminal_test_upstream_staging_epoch_ut=-INFINITY;
    g->terminal_test_upstream_capture_ready=false;
    g->terminal_test_upstream_energy_rejected=false;
    g->terminal_test_upstream_plan_valid=false;
    g->terminal_test_upstream_provisional_plan_valid=false;
    g->terminal_test_upstream_planned_radius=0.0;
    g->terminal_test_upstream_speed_target_valid=false;
    g->terminal_test_upstream_speed_target=0.0;
    g->terminal_test_upstream_tuple_valid=false;
    g->terminal_test_upstream_p0_e=0.0;
    g->terminal_test_upstream_p0_n=0.0;
    g->terminal_test_upstream_p3_e=0.0;
    g->terminal_test_upstream_p3_n=0.0;
    g->terminal_test_upstream_center_e=0.0;
    g->terminal_test_upstream_center_n=0.0;
    g->terminal_test_upstream_cone_start_angle=0.0;
    g->terminal_test_upstream_cone_end_angle=0.0;
    g->terminal_test_upstream_cone_arc_length=0.0;
    g->terminal_test_upstream_lead_curve=false;
    g->terminal_test_upstream_lead_start_e=0.0;
    g->terminal_test_upstream_lead_start_n=0.0;
    g->terminal_test_upstream_lead_p1_e=0.0;
    g->terminal_test_upstream_lead_p1_n=0.0;
    g->terminal_test_upstream_lead_p2_e=0.0;
    g->terminal_test_upstream_lead_p2_n=0.0;
    g->terminal_test_upstream_lead_start_course=0.0;
    g->terminal_test_upstream_lead_end_course=0.0;
    g->terminal_test_upstream_lead_length=0.0;
    g->terminal_test_upstream_vertical_profile_valid=false;
    g->terminal_test_upstream_profile_start_altitude=0.0;
    g->terminal_test_upstream_profile_final_altitude=0.0;
    g->terminal_test_upstream_profile_slope_deg=0.0;
    g->terminal_test_upstream_profile_initial_slope_deg=0.0;
    g->terminal_test_upstream_profile_settled_slope_deg=0.0;
    g->terminal_test_upstream_profile_final_slope_deg=0.0;
    g->terminal_test_upstream_profile_transition_distance=0.0;
    g->terminal_test_upstream_profile_exit_transition_distance=0.0;
    g->terminal_test_upstream_avoidable_aoa_energy_loss=0.0;
    g->terminal_test_upstream_last_avoidable_aoa_loss=0.0;
    g->terminal_test_upstream_last_plan_ut=-INFINITY;
    g->terminal_test_upstream_last_log_ut=-INFINITY;
    g->terminal_test_upstream_future_ref_valid=false;
    g->terminal_test_upstream_future_ref_ut=-INFINITY;
    g->terminal_test_upstream_future_ref_stage_time=0.0;
    g->terminal_test_upstream_future_ref_stage_distance=0.0;
    g->terminal_test_upstream_future_ref_radius=0.0;
    g->terminal_test_upstream_future_ref_p0_e=0.0;
    g->terminal_test_upstream_future_ref_p0_n=0.0;
    g->terminal_test_upstream_future_ref_speed=0.0;
    g->terminal_test_upstream_future_ref_altitude=0.0;
    g->terminal_test_upstream_future_ref_margin=NAN;
    if(!taem_exec_enter(g,t,s,p,aero,cfg,true)){
        if(message&&message_size)snprintf(message,message_size,"HAC-only landing test could not reconstruct MM305 executive ownership.");
        return false;
    }
    g->phase=PHASE_TAEM;
    if(message&&message_size)snprintf(message,message_size,
        "HAC-only landing test armed: preflare gate %.0f m / %.0f m/s (minimum %.0f m/s); capturing a steep unpowered glide before selecting HAC from live turn authority and gate energy.",
        g->terminal_test_preflare_altitude,g->terminal_test_preflare_target_speed,g->terminal_test_preflare_min_speed);
    return true;
}

       double terminal_hac_energy_aoa(const VehicleProfile*v){
    return clampd(v->entry_angle_of_attack-8.0,8.0,10.5);
}

