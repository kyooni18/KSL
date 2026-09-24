       bool terminal_candidate_commit_ready(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    const TerminalCandidate*c=&g->terminal_candidate;
    if(g->terminal_path_committed||!c->valid)return false;
    double live_course=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    if(!terminal_candidate_operationally_usable(g,c,t,live_course,p,cfg))return false;

    double runway_heading=cfg->site.runway_heading*DEG2RAD;
    double lead_along=c->join.lead_start.e*sin(runway_heading)+
        c->join.lead_start.n*cos(runway_heading);
    double lead_cross=c->join.lead_start.e*cos(runway_heading)-
        c->join.lead_start.n*sin(runway_heading);
    double arrival_horizon=fmax(0.0,c->arrival_ut-t->ut);
    double minimum_turn_radius=0.0;
    double forecast_path=0.0;
    double forecast_time=0.0;
    /* The response lead starts at lead_start.  For a live-origin candidate
       that point is the current vehicle position, even though the curved
       route from there to p0 may be kilometres long.  A point-capture query
       aimed back at the vehicle can invent an entire turn circuit and then
       project the arrival tens of seconds beyond the real lead origin. */
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double current_e=0.0,current_n=0.0;
    local_offsets(origin,current,p->radius,&current_e,&current_n);
    double lead_distance=hypot(c->join.lead_start.e-current_e,
        c->join.lead_start.n-current_n);
    double origin_tolerance=fmax(5.0,t->horizontal_speed*0.1);
    /*
     * selected_ut == arrival_ut is the durable identity of a live-origin path.
     * After MM305 starts flying that path, the vehicle is expected to move away
     * from lead_start. Requiring it to remain near the historical origin turns
     * the retained path into a fictitious point-reacquisition problem with zero
     * arrival horizon and permanently blocks commit.
     */
    bool live_origin=isfinite(c->selected_ut)&&isfinite(c->arrival_ut)&&
        fabs(c->selected_ut-c->arrival_ut)<=1e-6;
    if(!live_origin&&lead_distance>origin_tolerance){
        forecast_path=decision_target_path_length(g,t,live_course,
            lead_along,lead_cross,c->course,NAN,p,cfg,&minimum_turn_radius);
        forecast_time=isfinite(forecast_path)&&t->horizontal_speed>0.0?
            forecast_path/t->horizontal_speed:INFINITY;
    }
    DecisionMargin forecast_margin=decision_margin(arrival_horizon,forecast_time);
    if(!forecast_margin.valid)return false;

    Telemetry arrival=*t;
    double arrival_e=0.0,arrival_n=0.0;
    double arrival_course=t->ground_track_heading;
    terminal_project_planning_state(g,t,t->ground_track_heading,p,aero,cfg,
        arrival_horizon,&arrival,&arrival_e,&arrival_n,&arrival_course);

    double response_drop=0.0,arrival_height_to_gate=0.0;
    bool vertical_ready=terminal_candidate_vertical_response_ready_from_arrival(
        g,&arrival,arrival_course,p,c,cfg,
        &response_drop,&arrival_height_to_gate);
    bool energy_ready=terminal_candidate_live_energy_ready(g,t,p,aero,cfg,c);

    double forecast_tolerance=isfinite(forecast_time)?
        fmax(0.5,0.15*forecast_time):0.5;
    /* Commit on a modestly negative energy margin: the HAC radius and the
       speedbrake absorb a few percent of error, whereas waiting uncommitted
       only bleeds more energy. */
    if(!energy_ready&&g->terminal_candidate_live_energy_valid&&
       isfinite(terminal_energy_commit_tolerance)&&
       g->terminal_candidate_live_energy_margin>=-terminal_energy_commit_tolerance)
        energy_ready=true;
    bool ready=forecast_margin.margin>=-forecast_tolerance&&vertical_ready&&energy_ready;

    const char*diag_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    static double last_commit_diag_ut=-INFINITY;
    double diag_second=floor(t->ut);
    if(!ready&&!g->diagnostic_shadow&&diag_env&&strcmp(diag_env,"1")==0&&
       diag_second>last_commit_diag_ut){
        fprintf(stderr,
            "HAC commit margin: UT %.1f forecast %+.2fs (need %.2f avail %.2f path %.0f) vertical=%s energy=%s drop %.0f/%.0f turn %.0fm response %.2fs\n",
            t->ut,forecast_margin.margin,forecast_time,arrival_horizon,forecast_path,
            vertical_ready?"yes":"no",energy_ready?"yes":"no",
            response_drop,arrival_height_to_gate,
            minimum_turn_radius,c->response);
        last_commit_diag_ut=diag_second;
    }
    return ready;
}

       HACGuidance terminal_test_update_spiral(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double radius=fmax(1800.0,g->hac_radius);
    HACGuidance h=hac_guidance_radius(t,&cfg->site,s,p->radius,g->hac_side,
        planet_surface_gravity(p),course,radius);

    double progress_rate=hac_oriented_progress_rate(t,&cfg->site,s,p->radius,g->hac_side,radius,course);
    g->terminal_test_revolution_remaining=fmax(0,
        g->terminal_test_revolution_remaining-progress_rate*fmax(0,dt));

    double bank=fmin(55.0,dynamic_bank_limit(t,v));
    double min_turn=live_turn_radius(t,aero,v,bank);
    g->minimum_turn_radius=min_turn;
    double total_angle=g->terminal_test_revolution_remaining;
    /* Once a HAC has been selected, treat it as the lateral reference path.
       Re-solving or aggressively shrinking the circle while flying it moves
       the waypoint under the vehicle and turns a stable intercept into a
       chasing problem. Keep the computed HAC geometry fixed; radial/course
       pursuit plus the vertical/energy loops provide the small corrections
       needed after capture. */
    total_angle=g->terminal_test_revolution_remaining;
    g->hac_remaining=fmax(0,total_angle*radius);
    g->hac_previous_angle=h.angle;
    return h;
}

       double terminal_test_speed_floor(const VehicleProfile*v){
    return fmax(65.0,v->touchdown_speed*.88);
}

       GuidanceSettings terminal_path_settings(const GuidanceMachine*g,const GuidanceSettings*s){
    GuidanceSettings r=*s;
    if(g->terminal_final_handoff_latched&&isfinite(g->terminal_final_handoff_distance)&&
       g->terminal_final_handoff_distance>0.0)
        r.final_approach_distance=g->terminal_final_handoff_distance;
    else if(g->terminal_glide_mode&&g->terminal_test_final_approach_distance>0)
        r.final_approach_distance=g->terminal_test_final_approach_distance;
    if(g->terminal_glide_mode&&g->terminal_test_glide_slope>0){
        r.final_glide_slope=g->terminal_test_glide_slope;
        r.taem_glide_slope=g->terminal_test_glide_slope;
    }
    if(g->hac_circuit_slope>0)r.taem_glide_slope=g->hac_circuit_slope;
    return r;
}

static double terminal_test_speed_shortfall(const Telemetry*t,double target_speed){
    return fmax(0.0,target_speed-t->true_air_speed-3.0);
}

static double terminal_test_energy_gamma_bias(const Telemetry*t,double target_speed){
    /* Energy recovery must key off the speed schedule we are actually trying
       to reach, not an absolute near-stall guard.  In the live HAC rehearsal
       the vehicle was already 25-40 m/s below the arc schedule while still
       above 100 m/s, so the old guard remained inactive.  The altitude loop
       then treated the naturally steeper low-energy glide as a path error and
       commanded more AoA, spending still more kinetic energy to climb back to
       the geometric path.  Move the commanded FPA progressively steeper as
       soon as schedule shortfall appears.  The earlier .28 gain arrested most
       of the loss but still settled around a 24-25 deg glide with TAS slowly
       decaying.  Give speed priority enough authority to use the already
       accepted ~35 deg outer-glide envelope; the bias fades automatically as
       airspeed returns to schedule. */
    return -clampd(terminal_test_speed_shortfall(t,target_speed)*.38,0,16.0);
}

static double terminal_test_energy_trim(const Telemetry*t,double target_speed,double nominal_aoa){
    /* FPA bias alone is not enough: at 120 m/s the previous law could still
       request 20+ deg AoA while trying to recover the high side of the path.
       Explicitly unload incidence when kinetic energy is below schedule so
       gravity can accelerate the aircraft instead of being converted into
       induced/profile drag.  Preserve a small positive incidence floor for
       controllability; once speed recovers the relief fades continuously back
       to the nominal measured/best-glide AoA. */
    /* The 0.20 deg/(m/s) law still left about 10 deg AoA with a 35-45 m/s
       speed deficit.  At that incidence the shuttle settled near a -22 deg
       glide even while guidance asked for -28..-30 deg, so it kept trading
       kinetic energy away instead of diving to recover it.  Unload more
       decisively once the deficit is established; 6-ish degrees remains a
       useful aerodynamic incidence while allowing gravity to rebuild speed. */
    double relief=clampd(terminal_test_speed_shortfall(t,target_speed)*.36,0,12.5);
    return clampd(nominal_aoa-relief,0.0,22.0);
}

static void terminal_test_energy_limit_aoa(GuidanceCommand*c,const Telemetry*t,
        double target_speed,double nominal_aoa){
    double shortfall=terminal_test_speed_shortfall(t,target_speed);
    if(!c||shortfall<=0)return;
    /* A steep real FPA used to feed back through aerodynamic_pitch_target as
       a request for still more AoA: while the aircraft was 25-35 m/s below
       schedule, the path-error term could raise a deliberately relieved
       11-13 deg trim back to 18+ deg.  Energy recovery has priority over
       climbing back to the geometric altitude line.  Collapse the extra
       path-restoration incidence as the shortfall grows, but keep a small
       incidence floor so the terminal controller retains useful lift and
       pitch authority during the dive. */
    double recovery=clampd(shortfall/30.0,0,1);
    double recovery_trim=terminal_test_energy_trim(t,target_speed,nominal_aoa);
    double ceiling=recovery_trim+(1.0-recovery)*5.0;
    c->target_aoa=clampd(fmin(c->target_aoa,ceiling),0.0,22.0);
    c->target_pitch=t->flight_path_angle+c->target_aoa;
}

static double terminal_taem_drag_aoa_ceiling(const Telemetry*t,const VehicleProfile*v,
        double expected_drag,double drag_budget,double nominal_aoa){
    /* Scale the observed specific-energy loss with the same incidence model
       used by the terminal predictor. Net TAS loss is unsuitable here: it
       already includes gravity's contribution along the descent. */
    double current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,fmax(1.0,hypot(t->angle_of_attack,t->sideslip)),
        v,NULL,&current_df);
    double upper=clampd(nominal_aoa,0.0,fmin(22.0,v->maximum_angle_of_attack));
    for(double aoa=upper;aoa>0.0;aoa=fmax(0.0,aoa-.5)){
        double df=1.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,NULL,&df);
        if(expected_drag*df/fmax(.05,current_df)<=drag_budget)return aoa;
    }
    return 0.0;
}

       double terminal_projected_lift_accel_at_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double aoa){
    if(!t||!v||!isfinite(aoa))return NAN;
    double target=clampd(aoa,0.0,v->maximum_angle_of_attack);
    double lf=0.0;
    aerodynamic_force_factors_mach(t->mach,target,v,&lf,NULL);
    double modeled=NAN;
    if(isfinite(t->dynamic_pressure)&&t->dynamic_pressure>=0.0&&
       isfinite(aero.ballistic_coefficient)&&aero.ballistic_coefficient>DBL_MIN&&
       isfinite(aero.lift_to_drag)&&aero.lift_to_drag>=0.0)
        modeled=t->dynamic_pressure/aero.ballistic_coefficient*
            aero.lift_to_drag*fabs(lf);

    double incidence=hypot(t->angle_of_attack,t->sideslip);
    double measured=t->mass>DBL_MIN&&isfinite(t->lift_force)&&t->lift_force>=0.0?
        t->lift_force/t->mass:NAN;
    double same_incidence=sqrt(DBL_EPSILON)*fmax(1.0,fabs(incidence));
    if(isfinite(measured)&&fabs(target-incidence)<=same_incidence)
        return measured;
    return isfinite(modeled)&&modeled>=0.0?modeled:NAN;
}

/* A fixed analytic arc shares the lift vector with the vertical FPA response.
   When the actual velocity vector has already fallen materially below the
   recoverability-clamped target, spending the remaining lift on lateral
   curvature can deepen the dive faster than the airframe can arrest it.  The
   limit below is derived from the same normal-force request as
   terminal_fpa_force_aoa() and the maximum-AoA projected lift.  It is only a
   defensive cap for the diagnostic/committed arc; it does not alter the HAC
   reference, its radius, or normal authority admission. */
static double terminal_lateral_aoa_floor(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg){
    return terminal_required_aoa_for_lateral(t,aero,v,lateral_accel,bank_deg);
}

static bool terminal_energy_path_unrecoverable(double expected_loss,double gravity,
        double kinetic_to_gate,double air_path,double recoverable_fpa_deg){
    if(!isfinite(expected_loss)||!isfinite(gravity)||!isfinite(kinetic_to_gate)||
       !isfinite(air_path)||!isfinite(recoverable_fpa_deg)||air_path<=0.0)
        return true;
    double down=clampd(recoverable_fpa_deg,0.0,90.0)*DEG2RAD;
    double max_recoverable=fmax(0.0,gravity)*sin(down)+
        fmax(0.0,kinetic_to_gate)/air_path;
    return expected_loss>max_recoverable;
}

static bool terminal_vertical_path_unrecoverable(double height_to_gate,
        double ground_path_to_gate,double recoverable_fpa_deg){
    if(!isfinite(height_to_gate)||!isfinite(ground_path_to_gate)||
       !isfinite(recoverable_fpa_deg))return true;
    double down=clampd(recoverable_fpa_deg,0.0,nextafter(90.0,0.0))*DEG2RAD;
    double maximum_drop=fmax(0.0,ground_path_to_gate)*tan(down);
    return fmax(0.0,height_to_gate)>maximum_drop;
}

                                                               
                                                                               

static bool terminal_vertical_energy_priority(double flight_path_angle,double target_fpa,
        double expected_loss,double drag_budget){
    double shallow_error=flight_path_angle-target_fpa;
    return shallow_error>0.0||(drag_budget>0.0&&expected_loss>drag_budget);
}

static bool terminal_lateral_incidence_priority(const GuidanceMachine*g,bool energy_priority){
    /* Before publication, the preview may trade lateral convergence for a
       steeper energy-recovery dive because its geometry is still disposable.
       After publication the HAC is a hard lateral constraint: incidence may
       not be unloaded below the lift needed to remain on that frozen path.
       If the two requirements conflict, the path should have remained a
       preview instead of silently becoming unflyable after commit. */
    return g&&g->terminal_path_committed ? true : !energy_priority;
}

       double taem_bank_demand(const Telemetry*t,const HACGuidance*h,double hac_radius,double side,AerodynamicModel aero,const VehicleProfile*v){
    double eff=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    double lim=fmin(fabs(v->maximum_bank_angle),dynamic_bank_limit(t,v));
    double lateral=h?h->lateral_acceleration:0.0;
    if(!isfinite(lateral)||hac_radius<=DBL_MIN||!(lim>0.0))return 0.0;
    (void)side;

    /*
     * Solve L sin(phi_eff) = a_lat directly.  Prefer the force actually
     * measured from stock KSP; if the telemetry force sample is unavailable,
     * project lift at the same Mach/q from the calibrated aerodynamic model.
     * Do not substitute an Earth-gravity coordinated-turn approximation.
     */
    double lift=live_lift_accel(t,aero,v);
    if(!(lift>DBL_MIN)||!isfinite(lift))
        lift=terminal_projected_lift_accel_at_aoa(t,aero,v,
            terminal_hac_energy_aoa(t,v));
    if(!(lift>DBL_MIN)||!isfinite(lift))return 0.0;

    double effective_limit=fmin(lim*eff,nextafter(90.0,0.0))*DEG2RAD;
    double available=fabs(lift*sin(effective_limit));
    double commanded=fmin(fabs(lateral),available);
    double effective_bank=asin(clampd(commanded/lift,0.0,1.0));
    double bank=effective_bank/fmax(eff,DBL_MIN)*RAD2DEG;
    return clampd(copysign(bank,lateral),-lim,lim);
}

       GuidanceResult taem_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    /* During MM305 acquisition, a geometry-clean candidate is deliberately
       advisory until its live energy and vertical-response margins qualify.
       Still, the aircraft must fly that candidate while it converges.  Use a
       private published view for path generation so the executive's committed
       bit remains false and the candidate can still be replaced by the next
       planner refresh. */
    GuidanceMachine preview_owner={0};
    const GuidanceMachine*path_owner=g;
    bool preview_candidate=g->terminal_glide_mode&&g->terminal_test_capture_active&&
        g->terminal_candidate.valid&&
        terminal_candidate_operationally_usable(g,&g->terminal_candidate,t,course,p,cfg)&&
        !g->terminal_candidate.geometry_degraded;
    if(preview_candidate){
        preview_owner=*g;
        terminal_publish_candidate(&preview_owner);
        path_owner=&preview_owner;
    }
    GuidanceSettings path_settings=terminal_path_settings(path_owner,s);
    const GuidanceSettings*hs=&path_settings;
    double radius=path_owner->hac_radius>DBL_MIN?
        path_owner->hac_radius:s->hac_radius;
    RunwayCaptureEnvelope vertical_envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    /* An invalid envelope usually means no pull-up margin at the current
       speed (e.g. CL-max in a banked glide).  Latching the bound to the live
       FPA then forbids the dive that restores speed and lift authority, so the
       vehicle stays pinned at CL-max down to stall speed.  Above the preflare
       gate allow the -35 deg terminal limit; near the ground keep the latch. */
    double preflare_gate=isfinite(g->terminal_test_preflare_altitude)&&
        g->terminal_test_preflare_altitude>0.0?g->terminal_test_preflare_altitude:
        s->flare_altitude;
    double capture_radar=isfinite(t->radar_altitude)?t->radar_altitude:
        t->mean_altitude-cfg->site.altitude;
    double recoverable_fpa=vertical_envelope.valid?
        vertical_envelope.recoverable_fpa_deg:
        (capture_radar>preflare_gate?fmax(35.0,-t->flight_path_angle):
            fmax(0.0,-t->flight_path_angle));
    TaemPhase lateral_stage=taem_exec_owns_vehicle(&g->taem_exec)?g->taem_exec.phase:
        (g->hac_captured&&!g->hac_transition_active?TAEM_PHASE_RUNWAY_ALIGNMENT:TAEM_PHASE_PATH_ACQUISITION);
    HACPhaseGuidance phase_path=hac_phase_guidance(lateral_stage,path_owner,t,&cfg->site,hs,p->radius,
        path_owner->hac_side,planet_surface_gravity(p),course,radius);
    if(!phase_path.valid){
        GuidanceCommand safe=atmospheric(t,t->ground_track_heading,0.0,v,0.0,false,PROFILE_TAEM);
        safe.heading_control_enabled=false;safe.has_target_aoa=true;
        safe.target_aoa=clampd(v->entry_angle_of_attack,8.0,v->maximum_angle_of_attack);
        safe.target_pitch=t->flight_path_angle+safe.target_aoa;
        return stabilized(g,result_make(PHASE_TAEM,safe,
            "MM305 phase has no qualified lateral path law; holding a bounded unloaded attitude.",
            "TAEM phase execution is inhibited until its dedicated path provider is available."),t,v,s,dt);
    }
    HACGuidance h=phase_path.guidance;
    bool fixed_hac_lead_active=path_owner==g&&
        path_owner->fixed_alignment_hac_latched&&
        path_owner->hac_transition_active&&
        path_owner->hac_transition_heading_cone&&
        path_owner->hac_transition_lead_length>1.0&&
        path_owner->hac_transition_lead_progress<.995;
    if(fixed_hac_lead_active){
        /* Keep the last lead demand as the initial condition for the circle
           controller.  Position and tangent are already C1 at the analytic
           entry, but curvature feedback can still jump when the vehicle
           reaches p0; retaining this measured lead demand gives the airframe
           time to rotate its lift vector before the circle feed-forward owns
           the full turn. */
        g->hac_transition_handoff_lateral_acceleration=h.lateral_acceleration;
        g->hac_transition_handoff_lateral_valid=true;
        g->hac_transition_handoff_blend=0.0;
    }else if(path_owner==g&&g->fixed_alignment_hac_latched&&
             g->hac_transition_active&&g->hac_transition_heading_cone&&
             g->hac_transition_lead_length>1.0&&
             g->hac_transition_handoff_lateral_valid){
        double q=clampd(g->hac_transition_handoff_blend,0.0,1.0);
        double smooth_q=q*q*(3.0-2.0*q);
        double lead_lateral=g->hac_transition_handoff_lateral_acceleration;
        h.lateral_acceleration=lead_lateral+
            smooth_q*(h.lateral_acceleration-lead_lateral);
        h.bank=clampd(atan2(h.lateral_acceleration,
            fmax(planet_surface_gravity(p),.01))*RAD2DEG,-70.0,70.0);
        /* The bridge is C1 in position/tangent and stabilized() already owns the
           measured roll-rate/acceleration limits.  A multi-second curvature blend
           duplicated that response delay: the fixture spent the first ~3 s of the
           analytic arc asking for less bank than the airframe could safely attain.
           Keep only a short command-reference blend; the actuator path remains
           smooth through the measured limiter. */
        double blend_time=clampd(fmax(1.25,
            g->hac_transition_response_time*.22),1.25,1.75);
        g->hac_transition_handoff_blend=clampd(q+
            fmax(0.0,dt)/blend_time,0.0,1.0);
        if(g->hac_transition_handoff_blend>=.999999){
            g->hac_transition_handoff_lateral_valid=false;
            g->hac_transition_handoff_aoa_valid=false;
        }
    }
    const bool fixed_hac_turn=path_owner->fixed_alignment_hac_latched&&
        path_owner->hac_transition_heading_cone&&path_owner->hac_transition_active&&
        !path_owner->hac_completed&&!g->final_approach_captured;
    const bool fixed_hac_runway_line=path_owner->fixed_alignment_hac_latched&&
        path_owner->hac_transition_heading_cone&&!path_owner->hac_transition_active&&
        !g->final_approach_captured;
    double path_remaining=path_owner->hac_transition_active?h.arc_remaining:path_owner->hac_remaining;
    if(fixed_hac_runway_line)path_remaining=fmax(0.0,-t->runway_along_track);
    double hac_slope=path_owner->hac_circuit_slope>0?path_owner->hac_circuit_slope:(path_owner->terminal_glide_mode&&path_owner->terminal_test_glide_slope>0?path_owner->terminal_test_glide_slope:hs->taem_glide_slope);
    if(fixed_hac_turn&&path_owner->fixed_hac_reference_valid)
        hac_slope=fixed_hac_reference_slope(path_owner,path_remaining);
    if(g->terminal_glide_mode||path_owner->hac_transition_heading_cone){
        double final_slope=g->terminal_glide_mode?
            path_owner->terminal_test_glide_slope:hs->final_glide_slope;
        double final_entry_altitude=cfg->site.altitude+
            hs->final_approach_distance*tan(final_slope*DEG2RAD);
        if(fixed_hac_turn&&path_owner->fixed_hac_reference_valid)
            h.desired_altitude=fixed_hac_reference_altitude(path_owner,path_remaining);
        else if(fixed_hac_runway_line)
            h.desired_altitude=cfg->site.altitude+
                path_remaining*tan(final_slope*DEG2RAD);
        else
            h.desired_altitude=final_entry_altitude+
                path_remaining*tan(hac_slope*DEG2RAD);
    }else h.desired_altitude+=(g->hac_remaining-h.arc_remaining)*tan(hac_slope*DEG2RAD);
    h.arc_remaining=path_remaining;
    double path_length=fixed_hac_runway_line?fmax(1.0,path_remaining):
        fmax(hs->final_approach_distance,h.arc_remaining+hs->final_approach_distance);
    double drag=live_drag_accel(t,aero,v);
    double final_target=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.08);
    double target_energy=rotating_specific_energy(cfg->site.latitude,cfg->site.altitude,final_target,p);
    double zero_speed_energy=rotating_specific_energy(t->latitude,t->mean_altitude,0,p);
    double allowable=fmax(final_target,sqrt(fmax(0,2*(target_energy+fmax(.03,drag)*path_length*.95-zero_speed_energy))));
    double speed_schedule=final_target+clampd(path_remaining/550.0,0,90.0);
    double gate_gamma=0.0,energy_gamma=0.0,expected_loss=0.0,drag_budget=0.0;
    bool committed_energy_unrecoverable=false,committed_vertical_unrecoverable=false;
    if(g->terminal_glide_mode){
        double gate_altitude=cfg->site.altitude+g->terminal_test_preflare_altitude;
        double gate_speed=g->terminal_test_preflare_target_speed;
        double gate_potential=rotating_specific_energy(cfg->site.latitude,gate_altitude,0.0,p);
        double gate_ground=g->terminal_test_preflare_altitude/
            fmax(tan(g->terminal_test_glide_slope*DEG2RAD),1e-3);
        double path_to_gate=fixed_hac_runway_line?
            fmax(0.0,path_remaining-gate_ground):
            fmax(0.0,path_remaining)+
                fmax(0.0,hs->final_approach_distance-gate_ground);
        double height=fmax(0.0,t->mean_altitude-gate_altitude);
        double air_path=fmax(1.0,hypot(path_to_gate,height));
        double potential=zero_speed_energy-gate_potential;
        double kinetic=.5*(t->true_air_speed*t->true_air_speed-gate_speed*gate_speed);
        double gravity=fmax(.1,planet_surface_gravity(p));
        expected_loss=terminal_expected_energy_loss_accel(g,t,aero,v);
        /* Close height and kinetic energy over the SAME remaining path.
           With dK/ds=-kinetic/air_path, excess V^2 falls in proportion to
           distance remaining, reaching gate speed only at the gate. At or
           below gate speed the permitted deceleration is zero or negative.
           D = -g*sin(gamma) + kinetic/air_path supplies the corresponding
           FPA; actual height independently supplies the geometric closure.
           Unlike the old inverse energy equation, abundant potential energy
           cannot collapse an immediate speed target to the preflare floor. */
        gate_gamma=-atan2(height,fmax(path_to_gate,1.0))*RAD2DEG;
        energy_gamma=-asin(clampd((expected_loss-kinetic/air_path)/gravity,
            -1.0,1.0))*RAD2DEG;
        /* D is loss per AIR metre, not horizontal metre. Bound it by both
           the total gate-energy budget and gravity along the CURRENT FPA:
           a still-shallow aircraft must not spend speed while waiting for
           the commanded dive. Credit no descent beyond the safe envelope. */
        double gravity_along_path=-gravity*sin(
            clampd(t->flight_path_angle,-recoverable_fpa,90.0)*DEG2RAD);
        drag_budget=fmax(0.0,fmin((potential+kinetic)/air_path,
            gravity_along_path+kinetic/air_path));
        committed_energy_unrecoverable=g->terminal_path_committed&&
            terminal_energy_path_unrecoverable(expected_loss,gravity,kinetic,air_path,
                recoverable_fpa);

        committed_vertical_unrecoverable=g->terminal_path_committed&&
            terminal_vertical_path_unrecoverable(height,path_to_gate,recoverable_fpa);
        /* This is now only the recovery floor, not an intermediate TAS
           demand. The rate/drag law above owns planned deceleration. */
        speed_schedule=gate_speed;
        allowable=speed_schedule;
    }

    if(g->terminal_path_committed&&g->terminal_glide_mode&&!fixed_hac_turn){
        bool replan_margin=vertical_envelope.valid&&
            vertical_envelope.runway_time.margin>vertical_envelope.control_response_time_s&&
            vertical_envelope.vertical_recovery.margin>=0.0&&
            vertical_envelope.speed.margin>=0.0;
        bool frozen_path_unrecoverable=committed_energy_unrecoverable||
            committed_vertical_unrecoverable;
        if(frozen_path_unrecoverable&&replan_margin)
            g->terminal_energy_mismatch_duration+=fmax(0.0,dt);
        else
            g->terminal_energy_mismatch_duration=0.0;

        double confirmation_time=vertical_envelope.valid?
            fmax(dt,vertical_envelope.control_response_time_s):INFINITY;
        if(g->terminal_energy_mismatch_duration>=confirmation_time){
            /* A frozen path is no longer executable if either measured drag
               exceeds the unpowered energy budget or its remaining ground
               distance cannot close the altitude inside the -35 deg FPA
               envelope. Preserve measured energy, discard only geometry,
               and let TAEM search a replacement while maneuver room remains. */
            bool vertical_miss=committed_vertical_unrecoverable;
            terminal_invalidate_frozen_path_preserve_energy(g);
            g->terminal_energy_mismatch_duration=0.0;
            terminal_predict(g,t,course,p,aero,cfg,dt);
            GuidanceCommand replanning=atmospheric(t,g->terminal_reference_heading,
                g->terminal_reference_bank,v,0.0,false,PROFILE_TAEM);
            replanning.heading_control_enabled=false;
            replanning.has_target_aoa=true;
            replanning.target_aoa=g->terminal_reference_aoa;
            replanning.target_pitch=t->flight_path_angle+replanning.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,replanning,
                vertical_miss?
                    "Frozen terminal path cannot close altitude inside the allowed FPA envelope; replanning longer geometry in TAEM.":
                    "Frozen terminal path energy model diverged from live drag; replanning terminal geometry in TAEM.",
                vertical_miss?
                    "The remaining ground path cannot reach preflare height even at the maximum allowed terminal descent; the frozen path was discarded before outer-final passage.":
                    "Measured aerodynamic loss cannot be recovered within the allowed terminal FPA envelope; the previous terminal path was discarded without returning to S-turn."),
                t,v,s,dt);
        }
    }else g->terminal_energy_mismatch_duration=0.0;
    double ae=h.desired_altitude-t->mean_altitude;
    double path_limit=fmax(0.0,recoverable_fpa);
    double path=robust_pid_update(&g->taem_altitude_pid,ae,dt,-path_limit,path_limit);
    double energy_schedule=speed_schedule;
    double gamma_request=g->terminal_glide_mode?
        fmin(-hac_slope+path,fmin(gate_gamma,energy_gamma)):
        -hac_slope+path+terminal_test_energy_gamma_bias(t,energy_schedule);
    double gamma=fmax(-recoverable_fpa,gamma_request);
    if(g->terminal_path_committed&&g->terminal_glide_mode&&
       isfinite(g->terminal_test_glide_slope)&&
       g->terminal_test_glide_slope>0.0){
        /* A committed candidate was certified against its selected vertical
           slope.  The recoverable-FPA bound is an airframe limit, not a new
           route: letting the energy loop replace the candidate with a much
           steeper descent spends altitude and lateral authority faster than
           the path provider modeled, then reports the resulting cross-track
           loss as a late live-energy failure.  Keep the path's FPA contract
           as the shallow-side bound; if the contract is no longer flyable,
           the mismatch/replan logic above must discard it rather than silently
           turning it into a different trajectory.
         */
        double committed_path_fpa=-clampd(g->terminal_test_glide_slope,
            0.0,fmax(0.0,recoverable_fpa));
        gamma=fmax(gamma,committed_path_fpa);
    }
    if(fixed_hac_turn&&path_owner->fixed_hac_reference_valid){
        /* The fixed HAC vertical profile is part of the committed trajectory, not
           merely a nominal suggestion.  The energy/gate law above may diagnose a
           deficit, but it must not spend altitude faster than the frozen profile:
           doing so kept the vehicle near -22 deg while the profile shallowed toward
           zero and caused terrain impact at only ~68 percent arc progress despite
           sub-100 m radial tracking.  The altitude PID term remains in this
           reference, so normal closed-loop vertical correction is preserved.
           Energy shortfall is handled by AoA/unload or by declaring the path
           infeasible, never by silently flying a different vertical trajectory. */
        double fixed_reference=fmax(-recoverable_fpa,-hac_slope+path);
        gamma=fixed_reference;
    }
    if(g->terminal_path_committed){
        g->terminal_reference_fpa=gamma;
    }
    double learned=t->calibrated_best_glide_angle_of_attack>0?
        t->calibrated_best_glide_angle_of_attack:v->entry_angle_of_attack;
    double glide=clampd(learned,0.0,v->maximum_angle_of_attack);
    double trim=g->terminal_glide_mode?
        terminal_test_energy_trim(t,energy_schedule,glide):
        clampd(glide+clampd((t->true_air_speed-allowable)*.012,0,6),3,14);
    if(!g->terminal_glide_mode&&terminal_test_speed_shortfall(t,energy_schedule)>0)
        trim=fmin(trim,terminal_test_energy_trim(t,energy_schedule,glide));
    double energy_aoa_ceiling=v->maximum_angle_of_attack;
    bool terminal_unload=g->terminal_glide_mode&&
        !g->terminal_path_committed&&t->flight_path_angle>-recoverable_fpa;
    if(terminal_unload){
        energy_aoa_ceiling=terminal_taem_drag_aoa_ceiling(t,v,expected_loss,drag_budget,glide);
        trim=fmin(trim,energy_aoa_ceiling);
    }else if(g->terminal_glide_mode)trim=glide;
    /*
     * Explicit mission contract: terminal energy control has only trajectory,
     * bank and AoA available.  Do not treat speedbrakes as latent energy
     * authority even when a physics model for them is present.
     */
    terminal_speedbrake_closed(g);
    double hac_course_rate_limit_deg=INFINITY;
    if(g->hac_side_selected&&g->terminal_region_entered){
        /* Keep actual ground-track rotation in the same bandwidth as the
           moving heading reference.  During the join, release the rate with
           exactly the same smooth progress law used by the heading limiter;
           a separate fixed 1.25 deg/s ceiling here used to pin the transition
           forever even after its reference had legitimately sped up.  Once
           on the HAC, allow roughly the circle's natural V/R rate with only a
           modest comfort ceiling. */
        double max_course_rate_deg;
        if(g->hac_transition_active){
            max_course_rate_deg=hac_join_rate_cap(t,g->hac_transition_progress);
        }else{
            /* Once established on the frozen circle, never cap below its own
               V/R feed-forward.  The selector guarantees that this natural
               rate remains inside the shared continuous envelope. */
            double natural=t->true_air_speed/fmax(radius,1.0)*RAD2DEG;
            max_course_rate_deg=clampd(natural*1.25,1.05,hac_course_rate_cap(t));
        }
        /* Energy recovery is handled by the FPA/AoA schedule.  Reducing the
           lateral-rate cap below the frozen path curvature made the aircraft
           save lift by abandoning the HAC, which is not a valid reference
           law and caused radial error to grow monotonically. */
        if(fixed_hac_turn){
            /* A committed circle cannot be flown below its own V/R rate.
               Join comfort limits apply to acquisition, not to this frozen arc. */
            double natural_rate=guidance_lateral_speed(t)/fmax(radius,1.0)*RAD2DEG;
            max_course_rate_deg=fmax(max_course_rate_deg,natural_rate*1.25);
        }
        hac_course_rate_limit_deg=max_course_rate_deg;
        double max_course_rate=max_course_rate_deg*DEG2RAD;
        double lateral_cap=fmax(.45,t->true_air_speed*max_course_rate);
        h.lateral_acceleration=clampd(h.lateral_acceleration,-lateral_cap,lateral_cap);
    }
    bool fixed_hac_arc=fixed_hac_turn&&
        (!path_owner->hac_transition_active||
         path_owner->hac_transition_lead_length<=1.0||
         path_owner->hac_transition_lead_progress>=.995);
    double fixed_hac_required_lateral=NAN;
    double fixed_hac_available_lateral=NAN;
    double fixed_hac_lift_limit=NAN;
    double fixed_hac_normal_limit=NAN;
    if(fixed_hac_arc){
        /* The analytic circle remains the active reference, but a forced
           diagnostic must not turn an authority shortfall into an ever larger
           bank/AoA chase.  Bound only the executable curvature by the lift and
           bank that exist on this frame; normal admission still rejects a
           circle whose required curvature is outside this envelope. */
        double gravity=fmax(.1,planet_surface_gravity(p));
        double lift=fmax(0.0,live_lift_accel(t,aero,v));
        /*
         * Do not close a circular dependency at the current incidence.  The
         * fixed-arc allocator is also the mechanism that asks the pitch loop
         * for the incidence required to supply lateral lift.  Limiting the
         * path law to live_lift at a temporarily unloaded AoA would request
         * only the lift already present, so the AoA floor could never build
         * the lift needed by the analytic circle.  Project the same measured
         * polar to the vehicle's bounded maximum AoA for this command-side
         * ceiling; the eventual command remains bounded by bank, AoA, normal
         * load, and stabilized() response limits.  Normal admission still
         * rejects a circle whose required curvature cannot be met.
         */
        double projected_lift=terminal_projected_lift_accel_at_aoa(t,aero,v,
            v->maximum_angle_of_attack);
        if(isfinite(projected_lift)&&projected_lift>lift)
            lift=projected_lift;
        double normal_limit=fmax(0.0,v->maximum_g_load*gravity);
        double effectiveness=clampd(isfinite(t->bank_effectiveness)&&
            t->bank_effectiveness>0.0?t->bank_effectiveness:1.0,.35,1.8);
        double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
        double effective_bank=clampd(bank_limit*effectiveness,0.0,
            nextafter(89.0,0.0))*DEG2RAD;
        double available_lateral=fmin(lift,normal_limit)*sin(effective_bank);
        fixed_hac_required_lateral=guidance_lateral_speed(t)*
            guidance_lateral_speed(t)/fmax(radius,1.0);
        fixed_hac_available_lateral=available_lateral;
        fixed_hac_lift_limit=lift;
        fixed_hac_normal_limit=normal_limit;
        if(isfinite(available_lateral)&&available_lateral>0.0)
            h.lateral_acceleration=clampd(h.lateral_acceleration,
                -available_lateral,available_lateral);
    }
    double bank=taem_bank_demand(t,&h,radius,g->hac_side,aero,v);
    double incidence_fraction=clampd((t->angle_of_attack+1.0)/8.0,0,1);
    double bank_guard=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    double vertical_bank_limit=bank_guard;
    /* The runway-capture envelope assumes immediate wings-level sink arrest
       over direct runway range. Applying it during the descending fixed HAC
       makes acos(required/max lift) collapse to zero and abandons the turn.
       The committed arc keeps its dynamic/structural bank envelope; its AoA
       allocator supplies lateral lift and the vertical recovery/abort law
       remains responsible for the remaining altitude reserve. */
    if(!fixed_hac_turn&&vertical_envelope.valid&&vertical_envelope.maximum_normal_accel_mps2>0.0){
        double gravity=planet_surface_gravity(p);
        double required_vertical=gravity+
            fmax(0.0,-t->vertical_speed)/
            fmax(vertical_envelope.time_available_s,DBL_MIN);
        double ratio=clampd(required_vertical/
            vertical_envelope.maximum_normal_accel_mps2,0.0,1.0);
        vertical_bank_limit=acos(ratio)*RAD2DEG;
        bank_guard=fmin(bank_guard,vertical_bank_limit);
    }
    (void)incidence_fraction;
    if(g->hac_side_selected&&g->terminal_region_entered&&isfinite(hac_course_rate_limit_deg)&&
       !g->terminal_glide_mode){
        /* Non-terminal TAEM still uses the coordinated-turn bank envelope.
           In the unpowered terminal HAC, however, course rate has already
           been limited by clamping lateral acceleration to V*omega above.
           Applying atan(V*omega/g) a second time assumes lift ~= weight and
           forces an artificially small bank during a steep, deliberately
           unloaded glide.  The lateral-AoA allocator then has to raise AoA
           back toward 10 deg to make the same lateral force, which adds drag
           and vertical lift and flattens the requested -30..-35 deg descent.
           Let terminal HAC use the measured/projected lift with the existing
           dynamic bank envelope; the already-clamped lateral acceleration is
           the actual course-rate safety constraint. */
        double rate_deg=hac_course_rate_limit_deg;
        double gravity=fmax(.1,planet_surface_gravity(p));
        double rate_bank=atan(t->true_air_speed*(rate_deg*DEG2RAD)/gravity)*RAD2DEG;
        bank_guard=fmin(bank_guard,clampd(rate_bank*1.05,6.0,45.0));
    }
    bank=clampd(bank,-bank_guard,bank_guard);
    /*
     * STS-N terminal flight is fully unpowered.  An energy deficit remains a
     * feasibility/control problem; it may not be converted into a thrust
     * recovery branch by configuration.
     */
    robust_pid_reset(&g->speed_pid);
    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,h.heading,bank,v,0.0,false,PROFILE_TAEM);
    /* Do not re-impose the old -12 deg surface-pitch floor after the terminal
       energy law has deliberately unloaded AoA.  With FPA near -23 deg that
       floor mechanically forced ~11 deg incidence, so the shuttle could not
       dive toward the requested -28..-30 deg path even while 40 m/s below its
       speed schedule.  The terminal gamma law is already bounded to -35 deg
       and incidence unloading is suspended at that descent limit. */
    double taem_pitch_floor=(g->terminal_glide_mode||g->hac_circuit_count>0||t->true_air_speed<energy_schedule)?-30.0:-12.0;
    if(g->terminal_path_committed&&g->terminal_glide_mode){
        /* A committed candidate is certified against the force-derived AoA
           needed to move the measured velocity vector toward its selected
           FPA.  Converting that FPA back through gamma+nominal-trim asks for
           the wrong surface pitch when the live FPA is still steeper than the
           route; in the 22 km probe it turned a ~6 deg requirement into
           ~26--28 deg commanded AoA.  Use the same bounded reference that the
           selector and live-energy gate evaluated.  The lateral allocator
           below may raise it if the frozen curvature requires more lift.
         */
        double committed_aoa=terminal_fpa_force_aoa(t,p,aero,v,gamma);
        if(isfinite(committed_aoa)){
            c.has_target_aoa=true;
            c.target_aoa=clampd(committed_aoa,0.0,v->maximum_angle_of_attack);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }else{
            aerodynamic_pitch_target(&c,t,v,clampd(gamma+trim,taem_pitch_floor,24));
        }
    }else{
        aerodynamic_pitch_target(&c,t,v,clampd(gamma+trim,taem_pitch_floor,24));
    }
    if(!g->terminal_glide_mode||terminal_unload)
        terminal_test_energy_limit_aoa(&c,t,energy_schedule,glide);
    if(terminal_unload){
        c.target_aoa=fmin(c.target_aoa,energy_aoa_ceiling);
        c.target_pitch=t->flight_path_angle+c.target_aoa;
    }
    double hard_lateral_aoa_floor=0.0;
    double fixed_hac_lead_brake_aoa=NAN;
    if(fixed_hac_lead_active&&
       path_owner->fixed_hac_lead_speed_target_valid&&
       isfinite(path_owner->fixed_hac_lead_target_speed)){
        double lead_remaining=fixed_hac_lead_remaining(path_owner);
        double requested_lead_aoa=fixed_hac_lead_drag_aoa(t,p,aero,v,
            path_owner->fixed_hac_lead_target_speed,lead_remaining,
            hac_slope);
        if(isfinite(requested_lead_aoa)&&c.has_target_aoa&&
           isfinite(c.target_aoa)&&
           requested_lead_aoa>c.target_aoa){
            /* The lead speed target is advisory command-path evidence for a
               forced diagnostic, not permission to redraw or resize the
               committed path.  Preserve the polar-derived AoA request here;
               stabilized() remains the sole executable slew/acceleration
               boundary, so the vehicle can only deliver the incidence that
               its measured response permits over the remaining lead.  The
               previous 0.75--2.0 degree incremental cap was a second,
               unmeasured response model: it let terminal FPA unloading reduce
               the command to 1--2 degrees even when the lead drag calculation
               required roughly 19--28 degrees.  The analytic circle, its
               radius, and all lead/arc accounting remain unchanged. */
            fixed_hac_lead_brake_aoa=requested_lead_aoa;
            c.target_aoa=fixed_hac_lead_brake_aoa;
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
    }
    if(g->terminal_glide_mode&&g->hac_side_selected){
        /* The direct atmospheric controller closes the loop on AoA, not FPA.
           Therefore a -35 deg gamma request is only achieved dynamically by
           unloading lift until gravity steepens the velocity vector. The old
           5 deg incidence floor left enough lift at Mach 2-3 to settle near
           -13..-15 deg even while gamma stayed at -35. Use the measured FPA
           error as an additional unload term; it fades continuously once the
           velocity vector catches the commanded descent. */
        double shallow_gamma_error=t->flight_path_angle-gamma;
        if(shallow_gamma_error>3.0){
            double extra_unload=clampd((shallow_gamma_error-3.0)*.20,0.0,4.5);
            double incidence_floor=terminal_test_speed_shortfall(t,energy_schedule)>12.0?0.0:.5;
            c.target_aoa=fmax(incidence_floor,c.target_aoa-extra_unload);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
        /* Energy recovery and lateral capture share the same lift vector.
           First ask whether the requested curvature can be flown at the
           *maximum permitted bank* with the energy-relieved incidence. The
           old code evaluated the AoA floor at the current small bank, then
           raised incidence to 12-14 deg even when simply banking a little
           more would have produced the same lateral acceleration. That
           cancelled the terminal unload command and flattened the descent.
           Only add incidence when the bank envelope itself is insufficient. */
        bool energy_priority=terminal_vertical_energy_priority(
            t->flight_path_angle,gamma,expected_loss,drag_budget);
        double lateral_floor=fixed_hac_turn?
            terminal_required_aoa_for_lateral_limit(t,aero,v,h.lateral_acceleration,
                bank_guard,v->maximum_angle_of_attack):
            terminal_lateral_aoa_floor(t,aero,v,h.lateral_acceleration,bank_guard);
        hard_lateral_aoa_floor=lateral_floor;
        /* A committed path is only valid while its lateral curvature and its
           vertical energy law can be satisfied by the same lift vector.  In
           the live 86 km replay the frozen 48 km HAC eventually needed about
           ten degrees of incidence for lateral force while energy recovery
           needed roughly three to five degrees to steepen from -11 toward
           -25..-31 deg.  The hard lateral floor therefore cancelled the
           unload command for tens of seconds until the path became formally
           unrecoverable.  If that conflict is already strong while we still
           have replanning height, discard only the frozen geometry and let
           TAEM keep the energy-preserving command while it searches a shorter
           path. */
        double energy_limited_aoa=c.target_aoa;
        double incidence_conflict=lateral_floor-energy_limited_aoa;
        double conflict_replan_height=fmax(3500.0,g->terminal_test_preflare_altitude*5.0);
        bool fixed_heading_cone_hac=g->terminal_path_committed&&
            g->terminal_path_kind==TERMINAL_PATH_HAC&&
            g->hac_transition_heading_cone;
        bool conflict_replan_margin=g->terminal_path_committed&&
            !fixed_heading_cone_hac&&
            h.arc_remaining>s->final_approach_distance+3000.0&&
            t->radar_altitude>conflict_replan_height&&
            t->true_air_speed>v->minimum_safe_speed*1.45;
        bool lateral_energy_conflict=conflict_replan_margin&&energy_priority&&
            shallow_gamma_error>7.0&&incidence_conflict>2.0&&
            expected_loss>drag_budget*1.05+.10;
        if(lateral_energy_conflict){
            terminal_invalidate_frozen_path_preserve_energy(g);
            terminal_predict(g,t,course,p,aero,cfg,dt);
            GuidanceCommand replanning=atmospheric(t,g->terminal_reference_heading,
                g->terminal_reference_bank,v,0.0,false,PROFILE_TAEM);
            replanning.heading_control_enabled=false;
            replanning.has_target_aoa=true;
            replanning.target_aoa=g->terminal_reference_aoa;
            replanning.target_pitch=t->flight_path_angle+replanning.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,replanning,
                "Frozen HAC lateral lift conflicted with terminal energy recovery; replanning shorter geometry in TAEM.",
                "The committed curvature required enough incidence to prevent the commanded energy-recovery descent; measured drag history was retained for the replacement plan."),
                t,v,s,dt);
        }
        /* While the HAC is still only a preview, a materially shallow or
           over-dissipating vehicle may let vertical energy closure own
           incidence and accept temporary lateral lag.  Once the path is
           committed that trade is no longer legal: the frozen curvature gets
           its required lift first and the vertical profile must absorb the
           remaining energy error. */
        if(terminal_lateral_incidence_priority(g,energy_priority)&&
           lateral_floor>0.0&&c.target_aoa<lateral_floor){
            c.target_aoa=lateral_floor;
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
        /* Allocate roll and incidence from one future force state.  Using the
           instantaneous lift here while pitch simultaneously commanded a very
           different AoA created a two-actuator phase error: low-lift frames
           demanded near-maximum bank, then high-lift frames demanded only a few
           degrees after the roll actuator had already committed.  The resulting
           bank oscillation spiraled a geometrically correct fixed HAC inward.
           Project lift at the same target AoA used by the pitch command; retain
           measured live lift only as a no-projection fallback.  stabilized()
           still enforces the measured pitch/roll rate and acceleration limits. */
        double target_lift=terminal_projected_lift_accel_at_aoa(t,aero,v,c.target_aoa);
        if(!(target_lift>.01))
            target_lift=live_lift_accel(t,aero,v);
        double eff=clampd(t->bank_effectiveness,.35,1.8);
        double maximum_component=sin(clampd(bank_guard*eff,0.0,89.0)*DEG2RAD);
        if(fabs(h.lateral_acceleration)>=.35&&target_lift>.01&&maximum_component>.01){
            double required_component=clampd(fabs(h.lateral_acceleration)*1.05/target_lift,
                0.0,maximum_component);
            double required_bank=asin(required_component)/eff*RAD2DEG;
            c.target_roll=clampd(copysign(required_bank,h.lateral_acceleration),
                -bank_guard,bank_guard);
        }
    }
    if(g->terminal_path_committed){
        /* Crossfade from the already-followed TAEM preview into the exact
           frozen join.  The transition geometry is continuous, but a commit
           can still move bank/AoA references by a few degrees; blending those
           references explicitly makes the ownership change visually natural
           instead of relying on actuator rate limits to hide a command step. */
        double blend_time=clampd(fmax(3.0,g->hac_transition_response_time*.85),3.0,8.0);
        g->hac_commit_blend=clampd(g->hac_commit_blend+fmax(0.0,dt)/blend_time,0.0,1.0);
        double q=g->hac_commit_blend;
        q=q*q*(3.0-2.0*q);
        c.target_heading=norm_deg(g->terminal_reference_heading+
            q*norm_signed_deg(c.target_heading-g->terminal_reference_heading));
        c.target_roll=g->terminal_reference_bank+
            q*(c.target_roll-g->terminal_reference_bank);
        if(c.has_target_aoa){
            c.target_aoa=g->terminal_reference_aoa+
                q*(c.target_aoa-g->terminal_reference_aoa);
            /* The ownership crossfade is cosmetic; it may not undo the hard
               lift floor of a committed path.  This matters most immediately
               after commit, when the frozen preview AoA can still be lower
               than the incidence required by the first curved join segment. */
            if(hard_lateral_aoa_floor>0.0)
                c.target_aoa=fmax(c.target_aoa,hard_lateral_aoa_floor);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
    }
    if(fixed_hac_lead_active&&isfinite(fixed_hac_lead_brake_aoa)&&
       c.has_target_aoa&&isfinite(c.target_aoa)){
        /* Commit blending is a reference handoff, not permission to discard
           the finite lead's latched speed adaptation. Reapply the lead-only
           drag floor after that cosmetic crossfade; stabilized() still limits
           the actual AoA slew and the analytic circle remains untouched. */
        double lead_aoa=fmax(fixed_hac_lead_brake_aoa,hard_lateral_aoa_floor);
        c.target_aoa=fmax(c.target_aoa,lead_aoa);
        c.target_pitch=t->flight_path_angle+c.target_aoa;
    }
    if(fixed_hac_lead_active&&c.has_target_aoa&&isfinite(c.target_aoa)&&
       path_owner->hac_transition_lead_progress>.82){
        /* Pre-position incidence during only the final part of the finite lead.
           The lead remains the active C1 path and its lateral curvature is not
           replaced by the circle early.  This removes the observed 2 deg -> 28 deg
           incidence step at p0, so the analytic arc receives useful lift as soon as
           the roll limiter begins capturing its curvature. */
        double arc_lateral=guidance_lateral_speed(t)*guidance_lateral_speed(t)/
            fmax(radius,1.0);
        double arc_entry_aoa=terminal_required_aoa_for_lateral_limit(t,aero,v,
            arc_lateral,bank_guard,v->maximum_angle_of_attack);
        if(isfinite(arc_entry_aoa)&&arc_entry_aoa>c.target_aoa){
            double u=clampd((path_owner->hac_transition_lead_progress-.82)/.18,
                0.0,1.0);
            double q=u*u*(3.0-2.0*u);
            c.target_aoa+=q*(arc_entry_aoa-c.target_aoa);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
    }
    if(fixed_hac_lead_active&&c.has_target_aoa&&isfinite(c.target_aoa)){
        /* The finite lead is the vehicle's actual adaptation path.  Preserve
           its final incidence as the initial condition for the analytic
           circle instead of asking the airframe to accept a vertical command
           step at the C1 lateral handoff. */
        g->hac_transition_handoff_aoa=clampd(c.target_aoa,0.0,v->maximum_angle_of_attack);
        g->hac_transition_handoff_aoa_valid=true;
    }else if(path_owner==g&&g->fixed_alignment_hac_latched&&
             g->hac_transition_active&&g->hac_transition_heading_cone&&
             g->hac_transition_lead_length>1.0&&
             g->hac_transition_handoff_aoa_valid&&c.has_target_aoa&&
             isfinite(c.target_aoa)){
        double q=clampd(g->hac_transition_handoff_blend,0.0,1.0);
        double smooth_q=q*q*(3.0-2.0*q);
        double lead_aoa=g->hac_transition_handoff_aoa;
        c.target_aoa=lead_aoa+smooth_q*(c.target_aoa-lead_aoa);
        if(hard_lateral_aoa_floor>0.0)
            c.target_aoa=fmax(c.target_aoa,hard_lateral_aoa_floor);
        c.target_pitch=t->flight_path_angle+c.target_aoa;
    }
    c.gear=t->gear;
    bool spline_path=path_owner->terminal_path_kind==TERMINAL_PATH_SPLINE;
    char status[320],warn[256]={0};
    if(spline_path)
        snprintf(status,sizeof(status),"Aero TAEM spline: %s %.1f km, cross %+.1f km, speed %.0f / %.0f m/s, FPA %.1f / %.1f deg, energy ceiling %.0f m/s.",g->hac_transition_active?"curve remaining":"line capture",h.arc_remaining/1000,t->runway_cross_track/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,allowable);
    else snprintf(status,sizeof(status),"Aero TAEM HAC: R %.1f km, %s %.1f km, radial %+.1f km, speed %.0f / %.0f m/s, FPA %.1f / %.1f deg, energy ceiling %.0f m/s.",radius/1000,g->hac_transition_active?"join+arc":"arc",h.arc_remaining/1000,h.radial_error/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,allowable);
    if(g->terminal_glide_mode){
        if(spline_path)
            snprintf(status,sizeof(status),"Aero TAEM spline: %s %.1f km, cross %+.1f km, speed %.0f m/s, preflare gate %.0f m/s, FPA %.1f / %.1f deg, drag %.2f / %.2f m/s2 budget.",g->hac_transition_active?"curve remaining":"line capture",h.arc_remaining/1000,t->runway_cross_track/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,expected_loss,drag_budget);
        else snprintf(status,sizeof(status),"Aero TAEM HAC: R %.1f km, %s %.1f km, radial %+.1f km, speed %.0f m/s, preflare gate %.0f m/s, FPA %.1f / %.1f deg, drag %.2f / %.2f m/s2 budget.",radius/1000,g->hac_transition_active?"join+arc":"arc",h.arc_remaining/1000,h.radial_error/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,expected_loss,drag_budget);
    }
    if(g->hac_circuit_count>0){
        size_t used=strlen(status);
        snprintf(status+used,sizeof(status)-used," High-pass circuit %u.",g->hac_circuit_count);
    }
    if(g->hac_completed&&!g->final_approach_captured&&!g->taem_exec.terminal_evaluation.feasible){
        const TaemTerminalContract*contract=&g->taem_exec.terminal_contract;
        const char*reason=taem_terminal_block_reason_string(g->taem_exec.terminal_evaluation.block_reason);
        if(g->taem_exec.terminal_evaluation.valid)
            snprintf(warn,sizeof(warn),"Final delivery blocked: %s; margins range %.0f m, q %.0f Pa, altitude %.0f m, FPA %.1f deg, energy %.0f J/kg, response %.1f/%.1f s.",
                reason,contract->range_margin,contract->dynamic_pressure_margin,contract->altitude_margin,
                contract->flight_path_angle_margin,contract->specific_energy_margin,
                contract->response_time_available,contract->response_time_required);
        else snprintf(warn,sizeof(warn),"Final delivery blocked: %s.",reason);
    }else if(g->hac_plan_degraded)
        snprintf(warn,sizeof(warn),"Flying the best available terminal path; one or more preferred feasibility envelopes are violated.");
    else if(g->terminal_glide_mode&&t->true_air_speed<g->terminal_test_preflare_min_speed)
        snprintf(warn,sizeof(warn),"Terminal energy is below the preflare floor; unloading AoA and trading altitude for airspeed.");
    else if(fabs(ae)>8000)snprintf(warn,sizeof(warn),"TAEM altitude is outside the model-derived path corridor.");
    else if(!spline_path&&fabs(h.radial_error)>radius*.25)snprintf(warn,sizeof(warn),"Capturing the model-derived HAC radius.");
    GuidanceResult r=result_make(g->phase,c,status,warn[0]?warn:NULL);
    trajectory_copy(&r.reference,ref);
    r=stabilized(g,r,t,v,s,dt);
    if(!g->diagnostic_shadow&&getenv("KSP_LANDER_HAC_COMMAND_TRACE")){
        double fixed_circle_radial=h.radial_error;
        if(g->fixed_alignment_hac_latched&&g->hac_transition_heading_cone){
            GeoPoint trace_origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
            GeoPoint trace_current={t->latitude,t->longitude,t->mean_altitude};
            double trace_e=0.0,trace_n=0.0;
            local_offsets(trace_origin,trace_current,p->radius,&trace_e,&trace_n);
            fixed_circle_radial=hypot(trace_e-g->hac_transition_cone_center_e,
                trace_n-g->hac_transition_cone_center_n)-radius;
        }
        const char*trace_stage=g->hac_transition_active&&
            g->hac_transition_lead_length>1.0&&g->hac_transition_lead_progress<.995?
            "lead":(g->hac_transition_active?"arc":"steady");
        double fixed_reference_gamma=(fixed_hac_turn&&
            path_owner->fixed_hac_reference_valid)?-hac_slope+path:NAN;
        fprintf(stderr,"HAC_COMMAND ut=%.3f along=%.1f cross=%.1f heading=%.3f speed=%.3f radial=%.1f lateral=%.4f raw_bank=%.4f guard=%.4f guarded=%.4f allocated=%.4f final=%.4f actual=%.4f aoa=%.3f lift=%.4f max_normal=%.4f available=%.4f fixed=%d committed=%d radius=%.1f circle_radial=%.1f stage=%s lead_active=%d owner_is_g=%d lead_curve=%d lead_progress=%.4f cone_progress=%.4f handoff_blend=%.4f commit_blend=%.4f reference_aoa=%.3f handoff_aoa=%.3f lead_target_v=%.1f lead_remaining=%.1f lead_brake_aoa=%.3f path_remaining=%.1f hac_slope=%.3f fixed_gamma=%.3f gamma_request=%.3f gamma=%.3f gate_gamma=%.3f energy_gamma=%.3f required_lateral=%.4f available_lateral=%.4f arc_lift=%.4f arc_normal=%.4f\n",
            t->ut,t->runway_along_track,t->runway_cross_track,course,
            t->true_air_speed,h.radial_error,h.lateral_acceleration,
            taem_bank_demand(t,&h,radius,g->hac_side,aero,v),bank_guard,
            bank,c.target_roll,r.command.target_roll,t->roll,c.target_aoa,
            live_lift_accel(t,aero,v),vertical_envelope.maximum_normal_accel_mps2,
            vertical_envelope.time_available_s,g->fixed_alignment_hac_latched,
            g->terminal_path_committed,radius,fixed_circle_radial,trace_stage,
            fixed_hac_lead_active,path_owner==g,g->hac_transition_lead_curve,
            g->hac_transition_lead_progress,g->hac_transition_progress,
            g->hac_transition_handoff_blend,g->hac_commit_blend,
            g->terminal_reference_aoa,g->hac_transition_handoff_aoa,
            path_owner->fixed_hac_lead_target_speed,
            fixed_hac_lead_remaining(path_owner),fixed_hac_lead_brake_aoa,
            path_remaining,hac_slope,fixed_reference_gamma,gamma_request,gamma,
            gate_gamma,energy_gamma,fixed_hac_required_lateral,
            fixed_hac_available_lateral,fixed_hac_lift_limit,
            fixed_hac_normal_limit);
    }
    return r;
}
