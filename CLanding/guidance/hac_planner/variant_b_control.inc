/* Variant B owns an explicit simulator-only pre-state.  It first searches for
   a feasible dynamic radius and its matching runway-anchored p0.  Until such
   a tuple exists, it may continue the bounded upstream energy segment; that
   staging distance is intentionally absent from any committed lead+arc
   reference budget. */
       GuidanceResult terminal_variant_b_upstream_guidance(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt,bool*capture_ready){
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    if(capture_ready)*capture_ready=false;
    g->terminal_test_upstream_capture_ready=false;
    g->terminal_test_upstream_energy_rejected=false;

    /* MM305 permits either runway-perpendicular inlet.  Choose the one already
       nearest the measured course instead of hardcoding the southbound branch;
       side +1 remains the right-turn convention for the HAC itself. */
    double positive_capture=norm_deg(cfg->site.runway_heading+90.0);
    double negative_capture=norm_deg(cfg->site.runway_heading-90.0);
    double capture_course=fabs(norm_signed_deg(positive_capture-course))<=
        fabs(norm_signed_deg(negative_capture-course))?
        positive_capture:negative_capture;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e=0.0,n=0.0;
    local_offsets(origin,current,p->radius,&e,&n);
    HACPoint2 p0={0},exit={0};
    HACTransitionPlan dynamic_plan={0};
    double planned_radius=NAN,authority_minimum_radius=INFINITY,
        energy_sustainable_radius=0.0,planned_entry_speed=NAN;
    bool dynamic_plan_valid=false,provisional_plan_valid=false;
    /* A forced fallback is frozen just like a certified tuple once selected.
       Fresh searches may still replace it with an ordinary-feasible tuple,
       but must not move the fallback p0 every planning cadence.  Once
       terminal_force_acquisition_with_aero() commits, this function is no
       longer called and the committed tuple is immutable. */
    HACTransitionPlan cached_plan={0};
    bool cached_tuple_valid=terminal_variant_b_load_tuple(g,&cached_plan);
    bool cached_production_valid=cached_tuple_valid&&
        g->terminal_test_upstream_plan_valid&&!g->terminal_test_upstream_provisional_plan_valid;
    double cached_radius=cached_tuple_valid?
        g->terminal_test_upstream_planned_radius:NAN;
    double cached_speed=cached_tuple_valid&&
        g->terminal_test_upstream_speed_target_valid?
        g->terminal_test_upstream_speed_target:NAN;
    bool fresh_provisional=false;
    HACTransitionPlan searched_plan={0};
    GuidanceMachine searched_profile={0};
    double searched_radius=NAN,searched_speed=NAN;
    /* The guidance loop still checks the current cached tuple at 10 Hz, but
       the bounded 98x12 candidate sweep is evaluated at the existing
       one-second diagnostic/planning cadence.  This keeps long simulator
       runs realtime-bounded without allowing a provisional tuple to suppress
       later state-dependent planning. */
    bool plan_due=!isfinite(g->terminal_test_upstream_last_plan_ut)||
        t->ut-g->terminal_test_upstream_last_plan_ut>=1.0;
    bool searched_dynamic=false;
    if(plan_due){
        g->terminal_test_upstream_last_plan_ut=t->ut;
        searched_dynamic=terminal_variant_b_find_dynamic_plan(g,t,course,p,
            aero,cfg,e,n,capture_course,&searched_plan,&searched_radius,
            &authority_minimum_radius,&energy_sustainable_radius,&searched_speed,
            &fresh_provisional,&searched_profile);
    }
    /* Keep advisory future probing only while no certified execution tuple
       exists. A production-valid cached tuple remains the operational lead
       reference until a fresh certified tuple replaces it or the p0 is missed;
       a later search failure must not erase a lead already being flown. */
    if(plan_due&&!searched_dynamic&&!cached_production_valid)
        (void)terminal_variant_b_preview_future_hac(g,t,course,p,aero,cfg,e,n,
            capture_course);

    if(plan_due&&searched_dynamic){
        /* A fresh production-valid result is the only pre-commit replacement
           allowed.  This is not a recapture: the path has not been committed
           and therefore has no executable HAC ownership to redraw. */
        dynamic_plan=searched_plan;
        planned_radius=searched_radius;
        planned_entry_speed=searched_speed;
        dynamic_plan_valid=true;
        terminal_variant_b_store_tuple(g,&dynamic_plan);
        terminal_variant_b_store_vertical_profile(g,&searched_profile);
    }else if(cached_tuple_valid){
        /* Preserve a previously certified tuple across later failed searches.
           The dynamic search may replace it only with another certified tuple;
           provisional results remain diagnostic and cannot turn a live finite
           lead into a moving/disappearing target. */
        dynamic_plan=cached_plan;
        planned_radius=cached_radius;
        planned_entry_speed=cached_speed;
        dynamic_plan_valid=cached_production_valid;
        provisional_plan_valid=!cached_production_valid;
    }else if(plan_due&&fresh_provisional&&searched_plan.valid){
        dynamic_plan=searched_plan;
        planned_radius=searched_radius;
        planned_entry_speed=searched_speed;
        provisional_plan_valid=true;
        terminal_variant_b_store_tuple(g,&dynamic_plan);
        terminal_variant_b_store_vertical_profile(g,&searched_profile);
    }else{
        g->terminal_test_upstream_tuple_valid=false;
        g->terminal_test_upstream_vertical_profile_valid=false;
        g->terminal_test_upstream_planned_radius=0.0;
        g->terminal_test_upstream_speed_target_valid=false;
        g->terminal_test_upstream_speed_target=0.0;
    }
    g->terminal_test_upstream_plan_valid=dynamic_plan_valid;
    g->terminal_test_upstream_provisional_plan_valid=provisional_plan_valid;
    g->terminal_test_upstream_planned_radius=(dynamic_plan_valid||
        provisional_plan_valid)?planned_radius:0.0;
    g->terminal_test_upstream_speed_target_valid=(dynamic_plan_valid||
        provisional_plan_valid)&&isfinite(planned_entry_speed)&&
        planned_entry_speed>0.0;
    g->terminal_test_upstream_speed_target=
        g->terminal_test_upstream_speed_target_valid?planned_entry_speed:0.0;
    /* If the feasible set is empty, fly the frozen best fallback instead of
       holding a neutral upstream state forever.  The tuple is latched above,
       so this does not reintroduce the old moving-p0 zig-zag. */
    bool execution_plan_valid=dynamic_plan_valid||provisional_plan_valid;
    bool path_plan_valid=execution_plan_valid;
    if(path_plan_valid){
        p0=dynamic_plan.p0;
        exit=dynamic_plan.p3;
    }else{
        /* No candidate above the 3 km floor currently has both authority and
           energy overlap.  Keep the upstream state neutral while the planner
           is allowed to search again; never turn the configured radius into a
           hidden staging target. */
        p0=(HACPoint2){e,n};
        exit=p0;
    }

    double tangent_e=sin(capture_course*DEG2RAD);
    double tangent_n=cos(capture_course*DEG2RAD);
    double right_e=cos(capture_course*DEG2RAD);
    double right_n=-sin(capture_course*DEG2RAD);
    double dx=e-p0.e,dy=n-p0.n;
    double along_to_p0=path_plan_valid?
        dx*tangent_e+dy*tangent_n:0.0;
    double cross_to_p0=path_plan_valid?
        dx*right_e+dy*right_n:0.0;
    double distance_to_p0=path_plan_valid?hypot(dx,dy):0.0;
    double course_error=path_plan_valid?
        fabs(norm_signed_deg(course-capture_course)):0.0;

    /* Once the tangent plane is behind the vehicle by a meaningful amount,
       the one-way fixture has missed p0.  Do not hand authority to the normal
       terminal planner or synthesize a replacement path. */
    bool missed=path_plan_valid&&(along_to_p0>250.0||
        (along_to_p0>0.0&&fabs(cross_to_p0)>600.0));
    if(missed){
        char reason[256];
        snprintf(reason,sizeof(reason),
            "MM305 variant-B staging missed fixed tangent p0: remaining %.0f m, along-to-p0 %+0.f m, cross %+0.f m.",
            distance_to_p0,along_to_p0,cross_to_p0);
        return terminal_abort(g,reason);
    }

    /* MM305 ownership is already latched while Variant-B stages toward p0.
       Do not re-apply the MM305 entry-altitude floor here: the 18.5 km
       regression reached p0 at ~8.2 km with valid geometry and authority, then
       missed solely because this gate still required >= mm305_min_altitude. */
    bool dynamic_valid=isfinite(t->mean_altitude)&&
        t->mean_altitude>cfg->site.altitude&&
        t->mean_altitude<=s->mm305_max_altitude&&
        isfinite(t->true_air_speed)&&t->true_air_speed>v->minimum_safe_speed*1.20&&
        isfinite(t->dynamic_pressure)&&
        t->dynamic_pressure<=v->maximum_dynamic_pressure*1.05;
    bool vertical_valid=isfinite(t->flight_path_angle)&&
        t->flight_path_angle<-1.0&&t->flight_path_angle>-35.0;
    double capture_required=NAN,capture_available=NAN,capture_minimum=INFINITY;
    bool capture_authority=path_plan_valid&&
        terminal_fixed_hac_authority(t,p,aero,v,s,planned_radius,
            &capture_required,&capture_available,&capture_minimum);
    bool ready=path_plan_valid&&distance_to_p0<=500.0&&
        fabs(cross_to_p0)<=350.0&&course_error<=5.0&&dynamic_valid&&
        vertical_valid&&(capture_authority||provisional_plan_valid);
    g->terminal_test_upstream_capture_ready=ready;
    if(capture_ready)*capture_ready=ready;

    double staging_radius=path_plan_valid?planned_radius:NAN;
    double speed=fmax(1.0,guidance_lateral_speed(t));
    double tracking_distance=fmax(6000.0,distance_to_p0);
    double tracking_time=clampd(.22*tracking_distance/speed,7.0,14.0);
    HACGuidance path={0};
    double staging_intercept_deg=NAN;
    if(path_plan_valid){
        path=terminal_variant_b_latched_lead_guidance(&dynamic_plan,t,
            planet_surface_gravity(p),course,staging_radius,e,n);
    }else{
        /* With no admissible tuple there is no p0 to pursue.  The optional
           staging experiment holds a frozen runway-parallel line through the
           current cross-track position while the normal planner continues to
           search.  It has no destination and never writes HAC geometry,
           progress, accounting, or commit state. */
        double runway_heading=norm_deg(cfg->site.runway_heading);
        double parallel_error=fmin(
            fabs(norm_signed_deg(course-runway_heading)),
            fabs(norm_signed_deg(course-norm_deg(runway_heading+180.0))));
        bool staging_line=hac_upstream_runway_staging_requested()&&
            isfinite(parallel_error)&&parallel_error<=15.0&&
            !g->terminal_test_upstream_staging_line_active&&
            !g->terminal_path_committed&&!g->hac_transition_active&&
            !g->final_approach_captured;
        if(staging_line){
            g->terminal_test_upstream_staging_line_active=true;
            g->terminal_test_upstream_staging_y=t->runway_cross_track;
            g->terminal_test_upstream_staging_course=course;
            g->terminal_test_upstream_staging_epoch_ut=t->ut;
        }
        bool staging_active=hac_upstream_runway_staging_requested()&&
            isfinite(parallel_error)&&parallel_error<=15.0&&
            g->terminal_test_upstream_staging_line_active&&
            isfinite(g->terminal_test_upstream_staging_y)&&
            isfinite(g->terminal_test_upstream_staging_course)&&
            !g->terminal_path_committed&&!g->hac_transition_active&&
            !g->final_approach_captured;
        if(staging_active){
            double cross_error=t->runway_cross_track-
                g->terminal_test_upstream_staging_y;
            double look=clampd(fmax(speed*5.5,1500.0),700.0,5200.0);
            double intercept=clampd(atan2(cross_error,look)*RAD2DEG,
                -15.0,15.0);
            staging_intercept_deg=intercept;
            path.heading=norm_deg(g->terminal_test_upstream_staging_course+
                intercept);
            path.course_error=norm_signed_deg(path.heading-course);
            double lateral=2.0*speed*speed/fmax(look,1.0)*
                sin(path.course_error*DEG2RAD);
            double rate_cap=fmin(1.35,hac_course_rate_cap(t))*DEG2RAD;
            lateral=clampd(lateral,-speed*rate_cap,speed*rate_cap);
            path.lateral_acceleration=lateral;
            path.bank=clampd(atan2(lateral,
                fmax(planet_surface_gravity(p),.01))*RAD2DEG,-20.0,20.0);
        }else{
            if(!hac_upstream_runway_staging_requested())
                g->terminal_test_upstream_staging_line_active=false;
            path.heading=course;
            path.course_error=0.0;
            path.lateral_acceleration=0.0;
            path.bank=0.0;
        }
    }
    if(!isfinite(path.heading)||fabs(path.heading)>360.0){
        path.heading=capture_course;
        path.course_error=norm_signed_deg(capture_course-course);
        path.lateral_acceleration=hac_frenet_lateral(speed,0.0,
            clampd(cross_to_p0,-6000.0,6000.0),path.course_error,tracking_time);
        path.bank=isfinite(staging_radius)?
            taem_bank_demand(t,&path,staging_radius,1.0,aero,v):0.0;
        if(!isfinite(path.bank))
            path.bank=clampd(atan2(path.lateral_acceleration,
                fmax(planet_surface_gravity(p),.01))*RAD2DEG,-45.0,45.0);
    }

    double lead_target_speed=NAN;
    if(path_plan_valid){
        /* A provisional tuple is an execution reference only, but it still
           needs a finite lead target.  Cached diagnostic tuples can predate
           the speed-target latch (and the target is never an admission
           signal), so fall back to the measured-radius target whenever the
           cached value is absent or invalid. */
        bool latched_target_valid=
            g->terminal_test_upstream_speed_target_valid&&
            isfinite(g->terminal_test_upstream_speed_target)&&
            g->terminal_test_upstream_speed_target>0.0;
        lead_target_speed=dynamic_plan_valid&&latched_target_valid?
            g->terminal_test_upstream_speed_target:
            fixed_hac_lead_target_speed(t,aero,v,staging_radius);
    }
    double authority_required=NAN,authority_available=NAN;
    bool authority_valid=dynamic_plan_valid&&
        terminal_fixed_hac_authority(t,p,aero,v,s,staging_radius,
            &authority_required,&authority_available,&authority_minimum_radius);
    /* Variant-B staging is allowed to preserve a useful entry speed for the
       already-frozen HAC, but the floor is derived from the same measured
       lateral authority used by the normal gate.  It is not an admission
       decision and it never grants authority to the committed path. */
    double authority_speed=authority_available>0.0&&isfinite(authority_available)?
        sqrt(authority_available*staging_radius)*.97:NAN;
    double staging_speed_floor=fmax(v->final_approach_speed*2.70,
        v->minimum_safe_speed*3.50);
    /* The latched lead target is the diagnostic baseline and must never be
       reduced by this optional preparation heuristic.  Authority can only
       authorize raising that target to the staging floor; if the measured
       state cannot support the floor, leave the original target untouched
       and let the normal arbitration remain observable. */
    bool floor_authorized=isfinite(authority_speed)&&
        authority_speed>=staging_speed_floor;
    /* authority_speed is an upper supportable circle speed, not a lower
       commanded speed.  Once a production-valid Variant-B tuple is frozen,
       its lead target is part of the certified state prediction and must not
       be replaced by this optional preservation heuristic. */
    bool speed_floor_active=!dynamic_plan_valid&&
        isfinite(lead_target_speed)&&floor_authorized&&
        staging_speed_floor>lead_target_speed+0.5;
    double target_speed=speed_floor_active?staging_speed_floor:
        lead_target_speed;
    /* The planner certified the delivered p0 state against one frozen vertical
       profile over lead+arc.  Execute that same remaining-path profile here;
       the old generic outer-final slope made live staging arrive kilometres
       below and tens of m/s slower than the state used by admission. */
    GuidanceMachine upstream_profile={0};
    bool upstream_profile_valid=path_plan_valid&&
        terminal_variant_b_load_vertical_profile(g,&upstream_profile);
    double profile_lead_remaining=path_plan_valid&&isfinite(path.arc_remaining)?
        fmax(0.0,path.arc_remaining):0.0;
    double profile_remaining=path_plan_valid?
        fmax(0.0,dynamic_plan.cone_arc_length)+profile_lead_remaining:NAN;
    double profile_slope=upstream_profile_valid?
        fixed_hac_reference_slope(&upstream_profile,profile_remaining):NAN;
    double profile_altitude=upstream_profile_valid?
        fixed_hac_reference_altitude(&upstream_profile,profile_remaining):NAN;
    double slope=isfinite(profile_slope)?profile_slope:
        terminal_outer_glide_slope(g,s);
    double fpa_aoa=terminal_fpa_force_aoa(t,p,aero,v,-slope);
    double drag_aoa=isfinite(target_speed)?fixed_hac_lead_drag_aoa(t,aero,v,
        target_speed,fmax(100.0,distance_to_p0),slope):NAN;
    double energy_aoa=isfinite(drag_aoa)?drag_aoa:terminal_hac_energy_aoa(v);
    double vertical_aoa=fmax(isfinite(fpa_aoa)?fpa_aoa:0.0,
        isfinite(drag_aoa)?drag_aoa:0.0);
    double energy_aoa_request=fmax(isfinite(energy_aoa)?energy_aoa:0.0,
        isfinite(drag_aoa)?drag_aoa:0.0);
    bool speed_near_target=isfinite(lead_target_speed)&&
        isfinite(t->true_air_speed)&&
        t->true_air_speed<lead_target_speed+45.0;
    /* Keep live lead execution identical to terminal_fixed_hac_lead_reachable:
       close to the target speed the certified propagator chooses the LOWER of
       the FPA incidence and drag incidence so it does not burn away the very
       energy it expects to deliver at p0. */
    double planner_lead_aoa=speed_near_target?
        fmin(isfinite(fpa_aoa)?fpa_aoa:energy_aoa,
            fmax(energy_aoa,isfinite(drag_aoa)?drag_aoa:0.0)):
        fmax(isfinite(fpa_aoa)?fpa_aoa:0.0,
            isfinite(drag_aoa)?drag_aoa:0.0);
    /* Once staging has spent through the latched speed target, the old
       max(FPA AoA, drag AoA) arbitration kept commanding 19--24 deg merely
       to hold the steep instantaneous FPA.  That increased drag, made the
       speed shortfall worse, and left no energy-radius candidate at p0.
       Preserve vertical validity, but let the measured energy request own
       incidence while speed is below target.  When speed is above target the
       existing vertical/drag maximum still dissipates the surplus. */
    double speed_shortfall=isfinite(lead_target_speed)&&
        isfinite(t->true_air_speed)?
        fmax(0.0,lead_target_speed-t->true_air_speed):0.0;
    /* Below the latched target, the nominal terminal energy trim is a ceiling,
       not a required incidence.  Reduce it only in proportion to measured
       shortfall, with a bounded floor so the vehicle still has a controllable
       aerodynamic attitude while the FPA is allowed to steepen. */
    double unload_relief=clampd(speed_shortfall*.08,0.0,6.0);
    if(speed_shortfall>0.0)
        energy_aoa_request=fmax(.5,energy_aoa_request-unload_relief);
    double vertical_floor=.5;
    /* When the search has found authority-valid provisional tuples but none
       has passed the energy certificate, the fixed 10 deg fallback is the
       wrong upstream owner at a shallow high-altitude state.  It keeps lift
       and drag high, delays the descent into denser air, and the live audit
       records avoidable dissipation even though fpa_aoa is zero.  This is
       conditioning only: it does not create a plan, alter the normal gate,
       or provide a p0/reference for execution.  Require the provisional
       authority evidence and a recoverable vertical state; otherwise retain
       the established energy fallback. stabilized() remains the sole
       executable AoA limiter.
     */
    bool provisional_energy_conditioning=provisional_plan_valid&&
        !dynamic_plan_valid&&vertical_valid&&
        isfinite(t->true_air_speed)&&
        t->true_air_speed>v->minimum_safe_speed*1.20&&
        isfinite(t->dynamic_pressure)&&t->dynamic_pressure>0.0;
    double conditioning_aoa=clampd(fmax(vertical_floor,
        isfinite(fpa_aoa)?fpa_aoa:vertical_floor),0.0,
        v->maximum_angle_of_attack);
    /* An authority-valid but energy-invalid provisional tuple is a
       feasibility sensor, not permission to spend unlimited drag holding the
       instantaneous FPA.  Keep the existing vertical request as an upper
       bound in this branch; stabilized() still applies the actuator slew and
       attitude limits.  This changes incidence only, never the provisional
       geometry or the production gate. */
    if(provisional_energy_conditioning&&isfinite(energy_aoa_request))
        conditioning_aoa=fmin(conditioning_aoa,
            clampd(energy_aoa_request,vertical_floor,
                v->maximum_angle_of_attack));
    const double upstream_aoa_bias=provisional_energy_conditioning?
        hac_upstream_aoa_bias_deg():0.0;
    if(provisional_energy_conditioning&&upstream_aoa_bias>0.0)
        conditioning_aoa=clampd(conditioning_aoa+upstream_aoa_bias,
            vertical_floor,v->maximum_angle_of_attack);
    double target_aoa=(speed_floor_active?energy_aoa:
        (provisional_energy_conditioning?conditioning_aoa:
        (dynamic_plan_valid&&path_plan_valid?planner_lead_aoa:
        (!dynamic_plan_valid?energy_aoa:
        (speed_near_target?
            fmin(fmax(vertical_aoa,vertical_floor),energy_aoa_request):
            vertical_aoa)))));
    if(!isfinite(target_aoa))
        target_aoa=terminal_hac_energy_aoa(v);
    else if(target_aoa<=0.0)
        target_aoa=dynamic_plan_valid&&path_plan_valid?
            vertical_floor:terminal_hac_energy_aoa(v);
    target_aoa=clampd(target_aoa,0.0,v->maximum_angle_of_attack);

    bool runway_staging_active=!path_plan_valid&&
        g->terminal_test_upstream_staging_line_active&&
        hac_upstream_runway_staging_requested();
    g->terminal_reference_heading=path_plan_valid?capture_course:
        (runway_staging_active?path.heading:course);
    g->terminal_reference_bank=path.bank;
    g->terminal_reference_aoa=target_aoa;
    char status[320];
    snprintf(status,sizeof(status),
        "MM305 variant-B upstream staging: p0 remaining %.1f m, along %+0.1f m, cross %+0.1f m, R %.0f m, speed %.1f / %.1f m/s, AoA %.1f deg%s%s.",
        distance_to_p0,along_to_p0,cross_to_p0,staging_radius,t->true_air_speed,
        target_speed,target_aoa,dynamic_plan_valid?"; dynamic tuple selected":
        (provisional_plan_valid?"; provisional geometry selected; commit gate closed":
        "; CRITICAL: no feasible normal HAC tuple; holding upstream search"),
        runway_staging_active?"; runway-parallel staging line active":"");
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    GuidanceCommand command=atmospheric(t,path.heading,path.bank,v,0.0,
        runway_staging_active,PROFILE_TAEM);
    command.heading_control_enabled=runway_staging_active;
    command.has_target_aoa=true;
    command.target_aoa=target_aoa;
    command.target_pitch=t->flight_path_angle+target_aoa;
    GuidanceResult result=stabilized(g,result_make(PHASE_TAEM,command,status,
        ready?"Variant-B is at the measured-authority admission window; the original gate remains decisive.":
        "Variant-B is flying the upstream energy-management segment; no committed HAC distance is counted yet."),
        t,v,s,dt);
    double arbitration_loss_rate=0.0;
    double selected_drag=fixed_hac_projected_drag_accel(t,aero,v,
        result.command.target_aoa);
    double energy_drag=fixed_hac_projected_drag_accel(t,aero,v,energy_aoa);
    /* Count only the narrow case in which the selected command exceeded the
       energy request while the existing FPA allocator did not require more
       incidence. This audits avoidable dissipation; it does not override
       vertical validity, the normal gate, or stabilized(). */
    if(isfinite(selected_drag)&&isfinite(energy_drag)&&
       isfinite(result.command.target_aoa)&&isfinite(energy_aoa)&&
       result.command.target_aoa>energy_aoa+.05&&
       (!isfinite(fpa_aoa)||fpa_aoa<=energy_aoa+.05))
        arbitration_loss_rate=fmax(0.0,selected_drag-energy_drag);
    if(isfinite(arbitration_loss_rate)&&arbitration_loss_rate>0.0)
        g->terminal_test_upstream_avoidable_aoa_energy_loss+=
            arbitration_loss_rate*fmax(0.0,dt);
    g->terminal_test_upstream_last_avoidable_aoa_loss=arbitration_loss_rate;
    if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
       t->ut-g->terminal_test_upstream_last_log_ut>=1.0){
        double specific_energy=rotating_specific_energy(t->latitude,
            t->mean_altitude,t->true_air_speed,p);
        /* The invariant is about the selected pre-limiter candidate.  The
           stabilized output may remain above it while the existing slew
           limiter safely converges; that is expected actuator behavior, not
           an arbitration failure. */
        bool arbitration_violation=speed_floor_active&&isfinite(fpa_aoa)&&
            isfinite(energy_aoa)&&fpa_aoa>energy_aoa+.05&&
            target_aoa>energy_aoa+.05;
        if(runway_staging_active)
            fprintf(stderr,"STAGING_GUIDANCE ut=%.3f runwayCrossTrack=%.3f stageY=%.3f crossError=%.3f measuredCourse=%.3f chiBase=%.3f interceptDeg=%.3f targetHeading=%.3f headingError=%.3f lateralAccel=%.3f targetRoll=%.3f\n",
                t->ut,t->runway_cross_track,
                g->terminal_test_upstream_staging_y,
                t->runway_cross_track-g->terminal_test_upstream_staging_y,
                course,g->terminal_test_upstream_staging_course,
                staging_intercept_deg,path.heading,path.course_error,
                path.lateral_acceleration,path.bank);
        if(path_plan_valid)
            fprintf(stderr,
                "HAC_STAGE_PROFILE ut=%.3f valid=%d remaining=%.1f leadRemaining=%.1f desiredAlt=%.1f actualAlt=%.1f altError=%+.1f slope=%.3f actualFpa=%.3f p0TargetSpeed=%.3f actualSpeed=%.3f\n",
                t->ut,upstream_profile_valid?1:0,profile_remaining,
                profile_lead_remaining,profile_altitude,t->mean_altitude,
                isfinite(profile_altitude)?profile_altitude-t->mean_altitude:NAN,
                slope,t->flight_path_angle,lead_target_speed,t->true_air_speed);
        if(path_plan_valid)
            fprintf(stderr,
                "HAC_STAGE_ARBITRATION ut=%.3f dynamic=%d nearTarget=%d fpaAoA=%.3f dragAoA=%.3f plannerLeadAoA=%.3f selectedAoA=%.3f stabilizedAoA=%.3f\n",
                t->ut,dynamic_plan_valid?1:0,speed_near_target?1:0,
                fpa_aoa,drag_aoa,planner_lead_aoa,target_aoa,
                result.command.target_aoa);
        fprintf(stderr,"HAC_STAGE ut=%.3f variant=B p0=(%.1f,%.1f) exit=(%.1f,%.1f) remaining=%.1f alongToP0=%+.1f crossToP0=%+.1f heading=%.3f headingError=%.3f speed=%.3f stagingR=%.1f dynamicPlan=%d provisionalPlan=%d candidateEnergyR=%.1f leadTargetSpeed=%.3f targetSpeed=%.3f authoritySpeed=%.3f speedFloor=%.3f floorAuthorized=%d speedFloorActive=%d aoa=%.3f aoaBias=%.3f fpaAoA=%.3f energyAoA=%.3f stabilizedAoA=%.3f fpa=%.3f fpaTarget=%.3f fpaConditioning=%d bank=%.3f specificEnergy=%.3f ready=%d dynamic=%d vertical=%d arbitrationViolation=%d authorityValid=%d authorityRequired=%.3f authorityAvailable=%.3f authorityMinR=%.1f arbitrationLoss=%.5f arbitrationLossTotal=%.3f terminalContractValid=%d terminalEvaluationValid=%d terminalBlock=%s terminalPathCommitted=%d leadArcBudget=excluded\n",
            t->ut,p0.e,p0.n,exit.e,exit.n,distance_to_p0,along_to_p0,
            cross_to_p0,course,course_error,t->true_air_speed,staging_radius,
            dynamic_plan_valid,provisional_plan_valid,energy_sustainable_radius,lead_target_speed,
            target_speed,authority_speed,staging_speed_floor,
            floor_authorized,speed_floor_active,target_aoa,upstream_aoa_bias,fpa_aoa,energy_aoa,
            result.command.target_aoa,t->flight_path_angle,t->flight_path_angle,
            0,path.bank,
            specific_energy,ready,dynamic_valid,vertical_valid,
            arbitration_violation,authority_valid,authority_required,
            authority_available,authority_minimum_radius,
            arbitration_loss_rate,
            g->terminal_test_upstream_avoidable_aoa_energy_loss,
            g->taem_exec.terminal_contract.valid?1:0,
            g->taem_exec.terminal_evaluation.valid?1:0,
            taem_terminal_block_reason_string(g->taem_exec.terminal_evaluation.block_reason),
            g->taem_exec.terminal_contract.path_committed?1:0);
        g->terminal_test_upstream_last_log_ut=t->ut;
    }
    return result;
}


