
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

    /*
     * Both runway-perpendicular HAC inlets are physically valid candidates.
     * Current heading is not a reason to delete one side from the search: at
     * high energy the initially longer turn can become the only lead whose
     * delivered p0 state has both authority and energy margin.
     */
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
    bool dynamic_plan_valid=false;
    bool advisory_plan_valid=false;

    HACTransitionPlan cached_plan={0};
    bool cached_tuple_valid=terminal_variant_b_load_tuple(g,&cached_plan);
    bool cached_production_valid=cached_tuple_valid&&
        g->terminal_test_upstream_plan_valid&&
        !g->terminal_test_upstream_provisional_plan_valid;
    bool cached_provisional_valid=cached_tuple_valid&&
        !g->terminal_test_upstream_plan_valid&&
        g->terminal_test_upstream_provisional_plan_valid;
    double cached_radius=cached_tuple_valid?
        g->terminal_test_upstream_planned_radius:NAN;
    double cached_speed=cached_tuple_valid&&
        g->terminal_test_upstream_speed_target_valid?
        g->terminal_test_upstream_speed_target:NAN;
    if(cached_tuple_valid&&isfinite(g->terminal_test_upstream_staging_course))
        capture_course=norm_deg(g->terminal_test_upstream_staging_course);

    bool fresh_provisional=false;
    HACTransitionPlan searched_plan={0};
    GuidanceMachine searched_profile={0};
    double searched_radius=NAN,searched_speed=NAN;
    double searched_capture_course=NAN;

    /*
     * Search the two outer-HAC sides under identical force, vertical, energy,
     * and finite-lead certificates.  A fully certified tuple always outranks a
     * provisional one.  When both sides certify, choose the shorter complete
     * lead+HAC+final path; there is no heading-side preference.
     */
    bool plan_due=!isfinite(g->terminal_test_upstream_last_plan_ut)||
        t->ut-g->terminal_test_upstream_last_plan_ut>=1.0;
    bool searched_dynamic=false;
    if(plan_due){
        g->terminal_test_upstream_last_plan_ut=t->ut;
        double capture_candidates[2]={positive_capture,negative_capture};
        bool have_dynamic=false,have_provisional=false;
        double best_dynamic_path=INFINITY,best_provisional_path=INFINITY;
        HACTransitionPlan provisional_plan={0};
        GuidanceMachine provisional_profile={0};
        double provisional_radius=NAN,provisional_speed=NAN,
            provisional_authority_min=INFINITY,provisional_energy_radius=0.0,
            provisional_capture_course=NAN;

        for(size_t inlet=0;inlet<2;inlet++){
            HACTransitionPlan candidate_plan={0};
            GuidanceMachine candidate_profile={0};
            double candidate_radius=NAN,candidate_authority_min=INFINITY,
                candidate_energy_radius=0.0,candidate_speed=NAN;
            bool candidate_provisional=false;
            bool candidate_dynamic=terminal_variant_b_find_dynamic_plan(
                g,t,course,p,aero,cfg,e,n,capture_candidates[inlet],
                &candidate_plan,&candidate_radius,&candidate_authority_min,
                &candidate_energy_radius,&candidate_speed,
                &candidate_provisional,&candidate_profile);
            double total=candidate_plan.valid?
                candidate_plan.lead_length+candidate_plan.cone_arc_length+
                    fmax(0.0,s->final_approach_distance):INFINITY;

            if(candidate_dynamic&&isfinite(total)&&total<best_dynamic_path){
                best_dynamic_path=total;
                searched_plan=candidate_plan;
                searched_profile=candidate_profile;
                searched_radius=candidate_radius;
                searched_speed=candidate_speed;
                authority_minimum_radius=candidate_authority_min;
                energy_sustainable_radius=candidate_energy_radius;
                searched_capture_course=capture_candidates[inlet];
                have_dynamic=true;
            }else if(!candidate_dynamic&&candidate_provisional&&
                    candidate_plan.valid&&isfinite(total)&&
                    total<best_provisional_path){
                best_provisional_path=total;
                provisional_plan=candidate_plan;
                provisional_profile=candidate_profile;
                provisional_radius=candidate_radius;
                provisional_speed=candidate_speed;
                provisional_authority_min=candidate_authority_min;
                provisional_energy_radius=candidate_energy_radius;
                provisional_capture_course=capture_candidates[inlet];
                have_provisional=true;
            }
        }

        searched_dynamic=have_dynamic;
        if(!have_dynamic&&have_provisional){
            searched_plan=provisional_plan;
            searched_profile=provisional_profile;
            searched_radius=provisional_radius;
            searched_speed=provisional_speed;
            authority_minimum_radius=provisional_authority_min;
            energy_sustainable_radius=provisional_energy_radius;
            searched_capture_course=provisional_capture_course;
            fresh_provisional=true;
        }
    }

    if(plan_due&&!searched_dynamic&&!fresh_provisional&&
       !cached_production_valid&&!cached_provisional_valid){
        bool previewed=terminal_variant_b_preview_future_hac(g,t,course,p,aero,cfg,
            e,n,positive_capture);
        if(!previewed)
            (void)terminal_variant_b_preview_future_hac(g,t,course,p,aero,cfg,
                e,n,negative_capture);
    }

    if(plan_due&&searched_dynamic){
        dynamic_plan=searched_plan;
        planned_radius=searched_radius;
        planned_entry_speed=searched_speed;
        capture_course=searched_capture_course;
        dynamic_plan_valid=true;
        advisory_plan_valid=true;
        terminal_variant_b_store_tuple(g,&dynamic_plan);
        terminal_variant_b_store_vertical_profile(g,&searched_profile);
        g->terminal_test_upstream_staging_course=capture_course;
    }else if(cached_production_valid){
        dynamic_plan=cached_plan;
        planned_radius=cached_radius;
        planned_entry_speed=cached_speed;
        dynamic_plan_valid=true;
        advisory_plan_valid=true;
        if(isfinite(g->terminal_test_upstream_staging_course))
            capture_course=norm_deg(g->terminal_test_upstream_staging_course);
    }else if(cached_provisional_valid){
        /*
         * A provisional tuple owns no HAC admission, but its live-state
         * acquisition lead is still a continuous, authority-bounded steering
         * reference. Preserve it between planner frames instead of redrawing a
         * shrinking pseudo-HAC every tick.
         */
        dynamic_plan=cached_plan;
        planned_radius=cached_radius;
        planned_entry_speed=cached_speed;
        advisory_plan_valid=true;
        if(isfinite(g->terminal_test_upstream_staging_course))
            capture_course=norm_deg(g->terminal_test_upstream_staging_course);
    }else if(plan_due&&fresh_provisional){
        dynamic_plan=searched_plan;
        planned_radius=searched_radius;
        planned_entry_speed=searched_speed;
        capture_course=searched_capture_course;
        advisory_plan_valid=true;
        terminal_variant_b_store_tuple(g,&dynamic_plan);
        terminal_variant_b_store_vertical_profile(g,&searched_profile);
        g->terminal_test_upstream_staging_course=capture_course;
    }else{
        g->terminal_test_upstream_tuple_valid=false;
        g->terminal_test_upstream_vertical_profile_valid=false;
        g->terminal_test_upstream_planned_radius=0.0;
        g->terminal_test_upstream_speed_target_valid=false;
        g->terminal_test_upstream_speed_target=0.0;
        g->terminal_test_upstream_staging_course=NAN;
    }

    g->terminal_test_upstream_plan_valid=dynamic_plan_valid;
    g->terminal_test_upstream_provisional_plan_valid=
        advisory_plan_valid&&!dynamic_plan_valid;
    g->terminal_test_upstream_planned_radius=
        advisory_plan_valid?planned_radius:0.0;
    g->terminal_test_upstream_speed_target_valid=dynamic_plan_valid&&
        isfinite(planned_entry_speed)&&planned_entry_speed>0.0;
    g->terminal_test_upstream_speed_target=
        g->terminal_test_upstream_speed_target_valid?planned_entry_speed:0.0;

    bool path_plan_valid=dynamic_plan_valid;
    bool steering_plan_valid=advisory_plan_valid;
    if(steering_plan_valid){
        p0=dynamic_plan.p0;
        exit=dynamic_plan.p3;
    }else{
        p0=(HACPoint2){e,n};
        exit=p0;
    }

    double tangent_e=sin(capture_course*DEG2RAD);
    double tangent_n=cos(capture_course*DEG2RAD);
    double right_e=cos(capture_course*DEG2RAD);
    double right_n=-sin(capture_course*DEG2RAD);
    double dx=e-p0.e,dy=n-p0.n;
    double along_to_p0=steering_plan_valid?
        dx*tangent_e+dy*tangent_n:0.0;
    double cross_to_p0=steering_plan_valid?
        dx*right_e+dy*right_n:0.0;
    double distance_to_p0=steering_plan_valid?hypot(dx,dy):0.0;
    double course_error=steering_plan_valid?
        fabs(norm_signed_deg(course-capture_course)):0.0;

    /* MM305 ownership is already latched while Variant-B stages toward p0.
       Do not re-apply the MM305 entry-altitude floor here; the finite lead
       already certified the delivered state. */
    double operational_floor=fmax(v->minimum_safe_speed,v->final_approach_speed);
    bool dynamic_valid=isfinite(t->mean_altitude)&&
        t->mean_altitude>cfg->site.altitude&&
        t->mean_altitude<=s->mm305_max_altitude&&
        isfinite(t->true_air_speed)&&t->true_air_speed>=operational_floor&&
        isfinite(t->dynamic_pressure)&&
        t->dynamic_pressure<=v->maximum_dynamic_pressure;
    bool vertical_valid=isfinite(t->flight_path_angle);
    double capture_required=NAN,capture_available=NAN,capture_minimum=INFINITY;
    bool capture_authority=path_plan_valid&&
        terminal_fixed_hac_authority(g,t,p,aero,v,s,planned_radius,
            &capture_required,&capture_available,&capture_minimum);
    bool sampled_p0_capture=path_plan_valid&&
        terminal_variant_b_p0_capture_ready(g,t,course,p,aero,cfg,
            &dynamic_plan,planned_radius,e,n);
    bool ready=path_plan_valid&&sampled_p0_capture&&dynamic_valid&&
        vertical_valid&&capture_authority;

    /* A p0 is missed only after the tangent plane is farther behind than the
       vehicle can traverse/turn in one configured guidance frame. */
    double frame_dt=s->guidance_rate>DBL_MIN?1.0/s->guidance_rate:NAN;
    double frame_allowance=isfinite(frame_dt)&&isfinite(capture_available)?
        guidance_lateral_speed(t)*frame_dt+
            0.5*fmax(0.0,capture_available)*frame_dt*frame_dt:NAN;
    bool missed=path_plan_valid&&!ready&&isfinite(frame_allowance)&&
        along_to_p0>frame_allowance;
    if(missed){
        char reason[256];
        snprintf(reason,sizeof(reason),
            "MM305 variant-B staging missed fixed tangent p0: remaining %.0f m, along-to-p0 %+0.f m, cross %+0.f m.",
            distance_to_p0,along_to_p0,cross_to_p0);
        return terminal_abort(g,reason);
    }
    g->terminal_test_upstream_capture_ready=ready;
    if(capture_ready)*capture_ready=ready;

    double staging_radius=steering_plan_valid?planned_radius:NAN;
    double speed=fmax(DBL_MIN,guidance_lateral_speed(t));
    double tracking_distance=steering_plan_valid?
        fmax(6000.0,distance_to_p0):0.0;
    double tracking_time=steering_plan_valid?
        clampd(.22*tracking_distance/speed,7.0,14.0):0.0;
    double acquisition_path_remaining=NAN;
    HACGuidance path={0};
    double staging_intercept_deg=NAN;
    ControlAuthorityEnvelope acquisition_authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!acquisition_authority.valid||
       !(acquisition_authority.maximum_lateral_accel_mps2>DBL_MIN))
        return terminal_abort(g,
            "MM305 has no certified lateral authority for acquisition.");

    if(steering_plan_valid){
        /*
         * A certified tuple and a provisional tuple share the same continuous
         * live-state acquisition geometry.  Only the certified tuple may cross
         * the HAC ownership gate; a provisional tuple is steering evidence only.
         * Keeping this finite lead frozen prevents the old radius-shrink/redraw
         * loop from moving the target out from under the shuttle.
         */
        path=terminal_variant_b_latched_lead_guidance(&dynamic_plan,t,
            planet_surface_gravity(p),course,staging_radius,e,n);
        acquisition_path_remaining=
            (isfinite(path.arc_remaining)?fmax(0.0,path.arc_remaining):
                fmax(0.0,distance_to_p0))+
            fmax(0.0,dynamic_plan.cone_arc_length)+
            fmax(0.0,s->final_approach_distance);
        path.distance_final=distance_to_p0;
        staging_intercept_deg=path.course_error;

        if(!path_plan_valid&&isfinite(path.lateral_acceleration)){
            double cap=acquisition_authority.maximum_lateral_accel_mps2;
            path.lateral_acceleration=clampd(path.lateral_acceleration,-cap,cap);
            path.bank=taem_bank_demand(t,&path,
                fmax(staging_radius,1.0),
                path.lateral_acceleration>=0.0?1.0:-1.0,aero,v);
        }
    }else{
        /*
         * No HAC candidate has even a usable finite acquisition lead yet.  Do
         * not synthesize a new pseudo-HAC from the instantaneous turn radius.
         * That was the mechanism that grew to a huge circle, then shrank every
         * frame while the vehicle flew past KSC.  Instead close on one stable
         * runway-relative terminal station while the physical two-side search
         * continues.  The target course is the selected perpendicular inlet;
         * the acquisition lead comes only from measured control authority and
         * actuator response.
         */
        double live_minimum_radius=speed*speed/
            fmax(acquisition_authority.maximum_lateral_accel_mps2,DBL_MIN);
        staging_radius=fmax(live_minimum_radius,1.0);

        double target_along=0.0;
        double target_cross=0.0;
        double point_distance=hypot(
            target_along-t->runway_along_track,
            target_cross-t->runway_cross_track);
        double target_minimum_radius=INFINITY;
        double best_path=decision_target_path_length(g,t,course,
            target_along,target_cross,capture_course,
            acquisition_authority.maximum_lateral_accel_mps2,p,cfg,
            &target_minimum_radius);
        acquisition_path_remaining=isfinite(best_path)?
            best_path:fmax(point_distance,1.0);

        double response=fmax(0.0,
            acquisition_authority.control_response_time_s);
        double response_distance=speed*response;
        double acquisition_lead=isfinite(target_minimum_radius)?
            fmax(target_minimum_radius,response_distance):response_distance;
        if(!(acquisition_lead>DBL_MIN))
            acquisition_lead=fmax(point_distance,1.0);

        TaemInterfaceTarget handoff_target={0};
        handoff_target.valid=true;
        handoff_target.along_track=target_along;
        handoff_target.cross_track=target_cross;
        handoff_target.course=capture_course;
        handoff_target.acquisition_lead=acquisition_lead;

        path.heading=taem_interface_line_heading(
            t->runway_along_track,t->runway_cross_track,
            cfg->site.runway_heading,&handoff_target);
        path.course_error=norm_signed_deg(path.heading-course);
        staging_intercept_deg=path.course_error;

        double geometric_time=point_distance/speed;
        double steering_time=fmax(response,
            isfinite(geometric_time)?geometric_time:response);
        steering_time=fmax(steering_time,DBL_EPSILON);
        double requested_rate=fabs(path.course_error)*DEG2RAD/steering_time;
        double requested_lateral=speed*requested_rate;
        path.lateral_acceleration=copysign(
            fmin(requested_lateral,
                acquisition_authority.maximum_lateral_accel_mps2),
            path.course_error);
        path.bank=taem_bank_demand(t,&path,staging_radius,
            path.course_error>=0.0?1.0:-1.0,aero,v);
        path.arc_remaining=acquisition_path_remaining;
        path.distance_final=point_distance;
        g->terminal_test_upstream_staging_line_active=false;
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
        bool latched_target_valid=
            g->terminal_test_upstream_speed_target_valid&&
            isfinite(g->terminal_test_upstream_speed_target)&&
            g->terminal_test_upstream_speed_target>0.0;
        lead_target_speed=dynamic_plan_valid&&latched_target_valid?
            g->terminal_test_upstream_speed_target:
            fixed_hac_lead_target_speed(t,p,aero,v,staging_radius);
    }else{
        lead_target_speed=fixed_hac_lead_target_speed(
            t,p,aero,v,staging_radius);
    }

    double authority_required=NAN,authority_available=NAN;
    bool authority_valid=false;
    if(dynamic_plan_valid){
        authority_valid=terminal_fixed_hac_authority(g,t,p,aero,v,s,staging_radius,
            &authority_required,&authority_available,&authority_minimum_radius);
    }else{
        /*
         * A no-plan acquisition still has measured control authority.  Keep that
         * authority out of HAC admission, but use it to prevent the energy
         * controller from bleeding below the configured MM305 speed band.
         */
        ControlAuthorityEnvelope acquisition_speed_authority=
            decision_control_authority_envelope(g,t,p,cfg);
        if(acquisition_speed_authority.valid&&
           acquisition_speed_authority.maximum_lateral_accel_mps2>DBL_MIN){
            authority_available=
                acquisition_speed_authority.maximum_lateral_accel_mps2;
            authority_required=0.0;
            authority_minimum_radius=speed*speed/authority_available;
            authority_valid=true;
        }
    }

    double authority_speed=authority_available>0.0&&isfinite(authority_available)?
        sqrt(authority_available*staging_radius):NAN;
    double mach_target_speed=isfinite(t->speed_of_sound)&&
        t->speed_of_sound>DBL_MIN&&s->mm305_target_mach>0.0?
        s->mm305_target_mach*t->speed_of_sound:NAN;
    double staging_speed_floor=fmax(v->minimum_safe_speed,
        fmax(s->final_alignment_speed,
            isfinite(mach_target_speed)?mach_target_speed:0.0));
    bool floor_authorized=isfinite(authority_speed)&&
        authority_speed>=staging_speed_floor;
    bool speed_floor_active=!dynamic_plan_valid&&
        isfinite(lead_target_speed)&&floor_authorized&&
        staging_speed_floor>lead_target_speed+0.5;
    double target_speed=speed_floor_active?staging_speed_floor:
        lead_target_speed;

    /* The planner certified the delivered p0 state against one frozen vertical
       profile over lead+arc. Execute that same remaining-path profile once a
       production HAC tuple exists. */
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

    /*
     * Keep an energy-distance estimate for the uncommitted state, but do not
     * use the final/preflare altitude as its vertical target.
     */
    double uncommitted_downstream_path=0.0;
    if(!path_plan_valid){
        double turn_angle=fabs(norm_signed_deg(
            cfg->site.runway_heading-capture_course))*DEG2RAD;
        double reference_radius=s->hac_radius;
        if(isfinite(authority_minimum_radius)&&authority_minimum_radius>0.0)
            reference_radius=fmax(reference_radius,authority_minimum_radius);
        uncommitted_downstream_path=
            turn_angle*fmax(reference_radius,0.0)+
            fmax(0.0,s->final_approach_distance);
    }

    double slope=isfinite(profile_slope)?profile_slope:
        terminal_outer_glide_slope(g,s);
    if(!path_plan_valid&&isfinite(path.distance_final)&&
       path.distance_final>DBL_MIN){
        /*
         * Before HAC commitment, descend only toward the MM305/TAEM interface.
         * The previous preflare-target calculation commanded 25--35 degree dives
         * and destroyed the altitude/energy needed to acquire the HAC.
         */
        double interface_altitude=clampd(s->taem_interface_altitude,
            s->mm305_min_altitude,s->mm305_max_altitude);
        double height_to_interface=fmax(0.0,
            t->mean_altitude-interface_altitude);
        double required_slope=atan2(height_to_interface,
            path.distance_final)*RAD2DEG;
        /*
         * Do not command an energy-expensive pull-up merely because the current
         * glide is steeper than the geometric average.  Maintaining the measured
         * FPA requires zero commanded gamma acceleration; it keeps altitude loss
         * predictable while preserving the kinetic energy needed for the turn.
         * If the geometry needs a steeper glide, the geometric slope wins.
         */
        double current_descent_slope=isfinite(t->flight_path_angle)?
            fmax(0.0,-t->flight_path_angle):0.0;
        slope=fmax(required_slope,current_descent_slope);
    }
    /*
     * Allocate lift between vertical support and turning before asking for AoA.
     * Saturating bank first can make the lateral command self-defeating: near
     * max bank almost all lift is horizontal, FPA runs away, and the vehicle
     * spends the altitude that was supposed to buy the turn.  Derive the bank
     * ceiling from the same point-mass normal-force equation used by
     * terminal_fpa_force_aoa(), at the maximum currently legal incidence.
     */
    if(!path_plan_valid&&isfinite(path.bank)&&isfinite(slope)){
        double aoa_limit=v->maximum_angle_of_attack;
        if(t->mach<1.0&&isfinite(v->terminal_maximum_lift_angle_of_attack)&&
           v->terminal_maximum_lift_angle_of_attack>0.0)
            aoa_limit=fmin(aoa_limit,v->terminal_maximum_lift_angle_of_attack);
        double max_lift=terminal_projected_lift_accel_at_aoa(t,aero,v,
            fmax(0.0,aoa_limit));
        double radius=p->radius+t->mean_altitude;
        double gravity=(radius>DBL_MIN&&p->gravitational_parameter>0.0)?
            p->gravitational_parameter/(radius*radius):NAN;
        double gamma=t->flight_path_angle*DEG2RAD;
        double base=isfinite(gravity)?
            (gravity-t->horizontal_speed*t->horizontal_speed/fmax(radius,1.0))*
                cos(gamma):NAN;
        double pitch_response=decision_pitch_capture_time(t,aoa_limit);
        double desired_gamma_rate=0.0;
        if(isfinite(pitch_response)&&pitch_response>DBL_MIN)
            desired_gamma_rate=((-slope-t->flight_path_angle)*DEG2RAD)/
                pitch_response;
        double vertical_required=isfinite(base)?
            base+t->true_air_speed*desired_gamma_rate:NAN;
        if(isfinite(max_lift)&&max_lift>DBL_MIN&&
           isfinite(vertical_required)&&vertical_required>0.0){
            double effectiveness=isfinite(t->bank_effectiveness)&&
                t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
            double effective_cap=acos(clampd(vertical_required/max_lift,0.0,1.0));
            double bank_cap=effective_cap/fmax(effectiveness,DBL_MIN)*RAD2DEG;
            bank_cap=fmin(bank_cap,
                fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v)));
            path.bank=clampd(path.bank,-bank_cap,bank_cap);
            double lateral_cap=max_lift*sin(fabs(path.bank*effectiveness)*DEG2RAD);
            if(isfinite(lateral_cap))
                path.lateral_acceleration=copysign(
                    fmin(fabs(path.lateral_acceleration),fmax(0.0,lateral_cap)),
                    path.lateral_acceleration);
        }
    }
    double fpa_aoa=terminal_fpa_force_aoa(t,p,aero,v,-slope);

    /* The no-plan acquisition bank is a real lift-vector tilt, not a cosmetic
       heading request.  Solve the FPA incidence against the commanded bank so a
       large lateral correction does not silently remove most of the vertical
       lift and drive the orbiter below the HAC engagement band. */
    Telemetry banked_state=*t;
    banked_state.roll=path.bank;
    double banked_fpa_aoa=terminal_fpa_force_aoa(&banked_state,p,aero,v,-slope);
    double lateral_aoa=terminal_required_aoa_for_lateral_limit(t,aero,v,
        path.lateral_acceleration,path.bank,v->maximum_angle_of_attack);
    double energy_path_remaining=path_plan_valid?
        distance_to_p0:acquisition_path_remaining+uncommitted_downstream_path;
    double drag_aoa=isfinite(target_speed)&&
        isfinite(energy_path_remaining)&&energy_path_remaining>DBL_MIN?
        fixed_hac_lead_drag_aoa(t,p,aero,v,target_speed,
            energy_path_remaining,slope):NAN;
    double energy_aoa=isfinite(drag_aoa)?drag_aoa:terminal_hac_energy_aoa(t,v);
    double vertical_aoa=fmax(isfinite(fpa_aoa)?fpa_aoa:0.0,
        isfinite(drag_aoa)?drag_aoa:0.0);
    double energy_aoa_request=fmax(isfinite(energy_aoa)?energy_aoa:0.0,
        isfinite(drag_aoa)?drag_aoa:0.0);
    bool speed_not_overspeed=isfinite(lead_target_speed)&&
        isfinite(t->true_air_speed)&&
        t->true_air_speed<=lead_target_speed+45.0;
    /* Keep live lead execution identical to terminal_fixed_hac_lead_reachable:
       when it is no more than 45m/s above target speed, the certified propagator
       chooses the LOWER FPA/drag incidence to preserve energy for delivery at p0. */
    bool lead_vertical_recovery=t->flight_path_angle< -slope-1.0;
    double planner_lead_aoa=speed_not_overspeed&&!lead_vertical_recovery?
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
    /* An infeasible HAC candidate is diagnostic evidence only; it must not
       alter the executable aerodynamic command. */
    bool provisional_energy_conditioning=false;
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
    double acquisition_support_aoa=fmax(
        isfinite(banked_fpa_aoa)?banked_fpa_aoa:0.0,
        isfinite(lateral_aoa)?lateral_aoa:0.0);
    double target_aoa=(speed_floor_active?
        fmax(isfinite(energy_aoa)?energy_aoa:0.0,acquisition_support_aoa):
        (provisional_energy_conditioning?conditioning_aoa:
        (dynamic_plan_valid&&path_plan_valid?planner_lead_aoa:
        (!dynamic_plan_valid?fmax(energy_aoa_request,acquisition_support_aoa):
        (speed_not_overspeed?
            fmin(fmax(vertical_aoa,vertical_floor),energy_aoa_request):
            vertical_aoa)))));
    if(!isfinite(target_aoa))
        target_aoa=terminal_hac_energy_aoa(t,v);
    else if(target_aoa<=0.0)
        target_aoa=dynamic_plan_valid&&path_plan_valid?
            vertical_floor:terminal_hac_energy_aoa(t,v);
    target_aoa=clampd(target_aoa,0.0,v->maximum_angle_of_attack);

    bool runway_staging_active=false;
    g->terminal_reference_heading=path_plan_valid?capture_course:path.heading;
    g->terminal_reference_bank=path.bank;
    g->terminal_reference_aoa=target_aoa;
    char status[320];
    snprintf(status,sizeof(status),
        "MM305 variant-B upstream staging: p0 remaining %.1f m, along %+0.1f m, cross %+0.1f m, R %.0f m, speed %.1f / %.1f m/s, AoA %.1f deg%s%s.",
        distance_to_p0,along_to_p0,cross_to_p0,staging_radius,t->true_air_speed,
        target_speed,target_aoa,dynamic_plan_valid?"; dynamic tuple selected":
        "; no certified HAC tuple; flying uncommitted energy-acquisition geometry",
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
                "HAC_STAGE_ARBITRATION ut=%.3f dynamic=%d notOverspeed=%d fpaAoA=%.3f dragAoA=%.3f plannerLeadAoA=%.3f selectedAoA=%.3f stabilizedAoA=%.3f\n",
                t->ut,dynamic_plan_valid?1:0,speed_not_overspeed?1:0,
                fpa_aoa,drag_aoa,planner_lead_aoa,target_aoa,
                result.command.target_aoa);
        fprintf(stderr,"HAC_STAGE ut=%.3f variant=B p0=(%.1f,%.1f) exit=(%.1f,%.1f) remaining=%.1f alongToP0=%+.1f crossToP0=%+.1f heading=%.3f headingError=%.3f speed=%.3f stagingR=%.1f dynamicPlan=%d provisionalPlan=%d candidateEnergyR=%.1f leadTargetSpeed=%.3f targetSpeed=%.3f authoritySpeed=%.3f speedFloor=%.3f floorAuthorized=%d speedFloorActive=%d aoa=%.3f aoaBias=%.3f fpaAoA=%.3f energyAoA=%.3f stabilizedAoA=%.3f fpa=%.3f fpaTarget=%.3f fpaConditioning=%d bank=%.3f specificEnergy=%.3f ready=%d dynamic=%d vertical=%d arbitrationViolation=%d authorityValid=%d authorityRequired=%.3f authorityAvailable=%.3f authorityMinR=%.1f arbitrationLoss=%.5f arbitrationLossTotal=%.3f terminalContractValid=%d terminalEvaluationValid=%d terminalBlock=%s terminalPathCommitted=%d leadArcBudget=excluded\n",
            t->ut,p0.e,p0.n,exit.e,exit.n,distance_to_p0,along_to_p0,
            cross_to_p0,course,course_error,t->true_air_speed,staging_radius,
            dynamic_plan_valid,0,energy_sustainable_radius,lead_target_speed,
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
