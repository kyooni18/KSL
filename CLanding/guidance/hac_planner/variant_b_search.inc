/* A direct p0 capture is a discrete-guidance sampling question, not a
   hand-tuned 500 m / 5 deg box.  During one guidance frame the vehicle can
   travel V*dt and rotate its velocity vector by a_lat/V*dt.  Use the same
   KSP-calibrated HAC authority that certifies the circle. */
static bool terminal_variant_b_p0_capture_ready(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const HACTransitionPlan*plan,double radius,
        double current_e,double current_n){
    if(!g||!t||!p||!cfg||!plan||!plan->valid||!(radius>0.0)||
       !(cfg->guidance.guidance_rate>DBL_MIN))
        return false;
    double required=NAN,available=NAN,minimum=INFINITY;
    if(!terminal_fixed_hac_authority(g,t,p,aero,&cfg->vehicle,&cfg->guidance,
            radius,&required,&available,&minimum)||
       !(available>0.0))
        return false;
    double dt=1.0/cfg->guidance.guidance_rate;
    double speed=guidance_lateral_speed(t);
    if(!(speed>DBL_MIN)||!isfinite(course))return false;
    double position_limit=speed*dt+0.5*available*dt*dt;
    double course_limit=available/speed*dt*RAD2DEG;
    double distance=hypot(current_e-plan->p0.e,current_n-plan->p0.n);
    double course_error=fabs(norm_signed_deg(course-plan->lead_end_course));
    double position_eps=sqrt(DBL_EPSILON)*fmax(1.0,position_limit);
    double course_eps=sqrt(DBL_EPSILON)*fmax(1.0,course_limit);
    return isfinite(distance)&&isfinite(course_error)&&
        distance<=position_limit+position_eps&&
        course_error<=course_limit+course_eps;
}

/* Bound prospective HAC geometry by the distance the current stock-KSP
   vehicle can actually glide before reaching its real operational speed floor.
   The integration uses the atmosphere table and calibrated drag model. */
static double terminal_variant_b_reachable_ground_range(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!(p->radius>0.0))return NAN;
    double floor_speed=fmax(cfg->vehicle.minimum_safe_speed,
        cfg->vehicle.final_approach_speed);
    double lo=0.0,hi=LANDER_PI*p->radius;
    for(unsigned i=0;i<(unsigned)(2*DBL_MANT_DIG);i++){
        double mid=0.5*(lo+hi);
        UnpoweredPathProjection projection=decision_unpowered_path_projection(
            g,t,p,cfg,cfg->site.altitude,mid);
        bool reachable=projection.valid&&
            isfinite(projection.terminal_speed_mps)&&
            projection.terminal_speed_mps>=floor_speed;
        if(reachable)lo=mid;else hi=mid;
        if(hi-lo<=sqrt(DBL_EPSILON)*fmax(1.0,hi))break;
    }
    return lo;
}

/* Search the actual dynamic capture tuple before Variant-B reaches the old
   nominal p0.  The candidate's radius, runway-anchored p0, C1 lead and
   analytic arc are evaluated together.  This is deliberately a pre-capture
   planner: it may be called again while no path is committed, but the result
   is handed to the ordinary admission helper in the same frame once a
   feasible tuple exists. */
static bool terminal_variant_b_find_dynamic_plan(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double current_e,double current_n,
        double capture_course,HACTransitionPlan*plan_out,double*radius_out,
        double*authority_min_out,double*energy_radius_out,
        double*entry_speed_out,bool*provisional_out,
        GuidanceMachine*vertical_profile_out){
    if(plan_out)memset(plan_out,0,sizeof(*plan_out));
    if(vertical_profile_out)memset(vertical_profile_out,0,sizeof(*vertical_profile_out));
    if(radius_out)*radius_out=NAN;
    if(authority_min_out)*authority_min_out=INFINITY;
    if(energy_radius_out)*energy_radius_out=0.0;
    if(entry_speed_out)*entry_speed_out=NAN;
    if(provisional_out)*provisional_out=false;
    if(!g||!t||!p||!cfg||!isfinite(current_e)||!isfinite(current_n)||
       !isfinite(course)||!isfinite(capture_course))return false;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    const char*audit_env=getenv("KSP_LANDER_HAC_ENERGY_AUDIT");
    const char*nominal_env=getenv("KSP_LANDER_HAC_PROVISIONAL_NOMINAL");
    bool nominal_selector=diag&&strcmp(diag,"1")==0&&nominal_env&&
        strcmp(nominal_env,"1")==0&&!g->diagnostic_shadow;
    bool audit_enabled=diag&&strcmp(diag,"1")==0&&
        ((audit_env&&strcmp(audit_env,"1")==0)||nominal_selector)&&
        !g->diagnostic_shadow;
    static int variant_b_search_diag_count=0;
    if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
       variant_b_search_diag_count++<24){
        fprintf(stderr,"HAC_SEARCH_BEGIN ut=%.3f source=variant-b phase=%d currentE=%.3f currentN=%.3f course=%.3f capture=%.3f terminalContractValid=%d terminalEvaluationValid=%d terminalBlock=%s terminalPathCommitted=%d\n",
            t->ut,g->phase,current_e,current_n,course,capture_course,
            g->taem_exec.terminal_contract.valid?1:0,
            g->taem_exec.terminal_evaluation.valid?1:0,
            taem_terminal_block_reason_string(g->taem_exec.terminal_evaluation.block_reason),
            g->taem_exec.terminal_contract.path_committed?1:0);
    }
    /* One identity per complete search; diagnostics never change selection. */
    static unsigned long long next_evaluation_id=0;
    const unsigned long long evaluation_id=audit_enabled?++next_evaluation_id:0;
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    double relative_capture=norm_signed_deg(cfg->site.runway_heading-capture_course);
    double angular_epsilon=sqrt(DBL_EPSILON)*RAD2DEG;
    if(fabs(relative_capture)<=angular_epsilon)return false;
    /*
     * HAC side is the signed turn from inlet course to runway course.  The
     * transition geometry defines arc angle as side*(runway-entry), so RW09
     * from a northbound 0 deg inlet is side +1 (a 90 deg turn), not -1.
     */
    double hac_side=relative_capture>0.0?1.0:-1.0;
    /*
     * Prospective radius domain:
     *   lower -- best-case physical turn radius at the densest reachable
     *            atmosphere (the runway/site altitude);
     *   upper -- any p0 farther away than the vehicle's best-glide reachable
     *            ground range cannot be an executable unpowered lead.
     * The configured HAC radius remains a preference/sample, not a bound.
     */
    TaemSpeedEnvelope site_speed=decision_taem_speed_envelope(v,p,cfg->site.altitude);
    double numerical_radius=sqrt(DBL_EPSILON)*
        fmax(1.0,fabs(s->final_approach_distance));
    double lower=site_speed.valid&&isfinite(site_speed.minimum_turn_radius_m)?
        fmax(numerical_radius,site_speed.minimum_turn_radius_m):numerical_radius;

    HACPoint2 unit_entry={0},unit_center={0},unit_exit={0};
    if(!hac_fixed_alignment_geometry(&unit_entry,&unit_center,&unit_exit,
            &cfg->site,s,1.0,capture_course,hac_side))
        return false;
    double basis_e=unit_entry.e-unit_exit.e;
    double basis_n=unit_entry.n-unit_exit.n;
    double basis_norm=hypot(basis_e,basis_n);
    if(!(basis_norm>DBL_MIN))return false;
    double reachable_range=terminal_variant_b_reachable_ground_range(g,t,p,cfg);
    if(!isfinite(reachable_range)||!(reachable_range>0.0))return false;
    double current_to_exit=hypot(current_e-unit_exit.e,current_n-unit_exit.n);
    double upper=(reachable_range+current_to_exit)/basis_norm;
    if(!isfinite(lower)||!isfinite(upper)||lower>upper)return false;

    double authority_required=NAN,authority_available=NAN,minimum_radius=INFINITY;
    (void)terminal_fixed_hac_authority(g,t,p,aero,v,s,
        fmax(lower,terminal_hac_radius_live(t,p,aero,v,s)),
        &authority_required,&authority_available,&minimum_radius);
    if(authority_min_out)*authority_min_out=minimum_radius;

    double current_speed=fmax(DBL_MIN,guidance_lateral_speed(t));
    /*
     * Geometry must use the same force-book authority that certifies the lead.
     * The generic decision envelope can be optimistic during a low-incidence
     * high-energy state; mixing the two produced acquisition circles that were
     * guaranteed to fail terminal_fixed_hac_authority() on their first sample.
     */
    if(!isfinite(authority_available)||!(authority_available>DBL_MIN)||
       !isfinite(minimum_radius)||!(minimum_radius>DBL_MIN))
        return false;

    /*
     * Shuttle TAEM acquisition steers on heading error to the selected HAC
     * tangent.  Keep the force/drag inputs outside the radius sweep, but derive
     * response bank and response-projected state from each candidate tangent.
     * The strict finite lead propagator remains authoritative afterward.
     */
    double live_drag=terminal_expected_energy_loss_accel(g,t,aero,v);
    if(!isfinite(live_drag)||live_drag<0.0)live_drag=0.0;
    double bank_effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));


    double preferred=terminal_hac_radius_live(t,p,aero,v,s);
    double inferred_radius=NAN;
    double basis2=basis_e*basis_e+basis_n*basis_n;
    if(basis2>DBL_MIN){
        double rel_e=current_e-unit_exit.e;
        double rel_n=current_n-unit_exit.n;
        double projected=(rel_e*basis_e+rel_n*basis_n)/basis2;
        /* Projection is only a numerical sample.  The complete lead,
           authority and energy certificates below decide whether it is real. */
        if(isfinite(projected)&&projected>=lower&&projected<=upper)
            inferred_radius=projected;
    }
    double samples[128];size_t count=0;
    samples[count++]=lower;
    if(isfinite(inferred_radius)&&inferred_radius>lower&&inferred_radius<upper)
        samples[count++]=inferred_radius;
    /* The old 32-point scan stepped about 3 km across the useful 18--30 km
       authority band and could skip a narrow authority/energy intersection.
       Keep the same bounded radius domain, but resolve it at roughly 1 km. */
    for(int i=1;i<=95;i++)
        samples[count++]=lower+(upper-lower)*(double)i/95.0;
    if(isfinite(preferred)&&preferred>lower&&preferred<upper)
        samples[count++]=preferred;
    samples[count++]=upper;

    HACTransitionPlan fallback_plan={0};
    GuidanceMachine fallback_profile={0};
    double fallback_radius=NAN,fallback_energy=0.0,fallback_speed=NAN;
    bool have_fallback=false;
    /* Keep failing tuples only as diagnostics explaining the nearest violated
       certificate.  They are never executable HAC state. */
    HACTransitionPlan provisional_plan={0};
    GuidanceMachine provisional_profile={0};
    double provisional_radius=NAN,provisional_speed=NAN;
    bool have_provisional=false;
    /* Retain the least authority-violating tuple for diagnostics only. */
    HACTransitionPlan authority_fallback_plan={0};
    GuidanceMachine authority_fallback_profile={0};
    double authority_fallback_radius=NAN,authority_fallback_speed=NAN,
        authority_fallback_violation=INFINITY,
        authority_fallback_required=NAN,authority_fallback_available=NAN,
        authority_fallback_min_radius=INFINITY;
    bool have_authority_fallback=false,authority_fallback_selected=false;
    HACTransitionPlan nominal_plan={0};
    GuidanceMachine nominal_profile={0};
    double nominal_radius=NAN,nominal_speed=NAN;
    bool have_nominal=false;
    int geometry_count=0,lead_reachable_count=0,predicted_count=0,energy_count=0;
    int authority_projection_diag_count=0;
    int nominal_positive_count=0,certified_positive_count=0;
    double smallest_predicted_radius=INFINITY,largest_test_radius=0.0;
    int window_rejected=0,lead_rejected=0,authority_rejected=0,energy_rejected=0;
    unsigned long long best_candidate_id=0,selected_candidate_id=0;
    double best_candidate_lead=NAN,best_authority_margin=NAN;
    double best_candidate_nominal_margin=NAN;
    double largest_energy_radius=0.0,largest_energy_margin=-INFINITY;
    double best_nominal_margin=-INFINITY,best_nominal_radius=NAN,
        best_nominal_speed=NAN;
    double best_authority_radius=NAN,best_authority_speed=NAN,
        best_authority_altitude=NAN,best_authority_aoa=NAN,
        best_authority_distance=NAN;
    double best_energy_radius=NAN,best_energy_sustainable=NAN,
        best_energy_margin=NAN,best_energy_available=NAN,
        best_energy_required=NAN,best_energy_loss=NAN,
        best_energy_lead=NAN,best_energy_arc=NAN,
        best_energy_closure=NAN;
    for(size_t i=0;i<count;i++){
        double radius=clampd(samples[i],lower,upper);
        HACTransitionPlan candidate={0};

        /*
         * Use the same finite C1 live-state -> HAC inlet bridge that production
         * MM305 commits.  A constant-radius acquisition circle sized at the live
         * 500+ m/s state is not a valid model of a descending shuttle: speed and
         * available lift change continuously along the lead.  Geometry stays
         * independent of that changing authority; terminal_fixed_hac_lead_reachable()
         * below propagates the real KSP force/drag model along the entire bridge
         * and rejects any sample that cannot actually fly it.
         */
        HACTransitionPlan direct_candidate={0};
        bool have_direct_candidate=false;
        bool at_capture_window=false;
        double acquisition_course=NAN;
        if(hac_fixed_alignment_tangent_course(current_e,current_n,&cfg->site,s,
                radius,hac_side,&acquisition_course)){
            double acquisition_error=norm_signed_deg(acquisition_course-course);
            double response_bank=fabs(acquisition_error)>
                    sqrt(DBL_EPSILON)*RAD2DEG?
                copysign(bank_limit,acquisition_error):0.0;
            double response_time=hac_response_lead_time(t,s,response_bank);
            if(!isfinite(response_time)||response_time<0.0)response_time=0.0;
            double response_e=current_e,response_n=current_n,response_course=course;
            hac_projected_local_state(t,&cfg->site,p->radius,course,response_time,
                response_bank,s->taem_roll_rate,s->entry_roll_acceleration,
                live_lift_accel(t,aero,v),bank_effectiveness,live_drag,
                &response_e,&response_n,&response_course);
            double response_distance=hypot(response_e-current_e,response_n-current_n);
            double response_v2=t->true_air_speed*t->true_air_speed-
                2.0*live_drag*fmax(0.0,response_distance);
            double response_speed=sqrt(fmax(
                v->minimum_safe_speed*v->minimum_safe_speed,response_v2));
            double preferred_path=fmax(response_distance+radius,DBL_MIN);
            have_direct_candidate=hac_transition_plan(&direct_candidate,
                current_e,current_n,response_e,response_n,course,response_course,
                &cfg->site,s,radius,hac_side,s->final_approach_distance,
                current_speed,response_speed,live_drag,
                authority_available,
                preferred_path,0.0,reachable_range);
            if(have_direct_candidate)
                at_capture_window=terminal_variant_b_p0_capture_ready(g,t,course,p,aero,
                    cfg,&direct_candidate,radius,current_e,current_n);
        }

        if(at_capture_window){
            candidate=direct_candidate;
            candidate.lead_start=hac_point(current_e,current_n);
            candidate.lead_p1=candidate.lead_start;
        }else{
            /*
             * The acquisition must be allowed to tighten as drag reduces speed.
             * A circle frozen at the instantaneous V^2/a radius over-predicts the
             * distance badly at 500+ m/s. Prefer the C1 finite lead: its curvature
             * varies continuously, and terminal_fixed_hac_lead_reachable() below
             * propagates speed/lift/altitude along every segment and rejects any
             * point whose local curvature outruns the live KSP force envelope.
             * Keep the exact circle+tangent topology only as a geometry fallback.
             */
            if(!hac_fixed_alignment_plan(&candidate,current_e,current_n,course,
                    capture_course,&cfg->site,s,radius,hac_side)){
                double acquisition_radius=minimum_radius;
                if(!(acquisition_radius>DBL_MIN)||!isfinite(acquisition_radius)||
                   !hac_shuttle_acquisition_plan(&candidate,current_e,current_n,course,
                        &cfg->site,s,radius,hac_side,acquisition_radius,
                        s->final_approach_distance))
                    continue;
            }
        }

        double candidate_total=candidate.lead_length+candidate.cone_arc_length;
        if(candidate_total>reachable_range){
            static int acquisition_range_diag_count=0;
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
               acquisition_range_diag_count++<8)
                fprintf(stderr,"HAC_LEAD_RANGE_REJECT ut=%.3f hacR=%.1f lead=%.1f hacArc=%.1f total=%.1f reachable=%.1f\n",
                    t->ut,radius,candidate.lead_length,candidate.cone_arc_length,
                    candidate_total,reachable_range);
            continue;
        }
        geometry_count++;

        double distance_to_p0=hypot(current_e-candidate.p0.e,
            current_n-candidate.p0.n);
        double p0_course_error=fabs(norm_signed_deg(
            course-candidate.lead_end_course));

        /* A finite certified lead is itself the acquisition course.  Direct-p0
           candidates alone require the discrete capture set. */
        if(!candidate.lead_curve&&!candidate.lead_acquisition&&!at_capture_window){
            window_rejected++;
            continue;
        }
        largest_test_radius=fmax(largest_test_radius,radius);
        double arc_angle=candidate.cone_arc_length/fmax(radius,1.0);
        GuidanceMachine candidate_profile={0};
        double candidate_response_time=NAN;
        if(!fixed_hac_candidate_vertical_profile(&candidate_profile,g,t,&candidate,
                v,s,&cfg->site,hac_side,&candidate_response_time)){
            if(audit_enabled)fprintf(stderr,"HAC_PROFILE_OUTER_REJECT R=%.3f lead=%.3f arc=%.3f valid=%d\n",radius,candidate.lead_length,candidate.cone_arc_length,candidate.valid?1:0);
            lead_rejected+=12;
            continue;
        }
        /* Static angle boxes do not certify an unpowered vertical path.  The
           shared profile checker enforces geometric regularity; the finite
           lead propagator below proves the actual KSP lift/drag response. */
        bool vertical_profile_executable=
            fixed_hac_vertical_profile_executable(&candidate_profile,s);
        double final_profile_slope=candidate_profile.terminal_glide_mode&&
            candidate_profile.terminal_test_glide_slope>0.0?
            candidate_profile.terminal_test_glide_slope:s->final_glide_slope;
        double initial_profile_slope=
            candidate_profile.fixed_hac_reference_initial_slope_deg;
        double settled_profile_slope=
            candidate_profile.fixed_hac_reference_settled_slope_deg;
        double exit_profile_slope=fixed_hac_reference_slope(&candidate_profile,0.0);
        if(!vertical_profile_executable){
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
                fprintf(stderr,
                    "HAC_VERTICAL_PROFILE_REJECT UT %.3f R %.1f lead %.1f arc %.1f initialSlope %.3f settledSlope %.3f exitSlope %.3f finalSlope %.3f transition %.1f exitTransition %.1f\n",
                    t->ut,radius,candidate.lead_length,candidate.cone_arc_length,
                    initial_profile_slope,settled_profile_slope,exit_profile_slope,
                    final_profile_slope,
                    candidate_profile.fixed_hac_reference_transition_distance,
                    candidate_profile.fixed_hac_reference_exit_transition_distance);
            lead_rejected+=12;
            continue;
        }
        double maximum_entry_speed=current_speed;
            /* Search every p0 speed that is still operationally valid.  Radius
               authority and the full energy certificate are the strict HAC
               gates; the old 1.55x final-speed floor rejected reachable leads
               before either gate could evaluate them. */
            double minimum_entry_speed=fmax(v->minimum_safe_speed,
                v->final_approach_speed);
            minimum_entry_speed=fmin(minimum_entry_speed,maximum_entry_speed);
            const int entry_speed_samples=12;
            for(int speed_index=0;speed_index<entry_speed_samples;speed_index++){
            if(at_capture_window&&speed_index>0)break;
            double mix=entry_speed_samples>1?
                (double)speed_index/(double)(entry_speed_samples-1):0.0;
            /* At p0 there is no lead-speed target to solve.  Elsewhere radius
               does not uniquely determine entry speed, so search the bounded
               lead-reachable speed interval from high to low. */
            double entry_speed=at_capture_window?current_speed:
                maximum_entry_speed+(minimum_entry_speed-maximum_entry_speed)*mix;
            Telemetry predicted={0};
            double lead_work=NAN,lead_course_error=INFINITY,
                lead_lateral_margin=-INFINITY;
            if(at_capture_window){
                predicted=*t;
                lead_work=0.0;
                lead_course_error=p0_course_error;
                lead_lateral_margin=INFINITY;
            }else if(!terminal_fixed_hac_lead_reachable(g,t,&candidate,p,aero,v,s,
                    &cfg->site,entry_speed,&candidate_profile,&predicted,&lead_work,
                    &lead_course_error,&lead_lateral_margin)){lead_rejected++;continue;}
            /* The lead forecast owns the delivered p0 state.  Require only the
               vehicle's real operational speed floor here; circle authority
               and the downstream energy certificate decide whether that state
               is usable.  Do not require the requested target to be achieved. */
            double delivered_entry_floor=fmax(v->minimum_safe_speed,
                v->final_approach_speed);
            bool delivered_entry_speed_ok=isfinite(predicted.true_air_speed)&&
                predicted.true_air_speed>=delivered_entry_floor;
            if(!delivered_entry_speed_ok){
                lead_rejected++;
                if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
                   lead_rejected<=12)
                    fprintf(stderr,
                        "HAC_LEAD_SPEED_FLOOR_REJECT UT %.3f R %.1f requested=%.3f delivered=%.3f floor=%.3f\n",
                        t->ut,radius,entry_speed,predicted.true_air_speed,
                        delivered_entry_floor);
                continue;
            }
            /* MM305's 15--25 km envelope is an ownership-entry contract, not
               a persistent altitude floor after the one-way handoff.  Once
               MM305 owns the vehicle, a legitimate descending lead may reach
               p0 below mm305_min_altitude.  The vertical-profile, speed,
               dynamic-pressure, authority and energy certificates decide
               whether that delivered p0 is executable. */
            bool p0_dynamic_valid=isfinite(predicted.mean_altitude)&&
                predicted.mean_altitude>cfg->site.altitude&&
                predicted.mean_altitude<=s->mm305_max_altitude&&
                isfinite(predicted.true_air_speed)&&
                predicted.true_air_speed>=delivered_entry_floor&&
                isfinite(predicted.dynamic_pressure)&&
                predicted.dynamic_pressure<=v->maximum_dynamic_pressure;
            /* Vertical feasibility was propagated through the actual profile
               above; only a finite delivered FPA is required here. */
            bool p0_vertical_valid=isfinite(predicted.flight_path_angle);
            if(!p0_dynamic_valid||!p0_vertical_valid){
                lead_rejected++;
                if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
                   lead_rejected<=24)
                    fprintf(stderr,
                        "HAC_P0_CONTRACT_REJECT UT %.3f R %.1f p0Alt %.1f p0V %.3f p0Fpa %.3f q %.3f dynamic=%d vertical=%d direct=%d\n",
                        t->ut,radius,predicted.mean_altitude,predicted.true_air_speed,
                        predicted.flight_path_angle,predicted.dynamic_pressure,
                        p0_dynamic_valid?1:0,p0_vertical_valid?1:0,
                        at_capture_window?1:0);
                continue;
            }
            lead_reachable_count++;
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
               lead_reachable_count<=3){
                fprintf(stderr,
                    "HAC_LEAD_REACHABLE UT %.3f R %.1f entryV %.3f p0V %.3f alt %.1f q %.3f mach %.3f aoa %.3f leadWork %.3f courseErr %.3f lateralMargin %.6f\n",
                    t->ut,radius,entry_speed,predicted.true_air_speed,
                    predicted.mean_altitude,predicted.dynamic_pressure,
                    predicted.mach,predicted.angle_of_attack,lead_work,
                    lead_course_error,lead_lateral_margin);
            }
            unsigned long long candidate_id=((unsigned long long)i<<32)|
                (unsigned long long)speed_index;
            /* Propagated p0 states must not inherit live-origin force samples.
               Direct-p0 candidates are the live state, so keep their measured
               forces for the same strict authority test used at handoff. */
            if(!at_capture_window){
                predicted.lift_force=0.0;
                predicted.drag_force=0.0;
                predicted.sideslip=0.0;
            }
            Telemetry authority_state=predicted;
            if(!at_capture_window){
                double candidate_lateral=guidance_lateral_speed(&predicted)*
                    guidance_lateral_speed(&predicted)/fmax(radius,1.0);
                double candidate_bank=fmin(v->maximum_bank_angle,
                    dynamic_bank_limit(&predicted,v));
                double authority_aoa_limit=v->maximum_angle_of_attack;
                if(predicted.mach<1.0&&
                   isfinite(v->terminal_maximum_lift_angle_of_attack)&&
                   v->terminal_maximum_lift_angle_of_attack>0.0)
                    authority_aoa_limit=fmin(authority_aoa_limit,
                        v->terminal_maximum_lift_angle_of_attack);
                double lateral_aoa=terminal_required_aoa_for_lateral_limit(
                    &predicted,aero,v,candidate_lateral,candidate_bank,
                    authority_aoa_limit);
                double candidate_profile_slope=fixed_hac_reference_slope(
                    &candidate_profile,candidate.cone_arc_length);
                double profile_aoa=terminal_fpa_force_aoa(&predicted,p,aero,v,
                    -candidate_profile_slope);
                if(!isfinite(profile_aoa))profile_aoa=terminal_hac_energy_aoa(&predicted,v);
                double candidate_aoa=fmax(profile_aoa,
                    isfinite(lateral_aoa)?lateral_aoa:0.0);
                authority_state.angle_of_attack=clampd(candidate_aoa,0.0,
                    authority_aoa_limit);
            }
            double predicted_required=NAN,predicted_available=NAN,
                predicted_min_radius=INFINITY;
            bool candidate_authority=terminal_fixed_hac_authority(g,&authority_state,p,
                aero,v,s,radius,&predicted_required,&predicted_available,
                &predicted_min_radius);
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
               authority_projection_diag_count<12){
                double projected_lift_accel=live_lift_accel(&authority_state,aero,v);
                double projected_bank_limit=fmin(v->maximum_bank_angle,
                    dynamic_bank_limit(&authority_state,v));
                fprintf(stderr,
                    "HAC_P0_AUTHORITY_PROJECTION UT %.3f R %.1f entryV %.3f p0V %.3f alt %.1f q %.3f mach %.3f aoa %.3f liftAccel %.6f bankLimit %.3f required %.6f available %.6f minR %.3f candidate=%d\n",
                    t->ut,radius,entry_speed,predicted.true_air_speed,
                    predicted.mean_altitude,predicted.dynamic_pressure,
                    predicted.mach,authority_state.angle_of_attack,
                    projected_lift_accel,projected_bank_limit,
                    predicted_required,predicted_available,predicted_min_radius,
                    candidate_authority?1:0);
                authority_projection_diag_count++;
            }
            if(isfinite(predicted_min_radius))
                smallest_predicted_radius=fmin(smallest_predicted_radius,
                    predicted_min_radius);
            bool authority_qualified=candidate_authority&&
                isfinite(predicted_min_radius)&&radius>=predicted_min_radius;
            if(!authority_qualified){
                double authority_violation=INFINITY;
                if(isfinite(predicted_required)&&isfinite(predicted_available)&&
                   predicted_available>DBL_MIN)
                    authority_violation=fmax(0.0,
                        predicted_required/predicted_available-1.0);
                if(isfinite(predicted_min_radius)&&radius>DBL_MIN){
                    double radius_violation=fmax(0.0,
                        predicted_min_radius/radius-1.0);
                    authority_violation=isfinite(authority_violation)?
                        fmax(authority_violation,radius_violation):radius_violation;
                }
                if(!isfinite(authority_violation))
                    authority_violation=1.0e9;
                if(!have_authority_fallback||
                   authority_violation<authority_fallback_violation-
                        sqrt(DBL_EPSILON)*fmax(1.0,
                            fabs(authority_fallback_violation))){
                    authority_fallback_plan=candidate;
                    authority_fallback_profile=candidate_profile;
                    authority_fallback_radius=radius;
                    authority_fallback_speed=entry_speed;
                    authority_fallback_violation=authority_violation;
                    authority_fallback_required=predicted_required;
                    authority_fallback_available=predicted_available;
                    authority_fallback_min_radius=predicted_min_radius;
                    have_authority_fallback=true;
                }
                authority_rejected++;
                continue;
            }
            predicted_count++;
            if(!have_provisional||radius<provisional_radius){
                provisional_plan=candidate;
                provisional_profile=candidate_profile;
                provisional_radius=radius;
                provisional_speed=entry_speed;
                have_provisional=true;
            }
            if(!isfinite(best_authority_radius)||
               predicted_min_radius<best_authority_radius){
                best_authority_radius=radius;
                best_authority_speed=predicted.true_air_speed;
                best_authority_altitude=predicted.mean_altitude;
                best_authority_aoa=authority_state.angle_of_attack;
                best_authority_distance=distance_to_p0;
            }
            if(!isfinite(predicted.angle_of_attack)||predicted.angle_of_attack<=0.0)
                predicted.angle_of_attack=terminal_hac_energy_aoa(&predicted,v);
            double sustainable=0.0,margin=NAN,loss=NAN,
                energy_available=NAN,energy_required=NAN;
            FixedHacEnergyAudit audit={0};
            /* predicted is p0, so the lead is intentionally charged zero
               times here. The live-origin admission helper charges it once. */
            (void)terminal_fixed_hac_energy_radius_impl(g,&predicted,p,aero,v,s,
                &cfg->site,t,radius,0.0,arc_angle,
                &sustainable,&margin,&loss,
                &energy_available,&energy_required,
                audit_enabled?&audit:NULL,
                &candidate,&candidate_profile,
                candidate_profile.fixed_hac_reference_slope_deg,NAN);
            if(audit_enabled){
                double live_specific_energy=rotating_specific_energy(t->latitude,
                    t->mean_altitude,t->true_air_speed,p);
                double p0_specific_energy=rotating_specific_energy(predicted.latitude,
                    predicted.mean_altitude,predicted.true_air_speed,p);
                audit.candidate_id=candidate_id;
                audit.candidate_lead_length=candidate.lead_length;
                audit.lead_work=lead_work;
                audit.lead_energy_closure=isfinite(live_specific_energy)&&
                    isfinite(p0_specific_energy)&&isfinite(lead_work)?
                    (live_specific_energy-p0_specific_energy)-lead_work:NAN;
            }
            if(audit_enabled){
                /* Every authority-valid candidate, not just the first per UT. */
                audit.id=evaluation_id;
                    fprintf(stderr,
                        "HAC_ENERGY_AUDIT id=%llu plannerUt=%.3f evalUt=%.3f "
                        "source=predicted-p0 candidateId=%llu candidateRadius=%.3f entrySpeed=%.3f "
                        "authorityRequired=%.6f authorityAvailable=%.6f authorityMinR=%.3f "
                        "candidateLeadLength=%.3f leadWork=%.6f leadEnergyClosure=%.6f "
                        "leadLength=%.3f arcLength=%.3f finalPath=%.3f totalPath=%.3f "
                        "profileSlope=%.6f targetAoA=%.6f p0Alt=%.3f p0Speed=%.3f "
                        "p0Fpa=%.6f p0SpecificE=%.6f gateSpecificE=%.6f "
                        "energyAvailable=%.6f availableExpected=%.6f availableResidual=%.6f "
                        "modeledWork=%.6f uncertainty=%.6f energyRequired=%.6f "
                        "energyMargin=%.6f exitAlt=%.3f exitSpeed=%.3f "
                        "exitSpecificE=%.6f closure=%.6f arcWorkShadow=%.6f "
                        "finalWorkShadow=%.6f arcLengthExpected=%.6f "
                        "arcMeanDragAccel=%.9f partitionResidual=%.6f "
                        "modeledWorkResidual=%.6f requiredWorkResidual=%.6f "
                        "arcEnergyClosure=%.6f finalEnergyClosure=%.6f "
                        "altResidual=%.6f altUncertainty=%.6f "
                        "speedResidual=%.6f speedUncertainty=%.6f "
                        "rangeResidual=%.6f rangeUncertainty=%.6f "
                        "physicsRelRaw=%.6f physicsRel=%.6f physicsUncertainty=%.6f "
                        "uncertaintyReconstructed=%.6f uncertaintyResidual=%.6f\n",
                        audit.id,t->ut,predicted.ut,audit.candidate_id,radius,entry_speed,
                        predicted_required,predicted_available,predicted_min_radius,
                        audit.candidate_lead_length,audit.lead_work,
                        audit.lead_energy_closure,
                        audit.lead_length,audit.arc_length,audit.final_path_length,
                        audit.total_path_length,audit.profile_slope,audit.target_aoa,
                        audit.altitude,audit.speed,audit.fpa,audit.specific_energy,
                        audit.gate_specific_energy,audit.available,
                        audit.available_expected,audit.available_residual,
                        audit.modeled_work,audit.uncertainty,
                        audit.required,audit.margin,audit.exit_altitude,audit.exit_speed,
                        audit.exit_specific_energy,audit.closure,audit.arc_work_shadow,
                        audit.final_work_shadow,audit.arc_length_expected,
                        audit.arc_mean_drag_accel,audit.partition_residual,
                        audit.modeled_work_residual,audit.required_work_residual,
                        audit.arc_energy_closure,audit.final_energy_closure,
                        audit.altitude_residual,audit.altitude_uncertainty,
                        audit.speed_residual,audit.speed_uncertainty,
                        audit.range_residual,audit.range_uncertainty,
                        audit.physics_relative_uncertainty_raw,
                        audit.physics_relative_uncertainty,audit.physics_uncertainty,
                        audit.uncertainty_reconstructed,audit.uncertainty_residual);
                double gate_alt=cfg->site.altitude+fmax(0.0,g->terminal_test_preflare_altitude);
                double gate_v=g->terminal_test_preflare_min_speed>0.0?
                    g->terminal_test_preflare_min_speed:g->terminal_test_preflare_target_speed;
                double live_available=entry_remaining_specific_energy(t->latitude,
                    t->mean_altitude,t->true_air_speed,cfg->site.latitude,gate_alt,gate_v,p);
                fprintf(stderr,"HAC_LEDGER evalId=%llu candidateId=%llu liveLat=%.12f liveAlt=%.9f liveV=%.9f gateLat=%.12f gateAlt=%.9f gateV=%.9f planetR=%.9f mu=%.9f liveAvailable=%.9f leadWork=%.9f closureCorrection=%.9f arcWork=%.9f finalWork=%.9f effectiveUncertainty=%.9f nominalMargin=%.9f certifiedMargin=%.9f p0Vertical=%.9f p0Fpa=%.9f p0Horizontal=%.9f p0Ut=%.9f liveUt=%.9f\n",
                    evaluation_id,candidate_id,t->latitude,t->mean_altitude,t->true_air_speed,
                    cfg->site.latitude,gate_alt,gate_v,p->radius,p->gravitational_parameter,
                    live_available,audit.lead_work,audit.lead_energy_closure,
                    audit.arc_work_shadow,audit.final_work_shadow,audit.uncertainty,
                    audit.nominal_margin,audit.margin,predicted.vertical_speed,
                    predicted.flight_path_angle,predicted.horizontal_speed,predicted.ut,t->ut);
                fprintf(stderr,"HAC_LEG_STATES evalId=%llu candidateId=%llu p0Lat=%.12f p0Alt=%.9f p0V=%.9f arcEndAlt=%.9f arcEndV=%.9f finalEndAlt=%.9f finalEndV=%.9f appliedDragAnchor=%.9f\n",
                    evaluation_id,candidate_id,predicted.latitude,predicted.mean_altitude,
                    predicted.true_air_speed,audit.arc_end_altitude,audit.arc_end_speed,
                    audit.exit_altitude,audit.exit_speed,audit.drag_anchor);
                fprintf(stderr,"HAC_DRAG_REFERENCE evalId=%llu candidateId=%llu liveAnchor=%.9f p0Anchor=%.9f liveEMA=%.9f p0Ut=%.9f\n",
                    evaluation_id,candidate_id,terminal_drag_projection_anchor(g,t,p,aero,v),
                    terminal_drag_projection_anchor(g,&predicted,p,aero,v),
                    g->terminal_energy_loss_accel_ema,predicted.ut);
                fprintf(stderr,"HAC_CANDIDATE_RESULT evalId=%llu candidateId=%llu certificateValid=%d energyValid=%d reason=%s\n",
                    evaluation_id,candidate_id,audit.valid?1:0,
                    isfinite(margin)&&margin>=0.0&&sustainable>=radius?1:0,
                    !audit.valid?"invalid-work-or-envelope":
                    (!isfinite(margin)?"nonfinite-margin":(margin<0.0?"negative-certified-margin":"pass")));
            }
            if(audit_enabled&&isfinite(audit.nominal_margin)){
                if(audit.nominal_margin>0.0)nominal_positive_count++;
                bool new_nominal_best=!isfinite(best_nominal_margin)||
                    audit.nominal_margin>best_nominal_margin;
                if(new_nominal_best){
                    best_nominal_margin=audit.nominal_margin;
                    best_nominal_radius=radius;
                    best_nominal_speed=predicted.true_air_speed;
                }
                if(!have_nominal||new_nominal_best){
                    nominal_plan=candidate;
                    nominal_profile=candidate_profile;
                    nominal_radius=radius;
                    nominal_speed=entry_speed;
                    have_nominal=true;
                }
            }
            if(audit_enabled&&isfinite(margin)&&margin>0.0)
                certified_positive_count++;
            if(isfinite(sustainable))largest_energy_radius=fmax(
                largest_energy_radius,sustainable);
            if(isfinite(margin))largest_energy_margin=fmax(
                largest_energy_margin,margin);
            if(!isfinite(best_energy_margin)||
               (isfinite(margin)&&margin>best_energy_margin)){
                best_candidate_id=candidate_id;
                best_candidate_lead=candidate.lead_length;
                best_authority_margin=predicted_available-predicted_required;
                best_candidate_nominal_margin=audit_enabled?audit.nominal_margin:NAN;
                best_energy_radius=radius;
                best_energy_sustainable=sustainable;
                best_energy_margin=margin;
                best_energy_available=energy_available;
                best_energy_required=energy_required;
                best_energy_loss=loss;
                best_energy_lead=0.0;
                best_energy_arc=candidate.cone_arc_length;
                /* The least-negative certified margin is the forced fallback
                   when the ordinary feasible set is empty.  This replaces the
                   old smallest-radius provisional heuristic with the candidate
                   closest to satisfying the actual downstream certificate. */
                provisional_plan=candidate;
                provisional_profile=candidate_profile;
                provisional_radius=radius;
                provisional_speed=entry_speed;
                have_provisional=true;
                /* Report the evaluated certificate, not a second, differently
                   parameterized diagnostic integration of the same arc. */
                best_energy_closure=audit_enabled?audit.closure:NAN;
            }
            if(!isfinite(sustainable)||sustainable<radius||
               !isfinite(margin)||margin<0.0){energy_rejected++;continue;}
            energy_count++;
            if(energy_radius_out&&isfinite(sustainable)&&
               sustainable>*energy_radius_out)*energy_radius_out=sustainable;
            if(!have_fallback){
                selected_candidate_id=candidate_id;
                fallback_plan=candidate;fallback_profile=candidate_profile;
                fallback_radius=radius;
                fallback_energy=sustainable;fallback_speed=entry_speed;
                have_fallback=true;
            }
            }
    }
    if(audit_enabled){
        fprintf(stderr,"HAC_EVAL diagRev=2 evalId=%llu ut=%.9f samples=%zu geometry=%d leadReachable=%d authorityValid=%d energyValid=%d bestCandidateId=%llu selectedCandidateId=%llu selected=%d bestR=%.9f bestLead=%.9f authorityMargin=%.9f nominalMargin=%.9f certifiedMargin=%.9f rejectGeometry=%zu rejectWindow=%d rejectLead=%d rejectAuthority=%d rejectEnergy=%d\n",
            evaluation_id,t->ut,count,geometry_count,lead_reachable_count,predicted_count,
            energy_count,best_candidate_id,selected_candidate_id,have_fallback?1:0,
            best_energy_radius,best_candidate_lead,best_authority_margin,best_candidate_nominal_margin,
            best_energy_margin,count-(size_t)geometry_count,window_rejected,lead_rejected,
            authority_rejected,energy_rejected);
    }
    if(have_fallback){
        if(plan_out)*plan_out=fallback_plan;
        if(radius_out)*radius_out=fallback_radius;
        if(energy_radius_out)*energy_radius_out=fallback_energy;
        if(entry_speed_out)*entry_speed_out=fallback_speed;
        if(vertical_profile_out)*vertical_profile_out=fallback_profile;
        return true;
    }
    if(!have_provisional&&have_authority_fallback){
        provisional_plan=authority_fallback_plan;
        provisional_profile=authority_fallback_profile;
        provisional_radius=authority_fallback_radius;
        provisional_speed=authority_fallback_speed;
        have_provisional=true;
        authority_fallback_selected=true;
    }
    if(nominal_selector&&have_nominal){
        /* Experimental selector may still override the fallback ranking when
           explicitly requested, but it remains marked as a forced fallback. */
        provisional_plan=nominal_plan;
        provisional_profile=nominal_profile;
        provisional_radius=nominal_radius;
        provisional_speed=nominal_speed;
        have_provisional=true;
        authority_fallback_selected=false;
    }
    if(have_provisional){
        if(plan_out)*plan_out=provisional_plan;
        if(radius_out)*radius_out=provisional_radius;
        if(energy_radius_out&&isfinite(best_energy_sustainable))
            *energy_radius_out=best_energy_sustainable;
        if(entry_speed_out)*entry_speed_out=provisional_speed;
        if(provisional_out)*provisional_out=true;
        if(vertical_profile_out)*vertical_profile_out=provisional_profile;
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
            if(authority_fallback_selected)
                fprintf(stderr,
                    "HAC_FORCED_AUTHORITY_FALLBACK_SELECT ut=%.3f R=%.1f entryV=%.1f authorityViolation=%.6f required=%.6f available=%.6f minR=%.1f reason=no-authority-qualified-candidate\n",
                    t->ut,provisional_radius,provisional_speed,
                    authority_fallback_violation,authority_fallback_required,
                    authority_fallback_available,authority_fallback_min_radius);
            else
                fprintf(stderr,
                    "HAC_FORCED_FALLBACK_SELECT ut=%.3f R=%.1f entryV=%.1f certifiedMargin=%.1f sustainableR=%.1f authorityMargin=%.3f reason=no-fully-feasible-candidate\n",
                    t->ut,provisional_radius,provisional_speed,best_energy_margin,
                    best_energy_sustainable,best_authority_margin);
        }
    }
    if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
        static double last_probe_ut=-INFINITY;
        if(t->ut-last_probe_ut>=5.0){
            fprintf(stderr,"HAC_PROSPECT ut=%.3f geometry=%d leadReachable=%d predictedAuthority=%d energyPass=%d nominalPositive=%d certifiedPositive=%d provisional=%d provisionalR=%.1f provisionalV=%.1f currentMinR=%.1f currentV=%.1f bounds=%.1f..%.1f smallestPredR=%.1f largestTestR=%.1f largestEnergyR=%.1f largestEnergyMargin=%.1f bestNominalMargin=%.1f bestNominalR=%.1f bestNominalV=%.1f authorityBestR=%.1f authorityBestV=%.1f authorityBestAlt=%.1f authorityBestAoA=%.1f authorityBestD=%.1f energyBestR=%.1f energyBestSustainable=%.1f energyBestMargin=%.1f energyAvail=%.1f energyRequired=%.1f energyLoss=%.3f energyLead=%.1f energyArc=%.1f energyClosure=%.3f noFeasibleTuple\n",
                t->ut,geometry_count,lead_reachable_count,predicted_count,energy_count,
                nominal_positive_count,certified_positive_count,
                have_provisional?1:0,provisional_radius,provisional_speed,minimum_radius,
                current_speed,lower,upper,smallest_predicted_radius,
                largest_test_radius,largest_energy_radius,largest_energy_margin,
                best_nominal_margin,best_nominal_radius,best_nominal_speed,
                best_authority_radius,best_authority_speed,best_authority_altitude,
                best_authority_aoa,best_authority_distance,best_energy_radius,
                best_energy_sustainable,best_energy_margin,best_energy_available,
                best_energy_required,best_energy_loss,best_energy_lead,
                best_energy_arc,best_energy_closure);
            last_probe_ut=t->ut;
        }
    }
    return false;
}
