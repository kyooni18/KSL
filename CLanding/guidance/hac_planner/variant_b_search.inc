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
    double authority_required=NAN,authority_available=NAN,minimum_radius=INFINITY;
    (void)terminal_fixed_hac_authority(t,p,aero,v,s,3000.0,
        &authority_required,&authority_available,&minimum_radius);
    if(authority_min_out)*authority_min_out=minimum_radius;
    if(!isfinite(minimum_radius)||!(authority_available>0.0))return false;

    const double minimum_hac_radius=3000.0;
    /* This is a prospective search.  The current speed may require a
       50-km instantaneous turn radius, but the finite lead exists precisely
       to reach a lower-speed entry state.  Enforce authority on the predicted
       circle-entry speed below; do not discard every useful future radius
       because of the present upstream velocity. */
    double lower=minimum_hac_radius;
    double upper=fmax(minimum_hac_radius,
        fmin(p->radius*.20,fmax(s->hac_radius*8.0,s->taem_interface_range*2.5)));
    if(!isfinite(lower)||!isfinite(upper)||lower>upper)return false;
    double current_speed=fmax(1.0,guidance_lateral_speed(t));
    HACPoint2 nominal_p0={0},nominal_center={0},nominal_exit={0};
    double nominal_distance=INFINITY;
    if(hac_fixed_alignment_geometry(&nominal_p0,&nominal_center,&nominal_exit,
            &cfg->site,s,s->hac_radius,capture_course,1.0))
        nominal_distance=hypot(nominal_p0.e-current_e,
            nominal_p0.n-current_n);
    /* A dynamic circle may move its tangent p0 away from the nominal circle.
       Bound that displacement by the existing TAEM interface envelope rather
       than allowing an unbounded search (the earlier bug selected a 75 km
       circle with a 109 km p0). */
    double maximum_candidate_distance=fmax(12000.0,
        nominal_distance+fmax(10000.0,s->taem_interface_range*.75));
    /* The nominal configured radius is not a physical lower bound for a
       dynamic candidate.  It made a smaller, reachable candidate look
       invalid merely because its target speed differed from the old 12 km
       rehearsal.  Retain only the existing vehicle/staging safety floor; the
       candidate's own target-speed and normal admission checks remain final. */

    /* Include the live comfort-radius preference and, when the vehicle is
       already close to an analytic tangent, the radius implied by that
       measured p0.  The latter prevents an ideal handoff from being rounded
       away by the ~1 km prospective scan. */
    double preferred=terminal_hac_radius_live(t,p,aero,v,s);
    double inferred_radius=NAN;
    HACPoint2 unit_entry={0},unit_center={0},unit_exit={0};
    if(hac_fixed_alignment_geometry(&unit_entry,&unit_center,&unit_exit,
            &cfg->site,s,1.0,capture_course,1.0)){
        double basis_e=unit_entry.e-unit_exit.e;
        double basis_n=unit_entry.n-unit_exit.n;
        double basis2=basis_e*basis_e+basis_n*basis_n;
        if(basis2>DBL_MIN){
            double rel_e=current_e-unit_exit.e;
            double rel_n=current_n-unit_exit.n;
            double projected=(rel_e*basis_e+rel_n*basis_n)/basis2;
            double implied_e=unit_exit.e+projected*basis_e;
            double implied_n=unit_exit.n+projected*basis_n;
            double residual=hypot(current_e-implied_e,current_n-implied_n);
            if(isfinite(projected)&&projected>=lower&&projected<=upper&&
               residual<=750.0&&
               fabs(norm_signed_deg(course-capture_course))<=10.0)
                inferred_radius=projected;
        }
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
    /* Geometry, lead reachability and authority remain useful even when no
       candidate passes the full energy certificate.  Keep the closest failing
       tuple as a frozen forced fallback.  A normally feasible tuple always
       wins; the fallback exists so MM305 still flies the best available HAC
       instead of aborting solely because the feasible set is empty. */
    HACTransitionPlan provisional_plan={0};
    GuidanceMachine provisional_profile={0};
    double provisional_radius=NAN,provisional_speed=NAN;
    bool have_provisional=false;
    /* If no tuple reaches the energy certificate at all, retain the
       lead-reachable circle with the smallest normalized authority violation.
       This is strictly lower priority than every authority-valid tuple. */
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
    double tangent_e=sin(capture_course*DEG2RAD);
    double tangent_n=cos(capture_course*DEG2RAD);
    double right_e=cos(capture_course*DEG2RAD);
    double right_n=-sin(capture_course*DEG2RAD);
    for(size_t i=0;i<count;i++){
        double radius=clampd(samples[i],lower,upper);
        HACTransitionPlan candidate={0};
        if(!hac_fixed_alignment_plan(&candidate,current_e,current_n,course,
                capture_course,&cfg->site,s,radius,1.0))continue;
        geometry_count++;
        double dx=current_e-candidate.p0.e,dy=current_n-candidate.p0.n;
        double along_to_p0=dx*tangent_e+dy*tangent_n;
        double cross_to_p0=dx*right_e+dy*right_n;
        double distance_to_p0=hypot(dx,dy);
        double p0_course_error=fabs(norm_signed_deg(course-capture_course));
        bool at_capture_window=distance_to_p0<=500.0&&
            fabs(cross_to_p0)<=350.0&&p0_course_error<=5.0;
        bool p0_still_ahead=along_to_p0< -fmax(1000.0,current_speed*2.0);
        if((!at_capture_window&&!p0_still_ahead)||
           distance_to_p0>maximum_candidate_distance){window_rejected++;continue;}
        /* An ideal MM305 handoff may already be inside the measured p0
           capture set.  Do not manufacture a tiny Bezier lead and do not
           reject it for lacking 1 km of staging distance: certify the live
           state directly as p0, then let the same authority/energy gates
           decide whether this radius may be committed. */
        if(at_capture_window){
            candidate.lead_start=hac_point(current_e,current_n);
            candidate.lead_p1=candidate.lead_start;
            candidate.lead_p2=candidate.p0;
            candidate.lead_curve=false;
            candidate.lead_length=0.0;
        }
        largest_test_radius=fmax(largest_test_radius,radius);
        double arc_angle=candidate.cone_arc_length/fmax(radius,1.0);
        GuidanceMachine candidate_profile={0};
        double candidate_response_time=NAN;
        if(!fixed_hac_candidate_vertical_profile(&candidate_profile,g,t,&candidate,
                v,s,&cfg->site,1.0,&candidate_response_time)){
            if(audit_enabled)fprintf(stderr,"HAC_PROFILE_OUTER_REJECT R=%.3f lead=%.3f arc=%.3f valid=%d\n",radius,candidate.lead_length,candidate.cone_arc_length,candidate.valid?1:0);
            lead_rejected+=12;
            continue;
        }
        /* A dynamic radius must also make the frozen vertical trajectory
           executable.  The middle descent may be steeper than the outer-final
           tangent, but both response segments must remain inside the live -35
           deg FPA envelope and the profile must actually recover to the final
           tangent at HAC exit. */
        double final_profile_slope=candidate_profile.terminal_glide_mode&&
            candidate_profile.terminal_test_glide_slope>0.0?
            candidate_profile.terminal_test_glide_slope:s->final_glide_slope;
        double initial_profile_slope=
            candidate_profile.fixed_hac_reference_initial_slope_deg;
        double settled_profile_slope=
            candidate_profile.fixed_hac_reference_settled_slope_deg;
        double exit_profile_slope=fixed_hac_reference_slope(&candidate_profile,0.0);
        bool vertical_profile_executable=isfinite(initial_profile_slope)&&
            isfinite(settled_profile_slope)&&isfinite(final_profile_slope)&&
            isfinite(exit_profile_slope)&&
            initial_profile_slope<=35.0+1e-6&&
            settled_profile_slope<=35.0+1e-6&&
            final_profile_slope<=35.0+1e-6&&
            fabs(exit_profile_slope-final_profile_slope)<=0.5;
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
                predicted.true_air_speed>v->minimum_safe_speed*1.20&&
                isfinite(predicted.dynamic_pressure)&&
                predicted.dynamic_pressure<=v->maximum_dynamic_pressure*1.05;
            bool p0_vertical_valid=isfinite(predicted.flight_path_angle)&&
                predicted.flight_path_angle<-1.0&&
                predicted.flight_path_angle>-35.0;
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
                if(!isfinite(profile_aoa))profile_aoa=terminal_hac_energy_aoa(v);
                double candidate_aoa=fmax(profile_aoa,
                    isfinite(lateral_aoa)?lateral_aoa:0.0);
                authority_state.angle_of_attack=clampd(candidate_aoa,0.0,
                    authority_aoa_limit);
            }
            double predicted_required=NAN,predicted_available=NAN,
                predicted_min_radius=INFINITY;
            bool candidate_authority=terminal_fixed_hac_authority(&authority_state,p,
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
                predicted.angle_of_attack=terminal_hac_energy_aoa(v);
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

