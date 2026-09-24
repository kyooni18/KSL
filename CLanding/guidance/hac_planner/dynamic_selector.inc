/* Select a dynamic MM305 fixed-circle candidate before ownership commit.  The
   instantaneous turn-radius helper is only an authority bound; it is not a
   valid selector because the finite lead changes speed, altitude, and energy
   before p0.  Every candidate below uses the exact runway-anchored circle and
   the existing C1 lead, then pays the same predicted-p0 authority and full
   lead+arc+final energy certificates used by admission. */
       bool terminal_force_acquisition_with_aero(GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    /* A published MM304 inlet owns the HAC radius that generated its geometry.
       Re-prove that exact circle at MM305; never replace it with the configured
       nominal radius after ownership transfer. Rehearsal/debug paths without a
       published inlet may still start from the configured nominal radius. */
    double frozen_radius=s->hac_radius;
    double required_lateral=NAN,available_lateral=NAN,minimum_radius=INFINITY;
    double energy_sustainable_radius=INFINITY,energy_margin=NAN;
    double energy_loss=NAN,energy_available=NAN,energy_required=NAN;
    Telemetry audit_predicted_state={0};
    bool audit_predicted_valid=false;
    bool energy_authority=true;
    bool dynamic_radius_selected=false;
    bool dynamic_search_fallback=false;
    bool fixture_admission_latched=g->terminal_fixed_hac_fixture_latched&&
        !g->fixed_alignment_hac_latched;
    bool fixed_geometry=!fixture_admission_latched&&
        entry_taem_tangent_target_geometry(&g->taem_interface_target,cfg);
    bool force_debug_requested=terminal_hac_debug_force_requested();
    double debug_capture_course=NAN;
    bool debug_fixture=!fixed_geometry&&g->terminal_test_capture_active&&
        terminal_hac_debug_fixture_state(t,course,cfg,&debug_capture_course);
    bool upstream_variant_capture=hac_variant_b_requested()&&
        g->terminal_test_upstream_staging&&
        g->terminal_test_upstream_capture_ready;
    bool upstream_planned=upstream_variant_capture&&
        g->terminal_test_upstream_plan_valid&&
        !g->terminal_test_upstream_provisional_plan_valid&&
        isfinite(g->terminal_test_upstream_planned_radius)&&
        g->terminal_test_upstream_planned_radius>0.0;
    /* Acquisition is one-way only for a tuple whose live authority/energy
       contract remains valid. A failed recheck is a failed capture, not a
       reason to promote a previously rejected candidate. */
    bool upstream_forced_commit_allowed=false;
    double upstream_positive_capture=norm_deg(cfg->site.runway_heading+90.0);
    double upstream_negative_capture=norm_deg(cfg->site.runway_heading-90.0);
    double upstream_capture_course=isfinite(g->terminal_test_upstream_staging_course)?
        norm_deg(g->terminal_test_upstream_staging_course):
        (fabs(norm_signed_deg(upstream_positive_capture-course))<=
            fabs(norm_signed_deg(upstream_negative_capture-course))?
            upstream_positive_capture:upstream_negative_capture);
    GuidanceSettings geometry_settings=*s;
    if(g->terminal_test_final_approach_distance>0.0)
        geometry_settings.final_approach_distance=
            g->terminal_test_final_approach_distance;
    if(debug_fixture){
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
        double e=0.0,n=0.0,tangent_course=NAN;
        local_offsets(origin,current,p->radius,&e,&n);
        if(hac_fixed_alignment_tangent_course(e,n,&cfg->site,
                &geometry_settings,frozen_radius,1.0,&tangent_course))
            debug_capture_course=tangent_course;
        else
            return false;
    }
    double capture_course=fixed_geometry?g->taem_interface_target.course:
        upstream_variant_capture?upstream_capture_course:
        debug_capture_course;
    bool fixed_path_geometry=fixed_geometry||debug_fixture||
        upstream_variant_capture;
    bool dynamic_fixed_fixture=fixed_path_geometry&&!upstream_planned;
    if(fixed_path_geometry){
        if(upstream_planned){
            frozen_radius=g->terminal_test_upstream_planned_radius;
        }else if(fixed_geometry){
            if(!isfinite(g->taem_interface_target.hac_radius)||
               !(g->taem_interface_target.hac_radius>0.0))return false;
            frozen_radius=g->taem_interface_target.hac_radius;
            dynamic_radius_selected=false;
        }else if(dynamic_fixed_fixture){
            frozen_radius=s->hac_radius;
            dynamic_radius_selected=false;
        }else if(upstream_variant_capture){
            double dynamic_radius=terminal_hac_radius_live(t,p,aero,v,s);
            /* The simulator fixture is the only fixed-HAC path allowed to
               select this radius prospectively. Normal authority and energy
               certificates still decide whether it may be committed. */
            if(!isfinite(dynamic_radius)||dynamic_radius<=0.0)return false;
            frozen_radius=dynamic_radius;
            dynamic_radius_selected=true;
        }
    }
    bool fixed_authority=fixed_path_geometry&&terminal_fixed_hac_authority(g,t,p,aero,v,s,
        frozen_radius,
        &required_lateral,&available_lateral,&minimum_radius);
    if(fixed_path_geometry&&dynamic_radius_selected){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
            fprintf(stderr,"MM305 dynamic HAC radius: source=%s selectedR=%.1f configuredNominal=%.1f normalCandidate=%d forceDiagnostic=%d.\n",
                upstream_planned?"candidate-local":
                    dynamic_search_fallback?"diagnostic-fallback":"candidate-search",
                frozen_radius,s->hac_radius,upstream_planned?1:0,
                force_debug_requested?1:0);
        }
    }
    bool force_debug=fixed_path_geometry&&force_debug_requested;
    bool force_fallback=false;
    /* Variant B has already flown its upstream staging segment to the planned
       p0 window, so live circle authority is meaningful there.  Production
       fixed-HAC acquisition still owns a finite lead; qualify its delivered
       p0 state below instead of rejecting the 12 km circle at the high-speed
       MM305 handoff state. */
    if(upstream_variant_capture&&!fixed_authority&&!force_debug&&!force_fallback&&
       !upstream_forced_commit_allowed){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        static double last_rejection_ut=-INFINITY;
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
           t->ut-last_rejection_ut>=5.0){
            fprintf(stderr,"MM305 fixed-HAC admission rejected: UT %.2f R %.0f minR %.0f requiredLat %.3f availableLat %.3f V %.1f lift %.3f.\n",
                t->ut,frozen_radius,minimum_radius,required_lateral,available_lateral,
                guidance_lateral_speed(t),live_lift_accel(t,aero,v));
            last_rejection_ut=t->ut;
        }
        return false;
    }
    if(force_debug&&!fixed_authority&&!g->diagnostic_shadow){
        fprintf(stderr,"MM305 DEBUG force-capture: fixed R %.0f admitted for command-path tracing despite requiredLat %.3f > availableLat %.3f (minR %.0f).\n",
            frozen_radius,required_lateral,available_lateral,minimum_radius);
        if(debug_fixture)
            fprintf(stderr,"MM305 DEBUG direct fixture capture: live station/course used for diagnostics only; normal MM304 acceptance remains bypass-free.\n");
    }
    double loss=g->terminal_energy_loss_accel_ema,speed_loss=g->terminal_speed_loss_accel_ema;
    terminal_glide_initialize(g,v,s);
    g->terminal_rehearsal_mode=false;
    g->terminal_energy_loss_accel_ema=loss;g->terminal_speed_loss_accel_ema=speed_loss;
    g->terminal_region_entered=true;
    g->terminal_test_capture_active=true;
    g->taem_safety_handoff=false;

    /* Seed terminal references with a neutral ownership-transfer command. The
       stabilized output keeps its existing limiter state, so this does not step
       the actuators; it gives both the limiter and the future-state predictor the
       same smooth roll-out target immediately after MM304 releases authority. */
    g->terminal_reference_heading=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    g->terminal_reference_bank=0.0;
    /* Seed the post-handoff predictor on the shallow edge of the current
       state-derived terminal slope envelope.  Carrying the measured entry FPA
       into every future sample made a candidate be certified from a steep
       -16 deg origin while its selected runway path was only -8 deg; the
       planner then charged energy for a shallow path that the live aircraft
       had never been brought onto.  TAEM owns the transition now, so its
       first reference must be the same bounded vertical contract used by the
       path selector. */
    double minimum_terminal_slope=0.0,maximum_terminal_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_terminal_slope,
        &maximum_terminal_slope);
    g->terminal_reference_fpa=isfinite(minimum_terminal_slope)&&
        isfinite(maximum_terminal_slope)&&maximum_terminal_slope>=minimum_terminal_slope?
        -minimum_terminal_slope:t->flight_path_angle;
    g->terminal_reference_aoa=clampd(t->angle_of_attack,0.0,v->maximum_angle_of_attack);

    /* Preserve a genuinely fresh, regular MM304 preview across the one-way
       ownership boundary. Its geometry is exactly the coupled interface MM304
       was flying toward. Only the timing/projection cache is rebased under TAEM.
       Old/short-lead previews (v8 and v11) are still discarded. */
    bool preserve_preview=terminal_candidate_operationally_usable(
        g,&g->terminal_candidate,t,course,p,cfg);
    if(preserve_preview)g->terminal_candidate.selected_ut=t->ut;
    else g->terminal_candidate.valid=false;
    g->terminal_prediction_valid=false;
    g->terminal_prediction_ut=-INFINITY;
    g->terminal_prediction_altitude=NAN;
    g->terminal_prediction_speed=NAN;
    g->terminal_prediction_time=0.0;

    g->terminal_test_spiral_active=false;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->hac_progress_valid=false;
    g->hac_captured=false;
    g->hac_completed=false;
    g->hac_remaining=0;
    g->hac_radius=frozen_radius;
    g->hac_side=fabs(norm_signed_deg(t->roll))>=4?(norm_signed_deg(t->roll)>=0?1:-1):
        terminal_default_hac_side(t,&cfg->site,course);
    g->hac_side_selected=false;
    g->hac_plan_degraded=false;
    g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;
    g->hac_plan_violation_score=0.0;
    g->hac_commit_blend=0.0;
    g->entry_control_plan_valid=false;

    if(fixed_path_geometry){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        double relative=norm_signed_deg(capture_course-cfg->site.runway_heading);
        /* The published inlet point, not the perpendicular course by itself,
           identifies the committed short/long HAC turn side. */
        double published_side=fixed_geometry?
            entry_taem_tangent_target_hac_side(&g->taem_interface_target,cfg):0.0;
        double side=fabs(published_side)==1.0?published_side:
            (fabs(relative)>sqrt(DBL_EPSILON)*RAD2DEG?
                (relative>=0.0?1.0:-1.0):
                terminal_default_hac_side(t,&cfg->site,course));
        double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
        double turn_radius=terminal_hac_radius_live(t,p,aero,v,s);
        if(!isfinite(turn_radius)){
            double sustained=live_turn_radius(t,aero,v,side*bank_limit);
            double response=roll_capture_time(t,side*bank_limit,
                s->taem_roll_rate,s->entry_roll_acceleration);
            double response_allowance=isfinite(response)?
                guidance_lateral_speed(t)*clampd(response,0.0,8.0)*0.18:0.0;
            turn_radius=isfinite(sustained)?sustained+response_allowance:s->hac_radius;
        }
        (void)turn_radius;
        /* The published/reference circle is now immutable across the ownership
           boundary; controller saturation is handled in the command law, not by
           silently resizing the circle after this point. */
        g->hac_radius=frozen_radius;

        /* Freeze the circle on runway geometry, but acquire it from the actual
           MM305 state.  The live-to-entry bridge is a finite C1 segment and is
           stored separately from the analytic circle, so it cannot translate,
           resize, or double-count the committed HAC arc. */
        double runway=cfg->site.runway_heading*DEG2RAD;
        GeoPoint fixed_origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint fixed_current={t->latitude,t->longitude,t->mean_altitude};
        double e=0.0,n=0.0;
        local_offsets(fixed_origin,fixed_current,p->radius,&e,&n);
        HACTransitionPlan fixed_plan={0};
        bool fixed_plan_loaded=false;
        if(upstream_planned){
            HACTransitionPlan frozen_plan={0};
            if(!terminal_variant_b_load_tuple(g,&frozen_plan))
                return false;
            /* The cached Variant-B tuple includes the finite staging lead from
               the original search state.  That lead has already been flown by
               the time capture_ready is asserted.  Rebuild only the local
               current->p0 bridge while keeping the selected radius and exact
               runway-anchored analytic circle immutable. */
            if(!hac_fixed_alignment_plan(&fixed_plan,e,n,course,capture_course,
                    &cfg->site,&geometry_settings,g->hac_radius,side))
                return false;
            double frozen_geometry_error=fmax(
                hypot(fixed_plan.p0.e-frozen_plan.p0.e,
                    fixed_plan.p0.n-frozen_plan.p0.n),
                fmax(hypot(fixed_plan.p3.e-frozen_plan.p3.e,
                        fixed_plan.p3.n-frozen_plan.p3.n),
                    hypot(fixed_plan.cone_center.e-frozen_plan.cone_center.e,
                        fixed_plan.cone_center.n-frozen_plan.cone_center.n)));
            double frozen_arc_error=fabs(fixed_plan.cone_arc_length-
                frozen_plan.cone_arc_length);
            if(frozen_geometry_error>1.0||frozen_arc_error>1.0){
                if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
                    fprintf(stderr,
                        "HAC_VARIANT_B_REBASE_REJECT ut=%.3f R=%.1f geometryError=%.3f arcError=%.3f oldLead=%.1f newLead=%.1f\n",
                        t->ut,g->hac_radius,frozen_geometry_error,
                        frozen_arc_error,frozen_plan.lead_length,
                        fixed_plan.lead_length);
                return false;
            }
            fixed_plan_loaded=true;
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
                fprintf(stderr,
                    "HAC_VARIANT_B_COMMIT_REBASE ut=%.3f R=%.1f oldLead=%.1f newLead=%.1f geometryError=%.3f arcError=%.3f\n",
                    t->ut,g->hac_radius,frozen_plan.lead_length,
                    fixed_plan.lead_length,frozen_geometry_error,
                    frozen_arc_error);
        }
        if(!fixed_plan_loaded&&
           !hac_fixed_alignment_plan(&fixed_plan,e,n,course,capture_course,
                &cfg->site,&geometry_settings,g->hac_radius,side)){
            return false;
        }
        if(!(geometry_settings.final_approach_distance>0.0)||
           !isfinite(geometry_settings.final_approach_distance))
            return false;

        /* The simulator circle is dynamic before commit, while the live
           production circle remains configuration-owned. In both cases the
           selected radius is frozen here and the exact finite C1 lead is
           admitted only when its delivered p0 state satisfies authority. */
        double response_time=NAN;
        double energy_profile_slope=NAN;
        GuidanceMachine authority_profile={0};
        bool authority_profile_valid=false;
        if(!upstream_variant_capture){
            Telemetry authority_state=*t;
            bool authority_lead_reachable=fixed_plan.lead_length<=1.0;
            double authority_lead_target_speed=fixed_hac_lead_target_speed(
                t,p,aero,v,frozen_radius);
            double authority_lead_work=0.0;
            double authority_lead_course_error=0.0;
            double authority_lead_lateral_margin=INFINITY;
            double authority_lead_slope=fabs(isfinite(t->flight_path_angle)?
                t->flight_path_angle:8.0);
            authority_profile_valid=fixed_hac_candidate_vertical_profile(
                &authority_profile,g,t,&fixed_plan,v,s,&cfg->site,side,
                &response_time);
            bool authority_profile_executable=authority_profile_valid&&
                fixed_hac_vertical_profile_executable(&authority_profile,s);
            if(authority_profile_valid)
                energy_profile_slope=authority_profile.fixed_hac_reference_slope_deg;
            if(!authority_profile_executable){
                const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
                if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
                    double final_slope=authority_profile.terminal_glide_mode&&
                        authority_profile.terminal_test_glide_slope>0.0?
                        authority_profile.terminal_test_glide_slope:s->final_glide_slope;
                    fprintf(stderr,
                        "MM305 fixed-HAC admission rejected by vertical delivery: R %.0f initialSlope %.3f settledSlope %.3f exitSlope %.3f finalSlope %.3f.\n",
                        frozen_radius,
                        authority_profile.fixed_hac_reference_initial_slope_deg,
                        authority_profile.fixed_hac_reference_settled_slope_deg,
                        fixed_hac_reference_slope(&authority_profile,0.0),final_slope);
                }
                return false;
            }

            fixed_authority=false;
            if(fixed_plan.lead_length<=1.0){
                fixed_authority=terminal_fixed_hac_authority(g,&authority_state,p,
                    aero,v,s,frozen_radius,&required_lateral,&available_lateral,
                    &minimum_radius);
            }else if(authority_profile_valid&&
                     isfinite(authority_lead_target_speed)&&
                     authority_lead_target_speed>0.0){
                Telemetry predicted={0};
                double lead_work=NAN,course_error=INFINITY,
                    lateral_margin=-INFINITY;
                if(terminal_fixed_hac_lead_reachable(g,t,&fixed_plan,p,aero,v,s,
                        &cfg->site,authority_lead_target_speed,&authority_profile,
                        &predicted,&lead_work,&course_error,&lateral_margin)){
                    double predicted_required=NAN,predicted_available=NAN,
                        predicted_minimum=INFINITY;
                    if(terminal_fixed_hac_authority(g,&predicted,p,aero,v,s,
                            frozen_radius,&predicted_required,&predicted_available,
                            &predicted_minimum)){
                        authority_state=predicted;
                        required_lateral=predicted_required;
                        available_lateral=predicted_available;
                        minimum_radius=predicted_minimum;
                        authority_lead_work=lead_work;
                        authority_lead_course_error=course_error;
                        authority_lead_lateral_margin=lateral_margin;
                        authority_lead_slope=fabs(predicted.flight_path_angle);
                        authority_lead_reachable=true;
                        fixed_authority=true;
                        audit_predicted_state=predicted;
                        audit_predicted_valid=true;
                    }
                }
            }

            if((!authority_lead_reachable||!fixed_authority)&&!force_debug){
                if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
                    fprintf(stderr,"MM305 fixed-HAC lead admission rejected: R %.0f lead %.1f targetV %.1f reachable=%d p0MinR %.1f p0RequiredLat %.3f p0AvailableLat %.3f.\n",
                        frozen_radius,fixed_plan.lead_length,
                        authority_lead_target_speed,authority_lead_reachable?1:0,
                        minimum_radius,required_lateral,available_lateral);
                return false;
            }
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&fixed_authority)
                fprintf(stderr,"MM305 fixed-HAC lead admission: R %.0f lead %.1f targetV %.1f p0V %.1f p0Alt %.1f slope %.2f work %.1f courseErr %.2f lateralMargin %.3f p0MinR %.1f.\n",
                    frozen_radius,fixed_plan.lead_length,authority_lead_target_speed,
                    authority_state.true_air_speed,authority_state.mean_altitude,
                    authority_lead_slope,authority_lead_work,
                    authority_lead_course_error,authority_lead_lateral_margin,
                    minimum_radius);
        }
        if(!isfinite(response_time)||response_time<0.0){
            response_time=hac_response_lead_time(t,s,side*bank_limit);
            if(!isfinite(response_time)||response_time<0.0)response_time=0.0;
        }
        double lead_response_distance=guidance_lateral_speed(t)*response_time;
        double lead_response_margin=fixed_plan.lead_length-
            lead_response_distance;
        double maneuver_required_aoa=NAN,maneuver_projected_drag=NAN;
        if(isfinite(required_lateral)&&required_lateral>0.0){
            maneuver_required_aoa=terminal_required_aoa_for_lateral_limit(t,
                aero,v,required_lateral,bank_limit,v->maximum_angle_of_attack);
            double current_incidence=fmax(.25,
                hypot(t->angle_of_attack,t->sideslip));
            double current_df=1.0,target_df=1.0;
            aerodynamic_force_factors_mach(t->mach,current_incidence,v,
                NULL,&current_df);
            aerodynamic_force_factors_mach(t->mach,
                clampd(maneuver_required_aoa,0.0,
                    v->maximum_angle_of_attack),v,NULL,&target_df);
            double live_loss=live_drag_accel(t,aero,v);
            if(isfinite(live_loss)&&live_loss>=0.0&&
               isfinite(current_df)&&current_df>.02&&isfinite(target_df))
                maneuver_projected_drag=live_loss*target_df/current_df;
        }
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
            fprintf(stderr,"MM305 fixed-HAC maneuver: requiredLat %.3f availableLat %.3f authorityRatio %.3f requiredAoA %.2f projectedDPerM %.3f responseTime %.2f responseDist %.1f leadResponseMargin %.1f.\n",
                required_lateral,available_lateral,
                (isfinite(required_lateral)&&isfinite(available_lateral)&&
                 available_lateral>DBL_MIN)?required_lateral/available_lateral:NAN,
                maneuver_required_aoa,maneuver_projected_drag,response_time,
                lead_response_distance,lead_response_margin);
        }
        /* Evaluate the already-probed p0 state against the analytic arc before
           latching the vertical profile or committing ownership.  Modeling
           the lead and downstream arc as one path obscures the lead probe's
           predicted speed and AoA transition.  The returned radius is
           admission evidence only; the executable circle remains the
           selected-radius analytic HAC. */
        double arc_angle=fixed_plan.cone_arc_length/fmax(frozen_radius,1.0);
        const Telemetry*energy_origin=audit_predicted_valid?
            &audit_predicted_state:t;
        double energy_lead_length=audit_predicted_valid?0.0:
            fixed_plan.lead_length;
        const char*audit_env=getenv("KSP_LANDER_HAC_ENERGY_AUDIT");
        bool admission_audit_requested=diag&&strcmp(diag,"1")==0&&
            audit_env&&strcmp(audit_env,"1")==0&&!g->diagnostic_shadow;
        FixedHacEnergyAudit admission_audit={0};
        energy_authority=terminal_fixed_hac_energy_radius_impl(g,energy_origin,p,aero,v,s,
            &cfg->site,t,frozen_radius,energy_lead_length,arc_angle,
            &energy_sustainable_radius,&energy_margin,&energy_loss,
            &energy_available,&energy_required,
            admission_audit_requested?&admission_audit:NULL,
            &fixed_plan,authority_profile_valid?&authority_profile:NULL,
            /* A predicted p0 state needs a slope rebased to the remaining path. */
            audit_predicted_valid?NAN:energy_profile_slope,NAN);
        if(admission_audit_requested){
            fprintf(stderr,
                "MM305_HAC_ENERGY_AUDIT R=%.1f available=%.6f modeledWork=%.6f nominalMargin=%.6f uncertainty=%.6f required=%.6f margin=%.6f path=%.6f arcWork=%.6f finalWork=%.6f dragAnchor=%.9f profileSlope=%.6f targetAoA=%.6f altUnc=%.6f speedUnc=%.6f rangeUnc=%.6f physicsRaw=%.9f physicsUsed=%.9f physicsUnc=%.6f uncRecon=%.6f uncResidual=%.6f closure=%.6f arcClosure=%.6f finalClosure=%.6f\n",
                frozen_radius,admission_audit.available,admission_audit.modeled_work,
                admission_audit.nominal_margin,admission_audit.uncertainty,
                admission_audit.required,admission_audit.margin,
                admission_audit.total_path_length,admission_audit.arc_work_shadow,
                admission_audit.final_work_shadow,admission_audit.drag_anchor,
                admission_audit.profile_slope,admission_audit.target_aoa,
                admission_audit.altitude_uncertainty,admission_audit.speed_uncertainty,
                admission_audit.range_uncertainty,
                admission_audit.physics_relative_uncertainty_raw,
                admission_audit.physics_relative_uncertainty,
                admission_audit.physics_uncertainty,
                admission_audit.uncertainty_reconstructed,
                admission_audit.uncertainty_residual,admission_audit.closure,
                admission_audit.arc_energy_closure,
                admission_audit.final_energy_closure);
        }
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
            double live_loss=live_drag_accel(t,aero,v);
            fprintf(stderr,"MM305 fixed-HAC energy: requestedR %.0f sustainableR %.0f margin %.1f available %.1f required %.1f lossPerM %.3f liveDPerM %.3f emaDPerM %.3f AoA %.2f lead %.1f arc %.1f finalOnce %.1f.\n",
                frozen_radius,energy_sustainable_radius,energy_margin,
                energy_available,energy_required,energy_loss,live_loss,
                g->terminal_energy_loss_accel_ema,t->angle_of_attack,
                fixed_plan.lead_length,fixed_plan.cone_arc_length,
                fmax(0.0,s->final_approach_distance-
                    g->terminal_test_preflare_altitude/
                    fmax(tan(clampd(g->terminal_test_glide_slope,1.0,75.0)*DEG2RAD),1e-3)));
        }
        bool audit_requested=admission_audit_requested&&
            (force_debug||force_fallback||upstream_forced_commit_allowed);
        /* Arm once for the committed path.  terminal-force acquisition may
           be revisited while the same path is already committed; that must
           not erase the immutable audit snapshot or re-arm after close. */
        if(audit_requested&&!g->hac_energy_audit_active&&
           !g->hac_energy_audit_logged){
            g->hac_energy_audit_active=true;
            g->hac_energy_audit_was_lead=false;
            g->hac_energy_audit_rebase_count=0;
            g->hac_energy_audit_start_ut=t->ut;
            g->hac_energy_audit_last_ut=t->ut;
            g->hac_energy_audit_start_specific_energy=
                rotating_specific_energy(t->latitude,t->mean_altitude,
                    t->true_air_speed,p);
            g->hac_energy_audit_actual_drag_work=0.0;
            g->hac_energy_audit_actual_distance=0.0;
            g->hac_energy_audit_forecast_loss=energy_loss;
            g->hac_energy_audit_forecast_available=energy_available;
            g->hac_energy_audit_forecast_required=energy_required;
            g->hac_energy_audit_forecast_margin=energy_margin;
            g->hac_energy_audit_forecast_lead_length=fixed_plan.lead_length;
            g->hac_energy_audit_forecast_p0_speed=audit_predicted_valid?
                audit_predicted_state.true_air_speed:NAN;
            g->hac_energy_audit_forecast_p0_altitude=audit_predicted_valid?
                audit_predicted_state.mean_altitude:NAN;
            g->hac_energy_audit_forecast_p0_fpa=audit_predicted_valid?
                audit_predicted_state.flight_path_angle:NAN;
            g->hac_energy_audit_forecast_p0_aoa=audit_predicted_valid?
                audit_predicted_state.angle_of_attack:NAN;
            g->hac_energy_audit_start_e=e;
            g->hac_energy_audit_start_n=n;
            g->hac_energy_audit_p0_e=fixed_plan.p0.e;
            g->hac_energy_audit_p0_n=fixed_plan.p0.n;
        }
        if(!energy_authority&&!force_debug&&!force_fallback&&
           !upstream_forced_commit_allowed){
            if(upstream_variant_capture)
                g->terminal_test_upstream_energy_rejected=true;
            static double last_energy_rejection_ut=-INFINITY;
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow&&
               t->ut-last_energy_rejection_ut>=5.0){
                double live_loss=live_drag_accel(t,aero,v);
                fprintf(stderr,"MM305 fixed-HAC admission rejected by energy: UT %.2f requestedR %.0f sustainableR %.0f margin %.1f available %.1f required %.1f lossPerM %.3f liveDPerM %.3f emaDPerM %.3f AoA %.2f lead %.1f arc %.1f.\n",
                    t->ut,frozen_radius,energy_sustainable_radius,energy_margin,
                    energy_available,energy_required,energy_loss,live_loss,
                    g->terminal_energy_loss_accel_ema,t->angle_of_attack,
                    fixed_plan.lead_length,fixed_plan.cone_arc_length);
                last_energy_rejection_ut=t->ut;
            }
            return false;
        }
        if(!energy_authority&&force_debug&&!g->diagnostic_shadow)
            fprintf(stderr,"MM305 DEBUG force-capture: energy radius %.0f < requested R %.0f; continuing for diagnostic command-path evidence only.\n",
                energy_sustainable_radius,frozen_radius);
        bool forced_degraded_commit=force_fallback||
            (upstream_forced_commit_allowed&&
             (!fixed_authority||!energy_authority));
        if(forced_degraded_commit){
            double authority_violation=(!fixed_authority&&
                isfinite(required_lateral)&&isfinite(available_lateral)&&
                available_lateral>DBL_MIN)?
                fmax(0.0,required_lateral/available_lateral-1.0):
                (!fixed_authority?1.0:0.0);
            double energy_scale=fmax(1.0,fabs(energy_available));
            double energy_violation=!energy_authority&&isfinite(energy_margin)?
                fmax(0.0,-energy_margin/energy_scale):
                (!energy_authority?1.0:0.0);
            g->hac_plan_degraded=true;
            g->hac_plan_geometry_degraded=!fixed_authority;
            g->hac_plan_energy_degraded=!energy_authority;
            g->hac_plan_violation_score=fmax(authority_violation,energy_violation);
            if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
                fprintf(stderr,
                    "HAC_FORCED_FALLBACK_COMMIT ut=%.3f R=%.1f authority=%d energy=%d authorityViolation=%.6f energyViolation=%.6f certifiedMargin=%.1f sustainableR=%.1f\n",
                    t->ut,frozen_radius,fixed_authority?1:0,energy_authority?1:0,
                    authority_violation,energy_violation,energy_margin,
                    energy_sustainable_radius);
        }
        if(!fixed_hac_vertical_profile_latch(g,t->mean_altitude,
                t->flight_path_angle,t->horizontal_speed,response_time,
                fixed_plan.lead_length,fixed_plan.cone_arc_length,
                &cfg->site,s))
            return false;
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
            fprintf(stderr,"HAC_COMMIT_GATE ut=%.3f geometry=1 authority=%d energy=%d vertical=1 forced=%d degraded=%d violation=%.6f terminalContractValid=%d terminalEvaluationValid=%d terminalBlock=%s preCommit=1\n",
                t->ut,fixed_authority?1:0,energy_authority?1:0,
                forced_degraded_commit?1:0,g->hac_plan_degraded?1:0,
                g->hac_plan_violation_score,
                g->taem_exec.terminal_contract.valid?1:0,
                g->taem_exec.terminal_evaluation.valid?1:0,
                taem_terminal_block_reason_string(g->taem_exec.terminal_evaluation.block_reason));
        g->fixed_hac_lead_target_speed=fixed_hac_lead_target_speed(
            t,p,aero,v,g->hac_radius);
        if(upstream_planned&&g->terminal_test_upstream_speed_target_valid&&
           isfinite(g->terminal_test_upstream_speed_target)&&
           g->terminal_test_upstream_speed_target>0.0)
            g->fixed_hac_lead_target_speed=
                g->terminal_test_upstream_speed_target;
        g->fixed_hac_lead_speed_target_valid=isfinite(
            g->fixed_hac_lead_target_speed)&&
            g->fixed_hac_lead_target_speed>0.0;

        /* The committed fixed-HAC plan owns the runway-final station used by
           its circle.  Keep the downstream terminal reference synchronized
           with that immutable geometry instead of relying on a separate
           test-state initializer to have populated it earlier. */
        g->terminal_test_final_approach_distance=
            geometry_settings.final_approach_distance;
        g->terminal_final_handoff_latched=true;
        g->terminal_final_handoff_distance=geometry_settings.final_approach_distance;
        g->terminal_path_kind=TERMINAL_PATH_HAC;
        g->terminal_path_committed=true;
        g->terminal_test_capture_active=false;
        g->terminal_candidate.valid=false;
        g->hac_side=side;
        g->hac_side_selected=true;
        g->hac_captured=false;
        g->hac_completed=false;
        g->hac_progress_valid=false;
        /* transition_length owns the frozen cone and lead_length owns only the
           finite C1 acquisition.  hac_remaining must duplicate neither one. */
        g->hac_remaining=0.0;
        g->fixed_alignment_hac_latched=true;
        g->hac_transition_lead_rebase_attempted=false;
        hac_transition_store(g,&fixed_plan,response_time);
        g->hac_transition_handoff_lateral_valid=false;
        g->hac_transition_handoff_lateral_acceleration=0.0;
        g->hac_transition_handoff_blend=fixed_plan.lead_length<=1.0?1.0:0.0;
        g->hac_transition_handoff_aoa_valid=false;
        g->hac_transition_handoff_aoa=0.0;
        if(g->hac_transition_lead_length<=1.0)
            g->hac_transition_lead_progress=1.0;
        g->hac_previous_angle=fixed_plan.end_angle;
        g->terminal_reference_heading=course;
        g->terminal_reference_bank=0.0;
        g->minimum_turn_radius=minimum_radius;
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow){
            double upper_e=fixed_plan.cone_center.e;
            double upper_n=fixed_plan.cone_center.n+g->hac_radius;
            double exit_along=fixed_plan.p3.e*sin(runway)+fixed_plan.p3.n*cos(runway);
            double exit_cross=fixed_plan.p3.e*cos(runway)-fixed_plan.p3.n*sin(runway);
            double runway_station_e=-sin(runway)*geometry_settings.final_approach_distance;
            double runway_station_n=-cos(runway)*geometry_settings.final_approach_distance;
            fprintf(stderr,"MM305 fixed-HAC geometry: variant=%s runwayStation=(%.3f,%.3f) center=(%.3f,%.3f) upper=(%.3f,%.3f) exit=(%.3f,%.3f) exitAlong=%.3f exitCross=%.3f entry=(%.3f,%.3f) lead=%.3f arc=%.3f.\n",
                hac_variant_b_requested()?"B":"default",
                runway_station_e,runway_station_n,
                fixed_plan.cone_center.e,fixed_plan.cone_center.n,upper_e,upper_n,
                fixed_plan.p3.e,fixed_plan.p3.n,exit_along,exit_cross,
                fixed_plan.p0.e,fixed_plan.p0.n,fixed_plan.lead_length,
                fixed_plan.cone_arc_length);
            fprintf(stderr,"MM305 fixed-HAC vertical: startAlt=%.3f finalEntryAlt=%.3f path=%.3f slope=%.3f deg initialSlope=%.3f settledSlope=%.3f transition=%.1f m.\n",
                g->fixed_hac_reference_start_altitude,
                g->fixed_hac_reference_final_altitude,
                fixed_hac_total_path_length(g),
                g->fixed_hac_reference_slope_deg,
                g->fixed_hac_reference_initial_slope_deg,
                g->fixed_hac_reference_settled_slope_deg,
                g->fixed_hac_reference_transition_distance);
            fprintf(stderr,"MM305 fixed-HAC capture: UT %.2f side %+.0f R %.0f arc %.1f deg h %.0f V %.1f roll %.1f minR %.0f requiredLat %.3f availableLat %.3f leadTargetV %.1f.\n",
                t->ut,side,g->hac_radius,
                fixed_plan.cone_arc_length/g->hac_radius*RAD2DEG,t->mean_altitude,
                t->true_air_speed,t->roll,minimum_radius,required_lateral,
                available_lateral,g->fixed_hac_lead_target_speed);
        }
    }

    g->phase=PHASE_TAEM;
    if(!taem_exec_enter(g,t,s,p,aero,cfg,false))return false;
    taem_exec_sync(g,t,s,p,aero,cfg);
    if(!taem_exec_owns_vehicle(&g->taem_exec))return false;
    /* Preserve the Entry limiter state across the ownership handoff. The
       S-turn tail is already blending toward terminal incidence, so clearing
       the pitch/heading jerk state here recreates a visible command step. */
    robust_pid_reset(&g->taem_altitude_pid);robust_pid_reset(&g->speed_pid);
    /* The helper's result is the fixed-path admission decision.  Terminal mode
       being active is not evidence that a fixed path was acquired. */
    return fixed_path_geometry;
}


static void terminal_variant_b_store_tuple(GuidanceMachine*g,
        const HACTransitionPlan*p){
    if(!g||!p||!p->valid)return;
    g->terminal_test_upstream_tuple_valid=true;
    g->terminal_test_upstream_p0_e=p->p0.e;
    g->terminal_test_upstream_p0_n=p->p0.n;
    g->terminal_test_upstream_p3_e=p->p3.e;
    g->terminal_test_upstream_p3_n=p->p3.n;
    g->terminal_test_upstream_center_e=p->cone_center.e;
    g->terminal_test_upstream_center_n=p->cone_center.n;
    g->terminal_test_upstream_cone_start_angle=p->cone_start_angle;
    g->terminal_test_upstream_cone_end_angle=p->cone_end_angle;
    g->terminal_test_upstream_cone_arc_length=p->cone_arc_length;
    g->terminal_test_upstream_lead_curve=p->lead_curve;
    g->terminal_test_upstream_lead_acquisition=p->lead_acquisition;
    g->terminal_test_upstream_lead_start_e=p->lead_start.e;
    g->terminal_test_upstream_lead_start_n=p->lead_start.n;
    g->terminal_test_upstream_acquisition_center_e=p->acquisition_center.e;
    g->terminal_test_upstream_acquisition_center_n=p->acquisition_center.n;
    g->terminal_test_upstream_acquisition_tangent_e=p->acquisition_tangent.e;
    g->terminal_test_upstream_acquisition_tangent_n=p->acquisition_tangent.n;
    g->terminal_test_upstream_acquisition_radius=p->acquisition_radius;
    g->terminal_test_upstream_acquisition_side=p->acquisition_side;
    g->terminal_test_upstream_acquisition_start_angle=p->acquisition_start_angle;
    g->terminal_test_upstream_acquisition_end_angle=p->acquisition_end_angle;
    g->terminal_test_upstream_acquisition_arc_length=p->acquisition_arc_length;
    g->terminal_test_upstream_acquisition_tangent_length=p->acquisition_tangent_length;
    g->terminal_test_upstream_lead_p1_e=p->lead_p1.e;
    g->terminal_test_upstream_lead_p1_n=p->lead_p1.n;
    g->terminal_test_upstream_lead_p2_e=p->lead_p2.e;
    g->terminal_test_upstream_lead_p2_n=p->lead_p2.n;
    g->terminal_test_upstream_lead_start_course=p->lead_start_course;
    g->terminal_test_upstream_lead_end_course=p->lead_end_course;
    g->terminal_test_upstream_lead_length=p->lead_length;
}

static bool terminal_variant_b_load_tuple(const GuidanceMachine*g,
        HACTransitionPlan*out){
    if(out)memset(out,0,sizeof(*out));
    if(!g||!out||!g->terminal_test_upstream_tuple_valid||
       !(g->terminal_test_upstream_planned_radius>0.0))return false;
    out->valid=true;
    out->heading_cone=true;
    out->p0=(HACPoint2){g->terminal_test_upstream_p0_e,
        g->terminal_test_upstream_p0_n};
    out->p3=(HACPoint2){g->terminal_test_upstream_p3_e,
        g->terminal_test_upstream_p3_n};
    out->cone_center=(HACPoint2){g->terminal_test_upstream_center_e,
        g->terminal_test_upstream_center_n};
    out->cone_start_angle=g->terminal_test_upstream_cone_start_angle;
    out->cone_end_angle=g->terminal_test_upstream_cone_end_angle;
    out->cone_arc_length=g->terminal_test_upstream_cone_arc_length;
    out->lead_curve=g->terminal_test_upstream_lead_curve;
    out->lead_acquisition=g->terminal_test_upstream_lead_acquisition;
    out->lead_start=(HACPoint2){g->terminal_test_upstream_lead_start_e,
        g->terminal_test_upstream_lead_start_n};
    out->acquisition_center=(HACPoint2){
        g->terminal_test_upstream_acquisition_center_e,
        g->terminal_test_upstream_acquisition_center_n};
    out->acquisition_tangent=(HACPoint2){
        g->terminal_test_upstream_acquisition_tangent_e,
        g->terminal_test_upstream_acquisition_tangent_n};
    out->acquisition_radius=g->terminal_test_upstream_acquisition_radius;
    out->acquisition_side=g->terminal_test_upstream_acquisition_side;
    out->acquisition_start_angle=g->terminal_test_upstream_acquisition_start_angle;
    out->acquisition_end_angle=g->terminal_test_upstream_acquisition_end_angle;
    out->acquisition_arc_length=g->terminal_test_upstream_acquisition_arc_length;
    out->acquisition_tangent_length=g->terminal_test_upstream_acquisition_tangent_length;
    out->lead_p1=(HACPoint2){g->terminal_test_upstream_lead_p1_e,
        g->terminal_test_upstream_lead_p1_n};
    out->lead_p2=(HACPoint2){g->terminal_test_upstream_lead_p2_e,
        g->terminal_test_upstream_lead_p2_n};
    out->lead_start_course=g->terminal_test_upstream_lead_start_course;
    out->lead_end_course=g->terminal_test_upstream_lead_end_course;
    out->lead_length=g->terminal_test_upstream_lead_length;
    return isfinite(out->p0.e)&&isfinite(out->p0.n)&&
        isfinite(out->cone_arc_length)&&out->cone_arc_length>0.0&&
        isfinite(out->lead_length)&&out->lead_length>=0.0;
}

static void terminal_variant_b_store_vertical_profile(GuidanceMachine*g,
        const GuidanceMachine*profile){
    if(!g)return;
    g->terminal_test_upstream_vertical_profile_valid=false;
    if(!profile||!profile->fixed_hac_reference_valid)return;
    g->terminal_test_upstream_profile_start_altitude=
        profile->fixed_hac_reference_start_altitude;
    g->terminal_test_upstream_profile_final_altitude=
        profile->fixed_hac_reference_final_altitude;
    g->terminal_test_upstream_profile_slope_deg=
        profile->fixed_hac_reference_slope_deg;
    g->terminal_test_upstream_profile_initial_slope_deg=
        profile->fixed_hac_reference_initial_slope_deg;
    g->terminal_test_upstream_profile_settled_slope_deg=
        profile->fixed_hac_reference_settled_slope_deg;
    g->terminal_test_upstream_profile_final_slope_deg=
        profile->fixed_hac_reference_final_slope_deg;
    g->terminal_test_upstream_profile_transition_distance=
        profile->fixed_hac_reference_transition_distance;
    g->terminal_test_upstream_profile_exit_transition_distance=
        profile->fixed_hac_reference_exit_transition_distance;
    g->terminal_test_upstream_vertical_profile_valid=true;
}

static bool terminal_variant_b_load_vertical_profile(const GuidanceMachine*g,
        GuidanceMachine*out){
    if(out)memset(out,0,sizeof(*out));
    if(!g||!out||!g->terminal_test_upstream_vertical_profile_valid||
       !g->terminal_test_upstream_tuple_valid)return false;
    out->fixed_hac_reference_valid=true;
    out->fixed_hac_reference_start_altitude=
        g->terminal_test_upstream_profile_start_altitude;
    out->fixed_hac_reference_final_altitude=
        g->terminal_test_upstream_profile_final_altitude;
    out->fixed_hac_reference_slope_deg=
        g->terminal_test_upstream_profile_slope_deg;
    out->fixed_hac_reference_initial_slope_deg=
        g->terminal_test_upstream_profile_initial_slope_deg;
    out->fixed_hac_reference_settled_slope_deg=
        g->terminal_test_upstream_profile_settled_slope_deg;
    out->fixed_hac_reference_final_slope_deg=
        g->terminal_test_upstream_profile_final_slope_deg;
    out->fixed_hac_reference_transition_distance=
        g->terminal_test_upstream_profile_transition_distance;
    out->fixed_hac_reference_exit_transition_distance=
        g->terminal_test_upstream_profile_exit_transition_distance;
    out->hac_transition_lead_length=g->terminal_test_upstream_lead_length;
    out->hac_transition_cone_arc_length=g->terminal_test_upstream_cone_arc_length;
    return isfinite(out->fixed_hac_reference_start_altitude)&&
        isfinite(out->fixed_hac_reference_final_altitude)&&
        isfinite(out->fixed_hac_reference_slope_deg)&&
        isfinite(out->hac_transition_lead_length)&&
        isfinite(out->hac_transition_cone_arc_length)&&
        out->hac_transition_cone_arc_length>0.0;
}
