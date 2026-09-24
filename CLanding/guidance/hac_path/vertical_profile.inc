       void hac_transition_store(GuidanceMachine*g,const HACTransitionPlan*p,double response_time){
    g->hac_transition_active=p&&p->valid;
    g->hac_transition_heading_cone=g->hac_transition_active&&p->heading_cone;
    g->hac_transition_lead_curve=g->hac_transition_active&&p->lead_curve;
    g->hac_transition_lead_acquisition=g->hac_transition_active&&p->lead_acquisition;
    g->hac_transition_progress=0;g->hac_transition_lead_progress=0;
    if(!g->hac_transition_active){g->hac_transition_length=0;g->hac_transition_lead_length=0;g->hac_transition_exit_speed=0;g->hac_transition_heading_cone=false;g->hac_transition_lead_curve=false;g->hac_transition_lead_acquisition=false;return;}
    g->hac_transition_lead_start_e=p->lead_start.e;g->hac_transition_lead_start_n=p->lead_start.n;
    g->hac_transition_acquisition_center_e=p->acquisition_center.e;
    g->hac_transition_acquisition_center_n=p->acquisition_center.n;
    g->hac_transition_acquisition_tangent_e=p->acquisition_tangent.e;
    g->hac_transition_acquisition_tangent_n=p->acquisition_tangent.n;
    g->hac_transition_acquisition_radius=p->acquisition_radius;
    g->hac_transition_acquisition_side=p->acquisition_side;
    g->hac_transition_acquisition_start_angle=p->acquisition_start_angle;
    g->hac_transition_acquisition_end_angle=p->acquisition_end_angle;
    g->hac_transition_acquisition_arc_length=p->acquisition_arc_length;
    g->hac_transition_acquisition_tangent_length=p->acquisition_tangent_length;
    g->hac_transition_lead_p1_e=p->lead_p1.e;g->hac_transition_lead_p1_n=p->lead_p1.n;
    g->hac_transition_lead_p2_e=p->lead_p2.e;g->hac_transition_lead_p2_n=p->lead_p2.n;
    g->hac_transition_lead_length=p->lead_length;
    g->hac_transition_lead_start_course=p->lead_start_course;
    g->hac_transition_lead_end_course=p->lead_end_course;
    g->hac_transition_p0_e=p->p0.e;g->hac_transition_p0_n=p->p0.n;g->hac_transition_p1_e=p->p1.e;g->hac_transition_p1_n=p->p1.n;
    g->hac_transition_p2_e=p->p2.e;g->hac_transition_p2_n=p->p2.n;g->hac_transition_p3_e=p->p3.e;g->hac_transition_p3_n=p->p3.n;
    g->hac_transition_cone_center_e=p->cone_center.e;g->hac_transition_cone_center_n=p->cone_center.n;
    g->hac_transition_cone_start_angle=p->cone_start_angle;
    g->hac_transition_cone_end_angle=p->cone_end_angle;
    g->hac_transition_cone_arc_length=p->cone_arc_length;
    g->hac_transition_length=p->length;g->hac_transition_end_angle=p->end_angle;g->hac_transition_exit_speed=p->exit_speed;g->hac_transition_response_time=response_time;
}

/* The fixed circle owns the lateral geometry, but its vertical reference must
   start where the shuttle actually is when MM305 commits it.  Deriving the
   first HAC altitude by walking backwards from the runway at the nominal
   terminal slope can put the first reference point many kilometres above the
   vehicle.  Store one finite descent profile over the disjoint lead+arc path;
   the lead and analytic circle remain separate lateral objects.  The vertical
   slope also starts at the measured flight-path angle and settles over a
   finite distance already contained in that same lead+arc path.  This is a
   profile transition, not another distance budget or lateral path segment. */
       bool fixed_hac_vertical_profile_latch(GuidanceMachine*g,
        double start_altitude,double start_flight_path_angle,
        double start_horizontal_speed,double response_time,
        double lead_length,double arc_length,
        const LandingSite*site,const GuidanceSettings*s){
    if(!g||!site||!s||!isfinite(start_altitude)||
       !isfinite(start_flight_path_angle)||
       !isfinite(start_horizontal_speed)||!isfinite(response_time)||
       !isfinite(lead_length)||!isfinite(arc_length)||
       !(lead_length>=0.0)||!(arc_length>0.0))return false;
    double final_distance=g->terminal_glide_mode&&
        g->terminal_test_final_approach_distance>0.0?
        g->terminal_test_final_approach_distance:s->final_approach_distance;
    double final_slope=g->terminal_glide_mode&&
        g->terminal_test_glide_slope>0.0?
        g->terminal_test_glide_slope:s->final_glide_slope;
    double final_altitude=site->altitude+
        final_distance*tan(final_slope*DEG2RAD);
    double total=lead_length+arc_length;
    double drop=start_altitude-final_altitude;
    if(!isfinite(final_altitude)||!isfinite(total)||!(total>1.0)||
       !isfinite(drop)||drop<0.0)return false;
    g->fixed_hac_reference_start_altitude=start_altitude;
    g->fixed_hac_reference_final_altitude=final_altitude;
    g->fixed_hac_reference_slope_deg=atan2(drop,total)*RAD2DEG;
    g->fixed_hac_reference_initial_slope_deg=g->fixed_hac_reference_slope_deg;
    g->fixed_hac_reference_settled_slope_deg=g->fixed_hac_reference_slope_deg;
    g->fixed_hac_reference_final_slope_deg=clampd(final_slope,0.0,75.0);
    g->fixed_hac_reference_transition_distance=0.0;
    g->fixed_hac_reference_exit_transition_distance=0.0;

    /* Match the first vertical tangent to the measured velocity vector and,
       whenever the available HAC distance permits it, also match the outer-final
       glide tangent at the circle exit.  A smoothstep transition has mean tangent
       0.5*(tan(initial)+tan(final)) over its transition distance L, followed by
       the final tangent for the remaining distance.  Solving the exact altitude
       integral gives

           L = 2 * (drop - tan(final)*total) /
                   (tan(initial) - tan(final)).

       This removes the former underconstrained profile which could satisfy exit
       altitude while still arriving at the runway tangent near -35..-40 deg FPA.
       The lateral lead/circle geometry is unchanged and L is entirely contained
       in the already committed lead+arc distance. */
    double initial_slope=clampd(-start_flight_path_angle,0.0,75.0);
    double initial_tan=tan(initial_slope*DEG2RAD);
    double final_profile_slope=g->fixed_hac_reference_final_slope_deg;
    double final_tan=tan(final_profile_slope*DEG2RAD);
    bool endpoint_profile=false;
    double denominator=initial_tan-final_tan;
    if(fabs(denominator)>sqrt(DBL_EPSILON)){
        double solved_transition=2.0*(drop-final_tan*total)/denominator;
        double tolerance=sqrt(DBL_EPSILON)*fmax(1.0,total);
        if(isfinite(solved_transition)&&solved_transition>1.0&&
           solved_transition<=total+tolerance){
            solved_transition=clampd(solved_transition,0.0,total);
            g->fixed_hac_reference_initial_slope_deg=initial_slope;
            g->fixed_hac_reference_settled_slope_deg=final_profile_slope;
            g->fixed_hac_reference_transition_distance=solved_transition;
            endpoint_profile=true;
        }
    }else if(fabs(drop-final_tan*total)<=
             sqrt(DBL_EPSILON)*fmax(1.0,drop)){
        g->fixed_hac_reference_initial_slope_deg=final_profile_slope;
        g->fixed_hac_reference_settled_slope_deg=final_profile_slope;
        g->fixed_hac_reference_transition_distance=0.0;
        endpoint_profile=true;
    }

    if(!endpoint_profile){
        /* A short, energy-feasible HAC can require more altitude loss than a
           monotonic initial->final tangent profile can provide.  Use two finite
           smoothstep response windows: measured FPA -> middle descent, then
           middle descent -> outer-final tangent.  Solve the middle tangent from
           the exact altitude integral; no lateral distance or circle geometry is
           added. */
        double response_window=clampd(fmax(4.0,response_time*1.25),4.0,8.0);
        double transition=lead_length>1.0?lead_length:fmin(total*.25,
            fmax(0.0,start_horizontal_speed)*response_window);
        transition=clampd(transition,0.0,total*.45);
        if(transition>1.0&&initial_tan>0.0&&drop>0.0)
            transition=fmin(transition,drop/(initial_tan*.5)*.9);
        /* The exit transition must include both the finite pitch response and
           the subsequent FPA-state recovery.  One response distance proved
           sufficient to move AoA but left the flown HAC about 4 deg steep at
           the runway tangent.  Reserve two measured response horizons for the
           recovery while keeping it inside the already committed arc. */
        double exit_transition=fmin(total*.35,
            fmax(0.0,start_horizontal_speed)*response_window*2.0);
        exit_transition=clampd(exit_transition,0.0,total*.35);
        double combined=transition+exit_transition;
        if(combined>total*.70&&combined>DBL_MIN){
            double scale=total*.70/combined;
            transition*=scale;
            exit_transition*=scale;
        }
        double middle_denominator=total-.5*(transition+exit_transition);
        if(transition>1.0&&exit_transition>1.0&&middle_denominator>1.0){
            double settled_tan=(drop-.5*initial_tan*transition-
                .5*final_tan*exit_transition)/middle_denominator;
            double settled_slope=isfinite(settled_tan)&&settled_tan>=0.0?
                atan(settled_tan)*RAD2DEG:NAN;
            if(isfinite(settled_slope)&&settled_slope>=0.0&&settled_slope<=75.0){
                g->fixed_hac_reference_initial_slope_deg=initial_slope;
                g->fixed_hac_reference_settled_slope_deg=settled_slope;
                g->fixed_hac_reference_transition_distance=transition;
                g->fixed_hac_reference_exit_transition_distance=exit_transition;
                endpoint_profile=true;
            }
        }
        if(!endpoint_profile&&transition>1.0&&isfinite(transition)&&transition<total){
            /* Last-resort compatibility path for states whose exact endpoint
               profile is genuinely outside the bounded response envelope. */
            double settled_tan=(drop-.5*initial_tan*transition)/
                (total-.5*transition);
            if(isfinite(settled_tan)&&settled_tan>=0.0){
                g->fixed_hac_reference_initial_slope_deg=initial_slope;
                g->fixed_hac_reference_settled_slope_deg=atan(settled_tan)*RAD2DEG;
                g->fixed_hac_reference_transition_distance=transition;
                g->fixed_hac_reference_exit_transition_distance=0.0;
            }
        }
    }
    g->fixed_hac_reference_valid=true;
    return true;
}

       double fixed_hac_total_path_length(const GuidanceMachine*g){
    if(!g)return 0.0;
    return fmax(0.0,g->hac_transition_lead_length)+
        fmax(0.0,g->hac_transition_cone_arc_length);
}

       double fixed_hac_reference_slope(const GuidanceMachine*g,
        double remaining){
    if(!g||!g->fixed_hac_reference_valid)return NAN;
    double total=fixed_hac_total_path_length(g);
    if(!(total>0.0)||!isfinite(g->fixed_hac_reference_slope_deg))return NAN;
    double path_remaining=clampd(remaining,0.0,total);
    double transition=clampd(g->fixed_hac_reference_transition_distance,
        0.0,total);
    double exit_transition=clampd(g->fixed_hac_reference_exit_transition_distance,
        0.0,fmax(0.0,total-transition));
    if(!(transition>1.0)||
       !isfinite(g->fixed_hac_reference_initial_slope_deg)||
       !isfinite(g->fixed_hac_reference_settled_slope_deg))
        return g->fixed_hac_reference_slope_deg;
    double travelled=total-path_remaining;
    double initial_tan=tan(g->fixed_hac_reference_initial_slope_deg*DEG2RAD);
    double settled_tan=tan(g->fixed_hac_reference_settled_slope_deg*DEG2RAD);
    if(travelled<transition){
        double u=clampd(travelled/transition,0.0,1.0);
        double q=u*u*(3.0-2.0*u);
        return atan(initial_tan+(settled_tan-initial_tan)*q)*RAD2DEG;
    }
    if(exit_transition>1.0&&isfinite(g->fixed_hac_reference_final_slope_deg)){
        double exit_start=total-exit_transition;
        if(travelled>exit_start){
            double u=clampd((travelled-exit_start)/exit_transition,0.0,1.0);
            double q=u*u*(3.0-2.0*u);
            double final_tan=tan(g->fixed_hac_reference_final_slope_deg*DEG2RAD);
            return atan(settled_tan+(final_tan-settled_tan)*q)*RAD2DEG;
        }
    }
    return g->fixed_hac_reference_settled_slope_deg;
}

       double fixed_hac_reference_altitude(const GuidanceMachine*g,
        double remaining){
    if(!g||!g->fixed_hac_reference_valid)return NAN;
    double total=fixed_hac_total_path_length(g);
    if(!(total>0.0)||!isfinite(g->fixed_hac_reference_final_altitude)||
       !isfinite(g->fixed_hac_reference_slope_deg)||
       !isfinite(g->fixed_hac_reference_start_altitude))return NAN;
    double path_remaining=clampd(remaining,0.0,total);
    if(path_remaining<=0.0)return g->fixed_hac_reference_final_altitude;
    if(path_remaining>=total)return g->fixed_hac_reference_start_altitude;

    double transition=clampd(g->fixed_hac_reference_transition_distance,
        0.0,total);
    double exit_transition=clampd(g->fixed_hac_reference_exit_transition_distance,
        0.0,fmax(0.0,total-transition));
    if(!(transition>1.0)||
       !isfinite(g->fixed_hac_reference_initial_slope_deg)||
       !isfinite(g->fixed_hac_reference_settled_slope_deg))
        return g->fixed_hac_reference_final_altitude+
            path_remaining*tan(g->fixed_hac_reference_slope_deg*DEG2RAD);

    double travelled=total-path_remaining;
    double initial_tan=tan(g->fixed_hac_reference_initial_slope_deg*DEG2RAD);
    double settled_tan=tan(g->fixed_hac_reference_settled_slope_deg*DEG2RAD);
    double drop;
    if(travelled<transition){
        double u=clampd(travelled/transition,0.0,1.0);
        /* Integral of smoothstep(0..u) is u^3 - u^4/2. */
        double eased_integral=u*u*u-.5*u*u*u*u;
        drop=initial_tan*travelled+
            (settled_tan-initial_tan)*transition*eased_integral;
    }else{
        double transition_drop=transition*(initial_tan+settled_tan)*.5;
        double exit_start=total-exit_transition;
        if(!(exit_transition>1.0)||travelled<=exit_start){
            drop=transition_drop+settled_tan*(travelled-transition);
        }else{
            double before_exit=transition_drop+
                settled_tan*(exit_start-transition);
            double exit_travel=travelled-exit_start;
            double u=clampd(exit_travel/exit_transition,0.0,1.0);
            double eased_integral=u*u*u-.5*u*u*u*u;
            double final_tan=isfinite(g->fixed_hac_reference_final_slope_deg)?
                tan(g->fixed_hac_reference_final_slope_deg*DEG2RAD):settled_tan;
            drop=before_exit+settled_tan*exit_travel+
                (final_tan-settled_tan)*exit_transition*eased_integral;
        }
    }
    return g->fixed_hac_reference_start_altitude-drop;
}

/* If the vehicle has physically left the finite acquisition bridge before it
   reaches the anchored circle, give the bridge one state-derived restart.  The
   restart changes neither the fixed circle nor its runway tangent; it only
   makes the one allowed lead begin at the vehicle's current position and
   velocity vector.  A single attempt is latched so this cannot become a
   recapture/redraw loop. */
       bool hac_fixed_lead_rebase_from_vector(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        const LandingConfiguration*cfg,double current_e,double current_n){
    if(!g||!t||!p||!cfg||!g->fixed_alignment_hac_latched||
       !g->hac_transition_active||!g->hac_transition_heading_cone||
       g->hac_transition_lead_rebase_attempted||
       !(g->hac_transition_lead_length>1.0)||
       g->hac_transition_lead_progress>=.995||
       !isfinite(current_e)||!isfinite(current_n)||!isfinite(course)||
       !isfinite(g->hac_transition_lead_end_course)||fabs(g->hac_side)!=1.0)
        return false;

    HACTransitionPlan old_lead={.valid=true,
        .p0={g->hac_transition_lead_start_e,g->hac_transition_lead_start_n},
        .p1={g->hac_transition_lead_p1_e,g->hac_transition_lead_p1_n},
        .p2={g->hac_transition_lead_p2_e,g->hac_transition_lead_p2_n},
        .p3={g->hac_transition_p0_e,g->hac_transition_p0_n},
        .length=g->hac_transition_lead_length};
    double u=hac_bezier_nearest_u(&old_lead,current_e,current_n);
    HACPoint2 point=hac_bezier_point(&old_lead,u);
    double lead_error=hypot(current_e-point.e,current_n-point.n);
    double lead_tangent=norm_deg(atan2(
        hac_bezier_derivative(&old_lead,u).e,
        hac_bezier_derivative(&old_lead,u).n)*RAD2DEG);
    double course_error=fabs(norm_signed_deg(lead_tangent-course));
    double position_tolerance=fmax(350.0,g->hac_radius*.12);
    if(!(lead_error>position_tolerance))return false;

    /* Consume the one-shot retry even if the replacement cannot be certified;
       retaining the old frozen path is safer than repeatedly redrawing it. */
    g->hac_transition_lead_rebase_attempted=true;
    HACTransitionPlan replacement={0};
    GuidanceSettings geometry_settings=cfg->guidance;
    if(g->terminal_test_final_approach_distance>0.0)
        geometry_settings.final_approach_distance=
            g->terminal_test_final_approach_distance;
    if(!hac_fixed_alignment_plan(&replacement,current_e,current_n,course,
            g->hac_transition_lead_end_course,&cfg->site,&geometry_settings,
            g->hac_radius,g->hac_side)){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
            fprintf(stderr,"MM305 fixed-HAC vector lead retained: UT %.2f residual %.0f m course error %.1f deg; no regular replacement lead.\n",
                t->ut,lead_error,course_error);
        return false;
    }
    if(hypot(replacement.p3.e-g->hac_transition_p3_e,
             replacement.p3.n-g->hac_transition_p3_n)>1e-3||
       hypot(replacement.cone_center.e-g->hac_transition_cone_center_e,
             replacement.cone_center.n-g->hac_transition_cone_center_n)>1e-3||
       fabs(replacement.cone_arc_length-g->hac_transition_cone_arc_length)>1e-3){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
            fprintf(stderr,"MM305 fixed-HAC vector lead rejected: UT %.2f would alter the frozen circle.\n",t->ut);
        return false;
    }
    double response_time=g->hac_transition_response_time;
    if(!fixed_hac_vertical_profile_latch(g,t->mean_altitude,
            t->flight_path_angle,t->horizontal_speed,response_time,
            replacement.lead_length,replacement.cone_arc_length,
            &cfg->site,&cfg->guidance))return false;
    double old_progress=g->hac_transition_lead_progress;
    double old_remaining=(1.0-clampd(old_progress,0.0,1.0))*old_lead.length;
    hac_transition_store(g,&replacement,response_time);
    g->hac_transition_handoff_lateral_valid=false;
    g->hac_transition_handoff_lateral_acceleration=0.0;
    g->hac_transition_handoff_blend=0.0;
    g->hac_transition_handoff_aoa_valid=false;
    g->hac_transition_handoff_aoa=0.0;
    g->hac_remaining=0.0;
    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&
       !g->diagnostic_shadow){
        fprintf(stderr,"HAC_LEAD_STATE_RESET ut=%.6f reason=vector-rebase "
            "oldProgress=%.6f newProgress=%.6f oldRemaining=%.6f "
            "newRemaining=%.6f oldBrakeAoA=%.6f newBrakeAoA=nan "
            "rebaseCount=%u oldLead=%.6f newLead=%.6f\n",
            t->ut,old_progress,0.0,old_remaining,replacement.lead_length,
            NAN,g->hac_energy_audit_rebase_count+1,
            old_lead.length,replacement.lead_length);
    }
    g->hac_energy_audit_rebase_count++;
    g->hac_transition_progress=0.0;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
        fprintf(stderr,"MM305 fixed-HAC vector lead rebase: UT %.2f residual %.0f m course error %.1f deg start=(%.0f,%.0f) course %.1f lead %.0f arc %.0f.\n",
            t->ut,lead_error,course_error,current_e,current_n,course,
            replacement.lead_length,replacement.cone_arc_length);
    return true;
}

