static bool terminal_test_start_spiral(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!isfinite(t->ut)||!isfinite(course)||
       !isfinite(t->latitude)||!isfinite(t->longitude)||
       !isfinite(t->true_air_speed)||!isfinite(t->horizontal_speed)||
       !isfinite(t->mean_altitude)||!(t->true_air_speed>0.0)||
       !(t->horizontal_speed>0.0))
        return false;

    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;

    /*
     * The search envelope is derived from demonstrated turning authority.
     * There is no fixed HAC-radius contract inside the selector: the mission
     * contract decides whether MM304 may hand off, while MM305 chooses whatever
     * radius the live vehicle can actually fly inside the remaining path budget.
     */
    double bank=fmin(fabs(v->maximum_bank_angle),dynamic_bank_limit(t,v));
    if(!(bank>0.0))return false;
    double live_turn=live_turn_radius(t,aero,v,bank);
    double projected_turn=terminal_test_projected_turn_radius(t,p,aero,v,bank);
    double min_turn=INFINITY;
    if(isfinite(live_turn)&&live_turn>0.0)min_turn=live_turn;
    if(isfinite(projected_turn)&&projected_turn>0.0)
        min_turn=fmin(min_turn,projected_turn);
    if(!isfinite(min_turn))return false;

    double energy_total_path=0.0;
    EnergyPathEnvelope energy_envelope={0};
    double energy_slope=terminal_test_energy_selected_slope(
        g,t,p,aero,cfg,&energy_total_path,&energy_envelope);
    if(!(energy_total_path>0.0)||!isfinite(energy_slope)||
       !energy_envelope.valid)return false;

    double minimum_path_slope=0.0,maximum_path_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_path_slope,&maximum_path_slope);
    if(!(minimum_path_slope>0.0)||maximum_path_slope<minimum_path_slope)
        return false;

    double gate_height=fmax(0.0,g->terminal_test_preflare_altitude);
    double slope_tangent=tan(energy_slope*DEG2RAD);
    if(!(slope_tangent>0.0))return false;
    double gate_ground_distance=gate_height/slope_tangent;

    GeoPoint site_point={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint current_point={t->latitude,t->longitude,t->mean_altitude};
    double current_e=0.0,current_n=0.0;
    local_offsets(site_point,current_point,p->radius,&current_e,&current_n);

    double effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0;
    double effective_bank=fmin(bank*effectiveness,nextafter(90.0,0.0));
    double live_lift=live_lift_accel(t,aero,v);
    if(!(live_lift>0.0))return false;
    double max_lateral=live_lift*fabs(sin(effective_bank*DEG2RAD));
    if(!(max_lateral>0.0))return false;

    double transition_drag=terminal_expected_speed_loss_accel(g,t,p,aero,v);
    if(!(transition_drag>=0.0)||!isfinite(transition_drag))return false;

    double circle_design_speed=fmax(g->terminal_test_preflare_target_speed,
        g->terminal_test_preflare_min_speed);
    if(!(circle_design_speed>0.0))
        circle_design_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);

    /* Lift authority scales approximately with V^2 for the same aerodynamic
       state, so scale the demonstrated live turn radius to the terminal design
       speed.  Course-rate authority supplies an independent lower radius. */
    double design_scale=circle_design_speed/t->true_air_speed;
    double design_turn=min_turn*design_scale*design_scale;
    double design_rate_cap=hac_course_rate_cap_for_speed(circle_design_speed);
    if(!(design_rate_cap>0.0))return false;
    double tracking_radius=circle_design_speed/(design_rate_cap*DEG2RAD);
    double min_radius=fmax(design_turn,tracking_radius);

    /*
     * A useful circle cannot have a radius larger than the remaining terminal
     * ground path and still consume a meaningful amount of that path.  The
     * planet radius is the geometric upper bound for the local-plane model.
     */
    double max_radius=fmin(p->radius,fmax(min_radius,energy_total_path));
    if(!(max_radius>=min_radius))return false;

    double configured_final_distance=s->final_approach_distance;
    double height=fmax(0.0,t->mean_altitude-
        (cfg->site.altitude+g->terminal_test_preflare_altitude));
    double preferred_total_path=energy_total_path;
    double min_total_path=fmax(0.0,
        height/tan(maximum_path_slope*DEG2RAD));
    double max_total_path=fmax(0.0,
        height/tan(minimum_path_slope*DEG2RAD));
    if(max_total_path<min_total_path)return false;

    const int radius_samples=13;
    HACTransitionPlan best_transition={0},fallback_transition={0};
    double best_radius=0.0,best_side=1.0,best_slope=0.0,best_response=0.0,
        best_final_distance=configured_final_distance;
    double fallback_radius=0.0,fallback_side=1.0,fallback_slope=0.0,
        fallback_response=0.0,fallback_final_distance=configured_final_distance;
    double best_path_error=INFINITY,best_control_use=INFINITY;
    double best_execution_time=INFINITY;
    double fallback_violation=INFINITY,fallback_path_error=INFINITY,
        fallback_control_use=INFINITY;
    double fallback_execution_time=INFINITY;
    bool fallback_geometry_bad=false,fallback_energy_bad=false;

    for(int ri=0;ri<radius_samples;ri++){
        double u=(double)ri/(double)(radius_samples-1);
        /* Quadratic spacing is only numerical resolution: it places more
           samples near the authority-limited radius without changing decisions. */
        double radius=min_radius+(max_radius-min_radius)*u*u;

        for(int side=-1;side<=1;side+=2){
            double desired_lateral=circle_design_speed*circle_design_speed/radius;
            if(desired_lateral>max_lateral)continue;
            double desired_sine=desired_lateral/live_lift;
            if(desired_sine<0.0||desired_sine>1.0)continue;
            double candidate_bank=asin(desired_sine)/effectiveness*RAD2DEG;
            if(!isfinite(candidate_bank)||fabs(candidate_bank)>bank)continue;
            candidate_bank*=side;

            double response_time=hac_response_lead_time(t,s,candidate_bank);
            if(!(response_time>=0.0)||!isfinite(response_time))continue;

            /*
             * Propagate response using measured force/drag.  No synthetic
             * deceleration wait or safety multiplier is added: the Bezier
             * itself owns all path after the actuator response.
             */
            double response_distance=t->horizontal_speed*response_time;
            double projected_v2=t->true_air_speed*t->true_air_speed-
                2.0*transition_drag*fmax(0.0,response_distance);
            double projected_join_speed=sqrt(fmax(
                circle_design_speed*circle_design_speed,projected_v2));

            double future_e=0.0,future_n=0.0,future_course=course;
            hac_projected_local_state(t,&cfg->site,p->radius,course,response_time,
                candidate_bank,s->taem_roll_rate,s->entry_roll_acceleration,
                live_lift,effectiveness,transition_drag,
                &future_e,&future_n,&future_course);

            HACTransitionPlan transition={0};
            double preferred_path=fmax(1000.0,response_distance+radius);
            double min_path=fmax(0.0,preferred_path-fmax(1000.0,radius*.30));
            double max_path=preferred_path+fmax(1000.0,radius*.60);
            if(!hac_transition_plan(&transition,current_e,current_n,
                    future_e,future_n,course,future_course,&cfg->site,s,
                    radius,side,projected_join_speed,circle_design_speed,
                    transition_drag,max_lateral,preferred_path,min_path,max_path))
                continue;

            double runway=cfg->site.runway_heading*DEG2RAD;
            double exit_along=transition.p3.e*sin(runway)+
                transition.p3.n*cos(runway);
            double candidate_final_distance=fmax(0.0,-exit_along);
            double line_to_gate=candidate_final_distance-gate_ground_distance;
            double endpoint_violation=line_to_gate>=0.0?0.0:
                (gate_ground_distance-candidate_final_distance)/
                    fmax(1000.0,gate_ground_distance);
            double fixed_path=transition.lead_length+transition.length+
                transition.arc_remaining;
            double path_to_gate=fixed_path+fmax(0.0,line_to_gate);
            if(!(path_to_gate>0.0))continue;
            double geometric_slope=atan2(height,path_to_gate)*RAD2DEG;
            double path_violation=terminal_normalized_band_violation(
                path_to_gate,min_total_path,max_total_path);
            transition.degraded_path=path_violation>0.0;
            transition.degraded_end=endpoint_violation>0.0;
            transition.violation_score=fmax(transition.violation_score,
                fmax(path_violation,endpoint_violation));
            transition.degraded=transition.violation_score>sqrt(DBL_EPSILON);

            double circle_entry_speed=fmax(circle_design_speed,
                transition.exit_speed);
            double circle_lateral=circle_entry_speed*circle_entry_speed/radius;
            double control_ratio=circle_lateral/fmax(max_lateral,DBL_MIN);
            double circle_rate=circle_entry_speed/radius*RAD2DEG;
            double circle_rate_cap=hac_course_rate_cap_for_speed(
                circle_entry_speed);
            if(!(circle_rate_cap>0.0))continue;

            double slope_violation=terminal_normalized_band_violation(
                geometric_slope,minimum_path_slope,maximum_path_slope);
            double control_violation=terminal_normalized_upper_violation(
                control_ratio,1.0);
            double circle_rate_violation=terminal_normalized_upper_violation(
                circle_rate,circle_rate_cap);
            double path_scale=fmax(fabs(preferred_total_path),DBL_MIN);
            double path_error=fabs(path_to_gate-preferred_total_path)/path_scale;
            double control_use=fmax(control_ratio,
                fmax(circle_rate/circle_rate_cap,
                    transition.peak_course_rate_ratio));

            /* A HAC side is executable only if the current velocity is
             * already entering that circle, or can turn inward within the
             * same bounded course-rate envelope.  This distinguishes the
             * south-side loop at the MM304 perpendicular inlet from the
             * mirror loop that starts by flying away from the vehicle. */
            TerminalCandidate acquisition={.valid=true,.join=transition,
                .kind=TERMINAL_PATH_HAC,.radius=radius,.side=side,
                .final_distance=candidate_final_distance,.slope=geometric_slope,
                .response=response_time};
            terminal_candidate_execution_cost(&acquisition,g,t,course,p,aero,cfg);
            double execution_time=acquisition.execution_time;
            double execution_horizon=acquisition.execution_horizon;
            double execution_violation=terminal_normalized_upper_violation(
                execution_time,execution_horizon);

            double violation=fmax(transition.violation_score,
                fmax(slope_violation,fmax(control_violation,
                    circle_rate_violation)));
            violation=fmax(violation,execution_violation);

            bool geometry_bad=transition.degraded_control||
                transition.degraded_rate||transition.degraded_end||
                control_violation>0.0||circle_rate_violation>0.0;
            bool energy_bad=transition.degraded_path||
                slope_violation>0.0||execution_violation>0.0;

            if(violation<=sqrt(DBL_EPSILON)){
                double path_scale_cmp=fmax(1.0,fmax(fabs(path_error),
                    fabs(best_path_error)));
                bool path_better=!best_transition.valid||
                    path_error<best_path_error-sqrt(DBL_EPSILON)*path_scale_cmp;
                bool path_same=fabs(path_error-best_path_error)<=
                    sqrt(DBL_EPSILON)*path_scale_cmp;
                double execution_scale=fmax(1.0,fmax(fabs(execution_time),
                    fabs(best_execution_time)));
                bool execution_better=execution_time<
                    best_execution_time-sqrt(DBL_EPSILON)*execution_scale;
                bool execution_same=fabs(execution_time-best_execution_time)<=
                    sqrt(DBL_EPSILON)*execution_scale;
                bool better=path_better||
                    (path_same&&(execution_better||
                    (execution_same&&control_use<best_control_use)));
                if(better){
                    best_transition=transition;
                    best_transition.violation_score=0.0;
                    best_radius=radius;best_side=side;
                    best_slope=geometric_slope;best_response=response_time;
                    best_final_distance=candidate_final_distance;
                    best_path_error=path_error;best_control_use=control_use;
                    best_execution_time=execution_time;
                }
            }else{
                double violation_scale=fmax(1.0,fmax(fabs(violation),
                    fabs(fallback_violation)));
                bool violation_better=!fallback_transition.valid||
                    violation<fallback_violation-
                        sqrt(DBL_EPSILON)*violation_scale;
                bool violation_same=fabs(violation-fallback_violation)<=
                    sqrt(DBL_EPSILON)*violation_scale;
                double path_scale_cmp=fmax(1.0,fmax(fabs(path_error),
                    fabs(fallback_path_error)));
                bool path_better=path_error<fallback_path_error-
                    sqrt(DBL_EPSILON)*path_scale_cmp;
                bool path_same=fabs(path_error-fallback_path_error)<=
                    sqrt(DBL_EPSILON)*path_scale_cmp;
                double execution_scale=fmax(1.0,fmax(fabs(execution_time),
                    fabs(fallback_execution_time)));
                bool execution_better=execution_time<
                    fallback_execution_time-sqrt(DBL_EPSILON)*execution_scale;
                bool execution_same=fabs(execution_time-fallback_execution_time)<=
                    sqrt(DBL_EPSILON)*execution_scale;
                /* The commit gate rejects geometry-degraded candidates outright
                   but can fly an energy-only (too high / path length) fallback
                   and dissipate the excess.  A single blended violation score
                   let a slightly unflyable join outrank a flyable-but-high one,
                   so the selector kept choosing candidates the gate always
                   refused and never committed.  Rank flyability first. */
                bool geometry_class_better=fallback_transition.valid&&
                    fallback_geometry_bad&&!geometry_bad;
                bool geometry_class_worse=fallback_transition.valid&&
                    !fallback_geometry_bad&&geometry_bad;
                bool better=geometry_class_better||(!geometry_class_worse&&
                    (violation_better||
                    (violation_same&&(path_better||
                    (path_same&&(execution_better||
                    (execution_same&&control_use<fallback_control_use)))))));
                if(better){
                    fallback_transition=transition;
                    fallback_transition.degraded=true;
                    fallback_transition.violation_score=violation;
                    fallback_radius=radius;fallback_side=side;
                    fallback_slope=clampd(geometric_slope,
                        minimum_path_slope,maximum_path_slope);
                    fallback_response=response_time;
                    fallback_final_distance=candidate_final_distance;
                    fallback_violation=violation;
                    fallback_path_error=path_error;
                    fallback_control_use=control_use;
                    fallback_execution_time=execution_time;
                    fallback_geometry_bad=geometry_bad;
                    fallback_energy_bad=energy_bad;
                    const char*why_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
                    if(why_diag&&strcmp(why_diag,"1")==0&&!g->diagnostic_shadow)
                        fprintf(stderr,"HAC_FALLBACK_WHY ut=%.2f R=%.0f side=%+.0f ctrl=%d rate=%d end=%d path=%d controlRatio=%.3f circleRate=%.3f rateCap=%.3f peakLat=%.3f maxLat=%.3f peakRateRatio=%.3f slope=%.2f slopeBand=%.2f..%.2f execT=%.1f execH=%.1f finalDist=%.0f gateGround=%.0f violation=%.3f\n",
                            t->ut,(double)radius,(double)side,(int)transition.degraded_control,(int)transition.degraded_rate,
                            (int)transition.degraded_end,(int)transition.degraded_path,control_ratio,circle_rate,
                            circle_rate_cap,transition.peak_lateral,max_lateral,
                            transition.peak_course_rate_ratio,geometric_slope,minimum_path_slope,
                            maximum_path_slope,execution_time,execution_horizon,
                            candidate_final_distance,gate_ground_distance,violation);
                }
            }
        }
    }

    bool use_fallback=!best_transition.valid&&fallback_transition.valid;
    if(!best_transition.valid&&!fallback_transition.valid)return false;
    HACTransitionPlan selected=use_fallback?fallback_transition:best_transition;
    double selected_radius=use_fallback?fallback_radius:best_radius;
    double selected_side=use_fallback?fallback_side:best_side;
    double selected_slope=use_fallback?fallback_slope:best_slope;
    double selected_response=use_fallback?fallback_response:best_response;
    double selected_final_distance=use_fallback?
        fallback_final_distance:best_final_distance;

    if(!hac_handoff_geometry_valid(&selected,current_e,current_n,course,
            &cfg->site,s,selected_radius,selected_side))
        return false;

    g->minimum_turn_radius=min_turn;
    g->hac_plan_degraded=selected.degraded;
    g->hac_plan_geometry_degraded=use_fallback?fallback_geometry_bad:false;
    g->hac_plan_energy_degraded=use_fallback?fallback_energy_bad:false;
    g->hac_plan_violation_score=fmax(0.0,selected.violation_score);
    g->hac_side=selected_side;
    g->hac_side_selected=true;
    g->hac_radius=selected_radius;
    g->hac_previous_angle=selected.end_angle;
    g->hac_progress_valid=false;
    g->terminal_test_spiral_active=false;
    g->terminal_test_revolution_remaining=0.0;
    g->terminal_test_glide_slope=selected_slope;
    g->terminal_test_final_approach_distance=selected_final_distance;
    g->hac_remaining=selected.arc_remaining;
    hac_transition_store(g,&selected,selected_response);

    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
        fprintf(stderr,
            "HAC constrained selector: UT %.1f degraded=%d R %.0f side %+.0f exec %.1fs pathErr %.4f violation %.4f controlUse %.4f slope %.2f energySlope %.2f\n",
            t->ut,selected.degraded,selected_radius,selected_side,
            use_fallback?fallback_execution_time:best_execution_time,
            use_fallback?fallback_path_error:best_path_error,
            selected.violation_score,
            use_fallback?fallback_control_use:best_control_use,
            selected_slope,energy_slope);
    }
    return true;
}


/* Terminal prediction and ownership are independent of the phase enum. The
   selector runs on a response-propagated future sample. A fully qualified join
   is preferred, but a regular least-bad join is still retained when every
   candidate violates a preferred envelope; "no feasible HAC" is not the same
   thing as "no HAC plan". No interpolation of Bezier control points is
   permitted: that would bypass the selector's curvature tests. */
static bool terminal_candidate_geometry_regular(const TerminalCandidate*c){
    return c&&c->valid&&!c->geometry_degraded;
}

static bool terminal_candidate_geometry_executable(const TerminalCandidate*c){
    /* Execution reachability is a separate acquisition constraint.  Keep this
       helper for places that truly require both, but do not use it as the
       geometry/control class discriminator. */
    return terminal_candidate_geometry_regular(c)&&
        (!c->execution_evaluated||c->execution_margin>=0.0);
}

static double terminal_candidate_delivery_time(const TerminalCandidate*c){
    if(!c||!c->valid||!c->join.valid||!(c->speed>0.0))return INFINITY;
    double path=fmax(0.0,c->join.lead_length)+
        fmax(0.0,c->join.length)+fmax(0.0,c->join.arc_remaining)+
        fmax(0.0,c->final_distance);
    double forecast=fmax(0.0,c->arrival_ut-c->selected_ut);
    return forecast+fmax(0.0,c->response)+path/c->speed;
}

/* A candidate is not executable merely because its downstream HAC is
 * regular.  The shuttle first has to reach the candidate's response lead and
 * turn onto its initial tangent.  Measure that physical acquisition cost from
 * the current sample so a path whose beautiful loop is already behind the
 * vehicle cannot outrank one that can be started now. */

static double terminal_candidate_constraint_violation(const TerminalCandidate*c){
    if(!c||!c->valid)return INFINITY;
    if(isfinite(c->join.violation_score))
        return fmax(0.0,c->join.violation_score);
    return c->degraded?INFINITY:0.0;
}

/*
 * Geometry executability is a hard prerequisite because a path outside
 * lateral/rate authority cannot be flown. Within the same executable class,
 * order candidates by the selector's dimensionless worst normalized physical
 * constraint violation. A return value <0 means b is better than a.
 */
static int terminal_candidate_constraint_order(const TerminalCandidate*a,
        const TerminalCandidate*b){
    /* Geometry/control executability is the hard class boundary.  A candidate
       whose forecast origin is easier to reach may not displace a path that is
       actually within lateral/rate authority with one that is not.  The old
       ordering checked execution reachability first, which let a degraded HAC
       replace a geometry-clean spline late in MM305. */
    bool a_executable=terminal_candidate_geometry_regular(a);
    bool b_executable=terminal_candidate_geometry_regular(b);
    if(a_executable!=b_executable)return b_executable?-1:1;
    bool a_reachable=!a->execution_evaluated||a->execution_margin>=0.0;
    bool b_reachable=!b->execution_evaluated||b->execution_margin>=0.0;
    if(a_reachable!=b_reachable)return b_reachable?-1:1;

    double av=terminal_candidate_constraint_violation(a);
    double bv=terminal_candidate_constraint_violation(b);
    if(isfinite(bv)&&!isfinite(av))return -1;
    if(!isfinite(bv)&&isfinite(av))return 1;
    if(!isfinite(av)&&!isfinite(bv))return 0;

    double scale=fmax(1.0,fmax(fabs(av),fabs(bv)));
    double tolerance=sqrt(DBL_EPSILON)*scale;
    if(bv<av-tolerance)return -1;
    if(bv>av+tolerance)return 1;
    return 0;
}

static int terminal_candidate_execution_order(const TerminalCandidate*a,
        const TerminalCandidate*b){
    if(!a||!b)return 0;
    if(isfinite(a->execution_time)||isfinite(b->execution_time)){
        if(!isfinite(b->execution_time))return 1;
        if(!isfinite(a->execution_time))return -1;
        double scale=fmax(1.0,fmax(fabs(a->execution_time),
            fabs(b->execution_time)));
        double tolerance=sqrt(DBL_EPSILON)*scale;
        if(b->execution_time<a->execution_time-tolerance)return -1;
        if(b->execution_time>a->execution_time+tolerance)return 1;
    }
    return 0;
}

static bool terminal_candidate_better(const TerminalCandidate*a,const TerminalCandidate*b){
    if(!b||!b->valid)return false;
    if(!a||!a->valid)return true;
    bool a_executable=terminal_candidate_geometry_regular(a);
    bool b_executable=terminal_candidate_geometry_regular(b);
    int order=terminal_candidate_constraint_order(a,b);
    if(order==0&&a_executable&&b_executable)
        order=terminal_candidate_execution_order(a,b);
    if(order<0)return true;
    if(order>0)return false;
    return terminal_candidate_delivery_time(b)<
        terminal_candidate_delivery_time(a);
}

static bool terminal_candidate_refinement_ok(const TerminalCandidate*a,
        const TerminalCandidate*b){
    if(!b||!b->valid)return false;
    if(!a||!a->valid)return true;

    int order=terminal_candidate_constraint_order(a,b);
    if(order>0)return false;
    if(order<0&&!terminal_candidate_geometry_executable(a))return true;

    if(terminal_candidate_geometry_executable(a)&&
       terminal_candidate_geometry_executable(b)){
        int execution=terminal_candidate_execution_order(a,b);
        if(execution<0){
            /* Replacing a path costs the response time of the new join.  A
             * candidate that is only microscopically closer to the current
             * state is not an improvement: the shuttle would spend that
             * margin changing plans instead of flying either one. */
            double improvement=a->execution_time-b->execution_time;
            double switching_time=fmax(fmax(0.0,a->response),
                fmax(0.0,b->response));
            if(isfinite(improvement)&&improvement>switching_time)
                return true;
        }
        if(execution!=0)return false;
        if(a->side!=b->side)return false;
    }

    /*
     * Switching a path is itself a maneuver. Require the new path to save
     * more delivery time than the response time needed to acquire it. This is
     * physical hysteresis rather than a radius/position/percentage threshold.
     */
    double switching_time=fmax(fmax(0.0,a->response),fmax(0.0,b->response));
    return terminal_candidate_delivery_time(b)+switching_time<
        terminal_candidate_delivery_time(a);
}

static bool terminal_candidate_refresh_allowed(const GuidanceMachine*g,
        const TerminalCandidate*a,const TerminalCandidate*b){
    (void)g;
    if(!b||!b->valid)return false;
    if(!a||!a->valid)return true;
    if(terminal_candidate_refinement_ok(a,b))return true;

    /*
     * Once the retained forecast reaches its arrival time it no longer
     * describes a future acquisition state. Freshness may replace it only with
     * a candidate whose executable class and normalized physical violation are
     * no worse; time expiry never relaxes a physical constraint.
     */
    bool expired=isfinite(a->arrival_ut)&&isfinite(b->selected_ut)&&
        b->selected_ut>=a->arrival_ut;
    /* Expiry is not permission to swap mirror sides or restart a live-origin
       preview. Such replacements must pay the response cost above. */
    return expired&&a->arrival_ut>a->selected_ut&&a->side==b->side&&
        terminal_candidate_constraint_order(a,b)<=0&&
        b->execution_time+fmax(a->response,b->response)<a->execution_time;
}

       void terminal_publish_candidate(GuidanceMachine*g){
    const TerminalCandidate*c=&g->terminal_candidate;
    g->terminal_path_kind=c->kind;
    g->hac_radius=c->radius;g->hac_side=c->side;g->hac_side_selected=true;
    g->hac_plan_degraded=c->degraded;
    g->hac_plan_geometry_degraded=c->geometry_degraded;
    g->hac_plan_energy_degraded=c->energy_degraded;
    g->hac_plan_violation_score=terminal_candidate_constraint_violation(c);
    g->hac_commit_blend=0;
    g->hac_previous_angle=c->join.end_angle;g->hac_remaining=c->join.arc_remaining;
    g->hac_progress_valid=false;
    g->terminal_test_glide_slope=c->slope;
    g->terminal_test_final_approach_distance=c->final_distance;
    hac_transition_store(g,&c->join,c->response);
}
static bool terminal_candidate_energy_reachable(const TerminalCandidate*c,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg){
    if(!c||!t||!p||!cfg||!c->valid)return false;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint target=local_point(origin,c->join.lead_start.e,c->join.lead_start.n,
        p->radius,c->altitude);
    double current=rotating_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,p);
    double required=rotating_specific_energy(target.latitude,c->altitude,c->speed,p);
    if(!isfinite(current)||!isfinite(required))return false;
    double scale=fmax(1.0,fmax(fabs(current),fabs(required)));
    return current+sqrt(DBL_EPSILON)*scale>=required;
}

static bool terminal_candidate_expired(const GuidanceMachine*g,
        const TerminalCandidate*c,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg){
    (void)g;
    (void)course;
    if(!c||!t||!c->valid)return false;
    /* A live-origin candidate is anchored at the same sample that selected it
       (selected_ut == arrival_ut).  Its origin is the vehicle, not a future
       point target whose altitude/speed must remain unchanged.  Rechecking
       point-target energy on every later sample would expire this continuous
       path as soon as normal unpowered drag changed the state; the live
       energy/response commitment gate below owns that re-evaluation. */
    if(isfinite(c->selected_ut)&&isfinite(c->arrival_ut)&&
       c->selected_ut==c->arrival_ut)
        return false;
    /*
     * The selector constructs a candidate from a response-propagated future
     * state.  While that origin is still in the future, the vehicle is not
     * supposed to reacquire it as a fresh point target: charging another
     * control-response delay here makes a tangent forecast fail its own
     * reachability test before the commanded path can be flown.  The live
     * lead-capture envelope remains a commitment gate below, once the
     * forecast origin is current.  Until then, freshness is governed by the
     * candidate's future energy contract and its arrival timestamp.
     */
    bool reachable=terminal_candidate_energy_reachable(c,t,p,cfg);
    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&!reachable){
        fprintf(stderr,"HAC candidate expired by energy: UT %.1f arrival %.1f live h %.0f V %.0f cand h %.0f V %.0f lead %.0f %.0f\n",
            t->ut,c->arrival_ut,t->mean_altitude,t->true_air_speed,
            c->altitude,c->speed,c->join.lead_start.e,c->join.lead_start.n);
    }
    if(isfinite(c->arrival_ut)&&c->arrival_ut>t->ut+sqrt(DBL_EPSILON))
        return !reachable;
    /* Once the response-projected origin is current, its forecast clock is no
       longer an available-time budget.  Reusing the target-capture envelope
       here compares the live vehicle with a point target whose arrival time is
       already zero, so it declares every otherwise usable candidate expired
       exactly as its lead reaches the vehicle.  MM305's retained geometry and
       live energy check own this post-arrival decision; the planner will
       replace the candidate on its normal refresh when a better reachable
       topology exists. */
    return !reachable;
}

       bool terminal_prediction_ready(const GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,const LandingConfiguration*cfg){
    return g&&g->terminal_candidate.valid&&
        !terminal_candidate_expired(g,&g->terminal_candidate,t,course,p,cfg);
}

/* A degraded selector result is still a complete, regular path.  Treat the
   preferred control/rate/path envelopes as ranking constraints rather than a
   binary ownership veto: TAEM must not fly straight toward the runway for a
   minute while waiting for a mathematically perfect HAC that may never
   appear.  Mild violations can be frozen early; as the alignment station
   approaches, progressively accept the least-violating regular path.  Truly
   malformed or grossly outside the selector envelope remains preview-only. */
       bool terminal_candidate_operationally_usable(const GuidanceMachine*g,
        const TerminalCandidate*c,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg){
    const char*reason=NULL;
    if(!g||!c||!t||!p||!cfg)reason="null input";
    else if(!c->valid)reason="candidate invalid";
    else if(!c->join.valid)reason="join invalid";
    else if(terminal_candidate_expired(g,c,t,course,p,cfg))reason="candidate expired";
    else if(!isfinite(c->join.length)||!(c->join.length>0.0))reason="join length invalid";
    else if(!isfinite(c->join.arc_remaining)||c->join.arc_remaining<0.0)
        reason="arc invalid";
    else if(!c->join.heading_cone&&!hac_bezier_regular(&c->join))reason="bezier irregular";
    else if(c->kind==TERMINAL_PATH_HAC){
        if(!isfinite(c->radius)||!(c->radius>0.0))reason="HAC radius invalid";
    }else if(c->kind==TERMINAL_PATH_SPLINE){
        /* A spline has no required nominal radius. Its actual curvature/control
           feasibility is already carried by geometry_degraded. */
    }else reason="unknown terminal path kind";
    /* Keep a least-violating path available as a preview, but never freeze a
       path that the selector itself classified as dynamically/geometry
       degraded. The 2026-09-09 live failure committed a 16.2 km degraded HAC
       whose cubic needed substantially more lateral acceleration than the
       aircraft could supply. */
    if(!reason&&c->geometry_degraded)reason="geometry degraded";
    if(reason){
        const char*diag_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        static double last_operational_diag_ut=-INFINITY;
        if(t&&!g->diagnostic_shadow&&diag_env&&strcmp(diag_env,"1")==0&&
           t->ut-last_operational_diag_ut>=1.0){
            fprintf(stderr,
                "HAC candidate unusable: UT %.1f reason=%s valid=%d join=%d kind=%d R=%.0f len=%.0f arc=%.0f geom=%d energy=%d shell=%d degraded=%d peakLat=%.3f rateRatio=%.3f arrival=%.1f age=%.1f\n",
                t->ut,reason,c?c->valid:0,c?c->join.valid:0,c?c->kind:0,
                c?c->radius:NAN,c?c->join.length:NAN,c?c->join.arc_remaining:NAN,
                c?c->geometry_degraded:0,c?c->energy_degraded:0,c?c->shell_degraded:0,
                c?c->degraded:0,c?c->join.peak_lateral:NAN,
                c?c->join.peak_course_rate_ratio:NAN,c?c->arrival_ut:NAN,
                c? t->ut-c->selected_ut:NAN);
            last_operational_diag_ut=t->ut;
        }
        return false;
    }
    return true;
}


static void terminal_project_future_aero_sample(Telemetry*future,const Telemetry*current,
        const PlanetModel*p,const VehicleProfile*v){
    if(!future||!current||!p||!v)return;
    /* A propagated Telemetry value is not a live measurement.  Copying the
       current sample wholesale and changing only altitude/speed leaves q,
       Mach and measured forces tied to the old state.  The HAC selector then
       sees high-speed lift/drag at a low-speed future point and can predict an
       impossibly small circle plus unrealistically rapid deceleration.

       Rebuild the atmosphere at the propagated altitude and transport the
       measured force calibration only through q and the aerodynamic factor
       ratios.  This retains the live vehicle calibration without pretending
       that the original force measurement itself exists in the future. */
    double base_density_now=planet_atmospheric_density(p,current->mean_altitude);
    double density_scale=current->atmospheric_density>0&&base_density_now>1e-9?
        current->atmospheric_density/base_density_now:
        clampd(current->trajectory_density_scale,.35,2.8);
    density_scale=clampd(density_scale,.35,2.8);
    double base_sound_now=planet_atmospheric_speed_of_sound(p,current->mean_altitude);
    double sound_scale=current->speed_of_sound>1&&base_sound_now>1?
        current->speed_of_sound/base_sound_now:1.0;
    sound_scale=clampd(sound_scale,.70,1.30);

    future->atmospheric_density=planet_atmospheric_density(p,future->mean_altitude)*density_scale;
    future->speed_of_sound=planet_atmospheric_speed_of_sound(p,future->mean_altitude)*sound_scale;
    if(future->speed_of_sound<=1.0)future->speed_of_sound=fmax(180.0,current->speed_of_sound);
    future->mach=future->true_air_speed/fmax(future->speed_of_sound,1.0);
    future->dynamic_pressure=.5*fmax(0.0,future->atmospheric_density)*
        future->true_air_speed*future->true_air_speed;
    future->surface_speed=future->true_air_speed;
    future->sideslip=0.0;
    future->static_pressure=0.0;

    double current_incidence=hypot(current->angle_of_attack,current->sideslip);
    double future_incidence=fabs(future->angle_of_attack);
    double current_lf=0.0,current_df=1.0,future_lf=0.0,future_df=1.0;
    aerodynamic_force_factors_mach(current->mach,current_incidence,v,&current_lf,&current_df);
    aerodynamic_force_factors_mach(future->mach,future_incidence,v,&future_lf,&future_df);
    double q_ratio=current->dynamic_pressure>1.0?
        future->dynamic_pressure/current->dynamic_pressure:0.0;
    future->lift_force=0.0;future->drag_force=0.0;
    if(q_ratio>0.0&&isfinite(q_ratio)){
        if(current->lift_force>0.0&&isfinite(current->lift_force))
            future->lift_force=current->lift_force*q_ratio*
                fabs(future_lf)/fmax(.05,fabs(current_lf));
        if(current->drag_force>0.0&&isfinite(current->drag_force))
            future->drag_force=current->drag_force*q_ratio*
                fmax(.05,future_df)/fmax(.05,current_df);
    }
}

static void terminal_projected_force_accels(const Telemetry*current,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double reference_drag,
        double altitude,double speed,double angle_of_attack,double*lift_accel,double*drag_accel){
    if(lift_accel)*lift_accel=0.0;
    if(drag_accel)*drag_accel=fmax(.02,reference_drag);
    if(!current||!p||!v)return;

    double base_density_now=planet_atmospheric_density(p,current->mean_altitude);
    double density_scale=current->atmospheric_density>0.0&&base_density_now>1e-9?
        current->atmospheric_density/base_density_now:
        clampd(current->trajectory_density_scale,.35,2.8);
    density_scale=clampd(density_scale,.35,2.8);
    double density=planet_atmospheric_density(p,altitude)*density_scale;

    double base_sound_now=planet_atmospheric_speed_of_sound(p,current->mean_altitude);
    double sound_scale=current->speed_of_sound>1.0&&base_sound_now>1.0?
        current->speed_of_sound/base_sound_now:1.0;
    sound_scale=clampd(sound_scale,.70,1.30);
    double sound=planet_atmospheric_speed_of_sound(p,altitude)*sound_scale;
    if(sound<=1.0)sound=fmax(180.0,current->speed_of_sound);
    double mach=speed/fmax(sound,1.0);

    double q=.5*fmax(0.0,density)*speed*speed;
    double current_q=current->dynamic_pressure;
    if(current_q<=1.0)
        current_q=.5*fmax(0.0,current->atmospheric_density)*
            current->true_air_speed*current->true_air_speed;
    double q_ratio=current_q>1.0?q/current_q:0.0;

    double current_incidence=hypot(current->angle_of_attack,current->sideslip);
    double current_lf=0.0,current_df=1.0,future_lf=0.0,future_df=1.0;
    aerodynamic_force_factors_mach(current->mach,current_incidence,v,&current_lf,&current_df);
    aerodynamic_force_factors_mach(mach,fabs(angle_of_attack),v,&future_lf,&future_df);

    double measured_lift=current->mass>1.0&&current->lift_force>0.0?
        current->lift_force/current->mass:0.0;
    double projected_lift=measured_lift>0.0&&q_ratio>0.0?
        measured_lift*q_ratio*fabs(future_lf)/fmax(.05,fabs(current_lf)):0.0;
    double modeled_lift=q/fmax(20.0,aero.ballistic_coefficient)*
        fmax(0.0,aero.lift_to_drag)*fabs(future_lf);
    projected_lift=fmax(projected_lift,modeled_lift);

    /* The measured aerodynamic drag is the force anchor (not net speed loss), but
       it is only valid at the live q/Mach/incidence. Transport that anchor to
       the future state instead of holding it constant for the whole horizon.
       This makes drag fall with V^2 (and rise with density) as the vehicle
       descends, matching the distance-domain model used by the HAC join. */
    double projected_drag=fmax(.02,reference_drag);
    if(q_ratio>0.0&&isfinite(q_ratio))
        projected_drag=fmax(.02,reference_drag*q_ratio*
            fmax(.05,future_df)/fmax(.05,current_df));
    double modeled_drag=q/fmax(20.0,aero.ballistic_coefficient)*fmax(.05,future_df);
    if(current_q<=1.0)projected_drag=fmax(projected_drag,modeled_drag);

    if(lift_accel)*lift_accel=clampd(projected_lift,0.0,planet_surface_gravity(p)*3.5);
    if(drag_accel)*drag_accel=clampd(projected_drag,.02,40.0);
}

static void terminal_project_planning_state(const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double horizon,
        Telemetry*future,double*out_e,double*out_n,double*out_course){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    *future=*t;
    horizon=fmax(0.0,horizon);
    if(horizon<=1e-6){
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint point={t->latitude,t->longitude,t->mean_altitude};
        double e=0,n=0;local_offsets(origin,point,p->radius,&e,&n);
        if(out_e)*out_e=e;if(out_n)*out_n=n;if(out_course)*out_course=course;
        return;
    }
    double speed_loss=terminal_expected_speed_loss_accel(g,t,p,aero,v);
    double force_drag=live_drag_accel(t,aero,v);
    double planning_bank=norm_signed_deg(t->roll);
    if(g->entry_control_plan_valid&&t->ut<=g->entry_control_segment_until_ut+1.0)
        planning_bank=g->entry_control_bank;
    /* After the one-way MM304 -> TAEM transfer, propagate the bank that TAEM
       is actually commanding, even before a terminal candidate is usable. The
       v9b 50 km flight handed over at +55.8 deg roll; holding a synthetic +40
       deg bank throughout every planning horizon made fresh candidates turn
       away from the runway while the live limiter was already rolling out. */
    if(g->terminal_region_entered&&isfinite(g->terminal_reference_bank))
        planning_bank=g->terminal_reference_bank;
    planning_bank=clampd(planning_bank,-40.0,40.0);
    double e=0,n=0,future_course=course;
    hac_projected_local_state(t,&cfg->site,p->radius,course,horizon,planning_bank,
        s->taem_roll_rate,fmax(2.0,s->entry_roll_acceleration*.85),
        live_lift_accel(t,aero,v),clampd(t->bank_effectiveness,.35,1.8),speed_loss,
        &e,&n,&future_course);

    double height=t->mean_altitude,speed=t->true_air_speed,fpa=t->flight_path_angle;
    double target_fpa=clampd(t->flight_path_angle,-35.0,-2.0);
    if(g->terminal_region_entered&&isfinite(g->terminal_reference_fpa))
        target_fpa=clampd(g->terminal_reference_fpa,-35.0,-2.0);
    double response=fmax(1.0,hac_response_lead_time(t,s,planning_bank));
    /* Propagate the command actually being flown, not a hypothetical fixed
       HAC incidence. The published future force sample must use this same
       angle of attack as the speed/height integration. */
    double target_aoa=t->angle_of_attack;
    if(g->terminal_region_entered&&isfinite(g->terminal_reference_aoa))
        target_aoa=clampd(g->terminal_reference_aoa,0.0,v->maximum_angle_of_attack);
    double projected_aoa=t->angle_of_attack;
    for(double elapsed=0;elapsed<horizon;elapsed+=.5){
        double step=fmin(.5,horizon-elapsed);
        double commanded_aoa=target_aoa;
        if(g->terminal_region_entered&&isfinite(g->terminal_reference_fpa)){
            /* The live terminal law closes force demand from the measured FPA.
               Rebuild the projected aerodynamic sample before asking for its
               next AoA so a steepening velocity vector cannot be hidden behind
               the one target that was present at the start of this horizon. */
            Telemetry projected=*t;
            projected.mean_altitude=height;
            projected.radar_altitude=fmax(0.0,height-cfg->site.altitude);
            projected.true_air_speed=speed;
            projected.surface_speed=speed;
            projected.horizontal_speed=speed*cos(fpa*DEG2RAD);
            projected.vertical_speed=speed*sin(fpa*DEG2RAD);
            projected.flight_path_angle=fpa;
            projected.angle_of_attack=projected_aoa;
            terminal_project_future_aero_sample(&projected,t,p,v);
            double feedback_aoa=terminal_fpa_force_aoa(&projected,p,aero,v,
                target_fpa);
            if(isfinite(feedback_aoa))
                commanded_aoa=clampd(feedback_aoa,0.0,v->maximum_angle_of_attack);
        }
        double aoa_mix=clampd(step/response,0.0,1.0);
        projected_aoa+=
            (commanded_aoa-projected_aoa)*aoa_mix;
        double lift=0.0,projected_drag=force_drag;
        terminal_projected_force_accels(t,p,aero,v,force_drag,height,speed,projected_aoa,
            &lift,&projected_drag);
        double fpa_rate=fmax(.15,lift/fmax(speed,1.0)*RAD2DEG*.35);
        fpa+=clampd(target_fpa-fpa,-fpa_rate*step,fpa_rate*step);
        double next_height=height+speed*sin(fpa*DEG2RAD)*step;
        double potential_before=rotating_specific_energy(t->latitude,height,0.0,p);
        double potential_after=rotating_specific_energy(t->latitude,next_height,0.0,p);
        double drag_work=projected_drag*speed*step;
        speed=sqrt(fmax(1.0,speed*speed+2.0*(potential_before-potential_after-drag_work)));
        height=next_height;
    }
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint point=local_point(origin,e,n,p->radius,height);
    future->latitude=point.latitude;future->longitude=point.longitude;
    future->mean_altitude=height;future->radar_altitude=fmax(0.0,height-cfg->site.altitude);
    future->ut=t->ut+horizon;future->true_air_speed=speed;
    future->horizontal_speed=speed*cos(fpa*DEG2RAD);
    future->vertical_speed=speed*sin(fpa*DEG2RAD);future->flight_path_angle=fpa;
    future->roll=planning_bank;future->ground_track_heading=future_course;
    future->body_roll_rate=0;future->roll_rate=0;
    future->angle_of_attack=projected_aoa;
    terminal_project_future_aero_sample(future,t,p,v);
    runway_coordinates(point,origin,cfg->site.runway_heading,p->radius,
        &future->runway_along_track,&future->runway_cross_track);
    future->range_to_site=great_circle_distance(point,origin,p->radius);
    if(out_e)*out_e=e;if(out_n)*out_n=n;if(out_course)*out_course=future_course;
}

static bool terminal_spline_candidate_from_future(const GuidanceMachine*g,const Telemetry*live,
        const Telemetry*future,double future_course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,TerminalCandidate*out){
    if(!g||!live||!future||!p||!cfg||!out)return false;
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;

    /*
     * Topology search is admitted by measured/modelled control authority, not a
     * speed multiplier.  A state that cannot control itself at the calibrated
     * minimum speed is not a valid terminal-path origin.
     */
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,future,p,cfg);
    if(!authority.valid||!authority.controllable||authority.speed.margin<0.0||
       future->vertical_speed>=0.0)return false;

    GuidanceMachine trial=*g;
    double energy_ema=g->terminal_energy_loss_accel_ema;
    double speed_ema=g->terminal_speed_loss_accel_ema;
    terminal_glide_initialize(&trial,v,s);
    trial.terminal_energy_loss_accel_ema=energy_ema;
    trial.terminal_speed_loss_accel_ema=speed_ema;

    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint fp={future->latitude,future->longitude,future->mean_altitude};
    double start_e=0.0,start_n=0.0;
    local_offsets(origin,fp,p->radius,&start_e,&start_n);
    GeoPoint lp={live->latitude,live->longitude,live->mean_altitude};
    double live_e=0.0,live_n=0.0;
    local_offsets(origin,lp,p->radius,&live_e,&live_n);
    double response_distance=hypot(start_e-live_e,start_n-live_n);
    double runway=cfg->site.runway_heading*DEG2RAD;
    double final_distance=s->final_approach_distance;
    double end_e=-sin(runway)*final_distance,end_n=-cos(runway)*final_distance;
    double chord=hypot(end_e-start_e,end_n-start_n);
    if(!isfinite(chord)||!(chord>DBL_MIN))return false;

    /*
     * Geometry generation is deliberately independent of the scalar energy
     * budget.  The previous planner first solved for an energy-consuming path
     * length and then stretched the Bezier toward that length.  At high-energy
     * MM305 starts this manufactured 40-80 km curves whose curvature and course
     * rate were not flyable.  Construct a pose/vertical/control path first; the
     * actual curve is energy-certified after it exists.
     */
    double minimum_slope=0.0,maximum_slope=0.0;
    terminal_path_slope_bounds(future,v,s,&minimum_slope,&maximum_slope);
    if(!(minimum_slope>0.0)||!(maximum_slope>=minimum_slope))return false;

    double gate_height=fmax(0.0,trial.terminal_test_preflare_altitude);
    double gate_altitude=cfg->site.altitude+gate_height;
    double gate_speed=fmax(trial.terminal_test_preflare_target_speed,
        trial.terminal_test_preflare_min_speed);
    double available_energy=entry_remaining_specific_energy(
        future->latitude,future->mean_altitude,future->true_air_speed,
        cfg->site.latitude,gate_altitude,gate_speed,p);
    double spline_energy_aoa=clampd(fmax(3.0,future->angle_of_attack),3.0,
        terminal_hac_energy_aoa(v));
    TaemSpeedEnvelope gate_speed_envelope=
        decision_taem_speed_envelope(v,p,gate_altitude);
    if(!isfinite(available_energy)||!gate_speed_envelope.valid)return false;

    double terminal_glide_slope=clampd(trial.terminal_test_glide_slope,5.0,35.0);
    double terminal_tangent=tan(terminal_glide_slope*DEG2RAD);
    double minimum_tangent=tan(minimum_slope*DEG2RAD);
    double maximum_tangent=tan(maximum_slope*DEG2RAD);
    if(!(terminal_tangent>0.0)||!(minimum_tangent>0.0)||!(maximum_tangent>0.0))
        return false;

    /* The preflare location belongs to the runway glide geometry, not to the
       amount of excess energy carried into MM305. */
    double gate_ground=gate_height/terminal_tangent;
    double final_to_gate=fmax(0.0,final_distance-gate_ground);
    double height_to_gate=fmax(0.0,future->mean_altitude-gate_altitude);

    /* Path-length bounds come only from the allowed vertical corridor. */
    double minimum_curve=fmax(chord,
        fmax(0.0,height_to_gate/maximum_tangent-final_to_gate));
    double maximum_curve=fmax(0.0,
        height_to_gate/minimum_tangent-final_to_gate);
    if(!isfinite(minimum_curve)||!isfinite(maximum_curve)||
       maximum_curve<minimum_curve)return false;

    /* Search a compact geometric control domain.  Excess energy is handled by
       the physically propagated MM305 S-turn phase; it may not enlarge this
       terminal spline. */
    double handle_span=fmin(maximum_curve,fmax(chord,minimum_curve));
    if(!(handle_span>0.0)||!isfinite(handle_span))return false;

    double max_lateral=authority.maximum_lateral_accel_mps2;
    if(!(max_lateral>0.0)||!isfinite(max_lateral))return false;

    double speed_loss=terminal_expected_speed_loss_accel(g,live,p,aero,v);
    double final_course=cfg->site.runway_heading*DEG2RAD;
    double start_course=future_course*DEG2RAD;
    double live_course_deg=isfinite(live->ground_track_heading)?
        live->ground_track_heading:live->heading;
    if(!isfinite(live_course_deg))return false;
    double live_course=live_course_deg*DEG2RAD;
    double endpoint_speed_floor=fmax(trial.terminal_test_preflare_target_speed,
        authority.minimum_speed_mps);

    HACTransitionPlan best={0},fallback={0};
    bool have_best=false,have_fallback=false;
    double best_worst_margin=-INFINITY,fallback_worst_margin=-INFINITY;
    double best_slope=terminal_glide_slope,best_radius=p->radius,best_side=1.0;
    double fallback_slope=terminal_glide_slope,fallback_radius=p->radius,fallback_side=1.0;
    double best_energy_aoa=spline_energy_aoa,fallback_energy_aoa=spline_energy_aoa;
    bool fallback_geometry_bad=false,fallback_energy_bad=false;

    /*
     * This is numerical search resolution, not flight policy.  It scales with
     * floating-point precision so the set of candidates is not a tuned list of
     * vehicle-specific handle fractions.
     */
    const int search_samples=(int)fmax(4.0,floor(sqrt((double)DBL_MANT_DIG)*2.0));
    const int arc_samples=DBL_MANT_DIG/2;
    double minimum_handle=sqrt(DBL_EPSILON)*handle_span;
    double handle_range=fmax(0.0,handle_span-minimum_handle);

    for(int si=0;si<search_samples;si++)for(int ei=0;ei<search_samples;ei++){
        double su=((double)si+0.5)/(double)search_samples;
        double eu=((double)ei+0.5)/(double)search_samples;
        double start_handle=minimum_handle+handle_range*su;
        double end_handle=minimum_handle+handle_range*eu;

        HACTransitionPlan c={.valid=true};
        /* The spline is generated from a response-propagated future state.
           The measured-to-future segment is part of the commanded path, so it
           must be C1 at both ends: current shuttle velocity -> response state
           velocity -> runway spline.  A straight chord is only C0 and was the
           source of the visibly disconnected path in the west-side replay. */
        c.lead_start=hac_point(live_e,live_n);
        c.p0=hac_point(start_e,start_n);
        c.lead_start_course=norm_deg(live_course_deg);
        c.lead_end_course=norm_deg(future_course);
        if(response_distance>sqrt(DBL_EPSILON)*fmax(1.0,chord)){
            double lead_handle=response_distance/3.0;
            c.lead_curve=true;
            c.lead_p1=hac_point(live_e+sin(live_course)*lead_handle,
                live_n+cos(live_course)*lead_handle);
            c.lead_p2=hac_point(start_e-sin(start_course)*lead_handle,
                start_n-cos(start_course)*lead_handle);
            HACTransitionPlan response_lead=hac_lead_bezier_from_plan(&c);
            if(!hac_bezier_regular(&response_lead))continue;
            c.lead_length=hac_bezier_length_between(&response_lead,0.0,1.0,32);
            if(!isfinite(c.lead_length)||!(c.lead_length>0.0)||
               c.lead_length>response_distance*1.25)continue;
        }else{
            c.lead_curve=false;
            c.lead_p1=c.lead_start;
            c.lead_p2=c.p0;
            c.lead_length=0.0;
        }
        c.p1=hac_point(start_e+sin(start_course)*start_handle,
            start_n+cos(start_course)*start_handle);
        c.p3=hac_point(end_e,end_n);
        c.p2=hac_point(end_e-sin(final_course)*end_handle,
            end_n-cos(final_course)*end_handle);
        if(!hac_bezier_regular(&c))continue;

        c.length=hac_bezier_length_between(&c,0.0,1.0,arc_samples);
        c.arc_remaining=0.0;c.end_angle=final_course;
        if(!isfinite(c.length)||!(c.length>DBL_MIN))continue;

        double distance_along=0.0,max_curvature=0.0,end_rate=0.0;
        if(c.lead_curve){
            HACTransitionPlan response_lead=hac_lead_bezier_from_plan(&c);
            for(int k=0;k<=arc_samples;k++){
                double u=(double)k/(double)arc_samples;
                double curvature=hac_bezier_signed_curvature(&response_lead,u);
                if(!isfinite(curvature)){c.valid=false;break;}
                max_curvature=fmax(max_curvature,fabs(curvature));
                double speed=live->true_air_speed+
                    (future->true_air_speed-live->true_air_speed)*u;
                speed=fmax(1.0,speed);
                c.peak_lateral=fmax(c.peak_lateral,
                    fabs(speed*speed*curvature));
                double rate=fabs(speed*curvature)*RAD2DEG;
                double rate_cap=hac_course_rate_cap_for_speed(speed);
                if(!(rate_cap>0.0)||!isfinite(rate_cap)){c.valid=false;break;}
                c.peak_course_rate_ratio=fmax(c.peak_course_rate_ratio,
                    rate/rate_cap);
            }
            if(!c.valid)continue;
        }
        HACPoint2 previous=c.p0;
        for(int k=0;k<=arc_samples;k++){
            double u=(double)k/(double)arc_samples;
            HACPoint2 point=hac_bezier_point(&c,u);
            if(k>0)distance_along+=hypot(point.e-previous.e,point.n-previous.n);
            previous=point;
            double curvature=hac_bezier_signed_curvature(&c,u);
            max_curvature=fmax(max_curvature,fabs(curvature));
            double speed_scale=fmax(future->true_air_speed*future->true_air_speed,DBL_MIN);
            double decay=fmax(0.0,speed_loss)*distance_along/speed_scale;
            double speed=fmax(endpoint_speed_floor,future->true_air_speed*exp(-decay));
            if(k==arc_samples)c.exit_speed=speed;
            double lateral=speed*speed*curvature;
            c.peak_lateral=fmax(c.peak_lateral,fabs(lateral));
            double rate=fabs(speed*curvature)*RAD2DEG;
            double rate_cap=hac_course_rate_cap_for_speed(speed);
            if(!(rate_cap>0.0)||!isfinite(rate_cap)){c.valid=false;break;}
            c.peak_course_rate_ratio=fmax(c.peak_course_rate_ratio,rate/rate_cap);
            if(k==arc_samples)end_rate=rate;
        }
        if(!c.valid)continue;

        double effective_radius=max_curvature>DBL_MIN?1.0/max_curvature:p->radius;
        double path_to_gate=c.length+final_to_gate;
        double geometric_slope=atan2(height_to_gate,fmax(path_to_gate,DBL_MIN))*RAD2DEG;
        double end_rate_cap=hac_course_rate_cap_for_speed(c.exit_speed);
        PathFeasibilityEnvelope constraints=decision_path_feasibility_envelope(
            c.length,minimum_curve,maximum_curve,
            geometric_slope,minimum_slope,maximum_slope,
            c.peak_lateral,max_lateral,
            c.peak_course_rate_ratio,end_rate,end_rate_cap);
        if(!constraints.valid)continue;

        /* Certify the curve we will actually fly, not only the scalar path
           length that seeded the handle search.  A Bezier can be materially
           longer than handle_span; the old code marked such a curve energy
           nominal if its slope/curvature were regular, even when its exact
           drag work exceeded the unpowered budget. */
        double candidate_total_path=c.length+final_to_gate;
        double spline_bank_guard=fmin(v->maximum_bank_angle,
            dynamic_bank_limit(future,v));
        double lateral_aoa=terminal_required_aoa_for_lateral(
            future,aero,v,c.peak_lateral,spline_bank_guard);
        double candidate_energy_aoa=fmax(spline_energy_aoa,lateral_aoa);
        double projected_gate_speed=NAN,projected_gate_altitude=NAN;
        double candidate_work=terminal_projected_drag_work_state(&trial,future,p,aero,
            v,s,candidate_energy_aoa,candidate_total_path,geometric_slope,
            &projected_gate_speed,&projected_gate_altitude);
        if(!isfinite(candidate_work)||candidate_work<0.0||
           !isfinite(projected_gate_speed)||!isfinite(projected_gate_altitude))
            continue;
        EnergyPathEnvelope candidate_energy=decision_energy_path_envelope(
            future,p,available_energy,candidate_work,candidate_total_path);
        if(!candidate_energy.valid)continue;

        double gate_min_speed=fmax(trial.terminal_test_preflare_min_speed,
            gate_speed_envelope.minimum_speed_mps);
        double gate_max_speed=gate_speed_envelope.maximum_speed_mps;
        if(!(gate_max_speed>=gate_min_speed))continue;
        double low_speed_margin=(projected_gate_speed-gate_min_speed)/
            fmax(gate_min_speed,DBL_MIN);
        double high_speed_margin=(gate_max_speed-projected_gate_speed)/
            fmax(gate_max_speed,DBL_MIN);
        double gate_speed_margin=fmin(low_speed_margin,high_speed_margin);
        bool energy_feasible=candidate_energy.feasible&&gate_speed_margin>=0.0;

        c.degraded_path=
            constraints.path_minimum.margin<0.0||
            constraints.path_maximum.margin<0.0||
            constraints.slope_minimum.margin<0.0||
            constraints.slope_maximum.margin<0.0;
        c.degraded_control=constraints.lateral_authority.margin<0.0;
        c.degraded_rate=constraints.course_rate.margin<0.0;
        c.degraded_end=constraints.endpoint_rate.margin<0.0;
        c.degraded=c.degraded_path||c.degraded_control||
            c.degraded_rate||c.degraded_end||!energy_feasible;
        double combined_worst_margin=fmin(constraints.worst_normalized_margin,
            fmin(candidate_energy.energy.normalized_margin,gate_speed_margin));
        c.violation_score=fmax(0.0,-combined_worst_margin);

        double turn_delta=norm_signed_deg(cfg->site.runway_heading-future_course);
        double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        double spatial_resolution=sqrt(DBL_EPSILON)*fmax(chord,1.0);
        double side;
        if(fabs(turn_delta)>angular_resolution)side=turn_delta>=0.0?-1.0:1.0;
        else if(fabs(future->runway_cross_track)>spatial_resolution)
            side=future->runway_cross_track>=0.0?1.0:-1.0;
        else side=g->s_turn_sign>=0.0?1.0:-1.0;

        if(constraints.feasible&&energy_feasible){
            double scale=fmax(1.0,fmax(fabs(best_worst_margin),
                fabs(combined_worst_margin)));
            bool better_margin=!have_best||
                combined_worst_margin>best_worst_margin+
                    sqrt(DBL_EPSILON)*scale;
            bool same_margin=have_best&&
                fabs(combined_worst_margin-best_worst_margin)<=
                    sqrt(DBL_EPSILON)*scale;
            if(better_margin||(same_margin&&c.length<best.length)){
                have_best=true;best=c;best_worst_margin=combined_worst_margin;
                best_slope=geometric_slope;best_radius=effective_radius;best_side=side;
                best_energy_aoa=candidate_energy_aoa;
            }
        }else{
            double scale=fmax(1.0,fmax(fabs(fallback_worst_margin),
                fabs(combined_worst_margin)));
            bool better_margin=!have_fallback||
                combined_worst_margin>fallback_worst_margin+
                    sqrt(DBL_EPSILON)*scale;
            bool same_margin=have_fallback&&
                fabs(combined_worst_margin-fallback_worst_margin)<=
                    sqrt(DBL_EPSILON)*scale;
            if(better_margin||(same_margin&&c.length<fallback.length)){
                have_fallback=true;fallback=c;
                fallback_worst_margin=combined_worst_margin;
                fallback_slope=geometric_slope;fallback_radius=effective_radius;
                fallback_side=side;
                fallback_energy_aoa=candidate_energy_aoa;
                fallback_geometry_bad=!constraints.feasible;
                fallback_energy_bad=!energy_feasible;
            }
        }
    }

    bool nominal=have_best;
    if(!nominal&&!have_fallback)return false;
    HACTransitionPlan chosen=nominal?best:fallback;
    TerminalCandidate next={.valid=true,.degraded=chosen.degraded,
        .geometry_degraded=nominal?false:fallback_geometry_bad,
        .energy_degraded=nominal?false:fallback_energy_bad,
        .shell_degraded=false,.kind=TERMINAL_PATH_SPLINE,.join=chosen,
        .radius=nominal?best_radius:fallback_radius,
        .side=nominal?best_side:fallback_side,
        .slope=nominal?best_slope:fallback_slope,.final_distance=final_distance,
        .response=authority.control_response_time_s,
        /* Carry the AoA used by the exact curve energy certificate into the
           live candidate check; re-evaluating at a fixed HAC incidence would
           compare two different drag models. */
        .energy_aoa=nominal?best_energy_aoa:fallback_energy_aoa,
        .altitude=future->mean_altitude,.speed=future->true_air_speed,
        .course=future_course,.selected_ut=live->ut,.arrival_ut=future->ut,
        .quality_score=chosen.violation_score};
    *out=next;
    return true;
}


                                                                                
                                                                                  

/* Ranking must use the same live forecast, vertical and energy contract that
   will authorize ownership.  Probe a copy so search never publishes a path or
   overwrites the retained candidate's diagnostic state. */
static bool terminal_candidate_probe_commit_ready(const GuidanceMachine*g,
        const TerminalCandidate*c,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!c||!c->valid||!g->terminal_region_entered)return false;
    GuidanceMachine trial=*g;
    trial.terminal_candidate=*c;
    trial.diagnostic_shadow=true;
    return terminal_candidate_commit_ready(&trial,t,p,aero,cfg);
}

static bool terminal_candidate_from_future(const GuidanceMachine*g,const Telemetry*live,
        const Telemetry*future,double future_course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,TerminalCandidate*out){
    ControlAuthorityEnvelope future_authority=
        decision_control_authority_envelope(g,future,p,cfg);
    if(!future_authority.valid||!future_authority.controllable||
       future_authority.speed.margin<0.0)return false;
    TerminalCandidate best={0};

    /* Evaluate the free-form runway-line path.  It is not an MM304 ownership
       substitute, but after the strict fixed-point handoff has latched it is a
       valid MM305 terminal option and often the shortest way to avoid a large
       outbound HAC excursion. */
    TerminalCandidate spline={0};
    bool spline_ok=terminal_spline_candidate_from_future(g,live,future,future_course,p,aero,cfg,&spline);
    bool mm305_owns=taem_exec_owns_vehicle(&g->taem_exec);
    if(spline_ok&&(g->taem_interface_captured||mm305_owns)&&
       !spline.geometry_degraded){
        /* A direct spline is a terminal topology, not an MM304 shortcut.
           Once the TAEM executive has latched ownership it is already past the
           strict handoff boundary, even in rehearsal/restart paths where the
           legacy taem_interface_captured mirror bit is not populated. */
        terminal_candidate_execution_cost(&spline,g,live,
            isfinite(live->ground_track_heading)?live->ground_track_heading:live->heading,
            p,aero,cfg);
        spline.quality_score=terminal_candidate_delivery_time(&spline);
        if(terminal_candidate_better(&best,&spline))best=spline;
    }
    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
        fprintf(stderr,"terminal topology: UT %.1f future alt %.0f V %.0f range %.0f spline=%s geom=%d energy=%d degraded=%d q %.2f len %.0f peakLat %.3f rate %.3f slope %.2f violation %.3f exec %.1fs/%.0fm/%.1fdeg chosen=%s\n",
            live->ut,future->mean_altitude,future->true_air_speed,future->range_to_site,
            spline_ok?"yes":"no",spline.geometry_degraded,spline.energy_degraded,
            spline.degraded,spline.quality_score,spline.join.length,
            spline.join.peak_lateral,spline.join.peak_course_rate_ratio,
            spline.slope,spline.join.violation_score,spline.execution_time,
            spline.execution_distance,spline.execution_course_error,
            best.kind==TERMINAL_PATH_SPLINE?"spline":"none");
    }

    GuidanceMachine trial=*g;
    double energy_ema=g->terminal_energy_loss_accel_ema;
    double speed_ema=g->terminal_speed_loss_accel_ema;
    terminal_glide_initialize(&trial,&cfg->vehicle,&cfg->guidance);
    trial.terminal_energy_loss_accel_ema=energy_ema;
    trial.terminal_speed_loss_accel_ema=speed_ema;
    if(terminal_test_start_spiral(&trial,future,future_course,p,aero,cfg)){
        TerminalCandidate hac={.valid=true,.degraded=trial.hac_plan_degraded,
            .geometry_degraded=trial.hac_plan_geometry_degraded,
            .energy_degraded=trial.hac_plan_energy_degraded,.shell_degraded=false,
            .kind=TERMINAL_PATH_HAC,.radius=trial.hac_radius,.side=trial.hac_side,
            .slope=trial.terminal_test_glide_slope,
            .final_distance=trial.terminal_test_final_approach_distance,
            .response=trial.hac_transition_response_time,.altitude=future->mean_altitude,
            .energy_aoa=clampd(fmax(3.0,future->angle_of_attack),3.0,
                terminal_hac_energy_aoa(&cfg->vehicle)),
            .speed=future->true_air_speed,.course=future_course,.selected_ut=live->ut,
            .arrival_ut=future->ut};
        hac.join=(HACTransitionPlan){.valid=true,.degraded=hac.degraded,
            .violation_score=trial.hac_plan_violation_score,
            .heading_cone=trial.hac_transition_heading_cone,
            .lead_start={trial.hac_transition_lead_start_e,trial.hac_transition_lead_start_n},
            .p0={trial.hac_transition_p0_e,trial.hac_transition_p0_n},
            .p1={trial.hac_transition_p1_e,trial.hac_transition_p1_n},
            .p2={trial.hac_transition_p2_e,trial.hac_transition_p2_n},
            .p3={trial.hac_transition_p3_e,trial.hac_transition_p3_n},
            .cone_center={trial.hac_transition_cone_center_e,trial.hac_transition_cone_center_n},
            .cone_start_angle=trial.hac_transition_cone_start_angle,
            .cone_end_angle=trial.hac_transition_cone_end_angle,
            .cone_arc_length=trial.hac_transition_cone_arc_length,
            .lead_length=trial.hac_transition_lead_length,.length=trial.hac_transition_length,
            .end_angle=trial.hac_transition_end_angle,.arc_remaining=trial.hac_remaining,
            .exit_speed=trial.hac_transition_exit_speed};
        terminal_candidate_execution_cost(&hac,g,live,
            isfinite(live->ground_track_heading)?live->ground_track_heading:live->heading,
            p,aero,cfg);
        hac.quality_score=terminal_candidate_delivery_time(&hac);
        bool hac_ready=terminal_candidate_probe_commit_ready(g,&hac,live,p,aero,cfg);
        bool best_ready=terminal_candidate_probe_commit_ready(g,&best,live,p,aero,cfg);
        bool hac_geometry_ok=terminal_candidate_geometry_regular(&hac);
        bool best_geometry_ok=terminal_candidate_geometry_regular(&best);
        /* Commit readiness is meaningful only inside the same geometry/control
           class.  A ready-but-unflyable HAC must never displace a regular spline;
           that late override was undoing the geometry-first candidate comparator. */
        if(hac_geometry_ok&&!best_geometry_ok)
            best=hac;
        else if(hac_geometry_ok==best_geometry_ok&&
                ((hac_ready&&!best_ready)||
                 (hac_ready==best_ready&&terminal_candidate_better(&best,&hac))))
            best=hac;
        if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
            fprintf(stderr,"terminal topology HAC: UT %.1f geom=%d energy=%d degraded=%d q %.2f exec %.1fs/%.0fm/%.1fdeg chosen=%s\n",
                live->ut,hac.geometry_degraded,hac.energy_degraded,hac.degraded,hac.quality_score,
                hac.execution_time,hac.execution_distance,hac.execution_course_error,
                best.kind==TERMINAL_PATH_HAC?"hac":"none");
        }
    }
    if(!best.valid)return false;
    *out=best;
    return true;
}

/* A candidate is built from a response-propagated future state.  That state is
   only useful when the live vehicle can reach its forecast origin inside the
   same horizon used to construct it.  The commit gate performs this check
   again, but admitting an unreachable candidate into the retained preview
   lets refinement hysteresis preserve a path whose arrival timestamp is
   already impossible.  Use the shared bounded-curvature path and the live
   horizontal speed here so preview selection and commitment share one
   reachability contract. */
static bool terminal_future_origin_reachable(const GuidanceMachine*g,
        const Telemetry*live,double course,const Telemetry*future,
        double future_course,double horizon,const PlanetModel*p,
        const LandingConfiguration*cfg){
    if(!g||!live||!future||!p||!cfg||!isfinite(course)||
       !isfinite(future_course)||!isfinite(horizon)||horizon<0.0||
       !(live->horizontal_speed>0.0)||
       !isfinite(future->runway_along_track)||
       !isfinite(future->runway_cross_track))return false;
    double minimum_turn_radius=0.0;
    double path=decision_target_path_length(g,live,course,
        future->runway_along_track,future->runway_cross_track,
        future_course,NAN,p,cfg,&minimum_turn_radius);
    if(!isfinite(path)||!isfinite(minimum_turn_radius)||
       !(minimum_turn_radius>0.0))return false;
    double required_time=path/live->horizontal_speed;
    double tolerance=sqrt(DBL_EPSILON)*fmax(1.0,horizon);
    return isfinite(required_time)&&required_time<=horizon+tolerance;
}

                                                                                   
                                                                           

                                                            
                                                                
                                                  
                                                                  

       bool terminal_capture_margin_exhausted(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    (void)aero;
    /* A perpendicular MM305 candidate owns a longer path than the direct
       runway reserve below.  Do not let that reserve abort a candidate that
       still has a valid geometry and candidate-specific vertical response
       margin; the direct runway bound is only the fallback when no such path
       exists. */
    if(g&&g->terminal_candidate.valid&&
       terminal_candidate_operationally_usable(g,&g->terminal_candidate,t,course,p,cfg)&&
       !g->terminal_candidate.geometry_degraded)
        return false;
    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return false;
    const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    if(diag&&strcmp(diag,"1")==0){
        fprintf(stderr,
            "terminal envelope: UT %.1f runwayTime %+.2fs vertical %+.1fm speed %+.1fm/s q %+.0fPa load %+.2fg runway %+.1fm requiredLat %.2fs requiredHead %.2fs available %.2fs\n",
            t->ut,envelope.runway_time.margin,envelope.vertical_recovery.margin,
            envelope.speed.margin,envelope.dynamic_pressure.margin,envelope.load.margin,
            envelope.runway_remaining.margin,envelope.lateral_capture_time_s,
            envelope.heading_capture_time_s,envelope.time_available_s);
    }
    /*
     * Acquisition is exhausted only when a hard physical reserve is gone.
     * Lateral/time deficit alone does not abort here because MM305 may still
     * add path.  Height, minimum-speed, or runway-remaining deficits cannot be
     * repaired by inventing another HAC.
     */
    return envelope.vertical_recovery.margin<0.0||
        envelope.speed.margin<0.0||
        envelope.runway_remaining.margin<0.0;
}


/* Minimum specific drag work to fly a ground path in equilibrium glide.
   Lift must balance weight (times the turn load factor n), so drag is at
   least n*g/(L/D) at the best available incidence; any additional altitude
   is excess energy the profile dissipates.  Pricing a fixed incidence at the
   live dynamic pressure instead charged drag for lift far above weight, the
   aircraft would never fly (work/available ~1.3 from states with ample glide
   range), so no candidate ever passed the live energy gate. */
static double terminal_min_glide_work(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double straight_path,
        double arc_path,double arc_radius,double speed){
    if(!t||!p||!v||!isfinite(aero.lift_to_drag)||!(aero.lift_to_drag>0.0)||
       !isfinite(straight_path)||!isfinite(arc_path)||straight_path<0.0||
       arc_path<0.0)return NAN;
    double mach=isfinite(t->mach)?t->mach:0.5;
    double best_ratio=-INFINITY;
    for(double aoa=2.0;aoa<=v->maximum_angle_of_attack-1.0+1e-9;aoa+=.25){
        double lf=0.0,df=1.0;
        aerodynamic_force_factors_mach(mach,aoa,v,&lf,&df);
        double ratio=lf/fmax(df,.02);
        if(isfinite(ratio))best_ratio=fmax(best_ratio,ratio);
    }
    if(!(best_ratio>0.0))return NAN;
    double ld=aero.lift_to_drag*best_ratio;
    double gravity=planet_surface_gravity(p);
    double n_arc=1.0;
    if(arc_path>0.0&&isfinite(arc_radius)&&arc_radius>0.0&&isfinite(speed)){
        double lateral=speed*speed/arc_radius/gravity;
        n_arc=sqrt(1.0+lateral*lateral);
    }
    return gravity/ld*(straight_path+n_arc*arc_path);
}

double terminal_energy_commit_tolerance=NAN;
static bool terminal_candidate_live_energy_ready(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TerminalCandidate*c){
    if(g){
        g->terminal_candidate_live_energy_margin=NAN;
        g->terminal_candidate_live_energy_valid=false;
    }
    if(!g->terminal_glide_mode)return true;
    if(!c||!c->valid)return false;
    double gate_height=fmax(0.0,g->terminal_test_preflare_altitude);
    double gate_altitude=cfg->site.altitude+gate_height;
    double gate_speed=fmax(g->terminal_test_preflare_target_speed,
        g->terminal_test_preflare_min_speed);
    double candidate_slope=c->slope;
    double tangent=tan(candidate_slope*DEG2RAD);
    if(!(candidate_slope>0.0)||!(candidate_slope<90.0)||!(tangent>0.0))
        return false;

    double gate_ground=gate_height/tangent;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint current_point={t->latitude,t->longitude,t->mean_altitude};
    double current_e=0.0,current_n=0.0;
    local_offsets(origin,current_point,p->radius,&current_e,&current_n);
    /* A future-origin HAC carries a response lead beginning at its forecast
       state, not at the live vehicle. Charge the upstream forecast leg here so
       a positive candidate margin cannot hide the distance still required to
       reach that origin. Live-origin spline candidates contribute zero. */
    double upstream=fmax(0.0,hypot(c->join.lead_start.e-current_e,
        c->join.lead_start.n-current_n));
    double fixed_path=upstream+fmax(0.0,c->join.lead_length)+fmax(0.0,c->join.length)+
        fmax(0.0,c->join.arc_remaining);
    double path_to_gate=fixed_path+fmax(0.0,c->final_distance-gate_ground);
    if(!(path_to_gate>0.0)||!isfinite(path_to_gate))return false;

    double available=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,cfg->site.latitude,gate_altitude,gate_speed,p);
    double target_aoa=isfinite(c->energy_aoa)&&c->energy_aoa>0.0?
        clampd(c->energy_aoa,0.0,cfg->vehicle.maximum_angle_of_attack):
        terminal_hac_energy_aoa(&cfg->vehicle);
    double transition_aoa=NAN;
    double transition_path=0.0;
    if(t->flight_path_angle< -c->slope){
        /* The candidate origin is a future state.  Its downstream energy
           model starts at the selected path slope, but the live vehicle may
           still be descending more steeply.  Flattening that mismatch needs
           normal force and therefore incidence/drag before the origin can be
           reached.  Charge the physically required transition AoA here so a
           positive downstream margin cannot certify a path whose first leg
           spends the reserve just pulling the velocity vector onto the path.
         */
        transition_aoa=terminal_fpa_force_aoa(t,p,aero,&cfg->vehicle,
            -c->slope);
        /* The response lead is the segment over which the velocity vector is
           brought onto the selected path.  Charge the extra incidence drag
           only over that upstream/response segment; applying the transition
           AoA to the entire downstream route made a short FPA correction look
           like a full-HAC energy demand and rejected otherwise executable
           paths. */
        transition_path=fmin(path_to_gate,upstream+fmax(0.0,c->join.lead_length));
    }
    double integrated=terminal_projected_drag_work(g,t,p,aero,&cfg->vehicle,&cfg->guidance,
        target_aoa,path_to_gate,candidate_slope);
    if(isfinite(transition_aoa)&&transition_path>0.0&&
       transition_aoa>target_aoa){
        double transition_work=terminal_projected_drag_work(g,t,p,aero,
            &cfg->vehicle,&cfg->guidance,transition_aoa,transition_path,
            candidate_slope);
        double baseline_work=terminal_projected_drag_work(g,t,p,aero,
            &cfg->vehicle,&cfg->guidance,target_aoa,transition_path,
            candidate_slope);
        if(isfinite(transition_work)&&isfinite(baseline_work))
            integrated+=fmax(0.0,transition_work-baseline_work);
        else
            integrated=INFINITY;
    }
    if(!isfinite(available)||available<0.0)return false;

    double modeled_work=integrated;
    if(!isfinite(modeled_work)||modeled_work<0.0)
        modeled_work=terminal_expected_energy_loss_accel(g,t,aero,&cfg->vehicle)*
            path_to_gate;
    double arc_path=c->join.heading_cone?fmax(0.0,c->join.cone_arc_length):0.0;
    double glide_work=terminal_min_glide_work(t,p,aero,&cfg->vehicle,
        fmax(0.0,path_to_gate-arc_path),arc_path,c->radius,t->true_air_speed);
    if(isfinite(glide_work)&&glide_work>=0.0&&
       (!isfinite(modeled_work)||glide_work<modeled_work))
        modeled_work=glide_work;

    EnergyPathEnvelope energy=decision_energy_path_envelope(
        t,p,available,modeled_work,path_to_gate);
    terminal_energy_commit_tolerance=energy.valid?
        energy.uncertainty_specific_energy+0.05*fmax(0.0,modeled_work):NAN;
    if(g&&energy.valid){
        g->terminal_candidate_live_energy_margin=energy.energy.margin;
        g->terminal_candidate_live_energy_valid=true;
    }
    if(!g->diagnostic_shadow&&getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
        static double last_energy_diag_ut=-INFINITY;
        if(t->ut-last_energy_diag_ut>=1.0){
            fprintf(stderr,
                "terminal energy envelope: UT %.1f path %.0f upstream %.0f transitionAoA %.1f aoa %.1f avail %.0f work %.0f uncertainty %.0f margin %+.0f\n",
                t->ut,path_to_gate,upstream,transition_aoa,target_aoa,
                available,modeled_work,
                energy.uncertainty_specific_energy,
                energy.valid?energy.energy.margin:NAN);
            last_energy_diag_ut=t->ut;
        }
    }
    return energy.valid&&energy.feasible;
}

/* Build a terminal HAC whose response lead starts at the live sample.  The
   normal selector evaluates response-projected future origins because that is
   useful for preview timing.  Once a future candidate is charged back to the
   current state, however, its upstream lead can consume the remaining energy
   budget.  This helper gives the live-origin topology a chance to prove the
   same geometry/energy contract without silently shortening a forecast path. */
static bool terminal_live_origin_hac_candidate(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,TerminalCandidate*out){
    if(!g||!t||!p||!cfg||!out)return false;
    GuidanceMachine trial=*g;
    double energy_ema=g->terminal_energy_loss_accel_ema;
    double speed_ema=g->terminal_speed_loss_accel_ema;
    terminal_glide_initialize(&trial,&cfg->vehicle,&cfg->guidance);
    trial.terminal_energy_loss_accel_ema=energy_ema;
    trial.terminal_speed_loss_accel_ema=speed_ema;
    if(!terminal_test_start_spiral(&trial,t,course,p,aero,cfg))return false;

    TerminalCandidate candidate={.valid=true,
        .degraded=trial.hac_plan_degraded,
        .geometry_degraded=trial.hac_plan_geometry_degraded,
        .energy_degraded=trial.hac_plan_energy_degraded,
        .shell_degraded=false,.kind=TERMINAL_PATH_HAC,
        .radius=trial.hac_radius,.side=trial.hac_side,
        .slope=trial.terminal_test_glide_slope,
        .final_distance=trial.terminal_test_final_approach_distance,
        .response=trial.hac_transition_response_time,
        .altitude=t->mean_altitude,
        .energy_aoa=clampd(fmax(3.0,t->angle_of_attack),3.0,
            terminal_hac_energy_aoa(&cfg->vehicle)),
        .speed=t->true_air_speed,.course=course,
        .selected_ut=t->ut,.arrival_ut=t->ut};
    candidate.join=(HACTransitionPlan){.valid=true,
        .degraded=candidate.degraded,
        .violation_score=trial.hac_plan_violation_score,
        .heading_cone=trial.hac_transition_heading_cone,
        .lead_start={trial.hac_transition_lead_start_e,
            trial.hac_transition_lead_start_n},
        .p0={trial.hac_transition_p0_e,trial.hac_transition_p0_n},
        .p1={trial.hac_transition_p1_e,trial.hac_transition_p1_n},
        .p2={trial.hac_transition_p2_e,trial.hac_transition_p2_n},
        .p3={trial.hac_transition_p3_e,trial.hac_transition_p3_n},
        .cone_center={trial.hac_transition_cone_center_e,trial.hac_transition_cone_center_n},
        .cone_start_angle=trial.hac_transition_cone_start_angle,
        .cone_end_angle=trial.hac_transition_cone_end_angle,
        .cone_arc_length=trial.hac_transition_cone_arc_length,
        .lead_length=trial.hac_transition_lead_length,
        .length=trial.hac_transition_length,
        .end_angle=trial.hac_transition_end_angle,
        .arc_remaining=trial.hac_remaining,
        .exit_speed=trial.hac_transition_exit_speed};
    terminal_candidate_execution_cost(&candidate,g,t,course,p,aero,cfg);
    candidate.quality_score=terminal_candidate_delivery_time(&candidate);
    *out=candidate;
    return true;
}

static bool terminal_candidate_vertical_response_ready_from_arrival(
        const GuidanceMachine*g,const Telemetry*arrival,double arrival_course,
        const PlanetModel*p,const TerminalCandidate*c,
        const LandingConfiguration*cfg,double*drop_out,double*height_out){
    if(!g||!arrival||!p||!c||!c->valid||!cfg)return false;
    (void)arrival_course;

    double gate_height=g->terminal_glide_mode?
        fmax(0.0,g->terminal_test_preflare_altitude):
        fmax(0.0,cfg->guidance.flare_altitude);
    double gate_altitude=cfg->site.altitude+gate_height;

    double slope=fabs(c->slope);
    if(!(slope>0.0)||!(slope<90.0))return false;
    double tangent=tan(slope*DEG2RAD);
    if(!(tangent>0.0))return false;

    double gate_ground=gate_height/tangent;
    double path_to_gate=fmax(0.0,c->join.lead_length)+
        fmax(0.0,c->join.length)+
        fmax(0.0,c->join.arc_remaining)+
        fmax(0.0,c->final_distance-gate_ground);
    double height=fmax(0.0,arrival->mean_altitude-gate_altitude);

    /* The candidate owns the vertical leg after its response-projected
       origin.  Do not ask the point-to-runway envelope to certify that leg:
       the vehicle is intentionally still perpendicular to the strip and that
       unrelated lateral demand can make its runway recovery margin invalid.
       Use the candidate slope for geometric height closure and the shared
       local recovery envelope only for the response needed to stay on that
       slope. */
    ControlAuthorityEnvelope authority=decision_control_authority_envelope(
        g,arrival,p,cfg);
    if(!authority.valid||!authority.controllable||
       authority.speed.margin<0.0||authority.dynamic_pressure.margin<0.0||
       authority.load.margin<0.0||authority.stall.margin<0.0)return false;

    double gravity=fmax(.1,planet_surface_gravity(p));
    double vertical_projection=cos(arrival->flight_path_angle*DEG2RAD);
    double recoverable_accel=fmax(0.0,
        authority.maximum_normal_accel_mps2*vertical_projection-gravity);
    double target_sink=arrival->horizontal_speed*tangent;
    VerticalRecoveryEnvelope response=decision_vertical_recovery_envelope(
        height,fmax(0.0,-arrival->vertical_speed),target_sink,
        authority.control_response_time_s,gravity,recoverable_accel);
    /* The actuator cannot establish a steep descent at the lead origin.
       Credit the first response interval with at most the drop achievable
       from the measured sink under gravity, then use the planned slope for
       the rest of the route.  Otherwise a short path can pass this gate by
       counting descent that the aircraft has no time to fly. */
    double response_time=fmin(fmax(0.0,authority.control_response_time_s),
        path_to_gate/fmax(1.0,arrival->horizontal_speed));
    double response_distance=fmin(path_to_gate,
        fmax(0.0,arrival->horizontal_speed)*response_time);
    double response_drop=fmax(0.0,-arrival->vertical_speed)*response_time+
        .5*gravity*response_time*response_time;
    double drop=(path_to_gate-response_distance)*tangent+
        fmin(response_distance*tangent,response_drop);

    if(drop_out)*drop_out=drop;
    if(height_out)*height_out=height;
    return response.valid&&response.reachable&&isfinite(drop)&&drop>=height;
}

       bool terminal_candidate_vertical_response_ready_live(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const TerminalCandidate*c){
    if(!g||!t||!p||!cfg||!c||!c->valid)return false;
    double horizon=fmax(0.0,c->arrival_ut-t->ut);
    Telemetry arrival=*t;double e=0.0,n=0.0,arrival_course=course;
    terminal_project_planning_state(g,t,course,p,aero,cfg,horizon,
        &arrival,&e,&n,&arrival_course);
    return terminal_candidate_vertical_response_ready_from_arrival(
        g,&arrival,arrival_course,p,c,cfg,NULL,NULL);
}

static bool terminal_interface_target_spatially_relevant(const TaemInterfaceTarget*q,
        const Telemetry*t,const LandingConfiguration*cfg){
    if(!q||!t||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->acquisition_lead)||q->acquisition_lead<=0.0||
       !isfinite(q->response_time)||q->response_time<=0.0||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(t->horizontal_speed))return false;
    double h=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double da=q->along_track-t->runway_along_track;
    double dc=q->cross_track-t->runway_cross_track;
    double along=da*cos(h)+dc*sin(h);
    double response=fmax(5.0,q->response_time);
    double lead=fmax(q->acquisition_lead,fmax(80.0,t->horizontal_speed)*response);
    /* Match the downstream grace used by entry_taem_interface_capture(). A clean
       inlet remains a useful vertical/acquisition contract briefly after crossing
       its station, but not once the orbiter has flown materially beyond it. */
    return isfinite(along)&&along>=-lead*.35;
}

static void terminal_publish_interface_target(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->terminal_candidate.valid)return;
    /* MM304's tangent point is a one-way ownership boundary.  Once the strict
       handoff has latched, the terminal selector may replace its private HAC or
       spline candidate, but it must not publish that candidate's upstream
       acquisition lead as a new MM304 target.  Doing so lets a post-handoff
       preview move the fixed interface behind the live vehicle and makes the
       next capture evaluation demand an impossible backwards turn. */
    if(g->taem_interface_captured)return;
    const TerminalCandidate*c=&g->terminal_candidate;
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    /* A selector geometry-degraded path is preview-only: TAEM cannot freeze it
       because its lateral/rate/end geometry exceeds demonstrated authority. v18
       showed that publishing its inverse merge geometry to MM304 is equally bad:
       an expired clean inlet near -74 km was replaced by a degraded HAC whose
       acquisition demand jumped to roughly -172 km and ~30 deg of course error.
       Preserve the last clean inlet while it remains inside the normal downstream
       capture grace. Once that demand is truly passed, clear it and let Entry/staging
       guidance wait for a clean terminal candidate rather than chase impossible
       inverse geometry. */
    if(c->geometry_degraded){
        if(!terminal_interface_target_spatially_relevant(&g->taem_interface_target,t,cfg))
            g->taem_interface_target.valid=false;
        return;
    }
    if(!c->join.valid||!hac_bezier_regular(&c->join))return;
    /* Demand comes from the terminal merge geometry, never from the predicted
       future state of the current entry trajectory. Its lead must pay both
       acquisition response and kinetic-energy conversion before that merge. */
    HACPoint2 merge=c->join.p3;
    double te=merge.e-c->join.p2.e,tn=merge.n-c->join.p2.n,tm=hypot(te,tn);
    if(tm<1.0){te=merge.e-c->join.p0.e;tn=merge.n-c->join.p0.n;tm=hypot(te,tn);}
    if(tm<1.0||!isfinite(tm))return;
    te/=tm;tn/=tm;
    double tangent_course=norm_deg(atan2(te,tn)*RAD2DEG);
    double rh=cfg->site.runway_heading*DEG2RAD;
    double current_e=t->runway_along_track*sin(rh)+t->runway_cross_track*cos(rh);
    double current_n=t->runway_along_track*cos(rh)-t->runway_cross_track*sin(rh);
    double merge_along=(merge.e-current_e)*te+(merge.n-current_n)*tn;
    double merge_cross=(merge.e-current_e)*tn-(merge.n-current_n)*te;
    double live_course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
    double course_error=fabs(norm_signed_deg(tangent_course-live_course));
    /* A regular cubic does not prove its acquisition tangent is reachable.
       Retain the previous demand rather than flip entry toward a backwards
       endpoint after an unrelated future-state candidate timer expires. */
    if(!g->entry_exec.entry_complete&&(merge_along<=0.0||course_error>75.0))return;

    double gravity=fmax(.1,planet_surface_gravity(p));
    double ceiling=s->taem_force_handoff_speed;
    double base_speed=clampd(entry_taem_speed_target(v,s,p)+55.0,
        v->minimum_safe_speed*1.6,ceiling);
    double response=fmax(7.0,c->response);
    double base_lead=fmax(s->hac_radius*1.25,base_speed*response*1.05);
    double gate_height=clampd(fmax(500.0,s->flare_altitude*8.0),450.0,750.0);
    double final_slope=clampd(s->final_glide_slope,5.0,35.0)*DEG2RAD;
    double gate_ground=gate_height/fmax(tan(final_slope),1e-3);
    double arc=fmax(0.0,c->join.arc_remaining);
    double final_part=fmax(0.0,c->final_distance-gate_ground);
    double downstream=arc+final_part;
    double merge_altitude=cfg->site.altitude+gate_height+
        arc*tan(clampd(s->taem_glide_slope,6.0,22.0)*DEG2RAD)+final_part*tan(final_slope);
    double current_lf=1,reference_lf=1,current_df=1,reference_df=1;
    aerodynamic_force_factors_mach(t->mach,fmax(1.0,fabs(t->angle_of_attack)),v,&current_lf,&current_df);
    aerodynamic_force_factors_mach(t->mach,v->entry_angle_of_attack,v,&reference_lf,&reference_df);
    double lift_per_q=live_lift_accel(t,aero,v)/fmax(1.0,t->dynamic_pressure)*
        clampd(fabs(reference_lf)/fmax(.05,fabs(current_lf)),.5,2.0);
    double drag_per_q=terminal_expected_energy_loss_accel(g,t,aero,v)/fmax(1.0,t->dynamic_pressure)*
        clampd(reference_df/fmax(.05,current_df),.5,2.0);
    if(!isfinite(lift_per_q)||lift_per_q<=1e-8||!isfinite(drag_per_q)||drag_per_q<=0)return;
    /* Highest inlet altitude that still supplies meaningful turn authority
       at the eligibility ceiling. This replaces the 0.85..1.35 fixed-shell
       clamp with a measured lift/density limit. It is not a handoff altitude. */
    double required_q=gravity*.65/lift_per_q;
    if(required_q>v->maximum_dynamic_pressure*.85)return;
    double lo=cfg->site.altitude+gate_height,hi=p->atmosphere_depth;
    for(int i=0;i<18;i++){
        double mid=.5*(lo+hi);
        double q=.5*planet_atmospheric_density(p,mid)*ceiling*ceiling;
        if(q>=required_q)lo=mid;else hi=mid;
    }
    double control_altitude=.5*(lo+hi);
    /* Neither the candidate's propagated altitude, speed nor its fitted
       descent slope owns the inlet. Solve the vertical leg backwards from
       the geometric merge height and measured high-speed control boundary. */
    double height_budget=control_altitude-merge_altitude;
    if(!isfinite(height_budget)||height_budget<=500.0)return;
    double minimum_slope=clampd(s->taem_glide_slope-4.0,6.0,18.0);
    double maximum_slope=clampd(fmax(s->final_glide_slope+12.0,s->taem_glide_slope+10.0),
        minimum_slope+4.0,35.0);
    /* v12 disproved the old assumption that a shallow inlet could feed a much
       steeper acquisition leg and let TAEM create the missing vertical area
       after handoff.  Put most descent into the inlet contract while preserving
       the measured-energy-selected path length.  The same shared debt limit is
       used by MM304 delivery, predictor compatibility and live capture. */
    const double inlet_slope_cap=16.0;
    double minimum_lead=height_budget/tan(maximum_slope*DEG2RAD);
    double response_limited_slope=fmin(maximum_slope,
        inlet_slope_cap+TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG);
    minimum_lead=fmax(minimum_lead,height_budget/tan(response_limited_slope*DEG2RAD));
    double maximum_lead=height_budget/tan(minimum_slope*DEG2RAD);
    double loss=drag_per_q*required_q;
    double lateral=gravity*.65*sin(40.0*DEG2RAD)*
        clampd(t->bank_effectiveness>0?t->bank_effectiveness:1.0,.35,1.4);
    double radius=ceiling*ceiling/fmax(.2,lateral);
    double turn_lead=fmax(radius*course_error*DEG2RAD,sqrt(2.0*radius*fabs(merge_cross)));
    double lead=clampd(fmax(base_lead,fmax(minimum_lead,turn_lead+ceiling*response)),
        minimum_lead,maximum_lead);
    for(int i=0;i<6;i++){
        double angle=atan2(height_budget,lead);
        double net_deceleration=fmax(gravity*.35,loss-gravity*sin(angle));
        double conversion_lead=fmax(0.0,ceiling*ceiling-base_speed*base_speed)/(2.0*net_deceleration);
        lead=clampd(fmax(base_lead,fmax(turn_lead+ceiling*response,conversion_lead)),
            minimum_lead,maximum_lead);
    }
    double slope_rad=atan2(height_budget,lead),slope=slope_rad*RAD2DEG;
    double remaining=lead+downstream;
    double target_altitude=control_altitude;
    double base_altitude=merge_altitude+fmin(base_lead,lead)*tan(slope_rad);
    double extra_work=loss*fmax(0.0,lead-base_lead)/fmax(.8,cos(slope_rad));
    double potential=rotating_specific_energy(t->latitude,target_altitude,0,p)-
        rotating_specific_energy(t->latitude,base_altitude,0,p);
    double target_speed=sqrt(fmax(base_speed*base_speed,
        base_speed*base_speed+2.0*(extra_work-potential)));
    TaemSpeedEnvelope speed_envelope=
        decision_taem_speed_envelope(v,p,target_altitude);
    double reserve_cap=speed_envelope.valid?
        fmax(base_speed,speed_envelope.maximum_speed_mps):base_speed;
    target_speed=fmin(target_speed,fmin(ceiling,reserve_cap));
    HACPoint2 acquisition={merge.e-te*lead,merge.n-tn*lead};
    TaemInterfaceTarget next={0};
    next.valid=isfinite(acquisition.e)&&isfinite(acquisition.n)&&isfinite(target_altitude)&&isfinite(target_speed);
    if(!next.valid)return;
    next.along_track=acquisition.e*sin(rh)+acquisition.n*cos(rh);
    next.cross_track=acquisition.e*cos(rh)-acquisition.n*sin(rh);
    next.course=tangent_course;next.altitude=target_altitude;next.speed=target_speed;
    /* The acquisition FPA is a real delivery contract, not one endpoint of a
       fictitious slope+12 deg ramp.  Carry as much of the geometric descent as
       the established 16 deg high-speed inlet envelope allows; the lead bound
       above limits the finite response that remains for TAEM after handoff. */
    next.flight_path_angle=-clampd(slope,4.0,inlet_slope_cap);
    next.acquisition_lead=lead;next.remaining_path=remaining;next.response_time=response;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint point=local_point(origin,acquisition.e,acquisition.n,p->radius,target_altitude);
    next.specific_energy=rotating_specific_energy(point.latitude,target_altitude,target_speed,p);
    next.selected_ut=t->ut;
    next.arrival_ut=t->ut+hypot(acquisition.e-current_e,acquisition.n-current_n)/fmax(80.0,t->horizontal_speed);
    next.quality_score=c->quality_score;
    g->taem_interface_target=next;
}

