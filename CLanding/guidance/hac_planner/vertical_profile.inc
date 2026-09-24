/*
 * A dynamic candidate must deliver the same p0 state that its committed path
 * can actually fly.  The lateral lead is already a finite C1 Bezier; its
 * vertical component must come from the exact fixed-HAC profile that commit
 * latches, not from an independently searched constant FPA.
 */
static bool fixed_hac_candidate_vertical_profile(GuidanceMachine*out,
        const GuidanceMachine*source,const Telemetry*t,
        const HACTransitionPlan*plan,const VehicleProfile*v,
        const GuidanceSettings*s,const LandingSite*site,double side,
        double*response_time_out){
    if(response_time_out)*response_time_out=NAN;
    if(!out||!source||!t||!plan||!plan->valid||!v||!s||!site||
       !isfinite(side)||side==0.0){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        static int profile_precondition_diag_count=0;
        if(diag&&strcmp(diag,"1")==0&&profile_precondition_diag_count++<12)
            fprintf(stderr,"HAC_PROFILE_PRECONDITION_REJECT out=%d source=%d telemetry=%d plan=%d planValid=%d vehicle=%d settings=%d site=%d sideFinite=%d side=%.3f\n",
                out!=NULL,source!=NULL,t!=NULL,plan!=NULL,plan?plan->valid:0,
                v!=NULL,s!=NULL,site!=NULL,isfinite(side),side);
        return false;
    }
    *out=*source;
    double loss=source->terminal_energy_loss_accel_ema;
    double speed_loss=source->terminal_speed_loss_accel_ema;
    terminal_glide_initialize(out,v,s);
    out->terminal_energy_loss_accel_ema=loss;
    out->terminal_speed_loss_accel_ema=speed_loss;
    double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    double response_time=hac_response_lead_time(t,s,side*bank_limit);
    if(!isfinite(response_time)||response_time<0.0)response_time=0.0;
    out->hac_transition_lead_length=plan->lead_length;
    out->hac_transition_cone_arc_length=plan->cone_arc_length;
    if(!fixed_hac_vertical_profile_latch(out,t->mean_altitude,
            t->flight_path_angle,t->horizontal_speed,response_time,
            plan->lead_length,plan->cone_arc_length,site,s)){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        static int profile_fail_diag_count=0;
        if(diag&&strcmp(diag,"1")==0&&!source->diagnostic_shadow&&profile_fail_diag_count++<12)
            fprintf(stderr,"HAC_PROFILE_LATCH_REJECT alt=%.3f fpa=%.3f hspeed=%.3f response=%.3f lead=%.3f arc=%.3f finalDistance=%.3f glideSlope=%.3f\n",
                t->mean_altitude,t->flight_path_angle,t->horizontal_speed,response_time,
                plan->lead_length,plan->cone_arc_length,
                out->terminal_test_final_approach_distance,out->terminal_test_glide_slope);
        return false;
    }
    if(response_time_out)*response_time_out=response_time;
    return true;
}

static bool fixed_hac_vertical_profile_executable(const GuidanceMachine*profile,
        const GuidanceSettings*s){
    if(!profile||!s||!profile->fixed_hac_reference_valid)return false;
    double final_slope=profile->terminal_glide_mode&&
        profile->terminal_test_glide_slope>0.0?
        profile->terminal_test_glide_slope:s->final_glide_slope;
    double initial_slope=profile->fixed_hac_reference_initial_slope_deg;
    double settled_slope=profile->fixed_hac_reference_settled_slope_deg;
    double exit_slope=fixed_hac_reference_slope(profile,0.0);
    double continuity_tolerance=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(fabs(exit_slope),fabs(final_slope)));
    return isfinite(initial_slope)&&isfinite(settled_slope)&&
        isfinite(final_slope)&&isfinite(exit_slope)&&
        initial_slope>=0.0&&initial_slope<90.0&&
        settled_slope>=0.0&&settled_slope<90.0&&
        final_slope>=0.0&&final_slope<90.0&&
        fabs(exit_slope-final_slope)<=continuity_tolerance;
}

static bool terminal_fixed_hac_lead_reachable(const GuidanceMachine*g,
        const Telemetry*initial,const HACTransitionPlan*plan,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        const LandingSite*site,double target_speed,
        const GuidanceMachine*vertical_profile,Telemetry*out,
        double*work_out,double*course_error_out,double*minimum_lateral_margin_out){
    if(out)memset(out,0,sizeof(*out));
    if(work_out)*work_out=NAN;
    if(course_error_out)*course_error_out=INFINITY;
    if(minimum_lateral_margin_out)*minimum_lateral_margin_out=-INFINITY;
    if(!g||!initial||!plan||!plan->valid||!p||!v||!s||!site||
       !vertical_profile||!vertical_profile->fixed_hac_reference_valid||
       !isfinite(target_speed)||!(target_speed>0.0)){
        const char*pre_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        static int lead_precondition_diag_count=0;
        if(pre_diag&&strcmp(pre_diag,"1")==0&&lead_precondition_diag_count++<12)
            fprintf(stderr,"HAC_LEAD_PRECONDITION_REJECT g=%d initial=%d plan=%d planValid=%d planet=%d vehicle=%d settings=%d site=%d profile=%d profileValid=%d speedFinite=%d speed=%.6f\n",
                g!=NULL,initial!=NULL,plan!=NULL,plan?plan->valid:0,p!=NULL,v!=NULL,s!=NULL,site!=NULL,
                vertical_profile!=NULL,vertical_profile?vertical_profile->fixed_hac_reference_valid:0,
                isfinite(target_speed),target_speed);
        return false;
    }
    const char*lead_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    static int lead_begin_diag_count=0;
    if(lead_diag&&strcmp(lead_diag,"1")==0&&!g->diagnostic_shadow&&lead_begin_diag_count++<12)
        fprintf(stderr,"HAC_LEAD_BEGIN target=%.3f lead=%.3f arc=%.3f profileStart=%.3f profileFinal=%.3f settledSlope=%.3f transition=%.3f initialE=%.3f initialN=%.3f p0E=%.3f p0N=%.3f startCourse=%.3f endCourse=%.3f p1=(%.3f,%.3f) p2=(%.3f,%.3f) speed=%.6f hspeed=%.6f fpa=%.6f aoa=%.6f q=%.6f\n",
            target_speed,plan->lead_length,plan->cone_arc_length,
            vertical_profile->fixed_hac_reference_start_altitude,
            vertical_profile->fixed_hac_reference_final_altitude,
            vertical_profile->fixed_hac_reference_settled_slope_deg,
            vertical_profile->fixed_hac_reference_transition_distance,
            initial->runway_along_track,initial->runway_cross_track,
            plan->p0.e,plan->p0.n,plan->lead_start_course,plan->lead_end_course,
            plan->lead_p1.e,plan->lead_p1.n,plan->lead_p2.e,plan->lead_p2.n,
            initial->true_air_speed,
            initial->horizontal_speed,initial->flight_path_angle,
            initial->angle_of_attack,initial->dynamic_pressure);

    double total_profile_path=plan->lead_length+plan->cone_arc_length;
    double stored_profile_path=fixed_hac_total_path_length(vertical_profile);
    if(!(total_profile_path>0.0)||!isfinite(total_profile_path)||
       !isfinite(stored_profile_path)||
       fabs(stored_profile_path-total_profile_path)>
           sqrt(DBL_EPSILON)*fmax(1.0,total_profile_path))
        return false;

    HACTransitionPlan lead=hac_lead_bezier_from_plan(plan);
    bool acquisition=plan->lead_acquisition;
    double lead_length=plan->lead_length;
    if((!acquisition&&!lead.lead_curve)||!(lead_length>1.0)||!isfinite(lead_length)){
        if(out)*out=*initial;
        if(work_out)*work_out=0.0;
        if(course_error_out)*course_error_out=0.0;
        if(minimum_lateral_margin_out)*minimum_lateral_margin_out=INFINITY;
        return initial->true_air_speed+1.0>=target_speed;
    }

    double initial_density=planet_atmospheric_density(p,initial->mean_altitude);
    double initial_df=1.0;
    aerodynamic_force_factors_mach(initial->mach,
        fmax(.25,hypot(initial->angle_of_attack,initial->sideslip)),v,
        NULL,&initial_df);
    double modeled0=.5*initial_density*initial->true_air_speed*
        initial->true_air_speed/fmax(20.0,aero.ballistic_coefficient)*initial_df;
    double observed0=terminal_expected_energy_loss_accel(g,initial,aero,v);
    if(!isfinite(initial_density)||initial_density<0.0||
       !isfinite(modeled0)||modeled0<0.0||!isfinite(observed0)||observed0<0.0)
        return false;
    double anchor=modeled0>DBL_MIN?observed0/modeled0:1.0;
    if(!isfinite(anchor)||anchor<0.0)return false;

    Telemetry sim=*initial;
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    double latitude=initial->latitude,longitude=initial->longitude,elapsed=0.0;
    sim.has_force_vectors=false;sim.has_attitude_quaternion=false;
    sim.has_command_pitch_error=false;sim.has_command_roll_error=false;
    sim.has_command_heading_error=false;sim.physics_sample_valid=false;
    double speed=fmax(1.0,initial->true_air_speed);
    double altitude=initial->mean_altitude;
    double flight_path_angle=initial->flight_path_angle*DEG2RAD;
    double reference_start=fixed_hac_reference_altitude(vertical_profile,
        total_profile_path);
    if(!isfinite(reference_start)||
       fabs(reference_start-altitude)>sqrt(DBL_EPSILON)*fmax(1.0,altitude))
        return false;
    double aoa=clampd(fabs(initial->angle_of_attack),0.0,
        v->maximum_angle_of_attack);
    JerkLimiter aoa_limiter=g->pitch_limiter;
    seed_stabilized_limiter(&aoa_limiter,aoa,
        controlled_aoa_rate(initial),false);
    GuidanceAttitudeLimits pitch_limits=stabilized_attitude_limits(
        initial,s,PHASE_TAEM,true);
    double course=isfinite(initial->ground_track_heading)?
        initial->ground_track_heading:initial->heading;
    if(!isfinite(course))course=plan->lead_start_course;
    double work=0.0,travelled=0.0;
    double lead_min_radius=INFINITY,lead_peak_required=0.0;
    double lead_peak_curvature=0.0,lead_peak_s=NAN,lead_peak_u=NAN;
    double lead_peak_speed=NAN,lead_peak_slope=NAN,lead_peak_available=NAN;
    double minimum_lateral_margin=INFINITY;
    size_t steps=(size_t)ceil(lead_length/300.0);
    steps=(size_t)clampd((double)steps,16.0,300.0);
    for(size_t i=0;i<steps;i++){
        double u0=(double)i/(double)steps;
        double u1=(double)(i+1)/(double)steps;
        double travelled0=lead_length*u0;
        double travelled1=lead_length*u1;
        double ds=travelled1-travelled0;
        if(!acquisition){
            ds=hac_bezier_length_between(&lead,u0,u1,10);
            travelled0=travelled;
            travelled1=travelled0+ds;
        }
        if(!(ds>0.0)||!isfinite(ds))return false;
        double profile_mid_remaining=fmax(0.0,total_profile_path-
            (travelled0+.5*ds));
        double lead_slope=fixed_hac_reference_slope(vertical_profile,
            profile_mid_remaining);
        if(!isfinite(lead_slope)||lead_slope<0.0||lead_slope>=75.0||
           !isfinite(altitude)||altitude<0.0)return false;
        double cosine=cos(flight_path_angle);
        if(!(cosine>0.0))return false;

        double um=.5*(u0+u1);
        double mid_distance=.5*(travelled0+travelled1);
        HACPoint2 mid_point={0};
        double tangent=NAN,curvature=NAN;
        if(acquisition){
            if(!hac_lead_sample_distance(plan,mid_distance,&mid_point,
                    &tangent,&curvature))return false;
        }else{
            HACPoint2 d=hac_bezier_derivative(&lead,um);
            tangent=norm_deg(atan2(d.e,d.n)*RAD2DEG);
            curvature=hac_bezier_signed_curvature(&lead,um);
        }
        if(isfinite(curvature)){
            lead_peak_curvature=fmax(lead_peak_curvature,fabs(curvature));
            if(fabs(curvature)>DBL_MIN)
                lead_min_radius=fmin(lead_min_radius,1.0/fabs(curvature));
        }
        double dt=ds/(fmax(speed,1.0)*cosine);
        if(!(dt>0.0)||!isfinite(dt))return false;
        course=norm_deg(course+clampd(norm_signed_deg(tangent-course),
            -hac_course_rate_cap(&sim)*dt,hac_course_rate_cap(&sim)*dt));

        sim.mean_altitude=altitude;
        sim.true_air_speed=speed;sim.horizontal_speed=speed*cosine;
        sim.surface_speed=speed;sim.flight_path_angle=flight_path_angle*RAD2DEG;
        sim.vertical_speed=speed*sin(flight_path_angle);
        sim.ut=initial->ut+elapsed;sim.latitude=latitude;sim.longitude=longitude;
        sim.angle_of_attack=aoa;sim.sideslip=0.0;
        sim.atmospheric_density=planet_atmospheric_density(p,altitude);
        sim.speed_of_sound=planet_atmospheric_speed_of_sound(p,altitude);
        sim.mach=sim.speed_of_sound>DBL_MIN?speed/sim.speed_of_sound:NAN;
        sim.dynamic_pressure=.5*sim.atmospheric_density*speed*speed;
        sim.lift_force=terminal_fixed_hac_projected_lift(g,&sim,aero,v,aoa)*
            fmax(1.0,sim.mass);
        sim.drag_force=0.0;
        if(!isfinite(sim.atmospheric_density)||sim.atmospheric_density<0.0||
           !isfinite(sim.speed_of_sound)||sim.speed_of_sound<=0.0||
           !isfinite(sim.dynamic_pressure))return false;

        double remaining=fmax(100.0,lead_length-travelled0);
        /* Optional fly-and-learn fixture export.  This is strictly diagnostic:
           it reports one state from the same finite-lead propagator used by
           admission, but never changes the propagated state or any gate. */
        const char*sample_env=getenv("KSP_LANDER_HAC_LEAD_SAMPLE_FRACTION");
        static bool lead_sample_emitted=false;
        if(sample_env&&!lead_sample_emitted&&!g->diagnostic_shadow&&
           isfinite(initial->mean_altitude)&&initial->mean_altitude>12000.0&&
           initial->true_air_speed>190.0&&initial->true_air_speed<200.0&&
           plan->lead_length>3000.0&&plan->lead_length<3100.0){
            char*end=NULL;
            double fraction=strtod(sample_env,&end);
            double sample_s=plan->lead_length*clampd(fraction,0.05,0.95);
            if(end!=sample_env&&isfinite(sample_s)&&
               sample_s>=travelled0&&sample_s<=travelled1){
                HACPoint2 sample_point={0};
                double sample_course=NAN,sample_curvature=NAN;
                if(acquisition)
                    (void)hac_lead_sample_distance(plan,sample_s,&sample_point,
                        &sample_course,&sample_curvature);
                else{
                    sample_point=hac_bezier_point(&lead,u0);
                    HACPoint2 sample_d=hac_bezier_derivative(&lead,u0);
                    sample_course=norm_deg(atan2(sample_d.e,sample_d.n)*RAD2DEG);
                }
                fprintf(stderr,"HAC_LEAD_SAMPLE fraction=%.9f s=%.9f remaining=%.9f e=%.12f n=%.12f course=%.12f altitude=%.12f speed=%.12f fpa=%.12f aoa=%.12f q=%.12f\\n",
                    sample_s/plan->lead_length,sample_s,lead_length-sample_s,
                    sample_point.e,sample_point.n,sample_course,altitude,speed,
                    -lead_slope,aoa,sim.dynamic_pressure);
                lead_sample_emitted=true;
            }
        }
        double fpa_aoa=terminal_fpa_force_aoa(&sim,p,aero,v,-lead_slope);
        double drag_requested=fixed_hac_lead_drag_aoa(&sim,p,aero,v,target_speed,
            remaining,lead_slope);
        double energy_aoa=isfinite(drag_requested)?drag_requested:
            terminal_hac_energy_aoa(&sim,v);
        bool speed_not_overspeed=isfinite(target_speed)&&
            speed<=target_speed+45.0;
        bool lead_vertical_recovery=flight_path_angle*RAD2DEG<
            -lead_slope-1.0;
        double requested=speed_not_overspeed&&!lead_vertical_recovery?
            fmin(isfinite(fpa_aoa)?fpa_aoa:energy_aoa,
                 fmax(energy_aoa,isfinite(drag_requested)?drag_requested:0.0)):
            fmax(isfinite(fpa_aoa)?fpa_aoa:0.0,
                 isfinite(drag_requested)?drag_requested:0.0);
        if(!isfinite(requested)||requested<=0.0)requested=aoa;
        double lead_aoa_bias=hac_lead_aoa_bias_deg();
        if(lead_aoa_bias>0.0&&isfinite(energy_aoa))
            requested=fmax(requested,energy_aoa+lead_aoa_bias);
        double lookahead_u=fmin(1.0,((double)i+1.5)/(double)steps);
        double lookahead_curvature=NAN;
        if(acquisition){
            HACPoint2 lookahead_point={0};double lookahead_course=NAN;
            double lookahead_distance=fmin(lead_length,
                lead_length*lookahead_u);
            if(!hac_lead_sample_distance(plan,lookahead_distance,
                    &lookahead_point,&lookahead_course,&lookahead_curvature))
                return false;
        }else
            lookahead_curvature=hac_bezier_signed_curvature(&lead,lookahead_u);
        double lateral_curvature=fmax(fabs(curvature),fabs(lookahead_curvature));
        if(lateral_curvature>DBL_MIN){
            double lead_bank_limit=fmin(v->maximum_bank_angle,
                dynamic_bank_limit(&sim,v));
            double horizontal_speed=speed*fabs(cos(flight_path_angle));
            double lateral_request=horizontal_speed*horizontal_speed*
                lateral_curvature;
            double lateral_aoa=terminal_required_aoa_for_lateral_limit(&sim,aero,v,
                lateral_request,lead_bank_limit,v->maximum_angle_of_attack);
            if(isfinite(lateral_aoa))requested=hypot(fmax(0.0,requested),fmax(0.0,lateral_aoa));
        }
        requested=clampd(requested,0.0,v->maximum_angle_of_attack);
        static int lead_first_step_diag_count=0;
        if(i==0&&lead_diag&&strcmp(lead_diag,"1")==0&&!g->diagnostic_shadow&&
           lead_first_step_diag_count++<12)
            fprintf(stderr,"HAC_LEAD_FIRST_STEP target=%.6f speedIn=%.6f slope=%.6f dt=%.6f remaining=%.6f fpaAoA=%.6f dragAoA=%.6f energyAoA=%.6f requestedAoA=%.6f initialAoA=%.6f anchor=%.6f modeledDrag=%.6f observedDrag=%.6f\n",
                target_speed,speed,lead_slope,dt,remaining,
                fpa_aoa,drag_requested,energy_aoa,requested,aoa,
                anchor,modeled0,observed0);
        static int lead_aoa_sample_diag_count=0;
        if(lead_diag&&strcmp(lead_diag,"1")==0&&!g->diagnostic_shadow&&
           lead_aoa_sample_diag_count<96&&
           (i==0||i==steps/4||i==steps/2||i==(steps*3)/4||i+1==steps)){
            fprintf(stderr,
                "HAC_LEAD_AOA_SAMPLE i=%zu steps=%zu target=%.6f speed=%.6f altitude=%.3f fpa=%.3f remaining=%.6f slopeRef=%.6f fpaAoA=%.6f dragAoA=%.6f energyAoA=%.6f requestedAoA=%.6f appliedAoA=%.6f speedNotOverspeed=%d\n",
                i,steps,target_speed,speed,altitude,flight_path_angle*RAD2DEG,
                remaining,lead_slope,fpa_aoa,
                drag_requested,energy_aoa,requested,aoa,speed_not_overspeed?1:0);
            lead_aoa_sample_diag_count++;
        }
        aoa=clampd(jerk_update(&aoa_limiter,requested,
            pitch_limits.rate_deg_s,pitch_limits.accel_deg_s2,dt),
            0.0,v->maximum_angle_of_attack);
        aoa_limiter.value=aoa;
        /* Authority must be evaluated after the finite AoA response for this
           sample, not before the lead command has had any chance to produce
           the lift required by its curvature.  This remains a strict gate:
           the rate/acceleration limits above can still leave the candidate
           below the required lift, in which case it is rejected. */
        sim.angle_of_attack=aoa;
        sim.lift_force=terminal_fixed_hac_projected_lift(g,&sim,aero,v,aoa)*
            fmax(1.0,sim.mass);
        if(fabs(curvature)>DBL_MIN){
            double lead_radius=1.0/fabs(curvature);
            double required=NAN,available=NAN,minimum=INFINITY;
            bool authority=terminal_fixed_hac_authority(g,&sim,p,aero,v,s,
                lead_radius,&required,&available,&minimum);
            if(isfinite(required)&&required>lead_peak_required){
                lead_peak_required=required;
                lead_peak_s=travelled0+.5*ds;
                lead_peak_u=acquisition?
                    clampd(mid_distance/fmax(lead_length,DBL_MIN),0.0,1.0):um;
                lead_peak_speed=sim.true_air_speed;
                lead_peak_slope=lead_slope;
                lead_peak_available=available;
            }
            if(!authority||!isfinite(required)||!isfinite(available)){
                const char*lead_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
                static int lead_authority_diag_count=0;
                if(lead_diag&&strcmp(lead_diag,"1")==0&&
                   !g->diagnostic_shadow&&lead_authority_diag_count++<12){
                    double delta_heading=fabs(norm_signed_deg(
                        plan->lead_end_course-plan->lead_start_course));
                    double turn_budget=isfinite(minimum)&&minimum>DBL_MIN?
                        lead_length/minimum:NAN;
                    double arc_angle=fabs(plan->cone_end_angle-
                        plan->cone_start_angle);
                    double arc_radius=arc_angle>DBL_MIN?
                        plan->cone_arc_length/arc_angle:NAN;
                    double arc_required=isfinite(arc_radius)&&arc_radius>0.0?
                        sim.true_air_speed*sim.true_air_speed/arc_radius:NAN;
                    double arc_available=available;
                    fprintf(stderr,"HAC_LEAD_AUTHORITY_REJECT arc=%.1f speed=%.3f slope=%.3f curvature=%.9f leadR=%.1f required=%.6f available=%.6f minR=%.1f aoa=%.3f q=%.3f mach=%.3f mass=%.1f liftForce=%.3f liftAccel=%.6f dynBank=%.3f bankEff=%.3f maxG=%.3f leadLength=%.3f leadMinR=%.3f leadPeakReq=%.6f leadPeakCurv=%.9f peakS=%.3f peakU=%.6f peakSpeed=%.3f peakSlope=%.3f peakAvailable=%.6f deltaHeading=%.3f turnBudget=%.6f turnBudgetPass=%d arcRadius=%.3f arcRequiredAtR=%.6f arcAvailable=%.6f arcPass=%d\n",
                        plan->cone_arc_length,speed,lead_slope,curvature,lead_radius,
                        required,available,minimum,sim.angle_of_attack,sim.dynamic_pressure,
                        sim.mach,sim.mass,sim.lift_force,live_lift_accel(&sim,aero,v),
                        dynamic_bank_limit(&sim,v),sim.bank_effectiveness,
                        v->maximum_g_load,lead_length,lead_min_radius,
                        lead_peak_required,lead_peak_curvature,lead_peak_s,lead_peak_u,
                        lead_peak_speed,lead_peak_slope,lead_peak_available,delta_heading,
                        turn_budget,isfinite(turn_budget)&&
                        isfinite(delta_heading)&&turn_budget+1e-9>=
                        delta_heading*DEG2RAD,arc_radius,arc_required,
                        arc_available,isfinite(arc_required)&&
                        arc_available>=arc_required?1:0);
                }
                return false;
            }
            minimum_lateral_margin=fmin(minimum_lateral_margin,
                available-required);
        }

        double df=1.0;
        aerodynamic_force_factors_mach(sim.mach,aoa,v,NULL,&df);
        double drag=.5*sim.atmospheric_density*speed*speed/
            fmax(20.0,aero.ballistic_coefficient)*df*anchor;
        if(!isfinite(drag)||drag<0.0)return false;
        double next_altitude=NAN,next_fpa=NAN;
        double lift_accel=terminal_fixed_hac_projected_lift(g,&sim,aero,v,aoa);
        if(!fixed_hac_lead_vertical_step(&sim,p,v,curvature,lift_accel,dt,
                &next_altitude,&next_fpa)){
            static int lead_vertical_reject_diag_count=0;
            if(lead_diag&&strcmp(lead_diag,"1")==0&&!g->diagnostic_shadow&&
               lead_vertical_reject_diag_count++<12)
                fprintf(stderr,"HAC_LEAD_VERTICAL_REJECT i=%zu steps=%zu s=%.3f alt=%.3f speed=%.3f fpa=%.3f slopeRef=%.3f aoa=%.3f lift=%.6f curvature=%.9f dt=%.6f\n",
                    i,steps,travelled0,altitude,speed,
                    flight_path_angle*RAD2DEG,lead_slope,aoa,lift_accel,
                    curvature,dt);
            return false;
        }
        double air_ds=speed*dt;
        HACPoint2 next_local={0};
        if(acquisition){
            double next_course=NAN,next_curvature=NAN;
            if(!hac_lead_sample_distance(plan,travelled1,&next_local,
                    &next_course,&next_curvature))return false;
        }else
            next_local=hac_bezier_point(&lead,u1);
        GeoPoint next_point=local_point(origin,next_local.e,next_local.n,p->radius,
            next_altitude);
        double potential_drop=rotating_specific_energy(latitude,altitude,0.0,p)-
            rotating_specific_energy(next_point.latitude,next_altitude,0.0,p);
        double next_speed=NAN,segment_work=NAN;
        if(!terminal_quadratic_drag_energy_step(speed,drag,air_ds,potential_drop,
                &next_speed,&segment_work))return false;
        work+=segment_work;
        elapsed+=dt;
        speed=next_speed;altitude=next_altitude;flight_path_angle=next_fpa;
        travelled=travelled1;
        latitude=next_point.latitude;longitude=next_point.longitude;
    }

    double p0_slope=fixed_hac_reference_slope(vertical_profile,
        plan->cone_arc_length);
    double p0_altitude=fixed_hac_reference_altitude(vertical_profile,
        plan->cone_arc_length);
    if(!isfinite(p0_slope)||!isfinite(p0_altitude))return false;
    sim.mean_altitude=altitude;sim.true_air_speed=speed;
    sim.horizontal_speed=speed*cos(flight_path_angle);
    sim.surface_speed=speed;sim.flight_path_angle=flight_path_angle*RAD2DEG;
    sim.vertical_speed=speed*sin(flight_path_angle);
    sim.ut=initial->ut+elapsed;sim.latitude=latitude;sim.longitude=longitude;
    sim.radar_altitude=fmax(0.0,altitude-site->altitude);
    GeoPoint p0_point={latitude,longitude,altitude};
    runway_coordinates(p0_point,origin,site->runway_heading,p->radius,
        &sim.runway_along_track,&sim.runway_cross_track);
    sim.range_to_site=great_circle_distance(p0_point,origin,p->radius);
    sim.pitch=sim.flight_path_angle+aoa;
    sim.angle_of_attack=aoa;sim.angle_of_attack_rate=aoa_limiter.rate;
    sim.has_angle_of_attack_rate=true;sim.heading=course;
    sim.ground_track_heading=course;sim.mach=planet_atmospheric_speed_of_sound(
        p,altitude)>DBL_MIN?speed/planet_atmospheric_speed_of_sound(p,altitude):NAN;
    sim.speed_of_sound=planet_atmospheric_speed_of_sound(p,altitude);
    sim.atmospheric_density=planet_atmospheric_density(p,altitude);
    sim.dynamic_pressure=.5*sim.atmospheric_density*speed*speed;
    sim.lift_force=terminal_fixed_hac_projected_lift(g,&sim,aero,v,aoa)*
        fmax(1.0,sim.mass);
    double course_error=fabs(norm_signed_deg(course-plan->lead_end_course));
    double p0_altitude_error=fabs(altitude-p0_altitude);
    double p0_fpa_error=fabs(norm_signed_deg(sim.flight_path_angle+p0_slope));
    bool p0_vertical_match=p0_altitude_error<=1000.0&&p0_fpa_error<=5.0;
    double p0_required=NAN,p0_available=NAN,p0_minimum_radius=NAN;
    bool p0_authority=terminal_fixed_hac_authority(g,&sim,p,aero,v,s,12000.0,
        &p0_required,&p0_available,&p0_minimum_radius);
    static int lead_p0_authority_diag_count=0;
    if(lead_diag&&strcmp(lead_diag,"1")==0&&!g->diagnostic_shadow&&
       lead_p0_authority_diag_count++<12)
        fprintf(stderr,"HAC_LEAD_P0_AUTHORITY speed=%.3f altitude=%.3f q=%.3f mach=%.3f required=%.6f available=%.6f minimumRadius=%.3f fixedRadius=12000 pass=%d\n",
            speed,altitude,sim.dynamic_pressure,sim.mach,p0_required,
            p0_available,p0_minimum_radius,p0_authority?1:0);
    static int lead_end_diag_count=0;
    if(lead_diag&&strcmp(lead_diag,"1")==0&&!g->diagnostic_shadow&&
       lead_end_diag_count++<12)
        fprintf(stderr,"HAC_LEAD_END target=%.6f lead=%.6f arc=%.6f elapsed=%.6f p0Speed=%.6f p0Alt=%.6f p0Fpa=%.6f p0RefAlt=%.6f p0RefFpa=%.6f p0AoA=%.6f p0Course=%.6f courseError=%.6f work=%.6f lateralMargin=%.6f\n",
            target_speed,plan->lead_length,plan->cone_arc_length,elapsed,
            speed,altitude,sim.flight_path_angle,p0_altitude,-p0_slope,
            aoa,course,course_error,work,
            minimum_lateral_margin);
    if(work_out)*work_out=work;
    if(course_error_out)*course_error_out=course_error;
    if(minimum_lateral_margin_out)*minimum_lateral_margin_out=minimum_lateral_margin;
    if(out)*out=sim;
    bool lead_ok=isfinite(course_error)&&course_error<=5.0&&
        isfinite(minimum_lateral_margin)&&minimum_lateral_margin>=0.0&&
        speed>0.0&&isfinite(altitude)&&p0_vertical_match;
    if(!lead_ok){
        const char*lead_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        static int lead_final_diag_count=0;
        if(lead_diag&&strcmp(lead_diag,"1")==0&&
           !g->diagnostic_shadow&&lead_final_diag_count++<12)
            fprintf(stderr,"HAC_LEAD_FINAL_REJECT lead=%.1f arc=%.1f courseError=%.6f lateralMargin=%.6f speed=%.3f alt=%.3f slope=%.3f p0AltError=%.1f p0FpaError=%.2f\n",
                plan->lead_length,plan->cone_arc_length,course_error,
                minimum_lateral_margin,speed,altitude,p0_slope,
                p0_altitude_error,p0_fpa_error);
    }
    return lead_ok;
}

                
               
                          
                                                                             
                                    
                                              
                                                                                
                                                                   
                                    
                                                               
                                                                 
                                                                 
                                                                        
                                                                       
                                                      
                                                           
                                                                    
                                                                         
                               
                                                          
                      
