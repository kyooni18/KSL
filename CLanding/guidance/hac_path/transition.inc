static bool hac_transition_plan_entry(HACTransitionPlan*out,double current_e,double current_n,
        double future_e,double future_n,double start_course,double future_course,
        const LandingSite*site,const GuidanceSettings*s,double hac_radius,double side,
        double start_air_speed,double end_air_speed,double speed_loss_accel,
        double max_lateral_accel,double preferred_fixed_path,
        double min_fixed_path,double max_fixed_path,double entry_course){
    if(!out||!site||!s||!(hac_radius>0.0)||!(max_lateral_accel>0.0)||
       !isfinite(current_e)||!isfinite(current_n)||
       !isfinite(future_e)||!isfinite(future_n)||
       !isfinite(start_course)||!isfinite(future_course)||!isfinite(entry_course)||
       !isfinite(preferred_fixed_path)||!isfinite(min_fixed_path)||
       !isfinite(max_fixed_path)||max_fixed_path<min_fixed_path||
       fabs(side)!=1.0)return false;
    memset(out,0,sizeof(*out));

    double runway=site->runway_heading*DEG2RAD;
    double future_heading=entry_course*DEG2RAD;
    double start_heading=start_course*DEG2RAD;
    double vh_e=sin(runway),vh_n=cos(runway);
    double rh_e=cos(runway),rh_n=-sin(runway);
    double rf_e=cos(future_heading),rf_n=-sin(future_heading);
    double vf_e=sin(future_heading),vf_n=cos(future_heading);
    double vs_e=sin(start_heading),vs_n=cos(start_heading);

    /* The outer-final endpoint is part of the HAC definition.  Fixing it here
       makes geometry, energy accounting and final-intercept ownership refer to
       the same physical station. */
    HACPoint2 exit=hac_point(
        -vh_e*s->final_approach_distance,
        -vh_n*s->final_approach_distance);
    HACPoint2 center=hac_point(
        exit.e+side*hac_radius*rh_e,
        exit.n+side*hac_radius*rh_n);
    HACPoint2 entry=hac_point(
        center.e-side*hac_radius*rf_e,
        center.n-side*hac_radius*rf_n);

    /* Normal HAC owns the shortest partial heading-alignment arc.  The
       opposite-side long way around is an energy-management circuit, not a
       heading-alignment cone; MM304 must dissipate that excess before handoff. */
    double arc_angle_deg=norm_deg(side*(site->runway_heading-entry_course));
    if(arc_angle_deg>180.0+sqrt(DBL_EPSILON)*RAD2DEG)
        return false;
    if(arc_angle_deg<sqrt(DBL_EPSILON)*RAD2DEG)
        arc_angle_deg=0.0;
    double arc_angle=arc_angle_deg*DEG2RAD;
    double start_angle=atan2(entry.n-center.n,entry.e-center.e);
    double end_angle=start_angle-side*arc_angle;
    double arc_length=arc_angle*hac_radius;

    HACTransitionPlan candidate={0};
    candidate.valid=true;
    candidate.heading_cone=true;
    candidate.lead_curve=true;
    candidate.lead_start=hac_point(current_e,current_n);
    candidate.p0=entry;
    candidate.p3=exit;
    candidate.cone_center=center;
    candidate.cone_start_angle=start_angle;
    candidate.cone_end_angle=end_angle;
    candidate.cone_arc_length=arc_length;
    candidate.lead_start_course=norm_deg(start_course);
    candidate.lead_end_course=norm_deg(entry_course);
    candidate.length=arc_length;
    candidate.end_angle=end_angle;
    candidate.arc_remaining=0.0;

    /* A cubic Hermite bridge is used only for the finite actuator-response
       acquisition.  Its endpoint tangents are the measured current vector and
       the exact cone-entry tangent, so the commanded path is C1 continuous.
       One-third chord handles are the standard cubic Hermite scaling and do
       not introduce a flight-policy preference. */
    double lead_chord=hypot(entry.e-current_e,entry.n-current_n);
    double response_distance=hypot(future_e-current_e,future_n-current_n);
    double response_course_delta=fabs(norm_signed_deg(future_course-start_course));
    double guidance_frame=1.0/fmax(1.0,s->guidance_rate);
    double one_frame_distance=fmax(1.0,start_air_speed*guidance_frame);
    double one_frame_course=fmax(sqrt(DBL_EPSILON)*RAD2DEG,
        hac_course_rate_cap_for_speed(start_air_speed)*guidance_frame);
    bool response_already_settled=
        response_distance<=one_frame_distance&&
        response_course_delta<=one_frame_course&&
        lead_chord<=one_frame_distance;

    if(lead_chord>sqrt(DBL_EPSILON)*fmax(1.0,hac_radius)&&
       !response_already_settled){
        /* Fixed chord/3 handles put a tight kink at the start of the bridge
           whenever the cone entry is laterally offset from the live track;
           sweep logs showed peaks of 20-29x the available lateral
           acceleration.  Evaluate a small family of start/end handle lengths
           and keep the bridge with the smallest normalized violation.  Every
           qualification below is unchanged; only the curve shape is chosen. */
        static const double handle_scales[]={1.0/3.0,0.5,0.75,1.0,1.25};
        const size_t handle_count=sizeof(handle_scales)/sizeof(handle_scales[0]);
        double fit_position_limit=fmax(1.0,s->mm304_handoff_radius);
        double fit_course_limit=fmax(1.0,s->mm304_perpendicular_heading_half_width);
        double rate_limit=hac_transition_rate_ratio_limit();
        bool have_lead=false,have_standard=false;
        double best_score=INFINITY;
        HACTransitionPlan best={0},standard={0};
        for(size_t a=0;a<handle_count;a++)for(size_t b=0;b<handle_count;b++){
            HACTransitionPlan trial_plan=candidate;
            double h1=lead_chord*handle_scales[a],h2=lead_chord*handle_scales[b];
            trial_plan.lead_p1=hac_point(current_e+vs_e*h1,current_n+vs_n*h1);
            trial_plan.lead_p2=hac_point(entry.e-vf_e*h2,entry.n-vf_n*h2);
            HACTransitionPlan lead=hac_lead_bezier_from_plan(&trial_plan);
            if(!hac_bezier_regular(&lead))continue;
            trial_plan.lead_length=hac_bezier_length_between(&lead,0.0,1.0,32);
            if(!(trial_plan.lead_length>0.0)||!isfinite(trial_plan.lead_length))
                continue;
            /* hac_handoff_geometry_valid() bounds the bridge to 1.25 chords. */
            if(trial_plan.lead_length>lead_chord*1.25)continue;

            /* The lead must also agree with the finite-response state used to
               design it.  Evaluate where the response-projected travel distance
               falls on the lead and measure both position and tangent residual. */
            double response_u=hac_bezier_advance_distance(&lead,0.0,response_distance);
            HACPoint2 response_point=hac_bezier_point(&lead,response_u);
            HACPoint2 response_tangent=hac_bezier_derivative(&lead,response_u);
            double response_fit=hypot(response_point.e-future_e,
                response_point.n-future_n);
            double response_course=norm_deg(
                atan2(response_tangent.e,response_tangent.n)*RAD2DEG);
            double response_course_error=fabs(
                norm_signed_deg(response_course-future_course));
            double fit_violation=fmax(
                terminal_normalized_upper_violation(response_fit,fit_position_limit),
                terminal_normalized_upper_violation(response_course_error,fit_course_limit));
            trial_plan.degraded_end=fit_violation>0.0;
            trial_plan.violation_score=fit_violation;
            trial_plan.peak_lateral=0.0;
            trial_plan.opposite_lateral=0.0;
            trial_plan.peak_course_rate_ratio=0.0;

            /* Qualify every acquisition curvature, not only the downstream cone.
               This is the path the shuttle will actually fly while its bank is
               responding. */
            double distance_along=0.0;
            HACPoint2 previous=lead.p0;
            for(int i=0;i<=32;i++){
                double u=(double)i/32.0;
                HACPoint2 point=hac_bezier_point(&lead,u);
                if(i>0)distance_along+=hypot(point.e-previous.e,point.n-previous.n);
                previous=point;
                double decay=fmax(0.0,speed_loss_accel)*distance_along/
                    fmax(start_air_speed*start_air_speed,DBL_MIN);
                double speed=fmax(end_air_speed,start_air_speed*exp(-decay));
                double curvature=hac_bezier_signed_curvature(&lead,u);
                double lateral=speed*speed*curvature;
                trial_plan.peak_lateral=fmax(trial_plan.peak_lateral,fabs(lateral));
                if(lateral*side<0.0)
                    trial_plan.opposite_lateral=fmax(trial_plan.opposite_lateral,
                        fabs(lateral));
                double rate=fabs(speed*curvature)*RAD2DEG;
                double rate_cap=hac_course_rate_cap_for_speed(speed);
                trial_plan.peak_course_rate_ratio=fmax(
                    trial_plan.peak_course_rate_ratio,
                    rate/fmax(rate_cap,DBL_MIN));
            }
            double score=fmax(fit_violation,fmax(
                terminal_normalized_upper_violation(trial_plan.peak_lateral,max_lateral_accel),
                terminal_normalized_upper_violation(trial_plan.peak_course_rate_ratio,rate_limit)));
            /* The first pair is the standard chord/3 Hermite; another shape
               replaces it only when it strictly reduces the violation. */
            if(!isfinite(score))continue;
            if(!have_standard&&a==0&&b==0){have_standard=true;standard=trial_plan;}
            if(!have_lead||score<best_score-1e-9){
                have_lead=true;best_score=score;best=trial_plan;
            }
        }
        if(!have_lead)return false;
        /* Only a fully qualified alternative replaces the standard bridge; a
           merely less-infeasible curve is rejected downstream either way. */
        candidate=(best_score<=0.0||!have_standard)?best:standard;
    }else{
        candidate.lead_curve=false;
        candidate.lead_p1=candidate.lead_start;
        candidate.lead_p2=entry;
        candidate.lead_length=0.0;
    }

    /* p1/p2 are retained only for visual/debug compatibility.  The flight law
       and progress state use the analytic cone, so large partial arcs never
       depend on a single cubic approximation. */
    double handle=fmax(100.0,
        4.0*hac_radius*sin(fmin(arc_angle*.25,LANDER_PI*.25))/3.0);
    candidate.p1=hac_point(entry.e+vf_e*handle,entry.n+vf_n*handle);
    candidate.p2=hac_point(exit.e-vh_e*handle,exit.n-vh_n*handle);

    double distance_along=candidate.lead_length;
    for(int i=0;i<=32;i++){
        double fraction=(double)i/32.0;
        double path_distance=distance_along+arc_length*fraction;
        double decay=fmax(0.0,speed_loss_accel)*path_distance/
            fmax(start_air_speed*start_air_speed,DBL_MIN);
        double speed=fmax(end_air_speed,start_air_speed*exp(-decay));
        double lateral=arc_length>DBL_MIN?speed*speed/hac_radius:0.0;
        candidate.peak_lateral=fmax(candidate.peak_lateral,lateral);
        double rate=arc_length>DBL_MIN?speed/hac_radius*RAD2DEG:0.0;
        double rate_cap=hac_course_rate_cap_for_speed(speed);
        candidate.peak_course_rate_ratio=fmax(
            candidate.peak_course_rate_ratio,
            rate/fmax(rate_cap,DBL_MIN));
        if(i==32)candidate.exit_speed=speed;
    }

    double total_path=candidate.lead_length+candidate.cone_arc_length;
    if(!(isfinite(total_path)&&total_path>DBL_MIN))return false;
    double path_violation=terminal_normalized_band_violation(
        total_path,min_fixed_path,max_fixed_path);
    double control_violation=terminal_normalized_upper_violation(
        candidate.peak_lateral,max_lateral_accel);
    double rate_violation=terminal_normalized_upper_violation(
        candidate.peak_course_rate_ratio,hac_transition_rate_ratio_limit());
    double lead_fit_violation=isfinite(candidate.violation_score)?
        candidate.violation_score:0.0;
    double violation=fmax(lead_fit_violation,
        fmax(path_violation,fmax(control_violation,rate_violation)));

    candidate.degraded_path=path_violation>0.0;
    candidate.degraded_control=control_violation>0.0;
    candidate.degraded_rate=rate_violation>0.0;
    candidate.degraded=candidate.degraded_end||
        candidate.degraded_path||candidate.degraded_control||
        candidate.degraded_rate;
    candidate.violation_score=violation;

    *out=candidate;
    return true;
}


/* Tangent course from (e,n) to the circle (center,radius) such that flying
   that course reaches the circle tangentially while turning toward `side`.
   Returns NAN when the point is inside the circle. */
static double hac_tangent_entry_course(double e,double n,HACPoint2 center,
        double radius,double side){
    double de=center.e-e,dn=center.n-n;
    double d2=de*de+dn*dn;
    if(!(d2>radius*radius*(1.0+1e-9)))return NAN;
    double along=sqrt(d2-radius*radius);
    double bearing=atan2(de,dn)*RAD2DEG;
    return norm_deg(bearing-atan2(side*radius,along)*RAD2DEG);
}

/* Two acquisition geometries are qualified with identical checks:
     1. entry where the circle tangent equals the response-projected course
        (flyable only when the aircraft already points along a tangent), and
     2. entry at the true tangent point from the projected position, which is
        the standard HAC acquisition: turn onto the tangent line, fly it, roll
        into the circle.
   Using only (1) put the entry laterally off the live track for any inlet
   that was not already tangent and forced an unflyable S-shaped bridge
   (300-1200 m/s^2 peaks in the HAC bearing sweep).  Keep the better of the
   two: flyable geometry first, then the smaller violation. */
       bool hac_transition_plan(HACTransitionPlan*out,double current_e,double current_n,
        double future_e,double future_n,double start_course,double future_course,
        const LandingSite*site,const GuidanceSettings*s,double hac_radius,double side,
        double start_air_speed,double end_air_speed,double speed_loss_accel,
        double max_lateral_accel,double preferred_fixed_path,
        double min_fixed_path,double max_fixed_path){
    HACTransitionPlan projected={0},tangent={0};
    bool have_projected=hac_transition_plan_entry(&projected,current_e,current_n,
        future_e,future_n,start_course,future_course,site,s,hac_radius,side,
        start_air_speed,end_air_speed,speed_loss_accel,max_lateral_accel,
        preferred_fixed_path,min_fixed_path,max_fixed_path,future_course);
    bool have_tangent=false;
    if(out&&site&&s&&hac_radius>0.0&&fabs(side)==1.0){
        double runway=site->runway_heading*DEG2RAD;
        HACPoint2 exit=hac_point(-sin(runway)*s->final_approach_distance,
            -cos(runway)*s->final_approach_distance);
        HACPoint2 center=hac_point(exit.e+side*hac_radius*cos(runway),
            exit.n-side*hac_radius*sin(runway));
        double tangent_course=hac_tangent_entry_course(future_e,future_n,
            center,hac_radius,side);
        if(isfinite(tangent_course)&&
           fabs(norm_signed_deg(tangent_course-future_course))>1e-6)
            have_tangent=hac_transition_plan_entry(&tangent,current_e,current_n,
                future_e,future_n,start_course,future_course,site,s,hac_radius,side,
                start_air_speed,end_air_speed,speed_loss_accel,max_lateral_accel,
                preferred_fixed_path,min_fixed_path,max_fixed_path,tangent_course);
    }
    if(!have_projected&&!have_tangent)return false;
    if(have_projected&&have_tangent){
        bool projected_geometry=projected.degraded_control||projected.degraded_rate||
            projected.degraded_end;
        bool tangent_geometry=tangent.degraded_control||tangent.degraded_rate||
            tangent.degraded_end;
        bool use_tangent=!tangent_geometry&&(projected_geometry||
             tangent.violation_score<projected.violation_score-1e-9);
        *out=use_tangent?tangent:projected;
    }else *out=have_projected?projected:tangent;
    return true;
}

       bool hac_variant_b_requested(void){
    const char*simulator=getenv("KSP_LANDER_SIMULATOR");
    const char*variant=getenv("KSP_LANDER_HAC_VARIANT_B");
    return simulator&&strcmp(simulator,"1")==0&&variant&&
        strcmp(variant,"1")==0;
}

/* Optional fly-and-learn preparation for the no-candidate state.  This is
 * deliberately separate from Variant-B selection: it holds a runway-parallel
 * staging line only while the normal planner is still searching.  It may not
 * create, accept, own, or account for any HAC geometry. */
       bool hac_upstream_runway_staging_requested(void){
    const char*simulator=getenv("KSP_LANDER_SIMULATOR");
    const char*staging=getenv("KSP_LANDER_HAC_UPSTREAM_RUNWAY_STAGING");
    return simulator&&strcmp(simulator,"1")==0&&staging&&
        strcmp(staging,"1")==0;
}

/* Fly-and-learn only: an explicit, bounded incidence increment for the
   provisional Variant-B energy-shaping experiment.  It is deliberately
   opt-in and is never an admission signal.  The normal AoA maximum and
   stabilized() response limits remain authoritative downstream. */
       double hac_upstream_aoa_bias_deg(void){
    const char *value=getenv("KSP_LANDER_HAC_UPSTREAM_AOA_BIAS_DEG");
    if(!value||!*value)return 0.0;
    char *end=NULL;
    double bias=strtod(value,&end);
    if(end==value||*end!='\0'||!isfinite(bias))return 0.0;
    /* The upper bound is still below the vehicle's normal maximum-AoA
       envelope after the existing 10-degree energy request is added.  This
       is only for the explicit maximum-shed experiment; the default remains
       zero and no production gate consumes this value. */
    return clampd(bias,0.0,12.0);
}

/* Diagnostic-only lead energy experiment.  This never participates in
   admission and defaults to zero; the ordinary lead request, AoA envelope,
   and stabilized() response limits remain unchanged unless an explicit
   fly-and-learn run opts in. */
       double hac_lead_aoa_bias_deg(void){
    const char *value=getenv("KSP_LANDER_HAC_LEAD_AOA_BIAS_DEG");
    if(!value||!*value)return 0.0;
    char *end=NULL;
    double bias=strtod(value,&end);
    if(end==value||*end!='\0'||!isfinite(bias))return 0.0;
    return clampd(bias,0.0,12.0);
}

/* ShuttleSim MM305 fixtures exercise the production candidate contract at
 * the configured runway-anchored HAC radius and the explicit 3.5 km
 * runway-alignment station. The sustainable radius is an admission diagnostic
 * only; this selector never changes the committed circle. */
       bool hac_mm305_dynamic_fixture_requested(void){
    const char*simulator=getenv("KSP_LANDER_SIMULATOR");
    const char*fixture=getenv("KSP_LANDER_FORCE_REENTRY_TEST");
    const char*identity=getenv("KSP_LANDER_MM305_FIXED_HAC");
    return simulator&&strcmp(simulator,"1")==0&&fixture&&
        strcmp(fixture,"1")==0&&identity&&strcmp(identity,"1")==0;
}

       bool hac_fixed_alignment_geometry(HACPoint2*entry_out,
        HACPoint2*center_out,HACPoint2*exit_out,const LandingSite*site,
        const GuidanceSettings*s,double hac_radius,double capture_course,
        double side){
    if(!entry_out||!center_out||!exit_out||!site||!s||
       !(hac_radius>0.0)||fabs(side)!=1.0||!isfinite(capture_course))
        return false;

    double runway=site->runway_heading*DEG2RAD;
    double capture=capture_course*DEG2RAD;
    double vh_e=sin(runway),vh_n=cos(runway);
    double rh_e=cos(runway),rh_n=-sin(runway);
    double rf_e=cos(capture),rf_n=-sin(capture);
    HACPoint2 runway_station=hac_point(-vh_e*s->final_approach_distance,
        -vh_n*s->final_approach_distance);
    HACPoint2 exit;
    if((hac_variant_b_requested()||hac_mm305_dynamic_fixture_requested())&&side>0.0){
        /* MM305 ShuttleSim fixtures use an exit 3.5 km behind the runway
           threshold; the selected dynamic circle's upper tip remains on
           runway latitude. The upstream staging distance is not part of the
           committed path. */
        double exit_along=-3500.0;
        exit=hac_point(exit_along*vh_e,exit_along*vh_n);
    }else{
        /* The production/default HAC exits exactly at the configured final-
           approach station.  The circle center moves to the turn side of that
           fixed exit; never translate the exit by one radius along the runway. */
        exit=runway_station;
    }
    HACPoint2 center=hac_point(exit.e+side*hac_radius*rh_e,
        exit.n+side*hac_radius*rh_n);
    HACPoint2 entry=hac_point(center.e-side*hac_radius*rf_e,
        center.n-side*hac_radius*rf_n);
    *entry_out=entry;*center_out=center;*exit_out=exit;
    return true;
}

/* Build the mission-fixed MM305 HAC from the runway outer-final station and
   the shuttle's inlet vector.  The exit is frozen at that station; the center
   is one radius to the selected turn side and the tangent entry follows from
   the inlet course.  For RW09/side +1 this puts the center south of the runway
   and makes the exit the circle's exact northern tip.  A finite Hermite lead
   is the only acquisition geometry permitted when the live MM305 state is not
   already tangent to the frozen circle. */
       bool hac_fixed_alignment_plan(HACTransitionPlan*out,
        double current_e,double current_n,double start_course,double capture_course,
        const LandingSite*site,const GuidanceSettings*s,double hac_radius,double side){
    if(!out||!site||!s||!(hac_radius>0.0)||fabs(side)!=1.0||
       !isfinite(current_e)||!isfinite(current_n)||
       !isfinite(start_course)||!isfinite(capture_course))return false;
    memset(out,0,sizeof(*out));

    double runway=site->runway_heading*DEG2RAD;
    double capture=capture_course*DEG2RAD;
    double vh_e=sin(runway),vh_n=cos(runway);
    double vf_e=sin(capture),vf_n=cos(capture);

    HACPoint2 exit={0},center={0},entry={0};
    if(!hac_fixed_alignment_geometry(&entry,&center,&exit,site,s,hac_radius,
            capture_course,side))return false;

    /* Preserve the selected bank/course-rate side all the way to the runway
       tangent.  A southbound RW09 inlet is side +1, so heading increases
       180 -> 190 -> ... until the anchored circle reaches runway heading.
       This may be the long partial arc; it is still one finite HAC, never a
       resized circle or an added circuit. */
    double arc_angle_deg=norm_deg(side*(site->runway_heading-capture_course));
    double angular_epsilon=sqrt(DBL_EPSILON)*RAD2DEG;
    if(arc_angle_deg<=angular_epsilon)
        return false;
    double arc_angle=arc_angle_deg*DEG2RAD;
    double start_angle=atan2(entry.n-center.n,entry.e-center.e);
    double end_angle=start_angle-side*arc_angle;
    double arc_length=arc_angle*hac_radius;

    HACTransitionPlan candidate={0};
    candidate.valid=true;
    candidate.heading_cone=true;
    candidate.lead_start=hac_point(current_e,current_n);
    candidate.p0=entry;
    candidate.p3=exit;
    candidate.cone_center=center;
    candidate.cone_start_angle=start_angle;
    candidate.cone_end_angle=end_angle;
    candidate.cone_arc_length=arc_length;
    candidate.lead_start_course=norm_deg(start_course);
    candidate.lead_end_course=norm_deg(capture_course);
    candidate.length=arc_length;
    candidate.end_angle=end_angle;
    candidate.arc_remaining=0.0;

    double lead_chord=hypot(entry.e-current_e,entry.n-current_n);
    double position_tolerance=fmax(1.0,hac_radius*1e-6);
    double course_error=fabs(norm_signed_deg(start_course-capture_course));
    /* At p0 there is no chord to build a lead from, so a small residual
       course error (sampling/frame rounding at the fixture handoff) is left
       to the arc tracking law rather than rejecting the circle outright. */
    /* A chord of a few metres (handoff sampling, one guidance tick short of
       p0) cannot carry a meaningful lead: the Bezier through it degenerates to
       sub-metre radii and fails authority at any speed.  Within a small
       fraction of R the arc tracking law absorbs the offset directly. */
    double capture_tolerance=fmax(position_tolerance,fmin(hac_radius*.02,250.0));
    bool already_tangent=lead_chord<=capture_tolerance&&course_error<=2.0;
    if(!already_tangent){
        if(lead_chord<=position_tolerance)return false;
        /* Fixed chord/3 handles put a tight kink near u=0 whenever the entry
           is laterally offset from the live track or the heading change is
           large (the lead-authority rejects peaked at u<0.1 in >80% of runs).
           Choose the start/end handle pair that minimizes peak curvature; the
           authority certificate downstream still judges the chosen curve. */
        static const double handle_scales[]={1.0/3.0,0.5,0.75,1.0,1.25};
        const size_t handle_count=sizeof(handle_scales)/sizeof(handle_scales[0]);
        double best_peak=INFINITY;
        HACPoint2 best_p1={0},best_p2={0};
        candidate.lead_curve=true;
        for(size_t a=0;a<handle_count;a++)for(size_t b=0;b<handle_count;b++){
            double h1=lead_chord*handle_scales[a],h2=lead_chord*handle_scales[b];
            candidate.lead_p1=hac_point(current_e+sin(start_course*DEG2RAD)*h1,
                current_n+cos(start_course*DEG2RAD)*h1);
            candidate.lead_p2=hac_point(entry.e-sin(capture_course*DEG2RAD)*h2,
                entry.n-cos(capture_course*DEG2RAD)*h2);
            HACTransitionPlan trial=hac_lead_bezier_from_plan(&candidate);
            if(!hac_bezier_regular(&trial))continue;
            if(hac_bezier_length_between(&trial,0.0,1.0,32)>lead_chord*1.25)continue;
            double peak=0.0;
            for(int k=0;k<=64;k++){
                double kappa=fabs(hac_bezier_signed_curvature(&trial,(double)k/64.0));
                if(!isfinite(kappa)){peak=INFINITY;break;}
                peak=fmax(peak,kappa);
            }
            if(peak<best_peak){best_peak=peak;best_p1=candidate.lead_p1;best_p2=candidate.lead_p2;}
        }
        if(!isfinite(best_peak))return false;
        candidate.lead_p1=best_p1;candidate.lead_p2=best_p2;
        HACTransitionPlan lead=hac_lead_bezier_from_plan(&candidate);
        if(!hac_bezier_regular(&lead))return false;
        candidate.lead_length=hac_bezier_length_between(&lead,0.0,1.0,32);
        if(!(candidate.lead_length>0.0)||!isfinite(candidate.lead_length))return false;
    }else{
        candidate.lead_curve=false;
        candidate.lead_p1=candidate.lead_start;
        candidate.lead_p2=entry;
        candidate.lead_length=0.0;
    }

    /* p1/p2 describe the analytic cone only for visual/debug compatibility.
       Guidance follows cone_center/start/end directly, never this cubic. */
    double arc_handle=fmax(100.0,
        4.0*hac_radius*sin(fmin(arc_angle*.25,LANDER_PI*.25))/3.0);
    candidate.p1=hac_point(entry.e+vf_e*arc_handle,
        entry.n+vf_n*arc_handle);
    candidate.p2=hac_point(exit.e-vh_e*arc_handle,
        exit.n-vh_n*arc_handle);

    *out=candidate;
    return true;
}

