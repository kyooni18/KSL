       HACGuidance hac_guidance_radius(const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity,double course,double hac_radius){
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    return hac_guidance_compute_radius(current,t->true_air_speed,course,site,s,radius,side,gravity,hac_radius);
}

                
                         
                         
               
                       
                   

/* Heading Alignment owns the frozen circle itself.  Keep this separate from
   Acquisition so a stale lead/cubic cannot accidentally remain the active
   reference after the executive declares HAC capture. */
static HACGuidance hac_circle_path_guidance(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double gravity,
        double course,double hac_radius){
    HACGuidance out=hac_guidance_radius(t,site,s,planet_radius,side,gravity,course,hac_radius);
    hac_radius=fmax(1000.0,hac_radius);
    double h=site->runway_heading*DEG2RAD;
    double ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h);
    double fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance;
    double ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius;
    double orient=-side;
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e,n;local_offsets(sp,current,planet_radius,&e,&n);
    double dx=e-ce,dy=n-cn,distance=hypot(dx,dy);
    double ca=atan2(dy,dx);
    double radial=distance-hac_radius;
    double tangent_e=-orient*sin(ca),tangent_n=orient*cos(ca);
    double target_heading=norm_deg(atan2(tangent_e,tangent_n)*RAD2DEG);
    /* HAC curvature is ground-track geometry.  Use horizontal ground speed here,
       matching the committed heading-cone tracker, rather than total airspeed;
       vertical velocity must not inflate the lateral curvature command. */
    double speed=fmax(1.0,guidance_lateral_speed(t));
    double course_error=norm_signed_deg(target_heading-course);
    double cross_track=-side*radial;
    double tracking_time=hac_path_tracking_time(speed,hac_radius);
    double lateral=hac_frenet_lateral(speed,side/hac_radius,
        clampd(cross_track,-fmax(2500.0,hac_radius*.30),fmax(2500.0,hac_radius*.30)),
        course_error,tracking_time);
    out.heading=target_heading;
    out.course_error=course_error;
    out.lateral_acceleration=lateral;
    out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70);
    return out;
}

/* A circle is only one way to arrive at outer final.  For a committed direct
   spline, the cubic ends on the runway-aligned outer-final station and this
   line law owns the small residual after the cubic is exhausted.  It is an
   L1/Frenet-style capture, not a waypoint chase: cross-track and course error
   are converted directly to lateral acceleration with a speed-scaled look
   distance. */
       HACGuidance terminal_runway_line_path_guidance(const Telemetry*t,
        const LandingSite*site,const GuidanceSettings*s,double gravity,double course){
    HACGuidance out;memset(&out,0,sizeof(out));
    double speed=fmax(1.0,guidance_lateral_speed(t));
    /* A runway-line law owns the runway line only.  MM304 acquisition geometry
       comes from TaemInterfaceTarget; hiding another staging station here created
       a second lateral program that could disagree with the handoff contract. */
    double station=-s->final_approach_distance;
    double station_gap=fmax(0.0,station-t->runway_along_track);
    double look=clampd(fmax(speed*5.5,station_gap*.45),700.0,5200.0);
    double intercept=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-24.0,24.0);
    out.heading=norm_deg(site->runway_heading+intercept);
    out.course_error=norm_signed_deg(out.heading-course);
    double lateral=2.0*speed*speed/fmax(look,1.0)*sin(out.course_error*DEG2RAD);
    double rate_cap=fmin(1.35,hac_course_rate_cap(t))*DEG2RAD;
    lateral=clampd(lateral,-speed*rate_cap,speed*rate_cap);
    out.lateral_acceleration=lateral;
    out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-45.0,45.0);
    out.radial_error=t->runway_cross_track;
    out.arc_remaining=station_gap;
    out.distance_final=fmax(0.0,-t->runway_along_track);
    /* Before the outer-final station, distance_final is exactly
       final_approach_distance + station_gap. After the station it keeps
       shrinking toward the threshold instead of freezing at the HAC exit. */
    out.desired_altitude=site->altitude+
        out.distance_final*tan(s->taem_glide_slope*DEG2RAD);
    return out;
}

       HACGuidance hac_heading_cone_path_guidance(const GuidanceMachine*g,
        const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,
        double planet_radius,double gravity,double course){
    HACGuidance out;memset(&out,0,sizeof(out));
    if(!g||!t||!site||!s||!g->hac_transition_heading_cone)return out;
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e=0.0,n=0.0;local_offsets(origin,current,planet_radius,&e,&n);
    double radius=fmax(1000.0,g->hac_radius);
    double total=clampd(g->hac_transition_cone_arc_length/radius,0.0,2.0*LANDER_PI);
    double distance=hypot(e-g->hac_transition_cone_center_e,
        n-g->hac_transition_cone_center_n);
    double radial=distance-radius;
    double endpoint_distance=hypot(e-g->hac_transition_p3_e,
        n-g->hac_transition_p3_n);
    /* Angular station is useful for tracking, but it is not proof that the
       vehicle traversed the committed circle.  In particular, an off-circle
       point can lie in the p3 half-plane and otherwise trigger the old p0/p3
       fallback, falsely producing progress=1/path_remaining=0.  Only permit
       that saturation in the frozen endpoint capture set; otherwise retain
       the already measured station and keep the path live. */
    bool endpoint_geometry_valid=fabs(radial)<=fmax(250.0,radius*.05)&&
        endpoint_distance<=fmax(260.0,t->true_air_speed*2.8)&&
        fabs(norm_signed_deg(course-site->runway_heading))<=22.0;
    double raw=atan2(n-g->hac_transition_cone_center_n,
        e-g->hac_transition_cone_center_e);
    double progress=fmod(-g->hac_side*(raw-g->hac_transition_cone_start_angle),
        2.0*LANDER_PI);
    if(progress<0.0)progress+=2.0*LANDER_PI;
    if(progress>=total-fmax(1e-6,0.5*DEG2RAD)&&!endpoint_geometry_valid){
        /* Do not use endpoint proximity or a chord-side test to manufacture
           completion for a 270-degree major arc.  The committed progress is
           normalized, so this preserves the last valid station without
           redrawing or recapturing the HAC. */
        progress=clampd(g->hac_transition_progress,0.0,1.0)*total;
    }
    double committed_progress=clampd(g->hac_transition_progress,0.0,1.0)*total;
    /* TAEM acquisition and heading alignment are distinct states.  Angular
       bearing around the cone is not proof that the vehicle has flown HAC arc:
       a vehicle displaced well outside the circle can sweep a large polar angle
       while merely passing the HAC.  Hold the last committed HAC station until
       it is back inside the same radial capture tube used by the fixed-HAC
       lead/endpoint contract.  This keeps remaining range, vertical reference,
       and energy scheduling tied to flown path instead of apparent angle. */
    double radial_tracking_limit=fmax(250.0,radius*.05);
    bool circle_reacquisition=radial>radial_tracking_limit;
    if(circle_reacquisition)progress=committed_progress;
    if(total>0.0&&isfinite(g->hac_transition_lead_start_course)){
        double heading_progress=norm_deg(g->hac_side*
            norm_signed_deg(course-g->hac_transition_lead_start_course))*DEG2RAD;
        double progress_cap=fmin(total,heading_progress+15.0*DEG2RAD);
        progress=fmin(progress,progress_cap);
    }
    /* Once execution has accepted a farther station, neither angular wrap nor
       the heading anti-teleport cap may rewind the committed HAC. Rewinding
       here makes arc_remaining grow again and raises the frozen vertical
       reference after the vehicle has already flown that altitude away. */
    progress=fmax(progress,committed_progress);
    double angle=g->hac_transition_cone_start_angle-g->hac_side*progress;
    /* Frenet cross-track is positive RIGHT of the tangent and its feedback
       term is subtracted. Outside a clockwise (+side) circle is LEFT of the
       tangent, so signed cross-track is -side*radial. This does not change
       bank/heading handedness: outward error must increase the selected turn. */
    double cross_track=-g->hac_side*radial;
    double tangent=norm_deg(atan2(g->hac_side*sin(angle),
        -g->hac_side*cos(angle))*RAD2DEG);
    double target_heading=tangent;
    /* NASA TAEM keeps acquisition separate from heading alignment: while the
       vehicle is outside the HAC tracking tube, steer to the true circle
       tangency from the live position and do not apply circle curvature yet.
       The committed HAC itself remains frozen; this is only a recovery law. */
    if(circle_reacquisition){
        HACPoint2 center={g->hac_transition_cone_center_e,
            g->hac_transition_cone_center_n};
        double acquisition_heading=hac_tangent_entry_course(e,n,center,
            radius,g->hac_side);
        if(isfinite(acquisition_heading))target_heading=acquisition_heading;
    }
    double course_error=norm_signed_deg(target_heading-course);
    double speed=fmax(1.0,guidance_lateral_speed(t));
    double tracking_time=hac_path_tracking_time(speed,radius);
    double turn_sign=g->hac_side;
    double control_curvature=circle_reacquisition?0.0:turn_sign/radius;
    double control_cross_track=circle_reacquisition?0.0:
        clampd(cross_track,-fmax(1200.0,radius*.25),fmax(1200.0,radius*.25));
    double lateral_raw=hac_frenet_lateral(speed,control_curvature,
        control_cross_track,course_error,tracking_time);
    double rate_limit=hac_join_rate_cap(t,total>0.0?progress/total:1.0)*DEG2RAD;
    double lateral_limit=speed*rate_limit;
    double progress_fraction=total>0.0?progress/total:1.0;
    double body_curve=speed*speed/fmax(radius,1.0);
    double effective_limit=lateral_limit;
    if(g->fixed_alignment_hac_latched)
        effective_limit=fmax(effective_limit,body_curve*1.05);
    double lateral=clampd(lateral_raw,-effective_limit,effective_limit);
    if(!circle_reacquisition&&progress_fraction<0.92){
        /* Once outside the frozen circle, less than V^2/R cannot arrest
           radial growth even with perfect bank tracking.  The generic join
           minimum was only 45% of circle curvature and could keep a
           same-sign but geometrically divergent turn alive indefinitely. */
        double minimum_turn=(g->fixed_alignment_hac_latched?
            fmin(effective_limit,body_curve*(radial>0.0?1.0:0.45)):
            fmin(lateral_limit,body_curve*0.45));
        if(lateral*turn_sign<=0.0)
            lateral=turn_sign*fmin(effective_limit,
                fmax(fabs(lateral)*0.25,minimum_turn));
        else if(fabs(lateral)<minimum_turn)
            lateral=turn_sign*minimum_turn;
    }
    out.heading=target_heading;out.course_error=course_error;
    out.lateral_acceleration=lateral;
    out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70);
    out.radial_error=radial;out.angle=angle;
    out.arc_remaining=fmax(0.0,(total-progress)*radius);
    out.distance_final=fmax(0.0,-t->runway_along_track);
    double final_alt=s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);
    out.desired_altitude=site->altitude+final_alt+
        out.arc_remaining*tan(s->taem_glide_slope*DEG2RAD);
    return out;
}

/* Track the committed join+HAC as a continuous geometric trajectory.  The
   aircraft is projected onto the closest path station every frame.  Bank is
   generated from the station's signed curvature plus damped Frenet
   cross-track/course feedback; no waypoint or heading-pursuit term owns the
   turn. targetHeading is only the local path tangent used for diagnostics and
   the existing reference-rate monitor. */
       HACGuidance hac_path_guidance(const GuidanceMachine*g,const Telemetry*t,
        const LandingSite*site,const GuidanceSettings*s,double planet_radius,
        double side,double gravity,double course,double hac_radius){
    bool spline=g&&g->terminal_path_kind==TERMINAL_PATH_SPLINE;
    HACGuidance out=spline?terminal_runway_line_path_guidance(t,site,s,gravity,course):
        hac_guidance_radius(t,site,s,planet_radius,side,gravity,course,hac_radius);
    if(!g||!g->hac_side_selected)return out;
    if(!g->hac_transition_active&&(!isfinite(g->hac_remaining)||g->hac_remaining<=0))return out;

    hac_radius=fmax(1000.0,hac_radius);
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e,n;local_offsets(sp,current,planet_radius,&e,&n);

    if(g->hac_transition_active&&g->hac_transition_length>1.0){
        HACTransitionPlan p={.valid=true,
            .p0={g->hac_transition_p0_e,g->hac_transition_p0_n},.p1={g->hac_transition_p1_e,g->hac_transition_p1_n},
            .p2={g->hac_transition_p2_e,g->hac_transition_p2_n},.p3={g->hac_transition_p3_e,g->hac_transition_p3_n},
            .length=g->hac_transition_length,.end_angle=g->hac_transition_end_angle};
        double speed=fmax(1.0,guidance_lateral_speed(t));
        if(g->hac_transition_lead_length>1.0&&g->hac_transition_lead_progress<.995){
            HACTransitionPlan lead={0};
            lead.valid=true;
            lead.lead_curve=g->hac_transition_lead_curve;
            lead.lead_acquisition=g->hac_transition_lead_acquisition;
            lead.lead_start=(HACPoint2){g->hac_transition_lead_start_e,
                g->hac_transition_lead_start_n};
            lead.lead_p1=(HACPoint2){g->hac_transition_lead_p1_e,
                g->hac_transition_lead_p1_n};
            lead.lead_p2=(HACPoint2){g->hac_transition_lead_p2_e,
                g->hac_transition_lead_p2_n};
            lead.p0=p.p0;lead.p3=p.p0;
            lead.lead_length=g->hac_transition_lead_length;
            lead.lead_start_course=g->hac_transition_lead_start_course;
            lead.lead_end_course=g->hac_transition_lead_end_course;
            lead.acquisition_center=(HACPoint2){g->hac_transition_acquisition_center_e,
                g->hac_transition_acquisition_center_n};
            lead.acquisition_tangent=(HACPoint2){g->hac_transition_acquisition_tangent_e,
                g->hac_transition_acquisition_tangent_n};
            lead.acquisition_radius=g->hac_transition_acquisition_radius;
            lead.acquisition_side=g->hac_transition_acquisition_side;
            lead.acquisition_start_angle=g->hac_transition_acquisition_start_angle;
            lead.acquisition_end_angle=g->hac_transition_acquisition_end_angle;
            lead.acquisition_arc_length=g->hac_transition_acquisition_arc_length;
            lead.acquisition_tangent_length=g->hac_transition_acquisition_tangent_length;

            double station=hac_lead_nearest_distance(&lead,e,n);
            if(station<lead.lead_length){
                HACPoint2 point={0};double tangent=NAN,station_curvature=NAN;
                if(!hac_lead_sample_distance(&lead,station,&point,&tangent,
                        &station_curvature))return out;
                double h=tangent*DEG2RAD;
                double cross_track=(e-point.e)*cos(h)+(n-point.n)*(-sin(h));
                double course_error=norm_signed_deg(tangent-course);
                double preview_distance=fmin(lead.lead_length,station+
                    speed*clampd(g->hac_transition_response_time*.65,1.3,4.2));
                HACPoint2 preview_point={0};double preview_course=NAN,curvature=NAN;
                if(!hac_lead_sample_distance(&lead,preview_distance,&preview_point,
                        &preview_course,&curvature))return out;
                double effective_radius=1.0/fmax(fabs(curvature),
                    1.0/(hac_radius*2.5));
                double tracking_time=hac_path_tracking_time(speed,effective_radius);
                double lateral=hac_frenet_lateral(speed,curvature,
                    clampd(cross_track,-fmax(1200.0,hac_radius*.25),
                        fmax(1200.0,hac_radius*.25)),
                    course_error,tracking_time);
                double progress=clampd(station/fmax(lead.lead_length,DBL_MIN),0.0,1.0);
                double lateral_limit=speed*hac_join_rate_cap(t,progress)*DEG2RAD;
                lateral=clampd(lateral,-lateral_limit,lateral_limit);
                out.heading=tangent;
                out.course_error=course_error;
                out.arc_remaining=g->hac_remaining+g->hac_transition_length+
                    fmax(0.0,lead.lead_length-station);
                out.radial_error=cross_track;
                out.lateral_acceleration=lateral;
                out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70);
                return out;
            }
        }
        if(g->hac_transition_heading_cone)
            return hac_heading_cone_path_guidance(g,t,site,s,planet_radius,
                gravity,course);
        double u=hac_transition_nearest_u(g,e,n);
        HACPoint2 point=hac_bezier_point(&p,u),d=hac_bezier_derivative(&p,u);
        double derivative=fmax(hypot(d.e,d.n),1.0);
        double tangent=norm_deg(atan2(d.e,d.n)*RAD2DEG);
        double cross_track=((e-point.e)*d.n-(n-point.n)*d.e)/derivative;
        double course_error=norm_signed_deg(tangent-course);
        /* Feed the roll loop the curvature that will reach the airframe after
           its finite response delay, while keeping position/course feedback
           tied to the actual closest station.  This is a curvature preview,
           not a look-ahead waypoint. */
        double preview_time=clampd(g->hac_transition_response_time*.65,1.3,4.2);
        double preview_u=hac_bezier_advance_distance(&p,u,speed*preview_time);
        double curvature=hac_bezier_signed_curvature(&p,preview_u);
        double effective_radius=1.0/fmax(fabs(curvature),1.0/(hac_radius*2.5));
        double tracking_time=hac_path_tracking_time(speed,effective_radius);
        double lateral=hac_frenet_lateral(speed,curvature,
            clampd(cross_track,-fmax(1200.0,hac_radius*.25),fmax(1200.0,hac_radius*.25)),
            course_error,tracking_time);
        /* The path itself can contain only the already-qualified transition
           curvature envelope.  Saturating the correction in course-rate
           space preserves that contract without converting the error back
           into a heading pursuit law. */
        double course_rate_limit=hac_join_rate_cap(t,fmax(u,g->hac_transition_progress))*DEG2RAD;
        double lateral_limit=speed*course_rate_limit;
        lateral=clampd(lateral,-lateral_limit,lateral_limit);
        out.heading=tangent;
        out.course_error=course_error;
        out.arc_remaining=g->hac_remaining+hac_bezier_length_between(&p,u,1.0,16);
        out.lateral_acceleration=lateral;
        out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70);
        return out;
    }

    if(spline)return terminal_runway_line_path_guidance(t,site,s,gravity,course);
    return hac_circle_path_guidance(t,site,s,planet_radius,side,gravity,course,hac_radius);
}

static double hac_prefinal_rollout_length(const Telemetry*t,const GuidanceSettings*s){
    double upper=fmax(1200.0,s->final_approach_distance*.35);
    return clampd(fmax(1200.0,guidance_lateral_speed(t)*6.0),1200.0,upper);
}

/* Prefinal is a lateral rollout from the frozen HAC tangent onto runway
   centerline.  At u=0 the command is exactly the Heading-Alignment circle
   command; smoothstep then removes HAC curvature while introducing the runway
   Frenet correction.  This makes the path C0 in lateral acceleration at the
   handoff and gives zero blend slope at both ends. */
static HACGuidance hac_prefinal_path_guidance(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double gravity,
        double course,double hac_radius,double*out_progress){
    HACGuidance out=hac_circle_path_guidance(t,site,s,planet_radius,side,gravity,course,hac_radius);
    double speed=fmax(1.0,t->true_air_speed);
    double station=-s->final_approach_distance;
    double rollout=hac_prefinal_rollout_length(t,s);
    double u=clampd((t->runway_along_track-station)/fmax(rollout,1.0),0.0,1.0);
    double blend=u*u*(3.0-2.0*u);
    double runway_heading=norm_deg(site->runway_heading);
    double line_error=norm_signed_deg(runway_heading-course);
    double line_time=hac_path_tracking_time(speed,fmax(6000.0,rollout*2.5));
    double cross_limit=fmax(500.0,s->final_approach_distance*.12);
    double line_lateral=hac_frenet_lateral(speed,0.0,
        clampd(t->runway_cross_track,-cross_limit,cross_limit),line_error,line_time);
    double lateral=(1.0-blend)*out.lateral_acceleration+blend*line_lateral;
    double end_rate=fmin(1.20,hac_course_rate_cap(t));
    double rate_cap=(1.0-blend)*hac_course_rate_cap(t)+blend*end_rate;
    double lateral_cap=speed*rate_cap*DEG2RAD;
    lateral=clampd(lateral,-lateral_cap,lateral_cap);
    out.heading=norm_deg(out.heading+blend*norm_signed_deg(runway_heading-out.heading));
    out.course_error=norm_signed_deg(out.heading-course);
    out.lateral_acceleration=lateral;
    out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70);
    out.arc_remaining=(1.0-u)*rollout;
    if(out_progress)*out_progress=u;
    return out;
}

/* Path generation consumes the public MM305 phase but never changes it.  The
   executive owns every transition; this function only returns a phase-specific
   geometric command plus the corresponding geometric handoff indication. */
       HACPhaseGuidance hac_phase_guidance(TaemPhase phase,const GuidanceMachine*g,
        const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,double planet_radius,
        double side,double gravity,double course,double hac_radius){
    HACPhaseGuidance out;memset(&out,0,sizeof(out));
    out.valid=true;
    if(phase==TAEM_PHASE_PATH_ACQUISITION){
        out.guidance=hac_path_guidance(g,t,site,s,planet_radius,side,gravity,course,hac_radius);
        if(g&&g->hac_transition_active){
            double lead_done=g->hac_transition_lead_length<=1.0?1.0:g->hac_transition_lead_progress;
            out.path_progress=.20*clampd(lead_done,0.0,1.0)+.80*clampd(g->hac_transition_progress,0.0,1.0);
        }else out.path_progress=g&&g->hac_captured?1.0:0.0;
        out.handoff_ready=g&&!g->hac_transition_active&&g->hac_captured;
        return out;
    }
    if(phase==TAEM_PHASE_RUNWAY_ALIGNMENT){
        bool active_heading_cone=g&&g->terminal_path_kind==TERMINAL_PATH_HAC&&
            g->hac_transition_heading_cone&&g->hac_transition_active;
        bool runway_line=g&&(g->terminal_path_kind==TERMINAL_PATH_SPLINE||
            (g->hac_transition_heading_cone&&!g->hac_transition_active));
        if(active_heading_cone)
            out.guidance=hac_path_guidance(g,t,site,s,planet_radius,side,gravity,course,hac_radius);
        else out.guidance=runway_line?
            terminal_runway_line_path_guidance(t,site,s,gravity,course):
            hac_circle_path_guidance(t,site,s,planet_radius,side,gravity,course,hac_radius);
        out.path_progress=g&&g->hac_captured?1.0:0.0;
        out.handoff_ready=g&&g->hac_completed;
        return out;
    }
    if(phase!=TAEM_PHASE_FINAL_INTERCEPT){
        out.valid=false;
        return out;
    }
    if(g&&(g->terminal_path_kind==TERMINAL_PATH_SPLINE||
            g->hac_transition_heading_cone)){
        out.guidance=terminal_runway_line_path_guidance(t,site,s,gravity,course);
        double station=-s->final_approach_distance;
        double rollout=fmax(1200.0,s->final_approach_distance*.35);
        out.path_progress=clampd((t->runway_along_track-(station-rollout))/rollout,0.0,1.0);
    }else out.guidance=hac_prefinal_path_guidance(t,site,s,planet_radius,side,gravity,course,hac_radius,
        &out.path_progress);
    double cross_limit=fmax(80.0,fmin(220.0,s->final_approach_distance*.025));
    out.handoff_ready=out.path_progress>=.995&&t->runway_along_track<0.0&&
        fabs(t->runway_cross_track)<=cross_limit&&
        fabs(norm_signed_deg(site->runway_heading-course))<=8.0;
    return out;
}
       void reference_trajectory_radius(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double hac_radius){trajectory_clear(out);hac_radius=fmax(1000,hac_radius);GeoPoint sp={site->latitude,site->longitude,site->altitude};double h=site->runway_heading*DEG2RAD,ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h),fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance,ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius,fa=atan2(fn-cn,fe-ce),orient=-side,extent=2*LANDER_PI;for(int i=0;i<=72;i++){double f=(double)i/72,a=fa-orient*extent*(1-f),e=ce+cos(a)*hac_radius,n=cn+sin(a)*hac_radius,arc=hac_radius*extent*(1-f),alt=site->altitude+s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD)+arc*tan(s->taem_glide_slope*DEG2RAD);GeoPoint p=local_point(sp,e,n,radius,alt);TrajectoryPoint tp={0,p.latitude,p.longitude,alt,0,PHASE_TAEM,TRAJ_REFERENCE};trajectory_append(out,tp);}for(int i=1;i<=40;i++){double f=(double)i/40,d=s->final_approach_distance*(1-f),e=-ae*d,n=-an*d,alt=site->altitude+d*tan(s->final_glide_slope*DEG2RAD);GeoPoint p=local_point(sp,e,n,radius,alt);TrajectoryPoint tp={0,p.latitude,p.longitude,alt,0,PHASE_FINAL,TRAJ_REFERENCE};trajectory_append(out,tp);}}

