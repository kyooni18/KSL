       void reference_trajectory_fixed_hac(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double planet_radius,const GuidanceMachine*g){
    trajectory_clear(out);
    if(!g||!g->fixed_alignment_hac_latched||!g->hac_transition_heading_cone||
       !g->fixed_hac_reference_valid||
       !isfinite(g->hac_radius)||g->hac_radius<=0.0||
       !isfinite(g->hac_transition_cone_arc_length)||
       g->hac_transition_cone_arc_length<=0.0)return;

    (void)s;
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    double arc_total=g->hac_transition_cone_arc_length/g->hac_radius;
    bool lead_active=g->hac_transition_active&&g->hac_transition_lead_length>1.0&&
        g->hac_transition_lead_progress<.995;
    double circle_progress=lead_active?0.0:
        clampd(g->hac_transition_progress,0.0,1.0);
    double circle_start=circle_progress*arc_total;
    double circle_remaining=fmax(0.0,(arc_total-circle_start)*g->hac_radius);

    if(lead_active){
        HACTransitionPlan lead={0};
        lead.valid=true;
        lead.lead_curve=g->hac_transition_lead_curve;
        lead.lead_acquisition=g->hac_transition_lead_acquisition;
        lead.lead_start=(HACPoint2){g->hac_transition_lead_start_e,g->hac_transition_lead_start_n};
        lead.p0=(HACPoint2){g->hac_transition_p0_e,g->hac_transition_p0_n};
        lead.p3=lead.p0;
        lead.lead_p1=(HACPoint2){g->hac_transition_lead_p1_e,g->hac_transition_lead_p1_n};
        lead.lead_p2=(HACPoint2){g->hac_transition_lead_p2_e,g->hac_transition_lead_p2_n};
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
        /* The published reference must retain the complete finite C1 lead
           from the shuttle's committed initial vector.  Lead progress is
           execution state, not permission to erase the vector-origin part
           of the plan; doing so makes the HAC appear detached once the
           vehicle has advanced along the bridge. */
        const int lead_samples=48;
        for(int i=0;i<=lead_samples;i++){
            double station=g->hac_transition_lead_length*
                (double)i/(double)lead_samples;
            HACPoint2 q={0};double sample_course=NAN,sample_curvature=NAN;
            if(!hac_lead_sample_distance(&lead,station,&q,&sample_course,
                    &sample_curvature))continue;
            double lead_remaining=fmax(0.0,
                g->hac_transition_lead_length-station);
            double alt=fixed_hac_reference_altitude(g,
                lead_remaining+circle_remaining);
            GeoPoint point=local_point(origin,q.e,q.n,planet_radius,alt);
            TrajectoryPoint tp={0,point.latitude,point.longitude,alt,0,PHASE_TAEM,TRAJ_REFERENCE};
            trajectory_append(out,tp);
        }
    }

    const int circle_samples=72;
    int first=lead_active?1:0;
    for(int i=first;i<=circle_samples;i++){
        double f=(double)i/(double)circle_samples;
        double travelled=circle_start+(arc_total-circle_start)*f;
        double a=g->hac_transition_cone_start_angle-g->hac_side*travelled;
        double e=g->hac_transition_cone_center_e+cos(a)*g->hac_radius;
        double n=g->hac_transition_cone_center_n+sin(a)*g->hac_radius;
        double remaining=fmax(0.0,(arc_total-travelled)*g->hac_radius);
        double alt=fixed_hac_reference_altitude(g,remaining);
        GeoPoint point=local_point(origin,e,n,planet_radius,alt);
        TrajectoryPoint tp={0,point.latitude,point.longitude,alt,0,PHASE_TAEM,TRAJ_REFERENCE};
        trajectory_append(out,tp);
    }
}

void reference_trajectory_terminal_candidate(Trajectory*out,
        const LandingSite*site,const GuidanceSettings*s,double planet_radius,
        const TerminalCandidate*c,double live_altitude){
    trajectory_clear(out);
    if(!out||!site||!s||!c||!c->valid||c->kind!=TERMINAL_PATH_HAC||
       !c->join.valid||!c->join.heading_cone||!(c->radius>0.0)||
       !(c->final_distance>0.0))return;

    const HACTransitionPlan*join=&c->join;
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    double lead_length=fmax(0.0,join->lead_length);
    double arc_length=fmax(0.0,join->cone_arc_length);
    double terminal_length=lead_length+arc_length;
    double final_entry_altitude=site->altitude+
        c->final_distance*tan(s->final_glide_slope*DEG2RAD);
    double start_altitude=isfinite(live_altitude)?live_altitude:
        (isfinite(c->altitude)?c->altitude:final_entry_altitude);
    double path_slope=isfinite(c->slope)?
        clampd(c->slope,0.0,60.0):s->taem_glide_slope;
    double nominal_start=final_entry_altitude+
        terminal_length*tan(path_slope*DEG2RAD);
    double start_correction=start_altitude-nominal_start;

    if(lead_length>1.0){
        const int lead_samples=48;
        for(int i=0;i<=lead_samples;i++){
            double station=lead_length*(double)i/(double)lead_samples;
            HACPoint2 q={0};double sample_course=NAN,sample_curvature=NAN;
            if(!hac_lead_sample_distance(join,station,&q,&sample_course,
                    &sample_curvature))continue;
            double remaining=fmax(0.0,lead_length-station)+arc_length;
            double blend=terminal_length>DBL_MIN?
                clampd(remaining/terminal_length,0.0,1.0):0.0;
            double alt=final_entry_altitude+
                remaining*tan(path_slope*DEG2RAD)+start_correction*blend;
            GeoPoint point=local_point(origin,q.e,q.n,planet_radius,alt);
            TrajectoryPoint tp={0,point.latitude,point.longitude,alt,0,
                PHASE_TAEM,TRAJ_REFERENCE};
            trajectory_append(out,tp);
        }
    }else{
        double remaining=arc_length;
        double blend=terminal_length>DBL_MIN?
            clampd(remaining/terminal_length,0.0,1.0):0.0;
        double alt=final_entry_altitude+
            remaining*tan(path_slope*DEG2RAD)+start_correction*blend;
        GeoPoint point=local_point(origin,join->p0.e,join->p0.n,
            planet_radius,alt);
        TrajectoryPoint tp={0,point.latitude,point.longitude,alt,0,
            PHASE_TAEM,TRAJ_REFERENCE};
        trajectory_append(out,tp);
    }

    if(arc_length>DBL_MIN){
        double arc_total=arc_length/c->radius;
        const int arc_samples=48;
        for(int i=1;i<=arc_samples;i++){
            double travelled=arc_total*(double)i/(double)arc_samples;
            double a=join->cone_start_angle-c->side*travelled;
            double e=join->cone_center.e+cos(a)*c->radius;
            double n=join->cone_center.n+sin(a)*c->radius;
            double remaining=fmax(0.0,arc_length-travelled*c->radius);
            double blend=terminal_length>DBL_MIN?
                clampd(remaining/terminal_length,0.0,1.0):0.0;
            double alt=final_entry_altitude+
                remaining*tan(path_slope*DEG2RAD)+start_correction*blend;
            GeoPoint point=local_point(origin,e,n,planet_radius,alt);
            TrajectoryPoint tp={0,point.latitude,point.longitude,alt,0,
                PHASE_TAEM,TRAJ_REFERENCE};
            trajectory_append(out,tp);
        }
    }

    /* The HAC exit is the outer-final station. Publish the runway-line segment
       from that exact endpoint so preview, controller and display share one path. */
    double runway=site->runway_heading*DEG2RAD;
    double ae=sin(runway),an=cos(runway);
    const int final_samples=32;
    for(int i=1;i<=final_samples;i++){
        double f=(double)i/(double)final_samples;
        double d=c->final_distance*(1.0-f);
        double e=-ae*d,n=-an*d;
        double alt=site->altitude+d*tan(s->final_glide_slope*DEG2RAD);
        GeoPoint point=local_point(origin,e,n,planet_radius,alt);
        TrajectoryPoint tp={0,point.latitude,point.longitude,alt,0,
            PHASE_FINAL,TRAJ_REFERENCE};
        trajectory_append(out,tp);
    }
}

void reference_trajectory(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double radius,double side){reference_trajectory_radius(out,site,s,radius,side,s->hac_radius);}

