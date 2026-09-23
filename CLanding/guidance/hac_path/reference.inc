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
        lead.p0=(HACPoint2){g->hac_transition_lead_start_e,g->hac_transition_lead_start_n};
        lead.p1=(HACPoint2){g->hac_transition_lead_p1_e,g->hac_transition_lead_p1_n};
        lead.p2=(HACPoint2){g->hac_transition_lead_p2_e,g->hac_transition_lead_p2_n};
        lead.p3=(HACPoint2){g->hac_transition_p0_e,g->hac_transition_p0_n};
        lead.length=g->hac_transition_lead_length;
        /* The published reference must retain the complete finite C1 lead
           from the shuttle's committed initial vector.  Lead progress is
           execution state, not permission to erase the vector-origin part
           of the plan; doing so makes the HAC appear detached once the
           vehicle has advanced along the bridge. */
        double u0=0.0;
        const int lead_samples=24;
        for(int i=0;i<=lead_samples;i++){
            double u=u0+(1.0-u0)*(double)i/(double)lead_samples;
            HACPoint2 q=g->hac_transition_lead_curve?hac_bezier_point(&lead,u):
                hac_point(lead.p0.e+(lead.p3.e-lead.p0.e)*u,
                    lead.p0.n+(lead.p3.n-lead.p0.n)*u);
            double lead_remaining;
            if(i==0)lead_remaining=g->hac_transition_lead_length;
            else if(i==lead_samples)lead_remaining=0.0;
            else lead_remaining=g->hac_transition_lead_curve?
                hac_bezier_length_between(&lead,u,1.0,24):
                (1.0-u)*g->hac_transition_lead_length;
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

void reference_trajectory(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double radius,double side){reference_trajectory_radius(out,site,s,radius,side,s->hac_radius);}

