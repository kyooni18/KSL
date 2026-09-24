/* Fly the already-latched finite acquisition/lead while Variant-B is upstream.
   This is the exact geometry certified by the planner: Shuttle-style acquisition
   arc + tangent when lead_acquisition is set, otherwise the legacy finite lead. */
static HACGuidance terminal_variant_b_latched_lead_guidance(
        const HACTransitionPlan*p,const Telemetry*t,double gravity,double course,
        double hac_radius,double current_e,double current_n){
    HACGuidance out={0};
    out.heading=NAN;
    if(!p||!p->valid||!t||!(p->lead_length>1.0))return out;

    double s=hac_lead_nearest_distance(p,current_e,current_n);
    if(s>=p->lead_length*.995)return out;
    HACPoint2 point={0};double tangent=NAN,station_curvature=NAN;
    if(!hac_lead_sample_distance(p,s,&point,&tangent,&station_curvature))
        return out;

    double h=tangent*DEG2RAD;
    double de=current_e-point.e,dn=current_n-point.n;
    double cross_track=de*cos(h)+dn*(-sin(h));
    double course_error=norm_signed_deg(tangent-course);
    double speed=fmax(1.0,guidance_lateral_speed(t));
    double preview_distance=fmin(p->lead_length,s+speed*1.8);
    HACPoint2 preview_point={0};double preview_course=NAN,curvature=NAN;
    if(!hac_lead_sample_distance(p,preview_distance,&preview_point,
            &preview_course,&curvature))return out;

    double effective_radius=1.0/fmax(fabs(curvature),
        1.0/fmax(hac_radius*2.5,1000.0));
    double tracking_time=hac_path_tracking_time(speed,effective_radius);
    double lateral=hac_frenet_lateral(speed,curvature,
        clampd(cross_track,-fmax(1200.0,hac_radius*.25),
            fmax(1200.0,hac_radius*.25)),course_error,tracking_time);
    double progress=clampd(s/fmax(p->lead_length,DBL_MIN),0.0,1.0);
    double lateral_limit=speed*hac_join_rate_cap(t,progress)*DEG2RAD;
    lateral=clampd(lateral,-lateral_limit,lateral_limit);
    out.heading=tangent;
    out.course_error=course_error;
    out.lateral_acceleration=lateral;
    out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70.0,70.0);
    out.radial_error=cross_track;
    out.arc_remaining=fmax(0.0,p->lead_length-s);
    return out;
}
