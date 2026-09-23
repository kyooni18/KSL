/* Fly the already-latched finite lead while Variant-B is still upstream.
   This is only an execution reference for the selected tuple; it does not
   admit the circle and it does not add the lead to any committed distance
   accounting.  In particular, preserve the lead's signed curvature instead
   of imposing the analytic HAC side on it. */
static HACGuidance terminal_variant_b_latched_lead_guidance(
        const HACTransitionPlan*p,const Telemetry*t,double gravity,double course,
        double hac_radius,double current_e,double current_n){
    HACGuidance out={0};
    out.heading=NAN;
    if(!p||!p->valid||!t||!p->lead_curve||!(p->lead_length>1.0))return out;
    HACTransitionPlan lead=hac_lead_bezier_from_plan(p);
    double e=current_e,n=current_n;
    double u=hac_bezier_nearest_u(&lead,e,n);
    if(u>=.995)return out;
    HACPoint2 point=hac_bezier_point(&lead,u);
    HACPoint2 d=hac_bezier_derivative(&lead,u);
    double derivative=fmax(hypot(d.e,d.n),1.0);
    double tangent=norm_deg(atan2(d.e,d.n)*RAD2DEG);
    double cross_track=((e-point.e)*d.n-(n-point.n)*d.e)/derivative;
    double course_error=norm_signed_deg(tangent-course);
    double speed=fmax(1.0,guidance_lateral_speed(t));
    double preview_u=hac_bezier_advance_distance(&lead,u,
        speed*1.8);
    double curvature=hac_bezier_signed_curvature(&lead,preview_u);
    double effective_radius=1.0/fmax(fabs(curvature),
        1.0/fmax(hac_radius*2.5,1000.0));
    double tracking_time=hac_path_tracking_time(speed,effective_radius);
    double lateral=hac_frenet_lateral(speed,curvature,
        clampd(cross_track,-fmax(1200.0,hac_radius*.25),
            fmax(1200.0,hac_radius*.25)),course_error,tracking_time);
    double lateral_limit=speed*hac_join_rate_cap(t,u)*DEG2RAD;
    lateral=clampd(lateral,-lateral_limit,lateral_limit);
    out.heading=tangent;
    out.course_error=course_error;
    out.lateral_acceleration=lateral;
    out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70.0,70.0);
    out.radial_error=cross_track;
    out.arc_remaining=hac_bezier_length_between(&lead,u,1.0,16);
    return out;
}

