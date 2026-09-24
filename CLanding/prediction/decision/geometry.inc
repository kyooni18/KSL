static double mod_two_pi(double x) {
    double two_pi = 2.0 * LANDER_PI;
    double y = fmod(x, two_pi);
    return y < 0.0 ? y + two_pi : y;
}

static bool dubins_lsl(double a,double b,double d,double*out) {
    double tmp=d+sin(a)-sin(b);
    double p2=2.0+d*d-2.0*cos(a-b)+2.0*d*(sin(a)-sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(cos(b)-cos(a),tmp);
    *out=mod_two_pi(-a+x)+p+mod_two_pi(b-x);
    return true;
}
static bool dubins_rsr(double a,double b,double d,double*out) {
    double tmp=d-sin(a)+sin(b);
    double p2=2.0+d*d-2.0*cos(a-b)+2.0*d*(-sin(a)+sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(cos(a)-cos(b),tmp);
    *out=mod_two_pi(a-x)+p+mod_two_pi(-b+x);
    return true;
}
static bool dubins_lsr(double a,double b,double d,double*out) {
    double p2=-2.0+d*d+2.0*cos(a-b)+2.0*d*(sin(a)+sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(-cos(a)-cos(b),d+sin(a)+sin(b))-atan2(-2.0,p);
    *out=mod_two_pi(-a+x)+p+mod_two_pi(-b+x);
    return true;
}
static bool dubins_rsl(double a,double b,double d,double*out) {
    double p2=d*d-2.0+2.0*cos(a-b)-2.0*d*(sin(a)+sin(b));
    if(p2<0.0)return false;
    double p=sqrt(fmax(0.0,p2));
    double x=atan2(cos(a)+cos(b),d-sin(a)-sin(b))-atan2(2.0,p);
    *out=mod_two_pi(a-x)+p+mod_two_pi(b-x);
    return true;
}
static bool dubins_rlr(double a,double b,double d,double*out) {
    double x=(6.0-d*d+2.0*cos(a-b)+2.0*d*(sin(a)-sin(b)))/8.0;
    if(x<-1.0||x>1.0)return false;
    double p=mod_two_pi(2.0*LANDER_PI-acos(clampd(x,-1.0,1.0)));
    double t=mod_two_pi(a-atan2(cos(a)-cos(b),d-sin(a)+sin(b))+p*.5);
    double q=mod_two_pi(a-b-t+p);
    *out=t+p+q;
    return true;
}
static bool dubins_lrl(double a,double b,double d,double*out) {
    double x=(6.0-d*d+2.0*cos(a-b)+2.0*d*(-sin(a)+sin(b)))/8.0;
    if(x<-1.0||x>1.0)return false;
    double p=mod_two_pi(2.0*LANDER_PI-acos(clampd(x,-1.0,1.0)));
    double t=mod_two_pi(-a-atan2(cos(a)-cos(b),d+sin(a)-sin(b))+p*.5);
    double q=mod_two_pi(b-a-t+p);
    *out=t+p+q;
    return true;
}

static double shortest_dubins_path(double de,double dn,double start_course,
        double end_course,double radius) {
    if(!isfinite(de)||!isfinite(dn)||!isfinite(start_course)||
       !isfinite(end_course)||!(radius>0.0)||!isfinite(radius))
        return INFINITY;
    double distance=hypot(de,dn);
    if(distance<=DBL_EPSILON&&
       fabs(norm_signed_deg((end_course-start_course)*RAD2DEG))<=
           sqrt(DBL_EPSILON)*RAD2DEG)
        return 0.0;

    double d=distance/radius;
    double theta=atan2(de,dn);
    double a=mod_two_pi(start_course-theta);
    double b=mod_two_pi(end_course-theta);
    double best=INFINITY,candidate=INFINITY;
    if(dubins_lsl(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_rsr(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_lsr(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_rsl(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_rlr(a,b,d,&candidate))best=fmin(best,candidate);
    if(dubins_lrl(a,b,d,&candidate))best=fmin(best,candidate);
    return isfinite(best)?best*radius:INFINITY;
}

static bool decision_target_path_geometry(
        const GuidanceMachine *g,const Telemetry *t,double course_deg,
        double target_along_m,double target_cross_m,double target_course_deg,
        double lateral_accel_limit_mps2,const PlanetModel *p,
        const LandingConfiguration *cfg,ControlAuthorityEnvelope *control_out,
        double *range_out,double *forward_out,double *cross_out,
        double *course_error_out,double *minimum_turn_radius_out,
        double *path_length_out) {
    if (!t || !p || !cfg || !isfinite(course_deg) ||
        !isfinite(target_along_m) || !isfinite(target_cross_m) ||
        !isfinite(target_course_deg) ||
        !isfinite(t->runway_along_track) || !isfinite(t->runway_cross_track))
        return false;

    ControlAuthorityEnvelope control =
        decision_control_authority_envelope(g, t, p, cfg);
    if (!control.valid || !(t->horizontal_speed > DBL_MIN) ||
        !isfinite(t->horizontal_speed))
        return false;

    double lateral=control.maximum_lateral_accel_mps2;
    if(isfinite(lateral_accel_limit_mps2))
        lateral=fmin(lateral,fmax(0.0,lateral_accel_limit_mps2));
    if(!(lateral>DBL_MIN)||!isfinite(lateral))return false;

    double da=target_along_m-t->runway_along_track;
    double dc=target_cross_m-t->runway_cross_track;
    double runway=cfg->site.runway_heading*DEG2RAD;
    double de=da*sin(runway)+dc*cos(runway);
    double dn=da*cos(runway)-dc*sin(runway);
    double target_course=target_course_deg*DEG2RAD;

    double minimum_turn=t->horizontal_speed*t->horizontal_speed/lateral;
    double path=shortest_dubins_path(de,dn,course_deg*DEG2RAD,target_course,
        minimum_turn);
    if(!isfinite(path)||!isfinite(minimum_turn))return false;

    if(control_out)*control_out=control;
    if(range_out)*range_out=hypot(de,dn);
    if(forward_out)*forward_out=de*sin(target_course)+dn*cos(target_course);
    if(cross_out)*cross_out=-de*cos(target_course)+dn*sin(target_course);
    if(course_error_out)*course_error_out=norm_signed_deg(target_course_deg-course_deg);
    if(minimum_turn_radius_out)*minimum_turn_radius_out=minimum_turn;
    if(path_length_out)*path_length_out=path;
    return true;
}

double decision_runway_end_path_score(
        const GuidanceMachine *g,const Telemetry *t,const PlanetModel *p,
        const LandingConfiguration *cfg,const LandingSite *site) {
    if(!g||!t||!p||!cfg||!site)return INFINITY;

    LandingConfiguration candidate=*cfg;
    candidate.site=*site;
    Telemetry framed=*t;
    telemetry_reframe_runway(&framed,site,p->radius);

    double course=isfinite(framed.ground_track_heading)?
        framed.ground_track_heading:framed.heading;
    if(!isfinite(course))return INFINITY;

    /*
     * Runway choice is a reachability problem, not a hand-tuned preference.
     * Score the shortest curvature-bounded path to the final-alignment point
     * with the required runway course. decision_target_path_length derives the
     * turn radius from live KSP force/control authority and solves the Dubins
     * geometry exactly.  A runway end with no current physical path is +inf.
     *
     * Excess energy is intentionally not hidden in this selector: HAC/TAEM
     * remains responsible for adding path length.  Runway selection only asks
     * which landing direction has the physically shorter reachable terminal
     * connection.
     */
    double minimum_turn_radius=INFINITY;
    double path=decision_target_path_length(g,&framed,course,
        -candidate.guidance.final_approach_distance,0.0,
        candidate.site.runway_heading,NAN,p,&candidate,
        &minimum_turn_radius);
    return isfinite(path)?path:INFINITY;
}

double decision_target_path_length(
        const GuidanceMachine *g,const Telemetry *t,double course_deg,
        double target_along_m,double target_cross_m,double target_course_deg,
        double lateral_accel_limit_mps2,const PlanetModel *p,
        const LandingConfiguration *cfg,double *minimum_turn_radius_m) {
    double path=INFINITY;
    if(!decision_target_path_geometry(g,t,course_deg,target_along_m,target_cross_m,
        target_course_deg,lateral_accel_limit_mps2,p,cfg,NULL,NULL,NULL,NULL,NULL,
        minimum_turn_radius_m,&path))
        return INFINITY;
    return path;
}

static TargetCaptureEnvelope decision_target_capture_envelope_impl(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        double target_along_m,
        double target_cross_m,
        double target_course_deg,
        double available_time_s,
        double lateral_accel_limit_mps2,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    TargetCaptureEnvelope out = {0};
    if (!t || !p || !cfg || !isfinite(course_deg) ||
        !isfinite(target_along_m) || !isfinite(target_cross_m) ||
        !isfinite(target_course_deg) ||
        !isfinite(t->runway_along_track) || !isfinite(t->runway_cross_track))
        return out;

    if(!decision_target_path_geometry(g,t,course_deg,target_along_m,target_cross_m,
        target_course_deg,lateral_accel_limit_mps2,p,cfg,&out.control,
        &out.range_m,&out.forward_m,&out.cross_m,&out.course_error_deg,
        &out.minimum_turn_radius_m,&out.path_length_m))
        return out;

    double lateral=t->horizontal_speed*t->horizontal_speed/
        out.minimum_turn_radius_m;
    out.required_time_s=isfinite(out.path_length_m)?
        out.control.control_response_time_s+
            out.path_length_m/t->horizontal_speed:INFINITY;
    out.time_available_s=available_time_s;
    out.capture_time=decision_margin(available_time_s,out.required_time_s);

    double course_rate=lateral/t->horizontal_speed;
    out.heading_capture_time_s=course_rate>0.0?
        out.control.control_response_time_s+
            fabs(out.course_error_deg*DEG2RAD)/course_rate:INFINITY;
    double cross_rate=t->horizontal_speed*
        sin(norm_signed_deg(course_deg-target_course_deg)*DEG2RAD);
    out.lateral_capture_time_s=decision_bounded_capture_time(
        out.cross_m,cross_rate,lateral,out.control.control_response_time_s);

    out.valid=out.control.valid&&out.capture_time.valid&&
        isfinite(out.path_length_m)&&isfinite(out.required_time_s);
    out.reachable=out.valid&&out.control.controllable&&
        out.capture_time.margin>=0.0;
    return out;
}

TargetCaptureEnvelope decision_target_capture_envelope(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        double target_along_m,
        double target_cross_m,
        double target_course_deg,
        double available_time_s,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    return decision_target_capture_envelope_impl(g,t,course_deg,target_along_m,
        target_cross_m,target_course_deg,available_time_s,NAN,p,cfg);
}

TargetCaptureEnvelope decision_target_capture_envelope_with_lateral_accel(
        const GuidanceMachine *g,
        const Telemetry *t,
        double course_deg,
        double target_along_m,
        double target_cross_m,
        double target_course_deg,
        double available_time_s,
        double lateral_accel_limit_mps2,
        const PlanetModel *p,
        const LandingConfiguration *cfg) {
    return decision_target_capture_envelope_impl(g,t,course_deg,target_along_m,
        target_cross_m,target_course_deg,available_time_s,
        lateral_accel_limit_mps2,p,cfg);
}

