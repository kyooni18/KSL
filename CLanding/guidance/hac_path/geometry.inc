       double hac_course_rate_cap_for_speed(double true_air_speed){
    return 1.50+1.50*clampd((220.0-true_air_speed)/70.0,0.0,1.0);
}
/* Keep the flight default conservative. ShuttleSim experiments may widen only
   the normalized Bezier-join tracking envelope without changing the HAC circle,
   lateral-force, energy, or vertical-response gates. */
static double hac_transition_rate_ratio_limit(void){
    const char *value=getenv("KSP_LANDER_HAC_JOIN_RATE_RATIO_LIMIT");
    if(value&&*value){
        char *end=NULL;double parsed=strtod(value,&end);
        if(end&&end!=value&&isfinite(parsed))return clampd(parsed,1.0,3.0);
    }
    return 1.60/1.50;
}
       double hac_course_rate_cap(const Telemetry*t){
    return hac_course_rate_cap_for_speed(t->true_air_speed);
}
       double guidance_lateral_speed(const Telemetry*t){
    if(!t)return 0.0;
    if(isfinite(t->horizontal_speed)&&t->horizontal_speed>1.0)
        return t->horizontal_speed;
    if(isfinite(t->surface_speed)&&t->surface_speed>1.0)
        return t->surface_speed;
    if(isfinite(t->true_air_speed)&&t->true_air_speed>1.0){
        double c=isfinite(t->flight_path_angle)?
            fabs(cos(t->flight_path_angle*DEG2RAD)):1.0;
        return fmax(1.0,t->true_air_speed*c);
    }
    return 1.0;
}

static void append_veto_reason(char *buf,size_t n,bool *first,const char *text){
    if(!buf||n==0||!first||!text)return;
    size_t used=strlen(buf);
    if(used>=n-1)return;
    int written=snprintf(buf+used,n-used,"%s%s",*first?"":"|",text);
    if(written>0)*first=false;
}
       void taem_capture_veto_reasons(unsigned veto,char *buf,size_t n){
    if(!buf||n==0)return;
    buf[0]='\0';
    if(veto==0u){snprintf(buf,n,"none");return;}
    bool first=true;
    if(veto&1u)append_veto_reason(buf,n,&first,"speed");
    if(veto&2u)append_veto_reason(buf,n,&first,"spatial");
    if(veto&4u)append_veto_reason(buf,n,&first,"course");
    if(veto&8u)append_veto_reason(buf,n,&first,"altitude");
    if(veto&16u)append_veto_reason(buf,n,&first,"maneuver-energy");
    if(veto&32u)append_veto_reason(buf,n,&first,"fpa");
    if(veto&64u)append_veto_reason(buf,n,&first,"structural");
    if(veto&128u)append_veto_reason(buf,n,&first,"hac-radius");
    if(veto&256u)append_veto_reason(buf,n,&first,"minimum-altitude");
    if(buf[0]=='\0')snprintf(buf,n,"unknown");
}
       double hac_join_rate_cap(const Telemetry*t,double progress){
    double u=clampd(progress/.42,0.0,1.0),smooth=u*u*(3.0-2.0*u);
    return hac_course_rate_cap(t)*(1.10+.50*smooth)/1.50;
}

       HACPoint2 hac_point(double e,double n){HACPoint2 p={e,n};return p;}
       HACPoint2 hac_bezier_point(const HACTransitionPlan*p,double u){
    double v=1.0-u,b0=v*v*v,b1=3*v*v*u,b2=3*v*u*u,b3=u*u*u;
    return hac_point(b0*p->p0.e+b1*p->p1.e+b2*p->p2.e+b3*p->p3.e,
        b0*p->p0.n+b1*p->p1.n+b2*p->p2.n+b3*p->p3.n);
}
       HACPoint2 hac_bezier_derivative(const HACTransitionPlan*p,double u){
    double v=1.0-u;
    return hac_point(3*v*v*(p->p1.e-p->p0.e)+6*v*u*(p->p2.e-p->p1.e)+3*u*u*(p->p3.e-p->p2.e),
        3*v*v*(p->p1.n-p->p0.n)+6*v*u*(p->p2.n-p->p1.n)+3*u*u*(p->p3.n-p->p2.n));
}
static HACPoint2 hac_bezier_second_derivative(const HACTransitionPlan*p,double u){
    double v=1.0-u;
    return hac_point(6*v*(p->p2.e-2*p->p1.e+p->p0.e)+6*u*(p->p3.e-2*p->p2.e+p->p1.e),
        6*v*(p->p2.n-2*p->p1.n+p->p0.n)+6*u*(p->p3.n-2*p->p2.n+p->p1.n));
}
       double hac_bezier_signed_curvature(const HACTransitionPlan*p,double u){
    HACPoint2 d=hac_bezier_derivative(p,u),dd=hac_bezier_second_derivative(p,u);
    double speed2=d.e*d.e+d.n*d.n;
    if(speed2<1e-6)return 0;
    /* e/n is a conventional x/y plane while positive aircraft bank increases
       compass heading clockwise. Negate the mathematical CCW curvature so a
       positive result has the same sign as the HAC bank command. */
    return -(d.e*dd.n-d.n*dd.e)/pow(speed2,1.5);
}
       bool hac_bezier_regular(const HACTransitionPlan*p){
    /* Uniform curvature samples can straddle a near-cusp and miss its
       arbitrarily large peak. A path coordinate is undefined at a zero
       derivative; reject the reversal before rating/scoring the candidate. */
    double chord=hypot(p->p3.e-p->p0.e,p->p3.n-p->p0.n);
    HACPoint2 previous=hac_bezier_derivative(p,0);
    for(int i=0;i<=64;i++){
        HACPoint2 d=hac_bezier_derivative(p,i/64.0);
        if(hypot(d.e,d.n)<fmax(1.0,chord*.03)||
           d.e*previous.e+d.n*previous.n<=0)return false;
        previous=d;
    }
    return true;
}
       double hac_bezier_length_between(const HACTransitionPlan*p,double from,double to,int segments){
    from=clampd(from,0,1);to=clampd(to,from,1);segments=segments<2?2:segments;
    HACPoint2 prev=hac_bezier_point(p,from);double total=0;
    for(int i=1;i<=segments;i++){
        double u=from+(to-from)*(double)i/(double)segments;
        HACPoint2 next=hac_bezier_point(p,u);total+=hypot(next.e-prev.e,next.n-prev.n);prev=next;
    }
    return total;
}
       double hac_transition_nearest_u(const GuidanceMachine*g,double e,double n){
    if(!g||!g->hac_transition_active||g->hac_transition_length<=1)return 1;
    HACTransitionPlan p={.valid=true,
        .p0={g->hac_transition_p0_e,g->hac_transition_p0_n},.p1={g->hac_transition_p1_e,g->hac_transition_p1_n},
        .p2={g->hac_transition_p2_e,g->hac_transition_p2_n},.p3={g->hac_transition_p3_e,g->hac_transition_p3_n},
        .length=g->hac_transition_length,.end_angle=g->hac_transition_end_angle};
    /* The transition controller is path-coordinate based, so the projection
       onto the cubic is part of the guidance state rather than a waypoint
       lookup.  Start with a dense global bracket, then solve d/du |B(u)-x|^2
       with Newton iterations inside that bracket.  This keeps the reference
       attached to the actual closest station even when the aircraft is a few
       hundred metres off the join. */
    enum{SAMPLES=48};
    double best_u=0,best2=INFINITY;
    int best_i=0;
    for(int i=0;i<=SAMPLES;i++){
        double u=(double)i/(double)SAMPLES;HACPoint2 q=hac_bezier_point(&p,u);
        double de=q.e-e,dn=q.n-n,d2=de*de+dn*dn;
        if(d2<best2){best2=d2;best_u=u;best_i=i;}
    }
    double lo=fmax(0.0,(double)(best_i-1)/(double)SAMPLES);
    double hi=fmin(1.0,(double)(best_i+1)/(double)SAMPLES);
    double u=best_u;
    for(int iter=0;iter<7;iter++){
        HACPoint2 q=hac_bezier_point(&p,u),d=hac_bezier_derivative(&p,u),dd=hac_bezier_second_derivative(&p,u);
        double re=q.e-e,rn=q.n-n;
        double gradient=re*d.e+rn*d.n;
        double hessian=d.e*d.e+d.n*d.n+re*dd.e+rn*dd.n;
        if(fabs(hessian)<1e-8)break;
        double next=clampd(u-gradient/hessian,lo,hi);
        if(fabs(next-u)<1e-7){u=next;break;}
        u=next;
    }
    return clampd(u,0,1);
}
bool hac_lead_sample_distance(const HACTransitionPlan*p,double distance,
        HACPoint2*point,double*course,double*curvature){
    if(point)*point=hac_point(NAN,NAN);
    if(course)*course=NAN;
    if(curvature)*curvature=NAN;
    if(!p||!p->valid||!isfinite(distance)||distance<0.0||
       !isfinite(p->lead_length)||p->lead_length<0.0)return false;

    double s=clampd(distance,0.0,p->lead_length);
    if(p->lead_acquisition){
        if(!(p->acquisition_radius>DBL_MIN)||
           !isfinite(p->acquisition_arc_length)||
           !isfinite(p->acquisition_tangent_length)||
           p->acquisition_arc_length<0.0||p->acquisition_tangent_length<0.0||
           fabs(p->acquisition_side)!=1.0)return false;
        if(s<=p->acquisition_arc_length+sqrt(DBL_EPSILON)){
            double theta=p->acquisition_start_angle-
                p->acquisition_side*s/p->acquisition_radius;
            HACPoint2 q=hac_point(
                p->acquisition_center.e+p->acquisition_radius*cos(theta),
                p->acquisition_center.n+p->acquisition_radius*sin(theta));
            if(point)*point=q;
            if(course)*course=norm_deg(atan2(
                p->acquisition_side*sin(theta),
                -p->acquisition_side*cos(theta))*RAD2DEG);
            if(curvature)*curvature=p->acquisition_side/p->acquisition_radius;
            return true;
        }
        double line_s=clampd(s-p->acquisition_arc_length,0.0,
            p->acquisition_tangent_length);
        double h=p->lead_end_course*DEG2RAD;
        HACPoint2 q=hac_point(
            p->acquisition_tangent.e+sin(h)*line_s,
            p->acquisition_tangent.n+cos(h)*line_s);
        if(point)*point=q;
        if(course)*course=norm_deg(p->lead_end_course);
        if(curvature)*curvature=0.0;
        return true;
    }

    if(p->lead_curve){
        HACTransitionPlan lead=hac_lead_bezier_from_plan(p);
        if(!(lead.length>DBL_MIN))return false;
        double u=hac_bezier_advance_distance(&lead,0.0,s);
        HACPoint2 q=hac_bezier_point(&lead,u);
        HACPoint2 d=hac_bezier_derivative(&lead,u);
        if(point)*point=q;
        if(course)*course=norm_deg(atan2(d.e,d.n)*RAD2DEG);
        if(curvature)*curvature=hac_bezier_signed_curvature(&lead,u);
        return true;
    }

    double h=p->lead_end_course*DEG2RAD;
    double line_length=fmax(p->lead_length,DBL_MIN);
    double f=clampd(s/line_length,0.0,1.0);
    HACPoint2 q=hac_point(
        p->lead_start.e+(p->p0.e-p->lead_start.e)*f,
        p->lead_start.n+(p->p0.n-p->lead_start.n)*f);
    if(point)*point=q;
    if(course)*course=norm_deg(atan2(p->p0.e-p->lead_start.e,
        p->p0.n-p->lead_start.n)*RAD2DEG);
    if(curvature)*curvature=0.0;
    (void)h;
    return true;
}

double hac_lead_nearest_distance(const HACTransitionPlan*p,double e,double n){
    if(!p||!p->valid||!isfinite(e)||!isfinite(n)||
       !isfinite(p->lead_length)||p->lead_length<=0.0)return 0.0;
    if(p->lead_acquisition){
        double best_s=0.0,best_d=INFINITY;
        if(p->acquisition_radius>DBL_MIN&&
           isfinite(p->acquisition_arc_length)){
            double raw=atan2(n-p->acquisition_center.n,
                e-p->acquisition_center.e);
            double progress=fmod(-p->acquisition_side*
                (raw-p->acquisition_start_angle),2.0*LANDER_PI);
            if(progress<0.0)progress+=2.0*LANDER_PI;
            double s_arc=clampd(progress*p->acquisition_radius,0.0,
                p->acquisition_arc_length);
            HACPoint2 q={0};double c=0.0,k=0.0;
            if(hac_lead_sample_distance(p,s_arc,&q,&c,&k)){
                double d=hypot(e-q.e,n-q.n);
                if(d<best_d){best_d=d;best_s=s_arc;}
            }
        }
        if(p->acquisition_tangent_length>DBL_MIN){
            double h=p->lead_end_course*DEG2RAD;
            double de=e-p->acquisition_tangent.e;
            double dn=n-p->acquisition_tangent.n;
            double along=clampd(de*sin(h)+dn*cos(h),0.0,
                p->acquisition_tangent_length);
            double s_line=p->acquisition_arc_length+along;
            HACPoint2 q={0};double c=0.0,k=0.0;
            if(hac_lead_sample_distance(p,s_line,&q,&c,&k)){
                double d=hypot(e-q.e,n-q.n);
                if(d<best_d){best_d=d;best_s=s_line;}
            }
        }
        return clampd(best_s,0.0,p->lead_length);
    }
    if(p->lead_curve){
        HACTransitionPlan lead=hac_lead_bezier_from_plan(p);
        return hac_bezier_nearest_u(&lead,e,n)*p->lead_length;
    }
    double de=p->p0.e-p->lead_start.e,dn=p->p0.n-p->lead_start.n;
    double l2=de*de+dn*dn;
    if(!(l2>DBL_MIN))return 0.0;
    double f=clampd(((e-p->lead_start.e)*de+(n-p->lead_start.n)*dn)/l2,
        0.0,1.0);
    return f*p->lead_length;
}

       double hac_bezier_nearest_u(const HACTransitionPlan*p,double e,double n){
    if(!p||!p->valid)return 1.0;
    enum{SAMPLES=48};
    double best_u=0.0,best2=INFINITY;
    int best_i=0;
    for(int i=0;i<=SAMPLES;i++){
        double u=(double)i/(double)SAMPLES;
        HACPoint2 q=hac_bezier_point(p,u);
        double de=q.e-e,dn=q.n-n,d2=de*de+dn*dn;
        if(d2<best2){best2=d2;best_u=u;best_i=i;}
    }
    double lo=fmax(0.0,(double)(best_i-1)/(double)SAMPLES);
    double hi=fmin(1.0,(double)(best_i+1)/(double)SAMPLES);
    double u=best_u;
    for(int iter=0;iter<7;iter++){
        HACPoint2 q=hac_bezier_point(p,u),d=hac_bezier_derivative(p,u),
            dd=hac_bezier_second_derivative(p,u);
        double re=q.e-e,rn=q.n-n;
        double gradient=re*d.e+rn*d.n;
        double hessian=d.e*d.e+d.n*d.n+re*dd.e+rn*dd.n;
        if(fabs(hessian)<1e-8)break;
        double next=clampd(u-gradient/hessian,lo,hi);
        if(fabs(next-u)<1e-7){u=next;break;}
        u=next;
    }
    return clampd(u,0.0,1.0);
}

       HACTransitionPlan hac_lead_bezier_from_plan(const HACTransitionPlan*p){
    HACTransitionPlan lead={0};
    if(!p)return lead;
    lead.valid=p->valid&&p->lead_curve;
    lead.lead_curve=p->lead_curve;
    lead.p0=p->lead_start;
    lead.p1=p->lead_p1;
    lead.p2=p->lead_p2;
    lead.p3=p->p0;
    lead.length=p->lead_length;
    return lead;
}

       double hac_bezier_advance_distance(const HACTransitionPlan*p,double u,double distance){
    u=clampd(u,0,1);distance=fmax(0.0,distance);
    if(distance<=0||u>=1)return u;
    if(hac_bezier_length_between(p,u,1.0,18)<=distance)return 1.0;
    double lo=u,hi=1.0;
    for(int i=0;i<10;i++){
        double mid=(lo+hi)*.5;
        double length=hac_bezier_length_between(p,u,mid,10);
        if(length<distance)lo=mid;else hi=mid;
    }
    return (lo+hi)*.5;
}
       double fixed_hac_lead_remaining(const GuidanceMachine*g){
    if(!g||!(g->hac_transition_lead_length>0.0))return 0.0;
    double u=clampd(g->hac_transition_lead_progress,0.0,1.0);
    if(!g->hac_transition_lead_curve)
        return (1.0-u)*g->hac_transition_lead_length;
    HACTransitionPlan lead={.valid=true,.lead_curve=true,
        .p0={g->hac_transition_lead_start_e,g->hac_transition_lead_start_n},
        .p1={g->hac_transition_lead_p1_e,g->hac_transition_lead_p1_n},
        .p2={g->hac_transition_lead_p2_e,g->hac_transition_lead_p2_n},
        .p3={g->hac_transition_p0_e,g->hac_transition_p0_n},
        .length=g->hac_transition_lead_length};
    return hac_bezier_length_between(&lead,u,1.0,24);
}
       double hac_path_tracking_time(double speed,double curvature_radius){
    /* Path error is intentionally slower than the inner roll loop.  Scale the
       Frenet feedback with turn time so a wide HAC does not become a nervous
       waypoint chaser, while a compact low-speed HAC still converges before
       the exit. */
    speed=fmax(1.0,speed);curvature_radius=fmax(1000.0,curvature_radius);
    return clampd(.22*curvature_radius/speed,7.0,14.0);
}
       double hac_frenet_lateral(double speed,double curvature,double cross_track,
        double course_error_deg,double tracking_time){
    speed=fmax(1.0,speed);tracking_time=fmax(1.0,tracking_time);
    double omega=1.0/tracking_time;
    double eta=clampd(course_error_deg,-70.0,70.0)*DEG2RAD;
    /* Signed curvature uses the aircraft convention: positive turns toward
       increasing compass heading. cross_track is positive to the right of
       the path, so the final term steers left when the aircraft is right of
       the reference. */
    return speed*speed*curvature+
        2.0*.92*omega*speed*sin(eta)-
        omega*omega*cross_track;
}
       double controlled_roll_rate(const Telemetry*t){
    if(!t)return 0.0;
    /* roll_rate is d(surface bank)/dt. Body p is an actuator-axis angular rate
       and can have the opposite sign in high-AoA coupled flight. */
    return isfinite(t->roll_rate)?t->roll_rate:0.0;
}
       double controlled_aoa_rate(const Telemetry*t){
    if(!t)return 0.0;
    /* Only an actual d(AoA)/dt estimate has the semantics required here.
       Euler pitch rate and body q are different coordinates; zero is the
       conservative cold-start fallback until the bridge has two AoA samples. */
    return t->has_angle_of_attack_rate&&isfinite(t->angle_of_attack_rate)?
        t->angle_of_attack_rate:0.0;
}

/* The guidance command is itself an attitude reference consumed by two
 * different executors (the native kRPC FCS and ShuttleSim).  It therefore
 * cannot be allowed to jump faster than the vehicle can react.  Runtime
 * response identification is authoritative when valid; the configured
 * guidance envelope remains the conservative cold-start bound. */
       GuidanceAttitudeLimits stabilized_attitude_limits(
        const Telemetry *t, const GuidanceSettings *s, GuidancePhase phase,
        bool pitch_axis) {
    return decision_guidance_attitude_limits(t,s,phase,pitch_axis);
}

       void seed_stabilized_limiter(JerkLimiter *limiter, double value,
        double rate, bool angle) {
    if (!limiter || limiter->has_value || !isfinite(value)) return;
    limiter->has_value = true;
    limiter->value = angle ? norm_deg(value) : value;
    limiter->rate = isfinite(rate) ? rate : 0.0;
}


       double roll_capture_time(const Telemetry*t,double target_bank,
        double configured_rate_limit,double configured_accel_limit){
    if(!t||!isfinite(t->roll)||!isfinite(target_bank))return INFINITY;
    double rate_limit=fabs(configured_rate_limit);
    double accel_limit=fabs(configured_accel_limit);
    /*
     * A runtime adapter may provide a coherent closed-loop response model
     * (ShuttleSim does; a future live calibrator may as well).  Otherwise the
     * configured rate/acceleration values are vehicle-response parameters, not
     * capture gates, and form the cold-start capability model.
     */
    if(t->attitude_response.roll_valid){
        rate_limit=t->attitude_response.maximum_roll_rate_deg_s;
        accel_limit=t->attitude_response.maximum_roll_accel_deg_s2;
    }
    if(!(rate_limit>DBL_MIN)||!(accel_limit>DBL_MIN)||
       !isfinite(rate_limit)||!isfinite(accel_limit))return INFINITY;
    double error=norm_signed_deg(target_bank-norm_signed_deg(t->roll));
    return decision_axis_capture_time(error,controlled_roll_rate(t),
        accel_limit,rate_limit);
}

       double hac_response_lead_time(const Telemetry*t,const GuidanceSettings*s,double nominal_bank){
    if(!s)return INFINITY;
    return roll_capture_time(t,nominal_bank,s->taem_roll_rate,
        s->entry_roll_acceleration);
}
       void hac_projected_local_state(const Telemetry*t,const LandingSite*site,double planet_radius,
        double course,double seconds,double target_bank,double roll_rate_limit,double roll_accel_limit,
        double lift_accel,double bank_effectiveness,double drag_accel,
        double*out_e,double*out_n,double*out_course){
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e=0,n=0;local_offsets(origin,current,planet_radius,&e,&n);
    seconds=fmax(0.0,seconds);
    target_bank=clampd(target_bank,-70.0,70.0);
    roll_rate_limit=fmax(1.0,fabs(roll_rate_limit));
    roll_accel_limit=fmax(.5,fabs(roll_accel_limit));
    bank_effectiveness=clampd(bank_effectiveness,.35,1.8);
    lift_accel=fmax(0.0,lift_accel);drag_accel=clampd(drag_accel,-40.0,40.0);
    double bank=norm_signed_deg(t->roll);
    double bank_rate=controlled_roll_rate(t);
    if(!isfinite(bank_rate))bank_rate=0.0;
    bank_rate=clampd(bank_rate,-roll_rate_limit*1.4,roll_rate_limit*1.4);
    double course_rate=isfinite(t->course_rate)?clampd(t->course_rate,-3.5,3.5):0.0;
    double tas0=fmax(1.0,t->true_air_speed),tas=tas0;
    /* horizontal_speed is tangential velocity at the vehicle radius R+h, while
       runway/local offsets are distances on the body's reference sphere R. Keep
       the physical horizontal velocity for turn-rate dynamics, but project the
       traveled distance back onto R before advancing e/n. At 45 km on Kerbin the
       old direct integration was about 7.5% fast, or 5-6 km over a 40 s lookahead. */
    double horizontal0=fmax(1.0,t->horizontal_speed),horizontal=horizontal0;
    int steps=(int)ceil(seconds/.12);if(steps<1)steps=1;if(steps>240)steps=240;
    double dt=steps>0?seconds/(double)steps:0.0;
    for(int i=0;i<steps;i++){
        double bank_error=norm_signed_deg(target_bank-bank);
        double desired_bank_rate=clampd(bank_error/1.05,-roll_rate_limit,roll_rate_limit);
        bank_rate+=clampd(desired_bank_rate-bank_rate,-roll_accel_limit*dt,roll_accel_limit*dt);
        bank=clampd(bank+bank_rate*dt,-70.0,70.0);
        double speed_scale=clampd((tas/tas0)*(tas/tas0),.45,1.15);
        double lateral=lift_accel*speed_scale*sin(clampd(bank*bank_effectiveness,-89.0,89.0)*DEG2RAD);
        double kinematic_rate=lateral/fmax(horizontal,20.0)*RAD2DEG;
        double rate_cap=hac_course_rate_cap_for_speed(tas)*1.60/1.50;
        kinematic_rate=clampd(kinematic_rate,-rate_cap,rate_cap);
        double rate_alpha=dt/fmax(.55+dt,1e-6);
        course_rate+=(kinematic_rate-course_rate)*rate_alpha;
        double mid_course=course+course_rate*dt*.5;
        double projected_altitude=t->mean_altitude+t->vertical_speed*(i+.5)*dt;
        double surface_horizontal=horizontal*planet_radius/
            fmax(1.0,planet_radius+projected_altitude);
        e+=surface_horizontal*sin(mid_course*DEG2RAD)*dt;
        n+=surface_horizontal*cos(mid_course*DEG2RAD)*dt;
        course=norm_deg(course+course_rate*dt);
        tas=fmax(1.0,tas-drag_accel*dt);
        horizontal=fmax(1.0,horizontal0*(tas/tas0));
    }
    if(out_e)*out_e=e;if(out_n)*out_n=n;if(out_course)*out_course=norm_deg(course);
}

       double terminal_normalized_upper_violation(double value,double limit){
    if(!isfinite(value)||!isfinite(limit)||!(limit>0.0))return INFINITY;
    return fmax(0.0,value/limit-1.0);
}

       double terminal_normalized_band_violation(double value,double minimum,double maximum){
    if(!isfinite(value)||!isfinite(minimum)||!isfinite(maximum)||maximum<minimum)
        return INFINITY;
    double span=fmax(maximum-minimum,fmax(fabs(minimum),fabs(maximum))*sqrt(DBL_EPSILON));
    if(value<minimum)return (minimum-value)/span;
    if(value>maximum)return (value-maximum)/span;
    return 0.0;
}


/* Build the actual MM305 Heading-Alignment Cone.  The vehicle first remains
   on its measured/projected velocity vector until it reaches the tangent
   entry of a bounded turn.  The rounded section then follows only the arc
   needed to leave on runway heading; it is never a complete circle. */
