#include "landing.h"
#include "entry_alpha.h"
#include "entry_drag_reference.h"
#include "decision_envelope.h"
#include "taem_exec.h"
#include <stdlib.h>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


/* A fixed angular limit can reject compact low-speed energy paths even
   when measured lift provides sufficient turn authority. Use one
   low-speed tracking envelope for selection, curvature and reference slew.
   Lift/bank/load qualification remains independent of this bandwidth cap. */
static double hac_course_rate_cap_for_speed(double true_air_speed){
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
static double hac_course_rate_cap(const Telemetry*t){
    return hac_course_rate_cap_for_speed(t->true_air_speed);
}

static void append_veto_reason(char *buf,size_t n,bool *first,const char *text){
    if(!buf||n==0||!first||!text)return;
    size_t used=strlen(buf);
    if(used>=n-1)return;
    int written=snprintf(buf+used,n-used,"%s%s",*first?"":"|",text);
    if(written>0)*first=false;
}
static void taem_capture_veto_reasons(unsigned veto,char *buf,size_t n){
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
static double hac_join_rate_cap(const Telemetry*t,double progress){
    double u=clampd(progress/.42,0.0,1.0),smooth=u*u*(3.0-2.0*u);
    return hac_course_rate_cap(t)*(1.10+.50*smooth)/1.50;
}

static HACPoint2 hac_point(double e,double n){HACPoint2 p={e,n};return p;}
static HACPoint2 hac_bezier_point(const HACTransitionPlan*p,double u){
    double v=1.0-u,b0=v*v*v,b1=3*v*v*u,b2=3*v*u*u,b3=u*u*u;
    return hac_point(b0*p->p0.e+b1*p->p1.e+b2*p->p2.e+b3*p->p3.e,
        b0*p->p0.n+b1*p->p1.n+b2*p->p2.n+b3*p->p3.n);
}
static HACPoint2 hac_bezier_derivative(const HACTransitionPlan*p,double u){
    double v=1.0-u;
    return hac_point(3*v*v*(p->p1.e-p->p0.e)+6*v*u*(p->p2.e-p->p1.e)+3*u*u*(p->p3.e-p->p2.e),
        3*v*v*(p->p1.n-p->p0.n)+6*v*u*(p->p2.n-p->p1.n)+3*u*u*(p->p3.n-p->p2.n));
}
static HACPoint2 hac_bezier_second_derivative(const HACTransitionPlan*p,double u){
    double v=1.0-u;
    return hac_point(6*v*(p->p2.e-2*p->p1.e+p->p0.e)+6*u*(p->p3.e-2*p->p2.e+p->p1.e),
        6*v*(p->p2.n-2*p->p1.n+p->p0.n)+6*u*(p->p3.n-2*p->p2.n+p->p1.n));
}
static double hac_bezier_signed_curvature(const HACTransitionPlan*p,double u){
    HACPoint2 d=hac_bezier_derivative(p,u),dd=hac_bezier_second_derivative(p,u);
    double speed2=d.e*d.e+d.n*d.n;
    if(speed2<1e-6)return 0;
    /* e/n is a conventional x/y plane while positive aircraft bank increases
       compass heading clockwise. Negate the mathematical CCW curvature so a
       positive result has the same sign as the HAC bank command. */
    return -(d.e*dd.n-d.n*dd.e)/pow(speed2,1.5);
}
static bool hac_bezier_regular(const HACTransitionPlan*p){
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
static double hac_bezier_length_between(const HACTransitionPlan*p,double from,double to,int segments){
    from=clampd(from,0,1);to=clampd(to,from,1);segments=segments<2?2:segments;
    HACPoint2 prev=hac_bezier_point(p,from);double total=0;
    for(int i=1;i<=segments;i++){
        double u=from+(to-from)*(double)i/(double)segments;
        HACPoint2 next=hac_bezier_point(p,u);total+=hypot(next.e-prev.e,next.n-prev.n);prev=next;
    }
    return total;
}
static double hac_transition_nearest_u(const GuidanceMachine*g,double e,double n){
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
static double hac_bezier_advance_distance(const HACTransitionPlan*p,double u,double distance){
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
static double hac_path_tracking_time(double speed,double curvature_radius){
    /* Path error is intentionally slower than the inner roll loop.  Scale the
       Frenet feedback with turn time so a wide HAC does not become a nervous
       waypoint chaser, while a compact low-speed HAC still converges before
       the exit. */
    speed=fmax(1.0,speed);curvature_radius=fmax(1000.0,curvature_radius);
    return clampd(.22*curvature_radius/speed,7.0,14.0);
}
static double hac_frenet_lateral(double speed,double curvature,double cross_track,
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
static double controlled_roll_rate(const Telemetry*t){
    if(!t)return 0.0;
    /* roll_rate is d(surface bank)/dt. Body p is an actuator-axis angular rate
       and can have the opposite sign in high-AoA coupled flight. */
    return isfinite(t->roll_rate)?t->roll_rate:0.0;
}
static double controlled_aoa_rate(const Telemetry*t){
    if(!t)return 0.0;
    /* Only an actual d(AoA)/dt estimate has the semantics required here.
       Euler pitch rate and body q are different coordinates; zero is the
       conservative cold-start fallback until the bridge has two AoA samples. */
    return t->has_angle_of_attack_rate&&isfinite(t->angle_of_attack_rate)?
        t->angle_of_attack_rate:0.0;
}

typedef struct {
    double rate_deg_s;
    double accel_deg_s2;
} GuidanceAttitudeLimits;

/* The guidance command is itself an attitude reference consumed by two
 * different executors (the native kRPC FCS and ShuttleSim).  It therefore
 * cannot be allowed to jump faster than the vehicle can react.  Runtime
 * response identification is authoritative when valid; the configured
 * guidance envelope remains the conservative cold-start bound. */
static GuidanceAttitudeLimits stabilized_attitude_limits(
        const Telemetry *t, const GuidanceSettings *s, GuidancePhase phase,
        bool pitch_axis) {
    GuidanceAttitudeLimits limits = {0};
    if (pitch_axis) {
        /* The entry response envelope is also the longitudinal AoA response
           envelope used by the predictor.  AoA is not a raw Euler-pitch pose. */
        limits.rate_deg_s = fabs(s ? s->entry_roll_rate : 0.0);
        limits.accel_deg_s2 = fabs(s ? s->entry_roll_acceleration : 0.0);
        if (t && t->attitude_response.pitch_valid) {
            limits.rate_deg_s = fmin(limits.rate_deg_s,
                fabs(t->attitude_response.maximum_pitch_rate_deg_s));
            limits.accel_deg_s2 = fmin(limits.accel_deg_s2,
                fabs(t->attitude_response.maximum_pitch_accel_deg_s2));
        }
    } else {
        limits.rate_deg_s = fabs(s ? s->entry_roll_rate : 0.0);
        limits.accel_deg_s2 = fabs(s ? s->entry_roll_acceleration : 0.0);
        if (phase == PHASE_TAEM || phase == PHASE_HEADING_ALIGNMENT) {
            limits.rate_deg_s = fabs(s ? s->taem_roll_rate : 0.0);
            limits.accel_deg_s2 = fabs(s ? s->entry_roll_acceleration * .85 : 0.0);
        } else if (phase == PHASE_FINAL || phase == PHASE_FLARE) {
            limits.rate_deg_s = fabs(s ? s->approach_roll_rate : 0.0);
            limits.accel_deg_s2 = fabs(s ? s->entry_roll_acceleration * .55 : 0.0);
        }
        if (t && t->attitude_response.roll_valid) {
            limits.rate_deg_s = fmin(limits.rate_deg_s,
                fabs(t->attitude_response.maximum_roll_rate_deg_s));
            limits.accel_deg_s2 = fmin(limits.accel_deg_s2,
                fabs(t->attitude_response.maximum_roll_accel_deg_s2));
        }
    }
    /* Configuration is normalized before guidance starts, but fail closed if
       a caller constructs a partial test/runtime configuration. */
    limits.rate_deg_s = isfinite(limits.rate_deg_s) && limits.rate_deg_s > 0.0
        ? limits.rate_deg_s : 1.0;
    limits.accel_deg_s2 = isfinite(limits.accel_deg_s2) && limits.accel_deg_s2 > 0.0
        ? limits.accel_deg_s2 : 1.0;
    return limits;
}

static void seed_stabilized_limiter(JerkLimiter *limiter, double value,
        double rate, bool angle) {
    if (!limiter || limiter->has_value || !isfinite(value)) return;
    limiter->has_value = true;
    limiter->value = angle ? norm_deg(value) : value;
    limiter->rate = isfinite(rate) ? rate : 0.0;
}


static double roll_capture_time(const Telemetry*t,double target_bank,
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

static double hac_response_lead_time(const Telemetry*t,const GuidanceSettings*s,double nominal_bank){
    if(!s)return INFINITY;
    return roll_capture_time(t,nominal_bank,s->taem_roll_rate,
        s->entry_roll_acceleration);
}
static void hac_projected_local_state(const Telemetry*t,const LandingSite*site,double planet_radius,
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

static double terminal_normalized_upper_violation(double value,double limit){
    if(!isfinite(value)||!isfinite(limit)||!(limit>0.0))return INFINITY;
    return fmax(0.0,value/limit-1.0);
}

static double terminal_normalized_band_violation(double value,double minimum,double maximum){
    if(!isfinite(value)||!isfinite(minimum)||!isfinite(maximum)||maximum<minimum)
        return INFINITY;
    double span=fmax(maximum-minimum,fmax(fabs(minimum),fabs(maximum))*sqrt(DBL_EPSILON));
    if(value<minimum)return (minimum-value)/span;
    if(value>maximum)return (value-maximum)/span;
    return 0.0;
}

static bool __attribute__((unused)) hac_transition_plan_circle_legacy(HACTransitionPlan*out,double current_e,double current_n,
        double future_e,double future_n,double start_course,double future_course,const LandingSite*site,const GuidanceSettings*s,
        double hac_radius,double side,double start_air_speed,double end_air_speed,double speed_loss_accel,
        double max_lateral_accel,double preferred_fixed_path,
        double min_fixed_path,double max_fixed_path){
    if(!out||!site||!s||!(hac_radius>0.0)||!(max_lateral_accel>0.0)||
       !isfinite(preferred_fixed_path)||!isfinite(min_fixed_path)||!isfinite(max_fixed_path)||
       max_fixed_path<min_fixed_path)return false;
    memset(out,0,sizeof(*out));
    out->debug_min_control_ratio=INFINITY;
    out->debug_min_rate_ratio=INFINITY;
    out->debug_min_peak_lateral=INFINITY;
    (void)start_course;

    double h=site->runway_heading*DEG2RAD;
    double ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h);
    double fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance;
    double ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius;
    double fa=atan2(fn-cn,fe-ce),orient=-side;
    double response_distance=hypot(future_e-current_e,future_n-current_n);
    double cr=future_course*DEG2RAD;

    /*
     * Search topology, not preferences.  The circle may consume any arc up to
     * one revolution.  Energy/path bounds below decide whether that arc is
     * admissible; there is no arbitrary minimum arc or preferred-radius bonus.
     */
    const int arc_samples=17;
    const int handle_samples=7;
    const int curvature_samples=16;
    double maximum_arc_angle=2.0*LANDER_PI;
    double preferred_arc_angle=clampd(
        side*norm_signed_deg(site->runway_heading-future_course)*DEG2RAD,
        0.0,maximum_arc_angle);

    HACTransitionPlan chosen={0},fallback={0};
    double chosen_path_error=INFINITY,chosen_control_use=INFINITY,chosen_length=INFINITY;
    double fallback_violation=INFINITY,fallback_path_error=INFINITY,fallback_control_use=INFINITY;

    for(int ai=0;ai<arc_samples;ai++){
        double arc;
        if(ai==arc_samples-1)arc=preferred_arc_angle;
        else arc=maximum_arc_angle*(double)ai/(double)(arc_samples-2);

        double angle=fa-orient*arc;
        double te=ce+cos(angle)*hac_radius,tn=cn+sin(angle)*hac_radius;
        double tangent=norm_deg(atan2(-orient*sin(angle),orient*cos(angle))*RAD2DEG);
        double tr=tangent*DEG2RAD;
        double chord=hypot(te-future_e,tn-future_n);
        double geometric_scale=fmax(1.0,fmax(hac_radius,response_distance));
        if(!(chord>sqrt(DBL_EPSILON)*geometric_scale))continue;

        double lower_bound=response_distance+chord+arc*hac_radius;
        bool chord_over_budget=lower_bound>max_fixed_path;
        if(chord_over_budget)out->reject_path++;

        /* The cubic handles describe the finite-curvature join; they are not
           an energy-management reservoir.  Letting them consume the whole
           fixed-path budget produced 70 km tangent joins followed by tiny
           HAC arcs.  Keep the handle search tied to the geometric chord and
           let the circular HAC arc carry any remaining modeled path. */
        double transition_budget=chord;
        double handle_floor=chord/(double)(handle_samples+1);

        for(int si=0;si<handle_samples;si++)for(int ei=0;ei<handle_samples;ei++){
            double sf=(double)(si+1)/(double)(handle_samples+1);
            double ef=(double)(ei+1)/(double)(handle_samples+1);
            double start_handle=handle_floor+(transition_budget-handle_floor)*sf;
            double end_handle=handle_floor+(transition_budget-handle_floor)*ef;

            HACTransitionPlan candidate={.valid=true};
            candidate.lead_start=hac_point(current_e,current_n);
            candidate.lead_length=response_distance;
            candidate.p0=hac_point(future_e,future_n);
            candidate.p1=hac_point(future_e+sin(cr)*start_handle,
                future_n+cos(cr)*start_handle);
            candidate.p3=hac_point(te,tn);
            candidate.p2=hac_point(te-sin(tr)*end_handle,
                tn-cos(tr)*end_handle);
            if(!hac_bezier_regular(&candidate)){out->reject_control++;continue;}

            candidate.length=hac_bezier_length_between(&candidate,0,1,curvature_samples);
            candidate.end_angle=angle;
            candidate.arc_remaining=arc*hac_radius;
            if(!isfinite(candidate.length)||
               !(candidate.length>sqrt(DBL_EPSILON)*geometric_scale))
                continue;

            HACPoint2 previous_point=hac_point(0,0);
            double distance_along=0.0;
            for(int k=0;k<=curvature_samples;k++){
                double u=(double)k/(double)curvature_samples;
                double curvature=hac_bezier_signed_curvature(&candidate,u);
                HACPoint2 sampled_point=hac_bezier_point(&candidate,u);
                if(k>0)distance_along+=hypot(sampled_point.e-previous_point.e,
                    sampled_point.n-previous_point.n);
                previous_point=sampled_point;

                double speed_decay=fmax(0.0,speed_loss_accel)*distance_along/
                    fmax(start_air_speed*start_air_speed,DBL_MIN);
                double curve_speed=fmax(end_air_speed,
                    start_air_speed*exp(-speed_decay));
                if(k==curvature_samples)candidate.exit_speed=curve_speed;

                double lateral=curve_speed*curve_speed*curvature;
                candidate.peak_lateral=fmax(candidate.peak_lateral,fabs(lateral));
                double curve_course_rate=fabs(curve_speed*curvature)*RAD2DEG;
                double curve_rate_cap=hac_course_rate_cap_for_speed(curve_speed);
                if(!(curve_rate_cap>0.0))continue;
                candidate.peak_course_rate_ratio=fmax(
                    candidate.peak_course_rate_ratio,
                    curve_course_rate/curve_rate_cap);
                if(lateral*side<0.0)
                    candidate.opposite_lateral=fmax(candidate.opposite_lateral,
                        fabs(lateral));
            }

            double candidate_control_ratio=
                candidate.peak_lateral/fmax(max_lateral_accel,DBL_MIN);
            if(candidate_control_ratio<out->debug_min_control_ratio){
                out->debug_min_control_ratio=candidate_control_ratio;
                out->debug_min_peak_lateral=candidate.peak_lateral;
                out->debug_min_control_length=candidate.lead_length+candidate.length;
                out->debug_min_control_arc=candidate.arc_remaining;
                out->debug_min_control_advance=(arc-preferred_arc_angle)*RAD2DEG;
            }
            out->debug_min_rate_ratio=fmin(out->debug_min_rate_ratio,
                candidate.peak_course_rate_ratio);

            double actual_end_speed=fmax(end_air_speed,candidate.exit_speed);
            double end_lateral=actual_end_speed*actual_end_speed*
                hac_bezier_signed_curvature(&candidate,1.0);
            if(!(end_lateral*side>0.0)){
                out->reject_end++;
                continue; /* wrong signed curvature cannot join this HAC topology */
            }

            double total_fixed_path=candidate.lead_length+
                candidate.length+candidate.arc_remaining;
            double path_violation=terminal_normalized_band_violation(
                total_fixed_path,min_fixed_path,max_fixed_path);
            double control_violation=terminal_normalized_upper_violation(
                candidate_control_ratio,1.0);
            double rate_limit=hac_transition_rate_ratio_limit();
            double rate_violation=terminal_normalized_upper_violation(
                candidate.peak_course_rate_ratio,rate_limit);
            double lower_bound_violation=chord_over_budget?
                terminal_normalized_upper_violation(lower_bound,max_fixed_path):0.0;

            double violation=fmax(fmax(path_violation,control_violation),
                fmax(rate_violation,lower_bound_violation));
            candidate.degraded_path=path_violation>0.0||lower_bound_violation>0.0;
            candidate.degraded_control=control_violation>0.0;
            candidate.degraded_rate=rate_violation>0.0;
            if(candidate.degraded_path)out->reject_path++;
            if(candidate.degraded_control)out->reject_control++;
            if(candidate.degraded_rate)out->reject_rate++;
            candidate.degraded=violation>sqrt(DBL_EPSILON);
            candidate.violation_score=violation;

            double path_scale=fmax(fabs(preferred_fixed_path),DBL_MIN);
            double path_error=fabs(total_fixed_path-preferred_fixed_path)/path_scale;
            double control_use=fmax(candidate_control_ratio,
                candidate.peak_course_rate_ratio);

            if(!candidate.degraded){
                bool better=path_error<chosen_path_error||
                    (fabs(path_error-chosen_path_error)<=sqrt(DBL_EPSILON)&&
                     (control_use<chosen_control_use||
                      (fabs(control_use-chosen_control_use)<=sqrt(DBL_EPSILON)&&
                       total_fixed_path<chosen_length)));
                if(better){
                    chosen=candidate;
                    chosen_path_error=path_error;
                    chosen_control_use=control_use;
                    chosen_length=total_fixed_path;
                }
            }else{
                bool better=violation<fallback_violation||
                    (fabs(violation-fallback_violation)<=sqrt(DBL_EPSILON)&&
                     (path_error<fallback_path_error||
                      (fabs(path_error-fallback_path_error)<=sqrt(DBL_EPSILON)&&
                       control_use<fallback_control_use)));
                if(better){
                    fallback=candidate;
                    fallback_violation=violation;
                    fallback_path_error=path_error;
                    fallback_control_use=control_use;
                }
            }
        }
    }

    if(chosen.valid)*out=chosen;
    else if(fallback.valid){fallback.degraded=true;*out=fallback;}
    else return false;
    return true;
}

/* Build the actual MM305 Heading-Alignment Cone.  The vehicle first remains
   on its measured/projected velocity vector until it reaches the tangent
   entry of a bounded turn.  The rounded section then follows only the arc
   needed to leave on runway heading; it is never a complete circle. */
static bool hac_transition_plan(HACTransitionPlan*out,double current_e,double current_n,
        double future_e,double future_n,double start_course,double future_course,const LandingSite*site,const GuidanceSettings*s,
        double hac_radius,double side,double start_air_speed,double end_air_speed,double speed_loss_accel,
        double max_lateral_accel,double preferred_fixed_path,
        double min_fixed_path,double max_fixed_path){
    if(!out||!site||!s||!(hac_radius>0.0)||!(max_lateral_accel>0.0)||
       !isfinite(preferred_fixed_path)||!isfinite(min_fixed_path)||
       !isfinite(max_fixed_path)||max_fixed_path<min_fixed_path||
       fabs(side)!=1.0)return false;
    memset(out,0,sizeof(*out));
    (void)start_course;

    double runway=site->runway_heading*DEG2RAD;
    double course=future_course*DEG2RAD;
    double vh_e=sin(runway),vh_n=cos(runway);
    double rh_e=cos(runway),rh_n=-sin(runway);
    double vc_e=sin(course),vc_n=cos(course);
    double rc_e=cos(course),rc_n=-sin(course);
    double cross_future=future_e*rh_e+future_n*rh_n;
    double course_cross_rate=vc_e*rh_e+vc_n*rh_n;
    double circle_cross_delta=rc_e*rh_e+rc_n*rh_n-1.0;
    double turn_sign=side;
    double target_cross=-turn_sign*hac_radius*circle_cross_delta;
    double adaptation_distance=0.0;
    double cross_resolution=sqrt(DBL_EPSILON)*fmax(1.0,hac_radius);
    if(fabs(course_cross_rate)>cross_resolution){
        adaptation_distance=(target_cross-cross_future)/course_cross_rate;
        if(adaptation_distance<0.0)adaptation_distance=0.0;
    }

    HACPoint2 entry=hac_point(future_e+vc_e*adaptation_distance,
        future_n+vc_n*adaptation_distance);
    HACPoint2 center=hac_point(entry.e+turn_sign*hac_radius*rc_e,
        entry.n+turn_sign*hac_radius*rc_n);
    HACPoint2 exit=hac_point(center.e-turn_sign*hac_radius*rh_e,
        center.n-turn_sign*hac_radius*rh_n);

    double heading_delta=norm_signed_deg(site->runway_heading-future_course);
    double arc_angle=turn_sign>0.0?norm_deg(heading_delta):norm_deg(-heading_delta);
    double maximum_arc_angle=1.5*LANDER_PI;
    if(arc_angle>maximum_arc_angle)arc_angle=maximum_arc_angle;
    if(arc_angle<sqrt(DBL_EPSILON)*RAD2DEG)arc_angle=0.0;
    double start_angle=atan2(entry.n-center.n,entry.e-center.e);
    double end_angle=start_angle-turn_sign*arc_angle;
    double arc_length=arc_angle*hac_radius;
    double lead_length=hypot(future_e-current_e,future_n-current_n)+adaptation_distance;
    double total_path=lead_length+arc_length;
    if(!(isfinite(total_path)&&total_path>DBL_MIN))return false;

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
    candidate.lead_length=lead_length;
    candidate.length=arc_length;
    candidate.end_angle=start_angle;
    candidate.arc_remaining=0.0;

    /* Keep control handles available for diagnostics and trajectory consumers.
       The flight law uses the exact stored circle, not this single-segment
       approximation, so a 270-degree cone cannot fold into a cubic loop. */
    double handle=fmax(100.0,4.0*hac_radius*sin(fmin(arc_angle*.25,LANDER_PI*.25))/3.0);
    candidate.p1=hac_point(entry.e+vc_e*handle,entry.n+vc_n*handle);
    candidate.p2=hac_point(exit.e-vh_e*handle,exit.n-vh_n*handle);

    double samples=32.0;
    for(int i=0;i<=32;i++){
        double fraction=(double)i/samples;
        double distance=adaptation_distance+arc_length*fraction;
        double decay=fmax(0.0,speed_loss_accel)*distance/
            fmax(start_air_speed*start_air_speed,DBL_MIN);
        double speed=fmax(end_air_speed,start_air_speed*exp(-decay));
        double lateral=speed*speed/hac_radius;
        candidate.peak_lateral=fmax(candidate.peak_lateral,lateral);
        double rate=speed/hac_radius*RAD2DEG;
        double rate_cap=hac_course_rate_cap_for_speed(speed);
        candidate.peak_course_rate_ratio=fmax(candidate.peak_course_rate_ratio,
            rate/fmax(rate_cap,DBL_MIN));
        if(i==32)candidate.exit_speed=speed;
    }

    double path_to_gate=total_path;
    double path_violation=terminal_normalized_band_violation(
        total_path,min_fixed_path,max_fixed_path);
    double control_violation=terminal_normalized_upper_violation(
        candidate.peak_lateral,max_lateral_accel);
    double rate_violation=terminal_normalized_upper_violation(
        candidate.peak_course_rate_ratio,hac_transition_rate_ratio_limit());
    double violation=fmax(path_violation,fmax(control_violation,rate_violation));
    candidate.degraded_path=path_violation>0.0;
    candidate.degraded_control=control_violation>0.0;
    candidate.degraded_rate=rate_violation>0.0;
    candidate.degraded=violation>sqrt(DBL_EPSILON);
    candidate.violation_score=violation;
    (void)path_to_gate;

    if(!candidate.degraded||isfinite(candidate.violation_score)){
        *out=candidate;
        return true;
    }
    return false;
}

static void hac_transition_store(GuidanceMachine*g,const HACTransitionPlan*p,double response_time){
    g->hac_transition_active=p&&p->valid;
    g->hac_transition_heading_cone=g->hac_transition_active&&p->heading_cone;
    g->hac_transition_progress=0;g->hac_transition_lead_progress=0;
    if(!g->hac_transition_active){g->hac_transition_length=0;g->hac_transition_lead_length=0;g->hac_transition_exit_speed=0;g->hac_transition_heading_cone=false;return;}
    g->hac_transition_lead_start_e=p->lead_start.e;g->hac_transition_lead_start_n=p->lead_start.n;
    g->hac_transition_lead_length=p->lead_length;
    g->hac_transition_p0_e=p->p0.e;g->hac_transition_p0_n=p->p0.n;g->hac_transition_p1_e=p->p1.e;g->hac_transition_p1_n=p->p1.n;
    g->hac_transition_p2_e=p->p2.e;g->hac_transition_p2_n=p->p2.n;g->hac_transition_p3_e=p->p3.e;g->hac_transition_p3_n=p->p3.n;
    g->hac_transition_cone_center_e=p->cone_center.e;g->hac_transition_cone_center_n=p->cone_center.n;
    g->hac_transition_cone_start_angle=p->cone_start_angle;
    g->hac_transition_cone_end_angle=p->cone_end_angle;
    g->hac_transition_cone_arc_length=p->cone_arc_length;
    g->hac_transition_length=p->length;g->hac_transition_end_angle=p->end_angle;g->hac_transition_exit_speed=p->exit_speed;g->hac_transition_response_time=response_time;
}

static HACGuidance hac_guidance_radius(const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity,double course,double hac_radius){
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    return hac_guidance_compute_radius(current,t->true_air_speed,course,site,s,radius,side,gravity,hac_radius);
}

typedef struct {
    HACGuidance guidance;
    double path_progress;
    bool valid;
    bool handoff_ready;
} HACPhaseGuidance;

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
    double speed=fmax(1.0,t->true_air_speed);
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
static HACGuidance terminal_runway_line_path_guidance(const Telemetry*t,
        const LandingSite*site,const GuidanceSettings*s,double gravity,double course){
    HACGuidance out;memset(&out,0,sizeof(out));
    double speed=fmax(1.0,t->horizontal_speed);
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
    out.desired_altitude=site->altitude+
        (s->final_approach_distance+station_gap)*tan(s->taem_glide_slope*DEG2RAD);
    return out;
}

static __attribute__((unused)) HACGuidance hac_heading_cone_path_guidance(const GuidanceMachine*g,
        const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,
        double gravity,double course){
    HACGuidance out;memset(&out,0,sizeof(out));
    (void)g;
    (void)s;
    (void)gravity;
    (void)course;
    HACPoint2 point;
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    local_offsets(sp,current,0.0,&point.e,&point.n);
    /* local_offsets expects the planet radius; the caller supplies the
       already projected coordinates through the transition state below. */
    (void)point;
    return out;
}

/* Track the committed join+HAC as a continuous geometric trajectory.  The
   aircraft is projected onto the closest path station every frame.  Bank is
   generated from the station's signed curvature plus damped Frenet
   cross-track/course feedback; no waypoint or heading-pursuit term owns the
   turn. targetHeading is only the local path tangent used for diagnostics and
   the existing reference-rate monitor. */
static HACGuidance hac_path_guidance(const GuidanceMachine*g,const Telemetry*t,
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
        double speed=fmax(1.0,t->horizontal_speed);
        if(g->hac_transition_lead_length>1.0&&g->hac_transition_lead_progress<.995){
            HACPoint2 start={g->hac_transition_lead_start_e,g->hac_transition_lead_start_n};
            HACPoint2 end=p.p0;
            HACPoint2 d={end.e-start.e,end.n-start.n};
            double length=fmax(hypot(d.e,d.n),1.0),length2=length*length;
            double lead_u=clampd(((e-start.e)*d.e+(n-start.n)*d.n)/length2,0.0,1.0);
            /* Preview publication resets stored progress, so use the live
               projection too. Otherwise every preview flies the lead forever
               and never reaches its retained finite-curvature join. */
            if(lead_u<1.0){
            HACPoint2 point={start.e+d.e*lead_u,start.n+d.n*lead_u};
            double tangent=norm_deg(atan2(d.e,d.n)*RAD2DEG);
            double cross_track=((e-point.e)*d.n-(n-point.n)*d.e)/length;
            double course_error=norm_signed_deg(tangent-course);
            double tracking_time=hac_path_tracking_time(speed,fmax(8000.0,hac_radius*2.5));
            double lateral=hac_frenet_lateral(speed,0.0,
                clampd(cross_track,-1200.0,1200.0),course_error,tracking_time);
            double lateral_limit=speed*hac_join_rate_cap(t,0.0)*DEG2RAD;
            lateral=clampd(lateral,-lateral_limit,lateral_limit);
            out.heading=tangent;
            out.course_error=course_error;
            out.arc_remaining=g->hac_remaining+g->hac_transition_length+
                (1.0-lead_u)*g->hac_transition_lead_length;
            out.lateral_acceleration=lateral;
            out.bank=clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70);
            return out;
            }
        }
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
    return clampd(fmax(1200.0,t->horizontal_speed*6.0),1200.0,upper);
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
static HACPhaseGuidance hac_phase_guidance(TaemPhase phase,const GuidanceMachine*g,
        const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,double planet_radius,
        double side,double gravity,double course,double hac_radius){
    HACPhaseGuidance out;memset(&out,0,sizeof(out));
    if(phase==TAEM_PHASE_S_TURN){
        /* TAEMEXEC deliberately does not prescribe an S-turn direction/path.
           Until that provider supplies its geometric objective, HACGUIDE must not
           manufacture one or reuse an MM304 reversal. */
        return out;
    }
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
        out.guidance=g&&g->terminal_path_kind==TERMINAL_PATH_SPLINE?
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
    if(g&&g->terminal_path_kind==TERMINAL_PATH_SPLINE){
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
static void reference_trajectory_radius(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double hac_radius){trajectory_clear(out);hac_radius=fmax(1000,hac_radius);GeoPoint sp={site->latitude,site->longitude,site->altitude};double h=site->runway_heading*DEG2RAD,ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h),fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance,ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius,fa=atan2(fn-cn,fe-ce),orient=-side,extent=2*LANDER_PI;for(int i=0;i<=72;i++){double f=(double)i/72,a=fa-orient*extent*(1-f),e=ce+cos(a)*hac_radius,n=cn+sin(a)*hac_radius,arc=hac_radius*extent*(1-f),alt=site->altitude+s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD)+arc*tan(s->taem_glide_slope*DEG2RAD);GeoPoint p=local_point(sp,e,n,radius,alt);TrajectoryPoint tp={0,p.latitude,p.longitude,alt,0,PHASE_TAEM,TRAJ_REFERENCE};trajectory_append(out,tp);}for(int i=1;i<=40;i++){double f=(double)i/40,d=s->final_approach_distance*(1-f),e=-ae*d,n=-an*d,alt=site->altitude+d*tan(s->final_glide_slope*DEG2RAD);GeoPoint p=local_point(sp,e,n,radius,alt);TrajectoryPoint tp={0,p.latitude,p.longitude,alt,0,PHASE_FINAL,TRAJ_REFERENCE};trajectory_append(out,tp);}}
void reference_trajectory(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double radius,double side){reference_trajectory_radius(out,site,s,radius,side,s->hac_radius);}

static GuidanceResult result_make(GuidancePhase phase,GuidanceCommand c,const char*status,const char*warning){GuidanceResult r;memset(&r,0,sizeof(r));r.phase=phase;r.command=c;snprintf(r.status,sizeof(r.status),"%s",status?status:"");if(warning&&*warning){r.has_warning=true;snprintf(r.warning,sizeof(r.warning),"%s",warning);}trajectory_init(&r.reference);return r;}
void guidance_result_clear(GuidanceResult*r){trajectory_clear(&r->reference);memset(r,0,sizeof(*r));}
static void reset_limiters(GuidanceMachine*g){memset(&g->pitch_limiter,0,sizeof(g->pitch_limiter));memset(&g->roll_limiter,0,sizeof(g->roll_limiter));memset(&g->heading_limiter,0,sizeof(g->heading_limiter));memset(&g->throttle_limiter,0,sizeof(g->throttle_limiter));g->has_last_stabilized_phase=false;}

static void reset_controllers(GuidanceMachine*g){
    if(!g)return;

    g->terminal_candidate.valid=false;
    g->taem_interface_target.valid=false;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->terminal_prediction_valid=false;
    g->terminal_path_committed=false;
    g->terminal_mix=0.0;
    g->terminal_reference_path_lateral_acceleration=0.0;
    g->terminal_reference_path_bank=0.0;
    g->terminal_reference_path_course_error=0.0;
    g->terminal_reference_path_arc_remaining=0.0;
    g->terminal_reference_path_transition_active=false;
    g->terminal_candidate_live_energy_margin=NAN;
    g->terminal_candidate_live_energy_valid=false;
    g->hac_plan_degraded=false;
    g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;
    g->hac_plan_violation_score=0.0;
    g->hac_commit_blend=0.0;
    g->terminal_prediction_ut=-INFINITY;

    robust_pid_reset(&g->entry_energy_pid);
    robust_pid_reset(&g->taem_altitude_pid);
    robust_pid_reset(&g->final_altitude_pid);
    robust_pid_reset(&g->speed_pid);
    robust_pid_reset(&g->flare_sink_pid);
    reset_limiters(g);
    speedbrake_controller_reset(&g->speedbrake_controller,false);

    g->airbrakes_deployed=false;
    g->final_approach_captured=false;
    g->hac_side_selected=false;
    g->hac_progress_valid=false;
    g->hac_captured=false;
    g->hac_completed=false;
    g->hac_radius=0.0;
    g->minimum_turn_radius=0.0;
    g->hac_transition_active=false;
    g->hac_transition_heading_cone=false;
    g->hac_transition_p0_e=g->hac_transition_p0_n=0.0;
    g->hac_transition_p1_e=g->hac_transition_p1_n=0.0;
    g->hac_transition_p2_e=g->hac_transition_p2_n=0.0;
    g->hac_transition_p3_e=g->hac_transition_p3_n=0.0;
    g->hac_transition_cone_center_e=g->hac_transition_cone_center_n=0.0;
    g->hac_transition_cone_start_angle=g->hac_transition_cone_end_angle=0.0;
    g->hac_transition_cone_arc_length=0.0;
    g->hac_transition_lead_start_e=g->hac_transition_lead_start_n=0.0;
    g->hac_transition_length=0.0;
    g->hac_transition_lead_length=0.0;
    g->hac_transition_lead_progress=0.0;
    g->hac_transition_response_time=0.0;
    g->hac_transition_progress=0.0;
    g->hac_transition_end_angle=0.0;
    g->hac_transition_exit_speed=0.0;
    g->hac_previous_angle=0.0;
    g->hac_remaining=0.0;

    g->terminal_region_entered=false;
    g->hac_capture_lost_duration=0.0;
    g->terminal_energy_mismatch_duration=0.0;
    g->terminal_reentry_after_ut=0.0;
    g->hac_circuit_slope=0.0;
    g->hac_circuit_count=0;

    g->terminal_glide_mode=false;
    g->terminal_rehearsal_mode=false;
    g->terminal_test_capture_active=false;
    g->terminal_test_spiral_active=false;
    g->terminal_test_glide_slope=0.0;
    g->terminal_test_final_approach_distance=0.0;
    g->terminal_test_revolution_remaining=0.0;
    g->terminal_test_preflare_altitude=0.0;
    g->terminal_test_preflare_target_speed=0.0;
    g->terminal_test_preflare_min_speed=0.0;

    g->terminal_vertical_stage=TERMINAL_TRAJECTORY_CAPTURE;
    g->terminal_vertical_stage_valid=false;
    g->gear_command_latched=false;
    g->ground_contact_latched=false;
    g->flare_sink_captured=false;
    g->terminal_response_sample_valid=false;
    g->terminal_preflare_plan_valid=false;
    g->terminal_energy_sample_valid=false;
    g->terminal_stage_started_ut=0.0;
    g->terminal_stage_good_duration=0.0;
    g->ground_contact_duration=0.0;
    g->terminal_previous_aoa=0.0;
    g->terminal_previous_vertical_speed=0.0;
    g->terminal_response_sample_ut=0.0;
    g->terminal_previous_specific_energy=0.0;
    g->terminal_previous_speed=0.0;
    g->terminal_energy_sample_ut=0.0;
    g->terminal_energy_loss_accel_ema=0.0;
    g->terminal_speed_loss_accel_ema=0.0;

    /* Unknown response stays unknown.  It is learned from telemetry or derived
       from live torque/inertia; reset must not manufacture actuator authority. */
    g->terminal_positive_aoa_rate_ema=0.0;
    g->terminal_sink_accel_ema=0.0;
    g->terminal_pitch_response_delay_ema=0.0;

    g->preflare_trigger_altitude=0.0;
    g->preflare_target_aoa=0.0;
    g->preflare_target_sink=0.0;
    g->preflare_minimum_speed=0.0;
    g->preflare_reference_speed=0.0;
    g->preflare_predicted_height_loss=0.0;
    g->preflare_predicted_kinetic_margin=0.0;
    g->preflare_effective_accel=0.0;

    g->final_invalid_duration=0.0;
    g->attitude_recovery=false;
    g->control_bad_duration=0.0;
    g->control_good_duration=0.0;
    g->recovery_duration=0.0;
    g->recovery_heading=0.0;
    g->recovery_aoa=0.0;
    g->previous_relative_roll_rate=0.0;
    g->roll_oscillation_score=0.0;
    g->roll_rate_excess_duration=0.0;
    g->has_previous_relative_roll_rate=false;
    g->previous_sideslip=0.0;
    g->has_previous_sideslip=false;

    g->entry_predictive_score=0.0;
    g->entry_reference_speed=0.0;
    g->entry_reference_altitude=0.0;
    g->entry_reference_range=0.0;
    g->has_entry_predictive_score=false;
    g->entry_control_plan_valid=false;
    g->entry_control_terminal_ready=false;
    g->entry_control_plan_ut=0.0;
    g->entry_control_bank=0.0;
    g->entry_control_aoa=0.0;
    g->entry_control_heading=0.0;
    g->entry_control_turn_radius=0.0;
    g->entry_control_segment_until_ut=0.0;
    g->entry_control_cost=0.0;
    g->entry_control_taem_range_error=0.0;
    g->entry_control_taem_speed=0.0;
    g->entry_control_taem_energy_error=0.0;
    g->entry_reversal_scheduled=false;
    g->entry_reversal_is_final=false;
    g->entry_final_reversal_pending=false;
    g->entry_final_reversal_completed=false;
    g->entry_topology_heading_locked=false;
    g->entry_continuation_bootstrap=false;
    g->entry_target_side_latched=false;
    g->entry_target_side=0.0;
    g->entry_reversal_ut=0.0;
    g->entry_reversal_range=0.0;
    g->entry_reversal_sign=0.0;
    g->entry_reversal_bank=0.0;
    g->entry_control_reversals=0u;
    g->entry_lateral_bank_magnitude=0.0;
    g->entry_geometry_bank=0.0;
    g->entry_vertical_bank_magnitude=0.0;
    g->entry_demand_bank=0.0;
    g->entry_lateral_required_bank=0.0;

    g->entry_planning_deferred=false;
    g->entry_planning_needed=false;
    g->entry_lateral_infeasible=false;
    g->entry_committed_infeasible=false;
    g->terminal_planning_deferred=false;
    g->entry_supervision_boundary_missed=false;

    entry_exec_reset(&g->entry_exec);
    memset(&g->entry_lateral,0,sizeof(g->entry_lateral));
    memset(&g->taem_s_turn_lateral,0,sizeof(g->taem_s_turn_lateral));
    memset(&g->taem_s_turn_plan,0,sizeof(g->taem_s_turn_plan));
    taem_exec_reset(&g->taem_exec);

    g->entry_alpha_has_target=false;
    g->entry_alpha_has_modulation=false;
    g->entry_alpha_target=0.0;
    g->entry_alpha_modulation=0.0;
    g->entry_drag_ratio_valid=false;
    g->entry_drag_velocity_ratio=0.0;
    g->entry_bank_authority_acquired=false;
    g->entry_supervision_valid=false;
    g->entry_supervision_mode=ENTRY_SUPERVISION_PASS_THROUGH;
    g->entry_supervision_bank_correction=0.0;
    g->entry_supervision_aoa_correction=0.0;
}

static void reset_entry_s_turn_program(GuidanceMachine*g){
    if(!g)return;
    memset(&g->entry_s_turn_plan,0,sizeof(g->entry_s_turn_plan));
    memset(&g->entry_topology,0,sizeof(g->entry_topology));
    g->entry_topology_capture_good_duration=0.0;
    g->entry_topology_heading_locked=false;
    g->entry_bank_authority_acquired=false;
    g->entry_supervision_ut=-INFINITY;
    g->entry_target_side_latched=false;
    g->entry_target_side=0.0;
}

static void reset_taem_handoff_state(GuidanceMachine*g){
    if(!g)return;
    g->taem_interface_captured=false;
    g->taem_safety_handoff=false;
}
void guidance_machine_init(GuidanceMachine*g){memset(g,0,sizeof(*g));entry_exec_reset(&g->entry_exec);taem_exec_reset(&g->taem_exec);g->entry_supervision_ut=-INFINITY;g->phase=PHASE_IDLE;g->s_turn_sign=1;g->hac_side=1;g->terminal_vertical_stage=TERMINAL_TRAJECTORY_CAPTURE;g->terminal_positive_aoa_rate_ema=0.0;g->terminal_pitch_response_delay_ema=0.0;robust_pid_init(&g->entry_energy_pid,.00048,.0000035,.00012,180000,2.5);robust_pid_init(&g->taem_altitude_pid,.0026,.000035,.005,8000,.8);robust_pid_init(&g->final_altitude_pid,.005,.00006,.012,2500,.55);robust_pid_init(&g->speed_pid,.02,.0018,.006,60,.7);robust_pid_init(&g->flare_sink_pid,.75,.08,.22,12,.35);speedbrake_controller_reset(&g->speedbrake_controller,false);}

/* Executable-plan identity is guidance state, not a logging-side guess. A new
   persistent segment receives a monotonic plan id only after all plan shaping is
   complete; the previous executable segment is retained as explicit lineage. */
static void control_plan_assign_lineage(GuidanceMachine*g,EntryControlPlan*plan,
        const EntryControlPlan*parent){
    if(!g||!plan||!plan->valid||plan->plan_id)return;
    uint64_t next=++g->control_plan_sequence;
    if(next==0)next=++g->control_plan_sequence;
    plan->plan_id=next;
    plan->plan_version=1;
    plan->parent_plan_id=parent?parent->plan_id:0;
    plan->parent_plan_version=parent?parent->plan_version:0;
}
void guidance_set_entry_predictor_models(GuidanceMachine*g,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal){
    if(!g)return;
    if(!env||!cal){g->entry_predictor_models_valid=false;return;}
    g->entry_predictor_envelope=*env;
    g->entry_predictor_calibration=*cal;
    g->entry_predictor_models_valid=true;
}
void guidance_initialize_reentry_continuation(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double initial_s_turn_sign,
        bool late_terminal_test,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal){
    if(!g||!t||!p||!cfg)return;
    guidance_machine_init(g);
    guidance_set_engaged(g,true);
    if(env&&cal)guidance_set_entry_predictor_models(g,env,cal);
    g->s_turn_sign=initial_s_turn_sign<0.0?-1.0:1.0;
    g->has_burn_command_started=true;
    g->deorbit_burn_completed=true;
    double entry_alt=entry_guidance_start_altitude(p,&cfg->guidance);
    g->atmospheric_interface_crossed=(late_terminal_test||t->mean_altitude<=entry_alt-500.0)&&
        t->vertical_speed<0.0;
    g->phase=late_terminal_test?PHASE_TAEM:
        (g->atmospheric_interface_crossed?PHASE_ENTRY_ENERGY:PHASE_ENTRY_INTERFACE);
    /* A saved atmospheric checkpoint has no durable MM304 leg history.  Mark only
       that restart bootstrap so its first fresh reversal cannot execute tens of
       seconds earlier than the uninterrupted leg solely because the old committed
       schedule was not serialized with the KSP vessel state. */
    g->entry_continuation_bootstrap=g->atmospheric_interface_crossed&&!late_terminal_test;
}
void guidance_set_engaged(GuidanceMachine*g,bool e){g->automation_engaged=e;if(e){g->aborted=false;g->abort_reason[0]=0;g->paused=false;if(g->phase==PHASE_IDLE||g->phase==PHASE_PAUSED||g->phase==PHASE_ABORT)g->phase=PHASE_COAST;}else{g->phase=PHASE_IDLE;reset_controllers(g);reset_taem_handoff_state(g);reset_entry_s_turn_program(g);speedbrake_controller_reset(&g->speedbrake_controller,false);}}
void guidance_set_paused(GuidanceMachine*g,bool p){g->paused=p;if(p){g->phase=PHASE_PAUSED;reset_limiters(g);}}
void guidance_abort(GuidanceMachine*g){g->aborted=true;g->paused=false;g->automation_engaged=false;g->phase=PHASE_ABORT;reset_controllers(g);reset_taem_handoff_state(g);reset_entry_s_turn_program(g);speedbrake_controller_reset(&g->speedbrake_controller,false);}
void guidance_reset_plan(GuidanceMachine*g){g->delivered_delta_v=0;g->has_burn_command_started=false;g->has_burn_progress_watch=false;g->burn_active_elapsed=0;g->burn_progress_watch_ut=0;g->burn_progress_watch_delta_v=0;g->deorbit_burn_completed=false;g->atmospheric_interface_crossed=false;g->has_previous_ut=false;g->hac_side_selected=false;g->final_approach_captured=false;g->has_s_turn_leg_started=false;g->has_s_turn_reversal_requested=false;g->airbrakes_deployed=false;reset_controllers(g);reset_taem_handoff_state(g);reset_entry_s_turn_program(g);speedbrake_controller_reset(&g->speedbrake_controller,false);if(g->automation_engaged)g->phase=PHASE_COAST;}
double guidance_entry_leg_elapsed(const GuidanceMachine*g,double ut){return g->phase==PHASE_ENTRY_ENERGY&&g->has_s_turn_leg_started?fmax(0,ut-g->s_turn_leg_started_ut):0;}

static double dynamic_bank_limit(const Telemetry*t,const VehicleProfile*v){
    return entry_bank_authority_limit(t->true_air_speed,t->dynamic_pressure,t->g_force,
        v,v->maximum_bank_angle);
}

static bool entry_program_shape_vertical_capture_plan(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        EntryControlPlan*plan);

static double entry_model_best_glide_aoa(const Telemetry*t,const VehicleProfile*v){
    if(t&&isfinite(t->calibrated_best_glide_angle_of_attack)&&
       t->calibrated_best_glide_angle_of_attack>=0.0)
        return clampd(t->calibrated_best_glide_angle_of_attack,0.0,
            v->maximum_angle_of_attack);

    const int samples=2*DBL_MANT_DIG;
    double best=0.0,best_ld=-INFINITY;
    for(int i=0;i<=samples;i++){
        double aoa=v->maximum_angle_of_attack*(double)i/(double)samples;
        double lf=0.0,df=0.0;
        aerodynamic_force_factors_mach(t?t->mach:0.0,aoa,v,&lf,&df);
        if(!(df>DBL_MIN)||!isfinite(lf)||!isfinite(df))continue;
        double ld=fabs(lf)/df;
        if(ld>best_ld){best_ld=ld;best=aoa;}
    }
    return best;
}

static double entry_survivability_recovery_aoa(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg,
        const ControlAuthorityEnvelope*authority){
    (void)g;
    const VehicleProfile*v=&cfg->vehicle;
    double best_glide=entry_model_best_glide_aoa(t,v);
    bool speed_or_stall=authority->speed.margin<0.0||
        (authority->stall_observed&&authority->stall.margin<0.0);
    bool load_exceeded=authority->load.margin<0.0;
    if(speed_or_stall||load_exceeded)return best_glide;

    if(authority->dynamic_pressure.margin<0.0){
        /*
         * Above q-bar limit, choose the drag-maximizing incidence that still
         * fits the configured normal-load envelope.  Candidate lift is scaled
         * from the measured current lift when available, so this is an
         * optimization against current vehicle physics rather than a fixed
         * high-AoA override.
         */
        double current_lf=0.0,current_df=0.0;
        aerodynamic_force_factors_mach(t->mach,fabs(t->angle_of_attack),v,
            &current_lf,&current_df);
        double measured_lift=t->mass>DBL_MIN&&isfinite(t->lift_force)?
            fmax(0.0,t->lift_force/t->mass):0.0;
        double gravity=planet_surface_gravity(p);
        double load_limit=v->maximum_g_load*gravity;
        const int samples=2*DBL_MANT_DIG;
        double best=best_glide,best_drag=-INFINITY;
        for(int i=0;i<=samples;i++){
            double aoa=v->maximum_angle_of_attack*(double)i/(double)samples;
            double lf=0.0,df=0.0;
            aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
            double predicted_lift=(measured_lift>0.0&&fabs(current_lf)>DBL_MIN)?
                measured_lift*fabs(lf/current_lf):0.0;
            if(predicted_lift>load_limit)continue;
            if(isfinite(df)&&df>best_drag){best_drag=df;best=aoa;}
        }
        return best;
    }
    return best_glide;
}
static void terminal_speedbrake_closed(GuidanceMachine*g){
    /*
     * Mission contract: STS-N atmospheric recovery and landing are flown
     * without speedbrakes/airbrakes.  Keep both the command latch and its
     * controller state closed so no later energy branch can resurrect them.
     */
    if(!g)return;
    g->airbrakes_deployed=false;
    g->speedbrake_controller.deployed=false;
    g->speedbrake_controller.initialized=true;
}
static GuidanceCommand atmospheric(const Telemetry*t,double heading,double roll,const VehicleProfile*v,double throttle,bool air,ControlProfile profile){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.target_heading=norm_deg(heading);c.target_roll=clampd(roll,-v->maximum_bank_angle,v->maximum_bank_angle);c.target_throttle=clampd(throttle,0,1);c.gear=t->gear;c.airbrakes=air;c.navball_speed_mode=SPEED_SURFACE;c.control_profile=profile;return c;}
static void aerodynamic_pitch_target(GuidanceCommand*c,const Telemetry*t,const VehicleProfile*v,double surface_pitch){
    c->has_target_aoa=true;
    c->target_aoa=clampd(surface_pitch-t->flight_path_angle,0,v->maximum_angle_of_attack);
    c->target_pitch=t->flight_path_angle+c->target_aoa;
}
static GuidanceResult stabilized(GuidanceMachine*g,GuidanceResult r,const Telemetry*t,const VehicleProfile*v,const GuidanceSettings*s,double dt){
    if(!r.command.autopilot_engaged||r.command.use_inertial_direction){
        reset_limiters(g);
        return r;
    }
    /*
     * Guidance owns the executable attitude reference as well as geometry and
     * safety envelopes. FlightControl still closes the surface/torque loop,
     * but it must not receive a discontinuous AoA or bank target that assumes
     * an instantaneous shuttle. The same bounded command is consumed by the
     * native FCS and by ShuttleSim, so simulator-only behavior cannot diverge
     * from the real guidance contract.
     */
    if(r.phase==PHASE_ATTITUDE_RECOVERY){
        reset_limiters(g);
        return r;
    }

    double aoa_ceiling=v->maximum_angle_of_attack;
    if(r.phase==PHASE_ENTRY_ENERGY&&r.command.has_target_aoa&&
       isfinite(r.command.target_aoa)&&r.command.target_aoa>aoa_ceiling)
        aoa_ceiling=entry_final_s_turn_aoa_ceiling(v);

    bool aerodynamic_phase=r.phase==PHASE_ENTRY_ENERGY||
        r.phase==PHASE_TAEM||r.phase==PHASE_HEADING_ALIGNMENT||
        r.phase==PHASE_FINAL||r.phase==PHASE_FLARE;

    if(aerodynamic_phase){
        double target_aoa=r.command.has_target_aoa?
            r.command.target_aoa:r.command.target_pitch-t->flight_path_angle;
        target_aoa=clampd(target_aoa,0.0,aoa_ceiling);
        r.command.has_target_aoa=true;
        r.command.target_aoa=target_aoa;
        r.command.target_pitch=t->flight_path_angle+target_aoa;
    }else{
        double target_aoa=r.command.target_pitch-t->flight_path_angle;
        target_aoa=clampd(target_aoa,0.0,aoa_ceiling);
        r.command.target_pitch=t->flight_path_angle+target_aoa;
    }

    double bank_limit=dynamic_bank_limit(t,v);
    r.command.target_roll=clampd(norm_signed_deg(r.command.target_roll),
        -bank_limit,bank_limit);
    r.command.target_heading=norm_deg(r.command.target_heading);
    r.command.target_throttle=clampd(r.command.target_throttle,0.0,1.0);

    double pitch_reference=r.command.has_target_aoa?
        r.command.target_aoa:r.command.target_pitch;
    bool pitch_reference_is_aoa=r.command.has_target_aoa;
    GuidanceAttitudeLimits pitch_limits=stabilized_attitude_limits(
        t,s,r.phase,true);
    seed_stabilized_limiter(&g->pitch_limiter,
        pitch_reference_is_aoa
            ? clampd(t->angle_of_attack,0.0,aoa_ceiling)
            : t->pitch,
        clampd(controlled_aoa_rate(t),-pitch_limits.rate_deg_s,
            pitch_limits.rate_deg_s),false);
    double limited_pitch=jerk_update(&g->pitch_limiter,pitch_reference,
        pitch_limits.rate_deg_s,pitch_limits.accel_deg_s2,dt);
    if(!isfinite(limited_pitch))
        limited_pitch=clampd(t->angle_of_attack,0.0,aoa_ceiling);
    if(pitch_reference_is_aoa)
        limited_pitch=clampd(limited_pitch,0.0,aoa_ceiling);
    g->pitch_limiter.value=limited_pitch;
    if(pitch_reference_is_aoa) {
        r.command.target_pitch=t->flight_path_angle+limited_pitch;
        r.command.target_aoa=limited_pitch;
    } else {
        r.command.target_pitch=limited_pitch;
    }

    double roll_reference=norm_signed_deg(r.command.target_roll);
    GuidanceAttitudeLimits roll_limits=stabilized_attitude_limits(
        t,s,r.phase,false);
    seed_stabilized_limiter(&g->roll_limiter,
        norm_signed_deg(t->roll),
        clampd(controlled_roll_rate(t),-roll_limits.rate_deg_s,
            roll_limits.rate_deg_s),true);
    double limited_roll=jerk_angle_update(&g->roll_limiter,roll_reference,
        roll_limits.rate_deg_s,roll_limits.accel_deg_s2,dt);
    if(!isfinite(limited_roll))
        limited_roll=norm_signed_deg(t->roll);
    r.command.target_roll=clampd(norm_signed_deg(limited_roll),
        -bank_limit,bank_limit);
    g->roll_limiter.value=norm_deg(r.command.target_roll);

    double heading_reference=norm_deg(r.command.target_heading);
    if(g->heading_limiter.has_value&&dt>0.0&&isfinite(dt))
        g->heading_limiter.rate=norm_signed_deg(heading_reference-
            norm_deg(g->heading_limiter.value))/dt;
    else g->heading_limiter.rate=0.0;
    g->heading_limiter.value=heading_reference;
    g->heading_limiter.has_value=true;

    g->throttle_limiter.value=r.command.target_throttle;
    g->throttle_limiter.has_value=true;
    g->last_stabilized_phase=r.phase;
    g->has_last_stabilized_phase=true;
    return r;
}
static GuidanceCommand entry_capture(const Telemetry*t,const VehicleState*state,const VehicleProfile*v){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.heading_control_enabled=true;double capture_aoa=entry_low_q_protective_aoa_floor(t->dynamic_pressure,v);c.has_target_aoa=true;c.target_aoa=capture_aoa;c.target_pitch=t->flight_path_angle+capture_aoa;c.target_heading=norm_deg(t->ground_track_heading);c.target_roll=0;c.control_profile=PROFILE_ENTRY;/* In vacuum, AoA/heading Euler errors are not a stable attitude target for a 180-degree retrograde-to-prograde flip. Request an inertial prograde vector instead; krpc_apply routes this particular ENTRY command through the bounded native quaternion/RCS capture law. Once q is measurable, return to the normal aerodynamic entry controller. */if(state&&fmax(0,t->dynamic_pressure)<0.5){c.use_inertial_direction=true;c.inertial_direction=vnorm(state->velocity,v3(1,0,0));c.navball_speed_mode=SPEED_ORBIT;}else{c.use_inertial_direction=false;c.navball_speed_mode=SPEED_SURFACE;}return c;}
static void request_side(GuidanceMachine*g,double sign,double ut){if(sign==g->s_turn_sign)return;g->s_turn_sign=sign;g->has_s_turn_leg_started=false;g->s_turn_reversal_requested_ut=ut;g->has_s_turn_reversal_requested=true;}

/* The reversal dwell belongs to a real aerodynamic S-turn leg, not to side
   bookkeeping. Near-vacuum roll overshoot can otherwise pre-satisfy the dwell
   hundreds of seconds before useful bank authority exists. A leg is therefore
   established only after either the executive load transition or the same physical
   aerodynamic-authority gate that permits authoritative PREENTRY bank work, while a
   meaningful same-side bank command is measurably captured. Losing that capture or
   transient PREENTRY authority restarts the dwell without changing the planned side/event. */
static void entry_update_s_turn_leg_capture(GuidanceMachine*g,const Telemetry*t,double bank,const VehicleProfile*v){
    if(!g||!t||!v)return;
    double magnitude=fabs(bank);
    double planned_sign=bank>=0?1.0:-1.0;

    double threshold=fmin(12.0,fmax(4.0,magnitude*.35));
    bool executive_loaded=g->entry_exec.initialized&&g->entry_exec.phase!=ENTRY_PHASE_PREENTRY;
    bool aerodynamic_authority=entry_s_turn_bank_authority_available(t->dynamic_pressure,
        t->true_air_speed,t->stall_fraction,t->g_force,v);
    bool loaded=executive_loaded||aerodynamic_authority;
    bool meaningful=loaded&&magnitude>=threshold;
    double actual=norm_signed_deg(t->roll);
    bool captured=meaningful&&planned_sign*actual>0.0&&fabs(actual)>=threshold;
    if(!captured){
        g->has_s_turn_leg_started=false;
        g->s_turn_leg_started_ut=0.0;
        return;
    }
    /* Measured roll capture confirms that an already-owned side is physically
       established; it does not authorize a new side. Side ownership changes only
       through the explicit initial-side selector or a committed reversal event.
       Otherwise a stray opposite bank command can become self-authorizing as soon
       as the airframe rolls through zero, bypassing the final-arc geometry gate. */
    if(planned_sign!=g->s_turn_sign){
        g->has_s_turn_leg_started=false;
        g->s_turn_leg_started_ut=0.0;
        return;
    }
    if(!g->has_s_turn_leg_started){
        g->s_turn_leg_started_ut=t->ut;
        g->has_s_turn_leg_started=true;
        g->has_s_turn_reversal_requested=false;
        if(g->entry_final_reversal_pending){
            g->entry_final_reversal_pending=false;
            g->entry_final_reversal_completed=true;
        }
    }
}
void guidance_update_entry_reversal(GuidanceMachine*g,const EntryControlPlan*plan,double ut){
    /* Reversal metadata is part of the accepted policy, not an independent
       timer. Preserve a deadline only while its bank/AoA policy is unchanged. */
    bool usable=plan&&plan->valid&&!g->entry_final_reversal_pending&&
        !g->entry_final_reversal_completed&&plan->has_planned_reversal&&
        fabs(plan->target_bank)>=4&&isfinite(plan->planned_reversal_ut)&&
        plan->planned_reversal_ut>=ut-1e-6&&plan->planned_reversal_sign*plan->target_bank<0;
    double event_ut=usable?fmax(ut,plan->planned_reversal_ut):0;
    if(usable&&g->entry_reversal_scheduled&&g->entry_reversal_sign*plan->planned_reversal_sign>0&&
            fabs(g->entry_control_bank-plan->target_bank)<1e-6&&
            fabs(g->entry_control_aoa-plan->target_aoa)<1e-6&&
            g->entry_reversal_is_final==plan->planned_reversal_is_final)
        event_ut=fmin(event_ut,g->entry_reversal_ut);
    g->entry_reversal_scheduled=usable;
    g->entry_reversal_is_final=usable&&plan->planned_reversal_is_final;
    if(usable){
        g->entry_reversal_ut=event_ut;g->entry_reversal_range=plan->planned_reversal_range;
        g->entry_reversal_sign=plan->planned_reversal_sign;
        g->entry_reversal_bank=fabs(plan->target_bank);
    }
}
static double live_lift_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v);
static double live_turn_radius(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v,double bank_deg);
static double terminal_hac_radius_live(const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s);
static double taem_bank_demand(const Telemetry*t,const HACGuidance*h,double hac_radius,double side,AerodynamicModel aero,const VehicleProfile*v);

EntryControlPlan guidance_terminal_control_plan(const GuidanceMachine*g,EntryControlPlan plan){
    if(!g||!plan.valid||!g->terminal_prediction_valid)return plan;
    /*
     * Ownership is discrete.  A terminal preview is advisory while MM304 owns
     * the vehicle and therefore cannot dilute Entry commands.  After the strict
     * handoff, MM305 may blend from the inherited command over the measured
     * candidate response time tracked in terminal_mix.
     */
    if(!g->terminal_region_entered)return plan;
    double mix=clampd(g->terminal_mix,0.0,1.0);
    plan.target_heading=norm_deg(plan.target_heading+mix*
        norm_signed_deg(g->terminal_reference_heading-plan.target_heading));
    plan.target_bank+=(g->terminal_reference_bank-plan.target_bank)*mix;
    plan.target_aoa+=(g->terminal_reference_aoa-plan.target_aoa)*mix;
    plan.has_planned_reversal=false;
    plan.final_heading_lock=false;
    return plan;
}

static bool terminal_prediction_ready(const GuidanceMachine*,const Telemetry*,double,
        const PlanetModel*,const LandingConfiguration*);

static GuidanceResult entry_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    /* Execute only after measured capture plus the minimum dwell. A forecast
       deadline cannot substitute for actual roll authority. */
    bool reversal_ready=g->has_s_turn_leg_started&&
        t->ut-g->s_turn_leg_started_ut>=s->s_turn_minimum_leg_duration;
    /* A predicted reversal time is not permission to cancel the S-turn before
       the measured trajectory has actually bent. The 50 km replay repeatedly
       rolled +/-45..60 deg while course stayed within ~2 deg of KSC, so the
       short legs cancelled their lateral acceleration and spent almost all
       downrange straight ahead. While the shuttle is high for the mission
       altitude-vs-range corridor, require a real signed course excursion before
       accepting the replayed reversal. The gate tapers away near TAEM range or
       the forced-speed handoff so terminal alignment still gets ownership. */
    double reversal_join_range=entry_taem_range_target(p,s);
    double reversal_entry_altitude=entry_guidance_start_altitude(p,s);
    double reversal_entry_range=fmax(s->target_entry_range,reversal_join_range+1000.0);
    double reversal_desired_altitude=entry_altitude_target_for_range(reversal_entry_altitude,
        reversal_entry_range,t->range_to_site,s->taem_interface_altitude,reversal_join_range);
    double reversal_altitude_debt=fmax(0.0,t->mean_altitude-reversal_desired_altitude);
    double reversal_debt_scale=fmax(4500.0,(reversal_entry_altitude-s->taem_interface_altitude)*.18);
    const TaemInterfaceTarget*reversal_target=g->taem_interface_target.valid?
        &g->taem_interface_target:NULL;
    double reversal_altitude_fraction=clampd(reversal_altitude_debt/reversal_debt_scale,0.0,1.0);
    double reversal_fpa_debt=entry_s_turn_fpa_delivery_debt(t->flight_path_angle,reversal_target);
    double reversal_fraction=clampd((t->range_to_site-reversal_join_range)/
        fmax(reversal_entry_range-reversal_join_range,1.0),0.0,1.0);
    reversal_fraction=reversal_fraction*reversal_fraction*(3.0-2.0*reversal_fraction);
    double reversal_near=atan2(2.0*s->hac_radius,fmax(reversal_entry_range,1.0))*RAD2DEG;
    double reversal_far=fmax(reversal_near,s->hac_look_ahead_angle);
    double reversal_corridor=reversal_near+(reversal_far-reversal_near)*reversal_fraction;
    /* TAEM delivery debt is a reason to keep producing useful bank/path work, not
       to oscillate around the runway bearing.  Let shallow/high trajectories fly
       farther laterally; the shared helper preserves the geometric terminal
       deadline and does not encode any desired reversal count. */
    reversal_corridor=entry_s_turn_reversal_corridor(reversal_corridor,t->range_to_site,
        reversal_join_range,reversal_altitude_fraction,reversal_fpa_debt);
    bool measured_reversal_ready=g->s_turn_sign*t->course_to_site_error<=-reversal_corridor;
    bool reversal_terminal_escape=t->true_air_speed<=s->taem_force_handoff_speed;
    bool reversal_geometry_ready=reversal_altitude_debt<=600.0||measured_reversal_ready||
        reversal_terminal_escape;
    /* A propagated non-final reversal is itself the geometry-deadline action:
       vetoing it on altitude/corridor readiness consumes the roll-through reserve
       that caused the 18:52 runway overflight. Final-exit events still require
       the stricter geometry gate before they may leave the energy-management loop. */
    bool reversal_event_admitted=!g->has_s_turn_reversal_requested&&
        (!g->entry_reversal_is_final||reversal_geometry_ready);
    if(g->entry_control_plan_valid&&g->entry_reversal_scheduled&&reversal_ready&&
       reversal_event_admitted&&t->ut+1e-6>=g->entry_reversal_ut){
        double next_sign=g->entry_reversal_sign>=0?1:-1;
        bool final_reversal=g->entry_reversal_is_final;
        double final_hac_radius=final_reversal?terminal_hac_radius_live(t,p,aero,v,s):INFINITY;
        /* A forecast may have marked this reversal as the final S-turn exit
           several tens of seconds ago. Revalidate that classification against
           the measured aerodynamic state at execution time. If the shuttle
           still cannot physically turn on the bounded dynamic HAC, executing
           point-capture now would spend the remaining KSC range before TAEM
           speed is reached. Keep the reversal, but treat it as another energy
           leg and let the next MPC solve choose a later final exit. */
        if(final_reversal&&(!isfinite(final_hac_radius)||!terminal_prediction_ready(g,t,course,p,cfg)))final_reversal=false;
        /* This is the reversal event propagated by the selected S-turn
           trajectory. Execute it directly instead of waiting for the current
           MPC segment to expire and rediscovering the same reversal later.
           Commit the new side for one minimum leg while the vehicle rolls
           through zero, otherwise the very next solve can undo the event. */
        request_side(g,next_sign,t->ut);
        if(final_reversal){
            /* The last reversal is the S-turn exit into the HAC capture tube.
               Freeze the selected terminal circle here; the subsequent entry
               frames may update measured turn authority, but they must not
               move the HAC under an aircraft that is already capturing it. */
            g->entry_final_reversal_pending=false;
            g->entry_final_reversal_completed=true;
            g->hac_side=next_sign;
            /* Final reversal accepts the retained prediction, not a new circle. */
            g->hac_side_selected=false;
            g->entry_control_bank=0;
        }else{
            double reversal_bank=fmax(fabs(g->entry_control_bank),g->entry_reversal_bank);
            if(reversal_bank>=1)g->entry_control_bank=next_sign*reversal_bank;
        }
        g->entry_control_segment_until_ut=t->ut+fmax(1,s->s_turn_minimum_leg_duration);
        g->entry_control_plan_ut=t->ut;
        g->entry_reversal_scheduled=false;
        g->entry_reversal_is_final=false;
    }
    double expiry_grace=fmax(1.0,s->prediction_interval*2.5);
    bool segment_alive=g->entry_control_segment_until_ut<=0||t->ut<=g->entry_control_segment_until_ut+expiry_grace;
    bool plan_valid=g->entry_control_plan_valid&&isfinite(g->entry_control_bank)&&isfinite(g->entry_control_aoa)&&isfinite(g->entry_control_heading)&&t->ut>=g->entry_control_plan_ut&&segment_alive;
    double bank=plan_valid?g->entry_control_bank:0;
    double aoa=plan_valid?g->entry_control_aoa:v->entry_angle_of_attack;
    double heading=plan_valid?g->entry_control_heading:t->ground_track_heading;
    /* Executive rejection must not erase the already-owned lateral job.  A
       wings-level legacy fallback can otherwise leave a scheduled reversal
       with no physically established S-turn leg, so the reversal dwell never
       completes.  Reintroduce only a small same-side course-rate floor when
       measured airflow, speed, load and stall margins prove bank authority and
       the live target still carries meaningful heading debt. */
    if(!plan_valid&&g->taem_interface_target.valid&&
       entry_s_turn_bank_authority_available(t->dynamic_pressure,t->true_air_speed,
           t->stall_fraction,t->g_force,v)){
        double target_course=g->taem_interface_target.course;
        double live_course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        double heading_debt=fabs(norm_signed_deg(target_course-live_course));
        if(isfinite(target_course)&&isfinite(live_course)&&heading_debt>20.0){
            double limit=dynamic_bank_limit(t,v);
            double floor=fmin(limit,12.0);
            bank=(norm_signed_deg(target_course-live_course)>=0.0?1.0:-1.0)*floor;
        }
    }
    bool final_heading_mode=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    if(final_heading_mode&&!g->terminal_prediction_valid){
        double radius=g->hac_side_selected&&g->hac_radius>1000.0?
            g->hac_radius:terminal_hac_radius_live(t,p,aero,v,s);
        if(isfinite(radius)){
            if(!g->hac_side_selected){
                g->hac_side=g->s_turn_sign>=0?1:-1;
                g->hac_side_selected=true;
                g->hac_radius=radius;
            }
            /* Before circle capture expose the forward tangent as the course
               reference instead of the old 500 m runway aimpoint chord. */
            GuidanceMachine tangent=*g;
            if(tangent.hac_remaining<=0)tangent.hac_remaining=2*LANDER_PI*radius;
            HACGuidance h=hac_path_guidance(&tangent,t,&cfg->site,s,p->radius,
                g->hac_side,planet_surface_gravity(p),course,radius);
            heading=h.heading;
            bank=taem_bank_demand(t,&h,radius,g->hac_side,aero,v);
        }
        g->entry_control_heading=heading;
        g->entry_control_bank=bank;
        g->entry_reversal_scheduled=false;
        g->entry_reversal_is_final=false;
    }

    /* Side selection remains plan-owned. This state only measures continuous,
       loaded capture of the currently commanded S-turn leg for reversal dwell. */
    if(!final_heading_mode)entry_update_s_turn_leg_capture(g,t,bank,v);

    const TaemInterfaceTarget*owned_target=g->taem_interface_target.valid?
        &g->taem_interface_target:NULL;
    double target_taem_speed=owned_target&&isfinite(owned_target->speed)?
        owned_target->speed:entry_taem_speed_target(v,s,p);
    double desired_s_turn_altitude=owned_target&&isfinite(owned_target->altitude)?
        owned_target->altitude:s->taem_interface_altitude;
    double s_turn_altitude_error=t->mean_altitude-desired_s_turn_altitude;
    double current_energy=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,cfg->site.latitude,desired_s_turn_altitude,
        target_taem_speed,p);
    const char*warn=NULL;
    if(!plan_valid)
        warn="Entry trajectory plan is stale; using shared physical capture shaping until the next predictor solve.";

    EntryControlPlan shaped={
        .valid=true,
        .target_bank=bank,
        .target_aoa=aoa,
        .target_heading=heading
    };
    shaped=guidance_terminal_control_plan(g,shaped);
    (void)entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&shaped);
    bank=shaped.target_bank;
    aoa=shaped.target_aoa;
    heading=shaped.target_heading;

    ControlAuthorityEnvelope safety=
        decision_control_authority_envelope(g,t,p,cfg);
    if(safety.valid&&!safety.survivable){
        bank=0.0;
        aoa=entry_survivability_recovery_aoa(g,t,p,cfg,&safety);
        warn="Entry control left the physical survivability envelope; wings-level recovery is using the active limiting margin.";
    }else{
        aoa=fmax(aoa,entry_thermal_protection_aoa_floor(v));
    }

    double bank_limit=dynamic_bank_limit(t,v);
    bank=clampd(bank,-bank_limit,bank_limit);
    aoa=clampd(aoa,0.0,v->maximum_angle_of_attack);
    /* Entry/S-turn energy management is bank/AoA only. Airbrakes are
       prohibited here because deploying them changes the shuttle's drag
       polar abruptly and invalidates the S-turn energy/trajectory solution.
       Explicitly retract any stale state inherited from another phase. */
    g->airbrakes_deployed=false;
    GuidanceCommand c=atmospheric(t,heading,bank,v,0,false,PROFILE_ENTRY);c.has_target_aoa=true;c.target_aoa=aoa;c.target_pitch=t->flight_path_angle+aoa;
    double segment_remaining=plan_valid?fmax(0,g->entry_control_segment_until_ut-t->ut):0;
    char reversal[80]={0};
    if(g->entry_reversal_scheduled)snprintf(reversal,sizeof(reversal),", next reversal %.0f s / %.0f km",fmax(0,g->entry_reversal_ut-t->ut),g->entry_reversal_range/1000);
    char status[384];if(plan_valid&&g->entry_control_terminal_ready)snprintf(status,sizeof(status),"Entry MPC: cost %.3f, TAEM %+.0f km / %.0f m/s / %+.0f kJ/kg, alt %.1f km (%+.1f), radius %.0f km, segment %.0f s, bank %+.0f°, AoA %.0f°, reversals %u%s.",g->entry_control_cost,g->entry_control_taem_range_error/1000,g->entry_control_taem_speed,g->entry_control_taem_energy_error/1000,desired_s_turn_altitude/1000,s_turn_altitude_error/1000,isfinite(g->entry_control_turn_radius)?g->entry_control_turn_radius/1000:INFINITY,segment_remaining,c.target_roll,aoa,g->entry_control_reversals,reversal);else snprintf(status,sizeof(status),"Entry MPC: cost %.3f, TAEM pending, energy %+.0f kJ/kg, capture miss %+.0f km, alt %.1f km (%+.1f), radius %.0f km, segment %.0f s, bank %+.0f°, AoA %.0f°, reversals %u%s.",plan_valid?g->entry_control_cost:INFINITY,current_energy/1000,plan_valid?g->entry_control_taem_range_error/1000:t->predicted_taem_range_error/1000,desired_s_turn_altitude/1000,s_turn_altitude_error/1000,plan_valid&&isfinite(g->entry_control_turn_radius)?g->entry_control_turn_radius/1000:INFINITY,segment_remaining,c.target_roll,aoa,g->entry_control_reversals,reversal);
    GuidanceResult r=stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,status,warn),t,v,s,dt);

    return r;
}

static double live_lift_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v){
    double incidence=hypot(t->angle_of_attack,t->sideslip),lf=0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,&lf,NULL);
    double modeled=fmax(0,t->dynamic_pressure)/fmax(20,aero.ballistic_coefficient)*fmax(0,aero.lift_to_drag)*fabs(lf);
    double measured=t->mass>1&&isfinite(t->lift_force)&&t->lift_force>0?t->lift_force/t->mass:0;
    return measured>.005?measured:modeled;
}
static double live_drag_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v){
    double incidence=hypot(t->angle_of_attack,t->sideslip),df=1;
    aerodynamic_force_factors_mach(t->mach,incidence,v,NULL,&df);
    double modeled=fmax(0,t->dynamic_pressure)/fmax(20,aero.ballistic_coefficient)*df;
    double measured=t->mass>1&&isfinite(t->drag_force)&&t->drag_force>0?t->drag_force/t->mass:0;
    return measured>.005?measured:modeled;
}

static EntryAlphaPhase entry_alpha_phase_for_exec(EntryPhase phase){
    switch(phase){
        case ENTRY_PHASE_PREENTRY:return ENTRY_ALPHA_PRE_ENTRY;
        case ENTRY_PHASE_TEMPERATURE_CONTROL:return ENTRY_ALPHA_TEMPERATURE_CONTROL;
        case ENTRY_PHASE_EQUILIBRIUM_GLIDE:return ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
        case ENTRY_PHASE_CONSTANT_DRAG:return ENTRY_ALPHA_CONSTANT_DRAG;
        case ENTRY_PHASE_TRANSITION:return ENTRY_ALPHA_TRANSITION;
        default:return ENTRY_ALPHA_PRE_ENTRY;
    }
}

static void entry_supervision_models(const Telemetry*t,AerodynamicModel aero,
        AerodynamicEnvelope*env,TrajectoryCalibrationModel*cal){
    for(int i=0;i<4;i++)env->regimes[i]=aero;
    memset(cal,0,sizeof(*cal));
    cal->density_scale=t->trajectory_density_scale>0?t->trajectory_density_scale:1.0;
    cal->drag_scale=t->trajectory_drag_scale>0?t->trajectory_drag_scale:1.0;
    cal->lift_scale=t->trajectory_lift_scale>0?t->trajectory_lift_scale:1.0;
    cal->bank_effectiveness=t->bank_effectiveness>0?t->bank_effectiveness:1.0;
    cal->speed_of_sound=t->speed_of_sound>0?t->speed_of_sound:340.0;
    cal->speed_of_sound_scale=1.0;
    cal->stress_drag_scale=1.0;cal->stress_lift_scale=1.0;
    cal->altitude_residual=t->trajectory_altitude_residual;
    cal->speed_residual=t->trajectory_speed_residual;
    cal->range_residual=t->trajectory_range_residual;
    cal->confidence=clampd(t->trajectory_calibration_confidence,0.0,1.0);
}


/* MM304 vertical capture may start before the stricter S-turn leg-authority gate.
   The existing dynamic bank limiter is already continuous from its 75 Pa knee,
   so use that envelope instead of holding wings-level until the 360 Pa leg gate.
   A synthetic low-q stall proxy must not veto this capture; only an independent
   measured stall observable may do so. Speed, q/g headroom, dynamic bank limits,
   thermal AoA protection, and later actuator/control-margin supervision remain
   authoritative. */
static bool entry_program_bank_capture_available(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg){
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    return authority.valid&&authority.controllable;
}

/* MM304 still owns the vertical state before TAEM. DRAGREF can make its
   longitudinal range residual small while the vehicle remains physically too high.
   Preserve the Shuttle-derived speed/range altitude corridor as an independent
   state objective: when the vehicle is high and shallow with useful aerodynamic
   authority, spend vertical lift with bank while preserving belly-first Entry AoA.
   Local q/g/stall and dynamic-bank protections remain authoritative. */


static double entry_program_taem_path_fpa(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*distance_out){
    if(distance_out)*distance_out=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return NAN;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double da=q->along_track-t->runway_along_track;
    double dc=q->cross_track-t->runway_cross_track;
    double distance=hypot(da,dc);
    if(distance_out)*distance_out=distance;
    if(!isfinite(distance)||distance<=DBL_MIN)return q->flight_path_angle;

    /* The vertical target is geometric outside the response/acquisition distance.
       Inside that physically reserved distance, smoothly converge to the exact
       MM304 inlet FPA. The length scale is therefore predicted acquisition lead
       plus measured response travel, not a tuned kilometre window. */
    double los=atan2(q->altitude-t->mean_altitude,distance)*RAD2DEG;
    double response_distance=fmax(0.0,q->speed)*fmax(0.0,q->response_time);
    double lead=fmax(q->acquisition_lead,response_distance);
    if(!(lead>DBL_MIN)||!isfinite(lead))return los;
    double blend=1.0-clampd(distance/lead,0.0,1.0);
    blend=blend*blend*(3.0-2.0*blend); /* decision-literal-ok: cubic smoothstep basis */
    return los+blend*(q->flight_path_angle-los);
}

/* Defined with the terminal path-provider helpers below.  MM304 uses the same
   distance-domain atmosphere/response integration as MM305 when it prices a
   candidate incidence; it must not replace that path integral with a current
   drag value multiplied by the whole future route. */
static double terminal_projected_drag_work(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double target_aoa,double ground_path,
        double slope_deg);

static bool entry_program_terminal_side_final_s_turn_armed(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double course){
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !g->entry_reversal_scheduled||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed||
       !isfinite(g->entry_reversal_sign)||!isfinite(course))return false;

    if(g->entry_control_reversals==0){
        double lead=g->entry_reversal_ut-t->ut;
        double lead_limit=fmax(60.0,cfg->guidance.s_turn_minimum_leg_duration*2.5);
        if(!isfinite(lead)||lead<0.0||lead>lead_limit)return false;
    }

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    if(!isfinite(q->course))return false;
    double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    double course_debt=norm_signed_deg(q->course-course);
    if(!isfinite(target_relative)||!isfinite(course_debt))return false;

    double resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(fabs(target_relative)<=resolution||fabs(course_debt)<=resolution)
        return false;
    double final_side=target_relative>=0.0?1.0:-1.0;
    return g->entry_reversal_sign*final_side>0.0&&
        final_side*course_debt>0.0;
}

static double entry_program_final_s_turn_drag_aoa(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const EntryDragReferenceOutput*drag,
        bool final_s_turn_active){
    if(!g||!t||!p||!cfg||!drag||!drag->valid||!final_s_turn_active||
       !g->taem_interface_target.valid)return NAN;

    const VehicleProfile*v=&cfg->vehicle;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.survivable)return NAN;

    double path_distance=NAN;
    double path_fpa=entry_program_taem_path_fpa(g,t,cfg,&path_distance);
    if(!isfinite(path_fpa)||!isfinite(path_distance)||path_distance<=DBL_MIN||
       !isfinite(t->flight_path_angle))return NAN;

    /* If the vehicle is already steeper than the geometric delivery path,
       additional drag would spend energy needed to recover that vertical debt. */
    if(t->flight_path_angle<path_fpa)return NAN;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double available=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,cfg->site.latitude,q->altitude,q->speed,p);
    if(!isfinite(available)||available<=0.0)return NAN;
    double speed_excess=t->true_air_speed-q->speed;
    double speed_excess_resolution=fmax(25.0,fabs(q->speed)*0.10);
    if(!isfinite(speed_excess)||speed_excess<=speed_excess_resolution)return NAN;

    double slope=-path_fpa;
    if(!(slope>0.0)||!(slope<90.0)||!isfinite(slope))return NAN;

    /* Price both the present incidence and each candidate over the same
       atmosphere-resolved path.  The old current-drag*air-path approximation
       implicitly held density, speed, and drag coefficient constant all the
       way to the inlet and therefore over-commanded alpha immediately after
       the shaping reversal. */
    double current_aoa=clampd(hypot(t->angle_of_attack,t->sideslip),0.0,
        v->maximum_angle_of_attack);
    double current_work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
        current_aoa,path_distance,slope);
    if(!isfinite(current_work)||current_work<0.0)return NAN;
    EnergyPathEnvelope current_energy=decision_energy_path_envelope(
        t,p,available,current_work,path_distance);
    if(!current_energy.valid)return NAN;

    double target_work=fmax(0.0,
        available-current_energy.uncertainty_specific_energy);
    /* The atmosphere-resolved path model is nominal; live Entry energy/range
       excess plus measured drag deficit is stronger evidence that the final
       S-turn must spend additional energy even when the local drag reference
       predicts a nominal undershoot.  Convert that range debt through the
       observed drag deficit so finalaoa-style cases can use the explicitly
       permitted final-S-turn AoA extension. */
    double live_range_excess=isfinite(t->energy_excess_range)?
        fmax(0.0,t->energy_excess_range):0.0;
    double drag_deficit=isfinite(drag->drag_error_accel)?
        fmax(0.0,drag->drag_error_accel):0.0;
    if(live_range_excess>0.0&&drag_deficit>0.0){
        double evidence_weight=isfinite(drag->confidence)&&drag->confidence>0.0?
            clampd(drag->confidence,0.0,1.0):1.0;
        double live_debt=live_range_excess*drag_deficit*evidence_weight;
        if(isfinite(live_debt)&&live_debt>0.0)
            target_work=fmax(target_work,current_work+live_debt);
    }
    if(current_work>=target_work)return NAN;

    double thermal=entry_thermal_protection_aoa_floor(v);
    double base=fmax(thermal,
        entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v));
    double ceiling=entry_final_s_turn_aoa_ceiling(v);
    if(!(ceiling>base))return NAN;

    double current_lf=0.0,current_df=0.0;
    aerodynamic_force_factors_mach(t->mach,fabs(t->angle_of_attack),v,
        &current_lf,&current_df);
    double measured_lift=t->mass>DBL_MIN&&isfinite(t->lift_force)?
        fmax(0.0,t->lift_force/t->mass):0.0;
    double gravity=planet_surface_gravity(p);
    double load_limit=v->maximum_g_load*gravity;

    /* Restrict the search to the portion of the incidence envelope that is
       compatible with current measured lift/load authority.  The bound is
       derived from the live polar and load limit, not from a flight-phase
       alpha constant. */
    double safe_ceiling=base;
    double model_aoa_resolution=cfg->calibration.angle_of_attack_step;
    if(!isfinite(model_aoa_resolution)||!(model_aoa_resolution>0.0))
        model_aoa_resolution=sqrt(DBL_EPSILON)*fmax(1.0,ceiling-base);
    int samples=(int)ceil((ceiling-base)/model_aoa_resolution);
    if(samples<1)samples=1;
    for(int i=0;i<=samples;i++){
        double aoa=base+(ceiling-base)*(double)i/(double)samples;
        double lf=0.0,df=0.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
        if(!isfinite(lf)||!isfinite(df)||!(df>DBL_MIN))continue;

        double predicted_lift=(measured_lift>0.0&&fabs(current_lf)>DBL_MIN)?
            measured_lift*fabs(lf/current_lf):measured_lift;
        if(predicted_lift>load_limit)continue;
        safe_ceiling=aoa;
    }

    /* The polar is calibrated in configured AoA increments.  Solving below
       that model resolution only repeats the full atmosphere/path integral
       without adding physical information, so use the model resolution as
       the numerical solve resolution and retain machine precision as a floor. */
    double resolution=fmax(model_aoa_resolution,
        sqrt(DBL_EPSILON)*fmax(1.0,safe_ceiling));
    if(!(safe_ceiling>base+resolution))return NAN;

    double low=base,high=safe_ceiling;
    double low_work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
        low,path_distance,slope);
    double high_work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
        high,path_distance,slope);
    if(!isfinite(low_work)||low_work<0.0)return NAN;
    if(!isfinite(high_work)||high_work<target_work)return safe_ceiling;
    if(low_work>=target_work)return base;

    /* Drag work is evaluated from the same live atmosphere model at every
       candidate.  Solve its first feasible crossing rather than sampling a
       preferred alpha table; this preserves the minimum drag that satisfies
       the current energy contract and avoids spending the reserve early. */
    for(unsigned i=0;i<(unsigned)(2*DBL_MANT_DIG);i++){
        if(high-low<=resolution)break;
        double mid=.5*(low+high);
        double work=terminal_projected_drag_work(g,t,p,aero,v,&cfg->guidance,
            mid,path_distance,slope);
        if(!isfinite(work)||work>=target_work)high=mid;
        else low=mid;
    }
    return high>base+resolution?high:NAN;
}

static double entry_program_vertical_bank_ceiling(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid)return NAN;
    double bank_limit=dynamic_bank_limit(t,&cfg->vehicle);
    if(!isfinite(bank_limit)||bank_limit<=0.0)return 0.0;

    double distance=NAN;
    double desired_fpa=entry_program_taem_path_fpa(g,t,cfg,&distance);
    if(!isfinite(desired_fpa)||!isfinite(t->flight_path_angle)||
       !isfinite(distance)||distance<=DBL_MIN)return bank_limit;

    double lift=live_lift_accel(t,aero,&cfg->vehicle);
    if(!isfinite(lift)||lift<=DBL_MIN)return 0.0;

    double radius=p->radius+fmax(0.0,t->mean_altitude);
    if(!(radius>DBL_MIN)||!isfinite(radius))return 0.0;
    double gravity=p->gravitational_parameter/(radius*radius);
    double horizontal=fmax(0.0,t->horizontal_speed);

    double latitude=isfinite(t->latitude)?clampd(t->latitude,-90.0,90.0):0.0;
    double course=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    double rotation=p->rotational_speed*radius*cos(latitude*DEG2RAD);
    double inertial_horizontal_sq=horizontal*horizontal+rotation*rotation+
        2.0*horizontal*rotation*sin(course*DEG2RAD);
    double equilibrium=gravity-fmax(0.0,inertial_horizontal_sq)/radius;

    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double travel_time=horizontal>DBL_MIN?distance/horizontal:INFINITY;
    double response=fmax(fmax(0.0,q->response_time),
        authority.valid?authority.control_response_time_s:0.0);
    if(isfinite(travel_time))response=fmax(response,travel_time);
    if(!(response>DBL_MIN)||!isfinite(response))return bank_limit;

    double gamma=t->flight_path_angle*DEG2RAD;
    double cos_gamma=cos(gamma);
    if(fabs(cos_gamma)<=DBL_EPSILON)return bank_limit;

    double fpa_accel=t->true_air_speed*
        (desired_fpa-t->flight_path_angle)*DEG2RAD/response;
    double altitude_accel=2.0*(q->altitude-t->mean_altitude-
        t->vertical_speed*response)/(response*response);
    double radial_lift=(equilibrium+altitude_accel+
        live_drag_accel(t,aero,&cfg->vehicle)*sin(gamma))/cos_gamma;
    double required_vertical=fmax(0.0,
        fmax(radial_lift,equilibrium*cos_gamma+fpa_accel));

    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double max_effective=fmin(bank_limit*effectiveness,
        nextafter(90.0,0.0))*DEG2RAD;
    double minimum_vertical=lift*cos(max_effective);
    if(required_vertical<=minimum_vertical)return bank_limit;
    if(required_vertical>=lift)return 0.0;

    double effective=acos(clampd(required_vertical/lift,0.0,1.0))*RAD2DEG;
    return clampd(effective/effectiveness,0.0,bank_limit);
}

static bool entry_program_altitude_capture(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        double*bank,double*aoa){
    if(!g||!t||!p||!cfg||!bank||!aoa||!g->taem_interface_target.valid)
        return false;

    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.controllable)return false;

    double demanded_magnitude=
        entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
    if(!isfinite(demanded_magnitude))return false;

    double original=*bank;
    double sign=fabs(original)>sqrt(DBL_EPSILON)?
        (original>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
    *bank=sign*demanded_magnitude;
    *aoa=fmax(*aoa,entry_thermal_protection_aoa_floor(&cfg->vehicle));

    double resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(fabs(original),fabs(*bank)));
    return fabs(*bank-original)>resolution;
}


/* Shared executable vertical shaping: both live ticks and guidance replay use
   these altitude/FPA budgets. Legacy topology proposals also receive this local
   seed, but only the full guidance replay evaluates its evolution over time. */
static bool entry_program_shape_vertical_capture_plan(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        EntryControlPlan*plan){
    if(!plan||!plan->valid)return false;
    const VehicleProfile*v=&cfg->vehicle;
    double bank=plan->target_bank,aoa=plan->target_aoa;
    bool changed=entry_program_altitude_capture(g,t,p,aero,cfg,&bank,&aoa);
    double ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
    double ceiling_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(fabs(bank),fabs(ceiling)));
    if(isfinite(ceiling)&&fabs(bank)>ceiling+ceiling_resolution){
        double sign=fabs(bank)>ceiling_resolution?
            (bank>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
        bank=sign*ceiling;
        changed=true;
    }

    /*
     * If the vehicle is already steeper than the geometric path to the owned
     * interface target, do not add drag merely because an old plan requested a
     * higher AoA.  Use the measured best-glide incidence when available, but
     * never below the configured thermal-protection floor.
     */
    double path_distance=NAN;
    double path_fpa=entry_program_taem_path_fpa(g,t,cfg,&path_distance);
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(authority.valid&&authority.survivable&&isfinite(path_fpa)&&
       isfinite(path_distance)&&path_distance>DBL_MIN&&
       isfinite(t->flight_path_angle)&&t->flight_path_angle<path_fpa){
        double thermal=entry_thermal_protection_aoa_floor(v);
        double best_glide=isfinite(t->calibrated_best_glide_angle_of_attack)&&
            t->calibrated_best_glide_angle_of_attack>0.0?
            t->calibrated_best_glide_angle_of_attack:thermal;
        double relief_target=clampd(fmax(thermal,best_glide),
            thermal,v->maximum_angle_of_attack);
        if(aoa>relief_target+sqrt(DBL_EPSILON)){
            aoa=relief_target;
            changed=true;
        }
    }


    if(!changed)return false;
    plan->target_bank=bank;
    plan->target_aoa=aoa;
    plan->target_turn_radius=live_turn_radius(t,aero,v,bank);
    return true;
}

static bool entry_program_terminal_course_endpoint_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign,
        double target_course,double*endpoint_error_out,double*radius_out);
static bool entry_program_final_reversal_geometry_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign);
static bool entry_program_terminal_geometry_blocked(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg);

typedef struct {
    bool valid;
    double vertical_score;
    double lateral_score;
    double capture_time_s;
} EntryBankAllocationSample;

static EntryBankAllocationSample entry_program_bank_allocation_sample(
        const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        const LandingConfiguration*cfg,const TaemInterfaceTarget*q,
        double course,double available_time,double lift,double drag,
        double effectiveness,double max_effective,double required_vertical,
        double available_energy,double bank){
    EntryBankAllocationSample out={0};
    double effective=fmin(bank*effectiveness,max_effective);
    double lateral=lift*fabs(sin(effective*DEG2RAD));
    TargetCaptureEnvelope capture=
        decision_target_capture_envelope_with_lateral_accel(
            g,t,course,q->along_track,q->cross_track,q->course,
            available_time,lateral,p,cfg);
    if(!capture.valid)return out;

    DecisionMargin vertical=decision_margin(
        lift*cos(effective*DEG2RAD),required_vertical);
    if(!vertical.valid)return out;

    double path_length=fmax(0.0,capture.path_length_m);
    EnergyPathEnvelope energy=decision_energy_path_envelope(
        t,p,available_energy,drag*path_length,path_length);
    if(!energy.valid||!energy.energy.valid)return out;

    out.valid=true;
    out.vertical_score=vertical.normalized_margin;
    out.lateral_score=fmin(capture.capture_time.normalized_margin,
        energy.energy.normalized_margin);
    out.capture_time_s=capture.required_time_s;
    return out;
}

/*
 * Allocate bank between MM304 vertical delivery and lateral fixed-point capture.
 *
 * The decision is a maximin balance between vertical lift margin, bounded-
 * curvature capture-time margin and unpowered energy-path margin.  The two
 * margin families are monotonic in opposite directions as bank grows, so the
 * optimum is their crossing; solve that crossing by bisection instead of
 * sampling a hand-tuned bank table.
 */
static void entry_program_apply_geometry_bank_demand(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double geometry_bank,EntryControlPlan*plan){
    if(!g||!t||!p||!cfg||!plan||!plan->valid||
       !g->taem_interface_target.valid)return;

    const VehicleProfile*v=&cfg->vehicle;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.controllable)return;

    double dynamic_limit=dynamic_bank_limit(t,v);
    double demand=clampd(fabs(geometry_bank),0.0,dynamic_limit);
    double current=fmin(fabs(plan->target_bank),demand);
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,demand);
    if(demand<=current+resolution)return;

    double vertical_ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
    if(!isfinite(vertical_ceiling))vertical_ceiling=dynamic_limit;
    vertical_ceiling=clampd(vertical_ceiling,0.0,dynamic_limit);

    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!(t->horizontal_speed>DBL_MIN))return;

    double direct_range=hypot(q->along_track-t->runway_along_track,
        q->cross_track-t->runway_cross_track);
    double available_time=NAN;
    if(isfinite(q->arrival_ut))
        available_time=fmax(0.0,q->arrival_ut-t->ut);
    else if(isfinite(q->remaining_path)&&q->remaining_path>=0.0)
        available_time=q->remaining_path/t->horizontal_speed;
    else if(isfinite(direct_range))
        available_time=authority.control_response_time_s+
            direct_range/t->horizontal_speed;
    if(!isfinite(available_time))return;
    if(entry_program_terminal_geometry_blocked(g,t,cfg))
        available_time=0.0;

    double lift=live_lift_accel(t,aero,v);
    if(!(lift>DBL_MIN)||!isfinite(lift))return;
    double drag=fmax(0.0,live_drag_accel(t,aero,v));
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double max_effective=nextafter(90.0,0.0);
    double required_effective=fmin(vertical_ceiling*effectiveness,max_effective);
    double required_vertical=lift*cos(required_effective*DEG2RAD);

    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,q->altitude,q->speed,p);
    if(!isfinite(available_energy))return;

    double low=fmax(current,resolution),high=demand;
    EntryBankAllocationSample high_sample=entry_program_bank_allocation_sample(
        g,t,p,cfg,q,course,available_time,lift,drag,effectiveness,max_effective,
        required_vertical,available_energy,high);
    if(!high_sample.valid)return;

    EntryBankAllocationSample low_sample=entry_program_bank_allocation_sample(
        g,t,p,cfg,q,course,available_time,lift,drag,effectiveness,max_effective,
        required_vertical,available_energy,low);

    double best_bank=high;
    EntryBankAllocationSample best=high_sample;
    if(low_sample.valid&&low_sample.vertical_score<=low_sample.lateral_score){
        best_bank=low;best=low_sample;
    }else if(high_sample.vertical_score<high_sample.lateral_score){
        unsigned guard=0;
        while(high-low>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
            double mid=.5*(low+high); /* decision-literal-ok: bisection midpoint */
            EntryBankAllocationSample sample=entry_program_bank_allocation_sample(
                g,t,p,cfg,q,course,available_time,lift,drag,effectiveness,max_effective,
                required_vertical,available_energy,mid);
            if(!sample.valid||sample.vertical_score>sample.lateral_score){
                low=mid;low_sample=sample;
            }else{
                high=mid;high_sample=sample;
            }
        }
        double low_score=low_sample.valid?
            fmin(low_sample.vertical_score,low_sample.lateral_score):-INFINITY;
        double high_score=fmin(high_sample.vertical_score,high_sample.lateral_score);
        if(low_score>high_score){
            best_bank=low;best=low_sample;
        }else{
            best_bank=high;best=high_sample;
        }
    }
    (void)best;

    if(best_bank<=current+resolution)return;
    double sign=g->s_turn_sign>=0.0?1.0:-1.0;
    plan->target_bank=sign*best_bank;
    plan->bank_cap=fmax(plan->bank_cap,best_bank);
    plan->target_turn_radius=live_turn_radius(t,aero,v,plan->target_bank);
}

static bool entry_program_longitudinal_bank_replan_due(const GuidanceMachine*g,
        const EntryControlPlan*demand,double ut,const GuidanceSettings*s,
        const EntryLateralLimits*limits){
    if(!g||!demand||!s||!limits||!g->entry_s_turn_plan.valid||!demand->valid||
       !isfinite(g->entry_s_turn_plan.planned_ut)||!isfinite(demand->target_bank)||
       !isfinite(g->entry_s_turn_plan.target_bank))return false;
    double minimum_age=fmax(1.0,s->prediction_interval*2.0);
    if(ut-g->entry_s_turn_plan.planned_ut<minimum_age)return false;
    double bank_delta=fabs(fabs(demand->target_bank)-fabs(g->entry_s_turn_plan.target_bank));
    double threshold=fmax(5.0,fabs(limits->maximum_bank_deg)*.10);
    return bank_delta>=threshold;
}


/* MM304 executes a propagated S-turn program. Reversal timing is selected by
   trajectory propagation. Measured leg capture proves that the commanded side
   actually became an executable leg; no second altitude/corridor/dwell policy
   is allowed to move the propagated event. */
static bool entry_program_planned_reversal_due(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const VehicleProfile*v,const GuidanceSettings*s){
    (void)p;(void)v;(void)s;
    if(!g||!t||!g->entry_reversal_scheduled||!g->has_s_turn_leg_started||
       g->has_s_turn_reversal_requested||!isfinite(g->entry_reversal_ut))return false;
    double time_resolution=sqrt(DBL_EPSILON)*fmax(1.0,fabs(t->ut));
    return t->ut+time_resolution>=g->entry_reversal_ut;
}

/*
 * The propagated first-leg timestamp is advisory.  It is deliberately durable
 * against ordinary replans, but a live setup-side crossrange proof may mature
 * before that timestamp when the measured atmosphere differs from the shadow.
 * Release only a nonfinal, genuine side change after the configured leg dwell;
 * the final fixed-point/perpendicular turn still has its separate physical
 * reachability gate in entry_program_execute_planned_reversal().
 */
static bool entry_program_release_nonfinal_reversal_now(GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->entry_reversal_scheduled||
       g->entry_reversal_is_final||!g->has_s_turn_leg_started||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed||
       !g->taem_interface_target.valid||
       g->entry_reversal_sign*g->s_turn_sign>=0.0||
       !isfinite(g->s_turn_leg_started_ut)||
       t->ut-g->s_turn_leg_started_ut<
           fmax(cfg->guidance.s_turn_minimum_leg_duration,8.0))return false;

    double built=NAN,required=NAN;
    if(!entry_taem_shaping_crossrange_ready(&g->taem_interface_target,t,p,cfg,
        g->entry_reversal_sign,&built,&required))return false;
    double distance=hypot(g->taem_interface_target.along_track-
        t->runway_along_track,g->taem_interface_target.cross_track-
        t->runway_cross_track);
    TaemHandoffContract handoff=taem_handoff_contract(&cfg->guidance);
    if(!isfinite(distance)||distance<=handoff.horizontal_radius_m)return false;

    g->entry_reversal_ut=t->ut;
    g->entry_reversal_range=distance;
    if(!g->diagnostic_shadow)fprintf(stderr,
        "MM304 setup reversal released by live crossrange: UT %.2f built %.0f/%.0f m distance %.0f m sign %+.0f.\n",
        t->ut,built,required,distance,g->entry_reversal_sign);
    return true;
}

static bool entry_program_terminal_course_endpoint_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign,
        double target_course,double*endpoint_error_out,double*radius_out){
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(radius_out)*radius_out=INFINITY;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(target_course))return false;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return false;

    double side=final_sign>=0.0?1.0:-1.0;
    double tangent_turn=norm_signed_deg(target_course-course);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(side*tangent_turn<=angular_resolution)return false;

    double usable_bank=dynamic_bank_limit(t,&cfg->vehicle);
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double measured_lift=t->mass>DBL_MIN&&isfinite(t->lift_force)&&
        t->lift_force>0.0?t->lift_force/t->mass:NAN;
    if(!(usable_bank>0.0)||!isfinite(measured_lift)||
       !(measured_lift>DBL_MIN))return false;

    double effective_bank=fmin(usable_bank*effectiveness,
        nextafter(90.0,0.0))*DEG2RAD;
    double lateral=measured_lift*fabs(sin(effective_bank));
    if(!(lateral>DBL_MIN))return false;
    double minimum_radius=t->horizontal_speed*t->horizontal_speed/lateral;
    if(!isfinite(minimum_radius)||!(minimum_radius>0.0))return false;

    double psi0=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    double psi1=norm_signed_deg(target_course-cfg->site.runway_heading)*DEG2RAD;
    double ua=(sin(psi1)-sin(psi0))/side;
    double uc=(cos(psi0)-cos(psi1))/side;
    double norm2=ua*ua+uc*uc;
    if(!isfinite(norm2)||norm2<=DBL_EPSILON)return false;

    double ba=q->along_track-t->runway_along_track;
    double bc=q->cross_track-t->runway_cross_track;
    double ideal_radius=(ba*ua+bc*uc)/norm2;
    if(!isfinite(ideal_radius)||ideal_radius<minimum_radius)return false;

    double endpoint_along=t->runway_along_track+ideal_radius*ua;
    double endpoint_cross=t->runway_cross_track+ideal_radius*uc;
    double endpoint_error=hypot(endpoint_along-q->along_track,
        endpoint_cross-q->cross_track);

    /*
     * Admission tolerance is the distance that can be accumulated while the
     * roll actuator establishes the requested bank, plus the measured
     * trajectory-range residual.  There is no radius-percentage or fixed-metre
     * capture tube here.
     */
    double target_bank=side*usable_bank;
    double response_time=roll_capture_time(t,target_bank,
        cfg->guidance.entry_roll_rate,cfg->guidance.entry_roll_acceleration);
    if(!isfinite(response_time))return false;
    double model_residual=isfinite(t->trajectory_range_residual)?
        fabs(t->trajectory_range_residual):0.0;
    double tolerance=t->horizontal_speed*response_time+model_residual;

    if(endpoint_error_out)*endpoint_error_out=endpoint_error;
    if(radius_out)*radius_out=ideal_radius;
    return endpoint_error<=tolerance;
}

/* A bank sign change is not instantaneous.  The final-arc admission point must
   therefore lead the geometric circle by the distance/course accumulated while the
   orbiter rolls through zero and establishes useful opposite bank. v27 reached a
   good ~84 km endpoint circle at ~17 km crossrange, but waiting for instantaneous
   readiness let the +50 -> - bank transition carry it to ~25 km crossrange before
   the final turn had authority. Project that actuator transient in runway coordinates
   and admit the reversal when the post-roll state, rather than the current state,
   closes the fixed MM304 endpoint. */
static bool entry_program_rollthrough_endpoint_ready(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double final_sign,
        double*endpoint_error_out,double*lead_time_out){
    (void)p;
    if(endpoint_error_out)*endpoint_error_out=INFINITY;
    if(lead_time_out)*lead_time_out=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    double lift=t->mass>DBL_MIN&&isfinite(t->lift_force)&&t->lift_force>0.0?
        t->lift_force/t->mass:NAN;
    double bank_limit=dynamic_bank_limit(t,&cfg->vehicle);
    double side=final_sign>=0.0?1.0:-1.0;
    EntryLateralTurnInput turn_input={
        .horizontal_speed_mps=t->horizontal_speed,
        .current_course_deg=norm_signed_deg(course-cfg->site.runway_heading),
        .target_course_deg=norm_signed_deg(q->course-cfg->site.runway_heading),
        .current_along_m=t->runway_along_track,
        .current_cross_m=t->runway_cross_track,
        .target_along_m=q->along_track,
        .target_cross_m=q->cross_track,
        .measured_bank_deg=norm_signed_deg(t->roll),
        .measured_bank_rate_deg_s=controlled_roll_rate(t),
        .lift_accel_mps2=lift,
        .bank_effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
            t->bank_effectiveness:1.0,
        .maximum_bank_deg=bank_limit,
        .maximum_roll_rate_deg_s=fabs(cfg->guidance.entry_roll_rate),
        .maximum_roll_accel_deg_s2=fabs(cfg->guidance.entry_roll_acceleration),
        .capture_radius_m=fmax(cfg->guidance.mm304_handoff_radius,
            cfg->guidance.hac_radius*0.10),
        .position_uncertainty_m=isfinite(t->trajectory_range_residual)?
            fabs(t->trajectory_range_residual):0.0
    };
    EntryLateralTurnEnvelope turn=
        entry_lateral_terminal_turn_envelope(&turn_input);
    if(!turn.valid||turn.turn_sign*side<=0.0)return false;
    if(endpoint_error_out)*endpoint_error_out=turn.endpoint_error_m;
    if(lead_time_out)*lead_time_out=turn.response_time_s;
    return turn.feasibility_margin_m>=0.0;
}

static bool entry_program_final_reversal_geometry_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign){
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    /* Proximity alone never proves room for a final turn. */
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return false;
    double side=final_sign>=0.0?1.0:-1.0;
    double tangent_turn=norm_signed_deg(q->course-course);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(side*tangent_turn<=angular_resolution)return false;

    /* Forecast timestamps and replay-state proximity are hints, never authority.
       The final reversal is legal only when the measured state, measured lift and
       current actuator envelope can close a single monotonic arc onto the fixed
       MM304 point/course contract. */
    return entry_program_terminal_course_endpoint_ready(g,t,cfg,final_sign,q->course,NULL,NULL);
}


static bool entry_program_terminal_geometry_blocked(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg){
    if(!g||!t||!cfg||g->entry_final_reversal_pending||g->entry_final_reversal_completed||
       !g->entry_reversal_scheduled||!isfinite(g->entry_reversal_ut)||
       t->ut+1e-6<g->entry_reversal_ut||!g->taem_interface_target.valid)return false;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!isfinite(g->taem_interface_target.course))return false;
    double next_sign=g->entry_reversal_sign>=0.0?1.0:-1.0;
    double terminal_turn=norm_signed_deg(g->taem_interface_target.course-course);
    double resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    bool terminal_side=next_sign*terminal_turn>resolution;
    return terminal_side&&!entry_program_final_reversal_geometry_ready(g,t,cfg,next_sign);
}


/* Use one chord-aware setup radius everywhere MM304 reasons about the first S-turn
   reversal. The former pose target always used the nominal TAEM/HAC radius (~25 km
   in the current 640 m/s case), while the reversal gate correctly expanded the
   required radius toward ~96 km when the shuttle was still far upstream. That
   mismatch made guidance aim for only ~7 km crossrange, declare the timer overdue,
   then demand 50-70 deg bank late to satisfy a ~25 km crossrange gate. */
static double entry_taem_shaping_setup_radius(const TaemInterfaceTarget*q,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!q||!t||!p||!cfg||!q->valid||!isfinite(q->course)||
       !isfinite(q->along_track)||!isfinite(t->runway_along_track))return NAN;
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(relative)||fabs(relative)<30.0)return NAN;
    double offset=fabs(relative)*DEG2RAD;
    double reference_speed=isfinite(q->speed)&&q->speed>0.0?q->speed:
        fmax(220.0,cfg->guidance.taem_force_handoff_speed*.65);
    double course_rate=fmax(1.0,hac_course_rate_cap_for_speed(reference_speed))*DEG2RAD;
    double nominal_radius=reference_speed/fmax(course_rate,1e-6);
    double nominal_max=fmax(cfg->guidance.hac_radius*4.0,
        cfg->guidance.taem_interface_range*1.5);
    nominal_radius=clampd(nominal_radius,cfg->guidance.hac_radius,nominal_max);
    double upstream=fmax(0.0,q->along_track-t->runway_along_track);
    double exact_radius=upstream/fmax(.10,sin(offset));
    double maximum_radius=fmax(cfg->guidance.hac_radius,
        fmin(p->radius*.20,fmax(cfg->guidance.hac_radius*8.0,
            cfg->guidance.taem_interface_range*2.5)));
    return clampd(exact_radius,nominal_radius,maximum_radius);
}
/* The first sign change only has to establish a real setup-side offset before
   MM304 begins the long, decelerating terminal turn. Live finalaoa7 showed that
   a ~6.6 km proof was not itself the failure: the controller then stopped the
   turn and flew runway-parallel for most of the remaining descent. Keep the
   first proof HAC-sized so the turn starts early; post-shaping guidance below
   now continues a bounded tangent-line lead instead of freezing the heading. */
static double entry_taem_shaping_parallel_cross_requirement(const LandingConfiguration*cfg){
    if(!cfg)return NAN;
    double required=fmax(6000.0,fmin(12000.0,cfg->guidance.hac_radius*.55));
    return isfinite(required)?required:NAN;
}
bool entry_taem_shaping_crossrange_ready(const TaemInterfaceTarget*q,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double next_sign,
        double*built_out,double*required_out){
    if(built_out)*built_out=NAN;if(required_out)*required_out=NAN;
    if(!q||!t||!p||!cfg||!q->valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return true;
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(relative)||fabs(relative)<30.0)return true;
    double final_side=relative>=0.0?1.0:-1.0;
    if(next_sign*final_side<=0.0)return true;
    double required=entry_taem_shaping_parallel_cross_requirement(cfg);
    if(!isfinite(required))return true;
    double setup_side=-final_side;
    double built=setup_side*(t->runway_cross_track-q->cross_track);
    if(built_out)*built_out=built;if(required_out)*required_out=required;
    return isfinite(built)&&built>=required;
}

static bool entry_program_supervised_reversal_ready(const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s){
    if(!g||!t||!s||!g->entry_s_turn_plan.valid||!g->entry_s_turn_plan.terminal_ready||
       !g->entry_supervision_valid||g->entry_supervision_boundary_missed||
       !isfinite(g->entry_supervision_ut))return false;
    double age=t->ut-g->entry_supervision_ut;
    double max_age=fmax(12.0,6.0*fmax(.25,s->prediction_interval));
    return isfinite(age)&&age>=0.0&&age<=max_age;
}

static bool entry_program_execute_planned_reversal(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg)return false;
    const GuidanceSettings*s=&cfg->guidance;
    if(!entry_program_planned_reversal_due(g,t,p,&cfg->vehicle,s))return false;

    double next_sign=g->entry_reversal_sign>=0.0?1.0:-1.0;
    double old_sign=g->s_turn_sign>=0.0?1.0:-1.0;
    bool genuine_side_change=next_sign*old_sign<0.0;
    bool planned_final=g->entry_reversal_is_final;

    double live_course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    double target_turn=g->taem_interface_target.valid&&
        isfinite(g->taem_interface_target.course)&&isfinite(live_course)?
        norm_signed_deg(g->taem_interface_target.course-live_course):NAN;
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    double required_terminal_side=
        isfinite(target_turn)&&fabs(target_turn)>angular_resolution?
        (target_turn>=0.0?1.0:-1.0):0.0;
    bool terminal_direction=required_terminal_side!=0.0&&
        next_sign*required_terminal_side>0.0;

    double rollout_error=INFINITY,rollout_lead=NAN;
    bool rollout_ready=terminal_direction&&
        entry_program_rollthrough_endpoint_ready(g,t,p,cfg,next_sign,
            &rollout_error,&rollout_lead);
    bool geometry_ready=terminal_direction&&
        entry_program_final_reversal_geometry_ready(g,t,cfg,next_sign);
    bool supervised_ready=terminal_direction&&
        entry_program_supervised_reversal_ready(g,t,s);
    bool terminal_feasible=rollout_ready||geometry_ready||supervised_ready;

    /*
     * A planner may propose a shaping sign change without knowing that the
     * measured state has become terminally reachable. Promote that event to the
     * final fixed-station turn only from current physics (or a fresh executable
     * shadow proof), never from reversal count or a course-error band.
     */
    bool terminal_side_event=!planned_final&&terminal_feasible;
    if(planned_final&&!g->diagnostic_shadow&&!terminal_feasible)return false;

    /*
     * A same-side event is not a physical S-turn reversal. It has meaning only
     * when it is the terminal turn; otherwise leave ownership unchanged and let
     * the planner replace the invalid event instead of inventing a pseudo-turn.
     */
    if(!planned_final&&!genuine_side_change&&!terminal_side_event)return false;

    if(!g->diagnostic_shadow&&terminal_feasible){
        fprintf(stderr,
            "MM304 terminal turn admitted by reachability: UT %.2f sign %+.0f endpoint=%d response=%d lead %.2fs error %.0f m shadow=%d.\n",
            t->ut,next_sign,geometry_ready?1:0,rollout_ready?1:0,
            rollout_lead,rollout_error,supervised_ready?1:0);
    }

    bool final_reversal=planned_final||terminal_side_event;
    EntryControlPlan previous_plan=g->entry_s_turn_plan;
    double reversal_bank=fmax(fabs(g->entry_control_bank),g->entry_reversal_bank);
    request_side(g,next_sign,t->ut);
    if(isfinite(reversal_bank)&&reversal_bank>DBL_EPSILON)
        g->entry_control_bank=next_sign*reversal_bank;
    if(final_reversal){
        /*
         * Terminal-turn ownership persists through roll/capture. The retained
         * child carries lineage and commands only; the fixed MM304 handoff
         * contract still decides when MM305 may take ownership.
         */
        g->entry_final_reversal_pending=true;
        g->entry_final_reversal_completed=false;
        g->entry_topology_heading_locked=false;
        EntryControlPlan capture_plan=previous_plan;
        capture_plan.valid=true;
        capture_plan.plan_id=0;capture_plan.plan_version=0;
        capture_plan.parent_plan_id=0;capture_plan.parent_plan_version=0;
        capture_plan.planned_ut=t->ut;
        capture_plan.segment_duration=fmax(2.0,s->prediction_interval*2.0);
        capture_plan.target_bank=g->entry_control_bank;
        if(!isfinite(capture_plan.target_aoa)||capture_plan.target_aoa<=0.0)
            capture_plan.target_aoa=fmax(g->entry_control_aoa,
                entry_thermal_protection_aoa_floor(&cfg->vehicle));
        capture_plan.target_heading=g->taem_interface_target.valid&&
            isfinite(g->taem_interface_target.course)?
            g->taem_interface_target.course:
            (isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading);
        capture_plan.has_planned_reversal=false;
        capture_plan.planned_reversal_is_final=false;
        capture_plan.planned_reversal_ut=NAN;capture_plan.planned_reversal_range=NAN;
        capture_plan.planned_reversal_sign=0.0;
        capture_plan.final_heading_lock=true;
        control_plan_assign_lineage(g,&capture_plan,previous_plan.plan_id?&previous_plan:NULL);
        g->entry_s_turn_plan=capture_plan;
    }else{
        g->entry_s_turn_plan.valid=false;
    }
    g->entry_control_segment_until_ut=t->ut;
    g->entry_reversal_scheduled=false;
    g->entry_reversal_is_final=false;
    if(genuine_side_change){
        g->entry_control_reversals++;
        g->entry_continuation_bootstrap=false;
    }
    return true;
}


/* Safety-envelope replans may resize the current bank/AoA command, but they
   are not evidence that an already accepted reversal deadline became unsafe.
   Keep anti-procrastination active for those replans: normal commit logic may
   still pull the same-side reversal earlier. Only an invalid supervisory
   trajectory may discard the committed milestone and install a later one. */
static bool entry_program_replan_may_discard_reversal(bool plan_safety_replan,
        bool supervision_replan,bool supervision_valid,bool terminal_geometry_blocked){
    (void)plan_safety_replan;
    return !terminal_geometry_blocked&&supervision_replan&&!supervision_valid;
}

/* Routine model checkpoints may resize bank/AoA, but they cannot silently postpone
   an already accepted reversal. A materially infeasible-plan replan may replace it. */
static void entry_program_commit_planned_reversal(GuidanceMachine*g,EntryControlPlan*plan,
        double ut,bool replace_existing){
    if(!g||!plan)return;
    bool executable_leg=g->has_s_turn_leg_started||
        (isfinite(plan->target_bank)&&fabs(plan->target_bank)>=4.0);
    bool candidate=plan->valid&&plan->has_planned_reversal&&executable_leg&&
        isfinite(plan->planned_reversal_ut)&&plan->planned_reversal_ut>=ut-1e-6&&
        fabs(plan->planned_reversal_sign)>.1;
    if(replace_existing){
        g->entry_reversal_scheduled=false;g->entry_reversal_is_final=false;
    }
    if(candidate){
        double sign=plan->planned_reversal_sign>=0?1.0:-1.0;
        double event=fmax(ut,plan->planned_reversal_ut);
        bool accept=!g->entry_reversal_scheduled||
            (g->entry_reversal_sign*sign>=0.0&&event<g->entry_reversal_ut-1e-6&&
             (!g->entry_reversal_is_final||plan->planned_reversal_is_final));
        if(accept){
            g->entry_reversal_scheduled=true;
            g->entry_reversal_is_final=plan->planned_reversal_is_final;
            g->entry_reversal_ut=event;
            g->entry_reversal_range=plan->planned_reversal_range;
            g->entry_reversal_sign=sign;
            g->entry_reversal_bank=fabs(plan->target_bank);
        }
    }
    /* The persistent plan is the executable contract exposed to logging and
       predictor replay.  If anti-procrastination retained an older deadline,
       overwrite the rejected proposal's event metadata with that effective
       durable event; candidate trace still preserves the raw proposal. */
    plan->has_planned_reversal=g->entry_reversal_scheduled;
    plan->planned_reversal_is_final=g->entry_reversal_scheduled&&g->entry_reversal_is_final;
    if(g->entry_reversal_scheduled){
        plan->planned_reversal_ut=g->entry_reversal_ut;
        plan->planned_reversal_range=g->entry_reversal_range;
        plan->planned_reversal_sign=g->entry_reversal_sign;
        if(plan->predicted_reversals<1u)plan->predicted_reversals=1u;
    }else{
        plan->planned_reversal_ut=NAN;plan->planned_reversal_range=NAN;
        plan->planned_reversal_sign=0.0;
    }
}

/* A refreshed advisory target keeps the same fixed inlet geometry while its
 * arrival ETA is recomputed from live speed and authority.  Worker plans must
 * reject a changed inlet or energy qualification, but not a timestamp-only
 * refresh of that same target. */
static bool entry_interface_target_plan_identity_equal(
        const TaemInterfaceTarget*a,const TaemInterfaceTarget*b){
    if(!a||!b||a->valid!=b->valid||a->energy_qualified!=b->energy_qualified)
        return false;
    if(!a->valid)return true;
    return a->along_track==b->along_track&&a->cross_track==b->cross_track&&
        a->course==b->course&&a->altitude==b->altitude&&a->speed==b->speed&&
        a->flight_path_angle==b->flight_path_angle;
}

/* Only plans cross the worker boundary. Never restore cloned executive, PID,
   limiter, capture, or reversal-execution state over newer live telemetry. */
bool guidance_accept_entry_plan(GuidanceMachine*g,const GuidanceMachine*request,
        const GuidanceMachine*result,const Telemetry*t,const LandingConfiguration*cfg){
    if(!g||!request||!result||!t||!cfg)return false;
    bool replaceable_ordinary_reversal=g->entry_reversal_scheduled&&
        request->entry_reversal_scheduled&&!g->entry_reversal_is_final&&
        !request->entry_reversal_is_final&&g->entry_control_reversals==0&&
        request->entry_control_reversals==0&&!g->entry_final_reversal_pending&&
        !g->entry_final_reversal_completed&&
        result->entry_topology.first_sign*g->s_turn_sign>0.0;
    bool topology_install=!g->entry_topology.valid&&!request->entry_topology.valid&&
        result->entry_topology.valid&&g->entry_control_reversals==0&&
        request->entry_control_reversals==0&&
        (!g->entry_reversal_scheduled||replaceable_ordinary_reversal);
    if(!result->entry_s_turn_plan.valid||
       result->phase!=request->phase||result->entry_final_reversal_pending||
       result->entry_final_reversal_completed||
       !g->automation_engaged||g->paused||g->aborted||g->phase!=PHASE_ENTRY_ENERGY||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed||
       g->control_plan_sequence!=request->control_plan_sequence||
       g->s_turn_sign!=request->s_turn_sign||
       (!topology_install&&result->s_turn_sign!=request->s_turn_sign)||
       g->entry_control_reversals!=request->entry_control_reversals||
       g->entry_reversal_scheduled!=request->entry_reversal_scheduled||
       (g->entry_reversal_scheduled&&(g->entry_reversal_ut!=request->entry_reversal_ut||
        g->entry_reversal_sign!=request->entry_reversal_sign||
        g->entry_reversal_is_final!=request->entry_reversal_is_final))||
       !entry_interface_target_plan_identity_equal(&g->taem_interface_target,
           &request->taem_interface_target))
        return false;
    double age=t->ut-request->previous_ut;
    const EntryControlPlan*p=&result->entry_s_turn_plan;
    bool retained_event=g->entry_reversal_scheduled&&p->has_planned_reversal&&
        p->planned_reversal_ut==g->entry_reversal_ut&&
        p->planned_reversal_sign==g->entry_reversal_sign&&
        p->planned_reversal_is_final==g->entry_reversal_is_final;
    if(p->has_planned_reversal&&(!isfinite(p->planned_reversal_ut)||
       !isfinite(p->planned_reversal_sign)||fabs(p->planned_reversal_sign)<.1))return false;
    bool first_final_event=!g->entry_reversal_scheduled&&p->has_planned_reversal&&
        p->planned_reversal_is_final;
    bool strip_unproven_final=first_final_event&&!topology_install&&
        (!p->terminal_ready||!result->entry_supervision_valid||result->entry_supervision_boundary_missed);
    /* An unproven terminal reversal must not become executable, but rejecting the
       entire propagated result also throws away a useful current-leg bank/AoA plan
       and leaves MM304 stuck on the expired five-second bootstrap. Accept the
       propagated current segment while stripping only the unproven reversal. */
    /* The full Entry shadow can legitimately take several seconds. A fixed 3 s age
       gate rejected every high-altitude MM304 proposal even though the live phase,
       side, plan lineage and fixed TAEM target were unchanged. Freshness is already
       bounded below by the request/live identity checks and below by the executable
       segment/reversal deadlines; reject only negative/non-finite age here. */
    if(!isfinite(age)||age<0.0||
       !isfinite(p->planned_ut)||!isfinite(p->segment_duration)||
       !isfinite(p->target_bank)||!isfinite(p->target_aoa)||
       p->planned_ut>t->ut||p->segment_duration<=0.0||
       t->ut>=p->planned_ut+p->segment_duration||
       (p->has_planned_reversal&&p->planned_reversal_ut<=t->ut&&!retained_event))return false;
    if(g->entry_reversal_scheduled&&!topology_install&&!request->entry_committed_infeasible&&
       (!p->has_planned_reversal||p->planned_reversal_ut!=g->entry_reversal_ut||
        p->planned_reversal_sign!=g->entry_reversal_sign||
        p->planned_reversal_is_final!=g->entry_reversal_is_final))return false;
    if(topology_install&&(!result->entry_topology.inlet.valid||
       result->entry_topology.reversal_ut<=t->ut||
       !isfinite(result->entry_topology.glide_reserve)||result->entry_topology.glide_reserve<0.0))return false;
    EntryControlPlan accepted=*p;
    accepted.plan_id=0;accepted.plan_version=0;
    if(strip_unproven_final){
        accepted.has_planned_reversal=false;
        accepted.planned_reversal_is_final=false;
        accepted.final_heading_lock=false;
        accepted.planned_reversal_ut=NAN;
        accepted.planned_reversal_range=NAN;
        accepted.planned_reversal_sign=0.0;
        accepted.terminal_ready=false;
        accepted.predicted_reversals=g->entry_control_reversals;
    }
    control_plan_assign_lineage(g,&accepted,&g->entry_s_turn_plan);
    if(topology_install){
        g->entry_topology=result->entry_topology;
        g->taem_interface_target=result->entry_topology.inlet;
        g->entry_topology_capture_good_duration=0.0;
        g->entry_topology_heading_locked=false;
        request_side(g,result->entry_topology.first_sign,t->ut);
    }
    /* No failed candidate or long-horizon miss revokes a durable event. An overdue
       terminal event blocked only by measured final-arc geometry is especially
       durable: keep the event so live guidance can continue building that geometry. */
    bool replace_committed=(topology_install&&g->entry_reversal_scheduled)||
        (request->entry_committed_infeasible&&
         !entry_program_terminal_geometry_blocked(g,t,cfg));
    entry_program_commit_planned_reversal(g,&accepted,t->ut,replace_committed);
    g->entry_s_turn_plan=accepted;
    g->entry_supervision_ut=request->previous_ut;
    g->entry_supervision_bank_correction=0.0;g->entry_supervision_aoa_correction=0.0;
    g->entry_supervision_valid=result->entry_supervision_valid;
    g->entry_supervision_mode=result->entry_supervision_mode;
    g->entry_committed_infeasible=result->entry_committed_infeasible;
    g->entry_supervision_boundary_missed=result->entry_supervision_boundary_missed;
    g->entry_planning_needed=false;
    if(topology_install&&!g->diagnostic_shadow)fprintf(stderr,
        "Entry topology ADOPTED: age %.2fs plan %llu/%llu remaining %.1fs first side %+.0f bank %.1f/%.1f reversal UT %.2f in %.1fs inlet %.0f %.0f course %.1f h %.0f V %.1f reserve %.0f\n",
        age,(unsigned long long)accepted.plan_id,(unsigned long long)accepted.plan_version,
        accepted.planned_ut+accepted.segment_duration-t->ut,g->entry_topology.first_sign,
        g->entry_topology.first_bank,g->entry_topology.turn_bank,g->entry_reversal_ut,
        g->entry_reversal_ut-t->ut,g->taem_interface_target.along_track,g->taem_interface_target.cross_track,
        g->taem_interface_target.course,g->taem_interface_target.altitude,g->taem_interface_target.speed,
        g->entry_topology.glide_reserve);
    return true;
}

static bool entry_program_first_segment(const GuidanceMachine*g){
    bool initial_entry=g&&(!g->entry_exec.initialized||g->entry_exec.phase==ENTRY_PHASE_PREENTRY);
    return initial_entry&&!g->entry_s_turn_plan.valid&&!g->has_s_turn_leg_started&&
        !g->entry_reversal_scheduled&&g->entry_control_reversals==0;
}

static void entry_program_seed_initial_side(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg){
    if(!entry_program_first_segment(g))return;
    /*
     * The published perpendicular course is the final inlet side, not
     * necessarily the side of the first energy-management leg.  When the
     * vehicle is upstream and its point bearing is closer to the runway
     * tangent than to the required inlet course, a direct turn cannot satisfy
     * both the fixed point and fixed course.  The executable topology therefore
     * starts on the opposite side and must later reverse into the inlet.
     *
     * Keep these two pieces of state separate: entry_target_side is the stable
     * final-course identity used by the target publisher, while s_turn_sign is
     * the currently owned aerodynamic leg.  The old code latched both from a
     * tiny point-bearing residual, which could erase the setup leg when the
     * vehicle was nearly on the runway tangent.
     */
    double sign=1.0;
    double target_side=1.0;
    if(t&&cfg&&g->taem_interface_target.valid&&
       isfinite(t->runway_along_track)&&isfinite(t->runway_cross_track)){
        double course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        double relative_course=norm_signed_deg(course-cfg->site.runway_heading);
        double target_course=norm_signed_deg(
            g->taem_interface_target.course-cfg->site.runway_heading);
        TaemHandoffContract handoff=taem_handoff_contract(&cfg->guidance);
        double target_distance=hypot(
            g->taem_interface_target.along_track-t->runway_along_track,
            g->taem_interface_target.cross_track-t->runway_cross_track);
        double target_course_error=fabs(norm_signed_deg(
            g->taem_interface_target.course-course));
        double final_turn=norm_signed_deg(target_course);
        double final_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        if(fabs(final_turn)>final_resolution)
            target_side=final_turn>=0.0?1.0:-1.0;

        bool inside_handoff=isfinite(target_distance)&&
            target_distance<=handoff.horizontal_radius_m&&
            isfinite(target_course_error)&&
            target_course_error<=handoff.perpendicular_heading_half_width_deg;
        double da=g->taem_interface_target.along_track-t->runway_along_track;
        double dc=g->taem_interface_target.cross_track-t->runway_cross_track;
        double point_course=norm_deg(cfg->site.runway_heading+
            atan2(dc,da)*RAD2DEG);
        double point_turn=fabs(norm_signed_deg(point_course-course));
        double inlet_turn=fabs(norm_signed_deg(
            g->taem_interface_target.course-course));
        bool perpendicular=fabs(fabs(final_turn)-90.0)<=
            handoff.perpendicular_heading_half_width_deg;
        bool upstream=isfinite(da)&&da>0.0;
        bool setup_required=!inside_handoff&&perpendicular&&upstream&&
            isfinite(point_turn)&&isfinite(inlet_turn)&&point_turn<inlet_turn;
        if(setup_required&&fabs(target_side)>DBL_EPSILON){
            /* A fixed-pose perpendicular inlet needs the opposite setup leg. */
            sign=-target_side;
        }else{
            double geometry_sign=entry_lateral_geometry_turn_sign(
                relative_course,
                t->runway_along_track,t->runway_cross_track,
                g->taem_interface_target.along_track,
                g->taem_interface_target.cross_track,
                target_course);
            if(fabs(geometry_sign)>DBL_EPSILON)sign=geometry_sign;
            if(inside_handoff&&fabs(target_side)>DBL_EPSILON)
                sign=target_side;
        }
    }
    g->s_turn_sign=sign;
    g->entry_target_side=target_side;
    g->entry_target_side_latched=true;
    g->has_s_turn_reversal_requested=false;
}

static EntryControlPlan entry_program_plan_s_turn(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double nominal_bank_magnitude,double geometry_bank_magnitude,double nominal_aoa){
    EntryControlPlan plan={0};plan.cost=INFINITY;
    if(!g||!t||!state||!p||!cfg)return plan;
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    AerodynamicEnvelope fallback_env;TrajectoryCalibrationModel fallback_cal;
    const AerodynamicEnvelope*env=&g->entry_predictor_envelope;
    const TrajectoryCalibrationModel*cal=&g->entry_predictor_calibration;
    if(!g->entry_predictor_models_valid){
        entry_supervision_models(t,aero,&fallback_env,&fallback_cal);
        env=&fallback_env;cal=&fallback_cal;
    }
    double roll_rate=controlled_roll_rate(t);
    double aoa_rate=controlled_aoa_rate(t);
    double leg_elapsed=g->has_s_turn_leg_started?fmax(0.0,t->ut-g->s_turn_leg_started_ut):0.0;
    bool first_segment=entry_program_first_segment(g);
    /* The tangent-gate selector has already chosen the mission-relative first
       side.  Do not overwrite it with the historical canonical +bank seed. */
    double planning_sign=g->s_turn_sign>=0.0?1.0:-1.0;
    bool side_locked=first_segment||g->entry_s_turn_plan.valid||g->has_s_turn_leg_started||
        g->entry_reversal_scheduled||g->entry_control_reversals>0;
    const TaemInterfaceTarget*interface_target=g->taem_interface_target.valid?&g->taem_interface_target:NULL;
    plan=predictor_plan_entry_control_to_interface(*state,p,aero,env,cal,v,&cfg->site,s,interface_target,true,
        norm_signed_deg(t->roll),roll_rate,t->angle_of_attack,aoa_rate,
        planning_sign,g->has_s_turn_leg_started,leg_elapsed,0.0,
        side_locked,false,1200.0);
    if(plan.valid){
        /* The legacy search still provides trajectory topology (initial side, heading,
           segment horizon).  Bank magnitude and alpha, however, are owned by the
           MM304 DRAGREF/ENTRYLATERAL/@ALPHA modules.  Seed the persistent segment
           from those nominal commands before any vertical shaping or forecast so a
           legacy free-alpha candidate cannot bypass a live stall-safe alpha limit. */
        double selected_sign=fabs(plan.target_bank)>=1.0?
            (plan.target_bank>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
        double seeded_bank=clampd(fabs(nominal_bank_magnitude),0.0,dynamic_bank_limit(t,v));
        plan.target_bank=selected_sign*seeded_bank;
        plan.target_aoa=clampd(fmax(nominal_aoa,entry_thermal_protection_aoa_floor(v)),0.0,v->maximum_angle_of_attack);
        plan.bank_cap=fabs(plan.target_bank);
        plan.target_turn_radius=live_turn_radius(t,aero,v,plan.target_bank);

        /* TAEM owns HAC/path topology only after handoff. MM304 still owns the
           terminal S-turn exit and must complete the selected HAC-inlet heading
           lock before relinquishing the aircraft. The propagated terminal solution
           may score the path, but it cannot replace this Entry-owned tangent lock. */
        plan.final_heading_lock=false;
        plan.planned_reversal_is_final=false;
        (void)entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&plan);
        entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank_magnitude,&plan);

        /* Reforecast the locally shaped seed to propose reversal topology. This
           constant-command surrogate is not a terminal-feasibility certificate;
           the worker subsequently replays the evolving executable guidance law. */
        plan.segment_duration=fmax(entry_s_turn_effective_minimum_leg(t->true_air_speed,
            entry_taem_speed_target(v,s,p),s),plan.segment_duration);
        plan.has_planned_reversal=false;
        plan.planned_reversal_is_final=false;
        plan.planned_reversal_ut=NAN;
        plan.planned_reversal_range=NAN;
        plan.planned_reversal_sign=0.0;
        GuidanceMachine event_contract=*g;
        if(g->entry_committed_infeasible){
            event_contract.entry_reversal_scheduled=false;event_contract.entry_reversal_is_final=false;
        }
        entry_program_commit_planned_reversal(&event_contract,&plan,t->ut,false);
        EntryPrediction shaped=predictor_simulate_entry_control_plan_to_interface(*state,p,aero,env,cal,v,
            &cfg->site,s,interface_target,true,norm_signed_deg(t->roll),roll_rate,t->angle_of_attack,aoa_rate,
            g->s_turn_sign,leg_elapsed,&plan,1200.0,false);
        double capture_hint=fmax(s->hac_radius*8.0,s->taem_interface_range*2.5);
        if(shaped.entered_atmosphere&&isfinite(shaped.closest_distance)){
            /* Legacy topology search is advisory; only executable shadow can certify it. */
            plan.terminal_ready=false;
            plan.taem_range_error=shaped.reached_taem?shaped.taem_range_error:
                shaped.closest_distance-capture_hint;
            plan.taem_speed=shaped.reached_taem?shaped.taem_speed:NAN;
            plan.taem_energy_error=shaped.reached_taem?shaped.taem_energy_error:NAN;
            plan.closest_distance=shaped.closest_distance;
            plan.predicted_reversals=shaped.s_turn_reversals;
            if(shaped.has_first_s_turn_reversal){
                double shaped_sign=fabs(plan.target_bank)>=1.0?
                    (plan.target_bank>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
                plan.has_planned_reversal=true;
                plan.planned_reversal_ut=shaped.first_s_turn_reversal_ut;
                plan.planned_reversal_range=shaped.first_s_turn_reversal_range;
                plan.planned_reversal_sign=fabs(shaped.first_s_turn_reversal_sign)>.1?
                    shaped.first_s_turn_reversal_sign:-shaped_sign;
                /*
                 * Local propagation proposes a shaping event only. Final
                 * classification belongs to live reachability at execution,
                 * using the fixed-station terminal-turn envelope. Reversal
                 * count and course-debt bands are not reachability evidence.
                 */
                plan.planned_reversal_is_final=false;
                plan.final_heading_lock=false;
            }
        }
        entry_prediction_clear(&shaped);
    }
    return plan;
}

static bool entry_supervision_replan_due(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    if(!g||!t||!s||!g->entry_s_turn_plan.valid||
       !isfinite(g->entry_supervision_ut)||!isfinite(g->entry_s_turn_plan.planned_ut))return false;
    /* Keep a future handoff-window miss observable even while the current segment
       remains safe. A miss reached inside the executable segment is an immediate
       invalidation. A farther-out warning is timed from the persistent executable
       plan, not from the supervisor refresh timestamp: entry_supervision_ut is
       refreshed every prediction_interval and therefore cannot be the dwell clock. */
    if(g->entry_supervision_boundary_missed){
        /* A proven miss invalidates an established plan promptly, but a replacement
           that is rejected again in the same tick must not spawn another child every
           control frame. Use the same age-based dwell as ordinary infeasible
           supervision so a fresh child gets one bounded execution window. */
        if(!g->entry_supervision_valid&&g->entry_supervision_mode==ENTRY_SUPERVISION_INFEASIBLE)
            return t->ut-g->entry_s_turn_plan.planned_ut>=
                fmax(5.0,s->prediction_interval*2.0);
        if(g->entry_supervision_valid)
            return t->ut-g->entry_s_turn_plan.planned_ut>=
                fmax(s->s_turn_minimum_leg_duration,s->prediction_interval);
        return false;
    }
    if(g->entry_supervision_valid||g->entry_supervision_mode!=ENTRY_SUPERVISION_INFEASIBLE)return false;
    return t->ut-g->entry_s_turn_plan.planned_ut>=fmax(5.0,s->prediction_interval*2.0);
}

/* A segment can be committed while thin-air/stall guards correctly deny usable
   bank authority. Once that physical authority first becomes available, the
   pre-leg segment may be reshaped immediately, but the child must not postpone
   the parent's already-scheduled ordinary refresh. Latch the acquisition so
   transient stall chatter cannot turn this into another level-triggered loop. */
static bool entry_bank_authority_replan_due(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v){
    if(!g||!t||!v)return false;
    bool available=entry_s_turn_bank_authority_available(t->dynamic_pressure,
        t->true_air_speed,t->stall_fraction,t->g_force,v);
    if(!available||g->entry_bank_authority_acquired)return false;
    g->entry_bank_authority_acquired=true;
    if(!g->entry_s_turn_plan.valid||!g->entry_control_plan_valid||g->has_s_turn_leg_started)
        return false;
    double parent_end=g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration;
    /* If the parent is already about to refresh, let normal expiry own the next
       solve rather than manufacturing a nearly-zero-lived child. */
    if(isfinite(parent_end)&&parent_end-t->ut<=1.0)return false;
    return true;
}

static double entry_bank_authority_child_duration(const EntryControlPlan*parent,
        double child_ut,double proposed_duration){
    double duration=fmax(0.0,proposed_duration);
    if(!parent||!parent->valid||!isfinite(parent->planned_ut)||
       !isfinite(parent->segment_duration)||!isfinite(child_ut))return duration;
    double parent_end=parent->planned_ut+parent->segment_duration;
    if(!isfinite(parent_end))return duration;
    return fmin(duration,fmax(0.0,parent_end-child_ut));
}

/* Bank authority grows quickly after the first useful-q edge. A persistent segment
   solved near that edge can become badly under-banked tens of seconds before its
   ordinary expiry even though the live vehicle has substantially more authority.
   Re-use the existing vertical-capture law as the demand oracle and only refresh
   when the newly executable same-side bank exceeds the committed demand by a
   material margin. The age gate makes this self-quenching rather than per-frame. */
static bool entry_vertical_capture_growth_replan_due(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->entry_s_turn_plan.valid||!g->entry_control_plan_valid)
        return false;
    const GuidanceSettings*s=&cfg->guidance;
    double age=t->ut-g->entry_s_turn_plan.planned_ut;
    double parent_end=g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    double refresh_period=fmax(fmax(0.0,s->prediction_interval),
        authority.valid?authority.control_response_time_s:0.0);
    if(!isfinite(age)||age+sqrt(DBL_EPSILON)<refresh_period||
       !isfinite(parent_end)||parent_end<=t->ut)
        return false;
    if(!entry_program_bank_capture_available(g,t,p,cfg))return false;

    double demanded_bank=g->entry_s_turn_plan.target_bank;
    double demanded_aoa=g->entry_s_turn_plan.target_aoa;
    GuidanceMachine probe=*g;
    if(!entry_program_altitude_capture(&probe,t,p,aero,cfg,
        &demanded_bank,&demanded_aoa))return false;
    double committed=fabs(g->entry_s_turn_plan.target_bank);
    double demanded=fabs(demanded_bank);
    double resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(committed,demanded));
    return demanded>committed+resolution;
}


static double entry_authority_refresh_child_duration(const GuidanceMachine*g,
        const EntryControlPlan*parent,double child_ut,double proposed_duration,
        const GuidanceSettings*s){
    double duration=entry_bank_authority_child_duration(parent,child_ut,proposed_duration);
    if(!g||!s||!g->entry_reversal_scheduled||!g->has_s_turn_leg_started||
       g->has_s_turn_reversal_requested||!isfinite(g->entry_reversal_ut))return duration;
    double executable_reversal=fmax(g->entry_reversal_ut,
        g->s_turn_leg_started_ut+s->s_turn_minimum_leg_duration);
    if(executable_reversal<=child_ut+1e-6)return duration;
    return fmin(duration,fmax(0.0,executable_reversal-child_ut));
}


/* The MM304 outlet is a pose constraint, not merely a waypoint. Before the final
   reversal, aim the owned S-turn leg at the start of the nominal terminal circle
   that would finish at the fixed TAEM point/course. This converts the required
   crossrange into an explicit setup target instead of relying on a fixed bearing
   lead. The live final-reversal gate still proves the measured-radius arc. */
static bool entry_taem_pose_setup_point(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double*setup_along,double*setup_cross,double*final_side){
    if(setup_along)*setup_along=NAN;if(setup_cross)*setup_cross=NAN;if(final_side)*final_side=0.0;
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!isfinite(q->course))return false;
    double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(fabs(target_relative)<30.0)return false;
    double side=target_relative>=0.0?1.0:-1.0;
    double radius=entry_taem_shaping_setup_radius(q,t,p,cfg);
    if(!isfinite(radius)||radius<1000.0)return false;
    double psi0=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    double psi1=target_relative*DEG2RAD;
    double along=q->along_track-radius/side*(sin(psi1)-sin(psi0));
    double cross=q->cross_track-radius/side*(cos(psi0)-cos(psi1));
    if(!isfinite(along)||!isfinite(cross))return false;
    if(setup_along)*setup_along=along;if(setup_cross)*setup_cross=cross;if(final_side)*final_side=side;
    return true;
}

static bool entry_taem_tangent_target_geometry(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg){
    if(!q||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->speed)||q->speed<=0.0)return false;
    const GuidanceSettings*s=&cfg->guidance;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    double station=-s->final_approach_distance;
    double point_error=hypot(q->along_track-station,q->cross_track);
    double runway_offset=fabs(norm_signed_deg(q->course-cfg->site.runway_heading));
    double perpendicular_error=fabs(runway_offset-90.0); /* decision-literal-ok: perpendicular runway geometry */
    return q->along_track<0.0&&isfinite(point_error)&&
        point_error<=contract.horizontal_radius_m&&
        perpendicular_error<=contract.perpendicular_heading_half_width_deg;
}

static void entry_taem_turnability_target(const GuidanceMachine*g,const Telemetry*t,
        double reference_course,AerodynamicModel aero,const LandingConfiguration*cfg,
        double ordinary_target,double*target_speed,double*target_course){
    if(target_speed)*target_speed=ordinary_target;if(target_course)*target_course=NAN;
    if(!g||!t||!cfg||!isfinite(reference_course))return;
    (void)reference_course;(void)aero;
    /* Publish the centerline of the explicit perpendicular heading contract.
       Acceptance width remains solely in TaemHandoffContract. */
    double side=g->entry_target_side_latched&&fabs(g->entry_target_side)>DBL_EPSILON?
        (g->entry_target_side>=0.0?1.0:-1.0):
        (g->s_turn_sign>=0.0?-1.0:1.0);
    const double perpendicular=90.0; /* decision-literal-ok: perpendicular runway geometry */
    /* Both perpendicular outlets are legal MM304 states: their acceptance
       sectors are 330..030 and 150..210.  Before the side is latched, choose
       the outlet requiring the smaller measured course change.  This keeps an
       initially southbound vehicle on the southbound legal inlet instead of
       commanding a 180 degree reversal merely because the opposite side was
       the historical default.  Preserve the existing side only for the exact
       runway-tangent tie; the latch then makes the choice one-way. */
    if(!g->entry_target_side_latched){
        double live=norm_deg(reference_course);
        double positive=norm_deg(cfg->site.runway_heading+perpendicular);
        double negative=norm_deg(cfg->site.runway_heading-perpendicular);
        double positive_error=fabs(norm_signed_deg(positive-live));
        double negative_error=fabs(norm_signed_deg(negative-live));
        double resolution=sqrt(DBL_EPSILON)*fmax(1.0,
            fmax(positive_error,negative_error));
        if(positive_error<negative_error-resolution)side=1.0;
        else if(negative_error<positive_error-resolution)side=-1.0;
    }
    /* The owned side is also the signed bank/course-rate side in the runway
       frame: positive bank produces positive cross-track course rate. */
    if(target_course)*target_course=norm_deg(cfg->site.runway_heading+side*perpendicular);
}

void entry_publish_taem_tangent_target(GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||g->terminal_region_entered)return;
    /* Keep a geometry-only candidate refreshable.  Its fixed inlet remains the
       same, but ETA and energy qualification are state-dependent; freezing the
       first high-altitude projection preserved a billion-metre turn radius all
       the way into the atmosphere. */
    if(entry_taem_tangent_target_geometry(&g->taem_interface_target,cfg)&&
       g->taem_interface_target.energy_qualified)return;

    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double reference_course=isfinite(course)?norm_deg(course):
        (isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading));
    if(!isfinite(reference_course)||!(t->horizontal_speed>DBL_MIN))return;

    double target_course=NAN;
    double compatibility_speed=entry_taem_speed_target(v,s,p);
    entry_taem_turnability_target(g,t,reference_course,aero,cfg,compatibility_speed,
        NULL,&target_course);
    if(!isfinite(target_course))return;

    double slope=clampd(fabs(s->taem_glide_slope),
        DBL_EPSILON,nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
    double target_fpa=-slope;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!isfinite(authority.control_response_time_s)||
       authority.control_response_time_s<0.0)return;
    double response=authority.control_response_time_s;

    double station=-s->final_approach_distance;
    double final_altitude=cfg->site.altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);

    /*
     * MM304 owns a configured interface altitude.  Do not move that ownership
     * surface upward merely because a low-density instantaneous turn-radius
     * estimate grows; downstream maneuver feasibility decides whether the live
     * vehicle may actually cross the boundary.
     */
    double target_altitude=fmax(s->taem_interface_altitude,final_altitude);
    TaemSpeedEnvelope speed_envelope=
        decision_taem_speed_envelope(v,p,target_altitude);
    if(!speed_envelope.valid)return;
    double terminal_radius=fmax(s->hac_radius,
        speed_envelope.minimum_turn_radius_m);

    double rh=cfg->site.runway_heading*DEG2RAD;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint point=local_point(origin,station*sin(rh),station*cos(rh),
        p->radius,target_altitude);

    HandoffSetEnvelope handoff_set=decision_mm304_handoff_set_envelope(
        g,t,reference_course,station,0.0,p,cfg);
    double minimum_turn_radius=handoff_set.minimum_turn_radius_m;
    double approach_path=handoff_set.path_to_set_m;
    const char*decision_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    bool decision_diag_on=decision_diag&&strcmp(decision_diag,"1")==0;
    if(!handoff_set.valid||!isfinite(approach_path)){
        if(decision_diag_on)fprintf(stderr,
            "MM304 target reject: handoff-set geometry invalid at UT %.2f course %.1f.\n",
            t->ut,reference_course);
        return;
    }
    double remaining=s->final_approach_distance+
        .5*LANDER_PI*terminal_radius; /* decision-literal-ok: half-circle path geometry */
    double distance=hypot(station-t->runway_along_track,t->runway_cross_track);
    UnpoweredPathProjection arrival=decision_unpowered_path_projection(
        g,t,p,cfg,target_altitude,approach_path);
    if(!arrival.valid){
        if(decision_diag_on)fprintf(stderr,
            "MM304 target reject: energy projection invalid at UT %.2f path %.0f Rmin %.0f.\n",
            t->ut,approach_path,minimum_turn_radius);
        return;
    }

    /*
     * A fixed-point perpendicular inlet has two independent facts:
     *
     *   1. its geometry can provide useful acquisition demand before the
     *      unpowered speed projection is certifiable; and
     *   2. its arrival speed/energy can certify MM304 ownership only after the
     *      same projection reaches the structural envelope.
     *
     * The old binary gate discarded (1) whenever (2) was not yet true.  The
     * vehicle then remained wings-level until it had crossed the point that
     * the geometry controller needed to approach.  Publish a geometry target
     * with a finite geometric ETA, but mark it energy-unqualified.  The live
     * capture contract still recomputes downstream energy and therefore cannot
     * accept this candidate as MM304 ownership.
     */
    bool energy_qualified=arrival.valid&&
        arrival.terminal_speed_mps>=speed_envelope.minimum_speed_mps;
    double target_speed=energy_qualified?
        fmin(speed_envelope.maximum_speed_mps,arrival.terminal_speed_mps):
        clampd(compatibility_speed,speed_envelope.minimum_speed_mps,
            speed_envelope.maximum_speed_mps);
    /*
     * A geometry-only target is an acquisition preview, not a certification of
     * the exact fixed-pose turn.  At the upper atmosphere the live lateral
     * authority can make handoff_set.path_to_set_m enormous (the instantaneous
     * minimum-turn radius tends toward infinity), even though the vehicle still
     * has a finite forward distance to the handoff station and a finite terminal
     * HAC reserve after that station.  Feeding that conservative proof path into
     * arrival_ut makes the preview effectively millions of seconds away and
     * contaminates the first-leg planner.  Keep the strict capture veto on the
     * live envelope, but give the advisory target a finite geometric ETA derived
     * from the current station distance plus the configured terminal reserve.
     */
    double geometry_path=isfinite(distance)&&distance>=0.0&&isfinite(remaining)&&remaining>=0.0?
        distance+remaining:approach_path;
    double arrival_time=energy_qualified?arrival.travel_time_s:
        response+geometry_path/fmax(t->horizontal_speed,DBL_MIN);
    if(!isfinite(arrival_time)||arrival_time<0.0)return;
    if(decision_diag_on){
        if(energy_qualified)
            fprintf(stderr,
                "MM304 target accept: V %.1f in [%.1f, %.1f], path %.0f Rmin %.0f dragWork %.0f travel %.1f.\n",
                target_speed,speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                approach_path,minimum_turn_radius,arrival.modeled_drag_work,arrival_time);
        else
            fprintf(stderr,
                "MM304 geometry target: projected %s V %.1f, guidance V %.1f in [%.1f, %.1f], path %.0f Rmin %.0f ETA %.1f.\n",
                arrival.valid?"below structural":"invalid",arrival.valid?arrival.terminal_speed_mps:NAN,
                target_speed,speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                approach_path,minimum_turn_radius,arrival_time);
    }

    double lead=t->horizontal_speed*response;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);

    TaemInterfaceTarget next={0};
    next.valid=isfinite(point.latitude)&&isfinite(point.longitude)&&
        isfinite(target_altitude)&&isfinite(target_speed)&&target_speed>0.0&&
        isfinite(target_course)&&isfinite(arrival_time)&&
        contract.horizontal_radius_m>0.0&&
        contract.perpendicular_heading_half_width_deg>0.0;
    if(!next.valid)return;
    next.energy_qualified=energy_qualified;
    next.along_track=station;next.cross_track=0.0;next.course=target_course;
    next.altitude=target_altitude;next.speed=target_speed;next.flight_path_angle=target_fpa;
    next.specific_energy=rotating_specific_energy(point.latitude,target_altitude,target_speed,p);
    next.selected_ut=t->ut;next.arrival_ut=t->ut+arrival_time;
    next.quality_score=
        fabs(norm_signed_deg(next.course-reference_course))/
            contract.perpendicular_heading_half_width_deg+
        distance/contract.horizontal_radius_m;
    next.acquisition_lead=lead;
    next.remaining_path=fmax(lead,remaining);
    next.response_time=response;
    g->taem_interface_target=next;
}

static bool entry_taem_alignment_station_missed(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*point_bearing){
    if(point_bearing)*point_bearing=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double rel=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double tangent_along=cos(rel),tangent_cross=sin(rel);
    double from_along=t->runway_along_track-q->along_track;
    double from_cross=t->runway_cross_track-q->cross_track;
    double progress=from_along*tangent_along+from_cross*tangent_cross;
    double distance=hypot(from_along,from_cross);
    double to_along=-from_along,to_cross=-from_cross;
    double bearing=norm_deg(cfg->site.runway_heading+atan2(to_cross,to_along)*RAD2DEG);
    if(point_bearing)*point_bearing=bearing;

    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    if(!isfinite(distance)||distance<=contract.horizontal_radius_m)return false;

    double live=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(live))return isfinite(progress)&&progress>0.0;
    double live_rel=norm_signed_deg(live-cfg->site.runway_heading)*DEG2RAD;
    double forward_to_point=to_along*cos(live_rel)+to_cross*sin(live_rel);
    bool passed_tangent=isfinite(progress)&&progress>0.0;
    bool point_behind_velocity=isfinite(forward_to_point)&&forward_to_point<0.0;
    return passed_tangent||point_behind_velocity;
}

static double entry_taem_tangent_capture_heading(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg){
    if(!g||!t||!cfg)return 0.0;
    if(!g->taem_interface_target.valid||!isfinite(t->runway_along_track)||
       !isfinite(t->runway_cross_track)){
        double live=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        return isfinite(live)?norm_deg(live):0.0;
    }
    double point_bearing=NAN;
    if(entry_taem_alignment_station_missed(g,t,cfg,&point_bearing)){
        double live=isfinite(t->ground_track_heading)?
            norm_deg(t->ground_track_heading):norm_deg(t->heading);
        if(!isfinite(live)||!isfinite(point_bearing))
            return g->taem_interface_target.course;
        double point_turn=norm_signed_deg(point_bearing-live);
        double course_turn=norm_signed_deg(g->taem_interface_target.course-live);
        double resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        double side=fabs(course_turn)>resolution?
            (course_turn>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
        if(side*point_turn>0.0)return point_bearing;
        /* The point is aft of the committed turn. Continue on the geometric
           quarter-turn tangent until it moves onto the owned side; do not
           reverse S-turn ownership merely to chase an aft waypoint. */
        return norm_deg(live+side*90.0); /* decision-literal-ok: quarter-turn tangent geometry */
    }
    return taem_interface_line_heading(t->runway_along_track,t->runway_cross_track,
        cfg->site.runway_heading,&g->taem_interface_target);
}


static double entry_taem_tangent_capture_bank(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const LandingConfiguration*cfg,double desired_heading){
    if(!g||!t||!cfg||!g->taem_interface_target.valid)return 0.0;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    return taem_course_rate_bank_command(t->horizontal_speed,live_lift_accel(t,aero,&cfg->vehicle),
        t->bank_effectiveness>0?t->bank_effectiveness:1.0,dynamic_bank_limit(t,&cfg->vehicle),
        course,desired_heading,g->taem_interface_target.response_time);
}

static double entry_taem_minimum_bank_for_capture(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double target_along,double target_cross,
        double target_course,double side){
    if(!g||!t||!p||!cfg||!isfinite(target_along)||!isfinite(target_cross)||
       !isfinite(target_course)||!(t->horizontal_speed>DBL_MIN))return 0.0;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return 0.0;

    double distance=hypot(target_along-t->runway_along_track,
        target_cross-t->runway_cross_track);
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.controllable)return 0.0;

    double available_time=isfinite(q->arrival_ut)?
        fmax(0.0,q->arrival_ut-t->ut):
        authority.control_response_time_s+distance/t->horizontal_speed;
    if(!isfinite(available_time))return 0.0;

    double lift=live_lift_accel(t,aero,&cfg->vehicle);
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double limit=dynamic_bank_limit(t,&cfg->vehicle);
    if(!(lift>DBL_MIN)||!isfinite(limit)||limit<=0.0)return 0.0;

    double signed_side=side>=0.0?1.0:-1.0;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,limit);
    double max_effective=fmin(limit*effectiveness,nextafter(90.0,0.0));
    double max_lateral=lift*fabs(sin(max_effective*DEG2RAD));
    TargetCaptureEnvelope maximum=
        decision_target_capture_envelope_with_lateral_accel(
            g,t,course,target_along,target_cross,target_course,
            available_time,max_lateral,p,cfg);
    if(!maximum.valid||!maximum.reachable)return signed_side*limit;

    double low=0.0,high=limit;
    unsigned guard=0;
    while(high-low>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(low+high); /* decision-literal-ok: bisection midpoint */
        double effective=fmin(mid*effectiveness,nextafter(90.0,0.0));
        double lateral=lift*fabs(sin(effective*DEG2RAD));
        TargetCaptureEnvelope capture=
            decision_target_capture_envelope_with_lateral_accel(
                g,t,course,target_along,target_cross,target_course,
                available_time,lateral,p,cfg);
        if(capture.valid&&capture.reachable)high=mid;
        else low=mid;
    }
    return signed_side*high;
}

static double entry_taem_gate_required_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return 0.0;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return 0.0;

    TaemInterfaceCapture handoff=
        entry_taem_interface_capture(q,t,course,p,cfg);
    if(handoff.valid&&handoff.ready)return 0.0;

    double side=g->s_turn_sign>=0.0?1.0:-1.0;
    double target_along=q->along_track,target_cross=q->cross_track;
    double target_course=q->course,final_side=0.0;
    double setup_along=NAN,setup_cross=NAN;
    bool have_setup=entry_taem_pose_setup_point(g,t,p,cfg,
        &setup_along,&setup_cross,&final_side);
    bool shaping_leg=have_setup&&side*final_side<0.0&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,final_side);
    if(shaping_leg){
        target_along=setup_along;
        target_cross=setup_cross;
        target_course=course;
    }
    return entry_taem_minimum_bank_for_capture(g,t,p,aero,cfg,
        target_along,target_cross,target_course,side);
}

static double entry_taem_gate_acquisition_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid)return 0.0;
    double bank=entry_taem_gate_required_bank(g,t,p,aero,cfg);
    double side=g->s_turn_sign>=0.0?1.0:-1.0;
    if(bank*side>0.0)return bank;

    double recovery_bearing=NAN;
    bool missed=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
    (void)recovery_bearing;
    bool blocked=entry_program_terminal_geometry_blocked(g,t,cfg);
    if(!missed&&!blocked)return 0.0;

    /* Missed-point recovery and an overdue blocked endpoint are topological
       requirements, not invitations to invent a tuned bank floor. Request the
       full live envelope; the coupled maximin allocator decides what portion is
       executable after vertical and energy margins are included. */
    return side*dynamic_bank_limit(t,&cfg->vehicle);
}


/* After the first energy-management reversal, keep the acquired setup-side
   crossrange and carry it downstream on a runway-parallel leg until the measured
   endpoint arc becomes feasible. This prevents the shaping reversal and the final
   inlet turn from collapsing into one long maximum-bank maneuver. */
static bool entry_program_post_shaping_parallel_stage(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double*bank_out){
    if(bank_out)*bank_out=0.0;
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       g->entry_control_reversals==0||g->entry_final_reversal_pending||
       g->entry_final_reversal_completed||!isfinite(course)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(!isfinite(target_relative)||fabs(target_relative)<30.0)return false;
    double final_side=target_relative>=0.0?1.0:-1.0;
    double owned_side=g->s_turn_sign>=0.0?1.0:-1.0;
    if(owned_side*final_side<=0.0||t->runway_along_track>=q->along_track-1000.0)return false;
    if(entry_program_final_reversal_geometry_ready(g,t,cfg,final_side))return false;

    /* After the first shaping reversal, the runway-parallel segment is a
       deceleration lane, not proof that the final 90 deg arc is already flyable.
       Requiring crossrange equal to the entire current upstream chord made this
       state unreachable at entry speed, so MM304 simply held maximum bank until
       the safety handoff. Preserve a HAC-sized lateral offset instead; the strict
       endpoint test above still owns release into the final perpendicular turn. */
    double setup_side=-final_side;
    double built_cross=setup_side*(t->runway_cross_track-q->cross_track);
    if(!isfinite(built_cross)||built_cross<-500.0)return false;

    /* The setup proof was consumed when the first shaping reversal executed. Do
       not require the crossrange to remain frozen afterward: a real final turn
       must spend that offset while bending back toward the fixed station. The old
       runway-parallel hold drove commanded bank to zero for tens of seconds in
       finalaoa7, leaving course near 90 deg all the way to the safety boundary.

       Lead the fixed tangent line directly with a bounded intercept heading. This
       is a decelerating spiral/conic lead, not permission to hand off early: the
       <=1 km point/course/energy/HAC capture contract remains unchanged. */
    double desired=entry_taem_tangent_capture_heading(g,t,cfg);
    double turn_error=norm_signed_deg(desired-course);
    double bank=0.0;
    if(isfinite(desired)&&final_side*turn_error>0.75){
        double limit=dynamic_bank_limit(t,&cfg->vehicle);
        /* finalaoa13 proved that artificially capping this lead at 60 deg made
           coordination worse: beta grew past 50 deg and tripped the genuine yaw
           departure guard. The 70 deg vehicle envelope in finalaoa12 remained
           controllable, so let the measured q/g/vertical guards own the limit. */
        double vertical=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
        if(isfinite(vertical))limit=fmin(limit,fmax(0.0,vertical));
        if(limit>0.25)
            bank=taem_course_rate_bank_command(t->horizontal_speed,
                live_lift_accel(t,aero,&cfg->vehicle),
                t->bank_effectiveness>0?t->bank_effectiveness:1.0,limit,
                course,desired,q->response_time);
        if(bank*final_side<0.0)bank=0.0;
    }
    if(bank_out)*bank_out=bank;
    return true;
}

static bool entry_program_tangent_reversal_captured(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v){
    if(!g||!t||!v||!g->entry_final_reversal_pending)return g&&g->entry_final_reversal_completed;
    bool authority=entry_s_turn_bank_authority_available(t->dynamic_pressure,t->true_air_speed,
        t->stall_fraction,t->g_force,v);
    double actual=norm_signed_deg(t->roll);
    double roll_rate=controlled_roll_rate(t);
    double angle_resolution=sqrt(DBL_EPSILON)*fmax(1.0,fabs(actual));
    double rate_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,isfinite(roll_rate)?fabs(roll_rate):1.0);
    bool crossed=g->s_turn_sign*actual>angle_resolution;
    bool not_rolling_back=!isfinite(roll_rate)||
        g->s_turn_sign*roll_rate>=-rate_resolution;
    if(authority&&crossed&&not_rolling_back){
        g->entry_final_reversal_pending=false;
        g->entry_final_reversal_completed=true;
        return true;
    }
    return false;
}

static TaemInterfaceCapture entry_dynamic_interface_capture(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg)return (TaemInterfaceCapture){0};
    TaemInterfaceCapture out=entry_taem_interface_capture(
        &g->taem_interface_target,t,course,p,cfg);
    /* MM304 owns only this precomputed tangent-state contract.  A future HAC or
       spline preview is deliberately not an ownership prerequisite: MM305 must be
       free to select its own HAC after receiving the required position, altitude,
       speed and tangential course. */
    out.ready=out.valid&&out.veto==0;
    return out;
}

static GuidanceResult entry_topology_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const EntryTopologyPlan*top=&g->entry_topology;const TaemInterfaceTarget*q=&g->taem_interface_target;
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    bool turning=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    double time_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,fmax(fabs(top->terminal_turn_ut),fabs(top->reversal_ut)));
    bool separate_terminal_turn=isfinite(top->terminal_turn_ut)&&
        isfinite(top->reversal_ut)&&top->terminal_turn_ut>top->reversal_ut+time_resolution;
    bool started=false;
    if(!turning&&g->entry_reversal_scheduled)started=entry_program_execute_planned_reversal(g,t,p,cfg);
    turning=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    bool shaping_reversal_complete=separate_terminal_turn&&!turning&&
        g->s_turn_sign*top->first_sign<0.0&&t->ut+time_resolution>=top->reversal_ut;
    if(started&&!g->diagnostic_shadow)fprintf(stderr,
        "Entry topology REVERSAL: UT %.2f deadline %.2f x %.0f/%.0f y %.0f/%.0f h %.0f/%.0f V %.1f/%.1f course %.1f/%.1f -> %.1f\n",
        t->ut,top->reversal_ut,t->runway_along_track,top->reversal_along,t->runway_cross_track,top->reversal_cross,
        t->mean_altitude,top->reversal_altitude,t->true_air_speed,top->reversal_speed,course,top->reversal_course,
        separate_terminal_turn?cfg->site.runway_heading:q->course);

    /* A complete propagated topology has two distinct events: first reverse the
       energy-management leg and settle near runway-parallel flight, then begin the
       same-side 90 deg terminal turn when the measured turn circle reaches the
       fixed -8 km alignment station.  The old executor collapsed these into one
       final reversal, so every otherwise-useful two-event topology was rejected. */
    double desired_heading=course;
    double bank=top->first_sign*top->first_bank;
    if(shaping_reversal_complete){
        desired_heading=norm_deg(cfg->site.runway_heading);
        bank=entry_taem_tangent_capture_bank(g,t,aero,cfg,desired_heading);
        double dynamic_limit=dynamic_bank_limit(t,v);
        if(isfinite(dynamic_limit))
            bank=clampd(bank,-dynamic_limit,dynamic_limit);

        double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
        double final_side=target_relative>=0.0?1.0:-1.0;
        double endpoint_error=INFINITY,rollthrough_error=INFINITY,rollthrough_lead=NAN;
        bool endpoint_ready=entry_program_terminal_course_endpoint_ready(
            g,t,cfg,final_side,q->course,&endpoint_error,NULL);
        bool rollthrough_ready=entry_program_rollthrough_endpoint_ready(
            g,t,p,cfg,final_side,&rollthrough_error,&rollthrough_lead);
        ControlAuthorityEnvelope authority=
            decision_control_authority_envelope(g,t,p,cfg);

        /* A propagated terminal-turn timestamp no longer authorizes anything.
           Start the final MM304 turn only when the measured state can close the
           fixed point/course endpoint now, or after the physically projected
           roll-through transient, with live control authority still survivable. */
        if(authority.controllable&&(endpoint_ready||rollthrough_ready)){
            g->entry_final_reversal_pending=true;
            g->entry_final_reversal_completed=false;
            turning=true;
            if(!g->diagnostic_shadow)fprintf(stderr,
                "MM304 terminal turn START: UT %.2f endpoint=%d err %.0f m rollthrough=%d err %.0f m lead %.2fs course %.1f -> %.1f.\n",
                t->ut,endpoint_ready?1:0,endpoint_error,rollthrough_ready?1:0,
                rollthrough_error,rollthrough_lead,course,q->course);
        }
    }

    double target_course_error=norm_signed_deg(q->course-course);
    TaemHandoffContract handoff_contract=taem_handoff_contract(&cfg->guidance);
    if(turning&&!g->entry_topology_heading_locked&&
       fabs(target_course_error)<=handoff_contract.perpendicular_heading_half_width_deg){
        g->entry_topology_heading_locked=true;
        if(!g->diagnostic_shadow)fprintf(stderr,
            "MM304 perpendicular HEADING LOCK: UT %.2f course %.1f target %.1f x %.0f y %.0f.\n",
            t->ut,course,q->course,t->runway_along_track,t->runway_cross_track);
    }
    bool point_recovery_required=false;
    if(turning&&!g->entry_topology_heading_locked){
        /* Recompute the circle that satisfies the complete fixed-pose contract at
           every step: current course -> fixed -8 km point -> perpendicular outlet.
           Command the bank needed for that radius at the *current* speed/lift, so
           deceleration opens the bank instead of shrinking the flown radius inside
           the planned arc. */
        double turn_sign=-top->first_sign;
        double pose_error=INFINITY,pose_radius=INFINITY;
        (void)entry_program_terminal_course_endpoint_ready(g,t,cfg,turn_sign,q->course,
            &pose_error,&pose_radius);
        double limit=fmin(top->turn_bank,dynamic_bank_limit(t,v));
        double lift=live_lift_accel(t,aero,v);
        double eff=clampd(t->bank_effectiveness,.35,1.8);
        double horizontal=fmax(80.0,t->horizontal_speed);
        double bank_mag=0.0;
        if(isfinite(pose_radius)&&pose_radius>=1000.0&&isfinite(lift)&&lift>.01){
            double lateral=horizontal*horizontal/pose_radius;
            double max_ratio=sin(clampd(limit*eff,0.0,89.0)*DEG2RAD);
            bank_mag=asin(clampd(lateral/lift,0.0,max_ratio))/eff*RAD2DEG;
        }else{
            double fallback=taem_course_rate_bank_command(horizontal,lift,eff,limit,
                course,q->course,q->response_time);
            bank_mag=fabs(fallback);
        }
        bank=turn_sign*clampd(bank_mag,0.0,limit);
        desired_heading=q->course;
    }else if(turning){
        /* Once the +/-90 deg course is acquired, capture the fixed -8 km point, not
           an infinite tangent line. If the point is crossed diagonally, continue the
           owned turn with full demonstrated bank authority until it is reacquired. */
        desired_heading=entry_taem_tangent_capture_heading(g,t,cfg);
        double recovery_bearing=NAN;
        point_recovery_required=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
        (void)recovery_bearing;
        bank=entry_taem_tangent_capture_bank(g,t,aero,cfg,desired_heading);
        double correction_limit=dynamic_bank_limit(t,v);
        bank=clampd(bank,-correction_limit,correction_limit);
    }
    double terminal_floor=entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v);
    double aoa=fmax(t->true_air_speed>top->aoa_switch_speed?top->first_aoa:top->turn_aoa,terminal_floor);
    /* The topology executor must preserve the same post-reversal energy policy as
       ordinary MM304. Once control authority is established and the shuttle has
       entered the TAEM-energy band, carry only the protected high-L/D incidence
       needed by the terminal turn instead of burning speed at the thermal-entry
       incidence. This is still bounded by q/g/stall protection below. */
    bool preserve_terminal_energy=(shaping_reversal_complete||turning)&&
        t->true_air_speed<s->taem_force_handoff_speed+350.0&&
        t->dynamic_pressure>=700.0&&t->dynamic_pressure<v->maximum_dynamic_pressure*.85&&
        t->g_force<v->maximum_g_load*.85&&
        (!t->stall_fraction_is_measured||t->stall_fraction<.10);
    bool extended_final_turn=turning&&isfinite(top->turn_aoa)&&
        top->turn_aoa>v->maximum_angle_of_attack+1e-6&&
        t->dynamic_pressure<v->maximum_dynamic_pressure*.88&&t->g_force<v->maximum_g_load*.90&&
        (!t->stall_fraction_is_measured||t->stall_fraction<.08);
    if((preserve_terminal_energy||point_recovery_required)&&!extended_final_turn)
        aoa=fmin(aoa,fmax(15.0,terminal_floor));
    bank=clampd(bank,-dynamic_bank_limit(t,v),dynamic_bank_limit(t,v));
    bool safety_alpha=(t->stall_fraction_is_measured&&t->stall_fraction>.12)||t->g_force>v->maximum_g_load*.92;
    if(safety_alpha)aoa=fmin(aoa,fmax(6.0,t->angle_of_attack-2.0));
    double topology_aoa_ceiling=extended_final_turn?entry_final_s_turn_aoa_ceiling(v):v->maximum_angle_of_attack;
    aoa=clampd(aoa,0.0,topology_aoa_ceiling);
    g->entry_control_bank=bank;
    if(turning)(void)entry_program_tangent_reversal_captured(g,t,v);
    else entry_update_s_turn_leg_capture(g,t,bank,v);

    TaemInterfaceCapture capture=entry_dynamic_interface_capture(g,t,course,p,cfg);
    bool handoff_locked=turning&&capture.ready&&fabs(norm_signed_deg(t->roll))<=18.0;
    g->entry_topology_capture_good_duration=handoff_locked?g->entry_topology_capture_good_duration+fmax(0.0,dt):0.0;
    double rh=cfg->site.runway_heading*DEG2RAD;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint gate=local_point(origin,q->along_track*sin(rh)+q->cross_track*cos(rh),
        q->along_track*cos(rh)-q->cross_track*sin(rh),p->radius,q->altitude);
    EntryDragReferenceConfig drag_cfg=entry_drag_reference_default_config();
    drag_cfg.entry_velocity_ratio=4.5;
    EntryDragReferenceInput drag_in={.phase=g->entry_exec.initialized?g->entry_exec.phase:ENTRY_PHASE_PREENTRY,
        .relative_velocity=t->true_air_speed,.latitude=t->latitude,.altitude=t->mean_altitude,
        .measured_drag_accel=live_drag_accel(t,aero,v),.modeled_drag_accel=live_drag_accel(t,aero,v),
        .aero_confidence=clampd(aero.confidence,0.0,1.0),.has_incidence=true,
        .angle_of_attack=t->angle_of_attack,.sideslip=t->sideslip,
        .range_to_site=hypot(q->along_track-t->runway_along_track,q->cross_track-t->runway_cross_track),
        .taem_range=0.0,.taem_latitude=gate.latitude,.taem_altitude=q->altitude,.taem_velocity=q->speed,
        .planet=p,.vehicle=v};
    EntryDragReferenceOutput drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
    /* A propagated topology owns reversal geometry, but live longitudinal energy
       still owns how much drag the final S-turn must make. finalaoa4 selected a
       nominal 16 deg turn incidence while the measured Entry-to-TAEM excess was
       still about +200 km, so the topology path bypassed the explicitly allowed
       ~35 deg drag correction used by ordinary MM304. Apply the same live q/g/
       stall/path-guarded correction here after the topology drag reference exists. */
    double live_final_s_turn_aoa=entry_program_final_s_turn_drag_aoa(g,t,p,aero,cfg,&drag,
        turning||shaping_reversal_complete);
    if(turning&&isfinite(live_final_s_turn_aoa)){
        aoa=fmax(aoa,live_final_s_turn_aoa);
        aoa=clampd(aoa,0.0,entry_final_s_turn_aoa_ceiling(v));
    }
    EntryExecProfile profile=entry_drag_reference_exec_profile(&drag_in,&drag_cfg,&drag);
    bool handoff_qualified=capture.ready&&g->entry_topology_capture_good_duration>=.75;
    EntryExecObservation obs={.ut=t->ut,.relative_velocity=t->true_air_speed,.altitude=t->mean_altitude,
        .dynamic_pressure=fmax(0.0,t->dynamic_pressure),.checkpoint_restart=false};
    bool executive_ok=entry_exec_update(&g->entry_exec,&obs,&profile);
    if(executive_ok&&handoff_qualified&&g->entry_exec.phase==ENTRY_PHASE_TRANSITION)
        executive_ok=entry_exec_accept_taem_handoff(&g->entry_exec,&obs);
    if(executive_ok&&g->entry_exec.entry_complete)g->taem_interface_captured=true;
    if(!g->diagnostic_shadow&&(t->ut-g->taem_interface_diagnostic_ut>=5.0||g->taem_interface_captured)){
        fprintf(stderr,"TAEM topology inlet: UT %.2f x %.0f y %.0f h %.0f V %.1f course %.1f -> %.1f bank %.1f roll %.1f veto %u along %.0f cross %.0f dh %.0f energy %.0f turn %.0f locked %.2fs ready %d\n",
            t->ut,t->runway_along_track,t->runway_cross_track,t->mean_altitude,t->true_air_speed,course,q->course,
            bank,norm_signed_deg(t->roll),capture.veto,capture.along,capture.cross,capture.altitude_error,
            capture.energy_margin,capture.turn_margin,g->entry_topology_capture_good_duration,g->taem_interface_captured);
        g->taem_interface_diagnostic_ut=t->ut;
    }
    const char*warning=NULL;
    if(!turning&&t->ut>top->reversal_ut+10.0){
        warning="Complete-path reversal setup was not reached inside its physical state tube; refusing a late near-point reversal.";
        g->entry_committed_infeasible=true;
        if(!g->diagnostic_shadow)fprintf(stderr,"Entry topology FAILED setup: UT %.2f deadline %.2f x %.0f y %.0f h %.0f V %.1f course %.1f\n",
            t->ut,top->reversal_ut,t->runway_along_track,t->runway_cross_track,t->mean_altitude,t->true_air_speed,course);
        guidance_abort(g);
    }else if(turning&&!g->taem_interface_captured){
        /* A forecast clock never releases MM304 ownership of the fixed alignment
           point. If capture takes longer than predicted, keep the live point/heading
           controller active (and flag the stale forecast) instead of aborting to
           straight flight. Only a real low-speed boundary makes this trajectory dead. */
        if(t->true_air_speed<v->minimum_safe_speed*1.6){
            warning="MM304 fixed-point capture became physically unrecoverable at low speed; handoff remains inhibited.";
            g->entry_committed_infeasible=true;
            if(!g->diagnostic_shadow)fprintf(stderr,"Entry topology FAILED inlet energy: UT %.2f deadline %.2f x %.0f y %.0f h %.0f V %.1f course %.1f veto %u [speed=%d spatial=%d course=%d altitude=%d maneuver=%d fpa=%d structural=%d HACradius=%d minAlt=%d].\n",
                t->ut,top->capture_ut,t->runway_along_track,t->runway_cross_track,t->mean_altitude,t->true_air_speed,course,capture.veto,
                (capture.veto&1u)!=0,(capture.veto&2u)!=0,(capture.veto&4u)!=0,(capture.veto&8u)!=0,
                (capture.veto&16u)!=0,(capture.veto&32u)!=0,(capture.veto&64u)!=0,
                (capture.veto&128u)!=0,(capture.veto&256u)!=0);
            guidance_abort(g);
        }else if(t->ut>top->capture_ut+30.0){
            warning=point_recovery_required?
                "MM304 capture forecast expired; continuing fixed-point recovery until the 1 km handoff contract is met.":
                "MM304 capture forecast expired; continuing fixed-point capture and requesting updated supervision.";
            g->entry_planning_needed=true;
        }
    }else if(!executive_ok)warning="Entry executive rejected the coupled path reference; handoff remains inhibited.";
    else if(safety_alpha)warning="Stall/load protection is overriding the propagated incidence; the trajectory requires live revalidation.";
    else if(g->entry_supervision_boundary_missed)warning="Latest advisory shadow did not certify this inlet; live capture remains mandatory.";
    g->entry_s_turn_plan.valid=true;
    g->entry_s_turn_plan.has_planned_reversal=!turning;
    g->entry_s_turn_plan.final_heading_lock=turning;
    g->entry_control_plan_valid=true;
    g->entry_control_terminal_ready=g->entry_s_turn_plan.terminal_ready;
    g->entry_control_plan_ut=t->ut;g->entry_control_aoa=aoa;g->entry_control_heading=desired_heading;
    g->entry_control_turn_radius=live_turn_radius(t,aero,v,bank);g->entry_control_cost=top->cost;
    g->entry_control_segment_until_ut=top->capture_ut+30.0;
    g->airbrakes_deployed=false;
    if(!turning&&t->ut-g->entry_supervision_ut>=15.0)g->entry_planning_needed=true;
    GuidanceCommand command=atmospheric(t,desired_heading,bank,v,0.0,false,PROFILE_ENTRY);
    command.heading_control_enabled=false;command.has_target_aoa=true;
    command.target_aoa=aoa;command.target_pitch=t->flight_path_angle+aoa;
    char status[384];
    if(!turning)snprintf(status,sizeof(status),"MM304 reachable S-turn: side %+.0f bank %+.1f alpha %.1f, plan %.1fs, reversal in %.1fs; inlet %.1f/%.1f km, course %.0f.",
        top->first_sign,bank,aoa,fmax(0.0,g->entry_control_segment_until_ut-t->ut),top->reversal_ut-t->ut,
        q->along_track/1000.0,q->cross_track/1000.0,q->course);
    else snprintf(status,sizeof(status),"MM304 %s: course %.1f -> %.1f ref %.1f, bank %+.1f roll %+.1f, gate %.1f km, h %.1f/%.1f V %.0f/%.0f, inlet veto %u.",
        g->entry_topology_heading_locked?"heading lock":"perpendicular turn",course,q->course,desired_heading,
        bank,norm_signed_deg(t->roll),drag_in.range_to_site/1000.0,t->mean_altitude/1000.0,q->altitude/1000.0,
        t->true_air_speed,q->speed,capture.veto);
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,command,status,warning),t,v,s,dt);
}

static double entry_alpha_transition_velocity(const GuidanceSettings*s,double strict_taem_velocity){
    /* The strict TAEM/HAC inlet speed is a capture contract, not an Entry AoA
       scheduling breakpoint. Reusing the <=500 m/s capture target here compressed
       the whole five-point alpha schedule below 1 km/s and forced max-alpha Entry
       while the orbiter was still above 2 km/s. Preserve the strict target for
       drag/capture accounting, but retain the established MM304 shaping speed as
       the aerodynamic schedule floor. */
    if(!s)return strict_taem_velocity;
    return fmax(strict_taem_velocity,s->taem_force_handoff_speed);
}

static GuidanceResult entry_program_contract_hold(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double dt,const char*warning){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    double bank_limit=dynamic_bank_limit(t,v);
    bool have_previous=g->entry_control_plan_valid&&
        isfinite(g->entry_control_bank)&&isfinite(g->entry_control_aoa)&&
        isfinite(g->entry_control_heading);

    /*
     * Invalid model data is not permission to switch controllers. Preserve the
     * last executable MM304 command, bounded by the live vehicle envelope. If
     * no executable command exists yet, preserve the measured aerodynamic
     * attitude instead. This is a zero-order continuity action, not a recovery
     * trajectory and it cannot satisfy or advance the handoff contract.
     */
    double bank=have_previous?g->entry_control_bank:norm_signed_deg(t->roll);
    double aoa=have_previous?g->entry_control_aoa:t->angle_of_attack;
    double heading=have_previous?g->entry_control_heading:
        (isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading);
    if(!isfinite(bank))bank=0.0;
    if(!isfinite(aoa))aoa=v->entry_angle_of_attack;
    if(!isfinite(heading))heading=cfg->site.runway_heading;
    bank=clampd(bank,-bank_limit,bank_limit);
    aoa=clampd(aoa,0.0,v->maximum_angle_of_attack);

    g->entry_control_terminal_ready=false;
    g->entry_planning_needed=true;
    g->airbrakes_deployed=false;

    GuidanceCommand c=atmospheric(t,heading,bank,v,0.0,false,PROFILE_ENTRY);
    c.heading_control_enabled=false;
    c.has_target_aoa=true;
    c.target_aoa=aoa;
    c.target_pitch=t->flight_path_angle+aoa;
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,
        "MM304 model contract unavailable; retaining Entry ownership and the bounded current command.",
        warning),t,v,s,dt);
}

static GuidanceResult entry_program_guidance(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    g->entry_lateral_bank_magnitude=NAN;
    g->entry_geometry_bank=NAN;
    g->entry_vertical_bank_magnitude=NAN;
    g->entry_demand_bank=NAN;
    g->entry_lateral_required_bank=NAN;
    bool seed_target_side=entry_program_first_segment(g)&&
        !g->entry_target_side_latched;
    const char *side_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    bool side_diag_on=side_diag&&strcmp(side_diag,"1")==0;
    double seed_side_before=g->s_turn_sign;
    entry_publish_taem_tangent_target(g,t,course,p,aero,cfg);
    if(seed_target_side&&side_diag_on&&!g->diagnostic_shadow)
        fprintf(stderr,
            "MM304 first-side before seed: UT %.2f gside %+.0f latched %d target %.1f current %.1f along %.1f cross %.1f\n",
            t->ut,seed_side_before,g->entry_target_side_latched,
            g->taem_interface_target.course,course,t->runway_along_track,
            t->runway_cross_track);
    /* The first publish may be the operation that creates the fixed inlet.  Seed
       its mission side from that geometry before the candidate reaches the
       planner, then rebuild the advisory record so its course cannot flip on
       the next telemetry sample. */
    if(seed_target_side){
        entry_program_seed_initial_side(g,t,cfg);
        if(g->entry_target_side_latched){
            g->taem_interface_target.valid=false;
            entry_publish_taem_tangent_target(g,t,course,p,aero,cfg);
        }
        if(side_diag_on&&!g->diagnostic_shadow)
            fprintf(stderr,
                "MM304 first-side after seed: UT %.2f side %+.0f target %.1f current %.1f\n",
                t->ut,g->s_turn_sign,g->taem_interface_target.course,course);
    }
    if(g->entry_topology.valid)return entry_topology_guidance(g,t,course,p,aero,cfg,dt);
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double taem_velocity=q->valid&&isfinite(q->speed)&&q->speed>0.0?
        q->speed:s->taem_force_handoff_speed;
    if(!g->entry_drag_ratio_valid){
        g->entry_drag_velocity_ratio=clampd(t->true_air_speed/fmax(taem_velocity,1.0)*1.05,1.20,4.50);
        g->entry_drag_ratio_valid=true;
    }

    double drag_range=fmax(0.0,t->range_to_site);
    double drag_taem_range=entry_taem_range_target(p,s);
    double drag_taem_latitude=cfg->site.latitude;
    double drag_taem_altitude=s->taem_interface_altitude;
    if(q->valid&&isfinite(t->runway_along_track)&&isfinite(t->runway_cross_track)){
        double da=q->along_track-t->runway_along_track;
        double dc=q->cross_track-t->runway_cross_track;
        drag_range=hypot(da,dc);
        drag_taem_range=0.0;
        drag_taem_altitude=q->altitude;
        double rh=cfg->site.runway_heading*DEG2RAD;
        double e=q->along_track*sin(rh)+q->cross_track*cos(rh);
        double n=q->along_track*cos(rh)-q->cross_track*sin(rh);
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint gate=local_point(origin,e,n,p->radius,q->altitude);
        if(isfinite(gate.latitude))drag_taem_latitude=gate.latitude;
    }

    EntryPhase phase=g->entry_exec.initialized?g->entry_exec.phase:ENTRY_PHASE_PREENTRY;
    EntryDragReferenceConfig drag_cfg=entry_drag_reference_default_config();
    drag_cfg.entry_velocity_ratio=g->entry_drag_velocity_ratio;
    double measured_drag=t->mass>1&&isfinite(t->drag_force)&&t->drag_force>0?t->drag_force/t->mass:NAN;
    double modeled_drag=live_drag_accel(t,aero,v);
    EntryDragReferenceInput drag_in={
        .phase=phase,.relative_velocity=t->true_air_speed,.latitude=t->latitude,.altitude=t->mean_altitude,
        .measured_drag_accel=measured_drag,.modeled_drag_accel=modeled_drag,
        .aero_confidence=clampd(fmax(aero.confidence,t->aerodynamic_confidence),0.0,1.0),
        .has_incidence=isfinite(t->angle_of_attack)&&isfinite(t->sideslip),
        .angle_of_attack=t->angle_of_attack,.sideslip=t->sideslip,
        .range_to_site=drag_range,.taem_range=drag_taem_range,
        .taem_latitude=drag_taem_latitude,.taem_altitude=drag_taem_altitude,
        .taem_velocity=taem_velocity,.planet=p,.vehicle=v
    };
    EntryDragReferenceOutput drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
    if(!drag.valid)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 drag-reference contract is invalid; no alternate guidance controller was entered.");

    EntryExecProfile exec_profile=entry_drag_reference_exec_profile(&drag_in,&drag_cfg,&drag);
    /* TAEM readiness is a live handoff contract, not a mutable velocity gate in
       the Entry profile. The executive advances through its longitudinal phases,
       then accepts ownership transfer only through entry_exec_accept_taem_handoff. */
    TaemInterfaceCapture capture=entry_dynamic_interface_capture(g,t,course,p,cfg);
    bool handoff_geometry_ready=capture.valid&&capture.ready;
    if(!g->diagnostic_shadow&&(t->ut-g->taem_interface_diagnostic_ut>=5.0||capture.ready)){
        const TaemInterfaceTarget*q=&g->taem_interface_target;
        char veto_reason[160];
        taem_capture_veto_reasons(capture.veto,veto_reason,sizeof(veto_reason));
        fprintf(stderr,"TAEM interface: UT %.2f live h %.0f V %.1f range %.0f course %.1f FPA %.1f target valid %d energy %d along %.0f cross %.0f h %.0f V %.1f course %.1f FPA %.1f lead %.0f capture valid %d veto %u reasons %s da %.0f dc %.0f dh %.0f dcourse %.1f energy %.0f turn %.0f ready %d\n",
            t->ut,t->mean_altitude,t->true_air_speed,t->range_to_site,course,t->flight_path_angle,
            q->valid,q->energy_qualified,q->along_track,q->cross_track,q->altitude,q->speed,q->course,q->flight_path_angle,q->acquisition_lead,
            capture.valid,capture.veto,veto_reason,capture.along,capture.cross,capture.altitude_error,capture.course_error,
            capture.energy_margin,capture.turn_margin,capture.ready);
        g->taem_interface_diagnostic_ut=t->ut;
    }
    bool checkpoint_restart=!g->entry_exec.initialized&&g->entry_continuation_bootstrap;
    EntryExecObservation obs={
        .ut=t->ut,.relative_velocity=t->true_air_speed,.altitude=t->mean_altitude,
        .dynamic_pressure=fmax(0.0,t->dynamic_pressure),.checkpoint_restart=checkpoint_restart
    };
    bool exec_ok=entry_exec_update(&g->entry_exec,&obs,&exec_profile);
    if(!exec_ok)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 executive rejected its reference profile; Entry ownership is retained without a controller bypass.");
    if(handoff_geometry_ready&&g->entry_exec.phase==ENTRY_PHASE_TRANSITION&&
       !entry_exec_accept_taem_handoff(&g->entry_exec,&obs))
        return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 handoff was rejected by the Entry executive; the fixed interface contract remains unsatisfied.");
    if(g->entry_exec.phase!=drag_in.phase){
        drag_in.phase=g->entry_exec.phase;
        drag=entry_drag_reference_compute(&drag_in,&drag_cfg);
        if(!drag.valid)return entry_program_contract_hold(g,t,cfg,dt,
            "MM304 phase transition invalidated the drag-reference contract; retaining the current MM304 command.");
    }
    if(g->entry_exec.entry_complete)g->taem_interface_captured=true;

    EntryAlphaSchedule alpha_schedule;
    double alpha_transition_velocity=entry_alpha_transition_velocity(s,taem_velocity);
    entry_alpha_schedule_default(&alpha_schedule,v,alpha_transition_velocity);
    EntryAlphaInput alpha_in={
        .relative_velocity=t->true_air_speed,.phase=entry_alpha_phase_for_exec(g->entry_exec.phase),
        .reference_drag_accel=drag.reference_drag_accel,.measured_drag_accel=drag.observed_drag_accel,
        .current_aoa=t->angle_of_attack,
        .current_aoa_rate=controlled_aoa_rate(t),
        .has_previous_target=g->entry_alpha_has_target,.previous_target_aoa=g->entry_alpha_target,
        .has_previous_modulation=g->entry_alpha_has_modulation,.previous_modulation=g->entry_alpha_modulation,
        .dt=fmax(.001,dt),.maximum_aoa=v->maximum_angle_of_attack,.stall_margin_aoa_limit=NAN,
        .stall_fraction=t->stall_fraction,.stall_fraction_is_measured=t->stall_fraction_is_measured,
        .calibrated_stall_speed=t->calibrated_stall_speed,
        .minimum_safe_speed=v->minimum_safe_speed,.dynamic_pressure=t->dynamic_pressure,
        .maximum_dynamic_pressure=v->maximum_dynamic_pressure,.g_load=t->g_force,
        .maximum_g_load=v->maximum_g_load,
        .aero_confidence=clampd(fmax(drag.confidence,t->aerodynamic_confidence),0.0,1.0)
    };
    EntryAlphaResult alpha=entry_alpha_command(&alpha_schedule,&alpha_in);
    if(!alpha.valid)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 alpha scheduling contract is invalid; retaining the bounded current MM304 command.");
    g->entry_alpha_has_target=true;g->entry_alpha_target=alpha.target_aoa;
    g->entry_alpha_has_modulation=true;g->entry_alpha_modulation=alpha.modulation;

    /* DRAGREF still provides the local longitudinal work request and ENTRYLATERAL
       still translates that request into an authority-aware fallback magnitude.
       Neither one owns S-turn side or reversal timing anymore. */
    EntryLateralLimits limits=entry_lateral_default_limits();
    limits.maximum_bank_deg=dynamic_bank_limit(t,v);
    limits.maximum_roll_rate_deg_s=s->entry_roll_rate;
    limits.maximum_roll_accel_deg_s2=s->entry_roll_acceleration;
    limits.minimum_leg_duration_s=s->s_turn_minimum_leg_duration;
    EntryLateralInput magnitude_in={0};
    magnitude_in.ut=t->ut;magnitude_in.dt=fmax(.001,dt);magnitude_in.relative_speed=t->true_air_speed;
    magnitude_in.measured_bank_deg=norm_signed_deg(t->roll);
    magnitude_in.measured_bank_rate_deg_s=controlled_roll_rate(t);
    magnitude_in.bank_effectiveness=t->bank_effectiveness>0?t->bank_effectiveness:1.0;
    magnitude_in.lift_accel=live_lift_accel(t,aero,v);
    magnitude_in.authority_confidence=clampd(fmax(t->aerodynamic_confidence,t->physics_authority_confidence[1]),0.0,1.0);
    magnitude_in.course_to_site_error_deg=0.0;
    magnitude_in.has_crossrange_error=false;
    magnitude_in.longitudinal=(EntryLateralLongitudinalDemand){
        .valid=drag.valid,.confidence=drag.confidence,.reference_drag_accel=drag.reference_drag_accel,
        .drag_error_accel=drag.drag_error_accel,.predicted_range_error=drag.predicted_range_error,
        .range_error_scale=drag.range_error_scale,.required_vertical_lift_accel=drag.required_vertical_lift_accel
    };
    if(!g->entry_lateral.initialized)
        entry_lateral_state_init(&g->entry_lateral,t->ut,g->s_turn_sign,magnitude_in.measured_bank_deg);
    EntryLateralOutput magnitude=entry_lateral_update(&g->entry_lateral,&magnitude_in,&limits);
    if(!magnitude.valid)return entry_program_contract_hold(g,t,cfg,dt,
        "MM304 lateral authority allocation is invalid; retaining the bounded current MM304 command.");

    /*
     * Admit the final MM304 turn from a physical signed envelope rather than
     * speed/heading/corridor thresholds. The envelope projects the measured
     * roll transient with the configured actuator limits, derives minimum turn
     * radius from measured lift and speed, solves the fixed-station circular
     * endpoint geometry, and consumes the explicit MM304 capture-radius
     * contract plus measured trajectory uncertainty.
     */
    if(!g->entry_reversal_scheduled&&!g->entry_final_reversal_pending&&
       !g->entry_final_reversal_completed&&q->valid&&isfinite(q->course)&&
       isfinite(course)&&isfinite(t->runway_along_track)&&
       isfinite(t->runway_cross_track)){
        double horizontal_speed=isfinite(t->horizontal_speed)&&t->horizontal_speed>DBL_MIN?
            t->horizontal_speed:t->surface_speed;
        double lift_accel=live_lift_accel(t,aero,v);
        double authority_confidence=clampd(
            fmax(t->aerodynamic_confidence,t->physics_authority_confidence[1]),0.0,1.0);
        double usable_bank=fmin(limits.maximum_bank_deg,magnitude.bank_magnitude_deg);
        EntryLateralTurnInput turn_input={
            .horizontal_speed_mps=horizontal_speed,
            .current_course_deg=norm_signed_deg(course-cfg->site.runway_heading),
            .target_course_deg=norm_signed_deg(q->course-cfg->site.runway_heading),
            .current_along_m=t->runway_along_track,
            .current_cross_m=t->runway_cross_track,
            .target_along_m=q->along_track,
            .target_cross_m=q->cross_track,
            .measured_bank_deg=norm_signed_deg(t->roll),
            .measured_bank_rate_deg_s=controlled_roll_rate(t),
            .lift_accel_mps2=lift_accel*authority_confidence,
            .bank_effectiveness=isfinite(t->bank_effectiveness)&&t->bank_effectiveness>0.0?
                t->bank_effectiveness:1.0,
            .maximum_bank_deg=usable_bank,
            .maximum_roll_rate_deg_s=limits.maximum_roll_rate_deg_s,
            .maximum_roll_accel_deg_s2=limits.maximum_roll_accel_deg_s2,
            .capture_radius_m=s->mm304_handoff_radius,
            .position_uncertainty_m=isfinite(t->trajectory_range_residual)?
                fabs(t->trajectory_range_residual):0.0
        };
        EntryLateralTurnEnvelope turn=
            entry_lateral_terminal_turn_envelope(&turn_input);
        if(turn.valid&&turn.feasibility_margin_m>=0.0){
            EntryControlPlan heading_lock={0};
            heading_lock.valid=true;
            heading_lock.target_bank=turn.turn_sign*usable_bank;
            heading_lock.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));
            heading_lock.has_planned_reversal=true;
            heading_lock.planned_reversal_is_final=true;
            heading_lock.final_heading_lock=true;
            heading_lock.planned_reversal_ut=t->ut;
            heading_lock.planned_reversal_range=
                hypot(q->along_track-t->runway_along_track,q->cross_track-t->runway_cross_track);
            heading_lock.planned_reversal_sign=turn.turn_sign;
            entry_program_commit_planned_reversal(g,&heading_lock,t->ut,false);
            if(!g->diagnostic_shadow)
                fprintf(stderr,
                    "MM304 final turn admitted by physical envelope: UT %.2f sign %+.0f response %.2fs Rmin %.0f Rideal %.0f radiusMargin %.0f captureMargin %.0f endpointErr %.0f m.\n",
                    t->ut,turn.turn_sign,turn.response_time_s,
                    turn.minimum_turn_radius_m,turn.ideal_turn_radius_m,
                    turn.radius_margin_m,turn.capture_margin_m,turn.endpoint_error_m);
        }
    }

    (void)entry_program_release_nonfinal_reversal_now(g,t,p,cfg);
    bool reversal_executed=entry_program_execute_planned_reversal(g,t,p,cfg);

    /*
     * Do not manufacture an unscheduled "ordinary" reversal when topology is
     * unavailable. A reversal changes the reachable set and therefore must come
     * from the propagated S-turn plan or the physical final-turn envelope above.
     * If neither is feasible, retain the owned side and request replanning below.
     */
    bool tangent_capture_mode=g->entry_final_reversal_pending||g->entry_final_reversal_completed;
    if(tangent_capture_mode)entry_program_tangent_reversal_captured(g,t,v);
    bool final_setup_blocked=!tangent_capture_mode&&g->entry_reversal_scheduled&&
        g->entry_reversal_is_final&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,g->entry_reversal_sign);
    /* A nonfinal event whose requested sign already matches the owned S-turn side is
       not a literal reversal. It is a terminal-turn setup gate: hold that side until
       the measured-radius geometry becomes feasible. Once its deadline is reached,
       treat the wait as terminal setup for energy purposes instead of continuing to
       spend speed at entry incidence. */
    bool terminal_side_setup_blocked=!tangent_capture_mode&&g->entry_reversal_scheduled&&
        !g->entry_reversal_is_final&&g->entry_reversal_sign*g->s_turn_sign>0.0&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,g->entry_reversal_sign);
    bool terminal_geometry_blocked=entry_program_terminal_geometry_blocked(g,t,cfg);
    double parallel_stage_bank=0.0;
    bool post_shaping_parallel_stage=!tangent_capture_mode&&
        entry_program_post_shaping_parallel_stage(g,t,course,p,aero,cfg,&parallel_stage_bank);
    double geometry_bank=tangent_capture_mode?0.0:entry_taem_gate_acquisition_bank(g,t,p,aero,cfg);
    EntryControlPlan longitudinal_demand={0};
    longitudinal_demand.valid=true;
    longitudinal_demand.target_bank=(g->s_turn_sign>=0.0?1.0:-1.0)*magnitude.bank_magnitude_deg;
    longitudinal_demand.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));
    /* A missing worker topology is not permission to spend the entire entry
       energy budget at the upper end of the generic alpha schedule.  Before a
       real S-turn leg exists, preserve the lift-bearing trim through the fast
       portion of entry; the measured q/g/stall guards below still own safety.
       This keeps the alpha schedule contract unchanged while preventing the
       70 km live handoff from arriving at TAEM with no disposable energy. */
    bool topology_absent=!g->entry_topology.valid;
    bool fast_energy_leg=topology_absent&&isfinite(t->true_air_speed)&&
        t->true_air_speed>fmax(1400.0,taem_velocity*1.50)&&
        isfinite(t->dynamic_pressure)&&t->dynamic_pressure<v->maximum_dynamic_pressure*.80&&
        isfinite(t->g_force)&&t->g_force<v->maximum_g_load*.85&&
        (!t->stall_fraction_is_measured||t->stall_fraction<=.12);
    if(fast_energy_leg)
        longitudinal_demand.target_aoa=fmin(v->maximum_angle_of_attack,
            fmax(longitudinal_demand.target_aoa,v->entry_angle_of_attack+4.0));
    (void)entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&longitudinal_demand);
    /* The post-shaping runway-parallel segment is the final S-turn energy-dump
       lane even before the fixed-point 90 deg turn is geometrically flyable.
       finalaoa5 reached the -8 km station at ~1.6 km/s because this lane kept the
       old ~20 deg high-L/D incidence. The mission explicitly permits ~35 deg here. */
    bool final_s_turn_armed=tangent_capture_mode||post_shaping_parallel_stage||
        entry_program_terminal_side_final_s_turn_armed(g,t,cfg,course);
    double final_s_turn_drag_aoa=entry_program_final_s_turn_drag_aoa(g,t,p,aero,cfg,&drag,
        final_s_turn_armed);
    if(isfinite(final_s_turn_drag_aoa))
        longitudinal_demand.target_aoa=fmax(longitudinal_demand.target_aoa,final_s_turn_drag_aoa);
    double vertical_bank_magnitude=fabs(longitudinal_demand.target_bank);
    if(post_shaping_parallel_stage){
        longitudinal_demand.target_bank=parallel_stage_bank;
        longitudinal_demand.bank_cap=fabs(parallel_stage_bank);
        longitudinal_demand.target_turn_radius=fabs(parallel_stage_bank)>.25?
            live_turn_radius(t,aero,v,parallel_stage_bank):INFINITY;
    }else{
        entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank,&longitudinal_demand);
    }
    /*
     * Lateral acquisition is owned by entry_taem_gate_acquisition_bank() and
     * entry_program_apply_geometry_bank_demand(). That path computes the
     * required fixed-point capture bank from current geometry and then solves a
     * maximin allocation across vertical-lift, bounded-curvature capture-time,
     * and unpowered-energy margins. Do not overlay fixed course-rate/bank floors
     * or altitude/range windows here; such overlays bypass the same feasibility
     * model they are intended to protect.
     */
    double control_bank_magnitude=fabs(longitudinal_demand.target_bank);
    bool longitudinal_bank_replan=!tangent_capture_mode&&
        entry_program_longitudinal_bank_replan_due(g,&longitudinal_demand,t->ut,s,&limits);
    bool plan_expired=!g->entry_s_turn_plan.valid||!g->entry_control_plan_valid||
        !isfinite(g->entry_s_turn_plan.planned_ut)||g->entry_s_turn_plan.planned_ut>t->ut+1.0||
        !isfinite(g->entry_s_turn_plan.segment_duration)||
        t->ut+1e-6>=g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration;
    bool bank_authority_replan=entry_bank_authority_replan_due(g,t,v);
    bool vertical_authority_growth_replan=!bank_authority_replan&&
        entry_vertical_capture_growth_replan_due(g,t,p,aero,cfg);
    bool supervision_replan=entry_supervision_replan_due(g,t,s);
    bool alpha_safety_replan=false;
    if(g->entry_s_turn_plan.valid){
        /* Stall/g-load recovery is an execution-time AoA override below the nominal
           thermal floor. Re-solving the persistent S-turn topology cannot absorb
           that override because entry_program_plan_s_turn() deliberately reseeds
           target AoA at the thermal floor; doing so every frame creates a solve ->
           thermal-floor plan -> still-stall-limited solve loop. Bank safety still
           has its independent replan path, and the executable entry_control_aoa is
           refreshed from @ALPHA below. Dynamic-pressure protection, by contrast,
           raises the nominal target and therefore still requires a plan refresh. */
        if(alpha.limiting_reason==ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE&&
           g->entry_s_turn_plan.target_aoa<alpha.target_aoa-.25)
            alpha_safety_replan=true;
    }
    bool bank_safety_replan=g->entry_s_turn_plan.valid&&
        fabs(g->entry_s_turn_plan.target_bank)>limits.maximum_bank_deg+.25;
    bool plan_safety_replan=alpha_safety_replan||bank_safety_replan;
    bool solve_plan=state&&!tangent_capture_mode&&(reversal_executed||plan_expired||supervision_replan||
        plan_safety_replan||bank_authority_replan||vertical_authority_growth_replan||longitudinal_bank_replan);
    const char*plan_warning=NULL;

    g->entry_planning_needed=solve_plan;
    if(solve_plan&&!g->entry_planning_deferred){
        double planning_geometry_bank=post_shaping_parallel_stage?fabs(parallel_stage_bank):fabs(geometry_bank);
        EntryControlPlan planned=entry_program_plan_s_turn(g,t,state,p,aero,cfg,
            control_bank_magnitude,planning_geometry_bank,alpha.target_aoa);
        if(planned.valid){

            if(post_shaping_parallel_stage){
                /* Runway-parallel staging is not another S-turn leg. Keep the live
                   vertical-safe parallel command and do not schedule an ordinary
                   reversal away from the eventual inlet while its endpoint circle
                   is still maturing downstream. */
                planned.target_bank=parallel_stage_bank;
                planned.has_planned_reversal=false;
                planned.planned_reversal_is_final=false;
                planned.final_heading_lock=false;
            }
            double planned_sign=fabs(planned.target_bank)>=1.0?(planned.target_bank>=0?1.0:-1.0):g->s_turn_sign;
            bool initial_side_choice=g->entry_control_reversals==0&&!g->has_s_turn_leg_started&&
                !g->entry_reversal_scheduled&&!reversal_executed&&!g->entry_s_turn_plan.valid;
            if(initial_side_choice&&fabs(planned.target_bank)>=1.0)
                request_side(g,planned_sign,t->ut);
            else if(fabs(planned.target_bank)>=1.0&&planned_sign*g->s_turn_sign<0.0){
                /* Once the first leg exists, a new solve may resize it but cannot
                   invent an immediate opposite leg. Only its propagated reversal
                   milestone is allowed to change side. */
                planned.target_bank=g->s_turn_sign*fabs(planned.target_bank);
            }

            planned.planned_ut=t->ut;
            planned.segment_duration=fmax(s->s_turn_minimum_leg_duration,planned.segment_duration);
            if(bank_authority_replan||vertical_authority_growth_replan)
                planned.segment_duration=entry_authority_refresh_child_duration(g,
                    &g->entry_s_turn_plan,t->ut,planned.segment_duration,s);
            planned.final_heading_lock=false;
            control_plan_assign_lineage(g,&planned,&g->entry_s_turn_plan);
            /* A valid future boundary warning or safety-envelope replan requests a
               fresh solve without permission to postpone the durable reversal.
               Only a genuinely invalid supervisory trajectory may replace it. */
            bool terminal_geometry_durable=entry_program_terminal_geometry_blocked(g,t,cfg);
            bool replace_reversal=entry_program_replan_may_discard_reversal(
                plan_safety_replan,supervision_replan&&g->entry_committed_infeasible,
                g->entry_supervision_valid,terminal_geometry_durable);
            entry_program_commit_planned_reversal(g,&planned,t->ut,replace_reversal);
            g->entry_s_turn_plan=planned;
            g->entry_supervision_valid=false;g->entry_supervision_boundary_missed=false;
        }else{
            double fallback_sign=g->s_turn_sign>=0?1.0:-1.0;
            EntryControlPlan fallback={0};
            fallback.valid=true;fallback.terminal_ready=false;fallback.planned_ut=t->ut;
            fallback.target_bank=fallback_sign*control_bank_magnitude;
            fallback.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));fallback.target_heading=norm_deg(t->ground_track_heading);
            fallback.bank_cap=limits.maximum_bank_deg;
            fallback.target_turn_radius=live_turn_radius(t,aero,v,fallback.target_bank);
            fallback.segment_duration=fmax(5.0,s->prediction_interval*2.0);fallback.cost=INFINITY;
            fallback.taem_range_error=drag.predicted_range_error;fallback.taem_speed=NAN;
            fallback.taem_energy_error=t->predicted_taem_energy_error;
            entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&fallback);
            if(!post_shaping_parallel_stage)
                entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank,&fallback);
            control_plan_assign_lineage(g,&fallback,&g->entry_s_turn_plan);
            g->entry_s_turn_plan=fallback;
            plan_warning="MM304 trajectory planner has no finite S-turn solution at this checkpoint; holding the current side with longitudinal energy control and retrying shortly.";
            g->entry_supervision_valid=false;g->entry_supervision_boundary_missed=false;
        }
    }else if(!tangent_capture_mode&&(!state||g->entry_planning_deferred)&&(!g->entry_s_turn_plan.valid||plan_expired)){
        double fallback_sign=g->s_turn_sign>=0?1.0:-1.0;
        EntryControlPlan fallback={0};
        fallback.valid=true;fallback.planned_ut=t->ut;
        fallback.target_bank=fallback_sign*control_bank_magnitude;
        fallback.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));fallback.target_heading=norm_deg(t->ground_track_heading);
        fallback.bank_cap=limits.maximum_bank_deg;
        fallback.target_turn_radius=live_turn_radius(t,aero,v,fallback.target_bank);
        fallback.segment_duration=fmax(5.0,s->prediction_interval*2.0);fallback.cost=INFINITY;
        fallback.taem_range_error=drag.predicted_range_error;fallback.taem_speed=NAN;
        fallback.taem_energy_error=t->predicted_taem_energy_error;
        entry_program_shape_vertical_capture_plan(g,t,p,aero,cfg,&fallback);
        if(!post_shaping_parallel_stage)
            entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,geometry_bank,&fallback);
        control_plan_assign_lineage(g,&fallback,&g->entry_s_turn_plan);
        g->entry_s_turn_plan=fallback;
        g->entry_supervision_boundary_missed=false;
        plan_warning="MM304 is awaiting a propagated plan; executing current energy and geometry demand on the owned side.";
    }

    /* Normalize recovered/persistent program metadata as well as newly solved plans.
       Emergency stall/g-load recovery is a live execution override, not a thermally
       unsafe trajectory contract for predictor replay or MM304->MM305 continuity. */
    if(g->entry_s_turn_plan.valid)
        g->entry_s_turn_plan.target_aoa=fmax(g->entry_s_turn_plan.target_aoa,
            entry_thermal_protection_aoa_floor(v));
    /* A terminal roll-through is still an MM304-owned executable segment.  If
       the preceding plan expired on the same tick that the final reversal was
       admitted, an async worker result can legitimately be rejected by the
       live-state identity check.  Do not let that race erase the capture
       contract: retain a short child segment with fresh lineage until the next
       predictor result arrives. */
    if(tangent_capture_mode&&!g->entry_s_turn_plan.valid){
        EntryControlPlan capture_plan={0};
        capture_plan.valid=true;
        capture_plan.planned_ut=t->ut;
        capture_plan.segment_duration=fmax(2.0,s->prediction_interval*2.0);
        capture_plan.target_bank=g->entry_control_bank;
        capture_plan.target_aoa=fmax(g->entry_control_aoa,
            entry_thermal_protection_aoa_floor(v));
        capture_plan.target_heading=g->taem_interface_target.valid&&
            isfinite(g->taem_interface_target.course)?g->taem_interface_target.course:
            (isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading);
        capture_plan.bank_cap=limits.maximum_bank_deg;
        capture_plan.target_turn_radius=live_turn_radius(t,aero,v,capture_plan.target_bank);
        capture_plan.final_heading_lock=true;
        capture_plan.terminal_ready=false;
        capture_plan.cost=INFINITY;
        capture_plan.predicted_reversals=g->entry_control_reversals;
        control_plan_assign_lineage(g,&capture_plan,NULL);
        g->entry_s_turn_plan=capture_plan;
    }
    EntryControlPlan nominal=g->entry_s_turn_plan;
    /* The executable policy recomputes coupled energy/geometry/alpha demand on
       every telemetry tick. A worker delay never freezes an old bank or AoA. */
    if(!tangent_capture_mode&&nominal.valid){
        nominal.target_bank=longitudinal_demand.target_bank;
        nominal.target_aoa=longitudinal_demand.target_aoa;
    }
    double lateral_requirement=(tangent_capture_mode||post_shaping_parallel_stage)?0.0:
        entry_taem_gate_required_bank(g,t,p,aero,cfg);
    g->entry_lateral_bank_magnitude=magnitude.bank_magnitude_deg;
    g->entry_geometry_bank=geometry_bank;
    g->entry_vertical_bank_magnitude=vertical_bank_magnitude;
    g->entry_demand_bank=longitudinal_demand.target_bank;
    g->entry_lateral_required_bank=lateral_requirement;
    bool geometry_saturated=!post_shaping_parallel_stage&&
        fabs(geometry_bank)>control_bank_magnitude+1.0;
    bool missing_reversal=lateral_requirement*g->s_turn_sign<-1.0&&!g->entry_reversal_scheduled;
    bool blocked_final=g->entry_reversal_scheduled&&g->entry_reversal_is_final&&
        t->ut>=g->entry_reversal_ut&&!entry_program_final_reversal_geometry_ready(g,t,cfg,g->entry_reversal_sign);
    /* A same-side geometry demand above the instantaneous vertical bank budget is
       saturation, not a topological contradiction. The executable law is already
       clamped to the feasible bank and can keep building crossrange while the async
       worker searches for the reversal. Only a missing required reversal or a due
       final event that still cannot execute makes the lateral program infeasible. */
    g->entry_lateral_infeasible=missing_reversal||blocked_final;
    if(g->entry_lateral_infeasible)nominal.terminal_ready=false;
    if(geometry_saturated&&!g->entry_lateral_infeasible)
        g->entry_planning_needed=true;
    nominal.segment_duration=nominal.valid?
        fmax(0.0,g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration-t->ut):0.0;
    nominal.has_planned_reversal=g->entry_reversal_scheduled;
    nominal.planned_reversal_is_final=g->entry_reversal_scheduled&&g->entry_reversal_is_final;
    nominal.final_heading_lock=tangent_capture_mode;
    nominal.planned_reversal_ut=g->entry_reversal_ut;
    nominal.planned_reversal_range=g->entry_reversal_range;
    nominal.planned_reversal_sign=g->entry_reversal_sign;
    nominal.predicted_reversals=g->entry_control_reversals+(g->entry_reversal_scheduled?1u:0u);
    if(tangent_capture_mode){
        const TaemInterfaceTarget*q=&g->taem_interface_target;
        nominal.valid=true;
        nominal.terminal_ready=capture.ready;
        nominal.planned_ut=t->ut;
        nominal.has_planned_reversal=false;
        nominal.planned_reversal_is_final=false;
        nominal.final_heading_lock=true;
        nominal.segment_duration=fmax(1.0,s->prediction_interval*2.0);
        nominal.bank_cap=limits.maximum_bank_deg;
        nominal.target_aoa=fmax(alpha.target_aoa,entry_thermal_protection_aoa_floor(v));
        /* Tangent capture is the actual final S-turn. Preserve an explicit
           excess-energy drag command here instead of replacing it with the
           conservative alpha scheduler during the terminal roll-through. */
        if(isfinite(final_s_turn_drag_aoa))
            nominal.target_aoa=fmax(nominal.target_aoa,final_s_turn_drag_aoa);
        if(q->valid){
            double live_course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
            double terminal_course_error=isfinite(live_course)?norm_signed_deg(q->course-live_course):0.0;
            if(!g->entry_topology_heading_locked&&isfinite(live_course)&&fabs(terminal_course_error)<=8.0)
                g->entry_topology_heading_locked=true;
            /* During the physical terminal turn, acquire the published inlet course
               first.  Using the tangent-line intercept immediately can point 20-30 deg
               away from q->course when the orbiter is still far upstream; v14 then
               relaxed a correct reversal into ~76 deg course and bled below 600 m/s.
               Once the course is actually acquired, switch to the Frenet/line capture
               law for the fixed -8 km station. */
            nominal.target_heading=g->entry_topology_heading_locked?
                entry_taem_tangent_capture_heading(g,t,cfg):q->course;
            if(g->entry_final_reversal_pending){
                /* Do not let the point-capture law cancel the actual final roll-through.
                   Hold the committed opposite side until measured bank has crossed it. */
                double reversal_mag=fmax(4.0,fabs(g->entry_control_bank));
                nominal.target_bank=g->s_turn_sign*fmin(limits.maximum_bank_deg,reversal_mag);
            }else{
                double tangent_geometry_bank=entry_taem_tangent_capture_bank(g,t,aero,cfg,nominal.target_heading);
                double recovery_bearing=NAN;
                bool point_recovery_required=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
                (void)recovery_bearing;
                /* After a normal perpendicular-course capture, allow only a bounded
                   opposite-bank line correction. A missed fixed station is different:
                   MM304 must retain full demonstrated turning authority to reacquire
                   the point instead of continuing straight down the infinite tangent. */
                if(!point_recovery_required&&tangent_geometry_bank*g->s_turn_sign<0.0&&
                   fabs(tangent_geometry_bank)>18.0)
                    tangent_geometry_bank=-g->s_turn_sign*18.0;
                bool heading_lock_required=!g->entry_topology_heading_locked&&isfinite(live_course)&&
                    fabs(terminal_course_error)>8.0;
                nominal.target_bank=tangent_geometry_bank;
                (void)entry_program_altitude_capture(g,t,p,aero,cfg,&nominal.target_bank,&nominal.target_aoa);
                bool preserve_lateral=heading_lock_required||point_recovery_required;
                if(preserve_lateral&&fabs(tangent_geometry_bank)>.25){
                    /* Course/point capture is a hard MM304 handoff condition. Do not
                       let the vertical allocator erase the bank needed to satisfy it;
                       q/g/stall/dynamic-bank limits are already in the bank command. */
                    nominal.target_bank=tangent_geometry_bank;
                }else{
                    double vertical_ceiling=entry_program_vertical_bank_ceiling(g,t,p,aero,cfg);
                    if(isfinite(vertical_ceiling)&&fabs(nominal.target_bank)>vertical_ceiling)
                        nominal.target_bank=copysign(vertical_ceiling,nominal.target_bank);
                    entry_program_apply_geometry_bank_demand(g,t,p,aero,cfg,tangent_geometry_bank,&nominal);
                }
                if(point_recovery_required&&t->true_air_speed<s->taem_force_handoff_speed+350.0){
                    double efficient_floor=entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v);
                    nominal.target_aoa=fmin(nominal.target_aoa,fmax(15.0,efficient_floor));
                }
                if(fabs(tangent_geometry_bank)>fabs(nominal.target_bank)+1.0)
                    g->entry_lateral_infeasible=true;
            }
            nominal.target_turn_radius=live_turn_radius(t,aero,v,nominal.target_bank);
        }else{
            /* A final reversal without its precomputed inlet must not reopen ordinary
               S-turn topology. Hold a bounded neutral recovery while MM304 remains owner. */
            nominal.target_heading=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
            nominal.target_bank=0.0;
            nominal.target_turn_radius=INFINITY;
        }
    }

    const char*supervision_warning=NULL;
    /* Supervision is a worker-side replay of this policy. The legacy optimizer's
       private alpha/bank corrections cannot certify or alter executable MM304. */
    if(!tangent_capture_mode&&(!isfinite(g->entry_supervision_ut)||
       t->ut-g->entry_supervision_ut>=s->prediction_interval))g->entry_planning_needed=true;
    if(g->entry_lateral_infeasible){
        nominal.terminal_ready=false;g->entry_s_turn_plan.terminal_ready=false;
        if(!tangent_capture_mode)g->entry_planning_needed=true;
    }
    if(g->entry_lateral_infeasible)
        supervision_warning="MM304 lateral/vertical budgets conflict with current lift or an uncommitted reversal; tangent delivery is unproven and requires replanning.";
    double bank=nominal.target_bank,aoa=nominal.target_aoa;
    /* Below ~Mach 4, after the committed shaping reversal, TAEM energy reserve
       is an explicit mission requirement. The ordinary low-q belly-first floor
       was re-clamping the executable command to ~18 deg even when the terminal
       leg requested 15 deg, throwing away the speed needed by MM305. Retain the
       normal thermal floor everywhere else and retain every live q/g/stall
       protection here. */
    bool alpha_emergency_override=alpha.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN||
        alpha.limiting_reason==ENTRY_ALPHA_LIMIT_G_LOAD;
    bool terminal_energy_preservation=!tangent_capture_mode&&
        (g->entry_control_reversals>0||final_setup_blocked||terminal_side_setup_blocked||
         terminal_geometry_blocked)&&
        q->valid&&!capture.ready&&t->true_air_speed<s->taem_force_handoff_speed+350.0&&
        t->dynamic_pressure>=fmax(250.0,v->maximum_dynamic_pressure*.008)&&t->dynamic_pressure<v->maximum_dynamic_pressure*.85&&
        t->g_force<v->maximum_g_load*.85&&
        (!t->stall_fraction_is_measured||t->stall_fraction<.10);
    double protective_aoa=terminal_energy_preservation?
        fmax(15.0,entry_terminal_turn_aoa_floor(t->true_air_speed,t->dynamic_pressure,v)):
        entry_low_q_protective_aoa_floor(t->dynamic_pressure,v);
    if(terminal_energy_preservation&&!isfinite(final_s_turn_drag_aoa))
        aoa=fmin(aoa,protective_aoa);
    if(alpha_emergency_override)aoa=fmin(aoa,alpha.target_aoa);
    else{
        aoa=fmax(aoa,protective_aoa);
        /* A replan is not a safety guarantee: it may be unavailable, fail, or
           be followed by a cached negative supervisory correction. Enforce
           ALPHA's live q protection on execution as well as requesting a solve. */
        if(alpha.limiting_reason==ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE)
            aoa=fmax(aoa,alpha.target_aoa);
    }
    bank=clampd(bank,-limits.maximum_bank_deg,limits.maximum_bank_deg);
    double executable_aoa_ceiling=isfinite(final_s_turn_drag_aoa)?
        entry_final_s_turn_aoa_ceiling(v):v->maximum_angle_of_attack;
    aoa=clampd(aoa,0.0,executable_aoa_ceiling);

    /* `entry_s_turn_plan` is the persistent program. The legacy entry_control_*
       fields deliberately remain the exact command currently executed, because
       controller.c publishes/replays those fields in the live trajectory forecast. */
    g->entry_control_plan_valid=nominal.valid;
    g->entry_control_terminal_ready=nominal.terminal_ready;
    g->entry_control_plan_ut=t->ut;
    g->entry_control_bank=bank;g->entry_control_aoa=aoa;g->entry_control_heading=nominal.target_heading;
    g->entry_control_turn_radius=live_turn_radius(t,aero,v,bank);
    g->entry_control_cost=nominal.cost;
    g->entry_control_taem_range_error=nominal.taem_range_error;
    g->entry_control_taem_speed=nominal.taem_speed;
    g->entry_control_taem_energy_error=nominal.taem_energy_error;
    g->entry_control_segment_until_ut=tangent_capture_mode?t->ut+nominal.segment_duration:
        (g->entry_s_turn_plan.valid?g->entry_s_turn_plan.planned_ut+g->entry_s_turn_plan.segment_duration:t->ut);

    /* Reversal dwell starts only on a continuously captured, aerodynamically
       loaded S-turn leg; pre-entry roll excursions and wings-level segments do not count. */
    if(!tangent_capture_mode)entry_update_s_turn_leg_capture(g,t,bank,v);

    g->airbrakes_deployed=false;
    GuidanceCommand c=atmospheric(t,nominal.target_heading,bank,v,0,false,PROFILE_ENTRY);
    c.heading_control_enabled=false;c.has_target_aoa=true;c.target_aoa=aoa;c.target_pitch=t->flight_path_angle+aoa;
    char status[448];
    double plan_remaining=fmax(0.0,g->entry_control_segment_until_ut-t->ut);
    double reversal_remaining=g->entry_reversal_scheduled&&isfinite(g->entry_reversal_ut)?
        g->entry_reversal_ut-t->ut:NAN;
    double terminal_arc_error=NAN,terminal_arc_radius=NAN;
    if(g->taem_interface_target.valid){
        double arc_side=g->entry_reversal_scheduled?
            (g->entry_reversal_sign>=0.0?1.0:-1.0):
            (norm_signed_deg(g->taem_interface_target.course-cfg->site.runway_heading)>=0.0?1.0:-1.0);
        (void)entry_program_terminal_course_endpoint_ready(g,t,cfg,arc_side,
            g->taem_interface_target.course,&terminal_arc_error,&terminal_arc_radius);
    }
    if(tangent_capture_mode){
        const TaemInterfaceTarget*q=&g->taem_interface_target;
        double gate_distance=q->valid?hypot(q->along_track-t->runway_along_track,
            q->cross_track-t->runway_cross_track):NAN;
        snprintf(status,sizeof(status),"MM304 tangent capture: gate %.1f km, course %.1f -> %.1f deg, bank %+.1f deg (roll %+.1f), h %.1f/%.1f km, V %.0f/%.0f m/s, final reversal %s.",
            gate_distance/1000.0,t->ground_track_heading,nominal.target_heading,bank,norm_signed_deg(t->roll),
            t->mean_altitude/1000.0,q->valid?q->altitude/1000.0:NAN,t->true_air_speed,q->valid?q->speed:NAN,
            g->entry_final_reversal_pending?"rolling":"captured");
    }else snprintf(status,sizeof(status),"MM304 %s planned S-turn: Dref %.2f m/s2, dD %+.2f, bank %+.1f/Bvert %.1f/Bgeo %.1f/Bdem %.1f deg, roll %+.1f leg %d, Crate %+.2f dps Beff %.2f Rturn %.1f km, alpha %.1f deg, plan %.1fs, reversal %s%.1fs, arcErr %.1f km/R %.1f km.",
        entry_phase_string(g->entry_exec.phase),drag.reference_drag_accel,drag.drag_error_accel,
        bank,vertical_bank_magnitude,fabs(geometry_bank),fabs(longitudinal_demand.target_bank),
        norm_signed_deg(t->roll),g->has_s_turn_leg_started?1:0,
        t->has_course_rate?t->course_rate:t->heading_rate,t->bank_effectiveness,
        isfinite(nominal.target_turn_radius)?nominal.target_turn_radius/1000.0:NAN,
        aoa,plan_remaining,g->entry_reversal_scheduled?"in ":"none/",
        g->entry_reversal_scheduled?reversal_remaining:0.0,
        isfinite(terminal_arc_error)?terminal_arc_error/1000.0:NAN,
        isfinite(terminal_arc_radius)?terminal_arc_radius/1000.0:NAN);
    const char*warning=alpha_emergency_override?
        (alpha.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN?
            "MM304 thermal AoA floor temporarily overridden by explicit stall-margin recovery.":
            "MM304 thermal AoA floor temporarily overridden by explicit g-load recovery."):
        (supervision_warning?supervision_warning:plan_warning);
    if(!warning&&(drag.degraded||alpha.degraded||magnitude.degraded_authority))
        warning="MM304 planned S-turn is operating with degraded aerodynamic/reference authority; the current side is preserved while commands stay inside the demonstrated envelope.";
    return stabilized(g,result_make(PHASE_ENTRY_ENERGY,c,status,warning),t,v,s,dt);
}
static double live_turn_radius(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v,double bank_deg){
    double lift=live_lift_accel(t,aero,v),eff=clampd(t->bank_effectiveness,.35,1.8);
    double lateral=lift*fabs(sin(clampd(fabs(bank_deg)*eff,0,89)*DEG2RAD));
    return lateral>.005?t->true_air_speed*t->true_air_speed/lateral:INFINITY;
}
static double terminal_hac_radius_live(const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s){
    double usable_bank=fmin(55.0,dynamic_bank_limit(t,v));
    double min_radius=live_turn_radius(t,aero,v,usable_bank);
    double max_terminal_radius=fmax(s->hac_radius,fmin(p->radius*.20,fmax(s->hac_radius*8.0,s->taem_interface_range*2.5)));
    double comfort_radius=t->true_air_speed/(hac_course_rate_cap(t)*DEG2RAD);
    if(!isfinite(min_radius)||min_radius*1.20>max_terminal_radius||comfort_radius>max_terminal_radius)return INFINITY;
    return clampd(fmax(comfort_radius,fmax(s->hac_radius,min_radius*1.30)),s->hac_radius,max_terminal_radius);
}
static void terminal_glide_initialize(GuidanceMachine*g,const VehicleProfile*v,const GuidanceSettings*s){
    g->terminal_glide_mode=true;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->hac_plan_degraded=false;
    g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;
    g->hac_plan_violation_score=0;
    g->hac_commit_blend=0;
    g->terminal_energy_sample_valid=false;
    g->terminal_previous_specific_energy=0;
    g->terminal_previous_speed=0;
    g->terminal_energy_sample_ut=0;
    g->terminal_energy_loss_accel_ema=0;
    g->terminal_speed_loss_accel_ema=0;
    g->terminal_test_preflare_altitude=clampd(fmax(500.0,s->flare_altitude*8.0),450.0,750.0);
    g->terminal_test_preflare_target_speed=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.40);
    g->terminal_test_preflare_min_speed=fmax(v->final_approach_speed*1.03,v->minimum_safe_speed*1.25);
    g->terminal_test_glide_slope=22.0;
    g->terminal_test_final_approach_distance=s->final_approach_distance;
}

static void terminal_energy_observe(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p){
    if(!g||!t||!p||!g->terminal_glide_mode)return;
    double specific=rotating_specific_energy(t->latitude,t->mean_altitude,t->true_air_speed,p);
    if(!isfinite(specific)||!isfinite(t->ut)||!isfinite(t->true_air_speed))return;
    if(g->terminal_energy_sample_valid){
        double dt=t->ut-g->terminal_energy_sample_ut;
        if(dt>=.08&&dt<=2.5){
            double mean_speed=fmax(30.0,.5*(g->terminal_previous_speed+t->true_air_speed));
            double energy_equiv=(g->terminal_previous_specific_energy-specific)/dt/mean_speed;
            double speed_loss=(g->terminal_previous_speed-t->true_air_speed)/dt;
            if(isfinite(energy_equiv)&&energy_equiv>=0&&energy_equiv<60){
                double a=clampd(dt/(2.2+dt),.03,.45);
                g->terminal_energy_loss_accel_ema=g->terminal_energy_loss_accel_ema>0?
                    g->terminal_energy_loss_accel_ema+(energy_equiv-g->terminal_energy_loss_accel_ema)*a:energy_equiv;
            }
            if(isfinite(speed_loss)&&speed_loss>=0&&speed_loss<60){
                double a=clampd(dt/(1.8+dt),.03,.50);
                g->terminal_speed_loss_accel_ema=g->terminal_speed_loss_accel_ema>0?
                    g->terminal_speed_loss_accel_ema+(speed_loss-g->terminal_speed_loss_accel_ema)*a:speed_loss;
            }
        }
    }
    g->terminal_previous_specific_energy=specific;
    g->terminal_previous_speed=t->true_air_speed;
    g->terminal_energy_sample_ut=t->ut;
    g->terminal_energy_sample_valid=true;
}

static double terminal_expected_energy_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v){
    /*
     * Specific mechanical energy lost per metre of air path is D/m. Prefer the
     * larger of the instantaneous measured/modelled drag acceleration and the
     * observed mechanical-energy-loss EMA when both exist. No confidence
     * percentage or safety multiplier is translated into extra drag here;
     * uncertainty is carried separately by EnergyPathEnvelope.
     */
    double live=live_drag_accel(t,aero,v);
    if(!isfinite(live)||live<0.0)return NAN;
    if(g&&isfinite(g->terminal_energy_loss_accel_ema)&&
       g->terminal_energy_loss_accel_ema>=0.0)
        return fmax(live,g->terminal_energy_loss_accel_ema);
    return live;
}

static double terminal_expected_speed_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v){
    (void)g;
    /* Aerodynamic energy loss is not speed loss. During the recorded F14
       -32 degree glide, D/m was about 5.2 m/s2 while TAS stayed near 200 m/s:
       descending potential energy paid the drag work. The former 0.85*D/m
       floor predicted 133 m/s twenty seconds later; live KSP retained 198.
       Keep signed acceleration rather than inventing deceleration in a dive. */
    double radius=fmax(p->radius+t->mean_altitude,1.0);
    double latitude=t->latitude*DEG2RAD;
    double gravity=p->gravitational_parameter/(radius*radius)-
        p->rotational_speed*p->rotational_speed*radius*cos(latitude)*cos(latitude);
    return clampd(live_drag_accel(t,aero,v)+gravity*sin(t->flight_path_angle*DEG2RAD),-40.0,40.0);
}

/* MM305 owns phase progression independently of the coarse public
   GuidancePhase. The optional TAEM S-turn is enabled only when the provider has
   a valid signed surplus and the current terminal solution is known infeasible.
   Zero surplus is the balance boundary; no tuned entry/exit hysteresis belongs in
   the executive. Ordinary MM304 reversals remain a separate Entry mechanism. */
static TaemExecProfile taem_exec_profile_production(const GuidanceSettings*s,
        bool terminal_hac_rehearsal){
    (void)s;
    TaemExecProfile profile={0};
    /* A HAC-only rehearsal starts after MM304 and exists to exercise the
       terminal path provider itself.  Its checkpoint-style initialization
       must not insert the optional MM305 energy S-turn before that provider
       gets a chance to evaluate the live HAC.  Normal production TAEM keeps
       the S-turn option enabled. */
    profile.s_turn_enabled=!terminal_hac_rehearsal;
    return profile;
}

static TaemExecInputs taem_exec_inputs_live_with_contract(const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s,const TaemTerminalContract*terminal_contract,
        bool has_final_approach_override,bool final_approach_override){
    TaemExecInputs inputs={0};
    inputs.mm304_complete=g->entry_exec.entry_complete;
    /* energy_excess_range is an ENTRY-to-TAEM altitude-crossing proxy.
       At/below that altitude its time-to-go is zero, reducing the proxy to
       target range minus current range regardless of speed or actual energy.
       The unpowered 8 km replay therefore saw "surplus" grow while stalling.
       Below the crossing, terminal-path energy/geometry must own guidance. */
    inputs.energy_valid=!g->taem_interface_captured&&isfinite(t->energy_excess_range)&&
        isfinite(t->mean_altitude)&&t->mean_altitude>s->taem_interface_altitude;
    inputs.energy_excess=t->energy_excess_range;
    double shell_min=0.0,shell_max=0.0;
    entry_taem_handoff_altitude_bounds(s,&shell_min,&shell_max);
    bool high_energy_staging=inputs.energy_valid&&
        inputs.energy_excess>0.0&&t->mean_altitude>shell_max;
    bool candidate_nominal=g->terminal_candidate.valid&&
        !g->terminal_candidate.geometry_degraded&&!g->terminal_candidate.energy_degraded;
    /* A future-path preview cannot waive a positive live range/energy surplus
       while the vehicle is still above the TAEM altitude shell. The 50 km replay
       reached V_TAEM at 28.5 km with +101 km excess range; accepting that preview
       spent the surplus in a steep terminal dive. Above the shell, the sign of the
       directly observed surplus declares that nominal terminal staging has not yet
       been reached. Once inside the shell, the terminal candidate owns feasibility. */
    inputs.terminal_feasibility_valid=g->terminal_candidate.valid||high_energy_staging;
    inputs.nominal_terminal_path_feasible=candidate_nominal&&!high_energy_staging;
    /* HAC/circle/spline flags are private path-provider diagnostics.  TAEMEXEC
       consumes only generic terminal path progress and remains the sole owner. */
    inputs.terminal_path_selected=g->terminal_path_committed||g->hac_side_selected;
    inputs.terminal_path_captured=g->hac_captured&&!g->hac_transition_active;
    inputs.final_intercept_ready=g->hac_completed;
    inputs.final_approach_ready=has_final_approach_override?
        final_approach_override:g->final_approach_captured;
    if(terminal_contract)inputs.terminal_contract=*terminal_contract;
    inputs.off_nominal_recovery_active=g->attitude_recovery;
    inputs.off_nominal_recovery_reason=g->attitude_recovery?
        TAEM_RECOVERY_ATTITUDE:TAEM_RECOVERY_NONE;
    return inputs;
}

static TaemExecInputs taem_exec_inputs_live(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    return taem_exec_inputs_live_with_contract(g,t,s,NULL,false,false);
}

static bool taem_exec_enter(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,bool checkpoint_resume){
    if(g->taem_exec.initialized&&g->taem_exec.ownership_latched)return true;
    TaemExecObservation observation={
        .ut=t->ut,.relative_velocity=t->true_air_speed,.checkpoint_restart=checkpoint_resume
    };
    TaemExecInputs inputs=taem_exec_inputs_live(g,t,s);
    inputs.resume_mm305=checkpoint_resume;
    if(checkpoint_resume)inputs.mm304_complete=false;
    TaemExecProfile profile=taem_exec_profile_production(s,
        g->terminal_rehearsal_mode);
    return taem_exec_initialize(&g->taem_exec,&observation,&inputs,&profile);
}

static bool taem_exec_sync_with_contract(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const TaemTerminalContract*terminal_contract,bool has_final_approach_override,
        bool final_approach_override){
    if(!g->taem_exec.initialized||!g->taem_exec.ownership_latched)return false;
    TaemExecObservation observation={.ut=t->ut,.relative_velocity=t->true_air_speed,.checkpoint_restart=false};
    TaemExecInputs inputs=taem_exec_inputs_live_with_contract(g,t,s,terminal_contract,
        has_final_approach_override,final_approach_override);
    TaemExecProfile profile=taem_exec_profile_production(s,
        g->terminal_rehearsal_mode);
    bool updated=taem_exec_update(&g->taem_exec,&observation,&inputs,&profile);
    if(g->taem_exec.phase!=TAEM_PHASE_S_TURN)g->taem_s_turn_plan.valid=false;
    return updated;
}

static void taem_exec_sync(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    (void)taem_exec_sync_with_contract(g,t,s,NULL,false,false);
}


typedef struct {
    bool valid;
    double along_track, cross_track, course, altitude, speed, specific_energy;
    double peak_dynamic_pressure_ratio, peak_g_ratio;
} TaemSTurnProjection;

static TaemSTurnProjection taem_s_turn_project_segment(const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        const LandingSite*site,double target_bank,double target_aoa,double duration){
    TaemSTurnProjection out={0};
    if(!t||!p||!v||!s||!site||!isfinite(duration)||duration<=0)return out;
    GeoPoint origin={site->latitude,site->longitude,site->altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e=0.0,n=0.0;local_offsets(origin,current,p->radius,&e,&n);
    double bank=norm_signed_deg(t->roll);
    double bank_rate=controlled_roll_rate(t);
    if(!isfinite(bank_rate))bank_rate=0.0;
    double aoa=isfinite(t->angle_of_attack)?t->angle_of_attack:target_aoa;
    double speed=fmax(1.0,t->true_air_speed);
    double altitude=t->mean_altitude;
    double gamma=clampd(t->flight_path_angle,-45.0,20.0)*DEG2RAD;
    double heading=course;
    double bank_eff=clampd(t->bank_effectiveness>0?t->bank_effectiveness:1.0,.35,1.8);
    double roll_rate_limit=fmax(2.0,s->taem_roll_rate);
    double roll_accel_limit=fmax(1.0,s->entry_roll_acceleration*.85);
    double live_lift=fmax(.02,live_lift_accel(t,aero,v));
    double live_drag=fmax(.02,live_drag_accel(t,aero,v));
    double rho0=planet_atmospheric_density(p,t->mean_altitude);
    double density_scale=t->atmospheric_density>0&&rho0>1e-9?
        clampd(t->atmospheric_density/rho0,.35,2.8):clampd(t->trajectory_density_scale,.35,2.8);
    double sound0=t->speed_of_sound>1?t->speed_of_sound:planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double current_incidence=hypot(t->angle_of_attack,t->sideslip);
    double current_lf=0.0,current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,&current_lf,&current_df);
    double q0=fmax(1.0,t->dynamic_pressure);
    double peak_q=fmax(0.0,t->dynamic_pressure);
    double peak_load=fmax(0.0,t->g_force);
    int steps=(int)ceil(duration/.20);if(steps<1)steps=1;if(steps>320)steps=320;
    double step=duration/(double)steps;
    for(int i=0;i<steps;i++){
        double bank_error=norm_signed_deg(target_bank-bank);
        double desired_rate=clampd(bank_error/1.05,-roll_rate_limit,roll_rate_limit);
        bank_rate+=clampd(desired_rate-bank_rate,-roll_accel_limit*step,roll_accel_limit*step);
        bank=clampd(bank+bank_rate*step,-v->maximum_bank_angle,v->maximum_bank_angle);
        aoa+=clampd(target_aoa-aoa,-4.5*step,4.5*step);

        double rho=planet_atmospheric_density(p,altitude)*density_scale;
        double sound=planet_atmospheric_speed_of_sound(p,altitude);
        if(sound<=1)sound=sound0;
        double mach=speed/fmax(sound,1.0);
        double lf=0.0,df=1.0;
        aerodynamic_force_factors_mach(mach,fabs(aoa),v,&lf,&df);
        double q=.5*fmax(0.0,rho)*speed*speed;
        double q_ratio=q/q0;
        double lift=live_lift*q_ratio*fabs(lf)/fmax(.05,fabs(current_lf));
        double drag=live_drag*q_ratio*fmax(.05,df)/fmax(.05,current_df);
        if(!isfinite(lift)||lift<0)lift=live_lift;
        if(!isfinite(drag)||drag<.01)drag=live_drag;
        lift=clampd(lift,0.0,80.0);drag=clampd(drag,.01,60.0);
        double gravity=p->gravitational_parameter/
            fmax(1.0,(p->radius+altitude)*(p->radius+altitude));
        double effective_bank=clampd(bank*bank_eff,-89.0,89.0)*DEG2RAD;
        double vertical_lift=lift*cos(effective_bank);
        double lateral_lift=lift*sin(effective_bank);
        double gamma_rate=(vertical_lift-gravity*cos(gamma))/fmax(speed,30.0)+
            speed*cos(gamma)/fmax(p->radius+altitude,1.0);
        gamma=clampd(gamma+gamma_rate*step,-45.0*DEG2RAD,20.0*DEG2RAD);
        double horizontal=fmax(20.0,speed*cos(gamma));
        double course_rate=lateral_lift/horizontal;
        heading=norm_deg(heading+course_rate*RAD2DEG*step);
        double mid=heading-.5*course_rate*RAD2DEG*step;
        e+=horizontal*sin(mid*DEG2RAD)*step;
        n+=horizontal*cos(mid*DEG2RAD)*step;
        altitude+=speed*sin(gamma)*step;
        speed=fmax(v->minimum_safe_speed*.90,
            speed+(-drag-gravity*sin(gamma))*step);
        peak_q=fmax(peak_q,q);
        peak_load=fmax(peak_load,hypot(lift,drag)/fmax(gravity,.1));
    }
    GeoPoint end=local_point(origin,e,n,p->radius,altitude);
    runway_coordinates(end,origin,site->runway_heading,p->radius,&out.along_track,&out.cross_track);
    out.course=heading;out.altitude=altitude;out.speed=speed;
    out.specific_energy=rotating_specific_energy(end.latitude,altitude,speed,p);
    out.peak_dynamic_pressure_ratio=peak_q/fmax(v->maximum_dynamic_pressure,1.0);
    out.peak_g_ratio=peak_load/fmax(v->maximum_g_load,.1);
    out.valid=isfinite(out.along_track)&&isfinite(out.cross_track)&&isfinite(out.course)&&
        isfinite(out.altitude)&&isfinite(out.speed)&&isfinite(out.specific_energy);
    return out;
}

static void taem_s_turn_target(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        const LandingConfiguration*cfg,double*target_along,double*target_cross,double*target_altitude,
        double*target_speed,double*target_course){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double shell_min=0.0,shell_max=0.0;entry_taem_handoff_altitude_bounds(s,&shell_min,&shell_max);
    double reserve=fmax(s->hac_radius*2.0,s->taem_interface_range*.65);
    *target_along=-s->final_approach_distance-reserve;
    *target_cross=0.0;
    *target_altitude=.5*(shell_min+shell_max);
    *target_speed=entry_taem_speed_target(v,s,p);
    *target_course=cfg->site.runway_heading;
    if(g->terminal_candidate.valid&&!g->terminal_candidate.geometry_degraded){
        const TerminalCandidate*c=&g->terminal_candidate;
        if(c->join.valid){
            double h=cfg->site.runway_heading*DEG2RAD;
            *target_along=c->join.lead_start.e*sin(h)+c->join.lead_start.n*cos(h);
            *target_cross=c->join.lead_start.e*cos(h)-c->join.lead_start.n*sin(h);
        }
        if(isfinite(c->altitude)&&c->altitude>cfg->site.altitude)*target_altitude=c->altitude;
        if(isfinite(c->speed)&&c->speed>v->minimum_safe_speed)*target_speed=c->speed;
        if(isfinite(c->course))*target_course=c->course;
    }
    (void)t;
}

typedef struct {
    bool valid;
    double safety_violation;
    double energy_deficit;
    double energy_excess;
    double position_error;
    double course_error;
    double altitude_error;
    double speed_error;
    double control_effort;
} TaemSTurnRank;

static bool taem_s_turn_rank_better(const TaemSTurnRank*a,
        const TaemSTurnRank*b){
    if(!a||!a->valid)return false;
    if(!b||!b->valid)return true;
    if(a->safety_violation<b->safety_violation)return true;
    if(a->safety_violation>b->safety_violation)return false;
    if(a->energy_deficit<b->energy_deficit)return true;
    if(a->energy_deficit>b->energy_deficit)return false;
    if(a->energy_excess<b->energy_excess)return true;
    if(a->energy_excess>b->energy_excess)return false;
    if(a->position_error<b->position_error)return true;
    if(a->position_error>b->position_error)return false;
    if(a->course_error<b->course_error)return true;
    if(a->course_error>b->course_error)return false;
    if(a->altitude_error<b->altitude_error)return true;
    if(a->altitude_error>b->altitude_error)return false;
    if(a->speed_error<b->speed_error)return true;
    if(a->speed_error>b->speed_error)return false;
    return a->control_effort<b->control_effort;
}

static EntryControlPlan taem_plan_s_turn_segment(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg,
        TaemSTurnProjection*chosen_projection){
    EntryControlPlan best={0};
    best.cost=INFINITY;
    best.planned_ut=t->ut;
    if(chosen_projection)memset(chosen_projection,0,
        sizeof(*chosen_projection));

    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    double target_along=0.0,target_cross=0.0,target_alt=0.0;
    double target_speed=0.0,target_course=0.0;
    taem_s_turn_target(g,t,p,cfg,&target_along,&target_cross,
        &target_alt,&target_speed,&target_course);

    double excess=fmax(0.0,t->energy_excess_range);
    double horizontal=fmax(t->horizontal_speed,DBL_MIN);
    double minimum_leg=fmax(0.0,s->s_turn_minimum_leg_duration);

    /*
     * One symmetric S-turn consists of two opposite-sign legs.  Divide the
     * extra path demand between those two legs; this is geometry, not a tuned
     * duration multiplier.  An inherited MM304 leg contributes its already
     * flown time to the first MM305 leg.
     */
    double full_leg_duration=fmax(minimum_leg,
        excess/(2.0*horizontal)); /* decision-literal-ok: two symmetric S-turn legs */

    if(g->taem_interface_target.valid&&
       isfinite(g->taem_interface_target.along_track)&&
       isfinite(t->runway_along_track)&&isfinite(course)&&
       isfinite(cfg->site.runway_heading)){
        double station_forward=
            g->taem_interface_target.along_track-t->runway_along_track;
        double relative=norm_signed_deg(
            course-cfg->site.runway_heading)*DEG2RAD;
        double station_closing=horizontal*cos(relative);
        if(station_forward>0.0&&station_closing>DBL_MIN){
            double station_time=station_forward/station_closing;
            if(isfinite(station_time))
                full_leg_duration=fmin(full_leg_duration,
                    fmax(0.0,station_time));
        }
    }

    double sign=g->s_turn_sign>=0.0?1.0:-1.0;
    double inherited_leg_elapsed=
        g->has_s_turn_leg_started&&
        isfinite(g->s_turn_leg_started_ut)&&
        t->ut>=g->s_turn_leg_started_ut?
            fmax(0.0,t->ut-g->s_turn_leg_started_ut):0.0;
    double remaining_leg=fmax(0.0,
        full_leg_duration-inherited_leg_elapsed);

    bool inherited_reversal=g->entry_reversal_scheduled&&
        isfinite(g->entry_reversal_ut)&&
        g->entry_reversal_sign*sign<0.0;
    if(inherited_reversal)
        remaining_leg=fmin(remaining_leg,
            fmax(0.0,g->entry_reversal_ut-t->ut));

    bool planned_reversal=excess>0.0||inherited_reversal;
    double leg_duration=remaining_leg;
    if(!(leg_duration>0.0))leg_duration=
        fmax(DBL_EPSILON,minimum_leg);

    double start_energy=rotating_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,p);
    double end_energy=rotating_specific_energy(
        cfg->site.latitude,target_alt,target_speed,p);
    double energy_span=fmax(fabs(start_energy-end_energy),DBL_MIN);
    double direct=hypot(t->runway_along_track-target_along,
        t->runway_cross_track-target_cross);
    double altitude_span=fmax(fabs(t->mean_altitude-target_alt),DBL_MIN);
    double speed_span=fmax(fabs(t->true_air_speed-target_speed),DBL_MIN);

    double max_bank=dynamic_bank_limit(t,v);
    max_bank=fmin(max_bank,v->maximum_bank_angle);
    if(!(max_bank>0.0))return best;

    double aoa_floor=entry_low_q_protective_aoa_floor(
        t->dynamic_pressure,v);
    double aoa_ceiling=v->maximum_angle_of_attack;
    if(aoa_ceiling<aoa_floor)aoa_ceiling=aoa_floor;

    const int bank_samples=7; /* numerical control-space resolution */
    const int aoa_samples=5;
    TaemSTurnRank best_rank={0};

    for(int bi=0;bi<bank_samples;bi++){
        double bank=sign*max_bank*
            (double)(bi+1)/(double)bank_samples;
        for(int ai=0;ai<aoa_samples;ai++){
            double aoa=aoa_floor+
                (aoa_ceiling-aoa_floor)*
                (double)ai/(double)(aoa_samples-1);

            TaemSTurnProjection pr=taem_s_turn_project_segment(
                t,course,p,aero,v,s,&cfg->site,
                bank,aoa,leg_duration);
            if(!pr.valid)continue;

            TaemSTurnRank rank={0};
            rank.valid=true;
            rank.safety_violation=fmax(
                fmax(0.0,pr.peak_dynamic_pressure_ratio-1.0),
                fmax(0.0,pr.peak_g_ratio-1.0));
            double energy_delta=pr.specific_energy-end_energy;
            rank.energy_deficit=fmax(0.0,-energy_delta)/energy_span;
            rank.energy_excess=fmax(0.0,energy_delta)/energy_span;
            rank.position_error=hypot(
                pr.along_track-target_along,
                pr.cross_track-target_cross)/fmax(direct,DBL_MIN);
            rank.course_error=fabs(norm_signed_deg(
                pr.course-target_course))/180.0;
            rank.altitude_error=fabs(
                pr.altitude-target_alt)/altitude_span;
            rank.speed_error=fabs(
                pr.speed-target_speed)/speed_span;
            rank.control_effort=fmax(
                fabs(bank)/fmax(v->maximum_bank_angle,DBL_MIN),
                fabs(aoa)/fmax(v->maximum_angle_of_attack,DBL_MIN));

            if(!best.valid||taem_s_turn_rank_better(
                    &rank,&best_rank)){
                best_rank=rank;
                best.valid=true;
                best.terminal_ready=false;
                best.planned_ut=t->ut;
                best.target_bank=bank;
                best.target_aoa=aoa;
                best.target_heading=target_course;
                best.bank_cap=max_bank;
                best.target_turn_radius=live_turn_radius(
                    t,aero,v,bank);
                best.segment_duration=leg_duration;
                best.cost=fmax(rank.safety_violation,
                    fmax(rank.energy_deficit,
                        fmax(rank.energy_excess,
                            fmax(rank.position_error,
                                fmax(rank.course_error,
                                    fmax(rank.altitude_error,
                                        rank.speed_error))))));
                best.taem_range_error=
                    pr.along_track-target_along;
                best.taem_speed=pr.speed;
                best.taem_energy_error=
                    pr.specific_energy-end_energy;
                best.closest_distance=hypot(
                    pr.along_track-target_along,
                    pr.cross_track-target_cross);
                best.has_planned_reversal=planned_reversal;
                best.planned_reversal_is_final=false;
                best.final_heading_lock=false;
                best.planned_reversal_ut=planned_reversal?
                    t->ut+leg_duration:NAN;
                best.planned_reversal_range=hypot(
                    pr.along_track,pr.cross_track);
                best.planned_reversal_sign=planned_reversal?
                    -sign:0.0;
                best.predicted_reversals=planned_reversal?1u:0u;
                if(chosen_projection)*chosen_projection=pr;
            }
        }
    }

    return best;
}

static bool taem_s_turn_inherited_reversal_due(
        const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s){
    if(!g||!t||!s||!g->has_s_turn_leg_started||
       !isfinite(g->s_turn_leg_started_ut)||
       t->ut<g->s_turn_leg_started_ut)
        return false;

    double sign=g->s_turn_sign>=0.0?1.0:-1.0;
    bool committed=g->entry_reversal_scheduled&&
        isfinite(g->entry_reversal_ut)&&
        g->entry_reversal_sign*sign<0.0;
    if(committed&&t->ut>=g->entry_reversal_ut)return true;

    double excess=fmax(0.0,t->energy_excess_range);
    if(!(excess>0.0))return false;
    double horizontal=fmax(t->horizontal_speed,DBL_MIN);
    double full_leg=fmax(
        fmax(0.0,s->s_turn_minimum_leg_duration),
        excess/(2.0*horizontal)); /* decision-literal-ok: two symmetric legs */
    return t->ut-g->s_turn_leg_started_ut>=full_leg;
}

static bool taem_s_turn_reversal_due(
        const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s){
    if(!g||!t||!s||!g->taem_s_turn_plan.valid||
       !g->taem_s_turn_plan.has_planned_reversal||
       !isfinite(g->taem_s_turn_plan.planned_reversal_ut))
        return false;

    double dwell=t->ut-g->taem_s_turn_plan.planned_ut;
    if(g->has_s_turn_leg_started&&
       isfinite(g->s_turn_leg_started_ut)&&
       t->ut>=g->s_turn_leg_started_ut&&
       g->s_turn_sign*g->taem_s_turn_plan.target_bank>0.0)
        dwell=fmax(dwell,t->ut-g->s_turn_leg_started_ut);

    return dwell>=fmax(0.0,s->s_turn_minimum_leg_duration)&&
        t->ut>=g->taem_s_turn_plan.planned_reversal_ut;
}

static GuidanceResult taem_s_turn_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    /* If the physical Entry leg has already consumed its complete planned dwell,
       reverse before constructing the first MM305 child.  Otherwise a zero-time
       old-side child would leak one stale command across the ownership boundary. */
    if(!g->taem_s_turn_plan.valid&&taem_s_turn_inherited_reversal_due(g,t,s))
        request_side(g,-g->s_turn_sign,t->ut);
    if(taem_s_turn_reversal_due(g,t,s)){
        double next_sign=g->taem_s_turn_plan.planned_reversal_sign>=0?1.0:-1.0;
        request_side(g,next_sign,t->ut);
        g->taem_s_turn_plan.valid=false;
    }
    bool plan_expired=!g->taem_s_turn_plan.valid||
        !isfinite(g->taem_s_turn_plan.planned_ut)||
        !isfinite(g->taem_s_turn_plan.segment_duration)||
        t->ut+1e-6>=g->taem_s_turn_plan.planned_ut+g->taem_s_turn_plan.segment_duration;
    TaemSTurnProjection projection={0};
    if(plan_expired){
        EntryControlPlan previous=g->taem_s_turn_plan;
        EntryControlPlan next=taem_plan_s_turn_segment(g,t,course,p,aero,cfg,&projection);
        const EntryControlPlan*parent=previous.plan_id?&previous:
            (g->entry_s_turn_plan.plan_id?&g->entry_s_turn_plan:NULL);
        control_plan_assign_lineage(g,&next,parent);
        g->taem_s_turn_plan=next;
        if(g->taem_s_turn_plan.valid){
            g->terminal_reference_bank=g->taem_s_turn_plan.target_bank;
            g->terminal_reference_aoa=g->taem_s_turn_plan.target_aoa;
            g->terminal_reference_heading=g->taem_s_turn_plan.target_heading;
            g->terminal_prediction_altitude=projection.valid?projection.altitude:t->mean_altitude;
            g->terminal_prediction_speed=projection.valid?projection.speed:t->true_air_speed;
            g->terminal_prediction_time=g->taem_s_turn_plan.segment_duration;
        }
    }
    if(!g->taem_s_turn_plan.valid){
        GuidanceCommand safe=atmospheric(t,t->ground_track_heading,0.0,v,0.0,false,PROFILE_TAEM);
        safe.heading_control_enabled=false;safe.has_target_aoa=true;
        safe.target_aoa=clampd(v->entry_angle_of_attack,8.0,18.0);
        safe.target_pitch=t->flight_path_angle+safe.target_aoa;
        return stabilized(g,result_make(PHASE_TAEM,safe,
            "TAEM S-turn planner unavailable; holding a bounded unloaded attitude while replanning.",
            "No finite planned terminal-energy segment is available on this frame."),t,v,s,dt);
    }

    EntryControlPlan*plan=&g->taem_s_turn_plan;
    GuidanceCommand c=atmospheric(t,plan->target_heading,plan->target_bank,v,0.0,false,PROFILE_TAEM);
    c.heading_control_enabled=false;c.has_target_aoa=true;c.target_aoa=plan->target_aoa;
    c.target_pitch=t->flight_path_angle+c.target_aoa;
    g->airbrakes_deployed=false;
    double until_reversal=plan->has_planned_reversal&&isfinite(plan->planned_reversal_ut)?
        fmax(0.0,plan->planned_reversal_ut-t->ut):fmax(0.0,plan->planned_ut+plan->segment_duration-t->ut);
    char status[384];
    snprintf(status,sizeof(status),
        "TAEM planned S-turn: excess %+.1f km, bank %+.1f deg, AoA %.1f deg, heading %.1f deg, plan %.1f s, reversal in %.1f s, forecast %.1f km / %.1f km / %.0f m/s.",
        t->energy_excess_range/1000.0,plan->target_bank,plan->target_aoa,plan->target_heading,
        plan->segment_duration,until_reversal,g->terminal_prediction_altitude/1000.0,
        plan->closest_distance/1000.0,g->terminal_prediction_speed);
    return stabilized(g,result_make(PHASE_TAEM,c,status,NULL),t,v,s,dt);
}

/* HAC side convention: +1 is the south-side HAC for a southbound shuttle,
   -1 is the north-side HAC for a northbound shuttle.  The first turn relative
   to the runway is authoritative; cross-track is only a tie-breaker when the
   vehicle is already aligned with the runway. */
static double terminal_default_hac_side(const Telemetry*t,const LandingSite*site,double course){
    if(!site)return 1.0;
    double reference=isfinite(course)?course:
        (t&&isfinite(t->ground_track_heading)?t->ground_track_heading:
         (t&&isfinite(t->heading)?t->heading:site->runway_heading));
    double turn_delta=norm_signed_deg(site->runway_heading-reference);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(turn_delta<-angular_resolution)return 1.0;
    if(turn_delta>angular_resolution)return -1.0;
    if(t&&isfinite(t->runway_cross_track)&&
       fabs(t->runway_cross_track)>sqrt(DBL_EPSILON))
        return t->runway_cross_track>=0.0?1.0:-1.0;
    return 1.0;
}

bool guidance_begin_hac_test(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,char*message,size_t message_size){
    if(!g||!t||!p||!cfg){if(message&&message_size)snprintf(message,message_size,"HAC test initialization received an invalid argument.");return false;}
    (void)aero;
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double altitude_floor=fmax(1000.0,s->flare_altitude*8.0);
    double speed_floor=fmax(v->minimum_safe_speed*1.20,v->final_approach_speed*1.25);
    if(t->radar_altitude<=altitude_floor){if(message&&message_size)snprintf(message,message_size,"HAC-only test requires at least %.0f m radar altitude.",altitude_floor);return false;}
    if(t->true_air_speed<speed_floor){if(message&&message_size)snprintf(message,message_size,"HAC-only test requires at least %.0f m/s airspeed for recovery margin.",speed_floor);return false;}
    if(t->dynamic_pressure>v->maximum_dynamic_pressure*1.05){if(message&&message_size)snprintf(message,message_size,"HAC-only test blocked because dynamic pressure is above the vehicle limit.");return false;}
    guidance_machine_init(g);guidance_set_engaged(g,true);
    g->terminal_glide_mode=true;
    g->terminal_rehearsal_mode=true;
    g->terminal_test_capture_active=true;
    
    g->has_burn_command_started=true;
    g->deorbit_burn_completed=true;
    g->atmospheric_interface_crossed=true;
    g->terminal_region_entered=true;
    /* The 15 km rehearsal begins too fast for the tight HAC that matches the
       shuttle's roughly 20 deg unpowered glide.  Do not lock a huge circle at
       the initial low-AoA state.  First capture the aerodynamic glide, then
       choose a one-revolution HAC from the measured turn authority once the
       vehicle has decelerated enough for a physically useful radius. */
    g->hac_side=terminal_default_hac_side(t,&cfg->site,course);g->hac_side_selected=false;g->hac_radius=s->hac_radius;
    g->hac_remaining=0;g->hac_previous_angle=0;g->hac_progress_valid=false;
    g->minimum_turn_radius=INFINITY;
    /* Preflare is the terminal energy gate.  HAC geometry is selected so the
       shuttle reaches this height with enough kinetic energy to execute the
       pull-up, rather than merely intersecting an arbitrary low-altitude
       final line.  Keep the values vehicle-relative so another shuttle can
       reuse the same guidance without inheriting STS-N-specific constants. */
    terminal_glide_initialize(g,v,s);
    if(!taem_exec_enter(g,t,s,true)){
        if(message&&message_size)snprintf(message,message_size,"HAC-only landing test could not reconstruct MM305 executive ownership.");
        return false;
    }
    g->phase=PHASE_TAEM;
    if(message&&message_size)snprintf(message,message_size,
        "HAC-only landing test armed: preflare gate %.0f m / %.0f m/s (minimum %.0f m/s); capturing a steep unpowered glide before selecting HAC from live turn authority and gate energy.",
        g->terminal_test_preflare_altitude,g->terminal_test_preflare_target_speed,g->terminal_test_preflare_min_speed);
    return true;
}

static double terminal_hac_energy_aoa(const VehicleProfile*v){
    return clampd(v->entry_angle_of_attack-8.0,8.0,10.5);
}

/* Specific mechanical energy along an unpowered path obeys
       dE/ds = -D/m.
   That makes drag acceleration, not FPA by itself, the primary TAEM energy
   actuator.  Estimate what the current measured vehicle would produce at a
   different incidence by scaling the measured drag with the calibrated Mach
   polar, then choose the largest AoA that stays inside the remaining drag-work
   budget.  Choosing the largest admissible AoA preserves as much lift/control
   authority as possible without spending the kinetic reserve needed later. */
static double terminal_drag_budget_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double allowed_drag,double preferred_aoa){
    /* An empty drag-work feasible set is not a valid zero-incidence solution. */
    if(!t||!v||!isfinite(allowed_drag)||allowed_drag<=0.0)return NAN;
    double incidence=fmax(.25,hypot(t->angle_of_attack,t->sideslip));
    double current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,NULL,&current_df);
    double measured=live_drag_accel(t,aero,v);
    if(!(measured>.005)||!isfinite(measured))return clampd(preferred_aoa,0.0,
        fmin(v->maximum_angle_of_attack-1.0,terminal_hac_energy_aoa(v)));
    double upper=clampd(fmax(preferred_aoa,terminal_hac_energy_aoa(v)),
        0.0,fmax(0.0,v->maximum_angle_of_attack-1.0));
    double best=NAN;
    for(double aoa=0.0;aoa<=upper+.01;aoa+=.25){
        double df=1.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,NULL,&df);
        double projected=measured*fmax(.02,df)/fmax(.02,current_df);
        if(projected<=allowed_drag*1.02)best=aoa;
    }
    return isfinite(best)?clampd(best,0.0,upper):NAN;
}

/* A drag-work budget may unload a shallow approach, but it cannot own normal
   force after the commanded dive has been captured.  In the recorded 27 km
   flight the zero-AoA drag itself exceeded the budget.  Treating that empty
   feasible set as a zero-AoA command overrode the FPA feedback all the way from
   -23 to -52 degrees and consumed the remaining maneuver height.

   Release the energy cap continuously across the existing three-degree path
   tracking band.  At/above the shallow edge the energy law retains authority;
   below the steep edge the existing bounded FPA-feedback command owns lift.
   An infeasible budget requests minimum drag only while the vehicle is still
   too shallow, never an indefinitely ballistic descent.  Command slew and
   dynamic-pressure/g-load protections remain downstream in stabilized(). */
/* Close the flight-path angle through normal acceleration, not a fixed
   alpha trim. The old 14 deg + 0.72*FPA-error law converged near -18 deg for
   a -32 deg request as dynamic pressure rose in the live flight: its small
   positive incidence already supplied almost all of weight. Invert the
   current measured lift/polar instead, retaining bounded incidence and the
   downstream load/attitude protections. */
static double terminal_fpa_force_aoa(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double reference_fpa){
    double speed=fmax(1.0,t->true_air_speed);
    double response=clampd(speed/35.0,8.0,25.0);
    /* The FPA transition is coupled to the same pitch plant that limits AoA
       delivery.  A speed-only 8--25 s surrogate made the forecast assume that
       lift could arrive gradually while the command path was already asking
       the real actuator to reach its full incidence envelope in about 5 s.
       Use the measured/shared attitude reachability model when it is valid;
       retain the speed-scaled value only as the no-response-data fallback. */
    double attitude_response=decision_pitch_capture_time(t,
        v?v->maximum_angle_of_attack:NAN);
    if(isfinite(attitude_response)&&attitude_response>0.0)
        response=clampd(attitude_response,1.0,25.0);
    double gamma=t->flight_path_angle*DEG2RAD;
    double gamma_rate=clampd((reference_fpa-t->flight_path_angle)/response,-2.5,2.5)*DEG2RAD;
    double radius=fmax(1.0,p->radius+t->mean_altitude);
    double gravity=p->gravitational_parameter/(radius*radius);
    double curvature=t->horizontal_speed*t->horizontal_speed/radius;
    double normal=(gravity-curvature)*cos(gamma)+speed*gamma_rate;
    if(normal<=0.0)return 0.0;
    double bank=clampd(norm_signed_deg(t->roll),-80.0,80.0)*DEG2RAD;
    double required_lift=normal/fmax(.20,cos(bank));
    double incidence=hypot(t->angle_of_attack,t->sideslip),current_lf=0.0;
    aerodynamic_force_factors_mach(t->mach,fmax(.5,incidence),v,&current_lf,NULL);
    double modeled_gain=fmax(0.0,t->dynamic_pressure)/fmax(20.0,aero.ballistic_coefficient)*
        fmax(.01,aero.lift_to_drag);
    double measured=t->mass>1.0&&isfinite(t->lift_force)?t->lift_force/t->mass:0.0;
    double gain=incidence>=.75&&measured>.02?
        measured/fmax(.015,fabs(current_lf)):modeled_gain;
    double upper=fmin(20.0,v->maximum_angle_of_attack);
    if(!isfinite(gain)||gain<=.005)return upper;
    double lo=0.0,hi=upper;
    for(int i=0;i<16;i++){
        double mid=.5*(lo+hi),lf=0.0;
        aerodynamic_force_factors_mach(t->mach,mid,v,&lf,NULL);
        if(gain*fabs(lf)<required_lift)lo=mid;else hi=mid;
    }
    return clampd(.5*(lo+hi),0.0,upper);
}

static double terminal_preview_recovery_aoa(double preferred_aoa,double budget_aoa,
        double reference_fpa,double actual_fpa){
    double limited=isfinite(budget_aoa)?fmin(preferred_aoa,budget_aoa):0.0;
    if(!isfinite(reference_fpa)||!isfinite(actual_fpa))return preferred_aoa;
    double error=fmax(reference_fpa,-35.0)-actual_fpa;
    double blend=clampd((error+3.0)/6.0,0.0,1.0);
    blend=blend*blend*(3.0-2.0*blend);
    return limited+(preferred_aoa-limited)*blend;
}

static double terminal_required_aoa_for_lateral(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg){
    double eff=clampd(t->bank_effectiveness,.35,1.8);
    double bank_component=fabs(sin(clampd(fabs(bank_deg)*eff,0.0,89.0)*DEG2RAD));
    if(fabs(lateral_accel)<.35||bank_component<.08)return 0.0;
    double required_lift=fabs(lateral_accel)/bank_component*1.06;
    double current_incidence=fmax(1.0,fabs(t->angle_of_attack));
    double current_lf=0;
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,&current_lf,NULL);
    double measured=t->mass>1&&isfinite(t->lift_force)&&t->lift_force>0?
        t->lift_force/t->mass:0.0;
    double maximum_aoa=terminal_hac_energy_aoa(v);
    for(double aoa=0.0;aoa<=maximum_aoa+.01;aoa+=.5){
        double lf=0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,NULL);
        double scaled_measured=measured>.005?
            measured*fabs(lf)/fmax(.05,fabs(current_lf)):0.0;
        double modeled=fmax(0.0,t->dynamic_pressure)/fmax(20.0,aero.ballistic_coefficient)*
            fmax(0.0,aero.lift_to_drag)*fabs(lf);
        if(fmax(scaled_measured,modeled)>=required_lift)return aoa;
    }
    return maximum_aoa;
}

static double terminal_atmosphere_vertical_resolution(const PlanetModel*p,
        double lower_altitude,double upper_altitude){
    if(!p||p->atmosphere_sample_count<2)return INFINITY;
    lower_altitude=fmax(0.0,lower_altitude);
    upper_altitude=fmax(lower_altitude,upper_altitude);
    double resolution=INFINITY;
    for(size_t i=1;i<p->atmosphere_sample_count;i++){
        double a=p->atmosphere_altitude[i-1],b=p->atmosphere_altitude[i];
        if(!isfinite(a)||!isfinite(b)||!(b>a))return NAN;
        if(b<lower_altitude||a>upper_altitude)continue;
        resolution=fmin(resolution,b-a);
    }
    return resolution;
}

static double terminal_projected_drag_work(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,double target_aoa,
        double ground_path,double slope_deg){
    if(!g||!t||!p||!v||!s||!isfinite(ground_path)||ground_path<0.0||
       !isfinite(slope_deg)||!isfinite(target_aoa)||
       !isfinite(aero.ballistic_coefficient)||!(aero.ballistic_coefficient>0.0)||
       !isfinite(t->true_air_speed)||!(t->true_air_speed>0.0))
        return NAN;
    if(ground_path==0.0)return 0.0;

    double slope=slope_deg*DEG2RAD;
    double cosine=cos(slope);
    if(!(slope>=0.0&&slope<.5*M_PI)||!(cosine>0.0))return NAN;
    double tangent=tan(slope);
    double path_factor=1.0/cosine;

    double current_df=1.0;
    double current_incidence=hypot(t->angle_of_attack,t->sideslip);
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,NULL,&current_df);
    if(!isfinite(current_df)||current_df<0.0)return NAN;

    double rho0=planet_atmospheric_density(p,t->mean_altitude);
    if(!isfinite(rho0)||rho0<0.0)return NAN;
    double modeled0=.5*rho0*t->true_air_speed*t->true_air_speed/
        aero.ballistic_coefficient*current_df;
    double observed0=terminal_expected_energy_loss_accel(g,t,aero,v);
    if(!isfinite(observed0)||observed0<0.0)return NAN;
    double anchor=modeled0>DBL_MIN?observed0/modeled0:1.0;
    if(!isfinite(anchor)||anchor<0.0)return NAN;

    /*
     * AoA cannot jump to its planned terminal value. Resolve the integration at
     * least as finely as one measured/modelled control-response distance and,
     * when a live atmosphere table exists, one atmosphere interpolation
     * interval in altitude. These are model resolutions, not flight gates.
     */
    double response_time=hac_response_lead_time(t,s,0.0);
    if(!isfinite(response_time)||response_time<0.0)return NAN;
    double response_distance=t->true_air_speed*response_time;
    double end_altitude=fmax(0.0,t->mean_altitude-ground_path*tangent);
    double vertical_resolution=terminal_atmosphere_vertical_resolution(
        p,end_altitude,t->mean_altitude);
    if(isnan(vertical_resolution))return NAN;
    double atmosphere_ground_resolution=
        isfinite(vertical_resolution)&&tangent>DBL_MIN?
            vertical_resolution/tangent:INFINITY;
    /* A zero-error response solve can leave a positive sub-metre residual
       because the capture-time model is continuous.  It is below the
       distance resolution of this integration, so treating it as a segment
       size would turn a harmless zero-delay response into billions of steps. */
    double response_resolution=sqrt(DBL_EPSILON)*fmax(1.0,ground_path);
    bool response_resolved=response_distance>response_resolution;
    double segment_length=ground_path;
    if(response_resolved)
        segment_length=fmin(segment_length,response_distance);
    if(isfinite(atmosphere_ground_resolution)&&atmosphere_ground_resolution>DBL_MIN)
        segment_length=fmin(segment_length,atmosphere_ground_resolution);
    if(!(segment_length>0.0)||!isfinite(segment_length))segment_length=ground_path;
    size_t steps=(size_t)ceil(ground_path/segment_length);
    if(steps<1)steps=1;

    double initial_aoa=clampd(fabs(t->angle_of_attack),0.0,v->maximum_angle_of_attack);
    double ds=ground_path/(double)steps;
    double speed=t->true_air_speed,altitude=t->mean_altitude;
    double work=0.0;
    for(size_t i=0;i<steps;i++){
        double sample_altitude=fmax(0.0,altitude-.5*ds*tangent);
        double density=planet_atmospheric_density(p,sample_altitude);
        if(!isfinite(density)||density<0.0)return NAN;
        double sound=planet_atmospheric_speed_of_sound(p,sample_altitude);
        double mach=isfinite(sound)&&sound>DBL_MIN?speed/sound:t->mach;

        double distance_mid=((double)i+.5)*ds;
        double response_progress=response_resolved?
            clampd(distance_mid/response_distance,0.0,1.0):1.0;
        response_progress=response_progress*response_progress*(3.0-2.0*response_progress);
        double projected_aoa=initial_aoa+(target_aoa-initial_aoa)*response_progress;
        double df=1.0;
        aerodynamic_force_factors_mach(mach,projected_aoa,v,NULL,&df);
        if(!isfinite(df)||df<0.0)return NAN;

        double drag=.5*density*speed*speed/aero.ballistic_coefficient*df*anchor;
        if(!isfinite(drag)||drag<0.0)return NAN;
        double air_ds=ds*path_factor;
        work+=drag*air_ds;

        double next_altitude=fmax(0.0,altitude-ds*tangent);
        double radius=p->radius+.5*(altitude+next_altitude);
        if(!(radius>0.0)||!isfinite(radius))return NAN;
        double gravity=p->gravitational_parameter/(radius*radius);
        if(!isfinite(gravity)||gravity<0.0)return NAN;
        double v2=speed*speed+2.0*gravity*fmax(0.0,altitude-next_altitude)-
            2.0*drag*air_ds;
        if(!isfinite(v2)||v2<0.0)return INFINITY;
        speed=sqrt(v2);
        altitude=next_altitude;
    }
    return work;
}

static double terminal_integrated_energy_path(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,double target_aoa,
        double available_specific_energy,double slope_deg,double min_path,double max_path,
        EnergyPathEnvelope*out_envelope){
    if(out_envelope)*out_envelope=(EnergyPathEnvelope){0};
    if(!isfinite(available_specific_energy)||!isfinite(min_path)||!isfinite(max_path)||
       max_path<min_path)return NAN;
    min_path=fmax(0.0,min_path);max_path=fmax(min_path,max_path);

    double min_work=terminal_projected_drag_work(g,t,p,aero,v,s,target_aoa,min_path,slope_deg);
    if(!isfinite(min_work)||min_work<0.0)return NAN;
    EnergyPathEnvelope minimum=decision_energy_path_envelope(
        t,p,available_specific_energy,min_work,min_path);
    if(!minimum.valid)return NAN;
    if(!minimum.feasible){
        if(out_envelope)*out_envelope=minimum;
        return min_path;
    }

    double max_work=terminal_projected_drag_work(g,t,p,aero,v,s,target_aoa,max_path,slope_deg);
    if(isnan(max_work)||max_work<0.0)return NAN;
    if(isfinite(max_work)){
        EnergyPathEnvelope maximum=decision_energy_path_envelope(
            t,p,available_specific_energy,max_work,max_path);
        if(!maximum.valid)return NAN;
        if(maximum.feasible){
            if(out_envelope)*out_envelope=maximum;
            return max_path;
        }
    }
    /*
     * +INFINITY is a meaningful result from the forward integrator: the vehicle
     * would exhaust kinetic energy before completing this path. Treat it as a
     * physically infeasible upper bracket, not as a broken model.
     */

    /*
     * Solve the longest path whose modeled drag work plus measured/model
     * uncertainty fits the available unpowered specific-energy budget.
     * Convergence is numerical: stop at sqrt(machine epsilon) relative bracket
     * width rather than using a tuned flight-behavior iteration count.
     */
    double lo=min_path,hi=max_path;
    double slope_rad=slope_deg*DEG2RAD;
    double path_resolution=sqrt(DBL_EPSILON)*fmax(1.0,max_path);
    if(isfinite(slope_rad)&&slope_rad>0.0&&slope_rad<.5*M_PI){
        double vertical_resolution=terminal_atmosphere_vertical_resolution(
            p,fmax(0.0,t->mean_altitude-max_path*tan(slope_rad)),
            t->mean_altitude);
        if(isfinite(vertical_resolution)&&vertical_resolution>0.0)
            path_resolution=fmax(path_resolution,
                vertical_resolution/tan(slope_rad));
    }
    EnergyPathEnvelope best=minimum;
    for(int i=0;i<DBL_MANT_DIG;i++){
        double scale=fmax(1.0,fmax(fabs(lo),fabs(hi)));
        if(hi-lo<=fmax(path_resolution,sqrt(DBL_EPSILON)*scale))break;
        double mid=.5*(lo+hi);
        double work=terminal_projected_drag_work(g,t,p,aero,v,s,target_aoa,mid,slope_deg);
        if(isnan(work)||work<0.0)return NAN;
        if(isinf(work)){
            hi=mid;
            continue;
        }
        EnergyPathEnvelope probe=decision_energy_path_envelope(
            t,p,available_specific_energy,work,mid);
        if(!probe.valid)return NAN;
        if(probe.feasible){lo=mid;best=probe;}else hi=mid;
    }
    if(out_envelope)*out_envelope=best;
    return lo;
}

static void terminal_path_slope_bounds(const Telemetry*t,const VehicleProfile*v,
        const GuidanceSettings*s,double*minimum,double*maximum){
    double taem=clampd(s?s->taem_glide_slope:12.0,6.0,30.0);
    double final=clampd(s?s->final_glide_slope:20.0,taem,35.0);
    /* Cool/nominal arrivals should retain the compact terminal corridor;
       only genuine excess speed earns a long shallow energy-management leg.
       This replaces the former universal 18 degree floor with a vehicle- and
       state-scaled envelope rather than simply lowering the constant. */
    double speed_reference=v?
        fmax(v->minimum_safe_speed,v->final_approach_speed):0.0;
    double speed_scale=fmax(speed_reference,DBL_MIN);
    double hot=t?clampd((t->true_air_speed-speed_reference)/speed_scale,
        0.0,1.0):0.0;
    double cool_lo=fmax(taem,final*.90);
    double hot_lo=clampd(taem-4.0,5.0,cool_lo);
    double lo=cool_lo+(hot_lo-cool_lo)*hot;
    double hi=clampd(fmax(final+12.0,taem+12.0),lo+4.0,35.0);
    if(minimum)*minimum=lo;
    if(maximum)*maximum=hi;
}

/* Until a lateral route is committed, do not spend inbound distance on an
   imagined energy-management loop. Close height against the real preflare
   gate. A qualified, committed HAC has its own path law and never calls this
   preview helper. Drag work is budgeted over air-path metres, not horizontal
   metres. This preserves the same bounded descent envelope. */
static double terminal_uncommitted_alignment_heading(const Telemetry*t,
        const LandingConfiguration*cfg){
    /* The terminal alignment station is a fixed point, not an always-positive
       look-ahead distance. The old max(4000, staging_along-along) moved the
       target east with the aircraft and commanded 35 degrees even after the
       vehicle passed east of the runway. Preserve the signed along delta. */
    double target_along=-fmax(1500.0,cfg->guidance.final_approach_distance);
    double along_delta=target_along-t->runway_along_track;
    return norm_deg(cfg->site.runway_heading+
        atan2(-t->runway_cross_track,along_delta)*RAD2DEG);
}

static double terminal_uncommitted_gate_geometry(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double*fpa){
    double gate_height=fmax(0.0,g->terminal_test_preflare_altitude);
    double gate_ground=gate_height/fmax(tan(clampd(cfg->guidance.final_glide_slope,
        5.0,35.0)*DEG2RAD),1e-3);
    double ground=hypot(-t->runway_along_track-gate_ground,
        t->runway_cross_track);
    double height=fmax(0.0,t->mean_altitude-cfg->site.altitude-gate_height);
    double minimum=0.0,maximum=0.0;
    terminal_path_slope_bounds(t,&cfg->vehicle,&cfg->guidance,&minimum,&maximum);
    if(fpa)*fpa=-clampd(atan2(height,fmax(1000.0,ground))*RAD2DEG,minimum,maximum);
    return fmax(1000.0,hypot(ground,height));
}

static double terminal_test_energy_selected_slope(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        double*preferred_total_path,EnergyPathEnvelope*out_envelope){
    const VehicleProfile*v=&cfg->vehicle;
    if(out_envelope)*out_envelope=(EnergyPathEnvelope){0};
    double gate_altitude=cfg->site.altitude+fmax(0,g->terminal_test_preflare_altitude);
    double target_speed=fmax(g->terminal_test_preflare_target_speed,g->terminal_test_preflare_min_speed);
    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,gate_altitude,target_speed,p);
    if(!isfinite(available_energy))return NAN;

    /*
     * The path budget is solved in specific-energy units. Uncertainty comes
     * from measured trajectory residuals and certified physics uncertainty in
     * decision_energy_path_envelope(); calibration confidence is not converted
     * into an arbitrary percentage reserve here.
     */
    double gate_height=fmax(0,t->mean_altitude-gate_altitude);
    double minimum_slope=0,maximum_slope=0;
    terminal_path_slope_bounds(t,v,&cfg->guidance,&minimum_slope,&maximum_slope);
    double min_total=gate_height/fmax(tan(maximum_slope*DEG2RAD),DBL_MIN);
    double max_total=gate_height/fmax(tan(minimum_slope*DEG2RAD),DBL_MIN);
    double baseline_aoa=clampd(fmax(3.0,t->angle_of_attack),3.0,terminal_hac_energy_aoa(v));
    double nominal_slope=clampd(
        atan2(gate_height,fmax(.5*(min_total+max_total),DBL_MIN))*RAD2DEG,
        minimum_slope,maximum_slope);

    EnergyPathEnvelope envelope={0};
    double selected_total=terminal_integrated_energy_path(
        g,t,p,aero,v,&cfg->guidance,baseline_aoa,available_energy,
        nominal_slope,min_total,max_total,&envelope);
    if(!isfinite(selected_total)||!envelope.valid)return NAN;

    double slope=atan2(gate_height,fmax(selected_total,DBL_MIN))*RAD2DEG;
    slope=clampd(slope,minimum_slope,maximum_slope);
    if(preferred_total_path)*preferred_total_path=selected_total;
    if(out_envelope)*out_envelope=envelope;
    return slope;
}

static double terminal_test_projected_turn_radius(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double bank_deg){
    double current_lf=0,target_lf=0;
    double current_incidence=fmax(1.0,fabs(t->angle_of_attack));
    double target_incidence=terminal_hac_energy_aoa(v);
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,&current_lf,NULL);
    aerodynamic_force_factors_mach(t->mach,target_incidence,v,&target_lf,NULL);
    double measured=t->mass>1&&isfinite(t->lift_force)&&t->lift_force>0?t->lift_force/t->mass:0;
    double projected=measured>0?measured*fabs(target_lf)/fmax(.05,fabs(current_lf)):0;
    double modeled=fmax(0,t->dynamic_pressure)/fmax(20,aero.ballistic_coefficient)*
        fmax(0,aero.lift_to_drag)*fabs(target_lf);
    projected=fmax(modeled,clampd(projected,0,planet_surface_gravity(p)*2.0));
    double eff=clampd(t->bank_effectiveness,.35,1.8);
    double lateral=projected*fabs(sin(clampd(fabs(bank_deg)*eff,0,89)*DEG2RAD));
    return lateral>.005?t->true_air_speed*t->true_air_speed/lateral:INFINITY;
}

static double hac_oriented_progress_rate(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double hac_radius,double course){
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    GeoPoint threshold={site->latitude,site->longitude,site->altitude};
    double e=0,n=0;local_offsets(threshold,current,planet_radius,&e,&n);
    double h=site->runway_heading*DEG2RAD,ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h);
    double fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance;
    double ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius;
    double x=e-ce,y=n-cn,r2=x*x+y*y;
    if(r2<1)return 0;
    double cr=course*DEG2RAD,ve=t->horizontal_speed*sin(cr),vn=t->horizontal_speed*cos(cr);
    double angular=(x*vn-y*ve)/r2;
    double oriented=-side*angular;
    double geometric_limit=t->horizontal_speed/fmax(sqrt(r2),1)*1.5;
    return clampd(oriented,0,fmax(0,geometric_limit));
}

/* Publication invariant shared by every HAC selection outcome. A distant
   circle is legal only with a continuous, qualified join and a real arc. */
static bool hac_handoff_geometry_valid(const HACTransitionPlan*join,double e,double n,
        const LandingSite*site,const GuidanceSettings*s,double radius,double side){
    if(!join->valid||!isfinite(radius)||radius<1000.0||fabs(side)!=1.0||
       !isfinite(join->arc_remaining)||join->arc_remaining<700.0||
       !isfinite(join->length)||join->length<200.0)return false;
    double h=site->runway_heading*DEG2RAD;
    double fe=-sin(h)*s->final_approach_distance,fn=-cos(h)*s->final_approach_distance;
    double ce=fe+cos(h)*side*radius,cn=fn-sin(h)*side*radius;
    double end=atan2(fn-cn,fe-ce)+side*join->arc_remaining/radius;
    double start_error=hypot(join->lead_start.e-e,join->lead_start.n-n);
    double end_error=hypot(join->p3.e-(ce+radius*cos(end)),
        join->p3.n-(cn+radius*sin(end)));
    double angle_error=fabs(norm_signed_deg((join->end_angle-end)*RAD2DEG));
    double lead_error=fabs(join->lead_length-hypot(join->p0.e-e,join->p0.n-n));
    return isfinite(start_error)&&start_error<.01&&isfinite(end_error)&&end_error<.01&&
        isfinite(angle_error)&&angle_error<1e-6&&isfinite(lead_error)&&lead_error<.01;
}

/* Acquisition is measured at the live aircraft, independently of downstream
   path fit. Positive radial closure means toward the circle; in this local
   frame +1 is south and -1 is north for an eastbound runway. */
static void terminal_candidate_execution_cost(TerminalCandidate*c,
        const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!c)return;
    c->execution_evaluated=true;
    c->execution_distance=c->execution_time=c->execution_course_error=INFINITY;
    c->execution_margin=-INFINITY;
    c->execution_horizon=0.0;
    if(!t||!c->valid||!c->join.valid||!(t->horizontal_speed>0.0))return;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    double e,n;local_offsets(origin,current,p->radius,&e,&n);
    double de=c->join.lead_start.e-e,dn=c->join.lead_start.n-n;
    double distance=hypot(de,dn);
    HACPoint2 d={c->join.p0.e-c->join.lead_start.e,
        c->join.p0.n-c->join.lead_start.n};
    if(hypot(d.e,d.n)<=sqrt(DBL_EPSILON))
        d=(HACPoint2){c->join.p1.e-c->join.p0.e,c->join.p1.n-c->join.p0.n};
    if(hypot(d.e,d.n)<=sqrt(DBL_EPSILON))return;
    double tangent=atan2(d.e,d.n)*RAD2DEG;
    double error=fabs(norm_signed_deg(tangent-course));
    double acquisition_error=error;
    if(distance>t->horizontal_speed*fmax(0.0,c->response))
        acquisition_error=fmax(error,fabs(norm_signed_deg(atan2(de,dn)*RAD2DEG-course)));
    double rate=hac_course_rate_cap_for_speed(t->true_air_speed);
    double bank=fmin(fabs(cfg->vehicle.maximum_bank_angle),dynamic_bank_limit(t,&cfg->vehicle));
    double lift=live_lift_accel(t,aero,&cfg->vehicle);
    double effectiveness=t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    rate=fmin(rate,lift*fabs(sin(fmin(bank*effectiveness,nextafter(90.0,0.0))*DEG2RAD))/t->horizontal_speed*RAD2DEG);
    double radial_turn=0.0;
    c->execution_radial_closure=0.0;
    if(c->kind==TERMINAL_PATH_HAC){
        double h=cfg->site.runway_heading*DEG2RAD;
        double ce=-sin(h)*c->final_distance+cos(h)*c->side*c->radius;
        double cn=-cos(h)*c->final_distance-sin(h)*c->side*c->radius;
        double r=hypot(ce-e,cn-n);
        if(r>sqrt(DBL_EPSILON)){
            c->execution_radial_closure=t->horizontal_speed*
                ((ce-e)*sin(course*DEG2RAD)+(cn-n)*cos(course*DEG2RAD))/r;
            double inward=fabs(norm_signed_deg(atan2(ce-e,cn-n)*RAD2DEG-course));
            radial_turn=fmax(0.0,inward-90.0);
        }
    }
    double turn=fmax(acquisition_error,radial_turn)/fmax(rate,DBL_MIN);
    double time=distance/t->horizontal_speed+turn+fmax(0.0,c->response);
    double gate=cfg->site.altitude+fmax(0.0,g->terminal_test_preflare_altitude);
    double height=fmax(0.0,t->mean_altitude-gate);
    double sink=fmax(fmax(0.0,-t->vertical_speed),t->horizontal_speed*fabs(tan(c->slope*DEG2RAD)));
    double drag=terminal_expected_speed_loss_accel(g,t,p,aero,&cfg->vehicle);
    double minimum_speed=fmax(g->terminal_test_preflare_min_speed,cfg->vehicle.minimum_safe_speed);
    double decel=fmax(0.0,drag-planet_surface_gravity(p)*fmax(0.0,-sin(t->flight_path_angle*DEG2RAD)));
    double horizon=sink>0.0?height/sink:INFINITY;
    if(decel>0.0)horizon=fmin(horizon,fmax(0.0,t->true_air_speed-minimum_speed)/decel);
    double reserve=planet_surface_gravity(p)*height+
        (t->true_air_speed*t->true_air_speed-minimum_speed*minimum_speed)/2.0;
    if(drag>0.0)horizon=fmin(horizon,fmax(0.0,reserve)/(drag*t->true_air_speed));
    c->execution_distance=distance;c->execution_course_error=error;
    c->execution_time=time;c->execution_horizon=horizon;
    c->execution_margin=horizon-time;
    if(!(c->execution_margin>=0.0)){c->degraded=true;c->energy_degraded=true;}
}

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

    double final_distance=s->final_approach_distance;
    double straight_to_gate=fmax(0.0,final_distance-gate_ground_distance);
    double height=fmax(0.0,t->mean_altitude-
        (cfg->site.altitude+g->terminal_test_preflare_altitude));
    double preferred_fixed_path=fmax(0.0,energy_total_path-straight_to_gate);
    double min_fixed_path=fmax(0.0,
        height/tan(maximum_path_slope*DEG2RAD)-straight_to_gate);
    double max_fixed_path=fmax(0.0,
        height/tan(minimum_path_slope*DEG2RAD)-straight_to_gate);
    if(max_fixed_path<min_fixed_path)return false;

    const int radius_samples=13;
    HACTransitionPlan best_transition={0},fallback_transition={0};
    double best_radius=0.0,best_side=1.0,best_slope=0.0,best_response=0.0;
    double fallback_radius=0.0,fallback_side=1.0,fallback_slope=0.0,
        fallback_response=0.0;
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

            GuidanceSettings local=*s;
            local.final_approach_distance=final_distance;
            HACTransitionPlan transition={0};
            if(!hac_transition_plan(&transition,current_e,current_n,
                    future_e,future_n,course,future_course,&cfg->site,&local,
                    radius,side,projected_join_speed,circle_design_speed,
                    transition_drag,max_lateral,preferred_fixed_path,
                    min_fixed_path,max_fixed_path))
                continue;

            double fixed_path=transition.lead_length+transition.length+
                transition.arc_remaining;
            double path_to_gate=fixed_path+straight_to_gate;
            if(!(path_to_gate>0.0))continue;
            double geometric_slope=atan2(height,path_to_gate)*RAD2DEG;

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
            double path_scale=fmax(fabs(preferred_fixed_path),DBL_MIN);
            double path_error=fabs(fixed_path-preferred_fixed_path)/path_scale;
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
                .final_distance=final_distance,.slope=geometric_slope,
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
                double scale=fmax(1.0,fmax(fabs(execution_time),
                    fabs(best_execution_time)));
                bool execution_better=!best_transition.valid||
                    execution_time<best_execution_time-sqrt(DBL_EPSILON)*scale;
                bool execution_same=fabs(execution_time-best_execution_time)<=
                    sqrt(DBL_EPSILON)*scale;
                bool better=execution_better||
                    (execution_same&&(path_error<best_path_error||
                    (fabs(path_error-best_path_error)<=sqrt(DBL_EPSILON)&&
                     control_use<best_control_use)));
                if(better){
                    best_transition=transition;
                    best_transition.violation_score=0.0;
                    best_radius=radius;best_side=side;
                    best_slope=geometric_slope;best_response=response_time;
                    best_path_error=path_error;best_control_use=control_use;
                    best_execution_time=execution_time;
                }
            }else{
                bool better=violation<fallback_violation||
                    (fabs(violation-fallback_violation)<=sqrt(DBL_EPSILON)&&
                     (execution_time<fallback_execution_time||
                      (fabs(execution_time-fallback_execution_time)<=sqrt(DBL_EPSILON)&&
                       (path_error<fallback_path_error||
                        (fabs(path_error-fallback_path_error)<=sqrt(DBL_EPSILON)&&
                         control_use<fallback_control_use)))));
                if(better){
                    fallback_transition=transition;
                    fallback_transition.degraded=true;
                    fallback_transition.violation_score=violation;
                    fallback_radius=radius;fallback_side=side;
                    fallback_slope=clampd(geometric_slope,
                        minimum_path_slope,maximum_path_slope);
                    fallback_response=response_time;
                    fallback_violation=violation;
                    fallback_path_error=path_error;
                    fallback_control_use=control_use;
                    fallback_execution_time=execution_time;
                    fallback_geometry_bad=geometry_bad;
                    fallback_energy_bad=energy_bad;
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

    if(!selected.degraded&&
       !hac_handoff_geometry_valid(&selected,current_e,current_n,
            &cfg->site,s,selected_radius,selected_side))
        return false;
    if(selected.degraded&&(!selected.valid||
       !hac_bezier_regular(&selected)||!isfinite(selected_radius)||
       !(selected_radius>0.0)))
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
    g->terminal_test_final_approach_distance=final_distance;
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
static bool terminal_candidate_geometry_executable(const TerminalCandidate*c){
    return c&&c->valid&&!c->geometry_degraded&&
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
    bool a_reachable=!a->execution_evaluated||a->execution_margin>=0.0;
    bool b_reachable=!b->execution_evaluated||b->execution_margin>=0.0;
    if(a_reachable!=b_reachable)return b_reachable?-1:1;
    bool a_executable=terminal_candidate_geometry_executable(a);
    bool b_executable=terminal_candidate_geometry_executable(b);
    if(a_executable!=b_executable)return b_executable?-1:1;

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
    bool a_executable=terminal_candidate_geometry_executable(a);
    bool b_executable=terminal_candidate_geometry_executable(b);
    int order=(a_executable&&b_executable)?
        terminal_candidate_execution_order(a,b):
        terminal_candidate_constraint_order(a,b);
    if(order==0&&a_executable&&b_executable)
        order=terminal_candidate_constraint_order(a,b);
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

static void terminal_publish_candidate(GuidanceMachine*g){
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

static bool terminal_prediction_ready(const GuidanceMachine*g,const Telemetry*t,
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
static bool terminal_candidate_operationally_usable(const GuidanceMachine*g,
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
    else if(!hac_bezier_regular(&c->join))reason="bezier irregular";
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
        if(t&&diag_env&&strcmp(diag_env,"1")==0&&
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

    double energy_total_path=0.0;
    EnergyPathEnvelope energy_envelope={0};
    double energy_slope=terminal_test_energy_selected_slope(
        &trial,future,p,aero,cfg,&energy_total_path,&energy_envelope);
    double minimum_slope=0.0,maximum_slope=0.0;
    terminal_path_slope_bounds(future,v,s,&minimum_slope,&maximum_slope);
    if(!isfinite(energy_total_path)||!(energy_total_path>0.0)||
       !isfinite(energy_slope)||!energy_envelope.valid||
       !(minimum_slope>0.0)||!(maximum_slope>=minimum_slope))return false;

    double gate_height=fmax(0.0,trial.terminal_test_preflare_altitude);
    double gate_altitude=cfg->site.altitude+gate_height;
    double energy_tangent=tan(energy_slope*DEG2RAD);
    double minimum_tangent=tan(minimum_slope*DEG2RAD);
    double maximum_tangent=tan(maximum_slope*DEG2RAD);
    if(!(energy_tangent>0.0)||!(minimum_tangent>0.0)||!(maximum_tangent>0.0))
        return false;

    double gate_ground=gate_height/energy_tangent;
    double final_to_gate=fmax(0.0,final_distance-gate_ground);
    double height_to_gate=fmax(0.0,future->mean_altitude-gate_altitude);

    /*
     * Path-length bounds come directly from the allowed vertical slope
     * envelope.  A planar curve can never be shorter than its chord.
     */
    double minimum_curve=fmax(chord,
        fmax(0.0,height_to_gate/maximum_tangent-final_to_gate));
    double maximum_curve=fmax(0.0,
        height_to_gate/minimum_tangent-final_to_gate);
    if(!isfinite(minimum_curve)||!isfinite(maximum_curve)||
       maximum_curve<minimum_curve)return false;

    double energy_curve=fmax(chord,energy_total_path-final_to_gate);
    double handle_span=fmin(maximum_curve,fmax(chord,energy_curve));
    if(!(handle_span>0.0)||!isfinite(handle_span))return false;

    double max_lateral=authority.maximum_lateral_accel_mps2;
    if(!(max_lateral>0.0)||!isfinite(max_lateral))return false;

    double speed_loss=terminal_expected_speed_loss_accel(g,live,p,aero,v);
    double final_course=cfg->site.runway_heading*DEG2RAD;
    double start_course=future_course*DEG2RAD;
    double endpoint_speed_floor=fmax(trial.terminal_test_preflare_target_speed,
        authority.minimum_speed_mps);

    HACTransitionPlan best={0},fallback={0};
    bool have_best=false,have_fallback=false;
    double best_energy_time=INFINITY,fallback_energy_time=INFINITY;
    double fallback_worst_margin=-INFINITY;
    double best_slope=energy_slope,best_radius=p->radius,best_side=1.0;
    double fallback_slope=energy_slope,fallback_radius=p->radius,fallback_side=1.0;
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
           Track the measured-to-future segment first, just as the HAC
           selector does, so a forecast origin cannot command its tangent
           before the vehicle has physically reached that state. */
        c.lead_start=hac_point(live_e,live_n);
        c.lead_length=response_distance;
        c.p0=hac_point(start_e,start_n);
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

        c.degraded_path=
            constraints.path_minimum.margin<0.0||
            constraints.path_maximum.margin<0.0||
            constraints.slope_minimum.margin<0.0||
            constraints.slope_maximum.margin<0.0;
        c.degraded_control=constraints.lateral_authority.margin<0.0;
        c.degraded_rate=constraints.course_rate.margin<0.0;
        c.degraded_end=constraints.endpoint_rate.margin<0.0;
        c.degraded=c.degraded_path||c.degraded_control||
            c.degraded_rate||c.degraded_end;
        c.violation_score=fmax(0.0,-constraints.worst_normalized_margin);

        double energy_time=fabs(path_to_gate-energy_total_path)/
            fmax(future->true_air_speed,DBL_MIN);
        double turn_delta=norm_signed_deg(cfg->site.runway_heading-future_course);
        double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        double spatial_resolution=sqrt(DBL_EPSILON)*fmax(chord,1.0);
        double side;
        if(fabs(turn_delta)>angular_resolution)side=turn_delta>=0.0?-1.0:1.0;
        else if(fabs(future->runway_cross_track)>spatial_resolution)
            side=future->runway_cross_track>=0.0?1.0:-1.0;
        else side=g->s_turn_sign>=0.0?1.0:-1.0;

        if(constraints.feasible){
            double scale=fmax(1.0,fmax(best_energy_time,energy_time));
            bool lower_energy=!have_best||energy_time<best_energy_time-
                sqrt(DBL_EPSILON)*scale;
            bool same_energy=have_best&&
                fabs(energy_time-best_energy_time)<=sqrt(DBL_EPSILON)*scale;
            if(lower_energy||(same_energy&&c.length<best.length)){
                have_best=true;best=c;best_energy_time=energy_time;
                best_slope=geometric_slope;best_radius=effective_radius;best_side=side;
            }
        }else{
            double scale=fmax(1.0,fmax(fabs(fallback_worst_margin),
                fabs(constraints.worst_normalized_margin)));
            bool better_margin=!have_fallback||
                constraints.worst_normalized_margin>fallback_worst_margin+
                    sqrt(DBL_EPSILON)*scale;
            bool same_margin=have_fallback&&
                fabs(constraints.worst_normalized_margin-fallback_worst_margin)<=
                    sqrt(DBL_EPSILON)*scale;
            if(better_margin||(same_margin&&energy_time<fallback_energy_time)){
                have_fallback=true;fallback=c;
                fallback_worst_margin=constraints.worst_normalized_margin;
                fallback_energy_time=energy_time;
                fallback_slope=geometric_slope;fallback_radius=effective_radius;
                fallback_side=side;
                fallback_geometry_bad=c.degraded_control||c.degraded_rate||c.degraded_end;
                fallback_energy_bad=c.degraded_path;
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
        /* The energy search above used the projected future incidence. Carry
           that same state into the live candidate check; re-evaluating the
           path at the fixed HAC incidence would compare two different drag
           models and could reject a geometrically valid join for bookkeeping
           rather than vehicle energy. */
        .energy_aoa=clampd(fmax(3.0,future->angle_of_attack),3.0,
            terminal_hac_energy_aoa(v)),
        .altitude=future->mean_altitude,.speed=future->true_air_speed,
        .course=future_course,.selected_ut=live->ut,.arrival_ut=future->ut,
        .quality_score=nominal?best_energy_time:fallback_energy_time};
    *out=next;
    return true;
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
    if(spline_ok&&g->taem_interface_captured&&!spline.geometry_degraded){
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
            .lead_start={trial.hac_transition_lead_start_e,trial.hac_transition_lead_start_n},
            .p0={trial.hac_transition_p0_e,trial.hac_transition_p0_n},
            .p1={trial.hac_transition_p1_e,trial.hac_transition_p1_n},
            .p2={trial.hac_transition_p2_e,trial.hac_transition_p2_n},
            .p3={trial.hac_transition_p3_e,trial.hac_transition_p3_n},
            .lead_length=trial.hac_transition_lead_length,.length=trial.hac_transition_length,
            .end_angle=trial.hac_transition_end_angle,.arc_remaining=trial.hac_remaining,
            .exit_speed=trial.hac_transition_exit_speed};
        terminal_candidate_execution_cost(&hac,g,live,
            isfinite(live->ground_track_heading)?live->ground_track_heading:live->heading,
            p,aero,cfg);
        hac.quality_score=terminal_candidate_delivery_time(&hac);
        if(terminal_candidate_better(&best,&hac))best=hac;
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

static bool terminal_energy_path_unrecoverable(double expected_loss,double gravity,
        double kinetic_to_gate,double air_path,double recoverable_fpa_deg);

static bool terminal_candidate_vertical_response_ready_live(
        const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const TerminalCandidate*c);

static bool terminal_capture_margin_exhausted(const GuidanceMachine*g,
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

    EnergyPathEnvelope energy=decision_energy_path_envelope(
        t,p,available,modeled_work,path_to_gate);
    if(g&&energy.valid){
        g->terminal_candidate_live_energy_margin=energy.energy.margin;
        g->terminal_candidate_live_energy_valid=true;
    }
    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
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
        .lead_start={trial.hac_transition_lead_start_e,
            trial.hac_transition_lead_start_n},
        .p0={trial.hac_transition_p0_e,trial.hac_transition_p0_n},
        .p1={trial.hac_transition_p1_e,trial.hac_transition_p1_n},
        .p2={trial.hac_transition_p2_e,trial.hac_transition_p2_n},
        .p3={trial.hac_transition_p3_e,trial.hac_transition_p3_n},
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
    double drop=path_to_gate*tangent;

    if(drop_out)*drop_out=drop;
    if(height_out)*height_out=height;
    return response.valid&&response.reachable&&isfinite(drop)&&drop>=height;
}

static bool terminal_candidate_vertical_response_ready_live(const GuidanceMachine*g,
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

static void terminal_predict(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    g->terminal_reference_path_lateral_acceleration=0.0;
    g->terminal_reference_path_bank=0.0;
    g->terminal_reference_path_course_error=0.0;
    g->terminal_reference_path_arc_remaining=0.0;
    g->terminal_reference_path_transition_active=false;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(g->terminal_path_committed||(g->hac_side_selected&&!g->terminal_test_capture_active)||
       g->final_approach_captured||t->vertical_speed>=0.0||
       !authority.valid||!authority.controllable)return;
    /* Preview geometry is disposable.  Once its propagated intercept is in
       the past (or the live unpowered vehicle has dropped below both its
       altitude and speed), remove it before scoring replacements.  Otherwise
       the refinement hysteresis can preserve a once-nominal but physically
       unreachable HAC forever simply because newer candidates are degraded. */
    if(g->terminal_candidate.valid&&terminal_candidate_expired(g,&g->terminal_candidate,t,course,p,cfg)){
        g->terminal_candidate.valid=false;
        g->terminal_mix=0.0;
    }
    /* A geometric acquisition demand does not expire with a forecast clock.
       It is replaced by a reachable better topology or reset with the mission.
       There is no separate staging dogleg: Entry/MM304 owns lateral energy
       management, and MM305 publishes one acquisition target. */
    double staging_along=g->taem_interface_target.valid?
        g->taem_interface_target.along_track:-s->final_approach_distance;
    double staging_cross=g->taem_interface_target.valid?
        g->taem_interface_target.cross_track:0.0;
    double staging_gap=staging_along-t->runway_along_track;
    /* Until a terminal path is frozen, vertical energy must remain an owned
       state rather than whatever descent angle the airframe happens to
       acquire while banking for staging geometry.  Use the same speed-scaled
       path-slope envelope as the HAC energy solver, and aim at the TAEM shell
       over the *remaining two-dimensional staging path* (along-track plus the
       lateral reserve needed for the dogleg).  This gives roughly 8-12 deg
       while the shuttle is still hot and naturally steepens toward the
       18-ish degree cool terminal corridor as speed falls.

       The previous fallback simply copied the measured FPA into the
       reference.  Once lift fell during the dogleg, -20/-30 deg became the
       new target every frame, so TAEM preserved crossrange while spending all
       of its altitude and could never make an energy-degraded HAC nominal. */
    double minimum_staging_slope=0.0,maximum_staging_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_staging_slope,&maximum_staging_slope);
    double staging_cross_gap=staging_cross-t->runway_cross_track;
    double staging_vertical_path=hypot(fmax(0.0,staging_gap),staging_cross_gap);
    double response_distance=t->horizontal_speed*
        fmax(0.0,authority.control_response_time_s);
    double staging_height=fmax(0.0,t->mean_altitude-s->taem_interface_altitude);
    double staging_geometric_slope=atan2(staging_height,
        fmax(DBL_MIN,fmax(staging_vertical_path,response_distance)))*RAD2DEG;
    double staging_reference_fpa=-clampd(staging_geometric_slope,
        minimum_staging_slope,maximum_staging_slope);
    /* TAEM energy is controlled by drag work, not by trying to cancel drag
       with a steeper FPA.  The latter failed in the live replay because diving
       sooner moved the shuttle into denser air, which increased D/m faster
       than gravity could replenish speed.  Use the integrated atmosphere/
       drag model to choose the vertical path and reserve a per-metre drag
       budget all the way to preflare.  AoA is clamped to this budget below. */
    double staging_energy_fpa=staging_reference_fpa;
    bool staging_energy_priority=false;
    double staging_allowed_drag=INFINITY;
    if(g->terminal_region_entered||g->terminal_glide_mode){
        double actual_gate_air_path=terminal_uncommitted_gate_geometry(g,t,cfg,
            &staging_energy_fpa);
        if(fabs(staging_energy_fpa-staging_reference_fpa)>1.0)
            staging_energy_priority=true;
        /* The integrated solve owns the vertical reference once terminal
           guidance owns the vehicle.  This can be either shallower or steeper
           than the staging-shell geometry depending on actual energy. */
        staging_reference_fpa=staging_energy_fpa;

        double gate_altitude=cfg->site.altitude+fmax(0.0,g->terminal_test_preflare_altitude);
        double gate_speed=fmax(g->terminal_test_preflare_target_speed,
            g->terminal_test_preflare_min_speed);
        double available=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
            t->true_air_speed,cfg->site.latitude,gate_altitude,gate_speed,p);
        double reserve=.5*pow(fmax(8.0,.08*gate_speed),2);
        double budget_energy=fmax(0.0,available-reserve);
        double budget_path=actual_gate_air_path;
        staging_allowed_drag=budget_energy/budget_path;
        /* Keep a model-error reserve.  A low-confidence/rapidly changing drag
           history should preserve more kinetic energy, never less. */
        double confidence=clampd(t->trajectory_calibration_confidence,0.0,1.0);
        staging_allowed_drag*=.82+.10*confidence;
    }
    /* Once the uncommitted vehicle reaches the upstream staging station, its
       remaining path is no longer the few kilometres of along-runway closure.
       It must spend energy laterally until a flyable HAC exists.  Use that
       crossrange path in the vertical schedule as well, otherwise the old
       4 km ground-distance floor commands a needless 30 deg dive while the
       dogleg is trying to preserve maneuver room. */
    if(!g->terminal_prediction_valid&&
       (!g->terminal_region_entered||!isfinite(g->terminal_reference_fpa)||
        fabs(g->terminal_reference_fpa)<=DBL_EPSILON))
        g->terminal_reference_fpa=t->flight_path_angle;
    g->terminal_prediction_valid=true;
    /* The predictive slope can move promptly, but no longer has to slam to a
       -35 deg gravity-recovery dive because AoA/drag is now the primary energy
       actuator. */
    double max_change=fmax(0,dt)*(staging_energy_priority?1.4:.7);
    /* The turn candidate can need more response time than a wings-level
       planning probe. Its own publication window is authoritative. Include
       the selection debounce and the two-second planning refresh so a new
       candidate cannot start with an already impossible lead deadline. */
    ControlAuthorityEnvelope planning_authority=
        decision_control_authority_envelope(g,t,p,cfg);
    double candidate_response=hac_response_lead_time(t,s,0.0);
    if(g->terminal_candidate.valid&&isfinite(g->terminal_candidate.response))
        candidate_response=fmax(candidate_response,g->terminal_candidate.response);
    if(planning_authority.valid)
        candidate_response=fmax(candidate_response,
            planning_authority.control_response_time_s);
    if(!(candidate_response>=0.0)||!isfinite(candidate_response))
        candidate_response=0.0;

    double horizontal=fmax(t->horizontal_speed,DBL_MIN);
    double staging_eta=staging_gap>0.0?staging_gap/horizontal:0.0;
    double horizon=fmax(candidate_response,staging_eta);
    double prediction_refresh=fmax(s->prediction_interval,DBL_EPSILON);

    if(!g->terminal_planning_deferred&&
       t->ut-g->terminal_prediction_ut>=prediction_refresh){
        g->terminal_prediction_ut=t->ut;

        /*
         * Bound the search horizon by actual remaining ground time.  Speed-loss
         * time is another physically meaningful event horizon, not a weighting:
         * it marks when the current aerodynamic state reaches the configured
         * TAEM/minimum-safe speed domain.
         */
        double geometric_limit=fmax(0.0,t->range_to_site)/horizontal;
        double terminal_speed=fmax(v->minimum_safe_speed,
            v->final_approach_speed);
        double speed_loss=terminal_expected_speed_loss_accel(g,t,p,aero,v);
        double speed_eta=INFINITY;
        if(t->true_air_speed<=terminal_speed) speed_eta=0.0;
        else if(speed_loss>DBL_MIN)
            speed_eta=(t->true_air_speed-terminal_speed)/speed_loss;

        double end_horizon=fmax(horizon,candidate_response);
        if(isfinite(speed_eta))
            end_horizon=fmax(end_horizon,fmin(speed_eta,geometric_limit));
        else
            end_horizon=fmax(end_horizon,geometric_limit);
        if(g->terminal_candidate.valid&&
           isfinite(g->terminal_candidate.arrival_ut))
            end_horizon=fmax(end_horizon,
                fmax(candidate_response,
                    g->terminal_candidate.arrival_ut-t->ut));
        if(geometric_limit>=horizon)
            end_horizon=fmin(end_horizon,geometric_limit);
        end_horizon=fmax(end_horizon,horizon);

        /*
         * Before MM305 owns the orbiter, estimate ownership time from the
         * fixed MM304 handoff target itself.  This uses bounded-curvature
         * capture and measured control authority; it does not infer handoff
         * from a speed threshold.
         */
        double handoff_eta=0.0;
        bool handoff_time_known=g->terminal_region_entered;
        if(!g->terminal_region_entered&&g->taem_interface_target.valid){
            TargetCaptureEnvelope handoff=decision_target_capture_envelope(
                g,t,course,g->taem_interface_target.along_track,
                g->taem_interface_target.cross_track,
                g->taem_interface_target.course,
                fmax(geometric_limit,horizon),p,cfg);
            if(handoff.valid&&isfinite(handoff.required_time_s)){
                handoff_eta=handoff.required_time_s;
                handoff_time_known=true;
            }
        }

        TerminalCandidate best={0};
        Telemetry best_future={0};
        double best_horizon=0.0;
        const int samples=7; /* numerical search resolution only */
        for(int i=0;i<samples;i++){
            double f=(double)i/(double)(samples-1);
            double sample_horizon=horizon+(end_horizon-horizon)*f;
            if(staging_eta>=horizon&&staging_eta<=end_horizon&&i==samples/2)
                sample_horizon=staging_eta;

            Telemetry future;
            double e=0.0,n=0.0,future_course=course;
            terminal_project_planning_state(g,t,course,p,aero,cfg,
                sample_horizon,&future,&e,&n,&future_course);

            /* Do not retain a future-origin candidate whose own bounded path
               time cannot reach that origin before the forecast sample. */
            if(!terminal_future_origin_reachable(g,t,course,&future,
                    future_course,sample_horizon,p,cfg))
                continue;

            TerminalCandidate next={0};
            if(!terminal_candidate_from_future(g,t,&future,future_course,
                    p,aero,cfg,&next))
                continue;

            if(!g->terminal_region_entered){
                if(!handoff_time_known)continue;
                double post_handoff_lead=sample_horizon-handoff_eta;
                if(post_handoff_lead<next.response)continue;
            }

            if(terminal_candidate_better(&best,&next)){
                best=next;
                best_future=future;
                best_horizon=sample_horizon;
            }
        }

        /* A future-origin route may be geometrically regular yet fail the
           current-state energy contract once its response lead is charged.
           In that case, prefer a complete live-origin HAC only when it passes
           the same candidate-specific energy gate.  This preserves the
           physical response lead and never promotes a merely shorter or
           degraded path. */
        if(g->terminal_region_entered){
            TerminalCandidate live_origin={0};
            bool have_live_origin=terminal_live_origin_hac_candidate(
                g,t,course,p,aero,cfg,&live_origin);
            bool live_energy_ready=have_live_origin&&
                !live_origin.geometry_degraded&&
                terminal_candidate_live_energy_ready(g,t,p,aero,cfg,&live_origin);
            bool best_energy_ready=best.valid&&
                terminal_candidate_live_energy_ready(g,t,p,aero,cfg,&best);
            bool live_geometry_ready=have_live_origin&&
                terminal_candidate_geometry_executable(&live_origin);
            /* After MM304, a future-origin candidate with an unqualified live
               energy margin is only a forecast.  Prefer a regular candidate
               anchored at the actual vehicle state so the first turn begins
               now; otherwise the preview keeps moving its lead point ahead of
               the vehicle and can spend the entire handoff reserve flying a
               tangent.  A future candidate still wins when it has already
               passed the same live energy gate. */
            if(live_geometry_ready&&terminal_candidate_better(&best,&live_origin)&&
               (!best.valid||best.geometry_degraded||!best_energy_ready)){
                best=live_origin;
                best_future=*t;
                best_horizon=0.0;
            }
            if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
               strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&
               have_live_origin){
                fprintf(stderr,
                    "terminal topology live-origin: UT %.1f geom=%d energy=%d degraded=%d ready=%d margin %.0f q %.2f len %.0f arc %.0f\n",
                    t->ut,live_origin.geometry_degraded,live_origin.energy_degraded,
                    live_origin.degraded,live_energy_ready,
                    g->terminal_candidate_live_energy_margin,
                    live_origin.quality_score,live_origin.join.length,
                    live_origin.join.arc_remaining);
            }
        }

        bool timing_stale=false;
        bool timing_refresh=false;
        if(!g->terminal_region_entered&&g->terminal_candidate.valid&&
           handoff_time_known){
            double retained_post_handoff_lead=
                g->terminal_candidate.arrival_ut-t->ut-handoff_eta;
            timing_stale=retained_post_handoff_lead<
                g->terminal_candidate.response;
            if(timing_stale&&best.valid){
                double best_post_handoff_lead=
                    best.arrival_ut-t->ut-handoff_eta;
                timing_refresh=best_post_handoff_lead>=best.response&&
                    terminal_candidate_constraint_order(
                        &g->terminal_candidate,&best)<=0;
            }
        }

        if(g->terminal_candidate.valid)
            terminal_candidate_execution_cost(&g->terminal_candidate,g,t,course,p,aero,cfg);
        bool replace=best.valid&&
            (timing_refresh||
             terminal_candidate_refresh_allowed(g,
                &g->terminal_candidate,&best));

        if(replace){
            if(!g->diagnostic_shadow){
                fprintf(stderr,"HAC selected (post-MM304 interface; +1=south -1=north): UT %.2f side %+.0f R %.3f lead %.3f join %.3f arc %.3f execution %.3fm %.3fs courseError %.3fdeg radialClosure %+.3fm/s (%s) margin %.3fs horizon %.3fs degraded=%d reason=%s priorSide=%+.0f\n",
                    t->ut,best.side,best.radius,best.join.lead_length,best.join.length,
                    best.join.arc_remaining,best.execution_distance,best.execution_time,
                    best.execution_course_error,best.execution_radial_closure,
                    best.execution_radial_closure>=0.0?"toward":"away",
                    best.execution_margin,best.execution_horizon,best.degraded,
                    !g->terminal_candidate.valid?"initial":
                    terminal_candidate_constraint_order(&g->terminal_candidate,&best)<0?
                    "physical feasibility improvement":"acquisition saving exceeds response cost",
                    g->terminal_candidate.valid?g->terminal_candidate.side:0.0);
            }
            g->terminal_candidate=best;
            /*
             * Before MM305 ownership the terminal planner is advisory only.
             * MM304's fixed mission handoff target remains authoritative.
             */
            if(g->terminal_region_entered){
                if(best.join.valid&&hac_bezier_regular(&best.join))
                    terminal_publish_interface_target(g,t,p,aero,cfg);
                else
                    g->taem_interface_target.valid=false;
            }
            g->terminal_prediction_altitude=best_future.mean_altitude;
            g->terminal_prediction_speed=best_future.true_air_speed;
            g->terminal_prediction_time=best_horizon;
        }else if(g->terminal_candidate.valid){
            double retained_horizon=fmax(candidate_response,
                g->terminal_candidate.arrival_ut-t->ut);
            Telemetry future;
            double e=0.0,n=0.0,future_course=course;
            terminal_project_planning_state(g,t,course,p,aero,cfg,
                retained_horizon,&future,&e,&n,&future_course);
            g->terminal_prediction_altitude=future.mean_altitude;
            g->terminal_prediction_speed=future.true_air_speed;
            g->terminal_prediction_time=retained_horizon;
        }else{
            Telemetry future;
            double e=0.0,n=0.0,future_course=course;
            terminal_project_planning_state(g,t,course,p,aero,cfg,
                horizon,&future,&e,&n,&future_course);
            g->terminal_prediction_altitude=future.mean_altitude;
            g->terminal_prediction_speed=future.true_air_speed;
            g->terminal_prediction_time=horizon;
        }
    }
    bool candidate_operationally_usable=g->terminal_candidate.valid&&
        terminal_candidate_operationally_usable(g,&g->terminal_candidate,t,course,p,cfg);
    bool candidate_energy_ready=candidate_operationally_usable&&
        terminal_candidate_live_energy_ready(g,t,p,aero,cfg,&g->terminal_candidate);
    bool candidate_vertical_ready=candidate_operationally_usable&&
        terminal_candidate_vertical_response_ready_live(g,t,course,p,aero,cfg,
            &g->terminal_candidate);
    /* A geometry-clean path may be preview-tracked while its local vertical
       response catches up.  Requiring that response envelope before issuing
       the first path command creates a deadlock when the measured FPA is
       steeper than the candidate: the vehicle must track the candidate to
       become vertically ready.  Vertical response and energy still authorize
       commitment; geometry remains the hard prerequisite for any lateral
       preview. */
    bool operational_preview=candidate_operationally_usable&&
        (candidate_vertical_ready||
         (g->terminal_region_entered&&!g->terminal_candidate.geometry_degraded));
    /* Before a fully qualified join exists, there is no certified lateral path
       to fly.  Hold the measured ground course and wings level while the
       selector searches; converging toward a fixed station here would spend
       the handoff reserve on an uncommitted turn and can create the very
       cross-track debt the next candidate is meant to avoid. */
    double heading=operational_preview?
        ((g->terminal_region_entered||g->terminal_glide_mode)?
            terminal_uncommitted_alignment_heading(t,cfg):
            norm_deg(cfg->site.runway_heading+
                atan2(staging_cross_gap,staging_gap)*RAD2DEG)):
        norm_deg(course);
    double bank=operational_preview?
        clampd(norm_signed_deg(heading-course)*.65,-35,35):0.0;
    if(operational_preview){
        GuidanceMachine preview=*g;terminal_publish_candidate(&preview);
        HACGuidance path=hac_path_guidance(&preview,t,&cfg->site,s,p->radius,
            preview.hac_side,planet_surface_gravity(p),course,preview.hac_radius);
        g->terminal_reference_path_lateral_acceleration=path.lateral_acceleration;
        g->terminal_reference_path_bank=path.bank;
        g->terminal_reference_path_course_error=path.course_error;
        g->terminal_reference_path_arc_remaining=path.arc_remaining;
        g->terminal_reference_path_transition_active=preview.hac_transition_active;
        heading=path.heading;bank=taem_bank_demand(t,&path,preview.hac_radius,preview.hac_side,aero,v);
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint point={t->latitude,t->longitude,t->mean_altitude};double e,n;
        local_offsets(origin,point,p->radius,&e,&n);
        double distance=hypot(e-g->terminal_candidate.join.lead_start.e,n-g->terminal_candidate.join.lead_start.n);
        double fpa=-atan2(t->mean_altitude-g->terminal_candidate.altitude,fmax(distance,500))*RAD2DEG;
        double kinetic_loss=(t->true_air_speed*t->true_air_speed-
            g->terminal_candidate.speed*g->terminal_candidate.speed)/(2*fmax(distance,500));
        double energy_fpa=asin(clampd((kinetic_loss-live_drag_accel(t,aero,v))/
            fmax(planet_surface_gravity(p),.1),-1,1))*RAD2DEG;
        fpa+=clampd((energy_fpa-fpa)*.25,-4,4);
        /* A path that is dynamically flyable but too long for the current
           energy state needs TAEM to preserve energy while geometry
           converges.  Never let such a preview pull the vehicle steeper than
           the staging glide law; otherwise the path budget shrinks at least
           as quickly as the geometry and the candidate can never become
           committable. */
        if(g->terminal_region_entered||g->terminal_glide_mode){
            /* This preview already passed live energy and control checks.
               Fly its planned slope so the actual state can converge to the
               exact FPA contract used at commit. The straight-to-runway slope
               commanded -35 degrees while a valid HAC required -18, making
               capture impossible even with sufficient altitude and energy. */
            fpa=-clampd(g->terminal_candidate.slope,5.0,35.0);
        }else if(g->terminal_candidate.energy_degraded)
            fpa=fmax(fpa,staging_reference_fpa);
        g->terminal_reference_fpa+=clampd(clampd(fpa,-35,-5)-g->terminal_reference_fpa,-max_change,max_change);
    }else g->terminal_reference_fpa+=clampd(staging_reference_fpa-g->terminal_reference_fpa,
        -max_change,max_change);

    /* If actual descent still runs materially steeper than the uncommitted
       energy corridor, trade some lateral bank for vertical lift.  This is a
       bounded staging priority only; once a nominal HAC is committed, the
       frozen path owns bank normally. */
    if(!g->terminal_path_committed&&
       (!operational_preview||g->terminal_candidate.energy_degraded)){
        double descent_excess=staging_reference_fpa-t->flight_path_angle;
        if(descent_excess>3.0){
            double vertical_priority=clampd(1.0-(descent_excess-3.0)/14.0,.35,1.0);
            bank*=vertical_priority;
        }
    }
    double requested_aoa=terminal_fpa_force_aoa(t,p,aero,v,
        g->terminal_reference_fpa);
    /* A nominal-geometry candidate whose live drag budget is not yet ready is
       a conservation problem, not a reason to keep demanding the lift needed
       for the old staging FPA ramp.  Bound incidence by the demonstrated
       terminal energy AoA until the live path budget catches up.  The
       candidate still cannot be committed without the full energy gate above. */
    if(g->terminal_region_entered&&g->terminal_candidate.valid&&
       !g->terminal_candidate.geometry_degraded&&!candidate_energy_ready)
        requested_aoa=fmin(requested_aoa,terminal_hac_energy_aoa(v));
    if(isfinite(staging_allowed_drag)){
        double budget_aoa=terminal_drag_budget_aoa(t,aero,v,staging_allowed_drag,
            requested_aoa);
        requested_aoa=terminal_preview_recovery_aoa(requested_aoa,budget_aoa,
            g->terminal_reference_fpa,t->flight_path_angle);
        /* When even the present incidence is spending energy faster than the
           remaining path permits, lateral geometry is secondary to staying
           airborne.  Ease bank continuously instead of waiting for a later
           low-speed/stall recovery to undo the damage. */
        double measured_drag=live_drag_accel(t,aero,v);
        if(staging_allowed_drag>.05&&measured_drag>staging_allowed_drag*1.05){
            double ratio=measured_drag/fmax(staging_allowed_drag,.05);
            bank*=clampd(1.0-(ratio-1.0)*.18,.55,1.0);
        }
    }
    g->terminal_reference_heading=heading;
    double reference_bank=clampd(bank,-40,40);
    /* Candidate publication can replace a future-origin join with a different
       path tangent on the next sample.  A raw reference jump from one side of
       the vehicle to the other is not a valid preview: the native attitude
       plant can only change bank at its measured roll-rate limit.  Apply that
       same response envelope to the high-level reference so selector updates
       cannot manufacture an S-turn reversal that the airframe cannot follow.
       This is a command-bandwidth contract, not a trajectory preference. */
    if(g->terminal_region_entered&&dt>0.0&&isfinite(g->terminal_reference_bank)){
        double reference_rate=t->attitude_response.roll_valid&&
            isfinite(t->attitude_response.maximum_roll_rate_deg_s)?
            fabs(t->attitude_response.maximum_roll_rate_deg_s):
            fmax(1.0,s->taem_roll_rate);
        reference_bank=g->terminal_reference_bank+
            clampd(reference_bank-g->terminal_reference_bank,
                -reference_rate*dt,reference_rate*dt);
    }
    g->terminal_reference_bank=reference_bank;
    g->terminal_reference_aoa=clampd(requested_aoa,0.0,20.0);
    double target_mix=g->terminal_region_entered&&operational_preview?1.0:0.0;
    double convergence_time=fmax(candidate_response,DBL_MIN);
    double mix_alpha=clampd(fmax(0.0,dt)/convergence_time,0.0,1.0);
    g->terminal_mix+=(target_mix-g->terminal_mix)*mix_alpha;
}


/* Native planner worker entry point. This mutates only its private snapshot;
   execution ownership, command filters and control output remain on the live
   control thread. Offline shadows execute the same search synchronously. */
bool guidance_plan_terminal_preview(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg)return false;
    double previous=g->terminal_prediction_ut;
    g->terminal_planning_deferred=false;
    double course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
    terminal_predict(g,t,course,p,aero,cfg,0.0);
    return isfinite(g->terminal_prediction_ut)&&
        (!isfinite(previous)||g->terminal_prediction_ut>previous);
}

bool guidance_accept_terminal_preview(GuidanceMachine*g,const GuidanceMachine*request,
        const GuidanceMachine*result,const Telemetry*t,const LandingConfiguration*cfg){
    if(!g||!request||!result||!t||!cfg||!g->automation_engaged||g->paused||g->aborted||
       g->attitude_recovery||g->phase==PHASE_ATTITUDE_RECOVERY||g->terminal_path_committed||
       g->final_approach_captured||(g->hac_side_selected&&!g->terminal_test_capture_active)||
       g->phase!=request->phase||g->terminal_region_entered!=request->terminal_region_entered||
       g->terminal_reentry_after_ut!=request->terminal_reentry_after_ut||
       g->s_turn_sign!=request->s_turn_sign)return false;
    double age=t->ut-result->terminal_prediction_ut;
    if(!isfinite(age)||age<-.001||age>fmax(3.0,cfg->guidance.prediction_interval*2.0)||
       result->terminal_prediction_ut<=g->terminal_prediction_ut)return false;
    /* Never resurrect an intercept that the live vehicle has already passed.
       Candidate validity is not authority to commit: all existing live geometry,
       energy, response and capture gates remain in terminal_guidance(). */
    bool candidate_is_future=result->terminal_candidate.valid&&
        isfinite(result->terminal_candidate.arrival_ut)&&
        result->terminal_candidate.arrival_ut>t->ut;
    if(candidate_is_future){
        g->terminal_candidate=result->terminal_candidate;
        if(g->terminal_region_entered)g->taem_interface_target=result->taem_interface_target;
    }
    g->terminal_prediction_ut=result->terminal_prediction_ut;
    g->terminal_prediction_altitude=result->terminal_prediction_altitude;
    g->terminal_prediction_speed=result->terminal_prediction_speed;
    g->terminal_prediction_time=result->terminal_prediction_time;
    return true;
}

static bool terminal_candidate_commit_ready(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    const TerminalCandidate*c=&g->terminal_candidate;
    if(g->terminal_path_committed||!c->valid)return false;
    double live_course=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    if(!terminal_candidate_operationally_usable(g,c,t,live_course,p,cfg))return false;

    double runway_heading=cfg->site.runway_heading*DEG2RAD;
    double lead_along=c->join.lead_start.e*sin(runway_heading)+
        c->join.lead_start.n*cos(runway_heading);
    double lead_cross=c->join.lead_start.e*cos(runway_heading)-
        c->join.lead_start.n*sin(runway_heading);
    double arrival_horizon=fmax(0.0,c->arrival_ut-t->ut);
    double minimum_turn_radius=0.0;
    double forecast_path=decision_target_path_length(g,t,live_course,
        lead_along,lead_cross,c->course,NAN,p,cfg,&minimum_turn_radius);
    double forecast_time=isfinite(forecast_path)&&t->horizontal_speed>0.0?
        forecast_path/t->horizontal_speed:INFINITY;
    DecisionMargin forecast_margin=decision_margin(arrival_horizon,forecast_time);
    if(!forecast_margin.valid)return false;

    Telemetry arrival=*t;
    double arrival_e=0.0,arrival_n=0.0;
    double arrival_course=t->ground_track_heading;
    terminal_project_planning_state(g,t,t->ground_track_heading,p,aero,cfg,
        arrival_horizon,&arrival,&arrival_e,&arrival_n,&arrival_course);

    double response_drop=0.0,arrival_height_to_gate=0.0;
    bool vertical_ready=terminal_candidate_vertical_response_ready_from_arrival(
        g,&arrival,arrival_course,p,c,cfg,
        &response_drop,&arrival_height_to_gate);
    bool energy_ready=terminal_candidate_live_energy_ready(g,t,p,aero,cfg,c);

    bool ready=forecast_margin.margin>=0.0&&vertical_ready&&energy_ready;

    const char*diag_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    static double last_commit_diag_ut=-INFINITY;
    double diag_second=floor(t->ut);
    if(!ready&&!g->diagnostic_shadow&&diag_env&&strcmp(diag_env,"1")==0&&
       diag_second>last_commit_diag_ut){
        fprintf(stderr,
            "HAC commit margin: UT %.1f forecast %+.2fs (need %.2f avail %.2f path %.0f) vertical=%s energy=%s drop %.0f/%.0f turn %.0fm response %.2fs\n",
            t->ut,forecast_margin.margin,forecast_time,arrival_horizon,forecast_path,
            vertical_ready?"yes":"no",energy_ready?"yes":"no",
            response_drop,arrival_height_to_gate,
            minimum_turn_radius,c->response);
        last_commit_diag_ut=diag_second;
    }
    return ready;
}

static HACGuidance terminal_test_update_spiral(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double radius=fmax(1800.0,g->hac_radius);
    HACGuidance h=hac_guidance_radius(t,&cfg->site,s,p->radius,g->hac_side,
        planet_surface_gravity(p),course,radius);

    double progress_rate=hac_oriented_progress_rate(t,&cfg->site,s,p->radius,g->hac_side,radius,course);
    g->terminal_test_revolution_remaining=fmax(0,
        g->terminal_test_revolution_remaining-progress_rate*fmax(0,dt));

    double bank=fmin(55.0,dynamic_bank_limit(t,v));
    double min_turn=live_turn_radius(t,aero,v,bank);
    g->minimum_turn_radius=min_turn;
    double total_angle=g->terminal_test_revolution_remaining;
    /* Once a HAC has been selected, treat it as the lateral reference path.
       Re-solving or aggressively shrinking the circle while flying it moves
       the waypoint under the vehicle and turns a stable intercept into a
       chasing problem. Keep the computed HAC geometry fixed; radial/course
       pursuit plus the vertical/energy loops provide the small corrections
       needed after capture. */
    total_angle=g->terminal_test_revolution_remaining;
    g->hac_remaining=fmax(0,total_angle*radius);
    g->hac_previous_angle=h.angle;
    return h;
}

static double terminal_test_speed_floor(const VehicleProfile*v){
    return fmax(65.0,v->touchdown_speed*.88);
}

static GuidanceSettings terminal_path_settings(const GuidanceMachine*g,const GuidanceSettings*s){
    GuidanceSettings r=*s;
    if(g->terminal_glide_mode&&g->terminal_test_final_approach_distance>0)
        r.final_approach_distance=g->terminal_test_final_approach_distance;
    if(g->terminal_glide_mode&&g->terminal_test_glide_slope>0){
        r.final_glide_slope=g->terminal_test_glide_slope;
        r.taem_glide_slope=g->terminal_test_glide_slope;
    }
    if(g->hac_circuit_slope>0)r.taem_glide_slope=g->hac_circuit_slope;
    return r;
}

static double terminal_test_speed_shortfall(const Telemetry*t,double target_speed){
    return fmax(0.0,target_speed-t->true_air_speed-3.0);
}

static double terminal_test_energy_gamma_bias(const Telemetry*t,double target_speed){
    /* Energy recovery must key off the speed schedule we are actually trying
       to reach, not an absolute near-stall guard.  In the live HAC rehearsal
       the vehicle was already 25-40 m/s below the arc schedule while still
       above 100 m/s, so the old guard remained inactive.  The altitude loop
       then treated the naturally steeper low-energy glide as a path error and
       commanded more AoA, spending still more kinetic energy to climb back to
       the geometric path.  Move the commanded FPA progressively steeper as
       soon as schedule shortfall appears.  The earlier .28 gain arrested most
       of the loss but still settled around a 24-25 deg glide with TAS slowly
       decaying.  Give speed priority enough authority to use the already
       accepted ~35 deg outer-glide envelope; the bias fades automatically as
       airspeed returns to schedule. */
    return -clampd(terminal_test_speed_shortfall(t,target_speed)*.38,0,16.0);
}

static double terminal_test_energy_trim(const Telemetry*t,double target_speed,double nominal_aoa){
    /* FPA bias alone is not enough: at 120 m/s the previous law could still
       request 20+ deg AoA while trying to recover the high side of the path.
       Explicitly unload incidence when kinetic energy is below schedule so
       gravity can accelerate the aircraft instead of being converted into
       induced/profile drag.  Preserve a small positive incidence floor for
       controllability; once speed recovers the relief fades continuously back
       to the nominal measured/best-glide AoA. */
    /* The 0.20 deg/(m/s) law still left about 10 deg AoA with a 35-45 m/s
       speed deficit.  At that incidence the shuttle settled near a -22 deg
       glide even while guidance asked for -28..-30 deg, so it kept trading
       kinetic energy away instead of diving to recover it.  Unload more
       decisively once the deficit is established; 6-ish degrees remains a
       useful aerodynamic incidence while allowing gravity to rebuild speed. */
    double relief=clampd(terminal_test_speed_shortfall(t,target_speed)*.36,0,12.5);
    return clampd(nominal_aoa-relief,0.0,22.0);
}

static void terminal_test_energy_limit_aoa(GuidanceCommand*c,const Telemetry*t,
        double target_speed,double nominal_aoa){
    double shortfall=terminal_test_speed_shortfall(t,target_speed);
    if(!c||shortfall<=0)return;
    /* A steep real FPA used to feed back through aerodynamic_pitch_target as
       a request for still more AoA: while the aircraft was 25-35 m/s below
       schedule, the path-error term could raise a deliberately relieved
       11-13 deg trim back to 18+ deg.  Energy recovery has priority over
       climbing back to the geometric altitude line.  Collapse the extra
       path-restoration incidence as the shortfall grows, but keep a small
       incidence floor so the terminal controller retains useful lift and
       pitch authority during the dive. */
    double recovery=clampd(shortfall/30.0,0,1);
    double recovery_trim=terminal_test_energy_trim(t,target_speed,nominal_aoa);
    double ceiling=recovery_trim+(1.0-recovery)*5.0;
    c->target_aoa=clampd(fmin(c->target_aoa,ceiling),0.0,22.0);
    c->target_pitch=t->flight_path_angle+c->target_aoa;
}

static double terminal_taem_drag_aoa_ceiling(const Telemetry*t,const VehicleProfile*v,
        double expected_drag,double drag_budget,double nominal_aoa){
    /* Scale the observed specific-energy loss with the same incidence model
       used by the terminal predictor. Net TAS loss is unsuitable here: it
       already includes gravity's contribution along the descent. */
    double current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,fmax(1.0,hypot(t->angle_of_attack,t->sideslip)),
        v,NULL,&current_df);
    double upper=clampd(nominal_aoa,0.0,fmin(22.0,v->maximum_angle_of_attack));
    for(double aoa=upper;aoa>0.0;aoa=fmax(0.0,aoa-.5)){
        double df=1.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,NULL,&df);
        if(expected_drag*df/fmax(.05,current_df)<=drag_budget)return aoa;
    }
    return 0.0;
}

static double terminal_projected_lift_accel_at_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double aoa){
    double current_incidence=fmax(1.0,hypot(t->angle_of_attack,t->sideslip));
    double current_lf=0,target_lf=0;
    aerodynamic_force_factors_mach(t->mach,current_incidence,v,&current_lf,NULL);
    aerodynamic_force_factors_mach(t->mach,clampd(aoa,0.0,v->maximum_angle_of_attack),v,
        &target_lf,NULL);
    double measured=t->mass>1&&isfinite(t->lift_force)&&t->lift_force>0?
        t->lift_force/t->mass:0.0;
    double scaled_measured=measured>.005?
        measured*fabs(target_lf)/fmax(.05,fabs(current_lf)):0.0;
    double modeled=fmax(0.0,t->dynamic_pressure)/fmax(20.0,aero.ballistic_coefficient)*
        fmax(0.0,aero.lift_to_drag)*fabs(target_lf);
    return fmax(scaled_measured,modeled);
}

static double terminal_lateral_aoa_floor(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg){
    return terminal_required_aoa_for_lateral(t,aero,v,lateral_accel,bank_deg);
}

static bool terminal_energy_path_unrecoverable(double expected_loss,double gravity,
        double kinetic_to_gate,double air_path,double recoverable_fpa_deg){
    if(!isfinite(expected_loss)||!isfinite(gravity)||!isfinite(kinetic_to_gate)||
       !isfinite(air_path)||!isfinite(recoverable_fpa_deg)||air_path<=0.0)
        return true;
    double down=clampd(recoverable_fpa_deg,0.0,90.0)*DEG2RAD;
    double max_recoverable=fmax(0.0,gravity)*sin(down)+
        fmax(0.0,kinetic_to_gate)/air_path;
    return expected_loss>max_recoverable;
}

static bool terminal_vertical_path_unrecoverable(double height_to_gate,
        double ground_path_to_gate,double recoverable_fpa_deg){
    if(!isfinite(height_to_gate)||!isfinite(ground_path_to_gate)||
       !isfinite(recoverable_fpa_deg))return true;
    double down=clampd(recoverable_fpa_deg,0.0,nextafter(90.0,0.0))*DEG2RAD;
    double maximum_drop=fmax(0.0,ground_path_to_gate)*tan(down);
    return fmax(0.0,height_to_gate)>maximum_drop;
}

static void terminal_invalidate_frozen_path(GuidanceMachine*g);
static void terminal_invalidate_frozen_path_preserve_energy(GuidanceMachine*g);

static bool terminal_vertical_energy_priority(double flight_path_angle,double target_fpa,
        double expected_loss,double drag_budget){
    double shallow_error=flight_path_angle-target_fpa;
    return shallow_error>0.0||(drag_budget>0.0&&expected_loss>drag_budget);
}

static bool terminal_lateral_incidence_priority(const GuidanceMachine*g,bool energy_priority){
    /* Before publication, the preview may trade lateral convergence for a
       steeper energy-recovery dive because its geometry is still disposable.
       After publication the HAC is a hard lateral constraint: incidence may
       not be unloaded below the lift needed to remain on that frozen path.
       If the two requirements conflict, the path should have remained a
       preview instead of silently becoming unflyable after commit. */
    return g&&g->terminal_path_committed ? true : !energy_priority;
}

static double taem_bank_demand(const Telemetry*t,const HACGuidance*h,double hac_radius,double side,AerodynamicModel aero,const VehicleProfile*v){
    double lift=live_lift_accel(t,aero,v),eff=clampd(t->bank_effectiveness,.35,1.8),lim=fmin(60.0,dynamic_bank_limit(t,v));
    if(lift<=.005||hac_radius<=1)return 0;
    (void)side;
    double lateral=h->lateral_acceleration,limit_angle=clampd(lim*eff,0,89)*DEG2RAD;
    double bank=asin(clampd(fabs(lateral)/lift,0,sin(limit_angle)))/eff*RAD2DEG;
    return clampd(copysign(bank,lateral),-lim,lim);
}

static GuidanceResult taem_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    /* During MM305 acquisition, a geometry-clean candidate is deliberately
       advisory until its live energy and vertical-response margins qualify.
       Still, the aircraft must fly that candidate while it converges.  Use a
       private published view for path generation so the executive's committed
       bit remains false and the candidate can still be replaced by the next
       planner refresh. */
    GuidanceMachine preview_owner={0};
    const GuidanceMachine*path_owner=g;
    bool preview_candidate=g->terminal_glide_mode&&g->terminal_test_capture_active&&
        g->terminal_candidate.valid&&
        terminal_candidate_operationally_usable(g,&g->terminal_candidate,t,course,p,cfg)&&
        !g->terminal_candidate.geometry_degraded;
    if(preview_candidate){
        preview_owner=*g;
        terminal_publish_candidate(&preview_owner);
        path_owner=&preview_owner;
    }
    GuidanceSettings path_settings=terminal_path_settings(path_owner,s);
    const GuidanceSettings*hs=&path_settings;
    double radius=path_owner->terminal_glide_mode?
        fmax(s->hac_radius,path_owner->hac_radius):
        fmax(s->hac_radius,path_owner->hac_radius);
    RunwayCaptureEnvelope vertical_envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    double recoverable_fpa=vertical_envelope.valid?
        vertical_envelope.recoverable_fpa_deg:fmax(0.0,-t->flight_path_angle);
    TaemPhase lateral_stage=taem_exec_owns_vehicle(&g->taem_exec)?g->taem_exec.phase:
        (g->hac_captured&&!g->hac_transition_active?TAEM_PHASE_RUNWAY_ALIGNMENT:TAEM_PHASE_PATH_ACQUISITION);
    HACPhaseGuidance phase_path=hac_phase_guidance(lateral_stage,path_owner,t,&cfg->site,hs,p->radius,
        path_owner->hac_side,planet_surface_gravity(p),course,radius);
    if(!phase_path.valid){
        GuidanceCommand safe=atmospheric(t,t->ground_track_heading,0.0,v,0.0,false,PROFILE_TAEM);
        safe.heading_control_enabled=false;safe.has_target_aoa=true;
        safe.target_aoa=clampd(v->entry_angle_of_attack,8.0,v->maximum_angle_of_attack);
        safe.target_pitch=t->flight_path_angle+safe.target_aoa;
        return stabilized(g,result_make(PHASE_TAEM,safe,
            "MM305 phase has no qualified lateral path law; holding a bounded unloaded attitude.",
            "TAEM phase execution is inhibited until its dedicated path provider is available."),t,v,s,dt);
    }
    HACGuidance h=phase_path.guidance;
    double path_remaining=path_owner->hac_transition_active?h.arc_remaining:path_owner->hac_remaining;
    double hac_slope=path_owner->hac_circuit_slope>0?path_owner->hac_circuit_slope:(path_owner->terminal_glide_mode&&path_owner->terminal_test_glide_slope>0?path_owner->terminal_test_glide_slope:hs->taem_glide_slope);
    if(g->terminal_glide_mode){
        double final_entry_altitude=cfg->site.altitude+hs->final_approach_distance*tan(path_owner->terminal_test_glide_slope*DEG2RAD);
        h.desired_altitude=final_entry_altitude+path_remaining*tan(hac_slope*DEG2RAD);
    }else h.desired_altitude+=(g->hac_remaining-h.arc_remaining)*tan(hac_slope*DEG2RAD);
    h.arc_remaining=path_remaining;
    double path_length=fmax(hs->final_approach_distance,h.arc_remaining+hs->final_approach_distance),drag=live_drag_accel(t,aero,v);
    double final_target=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.08);
    double target_energy=rotating_specific_energy(cfg->site.latitude,cfg->site.altitude,final_target,p);
    double zero_speed_energy=rotating_specific_energy(t->latitude,t->mean_altitude,0,p);
    double allowable=fmax(final_target,sqrt(fmax(0,2*(target_energy+fmax(.03,drag)*path_length*.95-zero_speed_energy))));
    double speed_schedule=final_target+clampd(path_remaining/550.0,0,90.0);
    double gate_gamma=0.0,energy_gamma=0.0,expected_loss=0.0,drag_budget=0.0;
    bool committed_energy_unrecoverable=false,committed_vertical_unrecoverable=false;
    if(g->terminal_glide_mode){
        double gate_altitude=cfg->site.altitude+g->terminal_test_preflare_altitude;
        double gate_speed=g->terminal_test_preflare_target_speed;
        double gate_potential=rotating_specific_energy(cfg->site.latitude,gate_altitude,0.0,p);
        double gate_ground=g->terminal_test_preflare_altitude/
            fmax(tan(g->terminal_test_glide_slope*DEG2RAD),1e-3);
        double path_to_gate=fmax(0.0,path_remaining)+
            fmax(0.0,hs->final_approach_distance-gate_ground);
        double height=fmax(0.0,t->mean_altitude-gate_altitude);
        double air_path=fmax(1.0,hypot(path_to_gate,height));
        double potential=zero_speed_energy-gate_potential;
        double kinetic=.5*(t->true_air_speed*t->true_air_speed-gate_speed*gate_speed);
        double gravity=fmax(.1,planet_surface_gravity(p));
        expected_loss=terminal_expected_energy_loss_accel(g,t,aero,v);
        /* Close height and kinetic energy over the SAME remaining path.
           With dK/ds=-kinetic/air_path, excess V^2 falls in proportion to
           distance remaining, reaching gate speed only at the gate. At or
           below gate speed the permitted deceleration is zero or negative.
           D = -g*sin(gamma) + kinetic/air_path supplies the corresponding
           FPA; actual height independently supplies the geometric closure.
           Unlike the old inverse energy equation, abundant potential energy
           cannot collapse an immediate speed target to the preflare floor. */
        gate_gamma=-atan2(height,fmax(path_to_gate,1.0))*RAD2DEG;
        energy_gamma=-asin(clampd((expected_loss-kinetic/air_path)/gravity,
            -1.0,1.0))*RAD2DEG;
        /* D is loss per AIR metre, not horizontal metre. Bound it by both
           the total gate-energy budget and gravity along the CURRENT FPA:
           a still-shallow aircraft must not spend speed while waiting for
           the commanded dive. Credit no descent beyond the safe envelope. */
        double gravity_along_path=-gravity*sin(
            clampd(t->flight_path_angle,-recoverable_fpa,90.0)*DEG2RAD);
        drag_budget=fmax(0.0,fmin((potential+kinetic)/air_path,
            gravity_along_path+kinetic/air_path));
        committed_energy_unrecoverable=g->terminal_path_committed&&
            terminal_energy_path_unrecoverable(expected_loss,gravity,kinetic,air_path,
                recoverable_fpa);

        committed_vertical_unrecoverable=g->terminal_path_committed&&
            terminal_vertical_path_unrecoverable(height,path_to_gate,recoverable_fpa);
        /* This is now only the recovery floor, not an intermediate TAS
           demand. The rate/drag law above owns planned deceleration. */
        speed_schedule=gate_speed;
        allowable=speed_schedule;
    }

    if(g->terminal_path_committed&&g->terminal_glide_mode){
        bool replan_margin=vertical_envelope.valid&&
            vertical_envelope.runway_time.margin>vertical_envelope.control_response_time_s&&
            vertical_envelope.vertical_recovery.margin>=0.0&&
            vertical_envelope.speed.margin>=0.0;
        bool frozen_path_unrecoverable=committed_energy_unrecoverable||
            committed_vertical_unrecoverable;
        if(frozen_path_unrecoverable&&replan_margin)
            g->terminal_energy_mismatch_duration+=fmax(0.0,dt);
        else
            g->terminal_energy_mismatch_duration=0.0;

        double confirmation_time=vertical_envelope.valid?
            fmax(dt,vertical_envelope.control_response_time_s):INFINITY;
        if(g->terminal_energy_mismatch_duration>=confirmation_time){
            /* A frozen path is no longer executable if either measured drag
               exceeds the unpowered energy budget or its remaining ground
               distance cannot close the altitude inside the -35 deg FPA
               envelope. Preserve measured energy, discard only geometry,
               and let TAEM search a replacement while maneuver room remains. */
            bool vertical_miss=committed_vertical_unrecoverable;
            terminal_invalidate_frozen_path_preserve_energy(g);
            g->terminal_energy_mismatch_duration=0.0;
            terminal_predict(g,t,course,p,aero,cfg,dt);
            GuidanceCommand replanning=atmospheric(t,g->terminal_reference_heading,
                g->terminal_reference_bank,v,0.0,false,PROFILE_TAEM);
            replanning.heading_control_enabled=false;
            replanning.has_target_aoa=true;
            replanning.target_aoa=g->terminal_reference_aoa;
            replanning.target_pitch=t->flight_path_angle+replanning.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,replanning,
                vertical_miss?
                    "Frozen terminal path cannot close altitude inside the allowed FPA envelope; replanning longer geometry in TAEM.":
                    "Frozen terminal path energy model diverged from live drag; replanning terminal geometry in TAEM.",
                vertical_miss?
                    "The remaining ground path cannot reach preflare height even at the maximum allowed terminal descent; the frozen path was discarded before outer-final passage.":
                    "Measured aerodynamic loss cannot be recovered within the allowed terminal FPA envelope; the previous terminal path was discarded without returning to S-turn."),
                t,v,s,dt);
        }
    }else g->terminal_energy_mismatch_duration=0.0;
    double ae=h.desired_altitude-t->mean_altitude;
    double path_limit=fmax(0.0,recoverable_fpa);
    double path=robust_pid_update(&g->taem_altitude_pid,ae,dt,-path_limit,path_limit);
    double energy_schedule=speed_schedule;
    double gamma_request=g->terminal_glide_mode?
        fmin(-hac_slope+path,fmin(gate_gamma,energy_gamma)):
        -hac_slope+path+terminal_test_energy_gamma_bias(t,energy_schedule);
    double gamma=fmax(-recoverable_fpa,gamma_request);
    if(g->terminal_path_committed&&g->terminal_glide_mode&&
       isfinite(g->terminal_test_glide_slope)&&
       g->terminal_test_glide_slope>0.0){
        /* A committed candidate was certified against its selected vertical
           slope.  The recoverable-FPA bound is an airframe limit, not a new
           route: letting the energy loop replace the candidate with a much
           steeper descent spends altitude and lateral authority faster than
           the path provider modeled, then reports the resulting cross-track
           loss as a late live-energy failure.  Keep the path's FPA contract
           as the shallow-side bound; if the contract is no longer flyable,
           the mismatch/replan logic above must discard it rather than silently
           turning it into a different trajectory.
         */
        double committed_path_fpa=-clampd(g->terminal_test_glide_slope,
            0.0,fmax(0.0,recoverable_fpa));
        gamma=fmax(gamma,committed_path_fpa);
    }
    if(g->terminal_path_committed){
        g->terminal_reference_fpa=gamma;
    }
    double learned=t->calibrated_best_glide_angle_of_attack>0?
        t->calibrated_best_glide_angle_of_attack:v->entry_angle_of_attack;
    double glide=clampd(learned,0.0,v->maximum_angle_of_attack);
    double trim=g->terminal_glide_mode?
        terminal_test_energy_trim(t,energy_schedule,glide):
        clampd(glide+clampd((t->true_air_speed-allowable)*.012,0,6),3,14);
    if(!g->terminal_glide_mode&&terminal_test_speed_shortfall(t,energy_schedule)>0)
        trim=fmin(trim,terminal_test_energy_trim(t,energy_schedule,glide));
    double energy_aoa_ceiling=v->maximum_angle_of_attack;
    bool terminal_unload=g->terminal_glide_mode&&
        !g->terminal_path_committed&&t->flight_path_angle>-recoverable_fpa;
    if(terminal_unload){
        energy_aoa_ceiling=terminal_taem_drag_aoa_ceiling(t,v,expected_loss,drag_budget,glide);
        trim=fmin(trim,energy_aoa_ceiling);
    }else if(g->terminal_glide_mode)trim=glide;
    /*
     * Explicit mission contract: terminal energy control has only trajectory,
     * bank and AoA available.  Do not treat speedbrakes as latent energy
     * authority even when a physics model for them is present.
     */
    terminal_speedbrake_closed(g);
    double hac_course_rate_limit_deg=INFINITY;
    if(g->hac_side_selected&&g->terminal_region_entered){
        /* Keep actual ground-track rotation in the same bandwidth as the
           moving heading reference.  During the join, release the rate with
           exactly the same smooth progress law used by the heading limiter;
           a separate fixed 1.25 deg/s ceiling here used to pin the transition
           forever even after its reference had legitimately sped up.  Once
           on the HAC, allow roughly the circle's natural V/R rate with only a
           modest comfort ceiling. */
        double max_course_rate_deg;
        if(g->hac_transition_active){
            max_course_rate_deg=hac_join_rate_cap(t,g->hac_transition_progress);
        }else{
            /* Once established on the frozen circle, never cap below its own
               V/R feed-forward.  The selector guarantees that this natural
               rate remains inside the shared continuous envelope. */
            double natural=t->true_air_speed/fmax(radius,1.0)*RAD2DEG;
            max_course_rate_deg=clampd(natural*1.25,1.05,hac_course_rate_cap(t));
        }
        /* Energy recovery is handled by the FPA/AoA schedule.  Reducing the
           lateral-rate cap below the frozen path curvature made the aircraft
           save lift by abandoning the HAC, which is not a valid reference
           law and caused radial error to grow monotonically. */
        hac_course_rate_limit_deg=max_course_rate_deg;
        double max_course_rate=max_course_rate_deg*DEG2RAD;
        double lateral_cap=fmax(.45,t->true_air_speed*max_course_rate);
        h.lateral_acceleration=clampd(h.lateral_acceleration,-lateral_cap,lateral_cap);
    }
    double bank=taem_bank_demand(t,&h,radius,g->hac_side,aero,v);
    double incidence_fraction=clampd((t->angle_of_attack+1.0)/8.0,0,1);
    double bank_guard=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    if(vertical_envelope.valid&&vertical_envelope.maximum_normal_accel_mps2>0.0){
        double gravity=planet_surface_gravity(p);
        double required_vertical=gravity+
            fmax(0.0,-t->vertical_speed)/
            fmax(vertical_envelope.time_available_s,DBL_MIN);
        double ratio=clampd(required_vertical/
            vertical_envelope.maximum_normal_accel_mps2,0.0,1.0);
        double vertical_bank_limit=acos(ratio)*RAD2DEG;
        bank_guard=fmin(bank_guard,vertical_bank_limit);
    }
    (void)incidence_fraction;
    if(g->hac_side_selected&&g->terminal_region_entered&&isfinite(hac_course_rate_limit_deg)&&
       !g->terminal_glide_mode){
        /* Non-terminal TAEM still uses the coordinated-turn bank envelope.
           In the unpowered terminal HAC, however, course rate has already
           been limited by clamping lateral acceleration to V*omega above.
           Applying atan(V*omega/g) a second time assumes lift ~= weight and
           forces an artificially small bank during a steep, deliberately
           unloaded glide.  The lateral-AoA allocator then has to raise AoA
           back toward 10 deg to make the same lateral force, which adds drag
           and vertical lift and flattens the requested -30..-35 deg descent.
           Let terminal HAC use the measured/projected lift with the existing
           dynamic bank envelope; the already-clamped lateral acceleration is
           the actual course-rate safety constraint. */
        double rate_deg=hac_course_rate_limit_deg;
        double gravity=fmax(.1,planet_surface_gravity(p));
        double rate_bank=atan(t->true_air_speed*(rate_deg*DEG2RAD)/gravity)*RAD2DEG;
        bank_guard=fmin(bank_guard,clampd(rate_bank*1.05,6.0,45.0));
    }
    bank=clampd(bank,-bank_guard,bank_guard);
    /*
     * STS-N terminal flight is fully unpowered.  An energy deficit remains a
     * feasibility/control problem; it may not be converted into a thrust
     * recovery branch by configuration.
     */
    robust_pid_reset(&g->speed_pid);
    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,h.heading,bank,v,0.0,false,PROFILE_TAEM);
    /* Do not re-impose the old -12 deg surface-pitch floor after the terminal
       energy law has deliberately unloaded AoA.  With FPA near -23 deg that
       floor mechanically forced ~11 deg incidence, so the shuttle could not
       dive toward the requested -28..-30 deg path even while 40 m/s below its
       speed schedule.  The terminal gamma law is already bounded to -35 deg
       and incidence unloading is suspended at that descent limit. */
    double taem_pitch_floor=(g->terminal_glide_mode||g->hac_circuit_count>0||t->true_air_speed<energy_schedule)?-30.0:-12.0;
    if(g->terminal_path_committed&&g->terminal_glide_mode){
        /* A committed candidate is certified against the force-derived AoA
           needed to move the measured velocity vector toward its selected
           FPA.  Converting that FPA back through gamma+nominal-trim asks for
           the wrong surface pitch when the live FPA is still steeper than the
           route; in the 22 km probe it turned a ~6 deg requirement into
           ~26--28 deg commanded AoA.  Use the same bounded reference that the
           selector and live-energy gate evaluated.  The lateral allocator
           below may raise it if the frozen curvature requires more lift.
         */
        double committed_aoa=terminal_fpa_force_aoa(t,p,aero,v,gamma);
        if(isfinite(committed_aoa)){
            c.target_aoa=clampd(committed_aoa,0.0,v->maximum_angle_of_attack);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }else{
            aerodynamic_pitch_target(&c,t,v,clampd(gamma+trim,taem_pitch_floor,24));
        }
    }else{
        aerodynamic_pitch_target(&c,t,v,clampd(gamma+trim,taem_pitch_floor,24));
    }
    if(!g->terminal_glide_mode||terminal_unload)
        terminal_test_energy_limit_aoa(&c,t,energy_schedule,glide);
    if(terminal_unload){
        c.target_aoa=fmin(c.target_aoa,energy_aoa_ceiling);
        c.target_pitch=t->flight_path_angle+c.target_aoa;
    }
    double hard_lateral_aoa_floor=0.0;
    if(g->terminal_glide_mode&&g->hac_side_selected){
        /* The direct atmospheric controller closes the loop on AoA, not FPA.
           Therefore a -35 deg gamma request is only achieved dynamically by
           unloading lift until gravity steepens the velocity vector. The old
           5 deg incidence floor left enough lift at Mach 2-3 to settle near
           -13..-15 deg even while gamma stayed at -35. Use the measured FPA
           error as an additional unload term; it fades continuously once the
           velocity vector catches the commanded descent. */
        double shallow_gamma_error=t->flight_path_angle-gamma;
        if(shallow_gamma_error>3.0){
            double extra_unload=clampd((shallow_gamma_error-3.0)*.20,0.0,4.5);
            double incidence_floor=terminal_test_speed_shortfall(t,energy_schedule)>12.0?0.0:.5;
            c.target_aoa=fmax(incidence_floor,c.target_aoa-extra_unload);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
        /* Energy recovery and lateral capture share the same lift vector.
           First ask whether the requested curvature can be flown at the
           *maximum permitted bank* with the energy-relieved incidence. The
           old code evaluated the AoA floor at the current small bank, then
           raised incidence to 12-14 deg even when simply banking a little
           more would have produced the same lateral acceleration. That
           cancelled the terminal unload command and flattened the descent.
           Only add incidence when the bank envelope itself is insufficient. */
        bool energy_priority=terminal_vertical_energy_priority(
            t->flight_path_angle,gamma,expected_loss,drag_budget);
        double lateral_floor=terminal_lateral_aoa_floor(t,aero,v,h.lateral_acceleration,bank_guard);
        hard_lateral_aoa_floor=lateral_floor;
        /* A committed path is only valid while its lateral curvature and its
           vertical energy law can be satisfied by the same lift vector.  In
           the live 86 km replay the frozen 48 km HAC eventually needed about
           ten degrees of incidence for lateral force while energy recovery
           needed roughly three to five degrees to steepen from -11 toward
           -25..-31 deg.  The hard lateral floor therefore cancelled the
           unload command for tens of seconds until the path became formally
           unrecoverable.  If that conflict is already strong while we still
           have replanning height, discard only the frozen geometry and let
           TAEM keep the energy-preserving command while it searches a shorter
           path. */
        double energy_limited_aoa=c.target_aoa;
        double incidence_conflict=lateral_floor-energy_limited_aoa;
        double conflict_replan_height=fmax(3500.0,g->terminal_test_preflare_altitude*5.0);
        bool conflict_replan_margin=g->terminal_path_committed&&
            h.arc_remaining>s->final_approach_distance+3000.0&&
            t->radar_altitude>conflict_replan_height&&
            t->true_air_speed>v->minimum_safe_speed*1.45;
        bool lateral_energy_conflict=conflict_replan_margin&&energy_priority&&
            shallow_gamma_error>7.0&&incidence_conflict>2.0&&
            expected_loss>drag_budget*1.05+.10;
        if(lateral_energy_conflict){
            terminal_invalidate_frozen_path_preserve_energy(g);
            terminal_predict(g,t,course,p,aero,cfg,dt);
            GuidanceCommand replanning=atmospheric(t,g->terminal_reference_heading,
                g->terminal_reference_bank,v,0.0,false,PROFILE_TAEM);
            replanning.heading_control_enabled=false;
            replanning.has_target_aoa=true;
            replanning.target_aoa=g->terminal_reference_aoa;
            replanning.target_pitch=t->flight_path_angle+replanning.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,replanning,
                "Frozen HAC lateral lift conflicted with terminal energy recovery; replanning shorter geometry in TAEM.",
                "The committed curvature required enough incidence to prevent the commanded energy-recovery descent; measured drag history was retained for the replacement plan."),
                t,v,s,dt);
        }
        /* While the HAC is still only a preview, a materially shallow or
           over-dissipating vehicle may let vertical energy closure own
           incidence and accept temporary lateral lag.  Once the path is
           committed that trade is no longer legal: the frozen curvature gets
           its required lift first and the vertical profile must absorb the
           remaining energy error. */
        if(terminal_lateral_incidence_priority(g,energy_priority)&&
           lateral_floor>0.0&&c.target_aoa<lateral_floor){
            c.target_aoa=lateral_floor;
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
        /* Pitch response is finite.  Keep bank allocation conservative with
           respect to the lift that exists *now*.  Preview flight can still
           unload for energy; committed flight keeps the lateral incidence
           floor above and therefore preserves the lift assumed by the frozen
           curvature. */
        double projected_target_lift=terminal_projected_lift_accel_at_aoa(t,aero,v,c.target_aoa);
        double target_lift=fmax(live_lift_accel(t,aero,v),projected_target_lift);
        double eff=clampd(t->bank_effectiveness,.35,1.8);
        double maximum_component=sin(clampd(bank_guard*eff,0.0,89.0)*DEG2RAD);
        if(fabs(h.lateral_acceleration)>=.35&&target_lift>.01&&maximum_component>.01){
            double required_component=clampd(fabs(h.lateral_acceleration)*1.05/target_lift,
                0.0,maximum_component);
            double required_bank=asin(required_component)/eff*RAD2DEG;
            c.target_roll=clampd(copysign(required_bank,h.lateral_acceleration),
                -bank_guard,bank_guard);
        }
    }
    if(g->terminal_path_committed){
        /* Crossfade from the already-followed TAEM preview into the exact
           frozen join.  The transition geometry is continuous, but a commit
           can still move bank/AoA references by a few degrees; blending those
           references explicitly makes the ownership change visually natural
           instead of relying on actuator rate limits to hide a command step. */
        double blend_time=clampd(fmax(3.0,g->hac_transition_response_time*.85),3.0,8.0);
        g->hac_commit_blend=clampd(g->hac_commit_blend+fmax(0.0,dt)/blend_time,0.0,1.0);
        double q=g->hac_commit_blend;
        q=q*q*(3.0-2.0*q);
        c.target_heading=norm_deg(g->terminal_reference_heading+
            q*norm_signed_deg(c.target_heading-g->terminal_reference_heading));
        c.target_roll=g->terminal_reference_bank+
            q*(c.target_roll-g->terminal_reference_bank);
        if(c.has_target_aoa){
            c.target_aoa=g->terminal_reference_aoa+
                q*(c.target_aoa-g->terminal_reference_aoa);
            /* The ownership crossfade is cosmetic; it may not undo the hard
               lift floor of a committed path.  This matters most immediately
               after commit, when the frozen preview AoA can still be lower
               than the incidence required by the first curved join segment. */
            if(hard_lateral_aoa_floor>0.0)
                c.target_aoa=fmax(c.target_aoa,hard_lateral_aoa_floor);
            c.target_pitch=t->flight_path_angle+c.target_aoa;
        }
    }
    c.gear=t->gear;
    bool spline_path=path_owner->terminal_path_kind==TERMINAL_PATH_SPLINE;
    char status[320],warn[256]={0};
    if(spline_path)
        snprintf(status,sizeof(status),"Aero TAEM spline: %s %.1f km, cross %+.1f km, speed %.0f / %.0f m/s, FPA %.1f / %.1f deg, energy ceiling %.0f m/s.",g->hac_transition_active?"curve remaining":"line capture",h.arc_remaining/1000,t->runway_cross_track/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,allowable);
    else snprintf(status,sizeof(status),"Aero TAEM HAC: R %.1f km, %s %.1f km, radial %+.1f km, speed %.0f / %.0f m/s, FPA %.1f / %.1f deg, energy ceiling %.0f m/s.",radius/1000,g->hac_transition_active?"join+arc":"arc",h.arc_remaining/1000,h.radial_error/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,allowable);
    if(g->terminal_glide_mode){
        if(spline_path)
            snprintf(status,sizeof(status),"Aero TAEM spline: %s %.1f km, cross %+.1f km, speed %.0f m/s, preflare gate %.0f m/s, FPA %.1f / %.1f deg, drag %.2f / %.2f m/s2 budget.",g->hac_transition_active?"curve remaining":"line capture",h.arc_remaining/1000,t->runway_cross_track/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,expected_loss,drag_budget);
        else snprintf(status,sizeof(status),"Aero TAEM HAC: R %.1f km, %s %.1f km, radial %+.1f km, speed %.0f m/s, preflare gate %.0f m/s, FPA %.1f / %.1f deg, drag %.2f / %.2f m/s2 budget.",radius/1000,g->hac_transition_active?"join+arc":"arc",h.arc_remaining/1000,h.radial_error/1000,t->true_air_speed,energy_schedule,t->flight_path_angle,gamma,expected_loss,drag_budget);
    }
    if(g->hac_circuit_count>0){
        size_t used=strlen(status);
        snprintf(status+used,sizeof(status)-used," High-pass circuit %u.",g->hac_circuit_count);
    }
    if(g->hac_completed&&!g->final_approach_captured&&!g->taem_exec.terminal_evaluation.feasible){
        const TaemTerminalContract*contract=&g->taem_exec.terminal_contract;
        const char*reason=taem_terminal_block_reason_string(g->taem_exec.terminal_evaluation.block_reason);
        if(g->taem_exec.terminal_evaluation.valid)
            snprintf(warn,sizeof(warn),"Final delivery blocked: %s; margins range %.0f m, q %.0f Pa, altitude %.0f m, FPA %.1f deg, energy %.0f J/kg, response %.1f/%.1f s.",
                reason,contract->range_margin,contract->dynamic_pressure_margin,contract->altitude_margin,
                contract->flight_path_angle_margin,contract->specific_energy_margin,
                contract->response_time_available,contract->response_time_required);
        else snprintf(warn,sizeof(warn),"Final delivery blocked: %s.",reason);
    }else if(g->hac_plan_degraded)
        snprintf(warn,sizeof(warn),"Flying the best available terminal path; one or more preferred feasibility envelopes are violated.");
    else if(g->terminal_glide_mode&&t->true_air_speed<g->terminal_test_preflare_min_speed)
        snprintf(warn,sizeof(warn),"Terminal energy is below the preflare floor; unloading AoA and trading altitude for airspeed.");
    else if(fabs(ae)>8000)snprintf(warn,sizeof(warn),"TAEM altitude is outside the model-derived path corridor.");
    else if(!spline_path&&fabs(h.radial_error)>radius*.25)snprintf(warn,sizeof(warn),"Capturing the model-derived HAC radius.");
    GuidanceResult r=result_make(g->phase,c,status,warn[0]?warn:NULL);trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static double terminal_outer_glide_slope(const GuidanceMachine*g,const GuidanceSettings*s){
    return g->terminal_glide_mode&&g->terminal_test_glide_slope>0?g->terminal_test_glide_slope:s->final_glide_slope;
}

static double __attribute__((unused)) terminal_preflare_altitude(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    double slope=terminal_outer_glide_slope(g,s);
    double nominal_sink=t->horizontal_speed*tan(slope*DEG2RAD);
    double sink=fmax(nominal_sink,fmax(0,-t->vertical_speed));
    if(g->terminal_glide_mode){
        double minimum=fmax(g->terminal_test_preflare_altitude,fmax(450.0,s->flare_altitude*8.0));
        return clampd(fmax(minimum,sink*7.0),minimum,900.0);
    }
    return clampd(sink*7.0,fmax(180.0,s->flare_altitude*3.0),700.0);
}

typedef struct {
    bool feasible;
    double trigger_altitude;
    double target_aoa;
    double target_sink;
    double minimum_speed;
    double reference_speed;
    double predicted_height_loss;
    double predicted_kinetic_margin;
    double effective_accel;
    double response_time;
    DecisionMargin height;
} TerminalPreflarePlan;

static double terminal_final_distance(const GuidanceMachine*g,const GuidanceSettings*s){
    if(g->terminal_glide_mode&&g->terminal_test_final_approach_distance>0)
        return g->terminal_test_final_approach_distance;
    return s->final_approach_distance;
}

static void terminal_observe_response(GuidanceMachine*g,const Telemetry*t,double dt){
    if(!g->terminal_response_sample_valid){
        g->terminal_response_sample_valid=true;
        g->terminal_previous_aoa=t->angle_of_attack;
        g->terminal_previous_vertical_speed=t->vertical_speed;
        g->terminal_response_sample_ut=t->ut;
        return;
    }
    double sample_dt=t->ut-g->terminal_response_sample_ut;
    if(!(sample_dt>=.04&&sample_dt<=.75)){
        g->terminal_previous_aoa=t->angle_of_attack;
        g->terminal_previous_vertical_speed=t->vertical_speed;
        g->terminal_response_sample_ut=t->ut;
        return;
    }
    double aoa_rate=(t->angle_of_attack-g->terminal_previous_aoa)/sample_dt;
    double sink_accel=(t->vertical_speed-g->terminal_previous_vertical_speed)/sample_dt;
    if(isfinite(aoa_rate)&&aoa_rate>.15){
        double observed=clampd(aoa_rate,.25,4.0);
        g->terminal_positive_aoa_rate_ema=.88*g->terminal_positive_aoa_rate_ema+.12*observed;
        if(isfinite(sink_accel)&&sink_accel>.1){
            double accel=clampd(sink_accel,.1,6.0);
            g->terminal_sink_accel_ema=g->terminal_sink_accel_ema>0?
                .90*g->terminal_sink_accel_ema+.10*accel:accel;
        }
    }
    (void)dt;
    g->terminal_previous_aoa=t->angle_of_attack;
    g->terminal_previous_vertical_speed=t->vertical_speed;
    g->terminal_response_sample_ut=t->ut;
}

static TerminalPreflarePlan terminal_preflare_plan(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    TerminalPreflarePlan plan={0};

    double gravity=planet_surface_gravity(p);
    if(!(gravity>0.0)||!isfinite(t->vertical_speed)||!isfinite(t->true_air_speed))
        return plan;

    double sink=fmax(0.0,-t->vertical_speed);
    double target_sink=fmax(0.0,-s->touchdown_sink_rate);
    RunwayCaptureEnvelope recovery=decision_runway_capture_envelope(
        g,t,cfg->site.runway_heading,p,cfg);
    if(!recovery.valid)return plan;
    double target_aoa=recovery.maximum_lift_aoa_deg;

    double current_lf=0.0,target_lf=0.0,current_df=0.0,target_df=0.0;
    aerodynamic_force_factors_mach(t->mach,t->angle_of_attack,v,&current_lf,&current_df);
    aerodynamic_force_factors_mach(t->mach,target_aoa,v,&target_lf,&target_df);
    if(!(fabs(current_lf)>DBL_EPSILON)||!(current_df>DBL_EPSILON))
        return plan;

    double measured_lift=live_lift_accel(t,aero,v);
    double measured_drag=live_drag_accel(t,aero,v);
    if(!(measured_lift>0.0)||!(measured_drag>=0.0))
        return plan;

    double modeled_up=recovery.vertical_recovery_accel_mps2;
    if(!(modeled_up>0.0)||!isfinite(modeled_up))
        return plan;

    /*
     * Pitch response uses the same state/rate/authority reachability model as
     * the native low-level controller. No terminal-only delay, fixed AoA-rate
     * fallback, or phase-specific rescue timing participates in feasibility.
     */
    double response_time=recovery.control_response_time_s;
    if(!isfinite(response_time))return plan;
    VerticalRecoveryEnvelope vertical=decision_vertical_recovery_envelope(
        t->radar_altitude-s->flare_altitude,sink,target_sink,response_time,
        recovery.response_down_accel_mps2,modeled_up);
    if(!vertical.valid)return plan;
    plan.height=vertical.height;
    double required_height=vertical.required_height_m;
    double trigger=s->flare_altitude+required_height;

    double pull_time=vertical.recovery_time_s;
    double target_drag=measured_drag*(target_df/current_df);
    double air_path=fmax(0.0,t->true_air_speed*pull_time);
    double predicted_v2=t->true_air_speed*t->true_air_speed+
        2.0*gravity*required_height-2.0*target_drag*air_path;

    /*
     * Preflare speed constraints belong to the certified vehicle envelope and
     * measured stall boundary. Preview/planning hints must not raise the live
     * execution contract; doing so creates a circular dependency where a tuned
     * nominal gate becomes evidence for its own feasibility.
     */
    double minimum_speed=v->minimum_safe_speed;
    if(isfinite(t->calibrated_stall_speed)&&t->calibrated_stall_speed>0.0)
        minimum_speed=fmax(minimum_speed,t->calibrated_stall_speed);

    double reference_speed=fmax(v->final_approach_speed,minimum_speed);

    double margin_v2=predicted_v2-minimum_speed*minimum_speed;
    if(!isfinite(trigger)||!(predicted_v2>=minimum_speed*minimum_speed)||
       t->true_air_speed<minimum_speed||
       t->angle_of_attack>v->maximum_angle_of_attack||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0))
        return plan;

    plan.feasible=true;
    plan.trigger_altitude=trigger;
    plan.target_aoa=target_aoa;
    plan.target_sink=-target_sink;
    plan.minimum_speed=minimum_speed;
    plan.reference_speed=reference_speed;
    plan.predicted_height_loss=required_height;
    plan.predicted_kinetic_margin=.5*margin_v2;
    plan.effective_accel=modeled_up;
    plan.response_time=response_time;
    return plan;
}

static void terminal_store_preflare_plan(GuidanceMachine*g,const TerminalPreflarePlan*p){
    g->terminal_preflare_plan_valid=p&&p->feasible;
    if(!p||!p->feasible)return;
    g->preflare_trigger_altitude=p->trigger_altitude;
    g->preflare_target_aoa=p->target_aoa;
    g->preflare_target_sink=p->target_sink;
    g->preflare_minimum_speed=p->minimum_speed;
    g->preflare_reference_speed=p->reference_speed;
    g->preflare_predicted_height_loss=p->predicted_height_loss;
    g->preflare_predicted_kinetic_margin=p->predicted_kinetic_margin;
    g->preflare_effective_accel=p->effective_accel;
}

static void terminal_set_stage(GuidanceMachine*g,TerminalVerticalStage stage,double ut){
    if(g->terminal_vertical_stage_valid&&g->terminal_vertical_stage==stage)return;
    /* Terminal vertical progression is one-way; once pull-up has begun it can
       never fall back to the steep outer-glide controller. */
    if(g->terminal_vertical_stage_valid&&stage<g->terminal_vertical_stage)return;
    g->terminal_vertical_stage=stage;
    g->terminal_vertical_stage_valid=true;
    g->terminal_stage_started_ut=ut;
    g->terminal_stage_good_duration=0;
    g->flare_sink_captured=false;
}

static bool terminal_outer_gate(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        TerminalPreflarePlan*out_plan){
    const GuidanceSettings*s=&cfg->guidance;
    double slope=terminal_outer_glide_slope(g,s);
    double distance=fmax(0.0,-t->runway_along_track);
    TerminalPreflarePlan plan=terminal_preflare_plan(g,t,p,aero,cfg);
    if(out_plan)*out_plan=plan;
    if(!plan.feasible||!plan.height.valid||plan.height.margin<0.0)return false;

    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return false;

    double tangent=tan(slope*DEG2RAD);
    if(!(tangent>0.0)||!(t->horizontal_speed>0.0))return false;
    double preflare_ground=plan.trigger_altitude/tangent;
    double ground_to_preflare=distance-preflare_ground;
    if(!(ground_to_preflare>0.0))return false;

    double time_to_preflare=ground_to_preflare/t->horizontal_speed;
    double lateral_time=fmax(envelope.lateral_capture_time_s,
        envelope.heading_capture_time_s);
    DecisionMargin capture_time=decision_margin(time_to_preflare,lateral_time);

    return t->runway_along_track<0.0&&
        envelope.runway_remaining.margin>=0.0&&
        envelope.vertical_recovery.margin>=0.0&&
        envelope.speed.margin>=0.0&&
        envelope.dynamic_pressure.margin>=0.0&&
        envelope.load.margin>=0.0&&
        capture_time.valid&&capture_time.margin>=0.0&&
        plan.predicted_kinetic_margin>=0.0;
}


/* Every terminal path provider uses the same final delivery envelope. */
static bool terminal_outer_capture_admissible(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        TerminalPreflarePlan*out_plan){
    /*
     * There is no second, wider magic-number envelope.  Spline and HAC delivery
     * use the same physically-derived runway-capture proof.
     */
    return terminal_outer_gate(g,t,course,p,aero,cfg,out_plan);
}


/* Reduce the currently committed terminal path and measured vehicle response to
   TAEMEXEC's shape-independent delivery contract.  This is deliberately derived
   from the same preflare, q-bar and energy quantities that Final will actually
   use; no optimistic candidate-only forecast is allowed to authorize delivery. */
static TaemTerminalContract terminal_delivery_contract(const GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,const LandingConfiguration*cfg,
        const TerminalPreflarePlan*plan){
    TaemTerminalContract contract={0};
    if(!g||!t||!p||!cfg||!plan||!plan->feasible)return contract;

    const LandingSite*site=&cfg->site;
    const VehicleProfile*v=&cfg->vehicle;
    const GuidanceSettings*s=&cfg->guidance;
    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return contract;

    double slope=terminal_outer_glide_slope(g,s);
    double tangent=tan(slope*DEG2RAD);
    double distance=fmax(0.0,-t->runway_along_track);
    if(!(tangent>0.0)||!(t->horizontal_speed>0.0))return contract;

    double preflare_ground=plan->trigger_altitude/tangent;
    double ground_to_gate=distance-preflare_ground;
    if(!(ground_to_gate>0.0))return contract;

    double response_available=ground_to_gate/t->horizontal_speed;
    if(!isfinite(plan->response_time))return contract;
    double response_required=fmax(plan->response_time,
        fmax(envelope.lateral_capture_time_s,envelope.heading_capture_time_s));

    double gravity=planet_surface_gravity(p);
    if(!(gravity>0.0))return contract;
    double height_to_gate=fmax(0.0,t->radar_altitude-plan->trigger_altitude);
    double air_path=hypot(ground_to_gate,height_to_gate);

    double minimum_speed=plan->minimum_speed;
    double gate_speed=fmax(plan->reference_speed,minimum_speed);
    double live_loss=t->mass>0.0&&isfinite(t->drag_force)&&t->drag_force>=0.0?
        t->drag_force/t->mass:NAN;
    if(!isfinite(live_loss))return contract;

    double predicted_gate_v2=t->true_air_speed*t->true_air_speed+
        2.0*gravity*height_to_gate-2.0*live_loss*air_path;
    if(!isfinite(predicted_gate_v2))return contract;

    double low_energy_margin=.5*(predicted_gate_v2-minimum_speed*minimum_speed);
    double gate_energy=.5*gate_speed*gate_speed;
    double predicted_energy=.5*predicted_gate_v2;
    /*
     * High terminal energy must be dissipated by the selected unpowered path.
     * With airbrakes forbidden, any predicted excess at the preflare gate is a
     * negative feasibility margin and must cause replanning rather than an
     * actuator-dependent delivery.
     */
    double high_energy_margin=gate_energy-predicted_energy;

    double down_fpa=fmax(0.0,-t->flight_path_angle);
    double fpa_margin=envelope.recoverable_fpa_deg-down_fpa;

    double current_specific=gravity*(t->mean_altitude-site->altitude)+
        .5*t->true_air_speed*t->true_air_speed;

    if(!isfinite(response_available)||!isfinite(response_required)||
       !isfinite(low_energy_margin)||!isfinite(high_energy_margin)||
       !isfinite(current_specific)||!isfinite(fpa_margin))return contract;

    contract.valid=true;
    contract.path_committed=g->terminal_path_committed&&!g->hac_transition_active;
    contract.range_to_go=distance;
    contract.range_margin=(response_available-response_required)*t->horizontal_speed;
    contract.dynamic_pressure=t->dynamic_pressure;
    contract.dynamic_pressure_margin=v->maximum_dynamic_pressure-t->dynamic_pressure;
    contract.speedbrake_required=false;
    contract.speedbrake_available=false;
    contract.speedbrake_dynamic_pressure_margin=v->maximum_dynamic_pressure-t->dynamic_pressure;
    contract.altitude=t->mean_altitude;
    /* Delivery reserves the configured flare boundary, not merely ground
       clearance. A plan can be numerically valid yet physically unreachable. */
    contract.altitude_margin=fmin(envelope.vertical_recovery.margin,
        plan->height.valid?plan->height.margin:-INFINITY);
    contract.flight_path_angle=t->flight_path_angle;
    contract.flight_path_angle_margin=fpa_margin;
    contract.specific_energy=current_specific;
    contract.specific_energy_margin=fmin(low_energy_margin,high_energy_margin);
    contract.response_time_available=response_available;
    contract.response_time_required=response_required;
    contract.attitude_response_qualified=plan->effective_accel>0.0&&
        envelope.vertical_recovery_accel_mps2>0.0;
    return contract;
}


static bool terminal_preflare_alignment_valid(const GuidanceMachine*g,const Telemetry*t,double course,
        const LandingConfiguration*cfg){
    (void)g;
    const LandingSite*site=&cfg->site;
    const VehicleProfile*v=&cfg->vehicle;
    if(!isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(course)||!isfinite(t->horizontal_speed)||!(t->horizontal_speed>0.0))
        return false;

    /*
     * Preflare alignment uses the runway geometry itself.  At this stage there
     * is no remaining terminal turn: lateral velocity must be converging toward
     * centerline and the projected intercept must lie on the physical runway.
     */
    double error=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=t->horizontal_speed*sin(error);
    double along_rate=t->horizontal_speed*cos(error);
    if(!(along_rate>0.0))return false;

    double time_to_threshold=fmax(0.0,-t->runway_along_track)/along_rate;
    double projected_cross=t->runway_cross_track+cross_rate*time_to_threshold;
    double minimum=g&&g->terminal_preflare_plan_valid?
        g->preflare_minimum_speed:v->minimum_safe_speed;

    return t->runway_along_track<site->runway_length&&
        fabs(projected_cross)<=site->runway_width*.5&&
        t->true_air_speed>=minimum;
}


static void terminal_update_gear_latch(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s){
    if(g->gear_command_latched)return;
    if(t->gear||t->radar_altitude<=s->gear_deployment_altitude){g->gear_command_latched=true;return;}
    if(!g->terminal_preflare_plan_valid)return;
    double sink=fmax(.5,-t->vertical_speed);
    double seconds=(t->radar_altitude-g->preflare_trigger_altitude)/sink;
    if(seconds<=12.0&&g->preflare_predicted_kinetic_margin>=0)g->gear_command_latched=true;
}

static bool terminal_runway_contact_position(const Telemetry*t,const LandingConfiguration*cfg){
    const LandingSite*site=&cfg->site;
    return t->runway_along_track>=-40.0&&t->runway_along_track<site->runway_length+80.0&&
        fabs(t->runway_cross_track)<fmax(site->runway_width*.75,45.0);
}

static bool terminal_update_ground_latch(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;
    if(g->ground_contact_latched)return true;
    bool over_runway=terminal_runway_contact_position(t,cfg);
    bool landed_on_runway=!strcasecmp(t->vessel_situation,"landed")&&over_runway&&t->gear;
    bool flare_stage=g->terminal_vertical_stage_valid&&g->terminal_vertical_stage>=TERMINAL_TOUCHDOWN_FLARE;
    bool near_contact=flare_stage&&t->gear&&over_runway&&
        t->radar_altitude<.6&&t->vertical_speed>-3.0&&fabs(t->vertical_speed)<3.0&&
        t->surface_speed<fmax(190.0,v->final_approach_speed*1.65);
    if(near_contact)g->ground_contact_duration+=fmax(0,dt);
    else g->ground_contact_duration=fmax(0,g->ground_contact_duration-fmax(0,dt)*2.0);
    if(landed_on_runway||g->ground_contact_duration>=.25){
        g->ground_contact_latched=true;g->gear_command_latched=true;
        terminal_set_stage(g,TERMINAL_GROUND,t->ut);
        return true;
    }
    return false;
}

static double terminal_approach_drag_aoa(const Telemetry*t,const VehicleProfile*v,
        double measured_loss,double required_loss,double nominal_aoa){
    double current_df=1.0;
    aerodynamic_force_factors_mach(t->mach,
        fmax(1.0,hypot(t->angle_of_attack,t->sideslip)),v,NULL,&current_df);
    double upper=fmin(22.0,v->maximum_angle_of_attack-1.5);
    upper=fmax(2.0,upper);
    double best=clampd(nominal_aoa,0.0,upper),best_cost=INFINITY;
    for(double aoa=0.0;aoa<=upper+.01;aoa+=.5){
        double df=1.0;
        aerodynamic_force_factors_mach(t->mach,aoa,v,NULL,&df);
        double projected=fmax(.01,measured_loss)*fmax(.05,df)/fmax(.05,current_df);
        double normalized=fabs(projected-required_loss)/fmax(.35,required_loss);
        double trim_cost=fabs(aoa-nominal_aoa)*.012;
        double cost=normalized+trim_cost;
        if(cost<best_cost){best_cost=cost;best=aoa;}
    }
    return best;
}

static GuidanceResult final_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const LandingSite*site=&cfg->site;const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    double slope=terminal_outer_glide_slope(g,s),dist=fmax(0,-t->runway_along_track);
    double desired=site->altitude+dist*tan(slope*DEG2RAD),ae=desired-t->mean_altitude;
    double sink=-t->horizontal_speed*tan(slope*DEG2RAD),se=sink-t->vertical_speed;
    double path_limit=g->terminal_glide_mode?1.25:6.5;
    double sink_limit=g->terminal_glide_mode?1.8:2.5;
    double path=robust_pid_update(&g->final_altitude_pid,ae,dt,-path_limit,path_limit)+clampd(se*.18,-sink_limit,sink_limit);
    double preflare_alt=g->terminal_preflare_plan_valid?g->preflare_trigger_altitude:terminal_preflare_altitude(g,t,s);
    double gate_ground=preflare_alt/fmax(tan(slope*DEG2RAD),1e-3);
    double ground_to_gate=fmax(120.0,dist-gate_ground);
    double height_to_gate=fmax(0.0,t->radar_altitude-preflare_alt);
    double time_to_gate=ground_to_gate/fmax(t->horizontal_speed,1.0);

    /* Lateral final is a damped cross-track state controller.  The previous
       fixed lookahead reacted mostly to heading and could either understeer
       far out or become twitchy near the runway.  Cross-track rate gives the
       controller the missing state: it can remove lateral velocity before
       asking for zero position error. */
    double runway_error=norm_signed_deg(course-site->runway_heading)*DEG2RAD;
    double cross_rate=t->horizontal_speed*sin(runway_error);
    double omega=clampd(2.4/fmax(time_to_gate,7.0),.045,.28);
    double lat=-2.0*.92*omega*cross_rate-omega*omega*t->runway_cross_track;
    double look=clampd(t->horizontal_speed*fmax(3.0,fmin(7.0,time_to_gate*.32)),350.0,3200.0);
    double inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-24.0,24.0);
    double target_course=norm_deg(site->runway_heading+inter),ce=norm_signed_deg(target_course-course);
    /* Add only a small tangent-tracking term; the state feedback above owns
       convergence and therefore remains well damped through shrinking look. */
    lat+=t->horizontal_speed*clampd(ce*DEG2RAD,-.35,.35)*omega*.55;
    double frac=clampd(dist/fmax(s->final_approach_distance,1),0,1);
    double banklim=fmin(30,dynamic_bank_limit(t,v));banklim=fmin(banklim,8+22*sqrt(frac));
    if(t->radar_altitude<preflare_alt*1.25)banklim=fmin(banklim,6+10*clampd(t->radar_altitude/fmax(preflare_alt*1.25,1),0,1));
    double bank=clampd(atan(lat/fmax(planet_surface_gravity(p),.1))*RAD2DEG,-banklim,banklim);
    double target;
    if(g->terminal_preflare_plan_valid)target=g->preflare_reference_speed+clampd((dist-terminal_final_distance(g,s)*.45)/180.0,0,35.0);
    else if(g->terminal_glide_mode){
        double gate_speed=fmax(g->terminal_test_preflare_target_speed,g->terminal_test_preflare_min_speed);
        double gate_distance=fmax(0,g->terminal_test_preflare_altitude)/fmax(tan(slope*DEG2RAD),1e-3);
        target=gate_speed+clampd((dist-gate_distance)/160.0,0,55.0);
    }else target=v->final_approach_speed+clampd(dist/420.0,0,24.0);
    double speedE=.5*(target*target-t->true_air_speed*t->true_air_speed),altE=planet_surface_gravity(p)*ae,total=altE+speedE;
    double balance=clampd((altE-speedE)/fmax(planet_surface_gravity(p)*500,1),-4.0,4.0);
    double minimum=g->terminal_preflare_plan_valid?g->preflare_minimum_speed:
        fmax(v->final_approach_speed*1.03,v->minimum_safe_speed*1.25);
    double speed_shortfall=fmax(0,target-t->true_air_speed-3.0);
    double speed_margin=t->true_air_speed-minimum;
    double altitude_weight=clampd(speed_margin/fmax(12.0,.12*target),0.0,1.0);
    double gate_speed=g->terminal_preflare_plan_valid?g->preflare_reference_speed:
        fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.40);
    double air_path=fmax(1.0,hypot(ground_to_gate,height_to_gate));
    double energy_to_gate=planet_surface_gravity(p)*height_to_gate+
        .5*(t->true_air_speed*t->true_air_speed-gate_speed*gate_speed);
    double required_loss=clampd(energy_to_gate/fmax(air_path,1.0),.02,40.0);
    double live_loss=t->mass>1.0&&isfinite(t->drag_force)&&t->drag_force>0.0?
        t->drag_force/t->mass:0.0;
    double measured_loss=g->terminal_energy_loss_accel_ema>.05?
        fmax(live_loss*1.02,g->terminal_energy_loss_accel_ema):fmax(.05,live_loss*1.15);
    double predicted_gate_v2=t->true_air_speed*t->true_air_speed+
        2.0*planet_surface_gravity(p)*height_to_gate-2.0*measured_loss*air_path;
    double predicted_gate_speed=sqrt(fmax(0.0,predicted_gate_v2));
    double predicted_deficit=fmax(0.0,gate_speed-predicted_gate_speed);
    double predicted_excess=fmax(0.0,predicted_gate_speed-gate_speed);
    bool low_energy=total>planet_surface_gravity(p)*120||speed_shortfall>2.0||predicted_deficit>4.0;
    double effective_slope=clampd(slope+fmax(speed_shortfall*.30,predicted_deficit*.22)-
        fmin(4.0,predicted_excess*.055),8.0,35.0);
    if(path>0)path*=altitude_weight;
    if(balance>0)balance*=altitude_weight;
    if(low_energy){path=fmin(path,0.0);balance=fmin(balance,0.0);}
    terminal_speedbrake_closed(g);
    double gamma=fmax(-35.0,-effective_slope+path+balance);
    double learned=t->calibrated_best_glide_angle_of_attack>0?t->calibrated_best_glide_angle_of_attack:0;
    double glide=learned>0?learned:clampd(v->entry_angle_of_attack,8.0,20.0);
    double trim=terminal_approach_drag_aoa(t,v,measured_loss,required_loss,glide);
    trim=clampd(trim-clampd(fmax(speed_shortfall,predicted_deficit)*.18,0,8.0),0.0,22.0);
    robust_pid_reset(&g->speed_pid);
    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,false,PROFILE_APPROACH);
    aerodynamic_pitch_target(&c,t,v,clampd(gamma+trim,-12,18));
    if(t->true_air_speed<minimum)c.target_aoa=fmin(c.target_aoa,trim);
    c.target_pitch=t->flight_path_angle+c.target_aoa;
    c.gear=t->gear||g->gear_command_latched;
    char status[320],warn[256]={0};
    snprintf(status,sizeof(status),"Outer final %.1f deg%s: %.1f km, cross %+.0f m, FPA %.1f / %.1f deg, V %.0f -> %.0f/%.0f m/s, loss %.2f / %.2f m/s2, preflare %.0f m.",effective_slope,low_energy?" energy recovery":"",dist/1000,t->runway_cross_track,t->flight_path_angle,gamma,t->true_air_speed,predicted_gate_speed,gate_speed,measured_loss,required_loss,preflare_alt);
    bool sl=fabs(t->runway_cross_track)<fmax(120,dist*.08)&&fabs(ce)<12;
    bool sv=fabs(ae)<fmax(120,dist*.08)&&fabs(se)<fmax(g->terminal_glide_mode?10.0:5.0,fabs(sink)*.20);
    double speed_floor=g->terminal_glide_mode?fmax(terminal_test_speed_floor(v),g->terminal_test_preflare_min_speed):v->minimum_safe_speed;
    bool ss=t->true_air_speed>speed_floor&&fabs(t->true_air_speed-target)<(g->terminal_glide_mode?45:18);
    if(t->true_air_speed<speed_floor)snprintf(warn,sizeof(warn),"Airspeed is below the terminal aerodynamic floor.");
    else if(dist<5000&&!(sl&&sv&&ss))snprintf(warn,sizeof(warn),"Final approach is not stabilized. Preserve the outer glide and recapture the centerline.");
    else if(total>planet_surface_gravity(p)*500&&!v->allow_powered_approach)snprintf(warn,sizeof(warn),"The unpowered approach is below the preferred energy corridor.");
    GuidanceResult r=result_make(PHASE_FINAL,c,status,warn[0]?warn:NULL);trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static GuidanceResult preflare_guidance(GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;const GuidanceSettings*s=&cfg->guidance;
    double trigger=fmax(g->preflare_trigger_altitude,fmax(250.0,s->flare_altitude*4.0));
    double inner_gate=fmax(120.0,s->flare_altitude*2.0);
    double progress=clampd((trigger-t->radar_altitude)/fmax(trigger-inner_gate,1.0),0,1);
    double outer_sink=-t->horizontal_speed*tan(terminal_outer_glide_slope(g,s)*DEG2RAD);
    double final_sink=g->terminal_preflare_plan_valid?g->preflare_target_sink:-clampd(t->horizontal_speed*tan(4.5*DEG2RAD),6,14);
    double desired=outer_sink+(final_sink-outer_sink)*(progress*progress*(3-2*progress));
    double se=desired-t->vertical_speed,corr=robust_pid_update(&g->flare_sink_pid,se,dt,-3.0,4.0);
    double target_aoa=g->terminal_preflare_plan_valid?g->preflare_target_aoa:clampd(v->entry_angle_of_attack+3,4,v->maximum_angle_of_attack-2);
    double minimum=g->terminal_preflare_plan_valid?g->preflare_minimum_speed:v->minimum_safe_speed;
    if(t->true_air_speed<minimum){
        double relief=clampd((minimum-t->true_air_speed)*.55,0,8.0);
        target_aoa=fmax(4.0,target_aoa-relief);
        corr=fmin(corr,0.0);
    }
    double gamma=atan2(desired,fmax(t->horizontal_speed,1))*RAD2DEG;
    double look=clampd(t->true_air_speed*4.5,320,1500),inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-14,14);
    double target_course=norm_deg(site->runway_heading+inter),ce=norm_signed_deg(target_course-course);
    double bank=clampd(ce*.55,-8.0,8.0);
    double target=g->terminal_preflare_plan_valid?g->preflare_reference_speed:fmax(v->final_approach_speed,v->minimum_safe_speed*1.08);
    /* Preflare preserves the pull-up energy reserve using pitch/AoA only. */
    terminal_speedbrake_closed(g);
    robust_pid_reset(&g->speed_pid);
    GuidanceCommand c=atmospheric(t,target_course,bank,v,0.0,false,PROFILE_APPROACH);
    aerodynamic_pitch_target(&c,t,v,gamma+target_aoa+corr);
    c.gear=t->gear||g->gear_command_latched;
    char status[320];
    snprintf(status,sizeof(status),"Preflare: %.0f / %.0f m, sink %.1f / %.1f m/s, AoA %.1f / %.1f deg, speed %.1f / %.1f m/s.",t->radar_altitude,trigger,t->vertical_speed,desired,t->angle_of_attack,c.target_aoa,t->true_air_speed,target);
    GuidanceResult r=result_make(PHASE_FINAL,c,status,t->true_air_speed<g->preflare_minimum_speed?"Preflare speed margin is low; pitch-up is being limited by the energy floor.":NULL);
    trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static GuidanceResult inner_final_guidance(GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;const GuidanceSettings*s=&cfg->guidance;
    double top=fmax(g->preflare_trigger_altitude,s->flare_altitude*4.0),bottom=fmax(s->flare_altitude,1.0);
    double hf=clampd((t->radar_altitude-bottom)/fmax(top-bottom,1.0),0,1);
    double entry_sink=g->terminal_preflare_plan_valid?g->preflare_target_sink:-10.0;
    double flare_entry=-fmax(3.5,fabs(s->touchdown_sink_rate)*2.5);
    double desired=flare_entry+(entry_sink-flare_entry)*sqrt(hf);
    double se=desired-t->vertical_speed,corr=robust_pid_update(&g->flare_sink_pid,se,dt,-2.5,3.5);
    double look=clampd(t->true_air_speed*4.0,260,1000),inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-10,10);
    double target_course=norm_deg(site->runway_heading+inter),ce=norm_signed_deg(target_course-course);
    double roll=clampd(ce*.55,-6.0,6.0);
    double high=g->terminal_preflare_plan_valid?g->preflare_reference_speed:v->final_approach_speed;
    double low=fmax(v->minimum_safe_speed*1.03,v->touchdown_speed*1.22);
    double target=low+(fmax(low,high)-low)*sqrt(hf);
    double base=t->calibrated_best_glide_angle_of_attack>0?t->calibrated_best_glide_angle_of_attack:fmin(12.0,v->entry_angle_of_attack);
    double gamma=atan2(desired,fmax(t->horizontal_speed,1))*RAD2DEG;
    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,target_course,roll,v,0.0,false,PROFILE_APPROACH);
    aerodynamic_pitch_target(&c,t,v,gamma+clampd(base+2.0+corr,3.0,v->maximum_angle_of_attack-2.0));
    c.gear=true;
    char status[256];snprintf(status,sizeof(status),"Inner final: %.0f m, sink %.1f / %.1f m/s, speed %.1f / %.1f m/s, cross %+.0f m.",t->radar_altitude,t->vertical_speed,desired,t->true_air_speed,target,t->runway_cross_track);
    GuidanceResult r=result_make(PHASE_FINAL,c,status,t->vertical_speed<desired-5?"Inner-final sink rate remains high for the available flare height.":NULL);trajectory_copy(&r.reference,ref);return stabilized(g,r,t,v,s,dt);
}

static GuidanceResult flare_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const VehicleProfile*v=&cfg->vehicle;
    const LandingSite*site=&cfg->site;
    const GuidanceSettings*s=&cfg->guidance;
    double hf=clampd(t->radar_altitude/fmax(s->flare_altitude,1),0,1);
    double approach=-fmax(3.5,fabs(s->touchdown_sink_rate)*2.5);
    double desired=s->touchdown_sink_rate+(approach-s->touchdown_sink_rate)*sqrt(hf);
    double se=desired-t->vertical_speed;
    double corr=robust_pid_update(&g->flare_sink_pid,se,dt,-3,5);
    double aoa=t->calibrated_best_glide_angle_of_attack>0?
        t->calibrated_best_glide_angle_of_attack:5;
    double top=fmax(v->minimum_safe_speed*1.03,v->touchdown_speed*1.22);
    double touch=fmin(top,fmax(v->touchdown_speed,10));
    double blend=pow(hf,.65);
    double target=touch+(top-touch)*blend;
    double floor=touch+(fmax(v->touchdown_speed*.95,touch)-touch)*fmin(1,hf*2);
    double red=clampd((floor-t->true_air_speed)*.12,0,4);
    double flare=clampd(aoa+(1-hf)*4-red,2,14);
    double gamma=atan2(desired,fmax(t->horizontal_speed,1))*RAD2DEG;
    double look=clampd(t->true_air_speed*3.5,220,900);
    double inter=clampd(atan2(-t->runway_cross_track,look)*RAD2DEG,-10,10);
    double tc=norm_deg(site->runway_heading+inter);
    double ce=norm_signed_deg(tc-course);
    double banklim=t->radar_altitude<=15.0?2.0:4.0;
    double roll=clampd(ce*.6,-banklim,banklim);
    terminal_speedbrake_closed(g);
    GuidanceCommand c=atmospheric(t,tc,roll,v,0.0,false,PROFILE_FLARE);
    aerodynamic_pitch_target(&c,t,v,clampd(gamma+flare+corr,0,15));
    c.gear=true;
    char status[256];
    snprintf(status,sizeof(status),
        "Touchdown flare: %.1f m, sink %.1f / %.1f m/s, speed %.1f / %.1f m/s.",
        t->radar_altitude,t->vertical_speed,desired,t->true_air_speed,target);
    GuidanceResult r=result_make(PHASE_FLARE,c,status,
        t->vertical_speed<fmin(-5,desired-3)?
            "Sink rate is high for the remaining touchdown-flare height.":NULL);
    trajectory_copy(&r.reference,ref);
    return stabilized(g,r,t,v,s,dt);
}
static GuidanceResult terminal_abort(GuidanceMachine *g, const char *reason);

static GuidanceResult touchdown(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,const Trajectory*ref){
    double heading_error=norm_signed_deg(cfg->site.runway_heading-t->heading);
    double course_error=norm_signed_deg(t->ground_track_heading-cfg->site.runway_heading);
    double lateral_rate=t->surface_speed*sin(course_error*DEG2RAD);
    if(!isfinite(lateral_rate))lateral_rate=0.0;
    /* During rollout, position-only steering can keep commanding toward the
       centerline after the vehicle already has enough lateral velocity to cross it.
       Damp measured ground-track cross-rate while it is trustworthy, then fade the
       derivative term out near walking speed where course becomes noisy. */
    double rate_weight=clampd((t->surface_speed-8.0)/35.0,0.0,1.0);
    double authority=clampd(55.0/fmax(t->surface_speed,15.0),.25,1.0);
    double steer=clampd((heading_error*.03-t->runway_cross_track*.0011-
        lateral_rate*.012*rate_weight)*authority,-1.0,1.0);
    GuidanceCommand c;guidance_command_init(&c);
    c.gear=true;
    c.brakes=t->surface_speed<fmax(120.0,cfg->vehicle.touchdown_speed*1.35);
    c.airbrakes=true;
    c.wheel_steering=steer;
    c.navball_speed_mode=SPEED_SURFACE;
    c.control_profile=PROFILE_ROLLOUT;
    char status[160];GuidancePhase phase;
    if(t->surface_speed<1.5){
        if(!terminal_runway_contact_position(t,cfg))
            return terminal_abort(g,"Rollout stopped outside the selected runway envelope.");
        phase=PHASE_COMPLETE;c.wheel_steering=0;c.brakes=true;reset_limiters(g);
        snprintf(status,sizeof(status),"Landing complete.");
    }else if(g->phase!=PHASE_TOUCHDOWN&&g->phase!=PHASE_ROLLOUT){
        phase=PHASE_TOUCHDOWN;
        snprintf(status,sizeof(status),"Touchdown: %.1f m/s, sink %.1f m/s.",t->surface_speed,t->vertical_speed);
    }else{
        phase=PHASE_ROLLOUT;
        snprintf(status,sizeof(status),"Rollout: %.1f m/s, steering %+.2f, lateral %.1f m/s.",
            t->surface_speed,steer,lateral_rate);
    }
    g->phase=phase;
    GuidanceResult r=result_make(phase,c,status,NULL);trajectory_copy(&r.reference,ref);return r;
}

/* A terminal abort is latched. No unvalidated recovery orbit is synthesized. */
static GuidanceResult terminal_abort(GuidanceMachine *g, const char *reason) {
    guidance_abort(g);
    snprintf(g->abort_reason, sizeof(g->abort_reason), "%s", reason);
    GuidanceCommand c;
    guidance_command_init(&c);
    return result_make(PHASE_ABORT, c, reason, "Nominal landing terminated. Manual control restored.");
}


static double terminal_touchdown_flare_altitude(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->terminal_preflare_plan_valid)return NAN;
    const GuidanceSettings*s=&cfg->guidance;
    double sink=fmax(0.0,-t->vertical_speed);
    double touchdown=fmax(0.0,-s->touchdown_sink_rate);
    RunwayCaptureEnvelope authority=decision_runway_capture_envelope(
        g,t,cfg->site.runway_heading,p,cfg);
    if(!authority.valid)return NAN;
    VerticalRecoveryEnvelope recovery=decision_vertical_recovery_envelope(
        t->radar_altitude,sink,touchdown,authority.control_response_time_s,
        authority.response_down_accel_mps2,authority.vertical_recovery_accel_mps2);
    return recovery.valid?fmax(s->flare_altitude,recovery.required_height_m):NAN;
}

static bool terminal_airborne_corridor_valid(const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg)return false;
    TerminalVerticalStage stage=g->terminal_vertical_stage_valid?
        g->terminal_vertical_stage:TERMINAL_TRAJECTORY_CAPTURE;
    if(stage>=TERMINAL_GROUND)return true;

    /*
     * Final survivability has one source of truth.  The runway envelope derives
     * capture time from live lateral acceleration, heading rate and measured
     * control response; vertical reserve from live sink and lift authority; and
     * speed/q/load/runway margins from the vehicle and site.  Do not rebuild a
     * second set of stage-specific angle/cross-track/speed thresholds here.
     */
    RunwayCaptureEnvelope envelope=decision_runway_capture_envelope(g,t,course,p,cfg);
    if(!envelope.valid)return false;
    bool survivable=envelope.speed.margin>=0.0&&
        envelope.dynamic_pressure.margin>=0.0&&
        envelope.load.margin>=0.0&&
        envelope.vertical_recovery.margin>=0.0&&
        envelope.runway_remaining.margin>=0.0;
    if(!survivable)return false;

    if(stage<=TERMINAL_OUTER_FINAL)
        return envelope.reachable;

    /*
     * Once preflare starts there is no remaining terminal-area turn to invent.
     * Require the current velocity vector to intersect the physical runway and
     * retain the same physics-derived survivability margins.
     */
    return terminal_preflare_alignment_valid(g,t,course,cfg);
}

static GuidanceResult terminal_approach_sequence(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;const LandingSite*site=&cfg->site;
    if(!strcasecmp(t->vessel_situation,"landed")&&!terminal_runway_contact_position(t,cfg))
        return terminal_abort(g,"Touchdown occurred outside the runway 09 contact envelope.");
    if(!strcasecmp(t->vessel_situation,"landed")&&!t->gear)
        return terminal_abort(g,"Touchdown occurred without confirmed landing-gear deployment.");
    if(!g->terminal_vertical_stage_valid)terminal_set_stage(g,TERMINAL_TRAJECTORY_CAPTURE,t->ut);
    terminal_observe_response(g,t,dt);
    if(terminal_update_ground_latch(g,t,cfg,dt))return touchdown(g,t,cfg,ref);
    RunwayCaptureEnvelope live_envelope=
        decision_runway_capture_envelope(g,t,course,p,cfg);
    bool corridor_valid=terminal_airborne_corridor_valid(g,t,course,p,cfg);
    if(corridor_valid)g->final_invalid_duration=fmax(0.0,
        g->final_invalid_duration-fmax(0.0,dt));
    else g->final_invalid_duration+=fmax(0.0,dt);

    /*
     * Invalid-state hysteresis is bounded by physics, not a stage-tuned dwell:
     * allow at most one measured control-response interval, and never longer
     * than the remaining ballistic time-to-ground at the current sink rate.
     */
    double radar=isfinite(t->radar_altitude)&&t->radar_altitude>=0.0?
        t->radar_altitude:fmax(0.0,t->mean_altitude-site->altitude);
    double sink=fmax(0.0,-t->vertical_speed);
    double time_to_ground=sink>DBL_MIN?radar/sink:INFINITY;
    double corridor_grace=live_envelope.valid?
        fmin(live_envelope.control_response_time_s,time_to_ground):0.0;
    if(!(corridor_grace>0.0)||!isfinite(corridor_grace))
        corridor_grace=fmax(0.0,dt);
    if(g->final_invalid_duration>=corridor_grace)
        return terminal_abort(g,"Final approach lost: the physics-derived runway capture envelope remained infeasible for a full available response interval.");

    bool preflare_due=false;
    if(g->terminal_vertical_stage<=TERMINAL_OUTER_FINAL){
        TerminalPreflarePlan live=terminal_preflare_plan(g,t,p,aero,cfg);
        g->terminal_preflare_plan_valid=false;
        if(!live.feasible||!live.height.valid)
            return terminal_abort(g,"No feasible preflare pull-up remains during final capture.");
        if(live.height.margin<0.0)
            return terminal_abort(g,"Preflare initiation missed: remaining height cannot preserve the pull-up reserve.");
        terminal_store_preflare_plan(g,&live);
        /* Start before a control sample can consume the remaining recovery
           margin. This uses the actual integration interval, not an altitude
           window or a grace timer. Capture cannot skip the outer-final state. */
        double sample_time=fmax(0.0,dt);
        double down=live_envelope.response_down_accel_mps2;
        // decision-literal: physical-law-constant | Constant acceleration height loss over one controller sample.
        double next_height=t->radar_altitude-s->flare_altitude-
            sink*sample_time-0.5*down*sample_time*sample_time;
        VerticalRecoveryEnvelope next=decision_vertical_recovery_envelope(
            next_height,sink+down*sample_time,-live.target_sink,
            live.response_time,down,live.effective_accel);
        if(!next.valid)
            return terminal_abort(g,"Preflare response forecast is numerically invalid.");
        preflare_due=next.height.margin<=0.0;
        if(preflare_due&&(g->terminal_vertical_stage!=TERMINAL_OUTER_FINAL||
           !terminal_preflare_alignment_valid(g,t,course,cfg)))
            return terminal_abort(g,"Preflare recovery boundary reached before final alignment was delivered.");
    }
    terminal_update_gear_latch(g,t,s);

    if(g->terminal_vertical_stage==TERMINAL_TRAJECTORY_CAPTURE){
        bool good=terminal_outer_gate(g,t,course,p,aero,cfg,NULL);
        g->terminal_stage_good_duration=good?
            g->terminal_stage_good_duration+fmax(0.0,dt):0.0;
        double stable_for=live_envelope.valid?
            live_envelope.control_response_time_s:INFINITY;
        if(good&&isfinite(stable_for)&&
           g->terminal_stage_good_duration>=stable_for)
            terminal_set_stage(g,TERMINAL_OUTER_FINAL,t->ut);
        g->phase=PHASE_FINAL;
        return final_guidance(g,t,course,p,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_OUTER_FINAL){
        bool aligned=terminal_preflare_alignment_valid(g,t,course,cfg);
        if(g->terminal_preflare_plan_valid&&aligned&&preflare_due){
            terminal_set_stage(g,TERMINAL_PREFLARE,t->ut);
            robust_pid_reset(&g->flare_sink_pid);
            terminal_speedbrake_closed(g);
            g->phase=PHASE_FINAL;
            return preflare_guidance(g,t,course,cfg,ref,dt);
        }
        g->phase=PHASE_FINAL;
        return final_guidance(g,t,course,p,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_PREFLARE){
        bool stable=t->gear&&t->vertical_speed>=g->preflare_target_sink-2.0&&
            g->terminal_sink_accel_ema>=-.2&&t->true_air_speed>=g->preflare_minimum_speed&&
            t->angle_of_attack<=v->maximum_angle_of_attack-2.0&&t->stall_fraction<.15&&
            fabs(norm_signed_deg(t->roll))<=8.0&&fabs(norm_signed_deg(site->runway_heading-course))<=8.0&&
            fabs(t->runway_cross_track)<=fmax(site->runway_width,70.0);
        g->terminal_stage_good_duration=stable?g->terminal_stage_good_duration+fmax(0,dt):0;
        if(g->terminal_stage_good_duration>=.75){
            terminal_set_stage(g,TERMINAL_INNER_FINAL,t->ut);robust_pid_reset(&g->flare_sink_pid);
            g->phase=PHASE_FINAL;return inner_final_guidance(g,t,course,cfg,ref,dt);
        }
        if(t->radar_altitude<fmax(80.0,s->flare_altitude*1.2)&&t->vertical_speed<-12.0)
            return terminal_abort(g,"Preflare failed to arrest sink before the inner-final height floor.");
        g->phase=PHASE_FINAL;return preflare_guidance(g,t,course,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_INNER_FINAL){
        double flare_height=terminal_touchdown_flare_altitude(g,t,p,cfg);
        if(!isfinite(flare_height))
            return terminal_abort(g,"Touchdown flare response envelope is unavailable.");
        double flare_sink=clampd(fmax(4.0,.04*t->horizontal_speed),4.0,7.0);
        double flare_min=fmax(v->touchdown_speed*1.10,v->minimum_safe_speed*.95);
        if(t->calibrated_stall_speed>0)flare_min=fmax(flare_min,t->calibrated_stall_speed*1.15);
        bool aligned=t->runway_along_track>=-fmax(1000.0,t->horizontal_speed*10.0)&&
            t->runway_along_track<site->runway_length&&fabs(t->runway_cross_track)<fmax(site->runway_width,70.0)&&
            fabs(norm_signed_deg(site->runway_heading-course))<=8.0;
        if(t->radar_altitude<=flare_height&&t->vertical_speed>=-(flare_sink+2.0)&&t->true_air_speed>=flare_min&&aligned){
            terminal_set_stage(g,TERMINAL_TOUCHDOWN_FLARE,t->ut);robust_pid_reset(&g->flare_sink_pid);g->phase=PHASE_FLARE;
            return flare_guidance(g,t,course,cfg,ref,dt);
        }
        if(t->radar_altitude<=fmax(12.0,flare_height*.45)&&t->vertical_speed<-(flare_sink+4.0))
            return terminal_abort(g,"Inner-final sink rate is too high for the remaining flare height.");
        g->phase=PHASE_FINAL;return inner_final_guidance(g,t,course,cfg,ref,dt);
    }

    if(g->terminal_vertical_stage==TERMINAL_TOUCHDOWN_FLARE){
        bool sink_good=fabs(t->vertical_speed-s->touchdown_sink_rate)<=2.0&&t->stall_fraction<.2&&
            t->angle_of_attack<=v->maximum_angle_of_attack-1.0;
        g->terminal_stage_good_duration=sink_good?g->terminal_stage_good_duration+fmax(0,dt):0;
        if(g->terminal_stage_good_duration>=.50)g->flare_sink_captured=true;
        g->phase=PHASE_FLARE;return flare_guidance(g,t,course,cfg,ref,dt);
    }

    return touchdown(g,t,cfg,ref);
}

static double observed_signed_roll_rate(const Telemetry *t) {
    /* Use one signed rate source for both departure magnitude and braking
       direction.  The September 9 false recoveries chose the large Euler-rate
       magnitude but the opposite-sign body-rate for the braking test, so a
       saturated aileron that was actually arresting the observed motion was
       mislabeled as divergent.  Near knife-edge Euler roll is geometrically
       contaminated; fade it out and select whichever signed signal still has
       the larger trustworthy magnitude. */
    double euler=t->roll_rate;
    if(!t->has_body_roll_rate||!isfinite(t->body_roll_rate))return euler;
    double c=cos(norm_signed_deg(t->roll)*DEG2RAD),euler_weight=c*c;
    double weighted_euler=euler*euler_weight;
    return fabs(t->body_roll_rate)>=fabs(weighted_euler)?t->body_roll_rate:weighted_euler;
}

static bool control_recovery_needed(GuidanceMachine *g, const Telemetry *t,
        const GuidanceSettings *s, const VehicleProfile *v, double dt) {
    bool hac_control_tuning=g->terminal_region_entered&&
        (g->phase==PHASE_TAEM||g->hac_side_selected)&&
        (!g->terminal_glide_mode||!g->terminal_test_capture_active||g->phase==PHASE_TAEM);
    double signed_roll_rate=observed_signed_roll_rate(t);
    double roll_rate=fabs(signed_roll_rate);
    double pitch_rate = fmax(fabs(t->pitch_rate),
        t->has_body_pitch_rate ? fabs(t->body_pitch_rate) : 0);
    double signed_pitch_rate = t->has_body_pitch_rate && isfinite(t->body_pitch_rate) ?
        t->body_pitch_rate : t->pitch_rate;
    double body_yaw_rate = t->has_body_yaw_rate ? fabs(t->body_yaw_rate) : 0;
    double sideslip = fabs(t->sideslip);
    double beta_rate = 0;
    if(g->has_previous_sideslip&&dt>1e-4)
        beta_rate=norm_signed_deg(t->sideslip-g->previous_sideslip)/dt;
    g->previous_sideslip=t->sideslip;
    g->has_previous_sideslip=true;
    bool roll_saturated = t->has_controls && fabs(t->control_roll) >= .35;
    bool pitch_saturated = t->has_controls && fabs(t->control_pitch) >= .35;
    bool yaw_saturated = t->has_controls && fabs(t->control_yaw) >= .20;
    bool saturated = roll_saturated || pitch_saturated || (!hac_control_tuning&&yaw_saturated);
    double nominal_roll_rate = s->entry_roll_rate;
    if (g->phase == PHASE_TAEM || g->phase == PHASE_HEADING_ALIGNMENT)
        nominal_roll_rate = s->taem_roll_rate;
    else if (g->phase == PHASE_FINAL || g->phase == PHASE_FLARE)
        nominal_roll_rate = s->approach_roll_rate;
    double bad_roll_rate = fmax(12, nominal_roll_rate * 3);
    double extreme_roll_rate = fmax(24, nominal_roll_rate * 6);
    double previous_roll_target = g->roll_limiter.has_value ?
        norm_signed_deg(g->roll_limiter.value) : norm_signed_deg(t->roll);
    double roll_error = norm_signed_deg(previous_roll_target - norm_signed_deg(t->roll));
    double reference_roll_rate = g->roll_limiter.has_value && isfinite(g->roll_limiter.rate) ?
        g->roll_limiter.rate : 0;
    double relative_roll_rate = signed_roll_rate - reference_roll_rate;

    /* A sampled-data limit cycle looks deceptively healthy to an
       instantaneous braking test: each aileron command is usually opposite
       the current body rate because it is undoing the previous overshoot.
       Track relative-rate energy and repeated sign reversals directly.  One
       deliberate S-turn reversal is allowed; a second energetic crossing, or
       sustained rate far outside the guidance envelope, is a control-system
       instability even if every individual command has the right sign. */
    double energetic_crossing_rate=fmax(8.0,nominal_roll_rate*1.30);
    double sustained_excess_rate=fmax(12.0,nominal_roll_rate*2.0);
    bool commanded_roll_motion = fabs(roll_error) > 1.5 &&
        roll_error * signed_roll_rate > 0;
    bool roll_rate_braking = t->has_controls && fabs(t->control_roll) > .03 &&
        t->control_roll * signed_roll_rate < 0;
    bool energetic_crossing=g->has_previous_relative_roll_rate&&
        g->previous_relative_roll_rate*relative_roll_rate<0&&
        fabs(g->previous_relative_roll_rate)>energetic_crossing_rate&&
        fabs(relative_roll_rate)>energetic_crossing_rate;
    g->roll_oscillation_score=fmax(0,g->roll_oscillation_score-dt*.35);
    if(energetic_crossing)g->roll_oscillation_score+=1.0;
    /* A large commanded bank is an intentional transient, not a sampled-data
       limit cycle.  The first exact postburn replay reached the simulator's
       modeled 12.45 deg/s roll response while still 50 deg away from the
       requested bank; counting that capture as sustained excess rate forced
       attitude recovery in near-vacuum before aerodynamic control existed.
       Keep the excess-rate timer for wrong-way motion or motion that has
       already crossed the reference; the independent emergency-rate gate
       still protects gross departures. */
    bool excess_roll_motion=fabs(relative_roll_rate)>sustained_excess_rate&&
        !commanded_roll_motion&&!roll_rate_braking;
    if(excess_roll_motion)
        g->roll_rate_excess_duration+=dt;
    else
        g->roll_rate_excess_duration=fmax(0,g->roll_rate_excess_duration-dt*1.5);
    g->previous_relative_roll_rate=relative_roll_rate;
    g->has_previous_relative_roll_rate=true;
    bool oscillatory_roll=g->roll_oscillation_score>=1.8||g->roll_rate_excess_duration>=1.25;
    /* Near capture the bank error can already be small while the airframe
       still carries substantial roll inertia. If the live aileron command is
       opposing that body rate, the controller is doing exactly the right
       thing: braking through the setpoint. Do not relabel that transient as a
       divergent roll merely because the angle error has fallen below 1.5 deg. */
    /* Atmospheric control is intentionally direct aircraft control. Large
       bank-angle error and brief saturation are normal during an S-turn
       reversal, so departure detection must key off actual angular motion,
       not generic AutoPilot attitude error. */
    /* Use measured aerodynamic force rather than a q threshold to decide
       whether the atmosphere owns the motion. This removes the old dead zone
       where the shuttle could already be aerodynamically diverging below the
       fixed 120 Pa gate. */
    double aero_accel=0;
    if(t->mass>1&&isfinite(t->lift_force)&&isfinite(t->drag_force))
        aero_accel=hypot(t->lift_force,t->drag_force)/t->mass;
    bool aero_loaded=aero_accel>.02;
    /* Heading/course rate is expected in a coordinated banked turn and is
       not a body-yaw departure signal. kRPC 0.5.4 can report an unusable
       constant-zero body yaw stream on this vessel, so only use heading rate
       as a fallback when substantial sideslip proves the turn is
       uncoordinated. */
    double bank_excess=fabs(norm_signed_deg(t->roll))-v->maximum_bank_angle;
    bool bank_departure=bank_excess>fmax(15.0,v->maximum_bank_angle*.22);
    double beta_growth=t->sideslip*beta_rate;
    double beta_growth_fraction=beta_growth/(t->sideslip*t->sideslip+25.0);
    /* A saturated rudder command can already be doing the right thing while
       the airframe still carries yaw inertia in the old direction. Attempt 13
       hit recovery at beta=-11 deg with yaw=-0.79 but body yaw still +3.2
       deg/s: beta was worsening for only the braking transient, and recovery
       interrupted the controller that was actively arresting it. Mirror the
       roll/pitch braking exemptions here. For kRPC's beta convention
       (approximately course-heading), corrective yaw has the same sign as
       beta; if that command opposes the measured body-yaw rate, let the
       ordinary beta loop finish the reversal unless a separate gross-rate
       departure gate fires. */
    double signed_body_yaw_rate=t->has_body_yaw_rate&&isfinite(t->body_yaw_rate)?
        t->body_yaw_rate:t->heading_rate;
    bool corrective_beta_yaw=t->has_controls&&fabs(t->control_yaw)>.08&&
        t->sideslip*t->control_yaw>0;
    bool yaw_rate_braking=corrective_beta_yaw&&
        t->control_yaw*signed_body_yaw_rate<0;
    bool beta_runaway=sideslip>10&&beta_growth_fraction>.12&&
        (saturated||bank_departure||body_yaw_rate>6)&&!yaw_rate_braking;
    bool uncoordinated_yaw=!hac_control_tuning&&(body_yaw_rate>20||
        (sideslip>20&&fabs(t->heading_rate)>12));
    bool extreme_yaw=!hac_control_tuning&&(body_yaw_rate>35||
        (sideslip>30&&fabs(t->heading_rate)>20));
    bool yaw_saturated_departure=!hac_control_tuning&&yaw_saturated&&
        (body_yaw_rate>10||(sideslip>12&&fabs(t->heading_rate)>8))&&!yaw_rate_braking;
    /* Beta magnitude by itself is not a control departure. A banked shuttle
       can carry a transient 10-20 deg sideslip while the rudder/RCS loop is
       actively damping it, especially near the first S-turn reversal. The
       old absolute |beta|>10 deg gate therefore latched recovery on a stable
       vehicle as soon as q crossed 120 Pa. Treat sideslip as a departure only
       when it is accompanied by actual yaw motion or saturated yaw authority;
       the normal entry law already unloads bank continuously as beta grows. */
    bool emergency_rate = roll_rate > 75 || pitch_rate > 45 ||
        (aero_loaded&&extreme_yaw)||(bank_departure&&sideslip>25);
    /* A roll rate above the nominal comfort envelope is not a departure when
       the live aileron command is already opposing that rate.  S-turn capture
       can briefly carry substantial angular momentum through the bank
       setpoint; the old extreme-rate branch ignored the braking exemption
       used by divergent_roll below and switched to recovery exactly while the
       controller was arresting the motion. Keep the unconditional 75 deg/s
       emergency gate, but let correctly-signed braking own the intermediate
       rate band. */
    bool extreme_roll_motion=roll_rate > extreme_roll_rate && !roll_rate_braking;
    bool extreme_rate = emergency_rate || (aero_loaded &&
        (extreme_roll_motion || pitch_rate > 35 || extreme_yaw));
    bool divergent_roll = !commanded_roll_motion && !roll_rate_braking &&
        (roll_rate > bad_roll_rate ||
         (roll_saturated && roll_rate > nominal_roll_rate * 2));
    double target_aoa = g->pitch_limiter.has_value ?
        clampd(g->pitch_limiter.value,0,v->maximum_angle_of_attack) : t->angle_of_attack;
    double aoa_error = target_aoa - t->angle_of_attack;
    /* Saturation is not a departure by itself when the elevator is saturated
       in the direction needed to recover a large incidence error.  The HAC
       test can begin with the shuttle below its requested AoA and still carry
       several deg/s of nose-down inertia; treating that corrective pulse as a
       failure switches to recovery before the elevator has time to arrest it.
       Keep the unconditional high-rate gate, and keep the lower saturated-rate
       gate whenever the actuator is neutral, wrong-signed, or near capture. */
    bool corrective_pitch_saturation = pitch_saturated && fabs(aoa_error) > 3 &&
        t->has_controls && aoa_error * t->control_pitch > 0;
    /* Near the AoA setpoint the terminal pitch loop can legitimately hit a
       large opposite elevator command to arrest accumulated body pitch rate.
       The failed HAC run entered recovery with only ~1-3 deg AoA error while
       +0.8 elevator was braking a -12 deg/s nose-down rate.  Treat that like
       the equivalent roll-rate braking case: the hard 18 deg/s rate gate
       remains, but a saturated actuator that is demonstrably removing rate is
       not itself evidence of a departure. */
    bool pitch_rate_braking = t->has_controls && fabs(t->control_pitch) > .08 &&
        t->control_pitch * signed_pitch_rate < 0;
    bool pitch_departure = pitch_rate > 18 ||
        (pitch_saturated && pitch_rate > 10 && !corrective_pitch_saturation && !pitch_rate_braking);
    bool bad = beta_runaway || (aero_loaded && (oscillatory_roll || extreme_rate || divergent_roll ||
        pitch_departure || uncoordinated_yaw || yaw_saturated_departure));
    g->control_bad_duration = bad ? g->control_bad_duration + dt :
        fmax(0, g->control_bad_duration - dt * 1.5);
    bool recovery_trigger=!g->attitude_recovery&&
        (extreme_rate||oscillatory_roll||g->control_bad_duration>=.35);
    if(recovery_trigger){
        const char*diag_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        if(diag_env&&strcmp(diag_env,"1")==0)
            fprintf(stderr,
                "CONTROL RECOVERY trigger: UT %.2f phase %d alt %.0f q %.3f V %.1f roll %.2f rollRate %.2f bodyRoll %.2f pitchRate %.2f bodyPitch %.2f beta %.2f betaRate %.2f bodyYaw %.2f headingRate %.2f controls %.3f/%.3f/%.3f aeroAccel %.4f loaded=%d badDur %.2f relRate %.2f refRate %.2f rollErr %.2f aoa %.2f targetAoA %.2f flags bad=%d beta=%d oscillatory=%d extreme=%d divergent=%d pitch=%d yaw=%d yawSat=%d emergency=%d braking=%d rollSat=%d pitchSat=%d yawSatCmd=%d\n",
                t->ut,g->phase,t->mean_altitude,t->dynamic_pressure,t->true_air_speed,
                t->roll,signed_roll_rate,t->has_body_roll_rate?t->body_roll_rate:NAN,
                pitch_rate,t->has_body_pitch_rate?t->body_pitch_rate:NAN,
                t->sideslip,beta_rate,body_yaw_rate,t->heading_rate,
                t->has_controls?t->control_roll:NAN,t->has_controls?t->control_pitch:NAN,
                t->has_controls?t->control_yaw:NAN,aero_accel,aero_loaded,
                g->control_bad_duration,relative_roll_rate,reference_roll_rate,roll_error,
                t->angle_of_attack,target_aoa,bad,beta_runaway,oscillatory_roll,
                extreme_rate,divergent_roll,pitch_departure,uncoordinated_yaw,
                yaw_saturated_departure,emergency_rate,roll_rate_braking,roll_saturated,
                pitch_saturated,yaw_saturated);
        g->attitude_recovery = true;
        g->recovery_duration = g->control_good_duration = 0;
        /* Recovery is an aerodynamic attitude-unload maneuver, not a hold of
           whatever heading happened to exist when the upset was detected.
           Preserve the prograde surface direction so that, once roll/yaw
           rates are quiet again, the bridge can reacquire the velocity
           direction instead of stabilizing a nose-backwards attitude. */
        g->recovery_heading = norm_deg(t->ground_track_heading);
        /* Terminal TAEM may intentionally be flying at very low incidence to
           trade altitude for airspeed.  Recovery must stabilize the airframe,
           not command a discontinuous return to the 18 deg entry trim.  That
           entry-AoA jump caused the 50 km replay to spend the full recovery
           timeout pitching toward a state the terminal energy law explicitly
           did not want. Freeze a modest unloaded incidence for terminal
           recovery; normal entry recovery retains the entry trim. */
        if(g->terminal_region_entered&&g->terminal_glide_mode)
            g->recovery_aoa=clampd(fmax(4.0,t->angle_of_attack+1.5),4.0,10.0);
        else
            g->recovery_aoa=clampd(v->entry_angle_of_attack,10.0,18.0);
        reset_limiters(g);
        g->attitude_recovery = true;
    }
    if (!g->attitude_recovery) return false;
    g->recovery_duration += dt;
    double recovery_aoa=g->recovery_aoa>0?g->recovery_aoa:
        clampd(v->entry_angle_of_attack,10,18);
    /* Low angular rate can mean the shuttle has settled at the wrong pitch.
       In the failed flight this released recovery more than 10 degrees below
       its pitch target and immediately re-entered the departure state. */
    /* Recovery is an aerodynamic maneuver.  Surface Euler pitch becomes a
       poor state coordinate once bank/yaw depart, so release on the same AoA
       state the direct controller is actually stabilizing. */
    bool pitch_captured=fabs(t->angle_of_attack-recovery_aoa)<4.0&&t->angle_of_attack>=0;
    /* Recovery release is about whether the airframe state has converged,
       not whether every actuator is already near neutral.  The 50 km replay
       on 2026-09-09 reached |roll| < 1 deg, |body roll rate| < 1 deg/s and
       sub-degree pitch error while the elevator still used a short braking
       pulse above the generic saturation threshold.  Treating that corrective
       pulse as instability kept control_good_duration pinned at zero until
       the 15 s recovery timeout aborted an otherwise recovered vehicle. */
    bool destabilizing_roll_saturation = roll_saturated && !roll_rate_braking;
    bool destabilizing_pitch_saturation = pitch_saturated &&
        !corrective_pitch_saturation && !pitch_rate_braking;
    bool destabilizing_yaw_saturation = !hac_control_tuning && yaw_saturated &&
        (sideslip > 8 || body_yaw_rate > 5);
    bool stable = pitch_captured&&roll_rate < 5 && pitch_rate < 5 && fabs(norm_signed_deg(t->roll)) < 10 &&
        (!aero_loaded || (sideslip < 8 && !destabilizing_roll_saturation &&
            !destabilizing_pitch_saturation && !destabilizing_yaw_saturation));
    g->control_good_duration = stable ? g->control_good_duration + dt : 0;
    if (g->control_good_duration >= 2) {
        g->attitude_recovery = false;
        g->control_bad_duration = 0;
        g->roll_oscillation_score=0;
        g->roll_rate_excess_duration=0;
        g->has_previous_relative_roll_rate=false;
        g->recovery_aoa=0;
        return false;
    }
    return true;
}

static bool __attribute__((unused)) approach_valid(const Telemetry *t, double course, const LandingConfiguration *cfg) {
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    return terminal_approach_valid(current,t->range_to_site,t->runway_along_track,t->runway_cross_track,course,t->true_air_speed,t->horizontal_speed,t->vertical_speed,t->flight_path_angle,&cfg->site,&cfg->vehicle,&cfg->guidance);
}

static bool __attribute__((unused)) terminal_test_outer_approach_valid(const GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg){
    const LandingSite*site=&cfg->site;const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    double slope=terminal_outer_glide_slope(g,s),distance=fmax(0,-t->runway_along_track);
    double final_distance=g->terminal_test_final_approach_distance>0?g->terminal_test_final_approach_distance:s->final_approach_distance;
    double desired=site->altitude+distance*tan(slope*DEG2RAD);
    double sink=-t->horizontal_speed*tan(slope*DEG2RAD);
    double altitude_tolerance=fmax(180.0,distance*.08),sink_tolerance=fmax(15.0,fabs(sink)*.35);
    double max_speed=fmax(v->final_approach_speed*2.2,v->minimum_safe_speed*2.6);
    double gate_floor=fmax(terminal_test_speed_floor(v),g->terminal_test_preflare_min_speed);
    return isfinite(t->range_to_site)&&t->range_to_site<final_distance*1.25&&
        t->runway_along_track>=-final_distance*1.18&&t->runway_along_track<site->runway_length&&
        fabs(t->runway_cross_track)<fmax(site->runway_width*.5,fmin(350,distance*.06))&&
        fabs(norm_signed_deg(site->runway_heading-course))<12&&
        t->true_air_speed>=gate_floor&&t->true_air_speed<=max_speed&&
        fabs(t->mean_altitude-desired)<altitude_tolerance&&
        t->flight_path_angle>-(slope+8)&&t->flight_path_angle<-fmax(4.0,slope-8.0)&&
        fabs(t->vertical_speed-sink)<sink_tolerance;
}

static bool __attribute__((unused)) terminal_test_preflare_geometry(const GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg){
    const LandingSite*site=&cfg->site;const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    double slope=terminal_outer_glide_slope(g,s),preflare=terminal_preflare_altitude(g,t,s);
    double max_sink=fmax(90.0,t->horizontal_speed*tan((slope+8.0)*DEG2RAD));
    return t->radar_altitude<=preflare*1.15&&
        t->runway_along_track>=-fmax(2500.0,t->horizontal_speed*15.0)&&t->runway_along_track<site->runway_length&&
        fabs(t->runway_cross_track)<fmax(100.0,site->runway_width*1.2)&&
        fabs(norm_signed_deg(site->runway_heading-course))<12&&
        t->flight_path_angle>-(slope+8)&&t->flight_path_angle<-2&&
        t->vertical_speed>-max_sink&&t->vertical_speed<2&&
        t->true_air_speed>=fmax(terminal_test_speed_floor(v),g->terminal_test_preflare_min_speed)&&
        t->true_air_speed<=fmax(v->final_approach_speed*2.0,v->minimum_safe_speed*2.2);
}

static bool terminal_entry_recovery_available(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const GuidanceSettings*s){
    /* TAEM acquisition is a one-way ownership boundary.  Before terminal
       acquisition this predicate can still describe whether Entry has enough
       altitude/energy to recover.  After terminal_region_entered is latched,
       no planning, capture or attitude-recovery failure is allowed to send
       the state machine back to S-turn; terminal guidance must replan in place
       or abort if the remaining envelope is unrecoverable. */
    if(!g||g->terminal_region_entered||g->terminal_rehearsal_mode||g->final_approach_captured||
       t->true_air_speed<v->minimum_safe_speed*2.0)return false;
    double response=fmax(4.0,hac_response_lead_time(t,s,0.0));
    double recovery_height=fmax(fmax(g->terminal_test_preflare_altitude,s->flare_altitude*8.0)*3.0,
        fmax(0.0,-t->vertical_speed)*(response+4.0));
    double runway_margin=-s->final_approach_distance-t->runway_along_track;
    double recovery_distance=t->horizontal_speed*response*1.5;
    return t->radar_altitude>recovery_height&&runway_margin>recovery_distance;
}

static void terminal_invalidate_frozen_path(GuidanceMachine*g){
    /* Recovery or a deliberate return to Entry moves the aircraft away from
       the state used to freeze the join.  Never resume that stale geometry.
       Keep terminal mode itself intact here so a low-altitude recovery can
       immediately search a fresh path if there is no safe S-turn margin. */
    g->terminal_candidate.valid=false;g->terminal_path_kind=TERMINAL_PATH_NONE;g->terminal_prediction_valid=false;g->terminal_mix=0;
    g->terminal_prediction_ut=-INFINITY;g->terminal_path_committed=false;
    g->hac_plan_degraded=false;g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;g->hac_plan_violation_score=0;g->hac_commit_blend=0;
    g->hac_side_selected=false;g->hac_captured=false;g->hac_completed=false;g->hac_progress_valid=false;
    g->hac_transition_active=false;g->hac_transition_heading_cone=false;g->hac_transition_progress=0;g->hac_transition_lead_progress=0;
    g->hac_transition_cone_arc_length=0.0;
    g->hac_remaining=0;g->hac_radius=0;g->hac_capture_lost_duration=0;
    g->terminal_energy_mismatch_duration=0;
    g->hac_circuit_count=0;g->hac_circuit_slope=0;g->terminal_test_spiral_active=false;
    g->terminal_test_capture_active=g->terminal_glide_mode;
    g->terminal_energy_sample_valid=false;g->terminal_energy_loss_accel_ema=0;g->terminal_speed_loss_accel_ema=0;
    g->terminal_preflare_plan_valid=false;g->final_invalid_duration=0;
}

static void terminal_invalidate_frozen_path_preserve_energy(GuidanceMachine*g){
    /* Geometry may become stale while the aerodynamic observation that proved
       it stale is still exactly the information the replacement selector
       needs. Preserve the filtered loss estimates, but intentionally restart
       the finite-difference sample so the next observation cannot span the
       geometry/state transition. */
    double energy_loss=g->terminal_energy_loss_accel_ema;
    double speed_loss=g->terminal_speed_loss_accel_ema;
    terminal_invalidate_frozen_path(g);
    g->terminal_energy_loss_accel_ema=energy_loss;
    g->terminal_speed_loss_accel_ema=speed_loss;
    g->terminal_energy_sample_valid=false;
}

static GuidanceResult terminal_return_to_entry(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    if(taem_exec_owns_vehicle(&g->taem_exec)){
        /* Unified MM305 ownership is one-way.  A circle/spline/HAC-like path may
           become stale, but that invalidates geometry only; it never hands the
           aircraft back to MM304.  Preserve measured energy, drop the frozen path,
           let TAEMEXEC choose S-turn vs path acquisition, and keep a bounded
           terminal command while the path provider replans. */
        terminal_invalidate_frozen_path_preserve_energy(g);
        g->terminal_region_entered=true;g->terminal_glide_mode=true;
        g->attitude_recovery=false;g->control_bad_duration=0;g->control_good_duration=0;g->recovery_duration=0;
        g->airbrakes_deployed=false;
        robust_pid_reset(&g->taem_altitude_pid);robust_pid_reset(&g->speed_pid);
        taem_exec_sync(g,t,s);
        g->phase=PHASE_TAEM;
        if(g->taem_exec.phase==TAEM_PHASE_S_TURN)
            return taem_s_turn_guidance(g,t,course,p,aero,cfg,dt);
        terminal_predict(g,t,course,p,aero,cfg,dt);
        GuidanceCommand c=atmospheric(t,t->ground_track_heading,0.0,v,0.0,false,PROFILE_TAEM);
        c.heading_control_enabled=false;c.has_target_aoa=true;
        c.target_aoa=clampd(v->entry_angle_of_attack,10.0,fmin(18.0,v->maximum_angle_of_attack));
        c.target_pitch=t->flight_path_angle+c.target_aoa;
        return stabilized(g,result_make(PHASE_TAEM,c,
            "TAEM terminal path invalidated; replanning energy and runway geometry without releasing MM305 ownership.",
            "The previous terminal path was discarded, but TAEM remains the sole terminal-area guidance owner."),
            t,v,s,dt);
    }

    /* Before the one-way MM304->MM305 latch, a discarded advisory preview may
       still return to bounded Entry planning. */
    terminal_invalidate_frozen_path(g);
    g->terminal_region_entered=false;g->terminal_glide_mode=false;g->terminal_rehearsal_mode=false;
    g->terminal_test_capture_active=false;g->terminal_reentry_after_ut=0.0;
    reset_taem_handoff_state(g);
    g->attitude_recovery=false;g->control_bad_duration=0;g->control_good_duration=0;g->recovery_duration=0;
    g->entry_control_plan_valid=false;g->entry_control_terminal_ready=false;
    g->entry_control_segment_until_ut=0;g->entry_reversal_scheduled=false;
    g->entry_reversal_ut=0;g->entry_reversal_range=0;g->entry_reversal_sign=0;g->entry_reversal_bank=0;
    g->has_entry_predictive_score=false;g->entry_predictive_score=0;g->airbrakes_deployed=false;
    g->entry_reversal_is_final=false;g->entry_final_reversal_pending=false;
    g->entry_final_reversal_completed=false;g->has_s_turn_reversal_requested=false;
    g->has_s_turn_leg_started=false;
    if(fabs(norm_signed_deg(t->roll))>=4)g->s_turn_sign=t->roll>=0?1:-1;
    robust_pid_reset(&g->taem_altitude_pid);robust_pid_reset(&g->speed_pid);
    g->phase=PHASE_ENTRY_ENERGY;
    GuidanceResult r=entry_guidance(g,t,course,p,aero,cfg,dt);
    snprintf(r.status,sizeof(r.status),"Pre-TAEM recovery: discarded advisory terminal geometry; MM304 must re-qualify the interface contract before TAEM ownership.");
    r.has_warning=true;
    snprintf(r.warning,sizeof(r.warning),"Previous advisory terminal preview discarded before TAEM ownership; an explicit MM304 recovery frame is permitted.");
    return r;
}

static void terminal_force_acquisition(GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,const LandingConfiguration*cfg){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    double loss=g->terminal_energy_loss_accel_ema,speed_loss=g->terminal_speed_loss_accel_ema;
    terminal_glide_initialize(g,v,s);
    g->terminal_rehearsal_mode=false;
    g->terminal_energy_loss_accel_ema=loss;g->terminal_speed_loss_accel_ema=speed_loss;
    g->terminal_region_entered=true;
    g->terminal_test_capture_active=true;
    g->taem_safety_handoff=false;

    /* Seed terminal references with a neutral ownership-transfer command. The
       stabilized output keeps its existing limiter state, so this does not step
       the actuators; it gives both the limiter and the future-state predictor the
       same smooth roll-out target immediately after MM304 releases authority. */
    g->terminal_reference_heading=isfinite(t->ground_track_heading)?
        t->ground_track_heading:t->heading;
    g->terminal_reference_bank=0.0;
    /* Seed the post-handoff predictor on the shallow edge of the current
       state-derived terminal slope envelope.  Carrying the measured entry FPA
       into every future sample made a candidate be certified from a steep
       -16 deg origin while its selected runway path was only -8 deg; the
       planner then charged energy for a shallow path that the live aircraft
       had never been brought onto.  TAEM owns the transition now, so its
       first reference must be the same bounded vertical contract used by the
       path selector. */
    double minimum_terminal_slope=0.0,maximum_terminal_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_terminal_slope,
        &maximum_terminal_slope);
    g->terminal_reference_fpa=isfinite(minimum_terminal_slope)&&
        isfinite(maximum_terminal_slope)&&maximum_terminal_slope>=minimum_terminal_slope?
        -minimum_terminal_slope:t->flight_path_angle;
    g->terminal_reference_aoa=clampd(t->angle_of_attack,0.0,v->maximum_angle_of_attack);

    /* Preserve a genuinely fresh, regular MM304 preview across the one-way
       ownership boundary. Its geometry is exactly the coupled interface MM304
       was flying toward. Only the timing/projection cache is rebased under TAEM.
       Old/short-lead previews (v8 and v11) are still discarded. */
    bool preserve_preview=terminal_candidate_operationally_usable(
        g,&g->terminal_candidate,t,course,p,cfg);
    if(preserve_preview)g->terminal_candidate.selected_ut=t->ut;
    else g->terminal_candidate.valid=false;
    g->terminal_prediction_valid=false;
    g->terminal_prediction_ut=-INFINITY;
    g->terminal_prediction_altitude=NAN;
    g->terminal_prediction_speed=NAN;
    g->terminal_prediction_time=0.0;

    g->terminal_test_spiral_active=false;
    g->terminal_path_kind=TERMINAL_PATH_NONE;
    g->hac_progress_valid=false;
    g->hac_captured=false;
    g->hac_completed=false;
    g->hac_remaining=0;
    g->hac_radius=s->hac_radius;
    g->hac_side=fabs(norm_signed_deg(t->roll))>=4?(norm_signed_deg(t->roll)>=0?1:-1):
        terminal_default_hac_side(t,&cfg->site,course);
    g->hac_side_selected=false;
    g->hac_plan_degraded=false;
    g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;
    g->hac_plan_violation_score=0.0;
    g->hac_commit_blend=0.0;
    g->entry_control_plan_valid=false;
    g->taem_s_turn_plan.valid=false;
    g->phase=PHASE_TAEM;
    (void)taem_exec_enter(g,t,s,false);
    /* Preserve the Entry limiter state across the ownership handoff. The
       S-turn tail is already blending toward terminal incidence, so clearing
       the pitch/heading jerk state here recreates a visible command step. */
    robust_pid_reset(&g->taem_altitude_pid);robust_pid_reset(&g->speed_pid);
}


static GuidanceResult terminal_guidance(GuidanceMachine *g, const Telemetry *t, const VehicleState *state, double course,
        const PlanetModel *p, AerodynamicModel aero, const LandingConfiguration *cfg, double dt) {
    const GuidanceSettings *s = &cfg->guidance;
    const VehicleProfile *v = &cfg->vehicle;
    /* Ground rollout owns wheel control; airborne flare limits no longer apply.
       A generic KSP `landed` state is not runway success: reject a grass/terrain
       touchdown rather than allowing the rollout state to count it as complete. */
    bool reported_landed=!strcasecmp(t->vessel_situation,"landed");
    if (g->final_approach_captured &&
        (g->phase==PHASE_ROLLOUT || g->phase==PHASE_COMPLETE || reported_landed)) {
        if(reported_landed&&!terminal_runway_contact_position(t,cfg))
            return terminal_abort(g,"Touchdown occurred outside the runway 09 contact envelope.");
        if(reported_landed&&!t->gear)
            return terminal_abort(g,"Touchdown occurred without confirmed landing-gear deployment.");
        if(reported_landed){
            g->ground_contact_latched=true;g->gear_command_latched=true;
            terminal_set_stage(g,TERMINAL_GROUND,t->ut);
        }
        Trajectory ref; trajectory_init(&ref);
        GuidanceResult r=touchdown(g,t,cfg,&ref); trajectory_clear(&ref); return r;
    }
    bool recovery_was_active=g->attitude_recovery;
    if (control_recovery_needed(g, t, s, v, dt)) {
        if(!recovery_was_active&&(g->terminal_path_committed||g->hac_side_selected))
            terminal_invalidate_frozen_path(g);
        if (g->recovery_duration >= 15)
            return terminal_abort(g, "Control departure: attitude recovery exceeded 15 s without stabilizing.");
        if (t->radar_altitude < fmax(300, -t->vertical_speed * 8))
            return terminal_abort(g, "Control departure: insufficient height remains to complete attitude recovery.");
        g->phase = PHASE_ATTITUDE_RECOVERY;
        GuidanceCommand c = atmospheric(t, g->recovery_heading, 0, v, 0, false, PROFILE_RECOVERY);
        /* Recovery is an unload/level maneuver, not a heading-capture
           maneuver.  At useful dynamic pressure, rudder/RCS heading pursuit
           can couple directly into roll.  The failed 50 km replay held nearly
           full yaw while trying to preserve the pre-upset ground track and
           drove beta beyond 50 degrees.  Let roll/pitch stabilize the airframe
           first; ordinary entry guidance will reacquire course after release. */
        c.heading_control_enabled = false;
        double recovery_aoa = g->recovery_aoa>0?g->recovery_aoa:
            clampd(v->entry_angle_of_attack, 10, 18);
        c.has_target_aoa = true;
        c.target_aoa = recovery_aoa;
        c.target_pitch = clampd(t->flight_path_angle + recovery_aoa, -12, 25);
        return stabilized(g, result_make(g->phase, c, "Attitude recovery: unloading and leveling wings; terminal progression inhibited.",
            "Sustained saturation/attitude error or excessive roll rate."), t, v, s, dt);
    }
    if(recovery_was_active&&!g->attitude_recovery){
        /* A successful recovery has changed position, energy and usually bank
           enough that the old terminal join is stale.  With adequate margin,
           hand authority back to Entry/S-turn for a clean replan; otherwise
           remain in terminal mode but require a brand-new HAC candidate. */
        if(terminal_entry_recovery_available(g,t,v,s))
            return terminal_return_to_entry(g,t,course,p,aero,cfg,dt);
        terminal_invalidate_frozen_path(g);
    }
    terminal_energy_observe(g,t,p);
    terminal_predict(g,t,course,p,aero,cfg,dt);

    /* MM304 -> MM305 ownership is evaluated only on the current strict
       interface contract below.  No speed/altitude/stability fallback may
       create terminal ownership independently of that contract. */
    /* Capture-preview returns early while it is searching.  MM305 S-turn must
       therefore synchronize and execute before that branch; otherwise its
       one-time phase decision is never revisited and the preview silently
       flies terminal geometry while telemetry still says S-turn. */
    if(taem_exec_owns_vehicle(&g->taem_exec)&&g->taem_exec.phase==TAEM_PHASE_S_TURN){
        taem_exec_sync(g,t,s);
        if(g->taem_exec.phase==TAEM_PHASE_S_TURN)
            return taem_s_turn_guidance(g,t,course,p,aero,cfg,dt);
    }
    if(g->terminal_glide_mode&&g->terminal_test_capture_active){
        bool margin_exhausted=terminal_capture_margin_exhausted(g,t,course,p,aero,cfg);
        bool executable_preview=terminal_candidate_operationally_usable(
            g,&g->terminal_candidate,t,course,p,cfg)&&
            !g->terminal_candidate.geometry_degraded;
        bool planning_blocked=!terminal_candidate_operationally_usable(
            g,&g->terminal_candidate,t,course,p,cfg)||
            !terminal_candidate_vertical_response_ready_live(g,t,course,p,aero,cfg,
                &g->terminal_candidate);
        g->hac_capture_lost_duration=planning_blocked?g->hac_capture_lost_duration+dt:
            fmax(0.0,g->hac_capture_lost_duration-dt*2.0);
        double planning_grace=g->terminal_candidate.valid?
            fmax(8.0,g->terminal_candidate.response*1.5):10.0;
        /* A dynamically regular path may be tracked before its energy budget
           is nominal. Freeze it only after both geometry/control and energy
           convergence are qualified. */
        if(terminal_candidate_commit_ready(g,t,p,aero,cfg)){
            terminal_publish_candidate(g);
            g->terminal_path_committed=true;g->terminal_test_capture_active=false;
            g->terminal_energy_mismatch_duration=0.0;
            g->hac_commit_blend=0.0;
            /* Preserve all command limiter state across the commit. */
        }else{
            bool spline_candidate=g->terminal_candidate.valid&&
                g->terminal_candidate.kind==TERMINAL_PATH_SPLINE;
            if(!executable_preview&&!margin_exhausted&&
               g->hac_capture_lost_duration>=planning_grace&&
               terminal_entry_recovery_available(g,t,v,s))
                return terminal_return_to_entry(g,t,course,p,aero,cfg,dt);
            if(margin_exhausted)
                return terminal_abort(g,g->terminal_candidate.valid&&
                    !g->terminal_candidate.geometry_degraded&&g->terminal_candidate.energy_degraded?
                    "Terminal path geometry was flyable, but its energy/path budget did not converge before maneuver margin was exhausted.":
                    "Terminal path optimizer produced no finite regular geometry before maneuver margin was exhausted.");
            if(!executable_preview){
                GuidanceCommand c=atmospheric(t,g->terminal_reference_heading,g->terminal_reference_bank,v,0,false,PROFILE_TAEM);
                c.heading_control_enabled=false;c.has_target_aoa=true;c.target_aoa=g->terminal_reference_aoa;
                c.target_pitch=t->flight_path_angle+c.target_aoa;
                return stabilized(g,result_make(PHASE_TAEM,c,
                    g->terminal_candidate.valid?
                        (g->terminal_candidate.geometry_degraded?"TAEM converging on the best available terminal path.":
                         g->terminal_candidate.energy_degraded?(spline_candidate?
                            "TAEM tracking a direct spline while its energy/path budget converges.":
                            "TAEM tracking the selected HAC while its energy/path budget converges."):
                         g->terminal_candidate.shell_degraded?"TAEM converging on a qualified terminal path outside the preferred altitude shell.":
                         spline_candidate?"TAEM converging on the selected direct spline.":
                            "TAEM converging on the selected HAC join."):
                        "TAEM optimizing terminal path topology.",
                    g->terminal_candidate.geometry_degraded?
                        (planning_blocked?
                            "The current terminal candidate is still outside the bounded executable envelope; TAEM is searching for a better regular path.":
                            "A regular terminal candidate is retained while geometry converges."):
                        g->terminal_candidate.energy_degraded?
                            "The selected terminal curve is dynamically flyable, but its predicted energy/path budget is not nominal yet; commit remains inhibited while TAEM replans.":
                        NULL),t,v,s,dt);
            }
            /* A regular candidate is safe to preview even when its energy or
               vertical-response margin has not yet converged.  Fall through
               to the common path-preview law below so the vehicle can spend
               the physically modeled response lead moving onto that path.
               The commit gate still requires both margins; this branch only
               prevents a neutral tangent hold from creating cross-track debt. */
        }
    }
    if (!g->terminal_region_entered && !g->final_approach_captured) {
        g->phase = PHASE_ENTRY_ENERGY;
        GuidanceResult entry=entry_program_guidance(g,t,state,course,p,aero,cfg,dt);
        TaemInterfaceCapture strict_capture=entry_dynamic_interface_capture(g,t,course,p,cfg);
        bool strict_handoff=strict_capture.ready&&g->entry_exec.entry_complete;
        g->taem_interface_captured=strict_handoff;
        if(strict_handoff){
            guidance_result_clear(&entry);
            /* The strict fixed-point MM304 contract plus the Entry executive's
               explicit qualified-handoff event is the sole ownership transfer.
               Neither layer can manufacture MM305 ownership independently. */
            terminal_force_acquisition(g,t,course,p,cfg);
            /* If the newly latched TAEM plan begins with an energy-management
               S-turn, execute it on this same frame so ownership and commands are
               continuous across MM304 -> MM305. */
            if(taem_exec_owns_vehicle(&g->taem_exec)&&g->taem_exec.phase==TAEM_PHASE_S_TURN)
                return taem_s_turn_guidance(g,t,course,p,aero,cfg,dt);
            /*
             * Ownership has just crossed the one-way boundary, but the first
             * terminal candidate is still only a preview.  Do not fall through
             * into the default HAC law on this same frame: that law has no
             * published join and can command a large bank before the selector
             * has established an executable path.  Give the next sample a
             * neutral tangent/response frame so the candidate search and live
             * vehicle state remain on the same side of the contract.
            */
            terminal_predict(g,t,course,p,aero,cfg,dt);
            /* The response-projected candidate already carries the next
               ownership boundary. If the shared forecast, vertical, and
               energy envelopes all agree on this handoff sample, commit it
               before the forecast clock becomes stale on the next tick. */
            if(g->terminal_test_capture_active&&
               terminal_candidate_commit_ready(g,t,p,aero,cfg)){
                terminal_publish_candidate(g);
                g->terminal_path_committed=true;
                g->terminal_test_capture_active=false;
                g->terminal_energy_mismatch_duration=0.0;
                g->hac_commit_blend=0.0;
            }else{
            GuidanceCommand handoff_hold=atmospheric(t,course,0.0,v,0.0,
                false,PROFILE_TAEM);
            handoff_hold.heading_control_enabled=false;
            handoff_hold.has_target_aoa=true;
            handoff_hold.target_aoa=clampd(g->terminal_reference_aoa,0.0,
                v->maximum_angle_of_attack);
            handoff_hold.target_pitch=t->flight_path_angle+
                handoff_hold.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,handoff_hold,
                "TAEM ownership latched; holding the current tangent while terminal geometry is qualified.",
                "No executable terminal path has been published; lateral turn is inhibited."),
                t,v,s,dt);
            }
        }else{
            /* MM304 -> TAEM ownership is now exclusively the strict fixed-point
               contract: live course 330..030 or 150..210 for KSC 09, positive
               energy/speed/altitude/HAC suitability, and <=1 km horizontal
               error at the final rear alignment point.  Do not use low-speed
               or altitude safety fallbacks to enter TAEM; those produced TAEM
               phases without a real capture_ready handoff. */
            return entry;
        }
    }

    /* The runway-capture envelope is the fallback reserve for an uncommitted
       terminal state.  Once MM305 has frozen a regular HAC/spline, its route
       is the lateral capture contract; applying the point-to-runway bound
       here would reject a valid long-path turn on the very next sample simply
       because the vehicle is intentionally still perpendicular to the strip. */
    if(g->terminal_region_entered&&!g->terminal_path_committed&&
       !g->final_approach_captured&&
       terminal_capture_margin_exhausted(g,t,course,p,aero,cfg))
        return terminal_abort(g,
            "Terminal trajectory lost: the modeled runway-capture reserve is physically exhausted.");

    GuidanceSettings path_settings=terminal_path_settings(g,s);
    const GuidanceSettings*hs=&path_settings;
    double radius=g->terminal_glide_mode?fmax(2500.0,g->hac_radius):fmax(s->hac_radius,g->hac_radius);
    bool spline_path=g->terminal_path_kind==TERMINAL_PATH_SPLINE;
    HACGuidance h=spline_path?
        terminal_runway_line_path_guidance(t,&cfg->site,hs,planet_surface_gravity(p),course):
        hac_guidance_radius(t,&cfg->site,hs,p->radius,g->hac_side,planet_surface_gravity(p),course,radius);
    double nominal_bank=fmin(55.0,dynamic_bank_limit(t,v));
    g->minimum_turn_radius=live_turn_radius(t,aero,v,nominal_bank);
    bool feasible=spline_path?true:
        (isfinite(g->minimum_turn_radius)&&g->minimum_turn_radius<=radius/1.15);
    bool transition_block=g->hac_transition_active;
    if(transition_block){
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
        double e=0,n=0;local_offsets(origin,current,p->radius,&e,&n);
        HACTransitionPlan transition={.valid=true,
            .p0={g->hac_transition_p0_e,g->hac_transition_p0_n},.p1={g->hac_transition_p1_e,g->hac_transition_p1_n},
            .p2={g->hac_transition_p2_e,g->hac_transition_p2_n},.p3={g->hac_transition_p3_e,g->hac_transition_p3_n},
            .length=g->hac_transition_length,.end_angle=g->hac_transition_end_angle};
        bool lead_done=g->hac_transition_lead_length<=1.0||g->hac_transition_lead_progress>=.995;
        if(!lead_done){
            HACPoint2 lead_start={g->hac_transition_lead_start_e,g->hac_transition_lead_start_n};
            HACPoint2 lead_end=transition.p0;
            HACPoint2 lead_dir={lead_end.e-lead_start.e,lead_end.n-lead_start.n};
            double lead_len=fmax(hypot(lead_dir.e,lead_dir.n),1.0),lead_len2=lead_len*lead_len;
            double projected=((e-lead_start.e)*lead_dir.e+(n-lead_start.n)*lead_dir.n)/lead_len2;
            double nearest=clampd(projected,0.0,1.0);
            if(nearest>g->hac_transition_lead_progress)g->hac_transition_lead_progress=nearest;
            double endpoint=hypot(e-lead_end.e,n-lead_end.n);
            double cross=fabs((e-lead_end.e)*lead_dir.n-(n-lead_end.n)*lead_dir.e)/lead_len;
            bool close_endpoint=endpoint<=fmax(120.0,t->true_air_speed*.65);
            bool passed_in_corridor=projected>=.995&&cross<=fmax(350.0,radius*.12);
            if(close_endpoint||passed_in_corridor)g->hac_transition_lead_progress=1.0;
            lead_done=g->hac_transition_lead_progress>=.995;
        }
        if(lead_done){
            double nearest=hac_transition_nearest_u(g,e,n);
            /* Progress is a measured path coordinate, not a command reference.
               Rate-limiting it by aircraft heading response made the stored
               station lag behind the real closest point, so altitude scheduling
               and transition completion were based on a fictitious location.
               Keep it monotonic, but otherwise take the geometric projection
               directly. Command bandwidth remains enforced on bank/curvature. */
            if(nearest>g->hac_transition_progress)g->hac_transition_progress=nearest;
            g->hac_transition_progress=clampd(g->hac_transition_progress,0,1);
            double endpoint=hypot(e-g->hac_transition_p3_e,n-g->hac_transition_p3_n);
            HACPoint2 end_dir=hac_bezier_derivative(&transition,1.0);
            double end_heading=norm_deg(atan2(end_dir.e,end_dir.n)*RAD2DEG);
            double end_heading_error=fabs(norm_signed_deg(end_heading-course));
            double end_norm=fmax(hypot(end_dir.e,end_dir.n),1.0);
            double passed_endpoint=((e-g->hac_transition_p3_e)*end_dir.e+
                (n-g->hac_transition_p3_n)*end_dir.n)/end_norm;
            double endpoint_cross=fabs((e-g->hac_transition_p3_e)*end_dir.n-
                (n-g->hac_transition_p3_n)*end_dir.e)/end_norm;
            bool close_endpoint=endpoint<=fmax(260.0,t->true_air_speed*2.8);
            if(spline_path){
                /* A free spline terminates on the runway-aligned outer-final
                   station, so there is no circle to capture after its endpoint.
                   Accept a small passed-end corridor and hand residual error to
                   the runway-line Frenet law. */
                bool passed_in_corridor=passed_endpoint>=0&&
                    endpoint_cross<=fmax(300.0,radius*.16);
                bool exhausted=g->hac_transition_progress>=.995&&
                    (passed_endpoint>=-fmax(250.0,radius*.06)||
                     endpoint<=fmax(350.0,t->true_air_speed*2.5));
                if(g->hac_transition_progress>=.975&&end_heading_error<=22.0&&
                   (close_endpoint||passed_in_corridor||exhausted)){
                    g->hac_transition_active=false;g->hac_remaining=0.0;
                    g->hac_progress_valid=true;g->hac_captured=true;
                    h=terminal_runway_line_path_guidance(t,&cfg->site,hs,
                        planet_surface_gravity(p),course);
                }
            }else{
                /* Handoff after crossing the tangent endpoint in the circle capture
                   corridor. Do not strand a completed join on its final tangent. */
                bool passed_in_corridor=passed_endpoint>=0&&endpoint_cross<=radius*.45&&
                    fabs(h.radial_error)<=radius*.45;
                bool circle_capture_corridor=fabs(h.radial_error)<=radius*.55&&
                    fabs(h.course_error)<=65.0;
                bool exhausted=g->hac_transition_progress>=.995&&
                    (passed_endpoint>=-radius*.10||endpoint<=radius*.45);
                if(g->hac_transition_progress>=.975&&
                   (((close_endpoint||passed_in_corridor)&&end_heading_error<=28.0)||
                    (exhausted&&circle_capture_corridor))){
                    g->hac_transition_active=false;
                    double join_advance=-g->hac_side*norm_signed_deg(
                        (h.angle-g->hac_transition_end_angle)*RAD2DEG)*DEG2RAD;
                    g->hac_remaining-=join_advance*radius;
                    g->hac_previous_angle=h.angle;g->hac_progress_valid=true;
                }
            }
        }
    }
    if(transition_block){
        /* The cubic join already carries its own curvature. Do not consume
           circle arc from raw polar-angle motion until the aircraft has
           actually reached the tangent endpoint. */
    }else if(spline_path){
        h=terminal_runway_line_path_guidance(t,&cfg->site,hs,planet_surface_gravity(p),course);
        g->hac_remaining=0.0;
    }else if(g->terminal_glide_mode&&g->terminal_test_spiral_active&&!g->hac_completed){
        h=terminal_test_update_spiral(g,t,course,p,aero,cfg,dt);
        radius=g->hac_radius;
        feasible=isfinite(g->minimum_turn_radius)&&g->minimum_turn_radius<=radius/1.06;
    } else if (g->hac_progress_valid && !g->hac_completed) {
        /* Integrate signed angular progress through atan2's branch cut. The
           remaining arc can never jump back to a fresh positive revolution.
           Reverse motion restores distance so repeated oscillations cannot
           manufacture progress. Once established, retain signed position even
           during loss of capture so the physical exit cannot disappear. */
        double progress = -g->hac_side * norm_signed_deg((h.angle - g->hac_previous_angle) * RAD2DEG) * DEG2RAD;
        g->hac_remaining -= progress * radius;
    } else if (!g->hac_progress_valid && (!g->terminal_glide_mode || g->hac_remaining <= 0)) {
        g->hac_remaining = h.arc_remaining;
    }
    g->hac_previous_angle = h.angle;
    TerminalPreflarePlan approach_plan={0};
    bool approach=terminal_outer_gate(g,t,course,p,aero,cfg,&approach_plan);
    if(spline_path&&!transition_block&&!approach){
        TerminalPreflarePlan spline_plan={0};
        if(terminal_outer_capture_admissible(g,t,course,p,aero,cfg,&spline_plan)){
            approach=true;approach_plan=spline_plan;
        }
    }
    double exit_window=fmax(200,radius*.012);
    double station=-hs->final_approach_distance;
    double lead=clampd(t->horizontal_speed*hac_response_lead_time(t,s,g->hac_side*25.0),180.0,2400.0);
    bool at_exit=t->runway_along_track>=station-lead&&t->runway_along_track<=station+exit_window&&
        fabs(t->runway_cross_track)<fmax(250.0,fmin(1000.0,radius*.10))&&
        fabs(norm_signed_deg(course-cfg->site.runway_heading))<12.0;
    if(!spline_path&&!g->hac_completed&&!g->final_approach_captured&&!transition_block&&!approach&&at_exit&&
       (!g->hac_progress_valid||g->hac_remaining<=lead+exit_window)){
        GuidanceSettings circuit_settings=*hs;
        circuit_settings.taem_glide_slope=s->taem_glide_slope;
        if(g->terminal_glide_mode)circuit_settings.final_glide_slope=g->terminal_test_glide_slope;
        double arc=0,slope=0;
        /* At the first crossing the geometric arc may already have wrapped.
           Convert it to signed distance past this exit before budgeting ONE circuit. */
        double remaining=g->hac_remaining;
        if(!g->hac_progress_valid&&remaining>LANDER_PI*radius)remaining-=2*LANDER_PI*radius;
        if(hac_high_pass_circuit(t->mean_altitude,t->true_air_speed,radius,remaining,
            g->minimum_turn_radius,live_drag_accel(t,aero,v),p,&cfg->site,v,&circuit_settings,&arc,&slope)){
            g->hac_remaining=arc;g->hac_circuit_slope=slope;g->hac_circuit_count++;
            g->hac_progress_valid=true;g->hac_captured=true;g->hac_previous_angle=h.angle;
            path_settings.taem_glide_slope=slope;
            h=hac_guidance_radius(t,&cfg->site,hs,p->radius,g->hac_side,planet_surface_gravity(p),course,radius);
            robust_pid_reset(&g->taem_altitude_pid);
        }
    }
    /* HAC capture is a lateral-path state, not an energy-state gate.  Energy
       still determines the vertical schedule, final-approach validity and
       whether an explicit high-pass circuit is feasible, but it must not keep
       a geometrically captured aircraft labelled as TAEM. */
    if(spline_path){
        double line_capture=fmax(220.0,fmin(850.0,hs->final_approach_distance*.085));
        g->hac_captured=!g->hac_transition_active&&
            fabs(t->runway_cross_track)<=line_capture&&
            fabs(norm_signed_deg(cfg->site.runway_heading-course))<24.0;
    }else g->hac_captured=!g->hac_transition_active&&feasible&&
        fabs(h.radial_error)<=fmax(250,fmin(1500.0,radius*.10))&&fabs(h.course_error)<24;

    bool lost=spline_path?
        (fabs(t->runway_cross_track)>fmax(1200.0,hs->final_approach_distance*.20)||
         fabs(norm_signed_deg(cfg->site.runway_heading-course))>65.0):
        (!feasible||fabs(h.radial_error)>fmax(750.0,fmin(5000.0,radius*.25))||fabs(h.course_error)>60.0);
    g->hac_capture_lost_duration=lost?g->hac_capture_lost_duration+dt:fmax(0,g->hac_capture_lost_duration-dt*2);
    bool entry_recoverable=terminal_entry_recovery_available(g,t,v,s);
    if(g->hac_capture_lost_duration>=5.0&&entry_recoverable)
        return terminal_return_to_entry(g,t,course,p,aero,cfg,dt);
    /* A committed join may cross the exit plane before looping back to its
       tangent. Judge station passage only after that join has completed. */
    bool missed_station=!spline_path&&!transition_block&&g->hac_progress_valid&&
        t->runway_along_track>station+exit_window&&
        fabs(norm_signed_deg(course-cfg->site.runway_heading))<45&&
        g->hac_remaining<=exit_window;
    if(!g->hac_completed&&!g->final_approach_captured&&!approach&&missed_station){
        if(entry_recoverable)return terminal_return_to_entry(g,t,course,p,aero,cfg,dt);
        return terminal_abort(g,"HAC missed: no feasible high-pass circuit or final capture remains.");
    }
    if (!g->hac_completed && g->hac_progress_valid && g->hac_remaining <= exit_window) {
        if(g->hac_remaining>=-exit_window&&g->hac_captured&&approach&&t->runway_along_track<0){g->hac_completed=true;g->hac_remaining=0;g->terminal_test_spiral_active=false;g->terminal_test_revolution_remaining=0;}
        else if(g->hac_remaining < -exit_window)
            return terminal_abort(g,"HAC missed: exit crossed without a valid final capture.");
    }
    TaemTerminalContract delivery_contract=terminal_delivery_contract(g,t,course,p,cfg,&approach_plan);
    /* Publish the live numeric contract before any public Final latch.  The
       executive must first admit FINAL_INTERCEPT, then explicitly deliver Final
       on the same qualified contract; coarse HAC-complete/approach booleans are
       no longer sufficient to bypass those checks. */
    (void)taem_exec_sync_with_contract(g,t,s,&delivery_contract,false,false);
    if(g->hac_completed && !g->final_approach_captured && !approach)
        return terminal_abort(g,spline_path?
            "Terminal spline exit rejected: final approach envelope is invalid.":
            "HAC exit rejected: final approach envelope is invalid.");
    if (!g->final_approach_captured && g->hac_completed && approach && t->runway_along_track < 0 &&
        g->taem_exec.phase==TAEM_PHASE_FINAL_INTERCEPT&&g->taem_exec.terminal_evaluation.feasible) {
        (void)taem_exec_sync_with_contract(g,t,s,&delivery_contract,true,true);
        if(g->taem_exec.taem_complete){
            g->final_approach_captured = true;
            terminal_store_preflare_plan(g,&approach_plan);
            terminal_set_stage(g,TERMINAL_TRAJECTORY_CAPTURE,t->ut);
            robust_pid_reset(&g->final_altitude_pid);
            robust_pid_reset(&g->speed_pid);
            robust_pid_reset(&g->flare_sink_pid);
        }
    }
    if(!g->final_approach_captured||!g->taem_exec.taem_complete)g->final_invalid_duration=0;
    Trajectory ref;
    trajectory_init(&ref);
    reference_trajectory_radius(&ref, &cfg->site, hs, p->radius, g->hac_side, radius);
    GuidanceResult r;
    if (!g->final_approach_captured || !g->taem_exec.taem_complete) {
        if(g->hac_captured)g->hac_progress_valid = true;
        /* MM305 owns path acquisition and runway alignment as internal TAEM
           substates.  Do not surface a peer Heading-Alignment guidance phase;
           public ownership stays PHASE_TAEM until final approach is captured. */
        g->phase = PHASE_TAEM;
        r = taem_guidance(g, t, course, p, aero, cfg, &ref, dt);
    } else {
        r=terminal_approach_sequence(g,t,course,p,aero,cfg,&ref,dt);
    }
    trajectory_clear(&ref);
    return r;
}

#define guidance_update guidance_update_impl
GuidanceResult guidance_update(GuidanceMachine*g,const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){double dt=g->has_previous_ut?clampd(t->ut-g->previous_ut,0,1):.1;g->previous_ut=t->ut;g->has_previous_ut=true;if(g->aborted){GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_ABORT,c,g->abort_reason[0]?g->abort_reason:"Automation aborted. Manual control restored.",NULL);}if(!g->automation_engaged){g->phase=PHASE_IDLE;reset_limiters(g);GuidanceCommand c;guidance_command_init(&c);if(plan){if(plan->execution_qualified)return result_make(PHASE_IDLE,c,plan->execution_degraded?"Recoverable deorbit plan is ready. Engage guidance to begin execution.":"Robust deorbit plan is ready. Engage guidance to begin execution.",plan->execution_degraded?"The plan misses the preferred strict corridor but passed the guarded recovery envelope.":NULL);return result_make(PHASE_IDLE,c,"The current deorbit plan is preview-only and cannot be executed safely.","Replan for a later orbital opportunity or revise the vehicle/site model before engagement.");}return result_make(PHASE_IDLE,c,"Guidance is not engaged.",NULL);}if(g->paused){g->phase=PHASE_PAUSED;GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_PAUSED,c,"Guidance paused. Attitude hold released.",NULL);}const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;double course=surface_course(state->position,state->velocity,planet_rotation_vector(p),p->north_axis,t->heading);if(g->terminal_glide_mode)return terminal_guidance(g,t,state,course,p,aero,cfg,dt);if(!plan){g->phase=PHASE_PLANNING;GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_PLANNING,c,"A deorbit plan is required before engagement.",NULL);}double entry_alt=entry_guidance_start_altitude(p,s);if(!g->deorbit_burn_completed&&t->mean_altitude<=entry_alt&&t->vertical_speed<0)g->deorbit_burn_completed=true;double peri_rem=t->periapsis_altitude-plan->predicted_post_burn_periapsis_altitude;bool has_peri=plan->predicted_post_burn_periapsis_altitude>10000&&plan->predicted_post_burn_periapsis_altitude<p->atmosphere_depth&&isfinite(t->periapsis_altitude)&&t->periapsis_altitude>-p->radius*.5,peri_hit=g->has_burn_command_started&&has_peri&&peri_rem<=300,state_hit=g->has_burn_command_started&&plan->live_cutoff_capture_qualified;double dvtarget=plan->delta_v+(has_peri?8:0);if(g->has_burn_command_started&&(g->delivered_delta_v>=dvtarget||peri_hit||state_hit))g->deorbit_burn_completed=true;bool still=g->has_burn_command_started&&!g->deorbit_burn_completed&&g->delivered_delta_v<dvtarget&&!peri_hit&&!state_hit;double burnstart=plan->burn_ut-plan->estimated_burn_duration*.5,open=t->ut>=burnstart;if(!g->deorbit_burn_completed&&t->mean_altitude>entry_alt-4500&&(t->vertical_speed>=-20||still||open)){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.use_inertial_direction=true;c.inertial_direction=vnorm(vscale(state->velocity,-1),v3(-1,0,0));c.navball_speed_mode=SPEED_ORBIT;c.control_profile=PROFILE_ORBITAL;if(t->ut<burnstart-12){g->phase=PHASE_COAST;char st[128];int sec=(int)fmax(0,round(burnstart-t->ut));snprintf(st,sizeof(st),"Coasting while acquiring retrograde. T−%02d:%02d.",sec/60,sec%60);return result_make(g->phase,c,st,NULL);}if(t->ut<burnstart){g->phase=PHASE_BURN_SETUP;return result_make(g->phase,c,"Aligning retrograde for the deorbit burn.",NULL);}if(g->delivered_delta_v<dvtarget&&!peri_hit&&!state_hit){g->phase=PHASE_DEORBIT_BURN;double maxthr=s->deorbit_maximum_throttle,avail=fmax(.05,t->available_thrust*maxthr/fmax(t->mass,1)),meas=fmax(0,t->current_thrust/fmax(t->mass,1)),align=fabs(t->autopilot_error);bool aligned=align<=10;if(!aligned&&!g->has_burn_command_started&&t->ut>burnstart+fmax(30,plan->estimated_burn_duration)){guidance_abort(g);GuidanceCommand safe;guidance_command_init(&safe);return result_make(PHASE_ABORT,safe,"Deorbit burn aborted because retrograde alignment missed the burn window.","The vehicle did not achieve the required retrograde alignment in time. Manual control restored; replan from the current orbit.");}if(aligned&&!g->has_burn_command_started){g->burn_command_started_ut=t->ut;g->has_burn_command_started=true;g->burn_active_elapsed=0;g->burn_progress_watch_ut=t->ut;g->burn_progress_watch_delta_v=g->delivered_delta_v;g->has_burn_progress_watch=true;}if(aligned&&g->has_burn_command_started){g->delivered_delta_v+=meas*fmax(0,cos(align*DEG2RAD))*dt;g->burn_active_elapsed+=dt;if(!g->has_burn_progress_watch||g->delivered_delta_v-g->burn_progress_watch_delta_v>=.25){g->burn_progress_watch_ut=t->ut;g->burn_progress_watch_delta_v=g->delivered_delta_v;g->has_burn_progress_watch=true;}}double rem=fmax(0,dvtarget-g->delivered_delta_v);double fraction=burn_fraction(g->burn_active_elapsed,rem,avail,s->deorbit_throttle_ramp_duration),authority=clampd((10-align)/6,0,1),pa=has_peri?clampd((peri_rem-250)/8000,0,1):1;c.target_throttle=rem<.08?0:(aligned?maxthr*fmin(fraction,pa)*authority:0);double expected_accel=fmax(0,t->available_thrust*c.target_throttle/fmax(t->mass,1));bool stalled=aligned&&g->has_burn_command_started&&g->has_burn_progress_watch&&rem>2&&t->ut-g->burn_progress_watch_ut>6&&expected_accel>.02&&meas<fmax(.01,expected_accel*.10);if(stalled){guidance_abort(g);GuidanceCommand safe;guidance_command_init(&safe);return result_make(PHASE_ABORT,safe,"Deorbit burn aborted after sustained loss of thrust/delta-v progress.","The burn stopped making measurable progress despite a meaningful commanded thrust level. Manual control restored; do not continue entry on the stale plan.");}char st[320];snprintf(st,sizeof(st),"Deorbit burn: %.1f / %.1f m/s, throttle %.0f%%.%s%s",g->delivered_delta_v,plan->delta_v,c.target_throttle*100,has_peri?" Periapsis closure active.":"",plan->live_cutoff_capture_qualified?" Cutoff-now trajectory is inside the entry corridor.":"");return result_make(g->phase,c,st,t->available_thrust<1?"No usable thrust is available.":align>10?"Holding throttle until retrograde alignment is stable.":NULL);}}
    if(!g->deorbit_burn_completed){g->phase=PHASE_COAST;GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.use_inertial_direction=true;c.inertial_direction=vnorm(vscale(state->velocity,-1),v3(-1,0,0));c.navball_speed_mode=SPEED_ORBIT;c.control_profile=PROFILE_ORBITAL;return result_make(g->phase,c,"Holding retrograde until the deorbit burn completes.",NULL);}if(!g->atmospheric_interface_crossed){double signed_entry_roll=norm_signed_deg(t->roll),entry_roll=fabs(signed_entry_roll),capture_aoa=entry_low_q_protective_aoa_floor(t->dynamic_pressure,v),entry_pitch_error=fabs(t->angle_of_attack-capture_aoa),entry_heading_error=fabs(norm_signed_deg(t->ground_track_heading-t->heading));bool entry_attitude_ready=entry_roll<=18&&entry_pitch_error<=4&&entry_heading_error<=15&&fabs(t->roll_rate)<=8&&fabs(t->pitch_rate)<=8&&fabs(t->heading_rate)<=8;bool entry_reached=t->mean_altitude<=entry_alt&&t->vertical_speed<0;if(entry_reached&&entry_attitude_ready){g->atmospheric_interface_crossed=true;/* Continue from the bank direction actually reached instead of forcing an unnecessary high-Mach reversal. */if(entry_roll>=5)g->s_turn_sign=signed_entry_roll<0?-1:1;}else{g->phase=PHASE_ENTRY_INTERFACE;const char*st=entry_reached?"Atmospheric interface reached. Holding prograde heading, entry AoA and wings-level attitude before MM304 guidance.":"Burn complete. Using RCS/direct control to capture prograde heading, entry AoA and wings-level attitude.";return result_make(g->phase,entry_capture(t,state,v),st,entry_reached&&!entry_attitude_ready?"MM304 guidance is inhibited until entry heading/AoA/roll attitude is stabilized.":NULL);}}
    return terminal_guidance(g,t,state,course,p,aero,cfg,dt);
}
#undef guidance_update

GuidanceResult guidance_update(GuidanceMachine*g,const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    GuidanceResult r=guidance_update_impl(g,t,state,plan,p,aero,cfg);
    if(plan&&r.phase==PHASE_DEORBIT_BURN&&g->has_burn_command_started&&!g->deorbit_burn_completed&&r.command.target_throttle<=0){
        bool has_peri=plan->predicted_post_burn_periapsis_altitude>10000&&plan->predicted_post_burn_periapsis_altitude<p->atmosphere_depth&&isfinite(t->periapsis_altitude)&&t->periapsis_altitude>-p->radius*.5;
        double target=plan->delta_v+(has_peri?8:0),remaining=target-g->delivered_delta_v;
        if(remaining>0&&remaining<=.08+1e-9){
            g->delivered_delta_v=target;
            g->deorbit_burn_completed=true;
            g->phase=PHASE_ENTRY_INTERFACE;
            guidance_result_clear(&r);
            return result_make(PHASE_ENTRY_INTERFACE,entry_capture(t,state,&cfg->vehicle),"Burn complete. Using RCS/direct control to capture prograde entry attitude until atmospheric interface.",NULL);
        }
    }
    /* TAEM already carries large, continuously moving bank references.  Use
       the same body-rate-damped terminal roll loop before HAC commit instead
       of the generic attitude loop, which was driving +/-100 deg/s roll
       limit cycles while chasing a 25-40 deg TAEM bank. */
    r.command.hac_control_tuning=g->terminal_region_entered&&
        (r.phase==PHASE_TAEM||g->hac_side_selected)&&
        (!g->terminal_glide_mode||!g->terminal_test_capture_active||r.phase==PHASE_TAEM);
    /* Pitch capture needs its own terminal-only controller before the HAC is
       fully selected in the dedicated rehearsal. Production entry/S-turn is
       deliberately excluded; normal flight only enables this after terminal
       region capture, while the HAC test may use it on the pre-HAC dogleg. */
    r.command.terminal_pitch_tuning=g->terminal_region_entered&&
        (g->hac_side_selected||(g->terminal_glide_mode&&g->terminal_test_capture_active));
    return r;
}

bool guidance_install_entry_topology(GuidanceMachine*g,const EntryTopologyPlan*top,double ut){
    if(!g||!top||!top->valid||!top->inlet.valid||!isfinite(ut)||top->reversal_ut<=ut||
       top->capture_ut<=top->reversal_ut||g->entry_topology.valid||g->entry_control_reversals||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed)return false;
    bool replaceable_ordinary_reversal=g->entry_reversal_scheduled&&
        !g->entry_reversal_is_final&&top->first_sign*g->s_turn_sign>0.0;
    if(g->entry_reversal_scheduled&&!replaceable_ordinary_reversal)return false;
    /* The propagated topology may contain a shaping S-turn reversal followed by a
       distinct terminal 90 deg turn. Install the first event as nonfinal; the live
       topology executor owns the later measured-radius terminal-turn release and
       keeps both events tied to the same fixed TAEM inlet contract. */
    bool separate_terminal_turn=isfinite(top->terminal_turn_ut)&&top->terminal_turn_ut>top->reversal_ut+1.0;
    g->entry_topology=*top;g->taem_interface_target=top->inlet;
    g->entry_topology_capture_good_duration=0.0;
    g->entry_topology_heading_locked=false;
    request_side(g,top->first_sign,ut);
    EntryControlPlan plan={.valid=true,.planned_ut=top->planned_ut,.target_bank=top->first_sign*top->first_bank,
        .target_aoa=top->first_aoa,.target_heading=top->inlet.course,.bank_cap=fmax(top->first_bank,top->turn_bank),
        .target_turn_radius=INFINITY,.segment_duration=top->capture_ut+30.0-top->planned_ut,
        .cost=top->cost,.taem_range_error=top->position_error,.taem_speed=top->capture_speed,.taem_energy_error=NAN,
        .has_planned_reversal=true,.planned_reversal_is_final=!separate_terminal_turn,.planned_reversal_ut=top->reversal_ut,
        .planned_reversal_range=hypot(top->reversal_along,top->reversal_cross),
        .planned_reversal_sign=-top->first_sign,.predicted_reversals=1};
    control_plan_assign_lineage(g,&plan,&g->entry_s_turn_plan);
    g->entry_s_turn_plan=plan;g->entry_control_plan_valid=true;
    entry_program_commit_planned_reversal(g,&plan,ut,replaceable_ordinary_reversal);
    g->entry_planning_needed=false;g->entry_committed_infeasible=false;
    return true;
}

GuidanceCommand guidance_entry_reference_step(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const GuidanceSettings*s,double bank,double aoa,double dt){
    GuidanceCommand command=atmospheric(t,t->ground_track_heading,bank,v,0.0,false,PROFILE_ENTRY);
    command.heading_control_enabled=false;command.has_target_aoa=true;
    command.target_aoa=aoa;command.target_pitch=t->flight_path_angle+aoa;
    GuidanceResult result=stabilized(g,result_make(PHASE_ENTRY_ENERGY,command,"",NULL),t,v,s,dt);
    command=result.command;guidance_result_clear(&result);return command;
}
