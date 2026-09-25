#include "landing.h"

#include <float.h>
#include <math.h>
#include <string.h>

double clampd(double v, double lo, double hi) { return fmin(fmax(v, lo), hi); }
double norm_deg(double a) { double r = fmod(a, 360.0); return r < 0 ? r + 360.0 : r; }
double norm_signed_deg(double a) { double r = norm_deg(a); return r > 180.0 ? r - 360.0 : r; }

TaemHandoffContract taem_handoff_contract(const GuidanceSettings *s) {
    TaemHandoffContract out={0};
    if(!s)return out;
    out.horizontal_radius_m=s->mm304_handoff_radius;
    out.perpendicular_heading_half_width_deg=
        s->mm304_perpendicular_heading_half_width;
    return out;
}

void mm304_handoff_station(const GuidanceSettings *settings,double *along_m,double *cross_m) {
    if(along_m)*along_m=settings?settings->mm304_handoff_along_track:NAN;
    if(cross_m)*cross_m=settings?settings->mm304_handoff_cross_track:NAN;
}

double entry_s_turn_effective_minimum_leg(double true_air_speed,double taem_speed,const GuidanceSettings *settings) {
    /*
     * The dwell is an executive anti-chatter requirement only.  Vehicle-response
     * time is computed from the bounded roll plant at each call site; do not hide
     * another speed-fitted flight law inside this guard.
     */
    (void)true_air_speed;
    (void)taem_speed;
    return settings?fmax(0.0,settings->s_turn_minimum_leg_duration):0.0;
}

Vector3 v3(double x,double y,double z) { Vector3 r = {x,y,z}; return r; }
Vector3 vadd(Vector3 a,Vector3 b) { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
Vector3 vsub(Vector3 a,Vector3 b) { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
Vector3 vscale(Vector3 a,double s) { return v3(a.x*s,a.y*s,a.z*s); }
double vdot(Vector3 a,Vector3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vector3 vcross(Vector3 a,Vector3 b) { return v3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x); }
double vmag(Vector3 a) {
    /* hypot preserves scale across very small/large finite components instead
       of squaring first and introducing artificial under/overflow. */
    return hypot(hypot(a.x,a.y),a.z);
}
Vector3 vnorm(Vector3 a, Vector3 fallback) {
    double m=vmag(a);
    /* Zero is the actual geometric degeneracy. A fixed magnitude epsilon is
       unit-dependent and can discard a perfectly valid direction. */
    return isfinite(m)&&m>0.0?vscale(a,1.0/m):fallback;
}
Vector3 vproject_plane(Vector3 a, Vector3 normal) { return vsub(a,vscale(normal,vdot(a,normal))); }
Vector3 vrotate(Vector3 a,Vector3 axis,double radians) {
    axis=vnorm(axis,v3(0,0,0)); double c=cos(radians),s=sin(radians);
    return vadd(vadd(vscale(a,c),vscale(vcross(axis,a),s)),vscale(axis,vdot(axis,a)*(1-c)));
}

double great_circle_distance(GeoPoint a, GeoPoint b, double radius) {
    double lat1=a.latitude*DEG2RAD,lat2=b.latitude*DEG2RAD;
    double dlat=(b.latitude-a.latitude)*DEG2RAD,dlon=norm_signed_deg(b.longitude-a.longitude)*DEG2RAD;
    double h=sin(dlat/2)*sin(dlat/2)+cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
    return radius*2*atan2(sqrt(h),sqrt(fmax(0,1-h)));
}
double initial_bearing(GeoPoint a, GeoPoint b) {
    double lat1=a.latitude*DEG2RAD,lat2=b.latitude*DEG2RAD,dlon=norm_signed_deg(b.longitude-a.longitude)*DEG2RAD;
    double y=sin(dlon)*cos(lat2),x=cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon);
    return norm_deg(atan2(y,x)*RAD2DEG);
}
GeoPoint destination_point(GeoPoint a,double bearing,double distance,double radius) {
    double ad=distance/radius,th=bearing*DEG2RAD,lat1=a.latitude*DEG2RAD,lon1=a.longitude*DEG2RAD;
    double lat2=asin(sin(lat1)*cos(ad)+cos(lat1)*sin(ad)*cos(th));
    double lon2=lon1+atan2(sin(th)*sin(ad)*cos(lat1),cos(ad)-sin(lat1)*sin(lat2));
    GeoPoint p={lat2*RAD2DEG,norm_signed_deg(lon2*RAD2DEG),a.altitude}; return p;
}
GeoPoint runway_approach_aimpoint(const LandingSite *site,double radius,double distance_before_threshold) {
    GeoPoint threshold={site->latitude,site->longitude,site->altitude};
    return destination_point(threshold,norm_deg(site->runway_heading+180.0),fmax(0,distance_before_threshold),radius);
}

LandingSite runway_reciprocal_site(const LandingSite *primary,double radius) {
    LandingSite out=*primary;
    GeoPoint threshold={primary->latitude,primary->longitude,primary->altitude};
    GeoPoint reciprocal=destination_point(threshold,primary->runway_heading,
        fmax(0.0,primary->runway_length),radius);
    out.latitude=reciprocal.latitude;
    out.longitude=reciprocal.longitude;
    out.altitude=reciprocal.altitude;
    out.runway_heading=norm_deg(primary->runway_heading+180.0);
    return out;
}

void telemetry_reframe_runway(Telemetry *t,const LandingSite *site,double radius) {
    if(!t||!site)return;
    GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
    GeoPoint threshold={site->latitude,site->longitude,site->altitude};
    t->range_to_site=great_circle_distance(current,threshold,radius);
    t->bearing_to_site=initial_bearing(current,threshold);
    t->heading_error=norm_signed_deg(t->bearing_to_site-t->heading);
    t->course_to_site_error=norm_signed_deg(t->bearing_to_site-t->ground_track_heading);
    runway_coordinates(current,threshold,site->runway_heading,radius,
        &t->runway_along_track,&t->runway_cross_track);
}

void local_offsets(GeoPoint origin,GeoPoint p,double radius,double *east,double *north) {
    /* Use exact spherical distance+bearing instead of the old equirectangular
       lat/lon approximation; Kerbin is small enough for terminal-scale map error
       to become material. East/north remains only a local HAC representation. */
    double distance=great_circle_distance(origin,p,radius);
    double bearing=initial_bearing(origin,p)*DEG2RAD;
    *east=distance*sin(bearing); *north=distance*cos(bearing);
}
GeoPoint local_point(GeoPoint origin,double east,double north,double radius,double altitude) {
    double distance=hypot(east,north);
    double bearing=distance>0.0?norm_deg(atan2(east,north)*RAD2DEG):0.0;
    GeoPoint p=destination_point(origin,bearing,distance,radius); p.altitude=altitude; return p;
}
double surface_course(Vector3 pos,Vector3 vel,Vector3 omega,Vector3 north_axis,double fallback) {
    Vector3 up=vnorm(pos,v3(0,1,0)),surface=vsub(vel,vcross(omega,pos)),horizontal=vproject_plane(surface,up);
    if (!(vmag(horizontal)>0.0)) return norm_deg(fallback);
    Vector3 north=vnorm(vsub(north_axis,vscale(up,vdot(north_axis,up))),v3(0,0,0));
    Vector3 east=vnorm(vcross(north,up),v3(0,0,0));
    if (!(vmag(north)>0.0)||!(vmag(east)>0.0)) return norm_deg(fallback);
    return norm_deg(atan2(vdot(horizontal,east),vdot(horizontal,north))*RAD2DEG);
}
void runway_coordinates(GeoPoint point,GeoPoint site,double heading,double radius,double *along,double *cross) {
    /*
     * Project onto the runway's oriented great-circle frame.  Treating the
     * site-to-point displacement as a local east/north vector is not valid
     * near the antipode: the shortest-path bearing becomes nearly due west
     * while the signed cross-track distance remains small.  The simulator
     * uses this same cross-track/along-track convention, so keep production
     * telemetry and the offline world model on one geometric definition.
     */
    double lat1=site.latitude*DEG2RAD,lat2=point.latitude*DEG2RAD;
    double dlon=norm_signed_deg(point.longitude-site.longitude)*DEG2RAD;
    double cos_delta=sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(dlon);
    double delta=acos(clampd(cos_delta,-1.0,1.0));
    double bearing=atan2(sin(dlon)*cos(lat2),
        cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon));
    double rel=bearing-heading*DEG2RAD;
    rel=atan2(sin(rel),cos(rel));
    if(along)*along=atan2(sin(delta)*cos(rel),cos(delta))*radius;
    if(cross)*cross=asin(clampd(sin(delta)*sin(rel),-1.0,1.0))*radius;
}

double taem_interface_line_heading(double runway_along,double runway_cross,double runway_heading,
        const TaemInterfaceTarget*q){
    if(!q||!q->valid||!isfinite(runway_along)||!isfinite(runway_cross)||
       !isfinite(runway_heading)||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course))
        return isfinite(runway_heading)?norm_deg(runway_heading):0.0;

    double rel=norm_signed_deg(q->course-runway_heading)*DEG2RAD;
    double tangent_along=cos(rel),tangent_cross=sin(rel);
    double normal_along=-tangent_cross,normal_cross=tangent_along;
    double da=runway_along-q->along_track,dc=runway_cross-q->cross_track;
    double along_error=da*tangent_along+dc*tangent_cross;
    double line_error=da*normal_along+dc*normal_cross;

    /*
     * Nonlinear path-following vector field. acquisition_lead is not a tuning
     * angle: it is the physical distance needed by the selected TAEM target for
     * actuator/trajectory acquisition. If it is unavailable, use the remaining
     * along-line distance itself. atan2 naturally bounds the intercept below
     * 90 degrees; no second arbitrary heading cap exists.
     */
    double scale=q->acquisition_lead>0.0?
        q->acquisition_lead:fabs(along_error);
    double intercept=-atan2(line_error,scale)*RAD2DEG;
    return norm_deg(q->course+intercept);
}

double taem_course_rate_bank_command(double horizontal_speed,double lift_accel,
        double bank_effectiveness,double bank_limit,double current_course,
        double desired_course,double response_time){
    if(!isfinite(horizontal_speed)||!(horizontal_speed>0.0)||
       !isfinite(lift_accel)||!(lift_accel>0.0)||
       !isfinite(bank_effectiveness)||!(bank_effectiveness>0.0)||
       !isfinite(bank_limit)||!(bank_limit>0.0)||
       !isfinite(current_course)||!isfinite(desired_course)||
       !isfinite(response_time)||!(response_time>0.0))
        return 0.0;

    double limit=fmin(fabs(bank_limit),nextafter(90.0,0.0)); /* decision-literal: mathematical-numerical-requirement | avoid the tan(bank) singularity at ninety degrees */
    double effective_limit=fmin(limit*bank_effectiveness,nextafter(90.0,0.0)); /* decision-literal: mathematical-numerical-requirement | avoid the tan(bank) singularity at ninety degrees */
    double max_lateral=lift_accel*sin(effective_limit*DEG2RAD);
    if(!(max_lateral>0.0))return 0.0;

    double max_course_rate=max_lateral/horizontal_speed;
    double error=norm_signed_deg(desired_course-current_course)*DEG2RAD;
    double requested_rate=error/response_time;
    double course_rate=clampd(requested_rate,-max_course_rate,max_course_rate);
    double lateral=horizontal_speed*course_rate;

    double ratio=clampd(lateral/lift_accel,
        -sin(effective_limit*DEG2RAD),sin(effective_limit*DEG2RAD));
    double bank=asin(ratio)/bank_effectiveness*RAD2DEG;
    return clampd(bank,-limit,limit);
}

double bank_for_point_capture(double speed,double lift_accel,double bank_effectiveness,double course_error,double distance,double maximum_bank) {
    if(!(speed>0.0)||!(lift_accel>0.0)||!(distance>0.0)||
       !(maximum_bank>0.0)||!(bank_effectiveness>0.0))return 0.0;
    double eff=bank_effectiveness;
    double limit=fmin(maximum_bank,nextafter(90.0,0.0)); /* decision-literal: mathematical-numerical-requirement | avoid the tan(bank) singularity at ninety degrees */
    /* The unique constant-curvature circle tangent to the current course and
       passing through the aimpoint has curvature 2*sin(error)/distance.  Turn
       that geometric curvature into lateral acceleration, then invert the
       measured lift vector for bank.  This makes final S-turn exit guidance a
       direct physical point-capture law rather than another heading PID. */
    double curvature=2.0*sin(norm_signed_deg(course_error)*DEG2RAD)/distance;
    double lateral=speed*speed*curvature;
    double effective_limit=fmin(limit*eff,nextafter(90.0,0.0))*DEG2RAD; /* decision-literal: mathematical-numerical-requirement | avoid the tan(bank) singularity at ninety degrees */
    double ratio=clampd(lateral/lift_accel,-sin(effective_limit),sin(effective_limit));
    return clampd(asin(ratio)/eff*RAD2DEG,-limit,limit);
}

void robust_pid_init(RobustPID *p,double kp,double ki,double kd,double limit,double tc) {
    memset(p,0,sizeof(*p)); p->kp=kp;p->ki=ki;p->kd=kd;p->integral_limit=fmax(0,limit);p->derivative_tc=fmax(0,tc);
}
void robust_pid_reset(RobustPID *p) { p->integral=0;p->previous_error=0;p->filtered_derivative=0;p->has_previous=false; }
double robust_pid_update(RobustPID *p,double error,double dt,double lo,double hi) {
    if (!(dt>0) || !isfinite(error)) return clampd(p->kp*error,lo,hi);
    double raw=p->has_previous?(error-p->previous_error)/dt:0;
    double alpha=p->derivative_tc<=0?1:dt/(dt+p->derivative_tc);
    p->filtered_derivative+=alpha*(raw-p->filtered_derivative); p->previous_error=error;p->has_previous=true;
    double candidate=clampd(p->integral+error*dt,-p->integral_limit,p->integral_limit);
    double pd=p->kp*error+p->kd*p->filtered_derivative;
    double out=pd+p->ki*candidate,limited=clampd(out,lo,hi);
    if (out==limited) p->integral=candidate;
    else if (out>hi) {
        if (error<0) p->integral=candidate;
        else if (p->ki!=0 && pd<hi) p->integral=clampd((hi-pd)/p->ki,-p->integral_limit,p->integral_limit);
    } else if (error>0) p->integral=candidate;
    else if (p->ki!=0 && pd>lo) p->integral=clampd((lo-pd)/p->ki,-p->integral_limit,p->integral_limit);
    return clampd(pd+p->ki*p->integral,lo,hi);
}
double lowpass_update(LowPass *f,double value,double dt) {
    if (!f->has_value) { f->value=value;f->has_value=true;return value; }
    if(!isfinite(value)||!isfinite(dt)||!(dt>0.0))return f->value;
    if(!isfinite(f->tc))return f->value;
    if(!(f->tc>0.0)){f->value=value;return value;}
    /* Exact first-order discretization parameter for a finite positive time
       constant. No absolute epsilon is needed: dt and tc carry the same unit. */
    double alpha=dt/(dt+f->tc);
    f->value+=alpha*(value-f->value);
    return f->value;
}
void lowpass_reset(LowPass *f) { f->has_value=false;f->value=0; }
double jerk_update(JerkLimiter *j,double target,double max_rate,double max_accel,double dt) {
    if(!j)return NAN;
    if(!isfinite(target)||!isfinite(dt)||!(dt>0.0))return j->has_value?j->value:NAN;
    if(!j->has_value){j->has_value=true;j->value=target;j->rate=0.0;return target;}
    if(!isfinite(max_rate)||!isfinite(max_accel))return j->value;
    /* Rate/acceleration are explicit control authority. Zero authority means
       hold the current command; inventing a small floor makes a disabled
       actuator move. Clear the shaping state so authority restoration starts
       from rest rather than from a stale rate that exceeded the new envelope. */
    if(!(max_rate>0.0)||!(max_accel>0.0)){j->rate=0.0;return j->value;}
    double error=target-j->value;
    double stopping=sqrt(2.0*max_accel*fabs(error));
    double desired=copysign(fmin(max_rate,fmin(stopping,fabs(error)/dt)),error);
    /* Limit velocity changes, including when a phase lowers max_rate or the
       target reverses. Snapping to the target would bypass acceleration limits. */
    j->rate+=clampd(desired-j->rate,-max_accel*dt,max_accel*dt);
    /* A measured or previous-phase rate can already exceed the new envelope.
       Acceleration limiting alone would leak that excess into the next
       command sample, violating the explicit rate cap. Clamp before applying
       the position step so the exported target is hard-bounded every frame. */
    j->rate=clampd(j->rate,-max_rate,max_rate);
    j->value+=j->rate*dt;
    return j->value;
}
double jerk_angle_update(JerkLimiter *j,double target,double max_rate,double max_accel,double dt) {
    double norm=norm_deg(target);if(!j->has_value){j->has_value=true;j->value=norm;j->rate=0;return norm;}
    double error=norm_signed_deg(norm-j->value),next=jerk_update(j,j->value+error,max_rate,max_accel,dt);j->value=norm_deg(next);return j->value;
}
double burn_fraction(double elapsed,double remaining,double max_accel,double ramp) {
    if (remaining<=GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS) return 0;
    ramp=fmax(0.5,ramp);double x=clampd(elapsed/ramp,0,1),smooth=x*x*(3-2*x),tdv=fmax(1,max_accel*1.25);
    double taper=remaining>=tdv?1:fmax(0.18,sqrt(fmax(0,remaining)/tdv));return clampd(fmin(smooth,taper),0,1);
}
double burn_estimated_duration(double dv,double max_accel,double ramp) {
    if(dv<=0) return 0;
    double acc=fmax(0.05,max_accel),remaining=dv,elapsed=0,step=.025;
    while(remaining>GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS&&elapsed<300){remaining=fmax(0,remaining-acc*burn_fraction(elapsed,remaining,acc,ramp)*step);elapsed+=step;}return elapsed;
}

FlightControlRegime flight_control_regime(double mach,double true_air_speed,double dynamic_pressure,double minimum_safe_speed) {
    /* Guidance shapes desired attitude, while the Python direct controller
       learns actual angular authority from the vehicle. Do not impose a second
       hard-coded Mach/q control law here: that previously made the same bank
       command behave differently across arbitrary pressure thresholds and
       caused predictor/live response mismatch. Only preserve a continuous
       low-speed margin near the calibrated safe-speed boundary, where asking
       for large attitude rates is intrinsically unsafe regardless of actuator
       authority. */
    (void)mach;(void)dynamic_pressure;
    double speed_ratio=true_air_speed/fmax(minimum_safe_speed,1.0);
    double x=clampd((speed_ratio-.95)/1.05,0,1);
    double low_speed=.45+.55*x*x*(3-2*x);
    FlightControlRegime r;
    r.command_rate_scale=low_speed;
    r.command_accel_scale=sqrt(low_speed);
    r.response_scale=low_speed;
    return r;
}

void speedbrake_controller_reset(SpeedbrakeController *c,bool deployed){
    if(!c) return;
    c->integral=0;c->deployed=deployed;c->initialized=true;
}

bool speedbrake_controller_update(SpeedbrakeController *c,double dynamic_pressure,double target_dynamic_pressure,double specific_energy_error,double energy_scale,double maximum_dynamic_pressure,bool inhibit,double dt){
    if(!c)return false;
    if(!c->initialized)speedbrake_controller_reset(c,false);
    if(inhibit||!isfinite(dynamic_pressure)||!isfinite(target_dynamic_pressure)||target_dynamic_pressure<=1||!isfinite(specific_energy_error)||!isfinite(energy_scale)||energy_scale<=1){
        c->integral*=exp(-fmax(0,dt)/2.0);c->deployed=false;return false;
    }
    /* The Shuttle TAEM speedbrake was essentially q-bar PI with an energy
       term. KSP exposes the brake as a binary action group, so keep the same
       state variables and use a Schmitt trigger instead of pretending that a
       continuously commanded surface angle exists. All terms are normalized
       by the current reference state; no vehicle aerodynamic constants live
       in this controller. */
    double qerr=(dynamic_pressure-target_dynamic_pressure)/fmax(target_dynamic_pressure,1.0);
    double eerr=specific_energy_error/fmax(energy_scale,1.0);
    qerr=clampd(qerr,-2.0,2.0);eerr=clampd(eerr,-2.0,2.0);
    if(dt>0&&isfinite(dt)){
        c->integral+=clampd(qerr,-1.0,1.0)*dt/6.0;
        c->integral=clampd(c->integral,-1.2,1.2);
    }
    double drive=qerr+.38*eerr+.22*c->integral;
    /* Generic KSP airbrakes can create a much larger load increment than the
       Orbiter's scheduled speedbrake. Do not command deployment while already
       at the structural q limit; energy is instead recovered with the path/AoA
       channels until q has margin again. */
    if(maximum_dynamic_pressure>0.0&&dynamic_pressure>=maximum_dynamic_pressure){
        c->deployed=false;return false;
    }
    if(c->deployed){if(drive<-.08)c->deployed=false;}
    else if(drive>.08)c->deployed=true;
    return c->deployed;
}

Vector3 planet_rotation_vector(const PlanetModel *p){return vscale(p->north_axis,p->rotational_speed);}
double planet_surface_gravity(const PlanetModel *p){
    if(!p||!isfinite(p->radius)||!(p->radius>0.0)||
       !isfinite(p->gravitational_parameter)||!(p->gravitational_parameter>0.0))return NAN;
    return p->gravitational_parameter/(p->radius*p->radius);
}

static double atmosphere_curve_sample(const PlanetModel *p,const double *values,double altitude,bool logarithmic){
    if(!p||!values||p->atmosphere_sample_count<2)return 0;
    size_t count=p->atmosphere_sample_count;
    if(count>LANDER_ATMOSPHERE_SAMPLE_MAX)count=LANDER_ATMOSPHERE_SAMPLE_MAX;
    double h=fmax(0,altitude);
    if(h<=p->atmosphere_altitude[0])return fmax(0,values[0]);
    if(h>=p->atmosphere_altitude[count-1])return fmax(0,values[count-1]);

    /* The live table is strictly altitude-sorted. Binary search matters here:
       one entry propagation evaluates the atmosphere thousands of times, and
       the old linear scan walked up to 141 samples for every RK4 stage. */
    size_t lo=0,hi=count-1;
    while(hi-lo>1){
        size_t mid=lo+(hi-lo)/2;
        if(h<=p->atmosphere_altitude[mid])hi=mid;else lo=mid;
    }
    double a=p->atmosphere_altitude[lo],b=p->atmosphere_altitude[hi];
    double altitude_span=b-a;
    /* Atmosphere tables are a strictly increasing altitude function. A
       non-positive span is malformed model data, not a reason to invent a
       length-scale epsilon that changes the interpolation physics. */
    if(!(altitude_span>0.0)||!isfinite(altitude_span))return NAN;
    double f=clampd((h-a)/altitude_span,0.0,1.0);
    double av=fmax(0,values[lo]),bv=fmax(0,values[hi]);
    if(logarithmic&&av>0&&bv>0)return exp(log(av)+f*(log(bv)-log(av)));
    return av+(bv-av)*f;
}

double planet_atmospheric_density(const PlanetModel *p,double altitude){
    if(!p||!isfinite(altitude)||!isfinite(p->atmosphere_depth)||p->atmosphere_depth<0.0)
        return NAN;
    if(altitude>=p->atmosphere_depth)return 0.0;
    if(p->atmosphere_sample_count<2)return NAN; /* decision-literal: mathematical-numerical-requirement | an atmospheric interpolation model requires at least two profile samples */
    return atmosphere_curve_sample(p,p->atmosphere_density,fmax(0.0,altitude),true);
}

double planet_atmospheric_pressure(const PlanetModel *p,double altitude){
    if(!p||!isfinite(altitude)||!isfinite(p->atmosphere_depth)||p->atmosphere_depth<0.0)
        return NAN;
    if(altitude>=p->atmosphere_depth)return 0.0;
    if(p->atmosphere_sample_count<2)return NAN; /* decision-literal: mathematical-numerical-requirement | an atmospheric interpolation model requires at least two profile samples */
    return atmosphere_curve_sample(p,p->atmosphere_pressure,fmax(0.0,altitude),true);
}

double planet_atmospheric_speed_of_sound(const PlanetModel *p,double altitude){
    if(!p||!isfinite(altitude)||!isfinite(p->atmosphere_depth)||p->atmosphere_depth<0.0)
        return NAN;
    if(altitude>=p->atmosphere_depth)return 0.0;
    double pressure=planet_atmospheric_pressure(p,altitude);
    double density=planet_atmospheric_density(p,altitude);
    double gamma=p->atmosphere_adiabatic_index;
    /* For an ideal gas a^2 = gamma*p/rho. Missing profile data or gamma is
       missing physics, not permission to inject a nominal atmosphere. */
    if(!(pressure>0.0)||!(density>0.0)||!isfinite(gamma)||!(gamma>1.0))return NAN;
    return sqrt(gamma*pressure/density);
}

HACGuidance hac_guidance_compute_radius(GeoPoint current,double true_air_speed,double course,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity,double hac_radius){
    hac_radius=fmax(1000,hac_radius);
    double h=site->runway_heading*DEG2RAD,ae=sin(h),an=cos(h),re=cos(h),rn=-sin(h);
    double fe=-ae*s->final_approach_distance,fn=-an*s->final_approach_distance;
    double ce=fe+re*side*hac_radius,cn=fn+rn*side*hac_radius;
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double e,n;local_offsets(sp,current,radius,&e,&n);
    double ca=atan2(n-cn,e-ce),fa=atan2(fn-cn,fe-ce),orient=-side;
    double configured=s->hac_look_ahead_angle*DEG2RAD;
    double speed=clampd(true_air_speed*7/fmax(hac_radius,1),6*DEG2RAD,34*DEG2RAD);
    double look=fmax(configured*.65,speed),ta=ca+orient*look;
    /* The look-ahead point must lie on the same dynamic HAC circle used for
       its center, radial error and arc length. Mixing the configured 12 km
       radius here with a 50-90 km aerodynamic HAC made the terminal geometry
       internally inconsistent: radial capture could look good while heading
       guidance was aiming at a completely different circle. */
    double te=ce+cos(ta)*hac_radius,tn=cn+sin(ta)*hac_radius;
    double target=norm_deg(atan2(te-e,tn-n)*RAD2DEG);
    double raw=orient>0?fmod(fa-ca+2*LANDER_PI,2*LANDER_PI):fmod(ca-fa+2*LANDER_PI,2*LANDER_PI);
    double arc=raw*hac_radius,actual=hypot(e-ce,n-cn),radial=actual-hac_radius;
    double capture=fmin(fabs(radial),hac_radius*1.5);
    double final_alt=s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);
    double desired=site->altitude+final_alt+(arc+capture*.65)*tan(s->taem_glide_slope*DEG2RAD);
    /* Pure pursuit of an actual point on the trajectory. The chord length
       supplies both nominal curvature and position feedback, without adding
       a second (and potentially conflicting) circle-turn command. */
    double eta=norm_signed_deg(target-course)*DEG2RAD;
    double chord=fmax(hypot(te-e,tn-n),1.0);
    double lateral=2*true_air_speed*true_air_speed/chord*
        sin(clampd(eta,-LANDER_PI*.5,LANDER_PI*.5));
    /* Capture qualification measures tangent alignment, not waypoint bearing:
       a perfectly tracked circle still has a nonzero pursuit bearing. */
    double tangent=norm_deg(atan2(-orient*sin(ca),orient*cos(ca))*RAD2DEG);
    double err=norm_signed_deg(tangent-course);
    HACGuidance g={target,clampd(atan2(lateral,fmax(gravity,.01))*RAD2DEG,-70,70),
        desired,hypot(e-fe,n-fn),arc,radial,err,ca,lateral};
    return g;
}

HACGuidance hac_guidance_compute(GeoPoint current,double true_air_speed,double course,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity){
    return hac_guidance_compute_radius(current,true_air_speed,course,site,s,radius,side,gravity,s->hac_radius);
}

double hac_guidance_score_radius(GeoPoint current,double true_air_speed,double course,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity,double hac_radius){
    HACGuidance g=hac_guidance_compute_radius(current,true_air_speed,course,site,s,radius,side,gravity,hac_radius);
    return fabs(g.radial_error)/fmax(hac_radius,1)*2.4+fabs(g.course_error)/45*1.4+g.arc_remaining/fmax(hac_radius*300*DEG2RAD,1)*.45+fmin(g.distance_final/fmax(hac_radius*3,1),2)*.15;
}

bool hac_entry_capture_geometry_ready(const HACGuidance*g,double hac_radius){
    if(!g||!isfinite(hac_radius)||hac_radius<=0||!isfinite(g->radial_error)||!isfinite(g->course_error))return false;
    /* Entry may hand the shuttle to TAEM before it is perfectly established
       on the HAC, but the vehicle must already be inside a physically
       capturable tube around the dynamic circle. Range-to-KSC alone is not a
       capture criterion: with a large dynamic HAC it can admit a wings-level
       flyby tens of kilometres outside the circle and let MPC postpone the
       S-turn indefinitely. TAEM then tightens this to the 10%/24 deg capture
       condition used while flying the arc. */
    double radial_limit=fmax(500.0,fmin(5000.0,hac_radius*.30));
    return fabs(g->radial_error)<=radial_limit&&fabs(g->course_error)<=45.0;
}

double hac_guidance_score(GeoPoint current,double true_air_speed,double course,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity){
    return hac_guidance_score_radius(current,true_air_speed,course,site,s,radius,side,gravity,s->hac_radius);
}


bool taem_alignment_maneuver_geometry(double altitude,double radius,
        double minimum_turn_radius,double alignment_turn_deg,const LandingSite*site,
        const GuidanceSettings*s,double*out_remaining,double*out_slope,
        double*out_final_altitude){
    if(!site||!s||!isfinite(altitude)||!isfinite(radius)||
       !isfinite(minimum_turn_radius)||!isfinite(alignment_turn_deg)||
       !(radius>0.0)||!(minimum_turn_radius>0.0)||minimum_turn_radius>radius)
        return false;

    double turn_angle=fabs(alignment_turn_deg)*DEG2RAD;
    double path=turn_angle*radius+s->final_approach_distance;
    if(!(path>0.0)||!isfinite(path))return false;

    double final_alt=site->altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);
    double height=altitude-final_alt;
    if(!(height>0.0)||!isfinite(height))return false;

    double slope=atan2(height,path)*RAD2DEG;
    if(!(slope>0.0)||!(slope<90.0))return false; /* decision-literal: mathematical-numerical-requirement | a finite descending path angle lies strictly between zero and ninety degrees */

    if(out_remaining)*out_remaining=path;
    if(out_slope)*out_slope=slope;
    if(out_final_altitude)*out_final_altitude=final_alt;
    return true;
}

bool taem_alignment_maneuver_feasible(double altitude,double speed,double radius,
        double minimum_turn_radius,double drag,double alignment_turn_deg,
        const PlanetModel*p,const LandingSite*site,const VehicleProfile*v,
        const GuidanceSettings*s,double*out_remaining,double*out_slope){
    if(!p||!site||!v||!s||!isfinite(speed)||!isfinite(drag)||drag<0.0||
       speed<v->minimum_safe_speed)
        return false;

    double path=0.0,slope=0.0,final_alt=0.0;
    if(!taem_alignment_maneuver_geometry(altitude,radius,minimum_turn_radius,
        alignment_turn_deg,site,s,&path,&slope,&final_alt))
        return false;

    double target=fmax(v->final_approach_speed,v->minimum_safe_speed);
    double energy=entry_remaining_specific_energy(site->latitude,altitude,speed,
        site->latitude,final_alt,target,p);
    double height=altitude-final_alt;
    double air_path=hypot(path,height);
    double loss=drag*air_path;
    if(!isfinite(energy)||!isfinite(loss)||energy<loss)return false;

    if(out_remaining)*out_remaining=path;
    if(out_slope)*out_slope=slope;
    return true;
}

bool terminal_approach_valid(GeoPoint current,double range_to_site,double runway_along,double runway_cross,double course,double true_air_speed,double horizontal_speed,double vertical_speed,double flight_path_angle,const LandingSite*site,const VehicleProfile*v,const GuidanceSettings*s){
    double distance=fmax(0,-runway_along);
    double desired=site->altitude+distance*tan(s->final_glide_slope*DEG2RAD);
    double sink=-horizontal_speed*tan(s->final_glide_slope*DEG2RAD);
    double altitude_tolerance=fmax(180.0,distance*.08);
    double sink_tolerance=fmax(15.0,fabs(sink)*.35);
    double speed_ceiling=fmax(v->final_approach_speed*2.0,v->minimum_safe_speed*2.4);
    double shallow_limit=-fmax(3.0,s->final_glide_slope-8.0);
    return isfinite(range_to_site)&&range_to_site<s->final_approach_distance*1.35&&
        runway_along>=-s->final_approach_distance*1.22&&runway_along<site->runway_length&&
        fabs(runway_cross)<fmax(site->runway_width*.5,fmin(400,distance*.07))&&
        fabs(norm_signed_deg(site->runway_heading-course))<12&&
        true_air_speed>=v->minimum_safe_speed&&true_air_speed<=speed_ceiling&&
        fabs(current.altitude-desired)<altitude_tolerance&&
        flight_path_angle>-(s->final_glide_slope+8.0)&&flight_path_angle<shallow_limit&&
        fabs(vertical_speed-sink)<sink_tolerance;
}

double entry_guidance_start_altitude(const PlanetModel*p,const GuidanceSettings*s){
    if(!p||!s)return 0;
    return fmax(s->taem_interface_altitude+1000.0,
                p->atmosphere_depth-s->entry_interface_altitude_margin-500.0);
}

void entry_taem_handoff_altitude_bounds(const GuidanceSettings*s,double*minimum,double*maximum){
    double lo=s&&isfinite(s->mm305_min_altitude)&&s->mm305_min_altitude>0.0?
        s->mm305_min_altitude:15000.0;
    double hi=s&&isfinite(s->mm305_max_altitude)&&s->mm305_max_altitude>=lo?
        s->mm305_max_altitude:25000.0;
    if(minimum)*minimum=lo;
    if(maximum)*maximum=hi;
}

bool entry_taem_handoff_geometry_ready(double altitude,double runway_along_track,double vertical_speed,double horizontal_speed,const GuidanceSettings*s){
    if(!s||!isfinite(altitude)||!isfinite(runway_along_track)||!isfinite(vertical_speed)||!isfinite(horizontal_speed))return false;
    double minimum=0.0,maximum=0.0;
    entry_taem_handoff_altitude_bounds(s,&minimum,&maximum);
    double station_along=NAN,station_cross=NAN;
    mm304_handoff_station(s,&station_along,&station_cross);
    (void)station_cross;
    double station_lead=clampd(fabs(horizontal_speed)*2.0,180.0,2400.0);
    return altitude>=minimum&&altitude<=maximum&&
        runway_along_track<station_along-station_lead&&vertical_speed<0.0;
}

bool entry_s_turn_bank_authority_available(double dynamic_pressure,double true_air_speed,
        double stall_fraction,double g_force,const VehicleProfile*v){
    if(!v||!isfinite(dynamic_pressure)||!isfinite(true_air_speed)||
       !isfinite(stall_fraction)||!isfinite(g_force))return false;
    return dynamic_pressure>DBL_MIN&&
        true_air_speed>=v->minimum_safe_speed&&
        dynamic_pressure<=v->maximum_dynamic_pressure&&
        g_force<=v->maximum_g_load&&
        stall_fraction<1.0;
}

double entry_bank_authority_limit(double true_air_speed,double dynamic_pressure,double g_force,
        const VehicleProfile*v,double maximum_bank){
    if(!v||!isfinite(true_air_speed)||!isfinite(dynamic_pressure)||!isfinite(g_force)||
       !isfinite(maximum_bank))return 0.0;
    if(dynamic_pressure<=DBL_MIN||true_air_speed<v->minimum_safe_speed||
       dynamic_pressure>v->maximum_dynamic_pressure||g_force>v->maximum_g_load)
        return 0.0;
    return fmin(fabs(maximum_bank),v->maximum_bank_angle);
}

double entry_taem_range_target(const PlanetModel*p,const GuidanceSettings*s){
    if(!p||!s)return 0;
    (void)p;
    /*
     * The high-energy TAEM/MM305 interface is an acquisition boundary, not a
     * reserve for the largest mathematically searchable HAC.  Expanding it with
     * body radius made the KSP shuttle enter MM305 tens of kilometres too early
     * and encouraged the terminal planner to solve energy with enormous circles.
     * Keep the configured interface range, but never place it inside the nominal
     * terminal geometry needed for one HAC plus final alignment.
     */
    double terminal_reserve=fmax(s->final_approach_distance*1.5,
        s->final_approach_distance+fmax(0.0,s->hac_radius));
    return fmax(s->taem_interface_range,terminal_reserve);
}

bool mm305_acquisition_ready(const Telemetry*t,const PlanetModel*p,
        const LandingConfiguration*cfg){
    if(!t||!p||!cfg)return false;
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    if(!isfinite(t->mean_altitude)||!isfinite(t->true_air_speed)||
       !isfinite(t->vertical_speed)||!isfinite(t->range_to_site)||
       !isfinite(t->dynamic_pressure)||!isfinite(t->g_force))
        return false;

    /*
     * MM305 owns high-energy TAEM path acquisition.  Do not require the orbiter
     * to reach the later rear-alignment/HAC station before giving it ownership:
     * that destroys the acquisition segment by definition.  Admission instead
     * uses the measured energy band, structural state, remaining terminal range,
     * and whether the current velocity vector is actually closing on KSC.
     */
    double sound=planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double mach=isfinite(t->mach)&&t->mach>0.0?t->mach:
        (isfinite(sound)&&sound>DBL_MIN?t->true_air_speed/sound:NAN);
    double mach_min=s->mm305_target_mach-s->mm305_mach_half_width;
    double mach_max=s->mm305_target_mach+s->mm305_mach_half_width;
    if(isfinite(s->hac_acquisition_mach)&&s->hac_acquisition_mach>0.0)
        mach_min=fmin(mach_min,s->hac_acquisition_mach);
    double min_altitude=s->mm305_min_altitude;
    if(isfinite(s->hac_acquisition_altitude)&&s->hac_acquisition_altitude>0.0)
        min_altitude=fmin(min_altitude,s->hac_acquisition_altitude);

    double maximum_range=entry_taem_range_target(p,s);
    double minimum_range=fmax(s->final_approach_distance*1.5,
        s->final_approach_distance+fmax(0.0,s->hac_radius));
    double closure=isfinite(t->course_to_site_error)?
        cos(t->course_to_site_error*DEG2RAD):NAN;

    bool structural=t->dynamic_pressure<=v->maximum_dynamic_pressure&&
        t->g_force<=v->maximum_g_load&&
        (!t->stall_fraction_is_measured||
         (isfinite(t->stall_fraction)&&t->stall_fraction<1.0));
    bool acquisition_region=isfinite(maximum_range)&&maximum_range>minimum_range&&
        t->range_to_site>=minimum_range&&t->range_to_site<=maximum_range&&
        isfinite(closure)&&closure>0.0;

    return t->mean_altitude>=min_altitude&&
        t->mean_altitude<=s->mm305_max_altitude&&
        isfinite(mach)&&mach>=mach_min&&mach<=mach_max&&
        t->vertical_speed<0.0&&t->true_air_speed>=v->minimum_safe_speed&&
        acquisition_region&&structural;
}
double rotating_specific_energy(double latitude,double altitude,double air_relative_speed,const PlanetModel*p){
    if(!p||!isfinite(latitude)||!isfinite(altitude)||!isfinite(air_relative_speed)||
       !isfinite(p->radius)||!(p->radius>0.0)||
       !isfinite(p->gravitational_parameter)||!(p->gravitational_parameter>0.0)||
       !isfinite(p->rotational_speed))return NAN;
    double radius=p->radius+altitude;
    /* Radius is a physical domain boundary. Clamping an invalid state to an
       arbitrary one-metre radius manufactures enormous but finite energy. */
    if(!isfinite(radius)||!(radius>0.0))return NAN;
    double transverse=radius*cos(latitude*DEG2RAD);
    double centrifugal=.5*p->rotational_speed*p->rotational_speed*transverse*transverse;
    return .5*air_relative_speed*air_relative_speed-p->gravitational_parameter/radius-centrifugal;
}

double entry_remaining_specific_energy(double latitude,double altitude,double air_relative_speed,double target_latitude,double target_altitude,double target_speed,const PlanetModel*p){
    double current=rotating_specific_energy(latitude,altitude,air_relative_speed,p);
    double target=rotating_specific_energy(target_latitude,target_altitude,target_speed,p);
    return current-target;
}

double entry_altitude_target_for_speed(double reference_altitude,double reference_speed,
        double true_air_speed,double target_altitude,double target_speed){
    if(!isfinite(reference_altitude)||!isfinite(reference_speed)||
       !isfinite(true_air_speed)||!isfinite(target_altitude)||!isfinite(target_speed))
        return target_altitude;
    double span=reference_speed-target_speed;
    /* Any finite positive speed span defines the schedule. The previous
       one-metre-per-second cutoff was a behavioral threshold with no
       numerical or physical basis. */
    if(!isfinite(span)||!(span>0.0))return target_altitude;
    double remaining=clampd((true_air_speed-target_speed)/span,0.0,1.0);
    /* Smoothly map the S-turn's speed bleed into an altitude schedule.  The
       profile starts exactly at the measured entry/restart state and ends at
       the configured TAEM altitude when the speed-handoff threshold is
       reached.  Using speed rather than elapsed time makes the target adapt to
       the live drag history instead of assuming a fixed atmospheric clock. */
    double smooth=remaining*remaining*(3.0-2.0*remaining);
    return target_altitude+(reference_altitude-target_altitude)*smooth;
}

double entry_altitude_target_for_range(double reference_altitude,double reference_range,
        double range,double target_altitude,double target_range){
    if(!isfinite(reference_altitude)||!isfinite(reference_range)||!isfinite(range)||
       !isfinite(target_altitude)||!isfinite(target_range))return target_altitude;
    double span=reference_range-target_range;
    /* Any finite positive range span defines a geometric interpolation.
       Resolution belongs to the caller/sensor model, not an absolute 1 m
       hidden gate inside this unit-preserving mapping. */
    if(!isfinite(span)||!(span>0.0))return target_altitude;
    /* Range is the hard geometric resource.  A linear altitude-vs-range
       corridor represents a constant average descent angle from the measured
       Entry state to the TAEM interface.  Unlike a smoothstep it begins
       spending altitude immediately, so a hot shuttle cannot consume the
       first half of its downrange while remaining near entry altitude. */
    double remaining=clampd((range-target_range)/span,0.0,1.0);
    return target_altitude+(reference_altitude-target_altitude)*remaining;
}


double entry_thermal_protection_aoa_floor(const VehicleProfile*v){
    if(!v)return 0.0;
    /* entry_angle_of_attack is the configured belly-first Entry incidence. It is
       therefore the nominal thermal-protection floor, not a value that altitude
       capture may unload below merely to steepen the trajectory. */
    return clampd(v->entry_angle_of_attack,0.0,v->maximum_angle_of_attack);
}

double entry_low_q_protective_aoa_floor(double dynamic_pressure,const VehicleProfile*v){
    (void)dynamic_pressure;
    return entry_thermal_protection_aoa_floor(v);
}

double entry_terminal_turn_aoa_floor(double true_air_speed,double dynamic_pressure,
        const VehicleProfile*v){
    (void)true_air_speed;
    (void)dynamic_pressure;
    return entry_thermal_protection_aoa_floor(v);
}

double entry_final_s_turn_aoa_ceiling(const VehicleProfile*v){
    /* The mission explicitly authorizes about 35 deg only for the final MM304
       S-turn/terminal-course turn. Keep maximum_angle_of_attack as the ordinary
       envelope everywhere else; callers must opt into this extension explicitly. */
    double nominal=v&&isfinite(v->maximum_angle_of_attack)?v->maximum_angle_of_attack:28.0;
    return fmax(nominal,35.0);
}



EntryTerminalDemand entry_terminal_demand(double latitude,double altitude,double range,
        double horizontal_speed,double course_error,double vertical_speed,
        double true_air_speed,double entry_reference_speed,const PlanetModel*p,
        const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s){
    EntryTerminalDemand d={0};
    if(!p||!v||!site||!s)return d;
    (void)latitude;
    (void)true_air_speed;
    (void)entry_reference_speed;

    double height=fmax(0.0,altitude-s->taem_interface_altitude);
    double sink=fmax(0.0,-vertical_speed);
    double radius=fmax(DBL_MIN,p->radius+altitude);
    double gravity=p->gravitational_parameter/(radius*radius);
    double curvature_accel=horizontal_speed*horizontal_speed/radius;
    double down_accel=fmax(0.0,gravity-curvature_accel);

    /*
     * Solve h = sink*t + 0.5*a*t^2 directly.  This is only a kinematic
     * projection; no early/late weighting or normalized score is allowed to
     * influence the range prediction.
     */
    double time_to_taem=0.0;
    if(height>0.0){
        if(down_accel>DBL_EPSILON)
            time_to_taem=(sqrt(sink*sink+2.0*down_accel*height)-sink)/
                down_accel;
        else if(sink>DBL_EPSILON)
            time_to_taem=height/sink;
        else
            time_to_taem=INFINITY;
    }

    double closing_speed=horizontal_speed*
        cos(norm_signed_deg(course_error)*DEG2RAD);
    double projected_remaining=isfinite(time_to_taem)?
        fmax(0.0,range)-closing_speed*time_to_taem:fmax(0.0,range);
    d.projected_taem_range_error=
        projected_remaining-entry_taem_range_target(p,s);
    return d;
}

typedef struct {
    double x,y,in_tangent,out_tangent;
} StockKspCurveKey;

static double stock_ksp_curve_eval(const StockKspCurveKey*keys,size_t count,double x){
    if(!keys||count==0||!isfinite(x))return NAN;
    if(x<=keys[0].x)return keys[0].y;
    if(x>=keys[count-1].x)return keys[count-1].y;
    for(size_t i=0;i+1<count;i++){
        const StockKspCurveKey*a=&keys[i],*b=&keys[i+1];
        if(x>b->x)continue;
        double h=b->x-a->x;
        if(!(h>DBL_MIN))return a->y;
        double u=clampd((x-a->x)/h,0.0,1.0);
        double u2=u*u,u3=u2*u;
        double h00=2.0*u3-3.0*u2+1.0;
        double h10=u3-2.0*u2+u;
        double h01=-2.0*u3+3.0*u2;
        double h11=u3-u2;
        return h00*a->y+h10*h*a->out_tangent+
            h01*b->y+h11*h*b->in_tangent;
    }
    return keys[count-1].y;
}

static void stock_ksp_lifting_surface_coefficients(double mach,double angle_of_attack,
        double*cl,double*cd){
    /*
     * Exact stock KSP 1.12 Physics.cfg LIFTING_SURFACE_CURVES/Default keys.
     * KSP evaluates the lift/drag curves with sin(AoA), then multiplies by
     * the corresponding Mach curve (and later by q, area, and the global
     * lift/lift-drag multipliers).  Curve interpolation is Unity's cubic
     * Hermite AnimationCurve interpolation using the stored key tangents.
     *
     * Whole-vessel drag cubes and body lift are deliberately not fabricated
     * here.  VesselPhysicsModel/live force samples remain the primary source
     * for those effects; this function is the physically faithful stock
     * lifting-surface cold-start prior.
     */
    static const StockKspCurveKey lift_curve[]={
        {0.0,0.0,0.0,1.965926},
        {0.258819,0.5114774,1.990092,1.905806},
        {0.5,0.9026583,0.7074468,-0.7074468},
        {0.7071068,0.5926583,-2.087948,-1.990095},
        {1.0,0.0,-2.014386,-2.014386}
    };
    static const StockKspCurveKey lift_mach[]={
        {0.0,1.0,0.0,0.0},
        {0.3,0.5,-1.671345,-0.8273422},
        {1.0,0.125,-0.0005291355,-0.02625772},
        {5.0,0.0625,0.0,0.0},
        {25.0,0.05,0.0,0.0}
    };
    static const StockKspCurveKey drag_curve[]={
        {0.0,0.01,0.0,0.0},
        {0.3420201,0.06,0.1750731,0.1750731},
        {0.5,0.24,2.60928,2.60928},
        {0.7071068,1.7,3.349777,3.349777},
        {1.0,2.4,1.387938,0.0}
    };
    static const StockKspCurveKey drag_mach[]={
        {0.0,0.35,0.0,-0.8463008},
        {0.15,0.125,0.0,0.0},
        {0.9,0.275,0.541598,0.541598},
        {1.1,0.75,0.0,0.0},
        {1.4,0.4,-0.3626955,-0.3626955},
        {1.6,0.35,-0.1545923,-0.1545923},
        {2.0,0.3,-0.09013031,-0.09013031},
        {5.0,0.22,0.0,0.0},
        {25.0,0.3,0.0006807274,0.0}
    };

    double signed_aoa=isfinite(angle_of_attack)?angle_of_attack:0.0;
    double flow=sin(clampd(fabs(signed_aoa),0.0,90.0)*DEG2RAD);
    double m=fmax(0.0,isfinite(mach)?mach:0.0);
    double lift=stock_ksp_curve_eval(lift_curve,
        sizeof(lift_curve)/sizeof(lift_curve[0]),flow)*
        stock_ksp_curve_eval(lift_mach,
        sizeof(lift_mach)/sizeof(lift_mach[0]),m);
    double drag=stock_ksp_curve_eval(drag_curve,
        sizeof(drag_curve)/sizeof(drag_curve[0]),flow)*
        stock_ksp_curve_eval(drag_mach,
        sizeof(drag_mach)/sizeof(drag_mach[0]),m);
    if(cl)*cl=copysign(fmax(0.0,lift),signed_aoa);
    if(cd)*cd=fmax(0.0,drag);
}

void aerodynamic_force_factors_mach(double mach,double angle_of_attack,const VehicleProfile*v,double*lift_factor,double*drag_factor){
    /*
     * VehicleProfile L/D and ballistic coefficient are defined at the existing
     * Mach-5 entry-trim reference.  Preserve that contract while using stock
     * KSP's real lifting-surface curves for variation with AoA and Mach.
     * Live/persisted whole-vessel force samples supersede this prior whenever
     * VesselPhysicsModel has support at the requested state.
     */
    double reference_aoa=v&&isfinite(v->entry_angle_of_attack)?
        fabs(v->entry_angle_of_attack):0.0;
    double raw_lift=0.0,raw_drag=0.0,ref_lift=0.0,ref_drag=0.0;
    stock_ksp_lifting_surface_coefficients(mach,angle_of_attack,
        &raw_lift,&raw_drag);
    stock_ksp_lifting_surface_coefficients(5.0,reference_aoa,
        &ref_lift,&ref_drag); /* Physics.cfg Mach-5 anchor / baseline contract. */

    double lf=fabs(ref_lift)>DBL_MIN?raw_lift/fabs(ref_lift):raw_lift;
    double df=ref_drag>DBL_MIN?raw_drag/ref_drag:raw_drag;
    if(lift_factor)*lift_factor=isfinite(lf)?lf:0.0;
    if(drag_factor)*drag_factor=isfinite(df)&&df>DBL_MIN?df:DBL_MIN;
}


double aerodynamic_best_glide_aoa(double mach,const VehicleProfile*v){
    if(!v||!isfinite(mach)||!(v->maximum_angle_of_attack>0.0))return 0.0;
    double hi=v->maximum_angle_of_attack;
    if(mach<1.0&&isfinite(v->terminal_maximum_lift_angle_of_attack)&&
       v->terminal_maximum_lift_angle_of_attack>0.0)
        hi=fmin(hi,v->terminal_maximum_lift_angle_of_attack);
    double lo=0.0;
    const double phi=(sqrt(5.0)-1.0)*0.5; /* golden-section ratio */
    double x1=hi-phi*(hi-lo),x2=lo+phi*(hi-lo);
    double l1=0.0,d1=0.0,l2=0.0,d2=0.0;
    aerodynamic_force_factors_mach(mach,x1,v,&l1,&d1);
    aerodynamic_force_factors_mach(mach,x2,v,&l2,&d2);
    double f1=(d1>DBL_MIN&&isfinite(l1)&&isfinite(d1))?fabs(l1)/d1:0.0;
    double f2=(d2>DBL_MIN&&isfinite(l2)&&isfinite(d2))?fabs(l2)/d2:0.0;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,hi);
    unsigned guard=0;
    while(hi-lo>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        if(f1<f2){
            lo=x1;x1=x2;f1=f2;x2=lo+phi*(hi-lo);
            aerodynamic_force_factors_mach(mach,x2,v,&l2,&d2);
            f2=(d2>DBL_MIN&&isfinite(l2)&&isfinite(d2))?fabs(l2)/d2:0.0;
        }else{
            hi=x2;x2=x1;f2=f1;x1=hi-phi*(hi-lo);
            aerodynamic_force_factors_mach(mach,x1,v,&l1,&d1);
            f1=(d1>DBL_MIN&&isfinite(l1)&&isfinite(d1))?fabs(l1)/d1:0.0;
        }
    }
    return clampd(.5*(lo+hi),0.0,v->maximum_angle_of_attack);
}

void aerodynamic_force_factors(double angle_of_attack,const VehicleProfile*v,double*lift_factor,double*drag_factor){
    aerodynamic_force_factors_mach(5.0,angle_of_attack,v,lift_factor,drag_factor);
}
