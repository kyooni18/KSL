/* Native MM305 horizontal geometry. Included by taem_planner.c so prediction,
 * visualization and the live Frenet tracker evaluate exactly the same curve. */
#include "taem_spiral.h"

static Point bezier_eval(const TaemPoint2 *p, int degree, double t) {
    Point q[6];
    for (int i=0;i<=degree;++i) q[i]=(Point){p[i].x,p[i].y};
    for (int n=degree;n>0;--n) for (int i=0;i<n;++i)
        q[i]=(Point){(1-t)*q[i].x+t*q[i+1].x,(1-t)*q[i].y+t*q[i+1].y};
    return q[0];
}

static bool capture_sample(const TaemRoute *r,double t,Point *p,double *h,double *k) {
    TaemPoint2 dp[5],ddp[4];
    for(int i=0;i<5;++i) dp[i]=(TaemPoint2){
        5*(r->capture_control[i+1].x-r->capture_control[i].x),
        5*(r->capture_control[i+1].y-r->capture_control[i].y)};
    for(int i=0;i<4;++i) ddp[i]=(TaemPoint2){4*(dp[i+1].x-dp[i].x),4*(dp[i+1].y-dp[i].y)};
    Point d=bezier_eval(dp,4,t),dd=bezier_eval(ddp,3,t);
    double norm=hypot(d.x,d.y);
    if(!(norm>1e-6)) return false;
    *p=bezier_eval(r->capture_control,5,t);
    *h=atan2(d.y,d.x); *k=(d.x*dd.y-d.y*dd.x)/(norm*norm*norm);
    return isfinite(*k);
}

static double capture_measure(TaemRoute *r,bool lookup) {
    Point previous={0},p; double h,k,peak=0,length=0;
    for(int i=0;i<TAEM_ROUTE_LEAD_LUT_POINTS;++i) {
        if(!capture_sample(r,(double)i/(TAEM_ROUTE_LEAD_LUT_POINTS-1),&p,&h,&k)) return INFINITY;
        peak=fmax(peak,fabs(k));
        if(i) length+=distance(p,previous);
        if(lookup) r->lead_arc_fraction_lut[i]=(float)length;
        previous=p;
    }
    if(lookup) {
        r->lead_length_m=length;
        if(!(length>0)) return INFINITY;
        for(int i=0;i<TAEM_ROUTE_LEAD_LUT_POINTS;++i) r->lead_arc_fraction_lut[i]/=length;
    }
    return peak;
}

static void capture_controls(TaemRoute *r,Point p0,Point p5,
        double h0,double h1,double k0,double k1,double v0,double v1) {
    Point d0={v0*cos(h0),v0*sin(h0)},d1={v1*cos(h1),v1*sin(h1)};
    Point a0={-k0*v0*v0*sin(h0),k0*v0*v0*cos(h0)};
    Point a1={-k1*v1*v1*sin(h1),k1*v1*v1*cos(h1)};
    r->capture_control[0]=(TaemPoint2){p0.x,p0.y};
    r->capture_control[1]=(TaemPoint2){p0.x+d0.x/5,p0.y+d0.y/5};
    r->capture_control[2]=(TaemPoint2){p0.x+2*d0.x/5+a0.x/20,p0.y+2*d0.y/5+a0.y/20};
    r->capture_control[3]=(TaemPoint2){p5.x-2*d1.x/5+a1.x/20,p5.y-2*d1.y/5+a1.y/20};
    r->capture_control[4]=(TaemPoint2){p5.x-d1.x/5,p5.y-d1.y/5};
    r->capture_control[5]=(TaemPoint2){p5.x,p5.y};
}

/* k=k0(1-3u^2+2u^3). Its integral is analytic. At both ends
 * dk/ds=0; at Final k=0. Integrate tangent, not a circular chord. */
static double unload_heading(const TaemRoute *r,double s) {
    double u=clamp(s/r->unload_length_m,0,1);
    return r->spiral_exit_heading_rad+r->side/r->spiral_exit_radius_m*
        r->unload_length_m*(u-u*u*u+.5*u*u*u*u);
}

static Point unload_delta(const TaemRoute *r,double s) {
    static const double x[4]={.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
    static const double w[4]={.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
    Point p={0};
    for(int i=0;i<4;++i) for(int sign=-1;sign<=1;sign+=2) {
        double h=unload_heading(r,.5*s*(1+sign*x[i]));
        p.x+=.5*s*w[i]*cos(h);p.y+=.5*s*w[i]*sin(h);
    }
    return p;
}

static bool route_sample_at_station(const TaemRoute *r,double station,
        Point *p,double *course,double *k) {
    if(!r||!r->valid||!isfinite(station)||!(r->spiral_exit_radius_m>0)) return false;
    station=clamp(station,0,r->length_m);
    double h;
    if(station<r->lead_length_m) {
        double f=station/r->lead_length_m;int i=1;
        while(i<TAEM_ROUTE_LEAD_LUT_POINTS-1&&r->lead_arc_fraction_lut[i]<f) ++i;
        double a=r->lead_arc_fraction_lut[i-1],b=r->lead_arc_fraction_lut[i];
        double t=((i-1)+(f-a)/fmax(b-a,1e-12))/(TAEM_ROUTE_LEAD_LUT_POINTS-1);
        if(!capture_sample(r,t,p,&h,k)) return false;
    } else if(station<r->lead_length_m+r->hac.arc_length_m) {
        double remaining=r->hac.arc_length_m-(station-r->lead_length_m);
        int i=1;
        while(i<TAEM_ROUTE_LEAD_LUT_POINTS-1&&r->spiral_arc_lut[i]<remaining) ++i;
        double a=r->spiral_arc_lut[i-1],b=r->spiral_arc_lut[i];
        double sweep=fabs(r->hac.arc_sweep_rad),step=sweep/(TAEM_ROUTE_LEAD_LUT_POINTS-1);
        double psi=step*((i-1)+(remaining-a)/(b-a));
        TaemSpiralSample q;
        /* Newton refines the LUT seed against the exact arc integral. */
        for(int j=0;j<2;++j) {
            if(!taem_spiral_sample(r->spiral_exit_radius_m,r->spiral_coefficient_m,
                psi,r->side,r->spiral_exit_heading_rad,r->hac.center.x,r->hac.center.y,&q)) return false;
            psi=clamp(psi-(taem_spiral_arc(r->spiral_exit_radius_m,r->spiral_coefficient_m,psi)-remaining)/q.ds_dpsi,0,sweep);
        }
        if(!taem_spiral_sample(r->spiral_exit_radius_m,r->spiral_coefficient_m,
            psi,r->side,r->spiral_exit_heading_rad,r->hac.center.x,r->hac.center.y,&q)) return false;
        *p=(Point){q.x,q.y};h=q.heading_rad;*k=q.curvature;
    } else {
        double s=station-r->lead_length_m-r->hac.arc_length_m;
        double u=clamp(s/r->unload_length_m,0,1);
        Point d=unload_delta(r,fmin(s,r->unload_length_m));
        *p=(Point){r->hac.exit.x+d.x+fmax(0,s-r->unload_length_m),r->hac.exit.y+d.y};
        h=unload_heading(r,s);*k=r->side/r->spiral_exit_radius_m*(1-3*u*u+2*u*u*u);
    }
    *course=r->runway_heading_deg+degrees(h);
    return isfinite(*k)&&isfinite(p->x)&&isfinite(*course);
}

static double route_station(const TaemRoute *r,size_t index) {
    return taem_route_station_at_index(r,index);
}
static Point route_position(const TaemRoute *r,size_t index) {
    Point p={NAN,NAN};double h,k;
    (void)route_sample_at_station(r,route_station(r,index),&p,&h,&k);return p;
}

static bool build_hac(const TerminalModel *m,const TaemGeometryState *start,
        double ri,double side,double sweep,double spacing,double initial_limit,
        double peak_limit,double target_length,TaemRoute *r,char *reason,size_t size) {
    if(reason&&size) snprintf(reason,size,"spiral geometry or acquisition is infeasible");
    if(!m||!start||!r||!(ri>0)||fabs(side)!=1||!(sweep>0)||sweep>8*planner_pi||
        !(spacing>=100)||!(initial_limit>0)||!isfinite(ri+sweep+spacing+initial_limit)) return false;
    memset(r,0,sizeof(*r));
    double vf=fmax(m->guidance.final_alignment_speed,m->vehicle.minimum_safe_speed);
    double vi=start->ground_speed_mps;
    double gravity=m->world.mu_m3_s2/pow(m->world.radius_m+m->site.altitude,2);
    double bank=radians(m->vehicle.maximum_bank_angle);
    if(!(vf>0&&vi>vf&&gravity>0&&bank>0&&bank<planner_pi/2)) return false;
    /* Equal nominal lateral acceleration at the two speed endpoints. The
       native profile/replay must prove the intervening speed/curvature pairing. */
    double rf=fmax(ri*vf*vf/(vi*vi),vf*vf/(gravity*tan(bank)));
    /* Keep r'' <= Rf on partial sweeps. This is a sufficient analytic
       no-inflection bound for the polar curvature numerator, not a fixed
       contraction coefficient. Short captures retain a larger exit radius. */
    rf=fmax(rf,ri/(1+sweep*sweep/12));
    if(!(rf<ri)) return false;
    r->side=side;r->spiral_exit_radius_m=rf;
    r->spiral_coefficient_m=(ri-rf)/pow(sweep,4);
    double wn=m->attitude.roll_wn,zeta=m->attitude.roll_zeta;
    double rate=m->attitude.max_roll_rate_rad_s,accel=m->attitude.max_roll_accel_rad_s2;
    if(!(wn>0&&zeta>0&&rate>0&&accel>0)) return false;
    double exit_bank=atan(vf*vf/(gravity*rf));
    double settle=4/(wn*zeta);
    /* Conservative bounds: |bank'|<=1.5*v*k0/g and
       |bank''|<=6*v^2*k0/(g*L^2), with time derivatives at vf. */
    double unload_time=fmax(settle,fmax(1.5*exit_bank/rate,sqrt(6*exit_bank/accel)));
    r->unload_length_m=vf*unload_time;r->settle_length_m=vf*settle;
    r->rollout_length_m=r->unload_length_m+r->settle_length_m;
    r->spiral_exit_heading_rad=-side*r->unload_length_m/(2*rf);
    r->alignment_along_m=-m->guidance.final_approach_distance;
    Point delta=unload_delta(r,r->unload_length_m);
    r->hac.exit=(TaemPoint2){r->alignment_along_m-r->settle_length_m-delta.x,-delta.y};
    double theta=r->spiral_exit_heading_rad-side*planner_pi/2;
    r->hac.center=(TaemPoint2){r->hac.exit.x-rf*cos(theta),r->hac.exit.y-rf*sin(theta)};
    TaemSpiralSample entry;
    if(!taem_spiral_sample(rf,r->spiral_coefficient_m,sweep,side,r->spiral_exit_heading_rad,
        r->hac.center.x,r->hac.center.y,&entry)) return false;
    r->hac.entry=(TaemPoint2){entry.x,entry.y};r->hac.radius_m=ri;r->hac.side=side;
    r->hac.capture_course_deg=m->site.runway_heading+degrees(entry.heading_rad);
    r->hac.arc_sweep_rad=side*sweep;
    for(int i=0;i<TAEM_ROUTE_LEAD_LUT_POINTS;++i) {
        double psi=sweep*i/(TAEM_ROUTE_LEAD_LUT_POINTS-1);
        TaemSpiralSample q;
        if(!taem_spiral_sample(rf,r->spiral_coefficient_m,psi,side,r->spiral_exit_heading_rad,
            r->hac.center.x,r->hac.center.y,&q)||side*q.curvature<=0) return false;
        r->spiral_arc_lut[i]=taem_spiral_arc(rf,r->spiral_coefficient_m,psi);
    }
    r->hac.arc_length_m=r->spiral_arc_lut[TAEM_ROUTE_LEAD_LUT_POINTS-1];
    Point p0={start->runway_along_m,start->runway_cross_m},p5={entry.x,entry.y};
    double direct=distance(p0,p5),best=INFINITY;
    TaemPoint2 controls[6];bool found=false;
    double k0=start->curvature_right_per_m;
    if(!isfinite(k0)||fabs(k0)>initial_limit) return false;
    for(int i=0;i<18;++i) for(int j=0;j<18;++j) {
        double v0=fmax(spacing,direct*.25)*pow(1.2,i),v1=fmax(spacing,direct*.25)*pow(1.2,j);
        capture_controls(r,p0,p5,radians(start->course_deg-m->site.runway_heading),entry.heading_rad,k0,entry.curvature,v0,v1);
        double peak=capture_measure(r,true);
        if(!isfinite(peak)||peak>peak_limit) continue;
        double score=isfinite(target_length)?fabs(r->lead_length_m-target_length):
            isfinite(peak_limit)?r->lead_length_m:peak;
        if(score<best) { best=score;memcpy(controls,r->capture_control,sizeof(controls));found=true; }
    }
    if(!found) return false;
    memcpy(r->capture_control,controls,sizeof(controls));capture_measure(r,true);
    r->p0_along_m=p0.x;r->p0_cross_m=p0.y;r->p3_along_m=p5.x;r->p3_cross_m=p5.y;
    r->p1_along_m=controls[1].x;r->p1_cross_m=controls[1].y;
    r->p2_along_m=controls[4].x;r->p2_cross_m=controls[4].y;
    r->initial_course_deg=start->course_deg;r->runway_heading_deg=m->site.runway_heading;
    r->lead_count=(size_t)fmax(2,ceil(r->lead_length_m/spacing));
    r->arc_count=(size_t)fmax(3,ceil(r->hac.arc_length_m/spacing));
    r->rollout_count=(size_t)fmax(2,ceil(r->rollout_length_m/spacing));
    r->count=1+r->lead_count+r->arc_count+r->rollout_count;
    if(r->count>TAEM_ROUTE_MAX_POINTS) return false;
    r->length_m=r->lead_length_m+r->hac.arc_length_m+r->rollout_length_m;
    r->hac.total_length_m=r->hac.arc_length_m+r->rollout_length_m;
    r->hac.final_length_m=r->rollout_length_m;
    r->profile_total_length_m=r->length_m;
    r->profile_start_altitude_m=m->site.altitude+start->altitude_above_runway_m;
    r->profile_start_fpa_deg=start->flight_path_angle_deg;
    r->profile_final_altitude_m=m->site.altitude+m->guidance.final_approach_distance*tan(radians(m->guidance.final_glide_slope));
    r->profile_final_slope_deg=m->guidance.final_glide_slope;
    r->profile_local_start_fraction=.08;r->profile_local_peak_fraction=.24;r->profile_local_end_fraction=.40;
    r->valid=true;
    if(reason&&size) snprintf(reason,size,"spiral geometry built; native qualification required");
    return true;
}
