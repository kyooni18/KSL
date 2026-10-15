/* MM305 predictive energy law (port of Tools/sts_taem_energy_proto.py).
 *
 * Runway frame: x along the runway (site at 0, Final station at -D), y to the
 * LEFT of the runway heading, h above the runway, psi heading measured from the
 * runway axis positive to the left.  Positive bank is a right turn.
 *
 * Vertical: drag-modulation level k in [0,1] (0 = maximum range).  Supersonic
 * the incidence is held between the maximum-L/D ceiling and AOA_SUP_MIN;
 * subsonic the flight path tracks a dynamic-pressure reference between Q_LO
 * and Q_HI.  k is solved by bisection on a point-mass forward prediction that
 * flies the remaining planned path (banking on its arcs) until the specific
 * energy reaches the Final station energy.
 * Lateral: straight-in on the extended centreline, or, when even maximum drag
 * leaves excess energy, an overhead loop: centreline past the station to
 * x_turn, a turnaround onto a downwind leg at y = 2R, and a heading-alignment
 * circle of radius R back onto the centreline at the station.  Pattern size
 * comes from the same predictor. */

#define TEL_AOA_SUP_MIN 8.0
#define TEL_AOA_MIN 1.0
#define TEL_Q_LO 3500.0
#define TEL_Q_HI 11000.0
#define TEL_BLEND 8000.0
#define TEL_AIM 9000.0
#define TEL_BANK_MAX 50.0
#define TEL_BANK_SUP 45.0
#define TEL_BANK_TURN 55.0
#define TEL_BANK_ARC 60.0
#define TEL_R_MIN 2200.0
/* floor for resizing a HAC already being flown (bank-limited, not planned) */
#define TEL_R_SPIRAL_MIN 1600.0
#define TEL_R_MAX 10000.0
#define TEL_R_TURN 2600.0
#define TEL_K_PLAN 0.5
#define TEL_K_LOOP 0.25
#define TEL_MODE_FREEZE 8000.0
#define TEL_UPDATE_S 2.0
/* on-final energy shortfall must persist this long before low-energy delivery */
#define TEL_LOW_ENERGY_DWELL_S 3.0
/* drag level at or below which the law is energy-short (flying maximum range) */
#define TEL_K_SHORT 0.05
/* Jet assist for a persistent MM305 energy shortfall.  The throttle scales with
   the measured deficit (J/kg) and never drops below the floor while requested.
   The full-scale deficit is an untuned placeholder pending flight data. */
#define JET_ASSIST_FULL_DEFICIT_J_KG 2000.0
#define JET_ASSIST_MIN_THROTTLE 0.25
#define TEL_PI 3.14159265358979323846
#define TEL_G0 9.81

enum { TEL_LEG_OUT=0, TEL_LEG_TURN, TEL_LEG_DOWN, TEL_LEG_HAC, TEL_LEG_FINAL };

typedef struct {
    double x,y,h,v,g,psi,m,aoa,aoa_rate,bank,ground_speed;
} TelPoint;

typedef struct {
    const TerminalModel *model;
    double d_station,h_station,v_station,e_station,aoa_max,slope_deg;
} TelContext;

typedef struct { double length,radius; } TelSegment;

static double tel_wrap(double a) { return remainder(a,2.0*TEL_PI); }

static double tel_gravity(const TelContext *c,double h) {
    double r=c->model->world.radius_m+c->model->site.altitude+h;
    return c->model->world.mu_m3_s2/(r*r);
}

static void tel_air(const TelContext *c,double h,double v,double *q,double *mach) {
    AtmosphereSample a=world_atmosphere_sample(&c->model->world,c->model->site.altitude+h);
    *q=0.5*a.density_kg_m3*v*v;
    *mach=a.speed_of_sound_mps>1.0?v/a.speed_of_sound_mps:0.0;
}

static void tel_forces(const TelContext *c,const TelPoint *p,double aoa,
        double *q,double *mach,double *lift,double *drag) {
    double cl,cd;
    tel_air(c,p->h,p->v,q,mach);
    aero_coefficients(&c->model->aero,*mach,aoa,&cl,&cd);
    *lift=cl**q*c->model->aero.reference_area_m2;
    *drag=cd**q*c->model->aero.reference_area_m2;
}

static double tel_lift_aoa(const TelContext *c,const TelPoint *p,double lift_needed) {
    double lo=-5.0,hi=c->aoa_max,q,mach,lift,drag;
    for (int i=0;i<30;++i) {
        double mid=0.5*(lo+hi);
        tel_forces(c,p,mid,&q,&mach,&lift,&drag);
        if (lift<lift_needed) lo=mid; else hi=mid;
    }
    return 0.5*(lo+hi);
}

/* Incidence of maximum L/D at this Mach within the vehicle incidence limit.
   The scan step is a discretization of the tabulated polar, not a policy. */
static double tel_best_ld_aoa(const TelContext *c,double mach) {
    const double step=0.25; /* decision-literal-ok: polar scan resolution */
    double best_aoa=c->aoa_max,best_ld=-INFINITY;
    for (double a=TEL_AOA_MIN;a<=c->aoa_max+1e-9;a+=step) {
        double cl,cd;
        aero_coefficients(&c->model->aero,mach,a,&cl,&cd);
        if (!(cd>0.0)) continue;
        double ld=cl/cd;
        if (ld>best_ld) { best_ld=ld; best_aoa=a; }
    }
    return best_aoa;
}

/* Incidence command for drag-modulation level k; `along` is the remaining path. */
static double tel_vertical(const TelContext *c,const TelPoint *p,double bank_deg,
        double k,double along,bool live) {
    double q,mach,lift,drag;
    tel_air(c,p->h,p->v,&q,&mach);
    if (mach>1.05) {
        /* Maximum range is the measured polar's best L/D at this Mach, not
           the incidence ceiling: the live polar peaks at 8-10 deg near Mach
           1.2-1.5 and above 15 deg from Mach 1.6. */
        double best=tel_best_ld_aoa(c,mach);
        if (p->g<-32.0*DEG2RAD) return c->aoa_max;
        return best+(fmin(best,TEL_AOA_SUP_MIN)-best)*k;
    }
    double q_ref=TEL_Q_LO+(TEL_Q_HI-TEL_Q_LO)*k;
    /* At maximum range (k=0) the station speed is already out of reach;
       blending toward it dives away range Final now needs more (live L7). */
    if (along<TEL_BLEND&&(!live||k>TEL_K_SHORT)) {
        double q_sta,m_sta;
        tel_air(c,p->h,c->v_station,&q_sta,&m_sta);
        double w=fmax(0.0,along)/TEL_BLEND;
        q_ref=w*q_ref+(1.0-w)*q_sta;
    }
    double gravity=tel_gravity(c,p->h);
    double a1=tel_lift_aoa(c,p,p->m*gravity);
    tel_forces(c,p,a1,&q,&mach,&lift,&drag);
    double ld=drag>0.0?fmax(0.3,lift/drag):1.0;
    double g_cmd=-atan(1.0/ld)-0.6*clampd((q_ref-q)/q_ref,-0.6,0.6);
    if (along>0.0&&along<TEL_AIM) {
        /* aim at the station point, but never shallower than the equilibrium
           glide: a glider below the station cannot climb to it, and trying to
           only bleeds the speed it needs */
        double g_aim=fmin(-atan2(p->h-c->h_station,fmax(500.0,along)),-atan(1.0/ld));
        double w=fmin(1.0,(TEL_AIM-along)/3000.0);
        g_cmd=(1.0-w)*g_cmd+w*g_aim;
    } else if (live&&along<=0.0) {
        /* past the station (Final not yet admitted): hold the outer glide slope */
        double ogs=atan(c->h_station/c->d_station),dist=fmax(1000.0,-p->x);
        g_cmd=-ogs-0.5*(p->h-dist*tan(ogs))/dist;
        g_cmd=clampd(g_cmd,-35.0*DEG2RAD,-10.0*DEG2RAD);
    }
    g_cmd=clampd(g_cmd,-35.0*DEG2RAD,6.0*DEG2RAD);
    double gdot=clampd((g_cmd-p->g)/4.0,-0.08,0.08);
    double cphi=fmax(0.3,cos(fabs(bank_deg)*DEG2RAD));
    double lift_req=p->m*(gravity*cos(p->g)+p->v*gdot)/cphi;
    return clampd(tel_lift_aoa(c,p,lift_req),TEL_AOA_MIN,c->aoa_max);
}

/* Bench servo: second-order rate/acceleration limited incidence response. */
static void tel_servo(double *v,double *r,double cmd,double dt) {
    const double rate_max=5.8,accel_max=14.0,wn=1.5;
    double want=clampd(wn*(cmd-*v),-rate_max,rate_max);
    double stop=sqrt(2.0*accel_max*fabs(cmd-*v));
    want=clampd(want,-stop,stop);
    double acc=clampd((want-*r)/dt,-accel_max,accel_max);
    *r+=acc*dt;
    *v+=*r*dt;
}

/* Path distance flown at level k until the Final station energy is reached. */
static double tel_predict_range_ex(const TelContext *c,const TelPoint *s,double k,
        double along,const TelSegment *segs,int nseg,double *bank_over_deg);

static double tel_predict_range(const TelContext *c,const TelPoint *s,double k,
        double along,const TelSegment *segs,int nseg) {
    return tel_predict_range_ex(c,s,k,along,segs,nseg,NULL);
}

/* Also reports the largest bank, beyond TEL_BANK_TURN, that a planned arc
   (other than the leading capture turn) would need at the predicted speed. */
static double tel_predict_range_ex(const TelContext *c,const TelPoint *s,double k,
        double along,const TelSegment *segs,int nseg,double *bank_over_deg) {
    TelPoint p=*s;
    p.x=0.0;p.y=0.0;p.psi=0.0;
    double dt=1.0,dist=0.0,used=0.0,t=0.0;
    int i=0;
    double rr=c->model->world.radius_m+c->model->site.altitude;
    while (t<1500.0) {
        double e=p.h+p.v*p.v/(2.0*TEL_G0);
        if (e<=c->e_station||p.h<0.0) break;
        while (i<nseg&&used>=segs[i].length) { used-=segs[i].length; ++i; }
        double rad=i<nseg?segs[i].radius:0.0;
        double gnd=p.v*cos(p.g);
        double need=rad>0.0?atan(gnd*gnd/(TEL_G0*rad))*RAD2DEG:0.0;
        if (bank_over_deg&&i>0&&i<nseg) *bank_over_deg=fmax(*bank_over_deg,need-TEL_BANK_TURN);
        double bank=fmin(TEL_BANK_MAX,need);
        double aoa_cmd=tel_vertical(c,&p,bank,k,along-dist,false);
        tel_servo(&p.aoa,&p.aoa_rate,aoa_cmd,dt);
        double q,mach,lift,drag;
        tel_forces(c,&p,p.aoa,&q,&mach,&lift,&drag);
        double v=fmax(p.v,1.0),grav=tel_gravity(c,p.h),phi=bank*DEG2RAD;
        double dv=-drag/p.m-grav*sin(p.g);
        double dg=lift*cos(phi)/(p.m*v)-(grav/v-v/(rr+p.h))*cos(p.g);
        double d=v*cos(p.g)*dt;
        p.v+=dv*dt;
        p.g+=dg*dt;
        p.h+=v*sin(p.g)*dt;
        dist+=d;used+=d;t+=dt;
    }
    return dist;
}

static int tel_segments_path(const TaemEnergyLawState *L,const TelContext *c,
        const TelPoint *s,TelSegment *out);
static bool tel_hac_tangent(const TaemEnergyLawState *L,const TelContext *c,
        double x,double y,double *tx,double *ty,double *arc);

/* Remaining path, led by the heading capture still to be flown on a
   centreline leg (charged as an arc at the leg's bank limit). */
static int tel_segments(const TaemEnergyLawState *L,const TelContext *c,const TelPoint *s,
        TelSegment *out) {
    TelSegment path[4]={{0}};
    int n=tel_segments_path(L,c,s,path);
    double turn,tx,ty,hac_arc;
    if (L->leg==TEL_LEG_DOWN&&tel_hac_tangent(L,c,s->x,s->y,&tx,&ty,&hac_arc))
        turn=fabs(tel_wrap(atan2(ty-s->y,tx-s->x)-s->psi));
    else if (L->leg==TEL_LEG_OUT&&!L->loop) {
        for (int i=0;i<n;++i) out[i]=path[i];
        return n;
    }
    else if (L->leg==TEL_LEG_OUT||L->leg==TEL_LEG_FINAL)
        turn=fabs(s->psi)+fabs(atan2(s->y,fmax(3000.0,0.4*fmax(0.0,-c->d_station-s->x))));
    else {
        for (int i=0;i<n;++i) out[i]=path[i];
        return n;
    }
    double gnd=s->v*cos(s->g),q,mach;
    tel_air(c,s->h,s->v,&q,&mach);
    double lim=(mach>1.1?TEL_BANK_SUP:TEL_BANK_MAX)*DEG2RAD;
    double rad=gnd*gnd/(TEL_G0*tan(lim));
    double arc=fmin(rad*turn,path[0].length);
    int m=0;
    if (arc>100.0) out[m++]=(TelSegment){arc,rad};
    out[m++]=(TelSegment){path[0].length-arc,0.0};
    for (int i=1;i<n;++i) out[m++]=path[i];
    return m;
}

/* Tangent point on the HAC (centre (-D, side*R), turning toward `side`, ending
   on the centreline at the station heading down the runway) seen from (x,y),
   and the arc from there to the station.  False when (x,y) is on/inside it. */
static bool tel_hac_tangent(const TaemEnergyLawState *L,const TelContext *c,
        double x,double y,double *tx,double *ty,double *arc) {
    double r=L->radius,cx=-c->d_station,cy=L->side*r;
    double dx=x-cx,dy=y-cy,d=hypot(dx,dy);
    if (d<=r*1.02) return false;
    double ang=atan2(dy,dx)+L->side*acos(r/d);
    *tx=cx+r*cos(ang);
    *ty=cy+r*sin(ang);
    double end=-L->side*TEL_PI/2.0;
    *arc=fmod(L->side*(end-ang),2.0*TEL_PI);
    if (*arc<0.0) *arc+=2.0*TEL_PI;
    *arc*=r;
    return true;
}

static double tel_hac_remaining(const TaemEnergyLawState *L,const TelContext *c,
        double x,double y) {
    double r=L->radius,cx=-c->d_station,cy=L->side*r;
    double ang=atan2(y-cy,x-cx),end=-L->side*TEL_PI/2.0;
    double rem=fmod(L->side*(end-ang),2.0*TEL_PI);
    if (rem<0.0) rem+=2.0*TEL_PI;
    if (rem>1.9*TEL_PI) rem=0.0;
    return r*rem;
}

/* Bank-limited turn radius at the energy-mean speed of the remaining TAEM:
   the vehicle slows from its present speed toward the station speed while it
   turns, so neither end-point speed prices the turn. */
static double tel_mean_turn_radius(const TelContext *c,const TelPoint *s) {
    double gnd=s->v*cos(s->g),q,mach;
    tel_air(c,s->h,s->v,&q,&mach);
    double lim=(mach>1.1?TEL_BANK_SUP:TEL_BANK_MAX)*DEG2RAD;
    double v2=0.5*(gnd*gnd+c->v_station*c->v_station);
    return v2/(TEL_G0*tan(lim));
}

/* Shortest turn-straight-turn (Dubins CSC) path from the vehicle pose to the
   Final station on runway heading.  Frame: x down the runway, y left, psi
   counter-clockwise.  The old straight-in chord clamped the along-track
   leg at the station, so a vehicle past the station abeam the runway was
   priced at |y| with no turn-back (2026-10-14 live: sGo = crossrange). */
static int tel_dubins_to_station(const TelContext *c,const TelPoint *s,double r,
        TelSegment *out) {
    double ex=-c->d_station,ey=0.0,epsi=0.0;
    double best=INFINITY,a1=0.0,st=0.0,a2=0.0;
    for (int d1=-1;d1<=1;d1+=2)
        for (int d2=-1;d2<=1;d2+=2) {
            double c1x=s->x-d1*r*sin(s->psi),c1y=s->y+d1*r*cos(s->psi);
            double c2x=ex-d2*r*sin(epsi),c2y=ey+d2*r*cos(epsi);
            double dx=c2x-c1x,dy=c2y-c1y,d=hypot(dx,dy),phi=atan2(dy,dx);
            double theta,straight;
            if (d1==d2) { theta=phi; straight=d; }
            else {
                if (d<2.0*r) continue;
                theta=phi+d1*asin(2.0*r/d);
                straight=sqrt(d*d-4.0*r*r);
            }
            double arc1=fmod(d1*(theta-s->psi),2.0*TEL_PI);
            if (arc1<0.0) arc1+=2.0*TEL_PI;
            double arc2=fmod(d2*(epsi-theta),2.0*TEL_PI);
            if (arc2<0.0) arc2+=2.0*TEL_PI;
            double total=r*(arc1+arc2)+straight;
            if (total<best) { best=total; a1=r*arc1; st=straight; a2=r*arc2; }
        }
    if (!isfinite(best)) {
        out[0]=(TelSegment){hypot(ex-s->x,ey-s->y),0.0};
        return 1;
    }
    out[0]=(TelSegment){a1,r};
    out[1]=(TelSegment){st,0.0};
    out[2]=(TelSegment){a2,r};
    return 3;
}

static int tel_segments_path(const TaemEnergyLawState *L,const TelContext *c,
        const TelPoint *s,TelSegment *out) {
    double r=L->radius,D=c->d_station,tx,ty,arc;
    if (L->leg==TEL_LEG_FINAL) {
        out[0]=(TelSegment){hypot(fmax(0.0,-D-s->x),s->y),0.0};
        return 1;
    }
    if (!L->loop)
        return tel_dubins_to_station(c,s,tel_mean_turn_radius(c,s),out);
    if (L->leg==TEL_LEG_OUT) {
        /* turnaround toward the centreline, then intercept the HAC on that side */
        TaemEnergyLawState q=*L;
        q.side=s->y<=0.0?1:-1;
        double rt=TEL_R_TURN,ex=fmax(L->x_turn,s->x),ey=s->y+2.0*q.side*rt;
        out[0]=(TelSegment){fmax(0.0,L->x_turn-s->x),0.0};
        out[1]=(TelSegment){TEL_PI*rt,rt};
        if (tel_hac_tangent(&q,c,ex,ey,&tx,&ty,&arc)) {
            out[2]=(TelSegment){hypot(tx-ex,ty-ey),0.0};
            out[3]=(TelSegment){arc,r};
        } else {
            out[2]=(TelSegment){0.0,0.0};
            out[3]=(TelSegment){tel_hac_remaining(&q,c,ex,ey),r};
        }
        return 4;
    }
    if (L->leg==TEL_LEG_TURN||L->leg==TEL_LEG_DOWN) {
        if (!tel_hac_tangent(L,c,s->x,s->y,&tx,&ty,&arc)) {
            out[0]=(TelSegment){tel_hac_remaining(L,c,s->x,s->y),r};
            return 1;
        }
        double rem=0.0;
        if (L->leg==TEL_LEG_TURN) {
            /* heading change still needed to point at the tangent point */
            double brg=atan2(ty-s->y,tx-s->x);
            rem=fmod(L->side*(brg-s->psi),2.0*TEL_PI);
            if (rem<0.0) rem+=2.0*TEL_PI;
            if (rem>1.9*TEL_PI) rem=0.0;
        }
        int n=0;
        if (rem*L->r_turn>50.0) out[n++]=(TelSegment){rem*L->r_turn,L->r_turn};
        out[n++]=(TelSegment){fmax(0.0,hypot(tx-s->x,ty-s->y)-rem*L->r_turn*0.6),0.0};
        out[n++]=(TelSegment){arc,r};
        return n;
    }
    out[0]=(TelSegment){fmax(0.0,L->hac_left_rad)*r,r};
    return 1;
}

static double tel_sum(const TelSegment *s,int n) {
    double a=0.0;
    for (int i=0;i<n;++i) a+=s[i].length;
    return a;
}

static double tel_predict_pattern(const TaemEnergyLawState *L,const TelContext *c,
        const TelPoint *s,double k,double *bank_over_deg);

static double tel_solve_k(const TaemEnergyLawState *L,const TelContext *c,
        const TelPoint *s,double *s_req) {
    TelSegment segs[5];
    int n=tel_segments(L,c,s,segs);
    double s_go=tel_sum(segs,n);
    double r_lo=L->capture_forecast?tel_predict_pattern(L,c,s,0.0,NULL):
        tel_predict_range(c,s,0.0,s_go,segs,n);
    double r_hi=L->capture_forecast?tel_predict_pattern(L,c,s,1.0,NULL):
        tel_predict_range(c,s,1.0,s_go,segs,n);
    if (r_hi>=s_go) { *s_req=r_hi; return 1.0; }
    if (r_lo<=s_go) { *s_req=r_lo; return 0.0; }
    double lo=0.0,hi=1.0;
    for (int i=0;i<7;++i) {
        double mid=0.5*(lo+hi);
        double reach=L->capture_forecast?tel_predict_pattern(L,c,s,mid,NULL):
            tel_predict_range(c,s,mid,s_go,segs,n);
        if (reach>s_go) lo=mid; else hi=mid;
    }
    *s_req=s_go;
    return 0.5*(lo+hi);
}

/* Excess of predicted range at K_PLAN over the planned path for one pattern. */
static double tel_pattern_excess(TaemEnergyLawState *L,const TelContext *c,const TelPoint *s) {
    TelSegment segs[5];
    int n=tel_segments(L,c,s,segs);
    return (L->capture_forecast?tel_predict_pattern(L,c,s,TEL_K_PLAN,NULL):
        tel_predict_range(c,s,TEL_K_PLAN,1e9,segs,n))-tel_sum(segs,n);
}

static void tel_size_hac(TaemEnergyLawState *L,const TelContext *c,const TelPoint *s,
        bool set_turn) {
    double v_hac=clampd(s->v*cos(s->g),130.0,170.0);
    double r_fly=v_hac*v_hac/(TEL_G0*tan(35.0*DEG2RAD));
    static const double radii[]={TEL_R_MIN,2600.0,3000.0,3500.0,4000.0,4500.0,
        5000.0,5500.0,6000.0,7000.0,8000.0,9000.0,TEL_R_MAX};
    double best_r=L->radius,best_e=INFINITY;
    for (size_t i=0;i<sizeof(radii)/sizeof(radii[0]);++i) {
        if (i>0&&radii[i]<=r_fly) continue;
        L->radius=fmax(radii[i],r_fly);
        if (set_turn) L->x_turn=-c->d_station;
        double ext=tel_pattern_excess(L,c,s);
        if (fabs(ext)<fabs(best_e)) { best_e=ext; best_r=L->radius; }
        if (set_turn&&ext<=0.0) break;
    }
    L->radius=best_r;
    if (set_turn) L->x_turn=-c->d_station+L->radius+500.0+fmax(0.0,best_e)/2.0;
}

/* A HAC met at its exit is ambiguous: the arc left is that small remainder
   or a full circle more.  A downwind leg that rejoins the centreline west of
   the station meets it this way with the pattern's energy still to lose.
   Every circle of the family is tangent to the runway line at the station,
   so on entry there the radius is free: fly the circle when its worst
   shortfall (smallest flyable radius at maximum range) is less than the
   excess without it, sized for the mid-drag range. */
static double tel_entry_arc(TaemEnergyLawState *L,const TelContext *c,const TelPoint *s,double rem) {
    double r=L->radius;
    if (rem>TEL_PI) return rem;
    TelSegment none[1]={{rem*r,r}};
    double excess=tel_predict_range(c,s,1.0,1e9,none,1)-rem*r;
    /* airspeed, not the diving horizontal speed: the vehicle accelerates
       into the circle and a radius sized for the dive cannot be held */
    double vg=s->v;
    double r_small=fmax(TEL_R_SPIRAL_MIN,vg*vg/(TEL_G0*tan(TEL_BANK_TURN*DEG2RAD)));
    double turn=rem+2.0*TEL_PI;
    TelSegment small[1]={{turn*r_small,r_small}};
    double shortfall=turn*r_small-tel_predict_range(c,s,0.0,1e9,small,1);
    TelSegment mid[1]={{turn*r,r}};
    double reach=tel_predict_range(c,s,TEL_K_PLAN,1e9,mid,1);
    /* away from the exit the radius cannot be chosen freely: demand the
       planner's switching margin before committing to the longer turn */
    bool circle=excess>fmax(0.0,shortfall)+(rem>0.35?2000.0:0.0);
    if (getenv("KSP_LANDER_TEL_TRACE"))
        fprintf(stderr,"TEL_ENTRY rem=%.2f r=%.0f excess=%.0f shortfall=%.0f r_small=%.0f reach=%.0f circle=%d\n",
            rem,r,excess,shortfall,r_small,reach,(int)circle);
    if (!circle) return rem;
    /* only at the exit is the vehicle on every circle of the family; earlier
       the spiral resizes it in flight */
    if (rem<=0.35) L->radius=clampd(reach/turn,r_small,TEL_R_MAX);
    return turn;
}

/* An along-track crossing is not a Final intercept. A polar pass can be
   directly abeam the station while still tens of kilometres off centreline;
   retaining the capture leg is necessary to keep the full lateral correction
   rather than the near-runway 2 km cross-track trim. */
static bool tel_final_line_ready(const TelContext *c,const TelPoint *s){
    if(!c||!s||!(c->d_station>0.0)||
       !isfinite(s->x)||!isfinite(s->y)||!isfinite(s->psi))return false;
    return -c->d_station-s->x<6000.0&&
        fabs(s->y)<=fmin(2000.0,0.25*c->d_station)&&
        fabs(tel_wrap(s->psi))<=15.0*DEG2RAD;
}

/* Shared by live control and the range forecast.  A capture turn must use
   the same heading error, bank saturation and leg transitions in both. */
static double tel_lateral(TaemEnergyLawState *L,const TelContext *c,const TelPoint *s) {
    double bank_cmd;
    double D=c->d_station;
    double q,mach;
    tel_air(c,s->h,s->v,&q,&mach);
    double gnd=s->ground_speed>0.0?s->ground_speed:s->v*cos(s->g);
    if (L->loop&&L->leg==TEL_LEG_OUT&&s->x>=L->x_turn) {
        L->leg=TEL_LEG_TURN;
        L->side=s->y<=0.0?1:-1;
        L->r_turn=fmax(TEL_R_TURN,gnd*gnd/(TEL_G0*tan(45.0*DEG2RAD)));
    }
    double r=L->radius,bank_ff=0.0,course_cmd;
    int side=L->side;
    if (L->leg==TEL_LEG_OUT||L->leg==TEL_LEG_FINAL) {
        double look=clampd(0.4*fmax(0.0,-D-s->x),3000.0,12000.0);
        /* Supersonic turn rates are a fraction of a degree per second: aim at a
           point ~45 s ahead so the capture does not overshoot the centreline. */
        if (mach>1.1) look=fmax(look,45.0*gnd);
        /* Far off axis, capture the actual station.  A positive lookahead
           assumes the station is ahead in x; after an abeam pass it instead
           aims farther east and can never close the along-track separation. */
        course_cmd=L->capture_forecast&&!L->loop&&fabs(s->y)>6000.0?atan2(-s->y,-D-s->x):atan2(-s->y,look);
        if(!L->loop&&tel_final_line_ready(c,s))L->leg=TEL_LEG_FINAL;
    } else if (L->leg==TEL_LEG_TURN||L->leg==TEL_LEG_DOWN) {
        double tx,ty,arc;
        if (!tel_hac_tangent(L,c,s->x,s->y,&tx,&ty,&arc)) {
            L->leg=TEL_LEG_HAC;
            {
                double cx=-D,cy=side*r,ang=atan2(s->y-cy,s->x-cx),end=-side*TEL_PI/2.0;
                double rem=fmod(side*(end-ang),2.0*TEL_PI);
                if (rem<0.0) rem+=2.0*TEL_PI;
                if (rem>1.9*TEL_PI) {
                    /* just past the exit: fly the full circle only if the energy needs it */
                    TelSegment full[1]={{rem*r,r}};
                    if (tel_predict_range(c,s,TEL_K_PLAN,1e9,full,1)<0.5*rem*r) rem=0.0;
                }
                L->hac_left_rad=tel_entry_arc(L,c,s,rem);
            }
            L->hac_last_ang=NAN;
            course_cmd=s->psi;
        } else {
            double brg=atan2(ty-s->y,tx-s->x);
            course_cmd=brg;
            if (L->leg==TEL_LEG_TURN) {
                /* turnaround: keep turning toward `side` at the turn radius until
                   the tangent point is ahead */
                double rem=tel_wrap(brg-s->psi);
                if (L->side*rem<0.0||fabs(rem)>0.35) {
                    bank_ff=side*atan(gnd*gnd/(TEL_G0*L->r_turn))*RAD2DEG;
                    course_cmd=s->psi;
                } else L->leg=TEL_LEG_DOWN;
            }
            if (L->leg==TEL_LEG_DOWN&&hypot(tx-s->x,ty-s->y)<fmax(300.0,2.0*gnd)) {
                L->leg=TEL_LEG_HAC;
                L->hac_left_rad=tel_entry_arc(L,c,s,arc/fmax(1.0,L->radius));
                L->hac_last_ang=NAN;
            }
        }
    }
    if (L->leg==TEL_LEG_HAC) {
        r=L->radius;   /* an exit entry may have resized the circle */
        double cx=-D,cy=side*r;
        double ang=atan2(s->y-cy,s->x-cx);
        if (isfinite(L->hac_last_ang))
            L->hac_left_rad-=side*tel_wrap(ang-L->hac_last_ang);
        L->hac_last_ang=ang;
        double r_err=hypot(s->x-cx,s->y-cy)-r;
        course_cmd=ang+side*(TEL_PI/2.0+clampd(r_err/1500.0,-0.6,0.6));
        bank_ff=side*atan(gnd*gnd/(TEL_G0*r))*RAD2DEG;
        if (L->hac_left_rad<0.3&&fabs(tel_wrap(s->psi))<0.35) {
            L->leg=TEL_LEG_FINAL;
            bank_ff=0.0;
            course_cmd=atan2(-s->y,3000.0);
        }
    }
    /* S-turns: with drag saturated (k=1) and range still in excess of the
       path, weave off the direct course so the flown path absorbs the excess,
       psi_w = acos(d/(d+E)).  Only on legs steered by course (the downwind to
       the HAC tangent and a straight-in centreline), subsonic, and ending
       3 km before the leg's end so the vehicle is realigned. */
    {
        double E=L->s_req-L->s_go,tx=-D,ty=0.0,arc=0.0;
        bool leg_ok=L->leg==TEL_LEG_DOWN||(L->leg==TEL_LEG_OUT&&!L->loop);
        if (L->leg==TEL_LEG_DOWN&&!tel_hac_tangent(L,c,s->x,s->y,&tx,&ty,&arc)) leg_ok=false;
        double d=hypot(tx-s->x,ty-s->y);
        if (L->leg==TEL_LEG_OUT) d=fmax(0.0,-D-6000.0-s->x);
        if (leg_ok&&L->k>0.999&&E>300.0&&mach<0.95&&d>3000.0) {
            double brg=atan2(ty-s->y,tx-s->x);
            if (!L->weave_on) { L->weave_on=true; L->weave_sign=1; L->weave_brg=brg; }
            double ux=cos(L->weave_brg),uy=sin(L->weave_brg);
            double off=-(s->x-tx)*uy+(s->y-ty)*ux;
            double ymax=clampd(0.25*d,800.0,2500.0);
            if (L->weave_sign>0&&off>ymax) L->weave_sign=-1;
            else if (L->weave_sign<0&&off<-ymax) L->weave_sign=1;
            double psi_w=fmin(acos(d/(d+E)),50.0*DEG2RAD);
            course_cmd=brg+L->weave_sign*psi_w;
            bank_ff=0.0;
        } else L->weave_on=false;
    }
    double cerr=tel_wrap(course_cmd-s->psi);
    double omega=clampd(cerr/6.0,-0.12,0.12);
    double bank_left=bank_ff+atan(gnd*omega/TEL_G0)*RAD2DEG;
    double lim=mach>1.1?TEL_BANK_SUP:TEL_BANK_MAX;
    if (L->leg==TEL_LEG_HAC||(L->leg==TEL_LEG_TURN&&bank_ff!=0.0)) lim=TEL_BANK_ARC;
    lim=fmin(lim,c->model->vehicle.maximum_bank_angle);
    bank_cmd=-clampd(bank_left,-lim,lim);
    return bank_cmd;
}

/* Price actual progress toward the selected route, rather than distance
   travelled along a fictitious tangent.  The former segment predictor reduced
   bank as speed fell on a fixed initial capture radius, although live control
   kept maximum bank until the heading was captured.  It also counted travel
   away from the station as useful range.  Integrate the shared lateral law and
   its lift-dependent turn rate; only reduction of remaining route counts. */
static double tel_predict_pattern(const TaemEnergyLawState *L,const TelContext *c,
        const TelPoint *s,double k,double *bank_over_deg) {
    TaemEnergyLawState route=*L;
    route.k=k;
    route.s_req=route.s_go=0.0; /* no unpriced surplus weave in a range probe */
    TelPoint p=*s;
    TelSegment segs[5];
    double initial=tel_sum(segs,tel_segments(L,c,s,segs));
    const double dt=1.0;
    double rr=c->model->world.radius_m+c->model->site.altitude;
    for (unsigned i=0;i<1500;++i) {
        if (p.h+p.v*p.v/(2.0*TEL_G0)<=c->e_station||p.h<0.0||p.v<1.0) break;
        double along=tel_sum(segs,tel_segments_path(&route,c,&p,segs));
        p.ground_speed=p.v*cos(p.g);
        double command=tel_lateral(&route,c,&p);
        p.bank+=clampd(command-p.bank,-6.0*dt,6.0*dt);
        if (bank_over_deg&&route.leg==TEL_LEG_HAC) {
            double gnd=p.v*cos(p.g);
            double need=atan(gnd*gnd/(TEL_G0*route.radius))*RAD2DEG;
            *bank_over_deg=fmax(*bank_over_deg,need-TEL_BANK_TURN);
        }
        double vertical_along=route.leg==TEL_LEG_FINAL?-c->d_station-p.x:along;
        double target=tel_vertical(c,&p,p.bank,k,vertical_along,true);
        tel_servo(&p.aoa,&p.aoa_rate,target,dt);
        double q,mach,lift,drag;
        tel_forces(c,&p,p.aoa,&q,&mach,&lift,&drag);
        double v=fmax(p.v,1.0),grav=tel_gravity(c,p.h),phi=p.bank*DEG2RAD;
        double gnd=v*cos(p.g);
        double dv=-drag/p.m-grav*sin(p.g);
        double dg=lift*cos(phi)/(p.m*v)-(grav/v-v/(rr+p.h))*cos(p.g);
        double dpsi=-lift*sin(phi)/(p.m*fmax(1.0,gnd));
        p.x+=gnd*cos(p.psi)*dt;
        p.y+=gnd*sin(p.psi)*dt;
        p.h+=v*sin(p.g)*dt;
        p.v+=dv*dt;
        p.g+=dg*dt;
        p.psi=tel_wrap(p.psi+dpsi*dt);
    }
    double remaining=tel_sum(segs,tel_segments(&route,c,&p,segs));
    if (route.leg==TEL_LEG_FINAL)
        remaining=-c->d_station-p.x+fabs(p.y);
    return initial-remaining;
}

/* One guidance tick.  Returns false when the inputs are unusable. */
/* Pattern choice is open while the first leg is still far from its end. */
static bool tel_plan_open(const TaemEnergyLawState *L,const TelContext *c,const TelPoint *s) {
    double D=c->d_station,tx,ty,arc;
    if (L->leg==TEL_LEG_OUT) {
        if (L->loop) return s->x<L->x_turn-6000.0;
        /* A near-polar inlet begins well off the runway centreline, often
           already abreast of the Final station (x ~ 0, |y| tens of km).
           The old x-only gate never evaluated an available tangent HAC. */
        double remaining=hypot(-D-s->x,s->y);
        return s->x<-D-TEL_MODE_FREEZE ||
            (fabs(s->y)>fmax(6000.0,0.75*D)&&
             remaining>D+TEL_MODE_FREEZE);
    }
    if (L->leg==TEL_LEG_DOWN)
        return tel_hac_tangent(L,c,s->x,s->y,&tx,&ty,&arc)&&hypot(tx-s->x,ty-s->y)>5000.0;
    return false;
}

/* Score a pattern by how far its path lies outside the range the drag level
   can reach (shortfall at maximum range, excess at maximum drag), plus a mild
   preference for a pattern flown near mid-range drag.  The loop can lengthen
   its downwind, so excess does not count against it. */
static double tel_score(TaemEnergyLawState *P,const TelContext *c,const TelPoint *s) {
    TelSegment segs[5];
    int n=tel_segments(P,c,s,segs);
    double path=tel_sum(segs,n);
    double r_max=P->capture_forecast?tel_predict_pattern(P,c,s,0.0,NULL):
        tel_predict_range(c,s,0.0,1e9,segs,n);
    double r_min=P->capture_forecast?tel_predict_pattern(P,c,s,1.0,NULL):
        tel_predict_range(c,s,1.0,1e9,segs,n);
    double over=0.0;
    double r_mid=P->capture_forecast?tel_predict_pattern(P,c,s,TEL_K_PLAN,&over):
        tel_predict_range_ex(c,s,TEL_K_PLAN,1e9,segs,n,&over);
    bool stretch=P->loop&&P->leg==TEL_LEG_OUT;
    /* a directly intercepted HAC can still spiral between its flyable minimum
       and TEL_R_MAX, scaling its arc */
    double path_lo=path,path_hi=path;
    if (P->loop&&P->leg==TEL_LEG_DOWN&&n>0&&segs[n-1].radius>0.0) {
        double theta=segs[n-1].length/segs[n-1].radius;
        path_lo=path-theta*fmax(0.0,segs[n-1].radius-TEL_R_MIN)*0.5;
        path_hi=path+theta*fmax(0.0,TEL_R_MAX-segs[n-1].radius)*0.5;
    }
    double outside=fmax(0.0,path_lo-r_max)+(stretch?0.0:fmax(0.0,r_min-path_hi));
    return outside+0.15*(stretch?fmax(0.0,path-r_mid):fabs(r_mid-path))+(stretch?1000.0:0.0)+
        1000.0*fmax(0.0,over);   /* an arc the vehicle cannot hold is not a plan */
}

static void tel_plan(TaemEnergyLawState *L,const TelContext *c,const TelPoint *s) {
    static const double radii[]={TEL_R_MIN,3000.0,3500.0,4000.0,5000.0,6000.0,7000.0,8000.0,9000.0,TEL_R_MAX};
    double D=c->d_station,tx,ty,arc;
    /* a HAC must be flyable at ~35 deg of bank at the speed it will be met */
    double v_hac=clampd(s->v*cos(s->g),130.0,170.0);
    double r_fly=v_hac*v_hac/(TEL_G0*tan(35.0*DEG2RAD));
    TaemEnergyLawState cur=*L,best=*L;
    double cur_score=tel_score(&cur,c,s),best_score=cur_score;
    TaemEnergyLawState p=*L;
    p.loop=false;p.leg=TEL_LEG_OUT;
    if (s->x<-D-TEL_MODE_FREEZE || (L->capture_forecast&&fabs(s->y)>6000.0)) {
        double sc=tel_score(&p,c,s);
        if (sc<best_score) { best_score=sc; best=p; }
    }
    for (int side=-1;side<=1;side+=2)
        for (size_t i=0;i<sizeof(radii)/sizeof(radii[0]);++i) {
            if (i>0&&radii[i]<=r_fly) continue;      /* one candidate at r_fly */
            p=*L;p.loop=true;p.leg=TEL_LEG_DOWN;p.side=side;p.radius=fmax(radii[i],r_fly);
            if (!tel_hac_tangent(&p,c,s->x,s->y,&tx,&ty,&arc)) continue;
            if (hypot(tx-s->x,ty-s->y)<6000.0) continue;
            double sc=tel_score(&p,c,s);
            if (sc<best_score) { best_score=sc; best=p; }
        }
    if (s->x<-D-6000.0) {
        p=*L;p.loop=true;p.leg=TEL_LEG_OUT;p.radius=fmax(TEL_R_MIN,r_fly);p.x_turn=-D;
        double sc=tel_score(&p,c,s);
        if (sc<best_score) { best_score=sc; best=p; }
    }
    if (best_score<cur_score-2000.0) *L=best;
    if (L->loop&&L->leg==TEL_LEG_OUT) tel_size_hac(L,c,s,true);
}

/* Outer-glide slope from the energy balance over the final distance D.
   With lift carrying the weight, specific energy falls by D*(Drag/L) along
   the glide, so arriving at the approach speed from the station speed needs
     tan(gamma) = Drag/L - (V_station^2 - V_approach^2)/(2 g D),
   with Drag/L from the (measured-scaled) polar at the mean-energy speed and
   mid-glide altitude.  Holding the station speed instead (no kinetic credit)
   is a ~30 deg dive at the low incidence that speed needs.  The
   altitude/slope coupling is a fixed point; the configured slope is a seed. */
static double tel_equilibrium_slope(const TelContext *c,double mass_kg,double seed_deg,
        double approach_speed) {
    double slope=isfinite(seed_deg)&&seed_deg>0.0?seed_deg*DEG2RAD:atan(0.5);
    if (!(mass_kg>0.0)||!(c->v_station>0.0)||!(c->d_station>0.0)) return slope*RAD2DEG;
    double v_a=isfinite(approach_speed)&&approach_speed>0.0?
        fmin(approach_speed,c->v_station):c->v_station;
    double kinetic=(c->v_station*c->v_station-v_a*v_a)/(2.0*TEL_G0*c->d_station);
    for (int i=0;i<8;++i) {
        TelPoint p={.h=0.5*c->d_station*tan(slope),
            .v=sqrt(0.5*(c->v_station*c->v_station+v_a*v_a)),.m=mass_kg};
        double weight=mass_kg*tel_gravity(c,p.h);
        double aoa=tel_lift_aoa(c,&p,weight*cos(slope));
        double q,mach,lift,drag;
        tel_forces(c,&p,aoa,&q,&mach,&lift,&drag);
        if (!(lift>0.0)||!(drag>0.0)) break;
        double next=atan(fmax(0.0,drag/lift-kinetic));
        if (fabs(next-slope)<1e-5) { slope=next; break; }
        slope=next;
    }
    return clampd(slope*RAD2DEG,3.0,45.0); /* handoff-slope validity bounds */
}

static void tel_setup(const TerminalModel *model,const TaemGeometryState *geo,
        double mass_kg,double aoa_deg,double bank_deg,TelContext *out_c,TelPoint *out_s) {
    TelContext c={.model=model,
        .d_station=fmax(500.0,model->guidance.final_approach_distance),
        .v_station=fmax(model->guidance.final_alignment_speed,model->vehicle.minimum_safe_speed),
        .aoa_max=fmax(TEL_AOA_SUP_MIN,model->vehicle.maximum_angle_of_attack)};
    if (model->vehicle.terminal_maximum_lift_angle_of_attack>0.0)
        c.aoa_max=fmin(c.aoa_max,model->vehicle.terminal_maximum_lift_angle_of_attack);
    c.slope_deg=tel_equilibrium_slope(&c,mass_kg,model->guidance.final_glide_slope,
        fmax(model->vehicle.final_approach_speed,model->vehicle.minimum_safe_speed));
    c.h_station=c.d_station*tan(c.slope_deg*DEG2RAD);
    c.e_station=c.h_station+c.v_station*c.v_station/(2.0*TEL_G0);
    TelPoint s={.x=geo->runway_along_m,.y=-geo->runway_cross_m,
        .h=geo->altitude_above_runway_m,.v=geo->airspeed_mps,
        .g=geo->flight_path_angle_deg*DEG2RAD,.psi=-geo->heading_error_deg*DEG2RAD,
        .m=mass_kg,.aoa=isfinite(aoa_deg)?aoa_deg:10.0,.aoa_rate=0.0,
        .bank=isfinite(bank_deg)?bank_deg:0.0,.ground_speed=geo->ground_speed_mps};
    *out_c=c;*out_s=s;
}

static void tel_initialize(TaemEnergyLawState *L,const TelContext *c,const TelPoint *s) {
    double D=c->d_station;
    *L=(TaemEnergyLawState){.initialized=true,.leg=TEL_LEG_OUT,.side=1,
        .capture_forecast=fabs(s->y)>6000.0,
        .x_turn=-D,.radius=5000.0,.k=0.5,.last_geo_ut=-INFINITY,.last_pred_ut=-INFINITY};
}

/* Physical cost of starting the law in this runway frame: the best pattern's
   distance outside the range its drag levels can reach (tel_score). Lower is
   better; the same predictor then flies the chosen frame. */
static double taem_energy_law_frame_score(const TerminalModel *model,
        const TaemGeometryState *geo,double mass_kg,double aoa_deg,double bank_deg) {
    if (!model||!geo||!(mass_kg>0.0)||!isfinite(geo->runway_along_m)) return INFINITY;
    TelContext c;TelPoint s;
    tel_setup(model,geo,mass_kg,aoa_deg,bank_deg,&c,&s);
    TaemEnergyLawState L;
    tel_initialize(&L,&c,&s);
    tel_plan(&L,&c,&s);
    double score=tel_score(&L,&c,&s);
    return isfinite(score)?score:INFINITY;
}

static bool taem_energy_law_step(TaemEnergyLawState *L,const TerminalModel *model,
        const TaemGeometryState *geo,double mass_kg,double aoa_deg,double ut,
        double bank_deg,double *aoa_cmd,double *bank_cmd) {
    if (!L||!model||!geo||!(mass_kg>0.0)||!isfinite(geo->runway_along_m)) return false;
    TelContext c;TelPoint s;
    tel_setup(model,geo,mass_kg,aoa_deg,bank_deg,&c,&s);
    L->station_slope_deg=c.slope_deg;
    double D=c.d_station;
    if (!L->initialized) tel_initialize(L,&c,&s);
    double q,mach;
    tel_air(&c,s.h,s.v,&q,&mach);
    double gnd=geo->ground_speed_mps;

    if (ut>=L->last_geo_ut+TEL_UPDATE_S&&tel_plan_open(L,&c,&s)) {
        L->last_geo_ut=ut;
        tel_plan(L,&c,&s);
    } else if (L->loop&&L->leg==TEL_LEG_OUT&&ut>=L->last_geo_ut+TEL_UPDATE_S) {
        L->last_geo_ut=ut;
        double ext=tel_pattern_excess(L,&c,&s);
        L->x_turn=fmax(-D+L->radius+500.0,fmax(s.x,L->x_turn+0.5*ext));
    }
    if (L->loop&&L->leg==TEL_LEG_OUT&&s.x>=L->x_turn) {
        L->leg=TEL_LEG_TURN;
        L->side=s.y<=0.0?1:-1;
        L->r_turn=fmax(TEL_R_TURN,gnd*gnd/(TEL_G0*tan(45.0*DEG2RAD)));
    }
    if (L->leg==TEL_LEG_HAC&&L->hac_left_rad>0.5&&ut>=L->last_geo_ut+TEL_UPDATE_S) {
        /* HAC spiral: resize the circle (still tangent to the runway line at
           the station) so the remaining arc matches the range at mid drag. */
        L->last_geo_ut=ut;
        TelSegment arc1[1]={{L->hac_left_rad*L->radius,L->radius}};
        double need=tel_predict_range(&c,&s,TEL_K_PLAN,1e9,arc1,1);
        double r_fly=gnd*gnd/(TEL_G0*tan(TEL_BANK_TURN*DEG2RAD));
        /* below the planning floor only while an added circle is still far
           from the exit: a tight HAC exit leaves Final a lateral offset */
        double r_floor=L->hac_left_rad>1.5*TEL_PI?TEL_R_SPIRAL_MIN:fmin(TEL_R_MIN,L->radius);
        double r_new=clampd(need/L->hac_left_rad,fmax(r_floor,r_fly),TEL_R_MAX);
        L->radius=clampd(r_new,L->radius-250.0,L->radius+250.0);
    }
    TelSegment segs[5];
    int nseg=tel_segments(L,&c,&s,segs);
    L->s_go=tel_sum(segs,nseg);
    if (getenv("KSP_LANDER_TEL_TRACE")&&(L->leg!=L->trace_leg||L->side!=L->trace_side||
            fabs(L->radius-L->trace_radius)>1.0||L->loop!=L->trace_loop)) {
        L->trace_leg=L->leg;L->trace_side=L->side;L->trace_radius=L->radius;L->trace_loop=L->loop;
        fprintf(stderr,"TEL_PLAN ut=%.1f loop=%d leg=%d side=%+d R=%.0f D=%.0f x=%.0f y=%.0f psi=%.1f xturn=%.0f segs:",
            ut,(int)L->loop,L->leg,L->side,L->radius,D,s.x,s.y,s.psi*RAD2DEG,L->x_turn);
        for (int i=0;i<nseg;++i) fprintf(stderr," [%.0f r%.0f]",segs[i].length,segs[i].radius);
        fprintf(stderr,"\n");
    }
    if (ut>=L->last_pred_ut+TEL_UPDATE_S) {
        L->last_pred_ut=ut;
        L->k=tel_solve_k(L,&c,&s,&L->s_req);
    }

    *bank_cmd=tel_lateral(L,&c,&s);
    /* on the runway line the vertical reference is the signed distance to the station */
    double along_v=L->leg==TEL_LEG_FINAL?-D-s.x:L->s_go;
    *aoa_cmd=tel_vertical(&c,&s,*bank_cmd,L->k,along_v,true);
    return isfinite(*aoa_cmd)&&isfinite(*bank_cmd);
}
