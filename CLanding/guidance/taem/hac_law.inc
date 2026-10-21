/* MM305 Shuttle TAEM: heading-alignment-cone guidance.
 *
 * Structure follows the Orbiter TAEM (T. E. Moore, "Space Shuttle Entry
 * Terminal Area Energy Management", NASA TM-104744, 1991):
 *   TGTRAN  phases S-turn -> acquisition -> heading alignment -> prefinal;
 *   GTP     range-to-go = acquisition turn + tangent line + spiral HAC + final;
 *           spiral RTURN(psi) = RF + R2 psi^2, arc RF psi + R2 psi^3/3;
 *   TGCOMP  energy/altitude references against predicted range;
 *   TGNZC   altitude tracking, mid-value limited by the energy band and the
 *           speed (dynamic-pressure) limits;
 *   TGPHIC  S-turn at the bank limit, acquisition 2.5 x heading error, HAC
 *           radial position/rate law with centrifugal feed-forward, prefinal
 *           lateral position/rate law;
 *   TGSBC   speedbrake on energy error (subsonic).
 *   Spiral adjustment of RF while turning (Shuttle shrinks it when low; with
 *   no nominal speedbrake bias here it also grows it when high).
 *
 * Runway frame as energy_law.inc: x along the runway (site 0, HAC exit at the
 * Final station x=-D), y LEFT, psi from the runway axis positive LEFT, positive
 * bank is a right turn.  side=+1 puts the HAC centre left of the runway line
 * (a left, counter-clockwise turn onto the line).
 *
 * The energy reference is integrated backward from the station along a speed
 * schedule (equilibrium glide at a reference incidence) and priced with the
 * polar corrected by live measured lift/drag per Mach bin.  Measured forces are
 * authoritative; the recorded polar only fills Mach regimes not yet flown. */

enum { TAEM_HAC_PHASE_STURN=0, TAEM_HAC_PHASE_ACQ, TAEM_HAC_PHASE_HAC, TAEM_HAC_PHASE_PREFINAL };

/* Shuttle spiral coefficient 0.093 ft/deg^2 in m/rad^2. */
#define HAC_R2 93.0
#define HAC_RF_MIN 2200.0
#define HAC_RF_MAX 9000.0
/* bank that sizes the nominal exit radius at the station speed */
#define HAC_BANK_NOMINAL 30.0
#define HAC_RF_RATE 15.0
/* reference L/D as a fraction of the maximum (front side of the polar) */
#define HAC_LD_FRACTION 0.9
#define HAC_STATION_MARGIN_M 300.0
#define HAC_BANK_SIZING 55.0
#define HAC_OGS_FULL_M 1500.0
#define HAC_OGS_BLEND_M 5000.0
#define HAC_Q_BLEND_M 12000.0
#define HAC_REF_DS 400.0
#define HAC_REF_REBUILD_S 2.0
/* Mach distance over which one bin's live ratio fades toward the polar */
#define HAC_KAPPA_REACH 1.5
#define HAC_KAPPA_TAU_S 6.0
#define HAC_GPHI 2.5
#define HAC_RERRLM 2100.0
#define HAC_STURN_MIN_RANGE 15000.0
#define HAC_STURN_MAX_PSI (200.0*DEG2RAD)
#define HAC_SELECT_FREEZE_M 10000.0
#define HAC_SELECT_MARGIN_M 1500.0
#define HAC_UPDATE_S 2.0

typedef struct {
    TelContext c;
    const AeroTable *aero;          /* recorded polar (unscaled) */
    const TaemHacLiveAero *live;
    double mass;
} HacCtx;

static int hac_bin(double mach) {
    int i=(int)floor(fmax(0.0,mach)/0.25);
    return i<0?0:(i>=TAEM_HAC_KAPPA_BINS?TAEM_HAC_KAPPA_BINS-1:i);
}

/* Live/recorded ratio at this Mach: the bin's own measurement, else the
   nearest measured bin fading toward the polar with Mach distance. */
static double hac_kappa(const TaemHacLiveAero *A,double mach,bool lift) {
    if (!A) return 1.0;
    int b=hac_bin(mach);
    const double *r=lift?A->lift:A->drag;
    if (A->weight[b]>0.0&&r[b]>0.0) return r[b];
    double best=INFINITY,value=1.0;
    for (int i=0;i<TAEM_HAC_KAPPA_BINS;++i) {
        if (!(A->weight[i]>0.0&&r[i]>0.0)) continue;
        double dm=fabs(mach-(i+0.5)*0.25);
        if (dm<best) { best=dm; value=r[i]; }
    }
    if (!isfinite(best)) return 1.0;
    return 1.0+(value-1.0)*exp(-best/HAC_KAPPA_REACH);
}

static void hac_live_aero_observe(TaemHacLiveAero *A,const AeroTable *aero,
        const Telemetry *t,double speedbrake) {
    if (!A||!aero||!t||!t->physics_sample_valid||!(t->mass>0.0)) return;
    double dt=isfinite(A->last_ut)&&t->ut>A->last_ut?fmin(t->ut-A->last_ut,1.0):0.0;
    A->last_ut=t->ut;
    /* the split-rudder brake is a control, not the airframe's drag */
    if (!(dt>0.0)||!(t->dynamic_pressure>300.0)||!isfinite(t->mach)||
            !isfinite(t->angle_of_attack)||t->angle_of_attack<1.0||speedbrake>0.02)
        return;
    double cl,cd;
    aero_coefficients(aero,t->mach,t->angle_of_attack,&cl,&cd);
    double qs=t->dynamic_pressure*aero->reference_area_m2;
    if (!(cl>0.02)||!(cd>1e-3)||!(qs>0.0)) return;
    double rl=fabs(t->lift_force)/(qs*cl),rd=fabs(t->drag_force)/(qs*cd);
    if (!isfinite(rl)||!isfinite(rd)||!(rl>0.0)||!(rd>0.0)) return;
    int b=hac_bin(t->mach);
    double a=dt/(HAC_KAPPA_TAU_S+dt);
    if (!(A->weight[b]>0.0)) { A->lift[b]=rl; A->drag[b]=rd; }
    else { A->lift[b]+=a*(rl-A->lift[b]); A->drag[b]+=a*(rd-A->drag[b]); }
    A->weight[b]=fmin(1.0,A->weight[b]+dt/HAC_KAPPA_TAU_S);
    if (getenv("KSP_LANDER_HAC_AERO_TRACE"))
        fprintf(stderr,"HAC_AERO ut=%.1f M=%.2f aoa=%.1f bin=%d rawL=%.3f rawD=%.3f L=%.3f D=%.3f w=%.2f\n",
            t->ut,t->mach,t->angle_of_attack,b,rl,rd,A->lift[b],A->drag[b],A->weight[b]);
}

static void hac_aero(const HacCtx *x,double mach,double aoa,double *cl,double *cd) {
    aero_coefficients(x->aero,mach,aoa,cl,cd);
    *cl*=hac_kappa(x->live,mach,true);
    *cd*=hac_kappa(x->live,mach,false);
}

static double hac_lift_aoa(const HacCtx *x,double mach,double q,double lift_needed) {
    double area=x->aero->reference_area_m2,lo=-5.0,hi=x->c.aoa_max;
    for (int i=0;i<30;++i) {
        double mid=0.5*(lo+hi),cl,cd;
        hac_aero(x,mach,mid,&cl,&cd);
        if (cl*q*area<lift_needed) lo=mid; else hi=mid;
    }
    return 0.5*(lo+hi);
}

/* Reference incidence per Mach: the front-side incidence where the
   live-corrected L/D (smoothed over +-0.5 deg against cell noise) falls to
   HAC_LD_FRACTION of its maximum, leaving stretch authority toward max L/D. */
static void hac_reference_incidence(TaemHacReference *R,const HacCtx *x) {
    for (int m=0;m<TAEM_HAC_REF_MACH_N;++m) {
        double mach=0.1*m,best=-INFINITY,best_aoa=x->c.aoa_max;
        double ld[64];
        int n=0;
        for (double a=2.0;a<=x->c.aoa_max+1e-9&&n<64;a+=0.5,++n) {
            double sum=0.0;
            for (int k=-1;k<=1;++k) {
                double cl,cd;
                hac_aero(x,mach,a+0.5*k,&cl,&cd);
                sum+=cd>1e-4?cl/cd:0.0;
            }
            ld[n]=sum/3.0;
            if (ld[n]>best) { best=ld[n]; best_aoa=a; }
        }
        double aoa=best_aoa;
        int i=(int)round((best_aoa-2.0)/0.5);
        while (i>0&&ld[i-1]>=HAC_LD_FRACTION*best) { --i; aoa-=0.5; }
        double cl,cd;
        hac_aero(x,mach,aoa,&cl,&cd);
        R->aoa[m]=aoa;
        R->cl[m]=fmax(0.02,cl);
    }
}

/* Reference dynamic pressure: equilibrium glide at the reference incidence,
   subsonic blending to the Final station pressure. */
static double hac_q_ref(const HacCtx *x,const TaemHacReference *R,double mach,double s) {
    double u=clampd(mach,0.0,0.1*(TAEM_HAC_REF_MACH_N-1))*10.0;
    int i=(int)floor(u);
    if (i>TAEM_HAC_REF_MACH_N-2) i=TAEM_HAC_REF_MACH_N-2;
    double cl=R->cl[i]+(u-i)*(R->cl[i+1]-R->cl[i]);
    double q=x->mass*TEL_G0/(cl*x->aero->reference_area_m2);
    if (mach<1.0) {
        double q_sta,m_sta;
        tel_air(&x->c,x->c.h_station,x->c.v_station,&q_sta,&m_sta);
        q=q_sta+(q-q_sta)*clampd(s/HAC_Q_BLEND_M,0.0,1.0);
    }
    return q;
}

/* Altitude/speed split of specific energy e on the speed schedule, never
   slower than the speed that keeps bank authority for the HAC. */
static void hac_split_ref(const HacCtx *x,const TaemHacReference *R,double e,double s,
        double *h,double *v) {
    double floor_v=1.3*x->c.model->vehicle.minimum_safe_speed;
    double lo=0.0,hi=fmax(1.0,e);
    for (int i=0;i<40;++i) {
        double mid=0.5*(lo+hi),vv=sqrt(fmax(1.0,2.0*TEL_G0*(e-mid))),q,mach;
        tel_air(&x->c,mid,vv,&q,&mach);
        if (q>hac_q_ref(x,R,mach,s)&&vv>floor_v) lo=mid; else hi=mid;
    }
    *h=0.5*(lo+hi);
    /* Prefinal: the altitude reference merges into the outer glide slope
       (Moore's OGS), so Final receives the vehicle on its own glide rather
       than on the steeper TAEM equilibrium slope. */
    double w=clampd((HAC_OGS_BLEND_M-s)/(HAC_OGS_BLEND_M-HAC_OGS_FULL_M),0.0,1.0);
    if (w>0.0) {
        double h_line=x->c.h_station+s*tan(x->c.slope_deg*DEG2RAD);
        *h=fmin(w*h_line+(1.0-w)**h,e-floor_v*floor_v/(2.0*TEL_G0));
    }
    *v=sqrt(fmax(1.0,2.0*TEL_G0*(e-*h)));
}

/* Energy height lost per ground metre on the reference at energy e. */
static double hac_eta(const HacCtx *x,const TaemHacReference *R,double e,double s,
        double bank_deg) {
    double h,v,q,mach;
    hac_split_ref(x,R,e,s,&h,&v);
    tel_air(&x->c,h,v,&q,&mach);
    double cphi=cos(fmin(fabs(bank_deg),70.0)*DEG2RAD);
    double aoa=hac_lift_aoa(x,mach,q,x->mass*tel_gravity(&x->c,h)/cphi);
    double cl,cd;
    hac_aero(x,mach,aoa,&cl,&cd);
    return cl>1e-3?cd/cl/cphi:3.0;
}

static void hac_reference_build(TaemHacReference *R,const HacCtx *x,double ut) {
    R->ds=HAC_REF_DS;
    R->mass=x->mass;
    R->built_ut=ut;
    hac_reference_incidence(R,x);
    /* Deliver a small surplus as speed on the glide: a shortfall at the
       station is unrecoverable (Final stretches into the stall), a surplus is
       Final's speedbrake trim. */
    R->e[0]=x->c.e_station+HAC_STATION_MARGIN_M;
    R->h[0]=x->c.h_station;
    R->v[0]=sqrt(2.0*TEL_G0*(R->e[0]-R->h[0]));
    for (int i=1;i<TAEM_HAC_REF_N;++i) {
        double s=(i-1)*R->ds,e=R->e[i-1];
        double k1=hac_eta(x,R,e,s,0.0);
        double k2=hac_eta(x,R,e+0.5*R->ds*k1,s+0.5*R->ds,0.0);
        R->e[i]=e+R->ds*k2;
        hac_split_ref(x,R,R->e[i],s+R->ds,&R->h[i],&R->v[i]);
    }
    R->valid=true;
    if (getenv("KSP_LANDER_HAC_REF_TRACE")) {
        static _Thread_local int dumped=0;
        if (!dumped++) for (int i=0;i<TAEM_HAC_REF_N;i+=(i<40?2:20)) {
            double q,mach;
            tel_air(&x->c,R->h[i],R->v[i],&q,&mach);
            fprintf(stderr,"HAC_REF s=%.0f e=%.0f h=%.0f v=%.1f M=%.2f q=%.0f\n",
                i*R->ds,R->e[i],R->h[i],R->v[i],mach,q);
        }
    }
}

static void hac_reference_at(const TaemHacReference *R,double s,double *e,double *h,
        double *v,double *dhds,double *deds) {
    double u=s/R->ds;
    int i=(int)floor(u);
    if (i<0) i=0;
    if (i>TAEM_HAC_REF_N-2) i=TAEM_HAC_REF_N-2;
    double f=u-i;   /* extrapolates linearly beyond either end */
    double de=(R->e[i+1]-R->e[i])/R->ds,dh=(R->h[i+1]-R->h[i])/R->ds;
    if (e) *e=R->e[i]+f*(R->e[i+1]-R->e[i]);
    if (h) *h=R->h[i]+f*(R->h[i+1]-R->h[i]);
    if (v) *v=R->v[i]+f*(R->v[i+1]-R->v[i]);
    if (dhds) *dhds=dh;
    if (deds) *deds=de;
}

static double hac_radius_at(double rf,double psi) { return rf+HAC_R2*psi*psi; }
static double hac_arc_length(double rf,double psi) {
    return rf*psi+HAC_R2*psi*psi*psi/3.0;
}
static double hac_exit_angle(int side) { return -side*TEL_PI/2.0; }

/* Remaining turn angle [0,2pi) of a point seen from the HAC centre. */
static double hac_point_psi(int side,double rf,double D,double px,double py) {
    double ang=atan2(py-side*rf,px+D);
    double psi=fmod(side*(hac_exit_angle(side)-ang),2.0*TEL_PI);
    return psi<0.0?psi+2.0*TEL_PI:psi;
}

/* Tangent point on the spiral from (px,py), turning toward `side`. */
static bool hac_spiral_tangent(int side,double rf,double D,double px,double py,
        double *tx,double *ty,double *psi_t) {
    double cx=-D,cy=side*rf,dx=px-cx,dy=py-cy,d=hypot(dx,dy);
    double psi=hac_point_psi(side,rf,D,px,py);
    for (int i=0;i<8;++i) {
        double r=hac_radius_at(rf,psi);
        if (d<=r*1.001) return false;
        double ang=atan2(dy,dx)+side*acos(r/d);
        double next=fmod(side*(hac_exit_angle(side)-ang),2.0*TEL_PI);
        if (next<0.0) next+=2.0*TEL_PI;
        psi=next;
        *tx=cx+r*cos(ang);
        *ty=cy+r*sin(ang);
    }
    *psi_t=psi;
    return true;
}

typedef struct {
    bool valid;
    double s_go, psi_t, tx, ty, tangent_dist, acq_turn, acq_arc, hac_arc, turn_cost, e_ref;
} HacPrediction;

/* Bank-angle cost of the planned turns on top of the wings-level reference:
   the polar priced at the banked lift, not a fixed 1/cos factor. */
static double hac_turn_cost(const HacCtx *x,const TaemHacReference *R,double rf,
        double psi_t,double acq_arc,double acq_bank_deg,double s_acq) {
    double cost=0.0;
    const int n=12;
    for (int i=0;i<n;++i) {
        double p0=psi_t*i/n,p1=psi_t*(i+1)/n,pm=0.5*(p0+p1);
        double sm=hac_arc_length(rf,pm),e,v;
        hac_reference_at(R,sm,&e,NULL,&v,NULL,NULL);
        double bank=fmin(60.0,atan(v*v/(TEL_G0*hac_radius_at(rf,pm)))*RAD2DEG);
        cost+=fmax(0.0,hac_eta(x,R,e,sm,bank)-hac_eta(x,R,e,sm,0.0))*
            (hac_arc_length(rf,p1)-hac_arc_length(rf,p0));
    }
    if (acq_arc>0.0) {
        double e;
        hac_reference_at(R,s_acq,&e,NULL,NULL,NULL,NULL);
        cost+=fmax(0.0,hac_eta(x,R,e,s_acq,acq_bank_deg)-hac_eta(x,R,e,s_acq,0.0))*acq_arc;
    }
    return cost;
}

static double hac_bank_limit(double mach,int phase) {
    bool sup=mach>1.0;
    if (phase==TAEM_HAC_PHASE_HAC) return sup?55.0:60.0;
    return sup?45.0:50.0;
}

/* GTP from the acquisition state: turn onto the tangent, fly it, then the spiral. */
static HacPrediction hac_predict_acquisition(const HacCtx *x,const TaemHacReference *R,
        int side,double rf,const TelPoint *s) {
    HacPrediction p={0};
    double D=x->c.d_station,tx,ty,psi_t;
    if (!hac_spiral_tangent(side,rf,D,s->x,s->y,&tx,&ty,&psi_t)) return p;
    double q,mach;
    tel_air(&x->c,s->h,s->v,&q,&mach);
    double vh=fmax(1.0,s->ground_speed>0.0?s->ground_speed:s->v*cos(s->g));
    double bank=0.8*hac_bank_limit(mach,TAEM_HAC_PHASE_ACQ);
    double r_acq=vh*vh/(TEL_G0*tan(bank*DEG2RAD));
    /* GTP acquisition: the velocity turns through DPSAC at the bank-limited
       radius (ARCAC), then flies RC to the tangent; the turn's own advance
       along the line of sight is credited, not iterated (a Mach 2.5 turn
       radius of ~100 km makes an iterated exit point unstable). */
    double dist=hypot(tx-s->x,ty-s->y),turn=tel_wrap(atan2(ty-s->y,tx-s->x)-s->psi);
    double advance=r_acq*sin(fmin(fabs(turn),0.5*TEL_PI));
    p.valid=true;
    p.psi_t=psi_t; p.tx=tx; p.ty=ty;
    p.acq_turn=turn;
    p.acq_arc=r_acq*fabs(turn);
    p.tangent_dist=fmax(0.0,dist-advance);
    p.hac_arc=hac_arc_length(rf,psi_t);
    p.s_go=p.acq_arc+p.tangent_dist+p.hac_arc;
    p.turn_cost=hac_turn_cost(x,R,rf,psi_t,p.acq_arc,bank,p.s_go);
    double e;
    hac_reference_at(R,p.s_go,&e,NULL,NULL,NULL,NULL);
    p.e_ref=e+p.turn_cost;
    return p;
}

static double hac_energy(const TelPoint *s) { return s->h+s->v*s->v/(2.0*TEL_G0); }

/* Energy bands above the station (m): dive limit, S-turn, stretch limit. */
static double hac_band_high(const HacCtx *x,double e_ref) {
    return 1000.0+0.06*fmax(0.0,e_ref-x->c.e_station);
}
static double hac_band_sturn(const HacCtx *x,double e_ref) {
    return 1500.0+0.10*fmax(0.0,e_ref-x->c.e_station);
}
static double hac_band_low(const HacCtx *x,double e_ref) {
    return 600.0+0.03*fmax(0.0,e_ref-x->c.e_station);
}

/* Shortfall is unrecoverable, surplus is dissipated by S-turns. */
static double hac_energy_cost(double e_err) { return e_err>=0.0?e_err:2.5*-e_err; }

/* Best spiral exit radius for one side: the predicted energy error nearest zero. */
static HacPrediction hac_size_side(const HacCtx *x,const TaemHacReference *R,int side,
        const TelPoint *s,double *rf_out) {
    static const double radii[]={2200.0,2600.0,3000.0,3500.0,4000.0,4500.0,5000.0,
        5500.0,6000.0,7000.0,8000.0,9000.0};
    HacPrediction best={0};
    double best_cost=INFINITY,e=hac_energy(s);
    double rf_min=fmax(HAC_RF_MIN,x->c.v_station*x->c.v_station/(TEL_G0*tan(50.0*DEG2RAD)));
    for (size_t i=0;i<sizeof(radii)/sizeof(radii[0]);++i) {
        double rf=fmax(rf_min,radii[i]);
        HacPrediction p=hac_predict_acquisition(x,R,side,rf,s);
        if (!p.valid) continue;
        double cost=hac_energy_cost(e-p.e_ref);
        if (cost<best_cost) { best_cost=cost; best=p; *rf_out=rf; }
    }
    return best;
}

static void hac_context(HacCtx *x,const TerminalModel *control,const AeroTable *recorded,
        const TaemHacLiveAero *live,const TaemGeometryState *geo,double mass,
        double aoa_deg,double bank_deg,TelPoint *s) {
    tel_setup(control,geo,mass,aoa_deg,bank_deg,&x->c,s);
    x->aero=recorded;
    x->live=live;
    x->mass=mass;
}

/* Choose side and exit radius: the pair whose predicted energy is nearest the
   reference (overhead or straight-in falls out of the side).  Returns cost. */
static double hac_select(TaemHacLawState *H,const HacCtx *x,const TelPoint *s,bool keep_bias) {
    double e=hac_energy(s),best=INFINITY;
    int best_side=H->side?H->side:1;
    double best_rf=H->rf;
    for (int side=-1;side<=1;side+=2) {
        double rf=H->rf;
        HacPrediction p=hac_size_side(x,&H->ref,side,s,&rf);
        if (!p.valid||p.psi_t>2.0*TEL_PI-0.05) continue;
        double cost=hac_energy_cost(e-p.e_ref);
        if (keep_bias&&side!=H->side) cost+=HAC_SELECT_MARGIN_M;
        if (cost<best) { best=cost; best_side=side; best_rf=rf; }
    }
    if (isfinite(best)) { H->side=best_side; H->rf=best_rf; }
    return best;
}

static void hac_initialize(TaemHacLawState *H,const HacCtx *x,const TelPoint *s,double ut) {
    *H=(TaemHacLawState){.initialized=true,.phase=TAEM_HAC_PHASE_ACQ,.side=s->y<=0.0?1:-1,
        .rf=clampd(x->c.v_station*x->c.v_station/(TEL_G0*tan(HAC_BANK_NOMINAL*DEG2RAD)),
            HAC_RF_MIN,HAC_RF_MAX),
        .last_ang=NAN,.last_sgo=NAN,.last_sgo_ut=-INFINITY,.last_plan_ut=-INFINITY,
        .last_side_ut=ut,.last_ut=ut,.eta_live=NAN,.sdot=NAN,.tangent_dist=INFINITY};
    hac_reference_build(&H->ref,x,ut);
    (void)hac_select(H,x,s,false);
}

/* TGNZC: altitude tracking with energy and speed mid-value limits -> incidence. */
static double hac_vertical(const TaemHacLawState *H,const HacCtx *x,const TelPoint *s,
        double bank_deg,double drag_accel) {
    double q,mach;
    tel_air(&x->c,s->h,s->v,&q,&mach);
    double e=hac_energy(s),dhds;
    hac_reference_at(&H->ref,H->s_go,NULL,NULL,NULL,&dhds,NULL);
    double vh=s->v*cos(s->g);
    double sdot=isfinite(H->sdot)?fmin(0.0,H->sdot):-vh;
    double hdot=s->v*sin(s->g),hdot_ref=dhds*sdot;
    /* Shuttle altitude loop: wn 0.054-0.179 rising as altitude falls, zeta 0.9 */
    double wn=0.06+0.09*clampd((20000.0-s->h)/17000.0,0.0,1.0),z=0.9;
    double damp=2.0*z*wn*(hdot_ref-hdot);
    double hdd_h=wn*wn*(H->h_ref-s->h)+damp;
    double hdd_hi=wn*wn*(H->e_ref+hac_band_high(x,H->e_ref)-e)+damp;
    double hdd_lo=wn*wn*(H->e_ref-hac_band_low(x,H->e_ref)-e)+damp;
    double hdd=hdd_h;
    if (hdd>hdd_hi) hdd=hdd_hi;   /* energy high: do not climb away from a dive */
    if (hdd<hdd_lo) hdd=hdd_lo;   /* energy low: do not dive to the altitude */
    double grav=tel_gravity(&x->c,s->h),v=fmax(s->v,1.0);
    double vdot=-drag_accel-grav*sin(s->g);
    double gdot=(hdd-vdot*sin(s->g))/(v*fmax(0.2,cos(s->g)));
    /* speed limits outrank energy: below the turning speed recover it (4 s),
       above the reference band unload into a climb */
    double safe=x->c.model->vehicle.minimum_safe_speed;
    double cphi=fmax(0.3,cos(fabs(bank_deg)*DEG2RAD));
    double v_min=safe>0.0?1.1*safe/sqrt(cphi):0.0;
    if (v_min>0.0&&s->v<v_min+15.0) {
        double need=drag_accel+(v_min+15.0-s->v)/4.0;
        double g_max=-asin(clampd(need/grav,0.0,1.0));
        gdot=fmin(gdot,(g_max-s->g)/3.0);
    }
    double q_cap=fmin(x->c.model->vehicle.maximum_dynamic_pressure*0.5,
        2.0*hac_q_ref(x,&H->ref,mach,H->s_go));
    if (q>q_cap) {
        double v_cap=s->v*sqrt(q_cap/q);
        double need=-(s->v-v_cap)/4.0-drag_accel;
        double g_min=-asin(clampd(need/grav,-1.0,1.0));
        gdot=fmax(gdot,(g_min-s->g)/3.0);
    }
    if (s->g<-35.0*DEG2RAD) gdot=fmax(gdot,0.0);
    if (s->g>6.0*DEG2RAD) gdot=fmin(gdot,0.0);
    gdot=clampd(gdot,-0.08,0.08);
    double lift=s->m*(grav*cos(s->g)+v*gdot)/cphi;
    return clampd(hac_lift_aoa(x,mach,q,lift),TEL_AOA_MIN,x->c.aoa_max);
}

/* TGPHIC: positive return is a LEFT bank (callers negate). */
static double hac_lateral_left(TaemHacLawState *H,const HacCtx *x,const TelPoint *s,
        const HacPrediction *acq) {
    double q,mach;
    tel_air(&x->c,s->h,s->v,&q,&mach);
    double vh=fmax(1.0,s->ground_speed>0.0?s->ground_speed:s->v*cos(s->g));
    double D=x->c.d_station,lim=hac_bank_limit(mach,H->phase),bank=0.0;
    if (H->phase==TAEM_HAC_PHASE_STURN) {
        /* turn away from the HAC side; level once the tangent is abeam */
        double dpsi=acq&&acq->valid?tel_wrap(atan2(acq->ty-s->y,acq->tx-s->x)-s->psi):0.0;
        bank=fabs(dpsi)<0.5*TEL_PI?H->s_turn_sign*lim:0.0;
    } else if (H->phase==TAEM_HAC_PHASE_ACQ) {
        double dpsi=acq&&acq->valid?tel_wrap(atan2(acq->ty-s->y,acq->tx-s->x)-s->psi):
            tel_wrap(atan2(H->side*H->rf-s->y,-D-s->x)-s->psi);
        bank=clampd(HAC_GPHI*dpsi*RAD2DEG,-lim,lim);
    } else if (H->phase==TAEM_HAC_PHASE_HAC) {
        double cx=-D,cy=H->side*H->rf,rx=s->x-cx,ry=s->y-cy,rc=hypot(rx,ry);
        double r=hac_radius_at(H->rf,H->psi);
        double rdot=(rx*vh*cos(s->psi)+ry*vh*sin(s->psi))/fmax(1.0,rc);
        double rdot_ref=-vh*2.0*HAC_R2*H->psi/fmax(1.0,r);
        /* wn 0.074, zeta 1.5; centrifugal feed-forward */
        double acc=vh*vh/r+0.0055*(rc-r)+0.222*(rdot-rdot_ref);
        bank=H->side*clampd(atan(acc/(TEL_G0*cos(s->g)))*RAD2DEG,0.0,lim);
    } else {
        double ydot=vh*sin(s->psi);
        double acc=-(0.0225*s->y+0.219*ydot);   /* wn 0.15, zeta 0.73 */
        bank=atan(acc/(TEL_G0*cos(s->g)))*RAD2DEG;
        lim=clampd(0.43*fabs(bank)+17.0,30.0,60.0);
        bank=clampd(bank,-lim,lim);
    }
    lim=fmin(lim,x->c.model->vehicle.maximum_bank_angle);
    double lift,drag;
    tel_forces(&x->c,s,s->aoa,&q,&mach,&lift,&drag);
    lim=entry_bank_authority_limit(s->v,q,lift/(s->m*TEL_G0),&x->c.model->vehicle,lim);
    return clampd(bank,-lim,lim);
}

/* One guidance tick of phase logic (TGTRAN), references (TGCOMP) and spiral
   adjustment.  Returns the acquisition prediction used for steering. */
static HacPrediction hac_update(TaemHacLawState *H,const HacCtx *x,const TelPoint *s,
        double ut,double drag_accel) {
    double D=x->c.d_station,e=hac_energy(s);
    double dt=ut>H->last_ut?fmin(ut-H->last_ut,1.0):0.0;
    H->last_ut=ut;
    if (!H->ref.valid||ut-H->ref.built_ut>=HAC_REF_REBUILD_S||
            fabs(H->ref.mass-x->mass)>0.01*x->mass)
        hac_reference_build(&H->ref,x,ut);
    /* measured energy height lost per ground metre */
    double vh=fmax(1.0,s->v*cos(s->g));
    double eta=drag_accel*s->v/(TEL_G0*vh);
    if (isfinite(eta)&&dt>0.0)
        H->eta_live=isfinite(H->eta_live)?H->eta_live+dt/(5.0+dt)*(eta-H->eta_live):eta;

    HacPrediction acq={0};
    if (H->phase<=TAEM_HAC_PHASE_ACQ) {
        if (ut-H->last_side_ut>=HAC_UPDATE_S&&H->tangent_dist>HAC_SELECT_FREEZE_M) {
            H->last_side_ut=ut;
            (void)hac_select(H,x,s,true);
        }
        acq=hac_predict_acquisition(x,&H->ref,H->side,H->rf,s);
        double cx=-D,cy=H->side*H->rf,rc=hypot(s->x-cx,s->y-cy);
        /* TGTRAN: start the turn within 10% of the spiral radius at the turn
           angle the tangent joins it (the position angle alone is ambiguous
           by the tangent offset until the vehicle is on the circle) */
        double psi_j=acq.valid?acq.psi_t:hac_point_psi(H->side,H->rf,D,s->x,s->y);
        bool inside=!acq.valid||rc<1.1*hac_radius_at(H->rf,psi_j);
        if (inside) {
            H->phase=TAEM_HAC_PHASE_HAC;
            H->psi=psi_j;
            if (H->psi>2.0*TEL_PI-0.3) H->psi=0.0;
            H->last_ang=atan2(s->y-cy,s->x-cx);
        } else {
            H->s_go=acq.s_go;
            H->turn_cost=acq.turn_cost;
            H->tangent_x=acq.tx; H->tangent_y=acq.ty;
            H->tangent_dist=acq.tangent_dist+acq.acq_arc;
            H->psi=acq.psi_t;
            H->e_ref=acq.e_ref;
        }
    }
    if (H->phase==TAEM_HAC_PHASE_HAC) {
        double cx=-D,cy=H->side*H->rf,ang=atan2(s->y-cy,s->x-cx);
        /* the turn angle only shrinks along the HAC; a vehicle cut inside
           the cone must not wind it up by whole turns */
        if (isfinite(H->last_ang)) H->psi=clampd(H->psi-H->side*tel_wrap(ang-H->last_ang),
            0.0,H->psi);
        H->last_ang=ang;
        double e_hac;
        hac_reference_at(&H->ref,hac_arc_length(H->rf,H->psi),&e_hac,NULL,NULL,NULL,NULL);
        H->s_go=hac_arc_length(H->rf,H->psi);
        H->turn_cost=hac_turn_cost(x,&H->ref,H->rf,H->psi,0.0,0.0,0.0);
        H->e_ref=e_hac+H->turn_cost;
        H->tangent_dist=0.0;
        /* Spiral adjustment: path change that nulls the energy error at the
           measured-polar dissipation of the remaining arc. */
        double err=e-H->e_ref;
        if (H->psi>0.5*TEL_PI&&fabs(err)>300.0&&dt>0.0) {
            double deds;
            hac_reference_at(&H->ref,hac_arc_length(H->rf,0.5*H->psi),NULL,NULL,NULL,NULL,&deds);
            double rf_target=H->rf+err/(fmax(0.05,deds)*H->psi);
            /* never tighter than the measured speed can turn at the sizing
               bank: a cone cut inside stops measuring the turn */
            double rf_min=fmax(fmax(HAC_RF_MIN,x->c.v_station*x->c.v_station/
                (TEL_G0*tan(50.0*DEG2RAD))),s->v*s->v/(TEL_G0*tan(HAC_BANK_SIZING*DEG2RAD))-
                HAC_R2*H->psi*H->psi);
            /* the floor is approached at the same rate: a jump moves the
               centre under the vehicle */
            rf_target=clampd(rf_target,rf_min,HAC_RF_MAX);
            double rf_new=clampd(H->rf+clampd(0.15*(rf_target-H->rf),-HAC_RF_RATE,HAC_RF_RATE)*dt,
                HAC_RF_MIN,HAC_RF_MAX);
            /* the exit point stays on the runway line; re-derive the turn
               angle about the moved centre, unwrapped next to the old one */
            double moved=fabs(rf_new-H->rf)/fmax(1.0,H->rf);
            H->rf=rf_new;
            double p=hac_point_psi(H->side,H->rf,D,s->x,s->y);
            p+=2.0*TEL_PI*round((H->psi-p)/(2.0*TEL_PI));
            H->psi=clampd(p,0.0,H->psi+moved);
            H->last_ang=atan2(s->y-H->side*H->rf,s->x-cx);
        }
        double rc=hypot(s->x-cx,s->y-H->side*H->rf);
        if (rc-hac_radius_at(H->rf,H->psi)>HAC_RERRLM) {
            /* far outside: reacquire (TGTRAN reverts to the acquisition law) */
            H->phase=TAEM_HAC_PHASE_ACQ;
            H->tangent_dist=0.0;
        }
        if (H->psi<0.05||(s->x>=-D-500.0&&fabs(tel_wrap(s->psi))<15.0*DEG2RAD&&
                fabs(s->y)<1500.0))
            H->phase=TAEM_HAC_PHASE_PREFINAL;
    }
    if (H->phase==TAEM_HAC_PHASE_PREFINAL) {
        H->s_go=-D-s->x;
        H->turn_cost=0.0;
        hac_reference_at(&H->ref,H->s_go,&H->e_ref,NULL,NULL,NULL,NULL);
    }
    hac_split_ref(x,&H->ref,H->e_ref,fmax(0.0,H->s_go),&H->h_ref,&H->v_ref);
    H->e_err=e-H->e_ref;
    H->energy_level=clampd(0.5+H->e_err/(2.0*hac_band_high(x,H->e_ref)),0.0,1.0);
    if (isfinite(H->last_sgo)&&ut>H->last_sgo_ut) {
        double rate=(H->s_go-H->last_sgo)/(ut-H->last_sgo_ut);
        if (isfinite(rate)&&fabs(rate)<3000.0)
            H->sdot=isfinite(H->sdot)?H->sdot+0.3*(rate-H->sdot):rate;
    }
    H->last_sgo=H->s_go; H->last_sgo_ut=ut;

    /* TGTRAN S-turn: energy above the S-turn line, far enough out, small turn */
    if (H->phase==TAEM_HAC_PHASE_ACQ&&H->e_err>hac_band_sturn(x,H->e_ref)&&
            H->s_go>HAC_STURN_MIN_RANGE&&H->psi<HAC_STURN_MAX_PSI) {
        H->phase=TAEM_HAC_PHASE_STURN;
        H->s_turn_sign=-H->side;
    } else if (H->phase==TAEM_HAC_PHASE_STURN&&
            (H->e_err<0.6*hac_band_sturn(x,H->e_ref)||H->s_go<HAC_STURN_MIN_RANGE))
        H->phase=TAEM_HAC_PHASE_ACQ;
    H->final_line=H->phase==TAEM_HAC_PHASE_PREFINAL&&tel_final_line_ready(&x->c,s);
    return acq;
}

/* TGSBC: split-rudder speedbrake opens on subsonic energy surplus. */
static double hac_speedbrake(TaemHacLawState *H,double mach,double dt) {
    if (mach>0.95||H->phase<TAEM_HAC_PHASE_ACQ||H->final_line||H->s_go<1000.0) {
        H->speedbrake_integral=0.0;
        return 0.0;
    }
    double err=H->e_err-200.0;
    H->speedbrake_integral=clampd(H->speedbrake_integral+2.0e-5*err*dt,0.0,SPEEDBRAKE_CAP);
    return clampd(err/2500.0+H->speedbrake_integral,0.0,SPEEDBRAKE_CAP);
}

/* Physical cost of starting in this runway frame: energy distance of the best
   HAC from its reference (shortfall weighted). */
static double taem_hac_law_frame_score(const TerminalModel *model,const AeroTable *recorded,
        const TaemHacLiveAero *live,const TaemGeometryState *geo,double mass_kg,
        double aoa_deg,double bank_deg,bool *delivery_feasible) {
    if (!model||!geo||!(mass_kg>0.0)||!isfinite(geo->runway_along_m)) return INFINITY;
    HacCtx x;TelPoint s;
    hac_context(&x,model,recorded,live,geo,mass_kg,aoa_deg,bank_deg,&s);
    static _Thread_local TaemHacLawState H;
    hac_initialize(&H,&x,&s,0.0);
    HacPrediction p=hac_predict_acquisition(&x,&H.ref,H.side,H.rf,&s);
    if (!p.valid) return INFINITY;
    double err=hac_energy(&s)-p.e_ref;
    if (getenv("KSP_LANDER_HAC_TRACE")) {
        double e0;
        hac_reference_at(&H.ref,p.s_go,&e0,NULL,NULL,NULL,NULL);
        fprintf(stderr,"HAC_FRAME x=%.0f y=%.0f psi=%.1f side=%+d RF=%.0f psiT=%.0f acqTurn=%.1f acqArc=%.0f tan=%.0f hacArc=%.0f sGo=%.0f Eref0=%.0f turn=%.0f E=%.0f err=%.0f\n",
            s.x,s.y,s.psi*RAD2DEG,H.side,H.rf,p.psi_t*RAD2DEG,p.acq_turn*RAD2DEG,p.acq_arc,
            p.tangent_dist,p.hac_arc,p.s_go,e0,p.turn_cost,hac_energy(&s),err);
    }
    if (delivery_feasible) *delivery_feasible=err>-hac_band_low(&x,p.e_ref);
    return hac_energy_cost(err);
}
