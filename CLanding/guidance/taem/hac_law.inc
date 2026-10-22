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
 *   Supersonic, ahead of these, a drag-energy phase (not in Moore): this
 *   airframe's supersonic L/D is under 1, so energy is shed with drag as in
 *   entry until the subsonic gate on the reference.
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

enum { TAEM_HAC_PHASE_DRAG=0, TAEM_HAC_PHASE_STURN, TAEM_HAC_PHASE_ACQ, TAEM_HAC_PHASE_HAC,
    TAEM_HAC_PHASE_PREFINAL };

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
#define HAC_BANK_OUTWARD 30.0
#define HAC_RERRLM 2100.0
#define HAC_STURN_MIN_RANGE 15000.0
#define HAC_STURN_MAX_PSI (200.0*DEG2RAD)
#define HAC_SELECT_FREEZE_M 10000.0
#define HAC_SELECT_MARGIN_M 1500.0
#define HAC_UPDATE_S 2.0
/* Drag-energy phase: subsonic gate on the reference, exit Mach, shortest
   path the required drag is spread over, load and bank limits, flight-path
   time constant and the bank-reversal heading deadband. */
#define HAC_GATE_MACH 0.95
#define HAC_DRAG_EXIT_MACH 1.0
#define HAC_DRAG_MIN_RUN 3000.0
#define HAC_DRAG_MAX_LOAD 2.5
#define HAC_DRAG_BANK_MAX 70.0
#define HAC_DRAG_GAMMA_TAU 6.0
#define HAC_DRAG_DEADBAND_DEG 15.0
#define HAC_DRAG_STURN_DEG 45.0     /* S-turn heading swing off the gate bearing */
#define HAC_DRAG_LOOKAHEAD 15000.0  /* centreline capture distance */
#define HAC_DRAG_AOA_SLEW 2.0       /* deg/s: the handoff incidence is MM304's (to 28 deg) */

typedef struct {
    TelContext c;
    const AeroTable *aero;          /* recorded polar (unscaled) */
    const TaemHacLiveAero *live;
    double mass;
    double aoa_supersonic;          /* configured limit; the trim ceiling starts subsonic */
} HacCtx;

static int hac_bin(double mach) {
    int i=(int)floor(fmax(0.0,mach)/0.25);
    return i<0?0:(i>=TAEM_HAC_KAPPA_BINS?TAEM_HAC_KAPPA_BINS-1:i);
}

/* Live/recorded ratio at this Mach: the bin's own measurement, else the
   nearest measured bin on the same side of Mach 1, fading toward the polar
   with Mach distance.  A transonic drag rise is no property of the subsonic
   polar: live, kD 1.28 measured at Mach 1.0-1.25 was lent to Mach 0.9 and
   steepened the subsonic reference to 0.94 m/m (flown: 0.49). */
static double hac_kappa(const TaemHacLiveAero *A,double mach,bool lift) {
    if (!A) return 1.0;
    int b=hac_bin(mach);
    const double *r=lift?A->lift:A->drag;
    if (A->weight[b]>0.0&&r[b]>0.0) return r[b];
    double best=INFINITY,value=1.0;
    for (int i=0;i<TAEM_HAC_KAPPA_BINS;++i) {
        if (!(A->weight[i]>0.0&&r[i]>0.0)) continue;
        if (((i+0.5)*0.25<1.0)!=(mach<1.0)) continue;
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

static double hac_lift_aoa_to(const HacCtx *x,double mach,double q,double lift_needed,
        double hi) {
    double area=x->aero->reference_area_m2,lo=-5.0;
    for (int i=0;i<30;++i) {
        double mid=0.5*(lo+hi),cl,cd;
        hac_aero(x,mach,mid,&cl,&cd);
        if (cl*q*area<lift_needed) lo=mid; else hi=mid;
    }
    return 0.5*(lo+hi);
}
/* Incidence of the live-corrected polar's best L/D, up to `hi`. */
static double hac_best_ld_aoa(const HacCtx *x,double mach,double hi) {
    double best_aoa=hi,best=-INFINITY;
    for (double a=TEL_AOA_MIN;a<=hi+1e-9;a+=0.25) {
        double cl,cd;
        hac_aero(x,mach,a,&cl,&cd);
        if (cd>1e-4&&cl/cd>best) { best=cl/cd; best_aoa=a; }
    }
    return best_aoa;
}
static double hac_lift_aoa(const HacCtx *x,double mach,double q,double lift_needed) {
    return hac_lift_aoa_to(x,mach,q,lift_needed,x->c.aoa_max);
}

/* Drag-phase incidence ceiling: the vehicle's supersonic limit, blending to
   the subsonic maximum-lift incidence through Mach 1.5-1.0. */
static double hac_drag_aoa_ceiling(const HacCtx *x,double mach) {
    double sup=fmax(x->c.aoa_max,x->aoa_supersonic);
    return x->c.aoa_max+(sup-x->c.aoa_max)*clampd((mach-1.0)/0.5,0.0,1.0);
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
    memcpy(R->h_eq,R->h,sizeof(R->h));
    memcpy(R->v_eq,R->v,sizeof(R->v));
    /* Subsonic gate: where the reference first reaches HAC_GATE_MACH. */
    R->s_gate=0.0; R->e_gate=R->e[0]; R->h_gate=R->h[0];
    for (int i=1;i<TAEM_HAC_REF_N;++i) {
        double q,m0,m1;
        tel_air(&x->c,R->h[i-1],R->v[i-1],&q,&m0);
        tel_air(&x->c,R->h[i],R->v[i],&q,&m1);
        if (m1>=HAC_GATE_MACH&&m0<HAC_GATE_MACH) {
            double f=(HAC_GATE_MACH-m0)/fmax(1e-9,m1-m0);
            R->s_gate=(i-1+f)*R->ds;
            R->e_gate=R->e[i-1]+f*(R->e[i]-R->e[i-1]);
            R->h_gate=R->h[i-1]+f*(R->h[i]-R->h[i-1]);
            /* Beyond the gate the drag-energy phase owns the supersonic
               flight, so the reference stays the subsonic glide: the gate's
               slope, surplus as altitude.  The equilibrium reference there
               climbs at the transonic drag rise (2+ m/m) and reads a vehicle
               just through Mach 1 as kilometres short. */
            double slope=INFINITY,v_gate=R->v[i-1]+f*(R->v[i]-R->v[i-1]);
            for (int j=i/2>1?i/2:1;j<i;++j) slope=fmin(slope,(R->e[j]-R->e[j-1])/R->ds);
            if (!isfinite(slope)) slope=(R->e[i]-R->e[i-1])/R->ds;
            for (int k=i;k<TAEM_HAC_REF_N;++k) {
                R->e[k]=R->e_gate+slope*(k*R->ds-R->s_gate);
                R->h[k]=R->h_gate+(R->e[k]-R->e_gate);
                R->v[k]=v_gate;
            }
            break;
        }
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
    x->aoa_supersonic=x->c.aoa_max;
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
    double q,mach;
    tel_air(&x->c,s->h,s->v,&q,&mach);
    if (mach>HAC_DRAG_EXIT_MACH) {
        H->phase=TAEM_HAC_PHASE_DRAG;
        H->drag_flown=true;
        H->drag_aoa_cmd=s->aoa;
        H->drag_bank_cmd=NAN;
    }
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
    double aoa=hac_lift_aoa(x,mach,q,lift);
    /* energy low: stretch at best L/D; pulling past it to hold the altitude
       reference only bleeds speed on the back side */
    if (H->e_err<-hac_band_low(x,H->e_ref)&&s->v>v_min+15.0)
        aoa=fmin(aoa,tel_best_ld_aoa(&x->c,mach));
    return clampd(aoa,TEL_AOA_MIN,x->c.aoa_max);
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
        /* outward bank too: inside the spiral, wings level cuts the chord */
        bank=H->side*clampd(atan(acc/(TEL_G0*cos(s->g)))*RAD2DEG,-HAC_BANK_OUTWARD,lim);
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

/* Drag-energy phase.  Supersonic this airframe's L/D is 0.8-0.9, so a glide
   reference cannot be tracked by pitching down: a low-incidence dive is its
   minimum-drag (stretch) mode, which is how the 70 km quicksave replay grew a
   6.8 km surplus to 20-30 km and overflew.  As in entry, the surplus over the
   subsonic reference for the path still to fly is shed with drag before the
   gate (where the reference reaches Mach 0.95):
     - incidence: the drag that sheds that surplus over the path to the gate,
       up to the vehicle's supersonic incidence limit and 2.5 g;
     - bank magnitude: the lift the descent to the gate altitude does not
       need; the path outranks the drag (lift short of it raises incidence);
     - bank sign: steers along the approach centreline (past the gate, for the
       nearer HAC), with +-45 deg S-turns while even full drag cannot shed the
       surplus over that distance.
   The runway end is held: supersonic turn radii are 25-50 km, so a reciprocal
   end means overflying the field and looping back.
   Positive bank is LEFT, as hac_lateral_left. */
/* Path to the station as the subsonic law will fly it from here: the turn
   to the nearer HAC's tangent at the gate speed (supersonic it is priced at
   a 25-50 km radius the vehicle will not keep), the line and the HAC.
   Before the gate this is the straight-in path; past it, the turn back and
   the overhead HAC still to be flown. */
static void hac_drag_path(const TaemHacLawState *H,const HacCtx *x,const TelPoint *s,
        double *s_go,double *e_tgt,double *bearing) {
    double D=x->c.d_station,gx=-D-H->ref.s_gate;
    double v_gate;
    hac_reference_at(&H->ref,H->ref.s_gate,NULL,NULL,&v_gate,NULL,NULL);
    TelPoint sub=*s;
    if (s->v>v_gate) {
        double k=v_gate/s->v;
        sub.v*=k;
        sub.ground_speed*=k;
    }
    double best=INFINITY,best_e=NAN,bx=-D,by=0.0;
    for (int side=-1;side<=1;side+=2) {
        HacPrediction p=hac_predict_acquisition(x,&H->ref,side,H->rf,&sub);
        double len,e,tx,ty;
        if (p.valid) {
            len=p.s_go; e=p.e_ref; tx=p.tx; ty=p.ty;
        } else {
            /* inside this HAC: out to its rim and once round */
            len=H->rf+hac_arc_length(H->rf,2.0*TEL_PI);
            hac_reference_at(&H->ref,len,&e,NULL,NULL,NULL,NULL);
            e+=hac_turn_cost(x,&H->ref,H->rf,2.0*TEL_PI,0.0,0.0,len);
            tx=-D; ty=side*2.0*H->rf;
        }
        if (!isfinite(best_e)||e<best_e) { best=len; best_e=e; bx=tx; by=ty; }
    }
    /* The supersonic flight still to come is priced at its own rate, not the
       subsonic glide's: from Mach M the speed down to Mach 1 is shed at best
       the polar's 1/(L/D) per metre (L/D under 1).  Priced at the 0.6 m/m
       glide, a live-orbit handoff reached Mach 1 short and the station
       0.7-1.3 km low. */
    double q,mach;
    tel_air(&x->c,s->h,s->v,&q,&mach);
    if (mach>1.0) {
        double m_mid=0.5*(mach+1.0),cl,cd;
        hac_aero(x,m_mid,hac_best_ld_aoa(x,m_mid,hac_drag_aoa_ceiling(x,m_mid)),&cl,&cd);
        double eta_sup=cl>1e-3?cd/cl:1.0,v1=s->v/mach,slope;
        hac_reference_at(&H->ref,best,NULL,NULL,NULL,NULL,&slope);
        double d=(s->v*s->v-v1*v1)/(2.0*TEL_G0*fmax(0.3,eta_sup));
        best_e+=fmax(0.0,eta_sup-slope)*d;
    }
    *s_go=best;
    *e_tgt=best_e;
    /* before the gate: the centreline ahead, never back to the gate point,
       which a supersonic vehicle would overrun and loop back to */
    if (s->x<gx) {
        bx=fmin(-D,fmax(gx,s->x+HAC_DRAG_LOOKAHEAD));
        by=0.0;
    }
    *bearing=atan2(by-s->y,bx-s->x);
}

static void hac_drag_command(TaemHacLawState *H,const HacCtx *x,const TelPoint *s,
        double *aoa_out,double *bank_left_out) {
    double q,mach;
    tel_air(&x->c,s->h,s->v,&q,&mach);
    double grav=tel_gravity(&x->c,s->h),v=fmax(s->v,1.0);
    double vh=fmax(1.0,s->ground_speed>0.0?s->ground_speed:v*cos(s->g));
    double area=x->aero->reference_area_m2;
    double s_go,bearing,e_tgt,deds;
    hac_drag_path(H,x,s,&s_go,&e_tgt,&bearing);
    hac_reference_at(&H->ref,s_go,NULL,NULL,NULL,NULL,&deds);
    /* the surplus over the reference for that path is shed before the gate */
    double run=fmax(HAC_DRAG_MIN_RUN,s_go-H->ref.s_gate);
    /* energy height per ground metre <-> drag acceleration (eta = D v / g vh) */
    double per_metre=grav*vh/v;
    double drag_req=fmax(0.0,deds+(hac_energy(s)-e_tgt)/run)*per_metre;
    double ceiling=hac_drag_aoa_ceiling(x,mach);
    double a_max=fmin(ceiling,hac_lift_aoa_to(x,mach,q,HAC_DRAG_MAX_LOAD*s->m*grav,ceiling));
    double cl,cd;
    hac_aero(x,mach,a_max,&cl,&cd);
    double drag_max=cd*q*area/s->m;
    double lo=TEL_AOA_MIN,hi=fmax(TEL_AOA_MIN,a_max);
    for (int i=0;i<30;++i) {
        double mid=0.5*(lo+hi);
        hac_aero(x,mach,mid,&cl,&cd);
        if (cd*q*area/s->m<drag_req) lo=mid; else hi=mid;
    }
    double aoa=0.5*(lo+hi);
    /* S-turn while full drag cannot shed the surplus over the distance:
       swing off the bearing and reverse on reaching the swing */
    if (drag_req>drag_max) H->drag_away=true;
    else if (drag_req<0.7*drag_max) H->drag_away=false;
    double dpsi_gate=tel_wrap(bearing-s->psi);
    if (!H->drag_away_sign) H->drag_away_sign=dpsi_gate>=0.0?-1:1;
    double swing=HAC_DRAG_STURN_DEG*DEG2RAD;
    if (H->drag_away&&H->drag_away_sign*-dpsi_gate>=swing-5.0*DEG2RAD)
        H->drag_away_sign=-H->drag_away_sign;
    double target=H->drag_away?bearing+H->drag_away_sign*swing:bearing;
    double dpsi=tel_wrap(target-s->psi);
    /* descend to the gate altitude over the longer of the distance to the
       gate and the distance full drag needs to shed the surplus: a dive
       sized on the distance alone overspeeds while turning away */
    double shed=drag_max>0.0?fmax(0.0,hac_energy(s)-e_tgt)*per_metre/drag_max:0.0;
    double g_ref=-atan2(fmax(0.0,s->h-H->ref.h_gate),fmax(run,shed));
    double gdot=clampd((g_ref-s->g)/HAC_DRAG_GAMMA_TAU,-0.08,0.08);
    if (s->g>3.0*DEG2RAD) gdot=fmin(gdot,0.0);
    if (s->g<-35.0*DEG2RAD) gdot=fmax(gdot,0.0);
    double lift_v=s->m*(grav*cos(s->g)+v*gdot);
    hac_aero(x,mach,aoa,&cl,&cd);
    double lift=cl*q*area;
    double lim=fmin(HAC_DRAG_BANK_MAX,x->c.model->vehicle.maximum_bank_angle);
    double steer=clampd(HAC_GPHI*dpsi*RAD2DEG,-lim,lim);
    double bank=lift>lift_v&&lift>0.0?
        acos(clampd(lift_v/lift,cos(lim*DEG2RAD),1.0))*RAD2DEG:0.0;
    /* reversals only outside the heading deadband: at 4.5-6 deg/s each one
       takes 20+ s */
    if (!H->drag_sign||fabs(dpsi)*RAD2DEG>HAC_DRAG_DEADBAND_DEG) {
        H->drag_sign=dpsi>=0.0?1:-1;
        bank=fmax(bank,fabs(steer));
    }
    double need=lift_v/fmax(0.2,cos(bank*DEG2RAD));
    if (lift<need) aoa=fmax(aoa,fmin(a_max,hac_lift_aoa_to(x,mach,q,need,ceiling)));
    /* Energy short of the path: the descent steepens rather than pulling
       past best L/D to hold it.  Live, 17-22 deg held the descent to the
       gate from Mach 1.5 to 1 at 1.29 m/m (L/D under 1) and the vehicle
       reached the station 1.3 km low. */
    if (hac_energy(s)<e_tgt&&s->g>-30.0*DEG2RAD)
        aoa=fmin(aoa,fmax(TEL_AOA_MIN,hac_best_ld_aoa(x,mach,ceiling)));
    double lift_now,drag_now;
    tel_forces(&x->c,s,s->aoa,&q,&mach,&lift_now,&drag_now);
    lim=entry_bank_authority_limit(s->v,q,lift_now/(s->m*TEL_G0),&x->c.model->vehicle,lim);
    H->drag_required=drag_req;
    *aoa_out=clampd(aoa,TEL_AOA_MIN,ceiling);
    *bank_left_out=H->drag_sign*clampd(bank,0.0,lim);
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
        /* well inside the selected HAC (the drag-energy phase ends wherever
           Mach 1 falls): choose again rather than capture a cone the vehicle
           is not on */
        if (!acq.valid&&H->phase!=TAEM_HAC_PHASE_DRAG&&
                hypot(s->x+D,s->y-H->side*H->rf)<0.9*hac_radius_at(H->rf,
                    hac_point_psi(H->side,H->rf,D,s->x,s->y))) {
            H->last_side_ut=ut;
            (void)hac_select(H,x,s,false);
            acq=hac_predict_acquisition(x,&H->ref,H->side,H->rf,s);
        }
        double cx=-D,cy=H->side*H->rf,rc=hypot(s->x-cx,s->y-cy);
        /* TGTRAN: start the turn within 10% of the spiral radius at the turn
           angle the tangent joins it (the position angle alone is ambiguous
           by the tangent offset until the vehicle is on the circle) */
        double psi_j=acq.valid?acq.psi_t:hac_point_psi(H->side,H->rf,D,s->x,s->y);
        /* ... and flying round it the HAC's way, within 60 deg of its tangent */
        double along_hac=tel_wrap(s->psi-(atan2(s->y-cy,s->x-cx)+H->side*0.5*TEL_PI));
        /* ... and at the tangent point: a shallow straight-in line runs
           within 10% of the radius for kilometres, and captured there it
           flew a chord inside the circle to an exit 470 m off the line (L7) */
        double vh_cap=fmax(1.0,s->ground_speed>0.0?s->ground_speed:s->v*cos(s->g));
        bool inside=(!acq.valid||(rc<1.1*hac_radius_at(H->rf,psi_j)&&
                acq.tangent_dist<fmax(500.0,4.0*vh_cap)))&&
            fabs(along_hac)<TEL_PI/3.0;
        if (inside&&H->phase!=TAEM_HAC_PHASE_DRAG) {
            H->phase=TAEM_HAC_PHASE_HAC;
            H->psi=psi_j;
            if (H->psi>2.0*TEL_PI-0.3) H->psi=0.0;
            H->last_ang=atan2(s->y-cy,s->x-cx);
        } else if (acq.valid) {
            /* an invalid prediction (inside the HAC, not yet flying round it)
               keeps the last path: its zeros read as a 15 km surplus */
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

    /* Drag-energy phase ends subsonic, at the gate, or once the energy is
       down to the gate's: below it the low-drag glide of the acquisition
       law is the stretch. */
    if (H->phase==TAEM_HAC_PHASE_DRAG) {
        double q,mach;
        tel_air(&x->c,s->h,s->v,&q,&mach);
        double s_go,bearing,e_tgt;
        hac_drag_path(H,x,s,&s_go,&e_tgt,&bearing);
        H->s_go=s_go;
        H->e_ref=e_tgt;
        H->e_err=e-e_tgt;
        if (mach<=HAC_DRAG_EXIT_MACH||e<=H->ref.e_gate) {
            H->phase=TAEM_HAC_PHASE_ACQ;
            if (H->trace)
                fprintf(stderr,"MM305 HAC drag-energy phase ends: UT %.1f Mach %.2f path %.1f km "
                    "E %.0f m path E %.0f m\n",ut,mach,s_go/1000.0,e,e_tgt);
        }
    }
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
