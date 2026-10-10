/*
 * Adaptive flare.
 *
 * The fixed flare (latch height, constant-deceleration arc, shallow glide,
 * attitude floor) assumes the incidence follows its command within about a
 * second.  Live STS flights showed 2-3 deg of incidence lag through the pull,
 * so the fixed arc started late, pulled hard, and either ballooned and floated
 * (L1) or touched down short (L3).
 *
 * Here the flare is sized from what the vehicle is doing now:
 *  1. Response identification.  Every Final cycle a decayed least-squares fit
 *     of   d(aoa)/dt = (gain*cmd + bias - aoa)/tau
 *     to the issued command and the measured incidence (with a prior so a
 *     quiet outer glide stays near nominal).  A stiff airframe against a slow
 *     trim integrator holds the incidence well short of a raised command for
 *     seconds: a gain below one, not a long lag.
 *  2. One flare law, shared by the forward predictor and the executed command:
 *     it tracks a sink-versus-height profile (arc, shallow glide, final flare)
 *     read one identified lag ahead, and compensates the command for the
 *     identified lag, gain and offset.
 *  3. A solver that simulates candidate profiles forward from the present
 *     state with the identified response and learned lift/drag, and keeps the
 *     cheapest under the touchdown constraints (sink, no climb, touchdown
 *     zone, speed floor, load).  The flare starts when the chosen profile's
 *     lead-read sink reaches its arc: a slow response starts it earlier.
 */

#define AF_TABLE_STEP .25
#define AF_TABLE_MAX 121
#define AF_FLOAT_HEIGHT 1.5
#define AF_FLOAT_SINK 1.0
#define AF_SGS_SINK 3.0
#define AF_SLEW 5.0          /* deg/s incidence command slew, as the executed flare */
#define AF_TD_PITCH 8.0
#define AF_FLOOR_START 10.0  /* wheel height where the mains-first attitude floor begins */
#define AF_FLOOR_FULL 3.0
#define AF_MAX_ONSET 600.0   /* wheel height above which the outer glide owns the energy */

typedef struct {
    double lq[AF_TABLE_MAX], dq[AF_TABLE_MAX];
    int n;
    double aoa_max, mass, g0, rho, sound;
    double tau, bias, gain;
    double v_floor, zone_lo, zone_hi;
} AfModel;

typedef struct { double arc_accel, sgs_height; } AfParams;

typedef struct {
    double h, sink, V, gamma, aoa, cmd, x;
    bool engaged;                   /* the flare has started (glide hold until then) */
    double glide_sink;              /* sink when it started: the profile never asks for more */
} AfState;

typedef struct {
    double cmd, accel, need;
    bool on_arc;
} AfLawOut;

typedef struct {
    bool landed;
    double speed, sink, along, peak_n, climb, cost;
} AfOutcome;

/* Regressors (cmd-aoa, [unused], 1):  y = (cmd-aoa)/tau + bias/tau; gain stays 1. */
static void af_response(const AdaptiveFlareState*F,double*tau,double*bias,double*gain){
    /* Prior: tau 0.8 s, unit gain, no offset.  A steady command/incidence
       gap with the incidence not moving is equally a long lag or an offset;
       physically it is trim/stiffness (the pitch loop itself is fast), so the
       lag prior is firm and moves only on transient evidence while the
       offset and gain priors are weak. */
    const double prior[3]={1.0/.8,0.0,0.0},lam[3]={5.0,3.0,.1};
    double A[3][3],b[3];
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)A[i][j]=F->sxx[i][j];
        A[i][i]+=lam[i];
        b[i]=F->sxy[i]+lam[i]*prior[i];
    }
    for(int c=0;c<3;c++){
        int piv=c;
        for(int r=c+1;r<3;r++)if(fabs(A[r][c])>fabs(A[piv][c]))piv=r;
        if(piv!=c){for(int k=0;k<3;k++){double t=A[c][k];A[c][k]=A[piv][k];A[piv][k]=t;}
            double t=b[c];b[c]=b[piv];b[piv]=t;}
        if(fabs(A[c][c])<1e-9){*tau=1.0;*bias=0.0;*gain=1.0;return;}
        for(int r=c+1;r<3;r++){double f=A[r][c]/A[c][c];for(int k=c;k<3;k++)A[r][k]-=f*A[c][k];b[r]-=f*b[c];}
    }
    double x[3];
    for(int c=2;c>=0;c--){double s=b[c];for(int k=c+1;k<3;k++)s-=A[c][k]*x[k];x[c]=s/A[c][c];}
    double inv_tau=clampd(x[0],1.0/4.0,1.0/.2);
    *tau=1.0/inv_tau;
    *gain=1.0;
    *bias=clampd(x[2]**tau,-4.0,4.0);
}

/* Identify the incidence response from the command issued last cycle. */
static void af_observe(GuidanceMachine*g,const Telemetry*t){
    AdaptiveFlareState*F=&g->adaptive_flare;
    double aoa=t->angle_of_attack,gam=t->flight_path_angle;
    if(!isfinite(t->ut)||!isfinite(aoa)||!isfinite(gam))return;
    if(F->has_last){
        double dt=t->ut-F->last_ut;
        if(dt<.02)return;
        if(dt<=1.0&&g->terminal_aoa_cmd_ut>0.0&&t->ut-g->terminal_aoa_cmd_ut<=1.5&&
           t->dynamic_pressure>1000.0&&fabs(t->roll)<45.0&&
           (!t->has_main_gear_grounded||!t->main_gear_grounded)){
            double decay=exp(-dt/5.0);
            for(int i=0;i<3;i++){F->sxy[i]*=decay;for(int j=0;j<3;j++)F->sxx[i][j]*=decay;}
            F->weight*=decay;
            double y=(aoa-F->last_aoa)/dt;
            /* Lag and offset only: over a few seconds a gain and an offset are
               nearly collinear and fight (gain at its clamps, offset -2 deg,
               a chattering inverted command).  The fast-forgetting offset
               carries the stiffness shortfall as the incidence rises. */
            double r[3]={g->terminal_aoa_cmd_last-F->last_aoa,0.0,1.0};
            if(isfinite(y)&&fabs(y)<20.0){
                for(int i=0;i<3;i++){F->sxy[i]+=dt*r[i]*y;for(int j=0;j<3;j++)F->sxx[i][j]+=dt*r[i]*r[j];}
                F->weight+=dt;
            }
        }
    }
    F->last_aoa=aoa;F->last_gamma=gam;F->last_ut=t->ut;F->has_last=true;
    af_response(F,&F->tau,&F->bias,&F->gain);
}

static void af_model(const GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v,
        double g0,double aoa_max,double v_td,double zone_lo,double zone_hi,AfModel*M){
    const AdaptiveFlareState*F=&g->adaptive_flare;
    M->aoa_max=aoa_max;
    M->n=(int)fmin(AF_TABLE_MAX,floor(aoa_max/AF_TABLE_STEP)+1);
    double ratio=g->terminal_lift_ratio>0.0&&isfinite(g->terminal_lift_ratio)?g->terminal_lift_ratio:1.0;
    for(int i=0;i<M->n;i++){
        double L=0.0,D=0.0;terminal_aero_q(g,t,v,i*AF_TABLE_STEP,&L,&D);
        M->lq[i]=L*ratio;M->dq[i]=D;
    }
    M->mass=fmax(t->mass,1.0);M->g0=g0;
    double V=fmax(1.0,t->true_air_speed);
    M->rho=isfinite(t->atmospheric_density)&&t->atmospheric_density>0.0?
        t->atmospheric_density:2.0*fmax(t->dynamic_pressure,1.0)/(V*V);
    M->sound=isfinite(t->mach)&&t->mach>.05?V/t->mach:340.0;
    M->tau=F->tau>0.0?F->tau:1.0;M->bias=F->bias;M->gain=F->gain>0.0?F->gain:1.0;
    M->v_floor=fmax(66.0,.88*v_td);
    M->zone_lo=zone_lo;M->zone_hi=zone_hi;
}

static void af_aero(const AfModel*M,double aoa,double*lq,double*dq){
    double u=clampd(aoa,0.0,(M->n-1)*AF_TABLE_STEP)/AF_TABLE_STEP;
    int i=(int)floor(u);if(i>=M->n-1)i=M->n-2;if(i<0)i=0;
    double w=u-i;
    *lq=M->lq[i]*(1.0-w)+M->lq[i+1]*w;
    *dq=M->dq[i]*(1.0-w)+M->dq[i+1]*w;
}

/* Smallest incidence giving the lift per dynamic pressure; max lift if none. */
static double af_aoa_for(const AfModel*M,double lq){
    double best=0.0,bestl=-INFINITY;
    for(int i=0;i<M->n;i++){
        if(M->lq[i]>=lq){
            if(i==0)return 0.0;
            double f=(lq-M->lq[i-1])/fmax(1e-9,M->lq[i]-M->lq[i-1]);
            return (i-1+clampd(f,0.0,1.0))*AF_TABLE_STEP;
        }
        if(M->lq[i]>bestl){bestl=M->lq[i];best=i*AF_TABLE_STEP;}
    }
    return best;
}

/* Sink versus wheel height: the constant-deceleration arc down to the
   shallow-glide sink at the shallow-glide height, then a proportional flare
   (sink falling linearly with height to the float sink at the float height).
   A short constant-deceleration final flare lasts about as long as the
   incidence lag of a slow vehicle and cannot be followed. */
static double af_profile(const AfParams*P,double g0,double h){
    (void)g0;
    h=fmax(0.0,h);
    if(h>P->sgs_height)
        return sqrt(AF_SGS_SINK*AF_SGS_SINK+2.0*P->arc_accel*(h-P->sgs_height));
    if(h<=AF_FLOAT_HEIGHT)return AF_FLOAT_SINK;
    return AF_FLOAT_SINK+(AF_SGS_SINK-AF_FLOAT_SINK)*
        (h-AF_FLOAT_HEIGHT)/fmax(.1,P->sgs_height-AF_FLOAT_HEIGHT);
}

/* The flare law.  Reads the profile one identified lag ahead so the command
   leads the incidence response.  Before the flare starts the glide is held;
   once started the profile is tracked both ways (an overshoot held as a
   shallow straight line from hundreds of metres up is a long float), with
   only a gentle push and none near the ground. */
static AfLawOut af_law(const AfModel*M,const AfParams*P,const AfState*s,double dt){
    AfLawOut o={0};
    double V=fmax(s->V,1.0),q=.5*M->rho*V*V,m=M->mass,g0=M->g0;
    double lq=0.0,dq=0.0;af_aero(M,s->aoa,&lq,&dq);
    double a_now=lq*q/m-g0*cos(s->gamma);
    double T=clampd(M->tau,.3,3.0);
    /* Only a deceleration already building is read ahead; an unloaded glide
       is not extrapolated into a steeper dive.  The law itself unloads it
       over about one lag, so it is carried at its first-order-decayed mean
       (~0.6): carrying the full pull through the lag predicts the sink gone,
       unloads, and the sink comes back at contact. */
    double a_lead=.6*fmax(0.0,a_now);
    double h_l=fmax(0.0,s->h-s->sink*T+.5*a_lead*T*T);
    double s_l=s->sink-a_lead*T;
    double ref=af_profile(P,g0,h_l);
    double ref_lo=af_profile(P,g0,fmax(0.0,h_l-.5));
    double ff=ref*(ref-ref_lo)/.5;          /* profile deceleration at the lead point */
    if(s->engaged&&isfinite(s->glide_sink)&&ref>s->glide_sink){ref=s->glide_sink;ff=0.0;}
    /* Sink-loop time constant scaled to the identified lag: a 1 s loop
       behind a 3 s incidence lag limit-cycles. */
    const double TC=clampd(T,.8,3.0);
    double a_d;
    o.on_arc=s->engaged||s->h<=P->sgs_height||(s->h<AF_MAX_ONSET&&s_l>=ref);
    a_d=o.on_arc?ff+(s_l-ref)/TC:0.0;
    /* Never a push near the ground, except out of a real float (sink under
       half the profile above 5 m): the incidence overshoots a reduced command
       and a 1-g hold there floats for seconds, bleeding the touchdown speed. */
    if(s->h<=P->sgs_height){
        bool floating=s->h>5.0&&s->sink<.5*af_profile(P,g0,s->h);
        a_d=fmax(a_d,floating?-.05*g0:0.0);
    }
    a_d=clampd(a_d,-.12*g0,1.2*g0);
    o.accel=a_d;
    double need=af_aoa_for(M,m*(a_d+g0*cos(s->gamma))/q);
    /* Invert the identified steady response (incidence = gain*cmd + bias).
       No overdrive on the lag: the lead-read profile already carries it. */
    o.need=need;
    double cmd=(need-M->bias)/M->gain;
    /* Mains-first attitude near the ground, never above the 1-g incidence. */
    if(s->h<AF_FLOOR_START){
        double u=clampd((AF_FLOOR_START-s->h)/(AF_FLOOR_START-AF_FLOOR_FULL),0.0,1.0);
        u=u*u*(3.0-2.0*u);
        double floor_aoa=fmin(AF_TD_PITCH*u-s->gamma*RAD2DEG,af_aoa_for(M,m*g0/q));
        cmd=fmax(cmd,(floor_aoa-M->bias)/M->gain);
    }
    cmd=clampd(cmd,0.0,M->aoa_max+1.0);
    if(isfinite(s->cmd)&&dt>0.0)cmd=clampd(cmd,s->cmd-AF_SLEW*dt,s->cmd+AF_SLEW*dt);
    o.cmd=cmd;
    return o;
}

/* Forward prediction from the present state to main-gear contact. */
static AfOutcome af_predict(const AfModel*M,const AfParams*P,AfState s){
    AfOutcome r={0};
    const double DT=.1;
    double peak=0.0,climb=-INFINITY;
    bool started=false;
    for(int i=0;i<450;i++){
        if(s.h<=0.0){r.landed=true;break;}
        if(s.V<.6*M->v_floor)break;
        AfLawOut o=af_law(M,P,&s,DT);
        if(o.on_arc&&!started){started=true;s.engaged=true;s.glide_sink=s.sink;}
        s.cmd=o.cmd;
        double V=s.V,q=.5*M->rho*V*V;
        s.aoa+=(M->gain*s.cmd+M->bias-s.aoa)/M->tau*DT;
        s.aoa=clampd(s.aoa,-2.0,M->aoa_max+3.0);
        double lq=0.0,dq=0.0;af_aero(M,s.aoa,&lq,&dq);
        double L=lq*q,D=dq*q;
        double gdot=(L/M->mass-M->g0*cos(s.gamma))/V;
        s.V+=(-D/M->mass-M->g0*sin(s.gamma))*DT;
        s.gamma+=gdot*DT;
        s.sink=-s.V*sin(s.gamma);
        s.h-=s.sink*DT;
        s.x+=s.V*cos(s.gamma)*DT;
        peak=fmax(peak,L/(M->mass*M->g0));
        if(started)climb=fmax(climb,-s.sink);
    }
    r.speed=s.V;r.sink=s.sink;r.along=s.x;r.peak_n=peak;r.climb=isfinite(climb)?climb:0.0;
    if(!r.landed){r.cost=1e6;return r;}
    double c=0.0;
    c+=30.0*pow(fmax(0.0,r.sink-1.5),2)+300.0*pow(fmax(0.0,r.sink-2.5),2);
    c+=200.0*pow(fmax(0.0,r.climb-.3),2);
    if(r.speed<M->v_floor)c+=pow(M->v_floor-r.speed,2);
    if(r.along<0.0)c+=2000.0+pow(r.along/20.0,2);   /* before the threshold */
    if(r.along<M->zone_lo)c+=pow((M->zone_lo-r.along)/50.0,2);
    if(r.along>M->zone_hi)c+=pow((r.along-M->zone_hi)/200.0,2);
    c+=5.0*pow(fmax(0.0,r.peak_n-1.8),2);
    r.cost=c;
    return r;
}

static const double AF_ARC[]={.06,.10,.14,.18,.24,.30};
/* Shallow-glide heights: a final segment of a few metres lasts about one
   incidence lag, and a stiff airframe trails a rising command by up to 3 deg
   there (unidentifiable within the flare): it hits at the arc sink. */
static const double AF_SGS[]={10.0,14.0,18.0};

/* Choose the profile; keeps the previous one unless a candidate is clearly
   better.  Each candidate is scored on the identified response and on a
   slower, weaker one (the identification lags a response that degrades as
   incidence rises), and the worse score counts.  Once the flare is under
   way near the ground the plan is locked: a switch there drops the
   deceleration the vehicle is already behind on. */
static void af_solve(AdaptiveFlareState*F,const AfModel*M,const AfState*s0,double ut){
    /* Engaged: the plan is fixed.  Re-choosing the arc each cycle flips it
       between extremes and the demanded deceleration with it; moving the
       shallow-glide height down mid-arc leaves the arc to stop the sink a
       few metres up.  The closed-loop law absorbs the deviations. */
    if(F->plan_valid&&s0->engaged)return;
    AfModel W=*M;
    /* The gain is not identified (it is collinear with the offset over a
       flare); score against a response 20% weaker than unit gain. */
    W.tau=fmin(3.0,1.3*M->tau);W.gain=fmax(.4,.8*M->gain);W.bias=M->bias-.3;
    double best=INFINITY;AfParams bp={0};AfOutcome bo={0};
    for(size_t i=0;i<sizeof(AF_ARC)/sizeof(AF_ARC[0]);i++)
        for(size_t j=0;j<sizeof(AF_SGS)/sizeof(AF_SGS[0]);j++){
            AfParams P={AF_ARC[i]*M->g0,AF_SGS[j]};
            AfOutcome o=af_predict(M,&P,*s0);
            AfOutcome w=af_predict(&W,&P,*s0);
            if(w.cost>o.cost)o=w;
            double c=o.cost;
            if(F->plan_valid&&fabs(P.arc_accel-F->arc_accel)<1e-6&&fabs(P.sgs_height-F->sgs_height)<1e-6)
                c=c*.9-.05;
            if(c<best){best=c;bp=P;bo=o;}
        }
    if(!isfinite(best))return;
    F->plan_valid=true;F->arc_accel=bp.arc_accel;F->sgs_height=bp.sgs_height;
    F->cost=bo.cost;F->pred_speed=bo.speed;F->pred_sink=bo.sink;F->pred_along=bo.along;
    F->pred_peak_n=bo.peak_n;F->pred_climb=bo.climb;F->plan_ut=ut;
}
