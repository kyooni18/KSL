#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "shuttlesim/aero.h"
#include "shuttlesim/math3.h"

static size_t lower_index(const double *a,size_t n,double x){
    if(n<2) return 0;
    if(x<=a[0]) return 0;
    for(size_t i=1;i<n;i++) if(x<=a[i]) return i-1;
    return n-2;
}

void aero_seed_stsn(AeroTable *a) {
    memset(a,0,sizeof(*a));
    const double m[]={0,0.8,1.2,2,5,10,15,25};
    const double al[]={-5,0,5,10,15,20,25,30,35,40,45};
    a->mach_count=sizeof(m)/sizeof(m[0]); a->alpha_count=sizeof(al)/sizeof(al[0]);
    memcpy(a->mach,m,sizeof(m)); memcpy(a->alpha_deg,al,sizeof(al));
    a->reference_area_m2=250.0;
    for(size_t i=0;i<a->mach_count;i++) for(size_t j=0;j<a->alpha_count;j++){
        double ar=deg2rad(al[j]);
        double hyp=(m[i]>=5.0)?1.0:(0.82+0.036*m[i]);
        double cl=hyp*(1.15*sin(2.0*ar));
        double cd0=(m[i]<0.8?0.16:(m[i]<1.2?0.28:(m[i]<3?0.34:0.42)));
        double cd=cd0 + hyp*1.55*sin(ar)*sin(ar);
        if(al[j]<0) cl*=0.8;
        a->cl[i][j]=cl; a->cd[i][j]=cd;
    }
}

bool aero_load_csv(AeroTable *a,const char *path){
    FILE *f=fopen(path,"r"); if(!f) return false;
    typedef struct{double m,al,cl,cd;} Row; Row rows[1600]; size_t nr=0;
    double ms[AERO_MACH_MAX], as[AERO_ALPHA_MAX]; size_t nm=0,na=0; char line[512];
    while(fgets(line,sizeof(line),f)&&nr<1600){
        if(line[0]=='#'||strstr(line,"mach")) continue;
        Row r; if(sscanf(line,"%lf,%lf,%lf,%lf",&r.m,&r.al,&r.cl,&r.cd)==4){
            rows[nr++]=r;
            bool found=false; for(size_t i=0;i<nm;i++) if(fabs(ms[i]-r.m)<1e-9) found=true;
            if(!found&&nm<AERO_MACH_MAX) ms[nm++]=r.m;
            found=false; for(size_t i=0;i<na;i++) if(fabs(as[i]-r.al)<1e-9) found=true;
            if(!found&&na<AERO_ALPHA_MAX) as[na++]=r.al;
        }
    }
    fclose(f); if(nm<2||na<2) return false;
    for(size_t i=0;i<nm;i++)for(size_t j=i+1;j<nm;j++)if(ms[j]<ms[i]){double t=ms[i];ms[i]=ms[j];ms[j]=t;}
    for(size_t i=0;i<na;i++)for(size_t j=i+1;j<na;j++)if(as[j]<as[i]){double t=as[i];as[i]=as[j];as[j]=t;}
    memset(a,0,sizeof(*a)); a->mach_count=nm;a->alpha_count=na;a->reference_area_m2=250.0;
    memcpy(a->mach,ms,nm*sizeof(double));memcpy(a->alpha_deg,as,na*sizeof(double));
    bool seen[AERO_MACH_MAX][AERO_ALPHA_MAX]={{false}};
    for(size_t r=0;r<nr;r++){
        size_t im=0,ia=0; for(size_t i=0;i<nm;i++)if(fabs(ms[i]-rows[r].m)<1e-9)im=i;
        for(size_t j=0;j<na;j++)if(fabs(as[j]-rows[r].al)<1e-9)ia=j;
        a->cl[im][ia]=rows[r].cl;a->cd[im][ia]=rows[r].cd;seen[im][ia]=true;
    }
    for(size_t i=0;i<nm;i++)for(size_t j=0;j<na;j++)if(!seen[i][j])return false;
    return true;
}

bool aero_load_book_csv(AeroTable *a,const char *path){
    FILE *f=fopen(path,"r"); if(!f)return false;
    char line[512]; size_t n=0; bool overflow=false;
    while(fgets(line,sizeof(line),f)){
        if(line[0]=='#'||strstr(line,"q_pa"))continue;
        AeroBookPoint p={0};
        int got=sscanf(line,"%lf,%lf,%lf,%lf,%lf,%lf",
                       &p.q_pa,&p.mach,&p.alpha_deg,&p.lift_per_q_m2,&p.drag_per_q_m2,&p.support);
        if(got>=5&&p.q_pa>0&&isfinite(p.q_pa)&&isfinite(p.mach)&&
           isfinite(p.alpha_deg)&&isfinite(p.lift_per_q_m2)&&isfinite(p.drag_per_q_m2)){
            if(got<6)p.support=1.0;
            if(n<AERO_BOOK_MAX)a->book[n++]=p; else overflow=true;
        }
    }
    fclose(f);
    if(overflow){a->book_count=0;a->book_enabled=false;return false;}
    a->book_count=n; a->book_enabled=n>0; return n>0;
}

static bool aero_book_lookup(const AeroTable *a,double q,double mach,double alpha,
        double *lift_per_q,double *drag_per_q,double *coverage_out){
    if(coverage_out)*coverage_out=0.0;
    if(!a->book_enabled||a->book_count==0||q<1.0)return false;
    const double log2v=0.69314718055994530942;
    double best_d2=INFINITY,second_d2=INFINITY;
    const AeroBookPoint *best=NULL,*second=NULL;
    for(size_t i=0;i<a->book_count;i++){
        const AeroBookPoint *p=&a->book[i];
        /* State-space distance for local force-book interpolation.  These are
           interpolation hyperparameters, not physical constants; production
           changes must be selected by cross-validation among training flights. */
        double dq=(log(q/p->q_pa)/log2v)/1.0;
        double dm=(mach-p->mach)/0.5;
        double da=(alpha-p->alpha_deg)/3.0;
        double d2=dq*dq+dm*dm+da*da;
        if(d2<best_d2){
            second_d2=best_d2;second=best;best_d2=d2;best=p;
        }else if(d2<second_d2){
            second_d2=d2;second=p;
        }
    }
    if(!best)return false;
    if(!second){second=best;second_d2=best_d2;}
    double w0=pow(fmax(1.0,best->support),0.25)/(0.02+best_d2);
    double w1=pow(fmax(1.0,second->support),0.25)/(0.02+second_d2);
    double sw=w0+w1;
    if(!(sw>0.0)||!isfinite(sw))return false;
    *lift_per_q=(w0*best->lift_per_q_m2+w1*second->lift_per_q_m2)/sw;
    *drag_per_q=(w0*best->drag_per_q_m2+w1*second->drag_per_q_m2)/sw;

    /* The direct KSP book owns only its calibrated neighborhood.  Fade to the
       smooth polar outside that neighborhood rather than extrapolating sparse
       measurements indefinitely.  Do not tune this coverage gate on held-out
       parity flights. */
    double nearest=sqrt(best_d2);
    double coverage=clampd((1.5-nearest)/0.75,0.0,1.0);
    if(coverage_out)*coverage_out=coverage;
    return true;
}

void aero_coefficients(const AeroTable *a,double mach,double alpha,double *cl,double *cd){
    size_t i=lower_index(a->mach,a->mach_count,mach), j=lower_index(a->alpha_deg,a->alpha_count,alpha);
    double m0=a->mach[i],m1=a->mach[i+1],a0=a->alpha_deg[j],a1=a->alpha_deg[j+1];
    double tm=clampd((mach-m0)/(m1-m0),0,1), ta=clampd((alpha-a0)/(a1-a0),0,1);
    double cl0=a->cl[i][j]*(1-ta)+a->cl[i][j+1]*ta, cl1=a->cl[i+1][j]*(1-ta)+a->cl[i+1][j+1]*ta;
    double cd0=a->cd[i][j]*(1-ta)+a->cd[i][j+1]*ta, cd1=a->cd[i+1][j]*(1-ta)+a->cd[i+1][j+1]*ta;
    *cl=cl0*(1-tm)+cl1*tm; *cd=cd0*(1-tm)+cd1*tm;
}
AeroForces aero_compute(const KerbinWorld *w,const AeroTable *a,Vec3 p,Vec3 v,double ut,double mass,double aoa,double bank){
    (void)mass; AeroForces out; memset(&out,0,sizeof(out));
    AtmosphereSample atm=world_atmosphere_sample_state(w,p,ut);
    Vec3 vair=v3_sub(v,world_atmosphere_velocity_i(w,p)); double speed=v3_norm(vair);
    out.airspeed_mps=speed; if(speed<1e-6||atm.density_kg_m3<=0)return out;
    out.mach=(atm.speed_of_sound_mps>1)?speed/atm.speed_of_sound_mps:0;
    out.dynamic_pressure_pa=0.5*atm.density_kg_m3*speed*speed;
    double cl,cd; aero_coefficients(a,out.mach,rad2deg(aoa),&cl,&cd);
    double fallback_lift_per_q=a->reference_area_m2*cl;
    double fallback_drag_per_q=a->reference_area_m2*cd;
    double lift_per_q=0,drag_per_q=0,book_coverage=0;
    if(aero_book_lookup(a,out.dynamic_pressure_pa,out.mach,rad2deg(aoa),
            &lift_per_q,&drag_per_q,&book_coverage)){
        lift_per_q=fallback_lift_per_q+book_coverage*(lift_per_q-fallback_lift_per_q);
        drag_per_q=fallback_drag_per_q+book_coverage*(drag_per_q-fallback_drag_per_q);
    }else{
        lift_per_q=fallback_lift_per_q;
        drag_per_q=fallback_drag_per_q;
    }
    out.lift_n=out.dynamic_pressure_pa*lift_per_q;
    out.drag_n=out.dynamic_pressure_pa*drag_per_q;
    Vec3 fwd=v3_normalized(vair), up=v3_normalized(p);
    Vec3 right=v3_normalized(v3_cross(fwd,up));
    if(v3_norm(right)<1e-8) right=v3(0,1,0);
    Vec3 lift0=v3_normalized(v3_cross(right,fwd));
    if(v3_dot(lift0,up)<0) lift0=v3_scale(lift0,-1);
    Vec3 lift=v3_rotate_axis(lift0,fwd,bank);
    Vec3 drag=v3_scale(fwd,-out.drag_n);
    out.force_i=v3_add(drag,v3_scale(lift,out.lift_n));
    return out;
}
