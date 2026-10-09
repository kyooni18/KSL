#ifndef KSP_LANDER_ADAPTIVE_TRIM_H
#define KSP_LANDER_ADAPTIVE_TRIM_H
#include <stdbool.h>
#include <math.h>

/* Passive local identification of u = intercept + slope * incidence.
 * No vessel-specific prior. Time weights avoid dependence on telemetry rate.
 * Configured limits, actuator reserve and replay admission remain separate. */
typedef struct {
    double weight, x, y, xx, xy, yy, lo, hi;
    double last_ut, mach, pressure;
    bool clock_valid, regime_valid;
} AdaptiveTrim;

static inline bool adaptive_trim_fit(const AdaptiveTrim *m, double *a,
        double *b, double *noise) {
    if (!m || m->weight < 2.0 || m->hi-m->lo < 3.0) return false;
    double variance=m->xx/m->weight-pow(m->x/m->weight,2);
    if (variance < 1.0) return false;
    *b=(m->xy/m->weight-m->x*m->y/(m->weight*m->weight))/variance;
    *a=m->y/m->weight-*b*m->x/m->weight;
    double residual=m->yy/m->weight-2*(*a)*m->y/m->weight-
        2*(*b)*m->xy/m->weight+(*a)*(*a)+
        2*(*a)*(*b)*m->x/m->weight+(*b)*(*b)*m->xx/m->weight;
    *noise=sqrt(fmax(0.0,residual));
    return isfinite(*a)&&isfinite(*b)&&isfinite(*noise)&&*noise<0.1;
}

static inline void adaptive_trim_observe(AdaptiveTrim *m, double ut,
        double mach, double pressure, double incidence, double input,
        bool settled) {
    if (!m || !isfinite(ut)) return;
    double dt=m->clock_valid?ut-m->last_ut:0;
    m->last_ut=ut; m->clock_valid=true;
    if (dt<=0 || dt>2.0) { m->weight=0; m->regime_valid=false; return; }
    double decay=exp(-dt/30.0);
    m->weight*=decay; m->x*=decay; m->y*=decay;
    m->xx*=decay; m->xy*=decay; m->yy*=decay;
    if (!settled || !isfinite(mach) || !isfinite(pressure) || pressure<=0 ||
            !isfinite(incidence) || fabs(incidence)>180 || !isfinite(input) || fabs(input)>=0.95) return;
    if (m->regime_valid && (fabs(mach-m->mach)>0.35 ||
            pressure/m->pressure>2 || pressure/m->pressure<0.5)) {
        m->weight=0; m->regime_valid=false;
    }
    if (!m->regime_valid || m->weight<0.05) {
        m->weight=m->x=m->y=m->xx=m->xy=m->yy=0;
        m->lo=m->hi=incidence; m->mach=mach; m->pressure=pressure;
        m->regime_valid=true;
    }
    m->weight+=dt; m->x+=dt*incidence; m->y+=dt*input;
    m->xx+=dt*incidence*incidence; m->xy+=dt*incidence*input;
    m->yy+=dt*input*input;
    m->lo=fmin(m->lo,incidence); m->hi=fmax(m->hi,incidence);
}

static inline bool adaptive_trim_input(const AdaptiveTrim *m, double incidence,
        double *input) {
    if (!m || m->weight<0.05 || !isfinite(incidence)) return false;
    double a,b,noise;
    if (adaptive_trim_fit(m,&a,&b,&noise)) *input=a+b*incidence;
    else {
        /* An unexcited observation supports only its local incidence. */
        if (fabs(incidence-m->x/m->weight)>1.0) return false;
        *input=m->y/m->weight;
    }
    *input=fmax(-0.75,fmin(0.75,*input));
    return isfinite(*input);
}

static inline double adaptive_trim_ceiling(const AdaptiveTrim *m,
        double configured_initial, double configured_maximum) {
    double initial=fmax(0.0,fmin(configured_initial>0?configured_initial:
        configured_maximum,configured_maximum));
    double a,b,noise;
    if (!adaptive_trim_fit(m,&a,&b,&noise)) return initial;
    double span=m->hi-m->lo;
    double upper=fmin(configured_maximum,m->hi+span);
    double variance=m->xx/m->weight-pow(m->x/m->weight,2);
    double mean=m->x/m->weight;
    double ceiling=0;
    for (double incidence=0;incidence<=upper+1e-9;incidence+=0.25) {
        double uncertainty=(noise+0.03)*sqrt(1+pow(incidence-mean,2)/variance);
        if (fabs(a+b*incidence)+2*uncertainty<=0.75) ceiling=incidence;
    }
    /* Positive cap: zero is the propagator sentinel for no trim constraint. */
    return fmin(configured_maximum,fmax(0.25,ceiling));
}
#endif
