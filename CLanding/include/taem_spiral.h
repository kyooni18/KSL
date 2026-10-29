#ifndef CLANDING_TAEM_SPIRAL_H
#define CLANDING_TAEM_SPIRAL_H

#include <math.h>
#include <stdbool.h>

/* Remaining angle decreases in flight. Axes: forward, right. A quartic
 * contraction has r'=r''=r'''=0 at exit, so curvature and its station
 * derivative join a smooth bank unload without a curvature impulse.
 * The coefficient is (entry_radius-exit_radius)/sweep^4, never universal. */
typedef struct {
    double x, y, heading_rad, radius_m, ds_dpsi;
    double curvature, curvature_rate, curvature_acceleration;
} TaemSpiralSample;

static inline bool taem_spiral_sample(double rf, double a, double psi,
        double side, double exit_heading, double cx, double cy,
        TaemSpiralSample *out) {
    if (!out || !(rf > 0.0) || !(a >= 0.0) || !(psi >= 0.0) ||
        !isfinite(rf+a+psi+exit_heading+cx+cy) || fabs(side) != 1.0)
        return false;
    double r=rf+a*psi*psi*psi*psi, rp=4*a*psi*psi*psi;
    double rpp=12*a*psi*psi, rppp=24*a*psi, rpppp=24*a;
    double theta=exit_heading-side*(acos(-1.0)/2+psi);
    double c=cos(theta), s=sin(theta), q=r*r+rp*rp, root=sqrt(q);
    double n=r*r+2*rp*rp-r*rpp;
    double np=2*r*rp+3*rp*rpp-r*rppp;
    double npp=2*rp*rp+2*r*rpp+3*rpp*rpp+2*rp*rppp-r*rpppp;
    double b=r*rp+rp*rpp, bp=rp*rp+r*rpp+rpp*rpp+rp*rppp;
    double kp=side*(np-3*n*b/q)/(q*root);
    double kpp=side*(npp-(6*np*b+3*n*bp)/q+15*n*b*b/(q*q))/(q*root);
    *out=(TaemSpiralSample){
        .x=cx+r*c,.y=cy+r*s,
        .heading_rad=atan2(-rp*s+side*r*c,-rp*c-side*r*s),
        .radius_m=r,.ds_dpsi=root,.curvature=side*n/(q*root),
        .curvature_rate=-kp/root,
        .curvature_acceleration=kpp/q-kp*b/(q*q)
    };
    return isfinite(out->curvature_acceleration) && isfinite(out->x);
}

/* Composite 8-point Gauss integration of the actual metric. Angular panels
 * bound quadrature error on multi-revolution curves; no r*dpsi shortcut. */
static inline double taem_spiral_arc(double rf, double a, double psi) {
    if (!(rf > 0) || !(a >= 0) || !(psi >= 0) || !isfinite(rf+a+psi)) return NAN;
    static const double x[4]={.1834346424956498,.5255324099163290,
        .7966664774136267,.9602898564975363};
    static const double w[4]={.3626837833783620,.3137066458778873,
        .2223810344533745,.1012285362903763};
    int panels=(int)ceil(psi/.25);
    if (panels < 1) return 0;
    double sum=0, width=psi/panels;
    for (int j=0;j<panels;++j) for (int i=0;i<4;++i) for (int sign=-1;sign<=1;sign+=2) {
        double p=width*(j+.5+.5*sign*x[i]);
        double r=rf+a*p*p*p*p, rp=4*a*p*p*p;
        sum+=.5*width*w[i]*hypot(r,rp);
    }
    return sum;
}
#endif
