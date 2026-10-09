#ifndef THRUST_ALIGNMENT_H
#define THRUST_ALIGNMENT_H
#include "landing.h"
/* Body coordinates here are kRPC vessel axes: right, forward, down.
   Rotate the measured resultant onto forward, then map that attitude into
   the desired inertial acceleration/up frame. */
static inline Vector3 thrust_alignment_rotate(Vector3 v,Vector3 from){
    Vector3 a=vnorm(from,v3(0,1,0)),b=v3(0,1,0),k=vcross(a,b);
    double c=clampd(vdot(a,b),-1,1);
    if(c<-1+1e-10)return v3(v.x,-v.y,-v.z);
    return vadd(v,vadd(vcross(k,v),vscale(vcross(k,vcross(k,v)),1/(1+c))));
}
static inline Vector3 thrust_alignment_map(Vector3 body,Vector3 direction,Vector3 up){
    Vector3 d=vnorm(direction,v3(0,1,0));
    Vector3 u=vnorm(vproject_plane(up,d),v3(0,0,1));
    Vector3 right=vnorm(vcross(d,u),v3(1,0,0));
    return vadd(vscale(right,body.x),vadd(vscale(d,body.y),vscale(u,-body.z)));
}
static inline void thrust_alignment_attitude(Vector3 axis,Vector3 direction,Vector3 up,
        Vector3 *nose,Vector3 *roof){
    *nose=thrust_alignment_map(thrust_alignment_rotate(v3(0,1,0),axis),direction,up);
    *roof=thrust_alignment_map(thrust_alignment_rotate(v3(0,0,-1),axis),direction,up);
}
#endif
