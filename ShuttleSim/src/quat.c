#include <math.h>
#include "shuttlesim/quat.h"
#include "shuttlesim/math3.h"

Quat quat_identity(void) { Quat q={1,0,0,0}; return q; }
Quat quat_normalized(Quat q) {
    double n=sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if(n<1e-12) return quat_identity();
    Quat r={q.w/n,q.x/n,q.y/n,q.z/n}; return r;
}
Quat quat_mul(Quat a, Quat b) {
    Quat q={
        a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
    };
    return q;
}
Quat quat_axis_angle(Vec3 axis, double angle_rad) {
    axis=v3_normalized(axis); double h=0.5*angle_rad, s=sin(h);
    Quat q={cos(h),axis.x*s,axis.y*s,axis.z*s}; return quat_normalized(q);
}
Vec3 quat_rotate(Quat q, Vec3 v) {
    q=quat_normalized(q); Vec3 u=v3(q.x,q.y,q.z);
    Vec3 t=v3_scale(v3_cross(u,v),2.0);
    return v3_add(v, v3_add(v3_scale(t,q.w), v3_cross(u,t)));
}
Quat quat_from_basis(Vec3 forward, Vec3 right, Vec3 up) {
    /* Rotation matrix columns are body axes: forward X, right Y, up Z. */
    double m00=forward.x,m01=right.x,m02=up.x;
    double m10=forward.y,m11=right.y,m12=up.y;
    double m20=forward.z,m21=right.z,m22=up.z;
    double tr=m00+m11+m22; Quat q;
    if(tr>0){
        double s=sqrt(tr+1.0)*2.0; q.w=0.25*s; q.x=(m21-m12)/s; q.y=(m02-m20)/s; q.z=(m10-m01)/s;
    }else if(m00>m11 && m00>m22){
        double s=sqrt(1.0+m00-m11-m22)*2.0; q.w=(m21-m12)/s; q.x=0.25*s; q.y=(m01+m10)/s; q.z=(m02+m20)/s;
    }else if(m11>m22){
        double s=sqrt(1.0+m11-m00-m22)*2.0; q.w=(m02-m20)/s; q.x=(m01+m10)/s; q.y=0.25*s; q.z=(m12+m21)/s;
    }else{
        double s=sqrt(1.0+m22-m00-m11)*2.0; q.w=(m10-m01)/s; q.x=(m02+m20)/s; q.y=(m12+m21)/s; q.z=0.25*s;
    }
    return quat_normalized(q);
}
