#include <math.h>
#include "shuttlesim/math3.h"

Vec3 v3(double x, double y, double z) { Vec3 v = {x,y,z}; return v; }
Vec3 v3_add(Vec3 a, Vec3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
Vec3 v3_sub(Vec3 a, Vec3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
Vec3 v3_scale(Vec3 a, double s) { return v3(a.x*s, a.y*s, a.z*s); }
double v3_dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 v3_cross(Vec3 a, Vec3 b) { return v3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x); }
double v3_norm(Vec3 a) { return sqrt(v3_dot(a,a)); }
Vec3 v3_normalized(Vec3 a) { double n=v3_norm(a); return n>1e-12?v3_scale(a,1.0/n):v3(0,0,0); }
Vec3 v3_rotate_axis(Vec3 v, Vec3 axis, double angle_rad) {
    axis = v3_normalized(axis);
    double c=cos(angle_rad), s=sin(angle_rad);
    return v3_add(v3_add(v3_scale(v,c), v3_scale(v3_cross(axis,v),s)),
                  v3_scale(axis, v3_dot(axis,v)*(1.0-c)));
}
double clampd(double x, double lo, double hi) { return x<lo?lo:(x>hi?hi:x); }
double deg2rad(double x) { return x * (3.14159265358979323846/180.0); }
double rad2deg(double x) { return x * (180.0/3.14159265358979323846); }
double wrap_pi(double x) {
    const double p=3.14159265358979323846, t=2.0*p;
    while (x>p) x-=t;
    while (x<-p) x+=t;
    return x;
}
