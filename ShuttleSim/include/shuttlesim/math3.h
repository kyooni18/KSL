#ifndef SHUTTLESIM_MATH3_H
#define SHUTTLESIM_MATH3_H
#include "types.h"
Vec3 ss_v3(double x, double y, double z);
Vec3 v3_add(Vec3 a, Vec3 b);
Vec3 v3_sub(Vec3 a, Vec3 b);
Vec3 v3_scale(Vec3 a, double s);
double v3_dot(Vec3 a, Vec3 b);
Vec3 v3_cross(Vec3 a, Vec3 b);
double v3_norm(Vec3 a);
Vec3 v3_normalized(Vec3 a);
Vec3 v3_rotate_axis(Vec3 v, Vec3 axis, double angle_rad);
double ss_clampd(double x, double lo, double hi);
double deg2rad(double x);
double rad2deg(double x);
double wrap_pi(double x);
#endif
