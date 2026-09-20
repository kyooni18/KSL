#ifndef SHUTTLESIM_QUAT_H
#define SHUTTLESIM_QUAT_H
#include "types.h"
Quat quat_identity(void);
Quat quat_normalized(Quat q);
Quat quat_mul(Quat a, Quat b);
Quat quat_axis_angle(Vec3 axis, double angle_rad);
Vec3 quat_rotate(Quat q, Vec3 v);
Quat quat_from_basis(Vec3 forward, Vec3 right, Vec3 up);
#endif
