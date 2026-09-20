#ifndef SHUTTLESIM_TYPES_H
#define SHUTTLESIM_TYPES_H

#include <stdbool.h>
#include <stddef.h>

typedef struct { double x, y, z; } Vec3;
typedef struct { double w, x, y, z; } Quat;

typedef struct {
    double lat_rad;
    double lon_rad;
    double altitude_m;
} LLA;

typedef struct {
    Vec3 north;
    Vec3 east;
    Vec3 up;
} LocalFrame;

typedef struct {
    double density_kg_m3;
    double pressure_pa;
    double temperature_k;
    double speed_of_sound_mps;
} AtmosphereSample;

typedef struct {
    double lift_n;
    double drag_n;
    double side_n;
    Vec3 force_i;
    double mach;
    double dynamic_pressure_pa;
    double airspeed_mps;
} AeroForces;

#endif
