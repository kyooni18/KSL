#ifndef SHUTTLESIM_WORLD_H
#define SHUTTLESIM_WORLD_H
#include "types.h"

#define ATM_TABLE_MAX 2048

typedef struct {
    double altitude_m;
    AtmosphereSample sample;
} AtmospherePoint;

typedef struct {
    AtmospherePoint p[ATM_TABLE_MAX];
    size_t count;
} AtmosphereTable;

typedef struct {
    double radius_m;
    double mu_m3_s2;
    double rotation_rate_rad_s;
    double atmosphere_top_m;
    double rotation_phase_rad_at_ut0;
    double orbital_rate_rad_s;
    double solar_phase_rad_at_ut0;
    bool stock_spatial_atmosphere;
    AtmosphereTable atmosphere;
} KerbinWorld;

typedef struct {
    double lat_rad;
    double lon_rad;
    double elevation_m;
    double heading_rad;
    double length_m;
    double width_m;
} Runway;

void world_seed_kerbin(KerbinWorld *w);
bool world_load_atmosphere_csv(KerbinWorld *w, const char *path);
Vec3 world_gravity_accel(const KerbinWorld *w, Vec3 position_i);
Vec3 world_fixed_to_inertial(const KerbinWorld *w, Vec3 fixed, double ut);
Vec3 world_inertial_to_fixed(const KerbinWorld *w, Vec3 inertial, double ut);
Vec3 world_atmosphere_velocity_i(const KerbinWorld *w, Vec3 position_i);
LLA world_lla(const KerbinWorld *w, Vec3 position_i, double ut);
Vec3 world_lla_to_inertial(const KerbinWorld *w, double lat_rad, double lon_rad, double altitude_m, double ut);
LocalFrame world_local_frame_i(const KerbinWorld *w, Vec3 position_i, double ut);
AtmosphereSample world_atmosphere_sample(const KerbinWorld *w, double altitude_m);
AtmosphereSample world_atmosphere_sample_state(const KerbinWorld *w, Vec3 position_i, double ut);
void runway_seed_ksp09(Runway *r);
bool runway_contains(const Runway *r, double along_m, double cross_m);
void ss_runway_coordinates(const KerbinWorld *w, const Runway *r, Vec3 position_i, double ut,
                         double *along_m, double *cross_m, double *vertical_m);
#endif
