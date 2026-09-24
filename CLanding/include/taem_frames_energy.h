#ifndef TAEM_FRAMES_ENERGY_H
#define TAEM_FRAMES_ENERGY_H

#include <stdbool.h>

/* SI units; angles are radians. These types intentionally do not alias the
 * ShuttleSim Vec3 type so this diagnostic leaf has no plant/build dependency. */
typedef struct { double x, y, z; } TaemVec3;

typedef struct {
    double radius_m;
    double mu_m3_s2;
    TaemVec3 rotation_axis_i; /* unit vector; ShuttleSim currently uses +Z */
    double rotation_rate_rad_s;
    double rotation_phase_rad_at_ut0;
} TaemFrameWorld;

typedef struct {
    double latitude_rad;
    double longitude_rad;
    double elevation_m;
    double heading_rad; /* clockwise from north, in the selected direction */
    double length_m;
    double width_m;
} TaemRunway;

typedef struct {
    TaemVec3 threshold_unit_b;
    TaemVec3 forward_b;
    TaemVec3 right_b;
    double elevation_m;
    double length_m;
    double width_m;
} TaemRunwayFrame;

typedef struct {
    TaemVec3 position_b_m;
    double radius_from_center_m;
    double altitude_above_datum_m;
    double altitude_above_runway_m;
    double runway_along_m;
    double runway_cross_m;
    double runway_vertical_m;
} TaemRunwayCoordinates;

/* Specific energy is J/kg and rate is W/kg. Effective energy uses the rotating
 * planet frame: 1/2 |v_b|^2 - mu/r - 1/2 |Omega x r|^2. Drag work rate is
 * signed negative: -D*|v_air|/m. */
typedef struct {
    double gravitational_potential_j_kg;
    double inertial_specific_energy_j_kg;
    double effective_specific_energy_j_kg;
    double inertial_energy_rate_w_kg;
    double effective_energy_rate_w_kg;
    double airspeed_mps;
    double drag_work_rate_w_kg;
} TaemEnergyDiagnostic;

typedef struct {
    double gravity_mps2;
    double surface_speed_mps;
    double centrifugal_accel_mps2;
} TaemGravityDiagnostic;

TaemVec3 taem_vec3_add(TaemVec3 a, TaemVec3 b);
TaemVec3 taem_vec3_sub(TaemVec3 a, TaemVec3 b);
TaemVec3 taem_vec3_scale(TaemVec3 a, double scale);
double taem_vec3_dot(TaemVec3 a, TaemVec3 b);
TaemVec3 taem_vec3_cross(TaemVec3 a, TaemVec3 b);
double taem_vec3_norm(TaemVec3 a);

bool taem_inertial_to_fixed(const TaemFrameWorld *world, TaemVec3 position_i,
                            TaemVec3 velocity_i, double ut,
                            TaemVec3 *position_b, TaemVec3 *velocity_b);
bool taem_fixed_to_inertial(const TaemFrameWorld *world, TaemVec3 position_b,
                            TaemVec3 velocity_b, double ut,
                            TaemVec3 *position_i, TaemVec3 *velocity_i);
bool taem_local_north_east_up(TaemVec3 position_b, TaemVec3 *north,
                              TaemVec3 *east, TaemVec3 *up);
bool taem_runway_frame_create(const TaemFrameWorld *world,
                              const TaemRunway *runway,
                              TaemRunwayFrame *frame);
/* Reciprocal threshold is the far endpoint along the great-circle runway. */
bool taem_runway_frame_reciprocal(const TaemFrameWorld *world,
                                  const TaemRunwayFrame *selected,
                                  TaemRunwayFrame *reciprocal);
bool taem_runway_project(const TaemFrameWorld *world,
                         const TaemRunwayFrame *frame, TaemVec3 position_b,
                         TaemRunwayCoordinates *coordinates);
bool taem_runway_unproject(const TaemFrameWorld *world,
                           const TaemRunwayFrame *frame, double along_m,
                           double cross_m, double altitude_above_runway_m,
                           TaemVec3 *position_b);
double taem_wrap_pi(double angle_rad);
bool taem_gravity_diagnostic(const TaemFrameWorld *world,
                             TaemVec3 position_b,
                             TaemGravityDiagnostic *diagnostic);
bool taem_energy_diagnostic(const TaemFrameWorld *world, TaemVec3 position_i,
                            TaemVec3 velocity_i, TaemVec3 velocity_b,
                            TaemVec3 air_velocity_b, TaemVec3 aerodynamic_force_i,
                            TaemVec3 aerodynamic_force_b, double mass_kg,
                            double drag_n, TaemEnergyDiagnostic *diagnostic);
bool taem_no_wind_energy_work_residual(const TaemEnergyDiagnostic *diagnostic,
                                      double *residual_w_kg);
/* Converts the spherical ground-arc energy gradient to its time rate; the
 * radius and flight-path factors cancel analytically. */
bool taem_ground_arc_work_rate(double drag_n, double mass_kg,
                               double airspeed_mps, double flight_path_angle_rad,
                               double radius_from_center_m, double reference_radius_m,
                               double *work_rate_w_kg);

#endif
