#ifndef SHUTTLESIM_SCENARIO_H
#define SHUTTLESIM_SCENARIO_H
#include "types.h"
#include <stdbool.h>

typedef struct {
    char name[96];
    double ut0;
    bool has_cartesian_state;
    Vec3 position_i_m;
    Vec3 velocity_i_mps;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double heading_deg;
    double mass_kg;
    double initial_aoa_deg;
    double initial_bank_deg;
    double deorbit_delta_v_mps;
    double deorbit_duration_s;
    double deorbit_delay_s;
    double orbital_engine_available_thrust_n;
    bool runway_override;
    double runway_latitude_deg;
    double runway_longitude_deg;
    double runway_elevation_m;
    double runway_heading_deg;
    double runway_length_m;
    double runway_width_m;
} Scenario;

void scenario_seed_86km(Scenario *s);
bool scenario_load(const char *path, Scenario *s);
#endif
