#ifndef CLANDING_TAEM_REACHABILITY_H
#define CLANDING_TAEM_REACHABILITY_H

#include <stdbool.h>

#include "taem_geometry.h"
#include "taem_route.h"

typedef struct {
    bool valid;
    bool lateral_authority_ok;
    bool energy_qualified;
    double required_lateral_accel_mps2;
    double available_lateral_accel_mps2;
    double lateral_margin_mps2;
    double minimum_turn_radius_m;
    double roll_settling_time_s;
    double roll_settling_distance_m;
} TaemReachability;

bool taem_hac_geometry_sweep(const TerminalModel *model, double radius_m,
        double side, double sweep_abs_rad, TaemFixedHacGeometry *geometry);
bool taem_fixed_hac_geometry(const TerminalModel *model, double radius_m,
        double side, TaemFixedHacGeometry *geometry);
bool taem_fixed_hac_turn_reachability(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry,
        double radius_m, TaemReachability *reachability);

#endif
