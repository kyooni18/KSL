#ifndef CLANDING_TAEM_PLANNER_H
#define CLANDING_TAEM_PLANNER_H

#include <stdbool.h>
#include <stddef.h>

#include "taem_reachability.h"
#include "taem_route.h"
#include "taem_tracker.h"

typedef enum {
    TAEM_PLAN_INFEASIBLE = 0,
    TAEM_PLAN_UNQUALIFIED,
    TAEM_PLAN_SEARCH_EXHAUSTED,
    TAEM_PLAN_QUALIFIED
} TaemPlanStatus;

typedef struct {
    TaemPlanStatus status;
    bool route_available;
    bool turn_authority_ok;
    bool energy_qualified;
    double selected_side;
    double geometry_margin_m;
    double lateral_margin_mps2;
    const char *reason;
} TaemPlanResult;

/* Builds an analytic HAC-circle route from the live state.  Radius and Final
 * exit stay fixed while sweep_abs_rad selects the capture point on the circle. */
bool taem_route_build_hac(const TerminalModel *model,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double point_spacing_m,
        double maximum_lead_curvature_per_m, TaemRoute *route,
        char *reason, size_t reason_size);

/* As taem_route_build_hac, but selects the shortest finite lead whose peak
 * curvature stays within peak_curvature_limit_per_m. */
bool taem_route_build_hac_shortest(const TerminalModel *model,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double point_spacing_m,
        double maximum_lead_curvature_per_m, double peak_curvature_limit_per_m,
        TaemRoute *route, char *reason, size_t reason_size);

/* As taem_route_build_hac_shortest, but selects the lead whose control
 * polygon (an upper bound on its length) is closest to target_lead_length_m:
 * a longer lead dissipates energy a high state cannot lose on a short route. */
bool taem_route_build_hac_length(const TerminalModel *model,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double sweep_abs_rad, double point_spacing_m,
        double maximum_lead_curvature_per_m, double peak_curvature_limit_per_m,
        double target_lead_length_m, TaemRoute *route,
        char *reason, size_t reason_size);

/* Interpolates between two already feasible routes for the same HAC family.
 * Used only after profile propagation has bracketed a too-short/too-long lead. */
bool taem_route_blend_hac_length(const TerminalModel *model,
        const TaemGeometryState *start, const TaemRoute *route_a,
        const TaemRoute *route_b, double point_spacing_m,
        double maximum_lead_curvature_per_m, double peak_curvature_limit_per_m,
        double target_lead_length_m, TaemRoute *route,
        char *reason, size_t reason_size);

/* Peak curvature of the finite C1 lead; INFINITY for an invalid route. */
double taem_route_lead_peak_curvature(const TaemRoute *route);

/* Compatibility/qualification wrapper for the historical full 270-degree HAC. */
bool taem_route_build_fixed_hac(const TerminalModel *model,
        const TaemGeometryState *start, double hac_radius_m, double side,
        double point_spacing_m, double maximum_lead_curvature_per_m,
        TaemRoute *route, char *reason, size_t reason_size);

/* Shapes the analytic vertical profile so its initial FPA curvature does
 * not demand more vertical lift than the measured/identified vehicle can
 * provide while also satisfying the route's initial lateral curvature. */
bool taem_route_limit_initial_vertical_authority(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry,
        TaemRoute *route);

/* Finds the closest point at or after cursor. The cursor is monotone so a
 * self-near 270-degree circle cannot make the tracker jump to another branch. */
/* Planned (zero-speedbrake) airspeed at a route station, or NAN when the
 * route carries no speed plan. */
double taem_route_planned_speed(const TaemRoute *route, double station_m);
double taem_route_planned_altitude(const TaemRoute *route, double station_m);
double taem_route_curvature_at_station(const TaemRoute *route, double station_m);

bool taem_route_point_at_index(const TaemRoute *route, size_t index,
        TaemRoutePoint *point);

bool taem_route_reference(const TaemRoute *route, const TaemGeometryState *state,
        size_t *cursor, TaemPathReference *reference, size_t *point_index);

#endif
