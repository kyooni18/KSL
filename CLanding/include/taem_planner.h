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

/* Builds an analytic fixed 270-degree HAC route from the live state. This
 * creates geometry only; it never qualifies energy or grants command ownership. */
bool taem_route_build_fixed_hac(const TerminalModel *model,
        const TaemGeometryState *start, double side, double point_spacing_m,
        double maximum_lead_curvature_per_m,
        TaemRoute *route, char *reason, size_t reason_size);

/* Finds the closest point at or after cursor. The cursor is monotone so a
 * self-near 270-degree circle cannot make the tracker jump to another branch. */
bool taem_route_reference(const TaemRoute *route, const TaemGeometryState *state,
        size_t *cursor, TaemPathReference *reference, size_t *point_index);

#endif
