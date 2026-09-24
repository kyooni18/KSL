#ifndef CLANDING_TAEM_CANDIDATE_SEARCH_H
#define CLANDING_TAEM_CANDIDATE_SEARCH_H

#include "terminal_solver.h"

typedef struct {
    TaemPlanStatus status;
    double side;
    bool route_built;
    bool final_tail_qualified;
    double required_lateral_accel_mps2;
    double available_lateral_accel_mps2;
    double quality_score;
    TaemRoute route;
    TerminalSolverResult replay;
    const char *reason;
} TaemFixedHacCandidate;

typedef struct {
    TaemPlanStatus status;
    int selected_candidate;
    TaemFixedHacCandidate candidates[2];
    const char *reason;
} TaemFixedHacSearch;

/* Searches both runway sides using one fixed geometry contract and a native
 * full-horizon replay. It cannot mark a candidate qualified until a separately
 * verified unchanged Final-tail result is attached. */
TaemFixedHacSearch taem_fixed_hac_search(const TerminalModel *model,
        const TerminalDynamicState *initial, double route_spacing_m,
        double dt_s, double maximum_elapsed_s);

#endif
