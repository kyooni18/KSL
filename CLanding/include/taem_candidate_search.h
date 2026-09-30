#ifndef CLANDING_TAEM_CANDIDATE_SEARCH_H
#define CLANDING_TAEM_CANDIDATE_SEARCH_H

#include "terminal_solver.h"

typedef struct {
    TaemPlanStatus status;
    double side;
    int runway_end; /* 0 = configured end, 1 = reciprocal end */
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
    int candidate_count;
    /* Index = runway_end * 2 + (side > 0 ? 1 : 0). */
    TaemFixedHacCandidate candidates[4];
    const char *reason;
} TaemFixedHacSearch;

/* Compare two replay-qualified MM305 candidates with the same energy-first
 * ordering used internally by the HAC search. */
bool taem_fixed_hac_candidate_preferred(const TerminalModel *model,
        const TaemFixedHacCandidate *trial, const TaemFixedHacCandidate *best);

/* Searches both runway sides using one fixed geometry contract and a native
 * full-horizon replay. It cannot mark a candidate qualified until a separately
 * verified unchanged Final-tail result is attached. */
TaemFixedHacSearch taem_fixed_hac_search(const TerminalModel *model,
        const TerminalDynamicState *initial, double hac_radius_m,
        double route_spacing_m, double dt_s, double maximum_elapsed_s);

/* Evaluates one already-frozen analytic HAC: runway end, side and radius
 * are inputs, not search variables. Join location/lead and vertical profile
 * remain dynamically solved and replay-qualified. */
TaemFixedHacCandidate taem_fixed_hac_evaluate_exact(const TerminalModel *model,
        const TerminalDynamicState *initial, double hac_radius_m, double side,
        double route_spacing_m, double dt_s, double maximum_elapsed_s);

/* Qualify one already-built route against the same native profile/replay
 * contracts used by the search.  This lets MM305 test the provisional route it
 * is already tracking before searching unrelated geometry. */
TaemFixedHacCandidate taem_fixed_hac_evaluate_route(const TerminalModel *model,
        const TerminalDynamicState *initial, const TaemRoute *route,
        int runway_end, double dt_s, double maximum_elapsed_s);

/* Same fixed-HAC contract evaluated from both runway ends.  reciprocal_model
 * is the configured model with its site replaced by the reciprocal threshold
 * (NULL searches the configured end only).  Each candidate's route is framed
 * in its own runway end's coordinates. */
TaemFixedHacSearch taem_fixed_hac_search_runway_ends(const TerminalModel *model,
        const TerminalModel *reciprocal_model, const TerminalDynamicState *initial,
        double hac_radius_m, double route_spacing_m, double dt_s,
        double maximum_elapsed_s);

#endif
