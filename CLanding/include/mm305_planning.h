#ifndef KSP_LANDER_MM305_PLANNING_H
#define KSP_LANDER_MM305_PLANNING_H

#include "landing.h"
#include "taem_candidate_search.h"

/*
 * MM305 route planning, separated from the per-tick tracker so it can run on
 * the prediction worker.  A request is an immutable snapshot: the vehicle
 * state, the frozen terminal model, and the measured/model aerodynamic scale
 * factors that correct the planning model toward the live vehicle.
 */
typedef struct {
    bool valid;
    double request_ut;
    uint64_t model_snapshot_id;
    TerminalDynamicState state;
    double hac_radius_m;
    bool search_both_ends;
    int upstream_end;          /* runway end to search when not searching both */
    bool restrict_side;        /* replan: keep the committed side */
    double side;
    double lift_scale, drag_scale;
} Mm305PlanRequest;

typedef struct {
    bool valid;                /* planner ran */
    bool found;                /* a qualified candidate exists */
    double request_ut;
    uint64_t model_snapshot_id;
    TaemFixedHacCandidate candidate;
    double solve_wall_s;
    char diagnostic[768];
} Mm305PlanResult;

/* Build a request from the live guidance state (NULL-safe; returns invalid
   when MM305 does not currently need a plan). */
Mm305PlanRequest guidance_mm305_plan_request(const GuidanceMachine *g,
    const Telemetry *t, const VehicleState *state,
    const LandingConfiguration *cfg, const TerminalModel *model);

/* Pure, re-entrant: may run on any thread. */
Mm305PlanResult mm305_plan(const TerminalModel *model,
    const Mm305PlanRequest *request);

/* Adopt a finished plan into guidance (control thread, under the controller
   lock).  Returns true when a new route was committed. */
bool guidance_mm305_accept_plan(GuidanceMachine *g, const Mm305PlanResult *result,
    const LandingConfiguration *cfg);

#endif
