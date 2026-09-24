#include "taem_candidate_search.h"

#include <math.h>
#include <string.h>

static TaemFixedHacCandidate evaluate_side(const TerminalModel *model,
        const TerminalDynamicState *initial, const TaemGeometryState *geometry,
        double side, double spacing, double dt, double maximum_elapsed) {
    TaemFixedHacCandidate candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.side = side;
    candidate.status = TAEM_PLAN_INFEASIBLE;
    candidate.reason = "fixed-HAC candidate is infeasible";
    char reason[160];
    TaemReachability reachability;
    memset(&reachability, 0, sizeof(reachability));
    if (!taem_fixed_hac_turn_reachability(model, initial, geometry,
            model->guidance.hac_radius, &reachability) ||
        !reachability.valid ||
        !(reachability.available_lateral_accel_mps2 > 0.0)) {
        candidate.required_lateral_accel_mps2 = reachability.required_lateral_accel_mps2;
        candidate.available_lateral_accel_mps2 = reachability.available_lateral_accel_mps2;
        candidate.reason = "no measurable lateral authority for the finite lead";
        return candidate;
    }
    candidate.required_lateral_accel_mps2 = reachability.required_lateral_accel_mps2;
    candidate.available_lateral_accel_mps2 = reachability.available_lateral_accel_mps2;
    /* Current-state lift bounds the finite lead.  HAC circle authority is
     * checked during native replay at the evolving capture speed; rejecting
     * from the initial speed here would discard leads that can slow the
     * vehicle before the fixed arc.  Keep a small numerical margin, but do not reserve nearly a third of
     * the available turn authority here.  The solver propagates the complete
     * route with the live force and response limits, so an over-conservative
     * geometric prefilter can otherwise discard a candidate that replay
     * would qualify. */
    double max_lead_curvature = 0.95 * reachability.available_lateral_accel_mps2 /
        fmax(geometry->airspeed_mps * geometry->airspeed_mps, 1.0);
    if (!taem_route_build_fixed_hac(model, geometry, side, spacing,
                                    max_lead_curvature,
                                    &candidate.route, reason, sizeof(reason))) {
        candidate.reason = strcmp(reason,"finite lead turn exceeds available curvature authority")==0 ?
            "finite lead exceeds live curvature authority" :
            (strcmp(reason,"lead-to-HAC distance is too short")==0 ?
                "live state is too close to the HAC entry point" :
             "fixed-HAC route geometry could not be constructed");
        return candidate;
    }
    candidate.route_built = true;
    candidate.replay = terminal_solver_replay(model, initial, &candidate.route,
                                               dt, maximum_elapsed);
    if (candidate.replay.status == TERMINAL_SOLVER_UNQUALIFIED &&
        candidate.replay.path_constraints_ok) {
        candidate.status = TAEM_PLAN_UNQUALIFIED;
        candidate.reason = "native HAC route reaches its exit; Final tail qualification is pending";
        double cross_limit = fmax(1200.0, 0.1 * candidate.route.hac.radius_m);
        double vx = initial->velocity_i_mps.x;
        double vy = initial->velocity_i_mps.y;
        double vz = initial->velocity_i_mps.z;
        double kinetic = 0.5 * (vx * vx + vy * vy + vz * vz);
        candidate.quality_score = candidate.replay.maximum_cross_track_m / cross_limit +
            candidate.replay.maximum_course_error_deg / 20.0 +
            fabs(candidate.replay.energy_closure_residual_j_kg) /
                fmax(250.0, 0.02 * fmax(fabs(candidate.replay.drag_work_j_kg),
                                       kinetic));
        if (!isfinite(candidate.quality_score)) candidate.quality_score = INFINITY;
    } else if (candidate.replay.status == TERMINAL_SOLVER_SEARCH_EXHAUSTED) {
        candidate.status = TAEM_PLAN_SEARCH_EXHAUSTED;
        candidate.reason = candidate.replay.reason;
    } else {
        candidate.status = TAEM_PLAN_INFEASIBLE;
        candidate.reason = candidate.replay.reason;
    }
    return candidate;
}

TaemFixedHacSearch taem_fixed_hac_search(const TerminalModel *model,
        const TerminalDynamicState *initial, double spacing, double dt,
        double maximum_elapsed) {
    TaemFixedHacSearch result;
    memset(&result, 0, sizeof(result));
    result.status = TAEM_PLAN_INFEASIBLE;
    result.selected_candidate = -1;
    result.reason = "both fixed-HAC candidates are infeasible";
    if (!model || !model->replay_validated ||
        !terminal_model_validate(model, NULL, 0) ||
        !terminal_state_validate(initial, NULL, 0) || !(spacing >= 100.0) ||
        !(dt > 0.0) || !(maximum_elapsed > 0.0) || !isfinite(spacing) ||
        !isfinite(dt) || !isfinite(maximum_elapsed)) {
        result.reason = "invalid model, state, or candidate-search bound";
        return result;
    }
    TaemGeometryState geometry;
    if (!taem_geometry_state(model, initial, &geometry)) {
        result.reason = "initial state cannot be projected into the runway frame";
        return result;
    }
    result.candidates[0] = evaluate_side(model, initial, &geometry, -1.0,
                                         spacing, dt, maximum_elapsed);
    result.candidates[1] = evaluate_side(model, initial, &geometry, 1.0,
                                         spacing, dt, maximum_elapsed);
    int best = -1;
    for (int i = 0; i < 2; ++i) {
        if (result.candidates[i].status == TAEM_PLAN_UNQUALIFIED &&
            (best < 0 || result.candidates[i].quality_score <
                         result.candidates[best].quality_score)) best = i;
    }
    if (best >= 0) {
        result.status = TAEM_PLAN_UNQUALIFIED;
        result.selected_candidate = best;
        result.reason = result.candidates[best].reason;
        return result;
    }
    if (result.candidates[0].status == TAEM_PLAN_SEARCH_EXHAUSTED ||
        result.candidates[1].status == TAEM_PLAN_SEARCH_EXHAUSTED) {
        result.status = TAEM_PLAN_SEARCH_EXHAUSTED;
        result.reason = "candidate search exhausted before a route reached the HAC exit";
    }
    return result;
}
