#include "taem_candidate_search.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
    double maximum_lead_curvature = 0.95 * reachability.available_lateral_accel_mps2 /
        fmax(geometry->airspeed_mps * geometry->airspeed_mps, 1.0);
    const double curvature_fractions[] = {0.25, 0.35, 0.50, 0.70, 1.00};
    const double bow_fractions[] = {-0.625, -0.60};
    /* Let replay test an early descent that can build density before the
     * subsonic lift cap becomes binding.  These bounded offsets remain inside
     * the route profile; the native solver still rejects any profile that
     * exceeds lift authority or reaches runway elevation before HAC exit. */
    const double initial_sag_fractions[] = {
        0.0, -0.10, -0.15, -0.20, -0.25, -0.35, -0.50, -1.0, -1.5, -2.0
    };
    const double local_profile_supports[][3] = {
        {0.12, 0.17, 0.42}, {0.12, 0.19, 0.50},
        {0.12, 0.21, 0.60}, {0.12, 0.23, 0.75}
    };
    const double local_bow_offsets_m[] = {
        -900.0, -800.0, -700.0, -500.0, -450.0, -400.0, -350.0, -300.0
    };
    TaemFixedHacCandidate best_survivor;
    memset(&best_survivor, 0, sizeof(best_survivor));
    best_survivor.quality_score = INFINITY;
    TaemFixedHacCandidate best_failure = {0};
    best_failure.status = TAEM_PLAN_INFEASIBLE;
    best_failure.reason = "fixed-HAC route geometry could not be constructed";
    double best_failure_deficit = INFINITY;
    bool have_failure = false, have_route = false;
    const char *diagnostics = getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    double vx = initial->velocity_i_mps.x;
    double vy = initial->velocity_i_mps.y;
    double vz = initial->velocity_i_mps.z;
    double kinetic = 0.5 * (vx * vx + vy * vy + vz * vz);
    for (size_t curvature = 0;
         curvature < sizeof(curvature_fractions) / sizeof(curvature_fractions[0]);
         ++curvature) {
        TaemFixedHacCandidate route_candidate = candidate;
        double curvature_limit = maximum_lead_curvature *
                                 curvature_fractions[curvature];
        if (!taem_route_build_fixed_hac(model, geometry, side, spacing,
                curvature_limit, &route_candidate.route, reason, sizeof(reason))) {
            route_candidate.reason =
                strcmp(reason,"finite lead turn exceeds available curvature authority")==0 ?
                    "finite lead exceeds live curvature authority" :
                (strcmp(reason,"lead-to-HAC distance is too short")==0 ?
                    "live state is too close to the HAC entry point" :
                 "fixed-HAC route geometry could not be constructed");
            if (!have_route && !have_failure) best_failure = route_candidate;
            continue;
        }
        route_candidate.route_built = true;
        have_route = true;
        double altitude_drop = route_candidate.route.profile_start_altitude_m -
                               route_candidate.route.profile_final_altitude_m;
        double bow_limit = fmin(6000.0, fmax(1000.0, 0.4 * altitude_drop));
        for (size_t bow = 0;
             bow < sizeof(bow_fractions) / sizeof(bow_fractions[0]); ++bow) {
            for (size_t sag = 0;
                 sag < sizeof(initial_sag_fractions) /
                       sizeof(initial_sag_fractions[0]); ++sag) {
              for (size_t support = 0;
                   support < sizeof(local_profile_supports) /
                             sizeof(local_profile_supports[0]); ++support) {
               for (size_t local = 0;
                    local < sizeof(local_bow_offsets_m) /
                            sizeof(local_bow_offsets_m[0]); ++local) {
                TaemFixedHacCandidate trial = route_candidate;
                trial.route.profile_midpoint_offset_m =
                    bow_limit * bow_fractions[bow];
                double initial_sag_limit=fmin(600.0, fmax(150.0,
                    geometry->ground_speed_mps));
                trial.route.profile_initial_sag_m=initial_sag_limit *
                    initial_sag_fractions[sag];
                trial.route.profile_initial_sag_length_m=
                    fmax(4000.0,geometry->ground_speed_mps * 20.0);
                trial.route.profile_local_offset_m = local_bow_offsets_m[local];
                trial.route.profile_local_start_fraction =
                    local_profile_supports[support][0];
                trial.route.profile_local_peak_fraction =
                    local_profile_supports[support][1];
                trial.route.profile_local_end_fraction =
                    local_profile_supports[support][2];
                trial.replay = terminal_solver_replay(model, initial,
                    &trial.route, dt, maximum_elapsed);
                if (diagnostics && strcmp(diagnostics, "1") == 0)
                    fprintf(stderr,
                        "TAEM candidate: side=%+.0f curvature=%.2f bow=%+.0f local=%+.0f@%.2f^%.2f-%.2f sag=%+.0f/%.0f lead=%zu length=%.0f status=%d elapsed=%.2f index=%zu reason=%s verticalError=%.2f lateralShort=%.2f vDemand=%.2f vDelivered=%.2f altErr=%.1f currentFPA=%.2f targetAlt=%.1f targetFPA=%.2f\n",
                        side, curvature_fractions[curvature],
                        trial.route.profile_midpoint_offset_m,
                        trial.route.profile_local_offset_m,
                        trial.route.profile_local_start_fraction,
                        trial.route.profile_local_peak_fraction,
                        trial.route.profile_local_end_fraction,
                        trial.route.profile_initial_sag_m,
                        trial.route.profile_initial_sag_length_m,
                        trial.route.lead_count, trial.route.length_m,
                        (int)trial.replay.status, trial.replay.elapsed_s,
                        trial.replay.failure_route_index,
                        trial.replay.reason ? trial.replay.reason : "none",
                        trial.replay.maximum_vertical_authority_shortfall_mps2,
                        trial.replay.maximum_lateral_authority_shortfall_mps2,
                        trial.replay.failure_required_vertical_lift_mps2,
                        trial.replay.failure_delivered_vertical_lift_mps2,
                        trial.replay.maximum_altitude_error_m,
                        trial.replay.final_geometry.flight_path_angle_deg,
                        trial.replay.failure_target_altitude_m,
                        trial.replay.failure_target_flight_path_angle_deg);
                if (trial.replay.status == TERMINAL_SOLVER_UNQUALIFIED &&
                    trial.replay.path_constraints_ok) {
                    trial.status = TAEM_PLAN_UNQUALIFIED;
                    trial.reason = "native HAC route reaches its exit; Final tail qualification is pending";
                    double cross_limit = fmax(1200.0, 0.1 * trial.route.hac.radius_m);
                    trial.quality_score = trial.replay.maximum_cross_track_m / cross_limit +
                        trial.replay.maximum_course_error_deg / 20.0 +
                        trial.replay.maximum_altitude_error_m / 2000.0 +
                        fabs(trial.replay.energy_closure_residual_j_kg) /
                            fmax(250.0, 0.02 * fmax(fabs(trial.replay.drag_work_j_kg),
                                                   kinetic));
                    if (!isfinite(trial.quality_score)) trial.quality_score = INFINITY;
                    if (trial.quality_score < best_survivor.quality_score)
                        best_survivor = trial;
                    continue;
                }
                trial.status = trial.replay.status == TERMINAL_SOLVER_SEARCH_EXHAUSTED ?
                    TAEM_PLAN_SEARCH_EXHAUSTED : TAEM_PLAN_INFEASIBLE;
                trial.reason = trial.replay.reason;
                /* If all profiles fail, retain the attempt that progressed furthest;
                 * break ties by the measured tracker authority deficit for diagnostics. */
                double deficit = trial.replay.maximum_lateral_authority_shortfall_mps2 +
                                 trial.replay.maximum_vertical_authority_shortfall_mps2;
                if (!have_failure || trial.replay.elapsed_s > best_failure.replay.elapsed_s ||
                    (trial.replay.elapsed_s == best_failure.replay.elapsed_s &&
                     deficit < best_failure_deficit)) {
                    best_failure = trial;
                    best_failure_deficit = deficit;
                    have_failure = true;
               }
              }
              }
            }
        }
    }
    if (!have_route) return best_failure;
    return best_survivor.status == TAEM_PLAN_UNQUALIFIED ?
        best_survivor : best_failure;
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
