#include "taem_candidate_search.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "shuttlesim/world.h"

/* Join stations on the fixed analytic HAC circle, expressed as the remaining
 * sweep to the Final exit.  The step is fine enough that a state already part
 * way around the circle finds a join with compatible chord/tangent geometry.
 * The search is hierarchical: cheap geometry/authority pruning and energy
 * ranking of every station, then full native replay of only a few survivors,
 * then vertical-profile refinement of the selected route only. */
#define TAEM_JOIN_SWEEP_MAX_DEG 270.0
#define TAEM_JOIN_SWEEP_MIN_DEG 15.0
#define TAEM_HAC_RADIUS_STEPS 5
#define TAEM_HAC_RADIUS_RATIO 0.6
#define TAEM_JOIN_SWEEP_STEP_DEG 7.5
#define TAEM_JOIN_MAX_STATIONS 120
#define TAEM_JOIN_MAX_REPLAYS 3
#define TAEM_JOIN_MAX_PROFILES 8
#define TAEM_PROFILE_DT_S 0.5
#define TAEM_MIN_HAC_RADIUS_M 3000.0

typedef struct {
    double sweep_rad;
    double score;
    TaemRoute route;
} JoinStation;

static int compare_station(const void *a, const void *b) {
    double x = ((const JoinStation *)a)->score;
    double y = ((const JoinStation *)b)->score;
    return (x > y) - (x < y);
}

static bool diagnostics_enabled(void) {
    const char *diagnostics = getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    return diagnostics && strcmp(diagnostics, "1") == 0;
}

static bool replay_reaches_exit(const TerminalSolverResult *replay) {
    return replay->status == TERMINAL_SOLVER_UNQUALIFIED &&
           replay->path_constraints_ok;
}

static double exit_speed_target(const TerminalModel *model) {
    return isfinite(model->guidance.final_alignment_speed) &&
        model->guidance.final_alignment_speed > 0.0 ?
        model->guidance.final_alignment_speed : model->vehicle.final_approach_speed;
}

/* Tightest HAC worth planning: a steady coordinated turn at the Final
 * alignment speed and the vehicle bank limit. */
static double hac_radius_floor(const TerminalModel *model) {
    double bank = fmin(model->vehicle.maximum_bank_angle, 80.0) *
                  3.14159265358979323846 / 180.0;
    double speed = exit_speed_target(model);
    double radius = model->world.radius_m + model->site.altitude;
    double gravity = model->world.mu_m3_s2 / (radius * radius);
    return bank > 0.0 && speed > 0.0 && isfinite(gravity) && gravity > 0.0 ?
        fmax(TAEM_MIN_HAC_RADIUS_M, speed * speed / (gravity * tan(bank))) : INFINITY;
}

static double initial_kinetic(const TerminalDynamicState *initial) {
    double vx = initial->velocity_i_mps.x;
    double vy = initial->velocity_i_mps.y;
    double vz = initial->velocity_i_mps.z;
    return 0.5 * (vx * vx + vy * vy + vz * vz);
}

static bool better_failure(const TaemFixedHacCandidate *trial,
                           const TaemFixedHacCandidate *best, bool have_best) {
    if (!have_best) return true;
    if (trial->route_built != best->route_built) return trial->route_built;
    return trial->replay.elapsed_s > best->replay.elapsed_s;
}

static bool candidate_reaches_target_speed(const TerminalModel *model,
        const TaemFixedHacCandidate *candidate) {
    return candidate->status == TAEM_PLAN_UNQUALIFIED &&
        candidate->replay.final_geometry.airspeed_mps >= exit_speed_target(model);
}

static bool candidate_preferred(const TerminalModel *model,
        const TaemFixedHacCandidate *trial, const TaemFixedHacCandidate *best) {
    if (trial->status != TAEM_PLAN_UNQUALIFIED) return false;
    if (best->status != TAEM_PLAN_UNQUALIFIED) return true;
    double target = exit_speed_target(model);
    double trial_deficit = fmax(0.0, target - trial->replay.final_geometry.airspeed_mps);
    double best_deficit = fmax(0.0, target - best->replay.final_geometry.airspeed_mps);
    return trial_deficit < best_deficit ||
        (trial_deficit == best_deficit && trial->quality_score < best->quality_score);
}
/* Replay qualified the route through the HAC exit.  Rank by tracking quality,
 * energy closure and how far the exit airspeed falls short of the configured
 * Final alignment speed. */
static void finish_candidate(const TerminalModel *model,
        const TerminalDynamicState *initial, TaemFixedHacCandidate *trial) {
    trial->status = TAEM_PLAN_UNQUALIFIED;
    trial->reason = "native HAC route reaches its exit; Final tail qualification is pending";
    double target_speed = exit_speed_target(model);
    double deficit = fmax(0.0, target_speed -
        trial->replay.final_geometry.airspeed_mps) / target_speed;
    double cross_limit = fmax(1200.0, 0.1 * trial->route.hac.radius_m);
    trial->quality_score = trial->replay.maximum_cross_track_m / cross_limit +
        trial->replay.maximum_course_error_deg / 20.0 +
        trial->replay.maximum_altitude_error_m / 2000.0 +
        fabs(trial->replay.energy_closure_residual_j_kg) /
            fmax(250.0, 0.02 * fmax(fabs(trial->replay.drag_work_j_kg),
                                    initial_kinetic(initial))) +
        4.0 * deficit;
    if (!isfinite(trial->quality_score)) trial->quality_score = INFINITY;
}

static void mark_failure(TaemFixedHacCandidate *trial) {
    trial->status = trial->replay.status == TERMINAL_SOLVER_SEARCH_EXHAUSTED ?
        TAEM_PLAN_SEARCH_EXHAUSTED : TAEM_PLAN_INFEASIBLE;
    trial->reason = trial->replay.reason;
}

static TaemFixedHacCandidate evaluate_side(const TerminalModel *model,
        const TerminalDynamicState *initial, const TaemGeometryState *geometry,
        double hac_radius, double side, double spacing, double dt,
        double maximum_elapsed, JoinStation *stations) {
    TaemFixedHacCandidate candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.side = side;
    candidate.status = TAEM_PLAN_INFEASIBLE;
    candidate.reason = "fixed-HAC candidate is infeasible";

    TaemReachability reachability;
    memset(&reachability, 0, sizeof(reachability));
    if (!taem_fixed_hac_turn_reachability(model, initial, geometry,
            hac_radius, &reachability) ||
        !reachability.valid ||
        !(reachability.available_lateral_accel_mps2 > 0.0)) {
        candidate.required_lateral_accel_mps2 = reachability.required_lateral_accel_mps2;
        candidate.available_lateral_accel_mps2 = reachability.available_lateral_accel_mps2;
        candidate.reason = "no measurable lateral authority for the finite lead";
        return candidate;
    }
    candidate.required_lateral_accel_mps2 = reachability.required_lateral_accel_mps2;
    candidate.available_lateral_accel_mps2 = reachability.available_lateral_accel_mps2;

    /* The live curvature authority bounds the roll-settling prefix of the lead
     * (enforced inside the planner).  Later lead curvature is judged by native
     * replay at the propagated state.  For cheap pruning only, bound it
     * optimistically: bank-limited curvature is 0.5*rho*S*CL*sin(bank)/m and
     * does not depend on speed, so descending to runway density is the most
     * it can grow.  A lead exceeding that ceiling cannot be flown.  The whole
     * lead is not ranked against the live thin-air authority again; that
     * systematically discarded joins that become executable as density builds. */
    double live_curvature = 0.95 * reachability.available_lateral_accel_mps2 /
        fmax(geometry->ground_speed_mps * geometry->ground_speed_mps, 1.0);
    double rho_live = world_atmosphere_sample(&model->world,
        model->site.altitude + geometry->altitude_above_runway_m).density_kg_m3;
    double rho_runway = world_atmosphere_sample(&model->world,
        model->site.altitude).density_kg_m3;
    double density_gain = rho_live > 0.0 && rho_runway > rho_live ?
        rho_runway / rho_live : 1.0;
    double curvature_ceiling = live_curvature * density_gain;

    TaemFixedHacCandidate best_failure = candidate;
    bool have_failure = false;
    size_t count = 0;
    char reason[160];
    for (double sweep_deg = TAEM_JOIN_SWEEP_MAX_DEG;
         sweep_deg >= TAEM_JOIN_SWEEP_MIN_DEG - 1e-9;
         sweep_deg -= TAEM_JOIN_SWEEP_STEP_DEG) {
        /* Two lead shapes per join: the smoothest lead and the shortest lead
         * within the optimistic authority ceiling.  The smoothest lead can be
         * a long loop when the vehicle is close to the circle; the compact
         * one is what an energy-limited state may need. */
        /* A third, energy-matched lead is sized so the whole route meets the
         * TAEM glide-slope length: a state too high for the direct lead
         * dissipates the excess along a longer lead rather than in a steep
         * descent near the runway. */
        double sweep_rad = sweep_deg * 3.14159265358979323846 / 180.0;
        double energy_lead_length = NAN;
        for (int variant = 0; variant < 3 && count < TAEM_JOIN_MAX_STATIONS; ++variant) {
            JoinStation *station = &stations[count];
            station->sweep_rad = sweep_rad;
            if (variant == 2 && !isfinite(energy_lead_length)) continue;
            bool built = variant == 0 ?
                taem_route_build_hac(model, geometry, hac_radius, side,
                    station->sweep_rad, spacing, live_curvature,
                    &station->route, reason, sizeof(reason)) :
                variant == 1 ?
                taem_route_build_hac_shortest(model, geometry, hac_radius, side,
                    station->sweep_rad, spacing, live_curvature, curvature_ceiling,
                    &station->route, reason, sizeof(reason)) :
                taem_route_build_hac_length(model, geometry, hac_radius, side,
                    station->sweep_rad, spacing, live_curvature, curvature_ceiling,
                    energy_lead_length, &station->route, reason, sizeof(reason));
            if (!built) {
                if (!have_failure) {
                    best_failure.reason =
                        strcmp(reason, "lead-to-HAC distance is too short") == 0 ?
                            "live state is too close to every HAC join" :
                            "finite lead exceeds live curvature authority at every HAC join";
                    have_failure = true;
                }
                continue;
            }
            if (variant > 0 && count > 0 &&
                fabs(stations[count - 1].sweep_rad - station->sweep_rad) < 1e-9 &&
                fabs(stations[count - 1].route.length_m - station->route.length_m) < 200.0)
                continue; /* identical to the previous lead */
            double peak = taem_route_lead_peak_curvature(&station->route);
            double hac_curvature = 1.0 / station->route.hac.radius_m;
            if (!isfinite(peak) || peak > curvature_ceiling) continue;
            /* Energy: an unpowered route whose altitude drop spread over its
             * length is much shallower than the configured TAEM glide slope
             * cannot be held, and one much steeper dives through the energy
             * Final needs.  Prefer lengths near the glide-slope length. */
            double drop = station->route.profile_start_altitude_m -
                          station->route.profile_final_altitude_m;
            double glide_length = drop > 0.0 ?
                drop / tan(model->guidance.taem_glide_slope * 3.14159265358979323846 / 180.0) :
                0.0;
            if (variant == 0 && glide_length > station->route.length_m + 1000.0)
                energy_lead_length = glide_length -
                    (station->route.length_m - station->route.lead_length_m);
            station->score =
                0.25 * fmax(0.0, peak - hac_curvature) / hac_curvature +
                fabs(station->route.length_m - glide_length) /
                    fmax(glide_length, station->route.hac.radius_m);
            ++count;
        }
    }
    if (count == 0) return best_failure;
    qsort(stations, count, sizeof(stations[0]), compare_station);

    /* Generate a dynamically feasible vertical reference for the most
     * promising lateral routes by native propagation.  Routes whose energy
     * cannot be brought to the HAC exit altitude by any incidence are dropped
     * here without a full replay; survivors are ordered by how well their exit
     * airspeed meets the Final alignment speed. */
    double target_speed = exit_speed_target(model);
    size_t profiled = 0;
    size_t profile_budget = count < TAEM_JOIN_MAX_PROFILES ? count : TAEM_JOIN_MAX_PROFILES;
    for (size_t i = 0; i < profile_budget; ++i) {
        TerminalProfileResult profile = terminal_solver_generate_profile(model,
            initial, &stations[i].route, TAEM_PROFILE_DT_S, maximum_elapsed);
        if (diagnostics_enabled())
            fprintf(stderr,
                "TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n",
                side, stations[i].route.hac.radius_m,
                stations[i].sweep_rad * 180.0 / 3.14159265358979323846,
                stations[i].route.length_m, profile.valid ? 1 : 0,
                profile.aoa_deg, profile.exit_speed_mps, profile.exit_fpa_deg,
                profile.reason);
        if (!profile.valid) continue;
        stations[i].score = stations[i].score +
            fmax(0.0, target_speed - profile.exit_speed_mps) / target_speed * 10.0;
        if (i != profiled) {
            JoinStation swap = stations[profiled];
            stations[profiled] = stations[i];
            stations[i] = swap;
        }
        ++profiled;
    }
    if (profiled == 0) {
        best_failure.reason = "no HAC join has an energy-feasible native vertical profile";
        return best_failure;
    }
    qsort(stations, profiled, sizeof(stations[0]), compare_station);
    count = profiled;

    have_failure = false;
    size_t replays = count < TAEM_JOIN_MAX_REPLAYS ? count : TAEM_JOIN_MAX_REPLAYS;
    for (size_t i = 0; i < replays; ++i) {
        TaemFixedHacCandidate trial = candidate;
        trial.route = stations[i].route;
        trial.route_built = true;
        trial.replay = terminal_solver_replay(model, initial, &trial.route,
                                              dt, maximum_elapsed);
        if (diagnostics_enabled())
            fprintf(stderr,
                "TAEM candidate: side=%+.0f join_sweep=%.1f lead=%.0f arc=%.0f leadPeakK=%.3e liveK=%.3e status=%d elapsed=%.2f index=%zu/%zu reason=%s cross=%.1f course=%.1f altErr=%.1f exitV=%.1f latShort=%.2f\n",
                side, stations[i].sweep_rad * 180.0 / 3.14159265358979323846,
                trial.route.lead_length_m, trial.route.hac.arc_length_m,
                taem_route_lead_peak_curvature(&trial.route), live_curvature,
                (int)trial.replay.status, trial.replay.elapsed_s,
                trial.replay.failure_route_index, trial.route.count,
                trial.replay.reason ? trial.replay.reason : "none",
                trial.replay.maximum_cross_track_m,
                trial.replay.maximum_course_error_deg,
                trial.replay.maximum_altitude_error_m,
                trial.replay.final_geometry.airspeed_mps,
                trial.replay.maximum_lateral_authority_shortfall_mps2);
        if (replay_reaches_exit(&trial.replay)) {
            finish_candidate(model, initial, &trial);
            if (candidate_reaches_target_speed(model, &trial)) return trial;
            if (candidate_preferred(model, &trial, &best_failure)) {
                best_failure = trial;
                have_failure = true;
            }
            continue;
        }
        mark_failure(&trial);
        if (best_failure.status != TAEM_PLAN_UNQUALIFIED &&
            better_failure(&trial, &best_failure, have_failure)) {
            best_failure = trial;
            have_failure = true;
        }
    }
    return best_failure;
}


static bool search_inputs_valid(const TerminalModel *model,
        const TerminalDynamicState *initial, double hac_radius, double spacing,
        double dt, double maximum_elapsed) {
    return model && model->replay_validated &&
        terminal_model_validate(model, NULL, 0) &&
        terminal_state_validate(initial, NULL, 0) && hac_radius >= TAEM_MIN_HAC_RADIUS_M &&
        isfinite(hac_radius) && spacing >= 100.0 && isfinite(spacing) &&
        dt > 0.0 && isfinite(dt) && maximum_elapsed > 0.0 &&
        isfinite(maximum_elapsed);
}

TaemFixedHacCandidate taem_fixed_hac_evaluate_exact(
        const TerminalModel *model,const TerminalDynamicState *initial,
        double hac_radius,double side,double spacing,double dt,
        double maximum_elapsed) {
    TaemFixedHacCandidate candidate;
    memset(&candidate,0,sizeof(candidate));
    candidate.status=TAEM_PLAN_INFEASIBLE;
    candidate.side=side<0.0?-1.0:1.0;
    candidate.reason="invalid model, state, or frozen-HAC contract";
    if(!search_inputs_valid(model,initial,hac_radius,spacing,dt,maximum_elapsed)||
       !isfinite(side)||fabs(side)<0.5)
        return candidate;

    TaemGeometryState geometry;
    if(!taem_geometry_state(model,initial,&geometry)){
        candidate.reason="initial state cannot be projected into the runway frame";
        return candidate;
    }
    JoinStation *stations=calloc(TAEM_JOIN_MAX_STATIONS,sizeof(*stations));
    if(!stations){
        candidate.reason="candidate search could not allocate join stations";
        return candidate;
    }
    candidate=evaluate_side(model,initial,&geometry,hac_radius,
        side<0.0?-1.0:1.0,spacing,dt,maximum_elapsed,stations);
    free(stations);
    return candidate;
}


TaemFixedHacSearch taem_fixed_hac_search_runway_ends(const TerminalModel *model,
        const TerminalModel *reciprocal_model, const TerminalDynamicState *initial,
        double hac_radius, double spacing, double dt, double maximum_elapsed) {
    TaemFixedHacSearch result;
    memset(&result, 0, sizeof(result));
    result.status = TAEM_PLAN_INFEASIBLE;
    result.selected_candidate = -1;
    result.reason = "every fixed-HAC candidate is infeasible";
    if (!search_inputs_valid(model, initial, hac_radius, spacing, dt,
                             maximum_elapsed) ||
        (reciprocal_model && !search_inputs_valid(reciprocal_model, initial,
            hac_radius, spacing, dt, maximum_elapsed))) {
        result.reason = "invalid model, state, or candidate-search bound";
        return result;
    }
    JoinStation *stations = calloc(TAEM_JOIN_MAX_STATIONS, sizeof(*stations));
    if (!stations) {
        result.reason = "candidate search could not allocate join stations";
        return result;
    }
    const TerminalModel *ends[2] = {model, reciprocal_model};
    int end_count = reciprocal_model ? 2 : 1;
    for (int end = 0; end < end_count; ++end) {
        TaemGeometryState geometry;
        bool projected = taem_geometry_state(ends[end], initial, &geometry);
        for (int side_index = 0; side_index < 2; ++side_index) {
            TaemFixedHacCandidate *slot = &result.candidates[end * 2 + side_index];
            if (!projected) {
                memset(slot, 0, sizeof(*slot));
                slot->status = TAEM_PLAN_INFEASIBLE;
                slot->side = side_index == 0 ? -1.0 : 1.0;
                slot->reason = "initial state cannot be projected into the runway frame";
            } else {
                /* The analytic circle stays the primitive; its radius is a
                 * bounded search variable.  The handed-over radius is tried
                 * first.  Only when no route around it is feasible are
                 * tighter circles tried, down to the steady turn radius at
                 * the Final alignment speed and the vehicle bank limit: a
                 * state with little energy near the runway can still fly a
                 * short HAC where a large circle is beyond its glide range. */
                double side = side_index == 0 ? -1.0 : 1.0;
                double floor_radius = hac_radius_floor(ends[end]);
                double radius = hac_radius;
                *slot = evaluate_side(ends[end], initial, &geometry, radius,
                    side, spacing, dt, maximum_elapsed, stations);
                for (int step = 1; step < TAEM_HAC_RADIUS_STEPS &&
                        !candidate_reaches_target_speed(ends[end], slot); ++step) {
                    double next_radius = fmax(floor_radius, radius * TAEM_HAC_RADIUS_RATIO);
                    if (!(next_radius < radius)) break;
                    radius = next_radius;
                    TaemFixedHacCandidate tighter = evaluate_side(ends[end],
                        initial, &geometry, radius, side, spacing, dt,
                        maximum_elapsed, stations);
                    if (candidate_preferred(ends[end], &tighter, slot) ||
                        (slot->status != TAEM_PLAN_UNQUALIFIED &&
                         tighter.route_built && !slot->route_built))
                        *slot = tighter;
                }
            }
            slot->runway_end = end;
        }
    }
    free(stations);
    result.candidate_count = end_count * 2;

    int best = -1;
    for (int i = 0; i < result.candidate_count; ++i) {
        if (result.candidates[i].status == TAEM_PLAN_UNQUALIFIED &&
            (best < 0 || candidate_preferred(ends[result.candidates[i].runway_end],
                &result.candidates[i], &result.candidates[best])))
            best = i;
    }
    if (best >= 0) {
        result.status = TAEM_PLAN_UNQUALIFIED;
        result.selected_candidate = best;
        result.reason = result.candidates[best].reason;
        return result;
    }
    for (int i = 0; i < result.candidate_count; ++i)
        if (result.candidates[i].status == TAEM_PLAN_SEARCH_EXHAUSTED) {
            result.status = TAEM_PLAN_SEARCH_EXHAUSTED;
            result.reason = "candidate search exhausted before a route reached the HAC exit";
        }
    return result;
}

TaemFixedHacSearch taem_fixed_hac_search(const TerminalModel *model,
        const TerminalDynamicState *initial, double hac_radius, double spacing,
        double dt, double maximum_elapsed) {
    return taem_fixed_hac_search_runway_ends(model, NULL, initial, hac_radius,
        spacing, dt, maximum_elapsed);
}
