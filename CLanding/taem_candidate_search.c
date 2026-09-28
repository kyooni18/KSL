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
 * ranking of every station, then full native replay of only a few survivors. */
#define TAEM_JOIN_SWEEP_MAX_DEG 270.0
#define TAEM_JOIN_SWEEP_MIN_DEG 15.0
#define TAEM_HAC_RADIUS_STEPS 7
#define TAEM_JOIN_SWEEP_STEP_DEG 30.0
#define TAEM_JOIN_MAX_STATIONS 48
#define TAEM_JOIN_MAX_REPLAYS 2
#define TAEM_JOIN_MAX_PROFILES 6
#define TAEM_PROFILE_DT_S 0.5
#define TAEM_MIN_HAC_RADIUS_M 3000.0

typedef struct {
    double sweep_rad;
    double score;
    TaemRoute route;
} JoinStation;

static int compare_station_length(const void *a, const void *b) {
    double la = ((const JoinStation *)a)->route.length_m;
    double lb = ((const JoinStation *)b)->route.length_m;
    return (la > lb) - (la < lb);
}

static int compare_station(const void *a, const void *b) {
    double x = ((const JoinStation *)a)->score;
    double y = ((const JoinStation *)b)->score;
    return (x > y) - (x < y);
}

static bool diagnostics_enabled(void) {
    const char *diagnostics = getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    return diagnostics &&
        (strcmp(diagnostics, "1") == 0 || strcmp(diagnostics, "2") == 0);
}
static double hermite_peak_descent_deg(const TaemRoute *route,
        double length_m) {
    if (!route || !(length_m > 0.0)) return INFINITY;
    const double d2r=3.14159265358979323846/180.0;
    double h0=route->profile_start_altitude_m;
    double h1=route->profile_final_altitude_m;
    double m0=tan(route->profile_start_fpa_deg*d2r);
    double m1=-tan(route->profile_final_slope_deg*d2r);
    double peak=0.0;
    for(int i=0;i<=96;++i) {
        double x=(double)i/96.0, x2=x*x;
        double dx=(6.0*x2-6.0*x)*h0+
            (3.0*x2-4.0*x+1.0)*length_m*m0+
            (-6.0*x2+6.0*x)*h1+
            (3.0*x2-2.0*x)*length_m*m1;
        double descent=-atan(dx/length_m)/d2r;
        peak=fmax(peak,descent);
    }
    return peak;
}

static double minimum_vertical_profile_length(const TerminalModel *model,
        const TaemRoute *route) {
    if (!model || !route) return INFINITY;
    const double d2r=3.14159265358979323846/180.0;
    double final_limit=route->profile_final_slope_deg+3.0;
    double limit=isfinite(model->guidance.taem_glide_slope) &&
        model->guidance.taem_glide_slope>0.0 ?
        fmax(final_limit,model->guidance.taem_glide_slope) : final_limit;
    double drop=route->profile_start_altitude_m-route->profile_final_altitude_m;
    if (!(drop>0.0) || !(limit>0.0) || limit>=89.0) return INFINITY;
    double lo=drop/tan(limit*d2r);
    double hi=fmax(lo,route->length_m);
    for(int i=0;i<8 && hermite_peak_descent_deg(route,hi)>limit;++i)
        hi*=1.25;
    if (hermite_peak_descent_deg(route,hi)>limit) return INFINITY;
    for(int i=0;i<24;++i) {
        double mid=0.5*(lo+hi);
        if (hermite_peak_descent_deg(route,mid)<=limit) hi=mid;
        else lo=mid;
    }
    return hi;
}


static double profile_lateral_authority_ratio(const TerminalModel *model,
        const TaemRoute *route, double live_curvature, double live_density) {
    if (!model || !route || !route->profile_tabulated ||
        !(route->length_m > 0.0) || !(live_curvature > 0.0) ||
        !(live_density > 0.0)) return INFINITY;

    double worst = 0.0;
    const int samples = 64;
    for (int i = 0; i <= samples; ++i) {
        double station = route->length_m * (double)i / (double)samples;
        double curvature = fabs(taem_route_curvature_at_station(route, station));
        double altitude = taem_route_planned_altitude(route, station);
        if (!isfinite(curvature) || !isfinite(altitude)) return INFINITY;
        AtmosphereSample atmosphere = world_atmosphere_sample(&model->world, altitude);
        if (!(atmosphere.density_kg_m3 > 0.0)) return INFINITY;
        double available_curvature = live_curvature *
            atmosphere.density_kg_m3 / live_density;
        if (!(available_curvature > 0.0) || !isfinite(available_curvature))
            return INFINITY;
        worst = fmax(worst, curvature / available_curvature);
    }
    return worst;
}





static bool replay_reaches_aligned_exit(const TerminalSolverResult *replay) {
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

static double estimated_energy_route_length(const TerminalModel *model,
        const TerminalDynamicState *initial, const TaemGeometryState *geometry) {
    if (!model || !initial || !geometry || !(initial->mass_kg > 1.0))
        return NAN;
    double target_speed = exit_speed_target(model);
    double final_height = model->guidance.final_approach_distance *
        tan(model->guidance.final_glide_slope *
            3.14159265358979323846 / 180.0);
    double radius = model->world.radius_m + model->site.altitude +
        fmax(0.0, geometry->altitude_above_runway_m);
    double gravity = model->world.mu_m3_s2 / fmax(radius * radius, 1.0);
    double energy = 0.5 * (geometry->airspeed_mps * geometry->airspeed_mps -
        target_speed * target_speed) +
        gravity * (geometry->altitude_above_runway_m - final_height);
    energy = fmax(0.0, energy);

    AeroForces forces = aero_compute(&model->world, &model->aero,
        initial->position_i_m, initial->velocity_i_mps, initial->ut_s,
        initial->mass_kg, initial->attitude.aoa_rad, initial->attitude.bank_rad);
    double drag = forces.drag_n > 0.0 ? forces.drag_n / initial->mass_kg : NAN;
    if (!(drag > 0.0) || !isfinite(drag)) return NAN;

    /* This is only a search-order estimate, never an admission rule.  Do not
       discount the measured drag: doing so lengthens the requested unpowered
       route even though density normally rises later in TAEM.  Native profile
       propagation remains the physical energy/vertical-feasibility proof. */
    double planning_drag = fmax(0.75, drag);
    return energy / planning_drag;
}

static bool better_failure(const TaemFixedHacCandidate *trial,
                           const TaemFixedHacCandidate *best, bool have_best) {
    if (!have_best) return true;
    if (trial->route_built != best->route_built) return trial->route_built;
    return trial->replay.elapsed_s > best->replay.elapsed_s;
}

static double candidate_circle_penalty(const TaemFixedHacCandidate *trial) {
    if (!trial || !trial->route_built) return INFINITY;
    double arc=fmax(trial->route.hac.arc_length_m,1.0);
    double lead=fmax(trial->route.lead_length_m,0.0);
    /* A HAC should spend its heading change on the analytic circle, not in a
     * long finite lead.  log1p keeps the penalty dimensionless and bounded
     * enough that energy feasibility remains dominant. */
    return log1p(lead/arc);
}

static bool candidate_preferred(const TerminalModel *model,
        const TaemFixedHacCandidate *trial, const TaemFixedHacCandidate *best) {
    if (trial->status != TAEM_PLAN_UNQUALIFIED) return false;
    if (best->status != TAEM_PLAN_UNQUALIFIED) return true;
    double target=exit_speed_target(model);
    double trial_speed=trial->replay.final_geometry.airspeed_mps;
    double best_speed=best->replay.final_geometry.airspeed_mps;
    double trial_shortfall=fmax(0.0,target-trial_speed);
    double best_shortfall=fmax(0.0,target-best_speed);
    /* An unpowered shuttle cannot repair a material energy deficit.  Keep
     * shortfall dominant, but treat differences below 2 m/s as prediction
     * noise so geometry can choose a steadier, more circular HAC. */
    if (fabs(trial_shortfall-best_shortfall)>2.0)
        return trial_shortfall<best_shortfall;
    return trial->quality_score<best->quality_score;
}
/* Replay qualified the HAC and runway-line roll-out. Rank by tracking quality,
 * energy closure and how far the aligned exit airspeed falls short of the
 * configured Final alignment speed. */
static void finish_candidate(const TerminalModel *model,
        const TerminalDynamicState *initial, TaemFixedHacCandidate *trial) {
    trial->status = TAEM_PLAN_UNQUALIFIED;
    trial->reason = "native HAC and runway alignment reach their exit; Final tail qualification is pending";
    double target_speed = exit_speed_target(model);
    double deficit = fabs(target_speed -
        trial->replay.final_geometry.airspeed_mps) / target_speed;
    double cross_limit = fmax(1200.0, 0.1 * trial->route.hac.radius_m);
    trial->quality_score = trial->replay.maximum_cross_track_m / cross_limit +
        trial->replay.maximum_course_error_deg / 20.0 +
        trial->replay.maximum_altitude_error_m / 2000.0 +
        fabs(trial->replay.energy_closure_residual_j_kg) /
            fmax(250.0, 0.02 * fmax(fabs(trial->replay.drag_work_j_kg),
                                    initial_kinetic(initial))) +
        4.0 * deficit +
        2.0 * trial->replay.turn_burden_integral_s /
            fmax(trial->replay.elapsed_s, 1.0) +
        2.0 * fmax(0.0,
            trial->replay.maximum_turn_authority_fraction - 0.8) +
        0.5 * trial->replay.bank_target_reversals +
        1.25 * candidate_circle_penalty(trial);
    if (!isfinite(trial->quality_score)) trial->quality_score = INFINITY;
}

static void mark_failure(TaemFixedHacCandidate *trial) {
    trial->status = trial->replay.status == TERMINAL_SOLVER_SEARCH_EXHAUSTED ?
        TAEM_PLAN_SEARCH_EXHAUSTED : TAEM_PLAN_INFEASIBLE;
    trial->reason = trial->replay.reason;
}

TaemFixedHacCandidate taem_fixed_hac_evaluate_route(const TerminalModel *model,
        const TerminalDynamicState *initial, const TaemRoute *route,
        int runway_end, double dt, double maximum_elapsed) {
    TaemFixedHacCandidate candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.status = TAEM_PLAN_INFEASIBLE;
    candidate.reason = "provisional route qualification failed";
    candidate.runway_end = runway_end;
    if (!model || !initial || !route || !route->valid ||
        !(dt > 0.0) || !(maximum_elapsed > 0.0))
        return candidate;

    candidate.side = route->side;
    candidate.route = *route;
    candidate.route_built = true;

    TerminalProfileResult profile = terminal_solver_generate_profile(
        model, initial, &candidate.route, dt, maximum_elapsed);
    if (!profile.valid) {
        candidate.reason = profile.reason ? profile.reason :
            "provisional route has no feasible native vertical profile";
        return candidate;
    }

    candidate.replay = terminal_solver_replay(
        model, initial, &candidate.route, dt, maximum_elapsed);
    if (replay_reaches_aligned_exit(&candidate.replay)) {
        finish_candidate(model, initial, &candidate);
        return candidate;
    }
    mark_failure(&candidate);
    return candidate;
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

    /* The lead occurs before the HAC join, so its most optimistic usable
       curvature authority is the authority available at the join altitude—not
       runway density.  Using runway density allowed the planner to generate
       turns that only become flyable after the maneuver should already have
       ended.  Compute a per-sweep ceiling below once the downstream HAC geometry
       is known. */
    double live_curvature = 0.95 * reachability.available_lateral_accel_mps2 /
        fmax(geometry->ground_speed_mps * geometry->ground_speed_mps, 1.0);
    double rho_live = world_atmosphere_sample(&model->world,
        model->site.altitude + geometry->altitude_above_runway_m).density_kg_m3;

    TaemFixedHacCandidate best_failure = candidate;
    bool have_failure = false;
    size_t count = 0;
    char reason[160];
    for (double sweep_deg = TAEM_JOIN_SWEEP_MAX_DEG;;) {
        double sweep_rad = sweep_deg * 3.14159265358979323846 / 180.0;
        TaemFixedHacGeometry sweep_geometry;
        if (!taem_hac_geometry_sweep(model, hac_radius, side, sweep_rad,
                &sweep_geometry))
            continue;

        /* The lead finishes at the HAC join, so its optimistic authority may
           grow only to the density expected at that join—not all the way to
           runway density. */
        double terminal_height = sweep_geometry.final_length_m *
            tan(model->guidance.final_glide_slope *
                3.14159265358979323846 / 180.0);
        double hac_height = terminal_height + sweep_geometry.arc_length_m *
            tan(fmin(model->guidance.taem_glide_slope, 12.0) *
                3.14159265358979323846 / 180.0);
        double rho_join = world_atmosphere_sample(&model->world,
            model->site.altitude + fmax(0.0, hac_height)).density_kg_m3;
        double density_gain = rho_live > 0.0 && rho_join > rho_live ?
            rho_join / rho_live : 1.0;
        double sweep_curvature_ceiling = live_curvature * density_gain;

        /* Three lead shapes per join: smoothest, shortest within authority,
           and an energy/vertical-geometry matched lead when the direct route is
           too short to satisfy the TAEM/Final endpoint tangents. */
        double energy_lead_length = NAN;
        for (int variant = 0;
             variant < 3 && count < TAEM_JOIN_MAX_STATIONS; ++variant) {
            JoinStation *station = &stations[count];
            station->sweep_rad = sweep_rad;
            if (variant == 2 && !isfinite(energy_lead_length)) continue;

            bool built = variant == 0 ?
                taem_route_build_hac(model, geometry, hac_radius, side,
                    station->sweep_rad, spacing, live_curvature,
                    &station->route, reason, sizeof(reason)) :
                variant == 1 ?
                taem_route_build_hac_shortest(model, geometry, hac_radius, side,
                    station->sweep_rad, spacing, live_curvature,
                    sweep_curvature_ceiling,
                    &station->route, reason, sizeof(reason)) :
                taem_route_build_hac_length(model, geometry, hac_radius, side,
                    station->sweep_rad, spacing, live_curvature,
                    sweep_curvature_ceiling, energy_lead_length,
                    &station->route, reason, sizeof(reason));

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
                fabs(stations[count - 1].route.length_m -
                     station->route.length_m) < 200.0)
                continue;

            double peak = taem_route_lead_peak_curvature(&station->route);
            double hac_curvature = 1.0 / station->route.hac.radius_m;
            if (!isfinite(peak)) continue;
            /* Do not apply the join-density curvature ceiling to the entire lead.
               The live q/speed state evolves along it; native propagation below
               is the authority gate for all non-initial stations. */
            /* Averages are insufficient here: with a shallower live inlet
               tangent and -Final-slope outlet tangent, a Hermite profile needs
               more distance than drop/tan(TAEM slope). */
            double profile_length =
                minimum_vertical_profile_length(model, &station->route);
            if (!isfinite(profile_length)) continue;
            if (variant == 0 &&
                profile_length > station->route.length_m + 1000.0)
                energy_lead_length = profile_length -
                    (station->route.length_m - station->route.lead_length_m);

            station->score =
                0.25 * fmax(0.0, peak - hac_curvature) / hac_curvature +
                fabs(station->route.length_m - profile_length) /
                    fmax(profile_length, station->route.hac.radius_m);
            ++count;
        }
        if (sweep_deg <= TAEM_JOIN_SWEEP_MIN_DEG + 1e-9) break;
        sweep_deg = fmax(TAEM_JOIN_SWEEP_MIN_DEG,
            sweep_deg - TAEM_JOIN_SWEEP_STEP_DEG);
    }
    if (count == 0) return best_failure;
    qsort(stations, count, sizeof(stations[0]), compare_station);

    /* Generate a dynamically feasible vertical reference by native
     * propagation.  Feasibility is only approximately monotone within one route
     * family; it is not monotone after different HAC radii/sweeps are mixed.
     * Order the expensive profile checks around the route length implied by the
     * current specific-energy surplus instead of bisecting the mixed list. */
    double target_speed = exit_speed_target(model);
    size_t profile_budget = count < TAEM_JOIN_MAX_PROFILES ? count : TAEM_JOIN_MAX_PROFILES;
    if (diagnostics_enabled() && count > 0) {
        size_t n = count < 8 ? count : 8;
        for (size_t i = 0; i < n; ++i)
            fprintf(stderr,
                "TAEM geometry: side=%+.0f radius=%.0f sweep=%.1f total=%.0f lead=%.0f arc=%.0f rollout=%.0f peakK=%.3e score=%.3f\n",
                side, stations[i].route.hac.radius_m,
                stations[i].sweep_rad * 180.0 / 3.14159265358979323846,
                stations[i].route.length_m, stations[i].route.lead_length_m,
                stations[i].route.hac.arc_length_m,
                stations[i].route.rollout_length_m,
                taem_route_lead_peak_curvature(&stations[i].route),
                stations[i].score);
    }
    qsort(stations, count, sizeof(stations[0]), compare_station_length);
    double target_length = estimated_energy_route_length(model, initial, geometry);
    if (!isfinite(target_length))
        target_length = stations[0].route.length_m;
    size_t center = 0;
    double center_error = fabs(stations[0].route.length_m - target_length);
    for (size_t i = 1; i < count; ++i) {
        double error = fabs(stations[i].route.length_m - target_length);
        if (error < center_error) {
            center = i;
            center_error = error;
        }
    }
    if (diagnostics_enabled()) {
        size_t n = count < 10 ? count : 10;
        fprintf(stderr,
            "TAEM energy length target: side=%+.0f target=%.0f nearest=%zu/%.0f count=%zu\n",
            side, target_length, center, stations[center].route.length_m, count);
        for (size_t i = 0; i < n; ++i)
            fprintf(stderr,
                "TAEM shortest: side=%+.0f idx=%zu/%zu radius=%.0f sweep=%.1f total=%.0f lead=%.0f arc=%.0f rollout=%.0f score=%.3f\n",
                side,i,count,stations[i].route.hac.radius_m,
                stations[i].sweep_rad*180.0/3.14159265358979323846,
                stations[i].route.length_m,stations[i].route.lead_length_m,
                stations[i].route.hac.arc_length_m,
                stations[i].route.rollout_length_m,stations[i].score);
    }
    unsigned char *tried = calloc(count, 1);
    TerminalProfileResult *results = calloc(count, sizeof(*results));
    if (!tried || !results) {
        free(tried); free(results);
        best_failure.reason = "profile workspace allocation failed";
        return best_failure;
    }
    size_t used = 0;
#define PROFILE_STATION(k) do {         if (!tried[(k)] && used < profile_budget) {             tried[(k)] = 1; ++used;             results[(k)] = terminal_solver_generate_profile(model, initial,                 &stations[(k)].route, TAEM_PROFILE_DT_S, maximum_elapsed);             if (diagnostics_enabled())                 fprintf(stderr, "TAEM profile: side=%+.0f radius=%.0f join_sweep=%.1f length=%.0f valid=%d aoa=%.2f exitV=%.1f exitFpa=%.1f reason=%s\n",                     side, stations[(k)].route.hac.radius_m,                     stations[(k)].sweep_rad * 180.0 / 3.14159265358979323846,                     stations[(k)].route.length_m, results[(k)].valid ? 1 : 0,                     results[(k)].aoa_deg, results[(k)].exit_speed_mps,                     results[(k)].exit_fpa_deg, results[(k)].reason);         } } while (0)
#define PROFILE_TOO_LONG(r) ((r) && (         strstr((r),"too long") ||         strcmp((r),"ground before route end")==0 ||         strcmp((r),"max elapsed before route end")==0 ||         strstr((r),"too slow for Final handoff")))
#define PROFILE_TOO_SHORT(r) ((r) && (         strstr((r),"too short") ||         strstr((r),"retains excess energy")))

    /* The energy-length estimate is only a search-order hint.  Spend the
       bounded profile budget on complete lead families first: a short member
       and its longest same-sweep companion give the refinement stage a real
       short/long bracket instead of unrelated points from the mixed list. */
    size_t anchor_sweeps = 0;
    double anchored_sweep[2] = {NAN, NAN};
    for (size_t i = 0; i < count && used < profile_budget &&
            anchor_sweeps < 2; ++i) {
        bool seen = false;
        for (size_t a = 0; a < anchor_sweeps; ++a)
            if (fabs(stations[i].sweep_rad - anchored_sweep[a]) < 1e-9)
                seen = true;
        if (seen) continue;

        anchored_sweep[anchor_sweeps++] = stations[i].sweep_rad;
        PROFILE_STATION(i);
        size_t mate = i;
        for (size_t j = i + 1; j < count; ++j) {
            if (fabs(stations[j].sweep_rad - stations[i].sweep_rad) < 1e-9 &&
                stations[j].route.lead_length_m > stations[mate].route.lead_length_m)
                mate = j;
        }
        if (mate != i) PROFILE_STATION(mate);
    }

    PROFILE_STATION(center);
    long left = (long)center - 1;
    size_t right = center + 1;
    while (used < profile_budget && (left >= 0 || right < count)) {
        bool take_left = false;
        if (left >= 0 && right < count) {
            double le = fabs(stations[left].route.length_m - target_length);
            double re = fabs(stations[right].route.length_m - target_length);
            take_left = le <= re;
        } else {
            take_left = left >= 0;
        }
        if (take_left) {
            PROFILE_STATION((size_t)left);
            --left;
        } else {
            PROFILE_STATION(right);
            ++right;
        }
    }

    /* If sampled variants of one exact HAC family straddle the feasible
       window, refine the lead continuously.  Search all tried pairs in that
       family; global length adjacency is meaningless across mixed geometry. */
    bool refined_valid = false;
    for (size_t i = 0; i < count && !refined_valid; ++i) {
        if (!tried[i] || results[i].valid) continue;
        bool i_short = PROFILE_TOO_SHORT(results[i].reason);
        bool i_long = PROFILE_TOO_LONG(results[i].reason);
        if (!i_short && !i_long) continue;
        for (size_t j = i + 1; j < count && !refined_valid; ++j) {
            if (!tried[j] || results[j].valid) continue;
            bool j_short = PROFILE_TOO_SHORT(results[j].reason);
            bool j_long = PROFILE_TOO_LONG(results[j].reason);
            if (!((i_short && j_long) || (i_long && j_short))) continue;
            JoinStation *a = &stations[i];
            JoinStation *b = &stations[j];
            if (fabs(a->sweep_rad - b->sweep_rad) > 1e-9 ||
                fabs(a->route.hac.radius_m - b->route.hac.radius_m) > 1e-6)
                continue;

            double short_lead = (i_short ? a : b)->route.lead_length_m;
            double long_lead = (i_long ? a : b)->route.lead_length_m;
            if (short_lead > long_lead) {
                double tmp = short_lead; short_lead = long_lead; long_lead = tmp;
            }
            if (!(long_lead - short_lead > 5.0)) continue;
            const TaemRoute *profile_short_route = i_short ? &a->route : &b->route;
            const TaemRoute *profile_long_route = i_long ? &a->route : &b->route;

            TaemFixedHacGeometry sweep_geometry;
            if (!taem_hac_geometry_sweep(model, a->route.hac.radius_m, side,
                    a->sweep_rad, &sweep_geometry))
                continue;
            double terminal_height = sweep_geometry.final_length_m *
                tan(model->guidance.final_glide_slope *
                    3.14159265358979323846 / 180.0);
            double hac_height = terminal_height + sweep_geometry.arc_length_m *
                tan(fmin(model->guidance.taem_glide_slope, 12.0) *
                    3.14159265358979323846 / 180.0);
            double rho_join = world_atmosphere_sample(&model->world,
                model->site.altitude + fmax(0.0, hac_height)).density_kg_m3;
            double density_gain = rho_live > 0.0 && rho_join > rho_live ?
                rho_join / rho_live : 1.0;
            double curvature_ceiling = live_curvature * density_gain;

            for (int refine = 0; refine < 5 && long_lead - short_lead > 2.0; ++refine) {
                double lead = 0.5 * (short_lead + long_lead);
                JoinStation candidate_station = {0};
                candidate_station.sweep_rad = a->sweep_rad;
                char refine_reason[160] = {0};
                if (!taem_route_blend_hac_length(model, geometry,
                        profile_short_route, profile_long_route, spacing,
                        live_curvature, curvature_ceiling, lead,
                        &candidate_station.route, refine_reason, sizeof(refine_reason)))
                    break;

                TerminalProfileResult pr = terminal_solver_generate_profile(
                    model, initial, &candidate_station.route,
                    TAEM_PROFILE_DT_S, maximum_elapsed);
                if (diagnostics_enabled())
                    fprintf(stderr,
                        "TAEM profile refine: side=%+.0f radius=%.0f sweep=%.1f lead=%.1f total=%.1f valid=%d exitV=%.1f exitFpa=%.1f reason=%s\n",
                        side, candidate_station.route.hac.radius_m,
                        candidate_station.sweep_rad * 180.0 / 3.14159265358979323846,
                        candidate_station.route.lead_length_m,
                        candidate_station.route.length_m, pr.valid ? 1 : 0,
                        pr.exit_speed_mps, pr.exit_fpa_deg,
                        pr.reason ? pr.reason : "none");

                if (pr.valid) {
                    candidate_station.score = 0.5 * (a->score + b->score);
                    stations[i] = candidate_station;
                    results[i] = pr;
                    tried[i] = 1;
                    refined_valid = true;
                    break;
                }
                if (PROFILE_TOO_SHORT(pr.reason))
                    short_lead = candidate_station.route.lead_length_m;
                else if (PROFILE_TOO_LONG(pr.reason))
                    long_lead = candidate_station.route.lead_length_m;
                else break;
            }
        }
    }
#undef PROFILE_STATION
#undef PROFILE_TOO_LONG
#undef PROFILE_TOO_SHORT

    size_t profiled = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!tried[i] || !results[i].valid) continue;
        double authority_ratio = profile_lateral_authority_ratio(model,
            &stations[i].route, live_curvature, rho_live);
        if (!isfinite(authority_ratio)) continue;
        if (diagnostics_enabled())
            fprintf(stderr,
                "TAEM authority: side=%+.0f sweep=%.1f length=%.0f ratio=%.3f leadPeakK=%.3e liveK=%.3e\n",
                side, stations[i].sweep_rad * 180.0 / 3.14159265358979323846,
                stations[i].route.length_m, authority_ratio,
                taem_route_lead_peak_curvature(&stations[i].route), live_curvature);

        /* Rank against the authority envelope before expensive closed-loop
           replay. Replay remains the final proof of feasibility. */
        stations[i].score +=
            fabs(target_speed - results[i].exit_speed_mps) / target_speed * 10.0 +
            20.0 * fmax(0.0, authority_ratio - 0.8);
        if (i != profiled) {
            JoinStation swap = stations[profiled];
            stations[profiled] = stations[i];
            stations[i] = swap;
        }
        ++profiled;
    }
    free(tried);
    free(results);
    if (profiled == 0) {
        best_failure.reason =
            "no HAC join has an energy-feasible native vertical profile";
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
                "TAEM candidate: side=%+.0f join_sweep=%.1f lead=%.0f arc=%.0f leadPeakK=%.3e liveK=%.3e status=%d elapsed=%.2f index=%zu/%zu reason=%s cross=%.1f course=%.1f altErr=%.1f exitV=%.1f latShort=%.2f turnMean=%.3f turnPeak=%.3f aoaMax=%.1f reversals=%u\n",
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
                trial.replay.maximum_lateral_authority_shortfall_mps2,
                trial.replay.turn_burden_integral_s /
                    fmax(trial.replay.elapsed_s, 1.0),
                trial.replay.maximum_turn_authority_fraction,
                trial.replay.maximum_commanded_aoa_deg,
                trial.replay.bank_target_reversals);
        if (replay_reaches_aligned_exit(&trial.replay)) {
            fprintf(stderr,
                "MM305_REPLAY_EXIT side=%+.0f radius=%.0f sweep=%.1f lead=%.0f refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f energy=%.1f reason=%s\n",
                trial.route.side, trial.route.hac.radius_m,
                trial.route.hac.arc_sweep_rad * 180.0 / 3.14159265358979323846,
                trial.route.lead_length_m,
                trial.replay.final_route_course_error_deg,
                trial.replay.final_geometry.heading_error_deg,
                trial.replay.final_geometry.runway_along_m,
                trial.replay.final_geometry.runway_cross_m,
                trial.replay.final_geometry.altitude_above_runway_m,
                trial.replay.final_geometry.airspeed_mps,
                trial.replay.final_geometry.flight_path_angle_deg,
                trial.replay.final_state.attitude.bank_rad * 180.0 / 3.14159265358979323846,
                trial.replay.final_state.attitude.aoa_rad * 180.0 / 3.14159265358979323846,
                trial.replay.energy_end_j_kg,
                trial.replay.reason ? trial.replay.reason : "none");
            finish_candidate(model, initial, &trial);
            if (diagnostics_enabled())
                fprintf(stderr, "TAEM rank: side=%+.0f radius=%.0f sweep=%.1f exitV=%.1f quality=%.3f reversals=%u\n",
                    side, trial.route.hac.radius_m,
                    trial.route.hac.arc_sweep_rad * 57.29577951308232,
                    trial.replay.final_geometry.airspeed_mps,
                    trial.quality_score, trial.replay.bank_target_reversals);
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
                 * bounded search variable.  The configured radius is only the
                 * top of the search, not a preferred answer.  Evaluate tighter
                 * circles as well and let Final-speed closure plus replayed turn/
                 * tracking quality choose the radius.  The lower bound is the
                 * steady turn radius at the Final alignment speed and bank limit,
                 * never below the 3 km HAC floor. */
                double side = side_index == 0 ? -1.0 : 1.0;
                double floor_radius = fmin(hac_radius, hac_radius_floor(ends[end]));
                double radius = hac_radius;
                *slot = evaluate_side(ends[end], initial, &geometry, radius,
                    side, spacing, dt, maximum_elapsed, stations);
                for (int step = 1; step < TAEM_HAC_RADIUS_STEPS; ++step) {
                    double fraction = (double)step / (double)(TAEM_HAC_RADIUS_STEPS - 1);
                    double next_radius = hac_radius +
                        fraction * (floor_radius - hac_radius);
                    if (!(next_radius < radius - 1.0)) continue;
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
