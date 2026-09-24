#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "landing_api.h"
#include "taem_candidate_search.h"
#include "taem_geometry.h"
#include "taem_reachability.h"
#include "terminal_solver.h"
#include "shuttlesim/math3.h"

static double fixture_value(const char *path, const char *key) {
    FILE *f = fopen(path, "r");
    assert(f);
    char line[256];
    size_t key_len = strlen(key);
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, key, key_len) == 0 && line[key_len] == '=') {
            char *end = NULL;
            double value = strtod(line + key_len + 1, &end);
            assert(end && (*end == '\n' || *end == '\0'));
            fclose(f);
            return value;
        }
    }
    fclose(f);
    assert(!"missing numeric scenario key");
    return NAN;
}

static TerminalDynamicState load_fixture_state(const TerminalModel *model) {
    const char *path = "ShuttleSim/scenarios/mm305-hac-ideal-p0-v220.ini";
    TerminalDynamicState state = {0};
    state.position_i_m = (Vec3){
        fixture_value(path, "position_x_m"),
        fixture_value(path, "position_y_m"),
        fixture_value(path, "position_z_m")
    };
    state.velocity_i_mps = (Vec3){
        fixture_value(path, "velocity_x_mps"),
        fixture_value(path, "velocity_y_mps"),
        fixture_value(path, "velocity_z_mps")
    };
    state.mass_kg = fixture_value(path, "mass_kg");
    state.ut_s = fixture_value(path, "ut0");
    state.attitude = model->attitude;
    state.attitude.aoa_rad = fixture_value(path, "initial_aoa_deg") *
        3.14159265358979323846 / 180.0;
    state.attitude.bank_rad = fixture_value(path, "initial_bank_deg") *
        3.14159265358979323846 / 180.0;
    return state;
}

static double first_lead_sample_distance(const TaemRoute *route) {
    const size_t intervals = TAEM_ROUTE_LEAD_LUT_POINTS - 1;
    const double target = 1.0 / (double)route->lead_count;
    size_t upper = 1;
    while (upper < intervals &&
           route->lead_arc_fraction_lut[upper] < target) ++upper;
    size_t lower = upper - 1;
    double a = route->lead_arc_fraction_lut[lower];
    double b = route->lead_arc_fraction_lut[upper];
    double part = b > a ? (target - a) / (b - a) : 0.0;
    double t = ((double)lower + part) / (double)intervals;
    double q = 1.0 - t;
    double x = q*q*q*route->p0_along_m + 3.0*q*q*t*route->p1_along_m +
        3.0*q*t*t*route->p2_along_m + t*t*t*route->p3_along_m;
    double y = q*q*q*route->p0_cross_m + 3.0*q*q*t*route->p1_cross_m +
        3.0*q*t*t*route->p2_cross_m + t*t*t*route->p3_cross_m;
    return hypot(x - route->p0_along_m, y - route->p0_cross_m);
}

int main(void) {
    LandingConfiguration configuration = landing_configuration_default();
    TerminalModelSourceFiles files = {
        .atmosphere_csv = "ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv",
        .aero_csv = "ShuttleSim/data/fitted/stsn_aero_ksp_robust.csv",
        .aero_book_csv = "ShuttleSim/data/fitted/stsn_force_book.csv",
        .attitude_ini = "ShuttleSim/data/fitted/stsn_attitude_ksp.ini"
    };
    TerminalModel model;
    char reason[192];
    assert(terminal_model_capture(&model, &files, &configuration, 41,
                                  reason, sizeof(reason)));
    assert(model.replay_validated && model.snapshot_id == 41);

    TerminalDynamicState state = load_fixture_state(&model);
    assert(terminal_state_validate(&state, reason, sizeof(reason)));
    TaemGeometryState geometry;
    assert(taem_geometry_state(&model, &state, &geometry));

    TaemReachability reachability;
    assert(taem_fixed_hac_turn_reachability(&model, &state, &geometry,
        model.guidance.hac_radius, &reachability));
    assert(reachability.valid && reachability.lateral_authority_ok);
    double maximum_curvature = 0.95 * reachability.available_lateral_accel_mps2 /
        fmax(geometry.airspeed_mps * geometry.airspeed_mps, 1.0);

    TaemFixedHacGeometry fixed_geometry;
    assert(taem_fixed_hac_geometry(&model, model.guidance.hac_radius, 1.0,
                                   &fixed_geometry));
    double capture_delta = (fixed_geometry.capture_course_deg -
        model.site.runway_heading) * 3.14159265358979323846 / 180.0;
    TaemGeometryState tangent_start = geometry;
    tangent_start.runway_along_m = fixed_geometry.entry.x -
        5000.0 * cos(capture_delta);
    tangent_start.runway_cross_m = fixed_geometry.entry.y -
        5000.0 * sin(capture_delta);
    tangent_start.course_deg = fixed_geometry.capture_course_deg;

    TaemRoute route;
    assert(taem_route_build_fixed_hac(&model, &tangent_start, 1.0, 500.0,
        maximum_curvature, &route, reason, sizeof(reason)));
    assert(route.valid && route.count > 3 && route.count <= TAEM_ROUTE_MAX_POINTS);
    assert(route.lead_arc_fraction_lut[0] == 0.0f);
    assert(route.lead_arc_fraction_lut[TAEM_ROUTE_LEAD_LUT_POINTS - 1] == 1.0f);
    assert(first_lead_sample_distance(&route) <= 550.0);

    /* The subsonic terminal lift cap is shared by the MM305 reachability
     * estimate and tracker search; never ask the native tracker to use a
     * post-stall angle above the configured terminal CL-max incidence. */
    TerminalModel limited_model = model;
    limited_model.vehicle.terminal_maximum_lift_angle_of_attack = 3.0;
    AeroForces flow = aero_compute(&limited_model.world, &limited_model.aero,
        state.position_i_m, state.velocity_i_mps, state.ut_s, state.mass_kg,
        0.0, 0.0);
    assert(flow.mach < 1.0);
    TaemPathReference high_lift_reference = {
        .runway_along_m = geometry.runway_along_m,
        .runway_cross_m = geometry.runway_cross_m,
        .course_deg = geometry.course_deg,
        .curvature_right_per_m = 0.0,
        .altitude_m = geometry.altitude_above_runway_m,
        .flight_path_angle_deg = geometry.flight_path_angle_deg + 20.0
    };
    TaemTrackerOutput capped_demand = taem_tracker_update(&limited_model,
        &state, &geometry, &high_lift_reference, 0.25);
    assert(capped_demand.valid);
    assert(fabs(capped_demand.control.angle_of_attack_rad *
        180.0 / 3.14159265358979323846 - 3.0) < 1e-9);
    TaemRoute authority_probe_route;
    assert(taem_route_build_fixed_hac(&limited_model, &geometry, 1.0,
        500.0, 1.0, &authority_probe_route, reason, sizeof(reason)));
    TerminalSolverResult authority_probe = terminal_solver_replay(
        &limited_model, &state, &authority_probe_route, 0.25, 10.0);
    assert(authority_probe.status == TERMINAL_SOLVER_INFEASIBLE);
    assert(strcmp(authority_probe.reason,
        "tracker exceeded vertical control authority") == 0);
    assert(authority_probe.elapsed_s == 0.0);

    /* A fixed-size descriptor copies independently with GuidanceMachine state. */
    GuidanceMachine first = {0};
    first.mm305_route = route;
    first.mm305_route_committed = true;
    first.mm305_model_snapshot_id = model.snapshot_id;
    GuidanceMachine snapshot = first;
    assert(snapshot.mm305_route_committed);
    assert(snapshot.mm305_model_snapshot_id == first.mm305_model_snapshot_id);
    assert(memcmp(&snapshot.mm305_route, &first.mm305_route, sizeof(route)) == 0);
    assert(sizeof(TaemRoute) < 1024);

    /* The MM305 descriptor ends at HAC exit at the configured Final glide
     * height; the untouched Final contract must still admit the live state. */
    size_t exit_cursor = route.count - 1;
    TaemGeometryState at_exit = geometry;
    at_exit.runway_along_m = route.hac.exit.x;
    at_exit.runway_cross_m = route.hac.exit.y;
    TaemPathReference exit_reference;
    size_t exit_index = 0;
    assert(taem_route_reference(&route, &at_exit, &exit_cursor,
        &exit_reference, &exit_index));
    double expected_exit_altitude = model.site.altitude +
        route.hac.final_length_m * tan(model.guidance.final_glide_slope *
                                       3.14159265358979323846 / 180.0);
    assert(exit_index == route.count - 1);
    assert(fabs(exit_reference.altitude_m - expected_exit_altitude) < 1e-6);
    assert(fabs(exit_reference.flight_path_angle_deg +
                model.guidance.final_glide_slope) < 1e-6);
    assert(exit_reference.altitude_m > model.site.altitude);

    /* Interior profile candidates preserve the live start and HAC-exit
     * altitude/FPA contracts while changing only the path between them. */
    TaemRoute shaped_route = route;
    shaped_route.profile_midpoint_offset_m = 1200.0;
    shaped_route.profile_initial_sag_m = -120.0;
    shaped_route.profile_initial_sag_length_m = 3000.0;
    size_t shaped_start_cursor = 0;
    TaemPathReference shaped_start;
    size_t shaped_start_index = SIZE_MAX;
    assert(taem_route_reference(&shaped_route, &tangent_start,
        &shaped_start_cursor, &shaped_start, &shaped_start_index));
    assert(shaped_start_index == 0);
    assert(fabs(shaped_start.altitude_m - route.profile_start_altitude_m) < 1e-6);
    assert(fabs(shaped_start.flight_path_angle_deg -
                route.profile_start_fpa_deg) < 1e-6);
    size_t shaped_exit_cursor = shaped_route.count - 1;
    TaemPathReference shaped_exit;
    assert(taem_route_reference(&shaped_route, &at_exit, &shaped_exit_cursor,
        &shaped_exit, NULL));
    assert(fabs(shaped_exit.altitude_m - exit_reference.altitude_m) < 1e-6);
    assert(fabs(shaped_exit.flight_path_angle_deg -
                exit_reference.flight_path_angle_deg) < 1e-6);

    /* Geometry alone cannot qualify a candidate: native replay must satisfy
     * all constraints through HAC exit before the existing Final-tail gate. */
    TaemFixedHacSearch search = taem_fixed_hac_search(&model, &state,
        500.0, 0.25, 1800.0);
    assert(search.selected_candidate == -1);
    for (size_t i = 0; i < 2; ++i) {
        const TaemFixedHacCandidate *candidate = &search.candidates[i];
        assert(candidate->status != TAEM_PLAN_UNQUALIFIED ||
               candidate->route_built);
        assert(candidate->replay.status != TERMINAL_SOLVER_UNQUALIFIED ||
               !candidate->replay.path_constraints_ok);
    }

    TerminalDynamicState faster_state = state;
    faster_state.velocity_i_mps = v3_scale(state.velocity_i_mps, 3.5);
    TaemGeometryState faster_geometry;
    assert(taem_geometry_state(&model, &faster_state, &faster_geometry));
    TaemReachability faster_reachability;
    assert(taem_fixed_hac_turn_reachability(&model, &faster_state,
        &faster_geometry, model.guidance.hac_radius, &faster_reachability));
    assert(!faster_reachability.lateral_authority_ok);
    TaemFixedHacSearch faster_search = taem_fixed_hac_search(&model,
        &faster_state, 500.0, 0.25, 1800.0);
    for (size_t i = 0; i < 2; ++i)
        assert(strcmp(faster_search.candidates[i].reason,
            "fixed HAC circle exceeds live turn authority") != 0);
    puts("TAEM native model, route descriptor, Final interface, and replay gate tests passed.");
    return 0;
}
