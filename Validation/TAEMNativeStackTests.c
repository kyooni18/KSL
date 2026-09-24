#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "landing_api.h"
#include "taem_candidate_search.h"
#include "taem_geometry.h"
#include "taem_reachability.h"
#include "terminal_solver.h"

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

    TaemRoute route;
    assert(taem_route_build_fixed_hac(&model, &geometry, 1.0, 500.0,
        maximum_curvature, &route, reason, sizeof(reason)));
    assert(route.valid && route.count > 3 && route.count <= TAEM_ROUTE_MAX_POINTS);

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

    /* Geometry alone cannot qualify a candidate: native replay must satisfy
     * all constraints through HAC exit before the existing Final-tail gate. */
    TaemFixedHacSearch search = taem_fixed_hac_search(&model, &state,
        500.0, 0.25, 1800.0);
    assert(search.selected_candidate == -1);
    for (size_t i = 0; i < 2; ++i) {
        const TaemFixedHacCandidate *candidate = &search.candidates[i];
        assert(candidate->route_built);
        assert(candidate->replay.status != TERMINAL_SOLVER_UNQUALIFIED ||
               !candidate->replay.path_constraints_ok);
    }
    puts("TAEM native model, route descriptor, Final interface, and replay gate tests passed.");
    return 0;
}
