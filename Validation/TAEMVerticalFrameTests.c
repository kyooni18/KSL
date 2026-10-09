#include "landing_api.h"
#include "sim_telemetry.h"
#include "taem_tracker.h"
#include "terminal_model.h"
#include "shuttlesim/math3.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double relative_gamma(const TerminalModel *model, const TerminalDynamicState *state) {
    Vec3 air = v3_sub(state->velocity_i_mps,
        world_atmosphere_velocity_i(&model->world, state->position_i_m));
    Vec3 up = v3_normalized(state->position_i_m);
    return asin(v3_dot(air, up) / v3_norm(air));
}

/* Independent central difference of a gravity-only inertial trajectory.
   No gamma-rate formula or fictitious-force expression is used by this oracle. */
static double gravity_only_gamma_rate(const TerminalModel *model, TerminalDynamicState state) {
    const double epsilon = 0.001;
    Vec3 gravity = world_gravity_accel(&model->world, state.position_i_m);
    TerminalDynamicState plus = state, minus = state;
    plus.ut_s += epsilon;
    minus.ut_s -= epsilon;
    Vec3 half_accel = v3_scale(gravity, 0.5 * epsilon * epsilon);
    plus.position_i_m = v3_add(v3_add(state.position_i_m,
        v3_scale(state.velocity_i_mps, epsilon)), half_accel);
    minus.position_i_m = v3_add(v3_sub(state.position_i_m,
        v3_scale(state.velocity_i_mps, epsilon)), half_accel);
    plus.velocity_i_mps = v3_add(state.velocity_i_mps, v3_scale(gravity, epsilon));
    minus.velocity_i_mps = v3_sub(state.velocity_i_mps, v3_scale(gravity, epsilon));
    return (relative_gamma(model, &plus) - relative_gamma(model, &minus)) / (2.0 * epsilon);
}

int main(void) {
    assert(setenv("KSP_LANDER_MODEL_PROFILE", "identified", 1) == 0);
    LandingConfiguration cfg = landing_configuration_default();
    char atmosphere[512], aero[512], book[512], attitude[512], reason[256];
    TerminalModelSourceFiles files = {
        .atmosphere_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATMOSPHERE, atmosphere, sizeof(atmosphere)),
        .aero_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_AERO, aero, sizeof(aero)),
        .aero_book_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_FORCE_BOOK, book, sizeof(book)),
        .attitude_ini = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATTITUDE, attitude, sizeof(attitude)),
    };
    TerminalModel model;
    assert(terminal_model_capture(&model, &files, &cfg, 1, reason, sizeof(reason)));
    const double rotation = model.world.rotation_rate_rad_s;
    const double latitudes[] = {0.0, 45.0, -70.0};
    const double courses[] = {90.0, 270.0, 0.0};
    const double speeds[] = {160.0, 800.0, 1800.0};
    const double gammas[] = {0.0, -8.0, -25.0};
    unsigned checked = 0;
    for (int rotating = 0; rotating < 2; ++rotating)
    for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
    for (unsigned k = 0; k < 3; ++k) {
        model.world.rotation_rate_rad_s = rotating ? rotation : 0.0;
        TerminalDynamicState state = {.ut_s = 1000.0, .mass_kg = 25816.072266};
        state.attitude = model.attitude;
        state.position_i_m = world_lla_to_inertial(&model.world, deg2rad(latitudes[i]),
                                                  deg2rad(-74.7), 26000.0, state.ut_s);
        LocalFrame local = world_local_frame_i(&model.world, state.position_i_m, state.ut_s);
        double speed = speeds[k], gamma = deg2rad(gammas[k]), course = deg2rad(courses[j]);
        Vec3 tangent = v3_add(v3_scale(local.north, cos(course)), v3_scale(local.east, sin(course)));
        Vec3 air = v3_add(v3_scale(tangent, speed * cos(gamma)), v3_scale(local.up, speed * sin(gamma)));
        state.velocity_i_mps = v3_add(air, world_atmosphere_velocity_i(&model.world, state.position_i_m));
        TaemGeometryState geometry;
        assert(taem_geometry_state(&model, &state, &geometry));
        TaemPathReference reference = {
            .runway_along_m = geometry.runway_along_m,
            .runway_cross_m = geometry.runway_cross_m,
            .course_deg = geometry.course_deg,
            .altitude_m = model.site.altitude + geometry.altitude_above_runway_m,
            .flight_path_angle_deg = geometry.flight_path_angle_deg,
        };
        TaemTrackerOutput command = taem_tracker_update(&model, &state, &geometry, &reference, 0.1);
        assert(command.valid);
        double free_rate = gravity_only_gamma_rate(&model, state);
        double expected = fmax(0.0, -geometry.airspeed_mps * free_rate);
        /* The profile initializer uses the signed support, including cases
           where free flight already curves upward. Verify against the same
           independent oracle rather than just comparing the two callers. */
        double support = taem_tracker_vertical_lift_demand(&model, &state, &geometry, 0.0);
        assert(fabs(support + geometry.airspeed_mps * free_rate) < 0.00002);
        double rate = deg2rad(0.5);
        assert(fabs(taem_tracker_vertical_lift_demand(&model, &state, &geometry, rate) -
            geometry.airspeed_mps * (rate - free_rate)) < 0.00002);
        if (fabs(command.required_vertical_lift_mps2 - expected) > 0.00002) {
            fprintf(stderr, "rotating=%d lat=%.0f course=%.0f airspeed=%.0f gamma=%.0f: support %.9f != propagated %.9f\n",
                rotating, latitudes[i], courses[j], speed, gammas[k],
                command.required_vertical_lift_mps2, expected);
            return EXIT_FAILURE;
        }
        ++checked;
    }
    printf("TAEM vertical support matches independent inertial propagation in %u cases.\n", checked);
    return 0;
}
