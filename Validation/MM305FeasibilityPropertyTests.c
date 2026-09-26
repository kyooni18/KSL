#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "landing_api.h"
#include "mm305_planning.h"
#include "sim_telemetry.h"
#include "taem_geometry.h"
#include "terminal_model.h"
#include "shuttlesim/math3.h"

static PlanetModel test_planet(void) {
    PlanetModel p = {
        .radius = 600000.0,
        .gravitational_parameter = 3531600000000.0,
        .rotational_speed = 0.000291570900559802,
        .atmosphere_depth = 70000.0,
        .surface_density = 1.225,
        .atmosphere_adiabatic_index = 1.4,
    };
    p.atmosphere_sample_count = 5;
    p.atmosphere_altitude[0] = 0.0;     p.atmosphere_pressure[0] = 101325.0; p.atmosphere_density[0] = 1.225;
    p.atmosphere_altitude[1] = 15000.0; p.atmosphere_pressure[1] = 12000.0;  p.atmosphere_density[1] = 0.18;
    p.atmosphere_altitude[2] = 25000.0; p.atmosphere_pressure[2] = 3600.0;   p.atmosphere_density[2] = 0.05;
    p.atmosphere_altitude[3] = 50000.0; p.atmosphere_pressure[3] = 120.0;    p.atmosphere_density[3] = 0.002;
    p.atmosphere_altitude[4] = 70000.0; p.atmosphere_pressure[4] = 1.0;      p.atmosphere_density[4] = 1e-6;
    return p;
}

static Telemetry create_telemetry(const PlanetModel *p, const LandingConfiguration *cfg,
                                  double altitude, double mach, double along, double cross) {
    Telemetry t;
    memset(&t, 0, sizeof(t));
    t.latitude = cfg->site.latitude;
    t.longitude = cfg->site.longitude;
    t.mean_altitude = altitude;
    t.radar_altitude = altitude - cfg->site.altitude;
    double sound = planet_atmospheric_speed_of_sound(p, t.mean_altitude);
    t.mach = mach;
    t.true_air_speed = sound * t.mach;
    t.horizontal_speed = t.true_air_speed * cos(8.0 * DEG2RAD);
    t.vertical_speed = -t.true_air_speed * sin(8.0 * DEG2RAD);
    t.flight_path_angle = -8.0;
    t.runway_along_track = along;
    t.runway_cross_track = cross;
    t.dynamic_pressure = fmin(7000.0, 0.35 * cfg->vehicle.maximum_dynamic_pressure);
    t.g_force = fmin(1.2, 0.35 * cfg->vehicle.maximum_g_load);
    t.mass = 40000.0;
    t.angle_of_attack = 18.0;
    t.sideslip = 0.0;
    t.lift_force = t.mass * 8.0;
    t.drag_force = t.mass * 1.0;
    t.bank_effectiveness = 1.0;
    t.estimated_ballistic_coefficient = cfg->vehicle.estimated_ballistic_coefficient;
    t.aerodynamic_confidence = 0.9;
    t.attitude_response.roll_valid = true;
    t.attitude_response.pitch_valid = true;
    t.attitude_response.maximum_roll_rate_deg_s = 18.0;
    t.attitude_response.maximum_roll_accel_deg_s2 = 12.0;
    t.attitude_response.maximum_pitch_rate_deg_s = 8.0;
    t.attitude_response.maximum_pitch_accel_deg_s2 = 5.0;
    t.stall_fraction_is_measured = true;
    t.stall_fraction = 0.0;
    return t;
}

static TerminalDynamicState telemetry_to_terminal_state(const TerminalModel *model,
                                                        const Telemetry *t,
                                                        double runway_heading_deg) {
    TerminalDynamicState state;
    memset(&state, 0, sizeof(state));
    state.ut_s = 1000.0;
    state.mass_kg = t->mass;

    /* Use the production spherical frame in both directions. The old tangent
     * offset reversed crossrange and changed altitude/course at large range.
     * A zeroed attitude model also made every replay fail validation. */
    state.attitude = model->attitude;
    state.attitude.aoa_rad = t->angle_of_attack * DEG2RAD;
    state.attitude.bank_rad = t->roll * DEG2RAD;
    TaemFrameWorld world;
    TaemRunwayFrame runway;
    TaemVec3 position_b, velocity_b, north, east, up, position_i, velocity_i;
    assert(taem_geometry_runway(model, &world, &runway));
    assert(taem_runway_unproject(&world, &runway, t->runway_along_track,
        t->runway_cross_track, t->mean_altitude-model->site.altitude, &position_b));
    assert(taem_local_north_east_up(position_b, &north, &east, &up));
    double psi = runway_heading_deg * DEG2RAD;
    velocity_b = taem_vec3_add(taem_vec3_scale(north, t->horizontal_speed*cos(psi)),
        taem_vec3_scale(east, t->horizontal_speed*sin(psi)));
    velocity_b = taem_vec3_add(velocity_b, taem_vec3_scale(up, t->vertical_speed));
    assert(taem_fixed_to_inertial(&world, position_b, velocity_b, state.ut_s,
        &position_i, &velocity_i));
    state.position_i_m = (Vec3){position_i.x, position_i.y, position_i.z};
    state.velocity_i_mps = (Vec3){velocity_i.x, velocity_i.y, velocity_i.z};
    assert(terminal_state_validate(&state, NULL, 0));
    TaemGeometryState geometry;
    assert(taem_geometry_state(model, &state, &geometry));
    assert(fabs(geometry.runway_along_m-t->runway_along_track)<1e-6);
    assert(fabs(geometry.runway_cross_m-t->runway_cross_track)<1e-6);
    assert(fabs(geometry.altitude_above_runway_m-t->radar_altitude)<1e-6);
    assert(fabs(geometry.flight_path_angle_deg-t->flight_path_angle)<1e-6);
    assert(fabs(geometry.airspeed_mps-t->true_air_speed)<1e-6);

    return state;
}

int main(void) {
    LandingConfiguration cfg = landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.latitude = -0.0486111111;
    cfg.site.longitude = -74.7283333333;
    cfg.site.altitude = 70.0;
    cfg.site.runway_heading = 90.0;
    PlanetModel p = test_planet();

    char atmosphere[512], aero[512], book[512], attitude[512];
    TerminalModelSourceFiles files = {
        .atmosphere_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATMOSPHERE, atmosphere, sizeof(atmosphere)),
        .aero_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_AERO, aero, sizeof(aero)),
        .aero_book_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_FORCE_BOOK, book, sizeof(book)),
        .attitude_ini = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATTITUDE, attitude, sizeof(attitude))
    };

    TerminalModel model;
    char reason[256];
    assert(terminal_model_capture(&model, &files, &cfg, 101, reason, sizeof(reason)));
    assert(model.replay_validated);

    GuidanceMachine g;
    memset(&g, 0, sizeof(g));
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;

    /* Property test: Sample points strictly inside the MM305 admission set */
    typedef struct { double alt, mach, along, cross; } AdmittedSample;
    const AdmittedSample samples[] = {
        { 0.5 * (cfg.guidance.mm305_min_altitude + cfg.guidance.mm305_max_altitude), cfg.guidance.mm305_target_mach, -30000.0, 0.0 },
        { cfg.guidance.mm305_max_altitude - 500.0, cfg.guidance.mm305_target_mach + 0.15, -35000.0, 5000.0 },
        { cfg.guidance.mm305_min_altitude + 500.0, cfg.guidance.mm305_target_mach - 0.15, -28000.0, -5000.0 }
    };

    int total_admitted = 0;
    int feasible_count = 0;

    for (size_t i = 0; i < sizeof(samples)/sizeof(samples[0]); ++i) {
        Telemetry t = create_telemetry(&p, &cfg, samples[i].alt, samples[i].mach, samples[i].along, samples[i].cross);
        TaemInterfaceCapture capture = entry_mm305_admission_envelope(&g, &t, cfg.site.runway_heading, &p, &cfg);

        if (!capture.ready || capture.veto != 0u) {
            continue; /* Test property only for states inside the admission set */
        }
        total_admitted++;

        TerminalDynamicState state = telemetry_to_terminal_state(&model, &t, cfg.site.runway_heading);
        Mm305PlanRequest req = {
            .valid = true,
            .request_ut = state.ut_s,
            .model_snapshot_id = model.snapshot_id,
            .state = state,
            .hac_radius_m = model.guidance.hac_radius,
            .search_both_ends = false,
            .upstream_end = 0,
            .lift_scale = 1.0,
            .drag_scale = 1.0
        };

        Mm305PlanResult result = mm305_plan(&model, &req);
        printf("sample=%zu altitude=%.1f mach=%.3f along=%.1f cross=%.1f found=%d solve_wall=%.3f reason=%s\n",
            i, t.mean_altitude, t.mach, t.runway_along_track, t.runway_cross_track,
            result.found, result.solve_wall_s, result.diagnostic);
        if (result.valid && result.found) {
            feasible_count++;
        }
    }

    assert(total_admitted > 0);
    printf("MM305 admission-feasibility property test: sampled %d admitted states, %d feasible routes found.\n",
           total_admitted, feasible_count);

    /* Qualification cannot depend on an opt-in environment flag. A failed
     * search is an admission/planner mismatch, not proof of physical
     * impossibility. Keep the full replay and report the failed contract. */
    if (feasible_count != total_admitted) {
        fprintf(stderr, "FAIL: MM305 admission/planner mismatch: %d/%d admitted states have no qualified route.\n",
                total_admitted-feasible_count, total_admitted);
        return EXIT_FAILURE;
    }
    puts("MM305 admission-feasibility property test passed.");
    return EXIT_SUCCESS;
}
