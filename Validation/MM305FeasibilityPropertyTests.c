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
                                  double altitude, double mach, double along, double cross, double mass_kg) {
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
    t.mass = mass_kg;
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
    /* Match production terminal_live_state: seed command history from the
     * observed attitude, not the model-file zero default. Otherwise replay
     * invents a pitch-down command before tracking the candidate route. */
    state.attitude.cmd_aoa_rad = state.attitude.requested_aoa_rad =
        state.attitude.aoa_rad;
    state.attitude.cmd_bank_rad = state.attitude.requested_bank_rad =
        state.attitude.bank_rad;
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

static bool load_configuration_file(const char *path, LandingConfiguration *cfg) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool ok = false;
    char *text = NULL;
    JsonToken *tokens = NULL;
    if (fseek(file, 0, SEEK_END) != 0) goto done;
    long length = ftell(file);
    if (length <= 0 || length > 1048576 || fseek(file, 0, SEEK_SET) != 0) goto done;
    text = calloc((size_t)length + 1, 1);
    tokens = calloc(16384, sizeof(*tokens));
    if (!text || !tokens || fread(text, 1, (size_t)length, file) != (size_t)length) goto done;
    JsonDoc document;
    if (json_parse(text, tokens, 16384, &document) <= 0) goto done;
    ok = landing_configuration_from_json(cfg, &document, 0);
done:
    free(tokens);
    free(text);
    fclose(file);
    return ok;
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    LandingConfiguration cfg = landing_configuration_default();
    const char *configuration_path = NULL;
    double mass_kg = 40000.0;
    bool explicit_mass = false;
    for (int n = 1; n < argc; ++n) {
        if (!strcmp(argv[n], "--configuration") && n + 1 < argc) {
            configuration_path = argv[++n];
        } else if (!strcmp(argv[n], "--mass-kg") && n + 1 < argc) {
            char *end = NULL;
            mass_kg = strtod(argv[++n], &end);
            if (!end || end == argv[n] || *end || !isfinite(mass_kg) || mass_kg <= 0.0) {
                fprintf(stderr, "mass must be a positive finite number in kg\n");
                return EXIT_FAILURE;
            }
            explicit_mass = true;
        } else {
            fprintf(stderr, "usage: %s [--configuration FILE --mass-kg MEASURED_MASS]\n", argv[0]);
            return EXIT_FAILURE;
        }
    }
    if (configuration_path && (!explicit_mass || !load_configuration_file(configuration_path, &cfg))) {
        fprintf(stderr, "explicit configuration requires a readable config and --mass-kg\n");
        return EXIT_FAILURE;
    }
    landing_configuration_normalize(&cfg);
    const char *profile = shuttle_sim_model_profile();
    const char *profile_vehicle = shuttle_sim_model_vehicle_id();
    if (!profile_vehicle || strcmp(profile_vehicle, cfg.vehicle.model_id)) {
        fprintf(stderr, "model mismatch: profile %s describes %s, configuration describes %s\n",
            profile ? profile : "unknown", profile_vehicle ? profile_vehicle : "unknown", cfg.vehicle.model_id);
        return EXIT_FAILURE;
    }
    printf("MM305 fixture: profile=%s modelId=%s massKg=%.6f configuration=%s state=%s\n",
        profile, cfg.vehicle.model_id, mass_kg, configuration_path ? configuration_path : "native-default",
        configuration_path ? "native-atmosphere-and-forces" : "legacy-envelope-property-fixture");
    fflush(stdout);
    if (!configuration_path) {
        cfg.site.latitude = -0.0486111111;
        cfg.site.longitude = -74.7283333333;
        cfg.site.altitude = 70.0;
        cfg.site.runway_heading = 90.0;
    }
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

    if (configuration_path && !shuttle_sim_load_planet(files.atmosphere_csv, &p, reason, sizeof(reason))) {
        fprintf(stderr, "configured fixture atmosphere unavailable: %s\n", reason);
        return EXIT_FAILURE;
    }
    GuidanceMachine g;
    memset(&g, 0, sizeof(g));
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;

    /* Downstream-feasibility contract.  The physical admission envelope is a
     * necessary lower bound: at Mach 2+ it cannot see the turn authority or the
     * energy excess the route needs, so a state inside it may have no flyable
     * MM305 route. These finite regression samples are not a proof of global
     * reachability or impossibility for either vehicle. Ownership transfers
     * only after the native search qualifies a route from the measured state,
     * and MM305 adopts that route. Properties:
     *   1. the handoff gate is true exactly when a route was found;
     *   2. a found route is adopted bumplessly as the MM305 route;
     *   3. a qualified route ages out (no handoff on a stale solution);
     *   4. this regression batch is not vacuous: at least one sample qualifies. */
    typedef struct { double alt, mach, along, cross; } AdmittedSample;
    const double mid_alt = 0.5 * (cfg.guidance.mm305_min_altitude + cfg.guidance.mm305_max_altitude);
    const AdmittedSample samples[] = {
        { mid_alt, cfg.guidance.mm305_target_mach, -30000.0, 0.0 },
        { cfg.guidance.mm305_min_altitude + 500.0, cfg.guidance.mm305_target_mach - 0.15, -28000.0, -5000.0 },
        { mid_alt, cfg.guidance.mm305_target_mach, -60000.0, 0.0 },
        { mid_alt, cfg.guidance.mm305_target_mach, -80000.0, 0.0 },
        { cfg.guidance.mm305_min_altitude + 1500.0, cfg.guidance.mm305_target_mach - 0.1, -70000.0, 8000.0 }
    };

    int total_admitted = 0, qualified = 0, contract_violations = 0;
    for (size_t i = 0; i < sizeof(samples)/sizeof(samples[0]); ++i) {
        Telemetry t = create_telemetry(&p, &cfg, samples[i].alt, samples[i].mach,
                                      samples[i].along, samples[i].cross, mass_kg);
        TerminalDynamicState state = telemetry_to_terminal_state(&model, &t, cfg.site.runway_heading);
        if (configuration_path) {
            /* A vehicle-specific experiment uses the same atmosphere, mass and
               forces as its native replay, not the legacy fake envelope values. */
            AeroForces forces = aero_compute(&model.world, &model.aero,
                state.position_i_m, state.velocity_i_mps, state.ut_s, state.mass_kg,
                state.attitude.aoa_rad, state.attitude.bank_rad);
            t.dynamic_pressure = forces.dynamic_pressure_pa;
            t.lift_force = forces.lift_n;
            t.drag_force = forces.drag_n;
            t.g_force = hypot(forces.lift_n, forces.drag_n) / (state.mass_kg * 9.80665);
            t.physics_sample_valid = true;
            t.ut = state.ut_s;
        }
        TaemInterfaceCapture capture = entry_mm305_admission_envelope(&g, &t, cfg.site.runway_heading, &p, &cfg);
        if (!capture.valid || !capture.ready) {
            printf("sample=%zu along=%.0f physically inadmissible (veto 0x%x)\n", i, t.runway_along_track, capture.veto);
            continue;
        }
        total_admitted++;

        GuidanceMachine gi = g;
        /* Before any route: never qualified, and a request is raised. */
        if (guidance_mm305_admission_qualified(&gi, &t, true, model.snapshot_id) || !gi.mm305_admission_needed) {
            fprintf(stderr, "sample=%zu: handoff allowed before a route was qualified\n", i);
            contract_violations++;
            continue;
        }

        Mm305PlanRequest req = {
            .valid = true,
            .request_ut = t.ut,
            .model_snapshot_id = model.snapshot_id,
            .state = state,
            .hac_radius_m = model.guidance.hac_radius,
            .search_both_ends = false,
            .upstream_end = 0,
            .lift_scale = 1.0,
            .drag_scale = 1.0
        };
        Mm305PlanResult result = mm305_plan(&model, &req);
        guidance_mm305_admission_accept(&gi, &result);
        bool gate = guidance_mm305_admission_qualified(&gi, &t, true, model.snapshot_id);
        printf("sample=%zu altitude=%.1f mach=%.3f along=%.1f cross=%.1f found=%d gate=%d solve_wall=%.3f\n",
            i, t.mean_altitude, t.mach, t.runway_along_track, t.runway_cross_track,
            result.found, gate, result.solve_wall_s);
        if (gate != (result.valid && result.found)) {
            fprintf(stderr, "sample=%zu: handoff gate %d disagrees with planner found=%d\n", i, gate, result.found);
            contract_violations++;
            continue;
        }
        if (!result.found) {
            printf("  rejected (MM304 retains ownership): %.300s\n", result.diagnostic);
            continue;
        }
        qualified++;

        Telemetry stale = t;
        stale.ut += MM305_ADMISSION_MAX_AGE_S + 1.0;
        GuidanceMachine aged = gi;
        if (guidance_mm305_admission_qualified(&aged, &stale, true, model.snapshot_id)) {
            fprintf(stderr, "sample=%zu: a stale qualified route still allowed handoff\n", i);
            contract_violations++;
        }
        if (guidance_mm305_admission_qualified(&aged, &t, true, model.snapshot_id + 1)) {
            fprintf(stderr, "sample=%zu: a route from another model snapshot allowed handoff\n", i);
            contract_violations++;
        }

        gi.phase = PHASE_TAEM;
        if (!guidance_mm305_adopt_admission_route(&gi, &cfg) || !gi.mm305_route_committed ||
            gi.mm305_route.length_m != result.candidate.route.length_m ||
            gi.hac_side != result.candidate.side) {
            fprintf(stderr, "sample=%zu: the qualified route was not adopted as the MM305 route\n", i);
            contract_violations++;
        }
    }

    printf("MM305 downstream-feasibility contract: %d physically admitted, %d qualified (handoff), %d held in MM304, %d violations.\n",
           total_admitted, qualified, total_admitted - qualified, contract_violations);
    assert(total_admitted > 0);
    if (contract_violations) {
        fprintf(stderr, "FAIL: MM305 downstream-feasibility contract violated %d time(s).\n", contract_violations);
        return EXIT_FAILURE;
    }
    if (qualified == 0) {
        fprintf(stderr, "FAIL: none of this fixture's tested states qualified an MM305 route; the non-vacuity check failed.\n");
        return EXIT_FAILURE;
    }
    puts("MM305 downstream-feasibility contract test passed.");
    return EXIT_SUCCESS;
}
