#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_telemetry.h"
#include "landing_api.h"
#include "taem_candidate_search.h"
#include "taem_alignment.h"
#include "taem_geometry.h"
#include "taem_reachability.h"
#include "terminal_solver.h"
#include "mm305_planning.h"
#include "shuttlesim/math3.h"
#include "taem_planner.h"
#include "guidance_internal.h"

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

/* The gamma equation resolves lift normal to the velocity, not radially up.
 * Reconstruct the delivered force independently at several descent angles. */
static void test_tracker_normal_lift_balance(const TerminalModel *model,
        const TerminalDynamicState *seed) {
    const double to_radians = acos(-1.0) / 180.0;
    const double path_angles[] = {-35.0, -20.0, 0.0, 20.0};
    for (size_t i=0;i<sizeof(path_angles)/sizeof(path_angles[0]);++i) {
        TerminalDynamicState sample=*seed;
        sample.position_i_m=world_lla_to_inertial(&model->world,
            model->site.latitude*to_radians,model->site.longitude*to_radians,
            4000.0,sample.ut_s);
        LocalFrame frame=world_local_frame_i(&model->world,sample.position_i_m,sample.ut_s);
        double gamma=path_angles[i]*to_radians;
        Vec3 air_velocity=v3_add(v3_scale(frame.north,180.0*cos(gamma)),
                                v3_scale(frame.up,180.0*sin(gamma)));
        sample.velocity_i_mps=v3_add(air_velocity,
            world_atmosphere_velocity_i(&model->world,sample.position_i_m));
        sample.attitude.requested_aoa_rad=NAN;
        sample.attitude.aoa_rad=NAN;
        sample.attitude.requested_bank_rad=NAN;
        sample.attitude.bank_rad=NAN;
        TaemGeometryState geometry;
        assert(taem_geometry_state(model,&sample,&geometry));
        TaemPathReference reference={
            .runway_along_m=geometry.runway_along_m,
            .runway_cross_m=geometry.runway_cross_m,
            .course_deg=geometry.course_deg,
            .altitude_m=model->site.altitude+geometry.altitude_above_runway_m,
            .flight_path_angle_deg=geometry.flight_path_angle_deg
        };
        TaemTrackerOutput demand=taem_tracker_update(model,&sample,&geometry,&reference,0.1);
        assert(demand.valid&&demand.lateral_authority_ok);
        AeroForces force=aero_compute(&model->world,&model->aero,
            sample.position_i_m,sample.velocity_i_mps,sample.ut_s,sample.mass_kg,
            demand.control.angle_of_attack_rad,demand.control.bank_rad);
        double normal_lift=force.lift_n/sample.mass_kg*cos(demand.control.bank_rad);
        double residual=normal_lift-demand.required_vertical_lift_mps2;
        fprintf(stderr,"normal lift balance: gamma=%+.0f required=%.6f delivered=%.6f residual=%+.6f\n",
            path_angles[i],demand.required_vertical_lift_mps2,normal_lift,residual);
        assert(fabs(residual)<0.25);
        assert(fabs(normal_lift-demand.delivered_vertical_lift_mps2)<1e-9);
    }
}

static void test_tracker_bank_target_continuity(const TerminalModel *model,
        const TerminalDynamicState *initial) {
    TerminalDynamicState state = *initial;
    TaemGeometryState geometry;
    assert(taem_geometry_state(model, &state, &geometry));
    TaemPathReference reference = {
        .runway_along_m = geometry.runway_along_m,
        .runway_cross_m = geometry.runway_cross_m,
        .course_deg = geometry.course_deg,
        .curvature_right_per_m = 1.0 / 2000.0,
        .altitude_m = model->site.altitude + geometry.altitude_above_runway_m,
        .flight_path_angle_deg = geometry.flight_path_angle_deg
    };
    const double dt = 0.1;
    const double max_step = 5.0 * model->attitude.max_roll_rate_rad_s * dt;
    state.attitude.requested_bank_rad = 0.0;
    TaemTrackerOutput right = taem_tracker_update(model, &state, &geometry,
                                                   &reference, dt);
    assert(right.valid && right.control.bank_rad > 0.0);
    assert(right.control.bank_rad <= max_step + 1e-9);
    state.attitude.requested_bank_rad = 70.0 * 3.14159265358979323846 / 180.0;
    reference.curvature_right_per_m = -reference.curvature_right_per_m;
    TaemTrackerOutput reversal = taem_tracker_update(model, &state, &geometry,
                                                      &reference, dt);
    assert(reversal.valid);
    assert(state.attitude.requested_bank_rad - reversal.control.bank_rad <=
           max_step + 1e-9);
    assert(reversal.control.angle_of_attack_rad <=
           model->vehicle.maximum_angle_of_attack *
           3.14159265358979323846 / 180.0 + 1e-9);
}
static TerminalDynamicState live_like_mm305_state(const TerminalModel *model) {
    TaemFrameWorld world;
    TaemRunwayFrame runway;
    assert(taem_geometry_runway(model, &world, &runway));
    TaemVec3 position_b, north, east, up;
    assert(taem_runway_unproject(&world, &runway, -46841.0, 10011.0,
                                 21052.0, &position_b));
    assert(taem_local_north_east_up(position_b, &north, &east, &up));
    const double course = 79.7 * DEG2RAD;
    const double fpa = -14.5 * DEG2RAD;
    const double speed = 761.8;
    const double horizontal = speed * cos(fpa);
    TaemVec3 velocity_b = taem_vec3_add(
        taem_vec3_add(taem_vec3_scale(north, horizontal * cos(course)),
                      taem_vec3_scale(east, horizontal * sin(course))),
        taem_vec3_scale(up, speed * sin(fpa)));
    TaemVec3 position_i, velocity_i;
    assert(taem_fixed_to_inertial(&world, position_b, velocity_b, 21901.45,
                                  &position_i, &velocity_i));
    TerminalDynamicState state = {0};
    state.position_i_m = (Vec3){position_i.x, position_i.y, position_i.z};
    state.velocity_i_mps = (Vec3){velocity_i.x, velocity_i.y, velocity_i.z};
    state.mass_kg = 43515.769531;
    state.ut_s = 21901.45;
    state.attitude = model->attitude;
    state.attitude.aoa_rad = 3.4 * DEG2RAD;
    state.attitude.bank_rad = 2.1 * DEG2RAD;
    state.attitude.aoa_rate_rad_s = 0.0;
    state.attitude.bank_rate_rad_s = 0.0;
    state.attitude.cmd_aoa_rad = state.attitude.requested_aoa_rad = state.attitude.aoa_rad;
    state.attitude.cmd_bank_rad = state.attitude.requested_bank_rad = state.attitude.bank_rad;
    state.trim_aoa_ceiling_rad = 13.6 * DEG2RAD;
    return state;
}
static void test_mm305_native_family_matrix(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry);


static void test_mm305_acquisition_qualification_contract(
        const TerminalModel *model, const LandingConfiguration *configuration) {
    /* Reproduce the pinned configuration from the reference live run without
       changing global production defaults. */
    LandingConfiguration live_configuration=*configuration;
    live_configuration.guidance.final_approach_distance=3200.0;
    live_configuration.guidance.final_glide_slope=28.0;
    live_configuration.guidance.taem_glide_slope=24.0;
    live_configuration.guidance.minimum_planning_lead_time=90.0;
    live_configuration.guidance.use_time_warp=false;
    TerminalModel live_model=*model;
    live_model.guidance=live_configuration.guidance;
    configuration=&live_configuration;
    model=&live_model;
    TerminalDynamicState live = live_like_mm305_state(model);
    TaemGeometryState geometry;
    assert(terminal_state_validate(&live, NULL, 0));
    assert(taem_geometry_state(model, &live, &geometry));
    assert(fabs(geometry.runway_along_m + 46841.0) < 2.0);
    assert(fabs(geometry.runway_cross_m - 10011.0) < 2.0);
    assert(fabs(geometry.altitude_above_runway_m - 21052.0) < 2.0);
    assert(fabs(geometry.airspeed_mps - 761.8) < 0.5);
    assert(fabs(geometry.course_deg - 79.7) < 0.1);
    assert(fabs(geometry.flight_path_angle_deg + 14.5) < 0.1);

    VehicleState vehicle = {
        .ut = live.ut_s,
        .position = {live.position_i_m.x, live.position_i_m.y, live.position_i_m.z},
        .velocity = {live.velocity_i_mps.x, live.velocity_i_mps.y, live.velocity_i_mps.z},
        .mass = live.mass_kg
    };
    Telemetry telemetry = {0};
    telemetry.ut = live.ut_s;
    telemetry.mean_altitude = model->site.altitude + geometry.altitude_above_runway_m;
    telemetry.radar_altitude = geometry.altitude_above_runway_m;
    telemetry.true_air_speed = geometry.airspeed_mps;
    telemetry.surface_speed = geometry.ground_speed_mps;
    telemetry.horizontal_speed = geometry.ground_speed_mps * cos(geometry.flight_path_angle_deg * DEG2RAD);
    telemetry.vertical_speed = geometry.ground_speed_mps * sin(geometry.flight_path_angle_deg * DEG2RAD);
    telemetry.flight_path_angle = geometry.flight_path_angle_deg;
    telemetry.heading = telemetry.ground_track_heading = geometry.course_deg;
    telemetry.angle_of_attack = 3.4;
    telemetry.roll = 2.1;
    telemetry.mass = live.mass_kg;
    telemetry.mach = 2.545;
    telemetry.speed_of_sound = geometry.airspeed_mps / telemetry.mach;

    const double trim_intercept = 0.75 - 0.047 * 13.6;
    GuidanceMachine acquisition;
    guidance_machine_init(&acquisition);
    acquisition.phase = PHASE_TAEM;
    acquisition.mm305_async_planning = true;
    acquisition.mm305_lift_scale = 0.5;
    acquisition.mm305_drag_scale = 0.5;
    acquisition.hac_radius = model->guidance.hac_radius;
    acquisition.mm305_trim_initialized = true;
    acquisition.mm305_trim_intercept = trim_intercept;

    PlanetModel planet = {0};
    planet.radius = model->world.radius_m;
    planet.gravitational_parameter = model->world.mu_m3_s2;
    planet.rotational_speed = model->world.rotation_rate_rad_s;
    AerodynamicModel aero = {
        .lift_to_drag = configuration->vehicle.estimated_lift_to_drag,
        .ballistic_coefficient = configuration->vehicle.estimated_ballistic_coefficient,
        .confidence = 1.0
    };
    GuidanceResult acquisition_result = taem_guidance_native(&acquisition, &telemetry,
        &vehicle, geometry.course_deg, &planet, aero, configuration, model, 0.1);
    (void)acquisition_result;
    assert(acquisition.mm305_acquisition_route_valid);
    assert(!acquisition.terminal_final_handoff_latched);

    GuidanceMachine qualifier;
    guidance_machine_init(&qualifier);
    qualifier.phase = PHASE_TAEM;
    qualifier.mm305_planning_needed = true;
    qualifier.mm305_lift_scale = 0.5;
    qualifier.mm305_drag_scale = 0.5;
    qualifier.mm305_trim_initialized = true;
    qualifier.mm305_trim_intercept = trim_intercept;
    /* Model the live async handoff: native qualification receives the exact
     * provisional route already being tracked, rather than searching as if it
     * had no seed. */
    qualifier.mm305_acquisition_route=acquisition.mm305_acquisition_route;
    qualifier.mm305_acquisition_route_valid=true;
    Mm305PlanRequest request = guidance_mm305_plan_request(&qualifier, &telemetry,
        &vehicle, configuration, model);
    assert(request.valid);
    double resolved_distance = mm305_final_alignment_distance(&qualifier, model,
        &request.state);
    assert(fabs(request.final_approach_distance_m - resolved_distance) < 1e-9);
    assert(fabs(acquisition.mm305_acquisition_route.alignment_along_m +
                resolved_distance) < 1.0);

    TerminalModel planning_model;
    mm305_prepare_planning_model(&planning_model,model,resolved_distance,
        request.scale_mach,request.lift_scale,request.drag_scale,
        request.state.trim_aoa_ceiling_rad);
    /* Acquisition and qualifier must agree on the complete Final interface,
     * not just its distance: target altitude is capped consistently, while
     * slope and alignment speed are resolved from the same unmodified base. */
    TerminalModel expected_contract;
    mm305_apply_final_alignment(&expected_contract,model,resolved_distance);
    assert(fabs(planning_model.guidance.final_approach_distance -
        expected_contract.guidance.final_approach_distance) < 1e-9);
    assert(fabs(planning_model.guidance.final_glide_slope -
        expected_contract.guidance.final_glide_slope) < 1e-9);
    assert(fabs(planning_model.guidance.final_alignment_speed -
        expected_contract.guidance.final_alignment_speed) < 1e-9);
    double expected_handoff_height=fmin(resolved_distance *
        tan(model->guidance.final_glide_slope * DEG2RAD),4000.0);
    double applied_handoff_height=planning_model.guidance.final_approach_distance *
        tan(planning_model.guidance.final_glide_slope * DEG2RAD);
    assert(fabs(applied_handoff_height-expected_handoff_height) < 1e-6);
    TerminalProfileResult acquisition_profile = terminal_solver_generate_profile(
        &planning_model, &request.state, &acquisition.mm305_acquisition_route,
        0.75, 420.0);
    Mm305PlanResult qualified = mm305_plan(model, &request);
    TaemFixedHacCandidate acquisition_replay=taem_fixed_hac_evaluate_route(
        &planning_model,&request.state,&acquisition.mm305_acquisition_route,
        0,0.5,420.0);

    fprintf(stderr,
        "MM305 contract smoke: final=%.0f slope=%.2f speed=%.1f acq R=%.0f sweep=%.1f lead=%.0f arc=%.0f path=%.0f profile=%d exitV=%.1f replayStatus=%d replayExitV=%.1f reason=%s",
        resolved_distance, planning_model.guidance.final_glide_slope,
        planning_model.guidance.final_alignment_speed,
        acquisition.mm305_acquisition_route.hac.radius_m,
        acquisition.mm305_acquisition_route.hac.arc_sweep_rad * RAD2DEG,
        acquisition.mm305_acquisition_route.lead_length_m,
        acquisition.mm305_acquisition_route.hac.arc_length_m,
        acquisition.mm305_acquisition_route.length_m,
        acquisition_profile.valid ? 1 : 0, acquisition_profile.exit_speed_mps,
        (int)acquisition_replay.status,
        acquisition_replay.replay.final_geometry.airspeed_mps,
        acquisition_replay.reason ? acquisition_replay.reason : "none");
    if (qualified.found) {
        TaemRoute qualified_profile_route=qualified.candidate.route;
        TerminalProfileResult qualified_profile=terminal_solver_generate_profile(
            &planning_model,&request.state,&qualified_profile_route,0.75,420.0);
        fprintf(stderr,
            " qualifier R=%.0f sweep=%.1f lead=%.0f arc=%.0f path=%.0f coarseExitV=%.1f replayExitV=%.1f\n",
            qualified.candidate.route.hac.radius_m,
            qualified.candidate.route.hac.arc_sweep_rad * RAD2DEG,
            qualified.candidate.route.lead_length_m,
            qualified.candidate.route.hac.arc_length_m,
            qualified.candidate.route.length_m,
            qualified_profile.exit_speed_mps,
            qualified.candidate.replay.final_geometry.airspeed_mps);
        assert(fabs(qualified.candidate.route.alignment_along_m + resolved_distance) < 1.0);
    } else {
        fprintf(stderr, " qualifier rejected: %s\n", qualified.diagnostic);
    }
    assert(qualified.found);
    assert(fabs(qualified.candidate.route.hac.radius_m -
                acquisition.mm305_acquisition_route.hac.radius_m) < 1.0);
    assert(fabs(qualified.candidate.route.hac.arc_sweep_rad -
                acquisition.mm305_acquisition_route.hac.arc_sweep_rad) < 1e-9);
    assert(fabs(qualified.candidate.route.lead_length_m -
                acquisition.mm305_acquisition_route.lead_length_m) < 1.0);
    assert(fabs(qualified.candidate.route.length_m -
                acquisition.mm305_acquisition_route.length_m) < 1.0);
    /* Prove convergence independently of the live seed-promotion fast path:
     * acquisition must agree with the qualifier's own search from this same
     * physical state, not merely pass its provisional route back as a seed. */
    Mm305PlanRequest independent_request=request;
    independent_request.seed_route_valid=false;
    Mm305PlanResult independent_qualified=mm305_plan(model,&independent_request);
    if (independent_qualified.found) {
        fprintf(stderr,"MM305 independent search: acq R=%.0f sweep=%.1f lead=%.0f path=%.0f vs qualifier R=%.0f sweep=%.1f lead=%.0f path=%.0f\\n",
            acquisition.mm305_acquisition_route.hac.radius_m,
            acquisition.mm305_acquisition_route.hac.arc_sweep_rad*RAD2DEG,
            acquisition.mm305_acquisition_route.lead_length_m,
            acquisition.mm305_acquisition_route.length_m,
            independent_qualified.candidate.route.hac.radius_m,
            independent_qualified.candidate.route.hac.arc_sweep_rad*RAD2DEG,
            independent_qualified.candidate.route.lead_length_m,
            independent_qualified.candidate.route.length_m);
    } else {
        fprintf(stderr,"MM305 independent search rejected: %s\\n",independent_qualified.diagnostic);
    }
    assert(independent_qualified.found);
    assert(fabs(independent_qualified.candidate.route.alignment_along_m +
        resolved_distance)<1.0);
    TaemGeometryState family_geometry;
    assert(taem_geometry_state(&planning_model, &request.state, &family_geometry));
    test_mm305_native_family_matrix(&planning_model, &request.state, &family_geometry);
    assert(!qualifier.terminal_final_handoff_latched);
}

static void test_mm305_native_family_matrix(const TerminalModel *model,
        const TerminalDynamicState *state, const TaemGeometryState *geometry) {
    const double radii_m[] = {3000.0, 4300.0, 7200.0, 12000.0, 18000.0};
    const double sweeps_deg[] = {15.0, 30.0, 60.0, 120.0,
                                 180.0, 195.0, 240.0, 270.0};
    TaemReachability reachability = {0};
    assert(taem_fixed_hac_turn_reachability(model, state, geometry,
        7200.0, &reachability));
    double curvature = 0.95 * reachability.available_lateral_accel_mps2 /
        fmax(geometry->ground_speed_mps * geometry->ground_speed_mps, 1.0);
    fprintf(stderr, "MM305 native family matrix (Final=%.0f slope=%.2f speed=%.1f):\n",
        model->guidance.final_approach_distance,
        model->guidance.final_glide_slope,
        model->guidance.final_alignment_speed);
    TaemFixedHacCandidate radius12_sweep30 = {0};
    TaemFixedHacCandidate radius18_sweep15 = {0};
    for (size_t ri = 0; ri < sizeof(radii_m) / sizeof(radii_m[0]); ++ri) {
        for (size_t si = 0; si < sizeof(sweeps_deg) / sizeof(sweeps_deg[0]); ++si) {
            TaemRoute route = {0};
            char reason[160] = {0};
            double sweep = sweeps_deg[si] * DEG2RAD;
            if (!taem_route_build_hac(model, geometry, radii_m[ri],
                    geometry->runway_cross_m >= 0.0 ? 1.0 : -1.0,
                    sweep, 420.0, curvature, &route, reason, sizeof(reason))) {
                fprintf(stderr, " family R=%.0f sweep=%.0f build=reject reason=%s\n",
                    radii_m[ri], sweeps_deg[si], reason[0] ? reason : "unavailable");
                continue;
            }
            TaemFixedHacCandidate replay = {0};
            if (sweeps_deg[si] <= 30.0)
                replay = taem_fixed_hac_evaluate_route(model, state, &route,
                    0, 0.5, 420.0);
            TerminalProfileResult profile = terminal_solver_generate_profile(
                model, state, &route, 0.75, 420.0);
            fprintf(stderr,
                " family R=%.0f sweep=%.0f lead=%.0f arc=%.0f path=%.0f profile=%d profileV=%.1f replayStatus=%d replayV=%.1f quality=%.3f profileReason=%s replayReason=%s\n",
                radii_m[ri], sweeps_deg[si], route.lead_length_m,
                route.hac.arc_length_m, route.length_m, profile.valid ? 1 : 0,
                profile.exit_speed_mps, (int)replay.status,
                replay.replay.final_geometry.airspeed_mps, replay.quality_score,
                profile.reason ? profile.reason : "none",
                replay.reason ? replay.reason : "not-run");
            if (fabs(radii_m[ri] - 12000.0) < 1.0 &&
                    fabs(sweeps_deg[si] - 30.0) < 1e-9)
                radius12_sweep30 = replay;
            if (fabs(radii_m[ri] - 18000.0) < 1.0 &&
                    fabs(sweeps_deg[si] - 15.0) < 1e-9)
                radius18_sweep15 = replay;
        }
    }
    assert(radius12_sweep30.status == TAEM_PLAN_UNQUALIFIED);
    assert(radius18_sweep15.status == TAEM_PLAN_UNQUALIFIED);
    assert(taem_fixed_hac_candidate_preferred(model, &radius12_sweep30,
        &radius18_sweep15));
    assert(radius12_sweep30.route.hac.arc_length_m >
           radius18_sweep15.route.hac.arc_length_m);
    assert(radius12_sweep30.route.lead_length_m /
               radius12_sweep30.route.hac.arc_length_m <
           radius18_sweep15.route.lead_length_m /
               radius18_sweep15.route.hac.arc_length_m);
    fprintf(stderr,
        "MM305 similar-energy circle ranking: R12k/30 lead=%.0f arc=%.0f exitV=%.1f quality=%.3f beats R18k/15 lead=%.0f arc=%.0f exitV=%.1f quality=%.3f\n",
        radius12_sweep30.route.lead_length_m,
        radius12_sweep30.route.hac.arc_length_m,
        radius12_sweep30.replay.final_geometry.airspeed_mps,
        radius12_sweep30.quality_score,
        radius18_sweep15.route.lead_length_m,
        radius18_sweep15.route.hac.arc_length_m,
        radius18_sweep15.replay.final_geometry.airspeed_mps,
        radius18_sweep15.quality_score);
}

int main(void) {
    LandingConfiguration configuration = landing_configuration_default();
    char atmosphere[512], aero[512], book[512], attitude[512];
    TerminalModelSourceFiles files = {
        .atmosphere_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATMOSPHERE,
                                                 atmosphere, sizeof(atmosphere)),
        .aero_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_AERO, aero, sizeof(aero)),
        .aero_book_csv = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_FORCE_BOOK,
                                                book, sizeof(book)),
        .attitude_ini = shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATTITUDE,
                                               attitude, sizeof(attitude))
    };
    TerminalModel model;
    char reason[192];
    assert(terminal_model_capture(&model, &files, &configuration, 41,
                                  reason, sizeof(reason)));
    assert(model.replay_validated && model.snapshot_id == 41);
    test_mm305_acquisition_qualification_contract(&model, &configuration);
    TerminalModel invalid=model;
    double *positive_fields[]={&invalid.world.radius_m,&invalid.world.mu_m3_s2,
        &invalid.world.atmosphere_top_m,&invalid.aero.reference_area_m2,
        &invalid.attitude.pitch_wn,&invalid.attitude.roll_wn,
        &invalid.attitude.pitch_zeta,&invalid.attitude.roll_zeta,
        &invalid.attitude.max_pitch_rate_rad_s,&invalid.attitude.max_roll_rate_rad_s,
        &invalid.attitude.max_pitch_accel_rad_s2,&invalid.attitude.max_roll_accel_rad_s2,
        &invalid.vehicle.touchdown_speed,&invalid.vehicle.maximum_angle_of_attack,
        &invalid.vehicle.maximum_bank_angle,&invalid.guidance.hac_radius,
        &invalid.guidance.final_approach_distance,
        &invalid.site.runway_length,&invalid.site.runway_width};
    const double invalid_values[]={INFINITY,NAN,0.0,-1.0};
    for(size_t i=0;i<sizeof(positive_fields)/sizeof(positive_fields[0]);++i){
        double original=*positive_fields[i];
        for(size_t j=0;j<sizeof(invalid_values)/sizeof(invalid_values[0]);++j){
            *positive_fields[i]=invalid_values[j];
            assert(!terminal_model_validate(&invalid,reason,sizeof(reason)));
        }
        *positive_fields[i]=original;
    }
    assert(terminal_model_validate(&invalid,reason,sizeof(reason)));

    TerminalDynamicState state = load_fixture_state(&model);
    assert(terminal_state_validate(&state, reason, sizeof(reason)));
    TaemGeometryState geometry;
    assert(taem_geometry_state(&model, &state, &geometry));
    test_tracker_normal_lift_balance(&model,&state);
    test_tracker_bank_target_continuity(&model,&state);

    TaemReachability reachability;
    assert(taem_fixed_hac_turn_reachability(&model, &state, &geometry,
        model.guidance.hac_radius, &reachability));
    /* The recorded fixture altitude was chosen for the KSP-fitted plant. With a
     * different (e.g. tracked reference) plant, descend the same state along the
     * local vertical until the model's own lift can turn the frozen HAC, so the
     * structural checks below remain meaningful for any plant model. */
    for (int step = 0; step < 40 && reachability.valid &&
            !reachability.lateral_authority_ok; ++step) {
        double r = v3_norm(state.position_i_m);
        state.position_i_m = v3_scale(state.position_i_m, (r - 500.0) / r);
        assert(taem_geometry_state(&model, &state, &geometry));
        assert(taem_fixed_hac_turn_reachability(&model, &state, &geometry,
            model.guidance.hac_radius, &reachability));
    }
    fprintf(stderr, "fixture: %.0f m above runway, lateral %.3f/%.3f m/s^2\n",
        geometry.altitude_above_runway_m, reachability.required_lateral_accel_mps2,
        reachability.available_lateral_accel_mps2);
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
    assert(taem_route_build_fixed_hac(&model, &tangent_start,
        model.guidance.hac_radius, 1.0, 500.0,
        maximum_curvature, &route, reason, sizeof(reason)));
    assert(route.valid && route.count > 3 && route.count <= TAEM_ROUTE_MAX_POINTS);
    assert(route.lead_arc_fraction_lut[0] == 0.0f);
    assert(route.lead_arc_fraction_lut[TAEM_ROUTE_LEAD_LUT_POINTS - 1] == 1.0f);
    assert(first_lead_sample_distance(&route) <= 550.0);

    /* Initial vertical shaping must affect the tracker immediately. The sag
     * preserves start altitude/FPA but contributes negative vertical curvature,
     * so a low-density descent can steepen without pretending that the vehicle
     * can instantaneously hold its entry FPA. */
    size_t base_cursor = 0, sag_cursor = 0;
    TaemPathReference base_reference, sag_reference;
    assert(taem_route_reference(&route, &tangent_start, &base_cursor,
                                &base_reference, NULL));
    TaemRoute sag_route = route;
    sag_route.profile_initial_sag_m = -200.0;
    sag_route.profile_initial_sag_length_m = 5000.0;
    assert(taem_route_reference(&sag_route, &tangent_start, &sag_cursor,
                                &sag_reference, NULL));
    assert(fabs(sag_reference.flight_path_angle_deg -
                base_reference.flight_path_angle_deg) < 1e-9);
    assert(sag_reference.vertical_curvature_per_m <
           base_reference.vertical_curvature_per_m);
    TaemTrackerOutput base_vertical = taem_tracker_update(&model, &state,
        &tangent_start, &base_reference, 0.25);
    TaemTrackerOutput sag_vertical = taem_tracker_update(&model, &state,
        &tangent_start, &sag_reference, 0.25);
    assert(base_vertical.valid && sag_vertical.valid);
    assert(sag_vertical.required_vertical_lift_mps2 <
           base_vertical.required_vertical_lift_mps2);

    /* A slower roll actuator must soften the lateral correction for the same
     * cross-track displacement.  The controller must wait for a bank reversal
     * to take effect before asking for the opposite one. */
    TaemGeometryState displaced = geometry;
    displaced.runway_cross_m += 1000.0;
    TaemPathReference straight = {
        .runway_along_m = geometry.runway_along_m,
        .runway_cross_m = geometry.runway_cross_m,
        .course_deg = geometry.course_deg,
        .curvature_right_per_m = 0.0,
        .altitude_m = model.site.altitude + geometry.altitude_above_runway_m,
        .flight_path_angle_deg = geometry.flight_path_angle_deg
    };
    TaemTrackerOutput normal_roll = taem_tracker_update(&model, &state,
        &displaced, &straight, 0.25);
    TerminalModel slower_roll = model;
    slower_roll.attitude.max_roll_rate_rad_s *= 0.5;
    TaemTrackerOutput slow_roll = taem_tracker_update(&slower_roll, &state,
        &displaced, &straight, 0.25);
    assert(normal_roll.valid && slow_roll.valid);
    assert(slow_roll.response_time_s > normal_roll.response_time_s);
    assert(fabs(slow_roll.required_lateral_accel_mps2) <
           fabs(normal_roll.required_lateral_accel_mps2));

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
    TerminalDynamicState capped_state = state;
    capped_state.attitude.requested_aoa_rad = 3.0 * 3.14159265358979323846 / 180.0;
    capped_state.attitude.aoa_rad = 3.0 * 3.14159265358979323846 / 180.0;
    TaemTrackerOutput capped_demand = taem_tracker_update(&limited_model,
        &capped_state, &geometry, &high_lift_reference, 0.25);
    assert(capped_demand.valid);
    assert(fabs(capped_demand.control.angle_of_attack_rad *
        180.0 / 3.14159265358979323846 - 3.0) < 1e-9);
    /* Replay must reject the low-lift vehicle quickly: the saturated tracker cannot hold
       the route, so the replay leaves the MM305 tracking corridor. */
    TaemRoute authority_probe_route;
    assert(taem_route_build_fixed_hac(&limited_model, &geometry,
        limited_model.guidance.hac_radius, 1.0,
        500.0, 1.0, &authority_probe_route, reason, sizeof(reason)));
    TerminalSolverResult authority_probe = terminal_solver_replay(
        &limited_model, &state, &authority_probe_route, 0.25, 10.0);
    assert(authority_probe.status == TERMINAL_SOLVER_INFEASIBLE);
    assert(strcmp(authority_probe.reason,
        "candidate diverged from the MM305 tracking corridor") == 0);
    assert(authority_probe.elapsed_s <= 5.0);

    /* A fixed-size descriptor copies independently with GuidanceMachine state. */
    GuidanceMachine first = {0};
    first.mm305_route = route;
    first.mm305_route_committed = true;
    first.mm305_model_snapshot_id = model.snapshot_id;
    GuidanceMachine snapshot = first;
    assert(snapshot.mm305_route_committed);
    assert(snapshot.mm305_model_snapshot_id == first.mm305_model_snapshot_id);
    assert(memcmp(&snapshot.mm305_route, &first.mm305_route, sizeof(route)) == 0);
    assert(sizeof(TaemRoute) < 4096);

    /* The MM305 descriptor ends at the alignment station at the configured Final glide
     * height; the untouched Final contract must still admit the live state. */
    size_t exit_cursor = route.count - 1;
    TaemGeometryState at_exit = geometry;
    at_exit.runway_along_m = route.alignment_along_m;
    at_exit.runway_cross_m = 0.0;
    TaemPathReference exit_reference;
    size_t exit_index = 0;
    assert(taem_route_reference(&route, &at_exit, &exit_cursor,
        &exit_reference, &exit_index));
    double expected_exit_altitude = route.profile_final_altitude_m;
    assert(exit_index == route.count - 1);
    assert(fabs(exit_reference.altitude_m - expected_exit_altitude) < 1e-6);
    assert(fabs(exit_reference.flight_path_angle_deg +
                model.guidance.final_glide_slope) < 1e-6);
    assert(exit_reference.altitude_m > model.site.altitude);
    /* Alignment guidance intentionally carries no absolute-altitude
     * command, but solver bookkeeping must retain a finite explicit Final
     * target altitude. This guards the live failure where ready=1 was later
     * rejected because reference.altitude_m (NAN) leaked into path checks. */
    TaemGeometryState alignment_geometry = at_exit;
    alignment_geometry.runway_along_m = -model.guidance.final_approach_distance;
    alignment_geometry.runway_cross_m = 0.0;
    TaemPathReference alignment_reference =
        taem_alignment_reference(&model, &alignment_geometry);
    assert(isnan(alignment_reference.altitude_m));
    assert(isfinite(taem_alignment_target_height(&model)));
    assert(isfinite(taem_alignment_target_altitude(&model)));
    assert(fabs(taem_alignment_target_altitude(&model) -
        (model.site.altitude + taem_alignment_target_height(&model))) < 1e-9);
    /* Live MM305 replay showed alignment could begin only ~2.2 km before
     * Final, with the unpowered vehicle still needing bank/FPA settlement.
     * Keep enough capture horizon to settle, and do not declare the strict
     * miss while a steep, low-energy vehicle is still upstream of Final. */
    alignment_geometry.ground_speed_mps = 170.0;
    assert(fabs(taem_alignment_capture_distance(&model,
        &alignment_geometry) - 5100.0) < 1e-9);
    TaemGeometryState steep_alignment = alignment_geometry;
    steep_alignment.flight_path_angle_deg =
        -model.guidance.final_glide_slope - 8.0;
    steep_alignment.runway_along_m =
        -model.guidance.final_approach_distance - 100.0;
    assert(!taem_alignment_exhausted(&model, &steep_alignment));
    steep_alignment.runway_along_m =
        -model.guidance.final_approach_distance + 1.0;
    assert(taem_alignment_exhausted(&model, &steep_alignment));
    steep_alignment.flight_path_angle_deg =
        -model.guidance.final_glide_slope;
    steep_alignment.runway_along_m =
        -model.guidance.final_approach_distance + 501.0;
    assert(taem_alignment_exhausted(&model, &steep_alignment));


    /* Interior profile candidates preserve the live start and HAC-exit
     * altitude/FPA contracts while changing only the path between them. */
    TaemRoute shaped_route = route;
    shaped_route.profile_midpoint_offset_m = 1200.0;
    shaped_route.profile_local_offset_m = -500.0;
    shaped_route.profile_local_start_fraction = 0.12;
    shaped_route.profile_local_peak_fraction = 0.22;
    shaped_route.profile_local_end_fraction = 0.42;
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
    size_t middle_cursor = (size_t)(0.27 * shaped_route.count);
    size_t base_middle_cursor = middle_cursor;
    TaemPathReference local_middle, base_middle;
    size_t local_middle_index = SIZE_MAX, base_middle_index = SIZE_MAX;
    TaemRoute base_profile = shaped_route;
    base_profile.profile_local_offset_m = 0.0;
    assert(taem_route_reference(&shaped_route, &tangent_start, &middle_cursor,
        &local_middle, &local_middle_index));
    assert(taem_route_reference(&base_profile, &tangent_start, &base_middle_cursor,
        &base_middle, &base_middle_index));
    assert(local_middle_index == base_middle_index);
    assert(local_middle.altitude_m < base_middle.altitude_m);
    assert(local_middle.altitude_m >= base_middle.altitude_m - 500.0);

    /* Geometry alone cannot qualify a candidate: native replay must satisfy
     * all constraints through HAC exit before the existing Final-tail gate. */
    TaemFixedHacSearch search = taem_fixed_hac_search(&model, &state, model.guidance.hac_radius,
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
        &faster_state, model.guidance.hac_radius, 500.0, 0.25, 1800.0);
    for (size_t i = 0; i < 2; ++i)
        assert(strcmp(faster_search.candidates[i].reason,
            "fixed HAC circle exceeds live turn authority") != 0);
    /* Re-entrant MM305 planning: the pure planner reports rejection with a
     * diagnostic, acceptance of a rejection counts a failure and clears the
     * request, and acceptance of a found route commits it (a second acceptance
     * counts as a replan). */
    Mm305PlanRequest plan_request = {
        .valid = true, .request_ut = state.ut_s, .model_snapshot_id = model.snapshot_id,
        .state = state, .hac_radius_m = model.guidance.hac_radius,
        .search_both_ends = false, .upstream_end = 0,
        .lift_scale = 1.0, .drag_scale = 1.0
    };
    Mm305PlanResult rejected = mm305_plan(&model, &plan_request);
    assert(rejected.valid && !rejected.found);
    assert(strncmp(rejected.diagnostic, "MM305 route rejected", 20) == 0);
    LandingConfiguration plan_cfg = landing_configuration_default();
    GuidanceMachine planner_g = {0};
    planner_g.phase = PHASE_TAEM;
    planner_g.mm305_planning_needed = true;
    assert(!guidance_mm305_accept_plan(&planner_g, &rejected, &plan_cfg));
    assert(planner_g.mm305_plan_failures == 1 && !planner_g.mm305_planning_needed);
    assert(!planner_g.mm305_route_committed);
    Mm305PlanResult found = rejected;
    found.found = true;
    memset(&found.candidate, 0, sizeof(found.candidate));
    found.candidate.route = route;
    found.candidate.side = route.side;
    found.candidate.runway_end = 0;
    found.candidate.route_built = true;
    planner_g.mm305_planning_needed = true;
    assert(guidance_mm305_accept_plan(&planner_g, &found, &plan_cfg));
    assert(planner_g.mm305_route_committed && planner_g.mm305_route_cursor == 0);
    assert(planner_g.mm305_replans == 0 && planner_g.runway_end_committed);
    assert(planner_g.mm305_model_snapshot_id == model.snapshot_id);
    assert(guidance_mm305_accept_plan(&planner_g, &found, &plan_cfg));
    assert(planner_g.mm305_replans == 1);
    /* A failed later replan must leave the already committed route
     * untouched; live MM305 continues flying the last qualified route. */
    TaemRoute held_route = planner_g.mm305_route;
    planner_g.mm305_planning_needed = true;
    assert(!guidance_mm305_accept_plan(&planner_g, &rejected, &plan_cfg));
    assert(planner_g.mm305_route_committed);
    assert(memcmp(&planner_g.mm305_route, &held_route, sizeof(held_route)) == 0);

    /* A plan arriving after the HAC exit must not be adopted. */
    planner_g.mm305_hac_exit_reached = true;
    assert(!guidance_mm305_accept_plan(&planner_g, &found, &plan_cfg));

    puts("TAEM native model, route descriptor, Final interface, and replay gate tests passed.");
    return 0;
}
