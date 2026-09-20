#include "../CLanding/guidance.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static PlanetModel verifier_kerbin(void) {
    PlanetModel p;
    memset(&p, 0, sizeof(p));
    snprintf(p.name, sizeof(p.name), "Kerbin");
    p.radius = 600000.0;
    p.gravitational_parameter = 3.5316e12;
    p.rotational_speed = 2.0 * LANDER_PI / 21549.425;
    p.atmosphere_depth = 70000.0;
    p.surface_density = 1.225;
    p.atmosphere_adiabatic_index = 1.4;
    p.north_axis = v3(0, 0, 1);
    p.prime_meridian_at_epoch = v3(1, 0, 0);
    return p;
}

static Telemetry verifier_approach_telemetry(const LandingConfiguration *cfg) {
    Telemetry t;
    telemetry_init(&t);
    t.ut = 1000.0;
    t.mean_altitude = cfg->site.altitude + 300.0;
    t.radar_altitude = 300.0;
    t.true_air_speed = 140.0;
    t.horizontal_speed = 139.8;
    t.surface_speed = 140.0;
    t.vertical_speed = -7.0;
    t.flight_path_angle = atan2(t.vertical_speed, t.horizontal_speed) * RAD2DEG;
    t.heading = cfg->site.runway_heading;
    t.ground_track_heading = cfg->site.runway_heading;
    t.runway_along_track = -1000.0;
    t.runway_cross_track = 0.0;
    t.range_to_site = 1000.0;
    t.angle_of_attack = 10.0;
    t.roll = 0.0;
    t.pitch = t.flight_path_angle + t.angle_of_attack;
    t.mass = 40000.0;
    t.lift_force = 500000.0;
    t.drag_force = 80000.0;
    t.dynamic_pressure = 6000.0;
    t.bank_effectiveness = 1.0;
    t.g_force = 1.0;
    t.mach = t.true_air_speed / 320.0;
    t.stall_fraction = 0.0;
    snprintf(t.vessel_situation, sizeof(t.vessel_situation), "flying");
    return t;
}

static GuidanceMachine verifier_machine(void) {
    GuidanceMachine g;
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    g.final_approach_captured = true;
    g.terminal_region_entered = true;
    g.terminal_sink_accel_ema = 1.0;
    g.terminal_positive_aoa_rate_ema = 2.0;
    g.terminal_pitch_response_delay_ema = 1.0;
    return g;
}

static void seed_preflare_plan(GuidanceMachine *g) {
    g->terminal_preflare_plan_valid = true;
    g->preflare_trigger_altitude = 600.0;
    g->preflare_target_aoa = 16.0;
    g->preflare_target_sink = -8.0;
    g->preflare_minimum_speed = 100.0;
    g->preflare_reference_speed = 130.0;
    g->preflare_predicted_height_loss = 120.0;
    g->preflare_predicted_kinetic_margin = 2000.0;
    g->preflare_effective_accel = 1.0;
}

static void test_terminal_stage_is_monotonic(void) {
    GuidanceMachine g = verifier_machine();
    terminal_set_stage(&g, TERMINAL_TRAJECTORY_CAPTURE, 1.0);
    terminal_set_stage(&g, TERMINAL_OUTER_FINAL, 2.0);
    terminal_set_stage(&g, TERMINAL_PREFLARE, 3.0);
    terminal_set_stage(&g, TERMINAL_OUTER_FINAL, 4.0);
    assert(g.terminal_vertical_stage == TERMINAL_PREFLARE);
    terminal_set_stage(&g, TERMINAL_INNER_FINAL, 5.0);
    terminal_set_stage(&g, TERMINAL_TOUCHDOWN_FLARE, 6.0);
    terminal_set_stage(&g, TERMINAL_INNER_FINAL, 7.0);
    assert(g.terminal_vertical_stage == TERMINAL_TOUCHDOWN_FLARE);
}

static void test_shallow_glide_cannot_be_bypassed(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700.0, .confidence = .8};
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    Trajectory ref;
    trajectory_init(&ref);

    terminal_set_stage(&g, TERMINAL_PREFLARE, t.ut);
    seed_preflare_plan(&g);

    /* Preflare sink capture alone must not advance without the strongest gear
       state the current telemetry exposes. The command latch is only intent. */
    for (int n = 0; n < 8; ++n) {
        t.ut += .1;
        GuidanceResult r = terminal_approach_sequence(&g, &t, cfg.site.runway_heading,
                                                       &p, aero, &cfg, &ref, .1);
        guidance_result_clear(&r);
    }
    assert(g.terminal_vertical_stage == TERMINAL_PREFLARE);

    t.gear = true;
    for (int n = 0; n < 8; ++n) {
        t.ut += .1;
        GuidanceResult r = terminal_approach_sequence(&g, &t, cfg.site.runway_heading,
                                                       &p, aero, &cfg, &ref, .1);
        guidance_result_clear(&r);
    }
    assert(g.terminal_vertical_stage == TERMINAL_INNER_FINAL);

    /* Attempting to send the machine back to steep/outer final is rejected. */
    terminal_set_stage(&g, TERMINAL_OUTER_FINAL, t.ut + .1);
    assert(g.terminal_vertical_stage == TERMINAL_INNER_FINAL);

    t.ut += .1;
    t.radar_altitude = 50.0;
    t.mean_altitude = cfg.site.altitude + t.radar_altitude;
    t.true_air_speed = 120.0;
    t.horizontal_speed = 119.9;
    t.surface_speed = 120.0;
    t.vertical_speed = -4.0;
    t.flight_path_angle = atan2(t.vertical_speed, t.horizontal_speed) * RAD2DEG;
    GuidanceResult flare = terminal_approach_sequence(&g, &t, cfg.site.runway_heading,
                                                       &p, aero, &cfg, &ref, .1);
    assert(g.terminal_vertical_stage == TERMINAL_TOUCHDOWN_FLARE);
    assert(flare.phase == PHASE_FLARE);
    guidance_result_clear(&flare);

    t.ut += .1;
    t.radar_altitude = 0.0;
    t.mean_altitude = cfg.site.altitude;
    t.surface_speed = 0.0;
    t.true_air_speed = 0.0;
    t.horizontal_speed = 0.0;
    t.vertical_speed = 0.0;
    t.runway_along_track = 500.0;
    t.runway_cross_track = 0.0;
    t.gear = true;
    snprintf(t.vessel_situation, sizeof(t.vessel_situation), "landed");
    GuidanceResult ground = terminal_approach_sequence(&g, &t, cfg.site.runway_heading,
                                                        &p, aero, &cfg, &ref, .1);
    assert(g.terminal_vertical_stage == TERMINAL_GROUND);
    assert(ground.phase == PHASE_COMPLETE || ground.phase == PHASE_TOUCHDOWN || ground.phase == PHASE_ROLLOUT);
    guidance_result_clear(&ground);
    trajectory_clear(&ref);
}

static void test_off_runway_landed_state_is_not_runway_success(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700.0, .confidence = .8};
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    VehicleState state;
    memset(&state, 0, sizeof(state));

    terminal_set_stage(&g, TERMINAL_TOUCHDOWN_FLARE, t.ut);
    t.radar_altitude = 0.0;
    t.mean_altitude = cfg.site.altitude;
    t.vertical_speed = 0.0;
    t.horizontal_speed = 70.0;
    t.surface_speed = 70.0;
    t.true_air_speed = 70.0;
    t.runway_along_track = 600.0;
    t.runway_cross_track = cfg.site.runway_width * 2.0;
    t.gear = true;
    snprintf(t.vessel_situation, sizeof(t.vessel_situation), "landed");

    GuidanceResult r = terminal_guidance(&g, &t, &state, cfg.site.runway_heading,
                                         &p, aero, &cfg, .1);
    assert(r.phase == PHASE_ABORT);
    assert(g.aborted);
    assert(!g.ground_contact_latched);
    assert(strstr(r.status, "outside the runway 09") != NULL);
    guidance_result_clear(&r);
}

static void test_on_runway_landed_without_gear_is_not_success(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700.0, .confidence = .8};
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    VehicleState state;
    memset(&state, 0, sizeof(state));

    terminal_set_stage(&g, TERMINAL_TOUCHDOWN_FLARE, t.ut);
    t.radar_altitude = 0.0;
    t.mean_altitude = cfg.site.altitude;
    t.vertical_speed = 0.0;
    t.horizontal_speed = 70.0;
    t.surface_speed = 70.0;
    t.true_air_speed = 70.0;
    t.runway_along_track = 600.0;
    t.runway_cross_track = 0.0;
    t.gear = false;
    snprintf(t.vessel_situation, sizeof(t.vessel_situation), "landed");

    GuidanceResult r = terminal_guidance(&g, &t, &state, cfg.site.runway_heading,
                                         &p, aero, &cfg, .1);
    assert(r.phase == PHASE_ABORT);
    assert(g.aborted);
    assert(!g.ground_contact_latched);
    assert(strstr(r.status, "landing-gear") != NULL);
    guidance_result_clear(&r);
}

static void test_runway_contact_debounces_without_landed_string(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700.0, .confidence = .8};
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    Trajectory ref;
    trajectory_init(&ref);

    terminal_set_stage(&g, TERMINAL_TOUCHDOWN_FLARE, t.ut);
    t.radar_altitude = .25;
    t.mean_altitude = cfg.site.altitude + t.radar_altitude;
    t.vertical_speed = -1.2;
    t.horizontal_speed = 88.0;
    t.surface_speed = 88.0;
    t.true_air_speed = 88.0;
    t.flight_path_angle = atan2(t.vertical_speed, t.horizontal_speed) * RAD2DEG;
    t.runway_along_track = 500.0;
    t.runway_cross_track = 8.0;
    t.gear = true;
    snprintf(t.vessel_situation, sizeof(t.vessel_situation), "flying");

    for (int n = 0; n < 2; ++n) {
        t.ut += .1;
        GuidanceResult r = terminal_approach_sequence(&g, &t, cfg.site.runway_heading,
                                                       &p, aero, &cfg, &ref, .1);
        assert(!g.ground_contact_latched);
        assert(r.phase == PHASE_FLARE);
        guidance_result_clear(&r);
    }

    t.ut += .1;
    GuidanceResult contact = terminal_approach_sequence(&g, &t, cfg.site.runway_heading,
                                                         &p, aero, &cfg, &ref, .1);
    assert(g.ground_contact_latched);
    assert(g.terminal_vertical_stage == TERMINAL_GROUND);
    assert(contact.phase == PHASE_TOUCHDOWN || contact.phase == PHASE_ROLLOUT);
    guidance_result_clear(&contact);
    trajectory_clear(&ref);
}

static void test_rollout_steering_damps_cross_track_rate(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    Trajectory ref;
    trajectory_init(&ref);

    g.phase = PHASE_ROLLOUT;
    g.ground_contact_latched = true;
    t.runway_along_track = 700.0;
    t.runway_cross_track = 20.0;
    t.heading = cfg.site.runway_heading;
    t.surface_speed = 80.0;
    t.true_air_speed = 80.0;
    t.horizontal_speed = 80.0;

    t.ground_track_heading = cfg.site.runway_heading + 6.0;
    GuidanceResult outward = touchdown(&g, &t, &cfg, &ref);
    double outward_steer = outward.command.wheel_steering;
    guidance_result_clear(&outward);

    g.phase = PHASE_ROLLOUT;
    t.ground_track_heading = cfg.site.runway_heading - 6.0;
    GuidanceResult inward = touchdown(&g, &t, &cfg, &ref);
    double inward_steer = inward.command.wheel_steering;
    guidance_result_clear(&inward);

    assert(outward_steer < 0.0);
    assert(inward_steer > 0.0);
    assert(outward_steer < inward_steer);
    trajectory_clear(&ref);
}

static void test_rollout_cannot_complete_after_stopping_off_runway(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    Trajectory ref;
    trajectory_init(&ref);

    g.phase = PHASE_ROLLOUT;
    g.ground_contact_latched = true;
    terminal_set_stage(&g, TERMINAL_GROUND, t.ut);
    t.runway_along_track = 700.0;
    t.runway_cross_track = cfg.site.runway_width * 2.0;
    t.heading = cfg.site.runway_heading;
    t.ground_track_heading = cfg.site.runway_heading;
    t.surface_speed = 0.5;
    t.true_air_speed = 0.5;
    t.horizontal_speed = 0.5;
    snprintf(t.vessel_situation, sizeof(t.vessel_situation), "flying");

    GuidanceResult r = touchdown(&g, &t, &cfg, &ref);
    assert(r.phase == PHASE_ABORT);
    assert(g.aborted);
    assert(strstr(r.status, "Rollout stopped outside") != NULL);
    guidance_result_clear(&r);
    trajectory_clear(&ref);
}

static void test_preflare_respects_weak_authority_and_slow_response(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700, .confidence = .8};
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    t.drag_force = 1000;
    t.vertical_speed = -20;
    t.flight_path_angle = atan2(t.vertical_speed, t.horizontal_speed) * RAD2DEG;
    g.terminal_sink_accel_ema = .2;
    TerminalPreflarePlan weak = terminal_preflare_plan(&g, &t, &p, aero, &cfg);
    assert(weak.feasible);
    /* The learned cap is .2 * 1.10, then conservatively derated, never raised. */
    assert(weak.effective_accel <= .22);
    g.terminal_sink_accel_ema = 1;
    TerminalPreflarePlan strong = terminal_preflare_plan(&g, &t, &p, aero, &cfg);
    assert(strong.feasible);
    assert(weak.trigger_altitude > strong.trigger_altitude);
    g.terminal_positive_aoa_rate_ema = .25;
    TerminalPreflarePlan slow = terminal_preflare_plan(&g, &t, &p, aero, &cfg);
    assert(slow.feasible);
    assert(slow.response_aoa_rate <= .25);
    assert(slow.trigger_altitude > strong.trigger_altitude);
}

static void test_preflare_contract_ignores_preview_hints(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700, .confidence = .8};
    GuidanceMachine baseline_machine = verifier_machine();
    GuidanceMachine perturbed_machine = baseline_machine;
    Telemetry t = verifier_approach_telemetry(&cfg);
    ControlAuthorityEnvelope authority =
        decision_control_authority_envelope(&baseline_machine, &t, &p, &cfg);
    assert(authority.valid && authority.controllable);
    t.angle_of_attack = authority.maximum_lift_aoa_deg;
    t.pitch = t.flight_path_angle + t.angle_of_attack;

    baseline_machine.terminal_glide_mode = true;
    perturbed_machine.terminal_glide_mode = true;
    baseline_machine.terminal_test_preflare_min_speed = 0.0;
    baseline_machine.terminal_test_preflare_target_speed = 0.0;
    perturbed_machine.terminal_test_preflare_min_speed = t.true_air_speed * 4.0;
    perturbed_machine.terminal_test_preflare_target_speed = t.true_air_speed * 5.0;

    TerminalPreflarePlan baseline =
        terminal_preflare_plan(&baseline_machine, &t, &p, aero, &cfg);
    TerminalPreflarePlan perturbed =
        terminal_preflare_plan(&perturbed_machine, &t, &p, aero, &cfg);

    assert(baseline.feasible);
    assert(perturbed.feasible);
    assert(fabs(perturbed.trigger_altitude - baseline.trigger_altitude) <=
        sqrt(DBL_EPSILON) * fmax(1.0, fabs(baseline.trigger_altitude)));
    assert(fabs(perturbed.minimum_speed - baseline.minimum_speed) <=
        sqrt(DBL_EPSILON) * fmax(1.0, fabs(baseline.minimum_speed)));
    assert(fabs(perturbed.reference_speed - baseline.reference_speed) <=
        sqrt(DBL_EPSILON) * fmax(1.0, fabs(baseline.reference_speed)));
    puts("PASS: physical preflare contract is invariant to terminal preview speed hints.");
}

static void test_terminal_mission_actuators_remain_forbidden(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.vehicle.allow_powered_approach = true;
    cfg.vehicle.maximum_approach_throttle = 1.0;
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    seed_preflare_plan(&g);
    terminal_set_stage(&g, TERMINAL_PREFLARE, t.ut);
    t.physics_airbrake_model_available = true;
    t.physics_airbrake_model_confidence = 1;
    t.physics_airbrake_drag_accel = 20.0;
    t.available_thrust = 1.0e6;
    t.radar_altitude = fmax(300, cfg.guidance.flare_altitude * 4);
    Trajectory ref;
    trajectory_init(&ref);

    double speeds[] = {
        g.preflare_reference_speed + 30.0,
        fmax(1.0, g.preflare_minimum_speed - 10.0)
    };
    for (size_t scenario = 0; scenario < sizeof(speeds) / sizeof(speeds[0]); ++scenario) {
        g.airbrakes_deployed = true;
        t.true_air_speed = speeds[scenario];
        t.ut += .1;
        GuidanceResult r = preflare_guidance(&g, &t, t.heading, &cfg, &ref, .1);
        assert(!g.airbrakes_deployed);
        assert(!r.command.airbrakes);
        assert(r.command.target_throttle == 0.0);
        guidance_result_clear(&r);
    }
    trajectory_clear(&ref);
    puts("PASS: terminal mission contract forbids airbrakes and thrust even when both are available.");
}

static void test_missed_preflare_cannot_continue_steep_descent(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700, .confidence = .8};
    for (int scenario = 0; scenario < 3; ++scenario) {
        GuidanceMachine g = verifier_machine();
        Telemetry t = verifier_approach_telemetry(&cfg);
        t.roll = 15; /* Within airborne corridor, outside preflare alignment. */
        t.vertical_speed = -20;
        t.flight_path_angle = atan2(t.vertical_speed, t.horizontal_speed) * RAD2DEG;
        t.drag_force = 1000;
        terminal_set_stage(&g, scenario == 1 ? TERMINAL_TRAJECTORY_CAPTURE : TERMINAL_OUTER_FINAL, t.ut);
        TerminalPreflarePlan plan = terminal_preflare_plan(&g, &t, &p, aero, &cfg);
        assert(plan.feasible);
        terminal_store_preflare_plan(&g, &plan);
        double critical = fmax(100, cfg.guidance.flare_altitude * 2) + plan.predicted_height_loss;
        if (scenario == 2) t.stall_fraction = .20; /* Invalidates live plan only. */
        Trajectory ref;
        trajectory_init(&ref);
        bool aborted = false;
        for (int n = 0; n < 6; ++n) {
            t.radar_altitude = critical + 20 - n * 10;
            t.mean_altitude = cfg.site.altitude + t.radar_altitude;
            t.ut += .1;
            GuidanceResult r = terminal_approach_sequence(&g, &t, t.heading,
                &p, aero, &cfg, &ref, .1);
            aborted = r.phase == PHASE_ABORT;
            guidance_result_clear(&r);
            if (aborted) break;
        }
        assert(aborted);
        assert(g.aborted);
        trajectory_clear(&ref);
    }
}

static void test_late_aligned_capture_rejects_exhausted_pullup_reserve(void) {
    PlanetModel p = verifier_kerbin();
    LandingConfiguration cfg = landing_configuration_default();
    AerodynamicModel aero = {.lift_to_drag = 2.5, .ballistic_coefficient = 700, .confidence = .8};
    GuidanceMachine g = verifier_machine();
    Telemetry t = verifier_approach_telemetry(&cfg);
    t.radar_altitude = 364;
    t.mean_altitude = cfg.site.altitude + t.radar_altitude;
    t.horizontal_speed = 140 * cos(20 * DEG2RAD);
    t.vertical_speed = -140 * sin(20 * DEG2RAD);
    t.flight_path_angle = -20;
    seed_preflare_plan(&g);
    TerminalPreflarePlan live = terminal_preflare_plan(&g, &t, &p, aero, &cfg);
    assert(live.feasible);
    assert(live.predicted_height_loss > t.radar_altitude);
    Trajectory ref;
    trajectory_init(&ref);
    GuidanceResult r = terminal_approach_sequence(&g, &t, t.heading,
        &p, aero, &cfg, &ref, .1);
    assert(r.phase == PHASE_ABORT);
    assert(strstr(r.status, "pull-up reserve") != NULL);
    guidance_result_clear(&r);
    trajectory_clear(&ref);
}

int main(void) {
    test_late_aligned_capture_rejects_exhausted_pullup_reserve();
    test_preflare_respects_weak_authority_and_slow_response();
    test_preflare_contract_ignores_preview_hints();
    test_terminal_mission_actuators_remain_forbidden();
    test_missed_preflare_cannot_continue_steep_descent();
    test_terminal_stage_is_monotonic();
    test_shallow_glide_cannot_be_bypassed();
    test_off_runway_landed_state_is_not_runway_success();
    test_on_runway_landed_without_gear_is_not_success();
    test_runway_contact_debounces_without_landed_string();
    test_rollout_steering_damps_cross_track_rate();
    test_rollout_cannot_complete_after_stopping_off_runway();
    puts("Approach sequence contract tests passed.");
    return 0;
}
