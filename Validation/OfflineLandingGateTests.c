#define main cnano_client_tests_embedded_main
#include "CNanoClientTests.c"
#undef main

#include "../CLanding/flight_control.h"
#include "../CLanding/guidance.c"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Reuse the stateful C-Nano fake server from CNanoClientTests so this gate
 * exercises the same standard kRPC Request/Response framing as the native
 * client tests. The synthetic flight state is deliberately deterministic: it
 * is an integration/phase-continuity gate, not a replacement for live KSP
 * aerodynamic validation.
 */

typedef struct {
    double cross_track;
    double heading_offset;
    double speed_scale;
} LandingGateScenario;

static PlanetModel gate_planet(void) {
    PlanetModel planet = {0};
    snprintf(planet.name, sizeof(planet.name), "Kerbin");
    planet.radius = 600000.0;
    planet.gravitational_parameter = 3.5316e12;
    planet.rotational_speed = 2.0 * LANDER_PI / 21549.425;
    planet.atmosphere_depth = 70000.0;
    planet.surface_density = 1.225;
    planet.atmosphere_adiabatic_index = 1.4;
    planet.north_axis = v3(0, 0, 1);
    planet.prime_meridian_at_epoch = v3(1, 0, 0);
    return planet;
}

static Telemetry gate_telemetry(const LandingConfiguration *configuration,
                                const LandingGateScenario *scenario) {
    Telemetry telemetry;
    telemetry_init(&telemetry);

    const double speed = 140.0 * scenario->speed_scale;
    const double slope = 20.0 * DEG2RAD;
    telemetry.ut = 2000.0;
    telemetry.radar_altitude = 364.0;
    telemetry.mean_altitude = configuration->site.altitude + telemetry.radar_altitude;
    telemetry.true_air_speed = speed;
    telemetry.surface_speed = speed;
    telemetry.horizontal_speed = speed * cos(slope);
    telemetry.vertical_speed = -speed * sin(slope);
    telemetry.flight_path_angle = atan2(telemetry.vertical_speed, telemetry.horizontal_speed) * RAD2DEG;
    telemetry.heading = configuration->site.runway_heading + scenario->heading_offset;
    telemetry.ground_track_heading = telemetry.heading;
    telemetry.runway_along_track = -1000.0;
    telemetry.runway_cross_track = scenario->cross_track;
    telemetry.range_to_site = hypot(telemetry.runway_along_track, telemetry.runway_cross_track);
    telemetry.angle_of_attack = 10.0;
    telemetry.pitch = telemetry.flight_path_angle + telemetry.angle_of_attack;
    telemetry.roll = 0.0;
    telemetry.mass = 40000.0;
    telemetry.dry_mass = 18000.0;
    telemetry.dynamic_pressure = 6000.0;
    telemetry.lift_force = 500000.0;
    telemetry.drag_force = 80000.0;
    telemetry.g_force = 1.0;
    telemetry.speed_of_sound = 320.0;
    telemetry.mach = telemetry.true_air_speed / telemetry.speed_of_sound;
    telemetry.bank_effectiveness = 1.0;
    telemetry.aerodynamic_confidence = 0.8;
    telemetry.trajectory_density_scale = 1.0;
    telemetry.trajectory_drag_scale = 1.0;
    telemetry.trajectory_lift_scale = 1.0;
    telemetry.trajectory_calibration_confidence = 0.8;
    telemetry.has_torque = true;
    telemetry.available_pitch_torque = 160000.0;
    telemetry.available_roll_torque = 140000.0;
    telemetry.available_yaw_torque = 100000.0;
    telemetry.has_inertia = true;
    telemetry.pitch_moment_of_inertia = 400000.0;
    telemetry.roll_moment_of_inertia = 350000.0;
    telemetry.yaw_moment_of_inertia = 300000.0;
    telemetry.has_body_pitch_rate = true;
    telemetry.has_body_roll_rate = true;
    telemetry.has_body_yaw_rate = true;
    snprintf(telemetry.vessel_situation, sizeof(telemetry.vessel_situation), "flying");
    return telemetry;
}

static GuidanceMachine gate_guidance(void) {
    GuidanceMachine guidance;
    guidance_machine_init(&guidance);
    guidance_set_engaged(&guidance, true);
    guidance.final_approach_captured = true;
    guidance.terminal_region_entered = true;
    /* This successful synthetic scenario has demonstrated 3 m/s² sink-arrest
       authority. The previous 1 m/s² assumption cannot support the steep
       final within the runway-alignment range; rejection is tested separately. */
    guidance.terminal_sink_accel_ema = 3.0;
    guidance.terminal_positive_aoa_rate_ema = 2.0;
    guidance.terminal_pitch_response_delay_ema = 1.0;
    guidance.terminal_preflare_plan_valid = true;
    guidance.preflare_trigger_altitude = 600.0;
    guidance.preflare_target_aoa = 16.0;
    guidance.preflare_target_sink = -8.0;
    guidance.preflare_minimum_speed = 100.0;
    guidance.preflare_reference_speed = 130.0;
    guidance.preflare_predicted_height_loss = 120.0;
    guidance.preflare_predicted_kinetic_margin = 2000.0;
    guidance.preflare_effective_accel = 1.0;
    return guidance;
}

static GuidanceResult gate_step(GuidanceMachine *guidance,
                                FlightControlState *flight_control,
                                CNanoFakeTransport *fake,
                                KrpcCNanoClient *client,
                                Telemetry *telemetry,
                                const LandingConfiguration *configuration,
                                const PlanetModel *planet,
                                AerodynamicModel aerodynamics) {
    Trajectory reference;
    trajectory_init(&reference);
    GuidanceResult result = terminal_approach_sequence(
        guidance,
        telemetry,
        telemetry->ground_track_heading,
        planet,
        aerodynamics,
        configuration,
        &reference,
        0.1);
    trajectory_clear(&reference);

    if (result.phase != PHASE_ABORT && result.phase != PHASE_COMPLETE) {
        FlightControlOutput output;
        assert(flight_control_step(flight_control, telemetry, &result.command, 0.1, &output));
        assert(output.valid);
        assert(isfinite(output.pitch) && fabs(output.pitch) <= 1.0001);
        assert(isfinite(output.roll) && fabs(output.roll) <= 1.0001);
        assert(isfinite(output.yaw) && fabs(output.yaw) <= 1.0001);

        char error[512] = {0};
        assert(krpc_cnano_client_set_direct_controls(
            client,
            output.pitch,
            output.roll,
            output.yaw,
            output.throttle,
            output.wheel_steering,
            output.rcs_requested,
            error,
            sizeof(error)));
        assert(krpc_cnano_client_set_gear(client, result.command.gear, error, sizeof(error)));
        assert(krpc_cnano_client_set_brakes(client, result.command.brakes, error, sizeof(error)));
        assert(krpc_cnano_client_set_airbrakes(
            client,
            configuration->vehicle.airbrake_action_group,
            result.command.airbrakes,
            error,
            sizeof(error)));
        assert(krpc_cnano_client_set_speed_mode(
            client, result.command.navball_speed_mode, error, sizeof(error)));

        /* The client owns its cumulative RPC budget, so the fake wire capture
           may be safely recycled every tick to keep this long gate bounded. */
        cnano_fake_clear_buffers(fake);
    }
    return result;
}

static unsigned run_scenario(const LandingGateScenario *scenario) {
    LandingConfiguration configuration = landing_configuration_default();
    PlanetModel planet = gate_planet();
    AerodynamicModel aerodynamics = {
        .lift_to_drag = 2.5,
        .ballistic_coefficient = 700.0,
        .confidence = 0.8,
    };

    CNanoFakeTransport fake;
    cnano_fake_init(&fake);
    ClientServerState server = {.ut = 1000.0};
    cnano_fake_set_responder(&fake, client_responder, &server);
    KrpcCNanoTransportConfig transport = cnano_fake_config(&fake);
    char error[512] = {0};
    KrpcCNanoClient *client = krpc_cnano_client_open_transport(
        &configuration, &transport, "offline-landing-gate", error, sizeof(error));
    assert(client);

    /* Exercise the production C-Nano telemetry decode path before the terminal
       scenario takes over with deterministic synthetic state. */
    Telemetry io_telemetry;
    VehicleState io_state;
    assert(krpc_cnano_client_read(
        client, &configuration, &io_telemetry, &io_state, error, sizeof(error)));
    assert(io_telemetry.ut > 0.0);
    cnano_fake_clear_buffers(&fake);

    GuidanceMachine guidance = gate_guidance();
    FlightControlState flight_control;
    flight_control_init(&flight_control, 0.1);
    Telemetry telemetry = gate_telemetry(&configuration, scenario);
    GuidanceResult result = {0};

    /* Start above the actual pull-up reserve, not the old arbitrary 364 m
       fixture (which was already too low for ~48 m/s sink). This remains a
       deterministic state-machine gate, not a simulated physical trajectory. */
    TerminalPreflarePlan initial = terminal_preflare_plan(
        &guidance, &telemetry, &planet, aerodynamics, &configuration);
    assert(initial.feasible);
    double critical = fmax(100.0, configuration.guidance.flare_altitude * 2.0) +
        initial.predicted_height_loss;
    assert(initial.trigger_altitude > critical);
    telemetry.radar_altitude = critical + (initial.trigger_altitude - critical) * .25;
    telemetry.mean_altitude = configuration.site.altitude + telemetry.radar_altitude;
    telemetry.runway_along_track = -telemetry.radar_altitude / tan(20.0 * DEG2RAD);
    telemetry.range_to_site = hypot(telemetry.runway_along_track, telemetry.runway_cross_track);
    assert(terminal_airborne_corridor_valid(&guidance, &telemetry,
        telemetry.ground_track_heading, &configuration));
    assert(terminal_preflare_alignment_valid(&guidance, &telemetry,
        telemetry.ground_track_heading, &configuration));

    /* Capture the steep 20-degree runway line, then cross the preflare gate. */
    /* Production commands gear well above preflare; model the returned gear-state
       confirmation before allowing the sink-arrest stage to complete. */
    telemetry.gear = true;
    for (int i = 0; i < 6; ++i) {
        telemetry.ut += 0.1;
        result = gate_step(
            &guidance, &flight_control, &fake, client, &telemetry,
            &configuration, &planet, aerodynamics);
        assert(result.phase != PHASE_ABORT);
        guidance_result_clear(&result);
    }
    assert(guidance.terminal_vertical_stage == TERMINAL_PREFLARE);

    /* The pull-up has arrested sink and settled onto the shallow inner final. */
    for (int i = 0; i < 12 && guidance.terminal_vertical_stage < TERMINAL_INNER_FINAL; ++i) {
        telemetry.ut += 0.1;
        telemetry.radar_altitude = 300.0 - i * 8.0;
        telemetry.mean_altitude = configuration.site.altitude + telemetry.radar_altitude;
        telemetry.runway_along_track = -1000.0;
        telemetry.range_to_site = hypot(telemetry.runway_along_track, telemetry.runway_cross_track);
        telemetry.vertical_speed = -7.0;
        telemetry.true_air_speed = 135.0 * scenario->speed_scale;
        telemetry.surface_speed = telemetry.true_air_speed;
        telemetry.horizontal_speed = telemetry.true_air_speed - 0.2;
        telemetry.flight_path_angle = atan2(telemetry.vertical_speed, telemetry.horizontal_speed) * RAD2DEG;
        telemetry.pitch = telemetry.flight_path_angle + 12.0;
        result = gate_step(
            &guidance, &flight_control, &fake, client, &telemetry,
            &configuration, &planet, aerodynamics);
        assert(result.phase != PHASE_ABORT);
        guidance_result_clear(&result);
    }
    assert(guidance.terminal_vertical_stage == TERMINAL_INNER_FINAL);

    /* Enter the touchdown flare inside the runway capture corridor. */
    telemetry.ut += 0.1;
    telemetry.radar_altitude = 50.0;
    telemetry.mean_altitude = configuration.site.altitude + telemetry.radar_altitude;
    telemetry.runway_along_track = -500.0;
    telemetry.true_air_speed = 120.0 * scenario->speed_scale;
    telemetry.surface_speed = telemetry.true_air_speed;
    telemetry.horizontal_speed = telemetry.true_air_speed - 0.1;
    telemetry.vertical_speed = -4.0;
    telemetry.flight_path_angle = atan2(telemetry.vertical_speed, telemetry.horizontal_speed) * RAD2DEG;
    telemetry.pitch = telemetry.flight_path_angle + 12.0;
    result = gate_step(
        &guidance, &flight_control, &fake, client, &telemetry,
        &configuration, &planet, aerodynamics);
    assert(result.phase == PHASE_FLARE);
    assert(guidance.terminal_vertical_stage == TERMINAL_TOUCHDOWN_FLARE);
    guidance_result_clear(&result);

    /* Debounce wheel contact without relying on the KSP situation string. */
    for (int i = 0; i < 6; ++i) {
        telemetry.ut += 0.1;
        telemetry.radar_altitude = 0.25;
        telemetry.mean_altitude = configuration.site.altitude + telemetry.radar_altitude;
        telemetry.runway_along_track = 100.0 + i * 12.0;
        telemetry.true_air_speed = 88.0 * scenario->speed_scale;
        telemetry.surface_speed = telemetry.true_air_speed;
        telemetry.horizontal_speed = telemetry.true_air_speed;
        telemetry.vertical_speed = -1.5;
        telemetry.flight_path_angle = atan2(telemetry.vertical_speed, telemetry.horizontal_speed) * RAD2DEG;
        telemetry.pitch = telemetry.flight_path_angle + 14.0;
        telemetry.gear = true;
        result = gate_step(
            &guidance, &flight_control, &fake, client, &telemetry,
            &configuration, &planet, aerodynamics);
        assert(result.phase != PHASE_ABORT);
        guidance_result_clear(&result);
    }

    /* Confirm an on-runway landed state and brake to a complete rollout stop. */
    snprintf(telemetry.vessel_situation, sizeof(telemetry.vessel_situation), "landed");
    telemetry.radar_altitude = 0.0;
    telemetry.mean_altitude = configuration.site.altitude;
    telemetry.vertical_speed = 0.0;
    telemetry.runway_along_track = 300.0;
    telemetry.true_air_speed = 70.0;
    telemetry.surface_speed = 70.0;
    telemetry.horizontal_speed = 70.0;
    for (int i = 0; i < 20; ++i) {
        telemetry.ut += 0.1;
        telemetry.true_air_speed = fmax(0.0, 70.0 - i * 4.0);
        telemetry.surface_speed = telemetry.true_air_speed;
        telemetry.horizontal_speed = telemetry.true_air_speed;
        result = gate_step(
            &guidance, &flight_control, &fake, client, &telemetry,
            &configuration, &planet, aerodynamics);
        assert(result.phase != PHASE_ABORT);
        if (result.phase == PHASE_COMPLETE) {
            break;
        }
        guidance_result_clear(&result);
    }
    assert(result.phase == PHASE_COMPLETE || guidance.phase == PHASE_COMPLETE);
    guidance_result_clear(&result);

    KrpcCNanoBudget budget = krpc_cnano_client_budget(client);
    assert(budget.total_wire_requests > 20);
    unsigned requests = budget.total_wire_requests;
    krpc_cnano_client_close(client);
    return requests;
}

static void test_orbital_up_reference_singular_fallback(void) {
    GuidanceCommand command;
    guidance_command_init(&command);

    command.control_profile = PROFILE_ORBITAL;
    command.use_inertial_direction = true;
    VehicleState state = {.position = {600000.0, 0.0, 0.0}};
    Vector3 up = {0};
    command.inertial_direction = v3(1.0, 0.0, 0.0);
    assert(krpc_orbital_up_reference(&command, &state, &up));
    assert(fabs(vdot(up, command.inertial_direction)) < 1e-9);
    assert(fabs(vmag(up) - 1.0) < 1e-9);
    command.inertial_direction = v3(-1.0, 0.0, 0.0);
    assert(krpc_orbital_up_reference(&command, &state, &up));
    assert(fabs(vdot(up, command.inertial_direction)) < 1e-9);
    assert(fabs(vmag(up) - 1.0) < 1e-9);
}

static void test_deorbit_burn_safety_gates(void) {
    LandingConfiguration configuration = landing_configuration_default();
    PlanetModel planet = gate_planet();
    AerodynamicModel aerodynamics = {0};
    VehicleState state = {
        .ut = 995.0,
        .position = {686000.0, 0.0, 0.0},
        .velocity = {0.0, 2035.0, 0.0},
        .mass = 40000.0,
    };
    DeorbitPlan plan = {0};
    plan.burn_ut = 1000.0;
    plan.estimated_burn_duration = 20.0;
    plan.delta_v = 80.0;
    plan.predicted_post_burn_periapsis_altitude = 40000.0;

    Telemetry telemetry;
    telemetry_init(&telemetry);
    telemetry.ut = state.ut;
    telemetry.mean_altitude = 86000.0;
    telemetry.radar_altitude = 86000.0;
    telemetry.vertical_speed = 0.0;
    telemetry.true_air_speed = 2035.0;
    telemetry.horizontal_speed = 2035.0;
    telemetry.surface_speed = 2035.0;
    telemetry.mass = state.mass;
    telemetry.available_thrust = 1000000.0;
    telemetry.current_thrust = 0.0;
    telemetry.periapsis_altitude = 86000.0;
    telemetry.autopilot_error = 45.0;

    GuidanceMachine guidance;
    guidance_machine_init(&guidance);
    guidance_set_engaged(&guidance, true);

    GuidanceResult result = guidance_update(
        &guidance, &telemetry, &state, &plan, &planet, aerodynamics, &configuration);
    assert(result.phase == PHASE_DEORBIT_BURN);
    assert(result.command.control_profile == PROFILE_ORBITAL);
    assert(result.command.use_inertial_direction);
    assert(result.command.target_throttle == 0.0);
    assert(!guidance.has_burn_command_started);
    guidance_result_clear(&result);

    /* Crossing the 10 degree alignment gate is the only event that may arm the burn. */
    telemetry.ut = state.ut = 996.0;
    telemetry.autopilot_error = 5.0;
    result = guidance_update(
        &guidance, &telemetry, &state, &plan, &planet, aerodynamics, &configuration);
    assert(result.phase == PHASE_DEORBIT_BURN);
    assert(guidance.has_burn_command_started);
    assert(result.command.target_throttle > 0.0);
    guidance_result_clear(&result);

    /* A subsequent attitude excursion must inhibit throttle immediately. */
    telemetry.ut = state.ut = 997.0;
    telemetry.autopilot_error = 20.0;
    result = guidance_update(
        &guidance, &telemetry, &state, &plan, &planet, aerodynamics, &configuration);
    assert(result.phase == PHASE_DEORBIT_BURN);
    assert(result.command.target_throttle == 0.0);
    guidance_result_clear(&result);

    /* If aligned thrust still makes no progress for six seconds, continuing the
       stale deorbit plan is forbidden. */
    telemetry.ut = state.ut = 1004.0;
    telemetry.autopilot_error = 5.0;
    telemetry.current_thrust = 0.0;
    result = guidance_update(
        &guidance, &telemetry, &state, &plan, &planet, aerodynamics, &configuration);
    assert(result.phase == PHASE_ABORT);
    assert(result.command.target_throttle == 0.0);
    assert(guidance.aborted);
    guidance_result_clear(&result);

    /* A vehicle that never reaches the alignment gate must not chase a missed
       deorbit window indefinitely. */
    guidance_machine_init(&guidance);
    guidance_set_engaged(&guidance, true);
    telemetry.ut = state.ut = 1021.0;
    telemetry.autopilot_error = 20.0;
    result = guidance_update(
        &guidance, &telemetry, &state, &plan, &planet, aerodynamics, &configuration);
    assert(result.phase == PHASE_ABORT);
    assert(result.command.target_throttle == 0.0);
    assert(!guidance.has_burn_command_started);
    guidance_result_clear(&result);
}

int main(void) {
    const LandingGateScenario scenarios[] = {
        {.cross_track = 0.0, .heading_offset = 0.0, .speed_scale = 1.00},
        {.cross_track = 35.0, .heading_offset = 4.0, .speed_scale = 0.96},
        {.cross_track = -35.0, .heading_offset = -4.0, .speed_scale = 1.05},
    };

    test_orbital_up_reference_singular_fallback();
    test_deorbit_burn_safety_gates();

    unsigned total_requests = 0;
    for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); ++i) {
        total_requests += run_scenario(&scenarios[i]);
    }

    printf("Offline deorbit-safety + final-to-rollout gate passed: %zu scenarios, %u C-Nano wire requests.\n",
           sizeof(scenarios) / sizeof(scenarios[0]), total_requests);
    return 0;
}
