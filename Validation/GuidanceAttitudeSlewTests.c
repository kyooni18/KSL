#include "../CLanding/guidance.c"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Telemetry test_telemetry(const LandingConfiguration *cfg) {
    Telemetry t;
    telemetry_init(&t);
    t.ut = 100.0;
    t.mean_altitude = 40000.0;
    t.radar_altitude = 40000.0;
    t.true_air_speed = 1800.0;
    t.horizontal_speed = 1799.0;
    t.vertical_speed = -60.0;
    t.flight_path_angle = -2.0;
    t.dynamic_pressure = 5000.0;
    t.mach = 5.0;
    t.heading = cfg->site.runway_heading;
    t.ground_track_heading = t.heading;
    t.angle_of_attack = 10.0;
    t.roll = 0.0;
    t.angle_of_attack_rate = 0.0;
    t.roll_rate = 0.0;
    t.has_angle_of_attack_rate = true;
    t.physics_sample_valid = true;
    return t;
}

static GuidanceResult bounded_command(GuidanceMachine *g, Telemetry *t,
        const LandingConfiguration *cfg, double aoa, double bank, double dt) {
    GuidanceCommand c;
    guidance_command_init(&c);
    c.autopilot_engaged = true;
    c.control_profile = PROFILE_ENTRY;
    c.has_target_aoa = true;
    c.target_aoa = aoa;
    c.target_pitch = t->flight_path_angle + aoa;
    c.target_roll = bank;
    c.target_heading = t->heading;
    return stabilized(g, result_make(PHASE_ENTRY_ENERGY, c, "test", NULL),
        t, &cfg->vehicle, &cfg->guidance, dt);
}

static void test_first_command_is_not_an_instant_pose_jump(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t = test_telemetry(&cfg);

    GuidanceResult r = bounded_command(&g, &t, &cfg, 28.0, 60.0, 0.1);
    assert(r.command.target_aoa > t.angle_of_attack);
    assert(r.command.target_aoa - t.angle_of_attack <=
        cfg.guidance.entry_roll_rate * 0.1 + 1e-9);
    assert(fabs(r.command.target_roll) <=
        cfg.guidance.entry_roll_rate * 0.1 + 1e-9);
    assert(r.command.target_aoa < 28.0);
    assert(fabs(r.command.target_roll) < 60.0);
    guidance_result_clear(&r);
}

static void test_reversal_brakes_the_previous_command_rate(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t = test_telemetry(&cfg);

    GuidanceResult r = bounded_command(&g, &t, &cfg, 20.0, 40.0, 0.1);
    double first_roll = r.command.target_roll;
    guidance_result_clear(&r);
    t.ut += 0.1;
    r = bounded_command(&g, &t, &cfg, 20.0, -40.0, 0.1);
    assert(fabs(r.command.target_roll - first_roll) <=
        cfg.guidance.entry_roll_rate * 0.1 + 1e-9);
    assert(r.command.target_roll > -40.0);
    assert(fabs(g.roll_limiter.rate) <=
        cfg.guidance.entry_roll_acceleration * 0.1 +
        cfg.guidance.entry_roll_rate + 1e-9);
    guidance_result_clear(&r);
}

static void test_live_response_caps_override_the_cold_start_envelope(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t = test_telemetry(&cfg);
    t.attitude_response.pitch_valid = true;
    t.attitude_response.maximum_pitch_rate_deg_s = 1.0;
    t.attitude_response.maximum_pitch_accel_deg_s2 = 0.5;
    t.attitude_response.roll_valid = true;
    t.attitude_response.maximum_roll_rate_deg_s = 1.0;
    t.attitude_response.maximum_roll_accel_deg_s2 = 0.5;

    GuidanceResult r = bounded_command(&g, &t, &cfg, 28.0, 60.0, 0.1);
    assert(r.command.target_aoa - t.angle_of_attack <= 0.1 + 1e-9);
    assert(fabs(r.command.target_roll) <= 0.1 + 1e-9);
    guidance_result_clear(&r);
}

int main(void) {
    test_first_command_is_not_an_instant_pose_jump();
    test_reversal_brakes_the_previous_command_rate();
    test_live_response_caps_override_the_cold_start_envelope();
    puts("Guidance attitude slew tests passed.");
    return 0;
}
