#include "../CLanding/decision_envelope.h"

#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

static PlanetModel planet(void) {
    PlanetModel p = {0};
    p.radius = 600000.0;
    p.gravitational_parameter = 3.5316e12;
    return p;
}

static LandingConfiguration config(void) {
    LandingConfiguration c = {0};
    c.site.altitude = 70.0;
    c.site.runway_heading = 90.0;
    c.site.runway_length = 2500.0;
    c.vehicle.maximum_bank_angle = 55.0;
    c.vehicle.maximum_dynamic_pressure = 30000.0;
    c.vehicle.maximum_g_load = 3.0;
    c.vehicle.minimum_safe_speed = 85.0;
    c.guidance.approach_roll_rate = 10.0;
    c.guidance.entry_roll_acceleration = 5.0;
    return c;
}

static Telemetry telemetry(void) {
    Telemetry t = {0};
    t.mean_altitude = 1500.0;
    t.radar_altitude = 1430.0;
    t.horizontal_speed = 120.0;
    t.true_air_speed = 130.0;
    t.vertical_speed = -15.0;
    t.runway_along_track = -9000.0;
    t.runway_cross_track = 500.0;
    t.mass = 70000.0;
    t.lift_force = 900000.0;
    t.g_force = 1.2;
    t.dynamic_pressure = 8000.0;
    t.bank_effectiveness = 1.0;
    t.attitude_response.pitch_valid = true;
    t.attitude_response.maximum_pitch_accel_deg_s2 = 5.0;
    t.attitude_response.maximum_pitch_rate_deg_s = 10.0;
    return t;
}

static void test_response_drift(void) {
    /* Independently integrate wait -> brake -> rest-to-rest translation. */
    double position = 100.0, velocity = 20.0, delay = 3.0, accel = 4.0;
    double after_wait = position + velocity * delay;
    double stop_time = velocity / accel;
    double after_stop = after_wait + velocity * stop_time - accel * stop_time * stop_time / 2.0;
    double expected = delay + stop_time + 2.0 * sqrt(after_stop / accel);
    double actual = decision_bounded_capture_time(position, velocity, accel, delay);
    assert(fabs(actual - expected) < 1e-12);
    assert(actual > decision_bounded_capture_time(position, velocity, accel, 0.0));
    assert(actual > decision_bounded_capture_time(position, velocity, 2.0 * accel, delay));
    assert(actual == decision_bounded_capture_time(-position, -velocity, accel, delay));
}

static void test_vertical_recovery(void) {
    VerticalRecoveryEnvelope base = decision_vertical_recovery_envelope(500, 30, 5, 2, 3, 4);
    assert(base.valid && base.reachable);
    assert(base.sink_after_response_mps == 36.0);
    assert(base.response_height_m == 66.0);
    assert(fabs(base.braking_height_m - (36.0*36.0 - 25.0)/8.0) < 1e-12);
    VerticalRecoveryEnvelope slower = decision_vertical_recovery_envelope(500, 30, 5, 3, 3, 4);
    VerticalRecoveryEnvelope weaker = decision_vertical_recovery_envelope(500, 30, 5, 2, 3, 2);
    VerticalRecoveryEnvelope banked = decision_vertical_recovery_envelope(500, 30, 5, 2, 6, 4);
    assert(slower.height.margin < base.height.margin);
    assert(weaker.height.margin < base.height.margin);
    assert(banked.height.margin < base.height.margin);
    VerticalRecoveryEnvelope boundary = decision_vertical_recovery_envelope(base.required_height_m, 30, 5, 2, 3, 4);
    assert(boundary.reachable && boundary.height.margin == 0.0);
    boundary = decision_vertical_recovery_envelope(nextafter(base.required_height_m, 0), 30, 5, 2, 3, 4);
    assert(boundary.valid && !boundary.reachable && boundary.height.margin < 0.0);
    assert(!decision_vertical_recovery_envelope(500, 30, 5, 2, 3, 0).valid);
    assert(!decision_vertical_recovery_envelope(500, NAN, 5, 2, 3, 4).valid);
    assert(!decision_vertical_recovery_envelope(500, 30, 5, DBL_MAX, 3, 4).valid);
    assert(!decision_margin(DBL_MAX, -DBL_MAX).valid);
    assert(!decision_margin(NAN, 0).valid);
}

int main(void) {
    test_response_drift();
    test_vertical_recovery();
    DecisionMargin m = decision_margin(10.0, 7.0);
    assert(m.valid && fabs(m.margin - 3.0) < 1e-12);

    double capture = decision_bounded_capture_time(500.0, 20.0, 4.0, 1.0);
    assert(isfinite(capture) && capture > 1.0);
    assert(isinf(decision_bounded_capture_time(1.0, 0.0, 0.0, 0.0)));

    double axis_base = decision_axis_capture_time(300.0, 0.0, 5.0, 20.0);
    double axis_more_accel = decision_axis_capture_time(300.0, 0.0, 10.0, 20.0);
    double axis_more_rate = decision_axis_capture_time(300.0, 0.0, 5.0, 40.0);
    assert(isfinite(axis_base) && axis_base > 0.0);
    assert(axis_more_accel < axis_base);
    assert(axis_more_rate <= axis_base);
    double axis_accel_only =
        decision_axis_capture_time(30.0, 0.0, 5.0, INFINITY);
    double axis_large_rate =
        decision_axis_capture_time(30.0, 0.0, 5.0, 1.0e9);
    assert(isfinite(axis_accel_only));
    assert(fabs(axis_accel_only - axis_large_rate) <=
        16.0 * DBL_EPSILON * fmax(1.0, axis_accel_only));

    double axis_toward = decision_axis_capture_time(30.0, 5.0, 5.0, 20.0);
    double axis_away = decision_axis_capture_time(30.0, -5.0, 5.0, 20.0);
    double axis_mirror = decision_axis_capture_time(-30.0, -5.0, 5.0, 20.0);
    assert(axis_toward < axis_away);
    assert(fabs(axis_toward - axis_mirror) <=
        16.0 * DBL_EPSILON * fmax(1.0, axis_toward));

    double axis_overshoot = decision_axis_capture_time(1.0, 10.0, 5.0, 20.0);
    double axis_zero_error_moving = decision_axis_capture_time(0.0, 5.0, 5.0, 20.0);
    assert(isfinite(axis_overshoot) && axis_overshoot > 0.0);
    assert(isfinite(axis_zero_error_moving) && axis_zero_error_moving > 0.0);
    assert(decision_axis_capture_time(0.0, 0.0, 5.0, 20.0) == 0.0);

    PlanetModel p = planet();
    LandingConfiguration c = config();
    Telemetry t = telemetry();
    GuidanceMachine g = {0};
    g.terminal_sink_accel_ema = 3.0;
    g.terminal_pitch_response_delay_ema = 0.5;

    RunwayCaptureEnvelope env = decision_runway_capture_envelope(&g, &t,
        90.0, &p, &c);
    assert(env.valid);
    assert(env.lateral_accel_mps2 > 0.0);
    assert(env.vertical_recovery.valid);
    assert(env.speed.margin > 0.0);

    ControlAuthorityEnvelope control =
        decision_control_authority_envelope(&g, &t, &p, &c);
    assert(control.valid);
    assert(control.survivable);
    assert(control.controllable);
    assert(control.maximum_lateral_accel_mps2 > 0.0);

    TargetCaptureEnvelope target = decision_target_capture_envelope(
        &g, &t, 90.0, -3000.0, 0.0, 90.0, 120.0, &p, &c);
    assert(target.valid);
    assert(target.forward_m > 0.0);
    assert(target.capture_time.valid);
    assert(target.path_length_m >= target.range_m);
    assert(target.required_time_s > 0.0);

    t.runway_along_track = c.site.runway_length + 1.0;
    env = decision_runway_capture_envelope(&g, &t, 90.0, &p, &c);
    assert(env.valid);
    assert(env.runway_remaining.margin < 0.0);
    assert(!env.reachable);

    puts("decision_envelope_tests: PASS");
    return 0;
}
