#include "../CLanding/flight_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int finite_output(const FlightControlOutput *o) {
    return isfinite(o->pitch) && isfinite(o->roll) && isfinite(o->yaw) &&
           isfinite(o->rcs_assist) && isfinite(o->diagnostics.attitude_error);
}

static void set_axis_authority(Telemetry *t, double pitch, double roll, double yaw) {
    const double inertia = 1000.0;
    t->has_torque = true;
    t->has_inertia = true;
    t->pitch_moment_of_inertia = inertia;
    t->roll_moment_of_inertia = inertia;
    t->yaw_moment_of_inertia = inertia;
    t->available_pitch_torque = pitch * DEG2RAD * inertia;
    t->available_roll_torque = roll * DEG2RAD * inertia;
    t->available_yaw_torque = yaw * DEG2RAD * inertia;
}

static Telemetry base_telemetry(void) {
    Telemetry t;
    memset(&t, 0, sizeof(t));
    t.ut = 100.0;
    t.mean_altitude = 15000.0;
    t.radar_altitude = 14000.0;
    t.true_air_speed = 220.0;
    t.horizontal_speed = 215.0;
    t.surface_speed = 220.0;
    t.vertical_speed = -28.0;
    t.flight_path_angle = atan2(t.vertical_speed, t.horizontal_speed) * RAD2DEG;
    t.pitch = 5.0;
    t.angle_of_attack = 12.0;
    t.roll = 0.0;
    t.heading = 90.0;
    t.ground_track_heading = 90.0;
    t.sideslip = 0.0;
    t.dynamic_pressure = 3500.0;
    t.mach = 0.75;
    t.mass = 40000.0;
    t.has_controls = true;
    t.has_body_pitch_rate = true;
    t.has_body_roll_rate = true;
    t.has_body_yaw_rate = true;
    set_axis_authority(&t, 28.0, 40.0, 15.0);
    return t;
}

static GuidanceCommand base_command(ControlProfile profile) {
    GuidanceCommand c;
    memset(&c, 0, sizeof(c));
    c.control_profile = profile;
    c.autopilot_engaged = true;
    c.heading_control_enabled = profile == PROFILE_ENTRY || profile == PROFILE_RECOVERY;
    c.hac_control_tuning = profile == PROFILE_TAEM || profile == PROFILE_APPROACH;
    c.terminal_pitch_tuning = profile == PROFILE_TAEM || profile == PROFILE_APPROACH || profile == PROFILE_FLARE;
    c.target_pitch = 5.0;
    c.has_target_aoa = true;
    c.target_aoa = 12.0;
    c.target_heading = 90.0;
    c.target_roll = 0.0;
    return c;
}

static FlightControlOutput single_step(FlightControlState *s, Telemetry *t, GuidanceCommand *c, double dt) {
    FlightControlOutput o;
    assert(flight_control_step(s, t, c, dt, &o));
    assert(o.valid);
    assert(finite_output(&o));
    assert(fabs(o.pitch) <= 1.0 + 1e-12);
    assert(fabs(o.roll) <= 1.0 + 1e-12);
    assert(fabs(o.yaw) <= 1.0 + 1e-12);
    assert(o.rcs_assist >= 0.0 && o.rcs_assist <= 1.0);
    return o;
}

static void test_aoa_coordinate_rate_damping(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_TAEM);
    c.target_aoa = 14.0;
    t.angle_of_attack = 12.0;

    FlightControlState toward;
    flight_control_init(&toward, .1);
    t.body_pitch_rate = 10.0;
    (void)single_step(&toward, &t, &c, .1);

    /* AoA is moving toward the higher target quickly while body q reports the
       opposite sign. Damping must follow d(AoA)/dt, not body q. */
    t.ut += .1;
    t.angle_of_attack = 13.0;
    t.body_pitch_rate = -10.0;
    FlightControlOutput braking = single_step(&toward, &t, &c, .1);
    assert(braking.pitch < 0.0);
    assert(braking.diagnostics.body_pitch_rate_available);
    assert(braking.diagnostics.effective_pitch_rate > 4.0);
    assert(braking.diagnostics.body_pitch_rate < -5.0);

    FlightControlState away;
    flight_control_init(&away, .1);
    t.ut = 100.0;
    t.angle_of_attack = 13.0;
    t.body_pitch_rate = -10.0;
    (void)single_step(&away, &t, &c, .1);
    t.ut += .1;
    t.angle_of_attack = 12.0;
    t.body_pitch_rate = 10.0;
    FlightControlOutput capture = single_step(&away, &t, &c, .1);
    assert(capture.diagnostics.effective_pitch_rate < -4.0);
    assert(capture.diagnostics.body_pitch_rate > 5.0);
    assert(capture.pitch > 0.0);
    assert(capture.pitch > braking.pitch);
}


static void test_frozen_body_pitch_rate_uses_geometric_damping(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    c.target_aoa = 14.0;
    t.angle_of_attack = 12.0;
    t.body_pitch_rate = 0.0;

    FlightControlState s;
    flight_control_init(&s, .1);
    (void)single_step(&s, &t, &c, .1);

    /* Reproduce the recorded failure mode: the kRPC body-rate field remains a
       finite zero while the independent AoA attitude derivative is moving. */
    t.ut += .1;
    t.angle_of_attack = 13.0;
    t.pitch += 1.0;
    t.body_pitch_rate = 0.0;
    FlightControlOutput o = single_step(&s, &t, &c, .1);

    assert(!o.diagnostics.body_pitch_rate_available);
    assert(o.diagnostics.effective_pitch_rate > 4.0);
    assert(o.pitch < 0.0);
}


static void test_entry_thin_air_brakes_before_aoa_overshoot(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    c.target_aoa = 12.0;
    t.angle_of_attack = 17.6;
    t.dynamic_pressure = 20.0;
    t.mach = 6.0;
    t.body_pitch_rate = 3.3;
    t.pitch_rate = 3.3;
    set_axis_authority(&t, 8.0, 8.0, 8.0);

    FlightControlState s;
    flight_control_init(&s, .1);
    (void)single_step(&s, &t, &c, .1);

    /* The controlled AoA is already moving toward the lower target quickly,
       while body q points the other way. Positive pitch must brake the AoA
       derivative instead of following the body-axis sign. */
    t.ut += .1;
    t.angle_of_attack = 17.0;
    FlightControlOutput o = single_step(&s, &t, &c, .1);

    assert(o.diagnostics.body_pitch_rate_available);
    assert(o.diagnostics.effective_pitch_rate < -2.0);
    assert(o.diagnostics.body_pitch_rate > 0.0);
    assert(o.pitch > 0.12);
}

static void test_roll_sample_hold_preserves_braking_reachability(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_TAEM);
    c.target_roll = 45.0;
    set_axis_authority(&t, 30.0, 1200.0, 20.0);

    FlightControlState s;
    flight_control_init(&s, .1);
    FlightControlOutput o = single_step(&s, &t, &c, .25);
    assert(o.roll > 0.0);
    assert(o.diagnostics.roll_guard_authority == 0.0);

    double a = o.diagnostics.roll_raw_authority;
    double hold = o.diagnostics.roll_hold_seconds;
    double end_rate = a * o.roll * hold;
    double distance_used = 0.5 * a * o.roll * hold * hold;
    double remaining = c.target_roll - distance_used;
    double braking_distance = end_rate * end_rate / (2.0 * a);
    assert(remaining + 1e-8 >= braking_distance);
    assert(o.roll < 1.0);
}

static void test_recovery_uses_generic_controlled_coordinate_rate(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_RECOVERY);
    c.target_roll = 0.0;
    t.roll = 35.0;
    t.body_roll_rate = -12.0;

    FlightControlState s;
    flight_control_init(&s, .1);
    (void)single_step(&s, &t, &c, .1);
    t.ut += .1;
    t.roll = 38.0;
    FlightControlOutput o = single_step(&s, &t, &c, .1);
    assert(o.diagnostics.effective_roll_rate > 2.0);
    assert(o.diagnostics.body_roll_rate < 0.0);
    assert(!o.diagnostics.recovery_rate_first);
    assert(fabs(o.diagnostics.commanded_roll_rate) < 1e-9);
    assert(o.roll < 0.0);
}

static void test_moving_bank_reference_feedforward(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_TAEM);
    FlightControlState s;
    flight_control_init(&s, .1);

    c.target_roll = 0.0;
    (void)single_step(&s, &t, &c, .1);
    t.ut += .1;
    c.target_roll = 2.0;
    FlightControlOutput o = single_step(&s, &t, &c, .1);
    assert(o.diagnostics.target_roll_rate > 0.0);
    /* Reference rate is the measured derivative of the requested bank. The
       acceleration-limited actuator controller owns attainability; the old
       fixed 6 deg/s feedforward cap is not a vehicle requirement. */
    assert(fabs(o.diagnostics.target_roll_rate - 2.0 / .1) < 1e-9);
    assert(o.diagnostics.commanded_roll_rate > 0.0);
    assert(o.roll > 0.0);
    assert(o.roll <= 1.0 && finite_output(&o));
}

static void test_terminal_pitch_control_has_no_hidden_integral(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_TAEM);
    c.target_aoa = 15.0;
    t.angle_of_attack = 8.0;
    t.body_pitch_rate = 0.0;

    FlightControlState s;
    flight_control_init(&s, .1);
    FlightControlOutput o = {0};
    for (int i = 0; i < 300; ++i) {
        t.ut += .1;
        o = single_step(&s, &t, &c, .1);
    }
    /* Disturbance acceleration is compensated by plant inversion. The current
       controller intentionally carries no hidden pitch trim or integral. */
    assert(fabs(s.terminal_pitch_integral) < 1e-12);
    assert(fabs(o.diagnostics.terminal_pitch_integral) < 1e-12);
    assert(fabs(s.pitch_trim) < 1e-12);
    assert(o.pitch > 0.0);
    assert(o.pitch <= 1.0 && finite_output(&o));
}

static void seed_beta_model(FlightControlState *s) {
    s->beta.initialized = true;
    /* kRPC beta ~= course - heading, hence positive yaw rate reduces beta. */
    s->beta.yaw_gain = -1.0;
    s->beta.roll_gain = 0.20;
    s->beta.confidence = 1.0;
}

static void test_beta_yaw_and_cross_axis_guard(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_TAEM);
    t.sideslip = 6.0;
    t.body_yaw_rate = 0.0;
    t.body_roll_rate = 0.0;

    FlightControlState quiet;
    flight_control_init(&quiet, .1);
    seed_beta_model(&quiet);
    FlightControlOutput nominal = single_step(&quiet, &t, &c, .1);
    assert(nominal.yaw < 0.0);

    FlightControlState rolling;
    flight_control_init(&rolling, .1);
    seed_beta_model(&rolling);
    t.sideslip = 1.5;
    t.roll = 0.0;
    t.body_roll_rate = 18.0;
    (void)single_step(&rolling, &t, &c, .1);
    t.ut += .1;
    t.roll = -3.0;
    FlightControlOutput guarded = single_step(&rolling, &t, &c, .1);
    assert(guarded.diagnostics.effective_roll_rate < -2.0);
    assert(guarded.diagnostics.body_roll_rate > 0.0);

    FlightControlState calm_small_beta;
    flight_control_init(&calm_small_beta, .1);
    seed_beta_model(&calm_small_beta);
    t.ut = 100.0;
    t.roll = 0.0;
    t.body_roll_rate = 0.0;
    (void)single_step(&calm_small_beta, &t, &c, .1);
    t.ut += .1;
    FlightControlOutput unguarded = single_step(&calm_small_beta, &t, &c, .1);
    /* Roll-coordinate disagreement must not invent a hidden yaw attenuation. */
    assert(guarded.yaw < 0.0);
    assert(unguarded.yaw < 0.0);
    assert(fabs(guarded.yaw - unguarded.yaw) < 1e-12);
}


static void test_entry_beta_uses_yaw_without_inventing_bank_command(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    c.heading_control_enabled = false;
    c.target_roll = 0.0;
    t.dynamic_pressure = 80.0;
    t.mach = 6.0;
    t.roll = 0.0;
    t.body_roll_rate = 0.0;
    t.sideslip = 8.0;

    FlightControlState positive;
    flight_control_init(&positive, .1);
    seed_beta_model(&positive);
    FlightControlOutput right_beta = single_step(&positive, &t, &c, .1);
    assert(fabs(right_beta.roll) < 1e-12);
    assert(right_beta.yaw < 0.0);

    FlightControlState negative;
    flight_control_init(&negative, .1);
    seed_beta_model(&negative);
    t.sideslip = -8.0;
    FlightControlOutput left_beta = single_step(&negative, &t, &c, .1);
    assert(fabs(left_beta.roll) < 1e-12);
    assert(left_beta.yaw > 0.0);
}

static void test_beta_estimator_never_flips_nonphysical_yaw_sign(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    FlightControlState s;
    flight_control_init(&s, .1);

    /* Seed a physically correct estimate, then feed a deliberately correlated
       roll/yaw/beta sequence that would otherwise make the unconstrained local
       regression try a positive yaw coefficient. The estimator may lower its
       confidence, but it must never reverse the kinematic yaw sign. */
    s.beta.initialized = true;
    s.beta.yaw_gain = -1.0;
    s.beta.roll_gain = 0.2;
    s.beta.confidence = 0.8;
    for (int i = 0; i < 160; ++i) {
        t.ut += .1;
        double r = 4.0 * sin((double)i * .13);
        t.body_yaw_rate = r;
        t.body_roll_rate = 0.85 * r + 0.25 * cos((double)i * .21);
        /* Nonphysical/confounded observation: beta moves with positive yaw. */
        t.sideslip += r * .08;
        (void)single_step(&s, &t, &c, .1);
        assert(s.beta.yaw_gain <= -0.05 + 1e-12);
    }
}

static void test_atmospheric_control_never_requests_rcs(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    c.target_aoa = 20.0;
    t.angle_of_attack = 8.0;
    t.dynamic_pressure = 0.0;
    t.body_pitch_rate = 0.0;

    const double mach_samples[] = {1.40, 1.00, 1.15, 1.30};
    FlightControlState s;
    flight_control_init(&s, .1);
    flight_control_seed_axis_authority(&s, FLIGHT_CONTROL_AXIS_PITCH, 8.0, 0.0, .5);
    flight_control_seed_axis_authority(&s, FLIGHT_CONTROL_AXIS_ROLL, 8.0, 0.0, .5);
    flight_control_seed_axis_authority(&s, FLIGHT_CONTROL_AXIS_YAW, 8.0, 0.0, .5);

    for (size_t i = 0; i < sizeof(mach_samples) / sizeof(mach_samples[0]); ++i) {
        t.mach = mach_samples[i];
        FlightControlOutput output = single_step(&s, &t, &c, .1);
        assert(!output.rcs_requested);
        assert(output.rcs_assist == 0.0);
        assert(output.diagnostics.rcs_transonic_cutoff);
        t.ut += .1;
    }
}

static void test_flare_synthetic_closed_loop(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_FLARE);
    c.target_aoa = 14.0;
    c.target_roll = 0.0;
    t.angle_of_attack = 7.0;
    t.roll = 16.0;
    t.body_pitch_rate = 0.0;
    t.body_roll_rate = 0.0;
    t.body_yaw_rate = 0.0;
    t.dynamic_pressure = 5000.0;
    t.mach = 0.45;
    set_axis_authority(&t, 24.0, 32.0, 12.0);

    FlightControlState s;
    flight_control_init(&s, .05);
    const double initial_aoa_error = fabs(c.target_aoa - t.angle_of_attack);
    const double initial_roll_error = fabs(t.roll);
    double peak_pitch_rate = 0.0;
    double peak_roll_rate = 0.0;
    FlightControlOutput o = {0};

    for (int i = 0; i < 500; ++i) {
        o = single_step(&s, &t, &c, .05);
        /* Simple damped synthetic plant. It is intentionally not a vehicle model;
           it only checks that the pure-C controller closes the signs and does not
           create a low-speed limit cycle under realistic order-of-magnitude axis
           authority. */
        double pitch_accel = 24.0 * o.pitch - 0.9 * t.body_pitch_rate;
        double roll_accel = 32.0 * o.roll - 1.0 * t.body_roll_rate;
        t.body_pitch_rate += pitch_accel * .05;
        t.body_roll_rate += roll_accel * .05;
        t.angle_of_attack += t.body_pitch_rate * .05;
        t.pitch += t.body_pitch_rate * .05;
        t.roll += t.body_roll_rate * .05;
        t.ut += .05;
        peak_pitch_rate = fmax(peak_pitch_rate, fabs(t.body_pitch_rate));
        peak_roll_rate = fmax(peak_roll_rate, fabs(t.body_roll_rate));
    }

    assert(fabs(c.target_aoa - t.angle_of_attack) < initial_aoa_error * .30);
    assert(fabs(t.roll) < initial_roll_error * .20);
    assert(fabs(t.body_pitch_rate) < 1.5);
    assert(fabs(t.body_roll_rate) < 1.5);
    /* Keep transient rates below the axis-authority scale while allowing the
       initial 16-degree roll recovery to use the controller's full response. */
    assert(peak_pitch_rate < 12.0);
    assert(peak_roll_rate < 18.0);
    assert(!o.rcs_requested);
}

static void test_rollout_is_neutral_and_ground_controls_pass_through(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ROLLOUT);
    c.gear = true;
    c.brakes = true;
    c.airbrakes = true;
    c.target_throttle = .2;
    c.wheel_steering = -.4;
    FlightControlState s;
    flight_control_init(&s, .1);
    FlightControlOutput o = single_step(&s, &t, &c, .1);
    assert(o.pitch == 0.0 && o.roll == 0.0 && o.yaw == 0.0);
    assert(!o.rcs_requested && o.rcs_assist == 0.0);
    assert(o.gear && o.brakes);
    assert(!o.airbrakes); /* explicit mission constraint: airbrakes stay disabled */
    assert(fabs(o.throttle - .2) < 1e-12);
    assert(fabs(o.wheel_steering + .4) < 1e-12);
}

static void test_rewind_resets_transient_derivatives(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_TAEM);
    FlightControlState s;
    flight_control_init(&s, .1);
    (void)single_step(&s, &t, &c, .1);
    t.ut += .1;
    t.roll = 20.0;
    (void)single_step(&s, &t, &c, .1);
    assert(fabs(s.roll_rate) > 1.0);
    t.ut = 10.0;
    t.roll = 0.0;
    (void)single_step(&s, &t, &c, .1);
    assert(fabs(s.roll_rate) < 1e-9);
}

static void test_cross_coupled_body_rates_do_not_override_attitude_derivatives(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    c.target_roll = 0.0;
    c.target_aoa = 28.0;
    t.mean_altitude = 59000.0;
    t.true_air_speed = 2100.0;
    t.dynamic_pressure = 100.0;
    t.mach = 7.1;
    t.roll = 30.0;
    t.angle_of_attack = 35.0;
    t.body_roll_rate = 2.0;
    t.body_pitch_rate = 6.0;
    set_axis_authority(&t, 28.0, 40.0, 15.0);

    FlightControlState s;
    flight_control_init(&s, .1);
    (void)single_step(&s, &t, &c, .1);

    /* Reproduce the 17:57 failure geometry: the bank and AoA are already
       moving toward their targets, while body p/q still have the opposite sign. */
    t.ut += .1;
    t.roll = 29.4;
    t.angle_of_attack = 34.4;
    FlightControlOutput o = single_step(&s, &t, &c, .1);

    assert(s.roll_rate < -1.0);
    assert(s.aoa_rate < -1.0);
    assert(o.diagnostics.effective_roll_rate < -1.0);
    assert(o.diagnostics.effective_pitch_rate < -1.0);
    assert(o.diagnostics.body_roll_rate_available);
    assert(o.diagnostics.body_pitch_rate_available);
    assert(fabs(o.diagnostics.body_roll_rate - 2.0) < 1e-12);
    assert(fabs(o.diagnostics.body_pitch_rate - 6.0) < 1e-12);
    assert(o.diagnostics.effective_roll_rate * o.diagnostics.body_roll_rate < 0.0);
    assert(o.diagnostics.effective_pitch_rate * o.diagnostics.body_pitch_rate < 0.0);
    /* With 29.4 degrees still to travel at -6 deg/s, the bounded controller
       commands positive roll to brake the observed bank-coordinate motion.
       The opposite-sign +2 deg/s body rate remains diagnostic-only. */
    assert(o.roll > 0.0);
    assert(o.pitch > 0.0);
}

static void test_cold_start_body_rates_do_not_become_controlled_coordinate_rates(void) {
    Telemetry t = base_telemetry();
    GuidanceCommand c = base_command(PROFILE_ENTRY);
    FlightControlState s;
    flight_control_init(&s, .1);

    /* First control sample has no coordinate derivative history yet. Body p/q
       may be large and even opposite the future bank/AoA derivative, but they
       belong only to rigid-body authority diagnostics. Controlled-coordinate
       damping must start neutral until d(bank)/dt and d(AoA)/dt are observed. */
    t.body_pitch_rate = 11.0;
    t.body_roll_rate = -13.0;
    t.angle_of_attack = c.target_aoa;
    t.roll = c.target_roll;
    FlightControlOutput o = single_step(&s, &t, &c, .1);
    assert(fabs(o.diagnostics.effective_pitch_rate) < 1e-12);
    assert(fabs(o.diagnostics.effective_roll_rate) < 1e-12);
    assert(o.diagnostics.body_pitch_rate_available);
    assert(o.diagnostics.body_roll_rate_available);
    assert(fabs(o.diagnostics.body_pitch_rate - 11.0) < 1e-12);
    assert(fabs(o.diagnostics.body_roll_rate + 13.0) < 1e-12);
}

int main(void) {
    assert(strcmp(flight_control_revision(), FLIGHT_CONTROL_REVISION) == 0);
    test_aoa_coordinate_rate_damping();
    test_frozen_body_pitch_rate_uses_geometric_damping();

    test_entry_thin_air_brakes_before_aoa_overshoot();
    test_roll_sample_hold_preserves_braking_reachability();
    test_recovery_uses_generic_controlled_coordinate_rate();
    test_moving_bank_reference_feedforward();
    test_terminal_pitch_control_has_no_hidden_integral();
    test_beta_yaw_and_cross_axis_guard();
    test_entry_beta_uses_yaw_without_inventing_bank_command();
    test_beta_estimator_never_flips_nonphysical_yaw_sign();
    test_atmospheric_control_never_requests_rcs();
    test_flare_synthetic_closed_loop();
    test_rollout_is_neutral_and_ground_controls_pass_through();
    test_rewind_resets_transient_derivatives();
    test_cold_start_body_rates_do_not_become_controlled_coordinate_rates();
    test_cross_coupled_body_rates_do_not_override_attitude_derivatives();
    puts("Native flight-control tests passed.");
    return 0;
}
