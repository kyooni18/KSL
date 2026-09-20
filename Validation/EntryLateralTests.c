#include "entry_lateral.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846264338327950288

static EntryLateralLimits test_limits(void) {
    EntryLateralLimits limits = entry_lateral_default_limits();
    limits.maximum_bank_deg = 50.0;
    limits.maximum_roll_rate_deg_s = 12.0;
    limits.maximum_roll_accel_deg_s2 = 8.0;
    limits.minimum_leg_duration_s = 0.0;
    return limits;
}

static EntryLateralInput nominal_input(double ut) {
    EntryLateralInput input;
    memset(&input, 0, sizeof(input));
    input.ut = ut;
    input.dt = 0.1;
    input.relative_speed = 1000.0;
    input.measured_bank_deg = 30.0;
    input.measured_bank_rate_deg_s = 0.0;
    input.bank_effectiveness = 1.0;
    input.lift_accel = 10.0;
    input.authority_confidence = 1.0;
    input.longitudinal.valid = true;
    input.longitudinal.confidence = 1.0;
    input.longitudinal.required_vertical_lift_accel =
        input.lift_accel * cos(30.0 * TEST_PI / 180.0);
    return input;
}

static EntryLateralTurnInput terminal_input(double side) {
    EntryLateralTurnInput input;
    memset(&input, 0, sizeof(input));
    double radius = 50000.0;
    input.horizontal_speed_mps = 500.0;
    input.current_course_deg = 0.0;
    input.target_course_deg = side * 90.0;
    input.target_along_m = -8000.0;
    input.target_cross_m = 0.0;
    input.current_along_m = input.target_along_m - radius;
    input.current_cross_m = -side * radius;
    input.measured_bank_deg = side * 50.0;
    input.measured_bank_rate_deg_s = 0.0;
    input.lift_accel_mps2 = 20.0;
    input.bank_effectiveness = 1.0;
    input.maximum_bank_deg = 50.0;
    input.maximum_roll_rate_deg_s = 12.0;
    input.maximum_roll_accel_deg_s2 = 8.0;
    input.capture_radius_m = 1000.0;
    input.position_uncertainty_m = 0.0;
    return input;
}

static void test_vertical_lift_conversion(void) {
    EntryLateralLimits limits = test_limits();
    EntryLateralState state;
    memset(&state, 0, sizeof(state));
    EntryLateralInput input = nominal_input(100.0);
    input.measured_bank_deg = 0.0;
    input.course_to_site_error_deg = 4.0;

    EntryLateralOutput output =
        entry_lateral_update(&state, &input, &limits);
    assert(output.valid);
    assert(output.bank_sign > 0.0);
    assert(fabs(output.bank_magnitude_deg - 30.0) < 1e-6);
    assert(!output.reversal_requested);
}

static void test_allocator_preserves_owned_side_and_rate_limits(void) {
    EntryLateralLimits limits = test_limits();
    EntryLateralState state;
    entry_lateral_state_init(&state, 0.0, -1.0, 30.0);
    EntryLateralInput input = nominal_input(1.0);
    input.measured_bank_deg = 30.0;

    EntryLateralOutput output =
        entry_lateral_update(&state, &input, &limits);
    assert(output.valid);
    assert(output.bank_sign < 0.0);
    assert(output.raw_target_bank_deg < 0.0);
    assert(!output.reversal_requested);
    assert(fabs(output.target_bank_rate_deg_s) <=
        limits.maximum_roll_rate_deg_s + 1e-12);
}

static void test_terminal_turn_speed_increases_required_radius(void) {
    EntryLateralTurnInput slow = terminal_input(1.0);
    EntryLateralTurnEnvelope slow_env =
        entry_lateral_terminal_turn_envelope(&slow);
    assert(slow_env.valid);
    assert(slow_env.feasibility_margin_m >= 0.0);

    EntryLateralTurnInput fast = slow;
    fast.horizontal_speed_mps *= 2.0;
    EntryLateralTurnEnvelope fast_env =
        entry_lateral_terminal_turn_envelope(&fast);
    assert(fast_env.valid);
    assert(fast_env.minimum_turn_radius_m >
        slow_env.minimum_turn_radius_m * 3.9);
    assert(fast_env.radius_margin_m < slow_env.radius_margin_m);
    assert(fast_env.feasibility_margin_m < slow_env.feasibility_margin_m);
}

static void test_terminal_turn_more_lift_improves_radius_margin(void) {
    EntryLateralTurnInput weak = terminal_input(1.0);
    weak.lift_accel_mps2 = 10.0;
    EntryLateralTurnEnvelope weak_env =
        entry_lateral_terminal_turn_envelope(&weak);
    assert(weak_env.valid);

    EntryLateralTurnInput strong = weak;
    strong.lift_accel_mps2 = 20.0;
    EntryLateralTurnEnvelope strong_env =
        entry_lateral_terminal_turn_envelope(&strong);
    assert(strong_env.valid);
    assert(strong_env.minimum_turn_radius_m <
        weak_env.minimum_turn_radius_m);
    assert(strong_env.radius_margin_m > weak_env.radius_margin_m);
}

static void test_terminal_turn_response_delay_is_actuator_derived(void) {
    EntryLateralTurnInput fast_roll = terminal_input(1.0);
    fast_roll.current_along_m -= 100000.0;
    fast_roll.measured_bank_deg = -50.0;
    fast_roll.maximum_roll_rate_deg_s = 16.0;
    EntryLateralTurnEnvelope fast_env =
        entry_lateral_terminal_turn_envelope(&fast_roll);
    assert(fast_env.valid);

    EntryLateralTurnInput slow_roll = fast_roll;
    slow_roll.maximum_roll_rate_deg_s = 6.0;
    EntryLateralTurnEnvelope slow_env =
        entry_lateral_terminal_turn_envelope(&slow_roll);
    assert(slow_env.valid);
    assert(slow_env.response_time_s > fast_env.response_time_s);
    assert(slow_env.response_distance_m > fast_env.response_distance_m);
}

static void test_terminal_turn_mirror_symmetry(void) {
    EntryLateralTurnInput right = terminal_input(1.0);
    EntryLateralTurnInput left = terminal_input(-1.0);
    EntryLateralTurnEnvelope right_env =
        entry_lateral_terminal_turn_envelope(&right);
    EntryLateralTurnEnvelope left_env =
        entry_lateral_terminal_turn_envelope(&left);

    assert(right_env.valid && left_env.valid);
    assert(right_env.turn_sign == -left_env.turn_sign);
    assert(fabs(right_env.minimum_turn_radius_m -
        left_env.minimum_turn_radius_m) < 1e-9);
    assert(fabs(right_env.ideal_turn_radius_m -
        left_env.ideal_turn_radius_m) < 1e-6);
    assert(fabs(right_env.endpoint_error_m -
        left_env.endpoint_error_m) < 1e-6);
    assert(fabs(right_env.feasibility_margin_m -
        left_env.feasibility_margin_m) < 1e-6);
}

static void test_capture_contract_and_uncertainty_reduce_margin(void) {
    EntryLateralTurnInput input = terminal_input(1.0);
    EntryLateralTurnEnvelope baseline =
        entry_lateral_terminal_turn_envelope(&input);
    assert(baseline.valid);

    input.position_uncertainty_m = 400.0;
    EntryLateralTurnEnvelope uncertain =
        entry_lateral_terminal_turn_envelope(&input);
    assert(uncertain.valid);
    assert(fabs((baseline.capture_margin_m -
        uncertain.capture_margin_m) - 400.0) < 1e-9);

    input.capture_radius_m = 500.0;
    EntryLateralTurnEnvelope tighter =
        entry_lateral_terminal_turn_envelope(&input);
    assert(tighter.valid);
    assert(fabs((uncertain.capture_margin_m -
        tighter.capture_margin_m) - 500.0) < 1e-9);
}

static void test_geometry_turn_sign_is_mirror_symmetric(void) {
    double right = entry_lateral_geometry_turn_sign(
        0.0, 0.0, 0.0, 100000.0, 30000.0, 90.0);
    double left = entry_lateral_geometry_turn_sign(
        0.0, 0.0, 0.0, 100000.0, -30000.0, -90.0);
    assert(right > 0.0);
    assert(left < 0.0);
    assert(right == -left);
}

static void test_geometry_turn_sign_uses_terminal_course_at_coincident_point(void) {
    double right = entry_lateral_geometry_turn_sign(
        10.0, 5000.0, -2000.0, 5000.0, -2000.0, 90.0);
    double left = entry_lateral_geometry_turn_sign(
        -10.0, 5000.0, 2000.0, 5000.0, 2000.0, -90.0);
    assert(right > 0.0);
    assert(left < 0.0);
}

int main(void) {
    test_vertical_lift_conversion();
    test_allocator_preserves_owned_side_and_rate_limits();
    test_terminal_turn_speed_increases_required_radius();
    test_terminal_turn_more_lift_improves_radius_margin();
    test_terminal_turn_response_delay_is_actuator_derived();
    test_terminal_turn_mirror_symmetry();
    test_geometry_turn_sign_is_mirror_symmetric();
    test_geometry_turn_sign_uses_terminal_course_at_coincident_point();
    test_capture_contract_and_uncertainty_reduce_margin();
    puts("Entry lateral tests passed.");
    return 0;
}
