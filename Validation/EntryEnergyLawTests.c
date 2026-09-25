#include "entry_energy_control.h"
#include "landing.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

static VehicleProfile test_vehicle(void) {
    VehicleProfile v;
    memset(&v, 0, sizeof(v));
    v.maximum_dynamic_pressure = 45000.0;
    v.maximum_g_load = 2.5;
    v.estimated_lift_to_drag = 1.2;
    v.estimated_ballistic_coefficient = 180.0;
    v.touchdown_speed = 65.0;
    return v;
}

static EntryEnergyInput test_input(const PlanetModel *p, const VehicleProfile *v) {
    EntryEnergyInput in;
    memset(&in, 0, sizeof(in));
    in.ut = 1000.0;
    in.relative_velocity = 1800.0;
    in.altitude = 38000.0;
    in.latitude = 0.0;
    in.flight_path_angle_deg = -2.5;
    in.vertical_speed = in.relative_velocity * sin(in.flight_path_angle_deg * DEG2RAD);
    in.measured_drag_accel = 6.0;
    in.measured_lift_accel = 7.2;
    in.dynamic_pressure = 2500.0;
    in.range_to_go = 150000.0;
    in.taem_velocity = 750.0;
    in.taem_altitude = 22000.0;
    in.taem_latitude = 0.0;
    in.transition_velocity = 950.0;
    in.maximum_bank_deg = 70.0;
    in.integrator_hold = false;
    in.planet = p;
    in.vehicle = v;
    return in;
}

static void test_drag_profile_solve_monotonic_in_range(const PlanetModel *p, const VehicleProfile *v) {
    EntryEnergyConfig cfg = entry_energy_default_config();
    const double ranges[] = {
        60000.0, 90000.0, 130000.0, 180000.0, 240000.0, 320000.0
    };
    const size_t n = sizeof(ranges) / sizeof(ranges[0]);
    double last_drag = 1e9;
    double last_pred_range = 0.0;

    for (size_t i = 0; i < n; ++i) {
        EntryEnergyState state;
        entry_energy_reset(&state);
        EntryEnergyInput in = test_input(p, v);
        in.range_to_go = ranges[i];

        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);

        /* Monotonicity: higher range required must demand lower or equal drag */
        assert(out.profile_drag_accel <= last_drag + 1e-9);
        /* Monotonicity: higher range required must predict longer or equal range */
        assert(out.predicted_range_m >= last_pred_range - 1e-9);

        if (!out.profile_at_minimum && !out.profile_at_maximum) {
            /* Inside feasible span, solver achieves required range within bisection tolerance */
            assert(fabs(out.range_error_m) < 2.0);
        }

        last_drag = out.profile_drag_accel;
        last_pred_range = out.predicted_range_m;
    }
}

static void test_drag_profile_saturation_flags(const PlanetModel *p, const VehicleProfile *v) {
    EntryEnergyConfig cfg = entry_energy_default_config();

    /* Minimum drag saturation: range to go is far beyond reachable gliding distance */
    {
        EntryEnergyState state;
        entry_energy_reset(&state);
        EntryEnergyInput in = test_input(p, v);
        in.range_to_go = 1e9; /* Exceeds maximum gliding range at minimum drag */

        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(out.profile_at_minimum);
        assert(!out.profile_at_maximum);
        assert(fabs(out.profile_drag_accel - cfg.minimum_drag_accel) < 1e-6);
        assert(out.range_error_m < 0.0); /* shortfall: cannot stretch that far */
    }

    /* Maximum drag saturation: range to go is tiny, requiring extreme deceleration */
    {
        EntryEnergyState state;
        entry_energy_reset(&state);
        EntryEnergyInput in = test_input(p, v);
        in.range_to_go = 500.0; /* 500 m */

        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(out.profile_at_maximum);
        assert(!out.profile_at_minimum);
        assert(fabs(out.profile_drag_accel - out.maximum_drag_accel) < 1e-6);
        assert(out.range_error_m > 0.0); /* overshoot: cannot stop that fast */
    }
}

static void test_lift_and_bank_saturation_flags(const PlanetModel *p, const VehicleProfile *v) {
    EntryEnergyConfig cfg = entry_energy_default_config();

    /* Saturated lift up: severe sink requiring maximum vertical lift pull-up */
    {
        EntryEnergyState state;
        entry_energy_reset(&state);
        EntryEnergyInput in = test_input(p, v);
        in.vertical_speed = -250.0;
        in.flight_path_angle_deg = -12.0;

        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(out.saturated_lift_up);
        assert(fabs(out.bank_magnitude_deg - 0.0) < 1e-6);
    }

    /* Saturated lift up via structural / dynamic pressure protection */
    {
        EntryEnergyState state;
        entry_energy_reset(&state);
        EntryEnergyInput in = test_input(p, v);
        in.dynamic_pressure = 46000.0; /* Exceeds protection fraction * max q */

        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(out.protection_active);
        assert(out.saturated_lift_up);
        assert(fabs(out.bank_magnitude_deg - 0.0) < 1e-6);
    }

    /* Saturated bank limit: vehicle high and climbing with excess energy,
       calling for steep bank to drop down */
    {
        EntryEnergyState state;
        entry_energy_reset(&state);
        EntryEnergyInput in = test_input(p, v);
        in.altitude = 48000.0;
        in.vertical_speed = 50.0;
        in.flight_path_angle_deg = 2.0;
        in.range_to_go = 30000.0;

        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(out.saturated_bank_limit);
        assert(fabs(out.bank_magnitude_deg - in.maximum_bank_deg) < 1e-6);
    }
}

static void test_integrator_hold_during_reversals(const PlanetModel *p, const VehicleProfile *v) {
    EntryEnergyConfig cfg = entry_energy_default_config();
    EntryEnergyState state;
    entry_energy_reset(&state);

    EntryEnergyInput in = test_input(p, v);
    /* Set drag and lift close to reference so altitude error is modest and the bank command
       operates in the linear, unsaturated range (0 < bank < max_bank). */
    in.measured_drag_accel = 8.2;
    in.measured_lift_accel = 9.84;
    in.maximum_bank_deg = 80.0;
    in.vertical_speed = in.relative_velocity * sin(in.flight_path_angle_deg * DEG2RAD);

    /* Step 1: Run several ticks with integrator active to accumulate integral */
    double dt = 0.5;
    for (int step = 0; step < 15; ++step) {
        in.ut += dt;
        in.integrator_hold = false;
        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(!out.saturated_lift_up && !out.saturated_bank_limit);
    }

    double frozen_integral = state.integral_m_s;
    assert(fabs(frozen_integral) > 5.0);

    /* Step 2: Set integrator_hold = true (simulating an active S-turn bank reversal).
       Verify that the integral is exactly held constant across all ticks. */
    for (int step = 0; step < 20; ++step) {
        in.ut += dt;
        in.integrator_hold = true;
        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
        assert(fabs(out.integral_m_s - frozen_integral) < 1e-12);
        assert(fabs(state.integral_m_s - frozen_integral) < 1e-12);
    }

    /* Step 3: Clear integrator_hold = false. Integration should resume. */
    for (int step = 0; step < 10; ++step) {
        in.ut += dt;
        in.integrator_hold = false;
        EntryEnergyOutput out = entry_energy_update(&state, &in, &cfg);
        assert(out.valid);
    }
    assert(fabs(state.integral_m_s - frozen_integral) > 1.0);
}

int main(void) {
    PlanetModel p = test_planet();
    VehicleProfile v = test_vehicle();

    test_drag_profile_solve_monotonic_in_range(&p, &v);
    test_drag_profile_saturation_flags(&p, &v);
    test_lift_and_bank_saturation_flags(&p, &v);
    test_integrator_hold_during_reversals(&p, &v);

    puts("Entry energy law unit tests passed.");
    return 0;
}
