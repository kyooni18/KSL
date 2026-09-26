#include "../CLanding/guidance/guidance_internal.h"

#include <assert.h>
#include <stdio.h>

static void init_native_route_case(GuidanceMachine *g, Telemetry *t,
        double bank_target, double roll_rate) {
    guidance_machine_init(g);
    g->phase = PHASE_TAEM;
    g->mm305_route.valid = true;
    g->mm305_route_committed = true;
    g->roll_limiter.value = bank_target;
    g->roll_limiter.has_value = false; /* committed route bypasses limiter */
    telemetry_init(t);
    t->mass = 40000.0;
    t->dynamic_pressure = 4000.0;
    t->lift_force = t->mass * 4.0;
    t->drag_force = t->mass * 2.0;
    t->roll = 0.0;
    t->roll_rate = roll_rate;
    t->has_body_roll_rate = true;
    t->body_roll_rate = roll_rate;
    t->angle_of_attack = 18.0;
}

static void test_commanded_native_capture_is_not_recovery(void) {
    LandingConfiguration cfg = landing_configuration_default();
    for (int direction = -1; direction <= 1; direction += 2) {
        GuidanceMachine g;
        Telemetry t;
        double target = 69.0 * direction;
        double rate = 12.45 * direction;
        init_native_route_case(&g, &t, target, rate);
        for (int i = 0; i < 8; ++i) {
            t.ut += .06;
            t.roll += rate * .06;
            assert(!control_recovery_needed(&g, &t, &cfg.guidance,
                &cfg.vehicle, .06));
        }
        assert(!g.attitude_recovery);
        assert(!g.roll_limiter.has_value);
    }
}

static void test_uncommanded_energetic_reversals_still_trigger(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    Telemetry t;
    init_native_route_case(&g, &t, 69.0, 12.0);
    g.previous_relative_roll_rate = 12.0;
    g.has_previous_relative_roll_rate = true;
    t.roll = 20.0;
    t.roll_rate = -12.0; /* wrong-way motion relative to +69 deg target */
    t.body_roll_rate = -12.0;
    assert(!control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06));
    t.roll_rate = 12.0;
    t.body_roll_rate = 12.0;
    assert(control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06));
    assert(g.attitude_recovery);
}

int main(void) {
    test_commanded_native_capture_is_not_recovery();
    test_uncommanded_energetic_reversals_still_trigger();
    puts("MM305 recovery monitor tests passed.");
    return 0;
}
