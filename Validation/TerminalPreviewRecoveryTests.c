#include "../CLanding/guidance.c"
#include <assert.h>
#include <stdio.h>

static void test_pitch_response_authority_moves_flare_envelope_monotonically(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    Telemetry t;telemetry_init(&t);
    t.angle_of_attack=10.0;
    t.angle_of_attack_rate=0.0;
    t.has_angle_of_attack_rate=true;
    t.vertical_speed=-20.0;
    t.attitude_response.pitch_valid=true;
    t.attitude_response.pitch_natural_frequency_s_inv=1.4;
    t.attitude_response.pitch_damping_ratio=.9;
    t.attitude_response.maximum_pitch_rate_deg_s=20.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=10.0;

    GuidanceMachine g={0};
    g.terminal_preflare_plan_valid=true;
    g.preflare_target_aoa=20.0;
    g.preflare_effective_accel=2.0;

    double fast_time=decision_pitch_capture_time(&t,g.preflare_target_aoa);

    t.attitude_response.maximum_pitch_rate_deg_s=10.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=5.0;
    double slow_time=decision_pitch_capture_time(&t,g.preflare_target_aoa);

    assert(isfinite(fast_time)&&isfinite(slow_time)&&slow_time>fast_time);
    puts("PASS: weaker pitch authority monotonically increases pitch capture time.");
}

int main(void){
    test_pitch_response_authority_moves_flare_envelope_monotonically();
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    Telemetry t;telemetry_init(&t);t.mass=41134.9453125;t.drag_force=t.mass*11.414;
    t.lift_force=500;t.dynamic_pressure=27000;t.mach=1.2;t.angle_of_attack=.1;t.sideslip=0;
    AerodynamicModel aero={.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=.6};
    assert(isnan(terminal_drag_budget_aoa(&t,aero,&cfg.vehicle,6.335,20)));
    assert(isnan(terminal_drag_budget_aoa(&t,aero,&cfg.vehicle,0,20)));
    assert(isfinite(terminal_drag_budget_aoa(&t,aero,&cfg.vehicle,25,20)));
    assert(fabs(terminal_preview_recovery_aoa(20,NAN,-22.86,-47.57)-20)<1e-9);
    assert(fabs(terminal_preview_recovery_aoa(14,NAN,-23,-10))<1e-9);
    assert(fabs(terminal_preview_recovery_aoa(14,4,-23,-10)-4)<1e-9);
    assert(fabs(terminal_preview_recovery_aoa(14,NAN,-23,-23)-7)<1e-9);
    double previous=0;
    for(double error=-10;error<=10;error+=.01){
        double value=terminal_preview_recovery_aoa(14,NAN,-23,-23-error);
        assert(value>=previous-1e-8&&value>=0&&value<=14+1e-8);previous=value;
    }
    assert(terminal_preview_recovery_aoa(20,0,-40,-38)>19.99);
    puts("PASS: empty-budget detection, recorded steep-dive recovery, shallow energy authority, smooth transition, and -35 degree envelope.");
    return 0;
}
