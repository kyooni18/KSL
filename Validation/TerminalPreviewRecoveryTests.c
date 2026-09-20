#include "../CLanding/guidance.c"
#include <assert.h>
#include <stdio.h>
static void test_terminal_energy_scope(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    GuidanceMachine g={0};g.entry_exec.entry_complete=true;
    g.terminal_candidate.valid=true;g.terminal_candidate.energy_degraded=true;
    Telemetry t;telemetry_init(&t);t.ut=100;t.true_air_speed=270;
    t.mean_altitude=7600;t.energy_excess_range=45000;
    TaemExecInputs inputs=taem_exec_inputs_live(&g,&t,&cfg.guidance);
    TaemExecProfile profile=taem_exec_profile_production(&cfg.guidance);
    TaemExecObservation observation={.ut=t.ut,.relative_velocity=t.true_air_speed};
    TaemExecutive exec={0};
    assert(!inputs.energy_valid);
    assert(taem_exec_initialize(&exec,&observation,&inputs,&profile));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    t.mean_altitude=28500;t.true_air_speed=1100;t.energy_excess_range=101000;
    inputs=taem_exec_inputs_live(&g,&t,&cfg.guidance);
    observation.ut=101;observation.relative_velocity=t.true_air_speed;
    assert(inputs.energy_valid);
    assert(taem_exec_initialize(&exec,&observation,&inputs,&profile));
    assert(exec.phase==TAEM_PHASE_S_TURN);
    t.mean_altitude=cfg.guidance.taem_interface_altitude;
    inputs=taem_exec_inputs_live(&g,&t,&cfg.guidance);observation.ut=102;
    assert(!inputs.energy_valid);
    assert(taem_exec_update(&exec,&observation,&inputs,&profile));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION&&exec.ownership_latched);
    assert(exec.last_transition_reason==TAEM_TRANSITION_S_TURN_ENERGY_INVALID);
    t.mean_altitude=28500;inputs=taem_exec_inputs_live(&g,&t,&cfg.guidance);
    observation.ut=103;assert(taem_exec_update(&exec,&observation,&inputs,&profile));
    assert(exec.phase==TAEM_PHASE_S_TURN);
    t.energy_excess_range=NAN;inputs=taem_exec_inputs_live(&g,&t,&cfg.guidance);
    observation.ut=104;assert(taem_exec_update(&exec,&observation,&inputs,&profile));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION);
    puts("PASS: low-altitude entry proxy rejected; high-altitude S-turn retained; invalid-energy S-turn exits without releasing TAEM.");
}
static void test_fixed_alignment_bearing(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    cfg.site.runway_heading=90;cfg.guidance.final_approach_distance=2000;
    Telemetry t;telemetry_init(&t);
    t.runway_along_track=2185.0;t.runway_cross_track=5686.0;
    double h=terminal_uncommitted_alignment_heading(&t,&cfg);
    assert(h>315&&h<330); /* Northwest, not the observed erroneous northeast. */
    t.runway_along_track=-12000;t.runway_cross_track=12000;
    h=terminal_uncommitted_alignment_heading(&t,&cfg);assert(h>35&&h<45);
    t.runway_along_track=-9000;t.runway_cross_track=-3400;
    h=terminal_uncommitted_alignment_heading(&t,&cfg);assert(h>110&&h<120);
    t.runway_along_track=-9000;t.runway_cross_track=0;
    assert(fabs(terminal_uncommitted_alignment_heading(&t,&cfg)-90)<1e-9);
    puts("PASS: fixed alignment bearing preserves upstream/downstream and both cross-track signs.");
}
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

    double fast_time=pitch_capture_time(&t,g.preflare_target_aoa);
    double fast_height=terminal_touchdown_flare_altitude(&g,&t,&cfg);

    t.attitude_response.maximum_pitch_rate_deg_s=10.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=5.0;
    double slow_time=pitch_capture_time(&t,g.preflare_target_aoa);
    double slow_height=terminal_touchdown_flare_altitude(&g,&t,&cfg);

    assert(isfinite(fast_time)&&isfinite(slow_time)&&slow_time>fast_time);
    assert(isfinite(fast_height)&&isfinite(slow_height)&&slow_height>fast_height);
    assert(fast_height>=cfg.guidance.flare_altitude);
    assert(slow_height>=cfg.guidance.flare_altitude);
    puts("PASS: weaker pitch authority monotonically increases response time and required flare height.");
}

int main(void){
    test_fixed_alignment_bearing();
    test_pitch_response_authority_moves_flare_envelope_monotonically();
    test_terminal_energy_scope();
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
