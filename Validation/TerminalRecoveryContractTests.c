#include "../CLanding/guidance.c"

#include <assert.h>
#include <stdio.h>

static Telemetry recovery_state(const LandingConfiguration *cfg) {
    Telemetry t;
    telemetry_init(&t);
    t.mean_altitude = cfg->site.altitude + 1500.0;
    t.radar_altitude = 1500.0;
    t.true_air_speed = 150.0;
    t.vertical_speed = -30.0;
    t.horizontal_speed = sqrt(t.true_air_speed*t.true_air_speed-t.vertical_speed*t.vertical_speed);
    t.flight_path_angle = atan2(t.vertical_speed,t.horizontal_speed)*RAD2DEG;
    t.runway_along_track = -9000.0;
    t.mass = 40000.0;
    t.lift_force = 700000.0;
    t.drag_force = 1000.0;
    t.dynamic_pressure = 6000.0;
    t.g_force = 1.8;
    t.angle_of_attack = 10.0;
    t.mach = 0.45;
    t.bank_effectiveness = 1.0;
    t.attitude_response.pitch_valid = true;
    t.attitude_response.maximum_pitch_accel_deg_s2 = 5.0;
    t.attitude_response.maximum_pitch_rate_deg_s = 10.0;
    t.attitude_response.roll_valid = true;
    t.attitude_response.maximum_roll_accel_deg_s2 = 5.0;
    t.attitude_response.maximum_roll_rate_deg_s = 10.0;
    return t;
}

int main(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = {.radius=600000.0, .gravitational_parameter=3.5316e12};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.terminal_path_committed=true;
    g.final_approach_captured=true;
    Telemetry t=recovery_state(&cfg);
    AerodynamicModel aero={.lift_to_drag=cfg.vehicle.estimated_lift_to_drag,
        .ballistic_coefficient=cfg.vehicle.estimated_ballistic_coefficient};
    TerminalPreflarePlan base=terminal_preflare_plan(&g,&t,&p,aero,&cfg);
    assert(base.feasible && base.height.valid && base.height.margin>0.0);
    RunwayCaptureEnvelope runway=decision_runway_capture_envelope(&g,&t,
        cfg.site.runway_heading,&p,&cfg);
    VerticalRecoveryEnvelope shared=decision_vertical_recovery_envelope(
        t.radar_altitude-cfg.guidance.flare_altitude,-t.vertical_speed,
        -cfg.guidance.touchdown_sink_rate,runway.control_response_time_s,
        runway.response_down_accel_mps2,runway.vertical_recovery_accel_mps2);
    assert(shared.valid && shared.height.margin==base.height.margin);
    assert(shared.required_height_m==base.predicted_height_loss);
    double baseline_height=base.predicted_height_loss;

    /* Historical acceleration cannot create current lift authority. */
    g.terminal_sink_accel_ema=1000.0;
    TerminalPreflarePlan stale=terminal_preflare_plan(&g,&t,&p,aero,&cfg);
    assert(stale.predicted_height_loss==baseline_height);

    t.attitude_response.maximum_pitch_accel_deg_s2=
        cfg.guidance.entry_roll_acceleration*.5;
    TerminalPreflarePlan slower=terminal_preflare_plan(&g,&t,&p,aero,&cfg);
    assert(slower.feasible && slower.predicted_height_loss>baseline_height);

    t=recovery_state(&cfg);
    t.roll=60.0;
    TerminalPreflarePlan banked=terminal_preflare_plan(&g,&t,&p,aero,&cfg);
    assert(banked.feasible && banked.predicted_height_loss>baseline_height);
    t.roll=-60.0;
    TerminalPreflarePlan mirrored=terminal_preflare_plan(&g,&t,&p,aero,&cfg);
    assert(mirrored.predicted_height_loss==banked.predicted_height_loss);

    t=recovery_state(&cfg);
    t.radar_altitude=base.trigger_altitude-1.0;
    t.mean_altitude=cfg.site.altitude+t.radar_altitude;
    TerminalPreflarePlan low=terminal_preflare_plan(&g,&t,&p,aero,&cfg);
    assert(low.feasible && low.height.valid && low.height.margin<0.0);
    assert(!terminal_outer_gate(&g,&t,cfg.site.runway_heading,&p,aero,&cfg,NULL));
    TaemTerminalContract contract=terminal_delivery_contract(&g,&t,
        cfg.site.runway_heading,&p,&cfg,&low);
    assert(contract.valid && contract.altitude_margin<0.0);
    assert(!taem_exec_evaluate_terminal_contract(&contract).feasible);

    /* The old guard aborted at required_height + max(100, 2*flare_altitude),
       before its own required_height + flare_altitude transition could fire.
       The switch now occurs while the recovery reserve is still nonnegative. */
    t=recovery_state(&cfg);
    t.radar_altitude=base.trigger_altitude+1.0;
    t.mean_altitude=cfg.site.altitude+t.radar_altitude;
    t.gear=true;
    Trajectory ref;
    trajectory_init(&ref);
    terminal_set_stage(&g,TERMINAL_OUTER_FINAL,t.ut);
    GuidanceResult command=terminal_approach_sequence(&g,&t,cfg.site.runway_heading,
        &p,aero,&cfg,&ref,0.1);
    assert(command.phase!=PHASE_ABORT);
    assert(g.terminal_vertical_stage==TERMINAL_PREFLARE);
    assert(command.command.target_throttle==0.0);
    assert(!command.command.airbrakes);
    guidance_result_clear(&command);

    guidance_machine_init(&g);
    terminal_set_stage(&g,TERMINAL_TRAJECTORY_CAPTURE,t.ut);
    command=terminal_approach_sequence(&g,&t,cfg.site.runway_heading,&p,aero,&cfg,&ref,0.1);
    assert(command.phase==PHASE_ABORT);
    guidance_result_clear(&command);
    trajectory_clear(&ref);

    t=recovery_state(&cfg);
    t.attitude_response.pitch_valid=false;
    assert(!terminal_preflare_plan(&g,&t,&p,aero,&cfg).feasible);


    printf("terminal_recovery_contract_tests: PASS (height %.3f m, slow %.3f m, banked %.3f m)\n",
        baseline_height,slower.predicted_height_loss,banked.predicted_height_loss);
    return 0;
}
