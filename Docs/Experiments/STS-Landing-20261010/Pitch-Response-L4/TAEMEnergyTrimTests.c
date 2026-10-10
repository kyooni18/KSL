#ifndef TAEM_TEST_SOURCE
#define TAEM_TEST_SOURCE "../CLanding/guidance/guidance_taem.c"
#endif
#include TAEM_TEST_SOURCE

#include <assert.h>

int main(void) {
    LandingConfiguration cfg=landing_configuration_default();
    cfg.guidance.taem_energy_law=true;
    TerminalModel model;
    const TerminalModelSourceFiles files={
        .atmosphere_csv="ShuttleSim/reference-model/kerbin_atmosphere_identified.csv",
        .aero_csv="ShuttleSim/reference-model/sts_aero_live20261010.csv",
        .aero_book_csv="none",
        .attitude_ini="ShuttleSim/reference-model/stsn_attitude_identified.ini"
    };
    char reason[192];
    assert(terminal_model_capture(&model,&files,&cfg,1,reason,sizeof(reason)));
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase=PHASE_TAEM;
    g.taem_energy_law=(TaemEnergyLawState){.initialized=true,
        .leg=TEL_LEG_FINAL,.side=1,.radius=5000,.last_geo_ut=100,
        .last_pred_ut=100,.k=.5};
    Telemetry t;
    telemetry_init(&t);
    t.ut=100;t.mass=26000;t.mean_altitude=10070;t.radar_altitude=10000;
    t.mach=.5;t.dynamic_pressure=5000;t.true_air_speed=160;
    t.horizontal_speed=152;t.vertical_speed=-49;t.flight_path_angle=-18;
    t.angle_of_attack=6;t.heading=cfg.site.runway_heading;
    t.runway_along_track=-20000;t.lift_force=t.mass*9.81;
    t.has_controls=true;t.has_angle_of_attack_rate=true;
    TaemGeometryState geo={.runway_along_m=-20000,
        .altitude_above_runway_m=10000,.ground_speed_mps=152,
        .airspeed_mps=160,.flight_path_angle_deg=-18};
    TerminalDynamicState current={.mass_kg=t.mass};
    PlanetModel planet={.radius=600000,.gravitational_parameter=3.5316e12};
    AerodynamicModel aero={0};

    /* Exercise the actual early-return branch, including its cold observer. */
    GuidanceResult r=taem_energy_law_guidance(&g,&t,t.heading,&planet,aero,
        &cfg,&model,&geo,&current,.1);
    assert(r.phase==PHASE_TAEM&&g.mm305_command_valid);
    assert(fabs(g.mm305_command_aoa_rad-r.command.target_aoa*DEG2RAD)<1e-9);
    assert(fabs(g.mm305_command_bank_rad-r.command.target_roll*DEG2RAD)<1e-9);
    assert(g.mm305_command_ut==t.ut&&!r.command.has_pitch_trim_feedforward);

    for(int i=0;i<200;i++) {
        double incidence=4.0+(i%100)*.1;
        adaptive_trim_observe(&g.mm305_trim,80+.1*i,.5,5000,
            incidence,.04*incidence-.06,true);
    }
    r=taem_energy_law_guidance(&g,&t,t.heading,&planet,aero,
        &cfg,&model,&geo,&current,.1);
    assert(r.command.has_pitch_trim_feedforward);
    assert(fabs(r.command.pitch_trim_feedforward-(.04*r.command.target_aoa-.06))<1e-8);
    double weight=g.mm305_trim.weight;
    t.ut+=.1;t.angle_of_attack=r.command.target_aoa;
    t.control_pitch=r.command.pitch_trim_feedforward;
    mm305_trim_observe(&g,&t);
    assert(g.mm305_trim.weight>weight);
    weight=g.mm305_trim.weight;
    t.ut+=.1;t.roll_rate=3;
    mm305_trim_observe(&g,&t);
    assert(g.mm305_trim.weight<weight); /* rolling input is not static trim */
    weight=g.mm305_trim.weight;
    t.ut+=.1;t.roll_rate=0;t.heading_rate=3;
    mm305_trim_observe(&g,&t);
    assert(g.mm305_trim.weight<weight);
    puts("PASS: energy-law commands drive settled trim learning and measured feedforward.");
    return 0;
}
