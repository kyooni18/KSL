#include "../../CLanding/landing.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static double monotonic_seconds(void){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return (double)ts.tv_sec+(double)ts.tv_nsec/1e9;
}

static PlanetModel kerbin(void){
    PlanetModel p;memset(&p,0,sizeof(p));
    snprintf(p.name,sizeof(p.name),"Kerbin");
    p.radius=600000;p.gravitational_parameter=3.5316e12;
    p.rotational_speed=2*LANDER_PI/21549.425;p.atmosphere_depth=70000;
    p.surface_density=1.225;p.atmosphere_adiabatic_index=1.4;
    p.north_axis=v3(0,0,1);p.prime_meridian_at_epoch=v3(1,0,0);
    return p;
}

static TaemTerminalContract terminal_contract(void){
    TaemTerminalContract c;memset(&c,0,sizeof(c));
    c.valid=true;c.path_committed=true;c.range_to_go=8066.5541;c.range_margin=1000;
    c.dynamic_pressure=15774.576;c.dynamic_pressure_margin=10000;
    c.speedbrake_available=true;c.speedbrake_dynamic_pressure_margin=8000;
    c.altitude=23895.774;c.altitude_margin=2000;c.flight_path_angle=-8.1887;c.flight_path_angle_margin=4;
    c.specific_energy=-4832579.409;c.specific_energy_margin=50000;
    c.response_time_available=20;c.response_time_required=5;c.attitude_response_qualified=true;
    return c;
}

int main(void){
    PlanetModel p=kerbin();
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    AerodynamicModel aero={.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=.6};
    AerodynamicEnvelope env;for(int i=0;i<4;i++)env.regimes[i]=aero;
    TrajectoryCalibrationModel cal={.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,
        .speed_of_sound=340,.speed_of_sound_scale=1,.stress_drag_scale=1,.stress_lift_scale=1,.confidence=.2};

    VehicleState state={
        .ut=67587.9725486172,
        .position={269539.580867591,562596.558715328,-8914.19295280005},
        .velocity={-1403.70460955308,466.571442407433,-41.7309589006274},
        .mass=40251.265625
    };
    Telemetry t;telemetry_init(&t);
    t.ut=state.ut;t.latitude=-0.818667211542707;t.longitude=-74.7089826571486;
    t.mean_altitude=23895.7736422049;t.radar_altitude=t.mean_altitude;
    t.true_air_speed=1299.63354492188;t.horizontal_speed=1286.38294821543;t.vertical_speed=-185.111794769201;
    t.flight_path_angle=-8.188706299173;t.roll=-58.058837890625;t.heading=84.3483276367188;
    t.ground_track_heading=t.heading;t.angle_of_attack=9.11435508728027;t.sideslip=-.327165096998215;
    t.dynamic_pressure=15774.576171875;t.static_pressure=1207.10778808594;t.atmospheric_density=.0186786666007091;
    t.mach=4.30836985152215;t.g_force=1.33650612831116;t.lift_force=238764.281237821;t.drag_force=473914.95200155;
    t.mass=state.mass;t.range_to_site=8066.55410024219;t.runway_along_track=202.639733864633;t.runway_cross_track=8064.00862667604;
    t.has_body_roll_rate=true;t.body_roll_rate=1.98674056207559;t.has_body_pitch_rate=true;t.body_pitch_rate=.0771418921581125;
    t.physics_certified_uncertainty=.15;t.trajectory_density_scale=1;t.trajectory_drag_scale=1;t.trajectory_lift_scale=1;t.bank_effectiveness=1;

    GuidanceMachine g;guidance_machine_init(&g);g.automation_engaged=true;g.deorbit_burn_completed=true;
    g.atmospheric_interface_crossed=true;g.terminal_region_entered=true;g.terminal_glide_mode=true;g.phase=PHASE_TAEM;
    TaemExecObservation observation={.ut=state.ut,.relative_velocity=t.true_air_speed,.checkpoint_restart=false};
    TaemExecInputs inputs;memset(&inputs,0,sizeof(inputs));inputs.mm304_complete=true;inputs.energy_valid=true;inputs.energy_excess=60923.422;
    inputs.terminal_feasibility_valid=true;inputs.nominal_terminal_path_feasible=true;inputs.terminal_contract=terminal_contract();
    TaemExecProfile profile={.s_turn_enabled=true,.s_turn_entry_energy_excess=100,.s_turn_termination_energy_excess=25};
    if(!taem_exec_initialize(&g.taem_exec,&observation,&inputs,&profile)){fprintf(stderr,"TAEM init failed\n");return 2;}

    const double horizons[]={60,360,600};
    for(size_t i=0;i<sizeof(horizons)/sizeof(horizons[0]);i++){
        double start=monotonic_seconds();
        EntryPrediction pr=predictor_simulate_terminal_shadow_ensemble(state,&t,&g,NULL,&p,aero,&env,&cal,&cfg,horizons[i],true);
        double wall=monotonic_seconds()-start;
        printf("horizon=%.0f wall=%.6f trajectory=%zu scenarios=%u final_dt=%.3f aborted=%d feasible=%d\n",
            horizons[i],wall,pr.trajectory.count,pr.uncertainty_scenarios,pr.final_state.ut-state.ut,
            pr.shadow_aborted?1:0,pr.shadow_terminal_policy_feasible?1:0);
        entry_prediction_clear(&pr);
    }
    return 0;
}
