#include "../CLanding/guidance.c"
#include <assert.h>
#include <stdio.h>

static Telemetry entry_sample(void){
    Telemetry t;telemetry_init(&t);
    t.ut=100;t.latitude=0;t.longitude=-118;t.mean_altitude=49693;t.radar_altitude=49693;
    t.true_air_speed=2059;t.horizontal_speed=2058;t.surface_speed=2059;
    t.vertical_speed=-57;t.flight_path_angle=-1.6;t.ground_track_heading=90;t.heading=90;
    t.mass=40231;t.dynamic_pressure=500;t.g_force=.10;t.mach=6.5;
    t.angle_of_attack=27;t.sideslip=0;t.bank_effectiveness=1;
    t.attitude_response.pitch_valid=true;
    t.attitude_response.maximum_pitch_rate_deg_s=18.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=4.0;
    t.attitude_response.roll_valid=true;
    t.attitude_response.maximum_roll_rate_deg_s=4.5;
    t.attitude_response.maximum_roll_accel_deg_s2=1.8;
    t.lift_force=t.mass*.455;t.drag_force=t.mass*.80;
    t.runway_along_track=-455000;t.runway_cross_track=0;t.range_to_site=455000;
    return t;
}
static GuidanceMachine entry_machine(void){
    GuidanceMachine g;guidance_machine_init(&g);g.automation_engaged=true;
    g.phase=PHASE_ENTRY_ENERGY;g.s_turn_sign=1;g.terminal_prediction_ut=90;
    g.taem_interface_target.valid=true;g.taem_interface_target.along_track=-32879;
    g.taem_interface_target.cross_track=12000;g.taem_interface_target.altitude=21800;
    g.taem_interface_target.speed=775.1;g.taem_interface_target.course=90;
    g.taem_interface_target.flight_path_angle=-9.8;
    g.taem_interface_target.acquisition_lead=15000;g.taem_interface_target.response_time=6;
    return g;
}
static PlanetModel test_planet(void){
    PlanetModel p={.radius=600000,.gravitational_parameter=3531600000000,
        .rotational_speed=.000291570900559802,.atmosphere_depth=70000,
        .surface_density=1.14,.atmosphere_adiabatic_index=1.4,
        .atmosphere_sample_count=2};
    p.atmosphere_altitude[0]=0.0;p.atmosphere_altitude[1]=70000.0;
    p.atmosphere_pressure[0]=101325.0;p.atmosphere_pressure[1]=0.1;
    p.atmosphere_density[0]=1.14;p.atmosphere_density[1]=0.0001;
    return p;
}
static void bank_allocation_test(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    PlanetModel p=test_planet();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=800,.confidence=1};
    GuidanceMachine g=entry_machine();Telemetry t=entry_sample();
    double east=entry_program_vertical_bank_ceiling(&g,&t,&p,aero,&cfg);
    p.rotational_speed=0;
    double stationary=entry_program_vertical_bank_ceiling(&g,&t,&p,aero,&cfg);
    assert(isfinite(east)&&east>20&&east<=dynamic_bank_limit(&t,&cfg.vehicle));
    assert(stationary<east);
    p.rotational_speed=.000291570900559802;
    t.flight_path_angle=-8;t.vertical_speed=t.true_air_speed*sin(-8*DEG2RAD);
    double steep=entry_program_vertical_bank_ceiling(&g,&t,&p,aero,&cfg);
    assert(steep<east);
    printf("PASS: measured 50 km bank allocation east %.2f, nonrotating %.2f, steep %.2f deg.\n",east,stationary,steep);
}
static void publication_test(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    GuidanceMachine live=entry_machine(),request=live,result=request;Telemetry t=entry_sample();
    result.terminal_prediction_ut=99;result.terminal_candidate.valid=true;
    result.terminal_candidate.arrival_ut=125;result.terminal_candidate.altitude=25000;
    result.terminal_candidate.speed=850;result.terminal_candidate.radius=12000;
    result.terminal_prediction_altitude=25000;result.terminal_prediction_speed=850;
    result.terminal_prediction_time=26;result.taem_interface_target.along_track=99999;
    result.phase=PHASE_COMPLETE;result.terminal_path_committed=true;
    assert(guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    assert(live.terminal_candidate.valid&&live.terminal_candidate.radius==12000);
    assert(live.phase==PHASE_ENTRY_ENERGY&&!live.terminal_path_committed);
    assert(live.taem_interface_target.along_track==-32879);
    assert(!guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    live=request;t.ut=110;assert(!guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    t.ut=100;live=request;live.attitude_recovery=true;
    assert(!guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    live=request;live.terminal_path_committed=true;
    assert(!guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    live=request;live.phase=PHASE_TAEM;
    assert(!guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    live=request;live.s_turn_sign=-1;
    assert(!guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    live=request;result.terminal_candidate.arrival_ut=80;
    assert(guidance_accept_terminal_preview(&live,&request,&result,&t,&cfg));
    assert(!live.terminal_candidate.valid);
    puts("PASS: fresh publication; stale/recovery/commit/phase/reversal rejection; entry ownership and inlet preserved.");
}
static void deferred_search_test(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    PlanetModel p=test_planet();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=800,.confidence=1};
    GuidanceMachine g=entry_machine();Telemetry t=entry_sample();
    g.terminal_planning_deferred=true;
    terminal_predict(&g,&t,90,&p,aero,&cfg,.1);
    assert(g.terminal_prediction_ut==90);
    assert(g.terminal_prediction_valid);
    assert(!g.terminal_candidate.valid);
    puts("PASS: live reference update does not execute a deferred candidate search.");
}
static TerminalCandidate candidate_fixture(double violation,bool geometry_degraded,
        double path_length,double response){
    TerminalCandidate c={0};
    c.valid=true;c.kind=TERMINAL_PATH_SPLINE;c.geometry_degraded=geometry_degraded;
    c.degraded=geometry_degraded||violation>0.0;
    c.speed=200.0;c.selected_ut=0.0;c.arrival_ut=10.0;c.response=response;
    c.join.valid=true;c.join.length=path_length;c.join.violation_score=violation;
    return c;
}

static void candidate_margin_order_test(void){
    TerminalCandidate retained=candidate_fixture(.20,false,1000.0,2.0);
    TerminalCandidate lower_violation=candidate_fixture(.10,false,8000.0,2.0);
    assert(terminal_candidate_better(&retained,&lower_violation));

    TerminalCandidate unexecutable=candidate_fixture(.01,true,500.0,1.0);
    assert(!terminal_candidate_better(&lower_violation,&unexecutable));

    TerminalCandidate feasible=candidate_fixture(0.0,false,12000.0,2.0);
    assert(terminal_candidate_better(&lower_violation,&feasible));

    double numerical=sqrt(DBL_EPSILON);
    TerminalCandidate near_a=candidate_fixture(.10,false,5000.0,2.0);
    TerminalCandidate near_b=candidate_fixture(.10+.25*numerical,false,1000.0,2.0);
    assert(terminal_candidate_constraint_order(&near_a,&near_b)==0);
    assert(terminal_candidate_better(&near_a,&near_b));
    puts("PASS: terminal candidates rank by hard geometry executability, normalized physical margin, then delivery time.");
}

static void energy_uncertainty_monotonicity_test(void){
    LandingConfiguration cfg=landing_configuration_default();landing_configuration_normalize(&cfg);
    PlanetModel p=test_planet();
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=800,.confidence=1};
    GuidanceMachine g=entry_machine();Telemetry t=entry_sample();
    t.mean_altitude=26000;t.radar_altitude=26000;t.true_air_speed=900;
    t.horizontal_speed=870;t.vertical_speed=-230;t.flight_path_angle=-14.8;
    t.mach=3.0;t.drag_force=t.mass*4.0;t.angle_of_attack=12.0;
    t.physics_certified_uncertainty=0.0;t.predicted_physics_relative_uncertainty=0.0;
    t.trajectory_altitude_residual=0.0;t.trajectory_speed_residual=0.0;
    t.trajectory_range_residual=0.0;

    double slope=cfg.guidance.taem_glide_slope;
    double minimum_path=5000.0,maximum_path=50000.0;
    double target_aoa=fmin(t.angle_of_attack,cfg.vehicle.maximum_angle_of_attack);
    double minimum_work=terminal_projected_drag_work(
        &g,&t,&p,aero,&cfg.vehicle,&cfg.guidance,target_aoa,minimum_path,slope);
    assert(isfinite(minimum_work)&&minimum_work>0.0);

    double gate_altitude=cfg.site.altitude+cfg.guidance.flare_altitude;
    double available=entry_remaining_specific_energy(
        t.latitude,t.mean_altitude,t.true_air_speed,
        cfg.site.latitude,gate_altitude,cfg.vehicle.final_approach_speed,&p);
    assert(isfinite(available)&&available>minimum_work);

    EnergyPathEnvelope baseline={0},uncertain={0},richer={0};
    double baseline_path=terminal_integrated_energy_path(
        &g,&t,&p,aero,&cfg.vehicle,&cfg.guidance,target_aoa,available,
        slope,minimum_path,maximum_path,&baseline);
    assert(isfinite(baseline_path)&&baseline.valid);

    t.physics_certified_uncertainty=.25;
    double uncertain_path=terminal_integrated_energy_path(
        &g,&t,&p,aero,&cfg.vehicle,&cfg.guidance,target_aoa,available,
        slope,minimum_path,maximum_path,&uncertain);
    assert(isfinite(uncertain_path)&&uncertain.valid);
    assert(uncertain_path<=baseline_path+
        sqrt(DBL_EPSILON)*fmax(1.0,baseline_path));

    t.physics_certified_uncertainty=0.0;
    double richer_path=terminal_integrated_energy_path(
        &g,&t,&p,aero,&cfg.vehicle,&cfg.guidance,target_aoa,
        available+minimum_work,slope,minimum_path,maximum_path,&richer);
    assert(isfinite(richer_path)&&richer.valid);
    assert(richer_path+sqrt(DBL_EPSILON)*fmax(1.0,baseline_path)>=baseline_path);
    puts("PASS: terminal energy-path reach decreases with certified uncertainty and increases with available energy.");
}
int main(void){
    candidate_margin_order_test();energy_uncertainty_monotonicity_test();
    bank_allocation_test();publication_test();deferred_search_test();
    printf("Native snapshot sizes: guidance=%zu physics=%zu bytes.\n",sizeof(GuidanceMachine),sizeof(VesselPhysicsModel));
    return 0;
}
