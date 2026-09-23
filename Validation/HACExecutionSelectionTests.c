#include "../CLanding/guidance.c"
#include <assert.h>
#include <stdio.h>

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.latitude=0;cfg.site.longitude=0;cfg.site.altitude=70;
    cfg.site.runway_heading=90;
    PlanetModel p={.radius=600000,.gravitational_parameter=3531600000000};
    AerodynamicModel aero={.lift_to_drag=1,.ballistic_coefficient=800,.confidence=1};
    GuidanceMachine g;guidance_machine_init(&g);
    terminal_glide_initialize(&g,&cfg.vehicle,&cfg.guidance);
    Telemetry t;telemetry_init(&t);
    t.ut=100;t.mean_altitude=20000;t.radar_altitude=19930;
    t.mass=40252.9;t.true_air_speed=750;t.horizontal_speed=720;
    t.vertical_speed=-100;t.flight_path_angle=asin(-100.0/750)*RAD2DEG;
    t.bank_effectiveness=1;t.lift_force=t.mass*10;t.drag_force=t.mass*2;
    t.angle_of_attack=18;t.dynamic_pressure=10000;t.mach=2.5;
    GeoPoint origin={0,0,70};
    GeoPoint pos=local_point(origin,-cfg.guidance.final_approach_distance,0,p.radius,20000);
    t.latitude=pos.latitude;t.longitude=pos.longitude;
    assert(terminal_default_hac_side(&t,&cfg.site,180)==1.0);
    assert(terminal_default_hac_side(&t,&cfg.site,0)==-1.0);
    TerminalCandidate south={.valid=true,.kind=TERMINAL_PATH_HAC,.side=1,
        .radius=10000,.final_distance=cfg.guidance.final_approach_distance,
        .slope=10,.response=4,.speed=750,.selected_ut=100,.arrival_ut=100};
    south.join=(HACTransitionPlan){.valid=true,.lead_start={-south.final_distance,0},
        .p0={-south.final_distance,-2880},.p1={-south.final_distance,-5000},
        .p2={-south.final_distance+5000,-10000},.p3={-south.final_distance+10000,-10000},
        .length=15000,.lead_length=2880};
    TerminalCandidate north=south;north.side=-1;
    /* Same current-course response lead: only circle acquisition differs. */
    terminal_candidate_execution_cost(&south,&g,&t,180,&p,aero,&cfg);
    terminal_candidate_execution_cost(&north,&g,&t,180,&p,aero,&cfg);
    assert(south.execution_radial_closure>0&&north.execution_radial_closure<0);
    assert(terminal_candidate_better(&north,&south));
    assert(!terminal_candidate_better(&south,&north));
    double south_time=south.execution_time,south_margin=south.execution_margin;
    south.join.p0.n*=-1;south.join.p1.n*=-1;
    north.join=south.join;
    terminal_candidate_execution_cost(&south,&g,&t,0,&p,aero,&cfg);
    terminal_candidate_execution_cost(&north,&g,&t,0,&p,aero,&cfg);
    assert(terminal_candidate_better(&south,&north));
    assert(fabs(north.execution_time-south_time)<1e-8);
    assert(fabs(north.execution_margin-south_margin)<1e-8);
    puts("PASS southbound +1, northbound -1, away-side loses, mirrored cost symmetry");

    TerminalCandidate impossible=north;
    impossible.join.lead_start.e+=1000000;
    impossible.join.length=1;impossible.join.violation_score=0;
    terminal_candidate_execution_cost(&impossible,&g,&t,0,&p,aero,&cfg);
    assert(impossible.execution_margin<0&&impossible.energy_degraded&&impossible.degraded);
    assert(!terminal_candidate_better(&north,&impossible));
    assert(terminal_candidate_better(&impossible,&north));
    puts("PASS downstream path quality cannot override impossible execution lead");

    TerminalCandidate rival=north;rival.side=1;rival.execution_time-=1;
    rival.join.length=1;rival.selected_ut=200;
    for(int i=0;i<100;i++){
        rival.selected_ut+=1;
        assert(!terminal_candidate_refresh_allowed(&g,&north,&rival));
    }
    rival.side=north.side;
    rival.execution_time=north.execution_time-north.response-1;
    assert(terminal_candidate_refresh_allowed(&g,&north,&rival));
    assert(!terminal_candidate_refresh_allowed(&g,&rival,&north));
    puts("PASS expiry cannot churn mirror sides; improvement must pay response cost");
    return 0;
}
