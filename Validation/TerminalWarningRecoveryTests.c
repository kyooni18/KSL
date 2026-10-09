#include "../CLanding/guidance/guidance_internal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    LandingConfiguration cfg=landing_configuration_default();
    PlanetModel p={.radius=600000,.gravitational_parameter=3531600000000,
        .atmosphere_depth=70000,.surface_density=1.225,.north_axis={0,0,1}};
    AerodynamicModel aero={.lift_to_drag=.6,.ballistic_coefficient=700};
    GuidanceMachine g;guidance_machine_init(&g);
    g.automation_engaged=true;g.terminal_glide_mode=true;
    g.terminal_region_entered=true;g.taem_interface_captured=true;
    g.phase=PHASE_TAEM;
    Telemetry t;telemetry_init(&t);
    t.ut=100;t.heading=cfg.site.runway_heading;t.ground_track_heading=t.heading;
    t.latitude=cfg.site.latitude;t.longitude=cfg.site.longitude-.4;
    t.mean_altitude=24000;t.radar_altitude=24000;t.mass=25816;
    t.true_air_speed=650;t.horizontal_speed=640;t.vertical_speed=-90;
    t.flight_path_angle=-8;t.dynamic_pressure=4000;t.angle_of_attack=10;
    VehicleState state={.ut=t.ut,.position={624000,0,0},.velocity={-90,650,0},.mass=t.mass};
    GuidanceResult r=guidance_update_with_terminal_model(&g,&t,&state,NULL,&p,aero,&cfg,NULL);
    assert(r.phase!=PHASE_ABORT&&r.phase!=PHASE_COMPLETE);
    assert(r.has_warning&&g.automation_engaged&&!g.aborted);
    assert(r.command.autopilot_engaged&&r.command.target_throttle==0);
    assert(isfinite(r.command.target_pitch)&&isfinite(r.command.target_roll));
    assert(fabs(r.command.target_roll)<=cfg.vehicle.maximum_bank_angle);
    assert(!g.mm305_route_committed&&!g.hac_captured);
    guidance_result_clear(&r);

    g.final_approach_captured=true;g.phase=PHASE_ROLLOUT;
    t.latitude=cfg.site.latitude+1;t.mean_altitude=cfg.site.altitude;
    t.radar_altitude=0;t.gear=true;t.main_gear_grounded=true;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"landed");
    t.ut+=.1;
    r=guidance_update_with_terminal_model(&g,&t,&state,NULL,&p,aero,&cfg,NULL);
    assert(r.phase==PHASE_ROLLOUT&&r.has_warning&&r.command.brakes);
    assert(g.automation_engaged&&!g.aborted&&g.terminal_landing_warning_latched);
    guidance_result_clear(&r);

    guidance_abort(&g);t.ut+=.1;
    r=guidance_update_with_terminal_model(&g,&t,&state,NULL,&p,aero,&cfg,NULL);
    assert(r.phase==PHASE_ABORT&&g.aborted&&!g.automation_engaged);
    guidance_result_clear(&r);
    puts("Terminal warnings continue bounded control; failed contact and manual abort stay distinct.");
}
