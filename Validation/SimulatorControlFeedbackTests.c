#include "sim_telemetry.h"
#include "shuttlesim/sim.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    Scenario scenario;
    scenario_seed_86km(&scenario);
    Simulation sim;
    sim_init(&sim,&scenario);
    PlanetModel planet={.radius=600000,.gravitational_parameter=3.5316e12,
        .rotational_speed=2.0*LANDER_PI/21549.425};
    char packet[4096],reason[192];
    Telemetry t;
    VehicleState state;
    sim.state.attitude.direct=true;
    sim.state.attitude.input_pitch=.23;
    sim.state.attitude.input_roll=-.17;
    sim.state.attitude.input_yaw=.09;
    sim_build_telemetry_json(&sim,1,packet,sizeof(packet));
    assert(shuttle_sim_decode_telemetry(packet,&planet,NULL,&t,&state,reason,sizeof(reason)));
    assert(t.has_controls&&fabs(t.control_pitch-.23)<1e-9&&
        fabs(t.control_roll+.17)<1e-9&&fabs(t.control_yaw-.09)<1e-9);

    /* A partial legacy packet must not silently fabricate zero stick feedback. */
    char *start=strstr(packet,"\"input_pitch\":");
    assert(start);
    char *end=strchr(start,',');assert(end);
    memmove(start,end+1,strlen(end+1)+1);
    assert(shuttle_sim_decode_telemetry(packet,&planet,NULL,&t,&state,reason,sizeof(reason)));
    assert(!t.has_controls);

    sim.state.attitude.direct=false;
    sim_build_telemetry_json(&sim,1,packet,sizeof(packet));
    assert(shuttle_sim_decode_telemetry(packet,&planet,NULL,&t,&state,reason,sizeof(reason)));
    assert(!t.has_controls);
    sim.state.attitude.direct=true;sim.state.attitude.input_pitch=1.2;
    sim_build_telemetry_json(&sim,1,packet,sizeof(packet));
    assert(shuttle_sim_decode_telemetry(packet,&planet,NULL,&t,&state,reason,sizeof(reason)));
    assert(!t.has_controls);
    puts("PASS: native simulator actuator feedback round trip and legacy fallback.");
    return 0;
}
