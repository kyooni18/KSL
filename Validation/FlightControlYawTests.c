#include "flight_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Telemetry sample(double ut,double beta,double body_yaw_rate){
    Telemetry t={0};
    t.ut=ut;
    t.pitch=23.0;
    t.angle_of_attack=25.0;
    t.roll=70.0;
    t.heading=90.0;
    t.ground_track_heading=90.0;
    t.sideslip=beta;
    t.true_air_speed=2100.0;
    t.surface_speed=2100.0;
    t.horizontal_speed=2099.0;
    t.vertical_speed=-65.0;
    t.flight_path_angle=atan2(t.vertical_speed,t.horizontal_speed)*RAD2DEG;
    t.dynamic_pressure=130.0;
    t.has_torque=true;
    t.available_pitch_torque=160000.0;
    t.available_roll_torque=140000.0;
    t.available_yaw_torque=100000.0;
    t.has_inertia=true;
    t.pitch_moment_of_inertia=400000.0;
    t.roll_moment_of_inertia=350000.0;
    t.yaw_moment_of_inertia=300000.0;
    t.has_body_pitch_rate=true;
    t.has_body_roll_rate=true;
    t.has_body_yaw_rate=true;
    t.body_yaw_rate=body_yaw_rate;
    t.has_angle_of_attack_rate=true;
    t.angle_of_attack_rate=0.0;
    t.heading_rate=body_yaw_rate;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    return t;
}

static GuidanceCommand entry_command(void){
    GuidanceCommand c={0};
    c.autopilot_engaged=true;
    c.control_profile=PROFILE_ENTRY;
    c.has_target_aoa=true;
    c.target_aoa=25.0;
    c.target_pitch=23.0;
    c.target_roll=70.0;
    c.target_heading=90.0;
    return c;
}

static void beta_loop_ignores_coordinated_body_yaw_rate(void){
    FlightControlState s;
    flight_control_init(&s,0.1);
    GuidanceCommand c=entry_command();
    FlightControlOutput out={0};

    Telemetry t0=sample(100.0,1.00,5.0);
    assert(flight_control_step(&s,&t0,&c,0.1,&out));

    /* Beta is already moving toward zero. A large positive body yaw rate from
       the banked turn must not make the beta controller reverse rudder. */
    Telemetry t1=sample(100.1,0.98,5.0);
    assert(flight_control_step(&s,&t1,&c,0.1,&out));
    assert(out.valid);
    assert(out.diagnostics.sideslip_rate < -0.15);
    assert(out.diagnostics.body_yaw_rate > 4.0);
    assert(out.yaw > 0.0);
}

static void heading_hold_still_damps_heading_rate(void){
    FlightControlState s;
    flight_control_init(&s,0.1);
    GuidanceCommand c=entry_command();
    c.heading_control_enabled=true;
    c.target_heading=90.0;
    FlightControlOutput out={0};

    Telemetry t0=sample(200.0,0.0,5.0);
    assert(flight_control_step(&s,&t0,&c,0.1,&out));
    Telemetry t1=t0;
    t1.ut=200.1;
    t1.heading_rate=5.0;
    t1.body_yaw_rate=5.0;
    assert(flight_control_step(&s,&t1,&c,0.1,&out));
    assert(out.yaw < 0.0);
}

int main(void){
    beta_loop_ignores_coordinated_body_yaw_rate();
    heading_hold_still_damps_heading_rate();
    puts("flight control yaw tests passed");
    return 0;
}
