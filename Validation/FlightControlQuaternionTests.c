#include "flight_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_close(double actual,double expected,double tolerance){
    assert(isfinite(actual));
    assert(fabs(actual-expected)<=tolerance);
}

static Telemetry atmospheric_sample(void){
    Telemetry t={0};
    t.ut=100.0;
    t.pitch=0.0;
    t.angle_of_attack=0.0;
    /* KSP surface Euler roll can alias by 180 degrees even when the physical
       air-relative attitude is correct. Quaternion control must ignore it. */
    t.roll=180.0;
    t.heading=90.0;
    t.ground_track_heading=90.0;
    t.sideslip=0.0;
    t.true_air_speed=1000.0;
    t.surface_speed=1000.0;
    t.horizontal_speed=1000.0;
    t.vertical_speed=0.0;
    t.flight_path_angle=0.0;
    t.dynamic_pressure=1000.0;
    t.has_torque=true;
    t.available_pitch_torque=200000.0;
    t.available_roll_torque=200000.0;
    t.available_yaw_torque=150000.0;
    t.has_inertia=true;
    t.pitch_moment_of_inertia=400000.0;
    t.roll_moment_of_inertia=400000.0;
    t.yaw_moment_of_inertia=300000.0;
    t.has_body_pitch_rate=true;
    t.has_body_roll_rate=true;
    t.has_body_yaw_rate=true;
    t.has_angle_of_attack_rate=true;
    return t;
}

static GuidanceCommand entry_command(void){
    GuidanceCommand c={0};
    c.autopilot_engaged=true;
    c.control_profile=PROFILE_ENTRY;
    c.has_target_aoa=true;
    c.target_aoa=0.0;
    c.target_pitch=0.0;
    c.target_roll=0.0;
    c.target_heading=90.0;
    return c;
}

static void airframe_error_tracks_aerodynamic_coordinates(void){
    const double identity[4]={0.0,0.0,0.0,1.0};
    const Vector3 air={0.0,1.0,0.0};
    const Vector3 radial_up={0.0,0.0,-1.0};
    FlightControlAttitudeError e={0};

    assert(flight_control_airframe_error(identity,air,radial_up,0.0,0.0,true,&e));
    assert(e.valid);
    assert_close(e.pitch_error_deg,0.0,1e-9);
    assert_close(e.roll_error_deg,0.0,1e-9);
    assert_close(e.yaw_error_deg,0.0,1e-9);

    assert(flight_control_airframe_error(identity,air,radial_up,10.0,0.0,true,&e));
    assert_close(e.pitch_error_deg,10.0,1e-6);
    assert_close(e.roll_error_deg,0.0,1e-6);
    assert_close(e.yaw_error_deg,0.0,1e-6);

    assert(flight_control_airframe_error(identity,air,radial_up,0.0,20.0,true,&e));
    assert_close(e.pitch_error_deg,0.0,1e-6);
    assert_close(e.roll_error_deg,20.0,1e-6);
    assert_close(e.yaw_error_deg,0.0,1e-6);
}

static void airframe_error_uses_shortest_roll_rotation(void){
    const double actual_bank=179.0*DEG2RAD;
    /* Positive guidance bank is negative raw-frame rotation about vessel Y. */
    const double q[4]={0.0,-sin(actual_bank*0.5),0.0,cos(actual_bank*0.5)};
    const Vector3 air={0.0,1.0,0.0};
    const Vector3 radial_up={0.0,0.0,-1.0};
    FlightControlAttitudeError e={0};

    assert(flight_control_airframe_error(q,air,radial_up,0.0,-179.0,true,&e));
    assert_close(e.roll_error_deg,2.0,1e-6);
    assert(fabs(e.pitch_error_deg)<1e-6);
    assert(fabs(e.yaw_error_deg)<1e-6);
}

static void quaternion_controller_ignores_euler_roll_alias(void){
    FlightControlState state;
    flight_control_init(&state,0.1);
    Telemetry t=atmospheric_sample();
    GuidanceCommand c=entry_command();
    FlightControlAttitudeError e={
        .valid=true,
        .use_yaw_error=true,
        .pitch_error_deg=0.0,
        .roll_error_deg=0.0,
        .yaw_error_deg=0.0
    };
    FlightControlOutput out={0};

    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));
    assert(out.valid);
    assert(out.diagnostics.quaternion_attitude_control);
    assert(fabs(out.roll)<1e-9);
    assert(fabs(out.yaw)<1e-9);
}

static void quaternion_controller_commands_body_error_signs(void){
    FlightControlState state;
    flight_control_init(&state,0.1);
    Telemetry t=atmospheric_sample();
    GuidanceCommand c=entry_command();
    FlightControlAttitudeError e={
        .valid=true,
        .use_yaw_error=true,
        .pitch_error_deg=5.0,
        .roll_error_deg=-8.0,
        .yaw_error_deg=3.0
    };
    FlightControlOutput out={0};

    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));
    assert(out.pitch>0.0);
    assert(out.roll<0.0);
    assert(out.yaw>0.0);
}

static void quaternion_damping_tracks_relative_error_not_absolute_body_rate(void){
    FlightControlState state;
    flight_control_init(&state,0.1);
    Telemetry t=atmospheric_sample();
    t.body_pitch_rate=7.0;
    t.body_roll_rate=-4.0;
    t.body_yaw_rate=3.0;
    GuidanceCommand c=entry_command();
    FlightControlAttitudeError e={
        .valid=true,
        .use_yaw_error=true,
        .pitch_error_deg=5.0,
        .roll_error_deg=-6.0,
        .yaw_error_deg=2.0
    };
    FlightControlOutput out={0};

    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));
    t.ut+=0.1;
    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));

    assert_close(out.diagnostics.effective_pitch_rate,0.0,1e-9);
    assert_close(out.diagnostics.effective_roll_rate,0.0,1e-9);
    assert_close(state.quaternion_yaw_error_rate,0.0,1e-9);
    assert_close(out.diagnostics.commanded_roll_rate,0.0,1e-9);
}

static void quaternion_damping_detects_closing_error(void){
    FlightControlState state;
    flight_control_init(&state,0.1);
    Telemetry t=atmospheric_sample();
    GuidanceCommand c=entry_command();
    FlightControlAttitudeError e={
        .valid=true,
        .use_yaw_error=true,
        .pitch_error_deg=5.0,
        .roll_error_deg=0.0,
        .yaw_error_deg=0.0
    };
    FlightControlOutput out={0};

    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));
    t.ut+=0.1;
    e.pitch_error_deg=4.0;
    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));

    assert(state.quaternion_pitch_error_rate<0.0);
    assert(out.diagnostics.effective_pitch_rate>0.0);
}

static void live_reported_authority_wins_over_stale_learned_authority(void){
    FlightControlState state;
    flight_control_init(&state,0.1);
    flight_control_seed_axis_authority(&state,FLIGHT_CONTROL_AXIS_PITCH,0.25,0.0,1.0);
    flight_control_seed_axis_authority(&state,FLIGHT_CONTROL_AXIS_ROLL,0.25,0.0,1.0);
    Telemetry t=atmospheric_sample();
    GuidanceCommand c=entry_command();
    FlightControlAttitudeError e={
        .valid=true,
        .use_yaw_error=true
    };
    FlightControlOutput out={0};

    assert(flight_control_step_attitude(&state,&t,&c,&e,0.1,&out));
    double reported=0.5*RAD2DEG;
    assert_close(out.diagnostics.pitch_raw_authority,reported,1e-6);
    assert_close(out.diagnostics.roll_raw_authority,reported,1e-6);
}

int main(void){
    airframe_error_tracks_aerodynamic_coordinates();
    airframe_error_uses_shortest_roll_rotation();
    quaternion_controller_ignores_euler_roll_alias();
    quaternion_controller_commands_body_error_signs();
    quaternion_damping_tracks_relative_error_not_absolute_body_rate();
    quaternion_damping_detects_closing_error();
    live_reported_authority_wins_over_stale_learned_authority();
    puts("flight control quaternion tests passed");
    return 0;
}
