#include "../CLanding/landing.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    cfg.site.latitude=-0.0486111111;cfg.site.longitude=-74.7283333333;cfg.site.altitude=70.0;cfg.site.runway_heading=90.0;
    PlanetModel p={.radius=600000.0,.gravitational_parameter=3531600000000.0,
        .rotational_speed=.000291570900559802,.atmosphere_depth=70000.0};
    TaemInterfaceTarget q={0};
    q.valid=true;q.along_track=-cfg.guidance.final_approach_distance;q.cross_track=0.0;q.course=0.0;
    q.altitude=26500.0;q.speed=1247.0;q.flight_path_angle=-9.8;
    q.acquisition_lead=15000.0;q.remaining_path=55000.0;q.response_time=7.0;
    q.arrival_ut=100.0;
    GeoPoint origin={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    double rh=cfg.site.runway_heading*DEG2RAD;
    double e=q.along_track*sin(rh)+q.cross_track*cos(rh);
    double n=q.along_track*cos(rh)-q.cross_track*sin(rh);
    GeoPoint gp=local_point(origin,e,n,p.radius,q.altitude);
    q.specific_energy=rotating_specific_energy(gp.latitude,q.altitude,q.speed,&p);

    Telemetry t={0};
    t.latitude=cfg.site.latitude;t.runway_along_track=q.along_track;t.runway_cross_track=q.cross_track+700.0;
    t.true_air_speed=1250.0;t.horizontal_speed=1230.0;t.vertical_speed=-215.0;t.flight_path_angle=-9.8;
    t.angle_of_attack=12.0;t.mass=40000.0;t.lift_force=t.mass*5.0;t.drag_force=t.mass*1.0;
    double slope=9.8*DEG2RAD;
    t.mean_altitude=q.altitude+700.0*tan(slope);t.radar_altitude=t.mean_altitude;
    t.dynamic_pressure=7000.0;t.g_force=1.0;t.bank_effectiveness=1.0;
    t.attitude_response.pitch_valid=true;t.attitude_response.roll_valid=true;
    t.attitude_response.maximum_pitch_rate_deg_s=8.0;
    t.attitude_response.maximum_roll_rate_deg_s=18.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=5.0;
    t.attitude_response.maximum_roll_accel_deg_s2=15.0;
    t.stall_fraction=0.0;t.stall_fraction_is_measured=true;
    /* Inside the fixed 1 km alignment tube, only drag before the ownership
       boundary belongs to MM304.  Use a deliberately large instantaneous drag
       value so charging the whole local-to-target distance would fail the old
       energy test even though the vehicle has already reached the handoff tube. */
    double drag_accel=250.0;

    TaemInterfaceCapture c=entry_taem_interface_capture(&q,&t,0.0,&p,&cfg);
    double current=rotating_specific_energy(t.latitude,t.mean_altitude,t.true_air_speed,&p);
    double legacy=current-q.specific_energy-drag_accel*fmax(0.0,c.along)/fmax(.8,cos(slope));
    double low_limit=-.10*q.speed*q.speed;
    printf("capture along=%.0f localTube=1000 newE=%+.0f legacyE=%+.0f limit=%+.0f veto=%u ready=%d\n",
        c.along,c.energy_margin,legacy,low_limit,c.veto,c.ready);
    assert(c.valid);
    assert(c.along>0.0&&c.along<1000.0);
    assert(hypot(c.along,c.cross)<=1000.0);
    assert(legacy<low_limit);
    assert(c.energy_margin>0.0);
    assert(c.turn_margin>0.0);
    assert((c.veto&2u)==0u);

    /* A geometrically valid handoff with no time to execute its bounded-turn
       path is still unsafe.  The evaluator must expose that as a maneuver
       veto instead of reporting a ready state. */
    TaemInterfaceTarget late=q;
    late.arrival_ut=1.0;
    TaemInterfaceCapture late_capture=entry_taem_interface_capture(
        &late,&t,0.0,&p,&cfg);
    assert(late_capture.valid);
    assert(late_capture.turn_margin<0.0);
    assert((late_capture.veto&16u)!=0u);
    assert(!late_capture.ready);
    return 0;
}
