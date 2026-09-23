#include "../CLanding/landing.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void){
    const double radius=600000.0;
    LandingConfiguration cfg=landing_configuration_default();
    cfg.site.latitude=0.0;
    cfg.site.longitude=0.0;
    cfg.site.altitude=70.0;
    cfg.site.runway_heading=90.0;
    cfg.site.runway_length=2500.0;
    cfg.guidance.final_approach_distance=8000.0;
    cfg.guidance.hac_radius=12000.0;
    cfg.guidance.taem_interface_range=35000.0;

    LandingSite reciprocal=runway_reciprocal_site(&cfg.site,radius);
    GeoPoint primary_point={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    GeoPoint reciprocal_point={reciprocal.latitude,reciprocal.longitude,reciprocal.altitude};
    assert(fabs(great_circle_distance(primary_point,reciprocal_point,radius)-2500.0)<1.0);
    assert(fabs(norm_signed_deg(reciprocal.runway_heading-270.0))<1e-9);

    Telemetry eastbound;memset(&eastbound,0,sizeof(eastbound));
    GeoPoint west=destination_point(primary_point,270.0,20000.0,radius);
    eastbound.latitude=west.latitude;eastbound.longitude=west.longitude;eastbound.mean_altitude=27000.0;
    eastbound.ground_track_heading=90.0;eastbound.heading=90.0;eastbound.horizontal_speed=1300.0;
    double score09=runway_end_acquisition_score(&eastbound,&cfg.site,&cfg.guidance,radius);
    double score27=runway_end_acquisition_score(&eastbound,&reciprocal,&cfg.guidance,radius);
    assert(score09<score27);
    telemetry_reframe_runway(&eastbound,&cfg.site,radius);
    assert(eastbound.runway_along_track<0.0&&fabs(eastbound.runway_cross_track)<5.0);

    Telemetry westbound;memset(&westbound,0,sizeof(westbound));
    GeoPoint east=destination_point(reciprocal_point,90.0,20000.0,radius);
    westbound.latitude=east.latitude;westbound.longitude=east.longitude;westbound.mean_altitude=27000.0;
    westbound.ground_track_heading=270.0;westbound.heading=270.0;westbound.horizontal_speed=1300.0;
    score09=runway_end_acquisition_score(&westbound,&cfg.site,&cfg.guidance,radius);
    score27=runway_end_acquisition_score(&westbound,&reciprocal,&cfg.guidance,radius);
    assert(score27<score09);
    telemetry_reframe_runway(&westbound,&reciprocal,radius);
    assert(westbound.runway_along_track<0.0&&fabs(westbound.runway_cross_track)<5.0);

    puts("PASS: reciprocal runway threshold, heading, frame, and acquisition-end selection geometry.");
    return 0;
}
