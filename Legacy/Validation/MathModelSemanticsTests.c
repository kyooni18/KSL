#include "landing.h"

#include <assert.h>
#include <float.h>
#include <math.h>

static void near(double actual,double expected){
    double scale=fmax(1.0,fmax(fabs(actual),fabs(expected)));
    assert(fabs(actual-expected)<=32.0*DBL_EPSILON*scale);
}

int main(void){
    Vector3 fallback=v3(0.0,1.0,0.0);
    Vector3 tiny=vnorm(v3(1e-15,0.0,0.0),fallback);
    near(tiny.x,1.0);near(tiny.y,0.0);near(tiny.z,0.0);
    Vector3 zero=vnorm(v3(0.0,0.0,0.0),fallback);
    near(zero.x,fallback.x);near(zero.y,fallback.y);near(zero.z,fallback.z);
    assert(isfinite(vmag(v3(1e200,0.0,0.0))));
    near(vmag(v3(1e200,0.0,0.0)),1e200);

    /* Machine epsilon is dimensionless; tiny but nonzero distances/speeds are
       still geometrically defined. These cases used to be discarded by
       comparing metres or m/s directly with DBL_EPSILON. */
    double tiny_course=surface_course(v3(600000.0,0.0,0.0),
        v3(0.0,1e-17,0.0),v3(0.0,0.0,0.0),v3(0.0,0.0,1.0),123.0);
    near(tiny_course,90.0);
    double tiny_capture=bank_for_point_capture(100.0,10.0,1.0,
        10.0,1e-17,45.0);
    assert(tiny_capture>44.0);

    /* Runway coordinates are signed distances to the runway great circle,
       not components of the shortest-path displacement.  The nominal KSP
       post-burn state is close to the antipode of the runway, where the two
       definitions differ by tens of kilometres.  The coordinate frame must
       remain stable as the vehicle advances through that singular-looking
       longitude region and must agree with ShuttleSim's telemetry. */
    GeoPoint runway={-0.0486111111,-74.7283333333,70.0};
    GeoPoint postburn={-0.002878630,107.084175676,86657.532};
    GeoPoint later={-0.002880073,107.097781661,86656.986};
    double along=0.0,cross=0.0,later_along=0.0,later_cross=0.0;
    runway_coordinates(postburn,runway,90.0,600000.0,&along,&cross);
    runway_coordinates(later,runway,90.0,600000.0,&later_along,&later_cross);
    assert(along<-1.8e6&&fabs(cross)<2000.0);
    assert(later_along>along&&fabs(later_cross-cross)<20.0);

    JerkLimiter zero_authority={0};
    near(jerk_update(&zero_authority,0.0,1.0,1.0,0.1),0.0);
    near(jerk_update(&zero_authority,10.0,0.0,1.0,0.1),0.0);
    near(zero_authority.rate,0.0);
    near(jerk_update(&zero_authority,10.0,1.0,0.0,0.1),0.0);
    near(zero_authority.rate,0.0);

    JerkLimiter slow={0},fast={0};
    near(jerk_update(&slow,0.0,1.0,0.5,0.1),0.0);
    near(jerk_update(&fast,0.0,2.0,1.0,0.1),0.0);
    double slow_step=jerk_update(&slow,10.0,1.0,0.5,0.5);
    double fast_step=jerk_update(&fast,10.0,2.0,1.0,0.5);
    assert(slow_step>0.0&&fast_step>slow_step);

    /* A first-order filter has no hidden absolute time floor. Invalid or zero
       elapsed time leaves state unchanged; a non-positive time constant means
       no filtering, which is the limiting tc -> 0 behavior. */
    LowPass filter={.tc=2.0};
    near(lowpass_update(&filter,10.0,1.0),10.0);
    near(lowpass_update(&filter,20.0,0.0),10.0);
    near(lowpass_update(&filter,20.0,1.0),10.0+10.0/3.0);
    LowPass direct={.tc=0.0};
    near(lowpass_update(&direct,3.0,1.0),3.0);
    near(lowpass_update(&direct,7.0,1.0),7.0);

    /* These mappings are dimensionless interpolations. Sub-unit spans are
       physically valid and must not collapse to the endpoint merely because
       meters or metres/second happen to be below 1. */
    double speed_mid=entry_altitude_target_for_speed(
        10000.0,100.5,100.25,9000.0,100.0);
    near(speed_mid,9500.0);
    assert(entry_altitude_target_for_speed(
        10000.0,100.5,100.4,9000.0,100.0)>speed_mid);

    double range_mid=entry_altitude_target_for_range(
        10000.0,1000.5,1000.25,9000.0,1000.0);
    near(range_mid,9500.0);
    assert(entry_altitude_target_for_range(
        10000.0,1000.5,1000.4,9000.0,1000.0)>range_mid);

    PlanetModel p={
        .radius=600000.0,
        .gravitational_parameter=3.5e12,
        .rotational_speed=0.0002915709
    };
    double base=rotating_specific_energy(0.0,0.0,100.0,&p);
    double higher=rotating_specific_energy(0.0,1000.0,100.0,&p);
    double faster=rotating_specific_energy(0.0,0.0,200.0,&p);
    assert(isfinite(base));
    assert(higher>base);
    assert(faster>base);

    /* The center/singularity of the gravity model is outside its physical
       domain; it must fail closed rather than be silently clamped to 1 m. */
    assert(isnan(rotating_specific_energy(0.0,-p.radius,100.0,&p)));
    PlanetModel invalid=p;
    invalid.radius=0.0;
    assert(isnan(rotating_specific_energy(0.0,0.0,100.0,&invalid)));
    invalid=p;
    invalid.gravitational_parameter=0.0;
    assert(isnan(rotating_specific_energy(0.0,0.0,100.0,&invalid)));
    assert(isnan(planet_surface_gravity(&invalid)));
    near(planet_surface_gravity(&p),p.gravitational_parameter/(p.radius*p.radius));

    /* Inside an atmosphere, absent profile samples are invalid model data.
       Outside the atmosphere, zero density/pressure/sound is physical. */
    PlanetModel missing_profile=p;
    missing_profile.atmosphere_depth=70000.0;
    missing_profile.atmosphere_adiabatic_index=1.4;
    assert(isnan(planet_atmospheric_density(&missing_profile,1000.0)));
    assert(isnan(planet_atmospheric_pressure(&missing_profile,1000.0)));
    assert(isnan(planet_atmospheric_speed_of_sound(&missing_profile,1000.0)));
    near(planet_atmospheric_density(&missing_profile,70000.0),0.0);
    near(planet_atmospheric_pressure(&missing_profile,70000.0),0.0);
    near(planet_atmospheric_speed_of_sound(&missing_profile,70000.0),0.0);

    /* Interpolation is dimensionless and must preserve arbitrarily small but
       representable positive altitude spans. The old absolute 1e-9 m floor
       biased this midpoint toward the lower sample. */
    p.atmosphere_depth=1.0;
    p.atmosphere_sample_count=2;
    p.atmosphere_altitude[0]=0.0;
    p.atmosphere_altitude[1]=5e-10;
    p.atmosphere_density[0]=1.0;
    p.atmosphere_density[1]=0.25;
    p.atmosphere_pressure[0]=100000.0;
    p.atmosphere_pressure[1]=25000.0;
    p.atmosphere_adiabatic_index=1.4;
    near(planet_atmospheric_density(&p,2.5e-10),0.5);
    near(planet_atmospheric_pressure(&p,2.5e-10),50000.0);
    near(planet_atmospheric_speed_of_sound(&p,2.5e-10),sqrt(1.4*50000.0/0.5));
    p.atmosphere_adiabatic_index=0.0;
    assert(isnan(planet_atmospheric_speed_of_sound(&p,2.5e-10)));

    return 0;
}
