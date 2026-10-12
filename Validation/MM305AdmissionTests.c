#include "landing.h"
#include "decision_envelope.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static PlanetModel kerbin(void){
    PlanetModel p={
        .radius=600000.0,
        .gravitational_parameter=3531600000000.0,
        .rotational_speed=.000291570900559802,
        .atmosphere_depth=70000.0,
        .surface_density=1.225,
        .atmosphere_adiabatic_index=1.4,
    };
    p.atmosphere_sample_count=5;
    p.atmosphere_altitude[0]=0.0;     p.atmosphere_pressure[0]=101325.0; p.atmosphere_density[0]=1.225;
    p.atmosphere_altitude[1]=15000.0; p.atmosphere_pressure[1]=12000.0;  p.atmosphere_density[1]=0.18;
    p.atmosphere_altitude[2]=25000.0; p.atmosphere_pressure[2]=3600.0;   p.atmosphere_density[2]=0.05;
    p.atmosphere_altitude[3]=50000.0; p.atmosphere_pressure[3]=120.0;    p.atmosphere_density[3]=0.002;
    p.atmosphere_altitude[4]=70000.0; p.atmosphere_pressure[4]=1.0;      p.atmosphere_density[4]=1e-6;
    return p;
}

static Telemetry admissible_state(const PlanetModel*p,const LandingConfiguration*cfg){
    Telemetry t;
    memset(&t,0,sizeof(t));
    t.latitude=cfg->site.latitude;
    t.mean_altitude=.5*(cfg->guidance.mm305_min_altitude+cfg->guidance.mm305_max_altitude);
    t.radar_altitude=t.mean_altitude;
    double sound=planet_atmospheric_speed_of_sound(p,t.mean_altitude);
    t.mach=cfg->guidance.mm305_target_mach;
    t.true_air_speed=sound*t.mach;
    t.horizontal_speed=t.true_air_speed*cos(8.0*DEG2RAD);
    t.vertical_speed=-t.true_air_speed*sin(8.0*DEG2RAD);
    t.flight_path_angle=-8.0;
    t.runway_along_track=-30000.0;
    t.runway_cross_track=8000.0;
    t.dynamic_pressure=fmin(7000.0,.35*cfg->vehicle.maximum_dynamic_pressure);
    t.g_force=fmin(1.2,.35*cfg->vehicle.maximum_g_load);
    t.mass=40000.0;
    t.angle_of_attack=18.0;
    t.sideslip=0.0;
    t.lift_force=t.mass*8.0;
    t.drag_force=t.mass*1.0;
    t.bank_effectiveness=1.0;
    t.estimated_ballistic_coefficient=cfg->vehicle.estimated_ballistic_coefficient;
    t.aerodynamic_confidence=.9;
    t.attitude_response.roll_valid=true;
    t.attitude_response.pitch_valid=true;
    t.attitude_response.maximum_roll_rate_deg_s=18.0;
    t.attitude_response.maximum_roll_accel_deg_s2=12.0;
    t.attitude_response.maximum_pitch_rate_deg_s=8.0;
    t.attitude_response.maximum_pitch_accel_deg_s2=5.0;
    t.stall_fraction_is_measured=true;
    t.stall_fraction=0.0;
    return t;
}

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.latitude=-0.0486111111;
    cfg.site.longitude=-74.7283333333;
    cfg.site.altitude=70.0;
    cfg.site.runway_heading=90.0;
    PlanetModel p=kerbin();
    GuidanceMachine g={0};
    g.phase=PHASE_ENTRY_ENERGY;
    g.automation_engaged=true;

    Telemetry nominal=admissible_state(&p,&cfg);
    TaemInterfaceCapture ready=entry_mm305_admission_envelope(&g,&nominal,90.0,&p,&cfg);
    assert(ready.valid);
    assert(ready.path_length_m>0.0);
    assert(ready.energy_margin>0.0);
    assert(ready.veto==0u);
    assert(ready.ready);

    /* Drag work and admission energy must close in the rotating frame, even
       when latitude changes along the physically propagated reference. */
    double final_alt=cfg.site.altitude+cfg.guidance.final_approach_distance*
        tan(cfg.guidance.final_glide_slope*DEG2RAD);
    UnpoweredPathProjection projection=decision_unpowered_path_projection(
        &g,&nominal,&p,&cfg,final_alt,ready.path_length_m);
    assert(projection.valid&&projection.reference_reachable);
    double loss=entry_remaining_specific_energy(nominal.latitude,nominal.mean_altitude,
        nominal.true_air_speed,projection.terminal_latitude_deg,final_alt,
        projection.terminal_speed_mps,&p);
    assert(fabs(loss-projection.modeled_drag_work)<=projection.integration_error_j_kg+1e-6);
    assert(projection.integration_error_j_kg<1e-4*fabs(ready.energy_available));
    EnergyPathEnvelope positive=decision_energy_path_envelope(&nominal,&p,100.0,90.0,1.0);
    EnergyPathEnvelope negative=decision_energy_path_envelope(&nominal,&p,90.0,100.0,1.0);
    assert(positive.valid&&positive.energy.margin==10.0);
    assert(negative.valid&&negative.energy.margin==-10.0&&!negative.feasible);
    Telemetry invalid=nominal;invalid.true_air_speed=NAN;
    assert(!decision_unpowered_path_projection(&g,&invalid,&p,&cfg,final_alt,ready.path_length_m).valid);
    invalid=nominal;invalid.latitude=91.0;
    assert(!decision_unpowered_path_projection(&g,&invalid,&p,&cfg,final_alt,ready.path_length_m).valid);
    assert(isnan(rotating_specific_energy(91.0,1000.0,100.0,&p)));
    assert(isnan(rotating_specific_energy(0.0,1000.0,-100.0,&p)));
    assert(!decision_unpowered_path_projection(&g,&nominal,&p,&cfg,final_alt,INFINITY).valid);

    Telemetry low=nominal;
    low.mean_altitude=cfg.guidance.mm305_min_altitude-1.0;
    TaemInterfaceCapture altitude=entry_mm305_admission_envelope(&g,&low,90.0,&p,&cfg);
    assert(altitude.valid&&((altitude.veto&8u)!=0u)&&!altitude.ready);

    Telemetry slow=nominal;
    slow.mach=cfg.guidance.mm305_target_mach-cfg.guidance.mm305_mach_half_width-.01;
    slow.true_air_speed=planet_atmospheric_speed_of_sound(&p,slow.mean_altitude)*slow.mach;
    slow.horizontal_speed=slow.true_air_speed*cos(8.0*DEG2RAD);
    slow.vertical_speed=-slow.true_air_speed*sin(8.0*DEG2RAD);
    TaemInterfaceCapture speed=entry_mm305_admission_envelope(&g,&slow,90.0,&p,&cfg);
    assert(speed.valid&&((speed.veto&1u)!=0u)&&!speed.ready);

    Telemetry far=nominal;
    far.runway_along_track=-(entry_taem_range_target(&p,&cfg.guidance)+1000.0);
    far.runway_cross_track=0.0;
    TaemInterfaceCapture range=entry_mm305_admission_envelope(&g,&far,90.0,&p,&cfg);
    assert(range.valid&&((range.veto&2u)!=0u)&&!range.ready);

    Telemetry climbing=nominal;
    climbing.vertical_speed=fabs(climbing.vertical_speed);
    TaemInterfaceCapture descent=entry_mm305_admission_envelope(&g,&climbing,90.0,&p,&cfg);
    assert(descent.valid&&((descent.veto&32u)!=0u)&&!descent.ready);

    Telemetry overloaded=nominal;
    overloaded.dynamic_pressure=cfg.vehicle.maximum_dynamic_pressure+1.0;
    TaemInterfaceCapture safety=entry_mm305_admission_envelope(&g,&overloaded,90.0,&p,&cfg);
    assert(safety.valid&&((safety.veto&64u)!=0u)&&!safety.ready);

    Telemetry draggy=nominal;
    draggy.runway_along_track=-85000.0;
    draggy.runway_cross_track=0.0;
    draggy.estimated_ballistic_coefficient=30.0;
    TaemInterfaceCapture energy=entry_mm305_admission_envelope(&g,&draggy,90.0,&p,&cfg);
    assert(energy.valid);
    assert(energy.energy_margin<ready.energy_margin);
    assert((energy.veto&16u)!=0u);
    assert(!energy.ready);

    puts("MM305 admissible-state tests passed.");
    return 0;
}
