#include "../CLanding/guidance.c"
#include <assert.h>
#include <stdio.h>

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.latitude=-0.0486111111;cfg.site.longitude=-74.7283333333;
    cfg.site.altitude=70;cfg.site.runway_heading=90;
    cfg.guidance.final_approach_distance=3500;
    PlanetModel p={.radius=600000,.gravitational_parameter=3531600000000,
        .rotational_speed=2.0*LANDER_PI/21549.425,.atmosphere_depth=70000,
        .surface_density=.65,.atmosphere_adiabatic_index=1.4,.atmosphere_sample_count=2};
    p.atmosphere_altitude[0]=0;p.atmosphere_altitude[1]=70000;
    p.atmosphere_density[0]=p.atmosphere_density[1]=.65;
    p.atmosphere_pressure[0]=p.atmosphere_pressure[1]=50000;
    AerodynamicModel aero={.lift_to_drag=2,.ballistic_coefficient=800,.confidence=1};
    GuidanceMachine g;guidance_machine_init(&g);
    terminal_glide_initialize(&g,&cfg.vehicle,&cfg.guidance);
    Telemetry t;telemetry_init(&t);
    t.ut=100;t.mean_altitude=4500;t.radar_altitude=4430;
    t.latitude=cfg.site.latitude;t.longitude=cfg.site.longitude;
    t.mass=40252.91796875;t.true_air_speed=150;t.horizontal_speed=150;
    t.surface_speed=150;t.vertical_speed=0;t.flight_path_angle=0;
    t.heading=180;t.ground_track_heading=180;t.bank_effectiveness=1;
    t.angle_of_attack=10;t.roll=0;t.roll_rate=0;t.angle_of_attack_rate=0;
    t.atmospheric_density=.65;t.speed_of_sound=planet_atmospheric_speed_of_sound(&p,4500);
    t.mach=t.true_air_speed/t.speed_of_sound;t.dynamic_pressure=.5*.65*150*150;
    t.physics_certified_uncertainty=.08;
    double df=0,lf=0;aerodynamic_force_factors_mach(t.mach,10,&cfg.vehicle,&lf,&df);
    t.drag_force=t.mass*t.dynamic_pressure/aero.ballistic_coefficient*df;
    t.lift_force=t.drag_force*aero.lift_to_drag*lf/fmax(df,1e-12);
    const double paths[]={100,1000,5000,15000};
    int failures=0;
    /* Independent closed-form passive-drag solution. It cannot reach zero
       speed over a finite distance; composing cells must give the same work. */
    double analytic_speed=150*exp(-.65*df*15000/(2*800));
    double next_speed=0,exact_work=0;
    assert(terminal_quadratic_drag_energy_step(150,t.drag_force/t.mass,15000,0,
        &next_speed,&exact_work));
    assert(fabs(next_speed-analytic_speed)<1e-10);
    assert(fabs(exact_work-.5*(150*150-analytic_speed*analytic_speed))<1e-8);
    g.terminal_energy_loss_accel_ema=2*t.drag_force/t.mass;
    double source_anchor=terminal_drag_projection_anchor(&g,&t,&p,aero,&cfg.vehicle);
    Telemetry future=t;future.ut+=30;future.true_air_speed=75;
    future.surface_speed=75;future.horizontal_speed=75;future.mach=75/t.speed_of_sound;
    future.dynamic_pressure=.5*.65*75*75;future.drag_force=0;future.lift_force=0;
    double legacy_anchor=terminal_drag_projection_anchor(&g,&future,&p,aero,&cfg.vehicle);
    assert(fabs(source_anchor-2)<1e-12&&legacy_anchor>source_anchor*3.9);
    double future_v1=0,future_h1=0,future_v2=0,future_h2=0;
    double future_w1=terminal_projected_drag_work_state_anchored(&g,&future,&p,aero,
        &cfg.vehicle,&cfg.guidance,10,1000,0,&future_v1,&future_h1,source_anchor);
    g.terminal_energy_loss_accel_ema=1000;
    double future_w2=terminal_projected_drag_work_state_anchored(&g,&future,&p,aero,
        &cfg.vehicle,&cfg.guidance,10,1000,0,&future_v2,&future_h2,source_anchor);
    assert(isfinite(future_w1)&&future_w1==future_w2&&future_v1==future_v2);
    g.terminal_energy_loss_accel_ema=0;
    puts("PASS projected drag carries one source coefficient; stale live EMA cannot re-anchor it");
    for(size_t j=0;j<2;j++)for(size_t i=0;i<4;i++){
        double path=paths[i],slope=j?5:0,ve=NAN,he=NAN;
        double work=terminal_projected_drag_work_state(&g,&t,&p,aero,&cfg.vehicle,
            &cfg.guidance,10,path,slope,&ve,&he);
        double closure=rotating_specific_energy(t.latitude,t.mean_altitude,150,&p)-
            rotating_specific_energy(t.latitude,he,ve,&p)-work;
        printf("WORK path=%.0f slope=%.0f work=%.9f endV=%.9f endH=%.9f closure=%.9f\n",
            path,slope,work,ve,he,closure);
        if(!isfinite(work)||!isfinite(ve)||ve<=0||fabs(closure)>=1.0)failures++;
    }
    HACTransitionPlan plan={0};
    bool geometry=hac_fixed_alignment_plan(&plan,0,0,180,180,&cfg.site,&cfg.guidance,3000,1);
    assert(geometry);
    assert(fabs(plan.cone_arc_length-3000*1.5*LANDER_PI)<1e-6);
    assert(fabs(plan.p3.e+3500)<1e-6&&fabs(plan.p3.n)<1e-6);
    GuidanceMachine profile={0};double profile_response=NAN;
    bool profile_valid=fixed_hac_candidate_vertical_profile(&profile,&g,&t,&plan,
        &cfg.vehicle,&cfg.guidance,&cfg.site,1.0,&profile_response);
    assert(profile_valid);
    double p0_slope=fixed_hac_reference_slope(&profile,plan.cone_arc_length);
    double p0_altitude=fixed_hac_reference_altitude(&profile,plan.cone_arc_length);
    Telemetry p0={0};double work=0,course_error=0,lateral_margin=0;
    bool reached=terminal_fixed_hac_lead_reachable(&g,&t,&plan,&p,aero,&cfg.vehicle,
        &cfg.guidance,&cfg.site,150,&profile,&p0,&work,&course_error,&lateral_margin);
    assert(reached); /* A skipped p0 forecast must not make this regression pass. */
    printf("LEAD reached=%d elapsed=%.9f vertical=%.9f expectedVertical=%.9f surface=%.9f air=%.9f slope=%.9f profileAlt=%.9f\n",
        reached,p0.ut-t.ut,p0.vertical_speed,-p0.true_air_speed*sin(p0_slope*DEG2RAD),
        p0.surface_speed,p0.true_air_speed,p0_slope,p0_altitude);
    if(reached&&(p0.ut<=t.ut||
        fabs(p0.vertical_speed+p0.true_air_speed*sin(p0_slope*DEG2RAD))>1e-9||
        fabs(p0.mean_altitude-p0_altitude)>1e-9||
        fabs(p0.surface_speed-p0.true_air_speed)>1e-9))failures++;
    if(reached){
        double closure=rotating_specific_energy(t.latitude,t.mean_altitude,t.true_air_speed,&p)-
            rotating_specific_energy(p0.latitude,p0.mean_altitude,p0.true_air_speed,&p)-work;
        GeoPoint point=local_point((GeoPoint){cfg.site.latitude,cfg.site.longitude,cfg.site.altitude},
            plan.p0.e,plan.p0.n,p.radius,p0.mean_altitude);
        if(fabs(closure)>=1.0||fabs(point.latitude-p0.latitude)>1e-9||
           fabs(point.longitude-p0.longitude)>1e-9)failures++;
        printf("LEAD closure=%.12f p0Latitude=%.12f p0Longitude=%.12f\n",closure,p0.latitude,p0.longitude);
    }
    printf("HAC_ENERGY_TEST failures=%d\n",failures);
    return failures?1:0;
}
