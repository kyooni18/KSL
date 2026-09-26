#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "landing.h"
#include "sim_telemetry.h"

/* Compare the public forecast's initial admission opportunity with the public
 * measured-state envelope, using the same imperfect aero model on both sides.
 * This checks policy parity, not downstream physical route feasibility. */
static void check_case(const PlanetModel *p, const LandingConfiguration *cfg,
        double range, double heading_offset, double mach, double height,
        bool expected, const char *label) {
    AerodynamicModel aero={.lift_to_drag=3.0,.ballistic_coefficient=3000.0,.confidence=.8};
    AerodynamicEnvelope env={.regimes={aero,aero,aero,aero}};
    TrajectoryCalibrationModel cal={.density_scale=1,.drag_scale=1,.lift_scale=1,
        .bank_effectiveness=1,.speed_of_sound_scale=1};
    GeoPoint site={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    GeoPoint point=local_point(site,-range,0,p->radius,height);
    VehicleState state={.ut=1000,.mass=40000};
    state.position=predictor_inertial_position(point,p,state.ut);
    Vector3 up=vnorm(state.position,v3(0,1,0));
    Vector3 north=vnorm(vproject_plane(p->north_axis,up),v3(0,0,1));
    Vector3 east=vnorm(vcross(north,up),v3(1,0,0));
    double course=initial_bearing(point,site)+heading_offset;
    double density,sound;
    vessel_physics_environment(p,&cal,height,&density,&sound);
    double speed=mach*sound, gamma=-8*DEG2RAD, alpha=18;
    Vector3 horizontal=vadd(vscale(north,cos(course*DEG2RAD)),vscale(east,sin(course*DEG2RAD)));
    state.velocity=vadd(vscale(vadd(vscale(horizontal,cos(gamma)),vscale(up,sin(gamma))),speed),
        vcross(planet_rotation_vector(p),state.position));
    Telemetry t={0};
    t.ut=state.ut;t.latitude=point.latitude;t.longitude=point.longitude;
    t.mean_altitude=height;t.radar_altitude=height-site.altitude;
    t.true_air_speed=speed;t.horizontal_speed=speed*cos(gamma);t.vertical_speed=speed*sin(gamma);
    t.flight_path_angle=-8;t.mach=mach;t.angle_of_attack=alpha;t.mass=state.mass;
    t.dynamic_pressure=.5*density*speed*speed;t.bank_effectiveness=1;
    t.course_to_site_error=norm_signed_deg(-heading_offset);
    t.estimated_lift_to_drag=aero.lift_to_drag;t.estimated_ballistic_coefficient=aero.ballistic_coefficient;
    t.aerodynamic_confidence=aero.confidence;
    runway_coordinates(point,site,cfg->site.runway_heading,p->radius,&t.runway_along_track,&t.runway_cross_track);
    double confidence;
    bool observed;
    Vector3 force=vessel_physics_force_best_estimate_config(NULL,t.dynamic_pressure,mach,alpha,0,
        state.mass,false,false,0,aero,&cal,&cfg->vehicle,&confidence,&observed,NULL);
    t.lift_force=fabs(force.y)*state.mass;t.drag_force=fabs(force.x)*state.mass;
    t.g_force=fabs(force.y)/planet_surface_gravity(p);
    TaemInterfaceCapture admission=entry_mm305_admission_envelope(NULL,&t,course,p,cfg);
    EntryPrediction forecast=predictor_simulate_entry_with_attitude(state,p,aero,&env,&cal,
        &cfg->vehicle,&cfg->site,&cfg->guidance,70,0,0,alpha,0,1,0,1e-6,false);
    bool ready=admission.valid&&admission.ready;
    printf("%s: envelope=%d forecast=%d veto=0x%x q=%.0f g=%.2f\n",
        label,ready,forecast.reached_taem,admission.veto,t.dynamic_pressure,t.g_force);
    bool matches=ready==expected&&forecast.reached_taem==ready;
    entry_prediction_clear(&forecast);
    if(!matches) exit(EXIT_FAILURE);
}

int main(void) {
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    PlanetModel planet;
    char path[512],reason[256];
    shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,path,sizeof(path));
    assert(shuttle_sim_load_planet(path,&planet,reason,sizeof(reason)));
    check_case(&planet,&cfg,30000,0,2.5,26000,true,"closing");
    check_case(&planet,&cfg,30000,180,2.5,26000,false,"receding");
    check_case(&planet,&cfg,140000,0,2.5,26000,false,"outside range");
    check_case(&planet,&cfg,30000,0,1.5,26000,false,"below Mach band");
    check_case(&planet,&cfg,30000,0,2.5,31000,false,"above altitude band");
    cfg.site.runway_heading=270;
    check_case(&planet,&cfg,30000,0,2.5,26000,true,"reciprocal closing");
    check_case(&planet,&cfg,30000,180,2.5,26000,false,"reciprocal receding");
    cfg.vehicle.maximum_dynamic_pressure=1;
    check_case(&planet,&cfg,30000,0,2.5,26000,false,"structural q veto");
    puts("Entry forecast/admission parity passed.");
    return EXIT_SUCCESS;
}
