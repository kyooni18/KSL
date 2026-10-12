#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "landing.h"
#include "../CLanding/guidance/guidance_internal.h"
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
    if(ready){
        /* A numerically admissible inlet without a terminal model/qualified
           route cannot complete Entry or acquire MM305 ownership. */
        GuidanceMachine blocked={0};
        blocked.automation_engaged=true;
        blocked.phase=PHASE_ENTRY_ENERGY;
        blocked.terminal_glide_mode=true;
        blocked.deorbit_burn_completed=true;
        blocked.atmospheric_interface_crossed=true;
        blocked.entry_exec.initialized=true;
        blocked.entry_exec.phase=ENTRY_PHASE_TRANSITION;
        blocked.s_turn_sign=1.0;
        Telemetry coherent=t;
        coherent.pitch=coherent.flight_path_angle+coherent.angle_of_attack;
        coherent.heading=coherent.ground_track_heading=course;
        GuidanceResult held=guidance_update_with_terminal_model(&blocked,&coherent,&state,
            NULL,p,aero,cfg,NULL);
        if(held.phase!=PHASE_ENTRY_ENERGY)fprintf(stderr,"blocked handoff phase=%d status=%s\n",held.phase,held.status);
        assert(held.phase==PHASE_ENTRY_ENERGY);
        assert(!blocked.entry_exec.entry_complete);
        assert(!blocked.terminal_region_entered);
        assert(!taem_exec_owns_vehicle(&blocked.taem_exec));
        guidance_result_clear(&held);
    }
    bool matches=ready==expected&&forecast.reached_taem==ready;
    entry_prediction_clear(&forecast);
    if(!matches) exit(EXIT_FAILURE);
}

static void test_entry_measured_drag_requires_valid_physics(void) {
    Telemetry t={0};
    t.mass=25816.0;
    t.drag_force=1006.824;
    assert(isnan(entry_measured_drag_accel(&t)));
    t.physics_sample_valid=true;
    assert(fabs(entry_measured_drag_accel(&t)-0.039)<1e-12);
}

static void test_entry_drag_authority_is_continuous_at_output_floor(
        const PlanetModel *planet,const LandingConfiguration *cfg) {
    EntryDragReferenceConfig drag_cfg=entry_drag_reference_default_config();
    EntryDragReferenceInput input={
        .phase=ENTRY_PHASE_TEMPERATURE_CONTROL,
        .relative_velocity=2110.0,
        .latitude=cfg->site.latitude,
        .altitude=66920.0,
        .range_to_site=936190.0,
        .taem_range=35000.0,
        .taem_latitude=cfg->site.latitude,
        .taem_altitude=cfg->guidance.taem_interface_altitude,
        .taem_velocity=cfg->guidance.taem_force_handoff_speed,
        .measured_drag_accel=NAN,
        .modeled_drag_accel=0.0399,
        .aero_confidence=0.8,
        .planet=planet,
        .vehicle=&cfg->vehicle
    };
    EntryDragReferenceOutput below=entry_drag_reference_compute(&input,&drag_cfg);
    input.modeled_drag_accel=0.0401;
    EntryDragReferenceOutput above=entry_drag_reference_compute(&input,&drag_cfg);
    assert(below.valid&&above.valid);
    assert(fabs(below.reference_drag_accel-above.reference_drag_accel)<0.1);
}

int main(void) {
    /* Prospective feedback consumes the rollout's force, independently of the
       fallback polar, without marking the synthetic observation measured. */
    {
        GuidanceMachine g={0}; Telemetry t={0}; VehicleProfile v={0};
        AerodynamicModel a={.lift_to_drag=.4,.ballistic_coefficient=700};
        t.mass=25000; t.lift_force=50000; t.dynamic_pressure=1000;
        t.angle_of_attack=18; t.mach=5;
        double fallback=live_lift_accel(&t,a,&v);
        assert(guidance_feedback_lift_accel(&g,&t,a,&v)==fallback);
        g.diagnostic_shadow=true;
        assert(guidance_feedback_lift_accel(&g,&t,a,&v)==2.0);
        assert(!t.physics_sample_valid);
        t.lift_force=0;
        assert(guidance_feedback_lift_accel(&g,&t,a,&v)==0.0);
        t.lift_force=NAN;
        assert(guidance_feedback_lift_accel(&g,&t,a,&v)==live_lift_accel(&t,a,&v));
        t.lift_force=50000; t.physics_sample_valid=true;
        g.diagnostic_shadow=false;
        assert(guidance_feedback_lift_accel(&g,&t,a,&v)==2.0);
    }
    test_entry_measured_drag_requires_valid_physics();

    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    PlanetModel planet;
    char path[512],reason[256];
    shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,path,sizeof(path));
    assert(shuttle_sim_load_planet(path,&planet,reason,sizeof(reason)));
    test_entry_drag_authority_is_continuous_at_output_floor(&planet,&cfg);
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
