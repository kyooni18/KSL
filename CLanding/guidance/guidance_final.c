#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
typedef struct {
    double arrest_accel,sgs_height,sgs_sink,final_accel,float_height,float_sink,lift_ratio,aoa_max,g0;
    const PlanetModel*planet;double site_altitude;
    double preflare_trigger_altitude;
} TerminalFlarePlan;

static double final_lift_aoa_limit(const Telemetry *t, const VehicleProfile *v);
static void terminal_learn_aero(GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v);
static double terminal_predict_touchdown_speed(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,double height,double V,double sink,bool latched,
        const TerminalFlarePlan*p);
static double terminal_wheel_height(const Telemetry*t,const LandingConfiguration*cfg);
static TerminalFlarePlan terminal_flare_plan(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double trigger_altitude);
static double terminal_final_distance(const GuidanceMachine*g,const GuidanceSettings*s);
static double terminal_final_alignment_speed(const GuidanceMachine*g,
        const GuidanceSettings*s,const VehicleProfile*v);
static double terminal_final_speed_target(const GuidanceMachine*g,
        const VehicleProfile*v,const GuidanceSettings*s,double distance);
static void terminal_observe_response(GuidanceMachine*g,const Telemetry*t,double dt);
static TerminalPreflarePlan terminal_preflare_plan(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
/* terminal_preflare_alignment_valid declared in guidance_internal.h */
static bool terminal_update_ground_latch(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,double dt);
static double observed_signed_roll_rate(const Telemetry *t);


/* Internal source slices; compiled as this single translation unit. */
#include "final/planning.inc"
#include "final/phases.inc"
#include "final/sequence.inc"
#include "final/recovery.inc"

bool terminal_native_delivery_admissible(const TerminalModel*m,
        const TerminalDynamicState*state,const GuidanceMachine*seed,
        TaemTerminalEvaluation*evaluation){
    TaemTerminalContract contract={0};
    if(evaluation)*evaluation=taem_exec_evaluate_terminal_contract(&contract);
    if(!m||!state||!seed||m->world.atmosphere.count==0||
            m->world.atmosphere.count>LANDER_ATMOSPHERE_SAMPLE_MAX)return false;
    TaemGeometryState geometry;
    if(!taem_geometry_state(m,state,&geometry))return false;
    PlanetModel planet={0};
    planet.radius=m->world.radius_m;
    planet.gravitational_parameter=m->world.mu_m3_s2;
    planet.rotational_speed=m->world.rotation_rate_rad_s;
    planet.atmosphere_depth=m->world.atmosphere_top_m;
    planet.atmosphere_adiabatic_index=1.4;
    planet.north_axis=(Vector3){0,0,1};
    planet.prime_meridian_at_epoch=(Vector3){cos(m->world.rotation_phase_rad_at_ut0),
        sin(m->world.rotation_phase_rad_at_ut0),0};
    planet.atmosphere_sample_count=m->world.atmosphere.count;
    for(size_t i=0;i<planet.atmosphere_sample_count;++i){
        const AtmospherePoint*point=&m->world.atmosphere.p[i];
        planet.atmosphere_altitude[i]=point->altitude_m;
        planet.atmosphere_pressure[i]=point->sample.pressure_pa;
        planet.atmosphere_density[i]=point->sample.density_kg_m3;
    }
    planet.surface_density=planet.atmosphere_density[0];
    LLA lla=world_lla(&m->world,state->position_i_m,state->ut_s);
    AtmosphereSample air=world_atmosphere_sample_state(&m->world,
        state->position_i_m,state->ut_s);
    AeroForces forces=aero_compute(&m->world,&m->aero,state->position_i_m,
        state->velocity_i_mps,state->ut_s,state->mass_kg,
        state->attitude.aoa_rad,state->attitude.bank_rad);
    Telemetry t={0};
    t.ut=state->ut_s;t.mass=state->mass_kg;
    t.latitude=lla.lat_rad*RAD2DEG;t.longitude=lla.lon_rad*RAD2DEG;
    t.mean_altitude=lla.altitude_m;
    t.radar_altitude=fmax(0.0,geometry.altitude_above_runway_m);
    t.runway_along_track=geometry.runway_along_m;
    t.runway_cross_track=geometry.runway_cross_m;
    t.true_air_speed=geometry.airspeed_mps;
    t.horizontal_speed=geometry.ground_speed_mps;
    t.vertical_speed=t.true_air_speed*sin(geometry.flight_path_angle_deg*DEG2RAD);
    t.flight_path_angle=geometry.flight_path_angle_deg;
    t.heading=t.ground_track_heading=geometry.course_deg;
    t.angle_of_attack=state->attitude.aoa_rad*RAD2DEG;
    t.pitch=t.flight_path_angle+t.angle_of_attack;
    t.roll=state->attitude.bank_rad*RAD2DEG;
    t.roll_rate=state->attitude.bank_rate_rad_s*RAD2DEG;
    t.mach=forces.mach;t.dynamic_pressure=forces.dynamic_pressure_pa;
    t.atmospheric_density=air.density_kg_m3;t.speed_of_sound=air.speed_of_sound_mps;
    t.lift_force=forces.lift_n;t.drag_force=forces.drag_n;
    t.g_force=fabs(forces.lift_n)/fmax(t.mass,1.0)/9.80665;
    t.estimated_ballistic_coefficient=m->vehicle.estimated_ballistic_coefficient;
    t.aerodynamic_confidence=1.0;
    snprintf(t.vessel_situation,sizeof(t.vessel_situation),"flying");
    LandingConfiguration cfg={.site=m->site,.vehicle=m->vehicle,.guidance=m->guidance};
    GuidanceMachine guidance=*seed;
    /* MM305 production enters the terminal routing mode before Final. The
       dedicated Final-test owner clears it when evaluating its own gates. */
    guidance.terminal_glide_mode=!seed->terminal_final_test_mode;
    guidance.terminal_test_glide_slope=0.0;
    guidance.terminal_test_final_approach_distance=0.0;
    guidance.terminal_final_handoff_latched=true;
    guidance.terminal_final_handoff_distance=m->guidance.final_approach_distance;
    guidance.terminal_final_handoff_slope_deg=m->guidance.final_glide_slope;
    guidance.terminal_final_handoff_speed_mps=m->guidance.final_alignment_speed;
    guidance.terminal_path_committed=guidance.mm305_route_committed=true;
    terminal_observe_landing_aero(&guidance,&t,&cfg.vehicle,planet_surface_gravity(&planet),.1);
    AerodynamicModel aero={cfg.vehicle.estimated_lift_to_drag,
        cfg.vehicle.estimated_ballistic_coefficient,1.0};
    TerminalPreflarePlan plan={0};
    bool approach=terminal_outer_capture_admissible(&guidance,&t,geometry.course_deg,
        &planet,aero,&cfg,&plan);
    contract=terminal_delivery_contract(&guidance,&t,geometry.course_deg,&planet,&cfg,&plan);
    TaemTerminalEvaluation verdict=taem_exec_evaluate_terminal_contract(&contract);
    if(evaluation)*evaluation=verdict;
    return approach&&verdict.valid&&verdict.feasible;
}
