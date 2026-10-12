#include "guidance_internal.h"
#include "mm305_planning.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers used by the closed-loop Entry law. */
static EntryAlphaPhase entry_alpha_phase_for_exec(EntryPhase phase){
    switch(phase){
        case ENTRY_PHASE_PREENTRY:return ENTRY_ALPHA_PRE_ENTRY;
        case ENTRY_PHASE_TEMPERATURE_CONTROL:return ENTRY_ALPHA_TEMPERATURE_CONTROL;
        case ENTRY_PHASE_EQUILIBRIUM_GLIDE:return ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
        case ENTRY_PHASE_CONSTANT_DRAG:return ENTRY_ALPHA_CONSTANT_DRAG;
        case ENTRY_PHASE_TRANSITION:return ENTRY_ALPHA_TRANSITION;
        default:return ENTRY_ALPHA_PRE_ENTRY;
    }
}

static double entry_alpha_transition_velocity(const GuidanceSettings*s,double taem_velocity){
    return s?fmax(taem_velocity,s->taem_force_handoff_speed):taem_velocity;
}

static GuidanceResult entry_program_contract_hold(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double dt,const char*warning);

       double live_lift_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v){
    if(t->physics_sample_valid&&isfinite(t->mass)&&t->mass>DBL_MIN&&
       isfinite(t->lift_force)&&t->lift_force>=0.0)
        return t->lift_force/t->mass;

    double incidence=hypot(t->angle_of_attack,t->sideslip),lf=0.0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,&lf,NULL);
    if(!isfinite(aero.ballistic_coefficient)||aero.ballistic_coefficient<=DBL_MIN||
       !isfinite(aero.lift_to_drag)||aero.lift_to_drag<0.0)
        return 0.0;
    return fmax(0.0,t->dynamic_pressure)/aero.ballistic_coefficient*
        aero.lift_to_drag*fabs(lf);
}
/* A prospective sensor force is available inside the closed-loop rollout.
   It is never evidence of a real measurement or aerodynamic certification. */
double guidance_feedback_lift_accel(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v){
    if(g&&g->diagnostic_shadow&&t&&isfinite(t->mass)&&t->mass>DBL_MIN&&
       isfinite(t->lift_force)&&t->lift_force>=0.0)
        return t->lift_force/t->mass;
    return live_lift_accel(t,aero,v);
}

       double live_drag_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v){
    if(t->physics_sample_valid&&isfinite(t->mass)&&t->mass>DBL_MIN&&
       isfinite(t->drag_force)&&t->drag_force>=0.0)
        return t->drag_force/t->mass;

    double incidence=hypot(t->angle_of_attack,t->sideslip),df=1.0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,NULL,&df);
    if(!isfinite(aero.ballistic_coefficient)||aero.ballistic_coefficient<=DBL_MIN)
        return 0.0;
    return fmax(0.0,t->dynamic_pressure)/aero.ballistic_coefficient*fmax(0.0,df);
}

double entry_measured_drag_accel(const Telemetry*t){
    if(!t||!t->physics_sample_valid||!isfinite(t->mass)||t->mass<=DBL_MIN||
       !isfinite(t->drag_force)||t->drag_force<0.0)
        return NAN;
    return t->drag_force/t->mass;
}


/* Live Entry and async forecast acceptance share one translation unit. */
#include "entry/forecast.inc"
#include "entry/contract.inc"
