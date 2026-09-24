#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static double terminal_fixed_hac_projected_lift(const GuidanceMachine*g,
        const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v,
        double aoa){
    const TrajectoryCalibrationModel*cal=g&&g->entry_predictor_models_valid?
        &g->entry_predictor_calibration:NULL;
    if(cal&&cal->physics&&t&&v&&t->mass>1.0&&t->dynamic_pressure>0.0){
        bool observed=false;double confidence=0.0,uncertainty=0.0;
        Vector3 force=vessel_physics_force_best_estimate_config(cal->physics,
            t->dynamic_pressure,t->mach,aoa,t->sideslip,t->mass,t->gear,
            t->brakes,t->has_airbrakes?(t->airbrakes?1:0):0,aero,cal,v,
            &confidence,&observed,&uncertainty);
        if(isfinite(force.y))return fmax(0.0,force.y);
    }
    return terminal_projected_lift_accel_at_aoa(t,aero,v,aoa);
}

static double terminal_fixed_hac_projected_drag(const GuidanceMachine*g,
        const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v,
        double aoa,double fallback_anchor){
    const TrajectoryCalibrationModel*cal=g&&g->entry_predictor_models_valid?
        &g->entry_predictor_calibration:NULL;
    if(cal&&cal->physics&&t&&v&&t->mass>1.0&&t->dynamic_pressure>=0.0){
        bool observed=false;double confidence=0.0,uncertainty=0.0;
        Vector3 force=vessel_physics_force_best_estimate_config(cal->physics,
            t->dynamic_pressure,t->mach,aoa,t->sideslip,t->mass,t->gear,
            t->brakes,t->has_airbrakes?(t->airbrakes?1:0):0,aero,cal,v,
            &confidence,&observed,&uncertainty);
        if(isfinite(force.x)&&force.x>=0.0)return force.x;
    }

    /*
     * Cold-start fallback only: preserve one correction measured at the real
     * source state, then let stock-KSP Mach/AoA/atmosphere curves evolve it.
     * Re-anchoring each predicted point would silently fit the future to a
     * measurement that does not exist there.
     */
    if(!t||!v||!(aero.ballistic_coefficient>DBL_MIN)||
       !isfinite(t->atmospheric_density)||t->atmospheric_density<0.0||
       !isfinite(t->true_air_speed)||t->true_air_speed<0.0||
       !isfinite(fallback_anchor)||fallback_anchor<0.0)return NAN;
    double df=1.0;
    aerodynamic_force_factors_mach(t->mach,
        clampd(aoa,0.0,v->maximum_angle_of_attack),v,NULL,&df);
    if(!isfinite(df)||df<0.0)return NAN;
    return .5*t->atmospheric_density*t->true_air_speed*t->true_air_speed/
        aero.ballistic_coefficient*df*fallback_anchor;
}

static bool terminal_fixed_hac_authority(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        double hac_radius,
        double*required_lateral_out,double*available_lateral_out,
        double*minimum_radius_out);
static bool fixed_hac_candidate_vertical_profile(GuidanceMachine*out,
        const GuidanceMachine*source,const Telemetry*t,
        const HACTransitionPlan*plan,const VehicleProfile*v,
        const GuidanceSettings*s,const LandingSite*site,double side,
        double*response_time_out);
static bool fixed_hac_vertical_profile_executable(const GuidanceMachine*profile,
        const GuidanceSettings*s);
static bool terminal_fixed_hac_lead_reachable(const GuidanceMachine*g,
        const Telemetry*initial,const HACTransitionPlan*plan,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        const LandingSite*site,double target_speed,
        const GuidanceMachine*vertical_profile,Telemetry*out,
        double*work_out,double*course_error_out,double*minimum_lateral_margin_out);
static bool terminal_fixed_hac_energy_radius_impl(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const VehicleProfile*v,const GuidanceSettings*s,const LandingSite*site,
        const Telemetry*drag_reference,double requested_radius,double lead_length,
        double arc_angle,double*radius_out,
        double*margin_out,double*loss_out,double*available_out,
        double*required_out,FixedHacEnergyAudit*audit,
        const HACTransitionPlan*plan,const GuidanceMachine*vertical_profile,
        double fixed_profile_slope,double fixed_target_aoa);
static void terminal_variant_b_store_tuple(GuidanceMachine*g,
        const HACTransitionPlan*p);
static bool terminal_variant_b_load_tuple(const GuidanceMachine*g,
        HACTransitionPlan*out);
static void terminal_variant_b_store_vertical_profile(GuidanceMachine*g,
        const GuidanceMachine*profile);
static bool terminal_variant_b_load_vertical_profile(const GuidanceMachine*g,
        GuidanceMachine*out);
static bool terminal_variant_b_find_dynamic_plan(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double current_e,double current_n,
        double capture_course,HACTransitionPlan*plan_out,double*radius_out,
        double*authority_min_out,double*energy_radius_out,
        double*entry_speed_out,bool*provisional_out,
        GuidanceMachine*vertical_profile_out);
static bool terminal_variant_b_project_staging_state(const Telemetry*initial,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double course,double horizon,
        double initial_e,double initial_n,Telemetry*out,double*e_out,
        double*n_out,double*distance_out);
static bool terminal_variant_b_preview_future_hac(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double current_e,double current_n,
        double capture_course);
static HACGuidance terminal_variant_b_latched_lead_guidance(
        const HACTransitionPlan*p,const Telemetry*t,double gravity,double course,
        double hac_radius,double current_e,double current_n);

static bool terminal_fixed_hac_authority(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,
        double hac_radius,
        double*required_lateral_out,double*available_lateral_out,
        double*minimum_radius_out){
    if(required_lateral_out)*required_lateral_out=NAN;
    if(available_lateral_out)*available_lateral_out=NAN;
    if(minimum_radius_out)*minimum_radius_out=INFINITY;
    if(!t||!p||!v||!s||!(hac_radius>0.0))return false;
    double speed=guidance_lateral_speed(t);
    double gravity=planet_surface_gravity(p);
    /* Project the force-book cell at this q, Mach and incidence. */
    double lift=terminal_fixed_hac_projected_lift(g,t,aero,v,
        v->maximum_angle_of_attack);
    double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double normal_limit=v->maximum_g_load*gravity;
    double effective_bank=clampd(bank_limit*effectiveness,0.0,
        nextafter(90.0,0.0))*DEG2RAD;
    double available_normal=fmin(fmax(0.0,lift),fmax(0.0,normal_limit));
    double available_lateral=available_normal*sin(effective_bank);
    double required_lateral=speed*speed/hac_radius;
    double minimum_radius=available_lateral>DBL_MIN?
        speed*speed/available_lateral:INFINITY;
    if(required_lateral_out)*required_lateral_out=required_lateral;
    if(available_lateral_out)*available_lateral_out=available_lateral;
    if(minimum_radius_out)*minimum_radius_out=minimum_radius;
    if(!isfinite(required_lateral)||!isfinite(available_lateral)||
       !(available_lateral>0.0))return false;
    return required_lateral<=available_lateral+
        sqrt(DBL_EPSILON)*fmax(1.0,available_lateral);
}


/* HAC planning remains one translation unit so the current selector and its
 * static helpers keep exact behavior. Shared targets/selection geometry are
 * separated from path search, diagnostics, and control by responsibility. */
#include "hac_planner/selection_geometry.inc"
#include "hac_planner/energy_targets.inc"
#include "hac_planner/lead_dynamics.inc"
#include "hac_planner/vertical_profile.inc"
#include "hac_planner/energy_pricing.inc"
#include "hac_planner/dynamic_selector.inc"
#include "hac_planner/variant_b_search.inc"
#include "hac_planner/diagnostic_preview.inc"
#include "hac_planner/latched_lead.inc"
#include "hac_planner/variant_b_control.inc"
