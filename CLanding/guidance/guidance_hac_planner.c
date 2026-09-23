#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static bool terminal_fixed_hac_authority(const Telemetry*t,const PlanetModel*p,
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

static bool terminal_fixed_hac_authority(const Telemetry*t,const PlanetModel*p,
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
    double lift=live_lift_accel(t,aero,v);
    /* The fixed-HAC command law raises incidence up to the vehicle AoA limit
       for lateral force (terminal_required_aoa_for_lateral_limit), so the
       authority gate must evaluate the lift reachable inside that envelope,
       not the lift at whatever incidence the handoff state happens to fly.
       The drag cost of that incidence is charged by the energy gate. */
    if(isfinite(lift)&&lift>.005){
        double current_lf=0.0;
        aerodynamic_force_factors_mach(t->mach,
            fmax(1.0,fabs(t->angle_of_attack)),v,&current_lf,NULL);
        double best_lf=fabs(current_lf);
        for(double aoa=0.0;aoa<=v->maximum_angle_of_attack+.01;aoa+=.5){
            double lf=0.0;
            aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,NULL);
            if(isfinite(lf))best_lf=fmax(best_lf,fabs(lf));
        }
        lift*=best_lf/fmax(.05,fabs(current_lf));
    }
    double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    double effectiveness=clampd(isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0,.35,1.8);
    double normal_limit=v->maximum_g_load*gravity;
    double effective_bank=clampd(bank_limit*effectiveness,0.0,
        nextafter(89.0,0.0))*DEG2RAD;
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
 * static helpers keep exact behavior. Production planning, Variant-B search,
 * diagnostics, and rehearsal control are separated by ownership below. */
#include "hac_planner/vertical_profile.inc"
#include "hac_planner/energy_pricing.inc"
#include "hac_planner/dynamic_selector.inc"
#include "hac_planner/variant_b_search.inc"
#include "hac_planner/diagnostic_preview.inc"
#include "hac_planner/latched_lead.inc"
#include "hac_planner/variant_b_control.inc"
