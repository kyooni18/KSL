#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static EntryAlphaPhase entry_alpha_phase_for_exec(EntryPhase phase);
static void entry_supervision_models(const Telemetry*t,AerodynamicModel aero,
        AerodynamicEnvelope*env,TrajectoryCalibrationModel*cal);
static bool entry_program_bank_capture_available(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg);
static double entry_program_taem_path_fpa(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*distance_out);
static bool entry_program_terminal_side_final_s_turn_armed(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double course);
static double entry_program_final_s_turn_drag_aoa(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const EntryDragReferenceOutput*drag,
        bool final_s_turn_active);
static double entry_program_vertical_bank_ceiling(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
static bool entry_program_altitude_capture(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        double*bank,double*aoa);
static bool entry_program_shape_vertical_capture_plan(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        EntryControlPlan*plan);
static EntryBankAllocationSample entry_program_bank_allocation_sample(
        const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        const LandingConfiguration*cfg,const TaemInterfaceTarget*q,
        double course,double available_time,double lift,double drag,
        double effectiveness,double max_effective,double required_vertical,
        double available_energy,double bank);
static void entry_program_apply_geometry_bank_demand(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double geometry_bank,EntryControlPlan*plan);
static bool entry_program_longitudinal_bank_replan_due(const GuidanceMachine*g,
        const EntryControlPlan*demand,double ut,const GuidanceSettings*s,
        const EntryLateralLimits*limits);
static bool entry_program_planned_reversal_due(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const VehicleProfile*v,const GuidanceSettings*s);
static bool entry_program_schedule_live_setup_reversal(GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg,
        double reversal_bank_magnitude);
static bool entry_program_release_nonfinal_reversal_now(GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg);
static bool entry_program_terminal_course_endpoint_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign,
        double target_course,double*endpoint_error_out,double*radius_out);
static bool entry_program_rollthrough_endpoint_ready(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double final_sign,
        double*endpoint_error_out,double*lead_time_out);
static bool entry_program_final_reversal_geometry_ready(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double final_sign);
static bool entry_program_terminal_geometry_blocked(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg);
static double entry_taem_shaping_setup_radius(const TaemInterfaceTarget*q,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg);
static double entry_taem_shaping_parallel_cross_requirement(const LandingConfiguration*cfg);
static bool entry_program_supervised_reversal_ready(const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s);
static bool entry_program_execute_planned_reversal(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg);
static bool entry_program_replan_may_discard_reversal(bool plan_safety_replan,
        bool supervision_replan,bool supervision_valid,bool terminal_geometry_blocked);
static bool entry_interface_target_plan_identity_equal(
        const TaemInterfaceTarget*a,const TaemInterfaceTarget*b);
static bool entry_program_first_segment(const GuidanceMachine*g);
static void entry_program_seed_initial_side(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg);
static EntryControlPlan entry_program_plan_s_turn(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double nominal_bank_magnitude,double geometry_bank_magnitude,double nominal_aoa);
static bool entry_supervision_replan_due(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s);
static bool entry_bank_authority_replan_due(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v);
static double entry_bank_authority_child_duration(const EntryControlPlan*parent,
        double child_ut,double proposed_duration);
static bool entry_vertical_capture_growth_replan_due(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
static double entry_authority_refresh_child_duration(const GuidanceMachine*g,
        const EntryControlPlan*parent,double child_ut,double proposed_duration,
        const GuidanceSettings*s);
static bool entry_taem_pose_setup_point(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double*setup_along,double*setup_cross,double*final_side);
static void entry_taem_turnability_target(const GuidanceMachine*g,const Telemetry*t,
        double reference_course,AerodynamicModel aero,const LandingConfiguration*cfg,
        double ordinary_target,double*target_speed,double*target_course);
static bool entry_taem_alignment_station_missed(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*point_bearing);
static double entry_taem_tangent_capture_heading(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg);
static double entry_taem_tangent_capture_bank(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const LandingConfiguration*cfg,double desired_heading);
static double entry_taem_minimum_bank_for_capture(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double target_along,double target_cross,
        double target_course,double side);
static double entry_taem_gate_required_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
static double entry_taem_gate_acquisition_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
static bool entry_program_post_shaping_parallel_stage(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double*bank_out);
static bool entry_program_tangent_reversal_captured(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v);
static GuidanceResult entry_topology_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt);
static double entry_alpha_transition_velocity(const GuidanceSettings*s,double strict_taem_velocity);
static GuidanceResult entry_program_contract_hold(GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double dt,const char*warning);


       double live_lift_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v){
    double incidence=hypot(t->angle_of_attack,t->sideslip),lf=0;
    aerodynamic_force_factors_mach(t->mach,incidence,v,&lf,NULL);
    double modeled=fmax(0,t->dynamic_pressure)/fmax(20,aero.ballistic_coefficient)*fmax(0,aero.lift_to_drag)*fabs(lf);
    double measured=t->mass>1&&isfinite(t->lift_force)&&t->lift_force>0?t->lift_force/t->mass:0;
    return measured>.005?measured:modeled;
}
       double live_drag_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v){
    double incidence=hypot(t->angle_of_attack,t->sideslip),df=1;
    aerodynamic_force_factors_mach(t->mach,incidence,v,NULL,&df);
    double modeled=fmax(0,t->dynamic_pressure)/fmax(20,aero.ballistic_coefficient)*df;
    double measured=t->mass>1&&isfinite(t->drag_force)&&t->drag_force>0?t->drag_force/t->mass:0;
    return measured>.005?measured:modeled;
}


/* MM304 Entry stays one translation unit for static-symbol and numerical parity.
 * Source is partitioned by control responsibility rather than chronology. */
#include "entry/reference.inc"
#include "entry/bank_allocation.inc"
#include "entry/sturn_planning.inc"
#include "entry/topology.inc"
#include "entry/contract.inc"
