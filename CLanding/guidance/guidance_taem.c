#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers; TAEM source slices are compiled below. */
static double terminal_expected_speed_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v);
static TaemExecInputs taem_exec_inputs_live_with_contract(const GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const TaemTerminalContract*terminal_contract,
        bool has_final_approach_override,bool final_approach_override);
static double terminal_drag_budget_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double allowed_drag,double preferred_aoa);
static double terminal_preview_recovery_aoa(double preferred_aoa,double budget_aoa,
        double reference_fpa,double actual_fpa);
static double terminal_required_aoa_for_lateral(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg);
static double terminal_atmosphere_vertical_resolution(const PlanetModel*p,
        double lower_altitude,double upper_altitude);
static double terminal_integrated_energy_path(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,double target_aoa,
        double available_specific_energy,double slope_deg,double min_path,double max_path,
        EnergyPathEnvelope*out_envelope);
static double terminal_uncommitted_alignment_heading(const Telemetry*t,
        const LandingConfiguration*cfg);
static double terminal_uncommitted_gate_geometry(const GuidanceMachine*g,
        const Telemetry*t,const LandingConfiguration*cfg,double*fpa);
static double terminal_test_energy_selected_slope(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        double*preferred_total_path,EnergyPathEnvelope*out_envelope);
static double terminal_test_projected_turn_radius(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double bank_deg);
static double hac_oriented_progress_rate(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double hac_radius,double course);
static bool hac_handoff_geometry_valid(const HACTransitionPlan*join,double e,double n,
        double course,const LandingSite*site,const GuidanceSettings*s,double radius,double side);
static void terminal_candidate_execution_cost(TerminalCandidate*c,
        const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
static bool terminal_test_start_spiral(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg);
static bool terminal_candidate_geometry_regular(const TerminalCandidate*c);
static bool terminal_candidate_geometry_executable(const TerminalCandidate*c);
static double terminal_candidate_delivery_time(const TerminalCandidate*c);
static double terminal_candidate_constraint_violation(const TerminalCandidate*c);
static int terminal_candidate_constraint_order(const TerminalCandidate*a,
        const TerminalCandidate*b);
static int terminal_candidate_execution_order(const TerminalCandidate*a,
        const TerminalCandidate*b);
static bool terminal_candidate_better(const TerminalCandidate*a,const TerminalCandidate*b);
static bool terminal_candidate_refinement_ok(const TerminalCandidate*a,
        const TerminalCandidate*b);
static bool terminal_candidate_refresh_allowed(const GuidanceMachine*g,
        const TerminalCandidate*a,const TerminalCandidate*b);
static bool terminal_candidate_energy_reachable(const TerminalCandidate*c,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg);
static bool terminal_candidate_expired(const GuidanceMachine*g,
        const TerminalCandidate*c,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg);
static void terminal_project_future_aero_sample(Telemetry*future,const Telemetry*current,
        const PlanetModel*p,const VehicleProfile*v);
static void terminal_projected_force_accels(const Telemetry*current,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double reference_drag,
        double altitude,double speed,double angle_of_attack,double*lift_accel,double*drag_accel);
static void terminal_project_planning_state(const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double horizon,
        Telemetry*future,double*out_e,double*out_n,double*out_course);
static bool terminal_spline_candidate_from_future(const GuidanceMachine*g,const Telemetry*live,
        const Telemetry*future,double future_course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,TerminalCandidate*out);
static bool terminal_candidate_probe_commit_ready(const GuidanceMachine*g,
        const TerminalCandidate*c,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg);
static bool terminal_candidate_from_future(const GuidanceMachine*g,const Telemetry*live,
        const Telemetry*future,double future_course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,TerminalCandidate*out);
static bool terminal_future_origin_reachable(const GuidanceMachine*g,
        const Telemetry*live,double course,const Telemetry*future,
        double future_course,double horizon,const PlanetModel*p,
        const LandingConfiguration*cfg);
static double terminal_min_glide_work(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double straight_path,
        double arc_path,double arc_radius,double speed);
static bool terminal_candidate_live_energy_ready(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TerminalCandidate*c);
static bool terminal_live_origin_hac_candidate(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,TerminalCandidate*out);
static bool terminal_candidate_vertical_response_ready_from_arrival(
        const GuidanceMachine*g,const Telemetry*arrival,double arrival_course,
        const PlanetModel*p,const TerminalCandidate*c,
        const LandingConfiguration*cfg,double*drop_out,double*height_out);
static bool terminal_interface_target_spatially_relevant(const TaemInterfaceTarget*q,
        const Telemetry*t,const LandingConfiguration*cfg);
static void terminal_publish_interface_target(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg);
static double terminal_prediction_refresh_period(const GuidanceMachine*g,
        const GuidanceSettings*s);
static double terminal_test_speed_shortfall(const Telemetry*t,double target_speed);
static double terminal_test_energy_gamma_bias(const Telemetry*t,double target_speed);
static double terminal_test_energy_trim(const Telemetry*t,double target_speed,double nominal_aoa);
static void terminal_test_energy_limit_aoa(GuidanceCommand*c,const Telemetry*t,
        double target_speed,double nominal_aoa);
static double terminal_taem_drag_aoa_ceiling(const Telemetry*t,const VehicleProfile*v,
        double expected_drag,double drag_budget,double nominal_aoa);
static double terminal_lateral_aoa_floor(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg);
static bool terminal_energy_path_unrecoverable(double expected_loss,double gravity,
        double kinetic_to_gate,double air_path,double recoverable_fpa_deg);
static bool terminal_vertical_path_unrecoverable(double height_to_gate,
        double ground_path_to_gate,double recoverable_fpa_deg);
static bool terminal_vertical_energy_priority(double flight_path_angle,double target_fpa,
        double expected_loss,double drag_budget);
static bool terminal_lateral_incidence_priority(const GuidanceMachine*g,bool energy_priority);


/* Internal source slices; compiled as this single translation unit. */
#include "taem/state.inc"
#include "taem/energy.inc"
#include "taem/executive_bridge.inc"
#include "taem/rehearsal.inc"
#include "taem/candidates.inc"
#include "taem/preview.inc"
#include "taem/control.inc"
