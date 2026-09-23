#ifndef KSP_LANDER_GUIDANCE_INTERNAL_H
#define KSP_LANDER_GUIDANCE_INTERNAL_H

/* Private interfaces shared by the modular guidance implementation. */
#include "landing.h"
#include "decision_envelope.h"
#include "entry_alpha.h"
#include "entry_drag_reference.h"
#include "taem_exec.h"

typedef struct {
    HACGuidance guidance;
    double path_progress;
    bool valid;
    bool handoff_ready;
} HACPhaseGuidance;

typedef struct {
    bool valid;
    double vertical_score;
    double lateral_score;
    double capture_time_s;
} EntryBankAllocationSample;

typedef struct {
    bool valid;
    double along_track, cross_track, course, altitude, speed, specific_energy;
    double minimum_speed, minimum_altitude, minimum_flight_path_angle;
    double peak_dynamic_pressure_ratio, peak_g_ratio;
} TaemSTurnProjection;

typedef struct {
    bool valid;
    double safety_violation;
    double energy_deficit;
    double energy_excess;
    double position_error;
    double course_error;
    double altitude_error;
    double speed_error;
    double control_effort;
} TaemSTurnRank;

typedef struct {
    bool feasible;
    const char *reject_reason;
    double trigger_altitude;
    double target_aoa;
    double target_sink;
    double minimum_speed;
    double reference_speed;
    double predicted_height_loss;
    double predicted_kinetic_margin;
    double effective_accel;
    double response_time;
    DecisionMargin height;
} TerminalPreflarePlan;

typedef struct {
    bool valid;
    unsigned long long id;
    double radius,lead_length,arc_length,final_path_length,total_path_length;
    double profile_slope,target_aoa;
    double altitude,speed,fpa,specific_energy;
    double gate_specific_energy,available,available_expected,available_residual;
    double modeled_work,nominal_margin,uncertainty,required,margin;
    unsigned long long candidate_id;
    double candidate_lead_length,lead_work,lead_energy_closure;
    double exit_altitude,exit_speed,exit_specific_energy,closure;
    double arc_length_expected,arc_work_shadow,final_work_shadow;
    double arc_mean_drag_accel,partition_residual,modeled_work_residual,
        required_work_residual,arc_energy_closure,final_energy_closure;
    double drag_anchor,arc_end_altitude,arc_end_speed;
    double altitude_residual,speed_residual,range_residual;
    double altitude_uncertainty,speed_uncertainty,range_uncertainty;
    double physics_relative_uncertainty,physics_relative_uncertainty_raw;
    double physics_uncertainty;
    double uncertainty_reconstructed,uncertainty_residual;
} FixedHacEnergyAudit;

double hac_course_rate_cap_for_speed(double true_air_speed);
double hac_course_rate_cap(const Telemetry*t);
double guidance_lateral_speed(const Telemetry*t);
void taem_capture_veto_reasons(unsigned veto,char *buf,size_t n);
double hac_join_rate_cap(const Telemetry*t,double progress);
HACPoint2 hac_point(double e,double n);
HACPoint2 hac_bezier_point(const HACTransitionPlan*p,double u);
HACPoint2 hac_bezier_derivative(const HACTransitionPlan*p,double u);
double hac_bezier_signed_curvature(const HACTransitionPlan*p,double u);
bool hac_bezier_regular(const HACTransitionPlan*p);
double hac_bezier_length_between(const HACTransitionPlan*p,double from,double to,int segments);
double hac_transition_nearest_u(const GuidanceMachine*g,double e,double n);
double hac_bezier_nearest_u(const HACTransitionPlan*p,double e,double n);
HACTransitionPlan hac_lead_bezier_from_plan(const HACTransitionPlan*p);
double hac_bezier_advance_distance(const HACTransitionPlan*p,double u,double distance);
double fixed_hac_lead_remaining(const GuidanceMachine*g);
double hac_path_tracking_time(double speed,double curvature_radius);
double hac_frenet_lateral(double speed,double curvature,double cross_track,
        double course_error_deg,double tracking_time);
double controlled_roll_rate(const Telemetry*t);
double controlled_aoa_rate(const Telemetry*t);
GuidanceAttitudeLimits stabilized_attitude_limits(
        const Telemetry *t, const GuidanceSettings *s, GuidancePhase phase,
        bool pitch_axis);
void seed_stabilized_limiter(JerkLimiter *limiter, double value,
        double rate, bool angle);
double roll_capture_time(const Telemetry*t,double target_bank,
        double configured_rate_limit,double configured_accel_limit);
double hac_response_lead_time(const Telemetry*t,const GuidanceSettings*s,double nominal_bank);
void hac_projected_local_state(const Telemetry*t,const LandingSite*site,double planet_radius,
        double course,double seconds,double target_bank,double roll_rate_limit,double roll_accel_limit,
        double lift_accel,double bank_effectiveness,double drag_accel,
        double*out_e,double*out_n,double*out_course);
double terminal_normalized_upper_violation(double value,double limit);
double terminal_normalized_band_violation(double value,double minimum,double maximum);
bool hac_transition_plan(HACTransitionPlan*out,double current_e,double current_n,
        double future_e,double future_n,double start_course,double future_course,
        const LandingSite*site,const GuidanceSettings*s,double hac_radius,double side,
        double start_air_speed,double end_air_speed,double speed_loss_accel,
        double max_lateral_accel,double preferred_fixed_path,
        double min_fixed_path,double max_fixed_path);
bool hac_variant_b_requested(void);
bool hac_upstream_runway_staging_requested(void);
double hac_upstream_aoa_bias_deg(void);
double hac_lead_aoa_bias_deg(void);
bool hac_mm305_dynamic_fixture_requested(void);
bool hac_fixed_alignment_geometry(HACPoint2*entry_out,
        HACPoint2*center_out,HACPoint2*exit_out,const LandingSite*site,
        const GuidanceSettings*s,double hac_radius,double capture_course,
        double side);
bool hac_fixed_alignment_plan(HACTransitionPlan*out,
        double current_e,double current_n,double start_course,double capture_course,
        const LandingSite*site,const GuidanceSettings*s,double hac_radius,double side);
void hac_transition_store(GuidanceMachine*g,const HACTransitionPlan*p,double response_time);
bool fixed_hac_vertical_profile_latch(GuidanceMachine*g,
        double start_altitude,double start_flight_path_angle,
        double start_horizontal_speed,double response_time,
        double lead_length,double arc_length,
        const LandingSite*site,const GuidanceSettings*s);
double fixed_hac_total_path_length(const GuidanceMachine*g);
double fixed_hac_reference_slope(const GuidanceMachine*g,
        double remaining);
double fixed_hac_reference_altitude(const GuidanceMachine*g,
        double remaining);
bool hac_fixed_lead_rebase_from_vector(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        const LandingConfiguration*cfg,double current_e,double current_n);
HACGuidance hac_guidance_radius(const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double gravity,double course,double hac_radius);
HACGuidance terminal_runway_line_path_guidance(const Telemetry*t,
        const LandingSite*site,const GuidanceSettings*s,double gravity,double course);
HACGuidance hac_heading_cone_path_guidance(const GuidanceMachine*g,
        const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,
        double planet_radius,double gravity,double course);
HACGuidance hac_path_guidance(const GuidanceMachine*g,const Telemetry*t,
        const LandingSite*site,const GuidanceSettings*s,double planet_radius,
        double side,double gravity,double course,double hac_radius);
HACPhaseGuidance hac_phase_guidance(TaemPhase phase,const GuidanceMachine*g,
        const Telemetry*t,const LandingSite*site,const GuidanceSettings*s,double planet_radius,
        double side,double gravity,double course,double hac_radius);
void reference_trajectory_radius(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double radius,double side,double hac_radius);
void reference_trajectory_fixed_hac(Trajectory*out,const LandingSite*site,const GuidanceSettings*s,double planet_radius,const GuidanceMachine*g);
GuidanceResult result_make(GuidancePhase phase,GuidanceCommand c,const char*status,const char*warning);
void guidance_result_clear(GuidanceResult*r);
void reset_limiters(GuidanceMachine*g);
void reset_taem_handoff_state(GuidanceMachine*g);
void guidance_machine_init(GuidanceMachine*g);
void control_plan_assign_lineage(GuidanceMachine*g,EntryControlPlan*plan,
        const EntryControlPlan*parent);
void guidance_set_engaged(GuidanceMachine*g,bool e);
void guidance_abort(GuidanceMachine*g);
double dynamic_bank_limit(const Telemetry*t,const VehicleProfile*v);
double entry_survivability_recovery_aoa(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,const LandingConfiguration*cfg,
        const ControlAuthorityEnvelope*authority);
void terminal_speedbrake_closed(GuidanceMachine*g);
GuidanceCommand atmospheric(const Telemetry*t,double heading,double roll,const VehicleProfile*v,double throttle,bool air,ControlProfile profile);
void aerodynamic_pitch_target(GuidanceCommand*c,const Telemetry*t,const VehicleProfile*v,double surface_pitch);
GuidanceResult stabilized(GuidanceMachine*g,GuidanceResult r,const Telemetry*t,const VehicleProfile*v,const GuidanceSettings*s,double dt);
GuidanceCommand entry_capture(const Telemetry*t,const VehicleState*state,const VehicleProfile*v);
void request_side(GuidanceMachine*g,double sign,double ut);
void entry_update_s_turn_leg_capture(GuidanceMachine*g,const Telemetry*t,double bank,const VehicleProfile*v);
EntryControlPlan guidance_terminal_control_plan(const GuidanceMachine*g,EntryControlPlan plan);
double live_lift_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v);
double live_drag_accel(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v);
void entry_program_commit_planned_reversal(GuidanceMachine*g,EntryControlPlan*plan,
        double ut,bool replace_existing);
bool entry_taem_tangent_target_geometry(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg);
TaemInterfaceCapture entry_dynamic_interface_capture(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg);
GuidanceResult entry_program_guidance(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt);
double live_turn_radius(const Telemetry*t,AerodynamicModel aero,const VehicleProfile*v,double bank_deg);
double terminal_hac_radius_live(const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s);
void terminal_glide_initialize(GuidanceMachine*g,const VehicleProfile*v,const GuidanceSettings*s);
void terminal_energy_observe(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p);
double terminal_expected_energy_loss_accel(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v);
void hac_energy_audit_observe(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        double dt);
void hac_energy_audit_close(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p);
double terminal_drag_projection_anchor(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const VehicleProfile*v);
bool taem_exec_enter(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        bool checkpoint_resume);
bool taem_exec_sync_with_contract(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TaemTerminalContract*terminal_contract,bool has_final_approach_override,
        bool final_approach_override);
void taem_exec_sync(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
GuidanceResult taem_s_turn_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt);
double terminal_default_hac_side(const Telemetry*t,const LandingSite*site,double course);
double terminal_hac_energy_aoa(const VehicleProfile*v);
double terminal_fpa_force_aoa(const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,double reference_fpa);
double fixed_hac_lead_target_speed(const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v,double hac_radius);
double fixed_hac_lead_drag_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double target_speed,double remaining_distance,
        double reference_slope_deg);
double fixed_hac_projected_drag_accel(const Telemetry*t,
        AerodynamicModel aero,const VehicleProfile*v,double aoa);
double terminal_required_aoa_for_lateral_limit(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double lateral_accel,double bank_deg,double maximum_aoa);
bool terminal_quadratic_drag_energy_step(double speed,double drag,
        double air_distance,double potential_drop,double*next_speed,double*work);
double terminal_projected_drag_work_state_anchored(const GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const VehicleProfile*v,const GuidanceSettings*s,double target_aoa,
        double ground_path,double slope_deg,double*end_speed_out,
        double*end_altitude_out,double inherited_anchor);
double terminal_projected_drag_work_state(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double target_aoa,double ground_path,double slope_deg,
        double*end_speed_out,double*end_altitude_out);
double terminal_projected_drag_work(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const VehicleProfile*v,
        const GuidanceSettings*s,double target_aoa,double ground_path,
        double slope_deg);
void terminal_path_slope_bounds(const Telemetry*t,const VehicleProfile*v,
        const GuidanceSettings*s,double*minimum,double*maximum);
void terminal_publish_candidate(GuidanceMachine*g);
bool terminal_prediction_ready(const GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,const LandingConfiguration*cfg);
bool terminal_candidate_operationally_usable(const GuidanceMachine*g,
        const TerminalCandidate*c,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg);
bool terminal_capture_margin_exhausted(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg);
bool terminal_candidate_vertical_response_ready_live(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const TerminalCandidate*c);
void terminal_predict(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt);
bool terminal_candidate_commit_ready(GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
HACGuidance terminal_test_update_spiral(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt);
double terminal_test_speed_floor(const VehicleProfile*v);
GuidanceSettings terminal_path_settings(const GuidanceMachine*g,const GuidanceSettings*s);
double terminal_projected_lift_accel_at_aoa(const Telemetry*t,AerodynamicModel aero,
        const VehicleProfile*v,double aoa);
double taem_bank_demand(const Telemetry*t,const HACGuidance*h,double hac_radius,double side,AerodynamicModel aero,const VehicleProfile*v);
GuidanceResult taem_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt);
double terminal_outer_glide_slope(const GuidanceMachine*g,const GuidanceSettings*s);
void terminal_store_preflare_plan(GuidanceMachine*g,const TerminalPreflarePlan*p);
void terminal_set_stage(GuidanceMachine*g,TerminalVerticalStage stage,double ut);
bool terminal_outer_gate(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        TerminalPreflarePlan*out_plan);
bool terminal_outer_capture_admissible(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        TerminalPreflarePlan*out_plan);
TaemTerminalContract terminal_delivery_contract(const GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,const LandingConfiguration*cfg,
        const TerminalPreflarePlan*plan);
bool terminal_final_conditioning_reachable(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,EnergyPathEnvelope*energy_out,double*end_speed_out,
        double*slope_out);
bool terminal_runway_contact_position(const Telemetry*t,const LandingConfiguration*cfg);
GuidanceResult final_guidance(GuidanceMachine*g,const Telemetry*t,double course,const PlanetModel*p,const LandingConfiguration*cfg,const Trajectory*ref,double dt);
GuidanceResult touchdown(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,const Trajectory*ref);
GuidanceResult terminal_abort(GuidanceMachine *g, const char *reason);
GuidanceResult terminal_approach_sequence(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,const Trajectory*ref,double dt);
bool control_recovery_needed(GuidanceMachine *g, const Telemetry *t,
        const GuidanceSettings *s, const VehicleProfile *v, double dt);
bool terminal_entry_recovery_available(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const GuidanceSettings*s);
void terminal_invalidate_frozen_path(GuidanceMachine*g);
void terminal_invalidate_frozen_path_preserve_energy(GuidanceMachine*g);
GuidanceResult terminal_return_to_entry(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt);
bool terminal_hac_debug_force_requested(void);
bool terminal_hac_debug_fixture_state(const Telemetry*t,double course,
        const LandingConfiguration*cfg,double*capture_course_out);
bool terminal_force_acquisition_with_aero(GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg);
GuidanceResult terminal_variant_b_upstream_guidance(GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt,bool*capture_ready);

extern double terminal_energy_commit_tolerance;
GuidanceResult guidance_update_impl(GuidanceMachine*g,const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);

#endif
