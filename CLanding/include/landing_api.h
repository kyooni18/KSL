#ifndef KSP_LANDER_API_H
#define KSP_LANDER_API_H

#include "vehicle_types.h"

typedef struct TerminalModel TerminalModel;

void vessel_physics_init(VesselPhysicsModel *m);
void vessel_physics_import_sample(VesselPhysicsModel *m,const VesselAeroSample *sample);
void vessel_physics_observe(VesselPhysicsModel *m, Telemetry *t, const VehicleState *s, const PlanetModel *p);
bool vessel_physics_aero(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,Vector3 *specific,double *confidence);
bool vessel_physics_aero_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,Vector3 *specific,double *confidence);
Vector3 vessel_physics_gravity(Vector3 position,const PlanetModel *p);
Vector3 vessel_physics_force(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed);
Vector3 vessel_physics_force_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed);
Vector3 vessel_physics_force_best_estimate_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed,double *relative_uncertainty);
bool vessel_physics_derive_envelope(VesselPhysicsModel *m,const VehicleProfile *baseline,AerodynamicEnvelope *envelope,AerodynamicModel *planning);
Vector3 vessel_physics_acceleration(Vector3 position,Vector3 air_velocity,const PlanetModel *p,Vector3 specific_force,double bank);
void vessel_physics_environment(const PlanetModel *p,const TrajectoryCalibrationModel *cal,double altitude,double *density,double *sound);
double vessel_physics_conservative_stall_fraction(double true_air_speed,double angle_of_attack,double dynamic_pressure,const VehicleProfile *vehicle);
double vessel_physics_burn_mass(const VesselPhysicsModel *m,double mass,double thrust,double dt);
void vessel_physics_axis_step(const VesselPhysicsModel *m,int axis,double target,double q,double rate_limit,double dt,double *angle,double *rate);
void landing_snapshot_set_sample(LandingSnapshot *snapshot,const Telemetry *t,const VehicleState *state);

/* math/navigation */
double clampd(double v, double lo, double hi); double norm_deg(double a); double norm_signed_deg(double a);
double entry_guidance_start_altitude(const PlanetModel *planet,const GuidanceSettings *settings);
void entry_taem_handoff_altitude_bounds(const GuidanceSettings *settings,double *minimum,double *maximum);
bool entry_taem_handoff_geometry_ready(double altitude,double runway_along_track,double vertical_speed,double horizontal_speed,const GuidanceSettings *settings);
bool entry_s_turn_bank_authority_available(double dynamic_pressure,double true_air_speed,double stall_fraction,double g_force,const VehicleProfile *vehicle);
double entry_bank_authority_limit(double true_air_speed,double dynamic_pressure,double g_force,const VehicleProfile *vehicle,double maximum_bank);
double entry_taem_range_target(const PlanetModel *planet,const GuidanceSettings *settings);
bool mm305_acquisition_ready(const Telemetry *telemetry,const PlanetModel *planet,const LandingConfiguration *configuration);
double rotating_specific_energy(double latitude,double altitude,double air_relative_speed,const PlanetModel *planet);
double entry_remaining_specific_energy(double latitude,double altitude,double air_relative_speed,double target_latitude,double target_altitude,double target_speed,const PlanetModel *planet);
double entry_altitude_target_for_speed(double reference_altitude,double reference_speed,double true_air_speed,double target_altitude,double target_speed);
double entry_altitude_target_for_range(double reference_altitude,double reference_range,double range,double target_altitude,double target_range);
double entry_thermal_protection_aoa_floor(const VehicleProfile *vehicle);
double entry_low_q_protective_aoa_floor(double dynamic_pressure,const VehicleProfile *vehicle);
double entry_terminal_turn_aoa_floor(double true_air_speed,double dynamic_pressure,const VehicleProfile *vehicle);
double entry_final_s_turn_aoa_ceiling(const VehicleProfile *vehicle);
EntryTerminalDemand entry_terminal_demand(double latitude,double altitude,double range,double horizontal_speed,double course_error,double vertical_speed,double true_air_speed,double entry_reference_speed,const PlanetModel *planet,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings);
Vector3 v3(double x,double y,double z); Vector3 vadd(Vector3 a,Vector3 b); Vector3 vsub(Vector3 a,Vector3 b);
Vector3 vscale(Vector3 a,double s); double vdot(Vector3 a,Vector3 b); Vector3 vcross(Vector3 a,Vector3 b);
double vmag(Vector3 a); Vector3 vnorm(Vector3 a, Vector3 fallback); Vector3 vproject_plane(Vector3 a, Vector3 normal);
Vector3 vrotate(Vector3 a, Vector3 axis, double radians);
double great_circle_distance(GeoPoint a, GeoPoint b, double radius); double initial_bearing(GeoPoint a, GeoPoint b);
GeoPoint destination_point(GeoPoint a,double bearing,double distance,double radius); void local_offsets(GeoPoint origin,GeoPoint p,double radius,double *east,double *north);
GeoPoint runway_approach_aimpoint(const LandingSite *site,double radius,double distance_before_threshold);
GeoPoint local_point(GeoPoint origin,double east,double north,double radius,double altitude); double surface_course(Vector3 pos,Vector3 vel,Vector3 omega,Vector3 north,double fallback);
void runway_coordinates(GeoPoint point, GeoPoint site, double heading, double radius, double *along, double *cross);
double taem_interface_line_heading(double runway_along,double runway_cross,double runway_heading,
    const TaemInterfaceTarget *target);
double taem_course_rate_bank_command(double horizontal_speed,double lift_accel,double bank_effectiveness,
    double bank_limit,double current_course,double desired_course,double response_time);
LandingSite runway_reciprocal_site(const LandingSite *primary,double radius);
void telemetry_reframe_runway(Telemetry *telemetry,const LandingSite *site,double radius);
double bank_for_point_capture(double speed,double lift_accel,double bank_effectiveness,double course_error,double distance,double maximum_bank);
HACGuidance hac_guidance_compute(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double radius,double side,double gravity);
double hac_guidance_score(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double radius,double side,double gravity);
HACGuidance hac_guidance_compute_radius(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double planet_radius,double side,double gravity,double hac_radius);
double hac_guidance_score_radius(GeoPoint current,double true_air_speed,double course,const LandingSite *site,const GuidanceSettings *settings,double planet_radius,double side,double gravity,double hac_radius);
bool hac_entry_capture_geometry_ready(const HACGuidance *guidance,double hac_radius);
bool hac_high_pass_circuit(double altitude, double speed, double hac_radius, double remaining,
    double minimum_turn_radius, double drag_accel, const PlanetModel *planet,
    const LandingSite *site, const VehicleProfile *vehicle, const GuidanceSettings *settings,
    double *new_remaining, double *glide_slope);
bool taem_alignment_maneuver_geometry(double altitude, double radius,
    double minimum_turn_radius, double alignment_turn_deg, const LandingSite *site,
    const GuidanceSettings *settings, double *remaining_path, double *glide_slope,
    double *final_altitude);
bool taem_alignment_maneuver_feasible(double altitude, double speed, double radius,
    double minimum_turn_radius, double drag_accel, double alignment_turn_deg, const PlanetModel *planet,
    const LandingSite *site, const VehicleProfile *vehicle, const GuidanceSettings *settings,
    double *remaining_path, double *glide_slope);
bool terminal_approach_valid(GeoPoint current,double range_to_site,double runway_along,double runway_cross,double course,double true_air_speed,double horizontal_speed,double vertical_speed,double flight_path_angle,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings);
void robust_pid_init(RobustPID *p,double kp,double ki,double kd,double limit,double tc); void robust_pid_reset(RobustPID *p); double robust_pid_update(RobustPID *p,double error,double dt,double lo,double hi);
double lowpass_update(LowPass *f,double value,double dt); void lowpass_reset(LowPass *f);
double jerk_update(JerkLimiter *j,double target,double max_rate,double max_accel,double dt); double jerk_angle_update(JerkLimiter *j,double target,double max_rate,double max_accel,double dt);
#define GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS 0.08
double burn_fraction(double elapsed,double remaining,double max_accel,double ramp); double burn_estimated_duration(double dv,double max_accel,double ramp);
FlightControlRegime flight_control_regime(double mach,double true_air_speed,double dynamic_pressure,double minimum_safe_speed);
double entry_s_turn_effective_minimum_leg(double true_air_speed,double taem_speed,const GuidanceSettings *settings);
void speedbrake_controller_reset(SpeedbrakeController *c,bool deployed);
bool speedbrake_controller_update(SpeedbrakeController *c,double dynamic_pressure,double target_dynamic_pressure,double specific_energy_error,double energy_scale,double maximum_dynamic_pressure,bool inhibit,double dt);

/* models/serialization */
LandingConfiguration landing_configuration_default(void); void landing_configuration_normalize(LandingConfiguration *c);
bool landing_configuration_from_json(LandingConfiguration *c,const JsonDoc *doc,int index); void landing_configuration_json(JsonWriter *w,const LandingConfiguration *c); void planet_model_replay_json(JsonWriter *w,const PlanetModel *p);
void telemetry_init(Telemetry *t); void guidance_command_init(GuidanceCommand *c); void calibration_snapshot_init(CalibrationSnapshot *c); void landing_snapshot_init(LandingSnapshot *s,const VehicleProfile *p);
const char *connection_status_string(ConnectionStatus s); const char *phase_string(GuidancePhase p); const char *speed_mode_string(NavballSpeedMode s); const char *profile_string(ControlProfile p); const char *cal_state_string(CalibrationRunState s);
void trajectory_init(Trajectory *t); void trajectory_clear(Trajectory *t); bool trajectory_append(Trajectory *t,TrajectoryPoint p); bool trajectory_copy(Trajectory *dst,const Trajectory *src);
void deorbit_plan_clear(DeorbitPlan *p); void snapshot_json(JsonWriter *w,const LandingSnapshot *s); void snapshot_log_json(JsonWriter *w,const LandingSnapshot *s); void trajectory_json(JsonWriter *w,const Trajectory *t);

/* prediction/guidance */
Vector3 planet_rotation_vector(const PlanetModel *p); double planet_surface_gravity(const PlanetModel *p);
double planet_atmospheric_density(const PlanetModel *p,double altitude);
double planet_atmospheric_pressure(const PlanetModel *p,double altitude);
double planet_atmospheric_speed_of_sound(const PlanetModel *p,double altitude);
void aerodynamic_force_factors(double angle_of_attack,const VehicleProfile *vehicle,double *lift_factor,double *drag_factor);
void aerodynamic_force_factors_mach(double mach,double angle_of_attack,const VehicleProfile *vehicle,double *lift_factor,double *drag_factor);
double aerodynamic_best_glide_aoa(double mach,const VehicleProfile *vehicle);
GeoPoint predictor_geo_point(Vector3 position,const PlanetModel *planet,double ut); Vector3 predictor_inertial_position(GeoPoint point,const PlanetModel *planet,double ut);
VehicleState predictor_propagate_vacuum(VehicleState state,double target_ut,const PlanetModel *planet,double max_step);
double predictor_postburn_periapsis(VehicleState state,const PlanetModel *planet);
double predictor_directed_taem_range_error(double range,double closest_distance,double target_range);
double entry_taem_speed_target(const VehicleProfile *vehicle,const GuidanceSettings *settings,const PlanetModel *planet);
double hac_acquisition_speed_target(const VehicleProfile *vehicle,const GuidanceSettings *settings,const PlanetModel *planet);
void entry_publish_taem_tangent_target(GuidanceMachine *guidance,const Telemetry *telemetry,double course,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg);
bool entry_taem_shaping_crossrange_ready(const TaemInterfaceTarget *target,const Telemetry *telemetry,const PlanetModel *planet,const LandingConfiguration *cfg,double next_sign,double *built_cross,double *required_cross);
EntryPrediction predictor_simulate_entry(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double max_bank,double initial_bank,double initial_bank_sign,double initial_leg_elapsed,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_with_rate(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double max_bank,double initial_bank,double initial_bank_rate,double initial_bank_sign,double initial_leg_elapsed,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_with_attitude(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double max_bank,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double initial_bank_sign,double initial_leg_elapsed,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_control_plan(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,double current_leg_elapsed,const EntryControlPlan *plan,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_control_plan_to_interface(VehicleState state,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const VehicleProfile *vehicle,const LandingSite *site,const GuidanceSettings *settings,const TaemInterfaceTarget *interface_target,bool enforce_terminal_delivery_budget,double initial_bank,double initial_bank_rate,double initial_angle_of_attack,double initial_angle_of_attack_rate,double current_bank_sign,double current_leg_elapsed,const EntryControlPlan *plan,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_entry_guidance_shadow(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingConfiguration *cfg,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_terminal_shadow(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingConfiguration *cfg,double max_duration,bool include_trajectory);
EntryPrediction predictor_simulate_terminal_shadow_ensemble(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingConfiguration *cfg,double max_duration,bool include_trajectory);
void entry_prediction_clear(EntryPrediction *p);
bool reentry_guidance_shadow_recovery_qualified(const EntryPrediction *prediction,const VehicleProfile *vehicle,const GuidanceSettings *settings);
bool deorbit_plan_create(DeorbitPlan *out,VehicleState state,double orbit_period,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double available_thrust);
bool deorbit_plan_create_with_telemetry(DeorbitPlan *out,VehicleState state,double orbit_period,const PlanetModel *planet,AerodynamicModel aero,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double available_thrust,const Telemetry *authority_source);
bool deorbit_plan_preburn_state_compatible(const DeorbitPlan *plan,double live_mass,double live_available_thrust,const GuidanceSettings *settings);
bool deorbit_capture_qualified(const EntryPrediction *p,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double postburn_periapsis,bool require_entry_corridor);
bool deorbit_recovery_qualified(const EntryPrediction *p,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double postburn_periapsis);
bool deorbit_live_cutoff_capture_qualified(const EntryPrediction *p,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double postburn_periapsis);
bool deorbit_runway_capture_qualified(const EntryPrediction *p,const LandingSite *site,const VehicleProfile *vehicle,const GuidanceSettings *settings,double postburn_periapsis);

bool guidance_terminal_preview_allowed(const GuidanceMachine *g);
bool guidance_plan_terminal_preview(GuidanceMachine *g,const Telemetry *t,const PlanetModel *p,
    AerodynamicModel aero,const LandingConfiguration *cfg);
bool guidance_accept_terminal_preview(GuidanceMachine *g,const GuidanceMachine *request,
    const GuidanceMachine *result,const Telemetry *t,const LandingConfiguration *cfg);
bool guidance_accept_entry_plan(GuidanceMachine *g,const GuidanceMachine *request,
    const GuidanceMachine *result,const Telemetry *t,const LandingConfiguration *cfg);
void guidance_update_entry_reversal(GuidanceMachine *machine,const EntryControlPlan *plan,double ut);
void guidance_machine_init(GuidanceMachine *g); void guidance_set_engaged(GuidanceMachine *g,bool engaged); void guidance_set_paused(GuidanceMachine *g,bool paused); void guidance_abort(GuidanceMachine *g); void guidance_reset_plan(GuidanceMachine *g); double guidance_entry_leg_elapsed(const GuidanceMachine *g,double ut);
void guidance_set_entry_predictor_models(GuidanceMachine *g,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal);
void guidance_initialize_reentry_continuation(GuidanceMachine *g,const Telemetry *t,const PlanetModel *planet,const LandingConfiguration *cfg,double initial_s_turn_sign,bool late_terminal_test,const AerodynamicEnvelope *env,const TrajectoryCalibrationModel *cal);
bool guidance_begin_hac_test(GuidanceMachine *g,const Telemetry *t,double course,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg,char *message,size_t message_size);
bool guidance_begin_final_test(GuidanceMachine *g,const Telemetry *t,double course,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg,char *message,size_t message_size);
GuidanceResult guidance_update(GuidanceMachine *g,const Telemetry *t,const VehicleState *state,const DeorbitPlan *plan,const PlanetModel *planet,AerodynamicModel aero,const LandingConfiguration *cfg);
GuidanceResult guidance_update_with_terminal_model(GuidanceMachine *g,
    const Telemetry *t,const VehicleState *state,const DeorbitPlan *plan,
    const PlanetModel *planet,AerodynamicModel aero,
    const LandingConfiguration *cfg,const TerminalModel *terminal_model);
EntryControlPlan guidance_terminal_control_plan(const GuidanceMachine *g,EntryControlPlan plan);
void guidance_result_clear(GuidanceResult *r); void reference_trajectory(Trajectory *out,const LandingSite *site,const GuidanceSettings *settings,double radius,double hac_side);

/* calibration */
void adaptive_calibrator_init(AdaptiveFlightCalibrator *c,const VehicleProfile *profile); void adaptive_calibrator_reset(AdaptiveFlightCalibrator *c,const VehicleProfile *profile);
void adaptive_calibrator_update(AdaptiveFlightCalibrator *c,const Telemetry *t,const CalibrationSettings *settings,const VehicleProfile *baseline,const GuidanceCommand *command,double dt,bool allow_adaptation,bool active,bool sampling,CalibrationRunState run_state,double progress,double target_aoa,const char *run_status,const char *warning,AerodynamicModel *current,AerodynamicModel *planning,AerodynamicEnvelope *envelope,VehicleProfile *recommended,CalibrationSnapshot *snapshot);
void glide_calibration_init(GlideCalibrationMachine *g); void glide_calibration_start(GlideCalibrationMachine *g,const Telemetry *t,const CalibrationSettings *settings); void glide_calibration_stop(GlideCalibrationMachine *g,const char *status); GuidanceResult glide_calibration_update(GlideCalibrationMachine *g,const Telemetry *t,const VehicleProfile *profile,const CalibrationSettings *settings,bool *sampling,bool *finished,double *progress,double *target_aoa);
void trajectory_calibrator_init(InFlightTrajectoryCalibrator *c); void trajectory_calibrator_clear(InFlightTrajectoryCalibrator *c); void trajectory_calibrator_set_forecast(InFlightTrajectoryCalibrator *c,const Trajectory *forecast); TrajectoryCalibrationModel trajectory_calibrator_update(InFlightTrajectoryCalibrator *c,const Telemetry *t,const VehicleState *state,const PlanetModel *planet,const LandingSite *site,const AerodynamicEnvelope *env,const VehicleProfile *vehicle,const GuidanceCommand *command,const CalibrationSettings *settings,bool allow_adaptation,double dt);

typedef struct {
    unsigned read_calls, read_wire_requests, apply_calls, apply_wire_requests;
    unsigned total_calls, total_wire_requests;
    double read_seconds, apply_seconds;
} KRPCTransportBudget;

/* native kRPC C-Nano runtime */
KRPCSession *krpc_session_open(const LandingConfiguration *cfg,char *error,size_t error_size); void krpc_session_close(KRPCSession *s); const PlanetModel *krpc_session_planet(const KRPCSession *s); const char *krpc_session_vessel(const KRPCSession *s);
const char *krpc_session_transport(const KRPCSession *s); const char *krpc_session_library_version(const KRPCSession *s); KRPCTransportBudget krpc_session_transport_budget(const KRPCSession *s); bool krpc_session_is_simulator(const KRPCSession *s);
const char *krpc_session_physics_structure_id(const KRPCSession *s); const char *krpc_session_physics_environment_id(const KRPCSession *s); const char *krpc_session_physics_storage(const KRPCSession *s); unsigned krpc_session_physics_history_count(const KRPCSession *s);
void krpc_session_seed_physics(const KRPCSession *s,VesselPhysicsModel *model);
bool krpc_orbital_up_reference(const GuidanceCommand *command,const VehicleState *state,Vector3 *out);
bool krpc_read_telemetry(KRPCSession *s,const LandingConfiguration *cfg,Telemetry *t,VehicleState *state,char *error,size_t error_size); bool krpc_apply(KRPCSession *s,const GuidanceCommand *command,unsigned airbrake_group,const char *phase,const char *status,const char *warning,const Trajectory *hud_predicted_trajectory,const Trajectory *hud_reference_trajectory,KRPCApplyResult *result,char *error,size_t error_size); void krpc_safe(KRPCSession *s); bool krpc_set_gear(KRPCSession *s,bool value,char *error,size_t error_size); bool krpc_set_brakes(KRPCSession *s,bool value,char *error,size_t error_size); bool krpc_save_game(KRPCSession *s,const char *name,char *error,size_t error_size); bool krpc_warp(KRPCSession *s,double ut,char *error,size_t error_size);

/* controller */
typedef void (*SnapshotCallback)(const LandingSnapshot *snapshot, void *context);
LandingController *landing_controller_create(const LandingConfiguration *cfg,SnapshotCallback callback,void *context); void landing_controller_destroy(LandingController *c); void landing_controller_update_configuration(LandingController *c,const LandingConfiguration *cfg); LandingConfiguration landing_controller_configuration(LandingController *c);
void landing_controller_connect(LandingController *c); void landing_controller_disconnect(LandingController *c); void landing_controller_create_plan(LandingController *c); void landing_controller_engage(LandingController *c); void landing_controller_engage_reentry(LandingController *c); void landing_controller_engage_hac_test(LandingController *c); void landing_controller_engage_final_test(LandingController *c); void landing_controller_start_calibration(LandingController *c); void landing_controller_stop_calibration(LandingController *c,bool apply); void landing_controller_reset_calibration(LandingController *c); void landing_controller_set_paused(LandingController *c,bool paused); void landing_controller_abort(LandingController *c); void landing_controller_set_gear(LandingController *c,bool deployed); void landing_controller_set_brakes(LandingController *c,bool enabled); bool landing_controller_save_checkpoint(LandingController *c,const char *name,char *error,size_t error_size); void landing_controller_shutdown(LandingController *c);

/* Coupled one-reversal Entry topology. Search only on the prediction worker. */
EntryTopologyPlan predictor_plan_entry_topology(VehicleState state,const Telemetry *telemetry,const GuidanceMachine *guidance,
    const PlanetModel *planet,const AerodynamicEnvelope *envelope,
    const TrajectoryCalibrationModel *calibration,const LandingConfiguration *configuration);
bool entry_topology_search_needed(const GuidanceMachine *guidance);
bool guidance_install_entry_topology(GuidanceMachine *guidance,const EntryTopologyPlan *plan,double ut);
GuidanceCommand guidance_entry_reference_step(GuidanceMachine *guidance,const Telemetry *telemetry,
    const VehicleProfile *vehicle,const GuidanceSettings *settings,double bank,double aoa,double dt);

#endif
