#include "landing.h"
#include "flight_log_codec.h"
#include "planner_log.h"
#include "async_prediction_policy.h"
#include "decision_envelope.h"
#include "terminal_model.h"
#include "taem_planner.h"
#include "sim_telemetry.h"
#include "mm305_planning.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

struct LandingController {
    pthread_mutex_t mutex;
    pthread_t thread;
    bool thread_started;
    bool stop_thread;
    pthread_t prediction_thread;
    bool prediction_thread_started;
    bool stop_prediction_thread;
    uint64_t prediction_generation;
    double last_terminal_prediction_request_ut, last_terminal_prediction_completion_ut;
    double last_terminal_prediction_completion_wall, last_terminal_prediction_solve_wall;
    LandingConfiguration configuration;
    KRPCSession *session;
    GuidanceMachine guidance;
    TerminalModel terminal_model;
    bool terminal_model_valid;
    uint64_t terminal_model_capture_sequence;
    VesselPhysicsModel physics;
    AdaptiveFlightCalibrator adaptive;
    GlideCalibrationMachine glide;
    AerodynamicModel aerodynamics;
    AerodynamicModel planning_aerodynamics;
    AerodynamicEnvelope envelope;
    InFlightTrajectoryCalibrator trajectory_calibrator;
    TrajectoryCalibrationModel trajectory_calibration;
    GuidanceCommand last_command;
    VehicleProfile adaptive_profile;
    DeorbitPlan plan;
    bool has_plan;
    bool plan_stale;
    LandingSnapshot snapshot;
    Trajectory dynamic_trajectory;
    Trajectory stabilized_trajectory;
    bool forecast_entry_plan_identity_valid;
    uint64_t forecast_entry_plan_id, forecast_entry_plan_version;
    bool prediction_lineage_discontinuity;
    Telemetry latest_telemetry;
    VehicleState latest_state;
    bool has_latest;
    double last_trajectory_ut;
    double last_prediction_ut;
    bool has_prediction_cache;
    double cached_physics_observed_seconds, cached_physics_fallback_seconds;
    double cached_predicted_miss_distance;
    double cached_predicted_taem_distance;
    double cached_predicted_taem_range_error;
    double cached_predicted_entry_range;
    double cached_predicted_entry_flight_path_angle;
    double cached_predicted_taem_speed;
    double cached_predicted_taem_energy_error;
    double cached_predicted_peak_dynamic_pressure;
    double cached_predicted_peak_g_load;
    double cached_predicted_s_turn_reversals;
    bool cached_predicted_shadow_guidance, cached_predicted_terminal_policy_feasible;
    double cached_predicted_terminal_survivability_stress_score, cached_predicted_physics_relative_uncertainty;
    unsigned cached_predicted_uncertainty_scenarios;
    double cached_predicted_raw_published_position_delta_30s;
    double cached_predicted_raw_published_position_delta_60s;
    double cached_predicted_raw_published_position_delta_120s;
    double cached_prediction_ut;
    double cached_prediction_sign;
    double last_tick_ut;
    bool has_last_tick_ut;
    double last_tick_wall;
    bool has_last_tick_wall;
    bool time_warp_issued;
    uint64_t tick_sequence, log_sequence, vehicle_record_sequence;
    SnapshotCallback callback;
    void *callback_context;
    bool sim_publication_valid, publication_event_pending;
    double sim_last_publish_wall, sim_last_publish_ut;
    GuidancePhase sim_last_publish_phase;
    ConnectionStatus sim_last_publish_connection;
    bool sim_last_publish_engaged, sim_last_publish_paused;
    FILE *flight_log;
    PlannerLog *planner_log;
    VehicleLogEncoder vehicle_log_encoder;
    char log_session_id[192];
    char log_native_build[96], log_planner_path[1700];
    bool log_actual_trajectory_dirty;
    bool log_reference_trajectory_dirty;
    bool log_plan_trajectory_dirty;
    bool log_raw_plan_trajectory_dirty;
};

static void start_prediction_worker(LandingController *c);
static void stop_prediction_worker(LandingController *c);

static double monotonic_seconds(void){struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);return (double)ts.tv_sec+(double)ts.tv_nsec/1e9;}
static void sleep_seconds(double s){if(s<=0)return;struct timespec ts={(time_t)s,(long)((s-floor(s))*1e9)};while(nanosleep(&ts,&ts)<0&&errno==EINTR){}}


/* Controller state and thread primitives live here; implementation is grouped
 * by application responsibility while remaining one translation unit. */
#include "controller/configuration.inc"
#include "controller/logging.inc"
#include "controller/prediction_workers.inc"
#include "controller/runtime.inc"
#include "controller/api.inc"
