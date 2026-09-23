#ifndef KSP_LANDER_PLANNER_LOG_H
#define KSP_LANDER_PLANNER_LOG_H

#include "landing.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PLANNER_LOG_SCALAR_INTERVAL_SECONDS 2.0
#define PLANNER_LOG_TRAJECTORY_INTERVAL_SECONDS 12.0
#define PLANNER_LOG_DEFAULT_MAX_SEGMENT_BYTES (64u*1024u*1024u)
#define PLANNER_LOG_MAX_TRAJECTORY_POINTS 160u

typedef enum {
    PLANNER_LOG_EVENT_NONE=0,
    PLANNER_LOG_EVENT_PHASE=1u<<0,
    PLANNER_LOG_EVENT_PLAN=1u<<1,
    PLANNER_LOG_EVENT_CONSTRAINT=1u<<2,
    PLANNER_LOG_EVENT_PREDICTOR_DISCONTINUITY=1u<<3,
    PLANNER_LOG_EVENT_EXPLICIT=1u<<4,
    PLANNER_LOG_EVENT_TIME_REWIND=1u<<5
} PlannerLogEventFlags;

typedef struct {
    bool initialized, last_plan_valid, last_constraint_active;
    GuidancePhase last_phase;
    EntryControlPlan last_plan;
    double last_scalar_ut, last_trajectory_ut, last_ut;
    uint64_t next_plan_id, plan_id, plan_version, parent_plan_id, parent_plan_version;
} PlannerLogPolicyState;

typedef struct {
    bool emit_scalar, emit_trajectory, forced, durable;
    bool phase_changed, plan_changed, constraint_changed, predictor_discontinuity, time_rewind, executable_lineage;
    unsigned event_flags;
    uint64_t plan_id, plan_version, parent_plan_id, parent_plan_version;
} PlannerLogDecision;

void planner_log_policy_init(PlannerLogPolicyState *state);
PlannerLogDecision planner_log_policy_step(PlannerLogPolicyState *state,double ut,GuidancePhase phase,
    const EntryControlPlan *plan,bool constraint_active,bool predictor_discontinuity,bool explicit_force);

typedef struct PlannerLog PlannerLog;
typedef struct {
    uint64_t tick_sequence;
    double ut;
    GuidancePhase phase;
    const Telemetry *telemetry;
    const LandingSnapshot *snapshot;
    const EntryControlPlan *plan;
    const Trajectory *raw_prediction;
    const Trajectory *published_prediction;
    const TrajectoryCalibrationModel *calibration;
    const AerodynamicEnvelope *envelope;
    bool constraint_active;
    bool predictor_discontinuity;
    bool explicit_force;
    const char *event_reason;
} PlannerLogSample;

PlannerLog *planner_log_open(const char *directory,const char *stamp,const char *vessel,const char *session_id,
    const char *native_build,const char *vehicle_peer_path,bool vehicle_peer_available,
    const LandingConfiguration *configuration,size_t max_segment_bytes);
void planner_log_close(PlannerLog *log);
PlannerLogDecision planner_log_record(PlannerLog *log,const PlannerLogSample *sample);
unsigned planner_log_segment_index(const PlannerLog *log);
size_t planner_log_segment_bytes(const PlannerLog *log);
const char *planner_log_current_path(const PlannerLog *log);

#endif
