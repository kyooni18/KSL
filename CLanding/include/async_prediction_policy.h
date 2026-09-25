#ifndef KSP_LANDER_ASYNC_PREDICTION_POLICY_H
#define KSP_LANDER_ASYNC_PREDICTION_POLICY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int guidance_phase;
    int taem_phase;
    unsigned taem_transition_count;
    uint64_t control_plan_sequence;
    uint64_t plan_id;
    uint64_t plan_version;
    int terminal_path_kind;
    bool terminal_path_committed;
    bool hac_side_selected;
    bool hac_captured;
    bool final_approach_captured;
} AsyncPredictionIdentity;

double async_prediction_cooldown_seconds(double configured_interval,double previous_solve_wall_seconds);
bool async_prediction_request_due(double latest_ut,double last_request_ut,double now_wall,double last_completion_wall,double previous_solve_wall_seconds,double configured_interval);
bool async_prediction_identity_equal(const AsyncPredictionIdentity *a,const AsyncPredictionIdentity *b);
bool async_prediction_result_fresh(const AsyncPredictionIdentity *request_identity,const AsyncPredictionIdentity *current_identity,double request_ut,double latest_ut,double configured_interval);

#endif
