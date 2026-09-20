#include "async_prediction_policy.h"

#include <math.h>

static double prediction_interval(double configured_interval){
    return isfinite(configured_interval)?fmax(configured_interval,2.0):2.0;
}

double async_prediction_cooldown_seconds(double configured_interval,double previous_solve_wall_seconds){
    double interval=prediction_interval(configured_interval);
    double solve=isfinite(previous_solve_wall_seconds)?fmax(0.0,previous_solve_wall_seconds):0.0;
    return fmax(interval,fmin(solve,8.0));
}

bool async_prediction_request_due(double latest_ut,double last_request_ut,double now_wall,double last_completion_wall,double previous_solve_wall_seconds,double configured_interval){
    if(!isfinite(latest_ut)||!isfinite(now_wall))return false;
    double interval=prediction_interval(configured_interval);
    double since_request=isfinite(last_request_ut)?latest_ut-last_request_ut:INFINITY;
    double since_completion=isfinite(last_completion_wall)?now_wall-last_completion_wall:INFINITY;
    return since_request+1e-9>=interval&&
        since_completion+1e-9>=async_prediction_cooldown_seconds(configured_interval,previous_solve_wall_seconds);
}

static bool same_optional_time(bool valid_a,double a,bool valid_b,double b){
    if(valid_a!=valid_b)return false;
    if(!valid_a)return true;
    if(!isfinite(a)||!isfinite(b))return false;
    return fabs(a-b)<=1e-6;
}

bool async_prediction_identity_equal(const AsyncPredictionIdentity *a,const AsyncPredictionIdentity *b){
    if(!a||!b)return false;
    return a->guidance_phase==b->guidance_phase&&
        a->taem_phase==b->taem_phase&&
        a->taem_transition_count==b->taem_transition_count&&
        a->control_plan_sequence==b->control_plan_sequence&&
        a->plan_id==b->plan_id&&a->plan_version==b->plan_version&&
        a->terminal_path_kind==b->terminal_path_kind&&
        a->terminal_path_committed==b->terminal_path_committed&&
        a->hac_side_selected==b->hac_side_selected&&
        a->hac_captured==b->hac_captured&&
        a->final_approach_captured==b->final_approach_captured&&
        same_optional_time(a->terminal_candidate_valid,a->terminal_candidate_selected_ut,
            b->terminal_candidate_valid,b->terminal_candidate_selected_ut);
}

bool async_prediction_result_fresh(const AsyncPredictionIdentity *request_identity,const AsyncPredictionIdentity *current_identity,double request_ut,double latest_ut,double configured_interval){
    if(!async_prediction_identity_equal(request_identity,current_identity)||
       !isfinite(request_ut)||!isfinite(latest_ut)||latest_ut+1e-6<request_ut)return false;
    double maximum_age=fmax(8.0,prediction_interval(configured_interval)*4.0);
    return latest_ut-request_ut<=maximum_age+1e-6;
}
