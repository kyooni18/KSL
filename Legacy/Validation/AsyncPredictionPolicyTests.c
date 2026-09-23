#include "async_prediction_policy.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static AsyncPredictionIdentity identity(void){
    AsyncPredictionIdentity i={0};
    i.guidance_phase=6;
    i.taem_phase=1;
    i.taem_transition_count=3;
    i.control_plan_sequence=9;
    i.plan_id=12;
    i.plan_version=4;
    i.terminal_candidate_valid=true;
    i.terminal_candidate_selected_ut=102.5;
    i.terminal_path_kind=2;
    i.terminal_path_committed=false;
    i.hac_side_selected=true;
    return i;
}

static void test_slow_solve_cannot_trigger_immediate_catchup(void){
    assert(fabs(async_prediction_cooldown_seconds(1.5,6.5)-6.5)<1e-9);
    assert(!async_prediction_request_due(108.0,100.0,206.6,206.5,6.5,1.5));
    assert(!async_prediction_request_due(108.0,100.0,212.9,206.5,6.5,1.5));
    assert(async_prediction_request_due(108.0,100.0,213.0,206.5,6.5,1.5));
    assert(!async_prediction_request_due(101.0,100.0,220.0,206.5,6.5,1.5));
    assert(async_prediction_request_due(102.0,100.0,220.0,206.5,0.0,1.5));
}

static void test_async_result_requires_current_lineage_and_bounded_age(void){
    AsyncPredictionIdentity request=identity(),current=request;
    assert(async_prediction_result_fresh(&request,&current,100.0,106.0,1.5));
    assert(!async_prediction_result_fresh(&request,&current,100.0,108.01,1.5));
    current=request;current.plan_version++;
    assert(!async_prediction_result_fresh(&request,&current,100.0,104.0,1.5));
    current=request;current.control_plan_sequence++;
    assert(!async_prediction_result_fresh(&request,&current,100.0,104.0,1.5));
    current=request;current.taem_phase++;
    assert(!async_prediction_result_fresh(&request,&current,100.0,104.0,1.5));
    current=request;current.terminal_candidate_selected_ut+=.5;
    assert(!async_prediction_result_fresh(&request,&current,100.0,104.0,1.5));
    current=request;current.terminal_path_committed=true;
    assert(!async_prediction_result_fresh(&request,&current,100.0,104.0,1.5));
    request.terminal_candidate_valid=false;current=request;
    request.terminal_candidate_selected_ut=NAN;current.terminal_candidate_selected_ut=1234.0;
    assert(async_prediction_identity_equal(&request,&current));
}

int main(void){
    test_slow_solve_cannot_trigger_immediate_catchup();
    test_async_result_requires_current_lineage_and_bounded_age();
    puts("Async prediction policy tests passed.");
    return 0;
}
