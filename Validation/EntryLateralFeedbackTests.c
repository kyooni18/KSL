#include "entry_lateral.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static EntryLateralLimits limits(void){
    EntryLateralLimits l=entry_lateral_default_limits();
    l.maximum_bank_deg=70.0;
    l.maximum_roll_rate_deg_s=10.0;
    l.maximum_roll_accel_deg_s2=5.0;
    return l;
}

static EntryLateralInput input(void){
    EntryLateralInput in;
    memset(&in,0,sizeof(in));
    in.ut=100.0;
    in.dt=0.1;
    in.relative_speed=1000.0;
    in.measured_bank_deg=60.0;
    in.measured_bank_rate_deg_s=0.0;
    in.bank_effectiveness=1.0;
    in.lift_accel=10.0;
    in.authority_confidence=1.0;
    in.has_crossrange_error=true;
    in.longitudinal.valid=true;
    in.longitudinal.confidence=1.0;
    in.longitudinal.required_vertical_lift_accel=5.0;
    return in;
}

static void initial_side_follows_crossrange_sign(void){
    EntryLateralLimits l=limits();
    EntryLateralState right={0},left={0};
    EntryLateralInput r=input(),q=input();
    r.measured_bank_deg=0.0;
    r.crossrange_error_m=1000.0;
    EntryLateralOutput ro=entry_lateral_update(&right,&r,&l);
    assert(ro.valid&&ro.bank_sign>0.0);

    q.measured_bank_deg=0.0;
    q.crossrange_error_m=-1000.0;
    EntryLateralOutput qo=entry_lateral_update(&left,&q,&l);
    assert(qo.valid&&qo.bank_sign<0.0);
}

static void predicts_crossing_over_physical_roll_response(void){
    EntryLateralLimits l=limits();
    EntryLateralState state={0};
    entry_lateral_state_init(&state,90.0,1.0,60.0);
    state.leg_captured=true;
    EntryLateralInput in=input();
    in.crossrange_error_m=100.0;
    in.has_crossrange_rate=true;
    in.crossrange_error_rate_mps=-100.0;
    in.crossrange_uncertainty_m=10.0;
    EntryLateralOutput out=entry_lateral_update(&state,&in,&l);
    assert(out.valid);
    assert(out.reversal_requested);
    assert(out.bank_sign<0.0);
    assert(out.reversal_response_time_s>0.0);
    assert(out.projected_crossrange_error_m< -out.crossrange_corridor_m);
}

static void uncertainty_corridor_prevents_chatter(void){
    EntryLateralLimits l=limits();
    EntryLateralState state={0};
    entry_lateral_state_init(&state,90.0,1.0,60.0);
    state.leg_captured=true;
    EntryLateralInput in=input();
    in.crossrange_error_m=-5.0;
    in.has_crossrange_rate=true;
    in.crossrange_error_rate_mps=0.0;
    in.crossrange_uncertainty_m=10.0;
    EntryLateralOutput out=entry_lateral_update(&state,&in,&l);
    assert(out.valid);
    assert(!out.reversal_requested);
    assert(out.bank_sign>0.0);
    assert(fabs(out.projected_crossrange_error_m)<=out.crossrange_corridor_m);
}

int main(void){
    initial_side_follows_crossrange_sign();
    predicts_crossing_over_physical_roll_response();
    uncertainty_corridor_prevents_chatter();
    puts("Entry lateral feedback tests passed.");
    return 0;
}
