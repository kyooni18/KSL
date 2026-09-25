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

static EntryLateralInput azimuth_input(double error,double deadband){
    EntryLateralInput in=input();
    in.has_crossrange_error=false;
    in.has_bank_magnitude=true;
    in.bank_magnitude_deg=50.0;
    in.has_azimuth_error=true;
    in.azimuth_error_deg=error;
    in.azimuth_deadband_deg=deadband;
    in.minimum_turn_bank_deg=30.0;
    return in;
}

/* Azimuth logic is heading-independent: the first bank points toward the
   target whatever the flight direction or runway crossrange sign. */
static void azimuth_initial_side_points_at_target(void){
    EntryLateralLimits l=limits();
    EntryLateralState right={0},left={0};
    EntryLateralInput r=azimuth_input(12.0,5.0),q=azimuth_input(-12.0,5.0);
    r.measured_bank_deg=q.measured_bank_deg=0.0;
    r.has_crossrange_error=q.has_crossrange_error=true;
    r.crossrange_error_m=-5000.0; /* contradicts the azimuth on purpose */
    q.crossrange_error_m=5000.0;
    EntryLateralOutput ro=entry_lateral_update(&right,&r,&l);
    EntryLateralOutput qo=entry_lateral_update(&left,&q,&l);
    assert(ro.valid&&ro.bank_sign>0.0);
    assert(qo.valid&&qo.bank_sign<0.0);
}

/* Inside the deadband the leg holds; outside it on the wrong side a captured
   leg reverses toward the target. */
static void azimuth_deadband_reversal(void){
    EntryLateralLimits l=limits();
    EntryLateralState state={0};
    entry_lateral_state_init(&state,90.0,1.0,50.0);
    state.leg_captured=true;
    EntryLateralInput in=azimuth_input(-4.0,5.0);
    in.measured_bank_deg=50.0;
    EntryLateralOutput hold=entry_lateral_update(&state,&in,&l);
    assert(hold.valid&&!hold.reversal_requested&&hold.bank_sign>0.0);

    in=azimuth_input(-6.0,5.0);
    in.ut=100.5;
    in.measured_bank_deg=50.0;
    EntryLateralOutput rev=entry_lateral_update(&state,&in,&l);
    assert(rev.valid&&rev.reversal_requested&&rev.bank_sign<0.0);
}

/* A large azimuth error keeps a turning bank even when the energy law asks
   for lift-up. */
static void azimuth_turn_bank_floor(void){
    EntryLateralLimits l=limits();
    EntryLateralState state={0};
    EntryLateralInput in=azimuth_input(40.0,5.0);
    in.measured_bank_deg=0.0;
    in.bank_magnitude_deg=0.0;
    EntryLateralOutput out=entry_lateral_update(&state,&in,&l);
    assert(out.valid&&out.bank_sign>0.0);
    assert(out.bank_magnitude_deg>=30.0-1e-9);
}

/* Mirror property: flipping the target side (azimuth error, crossrange and
   measured bank) must flip the commanded bank exactly, from any state, for
   any flight direction.  This is the check that the old eastbound/crossrange
   assumption failed. */
static void mirror_symmetry(void){
    EntryLateralLimits l=limits();
    const double errors[]={-40.0,-12.0,-6.0,-2.0,2.0,6.0,12.0,40.0};
    const double banks[]={-50.0,-10.0,0.0,10.0,50.0};
    for(size_t i=0;i<sizeof(errors)/sizeof(errors[0]);++i)
        for(size_t j=0;j<sizeof(banks)/sizeof(banks[0]);++j){
            EntryLateralState a={0},b={0};
            EntryLateralInput ia=azimuth_input(errors[i],5.0),ib=azimuth_input(-errors[i],5.0);
            ia.measured_bank_deg=banks[j];ib.measured_bank_deg=-banks[j];
            ia.has_crossrange_error=ib.has_crossrange_error=true;
            ia.crossrange_error_m=3000.0*errors[i];ib.crossrange_error_m=-3000.0*errors[i];
            for(int k=0;k<50;++k){
                ia.ut=ib.ut=100.0+.1*k;
                EntryLateralOutput oa=entry_lateral_update(&a,&ia,&l);
                EntryLateralOutput ob=entry_lateral_update(&b,&ib,&l);
                assert(oa.valid&&ob.valid);
                assert(fabs(oa.target_bank_deg+ob.target_bank_deg)<1e-9);
                assert(oa.bank_sign==-ob.bank_sign);
                ia.measured_bank_deg=oa.target_bank_deg;ib.measured_bank_deg=ob.target_bank_deg;
            }
        }
}

int main(void){
    initial_side_follows_crossrange_sign();
    predicts_crossing_over_physical_roll_response();
    uncertainty_corridor_prevents_chatter();
    azimuth_initial_side_points_at_target();
    azimuth_deadband_reversal();
    azimuth_turn_bank_floor();
    mirror_symmetry();
    puts("Entry lateral feedback tests passed.");
    return 0;
}
