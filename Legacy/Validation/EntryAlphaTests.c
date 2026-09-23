#include "entry_alpha.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int near(double a,double b,double eps){return fabs(a-b)<=eps;}

static EntryAlphaSchedule test_schedule(void){
    EntryAlphaSchedule s;memset(&s,0,sizeof(s));
    s.point_count=3;
    s.relative_velocity[0]=1000;s.relative_velocity[1]=1500;s.relative_velocity[2]=2000;
    s.alpha[0]=10;s.alpha[1]=20;s.alpha[2]=30;
    for(int i=0;i<ENTRY_ALPHA_PHASE_COUNT;i++)s.phase_modulation_scale[i]=1.0;
    s.drag_error_gain=6.0;s.maximum_modulation=3.0;s.modulation_rate_limit=100.0;s.target_rate_limit=100.0;
    s.minimum_aero_confidence=.2;s.dynamic_pressure_protect_ratio=.92;s.g_load_protect_ratio=.92;
    s.minimum_protective_aoa=0.0;
    s.stall_fraction_limit=.12;s.calibrated_stall_speed_margin=1.15;
    return s;
}

static EntryAlphaInput test_input(void){
    EntryAlphaInput i;memset(&i,0,sizeof(i));
    i.relative_velocity=1500;i.phase=ENTRY_ALPHA_CONSTANT_DRAG;
    i.reference_drag_accel=10;i.measured_drag_accel=10;
    i.current_aoa=20;i.current_aoa_rate=0;i.dt=1;
    i.maximum_aoa=40;i.stall_margin_aoa_limit=NAN;i.stall_fraction=0;
    i.calibrated_stall_speed=NAN;i.minimum_safe_speed=80;
    i.dynamic_pressure=1000;i.maximum_dynamic_pressure=45000;
    i.g_load=1;i.maximum_g_load=3.5;i.aero_confidence=1;
    return i;
}

static void test_interpolation(void){
    EntryAlphaSchedule s=test_schedule();
    assert(entry_alpha_schedule_valid(&s));
    assert(near(entry_alpha_schedule_interpolate(&s,500),10,1e-9));
    assert(near(entry_alpha_schedule_interpolate(&s,1000),10,1e-9));
    assert(near(entry_alpha_schedule_interpolate(&s,1250),15,1e-9));
    assert(near(entry_alpha_schedule_interpolate(&s,1750),25,1e-9));
    assert(near(entry_alpha_schedule_interpolate(&s,2500),30,1e-9));
    double left=entry_alpha_schedule_interpolate(&s,1499.999);
    double right=entry_alpha_schedule_interpolate(&s,1500.001);
    assert(fabs(left-right)<1e-3);
}

static void test_drag_modulation(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.measured_drag_accel=5;
    EntryAlphaResult positive=entry_alpha_command(&s,&i);
    assert(positive.valid);assert(positive.modulation>0);assert(positive.target_aoa>positive.nominal_aoa);
    i.measured_drag_accel=15;
    EntryAlphaResult negative=entry_alpha_command(&s,&i);
    assert(negative.valid);assert(negative.modulation<0);assert(negative.target_aoa<negative.nominal_aoa);
}

static void test_modulation_clamp(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.reference_drag_accel=10;i.measured_drag_accel=-100;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(near(r.requested_modulation,s.maximum_modulation,1e-9));
    assert(r.target_aoa<=r.nominal_aoa+s.maximum_modulation+1e-9);
}

static void test_max_aoa_limit(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.relative_velocity=2000;i.current_aoa=25;i.maximum_aoa=25;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(near(r.target_aoa,25,1e-9));
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_MAX_AOA);
}

static void test_stall_limit(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.stall_fraction=.2;i.stall_fraction_is_measured=true;i.stall_margin_aoa_limit=12;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(r.target_aoa<=12+1e-9);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN);
}

static void test_unmeasured_low_q_stall_proxy_does_not_authorize_emergency_unload(void){
    VehicleProfile v={0};
    v.entry_angle_of_attack=18;v.maximum_angle_of_attack=28;v.minimum_safe_speed=85;v.final_approach_speed=115;
    EntryAlphaSchedule s;entry_alpha_schedule_default(&s,&v,1300);
    EntryAlphaInput i=test_input();
    i.relative_velocity=2000;i.current_aoa=25;i.maximum_aoa=28;i.minimum_safe_speed=85;
    i.dynamic_pressure=117;i.maximum_dynamic_pressure=45000;i.stall_fraction=.57;
    i.stall_fraction_is_measured=false;i.stall_margin_aoa_limit=NAN;
    EntryAlphaResult proxy=entry_alpha_command(&s,&i);
    assert(proxy.valid);
    assert(proxy.limiting_reason!=ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    assert(proxy.target_aoa>=s.minimum_protective_aoa-1e-9);

    i.stall_fraction_is_measured=true;
    EntryAlphaResult measured=entry_alpha_command(&s,&i);
    assert(measured.valid);
    assert(measured.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    assert(measured.target_aoa<proxy.target_aoa);

    /* Live STS-N, UT 67416.34: 1967.7 m/s, q=1354.5 Pa, alpha=21.81
       and an UNMEASURED 0.123 proxy triggered the 11.2-degree emergency cap.
       Releasing thin-air protection does not turn this AoA-derived estimate
       into independent evidence of a stall during a high-incidence entry. */
    const double pressures[]={700.0,1354.5,4389.0,15000.0};
    i.phase=ENTRY_ALPHA_TEMPERATURE_CONTROL;i.relative_velocity=1967.7;
    i.current_aoa=21.81;i.current_aoa_rate=.1;i.stall_fraction=.123;
    i.has_previous_target=true;i.previous_target_aoa=22.41;i.dt=.1;
    for(size_t n=0;n<sizeof(pressures)/sizeof(pressures[0]);n++){
        i.dynamic_pressure=pressures[n];i.stall_fraction_is_measured=false;
        EntryAlphaResult loaded_proxy=entry_alpha_command(&s,&i);
        assert(loaded_proxy.valid);
        assert(loaded_proxy.limiting_reason!=ENTRY_ALPHA_LIMIT_STALL_MARGIN);
        assert(loaded_proxy.target_aoa>=s.minimum_protective_aoa);
        i.stall_fraction_is_measured=true;
        EntryAlphaResult loaded_measured=entry_alpha_command(&s,&i);
        assert(loaded_measured.valid);
        assert(loaded_measured.limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN);
        assert(loaded_measured.target_aoa<loaded_proxy.target_aoa);
    }
    /* Actual airspeed margins still authorize recovery without a stall sensor. */
    i.stall_fraction_is_measured=false;i.stall_fraction=0;
    i.relative_velocity=90;
    assert(entry_alpha_command(&s,&i).limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    i.relative_velocity=110;i.calibrated_stall_speed=105;
    assert(entry_alpha_command(&s,&i).limiting_reason==ENTRY_ALPHA_LIMIT_STALL_MARGIN);
}

static void test_dynamic_pressure_protection(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.dynamic_pressure=i.maximum_dynamic_pressure;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(r.target_aoa>r.nominal_aoa);
    assert(r.target_aoa<=r.nominal_aoa+s.maximum_modulation+1e-9);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE);
}

static void test_g_load_protection(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.g_load=i.maximum_g_load;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(r.target_aoa<r.nominal_aoa);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_G_LOAD);
}

static void test_degraded_model_fallback(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    i.measured_drag_accel=5;i.aero_confidence=.05;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(r.degraded);assert(near(r.requested_modulation,0,1e-9));
    assert(near(r.modulation,0,1e-9));assert(near(r.target_aoa,r.nominal_aoa,1e-9));
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE);
}

static void test_target_rate_continuity(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    s.target_rate_limit=2.0;i.current_aoa=10;i.has_previous_target=true;i.previous_target_aoa=10;
    i.relative_velocity=2000;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);assert(r.target_aoa<=12+1e-9);assert(r.target_aoa>=8-1e-9);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_COMMAND_RATE);
}

static void test_recorded_late_entry_overdrag_preserves_lift_trim(void){
    VehicleProfile v={0};
    v.entry_angle_of_attack=18;v.maximum_angle_of_attack=28;v.minimum_safe_speed=85;v.final_approach_speed=115;
    EntryAlphaSchedule s;entry_alpha_schedule_default(&s,&v,1300);
    EntryAlphaInput i=test_input();
    /* Representative 2026-09-10T02-08-11Z state near 1490 m/s: the old law
       saw about 2.84 m/s2 reference drag versus 6.39 m/s2 measured drag and
       drove AoA roughly four degrees below the lift-bearing schedule. */
    i.relative_velocity=1490;i.phase=ENTRY_ALPHA_CONSTANT_DRAG;
    i.reference_drag_accel=2.84;i.measured_drag_accel=6.39;
    i.current_aoa=19.0;i.current_aoa_rate=0;i.dt=1.0;
    i.maximum_aoa=28;i.minimum_safe_speed=85;i.dynamic_pressure=5354;
    i.maximum_dynamic_pressure=45000;i.g_load=.75;i.maximum_g_load=3.5;i.aero_confidence=1;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);
    assert(r.requested_modulation<0.0);
    assert(r.requested_modulation>=-0.85);
    assert(r.target_aoa>=r.nominal_aoa-1.0);
}

static void test_overdrag_limit_does_not_weaken_g_protection(void){
    VehicleProfile v={0};
    v.entry_angle_of_attack=18;v.maximum_angle_of_attack=28;v.minimum_safe_speed=85;v.final_approach_speed=115;
    EntryAlphaSchedule s;entry_alpha_schedule_default(&s,&v,1300);
    EntryAlphaInput i=test_input();
    i.relative_velocity=1490;i.phase=ENTRY_ALPHA_CONSTANT_DRAG;
    i.reference_drag_accel=2.84;i.measured_drag_accel=6.39;
    i.current_aoa=entry_alpha_schedule_interpolate(&s,i.relative_velocity);
    i.current_aoa_rate=0;i.dt=1.0;i.maximum_aoa=28;i.minimum_safe_speed=85;
    i.dynamic_pressure=5354;i.maximum_dynamic_pressure=45000;
    i.g_load=3.5;i.maximum_g_load=3.5;i.aero_confidence=1;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_G_LOAD);
    assert(r.target_aoa<=r.nominal_aoa-1.5);
}

static void test_default_thermal_floor_blocks_non_emergency_unload(void){
    VehicleProfile v={0};
    v.entry_angle_of_attack=18;v.maximum_angle_of_attack=28;v.minimum_safe_speed=85;v.final_approach_speed=115;
    EntryAlphaSchedule s;entry_alpha_schedule_default(&s,&v,1300);
    EntryAlphaInput i=test_input();
    i.relative_velocity=1300;i.phase=ENTRY_ALPHA_CONSTANT_DRAG;
    i.reference_drag_accel=10;i.measured_drag_accel=30;
    i.current_aoa=18;i.current_aoa_rate=0;i.dt=1;
    i.maximum_aoa=28;i.minimum_safe_speed=85;i.dynamic_pressure=1000;
    i.maximum_dynamic_pressure=45000;i.g_load=1;i.maximum_g_load=3.5;i.aero_confidence=1;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);
    assert(r.requested_modulation<0.0);
    assert(r.target_aoa>=v.entry_angle_of_attack-1e-9);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION);
}

static void test_default_schedule(void){
    VehicleProfile v={0};
    v.entry_angle_of_attack=18;v.maximum_angle_of_attack=28;v.minimum_safe_speed=85;v.final_approach_speed=115;
    EntryAlphaSchedule s;entry_alpha_schedule_default(&s,&v,1300);
    assert(entry_alpha_schedule_valid(&s));
    assert(s.point_count==5);
    assert(near(s.minimum_protective_aoa,v.entry_angle_of_attack,1e-9));
    for(size_t n=0;n<s.point_count;n++){
        assert(s.alpha[n]>=s.minimum_protective_aoa-1e-9);
        assert(s.alpha[n]<=v.maximum_angle_of_attack);
    }
    assert(s.alpha[s.point_count-1]>s.alpha[1]);
    assert(near(entry_alpha_schedule_interpolate(&s,1300),18,1e-9));
}

static void test_thermal_floor_is_not_clipped_by_modulation_band(void){
    EntryAlphaSchedule s=test_schedule();EntryAlphaInput i=test_input();
    s.minimum_protective_aoa=26;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);
    assert(r.target_aoa>=26);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION);
    /* Structural unloading still takes precedence over thermal incidence. */
    i.g_load=i.maximum_g_load;
    r=entry_alpha_command(&s,&i);
    assert(r.target_aoa<r.nominal_aoa);
    assert(r.limiting_reason==ENTRY_ALPHA_LIMIT_G_LOAD);
}

static void test_minimum_drag_tracking_aoa_matches_steady_production_command(void){
    VehicleProfile v={0};
    v.entry_angle_of_attack=18;v.maximum_angle_of_attack=28;v.minimum_safe_speed=85;v.final_approach_speed=115;
    EntryAlphaSchedule s;entry_alpha_schedule_default(&s,&v,1247.2);
    double q=2311.0,max_q=45000.0,confidence=.95,velocity=1840.6;
    double floor=entry_alpha_minimum_drag_tracking_aoa(&s,ENTRY_ALPHA_EQUILIBRIUM_GLIDE,
        velocity,q,max_q,confidence);
    assert(isfinite(floor));
    assert(floor>21.5&&floor<22.8);

    EntryAlphaInput i=test_input();
    i.relative_velocity=velocity;i.phase=ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
    i.reference_drag_accel=4.0;i.measured_drag_accel=100.0;
    i.current_aoa=entry_alpha_schedule_interpolate(&s,velocity);i.current_aoa_rate=0;
    i.dt=100.0;i.maximum_aoa=28;i.minimum_safe_speed=85;
    i.dynamic_pressure=q;i.maximum_dynamic_pressure=max_q;i.g_load=1;i.maximum_g_load=3.5;
    i.aero_confidence=confidence;i.stall_fraction=0;i.stall_fraction_is_measured=false;
    i.stall_margin_aoa_limit=NAN;i.calibrated_stall_speed=NAN;
    EntryAlphaResult r=entry_alpha_command(&s,&i);
    assert(r.valid);
    assert(near(floor,r.target_aoa,1e-9));

    double high_q=entry_alpha_minimum_drag_tracking_aoa(&s,ENTRY_ALPHA_EQUILIBRIUM_GLIDE,
        velocity,max_q,max_q,confidence);
    assert(high_q>floor);
}

int main(void){
    test_thermal_floor_is_not_clipped_by_modulation_band();
    test_interpolation();
    test_drag_modulation();
    test_modulation_clamp();
    test_max_aoa_limit();
    test_stall_limit();
    test_unmeasured_low_q_stall_proxy_does_not_authorize_emergency_unload();
    test_dynamic_pressure_protection();
    test_g_load_protection();
    test_degraded_model_fallback();
    test_target_rate_continuity();
    test_recorded_late_entry_overdrag_preserves_lift_trim();
    test_overdrag_limit_does_not_weaken_g_protection();
    test_default_thermal_floor_blocks_non_emergency_unload();
    test_default_schedule();
    test_minimum_drag_tracking_aoa_matches_steady_production_command();
    puts("Entry alpha tests passed.");
    return 0;
}
