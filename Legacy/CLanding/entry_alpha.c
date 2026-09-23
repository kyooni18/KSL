#include "entry_alpha.h"

#include <math.h>
#include <string.h>

static double smoothstep01(double x){
    x=clampd(x,0.0,1.0);
    return x*x*(3.0-2.0*x);
}

static int reason_priority(EntryAlphaLimitReason reason){
    switch(reason){
        case ENTRY_ALPHA_LIMIT_INVALID_INPUT:return 100;
        case ENTRY_ALPHA_LIMIT_STALL_MARGIN:return 90;
        case ENTRY_ALPHA_LIMIT_G_LOAD:return 80;
        case ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION:return 75;
        case ENTRY_ALPHA_LIMIT_MAX_AOA:return 70;
        case ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE:return 60;
        case ENTRY_ALPHA_LIMIT_COMMAND_RATE:return 50;
        case ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE:return 40;
        case ENTRY_ALPHA_LIMIT_NONE:return 0;
    }
    return 0;
}

static void set_reason(EntryAlphaResult *result, EntryAlphaLimitReason reason){
    if(result && reason_priority(reason)>reason_priority(result->limiting_reason))
        result->limiting_reason=reason;
}

/* Reducing AoA is an effective way to shed drag only while the vehicle still
   has vertical-lift margin. Late in Entry, measured drag can rise because the
   trajectory is already descending into denser air; aggressively chasing that
   drag error by lowering AoA removes lift, steepens the descent, and creates a
   self-reinforcing density dive. Keep the nominal Shuttle-derived schedule as
   the lift-bearing reference and make only the negative drag trim asymmetric.
   Explicit q/g/stall protections below retain full authority to move AoA when
   safety, rather than drag tracking, requires it. */
static double negative_drag_modulation_scale(EntryAlphaPhase phase){
    switch(phase){
        case ENTRY_ALPHA_PRE_ENTRY:return 1.0;
        case ENTRY_ALPHA_TEMPERATURE_CONTROL:return .85;
        case ENTRY_ALPHA_EQUILIBRIUM_GLIDE:return .50;
        case ENTRY_ALPHA_CONSTANT_DRAG:return .20;
        case ENTRY_ALPHA_TRANSITION:return .35;
        case ENTRY_ALPHA_PHASE_COUNT:return 1.0;
    }
    return 1.0;
}

void entry_alpha_schedule_default(EntryAlphaSchedule *schedule,
                                  const VehicleProfile *vehicle,
                                  double transition_velocity){
    if(!schedule)return;
    memset(schedule,0,sizeof(*schedule));
    if(!vehicle || !isfinite(transition_velocity) || transition_velocity<=0)return;

    const double trim=clampd(vehicle->entry_angle_of_attack,0.0,vehicle->maximum_angle_of_attack);
    const double maximum=fmax(trim,vehicle->maximum_angle_of_attack);
    const double span=fmax(0.0,maximum-trim);
    const double v=fmax(transition_velocity,vehicle->minimum_safe_speed*3.0);

    schedule->point_count=5;
    schedule->relative_velocity[0]=v*0.90;
    schedule->relative_velocity[1]=v;
    schedule->relative_velocity[2]=v*1.25;
    schedule->relative_velocity[3]=v*1.55;
    schedule->relative_velocity[4]=v*1.85;

    schedule->alpha[0]=trim;
    schedule->alpha[1]=trim;
    schedule->alpha[2]=clampd(trim+span*0.30,0.0,maximum);
    schedule->alpha[3]=clampd(trim+span*0.65,0.0,maximum);
    schedule->alpha[4]=clampd(trim+span*0.90,0.0,maximum);

    schedule->phase_modulation_scale[ENTRY_ALPHA_PRE_ENTRY]=0.0;
    schedule->phase_modulation_scale[ENTRY_ALPHA_TEMPERATURE_CONTROL]=0.35;
    schedule->phase_modulation_scale[ENTRY_ALPHA_EQUILIBRIUM_GLIDE]=0.65;
    schedule->phase_modulation_scale[ENTRY_ALPHA_CONSTANT_DRAG]=1.0;
    schedule->phase_modulation_scale[ENTRY_ALPHA_TRANSITION]=0.45;

    schedule->drag_error_gain=3.0;
    schedule->maximum_modulation=4.0;
    schedule->modulation_rate_limit=0.8;
    schedule->target_rate_limit=2.0;
    schedule->minimum_protective_aoa=trim;
    schedule->minimum_aero_confidence=0.20;
    schedule->dynamic_pressure_protect_ratio=0.92;
    schedule->g_load_protect_ratio=0.92;
    schedule->stall_fraction_limit=0.12;
    schedule->calibrated_stall_speed_margin=1.15;
}

bool entry_alpha_schedule_valid(const EntryAlphaSchedule *schedule){
    if(!schedule || schedule->point_count<2 ||
       schedule->point_count>ENTRY_ALPHA_MAX_SCHEDULE_POINTS)return false;
    for(size_t i=0;i<schedule->point_count;i++){
        if(!isfinite(schedule->relative_velocity[i]) ||
           !isfinite(schedule->alpha[i]) || schedule->relative_velocity[i]<0)return false;
        if(i>0 && schedule->relative_velocity[i]<=schedule->relative_velocity[i-1])return false;
    }
    for(size_t i=0;i<ENTRY_ALPHA_PHASE_COUNT;i++)
        if(!isfinite(schedule->phase_modulation_scale[i]) || schedule->phase_modulation_scale[i]<0)return false;
    return isfinite(schedule->drag_error_gain) &&
        isfinite(schedule->maximum_modulation) && schedule->maximum_modulation>=0 &&
        isfinite(schedule->modulation_rate_limit) && schedule->modulation_rate_limit>0 &&
        isfinite(schedule->target_rate_limit) && schedule->target_rate_limit>0 &&
        isfinite(schedule->minimum_protective_aoa) && schedule->minimum_protective_aoa>=0 &&
        isfinite(schedule->minimum_aero_confidence) && schedule->minimum_aero_confidence>=0 &&
        isfinite(schedule->dynamic_pressure_protect_ratio) && schedule->dynamic_pressure_protect_ratio>=0 && schedule->dynamic_pressure_protect_ratio<1 &&
        isfinite(schedule->g_load_protect_ratio) && schedule->g_load_protect_ratio>=0 && schedule->g_load_protect_ratio<1 &&
        isfinite(schedule->stall_fraction_limit) && schedule->stall_fraction_limit>=0 &&
        isfinite(schedule->calibrated_stall_speed_margin) && schedule->calibrated_stall_speed_margin>=1.0;
}

double entry_alpha_schedule_interpolate(const EntryAlphaSchedule *schedule,
                                        double relative_velocity){
    if(!entry_alpha_schedule_valid(schedule) || !isfinite(relative_velocity))return NAN;
    if(relative_velocity<=schedule->relative_velocity[0])return schedule->alpha[0];
    size_t last=schedule->point_count-1;
    if(relative_velocity>=schedule->relative_velocity[last])return schedule->alpha[last];
    for(size_t i=0;i<last;i++){
        double lo=schedule->relative_velocity[i],hi=schedule->relative_velocity[i+1];
        if(relative_velocity<=hi){
            double u=(relative_velocity-lo)/(hi-lo);
            return schedule->alpha[i]+(schedule->alpha[i+1]-schedule->alpha[i])*u;
        }
    }
    return schedule->alpha[last];
}

double entry_alpha_minimum_drag_tracking_aoa(const EntryAlphaSchedule *schedule,
                                             EntryAlphaPhase phase,
                                             double relative_velocity,
                                             double dynamic_pressure,
                                             double maximum_dynamic_pressure,
                                             double aero_confidence){
    if(!entry_alpha_schedule_valid(schedule) || phase<0 || phase>=ENTRY_ALPHA_PHASE_COUNT ||
       !isfinite(relative_velocity) || relative_velocity<0 ||
       !isfinite(dynamic_pressure) || dynamic_pressure<0 ||
       !isfinite(maximum_dynamic_pressure) || maximum_dynamic_pressure<=0 ||
       !isfinite(aero_confidence))return NAN;

    double nominal=entry_alpha_schedule_interpolate(schedule,relative_velocity);
    if(!isfinite(nominal))return NAN;
    double confidence_scale=1.0;
    if(aero_confidence<schedule->minimum_aero_confidence){
        confidence_scale=0.0;
    }else if(schedule->minimum_aero_confidence<1.0){
        confidence_scale=smoothstep01((aero_confidence-schedule->minimum_aero_confidence)/
            fmax(1.0-schedule->minimum_aero_confidence,1e-6));
    }

    /* entry_alpha_command clamps normalized drag error at -1.5. Evaluating that
       extremum gives the lowest steady drag-tracking modulation this phase can
       request without invoking a structural/stall emergency. */
    double modulation=-1.5*schedule->drag_error_gain*
        schedule->phase_modulation_scale[phase]*confidence_scale;
    modulation*=negative_drag_modulation_scale(phase);
    modulation=clampd(modulation,-schedule->maximum_modulation,schedule->maximum_modulation);
    double desired=clampd(nominal+modulation,
        nominal-schedule->maximum_modulation,nominal+schedule->maximum_modulation);
    desired=fmax(desired,schedule->minimum_protective_aoa);

    double q_ratio=dynamic_pressure/fmax(maximum_dynamic_pressure,1.0);
    if(q_ratio>schedule->dynamic_pressure_protect_ratio){
        double severity=smoothstep01((q_ratio-schedule->dynamic_pressure_protect_ratio)/
            fmax(1.0-schedule->dynamic_pressure_protect_ratio,1e-6));
        desired=fmax(desired,nominal+schedule->maximum_modulation*.75*severity);
    }
    return fmax(0.0,desired);
}

static bool entry_alpha_input_valid(const EntryAlphaInput *input){
    if(!input || input->phase<0 || input->phase>=ENTRY_ALPHA_PHASE_COUNT)return false;
    if(!isfinite(input->relative_velocity) || input->relative_velocity<0 ||
       !isfinite(input->current_aoa) || !isfinite(input->current_aoa_rate) ||
       !isfinite(input->dt) || input->dt<=0 ||
       !isfinite(input->maximum_aoa) || input->maximum_aoa<=0 ||
       !isfinite(input->minimum_safe_speed) || input->minimum_safe_speed<=0 ||
       !isfinite(input->dynamic_pressure) || input->dynamic_pressure<0 ||
       !isfinite(input->maximum_dynamic_pressure) || input->maximum_dynamic_pressure<=0 ||
       !isfinite(input->g_load) || input->g_load<0 ||
       !isfinite(input->maximum_g_load) || input->maximum_g_load<=0 ||
       !isfinite(input->aero_confidence))return false;
    if(input->has_previous_target && !isfinite(input->previous_target_aoa))return false;
    if(input->has_previous_modulation && !isfinite(input->previous_modulation))return false;
    if(isfinite(input->calibrated_stall_speed) && input->calibrated_stall_speed<0)return false;
    return true;
}

EntryAlphaResult entry_alpha_command(const EntryAlphaSchedule *schedule,
                                     const EntryAlphaInput *input){
    EntryAlphaResult result={0};
    result.limiting_reason=ENTRY_ALPHA_LIMIT_INVALID_INPUT;
    if(!entry_alpha_schedule_valid(schedule) || !entry_alpha_input_valid(input)){
        if(input && isfinite(input->current_aoa))result.target_aoa=input->current_aoa;
        return result;
    }

    result.nominal_aoa=entry_alpha_schedule_interpolate(schedule,input->relative_velocity);
    if(!isfinite(result.nominal_aoa))return result;

    result.valid=true;
    result.limiting_reason=ENTRY_ALPHA_LIMIT_NONE;

    double phase_scale=schedule->phase_modulation_scale[input->phase];
    double confidence_scale=1.0;
    if(input->aero_confidence<schedule->minimum_aero_confidence){
        confidence_scale=0.0;
        result.degraded=true;
        set_reason(&result,ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE);
    }else if(schedule->minimum_aero_confidence<1.0){
        confidence_scale=smoothstep01((input->aero_confidence-schedule->minimum_aero_confidence)/
                                      fmax(1.0-schedule->minimum_aero_confidence,1e-6));
    }

    double drag_error=0.0;
    if(isfinite(input->reference_drag_accel) && isfinite(input->measured_drag_accel)){
        double scale=fmax(fabs(input->reference_drag_accel),0.25);
        drag_error=clampd((input->reference_drag_accel-input->measured_drag_accel)/scale,-1.5,1.5);
    }else{
        result.degraded=true;
        confidence_scale=0.0;
        set_reason(&result,ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE);
    }

    double requested_modulation=schedule->drag_error_gain*drag_error*phase_scale*confidence_scale;
    if(requested_modulation<0.0)
        requested_modulation*=negative_drag_modulation_scale(input->phase);
    result.requested_modulation=clampd(requested_modulation,
                                       -schedule->maximum_modulation,schedule->maximum_modulation);
    double previous_mod=input->has_previous_modulation?input->previous_modulation:0.0;
    double modulation_step=schedule->modulation_rate_limit*input->dt;
    result.modulation=clampd(result.requested_modulation,
                             previous_mod-modulation_step,
                             previous_mod+modulation_step);
    if(fabs(result.modulation-result.requested_modulation)>1e-9)
        set_reason(&result,ENTRY_ALPHA_LIMIT_COMMAND_RATE);

    /* The modulation band bounds drag tracking, not safety protections. Apply
       it before the thermal floor and q/g overrides so a valid protective
       floor above the nominal band cannot be silently clipped away. */
    double desired=clampd(result.nominal_aoa+result.modulation,
        result.nominal_aoa-schedule->maximum_modulation,
        result.nominal_aoa+schedule->maximum_modulation);
    double protective_floor=clampd(schedule->minimum_protective_aoa,0.0,input->maximum_aoa);
    if(desired<protective_floor){
        desired=protective_floor;
        set_reason(&result,ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION);
    }
    double q_ratio=input->dynamic_pressure/fmax(input->maximum_dynamic_pressure,1.0);
    if(q_ratio>schedule->dynamic_pressure_protect_ratio){
        double severity=smoothstep01((q_ratio-schedule->dynamic_pressure_protect_ratio)/
                                     fmax(1.0-schedule->dynamic_pressure_protect_ratio,1e-6));
        desired=fmax(desired,result.nominal_aoa+schedule->maximum_modulation*0.75*severity);
        set_reason(&result,ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE);
    }

    double g_ratio=input->g_load/fmax(input->maximum_g_load,0.1);
    if(g_ratio>schedule->g_load_protect_ratio){
        double severity=smoothstep01((g_ratio-schedule->g_load_protect_ratio)/
                                     fmax(1.0-schedule->g_load_protect_ratio,1e-6));
        desired=fmin(desired,result.nominal_aoa-schedule->maximum_modulation*severity);
        set_reason(&result,ENTRY_ALPHA_LIMIT_G_LOAD);
    }

    result.unconstrained_target_aoa=desired;

    double target_reference=input->has_previous_target?input->previous_target_aoa:input->current_aoa;
    double direction=desired-target_reference;
    double rate_limit=schedule->target_rate_limit;
    if(direction*input->current_aoa_rate<0.0)
        rate_limit*=clampd(1.0-fabs(input->current_aoa_rate)/fmax(schedule->target_rate_limit*4.0,1e-6),0.45,1.0);
    double target_step=rate_limit*input->dt;
    double target=clampd(desired,target_reference-target_step,target_reference+target_step);
    if(fabs(target-desired)>1e-9)set_reason(&result,ENTRY_ALPHA_LIMIT_COMMAND_RATE);

    if(target>input->maximum_aoa){
        target=input->maximum_aoa;
        set_reason(&result,ENTRY_ALPHA_LIMIT_MAX_AOA);
    }
    if(target<0.0)target=0.0;

    bool stall_speed_risk=false;
    if(isfinite(input->calibrated_stall_speed) && input->calibrated_stall_speed>0)
        stall_speed_risk=input->relative_velocity<input->calibrated_stall_speed*schedule->calibrated_stall_speed_margin;
    stall_speed_risk=stall_speed_risk || input->relative_velocity<input->minimum_safe_speed*1.10;
    /* The AoA-derived fallback is not independent stall evidence at ANY q.
       Live STS-N entry at 1968 m/s and 1354 Pa crossed that proxy's 0.12
       threshold while tracking an ordinary 22.4-degree incidence. Treating
       it as a measured stall commanded an 11.2-degree unload and seeded a
       low-drag reversal plan. Only an independent stall measurement or the
       airspeed margins above may authorize this thermal-floor escape. */
    bool stall_fraction_risk=input->stall_fraction_is_measured&&
        input->stall_fraction>schedule->stall_fraction_limit;
    if(stall_fraction_risk || stall_speed_risk){
        double stall_cap=isfinite(input->stall_margin_aoa_limit) && input->stall_margin_aoa_limit>0 ?
            input->stall_margin_aoa_limit : fmin(result.nominal_aoa,fmax(4.0,input->maximum_aoa*0.40));
        stall_cap=clampd(stall_cap,0.0,input->maximum_aoa);
        if(target>stall_cap)target=stall_cap;
        set_reason(&result,ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    }else if(isfinite(input->stall_margin_aoa_limit) && input->stall_margin_aoa_limit>0 &&
             target>input->stall_margin_aoa_limit){
        target=clampd(input->stall_margin_aoa_limit,0.0,input->maximum_aoa);
        set_reason(&result,ENTRY_ALPHA_LIMIT_STALL_MARGIN);
    }

    result.target_aoa=target;
    return result;
}

const char *entry_alpha_limit_reason_string(EntryAlphaLimitReason reason){
    switch(reason){
        case ENTRY_ALPHA_LIMIT_NONE:return "none";
        case ENTRY_ALPHA_LIMIT_MAX_AOA:return "maximum-aoa";
        case ENTRY_ALPHA_LIMIT_STALL_MARGIN:return "stall-margin";
        case ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE:return "dynamic-pressure";
        case ENTRY_ALPHA_LIMIT_G_LOAD:return "g-load";
        case ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION:return "thermal-protection";
        case ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE:return "aero-confidence";
        case ENTRY_ALPHA_LIMIT_COMMAND_RATE:return "command-rate";
        case ENTRY_ALPHA_LIMIT_INVALID_INPUT:return "invalid-input";
    }
    return "invalid-input";
}
