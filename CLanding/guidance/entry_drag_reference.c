#include "entry_drag_reference.h"
#include "entry_alpha.h"

#include <math.h>
#include <string.h>

static double smoothstep01(double x){
    x=clampd(x,0.0,1.0);
    return x*x*(3.0-2.0*x);
}

static bool phase_valid(EntryPhase phase){
    return phase>=ENTRY_PHASE_PREENTRY&&phase<=ENTRY_PHASE_TRANSITION;
}

EntryDragReferenceConfig entry_drag_reference_default_config(void){
    EntryDragReferenceConfig c;
    memset(&c,0,sizeof(c));
    c.entry_velocity_ratio=3.40;
    c.temperature_end_progress=.30;
    c.equilibrium_end_progress=.58;
    c.constant_end_progress=.82;
    c.temperature_start_drag_ratio=.55;
    c.equilibrium_blend=.34;
    c.equilibrium_anchor_weight=.30;
    c.range_gain=.80;
    c.minimum_range_authority=.40;
    c.minimum_drag_scale=.55;
    c.maximum_drag_scale=1.75;
    c.range_error_scale_fraction=.28;
    c.minimum_range_error_scale=10000.0;
    c.vertical_drag_gain=.28;
    c.vertical_range_gain=.22;
    c.maximum_vertical_lift_correction=.72;
    c.maximum_drag_g_fraction=.72;
    c.local_drag_authority_multiplier=1.85;
    c.minimum_local_drag_confidence=.20;
    c.minimum_drag_accel=.04;
    c.prediction_steps=96;
    return c;
}

bool entry_drag_reference_config_valid(const EntryDragReferenceConfig*c){
    if(!c)return false;
    return isfinite(c->entry_velocity_ratio)&&c->entry_velocity_ratio>1.05&&
        isfinite(c->temperature_end_progress)&&c->temperature_end_progress>0&&
        isfinite(c->equilibrium_end_progress)&&c->equilibrium_end_progress>c->temperature_end_progress&&
        isfinite(c->constant_end_progress)&&c->constant_end_progress>c->equilibrium_end_progress&&c->constant_end_progress<1&&
        isfinite(c->temperature_start_drag_ratio)&&c->temperature_start_drag_ratio>0&&c->temperature_start_drag_ratio<=1.5&&
        isfinite(c->equilibrium_blend)&&c->equilibrium_blend>=0&&c->equilibrium_blend<=1&&
        isfinite(c->equilibrium_anchor_weight)&&c->equilibrium_anchor_weight>=0&&c->equilibrium_anchor_weight<=1&&
        isfinite(c->range_gain)&&c->range_gain>=0&&
        isfinite(c->minimum_range_authority)&&c->minimum_range_authority>=0&&c->minimum_range_authority<=1&&
        isfinite(c->minimum_drag_scale)&&c->minimum_drag_scale>0&&
        isfinite(c->maximum_drag_scale)&&c->maximum_drag_scale>=c->minimum_drag_scale&&
        isfinite(c->range_error_scale_fraction)&&c->range_error_scale_fraction>0&&
        isfinite(c->minimum_range_error_scale)&&c->minimum_range_error_scale>0&&
        isfinite(c->vertical_drag_gain)&&c->vertical_drag_gain>=0&&
        isfinite(c->vertical_range_gain)&&c->vertical_range_gain>=0&&
        isfinite(c->maximum_vertical_lift_correction)&&c->maximum_vertical_lift_correction>=0&&c->maximum_vertical_lift_correction<1&&
        isfinite(c->maximum_drag_g_fraction)&&c->maximum_drag_g_fraction>0&&c->maximum_drag_g_fraction<=1&&
        isfinite(c->local_drag_authority_multiplier)&&c->local_drag_authority_multiplier>=1.0&&c->local_drag_authority_multiplier<=5.0&&
        isfinite(c->minimum_local_drag_confidence)&&c->minimum_local_drag_confidence>=0&&c->minimum_local_drag_confidence<=1.0&&
        isfinite(c->minimum_drag_accel)&&c->minimum_drag_accel>0&&
        c->prediction_steps>=16&&c->prediction_steps<=512;
}

static bool input_valid(const EntryDragReferenceInput*i){
    return i&&phase_valid(i->phase)&&i->planet&&i->vehicle&&
        isfinite(i->relative_velocity)&&i->relative_velocity>=0&&
        isfinite(i->latitude)&&isfinite(i->altitude)&&
        isfinite(i->range_to_site)&&i->range_to_site>=0&&
        isfinite(i->taem_range)&&i->taem_range>=0&&
        isfinite(i->taem_latitude)&&isfinite(i->taem_altitude)&&
        isfinite(i->taem_velocity)&&i->taem_velocity>0&&
        (!i->has_incidence||(isfinite(i->angle_of_attack)&&isfinite(i->sideslip)))&&
        isfinite(i->planet->radius)&&i->planet->radius>0&&
        isfinite(i->planet->gravitational_parameter)&&i->planet->gravitational_parameter>0&&
        isfinite(i->vehicle->estimated_lift_to_drag)&&i->vehicle->estimated_lift_to_drag>0&&
        isfinite(i->vehicle->estimated_ballistic_coefficient)&&i->vehicle->estimated_ballistic_coefficient>0&&
        isfinite(i->vehicle->maximum_dynamic_pressure)&&i->vehicle->maximum_dynamic_pressure>0&&
        isfinite(i->vehicle->maximum_g_load)&&i->vehicle->maximum_g_load>0;
}

static double local_gravity(const PlanetModel*p,double altitude){
    double r=fmax(1.0,p->radius+altitude);
    return p->gravitational_parameter/(r*r);
}

static double entry_anchor_velocity(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c){
    return fmax(i->taem_velocity+1.0,i->taem_velocity*c->entry_velocity_ratio);
}

static double progress_for_velocity(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,double velocity){
    double entry=entry_anchor_velocity(i,c),span=fmax(1.0,entry-i->taem_velocity);
    return clampd((entry-velocity)/span,0.0,1.0);
}

static double velocity_for_progress(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,double progress){
    double entry=entry_anchor_velocity(i,c);
    return entry-clampd(progress,0.0,1.0)*(entry-i->taem_velocity);
}

static EntryAlphaPhase alpha_phase_for_entry_phase(EntryPhase phase){
    switch(phase){
        case ENTRY_PHASE_PREENTRY:return ENTRY_ALPHA_PRE_ENTRY;
        case ENTRY_PHASE_TEMPERATURE_CONTROL:return ENTRY_ALPHA_TEMPERATURE_CONTROL;
        case ENTRY_PHASE_EQUILIBRIUM_GLIDE:return ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
        case ENTRY_PHASE_CONSTANT_DRAG:return ENTRY_ALPHA_CONSTANT_DRAG;
        case ENTRY_PHASE_TRANSITION:return ENTRY_ALPHA_TRANSITION;
    }
    return ENTRY_ALPHA_PRE_ENTRY;
}

static EntryAlphaPhase alpha_phase_for_profile_velocity(const EntryDragReferenceInput*i,
        const EntryDragReferenceConfig*c,double velocity){
    double p=progress_for_velocity(i,c,velocity);
    EntryAlphaPhase projected=ENTRY_ALPHA_TEMPERATURE_CONTROL;
    if(p>c->constant_end_progress)projected=ENTRY_ALPHA_TRANSITION;
    else if(p>c->equilibrium_end_progress)projected=ENTRY_ALPHA_CONSTANT_DRAG;
    else if(p>c->temperature_end_progress)projected=ENTRY_ALPHA_EQUILIBRIUM_GLIDE;
    EntryAlphaPhase current=alpha_phase_for_entry_phase(i->phase);
    return projected>current?projected:current;
}

static double maximum_reference_drag(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c){
    double g=local_gravity(i->planet,i->taem_altitude);
    double by_g=c->maximum_drag_g_fraction*i->vehicle->maximum_g_load*g;
    double by_q=i->vehicle->maximum_dynamic_pressure/i->vehicle->estimated_ballistic_coefficient;
    double cap=by_g;
    if(isfinite(by_q)&&by_q>0)cap=fmin(cap,by_q);
    return fmax(c->minimum_drag_accel,cap);
}

static double achievable_profile_drag_cap(const EntryDragReferenceInput*i,
        const EntryDragReferenceConfig*c,double velocity){
    double structural=maximum_reference_drag(i,c);
    double confidence=clampd(isfinite(i->aero_confidence)?i->aero_confidence:0.0,0.0,1.0);
    if(confidence<c->minimum_local_drag_confidence)return structural;

    bool measured=isfinite(i->measured_drag_accel)&&i->measured_drag_accel>c->minimum_drag_accel;
    bool modeled=isfinite(i->modeled_drag_accel)&&i->modeled_drag_accel>c->minimum_drag_accel;
    if(!measured&&!modeled)return structural;
    double achieved=measured&&modeled?fmax(i->measured_drag_accel,i->modeled_drag_accel):
        (measured?i->measured_drag_accel:i->modeled_drag_accel);

    /* Calibrate q/beta to the drag actually demonstrated at the current state,
       then project that calibration along a smooth current-altitude -> TAEM-
       altitude path. This bounds the reduced 1-D predictor by local aerodynamic
       authority without freezing future drag at today's thin-air value. */
    double beta=fmax(1.0,i->vehicle->estimated_ballistic_coefficient);
    double rho_now=fmax(0.0,planet_atmospheric_density(i->planet,i->altitude));
    double ballistic_now=.5*rho_now*i->relative_velocity*i->relative_velocity/beta;
    /* q/beta is an acceleration. Calibration only requires a positive finite
       baseline; comparing it with a fraction of an unrelated behavioral floor
       discards valid measured evidence and invents a confidence threshold. */
    if(!isfinite(ballistic_now)||!(ballistic_now>0.0))return structural;
    double calibration=clampd(achieved/ballistic_now,.35,3.0);

    double span=fmax(1.0,i->relative_velocity-i->taem_velocity);
    double remaining=clampd((velocity-i->taem_velocity)/span,0.0,1.0);
    double altitude=i->taem_altitude+(i->altitude-i->taem_altitude)*smoothstep01(remaining);
    double rho=fmax(0.0,planet_atmospheric_density(i->planet,altitude));
    double projected=.5*rho*velocity*velocity/beta*calibration*c->local_drag_authority_multiplier;
    if(!(projected>0)||!isfinite(projected))return structural;
    return clampd(projected,c->minimum_drag_accel,structural);
}

static double achievable_profile_drag_floor(const EntryDragReferenceInput*i,
        const EntryDragReferenceConfig*c,double velocity){
    if(!i->has_incidence)return c->minimum_drag_accel;
    double confidence=clampd(isfinite(i->aero_confidence)?i->aero_confidence:0.0,0.0,1.0);
    if(confidence<c->minimum_local_drag_confidence)return c->minimum_drag_accel;

    bool measured=isfinite(i->measured_drag_accel)&&i->measured_drag_accel>c->minimum_drag_accel;
    bool modeled=isfinite(i->modeled_drag_accel)&&i->modeled_drag_accel>c->minimum_drag_accel;
    if(!measured&&!modeled)return c->minimum_drag_accel;
    /* Use the lower trustworthy anchor. A minimum-drag envelope must not become
       pessimistic because one force source happens to read high. In live MM304 the
       measured source normally wins and this remains anchored to observed energy
       loss rather than the nominal ballistic coefficient. */
    double achieved=measured&&modeled?fmin(i->measured_drag_accel,i->modeled_drag_accel):
        (measured?i->measured_drag_accel:i->modeled_drag_accel);

    double rho_now=fmax(0.0,planet_atmospheric_density(i->planet,i->altitude));
    double q_now=.5*rho_now*i->relative_velocity*i->relative_velocity;
    /* Dynamic pressure is measured in Pa, so it must never be compared with the
       minimum drag acceleration (m/s^2). Only the positive-flow domain is needed
       before using q ratios below. */
    if(!isfinite(q_now)||!(q_now>0.0))return c->minimum_drag_accel;
    double sound_now=planet_atmospheric_speed_of_sound(i->planet,i->altitude);
    double mach_now=sound_now>1.0?i->relative_velocity/sound_now:5.0;
    double current_incidence=hypot(i->angle_of_attack,i->sideslip);
    double current_df=1.0,floor_df=1.0;
    aerodynamic_force_factors_mach(mach_now,current_incidence,i->vehicle,NULL,&current_df);

    EntryAlphaSchedule alpha_schedule;
    entry_alpha_schedule_default(&alpha_schedule,i->vehicle,i->taem_velocity);
    EntryAlphaPhase floor_phase=alpha_phase_for_profile_velocity(i,c,i->relative_velocity);
    double floor_aoa=entry_alpha_minimum_drag_tracking_aoa(&alpha_schedule,floor_phase,
        i->relative_velocity,q_now,i->vehicle->maximum_dynamic_pressure,confidence);
    if(!isfinite(floor_aoa))floor_aoa=entry_low_q_protective_aoa_floor(q_now,i->vehicle);
    aerodynamic_force_factors_mach(mach_now,floor_aoa,i->vehicle,NULL,&floor_df);
    double floor_now=achieved*fmax(.02,floor_df)/fmax(.05,current_df);

    /* Project the measured lower envelope along the same smooth current->TAEM
       altitude path used by the upper authority cap. The incidence floor comes
       from the actual @ALPHA steady drag-tracking law, including its asymmetric
       negative modulation and q protection. Finite modulation/target-rate lag can
       only keep incidence (and drag) above this lower envelope during unloading. */
    double span=fmax(1.0,i->relative_velocity-i->taem_velocity);
    double remaining=clampd((velocity-i->taem_velocity)/span,0.0,1.0);
    double altitude=i->taem_altitude+(i->altitude-i->taem_altitude)*smoothstep01(remaining);
    double rho=fmax(0.0,planet_atmospheric_density(i->planet,altitude));
    double q=.5*rho*velocity*velocity;
    if(!(q>0.0)||!isfinite(q))return c->minimum_drag_accel;
    double sound=planet_atmospheric_speed_of_sound(i->planet,altitude);
    double mach=sound>1.0?velocity/sound:mach_now;
    EntryAlphaPhase projected_phase=alpha_phase_for_profile_velocity(i,c,velocity);
    double projected_aoa=entry_alpha_minimum_drag_tracking_aoa(&alpha_schedule,projected_phase,
        velocity,q,i->vehicle->maximum_dynamic_pressure,confidence);
    if(!isfinite(projected_aoa))projected_aoa=entry_low_q_protective_aoa_floor(q,i->vehicle);
    double projected_df=1.0;
    aerodynamic_force_factors_mach(mach,projected_aoa,i->vehicle,NULL,&projected_df);
    double projected=floor_now*(q/q_now)*fmax(.02,projected_df)/fmax(.02,floor_df);
    return clampd(projected,c->minimum_drag_accel,maximum_reference_drag(i,c));
}

static double equilibrium_vertical_lift(const EntryDragReferenceInput*i,double velocity,double altitude){
    double r=fmax(1.0,i->planet->radius+altitude),g=local_gravity(i->planet,altitude);
    double required=g-velocity*velocity/r;
    /* A small gravity fraction keeps the nominal demand well behaved around the
       near-orbital end of Entry without manufacturing a bank command in vacuum. */
    return clampd(required,.12*g,.995*g);
}

static double equilibrium_drag_at(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,double velocity){
    double lift=equilibrium_vertical_lift(i,velocity,i->taem_altitude);
    double drag=lift/fmax(.05,i->vehicle->estimated_lift_to_drag);
    return clampd(drag,c->minimum_drag_accel,maximum_reference_drag(i,c));
}

static double taem_drag_at_target(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,double fallback){
    double density=planet_atmospheric_density(i->planet,i->taem_altitude);
    if(!(density>0)&&!(i->taem_altitude>=i->planet->atmosphere_depth))return fallback;
    double q=.5*fmax(0.0,density)*i->taem_velocity*i->taem_velocity;
    double drag=q/fmax(1.0,i->vehicle->estimated_ballistic_coefficient);
    if(!(drag>0)||!isfinite(drag))return fallback;
    return clampd(drag,c->minimum_drag_accel,maximum_reference_drag(i,c));
}

static double profile_drag_at(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,
        double constant_drag,double taem_drag,double velocity,double scale){
    double p=progress_for_velocity(i,c,velocity),drag=constant_drag;
    if(p<=c->temperature_end_progress){
        double x=p/fmax(c->temperature_end_progress,1e-6);
        double ratio=c->temperature_start_drag_ratio+
            (1.0-c->temperature_start_drag_ratio)*smoothstep01(x);
        drag=constant_drag*ratio;
    }else if(p<=c->equilibrium_end_progress){
        double x=(p-c->temperature_end_progress)/
            fmax(c->equilibrium_end_progress-c->temperature_end_progress,1e-6);
        double window=sin(LANDER_PI*x);
        window*=window;
        double eq=equilibrium_drag_at(i,c,velocity);
        drag=constant_drag+(eq-constant_drag)*c->equilibrium_blend*window;
    }else if(p<=c->constant_end_progress){
        drag=constant_drag;
    }else{
        double x=(p-c->constant_end_progress)/fmax(1.0-c->constant_end_progress,1e-6);
        drag=constant_drag+(taem_drag-constant_drag)*smoothstep01(x);
    }
    return clampd(drag*scale,c->minimum_drag_accel,achievable_profile_drag_cap(i,c,velocity));
}

static double feasible_profile_drag_at(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,
        double constant_drag,double taem_drag,double velocity,double scale){
    double desired=profile_drag_at(i,c,constant_drag,taem_drag,velocity,scale);
    double lower=achievable_profile_drag_floor(i,c,velocity);
    double upper=achievable_profile_drag_cap(i,c,velocity);
    if(lower>upper)lower=upper;
    return clampd(desired,lower,upper);
}

static double predicted_range(const EntryDragReferenceInput*i,const EntryDragReferenceConfig*c,
        double constant_drag,double taem_drag,double scale,double remaining_energy){
    if(remaining_energy<=0)return 0.0;

    /* dE/ds = -D.  The velocity integral below accounts exactly for the kinetic
       portion of the remaining specific energy under the reduced 1-D model.
       Altitude/rotation energy is then removed at a representative geometric-mean
       drag.  Unlike a pure V integral this remains physical when velocity is already
       near the TAEM target but the vehicle is still high. */
    double signed_kinetic=.5*(i->relative_velocity*i->relative_velocity-
        i->taem_velocity*i->taem_velocity);
    double range=0.0;
    if(i->relative_velocity>i->taem_velocity){
        unsigned steps=c->prediction_steps;
        double dv=(i->relative_velocity-i->taem_velocity)/(double)steps;
        for(unsigned n=0;n<steps;n++){
            double velocity=i->taem_velocity+((double)n+.5)*dv;
            double drag=feasible_profile_drag_at(i,c,constant_drag,taem_drag,velocity,scale);
            range+=velocity/fmax(c->minimum_drag_accel,drag)*dv;
        }
    }

    double nonkinetic=remaining_energy-fmax(0.0,signed_kinetic);
    if(nonkinetic!=0.0){
        double current_drag=feasible_profile_drag_at(i,c,constant_drag,taem_drag,
            fmax(i->relative_velocity,i->taem_velocity),scale);
        double target_drag=feasible_profile_drag_at(i,c,constant_drag,taem_drag,i->taem_velocity,scale);
        double representative=sqrt(fmax(c->minimum_drag_accel,current_drag)*
            fmax(c->minimum_drag_accel,target_drag));
        range+=nonkinetic/fmax(c->minimum_drag_accel,representative);
    }
    return fmax(0.0,range);
}

static double geometric_blend(double a,double b,double b_weight){
    a=fmax(a,1e-9);b=fmax(b,1e-9);b_weight=clampd(b_weight,0.0,1.0);
    return exp((1.0-b_weight)*log(a)+b_weight*log(b));
}

EntryDragReferenceOutput entry_drag_reference_compute(const EntryDragReferenceInput*i,
        const EntryDragReferenceConfig*c){
    EntryDragReferenceOutput out;
    memset(&out,0,sizeof(out));
    if(!entry_drag_reference_config_valid(c)||!input_valid(i))return out;

    out.valid=true;
    out.active=i->phase!=ENTRY_PHASE_PREENTRY;
    out.entry_anchor_velocity=entry_anchor_velocity(i,c);
    out.profile_progress=progress_for_velocity(i,c,i->relative_velocity);
    out.range_authority=c->minimum_range_authority+
        (1.0-c->minimum_range_authority)*smoothstep01(
            out.profile_progress/fmax(c->constant_end_progress,1e-6));

    double remaining_energy=entry_remaining_specific_energy(i->latitude,i->altitude,i->relative_velocity,
        i->taem_latitude,i->taem_altitude,i->taem_velocity,i->planet);
    if(!isfinite(remaining_energy)){memset(&out,0,sizeof(out));return out;}

    out.available_range_to_taem=fmax(0.0,i->range_to_site-i->taem_range);
    double sizing_range=fmax(1000.0,out.available_range_to_taem);
    out.required_average_drag_accel=clampd(fmax(0.0,remaining_energy)/sizing_range,
        c->minimum_drag_accel,maximum_reference_drag(i,c));

    double equilibrium_end_velocity=velocity_for_progress(i,c,c->equilibrium_end_progress);
    double equilibrium_anchor=equilibrium_drag_at(i,c,equilibrium_end_velocity);
    out.constant_drag_accel=geometric_blend(out.required_average_drag_accel,equilibrium_anchor,
        c->equilibrium_anchor_weight);
    out.constant_drag_accel=clampd(out.constant_drag_accel,c->minimum_drag_accel,
        maximum_reference_drag(i,c));
    out.equilibrium_drag_accel=equilibrium_drag_at(i,c,i->relative_velocity);
    out.taem_drag_accel=taem_drag_at_target(i,c,out.constant_drag_accel);
    out.minimum_achievable_drag_accel=achievable_profile_drag_floor(i,c,i->relative_velocity);
    out.nominal_drag_accel=profile_drag_at(i,c,out.constant_drag_accel,out.taem_drag_accel,
        i->relative_velocity,1.0);

    out.range_error_scale=fmax(c->minimum_range_error_scale,
        fmax(1000.0,out.available_range_to_taem)*c->range_error_scale_fraction);
    out.unshaped_predicted_range=predicted_range(i,c,out.constant_drag_accel,out.taem_drag_accel,
        1.0,fmax(0.0,remaining_energy));
    out.unshaped_range_error=out.unshaped_predicted_range-out.available_range_to_taem;
    double normalized=out.unshaped_range_error/out.range_error_scale;
    double exponent=c->range_gain*out.range_authority*tanh(normalized);
    out.drag_range_scale=clampd(exp(exponent),c->minimum_drag_scale,c->maximum_drag_scale);
    out.reference_drag_accel=profile_drag_at(i,c,out.constant_drag_accel,out.taem_drag_accel,
        i->relative_velocity,out.drag_range_scale);
    out.drag_floor_active=out.minimum_achievable_drag_accel>c->minimum_drag_accel+1e-6&&
        out.reference_drag_accel<out.minimum_achievable_drag_accel-1e-6;
    /* The command and range integration must consume the same achievable
       profile. A below-floor request falsely demanded extra vertical lift even
       though alpha could not shed the implied drag. Keep drag_floor_active as
       explicit evidence that the unconstrained mission request was impossible. */
    out.reference_drag_accel=feasible_profile_drag_at(i,c,out.constant_drag_accel,out.taem_drag_accel,
        i->relative_velocity,out.drag_range_scale);
    out.predicted_range_to_taem=predicted_range(i,c,out.constant_drag_accel,out.taem_drag_accel,
        out.drag_range_scale,fmax(0.0,remaining_energy));
    out.predicted_range_error=out.predicted_range_to_taem-out.available_range_to_taem;

    bool measured=isfinite(i->measured_drag_accel)&&i->measured_drag_accel>=0.0;
    bool modeled=isfinite(i->modeled_drag_accel)&&i->modeled_drag_accel>=0.0;
    double aero=clampd(isfinite(i->aero_confidence)?i->aero_confidence:0.0,0.0,1.0);
    if(measured){
        out.observed_drag_accel=i->measured_drag_accel;
        out.confidence=.50+.50*aero;
    }else if(modeled){
        out.observed_drag_accel=i->modeled_drag_accel;
        out.confidence=.30+.45*aero;
        out.degraded=true;
    }else{
        /* No trustworthy force source: keep the downstream correction neutral. */
        out.observed_drag_accel=out.reference_drag_accel;
        out.confidence=.12+.18*aero;
        out.degraded=true;
    }
    if(aero<.20)out.degraded=true;
    out.drag_error_accel=out.reference_drag_accel-out.observed_drag_accel;
    out.drag_error_fraction=out.drag_error_accel/fmax(.25,fabs(out.reference_drag_accel));

    double delta=fmax(.5,i->relative_velocity*.001);
    double lo=fmax(i->taem_velocity,i->relative_velocity-delta),hi=i->relative_velocity+delta;
    if(hi>lo){
        double dlo=feasible_profile_drag_at(i,c,out.constant_drag_accel,out.taem_drag_accel,lo,out.drag_range_scale);
        double dhi=feasible_profile_drag_at(i,c,out.constant_drag_accel,out.taem_drag_accel,hi,out.drag_range_scale);
        out.reference_drag_dv=(dhi-dlo)/(hi-lo);
    }
    out.predicted_range_d_drag=-out.predicted_range_to_taem/
        fmax(c->minimum_drag_accel,out.reference_drag_accel);

    double vertical=equilibrium_vertical_lift(i,i->relative_velocity,i->altitude);
    double range_fraction=out.predicted_range_error/out.range_error_scale;
    double correction=c->vertical_drag_gain*clampd(out.drag_error_fraction,-1.5,1.5)+
        c->vertical_range_gain*clampd(range_fraction,-1.5,1.5);
    correction=clampd(correction,-c->maximum_vertical_lift_correction,
        c->maximum_vertical_lift_correction);
    out.required_vertical_lift_accel=vertical*clampd(1.0-correction,.10,1.50);

    if(remaining_energy<=0||i->relative_velocity<=i->taem_velocity)out.degraded=true;
    out.confidence=clampd(out.confidence,0.0,1.0);
    return out;
}

EntryExecProfile entry_drag_reference_exec_profile(const EntryDragReferenceInput*i,
        const EntryDragReferenceConfig*c,const EntryDragReferenceOutput*r){
    EntryExecProfile p;
    memset(&p,0,sizeof(p));
    if(!i||!c||!r||!r->valid||!entry_drag_reference_config_valid(c)||!input_valid(i))return p;

    double temp_end=velocity_for_progress(i,c,c->temperature_end_progress);
    double eq_end=velocity_for_progress(i,c,c->equilibrium_end_progress);
    double const_end=velocity_for_progress(i,c,c->constant_end_progress);
    p.has_temperature_velocity_gate=true;p.temperature_end_velocity=temp_end;
    p.equilibrium_intercept_valid=true;p.equilibrium_intercept=i->relative_velocity<=eq_end;
    p.has_constant_drag_velocity_gate=true;p.constant_drag_end_velocity=const_end;

    p.has_phase_floor=true;
    if(i->relative_velocity<=const_end)p.phase_floor=ENTRY_PHASE_TRANSITION;
    else if(i->relative_velocity<=eq_end)p.phase_floor=ENTRY_PHASE_CONSTANT_DRAG;
    else if(i->relative_velocity<=temp_end)p.phase_floor=ENTRY_PHASE_EQUILIBRIUM_GLIDE;
    else p.phase_floor=ENTRY_PHASE_TEMPERATURE_CONTROL;

    return p;
}
