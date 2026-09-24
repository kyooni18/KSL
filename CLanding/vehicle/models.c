#include "landing.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static void copystr(char *dst,size_t cap,const char *src){if(cap==0)return;if(!src)src="";snprintf(dst,cap,"%s",src);}

const char *connection_status_string(ConnectionStatus s){static const char *v[]={"disconnected","connecting","connected","failed"};return (unsigned)s<4?v[s]:v[0];}
const char *phase_string(GuidancePhase p){static const char *v[]={"Idle","Planning","Calibration Glide","Coast to Burn","Burn Setup","Deorbit Burn","Entry Interface","MM304 Entry","TAEM","Heading Alignment","Final Approach","Flare","Touchdown","Rollout","Complete","Paused","Abort","Fault","Attitude Recovery"};return (unsigned)p<19?v[p]:v[0];}
const char *speed_mode_string(NavballSpeedMode s){static const char *v[]={"unchanged","orbit","surface"};return (unsigned)s<3?v[s]:v[0];}
const char *profile_string(ControlProfile p){static const char *v[]={"orbital","entry","taem","approach","flare","rollout","recovery"};return (unsigned)p<7?v[p]:v[0];}
const char *cal_state_string(CalibrationRunState s){static const char *v[]={"Idle","Settling","Sampling","Complete","Stopped","Aborted"};return (unsigned)s<6?v[s]:v[0];}

LandingConfiguration landing_configuration_default(void){
    LandingConfiguration c;memset(&c,0,sizeof(c));
    copystr(c.connection.serial_port,sizeof(c.connection.serial_port),"/dev/tty.krpc");c.connection.baud_rate=921600;c.connection.timeout_ms=2000;copystr(c.connection.client_name,sizeof(c.connection.client_name),"Spaceplane Landing Guidance");
    copystr(c.site.name,sizeof(c.site.name),"KSC Runway 09");c.site.latitude=-0.0486111111;c.site.longitude=-74.7283333333;c.site.altitude=70;c.site.runway_heading=90;c.site.runway_length=2500;c.site.runway_width=70;c.site.allow_reciprocal_runway=true;
    copystr(c.vehicle.model_id,sizeof(c.vehicle.model_id),"STS-N");c.vehicle.estimated_lift_to_drag=.4;c.vehicle.estimated_ballistic_coefficient=700;c.vehicle.entry_angle_of_attack=18;c.vehicle.maximum_angle_of_attack=28;c.vehicle.terminal_maximum_lift_angle_of_attack=15;c.vehicle.maximum_bank_angle=70;c.vehicle.maximum_dynamic_pressure=45000;c.vehicle.maximum_g_load=3.5;c.vehicle.minimum_safe_speed=85;c.vehicle.final_approach_speed=115;c.vehicle.touchdown_speed=75;c.vehicle.allow_powered_approach=false;c.vehicle.maximum_approach_throttle=0;c.vehicle.airbrake_action_group=1;
    GuidanceSettings *g=&c.guidance;g->target_deorbit_capture_radius=100000;g->entry_interface_altitude_margin=500;g->target_entry_range=1030000;g->target_entry_flight_path_angle=-2.0;g->maximum_entry_flight_path_angle=-3.0;g->target_post_burn_periapsis_altitude=50000;g->deorbit_maximum_throttle=.18;g->deorbit_throttle_ramp_duration=12;g->deorbit_timing_uncertainty=.5;g->deorbit_thrust_uncertainty_fraction=.08;g->deorbit_delta_v_uncertainty=.5;g->deorbit_mass_uncertainty_fraction=.03;g->deorbit_position_uncertainty=50;g->deorbit_velocity_uncertainty=.5;g->deorbit_pointing_uncertainty=2;g->deorbit_atmosphere_uncertainty_fraction=.12;g->deorbit_robust_minimum_pass_fraction=.75;g->taem_interface_altitude=25000;g->taem_interface_range=96500;g->taem_force_handoff_speed=1300;g->mm304_handoff_radius=1000;g->mm304_perpendicular_heading_half_width=30;g->mm304_handoff_along_track=-8000;g->mm304_handoff_cross_track=0;g->mm305_min_altitude=22000;g->mm305_max_altitude=30000;g->mm305_target_mach=2.5;g->mm305_mach_half_width=.35;g->hac_acquisition_altitude=10700;g->hac_acquisition_mach=.95;g->hac_radius=12000;g->hac_look_ahead_angle=16;g->final_approach_distance=8000;g->final_alignment_speed=160;g->final_glide_slope=20;g->taem_glide_slope=12;g->gear_deployment_altitude=1500;g->flare_altitude=55;g->touchdown_sink_rate=-1.5;g->guidance_rate=10;g->prediction_interval=1.5;g->s_turn_minimum_leg_duration=24;g->entry_roll_rate=4.5;g->entry_roll_acceleration=1.8;g->taem_roll_rate=6;g->approach_roll_rate=3.5;g->use_time_warp=true;g->minimum_planning_lead_time=90;
    CalibrationSettings *a=&c.calibration;a->enable_passive_calibration=true;a->enable_trajectory_calibration=true;a->auto_apply_in_flight=true;a->minimum_dynamic_pressure=500;a->maximum_sample_dynamic_pressure=55000;a->maximum_sideslip=8;a->minimum_calibration_radar_altitude=3000;a->maximum_calibration_dynamic_pressure=35000;a->maximum_calibration_g_load=2.5;a->maximum_calibration_sink_rate=80;a->abort_stall_fraction=.32;a->minimum_angle_of_attack=3;a->maximum_angle_of_attack=18;a->angle_of_attack_step=3;a->settling_duration=4;a->sampling_duration=8;a->trajectory_learning_rate=.18;
    return c;
}

static double finite_or(double value,double fallback){
    return isfinite(value)?value:fallback;
}

static double positive_or(double value,double fallback){
    return isfinite(value)&&value>0.0?value:fallback;
}

static double nonnegative_or(double value,double fallback){
    return isfinite(value)&&value>=0.0?value:fallback;
}

static double fraction_or(double value,double fallback){
    if(!isfinite(value))return fallback;
    return clampd(value,0.0,1.0); /* decision-literal: protocol-domain-requirement | normalized fractions are defined on the closed zero-to-one domain */
}

static double descent_angle_or(double value,double fallback){
    if(!isfinite(value)||!(value>0.0)||!(value<90.0))return fallback; /* decision-literal: mathematical-numerical-requirement | a finite descent angle lies strictly between zero and ninety degrees */
    return value;
}

void landing_configuration_normalize(LandingConfiguration *c){
    if(!c)return;
    LandingConfiguration d=landing_configuration_default();

    if(!c->connection.serial_port[0])
        copystr(c->connection.serial_port,sizeof(c->connection.serial_port),d.connection.serial_port);
    if(!c->connection.client_name[0])
        copystr(c->connection.client_name,sizeof(c->connection.client_name),d.connection.client_name);
    if(c->connection.baud_rate<=0)c->connection.baud_rate=d.connection.baud_rate;
    if(c->connection.timeout_ms<=0)c->connection.timeout_ms=d.connection.timeout_ms;

    c->site.latitude=clampd(finite_or(c->site.latitude,d.site.latitude),-90.0,90.0); /* decision-literal: protocol-domain-requirement | geographic latitude is defined from minus ninety through plus ninety degrees */
    c->site.longitude=norm_signed_deg(finite_or(c->site.longitude,d.site.longitude));
    c->site.altitude=finite_or(c->site.altitude,d.site.altitude);
    c->site.runway_heading=norm_deg(finite_or(c->site.runway_heading,d.site.runway_heading));
    c->site.runway_length=positive_or(c->site.runway_length,d.site.runway_length);
    c->site.runway_width=positive_or(c->site.runway_width,d.site.runway_width);

    VehicleProfile *v=&c->vehicle;
    const VehicleProfile *vd=&d.vehicle;
    if(!v->model_id[0])copystr(v->model_id,sizeof(v->model_id),vd->model_id);
    v->estimated_lift_to_drag=positive_or(v->estimated_lift_to_drag,vd->estimated_lift_to_drag);
    v->estimated_ballistic_coefficient=positive_or(v->estimated_ballistic_coefficient,vd->estimated_ballistic_coefficient);
    if(!isfinite(v->maximum_angle_of_attack)||!(v->maximum_angle_of_attack>0.0)||
       !(v->maximum_angle_of_attack<90.0)) /* decision-literal: mathematical-numerical-requirement | incidence must remain below the ninety-degree geometric singular boundary */
        v->maximum_angle_of_attack=vd->maximum_angle_of_attack;
    v->entry_angle_of_attack=clampd(finite_or(v->entry_angle_of_attack,vd->entry_angle_of_attack),
        0.0,v->maximum_angle_of_attack);
    v->terminal_maximum_lift_angle_of_attack=clampd(positive_or(
        v->terminal_maximum_lift_angle_of_attack,vd->terminal_maximum_lift_angle_of_attack),
        1.0,v->maximum_angle_of_attack);
    if(!isfinite(v->maximum_bank_angle)||!(v->maximum_bank_angle>0.0)||
       !(v->maximum_bank_angle<90.0)) /* decision-literal: mathematical-numerical-requirement | bank remains below ninety degrees where vertical lift projection stays nonnegative */
        v->maximum_bank_angle=vd->maximum_bank_angle;
    v->maximum_dynamic_pressure=positive_or(v->maximum_dynamic_pressure,vd->maximum_dynamic_pressure);
    v->maximum_g_load=positive_or(v->maximum_g_load,vd->maximum_g_load);
    v->minimum_safe_speed=positive_or(v->minimum_safe_speed,vd->minimum_safe_speed);
    v->final_approach_speed=positive_or(v->final_approach_speed,vd->final_approach_speed);
    v->final_approach_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);
    v->touchdown_speed=positive_or(v->touchdown_speed,vd->touchdown_speed);
    v->touchdown_speed=fmin(v->touchdown_speed,v->final_approach_speed);
    v->maximum_approach_throttle=fraction_or(v->maximum_approach_throttle,vd->maximum_approach_throttle);
    if(v->airbrake_action_group<1||v->airbrake_action_group>10) /* decision-literal: protocol-domain-requirement | KSP custom action groups are numbered one through ten */
        v->airbrake_action_group=vd->airbrake_action_group;

    GuidanceSettings *g=&c->guidance;
    const GuidanceSettings *gd=&d.guidance;
    g->target_deorbit_capture_radius=positive_or(g->target_deorbit_capture_radius,gd->target_deorbit_capture_radius);
    g->entry_interface_altitude_margin=nonnegative_or(g->entry_interface_altitude_margin,gd->entry_interface_altitude_margin);
    g->target_entry_range=positive_or(g->target_entry_range,gd->target_entry_range);
    if(!isfinite(g->target_entry_flight_path_angle)||g->target_entry_flight_path_angle>=0.0||
       g->target_entry_flight_path_angle<=-90.0) /* decision-literal: mathematical-numerical-requirement | descending flight-path angle lies strictly between minus ninety and zero degrees */
        g->target_entry_flight_path_angle=gd->target_entry_flight_path_angle;
    if(!isfinite(g->maximum_entry_flight_path_angle)||g->maximum_entry_flight_path_angle>=0.0||
       g->maximum_entry_flight_path_angle<=-90.0) /* decision-literal: mathematical-numerical-requirement | descending flight-path angle lies strictly between minus ninety and zero degrees */
        g->maximum_entry_flight_path_angle=gd->maximum_entry_flight_path_angle;
    if(g->maximum_entry_flight_path_angle>g->target_entry_flight_path_angle)
        g->maximum_entry_flight_path_angle=g->target_entry_flight_path_angle;

    g->target_post_burn_periapsis_altitude=finite_or(g->target_post_burn_periapsis_altitude,gd->target_post_burn_periapsis_altitude);
    g->deorbit_maximum_throttle=fraction_or(g->deorbit_maximum_throttle,gd->deorbit_maximum_throttle);
    g->deorbit_throttle_ramp_duration=nonnegative_or(g->deorbit_throttle_ramp_duration,gd->deorbit_throttle_ramp_duration);
    g->deorbit_timing_uncertainty=nonnegative_or(g->deorbit_timing_uncertainty,gd->deorbit_timing_uncertainty);
    g->deorbit_thrust_uncertainty_fraction=fraction_or(g->deorbit_thrust_uncertainty_fraction,gd->deorbit_thrust_uncertainty_fraction);
    g->deorbit_delta_v_uncertainty=nonnegative_or(g->deorbit_delta_v_uncertainty,gd->deorbit_delta_v_uncertainty);
    g->deorbit_mass_uncertainty_fraction=fraction_or(g->deorbit_mass_uncertainty_fraction,gd->deorbit_mass_uncertainty_fraction);
    g->deorbit_position_uncertainty=nonnegative_or(g->deorbit_position_uncertainty,gd->deorbit_position_uncertainty);
    g->deorbit_velocity_uncertainty=nonnegative_or(g->deorbit_velocity_uncertainty,gd->deorbit_velocity_uncertainty);
    g->deorbit_pointing_uncertainty=nonnegative_or(g->deorbit_pointing_uncertainty,gd->deorbit_pointing_uncertainty);
    g->deorbit_atmosphere_uncertainty_fraction=fraction_or(g->deorbit_atmosphere_uncertainty_fraction,gd->deorbit_atmosphere_uncertainty_fraction);
    g->deorbit_robust_minimum_pass_fraction=fraction_or(g->deorbit_robust_minimum_pass_fraction,gd->deorbit_robust_minimum_pass_fraction);

    g->taem_interface_altitude=positive_or(g->taem_interface_altitude,gd->taem_interface_altitude);
    g->taem_interface_range=positive_or(g->taem_interface_range,gd->taem_interface_range);
    g->taem_force_handoff_speed=positive_or(g->taem_force_handoff_speed,gd->taem_force_handoff_speed);
    g->taem_force_handoff_speed=fmax(g->taem_force_handoff_speed,v->minimum_safe_speed);
    g->mm304_handoff_radius=positive_or(g->mm304_handoff_radius,gd->mm304_handoff_radius);
    g->mm304_handoff_along_track=finite_or(g->mm304_handoff_along_track,gd->mm304_handoff_along_track);
    g->mm304_handoff_cross_track=finite_or(g->mm304_handoff_cross_track,gd->mm304_handoff_cross_track);
    g->mm304_perpendicular_heading_half_width=finite_or(
        g->mm304_perpendicular_heading_half_width,
        gd->mm304_perpendicular_heading_half_width);
    g->mm304_perpendicular_heading_half_width=clampd(
        g->mm304_perpendicular_heading_half_width,0.0,90.0); /* decision-literal: mathematical-numerical-requirement | perpendicular-sector half-width cannot exceed a right angle */
    g->mm305_min_altitude=positive_or(g->mm305_min_altitude,gd->mm305_min_altitude);
    g->mm305_max_altitude=positive_or(g->mm305_max_altitude,gd->mm305_max_altitude);
    if(g->mm305_max_altitude<g->mm305_min_altitude)
        g->mm305_max_altitude=g->mm305_min_altitude;
    g->mm305_target_mach=positive_or(g->mm305_target_mach,gd->mm305_target_mach);
    g->mm305_mach_half_width=positive_or(g->mm305_mach_half_width,gd->mm305_mach_half_width);
    g->hac_acquisition_altitude=positive_or(g->hac_acquisition_altitude,gd->hac_acquisition_altitude);
    g->hac_acquisition_altitude=fmin(g->hac_acquisition_altitude,g->taem_interface_altitude);
    g->hac_acquisition_mach=positive_or(g->hac_acquisition_mach,gd->hac_acquisition_mach);
    g->hac_radius=positive_or(g->hac_radius,gd->hac_radius);
    g->hac_look_ahead_angle=finite_or(g->hac_look_ahead_angle,gd->hac_look_ahead_angle);
    g->hac_look_ahead_angle=clampd(g->hac_look_ahead_angle,0.0,180.0); /* decision-literal: mathematical-numerical-requirement | unsigned lookahead over one planar half-turn spans zero through one hundred eighty degrees */
    g->final_approach_distance=positive_or(g->final_approach_distance,gd->final_approach_distance);
    g->final_alignment_speed=positive_or(g->final_alignment_speed,gd->final_alignment_speed);
    g->final_alignment_speed=fmax(g->final_alignment_speed,v->touchdown_speed);
    g->final_glide_slope=descent_angle_or(g->final_glide_slope,gd->final_glide_slope);
    g->taem_glide_slope=descent_angle_or(g->taem_glide_slope,gd->taem_glide_slope);
    g->gear_deployment_altitude=nonnegative_or(g->gear_deployment_altitude,gd->gear_deployment_altitude);
    g->flare_altitude=nonnegative_or(g->flare_altitude,gd->flare_altitude);
    if(!isfinite(g->touchdown_sink_rate)||!(g->touchdown_sink_rate<0.0))
        g->touchdown_sink_rate=gd->touchdown_sink_rate;
    g->guidance_rate=positive_or(g->guidance_rate,gd->guidance_rate);
    g->prediction_interval=positive_or(g->prediction_interval,gd->prediction_interval);
    g->s_turn_minimum_leg_duration=nonnegative_or(g->s_turn_minimum_leg_duration,gd->s_turn_minimum_leg_duration);
    g->entry_roll_rate=positive_or(g->entry_roll_rate,gd->entry_roll_rate);
    g->entry_roll_acceleration=positive_or(g->entry_roll_acceleration,gd->entry_roll_acceleration);
    g->taem_roll_rate=positive_or(g->taem_roll_rate,gd->taem_roll_rate);
    g->approach_roll_rate=positive_or(g->approach_roll_rate,gd->approach_roll_rate);
    g->minimum_planning_lead_time=nonnegative_or(g->minimum_planning_lead_time,gd->minimum_planning_lead_time);

    CalibrationSettings *a=&c->calibration;
    const CalibrationSettings *ad=&d.calibration;
    a->minimum_dynamic_pressure=nonnegative_or(a->minimum_dynamic_pressure,ad->minimum_dynamic_pressure);
    a->maximum_sample_dynamic_pressure=positive_or(a->maximum_sample_dynamic_pressure,ad->maximum_sample_dynamic_pressure);
    a->maximum_sample_dynamic_pressure=fmax(a->maximum_sample_dynamic_pressure,a->minimum_dynamic_pressure);
    a->maximum_sideslip=nonnegative_or(a->maximum_sideslip,ad->maximum_sideslip);
    a->minimum_calibration_radar_altitude=nonnegative_or(a->minimum_calibration_radar_altitude,ad->minimum_calibration_radar_altitude);
    a->maximum_calibration_dynamic_pressure=positive_or(a->maximum_calibration_dynamic_pressure,ad->maximum_calibration_dynamic_pressure);
    a->maximum_calibration_dynamic_pressure=fmax(a->maximum_calibration_dynamic_pressure,a->minimum_dynamic_pressure);
    a->maximum_calibration_g_load=positive_or(a->maximum_calibration_g_load,ad->maximum_calibration_g_load);
    a->maximum_calibration_sink_rate=positive_or(a->maximum_calibration_sink_rate,ad->maximum_calibration_sink_rate);
    a->abort_stall_fraction=fraction_or(a->abort_stall_fraction,ad->abort_stall_fraction);
    a->minimum_angle_of_attack=clampd(finite_or(a->minimum_angle_of_attack,ad->minimum_angle_of_attack),
        0.0,v->maximum_angle_of_attack);
    a->maximum_angle_of_attack=clampd(finite_or(a->maximum_angle_of_attack,ad->maximum_angle_of_attack),
        a->minimum_angle_of_attack,v->maximum_angle_of_attack);
    a->angle_of_attack_step=positive_or(a->angle_of_attack_step,ad->angle_of_attack_step);
    a->settling_duration=nonnegative_or(a->settling_duration,ad->settling_duration);
    a->sampling_duration=nonnegative_or(a->sampling_duration,ad->sampling_duration);
    a->trajectory_learning_rate=fraction_or(a->trajectory_learning_rate,ad->trajectory_learning_rate);
}

static int obj(const JsonDoc*d,int p,const char*k){return json_object_get(d,p,k);} static double num(const JsonDoc*d,int p,const char*k,double f){return json_number(d,obj(d,p,k),f);} static bool boo(const JsonDoc*d,int p,const char*k,bool f){return json_boolean(d,obj(d,p,k),f);} static void strv(const JsonDoc*d,int p,const char*k,char*out,size_t n){int i=obj(d,p,k);if(i>=0)json_string(d,i,out,n);}

bool landing_configuration_from_json(LandingConfiguration *c,const JsonDoc *d,int root){
    if(root<0||d->tokens[root].type!=JSON_OBJECT)return false;
    LandingConfiguration n=*c;
    int x=obj(d,root,"connection");if(x>=0){strv(d,x,"serialPort",n.connection.serial_port,sizeof(n.connection.serial_port));n.connection.baud_rate=(int)json_integer(d,obj(d,x,"baudRate"),n.connection.baud_rate);n.connection.timeout_ms=(int)json_integer(d,obj(d,x,"timeoutMs"),n.connection.timeout_ms);strv(d,x,"clientName",n.connection.client_name,sizeof(n.connection.client_name));int legacy=obj(d,x,"address");if(obj(d,x,"serialPort")<0&&legacy>=0){char candidate[256]={0};json_string(d,legacy,candidate,sizeof(candidate));if(candidate[0]=='/')copystr(n.connection.serial_port,sizeof(n.connection.serial_port),candidate);}}
    x=obj(d,root,"site");if(x>=0){strv(d,x,"name",n.site.name,sizeof(n.site.name));n.site.latitude=num(d,x,"latitude",n.site.latitude);n.site.longitude=num(d,x,"longitude",n.site.longitude);n.site.altitude=num(d,x,"altitude",n.site.altitude);n.site.runway_heading=num(d,x,"runwayHeading",n.site.runway_heading);n.site.runway_length=num(d,x,"runwayLength",n.site.runway_length);n.site.runway_width=num(d,x,"runwayWidth",n.site.runway_width);n.site.allow_reciprocal_runway=boo(d,x,"allowReciprocalRunway",n.site.allow_reciprocal_runway);}
    x=obj(d,root,"vehicle");if(x>=0){VehicleProfile*v=&n.vehicle;
        strv(d,x,"modelId",v->model_id,sizeof(v->model_id));
        v->estimated_lift_to_drag=num(d,x,"estimatedLiftToDrag",v->estimated_lift_to_drag);
        v->estimated_ballistic_coefficient=num(d,x,"estimatedBallisticCoefficient",v->estimated_ballistic_coefficient);
        v->entry_angle_of_attack=num(d,x,"entryAngleOfAttack",v->entry_angle_of_attack);
        v->maximum_angle_of_attack=num(d,x,"maximumAngleOfAttack",v->maximum_angle_of_attack);
        v->terminal_maximum_lift_angle_of_attack=num(d,x,"terminalMaximumLiftAngleOfAttack",v->terminal_maximum_lift_angle_of_attack);
        v->maximum_bank_angle=num(d,x,"maximumBankAngle",v->maximum_bank_angle);
        v->maximum_dynamic_pressure=num(d,x,"maximumDynamicPressure",v->maximum_dynamic_pressure);
        v->maximum_g_load=num(d,x,"maximumGLoad",v->maximum_g_load);
        v->minimum_safe_speed=num(d,x,"minimumSafeSpeed",v->minimum_safe_speed);
        v->final_approach_speed=num(d,x,"finalApproachSpeed",v->final_approach_speed);
        v->touchdown_speed=num(d,x,"touchdownSpeed",v->touchdown_speed);
        v->allow_powered_approach=boo(d,x,"allowPoweredApproach",v->allow_powered_approach);
        v->maximum_approach_throttle=num(d,x,"maximumApproachThrottle",v->maximum_approach_throttle);
        v->airbrake_action_group=(int)json_integer(d,obj(d,x,"airbrakeActionGroup"),v->airbrake_action_group);
    }
    x=obj(d,root,"guidance");if(x>=0){GuidanceSettings*g=&n.guidance;
#define G(F,K) g->F=num(d,x,K,g->F)
        G(target_deorbit_capture_radius,"targetDeorbitCaptureRadius");G(entry_interface_altitude_margin,"entryInterfaceAltitudeMargin");G(target_entry_range,"targetEntryRange");G(target_entry_flight_path_angle,"targetEntryFlightPathAngle");G(maximum_entry_flight_path_angle,"maximumEntryFlightPathAngle");G(target_post_burn_periapsis_altitude,"targetPostBurnPeriapsisAltitude");G(deorbit_maximum_throttle,"deorbitMaximumThrottle");G(deorbit_throttle_ramp_duration,"deorbitThrottleRampDuration");G(deorbit_timing_uncertainty,"deorbitTimingUncertainty");G(deorbit_thrust_uncertainty_fraction,"deorbitThrustUncertaintyFraction");G(deorbit_delta_v_uncertainty,"deorbitDeltaVUncertainty");G(deorbit_mass_uncertainty_fraction,"deorbitMassUncertaintyFraction");G(deorbit_position_uncertainty,"deorbitPositionUncertainty");G(deorbit_velocity_uncertainty,"deorbitVelocityUncertainty");G(deorbit_pointing_uncertainty,"deorbitPointingUncertainty");G(deorbit_atmosphere_uncertainty_fraction,"deorbitAtmosphereUncertaintyFraction");G(deorbit_robust_minimum_pass_fraction,"deorbitRobustMinimumPassFraction");G(taem_interface_altitude,"taemInterfaceAltitude");G(taem_interface_range,"taemInterfaceRange");G(taem_force_handoff_speed,"taemForceHandoffSpeed");G(mm304_handoff_radius,"mm304HandoffRadius");G(mm304_perpendicular_heading_half_width,"mm304PerpendicularHeadingHalfWidth");G(mm304_handoff_along_track,"mm304HandoffAlongTrack");G(mm304_handoff_cross_track,"mm304HandoffCrossTrack");G(mm305_min_altitude,"mm305MinimumAltitude");G(mm305_max_altitude,"mm305MaximumAltitude");G(mm305_target_mach,"mm305TargetMach");G(mm305_mach_half_width,"mm305MachHalfWidth");G(hac_acquisition_altitude,"hacAcquisitionAltitude");G(hac_acquisition_mach,"hacAcquisitionMach");G(hac_radius,"hacRadius");G(hac_look_ahead_angle,"hacLookAheadAngle");G(final_approach_distance,"finalApproachDistance");G(final_alignment_speed,"finalAlignmentSpeed");G(final_glide_slope,"finalGlideSlope");G(taem_glide_slope,"taemGlideSlope");G(gear_deployment_altitude,"gearDeploymentAltitude");G(flare_altitude,"flareAltitude");G(touchdown_sink_rate,"touchdownSinkRate");G(guidance_rate,"guidanceRate");G(prediction_interval,"predictionInterval");G(s_turn_minimum_leg_duration,"sTurnMinimumLegDuration");G(entry_roll_rate,"entryRollRate");G(entry_roll_acceleration,"entryRollAcceleration");G(taem_roll_rate,"taemRollRate");G(approach_roll_rate,"approachRollRate");G(minimum_planning_lead_time,"minimumPlanningLeadTime");g->use_time_warp=boo(d,x,"useTimeWarp",g->use_time_warp);
#undef G
    }
    x=obj(d,root,"calibration");if(x>=0){CalibrationSettings*a=&n.calibration;
        a->enable_passive_calibration=boo(d,x,"enablePassiveCalibration",a->enable_passive_calibration);a->enable_trajectory_calibration=boo(d,x,"enableTrajectoryCalibration",a->enable_trajectory_calibration);a->auto_apply_in_flight=boo(d,x,"autoApplyInFlight",a->auto_apply_in_flight);
#define A(F,K) a->F=num(d,x,K,a->F)
        A(minimum_dynamic_pressure,"minimumDynamicPressure");A(maximum_sample_dynamic_pressure,"maximumSampleDynamicPressure");A(maximum_sideslip,"maximumSideslip");A(minimum_calibration_radar_altitude,"minimumCalibrationRadarAltitude");A(maximum_calibration_dynamic_pressure,"maximumCalibrationDynamicPressure");A(maximum_calibration_g_load,"maximumCalibrationGLoad");A(maximum_calibration_sink_rate,"maximumCalibrationSinkRate");A(abort_stall_fraction,"abortStallFraction");A(minimum_angle_of_attack,"minimumAngleOfAttack");A(maximum_angle_of_attack,"maximumAngleOfAttack");A(angle_of_attack_step,"angleOfAttackStep");A(settling_duration,"settlingDuration");A(sampling_duration,"samplingDuration");A(trajectory_learning_rate,"trajectoryLearningRate");
#undef A
    }
    landing_configuration_normalize(&n);*c=n;return true;
}

#define KEY(w,k) do{jw_string(w,k);jw_char(w,':');}while(0)
#define COMMA(w) jw_char(w,',')
static void model_vec3_json(JsonWriter*w,Vector3 v){jw_char(w,'[');jw_number(w,v.x);COMMA(w);jw_number(w,v.y);COMMA(w);jw_number(w,v.z);jw_char(w,']');}

void planet_model_replay_json(JsonWriter*w,const PlanetModel*p){
    if(!w||!p){if(w)jw_null(w);return;}
    jw_char(w,'{');KEY(w,"name");jw_string(w,p->name);COMMA(w);
    KEY(w,"radius");jw_number(w,p->radius);COMMA(w);KEY(w,"gravitationalParameter");jw_number(w,p->gravitational_parameter);COMMA(w);
    KEY(w,"rotationalSpeed");jw_number(w,p->rotational_speed);COMMA(w);KEY(w,"atmosphereDepth");jw_number(w,p->atmosphere_depth);COMMA(w);
    KEY(w,"surfaceDensity");jw_number(w,p->surface_density);COMMA(w);KEY(w,"atmosphereAdiabaticIndex");jw_number(w,p->atmosphere_adiabatic_index);COMMA(w);
    KEY(w,"northAxis");model_vec3_json(w,p->north_axis);COMMA(w);KEY(w,"primeMeridianAtEpoch");model_vec3_json(w,p->prime_meridian_at_epoch);COMMA(w);
    KEY(w,"epochUT");jw_number(w,p->epoch_ut);COMMA(w);KEY(w,"atmosphereSamples");jw_char(w,'[');
    size_t count=p->atmosphere_sample_count;if(count>LANDER_ATMOSPHERE_SAMPLE_MAX)count=LANDER_ATMOSPHERE_SAMPLE_MAX;
    for(size_t i=0;i<count;i++){if(i)COMMA(w);jw_char(w,'{');KEY(w,"altitude");jw_number(w,p->atmosphere_altitude[i]);COMMA(w);KEY(w,"pressure");jw_number(w,p->atmosphere_pressure[i]);COMMA(w);KEY(w,"density");jw_number(w,p->atmosphere_density[i]);jw_char(w,'}');}
    jw_char(w,']');jw_char(w,'}');
}

void landing_configuration_json(JsonWriter *w,const LandingConfiguration *c){
    jw_char(w,'{');KEY(w,"connection");jw_char(w,'{');KEY(w,"serialPort");jw_string(w,c->connection.serial_port);COMMA(w);KEY(w,"baudRate");jw_integer(w,c->connection.baud_rate);COMMA(w);KEY(w,"timeoutMs");jw_integer(w,c->connection.timeout_ms);COMMA(w);KEY(w,"clientName");jw_string(w,c->connection.client_name);jw_char(w,'}');COMMA(w);
    KEY(w,"site");jw_char(w,'{');KEY(w,"name");jw_string(w,c->site.name);COMMA(w);KEY(w,"latitude");jw_number(w,c->site.latitude);COMMA(w);KEY(w,"longitude");jw_number(w,c->site.longitude);COMMA(w);KEY(w,"altitude");jw_number(w,c->site.altitude);COMMA(w);KEY(w,"runwayHeading");jw_number(w,c->site.runway_heading);COMMA(w);KEY(w,"runwayLength");jw_number(w,c->site.runway_length);COMMA(w);KEY(w,"runwayWidth");jw_number(w,c->site.runway_width);COMMA(w);KEY(w,"allowReciprocalRunway");jw_bool(w,c->site.allow_reciprocal_runway);jw_char(w,'}');COMMA(w);
    const VehicleProfile*v=&c->vehicle;KEY(w,"vehicle");jw_char(w,'{');KEY(w,"modelId");jw_string(w,v->model_id);COMMA(w);KEY(w,"estimatedLiftToDrag");jw_number(w,v->estimated_lift_to_drag);COMMA(w);KEY(w,"estimatedBallisticCoefficient");jw_number(w,v->estimated_ballistic_coefficient);COMMA(w);KEY(w,"entryAngleOfAttack");jw_number(w,v->entry_angle_of_attack);COMMA(w);KEY(w,"maximumAngleOfAttack");jw_number(w,v->maximum_angle_of_attack);COMMA(w);KEY(w,"terminalMaximumLiftAngleOfAttack");jw_number(w,v->terminal_maximum_lift_angle_of_attack);COMMA(w);KEY(w,"maximumBankAngle");jw_number(w,v->maximum_bank_angle);COMMA(w);KEY(w,"maximumDynamicPressure");jw_number(w,v->maximum_dynamic_pressure);COMMA(w);KEY(w,"maximumGLoad");jw_number(w,v->maximum_g_load);COMMA(w);KEY(w,"minimumSafeSpeed");jw_number(w,v->minimum_safe_speed);COMMA(w);KEY(w,"finalApproachSpeed");jw_number(w,v->final_approach_speed);COMMA(w);KEY(w,"touchdownSpeed");jw_number(w,v->touchdown_speed);COMMA(w);KEY(w,"allowPoweredApproach");jw_bool(w,v->allow_powered_approach);COMMA(w);KEY(w,"maximumApproachThrottle");jw_number(w,v->maximum_approach_throttle);COMMA(w);KEY(w,"airbrakeActionGroup");jw_integer(w,v->airbrake_action_group);jw_char(w,'}');COMMA(w);
    const GuidanceSettings*g=&c->guidance;KEY(w,"guidance");jw_char(w,'{');
#define WGF(K,F) KEY(w,K);jw_number(w,g->F);COMMA(w)
    WGF("targetDeorbitCaptureRadius",target_deorbit_capture_radius);WGF("entryInterfaceAltitudeMargin",entry_interface_altitude_margin);WGF("targetEntryRange",target_entry_range);WGF("targetEntryFlightPathAngle",target_entry_flight_path_angle);WGF("maximumEntryFlightPathAngle",maximum_entry_flight_path_angle);WGF("targetPostBurnPeriapsisAltitude",target_post_burn_periapsis_altitude);WGF("deorbitMaximumThrottle",deorbit_maximum_throttle);WGF("deorbitThrottleRampDuration",deorbit_throttle_ramp_duration);WGF("deorbitTimingUncertainty",deorbit_timing_uncertainty);WGF("deorbitThrustUncertaintyFraction",deorbit_thrust_uncertainty_fraction);WGF("deorbitDeltaVUncertainty",deorbit_delta_v_uncertainty);WGF("deorbitMassUncertaintyFraction",deorbit_mass_uncertainty_fraction);WGF("deorbitPositionUncertainty",deorbit_position_uncertainty);WGF("deorbitVelocityUncertainty",deorbit_velocity_uncertainty);WGF("deorbitPointingUncertainty",deorbit_pointing_uncertainty);WGF("deorbitAtmosphereUncertaintyFraction",deorbit_atmosphere_uncertainty_fraction);WGF("deorbitRobustMinimumPassFraction",deorbit_robust_minimum_pass_fraction);WGF("taemInterfaceAltitude",taem_interface_altitude);WGF("taemInterfaceRange",taem_interface_range);WGF("taemForceHandoffSpeed",taem_force_handoff_speed);WGF("mm304HandoffRadius",mm304_handoff_radius);WGF("mm304PerpendicularHeadingHalfWidth",mm304_perpendicular_heading_half_width);WGF("mm304HandoffAlongTrack",mm304_handoff_along_track);WGF("mm304HandoffCrossTrack",mm304_handoff_cross_track);WGF("mm305MinimumAltitude",mm305_min_altitude);WGF("mm305MaximumAltitude",mm305_max_altitude);WGF("mm305TargetMach",mm305_target_mach);WGF("mm305MachHalfWidth",mm305_mach_half_width);WGF("hacAcquisitionAltitude",hac_acquisition_altitude);WGF("hacAcquisitionMach",hac_acquisition_mach);WGF("hacRadius",hac_radius);WGF("hacLookAheadAngle",hac_look_ahead_angle);WGF("finalApproachDistance",final_approach_distance);WGF("finalAlignmentSpeed",final_alignment_speed);WGF("finalGlideSlope",final_glide_slope);WGF("taemGlideSlope",taem_glide_slope);WGF("gearDeploymentAltitude",gear_deployment_altitude);WGF("flareAltitude",flare_altitude);WGF("touchdownSinkRate",touchdown_sink_rate);WGF("guidanceRate",guidance_rate);WGF("predictionInterval",prediction_interval);WGF("sTurnMinimumLegDuration",s_turn_minimum_leg_duration);WGF("entryRollRate",entry_roll_rate);WGF("entryRollAcceleration",entry_roll_acceleration);WGF("taemRollRate",taem_roll_rate);WGF("approachRollRate",approach_roll_rate);KEY(w,"useTimeWarp");jw_bool(w,g->use_time_warp);COMMA(w);KEY(w,"minimumPlanningLeadTime");jw_number(w,g->minimum_planning_lead_time);jw_char(w,'}');COMMA(w);
#undef WGF
    const CalibrationSettings*a=&c->calibration;KEY(w,"calibration");jw_char(w,'{');KEY(w,"enablePassiveCalibration");jw_bool(w,a->enable_passive_calibration);COMMA(w);KEY(w,"enableTrajectoryCalibration");jw_bool(w,a->enable_trajectory_calibration);COMMA(w);KEY(w,"autoApplyInFlight");jw_bool(w,a->auto_apply_in_flight);COMMA(w);
#define WAF(K,F) KEY(w,K);jw_number(w,a->F);COMMA(w)
    WAF("minimumDynamicPressure",minimum_dynamic_pressure);WAF("maximumSampleDynamicPressure",maximum_sample_dynamic_pressure);WAF("maximumSideslip",maximum_sideslip);WAF("minimumCalibrationRadarAltitude",minimum_calibration_radar_altitude);WAF("maximumCalibrationDynamicPressure",maximum_calibration_dynamic_pressure);WAF("maximumCalibrationGLoad",maximum_calibration_g_load);WAF("maximumCalibrationSinkRate",maximum_calibration_sink_rate);WAF("abortStallFraction",abort_stall_fraction);WAF("minimumAngleOfAttack",minimum_angle_of_attack);WAF("maximumAngleOfAttack",maximum_angle_of_attack);WAF("angleOfAttackStep",angle_of_attack_step);WAF("settlingDuration",settling_duration);WAF("samplingDuration",sampling_duration);KEY(w,"trajectoryLearningRate");jw_number(w,a->trajectory_learning_rate);jw_char(w,'}');jw_char(w,'}');
#undef WAF
}

void telemetry_init(Telemetry*t){
    memset(t,0,sizeof(*t));
    copystr(t->vessel_name,sizeof(t->vessel_name),"—");
    /* Zero means unavailable. The live and simulator telemetry readers either
       provide measured speed of sound or derive it from PlanetModel. Seeding
       this structure with a plausible atmosphere value would turn missing
       telemetry into invented physics. */
    t->speed_of_sound=0.0;
    t->navball_speed_mode=SPEED_UNCHANGED;
    t->trajectory_density_scale=t->trajectory_drag_scale=t->trajectory_lift_scale=
        t->bank_effectiveness=1.0;
    copystr(t->vessel_situation,sizeof(t->vessel_situation),"Unknown");
}
void guidance_command_init(GuidanceCommand*c){memset(c,0,sizeof(*c));c->navball_speed_mode=SPEED_UNCHANGED;c->control_profile=PROFILE_ORBITAL;}
void calibration_snapshot_init(CalibrationSnapshot*c){memset(c,0,sizeof(*c));c->state=CAL_IDLE;c->confidence=.2;c->planning_confidence=.18;copystr(c->status,sizeof(c->status),"Passive calibration is ready.");}
void landing_snapshot_init(LandingSnapshot*s,const VehicleProfile*p){memset(s,0,sizeof(*s));s->connection_status=CONN_DISCONNECTED;s->phase=PHASE_IDLE;telemetry_init(&s->telemetry);guidance_command_init(&s->command);trajectory_init(&s->actual_trajectory);trajectory_init(&s->reference_trajectory);trajectory_init(&s->predicted_trajectory);copystr(s->status_message,sizeof(s->status_message),"Not connected");calibration_snapshot_init(&s->calibration);if(p)s->adaptive_vehicle_profile=*p;}

void trajectory_init(Trajectory*t){memset(t,0,sizeof(*t));}void trajectory_clear(Trajectory*t){free(t->points);memset(t,0,sizeof(*t));}
bool trajectory_append(Trajectory*t,TrajectoryPoint p){if(t->count==t->capacity){size_t n=t->capacity?t->capacity*2:128;TrajectoryPoint*q=realloc(t->points,n*sizeof(*q));if(!q)return false;t->points=q;t->capacity=n;}t->points[t->count++]=p;return true;}
bool trajectory_copy(Trajectory*d,const Trajectory*s){trajectory_clear(d);if(!s->count)return true;d->points=malloc(s->count*sizeof(*d->points));if(!d->points)return false;memcpy(d->points,s->points,s->count*sizeof(*d->points));d->count=d->capacity=s->count;return true;}
void deorbit_plan_clear(DeorbitPlan*p){if(!p)return;trajectory_clear(&p->trajectory);memset(p,0,sizeof(*p));}

static void nullable_number(JsonWriter*w,bool has,double v){if(has)jw_number(w,v);else jw_null(w);} static void vec3_json(JsonWriter*w,const double a[3]){jw_char(w,'[');jw_number(w,a[0]);COMMA(w);jw_number(w,a[1]);COMMA(w);jw_number(w,a[2]);jw_char(w,']');}static void vec4_json(JsonWriter*w,const double a[4]){jw_char(w,'[');jw_number(w,a[0]);COMMA(w);jw_number(w,a[1]);COMMA(w);jw_number(w,a[2]);COMMA(w);jw_number(w,a[3]);jw_char(w,']');}
void trajectory_json(JsonWriter*w,const Trajectory*t){jw_char(w,'[');for(size_t i=0;i<t->count;i++){if(i)COMMA(w);const TrajectoryPoint*p=&t->points[i];jw_char(w,'{');KEY(w,"id");char id[40];snprintf(id,sizeof(id),"c-%zu-%.0f",i,p->ut*1000);jw_string(w,id);COMMA(w);KEY(w,"ut");jw_number(w,p->ut);COMMA(w);KEY(w,"latitude");jw_number(w,p->latitude);COMMA(w);KEY(w,"longitude");jw_number(w,p->longitude);COMMA(w);KEY(w,"altitude");jw_number(w,p->altitude);COMMA(w);KEY(w,"speed");jw_number(w,p->speed);COMMA(w);KEY(w,"phase");jw_string(w,phase_string(p->phase));COMMA(w);KEY(w,"kind");jw_string(w,p->kind==TRAJ_ACTUAL?"actual":p->kind==TRAJ_REFERENCE?"reference":"planned");jw_char(w,'}');}jw_char(w,']');}

static void vehicle_json(JsonWriter*w,const VehicleProfile*v){jw_char(w,'{');KEY(w,"modelId");jw_string(w,v->model_id);COMMA(w);KEY(w,"estimatedLiftToDrag");jw_number(w,v->estimated_lift_to_drag);COMMA(w);KEY(w,"estimatedBallisticCoefficient");jw_number(w,v->estimated_ballistic_coefficient);COMMA(w);KEY(w,"entryAngleOfAttack");jw_number(w,v->entry_angle_of_attack);COMMA(w);KEY(w,"maximumAngleOfAttack");jw_number(w,v->maximum_angle_of_attack);COMMA(w);KEY(w,"terminalMaximumLiftAngleOfAttack");jw_number(w,v->terminal_maximum_lift_angle_of_attack);COMMA(w);KEY(w,"maximumBankAngle");jw_number(w,v->maximum_bank_angle);COMMA(w);KEY(w,"maximumDynamicPressure");jw_number(w,v->maximum_dynamic_pressure);COMMA(w);KEY(w,"maximumGLoad");jw_number(w,v->maximum_g_load);COMMA(w);KEY(w,"minimumSafeSpeed");jw_number(w,v->minimum_safe_speed);COMMA(w);KEY(w,"finalApproachSpeed");jw_number(w,v->final_approach_speed);COMMA(w);KEY(w,"touchdownSpeed");jw_number(w,v->touchdown_speed);COMMA(w);KEY(w,"allowPoweredApproach");jw_bool(w,v->allow_powered_approach);COMMA(w);KEY(w,"maximumApproachThrottle");jw_number(w,v->maximum_approach_throttle);COMMA(w);KEY(w,"airbrakeActionGroup");jw_integer(w,v->airbrake_action_group);jw_char(w,'}');}

static void command_json(JsonWriter*w,const GuidanceCommand*c){jw_char(w,'{');KEY(w,"targetPitch");jw_number(w,c->target_pitch);COMMA(w);KEY(w,"targetAoA");nullable_number(w,c->has_target_aoa,c->target_aoa);COMMA(w);KEY(w,"targetHeading");jw_number(w,c->target_heading);COMMA(w);KEY(w,"targetRoll");jw_number(w,c->target_roll);COMMA(w);KEY(w,"targetThrottle");jw_number(w,c->target_throttle);COMMA(w);KEY(w,"wheelSteering");jw_number(w,c->wheel_steering);COMMA(w);KEY(w,"gear");jw_bool(w,c->gear);COMMA(w);KEY(w,"brakes");jw_bool(w,c->brakes);COMMA(w);KEY(w,"airbrakes");jw_bool(w,c->airbrakes);COMMA(w);KEY(w,"useInertialDirection");jw_bool(w,c->use_inertial_direction);COMMA(w);KEY(w,"inertialDirection");jw_char(w,'{');KEY(w,"x");jw_number(w,c->inertial_direction.x);COMMA(w);KEY(w,"y");jw_number(w,c->inertial_direction.y);COMMA(w);KEY(w,"z");jw_number(w,c->inertial_direction.z);jw_char(w,'}');COMMA(w);KEY(w,"autopilotEngaged");jw_bool(w,c->autopilot_engaged);COMMA(w);KEY(w,"headingControlEnabled");jw_bool(w,c->heading_control_enabled);COMMA(w);KEY(w,"hacControlTuning");jw_bool(w,c->hac_control_tuning);COMMA(w);KEY(w,"terminalPitchTuning");jw_bool(w,c->terminal_pitch_tuning);COMMA(w);KEY(w,"navballSpeedMode");jw_string(w,speed_mode_string(c->navball_speed_mode));COMMA(w);KEY(w,"controlProfile");jw_string(w,profile_string(c->control_profile));jw_char(w,'}');}

static void physics_vector_json(JsonWriter *w,bool valid,Vector3 v){
    if(!valid){jw_null(w);return;}
    jw_char(w,'[');jw_number(w,v.x);COMMA(w);jw_number(w,v.y);COMMA(w);jw_number(w,v.z);jw_char(w,']');
}
static void telemetry_json(JsonWriter*w,const Telemetry*t){jw_char(w,'{');
#define TN(K,F) KEY(w,K);jw_number(w,t->F);COMMA(w)
    KEY(w,"physicsReferenceFrame");jw_string(w,"body-non-rotating-canonical-xzy");COMMA(w);
    KEY(w,"physicsSampleValid");jw_bool(w,t->physics_sample_valid);COMMA(w);
    KEY(w,"liftVector");physics_vector_json(w,t->has_force_vectors,t->lift_vector);COMMA(w);
    KEY(w,"dragVector");physics_vector_json(w,t->has_force_vectors,t->drag_vector);COMMA(w);
    KEY(w,"centerOfMassRootReferenceFrame");jw_string(w,"root-part-krpc");COMMA(w);
    KEY(w,"centerOfMassRoot");physics_vector_json(w,t->has_center_of_mass_root,t->center_of_mass_root);COMMA(w);
    KEY(w,"centerOfMass");physics_vector_json(w,t->has_center_of_mass,t->center_of_mass);COMMA(w);
    TN("predictedPhysicsObservedSeconds",predicted_physics_observed_seconds);TN("predictedPhysicsFallbackSeconds",predicted_physics_fallback_seconds);KEY(w,"predictedShadowGuidance");jw_bool(w,t->predicted_shadow_guidance);COMMA(w);KEY(w,"predictedTerminalPolicyFeasible");jw_bool(w,t->predicted_terminal_policy_feasible);COMMA(w);TN("predictedTerminalSurvivabilityStressScore",predicted_terminal_survivability_stress_score);TN("predictedPhysicsRelativeUncertainty",predicted_physics_relative_uncertainty);TN("predictedUncertaintyScenarios",predicted_uncertainty_scenarios);TN("predictedRawPublishedPositionDelta30s",predicted_raw_published_position_delta_30s);TN("predictedRawPublishedPositionDelta60s",predicted_raw_published_position_delta_60s);TN("predictedRawPublishedPositionDelta120s",predicted_raw_published_position_delta_120s);TN("dryMass",dry_mass);TN("physicsConfidence",physics_confidence);TN("physicsMassFlow",physics_mass_flow);TN("physicsModelResidual",physics_model_residual);TN("physicsModelResidualConfidence",physics_model_residual_confidence);TN("physicsCertifiedUncertainty",physics_certified_uncertainty);KEY(w,"physicsForceResidualPerQ");physics_vector_json(w,true,t->physics_force_residual_per_q);COMMA(w);KEY(w,"physicsForceResidualSigmaPerQ");physics_vector_json(w,true,t->physics_force_residual_sigma_per_q);COMMA(w);TN("physicsForceResidualConfidence",physics_force_residual_confidence);KEY(w,"physicsAirbrakeModelAvailable");jw_bool(w,t->physics_airbrake_model_available);COMMA(w);TN("physicsAirbrakeModelConfidence",physics_airbrake_model_confidence);TN("physicsSamples",physics_samples);TN("physicsLiveSamples",physics_live_samples);
    TN("physicsAirbrakeDragAccel",physics_airbrake_drag_accel);
    KEY(w,"physicsAngularAuthority");jw_char(w,'[');for(int i=0;i<3;i++){if(i)COMMA(w);jw_number(w,t->physics_authority[i]);}jw_char(w,']');COMMA(w);
    KEY(w,"physicsAuthorityConfidence");jw_char(w,'[');for(int i=0;i<3;i++){if(i)COMMA(w);jw_number(w,t->physics_authority_confidence[i]);}jw_char(w,']');COMMA(w);
    TN("ut",ut);KEY(w,"vesselName");jw_string(w,t->vessel_name);COMMA(w);TN("latitude",latitude);TN("longitude",longitude);TN("meanAltitude",mean_altitude);TN("radarAltitude",radar_altitude);TN("verticalSpeed",vertical_speed);TN("horizontalSpeed",horizontal_speed);TN("surfaceSpeed",surface_speed);TN("trueAirSpeed",true_air_speed);TN("atmosphericDensity",atmospheric_density);TN("speedOfSound",speed_of_sound);TN("mach",mach);TN("heading",heading);TN("groundTrackHeading",ground_track_heading);TN("courseToSiteError",course_to_site_error);TN("pitch",pitch);TN("roll",roll);TN("pitchRate",pitch_rate);TN("rollRate",roll_rate);TN("headingRate",heading_rate);
    KEY(w,"angleOfAttackRate");nullable_number(w,t->has_angle_of_attack_rate,t->angle_of_attack_rate);COMMA(w);
    KEY(w,"bodyPitchRate");nullable_number(w,t->has_body_pitch_rate,t->body_pitch_rate);COMMA(w);KEY(w,"bodyRollRate");nullable_number(w,t->has_body_roll_rate,t->body_roll_rate);COMMA(w);KEY(w,"bodyYawRate");nullable_number(w,t->has_body_yaw_rate,t->body_yaw_rate);COMMA(w);KEY(w,"courseRate");nullable_number(w,t->has_course_rate,t->course_rate);COMMA(w);TN("autopilotError",autopilot_error);KEY(w,"commandPitchError");nullable_number(w,t->has_command_pitch_error,t->command_pitch_error);COMMA(w);KEY(w,"commandRollError");nullable_number(w,t->has_command_roll_error,t->command_roll_error);COMMA(w);KEY(w,"commandHeadingError");nullable_number(w,t->has_command_heading_error,t->command_heading_error);COMMA(w);
    KEY(w,"attitudeQuaternion");if(t->has_attitude_quaternion)vec4_json(w,t->attitude_quaternion);else jw_null(w);COMMA(w);KEY(w,"attitudeReferenceFrame");if(t->attitude_reference_frame[0])jw_string(w,t->attitude_reference_frame);else jw_null(w);COMMA(w);
    KEY(w,"autopilotPitchPIDGains");if(t->has_pid_gains)vec3_json(w,t->autopilot_pitch_pid_gains);else jw_null(w);COMMA(w);KEY(w,"autopilotRollPIDGains");if(t->has_pid_gains)vec3_json(w,t->autopilot_roll_pid_gains);else jw_null(w);COMMA(w);KEY(w,"autopilotYawPIDGains");if(t->has_pid_gains)vec3_json(w,t->autopilot_yaw_pid_gains);else jw_null(w);COMMA(w);KEY(w,"navballSpeedMode");jw_string(w,speed_mode_string(t->navball_speed_mode));COMMA(w);
    TN("angleOfAttack",angle_of_attack);TN("sideslip",sideslip);TN("dynamicPressure",dynamic_pressure);TN("staticPressure",static_pressure);TN("gForce",g_force);TN("stallFraction",stall_fraction);TN("liftForce",lift_force);TN("dragForce",drag_force);TN("mass",mass);TN("availableThrust",available_thrust);TN("currentThrust",current_thrust);TN("apoapsisAltitude",apoapsis_altitude);TN("periapsisAltitude",periapsis_altitude);TN("orbitPeriod",orbit_period);TN("throttle",throttle);
    KEY(w,"controlPitch");nullable_number(w,t->has_controls,t->control_pitch);COMMA(w);KEY(w,"controlRoll");nullable_number(w,t->has_controls,t->control_roll);COMMA(w);KEY(w,"controlYaw");nullable_number(w,t->has_controls,t->control_yaw);COMMA(w);KEY(w,"availablePitchTorque");nullable_number(w,t->has_torque,t->available_pitch_torque);COMMA(w);KEY(w,"availableRollTorque");nullable_number(w,t->has_torque,t->available_roll_torque);COMMA(w);KEY(w,"availableYawTorque");nullable_number(w,t->has_torque,t->available_yaw_torque);COMMA(w);KEY(w,"pitchMomentOfInertia");nullable_number(w,t->has_inertia,t->pitch_moment_of_inertia);COMMA(w);KEY(w,"rollMomentOfInertia");nullable_number(w,t->has_inertia,t->roll_moment_of_inertia);COMMA(w);KEY(w,"yawMomentOfInertia");nullable_number(w,t->has_inertia,t->yaw_moment_of_inertia);COMMA(w);
    KEY(w,"loopWallDeltaMilliseconds");nullable_number(w,t->has_loop_wall_delta,t->loop_wall_delta_ms);COMMA(w);KEY(w,"telemetryLatencyMilliseconds");nullable_number(w,t->has_telemetry_latency,t->telemetry_latency_ms);COMMA(w);KEY(w,"guidanceComputeMilliseconds");nullable_number(w,t->has_guidance_compute,t->guidance_compute_ms);COMMA(w);KEY(w,"applyLatencyMilliseconds");nullable_number(w,t->has_apply_latency,t->apply_latency_ms);COMMA(w);KEY(w,"controlLoopMilliseconds");nullable_number(w,t->has_control_loop,t->control_loop_ms);COMMA(w);KEY(w,"rpcReadCalls");if(t->has_rpc_budget)jw_integer(w,t->rpc_read_calls);else jw_null(w);COMMA(w);KEY(w,"rpcReadWireRequests");if(t->has_rpc_budget)jw_integer(w,t->rpc_read_wire_requests);else jw_null(w);COMMA(w);KEY(w,"rpcApplyCalls");if(t->has_rpc_budget)jw_integer(w,t->rpc_apply_calls);else jw_null(w);COMMA(w);KEY(w,"rpcApplyWireRequests");if(t->has_rpc_budget)jw_integer(w,t->rpc_apply_wire_requests);else jw_null(w);COMMA(w);KEY(w,"rpcTotalCalls");if(t->has_rpc_budget)jw_integer(w,t->rpc_total_calls);else jw_null(w);COMMA(w);KEY(w,"rpcTotalWireRequests");if(t->has_rpc_budget)jw_integer(w,t->rpc_total_wire_requests);else jw_null(w);COMMA(w);
    KEY(w,"gear");jw_bool(w,t->gear);COMMA(w);KEY(w,"brakes");jw_bool(w,t->brakes);COMMA(w);KEY(w,"airbrakes");if(t->has_airbrakes)jw_bool(w,t->airbrakes);else jw_null(w);COMMA(w);KEY(w,"vesselSituation");jw_string(w,t->vessel_situation);COMMA(w);
    TN("rangeToSite",range_to_site);TN("bearingToSite",bearing_to_site);TN("headingError",heading_error);TN("runwayAlongTrack",runway_along_track);TN("runwayCrossTrack",runway_cross_track);TN("flightPathAngle",flight_path_angle);TN("estimatedLiftToDrag",estimated_lift_to_drag);TN("estimatedBallisticCoefficient",estimated_ballistic_coefficient);TN("aerodynamicConfidence",aerodynamic_confidence);TN("calibratedBestGlideAngleOfAttack",calibrated_best_glide_angle_of_attack);TN("calibratedStallSpeed",calibrated_stall_speed);TN("predictedMissDistance",predicted_miss_distance);KEY(w,"predictedTAEMDistance");if(isfinite(t->predicted_taem_distance))jw_number(w,t->predicted_taem_distance);else jw_null(w);COMMA(w);KEY(w,"predictedTAEMRangeError");if(isfinite(t->predicted_taem_range_error))jw_number(w,t->predicted_taem_range_error);else jw_null(w);COMMA(w);TN("predictedEntryRange",predicted_entry_range);TN("predictedEntryFlightPathAngle",predicted_entry_flight_path_angle);KEY(w,"predictedTAEMSpeed");if(isfinite(t->predicted_taem_speed))jw_number(w,t->predicted_taem_speed);else jw_null(w);COMMA(w);KEY(w,"predictedTAEMEnergyError");if(isfinite(t->predicted_taem_energy_error))jw_number(w,t->predicted_taem_energy_error);else jw_null(w);COMMA(w);TN("predictedPeakDynamicPressure",predicted_peak_dynamic_pressure);TN("predictedPeakGLoad",predicted_peak_g_load);TN("predictedSTurnReversals",predicted_s_turn_reversals);TN("trajectoryDensityScale",trajectory_density_scale);TN("trajectoryDragScale",trajectory_drag_scale);TN("trajectoryLiftScale",trajectory_lift_scale);TN("bankEffectiveness",bank_effectiveness);TN("trajectoryCalibrationConfidence",trajectory_calibration_confidence);TN("trajectoryAltitudeResidual",trajectory_altitude_residual);TN("trajectorySpeedResidual",trajectory_speed_residual);TN("trajectoryRangeResidual",trajectory_range_residual);KEY(w,"energyExcessRange");jw_number(w,t->energy_excess_range);jw_char(w,'}');
#undef TN
}

static void plan_json(JsonWriter*w,const DeorbitPlan*p){if(!p){jw_null(w);return;}jw_char(w,'{');KEY(w,"createdUT");jw_number(w,p->created_ut);COMMA(w);KEY(w,"burnUT");jw_number(w,p->burn_ut);COMMA(w);KEY(w,"deltaV");jw_number(w,p->delta_v);COMMA(w);KEY(w,"estimatedBurnDuration");jw_number(w,p->estimated_burn_duration);COMMA(w);KEY(w,"predictedTAEMDistance");jw_number(w,p->predicted_taem_distance);COMMA(w);KEY(w,"predictedTAEMRangeError");jw_number(w,p->predicted_taem_range_error);COMMA(w);KEY(w,"predictedClosestDistance");jw_number(w,p->predicted_closest_distance);COMMA(w);KEY(w,"predictedEntryRange");jw_number(w,p->predicted_entry_range);COMMA(w);KEY(w,"predictedEntryFlightPathAngle");jw_number(w,p->predicted_entry_flight_path_angle);COMMA(w);KEY(w,"predictedPostBurnPeriapsisAltitude");jw_number(w,p->predicted_post_burn_periapsis_altitude);COMMA(w);KEY(w,"nominalCaptureAchieved");jw_bool(w,p->nominal_capture_achieved);COMMA(w);KEY(w,"robustnessQualified");jw_bool(w,p->robustness_qualified);COMMA(w);KEY(w,"robustnessScenarios");jw_integer(w,p->robustness_scenarios);COMMA(w);KEY(w,"robustnessPassed");jw_integer(w,p->robustness_passed);COMMA(w);KEY(w,"robustnessUnsafe");jw_integer(w,p->robustness_unsafe);COMMA(w);KEY(w,"robustnessPassFraction");jw_number(w,p->robustness_pass_fraction);COMMA(w);KEY(w,"recoveryPassed");jw_integer(w,p->recovery_passed);COMMA(w);KEY(w,"recoveryPassFraction");jw_number(w,p->recovery_pass_fraction);COMMA(w);KEY(w,"executionQualified");jw_bool(w,p->execution_qualified);COMMA(w);KEY(w,"executionDegraded");jw_bool(w,p->execution_degraded);COMMA(w);KEY(w,"worstCaseClosestDistance");jw_number(w,p->worst_case_closest_distance);COMMA(w);KEY(w,"worstCaseTAEMRangeError");jw_number(w,p->worst_case_taem_range_error);COMMA(w);KEY(w,"worstCaseEntryFlightPathAngle");jw_number(w,p->worst_case_entry_flight_path_angle);COMMA(w);KEY(w,"worstCasePostBurnPeriapsisAltitude");jw_number(w,p->worst_case_post_burn_periapsis_altitude);COMMA(w);KEY(w,"worstCasePeakDynamicPressure");jw_number(w,p->worst_case_peak_dynamic_pressure);COMMA(w);KEY(w,"worstCasePeakGLoad");jw_number(w,p->worst_case_peak_g_load);COMMA(w);KEY(w,"liveCutoffCaptureQualified");jw_bool(w,p->live_cutoff_capture_qualified);COMMA(w);KEY(w,"liveCutoffPeriapsisAltitude");jw_number(w,p->live_cutoff_periapsis_altitude);COMMA(w);KEY(w,"liveCutoffClosestDistance");jw_number(w,p->live_cutoff_closest_distance);COMMA(w);KEY(w,"liveCutoffEntryFlightPathAngle");jw_number(w,p->live_cutoff_entry_flight_path_angle);COMMA(w);KEY(w,"achievedStateVerified");jw_bool(w,p->achieved_state_verified);COMMA(w);KEY(w,"achievedStateCaptureQualified");jw_bool(w,p->achieved_state_capture_qualified);COMMA(w);KEY(w,"achievedPostBurnPeriapsisAltitude");jw_number(w,p->achieved_post_burn_periapsis_altitude);COMMA(w);KEY(w,"targetCaptureAchieved");jw_bool(w,p->target_capture_achieved);COMMA(w);KEY(w,"confidence");jw_number(w,p->confidence);COMMA(w);KEY(w,"trajectory");trajectory_json(w,&p->trajectory);COMMA(w);KEY(w,"notes");jw_char(w,'[');if(p->note[0])jw_string(w,p->note);jw_char(w,']');jw_char(w,'}');}

static void plan_log_json(JsonWriter*w,const DeorbitPlan*p){
    if(!p){jw_null(w);return;}
    jw_char(w,'{');
    KEY(w,"createdUT");jw_number(w,p->created_ut);COMMA(w);
    KEY(w,"burnUT");jw_number(w,p->burn_ut);COMMA(w);
    KEY(w,"deltaV");jw_number(w,p->delta_v);COMMA(w);
    KEY(w,"estimatedBurnDuration");jw_number(w,p->estimated_burn_duration);COMMA(w);
    KEY(w,"planningMass");jw_number(w,p->planning_mass);COMMA(w);
    KEY(w,"planningAvailableThrust");jw_number(w,p->planning_available_thrust);COMMA(w);
    KEY(w,"predictedTAEMDistance");jw_number(w,p->predicted_taem_distance);COMMA(w);
    KEY(w,"predictedTAEMRangeError");jw_number(w,p->predicted_taem_range_error);COMMA(w);
    KEY(w,"predictedClosestDistance");jw_number(w,p->predicted_closest_distance);COMMA(w);
    KEY(w,"predictedEntryRange");jw_number(w,p->predicted_entry_range);COMMA(w);
    KEY(w,"predictedEntryFlightPathAngle");jw_number(w,p->predicted_entry_flight_path_angle);COMMA(w);
    KEY(w,"predictedPostBurnPeriapsisAltitude");jw_number(w,p->predicted_post_burn_periapsis_altitude);COMMA(w);
    KEY(w,"nominalCaptureAchieved");jw_bool(w,p->nominal_capture_achieved);COMMA(w);
    KEY(w,"robustnessQualified");jw_bool(w,p->robustness_qualified);COMMA(w);
    KEY(w,"robustnessScenarios");jw_integer(w,p->robustness_scenarios);COMMA(w);
    KEY(w,"robustnessPassed");jw_integer(w,p->robustness_passed);COMMA(w);
    KEY(w,"robustnessUnsafe");jw_integer(w,p->robustness_unsafe);COMMA(w);
    KEY(w,"robustnessPassFraction");jw_number(w,p->robustness_pass_fraction);COMMA(w);
    KEY(w,"recoveryPassed");jw_integer(w,p->recovery_passed);COMMA(w);
    KEY(w,"recoveryPassFraction");jw_number(w,p->recovery_pass_fraction);COMMA(w);
    KEY(w,"executionQualified");jw_bool(w,p->execution_qualified);COMMA(w);
    KEY(w,"executionDegraded");jw_bool(w,p->execution_degraded);COMMA(w);
    KEY(w,"worstCaseClosestDistance");jw_number(w,p->worst_case_closest_distance);COMMA(w);
    KEY(w,"worstCaseTAEMRangeError");jw_number(w,p->worst_case_taem_range_error);COMMA(w);
    KEY(w,"worstCaseEntryFlightPathAngle");jw_number(w,p->worst_case_entry_flight_path_angle);COMMA(w);
    KEY(w,"worstCasePostBurnPeriapsisAltitude");jw_number(w,p->worst_case_post_burn_periapsis_altitude);COMMA(w);
    KEY(w,"worstCasePeakDynamicPressure");jw_number(w,p->worst_case_peak_dynamic_pressure);COMMA(w);
    KEY(w,"worstCasePeakGLoad");jw_number(w,p->worst_case_peak_g_load);COMMA(w);
    KEY(w,"liveCutoffCaptureQualified");jw_bool(w,p->live_cutoff_capture_qualified);COMMA(w);
    KEY(w,"liveCutoffPeriapsisAltitude");jw_number(w,p->live_cutoff_periapsis_altitude);COMMA(w);
    KEY(w,"liveCutoffClosestDistance");jw_number(w,p->live_cutoff_closest_distance);COMMA(w);
    KEY(w,"liveCutoffEntryFlightPathAngle");jw_number(w,p->live_cutoff_entry_flight_path_angle);COMMA(w);
    KEY(w,"achievedStateVerified");jw_bool(w,p->achieved_state_verified);COMMA(w);
    KEY(w,"achievedStateCaptureQualified");jw_bool(w,p->achieved_state_capture_qualified);COMMA(w);
    KEY(w,"achievedPostBurnPeriapsisAltitude");jw_number(w,p->achieved_post_burn_periapsis_altitude);COMMA(w);
    KEY(w,"targetCaptureAchieved");jw_bool(w,p->target_capture_achieved);COMMA(w);
    KEY(w,"confidence");jw_number(w,p->confidence);COMMA(w);
    KEY(w,"trajectoryPointCount");jw_integer(w,(long long)p->trajectory.count);COMMA(w);
    KEY(w,"notes");jw_char(w,'[');if(p->note[0])jw_string(w,p->note);jw_char(w,']');
    jw_char(w,'}');
}

static void applied_command_json(JsonWriter*w,const KRPCApplyResult*a){jw_char(w,'{');KEY(w,"applied");jw_bool(w,a->applied);COMMA(w);KEY(w,"autopilotEngaged");jw_bool(w,a->autopilot_engaged);COMMA(w);KEY(w,"referenceFrame");if(a->reference_frame[0])jw_string(w,a->reference_frame);else jw_null(w);COMMA(w);KEY(w,"speedMode");if(a->speed_mode[0])jw_string(w,a->speed_mode);else jw_null(w);COMMA(w);KEY(w,"controlProfile");if(a->control_profile[0])jw_string(w,a->control_profile);else jw_null(w);COMMA(w);KEY(w,"targetPitch");jw_number(w,a->target_pitch);COMMA(w);KEY(w,"targetHeading");jw_number(w,a->target_heading);COMMA(w);KEY(w,"targetRoll");jw_number(w,a->target_roll);COMMA(w);KEY(w,"throttle");jw_number(w,a->throttle);COMMA(w);KEY(w,"wheelSteering");jw_number(w,a->wheel_steering);COMMA(w);KEY(w,"gear");jw_bool(w,a->gear);COMMA(w);KEY(w,"brakes");jw_bool(w,a->brakes);COMMA(w);KEY(w,"airbrakes");jw_bool(w,a->airbrakes);COMMA(w);KEY(w,"actuatorFeedback");if(a->has_actuator_feedback){jw_char(w,'{');KEY(w,"pitch");jw_number(w,a->control_pitch);COMMA(w);KEY(w,"roll");jw_number(w,a->control_roll);COMMA(w);KEY(w,"yaw");jw_number(w,a->control_yaw);jw_char(w,'}');}else jw_null(w);COMMA(w);KEY(w,"controlDiagnostics");if(a->has_control_diagnostics){jw_char(w,'{');KEY(w,"revision");if(a->control_revision[0])jw_string(w,a->control_revision);else jw_null(w);COMMA(w);KEY(w,"targetAoA");jw_number(w,a->control_target_aoa);COMMA(w);KEY(w,"measuredAoA");jw_number(w,a->control_measured_aoa);COMMA(w);KEY(w,"aoaRate");jw_number(w,a->control_aoa_rate);COMMA(w);KEY(w,"rollRate");jw_number(w,a->control_roll_rate);COMMA(w);KEY(w,"bodyPitchRate");jw_number(w,a->control_body_pitch_rate);COMMA(w);KEY(w,"bodyPitchRateAvailable");jw_bool(w,a->control_body_pitch_rate_available);COMMA(w);KEY(w,"bodyRollRate");jw_number(w,a->control_body_roll_rate);COMMA(w);KEY(w,"bodyRollRateAvailable");jw_bool(w,a->control_body_roll_rate_available);COMMA(w);KEY(w,"bodyYawRate");jw_number(w,a->control_body_yaw_rate);COMMA(w);KEY(w,"bodyYawRateAvailable");jw_bool(w,a->control_body_yaw_rate_available);COMMA(w);KEY(w,"pitchError");jw_number(w,a->control_pitch_error);COMMA(w);KEY(w,"pitchTrim");jw_number(w,a->control_pitch_trim);COMMA(w);KEY(w,"pitchAuthority");jw_number(w,a->control_pitch_authority);COMMA(w);KEY(w,"pitchAeroFraction");jw_number(w,a->control_pitch_aero_fraction);COMMA(w);KEY(w,"rollAuthority");jw_number(w,a->control_roll_authority);COMMA(w);KEY(w,"rollRawAuthority");jw_number(w,a->control_roll_raw_authority);COMMA(w);KEY(w,"rollAeroFraction");jw_number(w,a->control_roll_aero_fraction);COMMA(w);KEY(w,"targetRollRate");jw_number(w,a->control_target_roll_rate);COMMA(w);KEY(w,"betaYawGain");jw_number(w,a->control_beta_yaw_gain);COMMA(w);KEY(w,"betaRollCoupling");jw_number(w,a->control_beta_roll_coupling);COMMA(w);KEY(w,"betaConfidence");jw_number(w,a->control_beta_confidence);COMMA(w);KEY(w,"yawCommand");jw_number(w,a->control_yaw_command);jw_char(w,'}');}else jw_null(w);jw_char(w,'}');}
static bool command_saturated(const LandingSnapshot*s){
    if(!s->command_applied)return false;
    const GuidanceCommand*c=&s->command;const KRPCApplyResult*a=&s->applied_command;
    /* KRPCApplyResult is populated in-process from the requested command and
       the actual FlightControlOutput; these values do not make a lossy
       transport round-trip. A difference therefore represents real limiting
       and needs no arbitrary floating-point deadband. */
    if(c->target_throttle!=a->throttle||c->wheel_steering!=a->wheel_steering)return true;
    if(c->autopilot_engaged&&!c->use_inertial_direction&&
       (c->target_pitch!=a->target_pitch||
        norm_signed_deg(c->target_heading-a->target_heading)!=0.0||
        norm_signed_deg(c->target_roll-a->target_roll)!=0.0))return true;
    return c->gear!=a->gear||c->brakes!=a->brakes||c->airbrakes!=a->airbrakes;
}
static void entry_exec_json(JsonWriter*w,const EntryExecTelemetry*e){jw_char(w,'{');KEY(w,"initialized");jw_bool(w,e->initialized);COMMA(w);KEY(w,"complete");jw_bool(w,e->entry_complete);COMMA(w);KEY(w,"phase");jw_string(w,entry_phase_string(e->active_phase));COMMA(w);KEY(w,"lastTransition");jw_string(w,entry_transition_reason_string(e->last_transition_reason));COMMA(w);KEY(w,"transitionCount");jw_integer(w,e->transition_count);jw_char(w,'}');}
static void taem_exec_json(JsonWriter*w,const TaemExecTelemetry*e){jw_char(w,'{');KEY(w,"initialized");jw_bool(w,e->initialized);COMMA(w);KEY(w,"ownershipLatched");jw_bool(w,e->ownership_latched);COMMA(w);KEY(w,"complete");jw_bool(w,e->taem_complete);COMMA(w);KEY(w,"phase");jw_string(w,taem_phase_string(e->active_phase));COMMA(w);KEY(w,"lastTransition");jw_string(w,taem_transition_reason_string(e->last_transition_reason));COMMA(w);KEY(w,"transitionCount");jw_integer(w,e->transition_count);COMMA(w);KEY(w,"recoveryActive");jw_bool(w,e->recovery_active);COMMA(w);KEY(w,"recoveryReason");jw_string(w,taem_recovery_reason_string(e->recovery_reason));COMMA(w);KEY(w,"terminalEvaluationValid");jw_bool(w,e->terminal_evaluation.valid);COMMA(w);KEY(w,"terminalPolicyFeasible");jw_bool(w,e->terminal_evaluation.feasible);COMMA(w);KEY(w,"terminalBlockReason");jw_string(w,taem_terminal_block_reason_string(e->terminal_evaluation.block_reason));COMMA(w);KEY(w,"terminalContract");jw_char(w,'{');KEY(w,"valid");jw_bool(w,e->terminal_contract.valid);COMMA(w);KEY(w,"pathCommitted");jw_bool(w,e->terminal_contract.path_committed);COMMA(w);KEY(w,"rangeToGo");jw_number(w,e->terminal_contract.range_to_go);COMMA(w);KEY(w,"rangeMargin");jw_number(w,e->terminal_contract.range_margin);COMMA(w);KEY(w,"dynamicPressure");jw_number(w,e->terminal_contract.dynamic_pressure);COMMA(w);KEY(w,"dynamicPressureMargin");jw_number(w,e->terminal_contract.dynamic_pressure_margin);COMMA(w);KEY(w,"speedbrakeRequired");jw_bool(w,e->terminal_contract.speedbrake_required);COMMA(w);KEY(w,"speedbrakeAvailable");jw_bool(w,e->terminal_contract.speedbrake_available);COMMA(w);KEY(w,"speedbrakeDynamicPressureMargin");jw_number(w,e->terminal_contract.speedbrake_dynamic_pressure_margin);COMMA(w);KEY(w,"altitude");jw_number(w,e->terminal_contract.altitude);COMMA(w);KEY(w,"altitudeMargin");jw_number(w,e->terminal_contract.altitude_margin);COMMA(w);KEY(w,"flightPathAngle");jw_number(w,e->terminal_contract.flight_path_angle);COMMA(w);KEY(w,"flightPathAngleMargin");jw_number(w,e->terminal_contract.flight_path_angle_margin);COMMA(w);KEY(w,"specificEnergy");jw_number(w,e->terminal_contract.specific_energy);COMMA(w);KEY(w,"specificEnergyMargin");jw_number(w,e->terminal_contract.specific_energy_margin);COMMA(w);KEY(w,"responseTimeAvailable");jw_number(w,e->terminal_contract.response_time_available);COMMA(w);KEY(w,"responseTimeRequired");jw_number(w,e->terminal_contract.response_time_required);COMMA(w);KEY(w,"attitudeResponseQualified");jw_bool(w,e->terminal_contract.attitude_response_qualified);jw_char(w,'}');jw_char(w,'}');}
void snapshot_json(JsonWriter*w,const LandingSnapshot*s){jw_char(w,'{');KEY(w,"schemaVersion");jw_integer(w,2);COMMA(w);KEY(w,"recordType");jw_string(w,"snapshot");COMMA(w);KEY(w,"logSequence");jw_integer(w,(long long)s->log_sequence);COMMA(w);KEY(w,"tickSequence");jw_integer(w,(long long)s->tick_sequence);COMMA(w);KEY(w,"wallMonotonicSeconds");jw_number(w,s->wall_monotonic_seconds);COMMA(w);KEY(w,"simulationDt");jw_number(w,s->simulation_dt);COMMA(w);KEY(w,"connectionStatus");jw_string(w,connection_status_string(s->connection_status));COMMA(w);KEY(w,"phase");jw_string(w,phase_string(s->phase));COMMA(w);KEY(w,"telemetry");telemetry_json(w,&s->telemetry);COMMA(w);KEY(w,"vehicleState");if(s->has_vehicle_state){jw_char(w,'{');KEY(w,"ut");jw_number(w,s->vehicle_state.ut);COMMA(w);KEY(w,"position");jw_char(w,'[');jw_number(w,s->vehicle_state.position.x);COMMA(w);jw_number(w,s->vehicle_state.position.y);COMMA(w);jw_number(w,s->vehicle_state.position.z);jw_char(w,']');COMMA(w);KEY(w,"velocity");jw_char(w,'[');jw_number(w,s->vehicle_state.velocity.x);COMMA(w);jw_number(w,s->vehicle_state.velocity.y);COMMA(w);jw_number(w,s->vehicle_state.velocity.z);jw_char(w,']');COMMA(w);KEY(w,"mass");jw_number(w,s->vehicle_state.mass);jw_char(w,'}');}else jw_null(w);COMMA(w);KEY(w,"attitude");if(s->has_attitude_quaternion){jw_char(w,'{');KEY(w,"quaternion");vec4_json(w,s->attitude_quaternion);COMMA(w);KEY(w,"referenceFrame");jw_string(w,s->attitude_reference_frame);jw_char(w,'}');}else jw_null(w);COMMA(w);KEY(w,"guidanceState");jw_char(w,'{');KEY(w,"burnStarted");jw_bool(w,s->guidance_has_burn_started);COMMA(w);KEY(w,"burnCompleted");jw_bool(w,s->guidance_burn_completed);COMMA(w);KEY(w,"atmosphereCrossed");jw_bool(w,s->guidance_atmosphere_crossed);COMMA(w);KEY(w,"finalCaptured");jw_bool(w,s->guidance_final_captured);COMMA(w);KEY(w,"airbrakesDeployed");jw_bool(w,s->guidance_airbrakes_deployed);COMMA(w);KEY(w,"hacSideSelected");jw_bool(w,s->guidance_hac_side_selected);COMMA(w);KEY(w,"deliveredDeltaV");jw_number(w,s->guidance_delivered_delta_v);COMMA(w);KEY(w,"burnActiveElapsed");jw_number(w,s->guidance_burn_active_elapsed);COMMA(w);KEY(w,"sTurnSign");jw_number(w,s->guidance_s_turn_sign);COMMA(w);KEY(w,"hacSide");jw_number(w,s->guidance_hac_side);COMMA(w);KEY(w,"hacCaptured");jw_bool(w,s->guidance_hac_captured);COMMA(w);KEY(w,"hacCompleted");jw_bool(w,s->guidance_hac_completed);COMMA(w);KEY(w,"hacProgressValid");jw_bool(w,s->guidance_hac_progress_valid);COMMA(w);KEY(w,"hacArcRemaining");jw_number(w,s->guidance_hac_remaining);COMMA(w);KEY(w,"minimumTurnRadius");jw_number(w,s->guidance_minimum_turn_radius);COMMA(w);KEY(w,"entryLegElapsed");jw_number(w,s->guidance_entry_leg_elapsed);COMMA(w);KEY(w,"hacTransitionActive");jw_bool(w,s->guidance_hac_transition_active);COMMA(w);KEY(w,"hacRadius");jw_number(w,s->guidance_hac_radius);COMMA(w);KEY(w,"hacCircuitCount");jw_integer(w,s->guidance_hac_circuit_count);COMMA(w);KEY(w,"hacCircuitGlideSlope");jw_number(w,s->guidance_hac_circuit_slope);COMMA(w);KEY(w,"terminalPredictionValid");jw_bool(w,s->guidance_terminal_prediction_valid);COMMA(w);KEY(w,"terminalCandidateValid");jw_bool(w,s->guidance_terminal_candidate_valid);COMMA(w);KEY(w,"terminalPathCommitted");jw_bool(w,s->guidance_terminal_committed);COMMA(w);KEY(w,"terminalPathSelected");jw_bool(w,s->guidance_terminal_committed||s->guidance_hac_side_selected);COMMA(w);KEY(w,"terminalPathCaptured");jw_bool(w,s->guidance_hac_captured&&!s->guidance_hac_transition_active);COMMA(w);KEY(w,"terminalPathComplete");jw_bool(w,s->guidance_hac_completed);COMMA(w);KEY(w,"terminalPathRemaining");jw_number(w,s->guidance_hac_remaining);COMMA(w);KEY(w,"terminalCandidateRadius");jw_number(w,s->guidance_terminal_candidate_radius);COMMA(w);KEY(w,"terminalCandidateAltitude");jw_number(w,s->guidance_terminal_candidate_altitude);COMMA(w);KEY(w,"terminalCandidateSpeed");jw_number(w,s->guidance_terminal_candidate_speed);COMMA(w);KEY(w,"terminalReferenceFPA");jw_number(w,s->guidance_terminal_reference_fpa);COMMA(w);KEY(w,"terminalBlend");jw_number(w,s->guidance_terminal_mix);COMMA(w);KEY(w,"hacTransitionProgress");jw_number(w,s->guidance_hac_transition_progress);COMMA(w);KEY(w,"commandedCourseRateEstimate");jw_number(w,s->guidance_commanded_course_rate_estimate);COMMA(w);KEY(w,"entryPlanValid");jw_bool(w,s->guidance_entry_plan_valid);COMMA(w);KEY(w,"entryPlanTerminalReady");jw_bool(w,s->guidance_entry_plan_terminal_ready);COMMA(w);KEY(w,"entryPlanTargetBank");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_target_bank);COMMA(w);KEY(w,"entryPlanTargetAoA");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_target_aoa);COMMA(w);KEY(w,"entryPlanTargetHeading");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_target_heading);COMMA(w);KEY(w,"entryPlanSegmentRemaining");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_segment_remaining);COMMA(w);KEY(w,"entryPlanCost");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_cost);COMMA(w);KEY(w,"entryPlanTAEMRangeError");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_taem_range_error);COMMA(w);KEY(w,"entryPlanTAEMSpeed");nullable_number(w,s->guidance_entry_plan_valid&&isfinite(s->guidance_entry_plan_taem_speed),s->guidance_entry_plan_taem_speed);COMMA(w);KEY(w,"entryPlanTAEMEnergyError");nullable_number(w,s->guidance_entry_plan_valid&&isfinite(s->guidance_entry_plan_taem_energy_error),s->guidance_entry_plan_taem_energy_error);COMMA(w);KEY(w,"entryPlanPredictedReversals");jw_integer(w,(long long)s->guidance_entry_plan_predicted_reversals);COMMA(w);KEY(w,"entryReversalScheduled");jw_bool(w,s->guidance_entry_reversal_scheduled);COMMA(w);KEY(w,"entryReversalIsFinal");jw_bool(w,s->guidance_entry_reversal_is_final);COMMA(w);KEY(w,"entryReversalTimeRemaining");nullable_number(w,s->guidance_entry_reversal_scheduled,s->guidance_entry_reversal_time_remaining);COMMA(w);KEY(w,"entryReversalRange");nullable_number(w,s->guidance_entry_reversal_scheduled,s->guidance_entry_reversal_range);COMMA(w);KEY(w,"entryReversalSign");nullable_number(w,s->guidance_entry_reversal_scheduled,s->guidance_entry_reversal_sign);COMMA(w);KEY(w,"entryExecutive");entry_exec_json(w,&s->guidance_entry_exec);COMMA(w);KEY(w,"taemExecutive");taem_exec_json(w,&s->guidance_taem_exec);jw_char(w,'}');COMMA(w);KEY(w,"decisionReason");if(s->decision_reason[0])jw_string(w,s->decision_reason);else jw_null(w);COMMA(w);KEY(w,"decisionTrace");jw_char(w,'{');KEY(w,"phase");jw_string(w,phase_string(s->phase));COMMA(w);KEY(w,"controlProfile");jw_string(w,profile_string(s->command.control_profile));COMMA(w);KEY(w,"automationEngaged");jw_bool(w,s->automation_engaged);COMMA(w);KEY(w,"paused");jw_bool(w,s->paused);COMMA(w);KEY(w,"burnStarted");jw_bool(w,s->guidance_has_burn_started);COMMA(w);KEY(w,"burnCompleted");jw_bool(w,s->guidance_burn_completed);COMMA(w);KEY(w,"atmosphereCrossed");jw_bool(w,s->guidance_atmosphere_crossed);COMMA(w);KEY(w,"finalCaptured");jw_bool(w,s->guidance_final_captured);COMMA(w);KEY(w,"deliveredDeltaV");jw_number(w,s->guidance_delivered_delta_v);COMMA(w);KEY(w,"entryLegElapsed");jw_number(w,s->guidance_entry_leg_elapsed);COMMA(w);KEY(w,"inputs");jw_char(w,'{');KEY(w,"rangeToSite");jw_number(w,s->telemetry.range_to_site);COMMA(w);KEY(w,"courseToSiteError");jw_number(w,s->telemetry.course_to_site_error);COMMA(w);KEY(w,"flightPathAngle");jw_number(w,s->telemetry.flight_path_angle);COMMA(w);KEY(w,"energyExcessRange");jw_number(w,s->telemetry.energy_excess_range);COMMA(w);KEY(w,"predictedMissDistance");jw_number(w,s->telemetry.predicted_miss_distance);COMMA(w);KEY(w,"predictedTAEMRangeError");jw_number(w,s->telemetry.predicted_taem_range_error);COMMA(w);KEY(w,"dynamicPressure");jw_number(w,s->telemetry.dynamic_pressure);COMMA(w);KEY(w,"gForce");jw_number(w,s->telemetry.g_force);COMMA(w);KEY(w,"angleOfAttack");jw_number(w,s->telemetry.angle_of_attack);COMMA(w);KEY(w,"roll");jw_number(w,s->telemetry.roll);COMMA(w);KEY(w,"heading");jw_number(w,s->telemetry.heading);jw_char(w,'}');COMMA(w);KEY(w,"status");jw_string(w,s->status_message);jw_char(w,'}');COMMA(w);KEY(w,"command");command_json(w,&s->command);COMMA(w);KEY(w,"commandLifecycle");jw_char(w,'{');KEY(w,"computedWallSeconds");jw_number(w,s->command_computed_wall_seconds);COMMA(w);KEY(w,"applyAttempted");jw_bool(w,s->command_apply_attempted);COMMA(w);KEY(w,"applyStartedWallSeconds");nullable_number(w,s->command_apply_attempted,s->command_apply_started_wall_seconds);COMMA(w);KEY(w,"applyFinishedWallSeconds");nullable_number(w,s->command_apply_attempted,s->command_apply_finished_wall_seconds);COMMA(w);KEY(w,"applied");jw_bool(w,s->command_applied);COMMA(w);KEY(w,"saturatedOrNormalized");jw_bool(w,command_saturated(s));COMMA(w);KEY(w,"appliedCommand");if(s->command_apply_attempted)applied_command_json(w,&s->applied_command);else jw_null(w);jw_char(w,'}');COMMA(w);KEY(w,"plan");plan_json(w,s->plan);COMMA(w);KEY(w,"actualTrajectory");trajectory_json(w,&s->actual_trajectory);COMMA(w);KEY(w,"referenceTrajectory");trajectory_json(w,&s->reference_trajectory);COMMA(w);KEY(w,"predictedTrajectory");trajectory_json(w,&s->predicted_trajectory);COMMA(w);KEY(w,"statusMessage");jw_string(w,s->status_message);COMMA(w);KEY(w,"warningMessage");if(s->has_warning)jw_string(w,s->warning_message);else jw_null(w);COMMA(w);KEY(w,"lastError");if(s->has_error)jw_string(w,s->last_error);else jw_null(w);COMMA(w);KEY(w,"automationEngaged");jw_bool(w,s->automation_engaged);COMMA(w);KEY(w,"paused");jw_bool(w,s->paused);COMMA(w);KEY(w,"calibration");const CalibrationSnapshot*c=&s->calibration;jw_char(w,'{');KEY(w,"state");jw_string(w,c->state==CAL_IDLE?"Idle":cal_state_string(c->state));COMMA(w);KEY(w,"active");jw_bool(w,c->active);COMMA(w);KEY(w,"progress");jw_number(w,c->progress);COMMA(w);KEY(w,"targetAngleOfAttack");jw_number(w,c->target_angle_of_attack);COMMA(w);KEY(w,"acceptedSamples");jw_integer(w,c->accepted_samples);COMMA(w);KEY(w,"rejectedSamples");jw_integer(w,c->rejected_samples);COMMA(w);KEY(w,"confidence");jw_number(w,c->confidence);COMMA(w);KEY(w,"planningConfidence");jw_number(w,c->planning_confidence);COMMA(w);KEY(w,"bestGlideAngleOfAttack");jw_number(w,c->best_glide_angle_of_attack);COMMA(w);KEY(w,"estimatedStallSpeed");jw_number(w,c->estimated_stall_speed);COMMA(w);KEY(w,"status");jw_string(w,c->status);COMMA(w);KEY(w,"warning");if(c->has_warning)jw_string(w,c->warning);else jw_null(w);jw_char(w,'}');COMMA(w);KEY(w,"adaptiveVehicleProfile");vehicle_json(w,&s->adaptive_vehicle_profile);jw_char(w,'}');}

void snapshot_log_json(JsonWriter*w,const LandingSnapshot*s){
    jw_char(w,'{');
    KEY(w,"schemaVersion");jw_integer(w,3);COMMA(w);
    KEY(w,"recordType");jw_string(w,"snapshot");COMMA(w);
    KEY(w,"logSequence");jw_integer(w,(long long)s->log_sequence);COMMA(w);KEY(w,"tickSequence");jw_integer(w,(long long)s->tick_sequence);COMMA(w);
    KEY(w,"wallMonotonicSeconds");jw_number(w,s->wall_monotonic_seconds);COMMA(w);
    KEY(w,"simulationDt");jw_number(w,s->simulation_dt);COMMA(w);
    KEY(w,"connectionStatus");jw_string(w,connection_status_string(s->connection_status));COMMA(w);
    KEY(w,"phase");jw_string(w,phase_string(s->phase));COMMA(w);
    KEY(w,"telemetry");telemetry_json(w,&s->telemetry);COMMA(w);
    KEY(w,"vehicleState");
    if(s->has_vehicle_state){
        jw_char(w,'{');KEY(w,"ut");jw_number(w,s->vehicle_state.ut);COMMA(w);
        KEY(w,"position");jw_char(w,'[');jw_number(w,s->vehicle_state.position.x);COMMA(w);jw_number(w,s->vehicle_state.position.y);COMMA(w);jw_number(w,s->vehicle_state.position.z);jw_char(w,']');COMMA(w);
        KEY(w,"velocity");jw_char(w,'[');jw_number(w,s->vehicle_state.velocity.x);COMMA(w);jw_number(w,s->vehicle_state.velocity.y);COMMA(w);jw_number(w,s->vehicle_state.velocity.z);jw_char(w,']');COMMA(w);
        KEY(w,"mass");jw_number(w,s->vehicle_state.mass);jw_char(w,'}');
    }else jw_null(w);
    COMMA(w);KEY(w,"attitude");
    if(s->has_attitude_quaternion){jw_char(w,'{');KEY(w,"quaternion");vec4_json(w,s->attitude_quaternion);COMMA(w);KEY(w,"referenceFrame");jw_string(w,s->attitude_reference_frame);jw_char(w,'}');}else jw_null(w);
    COMMA(w);KEY(w,"guidanceState");jw_char(w,'{');
    KEY(w,"burnStarted");jw_bool(w,s->guidance_has_burn_started);COMMA(w);
    KEY(w,"burnCompleted");jw_bool(w,s->guidance_burn_completed);COMMA(w);
    KEY(w,"atmosphereCrossed");jw_bool(w,s->guidance_atmosphere_crossed);COMMA(w);
    KEY(w,"finalCaptured");jw_bool(w,s->guidance_final_captured);COMMA(w);
    KEY(w,"airbrakesDeployed");jw_bool(w,s->guidance_airbrakes_deployed);COMMA(w);
    KEY(w,"hacSideSelected");jw_bool(w,s->guidance_hac_side_selected);COMMA(w);
    KEY(w,"deliveredDeltaV");jw_number(w,s->guidance_delivered_delta_v);COMMA(w);
    KEY(w,"burnActiveElapsed");jw_number(w,s->guidance_burn_active_elapsed);COMMA(w);
    KEY(w,"sTurnSign");jw_number(w,s->guidance_s_turn_sign);COMMA(w);
    KEY(w,"hacSide");jw_number(w,s->guidance_hac_side);COMMA(w);
    KEY(w,"hacCaptured");jw_bool(w,s->guidance_hac_captured);COMMA(w);
    KEY(w,"hacCompleted");jw_bool(w,s->guidance_hac_completed);COMMA(w);
    KEY(w,"hacProgressValid");jw_bool(w,s->guidance_hac_progress_valid);COMMA(w);
    KEY(w,"hacTransitionActive");jw_bool(w,s->guidance_hac_transition_active);COMMA(w);
    KEY(w,"hacArcRemaining");jw_number(w,s->guidance_hac_remaining);COMMA(w);
    KEY(w,"hacRadius");jw_number(w,s->guidance_hac_radius);COMMA(w);
    KEY(w,"hacCircuitCount");jw_integer(w,s->guidance_hac_circuit_count);COMMA(w);
    KEY(w,"hacCircuitGlideSlope");jw_number(w,s->guidance_hac_circuit_slope);COMMA(w);
    KEY(w,"terminalReentryAfterUT");jw_number(w,s->guidance_terminal_reentry_after_ut);COMMA(w);
    KEY(w,"terminalPredictionValid");jw_bool(w,s->guidance_terminal_prediction_valid);COMMA(w);
    KEY(w,"terminalCandidateValid");jw_bool(w,s->guidance_terminal_candidate_valid);COMMA(w);
    KEY(w,"terminalPathCommitted");jw_bool(w,s->guidance_terminal_committed);COMMA(w);
    /* Generic unified-TAEM progress is the primary terminal ownership view.
       Keep the HAC-prefixed fields above as geometry-provider diagnostics for
       backwards-compatible log forensics. */
    KEY(w,"terminalPathSelected");jw_bool(w,s->guidance_terminal_committed||s->guidance_hac_side_selected);COMMA(w);
    KEY(w,"terminalPathCaptured");jw_bool(w,s->guidance_hac_captured&&!s->guidance_hac_transition_active);COMMA(w);
    KEY(w,"terminalPathComplete");jw_bool(w,s->guidance_hac_completed);COMMA(w);
    KEY(w,"terminalPathTransitionActive");jw_bool(w,s->guidance_hac_transition_active);COMMA(w);
    KEY(w,"terminalPathRemaining");jw_number(w,s->guidance_hac_remaining);COMMA(w);
    KEY(w,"terminalCandidateRadius");jw_number(w,s->guidance_terminal_candidate_radius);COMMA(w);
    KEY(w,"terminalCandidateAltitude");jw_number(w,s->guidance_terminal_candidate_altitude);COMMA(w);
    KEY(w,"terminalCandidateSpeed");jw_number(w,s->guidance_terminal_candidate_speed);COMMA(w);
    KEY(w,"candidateKind");jw_integer(w,s->guidance_terminal_candidate_kind);COMMA(w);
    KEY(w,"candidateGeometryDegraded");jw_bool(w,s->guidance_terminal_candidate_geometry_degraded);COMMA(w);
    KEY(w,"candidateEnergyDegraded");jw_bool(w,s->guidance_terminal_candidate_energy_degraded);COMMA(w);
    KEY(w,"candidateShellDegraded");jw_bool(w,s->guidance_terminal_candidate_shell_degraded);COMMA(w);
    KEY(w,"candidateSide");jw_number(w,s->guidance_terminal_candidate_side);COMMA(w);
    KEY(w,"candidateSlope");jw_number(w,s->guidance_terminal_candidate_slope);COMMA(w);
    KEY(w,"candidateCurveLength");jw_number(w,s->guidance_terminal_candidate_curve_length);COMMA(w);
    KEY(w,"candidateLeadLength");jw_number(w,s->guidance_terminal_candidate_lead_length);COMMA(w);
    KEY(w,"candidateArcRemaining");jw_number(w,s->guidance_terminal_candidate_arc_remaining);COMMA(w);
    KEY(w,"candidateFinalDistance");jw_number(w,s->guidance_terminal_candidate_final_distance);COMMA(w);
    KEY(w,"candidateQuality");jw_number(w,s->guidance_terminal_candidate_quality);COMMA(w);
    KEY(w,"candidateArrivalUT");jw_number(w,s->guidance_terminal_candidate_arrival_ut);COMMA(w);
    KEY(w,"candidateCourse");jw_number(w,s->guidance_terminal_candidate_course);COMMA(w);
    KEY(w,"candidatePeakLateral");jw_number(w,s->guidance_terminal_candidate_peak_lateral);COMMA(w);
    KEY(w,"candidatePeakRateRatio");jw_number(w,s->guidance_terminal_candidate_peak_rate_ratio);COMMA(w);
    KEY(w,"candidateExitSpeed");jw_number(w,s->guidance_terminal_candidate_exit_speed);COMMA(w);
    KEY(w,"candidateTrackingEvaluated");jw_bool(w,s->guidance_terminal_candidate_tracking_evaluated);COMMA(w);
    KEY(w,"candidateTrackingError");jw_number(w,s->guidance_terminal_candidate_tracking_error);COMMA(w);
    KEY(w,"candidateTrackingCourseError");jw_number(w,s->guidance_terminal_candidate_tracking_course_error);COMMA(w);
    KEY(w,"candidateTrackingEndpointError");jw_number(w,s->guidance_terminal_candidate_tracking_endpoint_error);COMMA(w);
    KEY(w,"candidateTrackingMargin");jw_number(w,s->guidance_terminal_candidate_tracking_margin);COMMA(w);
    KEY(w,"candidatePathDegraded");jw_bool(w,s->guidance_terminal_candidate_path_degraded);COMMA(w);
    KEY(w,"candidateControlDegraded");jw_bool(w,s->guidance_terminal_candidate_control_degraded);COMMA(w);
    KEY(w,"candidateRateDegraded");jw_bool(w,s->guidance_terminal_candidate_rate_degraded);COMMA(w);
    KEY(w,"candidateEndDegraded");jw_bool(w,s->guidance_terminal_candidate_end_degraded);COMMA(w);
    KEY(w,"terminalReferenceFPA");jw_number(w,s->guidance_terminal_reference_fpa);COMMA(w);
    KEY(w,"terminalBlend");jw_number(w,s->guidance_terminal_mix);COMMA(w);
    KEY(w,"hacTransitionProgress");jw_number(w,s->guidance_hac_transition_progress);COMMA(w);
    KEY(w,"hacTransitionControlPoints");jw_char(w,'[');
    for(int i=0;i<8;i++){if(i)COMMA(w);jw_number(w,s->guidance_hac_transition_points[i]);}
    jw_char(w,']');COMMA(w);
    KEY(w,"hacTransitionHeadingCone");jw_bool(w,s->guidance_hac_transition_heading_cone);COMMA(w);
    KEY(w,"hacTransitionLeadCurve");jw_bool(w,s->guidance_hac_transition_lead_curve);COMMA(w);
    KEY(w,"hacTransitionLeadPoints");jw_char(w,'[');
    for(int i=0;i<6;i++){if(i)COMMA(w);jw_number(w,s->guidance_hac_transition_lead_points[i]);}
    jw_char(w,']');COMMA(w);
    KEY(w,"hacTransitionConeCenter");jw_char(w,'[');
    jw_number(w,s->guidance_hac_transition_cone_center[0]);COMMA(w);
    jw_number(w,s->guidance_hac_transition_cone_center[1]);jw_char(w,']');COMMA(w);
    KEY(w,"hacTransitionConeStartAngle");jw_number(w,s->guidance_hac_transition_cone_start_angle);COMMA(w);
    KEY(w,"hacTransitionConeEndAngle");jw_number(w,s->guidance_hac_transition_cone_end_angle);COMMA(w);
    KEY(w,"hacTransitionConeArcLength");jw_number(w,s->guidance_hac_transition_cone_arc_length);COMMA(w);
    KEY(w,"commandedCourseRateEstimate");jw_number(w,s->guidance_commanded_course_rate_estimate);COMMA(w);
    KEY(w,"minimumTurnRadius");jw_number(w,s->guidance_minimum_turn_radius);COMMA(w);
    KEY(w,"entryLegElapsed");jw_number(w,s->guidance_entry_leg_elapsed);COMMA(w);
    KEY(w,"entryPlanValid");jw_bool(w,s->guidance_entry_plan_valid);COMMA(w);
    KEY(w,"entryPlanTerminalReady");jw_bool(w,s->guidance_entry_plan_terminal_ready);COMMA(w);
    KEY(w,"entryPlanUT");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_ut);COMMA(w);
    KEY(w,"entryPlanTargetBank");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_target_bank);COMMA(w);
    KEY(w,"entryPlanTargetAoA");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_target_aoa);COMMA(w);
    KEY(w,"entryPlanTargetHeading");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_target_heading);COMMA(w);
    KEY(w,"entryPlanSegmentRemaining");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_segment_remaining);COMMA(w);
    KEY(w,"entryPlanCost");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_cost);COMMA(w);
    KEY(w,"entryPlanTAEMRangeError");nullable_number(w,s->guidance_entry_plan_valid,s->guidance_entry_plan_taem_range_error);COMMA(w);
    KEY(w,"entryPlanTAEMSpeed");nullable_number(w,s->guidance_entry_plan_valid&&isfinite(s->guidance_entry_plan_taem_speed),s->guidance_entry_plan_taem_speed);COMMA(w);
    KEY(w,"entryPlanTAEMEnergyError");nullable_number(w,s->guidance_entry_plan_valid&&isfinite(s->guidance_entry_plan_taem_energy_error),s->guidance_entry_plan_taem_energy_error);COMMA(w);
    KEY(w,"entryPlanPredictedReversals");jw_integer(w,(long long)s->guidance_entry_plan_predicted_reversals);COMMA(w);
    KEY(w,"entryReversalScheduled");jw_bool(w,s->guidance_entry_reversal_scheduled);COMMA(w);
    KEY(w,"entryReversalIsFinal");jw_bool(w,s->guidance_entry_reversal_is_final);COMMA(w);
    KEY(w,"entryReversalTimeRemaining");nullable_number(w,s->guidance_entry_reversal_scheduled,s->guidance_entry_reversal_time_remaining);COMMA(w);
    KEY(w,"entryReversalRange");nullable_number(w,s->guidance_entry_reversal_scheduled,s->guidance_entry_reversal_range);COMMA(w);
    KEY(w,"entryReversalSign");nullable_number(w,s->guidance_entry_reversal_scheduled,s->guidance_entry_reversal_sign);COMMA(w);
    KEY(w,"entryExecutive");entry_exec_json(w,&s->guidance_entry_exec);COMMA(w);
    KEY(w,"taemExecutive");taem_exec_json(w,&s->guidance_taem_exec);jw_char(w,'}');
    COMMA(w);KEY(w,"decisionReason");if(s->decision_reason[0])jw_string(w,s->decision_reason);else jw_null(w);
    COMMA(w);KEY(w,"command");command_json(w,&s->command);
    COMMA(w);KEY(w,"commandLifecycle");jw_char(w,'{');
    KEY(w,"computedWallSeconds");jw_number(w,s->command_computed_wall_seconds);COMMA(w);
    KEY(w,"applyAttempted");jw_bool(w,s->command_apply_attempted);COMMA(w);
    KEY(w,"applyStartedWallSeconds");nullable_number(w,s->command_apply_attempted,s->command_apply_started_wall_seconds);COMMA(w);
    KEY(w,"applyFinishedWallSeconds");nullable_number(w,s->command_apply_attempted,s->command_apply_finished_wall_seconds);COMMA(w);
    KEY(w,"applied");jw_bool(w,s->command_applied);COMMA(w);
    KEY(w,"saturatedOrNormalized");jw_bool(w,command_saturated(s));COMMA(w);
    KEY(w,"appliedCommand");if(s->command_apply_attempted)applied_command_json(w,&s->applied_command);else jw_null(w);jw_char(w,'}');
    COMMA(w);KEY(w,"plan");plan_log_json(w,s->plan);
    COMMA(w);KEY(w,"trajectoryState");jw_char(w,'{');
    KEY(w,"actualPointCount");jw_integer(w,(long long)s->actual_trajectory.count);COMMA(w);
    KEY(w,"referencePointCount");jw_integer(w,(long long)s->reference_trajectory.count);COMMA(w);
    KEY(w,"planPointCount");jw_integer(w,(long long)(s->plan?s->plan->trajectory.count:0));jw_char(w,'}');
    COMMA(w);KEY(w,"statusMessage");jw_string(w,s->status_message);
    COMMA(w);KEY(w,"warningMessage");if(s->has_warning)jw_string(w,s->warning_message);else jw_null(w);
    COMMA(w);KEY(w,"lastError");if(s->has_error)jw_string(w,s->last_error);else jw_null(w);
    COMMA(w);KEY(w,"automationEngaged");jw_bool(w,s->automation_engaged);
    COMMA(w);KEY(w,"paused");jw_bool(w,s->paused);
    COMMA(w);KEY(w,"calibration");const CalibrationSnapshot*c=&s->calibration;jw_char(w,'{');
    KEY(w,"state");jw_string(w,cal_state_string(c->state));COMMA(w);
    KEY(w,"active");jw_bool(w,c->active);COMMA(w);
    KEY(w,"progress");jw_number(w,c->progress);COMMA(w);
    KEY(w,"targetAngleOfAttack");jw_number(w,c->target_angle_of_attack);COMMA(w);
    KEY(w,"acceptedSamples");jw_integer(w,c->accepted_samples);COMMA(w);
    KEY(w,"rejectedSamples");jw_integer(w,c->rejected_samples);COMMA(w);
    KEY(w,"confidence");jw_number(w,c->confidence);COMMA(w);
    KEY(w,"planningConfidence");jw_number(w,c->planning_confidence);COMMA(w);
    KEY(w,"bestGlideAngleOfAttack");jw_number(w,c->best_glide_angle_of_attack);COMMA(w);
    KEY(w,"estimatedStallSpeed");jw_number(w,c->estimated_stall_speed);COMMA(w);
    KEY(w,"status");jw_string(w,c->status);COMMA(w);
    KEY(w,"warning");if(c->has_warning)jw_string(w,c->warning);else jw_null(w);jw_char(w,'}');
    COMMA(w);KEY(w,"adaptiveVehicleProfile");vehicle_json(w,&s->adaptive_vehicle_profile);
    jw_char(w,'}');
}
