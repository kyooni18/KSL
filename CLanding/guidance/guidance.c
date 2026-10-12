#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>


/* A rejected solution does not establish a control departure. Only the
 * measured departure monitor owns recovery; this remains unqualified guidance. */
static GuidanceResult terminal_warning_recovery(GuidanceMachine *g,
        const Telemetry *t,const PlanetModel *p,AerodynamicModel aero,
        const LandingConfiguration *cfg,const char *reason,double dt) {
    GuidanceCommand c; guidance_command_init(&c);
    bool grounded=t->main_gear_grounded||t->nose_gear_grounded||
        !strcasecmp(t->vessel_situation,"landed");
    if (grounded) g->terminal_landing_warning_latched=true;
    /* A failed landing can still brake to a stop, but cannot replan its way
       back into a successful mission. Match touchdown's rollout stop gate. */
    if (grounded && isfinite(t->surface_speed) &&
            t->surface_speed>=0.0 && t->surface_speed<1.5) {
        c.control_profile=PROFILE_ROLLOUT;
        c.gear=true;
        c.brakes=true;
        guidance_abort(g);
        return result_make(PHASE_ABORT,c,reason,reason);
    }
    bool recovering=!grounded&&g->attitude_recovery;
    GuidancePhase phase=grounded?PHASE_ROLLOUT:recovering?PHASE_ATTITUDE_RECOVERY:
        g->terminal_region_entered?PHASE_TAEM:PHASE_ENTRY_ENERGY;
    bool usable=isfinite(t->heading)&&isfinite(t->flight_path_angle)&&
        isfinite(t->true_air_speed)&&isfinite(t->mean_altitude);
    if (usable) {
        double course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        double bank=0,heading=course;
        double aoa=cfg->vehicle.terminal_maximum_lift_angle_of_attack;
        if (!grounded && !recovering && t->true_air_speed>=cfg->vehicle.minimum_safe_speed) {
            HACGuidance line=terminal_runway_line_path_guidance(t,&cfg->site,
                &cfg->guidance,&cfg->vehicle,planet_surface_gravity(p),course);
            if (isfinite(line.heading)&&isfinite(line.bank)) {
                heading=line.heading;
                bank=clampd(line.bank,-cfg->vehicle.maximum_bank_angle,
                    cfg->vehicle.maximum_bank_angle);
            }
            double distance=fmax(1.0,fabs(t->runway_along_track));
            double slope=atan2(fmax(0,t->mean_altitude-cfg->site.altitude),distance)*RAD2DEG;
            double candidate=terminal_fpa_force_aoa(t,p,aero,&cfg->vehicle,-slope);
            if (isfinite(candidate)) aoa=candidate;
        }
        if (recovering) aoa=g->recovery_aoa;
        aoa=clampd(aoa,0,cfg->vehicle.maximum_angle_of_attack);
        c=atmospheric(t,heading,bank,&cfg->vehicle,0,false,
            grounded?PROFILE_ROLLOUT:recovering?PROFILE_RECOVERY:
            g->terminal_region_entered?PROFILE_TAEM:PROFILE_ENTRY);
        c.has_target_aoa=true;c.target_aoa=aoa;
        c.target_pitch=t->flight_path_angle+aoa;
        c.heading_control_enabled=!grounded&&!recovering;
        c.hac_control_tuning=phase==PHASE_TAEM;
        c.terminal_pitch_tuning=phase==PHASE_TAEM&&g->terminal_glide_mode;
    }
    c.target_throttle=0;
    c.gear=grounded||t->gear||t->radar_altitude<=cfg->guidance.gear_deployment_altitude;
    c.brakes=grounded;
    /* Release a rejected route so the next tick can price a fresh solution. */
    g->mm305_route_committed=false;g->terminal_path_committed=false;
    g->mm305_acquisition_route_valid=false;g->hac_captured=false;
    g->phase=phase;
    GuidanceResult r=result_make(phase,c,recovering?
        "Attitude recovery continues after a guidance warning.":
        "Continuing bounded guidance while the rejected solution replans.",reason);
    return stabilized(g,r,t,&cfg->vehicle,&cfg->guidance,dt);
}

GuidanceResult guidance_update_with_terminal_model(GuidanceMachine*g,
        const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TerminalModel *terminal_model){
    /* Keep the public update in the configured runway frame. Terminal guidance
       owns reciprocal-runway preview, selection, reframing, and commitment in
       terminal_guidance_selected(). Pre-framing here makes a selected RW27
       appear as a new primary runway on the next preview and aliases the stored
       runway_end_index, causing the MM304/MM305 inlet to flip ends every cycle. */
    double dt=g->has_previous_ut?clampd(t->ut-g->previous_ut,0,1):.1;
    g->entry_terminal_model=terminal_model;
    GuidanceResult r=guidance_update_impl(g,t,state,plan,p,aero,cfg,terminal_model);
    g->entry_terminal_model=NULL;
    if ((r.phase==PHASE_ABORT ||
            (r.phase==PHASE_COMPLETE&&g->terminal_landing_warning_latched)) &&
            g->automation_engaged && !g->aborted) {
        char warning[sizeof(r.status)];
        snprintf(warning,sizeof(warning),"%s",
            r.phase==PHASE_COMPLETE?g->abort_reason:r.status);
        guidance_result_clear(&r);
        return terminal_warning_recovery(g,t,p,aero,cfg,warning,dt);
    }
    if(plan&&r.phase==PHASE_DEORBIT_BURN&&g->has_burn_command_started&&!g->deorbit_burn_completed&&r.command.target_throttle<=0){
        bool has_peri=plan->predicted_post_burn_periapsis_altitude>10000&&plan->predicted_post_burn_periapsis_altitude<p->atmosphere_depth&&isfinite(t->periapsis_altitude)&&t->periapsis_altitude>-p->radius*.5;
        double target=plan->delta_v+(has_peri?8:0),remaining=target-g->delivered_delta_v;
        if(remaining>0&&remaining<=GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS+1e-9){
            g->delivered_delta_v=target;
            g->deorbit_burn_completed=true;
            g->phase=PHASE_ENTRY_INTERFACE;
            guidance_result_clear(&r);
            return result_make(PHASE_ENTRY_INTERFACE,entry_capture(t,state,&cfg->vehicle),"Burn complete. Using RCS/direct control to capture prograde entry attitude until atmospheric interface.",NULL);
        }
    }
    /* TAEM already carries large, continuously moving bank references.  Use
       the same body-rate-damped terminal roll loop before HAC commit instead
       of the generic attitude loop, which was driving +/-100 deg/s roll
       limit cycles while chasing a 25-40 deg TAEM bank. */
    r.command.hac_control_tuning=g->terminal_region_entered&&
        (r.phase==PHASE_TAEM||g->hac_side_selected)&&
        (!g->terminal_glide_mode||!g->terminal_test_capture_active||r.phase==PHASE_TAEM);
    /* Pitch capture needs its own terminal-only controller before the HAC is
       fully selected in the dedicated rehearsal. Production entry/S-turn is
       deliberately excluded; normal flight only enables this after terminal
       region capture, while the HAC test may use it on the pre-HAC dogleg. */
    r.command.terminal_pitch_tuning=g->terminal_region_entered&&
        (g->hac_side_selected||(g->terminal_glide_mode&&g->terminal_test_capture_active));
    return r;
}

GuidanceResult guidance_update(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    return guidance_update_with_terminal_model(g,t,state,plan,p,aero,cfg,NULL);
}


GuidanceCommand guidance_entry_reference_step(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const GuidanceSettings*s,double bank,double aoa,double dt){
    GuidanceCommand command=atmospheric(t,t->ground_track_heading,bank,v,0.0,false,PROFILE_ENTRY);
    command.heading_control_enabled=false;command.has_target_aoa=true;
    command.target_aoa=aoa;command.target_pitch=t->flight_path_angle+aoa;
    GuidanceResult result=stabilized(g,result_make(PHASE_ENTRY_ENERGY,command,"",NULL),t,v,s,dt);
    command=result.command;guidance_result_clear(&result);return command;
}
