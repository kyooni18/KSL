#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


GuidanceResult guidance_update_with_terminal_model(GuidanceMachine*g,
        const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TerminalModel *terminal_model){
    /* Keep the public update in the configured runway frame. Terminal guidance
       owns reciprocal-runway preview, selection, reframing, and commitment in
       terminal_guidance_selected(). Pre-framing here makes a selected RW27
       appear as a new primary runway on the next preview and aliases the stored
       runway_end_index, causing the MM304/MM305 inlet to flip ends every cycle. */
    GuidanceResult r=guidance_update_impl(g,t,state,plan,p,aero,cfg,terminal_model);
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
