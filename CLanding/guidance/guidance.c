#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


GuidanceResult guidance_update(GuidanceMachine*g,const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    /* Keep the public update in the configured runway frame. Terminal guidance
       owns reciprocal-runway preview, selection, reframing, and commitment in
       terminal_guidance_selected(). Pre-framing here makes a selected RW27
       appear as a new primary runway on the next preview and aliases the stored
       runway_end_index, causing the MM304/MM305 inlet to flip ends every cycle. */
    GuidanceResult r=guidance_update_impl(g,t,state,plan,p,aero,cfg);
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

bool guidance_install_entry_topology(GuidanceMachine*g,const EntryTopologyPlan*top,double ut){
    if(!g||!top||!top->valid||!top->inlet.valid||!isfinite(ut)||top->reversal_ut<=ut||
       top->capture_ut<=top->reversal_ut||g->entry_topology.valid||g->entry_control_reversals||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed)return false;
    bool replaceable_ordinary_reversal=g->entry_reversal_scheduled&&
        !g->entry_reversal_is_final&&top->first_sign*g->s_turn_sign>0.0;
    if(g->entry_reversal_scheduled&&!replaceable_ordinary_reversal)return false;
    /* The propagated topology may contain a shaping S-turn reversal followed by a
       distinct terminal 90 deg turn. Install the first event as nonfinal; the live
       topology executor owns the later measured-radius terminal-turn release and
       keeps both events tied to the same fixed TAEM inlet contract. */
    bool separate_terminal_turn=isfinite(top->terminal_turn_ut)&&top->terminal_turn_ut>top->reversal_ut+1.0;
    g->entry_topology=*top;g->taem_interface_target=top->inlet;
    g->entry_topology_capture_good_duration=0.0;
    g->entry_topology_heading_locked=false;
    request_side(g,top->first_sign,ut);
    EntryControlPlan plan={.valid=true,.planned_ut=top->planned_ut,.target_bank=top->first_sign*top->first_bank,
        .target_aoa=top->first_aoa,.target_heading=top->inlet.course,.bank_cap=fmax(top->first_bank,top->turn_bank),
        .target_turn_radius=INFINITY,.segment_duration=top->capture_ut+30.0-top->planned_ut,
        .cost=top->cost,.taem_range_error=top->position_error,.taem_speed=top->capture_speed,.taem_energy_error=NAN,
        .has_planned_reversal=true,.planned_reversal_is_final=!separate_terminal_turn,.planned_reversal_ut=top->reversal_ut,
        .planned_reversal_range=hypot(top->reversal_along,top->reversal_cross),
        .planned_reversal_sign=-top->first_sign,.predicted_reversals=1};
    control_plan_assign_lineage(g,&plan,&g->entry_s_turn_plan);
    g->entry_s_turn_plan=plan;g->entry_control_plan_valid=true;
    entry_program_commit_planned_reversal(g,&plan,ut,replaceable_ordinary_reversal);
    g->entry_planning_needed=false;g->entry_committed_infeasible=false;
    return true;
}

GuidanceCommand guidance_entry_reference_step(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const GuidanceSettings*s,double bank,double aoa,double dt){
    GuidanceCommand command=atmospheric(t,t->ground_track_heading,bank,v,0.0,false,PROFILE_ENTRY);
    command.heading_control_enabled=false;command.has_target_aoa=true;
    command.target_aoa=aoa;command.target_pitch=t->flight_path_angle+aoa;
    GuidanceResult result=stabilized(g,result_make(PHASE_ENTRY_ENERGY,command,"",NULL),t,v,s,dt);
    command=result.command;guidance_result_clear(&result);return command;
}
