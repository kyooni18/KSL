/*
 * MM304 prediction crosses the worker boundary as advisory evidence only.
 * Executable bank, AoA, side, phase and handoff decisions are rebuilt from the
 * newest navigation sample in entry_program_guidance().
 */
bool guidance_accept_entry_plan(GuidanceMachine*g,const GuidanceMachine*request,
        const GuidanceMachine*result,const Telemetry*t,const LandingConfiguration*cfg){
    (void)cfg;
    if(!g||!request||!result||!t)return false;
    if(result->phase!=request->phase||
       !g->automation_engaged||g->paused||g->aborted||g->phase!=PHASE_ENTRY_ENERGY||
       g->control_plan_sequence!=request->control_plan_sequence||
       g->entry_control_reversals!=request->entry_control_reversals)
        return false;

    double age=t->ut-request->previous_ut;
    if(!isfinite(age)||age<0.0)return false;

    g->entry_supervision_ut=request->previous_ut;
    g->entry_supervision_valid=result->entry_supervision_valid;
    g->entry_supervision_mode=result->entry_supervision_mode;
    g->entry_committed_infeasible=result->entry_committed_infeasible;
    g->entry_supervision_boundary_missed=result->entry_supervision_boundary_missed;

    if(result->entry_s_turn_plan.valid){
        g->entry_s_turn_plan.valid=true;
        g->entry_s_turn_plan.terminal_ready=result->entry_s_turn_plan.terminal_ready;
        g->entry_s_turn_plan.closest_distance=result->entry_s_turn_plan.closest_distance;
        g->entry_s_turn_plan.taem_range_error=result->entry_s_turn_plan.taem_range_error;
        g->entry_s_turn_plan.taem_speed=result->entry_s_turn_plan.taem_speed;
        g->entry_s_turn_plan.taem_energy_error=result->entry_s_turn_plan.taem_energy_error;
        g->entry_s_turn_plan.predicted_reversals=result->entry_s_turn_plan.predicted_reversals;
        g->entry_s_turn_plan.cost=result->entry_s_turn_plan.cost;
        g->entry_s_turn_plan.planned_ut=result->entry_s_turn_plan.planned_ut;
        g->entry_s_turn_plan.target_bank=result->entry_s_turn_plan.target_bank;
        g->entry_s_turn_plan.target_aoa=result->entry_s_turn_plan.target_aoa;
        g->entry_s_turn_plan.has_planned_reversal=result->entry_s_turn_plan.has_planned_reversal;
        g->entry_s_turn_plan.planned_reversal_ut=result->entry_s_turn_plan.planned_reversal_ut;
        g->entry_s_turn_plan.planned_reversal_sign=result->entry_s_turn_plan.planned_reversal_sign;
        g->entry_s_turn_plan.planned_reversal_is_final=result->entry_s_turn_plan.planned_reversal_is_final;
        g->entry_s_turn_plan.schedule_count=result->entry_s_turn_plan.schedule_count;
        memcpy(g->entry_s_turn_plan.schedule_ut,result->entry_s_turn_plan.schedule_ut,sizeof(g->entry_s_turn_plan.schedule_ut));
    }
    if(result->entry_policy_valid&&isfinite(result->entry_policy_bank_deg)){
        g->entry_policy_valid=true;
        g->entry_policy_bank_deg=result->entry_policy_bank_deg;
        g->entry_policy_aoa_deg=result->entry_policy_aoa_deg;
        g->entry_policy_ut=result->entry_policy_ut;
    }
    g->entry_planning_needed=false;
    return true;
}
