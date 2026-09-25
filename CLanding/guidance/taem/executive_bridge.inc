/* MM305 executive bridge.
   Converts live guidance state into the small public TAEM executive contract.
   Path geometry, energy pricing, and control remain in their own modules. */

static TaemExecInputs taem_exec_inputs_live_with_contract(
        const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        const TaemTerminalContract*terminal_contract,
        bool has_final_approach_override,bool final_approach_override){
    (void)t;(void)s;(void)p;(void)aero;(void)cfg;
    TaemExecInputs inputs={0};
    inputs.mm304_complete=g->entry_exec.entry_complete;

    /* Native MM305 owns one value-semantic route from commit through HAC exit.
       Final-only tests own the runway line directly.  Do not infer ownership from
       legacy HAC transition/candidate latches. */
    bool native_route=g->mm305_route_committed&&g->mm305_route.valid;
    inputs.terminal_path_selected=g->terminal_final_test_mode||native_route;
    inputs.terminal_path_captured=g->terminal_final_test_mode||native_route;
    inputs.final_intercept_ready=g->terminal_final_test_mode||
        (native_route&&g->mm305_hac_exit_reached);
    inputs.final_approach_ready=has_final_approach_override?
        final_approach_override:g->final_approach_captured;
    if(terminal_contract)inputs.terminal_contract=*terminal_contract;

    inputs.off_nominal_recovery_active=g->attitude_recovery;
    inputs.off_nominal_recovery_reason=g->attitude_recovery?
        TAEM_RECOVERY_ATTITUDE:TAEM_RECOVERY_NONE;
    return inputs;
}

bool taem_exec_enter(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,
        bool checkpoint_resume){
    if(g->taem_exec.initialized&&g->taem_exec.ownership_latched)return true;
    TaemExecObservation observation={
        .ut=t->ut,.relative_velocity=t->true_air_speed,.checkpoint_restart=checkpoint_resume
    };
    TaemExecInputs inputs=taem_exec_inputs_live_with_contract(
        g,t,s,p,aero,cfg,NULL,false,false);
    inputs.resume_mm305=checkpoint_resume;
    if(checkpoint_resume)inputs.mm304_complete=false;
    return taem_exec_initialize(&g->taem_exec,&observation,&inputs);
}

bool taem_exec_sync_with_contract(GuidanceMachine*g,const Telemetry*t,
        const GuidanceSettings*s,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,const TaemTerminalContract*terminal_contract,
        bool has_final_approach_override,bool final_approach_override){
    if(!g->taem_exec.initialized||!g->taem_exec.ownership_latched)return false;
    TaemExecObservation observation={
        .ut=t->ut,.relative_velocity=t->true_air_speed,.checkpoint_restart=false
    };
    TaemExecInputs inputs=taem_exec_inputs_live_with_contract(
        g,t,s,p,aero,cfg,terminal_contract,
        has_final_approach_override,final_approach_override);
    return taem_exec_update(&g->taem_exec,&observation,&inputs);
}

void taem_exec_sync(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,NULL,false,false);
}

