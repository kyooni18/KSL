bool krpc_cnano_client_save(KrpcCNanoClient *client, const char *name, char *error, size_t error_size) {
    if (!client || !client->connected || !name || !*name) { set_error(error, error_size, "Save name is empty"); return false; }
    RPC_REQUIRED(client, krpc_SpaceCenter_Save(client->connection, name), error, error_size); return true;
}

static bool safe_flush_batch(KrpcCNanoClient *client,ClientBatch *batch){
    if(!batch||batch->count==0)return true;
    char ignored[256]={0};bool ok=batch_run(client,batch,"safe control",ignored,sizeof(ignored));
    memset(batch,0,sizeof(*batch));return ok;
}

static void set_paused_triplet(KrpcCNanoClient *client, bool paused){
    if(!client||!client->connected||client->pause_triplet_sent)return;
    /* Alternate the property so KSP observes the transition even if one
       setter is lost during the escape/scene-transition race. */
    RPC_OPTIONAL(client, krpc_KRPC_set_Paused(client->connection, paused));
    struct timespec pause_gap={.tv_sec=0,.tv_nsec=200000000L};
    (void)nanosleep(&pause_gap,NULL);
    RPC_OPTIONAL(client, krpc_KRPC_set_Paused(client->connection, !paused));
    (void)nanosleep(&pause_gap,NULL);
    RPC_OPTIONAL(client, krpc_KRPC_set_Paused(client->connection, paused));
    client->pause_triplet_sent=true;
}

void krpc_cnano_client_safe(KrpcCNanoClient *client){
    if(!client||!client->connected)return;
    ClientBatch batch={0};
#define SAFE_ADD(expr) do{if(!(expr)){(void)safe_flush_batch(client,&batch);goto safe_done;}}while(0)
    SAFE_ADD(batch_add_object_bool(&batch,PROC_AUTOPILOT_SET_ENGAGED,client->autopilot,false,false));
    SAFE_ADD(batch_add_object_bool(&batch,PROC_CONTROL_SET_SAS,client->control,false,false));
    SAFE_ADD(batch_add_object_float(&batch,PROC_CONTROL_SET_PITCH,client->control,0.0f,false));
    SAFE_ADD(batch_add_object_float(&batch,PROC_CONTROL_SET_ROLL,client->control,0.0f,false));
    SAFE_ADD(batch_add_object_float(&batch,PROC_CONTROL_SET_YAW,client->control,0.0f,false));
    SAFE_ADD(batch_add_object_float(&batch,PROC_CONTROL_SET_THROTTLE,client->control,0.0f,false));
    SAFE_ADD(batch_add_object_float(&batch,PROC_CONTROL_SET_WHEEL_STEERING,client->control,0.0f,false));
    SAFE_ADD(batch_add_object_bool(&batch,PROC_CONTROL_SET_RCS,client->control,false,false));

    for(size_t i=0;i<client->engine_count;i++){
        CommandableEngine *engine=&client->engines[i];
        size_t needed=(engine->independent_owned?2u:0u)+((engine->activated_by_client&&!engine->initially_active)?1u:0u);
        if(needed&&batch.count+needed>KRPC_CNANO_BATCH_MAX&&!safe_flush_batch(client,&batch))goto safe_done;
        if(engine->independent_owned){
            SAFE_ADD(batch_add_object_float(&batch,PROC_ENGINE_SET_THROTTLE,engine->object,0.0f,false));
            SAFE_ADD(batch_add_object_bool(&batch,PROC_ENGINE_SET_INDEPENDENT_THROTTLE,engine->object,false,false));
            engine->independent_owned=false;
        }
        if(engine->activated_by_client&&!engine->initially_active){
            SAFE_ADD(batch_add_object_bool(&batch,PROC_ENGINE_SET_ACTIVE,engine->object,false,false));
            engine->activated_by_client=false;
        }
    }
    (void)safe_flush_batch(client,&batch);
safe_done:
#undef SAFE_ADD
    /* C-Nano shutdown is also the escape path from automated control. */
    set_paused_triplet(client, true);
    client->cached_rcs=false;client->has_rcs=true;
    client->cached_sas=false;client->has_sas=true;
    client->cached_autopilot_engaged=false;client->has_autopilot_engaged=true;
    client->cached_command_throttle=0.0;client->has_command_throttle=true;
    client->cached_command_wheel=0.0;client->has_command_wheel=true;
}

KrpcCNanoBudget krpc_cnano_client_budget(const KrpcCNanoClient *client) {
    KrpcCNanoBudget budget = {0};
    if (!client) return budget;
    budget.read_calls = client->last_read_calls;
    budget.write_calls = client->last_apply_calls;
    budget.total_calls=client->total_rpc_calls;
    budget.read_wire_requests=client->last_read_wire_requests;
    budget.write_wire_requests=client->last_apply_wire_requests;
    budget.total_wire_requests=client->total_wire_requests;
    budget.last_read_seconds = client->last_read_seconds;
    budget.last_apply_seconds = client->last_apply_seconds;
    budget.read_calls_per_second = client->last_read_seconds > 1e-6 ? client->last_read_calls / client->last_read_seconds : 0.0;
    budget.apply_calls_per_second=client->last_apply_seconds>1e-6?client->last_apply_calls/client->last_apply_seconds:0.0;
    budget.read_wire_requests_per_second=client->last_read_seconds>1e-6?client->last_read_wire_requests/client->last_read_seconds:0.0;
    budget.apply_wire_requests_per_second=client->last_apply_seconds>1e-6?client->last_apply_wire_requests/client->last_apply_seconds:0.0;
    budget.bytes_read = client->connection ? krpc_cnano_transport_bytes_read(client->connection) : 0;
    budget.bytes_written = client->connection ? krpc_cnano_transport_bytes_written(client->connection) : 0;
    return budget;
}

void krpc_cnano_client_close(KrpcCNanoClient *client) {
    if (!client) return;
    if (client->connected) krpc_cnano_client_safe(client);
    if (client->connection && krpc_cnano_transport_is_open(client->connection)) (void)krpc_close(client->connection);
    client->connected = false;
    free(client);
}
