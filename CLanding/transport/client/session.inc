static double monotonic_seconds(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void set_error(char *out, size_t out_size, const char *fmt, ...) {
    if (!out || out_size == 0) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(out, out_size, fmt, args);
    va_end(args);
}

static bool rpc_failed(KrpcCNanoClient *client, krpc_error_t code, const char *name,
                       char *error, size_t error_size) {
    if (client) { client->total_rpc_calls++; client->total_wire_requests++; }
    if (code == KRPC_OK) return false;
#ifdef KRPC_ERROR_MESSAGES
    const char *message = krpc_get_error_message();
#else
    const char *message = "";
#endif
    KrpcCNanoTransportStatus transport = client && client->connection
        ? krpc_cnano_transport_last_status(client->connection)
        : KRPC_CNANO_TRANSPORT_INVALID;
    set_error(error, error_size, "%s failed: %s%s%s (transport=%s)",
              name ? name : "kRPC", krpc_get_error(code),
              message && *message ? ": " : "", message && *message ? message : "",
              krpc_cnano_transport_status_string(transport));
    return true;
}

#define RPC_REQUIRED(client, expression, error, error_size) \
    do { if (rpc_failed((client), (expression), #expression, (error), (error_size))) return false; } while (0)
#define RPC_OPTIONAL(client, expression) \
    do { krpc_error_t _e = (expression); (client)->total_rpc_calls++; (client)->total_wire_requests++; (void)_e; } while (0)

static Vector3 tuple_to_canonical(krpc_tuple_double_double_double_t value) {
    /* Preserve the old bridge's kRPC -> internal axis boundary: x,z,y. */
    return v3(value.e0, value.e2, value.e1);
}

static krpc_tuple_double_double_double_t canonical_to_tuple(Vector3 value) {
    krpc_tuple_double_double_double_t tuple;
    tuple.e0 = value.x;
    tuple.e1 = value.z;
    tuple.e2 = value.y;
    return tuple;
}

static double finite_or(double value, double fallback) {
    return isfinite(value) ? value : fallback;
}


/* SpaceCenter service/procedure IDs are pinned by the vendored C-Nano 0.6.0
   generated header. Keep them explicit here so high-rate telemetry can use the
   standard kRPC multi-call Request wire format instead of one serial round trip
   per property. */
enum {
    PROC_UT=51,
    PROC_FLIGHT_SURFACE_ALTITUDE=465,
    PROC_CELESTIALBODY_PRESSURE_AT=242,
    PROC_CELESTIALBODY_DENSITY_AT=241,
    PROC_FLIGHT_TRUE_AIR_SPEED=503,
    PROC_FLIGHT_HEADING=479,
    PROC_FLIGHT_PITCH=478,
    PROC_FLIGHT_ROLL=480,
    PROC_FLIGHT_AOA=506,
    PROC_FLIGHT_SIDESLIP=507,
    PROC_FLIGHT_DYNAMIC_PRESSURE=490,
    PROC_VESSEL_POSITION=636,
    PROC_VESSEL_VELOCITY=638,
    PROC_VESSEL_ANGULAR_VELOCITY=641,
    PROC_VESSEL_MASS=669,
    PROC_CONTROL_PITCH=403,
    PROC_CONTROL_ROLL=407,
    PROC_CONTROL_YAW=405,
    PROC_CONTROL_THROTTLE=399,
    PROC_VESSEL_ROTATION=639,
    PROC_FLIGHT_SPEED_OF_SOUND=500,
    PROC_FLIGHT_STATIC_PRESSURE=492,
    PROC_FLIGHT_G_FORCE=463,
    PROC_VESSEL_DRY_MASS=670,
    PROC_VESSEL_AVAILABLE_THRUST=672,
    PROC_VESSEL_THRUST=671,
    PROC_VESSEL_AVAILABLE_TORQUE=684,
    PROC_VESSEL_MOMENT_OF_INERTIA=682,
    PROC_FLIGHT_LIFT=495,
    PROC_FLIGHT_DRAG=496,
    PROC_FLIGHT_CENTER_OF_MASS=475,
    PROC_CONTROL_GEAR=371,
    PROC_CONTROL_BRAKES=379,
    PROC_CONTROL_GET_ACTION_GROUP=347,
    PROC_CONTROL_RCS=361,
    PROC_CONTROL_SAS=355,
    PROC_AUTOPILOT_ENGAGED=111,
    PROC_AUTOPILOT_ERROR=138,
    PROC_CONTROL_SPEED_MODE=359,
    PROC_VESSEL_SITUATION=646,
    PROC_ORBIT_APOAPSIS_ALTITUDE=561,
    PROC_ORBIT_PERIAPSIS_ALTITUDE=562,
    PROC_ORBIT_PERIOD=567,
    PROC_CONTROL_SET_PITCH=404,
    PROC_CONTROL_SET_ROLL=408,
    PROC_CONTROL_SET_YAW=406,
    PROC_CONTROL_SET_THROTTLE=400,
    PROC_CONTROL_SET_WHEEL_STEERING=424,
    PROC_CONTROL_SET_RCS=362,
    PROC_CONTROL_SET_SAS=356,
    PROC_AUTOPILOT_SET_ENGAGED=112,
    PROC_AUTOPILOT_SET_AUTOTUNE=159,
    PROC_ENGINE_SET_ACTIVE=817,
    PROC_ENGINE_SET_THROTTLE=834,
    PROC_ENGINE_SET_INDEPENDENT_THROTTLE=837
};

typedef enum {
    BATCH_NONE,
    BATCH_DOUBLE,
    BATCH_FLOAT,
    BATCH_BOOL,
    BATCH_ENUM,
    BATCH_TUPLE3,
    BATCH_TUPLE4,
    BATCH_TORQUE
} BatchReturnKind;

typedef struct {
    krpc_call_t call;
    krpc_argument_t arguments[3];
    uint64_t object_arguments[3];
    uint32_t uint_arguments[3];
    float float_arguments[3];
    double double_arguments[3];
    bool bool_arguments[3];
    BatchReturnKind kind;
    void *target;
    bool required;
} BatchItem;

typedef struct {
    BatchItem items[KRPC_CNANO_BATCH_MAX];
    size_t count;
} ClientBatch;

static bool batch_add(ClientBatch *batch,uint32_t procedure,BatchReturnKind kind,void *target,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=kind;item->target=target;item->required=required;
    return krpc_call(&item->call,2,procedure,0,item->arguments)==KRPC_OK;
}

static bool batch_add_object(ClientBatch *batch,uint32_t procedure,krpc_object_t instance,BatchReturnKind kind,void *target,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=kind;item->target=target;item->required=required;item->object_arguments[0]=instance;
    return krpc_call(&item->call,2,procedure,1,item->arguments)==KRPC_OK&&
        krpc_add_argument(&item->call,0,&krpc_encode_callback_uint64,&item->object_arguments[0])==KRPC_OK;
}

static bool batch_add_two_objects(ClientBatch *batch,uint32_t procedure,krpc_object_t first,krpc_object_t second,BatchReturnKind kind,void *target,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=kind;item->target=target;item->required=required;
    item->object_arguments[0]=first;item->object_arguments[1]=second;
    return krpc_call(&item->call,2,procedure,2,item->arguments)==KRPC_OK&&
        krpc_add_argument(&item->call,0,&krpc_encode_callback_uint64,&item->object_arguments[0])==KRPC_OK&&
        krpc_add_argument(&item->call,1,&krpc_encode_callback_uint64,&item->object_arguments[1])==KRPC_OK;
}

static bool batch_add_object_uint(ClientBatch *batch,uint32_t procedure,krpc_object_t instance,uint32_t value,BatchReturnKind kind,void *target,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=kind;item->target=target;item->required=required;
    item->object_arguments[0]=instance;item->uint_arguments[1]=value;
    return krpc_call(&item->call,2,procedure,2,item->arguments)==KRPC_OK&&
        krpc_add_argument(&item->call,0,&krpc_encode_callback_uint64,&item->object_arguments[0])==KRPC_OK&&
        krpc_add_argument(&item->call,1,&krpc_encode_callback_uint32,&item->uint_arguments[1])==KRPC_OK;
}

static bool batch_add_object_double(ClientBatch *batch,uint32_t procedure,krpc_object_t instance,double value,BatchReturnKind kind,void *target,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=kind;item->target=target;item->required=required;item->object_arguments[0]=instance;item->double_arguments[1]=value;
    return krpc_call(&item->call,2,procedure,2,item->arguments)==KRPC_OK&&
        krpc_add_argument(&item->call,0,&krpc_encode_callback_uint64,&item->object_arguments[0])==KRPC_OK&&
        krpc_add_argument(&item->call,1,&krpc_encode_callback_double,&item->double_arguments[1])==KRPC_OK;
}

static bool batch_add_object_float(ClientBatch *batch,uint32_t procedure,krpc_object_t instance,float value,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=BATCH_NONE;item->required=required;item->object_arguments[0]=instance;item->float_arguments[1]=value;
    return krpc_call(&item->call,2,procedure,2,item->arguments)==KRPC_OK&&
        krpc_add_argument(&item->call,0,&krpc_encode_callback_uint64,&item->object_arguments[0])==KRPC_OK&&
        krpc_add_argument(&item->call,1,&krpc_encode_callback_float,&item->float_arguments[1])==KRPC_OK;
}

static bool batch_add_object_bool(ClientBatch *batch,uint32_t procedure,krpc_object_t instance,bool value,bool required){
    if(!batch||batch->count>=KRPC_CNANO_BATCH_MAX)return false;
    BatchItem *item=&batch->items[batch->count++];memset(item,0,sizeof(*item));
    item->kind=BATCH_NONE;item->required=required;item->object_arguments[0]=instance;item->bool_arguments[1]=value;
    return krpc_call(&item->call,2,procedure,2,item->arguments)==KRPC_OK&&
        krpc_add_argument(&item->call,0,&krpc_encode_callback_uint64,&item->object_arguments[0])==KRPC_OK&&
        krpc_add_argument(&item->call,1,&krpc_encode_callback_bool,&item->bool_arguments[1])==KRPC_OK;
}

static krpc_error_t batch_decode_item(const BatchItem *item,krpc_result_t *result){
    pb_istream_t stream;if(!item||!result||krpc_get_return_value(result,&stream)!=KRPC_OK)return KRPC_ERROR_DECODING_FAILED;
    switch(item->kind){
        case BATCH_NONE:return KRPC_OK;
        case BATCH_DOUBLE:return krpc_decode_double(&stream,(double *)item->target);
        case BATCH_FLOAT:return krpc_decode_float(&stream,(float *)item->target);
        case BATCH_BOOL:return krpc_decode_bool(&stream,(bool *)item->target);
        case BATCH_ENUM:return krpc_decode_enum(&stream,(krpc_enum_t *)item->target);
        case BATCH_TUPLE3:return krpc_decode_tuple_double_double_double(&stream,(krpc_tuple_double_double_double_t *)item->target);
        case BATCH_TUPLE4:return krpc_decode_tuple_double_double_double_double(&stream,(krpc_tuple_double_double_double_double_t *)item->target);
        case BATCH_TORQUE:return krpc_decode_tuple_tuple_double_double_double_tuple_double_double_double(&stream,(krpc_tuple_tuple_double_double_double_tuple_double_double_double_t *)item->target);
        default:return KRPC_ERROR_DECODING_FAILED;
    }
}

static bool batch_run(KrpcCNanoClient *client,ClientBatch *batch,const char *label,char *error,size_t error_size){
    if(!client||!batch||batch->count==0){set_error(error,error_size,"%s batch is empty",label?label:"C-Nano");return false;}
    krpc_schema_ProcedureCall calls[KRPC_CNANO_BATCH_MAX];
    krpc_result_t results[KRPC_CNANO_BATCH_MAX];
    bool failed[KRPC_CNANO_BATCH_MAX];memset(failed,0,sizeof(failed));
    size_t initialized=0;
    for(size_t i=0;i<batch->count;i++){
        results[i]=(krpc_result_t)KRPC_RESULT_INIT_DEFAULT;
        if(krpc_init_result(&results[i])!=KRPC_OK){
            set_error(error,error_size,"Failed to initialize %s batch result",label?label:"C-Nano");
            for(size_t j=0;j<initialized;j++)(void)krpc_free_result(&results[j]);
            return false;
        }
        initialized++;
        calls[i]=batch->items[i].call.message;
    }
    krpc_error_t code=krpc_cnano_invoke_batch(client->connection,calls,results,failed,batch->count);
    client->total_rpc_calls+=(unsigned)batch->count;
    client->total_wire_requests++;
    if(code==KRPC_OK){
        for(size_t i=0;i<batch->count;i++){
            if(failed[i]){
                if(batch->items[i].required){
                    set_error(error,error_size,"%s batch procedure %u failed",label?label:"RPC",batch->items[i].call.message.procedure_id);
                    code=KRPC_ERROR_RPC_FAILED;break;
                }
                continue;
            }
            krpc_error_t decoded=batch_decode_item(&batch->items[i],&results[i]);
            if(decoded!=KRPC_OK&&batch->items[i].required){code=decoded;break;}
        }
    }
    for(size_t i=0;i<initialized;i++)(void)krpc_free_result(&results[i]);
    if(code!=KRPC_OK){
        if(!error||!error[0])set_error(error,error_size,"%s C-Nano batch failed: %s (transport=%s)",label?label:"RPC",krpc_get_error(code),krpc_cnano_transport_status_string(krpc_cnano_transport_last_status(client->connection)));
        return false;
    }
    return true;
}

static void reset_rate_filters(KrpcCNanoClient *client) {
    client->has_previous_attitude = false;
    client->has_previous_course = false;
    lowpass_reset(&client->pitch_rate_filter);
    lowpass_reset(&client->roll_rate_filter);
    lowpass_reset(&client->heading_rate_filter);
    lowpass_reset(&client->angle_of_attack_rate_filter);
    lowpass_reset(&client->course_rate_filter);
}

static bool load_planet(KrpcCNanoClient *client, char *error, size_t error_size) {
    PlanetModel *planet = &client->planet;
    memset(planet, 0, sizeof(*planet));
    planet->atmosphere_adiabatic_index = 1.4;

    char *name = NULL;
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_Name(client->connection, &name, client->body), error, error_size);
    if (name) {
        snprintf(planet->name, sizeof(planet->name), "%s", name);
        krpc_free(name);
    } else {
        snprintf(planet->name, sizeof(planet->name), "Unknown");
    }
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_EquatorialRadius(client->connection, &planet->radius, client->body), error, error_size);
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_GravitationalParameter(client->connection, &planet->gravitational_parameter, client->body), error, error_size);
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_RotationalSpeed(client->connection, &planet->rotational_speed, client->body), error, error_size);
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_AtmosphereDepth(client->connection, &planet->atmosphere_depth, client->body), error, error_size);
    RPC_REQUIRED(client, krpc_SpaceCenter_UT(client->connection, &planet->epoch_ut), error, error_size);

    krpc_tuple_double_double_double_t north = {0}, prime = {0};
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_SurfacePosition(client->connection, &north, client->body, 90.0, 0.0, client->body_nonrotating), error, error_size);
    RPC_REQUIRED(client, krpc_SpaceCenter_CelestialBody_SurfacePosition(client->connection, &prime, client->body, 0.0, 0.0, client->body_nonrotating), error, error_size);
    planet->north_axis = vnorm(tuple_to_canonical(north), v3(0, 0, 1));
    planet->prime_meridian_at_epoch = vnorm(tuple_to_canonical(prime), v3(1, 0, 0));

    if(planet->atmosphere_depth>0.0){
        ClientBatch batch={0};
        for(int i=0;i<CNANO_ATMOSPHERE_SAMPLES&&planet->atmosphere_sample_count<LANDER_ATMOSPHERE_SAMPLE_MAX;i++){
            size_t n=planet->atmosphere_sample_count++;
            double altitude=planet->atmosphere_depth*(double)i/(double)(CNANO_ATMOSPHERE_SAMPLES-1);
            planet->atmosphere_altitude[n]=altitude;
            if(batch.count+2>KRPC_CNANO_BATCH_MAX){
                if(!batch_run(client,&batch,"atmosphere table",error,error_size))return false;
                memset(&batch,0,sizeof(batch));
            }
            if(!batch_add_object_double(&batch,PROC_CELESTIALBODY_PRESSURE_AT,client->body,altitude,BATCH_DOUBLE,&planet->atmosphere_pressure[n],true)||
               !batch_add_object_double(&batch,PROC_CELESTIALBODY_DENSITY_AT,client->body,altitude,BATCH_DOUBLE,&planet->atmosphere_density[n],true)){
                set_error(error,error_size,"Could not construct C-Nano atmosphere table batch");return false;
            }
        }
        if(batch.count&&!batch_run(client,&batch,"atmosphere table",error,error_size))return false;
        for(size_t n=0;n<planet->atmosphere_sample_count;n++){
            planet->atmosphere_pressure[n]=fmax(0.0,finite_or(planet->atmosphere_pressure[n],0.0));
            planet->atmosphere_density[n]=fmax(0.0,finite_or(planet->atmosphere_density[n],0.0));
        }
    }
    planet->surface_density = planet->atmosphere_sample_count
        ? planet->atmosphere_density[0] : 0.0;
    return true;
}

static void inventory_engines(KrpcCNanoClient *client) {
    client->engine_count = 0;
    client->commandable_thrust = 0.0;
    krpc_SpaceCenter_Parts_t parts = KRPC_NULL;
    krpc_list_object_t engines = KRPC_NULL_LIST;
    krpc_error_t e = krpc_SpaceCenter_Vessel_Parts(client->connection, &parts, client->vessel);
    client->total_rpc_calls++; client->total_wire_requests++;
    if (e != KRPC_OK || parts == KRPC_NULL) return;
    e = krpc_SpaceCenter_Parts_Engines(client->connection, &engines, parts);
    client->total_rpc_calls++; client->total_wire_requests++;
    if (e != KRPC_OK) return;

    for (size_t i = 0; i < engines.size && client->engine_count < CNANO_ENGINE_LIMIT; ++i) {
        krpc_SpaceCenter_Engine_t engine = engines.items[i];
        bool locked = true, fuel = false, active = false;
        float available = 0.0f;
        krpc_error_t e_locked = krpc_SpaceCenter_Engine_ThrottleLocked(client->connection, &locked, engine); client->total_rpc_calls++; client->total_wire_requests++;
        krpc_error_t e_fuel = krpc_SpaceCenter_Engine_HasFuel(client->connection, &fuel, engine); client->total_rpc_calls++; client->total_wire_requests++;
        krpc_error_t e_available = krpc_SpaceCenter_Engine_AvailableThrust(client->connection, &available, engine); client->total_rpc_calls++; client->total_wire_requests++;
        krpc_error_t e_active = krpc_SpaceCenter_Engine_Active(client->connection, &active, engine); client->total_rpc_calls++; client->total_wire_requests++;
        if (e_locked != KRPC_OK || e_fuel != KRPC_OK || e_available != KRPC_OK || e_active != KRPC_OK ||
            locked || !fuel || !(available > 0.0f)) continue;
        CommandableEngine *entry = &client->engines[client->engine_count++];
        memset(entry, 0, sizeof(*entry));
        entry->object = engine;
        entry->initially_active = active;
        entry->available_thrust = available;
        client->commandable_thrust += available;
    }
    KRPC_FREE_LIST(engines);
}

static KrpcCNanoClient *allocate_client(char *error,size_t error_size){
    KrpcCNanoClient *client=calloc(1,sizeof(*client));
    if(!client){set_error(error,error_size,"Out of memory creating C-Nano client");return NULL;}
    client->last_medium_ut=-DBL_MAX;client->last_slow_ut=-DBL_MAX;
    client->pitch_rate_filter.tc=.32;client->roll_rate_filter.tc=.28;
    client->heading_rate_filter.tc=.38;client->angle_of_attack_rate_filter.tc=.32;client->course_rate_filter.tc=.30;
    return client;
}

static void discard_setup_client(KrpcCNanoClient *client){
    if(!client)return;
    if(client->connection&&krpc_cnano_transport_is_open(client->connection))(void)krpc_close(client->connection);
    client->connected=false;free(client);
}

static KrpcCNanoClient *finish_client_open(KrpcCNanoClient *client,
                                           const LandingConfiguration *configuration,
                                           const KrpcCNanoTransportConfig *transport_config,
                                           const char *transport_name,
                                           char *error,size_t error_size){
    if(!client||!configuration||!transport_config){set_error(error,error_size,"C-Nano open arguments are incomplete");discard_setup_client(client);return NULL;}
    krpc_error_t open_error=krpc_open(&client->connection,transport_config);
    if(open_error!=KRPC_OK){
        KrpcCNanoTransportStatus transport_status=client->connection?krpc_cnano_transport_last_status(client->connection):KRPC_CNANO_TRANSPORT_INVALID;
        set_error(error,error_size,"krpc_open failed: %s (transport=%s)",krpc_get_error(open_error),krpc_cnano_transport_status_string(transport_status));
        discard_setup_client(client);return NULL;
    }
    if(rpc_failed(client,krpc_connect(client->connection,configuration->connection.client_name),"krpc_connect",error,error_size)){discard_setup_client(client);return NULL;}
    client->connected=true;
    snprintf(client->transport_name,sizeof(client->transport_name),"%s",transport_name&&*transport_name?transport_name:"custom");

    if(rpc_failed(client,krpc_SpaceCenter_ActiveVessel(client->connection,&client->vessel),"SpaceCenter.ActiveVessel",error,error_size)||client->vessel==KRPC_NULL){discard_setup_client(client);return NULL;}
    if(rpc_failed(client,krpc_SpaceCenter_Vessel_Control(client->connection,&client->control,client->vessel),"Vessel.Control",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_AutoPilot(client->connection,&client->autopilot,client->vessel),"Vessel.AutoPilot",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_Orbit(client->connection,&client->orbit,client->vessel),"Vessel.Orbit",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Orbit_Body(client->connection,&client->body,client->orbit),"Orbit.Body",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_CelestialBody_NonRotatingReferenceFrame(client->connection,&client->body_nonrotating,client->body),"Body.NonRotatingReferenceFrame",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_ReferenceFrame(client->connection,&client->vessel_frame,client->vessel),"Vessel.ReferenceFrame",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_SurfaceReferenceFrame(client->connection,&client->surface_frame,client->vessel),"Vessel.SurfaceReferenceFrame",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_Flight(client->connection,&client->flight_surface,client->vessel,client->surface_frame),"Vessel.Flight(surface)",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_Flight(client->connection,&client->flight_body,client->vessel,client->body_nonrotating),"Vessel.Flight(body-non-rotating)",error,error_size)||
       rpc_failed(client,krpc_SpaceCenter_Vessel_Flight(client->connection,&client->flight_vessel,client->vessel,client->vessel_frame),"Vessel.Flight(vessel)",error,error_size)){
        discard_setup_client(client);return NULL;
    }

    char *vessel_name=NULL;
    if(!rpc_failed(client,krpc_SpaceCenter_Vessel_Name(client->connection,&vessel_name,client->vessel),"Vessel.Name",error,error_size)&&vessel_name){
        snprintf(client->vessel_name,sizeof(client->vessel_name),"%s",vessel_name);krpc_free(vessel_name);
    }else if(!client->vessel_name[0])snprintf(client->vessel_name,sizeof(client->vessel_name),"Active Vessel");
    if(!load_planet(client,error,error_size)){discard_setup_client(client);return NULL;}

    /* AutoPilot configuration and oscillation-detector state persist across
       engagements and even save reloads inside the running KSP process. Each
       native landing session must start from a known controller state or a
       previous test's up-reference/tuning can pin pitch/roll in the next run. */
    if(rpc_failed(client,krpc_SpaceCenter_AutoPilot_Reset(client->connection,client->autopilot),
                  "AutoPilot.Reset",error,error_size)){discard_setup_client(client);return NULL;}
    client->cached_autopilot_engaged=false;client->has_autopilot_engaged=true;

    double met=NAN,now_ut=client->planet.epoch_ut;
    krpc_error_t met_error=krpc_SpaceCenter_Vessel_MET(client->connection,&met,client->vessel);client->total_rpc_calls++;client->total_wire_requests++;
    if(met_error==KRPC_OK&&isfinite(met)&&met>=0.0&&isfinite(now_ut))snprintf(client->flight_key,sizeof(client->flight_key),"launch-ut:%.3f",now_ut-met);
    inventory_engines(client);reset_rate_filters(client);return client;
}

KrpcCNanoClient *krpc_cnano_client_open_transport(const LandingConfiguration *configuration,
                                                  const KrpcCNanoTransportConfig *transport_config,
                                                  const char *transport_name,
                                                  char *error,size_t error_size){
    if(!configuration||!transport_config){set_error(error,error_size,"C-Nano custom transport configuration is missing");return NULL;}
    KrpcCNanoClient *client=allocate_client(error,error_size);if(!client)return NULL;
    return finish_client_open(client,configuration,transport_config,transport_name,error,error_size);
}

KrpcCNanoClient *krpc_cnano_client_open(const LandingConfiguration *configuration,
                                        char *error,size_t error_size){
    if(!configuration){set_error(error,error_size,"C-Nano configuration is missing");return NULL;}
#ifndef _WIN32
    KrpcCNanoClient *client=allocate_client(error,error_size);if(!client)return NULL;
    const char *rpc_host=getenv("KSP_LANDER_RPC_HOST");
    const char *rpc_port_text=getenv("KSP_LANDER_RPC_PORT");
    if((rpc_host&&*rpc_host)||(rpc_port_text&&*rpc_port_text)){
        if(!rpc_host||!*rpc_host)rpc_host="127.0.0.1";
        char *end=NULL;long rpc_port=rpc_port_text&&*rpc_port_text?strtol(rpc_port_text,&end,10):50000;
        if(rpc_port<1||rpc_port>65535||(rpc_port_text&&*rpc_port_text&&(!end||*end))){
            set_error(error,error_size,"Invalid kRPC RPC port: %s",rpc_port_text?rpc_port_text:"");free(client);return NULL;
        }
        krpc_cnano_posix_tcp_init(&client->tcp,rpc_host,(int)rpc_port,configuration->connection.timeout_ms);
        KrpcCNanoTransportConfig config=krpc_cnano_transport_config(
            &client->transport,krpc_cnano_posix_tcp_ops(),&client->tcp,
            configuration->connection.timeout_ms);
        char name[sizeof(client->transport_name)];snprintf(name,sizeof(name),"tcp:%s:%ld",rpc_host,rpc_port);
        return finish_client_open(client,configuration,&config,name,error,error_size);
    }
    const char *path=configuration->connection.serial_port;
    const char *override=getenv("KSP_LANDER_SERIAL_PORT");if(override&&*override)path=override;
    if(!path||!*path){set_error(error,error_size,"No kRPC endpoint is configured");free(client);return NULL;}
    krpc_cnano_posix_serial_init(&client->serial,path,configuration->connection.timeout_ms,configuration->connection.baud_rate,true);
    KrpcCNanoTransportConfig config=krpc_cnano_transport_config(
        &client->transport,krpc_cnano_posix_serial_ops(),&client->serial,
        configuration->connection.timeout_ms);
    char name[sizeof(client->transport_name)];snprintf(name,sizeof(name),"serial:%s",path);
    return finish_client_open(client,configuration,&config,name,error,error_size);
#else
    set_error(error,error_size,"The native C-Nano production transport is currently POSIX-only");return NULL;
#endif
}

const PlanetModel *krpc_cnano_client_planet(const KrpcCNanoClient *client) {
    return client ? &client->planet : NULL;
}
const char *krpc_cnano_client_vessel(const KrpcCNanoClient *client) {
    return client ? client->vessel_name : "";
}
const char *krpc_cnano_client_transport_name(const KrpcCNanoClient *client) {
    return client ? client->transport_name : "";
}
const char *krpc_cnano_client_library_version(const KrpcCNanoClient *client) {
    (void)client; return "krpc-cnano-0.6.0";
}
const char *krpc_cnano_client_flight_key(const KrpcCNanoClient *client) {
    return client ? client->flight_key : "";
}

static const char *situation_name(krpc_SpaceCenter_VesselSituation_t situation) {
    switch (situation) {
        case KRPC_SPACECENTER_VESSELSITUATION_PRELAUNCH: return "pre-launch";
        case KRPC_SPACECENTER_VESSELSITUATION_ORBITING: return "orbiting";
        case KRPC_SPACECENTER_VESSELSITUATION_SUBORBITAL: return "sub-orbital";
        case KRPC_SPACECENTER_VESSELSITUATION_ESCAPING: return "escaping";
        case KRPC_SPACECENTER_VESSELSITUATION_FLYING: return "flying";
        case KRPC_SPACECENTER_VESSELSITUATION_LANDED: return "landed";
        case KRPC_SPACECENTER_VESSELSITUATION_SPLASHED: return "splashed";
        case KRPC_SPACECENTER_VESSELSITUATION_DOCKED: return "docked";
        default: return "unknown";
    }
}

