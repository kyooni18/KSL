#include "krpc_cnano_client.h"
#include "krpc_cnano_transport.h"
#include "krpc_cnano_batch.h"

#include <krpc_cnano.h>
#include <krpc_cnano/memory.h>
#include <krpc_cnano/encoder.h>
#include <krpc_cnano/decoder.h>
#include <krpc_cnano/services/krpc.h>
#include <krpc_cnano/services/space_center.h>

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define CNANO_ENGINE_LIMIT 32
#define CNANO_ATMOSPHERE_SAMPLES 71
#define CNANO_MEDIUM_PERIOD 0.25
#define CNANO_SLOW_PERIOD 1.0

typedef struct {
    krpc_SpaceCenter_Engine_t object;
    bool initially_active;
    bool activated_by_client;
    bool independent_owned;
    float available_thrust;
} CommandableEngine;

struct KrpcCNanoClient {
    KrpcCNanoTransport transport;
    KrpcCNanoPosixSerial serial;
    krpc_connection_t connection;
    bool connected;

    krpc_SpaceCenter_Vessel_t vessel;
    krpc_SpaceCenter_Control_t control;
    krpc_SpaceCenter_AutoPilot_t autopilot;
    krpc_SpaceCenter_Orbit_t orbit;
    krpc_SpaceCenter_CelestialBody_t body;
    krpc_SpaceCenter_ReferenceFrame_t body_nonrotating;
    krpc_SpaceCenter_ReferenceFrame_t vessel_frame;
    krpc_SpaceCenter_ReferenceFrame_t surface_frame;
    krpc_SpaceCenter_Flight_t flight_surface;
    krpc_SpaceCenter_Flight_t flight_body;
    krpc_SpaceCenter_Flight_t flight_vessel;

    PlanetModel planet;
    char vessel_name[128];
    char transport_name[192];
    char flight_key[128];
    char cached_situation[64];

    CommandableEngine engines[CNANO_ENGINE_LIMIT];
    size_t engine_count;
    double commandable_thrust;

    bool cached_gear;
    bool cached_brakes;
    bool cached_airbrakes;
    bool cached_rcs;
    bool cached_sas;
    bool cached_autopilot_engaged;
    bool has_autopilot_engaged;
    bool orbital_autotune_profile_active;
    double cached_command_throttle;
    double cached_command_wheel;
    bool has_command_throttle;
    bool has_command_wheel;
    bool has_gear;
    bool has_brakes;
    bool has_airbrakes;
    bool has_rcs;
    bool has_sas;
    bool pause_triplet_sent;
    NavballSpeedMode cached_speed_mode;
    bool has_speed_mode;

    double cached_speed_of_sound;
    double cached_static_pressure;
    double cached_g_force;
    double cached_available_thrust;
    double cached_current_thrust;
    double cached_dry_mass;
    double cached_torque[3];
    double cached_inertia[3];
    Vector3 cached_lift_vector, cached_drag_vector, cached_center_of_mass, cached_center_of_mass_root;
    bool cached_force_vectors_valid, cached_center_of_mass_valid, cached_center_of_mass_root_valid;
    double cached_apoapsis;
    double cached_periapsis;
    double cached_period;
    double last_medium_ut;
    double last_slow_ut;
    bool medium_valid;
    bool slow_valid;

    bool has_previous_attitude;
    double previous_ut;
    double previous_pitch;
    double previous_roll;
    double previous_heading;
    double previous_angle_of_attack;
    bool has_previous_course;
    double previous_course_ut;
    double previous_course;
    LowPass pitch_rate_filter;
    LowPass roll_rate_filter;
    LowPass heading_rate_filter;
    LowPass angle_of_attack_rate_filter;
    LowPass course_rate_filter;

    unsigned total_rpc_calls;
    unsigned total_wire_requests;
    unsigned last_read_wire_requests;
    unsigned last_apply_wire_requests;
    unsigned last_read_calls;
    unsigned last_apply_calls;
    double last_read_seconds;
    double last_apply_seconds;
};

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
    const char *path=configuration->connection.serial_port;
    const char *override=getenv("KSP_LANDER_SERIAL_PORT");if(override&&*override)path=override;
    if(!path||!*path){set_error(error,error_size,"No kRPC serial-protocol endpoint is configured (connection.serialPort)");free(client);return NULL;}
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

static bool refresh_medium(KrpcCNanoClient *client,unsigned airbrake_group,char *error,size_t error_size){
    float speed_of_sound=(float)client->cached_speed_of_sound;
    float static_pressure=(float)client->cached_static_pressure;
    float g_force=(float)client->cached_g_force;
    float dry_mass=(float)client->cached_dry_mass;
    float available_thrust=(float)client->cached_available_thrust;
    float current_thrust=(float)client->cached_current_thrust;
    krpc_tuple_tuple_double_double_double_tuple_double_double_double_t torque={0};
    krpc_tuple_double_double_double_t inertia={0},lift={0},drag={0},com={0},com_root={0};
    bool gear=client->cached_gear,brakes=client->cached_brakes,airbrakes=client->cached_airbrakes;
    bool rcs=client->cached_rcs,sas=client->cached_sas,autopilot=client->cached_autopilot_engaged;
    krpc_enum_t speed_mode=(krpc_enum_t)KRPC_SPACECENTER_SPEEDMODE_SURFACE;
    krpc_enum_t situation=(krpc_enum_t)KRPC_SPACECENTER_VESSELSITUATION_FLYING;
    if(airbrake_group<1)airbrake_group=1;if(airbrake_group>10)airbrake_group=10;

    ClientBatch batch={0};
#define ADD(expr) do{if(!(expr)){set_error(error,error_size,"Could not construct medium C-Nano telemetry batch");return false;}}while(0)
    ADD(batch_add_object(&batch,PROC_FLIGHT_SPEED_OF_SOUND,client->flight_surface,BATCH_FLOAT,&speed_of_sound,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_STATIC_PRESSURE,client->flight_surface,BATCH_FLOAT,&static_pressure,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_G_FORCE,client->flight_surface,BATCH_FLOAT,&g_force,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_DRY_MASS,client->vessel,BATCH_FLOAT,&dry_mass,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_AVAILABLE_THRUST,client->vessel,BATCH_FLOAT,&available_thrust,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_THRUST,client->vessel,BATCH_FLOAT,&current_thrust,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_AVAILABLE_TORQUE,client->vessel,BATCH_TORQUE,&torque,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_MOMENT_OF_INERTIA,client->vessel,BATCH_TUPLE3,&inertia,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_LIFT,client->flight_body,BATCH_TUPLE3,&lift,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_DRAG,client->flight_body,BATCH_TUPLE3,&drag,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_CENTER_OF_MASS,client->flight_body,BATCH_TUPLE3,&com,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_CENTER_OF_MASS,client->flight_vessel,BATCH_TUPLE3,&com_root,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_GEAR,client->control,BATCH_BOOL,&gear,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_BRAKES,client->control,BATCH_BOOL,&brakes,true));
    ADD(batch_add_object_uint(&batch,PROC_CONTROL_GET_ACTION_GROUP,client->control,airbrake_group,BATCH_BOOL,&airbrakes,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_RCS,client->control,BATCH_BOOL,&rcs,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_SAS,client->control,BATCH_BOOL,&sas,true));
    ADD(batch_add_object(&batch,PROC_AUTOPILOT_ENGAGED,client->autopilot,BATCH_BOOL,&autopilot,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_SPEED_MODE,client->control,BATCH_ENUM,&speed_mode,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_SITUATION,client->vessel,BATCH_ENUM,&situation,true));
#undef ADD
    if(!batch_run(client,&batch,"medium telemetry",error,error_size))return false;

    client->cached_speed_of_sound=speed_of_sound;
    client->cached_static_pressure=static_pressure;
    client->cached_g_force=g_force;
    client->cached_dry_mass=dry_mass;
    client->cached_available_thrust=available_thrust;
    client->cached_current_thrust=current_thrust;
    client->cached_torque[0]=fmax(fabs(torque.e0.e0),fabs(torque.e1.e0));
    client->cached_torque[1]=fmax(fabs(torque.e0.e1),fabs(torque.e1.e1));
    client->cached_torque[2]=fmax(fabs(torque.e0.e2),fabs(torque.e1.e2));
    client->cached_inertia[0]=fabs(inertia.e0);client->cached_inertia[1]=fabs(inertia.e1);client->cached_inertia[2]=fabs(inertia.e2);
    client->cached_lift_vector=tuple_to_canonical(lift);client->cached_drag_vector=tuple_to_canonical(drag);client->cached_force_vectors_valid=true;
    client->cached_center_of_mass=tuple_to_canonical(com);client->cached_center_of_mass_valid=true;
    client->cached_center_of_mass_root=tuple_to_canonical(com_root);client->cached_center_of_mass_root_valid=true;
    client->cached_gear=gear;client->has_gear=true;
    client->cached_brakes=brakes;client->has_brakes=true;
    client->cached_airbrakes=airbrakes;client->has_airbrakes=true;
    client->cached_rcs=rcs;client->has_rcs=true;
    client->cached_sas=sas;client->has_sas=true;
    client->cached_autopilot_engaged=autopilot;client->has_autopilot_engaged=true;
    client->cached_speed_mode=speed_mode==(krpc_enum_t)KRPC_SPACECENTER_SPEEDMODE_ORBIT?SPEED_ORBIT:SPEED_SURFACE;client->has_speed_mode=true;
    snprintf(client->cached_situation,sizeof(client->cached_situation),"%s",situation_name((krpc_SpaceCenter_VesselSituation_t)situation));
    client->medium_valid=true;
    return true;
}

static bool refresh_slow(KrpcCNanoClient *client,char *error,size_t error_size){
    double apoapsis=client->cached_apoapsis,periapsis=client->cached_periapsis,period=client->cached_period;
    ClientBatch batch={0};
    if(!batch_add_object(&batch,PROC_ORBIT_APOAPSIS_ALTITUDE,client->orbit,BATCH_DOUBLE,&apoapsis,true)||
       !batch_add_object(&batch,PROC_ORBIT_PERIAPSIS_ALTITUDE,client->orbit,BATCH_DOUBLE,&periapsis,true)||
       !batch_add_object(&batch,PROC_ORBIT_PERIOD,client->orbit,BATCH_DOUBLE,&period,true)){
        set_error(error,error_size,"Could not construct slow C-Nano telemetry batch");return false;
    }
    if(!batch_run(client,&batch,"slow telemetry",error,error_size))return false;
    client->cached_apoapsis=apoapsis;client->cached_periapsis=periapsis;client->cached_period=period;client->slow_valid=true;
    return true;
}

bool krpc_cnano_client_read(KrpcCNanoClient *client,
                            const LandingConfiguration *configuration,
                            Telemetry *t, VehicleState *state,
                            char *error, size_t error_size) {
    if (!client || !client->connected || !configuration || !t || !state) {
        set_error(error, error_size, "C-Nano telemetry read has invalid state");
        return false;
    }
    double started = monotonic_seconds();
    unsigned before=client->total_rpc_calls;
    unsigned wire_before=client->total_wire_requests;
    telemetry_init(t);
    snprintf(t->vessel_name, sizeof(t->vessel_name), "%s", client->vessel_name);

    double ut=0.0,radar=0.0;
    float true_air_speed=0.0f,heading=0.0f,pitch=0.0f,roll=0.0f,aoa=0.0f,beta=0.0f,q=0.0f;
    /* AutoPilot.Error throws on the real kRPC server while disengaged. Keep a
       deliberately unsafe default so guidance can never mistake unavailable
       alignment feedback for a perfect zero-degree error. */
    float mass=0.0f,cp=0.0f,cr=0.0f,cy=0.0f,throttle=0.0f,autopilot_error=180.0f;
    krpc_tuple_double_double_double_t position={0},velocity={0},angular={0};
    krpc_tuple_double_double_double_double_t rotation={0};
    ClientBatch fast={0};
#define FAST(expr) do{if(!(expr)){set_error(error,error_size,"Could not construct fast C-Nano telemetry batch");return false;}}while(0)
    FAST(batch_add(&fast,PROC_UT,BATCH_DOUBLE,&ut,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_SURFACE_ALTITUDE,client->flight_surface,BATCH_DOUBLE,&radar,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_TRUE_AIR_SPEED,client->flight_surface,BATCH_FLOAT,&true_air_speed,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_HEADING,client->flight_surface,BATCH_FLOAT,&heading,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_PITCH,client->flight_surface,BATCH_FLOAT,&pitch,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_ROLL,client->flight_surface,BATCH_FLOAT,&roll,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_AOA,client->flight_surface,BATCH_FLOAT,&aoa,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_SIDESLIP,client->flight_surface,BATCH_FLOAT,&beta,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_DYNAMIC_PRESSURE,client->flight_surface,BATCH_FLOAT,&q,true));
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_POSITION,client->vessel,client->body_nonrotating,BATCH_TUPLE3,&position,true));
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_VELOCITY,client->vessel,client->body_nonrotating,BATCH_TUPLE3,&velocity,true));
    /* Angular velocity in Vessel.ReferenceFrame is identically zero because
       that frame rotates with the craft. Read it in the non-rotating body
       frame and rotate the vector back into vessel axes below. */
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_ANGULAR_VELOCITY,client->vessel,client->body_nonrotating,BATCH_TUPLE3,&angular,true));
    FAST(batch_add_object(&fast,PROC_VESSEL_MASS,client->vessel,BATCH_FLOAT,&mass,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_PITCH,client->control,BATCH_FLOAT,&cp,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_ROLL,client->control,BATCH_FLOAT,&cr,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_YAW,client->control,BATCH_FLOAT,&cy,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_THROTTLE,client->control,BATCH_FLOAT,&throttle,true));
    if (client->has_autopilot_engaged && client->cached_autopilot_engaged)
        FAST(batch_add_object(&fast,PROC_AUTOPILOT_ERROR,client->autopilot,BATCH_FLOAT,&autopilot_error,true));
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_ROTATION,client->vessel,client->body_nonrotating,BATCH_TUPLE4,&rotation,true));
#undef FAST
    if(!batch_run(client,&fast,"fast telemetry",error,error_size))return false;

    t->ut=ut;t->radar_altitude=radar;t->true_air_speed=true_air_speed;t->heading=heading;t->pitch=pitch;t->roll=roll;
    t->angle_of_attack=aoa;t->sideslip=beta;t->dynamic_pressure=q;t->mass=mass;
    t->has_controls=true;t->control_pitch=cp;t->control_roll=cr;t->control_yaw=cy;t->throttle=throttle;
    t->autopilot_error=fabs((double)autopilot_error);
    Vector3 pos = tuple_to_canonical(position);
    Vector3 vel = tuple_to_canonical(velocity);
    GeoPoint geo = predictor_geo_point(pos, &client->planet, ut);
    t->latitude = geo.latitude;
    t->longitude = geo.longitude;
    t->mean_altitude = geo.altitude;

    Vector3 up = vnorm(pos, v3(0, 1, 0));
    Vector3 surface_velocity = vsub(vel, vcross(planet_rotation_vector(&client->planet), pos));
    t->vertical_speed = vdot(surface_velocity, up);
    t->horizontal_speed = vmag(vproject_plane(surface_velocity, up));
    t->surface_speed = vmag(surface_velocity);
    t->flight_path_angle = atan2(t->vertical_speed, fmax(.1, t->horizontal_speed)) * RAD2DEG;

    if (t->true_air_speed > 1.0 && t->dynamic_pressure >= 0.0)
        t->atmospheric_density = 2.0 * t->dynamic_pressure / (t->true_air_speed * t->true_air_speed);
    else
        t->atmospheric_density = planet_atmospheric_density(&client->planet, t->mean_altitude);

    /* `rotation` is kRPC's (x,y,z,w) quaternion mapping vessel-frame vectors
       into body-non-rotating coordinates. Apply its transpose to the inertial
       angular-velocity vector to recover true body-axis rates. */
    double qx=rotation.e0,qy=rotation.e1,qz=rotation.e2,qw=rotation.e3;
    double r00=1.0-2.0*(qy*qy+qz*qz),r01=2.0*(qx*qy-qz*qw),r02=2.0*(qx*qz+qy*qw);
    double r10=2.0*(qx*qy+qz*qw),r11=1.0-2.0*(qx*qx+qz*qz),r12=2.0*(qy*qz-qx*qw);
    double r20=2.0*(qx*qz-qy*qw),r21=2.0*(qy*qz+qx*qw),r22=1.0-2.0*(qx*qx+qy*qy);
    double body_x=r00*angular.e0+r10*angular.e1+r20*angular.e2;
    double body_y=r01*angular.e0+r11*angular.e1+r21*angular.e2;
    double body_z=r02*angular.e0+r12*angular.e1+r22*angular.e2;
    /* Keep the established bridge/control sign convention. */
    t->has_body_pitch_rate = t->has_body_roll_rate = t->has_body_yaw_rate = true;
    t->body_pitch_rate = -body_x * RAD2DEG;
    t->body_roll_rate = -body_y * RAD2DEG;
    t->body_yaw_rate = -body_z * RAD2DEG;

    t->has_attitude_quaternion=true;
    t->attitude_quaternion[0]=rotation.e0;t->attitude_quaternion[1]=rotation.e1;t->attitude_quaternion[2]=rotation.e2;t->attitude_quaternion[3]=rotation.e3;
    snprintf(t->attitude_reference_frame,sizeof(t->attitude_reference_frame),"body-non-rotating");

    if (client->has_previous_attitude) {
        double dt = clampd(ut - client->previous_ut, .01, 1.0);
        t->pitch_rate = lowpass_update(&client->pitch_rate_filter, norm_signed_deg(t->pitch - client->previous_pitch) / dt, dt);
        t->roll_rate = lowpass_update(&client->roll_rate_filter, norm_signed_deg(t->roll - client->previous_roll) / dt, dt);
        t->heading_rate = lowpass_update(&client->heading_rate_filter, norm_signed_deg(t->heading - client->previous_heading) / dt, dt);
        t->angle_of_attack_rate=lowpass_update(&client->angle_of_attack_rate_filter,
            norm_signed_deg(t->angle_of_attack-client->previous_angle_of_attack)/dt,dt);
        t->has_angle_of_attack_rate=true;
    }
    client->has_previous_attitude = true;
    client->previous_ut = ut; client->previous_pitch = t->pitch; client->previous_roll = t->roll; client->previous_heading = t->heading;
    client->previous_angle_of_attack=t->angle_of_attack;

    t->ground_track_heading = surface_course(pos, vel, planet_rotation_vector(&client->planet), client->planet.north_axis, t->heading);
    if (client->has_previous_course) {
        double dt = ut - client->previous_course_ut;
        if (dt >= .01 && dt <= 1.0) {
            t->course_rate = lowpass_update(&client->course_rate_filter,
                norm_signed_deg(t->ground_track_heading - client->previous_course) / dt, dt);
            t->has_course_rate = true;
        } else lowpass_reset(&client->course_rate_filter);
    }
    client->has_previous_course = true;
    client->previous_course_ut = ut; client->previous_course = t->ground_track_heading;

    if (!client->medium_valid || ut - client->last_medium_ut >= CNANO_MEDIUM_PERIOD) {
        if(!refresh_medium(client,configuration->vehicle.airbrake_action_group,error,error_size))return false;
        client->last_medium_ut=ut;
    }
    if (!client->slow_valid || ut - client->last_slow_ut >= CNANO_SLOW_PERIOD) {
        if(!refresh_slow(client,error,error_size))return false;
        client->last_slow_ut=ut;
    }

    t->speed_of_sound = client->cached_speed_of_sound > 1.0
        ? client->cached_speed_of_sound : planet_atmospheric_speed_of_sound(&client->planet, t->mean_altitude);
    t->mach = t->true_air_speed / fmax(1.0, t->speed_of_sound);
    t->static_pressure = client->cached_static_pressure;
    t->g_force = client->cached_g_force;
    t->dry_mass = client->cached_dry_mass;
    t->available_thrust = fmax(client->cached_available_thrust, client->commandable_thrust);
    t->current_thrust = client->cached_current_thrust;
    t->has_torque = client->cached_torque[0] > 0 || client->cached_torque[1] > 0 || client->cached_torque[2] > 0;
    t->available_pitch_torque = client->cached_torque[0];
    t->available_roll_torque = client->cached_torque[1];
    t->available_yaw_torque = client->cached_torque[2];
    t->has_inertia = client->cached_inertia[0] > 0 && client->cached_inertia[1] > 0 && client->cached_inertia[2] > 0;
    t->pitch_moment_of_inertia = client->cached_inertia[0];
    t->roll_moment_of_inertia = client->cached_inertia[1];
    t->yaw_moment_of_inertia = client->cached_inertia[2];
    t->apoapsis_altitude = client->cached_apoapsis;
    t->periapsis_altitude = client->cached_periapsis;
    t->orbit_period = client->cached_period;
    t->gear = client->cached_gear;
    t->brakes = client->cached_brakes;
    t->has_airbrakes = client->has_airbrakes;
    t->airbrakes = client->cached_airbrakes;
    t->navball_speed_mode = client->has_speed_mode ? client->cached_speed_mode : SPEED_UNCHANGED;
    if (client->cached_situation[0]) snprintf(t->vessel_situation, sizeof(t->vessel_situation), "%s", client->cached_situation);
    t->has_force_vectors = client->cached_force_vectors_valid;
    t->lift_vector = client->cached_lift_vector; t->drag_vector = client->cached_drag_vector;
    t->lift_force = vmag(t->lift_vector); t->drag_force = vmag(t->drag_vector);
    t->has_center_of_mass = client->cached_center_of_mass_valid; t->center_of_mass = client->cached_center_of_mass;
    t->has_center_of_mass_root=client->cached_center_of_mass_root_valid;t->center_of_mass_root=client->cached_center_of_mass_root;

    /* C-Nano exposes no stream-side stall metric. Share the same conservative
       q/speed/AoA fallback proxy with predictor shadow telemetry so safety decisions
       see identical inputs without letting nominal low-q protective AoA classify
       itself as a measured stall emergency. */
    t->stall_fraction = vessel_physics_conservative_stall_fraction(
        t->true_air_speed, t->angle_of_attack, t->dynamic_pressure, &configuration->vehicle);
    t->stall_fraction_is_measured = false;
    t->physics_sample_valid = t->has_force_vectors && t->dynamic_pressure > 1.0 && t->mass > 1.0;

    GeoPoint current = {t->latitude, t->longitude, t->mean_altitude};
    GeoPoint site = {configuration->site.latitude, configuration->site.longitude, configuration->site.altitude};
    t->range_to_site = great_circle_distance(current, site, client->planet.radius);
    t->bearing_to_site = initial_bearing(current, site);
    t->heading_error = norm_signed_deg(t->bearing_to_site - t->heading);
    t->course_to_site_error = norm_signed_deg(t->bearing_to_site - t->ground_track_heading);
    runway_coordinates(current, site, configuration->site.runway_heading, client->planet.radius,
                       &t->runway_along_track, &t->runway_cross_track);

    state->ut = ut; state->position = pos; state->velocity = vel; state->mass = t->mass;
    client->last_read_calls=client->total_rpc_calls-before;
    client->last_read_wire_requests=client->total_wire_requests-wire_before;
    client->last_read_seconds=monotonic_seconds()-started;
    return true;
}

static bool set_engine_throttle(KrpcCNanoClient *client, double throttle,
                                char *error, size_t error_size) {
    bool powered = throttle > 1e-4;
    for (size_t i = 0; i < client->engine_count; ++i) {
        CommandableEngine *engine = &client->engines[i];
        if (powered) {
            bool active = false, fuel = true;
            krpc_error_t e = krpc_SpaceCenter_Engine_Active(client->connection, &active, engine->object); client->total_rpc_calls++; client->total_wire_requests++;
            if (e != KRPC_OK) continue;
            e = krpc_SpaceCenter_Engine_HasFuel(client->connection, &fuel, engine->object); client->total_rpc_calls++; client->total_wire_requests++;
            if (e != KRPC_OK || !fuel) continue;
            if (!active) {
                if (rpc_failed(client, krpc_SpaceCenter_Engine_set_Active(client->connection, engine->object, true),
                               "Engine.Active", error, error_size)) return false;
                engine->activated_by_client = true;
            }
            if (rpc_failed(client, krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, true),
                           "Engine.IndependentThrottle", error, error_size) ||
                rpc_failed(client, krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, (float)throttle),
                           "Engine.Throttle", error, error_size)) return false;
            engine->independent_owned = true;
        } else if (engine->independent_owned) {
            RPC_OPTIONAL(client, krpc_SpaceCenter_Engine_set_Throttle(client->connection, engine->object, 0.0f));
            RPC_OPTIONAL(client, krpc_SpaceCenter_Engine_set_IndependentThrottle(client->connection, engine->object, false));
            engine->independent_owned = false;
        }
    }
    return true;
}

bool krpc_cnano_client_set_direct_controls(KrpcCNanoClient *client,
                                           double pitch,double roll,double yaw,
                                           double throttle,double wheel_steering,
                                           bool rcs,
                                           char *error,size_t error_size){
    if(!client||!client->connected){set_error(error,error_size,"C-Nano client is not connected");return false;}
    client->pause_triplet_sent=false;
    double started=monotonic_seconds();unsigned before=client->total_rpc_calls;unsigned wire_before=client->total_wire_requests;
    pitch=clampd(pitch,-1,1);roll=clampd(roll,-1,1);yaw=clampd(yaw,-1,1);
    throttle=clampd(throttle,0,1);wheel_steering=clampd(wheel_steering,-1,1);

    bool reset_orbital_profile=client->orbital_autotune_profile_active;
    if(reset_orbital_profile){
        /* Direct atmospheric control must not inherit orbital AutoPilot tuning.
           Reset is a single deterministic handoff: it disengages AutoPilot and
           restores all of its controller configuration/oscillation state. */
        RPC_REQUIRED(client,krpc_SpaceCenter_AutoPilot_Reset(client->connection,client->autopilot),error,error_size);
        client->cached_autopilot_engaged=false;client->has_autopilot_engaged=true;
        client->orbital_autotune_profile_active=false;
    }
    bool disengage_autopilot=!reset_orbital_profile&&(!client->has_autopilot_engaged||client->cached_autopilot_engaged);
    bool disable_sas=!client->has_sas||client->cached_sas;
    bool change_throttle=!client->has_command_throttle||fabs(client->cached_command_throttle-throttle)>1e-4;
    bool change_wheel=!client->has_command_wheel||fabs(client->cached_command_wheel-wheel_steering)>1e-3;
    bool change_rcs=!client->has_rcs||client->cached_rcs!=rcs;

    ClientBatch batch={0};
#define WRITE(expr) do{if(!(expr)){set_error(error,error_size,"Could not construct direct-control C-Nano batch");return false;}}while(0)
    if(disengage_autopilot)WRITE(batch_add_object_bool(&batch,PROC_AUTOPILOT_SET_ENGAGED,client->autopilot,false,true));
    if(disable_sas)WRITE(batch_add_object_bool(&batch,PROC_CONTROL_SET_SAS,client->control,false,true));
    WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_PITCH,client->control,(float)pitch,true));
    WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_ROLL,client->control,(float)roll,true));
    WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_YAW,client->control,(float)yaw,true));
    if(change_throttle)WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_THROTTLE,client->control,(float)throttle,true));
    if(change_wheel)WRITE(batch_add_object_float(&batch,PROC_CONTROL_SET_WHEEL_STEERING,client->control,(float)wheel_steering,true));
    if(change_rcs)WRITE(batch_add_object_bool(&batch,PROC_CONTROL_SET_RCS,client->control,rcs,true));
#undef WRITE
    if(!batch_run(client,&batch,"direct control",error,error_size))return false;

    if(disengage_autopilot){client->cached_autopilot_engaged=false;client->has_autopilot_engaged=true;}
    if(disable_sas){client->cached_sas=false;client->has_sas=true;}
    if(change_throttle){
        if(!set_engine_throttle(client,throttle,error,error_size))return false;
        client->cached_command_throttle=throttle;client->has_command_throttle=true;
    }
    if(change_wheel){client->cached_command_wheel=wheel_steering;client->has_command_wheel=true;}
    if(change_rcs){client->cached_rcs=rcs;client->has_rcs=true;}

    client->last_apply_calls=client->total_rpc_calls-before;
    client->last_apply_wire_requests=client->total_wire_requests-wire_before;
    client->last_apply_seconds=monotonic_seconds()-started;
    return true;
}

bool krpc_cnano_client_set_autopilot(KrpcCNanoClient *client,
                                     const GuidanceCommand *command,
                                     const Vector3 *inertial_up_reference,
                                     char *error, size_t error_size) {
    if (!client || !client->connected || !command) { set_error(error, error_size, "C-Nano autopilot state is invalid"); return false; }
    client->pause_triplet_sent=false;
    double started=monotonic_seconds(); unsigned before=client->total_rpc_calls; unsigned wire_before=client->total_wire_requests;
    if (!client->has_sas || client->cached_sas) {
        RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_SAS(client->connection, client->control, false), error, error_size);
        client->cached_sas = false; client->has_sas = true;
    }
    /* Orbital/inertial attitude capture has essentially no aerodynamic control
       authority. Own RCS explicitly there, then release it as soon as guidance
       returns to surface-frame atmospheric control. SAS remains off. */
    bool want_rcs = command->control_profile == PROFILE_ORBITAL;
    if (!client->has_rcs || client->cached_rcs != want_rcs) {
        RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_RCS(client->connection, client->control, want_rcs), error, error_size);
        client->cached_rcs = want_rcs; client->has_rcs = true;
    }
    krpc_SpaceCenter_ReferenceFrame_t frame = command->use_inertial_direction ? client->body_nonrotating : client->surface_frame;
    bool want_orbital_tuning = command->control_profile == PROFILE_ORBITAL;
    bool engaging_orbital = want_orbital_tuning && command->autopilot_engaged &&
        (!client->has_autopilot_engaged || !client->cached_autopilot_engaged);
    if (engaging_orbital && !client->orbital_autotune_profile_active) {
        /* Keep kRPC AutoTune live: it recomputes gains from the current torque and
           inertia every physics tick. STS-N's roll axis has much greater angular
           authority than pitch/yaw, so a lower roll bandwidth prevents the roll
           rate damper from railing together with pitch during the antipodal
           retrograde slew. The pitch/yaw overshoot allowance recovers capture-time
           margin without manually freezing any PID gains. Repeated live checkpoint
           trials: 27.44-28.58 s to <=10 deg, 0.26-0.36 s multi-axis saturation. */
        krpc_tuple_double_double_double_t time_to_peak = {0.8, 5.0, 0.8};
        krpc_tuple_double_double_double_t overshoot = {0.05, 0.01, 0.05};
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_AutoTune(client->connection, client->autopilot, true), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TimeToPeak(client->connection, client->autopilot, &time_to_peak), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_Overshoot(client->connection, client->autopilot, &overshoot), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_SoftStartTime(client->connection, client->autopilot, 0.5), error, error_size);
        client->orbital_autotune_profile_active = true;
    } else if (!want_orbital_tuning && client->orbital_autotune_profile_active) {
        /* Restore kRPC's complete default controller contract before any later
           surface-frame AutoPilot engagement in this session. */
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_Reset(client->connection, client->autopilot), error, error_size);
        client->cached_autopilot_engaged = false; client->has_autopilot_engaged = true;
        client->orbital_autotune_profile_active = false;
    }
    RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_ReferenceFrame(client->connection, client->autopilot, frame), error, error_size);
    if (command->use_inertial_direction) {
        krpc_tuple_double_double_double_t direction = canonical_to_tuple(command->inertial_direction);
        if (inertial_up_reference) {
            Vector3 normalized_up = vnorm(*inertial_up_reference, v3(1, 0, 0));
            krpc_tuple_double_double_double_t up = canonical_to_tuple(normalized_up);
            RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_SetDirectionAndUp(client->connection, client->autopilot, &direction, &up, (float)command->target_roll), error, error_size);
        } else {
            RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetDirection(client->connection, client->autopilot, &direction), error, error_size);
            RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetRoll(client->connection, client->autopilot, (float)command->target_roll), error, error_size);
        }
    } else {
        /* Vessel.SurfaceReferenceFrame is x=up, y=north, z=east.  Sending
           target pitch and heading as two independent RPCs makes kRPC rotate
           through an intermediate attitude every control tick; during the
           post-burn 180-degree RCS flip that produced a large roll/yaw limit
           cycle.  Build the desired local direction once and update it
           atomically as a quaternion target instead. */
        double pitch = clampd(command->target_pitch, -89.9, 89.9) * DEG2RAD;
        double heading = norm_deg(command->target_heading) * DEG2RAD;
        double cp = cos(pitch);
        krpc_tuple_double_double_double_t direction = {
            sin(pitch),
            cp * cos(heading),
            cp * sin(heading)
        };
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetDirection(client->connection, client->autopilot, &direction), error, error_size);
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_TargetRoll(client->connection, client->autopilot, (float)command->target_roll), error, error_size);
    }
    if (!client->has_autopilot_engaged || client->cached_autopilot_engaged != command->autopilot_engaged) {
        RPC_REQUIRED(client, krpc_SpaceCenter_AutoPilot_set_Engaged(client->connection, client->autopilot, command->autopilot_engaged), error, error_size);
        client->cached_autopilot_engaged = command->autopilot_engaged; client->has_autopilot_engaged = true;
    }
    double throttle = clampd(command->target_throttle, 0, 1);
    if (!client->has_command_throttle || fabs(client->cached_command_throttle - throttle) > 1e-4) {
        RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_Throttle(client->connection, client->control, (float)throttle), error, error_size);
        if (!set_engine_throttle(client, throttle, error, error_size)) return false;
        client->cached_command_throttle = throttle; client->has_command_throttle = true;
    }
    client->cached_sas = false; client->has_sas = true;
    client->last_apply_calls=client->total_rpc_calls-before;
    client->last_apply_wire_requests=client->total_wire_requests-wire_before;
    client->last_apply_seconds=monotonic_seconds()-started;
    return true;
}

bool krpc_cnano_client_set_gear(KrpcCNanoClient *client, bool value, char *error, size_t error_size) {
    if (!client || !client->connected) return false;
    if (client->has_gear && client->cached_gear == value) return true;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_Gear(client->connection, client->control, value), error, error_size);
    client->cached_gear = value; client->has_gear = true; return true;
}
bool krpc_cnano_client_set_brakes(KrpcCNanoClient *client, bool value, char *error, size_t error_size) {
    if (!client || !client->connected) return false;
    if (client->has_brakes && client->cached_brakes == value) return true;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_Brakes(client->connection, client->control, value), error, error_size);
    client->cached_brakes = value; client->has_brakes = true; return true;
}
bool krpc_cnano_client_set_airbrakes(KrpcCNanoClient *client, unsigned group, bool value, char *error, size_t error_size) {
    if (!client || !client->connected) return false;
    if (client->has_airbrakes && client->cached_airbrakes == value) return true;
    if (group < 1) group = 1; if (group > 10) group = 10;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_SetActionGroup(client->connection, client->control, group, value), error, error_size);
    client->cached_airbrakes = value; client->has_airbrakes = true; return true;
}
bool krpc_cnano_client_set_speed_mode(KrpcCNanoClient *client, NavballSpeedMode mode, char *error, size_t error_size) {
    if (!client || !client->connected || mode == SPEED_UNCHANGED) return true;
    if (client->has_speed_mode && client->cached_speed_mode == mode) return true;
    krpc_SpaceCenter_SpeedMode_t native = mode == SPEED_ORBIT ? KRPC_SPACECENTER_SPEEDMODE_ORBIT : KRPC_SPACECENTER_SPEEDMODE_SURFACE;
    RPC_REQUIRED(client, krpc_SpaceCenter_Control_set_SpeedMode(client->connection, client->control, native), error, error_size);
    client->cached_speed_mode = mode; client->has_speed_mode = true; return true;
}
/* decision-literal: protocol-domain-requirement | Stock KSP rails-warp factor indices map to these on-rails rates; live SpaceCenter.WarpRate replaces each prior once observed. */
static const double k_stock_rails_rate_by_factor[] = {
    1.0, 5.0, 10.0, 50.0, 100.0, 1000.0, 10000.0, 100000.0
};

static bool warp_decision_from_rates(double remaining_ut,
                                     double response_horizon_seconds,
                                     int maximum_factor,
                                     const double *rates,
                                     size_t rate_count,
                                     KrpcCNanoWarpDecision *decision) {
    if (!decision || !rates || rate_count == 0 ||
        !isfinite(remaining_ut) || remaining_ut < 0.0 ||
        !isfinite(response_horizon_seconds) || response_horizon_seconds <= 0.0 ||
        maximum_factor < 0 || (size_t)maximum_factor >= rate_count)
        return false;

    *decision = (KrpcCNanoWarpDecision){
        .factor = 0,
        .predicted_rate = rates[0],
        .response_horizon_seconds = response_horizon_seconds,
        .stopping_margin_ut = remaining_ut
    };
    for (int factor = maximum_factor; factor >= 1; --factor) {
        double rate = rates[factor];
        if (!isfinite(rate) || rate <= 0.0) return false;
        double margin = remaining_ut - rate * response_horizon_seconds;
        if (margin >= 0.0) {
            decision->factor = factor;
            decision->predicted_rate = rate;
            decision->stopping_margin_ut = margin;
            break;
        }
    }
    return true;
}

bool krpc_cnano_client_warp_decision(double remaining_ut,
                                     double response_horizon_seconds,
                                     int maximum_factor,
                                     KrpcCNanoWarpDecision *decision) {
    return warp_decision_from_rates(
        remaining_ut, response_horizon_seconds, maximum_factor,
        k_stock_rails_rate_by_factor,
        sizeof(k_stock_rails_rate_by_factor) /
            sizeof(k_stock_rails_rate_by_factor[0]),
        decision);
}

static bool set_rails_warp_factor_observed(KrpcCNanoClient *client,
                                           int32_t requested,
                                           int32_t *observed,
                                           double *latency_seconds,
                                           char *error,
                                           size_t error_size) {
    if (!client || !observed || !latency_seconds) return false;
    if (!krpc_cnano_transport_begin_deadline(client->connection)) {
        set_error(error, error_size,
                  "Could not start the configured rails-warp response deadline");
        return false;
    }

    double started = monotonic_seconds();
    krpc_error_t set_result =
        krpc_SpaceCenter_set_RailsWarpFactor(client->connection, requested);
    if (rpc_failed(client, set_result, "SpaceCenter.RailsWarpFactor(set)",
                   error, error_size)) {
        krpc_cnano_transport_end_deadline(client->connection);
        return false;
    }

    krpc_error_t read_result =
        krpc_SpaceCenter_RailsWarpFactor(client->connection, observed);
    *latency_seconds = monotonic_seconds() - started;
    krpc_cnano_transport_end_deadline(client->connection);
    if (rpc_failed(client, read_result,
                   "SpaceCenter.RailsWarpFactor(readback)",
                   error, error_size))
        return false;

    size_t rate_count = sizeof(k_stock_rails_rate_by_factor) /
                        sizeof(k_stock_rails_rate_by_factor[0]);
    if (*observed < 0 || (size_t)*observed >= rate_count ||
        (requested == 0 && *observed != 0) ||
        (requested > 0 && *observed > requested)) {
        set_error(error, error_size,
                  "KSP returned invalid rails-warp factor %d for request %d",
                  (int)*observed, (int)requested);
        return false;
    }
    if (!isfinite(*latency_seconds) || *latency_seconds <= 0.0) {
        set_error(error, error_size,
                  "Could not measure rails-warp command/response latency");
        return false;
    }
    return true;
}

typedef struct {
    double ut;
    int32_t factor;
    double rate;
    double latency_seconds;
} CNanoWarpState;

static bool read_warp_state(KrpcCNanoClient *client,
                            CNanoWarpState *state,
                            char *error,
                            size_t error_size) {
    if (!client || !state) return false;
    if (!krpc_cnano_transport_begin_deadline(client->connection)) {
        set_error(error, error_size,
                  "Could not start the configured rails-warp state deadline");
        return false;
    }

    double started = monotonic_seconds();
    float rate = NAN;
    krpc_error_t code =
        krpc_SpaceCenter_UT(client->connection, &state->ut);
    if (rpc_failed(client, code, "SpaceCenter.UT(warp)",
                   error, error_size)) {
        krpc_cnano_transport_end_deadline(client->connection);
        return false;
    }
    code = krpc_SpaceCenter_RailsWarpFactor(
        client->connection, &state->factor);
    if (rpc_failed(client, code, "SpaceCenter.RailsWarpFactor(warp-state)",
                   error, error_size)) {
        krpc_cnano_transport_end_deadline(client->connection);
        return false;
    }
    code = krpc_SpaceCenter_WarpRate(client->connection, &rate);
    state->latency_seconds = monotonic_seconds() - started;
    krpc_cnano_transport_end_deadline(client->connection);
    if (rpc_failed(client, code, "SpaceCenter.WarpRate",
                   error, error_size))
        return false;

    size_t rate_count = sizeof(k_stock_rails_rate_by_factor) /
                        sizeof(k_stock_rails_rate_by_factor[0]);
    state->rate = (double)rate;
    if (!isfinite(state->ut) ||
        state->factor < 0 || (size_t)state->factor >= rate_count ||
        !isfinite(state->rate) || state->rate <= 0.0 ||
        !isfinite(state->latency_seconds) ||
        state->latency_seconds <= 0.0) {
        set_error(error, error_size,
                  "KSP returned invalid rails-warp state "
                  "(ut=%.9g factor=%d rate=%.9g latency=%.9g)",
                  state->ut, (int)state->factor, state->rate,
                  state->latency_seconds);
        return false;
    }
    return true;
}

static bool read_maximum_rails_factor(KrpcCNanoClient *client,
                                      int32_t *maximum_factor,
                                      char *error,
                                      size_t error_size) {
    if (!krpc_cnano_transport_begin_deadline(client->connection)) {
        set_error(error, error_size,
                  "Could not start the maximum-rails-factor response deadline");
        return false;
    }
    krpc_error_t code = krpc_SpaceCenter_MaximumRailsWarpFactor(
        client->connection, maximum_factor);
    krpc_cnano_transport_end_deadline(client->connection);
    if (rpc_failed(client, code, "SpaceCenter.MaximumRailsWarpFactor",
                   error, error_size))
        return false;

    size_t rate_count = sizeof(k_stock_rails_rate_by_factor) /
                        sizeof(k_stock_rails_rate_by_factor[0]);
    if (*maximum_factor < 0 || (size_t)*maximum_factor >= rate_count) {
        set_error(error, error_size,
                  "KSP returned unsupported maximum rails-warp factor %d",
                  (int)*maximum_factor);
        return false;
    }
    return true;
}

static double ut_resolution(double ut) {
    double adjacent = nextafter(ut, INFINITY);
    double resolution = adjacent - ut;
    return isfinite(resolution) && resolution > 0.0 ? resolution : DBL_EPSILON;
}

bool krpc_cnano_client_warp(KrpcCNanoClient *client, double ut,
                            char *error, size_t error_size) {
    if (!client || !client->connected || !isfinite(ut)) return false;

    /* SpaceCenter.WarpTo is intentionally not used on the serial protocol:
       it withholds its RPC response until the target UT, so a serial timeout
       leaves the server-side warp coroutine alive without a request channel
       available to stop it. This controller uses short, deadline-bounded RPCs
       and chooses a rail factor from a finite-response stopping margin. */
    int timeout_ms = krpc_cnano_transport_timeout_ms(client->connection);
    if (timeout_ms <= 0) {
        set_error(error, error_size,
                  "Rails warp requires a positive configured transport timeout");
        return false;
    }
    /* decision-literal: mathematical-numerical-requirement | Convert configured milliseconds to seconds for monotonic wall-time comparison. */
    double progress_timeout = (double)timeout_ms / 1000.0;

    int32_t observed_factor = -1;
    double stop_latency = 0.0;
    if (!set_rails_warp_factor_observed(
            client, 0, &observed_factor, &stop_latency,
            error, error_size))
        return false;
    double factor_command_latency = stop_latency;

    int32_t maximum_factor = 0;
    if (!read_maximum_rails_factor(
            client, &maximum_factor, error, error_size))
        return false;
    if (maximum_factor < 1) {
        set_error(error, error_size,
                  "KSP reports that on-rails time warp is unavailable");
        return false;
    }

    CNanoWarpState state = {0};
    if (!read_warp_state(client, &state, error, error_size))
        return false;
    if (ut <= state.ut) {
        reset_rate_filters(client);
        return true;
    }

    double rates[sizeof(k_stock_rails_rate_by_factor) /
                 sizeof(k_stock_rails_rate_by_factor[0])];
    memcpy(rates, k_stock_rails_rate_by_factor, sizeof(rates));
    if ((size_t)state.factor < sizeof(rates) / sizeof(rates[0]))
        rates[state.factor] = state.rate;

    double last_progress_ut = state.ut;
    double last_progress_wall = monotonic_seconds();

    for (;;) {
        double remaining = ut - state.ut;
        if (remaining <= 0.0) {
            if (state.factor != 0) {
                double latency = 0.0;
                if (!set_rails_warp_factor_observed(
                        client, 0, &observed_factor, &latency,
                        error, error_size))
                    return false;
            }
            reset_rate_filters(client);
            return true;
        }

        if ((size_t)state.factor < sizeof(rates) / sizeof(rates[0]))
            rates[state.factor] = state.rate;
        /* Finite-response margin includes one factor command/readback,
           one state observation, and the eventual factor-zero command. */
        double response_horizon =
            factor_command_latency + state.latency_seconds + stop_latency;
        KrpcCNanoWarpDecision decision = {0};
        if (!warp_decision_from_rates(
                remaining, response_horizon, maximum_factor,
                rates, sizeof(rates) / sizeof(rates[0]), &decision)) {
            set_error(error, error_size,
                      "Could not derive a rails-warp factor from "
                      "remaining UT %.9g and response horizon %.9g",
                      remaining, response_horizon);
            return false;
        }

        if (decision.factor == 0) {
            if (state.factor != 0) {
                double latency = 0.0;
                if (!set_rails_warp_factor_observed(
                        client, 0, &observed_factor, &latency,
                        error, error_size))
                    return false;
                if (latency > stop_latency) stop_latency = latency;
            }
            reset_rate_filters(client);
            return true;
        }

        if (state.factor != decision.factor) {
            double latency = 0.0;
            if (!set_rails_warp_factor_observed(
                    client, decision.factor, &observed_factor, &latency,
                    error, error_size))
                return false;
            if (latency > factor_command_latency)
                factor_command_latency = latency;
            if (observed_factor == 0) {
                set_error(error, error_size,
                          "KSP declined requested rails-warp factor %d",
                          decision.factor);
                return false;
            }
            if (observed_factor < decision.factor)
                maximum_factor = observed_factor;
        }

        CNanoWarpState next = {0};
        if (!read_warp_state(client, &next, error, error_size)) {
            (void)krpc_SpaceCenter_set_RailsWarpFactor(
                client->connection, 0);
            return false;
        }

        double wall = monotonic_seconds();
        double resolution = ut_resolution(last_progress_ut);
        if (next.ut > last_progress_ut + resolution) {
            last_progress_ut = next.ut;
            last_progress_wall = wall;
        } else if (wall - last_progress_wall >= progress_timeout) {
            (void)krpc_SpaceCenter_set_RailsWarpFactor(
                client->connection, 0);
            set_error(error, error_size,
                      "Rails warp made no measurable UT progress within "
                      "the configured %.3f s transport timeout",
                      progress_timeout);
            return false;
        }

        state = next;
    }
}
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
