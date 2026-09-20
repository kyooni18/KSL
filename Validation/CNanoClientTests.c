#include "CNanoFakeTransport.h"
#include "../CLanding/krpc_cnano_client.h"
#include "../CLanding/krpc_cnano_batch.h"

#include <krpc_cnano/encoder.h>
#include <krpc_cnano/pb_decode.h>
#include <krpc_cnano/pb_encode.h>
#include <krpc_cnano/services/space_center.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    double ut;
    size_t surface_position_calls;
    size_t pressure_calls;
    size_t density_calls;
    size_t request_frames;
    size_t largest_batch;
    size_t set_direction_and_up_calls;
    size_t set_target_direction_calls;
    size_t set_target_roll_calls;
    size_t reset_autopilot_calls;
    size_t set_auto_tune_calls;
    size_t set_time_to_peak_calls;
    size_t set_soft_start_calls;
    size_t set_overshoot_calls;
    size_t pause_calls;
} ClientServerState;

typedef struct {
    krpc_object_t objects[KRPC_CNANO_BATCH_MAX];
    double doubles[KRPC_CNANO_BATCH_MAX];
    float floats[KRPC_CNANO_BATCH_MAX];
    bool bools[KRPC_CNANO_BATCH_MAX];
    krpc_enum_t enums[KRPC_CNANO_BATCH_MAX];
    const char *strings[KRPC_CNANO_BATCH_MAX];
    krpc_tuple_double_double_double_t tuple3[KRPC_CNANO_BATCH_MAX];
    krpc_tuple_double_double_double_double_t tuple4[KRPC_CNANO_BATCH_MAX];
    krpc_tuple_tuple_double_double_double_tuple_double_double_double_t torques[KRPC_CNANO_BATCH_MAX];
} ResponseValues;

static void result_object(krpc_schema_ProcedureResult *r,krpc_object_t *value){
    r->value.funcs.encode=&krpc_encode_callback_object;r->value.arg=value;
}
static void result_double(krpc_schema_ProcedureResult *r,double *value){
    r->value.funcs.encode=&krpc_encode_callback_double;r->value.arg=value;
}
static void result_float(krpc_schema_ProcedureResult *r,float *value){
    r->value.funcs.encode=&krpc_encode_callback_float;r->value.arg=value;
}
static void result_bool(krpc_schema_ProcedureResult *r,bool *value){
    r->value.funcs.encode=&krpc_encode_callback_bool;r->value.arg=value;
}
static void result_enum(krpc_schema_ProcedureResult *r,krpc_enum_t *value){
    r->value.funcs.encode=&krpc_encode_callback_enum;r->value.arg=value;
}
static void result_string(krpc_schema_ProcedureResult *r,const char **value){
    r->value.funcs.encode=&krpc_encode_callback_string;r->value.arg=value;
}
static void result_tuple3(krpc_schema_ProcedureResult *r,krpc_tuple_double_double_double_t *value){
    r->value.funcs.encode=&krpc_encode_callback_tuple_double_double_double;r->value.arg=value;
}
static void result_tuple4(krpc_schema_ProcedureResult *r,krpc_tuple_double_double_double_double_t *value){
    r->value.funcs.encode=&krpc_encode_callback_tuple_double_double_double_double;r->value.arg=value;
}
static void result_torque(krpc_schema_ProcedureResult *r,krpc_tuple_tuple_double_double_double_tuple_double_double_double_t *value){
    r->value.funcs.encode=&krpc_encode_callback_tuple_tuple_double_double_double_tuple_double_double_double;r->value.arg=value;
}

static void configure_result(ClientServerState *state,uint32_t proc,size_t i,
                             krpc_schema_ProcedureResult *result,ResponseValues *values){
    switch(proc){
        case 30: values->objects[i]=1001;result_object(result,&values->objects[i]);break; /* ActiveVessel */
        case 655: values->objects[i]=2001;result_object(result,&values->objects[i]);break; /* Control */
        case 657: values->objects[i]=2002;result_object(result,&values->objects[i]);break; /* AutoPilot */
        case 654: values->objects[i]=2003;result_object(result,&values->objects[i]);break; /* Orbit */
        case 558: values->objects[i]=3001;result_object(result,&values->objects[i]);break; /* Body */
        case 270: values->objects[i]=3002;result_object(result,&values->objects[i]);break; /* body RF */
        case 698: values->objects[i]=4001;result_object(result,&values->objects[i]);break; /* vessel RF */
        case 700: values->objects[i]=4002;result_object(result,&values->objects[i]);break; /* surface RF */
        case 627: values->objects[i]=5001;result_object(result,&values->objects[i]);break; /* Flight */
        case 668: values->objects[i]=0;result_object(result,&values->objects[i]);break; /* Parts: skip inventory */
        case 642: values->strings[i]="STS-N";result_string(result,&values->strings[i]);break;
        case 249: values->strings[i]="Kerbin";result_string(result,&values->strings[i]);break;
        case 258: values->doubles[i]=600000.0;result_double(result,&values->doubles[i]);break;
        case 252: values->doubles[i]=3.5316e12;result_double(result,&values->doubles[i]);break;
        case 255: values->doubles[i]=0.00029157090055980246;result_double(result,&values->doubles[i]);break;
        case 264: values->doubles[i]=70000.0;result_double(result,&values->doubles[i]);break;
        case 51: values->doubles[i]=state->ut;result_double(result,&values->doubles[i]);break;
        case 233:
            values->tuple3[i]=state->surface_position_calls++==0
                ? (krpc_tuple_double_double_double_t){0.0,600000.0,0.0}
                : (krpc_tuple_double_double_double_t){600000.0,0.0,0.0};
            result_tuple3(result,&values->tuple3[i]);break;
        case 242: {
            double altitude=70000.0*(double)(state->pressure_calls++)/70.0;
            values->doubles[i]=101325.0*exp(-altitude/5600.0);result_double(result,&values->doubles[i]);break;
        }
        case 241: {
            double altitude=70000.0*(double)(state->density_calls++)/70.0;
            values->doubles[i]=1.225*exp(-altitude/5600.0);result_double(result,&values->doubles[i]);break;
        }
        case 652: values->doubles[i]=100.0;result_double(result,&values->doubles[i]);break; /* MET */
        case 465: values->doubles[i]=12000.0;result_double(result,&values->doubles[i]);break;
        case 503: values->floats[i]=250.0f;result_float(result,&values->floats[i]);break;
        case 479: values->floats[i]=90.0f;result_float(result,&values->floats[i]);break;
        case 478: values->floats[i]=12.0f;result_float(result,&values->floats[i]);break;
        case 480: values->floats[i]=0.0f;result_float(result,&values->floats[i]);break;
        case 506: values->floats[i]=12.0f;result_float(result,&values->floats[i]);break;
        case 507: values->floats[i]=0.0f;result_float(result,&values->floats[i]);break;
        case 490: values->floats[i]=15000.0f;result_float(result,&values->floats[i]);break;
        case 636: values->tuple3[i]=(krpc_tuple_double_double_double_t){612000.0,0.0,0.0};result_tuple3(result,&values->tuple3[i]);break;
        case 638: values->tuple3[i]=(krpc_tuple_double_double_double_t){0.0,0.0,428.4};result_tuple3(result,&values->tuple3[i]);break;
        case 641: values->tuple3[i]=(krpc_tuple_double_double_double_t){0.1,0.2,0.3};result_tuple3(result,&values->tuple3[i]);break;
        case 669: values->floats[i]=30000.0f;result_float(result,&values->floats[i]);break;
        case 403: case 407: case 405: case 399: values->floats[i]=0.0f;result_float(result,&values->floats[i]);break;
        case 138: values->floats[i]=27.5f;result_float(result,&values->floats[i]);break;
        case 170: values->tuple3[i]=(krpc_tuple_double_double_double_t){100.0,200.0,300.0};result_tuple3(result,&values->tuple3[i]);break;
        case 639: values->tuple4[i]=(krpc_tuple_double_double_double_double_t){0.0,0.0,0.0,1.0};result_tuple4(result,&values->tuple4[i]);break;
        case 500: values->floats[i]=340.0f;result_float(result,&values->floats[i]);break;
        case 492: values->floats[i]=20000.0f;result_float(result,&values->floats[i]);break;
        case 463: values->floats[i]=1.0f;result_float(result,&values->floats[i]);break;
        case 670: values->floats[i]=18000.0f;result_float(result,&values->floats[i]);break;
        case 672: case 671: values->floats[i]=0.0f;result_float(result,&values->floats[i]);break;
        case 684:
            values->torques[i].e0=(krpc_tuple_double_double_double_t){120000.0,90000.0,80000.0};
            values->torques[i].e1=(krpc_tuple_double_double_double_t){-120000.0,-90000.0,-80000.0};
            result_torque(result,&values->torques[i]);break;
        case 682: values->tuple3[i]=(krpc_tuple_double_double_double_t){400000.0,350000.0,300000.0};result_tuple3(result,&values->tuple3[i]);break;
        case 495: values->tuple3[i]=(krpc_tuple_double_double_double_t){0.0,50000.0,0.0};result_tuple3(result,&values->tuple3[i]);break;
        case 496: values->tuple3[i]=(krpc_tuple_double_double_double_t){-10000.0,0.0,0.0};result_tuple3(result,&values->tuple3[i]);break;
        case 475: values->tuple3[i]=(krpc_tuple_double_double_double_t){0.0,0.0,0.0};result_tuple3(result,&values->tuple3[i]);break;
        case 371: case 379: case 347: case 361: case 355: case 111: values->bools[i]=false;result_bool(result,&values->bools[i]);break;
        case 359: values->enums[i]=KRPC_SPACECENTER_SPEEDMODE_SURFACE;result_enum(result,&values->enums[i]);break;
        case 646: values->enums[i]=KRPC_SPACECENTER_VESSELSITUATION_FLYING;result_enum(result,&values->enums[i]);break;
        case 561: values->doubles[i]=86000.0;result_double(result,&values->doubles[i]);break;
        case 562: values->doubles[i]=85000.0;result_double(result,&values->doubles[i]);break;
        case 567: values->doubles[i]=1800.0;result_double(result,&values->doubles[i]);break;
        default:
            /* Setters and other void procedures intentionally return an empty ProcedureResult. */
            break;
    }
}

static KrpcCNanoTransportStatus client_responder(CNanoFakeTransport *fake,size_t request_index,
                                                  const uint8_t *payload,size_t payload_size,void *context){
    (void)request_index;
    ClientServerState *state=(ClientServerState *)context;
    pb_istream_t input=pb_istream_from_buffer(payload,payload_size);
    krpc_schema_MultiplexedRequest request=krpc_schema_MultiplexedRequest_init_default;
    assert(pb_decode(&input,krpc_schema_MultiplexedRequest_fields,&request));
    state->request_frames++;
    if(request.has_connection_request){
        /* Empty ConnectionResponse => status OK. */
        return cnano_fake_queue_delimited_payload(fake,NULL,0)?KRPC_CNANO_TRANSPORT_OK:KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
    assert(request.has_request);
    if(request.request.calls_count>state->largest_batch)state->largest_batch=request.request.calls_count;
    bool fast=false;
    for(size_t i=0;i<request.request.calls_count;i++)if(request.request.calls[i].procedure_id==51&&request.request.calls_count>1)fast=true;
    if(fast)state->ut+=0.1;

    ResponseValues values={0};
    krpc_schema_MultiplexedResponse response=krpc_schema_MultiplexedResponse_init_default;
    response.has_response=true;
    response.response.results_count=request.request.calls_count;
    for(size_t i=0;i<request.request.calls_count;i++){
        uint32_t proc=request.request.calls[i].procedure_id;
        if(proc==105)state->set_direction_and_up_calls++;
        else if(proc==126)state->set_target_direction_calls++;
        else if(proc==122)state->set_target_roll_calls++;
        else if(proc==103)state->reset_autopilot_calls++;
        else if(proc==159)state->set_auto_tune_calls++;
        else if(proc==161)state->set_time_to_peak_calls++;
        else if(proc==163)state->set_soft_start_calls++;
        else if(proc==167)state->set_overshoot_calls++;
        else if(proc==15)state->pause_calls++;
        configure_result(state,proc,i,&response.response.results[i],&values);
    }

    uint8_t encoded[8192];pb_ostream_t output=pb_ostream_from_buffer(encoded,sizeof(encoded));
    assert(pb_encode(&output,krpc_schema_MultiplexedResponse_fields,&response));
    return cnano_fake_queue_delimited_payload(fake,encoded,output.bytes_written)?KRPC_CNANO_TRANSPORT_OK:KRPC_CNANO_TRANSPORT_IO_ERROR;
}

static void test_client_read_and_control_budget(void){
    CNanoFakeTransport fake;cnano_fake_init(&fake);cnano_fake_set_chunk_limits(&fake,7,11);
    ClientServerState server={.ut=1000.0};cnano_fake_set_responder(&fake,client_responder,&server);
    KrpcCNanoTransportConfig transport=cnano_fake_config(&fake);
    LandingConfiguration cfg=landing_configuration_default();
    char error[512]={0};
    KrpcCNanoClient *client=krpc_cnano_client_open_transport(&cfg,&transport,"fake-serial",error,sizeof(error));
    if(!client)fprintf(stderr,"client open: %s\n",error);
    assert(client);
    assert(strcmp(krpc_cnano_client_vessel(client),"STS-N")==0);
    assert(strcmp(krpc_cnano_client_planet(client)->name,"Kerbin")==0);
    assert(krpc_cnano_client_planet(client)->atmosphere_sample_count==71);
    assert(server.largest_batch==32);

    Telemetry t;VehicleState state;
    assert(krpc_cnano_client_read(client,&cfg,&t,&state,error,sizeof(error)));
    assert(fabs(t.ut-1000.1)<1e-9);
    assert(fabs(t.true_air_speed-250.0)<1e-6);
    assert(fabs(t.heading-90.0)<1e-6);
    assert(t.has_torque&&t.has_inertia&&t.has_force_vectors&&t.has_attitude_quaternion&&t.has_center_of_mass_root);
    assert(!t.stall_fraction_is_measured); /* C-Nano exposes only the conservative proxy. */
    assert(t.has_controls&&fabs(t.control_pitch)<1e-9);
    assert(t.has_body_pitch_rate&&t.has_body_roll_rate&&t.has_body_yaw_rate);
    assert(fabs(t.body_pitch_rate + 0.1*57.29577951308232)<1e-6);
    assert(fabs(t.body_roll_rate + 0.2*57.29577951308232)<1e-6);
    assert(fabs(t.body_yaw_rate + 0.3*57.29577951308232)<1e-6);
    assert(fabs(t.autopilot_error-180.0)<1e-6); /* getter is invalid while disengaged */
    assert(strcmp(t.vessel_situation,"flying")==0);
    assert(fabs(t.available_pitch_torque-120000.0)<1e-6);
    assert(fabs(t.roll_moment_of_inertia-350000.0)<1e-6);
    KrpcCNanoBudget budget=krpc_cnano_client_budget(client);
    assert(budget.read_calls==41);
    assert(budget.read_wire_requests==3);

    assert(krpc_cnano_client_read(client,&cfg,&t,&state,error,sizeof(error)));
    budget=krpc_cnano_client_budget(client);
    assert(fabs(t.ut-1000.2)<1e-9);
    assert(budget.read_calls==18);
    assert(budget.read_wire_requests==1);

    /* AutoPilot.Error is only legal after engagement on the real kRPC server.
       Exercise the exact orbital attitude contract used by krpc_apply: both
       retrograde and prograde must supply a radial-out up reference so C-Nano
       sends one SetDirectionAndUp target instead of the singular legacy pair. */
    VehicleState orbit_state={.position=v3(600000.0,0.0,0.0),.velocity=v3(0.0,2300.0,0.0)};
    GuidanceCommand orbital; memset(&orbital,0,sizeof(orbital));
    orbital.autopilot_engaged=true;
    orbital.use_inertial_direction=true;
    orbital.control_profile=PROFILE_ORBITAL;
    orbital.inertial_direction=v3(0.0,-1.0,0.0);
    Vector3 orbital_up={0};
    assert(krpc_orbital_up_reference(&orbital,&orbit_state,&orbital_up));
    assert(fabs(vdot(orbital_up,orbital.inertial_direction))<1e-9);
    assert(orbital_up.x>0.999999);
    size_t full_attitude_before=server.set_direction_and_up_calls;
    size_t direction_before=server.set_target_direction_calls;
    size_t roll_before=server.set_target_roll_calls;
    size_t reset_before=server.reset_autopilot_calls;
    size_t auto_tune_before=server.set_auto_tune_calls;
    size_t time_to_peak_before=server.set_time_to_peak_calls;
    size_t soft_start_before=server.set_soft_start_calls;
    size_t overshoot_before=server.set_overshoot_calls;
    assert(krpc_cnano_client_set_autopilot(client,&orbital,&orbital_up,error,sizeof(error)));
    assert(server.set_direction_and_up_calls==full_attitude_before+1);
    assert(server.set_target_direction_calls==direction_before);
    assert(server.set_target_roll_calls==roll_before);
    assert(server.reset_autopilot_calls==reset_before);
    assert(server.set_auto_tune_calls==auto_tune_before+1);
    assert(server.set_time_to_peak_calls==time_to_peak_before+1);
    assert(server.set_soft_start_calls==soft_start_before+1);
    assert(server.set_overshoot_calls==overshoot_before+1);
    assert(krpc_cnano_client_read(client,&cfg,&t,&state,error,sizeof(error)));
    assert(fabs(t.autopilot_error-27.5)<1e-6);

    orbital.inertial_direction=v3(0.0,1.0,0.0);
    assert(krpc_orbital_up_reference(&orbital,&orbit_state,&orbital_up));
    assert(fabs(vdot(orbital_up,orbital.inertial_direction))<1e-9);
    assert(orbital_up.x>0.999999);

    size_t reset_orbital=server.reset_autopilot_calls;
    size_t auto_tune_orbital=server.set_auto_tune_calls;
    assert(krpc_cnano_client_set_direct_controls(client,0.2,-0.1,0.03,0.0,0.0,false,error,sizeof(error)));
    budget=krpc_cnano_client_budget(client);
    assert(server.reset_autopilot_calls==reset_orbital+1); /* restore complete AP defaults */
    assert(server.set_auto_tune_calls==auto_tune_orbital); /* never freeze or retune manually */
    assert(budget.write_wire_requests==2); /* reset handoff + direct-control batch */
    assert(budget.write_calls==6); /* reset + xyz + wheel + release orbital RCS */

    assert(krpc_cnano_client_set_direct_controls(client,0.15,-0.08,0.02,0.0,0.0,false,error,sizeof(error)));
    budget=krpc_cnano_client_budget(client);
    assert(budget.write_wire_requests==1);
    assert(budget.write_calls==3); /* steady-state atmospheric axes in one frame */

    /* Exercise two seconds at the production 10 Hz cadence and turn the actual
       encoded byte/request counts into a pessimistic serial-line budget. 8N1
       uses ten line bits per byte; add 2 ms server/turnaround cost per request. */
    KrpcCNanoBudget cadence_start=krpc_cnano_client_budget(client);
    for(int tick=0;tick<20;tick++){
        assert(krpc_cnano_client_read(client,&cfg,&t,&state,error,sizeof(error)));
        double axis=0.08+0.002*(double)(tick%5);
        assert(krpc_cnano_client_set_direct_controls(client,axis,-axis*.5,axis*.2,0.0,0.0,false,error,sizeof(error)));
    }
    KrpcCNanoBudget cadence_end=krpc_cnano_client_budget(client);
    unsigned cadence_wires=cadence_end.total_wire_requests-cadence_start.total_wire_requests;
    size_t cadence_bytes=(cadence_end.bytes_read-cadence_start.bytes_read)+(cadence_end.bytes_written-cadence_start.bytes_written);
    double serial_seconds=(double)cadence_bytes*10.0/(double)cfg.connection.baud_rate+(double)cadence_wires*.002;
    double utilization=serial_seconds/2.0;
    assert(cadence_wires<=52);
    assert(utilization<0.50);
    printf("C-Nano 10 Hz budget: %u wire requests / 2 s, %zu bytes, %.1f%% pessimistic serial occupancy\n",
           cadence_wires,cadence_bytes,utilization*100.0);

    size_t frames_before_close=server.request_frames;
    krpc_cnano_client_close(client);
    assert(server.request_frames==frames_before_close+4); /* safe batch plus triple pause */
    assert(server.pause_calls==3); /* escape/close must leave KSP paused */
    assert(fake.close_count==1);
    puts("C-Nano client integration tests passed.");
}

static void test_warp_decision_response_envelope(void){
    KrpcCNanoWarpDecision near={0},far={0},fast_response={0},slow_response={0},limited={0};

    assert(krpc_cnano_client_warp_decision(100.0,0.25,7,&near));
    assert(krpc_cnano_client_warp_decision(10000.0,0.25,7,&far));
    assert(far.factor>=near.factor);
    assert(near.stopping_margin_ut>=0.0);
    assert(far.stopping_margin_ut>=0.0);

    assert(krpc_cnano_client_warp_decision(1000.0,0.05,7,&fast_response));
    assert(krpc_cnano_client_warp_decision(1000.0,1.0,7,&slow_response));
    assert(fast_response.factor>=slow_response.factor);
    assert(fast_response.stopping_margin_ut>=0.0);
    assert(slow_response.stopping_margin_ut>=0.0);

    assert(krpc_cnano_client_warp_decision(10000.0,0.05,2,&limited));
    assert(limited.factor<=2);
    assert(limited.stopping_margin_ut>=0.0);

    KrpcCNanoWarpDecision no_safe_warp={0};
    assert(krpc_cnano_client_warp_decision(0.01,1.0,7,&no_safe_warp));
    assert(no_safe_warp.factor==0);
    assert(no_safe_warp.stopping_margin_ut>=0.0);

    assert(!krpc_cnano_client_warp_decision(NAN,0.1,7,&near));
    assert(!krpc_cnano_client_warp_decision(100.0,0.0,7,&near));
    assert(!krpc_cnano_client_warp_decision(100.0,0.1,8,&near));
}

int main(void){
    test_warp_decision_response_envelope();
    test_client_read_and_control_budget();
    return 0;
}
