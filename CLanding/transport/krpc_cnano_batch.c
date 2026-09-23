#include "krpc_cnano_batch.h"

#include <krpc_cnano/pb_decode.h>
#include <krpc_cnano/pb_encode.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    bool failed;
} BatchErrorState;


static bool batch_write_callback(pb_ostream_t *stream,const uint8_t *buffer,size_t count){
    krpc_connection_t connection=(krpc_connection_t)(intptr_t)stream->state;
    return krpc_write(connection,buffer,count)==KRPC_OK;
}

static bool batch_read_callback(pb_istream_t *stream,uint8_t *buffer,size_t count){
    krpc_connection_t connection=(krpc_connection_t)(intptr_t)stream->state;
    krpc_error_t result=krpc_read(connection,buffer,count);
    if(result==KRPC_ERROR_EOF)stream->bytes_left=0;
    return result==KRPC_OK;
}

static bool batch_error_decoder(pb_istream_t *stream,const pb_field_t *field,void **arg){
    (void)field;
    BatchErrorState *state=(BatchErrorState *)(*arg);if(state)state->failed=true;
    uint8_t scratch[64];
    while(stream->bytes_left){
        size_t take=stream->bytes_left<sizeof(scratch)?stream->bytes_left:sizeof(scratch);
        if(!pb_read(stream,scratch,take))return false;
    }
    return true;
}

krpc_error_t krpc_cnano_invoke_batch(krpc_connection_t connection,
                                     const krpc_schema_ProcedureCall *calls,
                                     krpc_result_t *results,
                                     bool *result_failed,
                                     size_t count){
    if(!connection||!calls||!results||count==0||count>KRPC_CNANO_BATCH_MAX)
        return KRPC_ERROR_ENCODING_FAILED;

    krpc_schema_MultiplexedRequest request=krpc_schema_MultiplexedRequest_init_default;
    request.has_request=true;request.request.calls_count=(pb_size_t)count;
    for(size_t i=0;i<count;i++)request.request.calls[i]=calls[i];
    pb_ostream_t ostream={0};
    ostream.callback=&batch_write_callback;ostream.state=(void *)(intptr_t)connection;ostream.max_size=SIZE_MAX;
    if(!pb_encode_delimited(&ostream,krpc_schema_MultiplexedRequest_fields,&request))
        return KRPC_ERROR_ENCODING_FAILED;

    if(!krpc_cnano_transport_begin_deadline(connection))
        return KRPC_ERROR_IO;
#define BATCH_RESPONSE_RETURN(value) do { \
    krpc_cnano_transport_end_deadline(connection); \
    return (value); \
} while(0)

    for(;;){
        krpc_schema_MultiplexedResponse response=krpc_schema_MultiplexedResponse_init_default;
        BatchErrorState response_error={0};
        BatchErrorState item_errors[KRPC_CNANO_BATCH_MAX];
        memset(item_errors,0,sizeof(item_errors));
        response.response.error.funcs.decode=&batch_error_decoder;
        response.response.error.arg=&response_error;
        for(size_t i=0;i<count;i++){
            response.response.results[i]=results[i].message;
            response.response.results[i].error.funcs.decode=&batch_error_decoder;
            response.response.results[i].error.arg=&item_errors[i];
        }
        pb_istream_t istream={0};
        istream.callback=&batch_read_callback;istream.state=(void *)(intptr_t)connection;istream.bytes_left=SIZE_MAX;
        if(!pb_decode_delimited(&istream,krpc_schema_MultiplexedResponse_fields,&response))
            BATCH_RESPONSE_RETURN(KRPC_ERROR_DECODING_FAILED);

        /* The serial kRPC connection is multiplexed. A stream update or stale
         * empty RPC response may precede the response to the request just
         * written. Do not resend: direct-control writes are not idempotent.
         * Continue consuming protocol-valid unrelated frames until the
         * transport's configured response deadline expires. */
        if(!response.has_response){
            if(response.has_stream_update)continue;
            BATCH_RESPONSE_RETURN(KRPC_ERROR_NO_RESULTS);
        }
        if(response_error.failed)BATCH_RESPONSE_RETURN(KRPC_ERROR_RPC_FAILED);
        if(response.response.results_count==0)continue;
        if(response.response.results_count!=(pb_size_t)count)
            BATCH_RESPONSE_RETURN(KRPC_ERROR_NO_RESULTS);
        for(size_t i=0;i<count;i++){
            results[i].message=response.response.results[i];
            if(result_failed)result_failed[i]=item_errors[i].failed;
        }
        BATCH_RESPONSE_RETURN(KRPC_OK);
    }
#undef BATCH_RESPONSE_RETURN
}
