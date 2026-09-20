#include "CNanoFakeTransport.h"
#include "../CLanding/krpc_cnano_batch.h"

#include <krpc_cnano/decoder.h>
#include <krpc_cnano/encoder.h>
#include <krpc_cnano/pb_decode.h>
#include <krpc_cnano/pb_encode.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    size_t calls;
    size_t observed_batch_size;
    size_t prepend_empty_responses;
} BatchResponderState;

static KrpcCNanoTransportStatus batch_responder(
    CNanoFakeTransport *fake,
    size_t request_index,
    const uint8_t *payload,
    size_t payload_size,
    void *context) {
    (void)request_index;
    BatchResponderState *state = (BatchResponderState *)context;
    assert(state != NULL);
    assert(payload != NULL);

    pb_istream_t input = pb_istream_from_buffer(payload, payload_size);
    krpc_schema_MultiplexedRequest request = krpc_schema_MultiplexedRequest_init_default;
    assert(pb_decode(&input, krpc_schema_MultiplexedRequest_fields, &request));
    assert(request.has_request);
    assert(request.request.calls_count == 3);
    for (size_t i = 0; i < request.request.calls_count; ++i) {
        assert(request.request.calls[i].service_id == 2);
        assert(request.request.calls[i].procedure_id == 51 + i);
    }
    state->calls++;
    state->observed_batch_size = request.request.calls_count;

    for (size_t skipped = 0; skipped < state->prepend_empty_responses; ++skipped) {
        krpc_schema_MultiplexedResponse empty = krpc_schema_MultiplexedResponse_init_default;
        empty.has_response = true;
        uint8_t encoded_empty[64];
        pb_ostream_t empty_output = pb_ostream_from_buffer(encoded_empty, sizeof(encoded_empty));
        assert(pb_encode(&empty_output, krpc_schema_MultiplexedResponse_fields, &empty));
        if (!cnano_fake_queue_delimited_payload(fake, encoded_empty, empty_output.bytes_written))
            return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }

    double values[3] = {123.25, 456.5, 789.75};
    krpc_schema_MultiplexedResponse response = krpc_schema_MultiplexedResponse_init_default;
    response.has_response = true;
    response.response.results_count = 3;
    for (size_t i = 0; i < 3; ++i) {
        response.response.results[i].value.funcs.encode = &krpc_encode_callback_double;
        response.response.results[i].value.arg = &values[i];
    }

    uint8_t encoded[512];
    pb_ostream_t output = pb_ostream_from_buffer(encoded, sizeof(encoded));
    assert(pb_encode(&output, krpc_schema_MultiplexedResponse_fields, &response));
    return cnano_fake_queue_delimited_payload(fake, encoded, output.bytes_written)
               ? KRPC_CNANO_TRANSPORT_OK
               : KRPC_CNANO_TRANSPORT_IO_ERROR;
}

static void test_three_calls_one_wire_request(void) {
    CNanoFakeTransport fake;
    cnano_fake_init(&fake);
    cnano_fake_set_chunk_limits(&fake, 2, 3);
    BatchResponderState state = {0};
    cnano_fake_set_responder(&fake, batch_responder, &state);

    KrpcCNanoTransportConfig config = cnano_fake_config(&fake);
    krpc_connection_t connection = NULL;
    assert(krpc_open(&connection, &config) == KRPC_OK);

    krpc_call_t calls[3];
    krpc_schema_ProcedureCall messages[3];
    krpc_result_t results[3];
    bool failed[3] = {true, true, true};
    for (size_t i = 0; i < 3; ++i) {
        assert(krpc_call(&calls[i], 2, (uint32_t)(51 + i), 0, NULL) == KRPC_OK);
        messages[i] = calls[i].message;
        results[i] = (krpc_result_t)KRPC_RESULT_INIT_DEFAULT;
        assert(krpc_init_result(&results[i]) == KRPC_OK);
    }

    assert(krpc_cnano_invoke_batch(connection, messages, results, failed, 3) == KRPC_OK);
    assert(state.calls == 1);
    assert(state.observed_batch_size == 3);
    assert(cnano_fake_request_count(&fake) == 1);

    const double expected[3] = {123.25, 456.5, 789.75};
    for (size_t i = 0; i < 3; ++i) {
        assert(!failed[i]);
        pb_istream_t value_stream;
        assert(krpc_get_return_value(&results[i], &value_stream) == KRPC_OK);
        double value = 0.0;
        assert(krpc_decode_double(&value_stream, &value) == KRPC_OK);
        assert(fabs(value - expected[i]) < 1e-12);
        assert(krpc_free_result(&results[i]) == KRPC_OK);
    }
    assert(krpc_close(connection) == KRPC_OK);
}

static void test_many_empty_responses_before_batch_are_skipped(void) {
    CNanoFakeTransport fake;
    cnano_fake_init(&fake);
    cnano_fake_set_chunk_limits(&fake, 2, 3);
    BatchResponderState state = {.prepend_empty_responses = 12};
    cnano_fake_set_responder(&fake, batch_responder, &state);

    KrpcCNanoTransportConfig config = cnano_fake_config(&fake);
    krpc_connection_t connection = NULL;
    assert(krpc_open(&connection, &config) == KRPC_OK);

    krpc_call_t calls[3];
    krpc_schema_ProcedureCall messages[3];
    krpc_result_t results[3];
    bool failed[3] = {true, true, true};
    for (size_t i = 0; i < 3; ++i) {
        assert(krpc_call(&calls[i], 2, (uint32_t)(51 + i), 0, NULL) == KRPC_OK);
        messages[i] = calls[i].message;
        results[i] = (krpc_result_t)KRPC_RESULT_INIT_DEFAULT;
        assert(krpc_init_result(&results[i]) == KRPC_OK);
    }

    assert(krpc_cnano_invoke_batch(connection, messages, results, failed, 3) == KRPC_OK);
    assert(state.calls == 1);
    assert(cnano_fake_request_count(&fake) == 1);
    const double expected[3] = {123.25, 456.5, 789.75};
    for (size_t i = 0; i < 3; ++i) {
        assert(!failed[i]);
        pb_istream_t value_stream;
        double value = 0.0;
        assert(krpc_get_return_value(&results[i], &value_stream) == KRPC_OK);
        assert(krpc_decode_double(&value_stream, &value) == KRPC_OK);
        assert(fabs(value - expected[i]) < 1e-12);
        assert(krpc_free_result(&results[i]) == KRPC_OK);
    }
    assert(krpc_close(connection) == KRPC_OK);
}

int main(void) {
    test_three_calls_one_wire_request();
    test_many_empty_responses_before_batch_are_skipped();
    puts("C-Nano batch tests passed.");
    return 0;
}
