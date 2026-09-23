#ifndef KSP_LANDER_CNANO_FAKE_TRANSPORT_H
#define KSP_LANDER_CNANO_FAKE_TRANSPORT_H

#include "../CLanding/krpc_cnano_transport.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CNANO_FAKE_BUFFER_CAPACITY 65536u
#define CNANO_FAKE_MAX_FRAMES 128u
#define CNANO_FAKE_MAX_IO_STEPS 128u

typedef struct {
    KrpcCNanoTransportStatus status;
    size_t max_bytes;
} CNanoFakeIOStep;

typedef struct {
    size_t framed_offset;
    size_t framed_size;
    size_t payload_offset;
    size_t payload_size;
} CNanoFakeFrame;

typedef struct CNanoFakeTransport CNanoFakeTransport;

typedef KrpcCNanoTransportStatus (*CNanoFakeResponder)(
    CNanoFakeTransport *fake,
    size_t request_index,
    const uint8_t *payload,
    size_t payload_size,
    void *context);

struct CNanoFakeTransport {
    KrpcCNanoTransport connection_storage;

    uint8_t rx[CNANO_FAKE_BUFFER_CAPACITY];
    size_t rx_head;
    size_t rx_tail;

    uint8_t tx[CNANO_FAKE_BUFFER_CAPACITY];
    size_t tx_size;
    size_t tx_parse_offset;

    CNanoFakeFrame requests[CNANO_FAKE_MAX_FRAMES];
    size_t request_count;

    CNanoFakeIOStep read_steps[CNANO_FAKE_MAX_IO_STEPS];
    size_t read_step_count;
    size_t read_step_index;
    CNanoFakeIOStep write_steps[CNANO_FAKE_MAX_IO_STEPS];
    size_t write_step_count;
    size_t write_step_index;

    size_t read_chunk_limit;
    size_t write_chunk_limit;
    KrpcCNanoTransportStatus empty_read_status;
    KrpcCNanoTransportStatus open_status;
    KrpcCNanoTransportStatus close_status;
    KrpcCNanoTransportStatus pending_read_status;
    bool has_pending_read_status;

    CNanoFakeResponder responder;
    void *responder_context;

    bool opened;
    bool protocol_error;
    size_t open_count;
    size_t close_count;
};

void cnano_fake_init(CNanoFakeTransport *fake);
void cnano_fake_clear_buffers(CNanoFakeTransport *fake);
KrpcCNanoTransportConfig cnano_fake_config(CNanoFakeTransport *fake);

void cnano_fake_set_chunk_limits(CNanoFakeTransport *fake, size_t read_limit, size_t write_limit);
void cnano_fake_set_empty_read_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status);
void cnano_fake_set_open_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status);
void cnano_fake_set_close_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status);
void cnano_fake_set_responder(CNanoFakeTransport *fake, CNanoFakeResponder responder, void *context);

bool cnano_fake_script_read(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status, size_t max_bytes);
bool cnano_fake_script_write(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status, size_t max_bytes);

bool cnano_fake_queue_read(CNanoFakeTransport *fake, const uint8_t *data, size_t size);
bool cnano_fake_queue_delimited_payload(CNanoFakeTransport *fake, const uint8_t *payload, size_t size);

size_t cnano_fake_request_count(const CNanoFakeTransport *fake);
bool cnano_fake_request_payload(
    const CNanoFakeTransport *fake,
    size_t index,
    const uint8_t **payload,
    size_t *payload_size);
const uint8_t *cnano_fake_written_data(const CNanoFakeTransport *fake);
size_t cnano_fake_written_size(const CNanoFakeTransport *fake);

#endif
