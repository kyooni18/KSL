#include "CNanoFakeTransport.h"

#include <limits.h>
#include <string.h>

static size_t min_size(size_t a, size_t b) {
    return a < b ? a : b;
}

static KrpcCNanoTransportStatus fake_open(void *context);
static KrpcCNanoTransportStatus fake_close(void *context);
static KrpcCNanoTransportStatus fake_read(
    void *context, uint8_t *buffer, size_t requested, size_t *transferred,
    int timeout_ms);
static KrpcCNanoTransportStatus fake_write(
    void *context, const uint8_t *buffer, size_t requested, size_t *transferred,
    int timeout_ms);

static const KrpcCNanoTransportOps k_fake_ops = {
    fake_open,
    fake_close,
    fake_read,
    fake_write
};

static bool compact_rx(CNanoFakeTransport *fake, size_t required) {
    size_t remaining;

    if (required > CNANO_FAKE_BUFFER_CAPACITY) {
        return false;
    }
    if (CNANO_FAKE_BUFFER_CAPACITY - fake->rx_tail >= required) {
        return true;
    }

    remaining = fake->rx_tail - fake->rx_head;
    if (remaining != 0) {
        memmove(fake->rx, fake->rx + fake->rx_head, remaining);
    }
    fake->rx_head = 0;
    fake->rx_tail = remaining;
    return CNANO_FAKE_BUFFER_CAPACITY - fake->rx_tail >= required;
}

/* Returns 1 for complete, 0 for incomplete, -1 for malformed/overflow. */
static int decode_varint_length(
    const uint8_t *data,
    size_t available,
    size_t *value,
    size_t *prefix_size) {
    uint64_t result = 0;
    unsigned shift = 0;
    size_t i;

    for (i = 0; i < available && i < 10; ++i) {
        uint8_t byte = data[i];
        if (i == 9 && byte > 1) {
            return -1;
        }
        result |= ((uint64_t)(byte & 0x7fu)) << shift;
        if ((byte & 0x80u) == 0) {
            if (result > (uint64_t)SIZE_MAX) {
                return -1;
            }
            *value = (size_t)result;
            *prefix_size = i + 1;
            return 1;
        }
        shift += 7;
    }

    if (available >= 10) {
        return -1;
    }
    return 0;
}

static void set_pending_read_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status) {
    fake->pending_read_status = status;
    fake->has_pending_read_status = true;
}

static void scan_requests(CNanoFakeTransport *fake) {
    while (fake->tx_parse_offset < fake->tx_size && !fake->protocol_error) {
        size_t payload_size = 0;
        size_t prefix_size = 0;
        size_t available = fake->tx_size - fake->tx_parse_offset;
        int parsed = decode_varint_length(
            fake->tx + fake->tx_parse_offset, available, &payload_size, &prefix_size);

        if (parsed == 0) {
            return;
        }
        if (parsed < 0 || payload_size > available - prefix_size) {
            if (parsed < 0) {
                fake->protocol_error = true;
                set_pending_read_status(fake, KRPC_CNANO_TRANSPORT_IO_ERROR);
            }
            return;
        }
        if (fake->request_count >= CNANO_FAKE_MAX_FRAMES) {
            fake->protocol_error = true;
            set_pending_read_status(fake, KRPC_CNANO_TRANSPORT_IO_ERROR);
            return;
        }

        {
            CNanoFakeFrame *frame = &fake->requests[fake->request_count];
            KrpcCNanoTransportStatus response_status = KRPC_CNANO_TRANSPORT_OK;
            size_t index = fake->request_count;

            frame->framed_offset = fake->tx_parse_offset;
            frame->framed_size = prefix_size + payload_size;
            frame->payload_offset = fake->tx_parse_offset + prefix_size;
            frame->payload_size = payload_size;
            fake->request_count++;
            fake->tx_parse_offset += frame->framed_size;

            if (fake->responder != NULL) {
                response_status = fake->responder(
                    fake,
                    index,
                    fake->tx + frame->payload_offset,
                    frame->payload_size,
                    fake->responder_context);
                if (response_status != KRPC_CNANO_TRANSPORT_OK) {
                    set_pending_read_status(fake, response_status);
                }
            }
        }
    }
}

void cnano_fake_init(CNanoFakeTransport *fake) {
    if (fake == NULL) {
        return;
    }
    memset(fake, 0, sizeof(*fake));
    fake->empty_read_status = KRPC_CNANO_TRANSPORT_TIMEOUT;
    fake->open_status = KRPC_CNANO_TRANSPORT_OK;
    fake->close_status = KRPC_CNANO_TRANSPORT_OK;
}

void cnano_fake_clear_buffers(CNanoFakeTransport *fake) {
    if (fake == NULL) {
        return;
    }
    fake->rx_head = 0;
    fake->rx_tail = 0;
    fake->tx_size = 0;
    fake->tx_parse_offset = 0;
    fake->request_count = 0;
    fake->read_step_count = 0;
    fake->read_step_index = 0;
    fake->write_step_count = 0;
    fake->write_step_index = 0;
    fake->has_pending_read_status = false;
    fake->protocol_error = false;
}

KrpcCNanoTransportConfig cnano_fake_config(CNanoFakeTransport *fake) {
    return krpc_cnano_transport_config(
        &fake->connection_storage, &k_fake_ops, fake, 0);
}

void cnano_fake_set_chunk_limits(CNanoFakeTransport *fake, size_t read_limit, size_t write_limit) {
    if (fake == NULL) {
        return;
    }
    fake->read_chunk_limit = read_limit;
    fake->write_chunk_limit = write_limit;
}

void cnano_fake_set_empty_read_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status) {
    if (fake != NULL) {
        fake->empty_read_status = status;
    }
}

void cnano_fake_set_open_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status) {
    if (fake != NULL) {
        fake->open_status = status;
    }
}

void cnano_fake_set_close_status(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status) {
    if (fake != NULL) {
        fake->close_status = status;
    }
}

void cnano_fake_set_responder(CNanoFakeTransport *fake, CNanoFakeResponder responder, void *context) {
    if (fake == NULL) {
        return;
    }
    fake->responder = responder;
    fake->responder_context = context;
}

bool cnano_fake_script_read(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status, size_t max_bytes) {
    if (fake == NULL || fake->read_step_count >= CNANO_FAKE_MAX_IO_STEPS) {
        return false;
    }
    fake->read_steps[fake->read_step_count].status = status;
    fake->read_steps[fake->read_step_count].max_bytes = max_bytes;
    fake->read_step_count++;
    return true;
}

bool cnano_fake_script_write(CNanoFakeTransport *fake, KrpcCNanoTransportStatus status, size_t max_bytes) {
    if (fake == NULL || fake->write_step_count >= CNANO_FAKE_MAX_IO_STEPS) {
        return false;
    }
    fake->write_steps[fake->write_step_count].status = status;
    fake->write_steps[fake->write_step_count].max_bytes = max_bytes;
    fake->write_step_count++;
    return true;
}

bool cnano_fake_queue_read(CNanoFakeTransport *fake, const uint8_t *data, size_t size) {
    if (fake == NULL || (data == NULL && size != 0) || !compact_rx(fake, size)) {
        return false;
    }
    if (size != 0) {
        memcpy(fake->rx + fake->rx_tail, data, size);
        fake->rx_tail += size;
    }
    return true;
}

bool cnano_fake_queue_delimited_payload(CNanoFakeTransport *fake, const uint8_t *payload, size_t size) {
    uint8_t prefix[10];
    size_t prefix_size = 0;
    size_t value = size;

    do {
        uint8_t byte = (uint8_t)(value & 0x7fu);
        value >>= 7;
        if (value != 0) {
            byte |= 0x80u;
        }
        prefix[prefix_size++] = byte;
    } while (value != 0 && prefix_size < sizeof(prefix));

    return cnano_fake_queue_read(fake, prefix, prefix_size) &&
           cnano_fake_queue_read(fake, payload, size);
}

size_t cnano_fake_request_count(const CNanoFakeTransport *fake) {
    return fake != NULL ? fake->request_count : 0;
}

bool cnano_fake_request_payload(
    const CNanoFakeTransport *fake,
    size_t index,
    const uint8_t **payload,
    size_t *payload_size) {
    const CNanoFakeFrame *frame;

    if (fake == NULL || payload == NULL || payload_size == NULL || index >= fake->request_count) {
        return false;
    }
    frame = &fake->requests[index];
    *payload = fake->tx + frame->payload_offset;
    *payload_size = frame->payload_size;
    return true;
}

const uint8_t *cnano_fake_written_data(const CNanoFakeTransport *fake) {
    return fake != NULL ? fake->tx : NULL;
}

size_t cnano_fake_written_size(const CNanoFakeTransport *fake) {
    return fake != NULL ? fake->tx_size : 0;
}

static KrpcCNanoTransportStatus fake_open(void *context) {
    CNanoFakeTransport *fake = (CNanoFakeTransport *)context;
    if (fake == NULL) {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    fake->open_count++;
    if (fake->open_status == KRPC_CNANO_TRANSPORT_OK) {
        fake->opened = true;
    }
    return fake->open_status;
}

static KrpcCNanoTransportStatus fake_close(void *context) {
    CNanoFakeTransport *fake = (CNanoFakeTransport *)context;
    if (fake == NULL) {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    fake->close_count++;
    fake->opened = false;
    return fake->close_status;
}

static KrpcCNanoTransportStatus fake_read(
    void *context, uint8_t *buffer, size_t requested, size_t *transferred,
    int timeout_ms) {
    (void)timeout_ms;
    CNanoFakeTransport *fake = (CNanoFakeTransport *)context;
    CNanoFakeIOStep step = {KRPC_CNANO_TRANSPORT_OK, SIZE_MAX};
    size_t available;
    size_t amount;

    if (transferred == NULL) {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    *transferred = 0;
    if (fake == NULL || !fake->opened) {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    if (fake->has_pending_read_status) {
        fake->has_pending_read_status = false;
        return fake->pending_read_status;
    }
    if (fake->read_step_index < fake->read_step_count) {
        step = fake->read_steps[fake->read_step_index++];
    }
    if (step.status != KRPC_CNANO_TRANSPORT_OK) {
        return step.status;
    }

    available = fake->rx_tail - fake->rx_head;
    if (available == 0) {
        return fake->empty_read_status;
    }

    amount = min_size(requested, available);
    if (fake->read_chunk_limit != 0) {
        amount = min_size(amount, fake->read_chunk_limit);
    }
    amount = min_size(amount, step.max_bytes);
    if (amount == 0) {
        return KRPC_CNANO_TRANSPORT_OK;
    }

    memcpy(buffer, fake->rx + fake->rx_head, amount);
    fake->rx_head += amount;
    if (fake->rx_head == fake->rx_tail) {
        fake->rx_head = 0;
        fake->rx_tail = 0;
    }
    *transferred = amount;
    return KRPC_CNANO_TRANSPORT_OK;
}

static KrpcCNanoTransportStatus fake_write(
    void *context, const uint8_t *buffer, size_t requested, size_t *transferred,
    int timeout_ms) {
    (void)timeout_ms;
    CNanoFakeTransport *fake = (CNanoFakeTransport *)context;
    CNanoFakeIOStep step = {KRPC_CNANO_TRANSPORT_OK, SIZE_MAX};
    size_t amount = requested;

    if (transferred == NULL) {
        return KRPC_CNANO_TRANSPORT_INVALID;
    }
    *transferred = 0;
    if (fake == NULL || !fake->opened) {
        return KRPC_CNANO_TRANSPORT_CLOSED;
    }
    if (fake->protocol_error) {
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
    if (fake->write_step_index < fake->write_step_count) {
        step = fake->write_steps[fake->write_step_index++];
    }
    if (step.status != KRPC_CNANO_TRANSPORT_OK) {
        return step.status;
    }

    if (fake->write_chunk_limit != 0) {
        amount = min_size(amount, fake->write_chunk_limit);
    }
    amount = min_size(amount, step.max_bytes);
    if (amount > CNANO_FAKE_BUFFER_CAPACITY - fake->tx_size) {
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
    if (amount == 0) {
        return KRPC_CNANO_TRANSPORT_OK;
    }

    memcpy(fake->tx + fake->tx_size, buffer, amount);
    fake->tx_size += amount;
    *transferred = amount;
    scan_requests(fake);
    return KRPC_CNANO_TRANSPORT_OK;
}
