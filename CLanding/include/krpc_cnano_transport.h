#ifndef KSP_LANDER_KRPC_CNANO_TRANSPORT_H
#define KSP_LANDER_KRPC_CNANO_TRANSPORT_H

/*
 * Project-owned communication shim for kRPC C-Nano 0.6.0.
 *
 * C-Nano's wire protocol remains the kRPC serial-port protocol. This shim
 * replaces only the byte I/O backend so production can use a real serial or
 * pseudo-terminal endpoint and offline tests can use a deterministic fake.
 * It is deliberately not an adapter for the ordinary kRPC RPC/stream TCP
 * ports.
 *
 * Include this header before any kRPC C-Nano header. C-Nano core translation
 * units are also built with this file pre-included so the custom connection
 * types are visible before <krpc_cnano/communication.h> is parsed.
 */

#define KRPC_COMMUNICATION_CUSTOM 1

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef KRPC_CNANO_SERIAL_PATH_CAPACITY
#define KRPC_CNANO_SERIAL_PATH_CAPACITY 512
#endif

typedef enum {
    KRPC_CNANO_TRANSPORT_OK = 0,
    KRPC_CNANO_TRANSPORT_EOF,
    KRPC_CNANO_TRANSPORT_TIMEOUT,
    KRPC_CNANO_TRANSPORT_IO_ERROR,
    KRPC_CNANO_TRANSPORT_CLOSED,
    KRPC_CNANO_TRANSPORT_INVALID,
    KRPC_CNANO_TRANSPORT_ZERO_PROGRESS
} KrpcCNanoTransportStatus;

typedef struct KrpcCNanoTransport KrpcCNanoTransport;
typedef struct KrpcCNanoTransportConfig KrpcCNanoTransportConfig;
typedef struct KrpcCNanoTransportOps KrpcCNanoTransportOps;

/* Types required by <krpc_cnano/communication.h> in custom mode. */
typedef KrpcCNanoTransport *krpc_connection_t;
typedef KrpcCNanoTransportConfig krpc_connection_config_t;

typedef KrpcCNanoTransportStatus (*KrpcCNanoTransportOpenFn)(void *context);
typedef KrpcCNanoTransportStatus (*KrpcCNanoTransportCloseFn)(void *context);
typedef KrpcCNanoTransportStatus (*KrpcCNanoTransportReadFn)(
    void *context, uint8_t *buffer, size_t requested, size_t *transferred,
    int timeout_ms);
typedef KrpcCNanoTransportStatus (*KrpcCNanoTransportWriteFn)(
    void *context, const uint8_t *buffer, size_t requested, size_t *transferred,
    int timeout_ms);

struct KrpcCNanoTransportOps {
    KrpcCNanoTransportOpenFn open;
    KrpcCNanoTransportCloseFn close;
    KrpcCNanoTransportReadFn read;
    KrpcCNanoTransportWriteFn write;
};

struct KrpcCNanoTransport {
    const KrpcCNanoTransportOps *ops;
    void *context;
    KrpcCNanoTransportStatus last_status;
    size_t bytes_read;
    size_t bytes_written;
    int timeout_ms;
    int64_t deadline_ns;
    bool deadline_active;
    bool is_open;
};

struct KrpcCNanoTransportConfig {
    KrpcCNanoTransport *storage;
    const KrpcCNanoTransportOps *ops;
    void *context;
    int timeout_ms;
};

/* Production POSIX serial/pseudo-terminal context. */
typedef struct {
    int fd;
    int timeout_ms;
    int baud_rate;
    bool configure_termios;
    char path[KRPC_CNANO_SERIAL_PATH_CAPACITY];
} KrpcCNanoPosixSerial;

#include <krpc_cnano/communication.h>

KrpcCNanoTransportConfig krpc_cnano_transport_config(
    KrpcCNanoTransport *storage,
    const KrpcCNanoTransportOps *ops,
    void *context,
    int timeout_ms);

KrpcCNanoTransportStatus krpc_cnano_transport_last_status(krpc_connection_t connection);
const char *krpc_cnano_transport_status_string(KrpcCNanoTransportStatus status);
bool krpc_cnano_transport_is_open(krpc_connection_t connection);
size_t krpc_cnano_transport_bytes_read(krpc_connection_t connection);
size_t krpc_cnano_transport_bytes_written(krpc_connection_t connection);
void krpc_cnano_transport_reset_counters(krpc_connection_t connection);
int krpc_cnano_transport_timeout_ms(krpc_connection_t connection);
bool krpc_cnano_transport_begin_deadline(krpc_connection_t connection);
void krpc_cnano_transport_end_deadline(krpc_connection_t connection);

void krpc_cnano_posix_serial_init(KrpcCNanoPosixSerial *serial,
                                  const char *path,
                                  int timeout_ms,
                                  int baud_rate,
                                  bool configure_termios);
const KrpcCNanoTransportOps *krpc_cnano_posix_serial_ops(void);

#endif
