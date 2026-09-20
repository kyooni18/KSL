#include "CNanoFakeTransport.h"

#include <krpc_cnano.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifndef _WIN32
#include <signal.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#endif

typedef struct {
    size_t calls;
} ResponderState;

static KrpcCNanoTransportStatus scripted_responder(
    CNanoFakeTransport *fake,
    size_t request_index,
    const uint8_t *payload,
    size_t payload_size,
    void *context) {
    ResponderState *state = (ResponderState *)context;
    static const uint8_t invoke_ok_payload[] = {0x0a, 0x02, 0x12, 0x00};

    assert(state != NULL);
    assert(payload != NULL || payload_size == 0);
    state->calls++;

    if (request_index == 0) {
        /* Empty ConnectionResponse decodes to status=OK in proto3. */
        return cnano_fake_queue_delimited_payload(fake, NULL, 0)
                   ? KRPC_CNANO_TRANSPORT_OK
                   : KRPC_CNANO_TRANSPORT_IO_ERROR;
    }

    /* MultiplexedResponse { response { results { } } } */
    return cnano_fake_queue_delimited_payload(
               fake, invoke_ok_payload, sizeof(invoke_ok_payload))
               ? KRPC_CNANO_TRANSPORT_OK
               : KRPC_CNANO_TRANSPORT_IO_ERROR;
}

static void test_partial_io_and_request_capture(void) {
    CNanoFakeTransport fake;
    KrpcCNanoTransportConfig config;
    krpc_connection_t connection = NULL;
    const uint8_t connection_ok[] = {0x00};
    const uint8_t *payload = NULL;
    size_t payload_size = 0;

    cnano_fake_init(&fake);
    cnano_fake_set_chunk_limits(&fake, 1, 2);
    assert(cnano_fake_queue_read(&fake, connection_ok, sizeof(connection_ok)));
    config = cnano_fake_config(&fake);

    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(krpc_connect(connection, "shuttle-offline") == KRPC_OK);
    assert(cnano_fake_request_count(&fake) == 1);
    assert(cnano_fake_request_payload(&fake, 0, &payload, &payload_size));
    assert(payload_size > 0);
    assert(payload[0] == 0x0a); /* MultiplexedRequest.connection_request */
    assert(krpc_cnano_transport_bytes_read(connection) == 1);
    assert(krpc_cnano_transport_bytes_written(connection) == cnano_fake_written_size(&fake));
    assert(krpc_close(connection) == KRPC_OK);
    assert(fake.open_count == 1);
    assert(fake.close_count == 1);
}

static void test_stateful_responder_and_invoke(void) {
    CNanoFakeTransport fake;
    KrpcCNanoTransportConfig config;
    krpc_connection_t connection = NULL;
    krpc_schema_ProcedureCall call = krpc_schema_ProcedureCall_init_default;
    krpc_schema_ProcedureResult result = krpc_schema_ProcedureResult_init_default;
    ResponderState state = {0};
    const uint8_t *payload = NULL;
    size_t payload_size = 0;

    cnano_fake_init(&fake);
    cnano_fake_set_chunk_limits(&fake, 2, 3);
    cnano_fake_set_responder(&fake, scripted_responder, &state);
    config = cnano_fake_config(&fake);

    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(krpc_connect(connection, "stateful-client") == KRPC_OK);

    call.service_id = 17;
    call.procedure_id = 23;
    assert(krpc_invoke(connection, &result, &call) == KRPC_OK);

    assert(state.calls == 2);
    assert(cnano_fake_request_count(&fake) == 2);
    assert(cnano_fake_request_payload(&fake, 1, &payload, &payload_size));
    assert(payload_size > 0);
    assert(payload[0] == 0x12); /* MultiplexedRequest.request */
    assert(krpc_close(connection) == KRPC_OK);
}

static void test_timeout_propagation_and_reconnect(void) {
    CNanoFakeTransport fake;
    KrpcCNanoTransportConfig config;
    krpc_connection_t connection = NULL;
    const uint8_t connection_ok[] = {0x00};

    cnano_fake_init(&fake);
    config = cnano_fake_config(&fake);
    assert(cnano_fake_script_read(&fake, KRPC_CNANO_TRANSPORT_TIMEOUT, 0));

    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(krpc_connect(connection, "timeout-client") == KRPC_ERROR_DECODING_FAILED);
    assert(!krpc_cnano_transport_is_open(connection));
    assert(krpc_cnano_transport_last_status(connection) == KRPC_CNANO_TRANSPORT_TIMEOUT);
    assert(fake.close_count == 1);

    cnano_fake_clear_buffers(&fake);
    assert(cnano_fake_queue_read(&fake, connection_ok, sizeof(connection_ok)));
    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(krpc_connect(connection, "reconnected-client") == KRPC_OK);
    assert(fake.open_count == 2);
    assert(krpc_close(connection) == KRPC_OK);
    assert(fake.close_count == 2);
}

static void test_truncated_and_malformed_protobuf(void) {
    CNanoFakeTransport fake;
    KrpcCNanoTransportConfig config;
    krpc_connection_t connection = NULL;
    const uint8_t truncated[] = {0x03, 0x08};
    const uint8_t malformed[] = {0x01, 0x0f};

    cnano_fake_init(&fake);
    cnano_fake_set_empty_read_status(&fake, KRPC_CNANO_TRANSPORT_EOF);
    assert(cnano_fake_queue_read(&fake, truncated, sizeof(truncated)));
    config = cnano_fake_config(&fake);
    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(krpc_connect(connection, "truncated-client") == KRPC_ERROR_DECODING_FAILED);
    assert(krpc_cnano_transport_last_status(connection) == KRPC_CNANO_TRANSPORT_EOF);

    cnano_fake_clear_buffers(&fake);
    cnano_fake_set_empty_read_status(&fake, KRPC_CNANO_TRANSPORT_TIMEOUT);
    assert(cnano_fake_queue_read(&fake, malformed, sizeof(malformed)));
    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(krpc_connect(connection, "malformed-client") == KRPC_ERROR_DECODING_FAILED);
    assert(!krpc_cnano_transport_is_open(connection));
}

#ifndef _WIN32
static volatile sig_atomic_t transport_interruptions = 0;

static void transport_signal_handler(int signal_number) {
    (void)signal_number;
    transport_interruptions++;
}

static double transport_monotonic_seconds(void) {
    struct timespec now;
    assert(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    return (double)now.tv_sec + (double)now.tv_nsec / 1e9;
}

static void sleep_milliseconds(long milliseconds) {
    struct timespec delay = {
        .tv_sec = milliseconds / 1000,
        .tv_nsec = (milliseconds % 1000) * 1000000L
    };
    while (nanosleep(&delay, &delay) != 0) {}
}

static void test_config_is_not_silently_rewritten(void) {
    KrpcCNanoPosixSerial serial;
    krpc_cnano_posix_serial_init(&serial, "/dev/null", 0, 0, false);
    assert(serial.timeout_ms == 0);
    assert(serial.baud_rate == 0);
    assert(krpc_cnano_posix_serial_ops()->open(&serial) ==
           KRPC_CNANO_TRANSPORT_INVALID);
}

static void test_timeout_is_absolute_across_eintr(void) {
    int pipes[2];
    assert(pipe(pipes) == 0);

    struct sigaction action = {0}, previous = {0};
    action.sa_handler = transport_signal_handler;
    sigemptyset(&action.sa_mask);
    assert(sigaction(SIGUSR1, &action, &previous) == 0);

    pid_t parent = getpid();
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(pipes[0]);
        for (int i = 0; i < 3; ++i) {
            sleep_milliseconds(75);
            (void)kill(parent, SIGUSR1);
        }
        sleep_milliseconds(1000);
        close(pipes[1]);
        _exit(0);
    }

    close(pipes[1]);
    KrpcCNanoPosixSerial serial = {
        .fd = pipes[0],
        .timeout_ms = 300,
        .baud_rate = 115200,
        .configure_termios = false
    };
    uint8_t byte = 0;
    size_t transferred = 0;
    transport_interruptions = 0;
    double started = transport_monotonic_seconds();
    KrpcCNanoTransportStatus status =
        krpc_cnano_posix_serial_ops()->read(
            &serial, &byte, sizeof(byte), &transferred, serial.timeout_ms);
    double elapsed = transport_monotonic_seconds() - started;

    assert(status == KRPC_CNANO_TRANSPORT_TIMEOUT);
    assert(transferred == 0);
    assert(transport_interruptions >= 2);
    /* The configured 300 ms is the deadline. Three EINTR events must not
       restart it into the ~525 ms behavior of per-poll timeout accounting.
       The 420 ms upper guard is scheduler tolerance, not flight policy. */
    assert(elapsed >= 0.20);
    assert(elapsed < 0.42);

    close(pipes[0]);
    int child_status = 0;
    assert(waitpid(child, &child_status, 0) == child);
    assert(WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0);
    assert(sigaction(SIGUSR1, &previous, NULL) == 0);
}
#endif

static void test_write_error_and_zero_progress(void) {
    CNanoFakeTransport fake;
    KrpcCNanoTransportConfig config;
    krpc_connection_t connection = NULL;
    const uint8_t bytes[] = {1, 2, 3};

    cnano_fake_init(&fake);
    config = cnano_fake_config(&fake);
    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(cnano_fake_script_write(&fake, KRPC_CNANO_TRANSPORT_TIMEOUT, 0));
    assert(krpc_write(connection, bytes, sizeof(bytes)) == KRPC_ERROR_IO);
    assert(krpc_cnano_transport_last_status(connection) == KRPC_CNANO_TRANSPORT_TIMEOUT);
    assert(krpc_close(connection) == KRPC_OK);

    cnano_fake_clear_buffers(&fake);
    assert(krpc_open(&connection, &config) == KRPC_OK);
    assert(cnano_fake_script_write(&fake, KRPC_CNANO_TRANSPORT_OK, 0));
    assert(krpc_write(connection, bytes, sizeof(bytes)) == KRPC_ERROR_IO);
    assert(krpc_cnano_transport_last_status(connection) == KRPC_CNANO_TRANSPORT_ZERO_PROGRESS);
    assert(krpc_close(connection) == KRPC_OK);
}

int main(void) {
    test_partial_io_and_request_capture();
    test_stateful_responder_and_invoke();
    test_timeout_propagation_and_reconnect();
    test_truncated_and_malformed_protobuf();
#ifndef _WIN32
    test_config_is_not_silently_rewritten();
    test_timeout_is_absolute_across_eintr();
#endif
    test_write_error_and_zero_progress();
    puts("C-Nano transport tests passed.");
    return 0;
}
