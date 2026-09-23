#include "krpc_cnano_transport.h"

#include <limits.h>
#include <string.h>
#include <time.h>

static bool transport_now_ns(int64_t *now_ns) {
    if (!now_ns) return false;
    struct timespec now;
#ifdef _WIN32
    if (timespec_get(&now, TIME_UTC) != TIME_UTC) return false;
#else
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return false;
#endif
    *now_ns = (int64_t)now.tv_sec * 1000000000LL + (int64_t)now.tv_nsec;
    return true;
}

static bool transport_deadline_ns(int timeout_ms, int64_t *deadline_ns) {
    int64_t now_ns = 0;
    if (timeout_ms <= 0 || !deadline_ns || !transport_now_ns(&now_ns))
        return false;
    *deadline_ns = now_ns + (int64_t)timeout_ms * 1000000LL;
    return true;
}

static int transport_deadline_remaining_ms(int64_t deadline_ns) {
    int64_t now_ns = 0;
    if (!transport_now_ns(&now_ns)) return -1;
    int64_t remaining_ns = deadline_ns - now_ns;
    if (remaining_ns <= 0) return 0;
    int64_t remaining_ms = (remaining_ns + 999999LL) / 1000000LL;
    return remaining_ms > INT_MAX ? INT_MAX : (int)remaining_ms;
}

static krpc_error_t transport_error(KrpcCNanoTransportStatus status) {
    return status == KRPC_CNANO_TRANSPORT_EOF ? KRPC_ERROR_EOF : KRPC_ERROR_IO;
}

static bool transport_config_valid(const krpc_connection_config_t *config) {
    return config != NULL &&
           config->storage != NULL &&
           config->ops != NULL &&
           config->ops->read != NULL &&
           config->ops->write != NULL &&
           config->timeout_ms >= 0;
}

KrpcCNanoTransportConfig krpc_cnano_transport_config(
    KrpcCNanoTransport *storage,
    const KrpcCNanoTransportOps *ops,
    void *context,
    int timeout_ms) {
    KrpcCNanoTransportConfig config;
    config.storage = storage;
    config.ops = ops;
    config.context = context;
    config.timeout_ms = timeout_ms;
    return config;
}

krpc_error_t krpc_open(krpc_connection_t *connection, const krpc_connection_config_t *config) {
    KrpcCNanoTransportStatus status = KRPC_CNANO_TRANSPORT_OK;
    KrpcCNanoTransport *storage;

    if (connection == NULL || !transport_config_valid(config)) {
        return KRPC_ERROR_IO;
    }

    storage = config->storage;
    memset(storage, 0, sizeof(*storage));
    storage->ops = config->ops;
    storage->context = config->context;
    storage->timeout_ms = config->timeout_ms;
    storage->deadline_ns = 0;
    storage->deadline_active = false;
    storage->last_status = KRPC_CNANO_TRANSPORT_OK;

    if (storage->ops->open != NULL) {
        status = storage->ops->open(storage->context);
    }
    storage->last_status = status;
    if (status != KRPC_CNANO_TRANSPORT_OK) {
        *connection = NULL;
        return transport_error(status);
    }

    storage->is_open = true;
    *connection = storage;
    return KRPC_OK;
}

krpc_error_t krpc_close(krpc_connection_t connection) {
    KrpcCNanoTransportStatus status;

    if (connection == NULL) {
        return KRPC_ERROR_IO;
    }
    if (!connection->is_open) {
        return KRPC_OK;
    }

    status = KRPC_CNANO_TRANSPORT_OK;
    if (connection->ops != NULL && connection->ops->close != NULL) {
        status = connection->ops->close(connection->context);
    }
    connection->is_open = false;

    /* Preserve the transport cause that made C-Nano close the connection. */
    if (status != KRPC_CNANO_TRANSPORT_OK) {
        connection->last_status = status;
        return transport_error(status);
    }
    return KRPC_OK;
}

krpc_error_t krpc_read(krpc_connection_t connection, uint8_t *buffer, size_t count) {
    size_t total = 0;

    if (connection == NULL || !connection->is_open || connection->ops == NULL ||
        connection->ops->read == NULL || (buffer == NULL && count != 0)) {
        if (connection != NULL) {
            connection->last_status = KRPC_CNANO_TRANSPORT_CLOSED;
        }
        return KRPC_ERROR_IO;
    }

    while (total < count) {
        size_t transferred = 0;
        int timeout_ms = connection->timeout_ms;
        if (connection->deadline_active) {
            timeout_ms = transport_deadline_remaining_ms(connection->deadline_ns);
            if (timeout_ms < 0) {
                connection->last_status = KRPC_CNANO_TRANSPORT_INVALID;
                return KRPC_ERROR_IO;
            }
            if (timeout_ms == 0) {
                connection->last_status = KRPC_CNANO_TRANSPORT_TIMEOUT;
                return KRPC_ERROR_IO;
            }
        }
        KrpcCNanoTransportStatus status = connection->ops->read(
            connection->context, buffer + total, count - total, &transferred,
            timeout_ms);

        if (transferred > count - total ||
            (status != KRPC_CNANO_TRANSPORT_OK && transferred != 0)) {
            connection->last_status = KRPC_CNANO_TRANSPORT_INVALID;
            return KRPC_ERROR_IO;
        }
        if (status != KRPC_CNANO_TRANSPORT_OK) {
            connection->last_status = status;
            return transport_error(status);
        }
        if (transferred == 0) {
            connection->last_status = KRPC_CNANO_TRANSPORT_ZERO_PROGRESS;
            return KRPC_ERROR_IO;
        }

        total += transferred;
        connection->bytes_read += transferred;
    }

    connection->last_status = KRPC_CNANO_TRANSPORT_OK;
    return KRPC_OK;
}

krpc_error_t krpc_write(krpc_connection_t connection, const uint8_t *buffer, size_t count) {
    size_t total = 0;

    if (connection == NULL || !connection->is_open || connection->ops == NULL ||
        connection->ops->write == NULL || (buffer == NULL && count != 0)) {
        if (connection != NULL) {
            connection->last_status = KRPC_CNANO_TRANSPORT_CLOSED;
        }
        return KRPC_ERROR_IO;
    }

    while (total < count) {
        size_t transferred = 0;
        int timeout_ms = connection->timeout_ms;
        if (connection->deadline_active) {
            timeout_ms = transport_deadline_remaining_ms(connection->deadline_ns);
            if (timeout_ms < 0) {
                connection->last_status = KRPC_CNANO_TRANSPORT_INVALID;
                return KRPC_ERROR_IO;
            }
            if (timeout_ms == 0) {
                connection->last_status = KRPC_CNANO_TRANSPORT_TIMEOUT;
                return KRPC_ERROR_IO;
            }
        }
        KrpcCNanoTransportStatus status = connection->ops->write(
            connection->context, buffer + total, count - total, &transferred,
            timeout_ms);

        if (transferred > count - total ||
            (status != KRPC_CNANO_TRANSPORT_OK && transferred != 0)) {
            connection->last_status = KRPC_CNANO_TRANSPORT_INVALID;
            return KRPC_ERROR_IO;
        }
        if (status != KRPC_CNANO_TRANSPORT_OK) {
            connection->last_status = status;
            return transport_error(status);
        }
        if (transferred == 0) {
            connection->last_status = KRPC_CNANO_TRANSPORT_ZERO_PROGRESS;
            return KRPC_ERROR_IO;
        }

        total += transferred;
        connection->bytes_written += transferred;
    }

    connection->last_status = KRPC_CNANO_TRANSPORT_OK;
    return KRPC_OK;
}

KrpcCNanoTransportStatus krpc_cnano_transport_last_status(krpc_connection_t connection) {
    return connection != NULL ? connection->last_status : KRPC_CNANO_TRANSPORT_INVALID;
}

const char *krpc_cnano_transport_status_string(KrpcCNanoTransportStatus status) {
    switch (status) {
        case KRPC_CNANO_TRANSPORT_OK: return "ok";
        case KRPC_CNANO_TRANSPORT_EOF: return "eof";
        case KRPC_CNANO_TRANSPORT_TIMEOUT: return "timeout";
        case KRPC_CNANO_TRANSPORT_IO_ERROR: return "io-error";
        case KRPC_CNANO_TRANSPORT_CLOSED: return "closed";
        case KRPC_CNANO_TRANSPORT_INVALID: return "invalid";
        case KRPC_CNANO_TRANSPORT_ZERO_PROGRESS: return "zero-progress";
        default: return "unknown";
    }
}

bool krpc_cnano_transport_is_open(krpc_connection_t connection) {
    return connection != NULL && connection->is_open;
}

size_t krpc_cnano_transport_bytes_read(krpc_connection_t connection) {
    return connection != NULL ? connection->bytes_read : 0;
}

size_t krpc_cnano_transport_bytes_written(krpc_connection_t connection) {
    return connection != NULL ? connection->bytes_written : 0;
}

void krpc_cnano_transport_reset_counters(krpc_connection_t connection) {
    if (connection != NULL) {
        connection->bytes_read = 0;
        connection->bytes_written = 0;
    }
}

int krpc_cnano_transport_timeout_ms(krpc_connection_t connection) {
    return connection != NULL ? connection->timeout_ms : 0;
}

bool krpc_cnano_transport_begin_deadline(krpc_connection_t connection) {
    if (!connection) return false;
    connection->deadline_active = false;
    connection->deadline_ns = 0;
    if (connection->timeout_ms == 0) return true;
    if (!transport_deadline_ns(connection->timeout_ms, &connection->deadline_ns))
        return false;
    connection->deadline_active = true;
    return true;
}

void krpc_cnano_transport_end_deadline(krpc_connection_t connection) {
    if (!connection) return;
    connection->deadline_active = false;
    connection->deadline_ns = 0;
}

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

static speed_t serial_speed(int baud_rate) {
    switch (baud_rate) {
#ifdef B9600
        case 9600: return B9600;
#endif
#ifdef B19200
        case 19200: return B19200;
#endif
#ifdef B38400
        case 38400: return B38400;
#endif
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:
            return (speed_t)0;
    }
}

static KrpcCNanoTransportStatus serial_open(void *context) {
    KrpcCNanoPosixSerial *serial = context;
    if (!serial || !serial->path[0] ||
        serial->timeout_ms <= 0 || serial->baud_rate <= 0)
        return KRPC_CNANO_TRANSPORT_INVALID;
    if (serial->fd >= 0) close(serial->fd);
    serial->fd = open(serial->path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial->fd < 0) return KRPC_CNANO_TRANSPORT_IO_ERROR;

    if (serial->configure_termios && isatty(serial->fd)) {
        struct termios tty;
        if (tcgetattr(serial->fd, &tty) != 0) {
            close(serial->fd); serial->fd = -1;
            return KRPC_CNANO_TRANSPORT_IO_ERROR;
        }
        cfmakeraw(&tty);
        speed_t speed=serial_speed(serial->baud_rate);
#ifdef __APPLE__
        /* macOS termios only exposes standard constants through 230400 on
           current SDKs. IOSSIOSPEED is the supported path for the 921600 baud
           default and other arbitrary high-rate USB serial links. Set a valid
           base speed first, then apply the exact requested rate. */
        speed_t base=speed;
        if(base==0){
#ifdef B230400
            base=B230400;
#else
            base=B115200;
#endif
        }
        if(cfsetispeed(&tty,base)!=0||cfsetospeed(&tty,base)!=0||
           tcsetattr(serial->fd,TCSANOW,&tty)!=0){
            close(serial->fd);serial->fd=-1;return KRPC_CNANO_TRANSPORT_IO_ERROR;
        }
        if(speed==0){
            speed_t exact=(speed_t)serial->baud_rate;
            if(ioctl(serial->fd,IOSSIOSPEED,&exact)<0){
                close(serial->fd);serial->fd=-1;return KRPC_CNANO_TRANSPORT_IO_ERROR;
            }
        }
#else
        if(speed==0||cfsetispeed(&tty,speed)!=0||cfsetospeed(&tty,speed)!=0||
           tcsetattr(serial->fd,TCSANOW,&tty)!=0){
            close(serial->fd);serial->fd=-1;return KRPC_CNANO_TRANSPORT_INVALID;
        }
#endif
        (void)tcflush(serial->fd,TCIOFLUSH);
    }
    return KRPC_CNANO_TRANSPORT_OK;
}

static KrpcCNanoTransportStatus serial_close(void *context) {
    KrpcCNanoPosixSerial *serial = context;
    if (!serial) return KRPC_CNANO_TRANSPORT_INVALID;
    if (serial->fd >= 0) {
        int fd = serial->fd;
        serial->fd = -1;
        if (close(fd) != 0) return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
    return KRPC_CNANO_TRANSPORT_OK;
}

static KrpcCNanoTransportStatus serial_wait(int fd, short events, int timeout_ms) {
    struct pollfd pfd = {.fd = fd, .events = events, .revents = 0};
    int64_t deadline_ns = 0;
    if (!transport_deadline_ns(timeout_ms, &deadline_ns))
        return KRPC_CNANO_TRANSPORT_INVALID;
    for (;;) {
        int remaining_ms = transport_deadline_remaining_ms(deadline_ns);
        if (remaining_ms < 0) return KRPC_CNANO_TRANSPORT_IO_ERROR;
        if (remaining_ms == 0) return KRPC_CNANO_TRANSPORT_TIMEOUT;
        pfd.revents = 0;
        int result = poll(&pfd, 1, remaining_ms);
        if (result > 0) {
            if(pfd.revents&(POLLERR|POLLNVAL))return KRPC_CNANO_TRANSPORT_IO_ERROR;
            /* A pty may report POLLIN|POLLHUP together for the peer's final
               bytes. Consume readable data before treating HUP as EOF. */
            if(pfd.revents&events)return KRPC_CNANO_TRANSPORT_OK;
            if(pfd.revents&POLLHUP)return KRPC_CNANO_TRANSPORT_EOF;
            continue;
        }
        if (result == 0) return KRPC_CNANO_TRANSPORT_TIMEOUT;
        if (errno == EINTR) continue;
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
}

static KrpcCNanoTransportStatus serial_read(void *context, uint8_t *buffer,
                                             size_t requested, size_t *transferred,
                                             int timeout_ms) {
    KrpcCNanoPosixSerial *serial = context;
    if (transferred) *transferred = 0;
    if (!serial || serial->fd < 0 || (!buffer && requested) || !transferred)
        return KRPC_CNANO_TRANSPORT_CLOSED;
    if (requested == 0) return KRPC_CNANO_TRANSPORT_OK;
    KrpcCNanoTransportStatus wait = serial_wait(serial->fd, POLLIN, timeout_ms);
    if (wait != KRPC_CNANO_TRANSPORT_OK) return wait;
    for (;;) {
        ssize_t n = read(serial->fd, buffer, requested);
        if (n > 0) { *transferred = (size_t)n; return KRPC_CNANO_TRANSPORT_OK; }
        if (n == 0) return KRPC_CNANO_TRANSPORT_EOF;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return KRPC_CNANO_TRANSPORT_TIMEOUT;
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
}

static KrpcCNanoTransportStatus serial_write(void *context, const uint8_t *buffer,
                                              size_t requested, size_t *transferred,
                                              int timeout_ms) {
    KrpcCNanoPosixSerial *serial = context;
    if (transferred) *transferred = 0;
    if (!serial || serial->fd < 0 || (!buffer && requested) || !transferred)
        return KRPC_CNANO_TRANSPORT_CLOSED;
    if (requested == 0) return KRPC_CNANO_TRANSPORT_OK;
    KrpcCNanoTransportStatus wait = serial_wait(serial->fd, POLLOUT, timeout_ms);
    if (wait != KRPC_CNANO_TRANSPORT_OK) return wait;
    for (;;) {
        ssize_t n = write(serial->fd, buffer, requested);
        if (n > 0) { *transferred = (size_t)n; return KRPC_CNANO_TRANSPORT_OK; }
        if (n == 0) return KRPC_CNANO_TRANSPORT_ZERO_PROGRESS;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return KRPC_CNANO_TRANSPORT_TIMEOUT;
        return KRPC_CNANO_TRANSPORT_IO_ERROR;
    }
}

static const KrpcCNanoTransportOps k_posix_serial_ops = {
    .open = serial_open,
    .close = serial_close,
    .read = serial_read,
    .write = serial_write,
};

void krpc_cnano_posix_serial_init(KrpcCNanoPosixSerial *serial, const char *path,
                                  int timeout_ms, int baud_rate, bool configure_termios) {
    if (!serial) return;
    memset(serial, 0, sizeof(*serial));
    serial->fd = -1;
    /* Configuration normalization owns defaults. Preserve the supplied
       transport contract here and fail closed in serial_open if it is invalid. */
    serial->timeout_ms = timeout_ms;
    serial->baud_rate = baud_rate;
    serial->configure_termios = configure_termios;
    if (path) snprintf(serial->path, sizeof(serial->path), "%s", path);
}

const KrpcCNanoTransportOps *krpc_cnano_posix_serial_ops(void) {
    return &k_posix_serial_ops;
}
#else
void krpc_cnano_posix_serial_init(KrpcCNanoPosixSerial *serial, const char *path,
                                  int timeout_ms, int baud_rate, bool configure_termios) {
    (void)serial; (void)path; (void)timeout_ms; (void)baud_rate; (void)configure_termios;
}
const KrpcCNanoTransportOps *krpc_cnano_posix_serial_ops(void) { return NULL; }
#endif
