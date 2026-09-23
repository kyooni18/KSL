#ifndef SHUTTLESIM_PROTOCOL_H
#define SHUTTLESIM_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include "types.h"

typedef struct {
    int command_fd;
    int telemetry_fd;
    bool udp_enabled;
    char telemetry_host[64];
    int telemetry_port;
    int web_telemetry_port;
} Protocol;

typedef struct {
    bool has_attitude;
    double aoa_deg;
    double bank_deg;
    bool has_gear;
    bool gear_down;
    bool has_brakes;
    bool brakes;
    bool has_airbrakes;
    bool airbrakes;
    bool has_wheel_steering;
    double wheel_steering;
    bool has_throttle;
    double throttle;
    bool step;
    bool pause;
    bool resume;
} SimCommand;

bool protocol_open(Protocol *p, int command_port, const char *telemetry_host, int telemetry_port, int web_telemetry_port);
void protocol_close(Protocol *p);
bool protocol_poll_command(Protocol *p, SimCommand *cmd);
bool protocol_parse_command(const char *json, SimCommand *cmd);
void protocol_send_telemetry(Protocol *p, const char *json, size_t len);
#endif
