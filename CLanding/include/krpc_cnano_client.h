#ifndef KSP_LANDER_KRPC_CNANO_CLIENT_H
#define KSP_LANDER_KRPC_CNANO_CLIENT_H

#include "landing.h"
#include "krpc_cnano_transport.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct KrpcCNanoClient KrpcCNanoClient;

typedef struct {
    unsigned read_calls;
    unsigned write_calls;
    unsigned total_calls;
    unsigned read_wire_requests;
    unsigned write_wire_requests;
    unsigned total_wire_requests;
    double last_read_seconds;
    double last_apply_seconds;
    double read_calls_per_second;
    double apply_calls_per_second;
    double read_wire_requests_per_second;
    double apply_wire_requests_per_second;
    size_t bytes_read;
    size_t bytes_written;
} KrpcCNanoBudget;

KrpcCNanoClient *krpc_cnano_client_open(const LandingConfiguration *configuration,
                                        char *error, size_t error_size);
/* Deterministic custom-transport entry point used by offline integration tests. */
KrpcCNanoClient *krpc_cnano_client_open_transport(const LandingConfiguration *configuration,
                                                  const KrpcCNanoTransportConfig *transport_config,
                                                  const char *transport_name,
                                                  char *error,size_t error_size);
void krpc_cnano_client_close(KrpcCNanoClient *client);

const PlanetModel *krpc_cnano_client_planet(const KrpcCNanoClient *client);
const char *krpc_cnano_client_vessel(const KrpcCNanoClient *client);
const char *krpc_cnano_client_transport_name(const KrpcCNanoClient *client);
const char *krpc_cnano_client_library_version(const KrpcCNanoClient *client);
const char *krpc_cnano_client_flight_key(const KrpcCNanoClient *client);

bool krpc_cnano_client_read(KrpcCNanoClient *client,
                            const LandingConfiguration *configuration,
                            Telemetry *telemetry,
                            VehicleState *state,
                            char *error, size_t error_size);

bool krpc_cnano_client_set_direct_controls(KrpcCNanoClient *client,
                                           double pitch, double roll, double yaw,
                                           double throttle, double wheel_steering,
                                           bool rcs,
                                           char *error, size_t error_size);
bool krpc_cnano_client_set_autopilot(KrpcCNanoClient *client,
                                     const GuidanceCommand *command,
                                     const Vector3 *inertial_up_reference,
                                     char *error, size_t error_size);
bool krpc_cnano_client_set_gear(KrpcCNanoClient *client, bool value,
                                char *error, size_t error_size);
bool krpc_cnano_client_set_brakes(KrpcCNanoClient *client, bool value,
                                  char *error, size_t error_size);
bool krpc_cnano_client_set_airbrakes(KrpcCNanoClient *client, unsigned group, bool value,
                                     char *error, size_t error_size);
bool krpc_cnano_client_set_speed_mode(KrpcCNanoClient *client, NavballSpeedMode mode,
                                      char *error, size_t error_size);
typedef struct {
    int32_t factor;
    double predicted_rate;
    double response_horizon_seconds;
    double stopping_margin_ut;
} KrpcCNanoWarpDecision;

bool krpc_cnano_client_warp_decision(double remaining_ut,
                                     double response_horizon_seconds,
                                     int maximum_factor,
                                     KrpcCNanoWarpDecision *decision);
bool krpc_cnano_client_warp(KrpcCNanoClient *client, double ut,
                            char *error, size_t error_size);
bool krpc_cnano_client_save(KrpcCNanoClient *client, const char *name,
                            char *error, size_t error_size);
void krpc_cnano_client_safe(KrpcCNanoClient *client);

KrpcCNanoBudget krpc_cnano_client_budget(const KrpcCNanoClient *client);

#endif
