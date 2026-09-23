#include "krpc_cnano_client.h"
#include "krpc_cnano_transport.h"
#include "krpc_cnano_batch.h"

#include <krpc_cnano.h>
#include <krpc_cnano/memory.h>
#include <krpc_cnano/encoder.h>
#include <krpc_cnano/decoder.h>
#include <krpc_cnano/services/krpc.h>
#include <krpc_cnano/services/space_center.h>

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define CNANO_ENGINE_LIMIT 32
#define CNANO_ATMOSPHERE_SAMPLES 71
#define CNANO_MEDIUM_PERIOD 0.25
#define CNANO_SLOW_PERIOD 1.0

typedef struct {
    krpc_SpaceCenter_Engine_t object;
    bool initially_active;
    bool activated_by_client;
    bool independent_owned;
    float available_thrust;
} CommandableEngine;

struct KrpcCNanoClient {
    KrpcCNanoTransport transport;
    KrpcCNanoPosixSerial serial;
    krpc_connection_t connection;
    bool connected;

    krpc_SpaceCenter_Vessel_t vessel;
    krpc_SpaceCenter_Control_t control;
    krpc_SpaceCenter_AutoPilot_t autopilot;
    krpc_SpaceCenter_Orbit_t orbit;
    krpc_SpaceCenter_CelestialBody_t body;
    krpc_SpaceCenter_ReferenceFrame_t body_nonrotating;
    krpc_SpaceCenter_ReferenceFrame_t vessel_frame;
    krpc_SpaceCenter_ReferenceFrame_t surface_frame;
    krpc_SpaceCenter_Flight_t flight_surface;
    krpc_SpaceCenter_Flight_t flight_body;
    krpc_SpaceCenter_Flight_t flight_vessel;

    PlanetModel planet;
    char vessel_name[128];
    char transport_name[192];
    char flight_key[128];
    char cached_situation[64];

    CommandableEngine engines[CNANO_ENGINE_LIMIT];
    size_t engine_count;
    double commandable_thrust;

    bool cached_gear;
    bool cached_brakes;
    bool cached_airbrakes;
    bool cached_rcs;
    bool cached_sas;
    bool cached_autopilot_engaged;
    bool has_autopilot_engaged;
    bool orbital_autotune_profile_active;
    double cached_command_throttle;
    double cached_command_wheel;
    bool has_command_throttle;
    bool has_command_wheel;
    bool has_gear;
    bool has_brakes;
    bool has_airbrakes;
    bool has_rcs;
    bool has_sas;
    bool pause_triplet_sent;
    NavballSpeedMode cached_speed_mode;
    bool has_speed_mode;

    double cached_speed_of_sound;
    double cached_static_pressure;
    double cached_g_force;
    double cached_available_thrust;
    double cached_current_thrust;
    double cached_dry_mass;
    double cached_torque[3];
    double cached_inertia[3];
    Vector3 cached_lift_vector, cached_drag_vector, cached_center_of_mass, cached_center_of_mass_root;
    bool cached_force_vectors_valid, cached_center_of_mass_valid, cached_center_of_mass_root_valid;
    double cached_apoapsis;
    double cached_periapsis;
    double cached_period;
    double last_medium_ut;
    double last_slow_ut;
    bool medium_valid;
    bool slow_valid;

    bool has_previous_attitude;
    double previous_ut;
    double previous_pitch;
    double previous_roll;
    double previous_heading;
    double previous_angle_of_attack;
    bool has_previous_course;
    double previous_course_ut;
    double previous_course;
    LowPass pitch_rate_filter;
    LowPass roll_rate_filter;
    LowPass heading_rate_filter;
    LowPass angle_of_attack_rate_filter;
    LowPass course_rate_filter;

    unsigned total_rpc_calls;
    unsigned total_wire_requests;
    unsigned last_read_wire_requests;
    unsigned last_apply_wire_requests;
    unsigned last_read_calls;
    unsigned last_apply_calls;
    double last_read_seconds;
    double last_apply_seconds;
};


/* C-Nano client state and ABI stay in this translation unit. Session setup,
 * telemetry, controls, warp, and lifecycle code are separated by ownership. */
#include "client/session.inc"
#include "client/telemetry.inc"
#include "client/controls.inc"
#include "client/warp.inc"
#include "client/lifecycle.inc"
