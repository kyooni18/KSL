#ifndef KSP_LANDER_FLIGHT_LOG_CODEC_H
#define KSP_LANDER_FLIGHT_LOG_CODEC_H

#include "json.h"

#include <stdbool.h>
#include <stdint.h>

enum {
    VEHICLE_LOG_GROUP_STATE = 1u << 0,
    VEHICLE_LOG_GROUP_POSITION = 1u << 1,
    VEHICLE_LOG_GROUP_MOTION = 1u << 2,
    VEHICLE_LOG_GROUP_ATTITUDE = 1u << 3,
    VEHICLE_LOG_GROUP_AERO = 1u << 4,
    VEHICLE_LOG_GROUP_VEHICLE = 1u << 5,
    VEHICLE_LOG_GROUP_COMMAND = 1u << 6,
    VEHICLE_LOG_GROUP_CONTROL = 1u << 7,
    VEHICLE_LOG_GROUP_GUIDANCE = 1u << 8,
    VEHICLE_LOG_GROUP_RUNTIME = 1u << 9,
    VEHICLE_LOG_GROUP_PHYSICS = 1u << 10,
    VEHICLE_LOG_GROUP_ALL = (1u << 11) - 1u
};

typedef struct {
    uint64_t tick_sequence;
    double wall_monotonic_seconds;
    double ut;

    double latitude, longitude, altitude, radar_altitude;
    double position[3], velocity[3];
    double true_air_speed, horizontal_speed, vertical_speed, flight_path_angle;

    double pitch, roll, heading, angle_of_attack, sideslip;
    /* Legacy pitchRate/rollRate/yawRate retain body-first schema4 semantics.
       The explicit fields below remove frame ambiguity for new analysis. */
    double pitch_rate, roll_rate, yaw_rate, course_rate;
    double coordinate_pitch_rate, coordinate_roll_rate, heading_rate, angle_of_attack_rate;
    double body_pitch_rate, body_roll_rate, body_yaw_rate;

    double dynamic_pressure, static_pressure, atmospheric_density, mach, stall_fraction;
    bool stall_fraction_is_measured;
    double lift_force, drag_force, g_force, energy_excess_range;
    bool has_force_vectors;
    double lift_vector[3], drag_vector[3], aero_acceleration[3];

    double physics_confidence, physics_model_residual, physics_model_residual_confidence, physics_certified_uncertainty;
    double physics_force_residual_per_q[3], physics_force_residual_sigma_per_q[3];
    double physics_force_residual_confidence;
    bool physics_airbrake_model_available;
    double physics_airbrake_model_confidence, physics_airbrake_drag_accel;
    unsigned physics_samples, physics_live_samples;

    double mass, available_thrust, current_thrust;
    bool has_center_of_mass;
    double center_of_mass[3];

    double target_pitch, target_heading, target_roll, target_throttle;
    bool has_target_aoa;
    double target_aoa;
    bool target_gear, target_brakes, target_airbrakes;

    bool gear, brakes, has_airbrakes, airbrakes;
    bool has_control_state;
    double control_state_pitch, control_state_roll, control_state_yaw;
    bool command_applied, has_actuator_feedback;
    double control_pitch, control_roll, control_yaw;

    bool has_loop_wall_delta, has_telemetry_latency, has_guidance_compute, has_apply_latency, has_control_loop;
    double loop_wall_delta_ms, telemetry_latency_ms, guidance_compute_ms, apply_latency_ms, control_loop_ms;

    bool automation_engaged, paused, constraint_active;
    char phase[40];
    char control_profile[32];
    char vessel_situation[64];
    bool has_plan_identity;
    uint64_t plan_id, plan_version, parent_plan_id, parent_plan_version;

    double range_to_site, bearing_to_site, runway_along_track, runway_cross_track;
    double specific_mechanical_energy;
    bool entry_plan_valid, entry_reversal_scheduled;
    double entry_reversal_time_remaining, entry_reversal_ut, entry_reversal_range, entry_reversal_sign;

    char event[512];
} VehicleLogSample;

typedef struct {
    bool emit;
    bool keyframe;
    bool durable;
    uint32_t groups;
} VehicleLogDecision;

typedef struct {
    bool has_last;
    VehicleLogSample last;
    double last_emit_wall;
    double last_keyframe_wall;
} VehicleLogEncoder;

void vehicle_log_encoder_init(VehicleLogEncoder *encoder);
VehicleLogDecision vehicle_log_decide(
    const VehicleLogEncoder *encoder,
    const VehicleLogSample *sample,
    bool force_event,
    bool force_keyframe);
void vehicle_log_commit(
    VehicleLogEncoder *encoder,
    const VehicleLogSample *sample,
    const VehicleLogDecision *decision);
void vehicle_log_record_json(
    JsonWriter *writer,
    const char *session_id,
    uint64_t record_sequence,
    const VehicleLogSample *sample,
    const VehicleLogDecision *decision);

#endif
