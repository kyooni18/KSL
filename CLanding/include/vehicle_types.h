#ifndef KSP_LANDER_VEHICLE_TYPES_H
#define KSP_LANDER_VEHICLE_TYPES_H

#include "guidance_types.h"

typedef struct {
    bool active; CalibrationRunState state; double start_ut, state_start_ut; double initial_heading;
    double angles[16]; int angle_count, angle_index;
    char status[256];
} GlideCalibrationMachine;

typedef struct {
    double ld[4], beta[4], confidence[4]; int accepted, rejected; double best_glide_aoa, stall_speed;
    VehicleProfile recommended;
} AdaptiveFlightCalibrator;

typedef struct {
    Trajectory forecast; TrajectoryCalibrationModel model; double previous_ut; bool has_previous;
} InFlightTrajectoryCalibrator;

/* Unified vessel physics: wind axes are drag-opposed, lift-up at zero bank,
   and cross(air direction, lift-up). Angular arrays: pitch, roll, yaw; deg/s². */
#define VESSEL_AERO_SAMPLES 192
typedef struct {
    double q, mach, aoa, beta, mass;
    Vector3 force_per_q; /* signed wind-axis force / Pa, preserves side force */
    bool gear, brakes;
    int airbrakes; /* -1 unknown/legacy, 0 retracted, 1 deployed */
    /* Number of independent prior flight/timeline contributors represented by
       this certified cell. It is statistical support, not frame count. */
    unsigned observations;
    double trust; /* provenance/reliability weight in the normalized [0,1] domain */
    double last_observation_ut; /* current-flight observation timestamp */
} VesselAeroSample;
struct VesselPhysicsModel {
    VehicleState state;
    Vector3 center_of_mass, lift_vector, drag_vector, center_of_mass_root;
    bool has_center_of_mass_root;
    bool has_state, has_center_of_mass, gear, brakes;
    int airbrakes;
    double thrust, available_thrust, throttle, mass_flow, propellant_per_impulse;
    double authority[3], authority_confidence[3], previous_rate[3], previous_control[3];
    double torque[3], inertia[3];
    bool previous_rate_valid[3];
    double authority_q, sideslip, confidence;
    double model_residual, model_residual_confidence, certified_uncertainty, certified_confidence;
    double last_residual_ut; bool has_last_residual_ut;
    Vector3 force_residual_per_q, force_residual_variance_per_q;
    double force_residual_confidence;
    double dry_mass;
    unsigned count, next, accepted, rejected, live_observations;
    VesselAeroSample samples[VESSEL_AERO_SAMPLES];
};

#endif
