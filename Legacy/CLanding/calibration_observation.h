#ifndef KSP_LANDER_CALIBRATION_OBSERVATION_H
#define KSP_LANDER_CALIBRATION_OBSERVATION_H

#include "decision_envelope.h"

typedef struct {
    bool valid;
    bool airborne;
    bool clean_configuration;
    bool actuator_domain_valid;
    bool unpowered;
    bool force_observable;
    bool passive_aero_usable;
    bool trajectory_usable;

    DecisionMargin dynamic_pressure_minimum;
    DecisionMargin dynamic_pressure_maximum;
    DecisionMargin incidence;
    DecisionMargin sideslip;
    DecisionMargin load;
    DecisionMargin stall;
    DecisionMargin thrust;

    double incidence_deg;
    double aerodynamic_accel_mps2;
    double aerodynamic_response_time_s;
    double information_increment;
    double information_blend;
    double worst_normalized_margin;
} CalibrationObservationEnvelope;

typedef struct {
    bool valid;
    bool safe;

    DecisionMargin altitude;
    DecisionMargin dynamic_pressure;
    DecisionMargin load;
    DecisionMargin sink_rate;
    DecisionMargin stall;
    DecisionMargin speed;

    double worst_normalized_margin;
} CalibrationSafetyEnvelope;

/*
 * Passive model identification evaluates whether the actual measured state is
 * inside the model's domain. Direct force vectors are already observations at
 * the current Mach/incidence; body/coordinate rate thresholds are therefore
 * not sample-quality criteria.
 */
CalibrationObservationEnvelope calibration_observation_envelope(
    const Telemetry *telemetry,
    const VehicleProfile *vehicle,
    const CalibrationSettings *settings,
    double dt);

/*
 * The deliberate low-speed glide sweep has its own configured safety contract.
 * Do not reuse these low-speed flight-test bounds as passive entry-data gates.
 */
CalibrationSafetyEnvelope calibration_safety_envelope(
    const Telemetry *telemetry,
    const VehicleProfile *vehicle,
    const CalibrationSettings *settings);

#endif
