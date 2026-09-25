#include "landing.h"
#include "decision_envelope.h"
#include "async_prediction_policy.h"
#include "taem_candidate_search.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    double density; Vector3 air_velocity;
    double speed,speed_of_sound,mach,q,drag_accel,lift_accel,non_gravity;
    Vector3 specific_force;double physics_confidence,physics_uncertainty;bool direct_aero;
} AtmosState;
typedef struct {
    double sign,leg_elapsed; bool established,entry_loaded,configured_leg_dwell_satisfied;
    unsigned reversals; JerkLimiter limiter;
    bool has_first_reversal, first_reversal_final, final_heading_lock;
    double first_reversal_ut, first_reversal_range, first_reversal_sign;
    bool replay_event, replay_event_final; double replay_event_ut, replay_event_sign;
    double entry_score; bool has_entry_score;
    double actual_bank,actual_bank_rate,actual_aoa,actual_aoa_rate,entry_speed;
    bool has_interface_target,enforce_terminal_delivery_budget; TaemInterfaceTarget interface_target;
    bool force_control; double forced_until_ut,control_horizon_ut,forced_bank,forced_aoa;
    SpeedbrakeController speedbrake;
} PredictorGuidance;
typedef struct {
    const PlanetModel*p; const AerodynamicEnvelope*e; const TrajectoryCalibrationModel*c; const VehicleProfile*v;
    double bank,aoa,ut,mass;
    bool gear,brakes; int airbrakes;
} AeroContext;


/* Predictor algorithms remain one translation unit for exact numerical parity,
 * but source ownership is separated by responsibility. */
#include "entry_simulation.inc"
#include "entry_planning.inc"
#include "deorbit_planning.inc"
