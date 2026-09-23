#ifndef KSP_TAEM_EXEC_H
#define KSP_TAEM_EXEC_H

#include <stdbool.h>

/* MM305 is the single terminal-area owner after the one-way MM304 handoff.
   These are internal TAEM planning substates, not peer guidance phases: optional
   energy-management S-turn, terminal-path acquisition, runway alignment, and
   final-intercept delivery.  HAC/circle/spline geometry remains a private path
   primitive selected by the terminal path provider. */
typedef enum {
    TAEM_PHASE_S_TURN = 0,
    TAEM_PHASE_PATH_ACQUISITION = 1,
    TAEM_PHASE_RUNWAY_ALIGNMENT = 2,
    TAEM_PHASE_FINAL_INTERCEPT = 3
} TaemPhase;

typedef enum {
    TAEM_TRANSITION_NONE = 0,
    TAEM_TRANSITION_MM304_HANDOFF,
    TAEM_TRANSITION_RESTART_CLASSIFICATION,
    TAEM_TRANSITION_EXCESS_ENERGY_S_TURN,
    TAEM_TRANSITION_S_TURN_SURPLUS_EXHAUSTED,
    TAEM_TRANSITION_TERMINAL_PATH_CAPTURE,
    TAEM_TRANSITION_FINAL_INTERCEPT_GATE,
    TAEM_TRANSITION_FINAL_APPROACH_DELIVERY,
    TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE,
    TAEM_TRANSITION_TERMINAL_PATH_REPLAN,
    TAEM_TRANSITION_S_TURN_ENERGY_INVALID
} TaemTransitionReason;

typedef enum {
    TAEM_RECOVERY_NONE = 0,
    TAEM_RECOVERY_ATTITUDE,
    TAEM_RECOVERY_PATH_REPLAN,
    TAEM_RECOVERY_CAPTURE_LOSS,
    TAEM_RECOVERY_ENERGY_INFEASIBLE
} TaemRecoveryReason;

/* Why the current terminal path may not be delivered from TAEM into final approach.
   These are qualification results, not recovery commands. The path provider
   and native flight-control layer remain responsible for deciding how to fix a
   blocked contract. */
typedef enum {
    TAEM_TERMINAL_BLOCK_NONE = 0,
    TAEM_TERMINAL_BLOCK_CONTRACT_INVALID,
    TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED,
    TAEM_TERMINAL_BLOCK_RANGE,
    TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE,
    TAEM_TERMINAL_BLOCK_SPEEDBRAKE,
    TAEM_TERMINAL_BLOCK_ALTITUDE,
    TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE,
    TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY,
    TAEM_TERMINAL_BLOCK_FINITE_RESPONSE,
    TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE
} TaemTerminalBlockReason;

/* Stable unified-TAEM terminal-path contract.

   Detailed circle/spline/HAC-like geometry deliberately stays outside taem_exec. The path
   provider reduces its current, response-propagated candidate to signed
   feasibility margins while the native FCS supplies actuator availability.
   This keeps TAEM independent of kRPC transport details and of any particular
   terminal-path shape.

   All *_margin fields are feasible at >= 0. The provider must compute them
   against the same active candidate and current sample:

   - range_margin: remaining range-to-go after the candidate path plus runway-
     alignment reserve (m).
   - dynamic_pressure_margin: active vehicle/control q-bar limit minus current
     dynamic pressure (Pa).
   - speedbrake_dynamic_pressure_margin: speedbrake deployment q-bar limit minus
     current q-bar (Pa); used only when speedbrake_required is true.
   - altitude_margin / flight_path_angle_margin: distance inside the candidate's
     admissible vertical state envelope (m / deg).
   - specific_energy_margin: distance inside the candidate's usable specific-
     energy envelope after expected losses (m^2/s^2 == J/kg). It therefore
     protects both insufficient and excessive terminal energy.
   - response_time_available / response_time_required: time to the next frozen
     geometry gate versus the complete guidance + attitude + actuator response
     budget (s).

   Raw range/q-bar/altitude/FPA/specific-energy values are carried alongside the
   margins for deterministic replay and telemetry qualification. */
typedef struct {
    bool valid;
    bool path_committed;

    double range_to_go;
    double range_margin;

    double dynamic_pressure;
    double dynamic_pressure_margin;
    bool speedbrake_required;
    bool speedbrake_available;
    double speedbrake_dynamic_pressure_margin;

    double altitude;
    double altitude_margin;
    double flight_path_angle;
    double flight_path_angle_margin;

    double specific_energy;
    double specific_energy_margin;

    double response_time_available;
    double response_time_required;
    bool attitude_response_qualified;
} TaemTerminalContract;

typedef struct {
    bool valid;
    bool feasible;
    TaemTerminalBlockReason block_reason;
} TaemTerminalEvaluation;

typedef struct {
    double ut;
    double relative_velocity;
    bool checkpoint_restart;
} TaemExecObservation;

/* Inputs are deliberately narrow and owned by other guidance modules.

   mm304_complete is the normal one-way ownership handoff from the Entry
   executive. resume_mm305 is used only while reconstructing a checkpoint that
   was already inside TAEM.

   energy_excess is a signed, provider-defined surplus metric. Positive values
   mean the current terminal solution cannot absorb the remaining energy/range
   without adding path length; zero is the physical balance point and negative
   values mean continued dissipation would overspend terminal energy. TAEM S-turn
   entry therefore requires BOTH positive surplus and a valid indication that the
   current terminal-path envelope is not feasible. A nominal feasible candidate
   ends the S-turn immediately regardless of the remaining coarse surplus.

   Geometry remains outside this executive. The unified TAEM path provider
   supplies generic path-selected/path-captured/final-intercept readiness while
   circle, spline, and HAC-like details stay private.  Path loss after ownership
   is an internal TAEM replan, never permission to reacquire MM304. Final-approach
   delivery is additionally fail-closed by terminal_contract. */
typedef struct {
    bool mm304_complete;
    bool resume_mm305;

    bool energy_valid;
    double energy_excess;
    bool terminal_feasibility_valid;
    bool nominal_terminal_path_feasible;

    bool terminal_path_selected;
    bool terminal_path_captured;
    bool final_intercept_ready;
    bool final_approach_ready;

    TaemTerminalContract terminal_contract;

    bool off_nominal_recovery_active;
    TaemRecoveryReason off_nominal_recovery_reason;
} TaemExecInputs;

typedef struct {
    bool s_turn_enabled;
} TaemExecProfile;

typedef struct {
    bool initialized;
    bool ownership_latched;
    bool taem_complete;
    TaemPhase phase;

    double ownership_ut;
    double ownership_velocity;

    TaemTransitionReason last_transition_reason;
    TaemPhase last_transition_from;
    TaemPhase last_transition_to;
    double last_transition_ut;
    double last_transition_velocity;
    bool last_transition_completed_taem;
    unsigned transition_count;

    bool recovery_active;
    TaemRecoveryReason recovery_reason;
    TaemRecoveryReason last_recovery_reason;
    double last_recovery_ut;
    unsigned recovery_count;

    TaemTerminalContract terminal_contract;
    TaemTerminalEvaluation terminal_evaluation;
} TaemExecutive;

typedef struct {
    bool initialized;
    bool ownership_latched;
    bool taem_complete;
    TaemPhase active_phase;

    double ownership_ut;
    double ownership_velocity;

    TaemTransitionReason last_transition_reason;
    TaemPhase last_transition_from;
    TaemPhase last_transition_to;
    double last_transition_ut;
    double last_transition_velocity;
    bool last_transition_completed_taem;
    unsigned transition_count;

    bool recovery_active;
    TaemRecoveryReason recovery_reason;
    TaemRecoveryReason last_recovery_reason;
    double last_recovery_ut;
    unsigned recovery_count;

    TaemTerminalContract terminal_contract;
    TaemTerminalEvaluation terminal_evaluation;
} TaemExecTelemetry;

void taem_exec_reset(TaemExecutive *exec);
bool taem_exec_initialize(TaemExecutive *exec,
                          const TaemExecObservation *observation,
                          const TaemExecInputs *inputs,
                          const TaemExecProfile *profile);
bool taem_exec_update(TaemExecutive *exec,
                      const TaemExecObservation *observation,
                      const TaemExecInputs *inputs,
                      const TaemExecProfile *profile);
TaemExecTelemetry taem_exec_telemetry(const TaemExecutive *exec);
bool taem_exec_owns_vehicle(const TaemExecutive *exec);
TaemTerminalEvaluation taem_exec_evaluate_terminal_contract(
    const TaemTerminalContract *contract);

const char *taem_phase_string(TaemPhase phase);
const char *taem_transition_reason_string(TaemTransitionReason reason);
const char *taem_recovery_reason_string(TaemRecoveryReason reason);
const char *taem_terminal_block_reason_string(TaemTerminalBlockReason reason);

#endif
