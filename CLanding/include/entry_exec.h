#ifndef KSP_ENTRY_EXEC_H
#define KSP_ENTRY_EXEC_H

#include <stdbool.h>

/* MM304 Entry subphases. These are intentionally separate from the project's
   coarse GuidancePhase enum so the Entry executive can be integrated without
   changing the stable external phase schema in the specialist worker. */
typedef enum {
    ENTRY_PHASE_PREENTRY = 1,
    ENTRY_PHASE_TEMPERATURE_CONTROL = 2,
    ENTRY_PHASE_EQUILIBRIUM_GLIDE = 3,
    ENTRY_PHASE_CONSTANT_DRAG = 4,
    ENTRY_PHASE_TRANSITION = 5
} EntryPhase;

typedef enum {
    ENTRY_TRANSITION_NONE = 0,
    ENTRY_TRANSITION_ENTRY_START,
    ENTRY_TRANSITION_RESTART_CLASSIFICATION,
    ENTRY_TRANSITION_ATMOSPHERIC_CONTACT,
    ENTRY_TRANSITION_TEMPERATURE_VELOCITY,
    ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT,
    ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY,
    ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF
} EntryTransitionReason;

/* Current measured state needed by the executive. The executive deliberately
   does not derive guidance references or commands from these values. */
typedef struct {
    double ut;
    double relative_velocity;
    double altitude;
    double dynamic_pressure; /* Pa; positive means aerodynamic contact. */
    bool checkpoint_restart;
} EntryExecObservation;

/* Phase-specific transition inputs supplied by configuration and the future
   longitudinal drag/reference module. Numeric gates are optional so a provider
   can withhold a transition until its reference solution is valid.

   Velocity gates are used only for the decelerating longitudinal Entry phases:
   transition when relative_velocity <= supplied gate. The final MM304 -> MM305
   transfer is deliberately not represented by a profile threshold; a caller must
   explicitly accept the qualified TAEM handoff once the live contract is satisfied.

   phase_floor is only used during initialization/restart classification. It is
   never allowed to move an already-running executive backward or silently skip
   it forward. Running Entry progresses only through adjacent phase contracts. */
typedef struct {
    bool has_temperature_velocity_gate;
    double temperature_end_velocity;

    bool equilibrium_intercept_valid;
    bool equilibrium_intercept;

    bool has_constant_drag_velocity_gate;
    double constant_drag_end_velocity;


    bool has_phase_floor;
    EntryPhase phase_floor;
} EntryExecProfile;

typedef struct {
    bool initialized;
    bool entry_complete;
    EntryPhase phase;

    EntryTransitionReason last_transition_reason;
    EntryPhase last_transition_from;
    EntryPhase last_transition_to;
    double last_transition_ut;
    double last_transition_velocity;
    bool last_transition_completed_entry;
    unsigned transition_count;
} EntryExecutive;

typedef struct {
    bool initialized;
    bool entry_complete;
    EntryPhase active_phase;

    EntryTransitionReason last_transition_reason;
    EntryPhase last_transition_from;
    EntryPhase last_transition_to;
    double last_transition_ut;
    double last_transition_velocity;
    bool last_transition_completed_entry;
    unsigned transition_count;
} EntryExecTelemetry;

void entry_exec_reset(EntryExecutive *exec);
bool entry_exec_initialize(EntryExecutive *exec,
                           const EntryExecObservation *observation,
                           const EntryExecProfile *profile);
bool entry_exec_update(EntryExecutive *exec,
                       const EntryExecObservation *observation,
                       const EntryExecProfile *profile);
bool entry_exec_accept_taem_handoff(EntryExecutive *exec,
                                      const EntryExecObservation *observation);
EntryExecTelemetry entry_exec_telemetry(const EntryExecutive *exec);

const char *entry_phase_string(EntryPhase phase);
const char *entry_transition_reason_string(EntryTransitionReason reason);

#endif
