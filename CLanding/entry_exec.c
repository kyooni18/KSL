#include "entry_exec.h"

#include <math.h>
#include <string.h>

static bool phase_valid(EntryPhase phase) {
    return phase >= ENTRY_PHASE_PREENTRY && phase <= ENTRY_PHASE_TRANSITION;
}


static bool observation_valid(const EntryExecObservation *observation) {
    return observation && isfinite(observation->ut) &&
           isfinite(observation->relative_velocity) &&
           observation->relative_velocity >= 0.0 &&
           isfinite(observation->altitude) &&
           isfinite(observation->dynamic_pressure) && observation->dynamic_pressure >= 0.0;
}

static bool profile_valid(const EntryExecProfile *profile) {
    if (!profile) return false;
    if (profile->has_temperature_velocity_gate &&
        (!isfinite(profile->temperature_end_velocity) || profile->temperature_end_velocity < 0.0))
        return false;
    if (profile->has_constant_drag_velocity_gate &&
        (!isfinite(profile->constant_drag_end_velocity) || profile->constant_drag_end_velocity < 0.0))
        return false;
    if (profile->has_phase_floor && !phase_valid(profile->phase_floor)) return false;
    return true;
}

static EntryPhase max_phase(EntryPhase a, EntryPhase b) {
    return a > b ? a : b;
}

/* Restart classification is deliberately conservative but cumulative. Later
   valid velocity gates imply that the earlier velocity-ordered phases are no
   longer appropriate. The equilibrium profile intersection is not assumed to
   persist after it has been crossed, so phase_floor exists for checkpoint
   restores whose reference provider can classify the saved state more exactly. */
static EntryPhase restart_phase(const EntryExecObservation *observation,
                                const EntryExecProfile *profile) {
    EntryPhase phase = ENTRY_PHASE_PREENTRY;
    if (profile->has_phase_floor) phase = max_phase(phase, profile->phase_floor);
    if (observation->dynamic_pressure > 0.0)
        phase = max_phase(phase, ENTRY_PHASE_TEMPERATURE_CONTROL);
    if (profile->has_temperature_velocity_gate &&
        observation->relative_velocity <= profile->temperature_end_velocity)
        phase = max_phase(phase, ENTRY_PHASE_EQUILIBRIUM_GLIDE);
    if (profile->equilibrium_intercept_valid && profile->equilibrium_intercept)
        phase = max_phase(phase, ENTRY_PHASE_CONSTANT_DRAG);
    if (profile->has_constant_drag_velocity_gate &&
        observation->relative_velocity <= profile->constant_drag_end_velocity)
        phase = max_phase(phase, ENTRY_PHASE_TRANSITION);
    return phase;
}

static void record_transition(EntryExecutive *exec,
                              EntryPhase from,
                              EntryPhase to,
                              EntryTransitionReason reason,
                              const EntryExecObservation *observation,
                              bool completes_entry) {
    exec->phase = to;
    exec->entry_complete = exec->entry_complete || completes_entry;
    exec->last_transition_reason = reason;
    exec->last_transition_from = from;
    exec->last_transition_to = to;
    exec->last_transition_ut = observation->ut;
    exec->last_transition_velocity = observation->relative_velocity;
    exec->last_transition_completed_entry = completes_entry;
    exec->transition_count++;
}

void entry_exec_reset(EntryExecutive *exec) {
    if (!exec) return;
    memset(exec, 0, sizeof(*exec));
    exec->phase = ENTRY_PHASE_PREENTRY;
    exec->last_transition_from = ENTRY_PHASE_PREENTRY;
    exec->last_transition_to = ENTRY_PHASE_PREENTRY;
    exec->last_transition_reason = ENTRY_TRANSITION_NONE;
    exec->last_transition_ut = NAN;
    exec->last_transition_velocity = NAN;
}

bool entry_exec_initialize(EntryExecutive *exec,
                           const EntryExecObservation *observation,
                           const EntryExecProfile *profile) {
    if (!exec || !observation_valid(observation) || !profile_valid(profile)) return false;

    EntryExecutive next;
    entry_exec_reset(&next);
    next.initialized = true;
    if (observation->checkpoint_restart) {
        next.phase = restart_phase(observation, profile);
        next.entry_complete = false;
        next.last_transition_reason = ENTRY_TRANSITION_RESTART_CLASSIFICATION;
    } else {
        /* A normal MM304 start begins in Pre-entry. Atmospheric contact is
           observed directly from positive dynamic pressure on subsequent updates;
           checkpoint phase classification is used only for explicit restores. */
        next.phase = ENTRY_PHASE_PREENTRY;
        next.entry_complete = false;
        next.last_transition_reason = ENTRY_TRANSITION_ENTRY_START;
    }
    next.last_transition_from = next.phase;
    next.last_transition_to = next.phase;
    next.last_transition_ut = observation->ut;
    next.last_transition_velocity = observation->relative_velocity;
    next.last_transition_completed_entry = next.entry_complete;
    *exec = next;
    return true;
}

bool entry_exec_update(EntryExecutive *exec,
                       const EntryExecObservation *observation,
                       const EntryExecProfile *profile) {
    if (!exec || !observation_valid(observation) || !profile_valid(profile)) return false;
    if (!exec->initialized) return entry_exec_initialize(exec, observation, profile);
    if (!phase_valid(exec->phase)) return false;
    if (exec->entry_complete) return true;

    EntryPhase from = exec->phase;


    switch (exec->phase) {
        case ENTRY_PHASE_PREENTRY:
            if (observation->dynamic_pressure > 0.0)
                record_transition(exec, from, ENTRY_PHASE_TEMPERATURE_CONTROL,
                                  ENTRY_TRANSITION_ATMOSPHERIC_CONTACT, observation, false);
            break;
        case ENTRY_PHASE_TEMPERATURE_CONTROL:
            if (profile->has_temperature_velocity_gate &&
                observation->relative_velocity <= profile->temperature_end_velocity)
                record_transition(exec, from, ENTRY_PHASE_EQUILIBRIUM_GLIDE,
                                  ENTRY_TRANSITION_TEMPERATURE_VELOCITY, observation, false);
            break;
        case ENTRY_PHASE_EQUILIBRIUM_GLIDE:
            if (profile->equilibrium_intercept_valid && profile->equilibrium_intercept)
                record_transition(exec, from, ENTRY_PHASE_CONSTANT_DRAG,
                                  ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT, observation, false);
            break;
        case ENTRY_PHASE_CONSTANT_DRAG:
            if (profile->has_constant_drag_velocity_gate &&
                observation->relative_velocity <= profile->constant_drag_end_velocity)
                record_transition(exec, from, ENTRY_PHASE_TRANSITION,
                                  ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY, observation, false);
            break;
        case ENTRY_PHASE_TRANSITION:
            /* Entry completion is a qualified contract event, never a velocity gate.
               entry_exec_accept_taem_handoff() is the sole completion API. */
            break;
        default:
            return false;
    }
    return true;
}

bool entry_exec_accept_taem_handoff(EntryExecutive *exec,
                                    const EntryExecObservation *observation) {
    if (!exec || !observation_valid(observation) || !exec->initialized ||
        exec->phase != ENTRY_PHASE_TRANSITION)
        return false;
    if (exec->entry_complete) return true;
    record_transition(exec, exec->phase, exec->phase,
                      ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF,
                      observation, true);
    return true;
}

EntryExecTelemetry entry_exec_telemetry(const EntryExecutive *exec) {
    EntryExecTelemetry telemetry;
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.active_phase = ENTRY_PHASE_PREENTRY;
    telemetry.last_transition_from = ENTRY_PHASE_PREENTRY;
    telemetry.last_transition_to = ENTRY_PHASE_PREENTRY;
    telemetry.last_transition_ut = NAN;
    telemetry.last_transition_velocity = NAN;
    if (!exec) return telemetry;

    telemetry.initialized = exec->initialized;
    telemetry.entry_complete = exec->entry_complete;
    telemetry.active_phase = exec->phase;
    telemetry.last_transition_reason = exec->last_transition_reason;
    telemetry.last_transition_from = exec->last_transition_from;
    telemetry.last_transition_to = exec->last_transition_to;
    telemetry.last_transition_ut = exec->last_transition_ut;
    telemetry.last_transition_velocity = exec->last_transition_velocity;
    telemetry.last_transition_completed_entry = exec->last_transition_completed_entry;
    telemetry.transition_count = exec->transition_count;
    return telemetry;
}

const char *entry_phase_string(EntryPhase phase) {
    switch (phase) {
        case ENTRY_PHASE_PREENTRY: return "Pre-entry";
        case ENTRY_PHASE_TEMPERATURE_CONTROL: return "Temperature Control";
        case ENTRY_PHASE_EQUILIBRIUM_GLIDE: return "Equilibrium Glide";
        case ENTRY_PHASE_CONSTANT_DRAG: return "Constant Drag";
        case ENTRY_PHASE_TRANSITION: return "Transition";
        default: return "Unknown Entry Phase";
    }
}

const char *entry_transition_reason_string(EntryTransitionReason reason) {
    switch (reason) {
        case ENTRY_TRANSITION_NONE: return "None";
        case ENTRY_TRANSITION_ENTRY_START: return "Entry start";
        case ENTRY_TRANSITION_RESTART_CLASSIFICATION: return "Checkpoint/restart classification";
        case ENTRY_TRANSITION_ATMOSPHERIC_CONTACT: return "Atmospheric contact";
        case ENTRY_TRANSITION_TEMPERATURE_VELOCITY: return "Temperature-control velocity gate";
        case ENTRY_TRANSITION_EQUILIBRIUM_INTERCEPT: return "Equilibrium/constant-drag profile intersection";
        case ENTRY_TRANSITION_CONSTANT_DRAG_VELOCITY: return "Constant-drag transition velocity gate";
        case ENTRY_TRANSITION_TAEM_QUALIFIED_HANDOFF: return "Qualified TAEM handoff";
        default: return "Unknown Entry transition";
    }
}
