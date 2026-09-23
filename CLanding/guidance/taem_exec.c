#include "taem_exec.h"

#include <math.h>
#include <string.h>

static bool phase_valid(TaemPhase phase) {
    return phase >= TAEM_PHASE_S_TURN && phase <= TAEM_PHASE_FINAL_INTERCEPT;
}

static bool recovery_reason_valid(TaemRecoveryReason reason) {
    return reason >= TAEM_RECOVERY_ATTITUDE && reason <= TAEM_RECOVERY_ENERGY_INFEASIBLE;
}

static bool observation_valid(const TaemExecObservation *observation) {
    return observation && isfinite(observation->ut) &&
           isfinite(observation->relative_velocity) &&
           observation->relative_velocity >= 0.0;
}

static bool profile_valid(const TaemExecProfile *profile) {
    return profile != NULL;
}

static bool inputs_valid(const TaemExecInputs *inputs) {
    if (!inputs) return false;
    if (inputs->energy_valid && !isfinite(inputs->energy_excess)) return false;
    if (inputs->off_nominal_recovery_active &&
        !recovery_reason_valid(inputs->off_nominal_recovery_reason))
        return false;
    return true;
}

static void terminal_contract_clear(TaemTerminalContract *contract) {
    if (!contract) return;
    memset(contract, 0, sizeof(*contract));
    contract->range_to_go = NAN;
    contract->range_margin = NAN;
    contract->dynamic_pressure = NAN;
    contract->dynamic_pressure_margin = NAN;
    contract->speedbrake_dynamic_pressure_margin = NAN;
    contract->altitude = NAN;
    contract->altitude_margin = NAN;
    contract->flight_path_angle = NAN;
    contract->flight_path_angle_margin = NAN;
    contract->specific_energy = NAN;
    contract->specific_energy_margin = NAN;
    contract->response_time_available = NAN;
    contract->response_time_required = NAN;
}

static bool terminal_contract_numbers_valid(const TaemTerminalContract *contract) {
    if (!contract || !contract->valid) return false;
    if (!isfinite(contract->range_to_go) || contract->range_to_go < 0.0 ||
        !isfinite(contract->range_margin) ||
        !isfinite(contract->dynamic_pressure) || contract->dynamic_pressure < 0.0 ||
        !isfinite(contract->dynamic_pressure_margin) ||
        !isfinite(contract->speedbrake_dynamic_pressure_margin) ||
        !isfinite(contract->altitude) || !isfinite(contract->altitude_margin) ||
        !isfinite(contract->flight_path_angle) ||
        !isfinite(contract->flight_path_angle_margin) ||
        !isfinite(contract->specific_energy) ||
        !isfinite(contract->specific_energy_margin) ||
        !isfinite(contract->response_time_available) ||
        !isfinite(contract->response_time_required) ||
        contract->response_time_available < 0.0 ||
        contract->response_time_required < 0.0)
        return false;
    return true;
}

TaemTerminalEvaluation taem_exec_evaluate_terminal_contract(
    const TaemTerminalContract *contract) {
    TaemTerminalEvaluation evaluation = {
        .valid = false,
        .feasible = false,
        .block_reason = TAEM_TERMINAL_BLOCK_CONTRACT_INVALID,
    };
    if (!terminal_contract_numbers_valid(contract)) return evaluation;

    evaluation.valid = true;
    if (!contract->path_committed) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED;
        return evaluation;
    }
    if (contract->range_margin < 0.0) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_RANGE;
        return evaluation;
    }
    if (contract->dynamic_pressure_margin < 0.0) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE;
        return evaluation;
    }
    if (contract->speedbrake_required &&
        (!contract->speedbrake_available ||
         contract->speedbrake_dynamic_pressure_margin < 0.0)) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_SPEEDBRAKE;
        return evaluation;
    }
    if (contract->altitude_margin < 0.0) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_ALTITUDE;
        return evaluation;
    }
    if (contract->flight_path_angle_margin < 0.0) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE;
        return evaluation;
    }
    if (contract->specific_energy_margin < 0.0) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY;
        return evaluation;
    }
    if (contract->response_time_available < contract->response_time_required) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_FINITE_RESPONSE;
        return evaluation;
    }
    if (!contract->attitude_response_qualified) {
        evaluation.block_reason = TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE;
        return evaluation;
    }

    evaluation.feasible = true;
    evaluation.block_reason = TAEM_TERMINAL_BLOCK_NONE;
    return evaluation;
}

static void update_terminal_contract(TaemExecutive *exec,
                                     const TaemExecInputs *inputs) {
    exec->terminal_contract = inputs->terminal_contract;
    exec->terminal_evaluation =
        taem_exec_evaluate_terminal_contract(&inputs->terminal_contract);
}

static bool nominal_terminal_path_feasible(const TaemExecInputs *inputs) {
    return inputs->terminal_feasibility_valid && inputs->nominal_terminal_path_feasible;
}

static bool high_energy_s_turn_required(const TaemExecInputs *inputs,
                                        const TaemExecProfile *profile) {
    return profile->s_turn_enabled &&
           inputs->energy_valid &&
           inputs->terminal_feasibility_valid &&
           !inputs->nominal_terminal_path_feasible &&
           inputs->energy_excess > 0.0;
}

static void record_transition(TaemExecutive *exec,
                              TaemPhase from,
                              TaemPhase to,
                              TaemTransitionReason reason,
                              const TaemExecObservation *observation,
                              bool completes_taem) {
    exec->phase = to;
    exec->taem_complete = exec->taem_complete || completes_taem;
    exec->last_transition_reason = reason;
    exec->last_transition_from = from;
    exec->last_transition_to = to;
    exec->last_transition_ut = observation->ut;
    exec->last_transition_velocity = observation->relative_velocity;
    exec->last_transition_completed_taem = completes_taem;
    exec->transition_count++;
}

static void update_recovery(TaemExecutive *exec,
                            const TaemExecObservation *observation,
                            const TaemExecInputs *inputs) {
    if (inputs->off_nominal_recovery_active) {
        if (!exec->recovery_active ||
            exec->recovery_reason != inputs->off_nominal_recovery_reason) {
            exec->last_recovery_reason = inputs->off_nominal_recovery_reason;
            exec->last_recovery_ut = observation->ut;
            exec->recovery_count++;
        }
        exec->recovery_active = true;
        exec->recovery_reason = inputs->off_nominal_recovery_reason;
        return;
    }
    exec->recovery_active = false;
    exec->recovery_reason = TAEM_RECOVERY_NONE;
}

static TaemPhase restart_phase(const TaemExecInputs *inputs,
                               const TaemExecProfile *profile,
                               const TaemTerminalEvaluation *terminal) {
    if ((inputs->final_intercept_ready || inputs->final_approach_ready) &&
        terminal && terminal->feasible)
        return TAEM_PHASE_FINAL_INTERCEPT;
    /* A checkpoint that had reached the final-intercept/final-approach gates but no longer has a
       qualified terminal contract resumes in Runway Alignment, never by
       blindly skipping into final approach. This is still forward MM305 ownership, not an
       Entry rollback. */
    if (inputs->terminal_path_captured || inputs->final_intercept_ready || inputs->final_approach_ready)
        return TAEM_PHASE_RUNWAY_ALIGNMENT;
    if (high_energy_s_turn_required(inputs, profile))
        return TAEM_PHASE_S_TURN;
    return TAEM_PHASE_PATH_ACQUISITION;
}

void taem_exec_reset(TaemExecutive *exec) {
    if (!exec) return;
    memset(exec, 0, sizeof(*exec));
    /* Path Acquisition is the nominal MM305 entry state; S-turn is opt-in only when
       excessive energy and terminal infeasibility are both established. */
    exec->phase = TAEM_PHASE_PATH_ACQUISITION;
    exec->last_transition_from = TAEM_PHASE_PATH_ACQUISITION;
    exec->last_transition_to = TAEM_PHASE_PATH_ACQUISITION;
    exec->last_transition_reason = TAEM_TRANSITION_NONE;
    exec->ownership_ut = NAN;
    exec->ownership_velocity = NAN;
    exec->last_transition_ut = NAN;
    exec->last_transition_velocity = NAN;
    exec->recovery_reason = TAEM_RECOVERY_NONE;
    exec->last_recovery_reason = TAEM_RECOVERY_NONE;
    exec->last_recovery_ut = NAN;
    terminal_contract_clear(&exec->terminal_contract);
    exec->terminal_evaluation.valid = false;
    exec->terminal_evaluation.feasible = false;
    exec->terminal_evaluation.block_reason = TAEM_TERMINAL_BLOCK_CONTRACT_INVALID;
}

bool taem_exec_initialize(TaemExecutive *exec,
                          const TaemExecObservation *observation,
                          const TaemExecInputs *inputs,
                          const TaemExecProfile *profile) {
    if (!exec || !observation_valid(observation) ||
        !inputs_valid(inputs) || !profile_valid(profile))
        return false;

    bool normal_handoff = inputs->mm304_complete;
    bool restart_handoff = observation->checkpoint_restart && inputs->resume_mm305;
    if (!normal_handoff && !restart_handoff) return false;

    TaemExecutive next;
    taem_exec_reset(&next);
    next.initialized = true;
    next.ownership_latched = true;
    next.ownership_ut = observation->ut;
    next.ownership_velocity = observation->relative_velocity;
    update_terminal_contract(&next, inputs);

    if (restart_handoff) {
        next.phase = restart_phase(inputs, profile, &next.terminal_evaluation);
        /* A checkpoint must obey the same recovery hold as a normal update.
           Classify the phase, but defer delivery until recovery has cleared
           and the live terminal contract is evaluated again. */
        next.taem_complete = inputs->final_approach_ready &&
                             next.terminal_evaluation.feasible &&
                             !inputs->off_nominal_recovery_active;
        next.last_transition_reason = TAEM_TRANSITION_RESTART_CLASSIFICATION;
    } else if (high_energy_s_turn_required(inputs, profile)) {
        next.phase = TAEM_PHASE_S_TURN;
        next.last_transition_reason = TAEM_TRANSITION_EXCESS_ENERGY_S_TURN;
    } else {
        next.phase = TAEM_PHASE_PATH_ACQUISITION;
        next.last_transition_reason = TAEM_TRANSITION_MM304_HANDOFF;
    }

    next.last_transition_from = next.phase;
    next.last_transition_to = next.phase;
    next.last_transition_ut = observation->ut;
    next.last_transition_velocity = observation->relative_velocity;
    next.last_transition_completed_taem = next.taem_complete;
    update_recovery(&next, observation, inputs);
    *exec = next;
    return true;
}

bool taem_exec_update(TaemExecutive *exec,
                      const TaemExecObservation *observation,
                      const TaemExecInputs *inputs,
                      const TaemExecProfile *profile) {
    if (!exec || !observation_valid(observation) ||
        !inputs_valid(inputs) || !profile_valid(profile))
        return false;
    if (!exec->initialized)
        return taem_exec_initialize(exec, observation, inputs, profile);
    if (!exec->ownership_latched || !phase_valid(exec->phase)) return false;

    /* MM305 ownership is latched. A stale/recomputed Entry-complete flag cannot
       return nominal control to MM304 and therefore cannot create Entry/TAEM
       oscillation across predictor refreshes. */
    update_terminal_contract(exec, inputs);
    update_recovery(exec, observation, inputs);
    if (exec->taem_complete || exec->recovery_active) return true;

    TaemPhase from = exec->phase;
    switch (exec->phase) {
        case TAEM_PHASE_S_TURN:
            if (!inputs->energy_valid) {
                /* Missing/out-of-domain energy cannot authorize continued
                   dissipation. Keep MM305 ownership and acquire a terminal path. */
                record_transition(exec, from, TAEM_PHASE_PATH_ACQUISITION,
                                  TAEM_TRANSITION_S_TURN_ENERGY_INVALID,
                                  observation, false);
            } else if (inputs->energy_excess <= 0.0) {
                record_transition(exec, from, TAEM_PHASE_PATH_ACQUISITION,
                                  TAEM_TRANSITION_S_TURN_SURPLUS_EXHAUSTED,
                                  observation, false);
            } else if (nominal_terminal_path_feasible(inputs)) {
                /* A qualified terminal path can already absorb the remaining
                   energy. Extra S-turn distance now consumes alignment energy
                   rather than buying feasibility, so exit immediately. */
                record_transition(exec, from, TAEM_PHASE_PATH_ACQUISITION,
                                  TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE,
                                  observation, false);
            }
            break;
        case TAEM_PHASE_PATH_ACQUISITION:
            if (high_energy_s_turn_required(inputs, profile)) {
                record_transition(exec, from, TAEM_PHASE_S_TURN,
                                  TAEM_TRANSITION_EXCESS_ENERGY_S_TURN, observation, false);
            } else if (inputs->terminal_path_selected && inputs->terminal_path_captured) {
                record_transition(exec, from, TAEM_PHASE_RUNWAY_ALIGNMENT,
                                  TAEM_TRANSITION_TERMINAL_PATH_CAPTURE, observation, false);
            }
            break;
        case TAEM_PHASE_RUNWAY_ALIGNMENT:
            if (!inputs->terminal_path_selected || !inputs->terminal_path_captured) {
                TaemPhase replanned = high_energy_s_turn_required(inputs, profile)
                    ? TAEM_PHASE_S_TURN : TAEM_PHASE_PATH_ACQUISITION;
                record_transition(exec, from, replanned,
                                  TAEM_TRANSITION_TERMINAL_PATH_REPLAN, observation, false);
            } else if (inputs->final_intercept_ready && exec->terminal_evaluation.feasible) {
                record_transition(exec, from, TAEM_PHASE_FINAL_INTERCEPT,
                                  TAEM_TRANSITION_FINAL_INTERCEPT_GATE, observation, false);
            }
            break;
        case TAEM_PHASE_FINAL_INTERCEPT:
            if (!inputs->final_approach_ready &&
                (!inputs->terminal_path_selected || !inputs->terminal_path_captured)) {
                TaemPhase replanned = high_energy_s_turn_required(inputs, profile)
                    ? TAEM_PHASE_S_TURN : TAEM_PHASE_PATH_ACQUISITION;
                record_transition(exec, from, replanned,
                                  TAEM_TRANSITION_TERMINAL_PATH_REPLAN, observation, false);
            } else if (inputs->final_approach_ready && exec->terminal_evaluation.feasible) {
                record_transition(exec, from, TAEM_PHASE_FINAL_INTERCEPT,
                                  TAEM_TRANSITION_FINAL_APPROACH_DELIVERY,
                                  observation, true);
            }
            break;
        default:
            return false;
    }
    return true;
}

TaemExecTelemetry taem_exec_telemetry(const TaemExecutive *exec) {
    TaemExecTelemetry telemetry;
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.active_phase = TAEM_PHASE_PATH_ACQUISITION;
    telemetry.last_transition_from = TAEM_PHASE_PATH_ACQUISITION;
    telemetry.last_transition_to = TAEM_PHASE_PATH_ACQUISITION;
    telemetry.ownership_ut = NAN;
    telemetry.ownership_velocity = NAN;
    telemetry.last_transition_ut = NAN;
    telemetry.last_transition_velocity = NAN;
    telemetry.last_recovery_ut = NAN;
    terminal_contract_clear(&telemetry.terminal_contract);
    telemetry.terminal_evaluation.block_reason = TAEM_TERMINAL_BLOCK_CONTRACT_INVALID;
    if (!exec) return telemetry;

    telemetry.initialized = exec->initialized;
    telemetry.ownership_latched = exec->ownership_latched;
    telemetry.taem_complete = exec->taem_complete;
    telemetry.active_phase = exec->phase;
    telemetry.ownership_ut = exec->ownership_ut;
    telemetry.ownership_velocity = exec->ownership_velocity;
    telemetry.last_transition_reason = exec->last_transition_reason;
    telemetry.last_transition_from = exec->last_transition_from;
    telemetry.last_transition_to = exec->last_transition_to;
    telemetry.last_transition_ut = exec->last_transition_ut;
    telemetry.last_transition_velocity = exec->last_transition_velocity;
    telemetry.last_transition_completed_taem = exec->last_transition_completed_taem;
    telemetry.transition_count = exec->transition_count;
    telemetry.recovery_active = exec->recovery_active;
    telemetry.recovery_reason = exec->recovery_reason;
    telemetry.last_recovery_reason = exec->last_recovery_reason;
    telemetry.last_recovery_ut = exec->last_recovery_ut;
    telemetry.recovery_count = exec->recovery_count;
    telemetry.terminal_contract = exec->terminal_contract;
    telemetry.terminal_evaluation = exec->terminal_evaluation;
    return telemetry;
}

bool taem_exec_owns_vehicle(const TaemExecutive *exec) {
    return exec && exec->initialized && exec->ownership_latched;
}

const char *taem_phase_string(TaemPhase phase) {
    switch (phase) {
        case TAEM_PHASE_S_TURN: return "S-turn";
        case TAEM_PHASE_PATH_ACQUISITION: return "Path Acquisition";
        case TAEM_PHASE_RUNWAY_ALIGNMENT: return "Runway Alignment";
        case TAEM_PHASE_FINAL_INTERCEPT: return "Final Intercept";
        default: return "Unknown TAEM Phase";
    }
}

const char *taem_transition_reason_string(TaemTransitionReason reason) {
    switch (reason) {
        case TAEM_TRANSITION_NONE: return "None";
        case TAEM_TRANSITION_MM304_HANDOFF: return "MM304 qualified handoff";
        case TAEM_TRANSITION_RESTART_CLASSIFICATION: return "Checkpoint/restart classification";
        case TAEM_TRANSITION_EXCESS_ENERGY_S_TURN: return "Excess energy requires TAEM S-turn";
        case TAEM_TRANSITION_S_TURN_SURPLUS_EXHAUSTED: return "TAEM S-turn surplus exhausted";
        case TAEM_TRANSITION_TERMINAL_PATH_CAPTURE: return "Terminal path captured";
        case TAEM_TRANSITION_FINAL_INTERCEPT_GATE: return "Runway alignment/final intercept gate";
        case TAEM_TRANSITION_FINAL_APPROACH_DELIVERY: return "Final-approach delivery";
        case TAEM_TRANSITION_S_TURN_TERMINAL_FEASIBLE: return "Terminal path feasible; preserve alignment energy";
        case TAEM_TRANSITION_TERMINAL_PATH_REPLAN: return "Terminal path lost/stale; replan inside TAEM";
        case TAEM_TRANSITION_S_TURN_ENERGY_INVALID: return "Entry energy proxy no longer valid; acquire terminal path";
        default: return "Unknown TAEM transition";
    }
}

const char *taem_recovery_reason_string(TaemRecoveryReason reason) {
    switch (reason) {
        case TAEM_RECOVERY_NONE: return "None";
        case TAEM_RECOVERY_ATTITUDE: return "Off-nominal attitude recovery";
        case TAEM_RECOVERY_PATH_REPLAN: return "Off-nominal terminal path replan";
        case TAEM_RECOVERY_CAPTURE_LOSS: return "Off-nominal terminal path capture loss";
        case TAEM_RECOVERY_ENERGY_INFEASIBLE: return "Off-nominal terminal energy infeasible";
        default: return "Unknown TAEM recovery";
    }
}

const char *taem_terminal_block_reason_string(TaemTerminalBlockReason reason) {
    switch (reason) {
        case TAEM_TERMINAL_BLOCK_NONE: return "None";
        case TAEM_TERMINAL_BLOCK_CONTRACT_INVALID: return "Terminal contract invalid";
        case TAEM_TERMINAL_BLOCK_PATH_UNCOMMITTED: return "Terminal path not committed";
        case TAEM_TERMINAL_BLOCK_RANGE: return "Insufficient runway-alignment range reserve";
        case TAEM_TERMINAL_BLOCK_DYNAMIC_PRESSURE: return "Dynamic-pressure envelope exceeded";
        case TAEM_TERMINAL_BLOCK_SPEEDBRAKE: return "Speedbrake authority/envelope unavailable";
        case TAEM_TERMINAL_BLOCK_ALTITUDE: return "Terminal altitude envelope missed";
        case TAEM_TERMINAL_BLOCK_FLIGHT_PATH_ANGLE: return "Terminal flight-path-angle envelope missed";
        case TAEM_TERMINAL_BLOCK_SPECIFIC_ENERGY: return "Terminal specific-energy envelope missed";
        case TAEM_TERMINAL_BLOCK_FINITE_RESPONSE: return "Insufficient finite-response time";
        case TAEM_TERMINAL_BLOCK_ATTITUDE_RESPONSE: return "Attitude response not qualified";
        default: return "Unknown terminal block reason";
    }
}
