#ifndef CLANDING_TERMINAL_PROPAGATOR_H
#define CLANDING_TERMINAL_PROPAGATOR_H

#include <stdbool.h>

#include "terminal_model.h"

typedef struct {
    double angle_of_attack_rad;
    double bank_rad;
    double dt_s;
} TerminalControl;

typedef enum {
    TERMINAL_STEP_OK = 0,
    TERMINAL_STEP_INVALID_INPUT,
    TERMINAL_STEP_INVALID_RESULT
} TerminalStepStatus;

/* Advances exactly one ShuttleSim native airborne tick. Forces are evaluated
 * from the pre-step state, then velocity/position and attitude advance in the
 * same order as ShuttleSim. */
TerminalStepStatus terminal_propagator_step(const TerminalModel *model,
        TerminalDynamicState *state, const TerminalControl *control);

#endif
