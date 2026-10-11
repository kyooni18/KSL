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

/* Holds one command for no more than the 0.5 s solver tick, advancing the
 * native 50 Hz airborne kernel in substeps no larger than 0.02 s. */
TerminalStepStatus terminal_propagator_step(const TerminalModel *model,
        TerminalDynamicState *state, const TerminalControl *control);

#endif
