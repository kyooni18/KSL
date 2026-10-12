#ifndef KSL_THROTTLE_FEEDBACK_H
#define KSL_THROTTLE_FEEDBACK_H

#include <math.h>
#include <stdbool.h>

/* KSL drives selected engines with independent throttle. Global throttle must
 * therefore be zero even during a commanded OMS burn. A successful past write
 * is not evidence that an external controller has left either actuator alone.
 * Reassert ownership on contradictory feedback; do not abort the mission. */
static inline bool throttle_feedback_requires_reassertion(
        bool has_command, double commanded_throttle,
        double measured_global_throttle, double measured_thrust_n) {
    if (!has_command) return false; /* The first command is already uncached. */
    if (isfinite(measured_global_throttle) &&
        fabs(measured_global_throttle) > 1e-4) return true;
    return isfinite(commanded_throttle) && commanded_throttle <= 1e-4 &&
        isfinite(measured_thrust_n) && measured_thrust_n > 1.0;
}

#endif
