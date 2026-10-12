/* Pure feedback/cache and real log-codec regression. No RPC connection. */
#include "throttle_feedback.h"
#include "flight_log_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    assert(!throttle_feedback_requires_reassertion(false, 0, 1, 59000));
    assert(!throttle_feedback_requires_reassertion(true, 0, 0, 0));
    assert(!throttle_feedback_requires_reassertion(true, .18, 0, 10600));
    assert(throttle_feedback_requires_reassertion(true, .18, 1, 59000));
    assert(throttle_feedback_requires_reassertion(true, 0, 1, 0));
    assert(throttle_feedback_requires_reassertion(true, 0, 0, 59165.16));
    assert(!throttle_feedback_requires_reassertion(true, 0, NAN, NAN));
    assert(throttle_feedback_requires_reassertion(true, 0, NAN, 59165.16));
    assert(!throttle_feedback_requires_reassertion(true, 0, 0, .1));
    /* Reproduce a zero-command cache hit while the actuator was overridden. */
    bool cached = true;
    const double command = 0.0;
    assert(cached && fabs(command - 0.0) <= 1e-4); /* Old path skipped writing. */
    if (throttle_feedback_requires_reassertion(cached, command, 1., 59165.16))
        cached = false;
    assert(!cached); /* Both direct and AutoPilot apply paths now issue zero. */
    cached = true; /* Successful reassertion followed by settled telemetry. */
    assert(!throttle_feedback_requires_reassertion(cached, command, 0., 0.));

    VehicleLogEncoder encoder;
    vehicle_log_encoder_init(&encoder);
    VehicleLogSample sample = {0};
    sample.has_control_state = true;
    sample.wall_monotonic_seconds = 10.;
    VehicleLogDecision first = vehicle_log_decide(&encoder, &sample, false, true);
    vehicle_log_commit(&encoder, &sample, &first);
    sample.wall_monotonic_seconds += .2;
    sample.control_state_throttle = 1.;
    VehicleLogDecision delta = vehicle_log_decide(&encoder, &sample, false, false);
    assert(delta.emit && (delta.groups & VEHICLE_LOG_GROUP_CONTROL));
    JsonWriter w;
    jw_init(&w);
    vehicle_log_record_json(&w, "offline-feedback-test", 2, &sample, &delta);
    assert(!w.failed && strstr(w.data, "\"reportedThrottle\":1"));
    jw_free(&w);
    puts("Throttle feedback reassertion and measured-throttle delta tests passed.");
    return 0;
}
