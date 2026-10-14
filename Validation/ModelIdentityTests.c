#include "landing_api.h"
#include "sim_telemetry.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    unsigned snapshots;
    bool engaged;
    char warning[384];
} Observation;

static void observe(const LandingSnapshot *snapshot, void *context) {
    Observation *out = context;
    out->snapshots++;
    out->engaged = snapshot->automation_engaged;
    snprintf(out->warning, sizeof(out->warning), "%s", snapshot->warning_message);
}

int main(void) {
    char reason[384];
    const char *profiles[] = {"identified", "fitted-legacy", "reference-b", "sts-provisional"};
    for (unsigned n = 0; n < sizeof(profiles)/sizeof(profiles[0]); ++n) {
        assert(setenv("KSP_LANDER_MODEL_PROFILE", profiles[n], 1) == 0);
        const char *matching = n == 3 ? "STS" : "STS-N";
        const char *other = n == 3 ? "STS-N" : "STS";
        assert(strcmp(shuttle_sim_model_vehicle_id(), matching) == 0);
        assert(shuttle_sim_model_matches_vehicle(matching, reason, sizeof(reason)));
        assert(!*reason);
        assert(!shuttle_sim_model_matches_vehicle(other, reason, sizeof(reason)));
        assert(strstr(reason, "cross-vehicle model reuse is blocked"));
        assert(strstr(reason, matching) && strstr(reason, other));
        assert(!shuttle_sim_model_matches_vehicle(NULL, NULL, 0));
    }
    assert(setenv("KSP_LANDER_MODEL_PROFILE", "unknown-model", 1) == 0);
    assert(!shuttle_sim_model_matches_vehicle("STS", reason, sizeof(reason)));
    assert(strstr(reason, "unknown"));
    assert(unsetenv("KSP_LANDER_MODEL_PROFILE") == 0);
    assert(shuttle_sim_model_matches_vehicle("STS-N", reason, sizeof(reason)));

    /* Runtime identity: the live vessel name wins; the configured modelId is only
       the fallback when KSP reports no usable name. */
    assert(strcmp(shuttle_sim_resolve_vehicle_identity("Crater", "STS-N"), "Crater") == 0);
    assert(strcmp(shuttle_sim_resolve_vehicle_identity("STS-N", "STS"), "STS-N") == 0);
    assert(strcmp(shuttle_sim_resolve_vehicle_identity("", "STS"), "STS") == 0);
    assert(strcmp(shuttle_sim_resolve_vehicle_identity("Active Vessel", "STS"), "STS") == 0);
    assert(strcmp(shuttle_sim_resolve_vehicle_identity(NULL, NULL), "") == 0);

    /* An uncertified craft is admitted, not refused: commands proceed to the
       normal plan check.  No simulator, KSP process, or transport is opened. */
    LandingConfiguration cfg = landing_configuration_default();
    snprintf(cfg.vehicle.model_id, sizeof(cfg.vehicle.model_id), "STS");
    Observation seen = {0};
    LandingController *controller = landing_controller_create(&cfg, observe, &seen);
    assert(controller);
    void (*commands[])(LandingController *) = {
        landing_controller_engage, landing_controller_engage_reentry,
        landing_controller_engage_hac_test, landing_controller_engage_final_test,
    };
    for (unsigned n = 0; n < sizeof(commands)/sizeof(commands[0]); ++n) {
        unsigned previous = seen.snapshots;
        commands[n](controller);
        assert(seen.snapshots > previous);
        assert(!seen.engaged);
        assert(!strstr(seen.warning, "cross-vehicle model reuse is blocked"));
    }
    landing_controller_destroy(controller);
    puts("Native model identity and engagement regressions passed.");
    return 0;
}
