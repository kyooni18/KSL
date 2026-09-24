#ifndef CLANDING_TERMINAL_MODEL_H
#define CLANDING_TERMINAL_MODEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "landing_types.h"
#include "shuttlesim/aero.h"
#include "shuttlesim/attitude.h"
#include "shuttlesim/world.h"

/* Immutable input bundle for offline MM305 trajectory work. The model owns
 * copies of the exact native plant tables and the policy/vehicle snapshot used
 * to price and execute a candidate. Callers pass it as const after creation. */
typedef struct TerminalModel {
    uint32_t schema_version;
    uint64_t snapshot_id;
    uint64_t atmosphere_source_digest;
    uint64_t aero_source_digest;
    uint64_t aero_book_source_digest;
    uint64_t attitude_source_digest;
    bool replay_validated;
    KerbinWorld world;
    AeroTable aero;
    AttitudeModel attitude;
    VehicleProfile vehicle;
    LandingSite site;
    GuidanceSettings guidance;
} TerminalModel;

typedef struct {
    Vec3 position_i_m;
    Vec3 velocity_i_mps;
    AttitudeModel attitude;
    double mass_kg;
    double ut_s;
} TerminalDynamicState;

typedef struct {
    const char *atmosphere_csv;
    const char *aero_csv;
    const char *aero_book_csv; /* NULL or "none" selects coefficient tables only. */
    const char *attitude_ini;
} TerminalModelSourceFiles;

/* Loads and freezes the same native plant tables used by ShuttleSim. Policy
 * inputs are copied from the active application configuration. snapshot_id
 * identifies that combined plant/policy capture in diagnostics and replay. */
bool terminal_model_capture(TerminalModel *model,
        const TerminalModelSourceFiles *files,
        const LandingConfiguration *configuration, uint64_t snapshot_id,
        char *reason, size_t reason_size);

bool terminal_model_validate(const TerminalModel *model, char *reason,
                             size_t reason_size);
bool terminal_state_validate(const TerminalDynamicState *state,
                             char *reason, size_t reason_size);

#endif
