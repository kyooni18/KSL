#ifndef KSP_SHUTTLE_SIM_TELEMETRY_H
#define KSP_SHUTTLE_SIM_TELEMETRY_H

#include "landing.h"

#include <stdbool.h>
#include <stddef.h>

/* Simulator world constants and the complete fitted atmosphere used by every
   production-guidance simulator client. Failure leaves the destination intact. */
bool shuttle_sim_load_planet(const char *atmosphere_path, PlanetModel *planet,
                             char *error, size_t error_size);

/* Decode one ShuttleSim telemetry packet into the production telemetry ABI.
   previous may be NULL for the first packet.  Controlled-coordinate rates are
   derived only from a finite positive time interval; rigid-body rates remain
   unavailable because ShuttleSim does not publish body angular velocity. */
bool shuttle_sim_decode_telemetry(const char *packet,
                                  const PlanetModel *planet,
                                  const Telemetry *previous,
                                  Telemetry *telemetry,
                                  VehicleState *state,
                                  char *error,
                                  size_t error_size);

/* Apply the common guidance-facing metadata that depends on the configured
   vehicle/site rather than on the wire packet. Consumers may still add
   transport-specific rate or ground-track derivation before calling this
   helper; those fields are intentionally not recomputed here. */
void shuttle_sim_prepare_guidance_telemetry(
    Telemetry *telemetry,
    const LandingConfiguration *configuration,
    const PlanetModel *planet);

/* ShuttleSim model data files.  The KSP-fitted set lives under the git-ignored
   ShuttleSim/data/fitted/; a tracked, explicitly labelled reference model under
   ShuttleSim/reference-model/ keeps a clean checkout buildable, testable and
   simulatable.  The fitted file wins whenever it is readable. */
typedef enum {
    SHUTTLE_SIM_MODEL_ATMOSPHERE = 0,
    SHUTTLE_SIM_MODEL_AERO,
    SHUTTLE_SIM_MODEL_FORCE_BOOK,
    SHUTTLE_SIM_MODEL_ATTITUDE
} ShuttleSimModelFile;

/* Writes "<root>/<relative path>" (or the relative path when root is NULL or
   empty) into buffer and returns buffer, or NULL on invalid input/overflow. */
const char *shuttle_sim_model_path(const char *root, ShuttleSimModelFile kind,
                                   char *buffer, size_t buffer_size);

#endif
