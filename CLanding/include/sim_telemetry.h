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

/* Named model data profiles are selected explicitly; file existence never
   changes the selected plant. Profiles carry a vehicle model identity so
   STS and STS-N are not silently treated as interchangeable calibrations. */
typedef enum {
    SHUTTLE_SIM_MODEL_ATMOSPHERE = 0,
    SHUTTLE_SIM_MODEL_AERO,
    SHUTTLE_SIM_MODEL_FORCE_BOOK,
    SHUTTLE_SIM_MODEL_ATTITUDE,
    /* Guidance-side certified vessel-physics prior (force/q samples seeding
       VesselPhysicsModel).  Distinct from FORCE_BOOK, which is the direct force
       book aero_compute() consults (plant and MM305 terminal model). */
    SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR
} ShuttleSimModelFile;

/* Writes "<root>/<relative path>" (or the relative path when root is NULL or
   empty) of the active model profile's file into buffer and returns buffer, or
   NULL on invalid input/overflow/unknown profile.  A profile without a force
   book yields the literal "none".  The profile comes from
   KSP_LANDER_MODEL_PROFILE (default "identified"). */
const char *shuttle_sim_model_profile(void);
/* Aerodynamic profile identity, not a claim of independent certification. */
const char *shuttle_sim_model_vehicle_id(void);
bool shuttle_sim_model_matches_vehicle(const char *model_id, char *reason, size_t size);
const char *shuttle_sim_model_path(const char *root, ShuttleSimModelFile kind,
                                   char *buffer, size_t buffer_size);

#endif
