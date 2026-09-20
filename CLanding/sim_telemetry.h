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

#endif
