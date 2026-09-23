#ifndef KSP_LANDER_PHYSICS_STORE_H
#define KSP_LANDER_PHYSICS_STORE_H

#include "landing.h"

#include <stdbool.h>
#include <stddef.h>

#define PHYSICS_STORE_SCHEMA_VERSION 3
#define PHYSICS_STORE_ID_CAPACITY 128
#define PHYSICS_STORE_PATH_CAPACITY 1024

typedef enum {
    PHYSICS_STORE_READ_ONLY = 0,
    PHYSICS_STORE_READ_WRITE = 1
} PhysicsStoreMode;

typedef struct {
    const char *path;
    const char *model_id;
    const char *vessel_name;
    const char *structure_manifest_json;
    const char *environment_manifest_json;
    const char *session_id; /* Optional deterministic/test override. */
    const char *flight_key; /* Stable KSP-launch identity, e.g. UT-MET launch key. */
    PhysicsStoreMode mode;
} PhysicsStoreOptions;

typedef struct {
    bool sample_valid;
    bool body_non_rotating;
    const char *situation;
    double ut;
    double wall_time;
    double q;
    double mach;
    double aoa;
    double beta;
    double roll;
    double mass;
    bool has_dry_mass;
    double dry_mass;
    bool gear;
    bool brakes;
    int airbrakes;
    int rcs;
    bool has_center_of_mass_root;
    Vector3 center_of_mass_root; /* Raw kRPC root-part x,y,z axes. */
    bool has_moment_of_inertia;
    Vector3 moment_of_inertia;   /* Raw kRPC x,y,z ordering. */
    Vector3 position;            /* Raw kRPC body-non-rotating x,y,z. */
    Vector3 velocity;            /* Raw kRPC body-non-rotating x,y,z. */
    Vector3 lift;                /* Raw kRPC body-non-rotating x,y,z. */
    Vector3 drag;                /* Raw kRPC body-non-rotating x,y,z. */
    const char *control_profile;
    bool has_body_rates;
    Vector3 body_rates;          /* pitch, roll, yaw in deg/s. */
} PhysicsStoreObservation;

typedef struct PhysicsStore PhysicsStore;

/*
 * Open the prior-flight archive. READ_ONLY never creates, migrates, changes
 * pragmas, inserts sessions, or writes sidecar state; it is safe for the real
 * Runtime/Physics/observations.sqlite3 evidence during offline Skyline work.
 *
 * The manifest JSON shapes intentionally match the retired Python bridge:
 * structure manifests contain parts/geometry and environment manifests contain
 * planet/atmosphereCurveDigest. Model ID remains authoritative; structure
 * geometry is only a witness used to recover compatible pre-model-ID history.
 */
PhysicsStore *physics_store_open(const PhysicsStoreOptions *options,
                                 char *error, size_t error_size);
void physics_store_close(PhysicsStore *store);
bool physics_store_flush(PhysicsStore *store, char *error, size_t error_size);

const char *physics_store_structure_id(const PhysicsStore *store);
const char *physics_store_structure_witness_id(const PhysicsStore *store);
const char *physics_store_environment_id(const PhysicsStore *store);
const char *physics_store_session_id(const PhysicsStore *store);
const char *physics_store_flight_key(const PhysicsStore *store);
const char *physics_store_path(const PhysicsStore *store);
size_t physics_store_compatible_context_count(const PhysicsStore *store);

/*
 * Load the bounded prior into VesselAeroSample records. Current session and
 * current flight_key are excluded. A matching aero_observation_quality row is
 * the provenance marker for the modern validated-observation path; older rows
 * without that marker remain raw archive evidence but fail closed for
 * certification. Stored attitude/profile quality scores are diagnostic only.
 */
size_t physics_store_load_history(PhysicsStore *store,
                                  const PlanetModel *planet,
                                  VesselAeroSample *out_samples,
                                  size_t maximum,
                                  char *error, size_t error_size);

/*
 * Append a sparse raw observation for use by a future flight. Current-flight
 * data is persisted but is never returned by physics_store_load_history from
 * the same session/flight. READ_ONLY stores reject this call.
 */
bool physics_store_observe(PhysicsStore *store,
                           const PhysicsStoreObservation *observation,
                           char *error, size_t error_size);

#endif
