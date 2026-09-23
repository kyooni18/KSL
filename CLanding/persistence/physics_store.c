#include "physics_store.h"

#include <sqlite3.h>

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define HISTORY_QUERY_LIMIT 4096
#define STORE_COMMIT_BATCH 4
/*
 * Persistence-only decimation. Model support is counted by independent flights,
 * never by stored frame count, so this controls SQLite volume rather than
 * guidance confidence or feasibility. Two hertz is sufficient to retain
 * multiple raw observations while a vehicle traverses a local model cell.
 * decision-literal-ok: storage discretization, not a behavioral threshold.
 */
static const double archive_sample_period_s = 0.5;

typedef struct {
    char structure_id[PHYSICS_STORE_ID_CAPACITY];
    char environment_id[PHYSICS_STORE_ID_CAPACITY];
} ContextPair;

struct PhysicsStore {
    sqlite3 *db;
    PhysicsStoreMode mode;
    char path[PHYSICS_STORE_PATH_CAPACITY];
    char structure_id[PHYSICS_STORE_ID_CAPACITY];
    char structure_witness_id[PHYSICS_STORE_ID_CAPACITY];
    char environment_id[PHYSICS_STORE_ID_CAPACITY];
    char session_id[PHYSICS_STORE_ID_CAPACITY];
    char flight_key[PHYSICS_STORE_ID_CAPACITY];
    char vessel_name[128];
    char *structure_manifest_json;
    char *environment_manifest_json;
    char *environment_identity_json;
    ContextPair *contexts;
    size_t context_count;
    size_t context_capacity;
    bool has_contexts_table;
    bool has_sessions_table;
    bool has_flight_key;
    bool has_observations_table;
    bool has_quality_table;
    int timeline_epoch;
    bool has_last_ut;
    double last_ut;
    bool has_last_saved_ut;
    double last_saved_ut;
    unsigned pending;
};


/* Persistent physics state remains one translation unit so SQLite transactions,
 * schema helpers, and sample reduction retain exact behavior and static scope. */
#include "store/identity.inc"
#include "store/schema.inc"
#include "store/history.inc"
#include "store/reduction.inc"
#include "store/load_observe.inc"
