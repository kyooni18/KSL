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

static void set_error(char *error, size_t error_size, const char *message) {
    if (!error || error_size == 0) return;
    snprintf(error, error_size, "%s", message ? message : "physics store error");
}

static void set_sqlite_error(char *error, size_t error_size, sqlite3 *db, const char *prefix) {
    if (!error || error_size == 0) return;
    snprintf(error, error_size, "%s: %s", prefix ? prefix : "SQLite error",
             db ? sqlite3_errmsg(db) : "unknown SQLite error");
}

static bool copy_string(char *dst, size_t capacity, const char *src) {
    if (!dst || capacity == 0) return false;
    if (!src) src = "";
    int written = snprintf(dst, capacity, "%s", src);
    return written >= 0 && (size_t)written < capacity;
}

static char *duplicate_string(const char *src) {
    if (!src) return NULL;
    size_t length = strlen(src) + 1;
    char *copy = malloc(length);
    if (copy) memcpy(copy, src, length);
    return copy;
}

static bool starts_with(const char *text, const char *prefix) {
    if (!text || !prefix) return false;
    size_t n = strlen(prefix);
    return strncmp(text, prefix, n) == 0;
}

static bool finite_vector(Vector3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static double realtime_seconds(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return 0.0;
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* Compact, dependency-free SHA-256 used only for stable identity strings. */
typedef struct {
    uint32_t h[8];
    uint64_t bits;
    unsigned char block[64];
    size_t used;
} Sha256;

static uint32_t rotate_right(uint32_t v, unsigned n) {
    return (v >> n) | (v << (32U - n));
}

static void sha256_transform(Sha256 *s, const unsigned char block[64]) {
    static const uint32_t k[64] = {
        0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
        0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
        0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
        0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
        0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
        0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
        0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
        0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U
    };
    uint32_t w[64];
    for (unsigned i = 0; i < 16; i++) {
        unsigned j = i * 4U;
        w[i] = ((uint32_t)block[j] << 24) | ((uint32_t)block[j + 1] << 16) |
               ((uint32_t)block[j + 2] << 8) | (uint32_t)block[j + 3];
    }
    for (unsigned i = 16; i < 64; i++) {
        uint32_t s0 = rotate_right(w[i - 15], 7) ^ rotate_right(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = rotate_right(w[i - 2], 17) ^ rotate_right(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a=s->h[0],b=s->h[1],c=s->h[2],d=s->h[3],e=s->h[4],f=s->h[5],g=s->h[6],h=s->h[7];
    for (unsigned i = 0; i < 64; i++) {
        uint32_t s1=rotate_right(e,6)^rotate_right(e,11)^rotate_right(e,25);
        uint32_t ch=(e&f)^((~e)&g);
        uint32_t t1=h+s1+ch+k[i]+w[i];
        uint32_t s0=rotate_right(a,2)^rotate_right(a,13)^rotate_right(a,22);
        uint32_t maj=(a&b)^(a&c)^(b&c);
        uint32_t t2=s0+maj;
        h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
    }
    s->h[0]+=a;s->h[1]+=b;s->h[2]+=c;s->h[3]+=d;
    s->h[4]+=e;s->h[5]+=f;s->h[6]+=g;s->h[7]+=h;
}

static void sha256_init(Sha256 *s) {
    static const uint32_t initial[8] = {
        0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
        0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U
    };
    memcpy(s->h, initial, sizeof(initial));
    s->bits = 0;
    s->used = 0;
}

static void sha256_update(Sha256 *s, const void *data, size_t length) {
    const unsigned char *p = data;
    s->bits += (uint64_t)length * 8U;
    while (length) {
        size_t room = 64 - s->used;
        size_t take = length < room ? length : room;
        memcpy(s->block + s->used, p, take);
        s->used += take;
        p += take;
        length -= take;
        if (s->used == 64) {
            sha256_transform(s, s->block);
            s->used = 0;
        }
    }
}

static void sha256_finish(Sha256 *s, unsigned char digest[32]) {
    uint64_t bits = s->bits;
    unsigned char one = 0x80;
    sha256_update(s, &one, 1);
    unsigned char zero = 0;
    while (s->used != 56) sha256_update(s, &zero, 1);
    unsigned char length[8];
    for (unsigned i = 0; i < 8; i++) length[7 - i] = (unsigned char)(bits >> (i * 8U));
    sha256_update(s, length, sizeof(length));
    for (unsigned i = 0; i < 8; i++) {
        digest[i * 4] = (unsigned char)(s->h[i] >> 24);
        digest[i * 4 + 1] = (unsigned char)(s->h[i] >> 16);
        digest[i * 4 + 2] = (unsigned char)(s->h[i] >> 8);
        digest[i * 4 + 3] = (unsigned char)s->h[i];
    }
}

static void digest_text(const char *text, char out[PHYSICS_STORE_ID_CAPACITY]) {
    Sha256 sha;
    unsigned char digest[32];
    sha256_init(&sha);
    sha256_update(&sha, text ? text : "", text ? strlen(text) : 0);
    sha256_finish(&sha, digest);
    char *p = out;
    memcpy(p, "sha256:", 7); p += 7;
    for (unsigned i = 0; i < 32; i++) {
        static const char hex[] = "0123456789abcdef";
        *p++ = hex[digest[i] >> 4];
        *p++ = hex[digest[i] & 15];
    }
    *p = 0;
}

typedef struct {
    JsonToken *tokens;
    JsonDoc doc;
} ParsedJson;

static void parsed_json_clear(ParsedJson *p) {
    if (!p) return;
    free(p->tokens);
    memset(p, 0, sizeof(*p));
}

static bool parsed_json_init(ParsedJson *p, const char *text) {
    if (!p || !text) return false;
    memset(p, 0, sizeof(*p));
    int capacity = 256;
    while (capacity <= 32768) {
        JsonToken *tokens = calloc((size_t)capacity, sizeof(*tokens));
        if (!tokens) return false;
        JsonDoc doc;
        int count = json_parse(text, tokens, capacity, &doc);
        if (count >= 1) {
            p->tokens = tokens;
            p->doc = doc;
            return true;
        }
        free(tokens);
        if (count != -1) return false;
        capacity *= 2;
    }
    return false;
}

static char *json_string_dup(const JsonDoc *doc, int index) {
    if (!doc || index < 0 || index >= doc->count || doc->tokens[index].type != JSON_STRING) return NULL;
    size_t raw = (size_t)(doc->tokens[index].end - doc->tokens[index].start);
    char *result = malloc(raw + 1);
    if (!result) return NULL;
    if (!json_string(doc, index, result, raw + 1)) { free(result); return NULL; }
    return result;
}

typedef struct {
    char *key;
    int value_index;
} JsonPair;

static int compare_json_pair(const void *a, const void *b) {
    const JsonPair *pa = a, *pb = b;
    return strcmp(pa->key, pb->key);
}

static bool ignored_planet_identity_key(const char *key) {
    return strcmp(key, "northAxisKRPC") == 0 || strcmp(key, "primeMeridianAtEpochKRPC") == 0 ||
           strcmp(key, "epochUT") == 0;
}

static bool canonical_json_token(JsonWriter *writer, const JsonDoc *doc, int index, bool filter_planet_coordinates) {
    if (!writer || !doc || index < 0 || index >= doc->count) return false;
    const JsonToken *token = &doc->tokens[index];
    if (token->type == JSON_STRING) {
        char *value = json_string_dup(doc, index);
        if (!value) return false;
        jw_string(writer, value);
        free(value);
        return !writer->failed;
    }
    if (token->type == JSON_PRIMITIVE) {
        size_t n = (size_t)(token->end - token->start);
        const char *raw = doc->text + token->start;
        if ((n == 4 && strncmp(raw, "true", 4) == 0) || (n == 5 && strncmp(raw, "false", 5) == 0) ||
            (n == 4 && strncmp(raw, "null", 4) == 0)) {
            char small[6]; memcpy(small, raw, n); small[n] = 0; jw_raw(writer, small); return !writer->failed;
        }
        char number[96];
        if (n >= sizeof(number)) return false;
        memcpy(number, raw, n); number[n] = 0;
        errno = 0; char *end = NULL; double value = strtod(number, &end);
        if (errno || end == number || *end || !isfinite(value)) return false;
        jw_number(writer, value);
        return !writer->failed;
    }
    if (token->type == JSON_ARRAY) {
        jw_char(writer, '[');
        for (int i = 0; i < token->size; i++) {
            int child = json_array_get(doc, index, i);
            if (child < 0) return false;
            if (i) jw_char(writer, ',');
            if (!canonical_json_token(writer, doc, child, false)) return false;
        }
        jw_char(writer, ']');
        return !writer->failed;
    }
    if (token->type == JSON_OBJECT) {
        JsonPair *pairs = calloc((size_t)token->size / 2U + 1U, sizeof(*pairs));
        if (!pairs) return false;
        size_t count = 0;
        int i = index + 1;
        while (i < doc->count && doc->tokens[i].start < token->end) {
            int key_index = i, value_index = i + 1;
            if (value_index >= doc->count || doc->tokens[key_index].type != JSON_STRING) { free(pairs); return false; }
            char *key = json_string_dup(doc, key_index);
            if (!key) { free(pairs); return false; }
            if (!(filter_planet_coordinates && ignored_planet_identity_key(key))) {
                pairs[count].key = key;
                pairs[count].value_index = value_index;
                count++;
            } else free(key);
            i = json_skip(doc, value_index);
        }
        qsort(pairs, count, sizeof(*pairs), compare_json_pair);
        jw_char(writer, '{');
        for (size_t p = 0; p < count; p++) {
            if (p) jw_char(writer, ',');
            jw_string(writer, pairs[p].key); jw_char(writer, ':');
            if (!canonical_json_token(writer, doc, pairs[p].value_index, false)) {
                for (size_t j = 0; j < count; j++) free(pairs[j].key);
                free(pairs); return false;
            }
        }
        jw_char(writer, '}');
        for (size_t p = 0; p < count; p++) free(pairs[p].key);
        free(pairs);
        return !writer->failed;
    }
    return false;
}

static char *environment_identity_json(const char *manifest_json) {
    ParsedJson parsed;
    if (!parsed_json_init(&parsed, manifest_json)) return NULL;
    const JsonDoc *doc = &parsed.doc;
    int root = 0;
    if (doc->tokens[root].type != JSON_OBJECT) { parsed_json_clear(&parsed); return NULL; }
    int planet = json_object_get(doc, root, "planet");
    int digest = json_object_get(doc, root, "atmosphereCurveDigest");
    if (planet < 0 || doc->tokens[planet].type != JSON_OBJECT) { parsed_json_clear(&parsed); return NULL; }
    JsonWriter writer; jw_init(&writer);
    jw_raw(&writer, "{\"atmosphereCurveDigest\":");
    if (digest >= 0) canonical_json_token(&writer, doc, digest, false); else jw_null(&writer);
    jw_raw(&writer, ",\"planet\":");
    bool ok = canonical_json_token(&writer, doc, planet, true);
    jw_raw(&writer, ",\"schema\":3}");
    char *result = ok && !writer.failed ? duplicate_string(writer.data) : NULL;
    jw_free(&writer);
    parsed_json_clear(&parsed);
    return result;
}

typedef struct {
    char *name;
    long x, y, z;
} StructureGeometry;

typedef struct {
    char **part_names;
    size_t part_count;
    StructureGeometry *geometry;
    size_t geometry_count;
    bool geometry_available;
} StructureIdentity;

static int compare_string_ptr(const void *a, const void *b) {
    const char *const *sa = a, *const *sb = b;
    return strcmp(*sa, *sb);
}

static int compare_geometry(const void *a, const void *b) {
    const StructureGeometry *ga = a, *gb = b;
    int n = strcmp(ga->name, gb->name); if (n) return n;
    if (ga->x != gb->x) return ga->x < gb->x ? -1 : 1;
    if (ga->y != gb->y) return ga->y < gb->y ? -1 : 1;
    if (ga->z != gb->z) return ga->z < gb->z ? -1 : 1;
    return 0;
}

static void structure_identity_clear(StructureIdentity *identity) {
    if (!identity) return;
    for (size_t i = 0; i < identity->part_count; i++) free(identity->part_names[i]);
    for (size_t i = 0; i < identity->geometry_count; i++) free(identity->geometry[i].name);
    free(identity->part_names); free(identity->geometry); memset(identity, 0, sizeof(*identity));
}

static bool structure_identity_init(StructureIdentity *identity, const char *manifest_json) {
    memset(identity, 0, sizeof(*identity));
    ParsedJson parsed;
    if (!parsed_json_init(&parsed, manifest_json)) return false;
    const JsonDoc *doc = &parsed.doc;
    if (doc->tokens[0].type != JSON_OBJECT) { parsed_json_clear(&parsed); return false; }
    int parts = json_object_get(doc, 0, "parts");
    if (parts >= 0 && doc->tokens[parts].type == JSON_ARRAY) {
        int count = doc->tokens[parts].size;
        identity->part_names = calloc((size_t)count, sizeof(*identity->part_names));
        if (count && !identity->part_names) { parsed_json_clear(&parsed); return false; }
        for (int i = 0; i < count; i++) {
            int part = json_array_get(doc, parts, i);
            int name = json_object_get(doc, part, "name");
            char *value = name >= 0 ? json_string_dup(doc, name) : duplicate_string("<unavailable>");
            if (!value) { parsed_json_clear(&parsed); structure_identity_clear(identity); return false; }
            identity->part_names[identity->part_count++] = value;
        }
        qsort(identity->part_names, identity->part_count, sizeof(*identity->part_names), compare_string_ptr);
    }
    bool geometry_complete = json_boolean(doc, json_object_get(doc, 0, "geometryComplete"), false);
    int geometry = json_object_get(doc, 0, "geometry");
    bool geometry_valid = geometry_complete && geometry >= 0 && doc->tokens[geometry].type == JSON_ARRAY;
    if (geometry_valid) {
        int count = doc->tokens[geometry].size;
        identity->geometry = calloc((size_t)count, sizeof(*identity->geometry));
        if (count && !identity->geometry) { parsed_json_clear(&parsed); structure_identity_clear(identity); return false; }
        for (int i = 0; i < count; i++) {
            int item = json_array_get(doc, geometry, i);
            int name = json_object_get(doc, item, "name");
            int position = json_object_get(doc, item, "position");
            if (name < 0 || position < 0 || doc->tokens[position].type != JSON_ARRAY || doc->tokens[position].size != 3) { geometry_valid = false; break; }
            char *value = json_string_dup(doc, name);
            double p[3];
            for (int j = 0; j < 3; j++) p[j] = json_number(doc, json_array_get(doc, position, j), NAN);
            if (!value || !isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2])) { free(value); geometry_valid = false; break; }
            StructureGeometry *g = &identity->geometry[identity->geometry_count++];
            g->name = value;
            g->x = lrint(p[0] / 0.05); g->y = lrint(p[1] / 0.05); g->z = lrint(p[2] / 0.05);
        }
    }
    identity->geometry_available = geometry_valid && identity->part_count > 0 && identity->geometry_count == identity->part_count;
    if (!identity->geometry_available) {
        for (size_t i = 0; i < identity->geometry_count; i++) free(identity->geometry[i].name);
        free(identity->geometry); identity->geometry = NULL; identity->geometry_count = 0;
    } else qsort(identity->geometry, identity->geometry_count, sizeof(*identity->geometry), compare_geometry);
    parsed_json_clear(&parsed);
    return true;
}

static bool structure_identity_equal(const StructureIdentity *a, const StructureIdentity *b) {
    if (!a || !b || a->part_count != b->part_count || a->geometry_available != b->geometry_available) return false;
    for (size_t i = 0; i < a->part_count; i++) if (strcmp(a->part_names[i], b->part_names[i]) != 0) return false;
    if (!a->geometry_available) return true;
    for (size_t i = 0; i < a->geometry_count; i++) {
        const StructureGeometry *x=&a->geometry[i], *y=&b->geometry[i];
        if (strcmp(x->name,y->name)!=0 || x->x!=y->x || x->y!=y->y || x->z!=y->z) return false;
    }
    return true;
}

static char *structure_identity_text(const StructureIdentity *identity) {
    JsonWriter writer; jw_init(&writer);
    jw_raw(&writer, "structure-v3|parts="); jw_integer(&writer, (long long)identity->part_count);
    for (size_t i=0;i<identity->part_count;i++){jw_char(&writer,'|');jw_string(&writer,identity->part_names[i]);}
    jw_raw(&writer, "|geometry="); jw_bool(&writer, identity->geometry_available);
    if (identity->geometry_available) for (size_t i=0;i<identity->geometry_count;i++) {
        const StructureGeometry *g=&identity->geometry[i]; jw_char(&writer,'|'); jw_string(&writer,g->name);
        jw_char(&writer,':');jw_integer(&writer,g->x);jw_char(&writer,',');jw_integer(&writer,g->y);jw_char(&writer,',');jw_integer(&writer,g->z);
    }
    char *result = writer.failed ? NULL : duplicate_string(writer.data); jw_free(&writer); return result;
}

static bool table_exists(sqlite3 *db, const char *table) {
    sqlite3_stmt *stmt=NULL; bool exists=false;
    if (sqlite3_prepare_v2(db,"SELECT 1 FROM sqlite_master WHERE type='table' AND name=? LIMIT 1",-1,&stmt,NULL)==SQLITE_OK) {
        sqlite3_bind_text(stmt,1,table,-1,SQLITE_STATIC); exists=sqlite3_step(stmt)==SQLITE_ROW;
    }
    sqlite3_finalize(stmt); return exists;
}

static bool column_exists(sqlite3 *db, const char *table, const char *column) {
    char sql[256]; snprintf(sql,sizeof(sql),"PRAGMA table_info(%s)",table);
    sqlite3_stmt *stmt=NULL; bool exists=false;
    if (sqlite3_prepare_v2(db,sql,-1,&stmt,NULL)==SQLITE_OK) while (sqlite3_step(stmt)==SQLITE_ROW) {
        const char *name=(const char*)sqlite3_column_text(stmt,1); if (name&&strcmp(name,column)==0){exists=true;break;}
    }
    sqlite3_finalize(stmt); return exists;
}

static bool execute_sql(sqlite3 *db, const char *sql, char *error, size_t error_size) {
    char *message=NULL; int rc=sqlite3_exec(db,sql,NULL,NULL,&message);
    if (rc==SQLITE_OK) return true;
    if (error&&error_size) snprintf(error,error_size,"SQLite migration failed: %s",message?message:sqlite3_errmsg(db));
    sqlite3_free(message); return false;
}


static bool ensure_column(sqlite3 *db, const char *table, const char *column,
                          const char *declaration, char *error, size_t error_size) {
    if (column_exists(db, table, column)) return true;
    char sql[512];
    snprintf(sql, sizeof(sql), "ALTER TABLE %s ADD COLUMN %s %s", table, column, declaration);
    return execute_sql(db, sql, error, error_size);
}

static void refresh_schema_flags(PhysicsStore *store) {
    store->has_contexts_table = table_exists(store->db, "physics_contexts");
    store->has_sessions_table = table_exists(store->db, "physics_sessions");
    store->has_flight_key = store->has_sessions_table && column_exists(store->db, "physics_sessions", "flight_key");
    store->has_observations_table = table_exists(store->db, "aero_observations");
    store->has_quality_table = table_exists(store->db, "aero_observation_quality");
}

static bool migrate_schema(PhysicsStore *store, char *error, size_t error_size) {
    static const char *schema_sql =
        "BEGIN IMMEDIATE;"
        "CREATE TABLE IF NOT EXISTS physics_meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);"
        "CREATE TABLE IF NOT EXISTS physics_contexts("
        " structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,structure_json TEXT NOT NULL,"
        " environment_json TEXT NOT NULL,first_seen REAL NOT NULL,last_seen REAL NOT NULL,"
        " PRIMARY KEY(structure_id,environment_id));"
        "CREATE TABLE IF NOT EXISTS physics_sessions("
        " session_id TEXT PRIMARY KEY,structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,"
        " vessel_name TEXT NOT NULL,started_wall REAL NOT NULL,flight_key TEXT);"
        "CREATE TABLE IF NOT EXISTS aero_observations("
        " id INTEGER PRIMARY KEY AUTOINCREMENT,structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,"
        " session_id TEXT NOT NULL,timeline_epoch INTEGER NOT NULL,ut REAL NOT NULL,wall_time REAL NOT NULL,"
        " q REAL NOT NULL,mach REAL NOT NULL,aoa REAL NOT NULL,beta REAL NOT NULL,roll REAL NOT NULL,mass REAL NOT NULL,"
        " dry_mass REAL,gear INTEGER NOT NULL,brakes INTEGER NOT NULL,airbrakes INTEGER NOT NULL,rcs INTEGER NOT NULL,"
        " com_root_x REAL,com_root_y REAL,com_root_z REAL,inertia_x REAL,inertia_y REAL,inertia_z REAL,"
        " pos_x REAL NOT NULL,pos_y REAL NOT NULL,pos_z REAL NOT NULL,vel_x REAL NOT NULL,vel_y REAL NOT NULL,vel_z REAL NOT NULL,"
        " lift_x REAL NOT NULL,lift_y REAL NOT NULL,lift_z REAL NOT NULL,drag_x REAL NOT NULL,drag_y REAL NOT NULL,drag_z REAL NOT NULL);"
        "CREATE INDEX IF NOT EXISTS aero_context_recent ON aero_observations(structure_id,environment_id,id DESC);"
        "CREATE TABLE IF NOT EXISTS aero_observation_quality("
        " observation_id INTEGER PRIMARY KEY,control_profile TEXT NOT NULL,body_pitch_rate REAL,body_roll_rate REAL,body_yaw_rate REAL,"
        " quality REAL NOT NULL,eligible INTEGER NOT NULL,"
        " FOREIGN KEY(observation_id) REFERENCES aero_observations(id) ON DELETE CASCADE);"
        "COMMIT;";
    if (!execute_sql(store->db, schema_sql, error, error_size)) {
        sqlite3_exec(store->db, "ROLLBACK", NULL, NULL, NULL);
        return false;
    }
    /* Schema 2 already had the aerodynamic columns; schema 3 added launch
       identity and quality metadata. Be tolerant of early development DBs that
       missed optional mass/geometry witnesses. */
    if (!ensure_column(store->db, "physics_sessions", "flight_key", "TEXT", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "dry_mass", "REAL", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "com_root_x", "REAL", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "com_root_y", "REAL", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "com_root_z", "REAL", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "inertia_x", "REAL", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "inertia_y", "REAL", error, error_size) ||
        !ensure_column(store->db, "aero_observations", "inertia_z", "REAL", error, error_size)) return false;
    if (!execute_sql(store->db, "INSERT OR REPLACE INTO physics_meta(key,value) VALUES('schema','3')", error, error_size)) return false;
    refresh_schema_flags(store);
    return true;
}

static bool add_context(PhysicsStore *store, const char *structure_id, const char *environment_id) {
    for (size_t i=0;i<store->context_count;i++) {
        if (strcmp(store->contexts[i].structure_id,structure_id)==0 &&
            strcmp(store->contexts[i].environment_id,environment_id)==0) return true;
    }
    if (store->context_count == store->context_capacity) {
        size_t next = store->context_capacity ? store->context_capacity * 2 : 8;
        ContextPair *grown = realloc(store->contexts, next * sizeof(*grown));
        if (!grown) return false;
        store->contexts = grown; store->context_capacity = next;
    }
    ContextPair *pair = &store->contexts[store->context_count++];
    if (!copy_string(pair->structure_id,sizeof(pair->structure_id),structure_id) ||
        !copy_string(pair->environment_id,sizeof(pair->environment_id),environment_id)) {
        store->context_count--; return false;
    }
    return true;
}

static bool environment_has_curve_digest(const char *manifest_json) {
    ParsedJson parsed;
    if (!parsed_json_init(&parsed, manifest_json)) return false;
    int digest = json_object_get(&parsed.doc, 0, "atmosphereCurveDigest");
    bool present = digest >= 0 && parsed.doc.tokens[digest].type != JSON_PRIMITIVE ? true : false;
    if (digest >= 0 && parsed.doc.tokens[digest].type == JSON_PRIMITIVE) {
        present = !json_token_eq(&parsed.doc, digest, "null");
    }
    parsed_json_clear(&parsed);
    return present;
}

static char *environment_planet_identity_json(const char *manifest_json) {
    ParsedJson parsed;
    if (!parsed_json_init(&parsed, manifest_json)) return NULL;
    const JsonDoc *doc=&parsed.doc;
    int planet=json_object_get(doc,0,"planet");
    if(planet<0||doc->tokens[planet].type!=JSON_OBJECT){parsed_json_clear(&parsed);return NULL;}
    JsonWriter writer;jw_init(&writer);jw_raw(&writer,"{\"planet\":");
    bool ok=canonical_json_token(&writer,doc,planet,true);
    jw_raw(&writer,",\"schema\":3}");
    char *result=ok&&!writer.failed?duplicate_string(writer.data):NULL;
    jw_free(&writer);parsed_json_clear(&parsed);return result;
}

static bool environment_manifests_equal(const char *a, const char *b) {
    char *na=environment_identity_json(a), *nb=environment_identity_json(b);
    bool equal=na&&nb&&strcmp(na,nb)==0;
    free(na);free(nb);
    if(equal)return true;

    /* The retired Python bridge stored a SHA-256 of its sampled atmosphere
       curve. Native C-Nano has the actual sampled PlanetModel but intentionally
       does not reproduce Python's float-to-JSON byte spelling. During this one
       compatibility boundary, a missing/null current digest may therefore
       reuse an older manual-model context only when every durable planet field
       (name, radius, mu, rotation, atmosphere depth/density, gamma, etc.) is
       otherwise identical. A non-null digest mismatch remains a hard split. */
    if(environment_has_curve_digest(a)&&environment_has_curve_digest(b))return false;
    char *pa=environment_planet_identity_json(a),*pb=environment_planet_identity_json(b);
    equal=pa&&pb&&strcmp(pa,pb)==0;free(pa);free(pb);return equal;
}

static bool discover_compatible_contexts(PhysicsStore *store,
                                         const StructureIdentity *current_structure,
                                         char *error,size_t error_size) {
    if (!store->has_contexts_table) return add_context(store,store->structure_id,store->environment_id);
    sqlite3_stmt *stmt=NULL;
    const char *sql="SELECT structure_id,environment_id,structure_json,environment_json FROM physics_contexts ORDER BY last_seen DESC";
    if (sqlite3_prepare_v2(store->db,sql,-1,&stmt,NULL)!=SQLITE_OK) {
        set_sqlite_error(error,error_size,store->db,"Could not inspect physics contexts"); return false;
    }
    bool reused_manual_environment=false;
    while (sqlite3_step(stmt)==SQLITE_ROW) {
        const char *structure_id=(const char*)sqlite3_column_text(stmt,0);
        const char *environment_id=(const char*)sqlite3_column_text(stmt,1);
        const char *structure_json=(const char*)sqlite3_column_text(stmt,2);
        const char *environment_json=(const char*)sqlite3_column_text(stmt,3);
        if(!structure_id||!environment_id||!structure_json||!environment_json)continue;
        if(!environment_manifests_equal(store->environment_manifest_json,environment_json))continue;
        bool same_manual=strcmp(structure_id,store->structure_id)==0;
        bool matching_legacy=false;
        if(!same_manual&&starts_with(structure_id,"sha256:")){
            StructureIdentity legacy;
            if(structure_identity_init(&legacy,structure_json)){
                matching_legacy=structure_identity_equal(current_structure,&legacy);
                structure_identity_clear(&legacy);
            }
        }
        if(!same_manual&&!matching_legacy)continue;
        if(!add_context(store,structure_id,environment_id)){sqlite3_finalize(stmt);set_error(error,error_size,"Out of memory collecting physics contexts");return false;}
        /* Numeric JSON spelling differs slightly between Python and C. Reuse
           the persisted schema-3 ID when the semantic identity is the same so
           native C joins the accumulated archive instead of forking it. */
        if(same_manual&&!reused_manual_environment){
            if(!copy_string(store->environment_id,sizeof(store->environment_id),environment_id)){
                sqlite3_finalize(stmt);set_error(error,error_size,"Persisted environment ID is too long");return false;
            }
            reused_manual_environment=true;
        }
    }
    int rc=sqlite3_finalize(stmt);
    if(rc!=SQLITE_OK){set_sqlite_error(error,error_size,store->db,"Could not finish physics context scan");return false;}
    return add_context(store,store->structure_id,store->environment_id);
}

static bool store_current_context(PhysicsStore *store,char *error,size_t error_size){
    double now=realtime_seconds();sqlite3_stmt *stmt=NULL;
    const char *context_sql=
        "INSERT INTO physics_contexts(structure_id,environment_id,structure_json,environment_json,first_seen,last_seen)"
        " VALUES(?,?,?,?,?,?) ON CONFLICT(structure_id,environment_id) DO UPDATE SET"
        " structure_json=excluded.structure_json,environment_json=excluded.environment_json,last_seen=excluded.last_seen";
    if(sqlite3_prepare_v2(store->db,context_sql,-1,&stmt,NULL)!=SQLITE_OK){set_sqlite_error(error,error_size,store->db,"Could not prepare physics context");return false;}
    sqlite3_bind_text(stmt,1,store->structure_id,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,2,store->environment_id,-1,SQLITE_STATIC);
    sqlite3_bind_text(stmt,3,store->structure_manifest_json,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,4,store->environment_manifest_json,-1,SQLITE_STATIC);
    sqlite3_bind_double(stmt,5,now);sqlite3_bind_double(stmt,6,now);
    bool ok=sqlite3_step(stmt)==SQLITE_DONE;sqlite3_finalize(stmt);
    if(!ok){set_sqlite_error(error,error_size,store->db,"Could not store physics context");return false;}
    const char *session_sql="INSERT INTO physics_sessions(session_id,structure_id,environment_id,vessel_name,started_wall,flight_key) VALUES(?,?,?,?,?,?)";
    if(sqlite3_prepare_v2(store->db,session_sql,-1,&stmt,NULL)!=SQLITE_OK){set_sqlite_error(error,error_size,store->db,"Could not prepare physics session");return false;}
    sqlite3_bind_text(stmt,1,store->session_id,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,2,store->structure_id,-1,SQLITE_STATIC);
    sqlite3_bind_text(stmt,3,store->environment_id,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,4,store->vessel_name,-1,SQLITE_STATIC);
    sqlite3_bind_double(stmt,5,now);sqlite3_bind_text(stmt,6,store->flight_key,-1,SQLITE_STATIC);
    ok=sqlite3_step(stmt)==SQLITE_DONE;sqlite3_finalize(stmt);
    if(!ok){set_sqlite_error(error,error_size,store->db,"Could not store physics session");return false;}
    return sqlite3_exec(store->db,"COMMIT",NULL,NULL,NULL)==SQLITE_OK || sqlite3_get_autocommit(store->db);
}

static void make_session_id(char out[PHYSICS_STORE_ID_CAPACITY]){
    struct timespec ts;clock_gettime(CLOCK_REALTIME,&ts);
    snprintf(out,PHYSICS_STORE_ID_CAPACITY,"native-%ld-%ld-%ld",(long)getpid(),(long)ts.tv_sec,(long)ts.tv_nsec);
}

PhysicsStore *physics_store_open(const PhysicsStoreOptions *options,char *error,size_t error_size){
    if(!options||!options->path||!*options->path||!options->model_id||!*options->model_id||
       !options->structure_manifest_json||!options->environment_manifest_json){
        set_error(error,error_size,"Physics store requires path, model ID, structure manifest and environment manifest");return NULL;
    }
    StructureIdentity structure;
    if(!structure_identity_init(&structure,options->structure_manifest_json)){
        set_error(error,error_size,"Invalid structure manifest JSON");return NULL;
    }
    char *structure_text=structure_identity_text(&structure);
    char *environment_text=environment_identity_json(options->environment_manifest_json);
    if(!structure_text||!environment_text){structure_identity_clear(&structure);free(structure_text);free(environment_text);set_error(error,error_size,"Invalid physics identity manifest");return NULL;}
    PhysicsStore *store=calloc(1,sizeof(*store));
    if(!store){structure_identity_clear(&structure);free(structure_text);free(environment_text);set_error(error,error_size,"Out of memory opening physics store");return NULL;}
    store->mode=options->mode;
    if(!copy_string(store->path,sizeof(store->path),options->path)||
       snprintf(store->structure_id,sizeof(store->structure_id),"model:%s",options->model_id)>=(int)sizeof(store->structure_id)){
        set_error(error,error_size,"Physics store path or model ID is too long");goto fail;
    }
    digest_text(structure_text,store->structure_witness_id);digest_text(environment_text,store->environment_id);
    store->structure_manifest_json=duplicate_string(options->structure_manifest_json);
    store->environment_manifest_json=duplicate_string(options->environment_manifest_json);
    store->environment_identity_json=environment_text;environment_text=NULL;
    if(!store->structure_manifest_json||!store->environment_manifest_json){set_error(error,error_size,"Out of memory copying physics manifests");goto fail;}
    copy_string(store->vessel_name,sizeof(store->vessel_name),options->vessel_name?options->vessel_name:options->model_id);
    if(options->session_id&&*options->session_id)copy_string(store->session_id,sizeof(store->session_id),options->session_id);else make_session_id(store->session_id);
    if(options->flight_key&&*options->flight_key)copy_string(store->flight_key,sizeof(store->flight_key),options->flight_key);else copy_string(store->flight_key,sizeof(store->flight_key),store->session_id);
    int flags=options->mode==PHYSICS_STORE_READ_ONLY?SQLITE_OPEN_READONLY:(SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE);
    int rc=sqlite3_open_v2(store->path,&store->db,flags,NULL);
    if(rc!=SQLITE_OK){set_sqlite_error(error,error_size,store->db,"Could not open physics database");goto fail;}
    sqlite3_stmt *health=NULL;
    if(sqlite3_prepare_v2(store->db,"PRAGMA schema_version",-1,&health,NULL)!=SQLITE_OK||sqlite3_step(health)!=SQLITE_ROW){
        sqlite3_finalize(health);set_sqlite_error(error,error_size,store->db,"Physics database is corrupt or unreadable");goto fail;
    }
    sqlite3_finalize(health);
    sqlite3_busy_timeout(store->db,250);
    if(store->mode==PHYSICS_STORE_READ_WRITE){
        if(!migrate_schema(store,error,error_size))goto fail;
    }else refresh_schema_flags(store);
    if(!discover_compatible_contexts(store,&structure,error,error_size))goto fail;
    if(store->mode==PHYSICS_STORE_READ_WRITE&&!store_current_context(store,error,error_size))goto fail;
    structure_identity_clear(&structure);free(structure_text);return store;
fail:
    structure_identity_clear(&structure);free(structure_text);free(environment_text);
    physics_store_close(store);return NULL;
}

void physics_store_close(PhysicsStore *store){
    if(!store)return;
    if(store->db){if(store->mode==PHYSICS_STORE_READ_WRITE&&store->pending)sqlite3_exec(store->db,"COMMIT",NULL,NULL,NULL);sqlite3_close(store->db);}
    free(store->structure_manifest_json);free(store->environment_manifest_json);free(store->environment_identity_json);free(store->contexts);free(store);
}

bool physics_store_flush(PhysicsStore *store,char *error,size_t error_size){
    if(!store||!store->db){set_error(error,error_size,"Physics store is closed");return false;}
    if(store->mode==PHYSICS_STORE_READ_ONLY)return true;
    if(sqlite3_exec(store->db,"COMMIT",NULL,NULL,NULL)!=SQLITE_OK&&!sqlite3_get_autocommit(store->db)){
        set_sqlite_error(error,error_size,store->db,"Could not flush physics store");return false;
    }
    store->pending=0;return true;
}

const char *physics_store_structure_id(const PhysicsStore*s){return s?s->structure_id:"";}
const char *physics_store_structure_witness_id(const PhysicsStore*s){return s?s->structure_witness_id:"";}
const char *physics_store_environment_id(const PhysicsStore*s){return s?s->environment_id:"";}
const char *physics_store_session_id(const PhysicsStore*s){return s?s->session_id:"";}
const char *physics_store_flight_key(const PhysicsStore*s){return s?s->flight_key:"";}
const char *physics_store_path(const PhysicsStore*s){return s?s->path:"";}
size_t physics_store_compatible_context_count(const PhysicsStore*s){return s?s->context_count:0;}


typedef struct {
    long long id;
    char session_id[PHYSICS_STORE_ID_CAPACITY];
    int timeline_epoch;
    char source[PHYSICS_STORE_ID_CAPACITY];
    double ut, q, mach, aoa, beta, roll, mass;
    int gear, brakes, airbrakes, rcs;
    Vector3 position, velocity, lift, drag;
} HistoryRow;

typedef struct {
    int q, mach, aoa, beta, gear, brakes, airbrakes;
} CellKey;

typedef struct {
    CellKey key;
    HistoryRow row;
    char source[PHYSICS_STORE_ID_CAPACITY];
} SourceCandidate;

typedef struct {
    CellKey key;
    HistoryRow row;
    unsigned independent;
} SelectedCell;

static bool cell_equal(CellKey a,CellKey b){return a.q==b.q&&a.mach==b.mach&&a.aoa==b.aoa&&a.beta==b.beta&&a.gear==b.gear&&a.brakes==b.brakes&&a.airbrakes==b.airbrakes;}
static CellKey row_cell(const HistoryRow*r){
    double q=fmax(r->q,1e-9);CellKey k={(int)lrint(log(q)/log(2.0)),(int)lrint(r->mach*2.0),(int)lrint(r->aoa/4.0),(int)lrint(r->beta/4.0),r->gear,r->brakes,r->airbrakes};return k;
}

static void append_context_where(JsonWriter*w,const PhysicsStore*s,const char*alias){
    jw_char(w,'(');
    for(size_t i=0;i<s->context_count;i++){
        if(i)jw_raw(w," OR ");jw_char(w,'(');jw_raw(w,alias);jw_raw(w,".structure_id=? AND ");jw_raw(w,alias);jw_raw(w,".environment_id=?)");
    }
    jw_char(w,')');
}

static int bind_contexts(sqlite3_stmt*stmt,const PhysicsStore*s,int index){
    for(size_t i=0;i<s->context_count;i++){
        sqlite3_bind_text(stmt,index++,s->contexts[i].structure_id,-1,SQLITE_STATIC);
        sqlite3_bind_text(stmt,index++,s->contexts[i].environment_id,-1,SQLITE_STATIC);
    }
    return index;
}


static bool history_query(
        PhysicsStore *s,
        HistoryRow **rows,
        size_t *row_count,
        char *error,
        size_t error_size) {
    *rows = NULL;
    *row_count = 0;
    if (!s || !s->db || !s->has_observations_table ||
        !s->has_quality_table || s->context_count == 0)
        return true;

    /*
     * The quality-table join is a provenance boundary, not a behavioral
     * score. Rows predating the modern validated-observation path have no
     * machine-checkable sample-valid/body-frame contract and therefore are
     * not certified. For modern rows, recovery profile, body rate, AoA and
     * sideslip remain measured coordinates rather than reasons to discard an
     * aerodynamic force observation.
     */
    JsonWriter sql;
    jw_init(&sql);
    jw_raw(&sql,
        "SELECT a.id,a.session_id,a.timeline_epoch,a.ut,a.q,a.mach,a.aoa,"
        "a.beta,a.roll,a.mass,a.gear,a.brakes,a.airbrakes,a.rcs,"
        "a.pos_x,a.pos_y,a.pos_z,a.vel_x,a.vel_y,a.vel_z,"
        "a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z,");
    if (s->has_sessions_table && s->has_flight_key)
        jw_raw(&sql, "session.flight_key ");
    else
        jw_raw(&sql, "NULL ");
    jw_raw(&sql,
        "FROM aero_observations a "
        "JOIN aero_observation_quality q ON q.observation_id=a.id ");
    if (s->has_sessions_table && s->has_flight_key)
        jw_raw(&sql,
            "LEFT JOIN physics_sessions session "
            "ON session.session_id=a.session_id ");
    jw_raw(&sql, "WHERE ");
    append_context_where(&sql, s, "a");
    jw_raw(&sql, " AND a.session_id<>?");
    if (s->has_sessions_table && s->has_flight_key)
        jw_raw(&sql, " AND (session.flight_key IS NULL OR session.flight_key<>?)");
    jw_raw(&sql, " ORDER BY a.id DESC LIMIT 4096");

    if (sql.failed) {
        jw_free(&sql);
        set_error(error, error_size, "Out of memory building history query");
        return false;
    }

    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(s->db, sql.data, -1, &stmt, NULL) != SQLITE_OK) {
        set_sqlite_error(error, error_size, s->db,
            "Could not prepare physics history query");
        jw_free(&sql);
        return false;
    }
    jw_free(&sql);

    int bind = bind_contexts(stmt, s, 1);
    sqlite3_bind_text(stmt, bind++, s->session_id, -1, SQLITE_STATIC);
    if (s->has_sessions_table && s->has_flight_key)
        sqlite3_bind_text(stmt, bind++, s->flight_key, -1, SQLITE_STATIC);

    size_t capacity = 0;
    HistoryRow *items = NULL;
    int rc;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        long long id = sqlite3_column_int64(stmt, 0);
        const char *session =
            (const char *)sqlite3_column_text(stmt, 1);
        if (!session) continue;

        double q = sqlite3_column_double(stmt, 4);
        double mach = sqlite3_column_double(stmt, 5);
        double aoa = sqlite3_column_double(stmt, 6);
        double beta = sqlite3_column_double(stmt, 7);
        double roll = sqlite3_column_double(stmt, 8);
        double mass = sqlite3_column_double(stmt, 9);
        int airbrakes = sqlite3_column_int(stmt, 12);

        if ((airbrakes != 0 && airbrakes != 1) ||
            !isfinite(q) || q <= 0.0 ||
            !isfinite(mach) || !isfinite(aoa) ||
            !isfinite(beta) || !isfinite(roll) ||
            !isfinite(mass) || mass <= 0.0)
            continue;

        Vector3 vectors[4];
        bool valid = true;
        for (int vector_index = 0; vector_index < 4; ++vector_index) {
            int base = 14 + vector_index * 3;
            vectors[vector_index] = v3(
                sqlite3_column_double(stmt, base),
                sqlite3_column_double(stmt, base + 1),
                sqlite3_column_double(stmt, base + 2));
            if (!finite_vector(vectors[vector_index])) valid = false;
        }
        if (!valid) continue;

        if (*row_count == capacity) {
            size_t next = capacity ? capacity * 2 : 128;
            HistoryRow *grown = realloc(items, next * sizeof(*grown));
            if (!grown) {
                free(items);
                sqlite3_finalize(stmt);
                set_error(error, error_size,
                    "Out of memory reading physics history");
                return false;
            }
            items = grown;
            capacity = next;
        }

        HistoryRow *row = &items[(*row_count)++];
        memset(row, 0, sizeof(*row));
        row->id = id;
        copy_string(row->session_id, sizeof(row->session_id), session);
        row->timeline_epoch = sqlite3_column_int(stmt, 2);
        const char *flight =
            (const char *)sqlite3_column_text(stmt, 26);
        copy_string(row->source, sizeof(row->source),
            flight && *flight ? flight : session);
        row->ut = sqlite3_column_double(stmt, 3);
        row->q = q;
        row->mach = mach;
        row->aoa = aoa;
        row->beta = beta;
        row->roll = roll;
        row->mass = mass;
        row->gear = sqlite3_column_int(stmt, 10);
        row->brakes = sqlite3_column_int(stmt, 11);
        row->airbrakes = airbrakes;
        row->rcs = sqlite3_column_int(stmt, 13);
        row->position = vectors[0];
        row->velocity = vectors[1];
        row->lift = vectors[2];
        row->drag = vectors[3];
    }

    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        free(items);
        *rows = NULL;
        *row_count = 0;
        set_sqlite_error(error, error_size, s->db,
            "Could not read physics history");
        return false;
    }

    *rows = items;
    return true;
}

static int compare_double(const void*a,const void*b){double x=*(const double*)a,y=*(const double*)b;return x<y?-1:x>y?1:0;}
static void normalized_force_magnitudes(
        const HistoryRow *row, double *lift, double *drag) {
    *lift = vmag(row->lift) / row->q;
    *drag = vmag(row->drag) / row->q;
}

static bool numerical_tie(double a, double b) {
    double scale = fmax(1.0, fmax(fabs(a), fabs(b)));
    return fabs(a - b) <= DBL_EPSILON * scale;
}

static size_t choose_medoid(
        const HistoryRow *rows,
        const size_t *indices,
        size_t count,
        bool prefer_newer) {
    if (count == 0) return 0;

    double *lift = malloc(count * sizeof(*lift));
    double *drag = malloc(count * sizeof(*drag));
    if (!lift || !drag) {
        free(lift);
        free(drag);
        return indices[0];
    }

    for (size_t i = 0; i < count; ++i)
        normalized_force_magnitudes(
            &rows[indices[i]], &lift[i], &drag[i]);

    qsort(lift, count, sizeof(*lift), compare_double);
    qsort(drag, count, sizeof(*drag), compare_double);
    size_t middle = count / 2;
    double median_lift = count % 2
        ? lift[middle]
        : 0.5 * (lift[middle - 1] + lift[middle]);
    double median_drag = count % 2
        ? drag[middle]
        : 0.5 * (drag[middle - 1] + drag[middle]);

    size_t best = indices[0];
    double best_score = INFINITY;
    for (size_t i = 0; i < count; ++i) {
        size_t index = indices[i];
        double row_lift, row_drag;
        normalized_force_magnitudes(
            &rows[index], &row_lift, &row_drag);
        /*
         * Lift/q and drag/q have identical dimensions. Their Euclidean
         * distance from the component-wise median is therefore a direct robust
         * force-observation medoid metric, with no behavioral quality weight.
         */
        double score =
            hypot(row_lift - median_lift, row_drag - median_drag);
        bool tie = numerical_tie(score, best_score);
        if (score < best_score ||
            (tie && (prefer_newer
                ? rows[index].id > rows[best].id
                : rows[index].id < rows[best].id))) {
            best = index;
            best_score = score;
        }
    }

    free(lift);
    free(drag);
    return best;
}

static bool build_source_candidates(
        const HistoryRow *rows,
        size_t count,
        SourceCandidate **out,
        size_t *out_count,
        char *error,
        size_t error_size) {
    *out = NULL;
    *out_count = 0;
    if (count == 0) return true;

    bool *used = calloc(count, sizeof(*used));
    SourceCandidate *candidates = calloc(count, sizeof(*candidates));
    size_t *indices = malloc(count * sizeof(*indices));
    if (!used || !candidates || !indices) {
        free(used);
        free(candidates);
        free(indices);
        set_error(error, error_size,
            "Out of memory reducing physics history");
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        if (used[i]) continue;
        CellKey key = row_cell(&rows[i]);
        size_t n = 0;
        for (size_t j = i; j < count; ++j) {
            if (!used[j] &&
                cell_equal(key, row_cell(&rows[j])) &&
                strcmp(rows[i].source, rows[j].source) == 0) {
                used[j] = true;
                indices[n++] = j;
            }
        }

        size_t chosen = choose_medoid(rows, indices, n, false);
        SourceCandidate *candidate = &candidates[(*out_count)++];
        candidate->key = key;
        candidate->row = rows[chosen];
        copy_string(candidate->source, sizeof(candidate->source),
            rows[i].source);
    }

    free(used);
    free(indices);
    *out = candidates;
    return true;
}

static bool build_selected_cells(
        const SourceCandidate *candidates,
        size_t count,
        SelectedCell **out,
        size_t *out_count,
        char *error,
        size_t error_size) {
    *out = NULL;
    *out_count = 0;
    if (count == 0) return true;

    bool *used = calloc(count, sizeof(*used));
    SelectedCell *selected = calloc(count, sizeof(*selected));
    size_t *group = malloc(count * sizeof(*group));
    HistoryRow *group_rows = malloc(count * sizeof(*group_rows));
    size_t *local = malloc(count * sizeof(*local));
    if (!used || !selected || !group || !group_rows || !local) {
        free(used);
        free(selected);
        free(group);
        free(group_rows);
        free(local);
        set_error(error, error_size,
            "Out of memory aggregating independent flights");
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        if (used[i]) continue;
        size_t n = 0;
        for (size_t j = i; j < count; ++j) {
            if (!used[j] &&
                cell_equal(candidates[i].key, candidates[j].key)) {
                used[j] = true;
                group[n++] = j;
            }
        }

        for (size_t j = 0; j < n; ++j) {
            group_rows[j] = candidates[group[j]].row;
            local[j] = j;
        }
        size_t chosen_local =
            choose_medoid(group_rows, local, n, true);
        SelectedCell *cell = &selected[(*out_count)++];
        cell->key = candidates[i].key;
        cell->row = group_rows[chosen_local];
        cell->independent = (unsigned)n;
    }

    free(used);
    free(group);
    free(group_rows);
    free(local);
    *out = selected;
    return true;
}

/*
 * These boundaries are the midpoints of the predictor's Mach anchors
 * {0.35, 1.05, 2.6, 6.0}; they classify stored coverage into the same
 * interpolation regions rather than encoding separate flight behavior.
 */
static int certified_regime_for_store(double mach) {
    static const double anchors[] = {0.35, 1.05, 2.6, 6.0};
    for (int i = 0; i < 3; ++i)
        if (mach < 0.5 * (anchors[i] + anchors[i + 1])) return i;
    return 3;
}

static double cell_distance(CellKey a, CellKey b) {
    if (a.gear != b.gear || a.brakes != b.brakes ||
        a.airbrakes != b.airbrakes)
        return INFINITY;

    /*
     * Cell coordinates are already dimensionless model-space indices.
     * AoA/beta cells are 4 deg wide; dividing their index separation by
     * 2.5 expresses the existing 10 deg aerodynamic lookup neighborhood.
     */
    double dq = (double)(a.q - b.q);
    double dm = (double)(a.mach - b.mach);
    double da = (double)(a.aoa - b.aoa) / 2.5;
    double db = (double)(a.beta - b.beta) / 2.5;
    return sqrt(dq * dq + dm * dm + da * da + db * db);
}

static bool better_support(
        const SelectedCell *candidate,
        const SelectedCell *current) {
    if (candidate->independent != current->independent)
        return candidate->independent > current->independent;
    return candidate->row.id > current->row.id;
}

static size_t reduce_selected_cells(
        const SelectedCell *selected,
        size_t count,
        size_t maximum,
        size_t *indices) {
    if (count <= maximum) {
        for (size_t i = 0; i < count; ++i) indices[i] = i;
        return count;
    }

    bool *retained = calloc(count, sizeof(*retained));
    if (!retained) return 0;
    size_t n = 0;

    /*
     * First preserve every represented device/Mach-region topology. Within a
     * topology, prefer the state supported by more independent flights.
     */
    for (size_t i = 0; i < count && n < maximum; ++i) {
        int regime = certified_regime_for_store(selected[i].row.mach);
        bool group_seen = false;
        for (size_t j = 0; j < n; ++j) {
            const SelectedCell *existing = &selected[indices[j]];
            if (existing->key.gear == selected[i].key.gear &&
                existing->key.brakes == selected[i].key.brakes &&
                existing->key.airbrakes == selected[i].key.airbrakes &&
                certified_regime_for_store(existing->row.mach) == regime) {
                group_seen = true;
                break;
            }
        }
        if (group_seen) continue;

        size_t best = i;
        for (size_t j = i + 1; j < count; ++j) {
            if (selected[j].key.gear != selected[i].key.gear ||
                selected[j].key.brakes != selected[i].key.brakes ||
                selected[j].key.airbrakes != selected[i].key.airbrakes ||
                certified_regime_for_store(selected[j].row.mach) != regime)
                continue;
            if (better_support(&selected[j], &selected[best]))
                best = j;
        }

        if (!retained[best]) {
            retained[best] = true;
            indices[n++] = best;
        }
    }

    /*
     * Fill the remaining budget by farthest-point sampling in the model
     * coordinates. This maximizes retained coverage directly; support count
     * and recency only break numerical ties.
     */
    while (n < maximum) {
        size_t best = SIZE_MAX;
        double best_distance = -INFINITY;

        for (size_t i = 0; i < count; ++i) {
            if (retained[i]) continue;
            double nearest = INFINITY;
            for (size_t j = 0; j < n; ++j)
                nearest = fmin(nearest,
                    cell_distance(selected[i].key,
                        selected[indices[j]].key));

            bool tie = best != SIZE_MAX &&
                numerical_tie(nearest, best_distance);
            if (best == SIZE_MAX ||
                nearest > best_distance ||
                (tie && better_support(&selected[i], &selected[best]))) {
                best = i;
                best_distance = nearest;
            }
        }

        if (best == SIZE_MAX) break;
        retained[best] = true;
        indices[n++] = best;
    }

    free(retained);
    return n;
}

static Vector3 raw_krpc_to_canonical(Vector3 v){return v3(v.x,v.z,v.y);}
static bool selected_to_sample(const SelectedCell*selected,const PlanetModel*planet,VesselAeroSample*out){
    const HistoryRow*r=&selected->row;Vector3 pos=raw_krpc_to_canonical(r->position),vel=raw_krpc_to_canonical(r->velocity),lift_force=raw_krpc_to_canonical(r->lift),drag_force=raw_krpc_to_canonical(r->drag);
    Vector3 omega=vscale(planet->north_axis,planet->rotational_speed);Vector3 air=vsub(vel,vcross(omega,pos));if(vmag(air)<=1)return false;
    Vector3 dir=vnorm(air,v3(1,0,0)),up=vnorm(vproject_plane(pos,dir),v3(0,0,1)),side=vnorm(vcross(dir,up),v3(0,1,0));double bank=r->roll*DEG2RAD;
    Vector3 lift_axis=vadd(vscale(up,cos(bank)),vscale(side,sin(bank))),lateral=vsub(vscale(side,cos(bank)),vscale(up,sin(bank))),force=vadd(lift_force,drag_force);
    VesselAeroSample sample={
        .q=r->q,.mach=r->mach,.aoa=r->aoa,.beta=r->beta,.mass=r->mass,
        .force_per_q=vscale(
            v3(-vdot(force,dir),vdot(force,lift_axis),vdot(force,lateral)),
            1.0/r->q),
        .gear=r->gear!=0,.brakes=r->brakes!=0,
        .airbrakes=r->airbrakes,
        .observations=selected->independent,
        /*
         * Trust now means provenance-valid direct measurement. Statistical
         * support remains explicit in observations instead of being hidden in
         * a hand-tuned trust transform.
         */
        .trust=1.0,
        .last_observation_ut=isfinite(r->ut)?r->ut:-INFINITY};
    if(!finite_vector(sample.force_per_q)||sample.force_per_q.x<0)return false;*out=sample;return true;
}

size_t physics_store_load_history(PhysicsStore*store,const PlanetModel*planet,VesselAeroSample*out_samples,size_t maximum,char*error,size_t error_size){
    if(error&&error_size)error[0]=0;if(!store||!planet||!out_samples||maximum==0){if(!store||!planet||!out_samples)set_error(error,error_size,"Physics history requires store, planet and output buffer");return 0;}
    if(maximum>VESSEL_AERO_SAMPLES)maximum=VESSEL_AERO_SAMPLES;HistoryRow*rows=NULL;size_t row_count=0;
    if(!history_query(store,&rows,&row_count,error,error_size))return 0;
    SourceCandidate*candidates=NULL;size_t candidate_count=0;if(!build_source_candidates(rows,row_count,&candidates,&candidate_count,error,error_size)){free(rows);return 0;}
    SelectedCell*selected=NULL;size_t selected_count=0;if(!build_selected_cells(candidates,candidate_count,&selected,&selected_count,error,error_size)){free(candidates);free(rows);return 0;}
    size_t*indices=selected_count?malloc(selected_count*sizeof(*indices)):NULL;if(selected_count&&!indices){set_error(error,error_size,"Out of memory bounding physics history");free(selected);free(candidates);free(rows);return 0;}
    size_t chosen=selected_count?reduce_selected_cells(selected,selected_count,maximum,indices):0;if(selected_count&&chosen==0){set_error(error,error_size,"Out of memory reducing physics history coverage");free(indices);free(selected);free(candidates);free(rows);return 0;}
    size_t output=0;for(size_t i=0;i<chosen&&output<maximum;i++){VesselAeroSample sample;if(selected_to_sample(&selected[indices[i]],planet,&sample))out_samples[output++]=sample;}
    free(indices);free(selected);free(candidates);free(rows);return output;
}

static bool situation_excluded(const char*situation){
    if(!situation)return false;char lowered[32];size_t n=strlen(situation);if(n>=sizeof(lowered))n=sizeof(lowered)-1;for(size_t i=0;i<n;i++){char c=(char)tolower((unsigned char)situation[i]);lowered[i]=c=='_'?'-':c;}lowered[n]=0;
    return strcmp(lowered,"landed")==0||strcmp(lowered,"splashed")==0||strcmp(lowered,"pre-launch")==0;
}

bool physics_store_observe(PhysicsStore*store,const PhysicsStoreObservation*o,char*error,size_t error_size){
    if(error&&error_size)error[0]=0;if(!store||!o){set_error(error,error_size,"Physics observation requires store and sample");return false;}
    if(store->mode!=PHYSICS_STORE_READ_WRITE){set_error(error,error_size,"Physics store is read-only");return false;}
    if(!o->sample_valid||!o->body_non_rotating||situation_excluded(o->situation))return true;
    if(!isfinite(o->ut)||!isfinite(o->q)||o->q<=0.0||!isfinite(o->mach)||!isfinite(o->aoa)||!isfinite(o->beta)||!isfinite(o->roll)||!isfinite(o->mass)||o->mass<=0||
       !finite_vector(o->position)||!finite_vector(o->velocity)||!finite_vector(o->lift)||!finite_vector(o->drag))return true;
    if(store->has_last_ut&&o->ut<store->last_ut){store->timeline_epoch++;store->has_last_saved_ut=false;}store->last_ut=o->ut;store->has_last_ut=true;
    if(store->has_last_saved_ut&&o->ut-store->last_saved_ut<archive_sample_period_s)return true;store->last_saved_ut=o->ut;store->has_last_saved_ut=true;
    if(store->pending==0&&sqlite3_exec(store->db,"BEGIN",NULL,NULL,NULL)!=SQLITE_OK){set_sqlite_error(error,error_size,store->db,"Could not begin physics observation batch");return false;}
    const char*sql="INSERT INTO aero_observations(structure_id,environment_id,session_id,timeline_epoch,ut,wall_time,q,mach,aoa,beta,roll,mass,dry_mass,gear,brakes,airbrakes,rcs,com_root_x,com_root_y,com_root_z,inertia_x,inertia_y,inertia_z,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,lift_x,lift_y,lift_z,drag_x,drag_y,drag_z) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)";
    sqlite3_stmt*stmt=NULL;if(sqlite3_prepare_v2(store->db,sql,-1,&stmt,NULL)!=SQLITE_OK){sqlite3_exec(store->db,"ROLLBACK",NULL,NULL,NULL);store->pending=0;set_sqlite_error(error,error_size,store->db,"Could not prepare physics observation");return false;}
    int b=1;sqlite3_bind_text(stmt,b++,store->structure_id,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,b++,store->environment_id,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,b++,store->session_id,-1,SQLITE_STATIC);sqlite3_bind_int(stmt,b++,store->timeline_epoch);
    sqlite3_bind_double(stmt,b++,o->ut);sqlite3_bind_double(stmt,b++,isfinite(o->wall_time)?o->wall_time:realtime_seconds());sqlite3_bind_double(stmt,b++,o->q);sqlite3_bind_double(stmt,b++,o->mach);sqlite3_bind_double(stmt,b++,o->aoa);sqlite3_bind_double(stmt,b++,o->beta);sqlite3_bind_double(stmt,b++,o->roll);sqlite3_bind_double(stmt,b++,o->mass);
    if(o->has_dry_mass&&isfinite(o->dry_mass))sqlite3_bind_double(stmt,b++,o->dry_mass);else sqlite3_bind_null(stmt,b++);sqlite3_bind_int(stmt,b++,o->gear);sqlite3_bind_int(stmt,b++,o->brakes);sqlite3_bind_int(stmt,b++,o->airbrakes);sqlite3_bind_int(stmt,b++,o->rcs);
#define BIND_OPTIONAL_VECTOR(HAS,V) do{if((HAS)&&finite_vector((V))){sqlite3_bind_double(stmt,b++,(V).x);sqlite3_bind_double(stmt,b++,(V).y);sqlite3_bind_double(stmt,b++,(V).z);}else{sqlite3_bind_null(stmt,b++);sqlite3_bind_null(stmt,b++);sqlite3_bind_null(stmt,b++);}}while(0)
    BIND_OPTIONAL_VECTOR(o->has_center_of_mass_root,o->center_of_mass_root);BIND_OPTIONAL_VECTOR(o->has_moment_of_inertia,o->moment_of_inertia);
#undef BIND_OPTIONAL_VECTOR
#define BIND_VECTOR(V) do{sqlite3_bind_double(stmt,b++,(V).x);sqlite3_bind_double(stmt,b++,(V).y);sqlite3_bind_double(stmt,b++,(V).z);}while(0)
    BIND_VECTOR(o->position);BIND_VECTOR(o->velocity);BIND_VECTOR(o->lift);BIND_VECTOR(o->drag);
#undef BIND_VECTOR
    bool ok=sqlite3_step(stmt)==SQLITE_DONE;long long observation_id=sqlite3_last_insert_rowid(store->db);sqlite3_finalize(stmt);if(!ok){sqlite3_exec(store->db,"ROLLBACK",NULL,NULL,NULL);store->pending=0;set_sqlite_error(error,error_size,store->db,"Could not insert physics observation");return false;}
    /*
     * This companion row marks modern validated-observation provenance. Profile
     * and body rates are retained for diagnostics, but quality/eligible no
     * longer encode attitude-dependent acceptance policy.
     */
    const char*qsql="INSERT OR REPLACE INTO aero_observation_quality(observation_id,control_profile,body_pitch_rate,body_roll_rate,body_yaw_rate,quality,eligible) VALUES(?,?,?,?,?,?,?)";
    if(sqlite3_prepare_v2(store->db,qsql,-1,&stmt,NULL)!=SQLITE_OK){sqlite3_exec(store->db,"ROLLBACK",NULL,NULL,NULL);store->pending=0;set_sqlite_error(error,error_size,store->db,"Could not prepare observation quality");return false;}
    sqlite3_bind_int64(stmt,1,observation_id);sqlite3_bind_text(stmt,2,o->control_profile?o->control_profile:"",-1,SQLITE_STATIC);
    if(o->has_body_rates&&finite_vector(o->body_rates)){sqlite3_bind_double(stmt,3,o->body_rates.x);sqlite3_bind_double(stmt,4,o->body_rates.y);sqlite3_bind_double(stmt,5,o->body_rates.z);}else{sqlite3_bind_null(stmt,3);sqlite3_bind_null(stmt,4);sqlite3_bind_null(stmt,5);}
    sqlite3_bind_double(stmt,6,1.0);sqlite3_bind_int(stmt,7,1);ok=sqlite3_step(stmt)==SQLITE_DONE;sqlite3_finalize(stmt);if(!ok){sqlite3_exec(store->db,"ROLLBACK",NULL,NULL,NULL);store->pending=0;set_sqlite_error(error,error_size,store->db,"Could not insert observation provenance");return false;}
    store->pending++;if(store->pending>=STORE_COMMIT_BATCH){if(sqlite3_exec(store->db,"COMMIT",NULL,NULL,NULL)!=SQLITE_OK){sqlite3_exec(store->db,"ROLLBACK",NULL,NULL,NULL);store->pending=0;set_sqlite_error(error,error_size,store->db,"Could not commit physics observations");return false;}store->pending=0;}return true;
}
