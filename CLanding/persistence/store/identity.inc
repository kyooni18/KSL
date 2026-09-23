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

