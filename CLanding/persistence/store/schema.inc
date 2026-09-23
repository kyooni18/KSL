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

