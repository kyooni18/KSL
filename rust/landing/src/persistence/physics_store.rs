use crate::sys::*;
extern "C" {
    fn json_parse(
        text: *const ::core::ffi::c_char,
        tokens: *mut JsonToken,
        capacity: ::core::ffi::c_int,
        doc: *mut JsonDoc,
    ) -> ::core::ffi::c_int;
    fn json_object_get(
        doc: *const JsonDoc,
        object_index: ::core::ffi::c_int,
        key: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    fn json_array_get(
        doc: *const JsonDoc,
        array_index: ::core::ffi::c_int,
        element_index: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    fn json_token_eq(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        value: *const ::core::ffi::c_char,
    ) -> bool;
    fn json_number(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        fallback: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn json_boolean(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        fallback: bool,
    ) -> bool;
    fn json_string(
        doc: *const JsonDoc,
        index: ::core::ffi::c_int,
        out: *mut ::core::ffi::c_char,
        out_size: size_t,
    ) -> bool;
    fn json_skip(doc: *const JsonDoc, index: ::core::ffi::c_int) -> ::core::ffi::c_int;
    fn jw_init(w: *mut JsonWriter);
    fn jw_free(w: *mut JsonWriter);
    fn jw_raw(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_char(w: *mut JsonWriter, c: ::core::ffi::c_char);
    fn jw_string(w: *mut JsonWriter, text: *const ::core::ffi::c_char);
    fn jw_number(w: *mut JsonWriter, value: ::core::ffi::c_double);
    fn jw_integer(w: *mut JsonWriter, value: ::core::ffi::c_longlong);
    fn jw_bool(w: *mut JsonWriter, value: bool);
    fn jw_null(w: *mut JsonWriter);
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn lrint(__x: ::core::ffi::c_double) -> ::core::ffi::c_long;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fmin(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vector3;
    fn vadd(a: Vector3, b: Vector3) -> Vector3;
    fn vsub(a: Vector3, b: Vector3) -> Vector3;
    fn vscale(a: Vector3, s: ::core::ffi::c_double) -> Vector3;
    fn vdot(a: Vector3, b: Vector3) -> ::core::ffi::c_double;
    fn vcross(a: Vector3, b: Vector3) -> Vector3;
    fn vmag(a: Vector3) -> ::core::ffi::c_double;
    fn vnorm(a: Vector3, fallback: Vector3) -> Vector3;
    fn vproject_plane(a: Vector3, normal: Vector3) -> Vector3;
    fn sqlite3_close(_: *mut sqlite3) -> ::core::ffi::c_int;
    fn sqlite3_exec(
        _: *mut sqlite3,
        sql: *const ::core::ffi::c_char,
        callback: Option<
            unsafe extern "C" fn(
                *mut ::core::ffi::c_void,
                ::core::ffi::c_int,
                *mut *mut ::core::ffi::c_char,
                *mut *mut ::core::ffi::c_char,
            ) -> ::core::ffi::c_int,
        >,
        _: *mut ::core::ffi::c_void,
        errmsg: *mut *mut ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    fn sqlite3_last_insert_rowid(_: *mut sqlite3) -> sqlite3_int64;
    fn sqlite3_busy_timeout(
        _: *mut sqlite3,
        ms: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    fn sqlite3_free(_: *mut ::core::ffi::c_void);
    fn sqlite3_open_v2(
        filename: *const ::core::ffi::c_char,
        ppDb: *mut *mut sqlite3,
        flags: ::core::ffi::c_int,
        zVfs: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    fn sqlite3_errmsg(_: *mut sqlite3) -> *const ::core::ffi::c_char;
    fn sqlite3_prepare_v2(
        db: *mut sqlite3,
        zSql: *const ::core::ffi::c_char,
        nByte: ::core::ffi::c_int,
        ppStmt: *mut *mut sqlite3_stmt,
        pzTail: *mut *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    fn sqlite3_bind_double(
        _: *mut sqlite3_stmt,
        _: ::core::ffi::c_int,
        _: ::core::ffi::c_double,
    ) -> ::core::ffi::c_int;
    fn sqlite3_bind_int(
        _: *mut sqlite3_stmt,
        _: ::core::ffi::c_int,
        _: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    fn sqlite3_bind_int64(
        _: *mut sqlite3_stmt,
        _: ::core::ffi::c_int,
        _: sqlite3_int64,
    ) -> ::core::ffi::c_int;
    fn sqlite3_bind_null(
        _: *mut sqlite3_stmt,
        _: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    fn sqlite3_bind_text(
        _: *mut sqlite3_stmt,
        _: ::core::ffi::c_int,
        _: *const ::core::ffi::c_char,
        _: ::core::ffi::c_int,
        _: Option<unsafe extern "C" fn(*mut ::core::ffi::c_void) -> ()>,
    ) -> ::core::ffi::c_int;
    fn sqlite3_step(_: *mut sqlite3_stmt) -> ::core::ffi::c_int;
    fn sqlite3_column_double(
        _: *mut sqlite3_stmt,
        iCol: ::core::ffi::c_int,
    ) -> ::core::ffi::c_double;
    fn sqlite3_column_int(
        _: *mut sqlite3_stmt,
        iCol: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    fn sqlite3_column_int64(
        _: *mut sqlite3_stmt,
        iCol: ::core::ffi::c_int,
    ) -> sqlite3_int64;
    fn sqlite3_column_text(
        _: *mut sqlite3_stmt,
        iCol: ::core::ffi::c_int,
    ) -> *const ::core::ffi::c_uchar;
    fn sqlite3_finalize(pStmt: *mut sqlite3_stmt) -> ::core::ffi::c_int;
    fn sqlite3_get_autocommit(_: *mut sqlite3) -> ::core::ffi::c_int;
}
pub type size_t = usize;
pub type JsonType = ::core::ffi::c_uint;
pub const JSON_PRIMITIVE: JsonType = 4;
pub const JSON_STRING: JsonType = 3;
pub const JSON_ARRAY: JsonType = 2;
pub const JSON_OBJECT: JsonType = 1;
pub const JSON_UNDEFINED: JsonType = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonToken {
    pub type_0: JsonType,
    pub start: ::core::ffi::c_int,
    pub end: ::core::ffi::c_int,
    pub size: ::core::ffi::c_int,
    pub parent: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonDoc {
    pub text: *const ::core::ffi::c_char,
    pub tokens: *mut JsonToken,
    pub count: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonWriter {
    pub data: *mut ::core::ffi::c_char,
    pub length: size_t,
    pub capacity: size_t,
    pub failed: bool,
}
pub type __int32_t = i32;
pub type __uint32_t = u32;
pub type __uint64_t = u64;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PlanetModel {
    pub name: [::core::ffi::c_char; 96],
    pub radius: ::core::ffi::c_double,
    pub gravitational_parameter: ::core::ffi::c_double,
    pub rotational_speed: ::core::ffi::c_double,
    pub atmosphere_depth: ::core::ffi::c_double,
    pub surface_density: ::core::ffi::c_double,
    pub atmosphere_adiabatic_index: ::core::ffi::c_double,
    pub atmosphere_sample_count: size_t,
    pub atmosphere_altitude: [::core::ffi::c_double; 1024],
    pub atmosphere_pressure: [::core::ffi::c_double; 1024],
    pub atmosphere_density: [::core::ffi::c_double; 1024],
    pub north_axis: Vector3,
    pub prime_meridian_at_epoch: Vector3,
    pub epoch_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VesselAeroSample {
    pub q: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub beta: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub force_per_q: Vector3,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub observations: ::core::ffi::c_uint,
    pub trust: ::core::ffi::c_double,
    pub last_observation_ut: ::core::ffi::c_double,
}
pub type PhysicsStoreMode = ::core::ffi::c_uint;
pub const PHYSICS_STORE_READ_WRITE: PhysicsStoreMode = 1;
pub const PHYSICS_STORE_READ_ONLY: PhysicsStoreMode = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PhysicsStoreOptions {
    pub path: *const ::core::ffi::c_char,
    pub model_id: *const ::core::ffi::c_char,
    pub vessel_name: *const ::core::ffi::c_char,
    pub structure_manifest_json: *const ::core::ffi::c_char,
    pub environment_manifest_json: *const ::core::ffi::c_char,
    pub session_id: *const ::core::ffi::c_char,
    pub flight_key: *const ::core::ffi::c_char,
    pub structure_variant: *const ::core::ffi::c_char,
    pub mode: PhysicsStoreMode,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PhysicsStoreObservation {
    pub sample_valid: bool,
    pub body_non_rotating: bool,
    pub situation: *const ::core::ffi::c_char,
    pub ut: ::core::ffi::c_double,
    pub wall_time: ::core::ffi::c_double,
    pub q: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub beta: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub has_dry_mass: bool,
    pub dry_mass: ::core::ffi::c_double,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub rcs: ::core::ffi::c_int,
    pub has_center_of_mass_root: bool,
    pub center_of_mass_root: Vector3,
    pub has_moment_of_inertia: bool,
    pub moment_of_inertia: Vector3,
    pub position: Vector3,
    pub velocity: Vector3,
    pub lift: Vector3,
    pub drag: Vector3,
    pub control_profile: *const ::core::ffi::c_char,
    pub has_body_rates: bool,
    pub body_rates: Vector3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PhysicsStore {
    pub db: *mut sqlite3,
    pub mode: PhysicsStoreMode,
    pub path: [::core::ffi::c_char; 1024],
    pub structure_id: [::core::ffi::c_char; 128],
    pub structure_witness_id: [::core::ffi::c_char; 128],
    pub environment_id: [::core::ffi::c_char; 128],
    pub session_id: [::core::ffi::c_char; 128],
    pub flight_key: [::core::ffi::c_char; 128],
    pub vessel_name: [::core::ffi::c_char; 128],
    pub structure_manifest_json: *mut ::core::ffi::c_char,
    pub environment_manifest_json: *mut ::core::ffi::c_char,
    pub environment_identity_json: *mut ::core::ffi::c_char,
    pub contexts: *mut ContextPair,
    pub context_count: size_t,
    pub context_capacity: size_t,
    pub has_contexts_table: bool,
    pub has_sessions_table: bool,
    pub has_flight_key: bool,
    pub has_observations_table: bool,
    pub has_quality_table: bool,
    pub timeline_epoch: ::core::ffi::c_int,
    pub has_last_ut: bool,
    pub last_ut: ::core::ffi::c_double,
    pub has_last_saved_ut: bool,
    pub last_saved_ut: ::core::ffi::c_double,
    pub pending: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ContextPair {
    pub structure_id: [::core::ffi::c_char; 128],
    pub environment_id: [::core::ffi::c_char; 128],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ParsedJson {
    pub tokens: *mut JsonToken,
    pub doc: JsonDoc,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonPair {
    pub key: *mut ::core::ffi::c_char,
    pub value_index: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct StructureIdentity {
    pub part_names: *mut *mut ::core::ffi::c_char,
    pub part_count: size_t,
    pub geometry: *mut StructureGeometry,
    pub geometry_count: size_t,
    pub geometry_available: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct StructureGeometry {
    pub name: *mut ::core::ffi::c_char,
    pub x: ::core::ffi::c_long,
    pub y: ::core::ffi::c_long,
    pub z: ::core::ffi::c_long,
}
pub type sqlite3_destructor_type = Option<
    unsafe extern "C" fn(*mut ::core::ffi::c_void) -> (),
>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Sha256 {
    pub h: [uint32_t; 8],
    pub bits: uint64_t,
    pub block: [::core::ffi::c_uchar; 64],
    pub used: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HistoryRow {
    pub id: ::core::ffi::c_longlong,
    pub session_id: [::core::ffi::c_char; 128],
    pub timeline_epoch: ::core::ffi::c_int,
    pub source: [::core::ffi::c_char; 128],
    pub ut: ::core::ffi::c_double,
    pub q: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub beta: ::core::ffi::c_double,
    pub roll: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub gear: ::core::ffi::c_int,
    pub brakes: ::core::ffi::c_int,
    pub airbrakes: ::core::ffi::c_int,
    pub rcs: ::core::ffi::c_int,
    pub position: Vector3,
    pub velocity: Vector3,
    pub lift: Vector3,
    pub drag: Vector3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SourceCandidate {
    pub key: CellKey,
    pub row: HistoryRow,
    pub source: [::core::ffi::c_char; 128],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CellKey {
    pub q: ::core::ffi::c_int,
    pub mach: ::core::ffi::c_int,
    pub aoa: ::core::ffi::c_int,
    pub beta: ::core::ffi::c_int,
    pub gear: ::core::ffi::c_int,
    pub brakes: ::core::ffi::c_int,
    pub airbrakes: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SelectedCell {
    pub key: CellKey,
    pub row: HistoryRow,
    pub independent: ::core::ffi::c_uint,
}
pub type sqlite3_int64 = sqlite_int64;
pub type sqlite_int64 = ::core::ffi::c_longlong;
pub const SIZE_MAX: ::core::ffi::c_ulong = 18446744073709551615 as ::core::ffi::c_ulong;
pub const LANDER_PI: ::core::ffi::c_double = 3.14159265358979323846264338327950288f64;
pub const DEG2RAD: ::core::ffi::c_double = LANDER_PI / 180.0f64;
pub const PHYSICS_STORE_ID_CAPACITY: ::core::ffi::c_int = 128 as ::core::ffi::c_int;
pub const STORE_COMMIT_BATCH: ::core::ffi::c_int = 4 as ::core::ffi::c_int;
static mut archive_sample_period_s: ::core::ffi::c_double = 0.5f64;
pub const SQLITE_OK: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const SQLITE_ROW: ::core::ffi::c_int = 100 as ::core::ffi::c_int;
pub const SQLITE_DONE: ::core::ffi::c_int = 101 as ::core::ffi::c_int;
pub const SQLITE_OPEN_READONLY: ::core::ffi::c_int = 0x1 as ::core::ffi::c_int;
pub const SQLITE_OPEN_READWRITE: ::core::ffi::c_int = 0x2 as ::core::ffi::c_int;
pub const SQLITE_OPEN_CREATE: ::core::ffi::c_int = 0x4 as ::core::ffi::c_int;
pub const SQLITE_STATIC: sqlite3_destructor_type = None;
pub const VESSEL_AERO_SAMPLES: ::core::ffi::c_int = 192 as ::core::ffi::c_int;
unsafe extern "C" fn table_exists(
    mut db: *mut sqlite3,
    mut table: *const ::core::ffi::c_char,
) -> bool {
    let mut stmt: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    let mut exists: bool = false_0 != 0;
    if sqlite3_prepare_v2(
        db,
        b"SELECT 1 FROM sqlite_master WHERE type='table' AND name=? LIMIT 1\0"
            as *const u8 as *const ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) == SQLITE_OK
    {
        sqlite3_bind_text(
            stmt,
            1 as ::core::ffi::c_int,
            table,
            -(1 as ::core::ffi::c_int),
            SQLITE_STATIC,
        );
        exists = sqlite3_step(stmt) == SQLITE_ROW;
    }
    sqlite3_finalize(stmt);
    return exists;
}
unsafe extern "C" fn column_exists(
    mut db: *mut sqlite3,
    mut table: *const ::core::ffi::c_char,
    mut column: *const ::core::ffi::c_char,
) -> bool {
    let mut sql: [::core::ffi::c_char; 256] = [0; 256];
    snprintf(
        &raw mut sql as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 256]>() as size_t,
        b"PRAGMA table_info(%s)\0" as *const u8 as *const ::core::ffi::c_char,
        table,
    );
    let mut stmt: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    let mut exists: bool = false_0 != 0;
    if sqlite3_prepare_v2(
        db,
        &raw mut sql as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) == SQLITE_OK
    {
        while sqlite3_step(stmt) == SQLITE_ROW {
            let mut name: *const ::core::ffi::c_char = sqlite3_column_text(
                stmt,
                1 as ::core::ffi::c_int,
            ) as *const ::core::ffi::c_char;
            if !(!name.is_null() && strcmp(name, column) == 0 as ::core::ffi::c_int) {
                continue;
            }
            exists = true_0 != 0;
            break;
        }
    }
    sqlite3_finalize(stmt);
    return exists;
}
unsafe extern "C" fn execute_sql(
    mut db: *mut sqlite3,
    mut sql: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut message: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    let mut rc: ::core::ffi::c_int = sqlite3_exec(db, sql, None, NULL, &raw mut message);
    if rc == SQLITE_OK {
        return true_0 != 0;
    }
    if !error.is_null() && error_size != 0 {
        snprintf(
            error,
            error_size,
            b"SQLite migration failed: %s\0" as *const u8 as *const ::core::ffi::c_char,
            if !message.is_null() {
                message as *const ::core::ffi::c_char
            } else {
                sqlite3_errmsg(db)
            },
        );
    }
    sqlite3_free(message as *mut ::core::ffi::c_void);
    return false_0 != 0;
}
unsafe extern "C" fn ensure_column(
    mut db: *mut sqlite3,
    mut table: *const ::core::ffi::c_char,
    mut column: *const ::core::ffi::c_char,
    mut declaration: *const ::core::ffi::c_char,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if column_exists(db, table, column) {
        return true_0 != 0;
    }
    let mut sql: [::core::ffi::c_char; 512] = [0; 512];
    snprintf(
        &raw mut sql as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as size_t,
        b"ALTER TABLE %s ADD COLUMN %s %s\0" as *const u8 as *const ::core::ffi::c_char,
        table,
        column,
        declaration,
    );
    return execute_sql(db, &raw mut sql as *mut ::core::ffi::c_char, error, error_size);
}
unsafe extern "C" fn refresh_schema_flags(mut store: *mut PhysicsStore) {
    (*store).has_contexts_table = table_exists(
        (*store).db,
        b"physics_contexts\0" as *const u8 as *const ::core::ffi::c_char,
    );
    (*store).has_sessions_table = table_exists(
        (*store).db,
        b"physics_sessions\0" as *const u8 as *const ::core::ffi::c_char,
    );
    (*store).has_flight_key = (*store).has_sessions_table as ::core::ffi::c_int != 0
        && column_exists(
            (*store).db,
            b"physics_sessions\0" as *const u8 as *const ::core::ffi::c_char,
            b"flight_key\0" as *const u8 as *const ::core::ffi::c_char,
        ) as ::core::ffi::c_int != 0;
    (*store).has_observations_table = table_exists(
        (*store).db,
        b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
    );
    (*store).has_quality_table = table_exists(
        (*store).db,
        b"aero_observation_quality\0" as *const u8 as *const ::core::ffi::c_char,
    );
}
unsafe extern "C" fn migrate_schema(
    mut store: *mut PhysicsStore,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    static mut schema_sql: *const ::core::ffi::c_char = b"BEGIN IMMEDIATE;CREATE TABLE IF NOT EXISTS physics_meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);CREATE TABLE IF NOT EXISTS physics_contexts( structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,structure_json TEXT NOT NULL, environment_json TEXT NOT NULL,first_seen REAL NOT NULL,last_seen REAL NOT NULL, PRIMARY KEY(structure_id,environment_id));CREATE TABLE IF NOT EXISTS physics_sessions( session_id TEXT PRIMARY KEY,structure_id TEXT NOT NULL,environment_id TEXT NOT NULL, vessel_name TEXT NOT NULL,started_wall REAL NOT NULL,flight_key TEXT);CREATE TABLE IF NOT EXISTS aero_observations( id INTEGER PRIMARY KEY AUTOINCREMENT,structure_id TEXT NOT NULL,environment_id TEXT NOT NULL, session_id TEXT NOT NULL,timeline_epoch INTEGER NOT NULL,ut REAL NOT NULL,wall_time REAL NOT NULL, q REAL NOT NULL,mach REAL NOT NULL,aoa REAL NOT NULL,beta REAL NOT NULL,roll REAL NOT NULL,mass REAL NOT NULL, dry_mass REAL,gear INTEGER NOT NULL,brakes INTEGER NOT NULL,airbrakes INTEGER NOT NULL,rcs INTEGER NOT NULL, com_root_x REAL,com_root_y REAL,com_root_z REAL,inertia_x REAL,inertia_y REAL,inertia_z REAL, pos_x REAL NOT NULL,pos_y REAL NOT NULL,pos_z REAL NOT NULL,vel_x REAL NOT NULL,vel_y REAL NOT NULL,vel_z REAL NOT NULL, lift_x REAL NOT NULL,lift_y REAL NOT NULL,lift_z REAL NOT NULL,drag_x REAL NOT NULL,drag_y REAL NOT NULL,drag_z REAL NOT NULL);CREATE INDEX IF NOT EXISTS aero_context_recent ON aero_observations(structure_id,environment_id,id DESC);CREATE TABLE IF NOT EXISTS aero_observation_quality( observation_id INTEGER PRIMARY KEY,control_profile TEXT NOT NULL,body_pitch_rate REAL,body_roll_rate REAL,body_yaw_rate REAL, quality REAL NOT NULL,eligible INTEGER NOT NULL, FOREIGN KEY(observation_id) REFERENCES aero_observations(id) ON DELETE CASCADE);COMMIT;\0"
        as *const u8 as *const ::core::ffi::c_char;
    if !execute_sql((*store).db, schema_sql, error, error_size) {
        sqlite3_exec(
            (*store).db,
            b"ROLLBACK\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        );
        return false_0 != 0;
    }
    if !ensure_column(
        (*store).db,
        b"physics_sessions\0" as *const u8 as *const ::core::ffi::c_char,
        b"flight_key\0" as *const u8 as *const ::core::ffi::c_char,
        b"TEXT\0" as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"dry_mass\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"com_root_x\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"com_root_y\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"com_root_z\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"inertia_x\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"inertia_y\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
        || !ensure_column(
            (*store).db,
            b"aero_observations\0" as *const u8 as *const ::core::ffi::c_char,
            b"inertia_z\0" as *const u8 as *const ::core::ffi::c_char,
            b"REAL\0" as *const u8 as *const ::core::ffi::c_char,
            error,
            error_size,
        )
    {
        return false_0 != 0;
    }
    if !execute_sql(
        (*store).db,
        b"INSERT OR REPLACE INTO physics_meta(key,value) VALUES('schema','3')\0"
            as *const u8 as *const ::core::ffi::c_char,
        error,
        error_size,
    ) {
        return false_0 != 0;
    }
    refresh_schema_flags(store);
    return true_0 != 0;
}
unsafe extern "C" fn add_context(
    mut store: *mut PhysicsStore,
    mut structure_id: *const ::core::ffi::c_char,
    mut environment_id: *const ::core::ffi::c_char,
) -> bool {
    let mut i: size_t = 0 as size_t;
    while i < (*store).context_count {
        if strcmp(
            &raw mut (*(*store).contexts.offset(i as isize)).structure_id
                as *mut ::core::ffi::c_char,
            structure_id,
        ) == 0 as ::core::ffi::c_int
            && strcmp(
                &raw mut (*(*store).contexts.offset(i as isize)).environment_id
                    as *mut ::core::ffi::c_char,
                environment_id,
            ) == 0 as ::core::ffi::c_int
        {
            return true_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    if (*store).context_count == (*store).context_capacity {
        let mut next: size_t = if (*store).context_capacity != 0 {
            (*store).context_capacity.wrapping_mul(2 as size_t)
        } else {
            8 as size_t
        };
        let mut grown: *mut ContextPair = realloc(
            (*store).contexts as *mut ::core::ffi::c_void,
            next.wrapping_mul(::core::mem::size_of::<ContextPair>() as size_t),
        ) as *mut ContextPair;
        if grown.is_null() {
            return false_0 != 0;
        }
        (*store).contexts = grown;
        (*store).context_capacity = next;
    }
    let fresh1 = (*store).context_count;
    (*store).context_count = (*store).context_count.wrapping_add(1);
    let mut pair: *mut ContextPair = (*store).contexts.offset(fresh1 as isize)
        as *mut ContextPair;
    if !copy_string(
        &raw mut (*pair).structure_id as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
        structure_id,
    )
        || !copy_string(
            &raw mut (*pair).environment_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            environment_id,
        )
    {
        (*store).context_count = (*store).context_count.wrapping_sub(1);
        return false_0 != 0;
    }
    return true_0 != 0;
}
unsafe extern "C" fn environment_has_curve_digest(
    mut manifest_json: *const ::core::ffi::c_char,
) -> bool {
    let mut parsed: ParsedJson = ParsedJson {
        tokens: ::core::ptr::null_mut::<JsonToken>(),
        doc: JsonDoc {
            text: ::core::ptr::null::<::core::ffi::c_char>(),
            tokens: ::core::ptr::null_mut::<JsonToken>(),
            count: 0,
        },
    };
    if !parsed_json_init(&raw mut parsed, manifest_json) {
        return false_0 != 0;
    }
    let mut digest: ::core::ffi::c_int = json_object_get(
        &raw mut parsed.doc,
        0 as ::core::ffi::c_int,
        b"atmosphereCurveDigest\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut present: bool = if digest >= 0 as ::core::ffi::c_int
        && (*parsed.doc.tokens.offset(digest as isize)).type_0 as ::core::ffi::c_uint
            != JSON_PRIMITIVE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        true_0
    } else {
        false_0
    } != 0;
    if digest >= 0 as ::core::ffi::c_int
        && (*parsed.doc.tokens.offset(digest as isize)).type_0 as ::core::ffi::c_uint
            == JSON_PRIMITIVE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        present = !json_token_eq(
            &raw mut parsed.doc,
            digest,
            b"null\0" as *const u8 as *const ::core::ffi::c_char,
        );
    }
    parsed_json_clear(&raw mut parsed);
    return present;
}
unsafe extern "C" fn environment_planet_identity_json(
    mut manifest_json: *const ::core::ffi::c_char,
) -> *mut ::core::ffi::c_char {
    let mut parsed: ParsedJson = ParsedJson {
        tokens: ::core::ptr::null_mut::<JsonToken>(),
        doc: JsonDoc {
            text: ::core::ptr::null::<::core::ffi::c_char>(),
            tokens: ::core::ptr::null_mut::<JsonToken>(),
            count: 0,
        },
    };
    if !parsed_json_init(&raw mut parsed, manifest_json) {
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut doc: *const JsonDoc = &raw mut parsed.doc;
    let mut planet: ::core::ffi::c_int = json_object_get(
        doc,
        0 as ::core::ffi::c_int,
        b"planet\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if planet < 0 as ::core::ffi::c_int
        || (*(*doc).tokens.offset(planet as isize)).type_0 as ::core::ffi::c_uint
            != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        parsed_json_clear(&raw mut parsed);
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut writer: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut writer);
    jw_raw(
        &raw mut writer,
        b"{\"planet\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut ok: bool = canonical_json_token(&raw mut writer, doc, planet, true_0 != 0);
    jw_raw(
        &raw mut writer,
        b",\"schema\":3}\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut result: *mut ::core::ffi::c_char = if ok as ::core::ffi::c_int != 0
        && !writer.failed
    {
        duplicate_string(writer.data)
    } else {
        ::core::ptr::null_mut::<::core::ffi::c_char>()
    };
    jw_free(&raw mut writer);
    parsed_json_clear(&raw mut parsed);
    return result;
}
unsafe extern "C" fn environment_manifests_equal(
    mut a: *const ::core::ffi::c_char,
    mut b: *const ::core::ffi::c_char,
) -> bool {
    let mut na: *mut ::core::ffi::c_char = environment_identity_json(a);
    let mut nb: *mut ::core::ffi::c_char = environment_identity_json(b);
    let mut equal: bool = !na.is_null() && !nb.is_null()
        && strcmp(na, nb) == 0 as ::core::ffi::c_int;
    free(na as *mut ::core::ffi::c_void);
    free(nb as *mut ::core::ffi::c_void);
    if equal {
        return true_0 != 0;
    }
    if environment_has_curve_digest(a) as ::core::ffi::c_int != 0
        && environment_has_curve_digest(b) as ::core::ffi::c_int != 0
    {
        return false_0 != 0;
    }
    let mut pa: *mut ::core::ffi::c_char = environment_planet_identity_json(a);
    let mut pb: *mut ::core::ffi::c_char = environment_planet_identity_json(b);
    equal = !pa.is_null() && !pb.is_null() && strcmp(pa, pb) == 0 as ::core::ffi::c_int;
    free(pa as *mut ::core::ffi::c_void);
    free(pb as *mut ::core::ffi::c_void);
    return equal;
}
unsafe extern "C" fn discover_compatible_contexts(
    mut store: *mut PhysicsStore,
    mut current_structure: *const StructureIdentity,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !(*store).has_contexts_table {
        return add_context(
            store,
            &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
            &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
        );
    }
    let mut stmt: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    let mut sql: *const ::core::ffi::c_char = b"SELECT structure_id,environment_id,structure_json,environment_json FROM physics_contexts ORDER BY last_seen DESC\0"
        as *const u8 as *const ::core::ffi::c_char;
    if sqlite3_prepare_v2(
        (*store).db,
        sql,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) != SQLITE_OK
    {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not inspect physics contexts\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut reused_manual_environment: bool = false_0 != 0;
    while sqlite3_step(stmt) == SQLITE_ROW {
        let mut structure_id: *const ::core::ffi::c_char = sqlite3_column_text(
            stmt,
            0 as ::core::ffi::c_int,
        ) as *const ::core::ffi::c_char;
        let mut environment_id: *const ::core::ffi::c_char = sqlite3_column_text(
            stmt,
            1 as ::core::ffi::c_int,
        ) as *const ::core::ffi::c_char;
        let mut structure_json: *const ::core::ffi::c_char = sqlite3_column_text(
            stmt,
            2 as ::core::ffi::c_int,
        ) as *const ::core::ffi::c_char;
        let mut environment_json: *const ::core::ffi::c_char = sqlite3_column_text(
            stmt,
            3 as ::core::ffi::c_int,
        ) as *const ::core::ffi::c_char;
        if structure_id.is_null() || environment_id.is_null() || structure_json.is_null()
            || environment_json.is_null()
        {
            continue;
        }
        if !environment_manifests_equal(
            (*store).environment_manifest_json,
            environment_json,
        ) {
            continue;
        }
        let mut same_manual: bool = strcmp(
            structure_id,
            &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
        ) == 0 as ::core::ffi::c_int;
        let mut matching_legacy: bool = false_0 != 0;
        if !same_manual
            && starts_with(
                structure_id,
                b"sha256:\0" as *const u8 as *const ::core::ffi::c_char,
            ) as ::core::ffi::c_int != 0
        {
            let mut legacy: StructureIdentity = StructureIdentity {
                part_names: ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
                part_count: 0,
                geometry: ::core::ptr::null_mut::<StructureGeometry>(),
                geometry_count: 0,
                geometry_available: false,
            };
            if structure_identity_init(&raw mut legacy, structure_json) {
                matching_legacy = structure_identity_equal(
                    current_structure,
                    &raw mut legacy,
                );
                structure_identity_clear(&raw mut legacy);
            }
        }
        if !same_manual && !matching_legacy {
            continue;
        }
        if !add_context(store, structure_id, environment_id) {
            sqlite3_finalize(stmt);
            set_error(
                error,
                error_size,
                b"Out of memory collecting physics contexts\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
        if same_manual as ::core::ffi::c_int != 0 && !reused_manual_environment {
            if !copy_string(
                &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                environment_id,
            ) {
                sqlite3_finalize(stmt);
                set_error(
                    error,
                    error_size,
                    b"Persisted environment ID is too long\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
                return false_0 != 0;
            }
            reused_manual_environment = true_0 != 0;
        }
    }
    let mut rc: ::core::ffi::c_int = sqlite3_finalize(stmt);
    if rc != SQLITE_OK {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not finish physics context scan\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    return add_context(
        store,
        &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
        &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
    );
}
unsafe extern "C" fn store_current_context(
    mut store: *mut PhysicsStore,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    let mut now: ::core::ffi::c_double = realtime_seconds();
    let mut stmt: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    let mut context_sql: *const ::core::ffi::c_char = b"INSERT INTO physics_contexts(structure_id,environment_id,structure_json,environment_json,first_seen,last_seen) VALUES(?,?,?,?,?,?) ON CONFLICT(structure_id,environment_id) DO UPDATE SET structure_json=excluded.structure_json,environment_json=excluded.environment_json,last_seen=excluded.last_seen\0"
        as *const u8 as *const ::core::ffi::c_char;
    if sqlite3_prepare_v2(
        (*store).db,
        context_sql,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) != SQLITE_OK
    {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not prepare physics context\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    sqlite3_bind_text(
        stmt,
        1 as ::core::ffi::c_int,
        &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_text(
        stmt,
        2 as ::core::ffi::c_int,
        &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_text(
        stmt,
        3 as ::core::ffi::c_int,
        (*store).structure_manifest_json,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_text(
        stmt,
        4 as ::core::ffi::c_int,
        (*store).environment_manifest_json,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_double(stmt, 5 as ::core::ffi::c_int, now);
    sqlite3_bind_double(stmt, 6 as ::core::ffi::c_int, now);
    let mut ok: bool = sqlite3_step(stmt) == SQLITE_DONE;
    sqlite3_finalize(stmt);
    if !ok {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not store physics context\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut session_sql: *const ::core::ffi::c_char = b"INSERT INTO physics_sessions(session_id,structure_id,environment_id,vessel_name,started_wall,flight_key) VALUES(?,?,?,?,?,?)\0"
        as *const u8 as *const ::core::ffi::c_char;
    if sqlite3_prepare_v2(
        (*store).db,
        session_sql,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) != SQLITE_OK
    {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not prepare physics session\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    sqlite3_bind_text(
        stmt,
        1 as ::core::ffi::c_int,
        &raw mut (*store).session_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_text(
        stmt,
        2 as ::core::ffi::c_int,
        &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_text(
        stmt,
        3 as ::core::ffi::c_int,
        &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_text(
        stmt,
        4 as ::core::ffi::c_int,
        &raw mut (*store).vessel_name as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    sqlite3_bind_double(stmt, 5 as ::core::ffi::c_int, now);
    sqlite3_bind_text(
        stmt,
        6 as ::core::ffi::c_int,
        &raw mut (*store).flight_key as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    ok = sqlite3_step(stmt) == SQLITE_DONE;
    sqlite3_finalize(stmt);
    if !ok {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not store physics session\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    return sqlite3_exec(
        (*store).db,
        b"COMMIT\0" as *const u8 as *const ::core::ffi::c_char,
        None,
        NULL,
        ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
    ) == SQLITE_OK || sqlite3_get_autocommit((*store).db) != 0;
}
unsafe extern "C" fn make_session_id(mut out: *mut ::core::ffi::c_char) {
    let mut ts: timespec = timespec { tv_sec: 0, tv_nsec: 0 };
    clock_gettime(CLOCK_REALTIME, &raw mut ts);
    snprintf(
        out as *mut ::core::ffi::c_char,
        PHYSICS_STORE_ID_CAPACITY as size_t,
        b"native-%ld-%ld-%ld\0" as *const u8 as *const ::core::ffi::c_char,
        getpid() as ::core::ffi::c_long,
        ts.tv_sec,
        ts.tv_nsec,
    );
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_open(
    mut options: *const PhysicsStoreOptions,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> *mut PhysicsStore {
    let mut flags: ::core::ffi::c_int = 0;
    let mut rc: ::core::ffi::c_int = 0;
    let mut health: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    let mut current_block: u64;
    if options.is_null() || (*options).path.is_null() || *(*options).path == 0
        || (*options).model_id.is_null() || *(*options).model_id == 0
        || (*options).structure_manifest_json.is_null()
        || (*options).environment_manifest_json.is_null()
    {
        set_error(
            error,
            error_size,
            b"Physics store requires path, model ID, structure manifest and environment manifest\0"
                as *const u8 as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<PhysicsStore>();
    }
    let mut structure: StructureIdentity = StructureIdentity {
        part_names: ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        part_count: 0,
        geometry: ::core::ptr::null_mut::<StructureGeometry>(),
        geometry_count: 0,
        geometry_available: false,
    };
    if !structure_identity_init(&raw mut structure, (*options).structure_manifest_json) {
        set_error(
            error,
            error_size,
            b"Invalid structure manifest JSON\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<PhysicsStore>();
    }
    let mut structure_text: *mut ::core::ffi::c_char = structure_identity_text(
        &raw mut structure,
    );
    let mut environment_text: *mut ::core::ffi::c_char = environment_identity_json(
        (*options).environment_manifest_json,
    );
    if structure_text.is_null() || environment_text.is_null() {
        structure_identity_clear(&raw mut structure);
        free(structure_text as *mut ::core::ffi::c_void);
        free(environment_text as *mut ::core::ffi::c_void);
        set_error(
            error,
            error_size,
            b"Invalid physics identity manifest\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<PhysicsStore>();
    }
    let mut store: *mut PhysicsStore = calloc(
        1 as size_t,
        ::core::mem::size_of::<PhysicsStore>() as size_t,
    ) as *mut PhysicsStore;
    if store.is_null() {
        structure_identity_clear(&raw mut structure);
        free(structure_text as *mut ::core::ffi::c_void);
        free(environment_text as *mut ::core::ffi::c_void);
        set_error(
            error,
            error_size,
            b"Out of memory opening physics store\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return ::core::ptr::null_mut::<PhysicsStore>();
    }
    (*store).mode = (*options).mode;
    if !copy_string(
        &raw mut (*store).path as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 1024]>() as size_t,
        (*options).path,
    )
        || snprintf(
            &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            b"model:%s%s%s\0" as *const u8 as *const ::core::ffi::c_char,
            (*options).model_id,
            (if !(*options).structure_variant.is_null()
                && *(*options).structure_variant as ::core::ffi::c_int != 0
            {
                b"#\0" as *const u8 as *const ::core::ffi::c_char
            } else {
                b"\0" as *const u8 as *const ::core::ffi::c_char
            }),
            (if !(*options).structure_variant.is_null() {
                (*options).structure_variant
            } else {
                b"\0" as *const u8 as *const ::core::ffi::c_char
            }),
        ) >= ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as ::core::ffi::c_int
    {
        set_error(
            error,
            error_size,
            b"Physics store path or model ID is too long\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    } else {
        digest_text(
            structure_text,
            &raw mut (*store).structure_witness_id as *mut ::core::ffi::c_char,
        );
        digest_text(
            environment_text,
            &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
        );
        (*store).structure_manifest_json = duplicate_string(
            (*options).structure_manifest_json,
        );
        (*store).environment_manifest_json = duplicate_string(
            (*options).environment_manifest_json,
        );
        (*store).environment_identity_json = environment_text;
        environment_text = ::core::ptr::null_mut::<::core::ffi::c_char>();
        if (*store).structure_manifest_json.is_null()
            || (*store).environment_manifest_json.is_null()
        {
            set_error(
                error,
                error_size,
                b"Out of memory copying physics manifests\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
        } else {
            copy_string(
                &raw mut (*store).vessel_name as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                if !(*options).vessel_name.is_null() {
                    (*options).vessel_name
                } else {
                    (*options).model_id
                },
            );
            if !(*options).session_id.is_null()
                && *(*options).session_id as ::core::ffi::c_int != 0
            {
                copy_string(
                    &raw mut (*store).session_id as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                    (*options).session_id,
                );
            } else {
                make_session_id(
                    &raw mut (*store).session_id as *mut ::core::ffi::c_char,
                );
            }
            if !(*options).flight_key.is_null()
                && *(*options).flight_key as ::core::ffi::c_int != 0
            {
                copy_string(
                    &raw mut (*store).flight_key as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                    (*options).flight_key,
                );
            } else {
                copy_string(
                    &raw mut (*store).flight_key as *mut ::core::ffi::c_char,
                    ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                    &raw mut (*store).session_id as *mut ::core::ffi::c_char,
                );
            }
            flags = if (*options).mode as ::core::ffi::c_uint
                == PHYSICS_STORE_READ_ONLY as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                SQLITE_OPEN_READONLY
            } else {
                SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE
            };
            rc = sqlite3_open_v2(
                &raw mut (*store).path as *mut ::core::ffi::c_char,
                &raw mut (*store).db,
                flags,
                ::core::ptr::null::<::core::ffi::c_char>(),
            );
            if rc != SQLITE_OK {
                set_sqlite_error(
                    error,
                    error_size,
                    (*store).db,
                    b"Could not open physics database\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
            } else {
                health = ::core::ptr::null_mut::<sqlite3_stmt>();
                if sqlite3_prepare_v2(
                    (*store).db,
                    b"PRAGMA schema_version\0" as *const u8
                        as *const ::core::ffi::c_char,
                    -(1 as ::core::ffi::c_int),
                    &raw mut health,
                    ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
                ) != SQLITE_OK || sqlite3_step(health) != SQLITE_ROW
                {
                    sqlite3_finalize(health);
                    set_sqlite_error(
                        error,
                        error_size,
                        (*store).db,
                        b"Physics database is corrupt or unreadable\0" as *const u8
                            as *const ::core::ffi::c_char,
                    );
                } else {
                    sqlite3_finalize(health);
                    sqlite3_busy_timeout((*store).db, 250 as ::core::ffi::c_int);
                    if (*store).mode as ::core::ffi::c_uint
                        == PHYSICS_STORE_READ_WRITE as ::core::ffi::c_int
                            as ::core::ffi::c_uint
                    {
                        if !migrate_schema(store, error, error_size) {
                            current_block = 10393491857266413524;
                        } else {
                            current_block = 13131896068329595644;
                        }
                    } else {
                        refresh_schema_flags(store);
                        current_block = 13131896068329595644;
                    }
                    match current_block {
                        10393491857266413524 => {}
                        _ => {
                            if discover_compatible_contexts(
                                store,
                                &raw mut structure,
                                error,
                                error_size,
                            ) {
                                if !((*store).mode as ::core::ffi::c_uint
                                    == PHYSICS_STORE_READ_WRITE as ::core::ffi::c_int
                                        as ::core::ffi::c_uint
                                    && !store_current_context(store, error, error_size))
                                {
                                    structure_identity_clear(&raw mut structure);
                                    free(structure_text as *mut ::core::ffi::c_void);
                                    return store;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    structure_identity_clear(&raw mut structure);
    free(structure_text as *mut ::core::ffi::c_void);
    free(environment_text as *mut ::core::ffi::c_void);
    physics_store_close(store);
    return ::core::ptr::null_mut::<PhysicsStore>();
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_close(mut store: *mut PhysicsStore) {
    if store.is_null() {
        return;
    }
    if !(*store).db.is_null() {
        if (*store).mode as ::core::ffi::c_uint
            == PHYSICS_STORE_READ_WRITE as ::core::ffi::c_int as ::core::ffi::c_uint
            && (*store).pending != 0
        {
            sqlite3_exec(
                (*store).db,
                b"COMMIT\0" as *const u8 as *const ::core::ffi::c_char,
                None,
                NULL,
                ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
            );
        }
        sqlite3_close((*store).db);
    }
    free((*store).structure_manifest_json as *mut ::core::ffi::c_void);
    free((*store).environment_manifest_json as *mut ::core::ffi::c_void);
    free((*store).environment_identity_json as *mut ::core::ffi::c_void);
    free((*store).contexts as *mut ::core::ffi::c_void);
    free(store as *mut ::core::ffi::c_void);
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_flush(
    mut store: *mut PhysicsStore,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if store.is_null() || (*store).db.is_null() {
        set_error(
            error,
            error_size,
            b"Physics store is closed\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if (*store).mode as ::core::ffi::c_uint
        == PHYSICS_STORE_READ_ONLY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return true_0 != 0;
    }
    if sqlite3_exec(
        (*store).db,
        b"COMMIT\0" as *const u8 as *const ::core::ffi::c_char,
        None,
        NULL,
        ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
    ) != SQLITE_OK && sqlite3_get_autocommit((*store).db) == 0
    {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not flush physics store\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    (*store).pending = 0 as ::core::ffi::c_uint;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_structure_id(
    mut s: *const PhysicsStore,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).structure_id as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_structure_witness_id(
    mut s: *const PhysicsStore,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).structure_witness_id as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_environment_id(
    mut s: *const PhysicsStore,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).environment_id as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_session_id(
    mut s: *const PhysicsStore,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).session_id as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_flight_key(
    mut s: *const PhysicsStore,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).flight_key as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_path(
    mut s: *const PhysicsStore,
) -> *const ::core::ffi::c_char {
    return if !s.is_null() {
        &raw const (*s).path as *const ::core::ffi::c_char
    } else {
        b"\0" as *const u8 as *const ::core::ffi::c_char
    };
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_compatible_context_count(
    mut s: *const PhysicsStore,
) -> size_t {
    return if !s.is_null() { (*s).context_count } else { 0 as size_t };
}
unsafe extern "C" fn raw_krpc_to_canonical(mut v: Vector3) -> Vector3 {
    return v3(v.x, v.z, v.y);
}
unsafe extern "C" fn selected_to_sample(
    mut selected: *const SelectedCell,
    mut planet: *const PlanetModel,
    mut out: *mut VesselAeroSample,
) -> bool {
    let mut r: *const HistoryRow = &raw const (*selected).row;
    let mut pos: Vector3 = raw_krpc_to_canonical((*r).position);
    let mut vel: Vector3 = raw_krpc_to_canonical((*r).velocity);
    let mut lift_force: Vector3 = raw_krpc_to_canonical((*r).lift);
    let mut drag_force: Vector3 = raw_krpc_to_canonical((*r).drag);
    let mut omega: Vector3 = vscale((*planet).north_axis, (*planet).rotational_speed);
    let mut air: Vector3 = vsub(vel, vcross(omega, pos));
    if vmag(air) <= 1 as ::core::ffi::c_int as ::core::ffi::c_double {
        return false_0 != 0;
    }
    let mut dir: Vector3 = vnorm(
        air,
        v3(
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut up: Vector3 = vnorm(
        vproject_plane(pos, dir),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut side: Vector3 = vnorm(
        vcross(dir, up),
        v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        ),
    );
    let mut bank: ::core::ffi::c_double = (*r).roll * DEG2RAD;
    let mut lift_axis: Vector3 = vadd(vscale(up, cos(bank)), vscale(side, sin(bank)));
    let mut lateral: Vector3 = vsub(vscale(side, cos(bank)), vscale(up, sin(bank)));
    let mut force: Vector3 = vadd(lift_force, drag_force);
    let mut sample: VesselAeroSample = VesselAeroSample {
        q: (*r).q,
        mach: (*r).mach,
        aoa: (*r).aoa,
        beta: (*r).beta,
        mass: (*r).mass,
        force_per_q: vscale(
            v3(-vdot(force, dir), vdot(force, lift_axis), vdot(force, lateral)),
            1.0f64 / (*r).q,
        ),
        gear: (*r).gear != 0 as ::core::ffi::c_int,
        brakes: (*r).brakes != 0 as ::core::ffi::c_int,
        airbrakes: (*r).airbrakes,
        observations: (*selected).independent,
        trust: 1.0f64,
        last_observation_ut: if (*r).ut.is_finite() as i32 != 0 {
            (*r).ut
        } else {
            -::core::f32::INFINITY as ::core::ffi::c_double
        },
    };
    if !finite_vector(sample.force_per_q)
        || sample.force_per_q.x < 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return false_0 != 0;
    }
    *out = sample;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_load_history(
    mut store: *mut PhysicsStore,
    mut planet: *const PlanetModel,
    mut out_samples: *mut VesselAeroSample,
    mut maximum: size_t,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> size_t {
    if !error.is_null() && error_size != 0 {
        *error.offset(0 as ::core::ffi::c_int as isize) = 0 as ::core::ffi::c_char;
    }
    if store.is_null() || planet.is_null() || out_samples.is_null()
        || maximum == 0 as size_t
    {
        if store.is_null() || planet.is_null() || out_samples.is_null() {
            set_error(
                error,
                error_size,
                b"Physics history requires store, planet and output buffer\0"
                    as *const u8 as *const ::core::ffi::c_char,
            );
        }
        return 0 as size_t;
    }
    if maximum > VESSEL_AERO_SAMPLES as size_t {
        maximum = VESSEL_AERO_SAMPLES as size_t;
    }
    let mut rows: *mut HistoryRow = ::core::ptr::null_mut::<HistoryRow>();
    let mut row_count: size_t = 0 as size_t;
    if !history_query(store, &raw mut rows, &raw mut row_count, error, error_size) {
        return 0 as size_t;
    }
    let mut candidates: *mut SourceCandidate = ::core::ptr::null_mut::<
        SourceCandidate,
    >();
    let mut candidate_count: size_t = 0 as size_t;
    if !build_source_candidates(
        rows,
        row_count,
        &raw mut candidates,
        &raw mut candidate_count,
        error,
        error_size,
    ) {
        free(rows as *mut ::core::ffi::c_void);
        return 0 as size_t;
    }
    let mut selected: *mut SelectedCell = ::core::ptr::null_mut::<SelectedCell>();
    let mut selected_count: size_t = 0 as size_t;
    if !build_selected_cells(
        candidates,
        candidate_count,
        &raw mut selected,
        &raw mut selected_count,
        error,
        error_size,
    ) {
        free(candidates as *mut ::core::ffi::c_void);
        free(rows as *mut ::core::ffi::c_void);
        return 0 as size_t;
    }
    let mut indices: *mut size_t = (if selected_count != 0 {
        malloc(selected_count.wrapping_mul(::core::mem::size_of::<size_t>() as size_t))
    } else {
        NULL
    }) as *mut size_t;
    if selected_count != 0 && indices.is_null() {
        set_error(
            error,
            error_size,
            b"Out of memory bounding physics history\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        free(selected as *mut ::core::ffi::c_void);
        free(candidates as *mut ::core::ffi::c_void);
        free(rows as *mut ::core::ffi::c_void);
        return 0 as size_t;
    }
    let mut chosen: size_t = if selected_count != 0 {
        reduce_selected_cells(selected, selected_count, maximum, indices)
    } else {
        0 as size_t
    };
    if selected_count != 0 && chosen == 0 as size_t {
        set_error(
            error,
            error_size,
            b"Out of memory reducing physics history coverage\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        free(indices as *mut ::core::ffi::c_void);
        free(selected as *mut ::core::ffi::c_void);
        free(candidates as *mut ::core::ffi::c_void);
        free(rows as *mut ::core::ffi::c_void);
        return 0 as size_t;
    }
    let mut output: size_t = 0 as size_t;
    let mut i: size_t = 0 as size_t;
    while i < chosen && output < maximum {
        let mut sample: VesselAeroSample = VesselAeroSample {
            q: 0.,
            mach: 0.,
            aoa: 0.,
            beta: 0.,
            mass: 0.,
            force_per_q: Vector3 { x: 0., y: 0., z: 0. },
            gear: false,
            brakes: false,
            airbrakes: 0,
            observations: 0,
            trust: 0.,
            last_observation_ut: 0.,
        };
        if selected_to_sample(
            selected.offset(*indices.offset(i as isize) as isize) as *mut SelectedCell,
            planet,
            &raw mut sample,
        ) {
            let fresh7 = output;
            output = output.wrapping_add(1);
            *out_samples.offset(fresh7 as isize) = sample;
        }
        i = i.wrapping_add(1);
    }
    free(indices as *mut ::core::ffi::c_void);
    free(selected as *mut ::core::ffi::c_void);
    free(candidates as *mut ::core::ffi::c_void);
    free(rows as *mut ::core::ffi::c_void);
    return output;
}
unsafe extern "C" fn situation_excluded(
    mut situation: *const ::core::ffi::c_char,
) -> bool {
    if situation.is_null() {
        return false_0 != 0;
    }
    let mut lowered: [::core::ffi::c_char; 32] = [0; 32];
    let mut n: size_t = strlen(situation);
    if n >= ::core::mem::size_of::<[::core::ffi::c_char; 32]>() as usize {
        n = (::core::mem::size_of::<[::core::ffi::c_char; 32]>() as usize)
            .wrapping_sub(1 as usize) as size_t;
    }
    let mut i: size_t = 0 as size_t;
    while i < n {
        let mut c: ::core::ffi::c_char = ({
            let mut __res: ::core::ffi::c_int = 0;
            if ::core::mem::size_of::<::core::ffi::c_uchar>() as usize > 1 as usize {
                if 0 != 0 {
                    let mut __c: ::core::ffi::c_int = *situation.offset(i as isize)
                        as ::core::ffi::c_uchar as ::core::ffi::c_int;
                    __res = (if __c < -(128 as ::core::ffi::c_int)
                        || __c > 255 as ::core::ffi::c_int
                    {
                        __c as __int32_t
                    } else {
                        *(*__ctype_tolower_loc()).offset(__c as isize)
                    }) as ::core::ffi::c_int;
                } else {
                    __res = tolower(
                        *situation.offset(i as isize) as ::core::ffi::c_uchar
                            as ::core::ffi::c_int,
                    );
                }
            } else {
                __res = *(*__ctype_tolower_loc())
                    .offset(
                        *situation.offset(i as isize) as ::core::ffi::c_uchar
                            as ::core::ffi::c_int as isize,
                    ) as ::core::ffi::c_int;
            }
            __res
        }) as ::core::ffi::c_char;
        lowered[i as usize] = (if c as ::core::ffi::c_int == '_' as i32 {
            '-' as i32
        } else {
            c as ::core::ffi::c_int
        }) as ::core::ffi::c_char;
        i = i.wrapping_add(1);
    }
    lowered[n as usize] = 0 as ::core::ffi::c_char;
    return strcmp(
        &raw mut lowered as *mut ::core::ffi::c_char,
        b"landed\0" as *const u8 as *const ::core::ffi::c_char,
    ) == 0 as ::core::ffi::c_int
        || strcmp(
            &raw mut lowered as *mut ::core::ffi::c_char,
            b"splashed\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 as ::core::ffi::c_int
        || strcmp(
            &raw mut lowered as *mut ::core::ffi::c_char,
            b"pre-launch\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 as ::core::ffi::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn physics_store_observe(
    mut store: *mut PhysicsStore,
    mut o: *const PhysicsStoreObservation,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    if !error.is_null() && error_size != 0 {
        *error.offset(0 as ::core::ffi::c_int as isize) = 0 as ::core::ffi::c_char;
    }
    if store.is_null() || o.is_null() {
        set_error(
            error,
            error_size,
            b"Physics observation requires store and sample\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if (*store).mode as ::core::ffi::c_uint
        != PHYSICS_STORE_READ_WRITE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        set_error(
            error,
            error_size,
            b"Physics store is read-only\0" as *const u8 as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    if !(*o).sample_valid || !(*o).body_non_rotating
        || situation_excluded((*o).situation) as ::core::ffi::c_int != 0
    {
        return true_0 != 0;
    }
    if (*o).ut.is_finite() as i32 == 0 || (*o).q.is_finite() as i32 == 0
        || (*o).q <= 0.0f64 || (*o).mach.is_finite() as i32 == 0
        || (*o).aoa.is_finite() as i32 == 0 || (*o).beta.is_finite() as i32 == 0
        || (*o).roll.is_finite() as i32 == 0 || (*o).mass.is_finite() as i32 == 0
        || (*o).mass <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
        || !finite_vector((*o).position) || !finite_vector((*o).velocity)
        || !finite_vector((*o).lift) || !finite_vector((*o).drag)
    {
        return true_0 != 0;
    }
    if (*store).has_last_ut as ::core::ffi::c_int != 0 && (*o).ut < (*store).last_ut {
        (*store).timeline_epoch += 1;
        (*store).has_last_saved_ut = false_0 != 0;
    }
    (*store).last_ut = (*o).ut;
    (*store).has_last_ut = true_0 != 0;
    if (*store).has_last_saved_ut as ::core::ffi::c_int != 0
        && (*o).ut - (*store).last_saved_ut < archive_sample_period_s
    {
        return true_0 != 0;
    }
    (*store).last_saved_ut = (*o).ut;
    (*store).has_last_saved_ut = true_0 != 0;
    if (*store).pending == 0 as ::core::ffi::c_uint
        && sqlite3_exec(
            (*store).db,
            b"BEGIN\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        ) != SQLITE_OK
    {
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not begin physics observation batch\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut sql: *const ::core::ffi::c_char = b"INSERT INTO aero_observations(structure_id,environment_id,session_id,timeline_epoch,ut,wall_time,q,mach,aoa,beta,roll,mass,dry_mass,gear,brakes,airbrakes,rcs,com_root_x,com_root_y,com_root_z,inertia_x,inertia_y,inertia_z,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,lift_x,lift_y,lift_z,drag_x,drag_y,drag_z) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)\0"
        as *const u8 as *const ::core::ffi::c_char;
    let mut stmt: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    if sqlite3_prepare_v2(
        (*store).db,
        sql,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) != SQLITE_OK
    {
        sqlite3_exec(
            (*store).db,
            b"ROLLBACK\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        );
        (*store).pending = 0 as ::core::ffi::c_uint;
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not prepare physics observation\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut b: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
    let fresh19 = b;
    b = b + 1;
    sqlite3_bind_text(
        stmt,
        fresh19,
        &raw mut (*store).structure_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    let fresh20 = b;
    b = b + 1;
    sqlite3_bind_text(
        stmt,
        fresh20,
        &raw mut (*store).environment_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    let fresh21 = b;
    b = b + 1;
    sqlite3_bind_text(
        stmt,
        fresh21,
        &raw mut (*store).session_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    let fresh22 = b;
    b = b + 1;
    sqlite3_bind_int(stmt, fresh22, (*store).timeline_epoch);
    let fresh23 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh23, (*o).ut);
    let fresh24 = b;
    b = b + 1;
    sqlite3_bind_double(
        stmt,
        fresh24,
        if (*o).wall_time.is_finite() as i32 != 0 {
            (*o).wall_time
        } else {
            realtime_seconds()
        },
    );
    let fresh25 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh25, (*o).q);
    let fresh26 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh26, (*o).mach);
    let fresh27 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh27, (*o).aoa);
    let fresh28 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh28, (*o).beta);
    let fresh29 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh29, (*o).roll);
    let fresh30 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh30, (*o).mass);
    if (*o).has_dry_mass as ::core::ffi::c_int != 0
        && (*o).dry_mass.is_finite() as i32 != 0
    {
        let fresh31 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh31, (*o).dry_mass);
    } else {
        let fresh32 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh32);
    }
    let fresh33 = b;
    b = b + 1;
    sqlite3_bind_int(stmt, fresh33, (*o).gear as ::core::ffi::c_int);
    let fresh34 = b;
    b = b + 1;
    sqlite3_bind_int(stmt, fresh34, (*o).brakes as ::core::ffi::c_int);
    let fresh35 = b;
    b = b + 1;
    sqlite3_bind_int(stmt, fresh35, (*o).airbrakes);
    let fresh36 = b;
    b = b + 1;
    sqlite3_bind_int(stmt, fresh36, (*o).rcs);
    if (*o).has_center_of_mass_root as ::core::ffi::c_int != 0
        && finite_vector((*o).center_of_mass_root) as ::core::ffi::c_int != 0
    {
        let fresh37 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh37, (*o).center_of_mass_root.x);
        let fresh38 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh38, (*o).center_of_mass_root.y);
        let fresh39 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh39, (*o).center_of_mass_root.z);
    } else {
        let fresh40 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh40);
        let fresh41 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh41);
        let fresh42 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh42);
    }
    if (*o).has_moment_of_inertia as ::core::ffi::c_int != 0
        && finite_vector((*o).moment_of_inertia) as ::core::ffi::c_int != 0
    {
        let fresh43 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh43, (*o).moment_of_inertia.x);
        let fresh44 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh44, (*o).moment_of_inertia.y);
        let fresh45 = b;
        b = b + 1;
        sqlite3_bind_double(stmt, fresh45, (*o).moment_of_inertia.z);
    } else {
        let fresh46 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh46);
        let fresh47 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh47);
        let fresh48 = b;
        b = b + 1;
        sqlite3_bind_null(stmt, fresh48);
    }
    let fresh49 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh49, (*o).position.x);
    let fresh50 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh50, (*o).position.y);
    let fresh51 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh51, (*o).position.z);
    let fresh52 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh52, (*o).velocity.x);
    let fresh53 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh53, (*o).velocity.y);
    let fresh54 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh54, (*o).velocity.z);
    let fresh55 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh55, (*o).lift.x);
    let fresh56 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh56, (*o).lift.y);
    let fresh57 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh57, (*o).lift.z);
    let fresh58 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh58, (*o).drag.x);
    let fresh59 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh59, (*o).drag.y);
    let fresh60 = b;
    b = b + 1;
    sqlite3_bind_double(stmt, fresh60, (*o).drag.z);
    let mut ok: bool = sqlite3_step(stmt) == SQLITE_DONE;
    let mut observation_id: ::core::ffi::c_longlong = sqlite3_last_insert_rowid(
        (*store).db,
    ) as ::core::ffi::c_longlong;
    sqlite3_finalize(stmt);
    if !ok {
        sqlite3_exec(
            (*store).db,
            b"ROLLBACK\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        );
        (*store).pending = 0 as ::core::ffi::c_uint;
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not insert physics observation\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut qsql: *const ::core::ffi::c_char = b"INSERT OR REPLACE INTO aero_observation_quality(observation_id,control_profile,body_pitch_rate,body_roll_rate,body_yaw_rate,quality,eligible) VALUES(?,?,?,?,?,?,?)\0"
        as *const u8 as *const ::core::ffi::c_char;
    if sqlite3_prepare_v2(
        (*store).db,
        qsql,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) != SQLITE_OK
    {
        sqlite3_exec(
            (*store).db,
            b"ROLLBACK\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        );
        (*store).pending = 0 as ::core::ffi::c_uint;
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not prepare observation quality\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    sqlite3_bind_int64(stmt, 1 as ::core::ffi::c_int, observation_id as sqlite3_int64);
    sqlite3_bind_text(
        stmt,
        2 as ::core::ffi::c_int,
        if !(*o).control_profile.is_null() {
            (*o).control_profile
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        },
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    if (*o).has_body_rates as ::core::ffi::c_int != 0
        && finite_vector((*o).body_rates) as ::core::ffi::c_int != 0
    {
        sqlite3_bind_double(stmt, 3 as ::core::ffi::c_int, (*o).body_rates.x);
        sqlite3_bind_double(stmt, 4 as ::core::ffi::c_int, (*o).body_rates.y);
        sqlite3_bind_double(stmt, 5 as ::core::ffi::c_int, (*o).body_rates.z);
    } else {
        sqlite3_bind_null(stmt, 3 as ::core::ffi::c_int);
        sqlite3_bind_null(stmt, 4 as ::core::ffi::c_int);
        sqlite3_bind_null(stmt, 5 as ::core::ffi::c_int);
    }
    sqlite3_bind_double(stmt, 6 as ::core::ffi::c_int, 1.0f64);
    sqlite3_bind_int(stmt, 7 as ::core::ffi::c_int, 1 as ::core::ffi::c_int);
    ok = sqlite3_step(stmt) == SQLITE_DONE;
    sqlite3_finalize(stmt);
    if !ok {
        sqlite3_exec(
            (*store).db,
            b"ROLLBACK\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        );
        (*store).pending = 0 as ::core::ffi::c_uint;
        set_sqlite_error(
            error,
            error_size,
            (*store).db,
            b"Could not insert observation provenance\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    (*store).pending = (*store).pending.wrapping_add(1);
    if (*store).pending >= STORE_COMMIT_BATCH as ::core::ffi::c_uint {
        if sqlite3_exec(
            (*store).db,
            b"COMMIT\0" as *const u8 as *const ::core::ffi::c_char,
            None,
            NULL,
            ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
        ) != SQLITE_OK
        {
            sqlite3_exec(
                (*store).db,
                b"ROLLBACK\0" as *const u8 as *const ::core::ffi::c_char,
                None,
                NULL,
                ::core::ptr::null_mut::<*mut ::core::ffi::c_char>(),
            );
            (*store).pending = 0 as ::core::ffi::c_uint;
            set_sqlite_error(
                error,
                error_size,
                (*store).db,
                b"Could not commit physics observations\0" as *const u8
                    as *const ::core::ffi::c_char,
            );
            return false_0 != 0;
        }
        (*store).pending = 0 as ::core::ffi::c_uint;
    }
    return true_0 != 0;
}
#[inline]
unsafe extern "C" fn tolower(mut __c: ::core::ffi::c_int) -> ::core::ffi::c_int {
    return if __c >= -(128 as ::core::ffi::c_int) && __c < 256 as ::core::ffi::c_int {
        *(*__ctype_tolower_loc()).offset(__c as isize) as ::core::ffi::c_int
    } else {
        __c
    };
}
unsafe extern "C" fn set_error(
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
    mut message: *const ::core::ffi::c_char,
) {
    if error.is_null() || error_size == 0 as size_t {
        return;
    }
    snprintf(
        error,
        error_size,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        if !message.is_null() {
            message
        } else {
            b"physics store error\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
}
unsafe extern "C" fn set_sqlite_error(
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
    mut db: *mut sqlite3,
    mut prefix: *const ::core::ffi::c_char,
) {
    if error.is_null() || error_size == 0 as size_t {
        return;
    }
    snprintf(
        error,
        error_size,
        b"%s: %s\0" as *const u8 as *const ::core::ffi::c_char,
        if !prefix.is_null() {
            prefix
        } else {
            b"SQLite error\0" as *const u8 as *const ::core::ffi::c_char
        },
        if !db.is_null() {
            sqlite3_errmsg(db)
        } else {
            b"unknown SQLite error\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
}
unsafe extern "C" fn copy_string(
    mut dst: *mut ::core::ffi::c_char,
    mut capacity: size_t,
    mut src: *const ::core::ffi::c_char,
) -> bool {
    if dst.is_null() || capacity == 0 as size_t {
        return false_0 != 0;
    }
    if src.is_null() {
        src = b"\0" as *const u8 as *const ::core::ffi::c_char;
    }
    let mut written: ::core::ffi::c_int = snprintf(
        dst,
        capacity,
        b"%s\0" as *const u8 as *const ::core::ffi::c_char,
        src,
    );
    return written >= 0 as ::core::ffi::c_int && (written as size_t) < capacity;
}
unsafe extern "C" fn duplicate_string(
    mut src: *const ::core::ffi::c_char,
) -> *mut ::core::ffi::c_char {
    if src.is_null() {
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut length: size_t = strlen(src).wrapping_add(1 as size_t);
    let mut copy: *mut ::core::ffi::c_char = malloc(length) as *mut ::core::ffi::c_char;
    if !copy.is_null() {
        memcpy(
            copy as *mut ::core::ffi::c_void,
            src as *const ::core::ffi::c_void,
            length,
        );
    }
    return copy;
}
unsafe extern "C" fn starts_with(
    mut text: *const ::core::ffi::c_char,
    mut prefix: *const ::core::ffi::c_char,
) -> bool {
    if text.is_null() || prefix.is_null() {
        return false_0 != 0;
    }
    let mut n: size_t = strlen(prefix);
    return strncmp(text, prefix, n) == 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn finite_vector(mut v: Vector3) -> bool {
    return v.x.is_finite() as i32 != 0 && v.y.is_finite() as i32 != 0
        && v.z.is_finite() as i32 != 0;
}
unsafe extern "C" fn realtime_seconds() -> ::core::ffi::c_double {
    let mut ts: timespec = timespec { tv_sec: 0, tv_nsec: 0 };
    if clock_gettime(CLOCK_REALTIME, &raw mut ts) != 0 as ::core::ffi::c_int {
        return 0.0f64;
    }
    return ts.tv_sec as ::core::ffi::c_double
        + ts.tv_nsec as ::core::ffi::c_double / 1e9f64;
}
unsafe extern "C" fn rotate_right(
    mut v: uint32_t,
    mut n: ::core::ffi::c_uint,
) -> uint32_t {
    return v >> n | v << (32 as ::core::ffi::c_uint).wrapping_sub(n);
}
unsafe extern "C" fn sha256_transform(
    mut s: *mut Sha256,
    mut block: *const ::core::ffi::c_uchar,
) {
    static mut k: [uint32_t; 64] = [
        0x428a2f98 as ::core::ffi::c_uint,
        0x71374491 as ::core::ffi::c_uint,
        0xb5c0fbcf as ::core::ffi::c_uint,
        0xe9b5dba5 as ::core::ffi::c_uint,
        0x3956c25b as ::core::ffi::c_uint,
        0x59f111f1 as ::core::ffi::c_uint,
        0x923f82a4 as ::core::ffi::c_uint,
        0xab1c5ed5 as ::core::ffi::c_uint,
        0xd807aa98 as ::core::ffi::c_uint,
        0x12835b01 as ::core::ffi::c_uint,
        0x243185be as ::core::ffi::c_uint,
        0x550c7dc3 as ::core::ffi::c_uint,
        0x72be5d74 as ::core::ffi::c_uint,
        0x80deb1fe as ::core::ffi::c_uint,
        0x9bdc06a7 as ::core::ffi::c_uint,
        0xc19bf174 as ::core::ffi::c_uint,
        0xe49b69c1 as ::core::ffi::c_uint,
        0xefbe4786 as ::core::ffi::c_uint,
        0xfc19dc6 as ::core::ffi::c_uint,
        0x240ca1cc as ::core::ffi::c_uint,
        0x2de92c6f as ::core::ffi::c_uint,
        0x4a7484aa as ::core::ffi::c_uint,
        0x5cb0a9dc as ::core::ffi::c_uint,
        0x76f988da as ::core::ffi::c_uint,
        0x983e5152 as ::core::ffi::c_uint,
        0xa831c66d as ::core::ffi::c_uint,
        0xb00327c8 as ::core::ffi::c_uint,
        0xbf597fc7 as ::core::ffi::c_uint,
        0xc6e00bf3 as ::core::ffi::c_uint,
        0xd5a79147 as ::core::ffi::c_uint,
        0x6ca6351 as ::core::ffi::c_uint,
        0x14292967 as ::core::ffi::c_uint,
        0x27b70a85 as ::core::ffi::c_uint,
        0x2e1b2138 as ::core::ffi::c_uint,
        0x4d2c6dfc as ::core::ffi::c_uint,
        0x53380d13 as ::core::ffi::c_uint,
        0x650a7354 as ::core::ffi::c_uint,
        0x766a0abb as ::core::ffi::c_uint,
        0x81c2c92e as ::core::ffi::c_uint,
        0x92722c85 as ::core::ffi::c_uint,
        0xa2bfe8a1 as ::core::ffi::c_uint,
        0xa81a664b as ::core::ffi::c_uint,
        0xc24b8b70 as ::core::ffi::c_uint,
        0xc76c51a3 as ::core::ffi::c_uint,
        0xd192e819 as ::core::ffi::c_uint,
        0xd6990624 as ::core::ffi::c_uint,
        0xf40e3585 as ::core::ffi::c_uint,
        0x106aa070 as ::core::ffi::c_uint,
        0x19a4c116 as ::core::ffi::c_uint,
        0x1e376c08 as ::core::ffi::c_uint,
        0x2748774c as ::core::ffi::c_uint,
        0x34b0bcb5 as ::core::ffi::c_uint,
        0x391c0cb3 as ::core::ffi::c_uint,
        0x4ed8aa4a as ::core::ffi::c_uint,
        0x5b9cca4f as ::core::ffi::c_uint,
        0x682e6ff3 as ::core::ffi::c_uint,
        0x748f82ee as ::core::ffi::c_uint,
        0x78a5636f as ::core::ffi::c_uint,
        0x84c87814 as ::core::ffi::c_uint,
        0x8cc70208 as ::core::ffi::c_uint,
        0x90befffa as ::core::ffi::c_uint,
        0xa4506ceb as ::core::ffi::c_uint,
        0xbef9a3f7 as ::core::ffi::c_uint,
        0xc67178f2 as ::core::ffi::c_uint,
    ];
    let mut w: [uint32_t; 64] = [0; 64];
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < 16 as ::core::ffi::c_uint {
        let mut j: ::core::ffi::c_uint = i.wrapping_mul(4 as ::core::ffi::c_uint);
        w[i as usize] = (*block.offset(j as isize) as uint32_t)
            << 24 as ::core::ffi::c_int
            | (*block.offset(j.wrapping_add(1 as ::core::ffi::c_uint) as isize)
                as uint32_t) << 16 as ::core::ffi::c_int
            | (*block.offset(j.wrapping_add(2 as ::core::ffi::c_uint) as isize)
                as uint32_t) << 8 as ::core::ffi::c_int
            | *block.offset(j.wrapping_add(3 as ::core::ffi::c_uint) as isize)
                as uint32_t;
        i = i.wrapping_add(1);
    }
    let mut i_0: ::core::ffi::c_uint = 16 as ::core::ffi::c_uint;
    while i_0 < 64 as ::core::ffi::c_uint {
        let mut s0: uint32_t = rotate_right(
            w[i_0.wrapping_sub(15 as ::core::ffi::c_uint) as usize],
            7 as ::core::ffi::c_uint,
        )
            ^ rotate_right(
                w[i_0.wrapping_sub(15 as ::core::ffi::c_uint) as usize],
                18 as ::core::ffi::c_uint,
            )
            ^ w[i_0.wrapping_sub(15 as ::core::ffi::c_uint) as usize]
                >> 3 as ::core::ffi::c_int;
        let mut s1: uint32_t = rotate_right(
            w[i_0.wrapping_sub(2 as ::core::ffi::c_uint) as usize],
            17 as ::core::ffi::c_uint,
        )
            ^ rotate_right(
                w[i_0.wrapping_sub(2 as ::core::ffi::c_uint) as usize],
                19 as ::core::ffi::c_uint,
            )
            ^ w[i_0.wrapping_sub(2 as ::core::ffi::c_uint) as usize]
                >> 10 as ::core::ffi::c_int;
        w[i_0 as usize] = w[i_0.wrapping_sub(16 as ::core::ffi::c_uint) as usize]
            .wrapping_add(s0)
            .wrapping_add(w[i_0.wrapping_sub(7 as ::core::ffi::c_uint) as usize])
            .wrapping_add(s1);
        i_0 = i_0.wrapping_add(1);
    }
    let mut a: uint32_t = (*s).h[0 as ::core::ffi::c_int as usize];
    let mut b: uint32_t = (*s).h[1 as ::core::ffi::c_int as usize];
    let mut c: uint32_t = (*s).h[2 as ::core::ffi::c_int as usize];
    let mut d: uint32_t = (*s).h[3 as ::core::ffi::c_int as usize];
    let mut e: uint32_t = (*s).h[4 as ::core::ffi::c_int as usize];
    let mut f: uint32_t = (*s).h[5 as ::core::ffi::c_int as usize];
    let mut g: uint32_t = (*s).h[6 as ::core::ffi::c_int as usize];
    let mut h: uint32_t = (*s).h[7 as ::core::ffi::c_int as usize];
    let mut i_1: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i_1 < 64 as ::core::ffi::c_uint {
        let mut s1_0: uint32_t = rotate_right(e, 6 as ::core::ffi::c_uint)
            ^ rotate_right(e, 11 as ::core::ffi::c_uint)
            ^ rotate_right(e, 25 as ::core::ffi::c_uint);
        let mut ch: uint32_t = e & f ^ !e & g;
        let mut t1: uint32_t = h
            .wrapping_add(s1_0)
            .wrapping_add(ch)
            .wrapping_add(k[i_1 as usize])
            .wrapping_add(w[i_1 as usize]);
        let mut s0_0: uint32_t = rotate_right(a, 2 as ::core::ffi::c_uint)
            ^ rotate_right(a, 13 as ::core::ffi::c_uint)
            ^ rotate_right(a, 22 as ::core::ffi::c_uint);
        let mut maj: uint32_t = a & b ^ a & c ^ b & c;
        let mut t2: uint32_t = s0_0.wrapping_add(maj);
        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
        i_1 = i_1.wrapping_add(1);
    }
    (*s).h[0 as ::core::ffi::c_int as usize] = (*s)
        .h[0 as ::core::ffi::c_int as usize]
        .wrapping_add(a);
    (*s).h[1 as ::core::ffi::c_int as usize] = (*s)
        .h[1 as ::core::ffi::c_int as usize]
        .wrapping_add(b);
    (*s).h[2 as ::core::ffi::c_int as usize] = (*s)
        .h[2 as ::core::ffi::c_int as usize]
        .wrapping_add(c);
    (*s).h[3 as ::core::ffi::c_int as usize] = (*s)
        .h[3 as ::core::ffi::c_int as usize]
        .wrapping_add(d);
    (*s).h[4 as ::core::ffi::c_int as usize] = (*s)
        .h[4 as ::core::ffi::c_int as usize]
        .wrapping_add(e);
    (*s).h[5 as ::core::ffi::c_int as usize] = (*s)
        .h[5 as ::core::ffi::c_int as usize]
        .wrapping_add(f);
    (*s).h[6 as ::core::ffi::c_int as usize] = (*s)
        .h[6 as ::core::ffi::c_int as usize]
        .wrapping_add(g);
    (*s).h[7 as ::core::ffi::c_int as usize] = (*s)
        .h[7 as ::core::ffi::c_int as usize]
        .wrapping_add(h);
}
unsafe extern "C" fn sha256_init(mut s: *mut Sha256) {
    static mut initial: [uint32_t; 8] = [
        0x6a09e667 as ::core::ffi::c_uint,
        0xbb67ae85 as ::core::ffi::c_uint,
        0x3c6ef372 as ::core::ffi::c_uint,
        0xa54ff53a as ::core::ffi::c_uint,
        0x510e527f as ::core::ffi::c_uint,
        0x9b05688c as ::core::ffi::c_uint,
        0x1f83d9ab as ::core::ffi::c_uint,
        0x5be0cd19 as ::core::ffi::c_uint,
    ];
    memcpy(
        &raw mut (*s).h as *mut uint32_t as *mut ::core::ffi::c_void,
        &raw const initial as *const uint32_t as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[uint32_t; 8]>() as size_t,
    );
    (*s).bits = 0 as uint64_t;
    (*s).used = 0 as size_t;
}
unsafe extern "C" fn sha256_update(
    mut s: *mut Sha256,
    mut data: *const ::core::ffi::c_void,
    mut length: size_t,
) {
    let mut p: *const ::core::ffi::c_uchar = data as *const ::core::ffi::c_uchar;
    (*s).bits = (*s).bits.wrapping_add((length as uint64_t).wrapping_mul(8 as uint64_t));
    while length != 0 {
        let mut room: size_t = (64 as size_t).wrapping_sub((*s).used);
        let mut take: size_t = if length < room { length } else { room };
        memcpy(
            (&raw mut (*s).block as *mut ::core::ffi::c_uchar).offset((*s).used as isize)
                as *mut ::core::ffi::c_void,
            p as *const ::core::ffi::c_void,
            take,
        );
        (*s).used = (*s).used.wrapping_add(take);
        p = p.offset(take as isize);
        length = length.wrapping_sub(take);
        if (*s).used == 64 as size_t {
            sha256_transform(
                s,
                &raw mut (*s).block as *mut ::core::ffi::c_uchar
                    as *const ::core::ffi::c_uchar,
            );
            (*s).used = 0 as size_t;
        }
    }
}
unsafe extern "C" fn sha256_finish(
    mut s: *mut Sha256,
    mut digest: *mut ::core::ffi::c_uchar,
) {
    let mut bits: uint64_t = (*s).bits;
    let mut one: ::core::ffi::c_uchar = 0x80 as ::core::ffi::c_uchar;
    sha256_update(s, &raw mut one as *const ::core::ffi::c_void, 1 as size_t);
    let mut zero: ::core::ffi::c_uchar = 0 as ::core::ffi::c_uchar;
    while (*s).used != 56 as size_t {
        sha256_update(s, &raw mut zero as *const ::core::ffi::c_void, 1 as size_t);
    }
    let mut length: [::core::ffi::c_uchar; 8] = [0; 8];
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < 8 as ::core::ffi::c_uint {
        length[(7 as ::core::ffi::c_uint).wrapping_sub(i) as usize] = (bits
            >> i.wrapping_mul(8 as ::core::ffi::c_uint)) as ::core::ffi::c_uchar;
        i = i.wrapping_add(1);
    }
    sha256_update(
        s,
        &raw mut length as *mut ::core::ffi::c_uchar as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_uchar; 8]>() as size_t,
    );
    let mut i_0: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i_0 < 8 as ::core::ffi::c_uint {
        *digest.offset(i_0.wrapping_mul(4 as ::core::ffi::c_uint) as isize) = ((*s)
            .h[i_0 as usize] >> 24 as ::core::ffi::c_int) as ::core::ffi::c_uchar;
        *digest
            .offset(
                i_0
                    .wrapping_mul(4 as ::core::ffi::c_uint)
                    .wrapping_add(1 as ::core::ffi::c_uint) as isize,
            ) = ((*s).h[i_0 as usize] >> 16 as ::core::ffi::c_int)
            as ::core::ffi::c_uchar;
        *digest
            .offset(
                i_0
                    .wrapping_mul(4 as ::core::ffi::c_uint)
                    .wrapping_add(2 as ::core::ffi::c_uint) as isize,
            ) = ((*s).h[i_0 as usize] >> 8 as ::core::ffi::c_int)
            as ::core::ffi::c_uchar;
        *digest
            .offset(
                i_0
                    .wrapping_mul(4 as ::core::ffi::c_uint)
                    .wrapping_add(3 as ::core::ffi::c_uint) as isize,
            ) = (*s).h[i_0 as usize] as ::core::ffi::c_uchar;
        i_0 = i_0.wrapping_add(1);
    }
}
unsafe extern "C" fn digest_text(
    mut text: *const ::core::ffi::c_char,
    mut out: *mut ::core::ffi::c_char,
) {
    let mut sha: Sha256 = Sha256 {
        h: [0; 8],
        bits: 0,
        block: [0; 64],
        used: 0,
    };
    let mut digest: [::core::ffi::c_uchar; 32] = [0; 32];
    sha256_init(&raw mut sha);
    sha256_update(
        &raw mut sha,
        (if !text.is_null() {
            text
        } else {
            b"\0" as *const u8 as *const ::core::ffi::c_char
        }) as *const ::core::ffi::c_void,
        if !text.is_null() { strlen(text) } else { 0 as size_t },
    );
    sha256_finish(&raw mut sha, &raw mut digest as *mut ::core::ffi::c_uchar);
    let mut p: *mut ::core::ffi::c_char = out as *mut ::core::ffi::c_char;
    memcpy(
        p as *mut ::core::ffi::c_void,
        b"sha256:\0" as *const u8 as *const ::core::ffi::c_char
            as *const ::core::ffi::c_void,
        7 as size_t,
    );
    p = p.offset(7 as ::core::ffi::c_int as isize);
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < 32 as ::core::ffi::c_uint {
        static mut hex: [::core::ffi::c_char; 17] = unsafe {
            ::core::mem::transmute::<
                [u8; 17],
                [::core::ffi::c_char; 17],
            >(*b"0123456789abcdef\0")
        };
        let fresh5 = p;
        p = p.offset(1);
        *fresh5 = hex[(digest[i as usize] as ::core::ffi::c_int
            >> 4 as ::core::ffi::c_int) as usize];
        let fresh6 = p;
        p = p.offset(1);
        *fresh6 = hex[(digest[i as usize] as ::core::ffi::c_int
            & 15 as ::core::ffi::c_int) as usize];
        i = i.wrapping_add(1);
    }
    *p = 0 as ::core::ffi::c_char;
}
unsafe extern "C" fn parsed_json_clear(mut p: *mut ParsedJson) {
    if p.is_null() {
        return;
    }
    free((*p).tokens as *mut ::core::ffi::c_void);
    memset(
        p as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<ParsedJson>() as size_t,
    );
}
unsafe extern "C" fn parsed_json_init(
    mut p: *mut ParsedJson,
    mut text: *const ::core::ffi::c_char,
) -> bool {
    if p.is_null() || text.is_null() {
        return false_0 != 0;
    }
    memset(
        p as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<ParsedJson>() as size_t,
    );
    let mut capacity: ::core::ffi::c_int = 256 as ::core::ffi::c_int;
    while capacity <= 32768 as ::core::ffi::c_int {
        let mut tokens: *mut JsonToken = calloc(
            capacity as size_t,
            ::core::mem::size_of::<JsonToken>() as size_t,
        ) as *mut JsonToken;
        if tokens.is_null() {
            return false_0 != 0;
        }
        let mut doc: JsonDoc = JsonDoc {
            text: ::core::ptr::null::<::core::ffi::c_char>(),
            tokens: ::core::ptr::null_mut::<JsonToken>(),
            count: 0,
        };
        let mut count: ::core::ffi::c_int = json_parse(
            text,
            tokens,
            capacity,
            &raw mut doc,
        );
        if count >= 1 as ::core::ffi::c_int {
            (*p).tokens = tokens;
            (*p).doc = doc;
            return true_0 != 0;
        }
        free(tokens as *mut ::core::ffi::c_void);
        if count != -(1 as ::core::ffi::c_int) {
            return false_0 != 0;
        }
        capacity *= 2 as ::core::ffi::c_int;
    }
    return false_0 != 0;
}
unsafe extern "C" fn json_string_dup(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
) -> *mut ::core::ffi::c_char {
    if doc.is_null() || index < 0 as ::core::ffi::c_int || index >= (*doc).count
        || (*(*doc).tokens.offset(index as isize)).type_0 as ::core::ffi::c_uint
            != JSON_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut raw: size_t = ((*(*doc).tokens.offset(index as isize)).end
        - (*(*doc).tokens.offset(index as isize)).start) as size_t;
    let mut result: *mut ::core::ffi::c_char = malloc(raw.wrapping_add(1 as size_t))
        as *mut ::core::ffi::c_char;
    if result.is_null() {
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    if !json_string(doc, index, result, raw.wrapping_add(1 as size_t)) {
        free(result as *mut ::core::ffi::c_void);
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    return result;
}
unsafe extern "C" fn compare_json_pair(
    mut a: *const ::core::ffi::c_void,
    mut b: *const ::core::ffi::c_void,
) -> ::core::ffi::c_int {
    let mut pa: *const JsonPair = a as *const JsonPair;
    let mut pb: *const JsonPair = b as *const JsonPair;
    return strcmp((*pa).key, (*pb).key);
}
unsafe extern "C" fn ignored_planet_identity_key(
    mut key: *const ::core::ffi::c_char,
) -> bool {
    return strcmp(key, b"northAxisKRPC\0" as *const u8 as *const ::core::ffi::c_char)
        == 0 as ::core::ffi::c_int
        || strcmp(
            key,
            b"primeMeridianAtEpochKRPC\0" as *const u8 as *const ::core::ffi::c_char,
        ) == 0 as ::core::ffi::c_int
        || strcmp(key, b"epochUT\0" as *const u8 as *const ::core::ffi::c_char)
            == 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn canonical_json_token(
    mut writer: *mut JsonWriter,
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut filter_planet_coordinates: bool,
) -> bool {
    if writer.is_null() || doc.is_null() || index < 0 as ::core::ffi::c_int
        || index >= (*doc).count
    {
        return false_0 != 0;
    }
    let mut token: *const JsonToken = (*doc).tokens.offset(index as isize)
        as *mut JsonToken;
    if (*token).type_0 as ::core::ffi::c_uint
        == JSON_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        let mut value: *mut ::core::ffi::c_char = json_string_dup(doc, index);
        if value.is_null() {
            return false_0 != 0;
        }
        jw_string(writer, value);
        free(value as *mut ::core::ffi::c_void);
        return !(*writer).failed;
    }
    if (*token).type_0 as ::core::ffi::c_uint
        == JSON_PRIMITIVE as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        let mut n: size_t = ((*token).end - (*token).start) as size_t;
        let mut raw: *const ::core::ffi::c_char = (*doc)
            .text
            .offset((*token).start as isize);
        if n == 4 as size_t
            && strncmp(
                raw,
                b"true\0" as *const u8 as *const ::core::ffi::c_char,
                4 as size_t,
            ) == 0 as ::core::ffi::c_int
            || n == 5 as size_t
                && strncmp(
                    raw,
                    b"false\0" as *const u8 as *const ::core::ffi::c_char,
                    5 as size_t,
                ) == 0 as ::core::ffi::c_int
            || n == 4 as size_t
                && strncmp(
                    raw,
                    b"null\0" as *const u8 as *const ::core::ffi::c_char,
                    4 as size_t,
                ) == 0 as ::core::ffi::c_int
        {
            let mut small: [::core::ffi::c_char; 6] = [0; 6];
            memcpy(
                &raw mut small as *mut ::core::ffi::c_char as *mut ::core::ffi::c_void,
                raw as *const ::core::ffi::c_void,
                n,
            );
            small[n as usize] = 0 as ::core::ffi::c_char;
            jw_raw(writer, &raw mut small as *mut ::core::ffi::c_char);
            return !(*writer).failed;
        }
        let mut number: [::core::ffi::c_char; 96] = [0; 96];
        if n >= ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as usize {
            return false_0 != 0;
        }
        memcpy(
            &raw mut number as *mut ::core::ffi::c_char as *mut ::core::ffi::c_void,
            raw as *const ::core::ffi::c_void,
            n,
        );
        number[n as usize] = 0 as ::core::ffi::c_char;
        *__errno_location() = 0 as ::core::ffi::c_int;
        let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
            ::core::ffi::c_char,
        >();
        let mut value_0: ::core::ffi::c_double = strtod(
            &raw mut number as *mut ::core::ffi::c_char,
            &raw mut end,
        );
        if *__errno_location() != 0 || end == &raw mut number as *mut ::core::ffi::c_char
            || *end as ::core::ffi::c_int != 0 || value_0.is_finite() as i32 == 0
        {
            return false_0 != 0;
        }
        jw_number(writer, value_0);
        return !(*writer).failed;
    }
    if (*token).type_0 as ::core::ffi::c_uint
        == JSON_ARRAY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        jw_char(writer, '[' as i32 as ::core::ffi::c_char);
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < (*token).size {
            let mut child: ::core::ffi::c_int = json_array_get(doc, index, i);
            if child < 0 as ::core::ffi::c_int {
                return false_0 != 0;
            }
            if i != 0 {
                jw_char(writer, ',' as i32 as ::core::ffi::c_char);
            }
            if !canonical_json_token(writer, doc, child, false_0 != 0) {
                return false_0 != 0;
            }
            i += 1;
        }
        jw_char(writer, ']' as i32 as ::core::ffi::c_char);
        return !(*writer).failed;
    }
    if (*token).type_0 as ::core::ffi::c_uint
        == JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        let mut pairs: *mut JsonPair = calloc(
            ((*token).size as size_t)
                .wrapping_div(2 as size_t)
                .wrapping_add(1 as size_t),
            ::core::mem::size_of::<JsonPair>() as size_t,
        ) as *mut JsonPair;
        if pairs.is_null() {
            return false_0 != 0;
        }
        let mut count: size_t = 0 as size_t;
        let mut i_0: ::core::ffi::c_int = index + 1 as ::core::ffi::c_int;
        while i_0 < (*doc).count
            && (*(*doc).tokens.offset(i_0 as isize)).start < (*token).end
        {
            let mut key_index: ::core::ffi::c_int = i_0;
            let mut value_index: ::core::ffi::c_int = i_0 + 1 as ::core::ffi::c_int;
            if value_index >= (*doc).count
                || (*(*doc).tokens.offset(key_index as isize)).type_0
                    as ::core::ffi::c_uint
                    != JSON_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
            {
                free(pairs as *mut ::core::ffi::c_void);
                return false_0 != 0;
            }
            let mut key: *mut ::core::ffi::c_char = json_string_dup(doc, key_index);
            if key.is_null() {
                free(pairs as *mut ::core::ffi::c_void);
                return false_0 != 0;
            }
            if !(filter_planet_coordinates as ::core::ffi::c_int != 0
                && ignored_planet_identity_key(key) as ::core::ffi::c_int != 0)
            {
                let ref mut fresh0 = (*pairs.offset(count as isize)).key;
                *fresh0 = key;
                (*pairs.offset(count as isize)).value_index = value_index;
                count = count.wrapping_add(1);
            } else {
                free(key as *mut ::core::ffi::c_void);
            }
            i_0 = json_skip(doc, value_index);
        }
        qsort(
            pairs as *mut ::core::ffi::c_void,
            count,
            ::core::mem::size_of::<JsonPair>() as size_t,
            Some(
                compare_json_pair
                    as unsafe extern "C" fn(
                        *const ::core::ffi::c_void,
                        *const ::core::ffi::c_void,
                    ) -> ::core::ffi::c_int,
            ),
        );
        jw_char(writer, '{' as i32 as ::core::ffi::c_char);
        let mut p: size_t = 0 as size_t;
        while p < count {
            if p != 0 {
                jw_char(writer, ',' as i32 as ::core::ffi::c_char);
            }
            jw_string(writer, (*pairs.offset(p as isize)).key);
            jw_char(writer, ':' as i32 as ::core::ffi::c_char);
            if !canonical_json_token(
                writer,
                doc,
                (*pairs.offset(p as isize)).value_index,
                false_0 != 0,
            ) {
                let mut j: size_t = 0 as size_t;
                while j < count {
                    free((*pairs.offset(j as isize)).key as *mut ::core::ffi::c_void);
                    j = j.wrapping_add(1);
                }
                free(pairs as *mut ::core::ffi::c_void);
                return false_0 != 0;
            }
            p = p.wrapping_add(1);
        }
        jw_char(writer, '}' as i32 as ::core::ffi::c_char);
        let mut p_0: size_t = 0 as size_t;
        while p_0 < count {
            free((*pairs.offset(p_0 as isize)).key as *mut ::core::ffi::c_void);
            p_0 = p_0.wrapping_add(1);
        }
        free(pairs as *mut ::core::ffi::c_void);
        return !(*writer).failed;
    }
    return false_0 != 0;
}
unsafe extern "C" fn environment_identity_json(
    mut manifest_json: *const ::core::ffi::c_char,
) -> *mut ::core::ffi::c_char {
    let mut parsed: ParsedJson = ParsedJson {
        tokens: ::core::ptr::null_mut::<JsonToken>(),
        doc: JsonDoc {
            text: ::core::ptr::null::<::core::ffi::c_char>(),
            tokens: ::core::ptr::null_mut::<JsonToken>(),
            count: 0,
        },
    };
    if !parsed_json_init(&raw mut parsed, manifest_json) {
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut doc: *const JsonDoc = &raw mut parsed.doc;
    let mut root: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    if (*(*doc).tokens.offset(root as isize)).type_0 as ::core::ffi::c_uint
        != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        parsed_json_clear(&raw mut parsed);
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut planet: ::core::ffi::c_int = json_object_get(
        doc,
        root,
        b"planet\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut digest: ::core::ffi::c_int = json_object_get(
        doc,
        root,
        b"atmosphereCurveDigest\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if planet < 0 as ::core::ffi::c_int
        || (*(*doc).tokens.offset(planet as isize)).type_0 as ::core::ffi::c_uint
            != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        parsed_json_clear(&raw mut parsed);
        return ::core::ptr::null_mut::<::core::ffi::c_char>();
    }
    let mut writer: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut writer);
    jw_raw(
        &raw mut writer,
        b"{\"atmosphereCurveDigest\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if digest >= 0 as ::core::ffi::c_int {
        canonical_json_token(&raw mut writer, doc, digest, false_0 != 0);
    } else {
        jw_null(&raw mut writer);
    }
    jw_raw(
        &raw mut writer,
        b",\"planet\":\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut ok: bool = canonical_json_token(&raw mut writer, doc, planet, true_0 != 0);
    jw_raw(
        &raw mut writer,
        b",\"schema\":3}\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut result: *mut ::core::ffi::c_char = if ok as ::core::ffi::c_int != 0
        && !writer.failed
    {
        duplicate_string(writer.data)
    } else {
        ::core::ptr::null_mut::<::core::ffi::c_char>()
    };
    jw_free(&raw mut writer);
    parsed_json_clear(&raw mut parsed);
    return result;
}
unsafe extern "C" fn compare_string_ptr(
    mut a: *const ::core::ffi::c_void,
    mut b: *const ::core::ffi::c_void,
) -> ::core::ffi::c_int {
    let mut sa: *const *const ::core::ffi::c_char = a
        as *const *const ::core::ffi::c_char;
    let mut sb: *const *const ::core::ffi::c_char = b
        as *const *const ::core::ffi::c_char;
    return strcmp(*sa, *sb);
}
unsafe extern "C" fn compare_geometry(
    mut a: *const ::core::ffi::c_void,
    mut b: *const ::core::ffi::c_void,
) -> ::core::ffi::c_int {
    let mut ga: *const StructureGeometry = a as *const StructureGeometry;
    let mut gb: *const StructureGeometry = b as *const StructureGeometry;
    let mut n: ::core::ffi::c_int = strcmp((*ga).name, (*gb).name);
    if n != 0 {
        return n;
    }
    if (*ga).x != (*gb).x {
        return if (*ga).x < (*gb).x {
            -(1 as ::core::ffi::c_int)
        } else {
            1 as ::core::ffi::c_int
        };
    }
    if (*ga).y != (*gb).y {
        return if (*ga).y < (*gb).y {
            -(1 as ::core::ffi::c_int)
        } else {
            1 as ::core::ffi::c_int
        };
    }
    if (*ga).z != (*gb).z {
        return if (*ga).z < (*gb).z {
            -(1 as ::core::ffi::c_int)
        } else {
            1 as ::core::ffi::c_int
        };
    }
    return 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn structure_identity_clear(mut identity: *mut StructureIdentity) {
    if identity.is_null() {
        return;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*identity).part_count {
        free(*(*identity).part_names.offset(i as isize) as *mut ::core::ffi::c_void);
        i = i.wrapping_add(1);
    }
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*identity).geometry_count {
        free(
            (*(*identity).geometry.offset(i_0 as isize)).name as *mut ::core::ffi::c_void,
        );
        i_0 = i_0.wrapping_add(1);
    }
    free((*identity).part_names as *mut ::core::ffi::c_void);
    free((*identity).geometry as *mut ::core::ffi::c_void);
    memset(
        identity as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<StructureIdentity>() as size_t,
    );
}
unsafe extern "C" fn structure_identity_init(
    mut identity: *mut StructureIdentity,
    mut manifest_json: *const ::core::ffi::c_char,
) -> bool {
    memset(
        identity as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<StructureIdentity>() as size_t,
    );
    let mut parsed: ParsedJson = ParsedJson {
        tokens: ::core::ptr::null_mut::<JsonToken>(),
        doc: JsonDoc {
            text: ::core::ptr::null::<::core::ffi::c_char>(),
            tokens: ::core::ptr::null_mut::<JsonToken>(),
            count: 0,
        },
    };
    if !parsed_json_init(&raw mut parsed, manifest_json) {
        return false_0 != 0;
    }
    let mut doc: *const JsonDoc = &raw mut parsed.doc;
    if (*(*doc).tokens.offset(0 as ::core::ffi::c_int as isize)).type_0
        as ::core::ffi::c_uint
        != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        parsed_json_clear(&raw mut parsed);
        return false_0 != 0;
    }
    let mut parts: ::core::ffi::c_int = json_object_get(
        doc,
        0 as ::core::ffi::c_int,
        b"parts\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if parts >= 0 as ::core::ffi::c_int
        && (*(*doc).tokens.offset(parts as isize)).type_0 as ::core::ffi::c_uint
            == JSON_ARRAY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        let mut count: ::core::ffi::c_int = (*(*doc).tokens.offset(parts as isize)).size;
        (*identity).part_names = calloc(
            count as size_t,
            ::core::mem::size_of::<*mut ::core::ffi::c_char>() as size_t,
        ) as *mut *mut ::core::ffi::c_char;
        if count != 0 && (*identity).part_names.is_null() {
            parsed_json_clear(&raw mut parsed);
            return false_0 != 0;
        }
        let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i < count {
            let mut part: ::core::ffi::c_int = json_array_get(doc, parts, i);
            let mut name: ::core::ffi::c_int = json_object_get(
                doc,
                part,
                b"name\0" as *const u8 as *const ::core::ffi::c_char,
            );
            let mut value: *mut ::core::ffi::c_char = if name >= 0 as ::core::ffi::c_int
            {
                json_string_dup(doc, name)
            } else {
                duplicate_string(
                    b"<unavailable>\0" as *const u8 as *const ::core::ffi::c_char,
                )
            };
            if value.is_null() {
                parsed_json_clear(&raw mut parsed);
                structure_identity_clear(identity);
                return false_0 != 0;
            }
            let fresh2 = (*identity).part_count;
            (*identity).part_count = (*identity).part_count.wrapping_add(1);
            let ref mut fresh3 = *(*identity).part_names.offset(fresh2 as isize);
            *fresh3 = value;
            i += 1;
        }
        qsort(
            (*identity).part_names as *mut ::core::ffi::c_void,
            (*identity).part_count,
            ::core::mem::size_of::<*mut ::core::ffi::c_char>() as size_t,
            Some(
                compare_string_ptr
                    as unsafe extern "C" fn(
                        *const ::core::ffi::c_void,
                        *const ::core::ffi::c_void,
                    ) -> ::core::ffi::c_int,
            ),
        );
    }
    let mut geometry_complete: bool = json_boolean(
        doc,
        json_object_get(
            doc,
            0 as ::core::ffi::c_int,
            b"geometryComplete\0" as *const u8 as *const ::core::ffi::c_char,
        ),
        false_0 != 0,
    );
    let mut geometry: ::core::ffi::c_int = json_object_get(
        doc,
        0 as ::core::ffi::c_int,
        b"geometry\0" as *const u8 as *const ::core::ffi::c_char,
    );
    let mut geometry_valid: bool = geometry_complete as ::core::ffi::c_int != 0
        && geometry >= 0 as ::core::ffi::c_int
        && (*(*doc).tokens.offset(geometry as isize)).type_0 as ::core::ffi::c_uint
            == JSON_ARRAY as ::core::ffi::c_int as ::core::ffi::c_uint;
    if geometry_valid {
        let mut count_0: ::core::ffi::c_int = (*(*doc).tokens.offset(geometry as isize))
            .size;
        (*identity).geometry = calloc(
            count_0 as size_t,
            ::core::mem::size_of::<StructureGeometry>() as size_t,
        ) as *mut StructureGeometry;
        if count_0 != 0 && (*identity).geometry.is_null() {
            parsed_json_clear(&raw mut parsed);
            structure_identity_clear(identity);
            return false_0 != 0;
        }
        let mut i_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while i_0 < count_0 {
            let mut item: ::core::ffi::c_int = json_array_get(doc, geometry, i_0);
            let mut name_0: ::core::ffi::c_int = json_object_get(
                doc,
                item,
                b"name\0" as *const u8 as *const ::core::ffi::c_char,
            );
            let mut position: ::core::ffi::c_int = json_object_get(
                doc,
                item,
                b"position\0" as *const u8 as *const ::core::ffi::c_char,
            );
            if name_0 < 0 as ::core::ffi::c_int || position < 0 as ::core::ffi::c_int
                || (*(*doc).tokens.offset(position as isize)).type_0
                    as ::core::ffi::c_uint
                    != JSON_ARRAY as ::core::ffi::c_int as ::core::ffi::c_uint
                || (*(*doc).tokens.offset(position as isize)).size
                    != 3 as ::core::ffi::c_int
            {
                geometry_valid = false_0 != 0;
                break;
            } else {
                let mut value_0: *mut ::core::ffi::c_char = json_string_dup(doc, name_0);
                let mut p: [::core::ffi::c_double; 3] = [0.; 3];
                let mut j: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while j < 3 as ::core::ffi::c_int {
                    p[j as usize] = json_number(
                        doc,
                        json_array_get(doc, position, j),
                        ::core::f32::NAN as ::core::ffi::c_double,
                    );
                    j += 1;
                }
                if value_0.is_null()
                    || p[0 as ::core::ffi::c_int as usize].is_finite() as i32 == 0
                    || p[1 as ::core::ffi::c_int as usize].is_finite() as i32 == 0
                    || p[2 as ::core::ffi::c_int as usize].is_finite() as i32 == 0
                {
                    free(value_0 as *mut ::core::ffi::c_void);
                    geometry_valid = false_0 != 0;
                    break;
                } else {
                    let fresh4 = (*identity).geometry_count;
                    (*identity).geometry_count = (*identity)
                        .geometry_count
                        .wrapping_add(1);
                    let mut g: *mut StructureGeometry = (*identity)
                        .geometry
                        .offset(fresh4 as isize) as *mut StructureGeometry;
                    (*g).name = value_0;
                    (*g).x = lrint(p[0 as ::core::ffi::c_int as usize] / 0.05f64);
                    (*g).y = lrint(p[1 as ::core::ffi::c_int as usize] / 0.05f64);
                    (*g).z = lrint(p[2 as ::core::ffi::c_int as usize] / 0.05f64);
                    i_0 += 1;
                }
            }
        }
    }
    (*identity).geometry_available = geometry_valid as ::core::ffi::c_int != 0
        && (*identity).part_count > 0 as size_t
        && (*identity).geometry_count == (*identity).part_count;
    if !(*identity).geometry_available {
        let mut i_1: size_t = 0 as size_t;
        while i_1 < (*identity).geometry_count {
            free(
                (*(*identity).geometry.offset(i_1 as isize)).name
                    as *mut ::core::ffi::c_void,
            );
            i_1 = i_1.wrapping_add(1);
        }
        free((*identity).geometry as *mut ::core::ffi::c_void);
        (*identity).geometry = ::core::ptr::null_mut::<StructureGeometry>();
        (*identity).geometry_count = 0 as size_t;
    } else {
        qsort(
            (*identity).geometry as *mut ::core::ffi::c_void,
            (*identity).geometry_count,
            ::core::mem::size_of::<StructureGeometry>() as size_t,
            Some(
                compare_geometry
                    as unsafe extern "C" fn(
                        *const ::core::ffi::c_void,
                        *const ::core::ffi::c_void,
                    ) -> ::core::ffi::c_int,
            ),
        );
    }
    parsed_json_clear(&raw mut parsed);
    return true_0 != 0;
}
unsafe extern "C" fn structure_identity_equal(
    mut a: *const StructureIdentity,
    mut b: *const StructureIdentity,
) -> bool {
    if a.is_null() || b.is_null() || (*a).part_count != (*b).part_count
        || (*a).geometry_available as ::core::ffi::c_int
            != (*b).geometry_available as ::core::ffi::c_int
    {
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*a).part_count {
        if strcmp(
            *(*a).part_names.offset(i as isize),
            *(*b).part_names.offset(i as isize),
        ) != 0 as ::core::ffi::c_int
        {
            return false_0 != 0;
        }
        i = i.wrapping_add(1);
    }
    if !(*a).geometry_available {
        return true_0 != 0;
    }
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*a).geometry_count {
        let mut x: *const StructureGeometry = (*a).geometry.offset(i_0 as isize)
            as *mut StructureGeometry;
        let mut y: *const StructureGeometry = (*b).geometry.offset(i_0 as isize)
            as *mut StructureGeometry;
        if strcmp((*x).name, (*y).name) != 0 as ::core::ffi::c_int || (*x).x != (*y).x
            || (*x).y != (*y).y || (*x).z != (*y).z
        {
            return false_0 != 0;
        }
        i_0 = i_0.wrapping_add(1);
    }
    return true_0 != 0;
}
unsafe extern "C" fn structure_identity_text(
    mut identity: *const StructureIdentity,
) -> *mut ::core::ffi::c_char {
    let mut writer: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut writer);
    jw_raw(
        &raw mut writer,
        b"structure-v3|parts=\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_integer(&raw mut writer, (*identity).part_count as ::core::ffi::c_longlong);
    let mut i: size_t = 0 as size_t;
    while i < (*identity).part_count {
        jw_char(&raw mut writer, '|' as i32 as ::core::ffi::c_char);
        jw_string(&raw mut writer, *(*identity).part_names.offset(i as isize));
        i = i.wrapping_add(1);
    }
    jw_raw(&raw mut writer, b"|geometry=\0" as *const u8 as *const ::core::ffi::c_char);
    jw_bool(&raw mut writer, (*identity).geometry_available);
    if (*identity).geometry_available {
        let mut i_0: size_t = 0 as size_t;
        while i_0 < (*identity).geometry_count {
            let mut g: *const StructureGeometry = (*identity)
                .geometry
                .offset(i_0 as isize) as *mut StructureGeometry;
            jw_char(&raw mut writer, '|' as i32 as ::core::ffi::c_char);
            jw_string(&raw mut writer, (*g).name);
            jw_char(&raw mut writer, ':' as i32 as ::core::ffi::c_char);
            jw_integer(&raw mut writer, (*g).x as ::core::ffi::c_longlong);
            jw_char(&raw mut writer, ',' as i32 as ::core::ffi::c_char);
            jw_integer(&raw mut writer, (*g).y as ::core::ffi::c_longlong);
            jw_char(&raw mut writer, ',' as i32 as ::core::ffi::c_char);
            jw_integer(&raw mut writer, (*g).z as ::core::ffi::c_longlong);
            i_0 = i_0.wrapping_add(1);
        }
    }
    let mut result: *mut ::core::ffi::c_char = if writer.failed as ::core::ffi::c_int
        != 0
    {
        ::core::ptr::null_mut::<::core::ffi::c_char>()
    } else {
        duplicate_string(writer.data)
    };
    jw_free(&raw mut writer);
    return result;
}
unsafe extern "C" fn build_source_candidates(
    mut rows: *const HistoryRow,
    mut count: size_t,
    mut out: *mut *mut SourceCandidate,
    mut out_count: *mut size_t,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    *out = ::core::ptr::null_mut::<SourceCandidate>();
    *out_count = 0 as size_t;
    if count == 0 as size_t {
        return true_0 != 0;
    }
    let mut used: *mut bool = calloc(count, ::core::mem::size_of::<bool>() as size_t)
        as *mut bool;
    let mut candidates: *mut SourceCandidate = calloc(
        count,
        ::core::mem::size_of::<SourceCandidate>() as size_t,
    ) as *mut SourceCandidate;
    let mut indices: *mut size_t = malloc(
        count.wrapping_mul(::core::mem::size_of::<size_t>() as size_t),
    ) as *mut size_t;
    if used.is_null() || candidates.is_null() || indices.is_null() {
        free(used as *mut ::core::ffi::c_void);
        free(candidates as *mut ::core::ffi::c_void);
        free(indices as *mut ::core::ffi::c_void);
        set_error(
            error,
            error_size,
            b"Out of memory reducing physics history\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < count {
        if !*used.offset(i as isize) {
            let mut key: CellKey = row_cell(
                rows.offset(i as isize) as *const HistoryRow,
            );
            let mut n: size_t = 0 as size_t;
            let mut j: size_t = i;
            while j < count {
                if !*used.offset(j as isize)
                    && cell_equal(
                        key,
                        row_cell(rows.offset(j as isize) as *const HistoryRow),
                    ) as ::core::ffi::c_int != 0
                    && strcmp(
                        &raw const (*rows.offset(i as isize)).source
                            as *const ::core::ffi::c_char,
                        &raw const (*rows.offset(j as isize)).source
                            as *const ::core::ffi::c_char,
                    ) == 0 as ::core::ffi::c_int
                {
                    *used.offset(j as isize) = true_0 != 0;
                    let fresh12 = n;
                    n = n.wrapping_add(1);
                    *indices.offset(fresh12 as isize) = j;
                }
                j = j.wrapping_add(1);
            }
            let mut chosen: size_t = choose_medoid(rows, indices, n, false_0 != 0);
            let fresh13 = *out_count;
            *out_count = (*out_count).wrapping_add(1);
            let mut candidate: *mut SourceCandidate = candidates.offset(fresh13 as isize)
                as *mut SourceCandidate;
            (*candidate).key = key;
            (*candidate).row = *rows.offset(chosen as isize);
            copy_string(
                &raw mut (*candidate).source as *mut ::core::ffi::c_char,
                ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
                &raw const (*rows.offset(i as isize)).source
                    as *const ::core::ffi::c_char,
            );
        }
        i = i.wrapping_add(1);
    }
    free(used as *mut ::core::ffi::c_void);
    free(indices as *mut ::core::ffi::c_void);
    *out = candidates;
    return true_0 != 0;
}
unsafe extern "C" fn build_selected_cells(
    mut candidates: *const SourceCandidate,
    mut count: size_t,
    mut out: *mut *mut SelectedCell,
    mut out_count: *mut size_t,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    *out = ::core::ptr::null_mut::<SelectedCell>();
    *out_count = 0 as size_t;
    if count == 0 as size_t {
        return true_0 != 0;
    }
    let mut used: *mut bool = calloc(count, ::core::mem::size_of::<bool>() as size_t)
        as *mut bool;
    let mut selected: *mut SelectedCell = calloc(
        count,
        ::core::mem::size_of::<SelectedCell>() as size_t,
    ) as *mut SelectedCell;
    let mut group: *mut size_t = malloc(
        count.wrapping_mul(::core::mem::size_of::<size_t>() as size_t),
    ) as *mut size_t;
    let mut group_rows: *mut HistoryRow = malloc(
        count.wrapping_mul(::core::mem::size_of::<HistoryRow>() as size_t),
    ) as *mut HistoryRow;
    let mut local: *mut size_t = malloc(
        count.wrapping_mul(::core::mem::size_of::<size_t>() as size_t),
    ) as *mut size_t;
    if used.is_null() || selected.is_null() || group.is_null() || group_rows.is_null()
        || local.is_null()
    {
        free(used as *mut ::core::ffi::c_void);
        free(selected as *mut ::core::ffi::c_void);
        free(group as *mut ::core::ffi::c_void);
        free(group_rows as *mut ::core::ffi::c_void);
        free(local as *mut ::core::ffi::c_void);
        set_error(
            error,
            error_size,
            b"Out of memory aggregating independent flights\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut i: size_t = 0 as size_t;
    while i < count {
        if !*used.offset(i as isize) {
            let mut n: size_t = 0 as size_t;
            let mut j: size_t = i;
            while j < count {
                if !*used.offset(j as isize)
                    && cell_equal(
                        (*candidates.offset(i as isize)).key,
                        (*candidates.offset(j as isize)).key,
                    ) as ::core::ffi::c_int != 0
                {
                    *used.offset(j as isize) = true_0 != 0;
                    let fresh10 = n;
                    n = n.wrapping_add(1);
                    *group.offset(fresh10 as isize) = j;
                }
                j = j.wrapping_add(1);
            }
            let mut j_0: size_t = 0 as size_t;
            while j_0 < n {
                *group_rows.offset(j_0 as isize) = (*candidates
                    .offset(*group.offset(j_0 as isize) as isize))
                    .row;
                *local.offset(j_0 as isize) = j_0;
                j_0 = j_0.wrapping_add(1);
            }
            let mut chosen_local: size_t = choose_medoid(
                group_rows,
                local,
                n,
                true_0 != 0,
            );
            let fresh11 = *out_count;
            *out_count = (*out_count).wrapping_add(1);
            let mut cell: *mut SelectedCell = selected.offset(fresh11 as isize)
                as *mut SelectedCell;
            (*cell).key = (*candidates.offset(i as isize)).key;
            (*cell).row = *group_rows.offset(chosen_local as isize);
            (*cell).independent = n as ::core::ffi::c_uint;
        }
        i = i.wrapping_add(1);
    }
    free(used as *mut ::core::ffi::c_void);
    free(group as *mut ::core::ffi::c_void);
    free(group_rows as *mut ::core::ffi::c_void);
    free(local as *mut ::core::ffi::c_void);
    *out = selected;
    return true_0 != 0;
}
unsafe extern "C" fn certified_regime_for_store(
    mut mach: ::core::ffi::c_double,
) -> ::core::ffi::c_int {
    static mut anchors: [::core::ffi::c_double; 4] = [0.35f64, 1.05f64, 2.6f64, 6.0f64];
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if mach
            < 0.5f64
                * (anchors[i as usize] + anchors[(i + 1 as ::core::ffi::c_int) as usize])
        {
            return i;
        }
        i += 1;
    }
    return 3 as ::core::ffi::c_int;
}
unsafe extern "C" fn cell_distance(
    mut a: CellKey,
    mut b: CellKey,
) -> ::core::ffi::c_double {
    if a.gear != b.gear || a.brakes != b.brakes || a.airbrakes != b.airbrakes {
        return ::core::f32::INFINITY as ::core::ffi::c_double;
    }
    let mut dq: ::core::ffi::c_double = (a.q - b.q) as ::core::ffi::c_double;
    let mut dm: ::core::ffi::c_double = (a.mach - b.mach) as ::core::ffi::c_double;
    let mut da: ::core::ffi::c_double = (a.aoa - b.aoa) as ::core::ffi::c_double
        / 2.5f64;
    let mut db: ::core::ffi::c_double = (a.beta - b.beta) as ::core::ffi::c_double
        / 2.5f64;
    return sqrt(dq * dq + dm * dm + da * da + db * db);
}
unsafe extern "C" fn better_support(
    mut candidate: *const SelectedCell,
    mut current: *const SelectedCell,
) -> bool {
    if (*candidate).independent != (*current).independent {
        return (*candidate).independent > (*current).independent;
    }
    return (*candidate).row.id > (*current).row.id;
}
unsafe extern "C" fn reduce_selected_cells(
    mut selected: *const SelectedCell,
    mut count: size_t,
    mut maximum: size_t,
    mut indices: *mut size_t,
) -> size_t {
    if count <= maximum {
        let mut i: size_t = 0 as size_t;
        while i < count {
            *indices.offset(i as isize) = i;
            i = i.wrapping_add(1);
        }
        return count;
    }
    let mut retained: *mut bool = calloc(count, ::core::mem::size_of::<bool>() as size_t)
        as *mut bool;
    if retained.is_null() {
        return 0 as size_t;
    }
    let mut n: size_t = 0 as size_t;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < count && n < maximum {
        let mut regime: ::core::ffi::c_int = certified_regime_for_store(
            (*selected.offset(i_0 as isize)).row.mach,
        );
        let mut group_seen: bool = false_0 != 0;
        let mut j: size_t = 0 as size_t;
        while j < n {
            let mut existing: *const SelectedCell = selected
                .offset(*indices.offset(j as isize) as isize) as *const SelectedCell;
            if (*existing).key.gear == (*selected.offset(i_0 as isize)).key.gear
                && (*existing).key.brakes == (*selected.offset(i_0 as isize)).key.brakes
                && (*existing).key.airbrakes
                    == (*selected.offset(i_0 as isize)).key.airbrakes
                && certified_regime_for_store((*existing).row.mach) == regime
            {
                group_seen = true_0 != 0;
                break;
            } else {
                j = j.wrapping_add(1);
            }
        }
        if !group_seen {
            let mut best: size_t = i_0;
            let mut j_0: size_t = i_0.wrapping_add(1 as size_t);
            while j_0 < count {
                if !((*selected.offset(j_0 as isize)).key.gear
                    != (*selected.offset(i_0 as isize)).key.gear
                    || (*selected.offset(j_0 as isize)).key.brakes
                        != (*selected.offset(i_0 as isize)).key.brakes
                    || (*selected.offset(j_0 as isize)).key.airbrakes
                        != (*selected.offset(i_0 as isize)).key.airbrakes
                    || certified_regime_for_store(
                        (*selected.offset(j_0 as isize)).row.mach,
                    ) != regime)
                {
                    if better_support(
                        selected.offset(j_0 as isize) as *const SelectedCell,
                        selected.offset(best as isize) as *const SelectedCell,
                    ) {
                        best = j_0;
                    }
                }
                j_0 = j_0.wrapping_add(1);
            }
            if !*retained.offset(best as isize) {
                *retained.offset(best as isize) = true_0 != 0;
                let fresh8 = n;
                n = n.wrapping_add(1);
                *indices.offset(fresh8 as isize) = best;
            }
        }
        i_0 = i_0.wrapping_add(1);
    }
    while n < maximum {
        let mut best_0: size_t = SIZE_MAX as size_t;
        let mut best_distance: ::core::ffi::c_double = -::core::f32::INFINITY
            as ::core::ffi::c_double;
        let mut i_1: size_t = 0 as size_t;
        while i_1 < count {
            if !*retained.offset(i_1 as isize) {
                let mut nearest: ::core::ffi::c_double = ::core::f32::INFINITY
                    as ::core::ffi::c_double;
                let mut j_1: size_t = 0 as size_t;
                while j_1 < n {
                    nearest = fmin(
                        nearest,
                        cell_distance(
                            (*selected.offset(i_1 as isize)).key,
                            (*selected.offset(*indices.offset(j_1 as isize) as isize))
                                .key,
                        ),
                    );
                    j_1 = j_1.wrapping_add(1);
                }
                let mut tie: bool = best_0 != SIZE_MAX as size_t
                    && numerical_tie(nearest, best_distance) as ::core::ffi::c_int != 0;
                if best_0 == SIZE_MAX as size_t || nearest > best_distance
                    || tie as ::core::ffi::c_int != 0
                        && better_support(
                            selected.offset(i_1 as isize) as *const SelectedCell,
                            selected.offset(best_0 as isize) as *const SelectedCell,
                        ) as ::core::ffi::c_int != 0
                {
                    best_0 = i_1;
                    best_distance = nearest;
                }
            }
            i_1 = i_1.wrapping_add(1);
        }
        if best_0 == SIZE_MAX as size_t {
            break;
        }
        *retained.offset(best_0 as isize) = true_0 != 0;
        let fresh9 = n;
        n = n.wrapping_add(1);
        *indices.offset(fresh9 as isize) = best_0;
    }
    free(retained as *mut ::core::ffi::c_void);
    return n;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
unsafe extern "C" fn cell_equal(mut a: CellKey, mut b: CellKey) -> bool {
    return a.q == b.q && a.mach == b.mach && a.aoa == b.aoa && a.beta == b.beta
        && a.gear == b.gear && a.brakes == b.brakes && a.airbrakes == b.airbrakes;
}
unsafe extern "C" fn row_cell(mut r: *const HistoryRow) -> CellKey {
    let mut q: ::core::ffi::c_double = fmax((*r).q, 1e-9f64);
    let mut k: CellKey = CellKey {
        q: lrint(log(q) / log(2.0f64)) as ::core::ffi::c_int,
        mach: lrint((*r).mach * 2.0f64) as ::core::ffi::c_int,
        aoa: lrint((*r).aoa / 4.0f64) as ::core::ffi::c_int,
        beta: lrint((*r).beta / 4.0f64) as ::core::ffi::c_int,
        gear: (*r).gear,
        brakes: (*r).brakes,
        airbrakes: (*r).airbrakes,
    };
    return k;
}
unsafe extern "C" fn append_context_where(
    mut w: *mut JsonWriter,
    mut s: *const PhysicsStore,
    mut alias: *const ::core::ffi::c_char,
) {
    jw_char(w, '(' as i32 as ::core::ffi::c_char);
    let mut i: size_t = 0 as size_t;
    while i < (*s).context_count {
        if i != 0 {
            jw_raw(w, b" OR \0" as *const u8 as *const ::core::ffi::c_char);
        }
        jw_char(w, '(' as i32 as ::core::ffi::c_char);
        jw_raw(w, alias);
        jw_raw(w, b".structure_id=? AND \0" as *const u8 as *const ::core::ffi::c_char);
        jw_raw(w, alias);
        jw_raw(w, b".environment_id=?)\0" as *const u8 as *const ::core::ffi::c_char);
        i = i.wrapping_add(1);
    }
    jw_char(w, ')' as i32 as ::core::ffi::c_char);
}
unsafe extern "C" fn bind_contexts(
    mut stmt: *mut sqlite3_stmt,
    mut s: *const PhysicsStore,
    mut index: ::core::ffi::c_int,
) -> ::core::ffi::c_int {
    let mut i: size_t = 0 as size_t;
    while i < (*s).context_count {
        let fresh17 = index;
        index = index + 1;
        sqlite3_bind_text(
            stmt,
            fresh17,
            &raw mut (*(*s).contexts.offset(i as isize)).structure_id
                as *mut ::core::ffi::c_char,
            -(1 as ::core::ffi::c_int),
            SQLITE_STATIC,
        );
        let fresh18 = index;
        index = index + 1;
        sqlite3_bind_text(
            stmt,
            fresh18,
            &raw mut (*(*s).contexts.offset(i as isize)).environment_id
                as *mut ::core::ffi::c_char,
            -(1 as ::core::ffi::c_int),
            SQLITE_STATIC,
        );
        i = i.wrapping_add(1);
    }
    return index;
}
unsafe extern "C" fn history_query(
    mut s: *mut PhysicsStore,
    mut rows: *mut *mut HistoryRow,
    mut row_count: *mut size_t,
    mut error: *mut ::core::ffi::c_char,
    mut error_size: size_t,
) -> bool {
    *rows = ::core::ptr::null_mut::<HistoryRow>();
    *row_count = 0 as size_t;
    if s.is_null() || (*s).db.is_null() || !(*s).has_observations_table
        || !(*s).has_quality_table || (*s).context_count == 0 as size_t
    {
        return true_0 != 0;
    }
    let mut sql: JsonWriter = JsonWriter {
        data: ::core::ptr::null_mut::<::core::ffi::c_char>(),
        length: 0,
        capacity: 0,
        failed: false,
    };
    jw_init(&raw mut sql);
    jw_raw(
        &raw mut sql,
        b"SELECT a.id,a.session_id,a.timeline_epoch,a.ut,a.q,a.mach,a.aoa,a.beta,a.roll,a.mass,a.gear,a.brakes,a.airbrakes,a.rcs,a.pos_x,a.pos_y,a.pos_z,a.vel_x,a.vel_y,a.vel_z,a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z,\0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    if (*s).has_sessions_table as ::core::ffi::c_int != 0
        && (*s).has_flight_key as ::core::ffi::c_int != 0
    {
        jw_raw(
            &raw mut sql,
            b"session.flight_key \0" as *const u8 as *const ::core::ffi::c_char,
        );
    } else {
        jw_raw(&raw mut sql, b"NULL \0" as *const u8 as *const ::core::ffi::c_char);
    }
    jw_raw(
        &raw mut sql,
        b"FROM aero_observations a JOIN aero_observation_quality q ON q.observation_id=a.id \0"
            as *const u8 as *const ::core::ffi::c_char,
    );
    if (*s).has_sessions_table as ::core::ffi::c_int != 0
        && (*s).has_flight_key as ::core::ffi::c_int != 0
    {
        jw_raw(
            &raw mut sql,
            b"LEFT JOIN physics_sessions session ON session.session_id=a.session_id \0"
                as *const u8 as *const ::core::ffi::c_char,
        );
    }
    jw_raw(&raw mut sql, b"WHERE \0" as *const u8 as *const ::core::ffi::c_char);
    append_context_where(
        &raw mut sql,
        s,
        b"a\0" as *const u8 as *const ::core::ffi::c_char,
    );
    jw_raw(
        &raw mut sql,
        b" AND a.session_id<>?\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if (*s).has_sessions_table as ::core::ffi::c_int != 0
        && (*s).has_flight_key as ::core::ffi::c_int != 0
    {
        jw_raw(
            &raw mut sql,
            b" AND (session.flight_key IS NULL OR session.flight_key<>?)\0" as *const u8
                as *const ::core::ffi::c_char,
        );
    }
    jw_raw(
        &raw mut sql,
        b" ORDER BY a.id DESC LIMIT 4096\0" as *const u8 as *const ::core::ffi::c_char,
    );
    if sql.failed {
        jw_free(&raw mut sql);
        set_error(
            error,
            error_size,
            b"Out of memory building history query\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    let mut stmt: *mut sqlite3_stmt = ::core::ptr::null_mut::<sqlite3_stmt>();
    if sqlite3_prepare_v2(
        (*s).db,
        sql.data,
        -(1 as ::core::ffi::c_int),
        &raw mut stmt,
        ::core::ptr::null_mut::<*const ::core::ffi::c_char>(),
    ) != SQLITE_OK
    {
        set_sqlite_error(
            error,
            error_size,
            (*s).db,
            b"Could not prepare physics history query\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        jw_free(&raw mut sql);
        return false_0 != 0;
    }
    jw_free(&raw mut sql);
    let mut bind: ::core::ffi::c_int = bind_contexts(stmt, s, 1 as ::core::ffi::c_int);
    let fresh14 = bind;
    bind = bind + 1;
    sqlite3_bind_text(
        stmt,
        fresh14,
        &raw mut (*s).session_id as *mut ::core::ffi::c_char,
        -(1 as ::core::ffi::c_int),
        SQLITE_STATIC,
    );
    if (*s).has_sessions_table as ::core::ffi::c_int != 0
        && (*s).has_flight_key as ::core::ffi::c_int != 0
    {
        let fresh15 = bind;
        bind = bind + 1;
        sqlite3_bind_text(
            stmt,
            fresh15,
            &raw mut (*s).flight_key as *mut ::core::ffi::c_char,
            -(1 as ::core::ffi::c_int),
            SQLITE_STATIC,
        );
    }
    let mut capacity: size_t = 0 as size_t;
    let mut items: *mut HistoryRow = ::core::ptr::null_mut::<HistoryRow>();
    let mut rc: ::core::ffi::c_int = 0;
    loop {
        rc = sqlite3_step(stmt);
        if !(rc == SQLITE_ROW) {
            break;
        }
        let mut id: ::core::ffi::c_longlong = sqlite3_column_int64(
            stmt,
            0 as ::core::ffi::c_int,
        ) as ::core::ffi::c_longlong;
        let mut session: *const ::core::ffi::c_char = sqlite3_column_text(
            stmt,
            1 as ::core::ffi::c_int,
        ) as *const ::core::ffi::c_char;
        if session.is_null() {
            continue;
        }
        let mut q: ::core::ffi::c_double = sqlite3_column_double(
            stmt,
            4 as ::core::ffi::c_int,
        );
        let mut mach: ::core::ffi::c_double = sqlite3_column_double(
            stmt,
            5 as ::core::ffi::c_int,
        );
        let mut aoa: ::core::ffi::c_double = sqlite3_column_double(
            stmt,
            6 as ::core::ffi::c_int,
        );
        let mut beta: ::core::ffi::c_double = sqlite3_column_double(
            stmt,
            7 as ::core::ffi::c_int,
        );
        let mut roll: ::core::ffi::c_double = sqlite3_column_double(
            stmt,
            8 as ::core::ffi::c_int,
        );
        let mut mass: ::core::ffi::c_double = sqlite3_column_double(
            stmt,
            9 as ::core::ffi::c_int,
        );
        let mut airbrakes: ::core::ffi::c_int = sqlite3_column_int(
            stmt,
            12 as ::core::ffi::c_int,
        );
        if airbrakes != 0 as ::core::ffi::c_int && airbrakes != 1 as ::core::ffi::c_int
            || q.is_finite() as i32 == 0 || q <= 0.0f64 || mach.is_finite() as i32 == 0
            || aoa.is_finite() as i32 == 0 || beta.is_finite() as i32 == 0
            || roll.is_finite() as i32 == 0 || mass.is_finite() as i32 == 0
            || mass <= 0.0f64
        {
            continue;
        }
        let mut vectors: [Vector3; 4] = [Vector3 { x: 0., y: 0., z: 0. }; 4];
        let mut valid: bool = true_0 != 0;
        let mut vector_index: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
        while vector_index < 4 as ::core::ffi::c_int {
            let mut base: ::core::ffi::c_int = 14 as ::core::ffi::c_int
                + vector_index * 3 as ::core::ffi::c_int;
            vectors[vector_index as usize] = v3(
                sqlite3_column_double(stmt, base),
                sqlite3_column_double(stmt, base + 1 as ::core::ffi::c_int),
                sqlite3_column_double(stmt, base + 2 as ::core::ffi::c_int),
            );
            if !finite_vector(vectors[vector_index as usize]) {
                valid = false_0 != 0;
            }
            vector_index += 1;
        }
        if !valid {
            continue;
        }
        if *row_count == capacity {
            let mut next: size_t = if capacity != 0 {
                capacity.wrapping_mul(2 as size_t)
            } else {
                128 as size_t
            };
            let mut grown: *mut HistoryRow = realloc(
                items as *mut ::core::ffi::c_void,
                next.wrapping_mul(::core::mem::size_of::<HistoryRow>() as size_t),
            ) as *mut HistoryRow;
            if grown.is_null() {
                free(items as *mut ::core::ffi::c_void);
                sqlite3_finalize(stmt);
                set_error(
                    error,
                    error_size,
                    b"Out of memory reading physics history\0" as *const u8
                        as *const ::core::ffi::c_char,
                );
                return false_0 != 0;
            }
            items = grown;
            capacity = next;
        }
        let fresh16 = *row_count;
        *row_count = (*row_count).wrapping_add(1);
        let mut row: *mut HistoryRow = items.offset(fresh16 as isize) as *mut HistoryRow;
        memset(
            row as *mut ::core::ffi::c_void,
            0 as ::core::ffi::c_int,
            ::core::mem::size_of::<HistoryRow>() as size_t,
        );
        (*row).id = id;
        copy_string(
            &raw mut (*row).session_id as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            session,
        );
        (*row).timeline_epoch = sqlite3_column_int(stmt, 2 as ::core::ffi::c_int);
        let mut flight: *const ::core::ffi::c_char = sqlite3_column_text(
            stmt,
            26 as ::core::ffi::c_int,
        ) as *const ::core::ffi::c_char;
        copy_string(
            &raw mut (*row).source as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 128]>() as size_t,
            if !flight.is_null() && *flight as ::core::ffi::c_int != 0 {
                flight
            } else {
                session
            },
        );
        (*row).ut = sqlite3_column_double(stmt, 3 as ::core::ffi::c_int);
        (*row).q = q;
        (*row).mach = mach;
        (*row).aoa = aoa;
        (*row).beta = beta;
        (*row).roll = roll;
        (*row).mass = mass;
        (*row).gear = sqlite3_column_int(stmt, 10 as ::core::ffi::c_int);
        (*row).brakes = sqlite3_column_int(stmt, 11 as ::core::ffi::c_int);
        (*row).airbrakes = airbrakes;
        (*row).rcs = sqlite3_column_int(stmt, 13 as ::core::ffi::c_int);
        (*row).position = vectors[0 as ::core::ffi::c_int as usize];
        (*row).velocity = vectors[1 as ::core::ffi::c_int as usize];
        (*row).lift = vectors[2 as ::core::ffi::c_int as usize];
        (*row).drag = vectors[3 as ::core::ffi::c_int as usize];
    }
    sqlite3_finalize(stmt);
    if rc != SQLITE_DONE {
        free(items as *mut ::core::ffi::c_void);
        *rows = ::core::ptr::null_mut::<HistoryRow>();
        *row_count = 0 as size_t;
        set_sqlite_error(
            error,
            error_size,
            (*s).db,
            b"Could not read physics history\0" as *const u8
                as *const ::core::ffi::c_char,
        );
        return false_0 != 0;
    }
    *rows = items;
    return true_0 != 0;
}
unsafe extern "C" fn compare_double(
    mut a: *const ::core::ffi::c_void,
    mut b: *const ::core::ffi::c_void,
) -> ::core::ffi::c_int {
    let mut x: ::core::ffi::c_double = *(a as *const ::core::ffi::c_double);
    let mut y: ::core::ffi::c_double = *(b as *const ::core::ffi::c_double);
    return if x < y {
        -(1 as ::core::ffi::c_int)
    } else if x > y {
        1 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
}
unsafe extern "C" fn normalized_force_magnitudes(
    mut row: *const HistoryRow,
    mut lift: *mut ::core::ffi::c_double,
    mut drag: *mut ::core::ffi::c_double,
) {
    *lift = vmag((*row).lift) / (*row).q;
    *drag = vmag((*row).drag) / (*row).q;
}
unsafe extern "C" fn numerical_tie(
    mut a: ::core::ffi::c_double,
    mut b: ::core::ffi::c_double,
) -> bool {
    let mut scale: ::core::ffi::c_double = fmax(1.0f64, fmax(fabs(a), fabs(b)));
    return fabs(a - b) <= DBL_EPSILON * scale;
}
unsafe extern "C" fn choose_medoid(
    mut rows: *const HistoryRow,
    mut indices: *const size_t,
    mut count: size_t,
    mut prefer_newer: bool,
) -> size_t {
    if count == 0 as size_t {
        return 0 as size_t;
    }
    let mut lift: *mut ::core::ffi::c_double = malloc(
        count.wrapping_mul(::core::mem::size_of::<::core::ffi::c_double>() as size_t),
    ) as *mut ::core::ffi::c_double;
    let mut drag: *mut ::core::ffi::c_double = malloc(
        count.wrapping_mul(::core::mem::size_of::<::core::ffi::c_double>() as size_t),
    ) as *mut ::core::ffi::c_double;
    if lift.is_null() || drag.is_null() {
        free(lift as *mut ::core::ffi::c_void);
        free(drag as *mut ::core::ffi::c_void);
        return *indices.offset(0 as ::core::ffi::c_int as isize);
    }
    let mut i: size_t = 0 as size_t;
    while i < count {
        normalized_force_magnitudes(
            rows.offset(*indices.offset(i as isize) as isize) as *const HistoryRow,
            lift.offset(i as isize) as *mut ::core::ffi::c_double,
            drag.offset(i as isize) as *mut ::core::ffi::c_double,
        );
        i = i.wrapping_add(1);
    }
    qsort(
        lift as *mut ::core::ffi::c_void,
        count,
        ::core::mem::size_of::<::core::ffi::c_double>() as size_t,
        Some(
            compare_double
                as unsafe extern "C" fn(
                    *const ::core::ffi::c_void,
                    *const ::core::ffi::c_void,
                ) -> ::core::ffi::c_int,
        ),
    );
    qsort(
        drag as *mut ::core::ffi::c_void,
        count,
        ::core::mem::size_of::<::core::ffi::c_double>() as size_t,
        Some(
            compare_double
                as unsafe extern "C" fn(
                    *const ::core::ffi::c_void,
                    *const ::core::ffi::c_void,
                ) -> ::core::ffi::c_int,
        ),
    );
    let mut middle: size_t = count.wrapping_div(2 as size_t);
    let mut median_lift: ::core::ffi::c_double = if count.wrapping_rem(2 as size_t) != 0
    {
        *lift.offset(middle as isize)
    } else {
        0.5f64
            * (*lift.offset(middle.wrapping_sub(1 as size_t) as isize)
                + *lift.offset(middle as isize))
    };
    let mut median_drag: ::core::ffi::c_double = if count.wrapping_rem(2 as size_t) != 0
    {
        *drag.offset(middle as isize)
    } else {
        0.5f64
            * (*drag.offset(middle.wrapping_sub(1 as size_t) as isize)
                + *drag.offset(middle as isize))
    };
    let mut best: size_t = *indices.offset(0 as ::core::ffi::c_int as isize);
    let mut best_score: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut i_0: size_t = 0 as size_t;
    while i_0 < count {
        let mut index: size_t = *indices.offset(i_0 as isize);
        let mut row_lift: ::core::ffi::c_double = 0.;
        let mut row_drag: ::core::ffi::c_double = 0.;
        normalized_force_magnitudes(
            rows.offset(index as isize) as *const HistoryRow,
            &raw mut row_lift,
            &raw mut row_drag,
        );
        let mut score: ::core::ffi::c_double = hypot(
            row_lift - median_lift,
            row_drag - median_drag,
        );
        let mut tie: bool = numerical_tie(score, best_score);
        if score < best_score
            || tie as ::core::ffi::c_int != 0
                && (if prefer_newer as ::core::ffi::c_int != 0 {
                    ((*rows.offset(index as isize)).id
                        > (*rows.offset(best as isize)).id) as ::core::ffi::c_int
                } else {
                    ((*rows.offset(index as isize)).id
                        < (*rows.offset(best as isize)).id) as ::core::ffi::c_int
                }) != 0
        {
            best = index;
            best_score = score;
        }
        i_0 = i_0.wrapping_add(1);
    }
    free(lift as *mut ::core::ffi::c_void);
    free(drag as *mut ::core::ffi::c_void);
    return best;
}
pub const DBL_EPSILON: ::core::ffi::c_double = __DBL_EPSILON__;
pub const __DBL_EPSILON__: ::core::ffi::c_double = 2.2204460492503131e-16f64;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
#[repr(C)]
pub struct sqlite3 {
    _private: [u8; 0],
}
#[repr(C)]
pub struct sqlite3_stmt {
    _private: [u8; 0],
}
