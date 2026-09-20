#include "physics_store.h"

#include <sqlite3.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *STRUCTURE_CURRENT =
    "{\"schema\":1,\"partCount\":2,\"dryMassWitness\":100.0,\"geometryComplete\":true,"
    "\"parts\":[{\"name\":\"wing\",\"dryMass\":30.0},{\"name\":\"body\",\"dryMass\":70.0}],"
    "\"geometry\":[{\"name\":\"body\",\"position\":[1.01,2.0,3.0],\"rotation\":[0,0,0,1]},"
    "{\"name\":\"wing\",\"position\":[-1.0,0.0,0.0],\"rotation\":[0,0,0,1]}]}";
static const char *STRUCTURE_FLEX_EQUIVALENT =
    "{\"schema\":1,\"partCount\":2,\"dryMassWitness\":999.0,\"geometryComplete\":true,"
    "\"parts\":[{\"name\":\"body\",\"dryMass\":999.0},{\"name\":\"wing\",\"dryMass\":1.0}],"
    "\"geometry\":[{\"name\":\"wing\",\"position\":[-1.01,0.01,0.0],\"rotation\":[1,0,0,0]},"
    "{\"name\":\"body\",\"position\":[1.00,2.01,3.01],\"rotation\":[1,0,0,0]}]}";
static const char *STRUCTURE_DIFFERENT =
    "{\"schema\":1,\"partCount\":2,\"geometryComplete\":true,"
    "\"parts\":[{\"name\":\"body\"},{\"name\":\"wing\"}],"
    "\"geometry\":[{\"name\":\"body\",\"position\":[1.30,2.0,3.0]},"
    "{\"name\":\"wing\",\"position\":[-1.0,0.0,0.0]}]}";
static const char *ENV_CURRENT =
    "{\"schema\":1,\"krpcVersion\":\"0.6.0\",\"atmosphereCurveDigest\":\"sha256:curve\","
    "\"planet\":{\"name\":\"Kerbin\",\"radius\":600000.0,\"gravitationalParameter\":3531600000000.0,"
    "\"rotationalSpeed\":0.0002915709,\"surfaceDensity\":1.2,\"atmosphereDepth\":70000.0,"
    "\"epochUT\":123.0,\"northAxisKRPC\":[0,600000,0],\"primeMeridianAtEpochKRPC\":[1,2,3]}}";
static const char *ENV_COORD_EQUIVALENT =
    "{\"schema\":1,\"krpcVersion\":\"0.5.4\",\"atmosphereCurveDigest\":\"sha256:curve\","
    "\"planet\":{\"surfaceDensity\":1.20,\"radius\":600000,\"name\":\"Kerbin\",\"atmosphereDepth\":70000,"
    "\"gravitationalParameter\":3531600000000,\"rotationalSpeed\":0.0002915709,"
    "\"epochUT\":999999,\"northAxisKRPC\":[12,34,56],\"primeMeridianAtEpochKRPC\":[9,8,7]}}";
static const char *ENV_DIFFERENT =
    "{\"schema\":1,\"atmosphereCurveDigest\":\"sha256:curve\","
    "\"planet\":{\"name\":\"Kerbin\",\"radius\":600000,\"gravitationalParameter\":3531600000000,"
    "\"rotationalSpeed\":0.0002915709,\"surfaceDensity\":1.3,\"atmosphereDepth\":70000}}";
static const char *ENV_NATIVE_NO_DIGEST =
    "{\"schema\":3,\"atmosphereCurveDigest\":null,"
    "\"planet\":{\"name\":\"Kerbin\",\"radius\":600000,\"gravitationalParameter\":3531600000000,"
    "\"rotationalSpeed\":0.0002915709,\"surfaceDensity\":1.2,\"atmosphereDepth\":70000}}";

static void sql_ok(sqlite3 *db,const char *sql){char *message=NULL;int rc=sqlite3_exec(db,sql,NULL,NULL,&message);if(rc!=SQLITE_OK){fprintf(stderr,"SQL failed: %s\n%s\n",message?message:"unknown",sql);sqlite3_free(message);abort();}}
static long scalar_long(sqlite3 *db,const char *sql){sqlite3_stmt *stmt=NULL;assert(sqlite3_prepare_v2(db,sql,-1,&stmt,NULL)==SQLITE_OK);assert(sqlite3_step(stmt)==SQLITE_ROW);long value=(long)sqlite3_column_int64(stmt,0);sqlite3_finalize(stmt);return value;}
static char *temporary_path(const char *tag){char *path=malloc(256);assert(path);snprintf(path,256,"/tmp/ksp-%s-XXXXXX",tag);int fd=mkstemp(path);assert(fd>=0);close(fd);unlink(path);return path;}

static void insert_context(sqlite3 *db,const char *sid,const char *eid,const char *structure,const char *environment){
    sqlite3_stmt *stmt=NULL;assert(sqlite3_prepare_v2(db,"INSERT INTO physics_contexts VALUES(?,?,?,?,1,1)",-1,&stmt,NULL)==SQLITE_OK);
    sqlite3_bind_text(stmt,1,sid,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,2,eid,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,3,structure,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,4,environment,-1,SQLITE_STATIC);assert(sqlite3_step(stmt)==SQLITE_DONE);sqlite3_finalize(stmt);
}
static void insert_legacy_session(sqlite3 *db,const char *session,const char *sid,const char *eid){
    sqlite3_stmt *stmt=NULL;assert(sqlite3_prepare_v2(db,"INSERT INTO physics_sessions(session_id,structure_id,environment_id,vessel_name,started_wall) VALUES(?,?,?,?,1)",-1,&stmt,NULL)==SQLITE_OK);
    sqlite3_bind_text(stmt,1,session,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,2,sid,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,3,eid,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,4,"STS-N",-1,SQLITE_STATIC);assert(sqlite3_step(stmt)==SQLITE_DONE);sqlite3_finalize(stmt);
}
static void insert_legacy_observation(sqlite3 *db,const char *sid,const char *eid,const char *session,double ut,double q,double aoa,double beta,int airbrakes){
    const char *sql="INSERT INTO aero_observations(structure_id,environment_id,session_id,timeline_epoch,ut,wall_time,q,mach,aoa,beta,roll,mass,gear,brakes,airbrakes,rcs,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,lift_x,lift_y,lift_z,drag_x,drag_y,drag_z) VALUES(?,?,?,0,?,1,?,1.0,?,?,0,30000,0,0,?,0,600000,0,0,0,0,1000,50000,0,0,0,0,-10000)";
    sqlite3_stmt *stmt=NULL;assert(sqlite3_prepare_v2(db,sql,-1,&stmt,NULL)==SQLITE_OK);sqlite3_bind_text(stmt,1,sid,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,2,eid,-1,SQLITE_STATIC);sqlite3_bind_text(stmt,3,session,-1,SQLITE_STATIC);sqlite3_bind_double(stmt,4,ut);sqlite3_bind_double(stmt,5,q);sqlite3_bind_double(stmt,6,aoa);sqlite3_bind_double(stmt,7,beta);sqlite3_bind_int(stmt,8,airbrakes);assert(sqlite3_step(stmt)==SQLITE_DONE);sqlite3_finalize(stmt);
}

static void create_schema2_archive(const char *path){
    sqlite3 *db=NULL;assert(sqlite3_open(path,&db)==SQLITE_OK);
    sql_ok(db,"CREATE TABLE physics_meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);INSERT INTO physics_meta VALUES('schema','2');"
        "CREATE TABLE physics_contexts(structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,structure_json TEXT NOT NULL,environment_json TEXT NOT NULL,first_seen REAL NOT NULL,last_seen REAL NOT NULL,PRIMARY KEY(structure_id,environment_id));"
        "CREATE TABLE physics_sessions(session_id TEXT PRIMARY KEY,structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,vessel_name TEXT NOT NULL,started_wall REAL NOT NULL);"
        "CREATE TABLE aero_observations(id INTEGER PRIMARY KEY AUTOINCREMENT,structure_id TEXT NOT NULL,environment_id TEXT NOT NULL,session_id TEXT NOT NULL,timeline_epoch INTEGER NOT NULL,ut REAL NOT NULL,wall_time REAL NOT NULL,q REAL NOT NULL,mach REAL NOT NULL,aoa REAL NOT NULL,beta REAL NOT NULL,roll REAL NOT NULL,mass REAL NOT NULL,gear INTEGER NOT NULL,brakes INTEGER NOT NULL,airbrakes INTEGER NOT NULL,rcs INTEGER NOT NULL,pos_x REAL NOT NULL,pos_y REAL NOT NULL,pos_z REAL NOT NULL,vel_x REAL NOT NULL,vel_y REAL NOT NULL,vel_z REAL NOT NULL,lift_x REAL NOT NULL,lift_y REAL NOT NULL,lift_z REAL NOT NULL,drag_x REAL NOT NULL,drag_y REAL NOT NULL,drag_z REAL NOT NULL);");
    insert_context(db,"sha256:compatible-legacy","legacy-env",STRUCTURE_FLEX_EQUIVALENT,ENV_COORD_EQUIVALENT);
    insert_context(db,"sha256:wrong-airframe","legacy-env",STRUCTURE_DIFFERENT,ENV_COORD_EQUIVALENT);
    insert_context(db,"sha256:wrong-environment","different-env",STRUCTURE_FLEX_EQUIVALENT,ENV_DIFFERENT);
    insert_legacy_session(db,"legacy-good","sha256:compatible-legacy","legacy-env");
    insert_legacy_session(db,"legacy-wrong-airframe","sha256:wrong-airframe","legacy-env");
    insert_legacy_session(db,"legacy-wrong-env","sha256:wrong-environment","different-env");
    insert_legacy_observation(db,"sha256:compatible-legacy","legacy-env","legacy-good",1,10000,20,1,0);
    insert_legacy_observation(db,"sha256:compatible-legacy","legacy-env","legacy-good",2,10000,20,40,0);
    insert_legacy_observation(db,"sha256:compatible-legacy","legacy-env","legacy-good",3,10000,20,1,0); /* quarantined departure tail */
    insert_legacy_observation(db,"sha256:wrong-airframe","legacy-env","legacy-wrong-airframe",1,10000,20,1,0);
    insert_legacy_observation(db,"sha256:wrong-environment","different-env","legacy-wrong-env",1,10000,20,1,0);
    sqlite3_close(db);
}

static PhysicsStore *open_store(const char *path,PhysicsStoreMode mode,const char *session,const char *flight,char *error){
    PhysicsStoreOptions options={.path=path,.model_id="STS-N",.vessel_name="STS-N",.structure_manifest_json=STRUCTURE_CURRENT,.environment_manifest_json=ENV_CURRENT,.session_id=session,.flight_key=flight,.mode=mode};
    PhysicsStore *store=physics_store_open(&options,error,512);if(!store&&error)fprintf(stderr,"open_store: %s\n",error);return store;
}
static PlanetModel test_planet(void){PlanetModel p={0};snprintf(p.name,sizeof(p.name),"Kerbin");p.radius=600000;p.rotational_speed=.0002915709;p.north_axis=v3(0,0,1);return p;}
static PhysicsStoreObservation base_observation(double ut,double q,int airbrakes){
    PhysicsStoreObservation o={0};o.sample_valid=true;o.body_non_rotating=true;o.situation="flying";o.ut=ut;o.wall_time=ut;o.q=q;o.mach=1;o.aoa=20;o.beta=1;o.roll=0;o.mass=30000;o.gear=false;o.brakes=false;o.airbrakes=airbrakes;o.rcs=0;o.position=v3(600000,0,0);o.velocity=v3(0,0,1000);o.lift=v3(50000,0,0);o.drag=v3(0,0,-10000);o.control_profile="approach";o.has_body_rates=true;o.body_rates=v3(1,1,1);return o;
}

static void test_unprovenanced_legacy_rows_are_not_certified(void){
    char *path=temporary_path("physics-native-digest");create_schema2_archive(path);char error[512]={0};
    PhysicsStoreOptions options={.path=path,.model_id="STS-N",.vessel_name="STS-N",
        .structure_manifest_json=STRUCTURE_CURRENT,.environment_manifest_json=ENV_NATIVE_NO_DIGEST,
        .session_id="native-reader",.flight_key="native-flight",.mode=PHYSICS_STORE_READ_ONLY};
    PhysicsStore *store=physics_store_open(&options,error,sizeof(error));assert(store);
    PlanetModel planet=test_planet();VesselAeroSample samples[VESSEL_AERO_SAMPLES];
    size_t count=physics_store_load_history(store,&planet,samples,VESSEL_AERO_SAMPLES,error,sizeof(error));
    assert(error[0]==0);assert(count==0);
    physics_store_close(store);unlink(path);free(path);
}

static void test_migration_identity_quality_and_same_flight(void){
    char *path=temporary_path("physics-store");create_schema2_archive(path);char error[512]={0};
    PhysicsStore *writer=open_store(path,PHYSICS_STORE_READ_WRITE,"writer-a","launch-A",error);assert(writer);assert(strcmp(physics_store_structure_id(writer),"model:STS-N")==0);assert(strncmp(physics_store_structure_witness_id(writer),"sha256:",7)==0);assert(physics_store_compatible_context_count(writer)==2);
    PlanetModel planet=test_planet();VesselAeroSample samples[VESSEL_AERO_SAMPLES];
    /*
       Schema-2 rows predate the validated-observation provenance contract.
       Migration preserves them as raw evidence but must not certify them.
    */
    size_t count=physics_store_load_history(writer,&planet,samples,VESSEL_AERO_SAMPLES,error,sizeof(error));assert(error[0]==0);assert(count==0);

    PhysicsStoreObservation o=base_observation(100,10000,0);assert(physics_store_observe(writer,&o,error,sizeof(error)));
    o=base_observation(100.6,10000,1);assert(physics_store_observe(writer,&o,error,sizeof(error)));
    o=base_observation(101.2,20000,0);o.control_profile="recovery";assert(physics_store_observe(writer,&o,error,sizeof(error)));
    o=base_observation(101.8,40000,0);o.body_rates=v3(100,0,0);assert(physics_store_observe(writer,&o,error,sizeof(error)));
    o=base_observation(50,80000,0);assert(physics_store_observe(writer,&o,error,sizeof(error)));
    assert(physics_store_flush(writer,error,sizeof(error)));char persisted_environment[PHYSICS_STORE_ID_CAPACITY];snprintf(persisted_environment,sizeof(persisted_environment),"%s",physics_store_environment_id(writer));physics_store_close(writer);

    sqlite3 *db=NULL;assert(sqlite3_open(path,&db)==SQLITE_OK);
    assert(scalar_long(db,"SELECT value FROM physics_meta WHERE key='schema'")==3);
    assert(scalar_long(db,"SELECT COUNT(*) FROM pragma_table_info('physics_sessions') WHERE name='flight_key'")==1);
    assert(scalar_long(db,"SELECT COUNT(*) FROM pragma_table_info('aero_observations') WHERE name='dry_mass'")==1);
    assert(scalar_long(db,"SELECT COUNT(*) FROM aero_observation_quality")==5);
    assert(scalar_long(db,"SELECT SUM(eligible) FROM aero_observation_quality")==5);
    assert(scalar_long(db,"SELECT COUNT(DISTINCT timeline_epoch) FROM aero_observations WHERE session_id='writer-a'")==2);
    assert(scalar_long(db,"SELECT eligible FROM aero_observation_quality WHERE control_profile='recovery' LIMIT 1")==1);
    assert(scalar_long(db,"SELECT eligible FROM aero_observation_quality WHERE ABS(body_pitch_rate)>=100 LIMIT 1")==1);

    /*
       Simulate rows written by the retired attitude-quality policy. Presence
       of modern provenance, not the old score value, determines whether the
       direct force observation may enter the certified sample bank.
    */
    sql_ok(db,"UPDATE aero_observation_quality SET quality=0,eligible=0 WHERE control_profile='recovery' OR ABS(body_pitch_rate)>=100");
    sqlite3_close(db);

    PhysicsStore *same=open_store(path,PHYSICS_STORE_READ_ONLY,"reader-same","launch-A",error);assert(same);assert(strcmp(physics_store_environment_id(same),persisted_environment)==0);
    count=physics_store_load_history(same,&planet,samples,VESSEL_AERO_SAMPLES,error,sizeof(error));assert(error[0]==0);assert(count==0);physics_store_close(same);

    PhysicsStore *other=open_store(path,PHYSICS_STORE_READ_ONLY,"reader-other","launch-B",error);assert(other);
    count=physics_store_load_history(other,&planet,samples,VESSEL_AERO_SAMPLES,error,sizeof(error));assert(error[0]==0);assert(count==5);
    bool saw_airbrake=false;for(size_t i=0;i<count;i++){if(samples[i].airbrakes==1)saw_airbrake=true;assert(samples[i].trust==1.0);assert(samples[i].observations==1);}assert(saw_airbrake);
    physics_store_close(other);
    unlink(path);free(path);
}

static void test_bounded_coverage(void){
    char *path=temporary_path("physics-bounded");char error[512]={0};PhysicsStore *writer=open_store(path,PHYSICS_STORE_READ_WRITE,"bulk-writer","bulk-flight",error);assert(writer);
    for(int i=0;i<240;i++){double q=1000.0*pow(2.0,(double)(i%12));PhysicsStoreObservation o=base_observation(1000.0+.6*i,q,i%2);o.mach=.5*((i/12)%10);o.aoa=10.0+4.0*((i/120)%2);assert(physics_store_observe(writer,&o,error,sizeof(error)));}
    assert(physics_store_flush(writer,error,sizeof(error)));physics_store_close(writer);PhysicsStore *reader=open_store(path,PHYSICS_STORE_READ_ONLY,"bulk-reader","other-flight",error);assert(reader);PlanetModel planet=test_planet();VesselAeroSample samples[16];size_t count=physics_store_load_history(reader,&planet,samples,16,error,sizeof(error));assert(error[0]==0);assert(count==16);physics_store_close(reader);unlink(path);free(path);
}

static void test_partial_corrupt_and_read_only(void){
    char *partial=temporary_path("physics-partial");sqlite3 *db=NULL;assert(sqlite3_open(partial,&db)==SQLITE_OK);sql_ok(db,"CREATE TABLE physics_meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);INSERT INTO physics_meta VALUES('schema','3')");sqlite3_close(db);char error[512]={0};PhysicsStore *store=open_store(partial,PHYSICS_STORE_READ_ONLY,"partial-reader","partial-flight",error);assert(store);PlanetModel planet=test_planet();VesselAeroSample sample;assert(physics_store_load_history(store,&planet,&sample,1,error,sizeof(error))==0);PhysicsStoreObservation o=base_observation(1,1000,0);assert(!physics_store_observe(store,&o,error,sizeof(error)));assert(strstr(error,"read-only"));physics_store_close(store);unlink(partial);free(partial);
    char *corrupt=temporary_path("physics-corrupt");FILE *f=fopen(corrupt,"wb");assert(f);fputs("this is deliberately not a sqlite database",f);fclose(f);error[0]=0;store=open_store(corrupt,PHYSICS_STORE_READ_ONLY,"corrupt-reader","corrupt-flight",error);assert(!store);assert(error[0]);unlink(corrupt);free(corrupt);
}

static void test_real_archive_if_requested(void){
    const char *path=getenv("KSP_REAL_PHYSICS_DB");if(!path||!*path)return;sqlite3 *db=NULL;assert(sqlite3_open_v2(path,&db,SQLITE_OPEN_READONLY,NULL)==SQLITE_OK);sqlite3_stmt *stmt=NULL;assert(sqlite3_prepare_v2(db,"SELECT structure_json,environment_json FROM physics_contexts WHERE structure_id='model:STS-N' ORDER BY last_seen DESC LIMIT 1",-1,&stmt,NULL)==SQLITE_OK);assert(sqlite3_step(stmt)==SQLITE_ROW);const char *s=(const char*)sqlite3_column_text(stmt,0),*e=(const char*)sqlite3_column_text(stmt,1);assert(s&&e);char *structure=strdup(s),*environment=strdup(e);assert(structure&&environment);sqlite3_finalize(stmt);sqlite3_close(db);
    char error[512]={0};PhysicsStoreOptions options={.path=path,.model_id="STS-N",.vessel_name="STS-N",.structure_manifest_json=structure,.environment_manifest_json=environment,.session_id="native-offline-real-probe",.flight_key="native-offline-real-probe",.mode=PHYSICS_STORE_READ_ONLY};PhysicsStore *store=physics_store_open(&options,error,sizeof(error));if(!store)fprintf(stderr,"real archive: %s\n",error);assert(store);assert(physics_store_compatible_context_count(store)>=1);PlanetModel planet=test_planet();VesselAeroSample samples[VESSEL_AERO_SAMPLES];size_t count=physics_store_load_history(store,&planet,samples,VESSEL_AERO_SAMPLES,error,sizeof(error));if(error[0])fprintf(stderr,"real history: %s\n",error);assert(error[0]==0);assert(count>0&&count<=VESSEL_AERO_SAMPLES);printf("Real archive read-only import: %zu bounded certified cells\n",count);physics_store_close(store);free(structure);free(environment);
}

int main(void){test_unprovenanced_legacy_rows_are_not_certified();test_migration_identity_quality_and_same_flight();test_bounded_coverage();test_partial_corrupt_and_read_only();test_real_archive_if_requested();puts("Native physics store tests passed");return 0;}
