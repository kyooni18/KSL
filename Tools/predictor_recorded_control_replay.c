#include "landing.h"
#include "physics_store.h"

#include <math.h>
#include <sqlite3.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REPLAY_MAX_ROWS 20000

typedef struct {
    double ut,q,mach,aoa,beta,roll,mass;
    bool gear,brakes; int airbrakes;
    Vector3 position,velocity,lift,drag;
} ReplayRow;

typedef struct { double *values; size_t count,capacity; } Samples;

static Vector3 raw_krpc_to_canonical(Vector3 v){return v3(v.x,v.z,v.y);}
static bool finite_vector(Vector3 v){return isfinite(v.x)&&isfinite(v.y)&&isfinite(v.z);}
static void samples_add(Samples*s,double value){
    if(!s||!isfinite(value))return;
    if(s->count==s->capacity){size_t n=s->capacity?s->capacity*2:256;double*p=realloc(s->values,n*sizeof(*p));if(!p)return;s->values=p;s->capacity=n;}
    s->values[s->count++]=value;
}
static int compare_double(const void*a,const void*b){double x=*(const double*)a,y=*(const double*)b;return (x>y)-(x<y);}
static double percentile(Samples*s,double q){
    if(!s||s->count==0)return NAN;qsort(s->values,s->count,sizeof(*s->values),compare_double);
    double pos=clampd(q,0,1)*(s->count-1),lo=floor(pos),hi=ceil(pos),f=pos-lo;
    return s->values[(size_t)lo]+(s->values[(size_t)hi]-s->values[(size_t)lo])*f;
}
static double mean(const Samples*s){if(!s||!s->count)return NAN;double sum=0;for(size_t i=0;i<s->count;i++)sum+=s->values[i];return sum/s->count;}
static double maximum(const Samples*s){if(!s||!s->count)return NAN;double m=-INFINITY;for(size_t i=0;i<s->count;i++)m=fmax(m,s->values[i]);return m;}
static void print_number(double v){if(isfinite(v))printf("%.12g",v);else printf("null");}
static void print_stats(const char*name,Samples*s,bool comma){
    if(comma)printf(",");printf("\"%s\":{\"count\":%zu,\"mean\":",name,s->count);print_number(mean(s));
    printf(",\"p50\":");print_number(percentile(s,.50));printf(",\"p95\":");print_number(percentile(s,.95));printf(",\"max\":");print_number(maximum(s));printf("}");
}
static char*dup_column(sqlite3_stmt*stmt,int column){const unsigned char*s=sqlite3_column_text(stmt,column);return s?strdup((const char*)s):NULL;}
static bool load_context(sqlite3*db,const char*session,char**flight_key,char**structure,char**environment){
    const char*sql="SELECT s.flight_key,c.structure_json,c.environment_json FROM physics_sessions s JOIN aero_observations a ON a.session_id=s.session_id JOIN physics_contexts c ON c.structure_id=a.structure_id AND c.environment_id=a.environment_id WHERE s.session_id=? ORDER BY a.id DESC LIMIT 1";
    sqlite3_stmt*stmt=NULL;if(sqlite3_prepare_v2(db,sql,-1,&stmt,NULL)!=SQLITE_OK)return false;
    sqlite3_bind_text(stmt,1,session,-1,SQLITE_STATIC);bool ok=sqlite3_step(stmt)==SQLITE_ROW;
    if(ok){*flight_key=dup_column(stmt,0);*structure=dup_column(stmt,1);*environment=dup_column(stmt,2);ok=*flight_key&&*structure&&*environment;}
    sqlite3_finalize(stmt);return ok;
}
static ReplayRow row_from_stmt(sqlite3_stmt*s){
    ReplayRow r={0};int c=0;
    r.ut=sqlite3_column_double(s,c++);r.q=sqlite3_column_double(s,c++);r.mach=sqlite3_column_double(s,c++);r.aoa=sqlite3_column_double(s,c++);r.beta=sqlite3_column_double(s,c++);r.roll=sqlite3_column_double(s,c++);r.mass=sqlite3_column_double(s,c++);
    r.gear=sqlite3_column_int(s,c++)!=0;r.brakes=sqlite3_column_int(s,c++)!=0;r.airbrakes=sqlite3_column_int(s,c++);
    double px=sqlite3_column_double(s,c++),py=sqlite3_column_double(s,c++),pz=sqlite3_column_double(s,c++);r.position=v3(px,py,pz);
    double vx=sqlite3_column_double(s,c++),vy=sqlite3_column_double(s,c++),vz=sqlite3_column_double(s,c++);r.velocity=v3(vx,vy,vz);
    double lx=sqlite3_column_double(s,c++),ly=sqlite3_column_double(s,c++),lz=sqlite3_column_double(s,c++);r.lift=v3(lx,ly,lz);
    double dx=sqlite3_column_double(s,c++),dy=sqlite3_column_double(s,c++),dz=sqlite3_column_double(s,c++);r.drag=v3(dx,dy,dz);
    r.position=raw_krpc_to_canonical(r.position);r.velocity=raw_krpc_to_canonical(r.velocity);r.lift=raw_krpc_to_canonical(r.lift);r.drag=raw_krpc_to_canonical(r.drag);return r;
}
static size_t load_rows(sqlite3*db,const char*session,ReplayRow*rows,size_t maximum){
    const char*sql="SELECT a.ut,a.q,a.mach,a.aoa,a.beta,a.roll,a.mass,a.gear,a.brakes,a.airbrakes,a.pos_x,a.pos_y,a.pos_z,a.vel_x,a.vel_y,a.vel_z,a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z FROM aero_observations a LEFT JOIN aero_observation_quality q ON q.observation_id=a.id WHERE a.session_id=? AND a.q>1.0 AND COALESCE(q.eligible,1)=1 ORDER BY a.id";
    sqlite3_stmt*stmt=NULL;if(sqlite3_prepare_v2(db,sql,-1,&stmt,NULL)!=SQLITE_OK)return 0;sqlite3_bind_text(stmt,1,session,-1,SQLITE_STATIC);
    size_t n=0;while(n<maximum&&sqlite3_step(stmt)==SQLITE_ROW)rows[n++]=row_from_stmt(stmt);sqlite3_finalize(stmt);return n;
}
static bool actual_specific(const ReplayRow*r,const PlanetModel*p,Vector3*out,Vector3*out_air){
    if(!r||!p||!out||r->q<=1||r->mass<=0||!finite_vector(r->position)||!finite_vector(r->velocity))return false;
    Vector3 omega=vscale(p->north_axis,p->rotational_speed),air=vsub(r->velocity,vcross(omega,r->position));if(vmag(air)<=1)return false;
    Vector3 dir=vnorm(air,v3(1,0,0)),up=vnorm(vproject_plane(r->position,dir),v3(0,0,1)),side=vnorm(vcross(dir,up),v3(0,1,0));double b=r->roll*DEG2RAD;
    Vector3 lift_axis=vadd(vscale(up,cos(b)),vscale(side,sin(b))),lateral=vsub(vscale(side,cos(b)),vscale(up,sin(b))),force=vadd(r->lift,r->drag);
    *out=vscale(v3(-vdot(force,dir),vdot(force,lift_axis),vdot(force,lateral)),1.0/r->mass);if(out_air)*out_air=air;return finite_vector(*out)&&out->x>=0;
}

int main(int argc,char**argv){
    if(argc!=3){fprintf(stderr,"usage: %s <observations.sqlite3> <target-physics-session>\n",argv[0]);return 2;}
    const char*path=argv[1],*session=argv[2];sqlite3*db=NULL;
    if(sqlite3_open_v2(path,&db,SQLITE_OPEN_READONLY,NULL)!=SQLITE_OK){fprintf(stderr,"cannot open %s\n",path);return 2;}
    char *flight_key=NULL,*structure=NULL,*environment=NULL;
    if(!load_context(db,session,&flight_key,&structure,&environment)){fprintf(stderr,"target physics session not found: %s\n",session);sqlite3_close(db);return 2;}
    PlanetModel planet={0};snprintf(planet.name,sizeof(planet.name),"Kerbin");planet.radius=600000;planet.gravitational_parameter=3.5316e12;planet.rotational_speed=2*LANDER_PI/21549.425;planet.north_axis=v3(0,0,1);planet.prime_meridian_at_epoch=v3(1,0,0);
    char error[512]={0};PhysicsStoreOptions options={.path=path,.model_id="STS-N",.vessel_name="STS-N",.structure_manifest_json=structure,.environment_manifest_json=environment,.session_id="predictor-recorded-control-replay",.flight_key=flight_key,.mode=PHYSICS_STORE_READ_ONLY};
    PhysicsStore*store=physics_store_open(&options,error,sizeof(error));if(!store){fprintf(stderr,"physics store: %s\n",error);sqlite3_close(db);free(flight_key);free(structure);free(environment);return 2;}
    VesselAeroSample history[VESSEL_AERO_SAMPLES];size_t history_count=physics_store_load_history(store,&planet,history,VESSEL_AERO_SAMPLES,error,sizeof(error));
    if(error[0]){fprintf(stderr,"history: %s\n",error);physics_store_close(store);sqlite3_close(db);free(flight_key);free(structure);free(environment);return 2;}
    VesselPhysicsModel model;vessel_physics_init(&model);for(size_t i=0;i<history_count;i++)vessel_physics_import_sample(&model,&history[i]);
    ReplayRow*rows=calloc(REPLAY_MAX_ROWS,sizeof(*rows));if(!rows)return 2;size_t row_count=load_rows(db,session,rows,REPLAY_MAX_ROWS);
    Samples drag={0},lift={0},lateral={0},combined={0},confidence={0},step_pos={0},step_vel={0},step_dt={0};size_t covered=0,one_step=0;
    for(size_t i=0;i<row_count;i++){
        ReplayRow*r=&rows[i];Vector3 actual,air;if(!actual_specific(r,&planet,&actual,&air))continue;
        Vector3 predicted;double conf=0;if(!vessel_physics_aero_config(&model,r->q,r->mach,r->aoa,r->beta,r->mass,r->gear,r->brakes,r->airbrakes,&predicted,&conf))continue;
        covered++;double scale=fmax(vmag(actual),.05);samples_add(&drag,fabs(predicted.x-actual.x)/fmax(fabs(actual.x),.02));samples_add(&lift,fabs(predicted.y-actual.y)/fmax(fabs(actual.y),.02));samples_add(&lateral,fabs(predicted.z-actual.z)/scale);samples_add(&combined,vmag(vsub(predicted,actual))/scale);samples_add(&confidence,conf);
        if(i+1<row_count){ReplayRow*n=&rows[i+1];double dt=n->ut-r->ut;if(dt>=.02&&dt<=1.5){
            Vector3 accel=vessel_physics_acceleration(r->position,air,&planet,predicted,r->roll),pred_v=vadd(r->velocity,vscale(accel,dt)),pred_p=vadd(r->position,vadd(vscale(r->velocity,dt),vscale(accel,.5*dt*dt)));
            samples_add(&step_vel,vmag(vsub(pred_v,n->velocity)));samples_add(&step_pos,vmag(vsub(pred_p,n->position)));samples_add(&step_dt,dt);one_step++;
        }}
    }
    printf("{\"schemaVersion\":1,\"mode\":\"recorded-control-open-loop-physics\",\"database\":\"%s\",\"targetSession\":\"%s\",\"targetFlightKey\":\"%s\",",path,session,flight_key);
    printf("\"priorPolicy\":\"production physics_store_load_history; entire target flight_key excluded\",\"certifiedPriorCells\":%zu,\"eligibleRecordedStates\":%zu,\"coveredStates\":%zu,\"coverage\":",history_count,row_count,covered);print_number(row_count?(double)covered/row_count:0);
    printf(",\"forceResidual\":{");print_stats("combinedRelative",&combined,false);print_stats("dragRelative",&drag,true);print_stats("liftRelative",&lift,true);print_stats("lateralRelativeToTotal",&lateral,true);print_stats("modelConfidence",&confidence,true);printf("}");
    printf(",\"oneStepReplay\":{\"count\":%zu",one_step);print_stats("dtSeconds",&step_dt,true);print_stats("positionErrorM",&step_pos,true);print_stats("velocityErrorMps",&step_vel,true);printf("}");
    printf(",\"interpretation\":\"Recorded q/Mach/AoA/beta/bank/configuration are held to measured values; only certified-prior aerodynamic force and resulting one-step dynamics are replayed. Closed-loop guidance error is intentionally excluded.\"}\n");
    free(drag.values);free(lift.values);free(lateral.values);free(combined.values);free(confidence.values);free(step_pos.values);free(step_vel.values);free(step_dt.values);free(rows);physics_store_close(store);sqlite3_close(db);free(flight_key);free(structure);free(environment);return 0;
}
