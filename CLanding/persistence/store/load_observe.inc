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
