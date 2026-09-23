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

